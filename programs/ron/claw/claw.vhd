-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- CLAW (v0.5.0) -- an arcade claw machine seen through
-- the glass, filling the screen: marquee with chasing bulbs, chrome bezel,
-- gantry rail + carriage, cable and a three-prong chrome claw, a heaped pile
-- of video-toy prizes, an acrylic prize chute, and the cabinet base with
-- its prize-door window.  "Vinyl toy" style: 5-unit dark outlines, flat
-- base + highlight + shadow, rounded corners everywhere.
--
-- v0.1.0 scope (phases 1-2 of DESIGN.md + a minimal phase 3): the static
-- scene, the claw's motion and a grab / carry / win / drop loop.  K1 gantry
-- X (glide), K2 depth row, K3 grip tolerance, K4 glide speed, K5 cabinet
-- theme (6), P12 cable, S7 open/close (any edge: close low over a prize =
-- grab, open over the chute = WIN), S10 video on the back wall.  Sequences,
-- tumble physics, sway, attract and rigging come in later phases.
--
-- Rendering (no frame buffer; everything per pixel, in raster order):
--   * Coordinates are virtual 1080-line units (1920 x 1080 at 1080p/720p,
--     a 1440 x 1080 4:3 window at SD), stepped from the timing ID.
--   * Procedural cabinet: 16 rounded-rect elements share 9 per-LINE slots.
--     A per-line sequencer (hblank) evaluates each element's corner insets
--     from one EBR table and writes outer/inner/highlight x bounds into the
--     slot registers; the pixel path is then compares only.
--   * Prize pile: a dual-bank line buffer (2 banks x even/odd halves,
--     512 x 8 each, one entry = 2 virtual units = half a sprite pixel).  At
--     every active-line start a stamper walks the slot table (back row
--     first) and streams each intersecting prize row out of the 4bpp sprite
--     ROM, one sprite pixel (= one entry pair) per clock, into the bank the
--     NEXT line will display.  Entries hold the final palette address
--     (depth-row haze included); 0 = empty.  The displayed bank is cleared
--     one pair behind the beam.
--   * Everything composites in palette-index space; one 256-entry palette
--     EBR gives YUV.  Glass streaks / acrylic lift luma afterwards.
--   * Output conditioning against the HD HDMI thick bars ([1 2 1] on Y/U/V,
--     luma scaled toward black), then the blanking gate.
--   * Generated avid (parity-locked start, fixed width) -- the program
--     draws its own picture, so the encoders' Cb/Cr pairing stays coherent.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.claw_pkg.all;

architecture claw of program_top is

    constant C_LAT : natural := 20;          -- = processing_delay_clks
    -- P0 -> output register depth.  KEEP IT AT 16 (with C_LAT = 20): the
    -- encoders pair Cb/Cr by counting pixels from hsync, so changing this by
    -- one clock moves the generated avid one pixel against hsync and swaps
    -- U/V on hardware (v0.1/v0.2/v0.4.0 at 15 showed every colour swapped;
    -- v0.3.1/v0.3.2 at 16 were correct).  Add or remove pixel stages in pairs,
    -- or pad with a plain delay stage, never by one.
    constant C_PIX : natural := 16;
    -- the prize hole: hardwired element 16 (x from the right edge), slot 3;
    -- its slot code reuses E_MARQ_OUT's 0, which never lands in slot 3
    constant C_EL_HOLE_X0 : integer := -350;
    constant C_EL_HOLE_X1 : integer := -50;
    constant C_EL_HOLE_Y0 : integer := 972;
    constant C_EL_HOLE_Y1 : integer := 1060;
    constant C_EL_HOLE_RC : integer := 1;
    constant C_EL_HOLE_SLOT : integer := 3;
    constant E_HOLE : natural := 0;
    constant C_HW_UV_SWAP : boolean := true;  -- correct together with C_PIX = 16 (HW-confirmed v0.3.1/v0.3.2)

    subtype t_c is signed(12 downto 0);      -- virtual coordinate

    ---------------------------------------------------------------------------
    -- Timing class (from the timing ID)
    ---------------------------------------------------------------------------
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
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
    function f_width_m1(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(719, 12);
            when 5 | 6 | 14     => return to_unsigned(1279, 12);
            when others         => return to_unsigned(1919, 12);
        end case;
    end function;

    -- bulb shape on (|dx|, |dy|) <= 7: 0 none, 1 socket ring (d^2 < 49), 2 lamp (< 26)
    type t_bulb is array (0 to 63) of std_logic_vector(1 downto 0);
    constant C_BULB : t_bulb := (
        "10", "10", "10", "10", "10", "10", "01", "00", "10", "10", "10", "10", "10", "01", "01", "00",
        "10", "10", "10", "10", "10", "01", "01", "00", "10", "10", "10", "10", "10", "01", "01", "00",
        "10", "10", "10", "10", "01", "01", "00", "00", "10", "01", "01", "01", "01", "00", "00", "00",
        "01", "01", "01", "01", "00", "00", "00", "00", "00", "00", "00", "00", "00", "00", "00", "00");
    type t_bayer is array (0 to 15) of unsigned(3 downto 0);
    constant C_BAYER : t_bayer := (
        x"0", x"8", x"2", x"A", x"C", x"4", x"E", x"6",
        x"3", x"B", x"1", x"9", x"F", x"7", x"D", x"5");

    ---------------------------------------------------------------------------
    -- Generated avid (cubist / aurora recipe)
    ---------------------------------------------------------------------------
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1919, 12);

    ---------------------------------------------------------------------------
    -- Coordinates, line / field events
    ---------------------------------------------------------------------------
    signal s_xs     : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys     : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_sd     : std_logic := '0';
    signal s_vw     : t_c := to_signed(1920, 13);
    signal s_vw2    : t_c := to_signed(960, 13);
    signal s_xa     : unsigned(12 downto 0) := (others => '0');
    signal s_bcol   : unsigned(1 downto 0) := (others => '0');
    signal s_lc     : unsigned(1 downto 0) := (others => '0');
    signal s_ya     : unsigned(19 downto 0) := (others => '0');
    signal s_yn     : unsigned(19 downto 0) := (others => '0');
    signal s_gav_d  : std_logic := '0';
    signal s_first  : std_logic;             -- first pixel of a line (comb.)
    signal s_gav_r  : std_logic := '0';
    signal s_gav_f  : std_logic := '0';
    signal s_vsq    : std_logic := '1';
    signal s_saw    : std_logic := '0';
    signal s_vs_go  : std_logic := '0';
    signal s_vy     : t_c := (others => '0');

    ---------------------------------------------------------------------------
    -- Frame control (vblank)
    ---------------------------------------------------------------------------
    signal f_st     : unsigned(4 downto 0) := (others => '0');
    signal f_k1, f_k2, f_k4, f_k5, f_p12 : unsigned(9 downto 0) := (others => '0');
    signal f_sw7q   : std_logic := '0';
    signal m_a      : unsigned(9 downto 0) := (others => '0');
    signal m_b      : unsigned(20 downto 0) := (others => '0');
    signal m_acc    : unsigned(20 downto 0) := (others => '0');
    signal m_cnt    : unsigned(3 downto 0) := (others => '0');
    signal f_xt, f_yt : signed(15 downto 0) := (others => '0');   -- Q.4 targets
    signal f_xq     : signed(15 downto 0) := to_signed(640 * 16, 16);
    signal f_yq     : signed(15 downto 0) := to_signed(C_CLAW_Y_MIN * 16, 16);
    signal f_dx, f_dy : signed(15 downto 0) := (others => '0');
    signal f_g      : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal f_s, f_s2 : signed(15 downto 0) := (others => '0');
    signal f_sxl    : signed(15 downto 0) := (others => '0');
    -- win flash + autopilot
    signal s_wint   : unsigned(7 downto 0) := (others => '0');
    signal s_winfx  : std_logic := '0';
    signal f_wtq, gm_wtog : std_logic := '0';
    signal f_auto   : std_logic := '0';
    signal f_grabm  : std_logic := '0';      -- S9: 0 = Auto-close, 1 = Manual
    signal f_rig    : std_logic := '0';      -- S11: Rigged
    signal f_k6     : unsigned(9 downto 0) := (others => '0');   -- K6: rig odds
    signal s_aclk   : std_logic := '0';      -- auto-close fired / manual close: no auto-open until the claw rises
    type t_ap is (AP_PICK, AP_MOVE, AP_LOWER, AP_CLOSE, AP_LIFT, AP_CHUTE, AP_WAIT);
    signal ap_st    : t_ap := AP_PICK;
    signal ap_tmr   : unsigned(6 downto 0) := (others => '0');
    signal ap_x, ap_yt, ap_ylow, ap_ylow_n : t_c := to_signed(C_CLAW_Y_MIN, 13);
    signal ap_d     : unsigned(1 downto 0) := (others => '0');
    signal ap_roff  : t_c := (others => '0');
    signal ap_cx    : t_c := to_signed(1720, 13);
    signal ap_arr, ap_sent : std_logic := '0';
    signal f_areq, ap_tq, gm_aq, gm_tok : std_logic := '0';
    signal gm_tx, gm_ty : t_c := to_signed(960, 13);
    signal gm_td    : unsigned(1 downto 0) := (others => '0');
    -- joystick
    signal s_jo     : signed(4 downto 0) := (others => '0');      -- lean -8..8
    signal s_jpt    : unsigned(4 downto 0) := (others => '0');    -- rock timer
    signal s_jpd    : std_logic := '0';                           -- rock back
    signal s_jx     : t_c := to_signed(560, 13);
    signal s_jy     : t_c := to_signed(C_JOY_UP_Y, 13);
    signal s_jr     : t_c := to_signed(C_JOY_UP_R, 13);
    signal s_jrc    : unsigned(2 downto 0) := to_unsigned(C_JOY_UP_RC, 3);
    signal f_jp     : t_c := (others => '0');
    signal f_jb     : t_c := (others => '0');
    signal f_jm     : unsigned(10 downto 0) := (others => '0');
    signal s_cx     : t_c := to_signed(640, 13);
    signal s_cy     : t_c := to_signed(C_CLAW_Y_MIN, 13);
    signal s_open   : signed(4 downto 0) := to_signed(C_OPEN_MIN, 5);
    signal s_open_t : std_logic := '0';
    signal s_cdepth : unsigned(1 downto 0) := (others => '0');
    signal s_thhi   : unsigned(3 downto 0) := to_unsigned(4, 4);   -- theme + 4 (themes at 64)
    signal s_phase  : unsigned(7 downto 0) := (others => '0');
    signal s_vid_on : std_logic := '0';

    ---------------------------------------------------------------------------
    -- Per-line slot sequencer
    ---------------------------------------------------------------------------
    signal q_run    : std_logic := '0';
    signal q_e      : unsigned(4 downto 0) := (others => '0');
    signal q_go     : std_logic := '0';
    signal q_go2    : std_logic := '0';
    type t_qpipe_e is array (0 to 6) of unsigned(4 downto 0);   -- bit 4: the hole
    signal q_pe     : t_qpipe_e := (others => (others => '0'));
    signal q_pv     : std_logic_vector(6 downto 0) := (others => '0');
    signal q1_x0, q1_x1, q1_y0, q1_y1 : t_c := (others => '0');
    signal q2_x0, q2_x1, q2_kt, q2_kb : t_c := (others => '0');
    signal q2_in    : std_logic := '0';
    signal q3_x0, q3_x1, q3_kt, q3_kb : t_c := (others => '0');
    signal q3_in    : std_logic := '0';
    signal q3_prod  : unsigned(11 downto 0) := (others => '0');
    signal q3_neg   : std_logic := '0';
    signal q3_addr  : unsigned(7 downto 0) := (others => '0');
    signal q4_x0, q4_x1, q4_kt, q4_kb : t_c := (others => '0');
    signal q4_in    : std_logic := '0';
    signal q4_ins   : std_logic_vector(15 downto 0) := (others => '0');
    signal q5_x0, q5_x1, q5_kt, q5_kb : t_c := (others => '0');
    signal q5_in    : std_logic := '0';
    signal q5_ins   : std_logic_vector(15 downto 0) := (others => '0');
    signal q6_lo, q6_ro, q6_li, q6_ri, q6_hx : unsigned(10 downto 0) := (others => '0');
    signal q6_vi, q6_hl, q6_sh : std_logic := '0';
    signal q6_we    : std_logic_vector(8 downto 0) := (others => '0');
    signal q6_el    : unsigned(3 downto 0) := (others => '0');
    signal q6_fl, q6_fo : std_logic := '0';
    signal sl_fl, sl_fo : std_logic := '0';                  -- chute flap band (slot 3)
    -- cable (slot 8, direct logic): per-frame bounds
    signal cb_lo    : unsigned(10 downto 0) := (others => '0');
    signal cb_w     : unsigned(3 downto 0) := to_unsigned(12, 4);
    signal ln_cb    : std_logic := '0';
    signal r1_cbd   : unsigned(11 downto 0) := (others => '0');
    signal cb_y1    : t_c := (others => '0');
    signal x_cb     : boolean := false;
    -- lean operands, per frame
    signal s_oabs, s_jabs : unsigned(3 downto 0) := (others => '0');
    signal s_oneg, s_jneg : std_logic := '0';
    signal q3_rc, q4_rc, q5_rc : unsigned(2 downto 0) := (others => '0');

    type t_slot_c is array (0 to 8) of unsigned(10 downto 0);
    type t_slot_e is array (0 to 8) of unsigned(3 downto 0);
    signal sl_lo, sl_ro, sl_li, sl_ri, sl_hx : t_slot_c := (others => (others => '0'));
    signal sl_vo, sl_vi, sl_hl, sl_sh : std_logic_vector(8 downto 0) := (others => '0');
    signal sl_el    : t_slot_e := (others => (others => '0'));

    -- per-line extras
    signal q_gd     : std_logic_vector(5 downto 0) := (others => '0');
    signal x_wv, x_vf, x_d0, x_d1, x_stoff : t_c := (others => '0');
    signal x_r8     : unsigned(4 downto 0) := (others => '0');
    signal x_r39    : unsigned(5 downto 0) := (others => '0');
    signal x_ftc, x_ftd, y_b0, y_b1 : boolean := false;
    signal y_dy     : signed(4 downto 0) := (others => '0');
    signal ln_wsh   : unsigned(1 downto 0) := (others => '0');
    signal ln_wfr   : unsigned(3 downto 0) := (others => '0');
    signal ln_dark  : std_logic := '0';
    signal ln_dfr   : unsigned(4 downto 0) := (others => '0');
    signal ln_bulb  : std_logic := '0';
    signal ln_bady  : unsigned(2 downto 0) := (others => '0');
    signal ln_bdyhl : std_logic := '0';
    signal ln_tc, ln_td : std_logic := '0';
    signal ln_rowc, ln_rowd : unsigned(3 downto 0) := (others => '0');
    signal ln_stoff : t_c := (others => '0');

    ---------------------------------------------------------------------------
    -- Line buffer: 2 banks x (even, odd) halves, 512 x 8
    ---------------------------------------------------------------------------
    type t_lb is array (0 to 511) of std_logic_vector(7 downto 0);
    signal lb0e, lb0o, lb1e, lb1o : t_lb := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb0e : signal is true;
    attribute no_rw_check of lb0o : signal is true;
    attribute no_rw_check of lb1e : signal is true;
    attribute no_rw_check of lb1o : signal is true;
    signal lb_q0e, lb_q0o, lb_q1e, lb_q1o : std_logic_vector(7 downto 0) := (others => '0');
    signal s_par    : std_logic := '0';          -- bank displayed this line
    signal lb_ra    : unsigned(8 downto 0) := (others => '0');
    -- display clear port
    signal cl_a     : unsigned(8 downto 0) := (others => '0');
    signal cl_we    : std_logic := '0';
    signal cl_bank  : std_logic := '0';
    -- stamper write port
    signal st_a     : unsigned(8 downto 0) := (others => '0');
    signal st_de, st_do : std_logic_vector(7 downto 0) := (others => '0');
    signal st_wee, st_weo : std_logic := '0';
    signal st_bank  : std_logic := '0';
    -- registered write ports (one per bank; even/odd share the address)
    type t_lwa is array (0 to 1) of unsigned(8 downto 0);
    type t_lwd is array (0 to 1) of std_logic_vector(7 downto 0);
    signal lw_a     : t_lwa := (others => (others => '0'));
    signal lw_de, lw_do : t_lwd := (others => (others => '0'));
    signal lw_we, lw_wo : std_logic_vector(1 downto 0) := (others => '0');
    -- vblank wipe
    signal wp_on    : std_logic := '0';
    signal wp_a     : unsigned(8 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Stamper
    ---------------------------------------------------------------------------
    type t_sst is (S_IDLE, S_A0, S_A1, S_W, S_C0, S_C1, S_EV, S_EV2, S_ST);
    signal ss       : t_sst := S_IDLE;
    signal ss_y     : t_c := (others => '0');
    signal ss_bank  : std_logic := '0';
    signal ss_pre   : std_logic := '0';
    signal ss_i     : unsigned(5 downto 0) := (others => '0');
    signal sr_a     : unsigned(7 downto 0) := (others => '0');
    signal sr_q     : std_logic_vector(15 downto 0) := (others => '0');
    signal sr_q2    : std_logic_vector(15 downto 0) := (others => '0');
    signal sr_w0    : std_logic_vector(15 downto 0) := (others => '0');
    signal ss_m, ss_ml : unsigned(5 downto 0) := (others => '0');
    signal ss_mir   : std_logic := '0';
    signal ss_type  : unsigned(2 downto 0) := (others => '0');
    signal ss_dep   : unsigned(1 downto 0) := (others => '0');
    signal ss_pb    : signed(10 downto 0) := (others => '0');
    signal ss_odd   : std_logic := '0';
    signal ss_last  : std_logic := '0';
    signal ss_tail  : std_logic := '0';
    signal ss_first : std_logic := '0';
    signal ss_dy    : t_c := (others => '0');
    signal ss_x     : t_c := (others => '0');
    signal ss_tilt  : unsigned(1 downto 0) := (others => '0');
    signal ss_tr    : std_logic := '0';                    -- transposed (quarter turn)
    signal ss_tb    : unsigned(14 downto 0) := (others => '0');
    signal ss_v, ss_cf : unsigned(5 downto 0) := (others => '0');
    signal ss_v48, ss_c48 : unsigned(14 downto 0) := (others => '0');
    signal ss_pa    : unsigned(14 downto 0) := (others => '0');
    signal ss_step  : unsigned(14 downto 0) := (others => '0');
    -- stream pipe: a = address issued, b = ROM out, c = nibble, d = pal addr
    signal sp_addr  : unsigned(14 downto 0) := (others => '0');
    type t_ctl is record
        v    : std_logic;
        tail : std_logic;
        odd  : std_logic;
        first: std_logic;
        bank : std_logic;
        pa   : signed(10 downto 0);
        typ  : unsigned(2 downto 0);
        dep  : unsigned(1 downto 0);
    end record;
    constant C_CTL0 : t_ctl := ('0', '0', '0', '0', '0', (others => '0'), (others => '0'), (others => '0'));
    signal sc_a, sc_b, sc_c, sc_d : t_ctl := C_CTL0;
    signal sp_qa, sp_qb, sp_qc : std_logic_vector(3 downto 0) := (others => '0');
    signal sp_sel   : unsigned(1 downto 0) := (others => '0');
    signal sp_nib   : unsigned(3 downto 0) := (others => '0');
    signal sp_pal   : unsigned(7 downto 0) := (others => '0');
    signal sp_opq   : std_logic := '0';
    signal sp_prev  : std_logic_vector(7 downto 0) := (others => '0');
    signal sp_prevo : std_logic := '0';
    signal s_wpairs : signed(10 downto 0) := to_signed(480, 11);

    ---------------------------------------------------------------------------
    -- Game (vblank, shares the slot-RAM read port while the stamper idles)
    ---------------------------------------------------------------------------
    signal slot_ram : t_slots := C_SLOTS;
    signal gw_a     : unsigned(7 downto 0) := (others => '0');
    signal gw_d     : std_logic_vector(15 downto 0) := (others => '0');
    signal gw_we    : std_logic := '0';
    type t_gst is (G_IDLE, G_PRE, G_START, G_RD0, G_RD1, G_RW, G_RC0, G_RC1, G_E1, G_E1B, G_E2,
                   G_PICK, G_F0, G_F1, G_F2, G_F3, G_L0, G_L1, G_S0, G_S1, G_S2, G_S3,
                   G_S4, G_H0, G_H1, G_HD, G_RW1, G_W0, G_W1,
                   G_A0, G_A1, G_A2, G_A3, G_A4, G_A5);
    constant M_IDLE : unsigned(1 downto 0) := "00";
    constant M_HOLD : unsigned(1 downto 0) := "01";
    constant M_FALL : unsigned(1 downto 0) := "10";
    signal gm_mode  : unsigned(1 downto 0) := M_IDLE;
    signal gm_fx, gm_fy, gm_land, gm_dl : t_c := (others => '0');
    signal gm_fvy   : signed(7 downto 0) := (others => '0');
    signal gm_bnc, gm_win : std_logic := '0';
    -- held / falling prize orientation: (2) transpose, (1) mirror, (0) vertical
    -- flip, plus the pile lean (tilt 1/2) until it starts to tumble
    signal gm_ori   : std_logic_vector(2 downto 0) := (others => '0');
    signal gm_lean  : std_logic_vector(1 downto 0) := (others => '0');
    signal gm_bmir  : std_logic := '0';
    signal gm_e_mir, gm_e_tr : std_logic := '0';          -- record bits, pre-registered
    signal gm_e_tilt : std_logic_vector(1 downto 0) := (others => '0');
    signal gm_rt    : unsigned(1 downto 0) := (others => '0');
    signal s_flap_open : std_logic := '0';
    signal gm_evc, gm_evo, gm_chute, gm_bok : std_logic := '0';
    signal gm_cscan, gm_scanonly, gm_cand : std_logic := '0';  -- S9 auto-close: candidate scan
    signal gm_rig, gm_rigroll, gm_slip : std_logic := '0';      -- S11 rigged: the prize slips on the lift
    signal gm_sliph, gm_slipn : t_c := (others => '0');
    signal gm_bw1, gm_ow1 : std_logic_vector(15 downto 0) := (others => '0');
    signal gm_rst   : std_logic := '0';
    signal gm_hdx   : t_c := (others => '0');
    signal gm_ai    : unsigned(5 downto 0) := (others => '0');
    signal gm_areq  : std_logic := '0';
    signal gm_cwx, gm_relx, gm_holdy, gm_landy : t_c := (others => '0');
    signal gs       : t_gst := G_IDLE;
    signal gm_opq   : std_logic := '0';
    signal gm_type  : unsigned(2 downto 0) := (others => '0');
    signal gm_dep   : unsigned(1 downto 0) := (others => '0');
    signal gm_slot  : unsigned(5 downto 0) := (others => '0');
    signal gm_base  : unsigned(7 downto 0) := (others => '0');
    signal gm_np    : unsigned(5 downto 0) := (others => '0');
    signal gm_i     : unsigned(5 downto 0) := (others => '0');
    signal gm_w0    : std_logic_vector(15 downto 0) := (others => '0');
    signal gm_w1    : std_logic_vector(15 downto 0) := (others => '0');
    signal gm_dxc   : t_c := (others => '0');
    signal gm_ok    : std_logic := '0';
    signal gm_adx, gm_reach : t_c := (others => '0');
    signal gm_bv    : std_logic := '0';
    signal gm_bi    : unsigned(5 downto 0) := (others => '0');
    signal gm_by    : unsigned(10 downto 0) := (others => '1');
    signal gm_bt    : unsigned(2 downto 0) := (others => '0');
    signal gm_tgt   : unsigned(5 downto 0) := (others => '0');
    signal gm_ntype : unsigned(2 downto 0) := (others => '0');
    signal gm_refill: std_logic := '0';
    signal gm_lfsr  : unsigned(15 downto 0) := x"ACE1";
    signal gm_tol   : t_c := to_signed(48, 13);
    signal f_k3     : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Pixel pipeline
    ---------------------------------------------------------------------------
    signal r1_vx    : t_c := (others => '0');
    signal r1_ux    : unsigned(10 downto 0) := (others => '0');
    signal r1_av    : std_logic := '0';
    signal r1_b     : unsigned(3 downto 0) := (others => '0');
    signal r1_x1    : std_logic := '0';
    signal r1_pa    : unsigned(8 downto 0) := (others => '0');
    signal r1_par   : std_logic := '0';
    signal r2_o, r2_i, r2_hc : std_logic_vector(8 downto 0) := (others => '0');
    signal r2_bdx   : signed(5 downto 0) := (others => '0');
    signal r2_bxok  : std_logic := '0';
    signal r2_bidx  : unsigned(1 downto 0) := (others => '0');
    signal r2_bwin  : std_logic := '0';
    signal r2_st    : t_c := (others => '0');
    signal r2_b, r3_b : unsigned(3 downto 0) := (others => '0');
    signal r2_x1, r3_x1 : std_logic := '0';
    signal r2_par, r3_par : std_logic := '0';
    -- stage 3: classification
    signal r3_cls   : unsigned(1 downto 0) := (others => '0');  -- 0 opaque, 1 window, 2 text-bg, 3 door
    signal r3_pa    : unsigned(7 downto 0) := (others => '0');
    signal r3_fon   : std_logic := '0';
    signal r3_fpa   : unsigned(7 downto 0) := (others => '0');
    signal r3_con   : std_logic := '0';
    signal r3_cpa   : unsigned(7 downto 0) := (others => '0');
    signal r3_mon   : std_logic := '0';
    signal r3_mpa   : unsigned(7 downto 0) := (others => '0');
    signal r3_lift  : unsigned(2 downto 0) := (others => '0');
    signal r3_tsel  : unsigned(2 downto 0) := (others => '0');  -- text zone (0 none)
    signal r3_tci   : unsigned(2 downto 0) := (others => '0');
    signal r3_xlo, r2_xlo : unsigned(4 downto 0) := (others => '0');
    signal r2_cc, r2_cd : unsigned(5 downto 0) := (others => '0');
    -- text zone start cells (per mode): right block, title (64-unit cells x2), decal
    signal tz_c     : unsigned(5 downto 0) := to_unsigned(26, 6);
    signal tz_d     : unsigned(5 downto 0) := to_unsigned(36, 6);
    signal r3_bulb  : std_logic := '0';
    signal r3_bpa   : unsigned(7 downto 0) := (others => '0');
    signal r3_q0e, r3_q0o, r3_q1e, r3_q1o : std_logic_vector(7 downto 0) := (others => '0');
    -- stage 4
    signal r4_cls   : unsigned(1 downto 0) := (others => '0');
    signal r4_pa, r4_fpa, r4_cpa, r4_mpa, r4_wpa : unsigned(7 downto 0) := (others => '0');
    signal r4_fon, r4_con, r4_mon, r4_wall : std_logic := '0';
    signal r4_lift  : unsigned(2 downto 0) := (others => '0');
    signal r4_lb    : unsigned(7 downto 0) := (others => '0');
    signal r4_code  : unsigned(4 downto 0) := (others => '0');
    signal r4_row   : unsigned(3 downto 0) := (others => '0');
    signal r4_col   : unsigned(3 downto 0) := (others => '0');
    signal r4_ton   : std_logic := '0';
    signal r4_tid   : unsigned(1 downto 0) := (others => '0');  -- 0 title,1 score,2 decal
    -- stage 5..7: font fetch while the rest waits
    signal f5_addr  : unsigned(9 downto 0) := (others => '0');
    signal f6_q     : std_logic_vector(15 downto 0) := (others => '0');
    signal f7_q     : std_logic_vector(15 downto 0) := (others => '0');
    type t_d8 is array (5 to 7) of unsigned(7 downto 0);
    type t_d3 is array (5 to 7) of unsigned(2 downto 0);
    type t_d2 is array (5 to 7) of unsigned(1 downto 0);
    type t_d4 is array (5 to 7) of unsigned(3 downto 0);
    signal d_pa     : t_d8 := (others => (others => '0'));
    signal d_tid    : t_d2 := (others => (others => '0'));
    signal d_lift   : t_d3 := (others => (others => '0'));
    signal d_col    : t_d4 := (others => (others => '0'));
    signal d_wall, d_ton : std_logic_vector(7 downto 5) := (others => '0');
    -- stage 8: text pixel, stage 9: composite
    signal r8_tv    : unsigned(1 downto 0) := (others => '0');
    signal r8_pa    : unsigned(7 downto 0) := (others => '0');
    signal r8_tid   : unsigned(1 downto 0) := (others => '0');
    signal r8_lift  : unsigned(2 downto 0) := (others => '0');
    signal r8_wall  : std_logic := '0';
    signal r9_pa    : unsigned(7 downto 0) := (others => '0');
    signal r9_lift  : unsigned(2 downto 0) := (others => '0');
    signal r9_wall  : std_logic := '0';
    signal p10_q    : std_logic_vector(31 downto 0) := (others => '0');
    signal r10_lift, r11_lift : unsigned(2 downto 0) := (others => '0');
    signal r10_wall, r11_wall : std_logic := '0';
    signal r11_q    : std_logic_vector(31 downto 0) := (others => '0');
    signal d_hz     : t_d2 := (others => (others => '0'));
    signal r8_hz, r9_hz, r10_hz, r11_hz : unsigned(1 downto 0) := (others => '0');
    signal r16_y, r16_u, r16_v : unsigned(9 downto 0) := (others => '0');
    signal r11b_lift : unsigned(2 downto 0) := (others => '0');
    signal r11b_wall : std_logic := '0';
    signal s_dflash, s_desc : std_logic := '0';
    signal r12_y, r12_u, r12_v : unsigned(9 downto 0) := (others => '0');
    signal c_y0, c_y1, c_u0, c_u1, c_v0, c_v1 : unsigned(9 downto 0) := (others => '0');
    signal r14_y, r14_u, r14_v : unsigned(9 downto 0) := (others => '0');
    signal r15_y, r15_u, r15_v : unsigned(9 downto 0) := (others => '0');

    -- video tap for the back wall (S10)
    signal v_y0, v_u0, v_v0 : std_logic_vector(9 downto 0) := (others => '0');
    signal v_y, v_u, v_v    : unsigned(9 downto 0) := (others => '0');

    -- sync / avid delays
    type t_sync is array (0 to C_LAT - 1) of std_logic_vector(3 downto 0);
    signal s_sync   : t_sync := (others => (others => '1'));
    signal d_gv     : std_logic_vector(C_PIX downto 0) := (others => '0');
    signal s_out    : t_video_stream_yuv444_30b;

    -- helpers
    function f_th(th : unsigned(3 downto 0); c : natural) return unsigned is
        variable r : unsigned(7 downto 0);
    begin
        r(7 downto 4) := th;
        r(3 downto 0) := to_unsigned(c, 4);
        return r;
    end function;
    function f_faddr(code : unsigned(4 downto 0); row : unsigned(3 downto 0); h : std_logic)
        return unsigned is
        variable r : unsigned(9 downto 0);
    begin
        r(9 downto 5) := code;
        r(4 downto 1) := row;
        r(0) := h;
        return r;
    end function;
    -- glide step: d >> g (g = 1..5) as fixed slices
    function f_sh(d : signed(15 downto 0); g : unsigned(2 downto 0)) return signed is
    begin
        case to_integer(g) is
            when 1 => return shift_right(d, 1);
            when 2 => return shift_right(d, 2);
            when 3 => return shift_right(d, 3);
            when 4 => return shift_right(d, 4);
            when others => return shift_right(d, 5);
        end case;
    end function;
    -- a zero step with a non-zero distance moves by one (glides always land)
    function f_minstep(st, d : signed(15 downto 0)) return signed is
    begin
        if st = 0 and d /= 0 then
            if d(15) = '1' then return to_signed(-1, 16); else return to_signed(1, 16); end if;
        end if;
        return st;
    end function;
    -- one quarter turn clockwise of an orientation (transpose, mirror, vflip):
    -- all 8 flip/rotate states, computed from the stamper's pixel mapping
    function f_rot(o : std_logic_vector(2 downto 0)) return std_logic_vector is
        variable r : std_logic_vector(2 downto 0);
    begin
        case o is
            when "000" => r := "110";
            when "001" => r := "100";
            when "010" => r := "111";
            when "011" => r := "101";
            when "100" => r := "010";
            when "101" => r := "000";
            when "110" => r := "011";
            when others => r := "001";
        end case;
        return r;
    end function;
    -- quarter turn o -> (transpose, mirror, tilt(1:0)); tilt 3 = vertical flip
    function f_orient(o : unsigned(1 downto 0)) return std_logic_vector is
        variable r : std_logic_vector(3 downto 0);
    begin
        case to_integer(o) is
            when 1 => r := "1100";
            when 2 => r := "0111";
            when 3 => r := "1011";
            when others => r := "0000";
        end case;
        return r;
    end function;
    function f_bcd_inc(v : unsigned(7 downto 0)) return unsigned is
        variable r : unsigned(7 downto 0) := v;
    begin
        if v(3 downto 0) = 9 then
            r(3 downto 0) := (others => '0');
            if v(7 downto 4) = 9 then r(7 downto 4) := (others => '0');
            else r(7 downto 4) := v(7 downto 4) + 1; end if;
        else
            r(3 downto 0) := v(3 downto 0) + 1;
        end if;
        return r;
    end function;
    -- x < c against a constant, as AND/OR bit logic (no carry chain)
    function f_ltu(x : unsigned; c : integer) return boolean is
        variable xv : unsigned(x'length - 1 downto 0) := x;
        variable cb : unsigned(x'length - 1 downto 0);
        variable lt : boolean := false;
        variable eq : boolean := true;
    begin
        if c <= 0 then return false; end if;
        if c >= 2 ** x'length then return true; end if;
        cb := to_unsigned(c, x'length);
        for i in x'length - 1 downto 0 loop
            if cb(i) = '1' then
                lt := lt or (eq and xv(i) = '0');
                eq := eq and xv(i) = '1';
            else
                eq := eq and xv(i) = '0';
            end if;
        end loop;
        return lt;
    end function;
    function f_lts(x : signed; c : integer) return boolean is
        variable u : unsigned(x'length - 1 downto 0) := unsigned(x);
    begin
        u(x'length - 1) := not u(x'length - 1);          -- offset binary
        return f_ltu(u, c + 2 ** (x'length - 1));
    end function;
    -- clamp a coordinate to 0..2047 (bounds compare as 11-bit unsigned)
    function f_c11(x : signed(12 downto 0)) return unsigned is
    begin
        if x(12) = '1' then return to_unsigned(0, 11);
        elsif x(11) = '1' then return to_unsigned(2047, 11);
        else return unsigned(x(10 downto 0)); end if;
    end function;
    -- depth haze: c + k*(v - c), k = 1, 7/8, 3/4
    function f_haze(v : unsigned(9 downto 0); c : integer; hz : unsigned(1 downto 0))
        return unsigned is
        variable d : signed(11 downto 0) := signed(resize(v, 12)) - c;
        variable r : signed(11 downto 0);
    begin
        case hz is
            when "01"   => r := d - shift_right(d, 3);      -- x7/8
            when "10"   => r := d - shift_right(d, 2);      -- x3/4
            when others => r := d;
        end case;
        r := r + c;
        return unsigned(r(9 downto 0));              -- slice, not resize (sign fold)
    end function;
    function f_ramp(o, i, h, s : std_logic; sh, base, hl : unsigned(7 downto 0))
        return unsigned is
    begin
        if i = '0' then return PA_OUTLINE;
        elsif h = '1' then return hl;
        elsif s = '1' then return sh;
        else return base; end if;
    end function;

begin

    ---------------------------------------------------------------------------
    -- Timing class, free-running
    ---------------------------------------------------------------------------
    p_class : process(clk)
        variable v_t : std_logic_vector(3 downto 0);
    begin
        if rising_edge(clk) then
            v_t   := registers_in(8)(3 downto 0);
            s_xs  <= f_vstep_x(v_t);
            s_ys  <= f_vstep_y(v_t);
            g_wm1 <= f_width_m1(v_t);
            if f_vstep_x(v_t) = 4 then
                s_sd <= '1'; s_vw <= to_signed(1440, 13); s_vw2 <= to_signed(720, 13);
                s_wpairs <= to_signed(360, 11);
                tz_c <= to_unsigned(18, 6); tz_d <= to_unsigned(29, 6);
            else
                s_sd <= '0'; s_vw <= to_signed(1920, 13); s_vw2 <= to_signed(960, 13);
                s_wpairs <= to_signed(480, 11);
                tz_c <= to_unsigned(26, 6); tz_d <= to_unsigned(36, 6);
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Generated avid: starts 2-3 clocks after the source avid rise at the
    -- usual hsync parity, ends after the mode's width.
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
            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Coordinates and events
    ---------------------------------------------------------------------------
    s_first <= g_avid and not s_gav_d;

    p_coord : process(clk)
    begin
        if rising_edge(clk) then
            s_gav_d <= g_avid;
            s_gav_r <= g_avid and not s_gav_d;
            s_gav_f <= s_gav_d and not g_avid;
            if g_avid = '1' then
                s_xa   <= s_xa + s_xs;
                s_bcol <= s_bcol + 1;
            else
                s_xa   <= (others => '0');
                s_bcol <= (others => '0');
            end if;
            -- vsync edge, once per field (saw-active guard)
            s_vsq   <= data_in.vsync_n;
            s_vs_go <= '0';
            if g_avid = '1' then
                s_saw <= '1';
            end if;
            if s_vsq = '1' and data_in.vsync_n = '0' and s_saw = '1' then
                s_saw   <= '0';
                s_vs_go <= '1';
            end if;
            if s_vs_go = '1' then
                s_ya <= (others => '0');
                s_lc <= (others => '0');
            elsif s_gav_f = '1' then
                s_ya <= s_ya + resize(s_ys, 20);
                s_lc <= s_lc + 1;
            end if;
            s_yn <= s_ya + resize(s_ys, 20);
            s_vy <= signed(resize(s_ya(19 downto 8), 13));
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Frame control: knobs -> claw position (glide), open, depth, theme.
    -- Runs once per field from the vsync edge; one small op per state.
    ---------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_nd  : unsigned(1 downto 0);
        variable v_ta, v_tb : unsigned(15 downto 0);
        variable v_jt  : signed(15 downto 0);
        variable v_len : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- serial multiplier: m_acc = m_a * m_b (10 steps)
            if m_cnt /= 0 then
                if m_a(0) = '1' then
                    m_acc <= m_acc + m_b;
                end if;
                m_a   <= shift_right(m_a, 1);
                m_b   <= shift_left(m_b, 1);
                m_cnt <= m_cnt - 1;
            end if;

            ap_cx     <= s_vw - 200;     -- chute centre (registered: abc9 topo-order trap)
            -- arrived = the glide's own distances to the target are small
            -- (f_dx / f_dy, Q.4, recomputed each field after the targets move)
            if not f_lts(f_dx, -64) and f_lts(f_dx, 64) and not f_lts(f_dy, -64) and f_lts(f_dy, 64) then
                ap_arr <= '1';
            else
                ap_arr <= '0';
            end if;
            -- (the per-row offset is registered first: a case of constant adds
            -- feeding a clamp trips abc9's topological-order bug)
            ap_roff <= to_signed(30 - C_CLAW_REACH(to_integer(gm_td)), 13);
            -- (every pile row puts this inside the cable's range: no clamp)
            ap_ylow_n <= gm_ty + ap_roff;
            s_winfx <= '0';
            if s_wint /= 0 then s_winfx <= '1'; end if;
            case to_integer(f_st) is
                when 0 =>
                    if s_vs_go = '1' then
                        f_k1  <= unsigned(registers_in(0));
                        f_k2  <= unsigned(registers_in(1));
                        f_k3  <= unsigned(registers_in(2));
                        f_k4  <= unsigned(registers_in(3));
                        f_k5  <= unsigned(registers_in(4));
                        f_p12 <= unsigned(registers_in(7));
                        s_vid_on <= registers_in(6)(3);
                        f_sw7q <= registers_in(6)(0);
                        f_auto <= registers_in(6)(1);    -- S8: Auto plays itself
                        f_grabm <= registers_in(6)(2);   -- S9: Auto-close / Manual
                        f_rig  <= registers_in(6)(4);    -- S11: Rigged
                        f_k6   <= unsigned(registers_in(5));
                        if registers_in(6)(0) /= f_sw7q and registers_in(6)(1) = '0' then
                            s_open_t <= not s_open_t;
                            -- a hand close keeps auto-open off until the claw
                            -- rises; a hand open re-arms auto-close
                            s_aclk <= s_open_t;
                        end if;
                        -- win: the marquee bulbs flash for ~3 s
                        if gm_wtog /= f_wtq then
                            f_wtq  <= gm_wtog;
                            s_wint <= to_unsigned(180, 8);
                        elsif s_wint /= 0 then
                            s_wint <= s_wint - 1;
                        end if;
                        s_phase <= s_phase + 1;
                        f_st <= to_unsigned(1, 5);
                    end if;
                when 1 =>
                    m_a   <= f_k1;
                    m_b   <= resize(unsigned(s_vw - (C_CLAW_X_MARGIN + C_CLAW_X_MIN)), 21);
                    m_acc <= (others => '0');
                    m_cnt <= to_unsigned(10, 4);
                    f_st  <= to_unsigned(2, 5);
                when 2 =>
                    if m_cnt = 0 then
                        -- one add of gated operands (manual knob / autopilot
                        -- target): two arithmetic sources muxed into one
                        -- register trip abc9's topological-order bug
                        if f_auto = '1' then
                            v_ta := shift_left(resize(unsigned(ap_x), 16), 4);
                            v_tb := (others => '0');
                        else
                            v_ta := shift_left(resize(m_acc(20 downto 10), 16), 4);
                            v_tb := to_unsigned(C_CLAW_X_MIN * 16, 16);
                        end if;
                        f_xt  <= signed(v_ta + v_tb);
                        m_a   <= not f_p12;          -- slider down = claw down
                        m_b   <= to_unsigned(C_CLAW_Y_RANGE, 21);
                        m_acc <= (others => '0');
                        m_cnt <= to_unsigned(10, 4);
                        f_st  <= to_unsigned(3, 5);
                    end if;
                when 3 =>
                    if m_cnt = 0 then
                        if f_auto = '1' then
                            v_ta := shift_left(resize(unsigned(ap_yt), 16), 4);
                            v_tb := (others => '0');
                        else
                            v_ta := shift_left(resize(m_acc(20 downto 10), 16), 4);
                            v_tb := to_unsigned(C_CLAW_Y_MIN * 16, 16);
                        end if;
                        f_yt <= signed(v_ta + v_tb);
                        f_st <= to_unsigned(4, 5);
                    end if;
                when 4 =>
                    f_dx <= f_xt - f_xq;
                    f_dy <= f_yt - f_yq;
                    -- DROP flashes while the claw is on its way down
                    if f_dy > 32 then s_desc <= '1'; else s_desc <= '0'; end if;
                    s_dflash <= s_desc and s_phase(2);
                    -- glide shift from K4 (speed): slow 5 .. fast 2
                    f_g  <= to_unsigned(5, 3) - resize(f_k4(9 downto 8), 3);
                    f_st <= to_unsigned(5, 5);
                -- glide, one op per state: shift, min-step fix, add
                when 5 =>
                    f_s  <= f_sh(f_dx, f_g);
                    f_st <= to_unsigned(6, 5);
                when 6 =>
                    f_s2 <= f_minstep(f_s, f_dx);
                    f_st <= to_unsigned(8, 5);
                when 8 =>
                    f_xq <= f_xq + f_s2;
                    f_sxl <= f_s2;                   -- gantry speed, for the joystick
                    f_s  <= f_sh(f_dy, f_g - 1);
                    f_st <= to_unsigned(9, 5);
                when 9 =>
                    f_s2 <= f_minstep(f_s, f_dy);
                    f_st <= to_unsigned(10, 5);
                when 10 =>
                    f_yq <= f_yq + f_s2;
                    f_st <= to_unsigned(7, 5);
                when 7 =>
                    s_cx <= resize(f_xq(15 downto 4), 13);
                    s_cy <= resize(f_yq(15 downto 4), 13);
                    if s_open_t = '1' then
                        if s_open < C_OPEN_MAX then s_open <= s_open + 1; end if;
                    else
                        if s_open > C_OPEN_MIN then s_open <= s_open - 1; end if;
                    end if;
                    -- labelled knobs: a fresh load writes the label INDEX (0..N-1)
                    if f_ltu(f_k2, 3) then v_nd := f_k2(1 downto 0);
                    elsif f_ltu(f_k2, 341) then v_nd := "00";
                    elsif f_ltu(f_k2, 682) then v_nd := "01";
                    else v_nd := "10"; end if;
                    if f_auto = '1' then v_nd := ap_d; end if;
                    s_cdepth <= v_nd;
                    -- a row change rocks the joystick forward / back for a moment
                    if v_nd /= s_cdepth then
                        s_jpt <= to_unsigned(24, 5);
                        if v_nd > s_cdepth then s_jpd <= '1'; else s_jpd <= '0'; end if;
                    elsif s_jpt /= 0 then
                        s_jpt <= s_jpt - 1;
                    end if;
                    if f_ltu(f_k5, 6) then s_thhi <= f_k5(3 downto 0) + 4;
                    elsif f_ltu(f_k5, 171) then s_thhi <= to_unsigned(4, 4);
                    elsif f_ltu(f_k5, 341) then s_thhi <= to_unsigned(5, 4);
                    elsif f_ltu(f_k5, 512) then s_thhi <= to_unsigned(6, 4);
                    elsif f_ltu(f_k5, 683) then s_thhi <= to_unsigned(7, 4);
                    elsif f_ltu(f_k5, 853) then s_thhi <= to_unsigned(8, 4);
                    else s_thhi <= to_unsigned(9, 4); end if;
                    f_st <= to_unsigned(11, 5);
                -- joystick: lean follows the gantry's speed (springs back when it
                -- stops); pose from the row-change rock
                when 11 =>
                    v_jt := shift_right(f_sxl, 4);
                    if v_jt > 8 then v_jt := to_signed(8, 16);
                    elsif v_jt < -8 then v_jt := to_signed(-8, 16); end if;
                    if resize(s_jo, 16) < v_jt then s_jo <= s_jo + 1;
                    elsif resize(s_jo, 16) > v_jt then s_jo <= s_jo - 1; end if;
                    if s_jpt = 0 then
                        s_jy <= to_signed(C_JOY_UP_Y, 13); s_jr <= to_signed(C_JOY_UP_R, 13);
                        s_jrc <= to_unsigned(C_JOY_UP_RC, 3);
                    elsif s_jpd = '1' then
                        s_jy <= to_signed(C_JOY_BACK_Y, 13); s_jr <= to_signed(C_JOY_BACK_R, 13);
                        s_jrc <= to_unsigned(C_JOY_BACK_RC, 3);
                    else
                        s_jy <= to_signed(C_JOY_FRONT_Y, 13); s_jr <= to_signed(C_JOY_FRONT_R, 13);
                        s_jrc <= to_unsigned(C_JOY_FRONT_RC, 3);
                    end if;
                    f_st <= to_unsigned(12, 5);
                when 12 =>
                    -- ball x = stick base + (stick length * lean) >> 5, one op a state
                    v_len := unsigned(resize(C_JOY_BASE_Y - s_jy, 7));
                    f_jm  <= v_len * s_jabs;
                    f_st <= to_unsigned(14, 5);
                when 14 =>
                    if s_jneg = '1' then
                        f_jp <= -signed(resize(shift_right(f_jm, C_LEAN_SHIFT), 13));
                    else
                        f_jp <= signed(resize(shift_right(f_jm, C_LEAN_SHIFT), 13));
                    end if;
                    f_st <= to_unsigned(15, 5);
                when 15 =>
                    f_jb <= s_vw2 + C_JOY_X;
                    f_st <= to_unsigned(13, 5);
                when 13 =>
                    s_jx <= f_jb + f_jp;
                    cb_lo <= f_c11(s_cx - C_CABLE_HW(to_integer(s_cdepth)));
                    cb_w  <= to_unsigned(2 * C_CABLE_HW(to_integer(s_cdepth)), 4);
                    cb_y1 <= s_cy + C_CABLE_BOT(to_integer(s_cdepth));
                    if s_open < 0 then s_oabs <= unsigned(resize(-s_open, 4)); s_oneg <= '1';
                    else s_oabs <= unsigned(s_open(3 downto 0)); s_oneg <= '0'; end if;
                    if s_jo < 0 then s_jabs <= unsigned(resize(-s_jo, 4)); s_jneg <= '1';
                    else s_jabs <= unsigned(s_jo(3 downto 0)); s_jneg <= '0'; end if;
                    f_st <= to_unsigned(16, 5);
                -- autopilot (S8 Auto): drives the same targets / open-close the
                -- player would; one step per field
                when 16 =>
                    if f_auto = '0' then
                        ap_st   <= AP_PICK;
                        ap_sent <= '0';
                        -- S9 Auto-close: the claw opens as it drops and shuts by
                        -- itself once a prize is in reach (gm_cand: the game's
                        -- scan-only pass, last field); S7 still opens / closes
                        if f_lts(f_dy, -32) then s_aclk <= '0'; end if;
                        if f_grabm = '0' and gm_mode = M_IDLE and s_aclk = '0' then
                            if s_open_t = '0' and s_desc = '1' then
                                s_open_t <= '1';
                            elsif s_open_t = '1' and s_open = C_OPEN_MAX and gm_cand = '1' then
                                s_open_t <= '0';
                                s_aclk   <= '1';
                            end if;
                        end if;
                    else
                        case ap_st is
                            when AP_PICK =>
                                ap_yt <= to_signed(C_CLAW_Y_MIN, 13);
                                if gm_tok /= ap_tq then
                                    ap_tq   <= gm_tok;
                                    ap_sent <= '0';
                                    ap_x    <= gm_tx;
                                    ap_d    <= gm_td;
                                    ap_ylow <= ap_ylow_n;
                                    s_open_t <= '1';
                                    ap_st   <= AP_MOVE;
                                elsif ap_sent = '0' then
                                    f_areq  <= not f_areq;
                                    ap_sent <= '1';
                                end if;
                            when AP_MOVE =>
                                if ap_arr = '1' then
                                    ap_yt <= ap_ylow;
                                    ap_st <= AP_LOWER;
                                end if;
                            when AP_LOWER =>
                                if ap_arr = '1' then
                                    s_open_t <= '0';
                                    ap_tmr   <= to_unsigned(30, 7);
                                    ap_st    <= AP_CLOSE;
                                end if;
                            when AP_CLOSE =>
                                if ap_tmr = 0 then
                                    ap_yt <= to_signed(C_CLAW_Y_MIN, 13);
                                    ap_st <= AP_LIFT;
                                else
                                    ap_tmr <= ap_tmr - 1;
                                end if;
                            when AP_LIFT =>
                                if ap_arr = '1' then
                                    if gm_mode = M_HOLD then
                                        ap_x  <= ap_cx;
                                        ap_st <= AP_CHUTE;
                                    else
                                        ap_st <= AP_PICK;     -- missed: pick again
                                    end if;
                                end if;
                            when AP_CHUTE =>
                                if ap_arr = '1' then
                                    s_open_t <= '1';
                                    ap_tmr   <= to_unsigned(70, 7);
                                    ap_st    <= AP_WAIT;
                                end if;
                            when others =>               -- AP_WAIT
                                if ap_tmr = 0 then
                                    ap_st <= AP_PICK;
                                else
                                    ap_tmr <= ap_tmr - 1;
                                end if;
                        end case;
                    end if;
                    f_st <= to_unsigned(0, 5);
                when others =>
                    f_st <= to_unsigned(0, 5);
            end case;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Per-line slot sequencer.  Starts one clock after the line's vy is
    -- updated (avid fall, or the vsync edge for line 0); 16 elements, one per
    -- clock, 7-stage pipe; done long before the next active video.
    ---------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_e   : integer range 0 to 15;
        variable v_b   : t_c;
        variable v_k   : t_c;
        variable v_io  : t_c;
        variable v_ii  : t_c;
        variable v_lean: t_c;
        variable v_o   : unsigned(3 downto 0);
        variable v_t   : integer;
        variable v_v   : t_c;
        variable v_f   : t_c;
        variable v_dy  : signed(12 downto 0);
        variable v_x0, v_x1, v_y0, v_y1 : integer;
        variable v_rc  : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            q_go  <= s_gav_f or s_vs_go;
            q_go2 <= q_go;
            -- stage 0: start / step
            q_pv <= q_pv(5 downto 0) & '0';
            for k in 1 to 6 loop
                q_pe(k) <= q_pe(k - 1);
            end loop;
            if q_go = '1' then
                q_run <= '1';
                q_e   <= (others => '0');
                sl_vo <= (others => '0');
                sl_vi <= (others => '0');
            elsif q_run = '1' then
                v_e := to_integer(q_e(3 downto 0));
                q_pv(0) <= '1';
                q_pe(0) <= q_e;
                -- claw elements use the depth row's scaled offsets
                case s_cdepth is
                    when "01" =>
                        v_x0 := C_EL_X0_1(v_e); v_x1 := C_EL_X1_1(v_e);
                        v_y0 := C_EL_Y0_1(v_e); v_y1 := C_EL_Y1_1(v_e);
                    when "10" =>
                        v_x0 := C_EL_X0_2(v_e); v_x1 := C_EL_X1_2(v_e);
                        v_y0 := C_EL_Y0_2(v_e); v_y1 := C_EL_Y1_2(v_e);
                    when others =>
                        v_x0 := C_EL_X0(v_e); v_x1 := C_EL_X1(v_e);
                        v_y0 := C_EL_Y0(v_e); v_y1 := C_EL_Y1(v_e);
                end case;
                case C_EL_XA(v_e) is
                    when 0 => v_b := (others => '0');
                    when 1 => v_b := s_vw;
                    when 2 => v_b := s_vw2;
                    when 3 => v_b := s_cx;
                    when others => v_b := s_jx;
                end case;
                q1_x0 <= v_b + to_signed(v_x0, 13);
                case C_EL_XB(v_e) is
                    when 0 => v_b := (others => '0');
                    when 1 => v_b := s_vw;
                    when 2 => v_b := s_vw2;
                    when 3 => v_b := s_cx;
                    when others => v_b := s_jx;
                end case;
                q1_x1 <= v_b + to_signed(v_x1, 13);
                case C_EL_YA(v_e) is
                    when 1 => q1_y0 <= s_cy + to_signed(v_y0, 13);
                    when 2 => q1_y0 <= s_jy + to_signed(v_y0, 13);
                    when others => q1_y0 <= to_signed(v_y0, 13);
                end case;
                case C_EL_YB(v_e) is
                    when 1 => q1_y1 <= s_cy + to_signed(v_y1 - 1, 13);
                    when 2 => q1_y1 <= s_jy + to_signed(v_y1 - 1, 13);
                    when others => q1_y1 <= to_signed(v_y1 - 1, 13);
                end case;
                -- element 16 (hardwired, the table is 16 entries): the prize hole
                if q_e(4) = '1' then
                    q1_x0 <= s_vw + C_EL_HOLE_X0;
                    q1_x1 <= s_vw + C_EL_HOLE_X1;
                    q1_y0 <= to_signed(C_EL_HOLE_Y0, 13);
                    q1_y1 <= to_signed(C_EL_HOLE_Y1 - 1, 13);
                end if;
                -- the joystick ball's box follows its pose radius
                if v_e = E_JOY_BALL and q_e(4) = '0' then
                    q1_x0 <= s_jx - s_jr;
                    q1_x1 <= s_jx + s_jr;
                    q1_y0 <= s_jy - s_jr;
                    q1_y1 <= s_jy + s_jr - 1;
                end if;
                if q_e = C_NEL then              -- 16 table elements + the hole
                    q_run <= '0';
                end if;
                q_e <= q_e + 1;
            end if;

            -- stage 1: row tests
            q2_x0 <= q1_x0;
            q2_x1 <= q1_x1;
            q2_kt <= s_vy - q1_y0;
            q2_kb <= q1_y1 - s_vy;
            if s_vy >= q1_y0 and s_vy <= q1_y1 then
                q2_in <= '1';
            else
                q2_in <= '0';
            end if;

            -- stage 2: corner row index, lean product
            v_e := to_integer(q_pe(1)(3 downto 0));
            q3_x0 <= q2_x0;
            q3_x1 <= q2_x1;
            q3_kt <= q2_kt;
            q3_kb <= q2_kb;
            q3_in <= q2_in;
            if q2_kt < q2_kb then v_k := q2_kt; else v_k := q2_kb; end if;
            if v_k(12) = '1' then v_k := (others => '0');
            elsif v_k(11 downto 5) /= 0 then v_k := to_signed(31, 13); end if;
            if q_pe(1)(4) = '1' then v_rc := C_EL_HOLE_RC;
            elsif v_e = E_JOY_BALL then v_rc := to_integer(s_jrc);
            else v_rc := C_EL_RC(v_e); end if;
            q3_rc   <= to_unsigned(v_rc, 3);
            q3_addr <= to_unsigned(v_rc, 3) & unsigned(v_k(4 downto 0));
            if q_pe(1)(4) = '1' then
                q3_prod <= (others => '0');
                q3_neg  <= '0';
            elsif C_EL_LEAN(v_e) = 2 then
                -- joystick stick: pivot at the bottom row
                q3_prod <= unsigned(q2_kb(7 downto 0)) * s_jabs;
                q3_neg  <= s_jneg;
            elsif C_EL_LEAN(v_e) = 0 then
                q3_prod <= (others => '0');
                q3_neg  <= '0';
            else
                q3_prod <= unsigned(q2_kt(7 downto 0)) * s_oabs;
                if C_EL_LEAN(v_e) < 0 then q3_neg <= not s_oneg; else q3_neg <= s_oneg; end if;
            end if;

            -- stage 3: inset ROM read (registered EBR output), lean applied
            q4_ins <= C_INSET(to_integer(q3_addr));
            v_lean := signed(resize(q3_prod(11 downto C_LEAN_SHIFT), 13));
            if q3_neg = '1' then v_lean := -v_lean; end if;
            q4_x0 <= q3_x0 + v_lean;
            q4_x1 <= q3_x1 + v_lean;
            q4_kt <= q3_kt;
            q4_kb <= q3_kb;
            q4_in <= q3_in;
            q4_rc <= q3_rc;

            -- stage 4: fabric register for the EBR word
            q5_ins <= q4_ins;
            q5_x0 <= q4_x0; q5_x1 <= q4_x1; q5_kt <= q4_kt; q5_kb <= q4_kb; q5_in <= q4_in;
            q5_rc <= q4_rc;

            -- stage 5: bounds + pre-decoded slot write enable
            v_e  := to_integer(q_pe(4)(3 downto 0));
            v_t  := C_RC_T(to_integer(q5_rc));
            v_io := signed(resize(unsigned(q5_ins(5 downto 0)), 13));
            v_ii := signed(resize(unsigned(q5_ins(13 downto 8)), 13));
            q6_lo <= f_c11(q5_x0 + v_io);
            q6_ro <= f_c11(q5_x1 - v_io);
            q6_li <= f_c11(q5_x0 + v_ii);
            q6_ri <= f_c11(q5_x1 - v_ii);
            q6_hx <= f_c11(q5_x0 + v_ii + C_HL_COLS);
            if q5_ins(13 downto 8) = "111111" then q6_vi <= '0'; else q6_vi <= '1'; end if;
            if q5_kt < v_t + C_HL_ROWS then q6_hl <= '1'; else q6_hl <= '0'; end if;
            if q5_kb < v_t + C_SH_ROWS then q6_sh <= '1'; else q6_sh <= '0'; end if;
            -- chute flap = the chute's top band
            if q5_kt < v_t + C_FLAP_ROWS then q6_fl <= '1'; else q6_fl <= '0'; end if;
            if q5_kt >= v_t + C_FLAP_ROWS - C_FLAP_EDGE then q6_fo <= '1'; else q6_fo <= '0'; end if;
            if q_pe(4)(4) = '1' then
                q6_el <= to_unsigned(E_HOLE, 4);
            else
                q6_el <= q_pe(4)(3 downto 0);
            end if;
            for s in 0 to 8 loop
                if q_pv(4) = '1' and q5_in = '1' and
                   ((q_pe(4)(4) = '1' and s = C_EL_HOLE_SLOT) or (q_pe(4)(4) = '0' and C_EL_SLOT(v_e) = s)) then
                    q6_we(s) <= '1';
                else
                    q6_we(s) <= '0';
                end if;
            end loop;

            -- stage 6: slot register file write
            for s in 0 to 8 loop
                if q6_we(s) = '1' then
                    sl_lo(s) <= q6_lo;
                    sl_ro(s) <= q6_ro;
                    sl_li(s) <= q6_li;
                    sl_ri(s) <= q6_ri;
                    sl_hx(s) <= q6_hx;
                    sl_vo(s) <= '1';
                    sl_vi(s) <= q6_vi;
                    sl_hl(s) <= q6_hl;
                    sl_sh(s) <= q6_sh;
                    sl_el(s) <= q6_el;
                    if s = 3 then
                        sl_fl <= q6_fl;
                        sl_fo <= q6_fo;
                    end if;
                end if;
            end loop;

            -- per-line extras: free-running from s_vy (stable all line), two
            -- register stages, latched once settled (q_gd(5))
            q_gd <= q_gd(4 downto 0) & q_go;
            x_wv  <= s_vy - 164;
            x_vf  <= s_vy - C_PILE_DARK_Y;
            x_d0  <= s_vy - C_BULB_Y0;
            x_d1  <= s_vy - C_BULB_Y1;
            x_r8  <= unsigned(s_vy(4 downto 0)) - 8;      -- rows of y0 = 40, 72, 1000
            x_r39 <= unsigned(s_vy(5 downto 0)) - 39;     -- title rows
            x_ftc <= f_lts(s_vy, 103) and not f_lts(s_vy, 39);
            x_ftd <= f_lts(s_vy, 1032) and not f_lts(s_vy, 1000);
            x_stoff <= s_vy - s_vw2;
            x_cb    <= not f_lts(s_vy, C_CABLE_TOP) and (s_vy < cb_y1);
            -- stage 2
            y_b0 <= f_lts(x_d0, 8) and not f_lts(x_d0, -7);
            y_b1 <= f_lts(x_d1, 8) and not f_lts(x_d1, -7);
            if f_lts(x_d0, 8) and not f_lts(x_d0, -7) then
                y_dy <= x_d0(4 downto 0);
            else
                y_dy <= x_d1(4 downto 0);
            end if;
            if q_gd(5) = '1' then
                -- back wall gradient: v16 = (vy - 164) >> 4
                v_v := x_wv;
                if v_v(12) = '1' then v_v := (others => '0'); end if;
                if not f_ltu(unsigned(v_v(12 downto 4)), 48) then
                    ln_wsh <= "10"; ln_wfr <= x"F";
                else
                    if v_v(9 downto 8) = "11" then ln_wsh <= "10"; else ln_wsh <= unsigned(v_v(9 downto 8)); end if;
                    ln_wfr <= unsigned(v_v(7 downto 4));
                end if;
                if x_vf(12) = '0' then
                    ln_dark <= '1';
                    if not f_lts(x_vf, 64) then ln_dfr <= "10000"; else ln_dfr <= resize(unsigned(x_vf(5 downto 2)), 5); end if;
                else
                    ln_dark <= '0';
                    ln_dfr  <= (others => '0');
                end if;
                -- bulbs
                if y_b0 or y_b1 then ln_bulb <= '1'; else ln_bulb <= '0'; end if;
                ln_bady <= resize(unsigned(abs(y_dy)), 3);
                if not f_lts(y_dy, -4) and f_lts(y_dy, -1) then ln_bdyhl <= '1'; else ln_bdyhl <= '0'; end if;
                -- text rows (see claw_layout.TEXTS)
                ln_tc <= '0'; ln_td <= '0';
                ln_rowd <= x_r8(4 downto 1);
                ln_rowc <= x_r39(5 downto 2);
                if x_ftc then ln_tc <= '1'; end if;
                if x_ftd then ln_td <= '1'; end if;
                ln_stoff <= x_stoff;
                if x_cb then ln_cb <= '1'; else ln_cb <= '0'; end if;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Line buffer RAMs: one write statement each.  During the vblank wipe
    -- every RAM clears; otherwise the displayed bank takes the clear-behind
    -- port and the other bank the stamper.
    ---------------------------------------------------------------------------
    -- write-port mux, registered: (addr, data, we) per RAM
    p_lbw : process(clk)
    begin
        if rising_edge(clk) then
            for b in 0 to 1 loop
                if wp_on = '1' then
                    lw_a(b)  <= wp_a;
                    lw_de(b) <= (others => '0');
                    lw_do(b) <= (others => '0');
                    lw_we(b) <= '1';
                    lw_wo(b) <= '1';
                elsif (cl_bank = '1') = (b = 1) then
                    lw_a(b)  <= cl_a;
                    lw_de(b) <= (others => '0');
                    lw_do(b) <= (others => '0');
                    lw_we(b) <= cl_we;
                    lw_wo(b) <= cl_we;
                else
                    lw_a(b)  <= st_a;
                    lw_de(b) <= st_de;
                    lw_do(b) <= st_do;
                    if (st_bank = '1') = (b = 1) then
                        lw_we(b) <= st_wee;
                        lw_wo(b) <= st_weo;
                    else
                        lw_we(b) <= '0';
                        lw_wo(b) <= '0';
                    end if;
                end if;
            end loop;
        end if;
    end process;

    p_lb0e : process(clk)
    begin
        if rising_edge(clk) then
            if lw_we(0) = '1' then lb0e(to_integer(lw_a(0))) <= lw_de(0); end if;
            lb_q0e <= lb0e(to_integer(lb_ra));
        end if;
    end process;
    p_lb0o : process(clk)
    begin
        if rising_edge(clk) then
            if lw_wo(0) = '1' then lb0o(to_integer(lw_a(0))) <= lw_do(0); end if;
            lb_q0o <= lb0o(to_integer(lb_ra));
        end if;
    end process;
    p_lb1e : process(clk)
    begin
        if rising_edge(clk) then
            if lw_we(1) = '1' then lb1e(to_integer(lw_a(1))) <= lw_de(1); end if;
            lb_q1e <= lb1e(to_integer(lb_ra));
        end if;
    end process;
    p_lb1o : process(clk)
    begin
        if rising_edge(clk) then
            if lw_wo(1) = '1' then lb1o(to_integer(lw_a(1))) <= lw_do(1); end if;
            lb_q1o <= lb1o(to_integer(lb_ra));
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Stamper: per active line (and once in vblank for line 0), walk the slot
    -- records and stream each intersecting prize row into the next line's
    -- bank.  A record costs 6 clocks, a hit row its opaque width + 1.
    ---------------------------------------------------------------------------
    p_stamp : process(clk)
        variable v_dy   : t_c;
        variable v_x    : t_c;
        variable v_sh   : t_c;
        variable v_e0   : t_c;
        variable v_f, v_l : unsigned(5 downto 0);
        variable v_col  : unsigned(5 downto 0);
        variable v_ext  : unsigned(11 downto 0);
        variable v_v    : unsigned(5 downto 0);
        variable v_tb   : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            sr_q  <= slot_ram(to_integer(sr_a));
            sr_q2 <= sr_q;
            sc_a.v <= '0';

            -- vblank wipe, then the line-0 prestamp
            if s_vs_go = '1' then
                wp_on <= '1';
                wp_a  <= (others => '0');
            elsif wp_on = '1' then
                wp_a <= wp_a + 1;
                if wp_a = 511 then
                    wp_on <= '0';
                    ss_pre <= '1';
                end if;
            end if;

            if s_first = '1' then
                -- a new line: display flips banks; stamp the next line
                s_par   <= not s_par;
                ss_bank <= s_par;              -- = the bank NOT displayed now
                ss_y    <= signed(resize(s_yn(19 downto 8), 13));
                ss_i    <= (others => '0');
                ss      <= S_A0;
            elsif ss_pre = '1' and wp_on = '0' then
                ss_pre  <= '0';
                ss_bank <= not s_par;
                ss_y    <= (others => '0');
                ss_i    <= (others => '0');
                ss      <= S_A0;
            else
                case ss is
                    when S_IDLE =>
                        null;
                    when S_A0 =>
                        -- record i: words 2i, 2i+1 (SD list at 64)
                        if s_sd = '1' then
                            sr_a <= to_unsigned(64, 8) + shift_left(resize(ss_i, 8), 1);
                        else
                            sr_a <= shift_left(resize(ss_i, 8), 1);
                        end if;
                        ss <= S_A1;
                    when S_A1 =>
                        sr_a(0) <= '1';
                        ss <= S_W;
                    when S_W =>
                        ss <= S_C0;
                    when S_C0 =>
                        sr_w0 <= sr_q2;              -- w0 (issued in A0)
                        ss <= S_C1;
                    when S_C1 =>
                        -- sr_q2 = w1 now
                        v_dy := ss_y - signed(resize(unsigned(sr_q2(15 downto 5)), 13));
                        ss_dy   <= v_dy;
                        ss_tilt <= unsigned(sr_q2(2 downto 1));
                        ss_tr   <= sr_q2(0);
                        ss_tb   <= C_TBASE(to_integer(unsigned(sr_w0(4 downto 2))));
                        ss_dep  <= unsigned(sr_q2(4 downto 3));
                        ss_x    <= signed(resize(unsigned(sr_w0(15 downto 5)), 13));
                        ss_type <= unsigned(sr_w0(4 downto 2));
                        ss_mir  <= sr_w0(1);
                        ss_last <= sr_w0(0);
                        ss <= S_EV;
                    when S_EV =>
                        if ss_dy(12 downto 8) = "00000" and f_ltu(unsigned(ss_dy(7 downto 0)), 192) then
                            -- shear for tilt: +-((dy - 96) >> 3)
                            v_sh := shift_right(ss_dy - 96, 3);
                            v_x  := ss_x;
                            if ss_tilt = 1 then v_x := v_x + v_sh;
                            elsif ss_tilt = 2 then v_x := v_x - v_sh; end if;
                            v_e0 := shift_right(v_x, 1);          -- entry
                            -- transposed prizes span their opaque ROWS across
                            if ss_tr = '1' then
                                v_ext := C_YEXT(to_integer(ss_type));
                            else
                                v_ext := C_XEXT(to_integer(ss_type));
                            end if;
                            if ss_mir = '1' then
                                v_f := 47 - v_ext(5 downto 0);
                                v_l := 47 - v_ext(11 downto 6);
                            else
                                v_f := v_ext(11 downto 6);
                                v_l := v_ext(5 downto 0);
                            end if;
                            ss_odd <= v_e0(0);
                            ss_pb  <= resize(shift_right(v_e0, 1), 11);
                            ss_m   <= v_f;
                            if v_e0(0) = '1' then ss_ml <= v_l + 1; else ss_ml <= v_l; end if;
                            -- source row (vertical flip for tilt 3) and first column
                            v_v := unsigned(ss_dy(7 downto 2));
                            if ss_tilt = 3 then v_v := 47 - v_v; end if;
                            if ss_mir = '1' then v_col := 47 - v_f; else v_col := v_f; end if;
                            ss_v   <= v_v;
                            ss_cf  <= v_col;
                            ss_v48 <= shift_left(resize(v_v, 15), 5) + shift_left(resize(v_v, 15), 4);
                            ss_c48 <= shift_left(resize(v_col, 15), 5) + shift_left(resize(v_col, 15), 4);
                            ss_tail  <= '0';
                            ss_first <= '1';
                            ss <= S_EV2;
                        elsif ss_last = '1' then
                            ss <= S_IDLE;
                        else
                            ss_i <= ss_i + 1;
                            ss   <= S_A0;
                        end if;
                    when S_EV2 =>
                        -- start address and per-pixel step: a normal row walks
                        -- +-1 along {row, col}; a transposed one walks +-48 down
                        -- a source column
                        if ss_tr = '1' then
                            ss_pa <= ss_tb + resize(ss_v, 15) + ss_c48;
                            if ss_mir = '1' then ss_step <= to_unsigned(32768 - 48, 15);
                            else ss_step <= to_unsigned(48, 15); end if;
                        else
                            ss_pa <= ss_tb + ss_v48 + resize(ss_cf, 15);
                            if ss_mir = '1' then ss_step <= (others => '1');
                            else ss_step <= to_unsigned(1, 15); end if;
                        end if;
                        ss <= S_ST;
                    when S_ST =>
                        -- one sprite pixel per clock
                        sp_addr <= ss_pa;
                        ss_pa   <= ss_pa + ss_step;
                        sc_a.v    <= '1';
                        sc_a.first<= ss_first;
                        ss_first  <= '0';
                        sc_a.odd  <= ss_odd;
                        sc_a.bank <= ss_bank;
                        sc_a.pa   <= ss_pb + signed(resize(ss_m, 11));
                        sc_a.typ  <= ss_type;
                        sc_a.dep  <= ss_dep;
                        if ss_odd = '1' and ss_m = ss_ml then
                            sc_a.tail <= '1';      -- the extra pair for odd alignment
                        else
                            sc_a.tail <= '0';
                        end if;
                        if ss_m = ss_ml then
                            if ss_last = '1' then
                                ss <= S_IDLE;
                            else
                                ss_i <= ss_i + 1;
                                ss   <= S_A0;
                            end if;
                        else
                            ss_m <= ss_m + 1;
                        end if;
                end case;
            end if;

            -------------------------------------------------------------------
            -- Game: once per field during the vblank wipe (the stamper is
            -- idle).  Claw close low over a prize = grab (topmost reachable
            -- prize of the claw's depth row, centred within K3's tolerance);
            -- open while holding = win over the chute, else drop back.
            -------------------------------------------------------------------
            gw_we <= '0';
            -- decisions pre-registered every clock, so the game states below
            -- only test flops (the state -> enable cones were routing-bound)
            if s_open_t /= gm_opq and s_open_t = '0' and gm_mode = M_IDLE then gm_evc <= '1'; else gm_evc <= '0'; end if;
            if s_open_t /= gm_opq and s_open_t = '1' and gm_mode = M_HOLD then gm_evo <= '1'; else gm_evo <= '0'; end if;
            if f_grabm = '0' and f_auto = '0' and gm_mode = M_IDLE and s_open_t = '1' then gm_cscan <= '1'; else gm_cscan <= '0'; end if;
            -- rigged: roll the odds (K6) at the grab; slip at a random height
            -- on the lift (or at the top of it)
            if f_rig = '1' and gm_lfsr(9 downto 0) < f_k6 then gm_rigroll <= '1'; else gm_rigroll <= '0'; end if;
            -- one subtract, offset folded into the bits: 128..255 lines up
            -- (s_cy - rnd - 48 tripped abc9's topological-order bug)
            gm_slipn <= s_cy - signed(std_logic_vector'("00000" & '1' & std_logic_vector(gm_lfsr(15 downto 9))));
            if gm_mode = M_HOLD and gm_rig = '1' and (s_cy < gm_sliph or f_lts(s_cy, C_CLAW_Y_MIN + 12)) then
                gm_slip <= '1';
            else
                gm_slip <= '0';
            end if;
            gm_cwx   <= s_vw - C_CHUTE_WIN_X;
            if s_cx >= gm_cwx then gm_chute <= '1'; else gm_chute <= '0'; end if;
            gm_relx  <= s_cx - 96;
            gm_holdy <= s_cy + C_CLAW_HOLD(to_integer(gm_dep));
            gm_landy <= to_signed(C_ROW_LAND_Y(to_integer(gm_dep)), 13) + signed(resize(gm_lfsr(3 downto 0), 13));
            if gm_win = '0' and gm_bnc = '0' and gm_fvy > 9 then gm_bok <= '1'; else gm_bok <= '0'; end if;
            gm_e_mir <= gm_ori(1);
            gm_e_tr  <= gm_ori(2);
            if gm_ori(0) = '1' then gm_e_tilt <= "11"; else gm_e_tilt <= gm_lean; end if;
            if f_areq /= gm_aq and gm_mode = M_IDLE then gm_areq <= '1'; else gm_areq <= '0'; end if;
            gm_hdx     <= (s_vw + C_HOLE_PX) - gm_fx;     -- a won prize slides to the hole's x
            case gs is
                when G_IDLE =>
                    if s_vs_go = '1' then
                        gs <= G_PRE;
                    end if;
                when G_PRE =>
                    gs <= G_START;               -- event flags see this field's S7
                when G_START =>
                    gm_lfsr <= gm_lfsr(14 downto 0) & (gm_lfsr(15) xor gm_lfsr(13) xor gm_lfsr(12) xor gm_lfsr(10));
                    gm_tol  <= to_signed(20, 13) + signed(resize(f_k3(9 downto 4), 13));
                    if s_sd = '1' then
                        gm_base <= to_unsigned(64, 8); gm_np <= to_unsigned(C_NPILE_SD, 6);
                    else
                        gm_base <= to_unsigned(0, 8);  gm_np <= to_unsigned(C_NPILE_HD, 6);
                    end if;
                    gs <= G_F0;
                    gm_opq <= s_open_t;
                    -- (evc / cscan / areq / evo are mutually exclusive)
                    if gm_evc = '1' or gm_cscan = '1' then
                        gm_i  <= (others => '0');
                        gm_bv <= '0';
                        gm_by <= (others => '1');
                        gm_scanonly <= gm_cscan;     -- auto-close: look, don't grab
                        gs <= G_RD0;
                    elsif gm_areq = '1' then
                        gs <= G_A0;                  -- autopilot wants a prize to aim at
                    elsif gm_evo = '1' or gm_slip = '1' then
                        -- let go (or a rigged slip): the prize falls from the claw
                        gm_mode <= M_FALL;
                        gm_fx   <= gm_relx;
                        gm_fy   <= gm_holdy;
                        gm_fvy  <= (others => '0');
                        gm_bnc  <= '0';
                        gm_rt   <= (others => '0');
                        gm_rig  <= '0';
                        gm_win  <= gm_chute and not gm_slip;
                        if gm_chute = '1' and gm_slip = '0' then
                            gm_land <= to_signed(C_CHUTE_LAND_Y, 13);
                        else
                            gm_land <= gm_landy;
                        end if;
                    else
                        gm_cand <= '0';
                    end if;
                -- scan the pile records
                when G_RD0 =>
                    sr_a <= gm_base + shift_left(resize(gm_i, 8), 1);
                    gs <= G_RD1;
                when G_RD1 =>
                    sr_a(0) <= '1';
                    gs <= G_RW;
                when G_RW =>
                    gs <= G_RC0;
                when G_RC0 =>
                    gm_w0 <= sr_q2;
                    gs <= G_RC1;
                when G_RC1 =>
                    gm_w1  <= sr_q2;
                    gm_dxc <= s_cx - signed(resize(unsigned(gm_w0(15 downto 5)), 13)) - 96;
                    gs <= G_E1;
                when G_E1 =>
                    gm_adx   <= abs(gm_dxc);
                    gm_reach <= s_cy + C_CLAW_REACH(to_integer(s_cdepth))
                                - signed(resize(unsigned(gm_w1(15 downto 5)), 13));
                    gs <= G_E1B;
                when G_E1B =>
                    gm_ok <= '0';
                    -- (a hidden record has y = 2047: never reachable)
                    if unsigned(gm_w1(4 downto 3)) = s_cdepth
                       and gm_adx < gm_tol and gm_reach(12) = '0' and unsigned(gm_w1(15 downto 5)) < gm_by then
                        gm_ok <= '1';
                    end if;
                    gs <= G_E2;
                when G_E2 =>
                    if gm_ok = '1' then
                        gm_bv <= '1';
                        gm_bi <= gm_i;
                        gm_by <= unsigned(gm_w1(15 downto 5));
                        gm_bt <= unsigned(gm_w0(4 downto 2));
                        gm_bw1 <= gm_w1;
                        gm_bmir <= gm_w0(1);
                    end if;
                    if gm_i = gm_np - 1 then
                        gs <= G_PICK;
                    else
                        gm_i <= gm_i + 1;
                        gs <= G_RD0;
                    end if;
                when G_PICK =>
                    if gm_scanonly = '1' then
                        gm_cand <= gm_bv;            -- auto-close candidate in reach
                        gs <= G_H0;
                    elsif gm_bv = '1' then
                        gm_mode   <= M_HOLD;
                        gm_rig    <= gm_rigroll;     -- rigged: this one will slip
                        gm_sliph  <= gm_slipn;
                        gm_slot   <= gm_bi;
                        gm_type   <= gm_bt;
                        gm_dep    <= s_cdepth;
                        gm_ow1    <= gm_bw1;         -- its place in the pile, for a refill
                        -- it keeps the orientation it had in the pile
                        gm_ori(2) <= gm_bw1(0);
                        gm_ori(1) <= gm_bmir;
                        if gm_bw1(2 downto 1) = "11" then
                            gm_ori(0) <= '1'; gm_lean <= "00";
                        else
                            gm_ori(0) <= '0'; gm_lean <= gm_bw1(2 downto 1);
                        end if;
                        gs <= G_HD;
                    else
                        gs <= G_H0;
                    end if;
                -- falling prize: gravity, a quarter turn every 4 frames, one bounce
                when G_F0 =>
                    if gm_mode = M_FALL then
                        if gm_fvy < 40 then gm_fvy <= gm_fvy + 3; end if;
                        gm_rt <= gm_rt + 1;
                        gs <= G_F1;
                    else
                        gs <= G_H0;
                    end if;
                when G_F1 =>
                    gm_fy <= gm_fy + resize(gm_fvy, 13);
                    if gm_win = '1' then
                        gm_fx <= gm_fx + shift_right(gm_hdx, 2);
                    end if;
                    if gm_rt(1 downto 0) = "11" then
                        -- a quarter turn clockwise every 4 frames:
                        -- (tr, mir, vf) -> (not tr, not vf, mir)
                        gm_ori(2) <= not gm_ori(2);
                        gm_ori(1) <= not gm_ori(0);
                        gm_ori(0) <= gm_ori(1);
                        gm_lean <= "00";
                    end if;
                    gs <= G_F2;
                when G_F2 =>
                    gm_dl <= gm_fy - gm_land;
                    gs <= G_F3;
                when G_F3 =>
                    if gm_dl(12) = '0' then
                        if gm_bok = '1' then
                            gm_fy  <= gm_land;
                            gm_fvy <= -shift_right(gm_fvy, 2);
                            gm_bnc <= '1';
                            gs <= G_H0;
                        else
                            gm_fy   <= gm_land;
                            gm_mode <= M_IDLE;
                            if gm_win = '1' then
                                -- WIN: the prize stays in the hole; its pile slot refills
                                gm_wtog <= not gm_wtog;      -- marquee flash
                                gs <= G_W0;
                            else
                                gs <= G_L0;
                            end if;
                        end if;
                    else
                        gs <= G_H0;
                    end if;
                -- autopilot target: a random pile record (skip hidden ones)
                when G_A0 =>
                    gm_aq <= f_areq;
                    if gm_lfsr(4 downto 0) >= gm_np then
                        gm_ai <= resize(gm_lfsr(4 downto 0), 6) - gm_np;
                    else
                        gm_ai <= resize(gm_lfsr(4 downto 0), 6);
                    end if;
                    gs <= G_A1;
                when G_A1 =>
                    sr_a <= gm_base + shift_left(resize(gm_ai, 8), 1);
                    gs <= G_A2;
                when G_A2 =>
                    sr_a(0) <= '1';
                    gs <= G_A3;
                when G_A3 =>
                    gs <= G_A4;
                when G_A4 =>
                    gm_w0 <= sr_q2;
                    gs <= G_A5;
                when G_A5 =>
                    if sr_q2(15 downto 5) = "11111111111" then
                        gm_aq <= not f_areq;         -- hidden: try again next field
                    else
                        gm_tx  <= signed(resize(unsigned(gm_w0(15 downto 5)), 13)) + 96;
                        gm_ty  <= signed(resize(unsigned(sr_q2(15 downto 5)), 13));
                        gm_td  <= unsigned(sr_q2(4 downto 3));
                        gm_tok <= not gm_tok;
                    end if;
                    gs <= G_F0;
                -- the prize hole record (after the static control-panel synth)
                when G_W0 =>
                    gw_a  <= gm_base + shift_left(resize(gm_np, 8), 1) + 2;
                    gw_d(15 downto 5) <= std_logic_vector(resize(unsigned(s_vw + C_HOLE_PX), 11));
                    gw_d(4 downto 2)  <= std_logic_vector(gm_type);
                    gw_d(1)           <= gm_e_mir;
                    gw_d(0)           <= '0';
                    gw_we <= '1';
                    gs <= G_W1;
                when G_W1 =>
                    gw_a  <= gm_base + shift_left(resize(gm_np, 8), 1) + 3;
                    gw_d(15 downto 5) <= std_logic_vector(to_unsigned(C_HOLE_Y, 11));
                    gw_d(4 downto 3)  <= "00";
                    gw_d(2 downto 1)  <= gm_e_tilt;
                    gw_d(0)           <= gm_e_tr;
                    gw_we <= '1';
                    gm_refill <= '1';
                    gs <= G_S4;                    -- refill the won prize's pile slot
                -- landed in the pile: its slot record moves to where it fell
                when G_L0 =>
                    gw_a  <= gm_base + shift_left(resize(gm_slot, 8), 1);
                    gw_d(15 downto 5) <= std_logic_vector(resize(unsigned(gm_fx), 11));
                    gw_d(4 downto 2)  <= std_logic_vector(gm_type);
                    gw_d(1) <= gm_e_mir;
                    gw_d(0) <= '0';
                    gw_we <= '1';
                    gs <= G_L1;
                when G_L1 =>
                    gw_a  <= gm_base + shift_left(resize(gm_slot, 8), 1) + 1;
                    gw_d(15 downto 5) <= std_logic_vector(resize(unsigned(gm_land), 11));
                    gw_d(4 downto 3)  <= std_logic_vector(gm_dep);
                    gw_d(2 downto 1)  <= gm_e_tilt;
                    gw_d(0) <= gm_e_tr;
                    gw_we <= '1';
                    gs <= G_H0;
                -- hide the grabbed record in the pile: y = 2047
                when G_HD =>
                    gw_a  <= gm_base + shift_left(resize(gm_slot, 8), 1) + 1;
                    gw_d  <= "11111111111" & gm_ow1(4 downto 0);
                    gw_we <= '1';
                    gs <= G_H0;
                -- a won prize's slot comes back where it was (with its new type)
                when G_RW1 =>
                    gw_a  <= gm_base + shift_left(resize(gm_slot, 8), 1) + 1;
                    gw_d  <= gm_ow1;
                    gw_we <= '1';
                    gs <= G_H0;
                -- set the type of record gm_tgt to gm_ntype (read-modify-write w0)
                when G_S0 =>
                    sr_a <= gm_base + shift_left(resize(gm_tgt, 8), 1);
                    gs <= G_S1;
                when G_S1 =>
                    gs <= G_S2;
                when G_S2 =>
                    gs <= G_S3;
                when G_S3 =>
                    gw_a  <= gm_base + shift_left(resize(gm_tgt, 8), 1);
                    gw_d  <= sr_q2(15 downto 5) & std_logic_vector(gm_ntype) & sr_q2(1 downto 0);
                    gw_we <= '1';
                    gs <= G_S4;
                when G_S4 =>
                    if gm_refill = '1' then
                        -- the won prize's slot gets a fresh random prize
                        gm_refill <= '0';
                        gm_rst    <= '1';
                        gm_tgt    <= gm_slot;
                        gm_ntype  <= gm_lfsr(2 downto 0);
                        gs <= G_S0;
                    elsif gm_rst = '1' then
                        gm_rst <= '0';
                        gs <= G_RW1;
                    else
                        gs <= G_H0;
                    end if;
                -- the held / falling prize record (frontmost)
                when G_H0 =>
                    if gm_mode = M_FALL and gm_win = '1' then s_flap_open <= '1'; else s_flap_open <= '0'; end if;
                    gw_a <= gm_base + shift_left(resize(gm_np, 8), 1) + 4;
                    if gm_mode = M_FALL then
                        gw_d(15 downto 5) <= std_logic_vector(resize(unsigned(gm_fx), 11));
                    else
                        gw_d(15 downto 5) <= std_logic_vector(resize(unsigned(gm_relx), 11));
                    end if;
                    gw_d(1) <= gm_e_mir;
                    gw_d(4 downto 2) <= std_logic_vector(gm_type);
                    gw_d(0) <= '1';                  -- last record
                    gw_we <= '1';
                    gs <= G_H1;
                when G_H1 =>
                    gw_a <= gm_base + shift_left(resize(gm_np, 8), 1) + 5;
                    if gm_mode = M_IDLE then
                        gw_d(15 downto 5) <= (others => '1');      -- hidden
                        gw_d(2 downto 0)  <= "000";
                    elsif gm_mode = M_FALL then
                        gw_d(15 downto 5) <= std_logic_vector(resize(unsigned(gm_fy), 11));
                        gw_d(2 downto 1)  <= gm_e_tilt;
                        gw_d(0)           <= gm_e_tr;
                    else
                        gw_d(15 downto 5) <= std_logic_vector(resize(unsigned(gm_holdy), 11));
                        gw_d(2 downto 1)  <= gm_e_tilt;
                        gw_d(0)           <= gm_e_tr;
                    end if;
                    gw_d(4 downto 3) <= std_logic_vector(gm_dep);
                    gw_we <= '1';
                    gs <= G_IDLE;
            end case;

            -- stream stage b: ROM reads (registered EBR outputs)
            sp_qa  <= C_SPR_A(to_integer(sp_addr(12 downto 0)));
            sp_qb  <= C_SPR_B(to_integer(sp_addr(12 downto 0)));
            sp_qc  <= C_SPR_C(to_integer(sp_addr(10 downto 0)));
            sp_sel <= sp_addr(14 downto 13);      -- 0x A, 01 B, 1x C
            sc_b   <= sc_a;
            -- stage c: nibble select
            if sp_sel(1) = '1' then
                sp_nib <= unsigned(sp_qc);
            elsif sp_sel(0) = '1' then
                sp_nib <= unsigned(sp_qb);
            else
                sp_nib <= unsigned(sp_qa);
            end if;
            if sc_b.tail = '1' then
                sp_nib <= (others => '0');
            end if;
            sc_c <= sc_b;
            -- stage d: line-buffer entry = {depth row (7:6), master colour (5:0)}
            sp_pal(7 downto 6) <= sc_c.dep;
            sp_pal(5 downto 0) <= C_TYPEPAL(to_integer(sc_c.typ) * 16 + to_integer(sp_nib));
            if sp_nib /= 0 then sp_opq <= '1'; else sp_opq <= '0'; end if;
            sc_d <= sc_c;
            -- stage e: pair write (odd alignment: even half = previous pixel)
            st_wee <= '0';
            st_weo <= '0';
            if sc_d.v = '1' then
                st_a    <= unsigned(sc_d.pa(8 downto 0));
                st_bank <= sc_d.bank;
                if sc_d.pa(10) = '0' and ((s_sd = '1' and f_lts(sc_d.pa, 360)) or (s_sd = '0' and f_lts(sc_d.pa, 480))) then
                    if sc_d.odd = '1' then
                        st_de  <= sp_prev;
                        st_wee <= sp_prevo and not sc_d.first;
                    else
                        st_de  <= std_logic_vector(sp_pal);
                        st_wee <= sp_opq;
                    end if;
                    st_do  <= std_logic_vector(sp_pal);
                    st_weo <= sp_opq;
                end if;
                sp_prev  <= std_logic_vector(sp_pal);
                sp_prevo <= sp_opq;
            end if;
        end if;
    end process;

    p_slotw : process(clk)
    begin
        if rising_edge(clk) then
            if gw_we = '1' then
                slot_ram(to_integer(gw_a)) <= gw_d;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Pixel pipeline
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_vx  : t_c;
        variable v_o, v_i, v_h, v_s : std_logic_vector(8 downto 0);
        variable v_th  : unsigned(3 downto 0);
        variable v_pa  : unsigned(7 downto 0);
        variable v_cl  : unsigned(1 downto 0);
        variable v_bs  : std_logic_vector(1 downto 0);
        variable v_dx  : unsigned(4 downto 0);
        variable v_lb  : std_logic_vector(7 downto 0);
        variable v_lbd : unsigned(1 downto 0);
        variable v_wsh : unsigned(1 downto 0);
        variable v_ws  : unsigned(1 downto 0);
        variable v_sid : unsigned(2 downto 0);
        variable v_ci  : unsigned(2 downto 0);
        variable v_tdx : unsigned(8 downto 0);
        variable v_tv  : unsigned(1 downto 0);
        variable v_lift: unsigned(2 downto 0);
        variable v_y11 : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            v_th := s_thhi;
            -- P1: register coordinate, issue the line-buffer read
            v_vx   := signed(resize(s_xa(12 downto 1), 13));
            r1_vx  <= v_vx;
            r1_ux  <= s_xa(11 downto 1);
            r1_cbd <= resize(s_xa(11 downto 1), 12) - resize(cb_lo, 12);   -- cable: x - left edge
            r1_av  <= g_avid;
            r1_b   <= shift_left(resize(s_lc, 4), 2) + resize(s_bcol, 4);
            r1_x1  <= s_xa(2);                 -- entry parity (vx bit 1)
            r1_pa  <= s_xa(11 downto 3);       -- pair = vx >> 2
            r1_par <= s_par xor s_first;       -- the flip lands this clock
            lb_ra  <= s_xa(11 downto 3);
            -- clear behind the beam (pair granularity)
            cl_we   <= '0';
            cl_bank <= r1_par;
            if r1_av = '1' and (g_avid = '0' or s_xa(11 downto 3) /= r1_pa) then
                cl_a  <= r1_pa;
                cl_we <= '1';
            end if;

            -- P2: slot compares, text / bulb / streak operands
            for s in 0 to 7 loop
                if sl_vo(s) = '1' and r1_ux >= sl_lo(s) and r1_ux < sl_ro(s) then
                    r2_o(s) <= '1';
                else
                    r2_o(s) <= '0';
                end if;
                if sl_vi(s) = '1' and r1_ux >= sl_li(s) and r1_ux < sl_ri(s) then
                    r2_i(s) <= '1';
                else
                    r2_i(s) <= '0';
                end if;
                -- left highlight band: not on the frame, panel and flap slots
                if s /= 0 and s /= 1 and s /= 4 and r1_ux < sl_hx(s) then
                    r2_hc(s) <= '1';
                else
                    r2_hc(s) <= '0';
                end if;
            end loop;
            -- the cable (narrow, direct): d = x - left edge, outline 2, highlight 3
            r2_o(8) <= '0'; r2_i(8) <= '0'; r2_hc(8) <= '0';
            if ln_cb = '1' and r1_cbd(11 downto 4) = 0 and r1_cbd(3 downto 0) < cb_w then
                r2_o(8) <= '1';
                if r1_cbd(3 downto 0) >= C_CABLE_T and r1_cbd(3 downto 0) < cb_w - C_CABLE_T then
                    r2_i(8) <= '1';
                end if;
                if r1_cbd(3 downto 0) < C_CABLE_T + 3 then r2_hc(8) <= '1'; end if;
            end if;
            -- text zones start on 32-unit cells (title on 64): the char index
            -- is the cell number minus the zone's start cell, the column x bits
            r2_cc  <= unsigned(r1_vx(10 downto 5)) - tz_c;
            r2_cd  <= unsigned(r1_vx(10 downto 5)) - tz_d;
            r2_xlo <= unsigned(r1_vx(5 downto 1));
            r2_bdx <= signed(resize(unsigned(r1_vx(4 downto 0)), 6)) - 16;
            r2_bidx <= unsigned(r1_vx(6 downto 5)) + s_phase(4 downto 3);
            -- win: alternate bulbs swap every 2 fields, all-on bursts
            r2_bwin <= (r1_vx(5) xor s_phase(1)) or (s_phase(4) and s_phase(3));
            if not f_lts(r1_vx, C_BULB_X_MIN) and
               ((s_sd = '1' and f_lts(r1_vx, 1440 - C_BULB_X_MIN + 1)) or
                (s_sd = '0' and f_lts(r1_vx, 1920 - C_BULB_X_MIN + 1))) then
                r2_bxok <= '1';
            else
                r2_bxok <= '0';
            end if;
            r2_st  <= r1_vx + ln_stoff;
            r2_b   <= r1_b;
            r2_x1  <= r1_x1;
            r2_par <= r1_par;

            -- P3: classify; line-buffer word registered
            r3_q0e <= lb_q0e; r3_q0o <= lb_q0o; r3_q1e <= lb_q1e; r3_q1o <= lb_q1o;
            r3_b   <= r2_b;
            r3_x1  <= r2_x1;
            r3_par <= r2_par;
            for s in 0 to 8 loop
                v_o(s) := r2_o(s);
                v_i(s) := r2_i(s);
                v_h(s) := r2_i(s) and (r2_hc(s) or sl_hl(s));
                v_s(s) := r2_i(s) and sl_sh(s) and not v_h(s);
            end loop;
            -- glass streaks + acrylic lift
            v_lift := (others => '0');
            if not f_lts(r2_st, C_STREAK0_LO) and f_lts(r2_st, C_STREAK0_HI) then v_lift := to_unsigned(C_STREAK0_ST, 3); end if;
            if not f_lts(r2_st, C_STREAK1_LO) and f_lts(r2_st, C_STREAK1_HI) then v_lift := to_unsigned(C_STREAK1_ST, 3); end if;
            if not f_lts(r2_st, C_STREAK2_LO) and f_lts(r2_st, C_STREAK2_HI) then v_lift := to_unsigned(C_STREAK2_ST, 3); end if;
            r3_fon <= '0'; r3_con <= '0'; r3_mon <= '0';
            r3_fpa <= PA_OUTLINE; r3_cpa <= PA_OUTLINE; r3_mpa <= PA_OUTLINE;
            v_cl := "00";
            v_pa := f_th(v_th, C_TH_BG);
            if v_i(1) = '1' and sl_el(1) = E_WINDOW then
                v_cl := "01";
                -- front: flap, chute outline
                if v_i(3) = '1' and sl_el(3) = E_CHUTE and sl_fl = '1' and s_flap_open = '0' then
                    -- the flap: chrome bar across the chute top, outlined below
                    r3_fon <= '1';
                    if sl_fo = '1' then r3_fpa <= PA_OUTLINE;
                    elsif sl_hl(3) = '1' then r3_fpa <= PA_CHROME_HL;
                    else r3_fpa <= PA_CHROME; end if;
                elsif v_o(3) = '1' and sl_el(3) = E_CHUTE and v_i(3) = '0' then
                    r3_fon <= '1';
                    r3_fpa <= PA_OUTLINE;
                end if;
                if v_i(3) = '1' and sl_el(3) = E_CHUTE then
                    if v_h(3) = '1' then v_lift := v_lift + 5; else v_lift := v_lift + 2; end if;
                end if;
                -- claw: middle prong > hub > outer prongs > cable
                if v_o(7) = '1' then
                    r3_con <= '1';
                    r3_cpa <= f_ramp(v_o(7), v_i(7), v_h(7), v_s(7), PA_CHROME_SH, PA_CHROME, PA_CHROME_HL);
                elsif v_o(2) = '1' and sl_el(2) = E_HUB then
                    r3_con <= '1';
                    r3_cpa <= f_ramp(v_o(2), v_i(2), v_h(2), v_s(2), PA_CHROME_SH, PA_CHROME, PA_CHROME_HL);
                elsif v_o(5) = '1' then
                    r3_con <= '1';
                    r3_cpa <= f_ramp(v_o(5), v_i(5), v_h(5), v_s(5), PA_CHROME_SH, PA_CHROME, PA_CHROME_HL);
                elsif v_o(6) = '1' then
                    r3_con <= '1';
                    r3_cpa <= f_ramp(v_o(6), v_i(6), v_h(6), v_s(6), PA_CHROME_SH, PA_CHROME, PA_CHROME_HL);
                elsif v_o(8) = '1' then
                    r3_con <= '1';
                    r3_cpa <= f_ramp(v_o(8), v_i(8), v_h(8), v_s(8), PA_CHROME_SH, PA_CHROME, PA_CHROME_HL);
                end if;
                -- mid: carriage over rail
                if v_o(3) = '1' and sl_el(3) = E_CARRIAGE then
                    r3_mon <= '1';
                    r3_mpa <= f_ramp(v_o(3), v_i(3), v_h(3), v_s(3),
                                     f_th(v_th, C_TH_ACC_SH), f_th(v_th, C_TH_ACC),
                                     f_th(v_th, C_TH_ACC_HL));
                elsif v_o(2) = '1' and sl_el(2) = E_RAIL then
                    r3_mon <= '1';
                    r3_mpa <= f_ramp(v_o(2), v_i(2), v_h(2), v_s(2), PA_CHROME_SH, PA_CHROME, PA_CHROME_HL);
                end if;
            elsif v_i(1) = '1' and sl_el(1) = E_MARQ_PANEL then
                v_cl := "10";
                v_pa := f_th(v_th, C_TH_DEEP);
                v_lift := (others => '0');
            elsif v_i(1) = '1' and sl_el(1) = E_DOOR then
                v_cl := "11";
                v_pa := f_th(v_th, C_TH_DOOR);
                v_lift := (others => '0');
            elsif v_o(1) = '1' then
                v_pa := PA_OUTLINE;
                v_lift := (others => '0');
            elsif v_i(0) = '1' then
                v_lift := (others => '0');
                if sl_el(0) = E_BEZEL then
                    v_pa := f_ramp(v_o(0), v_i(0), v_h(0), v_s(0), PA_CHROME_SH, PA_CHROME, PA_CHROME_HL);
                elsif sl_el(0) = E_BASE and v_o(3) = '1' and sl_el(3) = E_HOLE then
                    -- the prize hole: won prizes land here
                    if v_i(3) = '1' then
                        v_cl := "11";
                        v_pa := f_th(v_th, C_TH_DOOR);
                    else
                        v_pa := PA_OUTLINE;
                    end if;
                elsif sl_el(0) = E_BASE and v_o(2) = '1' and sl_el(2) = E_DECAL then
                    if v_i(2) = '1' then v_cl := "10"; end if;
                    if s_dflash = '1' then
                        v_pa := f_ramp(v_o(2), v_i(2), v_h(2), v_s(2), PA_WHITE, PA_WHITE, PA_WHITE);
                    else
                        v_pa := f_ramp(v_o(2), v_i(2), v_h(2), v_s(2),
                                       f_th(v_th, C_TH_ACC_SH), f_th(v_th, C_TH_ACC),
                                       f_th(v_th, C_TH_ACC_HL));
                    end if;
                else
                    v_pa := f_ramp(v_o(0), v_i(0), v_h(0), v_s(0),
                                   f_th(v_th, C_TH_BASE_SH), f_th(v_th, C_TH_BASE),
                                   f_th(v_th, C_TH_BASE_HL));
                end if;
            elsif v_o(0) = '1' then
                v_pa := PA_OUTLINE;
                v_lift := (others => '0');
            else
                v_lift := (others => '0');
            end if;
            -- the joystick stands on the prize-door panel, in front of everything
            if v_o(2) = '1' and sl_el(2) = E_JOY_BALL then
                v_cl := "00"; v_lift := (others => '0');
                v_pa := f_ramp(v_o(2), v_i(2), v_h(2), v_s(2), PA_RED_SH, PA_RED, PA_RED_HL);
            elsif v_o(5) = '1' and sl_el(5) = E_JOY_STICK then
                v_cl := "00"; v_lift := (others => '0');
                v_pa := f_ramp(v_o(5), v_i(5), v_h(5), v_s(5), PA_CHROME_SH, PA_CHROME, PA_CHROME_HL);
            end if;
            r3_cls  <= v_cl;
            r3_pa   <= v_pa;
            r3_lift <= v_lift;
            -- bulbs (marquee frame only)
            v_dx := unsigned(abs(r2_bdx(4 downto 0)));
            v_bs := C_BULB(to_integer(v_dx(2 downto 0)) * 8 + to_integer(ln_bady));
            r3_bulb <= '0';
            if ln_bulb = '1' and r2_bxok = '1' and v_i(0) = '1' and sl_el(0) = E_MARQ_OUT
               and v_o(1) = '0' and v_dx(4 downto 3) = "00" and v_bs /= "00" then
                r3_bulb <= '1';
            end if;
            if v_bs(1) = '0' then
                r3_bpa <= PA_CHROME_SH;
            elsif ln_bdyhl = '1' and not f_lts(r2_bdx, -4) and f_lts(r2_bdx, -1) then
                r3_bpa <= PA_WHITE;
            elsif (s_winfx = '0' and r2_bidx(1) = '0') or (s_winfx = '1' and r2_bwin = '1') then
                r3_bpa <= PA_BULB_ON;
            else
                r3_bpa <= PA_BULB_OFF;
            end if;
            -- text zone
            r3_tsel <= (others => '0');
            r3_xlo <= r2_xlo;
            -- (v0.5.0: the CREDITS / TRIES / WINS scoreboard is gone; title + DROP)
            if ln_tc = '1' and r2_cc(5 downto 3) = "000" then
                r3_tsel <= "010"; r3_tci <= r2_cc(2 downto 0);
            elsif ln_td = '1' and r2_cd(5 downto 2) = "0000" then
                r3_tsel <= "100"; r3_tci <= r2_cd(2 downto 0);
            end if;

            -- P4: merge bulbs, wall colour, line-buffer pick, string lookup
            r4_cls <= r3_cls;
            if r3_bulb = '1' then r4_pa <= r3_bpa; else r4_pa <= r3_pa; end if;
            r4_fon <= r3_fon; r4_fpa <= r3_fpa;
            r4_con <= r3_con; r4_cpa <= r3_cpa;
            r4_mon <= r3_mon; r4_mpa <= r3_mpa;
            r4_lift <= r3_lift;
            -- back wall: 4-shade dithered gradient, dark heap interior below
            if ln_dark = '1' and (ln_dfr(4) = '1' or C_BAYER(to_integer(r3_b)) < ln_dfr(3 downto 0)) then
                r4_wpa  <= f_th(v_th, C_TH_DOOR);
                r4_wall <= '0';
            else
                if ln_dark = '1' then
                    v_ws := "11";
                elsif C_BAYER(to_integer(r3_b)) < ln_wfr then
                    v_ws := ln_wsh + 1;
                else
                    v_ws := ln_wsh;
                end if;
                r4_wpa  <= f_th(v_th, C_TH_WALL0 + to_integer(v_ws));
                r4_wall <= '1';
            end if;
            if r3_par = '0' then
                if r3_x1 = '0' then v_lb := r3_q0e; else v_lb := r3_q0o; end if;
            else
                if r3_x1 = '0' then v_lb := r3_q1e; else v_lb := r3_q1o; end if;
            end if;
            r4_lb <= unsigned(v_lb);
            -- string / glyph
            r4_ton <= '0';
            r4_tid <= "01";
            case r3_tsel is
                when "010" => v_sid := "000"; r4_row <= ln_rowc; v_ci := resize(r3_tci(2 downto 1), 3); r4_col <= r3_xlo(4 downto 1); r4_ton <= '1'; r4_tid <= "00";
                when "100" => v_sid := "101"; r4_row <= ln_rowd; v_ci := r3_tci; r4_col <= r3_xlo(3 downto 0); r4_ton <= '1'; r4_tid <= "10";
                when others => v_sid := "000"; v_ci := "000"; r4_row <= (others => '0'); r4_col <= (others => '0');
            end case;
            r4_code <= C_STR(to_integer(v_sid) * 8 + to_integer(v_ci));

            -- P5: font address; window / door composite (the line-buffer
            -- word is here, so only one palette address rides on)
            f5_addr <= f_faddr(r4_code, r4_row, r4_col(3));
            d_wall(5) <= '0';
            d_hz(5)   <= "00";
            v_lbd := r4_lb(7 downto 6);                  -- the prize's depth row
            case r4_cls is
                when "01" =>
                    if r4_fon = '1' then
                        d_pa(5) <= r4_fpa;
                    elsif r4_con = '1' and (r4_lb = 0 or v_lbd >= s_cdepth) then
                        d_pa(5) <= r4_cpa;
                    elsif r4_lb /= 0 then
                        d_pa(5) <= "00" & r4_lb(5 downto 0);
                        d_hz(5) <= v_lbd;                -- depth haze, applied after the palette
                    elsif r4_mon = '1' then
                        d_pa(5) <= r4_mpa;
                    else
                        d_pa(5)   <= r4_wpa;
                        d_wall(5) <= r4_wall;
                    end if;
                when "11" =>
                    if r4_lb /= 0 then d_pa(5) <= "00" & r4_lb(5 downto 0); else d_pa(5) <= r4_pa; end if;
                when others =>
                    d_pa(5) <= r4_pa;
            end case;
            if r4_cls = "10" then d_ton(5) <= r4_ton; else d_ton(5) <= '0'; end if;
            d_tid(5) <= r4_tid; d_lift(5) <= r4_lift; d_col(5) <= r4_col;
            -- P6: font EBR, P7: fabric
            f6_q <= C_FONT(to_integer(f5_addr));
            f7_q <= f6_q;
            for k in 6 to 7 loop
                d_pa(k) <= d_pa(k - 1); d_tid(k) <= d_tid(k - 1); d_lift(k) <= d_lift(k - 1);
                d_col(k) <= d_col(k - 1); d_wall(k) <= d_wall(k - 1); d_ton(k) <= d_ton(k - 1);
                d_hz(k) <= d_hz(k - 1);
            end loop;

            -- P8: text pixel value
            case d_col(7)(2 downto 0) is
                when "000" => v_tv := unsigned(f7_q(15 downto 14));
                when "001" => v_tv := unsigned(f7_q(13 downto 12));
                when "010" => v_tv := unsigned(f7_q(11 downto 10));
                when "011" => v_tv := unsigned(f7_q(9 downto 8));
                when "100" => v_tv := unsigned(f7_q(7 downto 6));
                when "101" => v_tv := unsigned(f7_q(5 downto 4));
                when "110" => v_tv := unsigned(f7_q(3 downto 2));
                when others => v_tv := unsigned(f7_q(1 downto 0));
            end case;
            if d_ton(7) = '0' then v_tv := "00"; end if;
            r8_tv <= v_tv;
            r8_pa <= d_pa(7); r8_tid <= d_tid(7); r8_lift <= d_lift(7); r8_wall <= d_wall(7);
            r8_hz <= d_hz(7);

            -- P9: text over its panel
            r9_lift <= r8_lift;
            r9_wall <= r8_wall;
            r9_hz   <= r8_hz;
            if r8_tv = "00" then
                r9_pa <= r8_pa;
            elsif r8_tv = "01" then
                r9_pa <= PA_OUTLINE;
            elsif r8_tid = "00" then
                if r8_tv = "10" then r9_pa <= f_th(s_thhi, C_TH_TITLE);
                else r9_pa <= f_th(s_thhi, C_TH_TITLE_HL); end if;
            elsif r8_tid = "10" then
                if s_dflash = '1' then r9_pa <= PA_RED; else r9_pa <= PA_WHITE; end if;
            else
                if r8_tv = "10" then r9_pa <= PA_BULB_ON; else r9_pa <= PA_WHITE; end if;
            end if;

            -- P10: palette EBR, P11: fabric
            p10_q <= C_CLAW_PAL(to_integer(r9_pa));
            r10_lift <= r9_lift; r10_wall <= r9_wall; r10_hz <= r9_hz;
            r11_q <= p10_q;
            r11_lift <= r10_lift; r11_wall <= r10_wall; r11_hz <= r10_hz;


            -- P12: lift (glass / acrylic), back-wall video
            if r11_wall = '1' and s_vid_on = '1' then
                v_y11 := resize(to_unsigned(32, 10) + shift_right(v_y, 1), 11) + shift_left(resize(r11_lift, 11), 3);
                r12_u <= to_unsigned(256, 10) + shift_right(v_u, 1);
                r12_v <= to_unsigned(256, 10) + shift_right(v_v, 1);
            else
                v_y11 := resize(unsigned(r11_q(29 downto 20)), 11) + shift_left(resize(r11_lift, 11), 4);
                r12_u <= unsigned(r11_q(19 downto 10));
                r12_v <= unsigned(r11_q(9 downto 0));
            end if;
            if v_y11 > 1000 then r12_y <= to_unsigned(1000, 10); else r12_y <= v_y11(9 downto 0); end if;

            -- P13: [1 2 1] window; P14: filtered centre
            c_y0 <= r12_y; c_y1 <= c_y0;
            c_u0 <= r12_u; c_u1 <= c_u0;
            c_v0 <= r12_v; c_v1 <= c_v0;
            r14_y <= resize(shift_right(resize(r12_y, 12) + shift_left(resize(c_y0, 12), 1) + resize(c_y1, 12), 2), 10);
            r14_u <= resize(shift_right(resize(r12_u, 12) + shift_left(resize(c_u0, 12), 1) + resize(c_u1, 12), 2), 10);
            r14_v <= resize(shift_right(resize(r12_v, 12) + shift_left(resize(c_v0, 12), 1) + resize(c_v1, 12), 2), 10);
            -- P15: luma toward black (white ~775), blanking gate, U/V swap
            if d_gv(C_PIX - 2) = '0' then
                r15_y <= to_unsigned(64, 10);
                r15_u <= to_unsigned(512, 10);
                r15_v <= to_unsigned(512, 10);
            else
                if not f_ltu(r14_y, 65) then
                    r15_y <= r14_y - shift_right(r14_y - 64, 3) - shift_right(r14_y - 64, 4);
                else
                    r15_y <= r14_y;
                end if;
                if C_HW_UV_SWAP then
                    r15_u <= r14_v; r15_v <= r14_u;
                else
                    r15_u <= r14_u; r15_v <= r14_v;
                end if;
            end if;
        end if;
    end process;

    -- back-wall video tap: 2 clocks deep, black outside the source avid
    p_vid : process(clk)
    begin
        if rising_edge(clk) then
            v_y0 <= data_in.y; v_u0 <= data_in.u; v_v0 <= data_in.v;
            if s_sync(0)(2) = '1' then
                -- v0.5.0: swap U/V at the input tap too. The P15 output swap
                -- (needed by the generated colours) also hit the camera video,
                -- so green came out yellow and red came out blue (Ron, HW).
                v_y <= unsigned(v_y0); v_u <= unsigned(v_v0); v_v <= unsigned(v_u0);
            else
                v_y <= to_unsigned(64, 10); v_u <= to_unsigned(512, 10); v_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Sync delay and output
    ---------------------------------------------------------------------------
    p_out : process(clk)
    begin
        if rising_edge(clk) then
            s_sync(0) <= data_in.field_n & data_in.avid & data_in.vsync_n & data_in.hsync_n;
            for i in 1 to C_LAT - 1 loop
                s_sync(i) <= s_sync(i - 1);
            end loop;
            d_gv(0) <= g_avid;
            for i in 1 to C_PIX loop
                d_gv(i) <= d_gv(i - 1);
            end loop;
            -- P16: plain output delay (keeps C_PIX = 16: see the constant)
            r16_y <= r15_y; r16_u <= r15_u; r16_v <= r15_v;
            s_out.y       <= std_logic_vector(r16_y);
            s_out.u       <= std_logic_vector(r16_u);
            s_out.v       <= std_logic_vector(r16_v);
            s_out.avid    <= d_gv(C_PIX);
            s_out.hsync_n <= s_sync(C_LAT - 2)(0);
            s_out.vsync_n <= s_sync(C_LAT - 2)(1);
            s_out.field_n <= s_sync(C_LAT - 2)(3);
        end if;
    end process;

    data_out <= s_out;

end architecture claw;
