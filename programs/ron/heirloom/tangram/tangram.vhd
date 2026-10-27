-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Tangram v0.5.0 -- Heirloom #9, rebuilt from the photo WIN_20230222_00_06_47.
--
-- The photo is an ADDITIVE RGB patch: each colour channel carries one oscillator layer.
--   R  hot-pink stripe fans: a log-chirped stripe phase (thin dense lines on one side, each next stripe
--      ~1.4x wider), two fans meeting in "<" elbows, keyed out of a top-centre triangle (key lines
--      x = 274.5 + 1.078 y and x = 2108.6 - 0.739 y), a torn top-left band and a glitch band at
--      y 626..643, smeared by an RC low-pass (rise 3/16, fall 1/8 per 1080-line pixel).
--   G  sawtooth ramps: a stepped plateau (bands 100..297 / 298..548 / 549..610) whose drops zig-zag
--      per line, and lower sawtooth stripes rising to the right (phase = A(y) + 1.8236 x / 1000).
--   B  navy base + a bright wedge |x - 996| < hw(y) and a lower V, each with a dark fringe outside its
--      edge and stripes parallel to it (measured profiles, 74 / 83 px sub-periods).
-- The pink phase is a quadratic B-spline table (64 px x 64 lines) fitted to 8779 stripe samples
-- (rms 0.013 stripe): a per-line coefficient engine blends three table rows (weights pre-scaled by the
-- Fan) into 32 coefficients and writes (P, Q, C) per 64 px segment; the pixel evaluates P + Q g + C g^2.
-- The plateau is a per-line breakpoint sawtooth (8 curves per band from a ROM); the blue layer and the
-- lower ramps are closed form.  A microcoded engine (one ALU, register file and program in block RAM,
-- Booth multiplier, serial divider; tangram.py UC) runs a field program in vblank (knob glides and
-- curves, the curve pointers) and a line program one line ahead of the beam (spline weights, the
-- wedge / ramp / key polynomials, the plateau breakpoints and their reciprocals), whose parameter block
-- XFER copies into the pixel path's line registers in hblank.
-- Composite RGB = M [r g b 1] (fitted) folded into three RGB -> YUV ROMs (hardware convention u = Cr).
--
--   K1  Width   pink stripe width (duty 0.08 .. 0.9, photo 0.46)
--   K2  Wedge   blue wedge opening x0.42 .. x2.4 (bipolar, centre = the photo)
--   K3  Zigzag  plateau zig-zag depth x0 .. x2 (0 = straight edges)
--   K4  Speed   bipolar flow: pink fans stream, green ramps scroll, blue rays flow (still at the centre)
--   K5  Ramps   lower green sawtooth count x0.5 .. x2.9
--   K6  Bend    how far the source (camera luma / Osc) bends the pink stripes and lower ramps
--   S7  Tear    torn top-left band + glitch band (default On)
--   S8  Key     centre-triangle key + right cut (default On)
--   S9  Palette Photo / Rotate (pink fans -> green, green ramps -> blue, blue wedge -> red)
--   S10 Mirror  pink and green layers mirrored about the centre
--   S11 Source  Video / Osc
--   P12 FAN     pink stripe count x0.46 .. x4.2: broad bands .. photo (35 %) .. dense fine fans
--
-- Shell (escalator v0.5): timing-ID virtual-1080 steps, vblank zone, generated avid, output
-- conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma to ~777) before the gate.
-- Latency 28 clocks in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.tangram_pkg.all;
use work.all;

architecture tangram of program_top is
    constant C_LAT : integer := 28;   -- matches processing_delay_clks
    constant C_XC8 : unsigned(18 downto 0) := to_unsigned(255027, 19);   -- 996.2 px, Q.8
    constant C_XC2 : unsigned(19 downto 0) := to_unsigned(510054, 20);
    constant C_XH8 : unsigned(18 downto 0) := to_unsigned(255232, 19);   -- 997 px (lower V axis)

    -- engine opcodes (tangram.py UC; .heirloom_work/rb_tangram/mc2.py)
    constant OP_LD   : integer := 1;  constant OP_ADD  : integer := 2;  constant OP_SUB  : integer := 3;
    constant OP_RSB  : integer := 4;  constant OP_MAX  : integer := 5;  constant OP_MIN  : integer := 6;
    constant OP_MUL  : integer := 7;  constant OP_MULL : integer := 8;  constant OP_DIV  : integer := 9;
    constant OP_ST   : integer := 10; constant OP_OUT  : integer := 11; constant OP_SHR4 : integer := 12;
    constant OP_GRA  : integer := 13; constant OP_SETX : integer := 14; constant OP_DJNZ : integer := 15;
    constant OP_BR   : integer := 16; constant OP_SLR  : integer := 17; constant OP_RET  : integer := 18;
    constant OP_JLINE : integer := 19; constant OP_KICK : integer := 20; constant OP_HALT : integer := 21;
    constant OP_XFER : integer := 22;

    -- timing
    signal t_xs  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal t_ys  : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal t_kax : unsigned(7 downto 0) := to_unsigned(57, 8);
    signal t_kbx : unsigned(7 downto 0) := to_unsigned(15, 8);
    signal t_kay : unsigned(7 downto 0) := to_unsigned(21, 8);
    signal t_kby : unsigned(9 downto 0) := to_unsigned(125, 10);

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq, s_avr : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal f_tear, f_key, f_pal, f_mir, f_osc : std_logic := '0';
    signal f_sd : std_logic := '0';                                   -- SD pixel width (faster smear)

    -- per-field values (engine OUT ports, hardwired phase accumulators)
    signal f_dutyh  : unsigned(7 downto 0) := to_unsigned(57, 8);
    signal f_bk     : unsigned(10 downto 0) := (others => '0');
    signal f_isl    : unsigned(10 downto 0) := to_unsigned(30, 11);   -- lower-ramp increment per pixel, Q.6
    signal f_thl    : signed(15 downto 0) := to_signed(-128, 16);     -- key thresholds on the scaled phase
    signal f_thh    : signed(15 downto 0) := to_signed(1664, 16);
    signal f_flowr  : unsigned(9 downto 0) := (others => '0');       -- flow phase (stripe / 256 units)
    signal f_pa, f_pb : unsigned(15 downto 0) := (others => '0');     -- Osc phases: field start, line start
    signal o_sa, o_sb : unsigned(15 downto 0) := (others => '0');
    signal u_z      : signed(15 downto 0) := (others => '0');         -- plateau zig offset (Q.4 px)
    type t_ws is array(0 to 5) of unsigned(12 downto 0);
    signal r_ws     : t_ws := (others => (others => '0'));            -- fan-scaled spline weights

    -- prepared line (engine inputs) and drawn line
    signal e_vy   : unsigned(18 downto 0) := (others => '0');       -- virtual line being prepared, Q.8
    signal e_f, e_ft : unsigned(7 downto 0) := (others => '0');
    signal e_rt   : unsigned(1 downto 0) := (others => '0');
    signal e_tb, e_tb2 : std_logic := '0';
    signal e_pb5  : unsigned(4 downto 0) := (others => '0');
    signal d_bank : std_logic := '0';

    -- microcoded engine: F (u_pc) -> D (u_ir, block RAM out) -> O (u_io) -> E (u_ie)
    signal rf     : t_g16 := C_RFI;
    attribute ram_style : string;
    attribute ram_style of rf : signal is "block";
    attribute no_rw_check : boolean;
    attribute no_rw_check of rf : signal is true;
    signal u_pc   : unsigned(8 downto 0) := (others => '0');
    signal u_run, u_pend : std_logic := '0';
    signal u_ir   : std_logic_vector(15 downto 0) := (others => '0');
    signal u_io, u_ie : std_logic_vector(15 downto 0) := (others => '0');
    signal u_vd, u_vo, u_ve : std_logic := '0';
    signal u_adv, u_fin, u_re, u_we : std_logic;
    signal u_ra, u_ra_d, u_wa : unsigned(7 downto 0);
    signal u_rfq  : std_logic_vector(15 downto 0) := (others => '0');
    signal u_inq  : signed(15 downto 0) := (others => '0');
    signal u_opb  : signed(19 downto 0) := (others => '0');
    signal u_acc  : signed(19 downto 0) := (others => '0');
    signal u_x    : unsigned(5 downto 0) := (others => '0');
    signal u_lr   : unsigned(8 downto 0) := (others => '0');
    signal u_ec   : unsigned(4 downto 0) := (others => '0');
    signal u_mlt  : std_logic := '0';
    signal u_h    : signed(21 downto 0) := (others => '0');          -- multiplier high / divider remainder
    signal u_l    : std_logic_vector(15 downto 0) := (others => '0');
    signal u_bp   : std_logic := '0';
    signal e_op   : integer range 0 to 31;
    signal rom_a  : unsigned(7 downto 0) := (others => '0');
    signal rom_gy, rom_gx, rom_gs : std_logic_vector(15 downto 0) := (others => '0');
    signal x_run, x_d1, x_w : std_logic := '0';
    signal x_i, x_i1 : unsigned(4 downto 0) := (others => '0');
    signal x_lc   : unsigned(4 downto 0) := (others => '0');

    -- line registers (XFER) and line flags (from d_vy)
    signal l_hw   : signed(15 downto 0) := (others => '0');          -- wedge half-width, Q.4
    signal l_ga   : signed(15 downto 0) := (others => '0');
    signal l_l1, l_l2 : unsigned(11 downto 0) := (others => '0');
    signal l_dph  : unsigned(6 downto 0) := (others => '0');
    type t_gb is array(0 to 6) of signed(12 downto 0);
    signal l_gb   : t_gb := (others => (others => '0'));
    type t_inc is array(0 to 4) of unsigned(10 downto 0);
    signal l_inc  : t_inc := (others => (others => '0'));           -- plateau ramp increments per pixel, Q.6
    signal l_dot, l_gl, l_tb, l_sep : std_logic := '0';
    signal l_goff : unsigned(13 downto 0) := (others => '0');
    signal l_gm   : unsigned(2 downto 0) := (others => '0');
    signal l_vyq  : unsigned(14 downto 0) := (others => '0');

    -- pink line engine: 32 B-spline coefficients (3 taps each, a 14 x 4 multiplier over the weights' four
    -- digits, most significant first) -> the spline sampled every 32 px -> (P, Q) chords into a ping-pong RAM
    signal r_go, r_run : std_logic := '0';
    signal r_k  : unsigned(5 downto 0) := (others => '0');
    signal r_i  : unsigned(1 downto 0) := (others => '0');       -- weight digit (3 = most significant)
    signal r_j  : unsigned(1 downto 0) := (others => '0');       -- tap
    signal r_rr : unsigned(4 downto 0) := (others => '0');
    signal r_rt : unsigned(1 downto 0) := (others => '0');
    signal r_tb, r_tb2, r_bank : std_logic := '0';
    signal r_ad : unsigned(9 downto 0) := (others => '0');
    signal r_q1 : signed(13 downto 0) := (others => '0');
    type t_d4 is array(1 to 3) of unsigned(3 downto 0);
    signal r_d  : t_d4 := (others => (others => '0'));
    signal r_v, r_f, r_s, r_e : std_logic_vector(1 to 4) := (others => '0');   -- valid, new digit, start, end
    signal r_pp : signed(17 downto 0) := (others => '0');
    signal r_acc : signed(27 downto 0) := (others => '0');
    signal r_a1, r_a2 : signed(15 downto 0) := (others => '0');
    signal r_na : unsigned(5 downto 0) := (others => '0');
    signal r_an : signed(15 downto 0) := (others => '0');          -- new coefficient
    signal r_av, r_bv, r_cv, r_dv : std_logic := '0';
    signal r_b0, r_b2 : signed(16 downto 0) := (others => '0');    -- 2 s(2k), 2 s(2k+2)
    signal r_bq, r_ca : signed(15 downto 0) := (others => '0');    -- a_k+1
    signal r_cs : signed(17 downto 0) := (others => '0');          -- a_k + 2 a_k+1 + a_k+2
    signal r_c0, r_c2, r_d0, r_d2, r_d1 : signed(16 downto 0) := (others => '0');
    signal r_s1 : signed(15 downto 0) := (others => '0');
    signal r_q0, r_q2 : signed(13 downto 0) := (others => '0');
    signal r_wph : std_logic_vector(1 downto 0) := (others => '0');
    signal r_seg : unsigned(5 downto 0) := (others => '0');
    signal r_we : std_logic := '0';
    signal r_wa : unsigned(6 downto 0) := (others => '0');
    signal r_wp : signed(15 downto 0) := (others => '0');
    signal r_wq : signed(13 downto 0) := (others => '0');
    type t_ramp is array(0 to 127) of signed(15 downto 0);
    type t_ramq is array(0 to 127) of signed(13 downto 0);
    signal ram_p : t_ramp := (others => (others => '0'));
    signal ram_q : t_ramq := (others => (others => '0'));
    signal r_ra : unsigned(6 downto 0) := (others => '0');
    signal ram_pq : signed(15 downto 0) := (others => '0');
    signal ram_qq : signed(13 downto 0) := (others => '0');
    attribute no_rw_check of ram_p : signal is true;
    attribute no_rw_check of ram_q : signal is true;

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(1 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- beam
    signal b_vx : unsigned(18 downto 0) := (others => '0');
    signal b_pa, b_pb : unsigned(15 downto 0) := (others => '0');
    signal v_y1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal v_src : signed(8 downto 0) := (others => '0');

    -- pixel pipeline (stage numbers in the names)
    signal p1_vxm : unsigned(18 downto 0) := (others => '0');
    signal p1_dx  : unsigned(14 downto 0) := (others => '0');       -- |x - 996.2|, Q.4
    signal p1_dc  : std_logic := '0';                               -- dot column
    signal p1_h   : unsigned(9 downto 0) := (others => '0');        -- |x - 997| px
    signal p1_lf  : unsigned(6 downto 0) := (others => '0');
    signal p1_ta, p1_tb : signed(8 downto 0) := (others => '0');
    signal p1_vs  : signed(8 downto 0) := (others => '0');
    signal p_rev  : std_logic_vector(1 to 4) := (others => '0');     -- mirrored half: x runs backwards
    signal p2_vxg : unsigned(18 downto 0) := (others => '0');
    signal p2_xi  : unsigned(10 downto 0) := (others => '0');
    signal p2_wi  : signed(11 downto 0) := (others => '0');
    signal p2_lf  : unsigned(6 downto 0) := (others => '0');
    signal p2_src : signed(8 downto 0) := (others => '0');
    signal r_yea  : unsigned(8 downto 0) := (others => '0');
    signal r_yeq  : std_logic_vector(15 downto 0) := (others => '0');
    signal p3_g5  : unsigned(4 downto 0) := (others => '0');
    signal p3_xi  : unsigned(10 downto 0) := (others => '0');
    signal p3_ge  : std_logic_vector(3 downto 0) := (others => '0');
    signal p3_gtr, p3_ltl : std_logic := '0';
    signal p3_kf  : std_logic_vector(2 downto 0) := (others => '0');   -- key flags: x >= l2, x > l1, torn column
    signal p3_wv  : signed(12 downto 0) := (others => '0');          -- wedge: distance - 30 (period part)
    signal p3_we  : unsigned(6 downto 0) := (others => '0');          --        edge-part address
    signal p3_wlo, p3_wcl, p3_wn : std_logic := '0';
    signal p4_wm  : unsigned(7 downto 0) := (others => '0');
    signal p4_we  : unsigned(6 downto 0) := (others => '0');
    signal p4_wlo, p4_wcl : std_logic := '0';
    signal p5_aw  : unsigned(8 downto 0) := (others => '0');
    signal p3_lf  : unsigned(6 downto 0) := (others => '0');
    signal p3_br  : signed(19 downto 0) := (others => '0');
    signal p4_g5  : unsigned(4 downto 0) := (others => '0');
    signal p4_seg : signed(2 downto 0) := (others => '0');
    signal p4_inc : unsigned(10 downto 0) := (others => '0');
    signal p4_off : std_logic := '0';
    signal p4_ye  : signed(15 downto 0) := (others => '0');
    signal p4_lf  : unsigned(6 downto 0) := (others => '0');
    signal p4_br, p4_bg : signed(9 downto 0) := (others => '0');
    signal p5_p   : signed(15 downto 0) := (others => '0');
    signal p5_q   : signed(13 downto 0) := (others => '0');
    signal p5_g5  : unsigned(4 downto 0) := (others => '0');
    signal p5_seg : signed(2 downto 0) := (others => '0');
    signal p5_off : std_logic := '0';
    signal p5_acc : unsigned(17 downto 0) := (others => '0');       -- ramp phase DDA, Q.6
    signal p5_ye  : signed(15 downto 0) := (others => '0');
    signal p5_br, p5_bg : signed(9 downto 0) := (others => '0');
    signal p6_p   : signed(15 downto 0) := (others => '0');
    signal p6_qg  : signed(19 downto 0) := (others => '0');
    signal p6_seg : signed(2 downto 0) := (others => '0');
    signal p6_off : std_logic := '0';
    signal p6_pr  : unsigned(14 downto 0) := (others => '0');
    signal p6_vi  : signed(11 downto 0) := (others => '0');          -- lower V: line - edge, px
    signal p7_vv  : signed(12 downto 0) := (others => '0');
    signal p7_ve  : unsigned(6 downto 0) := (others => '0');
    signal p7_vlo, p7_vcl, p7_vn : std_logic := '0';
    signal p8_vm  : unsigned(7 downto 0) := (others => '0');
    signal p8_ve  : unsigned(6 downto 0) := (others => '0');
    signal p8_vlo, p8_vcl : std_logic := '0';
    signal p9_av  : unsigned(8 downto 0) := (others => '0');
    signal p6_br, p6_bg : signed(9 downto 0) := (others => '0');
    signal p7_phi : signed(15 downto 0) := (others => '0');
    signal p7_gph : signed(15 downto 0) := (others => '0');
    signal p7_off : std_logic := '0';
    signal p7_br  : signed(9 downto 0) := (others => '0');
    signal p8_pr  : unsigned(15 downto 0) := (others => '0');
    signal p8_ok  : std_logic := '0';                                -- phase-range / key gate
    signal p8_gf  : unsigned(7 downto 0) := (others => '0');
    signal p8_off : std_logic := '0';
    signal p9_on  : std_logic := '0';
    signal p9_l0, p9_l1 : unsigned(7 downto 0) := (others => '0');
    signal p9_off : std_logic := '0';
    signal rb_w, rb_v : std_logic_vector(7 downto 0) := (others => '0');
    signal rl3_q  : std_logic_vector(7 downto 0) := (others => '0');
    signal p10_l0, p10_l1, p10_l3 : unsigned(7 downto 0) := (others => '0');
    signal p10_off : std_logic := '0';
    signal r_iir  : unsigned(15 downto 0) := (others => '0');
    signal p11_glv : unsigned(7 downto 0) := (others => '0');
    signal p11_r8 : unsigned(7 downto 0) := (others => '0');
    signal g_iir  : unsigned(7 downto 0) := (others => '0');
    signal p12_r8 : unsigned(7 downto 0) := (others => '0');
    -- blue pipe (ROM outputs re-registered, then delayed to the colour stage)
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    signal p_bw : t_u8a(7 to 12) := (others => (others => '0'));
    signal p_bv : t_u8a(11 to 12) := (others => (others => '0'));
    type t_sl is array (natural range <>) of std_logic;
    signal p_wn : t_sl(4 to 12) := (others => '0');
    signal p_vn : t_sl(8 to 12) := (others => '0');
    signal p_dot : t_sl(2 to 17) := (others => '0');
    type t_kf is array (natural range <>) of std_logic_vector(2 downto 0);
    signal p_k : t_kf(4 to 7) := (others => (others => '0'));
    signal p13_b8 : unsigned(7 downto 0) := (others => '0');
    signal p13_r8 : unsigned(7 downto 0) := (others => '0');
    signal p13_g8 : unsigned(7 downto 0) := (others => '0');
    signal p14_x0, p14_x1, p14_x2 : unsigned(7 downto 0) := (others => '0');
    type t_cyq is array(0 to 2) of std_logic_vector(9 downto 0);
    type t_cuvq is array(0 to 2) of std_logic_vector(15 downto 0);
    signal rc_y : t_cyq := (others => (others => '0'));
    signal rc_uv : t_cuvq := (others => (others => '0'));
    signal p16_y : t_cyq := (others => (others => '0'));
    signal p16_uv : t_cuvq := (others => (others => '0'));
    signal p17_y : unsigned(11 downto 0) := (others => '0');
    signal p17_u, p17_v : signed(11 downto 0) := (others => '0');
    signal p18_y, p18_u, p18_v : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- output conditioning
    type t_cd is array(0 to 255) of std_logic_vector(15 downto 0);
    signal cd_ram : t_cd := (others => x"8080");
    attribute ram_style of cd_ram : signal is "block";
    attribute no_rw_check of cd_ram : signal is true;
    signal cd_q   : std_logic_vector(15 downto 0) := x"8080";
    type t_sd is array(0 to 255) of std_logic_vector(3 downto 0);
    signal sd_ram : t_sd := (others => "1111");
    attribute ram_style of sd_ram : signal is "block";
    attribute no_rw_check of sd_ram : signal is true;
    signal sd_q   : std_logic_vector(3 downto 0) := "1111";
    signal dl_wa  : unsigned(7 downto 0) := (others => '0');
    signal dl_rc  : unsigned(7 downto 0) := to_unsigned(249, 8);   -- dl_wa - 7: chroma 8 clocks back
    signal dl_rs  : unsigned(7 downto 0) := to_unsigned(229, 8);   -- dl_wa - 27: syncs 28 clocks back
    signal s_c8   : std_logic_vector(15 downto 0) := x"8080";
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_y19, s_y20, s_y21, s_y22, s_y23, s_y24x, s_yf24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to 28) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

    -- ROM outputs
    signal rt_q : std_logic_vector(15 downto 0) := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- ROMs (registered address -> registered data; consumers add a fabric register)
    ---------------------------------------------------------------------------
    p_rom_rt : process(clk) begin
        if rising_edge(clk) then rt_q <= C_RT(to_integer(r_ad)); end if;
    end process;
    p_rom_g : process(clk) begin
        if rising_edge(clk) then
            rom_gy <= C_GY16(to_integer(rom_a)); rom_gx <= C_GX16(to_integer(rom_a)); rom_gs <= C_GS(to_integer(rom_a));
        end if;
    end process;
    p_rom_ye : process(clk) begin
        if rising_edge(clk) then r_yeq <= C_YE(to_integer(r_yea)); end if;
    end process;
    p_rom_b : process(clk) begin
        if rising_edge(clk) then rb_w <= C_BW(to_integer(p5_aw)); rb_v <= C_BV(to_integer(p9_av)); end if;
    end process;
    p_rom_l3 : process(clk) begin
        if rising_edge(clk) then rl3_q <= C_L3(to_integer(p8_gf)); end if;
    end process;
    p_rom_c : process(clk) begin
        if rising_edge(clk) then
            rc_y(0) <= C_CY_R(to_integer(p14_x0)); rc_uv(0) <= C_CUV_R(to_integer(p14_x0));
            rc_y(1) <= C_CY_G(to_integer(p14_x1)); rc_uv(1) <= C_CUV_G(to_integer(p14_x1));
            rc_y(2) <= C_CY_B(to_integer(p14_x2)); rc_uv(2) <= C_CUV_B(to_integer(p14_x2));
        end if;
    end process;

    -- pink coefficient RAMs (one write port each; reads by the pixel path never meet the bank being written)
    p_ram : process(clk) begin
        if rising_edge(clk) then
            if r_we = '1' then
                ram_p(to_integer(r_wa)) <= r_wp; ram_q(to_integer(r_wa)) <= r_wq;
            end if;
            ram_pq <= ram_p(to_integer(r_ra)); ram_qq <= ram_q(to_integer(r_ra));
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, width, sync delays
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_id : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            v_id := to_integer(unsigned(registers_in(8)(3 downto 0)));
            t_xs <= unsigned(C_T_XS(v_id)); t_ys <= unsigned(C_T_YS(v_id));
            t_kax <= unsigned(C_T_KAX(v_id)); t_kbx <= unsigned(C_T_KBX(v_id));
            t_kay <= unsigned(C_T_KAY(v_id)); t_kby <= unsigned(C_T_KBY(v_id));

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
            s_avr <= data_in.avid and not s_avq;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;
            g_wm1 <= s_w - 1;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                f_tear <= registers_in(6)(0);
                f_key  <= registers_in(6)(1);
                f_pal  <= registers_in(6)(2);
                f_mir  <= registers_in(6)(3);
                f_osc  <= registers_in(6)(4);
                if t_xs > 384 then f_sd <= '1'; else f_sd <= '0'; end if;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to 28 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Prepared-line inputs to the engine (from e_vy) and the drawn line's flags (from d_vy)
    ---------------------------------------------------------------------------
    p_lines : process(clk)
        variable v_y  : unsigned(10 downto 0);
        variable v_yl : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            v_y := e_vy(18 downto 8);
            e_f <= e_vy(13 downto 6);
            if v_y < 135 then v_yl := e_vy; e_tb2 <= '0'; else v_yl := e_vy - 34560; e_tb2 <= '1'; end if;
            e_ft <= v_yl(13 downto 6);
            e_rt <= v_yl(15 downto 14);
            if f_tear = '1' and v_y < 270 then e_tb <= '1'; else e_tb <= '0'; end if;
            if v_y < 100 then e_pb5 <= to_unsigned(0, 5);
            elsif v_y < 298 then e_pb5 <= to_unsigned(7, 5);
            elsif v_y < 549 then e_pb5 <= to_unsigned(15, 5);
            elsif v_y < 611 then e_pb5 <= to_unsigned(23, 5);
            else e_pb5 <= to_unsigned(0, 5); end if;

            v_y := l_vyq(14 downto 4);
            if f_tear = '1' and v_y < 270 then l_tb <= '1'; else l_tb <= '0'; end if;
            if f_tear = '1' and v_y >= 626 and v_y < 644 then l_gl <= '1'; else l_gl <= '0'; end if;
            l_goff <= shift_left(resize(v_y - 626, 14), 9);
            if v_y = 549 or v_y = 550 then l_sep <= '1'; else l_sep <= '0'; end if;
            if v_y < 100 then l_gm <= "000";
            elsif v_y < 298 then l_gm <= "001";
            elsif v_y < 549 then l_gm <= "010";
            elsif v_y < 611 then l_gm <= "011";
            else l_gm <= "100"; end if;
            if (l_dph <= 1 or l_dph = 127) and v_y < 990 then l_dot <= '1'; else l_dot <= '0'; end if;
        end if;
    end process p_lines;

    ---------------------------------------------------------------------------
    -- Microcoded engine.  One instruction a clock through F D O E; a taken jump refetches (4 clocks);
    -- MAX / MIN take 2 clocks, MUL / MULL 10 (radix-4 Booth, ACC 26 x B 16 signed), DIV 18 (B 16 / ACC 13).
    -- RAW distances (an RF read 3 instructions after the ST that feeds it, a ROM read 3 after GRA, an
    -- X-indexed access 3 after SETX) are enforced by the assembler (mc2.py), so there is no forwarding.
    -- Program and register file are block RAMs whose read enables stall with the pipeline.
    ---------------------------------------------------------------------------
    e_op <= to_integer(unsigned(u_ie(15 downto 11)));
    u_fin <= '1' when (e_op = OP_MAX or e_op = OP_MIN) and u_ec = 1 else
             '1' when (e_op = OP_MUL or e_op = OP_MULL) and u_ec = 9 else
             '1' when e_op = OP_DIV and u_ec = 17 else
             '1' when e_op = OP_XFER and u_ec = 1 and x_run = '0' and x_d1 = '0' else
             '0' when e_op = OP_MAX or e_op = OP_MIN or e_op = OP_MUL or e_op = OP_MULL or e_op = OP_DIV or e_op = OP_XFER else
             '1';
    u_adv <= u_run and ((not u_ve) or u_fin);
    u_ra_d <= unsigned(u_ir(7 downto 0)) + resize(u_x, 8) when u_ir(10 downto 8) = "001" else unsigned(u_ir(7 downto 0));
    u_ra <= resize(x_i, 8) when x_run = '1' else u_ra_d;
    u_re <= u_adv or x_run;
    u_we <= '1' when u_run = '1' and u_ve = '1' and e_op = OP_ST else '0';
    u_wa <= unsigned(u_ie(7 downto 0)) + resize(u_x, 8) when u_ie(10 downto 8) = "001" else
            unsigned(u_ie(7 downto 0)) + resize(u_x(2 downto 0), 8) when u_ie(10 downto 8) = "010" else
            unsigned(u_ie(7 downto 0));

    p_urom : process(clk) begin
        if rising_edge(clk) then
            if u_adv = '1' then u_ir <= C_UC(to_integer(u_pc)); end if;
        end if;
    end process;

    p_rf : process(clk) begin
        if rising_edge(clk) then
            if u_we = '1' then rf(to_integer(u_wa)) <= std_logic_vector(u_acc(15 downto 0)); end if;
            if u_re = '1' then u_rfq <= rf(to_integer(u_ra)); end if;
        end if;
    end process;

    p_eng : process(clk)
        variable v_sel : unsigned(3 downto 0);
        variable v_in  : signed(15 downto 0);
        variable v_x3  : unsigned(2 downto 0);
        variable v_ob  : signed(19 downto 0);
        variable v_a, v_b : signed(20 downto 0);
        variable v_s28 : signed(21 downto 0);
        variable v_sum : signed(20 downto 0);
        variable v_ci  : std_logic;
        variable v_bt  : signed(21 downto 0);
        variable v_bo  : signed(21 downto 0);
        variable v_bn  : std_logic;
        variable v_h29 : signed(22 downto 0);
        variable v_hs  : signed(21 downto 0);
        variable v_r2  : unsigned(13 downto 0);
        variable v_df  : unsigned(14 downto 0);
        variable v_lt, v_eq, v_c : boolean;
        variable v_jmp : boolean;
        variable v_tgt : unsigned(8 downto 0);
        variable v_m   : integer range 0 to 7;
        variable v_bd  : std_logic_vector(2 downto 0);
        variable v_pr  : std_logic_vector(19 downto 0);
    begin
        if rising_edge(clk) then
            r_go <= '0';
            if s_avr = '1' then u_pend <= '1'; end if;
            x_d1 <= x_run; x_i1 <= x_i;
            if x_run = '1' then
                if x_i = 18 then x_run <= '0'; end if;
                x_i <= x_i + 1;
            end if;
            -- XFER window: the drawn line has left the parameter stages (16 clocks after its avid)
            if g_avid = '1' then x_lc <= (others => '0');
            elsif x_lc /= 31 then x_lc <= x_lc + 1; end if;
            if g_start = '1' then x_w <= '0'; elsif x_lc = 16 then x_w <= '1'; end if;

            if s_zgo = '1' then
                -- field start: the field program (it falls into line 0's program)
                u_run <= '1'; u_pc <= to_unsigned(256, 9);
                u_vd <= '0'; u_vo <= '0'; u_ve <= '0'; u_ec <= (others => '0'); u_pend <= '0';
                e_vy <= (others => '0');
            elsif u_run = '0' then
                if u_pend = '1' then
                    -- the next line: prepare it
                    u_run <= '1'; u_pc <= (others => '0'); u_pend <= '0';
                    u_vd <= '0'; u_vo <= '0'; u_ve <= '0'; u_ec <= (others => '0');
                    e_vy <= e_vy + t_ys;
                end if;
            else
                v_jmp := false; v_tgt := (others => '0');
                v_m := to_integer(unsigned(u_ie(10 downto 8)));
                -- the one adder: LD 0 + B; ADD A + B; SUB / compare A + ~B + 1; RSB ~A + B + 1
                v_a := resize(u_acc, 21); v_b := resize(u_opb, 21); v_ci := '0';
                case e_op is
                    when OP_LD | OP_MAX | OP_MIN =>
                        if e_op = OP_LD or u_ec = 1 then v_a := (others => '0');
                        else v_b := not v_b; v_ci := '1'; end if;
                    when OP_SUB => v_b := not v_b; v_ci := '1';
                    when OP_RSB => v_a := not v_a; v_ci := '1';
                    when others => null;
                end case;
                v_s28 := (v_a & '1') + (v_b & v_ci);
                v_sum := v_s28(21 downto 1);
                v_lt := u_acc(19) = '1';
                v_eq := u_acc = 0;
                -- Booth radix-4 step operand: 0, +-A, +-2A
                v_bd := u_l(1 downto 0) & u_bp;
                v_bt := resize(u_acc, 22);
                if v_bd = "011" or v_bd = "100" then v_bo := shift_left(v_bt, 1); else v_bo := v_bt; end if;
                if v_bd = "000" or v_bd = "111" then v_bo := (others => '0'); end if;
                v_bn := v_bd(2) and not (v_bd(1) and v_bd(0));
                if v_bn = '1' then v_bo := not v_bo; end if;
                v_h29 := (u_h & '1') + (v_bo & v_bn);
                v_hs := v_h29(22 downto 1);
                -- divider step: remainder in u_h(13 downto 0), quotient shifts into u_l
                v_r2 := unsigned(u_h(12 downto 0)) & u_l(15);
                v_df := ('0' & v_r2) - ("00" & unsigned(u_acc(12 downto 0)));

                if u_ve = '1' then
                    case e_op is
                        when OP_LD | OP_ADD | OP_SUB | OP_RSB =>
                            u_acc <= v_sum(19 downto 0);
                        when OP_MAX | OP_MIN =>
                            if u_ec = 0 then
                                u_mlt <= v_sum(20); u_ec <= to_unsigned(1, 5);
                            else
                                if (e_op = OP_MAX and u_mlt = '1') or (e_op = OP_MIN and u_mlt = '0') then
                                    u_acc <= v_sum(19 downto 0);
                                end if;
                                u_ec <= (others => '0');
                            end if;
                        when OP_MUL | OP_MULL =>
                            if u_ec = 0 then
                                u_h <= (others => '0'); u_l <= std_logic_vector(u_opb(15 downto 0)); u_bp <= '0';
                                u_ec <= to_unsigned(1, 5);
                            elsif u_ec /= 9 then
                                u_h <= shift_right(v_hs, 2);
                                u_l <= std_logic_vector(v_hs(1 downto 0)) & u_l(15 downto 2);
                                u_bp <= u_l(1);
                                u_ec <= u_ec + 1;
                            else
                                if e_op = OP_MUL then v_pr := std_logic_vector(u_h(11 downto 0)) & u_l(15 downto 8);
                                else v_pr := std_logic_vector(u_h(3 downto 0)) & u_l; end if;
                                u_acc <= signed(v_pr);
                                u_ec <= (others => '0');
                            end if;
                        when OP_DIV =>
                            if u_ec = 0 then
                                u_h <= (others => '0'); u_l <= std_logic_vector(u_opb(15 downto 0));
                                u_ec <= to_unsigned(1, 5);
                            elsif u_ec /= 17 then
                                if v_df(14) = '0' then
                                    u_h(13 downto 0) <= signed(v_df(13 downto 0)); u_l <= u_l(14 downto 0) & '1';
                                else
                                    u_h(13 downto 0) <= signed(v_r2); u_l <= u_l(14 downto 0) & '0';
                                end if;
                                u_ec <= u_ec + 1;
                            else
                                u_acc <= signed(resize(unsigned(u_l), 20));
                                u_ec <= (others => '0');
                            end if;
                        when OP_OUT =>
                            case to_integer(unsigned(u_ie(3 downto 0))) is
                                when 0 => f_dutyh <= unsigned(u_acc(7 downto 0));
                                when 1 => f_bk <= unsigned(u_acc(10 downto 0));
                                when 2 => f_isl <= unsigned(u_acc(10 downto 0));
                                when 3 => f_thl <= u_acc(15 downto 0);
                                when 4 => f_thh <= u_acc(15 downto 0);
                                when 5 => f_flowr <= unsigned(u_acc(9 downto 0));
                                when 6 => u_z <= u_acc(15 downto 0);
                                when 7 => r_ws(0) <= unsigned(u_acc(12 downto 0));
                                when 8 => r_ws(1) <= unsigned(u_acc(12 downto 0));
                                when 9 => r_ws(2) <= unsigned(u_acc(12 downto 0));
                                when 10 => r_ws(3) <= unsigned(u_acc(12 downto 0));
                                when 11 => r_ws(4) <= unsigned(u_acc(12 downto 0));
                                when 12 => r_ws(5) <= unsigned(u_acc(12 downto 0));
                                when others => null;
                            end case;
                        when OP_SHR4 => u_acc <= shift_right(u_acc, 4);
                        when OP_GRA  => rom_a <= unsigned(u_acc(7 downto 0));
                        when OP_SETX => u_x <= unsigned(u_acc(5 downto 0));
                        when OP_DJNZ =>
                            if (u_ie(8) = '0' and u_x(2 downto 0) /= 0) or (u_ie(8) = '1' and u_x /= 0) then
                                u_x <= u_x - 1; v_jmp := true; v_tgt := u_pc(8) & unsigned(u_ie(7 downto 0));
                            end if;
                        when OP_BR =>
                            case v_m is
                                when 0 => v_c := true;
                                when 1 => v_c := v_lt;
                                when 2 => v_c := not v_lt;
                                when 3 => v_c := v_eq;
                                when 4 => v_c := not v_eq;
                                when 5 => v_c := v_lt or v_eq;
                                when others => v_c := not v_lt and not v_eq;
                            end case;
                            if v_c then v_jmp := true; v_tgt := u_pc(8) & unsigned(u_ie(7 downto 0)); end if;
                        when OP_SLR => u_lr <= u_pc(8) & unsigned(u_ie(7 downto 0));
                        when OP_RET => v_jmp := true; v_tgt := u_lr;
                        when OP_JLINE => v_jmp := true; v_tgt := (others => '0');
                        when OP_KICK => r_go <= '1';
                        when OP_XFER =>
                            if u_ec = 0 then
                                if x_w = '1' then
                                    x_run <= '1'; x_i <= (others => '0'); u_ec <= to_unsigned(1, 5);
                                    d_bank <= r_bank;
                                end if;
                            elsif x_run = '0' and x_d1 = '0' then
                                u_ec <= (others => '0');
                            end if;
                        when others => null;
                    end case;
                end if;

                if u_ve = '1' and e_op = OP_HALT then
                    u_run <= '0'; u_vd <= '0'; u_vo <= '0'; u_ve <= '0';
                elsif u_adv = '1' then
                    -- advance: IN mux (D -> O), operand mux (O -> E), stage valid bits, fetch address
                    v_sel := unsigned(u_ir(3 downto 0));
                    if u_ir(10 downto 8) = "110" then v_sel := v_sel or ('0' & u_x(2 downto 0)); end if;
                    v_x3 := u_x(2 downto 0);
                    case to_integer(v_sel) is
                        when 0  => v_in := signed(resize(e_vy(18 downto 4), 16));
                        when 1  => v_in := signed(resize(e_f, 16));
                        when 2  => v_in := signed(resize(e_ft, 16));
                        when 3  => v_in := signed(resize(e_pb5, 16));
                        when 4  =>
                            if (v_x3 >= 2 and v_x3 <= 6) or (v_x3 = 1 and u_x(4 downto 3) = "01") then v_in := u_z;
                            else v_in := (others => '0'); end if;
                        when 5  => v_in := signed(resize(t_xs, 16));
                        when 8  => v_in := signed(resize(unsigned(registers_in(0)), 16));
                        when 9  => v_in := signed(resize(unsigned(registers_in(1)), 16));
                        when 10 => v_in := signed(resize(unsigned(registers_in(2)), 16));
                        when 11 => v_in := signed(resize(unsigned(registers_in(4)), 16));
                        when 12 => v_in := signed(resize(unsigned(registers_in(7)), 16));
                        when 13 => v_in := signed(resize(unsigned(registers_in(3)), 16));
                        when 14 => v_in := signed(resize(unsigned(registers_in(5)), 16));
                        when others => v_in := (others => '0');
                    end case;
                    u_inq <= v_in;
                    case u_io(10 downto 8) is
                        when "000" | "001" => v_ob := resize(signed(u_rfq), 20);
                        when "010" => v_ob := signed(resize(unsigned(u_io(7 downto 0)), 20));
                        when "011" => v_ob := resize(signed(u_io(7 downto 0)), 20);
                        when "101" =>
                            case u_io(1 downto 0) is
                                when "00" => v_ob := resize(signed(rom_gy), 20);
                                when "01" => v_ob := resize(signed(rom_gx), 20);
                                when others => v_ob := resize(signed(rom_gs), 20);
                            end case;
                        when others => v_ob := resize(u_inq, 20);
                    end case;
                    u_opb <= v_ob;
                    u_ie <= u_io; u_io <= u_ir;
                    if v_jmp then
                        u_ve <= '0'; u_vo <= '0'; u_vd <= '0'; u_pc <= v_tgt;
                    else
                        u_ve <= u_vo; u_vo <= u_vd; u_vd <= '1'; u_pc <= u_pc + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_eng;

    -- XFER loader: the line-register block, one word a clock (mc2.py param block order)
    p_xfer : process(clk) begin
        if rising_edge(clk) then
            if x_d1 = '1' then
                case to_integer(x_i1) is
                    when 0  => l_hw <= signed(u_rfq);
                    when 1  => l_ga <= signed(u_rfq);
                    when 2  => l_l1 <= unsigned(u_rfq(11 downto 0));
                    when 3  => l_l2 <= unsigned(u_rfq(11 downto 0));
                    when 4  => l_dph <= unsigned(u_rfq(6 downto 0));
                    when 5  => l_gb(6) <= resize(signed(u_rfq(15 downto 4)), 13);
                    when 6  => l_gb(5) <= resize(signed(u_rfq(15 downto 4)), 13);
                    when 7  => l_gb(4) <= resize(signed(u_rfq(15 downto 4)), 13);
                    when 8  => l_gb(3) <= resize(signed(u_rfq(15 downto 4)), 13);
                    when 9  => l_gb(2) <= resize(signed(u_rfq(15 downto 4)), 13);
                    when 10 => l_gb(1) <= resize(signed(u_rfq(15 downto 4)), 13);
                    when 11 => l_gb(0) <= resize(signed(u_rfq(15 downto 4)), 13);
                    when 13 => l_inc(0) <= unsigned(u_rfq(10 downto 0));
                    when 14 => l_inc(1) <= unsigned(u_rfq(10 downto 0));
                    when 15 => l_inc(2) <= unsigned(u_rfq(10 downto 0));
                    when 16 => l_inc(3) <= unsigned(u_rfq(10 downto 0));
                    when 17 => l_inc(4) <= unsigned(u_rfq(10 downto 0));
                    when 18 => l_vyq <= unsigned(u_rfq(14 downto 0));
                    when others => null;
                end case;
            end if;
        end if;
    end process p_xfer;

    ---------------------------------------------------------------------------
    -- Pink coefficient engine: a_k = (w0 T[r][k] + w1 T[r+1][k] + w2 T[r+2][k]) >> 10 (weights x Fan) for
    -- k = 0..31, accumulated over the weights' 4-bit digits (Horner, 12 clocks a coefficient); then the
    -- spline at 32 px steps s(2k) = (a_k + a_k+1)/2, s(2k+1) = (a_k + 6 a_k+1 + a_k+2)/8 and the chords
    -- (P, Q) = (s(m), s(m+1) - s(m)) into RAM[bank, m], m = 0..59 (torn-band rows for k < 12 at 608 / 688 + 16 r)
    ---------------------------------------------------------------------------
    p_reng : process(clk)
        variable v_ad : unsigned(9 downto 0);
        variable v_w  : unsigned(15 downto 0);
        variable v_r  : unsigned(4 downto 0);
        variable v_t  : signed(27 downto 0);
        variable v_s1 : signed(19 downto 0);
        variable v_q  : signed(17 downto 0);
    begin
        if rising_edge(clk) then
            r_we <= '0';
            if r_go = '1' then
                r_run <= '1'; r_k <= (others => '0'); r_i <= "11"; r_j <= (others => '0'); r_na <= (others => '0');
                r_rr <= e_vy(18 downto 14); r_rt <= e_rt; r_tb <= e_tb; r_tb2 <= e_tb2;
                r_bank <= not r_bank;
            end if;
            -- issue: address + weight digit
            r_v(1) <= '0';
            if r_run = '1' then
                if r_tb = '1' and r_k < 12 then
                    v_r := resize(r_rt, 5) + resize(r_j, 5);
                    if r_tb2 = '1' then v_ad := to_unsigned(688, 10); else v_ad := to_unsigned(608, 10); end if;
                    case to_integer(r_j) is
                        when 0 => v_w := resize(r_ws(3), 16); when 1 => v_w := resize(r_ws(4), 16); when others => v_w := resize(r_ws(5), 16);
                    end case;
                    r_ad <= v_ad + shift_left(resize(v_r, 10), 4) + resize(r_k, 10);
                else
                    v_r := r_rr + resize(r_j, 5);
                    case to_integer(r_j) is
                        when 0 => v_w := resize(r_ws(0), 16); when 1 => v_w := resize(r_ws(1), 16); when others => v_w := resize(r_ws(2), 16);
                    end case;
                    r_ad <= shift_left(resize(v_r, 10), 5) + resize(r_k, 10);
                end if;
                case to_integer(r_i) is
                    when 3 => r_d(1) <= v_w(15 downto 12);
                    when 2 => r_d(1) <= v_w(11 downto 8);
                    when 1 => r_d(1) <= v_w(7 downto 4);
                    when others => r_d(1) <= v_w(3 downto 0);
                end case;
                r_v(1) <= '1';
                if r_j = 0 then r_f(1) <= '1'; else r_f(1) <= '0'; end if;
                if r_j = 0 and r_i = 3 then r_s(1) <= '1'; else r_s(1) <= '0'; end if;
                if r_j = 2 and r_i = 0 then r_e(1) <= '1'; else r_e(1) <= '0'; end if;
                if r_j = 2 then
                    r_j <= (others => '0');
                    if r_i = 0 then
                        r_i <= "11";
                        if r_k = 31 then r_run <= '0'; else r_k <= r_k + 1; end if;
                    else
                        r_i <= r_i - 1;
                    end if;
                else
                    r_j <= r_j + 1;
                end if;
            end if;
            -- 2: ROM data; 3: fabric copy; 4: T x digit
            r_v(2 to 4) <= r_v(1 to 3); r_f(2 to 4) <= r_f(1 to 3); r_s(2 to 4) <= r_s(1 to 3); r_e(2 to 4) <= r_e(1 to 3);
            r_d(2) <= r_d(1); r_d(3) <= r_d(2);
            r_q1 <= signed(rt_q(13 downto 0));
            r_pp <= resize(r_q1 * signed('0' & r_d(3)), 18);
            -- accumulate (Horner over the digits)
            if r_v(4) = '1' then
                if r_s(4) = '1' then v_t := resize(r_pp, 28);
                elsif r_f(4) = '1' then v_t := shift_left(r_acc, 4) + resize(r_pp, 28);
                else v_t := r_acc + resize(r_pp, 28); end if;
                r_acc <= v_t;
            end if;
            -- the finished coefficient (registered), then the chords over four clocks (no operand shares a net
            -- with another, so no self-adds): B: a1 + a2, a2 + a;  C: their sum, s(2k), s(2k+2);
            -- D: s(2k+1) = (a1 + 2 a2 + a + 4 a2)/8;  E: Q = s(m+1) - s(m), saturated; then the two writes
            r_av <= '0';
            if r_v(4) = '1' and r_e(4) = '1' then
                v_t := shift_right(v_t, 10);
                r_an <= v_t(15 downto 0); r_av <= '1';
            end if;
            r_bv <= '0';
            if r_av = '1' then
                r_a1 <= r_a2; r_a2 <= r_an;
                r_na <= r_na + 1;
                r_b0 <= resize(r_a1, 17) + resize(r_a2, 17);
                r_b2 <= resize(r_a2, 17) + resize(r_an, 17);
                r_bq <= r_a2;
                if r_na >= 2 then
                    r_seg <= resize(shift_left(r_na - 2, 1), 6);
                    r_bv <= '1';
                end if;
            end if;
            r_cv <= r_bv;
            r_cs <= resize(r_b0, 18) + resize(r_b2, 18);
            r_c0 <= shift_right(r_b0, 1); r_c2 <= shift_right(r_b2, 1); r_ca <= r_bq;
            r_dv <= r_cv;
            v_s1 := resize(r_cs, 20) + shift_left(resize(r_ca, 20), 2);
            r_d1 <= resize(shift_right(v_s1, 3), 17);
            r_d0 <= r_c0; r_d2 <= r_c2;
            if r_dv = '1' then
                v_q := resize(r_d1, 18) - resize(r_d0, 18);
                if v_q > 8191 then r_q0 <= to_signed(8191, 14); elsif v_q < -8191 then r_q0 <= to_signed(-8191, 14);
                else r_q0 <= resize(v_q, 14); end if;
                v_q := resize(r_d2, 18) - resize(r_d1, 18);
                if v_q > 8191 then r_q2 <= to_signed(8191, 14); elsif v_q < -8191 then r_q2 <= to_signed(-8191, 14);
                else r_q2 <= resize(v_q, 14); end if;
                r_wp <= resize(r_d0, 16); r_s1 <= resize(r_d1, 16);
                r_wph <= "01";
            end if;
            -- the two chord writes
            if r_wph = "01" then
                r_wq <= r_q0; r_wa <= r_bank & r_seg; r_we <= '1'; r_wph <= "10";
            elsif r_wph = "10" then
                r_wp <= r_s1; r_wq <= r_q2; r_wa <= r_bank & (r_seg + 1); r_we <= '1'; r_wph <= "00";
            end if;
        end if;
    end process p_reng;

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 2/3 clocks after the source's avid rise -- 3/4 at the output, where
    -- the pixel path plus the conditioning's 4-pixel centring is one clock longer than the syncs' delay --
    -- field-latched width)
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
            g_rsr <= g_rsr(0) & '0';
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
    g_start <= '1' when (g_sel = '0' and g_rsr(0) = '1') or (g_sel = '1' and g_rsr(1) = '1') else '0';

    ---------------------------------------------------------------------------
    -- Beam (stage 0): virtual x (Q.8), Osc phases
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if s_zgo = '1' then
                o_sa <= f_pa; o_sb <= f_pb;
                f_pa <= f_pa + 262; f_pb <= f_pb - 197;
            end if;
            if g_start = '1' then
                b_vx <= (others => '0');
                b_pa <= o_sa; b_pb <= o_sb;
                o_sa <= o_sa + t_kay; o_sb <= o_sb + t_kby;
            elsif g_avid = '1' then
                b_vx <= b_vx + t_xs;
                b_pa <= b_pa + t_kax; b_pb <= b_pb - t_kbx;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_vx  : unsigned(19 downto 0);
        variable v_t   : unsigned(15 downto 0);
        variable v_seg : signed(2 downto 0);
        variable v_inc : unsigned(10 downto 0);
        variable v_ga  : unsigned(18 downto 0);
        variable v_dn  : boolean;
        variable v_a   : signed(19 downto 0);
        variable v_bb  : signed(19 downto 0);
        variable v_e   : signed(17 downto 0);
        variable v_d8  : unsigned(7 downto 0);
        variable v_ds  : unsigned(7 downto 0);
        variable v_on  : std_logic;
        variable v_yy  : signed(12 downto 0);
        variable v_uu, v_vv : signed(12 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_dl  : unsigned(9 downto 0);
        variable v_lv  : unsigned(15 downto 0);
        variable v_g9  : signed(9 downto 0);
        variable v_dd, v_fl : unsigned(16 downto 0);
        variable v_rr  : unsigned(16 downto 0);
        variable v_vi  : signed(11 downto 0);
        variable v_ye  : signed(15 downto 0);
        variable v_xr  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- video tap: luma -> signed source (bend input), carried as a computed value only
            v_y1 <= unsigned(data_in.y);
            v_src <= signed(resize(v_y1(9 downto 2), 9)) - 128;

            ------------------------------------------------------------ 1
            if f_mir = '1' and b_vx > C_XC8 then p1_vxm <= resize(C_XC2 - resize(b_vx, 20), 19); p_rev(1) <= '1';
            else p1_vxm <= b_vx; p_rev(1) <= '0'; end if;
            p_rev(2 to 4) <= p_rev(1 to 3);
            if b_vx > C_XC8 then v_vx := resize(b_vx - C_XC8, 20); else v_vx := resize(C_XC8 - b_vx, 20); end if;
            p1_dx <= v_vx(18 downto 4);
            if v_vx(18 downto 8) < 7 then p1_dc <= '1'; else p1_dc <= '0'; end if;
            if b_vx > C_XH8 then v_vx := resize(b_vx - C_XH8, 20); else v_vx := resize(C_XH8 - b_vx, 20); end if;
            p1_h <= v_vx(17 downto 8);
            if b_vx(18 downto 8) < 90 then p1_lf <= resize(90 - b_vx(18 downto 8), 7); else p1_lf <= (others => '0'); end if;
            -- Osc triangles: 127 - |p - 32768| >> 7
            if b_pa(15) = '1' then v_t := b_pa - 32768; else v_t := 32767 - b_pa; end if;
            p1_ta <= to_signed(127, 9) - signed(resize(v_t(15 downto 7), 9));
            if b_pb(15) = '1' then v_t := b_pb - 32768; else v_t := 32767 - b_pb; end if;
            p1_tb <= to_signed(127, 9) - signed(resize(v_t(15 downto 7), 9));
            p1_vs <= v_src;

            ------------------------------------------------------------ 2
            if l_gl = '1' and p1_vxm(18 downto 8) > 1060 then p2_vxg <= p1_vxm - resize(l_goff, 19);
            else p2_vxg <= p1_vxm; end if;
            p2_xi <= p1_vxm(18 downto 8);
            v_bb := resize(l_hw, 20) - signed(resize(p1_dx, 20));
            p2_wi <= resize(shift_right(v_bb, 4), 12);   -- wedge distance, px
            r_yea <= p1_h(9 downto 1);                    -- ye ROM address (data at 3, fabric copy at 4)
            p2_lf <= p1_lf;
            if f_osc = '1' then p2_src <= resize(shift_right(resize(p1_ta, 10) + resize(p1_tb, 10), 1), 9);
            else p2_src <= p1_vs; end if;
            p_dot(2) <= p1_dc and l_dot;

            ------------------------------------------------------------ 3
            r_ra <= d_bank & p2_vxg(18 downto 13);       -- RAM read address (data at 4, fabric copy at 5)
            p3_g5 <= p2_vxg(12 downto 8);
            p3_xi <= p2_xi;
            for i in 0 to 3 loop
                if signed(resize(p2_xi, 13)) >= l_gb(i) then p3_ge(i) <= '1'; else p3_ge(i) <= '0'; end if;
            end loop;
            if signed(resize(p2_xi, 13)) > l_gb(5) then p3_gtr <= '1'; else p3_gtr <= '0'; end if;
            if signed(resize(p2_xi, 13)) < l_gb(6) then p3_ltl <= '1'; else p3_ltl <= '0'; end if;
            -- pink key flags on the (glitched) column
            v_xr := p2_vxg(18 downto 8);
            if v_xr >= l_l2 then p3_kf(0) <= '1'; else p3_kf(0) <= '0'; end if;
            if v_xr > l_l1 then p3_kf(1) <= '1'; else p3_kf(1) <= '0'; end if;
            if l_tb = '1' and v_xr < 640 then p3_kf(2) <= '1'; else p3_kf(2) <= '0'; end if;
            -- wedge profile address (3..5): edge part (d + 97) or one stripe period (x 256/296) + flow
            p3_wv <= resize(p2_wi, 13) - 30;
            v_bb := resize(p2_wi, 20) + 97;
            p3_we <= unsigned(v_bb(6 downto 0));
            if p2_wi < 30 then p3_wlo <= '1'; else p3_wlo <= '0'; end if;
            if p2_wi < -97 then p3_wcl <= '1'; else p3_wcl <= '0'; end if;
            if p2_wi >= -4 then p3_wn <= '1'; else p3_wn <= '0'; end if;
            p3_lf <= p2_lf;
            p3_br <= resize(p2_src * signed('0' & f_bk), 20);
            p_dot(3) <= p_dot(2);

            ------------------------------------------------------------ 4
            p4_g5 <= p3_g5;
            -- plateau segment: last breakpoint passed (lead / off when before the lit start)
            v_seg := to_signed(-2, 3);
            if p3_ge(0) = '1' then v_seg := to_signed(0, 3); end if;
            if p3_ge(1) = '1' then v_seg := to_signed(1, 3); end if;
            if p3_ge(2) = '1' then v_seg := to_signed(2, 3); end if;
            if p3_ge(3) = '1' then v_seg := to_signed(3, 3); end if;
            if l_gm = "011" and p3_ltl = '1' then v_seg := to_signed(-1, 3); end if;
            p4_seg <= v_seg;
            case to_integer(v_seg) is
                when 0 => v_inc := l_inc(0);
                when 1 => v_inc := l_inc(1);
                when 2 => v_inc := l_inc(2);
                when 3 => v_inc := l_inc(3);
                when -1 => v_inc := l_inc(4);
                when others => v_inc := (others => '0');
            end case;
            if l_gm = "100" then v_inc := f_isl; end if;
            p4_inc <= v_inc;
            if l_gm = "000" or (v_seg = -2 and l_gm /= "100") or (p3_gtr = '1' and (l_gm = "001" or l_gm = "010")) then p4_off <= '1';
            else p4_off <= '0'; end if;
            p4_ye <= signed(r_yeq);
            p4_lf <= p3_lf;
            p4_br <= resize(shift_right(p3_br, 10), 10);
            v_a := resize(shift_right(p3_br, 10), 20);
            p4_bg <= resize(shift_right(v_a, 1) + shift_right(v_a, 3) - shift_right(v_a, 5), 10);
            v_bb := resize(p3_wv, 20);
            v_bb := shift_right(shift_left(v_bb, 8) - shift_left(v_bb, 5) - shift_left(v_bb, 1) - v_bb, 8);
            p4_wm <= unsigned(v_bb(7 downto 0)); p4_we <= p3_we; p4_wlo <= p3_wlo; p4_wcl <= p3_wcl;
            p_k(4) <= p3_kf;
            p_wn(4) <= p3_wn;
            p_dot(4) <= p_dot(3);

            ------------------------------------------------------------ 5
            p5_p <= ram_pq; p5_q <= ram_qq;
            p5_g5 <= p4_g5;
            p5_seg <= p4_seg; p5_off <= p4_off;
            -- ramp phase DDA (Q.6, pr = acc >> 6): restarts with the line and at every breakpoint (plateau 0,
            -- lead 159; mirrored half: plateau 255, lead 0), then steps by the segment's increment, downwards on
            -- the lead ramp and on the mirrored half (they cancel)
            v_dn := (p5_seg = -1) xor (p_rev(4) = '1');
            if s_gav_sr(4) = '0' or (l_gm /= "100" and p4_seg /= p5_seg) then
                if l_gm = "100" or (p_rev(4) = '0' and p4_seg /= -1) or (p_rev(4) = '1' and p4_seg = -1) then
                    p5_acc <= (others => '0');
                elsif p4_seg = -1 then
                    p5_acc <= to_unsigned(10176, 18);
                else
                    p5_acc <= to_unsigned(16320, 18);
                end if;
            elsif v_dn and l_gm /= "100" then
                if p5_acc > resize(p4_inc, 18) then p5_acc <= p5_acc - p4_inc; else p5_acc <= (others => '0'); end if;
            elsif l_gm = "100" and p_rev(4) = '1' then
                if p5_acc > resize(p4_inc, 18) then p5_acc <= p5_acc - p4_inc; else p5_acc <= (others => '0'); end if;
            else
                v_ga := resize(p5_acc, 19) + p4_inc;
                if v_ga(18) = '1' then p5_acc <= (others => '1'); else p5_acc <= v_ga(17 downto 0); end if;
            end if;
            p5_ye <= p4_ye + signed(resize(shift_left(resize(p4_lf, 10), 2) + resize(p4_lf, 10), 16));
            p5_br <= p4_br; p5_bg <= p4_bg;
            if p4_wlo = '0' then p5_aw <= '1' & (p4_wm + f_flowr(9 downto 2));
            elsif p4_wcl = '1' then p5_aw <= (others => '0');
            else p5_aw <= "00" & p4_we; end if;
            p_k(5) <= p_k(4); p_wn(5) <= p_wn(4); p_dot(5) <= p_dot(4);

            ------------------------------------------------------------ 6
            p6_p <= p5_p;
            p6_qg <= resize(p5_q * signed('0' & p5_g5), 20);
            p6_seg <= p5_seg; p6_off <= p5_off;
            p6_pr <= resize(shift_right(p5_acc, 6), 15);
            -- lower V (6..9): v = line - ye (Q.4) -> px -> profile address, edge part or one period (x 256/332)
            v_ye := signed(resize(l_vyq, 16)) - p5_ye;
            p6_vi <= resize(shift_right(v_ye, 4), 12);
            p_bw(7) <= unsigned(rb_w);               -- wedge ROM (address at 5, data at 6) -> fabric 7
            p6_br <= p5_br; p6_bg <= p5_bg;
            p_k(6) <= p_k(5); p_dot(6) <= p_dot(5);

            ------------------------------------------------------------ 7
            v_e := resize(p6_p, 18) + resize(shift_right(p6_qg, 5), 18);
            if v_e > 32767 then p7_phi <= to_signed(32767, 16); elsif v_e < -32767 then p7_phi <= to_signed(-32767, 16);
            else p7_phi <= resize(v_e, 16); end if;
            -- green phase (Q.8; only the low 8 bits matter)
            if l_gm = "100" then
                p7_gph <= l_ga + signed(resize(p6_pr, 16)) + resize(p6_bg, 16);
            elsif p6_seg = -1 then
                p7_gph <= -signed(resize(p6_pr, 16));
            else
                if p6_pr > 255 then v_t := to_unsigned(255, 16); else v_t := resize(p6_pr, 16); end if;
                p7_gph <= signed(resize(unsigned(p6_seg(1 downto 0)) & v_t(7 downto 0), 16));
            end if;
            p7_off <= p6_off;
            p7_br <= p6_br;
            p_k(7) <= p_k(6); p_dot(7) <= p_dot(6);
            p7_vv <= resize(p6_vi, 13) - 30;
            v_bb := resize(p6_vi, 20) + 97;
            p7_ve <= unsigned(v_bb(6 downto 0));
            if p6_vi < 30 then p7_vlo <= '1'; else p7_vlo <= '0'; end if;
            if p6_vi < -97 then p7_vcl <= '1'; else p7_vcl <= '0'; end if;
            if p6_vi >= -4 then p7_vn <= '1'; else p7_vn <= '0'; end if;

            ------------------------------------------------------------ 8: pink phase + gates
            p8_pr <= unsigned(p7_phi) + resize(f_flowr, 16) + unsigned(resize(p7_br, 16));
            v_on := '1';
            if p7_phi <= f_thl and p_k(7)(2) = '0' then v_on := '0'; end if;
            if f_key = '1' then
                if p_k(7)(0) = '1' then v_on := '0'; end if;
                if p_k(7)(1) = '1' and (p_k(7)(2) = '1' or p7_phi < f_thh) then v_on := '0'; end if;
            end if;
            p8_ok <= v_on;
            p8_gf <= unsigned(p7_gph(7 downto 0)) + f_flowr(8 downto 1);
            p8_off <= p7_off;
            v_bb := resize(p7_vv, 20);
            v_bb := shift_right(shift_left(v_bb, 7) + shift_left(v_bb, 6) + shift_left(v_bb, 2) + v_bb, 8);
            p8_vm <= unsigned(v_bb(7 downto 0)); p8_ve <= p7_ve; p8_vlo <= p7_vlo; p8_vcl <= p7_vcl;
            p_vn(8) <= p7_vn;

            ------------------------------------------------------------ 9: stripe test, green linear levels
            v_d8 := p8_pr(7 downto 0) + 10;
            if v_d8(7) = '0' then v_ds := v_d8; else v_ds := 0 - v_d8; end if;
            if v_ds < f_dutyh and p8_ok = '1' then p9_on <= '1'; else p9_on <= '0'; end if;
            -- plateau 25 + 215 f >> 8, lower 22 + 230 f >> 8
            p9_l0 <= resize(25 + shift_right(shift_left(resize(p8_gf, 16), 8) - shift_left(resize(p8_gf, 16), 5)
                     - shift_left(resize(p8_gf, 16), 3) - resize(p8_gf, 16), 8), 8);
            p9_l1 <= resize(22 + shift_right(shift_left(resize(p8_gf, 16), 8) - shift_left(resize(p8_gf, 16), 4)
                     - shift_left(resize(p8_gf, 16), 3) - shift_left(resize(p8_gf, 16), 1), 8), 8);
            p9_off <= p8_off;
            if p8_vlo = '0' then p9_av <= '1' & (p8_vm + f_flowr(9 downto 2));
            elsif p8_vcl = '1' then p9_av <= (others => '0');
            else p9_av <= "00" & p8_ve; end if;

            ------------------------------------------------------------ 10: pink RC smear (HD 3/16 / 1/8, SD 7/16 / 5/16)
            v_dd := 65280 - resize(r_iir, 17);
            v_fl := resize(r_iir, 17);
            if s_gav_sr(8) = '0' then
                r_iir <= (others => '0');
            elsif p9_on = '1' then
                if f_sd = '1' then v_rr := v_fl + shift_right(v_dd, 1) - shift_right(v_dd, 4);
                else v_rr := v_fl + shift_right(v_dd, 3) + shift_right(v_dd, 4); end if;
                r_iir <= v_rr(15 downto 0);
            else
                if f_sd = '1' then v_rr := v_fl - shift_right(v_fl + 3, 2) - shift_right(v_fl + 15, 4);
                else v_rr := v_fl - shift_right(v_fl + 7, 3); end if;
                if v_rr(16) = '1' then r_iir <= (others => '0'); else r_iir <= v_rr(15 downto 0); end if;
            end if;
            p10_l0 <= p9_l0; p10_l1 <= p9_l1; p10_off <= p9_off;
            p10_l3 <= unsigned(rl3_q);               -- L3 ROM (address at 8, data at 9) -> fabric 10

            ------------------------------------------------------------ 11: green level select
            p11_r8 <= r_iir(15 downto 8);
            if p10_off = '1' then p11_glv <= to_unsigned(5, 8);
            else
                case l_gm is
                    when "011" => v_lv := resize(p10_l3, 16);
                    when "100" => v_lv := resize(p10_l1, 16);
                    when others => v_lv := resize(p10_l0, 16);
                end case;
                if l_sep = '1' and v_lv > 60 then v_lv := to_unsigned(60, 16); end if;
                p11_glv <= v_lv(7 downto 0);
            end if;

            ------------------------------------------------------------ 12: green smear (HD 1/2, SD none)
            if s_gav_sr(10) = '0' then
                g_iir <= (others => '0');
            elsif f_sd = '1' then
                g_iir <= p11_glv;
            else
                v_g9 := signed(resize(g_iir, 10)) + shift_right(signed(resize(p11_glv, 10)) - signed(resize(g_iir, 10)), 1);
                g_iir <= unsigned(v_g9(7 downto 0));
            end if;
            p12_r8 <= p11_r8;

            -- blue profiles delayed to 12, select into 13
            for i in 8 to 12 loop p_bw(i) <= p_bw(i - 1); end loop;
            p_bv(11) <= unsigned(rb_v);              -- lower-V ROM (address at 9, data at 10) -> fabric 11
            p_bv(12) <= p_bv(11);
            for i in 9 to 12 loop p_vn(i) <= p_vn(i - 1); end loop;
            for i in 6 to 12 loop p_wn(i) <= p_wn(i - 1); end loop;
            for i in 8 to 17 loop p_dot(i) <= p_dot(i - 1); end loop;

            ------------------------------------------------------------ 13: align the three layers
            if p_vn(12) = '1' then p13_b8 <= p_bv(12);
            elsif p_wn(12) = '1' then p13_b8 <= p_bw(12);
            elsif p_bw(12) < p_bv(12) then p13_b8 <= p_bw(12);
            else p13_b8 <= p_bv(12); end if;
            p13_r8 <= p12_r8;
            p13_g8 <= g_iir;

            ------------------------------------------------------------ 14: palette routing -> colour ROMs
            if f_pal = '1' then p14_x0 <= p13_b8; p14_x1 <= p13_r8; p14_x2 <= p13_g8;
            else p14_x0 <= p13_r8; p14_x1 <= p13_g8; p14_x2 <= p13_b8; end if;
            -- 15: ROM out (rc_*); 16: fabric copy
            p16_y <= rc_y; p16_uv <= rc_uv;

            ------------------------------------------------------------ 17: Y / U / V sums (M folded into the ROMs)
            p17_y <= resize(unsigned(p16_y(0)), 12) + resize(unsigned(p16_y(1)), 12) + resize(unsigned(p16_y(2)), 12) + C_YOFF;
            p17_u <= shift_left(resize(signed(p16_uv(0)(15 downto 8)), 12) + resize(signed(p16_uv(1)(15 downto 8)), 12)
                     + resize(signed(p16_uv(2)(15 downto 8)), 12), 2) + C_UOFF;
            p17_v <= shift_left(resize(signed(p16_uv(0)(7 downto 0)), 12) + resize(signed(p16_uv(1)(7 downto 0)), 12)
                     + resize(signed(p16_uv(2)(7 downto 0)), 12), 2) + C_VOFF;

            ------------------------------------------------------------ 18: dots, clip
            if p_dot(17) = '1' then
                v_yy := signed(resize(p17_y, 13)); v_yy := 64 + shift_right(v_yy - 64, 3);
                v_uu := resize(p17_u, 13); v_uu := 512 + shift_right(v_uu - 512, 3);
                v_vv := resize(p17_v, 13); v_vv := 512 + shift_right(v_vv - 512, 3);
            else
                v_yy := signed(resize(p17_y, 13)); v_uu := resize(p17_u, 13); v_vv := resize(p17_v, 13);
            end if;
            if v_yy > 1023 then p18_y <= to_unsigned(1023, 10); else p18_y <= unsigned(v_yy(9 downto 0)); end if;
            if v_uu > 1023 then p18_u <= to_unsigned(1023, 10); elsif v_uu < 0 then p18_u <= (others => '0');
            else p18_u <= unsigned(v_uu(9 downto 0)); end if;
            if v_vv > 1023 then p18_v <= to_unsigned(1023, 10); elsif v_vv < 0 then p18_v <= (others => '0');
            else p18_v <= unsigned(v_vv(9 downto 0)); end if;

            ------------------------------------------------------------ 19..28: output conditioning
            -- chroma box: 8-bit samples, neutral outside the line; the 8-clock-old sample comes from the
            -- delay RAM (cd_q); the running sums re-synchronise in every blanking interval
            if s_gav_sr(17) = '0' and s_gav_sr(24) = '0' then
                s_boxu <= to_unsigned(1024, 11); s_boxv <= to_unsigned(1024, 11);
            else
                s_boxu <= s_boxu + unsigned(s_c8(15 downto 8)) - unsigned(cd_q(15 downto 8));
                s_boxv <= s_boxv + unsigned(s_c8(7 downto 0)) - unsigned(cd_q(7 downto 0));
            end if;
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u24 <= v_sc(11 downto 2);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v24 <= v_sc(11 downto 2);
            s_y19 <= p18_y; s_y20 <= s_y19; s_y21 <= s_y20; s_y22 <= s_y21;
            s_y23 <= s_y22; s_y24x <= s_y23;
            v_sy := resize(s_y22, 12) + shift_left(resize(s_y23, 12), 1) + resize(s_y24x, 12);
            s_yf24 <= v_sy(11 downto 2);
            if s_yf24 > 64 then v_dl := s_yf24 - 64; else v_dl := (others => '0'); end if;
            -- gate: the conditioned value at stage 25 is centred 4 pixels back (box filter), so it belongs to the
            -- stage-28 pixel's avid
            if s_gav_sr(27) = '1' then
                s_y25 <= s_yf24 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u25 <= s_u24; s_v25 <= s_v24;
            else
                s_y25 <= to_unsigned(64, 10); s_u25 <= to_unsigned(512, 10); s_v25 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    -- stage-18 chroma (8 bits), neutral outside the line, into the box filter and its delay RAM
    s_c8 <= std_logic_vector(p18_u(9 downto 2)) & std_logic_vector(p18_v(9 downto 2)) when s_gav_sr(17) = '1' else x"8080";

    -- delay RAMs: chroma 8 clocks (box filter), syncs C_LAT clocks
    p_dly : process(clk) begin
        if rising_edge(clk) then
            cd_ram(to_integer(dl_wa)) <= s_c8;
            sd_ram(to_integer(dl_wa)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n & '1';
            cd_q <= cd_ram(to_integer(dl_rc));
            sd_q <= sd_ram(to_integer(dl_rs));
            dl_wa <= dl_wa + 1; dl_rc <= dl_rc + 1; dl_rs <= dl_rs + 1;
        end if;
    end process p_dly;

    -- the picture leaves 29 clocks after the beam (25 stages + the 4-pixel filter centring) with its avid; the
    -- syncs leave C_LAT = 28 behind the source, the beam starting one clock after the source's avid edge
    data_out.y       <= std_logic_vector(s_y25);
    data_out.u       <= std_logic_vector(s_u25);
    data_out.v       <= std_logic_vector(s_v25);
    data_out.hsync_n <= sd_q(3);
    data_out.vsync_n <= sd_q(2);
    data_out.field_n <= sd_q(1);
    data_out.avid    <= s_gav_sr(28);

end architecture tangram;
