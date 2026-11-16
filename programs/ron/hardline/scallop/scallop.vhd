-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Scallop v0.1.0 -- Hardline.  Seigaiha (the Japanese "blue sea wave" pattern of overlapping concentric-arc
-- fans) and Art Deco scale ornament, in black and white.
--
-- Rows of scales of radius rho = Fan * R on a lattice of pitch 2 R; rows step down by Overlap * R and every
-- other row is offset by R; lower rows lie in front.  A pixel belongs to the lowest row whose scale contains
-- it (four candidate rows: the lowest one that can reach it and the three above) and is inked by concentric
-- arcs:  phi = n (1 - d / rho) + camera,  d = the distance to the scale's centre (Round) or |dx| + |dy|
-- (Pointed).  Every scale carries a rigid black keyline at its rim, which the camera never moves, so the
-- lattice stays whole while the arcs inside each scale bend.
--
--   K1  X        pan, +-960 virtual px          K2  Y        pan, +-960 virtual px
--   K3  Period   R, 20 .. 300 virtual px (exponential)
--   K4  Rings    arcs per scale, 1 .. 8 (stepped; fewer when a band would fall under 3 pixels)
--   K5  Overlap  row step, R / 4 (deep fans) .. R (shallow scales), exponential
--   K6  Camera   the luma added to the arc phase, 0 .. 3.4 bands at white (default 100 %)
--   S7  Glide    Off = the knobs act the same field; On = 1/16 of the gap per field
--   S8  Form     Round / Pointed (diamond scales, chevrons)
--   S9  Shade    Flat / Shade (a sawtooth across each arc band, white at its outer edge: a shell)
--   S10 Weight   Bold (equal bands) / Fine (thin lines);  in Shade: 4 flat steps / a smooth ramp
--   S11 Source   Video / Osc (the shell's drifting triangle lattice at twice the level)
--   P12 FAN      the drawn radius of every scale, R / 4 (buds on black) .. R (the classic pattern, 67 %) .. 2 R
--
-- All geometry runs in rho-units (the drawn radius = 1.0; Q24 along the beam, sums of squares in Q16), so
-- the hit test is "d^2 < 1", the ring phase is n (1 - d) and nothing in the pixel path multiplies by a knob.
-- The lattice is a wrapping accumulator along the line (cell width 2 / Fan) and a remainder accumulator down
-- the field (row step Overlap / Fan).  Squares come from a ROM (high 8 bits) plus an 8 x 5 product, the
-- square root from a 512-point ROM with 6-bit interpolation, and both coverage divisions are shifts: the
-- divisor is rounded up to 2^k or 2^k * 32/23, so every edge is 1 .. 1.45 pixels wide.
--
-- Sections (the integer model .hardline_work/rb_scallop/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         glides (S7), switches, the Rings steps
--   [C] FIELD SEQUENCER                  one serial multiplier: exponentials from a 2^(i/256) ROM, the start
--                                        values of both accumulators, ring count, keyline and AA constants
--   [L] LINE ENGINE                      at each line end: the row remainder, then the four candidate rows'
--                                        dy^2 (four products on the same multiplier)
--   [P] PIXEL PATH                       beam accumulator -> |dx| per row parity -> two squarers -> four sums
--                                        -> front / back choice -> square root -> phase -> coverage -> tone
--   [D] SOURCE FIELD                     luma (or Osc) * K6 -> 128-px box -> vertical one-pole -> phase
--   [F] OUTPUT CONDITIONING              luma [1 2 1] / 4, 13/16 toward black, blanking gate; U = V = 512
--
-- Stage numbering: a register "at stage S" holds pixel n at T + S + n (T = the g_start clock).  Pixels
-- 0 .. W + 1 are rendered; output column j is the [1 2 1] filter over pixels j .. j + 2 and lands on screen
-- column j + 4 (the generated avid starts 4 clocks after the delayed source avid), so pixel m sits at
-- x = x0 + (m + 3) xs half virtual px.  s_gav_sr(S - 3) flags pixels 0 .. W - 1 on their way into a stage-S
-- register.  One latency (32) in every mode.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.scallop_pkg.all;
use work.all;

architecture scallop of program_top is
    constant C_LAT : integer := 32;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 760;
    constant C_DEF_K5  : integer := 512;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 683;

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
    -- m * l (8 x 5) as sums of gated operands, / 4: the squarers' low partial product
    function f_pp5(m : unsigned(7 downto 0); l : unsigned(4 downto 0)) return unsigned is
        variable g0, g1, g2, g3, g4, v_s : unsigned(12 downto 0) := (others => '0');
    begin
        if l(0) = '1' then g0 := resize(m, 13); end if;
        if l(1) = '1' then g1 := resize(m & "0", 13); end if;
        if l(2) = '1' then g2 := resize(m & "00", 13); end if;
        if l(3) = '1' then g3 := resize(m & "000", 13); end if;
        if l(4) = '1' then g4 := resize(m & "0000", 13); end if;
        v_s := ((g0 + g1) + (g2 + g3)) + g4;
        return v_s(12 downto 2);
    end function;
    -- a Q16 distance in 1/64 of the (rounded-up) pixel 2^k, saturated to 7 bits: bits k .. k - 6, unless the
    -- bits above (the per-field mask m: bit i set for i >= k) are not all copies of the sign
    function f_es7(e : signed(19 downto 0); k : unsigned(3 downto 0); m : std_logic_vector(19 downto 0)) return signed is
        variable v_e : std_logic_vector(19 downto 0);
        variable v_s : std_logic_vector(6 downto 0);
    begin
        v_e := std_logic_vector(e);
        case to_integer(k) is
            when 6      => v_s := v_e(6 downto 0);
            when 7      => v_s := v_e(7 downto 1);
            when 8      => v_s := v_e(8 downto 2);
            when 9      => v_s := v_e(9 downto 3);
            when 10     => v_s := v_e(10 downto 4);
            when 11     => v_s := v_e(11 downto 5);
            when 12     => v_s := v_e(12 downto 6);
            when 13     => v_s := v_e(13 downto 7);
            when others => v_s := v_e(14 downto 8);
        end case;
        if e(19) = '0' and (v_e and m) /= x"00000" then return to_signed(63, 7);
        elsif e(19) = '1' and ((not v_e) and m) /= x"00000" then return to_signed(-64, 7);
        else return signed(v_s); end if;
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
    signal w_dxm  : unsigned(10 downto 0);     -- the beam's pre-pixel: its distance left of the centre (h)

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

    -- [B] knobs: glide accumulators (Q6), switches, rings
    signal s_a1 : unsigned(15 downto 0) := to_unsigned(C_DEF_K1 * 64, 16);
    signal s_a2 : unsigned(15 downto 0) := to_unsigned(C_DEF_K2 * 64, 16);
    signal s_a3 : unsigned(15 downto 0) := to_unsigned(C_DEF_K3 * 64, 16);
    signal s_a5 : unsigned(15 downto 0) := to_unsigned(C_DEF_K5 * 64, 16);
    signal s_a6 : unsigned(15 downto 0) := to_unsigned(C_DEF_K6 * 64, 16);
    signal s_ap : unsigned(15 downto 0) := to_unsigned(C_DEF_P12 * 64, 16);
    signal s_gm : unsigned(15 downto 0) := (others => '0');
    signal s_gd : signed(16 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_glide : std_logic := '1';
    signal s_ptd, s_shade, s_fine, s_src : std_logic := '0';
    signal s_n0 : unsigned(2 downto 0) := to_unsigned(3, 3);      -- rings - 1
    signal s_lg7 : std_logic := '1';                               -- box: 128 samples (1080) / 64
    signal s_ish : unsigned(2 downto 0) := to_unsigned(5, 3);      -- the vertical one-pole's shift

    -- ROMs: 2^(i/256), tone -> luma, the squarers' high part (one per row parity), the square root
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal yr_q : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal sq0_q, sq1_q : unsigned(15 downto 0) := (others => '0');
    signal rtv_q : unsigned(15 downto 0) := (others => '0');
    signal rts_q : unsigned(7 downto 0) := (others => '0');

    -- [C] the serial multiplier (shared with the line engine) and the sequencer's scratch
    signal f_a, f_acc, z_fa : unsigned(31 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_oR, z_oF : unsigned(10 downto 0) := (others => '0');
    signal z_e, z_ex : unsigned(11 downto 0) := (others => '0');   -- exponent accumulator: 256 * (shift) + ROM index
    signal z_eop, z_esub : std_logic := '0';
    signal z_p    : signed(13 downto 0) := (others => '0');
    signal z_A, z_B : signed(18 downto 0) := (others => '0');
    signal z_xc   : unsigned(15 downto 0) := (others => '0');
    signal z_wpB, z_wp : unsigned(19 downto 0) := (others => '0');
    signal z_M    : unsigned(15 downto 0) := (others => '0');
    signal z_lg    : unsigned(3 downto 0) := (others => '0');
    signal z_D    : unsigned(16 downto 0) := (others => '0');
    signal z_c    : unsigned(3 downto 0) := (others => '0');
    signal z_w    : unsigned(15 downto 0) := (others => '0');
    signal z_inv  : unsigned(16 downto 0) := (others => '0');
    signal z_nom  : unsigned(15 downto 0) := (others => '0');
    signal z_ow   : unsigned(11 downto 0) := (others => '0');

    -- [C] per-field constants (rho-units: the drawn radius = 1.0)
    signal c_DUh  : unsigned(20 downto 0) := (others => '0');      -- per h, Q24
    signal c_DUP  : unsigned(22 downto 0) := (others => '0');      -- per pixel, Q24
    signal c_DV   : unsigned(24 downto 0) := (others => '0');      -- per field line, Q24
    signal c_DW   : unsigned(14 downto 0) := (others => '0');      -- per field line, Q16, at most 1/4
    signal c_CH   : unsigned(26 downto 0) := (others => '0');      -- half a cell, Q24
    signal c_S    : unsigned(25 downto 0) := (others => '0');      -- row step, Q24
    signal c_r0   : signed(27 downto 0) := (others => '0');        -- the beam at the pre-pixel (u - C)
    signal c_dupc : signed(27 downto 0) := (others => '0');        -- DUP - C
    signal c_yrem0 : unsigned(25 downto 0) := (others => '0');     -- line 0's row remainder
    signal c_pb0  : std_logic := '0';
    signal c_n    : unsigned(3 downto 0) := to_unsigned(4, 4);     -- rings
    signal c_wb   : unsigned(12 downto 0) := to_unsigned(64, 13);  -- bands per pixel, Q12
    signal c_kq   : unsigned(3 downto 0) := to_unsigned(9, 4);     -- the pixel, rounded up: 2^kq (x 32/23 if c_half), Q16
    signal c_half : std_logic := '0';
    signal c_msk  : std_logic_vector(19 downto 0) := (others => '1');   -- bits kq and up
    signal c_D16  : unsigned(15 downto 0) := to_unsigned(512, 16);
    signal c_O16  : unsigned(15 downto 0) := (others => '0');      -- keyline width, Q16
    signal c_Dq   : unsigned(15 downto 0) := (others => '0');      -- the front rim's reach past d = 1, in the sum's units
    signal c_Dk   : unsigned(16 downto 0) := (others => '0');      -- + down to the middle of the keyline
    signal c_offF : unsigned(11 downto 0) := to_unsigned(3072, 12);
    signal c_dhn  : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal w_Sq   : unsigned(15 downto 0);                         -- row step, Q14
    signal w_CH12 : unsigned(14 downto 0);
    signal w_thr  : signed(19 downto 0);                           -- 1 + Dq, Q16

    -- [L] line engine
    type t_m is array (0 to 3) of signed(19 downto 0);
    signal c_m    : t_m := (others => to_signed(1, 20));            -- candidate j: dy^2 (or |dy|) - threshold, Q16
    signal c_pb   : std_logic := '0';                               -- the lowest candidate row's parity
    signal e_st   : unsigned(6 downto 0) := (others => '1');
    signal e_zk   : std_logic := '0';
    signal e_yrem : unsigned(26 downto 0) := (others => '0');
    signal e_pb   : std_logic := '0';
    signal e_mld  : std_logic := '0';
    signal e_ma, e_mb : unsigned(12 downto 0) := (others => '0');
    signal e_far, e_farc : std_logic := '0';
    signal e_t    : unsigned(17 downto 0) := (others => '0');
    signal e_v    : signed(18 downto 0) := (others => '0');
    signal w_yq   : unsigned(15 downto 0);

    -- [P] pixel path
    signal b_r    : signed(27 downto 0) := (others => '0');
    signal s3_dxe : signed(15 downto 0) := (others => '0');
    signal s4_a0, s4_a1 : unsigned(12 downto 0) := (others => '0');
    signal s5_pr0, s5_pr1, s6_pr0, s6_pr1 : unsigned(10 downto 0) := (others => '0');
    signal s6_hs0, s6_hs1 : unsigned(15 downto 0) := (others => '0');
    signal s7_q0, s7_q1 : unsigned(17 downto 0) := (others => '0');
    signal s8_s   : t_m := (others => (others => '0'));
    signal s9_sf, s9_sb, s10_sel, s10_e1 : signed(19 downto 0) := (others => '0');
    signal s9_hasf, s9_hasb, s10_none, s10_isback : std_logic := '0';
    signal s11_sc, s12_sx : unsigned(16 downto 0) := (others => '0');
    signal s12_rr, s13_rr, s14_rr, s15_rr : std_logic_vector(1 downto 0) := "00";
    signal s13_x  : unsigned(14 downto 0) := (others => '0');
    signal s14_val, s15_val, s16_d : unsigned(12 downto 0) := (others => '0');
    signal s14_slope, s15_pr : unsigned(7 downto 0) := (others => '0');
    signal s14_fr : unsigned(4 downto 0) := (others => '0');
    type t_ee is array (11 to 19) of signed(6 downto 0);            -- the front rim's coverage term (+63: none in front)
    signal s_ee   : t_ee := (others => (others => '0'));
    signal s17_v  : signed(13 downto 0) := (others => '0');
    signal s18_phib, s19_phi : unsigned(15 downto 0) := (others => '0');
    signal s18_eown : signed(19 downto 0) := (others => '0');
    signal s19_w  : unsigned(12 downto 0) := (others => '0');
    signal s19_eos, s20_x : signed(6 downto 0) := (others => '0');
    signal s20_none : std_logic := '0';
    type t_g5 is array (21 to 24) of unsigned(4 downto 0);
    signal s_G    : t_g5 := (others => (others => '0'));
    signal s20_weff : unsigned(12 downto 0) := (others => '0');
    signal s20_dh : unsigned(10 downto 0) := (others => '0');
    signal s20_af : unsigned(11 downto 0) := (others => '0');
    signal s20_dsh, s21_d, s22_dd : signed(12 downto 0) := (others => '0');
    signal s21_kk, s22_kk : unsigned(3 downto 0) := (others => '0');
    signal s21_half : std_logic := '0';
    type t_fd is array (20 to 23) of unsigned(6 downto 0);          -- steps flag & f(11 downto 6)
    signal s_fd   : t_fd := (others => (others => '0'));
    signal s23_cov : unsigned(4 downto 0) := (others => '0');
    signal s24_t, s25_tone : unsigned(6 downto 0) := (others => '0');

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_l9  : unsigned(8 downto 0) := (others => '0');
    signal s_g8  : unsigned(7 downto 0) := (others => '1');
    signal s_p, s_sfirst : unsigned(12 downto 0) := (others => '0');
    signal s_sv, s_sv1, s_svd, s_bv, s_ev : std_logic := '0';
    signal s_col, s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    type t_bx is array (0 to 255) of unsigned(15 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(15 downto 0) := (others => '0');
    signal s_bcnt : unsigned(7 downto 0) := (others => '0');
    signal s_bsum : unsigned(19 downto 0) := (others => '0');
    signal s_eb  : unsigned(12 downto 0) := (others => '0');
    signal s_eq  : unsigned(15 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal s_upd, c_a, c_b, c_dy, s_Ls, s_wcam : unsigned(12 downto 0) := (others => '0');
    signal c_new : std_logic := '0';

    -- [F] output conditioning
    signal s27_y, s28_y, s29_y, s30_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s30_ya : unsigned(11 downto 0) := (others => '0');
    signal s31_y, s32_y, s33_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_SC_EXP(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            yr_q   <= C_SC_Y(to_integer(s25_tone));
            sq0_q  <= C_SC_SQ(to_integer(s_ptd & s4_a0(12 downto 5)));
            sq1_q  <= C_SC_SQ(to_integer(s_ptd & s4_a1(12 downto 5)));
            rtv_q  <= C_SC_RTV(to_integer(s12_sx(16 downto 8)));
            rts_q  <= C_SC_RTS(to_integer(s12_sx(16 downto 8)));
        end if;
    end process p_rom;

    -- box delay: read N samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv1 = '1' then bx_ram(to_integer(bx_wp)) <= resize(s_p, 16); end if;
        end if;
    end process p_bxram;

    -- luma line store (half horizontal resolution): the one-pole's own read, 2 addresses ahead of its write
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q <= lb_ram(to_integer(s_col(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    w_dxm  <= to_unsigned(1432, 11) when s_xs = 4 else to_unsigned(1914, 11) when s_xs = 3 else to_unsigned(1916, 11);
    w_Sq   <= c_S(25 downto 10);
    w_CH12 <= c_CH(26 downto 12);
    w_yq   <= e_yrem(25 downto 10);
    w_thr  <= signed(resize(c_Dq, 20)) + 65536;

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + [C] the per-field
    -- sequencer (one small operation per zone clock; the zone is quasi-static
    -- but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k  : unsigned(9 downto 0);
        variable v_c  : signed(10 downto 0);
        variable v_x  : unsigned(15 downto 0);
        variable v_w  : unsigned(16 downto 0);
        variable v_d2 : unsigned(15 downto 0);
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

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_ptd   <= registers_in(6)(1);
                s_shade <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                if s_xsr = 2 then s_lg7 <= '1'; else s_lg7 <= '0'; end if;
                if s_ys <= 384 then s_ish <= to_unsigned(5, 3);
                elsif s_ys >= 960 then s_ish <= to_unsigned(3, 3);
                else s_ish <= to_unsigned(4, 3); end if;
                -- rings: 8 steps with hysteresis (leave a step only 8 past its edge)
                v_k := unsigned(registers_in(3));
                v_c := signed(resize(v_k, 11)) - signed(resize(s_n0 & "1000000", 11));
                if v_c > 72 or v_c < -72 then s_n0 <= v_k(9 downto 7); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- [B] shared glide, four clocks per channel.  A knob within 2 counts of its accumulator is
            -- held (pot jitter).  On: 1/16 of the gap per field, never under half a count.  Off: the
            -- accumulator takes the knob, so this field already uses it.
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_a1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_a2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_a3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_a5; s_gr <= unsigned(registers_in(4));
                            when 4      => s_gm <= s_a6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_ap; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000000", 17)) - signed(resize(s_gm, 17));
                        z_gp <= "01";
                    when "01" =>
                        if s_gd(16 downto 7) = "0000000000" or s_gd(16 downto 7) = "1111111111" then
                            s_gd <= (others => '0');
                        elsif s_glide = '0' then
                            null;
                        elsif s_gd(16 downto 9) = "00000000" then
                            s_gd <= to_signed(32, 17);
                        elsif s_gd(16 downto 9) = "11111111" then
                            s_gd <= to_signed(-32, 17);
                        else
                            s_gd <= shift_right(s_gd, 4);
                        end if;
                        z_gp <= "10";
                    when others =>
                        v_x := unsigned(signed(s_gm) + s_gd(15 downto 0));
                        case to_integer(z_gc) is
                            when 0      => s_a1 <= v_x;
                            when 1      => s_a2 <= v_x;
                            when 2      => s_a3 <= v_x;
                            when 3      => s_a5 <= v_x;
                            when 4      => s_a6 <= v_x;
                            when others => s_ap <= v_x;
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] serial products: a slot is decoded at zc = P (below), its operands are pre-muxed one
            -- clock later, it loads the clock after that, runs 16 iterations and is stored at P + 20.
            -- A shift by a knob-derived amount is a product with a one-hot operand.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 46 => z_sel <= to_unsigned(0, 4); z_pre <= '1';
                    when 70 => z_sel <= to_unsigned(1, 4); z_pre <= '1';
                    when 118 => z_sel <= to_unsigned(2, 4); z_pre <= '1';
                    when 142 => z_sel <= to_unsigned(3, 4); z_pre <= '1';
                    when 166 => z_sel <= to_unsigned(4, 4); z_pre <= '1';
                    when 190 => z_sel <= to_unsigned(5, 4); z_pre <= '1';
                    when 214 => z_sel <= to_unsigned(6, 4); z_pre <= '1';
                    when 238 => z_sel <= to_unsigned(7, 4); z_pre <= '1';
                    when 262 => z_sel <= to_unsigned(8, 4); z_pre <= '1';
                    when 286 => z_sel <= to_unsigned(9, 4); z_pre <= '1';
                    when 310 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 334 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when 358 => z_sel <= to_unsigned(12, 4); z_pre <= '1';
                    when 382 => z_sel <= to_unsigned(13, 4); z_pre <= '1';
                    when 454 => z_sel <= to_unsigned(14, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_a3(15 downto 3), 32); z_fb <= to_unsigned(1001, 16);     -- octaves of R
                    when 1  => z_fa <= resize(s_ap(15 downto 3), 32); z_fb <= to_unsigned(769, 16);      -- octaves of the fan
                    when 2 | 3 | 4 | 10 =>                                                               -- mantissa << shift
                        z_fa <= resize(r_exf, 32); z_fb <= shift_left(to_unsigned(1, 16), to_integer(z_e(11 downto 8)));
                    when 5  => z_fa <= unsigned(resize(z_A, 32)); z_fb <= r_exf;                         -- pre-pixel, cells
                    when 6 | 9 =>
                        z_fa <= f_acc; z_fb <= shift_left(to_unsigned(1, 16), to_integer(z_e(11 downto 8)));
                    when 7  => z_fa <= resize(c_CH(26 downto 11), 32); z_fb <= z_xc;                     -- its place in the cell
                    when 8  => z_fa <= unsigned(resize(z_B, 32)); z_fb <= r_exf;                         -- line 0, row pairs
                    when 11 => z_fa <= resize(w_Sq, 32); z_fb <= z_wp(18 downto 3);                      -- its row remainder
                    when 12 => z_fa <= resize(c_DUh, 32); z_fb <= resize(s_xs, 16);                      -- per pixel
                    when 13 => z_fa <= resize(c_DUh, 32); z_fb <= resize(s_ys, 16);                      -- per field line
                    when others => z_fa <= resize(c_O16, 32); z_fb <= resize(c_n, 16);                   -- keyline in bands
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            elsif e_mld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(e_ma, 32); f_b <= resize(e_mb, 16);
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- the exponent accumulator: one adder; the ROM index is its low byte, the shift its high bits
            z_eop <= '0';
            if z_eop = '1' then z_e <= f_addsub(z_e, z_ex, z_esub); end if;

            e_zk <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- pan: +-960 virtual px = +-1920 h for +-512 counts: (knob - 512) * 15 in Q5 h
                    when 30  => z_p <= signed(resize(s_a1(15 downto 3), 14)) - 4096;
                    when 31  => z_A <= shift_left(resize(z_p, 19), 4) - resize(z_p, 19);
                    when 32  => z_A <= z_A + signed(resize(w_dxm & "00000", 19));
                    when 33  => z_p <= signed(resize(s_a2(15 downto 3), 14)) - 4096;
                    when 34  => z_B <= shift_left(resize(z_p, 19), 4) - resize(z_p, 19);
                    when 35  => z_B <= z_B + to_signed(34560, 19);
                    when 36  => s_g8 <= s_a6(15 downto 8);

                    when 66  => z_oR <= f_acc(23 downto 13);
                    -- rho-units per h: 2^(-(oR + oF + 850) / 256) (<< 11: Q26)
                    when 67  => z_e <= to_unsigned(1966, 12); z_ex <= resize(z_oR, 12); z_esub <= '1'; z_eop <= '1';
                    when 90  => z_oF <= f_acc(23 downto 13);
                    when 91  => z_ex <= resize(z_oF, 12); z_esub <= '1'; z_eop <= '1';
                    when 93  => r_ex_a <= z_e(7 downto 0);
                    when 138 => c_DUh <= f_acc(22 downto 2);
                    -- half a cell = 1 / fan: 2^(2 - oF / 256), Q24
                    when 120 => z_e <= to_unsigned(2816, 12); z_ex <= resize(z_oF, 12); z_esub <= '1'; z_eop <= '1';
                    when 123 => r_ex_a <= z_e(7 downto 0);
                    when 162 => c_CH <= f_acc(26 downto 0);
                    -- row step = overlap / fan: 2^((oO - oF) / 256), Q24
                    when 144 => z_e <= to_unsigned(2304, 12); z_ex <= resize(z_oF, 12); z_esub <= '1'; z_eop <= '1';
                    when 145 => z_ex <= resize(s_a5(15 downto 7), 12); z_esub <= '0'; z_eop <= '1';
                    when 148 => r_ex_a <= z_e(7 downto 0);
                    when 186 => c_S <= f_acc(25 downto 0);
                    -- cells per h: 2^(-(oR + 1618) / 256)
                    when 168 => z_e <= to_unsigned(1198, 12); z_ex <= resize(z_oR, 12); z_esub <= '1'; z_eop <= '1';
                    when 171 => r_ex_a <= z_e(7 downto 0);
                    when 234 => z_xc <= to_unsigned(32768, 16) - f_acc(30 downto 15);
                    when 258 => c_r0 <= signed(resize(f_acc(31 downto 4), 28)) - signed(resize(c_CH & '0', 28));   -- u0 - C
                    -- row pairs per h: 2^(-(oR + oO + 1106) / 256)
                    when 240 => z_e <= to_unsigned(1710, 12); z_ex <= resize(z_oR, 12); z_esub <= '1'; z_eop <= '1';
                    when 241 => z_ex <= resize(s_a5(15 downto 7), 12); z_esub <= '1'; z_eop <= '1';
                    when 244 => r_ex_a <= z_e(7 downto 0);
                    when 306 => z_wpB <= f_acc(30 downto 11);
                    -- rho / 2s: 2^((oF - oO - 256) / 256), Q20
                    when 288 => z_e <= to_unsigned(1024, 12); z_ex <= resize(z_oF, 12); z_esub <= '0'; z_eop <= '1';
                    when 289 => z_ex <= resize(s_a5(15 downto 7), 12); z_esub <= '1'; z_eop <= '1';
                    when 292 => r_ex_a <= z_e(7 downto 0);
                    when 330 => z_wp <= f_acc(19 downto 0) - z_wpB;
                    when 354 => c_yrem0 <= f_acc(31 downto 6); c_pb0 <= z_wp(19);
                    when 378 => c_DUP <= f_acc(22 downto 0);
                    when 402 => c_DV <= f_acc(31 downto 7);
                    -- one pixel (the field line) in rho-units, Q16, at most 1/4 ...
                    when 404 =>
                        if c_DV(24 downto 22) /= "000" then z_M <= to_unsigned(16384, 16); c_DW <= to_unsigned(16384, 15);
                        else z_M <= resize(c_DV(22 downto 8), 16); c_DW <= c_DV(22 downto 8); end if;
                        z_lg <= to_unsigned(15, 4);
                    -- ... normalised (405 .. 420) and rounded up to 2^k or 2^k * 32/23 (421 .. 437)
                    when 421 =>
                        if z_M(14 downto 0) = 0 then
                            c_kq <= z_lg; c_half <= '0'; z_D <= to_unsigned(32768, 17);
                        elsif z_M <= 45568 then
                            c_kq <= z_lg; c_half <= '1'; z_D <= to_unsigned(45568, 17);
                        else
                            c_kq <= z_lg + 1; c_half <= '0'; z_D <= to_unsigned(65536, 17);
                        end if;
                        z_c <= 15 - z_lg;
                    -- rings: as many as leave 3 pixels per band (439 .. 445)
                    when 438 =>
                        c_D16 <= z_D(15 downto 0); c_n <= to_unsigned(1, 4); z_w <= resize(c_DW, 16);
                        for i in 0 to 19 loop
                            if i >= to_integer(c_kq) then c_msk(i) <= '1'; else c_msk(i) <= '0'; end if;
                        end loop;
                    when 446 =>
                        c_wb <= resize(z_w(15 downto 4), 13);
                        z_inv <= C_SC_INVN(to_integer(c_n(2 downto 0) - 1));
                    when 447 =>                         -- keyline: half a band (Bold flat), else 1/8 band ...
                        if s_shade = '0' and s_fine = '0' then z_nom <= z_inv(16 downto 1);
                        else z_nom <= resize(z_inv(16 downto 3), 16); end if;
                    when 448 =>                         -- ... never more than 1/2 (1/4) of the row step ...
                        if s_shade = '0' and s_fine = '0' then
                            if resize(z_nom, 17) > c_S(25 downto 9) then z_nom <= c_S(24 downto 9); end if;
                        else
                            if z_nom > c_S(25 downto 10) then z_nom <= c_S(25 downto 10); end if;
                        end if;
                    when 449 =>                         -- ... and at least 2 pixels
                        v_d2 := c_D16(14 downto 0) & '0';
                        if v_d2 > z_nom then c_O16 <= v_d2; else c_O16 <= z_nom; end if;
                    when 474 => z_ow <= f_acc(15 downto 4);
                    when 475 =>
                        if s_fine = '1' then c_offF <= to_unsigned(1707, 12) - z_ow; c_dhn <= to_unsigned(341, 11);
                        else c_offF <= to_unsigned(1024, 12) - z_ow; c_dhn <= to_unsigned(1024, 11); end if;
                        if s_ptd = '1' then c_Dq <= c_D16; else c_Dq <= c_D16(14 downto 0) & '0'; end if;
                    when 476 =>
                        if s_ptd = '1' then c_Dk <= resize(c_O16(15 downto 1), 17) + resize(c_Dq, 17);
                        else c_Dk <= resize(c_O16, 17) + resize(c_Dq, 17); end if;
                    when 477 => c_dupc <= signed(resize(c_DUP, 28)) - signed(resize(c_CH & '0', 28));
                    when 494 => e_zk <= '1';            -- line 0
                    when others => null;
                end case;
                -- normalise the pixel: shift up until the top bit is set
                if s_zc >= 405 and s_zc <= 420 then
                    if z_M(15) = '0' then z_M <= z_M(14 downto 0) & '0'; z_lg <= z_lg - 1; end if;
                end if;
                -- and shift the rounded value back down
                if s_zc >= 422 and s_zc <= 437 then
                    if z_c /= 0 then z_D <= '0' & z_D(16 downto 1); z_c <= z_c - 1; end if;
                end if;
                -- ring count
                if s_zc >= 439 and s_zc <= 445 then
                    v_w := resize(z_w, 17) + resize(c_DW, 17);
                    if c_n <= resize(s_n0, 4) and v_w <= 21845 then
                        z_w <= v_w(15 downto 0); c_n <= c_n + 1;
                    end if;
                end if;
            end if;

            -- [A] sync delay and generated-avid shift registers (field_n is tied high by the core on every
            -- input, so it only gets a register)
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
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
    -- [L] LINE ENGINE: started at each line end (advance one field line) and
    -- once in the zone (line 0).  With t_j = yrem + j S (the pixel's height above
    -- candidate row j's centre, plus one radius), |dy_j| = |t_j - 1| (Q12, far
    -- from 1.25 up) and c_m(j) = dy_j^2 (Round) or |dy_j| (Pointed: a product
    -- with 1.0) minus the near-hit threshold, one product per candidate on the
    -- shared multiplier.  c_m is written 19 .. 61 clocks after the line end,
    -- after the last pixel has used the old values.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y  : unsigned(27 downto 0);
        variable v_ab : unsigned(18 downto 0);
        variable v_q  : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            e_mld <= '0';
            if e_zk = '1' then
                e_yrem <= resize(c_yrem0, 27); e_pb <= c_pb0;
                e_st <= to_unsigned(2, 7);
            elsif g_avid = '0' and s_gav_sr(0) = '1' then
                e_st <= (others => '0');
            elsif e_st /= 127 then
                e_st <= e_st + 1;
            end if;
            if e_mld = '1' then e_farc <= e_far; end if;

            if e_farc = '1' then v_q := to_signed(131072, 20) - w_thr;
            else v_q := signed(resize(f_acc(25 downto 8), 20)) - w_thr; end if;

            case to_integer(e_st) is
                when 0 =>
                    e_yrem <= e_yrem + resize(c_DV, 27);
                when 1 =>
                    v_y := resize(e_yrem, 28) - resize(c_S, 28);
                    if v_y(27) = '0' then e_yrem <= v_y(26 downto 0); e_pb <= not e_pb; end if;
                when 2 =>
                    e_t <= resize(w_yq, 18);
                when 3 | 17 | 31 | 45 =>
                    e_v <= signed(resize(e_t, 19)) - 16384;
                when 4 | 18 | 32 | 46 =>
                    if e_v(18) = '1' then v_ab := unsigned(-e_v); else v_ab := unsigned(e_v); end if;
                    if v_ab >= 20480 then e_far <= '1'; else e_far <= '0'; end if;
                    e_ma <= v_ab(14 downto 2);
                    if s_ptd = '1' then e_mb <= to_unsigned(4096, 13); else e_mb <= v_ab(14 downto 2); end if;
                    e_mld <= '1';
                    e_t <= e_t + resize(w_Sq, 18);
                when others => null;
            end case;
            case to_integer(e_st) is
                when 4  => c_pb <= e_pb;
                when 19 => c_m(0) <= v_q;
                when 33 => c_m(1) <= v_q;
                when 47 => c_m(2) <= v_q;
                when 61 => c_m(3) <= v_q;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [P] PIXEL PATH
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_t1, v_t2, v_dx : signed(27 downto 0);
        variable v_ab  : unsigned(15 downto 0);
        variable v_ae, v_ao : unsigned(12 downto 0);
        variable v_od  : signed(16 downto 0);
        variable v_m0, v_m1 : unsigned(7 downto 0);
        variable v_nh  : std_logic_vector(3 downto 0);
        variable v_f, v_b : integer range 0 to 3;
        variable v_hb, v_deep : std_logic;
        variable v_dp, v_sc, v_ee : signed(19 downto 0);
        variable v_pr  : unsigned(13 downto 0);
        variable v_g0, v_g1, v_g2, v_g3, v_g4 : unsigned(13 downto 0);
        variable v_dr  : unsigned(12 downto 0);
        variable v_vc  : unsigned(11 downto 0);
        variable v_n0, v_n1, v_n2 : unsigned(15 downto 0);
        variable v_xs  : signed(6 downto 0);
        variable v_gs  : signed(8 downto 0);
        variable v_f12, v_g12, v_q12 : unsigned(11 downto 0);
        variable v_w5  : unsigned(15 downto 0);
        variable v_hi  : unsigned(11 downto 0);
        variable v_dh  : unsigned(10 downto 0);
        variable v_wide : std_logic;
        variable v_p   : unsigned(3 downto 0);
        variable v_nx  : std_logic;
        variable v_sh, v_cs : signed(16 downto 0);
        variable v_c4, v_ramp, v_st : unsigned(6 downto 0);
        variable v_mq  : unsigned(1 downto 0);
        variable v_k9  : unsigned(8 downto 0);
        variable v_g4x : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: the beam, r = u - C in [-C, 0): one wrap per step at most
            if g_start = '1' then
                b_r <= c_r0;
            else
                v_t1 := b_r + signed(resize(c_DUP, 28));
                v_t2 := b_r + c_dupc;
                if v_t1(27) = '0' then b_r <= v_t2; else b_r <= v_t1; end if;
            end if;
            -- 3: dx to the even-parity centre (the cell's middle), Q3.12
            v_dx := b_r + signed(resize(c_CH, 28));
            s3_dxe <= v_dx(27 downto 12);
            -- 4: |dx| for both row parities, saturated under 2.0; slot 0 = the lowest candidate row's parity
            if s3_dxe(15) = '1' then v_ab := unsigned(-s3_dxe); else v_ab := unsigned(s3_dxe); end if;
            v_od := signed(resize(w_CH12, 17)) - signed(resize(v_ab, 17));
            if v_ab(15 downto 13) /= "000" then v_ae := (others => '1'); else v_ae := v_ab(12 downto 0); end if;
            if v_od(16) = '1' then v_ao := (others => '0');
            elsif v_od(15 downto 13) /= "000" then v_ao := (others => '1');
            else v_ao := unsigned(v_od(12 downto 0)); end if;
            if c_pb = '1' then s4_a0 <= v_ao; s4_a1 <= v_ae;
            else s4_a0 <= v_ae; s4_a1 <= v_ao; end if;
            -- 5 .. 7: dx^2, Q2.16 = (h^2 << 2) + (h l >> 2) with dx = 32 h + l (the ROM, p_rom: stage 5);
            -- Pointed: the ROM's second page and a product with 64 give |dx| << 4 instead
            if s_ptd = '1' then v_m0 := to_unsigned(64, 8); v_m1 := to_unsigned(64, 8);
            else v_m0 := s4_a0(12 downto 5); v_m1 := s4_a1(12 downto 5); end if;
            s5_pr0 <= f_pp5(v_m0, s4_a0(4 downto 0));
            s5_pr1 <= f_pp5(v_m1, s4_a1(4 downto 0));
            s6_pr0 <= s5_pr0; s6_pr1 <= s5_pr1;
            s6_hs0 <= sq0_q; s6_hs1 <= sq1_q;
            s7_q0 <= resize(s6_hs0 & "00", 18) + resize(s6_pr0, 18);
            s7_q1 <= resize(s6_hs1 & "00", 18) + resize(s6_pr1, 18);
            -- 8: the four candidates (negative = inside the scale, or within a pixel outside its rim)
            s8_s(0) <= signed(resize(s7_q0, 20)) + c_m(0);
            s8_s(1) <= signed(resize(s7_q1, 20)) + c_m(1);
            s8_s(2) <= signed(resize(s7_q0, 20)) + c_m(2);
            s8_s(3) <= signed(resize(s7_q1, 20)) + c_m(3);
            -- 9: the front scale (lowest row) and the next one behind it
            v_nh := s8_s(3)(19) & s8_s(2)(19) & s8_s(1)(19) & s8_s(0)(19);
            v_f := 3; v_b := 3; v_hb := '0';
            case v_nh is
                when "0001" => v_f := 0;
                when "0010" => v_f := 1;
                when "0100" => v_f := 2;
                when "1000" => v_f := 3;
                when "0011" | "0111" | "1011" | "1111" => v_f := 0; v_b := 1; v_hb := '1';
                when "0101" | "1101" => v_f := 0; v_b := 2; v_hb := '1';
                when "1001" => v_f := 0; v_b := 3; v_hb := '1';
                when "0110" | "1110" => v_f := 1; v_b := 2; v_hb := '1';
                when "1010" => v_f := 1; v_b := 3; v_hb := '1';
                when "1100" => v_f := 2; v_b := 3; v_hb := '1';
                when others => null;
            end case;
            s9_sf <= s8_s(v_f); s9_sb <= s8_s(v_b);
            if v_nh = "0000" then s9_hasf <= '0'; else s9_hasf <= '1'; end if;
            s9_hasb <= v_hb;
            -- 10: past the middle of the front scale's keyline, the front scale inks the pixel; before it
            -- (and out to a pixel beyond the rim) the one behind does, under the front rim's coverage
            v_dp := s9_sf + signed(resize(c_Dk, 20));
            v_deep := s9_hasf and v_dp(19);
            if v_deep = '1' then s10_sel <= s9_sf; else s10_sel <= s9_sb; end if;
            s10_e1 <= s9_sf + signed(resize(c_Dq, 20));
            s10_none <= (not s9_hasf) or ((not v_deep) and (not s9_hasb));
            s10_isback <= s9_hasf and not v_deep;
            -- 11: the inking scale's sum; the front rim's distance (its edge sits half a pixel outside d = 1)
            v_sc := s10_sel + w_thr;
            if s10_none = '1' then s11_sc <= (others => '0'); else s11_sc <= unsigned(v_sc(16 downto 0)); end if;
            if s_ptd = '1' then v_ee := s10_e1; else v_ee := shift_right(s10_e1, 1); end if;
            v_ee := v_ee - signed(resize(c_D16(15 downto 1), 20));
            if s10_none = '1' then s_ee(11) <= to_signed(-64, 7);
            elsif s10_isback = '1' then s_ee(11) <= f_es7(v_ee, c_kq, c_msk);
            else s_ee(11) <= to_signed(63, 7); end if;
            for i in 12 to 19 loop s_ee(i) <= s_ee(i - 1); end loop;
            -- 12: small sums are scaled up by 16 or 256 before the root (and the root down by 4 or 16)
            if s_ptd = '1' then
                s12_sx <= s11_sc; s12_rr <= "00";
            elsif s11_sc(16 downto 8) = 0 then
                s12_sx <= s11_sc(8 downto 0) & "00000000"; s12_rr <= "10";
            elsif s11_sc(16 downto 12) = 0 then
                s12_sx <= s11_sc(12 downto 0) & "0000"; s12_rr <= "01";
            else
                s12_sx <= s11_sc; s12_rr <= "00";
            end if;
            -- 13: the root ROM (p_rom); 14: fabric copies (Pointed: the sum itself is the distance)
            s13_x <= s12_sx(16 downto 2); s13_rr <= s12_rr;
            if s_ptd = '1' then s14_val <= s13_x(14 downto 2); s14_slope <= (others => '0');
            else s14_val <= rtv_q(12 downto 0); s14_slope <= rts_q; end if;
            s14_fr <= s13_x(5 downto 1); s14_rr <= s13_rr;
            -- 15: slope * fraction (8 x 5)
            v_g0 := (others => '0'); v_g1 := (others => '0'); v_g2 := (others => '0');
            v_g3 := (others => '0'); v_g4 := (others => '0');
            if s14_fr(0) = '1' then v_g0 := resize(s14_slope, 14); end if;
            if s14_fr(1) = '1' then v_g1 := resize(s14_slope & "0", 14); end if;
            if s14_fr(2) = '1' then v_g2 := resize(s14_slope & "00", 14); end if;
            if s14_fr(3) = '1' then v_g3 := resize(s14_slope & "000", 14); end if;
            if s14_fr(4) = '1' then v_g4 := resize(s14_slope & "0000", 14) + 16; else v_g4 := to_unsigned(16, 14); end if;
            v_pr := ((v_g0 + v_g1) + (v_g2 + v_g3)) + v_g4;
            s15_pr <= v_pr(12 downto 5);
            s15_val <= s14_val; s15_rr <= s14_rr;
            -- 16: d, Q1.12
            v_dr := s15_val + resize(s15_pr, 13);
            case s15_rr is
                when "10"   => s16_d <= resize(v_dr(12 downto 4), 13);
                when "01"   => s16_d <= resize(v_dr(12 downto 2), 13);
                when others => s16_d <= v_dr;
            end case;
            -- 17: v = 1 - d
            s17_v <= to_signed(4096, 14) - signed(resize(s16_d, 14));
            -- 18: ring phase n v (bands, Q12) and the distance inside the keyline's inner edge (Q16)
            if s17_v(13) = '1' then v_vc := (others => '0');
            elsif s17_v(12) = '1' then v_vc := (others => '1');
            else v_vc := unsigned(s17_v(11 downto 0)); end if;
            v_n0 := (others => '0'); v_n1 := (others => '0'); v_n2 := (others => '0');
            if c_n(0) = '1' then v_n0 := resize(v_vc, 16); end if;
            if c_n(1) = '1' then v_n1 := resize(v_vc & "0", 16); end if;
            if c_n(2) = '1' then v_n2 := resize(v_vc & "00", 16); end if;
            if c_n(3) = '1' then s18_phib <= resize(v_vc & "000", 16);
            else s18_phib <= (v_n0 + v_n1) + v_n2; end if;
            s18_eown <= shift_left(resize(s17_v, 20), 4) - signed(resize(c_O16, 20));
            -- 19: + the camera's phase; bands per pixel (the rings' or the camera's, whichever is steeper)
            s19_phi <= s18_phib + resize(s_Ls & "00", 16);
            if s_wcam > c_wb then s19_w <= s_wcam; else s19_w <= c_wb; end if;
            s19_eos <= f_es7(s18_eown, c_kq, c_msk);
            -- 20: the smaller of the two keyline distances; 21: G = clamp(1/2 + that), 17 steps
            if s_ee(19) < s19_eos then s20_x <= s_ee(19); else s20_x <= s19_eos; end if;
            if s_ee(19) = -64 then s20_none <= '1'; else s20_none <= '0'; end if;
            if c_half = '1' then v_xs := s20_x - shift_right(s20_x, 2) - shift_right(s20_x, 5); else v_xs := s20_x; end if;
            v_gs := shift_right(resize(v_xs, 9) + 2, 2) + 8;
            if s20_none = '1' or v_gs(8) = '1' then s_G(21) <= (others => '0');
            elsif v_gs > 16 then s_G(21) <= to_unsigned(16, 5);
            else s_G(21) <= unsigned(v_gs(4 downto 0)); end if;
            for i in 22 to 24 loop s_G(i) <= s_G(i - 1); end loop;

            -- 20: the tone's edge: band-centre form (Flat) or the signed distance to the nearest ramp
            -- edge (Shade); past 1/4 band per pixel the transition widens so the bands fade to grey
            v_f12 := s19_phi(11 downto 0);
            if s19_w(12 downto 10) /= "000" then v_wide := '1'; else v_wide := '0'; end if;
            if v_wide = '1' then
                v_w5 := shift_left(resize(s19_w, 16), 2) + resize(s19_w, 16) - 4096;
                if v_w5(15 downto 13) /= "000" then s20_weff <= (others => '1'); else s20_weff <= v_w5(12 downto 0); end if;
                s20_dh <= to_unsigned(1024, 11);
            else
                s20_weff <= s19_w;
                v_hi := to_unsigned(2048, 12) - resize(s19_w, 12);
                v_dh := c_dhn;
                if v_dh < s19_w(10 downto 0) then s20_dh <= s19_w(10 downto 0);
                elsif resize(v_dh, 12) > v_hi then s20_dh <= v_hi(10 downto 0);
                else s20_dh <= v_dh; end if;
            end if;
            v_g12 := v_f12 + c_offF;
            if v_g12(11) = '1' then s20_af <= '0' & v_g12(10 downto 0);
            else s20_af <= to_unsigned(2048, 12) - resize(v_g12(10 downto 0), 12); end if;
            v_q12 := v_f12 + 512;
            if s_shade = '1' and s_fine = '0' and v_wide = '0' then
                s20_dsh <= signed(resize(v_q12(9 downto 0), 13)) - 512;
                s_fd(20) <= '1' & v_f12(11 downto 6);
            else
                s20_dsh <= resize(signed(v_f12), 13);
                s_fd(20) <= '0' & v_f12(11 downto 6);
            end if;
            for i in 21 to 23 loop s_fd(i) <= s_fd(i - 1); end loop;
            -- 21: d; the divisor: weff rounded up to 2^(p + 1), or to 2^p * 32/23 when its second bit is clear
            if s_shade = '1' then s21_d <= s20_dsh;
            else s21_d <= signed(resize(s20_dh, 13)) - signed(resize(s20_af, 13)); end if;
            v_p := (others => '0'); v_nx := '0';
            for i in 1 to 12 loop
                if s20_weff(i) = '1' then v_p := to_unsigned(i, 4); v_nx := s20_weff(i - 1); end if;
            end loop;
            if v_nx = '1' then s21_kk <= v_p + 1; else s21_kk <= v_p; end if;
            s21_half <= not v_nx;
            -- 22: d * 23/32 for the in-between divisor
            if s21_half = '1' then s22_dd <= s21_d - shift_right(s21_d, 2) - shift_right(s21_d, 5);
            else s22_dd <= s21_d; end if;
            s22_kk <= s21_kk;
            -- 23: coverage = clamp(1/2 + d / divisor), 17 steps
            v_sh := shift_right(shift_left(resize(s22_dd, 17), 4), to_integer(s22_kk));
            if v_sh < -8 then s23_cov <= (others => '0');
            elsif v_sh > 8 then s23_cov <= to_unsigned(16, 5);
            else v_cs := v_sh + 8; s23_cov <= unsigned(v_cs(4 downto 0)); end if;
            -- 24: tone 0 .. 64
            v_c4 := s23_cov & "00";
            v_ramp := to_unsigned(64, 7) - resize(s_fd(23)(5 downto 0), 7);
            v_mq := resize(shift_right(resize(s_fd(23)(5 downto 3), 4) + 1, 1), 2);
            if v_mq = 2 then v_k9 := resize(s23_cov & "0000", 9) + resize(s23_cov & "00", 9) + resize(s23_cov & "0", 9) + 8;
            else v_k9 := resize(s23_cov & "0000", 9) + resize(s23_cov & "00", 9) + resize(s23_cov, 9) + 8; end if;
            case v_mq is
                when "00"   => v_st := v_c4;
                when "01"   => v_st := to_unsigned(64, 7) - resize(v_k9(8 downto 4), 7);
                when "10"   => v_st := to_unsigned(43, 7) - resize(v_k9(8 downto 4), 7);
                when others => v_st := to_unsigned(21, 7) - resize(v_k9(8 downto 4), 7);
            end case;
            if s_shade = '0' then
                s24_t <= v_c4;
            elsif s_fd(23)(6) = '1' then
                s24_t <= v_st;
            elsif s_fd(23)(5) = '0' then
                if v_ramp < v_c4 then s24_t <= v_ramp; else s24_t <= v_c4; end if;
            else
                if v_ramp > v_c4 then s24_t <= v_ramp; else s24_t <= v_c4; end if;
            end if;
            -- 25: under the keyline
            v_g4x := s_G(24) & "00";
            if s24_t < v_g4x then s25_tone <= s24_t; else s25_tone <= v_g4x; end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice, doubled) * K6 -> box of 128
    -- (1080) or 64 samples, edge-replicated and reloaded at every line start ->
    -- vertical one-pole in the line store (half horizontal resolution, Q13.3).
    -- The pixel path takes the one-pole's stream as it is written (the camera
    -- is never shown, so it needs no alignment): L = phase in 2^-10 band,
    -- wcam = |its x step| + |its y step| in bands per pixel.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(12 downto 0);
        variable v_pr : unsigned(16 downto 0);
        variable v_d : signed(16 downto 0);
        variable v_u, v_n : signed(16 downto 0);
        variable v_df : unsigned(12 downto 0);
        variable v_wc : unsigned(14 downto 0);
        variable v_mn : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting
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

            -- the source sample: (luma >> 2) - 16, or the lattice at twice the level
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_l9 <= s_osum & '0';
            elsif v_l > 16 then s_l9 <= resize(v_l - 16, 9);
            else s_l9 <= (others => '0'); end if;
            s_sv <= data_in.avid;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;
            -- * K6 (a stage to itself)
            v_pr := s_l9 * s_g8;
            s_p <= v_pr(16 downto 4);
            s_sv1 <= s_sv; s_svd <= s_sv1;

            -- box: sum of the last N samples; the samples before the line start count as the first one
            if s_sv1 = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else
                bx_wp <= (others => '0');
                if s_lg7 = '1' then bx_rp <= to_unsigned(129, 8); else bx_rp <= to_unsigned(193, 8); end if;
            end if;
            if (s_lg7 = '1' and s_bcnt(7) = '0') or (s_lg7 = '0' and s_bcnt(7 downto 6) = "00") then v_tap := s_sfirst;
            else v_tap := bx_q(12 downto 0); end if;
            if s_sv1 = '1' and s_svd = '0' then
                s_sfirst <= s_p;
                if s_lg7 = '1' then s_bsum <= resize(s_p & "0000000", 20); else s_bsum <= resize(s_p & "000000", 20); end if;
                s_bcnt <= to_unsigned(1, 8);
            elsif s_sv1 = '1' then
                s_bsum <= s_bsum + s_p - v_tap;
                if (s_lg7 = '1' and s_bcnt(7) = '0') or (s_lg7 = '0' and s_bcnt(7 downto 6) = "00") then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bv <= s_sv1; s_bcol <= s_col;

            -- vertical one-pole on the odd columns: q += (box - q) >> sh
            if s_lg7 = '1' then s_eb <= s_bsum(19 downto 7); else s_eb <= s_bsum(18 downto 6); end if;
            s_eq <= lb_q;
            s_ev <= s_bv and not s_bcol(0); s_ecol <= s_bcol;
            v_d := signed(resize(s_eb & "000", 17)) - signed(resize(s_eq, 17));
            case to_integer(s_ish) is
                when 3      => v_u := shift_right(v_d, 3);
                when 4      => v_u := shift_right(v_d, 4);
                when others => v_u := shift_right(v_d, 5);
            end case;
            lb_wd <= unsigned(signed(s_eq) + v_u(15 downto 0));
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);
            v_n := -v_u;
            if v_u(16) = '1' then s_upd <= unsigned(v_n(12 downto 0)); else s_upd <= unsigned(v_u(12 downto 0)); end if;

            -- the stream to the pixel path: the latest and the previous column's values, their mean on the
            -- clock in between
            c_new <= lb_we;
            if lb_we = '1' then
                c_b <= c_a; c_a <= lb_wd(15 downto 3); c_dy <= s_upd;
            end if;
            v_mn := resize(c_a, 14) + resize(c_b, 14);
            if c_new = '1' then s_Ls <= v_mn(13 downto 1); else s_Ls <= c_a; end if;
            if c_a > c_b then v_df := c_a - c_b; else v_df := c_b - c_a; end if;
            v_wc := resize(v_df & '0', 15) + resize(c_dy(12 downto 1), 15);
            if v_wc(14 downto 13) /= "00" then s_wcam <= (others => '1'); else s_wcam <= v_wc(12 downto 0); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: tone -> luma (ROM, stage 26), black outside pixels
    -- 0 .. W + 1 (27), luma [1 2 1] / 4 (30, 31), 13/16 toward black (32),
    -- blanking gate (33).  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_gav_sr(24) = '1' or s_gav_sr(26) = '1' then s27_y <= yr_q; else s27_y <= to_unsigned(64, 10); end if;
            s28_y <= s27_y; s29_y <= s28_y;
            s30_ya <= resize(s27_y, 12) + resize(s29_y, 12) + 2;
            s30_yb <= s28_y;
            v_y := s30_ya + shift_left(resize(s30_yb, 12), 1);
            s31_y <= v_y(11 downto 2);
            if s31_y > 64 then v_dl := s31_y - 64; else v_dl := (others => '0'); end if;
            s32_y <= s31_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if s_gav_sr(C_LAT - 2) = '1' then s33_y <= s32_y; else s33_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s33_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture scallop;
