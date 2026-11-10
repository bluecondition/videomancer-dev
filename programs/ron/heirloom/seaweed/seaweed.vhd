-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Seaweed v0.1.0 -- Heirloom (phase 3, video clips), from the still WIN_20230402_23_03_06 at 0.5 s.
--
-- The still: about nine wavy vertical curtain bands of green, mint and lavender, nearly straight at the left
-- edge and swinging harder to the right; dark brown cores with blue edges in the lower two thirds, violet lines
-- and magenta streaks between them, hard black pebble blobs along the top.  In the clip the swings travel down
-- the frame (one wave per 2.22 s) and the blobs morph on the right.
--
-- The patch, as fitted (seaweed.py has every constant):
--   VCO P (the bands) is reset by hsync (phase 0 at x0 = -155 virtual px) and FM'd per line by one vertical
--   LFO (562 lines, depth 9.6 %, travelling down):  F(y) = F0 (1 - D sin(2 pi y / 562 + theta(t))),
--   u = F (x - x0).  A sub-octave colouriser paints each band from fu = frac(u) and the band's parity (two
--   bands per palette): even bands carry a violet line with a blue edge, odd bands a brown core.
--   A half-rate VCO Q, q = (k + 0.33)(1/2 + es) + psi(y) (k = the band number, psi a vertical ramp, es the LFO
--   at 1/6 of P's depth), plus a quarter-rate term R4[k mod 4], give every band a key level z.  Odd bands grow
--   a brown core of width wo sqrt(r (2 - r)), r = clamp(z + zoff); even bands turn magenta and then cut a hard
--   black pebble blob of half-width 0.27 sqrt(r (2 - r)), r = clamp(2 (z + vr(y) - t2)), vr a vertical ramp
--   (high at the top).  1-2 px features (rims, the cyan tip) and the 6-8 px edge lines are dropped where the
--   band phase changes faster than 0.006 / 0.016 cycle per line, so no single-row line is ever drawn.
--
--   K1  Bands     VCO P frequency: the band count, x1/3 .. x3 (512 = the still's 9.4 across)
--   K2  Wave      LFO wavelength: 0.5 .. 7.7 swings per frame height (512 = the still's 1.9)
--   K3  Speed     bipolar: the swings travel down (right) or up (left); 768 = the clip's speed
--   K4  Blobs     the blob key's threshold: no blobs (0) .. the still (512) .. black pools everywhere (1023)
--   K5  Cores     the brown cores: thin threads (0) .. the still (512) .. fat dark cores in every odd band
--   K6  Bend      how far the camera (or the Osc, at four times the level) bends VCO P's phase
--   S7  Mirror    VCO P reset at the screen centre: a symmetric curtain, straight in the middle
--   S8  Pools     Ink (black blobs) / Glow (lit yellow pools with an orange rim)
--   S9  Flip      the vertical ramps inverted: blobs along the bottom, cores reaching up from the top
--   S10 Colours   Kelp (the still) / Coral (chroma rotated 160 degrees: coral pink, teal, amber)
--   S11 Source    Video / Osc
--   P12 SWAY      FM depth D = 0.02 + 0.38 p^2: straight curtain (0) .. the still (458) .. whipped braids
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, exp2 ROM), a per-line engine kicked at
-- each line start (computes the NEXT line: the interpolated LFO sine, the FM products, the increments, start
-- phases, the vertical phase step, Q's start and rate and the four R4 levels, on one serial multiplier), the
-- phase DDA (reversing at the centre for Mirror) with its vertical-step twin, a 32-stage pixel path.  Virtual
-- 1080-line coordinates (SD shows the centre 4:3 of the picture).  Generated avid; output conditioning (U and V
-- through two [1 2 2 2 2 2 2 2 1]/16 boxes, Y through one, with the running-sum resync, luma 13/16) before the
-- blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.seaweed_pkg.all;
use work.all;

architecture seaweed of program_top is
    constant C_LAT : integer := 32;   -- = processing_delay_clks

    -- fitted constants (phases: 2^24 per cycle; frequencies Q24 cycles per h = half virtual px)
    constant K_GP   : integer := 40725;      -- VCO P, 1 / 205.98 virtual px
    constant K_KL   : integer := 29853;      -- LFO, 2^24 / 562 per virtual line
    constant K_TH0  : integer := 15062962;   -- LFO phase at line 0 of the still (-0.642 rad)
    constant K_V    : integer := 524;        -- speed: d^2 * K_V >> 8 = Q24 per field (the clip at d = 248)
    constant K_DP0  : integer := 310;        -- reset point to x = 0, h
    constant K_Q0   : integer := 10813;      -- 0.33 / 2, Q16
    constant K_PS0  : integer := 12901679;   -- psi at line 0 (-0.231), Q24
    constant K_PS0F : integer := 11223943;   -- Flip: psi at line 0 = K_PS0 + 1080 * K_PS1 (mod 1)
    constant K_PS1  : integer := 13981;      -- psi step per virtual line (0.9 per frame height)
    constant K_PR0  : integer := 2801795;    -- psiR at line 0 (0.167)
    constant K_PR1  : integer := 15534;      -- psiR step per virtual line (1.0 per frame height)
    constant K_VR0  : integer := 6710886;    -- blob ramp at line 0 (0.4), Q24
    constant K_VR1  : integer := 12428;      -- its fall per virtual line (0.8 per frame height)
    constant K_RJ   : integer := 5578424;    -- R4 phase offset: 0.33 / 4 + 1/4 (cosine), Q24
    constant K_G1   : integer := 100663;     -- |vertical phase step| limits, Q24 per line (0.006, 0.016)
    constant K_G2   : integer := 268435;
    constant K_FL   : integer := 317;        -- core left edge, fu Q10
    constant K_FC   : integer := 338;        -- blob centre, fu Q10
    constant K_KB   : integer := 69;         -- blob half-width scale, Q8 cycles (0.27)

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
    -- x step in h per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    -- written at the source column, read at the beam column, which trails it by 4-5 pixels: never the
    -- address being written
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_sn_a : unsigned(9 downto 0) := (others => '0');
    signal r_sn_q : signed(11 downto 0) := (others => '0');
    signal r_zc_a : unsigned(9 downto 0) := (others => '0');
    signal r_zc_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_sq_a : unsigned(8 downto 0) := (others => '0');
    signal r_sq_q : unsigned(7 downto 0) := (others => '0');
    signal r_gd_a : unsigned(8 downto 0) := (others => '0');
    signal r_gd_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_ft_a : unsigned(7 downto 0) := (others => '0');
    signal r_ft_q : std_logic_vector(31 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 160 x 135 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(9 downto 0) := to_unsigned(408, 10);
    signal s_oys : unsigned(11 downto 0) := to_unsigned(484, 12);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_x0   : unsigned(8 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(150 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(458 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k3 : unsigned(9 downto 0) := to_unsigned(768, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_mir, s_glow, s_flip, s_cor, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_kl   : unsigned(17 downto 0) := (others => '0');
    signal z_p2   : unsigned(19 downto 0) := (others => '0');
    signal z_sq   : unsigned(17 downto 0) := (others => '0');
    signal z_b2   : unsigned(19 downto 0) := (others => '0');
    signal c_f0   : unsigned(17 downto 0) := to_unsigned(K_GP, 18);
    signal c_dl   : unsigned(20 downto 0) := (others => '0');          -- LFO step per line, Q24
    signal c_d    : unsigned(15 downto 0) := to_unsigned(6303, 16);    -- FM depth, Q16
    signal c_t    : unsigned(23 downto 0) := (others => '0');          -- LFO time, Q24
    signal c_bg   : unsigned(19 downto 0) := (others => '0');          -- bend, Q24 phase per luma step
    signal c_dps  : unsigned(17 downto 0) := (others => '0');          -- psi / psiR / blob-ramp steps per line
    signal c_dpr  : unsigned(17 downto 0) := (others => '0');
    signal c_dvr  : unsigned(17 downto 0) := (others => '0');
    signal c_wo   : unsigned(7 downto 0) := to_unsigned(97, 8);        -- core width scale, Q8 cycles
    signal c_zoff : unsigned(10 downto 0) := to_unsigned(717, 11);     -- core key offset, Q10
    signal c_t2   : signed(11 downto 0) := to_signed(635, 12);         -- blob threshold, Q10
    signal c_dist : unsigned(11 downto 0) := to_unsigned(K_DP0, 12);   -- reset point to the first pixel, h
    signal c_nc1  : unsigned(10 downto 0) := to_unsigned(959, 11);     -- centre pixel - 1 (Mirror)

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(23 downto 0) := (others => '0');
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

    -- line engine
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_th   : unsigned(23 downto 0) := (others => '0');          -- LFO phase of the line being computed
    signal e_ps, e_pr : unsigned(23 downto 0) := (others => '0');      -- psi, psiR
    signal e_vr   : signed(24 downto 0) := (others => '0');            -- blob ramp, Q24
    signal e_s0, e_s1 : signed(11 downto 0) := (others => '0');
    signal e_dn   : std_logic := '0';
    signal e_s    : signed(12 downto 0) := (others => '0');            -- interpolated sine, Q11
    signal e_sm   : unsigned(10 downto 0) := (others => '0');
    signal e_sn   : std_logic := '0';
    signal e_m    : signed(16 downto 0) := (others => '0');            -- m = D sin, Q16
    signal e_mg   : unsigned(15 downto 0) := (others => '0');
    signal e_es   : signed(16 downto 0) := (others => '0');            -- Q's rate offset, Q16
    signal e_qa   : signed(17 downto 0) := (others => '0');
    signal e_f, e_fo : unsigned(18 downto 0) := (others => '0');       -- F of the next line / of the last one
    signal e_df   : signed(19 downto 0) := (others => '0');
    signal e_dfs  : signed(19 downto 0) := (others => '0');            -- dF smoothed (1/8 per line)
    signal e_zk2, e_pre, e_first : std_logic := '0';
    signal e_dfm  : unsigned(18 downto 0) := (others => '0');
    signal e_vr10 : signed(10 downto 0) := (others => '0');
    signal e_rs   : signed(11 downto 0) := (others => '0');
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    signal n_inc  : unsigned(21 downto 0) := (others => '0');          -- next line: phase increment, Q24
    signal n_p0   : unsigned(30 downto 0) := (others => '0');          -- next line: start phase (7.24)
    signal n_ginc : unsigned(16 downto 0) := (others => '0');          -- next line: |vertical step| DDA
    signal n_g0   : unsigned(25 downto 0) := (others => '0');
    signal n_es5  : signed(8 downto 0) := (others => '0');            -- next line: Q rate - 1/2, Q16 >> 5
    signal n_q0   : unsigned(15 downto 0) := (others => '0');
    signal n_r40, n_r41, n_r42, n_r43 : signed(9 downto 0) := (others => '0');
    signal n_vb, n_vm : signed(12 downto 0) := (others => '0');

    -- beam
    signal b_p   : unsigned(30 downto 0) := (others => '0');
    signal b_g   : unsigned(25 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_dn  : std_logic := '0';
    signal l_inc : unsigned(21 downto 0) := (others => '0');
    signal l_ginc : unsigned(16 downto 0) := (others => '0');
    signal l_es5  : signed(8 downto 0) := (others => '0');
    signal l_q0   : unsigned(15 downto 0) := (others => '0');
    signal l_r40, l_r41, l_r42, l_r43 : signed(9 downto 0) := (others => '0');
    signal l_vb, l_vm : signed(12 downto 0) := (others => '0');

    -- pixel path
    type t_p17 is array (2 to 6) of unsigned(16 downto 0);
    signal pp     : t_p17 := (others => (others => '0'));
    signal p4_lm  : unsigned(7 downto 0) := (others => '0');
    signal p5_ea, p5_eb : unsigned(23 downto 0) := (others => '0');
    signal p6_b   : unsigned(13 downto 0) := (others => '0');
    signal p7_u   : unsigned(16 downto 0) := (others => '0');
    signal p8_qa  : signed(15 downto 0) := (others => '0');
    signal p8_k0  : std_logic := '0';
    signal p9_qs  : unsigned(15 downto 0) := (others => '0');
    signal p10_q  : unsigned(15 downto 0) := (others => '0');
    signal p11_f, p12_f, p13_f : unsigned(5 downto 0) := (others => '0');
    signal p12_r4, p13_r4, p14_r4 : signed(9 downto 0) := (others => '0');
    signal p13_zv : signed(10 downto 0) := (others => '0');
    signal p13_dz : signed(4 downto 0) := (others => '0');
    signal p14_zi : signed(11 downto 0) := (others => '0');
    signal p15_zt : signed(11 downto 0) := (others => '0');
    signal p16_rc, p16_rb, p16_mm : signed(12 downto 0) := (others => '0');
    signal p17_m3, p18_m3, p19_m3, p20_m3, p21_m3 : unsigned(2 downto 0) := (others => '0');
    signal p19_sq : unsigned(7 downto 0) := (others => '0');
    signal p20_w  : unsigned(9 downto 0) := (others => '0');
    signal p21_e1, p21_e2, p21_bd, p21_rd : signed(11 downto 0) := (others => '0');
    signal p21_hm : signed(11 downto 0) := (others => '0');
    signal p21_w  : unsigned(9 downto 0) := (others => '0');
    -- carried band data: fu (10 bits) and the steep flags ride the output ring's spare bits from stage 10 to
    -- stage 18; h = fu - K_FL and d = |fu - K_FC| are formed at 19 / 20; parity rides FFs
    signal c_fu8, c_fu9, c_fu10, c_fu19, c_fu20, c_fu21, c_fu22 : unsigned(9 downto 0) := (others => '0');
    signal c_x19  : signed(10 downto 0) := (others => '0');
    signal c_h19, c_h20, c_h21 : signed(10 downto 0) := (others => '0');
    signal c_dd20 : unsigned(9 downto 0) := (others => '0');
    signal c_par  : std_logic_vector(8 to 23) := (others => '0');
    signal c_st1, c_st2 : std_logic_vector(2 to 10) := (others => '0');
    signal c_s1_19, c_s2_19, c_s1_20, c_s2_20, c_s1_21, c_s2_21 : std_logic := '0';
    type t_k2 is array (8 to 11) of unsigned(1 downto 0);
    signal c_k2   : t_k2 := (others => (others => '0'));
    signal p22_idx : unsigned(6 downto 0) := (others => '0');
    signal p22_use : std_logic := '0';
    signal p23_use, p24_use, p25_use : std_logic := '0';
    signal p25_ft, p25_gd : std_logic_vector(31 downto 0) := (others => '0');
    signal p26_y, p26_u, p26_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y27, s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    -- the boxes' x(n - 8) taps: one 256 x 64 block-RAM ring {box A Y U V, box B U V, fu, steep flags}, written
    -- every clock at s_rw, read at s_rw - 6 (EBR + fabric register = the sample 8 clocks old); never the write
    -- address
    type t_ring is array (0 to 255) of std_logic_vector(63 downto 0);
    signal ring   : t_ring := (others => (others => '0'));
    attribute no_rw_check of ring : signal is true;
    signal s_rw   : unsigned(7 downto 0) := (others => '0');
    signal s_rr   : unsigned(7 downto 0) := to_unsigned(250, 8);
    signal s_rq   : std_logic_vector(63 downto 0) := (others => '0');
    signal s_rq2  : std_logic_vector(63 downto 0) := (others => '0');
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bbu, s_bbv, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 inputs of boxes A, B
    signal s_y29, s_u29, s_v29 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y30 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y31, s_u31, s_v31 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y32, s_u32, s_v32 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y33 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u33, s_v33 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

    -- black in the output filters: Y 64, U = V = 512 (x8 in the 8-sums)
    constant C_K8Y : unsigned(12 downto 0) := to_unsigned(512, 13);
    constant C_K8C : unsigned(12 downto 0) := to_unsigned(4096, 13);

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept) and ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_SW_EXP2(to_integer(r_ex_a));
            r_sn_q <= C_SW_SIN(to_integer(r_sn_a));
            r_zc_q <= C_SW_ZC(to_integer(r_zc_a));
            r_sq_q <= C_SW_SQRT(to_integer(r_sq_a));
            r_gd_q <= C_SW_GND(to_integer(r_gd_a));
            r_ft_q <= C_SW_FEAT(to_integer(r_ft_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);

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
            -- speed: |k3 - 512| - 8 (deadband +-8), sign
            s_k3 <= unsigned(registers_in(2));
            v_s := signed(resize(s_k3, 11)) - 512;
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
                s_mir  <= registers_in(6)(0);
                s_glow <= registers_in(6)(1);
                s_flip <= registers_in(6)(2);
                s_cor  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(816, 10);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(612, 10);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(408, 10);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 6), 12);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters (no zc decode on these enables)
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g4; s_gr <= unsigned(registers_in(3));
                            when 3      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 4      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
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
                            when 1      => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g4 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from
            -- load + 17 (stores at load + 18).  The slot number is decoded at 30 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 31 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 5);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 390 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_g1(12 downto 3), 24); z_fb <= to_unsigned(406, 16);      -- bands exponent
                    when 1  => z_fa <= to_unsigned(K_GP, 24);         z_fb <= z_mt;                       -- F0
                    when 2  => z_fa <= to_unsigned(K_KL, 24);         z_fb <= z_mt;                       -- LFO step
                    when 3  => z_fa <= resize(z_kl, 24);              z_fb <= resize(s_ys, 16);           -- per line
                    when 4  => z_fa <= resize(s_gp(12 downto 3), 24); z_fb <= resize(s_gp(12 downto 3), 16);  -- sway^2
                    when 5  => z_fa <= resize(z_p2, 24);              z_fb <= to_unsigned(24953, 16);     -- D
                    when 6  => z_fa <= resize(s_vm, 24);              z_fb <= resize(s_vm, 16);           -- speed^2
                    when 7  => z_fa <= resize(z_sq, 24);              z_fb <= to_unsigned(K_V, 16);       -- speed
                    when 8  => z_fa <= resize(s_g6(12 downto 3), 24); z_fb <= resize(s_g6(12 downto 3), 16);  -- bend^2
                    when 9  => z_fa <= resize(z_b2, 24);              z_fb <= to_unsigned(821, 16);       -- bend gain
                    when 10 => z_fa <= to_unsigned(K_PS1, 24);        z_fb <= resize(s_ys, 16);           -- psi step
                    when 11 => z_fa <= to_unsigned(K_PR1, 24);        z_fb <= resize(s_ys, 16);           -- psiR step
                    when 12 => z_fa <= to_unsigned(K_VR1, 24);        z_fb <= resize(s_ys, 16);           -- ramp step
                    when 13 => z_fa <= resize(s_g5(12 downto 3), 24); z_fb <= to_unsigned(164, 16);       -- core width
                    when 14 => z_fa <= resize(s_g5(12 downto 3), 24); z_fb <= to_unsigned(614, 16);       -- core offset
                    when others => z_fa <= resize(s_g4(12 downto 3), 24); z_fb <= to_unsigned(2457, 16);  -- blob threshold
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 36); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- geometry
                    when 20 =>
                        if s_mir = '1' then
                            c_dist <= to_unsigned(1920 + K_DP0, 12) - resize(s_x0, 12);
                        else
                            c_dist <= to_unsigned(K_DP0, 12) + resize(s_x0, 12);
                        end if;
                        case to_integer(s_xs) is
                            when 4      => c_nc1 <= to_unsigned(359, 11);
                            when 3      => c_nc1 <= to_unsigned(639, 11);
                            when others => c_nc1 <= to_unsigned(959, 11);
                        end case;

                    -- bands: e = (k1 * 406 >> 9) - 406, Q8 octaves (-406 .. 405)
                    when 50  => z_e <= signed(resize(f_acc(18 downto 9), 12)) - 406;
                    when 51  => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 53  => z_mt <= r_ex_q;
                    when 74  =>                       -- F0 = K_GP * mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when -2     => c_f0 <= resize(f_acc(35 downto 17), 18);
                            when -1     => c_f0 <= f_acc(33 downto 16);
                            when 0      => c_f0 <= f_acc(32 downto 15);
                            when others => c_f0 <= f_acc(31 downto 14);
                        end case;
                    when 75  =>                       -- wave: e = k2 - 512, Q8 octaves (-512 .. 511)
                        v_e := signed(resize(s_g2(12 downto 3), 12)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 77  => z_mt <= r_ex_q;
                    when 98  =>                       -- kL = K_KL * mt >> (15 - sh)
                        case to_integer(z_sh) is
                            when -2     => z_kl <= resize(f_acc(35 downto 17), 18);
                            when -1     => z_kl <= f_acc(33 downto 16);
                            when 0      => z_kl <= f_acc(32 downto 15);
                            when others => z_kl <= f_acc(31 downto 14);
                        end case;
                    when 122 => c_dl <= f_acc(28 downto 8);
                    when 146 => z_p2 <= f_acc(19 downto 0);
                    when 170 => c_d <= f_acc(35 downto 20) + 1311;
                    when 194 => z_sq <= f_acc(17 downto 0);
                    when 218 =>                       -- LFO time += speed (Q24 per field)
                        if s_vneg = '1' then c_t <= c_t - f_acc(31 downto 8); else c_t <= c_t + f_acc(31 downto 8); end if;
                    when 242 => z_b2 <= f_acc(19 downto 0);
                    when 266 =>                       -- bend = k6^2 * 821 >> 12; the Osc generator is patched in at
                                                      -- four times the camera's level
                        if s_src = '1' then c_bg <= f_acc(29 downto 10); else c_bg <= f_acc(31 downto 12); end if;
                    when 290 => c_dps <= f_acc(25 downto 8);
                    when 314 => c_dpr <= f_acc(25 downto 8);
                    when 338 => c_dvr <= f_acc(25 downto 8);
                    when 362 => c_wo <= f_acc(17 downto 10) + 15;
                    when 386 => c_zoff <= f_acc(19 downto 9) + 103;
                    when 410 => c_t2 <= resize(to_signed(1863, 13) - signed('0' & f_acc(21 downto 10)), 12);
                    when others => null;
                end case;
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
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
    -- Source side: oscillator (2-D triangle lattice, drifting), line-buffer
    -- write of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
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
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then v_l := s_osum; s_lb_wd <= std_logic_vector(v_l);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (16 iterations, result
    -- readable at load + 17):
    --   s   = sin(theta) interpolated between 1024-entry ROM samples      (Q11)
    --   m   = D |s| >> 11 (signed by s)                                    (Q16)
    --   F   = F0 -+ F0 |m| >> 16;  inc = F xs;  start = F dist           (Q24 per h)
    --   dF  = F - F(last line);  the vertical-step DDA = dF dist + n dF xs
    --   es  = m / 8 + m / 32;  r = 1/2 + es;  q0 = 0.33 / 2 + psi + 0.33 es
    --   R4[j] = 0.35 cos(2 pi ((j + 0.33) / 4 + psiR))                     (Q10)
    --   vb  = vr - t2,  vm = vb + 1   (blob and magenta keys)              (Q10)
    -- ~115 clocks, so it fits SD's 858-clock line.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d   : signed(12 downto 0);
        variable v_t   : unsigned(16 downto 0);
        variable v_f   : unsigned(18 downto 0);
        variable v_i   : unsigned(21 downto 0);
        variable v_gi  : unsigned(21 downto 0);
        variable v_ph  : unsigned(23 downto 0);
        variable v_r   : signed(11 downto 0);
        variable v_r4  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- two kicks in the zone: a pre-run for line -1 (its F seeds line 0's vertical step), then line 0
            e_zk <= '0'; e_zk2 <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;
            if s_zone = '1' and s_zc = 760 then e_zk2 <= '1'; end if;

            if e_zk = '1' then
                e_lc <= (others => '0'); e_pre <= '1';
            elsif e_zk2 = '1' then
                e_lc <= (others => '0'); e_pre <= '0'; e_first <= '1';
            elsif g_start = '1' then
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            -- R4 sine reads: addresses at lc 96 .. 99, EBR data 97 .. 100, fabric copy 98 .. 101, latched 99 .. 102
            case to_integer(e_lc(1 downto 0)) is
                when 0      => v_ph := e_pr + to_unsigned(K_RJ, 24);
                when 1      => v_ph := e_pr + to_unsigned(K_RJ + 4194304, 24);
                when 2      => v_ph := e_pr + to_unsigned(K_RJ + 8388608, 24);
                when others => v_ph := e_pr + to_unsigned(K_RJ + 12582912 - 16777216, 24);
            end case;
            e_rs <= r_sn_q;                                  -- fabric copy of the sine EBR
            v_r := e_rs;
            v_r4 := shift_right(v_r, 2) + shift_right(v_r, 4) + shift_right(v_r, 5) + shift_right(v_r, 7);

            case to_integer(e_lc) is
                when 2   => r_sn_a <= e_th(23 downto 14);
                when 3   => r_sn_a <= e_th(23 downto 14) + 1;
                when 4   => e_s0 <= r_sn_q;
                when 5   => e_s1 <= r_sn_q;
                when 6   =>                                  -- (s1 - s0) * frac, sign-magnitude
                    v_d := resize(e_s1, 13) - resize(e_s0, 13);
                    if v_d < 0 then m_a <= resize(unsigned(-v_d), 36); e_dn <= '1';
                    else m_a <= resize(unsigned(v_d), 36); e_dn <= '0'; end if;
                    m_b <= "00" & e_th(13 downto 0); m_acc <= (others => '0');
                when 23  =>
                    if e_dn = '1' then e_s <= resize(e_s0, 13) - signed(resize(m_acc(18 downto 14), 13));
                    else e_s <= resize(e_s0, 13) + signed(resize(m_acc(18 downto 14), 13)); end if;
                when 24  =>                                  -- m = D |s| >> 11
                    if e_s < 0 then e_sm <= resize(unsigned(-e_s), 11); e_sn <= '1';
                    else e_sm <= resize(unsigned(e_s), 11); e_sn <= '0'; end if;
                when 25  =>
                    m_a <= resize(c_d, 36); m_b <= resize(e_sm, 16); m_acc <= (others => '0');
                when 42  =>
                    e_mg <= m_acc(26 downto 11);
                    if e_sn = '1' then e_m <= -signed(resize(m_acc(26 downto 11), 17));
                    else e_m <= signed(resize(m_acc(26 downto 11), 17)); end if;
                when 43  =>                                  -- F0 |m| >> 16
                    m_a <= resize(c_f0, 36); m_b <= e_mg; m_acc <= (others => '0');
                    e_es <= shift_right(e_m, 3) + shift_right(e_m, 5);
                when 44  =>
                    n_es5 <= resize(shift_right(e_es, 5), 9);
                    e_qa <= resize(shift_right(e_es, 2), 18) + resize(shift_right(e_es, 4), 18) + resize(shift_right(e_es, 6), 18);
                when 45  =>
                    n_q0 <= to_unsigned(K_Q0, 16) + e_ps(23 downto 8) + unsigned(e_qa(15 downto 0));
                when 60  =>                                  -- F = F0 -+ t (F = F0 (1 - D sin))
                    v_t := m_acc(32 downto 16);
                    if e_sn = '1' then v_f := resize(c_f0, 19) + resize(v_t, 19);
                    else v_f := resize(c_f0, 19) - resize(v_t, 19); end if;
                    e_f <= v_f;
                    m_a <= resize(v_f, 36); m_b <= resize(c_dist, 16); m_acc <= (others => '0');
                when 61  =>                                  -- inc = F xs; dF
                    case to_integer(s_xs) is
                        when 4      => v_i := shift_left(resize(e_f, 22), 2);
                        when 3      => v_i := shift_left(resize(e_f, 22), 1) + resize(e_f, 22);
                        when others => v_i := shift_left(resize(e_f, 22), 1);
                    end case;
                    n_inc <= v_i;
                    e_df <= signed(resize(e_f, 20)) - signed(resize(e_fo, 20));
                when 62  =>
                    e_fo <= e_f;
                    if e_first = '1' then e_dfs <= e_df; else e_dfs <= e_dfs + shift_right(e_df - e_dfs, 3); end if;
                when 63  =>
                    if e_dfs < 0 then e_dfm <= resize(unsigned(-e_dfs), 19); else e_dfm <= resize(unsigned(e_dfs), 19); end if;
                when 64  =>
                    case to_integer(s_xs) is
                        when 4      => v_gi := shift_left(resize(e_dfm, 22), 2);
                        when 3      => v_gi := shift_left(resize(e_dfm, 22), 1) + resize(e_dfm, 22);
                        when others => v_gi := shift_left(resize(e_dfm, 22), 1);
                    end case;
                    n_ginc <= v_gi(16 downto 0);
                when 77  =>                                  -- start = F dist; vertical step g0 = dF dist
                    n_p0 <= m_acc(30 downto 0);
                    m_a <= resize(e_dfm, 36); m_b <= resize(c_dist, 16); m_acc <= (others => '0');
                when 94  =>
                    n_g0 <= m_acc(25 downto 0);
                    e_vr10 <= e_vr(24 downto 14);
                when 95  =>
                    n_vb <= resize(e_vr10, 13) - resize(c_t2, 13);
                when 96  =>
                    n_vm <= n_vb + 1024;
                when 99  => n_r40 <= v_r4(10 downto 1);      -- R4[j] = (0.3516 s) >> 1, Q10
                when 100 => n_r41 <= v_r4(10 downto 1);
                when 101 => n_r42 <= v_r4(10 downto 1);
                when 102 => n_r43 <= v_r4(10 downto 1);
                when others => null;
            end case;
            if e_lc >= 96 and e_lc <= 99 then r_sn_a <= v_ph(23 downto 14); end if;

            -- per-line DDAs: line-0 values at the zone kick, one step per line after the line's values
            if e_zk = '1' then
                e_th <= to_unsigned(K_TH0, 24) - c_t - resize(c_dl, 24);
                e_pr <= to_unsigned(K_PR0, 24);
                e_fo <= resize(c_f0, 19);
                if s_flip = '1' then
                    e_ps <= to_unsigned(K_PS0F, 24); e_vr <= to_signed(-K_VR0, 25);
                else
                    e_ps <= to_unsigned(K_PS0, 24);  e_vr <= to_signed(K_VR0, 25);
                end if;
            elsif e_lc = 110 then
                e_th <= e_th + resize(c_dl, 24);
                if e_pre = '0' then e_first <= '0'; end if;
                if e_pre = '1' then
                    null;                                    -- the pre-run only advances the LFO phase
                elsif s_flip = '1' then
                    e_ps <= e_ps - resize(c_dps, 24); e_pr <= e_pr - resize(c_dpr, 24); e_vr <= e_vr + signed(resize(c_dvr, 25));
                else
                    e_ps <= e_ps + resize(c_dps, 24); e_pr <= e_pr + resize(c_dpr, 24); e_vr <= e_vr - signed(resize(c_dvr, 25));
                end if;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the phase DDA and its vertical-step twin.  Mirror: both run down
    -- from the left edge to the centre pixel and up after it (the reset point
    -- sits at the centre).
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_ip : unsigned(30 downto 0);
        variable v_s32 : unsigned(31 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_p  <= n_p0;
                b_g  <= n_g0;
                b_px <= (others => '0');
                b_dn <= s_mir;
                l_inc <= n_inc; l_ginc <= n_ginc;
                l_es5 <= n_es5; l_q0 <= n_q0;
                l_r40 <= n_r40; l_r41 <= n_r41; l_r42 <= n_r42; l_r43 <= n_r43;
                l_vb <= n_vb; l_vm <= n_vm;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                if b_dn = '1' then v_ip := not resize(l_inc, 31); else v_ip := resize(l_inc, 31); end if;
                v_s32 := (b_p & '1') + (v_ip & b_dn);
                b_p <= v_s32(31 downto 1);
                if b_dn = '1' then b_g <= b_g - resize(l_ginc, 26); else b_g <= b_g + resize(l_ginc, 26); end if;
                if b_px = c_nc1 then b_dn <= '0'; end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage n = valid at the n-th clock after g_start; stage 1 = beam)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_v   : unsigned(15 downto 0);
        variable v_c   : signed(12 downto 0);
        variable v_ic, v_ib : unsigned(8 downto 0);
        variable v_b28 : unsigned(27 downto 0);
        variable v_zp  : signed(11 downto 0);
        variable v_ws  : unsigned(15 downto 0);
        variable v_idx : unsigned(6 downto 0);
        variable v_use : std_logic;
        variable v_h   : signed(10 downto 0);
        variable v_w   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2..6: carry the phase (bits 30..14) while the bend is formed
            pp(2) <= b_p(30 downto 14);
            for i in 3 to 6 loop pp(i) <= pp(i - 1); end loop;
            -- 2: |vertical step| against the two limits
            if b_g > K_G1 then c_st1(2) <= '1'; else c_st1(2) <= '0'; end if;
            if b_g > K_G2 then c_st2(2) <= '1'; else c_st2(2) <= '0'; end if;
            for i in 3 to 10 loop c_st1(i) <= c_st1(i - 1); c_st2(i) <= c_st2(i - 1); end loop;
            -- 4: luma from the line buffer (EBR data at 3, fabric copy)
            p4_lm <= unsigned(s_lb_q);
            -- 5: bend partials (luma nibbles x gain)
            p5_ea <= resize(p4_lm(3 downto 0) * c_bg, 24);
            p5_eb <= resize(p4_lm(7 downto 4) * c_bg, 24);
            -- 6: bend = luma * gain (bits 27..14 kept; the phase's low bits are dropped)
            v_b28 := resize(p5_ea, 28) + shift_left(resize(p5_eb, 28), 4);
            p6_b  <= v_b28(27 downto 14);
            -- 7: bent phase (bits 30..14 of 7.24)
            p7_u  <= pp(6) + resize(p6_b, 17);
            -- 8: band number k = u(16:10), fu = u(9:0), parity; Q partials k * r
            c_fu8 <= p7_u(9 downto 0);
            c_par(8) <= p7_u(10);
            c_k2(8) <= p7_u(11 downto 10);
            p8_qa <= resize(signed('0' & p7_u(16 downto 10)) * l_es5, 16);   -- k (es >> 5)
            p8_k0 <= p7_u(10);
            for i in 9 to 23 loop c_par(i) <= c_par(i - 1); end loop;
            for i in 9 to 11 loop c_k2(i) <= c_k2(i - 1); end loop;
            c_fu9 <= c_fu8; c_fu10 <= c_fu9;
            -- 9: Q partial sum
            p9_qs <= unsigned(shift_left(p8_qa, 5)) + (p8_k0 & "000000000000000");   -- k r = k / 2 + k es (mod 1)
            -- 10: q = q0 + k r (mod 1)
            p10_q <= p9_qs + l_q0;
            -- 11: the key ROM address: (q + 1/4) in 1024 steps, 6 bits below for the interpolation
            v_v := p10_q + 16384;
            r_zc_a <= v_v(15 downto 6);
            p11_f  <= v_v(5 downto 0);
            -- 12: ROM data; R4 by k mod 4
            case to_integer(c_k2(11)) is
                when 0      => p12_r4 <= l_r40;
                when 1      => p12_r4 <= l_r41;
                when 2      => p12_r4 <= l_r42;
                when others => p12_r4 <= l_r43;
            end case;
            p12_f <= p11_f;
            -- 13: fabric copy of {z, dz}
            p13_zv <= signed(r_zc_q(15 downto 5));
            p13_dz <= signed(r_zc_q(4 downto 0));
            p13_f  <= p12_f; p13_r4 <= p12_r4;
            -- 14: z = z[i] + dz f / 64 (Q10)
            v_zp := p13_dz * signed('0' & p13_f);
            p14_zi <= resize(p13_zv, 12) + resize(shift_right(v_zp, 6), 12);
            p14_r4 <= p13_r4;
            -- 15: z + R4
            p15_zt <= p14_zi + resize(p14_r4, 12);
            -- 16: key offsets: core (zoff), blob (vr - t2), magenta (vr - t1)
            p16_rc <= resize(p15_zt, 13) + signed(resize(c_zoff, 13));
            p16_rb <= resize(p15_zt, 13) + l_vb;
            p16_mm <= resize(p15_zt, 13) + l_vm;
            -- 17: clamps, the round-end ROM address by parity
            v_c := p16_rc;
            if v_c < 0 then v_ic := (others => '0'); elsif v_c > 511 then v_ic := (others => '1'); else v_ic := unsigned(v_c(8 downto 0)); end if;
            v_c := shift_left(p16_rb, 1);
            if p16_rb < 0 then v_ib := (others => '0'); elsif p16_rb > 255 then v_ib := (others => '1'); else v_ib := unsigned(v_c(8 downto 0)); end if;
            if c_par(16) = '1' then r_sq_a <= v_ic; else r_sq_a <= v_ib; end if;
            v_c := shift_right(p16_mm, 7);
            if v_c < 0 then p17_m3 <= (others => '0'); elsif v_c > 7 then p17_m3 <= (others => '1'); else p17_m3 <= unsigned(v_c(2 downto 0)); end if;
            p18_m3 <= p17_m3; p19_m3 <= p18_m3; p20_m3 <= p19_m3; p21_m3 <= p20_m3;
            -- 18: SQRT EBR; 19: fabric copy
            p19_sq <= r_sq_q;
            -- 20: width = scale * sqrt >> 6 (Q10)
            if c_par(19) = '1' then v_ws := c_wo * p19_sq; else v_ws := to_unsigned(K_KB, 8) * p19_sq; end if;
            p20_w <= v_ws(15 downto 6);
            -- 18: fu and the steep flags back from the ring (s_rq2's spare bits); 19: h, fu - K_FC; 20: d
            c_fu19 <= unsigned(s_rq2(13 downto 4)); c_s1_19 <= s_rq2(3); c_s2_19 <= s_rq2(2);
            c_h19 <= signed('0' & unsigned(s_rq2(13 downto 4))) - K_FL;
            c_x19 <= signed('0' & unsigned(s_rq2(13 downto 4))) - K_FC;
            c_fu20 <= c_fu19; c_h20 <= c_h19; c_s1_20 <= c_s1_19; c_s2_20 <= c_s2_19;
            if c_x19 < 0 then c_dd20 <= unsigned(-c_x19(9 downto 0)); else c_dd20 <= unsigned(c_x19(9 downto 0)); end if;
            c_fu21 <= c_fu20; c_h21 <= c_h20; c_s1_21 <= c_s1_20; c_s2_21 <= c_s2_20;
            c_fu22 <= c_fu21;
            -- 21: distances to the core edges and the blob edge
            v_h := c_h20; v_w := signed(resize(p20_w, 12));
            p21_e1 <= v_w - resize(v_h, 12);                         -- w - h (inside the core if > 0)
            p21_e2 <= resize(v_h, 12) - v_w;                         -- h - w (past its right edge)
            p21_bd <= signed(resize(c_dd20, 12)) - v_w;              -- blob if < 0
            p21_rd <= signed(resize(c_dd20, 12)) - v_w - 12;         -- rim if < 0
            p21_hm <= resize(v_h, 12) - 154;
            p21_w  <= p20_w;
            -- 22: feature index and select
            v_h := c_h21;
            v_use := '0'; v_idx := (others => '0');
            if c_par(21) = '1' then
                if v_h >= 0 and p21_e1 > 0 then
                    v_use := '1';
                    if p21_w < 51 and c_s1_21 = '0' then v_idx := to_unsigned(112, 7);
                    elsif v_h < 32 then v_idx := to_unsigned(96, 7) + unsigned(v_h(4 downto 2));
                    elsif p21_e1 > 511 then v_idx := to_unsigned(63, 7);
                    else v_idx := '0' & unsigned(p21_e1(8 downto 3)); end if;
                elsif p21_w >= 20 and p21_e2 >= 0 and p21_e2 < 92 and c_s2_21 = '0' then
                    v_use := '1'; v_idx := to_unsigned(64, 7) + unsigned(p21_e2(6 downto 2));
                end if;
            else
                if p21_bd < 0 then
                    v_use := '1'; if s_glow = '1' then v_idx := to_unsigned(115, 7); else v_idx := to_unsigned(113, 7); end if;
                elsif p21_w /= 0 and p21_rd < 0 and c_s1_21 = '0' then
                    v_use := '1'; if s_glow = '1' then v_idx := to_unsigned(116, 7); else v_idx := to_unsigned(114, 7); end if;
                elsif v_h >= 20 and v_h < 154 and p21_m3 /= 0 then
                    v_use := '1'; v_idx := to_unsigned(104, 7) + resize(p21_m3, 7);
                elsif v_h >= 154 and v_h < 226 and c_s2_21 = '0' then
                    v_use := '1'; v_idx := to_unsigned(117, 7) + unsigned(p21_hm(6 downto 3));
                end if;
            end if;
            p22_idx <= v_idx; p22_use <= v_use;
            -- 23: ROM addresses
            r_ft_a <= s_cor & p22_idx;
            r_gd_a <= s_cor & c_par(22) & c_fu22(9 downto 3);
            p23_use <= p22_use;
            -- 24: EBR data; 25: fabric copies
            p24_use <= p23_use;
            p25_ft <= r_ft_q; p25_gd <= r_gd_q; p25_use <= p24_use;
            -- 26: feature or ground
            if p25_use = '1' then
                p26_y <= unsigned(p25_ft(31 downto 22)); p26_u <= unsigned(p25_ft(21 downto 12)); p26_v <= unsigned(p25_ft(11 downto 2));
            else
                p26_y <= unsigned(p25_gd(31 downto 22)); p26_u <= unsigned(p25_gd(21 downto 12)); p26_v <= unsigned(p25_gd(11 downto 2));
            end if;
            -- 27: black outside the generated avid
            if s_gav_sr(24) = '0' then
                s_y27 <= to_unsigned(64, 10); s_u27 <= to_unsigned(512, 10); s_v27 <= to_unsigned(512, 10);
            else
                s_y27 <= p26_y; s_u27 <= p26_u; s_v27 <= p26_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(24);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes, Y
    -- through one (each box two adjacent 8-sums: the still's soft chroma and
    -- ~6 px luma edges, and the HD HDMI bar fix), luma 13/16 and the blanking
    -- gate.
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            ring(to_integer(s_rw)) <= std_logic_vector(s_y27) & std_logic_vector(s_u27) & std_logic_vector(s_v27)
                                      & std_logic_vector(s_u29) & std_logic_vector(s_v29)
                                      & std_logic_vector(c_fu10) & c_st1(10) & c_st2(10) & "00";   -- fu, steep flags: 10 -> 18
            s_rq <= ring(to_integer(s_rr));
        end if;
    end process p_ring;

    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_ay, v_au, v_av, v_bu, v_bv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_rw <= s_rw + 1; s_rr <= s_rr + 1;
            s_rq2 <= s_rq;
            v_ay := unsigned(s_rq2(63 downto 54)); v_au := unsigned(s_rq2(53 downto 44)); v_av := unsigned(s_rq2(43 downto 34));
            v_bu := unsigned(s_rq2(33 downto 24)); v_bv := unsigned(s_rq2(23 downto 14));
            -- The running 8-sums snap back to black (the off-avid input of both boxes) once 8 blank samples
            -- are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 28 sums, stage 29 output
            if s_bah = "00000000" then
                s_bay <= C_K8Y; s_bau <= C_K8C; s_bav <= C_K8C;
            else
                s_bay <= s_bay + s_y27 - v_ay;
                s_bau <= s_bau + s_u27 - v_au;
                s_bav <= s_bav + s_v27 - v_av;
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(26) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y29 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u29 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v29 <= v_sc(13 downto 4);
            else
                s_y29 <= to_unsigned(64, 10); s_u29 <= to_unsigned(512, 10); s_v29 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(26);
            -- box B (chroma only; luma takes box A alone): stage 30 sums, stage 31 output.  Luma: two plain stages.
            if s_bbh = "00000000" then
                s_bbu <= C_K8C; s_bbv <= C_K8C;
            else
                s_bbu <= s_bbu + s_u29 - v_bu;
                s_bbv <= s_bbv + s_v29 - v_bv;
            end if;
            s_bbud <= s_bbu; s_bbvd <= s_bbv;
            s_y30 <= s_y29; s_y31 <= s_y30;
            v_sc := resize(s_bbu, 14) + s_bbud; s_u31 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v31 <= v_sc(13 downto 4);
            -- 32: luma 13/16
            if s_y31 > 64 then v_dl := s_y31 - 64; else v_dl := (others => '0'); end if;
            s_y32 <= s_y31 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u32 <= s_u31; s_v32 <= s_v31;
            -- 33: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y33 <= s_y32; s_u33 <= s_u32; s_v33 <= s_v32;
            else
                s_y33 <= to_unsigned(64, 10); s_u33 <= to_unsigned(512, 10); s_v33 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;
    data_out.y       <= std_logic_vector(s_y33);
    data_out.u       <= std_logic_vector(s_u33);
    data_out.v       <= std_logic_vector(s_v33);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture seaweed;
