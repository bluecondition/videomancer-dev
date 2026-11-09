-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Velvet v0.1.1 -- Heirloom (video phase), from the clip WIN_20240216_23_50_17 at 0.5 s.
--
-- The still: soft vertical drapes in magenta, red-orange, yellow, pink-white and violet, with dark-green shadow
-- lines; the strands hang straight at the top and fan apart lower down, opening into two nested red chevrons
-- (x 518 and 1462, apexes on row 655) with a dark-green tongue left of each.  A "reflection" band sits in the
-- lower right: the drapes torn sideways into horizontal smears and oval blobs, dark-green shelves and green
-- glints.  In the clip the drapes never move; the band rolls UP the frame (340 lines/s, one band every 6 s).
--
-- The patch, as fitted (velvet.py has every table):
--   drapes   X = x + Sway (S1(x) F1(y) + S2(x) F2(y)),  F1 = sgn(u) u^2 (u = y/1080 - Hem),  F2 = y/1080
--            colour = PROF(Pleats(X)): the clip's top-row colours (a phase-modulated wavetable)
--   chevrons phi = 0.295 (y - ya) - T, T = distance to the axes 518 + 944 k: colour = CHEV(phi) where phi > -16
--   tongues  a dark-green lens 95 px left of each axis over 540 rows from ya - 200
--   band     a rolling vertical gate (2048-line period, 1070 on; the right side 0.153 line / px ahead) x an x ramp,
--            carrying a diagonal triangle wave (280 lines x 900 px), 40-line strips with a sample-and-hold offset
--            and an H-triangle phase (slewed line to line), dark-green shelves at strip changes, green glints
--
--   K1  Sway     the strands' sway: bent the other way (0) .. straight (256) .. the clip (512) .. 3x (1023)
--   K2  Pleats   the strands' width: x3 wide .. x1/3 fine (512 = the clip), in 1/8-octave steps
--   K3  Tide     bipolar: the band rolls up (+) or down (-); 700 = the clip's 340 lines/s
--   K4  Swell    the band's strip / wave scale: long swells .. the clip (512) .. fine chatter
--   K5  Hem      where the drapes start to fan and the chevrons open: high (0) .. the clip (340) .. low (1023)
--   K6  Etch     how far the camera (or the Osc) pushes the strands sideways
--   S7  Mirror   the curtain mirrored about the centre
--   S8  Hang     Top (the clip) / Floor: the curtain hangs from the floor (fan and chevrons at the top)
--   S9  Reflect  Band (the clip: a rolling band) / Flood: the reflection ripples the whole frame
--   S10 Light    Day / Night: the sibling clip's night look, near-black ground, glowing red / pink / green strands
--   S11 Source   Video / Osc
--   P12 RIPPLE   the band's depth: a faint shimmer (0) .. the clip (558) .. a torrent of smears (100 %)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line
-- start (computes the NEXT line: the sway terms, the strip slew, the strip offset, the chevron / tongue line
-- terms), beam DDAs (mirror-aware), a 25-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre
-- 4:3).  Generated avid; output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4, luma 13/16) before
-- the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.velvet_pkg.all;
use work.all;

architecture velvet of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

    constant K_GB0  : integer := 342016;       -- band coordinate of line 0 at power-up (Q8 lines: -712 lines, the still)

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
    -- per-line steps of the vertical terms (same timing-ID map as f_vstep_y), each = rate x vstep / 256:
    -- v (Q16), chevron phi (Q8 px, 0.295 px / line), tongue offset (Q8 px, 0.06 px / line), tongue p (Q16, 1/540)
    function f_step(t : std_logic_vector(3 downto 0); k : integer; neg : std_logic) return signed is
        type t_tab is array (0 to 6) of integer;
        constant C_DV   : t_tab := (270, 137, 228, 114, 91, 121, 61);     -- v (Q16)
        constant C_DPHI : t_tab := (336, 170, 283, 142, 113, 151, 76);    -- chevron phi (Q8 px)
        constant C_DTO  : t_tab := (68, 35, 58, 29, 23, 31, 15);          -- tongue offset (Q8 px)
        constant C_DTP  : t_tab := (540, 273, 455, 228, 182, 243, 121);   -- tongue p (Q16)
        variable i, v : integer;
    begin
        case to_integer(unsigned(t)) is
            when 0 => i := 0; when 4 => i := 1; when 8 => i := 2; when 12 => i := 3;
            when 5 | 6 | 14 => i := 4; when 1 | 2 | 10 => i := 5; when others => i := 6;
        end case;
        case k is
            when 0 => v := C_DV(i); when 1 => v := C_DPHI(i); when 2 => v := C_DTO(i); when others => v := C_DTP(i);
        end case;
        if neg = '1' then v := -v; end if;
        return to_signed(v, 11);
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
    -- triangle of a phase: +63 at 0.5 cycle, -64 at 0 (from the top 7 bits)
    function f_tri(ph : unsigned(15 downto 0)) return signed is
        variable f : unsigned(6 downto 0);
        variable d : unsigned(6 downto 0);
        variable t : signed(7 downto 0);
    begin
        f := ph(15 downto 9);
        if f >= 64 then d := f - 64; else d := 64 - f; end if;
        t := to_signed(63, 8) - signed(resize(d & '0', 8));
        if t < -64 then t := to_signed(-64, 8); end if;
        return t(6 downto 0);
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc lattice)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_rn_a : unsigned(7 downto 0) := (others => '0');
    signal r_rn_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_col_a : unsigned(8 downto 0) := (others => '0');
    signal r_col_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_pf_a : unsigned(9 downto 0) := (others => '0');
    signal r_pf_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_ch_a : unsigned(7 downto 0) := (others => '0');
    signal r_ch_q : std_logic_vector(31 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
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

    -- knobs: glided (Q3), raw tide, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(340 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(240 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(507 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k3 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_mir, s_flp, s_fld, s_ngt, s_src : std_logic := '0';

    -- per-field constants
    signal z_e2, z_e4 : signed(4 downto 0) := (others => '0');       -- pleats / swell exponents, 1/8 octave
    signal z_kb   : unsigned(16 downto 0) := to_unsigned(26216, 17);
    signal z_hneg : std_logic := '0';
    signal z_hm   : unsigned(9 downto 0) := (others => '0');          -- |k5 - 340|
    signal z_d    : unsigned(9 downto 0) := (others => '0');          -- P12 above the still (0 .. 512)
    signal z_sfdb : unsigned(6 downto 0) := to_unsigned(51, 7);       -- fade step / start at the still
    signal z_fd0  : unsigned(15 downto 0) := to_unsigned(56320, 16);
    signal z_gbx0 : unsigned(18 downto 0) := to_unsigned(449869, 19);
    signal c_on   : unsigned(10 downto 0) := to_unsigned(1070, 11);   -- band on-length (lines)
    signal c_neg  : unsigned(10 downto 0) := to_unsigned(1536, 11);   -- fade-shifted coordinate below 0 (wrapped)
    signal c_gs   : signed(10 downto 0) := to_signed(256, 11);        -- sway gain, Q8 (-1 .. 3)
    signal c_pm   : unsigned(2 downto 0) := (others => '0');          -- pleats = (8 + pm) / 8 * 2^ps
    signal c_ps   : signed(1 downto 0) := (others => '0');
    signal c_ks   : unsigned(17 downto 0) := to_unsigned(26216, 18);  -- strips per virtual line, Q20
    signal c_dsp  : unsigned(15 downto 0) := (others => '0');         -- ... per line, Q16
    signal c_p2   : unsigned(19 downto 0) := (others => '0');         -- P12^2
    signal c_dep  : unsigned(11 downto 0) := to_unsigned(1024, 12);   -- band depth, Q10
    signal c_a1   : unsigned(6 downto 0) := to_unsigned(36, 7);       -- band amplitude, 4 px units
    signal c_a1q  : unsigned(6 downto 0) := to_unsigned(27, 7);       -- ... x 3/4
    signal p5_amp : unsigned(6 downto 0) := (others => '0');
    signal c_m2   : unsigned(17 downto 0) := (others => '0');         -- tide^2
    signal c_dr   : unsigned(9 downto 0) := (others => '0');          -- band roll per field, Q4 lines
    signal c_drs  : unsigned(23 downto 0) := (others => '0');         -- ... in strips, Q16
    signal c_e2   : unsigned(19 downto 0) := (others => '0');         -- etch^2
    signal c_g6   : unsigned(5 downto 0) := to_unsigned(3, 6);        -- etch gain
    signal c_hem  : signed(13 downto 0) := (others => '0');           -- Hem, Q12 of the frame height
    signal c_ya   : unsigned(11 downto 0) := to_unsigned(655, 12);    -- chevron apex row
    signal c_phi0 : signed(19 downto 0) := (others => '0');           -- chevron phi of line 0, Q8 px
    signal c_gb0  : unsigned(18 downto 0) := to_unsigned(K_GB0, 19);  -- band coordinate of line 0 (rolls)
    signal c_sp0  : unsigned(23 downto 0) := to_unsigned(15610675, 24); -- strip phase of line 0 (rolls; -712 lines)
    signal c_tw0  : unsigned(15 downto 0) := (others => '0');         -- beam start
    signal c_gbx0, c_grx0 : unsigned(18 downto 0) := (others => '0');
    signal c_sfd  : unsigned(6 downto 0) := (others => '0');
    signal c_sd0  : signed(12 downto 0) := (others => '0');
    signal c_stw  : signed(9 downto 0) := (others => '0');             -- beam steps
    signal c_sgb : signed(8 downto 0) := (others => '0');
    signal c_ssd : signed(7 downto 0) := (others => '0');
    signal c_xb0  : unsigned(11 downto 0) := (others => '0');
    signal c_loff : unsigned(6 downto 0) := (others => '0');          -- Osc: lattice mean
    signal c_dv   : signed(10 downto 0) := to_signed(61, 11);         -- per-line steps (negative for Hang = Floor)
    signal c_dphi : signed(10 downto 0) := to_signed(76, 11);
    signal c_ty, c_tu, c_tv : unsigned(9 downto 0) := to_unsigned(512, 10);   -- off-avid box input (dark green)

    -- field serial multiplier
    signal f_a   : unsigned(27 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(27 downto 0) := (others => '0');
    signal z_fa  : unsigned(27 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
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
    signal e_lc   : unsigned(6 downto 0) := to_unsigned(127, 7);
    signal e_zk   : std_logic := '0';
    signal e_v    : signed(17 downto 0) := (others => '0');            -- v, Q16
    signal e_sp   : unsigned(23 downto 0) := (others => '0');          -- strip phase, Q16
    signal e_gb   : unsigned(18 downto 0) := (others => '0');          -- band coordinate, Q8 lines
    signal e_phi  : signed(19 downto 0) := (others => '0');            -- chevron phi, Q8 px
    signal e_ph   : unsigned(8 downto 0) := (others => '0');           -- slewed strip phase, Q9
    signal e_of   : signed(8 downto 0) := (others => '0');             -- slewed strip offset, Q8
    signal e_un, e_qn : std_logic := '0';
    signal e_tok  : std_logic := '0';
    signal e_tsg  : std_logic := '0';
    signal l_fa   : unsigned(15 downto 0) := (others => '0');          -- line-engine operands for the shared multiplier
    signal l_fb   : unsigned(12 downto 0) := (others => '0');
    signal l_ld   : std_logic := '0';
    signal n_f1   : signed(11 downto 0) := (others => '0');           -- sway terms x Sway, Q8
    signal n_f2   : signed(10 downto 0) := (others => '0');
    signal n_m    : signed(6 downto 0) := (others => '0');             -- strip offset term, Q6
    signal n_ph   : unsigned(15 downto 0) := (others => '0');
    signal n_gb   : unsigned(18 downto 0) := (others => '0');
    signal n_xc   : unsigned(10 downto 0) := (others => '0');          -- band shelf: centre x (px)
    signal n_hl   : unsigned(10 downto 0) := (others => '0');          -- ... half-length (the green halo: twice it)
    signal n_hal  : std_logic := '0';
    signal e_ss   : unsigned(13 downto 0) := (others => '0');          -- shelf profile down the strip (Q13)
    signal e_tl   : unsigned(5 downto 0) := (others => '0');           -- shelf length from the strip offset
    signal e_hal  : std_logic := '0';
    signal n_phi  : signed(12 downto 0) := (others => '0');            -- chevron phi, Q2 px
    signal n_to   : signed(11 downto 0) := (others => '0');            -- tongue offset, Q2 px
    signal n_w, n_w6 : unsigned(6 downto 0) := (others => '0');        -- tongue half-widths, Q2 px

    -- beam and line registers
    signal b_xh  : unsigned(11 downto 0) := (others => '0');
    signal b_tw  : unsigned(15 downto 0) := (others => '0');
    signal b_gb, b_gr : unsigned(18 downto 0) := (others => '0');
    signal b_sd  : signed(12 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_f1  : signed(11 downto 0) := (others => '0');
    signal l_f2  : signed(10 downto 0) := (others => '0');
    signal l_m   : signed(6 downto 0) := (others => '0');
    signal l_xc  : unsigned(10 downto 0) := (others => '0');
    signal l_hl  : unsigned(10 downto 0) := (others => '0');
    signal l_hal : std_logic := '0';
    signal l_phi : signed(12 downto 0) := (others => '0');
    signal l_to  : signed(11 downto 0) := (others => '0');
    signal l_w, l_w6 : unsigned(6 downto 0) := (others => '0');

    -- pixel path
    signal p2_xm, p3_xm, p4_xm, p5_xm, p6_xm, p7_xm, p8_xm : unsigned(11 downto 0) := (others => '0');
    signal p9_xb, p10_xd : signed(12 downto 0) := (others => '0');
    signal p11_ad : unsigned(11 downto 0) := (others => '0');
    signal p12_ks, p12_kh : std_logic := '0';
    signal p2_t2, p3_t2, p4_t2 : signed(6 downto 0) := (others => '0');
    signal p2_gl, p2_gr : unsigned(10 downto 0) := (others => '0');
    signal p2_sd, p3_sd, p4_sd, p5_sd, p6_sd, p7_sd, p8_sd : signed(12 downto 0) := (others => '0');
    signal p3_lg  : unsigned(2 downto 0) := (others => '1');
    signal p3_lm, p4_lm, p5_lm : signed(8 downto 0) := (others => '0');
    signal p2_l1, p2_l2, p2_l3 : unsigned(7 downto 0) := (others => '0');
    signal p4_lg  : unsigned(2 downto 0) := (others => '0');
    signal p4_col, p5_col : std_logic_vector(15 downto 0) := (others => '0');
    signal p5_sh, p6_sh, p7_sh, p8_sh, p9_sh, p10_sh, p11_sh, p12_sh : unsigned(2 downto 0) := (others => '1');
    signal p6_p1  : signed(19 downto 0) := (others => '0');
    signal p6_p2  : signed(16 downto 0) := (others => '0');
    signal p5_s   : signed(7 downto 0) := (others => '0');
    signal p6_q1  : signed(15 downto 0) := (others => '0');
    signal p6_et  : signed(14 downto 0) := (others => '0');
    signal p7_in  : signed(11 downto 0) := (others => '0');
    signal p7_x0  : signed(15 downto 0) := (others => '0');
    signal p7_w   : signed(13 downto 0) := (others => '0');
    signal p7_et  : signed(12 downto 0) := (others => '0');
    signal p8_dr  : signed(11 downto 0) := (others => '0');
    signal p8_xe  : signed(15 downto 0) := (others => '0');
    signal p8_et  : signed(12 downto 0) := (others => '0');
    signal p9_x   : signed(15 downto 0) := (others => '0');
    signal p9_sd  : signed(14 downto 0) := (others => '0');
    signal p10_t1, p10_t2 : signed(19 downto 0) := (others => '0');
    signal p10_sd : signed(14 downto 0) := (others => '0');
    signal p11_t  : signed(19 downto 0) := (others => '0');
    signal p11_sd : signed(12 downto 0) := (others => '0');
    signal p12_off : signed(13 downto 0) := (others => '0');
    signal p12_t  : unsigned(11 downto 0) := (others => '0');
    signal p12_dt : unsigned(12 downto 0) := (others => '0');
    signal p13_ix : signed(13 downto 0) := (others => '0');
    signal p13_phi : signed(13 downto 0) := (others => '0');
    signal p13_kt, p13_kd, p13_kb, p13_kg : std_logic := '0';
    signal p13_kl, p14_kl, p15_kl, p16_kl : std_logic := '0';
    signal p14_kc, p15_kc, p16_kc : std_logic := '0';
    signal p14_kt, p14_kd, p14_kb, p14_kg : std_logic := '0';
    signal p15_kt, p15_kd, p15_kb, p15_kg : std_logic := '0';
    signal p16_kt, p16_kd, p16_kb, p16_kg : std_logic := '0';
    signal p16_pf, p16_ch : std_logic_vector(31 downto 0) := (others => '0');
    signal p17_w  : std_logic_vector(31 downto 0) := (others => '0');
    signal p17_dk, p17_kg : std_logic := '0';
    signal p18_y, p18_u, p18_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent 8-sums, the 8-back tap from a
    -- block-RAM ring), luma [1 2 1]/4 on taps aligned with the chroma centre, luma 13/16
    type t_cring is array (0 to 255) of std_logic_vector(19 downto 0);
    signal cr : t_cring := (others => (others => '0'));
    signal cr_q, cr_f : std_logic_vector(19 downto 0) := (others => '0');
    type t_sring is array (0 to 255) of std_logic_vector(2 downto 0);
    signal sr : t_sring := (others => (others => '1'));
    signal sr_q, sr_f : std_logic_vector(2 downto 0) := (others => '1');
    attribute no_rw_check : boolean;
    attribute no_rw_check of cr : signal is true;
    attribute no_rw_check of sr : signal is true;
    signal rc     : unsigned(7 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_ly   : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y22, s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

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
            r_rn_q  <= C_VV_RND(to_integer(r_rn_a));
            r_col_q <= C_VV_COL(to_integer(r_col_a));
            r_pf_q  <= C_VV_PROF(to_integer(r_pf_a));
            r_ch_q  <= C_VV_CHEV(to_integer(r_ch_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_k  : std_logic_vector(31 downto 0);
        variable v_h  : signed(11 downto 0);
        variable v_t  : signed(15 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_a  : unsigned(6 downto 0);
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
            -- tide: bipolar, +-8 deadband
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
                s_mir <= registers_in(6)(0);
                s_flp <= registers_in(6)(1);
                s_fld <= registers_in(6)(2);
                s_ngt <= registers_in(6)(3);
                s_src <= registers_in(6)(4);
                s_xs  <= s_xsr;
                c_dv <= f_step(s_tid, 0, registers_in(6)(1)); c_dphi <= f_step(s_tid, 1, registers_in(6)(1));
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            case to_integer(unsigned(s_tid)) is                              -- Osc lattice step per line = vstep 121 / 256
                when 0 => s_oys <= to_unsigned(537, 10); when 4 => s_oys <= to_unsigned(272, 10);
                when 8 => s_oys <= to_unsigned(453, 10); when 12 => s_oys <= to_unsigned(226, 10);
                when 5 | 6 | 14 => s_oys <= to_unsigned(181, 10); when 1 | 2 | 10 => s_oys <= to_unsigned(242, 10);
                when others => s_oys <= to_unsigned(121, 10);
            end case;

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

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result read at 50 + 24 k.
            -- The slot number is decoded at 30 + 24 k and the operands are pre-muxed into z_fa / z_fb at
            -- 31 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(12, 4); z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(13, 4); z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(14, 4); z_pre <= '1';
                    when 390 => z_sel <= to_unsigned(15, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(c_ks, 28);           z_fb <= resize(s_ys, 16);          -- strips / line
                    when 1  => z_fa <= resize(z_d, 28);            z_fb <= to_unsigned(1853, 16);     -- on-length opens (P12)
                    when 2  => z_fa <= resize(s_gp(12 downto 3), 28); z_fb <= resize(s_gp(12 downto 3), 16);  -- p^2
                    when 3  => z_fa <= resize(c_p2(19 downto 4), 28); z_fb <= to_unsigned(2714, 16);  -- depth
                    when 4  => z_fa <= resize(c_dep, 28);          z_fb <= to_unsigned(36, 16);       -- band amplitude
                    when 5  => z_fa <= resize(s_vm, 28);           z_fb <= resize(s_vm, 16);          -- tide^2
                    when 6  => z_fa <= resize(c_m2, 28);           z_fb <= to_unsigned(183, 16);
                    when 7  => z_fa <= resize(c_ks, 28);           z_fb <= resize(c_dr, 16);          -- roll in strips
                    when 8  => z_fa <= resize(z_d, 28);            z_fb <= to_unsigned(960, 16);      -- wrap threshold
                    when 9  => z_fa <= resize(s_g6(12 downto 3), 28); z_fb <= resize(s_g6(12 downto 3), 16);  -- etch^2
                    when 10 => z_fa <= resize(c_e2, 28);           z_fb <= to_unsigned(117, 16);
                    when 11 => z_fa <= resize(z_hm, 28);           z_fb <= to_unsigned(4920, 16);     -- Hem
                    when 12 => z_fa <= resize(z_hm, 28);           z_fb <= to_unsigned(1297, 16);     -- apex row
                    when 13 => z_fa <= resize(c_ya, 28);           z_fb <= to_unsigned(151, 16);      -- phi of line 0
                    when 14 => z_fa <= resize(z_sfdb, 28);         z_fb <= resize(z_d, 16);           -- fade opens (P12)
                    when others => z_fa <= resize(z_fd0, 28);      z_fb <= resize(z_d, 16);
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            elsif l_ld = '1' then                 -- the line engine shares the multiplier outside the zone
                f_acc <= (others => '0');
                f_a <= resize(l_fa, 28); f_b <= resize(l_fb, 16);
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(26 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion: last field's roll; beam origin; sway; |k5 - 340|; exponents (1/8 octave)
                    when 20 =>
                        if s_vneg = '1' then
                            c_gb0 <= c_gb0 - resize(c_dr & "0000", 19); c_sp0 <= c_sp0 - c_drs;
                        else
                            c_gb0 <= c_gb0 + resize(c_dr & "0000", 19); c_sp0 <= c_sp0 + c_drs;
                        end if;
                        c_xb0 <= resize(s_x0, 12);
                        c_gs <= signed(resize(s_g1(12 downto 3), 11)) - 256;
                        if s_g5(12 downto 3) >= 340 then
                            z_hneg <= '0'; z_hm <= s_g5(12 downto 3) - 340;
                        else
                            z_hneg <= '1'; z_hm <= 340 - s_g5(12 downto 3);
                        end if;
                        if s_src = '1' then c_loff <= to_unsigned(111, 7); else c_loff <= (others => '0'); end if;
                        if s_gp(12 downto 3) <= 507 then z_d <= (others => '0');
                        elsif s_gp(12 downto 3) >= 1019 then z_d <= to_unsigned(512, 10);
                        else z_d <= s_gp(12 downto 3) - 507; end if;
                        v_d := signed(resize(s_g2(12 downto 3), 11)) - 512;                 -- pleats: 13 / 512
                        v_t := shift_left(resize(v_d, 16), 4) + shift_left(resize(v_d, 16), 3) + shift_left(resize(v_d, 16), 1) + 512;
                        z_e2 <= resize(shift_right(v_t, 10), 5);
                        v_d := signed(resize(s_g4(12 downto 3), 11)) - 512;                 -- swell: 12 / 512
                        v_t := shift_left(resize(v_d, 16), 3) + shift_left(resize(v_d, 16), 2) + 512;
                        z_e4 <= resize(shift_right(v_t, 10), 5);
                    when 21 =>
                        c_pm <= unsigned(z_e2(2 downto 0)); c_ps <= z_e2(4 downto 3);
                        z_kb <= to_unsigned(26216, 17);
                        if z_e4(0) = '1' then z_kb <= to_unsigned(26216 + 3277, 17); end if;
                        if z_e4(1) = '1' then z_kb <= to_unsigned(26216 + 6554, 17); end if;
                        if z_e4(1 downto 0) = "11" then z_kb <= to_unsigned(26216 + 9831, 17); end if;
                    when 22 =>
                        if z_e4(2) = '1' then z_kb <= z_kb + to_unsigned(13108, 17); end if;
                        v_k := C_VV_DARK;
                        c_ty <= unsigned(v_k(31 downto 22));
                        c_tu <= unsigned(v_k(21 downto 13) & '0'); c_tv <= unsigned(v_k(12 downto 4) & '0');
                    when 23 =>
                        case z_e4(4 downto 3) is
                            when "10"   => c_ks <= resize(z_kb(16 downto 2), 18);
                            when "11"   => c_ks <= resize(z_kb(16 downto 1), 18);
                            when "00"   => c_ks <= resize(z_kb, 18);
                            when others => c_ks <= z_kb & '0';
                        end case;
                        -- beam starts and steps (mirror / resolution)
                        case to_integer(s_xs) is
                            when 4 =>
                                c_stw <= to_signed(468, 10);
                                c_sgb <= to_signed(78, 9);  c_ssd <= to_signed(8, 8);
                                c_tw0 <= to_unsigned(56174, 16);
                                c_gbx0 <= to_unsigned(459269, 19); c_sd0 <= to_signed(-1112, 13);
                                z_gbx0 <= to_unsigned(459269, 19); z_fd0 <= to_unsigned(44032, 16); z_sfdb <= to_unsigned(102, 7);
                            when 3 =>
                                c_stw <= to_signed(351, 10);
                                c_sgb <= to_signed(59, 9);  c_ssd <= to_signed(6, 8);
                                c_tw0 <= to_unsigned(0, 16);
                                c_gbx0 <= to_unsigned(449869, 19); c_sd0 <= to_signed(1704, 13);
                                z_gbx0 <= to_unsigned(449869, 19); z_fd0 <= to_unsigned(56320, 16); z_sfdb <= to_unsigned(77, 7);
                            when others =>
                                c_stw <= to_signed(234, 10);
                                c_sgb <= to_signed(39, 9);  c_ssd <= to_signed(4, 8);
                                c_tw0 <= to_unsigned(0, 16);
                                c_gbx0 <= to_unsigned(449869, 19); c_sd0 <= to_signed(1704, 13);
                                z_gbx0 <= to_unsigned(449869, 19); z_fd0 <= to_unsigned(56320, 16); z_sfdb <= to_unsigned(51, 7);
                        end case;

                    -- products
                    when 50  => c_dsp <= f_acc(27 downto 12);
                    when 74  => c_on  <= to_unsigned(1070, 11) + resize(f_acc(19 downto 10), 11);    -- 1070 + 1.81 d
                    when 98  => c_p2  <= f_acc(19 downto 0);
                    when 122 => c_dep <= to_unsigned(358, 12) + resize(f_acc(27 downto 16), 12);
                    when 146 =>
                        if f_acc(27 downto 10) > 127 then c_a1 <= to_unsigned(127, 7); else c_a1 <= f_acc(16 downto 10); end if;
                    when 147 => c_a1q <= shift_right(c_a1, 1) + shift_right(c_a1, 2);
                    when 170 => c_m2  <= f_acc(17 downto 0);
                    when 194 => c_dr  <= f_acc(25 downto 16);
                    when 218 => c_drs <= resize(f_acc(27 downto 8), 24);
                    when 242 => c_neg <= to_unsigned(1536, 11) + resize(f_acc(18 downto 10), 11);    -- 1536 + 0.94 d
                    when 266 => c_e2  <= f_acc(19 downto 0);
                    when 290 => c_g6  <= f_acc(26 downto 21);
                    when 314 =>
                        if z_hneg = '1' then c_hem <= -signed(resize(f_acc(23 downto 10), 14));
                        else c_hem <= signed(resize(f_acc(23 downto 10), 14)); end if;
                    when 338 =>
                        if z_hneg = '1' then c_ya <= to_unsigned(655, 12) - resize(f_acc(21 downto 10), 12);
                        else c_ya <= to_unsigned(655, 12) + resize(f_acc(21 downto 10), 12); end if;
                    when 362 => c_phi0 <= -signed(resize(f_acc(19 downto 1), 20));
                    when 386 => c_sfd <= z_sfdb - resize(f_acc(15 downto 9), 7);
                    when 410 => c_grx0 <= z_gbx0 - resize(z_fd0, 19) + resize(f_acc(24 downto 9), 19);
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
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
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line on one serial multiplier (13 iterations,
    -- result at load + 14):
    --   F1 = Sway sgn(u) u^2 (u = v - Hem), F2 = Sway v (Hang = Floor: v runs 1 -> 0)
    --   strips: target phase / offset from the RND ROM at the strip index, slewed 1/4 per line
    --   the strip offset A3 of; the tongue half-width 104 (1 - |2p - 1|) (triangle)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_u  : signed(13 downto 0);
        variable v_d  : signed(8 downto 0);
        variable v_o  : signed(9 downto 0);
        variable v_r9 : signed(8 downto 0);
        variable v_t  : unsigned(29 downto 0);
        variable v_au : unsigned(13 downto 0);
        variable v_ph : signed(13 downto 0);
        variable v_w7 : unsigned(6 downto 0);
        variable v_tof : signed(7 downto 0);
        variable v_ls : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_sp <= c_sp0; e_gb <= c_gb0;
                if s_flp = '1' then
                    e_v <= to_signed(65536, 18);
                    e_phi <= c_phi0 + 81562;
                else
                    e_v <= (others => '0');
                    e_phi <= c_phi0;
                end if;
                e_tok <= '0';
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_sp  <= e_sp + resize(c_dsp, 24);
                e_gb  <= e_gb + resize(s_ys, 19);
                e_v   <= e_v + resize(c_dv, 18);
                e_phi <= e_phi + resize(c_dphi, 20);
                e_tok <= '1';
                e_lc  <= (others => '0');
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            l_ld <= '0';

            case to_integer(e_lc) is
                when 1 =>
                    -- slot A: |u| |u| (u Q12)
                    r_rn_a <= e_sp(23 downto 16);
                    v_u := resize(e_v(17 downto 4), 14) - resize(c_hem, 14);
                    e_un <= v_u(13);
                    v_au := unsigned(abs(v_u));
                    l_fa <= resize(v_au, 16); l_fb <= v_au(12 downto 0); l_ld <= '1';
                    n_gb <= e_gb;
                    n_phi <= resize(shift_right(e_phi, 6), 13);
                when 4 =>
                    -- strips: target (RND ROM, 2 clocks + fabric copy), slew 1/4 per line, dark key on big steps
                    v_d := signed(unsigned(r_rn_q(15 downto 7)) - e_ph);
                    v_r9 := signed(r_rn_q(6 downto 0)) & "00";
                    if e_tok = '0' then
                        e_ph <= unsigned(r_rn_q(15 downto 7));
                        e_of <= v_r9;
                    else
                        e_ph <= e_ph + unsigned(resize(shift_right(v_d, 2), 9));
                        v_o := resize(v_r9, 10) - resize(e_of, 10);
                        e_of <= e_of + resize(shift_right(v_o, 2), 9);
                    end if;
                    -- the strip's dark-green shelf: centred at its phase x 4, as long as its offset is big (|tof| - 24),
                    -- a flat-topped lens over the strip's first half (Q13 profile), a green halo over its first 5/8
                    n_xc <= unsigned(r_rn_q(15 downto 7)) & "00";
                    v_tof := abs(signed(resize(signed(r_rn_q(6 downto 0)), 8)));
                    if v_tof > 24 then e_tl <= resize(unsigned(v_tof) - 24, 6); else e_tl <= (others => '0'); end if;
                    if e_sp(15) = '0' then                     -- first half of the strip: a flat-topped lens
                        v_ls := to_unsigned(8192, 14) - resize(unsigned(abs(signed('0' & e_sp(14 downto 1)) - 8192)), 14);
                        if v_ls >= 4096 then e_ss <= to_unsigned(8192, 14); else e_ss <= v_ls(12 downto 0) & '0'; end if;
                    else
                        e_ss <= (others => '0');
                    end if;
                    if e_sp(15 downto 13) < 5 then e_hal <= '1'; else e_hal <= '0'; end if;
                when 6 =>
                    n_ph <= e_ph & "0000000";
                    n_m <= resize(shift_right(e_of, 3) - shift_right(e_of, 6), 7);    -- 7/16 of, Q6
                when 16 =>
                    -- slot B: (u^2 >> 16, Q8) |Sway|
                    l_fa <= resize(f_acc(25 downto 16), 16); l_fb <= resize(unsigned(abs(c_gs)), 13); l_ld <= '1';
                when 31 =>
                    -- F1 (Q8); slot C: |v| (Q8) |Sway|
                    v_t := resize(shift_right(f_acc, 8), 30);
                    if v_t > 2047 then v_t := to_unsigned(2047, 30); end if;
                    if (e_un xor c_gs(10)) = '1' then n_f1 <= -signed(resize(v_t(10 downto 0), 12));
                    else n_f1 <= signed(resize(v_t(10 downto 0), 12)); end if;
                    if e_v < 0 then l_fa <= (others => '0');
                    else l_fa <= resize(unsigned(e_v(17 downto 8)), 16); end if;
                    l_fb <= resize(unsigned(abs(c_gs)), 13); l_ld <= '1';
                when 46 =>
                    -- F2 (Q8)
                    v_t := resize(shift_right(f_acc, 8), 30);
                    if v_t > 1023 then v_t := to_unsigned(1023, 30); end if;
                    if c_gs(10) = '1' then n_f2 <= -signed(resize(v_t(9 downto 0), 11));
                    else n_f2 <= signed(resize(v_t(9 downto 0), 11)); end if;
                    -- tongue, from the chevron line term phi (Q2 px): half-width 104 - 0.328 |phi - 82|
                    v_ph := resize(shift_right(e_phi, 6), 14) - 82;
                    l_fa <= resize(unsigned(abs(v_ph)), 16); l_fb <= to_unsigned(84, 13); l_ld <= '1';
                when 61 =>
                    v_t := resize(shift_right(f_acc, 8), 30);
                    if v_t >= 104 then
                        n_w <= (others => '0'); n_w6 <= (others => '0');
                    else
                        v_w7 := to_unsigned(104, 7) - v_t(6 downto 0);
                        n_w <= v_w7; n_w6 <= shift_right(v_w7, 1) + shift_right(v_w7, 3);
                    end if;
                    -- ... and its offset from the axis 428 + 0.203 phi
                    v_ph := resize(shift_right(e_phi, 6), 14);
                    e_tsg <= v_ph(13);
                    l_fa <= resize(unsigned(abs(v_ph)), 16); l_fb <= to_unsigned(52, 13); l_ld <= '1';
                when 76 =>
                    v_t := resize(shift_right(f_acc, 8), 30);
                    if e_tsg = '1' then n_to <= to_signed(428, 12) - signed(resize(v_t(9 downto 0), 12));
                    else n_to <= to_signed(428, 12) + signed(resize(v_t(9 downto 0), 12)); end if;
                    -- shelf half-length = profile x A1 x (|tof| - 24) / 4 (two products)
                    l_fa <= resize(e_ss, 16); l_fb <= resize(c_a1, 13); l_ld <= '1';
                when 91 =>
                    l_fa <= resize(f_acc(20 downto 7), 16); l_fb <= resize(e_tl, 13); l_ld <= '1';
                when 106 =>
                    v_t := resize(shift_right(f_acc, 8), 30);
                    if v_t > 320 then v_t := to_unsigned(320, 30); end if;          -- shelves stay finite blobs
                    n_hl <= v_t(10 downto 0);
                    n_hal <= e_hal;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the mirrored virtual x (h units), the strip-triangle and wave
    -- phases, the band coordinate and the signed distance to the nearest
    -- chevron axis; with Mirror every DDA runs backward over the left half
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_sd : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_xh <= c_xb0;
                b_tw <= n_ph + c_tw0;
                b_gb <= n_gb + c_gbx0;
                b_gr <= n_gb + c_grx0;
                b_sd <= c_sd0;
                b_px <= (others => '0');
                l_f1 <= n_f1; l_f2 <= n_f2; l_m <= n_m;
                l_xc <= n_xc; l_hl <= n_hl; l_hal <= n_hal;
                l_phi <= n_phi; l_to <= n_to; l_w <= n_w; l_w6 <= n_w6;
            elsif g_avid = '1' then
                b_xh <= b_xh + s_xs;
                b_px <= b_px + 1;
                b_tw <= unsigned(signed(b_tw) + resize(c_stw, 16));
                b_gb <= unsigned(signed(b_gb) + resize(c_sgb, 19));
                -- the band's rising edge falls behind faster left of x = 1100 (the band fades in toward the right)
                if b_xh < 2200 then
                    b_gr <= unsigned(signed(b_gr) + resize(c_sgb, 19)) + resize(c_sfd, 19);
                else
                    b_gr <= unsigned(signed(b_gr) + resize(c_sgb, 19));
                end if;
                v_sd := b_sd + resize(c_ssd, 13);
                if v_sd >= 1888 then v_sd := v_sd - 3776;
                elsif v_sd < -1888 then v_sd := v_sd + 3776; end if;
                b_sd <= v_sd;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_xm  : unsigned(11 downto 0);
        variable v_gr  : unsigned(10 downto 0);
        variable v_tf  : signed(12 downto 0);
        variable v_l4  : unsigned(9 downto 0);
        variable v_in  : signed(12 downto 0);
        variable v_x0  : signed(16 downto 0);
        variable v_x   : signed(16 downto 0);
        variable v_sd  : signed(14 downto 0);
        variable v_xc  : signed(19 downto 0);
        variable v_ix  : signed(13 downto 0);
        variable v_c   : signed(13 downto 0);
        variable v_dt  : signed(13 downto 0);
        variable v_w   : std_logic_vector(31 downto 0);
        variable v_n   : unsigned(2 downto 0);
        variable v_et  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: mirrored x; table address; the two triangles; band coordinate
            if s_mir = '1' and b_xh < 1920 then v_xm := to_unsigned(3840, 12) - b_xh; else v_xm := b_xh; end if;
            p2_xm <= v_xm;
            r_col_a <= v_xm(11 downto 3);
            p2_t2 <= f_tri(b_tw);
            p2_gl <= b_gb(18 downto 8);
            p2_gr <= b_gr(18 downto 8);
            p2_sd <= b_sd;
            -- 3: band gate level, half-octave steps (x1, 3/4, 1/2, 3/8, 1/4, 3/16, 1/8; 7 = off): the rising edge
            -- (~150 lines) on the fade-shifted coordinate, a hard trailing edge after 1070 lines; Flood: x1
            v_gr := p2_gr;
            v_tf := signed(resize(p2_gl & '0', 13)) - signed(resize(v_gr, 13));   -- trailing edge, fade-shifted
            if s_fld = '1' then p3_lg <= "000";
            elsif p2_gl >= c_on or v_gr >= c_neg or v_tf >= signed(resize(c_on, 13)) then p3_lg <= "111";
            elsif v_gr >= 154 then p3_lg <= "000";
            elsif v_gr >= 109 then p3_lg <= "001";
            elsif v_gr >= 77 then p3_lg <= "010";
            elsif v_gr >= 54 then p3_lg <= "011";
            elsif v_gr >= 38 then p3_lg <= "100";
            elsif v_gr >= 27 then p3_lg <= "101";
            elsif v_gr >= 19 then p3_lg <= "110";
            else p3_lg <= "111"; end if;
            p3_t2 <= p2_t2; p3_xm <= p2_xm; p3_sd <= p2_sd;
            p2_l1 <= unsigned(s_lb_q); p2_l2 <= p2_l1; p2_l3 <= p2_l2;
            v_l4 := resize(unsigned(s_lb_q), 10) + resize(p2_l1, 10) + resize(p2_l2, 10) + resize(p2_l3, 10);
            p3_lm <= signed(resize(v_l4(9 downto 2), 9)) - signed(resize(c_loff, 9));
            -- 4: EBR data -> fabric
            p4_lg <= p3_lg;
            p4_t2 <= p3_t2; p4_xm <= p3_xm; p4_sd <= p3_sd; p4_lm <= p3_lm;
            p4_col <= r_col_q;
            -- 5: band level (7 = off); its amplitude (odd levels: x 3/4)
            p5_sh <= p4_lg;
            if p4_lg(0) = '1' then p5_amp <= c_a1q; else p5_amp <= c_a1; end if;
            p5_s <= resize(p4_t2, 8) + resize(l_m, 8);                                    -- A1 (t2 + 7/16 of)
            p5_col <= p4_col; p5_xm <= p4_xm; p5_sd <= p4_sd; p5_lm <= p4_lm;
            -- 6: products
            p6_p1 <= signed(p5_col(13 downto 6)) * l_f1;
            p6_p2 <= signed(p5_col(5 downto 0)) * l_f2;
            p6_q1 <= p5_s * signed('0' & p5_amp);
            p6_et <= resize(shift_right(p5_lm, 1), 8) * signed('0' & c_g6);
            p6_sh <= p5_sh; p6_xm <= p5_xm; p6_sd <= p5_sd;
            -- 7: X0 = x + sway (Q2 px); band inner (Q2 px); etch (Q2 px; Osc twice)
            v_x0 := signed(resize(p6_xm & '0', 17)) + resize(shift_right(p6_p1, 4), 17) + resize(shift_right(p6_p2, 3), 17);
            p7_x0 <= resize(v_x0, 16);
            v_in := resize(shift_right(p6_q1, 2), 13);
            if v_in > 2047 then p7_in <= to_signed(2047, 12);
            elsif v_in < -2048 then p7_in <= to_signed(-2048, 12);
            else p7_in <= resize(v_in, 12); end if;
            if s_src = '1' then p7_et <= resize(p6_et, 13); else p7_et <= resize(shift_right(p6_et, 2), 13); end if;   -- Osc: 4x
            p7_sh <= p6_sh; p7_sd <= p6_sd; p7_xm <= p6_xm;
            -- 8: band displacement = inner >> shift; X0 + etch
            case to_integer(p7_sh) is
                when 0 | 1 => p8_dr <= p7_in;
                when 2 | 3 => p8_dr <= shift_right(p7_in, 1);
                when 4 | 5 => p8_dr <= shift_right(p7_in, 2);
                when 6     => p8_dr <= shift_right(p7_in, 3);
                when others => p8_dr <= (others => '0');
            end case;
            p8_xe <= p7_x0 + resize(p7_et, 16);
            p8_et <= p7_et;
            p8_sh <= p7_sh; p8_sd <= p7_sd; p8_xm <= p7_xm;
            -- 9: X; displaced axis distance
            p9_x <= p8_xe + resize(p8_dr, 16);
            p9_sd <= resize(p8_sd, 15) + resize(p8_dr, 15) + resize(p8_et, 15);
            p9_sh <= p8_sh;
            p9_xb <= signed(resize(p8_xm(11 downto 1), 13)) + resize(shift_right(p8_dr, 2), 13);   -- band-displaced x (px)
            -- 10: pleats partials, (X - 960) (8 + pm); axis distance wrap (1)
            v_xc := resize(p9_x, 20) - 3840;
            if c_pm(0) = '1' then p10_t1 <= shift_left(v_xc, 3) + v_xc; else p10_t1 <= shift_left(v_xc, 3); end if;
            case c_pm(2 downto 1) is
                when "00"   => p10_t2 <= (others => '0');
                when "01"   => p10_t2 <= shift_left(v_xc, 1);
                when "10"   => p10_t2 <= shift_left(v_xc, 2);
                when others => p10_t2 <= shift_left(v_xc, 2) + shift_left(v_xc, 1);
            end case;
            v_sd := p9_sd;
            if v_sd >= 1888 then v_sd := v_sd - 3776; elsif v_sd < -1888 then v_sd := v_sd + 3776; end if;
            p10_sd <= v_sd;
            p10_sh <= p9_sh;
            p10_xd <= p9_xb - signed(resize(l_xc, 13));
            -- 11: sum; axis distance wrap (2)
            p11_t <= p10_t1 + p10_t2;
            p11_sd <= resize(p10_sd, 13);
            p11_sh <= p10_sh;
            if p10_xd < -4095 or p10_xd > 4095 then p11_ad <= (others => '1');
            else p11_ad <= unsigned(resize(abs(p10_xd), 12)); end if;
            -- 12: profile offset = (X - 960) (8 + pm) 2^ps / 128 (4 virtual px per entry); |axis distance|; tongue
            case c_ps is
                when "10"   => p12_off <= resize(shift_right(p11_t, 9), 14);
                when "11"   => p12_off <= resize(shift_right(p11_t, 8), 14);
                when "00"   => p12_off <= resize(shift_right(p11_t, 7), 14);
                when others => p12_off <= resize(shift_right(p11_t, 6), 14);
            end case;
            p12_t <= unsigned(resize(abs(p11_sd), 12));
            v_dt := resize(p11_sd, 14) + resize(l_to, 14);
            p12_dt <= unsigned(resize(abs(v_dt), 13));
            p12_sh <= p11_sh;
            if p11_ad < l_hl then p12_ks <= '1'; else p12_ks <= '0'; end if;
            if p11_ad(11 downto 1) < l_hl and l_hal = '1' then p12_kh <= '1'; else p12_kh <= '0'; end if;
            -- 13: profile index; chevron phi; keys
            p13_ix <= p12_off + 512;
            p13_phi <= resize(l_phi, 14) - signed(resize(p12_t, 14));
            if p12_dt < l_w then p13_kt <= '1'; else p13_kt <= '0'; end if;
            if p12_dt < l_w6 then p13_kd <= '1'; else p13_kd <= '0'; end if;
            if p12_ks = '1' and p12_sh <= 3 then p13_kb <= '1'; else p13_kb <= '0'; end if;    -- dark-green shelf
            if p12_kh = '1' and p12_sh <= 1 then p13_kg <= '1'; else p13_kg <= '0'; end if;    -- green glints around it
            if p12_sh /= 7 then p13_kl <= '1'; else p13_kl <= '0'; end if;                       -- inside the band
            -- 14: ROM addresses: profile (mirror-folded every 1024 entries), chevron (phi + 16, folded)
            v_ix := p13_ix;
            if v_ix(10) = '1' then r_pf_a <= not unsigned(v_ix(9 downto 0)); else r_pf_a <= unsigned(v_ix(9 downto 0)); end if;
            v_c := shift_right(p13_phi, 2) + 16;
            if v_c >= 256 then v_c := to_signed(255, 14); end if;
            if v_c < 0 then p14_kc <= '0'; r_ch_a <= (others => '0');
            else p14_kc <= '1'; r_ch_a <= unsigned(v_c(7 downto 0)); end if;
            p14_kt <= p13_kt; p14_kd <= p13_kd; p14_kb <= p13_kb; p14_kg <= p13_kg; p14_kl <= p13_kl;
            -- 15: ROM data (EBR)
            p15_kc <= p14_kc; p15_kt <= p14_kt; p15_kd <= p14_kd; p15_kb <= p14_kb; p15_kg <= p14_kg; p15_kl <= p14_kl;
            -- 16: fabric copies
            p16_pf <= r_pf_q; p16_ch <= r_ch_q;
            p16_kc <= p15_kc; p16_kt <= p15_kt; p16_kd <= p15_kd; p16_kb <= p15_kb; p16_kg <= p15_kg; p16_kl <= p15_kl;
            -- 17: select: band shelf / tongue core -> dark green, tongue rim -> violet, chevron, profile
            if p16_kb = '1' or (p16_kd = '1' and p16_kl = '0') then     -- (in the band the tongue core glows violet)
                p17_w <= C_VV_DARK; p17_dk <= '1';
            elsif p16_kt = '1' then
                p17_w <= C_VV_VIOL; p17_dk <= '1';
            elsif p16_kc = '1' then
                p17_w <= p16_ch; p17_dk <= '0';
            elsif p16_kl = '1' and unsigned(p16_pf(31 downto 22)) < 320 then
                p17_w <= C_VV_VIOL; p17_dk <= '1';        -- smeared dark strands glow violet inside the band
            else
                p17_w <= p16_pf; p17_dk <= '0';
            end if;
            p17_kg <= p16_kg;
            -- 18: Night palette; green glints inside the band (bright luma takes the green's chroma)
            v_w := p17_w;
            if s_ngt = '1' then
                if p17_dk = '1' then v_n := "000"; else v_n := unsigned(p17_w(3 downto 1)); end if;
                v_w := C_VV_NPAL(to_integer(v_n));
            end if;
            p18_y <= unsigned(v_w(31 downto 22));
            if p17_kg = '1' and unsigned(v_w(31 downto 22)) > 760 then
                p18_u <= unsigned(C_VV_GRN(21 downto 13) & '0'); p18_v <= unsigned(C_VV_GRN(12 downto 4) & '0');
            else
                p18_u <= unsigned(v_w(21 downto 13) & '0'); p18_v <= unsigned(v_w(12 downto 4) & '0');
            end if;
            -- 19: filter input, dark green outside the generated avid
            if s_gav_sr(16) = '0' then
                s_y19 <= c_ty; s_u19 <= c_tu; s_v19 <= c_tv;
            else
                s_y19 <= p18_y; s_u19 <= p18_u; s_v19 <= p18_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(16);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: chroma through one [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums; the 8-back tap from a block-RAM ring), luma [1 2 1]/4 on
    -- taps aligned with the chroma centre (the HD HDMI bar fix), luma 13/16 and
    -- the blanking gate.  The syncs ride a block-RAM ring of C_LAT.
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rc <= rc + 1;
            -- chroma ring: written at rc + 6, read at rc, RAM register, fabric register = the input of 8
            -- clocks before (never the write address)
            cr(to_integer(rc + 6)) <= std_logic_vector(s_u19) & std_logic_vector(s_v19);
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            -- sync ring: written at rc + 22, RAM register, fabric register = C_LAT clocks
            sr(to_integer(rc + 22)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
        end if;
    end process p_ring;

    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_l  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 20: chroma running 8-sums (snap back to the off-avid colour once 8 blank samples are in, so a
            -- glitched sample can't offset the colours for good); luma taps
            if s_bah = "00000000" then
                s_bau <= c_tu & "000"; s_bav <= c_tv & "000";
            else
                s_bau <= s_bau + s_u19 - unsigned(cr_f(19 downto 10));
                s_bav <= s_bav + s_v19 - unsigned(cr_f(9 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            s_ly(0) <= s_y19;
            for i in 1 to 5 loop s_ly(i) <= s_ly(i - 1); end loop;
            -- 21: chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4 (both centred 4 pixels back)
            v_sc := resize(s_bau, 14) + s_baud; s_u21 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v21 <= v_sc(13 downto 4);
            v_l := resize(s_ly(3), 12) + shift_left(resize(s_ly(4), 12), 1) + resize(s_ly(5), 12);
            s_y21 <= v_l(11 downto 2);
            -- 22: luma 13/16
            if s_y21 > 64 then v_dl := s_y21 - 64; else v_dl := (others => '0'); end if;
            s_y22 <= s_y21 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u22 <= s_u21; s_v22 <= s_v21;
            -- 23, 24: pipeline
            s_y23 <= s_y22; s_u23 <= s_u22; s_v23 <= s_v22;
            s_y24 <= s_y23; s_u24 <= s_u23; s_v24 <= s_v23;
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
    data_out.hsync_n <= sr_f(2);
    data_out.vsync_n <= sr_f(1);
    data_out.field_n <= sr_f(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture velvet;
