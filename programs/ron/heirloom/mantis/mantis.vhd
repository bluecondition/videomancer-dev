-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Mantis v0.1.1 -- Heirloom from the video clips (phase 3b), made from the still WIN_20230906_22_43_27 @ 2.5 s.
--
-- v0.1.1: the default Etch is a quarter of v0.1.0's bend (K6 104, about 8 px: clean wing curves over a live camera)
-- and the Osc lattice is fed x16 instead of x4, so the Osc flip at the default is unchanged; the push saturates.
--
-- The still: a green / purple / white kaleidoscope insect, mirrored top to bottom about row 518 and (roughly) left
-- to right about x 1075.  Two leaves per quadrant form an X of wings (magenta / green above, white / pink below),
-- dark "<" chevrons (an arm from each side edge plus a shadow from the top edge) meet at the wing tips, pairs of
-- pillars (period 490 px) stand behind it, lime / white lobes hang between the pillars under the arms, and fine
-- wood-grain contours fill the bands near the centre row.  In the clip nothing moves but the colour: every pixel
-- pulses through the palette once per 8.93 s, the bands sliding down through the static insect.
--
-- The patch, as fitted (mantis.py has every table):
--   channel_c = tone_c( cos(2 pi (psi_c(y) - tau)) + row_c(Y) + L_c )      c = R, G, B
--   psi_c     = a_c + m_c y + A_c w(frac((y - 150 - off_c) / P)): a skewed-triangle vertical oscillator (2.9 per
--               frame) patched into the three keyers at different gains, G half a cycle late
--   L_c       = col_c(X) + sum of the element-pair weights (10 shape elements, 2-bit soft keys) + grain_c
--   X = |x - 1075| (Twin: ||x - 960| - 480|) + etch * luma (the camera pushes the shapes sideways), Y = |y - 518|;
--   the wings are drawn in X / kw (P12)
--
--   K1  Pillars  pillar spacing: x0.25 (dense) .. the clip (512, 490 px) .. x4 (one pair)
--   K2  Arms     the arms' height: dropped onto the centre row (0) .. the clip (512) .. raised to the edges (1023)
--   K3  Bands    the vertical oscillator: one colour segment per frame (0) .. the clip (2.9) .. 8.7 (1023)
--   K4  Flow     bipolar colour flow, +-8 deadband; 687 = the clip (one cycle per 8.93 s, bands sliding down)
--   K5  Grain    wood-grain contour depth: none (0) .. the clip (512) .. x3.9 (1023)
--   K6  Etch     how far the camera (or the Osc lattice, x16) pushes the shapes sideways: 0 .. 89 px (default 104, 8 px)
--   S7  Twin     one mantis / two side by side
--   S8  Hue      R G B / G B R: the keyers rotated (green -> blue, magenta -> yellow ...)
--   S9  Key      Soft (the clip) / Hard: the keyers' gain x4, posterised neon edges
--   S10 Invert   the shape signal inverted: the insect's light and dark parts swap
--   S11 Source   Video / Osc
--   P12 WINGS    the wings' span x0.35 .. x2.8: folded along the spine (0) .. the clip (512) .. spread wide (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line start
-- (computes the NEXT line: the three vertical waves, the row table, the wing geometry), a 21-stage pixel path.
-- Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (Y, U and V through
-- a [1 2 2 2 2 2 2 2 1]/16 box: two 8-sums over a block-RAM delay ring, with the running-sum resync; luma 13/16)
-- before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.mantis_pkg.all;
use work.all;

architecture mantis of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- vertical oscillator offsets (Q24 cycles), ramps (Q24 per virtual line) and gains (Q12), mantis.py AMA
    constant C_A0 : integer := 9319274;    -- frac(-0.444528)
    constant C_A1 : integer := 9033759;    -- frac(-0.461546)
    constant C_A2 : integer := 16657410;   -- frac(-0.007141)
    constant C_RMP0 : integer := 7737;     -- |m_R| (negative)
    constant C_RMP1 : integer := 7621;     -- |m_G| (negative)
    constant C_RMP2 : integer := 37415;    -- m_B (positive)
    constant C_AR : integer := 1189;       -- A_R
    constant C_AG : integer := 1299;       -- A_G
    constant C_AB : integer := 823;        -- |A_B| (negative)

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
    function f_tri15(v : unsigned(15 downto 0)) return unsigned is
    begin
        if v(15) = '1' then return not v(14 downto 0); else return v(14 downto 0); end if;
    end function;
    -- 2-bit soft key of a signed inside-distance: (d >= -8) + (d >= 0) + (d >= 8), from the bits alone
    function f_al(d : signed) return unsigned is
        variable n : unsigned(1 downto 0);
        variable hi_any, hi_all : std_logic;
    begin
        hi_any := '0'; hi_all := '1';
        for i in d'high - 1 downto d'low + 3 loop
            hi_any := hi_any or d(i);
            hi_all := hi_all and d(i);
        end loop;
        if d(d'high) = '0' then
            if hi_any = '1' then n := "11"; else n := "10"; end if;
        else
            if hi_all = '1' then n := "01"; else n := "00"; end if;
        end if;
        return n;
    end function;
    -- the key of -d: (d <= 8) + (d <= 0) + (d <= -8), from the bits alone
    function f_aln(d : signed) return unsigned is
        variable n : unsigned(1 downto 0);
        variable hi_all, hi4_any, lo_any : std_logic;
    begin
        hi_all := '1'; hi4_any := '0'; lo_any := d(d'low) or d(d'low + 1) or d(d'low + 2);
        for i in d'high - 1 downto d'low + 3 loop
            hi_all := hi_all and d(i);
        end loop;
        for i in d'high - 1 downto d'low + 4 loop
            hi4_any := hi4_any or d(i);
        end loop;
        if d(d'high) = '1' then                                   -- d < 0: -7 .. -1 -> 2, <= -8 -> 3
            if hi_all = '1' and lo_any = '1' then n := "10"; else n := "11"; end if;
        elsif hi4_any = '0' and d(d'low + 3) = '0' then           -- 0 -> 2, 1 .. 7 -> 1
            if lo_any = '1' then n := "01"; else n := "10"; end if;
        elsif hi4_any = '0' and lo_any = '0' then                 -- 8 -> 1
            n := "01";
        else
            n := "00";
        end if;
        return n;
    end function;
    function f_mn(a, b : unsigned(1 downto 0)) return unsigned is
    begin
        if a < b then return a; else return b; end if;
    end function;
    function f_mx(a, b : unsigned(1 downto 0)) return unsigned is
    begin
        if a > b then return a; else return b; end if;
    end function;
    -- d * f (f = 0..7), 12-bit signed
    function f_frac(d : signed(8 downto 0); f : unsigned(2 downto 0)) return signed is
        variable r : signed(11 downto 0);
    begin
        r := (others => '0');
        if f(2) = '1' then r := r + shift_left(resize(d, 12), 2); end if;
        if f(1) = '1' then r := r + shift_left(resize(d, 12), 1); end if;
        if f(0) = '1' then r := r + resize(d, 12); end if;
        return r;
    end function;
    -- clip a signed value to -128 .. 127 and return it offset-binary (z + 128), from the bits alone
    function f_clip8(z : signed(12 downto 0)) return unsigned is
        variable allz, allo : std_logic;
    begin
        allz := '1'; allo := '1';
        for i in 11 downto 7 loop
            allz := allz and not z(i); allo := allo and z(i);
        end loop;
        if z(12) = '0' then
            if allz = '1' then return unsigned(z(7 downto 0)) xor x"80"; else return x"FF"; end if;
        else
            if allo = '1' then return unsigned(z(7 downto 0)) xor x"80"; else return x"00"; end if;
        end if;
    end function;
    function f_abs13(v : signed(12 downto 0)) return unsigned is
    begin
        if v(12) = '1' then return unsigned(-v); else return unsigned(v); end if;
    end function;

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q  : unsigned(15 downto 0) := (others => '0');
    signal r_cos_a : unsigned(8 downto 0) := (others => '0');
    signal r_cos_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_row_a : unsigned(6 downto 0) := (others => '0');
    signal r_row_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_col_a : unsigned(6 downto 0) := (others => '0');
    signal r_col_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_pil_a : unsigned(8 downto 0) := (others => '0');
    signal r_pil_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_sin_a : unsigned(7 downto 0) := (others => '0');
    signal r_sin_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_tr_a, r_tg_a, r_tb_a : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal r_tr_q, r_tg_q, r_tb_q : std_logic_vector(31 downto 0) := (others => '0');

    -- output box delay ring (8 samples back; collision-free)
    type t_dl is array (0 to 255) of std_logic_vector(31 downto 0);
    signal dl_ram : t_dl := (others => x"00000000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of dl_ram : signal is true;
    signal dl_wa, dl_ra : unsigned(7 downto 0) := (others => '0');
    signal dl_q, dl_qq : std_logic_vector(31 downto 0) := (others => '0');

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

    -- knobs: glided (Q3), raw flow, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(104 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(687, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_twin, s_hue, s_key, s_neg, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_r1, z_r3 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_h1, z_h3 : signed(3 downto 0) := (others => '0');
    signal z_dxb  : unsigned(11 downto 0) := (others => '0');
    signal z_dxn  : std_logic := '0';
    signal c_ikw  : unsigned(9 downto 0) := to_unsigned(256, 10);             -- 256 / wing span
    signal c_qh   : unsigned(17 downto 0) := to_unsigned(17119, 18);          -- pillar phase per h, Q24 / period
    signal c_qp   : unsigned(19 downto 0) := to_unsigned(34238, 20);          -- per pixel
    signal c_q0   : unsigned(23 downto 0) := (others => '0');                 -- at the line's first pixel
    signal c_vk   : unsigned(17 downto 0) := to_unsigned(45159, 18);          -- 2^24 / P
    signal c_vinc : unsigned(23 downto 0) := to_unsigned(45159, 24);          -- per output line
    signal c_u0   : unsigned(23 downto 0) := (others => '0');
    signal c_tau  : unsigned(23 downto 0) := to_unsigned(4696869, 24);        -- the colour flow phase (power-up: the still, 2.5 s into the cycle)
    signal c_tst  : unsigned(19 downto 0) := (others => '0');
    signal c_eg   : unsigned(6 downto 0) := (others => '0');                  -- etch px per luma step, Q8
    signal c_ga   : unsigned(4 downto 0) := to_unsigned(8, 5);                -- grain depth, Q3
    signal c_kt0, c_kb0 : signed(10 downto 0) := (others => '0');             -- arm edges at the side (207 / 119 + offset)
    signal c_xb0  : unsigned(11 downto 0) := (others => '0');                 -- beam start (h)
    signal c_w1p : unsigned(15 downto 0) := (others => '0');

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(17 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(17 downto 0) := (others => '0');
    signal z_fb  : unsigned(17 downto 0) := (others => '0');
    signal z_sel : unsigned(2 downto 0) := (others => '0');
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
    signal e_lc   : unsigned(8 downto 0) := to_unsigned(511, 9);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(19 downto 0) := (others => '0');
    signal e_yi   : unsigned(10 downto 0) := (others => '0');
    signal e_y    : unsigned(9 downto 0) := (others => '0');
    signal e_u    : unsigned(23 downto 0) := (others => '0');
    signal e_wrb, e_wg : unsigned(11 downto 0) := (others => '0');
    signal e_th   : unsigned(23 downto 0) := (others => '0');
    signal e_c0, e_c1, e_c2 : signed(7 downto 0) := (others => '0');
    signal e_dn, e_sgn, e_whi : std_logic := '0';
    signal e_dab  : unsigned(9 downto 0) := (others => '0');
    signal e_kk   : unsigned(8 downto 0) := to_unsigned(125, 9);   -- the operand unit's constant, decoded a clock ahead
    signal e_wsel : std_logic := '0';
    signal e_rsel : unsigned(1 downto 0) := "00";
    signal e_ka   : unsigned(23 downto 0) := (others => '0');      -- a_c - tau, ready a clock ahead
    signal o_ld, o_en, o_sub, o_hi : std_logic := '0';             -- theta operand controls, decoded a clock ahead
    signal e_op   : unsigned(23 downto 0) := (others => '0');      -- theta operand (a_c - tau, or -+ the product)
    signal e_upd, e_keep, e_cin : std_logic := '0';
    signal m_ld   : std_logic := '0';                              -- multiplier load, decoded a clock ahead
    signal m_sel  : unsigned(3 downto 0) := (others => '0');
    signal e_wop  : unsigned(11 downto 0) := (others => '0');
    signal e_drd  : unsigned(8 downto 0) := (others => '0');
    signal e_wp   : unsigned(15 downto 0) := (others => '0');
    signal e_ri   : unsigned(6 downto 0) := (others => '0');
    signal e_rA, e_rB : std_logic_vector(23 downto 0) := (others => '0');
    signal e_rn   : std_logic := '0';
    signal e_r0, e_r1, e_r2 : signed(7 downto 0) := (others => '0');
    signal e_oc, e_ic, e_sc : unsigned(9 downto 0) := (others => '0');
    signal m_a    : unsigned(27 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(27 downto 0) := (others => '0');
    -- next-line results
    signal n_vr0, n_vr1, n_vr2 : signed(8 downto 0) := (others => '0');  -- vertical wave + row table, Q6
    signal n_y    : unsigned(9 downto 0) := (others => '0');
    -- unscaled wing geometry (px): leaf / shadow edges (axis +- half-width), the shadow axis
    signal n_oa, n_ob, n_ia, n_ib, n_sa, n_sb, n_sc : unsigned(9 downto 0) := (others => '0');
    signal n_pwn  : unsigned(6 downto 0) := (others => '0');
    signal n_wy   : unsigned(2 downto 0) := (others => '0');
    signal n_acy  : unsigned(1 downto 0) := (others => '0');

    -- beam / per-line values in use
    signal b_x   : unsigned(11 downto 0) := (others => '0');
    signal b_q   : unsigned(23 downto 0) := (others => '0');
    signal b_w1  : unsigned(15 downto 0) := (others => '0');
    signal b_lm  : unsigned(7 downto 0) := (others => '0');
    signal l_vr0, l_vr1, l_vr2 : signed(8 downto 0) := (others => '0');
    signal l_y   : unsigned(9 downto 0) := (others => '0');
    signal l_kt, l_kbb : signed(11 downto 0) := (others => '0');            -- Y - 207 - offset, Y - 119 - offset
    signal l_oa, l_ob, l_ia, l_ib, l_sa, l_sb, l_sc : unsigned(9 downto 0) := (others => '0');
    signal l_pwn : unsigned(6 downto 0) := (others => '0');
    signal l_wy  : unsigned(2 downto 0) := (others => '0');
    signal l_acy : unsigned(1 downto 0) := (others => '0');
    -- oscillator source (Source = Osc): a drifting 2-D triangle lattice, 240 x 180 virtual px
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(9 downto 0) := to_unsigned(136, 10);
    signal s_oys : unsigned(11 downto 0) := to_unsigned(364, 12);

    -- pixel path (pK_ = stage K)
    signal p2_xa, p2_xb, p2_xe : unsigned(10 downto 0) := (others => '0');   -- Twin fold, |x - 1075|, edge distance
    signal p2_t1 : unsigned(14 downto 0) := (others => '0');
    signal p2_et : unsigned(14 downto 0) := (others => '0');
    signal p3_x, p3_xe : unsigned(10 downto 0) := (others => '0');
    signal p3_wob : unsigned(4 downto 0) := (others => '0');
    signal p4_x, p4_xe : unsigned(10 downto 0) := (others => '0');
    signal p4_axt, p4_axb : unsigned(8 downto 0) := (others => '0');
    signal p4_x4 : unsigned(8 downto 0) := (others => '0');
    signal p4_uh, p4_ul : unsigned(15 downto 0) := (others => '0');
    signal p4_sn : unsigned(8 downto 0) := (others => '0');
    signal p4_sag : unsigned(6 downto 0) := (others => '0');
    signal p4_wob : unsigned(4 downto 0) := (others => '0');
    signal p5_xu : unsigned(11 downto 0) := (others => '0');
    signal p5_dt, p5_db : signed(12 downto 0) := (others => '0');
    signal p5_denv : signed(11 downto 0) := (others => '0');
    signal p5_dcap, p5_deye : signed(11 downto 0) := (others => '0');
    signal p5_s  : unsigned(10 downto 0) := (others => '0');
    signal p5_dp : signed(9 downto 0) := (others => '0');
    signal p5_wob : unsigned(5 downto 0) := (others => '0');
    signal p5_x  : unsigned(10 downto 0) := (others => '0');
    -- stage 6: keys
    signal p6_ao, p6_ai, p6_as, p6_aso, p6_atp, p6_at40, p6_asc, p6_ax40, p6_envr : unsigned(1 downto 0) := "00";
    signal p6_adb, p6_anb, p6_adt, p6_ant, p6_acap, p6_aeye, p6_aenv, p6_ap, p6_aae, p6_adl : unsigned(1 downto 0) := "00";
    signal p6_x7, p7_x7 : unsigned(6 downto 0) := (others => '0');
    -- stage 7
    signal p7_ao, p7_ai, p7_as, p7_arm, p7_lobe, p7_above, p7_ae : unsigned(1 downto 0) := "00";
    signal p7_cap, p7_eye, p7_aenv, p7_ap, p7_asc, p7_ax40, p7_envr : unsigned(1 downto 0) := "00";
    -- stage 8
    signal p8_ao, p8_ai, p8_pil, p8_lobe, p8_above, p8_ae : unsigned(1 downto 0) := "00";
    signal p8_cap, p8_eye, p8_asc, p8_envr, p8_big4, p8_big : unsigned(1 downto 0) := "00";
    signal p8_sin : signed(7 downto 0) := (others => '0');
    -- stage 9 .. 17
    signal p9_p0, p9_p1, p9_p2, p9_p3 : std_logic_vector(26 downto 0) := (others => '0');
    signal p9_body, p9_gate : unsigned(1 downto 0) := "00";
    signal p9_sin : signed(7 downto 0) := (others => '0');
    signal p10_cr, p10_cg, p10_cb : signed(7 downto 0) := (others => '0');
    signal p10_p4 : std_logic_vector(26 downto 0) := (others => '0');
    signal p10_ar, p10_ag, p10_ab, p10_br, p10_bg, p10_bb : signed(9 downto 0) := (others => '0');
    signal p10_rg : signed(9 downto 0) := (others => '0');
    signal p11_lr, p11_lg, p11_lb : signed(11 downto 0) := (others => '0');
    signal p11_rp : signed(15 downto 0) := (others => '0');
    signal p12_lr, p12_lg, p12_lb : signed(11 downto 0) := (others => '0');
    signal p15_tr, p15_tg, p15_tb : std_logic_vector(29 downto 0) := (others => '0');
    signal p16_y  : unsigned(11 downto 0) := (others => '0');
    signal p16_u, p16_v : signed(12 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs, the box delay ring
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q  <= C_MT_EXP2(to_integer(r_ex_a));
            r_cos_q <= C_MT_COS(to_integer(r_cos_a));
            r_row_q <= C_MT_ROW(to_integer(r_row_a));
            r_col_q <= C_MT_COL(to_integer(r_col_a));
            r_pil_q <= C_MT_PIL(to_integer(r_pil_a));
            r_sin_q <= C_MT_SIN(to_integer(r_sin_a));
            r_tr_q  <= C_MT_TONE_R(to_integer(r_tr_a));
            r_tg_q  <= C_MT_TONE_G(to_integer(r_tg_a));
            r_tb_q  <= C_MT_TONE_B(to_integer(r_tb_a));
        end if;
    end process p_rom;

    -- writes stage-17 samples every clock; the read address runs 5 behind, so dl_qq holds the sample 8 back
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            dl_ram(to_integer(dl_wa)) <= "00" & std_logic_vector(s_y17) & std_logic_vector(s_u17) & std_logic_vector(s_v17);
            dl_wa <= dl_wa + 1;
            dl_ra <= dl_wa - 5;
            dl_q  <= dl_ram(to_integer(dl_ra));
            dl_qq <= dl_q;
        end if;
    end process p_dl;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_k  : signed(15 downto 0);
        variable v_u  : unsigned(15 downto 0);
        variable v_t  : unsigned(23 downto 0);
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
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
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
                s_twin <= registers_in(6)(0);
                s_hue  <= registers_in(6)(1);
                s_key  <= registers_in(6)(2);
                s_neg  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(546, 10);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(410, 10);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(273, 10);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            case to_integer(unsigned(s_tid)) is      -- Osc lattice step per line = ys 364 / 256
                when 0          => s_oys <= to_unsigned(1618, 12);
                when 4          => s_oys <= to_unsigned(819, 12);
                when 8          => s_oys <= to_unsigned(1365, 12);
                when 12         => s_oys <= to_unsigned(682, 12);
                when 5 | 6 | 14 => s_oys <= to_unsigned(546, 12);
                when 1 | 2 | 10 => s_oys <= to_unsigned(728, 12);
                when others     => s_oys <= to_unsigned(364, 12);
            end case;

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
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
                            when 2      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 24 k, 18 iterations, result stable from load + 19
            -- (stores at 52 + 24 k).  Slot number decoded at 30 + 24 k, operands pre-muxed at 31 + 24 k.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 3); z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 3); z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 3); z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 3); z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 3); z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 3); z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 3); z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 3); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0 => z_fa <= to_unsigned(17119, 18); z_fb <= resize(z_r1, 18);   -- pillar step / h
                    when 1 => z_fa <= resize(c_qh, 18); z_fb <= resize(s_xs, 18);         -- per pixel
                    when 2 => z_fa <= resize(c_qh, 18); z_fb <= resize(z_dxb, 18);        -- start phase
                    when 3 => z_fa <= to_unsigned(45159, 18); z_fb <= resize(z_r3, 18);   -- 2^24 / P
                    when 4 => z_fa <= resize(c_vk, 18); z_fb <= resize(s_ys, 18);         -- per line
                    when 5 => z_fa <= resize(c_vk, 18); z_fb <= to_unsigned(150, 18);     -- u at line 0
                    when 6 => z_fa <= resize(s_vm, 18); z_fb <= resize(s_vm, 18);         -- flow^2
                    when others => z_fa <= resize(s_g6(12 downto 3), 18); z_fb <= to_unsigned(97, 18);  -- etch
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 36); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(17 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- flow (last field's step), beam origin; wing exponent
                    when 20 =>
                        if s_vneg = '1' then c_tau <= c_tau - resize(c_tst, 24); else c_tau <= c_tau + resize(c_tst, 24); end if;
                        c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
                        -- Wings: ikw = 2^(-e / 256), -e = (512 - p12) 3 / 4 = (3 (1023 - p12) - 1533) / 4
                        -- (products of unsigned operands only: no sign-extension self-adds for router1)
                        v_u := resize(not s_gp(12 downto 3), 16);
                        v_u := shift_left(v_u, 1) + v_u;
                        v_k := shift_right(signed(v_u) - 1533, 2);
                        v_e := v_k(11 downto 0);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_e <= v_e;
                    when 21 =>
                        if c_xb0 >= 240 then z_dxb <= c_xb0 - 240; z_dxn <= '0';
                        else z_dxb <= 240 - c_xb0; z_dxn <= '1'; end if;
                    when 22 =>                       -- ikw = 256 2^(-e / 256)
                        case to_integer(z_e(11 downto 8)) is
                            when 1      => c_ikw <= resize(shift_right(r_ex_q, 6), 10);
                            when 0      => c_ikw <= resize(shift_right(r_ex_q, 7), 10);
                            when -1     => c_ikw <= resize(shift_right(r_ex_q, 8), 10);
                            when others => c_ikw <= resize(shift_right(r_ex_q, 9), 10);
                        end case;
                        v_e := 512 - signed(resize(s_g1(12 downto 3), 12));          -- Pillars: -(k1 - 512), max 511
                        if v_e > 511 then v_e := to_signed(511, 12); end if;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_h1 <= v_e(11 downto 8);
                    when 24 =>
                        z_r1 <= r_ex_q;
                        v_u := resize(s_g3(12 downto 3), 16);                        -- Bands: (25 k3 - 12800) / 32
                        v_u := shift_left(v_u, 4) + shift_left(v_u, 3) + v_u;
                        v_k := shift_right(signed(v_u) - 12800, 5);
                        v_e := v_k(11 downto 0);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_h3 <= v_e(11 downto 8);
                    when 26 =>
                        z_r3 <= r_ex_q;
                        -- grain depth (Q3): k5 / 64 below the centre, 1 + 3 (k5 - 512) / 512 above
                        if s_g5(12) = '0' then c_ga <= resize(s_g5(12 downto 9), 5);
                        else c_ga <= to_unsigned(8, 5) + resize(shift_right(resize(s_g5(11 downto 3), 11) + shift_left(resize(s_g5(11 downto 3), 11), 1), 6), 5); end if;
                        -- arm edges: 207 / 119 + (k2 - 512) 25 / 64 px
                        v_u := resize(s_g2(12 downto 3), 16);
                        v_u := shift_left(v_u, 4) + shift_left(v_u, 3) + v_u;
                        v_k := shift_right(signed(v_u) - 12800, 6);
                        c_kt0 <= resize(v_k, 11) + 207;
                        c_kb0 <= resize(v_k, 11) + 119;
                        -- wobble phase steps (per pixel) and starts
                        c_w1p <= resize(shift_left(resize(s_xs, 16), 8) - shift_left(resize(s_xs, 16), 5) - shift_left(resize(s_xs, 16), 3) + shift_left(resize(s_xs, 16), 1), 16);   -- 218 xs
                    when 52 =>                       -- qh = 17119 r1 >> (15 - sh)
                        case to_integer(z_h1) is
                            when 1      => c_qh <= f_acc(31 downto 14);
                            when 0      => c_qh <= f_acc(32 downto 15);
                            when -1     => c_qh <= f_acc(33 downto 16);
                            when others => c_qh <= f_acc(34 downto 17);
                        end case;
                    when 76 => c_qp <= f_acc(19 downto 0);
                    when 100 =>
                        v_t := f_acc(23 downto 0);
                        if z_dxn = '1' then c_q0 <= (not v_t) + 1; else c_q0 <= v_t; end if;
                    when 124 =>                      -- vk = 45159 r3 >> (15 - sh)
                        case to_integer(z_h3) is
                            when 1      => c_vk <= f_acc(31 downto 14);
                            when 0      => c_vk <= f_acc(32 downto 15);
                            when -1     => c_vk <= f_acc(33 downto 16);
                            when others => c_vk <= f_acc(34 downto 17);
                        end case;
                    when 148 => c_vinc <= f_acc(31 downto 8);
                    when 172 =>
                        v_t := f_acc(23 downto 0);
                        c_u0 <= (not v_t) + 1;
                    when 196 => c_tst <= resize(f_acc(17 downto 0), 20) + resize(f_acc(17 downto 3), 20);
                    when 220 => c_eg <= f_acc(16 downto 10);
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
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (16 iterations, result at
    -- load + 17): the three vertical waves, the row table, the wing geometry
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y  : signed(11 downto 0);
        variable v_t  : unsigned(23 downto 0);
        variable v_p  : unsigned(9 downto 0);
        variable v_h  : signed(12 downto 0);
        variable v_q  : unsigned(15 downto 0);
        variable v_nx : integer range 0 to 512;
        variable v_ea : unsigned(23 downto 0);
        variable v_eb : unsigned(23 downto 0);
        variable v_ra, v_rb : signed(7 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_u <= c_u0;
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 20);
                e_u <= e_u + c_vinc;
                e_lc <= (others => '0');
            elsif e_lc /= 511 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(26 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            -- controls decoded a clock ahead (from the next state) so no wide decode sits in front of an adder
            v_nx := to_integer(e_lc) + 1;
            case v_nx + 1 is                          -- the operand unit's K, two states ahead
                when 139    => e_kk <= to_unsigned(190, 9);
                when 156    => e_kk <= to_unsigned(10, 9);
                when 173    => e_kk <= to_unsigned(183, 9);
                when 190    => e_kk <= to_unsigned(395, 9);
                when others => e_kk <= to_unsigned(125, 9);
            end case;
            if v_nx = 2 then e_wsel <= '1'; else e_wsel <= '0'; end if;
            case v_nx is
                when 274    => e_rsel <= "00";
                when 291    => e_rsel <= "01";
                when others => e_rsel <= "10";
            end case;
            case v_nx is                              -- a_c - tau for the operand loads at 37 / 73 / 107
                when 37     => e_ka <= to_unsigned(C_A0, 24) - c_tau;
                when 73     => e_ka <= to_unsigned(C_A1, 24) - c_tau;
                when others => e_ka <= to_unsigned(C_A2, 24) - c_tau;
            end case;
            o_ld <= '0'; o_en <= '0'; o_sub <= '0'; o_hi <= '0';
            case v_nx is
                when 37 | 73 | 107 => o_ld <= '1';
                when 54 | 88       => o_en <= '1'; o_sub <= '1'; o_hi <= '1';   -- - m_c y (R, G)
                when 122           => o_en <= '1'; o_hi <= '1';                 -- + m_B y
                when 71 | 105      => o_en <= '1';                              -- + A_c w
                when 139           => o_en <= '1'; o_sub <= '1';                -- - |A_B| w
                when others => null;
            end case;
            m_ld <= '1';
            case v_nx is
                when 3 | 20 => m_sel <= x"0";
                when 37     => m_sel <= x"1";
                when 54     => m_sel <= x"2";
                when 71     => m_sel <= x"3";
                when 88     => m_sel <= x"4";
                when 105    => m_sel <= x"5";
                when 122    => m_sel <= x"6";
                when 139    => m_sel <= x"7";
                when 156    => m_sel <= x"8";
                when 173    => m_sel <= x"9";
                when 190 | 224 => m_sel <= x"A";
                when 207    => m_sel <= x"B";
                when 241    => m_sel <= x"C";
                when 258    => m_sel <= x"D";
                when 275 | 292 | 309 => m_sel <= x"E";
                when others => m_ld <= '0';
            end case;
            -- shared operand unit: |Y - K|
            v_y := signed(resize(e_y, 12)) - signed(resize(e_kk, 12));
            if v_y(11) = '1' then v_y := -v_y; e_dn <= '1'; else e_dn <= '0'; end if;
            e_dab <= unsigned(v_y(9 downto 0));
            -- shared skewed-triangle unit: w(u) operand for R / B (state 2) or G (u + 1/2, otherwise)
            if e_wsel = '1' then v_t := e_u; else v_t := (not e_u(23)) & e_u(22 downto 0); end if;
            if v_t < 13589545 then e_wop <= v_t(23 downto 12); e_whi <= '0';
            else v_t := (not v_t) + 1; e_wop <= v_t(23 downto 12); e_whi <= '1'; end if;
            -- shared row-difference unit: |rB - rA| for channel R, G, B
            case e_rsel is
                when "00"   => v_ra := signed(e_rA(23 downto 16)); v_rb := signed(e_rB(23 downto 16));
                when "01"   => v_ra := signed(e_rA(15 downto 8));  v_rb := signed(e_rB(15 downto 8));
                when others => v_ra := signed(e_rA(7 downto 0));   v_rb := signed(e_rB(7 downto 0));
            end case;
            v_h := resize(v_rb, 13) - resize(v_ra, 13);
            if v_h(12) = '1' then v_h := -v_h; e_rn <= '1'; else e_rn <= '0'; end if;
            e_drd <= unsigned(v_h(8 downto 0));
            -- theta accumulator: e_th = a_c - tau, then -+ m_c y (Q4 lines), then + A_c w.  The operand is captured
            -- in the state the product is ready (e_op, complemented for a subtraction), the add happens a clock later.
            if o_hi = '1' then v_eb := m_acc(27 downto 4); else v_eb := m_acc(23 downto 0); end if;
            if o_ld = '1' then e_op <= e_ka;
            elsif o_sub = '1' then e_op <= not v_eb;
            else e_op <= v_eb; end if;
            e_upd <= o_ld or o_en; e_keep <= not o_ld; e_cin <= o_sub;
            if e_upd = '1' then
                if e_keep = '1' then
                    e_th <= e_th + e_op + unsigned'("" & e_cin);
                else
                    e_th <= e_op;
                end if;
            end if;
            -- multiplier loads
            if m_ld = '1' then
                m_acc <= (others => '0');
                case m_sel is
                    when x"0" => m_a <= resize(e_wop, 28);
                                 if e_whi = '1' then m_b <= to_unsigned(21558, 16); else m_b <= to_unsigned(5057, 16); end if;
                    when x"1" => m_a <= resize(e_vy(18 downto 4), 28); m_b <= to_unsigned(C_RMP0, 16);
                    when x"2" => m_a <= resize(e_wrb, 28); m_b <= to_unsigned(C_AR, 16);
                    when x"3" => m_a <= resize(e_vy(18 downto 4), 28); m_b <= to_unsigned(C_RMP1, 16);
                    when x"4" => m_a <= resize(e_wg, 28); m_b <= to_unsigned(C_AG, 16);
                    when x"5" => m_a <= resize(e_vy(18 downto 4), 28); m_b <= to_unsigned(C_RMP2, 16);
                    when x"6" => m_a <= resize(e_wrb, 28); m_b <= to_unsigned(C_AB, 16);
                    when x"7" => m_a <= resize(e_dab, 28); m_b <= to_unsigned(2519, 16); e_sgn <= e_dn;
                    when x"8" => m_a <= resize(e_dab, 28); m_b <= to_unsigned(3191, 16); e_sgn <= e_dn;
                    when x"9" => m_a <= resize(e_dab, 28); m_b <= to_unsigned(3301, 16); e_sgn <= e_dn;
                    when x"A" => m_a <= resize(e_dab, 28); m_b <= resize(e_dab, 16);
                    when x"B" => m_a <= resize(m_acc(17 downto 0), 28); m_b <= to_unsigned(106, 16);
                    when x"C" => m_a <= resize(m_acc(17 downto 0), 28); m_b <= to_unsigned(307, 16);
                    when x"D" => if e_y > 183 then m_a <= resize(e_y - 183, 28); else m_a <= (others => '0'); end if;
                                 m_b <= to_unsigned(10760, 16);
                    when others => m_a <= resize(e_drd, 28); m_b <= resize(e_y(2 downto 0), 16); e_sgn <= e_rn;
                end case;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    e_yi <= e_vy(18 downto 8);
                when 2 =>                     -- Y = |yi - 518| (negated at full width, then sliced)
                    v_y := signed(resize(e_yi, 12)) - 518;
                    if v_y(11) = '1' then v_y := -v_y; end if;
                    e_y <= unsigned(v_y(9 downto 0));
                when 3 =>
                    if e_y(9 downto 3) > 68 then e_ri <= to_unsigned(69, 7); else e_ri <= e_y(9 downto 3); end if;
                    v_h := signed(resize(e_y, 13)) - 430;                    -- cap: key of (Y - 430)
                    n_acy <= f_al(v_h);
                when 5 =>
                    n_y <= e_y;
                    -- wood-grain wobble phase Y * 728 (Q16 per 90 lines)
                    e_wp <= resize(shift_left(resize(e_y, 16), 9) + shift_left(resize(e_y, 16), 7) + shift_left(resize(e_y, 16), 6), 16);
                when 6 =>
                    e_wp <= e_wp + resize(shift_left(resize(e_y, 16), 4) + shift_left(resize(e_y, 16), 3), 16);
                when 7 =>
                    v_q := resize(f_tri15(e_wp), 16);
                    n_wy <= v_q(14 downto 12);
                -- row table: entries i and i + 1 (the interpolation runs on the multiplier, states 275 .. 326)
                when 9 =>
                    r_row_a <= e_ri;
                when 10 =>
                    if e_ri = 69 then r_row_a <= e_ri; else r_row_a <= e_ri + 1; end if;
                when 11 =>
                    e_rA <= r_row_q(31 downto 8);
                    n_pwn <= unsigned(r_row_q(6 downto 0));   -- pillar half-width (1/1024 period) of entry i
                when 12 =>
                    e_rB <= r_row_q(31 downto 8);
                -- the vertical waves: theta_c = a_c - tau + m_c yi + A_c w (e_th, above)
                when 20 =>
                    e_wrb <= m_acc(23 downto 12);
                when 37 =>
                    e_wg <= m_acc(23 downto 12);
                when 73 => r_cos_a <= e_th(23 downto 15);
                when 75 => e_c0 <= signed(r_cos_q);
                when 107 => r_cos_a <= e_th(23 downto 15);
                when 109 => e_c1 <= signed(r_cos_q);
                when 141 => r_cos_a <= e_th(23 downto 15);
                when 143 => e_c2 <= signed(r_cos_q);
                when 156 =>
                    v_p := m_acc(21 downto 12);
                    if e_sgn = '1' then e_oc <= 260 - v_p; else e_oc <= 260 + v_p; end if;
                when 173 =>
                    v_p := m_acc(21 downto 12);
                    if e_sgn = '1' then e_ic <= 88 - v_p; else e_ic <= 88 + v_p; end if;
                when 190 =>                   -- (Y - 395)^2
                    v_p := m_acc(21 downto 12);
                    if e_sgn = '1' then e_sc <= 375 - v_p; else e_sc <= 375 + v_p; end if;
                when 224 =>                   -- outer leaf edges: axis +- max(68 - t, 0); then (Y - 125)^2
                    if m_acc(27 downto 16) >= 68 then v_p := (others => '0'); else v_p := resize(68 - m_acc(22 downto 16), 10); end if;
                    n_oa <= e_oc + v_p; n_ob <= e_oc - v_p;
                when 258 =>                   -- inner leaf edges: axis +- max(62 - t, 0)
                    if m_acc(27 downto 16) >= 62 then v_p := (others => '0'); else v_p := resize(62 - m_acc(22 downto 16), 10); end if;
                    n_ia <= e_ic + v_p; n_ib <= e_ic - v_p;
                when 275 =>                   -- shadow edges: axis +- (Y - 183)+ 0.164; row interpolation, R
                    n_sa <= e_sc + resize(m_acc(22 downto 16), 10); n_sb <= e_sc - resize(m_acc(22 downto 16), 10);
                    n_sc <= e_sc;
                when 292 =>
                    if e_sgn = '1' then e_r0 <= signed(e_rA(23 downto 16)) - signed(resize(m_acc(10 downto 3), 8));
                    else e_r0 <= signed(e_rA(23 downto 16)) + signed(resize(m_acc(10 downto 3), 8)); end if;
                when 309 =>
                    if e_sgn = '1' then e_r1 <= signed(e_rA(15 downto 8)) - signed(resize(m_acc(10 downto 3), 8));
                    else e_r1 <= signed(e_rA(15 downto 8)) + signed(resize(m_acc(10 downto 3), 8)); end if;
                when 326 =>
                    if e_sgn = '1' then e_r2 <= signed(e_rA(7 downto 0)) - signed(resize(m_acc(10 downto 3), 8));
                    else e_r2 <= signed(e_rA(7 downto 0)) + signed(resize(m_acc(10 downto 3), 8)); end if;
                when 327 =>                   -- vertical wave + row table
                    n_vr0 <= resize(e_c0, 9) + resize(e_r0, 9);
                    n_vr1 <= resize(e_c1, 9) + resize(e_r1, 9);
                    n_vr2 <= resize(e_c2, 9) + resize(e_r2, 9);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: x (h), the pillar phase, the wobble phases, the etch luma; per-line
    -- values; the Osc lattice
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= c_xb0;
                b_q  <= c_q0;
                b_w1 <= (others => '0');
                l_vr0 <= n_vr0; l_vr1 <= n_vr1; l_vr2 <= n_vr2;
                l_y <= n_y;
                l_kt  <= signed(resize(n_y, 12)) - resize(c_kt0, 12);
                l_kbb <= signed(resize(n_y, 12)) - resize(c_kb0, 12);
                l_oa <= n_oa; l_ob <= n_ob; l_ia <= n_ia; l_ib <= n_ib; l_sa <= n_sa; l_sb <= n_sb; l_sc <= n_sc;
                l_pwn <= n_pwn; l_wy <= n_wy; l_acy <= n_acy;
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            elsif g_avid = '1' then
                b_x  <= b_x + s_xs;
                b_q  <= b_q + resize(c_qp, 24);
                b_w1 <= b_w1 + c_w1p;
                s_ox <= s_ox + s_oxs;
            end if;
            -- luma for the etch (Video: Y >> 2 - 16; Osc: the lattice), stage 1
            if s_src = '1' then
                b_lm <= resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            elsif unsigned(data_in.y(9 downto 2)) > 16 then
                b_lm <= unsigned(data_in.y(9 downto 2)) - 16;
            else
                b_lm <= (others => '0');
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = the beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_xv : unsigned(10 downto 0);
        variable v_d, v_dn : signed(11 downto 0);
        variable v_x  : unsigned(10 downto 0);
        variable v_e12 : unsigned(11 downto 0);
        variable v_u  : unsigned(20 downto 0);
        variable v_a  : signed(12 downto 0);
        variable v_s  : signed(12 downto 0);
        variable v_sr : unsigned(10 downto 0);
        variable v_sb : unsigned(14 downto 0);
        variable v_b4 : unsigned(1 downto 0);
        variable v_rg : signed(9 downto 0);
        variable v_rr : signed(15 downto 0);
        variable v_i  : unsigned(3 downto 0);
        variable v_zr, v_zg, v_zb : signed(12 downto 0);
        variable v_ar, v_ag, v_ab : unsigned(7 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_c  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: x, the Twin fold (|x - 480| left of 960, |x - 1440| right of it), |x - 1075|, the nearer screen edge;
            --    pillar ROM address; wobble triangles; etch product
            v_xv := b_x(11 downto 1);
            if v_xv < 960 then v_d := signed(resize(v_xv, 12)) - 480; else v_d := signed(resize(v_xv, 12)) - 1440; end if;
            v_dn := -v_d;
            if v_d(11) = '1' then p2_xa <= unsigned(v_dn(10 downto 0)); else p2_xa <= unsigned(v_d(10 downto 0)); end if;
            v_d := signed(resize(v_xv, 12)) - 1075;
            v_dn := -v_d;
            if v_d(11) = '1' then p2_xb <= unsigned(v_dn(10 downto 0)); else p2_xb <= unsigned(v_d(10 downto 0)); end if;
            if v_xv < 960 then p2_xe <= v_xv;
            elsif v_xv > 1919 then p2_xe <= (others => '0');
            else p2_xe <= 1919 - v_xv; end if;
            r_pil_a <= b_q(23 downto 15);
            p2_t1 <= f_tri15(b_w1);
            p2_et <= b_lm * c_eg;
            -- 3: X pushed by the etch (px Q8; Osc x16), saturating at 2047; wobble = 15 tri(x / 150)
            if s_twin = '1' then v_x := p2_xa; else v_x := p2_xb; end if;
            if s_src = '1' then v_e12 := resize(p2_et(14 downto 4), 12); else v_e12 := resize(p2_et(14 downto 8), 12); end if;
            v_e12 := resize(v_x, 12) + v_e12;
            if v_e12(11) = '1' then p3_x <= (others => '1'); else p3_x <= v_e12(10 downto 0); end if;
            p3_xe <= p2_xe;
            p3_wob <= resize(shift_right(shift_left(resize(p2_t1, 19), 4) - resize(p2_t1, 19), 15), 5);
            -- 4: X products; X / kw partials; pillar ROM fabric copy; body grain partials
            v_x := p3_x;
            p4_x <= v_x; p4_xe <= p3_xe;
            p4_axt <= resize(shift_right(v_x, 3) + shift_right(v_x, 5), 9);
            p4_axb <= resize(shift_right(v_x, 3) + shift_right(v_x, 5) + shift_right(v_x, 6), 9);
            p4_x4  <= resize(shift_right(v_x, 2), 9);
            p4_uh <= v_x * c_ikw(9 downto 5);
            p4_ul <= v_x * c_ikw(4 downto 0);
            p4_sn  <= unsigned(r_pil_q(15 downto 7));
            p4_sag <= unsigned(r_pil_q(6 downto 0));
            p4_wob <= p3_wob;
            -- 5: X / kw; inside-distances that need no wing scaling
            v_u := shift_left(resize(p4_uh, 21), 5) + resize(p4_ul, 21);
            if v_u(20) = '1' then p5_xu <= (others => '1'); else p5_xu <= v_u(19 downto 8); end if;
            p5_dt <= resize(l_kt, 13) - signed(resize(p4_axt, 13));        -- Y - yt
            p5_db <= resize(l_kbb, 13) - signed(resize(p4_axb, 13));       -- Y - yb
            p5_denv <= signed(resize(l_y, 12)) - signed(resize(p4_x4, 12)) + 50;
            p5_dcap <= 260 - signed(resize(p4_x, 12));
            p5_deye <= 130 - signed(resize(p4_xe, 12)) - signed(resize(l_y, 12));
            p5_s <= resize(l_y, 11) + resize(p4_sag, 11);
            p5_dp  <= signed(resize(l_pwn, 10)) - signed(resize(p4_sn, 10));
            p5_wob <= resize(p4_wob, 6) + resize(l_wy, 6);
            p5_x <= p4_x;
            -- 6: keys: leaves and shadow (both edges, in X / kw), the stage-5 distances; grain sine address
            p6_ao <= f_mn(f_al(signed(resize(l_oa, 13)) - signed(resize(p5_xu, 13))), f_al(signed(resize(p5_xu, 13)) - signed(resize(l_ob, 13))));
            p6_ai <= f_mn(f_al(signed(resize(l_ia, 13)) - signed(resize(p5_xu, 13))), f_al(signed(resize(p5_xu, 13)) - signed(resize(l_ib, 13))));
            p6_as <= f_mn(f_al(signed(resize(l_sa, 13)) - signed(resize(p5_xu, 13))), f_al(signed(resize(p5_xu, 13)) - signed(resize(l_sb, 13))));
            p6_aso  <= f_al(signed(resize(p5_xu, 13)) - signed(resize(l_sa, 13)));
            p6_atp  <= f_al(signed(resize(p5_xu, 13)) - 375);
            p6_at40 <= f_al(signed(resize(p5_xu, 13)) - 335);
            v_a := signed(resize(l_sc, 13)) - signed(resize(p5_xu, 13));          -- sc - X / kw
            p6_asc  <= f_al(v_a);
            p6_ax40 <= f_al(signed(resize(p5_xu, 13)) - signed(resize(l_sc, 13)) + 40);
            p6_adb <= f_al(p5_db);
            p6_anb <= f_aln(p5_db);
            p6_adt <= f_al(p5_dt);
            p6_ant <= f_aln(p5_dt);
            p6_acap <= f_mn(f_al(p5_dcap), l_acy);
            p6_aeye <= f_al(p5_deye);
            p6_aenv <= f_al(p5_denv);
            p6_ap  <= f_al(p5_dp);
            p6_aae <= f_mn(f_al(16 - p5_dt), f_al(p5_dt + 4));                       -- 10 - |dt - 6|
            v_s := signed(resize(p5_s, 13));
            p6_adl <= f_al(v_s - 231);                                                -- S - 231
            if signed(resize(p5_xu, 13)) > signed(resize(l_sc, 13)) then              -- outside the wing's shadow: the scallop grain
                p6_envr <= f_al(231 - v_s);
                v_sr := p5_s + resize(p5_wob, 11);
            else                                                                      -- the body: vertical grain
                p6_envr <= "10";
                v_sr := p5_x + resize(p5_wob, 11);
            end if;
            r_sin_a <= v_sr(4 downto 0) & "000";
            p6_x7 <= p5_x(10 downto 4);
            -- 7: arm, lobe, node, above, arm edge
            p7_ao <= p6_ao; p7_ai <= p6_ai; p7_as <= p6_as;
            p7_arm <= f_mn(f_mn(p6_adb, p6_ant), p6_atp);
            p7_lobe <= f_mn(f_mn(p6_adl, p6_anb), p6_at40);
            p7_above <= f_mn(p6_adt, p6_aso);
            p7_ae <= f_mn(p6_aae, p6_aso);
            p7_cap <= p6_acap; p7_eye <= p6_aeye; p7_aenv <= p6_aenv; p7_ap <= p6_ap; p7_asc <= p6_asc;
            p7_ax40 <= p6_ax40; p7_envr <= p6_envr;
            p7_x7 <= p6_x7;
            -- 8: dark, pillars, the largest keys; column ROM address; sine fabric copy
            p8_ao <= p7_ao; p8_ai <= p7_ai; p8_lobe <= p7_lobe; p8_above <= p7_above; p8_ae <= p7_ae;
            p8_cap <= p7_cap; p8_eye <= p7_eye; p8_asc <= p7_asc; p8_envr <= p7_envr;
            p8_pil <= f_mn(f_mn(p7_ap, p7_aenv), f_mn(3 - p7_arm, p7_ax40));
            v_b4 := f_mx(f_mx(p7_ao, p7_ai), f_mx(p7_as, p7_arm));
            p8_big4 <= v_b4;
            p8_big <= f_mx(v_b4, f_mx(p7_above, p7_lobe));
            p8_sin <= signed(r_sin_q);
            r_col_a <= p7_x7;
            -- 9: body, grain gate; pair weights 0..3 (16-entry ROMs)
            p9_body <= f_mn(3 - p8_big, p8_asc);
            p9_gate <= f_mn(3 - p8_big4, p8_envr);
            v_i := p8_ao & p8_ai;    p9_p0 <= C_MT_P0(to_integer(v_i));
            v_i := p8_above & p8_ae; p9_p1 <= C_MT_P1(to_integer(v_i));
            v_i := p8_cap & p8_eye;  p9_p2 <= C_MT_P2(to_integer(v_i));
            v_i := p8_lobe & p8_pil; p9_p3 <= C_MT_P3(to_integer(v_i));
            p9_sin <= p8_sin;
            -- 10: pair 4; first sums; grain x gate; column table fabric copy
            v_i := p9_body & "00";    p10_p4 <= C_MT_P4(to_integer(v_i));
            p10_ar <= resize(signed(p9_p0(26 downto 18)), 10) + resize(signed(p9_p1(26 downto 18)), 10);
            p10_ag <= resize(signed(p9_p0(17 downto 9)), 10) + resize(signed(p9_p1(17 downto 9)), 10);
            p10_ab <= resize(signed(p9_p0(8 downto 0)), 10) + resize(signed(p9_p1(8 downto 0)), 10);
            p10_br <= resize(signed(p9_p2(26 downto 18)), 10) + resize(signed(p9_p3(26 downto 18)), 10);
            p10_bg <= resize(signed(p9_p2(17 downto 9)), 10) + resize(signed(p9_p3(17 downto 9)), 10);
            p10_bb <= resize(signed(p9_p2(8 downto 0)), 10) + resize(signed(p9_p3(8 downto 0)), 10);
            v_rg := (others => '0');
            if p9_gate(1) = '1' then v_rg := shift_left(resize(p9_sin, 10), 1); end if;
            if p9_gate(0) = '1' then v_rg := v_rg + resize(p9_sin, 10); end if;
            p10_rg <= v_rg;
            p10_cr <= signed(r_col_q(31 downto 24)); p10_cg <= signed(r_col_q(23 downto 16)); p10_cb <= signed(r_col_q(15 downto 8));
            -- 11: shape sums; grain x depth
            p11_lr <= resize(p10_ar, 12) + resize(p10_br, 12) + resize(signed(p10_p4(26 downto 18)), 12) + resize(p10_cr, 12);
            p11_lg <= resize(p10_ag, 12) + resize(p10_bg, 12) + resize(signed(p10_p4(17 downto 9)), 12) + resize(p10_cg, 12);
            p11_lb <= resize(p10_ab, 12) + resize(p10_bb, 12) + resize(signed(p10_p4(8 downto 0)), 12) + resize(p10_cb, 12);
            p11_rp <= p10_rg * signed('0' & c_ga);
            -- 12: L = shape + grain (rr = sin gate depth / 32: R and G rr / 16, B rr / 32)
            v_rr := shift_right(p11_rp, 5);
            p12_lr <= p11_lr + resize(shift_right(v_rr, 4), 12);
            p12_lg <= p11_lg + resize(shift_right(v_rr, 4), 12);
            p12_lb <= p11_lb + resize(shift_right(v_rr, 5), 12);
            -- 13: z = (V + row) +- L; Key (x4); clip to +-128; tone addresses (offset binary); Hue rotation
            if s_neg = '1' then
                v_zr := resize(l_vr0, 13) - resize(p12_lr, 13);
                v_zg := resize(l_vr1, 13) - resize(p12_lg, 13);
                v_zb := resize(l_vr2, 13) - resize(p12_lb, 13);
            else
                v_zr := resize(l_vr0, 13) + resize(p12_lr, 13);
                v_zg := resize(l_vr1, 13) + resize(p12_lg, 13);
                v_zb := resize(l_vr2, 13) + resize(p12_lb, 13);
            end if;
            if s_key = '1' then
                v_zr := shift_left(v_zr, 2); v_zg := shift_left(v_zg, 2); v_zb := shift_left(v_zb, 2);
            end if;
            v_ar := f_clip8(v_zr); v_ag := f_clip8(v_zg); v_ab := f_clip8(v_zb);
            if s_hue = '1' then
                r_tr_a <= v_ag; r_tg_a <= v_ab; r_tb_a <= v_ar;
            else
                r_tr_a <= v_ar; r_tg_a <= v_ag; r_tb_a <= v_ab;
            end if;
            -- 14: tone ROMs; 15: fabric copies
            p15_tr <= r_tr_q(29 downto 0); p15_tg <= r_tg_q(29 downto 0); p15_tb <= r_tb_q(29 downto 0);
            -- 16: Y, Cr, Cb sums
            p16_y <= to_unsigned(64, 12) + resize(unsigned(p15_tr(29 downto 20)), 12) + resize(unsigned(p15_tg(29 downto 20)), 12)
                     + resize(unsigned(p15_tb(29 downto 20)), 12);
            p16_u <= to_signed(512, 13) + resize(signed(p15_tr(19 downto 10)), 13) + resize(signed(p15_tg(19 downto 10)), 13)
                     + resize(signed(p15_tb(19 downto 10)), 13);
            p16_v <= to_signed(512, 13) + resize(signed(p15_tr(9 downto 0)), 13) + resize(signed(p15_tg(9 downto 0)), 13)
                     + resize(signed(p15_tb(9 downto 0)), 13);
            -- 17: cap / clamp; neutral outside the generated avid
            if s_gav_sr(14) = '0' then
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            else
                v_y := p16_y;
                if v_y > 940 then s_y17 <= to_unsigned(940, 10); else s_y17 <= v_y(9 downto 0); end if;
                v_c := p16_u;
                if v_c(12) = '1' then s_u17 <= (others => '0'); elsif v_c(11 downto 10) /= "00" then s_u17 <= (others => '1'); else s_u17 <= unsigned(v_c(9 downto 0)); end if;
                v_c := p16_v;
                if v_c(12) = '1' then s_v17 <= (others => '0'); elsif v_c(11 downto 10) /= "00" then s_v17 <= (others => '1'); else s_v17 <= unsigned(v_c(9 downto 0)); end if;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(14);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through a [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums over the delay ring, with the running-sum resync), luma 13/16
    -- and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 18 sums, stage 19 output; the 8-sums snap back to neutral once 8 blank samples are in
            if s_bah = "00000000" then
                s_bay <= to_unsigned(512, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y17 - unsigned(dl_qq(29 downto 20));
                s_bau <= s_bau + s_u17 - unsigned(dl_qq(19 downto 10));
                s_bav <= s_bav + s_v17 - unsigned(dl_qq(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bay, 14) + s_bayd; s_y23 <= v_sc(13 downto 4);
            v_sc := resize(s_bau, 14) + s_baud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v23 <= v_sc(13 downto 4);
            -- 20: luma 13/16
            if s_y23 > 64 then v_dl := s_y23 - 64; else v_dl := (others => '0'); end if;
            s_y24 <= s_y23 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u24 <= s_u23; s_v24 <= s_v23;
            -- 21: blanking gate
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
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture mantis;
