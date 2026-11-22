-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Gimbal v0.1.0 -- Kinetic.  After the armillary sphere and the gyroscope's gimbals, and the kinetic ring
-- sculptures in which nested hoops each turn about their own axis: five flat rings and a hub disc, each ring
-- hinged across a diameter and tilting in 3-D.  A tilted ring projects to an ellipse; its two faces carry
-- palette A and B; its front half passes in front of the other rings and its back half behind.
--
-- Element: a ring k (0 .. 4, inner to outer; the hub disc lies in ring 0's plane).  State: its tilt
--     alpha_k = Depth * wave(phase + Cascade * index_k) + camera_k                          (Rock)
--     alpha_k = (1 + k/4) phase + Cascade * index_k + Depth/2 * wave(..) + camera_k       (Tumble)
-- about a hinge at angle 90 deg + k * Fan.  In the hinge frame (u along it) the ring is
--     c'^2 u^2 + v^2 in [c'^2 Ri^2 - tau^2, c'^2 Ro^2],   c' = max(|cos alpha|, 2 / Ro)
-- which on the screen is one quadratic form per ring, normalised by 1 / (2 c' Ro) so that the difference to a
-- threshold is a distance in pixels across the ring's flat side.  Depth z = v tan alpha.
--
--   K1  Scale    ring radii x 0.55 .. 1.6 (exponential, glided)
--   K2  Fan      hinge step between rings 0 .. 90 degrees (0 = one axis: a slatted disc; 90 = a true gimbal)
--   K3  Depth    swing, square law, 0.04 .. 0.5 cycle (a half turn) each way
--   K4  Rate     wave speed, bipolar, centre = still (60 s per cycle .. 2 cycles per second)
--   K5  Colour   the 16 shared Kinetic palettes (hysteresis)
--   K6  Camera   camera (and Osc) strength: the luma under each ring adds to its tilt; bright camera areas
--                tint the faces toward the accent in posterised steps
--   S7  Wave     Smooth / Snap        S8  Field  Chase (index k) / Mirror (index |k - 2|)
--   S9  Spin     Rock / Tumble        S10 Rings  Wide / Wire        S11 Source  Video / Osc
--   P12 CASCADE  phase step per ring 0 .. 0.46 cycle, square law (0 = one flipping nest, top = a storm)
--
-- Sections (the integer model .kinetic_work/rb_gimbal/imodel.py mirrors each one; riley's chassis):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         switch latches, palette hysteresis (glides and the rate decode run
--                                        in the engine's microcode)
--   [C] FIELD ENGINE                     a microcoded 48-bit engine (gimbal.py: program, ROMs, emulator): one
--                                        adder, a serial multiplier / divider, a 256 x 48 register file in
--                                        block RAM; runs once per field in windows of the active lines and
--                                        leaves the NEXT field's values in shadow slots
--   [L] LINE ENGINE                      vblank copy (shadows -> live), pixel-constant loads, per line the
--                                        second-difference steps of every ring's start values, and the loads
--                                        of the pixel DDAs through their own adders
--   [D] SOURCE FIELD                     riley's Osc lattice, 32-px box, vertical IIR line store; the camera
--                                        samples (centre line, centre column) for the per-ring tilt
--   [E] RINGS + SELECTION + BLEND        per ring three DDAs (quadratic, its x difference, depth), coverage,
--                                        front / second ring by depth, colour table, coverage blend
--   [F] OUTPUT CONDITIONING              chroma [1 2 2 2 2 2 2 2 1]/16 (delay in block RAM), luma [1 2 1]/4,
--                                        luma 13/16, blanking gate
--
-- Stage numbering (riley's): a register "at stage S" holds pixel n at T + S + n (T = the g_start clock).
-- s_gav_sr(k) flags pixels 0 .. W - 1 at stage k + 2.  Pixels 0 .. W + 7 are rendered.  One latency (28).
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.gimbal_pkg.all;
use work.all;

architecture gimbal of program_top is
    constant C_LAT : integer := 28;   -- = processing_delay_clks
    constant NR    : integer := C_GB_NR;            -- rings
    constant BB    : integer := 16 * C_GB_NR;       -- register-file block B (pixel-constant shadows)

    ---------------------------------------------------------------------------
    -- [A] timing-ID steps (riley)
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
    -- signed a + b or a - b on ONE carry chain (49 bits)
    function f_as49(a, b : signed(48 downto 0); sub : std_logic) return signed is
        variable v_a, v_b, v_r : signed(49 downto 0);
    begin
        v_a := a & '1';
        if sub = '1' then v_b := (not b) & '1'; else v_b := b & '0'; end if;
        v_r := v_a + v_b;
        return v_r(49 downto 1);
    end function;
    -- clamp to 0 .. 16 by bit tests (no compare chain)
    function f_clamp16(v : signed) return unsigned is
    begin
        if v(v'high) = '1' then return to_unsigned(0, 5);
        elsif v(v'high - 1 downto 4) /= 0 then return to_unsigned(16, 5);
        else return '0' & unsigned(v(3 downto 0)); end if;
    end function;
    -- outline coverage clamp(t + 48, 0, 16) by bit tests: t >= -32 -> 16, -48 <= t < -32 -> t mod 16, else 0
    function f_out16(v : signed) return unsigned is
    begin
        if v(v'high) = '0' or v(v'high - 1 downto 5) = (v'high - 1 downto 5 => '1') then return to_unsigned(16, 5);
        elsif v(v'high - 1 downto 4) = ((v'high - 1 downto 6 => '1') & "01") then return '0' & unsigned(v(3 downto 0));
        else return to_unsigned(0, 5); end if;
    end function;
    -- t + 48 > 0
    function f_in48(v : signed) return std_logic is
    begin
        if v(v'high) = '0' or v(v'high - 1 downto 5) = (v'high - 1 downto 5 => '1') then return '1';
        elsif v(v'high - 1 downto 4) = ((v'high - 1 downto 6 => '1') & "01") and v(3 downto 0) /= 0 then return '1';
        else return '0'; end if;
    end function;
    function f_bit(v : signed; i : integer) return std_logic is
    begin
        return v(i);
    end function;
    -- register-file block A base of ring k
    function f_rb(k : unsigned(2 downto 0)) return unsigned is
    begin
        return '0' & k & "0000";
    end function;

    attribute no_rw_check : boolean;
    attribute keep : boolean;
    attribute fsm_encoding : string;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_wlim : unsigned(11 downto 0) := to_unsigned(1855, 12);

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
    signal s_st       : std_logic_vector(0 to 8) := (others => '0');

    -- [B] switches, palette
    signal s_snap, s_mirror, s_tumble, s_wire, s_osc : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(3, 4);

    -- ROMs (one 1024 x 16: sine, palettes, 2^x; the field engine and the fill engine never share a clock)
    signal rom_a, f_ra, e_rma : unsigned(9 downto 0) := (others => '0');
    signal rom_q, rom_f : unsigned(15 downto 0) := (others => '0');
    signal uc_a : unsigned(8 downto 0) := (others => '0');
    signal uc_q : unsigned(21 downto 0) := (others => '0');

    -- register file (256 x 48, initialised with the constants)
    signal rf_ram : t_c_gb_rf := C_GB_RF;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_ra, rf_wa : unsigned(7 downto 0) := (others => '0');
    signal rf_q, rf_d, rf_e, rf_wd : signed(47 downto 0) := (others => '0');
    signal rf_we : std_logic := '0';

    -- [C] field engine
    type t_es is (EI, EN, EF1, EF2, ED, ER1, ER2, EX, EMA, EMB, EMUL, EMUL2, EDIV, EDIV2, EDIV3, EDV2, EDV3, ESH,
                  EGL1, EGL2, EGL3, ENX, ENEG, EMM, ESN1, ESN2, ESN3, ESN4, ESN5, ESN6, ESN7, ESN8, ESN9, ESN10,
                  EEX1, EEX2, EEX3, EEX4, EEX5, ESM1, ESM2, ESM3, ESM4);
    -- the datapath controls of a state (registered when the state is entered)
    constant F_ACS : integer := 0;  constant F_ACD : integer := 1;  constant F_ACM : integer := 2;
    constant F_SHL : integer := 3;  constant F_SH : integer := 4;   constant F_SHR : integer := 5;
    constant F_XL : integer := 6;   constant F_Y0 : integer := 7;   constant F_YACC : integer := 8;
    constant F_YSHR : integer := 9; constant F_YSHL : integer := 10; constant F_AUXK : integer := 11;
    constant F_AUX0 : integer := 12; constant F_AUXY : integer := 13; constant F_AUXR : integer := 14;
    constant F_AUXS : integer := 15; constant F_AS : integer := 16; constant F_SRF : integer := 17;
    constant F_GXR : integer := 18; constant F_SB : integer := 19; constant F_EX : integer := 20;
    constant F_EN : integer := 21;
    subtype t_fl is std_logic_vector(0 to 21);
    function f_fl(s : t_es) return t_fl is
        variable r : t_fl := (others => '0');
    begin
        r(F_AS) := '1';                                          -- adder: aux + 0 unless set below
        case s is
            when ER1   => r(F_AUXK) := '1';
            when EX    => r(F_EX) := '1';
            when EN    => r(F_EN) := '1';
            when EMA   => r(F_ACS) := '1'; r(F_SRF) := '1';      -- 0 +- rf (sign patched)
            when EMB   => r(F_ACS) := '1'; r(F_YACC) := '1';
            when EMUL  => r(F_ACS) := '1'; r(F_AS) := '0';       -- acc + (bit ? x : 0) (bit patched)
            when EMUL2 => r(F_SHR) := '1'; r(F_YSHR) := '1';
            when EDIV  => r(F_SHL) := '1'; r(F_AS) := '0'; r(F_GXR) := '1'; r(F_SB) := '1';
            when EDIV2 => r(F_AS) := '0'; r(F_GXR) := '1'; r(F_SB) := '1';
            when EDIV3 => r(F_ACD) := '1'; r(F_YSHL) := '1'; r(F_AS) := '0'; r(F_GXR) := '1'; r(F_SB) := '1';
            when EDV2  => r(F_AUXY) := '1';
            when EDV3 | EEX5 | ESM4 => r(F_ACS) := '1';
            when EGL2  => r(F_AUX0) := '1';
            when EGL3  => r(F_ACS) := '1'; r(F_AS) := '0'; r(F_SRF) := '1';   -- acc + rf (0 + rf patched)
            when ENX   => r(F_XL) := '1';
            when ENEG  => r(F_ACS) := '1'; r(F_GXR) := '1'; r(F_SB) := '1';  -- aux - x
            when ESH   => r(F_SH) := '1';
            when EMM   => r(F_ACM) := '1'; r(F_SRF) := '1';
            when ESN1  => r(F_YACC) := '1';
            when ESN4 | EEX4 => r(F_AUXR) := '1';
            when ESN5  => r(F_ACS) := '1'; r(F_AUXR) := '1';
            when ESN6  => r(F_XL) := '1';
            when ESN7  => r(F_ACS) := '1'; r(F_AUX0) := '1'; r(F_GXR) := '1'; r(F_SB) := '1';
            when ESN8  => r(F_ACS) := '1'; r(F_XL) := '1';
            when ESN9  => r(F_XL) := '1'; r(F_AUXR) := '1';
            when ESN10 => r(F_ACS) := '1'; r(F_AUX0) := '1'; r(F_GXR) := '1';
            when ESM3  => r(F_AUXS) := '1';
            when others => null;
        end case;
        return r;
    end function f_fl;
    signal e_st : t_es := EI;
    signal e_xt : t_es := EN;
    signal e_xfl : t_fl := (others => '0');
    signal e_c1, e_db, e_it0, o_mul, o_cld : std_logic := '0';
    attribute fsm_encoding of e_st : signal is "none";
    signal e_pc, e_jt : unsigned(8 downto 0) := (others => '0');
    signal e_jf : std_logic := '0';
    signal e_ra, e_wa : unsigned(7 downto 0) := (others => '0');
    signal e_auxin : unsigned(23 downto 0) := (others => '0');
    -- instruction-class flags, decoded from e_ir every clock (valid from ER1 on)
    signal o_acs, o_xl, o_div, o_sin, o_max, o_min, o_exas, o_exsrf, o_exsb : std_logic := '0';
    -- the datapath controls of the CURRENT state, registered from the next-state value (no state decode in the datapath)
    signal c_acs, c_acd, c_acm, c_shl, c_sh, c_shr, c_xl : std_logic := '0';
    signal c_y0, c_yacc, c_yshr, c_yshl : std_logic := '0';
    signal c_auxk, c_aux0, c_auxy, c_auxr, c_auxs, c_ex, c_en : std_logic := '0';
    signal e_ir : unsigned(21 downto 0) := (others => '0');
    signal e_rk : unsigned(2 downto 0) := (others => '0');
    signal e_acc, e_x : signed(47 downto 0) := (others => '0');
    signal e_aux, e_y : unsigned(23 downto 0) := (others => '0');
    signal e_cnt : unsigned(4 downto 0) := (others => '0');
    signal e_f, e_neg, e_sq, e_lt, e_dir, e_glc, e_we, e_run, e_win : std_logic := '0';
    signal e_sb : std_logic := '0';
    -- the adder's operand selects, four copies (one per slice: the fan-out to 49 bits was the critical net)
    signal e_as, e_srf, e_gxr : std_logic_vector(0 to 3) := (others => '0');
    attribute keep of e_as : signal is true;
    attribute keep of e_srf : signal is true;
    attribute keep of e_gxr : signal is true;
    signal e_it : signed(3 downto 0) := (others => '0');
    signal e_a1, e_a2 : unsigned(9 downto 0) := (others => '0');
    signal w_a, w_b, w_sum : signed(48 downto 0);

    -- [L] line engine
    type t_ls is (LI, LCOPY, LCONST, LKH, LKC, LLOAD, LADD);
    signal l_st : t_ls := LI;
    attribute fsm_encoding of l_st : signal is "none";
    signal l_k : unsigned(2 downto 0) := (others => '0');
    signal l_w : unsigned(3 downto 0) := (others => '0');
    signal l_ra, l_wa : unsigned(7 downto 0) := (others => '0');
    signal l_wd : signed(47 downto 0) := (others => '0');
    signal l_we : std_logic := '0';
    signal l_busy : std_logic := '0';
    signal l_tail : unsigned(2 downto 0) := (others => '0');
    type t_tk is array (0 to 2) of unsigned(2 downto 0);
    type t_ta is array (0 to 2) of unsigned(7 downto 0);
    signal l_tk : t_tk := (others => (others => '0'));
    signal l_ta : t_ta := (others => (others => '0'));
    signal l_kz, l_kl, l_ka : std_logic := '0';
    -- load strobes (valid while rf_e holds the word)
    signal ld_k2a, ld_rq, ld_rd, ld_ceqd, ld_kz, ld_rz, ld_cez : std_logic_vector(0 to 7) := (others => '0');
    signal ld_wh, ld_kr, ld_lvl, ld_face : std_logic_vector(0 to 7) := (others => '0');
    signal ld_kh, ld_kc : std_logic := '0';

    -- [E] per-ring pixel path
    type t_s44 is array (0 to NR - 1) of signed(43 downto 0);
    type t_s24 is array (0 to NR - 1) of signed(23 downto 0);
    type t_u12 is array (0 to NR - 1) of unsigned(11 downto 0);
    type t_s14 is array (0 to NR - 1) of signed(13 downto 0);
    type t_s13 is array (0 to NR - 1) of signed(12 downto 0);
    type t_s10 is array (0 to NR - 1) of signed(9 downto 0);
    type t_u5 is array (0 to NR - 1) of unsigned(4 downto 0);
    type t_u2 is array (0 to NR - 1) of unsigned(1 downto 0);
    type t_s5 is array (0 to NR - 1) of signed(4 downto 0);
    type t_s7 is array (0 to NR - 1) of signed(6 downto 0);
    signal p_q, p_d, c_k2a : t_s44 := (others => (others => '0'));
    signal p_z, c_kz : t_s24 := (others => (others => '0'));
    signal c_wh : t_u12 := (others => (others => '0'));
    signal c_lvl : t_u5 := (others => (others => '0'));
    signal c_face : std_logic_vector(0 to NR - 1) := (others => '0');
    signal c_kh : signed(13 downto 0) := (others => '0');
    signal c_kc : unsigned(1 downto 0) := (others => '0');
    signal c_wg : unsigned(7 downto 0) := (others => '0');
    signal r_run : std_logic_vector(0 to NR - 1) := (others => '0');
    attribute keep of r_run : signal is true;
    signal r_run0, r_run0d : std_logic := '0';
    signal s3_qm : t_s13 := (others => (others => '0'));
    signal s3_zi : t_s10 := (others => (others => '0'));
    signal s3_far, s4_far : std_logic_vector(0 to NR - 1) := (others => '0');
    signal s4_t : t_s14 := (others => (others => '0'));
    signal s4_u : signed(13 downto 0) := (others => '0');
    signal s5_rq, s6_rq : t_u2 := (others => (others => '0'));
    signal s4_qs : std_logic_vector(0 to NR - 1) := (others => '0');
    signal s4_zs, s5_zs, s6_zs : t_s5 := (others => (others => '0'));
    signal s5_a, s5_o, s6_a, s6_o : t_u5 := (others => (others => '0'));
    signal s5_ah, s5_oh : unsigned(4 downto 0) := (others => '0');
    signal s5_pres : std_logic_vector(0 to NR - 1) := (others => '0');
    -- pairwise depth order: ge(k, j) (k > j) = ring k in front of ring j; index k * (k - 1) / 2 + j
    signal s4_ge, s5_ge : std_logic_vector(0 to 9) := (others => '0');
    signal s6_isf, s6_isb : std_logic_vector(0 to NR - 1) := (others => '0');
    -- selection / colour
    signal s7_af, s7_of, s7_ab, s8_af, s8_of, s8_ab : unsigned(4 downto 0) := (others => '0');
    signal s7_rqf, s7_rqb, s8_rqf, s8_rqb : unsigned(1 downto 0) := (others => '0');
    signal s7_lvf, s7_lvb : unsigned(4 downto 0) := (others => '0');
    signal s7_zsf, s7_zsb : signed(4 downto 0) := (others => '0');
    signal s8_lvf, s8_lvb : signed(6 downto 0) := (others => '0');
    signal s7_fcf, s7_fcb, s8_fcf, s8_fcb : std_logic := '0';
    signal s9_w, s10_w, s11_w, s12_w, s13_w, s14_w : unsigned(4 downto 0) := (others => '0');
    signal s9_lv, s10_lv : unsigned(4 downto 0) := (others => '0');
    signal s9_rq, s10_rq, s10_rqc : unsigned(1 downto 0) := (others => '0');
    signal s9_fc, s10_fc : std_logic := '0';
    signal ct_a : unsigned(7 downto 0) := (others => '0');
    type t_ct is array (0 to 255) of unsigned(15 downto 0);
    signal ct_y, ct_u, ct_v : t_ct := (others => (others => '0'));
    attribute no_rw_check of ct_y : signal is true;
    attribute no_rw_check of ct_u : signal is true;
    attribute no_rw_check of ct_v : signal is true;
    signal ct_qy, ct_qu, ct_qv : unsigned(15 downto 0) := (others => '0');
    signal s13_y, s13_u, s13_v : unsigned(9 downto 0) := (others => '0');

    -- fill engine (colour table, vblank)
    signal f_run : std_logic := '0';
    signal f_ch : unsigned(1 downto 0) := (others => '0');
    signal f_t : unsigned(9 downto 0) := (others => '0');
    signal f_ca, f_cb, f_cc : unsigned(9 downto 0) := (others => '0');
    signal f_s : signed(16 downto 0) := (others => '0');
    signal f_e1, f_e2, f_e3, f_e4 : unsigned(7 downto 0) := (others => '0');
    signal f_v1, f_v2, f_v3, f_v4 : std_logic := '0';
    signal f_sh3, f_sh4 : signed(11 downto 0) := (others => '0');
    signal f_dd3 : signed(11 downto 0) := (others => '0');
    signal f_wd : unsigned(15 downto 0) := (others => '0');
    signal f_wa : unsigned(7 downto 0) := (others => '0');
    signal f_we : std_logic_vector(0 to 2) := (others => '0');
    signal c_gy : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal c_gu, c_gv : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- [D] source field (riley) and camera samples
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal o_x8  : unsigned(10 downto 0) := (others => '0');
    signal s_opy : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_sfirst : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_svd : std_logic := '0';
    signal s_col : unsigned(10 downto 0) := (others => '0');
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(5 downto 0) := (others => '0');
    signal s_bsum : unsigned(12 downto 0) := (others => '0');
    signal s_bv, s_ev : std_logic := '0';
    signal s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal s_eb8, s_eq : unsigned(7 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(7 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_q2, lb_wd : unsigned(7 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal lb_rc : unsigned(10 downto 0) := (others => '0');
    signal s8_q, s8_qp, s9_l : unsigned(7 downto 0) := (others => '0');
    signal sp_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of sp_ram : signal is true;
    signal sp_ra, sp_wa : unsigned(7 downto 0) := (others => '0');
    signal sp_q, sp_f, sp_wd : unsigned(7 downto 0) := (others => '0');
    signal sp_we : std_logic := '0';
    signal s_wbank : std_logic := '0';
    signal s_vy, s_vyp : unsigned(19 downto 0) := (others => '0');
    signal s_cl, s_vl : std_logic := '0';
    signal s_vi : unsigned(5 downto 0) := (others => '0');
    signal s_bx, s_bxp : unsigned(13 downto 0) := (others => '0');
    signal s_vcap : std_logic := '0';
    signal s_vcl : unsigned(7 downto 0) := (others => '0');

    -- [E] blend
    signal s14_dy, s14_du, s14_dv : signed(10 downto 0) := (others => '0');
    signal s15_ay, s15_au, s15_av, s15_by, s15_bu, s15_bv : signed(15 downto 0) := (others => '0');
    signal s16_y, s16_u, s16_v : signed(15 downto 0) := (others => '0');
    signal s17_y, s17_u, s17_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- [F] output conditioning (the 8-back chroma tap from a block-RAM ring)
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_cr is array (0 to 255) of unsigned(15 downto 0);
    signal cr_u, cr_v : t_cr := (others => (others => '0'));
    attribute no_rw_check of cr_u : signal is true;
    attribute no_rw_check of cr_v : signal is true;
    signal cr_wp : unsigned(7 downto 0) := (others => '0');
    signal cr_qu, cr_qv, cr_fu, cr_fv : unsigned(15 downto 0) := (others => '0');
    signal s_yd : t_u10(1 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv2, s_bud, s_bvd : unsigned(12 downto 0) := to_unsigned(4100, 13);
    signal s_hist : std_logic_vector(7 downto 0) := (others => '0');
    signal s18_ya : unsigned(11 downto 0) := (others => '0');
    signal s18_yb : unsigned(9 downto 0) := (others => '0');
    signal s19_y, s20_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s19_u, s19_v, s20_u, s20_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s21_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s21_u, s21_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    constant OP_LD : integer := 1;  constant OP_ADD : integer := 2;  constant OP_SUB : integer := 3;
    constant OP_ST : integer := 4;  constant OP_MUL : integer := 5;  constant OP_DIV : integer := 6;
    constant OP_SHL : integer := 7; constant OP_SHR : integer := 8;  constant OP_SIN : integer := 9;
    constant OP_EXP : integer := 10; constant OP_GLIDE : integer := 11; constant OP_TST : integer := 12;
    constant OP_NEGF : integer := 13; constant OP_ABS : integer := 14; constant OP_MAX : integer := 15;
    constant OP_MIN : integer := 16; constant OP_IN : integer := 17; constant OP_SMP : integer := 18;
    constant OP_LOOP : integer := 19; constant OP_JS : integer := 20; constant OP_JMP : integer := 21;
    constant OP_END : integer := 22;

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    rom_a <= f_ra when f_run = '1' else e_rma;
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            rom_q <= C_GB_ROM(to_integer(rom_a));
            rom_f <= rom_q;
            uc_q <= C_GB_UC(to_integer(uc_a));
        end if;
    end process p_rom;

    -- register file: the line engine and the field engine never use it in the same clock
    rf_ra <= l_ra when l_busy = '1' else e_ra;
    rf_we <= l_we or e_we;
    rf_wa <= l_wa when l_we = '1' else e_wa;
    rf_wd <= l_wd when l_we = '1' else e_acc;
    p_rf : process(clk)
    begin
        if rising_edge(clk) then
            rf_q <= rf_ram(to_integer(rf_ra));
            if rf_we = '1' then rf_ram(to_integer(rf_wa)) <= rf_wd; end if;
            rf_d <= rf_q;
            rf_e <= rf_d;
        end if;
    end process p_rf;

    -- colour table: written by the fill engine in the vblank zone only, read by the pixel path
    p_ct : process(clk)
    begin
        if rising_edge(clk) then
            ct_qy <= ct_y(to_integer(ct_a));
            if f_we(0) = '1' then ct_y(to_integer(f_wa)) <= f_wd; end if;
        end if;
    end process p_ct;
    p_cu : process(clk)
    begin
        if rising_edge(clk) then
            ct_qu <= ct_u(to_integer(ct_a));
            if f_we(1) = '1' then ct_u(to_integer(f_wa)) <= f_wd; end if;
        end if;
    end process p_cu;
    p_cv : process(clk)
    begin
        if rising_edge(clk) then
            ct_qv <= ct_v(to_integer(ct_a));
            if f_we(2) = '1' then ct_v(to_integer(f_wa)) <= f_wd; end if;
        end if;
    end process p_cv;

    -- camera samples: two banks by field parity (written this field, read by the engine next field)
    p_sp : process(clk)
    begin
        if rising_edge(clk) then
            sp_q <= sp_ram(to_integer(sp_ra));
            sp_f <= sp_q;
            if sp_we = '1' then sp_ram(to_integer(sp_wa)) <= sp_wd; end if;
        end if;
    end process p_sp;

    -- box delay: read 32 samples behind the write (riley)
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv = '1' then bx_ram(to_integer(bx_wp)) <= s_sl; end if;
        end if;
    end process p_bxram;

    -- luma line store (riley): port 1 = the IIR's own read, port 2 = the pixel path
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_col(10 downto 1)));
            lb_q2 <= lb_ram(to_integer(lb_rc(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    -- chroma delay rings for the output filter: write every clock, read 6 behind (+ 2 registers = 8 back)
    p_cru : process(clk)
    begin
        if rising_edge(clk) then
            cr_qu <= cr_u(to_integer(cr_wp - 6));
            cr_u(to_integer(cr_wp)) <= resize(s17_u, 16);
        end if;
    end process p_cru;
    p_crv : process(clk)
    begin
        if rising_edge(clk) then
            cr_qv <= cr_v(to_integer(cr_wp - 6));
            cr_v(to_integer(cr_wp)) <= resize(s17_v, 16);
        end if;
    end process p_crv;

    ---------------------------------------------------------------------------
    -- [A] TIMING + [B] CONTROLS
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_c : signed(10 downto 0);
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

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_snap   <= registers_in(6)(0);
                s_mirror <= registers_in(6)(1);
                s_tumble <= registers_in(6)(2);
                s_wire   <= registers_in(6)(3);
                s_osc    <= registers_in(6)(4);
                s_xs     <= s_xsr;
                -- palette: 16 steps with hysteresis (leave a step only 8 past its edge)
                v_k := unsigned(registers_in(4));
                v_c := signed(resize(v_k, 11)) - signed(resize(s_pal & "100000", 11));
                if v_c > 40 or v_c < -40 then s_pal <= v_k(9 downto 6); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_wlim <= s_w - 65;

            -- sync delay, generated-avid and start-pulse shift registers
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 7);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [A] Generated avid (riley)
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
    -- [C] FIELD ENGINE (gimbal.py: the microcode and its emulator)
    --   instruction: op 21..17 | R 16 | sw 15..14 | a 13..5 | n 4..0
    --   ONE adder, operand selects registered one state ahead: A = acc or aux, B = rf or x (gated by y(0)
    --   while multiplying) or 0.  Every load into acc goes through it (aux holds the knob, the input, the
    --   ROM word, the sample, the quotient or zero).  acc has three sources: the sum, acc << 1, acc >> 1;
    --   x loads from acc only, y from acc's low bits.  Multiply: |y| through the adder, then per bit an add
    --   state and a shift state; divide (divisor in acc): a shift state and a subtract state per bit.
    ---------------------------------------------------------------------------
    g_wab : for i in 0 to 48 generate
        w_a(i) <= f_bit(signed(resize(e_aux, 49)), i) when e_as(i / 13) = '1' else f_bit(resize(e_acc, 49), i);
        w_b(i) <= (f_bit(resize(rf_d, 49), i) and e_srf(i / 13)) or (f_bit(resize(e_x, 49), i) and e_gxr(i / 13));
    end generate g_wab;
    w_sum <= f_as49(w_a, w_b, e_sb);

    p_eng : process(clk)
        variable v_op  : integer range 0 to 31;
        variable v_hi  : unsigned(4 downto 0);
        variable v_ad  : unsigned(8 downto 0);
        variable v_sw  : std_logic;
        variable v_j   : unsigned(9 downto 0);
        variable v_n   : unsigned(4 downto 0);
        variable v_rk1 : unsigned(3 downto 0);
        variable v_cnt : unsigned(4 downto 0);
        variable v_xt  : t_es;
        variable v_ns  : t_es;
        variable v_fl  : t_fl;
        procedure go(s : t_es) is
        begin
            v_ns := s; v_fl := f_fl(s);
        end procedure go;
    begin
        if rising_edge(clk) then
            e_win <= '0';
            if g_avid = '1' and g_cnt >= 160 and g_cnt < s_wlim then e_win <= '1'; end if;
            e_we <= '0';
            v_op := to_integer(e_ir(21 downto 17));
            v_n := e_ir(4 downto 0);

            -- instruction decode, every clock: e_ir changes only at EF2, everything below is read from ER1 on
            if e_ir(16) = '1' then v_hi := e_ir(13 downto 9) + e_rk; else v_hi := e_ir(13 downto 9); end if;
            v_ad := v_hi & e_ir(8 downto 5);
            case to_integer(e_ir(15 downto 14)) is
                when 1      => v_sw := s_mirror;
                when 2      => v_sw := s_wire;
                when 3      => v_sw := s_osc;
                when others => v_sw := '0';
            end case;
            if v_sw = '1' then v_ad(0) := not v_ad(0); end if;
            e_ra <= v_ad(7 downto 0);
            -- aux for EX: the knob (GLIDE), the input (IN), else zero (loaded in ER1)
            e_auxin <= (others => '0');
            if v_op = OP_GLIDE then
                case to_integer(v_n) is
                    when 0      => e_auxin <= resize(unsigned(registers_in(0)) & "000", 24);
                    when 1      => e_auxin <= resize(unsigned(registers_in(1)) & "000", 24);
                    when 2      => e_auxin <= resize(unsigned(registers_in(2)) & "000", 24);
                    when 3      => e_auxin <= resize(unsigned(registers_in(7)) & "000", 24);
                    when others => e_auxin <= resize(unsigned(registers_in(5)) & "000", 24);
                end case;
            elsif v_op = OP_IN then
                case to_integer(v_n) is
                    when 0      => e_auxin <= resize(unsigned(registers_in(3)), 24);
                    when 1      => e_auxin <= resize(s_ys, 24);
                    when 2      => e_auxin <= resize(s_xs, 24);
                    when others =>
                        if s_xs = 4 then e_auxin <= to_unsigned(720, 24); else e_auxin <= to_unsigned(960, 24); end if;
                end case;
            end if;
            o_acs <= '0'; o_xl <= '0'; o_div <= '0'; o_sin <= '0'; o_max <= '0'; o_min <= '0'; o_mul <= '0'; o_cld <= '0';
            case v_op is
                when OP_LD | OP_ADD | OP_SUB | OP_IN | OP_GLIDE => o_acs <= '1';
                when OP_DIV => o_acs <= '1'; o_xl <= '1'; o_div <= '1'; o_cld <= '1';
                when OP_MUL => o_xl <= '1'; o_mul <= '1'; o_cld <= '1';
                when OP_SHL | OP_SHR => o_cld <= '1';
                when OP_SIN => o_sin <= '1';
                when OP_MAX => o_max <= '1';
                when OP_MIN => o_min <= '1';
                when others => null;
            end case;
            -- the adder in EX: ADD acc + rf, SUB / MAX / MIN acc - rf, GLIDE aux - rf, LD / DIV 0 + rf, else aux + 0
            case v_op is
                when OP_ADD => o_exas <= '0'; o_exsrf <= '1'; o_exsb <= '0';
                when OP_SUB | OP_MAX | OP_MIN => o_exas <= '0'; o_exsrf <= '1'; o_exsb <= '1';
                when OP_GLIDE => o_exas <= '1'; o_exsrf <= '1'; o_exsb <= '1';
                when OP_LD | OP_DIV => o_exas <= '1'; o_exsrf <= '1'; o_exsb <= '0';
                when others => o_exas <= '1'; o_exsrf <= '0'; o_exsb <= '0';
            end case;
            -- the state after EX and its controls (acc and the flag are steady from ED to EX)
            case v_op is
                when OP_MUL => v_xt := EMA;
                when OP_DIV => v_xt := EDIV;
                when OP_SHL | OP_SHR =>
                    if v_n /= 0 then v_xt := ESH; else v_xt := EN; end if;
                when OP_SIN => v_xt := ESN1;
                when OP_EXP => v_xt := EEX1;
                when OP_GLIDE => v_xt := EGL1;
                when OP_NEGF =>
                    if e_f = '1' then v_xt := ENX; else v_xt := EN; end if;
                when OP_ABS =>
                    if e_acc(47) = '1' then v_xt := ENX; else v_xt := EN; end if;
                when OP_MAX | OP_MIN => v_xt := EMM;
                when OP_SMP => v_xt := ESM1;
                when others => v_xt := EN;
            end case;
            e_xt <= v_xt; e_xfl <= f_fl(v_xt);
            -- registered conditions
            e_lt <= w_sum(48);                        -- the adder's sign, every clock (EDIV3 reads EDIV2's, EMM EX's)
            if e_acc(47) = '0' then                   -- GLIDE deadband |knob * 8 - g| <= 16 (read in EGL2)
                if e_acc(15 downto 5) = 0 and (e_acc(4) = '0' or e_acc(3 downto 0) = 0) then e_db <= '1'; else e_db <= '0'; end if;
            else
                if e_acc(15 downto 4) = (15 downto 4 => '1') then e_db <= '1'; else e_db <= '0'; end if;
            end if;
            if e_it = 0 then e_it0 <= '1'; else e_it0 <= '0'; end if;

            -- EX's side effects (c_ex is EX's own registered flag: no state decode)
            e_wa <= e_ra; e_jt <= e_ir(13 downto 5);
            uc_a <= e_pc;
            e_ir <= uc_q;                             -- steady from EF2 until the next EN moves the counter
            if c_ex = '1' then
                e_jf <= '0';
                case v_op is
                    when OP_ST  => e_we <= '1';
                    when OP_MUL => e_neg <= rf_d(47);
                    when OP_SHL => e_dir <= '0';
                    when OP_SHR => e_dir <= '1';
                    when OP_EXP => e_rma <= "11" & unsigned(e_acc(7 downto 0));
                    when OP_TST => e_f <= e_acc(47);
                    when OP_SMP => sp_ra <= (not s_wbank) & unsigned(e_acc(6 downto 0));
                    when OP_LOOP =>
                        v_rk1 := resize(e_rk, 4) + 1;
                        if v_rk1 < resize(v_n, 4) then
                            e_rk <= v_rk1(2 downto 0); e_jf <= '1';
                        else
                            e_rk <= (others => '0');
                        end if;
                    when OP_JS =>
                        case to_integer(v_n) is
                            when 0      => v_sw := s_snap;
                            when 1      => v_sw := s_mirror;
                            when 2      => v_sw := s_tumble;
                            when 3      => v_sw := s_wire;
                            when others => v_sw := s_osc;
                        end case;
                        if v_sw = '1' then e_jf <= '1'; end if;
                    when OP_JMP => e_jf <= '1';
                    when OP_END => e_run <= '0';
                    when others => null;
                end case;
            end if;
            if c_en = '1' then                        -- EN: the only place the program counter moves
                if e_jf = '1' then e_pc <= e_jt; else e_pc <= e_pc + 1; end if;
            end if;

            v_cnt := e_cnt;
            v_ns := e_st; v_fl := (others => '0');
            if s_zgo = '1' then                       -- restart every field (the watchdog)
                go(EI); e_pc <= (others => '0'); e_rk <= (others => '0'); e_run <= '1';
                e_glc <= '0';
            else
                case e_st is
                    when EI =>
                        if e_run = '1' and e_win = '1' then go(EF1); end if;
                    when EN => go(EI);
                    when EF1 => go(EF2);
                    when EF2 => go(ED);
                    when ED  => go(ER1);
                    when ER1 => go(ER2);
                    when ER2 =>
                        go(EX);
                        v_fl(F_ACS) := o_acs; v_fl(F_XL) := o_xl; v_fl(F_Y0) := o_div;
                        v_fl(F_AS) := o_exas; v_fl(F_SRF) := o_exsrf; v_fl(F_SB) := o_exsb;
                    when EX =>
                        v_ns := e_xt; v_fl := e_xfl;
                        if o_mul = '1' then v_fl(F_SB) := rf_d(47); end if;     -- EMA: 0 +- rf, by rf's sign
                        if o_cld = '1' then v_cnt := v_n; end if;                -- MUL, DIV, SHL, SHR
                    when EMA => go(EMB);               -- acc = |rf|
                    when EMB =>                        -- y = |rf|, acc = 0
                        go(EMUL); v_fl(F_GXR) := e_acc(0);
                    when EMUL => go(EMUL2);            -- acc += y0 x
                    when EMUL2 =>                      -- acc >>= 1, y >>= 1
                        v_cnt := e_cnt - 1;
                        if e_c1 = '0' then
                            go(EMUL); v_fl(F_GXR) := e_y(1);
                        elsif o_sin = '1' then
                            go(ESN9);
                        elsif e_neg = '1' then
                            go(ENX);
                        else
                            go(EN);
                        end if;
                    when EDIV  => go(EDIV2);           -- R = 2 R
                    when EDIV2 => go(EDIV3);           -- (e_lt takes the sign of R - x)
                    when EDIV3 =>                      -- R >= x: R -= x, quotient bit 1
                        v_cnt := e_cnt - 1;
                        if e_c1 = '1' then go(EDV2); else go(EDIV); end if;
                    when EDV2 => go(EDV3);             -- aux = quotient
                    when EDV3 => go(EN);               -- acc = quotient
                    when ESH =>
                        v_cnt := e_cnt - 1;
                        if e_c1 = '0' then
                            go(ESH);
                        elsif e_glc = '1' then
                            e_glc <= '0'; go(EGL3);
                        else
                            go(EN);
                        end if;
                    when EGL1 => go(EGL2);             -- (e_db from d = knob * 8 - g)
                    when EGL2 =>                       -- deadband keeps g; else d >> 3, then + g
                        if e_db = '0' then
                            e_dir <= '1'; v_cnt := to_unsigned(3, 5); e_glc <= '1'; go(ESH);
                        else
                            go(EGL3); v_fl(F_AS) := '1';    -- 0 + rf (aux was zeroed in EGL2)
                        end if;
                    when EGL3 | ENEG => go(EN);
                    when ENX => go(ENEG);              -- negate: x = acc, then acc = 0 - x (aux is 0)
                    when EMM => go(EN);
                    -- SIN: v1 = T(i), v2 = T(i + 1) (quadrant-mirrored), d = v2 - v1, P = d f >> 13, v1 + P, sign
                    when ESN1 =>
                        v_j := resize(unsigned(e_acc(21 downto 13)), 10);
                        if e_acc(22) = '0' then
                            e_a1 <= v_j; e_a2 <= v_j + 1; e_rma <= v_j;
                        else
                            v_j := to_unsigned(512, 10) - v_j;
                            e_a1 <= v_j; e_a2 <= v_j - 1; e_rma <= v_j;
                        end if;
                        e_sq <= e_acc(23);
                        go(ESN2);
                    when ESN2 => e_rma <= e_a2; go(ESN3);
                    when ESN3 => go(ESN4);
                    when ESN4 => go(ESN5);             -- aux = v1
                    when ESN5 => e_rma <= e_a1; go(ESN6);   -- acc = v1, aux = v2, re-read v1
                    when ESN6 => go(ESN7);             -- x = v1
                    when ESN7 => go(ESN8);             -- acc = v2 - v1, aux = 0
                    when ESN8 =>                       -- x = d, acc = 0, 13 multiply steps by f
                        v_cnt := to_unsigned(13, 5); go(EMUL); v_fl(F_GXR) := e_y(0);
                    when ESN9 => go(ESN10);            -- x = P, aux = v1
                    when ESN10 =>                      -- acc = v1 + P, aux = 0, then the sign of the half cycle
                        if e_sq = '1' then go(ENX); else go(EN); end if;
                    when EEX1 => e_it <= e_acc(11 downto 8); go(EEX2);
                    when EEX2 => go(EEX3);
                    when EEX3 => go(EEX4);
                    when EEX4 => go(EEX5);             -- aux = ROM
                    when EEX5 =>                       -- acc = aux, then shift by the exponent
                        if e_it0 = '1' then
                            go(EN);
                        elsif e_it(3) = '0' then
                            e_dir <= '0'; v_cnt := resize(unsigned(e_it), 5); go(ESH);
                        else
                            e_dir <= '1'; v_cnt := resize(unsigned(-e_it), 5); go(ESH);
                        end if;
                    when ESM1 => go(ESM2);
                    when ESM2 => go(ESM3);
                    when ESM3 => go(ESM4);             -- aux = sample
                    when ESM4 => go(EN);               -- acc = aux
                end case;
            end if;
            e_st <= v_ns;
            e_cnt <= v_cnt;
            if v_cnt = 1 then e_c1 <= '1'; else e_c1 <= '0'; end if;
            -- the datapath controls of the next state
            c_acs <= v_fl(F_ACS); c_acd <= v_fl(F_ACD); c_acm <= v_fl(F_ACM);
            c_shl <= v_fl(F_SHL); c_sh <= v_fl(F_SH); c_shr <= v_fl(F_SHR); c_xl <= v_fl(F_XL);
            c_y0 <= v_fl(F_Y0); c_yacc <= v_fl(F_YACC); c_yshr <= v_fl(F_YSHR); c_yshl <= v_fl(F_YSHL);
            c_auxk <= v_fl(F_AUXK); c_aux0 <= v_fl(F_AUX0); c_auxy <= v_fl(F_AUXY);
            c_auxr <= v_fl(F_AUXR); c_auxs <= v_fl(F_AUXS); c_ex <= v_fl(F_EX); c_en <= v_fl(F_EN);
            e_as <= (others => v_fl(F_AS)); e_srf <= (others => v_fl(F_SRF));
            e_gxr <= (others => v_fl(F_GXR)); e_sb <= v_fl(F_SB);
        end if;
    end process p_eng;

    -- the engine's datapath: driven by the registered controls only
    p_edp : process(clk)
    begin
        if rising_edge(clk) then
            if c_acs = '1' or (c_acd = '1' and e_lt = '0') or
               (c_acm = '1' and ((o_max = '1' and e_lt = '1') or (o_min = '1' and e_lt = '0'))) then
                e_acc <= w_sum(47 downto 0);
            elsif c_shl = '1' or (c_sh = '1' and e_dir = '0') then
                e_acc <= e_acc(46 downto 0) & '0';
            elsif c_shr = '1' or (c_sh = '1' and e_dir = '1') then
                e_acc <= e_acc(47) & e_acc(47 downto 1);
            end if;
            if c_xl = '1' then e_x <= e_acc; end if;
            if c_y0 = '1' then
                e_y <= (others => '0');
            elsif c_yacc = '1' then
                e_y <= unsigned(e_acc(23 downto 0));
            elsif c_yshr = '1' then
                e_y <= '0' & e_y(23 downto 1);
            elsif c_yshl = '1' then
                e_y <= e_y(22 downto 0) & (not e_lt);
            end if;
            if c_auxk = '1' then
                e_aux <= e_auxin;
            elsif c_aux0 = '1' then
                e_aux <= (others => '0');
            elsif c_auxy = '1' then
                e_aux <= e_y;
            elsif c_auxr = '1' then
                e_aux <= resize(rom_f, 24);
            elsif c_auxs = '1' then
                e_aux <= resize(sp_f, 24);
            end if;
        end if;
    end process p_edp;

    ---------------------------------------------------------------------------
    -- [L] LINE ENGINE.  Every read is tagged; the tag rides with the address
    -- (l_tk(0)), the block RAM (l_tk(1)) and the fabric register (l_tk(2), when
    -- rf_d holds the word); load strobes are registered once more and act
    -- while rf_e holds the word.
    --   zone:  COPY (per ring: shadow words 7 .. 13 -> 0 .. 6, block B 0 / 1 -> 14 / 15),
    --          CONST (Wh, Kr, lvl, face of every ring), KH (ring 0's hub), KC, LOAD (line 0)
    --   line:  ADD at g_cnt = 64 (Q0 += E, E += E2, D0 += Dy, z0 += Zy), LOAD when the
    --          pixel DDAs have finished the line
    --   LOAD per ring, five words through the DDAs' own adders (no load multiplexers):
    --     Q0: K2A <- w, Q <- 0, D <- 0        D0: K2A <- w, D <- D + K2A (= Q0)
    --     z0: Kz <- w, z <- 0, Q <- Q + D (= Q0), D <- 0
    --     K2A: K2A <- w, D <- D + K2A (= D0), z <- z + Kz (= z0)        Kz: Kz <- w
    -- tag kinds: 1 copy (dst)  3 add Y (write rf_e + rf_d)  4 const (k, c)  5 load (k, w)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_add : std_logic;
        variable v_x, v_y : unsigned(3 downto 0);
        variable v_kk : integer range 0 to 7;
        variable v_c : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            l_kz <= s_zgo;
            r_run0d <= r_run0;
            l_kl <= r_run0d and not r_run0;
            l_ka <= '0';
            if g_avid = '1' and g_cnt = 64 then l_ka <= '1'; end if;

            -- issue
            l_tk(0) <= (others => '0');
            case l_st is
                when LI =>
                    l_k <= (others => '0'); l_w <= (others => '0');
                    if l_kz = '1' then l_st <= LCOPY;
                    elsif l_kl = '1' then l_st <= LLOAD;
                    elsif l_ka = '1' then l_st <= LADD; end if;
                when LCOPY =>
                    if l_w < 7 then
                        l_ra <= f_rb(l_k) + resize(l_w, 8) + 7; l_ta(0) <= f_rb(l_k) + resize(l_w, 8);
                    else
                        l_ra <= f_rb(l_k) + BB + resize(l_w - 7, 8); l_ta(0) <= f_rb(l_k) + 7 + resize(l_w, 8);
                    end if;
                    l_tk(0) <= "001";
                    if l_w = 8 then
                        l_w <= (others => '0');
                        if l_k = NR - 1 then l_k <= (others => '0'); l_st <= LCONST; else l_k <= l_k + 1; end if;
                    else
                        l_w <= l_w + 1;
                    end if;
                when LCONST =>
                    l_ra <= f_rb(l_k) + (BB + 2) + resize(l_w, 8);
                    l_tk(0) <= "100"; l_ta(0) <= "00" & l_k & l_w(2 downto 0);
                    if l_w = 3 then
                        l_w <= (others => '0');
                        if l_k = NR - 1 then l_k <= (others => '0'); l_st <= LKH; else l_k <= l_k + 1; end if;
                    else
                        l_w <= l_w + 1;
                    end if;
                when LKH =>
                    l_ra <= to_unsigned(BB + 13, 8);
                    l_tk(0) <= "100"; l_ta(0) <= "00" & "111" & "001";
                    l_st <= LKC;
                when LKC =>
                    l_ra <= to_unsigned(C_GB_KC, 8);
                    l_tk(0) <= "100"; l_ta(0) <= "00" & "111" & "000";
                    l_st <= LLOAD;
                when LLOAD =>
                    case to_integer(l_w) is
                        when 0      => l_ra <= f_rb(l_k);
                        when 1      => l_ra <= f_rb(l_k) + 2;
                        when 2      => l_ra <= f_rb(l_k) + 3;
                        when 3      => l_ra <= f_rb(l_k) + 14;
                        when others => l_ra <= f_rb(l_k) + 15;
                    end case;
                    l_tk(0) <= "101"; l_ta(0) <= "00" & l_k & l_w(2 downto 0);
                    if l_w = 4 then
                        l_w <= (others => '0');
                        if l_k = NR - 1 then l_k <= (others => '0'); l_st <= LI; else l_k <= l_k + 1; end if;
                    else
                        l_w <= l_w + 1;
                    end if;
                when LADD =>
                    case to_integer(l_w(2 downto 1)) is
                        when 0      => v_x := "0000"; v_y := "0001";
                        when 1      => v_x := "0001"; v_y := "0100";
                        when 2      => v_x := "0010"; v_y := "0101";
                        when others => v_x := "0011"; v_y := "0110";
                    end case;
                    if l_w(0) = '0' then
                        l_ra <= f_rb(l_k) + resize(v_x, 8);
                    else
                        l_ra <= f_rb(l_k) + resize(v_y, 8); l_tk(0) <= "011";
                    end if;
                    l_ta(0) <= f_rb(l_k) + resize(v_x, 8);
                    if l_w = 7 then
                        l_w <= (others => '0');
                        if l_k = NR - 1 then l_k <= (others => '0'); l_st <= LI; else l_k <= l_k + 1; end if;
                    else
                        l_w <= l_w + 1;
                    end if;
            end case;

            -- busy (owns the RF read port) while issuing and for the tail
            if l_st /= LI then
                l_busy <= '1'; l_tail <= to_unsigned(5, 3);
            elsif l_tail /= 0 then
                l_tail <= l_tail - 1;
            else
                l_busy <= '0';
            end if;

            -- tag pipeline
            l_tk(1) <= l_tk(0); l_ta(1) <= l_ta(0);
            l_tk(2) <= l_tk(1); l_ta(2) <= l_ta(1);

            -- actions at rf_d: copy (rf_d) or add (rf_e + rf_d) into the register file one clock later
            l_we <= '0';
            if l_tk(2) = "001" or l_tk(2) = "011" then l_we <= '1'; end if;
            l_wa <= l_ta(2);
            if l_tk(2) = "011" then v_add := '1'; else v_add := '0'; end if;
            l_wd <= rf_d + (rf_e and (47 downto 0 => v_add));
            ld_k2a <= (others => '0'); ld_rq <= (others => '0'); ld_rd <= (others => '0');
            ld_ceqd <= (others => '0'); ld_kz <= (others => '0'); ld_rz <= (others => '0');
            ld_cez <= (others => '0'); ld_wh <= (others => '0'); ld_kr <= (others => '0');
            ld_lvl <= (others => '0'); ld_face <= (others => '0'); ld_kh <= '0'; ld_kc <= '0';
            v_kk := to_integer(l_ta(2)(5 downto 3));
            v_c := to_integer(l_ta(2)(2 downto 0));
            case to_integer(l_tk(2)) is
                when 4 =>
                    if v_kk = 7 then
                        if v_c = 1 then ld_kh <= '1'; else ld_kc <= '1'; end if;
                    else
                        case v_c is
                            when 0      => ld_wh(v_kk) <= '1';
                            when 1      => ld_kr(v_kk) <= '1';
                            when 2      => ld_lvl(v_kk) <= '1';
                            when others => ld_face(v_kk) <= '1';
                        end case;
                    end if;
                when 5 =>
                    case v_c is
                        when 0 => ld_k2a(v_kk) <= '1'; ld_rq(v_kk) <= '1'; ld_rd(v_kk) <= '1'; ld_ceqd(v_kk) <= '1';
                        when 1 => ld_k2a(v_kk) <= '1'; ld_ceqd(v_kk) <= '1';
                        when 2 => ld_kz(v_kk) <= '1'; ld_rz(v_kk) <= '1'; ld_cez(v_kk) <= '1';
                                  ld_rd(v_kk) <= '1'; ld_ceqd(v_kk) <= '1';
                        when 3 => ld_k2a(v_kk) <= '1'; ld_ceqd(v_kk) <= '1'; ld_cez(v_kk) <= '1';
                        when others => ld_kz(v_kk) <= '1';
                    end case;
                when others => null;
            end case;
        end if;
    end process p_line;

    -- pixel constants (loaded from rf_e in the vblank zone)
    p_const : process(clk)
    begin
        if rising_edge(clk) then
            for r in 0 to NR - 1 loop
                if ld_wh(r) = '1' then c_wh(r) <= unsigned(rf_e(11 downto 0)); end if;
                if ld_lvl(r) = '1' then c_lvl(r) <= unsigned(rf_e(4 downto 0)); end if;
                if ld_face(r) = '1' then c_face(r) <= rf_e(0); end if;
            end loop;
            if ld_kh = '1' then c_kh <= rf_e(13 downto 0); end if;
            if ld_kc = '1' then c_kc <= unsigned(rf_e(1 downto 0)); c_wg <= unsigned(rf_e(9 downto 2)); end if;
        end if;
    end process p_const;

    ---------------------------------------------------------------------------
    -- colour table fill (vblank zone): index = face * 128 + level * 4 + rq;
    -- shade = base + ((C - base) (level + 1)) >> 5, then a mix toward the accent
    -- by rq: 0, 16, 32, 64 / 64; three passes (Y, U, V)
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_base : signed(11 downto 0);
        variable v_df : signed(16 downto 0);
        variable v_d : signed(18 downto 0);
        variable v_m : signed(11 downto 0);
        variable v_pa, v_pc : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            f_we <= (others => '0');
            f_v1 <= '0';
            if s_zgo = '1' then
                f_run <= '1'; f_ch <= (others => '0'); f_t <= (others => '0');
            elsif f_run = '1' then
                f_t <= f_t + 1;
                -- palette words: ground, A, B, accent for this channel (issued at t 0 .. 3, data at t 3 .. 6)
                if f_t < 4 then
                    v_pa := to_unsigned(C_GB_PAL, 10) + resize(s_pal & "000", 10) + resize(s_pal & "00", 10);
                    v_pc := resize(f_t(1 downto 0), 10);
                    f_ra <= v_pa + shift_left(v_pc, 1) + v_pc + resize(f_ch, 10);
                end if;
                case to_integer(f_t) is
                    when 3 =>
                        case to_integer(f_ch) is
                            when 0      => c_gy <= rom_f(9 downto 0);
                            when 1      => c_gu <= rom_f(9 downto 0);
                            when others => c_gv <= rom_f(9 downto 0);
                        end case;
                    when 4 => f_ca <= rom_f(9 downto 0);
                    when 5 => f_cb <= rom_f(9 downto 0);
                    when 6 => f_cc <= rom_f(9 downto 0);
                    when others => null;
                end case;
                if f_t >= 8 and f_t < 264 then
                    f_v1 <= '1'; f_e1 <= resize(f_t - 8, 8);
                end if;
                if f_t = 272 then
                    f_t <= (others => '0');
                    if f_ch = 2 then f_run <= '0'; else f_ch <= f_ch + 1; end if;
                end if;
            end if;
            -- entry pipeline
            if f_ch = 0 then v_base := to_signed(64, 12); else v_base := to_signed(512, 12); end if;
            if f_e1(7) = '0' then v_df := resize(signed('0' & f_ca), 17) - resize(v_base, 17);
            else v_df := resize(signed('0' & f_cb), 17) - resize(v_base, 17); end if;
            -- 2: the shade accumulator S = (C - base)(level + 1)
            f_v2 <= f_v1; f_e2 <= f_e1;
            if f_v1 = '1' then
                if f_e1(6 downto 0) = "0000000" then f_s <= v_df;
                elsif f_e1(1 downto 0) = "00" then f_s <= f_s + v_df; end if;
            end if;
            -- 3: shade
            f_v3 <= f_v2; f_e3 <= f_e2;
            f_sh3 <= v_base + resize(shift_right(f_s, 5), 12);
            -- 4: d = accent - shade
            f_v4 <= f_v3; f_e4 <= f_e3; f_sh4 <= f_sh3;
            f_dd3 <= signed(resize(f_cc, 12)) - f_sh3;
            -- 5: d * {0, 24, 40, 64}, then the write
            case to_integer(f_e4(1 downto 0)) is
                when 0      => v_d := (others => '0');
                when 1      => v_d := shift_left(resize(f_dd3, 19), 4);
                when 2      => v_d := shift_left(resize(f_dd3, 19), 5);
                when others => v_d := shift_left(resize(f_dd3, 19), 6);
            end case;
            v_m := f_sh4 + resize(shift_right(v_d, 6), 12);
            f_wd <= resize(unsigned(v_m(9 downto 0)), 16); f_wa <= f_e4;
            if f_v4 = '1' then f_we(to_integer(f_ch)) <= '1'; end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- [E] RINGS: per ring (stages 2 .. 6)
    --   2  DDAs: Q (Q24: the quadratic - T_out + W/2), D (its x difference), z (depth, Q11)
    --   3  qm = Q >> 20 (Q4 px, 13 bits), far = |Q| >= 256 px, zi = z >> 11 (12 bits)
    --   4  t = Wh' - |qm| (Wh' = W/2 + the per-field AA offset); rim rq from Kr + qm; zs = clamp(zi >> 7);
    --      ring 0: the hub, u = Kh - qm
    --   5  a = clamp(t), o = clamp(t + 40) (outline 2.5 px), present = o > 0 and not far;
    --      ring 0: the hub's a, o from 4 u + 8
    --   6  level = lvl + zs; ring 0: max with the hub (rq 3 where the hub covers); the rank (p_sel)
    ---------------------------------------------------------------------------
    p_run : process(clk)
    begin
        if rising_edge(clk) then
            -- the DDAs advance for stage-2 pixels 0 .. W + 7
            r_run0 <= g_avid or s_gav_sr(7);
            for r in 0 to NR - 1 loop r_run(r) <= g_avid or s_gav_sr(7); end loop;
        end if;
    end process p_run;

    g_ring : for r in 0 to NR - 1 generate
        p_ring : process(clk)
            variable v_am : unsigned(12 downto 0);
            variable v_r  : signed(9 downto 0);
            variable v_z5 : signed(4 downto 0);
        begin
            if rising_edge(clk) then
                -- 2: the DDAs
                if ld_rq(r) = '1' then p_q(r) <= (others => '0');
                elsif r_run(r) = '1' or ld_ceqd(r) = '1' then p_q(r) <= p_q(r) + p_d(r); end if;
                if ld_rd(r) = '1' then p_d(r) <= (others => '0');
                elsif r_run(r) = '1' or ld_ceqd(r) = '1' then p_d(r) <= p_d(r) + c_k2a(r); end if;
                if ld_k2a(r) = '1' then c_k2a(r) <= rf_e(43 downto 0); end if;
                if ld_rz(r) = '1' then p_z(r) <= (others => '0');
                elsif r_run(r) = '1' or ld_cez(r) = '1' then p_z(r) <= p_z(r) + c_kz(r); end if;
                if ld_kz(r) = '1' then c_kz(r) <= rf_e(23 downto 0); end if;
                -- 3
                if p_q(r)(43 downto 32) = (43 downto 32 => p_q(r)(32)) then s3_far(r) <= '0'; else s3_far(r) <= '1'; end if;
                s3_qm(r) <= p_q(r)(32 downto 20);
                s3_zi(r) <= p_z(r)(22 downto 13);
                -- 4
                if s3_qm(r)(12) = '1' then v_am := unsigned(not s3_qm(r)); else v_am := unsigned(s3_qm(r)); end if;
                s4_t(r) <= signed(resize(c_wh(r), 14)) - signed(resize(v_am, 14));
                s4_qs(r) <= s3_qm(r)(12);
                v_z5 := s3_zi(r)(9 downto 5);
                if v_z5 = -16 then s4_zs(r) <= to_signed(-15, 5); else s4_zs(r) <= v_z5; end if;
                s4_far(r) <= s3_far(r);
                -- 5
                s5_a(r) <= f_clamp16(s4_t(r));
                s5_o(r) <= f_out16(s4_t(r));
                -- the rim on the outer edge (qm >= 0): r = rim + 8 - t, narrow (t >= 256 is no rim)
                v_r := signed(resize(c_wg, 10) + 8) - signed(resize(unsigned(s4_t(r)(7 downto 0)), 10));
                if s4_qs(r) = '1' then s5_rq(r) <= "00";
                elsif s4_t(r)(13) = '1' then s5_rq(r) <= "11";
                elsif s4_t(r)(12 downto 8) /= 0 then s5_rq(r) <= "00";
                elsif v_r(9) = '1' then s5_rq(r) <= "00";
                elsif v_r(8 downto 4) /= 0 or v_r(3 downto 2) = "11" then s5_rq(r) <= "11";
                else s5_rq(r) <= unsigned(v_r(3 downto 2)); end if;
                s5_zs(r) <= s4_zs(r);
                -- 6
                s6_zs(r) <= s5_zs(r);
            end if;
        end process p_ring;
        -- stage 6 payload (separate processes: one driver per element)
        g_s60 : if r = 0 generate
            p_s60 : process(clk)
            begin
                if rising_edge(clk) then              -- ring 0 = max(band, hub); the hub is accent
                    if s5_ah > s5_a(0) then s6_a(0) <= s5_ah; s6_rq(0) <= "11";
                    else s6_a(0) <= s5_a(0); s6_rq(0) <= s5_rq(0); end if;
                    if s5_oh > s5_o(0) then s6_o(0) <= s5_oh; else s6_o(0) <= s5_o(0); end if;
                end if;
            end process p_s60;
        end generate g_s60;
        g_s6 : if r /= 0 generate
            p_s6 : process(clk)
            begin
                if rising_edge(clk) then
                    s6_a(r) <= s5_a(r); s6_o(r) <= s5_o(r); s6_rq(r) <= s5_rq(r);
                end if;
            end process p_s6;
        end generate g_s6;
    end generate g_ring;

    -- presence (stage 5) for every ring; ring 0 also carries the hub disc (accent), stage 6
    p_pres : process(clk)
        variable v_h  : signed(15 downto 0);
        variable v_p  : std_logic;
    begin
        if rising_edge(clk) then
            s4_u <= c_kh - resize(s3_qm(0), 14);
            v_h := shift_left(resize(s4_u, 16), 2) + 8;
            s5_ah <= f_clamp16(v_h);
            s5_oh <= f_out16(v_h);
            for r in 0 to NR - 1 loop
                v_p := f_in48(s4_t(r));
                if r = 0 and f_in48(v_h) = '1' then v_p := '1'; end if;
                s5_pres(r) <= v_p and not s4_far(r);
            end loop;
        end if;
    end process p_pres;

    -- pairwise depth order (stage 4) and its delay to stage 5
    p_ge : process(clk)
        variable v_i : integer;
    begin
        if rising_edge(clk) then
            for k in 1 to NR - 1 loop
                for j in 0 to k - 1 loop
                    v_i := k * (k - 1) / 2 + j;
                    if s3_zi(k) >= s3_zi(j) then s4_ge(v_i) <= '1'; else s4_ge(v_i) <= '0'; end if;
                end loop;
            end loop;
            s5_ge <= s4_ge;
        end if;
    end process p_ge;

    ---------------------------------------------------------------------------
    -- [E] SELECTION (stages 6 .. 11): F = the present ring nothing present is in
    -- front of, B = the present ring exactly one is in front of; X = F where its
    -- own material covers, else B through F's outline (w = min(16 - oF, aB));
    -- the colour-table address
    ---------------------------------------------------------------------------
    p_sel : process(clk)
        variable v_f : std_logic;
        variable v_cnt : integer range 0 to 7;
        variable v_af, v_of, v_ab : unsigned(4 downto 0);
        variable v_rqf, v_rqb : unsigned(1 downto 0);
        variable v_lvf, v_lvb : unsigned(4 downto 0);
        variable v_zsf, v_zsb : signed(4 downto 0);
        variable v_fcf, v_fcb : std_logic;
        variable v_lv : signed(6 downto 0);
        variable v_m : unsigned(4 downto 0);
        variable v_rq : unsigned(1 downto 0);
        variable v_lq : unsigned(1 downto 0);
        variable v_pr : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- 6: rank
            for k in 0 to NR - 1 loop
                v_cnt := 0;
                for j in 0 to NR - 1 loop
                    if j /= k then
                        if j > k then v_f := s5_pres(j) and s5_ge(j * (j - 1) / 2 + k);
                        else v_f := s5_pres(j) and not s5_ge(k * (k - 1) / 2 + j); end if;
                        if v_f = '1' then v_cnt := v_cnt + 1; end if;
                    end if;
                end loop;
                if v_cnt = 0 then s6_isf(k) <= s5_pres(k); else s6_isf(k) <= '0'; end if;
                if v_cnt = 1 then s6_isb(k) <= s5_pres(k); else s6_isb(k) <= '0'; end if;
            end loop;
            -- 7: select (one-hot AND-OR)
            v_af := (others => '0'); v_of := (others => '0'); v_ab := (others => '0');
            v_rqf := (others => '0'); v_rqb := (others => '0');
            v_lvf := (others => '0'); v_lvb := (others => '0'); v_zsf := (others => '0'); v_zsb := (others => '0');
            v_fcf := '0'; v_fcb := '0';
            for k in 0 to NR - 1 loop
                if s6_isf(k) = '1' then
                    v_af := v_af or s6_a(k); v_of := v_of or s6_o(k); v_rqf := v_rqf or s6_rq(k);
                    v_lvf := v_lvf or c_lvl(k); v_zsf := v_zsf or s6_zs(k); v_fcf := v_fcf or c_face(k);
                end if;
                if s6_isb(k) = '1' then
                    v_ab := v_ab or s6_a(k); v_rqb := v_rqb or s6_rq(k);
                    v_lvb := v_lvb or c_lvl(k); v_zsb := v_zsb or s6_zs(k); v_fcb := v_fcb or c_face(k);
                end if;
            end loop;
            s7_af <= v_af; s7_of <= v_of; s7_ab <= v_ab; s7_rqf <= v_rqf; s7_rqb <= v_rqb;
            s7_lvf <= v_lvf; s7_lvb <= v_lvb; s7_zsf <= v_zsf; s7_zsb <= v_zsb; s7_fcf <= v_fcf; s7_fcb <= v_fcb;
            -- 8
            s8_af <= s7_af; s8_of <= s7_of; s8_rqf <= s7_rqf; s8_fcf <= s7_fcf;
            s8_ab <= s7_ab; s8_rqb <= s7_rqb; s8_fcb <= s7_fcb;
            s8_lvf <= signed(resize(s7_lvf, 7)) + resize(s7_zsf, 7);     -- level = brightness + depth shade
            s8_lvb <= signed(resize(s7_lvb, 7)) + resize(s7_zsb, 7);
            -- 9: X = F or B, its coverage, level (clamped), rim
            if s8_af /= 0 then
                s9_w <= s8_af; v_lv := s8_lvf; s9_rq <= s8_rqf; s9_fc <= s8_fcf;
            else
                v_m := to_unsigned(16, 5) - s8_of;
                if v_m < s8_ab then s9_w <= v_m; else s9_w <= s8_ab; end if;
                v_lv := s8_lvb; s9_rq <= s8_rqb; s9_fc <= s8_fcb;
            end if;
            if v_lv(6) = '1' then s9_lv <= (others => '0');
            elsif v_lv(5) = '1' then s9_lv <= to_unsigned(31, 5);
            else s9_lv <= unsigned(v_lv(4 downto 0)); end if;
            -- 10: the camera tint from the luma (stage 9)
            v_lq := s9_l(7 downto 6);
            v_pr := (v_lq * c_kc) + 2;
            s10_rqc <= v_pr(3 downto 2);
            s10_rq <= s9_rq; s10_lv <= s9_lv; s10_fc <= s9_fc; s10_w <= s9_w;
            -- 11: table address
            if s10_rq > s10_rqc then v_rq := s10_rq; else v_rq := s10_rqc; end if;
            ct_a <= s10_fc & s10_lv & v_rq;
            s11_w <= s10_w;
            -- 12 (block RAM), 13 fabric
            s12_w <= s11_w;
            s13_y <= ct_qy(9 downto 0); s13_u <= ct_qu(9 downto 0); s13_v <= ct_qv(9 downto 0);
            s13_w <= s12_w;
        end if;
    end process p_sel;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD (riley): luma (or the Osc lattice) -> 32-px box ->
    -- vertical 1/8 IIR in the line store; the pixel path reads it at stage 9.
    -- Camera samples from the stage-9 luma: the centre line every 32 virtual px
    -- (0 .. 59), the centre column every 32 virtual lines (64 .. 97).
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_d : signed(9 downto 0);
        variable v_a : unsigned(8 downto 0);
        variable v_hc : std_logic;
    begin
        if rising_edge(clk) then
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;

            v_l := unsigned(data_in.y(9 downto 2));
            if v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid; s_svd <= s_sv;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            if s_sv = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(225, 8); end if;
            if s_bcnt < 32 then v_tap := s_sfirst; else v_tap := bx_q; end if;
            if s_sv = '1' and s_svd = '0' then
                s_sfirst <= s_sl;
                s_bsum <= shift_left(resize(s_sl, 13), 5);
                s_bcnt <= to_unsigned(1, 6);
            elsif s_sv = '1' then
                s_bsum <= s_bsum + s_sl - v_tap;
                if s_bcnt /= 32 then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bv <= s_sv; s_bcol <= s_col;

            s_eb8 <= s_bsum(12 downto 5); s_eq <= lb_q1;
            s_ev <= s_bv and s_bcol(0); s_ecol <= s_bcol;
            v_d := signed(resize(s_eb8, 10)) - signed(resize(s_eq, 10)) + 4;
            v_a := unsigned(signed(resize(s_eq, 9)) + resize(shift_right(v_d, 3), 9));
            lb_wd <= v_a(7 downto 0);
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            if s_st(3) = '1' or s_st(4) = '1' then lb_rc <= (others => '0'); else lb_rc <= lb_rc + 1; end if;
            s8_q <= lb_q2; s8_qp <= s8_q;
            v_a := resize(s8_q, 9) + resize(s8_qp, 9) + 1;
            -- Source = Osc: a 512 x 512 virtual-px triangle lattice drifting diagonally, injected after the
            -- smoothing (stage-8 beam x, the line's vy)
            if s_st(6) = '1' then
                case to_integer(s_xs) is
                    when 4      => o_x8 <= to_unsigned(4 * 240 + 4, 11);
                    when 3      => o_x8 <= to_unsigned(3, 11);
                    when others => o_x8 <= to_unsigned(2, 11);
                end case;
            else
                o_x8 <= o_x8 + resize(s_xs & '0', 11);
            end if;
            v_s := resize(f_tri7(o_x8(10 downto 3) + resize(s_ot(7 downto 1), 8)), 8) + resize(f_tri7(s_opy), 8);
            if s_osc = '1' then s9_l <= v_s - shift_right(v_s, 3); else s9_l <= v_a(8 downto 1); end if;

            -- camera samples
            -- vy = 256 (y + 540) + 123904 per line: the screen centre at 2^18, the column grid centred
            if s_zgo = '1' then
                s_vy <= to_unsigned(123904, 20) + shift_right(resize(s_ys, 20), 1) - resize(s_ys, 20);
                s_wbank <= not s_wbank;
            elsif g_start = '1' then
                s_vyp <= s_vy;
                s_vy <= s_vy + resize(s_ys, 20);
            end if;
            if s_st(0) = '1' then                     -- this line's flags (vy is the line's own now)
                s_opy <= s_vy(16 downto 9) + resize(s_ot(7 downto 2), 8) + resize(s_ot(7 downto 3), 8);
                s_cl <= s_vy(18) and not s_vyp(18);
                if s_vy(18 downto 13) /= s_vyp(18 downto 13) then s_vl <= '1'; else s_vl <= '0'; end if;
                s_vi <= s_vy(18 downto 13);
            end if;
            if s_st(7) = '1' then
                if s_xs = 4 then
                    s_bx  <= to_unsigned(4 * 240 + 4, 14);
                    s_bxp <= to_unsigned(4 * 240 + 4 - 8, 14);
                elsif s_xs = 3 then
                    s_bx  <= to_unsigned(3, 14);
                    s_bxp <= to_unsigned(16384 - 3, 14);
                else
                    s_bx  <= to_unsigned(2, 14);
                    s_bxp <= to_unsigned(16384 - 2, 14);
                end if;
            else
                s_bx <= s_bx + resize(s_xs & '0', 14);
                s_bxp <= s_bx;
            end if;
            sp_we <= '0';
            if s_bx(13 downto 7) /= s_bxp(13 downto 7) then v_hc := '1'; else v_hc := '0'; end if;
            s_vcap <= '0';
            if v_hc = '1' and s_gav_sr(7) = '1' and s_cl = '1' then
                sp_we <= '1'; sp_wa <= s_wbank & '0' & s_bx(12 downto 7); sp_wd <= s9_l;
            end if;
            if v_hc = '1' and s_bx(13 downto 7) = 30 and s_vl = '1' then
                s_vcap <= '1'; s_vcl <= s9_l;
            end if;
            if s_vcap = '1' then
                sp_we <= '1'; sp_wa <= s_wbank & '1' & s_vi; sp_wd <= s_vcl;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] BLEND (stages 14 .. 17): ground + (colour - ground) * w / 16
    ---------------------------------------------------------------------------
    p_blend : process(clk)
        variable v_g : signed(15 downto 0);
        variable v_win : std_logic;
    begin
        if rising_edge(clk) then
            -- 14
            s14_dy <= signed(resize(s13_y, 11)) - signed(resize(c_gy, 11));
            s14_du <= signed(resize(s13_u, 11)) - signed(resize(c_gu, 11));
            s14_dv <= signed(resize(s13_v, 11)) - signed(resize(c_gv, 11));
            s14_w <= s13_w;
            -- 15: difference * coverage as two sums of gated shifts
            if s14_w(4) = '1' then
                s15_ay <= shift_left(resize(s14_dy, 16), 4); s15_by <= (others => '0');
                s15_au <= shift_left(resize(s14_du, 16), 4); s15_bu <= (others => '0');
                s15_av <= shift_left(resize(s14_dv, 16), 4); s15_bv <= (others => '0');
            else
                if s14_w(3) = '1' then v_g := shift_left(resize(s14_dy, 16), 3); else v_g := (others => '0'); end if;
                if s14_w(2) = '1' then s15_ay <= v_g + shift_left(resize(s14_dy, 16), 2); else s15_ay <= v_g; end if;
                if s14_w(1) = '1' then v_g := shift_left(resize(s14_dy, 16), 1); else v_g := (others => '0'); end if;
                if s14_w(0) = '1' then s15_by <= v_g + resize(s14_dy, 16); else s15_by <= v_g; end if;
                if s14_w(3) = '1' then v_g := shift_left(resize(s14_du, 16), 3); else v_g := (others => '0'); end if;
                if s14_w(2) = '1' then s15_au <= v_g + shift_left(resize(s14_du, 16), 2); else s15_au <= v_g; end if;
                if s14_w(1) = '1' then v_g := shift_left(resize(s14_du, 16), 1); else v_g := (others => '0'); end if;
                if s14_w(0) = '1' then s15_bu <= v_g + resize(s14_du, 16); else s15_bu <= v_g; end if;
                if s14_w(3) = '1' then v_g := shift_left(resize(s14_dv, 16), 3); else v_g := (others => '0'); end if;
                if s14_w(2) = '1' then s15_av <= v_g + shift_left(resize(s14_dv, 16), 2); else s15_av <= v_g; end if;
                if s14_w(1) = '1' then v_g := shift_left(resize(s14_dv, 16), 1); else v_g := (others => '0'); end if;
                if s14_w(0) = '1' then s15_bv <= v_g + resize(s14_dv, 16); else s15_bv <= v_g; end if;
            end if;
            -- 16
            s16_y <= s15_ay + s15_by;
            s16_u <= s15_au + s15_bu;
            s16_v <= s15_av + s15_bv;
            -- 17: ground + (sum + 8) / 16; the ground colour outside pixels 0 .. W + 7
            v_win := s_gav_sr(14) or s_gav_sr(22);
            if v_win = '1' then
                v_g := signed(resize(c_gy & "1000", 16)) + s16_y; s17_y <= unsigned(v_g(13 downto 4));
                v_g := signed(resize(c_gu & "1000", 16)) + s16_u; s17_u <= unsigned(v_g(13 downto 4));
                v_g := signed(resize(c_gv & "1000", 16)) + s16_v; s17_v <= unsigned(v_g(13 downto 4));
            else
                s17_y <= c_gy; s17_u <= c_gu; s17_v <= c_gv;
            end if;
            s_hist <= s_hist(6 downto 0) & v_win;
        end if;
    end process p_blend;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING (stages 18 .. 21, riley; the 8-back chroma tap
    -- comes from the block-RAM rings p_cru / p_crv)
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            cr_wp <= cr_wp + 1;
            cr_fu <= cr_qu; cr_fv <= cr_qv;
            if s_hist = "00000000" then
                s_bu <= (c_gu & "000") + 4; s_bv2 <= (c_gv & "000") + 4;
            else
                s_bu  <= s_bu + s17_u - cr_fu(9 downto 0);
                s_bv2 <= s_bv2 + s17_v - cr_fv(9 downto 0);
            end if;
            s_bud <= s_bu; s_bvd <= s_bv2;
            s_yd(1) <= s17_y;
            for i in 2 to 5 loop s_yd(i) <= s_yd(i - 1); end loop;
            s18_ya <= resize(s_yd(3), 12) + resize(s_yd(5), 12) + 2;
            s18_yb <= s_yd(4);
            v_sc := resize(s_bu, 14) + s_bud;  s19_u <= v_sc(13 downto 4);
            v_sc := resize(s_bv2, 14) + s_bvd; s19_v <= v_sc(13 downto 4);
            v_y := s18_ya + shift_left(resize(s18_yb, 12), 1);
            s19_y <= v_y(11 downto 2);
            if s19_y > 64 then v_dl := s19_y - 64; else v_dl := (others => '0'); end if;
            s20_y <= s19_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s20_u <= s19_u; s20_v <= s19_v;
            if s_gav_sr(C_LAT - 2) = '1' then
                s21_y <= s20_y; s21_u <= s20_u; s21_v <= s20_v;
            else
                s21_y <= to_unsigned(64, 10); s21_u <= to_unsigned(512, 10); s21_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s21_y);
    data_out.u       <= std_logic_vector(s21_u);
    data_out.v       <= std_logic_vector(s21_v);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture gimbal;
