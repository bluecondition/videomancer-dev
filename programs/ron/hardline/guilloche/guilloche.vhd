-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Guilloche v0.1.0 -- Hardline.  The engine-turned lace of banknotes, share certificates and watch dials:
-- many copies of one wave, each shifted a step in phase, overlapping into a woven band.  Black and white
-- only: U = V = 512 on every pixel.
--
-- In a band with centre line c and amplitude A, s = (c - y) / A.  A single-humped cam w(t) (crest at t = 0,
-- symmetric) exceeds the level s on |t| < h(s).  Of M copies shifted by k / M, the number lying above the pixel
-- is floor(M (t + h)) - floor(M (t - h)), so the parity fill is
--
--     stripe(P + H) xor stripe(P - H),      P = M t (a phase DDA along the line), H = M h(s) (per line)
--
-- Three laces per band (Triple): (copies, wavelength, amplitude, drift) = (M, L, A, +1), (M2, 2L, 0.62 A, -3/4),
-- (M2, 4L, 0.34 A, +2/3) with M2 about M / 2; xor of the three.  The camera displaces all three by the same
-- distance (c0 cycles of the base lace), so the band warps as one engraving.
--
--   K1  X        slides every lace along its band (a pan)
--   K2  Y        moves the stack of bands up and down
--   K3  Period   the wavelength, 72 .. 1440 virtual px (exponential)
--   K4  Lines    M = 2, 3, 4, 6, 8, 12, 16, 24 copies (stepped)
--   K5  Drift    bipolar speed: the laces slide, alternate bands opposite ways, the three laces at 1, -3/4, 2/3
--   K6  Camera   how far the picture's luma pushes the laces along their bands (0 = none)
--   S7  Glide    Off = the knob values of this field start are used as they are / On = eased
--   S8  Form     Bands (stacked borders) / Sheet (one lace as tall as the screen)
--   S9  Layers   Single / Triple
--   S10 Weight   Bold (parity fill) / Fine (the curves themselves as thin lines: the engraving)
--   S11 Source   Video / Osc
--   P12 CAM      the wave shape: 0 = triangle, 50 % = sine, 100 % = cycloid with the cusps up
--
-- The chassis is the Kinetic riley shell (via hardline/tweed):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         +-1-count holds, glides, stepped knob, switches
--   [C] ENGINE                           per-field sequencer (one serial multiplier), drift, the line engine
--                                        (a second serial multiplier, the cam ROMs), beam DDAs
--   [D] SOURCE FIELD                     luma (or the Osc lattice) x camera gain -> box -> vertical IIR
--   [E] LACE                             per lace: phase, footprint, coverage, soft xor, fade; laces, frame
--   [F] OUTPUT CONDITIONING              luma [1 2 1]/4, 13/16 toward black, blanking gate
--
-- Stage numbering: stage 1 is the generated avid's own clock; a register "at stage S" holds pixel n at
-- T + S + n (T = the g_start clock).  Pixel n sits at x = x0 + (n + 1) xs half virtual px.  The output register
-- is at stage C_LAT - 3; output column j shows pixel j + 4 on screen column j + 4.  One latency (36) in every
-- mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.guilloche_pkg.all;
use work.all;

architecture guilloche of program_top is
    constant C_LAT : integer := 36;   -- = processing_delay_clks

    -- TOML defaults: the glides and holds start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 560;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 512;
    constant C_DEF_K5  : integer := 680;

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
    -- 16 * 65536 / ys: the rule coverage of a line
    function f_invys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(921, 13);
            when 4          => return to_unsigned(1820, 13);
            when 8          => return to_unsigned(1092, 13);
            when 12         => return to_unsigned(2185, 13);
            when 5 | 6 | 14 => return to_unsigned(2731, 13);
            when 1 | 2 | 10 => return to_unsigned(2048, 13);
            when others     => return to_unsigned(4096, 13);
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
    -- the fade toward grey in quarters (0 .. 4) from the footprint reciprocal r = rr x 2^(12 - t) (Q4.4 px):
    -- 0 from r >= 2 px (stripes of 4 px and more), (32 - r) / 4 between, 4 at 1 px and under
    function f_fade(rr : unsigned(4 downto 0); t : unsigned(3 downto 0)) return unsigned is
        variable v_r : unsigned(5 downto 0);
    begin
        case to_integer(t) is
            when 0 to 10 => return "000";
            when 11      => v_r := rr & '0';
            when 12      => v_r := '0' & rr;
            when others  => return "100";
        end case;
        if v_r >= 32 then return "000";
        elsif v_r <= 16 then return "100";
        else
            v_r := to_unsigned(32, 6) - v_r;
            return v_r(4 downto 2);
        end if;
    end function;

    attribute no_rw_check : boolean;

    type t_u16x3 is array (0 to 2) of unsigned(15 downto 0);
    type t_u14x3 is array (0 to 2) of unsigned(13 downto 0);
    type t_u12x3 is array (0 to 2) of unsigned(11 downto 0);
    type t_u22x3 is array (0 to 2) of unsigned(21 downto 0);
    type t_u21x3 is array (0 to 2) of unsigned(20 downto 0);
    type t_u20x3 is array (0 to 2) of unsigned(19 downto 0);
    type t_u8x3  is array (0 to 2) of unsigned(7 downto 0);
    type t_u7x3  is array (0 to 2) of unsigned(6 downto 0);
    type t_u6x3  is array (0 to 2) of unsigned(5 downto 0);
    type t_u5x3  is array (0 to 2) of unsigned(4 downto 0);
    type t_u4x3  is array (0 to 2) of unsigned(3 downto 0);
    type t_u3x3  is array (0 to 2) of unsigned(2 downto 0);
    type t_u15a  is array (0 to 2) of unsigned(14 downto 0);

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
    signal s_st       : std_logic_vector(0 to 9) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] knobs: +-1-count holds, glide accumulators (Q10.6), stepped knob, switches
    signal s_hx : unsigned(9 downto 0) := to_unsigned(C_DEF_K1, 10);
    signal s_hy : unsigned(9 downto 0) := to_unsigned(C_DEF_K2, 10);
    signal s_hp : unsigned(9 downto 0) := to_unsigned(C_DEF_K3, 10);
    signal s_hc : unsigned(9 downto 0) := to_unsigned(C_DEF_K6, 10);
    signal s_ht : unsigned(9 downto 0) := to_unsigned(C_DEF_P12, 10);
    signal s_hv : unsigned(9 downto 0) := to_unsigned(C_DEF_K5, 10);
    signal s_ax : unsigned(15 downto 0) := to_unsigned(C_DEF_K1 * 64, 16);
    signal s_ay : unsigned(15 downto 0) := to_unsigned(C_DEF_K2 * 64, 16);
    signal s_ap : unsigned(15 downto 0) := to_unsigned(C_DEF_K3 * 64, 16);
    signal s_ac : unsigned(15 downto 0) := to_unsigned(C_DEF_K6 * 64, 16);
    signal s_at : unsigned(15 downto 0) := to_unsigned(C_DEF_P12 * 64, 16);
    signal s_av : unsigned(15 downto 0) := to_unsigned(C_DEF_K5 * 64, 16);
    signal s_gm : unsigned(15 downto 0) := (others => '0');
    signal s_gr, s_gh, s_gt : unsigned(9 downto 0) := (others => '0');
    signal s_hd : signed(10 downto 0) := (others => '0');
    signal s_gd : signed(16 downto 0) := (others => '0');
    signal z_gp : unsigned(2 downto 0) := (others => '0');
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_mi : unsigned(2 downto 0) := to_unsigned(2, 3);     -- Lines: M = 4
    signal s_glide : std_logic := '1';
    signal s_tri   : std_logic := '1';
    signal s_form, s_fine, s_src : std_logic := '0';

    -- [C] ROM: the exponential
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');

    -- [C] per-field serial multiplier, slot counter and scratch
    signal f_a, f_acc, z_fa : unsigned(39 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_ph   : unsigned(4 downto 0) := (others => '0');
    signal z_slot : unsigned(4 downto 0) := (others => '0');
    signal z_run, z_pre, z_ld : std_logic := '0';
    signal z_oct  : unsigned(10 downto 0) := (others => '0');
    signal z_vneg, z_vzero : std_logic := '1';
    signal z_va   : unsigned(9 downto 0) := (others => '0');
    signal z_u    : unsigned(9 downto 0) := (others => '0');
    signal z_ls   : unsigned(9 downto 0) := (others => '0');
    signal z_mxneg : std_logic := '0';
    signal z_mxmag : unsigned(11 downto 0) := (others => '0');
    signal z_d1, z_d2 : unsigned(19 downto 0) := (others => '0');
    signal z_t1   : unsigned(17 downto 0) := (others => '0');
    signal s_spd  : unsigned(19 downto 0) := (others => '0');
    signal s_ssh  : unsigned(3 downto 0) := (others => '0');

    -- [C] per-field constants
    signal c_sh    : unsigned(2 downto 0) := (others => '0');
    signal c_invL  : unsigned(17 downto 0) := (others => '0');     -- cycles per virtual px, Q24
    signal c_K     : unsigned(22 downto 0) := (others => '0');     -- this lace's phase per virtual px, Q24
    signal c_dP    : t_u20x3 := (others => (others => '0'));       -- phase per pixel, Q20
    signal c_B     : t_u21x3 := (others => (others => '0'));       -- the pan phase at the pre-pixel, Q1.20
    signal s_T     : t_u20x3 := (others => (others => '0'));       -- drift: copy-phase accumulators, Q0.20
    signal c_mq    : t_u7x3 := (others => (others => '0'));
    signal c_mc    : t_u5x3 := (others => (others => '0'));        -- copies per lace
    signal c_dr1, c_dr2 : unsigned(15 downto 0) := (others => '0');
    signal c_rq    : std_logic_vector(0 to 2) := (others => '0');
    signal c_ks    : t_u3x3 := (others => (others => '0'));        -- rho's power of two + 1 (camera phase shift)
    signal c_toff  : t_u5x3 := (others => (others => '0'));        -- 13 + k + 2q (the footprint exponent offset)
    signal c_gain  : unsigned(7 downto 0) := (others => '0');      -- camera gain
    signal c_cmode : unsigned(11 downto 0) := to_unsigned(1828, 12);
    signal c_gx0   : unsigned(14 downto 0) := (others => '0');     -- base cycles per pixel, Q20
    signal c_hv    : unsigned(4 downto 0) := to_unsigned(16, 5);   -- Fine: 1 virtual px in px, Q4.4
    signal c_upper : std_logic := '0';
    signal c_w     : unsigned(10 downto 0) := (others => '0');     -- P12 blend weight, Q11
    signal c_pitch, c_half, c_a0, c_rcen : unsigned(18 downto 0) := (others => '0');
    signal c_rh    : unsigned(11 downto 0) := (others => '0');     -- rule half-width + half a line (Q8)
    signal c_2hy   : unsigned(11 downto 0) := (others => '0');     -- a line, Q8 (even)
    signal c_inva  : t_u15a := (others => (others => '0'));
    signal c_invys : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal c_y8    : signed(19 downto 0) := (others => '0');

    -- [C] line engine (computes the next line during this one)
    signal e_run   : std_logic := '0';
    signal e_cnt   : unsigned(1 downto 0) := (others => '0');
    signal e_blk   : unsigned(1 downto 0) := (others => '0');
    signal e_pc    : unsigned(6 downto 0) := (others => '0');
    signal e_v     : signed(20 downto 0) := (others => '0');
    signal e_par, e_vpos : std_logic := '0';
    signal e_dv    : signed(20 downto 0) := (others => '0');
    signal c_v0    : signed(20 downto 0) := (others => '0');       -- line -2's centre without Y (Q8)
    signal c_w2    : unsigned(11 downto 0) := (others => '0');     -- 2048 - w
    signal m_keep  : std_logic := '0';
    signal e_av    : unsigned(18 downto 0) := (others => '0');
    signal e_dq    : unsigned(18 downto 0) := (others => '0');
    signal e_ov    : unsigned(11 downto 0) := (others => '0');
    signal e_u     : unsigned(16 downto 0) := (others => '0');
    signal e_ra    : unsigned(9 downto 0) := (others => '0');
    signal rs_q, rs_f, rc_q, rc_f : unsigned(15 downto 0) := (others => '0');
    signal e_hb, e_h : unsigned(15 downto 0) := (others => '0');
    signal e_hp    : t_u12x3 := (others => (others => '0'));
    signal m_a, m_acc : unsigned(33 downto 0) := (others => '0');
    signal m_b     : unsigned(15 downto 0) := (others => '0');
    signal m_ld    : std_logic := '0';
    signal m_la    : unsigned(17 downto 0) := (others => '0');
    signal m_lb    : unsigned(15 downto 0) := (others => '0');
    signal n_Hm, cur_Hm : t_u14x3 := (others => (others => '0'));
    signal n_gyq, cur_gyq : t_u12x3 := (others => (others => '0'));
    signal n_ins, cur_ins : std_logic_vector(0 to 2) := (others => '0');
    signal n_par, n_in0, cur_in0, cur_par : std_logic := '0';
    signal n_rc, cur_rc : unsigned(4 downto 0) := (others => '0');
    signal b_P     : t_u21x3 := (others => (others => '0'));

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl  : unsigned(5 downto 0) := (others => '0');
    signal s_avd : std_logic_vector(1 to 5) := (others => '0');    -- data_in.avid delayed
    signal s_col, s_pcol, s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal s_p, s_pfirst : unsigned(13 downto 0) := (others => '0');
    signal c_bn  : unsigned(6 downto 0) := to_unsigned(96, 7);     -- box length: 96 / 64 / 48 pixels = 96 virtual px
    signal c_brp : unsigned(7 downto 0) := to_unsigned(161, 8);
    signal c_iir : unsigned(2 downto 0) := to_unsigned(5, 3);
    type t_bx is array (0 to 255) of unsigned(15 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(15 downto 0) := (others => '0');
    signal s_bcnt : unsigned(6 downto 0) := (others => '0');
    signal s_bsum : unsigned(18 downto 0) := (others => '0');
    signal s_ev, s_fl : std_logic := '0';
    signal s_eb, s_eq : unsigned(15 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal lb_dd : signed(13 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal h_cur, h_prev, s8_q : unsigned(15 downto 0) := (others => '0');
    signal h_dy, s8_dy : signed(13 downto 0) := (others => '0');
    signal h_ph : std_logic := '0';
    signal s9_D, s9_D3 : unsigned(15 downto 0) := (others => '0');
    signal s9_Dy : signed(13 downto 0) := (others => '0');

    -- [E] lace: footprint (stages 10 .. 16), phase (12 .. 16), distance (17, 18)
    signal s10_D  : unsigned(15 downto 0) := (others => '0');
    signal s10_dx : signed(15 downto 0) := (others => '0');
    signal s10_dy : signed(13 downto 0) := (others => '0');
    signal s11_bx : signed(24 downto 0) := (others => '0');
    signal s11_ady : unsigned(21 downto 0) := (others => '0');
    signal s12_base : unsigned(21 downto 0) := (others => '0');
    signal s13_w : t_u22x3 := (others => (others => '0'));
    signal s13_Pc, s14_P1, s14_P2 : t_u14x3 := (others => (others => '0'));
    signal s15_dm : t_u12x3 := (others => (others => '0'));
    signal s15_p, s16_p, s17_p, s18_p : std_logic_vector(0 to 2) := (others => '0');
    signal s14_n : t_u8x3 := (others => (others => '0'));    -- bits 21 .. 14 after the 8 / 4 shifts
    signal s14_sh : t_u4x3 := (others => (others => '0'));
    signal s15_t : t_u4x3 := (others => (others => '0'));
    signal s15_rr, s16_rr : t_u5x3 := (others => (others => '0'));
    signal s16_dsh : t_u6x3 := (others => (others => '0'));
    signal s16_f, s17_f, s18_f : t_u3x3 := (others => (others => '0'));
    signal s17_dpx, s18_dpx : t_u7x3 := (others => (others => '0'));
    -- all laces: nearest edge, parity, fade (18 .. 23), frame (24), luma (25 .. 27)
    signal s19_d, s19_d2, s20_d : unsigned(6 downto 0) := (others => '0');
    signal s19_p, s19_p2, s20_p : std_logic := '0';
    signal s19_f, s19_f2, s20_f, s21_f : unsigned(2 downto 0) := (others => '0');
    signal s21_x, s22_x, s23_v, s24_o : unsigned(4 downto 0) := (others => '0');
    signal s22_pf : signed(7 downto 0) := (others => '0');
    signal s25_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s26_y, s27_y : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- [F] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_yd : t_u10(1 to 3) := (others => to_unsigned(64, 10));
    signal s34_ya : unsigned(11 downto 0) := (others => '0');
    signal s34_yb : unsigned(9 downto 0) := (others => '0');
    signal s35_y, s36_y, s37_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_GU_EXP(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            rs_q   <= C_GU_SIN(to_integer(e_ra));
            rs_f   <= rs_q;
            rc_q   <= C_GU_CYC(to_integer(e_ra));
            rc_f   <= rc_q;
        end if;
    end process p_rom;

    -- box delay: read (box length) samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_avd(2) = '1' then bx_ram(to_integer(bx_wp)) <= resize(s_p, 16); end if;
        end if;
    end process p_bxram;

    -- camera-field line store (half horizontal resolution): read 3 pixels ahead of the write
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_pcol(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    -- [A] the beam's pre-pixel by video mode
    w_dxm <= to_unsigned(1444, 11) when s_xs = 4 else to_unsigned(1923, 11) when s_xs = 3 else to_unsigned(1922, 11);

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + [C] the per-field
    -- sequencer (one small operation per zone clock; the zone is quasi-static
    -- but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k   : unsigned(9 downto 0);
        variable v_e   : signed(11 downto 0);
        variable v_st  : signed(16 downto 0);
        variable v_11, v_11n : signed(10 downto 0);
        variable v_x   : signed(16 downto 0);
        variable v_mx, v_mxn : signed(13 downto 0);
        variable v_21  : unsigned(20 downto 0);
        variable v_i   : integer range 0 to 2;
        variable v_mi  : integer range 0 to 7;
        variable v_fm  : integer range 0 to 1;
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
                s_form  <= registers_in(6)(1);
                s_tri   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is                  -- Osc step, box length (96 virtual px), its read offset
                    when 4      => s_oxs <= to_unsigned(204, 9); c_bn <= to_unsigned(48, 7); c_brp <= to_unsigned(209, 8);
                                   c_cmode <= to_unsigned(3656, 12); c_hv <= to_unsigned(8, 5);
                    when 3      => s_oxs <= to_unsigned(153, 9); c_bn <= to_unsigned(64, 7); c_brp <= to_unsigned(193, 8);
                                   c_cmode <= to_unsigned(2742, 12); c_hv <= to_unsigned(11, 5);
                    when others => s_oxs <= to_unsigned(102, 9); c_bn <= to_unsigned(96, 7); c_brp <= to_unsigned(161, 8);
                                   c_cmode <= to_unsigned(1828, 12); c_hv <= to_unsigned(16, 5);
                end case;
                -- the vertical IIR's shift: about 32 .. 48 virtual lines in every mode
                if s_ys >= 960 then c_iir <= to_unsigned(3, 3);
                elsif s_ys >= 480 then c_iir <= to_unsigned(4, 3);
                else c_iir <= to_unsigned(5, 3); end if;
                c_invys <= f_invys(s_tid);
                -- stepped knob: a new step is taken only 8 counts inside it (and at either end of the travel)
                v_k := unsigned(registers_in(3));
                if v_k(9 downto 7) /= s_mi and ((v_k(6 downto 3) /= "0000" and v_k(6 downto 3) /= "1111")
                        or v_k(9 downto 3) = "0000000" or v_k(9 downto 3) = "1111111") then
                    s_mi <= v_k(9 downto 7);
                end if;
            end if;
            g_wm1 <= s_w - 1;
            case to_integer(unsigned(s_tid)) is                     -- the Osc's line step, ys * 121 / 256
                when 0          => s_oys <= to_unsigned(537, 10);
                when 4          => s_oys <= to_unsigned(272, 10);
                when 8          => s_oys <= to_unsigned(453, 10);
                when 12         => s_oys <= to_unsigned(226, 10);
                when 5 | 6 | 14 => s_oys <= to_unsigned(181, 10);
                when 1 | 2 | 10 => s_oys <= to_unsigned(242, 10);
                when others     => s_oys <= to_unsigned(121, 10);
            end case;

            -- [B] shared hold + glide, six clocks per channel (X, Y, Period, Camera, P12, Drift):
            --   hold: the knob is re-read only when it has moved more than one count (pot jitter)
            --   glide On: 1/16 of the gap per field (Drift's speed 1/8), never slower than one count, landing
            --   exactly.  Glide Off: the accumulator takes the held value of THIS field start (no easing, no lag)
            if s_zgo = '1' then
                z_gp <= "000"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                z_gp <= z_gp + 1;
                case to_integer(z_gp) is
                    when 0 =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_ax; s_gh <= s_hx; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_ay; s_gh <= s_hy; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_ap; s_gh <= s_hp; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_ac; s_gh <= s_hc; s_gr <= unsigned(registers_in(5));
                            when 4      => s_gm <= s_at; s_gh <= s_ht; s_gr <= unsigned(registers_in(7));
                            when others => s_gm <= s_av; s_gh <= s_hv; s_gr <= unsigned(registers_in(4));
                        end case;
                    when 1 =>
                        s_hd <= signed(resize(s_gr, 11)) - signed(resize(s_gh, 11));
                    when 2 =>
                        if s_hd > 1 or s_hd < -1 then s_gt <= s_gr; else s_gt <= s_gh; end if;
                    when 3 =>
                        s_gd <= signed(resize(s_gt & "000000", 17)) - signed(resize(s_gm, 17));
                        case to_integer(z_gc) is
                            when 0      => s_hx <= s_gt;
                            when 1      => s_hy <= s_gt;
                            when 2      => s_hp <= s_gt;
                            when 3      => s_hc <= s_gt;
                            when 4      => s_ht <= s_gt;
                            when others => s_hv <= s_gt;
                        end case;
                    when 4 =>
                        if s_glide = '0' or (s_gd <= 64 and s_gd >= -64) then
                            null;                                          -- the whole gap
                        elsif z_gc = 5 then                                -- Drift: 1/8
                            if s_gd < 512 and s_gd > -512 then
                                if s_gd(16) = '1' then s_gd <= to_signed(-64, 17); else s_gd <= to_signed(64, 17); end if;
                            else
                                s_gd <= shift_right(s_gd, 3);
                            end if;
                        elsif s_gd < 1024 and s_gd > -1024 then
                            if s_gd(16) = '1' then s_gd <= to_signed(-64, 17); else s_gd <= to_signed(64, 17); end if;
                        else
                            s_gd <= shift_right(s_gd, 4);
                        end if;
                    when others =>
                        v_st := signed(resize(s_gm, 17)) + s_gd;
                        case to_integer(z_gc) is
                            when 0      => s_ax <= unsigned(v_st(15 downto 0));
                            when 1      => s_ay <= unsigned(v_st(15 downto 0));
                            when 2      => s_ap <= unsigned(v_st(15 downto 0));
                            when 3      => s_ac <= unsigned(v_st(15 downto 0));
                            when 4      => s_at <= unsigned(v_st(15 downto 0));
                            when others => s_av <= unsigned(v_st(15 downto 0));
                        end case;
                        z_gp <= "000";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] before the slots (zc 40 .. 46): lace and form constants, the P12 blend, pan, Y, the Drift decode
            -- (Hypnos: |v| < 24 still; else 2^-12 .. 2^-2 copy per field, exponential, mantissa from the ROM)
            if s_zone = '1' then
                v_mi := to_integer(s_mi);
                if s_form = '1' then v_fm := 1; else v_fm := 0; end if;
                case to_integer(s_zc) is
                    when 40 =>
                        c_mq(0) <= to_unsigned(C_GU_MQ0(v_mi), 7);
                        c_mq(1) <= to_unsigned(C_GU_MQ1(v_mi), 7);
                        c_mq(2) <= to_unsigned(C_GU_MQ2(v_mi), 7);
                        c_mc(0) <= to_unsigned(C_GU_MC0(v_mi), 5);
                        c_mc(1) <= to_unsigned(C_GU_MC1(v_mi), 5);
                        c_mc(2) <= to_unsigned(C_GU_MC2(v_mi), 5);
                        c_dr1 <= to_unsigned(C_GU_DR1(v_mi), 16);
                        c_dr2 <= to_unsigned(C_GU_DR2(v_mi), 16);
                        if C_GU_RQ0(v_mi) = 1 then c_rq(0) <= '1'; else c_rq(0) <= '0'; end if;
                        if C_GU_RQ1(v_mi) = 1 then c_rq(1) <= '1'; else c_rq(1) <= '0'; end if;
                        if C_GU_RQ2(v_mi) = 1 then c_rq(2) <= '1'; else c_rq(2) <= '0'; end if;
                        c_ks(0) <= to_unsigned(C_GU_RK0(v_mi) + 1, 3);
                        c_ks(1) <= to_unsigned(C_GU_RK1(v_mi) + 1, 3);
                        c_ks(2) <= to_unsigned(C_GU_RK2(v_mi) + 1, 3);
                        c_toff(0) <= to_unsigned(13 + C_GU_RK0(v_mi) + 2 * C_GU_RQ0(v_mi), 5);
                        c_toff(1) <= to_unsigned(13 + C_GU_RK1(v_mi) + 2 * C_GU_RQ1(v_mi), 5);
                        c_toff(2) <= to_unsigned(13 + C_GU_RK2(v_mi) + 2 * C_GU_RQ2(v_mi), 5);
                        c_pitch <= to_unsigned(C_GU_PITCH(v_fm), 19);
                        c_half  <= to_unsigned(C_GU_HALF(v_fm), 19);
                        c_a0    <= to_unsigned(C_GU_A0(v_fm), 19);
                        c_rcen  <= to_unsigned(C_GU_RC(v_fm), 19);
                        c_rh    <= to_unsigned(C_GU_HWR(v_fm), 12) + resize(shift_right(s_ys, 1), 12);
                        c_2hy   <= shift_left(resize(shift_right(s_ys, 1), 12), 1);
                        c_inva(0) <= to_unsigned(C_GU_INVA0(v_fm), 15);
                        c_inva(1) <= to_unsigned(C_GU_INVA1(v_fm), 15);
                        c_inva(2) <= to_unsigned(C_GU_INVA2(v_fm), 15);
                        c_upper <= s_at(15);
                        c_w     <= s_at(14 downto 4);
                        c_w2    <= to_unsigned(2048, 12) - resize(s_at(14 downto 4), 12);
                        c_v0    <= to_signed(-138240, 21) + signed(resize(shift_right(s_ys, 1), 21))
                                   - signed(resize(shift_left(resize(s_ys, 13), 1), 21));
                        v_11 := signed('0' & s_av(15 downto 6)) - to_signed(512, 11);
                        v_11n := -v_11;
                        z_vneg <= v_11(10);
                        if v_11(10) = '1' then z_va <= unsigned(v_11n(9 downto 0)); else z_va <= unsigned(v_11(9 downto 0)); end if;
                    when 41 =>
                        if z_va < 24 then z_vzero <= '1'; z_u <= (others => '0');
                        else z_vzero <= '0'; z_u <= z_va - 24; end if;
                        -- the pre-pixel minus X, h: -dxm - (gX - 32768) / 16
                        v_x := signed(resize(s_ax, 17)) - to_signed(32768, 17);
                        v_mx := -signed(resize(w_dxm, 14)) - resize(shift_right(v_x, 4), 14);
                        v_mxn := -v_mx;
                        z_mxneg <= v_mx(13);
                        if v_mx(13) = '1' then z_mxmag <= unsigned(v_mxn(11 downto 0)); else z_mxmag <= unsigned(v_mx(11 downto 0)); end if;
                        -- Y in Q8 virtual px: (gY - 32768) * 4
                        v_x := signed(resize(s_ay, 17)) - to_signed(32768, 17);
                        c_y8 <= shift_left(resize(v_x, 20), 2);
                    when 42 =>
                        z_ls <= z_u + shift_right(z_u, 2) + shift_right(z_u, 4);
                    when 43 =>
                        r_ex_a <= z_ls(5 downto 0) & "00";
                    when 46 =>
                        if z_vzero = '1' then s_spd <= (others => '0');
                        else s_spd <= resize(r_exf(15 downto 7), 20); end if;
                        s_ssh <= resize(z_ls(9 downto 6), 4);
                    when others => null;
                end case;
            end if;
            if s_ssh /= 0 then                                    -- the speed: mantissa << octave, 1 bit per clock
                s_spd <= shift_left(s_spd, 1);
                s_ssh <= s_ssh - 1;
            end if;

            -- [C] serial products, one slot every 24 zone clocks from zc 64: the operands are muxed at phase
            -- 1, loaded at phase 2, 16 iterations, the product is stable from phase 19 and stored at phase 20.
            if s_zone = '0' then
                z_run <= '0';
            elsif s_zc = 64 then
                z_run <= '1'; z_ph <= (others => '0'); z_slot <= (others => '0');
            elsif z_run = '1' then
                if z_ph = 23 then
                    z_ph <= (others => '0');
                    if z_slot = 15 then z_run <= '0'; else z_slot <= z_slot + 1; end if;
                else
                    z_ph <= z_ph + 1;
                end if;
            end if;
            if z_run = '1' and z_ph = 0 then z_pre <= '1'; else z_pre <= '0'; end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_slot) is
                    when 0  => z_fa <= resize(s_spd, 40);   z_fb <= c_dr1;                       -- drift of laces 1, 2
                    when 1  => z_fa <= resize(s_spd, 40);   z_fb <= c_dr2;
                    when 2  => z_fa <= resize(s_ap, 40);    z_fb <= to_unsigned(1107, 16);       -- octaves above 72 px, Q8
                    when 3  => z_fa <= resize(r_exf, 40);   z_fb <= shift_right(to_unsigned(29127, 16), to_integer(c_sh));  -- 1 / lambda
                    when 4  => z_fa <= resize(c_invL, 40);  z_fb <= resize(c_mq(0), 16);         -- per lace: K = M / lambda_i,
                    when 7  => z_fa <= resize(c_invL, 40);  z_fb <= resize(c_mq(1), 16);         -- the phase per pixel, the
                    when 10 => z_fa <= resize(c_invL, 40);  z_fb <= resize(c_mq(2), 16);         -- phase at the pre-pixel
                    when 5 | 8 | 11  => z_fa <= resize(c_K, 40); z_fb <= resize(s_xs, 16);
                    when 6 | 9 | 12  => z_fa <= resize(c_K, 40); z_fb <= resize(z_mxmag, 16);
                    when 13 =>                                                                   -- camera gain
                        if c_invL > 203000 then z_fa <= to_unsigned(203000, 40); else z_fa <= resize(c_invL, 40); end if;
                        z_fb <= s_ac;
                    when 14 => z_fa <= resize(z_t1, 40);    z_fb <= resize(c_cmode, 16);
                    when others => z_fa <= resize(c_invL, 40); z_fb <= resize(s_xs, 16);        -- footprint base
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(38 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if z_run = '1' then
                if z_ph = 20 then
                    case to_integer(z_slot) is
                        when 0  => z_d1 <= f_acc(35 downto 16);
                        when 1  => z_d2 <= f_acc(35 downto 16);
                        when 2  => z_oct <= f_acc(26 downto 16);
                        when 3  => c_invL <= f_acc(29 downto 12);
                        when 4 | 7 | 10 => c_K <= f_acc(24 downto 2);
                        when 5  => c_dP(0) <= f_acc(24 downto 5);
                        when 8  => c_dP(1) <= f_acc(24 downto 5);
                        when 11 => c_dP(2) <= f_acc(24 downto 5);
                        when 6  => c_B(0) <= f_addsub(to_unsigned(0, 21), f_acc(25 downto 5), z_mxneg);   -- B = +-K |mx| / 32
                        when 9  => c_B(1) <= f_addsub(to_unsigned(0, 21), f_acc(25 downto 5), z_mxneg);
                        when 12 => c_B(2) <= f_addsub(to_unsigned(0, 21), f_acc(25 downto 5), z_mxneg);
                        when 13 => z_t1 <= f_acc(33 downto 16);
                        when 14 => c_gain <= f_acc(29 downto 22);
                        when others => c_gx0 <= f_acc(19 downto 5);
                    end case;
                end if;
                if z_ph = 21 and z_slot = 2 then                      -- 2^-oct = table(-oct mod 256) * 2^-sh
                    v_e := -signed(resize(z_oct, 12));
                    r_ex_a <= unsigned(v_e(7 downto 0));
                    v_e := -shift_right(v_e, 8);                       -- 0 .. 5
                    c_sh <= unsigned(v_e(2 downto 0));
                end if;
                -- the drift accumulators step once per field (after the slot-1 product): lace 0 +-spd, lace 1 -+d1,
                -- lace 2 +-d2 (the sign of the Drift knob)
                if z_slot = 2 and z_ph = 1 and z_vzero = '0' then
                    s_T(0) <= f_addsub(s_T(0), s_spd, z_vneg);
                    s_T(1) <= f_addsub(s_T(1), z_d1, not z_vneg);
                    s_T(2) <= f_addsub(s_T(2), z_d2, z_vneg);
                end if;
            end if;

            -- [A] sync delay, generated-avid and start-pulse shift registers (field_n is tied high by the
            -- core on every input, so it only gets a register)
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 8);
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
    -- [C] LINE ENGINE.  Kicked at every line start (and twice in the zone, for
    -- lines -1 and 0), it computes the NEXT line on its own serial multiplier:
    --   pre   v += ys (band coordinate, Q8 virtual px, wrapping by the pitch), |v|, the rule coverage
    --   lace  s = -v / A_i -> u = (s + 1) 65536 -> the sine and cycloid ROMs at u >> 7; the triangle is
    --         (131072 - u) / 4; P12 blends -> h = (ha (2048 - w) + hb w) / 2048 (two products, one sum);
    --         H = M h (mod 2, Q1.13); gy = |h - h of the line before| / 16 (Q12 cycles of the lace per line)
    -- At the line start the results become the current line's values.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_i    : integer range 0 to 2;
        variable v_pr   : unsigned(19 downto 0);
        variable v_u    : signed(21 downto 0);
        variable v_gy   : unsigned(11 downto 0);
        variable v_dv   : signed(20 downto 0);
    begin
        if rising_edge(clk) then
            m_ld <= '0'; m_keep <= '0';
            if e_blk = 0 then v_i := 0; else v_i := to_integer(e_blk) - 1; end if;

            -- the multiplier: load (clearing the sum, or keeping it to add a second product), 16 shift-add steps
            if m_ld = '1' then
                if m_keep = '0' then m_acc <= (others => '0'); end if;
                m_a <= resize(m_la, 34); m_b <= m_lb;
            else
                if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
                m_a <= m_a(32 downto 0) & '0';
                m_b <= '0' & m_b(15 downto 1);
            end if;

            -- the band coordinate: one adder (zone: load Y, add line -2's offset, then normalise into the band by
            -- adding or subtracting the pitch on every other clock; per line: + ys, then - pitch past the band)
            if s_zone = '1' and s_zc = 452 then
                e_v <= resize(c_y8, 21); e_par <= '0';
                e_dv <= c_v0;
            elsif s_zone = '1' and s_zc >= 453 and s_zc <= 463 then
                if s_zc(0) = '1' then                                  -- 453, 455, ..: add
                    e_v <= e_v + e_dv;
                    if s_zc /= 453 and e_dv /= 0 then e_par <= not e_par; end if;
                elsif e_v >= signed(resize(c_half, 21)) then           -- 454, 456, ..: decide
                    e_dv <= -signed(resize(c_pitch, 21));
                elsif e_v < -signed(resize(c_half, 21)) then
                    e_dv <= signed(resize(c_pitch, 21));
                else
                    e_dv <= (others => '0');
                end if;
            elsif s_zone = '1' and s_zc = 464 then
                e_run <= '1'; e_cnt <= "10"; e_blk <= "00"; e_pc <= (others => '0');
                e_dv <= signed(resize(s_ys, 21));
            elsif s_st(0) = '1' then
                cur_Hm <= n_Hm; cur_gyq <= n_gyq; cur_ins <= n_ins; cur_in0 <= n_in0; cur_rc <= n_rc; cur_par <= n_par;
                e_run <= '1'; e_cnt <= "01"; e_blk <= "00"; e_pc <= (others => '0');
                e_dv <= signed(resize(s_ys, 21));
            elsif e_run = '1' then
                e_pc <= e_pc + 1;
                if e_blk = 0 then
                    case to_integer(e_pc) is
                        when 0 =>                                              -- + ys
                            e_v <= e_v + e_dv;
                        when 1 =>
                            if e_v >= signed(resize(c_half, 21)) then e_dv <= -signed(resize(c_pitch, 21)); e_par <= not e_par;
                            else e_dv <= (others => '0'); end if;
                        when 2 =>                                              -- - pitch past the band
                            e_v <= e_v + e_dv;
                        when 3 =>
                            if e_v(20) = '1' then e_av <= unsigned(resize(-e_v, 19)); else e_av <= unsigned(e_v(18 downto 0)); end if;
                            if e_v > 0 then e_vpos <= '1'; else e_vpos <= '0'; end if;
                            n_par <= e_par;
                            e_dv <= signed(resize(s_ys, 21));
                        when 4 =>
                            if e_av <= c_a0 then n_in0 <= '1'; else n_in0 <= '0'; end if;
                            if e_av >= c_rcen then e_dq <= e_av - c_rcen; else e_dq <= c_rcen - e_av; end if;
                        when 5 =>
                            -- rule coverage: the line [|v| - ys/2, |v| + ys/2] against the rule [rcen - hwr, rcen + hwr]
                            if e_dq >= resize(c_rh, 19) then e_ov <= (others => '0');
                            elsif resize(c_rh, 19) - e_dq >= resize(c_2hy, 19) then e_ov <= c_2hy;
                            else e_ov <= resize(resize(c_rh, 19) - e_dq, 12); end if;
                        when 6 =>
                            m_la <= resize(e_ov, 18); m_lb <= resize(c_invys, 16); m_ld <= '1';
                        when 24 =>
                            if m_acc(33 downto 20) /= 0 then n_rc <= to_unsigned(16, 5);
                            else n_rc <= '0' & m_acc(19 downto 16); end if;
                            e_blk <= "01"; e_pc <= (others => '0');
                        when others => null;
                    end case;
                else
                    case to_integer(e_pc) is
                        when 0 =>
                            m_la <= resize(e_av, 18); m_lb <= resize(c_inva(v_i), 16); m_ld <= '1';
                        when 18 =>
                            v_pr := m_acc(31 downto 12);
                            if e_vpos = '1' then v_u := to_signed(65536, 22) - signed(resize(v_pr, 22));
                            else v_u := to_signed(65536, 22) + signed(resize(v_pr, 22)); end if;
                            if v_u >= 0 and v_u < 131072 then n_ins(v_i) <= '1'; else n_ins(v_i) <= '0'; end if;
                            if v_u < 0 then e_u <= (others => '0');
                            elsif v_u > 131071 then e_u <= to_unsigned(131071, 17);
                            else e_u <= unsigned(v_u(16 downto 0)); end if;
                        when 19 =>
                            e_ra <= e_u(16 downto 7);
                        when 22 =>
                            -- the ROMs at u >> 7 (fabric copies); P12 below 50 %: triangle -> sine, above: sine -> cycloid
                            if c_upper = '1' then m_la <= resize(rs_f, 18); e_hb <= rc_f;
                            else m_la <= resize(shift_right(to_unsigned(131072, 18) - resize(e_u, 18), 2), 18); e_hb <= rs_f; end if;
                            m_lb <= resize(c_w2, 16); m_ld <= '1';
                        when 40 =>
                            m_la <= resize(e_hb, 18); m_lb <= resize(c_w, 16); m_ld <= '1'; m_keep <= '1';
                        when 58 =>
                            e_h <= m_acc(26 downto 11);
                        when 59 =>
                            m_la <= resize(e_h, 18); m_lb <= resize(c_mc(v_i), 16); m_ld <= '1';
                            if e_h(15 downto 4) >= e_hp(v_i) then v_gy := e_h(15 downto 4) - e_hp(v_i);
                            else v_gy := e_hp(v_i) - e_h(15 downto 4); end if;
                            n_gyq(v_i) <= v_gy;
                            e_hp(v_i) <= e_h(15 downto 4);
                        when 77 =>
                            n_Hm(v_i) <= m_acc(16 downto 3);
                            e_pc <= (others => '0');
                            if e_blk = 3 then
                                e_blk <= "00";
                                if e_cnt = 1 then e_run <= '0'; end if;
                                e_cnt <= e_cnt - 1;
                            else
                                e_blk <= e_blk + 1;
                            end if;
                        when others => null;
                    end case;
                end if;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [C] BEAM DDAs: per lace, the phase at the pre-pixel (pan B, minus the
    -- drift T in even bands, plus in odd bands), stepped once per pixel so that
    -- pixel 0 is at stage 12.  Q1.20, modulo 2 copies.
    ---------------------------------------------------------------------------
    p_dda : process(clk)
        variable v_a, v_b : unsigned(20 downto 0);
    begin
        if rising_edge(clk) then
            for i in 0 to 2 loop
                -- one carry chain: load B -+ T at the line start, else step by dP
                if s_st(9) = '1' then v_a := c_B(i); v_b := resize(s_T(i), 21);
                else v_a := b_P(i); v_b := resize(c_dP(i), 21); end if;
                b_P(i) <= f_addsub(v_a, v_b, s_st(9) and not cur_par);
            end loop;
        end if;
    end process p_dda;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) x camera gain -> box over 96
    -- virtual px, edge-replicated and reloaded at every line start -> vertical
    -- one-pole IIR in the line store (half horizontal resolution; the first
    -- line of a field loads it).  The field c0 is the displacement in cycles
    -- of the base lace (Q4.12); the pixel path taps the store's write data
    -- (the camera is never shown, so it need not be aligned to the pixel).
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l6  : unsigned(5 downto 0);
        variable v_s   : unsigned(7 downto 0);
        variable v_pr  : unsigned(13 downto 0);
        variable v_tap : unsigned(13 downto 0);
        variable v_nf  : unsigned(18 downto 0);
        variable v_d   : signed(16 downto 0);
        variable v_dd  : signed(16 downto 0);
        variable v_n   : signed(16 downto 0);
        variable v_m   : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then   -- both phases restart from this field's drift
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);   -- ot * 96
            end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- 1: the source sample: (luma >> 4) - 4, or the lattice >> 2 (6 bits)
            v_l6 := unsigned(data_in.y(9 downto 4));
            if s_src = '1' then s_sl <= s_osum(7 downto 2);
            elsif v_l6 > 4 then s_sl <= v_l6 - 4;
            else s_sl <= (others => '0'); end if;
            s_avd <= data_in.avid & s_avd(1 to 4);
            if s_avd(1) = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- 2: x camera gain (the one multiplier of the source path); the lattice goes in at a quarter
            v_pr := s_sl * c_gain;
            if s_src = '1' then s_p <= resize(v_pr(13 downto 2), 14); else s_p <= v_pr(13 downto 0); end if;
            s_pcol <= s_col;

            -- 3: box: sum of the last N samples; the samples before the line start count as the first one
            if s_avd(2) = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= c_brp; end if;
            if s_bcnt /= c_bn then v_tap := s_pfirst; else v_tap := bx_q(13 downto 0); end if;
            case to_integer(s_xs) is
                when 4      => v_nf := shift_left(resize(s_p, 19), 5) + shift_left(resize(s_p, 19), 4);   -- 48
                when 3      => v_nf := shift_left(resize(s_p, 19), 6);                                    -- 64
                when others => v_nf := shift_left(resize(s_p, 19), 6) + shift_left(resize(s_p, 19), 5);   -- 96
            end case;
            if s_avd(2) = '1' and s_avd(3) = '0' then
                s_pfirst <= s_p;
                s_bsum <= v_nf;
                s_bcnt <= to_unsigned(1, 7);
            elsif s_avd(2) = '1' then
                s_bsum <= s_bsum + s_p - v_tap;
                if s_bcnt /= c_bn then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bcol <= s_pcol;

            -- 4: fabric copies; 5: vertical IIR on the odd columns, q += (box - q) >> k (the first line: q = box)
            s_eb <= s_bsum(18 downto 3); s_eq <= lb_q1;
            s_ev <= s_avd(3) and s_bcol(0); s_ecol <= s_bcol;
            if s_zone = '1' then s_fl <= '1';
            elsif s_avd(5) = '1' and s_avd(4) = '0' then s_fl <= '0'; end if;
            v_d := signed(resize(s_eb, 17)) - signed(resize(s_eq, 17));
            case to_integer(c_iir) is
                when 3      => v_dd := shift_right(v_d, 3);
                when 4      => v_dd := shift_right(v_d, 4);
                when others => v_dd := shift_right(v_d, 5);
            end case;
            v_n := signed(resize(s_eq, 17)) + v_dd;
            if s_fl = '1' then
                lb_wd <= s_eb; lb_dd <= (others => '0');
            else
                lb_wd <= unsigned(v_n(15 downto 0)); lb_dd <= v_dd(13 downto 0);
            end if;
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            -- the pixel path's tap: the last two columns written; stage 8 interpolates between them on the
            -- clock after a write
            if lb_we = '1' then
                h_cur <= lb_wd; h_prev <= h_cur; h_dy <= lb_dd; h_ph <= '1';
            else
                h_ph <= '0';
            end if;
            v_m := resize(h_cur, 17) + resize(h_prev, 17);
            if h_ph = '1' then s8_q <= v_m(16 downto 1); else s8_q <= h_cur; end if;
            s8_dy <= h_dy;
            s9_D  <= s8_q;
            s9_D3 <= s8_q + shift_left(s8_q, 1);
            s9_Dy <= s8_dy;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] LACE, per lace i:
    --   10  c0's pixel difference, the IIR increment     11  footprint terms
    --   12  base = |gx0 + dc0/dx| + |dc0/dy| (Q20 base cycles per pixel); the beam phase (Q1.20)
    --   13  w = base + gy_i (this line's vertical term, in base cycles); Pc = beam + the camera phase rho_i c0
    --       (q 2^k: a shift of c0 or 3 c0, c0 read 3 pixels on; lace 2 = lace 1 / 2)
    --   14  w normalised by 8 and 4 places; P1, P2 = Pc +- H
    --   15  by 2 and 1 -> sh, the 4-bit mantissa -> t = (13 + k + 2q) - sh, the exponent of 1 / (rho w), and
    --       rr = the mantissa's reciprocal (ROM); the nearer of P1's and P2's edges, the parity
    --   16  the distance x 2^(3 - t) (Q4.4 px before the mantissa, saturating at 63); the fade (from rr, t)
    --   17  x rr (6 x 5) -> the distance in px (Q4.4)
    ---------------------------------------------------------------------------
    p_lace : process(clk)
        variable v_cam  : unsigned(15 downto 0);
        variable v_c14  : unsigned(13 downto 0);
        variable v_c15  : unsigned(14 downto 0);
        variable v_bx   : unsigned(24 downto 0);
        variable v_s23  : unsigned(22 downto 0);
        variable v_w    : unsigned(22 downto 0);
        variable v_n    : unsigned(21 downto 0);
        variable v_sh   : unsigned(3 downto 0);
        variable v_m    : unsigned(7 downto 0);
        variable v_P    : unsigned(13 downto 0);
        variable v_d1, v_d2 : unsigned(11 downto 0);
        variable v_ix   : unsigned(4 downto 0);
        variable v_pr   : unsigned(10 downto 0);
        variable v_t    : signed(5 downto 0);
        variable v_x    : unsigned(14 downto 0);
    begin
        if rising_edge(clk) then
            -- 10
            s10_D  <= s9_D;
            s10_dx <= signed(s9_D - s10_D);
            s10_dy <= s9_Dy;
            -- 11
            s11_bx  <= signed(resize(c_gx0, 25)) + shift_left(resize(s10_dx, 25), 8);
            s11_ady <= shift_left(resize(unsigned(abs(resize(s10_dy, 15))), 22), 8);
            -- 12
            if s11_bx(24) = '1' then v_bx := unsigned(-s11_bx); else v_bx := unsigned(s11_bx); end if;
            v_s23 := resize(v_bx(21 downto 0), 23) + resize(s11_ady, 23);
            if v_bx(24 downto 22) /= "000" or v_s23(22) = '1' then s12_base <= (others => '1');
            else s12_base <= v_s23(21 downto 0); end if;
            -- 13: footprint per lace; the phase: beam (stage 12) + camera
            for i in 0 to 2 loop
                v_w := resize(s12_base, 23) + shift_left(resize(cur_gyq(i), 23), 8 + i);
                if v_w(22) = '1' then s13_w(i) <= (others => '1'); else s13_w(i) <= v_w(21 downto 0); end if;
            end loop;
            if c_rq(0) = '1' then v_cam := s9_D3; else v_cam := s9_D; end if;
            case to_integer(c_ks(0)) is
                when 1      => v_c14 := v_cam(12 downto 0) & '0';
                when 2      => v_c14 := v_cam(11 downto 0) & "00";
                when 3      => v_c14 := v_cam(10 downto 0) & "000";
                when 4      => v_c14 := v_cam(9 downto 0) & "0000";
                when others => v_c14 := v_cam(8 downto 0) & "00000";
            end case;
            s13_Pc(0) <= b_P(0)(20 downto 7) + v_c14;
            if c_rq(1) = '1' then v_cam := s9_D3; else v_cam := s9_D; end if;
            case to_integer(c_ks(1)) is
                when 1      => v_c15 := v_cam(13 downto 0) & '0';
                when 2      => v_c15 := v_cam(12 downto 0) & "00";
                when others => v_c15 := v_cam(11 downto 0) & "000";
            end case;
            s13_Pc(1) <= b_P(1)(20 downto 7) + v_c15(13 downto 0);
            s13_Pc(2) <= b_P(2)(20 downto 7) + v_c15(14 downto 1);
            -- 14
            for i in 0 to 2 loop
                -- normalise (the footprint is at least 2^9: 15 places of shift are enough)
                v_n := s13_w(i); v_sh := "0000";
                if v_n(21 downto 14) = 0 then v_n := v_n(13 downto 0) & x"00"; v_sh := "1000"; end if;
                if v_n(21 downto 18) = 0 then v_n := v_n(17 downto 0) & x"0"; v_sh := v_sh + 4; end if;
                s14_n(i) <= v_n(21 downto 14); s14_sh(i) <= v_sh;
                s14_P1(i) <= s13_Pc(i) + cur_Hm(i);
                s14_P2(i) <= s13_Pc(i) - cur_Hm(i);
            end loop;
            -- 15
            for i in 0 to 2 loop
                v_m := s14_n(i); v_sh := s14_sh(i);
                if v_m(7 downto 6) = 0 then v_m := v_m(5 downto 0) & "00"; v_sh := v_sh + 2; end if;
                if v_m(7) = '0' then v_m := v_m(6 downto 0) & '0'; v_sh := v_sh + 1; end if;
                v_t := signed(resize(c_toff(i), 6)) - signed(resize(v_sh, 6));
                if v_t < 0 then s15_t(i) <= (others => '0');
                elsif v_t > 15 then s15_t(i) <= (others => '1');
                else s15_t(i) <= unsigned(v_t(3 downto 0)); end if;
                if c_rq(i) = '1' then v_ix := '1' & v_m(6 downto 3); else v_ix := '0' & v_m(6 downto 3); end if;
                s15_rr(i) <= C_GU_RR(to_integer(v_ix));
                v_P := s14_P1(i);
                if v_P(12) = '0' then v_d1 := v_P(11 downto 0); else v_d1 := not v_P(11 downto 0); end if;
                v_P := s14_P2(i);
                if v_P(12) = '0' then v_d2 := v_P(11 downto 0); else v_d2 := not v_P(11 downto 0); end if;
                if v_d1 < v_d2 then s15_dm(i) <= v_d1; else s15_dm(i) <= v_d2; end if;
                s15_p(i) <= s14_P1(i)(13) xor s14_P2(i)(13);
            end loop;
            -- 16: x = (d << 3) >> t on a four-stage log shifter, saturating at 63; the fade
            for i in 0 to 2 loop
                v_x := s15_dm(i) & "000";
                if s15_t(i)(3) = '1' then v_x := x"00" & v_x(14 downto 8); end if;
                if s15_t(i)(2) = '1' then v_x := x"0" & v_x(14 downto 4); end if;
                if s15_t(i)(1) = '1' then v_x := "00" & v_x(14 downto 2); end if;
                if s15_t(i)(0) = '1' then v_x := '0' & v_x(14 downto 1); end if;
                if v_x(14 downto 6) /= 0 then s16_dsh(i) <= "111111"; else s16_dsh(i) <= v_x(5 downto 0); end if;
                s16_f(i) <= f_fade(s15_rr(i), s15_t(i));
                s16_rr(i) <= s15_rr(i); s16_p(i) <= s15_p(i);
            end loop;
            -- 17
            for i in 0 to 2 loop
                v_pr := s16_dsh(i) * s16_rr(i);
                s17_dpx(i) <= v_pr(10 downto 4);
                s17_f(i) <= s16_f(i); s17_p(i) <= s16_p(i);
            end loop;
        end if;
    end process p_lace;

    ---------------------------------------------------------------------------
    -- [E] ALL LACES (stages 18 .. 27): coverages 0 .. 16
    --   18  outside a lace's band, or Single for laces 1 and 2: no edge, no parity, no fade
    --   19, 20  the nearest edge of the six stripes, their parity, the strongest fade
    --   21  Bold: the parity's coverage 8 +- d; Fine: the line's 8 + hw - d (hw = 1 virtual px)
    --   22, 23  fade toward grey (Bold 8, Fine 4) in quarters
    --   24  outside the band: ground; the rule        25  luma 64 + o * 876 / 16 (white 940)   26, 27 registers
    ---------------------------------------------------------------------------
    p_cov : process(clk)
        variable v_t, v_tn : signed(5 downto 0);
        variable v_a  : unsigned(3 downto 0);
        variable v_m  : unsigned(5 downto 0);
        variable v_x  : signed(7 downto 0);
        variable v_e  : signed(8 downto 0);
        variable v_o  : unsigned(4 downto 0);
        variable v_y  : unsigned(14 downto 0);
    begin
        if rising_edge(clk) then
            -- 18
            for i in 0 to 2 loop
                if cur_ins(i) = '0' or (i > 0 and s_tri = '0') then
                    s18_dpx(i) <= (others => '1'); s18_p(i) <= '0'; s18_f(i) <= (others => '0');
                else
                    s18_dpx(i) <= s17_dpx(i); s18_p(i) <= s17_p(i); s18_f(i) <= s17_f(i);
                end if;
            end loop;
            -- 19
            if s18_dpx(1) < s18_dpx(0) then s19_d <= s18_dpx(1); else s19_d <= s18_dpx(0); end if;
            s19_p <= s18_p(0) xor s18_p(1);
            if s18_f(0) > s18_f(1) then s19_f <= s18_f(0); else s19_f <= s18_f(1); end if;
            s19_d2 <= s18_dpx(2); s19_p2 <= s18_p(2); s19_f2 <= s18_f(2);
            -- 20
            if s19_d2 < s19_d then s20_d <= s19_d2; else s20_d <= s19_d; end if;
            s20_p <= s19_p xor s19_p2;
            if s19_f > s19_f2 then s20_f <= s19_f; else s20_f <= s19_f2; end if;
            -- 21
            v_e := to_signed(8, 9) + signed(resize(c_hv, 9)) - signed(resize(s20_d, 9));
            if s_fine = '1' then
                if v_e < 0 then s21_x <= (others => '0');
                elsif v_e > 16 then s21_x <= to_unsigned(16, 5);
                else s21_x <= unsigned(v_e(4 downto 0)); end if;
            elsif s20_p = '1' then
                if s20_d >= 8 then s21_x <= to_unsigned(16, 5); else s21_x <= to_unsigned(8, 5) + s20_d(2 downto 0); end if;
            else
                if s20_d >= 8 then s21_x <= (others => '0'); else s21_x <= to_unsigned(8, 5) - s20_d(2 downto 0); end if;
            end if;
            s21_f <= s20_f;
            -- 22: f / 4 x (target - x), f in quarters: on the magnitude (no self-add on a sign-extended value),
            -- shifts and one add, then the sign
            if s_fine = '1' then v_t := to_signed(4, 6) - signed(resize(s21_x, 6));
            else v_t := to_signed(8, 6) - signed(resize(s21_x, 6)); end if;
            v_tn := -v_t;
            if v_t(5) = '1' then v_a := unsigned(v_tn(3 downto 0)); else v_a := unsigned(v_t(3 downto 0)); end if;
            case to_integer(s21_f) is
                when 1      => v_m := resize(v_a, 6);
                when 2      => v_m := resize(v_a, 6) sll 1;
                when 3      => v_m := (resize(v_a, 6) sll 1) + resize(v_a, 6);
                when 4      => v_m := resize(v_a, 6) sll 2;
                when others => v_m := (others => '0');
            end case;
            if v_t(5) = '1' then s22_pf <= -signed(resize(v_m, 8)); else s22_pf <= signed(resize(v_m, 8)); end if;
            s22_x <= s21_x;
            -- 23
            v_x := signed(resize(s22_x, 8)) + shift_right(s22_pf, 2);
            s23_v <= unsigned(v_x(4 downto 0));
            -- 24
            if cur_in0 = '1' then v_o := s23_v; else v_o := (others => '0'); end if;
            if cur_rc > v_o then v_o := cur_rc; end if;
            s24_o <= v_o;
            -- 25: o * 876 / 16 = ((o << 10) - (o << 7) - (o << 4) - (o << 2)) >> 4
            v_y := shift_left(resize(s24_o, 15), 10) - shift_left(resize(s24_o, 15), 7)
                   - shift_left(resize(s24_o, 15), 4) - shift_left(resize(s24_o, 15), 2);
            s25_y <= to_unsigned(64, 10) + resize(v_y(14 downto 4), 10);
            s26_y <= s25_y;
            s27_y <= s26_y;
        end if;
    end process p_cov;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING (stages 28 .. 33): luma [1 2 1]/4, luma 13/16
    -- toward black (white 776), blanking gate.  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 32 .. 34: taps y(n + 1), y(n), y(n - 1)
            s_yd(1) <= s27_y;
            for i in 2 to 3 loop s_yd(i) <= s_yd(i - 1); end loop;
            s34_ya <= resize(s_yd(1), 12) + resize(s_yd(3), 12) + 2;
            s34_yb <= s_yd(2);
            -- 35
            v_y := s34_ya + shift_left(resize(s34_yb, 12), 1);
            s35_y <= v_y(11 downto 2);
            -- 36: luma 13/16 toward black
            if s35_y > 64 then v_dl := s35_y - 64; else v_dl := (others => '0'); end if;
            s36_y <= s35_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            -- 37: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s37_y <= s36_y;
            else
                s37_y <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s37_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture guilloche;
