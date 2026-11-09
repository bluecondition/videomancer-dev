-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Ocelot v0.1.0 -- Heirloom (video phase), from the clip WIN_20231227_00_20_47 at 6.0 s.
--
-- The still: orange zebra bands with a yellow edge over salmon / grey-teal horizontal stripes, the bands dipped
-- into "<" chevrons on two rows; a black "ocelot" body rolls up through them with scalloped edges and red
-- flames where the orange bands push into it, and inside it rows of glowing tan dash triplets with blue / cyan
-- crescents.  In the clip the zebra stays put (the stripes creep up ~10 lines/s), the body rolls UP at ~124
-- lines/s (one body every 9.4 s) and the zebra bands fan out to the right as the body passes (the body's LFO
-- frequency-modulates the band oscillator).
--
-- The patch, as fitted (ocelot.py has every table):
--   key LFO  Rv(y): a triangle, 1165 lines, rolling up; eps = Stretch max(0, Rv - 0.32), xs = x (1 - eps)
--   zebra    phi = Bands PHIP(xs) + DPHI(xs) Chevron T(y): one chirped H oscillator dipped into chevrons on
--            the rows 252 and 780 (T = (1 - d / 160)^1.35); colour = ZEBRA(stripe weight, phi)
--   body     s = Rv + 0.31 W(theta) - 0.5 - kc > 0, W = |sin|^0.6 scallops 455 px across (theta = xs / 455)
--            the band pushes the key back: black where s - beta F(phi) > 0, red flames between
--   dashes   three glowing bars per pink half of the stripe oscillator: brightness GV(d) GH(phi) -> DASH(b)
--   blue     phi in 0.53 .. 0.97 and frac(theta + 1.1 Rv + 0.15) < 0.25 .. 0.62 (grey .. pink): BLUE(b)
--
--   K1  Stripes   the stripes' (and the dash rows') frequency, x0.4 .. x2.5 (512 = the clip)
--   K2  Bands     the zebra bands' frequency (and the dash columns, flames, crescents), x0.4 .. x2.5
--   K3  Chevron   the chevron depth: reversed ">" (0) .. straight (256) .. the clip (512) .. 3x (1023)
--   K4  Roll      bipolar: the body rolls up (+) or down (-); 700 = the clip's 2 lines / field
--   K5  Stretch   how far the body fans the bands: squeezed (0) .. none (256) .. the clip (512) .. 4x (1023)
--   K6  Etch      how far the camera (or the Osc) bends the bands and the body's edge
--   S7  Twin      one body / two bodies per frame
--   S8  Invert    the key inverted: zebra inside the body, the dash fur outside
--   S9  Fur       Crescents (the clip) / Flood: blue fur over every band phase window in the body
--   S10 Colours   Ocelot / Ice (U and V exchanged: cyan zebra, orange crescents)
--   S11 Source    Video / Osc
--   P12 BODY      the body's size: a thin string of scallops (0) .. the clip (512) .. the body fills the
--                 frame and the zebra shows only through scalloped windows, flames taller (100 %)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line
-- start (computes the NEXT line: the LFO, the stretch, the stripe / dash row, the chevron dip), beam DDAs, a
-- 21-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output
-- conditioning (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4, luma 13/16) before the blanking gate.  Colours in
-- the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.ocelot_pkg.all;
use work.all;

architecture ocelot of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant K_RK0  : integer := 4357000;      -- key phase of line 0 at power-up (Q24: the body centred on row 280)
    constant K_PSI0 : integer := 752864;       -- stripe phase of line 0 (Q20: pink starts on row 55)
    constant K_PC0  : integer := 23824;        -- cusp phase of line 0 (Q20: cusps on rows 252 and 780)
    constant K_TH0  : integer := 6084352;      -- W phase at xs = 0 (Q24: scallop cusps at xs = -165 + 455 k)

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
    -- per-line steps (same timing-ID map): key LFO (Q24, 1165 lines), cusp phase (Q20, 528 lines),
    -- stripe phase base (Q20, 195 lines)
    function f_mstep(t : std_logic_vector(3 downto 0); k : integer) return unsigned is
        type t_tab is array (0 to 6) of integer;
        constant C_DK : t_tab := (64017, 32402, 54004, 27002, 21602, 28802, 14401);
        constant C_DC : t_tab := (8828, 4468, 7447, 3724, 2979, 3972, 1986);
        constant C_BP : t_tab := (23904, 12099, 20165, 10082, 8066, 10755, 5377);
        variable i, v : integer;
    begin
        case to_integer(unsigned(t)) is
            when 0 => i := 0; when 4 => i := 1; when 8 => i := 2; when 12 => i := 3;
            when 5 | 6 | 14 => i := 4; when 1 | 2 | 10 => i := 5; when others => i := 6;
        end case;
        case k is
            when 0 => v := C_DK(i); when 1 => v := C_DC(i); when others => v := C_BP(i);
        end case;
        return to_unsigned(v, 17);
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
    -- 2^(i/32) x 256, i = 0 .. 31
    function f_exp2(i : unsigned(4 downto 0)) return unsigned is
        type t_e is array (0 to 31) of integer;
        constant C_E : t_e := (256, 262, 267, 273, 279, 285, 292, 298, 304, 311, 318, 325, 332, 339, 347, 354,
                               362, 370, 378, 386, 395, 403, 412, 421, 431, 440, 450, 460, 470, 480, 491, 502);
    begin
        return to_unsigned(C_E(to_integer(i)), 9);
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
    signal r_phi_a : unsigned(9 downto 0) := (others => '0');
    signal r_phi_q : std_logic_vector(19 downto 0) := (others => '0');
    signal r_w_a   : unsigned(7 downto 0) := (others => '0');
    signal r_w_q   : std_logic_vector(7 downto 0) := (others => '0');
    signal r_pal_a : unsigned(9 downto 0) := (others => '0');
    signal r_pal_q : std_logic_vector(23 downto 0) := (others => '0');

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

    -- knobs: glided (Q3), raw roll, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_twn, s_inv, s_fld, s_ice, s_src : std_logic := '0';

    -- per-field constants
    signal z_e1, z_e2 : signed(6 downto 0) := (others => '0');        -- stripe / band exponents, 1/32 octave
    signal z_m1 : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal z_bp : unsigned(16 downto 0) := to_unsigned(5377, 17);
    signal c_dk : unsigned(17 downto 0) := to_unsigned(14401, 18);   -- key phase per line (Q24)
    signal c_dc : unsigned(16 downto 0) := to_unsigned(1986, 17);    -- cusp phase per line (Q20)
    signal c_dpsi : unsigned(16 downto 0) := to_unsigned(5377, 17);  -- stripe phase per line (Q20)
    signal c_kb : unsigned(7 downto 0) := to_unsigned(64, 8);        -- band scale (Q6)
    signal c_dep : signed(8 downto 0) := to_signed(64, 9);           -- chevron depth (Q6, -64 .. 191)
    signal c_e  : signed(8 downto 0) := to_signed(60, 9);            -- stretch gain (Q8, -60 .. 239)
    signal c_kc : signed(10 downto 0) := to_signed(221, 11);         -- body threshold offset (Q10)
    signal c_bq : unsigned(4 downto 0) := to_unsigned(15, 5);        -- flame push (Q2 per F unit)
    signal c_m2 : unsigned(17 downto 0) := (others => '0');          -- roll^2
    signal c_roll : unsigned(17 downto 0) := (others => '0');        -- roll per field (Q24)
    signal c_ge : unsigned(5 downto 0) := to_unsigned(9, 6);        -- etch gain
    signal c_rk0  : unsigned(23 downto 0) := to_unsigned(K_RK0, 24);  -- key phase of line 0 (rolls)
    signal c_psi0 : unsigned(19 downto 0) := to_unsigned(K_PSI0, 20); -- stripe phase of line 0 (creeps up)
    signal c_loff : unsigned(6 downto 0) := (others => '0');          -- Osc: lattice mean
    signal c_ty, c_tu, c_tv : unsigned(9 downto 0) := to_unsigned(512, 10);   -- off-avid box input (black)

    -- field serial multiplier (shared with the line engine outside the zone)
    signal f_a   : unsigned(25 downto 0) := (others => '0');
    signal f_b   : unsigned(16 downto 0) := (others => '0');
    signal f_acc : unsigned(25 downto 0) := (others => '0');
    signal z_fa  : unsigned(25 downto 0) := (others => '0');
    signal z_fb  : unsigned(16 downto 0) := (others => '0');
    signal z_sel : unsigned(1 downto 0) := (others => '0');
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
    signal e_lc   : unsigned(5 downto 0) := to_unsigned(63, 6);
    signal e_zk   : std_logic := '0';
    signal e_rk   : unsigned(23 downto 0) := (others => '0');          -- key phase (Q24)
    signal e_psi  : unsigned(19 downto 0) := (others => '0');          -- stripe phase (Q20)
    signal e_pc   : unsigned(19 downto 0) := (others => '0');          -- cusp phase (Q20)
    signal e_rv   : unsigned(9 downto 0) := (others => '0');           -- Rv (Q10)
    signal e_ta   : unsigned(7 downto 0) := (others => '0');           -- T ROM address
    signal e_t8   : unsigned(7 downto 0) := (others => '0');
    signal e_p10  : unsigned(9 downto 0) := (others => '0');
    signal e_pk   : std_logic := '0';
    signal e_dd   : unsigned(9 downto 0) := (others => '0');
    signal e_db   : unsigned(9 downto 0) := (others => '0');
    signal e_e0, e_e1, e_b0, e_b1, e_b2, e_b3 : unsigned(9 downto 0) := (others => '0');
    signal e_eneg : std_logic := '0';
    signal e_om   : unsigned(10 downto 0) := (others => '0');          -- 1024 - eps
    signal e_st   : unsigned(12 downto 0) := (others => '0');          -- x step (Q10 h)
    signal e_x0   : unsigned(19 downto 0) := (others => '0');          -- xs of pixel 0 (Q10 h)
    signal l_fa   : unsigned(15 downto 0) := (others => '0');          -- line-engine operands for the shared multiplier
    signal l_fb   : unsigned(8 downto 0) := (others => '0');
    signal l_ld   : std_logic := '0';
    signal n_st   : unsigned(12 downto 0) := (others => '0');
    signal n_x0   : unsigned(23 downto 0) := (others => '0');
    signal n_ths  : unsigned(16 downto 0) := (others => '0');
    signal n_th0  : unsigned(23 downto 0) := (others => '0');
    signal n_tp   : signed(10 downto 0) := (others => '0');            -- T' = T Chevron (Q8)
    signal n_sl   : signed(11 downto 0) := (others => '0');            -- key line term (Q10)
    signal n_gl   : unsigned(9 downto 0) := (others => '0');           -- crescent gate line term (Q10 of W)
    signal n_gw   : unsigned(9 downto 0) := (others => '0');           -- crescent gate width
    signal n_w3   : unsigned(2 downto 0) := (others => '0');           -- stripe pink weight
    signal n_gv   : unsigned(5 downto 0) := (others => '0');           -- dash bar glow down the bar (0 = none)

    -- beam and line registers
    signal b_xs  : unsigned(23 downto 0) := (others => '0');           -- stretched x (Q10 h)
    signal b_th  : unsigned(23 downto 0) := (others => '0');           -- W phase (Q24)
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_st  : unsigned(12 downto 0) := (others => '0');
    signal l_ths : unsigned(16 downto 0) := (others => '0');
    signal l_tp  : signed(10 downto 0) := (others => '0');
    signal l_sl  : signed(11 downto 0) := (others => '0');
    signal l_gl  : unsigned(9 downto 0) := (others => '0');
    signal l_gw  : unsigned(9 downto 0) := (others => '0');
    signal l_w3  : unsigned(2 downto 0) := (others => '0');
    signal l_gv  : unsigned(5 downto 0) := (others => '0');

    -- pixel path
    signal p2_tg, p3_tg, p4_tg, p5_tg, p6_tg : unsigned(9 downto 0) := (others => '0');
    signal p2_l1, p2_l2, p2_l3 : unsigned(7 downto 0) := (others => '0');
    signal p3_lm, p4_lm : signed(8 downto 0) := (others => '0');
    signal p4_phi : std_logic_vector(19 downto 0) := (others => '0');
    signal p4_w   : unsigned(7 downto 0) := (others => '0');
    signal p5_m1  : unsigned(19 downto 0) := (others => '0');
    signal p5_m2  : signed(19 downto 0) := (others => '0');
    signal p5_wk  : unsigned(9 downto 0) := (others => '0');
    signal p5_pe  : signed(15 downto 0) := (others => '0');
    signal p6_phi : unsigned(7 downto 0) := (others => '0');
    signal p6_s   : signed(12 downto 0) := (others => '0');
    signal p7_p8  : unsigned(7 downto 0) := (others => '0');
    signal p7_fz  : unsigned(5 downto 0) := (others => '0');
    signal p7_s   : signed(12 downto 0) := (others => '0');
    signal p7_win, p7_gt : std_logic := '0';
    signal p7_gh  : unsigned(5 downto 0) := (others => '0');
    signal p7_sal, p8_sal : unsigned(1 downto 0) := (others => '0');
    signal p8_b   : unsigned(11 downto 0) := (others => '0');
    signal p8_bf  : unsigned(10 downto 0) := (others => '0');
    signal p8_s   : signed(12 downto 0) := (others => '0');
    signal p8_p6  : unsigned(5 downto 0) := (others => '0');
    signal p8_win, p8_gt : std_logic := '0';
    signal p9_fl, p10_fl, p11_fl : std_logic := '0';
    signal p11_pal : std_logic_vector(23 downto 0) := (others => '0');
    signal p12_y, p12_u, p12_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p13_y, p13_u, p13_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p14_y, p14_u, p14_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

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
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

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
            r_phi_q <= C_OC_PHI(to_integer(r_phi_a));
            r_w_q   <= C_OC_W(to_integer(r_w_a));
            r_pal_q <= C_OC_PAL(to_integer(r_pal_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_t  : signed(15 downto 0);
        variable v_k  : std_logic_vector(23 downto 0);
        variable v_g  : unsigned(9 downto 0);
        variable v_e2m : unsigned(8 downto 0);
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
            -- roll: bipolar, +-8 deadband
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
                s_twn <= registers_in(6)(0);
                s_inv <= registers_in(6)(1);
                s_fld <= registers_in(6)(2);
                s_ice <= registers_in(6)(3);
                s_src <= registers_in(6)(4);
                s_xs  <= s_xsr;
                c_dc  <= f_mstep(s_tid, 1);
                z_bp  <= f_mstep(s_tid, 2);
                if registers_in(6)(0) = '1' then
                    c_dk <= f_mstep(s_tid, 0) & '0';
                else
                    c_dk <= '0' & f_mstep(s_tid, 0);
                end if;
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

            -- serial products: slot k loads at zc = 32 + 24 k, 17 iterations, result read at 52 + 24 k.
            -- The slot number is decoded at 30 + 24 k and the operands are pre-muxed into z_fa / z_fb at
            -- 31 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 2); z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 2); z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 2); z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 2); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0      => z_fa <= resize(z_bp, 26);              z_fb <= resize(z_m1, 17);           -- stripe step
                    when 1      => z_fa <= resize(s_vm, 26);              z_fb <= resize(s_vm, 17);           -- roll^2
                    when 2      => z_fa <= resize(c_m2, 26);              z_fb <= to_unsigned(233, 17);
                    when others => z_fa <= resize(s_g6(12 downto 3), 26); z_fb <= resize(s_g6(12 downto 3), 17);  -- etch^2
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            elsif l_ld = '1' then                 -- the line engine shares the multiplier outside the zone
                f_acc <= (others => '0');
                f_a <= resize(l_fa, 26); f_b <= resize(l_fb, 17);
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(24 downto 0) & '0';
                f_b <= '0' & f_b(16 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion: last field's roll and the stripes' creep; exponents (1/32 octave); linear curves
                    when 20 =>
                        if s_vneg = '1' then c_rk0 <= c_rk0 - resize(c_roll, 24);
                        else c_rk0 <= c_rk0 + resize(c_roll, 24); end if;
                        c_psi0 <= c_psi0 + 912;
                        if s_src = '1' then c_loff <= to_unsigned(111, 7); else c_loff <= (others => '0'); end if;
                        v_d := signed(resize(s_g1(12 downto 3), 11)) - 512;                 -- stripes: 21 / 256
                        v_t := shift_left(resize(v_d, 16), 4) + shift_left(resize(v_d, 16), 2) + resize(v_d, 16);
                        z_e1 <= resize(shift_right(v_t, 8), 7);
                        v_d := signed(resize(s_g2(12 downto 3), 11)) - 512;                 -- bands: 21 / 256
                        v_t := shift_left(resize(v_d, 16), 4) + shift_left(resize(v_d, 16), 2) + resize(v_d, 16);
                        z_e2 <= resize(shift_right(v_t, 8), 7);
                        v_d := signed(resize(s_g3(12 downto 3), 11)) - 256;                 -- chevron depth (k - 256) / 4
                        c_dep <= resize(shift_right(v_d, 2), 9);
                        v_g := s_g5(12 downto 3);                                             -- stretch gain
                        if v_g <= 512 then
                            v_d := signed(resize(v_g, 11)) - 256;
                            v_t := shift_left(resize(v_d, 16), 4) - resize(v_d, 16);          -- x15
                            c_e <= resize(shift_right(v_t, 6), 9);
                        else
                            v_d := signed(resize(v_g, 11)) - 512;
                            v_t := shift_left(resize(v_d, 16), 5) + shift_left(resize(v_d, 16), 4) - resize(v_d, 16) - shift_left(resize(v_d, 16), 1);  -- x45
                            c_e <= to_signed(60, 9) + resize(shift_right(v_t, 7), 9);
                        end if;
                    when 21 =>
                        z_m1 <= f_exp2(unsigned(z_e1(4 downto 0)));
                        v_e2m := f_exp2(unsigned(z_e2(4 downto 0)));
                        case z_e2(6 downto 5) is                                               -- band scale (Q6)
                            when "01"   => c_kb <= v_e2m(8 downto 1);
                            when "00"   => c_kb <= resize(v_e2m(8 downto 2), 8);
                            when "11"   => c_kb <= resize(v_e2m(8 downto 3), 8);
                            when others => c_kb <= resize(v_e2m(8 downto 4), 8);
                        end case;
                        v_g := s_gp(12 downto 3);                                             -- body size, flame push
                        if v_g <= 512 then
                            c_kc <= to_signed(461, 11) - signed(resize(shift_right(resize(v_g, 15) * to_unsigned(15, 5), 5), 11));
                        else
                            c_kc <= to_signed(221, 11) - signed(resize(shift_right(resize(v_g - 512, 14) * to_unsigned(15, 5), 4), 11));
                        end if;
                        c_bq <= to_unsigned(7, 5) + resize(v_g(9 downto 6), 5);
                        v_k := C_OC_BLACK;
                        c_ty <= unsigned(v_k(23 downto 16)) & "00";
                        c_tu <= unsigned(v_k(15 downto 8)) & "00"; c_tv <= unsigned(v_k(7 downto 0)) & "00";

                    -- products
                    when 52 =>
                        case z_e1(6 downto 5) is
                            when "01"   => c_dpsi <= resize(f_acc(23 downto 7), 17);
                            when "00"   => c_dpsi <= resize(f_acc(24 downto 8), 17);
                            when "11"   => c_dpsi <= resize(f_acc(25 downto 9), 17);
                            when others => c_dpsi <= resize(f_acc(25 downto 10), 17);
                        end case;
                    when 76  => c_m2 <= f_acc(17 downto 0);
                    when 100 => c_roll <= f_acc(25 downto 8);
                    when 124 => c_ge <= f_acc(19 downto 14);
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
    -- 0), computes the NEXT line; two products on the shared serial multiplier
    -- (17 iterations, result at load + 18):
    --   Rv = 1 - |2 frac(key phase) - 1|; eps = Stretch max(0, Rv - 0.32); the x step (1 - eps) and the
    --   W phase step; the stripe pink weight and the dash bar distance; T' = T(cusp) Chevron; the crescent
    --   gate term 1.1 Rv + 0.15 and width; the key line term Rv - 0.5 - kc
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_f  : signed(11 downto 0);
        variable v_r  : signed(11 downto 0);
        variable v_a  : signed(8 downto 0);
        variable v_aa : signed(8 downto 0);
        variable v_q  : signed(11 downto 0);
        variable v_ep : signed(11 downto 0);
        variable v_om : unsigned(10 downto 0);
        variable v_st : unsigned(12 downto 0);
        variable v_tt : signed(17 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_rk <= c_rk0; e_psi <= c_psi0; e_pc <= to_unsigned(K_PC0, 20);
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_rk  <= e_rk + resize(c_dk, 24);
                e_psi <= e_psi + resize(c_dpsi, 20);
                e_pc  <= e_pc + resize(c_dc, 20);
                e_lc  <= (others => '0');
            elsif e_lc /= 63 then
                e_lc <= e_lc + 1;
            end if;

            -- T ROM (fabric, registered)
            e_t8 <= unsigned(C_OC_T(to_integer(e_ta)));

            l_ld <= '0';

            case to_integer(e_lc) is
                when 1 =>
                    v_f := signed(resize(e_rk(23 downto 13), 12)) - 1024;
                    v_r := to_signed(1023, 12) - abs(v_f);
                    if v_r < 0 then e_rv <= (others => '0'); else e_rv <= unsigned(v_r(9 downto 0)); end if;
                    v_a := signed(resize(e_pc(19 downto 12), 9)) - 128;
                    v_aa := abs(v_a);                                       -- 0 .. 128 (slice: resize would fold 128 to 0)
                    e_ta <= unsigned(v_aa(7 downto 0));
                    e_p10 <= e_psi(19 downto 10);
                when 2 =>
                    -- slot A: max(0, Rv - 328) |E|
                    if e_rv > 328 then l_fa <= resize(e_rv - 328, 16); else l_fa <= (others => '0'); end if;
                    l_fb <= unsigned(abs(c_e)); l_ld <= '1';
                    e_eneg <= c_e(8);
                    -- stripe: pink half, distances to its edges and to the bars (one subtract each)
                    if e_p10 < 502 then e_pk <= '1'; else e_pk <= '0'; end if;
                    if e_p10 < 512 then e_e0 <= e_p10; else e_e0 <= resize(to_unsigned(1024, 11) - resize(e_p10, 11), 10); end if;
                    if e_p10 >= 502 then e_e1 <= e_p10 - 502; else e_e1 <= 502 - e_p10; end if;
                    if e_p10 >= 63 then e_b0 <= e_p10 - 63; else e_b0 <= 63 - e_p10; end if;
                    if e_p10 >= 242 then e_b1 <= e_p10 - 242; else e_b1 <= 242 - e_p10; end if;
                    if e_p10 >= 420 then e_b2 <= e_p10 - 420; else e_b2 <= 420 - e_p10; end if;
                    if e_p10 >= 512 then e_b3 <= resize(to_unsigned(1087, 11) - resize(e_p10, 11), 10);   -- the next cycle's first bar
                    else e_b3 <= (others => '1'); end if;
                    -- crescent gate and key line terms
                    n_gl <= e_rv + resize(e_rv(9 downto 4), 10) + resize(e_rv(9 downto 5), 10) + 154;
                    n_sl <= signed(resize(e_rv, 12)) - resize(c_kc, 12) - 512;
                when 3 =>
                    if e_e0 < e_e1 then e_dd <= e_e0; else e_dd <= e_e1; end if;
                    if e_b1 < e_b0 then e_b0 <= e_b1; end if;
                    if e_b3 < e_b2 then e_b2 <= e_b3; end if;
                when 4 =>
                    if e_b0 < e_b2 then e_db <= e_b0; else e_db <= e_b2; end if;
                    if e_pk = '1' then v_q := signed(resize(e_dd, 12)) + 68;
                    else v_q := 68 - signed(resize(e_dd, 12)); end if;
                    if v_q < 0 then v_q := (others => '0'); elsif v_q > 136 then v_q := to_signed(136, 12); end if;
                    if unsigned(v_q(7 downto 4)) >= 7 then n_w3 <= "111"; else n_w3 <= unsigned(v_q(6 downto 4)); end if;
                when 5 =>
                    -- the bar glow (spills past the pink half); the crescent gate width follows the stripe's pink
                    -- weight: 256 (grey) .. 634 (pink), 54 per step
                    if e_db(9 downto 7) = "000" then n_gv <= unsigned(C_OC_GV(to_integer(e_db(6 downto 1))));
                    else n_gv <= (others => '0'); end if;
                    n_gw <= to_unsigned(256, 10) + shift_left(resize(n_w3, 10), 5) + shift_left(resize(n_w3, 10), 4)
                            + shift_left(resize(n_w3, 10), 2) + shift_left(resize(n_w3, 10), 1);
                when 20 =>
                    -- eps (Q10) -> 1024 - eps; slot B: T8 |Chevron|
                    if e_eneg = '1' then v_ep := -signed(resize(f_acc(19 downto 8), 12));
                    else v_ep := signed(resize(f_acc(19 downto 8), 12)); end if;
                    v_ep := to_signed(1024, 12) - v_ep;
                    v_om := unsigned(v_ep(10 downto 0));
                    e_om <= v_om;
                    l_fa <= resize(e_t8, 16); l_fb <= unsigned(abs(c_dep)); l_ld <= '1';
                when 21 =>
                    -- x step = xs (1024 - eps); SD: xs of pixel 0 = 480 (1024 - eps)
                    case to_integer(s_xs) is
                        when 4      => v_st := shift_left(resize(e_om, 13), 2);
                        when 3      => v_st := shift_left(resize(e_om, 13), 1) + resize(e_om, 13);
                        when others => v_st := shift_left(resize(e_om, 13), 1);
                    end case;
                    e_st <= v_st;
                    if s_x0 /= 0 then
                        e_x0 <= shift_left(resize(e_om, 20), 9) - shift_left(resize(e_om, 20), 5);
                    else
                        e_x0 <= (others => '0');
                    end if;
                when 22 =>
                    -- W phase step = x step x 18 (Q24 per pixel, 910 h per cycle); W phase of pixel 0
                    n_st  <= e_st;
                    n_ths <= resize(shift_left(resize(e_st, 17), 4) + shift_left(resize(e_st, 17), 1), 17);
                    n_x0  <= resize(e_x0, 24);
                    n_th0 <= resize(shift_left(resize(e_x0, 24), 4) + shift_left(resize(e_x0, 24), 1), 24) + to_unsigned(K_TH0, 24);
                when 39 =>
                    -- T' = T8 Chevron / 64
                    v_tt := signed(resize(f_acc(16 downto 0), 18));
                    if c_dep(8) = '1' then v_tt := -v_tt; end if;
                    n_tp <= resize(shift_right(v_tt, 6), 11);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the stretched x (Q10 h) and the W phase (Q24)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_xs <= n_x0;
                b_th <= n_th0;
                b_px <= (others => '0');
                l_st <= n_st; l_ths <= n_ths; l_tp <= n_tp; l_sl <= n_sl;
                l_gl <= n_gl; l_gw <= n_gw; l_w3 <= n_w3; l_gv <= n_gv;
            elsif g_avid = '1' then
                b_xs <= b_xs + resize(l_st, 24);
                b_th <= b_th + resize(l_ths, 24);
                b_px <= b_px + 1;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_l4  : unsigned(9 downto 0);
        variable v_phi : signed(15 downto 0);
        variable v_s   : signed(13 downto 0);
        variable v_d   : signed(8 downto 0);
        variable v_g   : unsigned(9 downto 0);
        variable v_ke  : signed(13 downto 0);
        variable v_bl, v_in : std_logic;
        variable v_y, v_u, v_v : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: table addresses (2 virtual px per PHI entry, clamped), W phase; crescent gate phase
            if b_xs(23 downto 22) /= "00" then r_phi_a <= (others => '1');
            else r_phi_a <= b_xs(21 downto 12); end if;
            r_w_a <= b_th(23 downto 16);
            p2_tg <= b_th(23 downto 14) + l_gl;
            -- 3: EBR read; luma taps (4-pixel box)
            p3_tg <= p2_tg;
            p2_l1 <= unsigned(s_lb_q); p2_l2 <= p2_l1; p2_l3 <= p2_l2;
            v_l4 := resize(unsigned(s_lb_q), 10) + resize(p2_l1, 10) + resize(p2_l2, 10) + resize(p2_l3, 10);
            p3_lm <= signed(resize(v_l4(9 downto 2), 9)) - signed(resize(c_loff, 9));
            -- 4: EBR data -> fabric
            p4_phi <= r_phi_q; p4_w <= unsigned(r_w_q);
            p4_tg <= p3_tg; p4_lm <= p3_lm;
            -- 5: products: Bands PHIP; DPHI T'; the scallop 5/4 W; luma x etch
            p5_m1 <= unsigned(p4_phi(19 downto 8)) * c_kb;
            p5_m2 <= signed('0' & p4_phi(7 downto 0)) * l_tp;
            p5_wk <= resize(shift_right(resize(p4_w, 11) + shift_left(resize(p4_w, 11), 2), 2), 10);
            p5_pe <= resize(p4_lm * signed('0' & c_ge), 16);
            p5_tg <= p4_tg;
            -- 6: band phase (Q8 cycles, fraction); key s (Q10)
            v_phi := signed(resize(p5_m1(19 downto 6), 16)) + resize(shift_right(p5_m2, 8), 16);
            if s_src = '1' then                                     -- the Osc: 16x (phase) / 8x (key) the camera
                v_phi := v_phi + resize(shift_right(p5_pe, 2), 16);
                v_s := resize(l_sl, 14) + signed(resize(p5_wk, 14)) + resize(shift_right(p5_pe, 2), 14);
            else
                v_phi := v_phi + resize(shift_right(p5_pe, 6), 16);
                v_s := resize(l_sl, 14) + signed(resize(p5_wk, 14)) + resize(shift_right(p5_pe, 5), 14);
            end if;
            p6_phi <= unsigned(v_phi(7 downto 0));
            if v_s > 4095 then p6_s <= to_signed(4095, 13);
            elsif v_s < -4096 then p6_s <= to_signed(-4096, 13);
            else p6_s <= resize(v_s, 13); end if;
            p6_tg <= p5_tg;
            -- 7: flame profile F = max(0, 56 - |phi - 53|); crescent window and gate; Invert
            v_d := signed(resize(p6_phi, 9)) - 53;
            if abs(v_d) >= 56 then p7_fz <= (others => '0');
            else p7_fz <= resize(unsigned(56 - abs(v_d)), 6); end if;
            p7_p8 <= p6_phi;
            p7_gh <= unsigned(C_OC_GH(to_integer(p6_phi(7 downto 2))));                -- dash window over the phase
            if p6_phi(7 downto 2) < 31 then p7_sal <= "00";                             -- salmon core steps
            elsif p6_phi(7 downto 2) < 34 then p7_sal <= "01";
            elsif p6_phi(7 downto 2) < 37 then p7_sal <= "10";
            else p7_sal <= "11"; end if;
            if p6_phi(7 downto 2) >= 34 and p6_phi(7 downto 2) < 62 then p7_win <= '1'; else p7_win <= '0'; end if;
            if s_fld = '1' or p6_tg < l_gw then p7_gt <= '1'; else p7_gt <= '0'; end if;
            if s_inv = '1' then p7_s <= -p6_s; else p7_s <= p6_s; end if;
            -- 8: flame push product; dash brightness = bar glow x window (no glow in the body's edge margin)
            p8_bf <= p7_fz * c_bq;
            if p7_s < 40 then p8_b <= (others => '0'); else p8_b <= l_gv * p7_gh; end if;
            p8_sal <= p7_sal;
            p8_s <= p7_s; p8_p6 <= p7_p8(7 downto 2);
            p8_win <= p7_win; p8_gt <= p7_gt;
            -- 9: key decisions -> palette address (0 w3 phi6 zebra | 10 s2 b6 dash glow | 1100 b6 crescents)
            v_ke := resize(p8_s, 14) - signed(resize(p8_bf(10 downto 2), 14));
            if v_ke > 0 then v_bl := '1'; else v_bl := '0'; end if;
            if p8_s > 0 then v_in := '1'; else v_in := '0'; end if;
            p9_fl <= v_in and not v_bl;
            if v_bl = '1' then
                if p8_win = '1' and p8_gt = '1' then
                    r_pal_a <= "1100" & p8_b(11 downto 6);
                else
                    r_pal_a <= "10" & p8_sal & p8_b(11 downto 6);
                end if;
            else
                r_pal_a <= '0' & l_w3 & p8_p6;
            end if;
            -- 10: palette ROM (EBR)
            p10_fl <= p9_fl;
            -- 11: fabric copy
            p11_pal <= r_pal_q; p11_fl <= p10_fl;
            -- 12: flame red; Ice (U <-> V); 10 bits
            if p11_fl = '1' then
                v_y := unsigned(C_OC_RED(23 downto 16)); v_u := unsigned(C_OC_RED(15 downto 8)); v_v := unsigned(C_OC_RED(7 downto 0));
            else
                v_y := unsigned(p11_pal(23 downto 16)); v_u := unsigned(p11_pal(15 downto 8)); v_v := unsigned(p11_pal(7 downto 0));
            end if;
            p12_y <= v_y & "00";
            if s_ice = '1' then p12_u <= v_v & "00"; p12_v <= v_u & "00";
            else p12_u <= v_u & "00"; p12_v <= v_v & "00"; end if;
            -- 13, 14: pipeline
            p13_y <= p12_y; p13_u <= p12_u; p13_v <= p12_v;
            p14_y <= p13_y; p14_u <= p13_u; p14_v <= p13_v;
            -- 15: filter input, black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= c_ty; s_u15 <= c_tu; s_v15 <= c_tv;
            else
                s_y15 <= p14_y; s_u15 <= p14_u; s_v15 <= p14_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
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
            cr(to_integer(rc + 6)) <= std_logic_vector(s_u15) & std_logic_vector(s_v15);
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            -- sync ring: written at rc + (C_LAT - 2), RAM register, fabric register = C_LAT clocks
            sr(to_integer(rc + (C_LAT - 2))) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n;
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
            -- 16: chroma running 8-sums (snap back to the off-avid colour once 8 blank samples are in, so a
            -- glitched sample can't offset the colours for good); luma taps
            if s_bah = "00000000" then
                s_bau <= c_tu & "000"; s_bav <= c_tv & "000";
            else
                s_bau <= s_bau + s_u15 - unsigned(cr_f(19 downto 10));
                s_bav <= s_bav + s_v15 - unsigned(cr_f(9 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            s_ly(0) <= s_y15;
            for i in 1 to 5 loop s_ly(i) <= s_ly(i - 1); end loop;
            -- 17: chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4 (both centred 4 pixels back)
            v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            v_l := resize(s_ly(3), 12) + shift_left(resize(s_ly(4), 12), 1) + resize(s_ly(5), 12);
            s_y17 <= v_l(11 downto 2);
            -- 18: luma 13/16
            if s_y17 > 64 then v_dl := s_y17 - 64; else v_dl := (others => '0'); end if;
            s_y18 <= s_y17 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u18 <= s_u17; s_v18 <= s_v17;
            -- 19, 20: pipeline
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
            -- 21: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y21 <= s_y20; s_u21 <= s_u20; s_v21 <= s_v20;
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y21);
    data_out.u       <= std_logic_vector(s_u21);
    data_out.v       <= std_logic_vector(s_v21);
    data_out.hsync_n <= sr_f(2);
    data_out.vsync_n <= sr_f(1);
    data_out.field_n <= sr_f(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture ocelot;
