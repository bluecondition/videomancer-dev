-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Herringbone v0.1.0 -- Heirloom (phase 3, video clips), from the still WIN_20231119_00_47_47 at 0.5 s.
--
-- The still: horizontal bands that alternate lavender / blue and cyan / navy, each filled with soft
-- vertical stripes that lean one way in one band and the other way in the next (a herringbone zigzag,
-- slight at the left edge, steep at the right).  The bands are thin at the top, two tall ones sit in the
-- lower middle, thin again at the bottom; the stripes widen down the frame.  In the clip the tall-band
-- zone rolls upward (~94 lines/s) and the stripes swing as the bands pass under them.
--
-- The patch, as fitted (herringbone.py has every constant):
--   band VCO  theta(y) = theta(0) + sum f, f = fmax 2^(-D (1 + cos(2 pi psi)) / 2), psi = psi0 + y / 1118
--             (a slow, frame-rate modulator that rolls); band A (lavender / blue) where frac(theta) < 1/2
--   stripes   phase = (x - d(x) + 178) / p(y) + C,  p(y) = A (1 + g y / 1080 - z T(theta))
--             T = the band VCO's triangle (the zigzag), d(x) = the left-edge squeeze
--   edges     square wave through two asymmetric one-pole stages (soft rise, long falling tail)
--   colour    1-D colouriser over v = stripe (0..2) + band A (+1): navy, blue, cyan, lavender
--
--   K1  Stripes   stripe period: x1/4 .. x4 (512 = the still, 169 px at the top)
--   K2  Fan       how the stripes widen going down: narrower (left) / even (512) / the still (772) / 3x (1023)
--   K3  Bands     the band VCO's top rate: x1/4 .. x4 (512 = the still, ~48-line bands)
--   K4  Roll      bipolar: the tall-band zone rolls up (+) or down (-); 693 = the clip's 94 lines/s
--   K5  Bunch     the band VCO's FM depth: even bands (0) .. the still (512, 2.72 octaves) .. 5.4 octaves
--   K6  Etch      how far the camera (or the Osc) pushes the stripes sideways
--   S7  Mirror    Off (the still) / On: the right half mirrored onto the left (chevrons meet at the centre)
--   S8  Colours   Blue (the still) / Ember
--   S9  Weave     Off / On: every cyan-navy band's stripes shifted half a period (a woven check)
--   S10 Wave      Square (the still) / Ramp: each stripe a gradient through the whole palette
--   S11 Source    Video / Osc
--   P12 ZIGZAG    the herringbone depth: straight stripes (0) .. the still (337) .. deep chevrons (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each
-- line start that computes the NEXT line (band VCO step through a cos ROM and an exp2 ROM, the zigzag
-- period, one serial divider for the stripe rate, the line-start phase), a beam phase accumulator with the
-- left-edge squeeze, a 21-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).
-- Generated avid; output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16)
-- before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.herringbone_pkg.all;
use work.all;

architecture herringbone of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- fitted constants (keep in step with imodel.py)
    constant C_A0  : integer := 2702;       -- stripe period at the top, Q4 virtual px (168.9)
    constant C_TH0 : integer := 11620000;   -- band VCO phase at line 0, Q24
    constant C_PH  : integer := 6655580;    -- stripe phase constant, Q24
    constant C_EB  : integer := 4452;       -- f exponent base, Q8 octaves (f_q24 = 2^(e / 256))
    constant C_PSI : integer := 451042;     -- modulator phase at power-up, Q20

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
    -- log2(lines step), Q8 octaves: the band rate per output line
    function f_eys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(551, 10);
            when 4          => return to_unsigned(300, 10);
            when 8          => return to_unsigned(488, 10);
            when 12         => return to_unsigned(232, 10);
            when 5 | 6 | 14 => return to_unsigned(150, 10);
            when 1 | 2 | 10 => return to_unsigned(256, 10);
            when others     => return to_unsigned(0, 10);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- line buffer: smoothed (luma >> 2) - 16
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    signal s_ls    : unsigned(9 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a, r_el_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_el_q : unsigned(15 downto 0) := (others => '0');
    signal r_cs_a : unsigned(8 downto 0) := (others => '0');
    signal r_cs_q : unsigned(9 downto 0) := (others => '0');
    signal r_pl_a : unsigned(8 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 320 x 270 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(204, 9);
    signal s_oys : unsigned(10 downto 0) := to_unsigned(242, 11);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_eysr : unsigned(9 downto 0) := (others => '0');
    signal s_x0   : unsigned(8 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(772 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(337 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(693, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_mir, s_pal, s_weave, s_ramp, s_src : std_logic := '0';

    -- per-field constants
    signal z_sh1  : signed(1 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_d2   : unsigned(9 downto 0) := (others => '0');
    signal z_k2   : unsigned(15 downto 0) := (others => '0');
    signal z_hi   : std_logic := '0';
    signal z_p1   : unsigned(27 downto 0) := (others => '0');
    signal z_p2   : unsigned(19 downto 0) := (others => '0');
    signal z_zq   : unsigned(14 downto 0) := (others => '0');
    signal z_vsq  : unsigned(17 downto 0) := (others => '0');
    signal z_k6   : unsigned(19 downto 0) := (others => '0');
    signal c_a    : unsigned(13 downto 0) := to_unsigned(C_A0, 14);          -- stripe period, Q4 px
    signal c_gq   : unsigned(13 downto 0) := (others => '0');                -- |fan| Q12
    signal c_bneg : std_logic := '0';
    signal c_bq   : unsigned(21 downto 0) := (others => '0');                -- fan: Q4 px per line, Q16
    signal c_z    : unsigned(11 downto 0) := to_unsigned(73, 12);            -- zigzag amplitude, Q4 px
    signal c_d    : unsigned(10 downto 0) := to_unsigned(689, 11);           -- FM depth, Q8 octaves
    signal c_e0   : unsigned(12 downto 0) := to_unsigned(C_EB, 13);          -- f exponent at m = 0
    signal c_dpsi : unsigned(12 downto 0) := to_unsigned(938, 13);           -- modulator step per line, Q20
    signal c_dpsi0 : unsigned(13 downto 0) := (others => '0');               -- roll per field, Q20
    signal c_psi0 : unsigned(19 downto 0) := to_unsigned(C_PSI, 20);
    signal c_eg   : unsigned(12 downto 0) := (others => '0');                -- etch gain per luma step >> 4
    signal c_n    : unsigned(29 downto 0) := to_unsigned(2 ** 28, 30);       -- divider numerator
    signal c_x0   : unsigned(12 downto 0) := to_unsigned(292, 13);           -- line-start offset, Q1 px
    signal c_lo   : std_logic := '0';                                        -- 720p / SD edge filter
    signal c_xb0  : unsigned(11 downto 0) := (others => '0');                -- beam start x (h)
    signal c_sel0 : unsigned(1 downto 0) := "00";
    signal c_ky, c_ku, c_kv : unsigned(9 downto 0) := to_unsigned(512, 10);  -- off-avid colour

    -- field serial multiplier
    signal f_a   : unsigned(37 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(37 downto 0) := (others => '0');
    signal z_fa  : unsigned(37 downto 0) := (others => '0');
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
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_psi  : unsigned(19 downto 0) := (others => '0');
    signal e_th   : unsigned(23 downto 0) := (others => '0');
    signal e_f    : unsigned(21 downto 0) := (others => '0');
    signal e_scnt : unsigned(3 downto 0) := (others => '0');
    signal e_shc  : unsigned(3 downto 0) := (others => '0');
    signal e_m    : unsigned(9 downto 0) := (others => '0');
    signal e_dm   : unsigned(10 downto 0) := (others => '0');
    signal e_e    : unsigned(12 downto 0) := (others => '0');
    signal e_mt   : unsigned(15 downto 0) := (others => '0');
    signal e_ba   : std_logic := '0';
    signal e_tq   : unsigned(10 downto 0) := (others => '0');
    signal e_vyi  : unsigned(10 downto 0) := (others => '0');
    signal e_fan  : unsigned(20 downto 0) := (others => '0');
    signal e_fs   : signed(18 downto 0) := (others => '0');
    signal e_p0   : signed(18 downto 0) := (others => '0');
    signal e_zt   : unsigned(12 downto 0) := (others => '0');
    signal e_p1   : signed(18 downto 0) := (others => '0');
    signal e_fq   : unsigned(22 downto 0) := (others => '0');
    signal e_px   : unsigned(23 downto 0) := (others => '0');
    signal m_a    : unsigned(37 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(37 downto 0) := (others => '0');
    signal d_den  : unsigned(15 downto 0) := to_unsigned(128, 16);
    signal d_num  : unsigned(29 downto 0) := (others => '0');
    signal d_rem  : unsigned(16 downto 0) := (others => '0');
    signal d_q    : unsigned(22 downto 0) := (others => '0');
    signal d_cnt  : unsigned(4 downto 0) := (others => '0');
    signal n_ph0  : unsigned(23 downto 0) := (others => '0');
    signal n_fq, n_f1, n_f2, n_fn : unsigned(23 downto 0) := (others => '0');
    signal n_ba   : std_logic := '0';

    -- beam
    signal b_ph  : unsigned(23 downto 0) := (others => '0');
    signal b_xn  : unsigned(11 downto 0) := (others => '0');
    signal b_sel : unsigned(1 downto 0) := "00";
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_fq, l_f1, l_f2, l_fn : unsigned(23 downto 0) := (others => '0');
    signal l_ba  : std_logic := '0';

    -- pixel path
    signal p2_ph, p3_ph, p4_ph, p5_ph, p6_ph : unsigned(23 downto 0) := (others => '0');
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(16 downto 0) := (others => '0');
    signal p5_et  : unsigned(20 downto 0) := (others => '0');
    signal p7_t   : unsigned(13 downto 0) := (others => '0');
    signal p8_s   : unsigned(13 downto 0) := (others => '0');
    signal p8_k   : std_logic := '0';                 -- unused direction flag of stage 1 (symmetric)
    signal p9_s   : unsigned(13 downto 0) := (others => '0');
    signal p9_up  : std_logic := '0';
    signal p12_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal p13_q, p14_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
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
            r_ex_q <= C_HB_EXP2(to_integer(r_ex_a));
            r_el_q <= C_HB_EXP2(to_integer(r_el_a));
            r_cs_q <= C_HB_COS(to_integer(r_cs_a));
            r_pl_q <= C_HB_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(10 downto 0);
        variable v_k  : std_logic_vector(31 downto 0);
        variable v_g  : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            s_tid  <= registers_in(8)(3 downto 0);
            s_ys   <= f_vstep_y(s_tid);
            s_xsr  <= f_vstep_x(s_tid);
            s_eysr <= f_eys(s_tid);

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
            -- roll: bipolar, +-8 deadband, |k4 - 512| - 8
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
                s_mir   <= registers_in(6)(0);
                s_pal   <= registers_in(6)(1);
                s_weave <= registers_in(6)(2);
                s_ramp  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(408, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(306, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(204, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 20) * to_unsigned(242, 8), 8), 11);

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

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from
            -- load + 17 (stores at load + 18).  The slot number is decoded at 30 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 31 + 24 k, so no zc decode sits on the multiplier's path.
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
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(C_A0, 38);          z_fb <= z_mt;                       -- A
                    when 1  => z_fa <= resize(z_d2, 38);               z_fb <= z_k2;                       -- |fan| Q12
                    when 2  => z_fa <= resize(c_a, 38);                z_fb <= resize(c_gq, 16);           -- A |g|
                    when 3  => z_fa <= resize(z_p1, 38);               z_fb <= to_unsigned(971, 16);       -- Bq
                    when 4  => z_fa <= resize(s_gp(12 downto 3), 38);  z_fb <= resize(s_gp(12 downto 3), 16);  -- p12^2
                    when 5  => z_fa <= resize(z_p2, 38);               z_fb <= to_unsigned(1026, 16);      -- z
                    when 6  => z_fa <= resize(c_a, 38);                z_fb <= resize(z_zq, 16);           -- Z
                    when 7  => z_fa <= resize(s_g5(12 downto 3), 38);  z_fb <= to_unsigned(1379, 16);      -- D
                    when 8  => z_fa <= resize(s_vm, 38);               z_fb <= resize(s_vm, 16);           -- speed^2
                    when 9  => z_fa <= resize(z_vsq, 38);              z_fb <= to_unsigned(3239, 16);      -- roll step
                    when 10 => z_fa <= resize(s_ys, 38);               z_fb <= to_unsigned(3748, 16);      -- psi step
                    when 11 => z_fa <= resize(s_g6(12 downto 3), 38);  z_fb <= resize(s_g6(12 downto 3), 16);  -- etch^2
                    when others => z_fa <= resize(z_k6, 38);           z_fb <= to_unsigned(4789, 16);      -- etch gain
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(36 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- roll (last field's step), stripe exponent, mode constants
                    when 20 =>
                        if s_vneg = '1' then c_psi0 <= c_psi0 - resize(c_dpsi0, 20);
                        else c_psi0 <= c_psi0 + resize(c_dpsi0, 20); end if;
                        v_e := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh1 <= v_e(9 downto 8);
                        c_e0 <= resize(s_g3(12 downto 3), 13) + resize(s_eysr, 13) + to_unsigned(C_EB - 512, 13);
                        c_xb0 <= resize(s_x0, 12);
                        if s_xs = 2 then c_lo <= '0'; else c_lo <= '1'; end if;
                        case to_integer(s_xs) is
                            when 4 =>
                                c_n <= to_unsigned(2 ** 29, 30);
                                if s_mir = '1' then c_x0 <= to_unsigned(1858, 13); else c_x0 <= to_unsigned(413, 13); end if;
                            when 3 =>
                                c_n <= to_unsigned(3 * 2 ** 27, 30);
                                if s_mir = '1' then c_x0 <= to_unsigned(2797, 13); else c_x0 <= to_unsigned(195, 13); end if;
                            when others =>
                                c_n <= to_unsigned(2 ** 28, 30);
                                if s_mir = '1' then c_x0 <= to_unsigned(4196, 13); else c_x0 <= to_unsigned(292, 13); end if;
                        end case;
                        if s_pal = '1' then v_k := C_HB_K1; else v_k := C_HB_K0; end if;
                        c_ky <= unsigned(v_k(31 downto 22)); c_ku <= unsigned(v_k(21 downto 12)); c_kv <= unsigned(v_k(11 downto 2));
                    when 21 =>
                        -- fan: d = k2 - 520 (two slopes, the still at 772), or 504 - k2 (narrowing)
                        v_g := resize(s_g2(12 downto 3), 13);
                        if v_g >= 520 then
                            c_bneg <= '0';
                            if v_g > 772 then z_d2 <= resize(v_g - 772, 10); z_k2 <= to_unsigned(25877, 16); z_hi <= '1';
                            else z_d2 <= resize(v_g - 520, 10); z_k2 <= to_unsigned(7513, 16); z_hi <= '0'; end if;
                        elsif v_g <= 504 then
                            c_bneg <= '1'; z_d2 <= resize(504 - v_g, 10); z_k2 <= to_unsigned(7075, 16); z_hi <= '0';
                        else
                            c_bneg <= '0'; z_d2 <= (others => '0'); z_k2 <= (others => '0'); z_hi <= '0';
                        end if;
                        -- beam: segment of the first pixel
                        if s_mir = '1' then c_sel0 <= "11";
                        elsif s_x0 >= 420 then c_sel0 <= "01";
                        else c_sel0 <= "00"; end if;
                    when 23 => z_mt <= r_ex_q;
                    when 50 =>                       -- A = 2707 mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh1) is
                            when -2     => c_a <= resize(f_acc(31 downto 17), 14);
                            when -1     => c_a <= resize(f_acc(31 downto 16), 14);
                            when 0      => c_a <= resize(f_acc(31 downto 15), 14);
                            when others => c_a <= resize(f_acc(31 downto 14), 14);
                        end case;
                    when 74 =>
                        if z_hi = '1' then c_gq <= resize(f_acc(23 downto 10), 14) + 1849;
                        else c_gq <= resize(f_acc(23 downto 10), 14); end if;
                    when 98  => z_p1 <= f_acc(27 downto 0);
                    when 122 => c_bq <= f_acc(37 downto 16);
                    when 146 => z_p2 <= f_acc(19 downto 0);
                    when 170 => z_zq <= f_acc(30 downto 16);
                    when 194 => c_z <= f_acc(27 downto 16);
                    when 218 => c_d <= f_acc(20 downto 10);
                    when 242 => z_vsq <= f_acc(17 downto 0);
                    when 266 => c_dpsi0 <= f_acc(29 downto 16);
                    when 290 => c_dpsi <= f_acc(22 downto 10);
                    when 314 => z_k6 <= f_acc(19 downto 0);
                    when 338 => c_eg <= f_acc(32 downto 20);
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
    -- write of (luma >> 2) - 16, smoothed along the line (1/4 per pixel)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(10 downto 0);
        variable v_n : signed(10 downto 0);
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
            if s_src = '1' then
                v_l := s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then v_l := v_l - 16; else v_l := (others => '0'); end if;
            end if;
            -- smoothing (Q2): ls += (l - ls) / 4
            v_d := signed(resize(v_l & "00", 11)) - signed(resize(s_ls, 11));
            v_n := signed(resize(s_ls, 11)) + shift_right(v_d, 2);
            s_ls <= unsigned(v_n(9 downto 0));
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            s_lb_wd <= std_logic_vector(s_ls(9 downto 2));
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line: the band VCO (cos ROM -> D m -> exp2 ROM ->
    -- shifter -> theta), the zigzag period p = A + Z + fan - Z Tq / 1024, the
    -- stripe rate Fq = N / p (serial divider) and the line-start phase.
    -- One serial multiplier (16 iterations, result at load + 17).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_r : unsigned(16 downto 0);
        variable v_p : signed(18 downto 0);
        variable v_x : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 400 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy  <= (others => '0');
                e_psi <= c_psi0;
                e_th  <= to_unsigned(C_TH0, 24);
                e_lc  <= (others => '0');
            elsif g_start = '1' then
                e_vy  <= e_vy + resize(s_ys, 19);
                e_psi <= e_psi + resize(c_dpsi, 20);
                e_th  <= e_th + resize(e_f, 24);
                e_lc  <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(36 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            -- f shifter: f = (mt << 6) >> shc, one bit per clock
            if e_scnt /= 0 then
                e_f <= '0' & e_f(21 downto 1);
                e_scnt <= e_scnt - 1;
            end if;

            -- divider: 30 restoring steps, quotient kept to 23 bits
            if d_cnt /= 0 then
                v_r := d_rem(15 downto 0) & d_num(29);
                d_num <= d_num(28 downto 0) & '0';
                if v_r >= resize(d_den, 17) then
                    d_rem <= v_r - resize(d_den, 17);
                    d_q <= d_q(21 downto 0) & '1';
                else
                    d_rem <= v_r;
                    d_q <= d_q(21 downto 0) & '0';
                end if;
                d_cnt <= d_cnt - 1;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    v_x := e_psi + 1024;
                    r_cs_a <= v_x(19 downto 11);
                    e_ba <= not e_th(23);
                    if e_th(23) = '0' then e_tq <= not e_th(22 downto 12); else e_tq <= e_th(22 downto 12); end if;
                    e_vyi <= e_vy(18 downto 8);
                when 3 =>
                    e_m <= r_cs_q;
                when 4 =>
                    m_a <= resize(c_d, 38); m_b <= resize(e_m, 16); m_acc <= (others => '0');
                when 21 =>
                    e_dm <= resize(shift_right(m_acc(20 downto 0) + 512, 10), 11);
                    m_a <= resize(c_bq, 38); m_b <= resize(e_vyi, 16); m_acc <= (others => '0');
                when 22 =>
                    e_e <= c_e0 - resize(e_dm, 13);
                when 23 =>
                    r_el_a <= e_e(7 downto 0);
                    if e_e(12 downto 8) > 21 then e_shc <= (others => '0');
                    elsif e_e(12 downto 8) < 7 then e_shc <= to_unsigned(15, 4);
                    else e_shc <= resize(to_unsigned(21, 5) - e_e(12 downto 8), 4); end if;
                when 25 =>
                    e_mt <= r_el_q;
                when 26 =>
                    e_f <= resize(e_mt & "000000", 22);
                    e_scnt <= e_shc;
                when 38 =>
                    e_fan <= m_acc(36 downto 16);
                    m_a <= resize(c_z, 38); m_b <= resize(e_tq, 16); m_acc <= (others => '0');
                when 39 =>
                    if c_bneg = '1' then e_fs <= -signed(resize(e_fan, 19)); else e_fs <= signed(resize(e_fan, 19)); end if;
                when 40 =>
                    e_p0 <= signed(resize(c_a, 19)) + signed(resize(c_z, 19)) + e_fs;
                when 55 =>
                    e_zt <= m_acc(22 downto 10);
                when 56 =>
                    e_p1 <= e_p0 - signed(resize(e_zt, 19));
                when 57 =>
                    v_p := e_p1;
                    if v_p < 128 then d_den <= to_unsigned(128, 16);
                    elsif v_p > 65535 then d_den <= (others => '1');
                    else d_den <= unsigned(v_p(15 downto 0)); end if;
                when 58 =>
                    d_num <= c_n; d_rem <= (others => '0'); d_q <= (others => '0'); d_cnt <= to_unsigned(30, 5);
                when 89 =>
                    e_fq <= d_q;
                when 90 =>
                    m_a <= resize(e_fq, 38); m_b <= resize(c_x0, 16); m_acc <= (others => '0');
                when 107 =>
                    e_px <= m_acc(24 downto 1);
                when 108 =>
                    if s_weave = '1' and e_ba = '0' then
                        n_ph0 <= e_px + to_unsigned(C_PH + 2 ** 23, 24);
                    else
                        n_ph0 <= e_px + to_unsigned(C_PH, 24);
                    end if;
                    n_fq <= resize(e_fq, 24);
                    n_f1 <= resize(e_fq, 24) + resize(e_fq(22 downto 3), 24);
                    n_f2 <= resize(e_fq, 24) + resize(e_fq(22 downto 5), 24);
                    n_fn <= (not resize(e_fq, 24)) + 1;
                    n_ba <= e_ba;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: phase accumulator.  Steps: x < 210 virtual px: F + F/8, x < 420:
    -- F + F/32 (the left-edge squeeze), else F; Mirror: -F left of the centre.
    -- The step select is registered a clock ahead from the next pixel's x.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_st : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_ph  <= n_ph0;
                b_xn  <= c_xb0 + s_xs;
                b_sel <= c_sel0;
                b_px  <= (others => '0');
                l_fq <= n_fq; l_f1 <= n_f1; l_f2 <= n_f2; l_fn <= n_fn; l_ba <= n_ba;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                case b_sel is
                    when "00"   => v_st := l_f1;
                    when "01"   => v_st := l_f2;
                    when "10"   => v_st := l_fq;
                    when others => v_st := l_fn;
                end case;
                b_ph <= b_ph + v_st;
                b_xn <= b_xn + s_xs;
                if s_mir = '1' then
                    if b_xn < 1920 then b_sel <= "11"; else b_sel <= "10"; end if;
                elsif b_xn < 420 then b_sel <= "00";
                elsif b_xn < 840 then b_sel <= "01";
                else b_sel <= "10"; end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_e  : signed(14 downto 0);
        variable v_sh : signed(14 downto 0);
        variable v_v  : unsigned(8 downto 0);
        variable v_n  : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            -- 2, 3: phase; luma from the line buffer
            p2_ph <= b_ph;
            p3_ph <= p2_ph;
            p3_lm <= unsigned(s_lb_q);
            -- 4: etch partials (luma nibbles x gain)
            p4_ea <= resize(p3_lm(3 downto 0) * c_eg, 17);
            p4_eb <= resize(p3_lm(7 downto 4) * c_eg, 17);
            p4_ph <= p3_ph;
            -- 5: etch
            p5_et <= resize(p4_ea, 21) + shift_left(resize(p4_eb, 21), 4);
            p5_ph <= p4_ph;
            -- 6: phase + etch (mod one cycle)
            p6_ph <= p5_ph + shift_left(resize(p5_et, 24), 4);
            -- 7: target: square (light where frac < 1/2) or ramp (2 .. 0 across the period)
            if s_ramp = '1' then
                p7_t <= to_unsigned(8192, 14) - resize(p6_ph(23 downto 11), 14);
            elsif p6_ph(23) = '0' then
                p7_t <= to_unsigned(8192, 14);
            else
                p7_t <= (others => '0');
            end if;
            -- 8: edge stage 1 (symmetric one-pole, 1/8 per pixel at 1080, 1/4 at 720p / SD); loads the
            --    first pixel's target
            if s_gav_sr(5) = '1' and s_gav_sr(6) = '0' then
                p8_s <= p7_t;
            else
                v_e := signed(resize(p7_t, 15)) - signed(resize(p8_s, 15));
                if c_lo = '1' then v_sh := shift_right(v_e, 2); else v_sh := shift_right(v_e, 3); end if;
                v_n := signed(resize(p8_s, 15)) + v_sh;
                p8_s <= unsigned(v_n(13 downto 0));
            end if;
            -- 9: edge stage 2 (asymmetric: rise 1/4, fall 1/2 at 1080; 1/2 and 1 at 720p / SD); the
            --    direction is the previous pixel's
            if s_gav_sr(6) = '1' and s_gav_sr(7) = '0' then
                p9_s <= p8_s; p9_up <= '0';
            else
                v_e := signed(resize(p8_s, 15)) - signed(resize(p9_s, 15));
                if c_lo = '0' and p9_up = '1' then v_sh := shift_right(v_e, 2);
                elsif c_lo = '1' and p9_up = '0' then v_sh := v_e;
                else v_sh := shift_right(v_e, 1); end if;
                v_n := signed(resize(p9_s, 15)) + v_sh;
                p9_s <= unsigned(v_n(13 downto 0));
                if v_e > 0 then p9_up <= '1'; else p9_up <= '0'; end if;
            end if;
            -- 10: colouriser address: palette, v = stripe (0..128) + band A (64)
            v_v := resize(p9_s(13 downto 6), 9);
            if l_ba = '1' then v_v := v_v + 64; end if;
            r_pl_a <= s_pal & v_v(7 downto 0);
            -- 11: ROM (EBR) -> 12: fabric copy -> 13, 14: delay
            p12_q <= r_pl_q;
            p13_q <= p12_q;
            p14_q <= p13_q;
            -- 15: compose; the off-avid colour outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= c_ky; s_u15 <= c_ku; s_v15 <= c_kv;
            else
                s_y15 <= unsigned(p14_q(31 downto 22)); s_u15 <= unsigned(p14_q(21 downto 12)); s_v15 <= unsigned(p14_q(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the still's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to the off-avid colour (the off-avid input of both boxes) once
            -- 8 blank samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 16 sums, stage 17 output
            s_cay(0) <= s_y15; s_cau(0) <= s_u15; s_cav(0) <= s_v15;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= c_ky & "000"; s_bau <= c_ku & "000"; s_bav <= c_kv & "000";
            else
                s_bay <= s_bay + s_y15 - s_cay(7);
                s_bau <= s_bau + s_u15 - s_cau(7);
                s_bav <= s_bav + s_v15 - s_cav(7);
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: the off-avid colour off the generated avid (what box A gives there anyway)
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= c_ky; s_u17 <= c_ku; s_v17 <= c_kv;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            -- box B: stage 18 sums, stage 19 output
            s_cby(0) <= s_y17; s_cbu(0) <= s_u17; s_cbv(0) <= s_v17;
            for i in 1 to 7 loop s_cby(i) <= s_cby(i - 1); s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bby <= c_ky & "000"; s_bbu <= c_ku & "000"; s_bbv <= c_kv & "000";
            else
                s_bby <= s_bby + s_y17 - s_cby(7);
                s_bbu <= s_bbu + s_u17 - s_cbu(7);
                s_bbv <= s_bbv + s_v17 - s_cbv(7);
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y19 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u19 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v19 <= v_sc(13 downto 4);
            -- 20: luma 13/16
            if s_y19 > 64 then v_dl := s_y19 - 64; else v_dl := (others => '0'); end if;
            s_y20 <= s_y19 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u20 <= s_u19; s_v20 <= s_v19;
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
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture herringbone;
