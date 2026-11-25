-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Subcarrier v0.1.0 -- Onevolt: the screen as the colour plane.
--
-- The rack: LZX Swatch (YIQ -> RGB), Syntonie VU005 sine shapers on a quadrature pair, LZX Factors (I / Q gain and
-- the keyed chroma inversion), LZX Angles + DSG3 (the hue ramp and the figure gradient), LZX Keychain (the figure's
-- key), LZX Ribbons (S8, in the Y path), LZX Proc (hue offset, luma level), LZX TBC2 (the camera's Y), a quadrature
-- LFO (chroma orbital displacement).
--
-- The patch:
--   hue phase = Proc offset (K1) + 31 s LFO + Angles 30 deg ramp or DSG3 |tH - tV| (S10), x P12
--             + DSG3 figure gradient (circle recipe, clamped at r = 760) x K3 (attenuverter)
--   phase -> quadrature shaper -> I+ / Q+ (S7 Out: 0 V on Q) ; Factors gain K2 ; Keychain key (K5 + 13 s LFO) inverts
--   I and Q inside the figure (S9: instead Y is patched into I+)
--   Y = Proc constant (K4) + figure shading + key step, crossfaded to the camera's Y (K6) ; S8 Ribbons digitiser
--   Swatch Clamped: a hue-preserving gamut limiter (table by S7, Y, hue) ; + quadrature LFO on I / Q (19 s)
--
--   K1 Hue  K2 Saturation  K3 Rings  K4 Luma  K5 Size  K6 Camera
--   S7 Q cable In / Out   S8 Y path Proc / Ribbons   S9 Figure Inverted / Luma>Chroma   S10 Ramp Angles / DSG3 Abs
--   S11 Source Video / Osc   P12 SPECTRUM (degrees of hue across the screen: 0 .. 8 turns, square law)
--
-- Engine: vblank-zone sequencer (one 32 x 16 serial multiplier, 13 slots, shared 7-channel glide, LFO reads on the
-- pixel sine ROMs); a per-line engine in hblank (Yh^2 by shift-add, the ramp's line term); per-pixel DDAs (the
-- ramp, x^2 + y^2); a 14-stage pixel path (K3 x the clamped gradient 12 x 10, two sine ROMs, a gamut table, two 8 x 8
-- multiplies, a 12 x 6 multiply for the camera crossfade).  Virtual 1080-line coordinates.  Generated avid; output
-- conditioning (two [1 2 2 2 2 2 2 2 1]/16 boxes on Y, U and V with the per-line resync, luma 13/16, a vertical
-- [1 1]/2 with the line above) before the blanking gate.  Colours in the hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.subcarrier_pkg.all;
use work.all;

architecture subcarrier of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

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
    -- bipolar knob: |k - 512| - 8 outside a +-8 deadband
    function f_bm(k : unsigned(9 downto 0)) return unsigned is
        variable v_s : signed(10 downto 0);
        variable v_m : unsigned(9 downto 0);
    begin
        v_s := signed(resize(k, 11)) - 512;
        if v_s > -9 and v_s < 9 then
            v_m := (others => '0');
        elsif v_s(10) = '1' then
            v_m := unsigned(not v_s(9 downto 0)) - 7;
        else
            v_m := unsigned(v_s(9 downto 0)) - 8;
        end if;
        return v_m(8 downto 0);
    end function;
    -- quarter-wave fold of a 10-bit phase: the ROM index (the sign is a10(9))
    function f_fold(a : unsigned(9 downto 0)) return unsigned is
    begin
        if a(8) = '1' then return not a(7 downto 0); else return a(7 downto 0); end if;
    end function;

    constant C_RGQ : unsigned(22 downto 0) := to_unsigned(2310400, 23);   -- (2 x 760)^2: the ring clamp radius
    constant K_HU  : unsigned(15 downto 0) := to_unsigned(24121, 16);     -- 132.5 deg: U = Cr = s sin(phi + C)
    constant K_HV  : unsigned(15 downto 0) := to_unsigned(40505, 16);     -- 222.5 deg: V = Cb = s sin(phi + C + 90)
    constant K_HO  : unsigned(15 downto 0) := to_unsigned(16384, 16);     -- 90 deg: S7 Out reads cos(phi)

    -- camera cell store: one 12-bit vertical-slew state per 4-pixel cell (read a cell ahead of the write)
    type t_lb is array (0 to 511) of std_logic_vector(15 downto 0);
    signal lb : t_lb := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    signal s_lb_wa, s_lb_ra : unsigned(8 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(15 downto 0) := x"0000";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(15 downto 0) := x"0000";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    signal s_lin   : unsigned(7 downto 0) := (others => '0');
    signal s_lav, s_f1, s_f2 : std_logic := '0';
    signal s_lc    : unsigned(1 downto 0) := (others => '0');
    signal s_lcell, s_fcell, s_wcell : unsigned(8 downto 0) := (others => '0');
    signal s_acc, s_sum4 : unsigned(9 downto 0) := (others => '0');
    signal s_cur, s_old, s_new : unsigned(11 downto 0) := (others => '0');
    signal s_lv    : unsigned(11 downto 0) := (others => '0');
    signal s_h1    : unsigned(13 downto 0) := (others => '0');
    signal s_camy  : unsigned(9 downto 0) := to_unsigned(86, 10);
    signal s_pcnt  : unsigned(11 downto 0) := (others => '0');
    signal s_pbig  : std_logic := '0';
    signal s_pres  : std_logic := '0';

    -- ROMs (registered address and data)
    signal r_su_a, r_sv_a : unsigned(7 downto 0) := (others => '0');
    signal r_su_q, r_sv_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_g_a  : unsigned(10 downto 0) := (others => '0');
    signal r_g_q  : std_logic_vector(7 downto 0) := (others => '0');

    -- output filter stores (as the shell)
    type t_dl is array (0 to 15) of std_logic_vector(31 downto 0);
    signal da, db : t_dl := (others => (others => '0'));
    attribute no_rw_check of da : signal is true;
    attribute no_rw_check of db : signal is true;
    signal s_dp : unsigned(3 downto 0) := to_unsigned(0, 4);
    signal s_dr : unsigned(3 downto 0) := to_unsigned(10, 4);
    signal s_qa, s_qb, s_fa, s_fb : std_logic_vector(31 downto 0) := (others => '0');
    type t_vy is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_vc is array (0 to 511) of std_logic_vector(15 downto 0);
    signal vy : t_vy := (others => x"10");
    signal vc : t_vc := (others => x"8080");
    attribute no_rw_check of vy : signal is true;
    attribute no_rw_check of vc : signal is true;
    signal v_k17, v_k19 : unsigned(10 downto 0) := (others => '0');
    signal v_qy, v_fy, v_wyd : std_logic_vector(7 downto 0) := x"10";
    signal v_qc, v_fc, v_wcd : std_logic_vector(15 downto 0) := x"8080";
    signal v_wya : unsigned(9 downto 0) := (others => '0');
    signal v_wca : unsigned(8 downto 0) := (others => '0');
    signal v_wye, v_wce : std_logic := '0';
    signal v_py, v_pu, v_pv : unsigned(9 downto 0) := to_unsigned(512, 10);

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
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(11 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(680 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(360 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(560 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_qout, s_rib, s_l2c, s_sal, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- LFOs (18-bit phases) and their sine reads
    signal z_l19, z_l31, z_l13 : unsigned(15 downto 0) := (others => '0');
    signal z_lfa, z_lfb : unsigned(7 downto 0) := (others => '0');
    signal z_lfsel : std_logic := '0';
    signal z_lna, z_lnb : std_logic_vector(3 downto 0) := (others => '0');
    signal z_ou, z_ov, z_s31, z_s13 : signed(8 downto 0) := (others => '0');

    -- per-field values
    signal z_m3  : unsigned(8 downto 0) := (others => '0');
    signal z_n3  : std_logic := '0';
    signal c_kr4 : unsigned(9 downto 0) := (others => '0');         -- |K3 ring gain| / 4, sign z_n3
    signal z_w   : unsigned(9 downto 0) := (others => '0');          -- camera presence glide
    signal c_p2  : unsigned(19 downto 0) := (others => '0');
    signal c_sa, c_sb : unsigned(12 downto 0) := (others => '0');
    signal c_a0  : signed(23 downto 0) := (others => '0');
    signal c_ast : unsigned(14 downto 0) := (others => '0');
    signal c_b0  : signed(23 downto 0) := (others => '0');
    signal c_bst : unsigned(16 downto 0) := (others => '0');
    signal c_rk0 : unsigned(10 downto 0) := (others => '0');
    signal c_rkey : unsigned(10 downto 0) := (others => '0');
    signal c_wq  : unsigned(13 downto 0) := (others => '0');
    signal c_tlo, c_thi : unsigned(22 downto 0) := (others => '0');
    signal c_f   : unsigned(5 downto 0) := (others => '0');
    signal c_sf  : unsigned(7 downto 0) := (others => '0');
    signal c_orbu, c_orbv : signed(8 downto 0) := (others => '0');
    signal c_y4, c_y4a, c_y4b : unsigned(9 downto 0) := to_unsigned(172, 10);
    signal c_hoff, c_hu, c_hv : unsigned(15 downto 0) := (others => '0');
    signal c_x0q : unsigned(21 downto 0) := to_unsigned(3686400, 22);
    signal c_dx0 : signed(15 downto 0) := to_signed(-7676, 16);
    signal c_ddx : unsigned(5 downto 0) := to_unsigned(8, 6);

    -- field serial multiply: acc = +- a x b (mod 2^32)
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : unsigned(23 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_ng  : std_logic := '0';
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine (in hblank: the next line's values)
    signal e_y8  : signed(19 downto 0) := to_signed(-276480, 20);
    signal e_bl  : signed(23 downto 0) := (others => '0');
    signal e_run : std_logic := '0';
    signal e_cnt : unsigned(3 downto 0) := (others => '0');
    signal e_ma  : unsigned(21 downto 0) := (others => '0');
    signal e_mb  : unsigned(10 downto 0) := (others => '0');
    signal e_acc : unsigned(21 downto 0) := (others => '0');
    signal e_go, e_gq : std_logic := '0';
    signal l_q   : unsigned(22 downto 0) := (others => '0');
    signal l_b   : signed(23 downto 0) := (others => '0');

    -- beam (stage 1)
    signal b_a   : signed(23 downto 0) := (others => '0');
    signal b_q   : unsigned(22 downto 0) := (others => '0');
    signal b_dx  : signed(15 downto 0) := (others => '0');

    -- pixel path
    signal p2_a  : signed(23 downto 0) := (others => '0');
    signal p2_g  : unsigned(11 downto 0) := (others => '0');
    signal p2_kin, p2_kout : std_logic := '0';
    signal p2_dm : unsigned(7 downto 0) := (others => '0');
    signal p3_r  : signed(24 downto 0) := (others => '0');
    signal p3_gl, p3_gh : unsigned(16 downto 0) := (others => '0');
    signal p3_yp, p4_yp : unsigned(10 downto 0) := (others => '0');
    signal p3_d  : signed(11 downto 0) := (others => '0');
    signal p3_key, p4_key, p5_key : std_logic := '0';
    signal p4_r16 : unsigned(15 downto 0) := (others => '0');
    signal p4_rp : unsigned(21 downto 0) := (others => '0');
    signal p4_xl, p4_xh : signed(14 downto 0) := (others => '0');
    signal p5_t  : unsigned(15 downto 0) := (others => '0');
    signal p5_y  : signed(12 downto 0) := (others => '0');
    signal p6_au, p6_av : unsigned(15 downto 0) := (others => '0');
    signal p6_ph : unsigned(4 downto 0) := (others => '0');
    signal p6_lev : unsigned(2 downto 0) := (others => '0');
    signal p6_ys : unsigned(9 downto 0) := (others => '0');
    signal p7_un, p7_vn, p8_un, p8_vn : std_logic := '0';
    type t_m8 is array (9 to 10) of unsigned(7 downto 0);
    signal pm_u, pm_v : t_m8 := (others => (others => '0'));
    signal pn_u, pn_v : std_logic_vector(12 downto 9) := (others => '0');
    signal p7_yf, p8_yf, p9_yf, p10_yf, p11_yf, p12_yf, p13_yf : unsigned(9 downto 0) := (others => '0');
    signal p9_l  : unsigned(7 downto 0) := (others => '0');
    signal p10_s : unsigned(7 downto 0) := (others => '0');
    signal p11_ulo, p11_uhi, p11_vlo, p11_vhi : unsigned(11 downto 0) := (others => '0');
    signal p11_dy : signed(10 downto 0) := (others => '0');
    signal p12_mu, p12_mv : unsigned(8 downto 0) := (others => '0');
    signal p12_ou, p12_ov : signed(12 downto 0) := (others => '0');
    signal p13_cu, p13_cv : signed(12 downto 0) := (others => '0');
    signal s_y14 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_KU : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KV : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories
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
            r_su_q <= C_SC_SINU(to_integer(r_su_a));
            r_sv_q <= C_SC_SINV(to_integer(r_sv_a));
            r_g_q  <= C_SC_GAM(to_integer(r_g_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_h   : unsigned(15 downto 0);
        variable v_kr  : unsigned(11 downto 0);
        variable v_t   : signed(31 downto 0);
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
            elsif s_zc /= 4095 then
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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_qout <= registers_in(6)(0);
                s_rib  <= registers_in(6)(1);
                s_l2c  <= registers_in(6)(2);
                s_sal  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                s_pres <= s_pbig;
                z_l19 <= z_l19 + 57;
                z_l31 <= z_l31 + 35;
                z_l13 <= z_l13 + 84;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            case to_integer(s_ys) is
                when 1138   => s_oys <= to_unsigned(537, 10);
                when 576    => s_oys <= to_unsigned(272, 10);
                when 960    => s_oys <= to_unsigned(453, 10);
                when 480    => s_oys <= to_unsigned(226, 10);
                when 384    => s_oys <= to_unsigned(181, 10);
                when 512    => s_oys <= to_unsigned(242, 10);
                when others => s_oys <= to_unsigned(121, 10);
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
                            when 3      => s_gm <= s_g4; s_gr <= unsigned(registers_in(3));
                            when 4      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 5      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
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
                            when 3      => s_g4 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 5      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 6 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- LFO sine reads on the pixel ROMs (zone clocks 36 and 40; data registered at 38 / 42, captured at 39 / 43)
            z_lfsel <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 36 =>
                        z_lfa <= f_fold(z_l19(15 downto 6));          z_lfsel <= '1';
                        z_lfb <= f_fold(z_l19(15 downto 6) + 256);
                    when 40 =>
                        z_lfa <= f_fold(z_l31(15 downto 6));          z_lfsel <= '1';
                        z_lfb <= f_fold(z_l13(15 downto 6));
                    when others => null;
                end case;
            end if;
            z_lna <= z_lna(2 downto 0) & '0';
            z_lnb <= z_lnb(2 downto 0) & '0';
            if s_zone = '1' and s_zc = 36 then
                z_lna(0) <= z_l19(15); v_h := resize(z_l19(15 downto 6), 16) + 256; z_lnb(0) <= v_h(9);
            elsif s_zone = '1' and s_zc = 40 then
                z_lna(0) <= z_l31(15); z_lnb(0) <= z_l13(15);
            end if;

            -- serial multiply slots: slot k loads at zc = 48 + 24 k, 16 iterations, result stored at load + 18.
            -- The slot number is decoded at 46 + 24 k and the operands and the sign are pre-muxed at 47 + 24 k.
            z_pre <= '0';
            if s_zone = '1' and s_zc >= 46 and s_zc <= 334 then
                case to_integer(s_zc) is
                    when 46  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 70  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 94  => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 118 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 142 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 166 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 190 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 214 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 238 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 262 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 286 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 310 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when 334 => z_sel <= to_unsigned(12, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_ng <= '0';
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_gp(12 downto 3), 24); z_fb <= resize(s_gp(12 downto 3), 16);   -- P12^2
                    when 1  => z_fa <= resize(c_p2, 24);                                                          -- sA
                               if s_sal = '1' then z_fb <= to_unsigned(1094, 16); else z_fb <= to_unsigned(365, 16); end if;
                    when 2  => z_fa <= resize(c_p2, 24);                                                          -- sB
                               if s_sal = '1' then z_fb <= to_unsigned(1945, 16); else z_fb <= to_unsigned(324, 16); end if;
                    when 3  => z_fa <= resize(c_sa, 24); z_ng <= '1';                                             -- A0 = sA X0
                               if s_xs = 4 then z_fb <= to_unsigned(1440, 16); else z_fb <= to_unsigned(1920, 16); end if;
                    when 4  => z_fa <= resize(c_sb, 24); z_fb <= to_unsigned(1080, 16); z_ng <= '1';             -- B0
                    when 5  => z_fa <= resize(c_sb, 24); z_fb <= resize(s_ys & '0', 16);                          -- B step
                    when 6  => z_fa <= resize(s_g5(12 downto 3), 24); z_fb <= to_unsigned(900, 16);               -- size
                    when 7  => z_fa <= resize(c_rk0, 24); z_ng <= z_s13(8);                                       -- breathing
                               if z_s13(8) = '1' then z_fb <= resize(unsigned(-z_s13), 16); else z_fb <= resize(unsigned(z_s13), 16); end if;
                    when 8  => z_fa <= resize(c_rkey, 24); z_fb <= resize(c_rkey, 16);                            -- rkey^2
                    when 9  => z_fa <= resize(s_g6(12 downto 3), 24); z_fb <= resize(z_w, 16);                    -- camera fade
                    when 10 => z_fa <= resize(s_g2(12 downto 3), 24); z_fb <= to_unsigned(135, 16);               -- saturation
                    when 11 => z_fa <= resize(c_sf, 24); z_ng <= z_ou(8);                                         -- orbit U
                               if z_ou(8) = '1' then z_fb <= resize(unsigned(-z_ou), 16); else z_fb <= resize(unsigned(z_ou), 16); end if;
                    when others => z_fa <= resize(c_sf, 24); z_ng <= z_ov(8);                                     -- orbit V
                               if z_ov(8) = '1' then z_fb <= resize(unsigned(-z_ov), 16); else z_fb <= resize(unsigned(z_ov), 16); end if;
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 32);
                f_b <= z_fb;
            else
                if f_b(0) = '1' then
                    if z_ng = '1' then f_acc <= f_acc - f_a; else f_acc <= f_acc + f_a; end if;
                end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 32 =>                                        -- knob decodes (after the glide)
                        z_m3 <= f_bm(s_g3(12 downto 3)); z_n3 <= not s_g3(12);
                        c_y4 <= to_unsigned(172, 10) + resize(s_g4(12 downto 4), 10) + resize(s_g4(12 downto 6), 10)
                                + resize(s_g4(12 downto 8), 10);
                        case to_integer(s_xs) is
                            when 4      => c_x0q <= to_unsigned(2073600, 22); c_dx0 <= to_signed(-11504, 16); c_ddx <= to_unsigned(32, 6);
                            when 3      => c_x0q <= to_unsigned(3686400, 22); c_dx0 <= to_signed(-11511, 16); c_ddx <= to_unsigned(18, 6);
                            when others => c_x0q <= to_unsigned(3686400, 22); c_dx0 <= to_signed(-7676, 16);  c_ddx <= to_unsigned(8, 6);
                        end case;
                        -- camera presence: a camera (or the Osc) crossfades in at once; a black input fades back to
                        -- the patch's own luma (1/16 per field)
                        if s_pres = '1' or s_src = '1' then
                            z_w <= to_unsigned(1023, 10);
                        elsif z_w > 15 then
                            z_w <= z_w - resize(z_w(9 downto 4), 10);
                        else
                            z_w <= (others => '0');
                        end if;
                    when 33 =>
                        v_kr := shift_left(resize(z_m3, 12), 2) + resize(z_m3, 12) + resize(z_m3(8 downto 1), 12);
                        c_kr4 <= v_kr(11 downto 2);
                        c_y4a <= c_y4 + 37; c_y4b <= c_y4 + 75;
                    when 39 =>                                        -- LFO reads: orbit (sin, cos)
                        if z_lna(2) = '1' then z_ou <= -signed(resize(unsigned(r_su_q(15 downto 8)), 9));
                        else z_ou <= signed(resize(unsigned(r_su_q(15 downto 8)), 9)); end if;
                        if z_lnb(2) = '1' then z_ov <= -signed(resize(unsigned(r_sv_q(15 downto 8)), 9));
                        else z_ov <= signed(resize(unsigned(r_sv_q(15 downto 8)), 9)); end if;
                    when 40 =>
                        z_ou <= shift_right(z_ou, 2);                 -- orbit radius: magnitude / 4 (sign-magnitude)
                        z_ov <= shift_right(z_ov, 2);
                        if z_ou(8) = '1' then z_ou <= -shift_right(-z_ou, 2); end if;
                        if z_ov(8) = '1' then z_ov <= -shift_right(-z_ov, 2); end if;
                    when 43 =>                                        -- hue slide (31 s), breathing (13 s)
                        if z_lna(2) = '1' then z_s31 <= -signed(resize(unsigned(r_su_q(15 downto 8)), 9));
                        else z_s31 <= signed(resize(unsigned(r_su_q(15 downto 8)), 9)); end if;
                        if z_lnb(2) = '1' then z_s13 <= -signed(resize(unsigned(r_sv_q(15 downto 8)), 9));
                        else z_s13 <= signed(resize(unsigned(r_sv_q(15 downto 8)), 9)); end if;
                    -- serial multiplier results (stored at 66 + 24 k)
                    when 66  => c_p2 <= f_acc(19 downto 0);
                    when 90  => c_sa <= f_acc(30 downto 18);
                    when 114 => c_sb <= f_acc(30 downto 18);
                    when 138 => c_a0 <= signed(f_acc(23 downto 0));
                                c_ast <= resize(shift_left(resize(c_sa, 15), 1), 15);
                    when 139 => if s_xs = 3 then c_ast <= c_ast + resize(c_sa, 15);
                                elsif s_xs = 4 then c_ast <= shift_left(resize(c_sa, 15), 2); end if;
                    when 162 => c_b0 <= signed(f_acc(23 downto 0));
                    when 186 => c_bst <= f_acc(24 downto 8);
                    when 210 => c_rk0 <= to_unsigned(70, 11) + f_acc(20 downto 10);
                    when 234 =>
                        v_t := signed(f_acc);
                        v_t := shift_right(v_t, 12);
                        c_rkey <= unsigned(signed(c_rk0) + v_t(10 downto 0));
                    when 235 =>                                       -- the key's 1-px edge band: 2 rkey xs
                        case to_integer(s_xs) is
                            when 4      => c_wq <= shift_left(resize(c_rkey, 14), 3);
                            when 3      => c_wq <= shift_left(resize(c_rkey, 14), 2) + shift_left(resize(c_rkey, 14), 1);
                            when others => c_wq <= shift_left(resize(c_rkey, 14), 2);
                        end case;
                    when 258 => c_tlo <= shift_left(f_acc(22 downto 0), 2) - c_wq;
                                c_thi <= shift_left(f_acc(22 downto 0), 2) + c_wq;
                    when 282 => c_f <= f_acc(19 downto 14);
                    when 306 => if f_acc(17) = '1' then c_sf <= (others => '1'); else c_sf <= f_acc(16 downto 9); end if;
                    when 330 => v_t := -signed(f_acc);
                                if z_ng = '1' then c_orbu <= -v_t(15 downto 7); else c_orbu <= signed(f_acc(15 downto 7)); end if;
                    when 354 => v_t := -signed(f_acc);
                                if z_ng = '1' then c_orbv <= -v_t(15 downto 7); else c_orbv <= signed(f_acc(15 downto 7)); end if;
                    -- hue offset and the read phases
                    when 356 =>
                        v_h := s_g1 & "000";
                        v_h := v_h + unsigned(resize(shift_left(resize(z_s31, 16), 5), 16));
                        if s_sal = '0' then v_h := v_h + c_p2(17 downto 2); end if;
                        c_hoff <= v_h;
                    when 357 =>
                        if s_qout = '1' then
                            c_hu <= c_hoff + K_HO; c_hv <= c_hoff + K_HO;
                        else
                            c_hu <= c_hoff + K_HU; c_hv <= c_hoff + K_HV;
                        end if;
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
    -- Source side (TBC2's Y, or the Osc): the oscillator (2-D triangle lattice,
    -- drifting) or camera luma ((Y >> 2) - 16), summed over 4-pixel cells, each
    -- cell slewed down the screen (one pole, 1/4 per line; the state lives in the
    -- cell store), then slewed across (one pole, 1/4 per pixel).  A per-field
    -- count of bright cells says whether a camera is present at all.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(13 downto 0);
        variable v_e : signed(15 downto 0);
        variable v_c : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- 1: sample
            if s_src = '1' then
                s_lin <= s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lin <= v_l - 16; else s_lin <= (others => '0'); end if;
            end if;
            s_lav   <= data_in.avid;
            s_lc    <= s_wcol(1 downto 0);
            s_lcell <= s_wcol(10 downto 2);
            if data_in.avid = '1' then
                s_wcol  <= s_wcol + 1;
                s_lb_ra <= s_wcol(10 downto 2) + 1;
            else
                s_wcol  <= (others => '0');
                s_lb_ra <= (others => '0');
            end if;
            -- 2: sum four; at the fourth, take the cell's state (read a cell ahead) for the update
            s_f1 <= '0';
            if s_lav = '0' then
                s_cur <= unsigned(s_lb_q(11 downto 0));
            else
                if s_lc = 0 then s_acc <= resize(s_lin, 10); else s_acc <= s_acc + s_lin; end if;
                if s_lc = 3 then
                    s_sum4  <= s_acc + s_lin;
                    s_old   <= s_cur;
                    s_cur   <= unsigned(s_lb_q(11 downto 0));
                    s_fcell <= s_lcell;
                    s_f1    <= '1';
                end if;
            end if;
            -- 3: state += (cell sum * 4 - state) / 4 ; presence count
            v_d := signed(resize(s_sum4 & "00", 14)) - signed(resize(s_old, 14));
            v_d := signed(resize(s_old, 14)) + shift_right(v_d, 2);
            s_new   <= unsigned(v_d(11 downto 0));
            s_wcell <= s_fcell;
            s_f2    <= s_f1;
            if s_zgo = '1' then
                s_pcnt <= (others => '0');
            elsif s_f1 = '1' and s_sum4 > 96 and s_pcnt /= 4095 then
                s_pcnt <= s_pcnt + 1;
            end if;
            if s_pcnt > 1024 then s_pbig <= '1'; else s_pbig <= '0'; end if;
            -- 4: write
            s_lb_we <= s_f2;
            s_lb_wa <= s_wcell;
            s_lb_wd <= "0000" & std_logic_vector(s_new);

            -- pixel side: the cell stream (the line above's state), one slew across, the camera's Swatch Y
            s_lv <= unsigned(s_lb_q(11 downto 0));
            v_e  := signed(resize(s_lv & "00", 16)) - signed(resize(s_h1, 16));
            v_e  := signed(resize(s_h1, 16)) + shift_right(v_e, 2);
            s_h1 <= unsigned(v_e(13 downto 0));
            v_c  := s_h1(13 downto 2);
            s_camy <= to_unsigned(86, 10) + resize(v_c(11 downto 2), 10) + resize(v_c(11 downto 7), 10);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine (in hblank, after each active line): advance y8 and the ramp's
    -- line term, then QY = Yh^2 for the next line (Yh = y8 >> 8, half virtual px)
    -- by shift-add in 11 steps; the ramp term is |B| under DSG3 Abs, B under
    -- Angles.  Line 0 is computed in the zone.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_yh : signed(11 downto 0);
        variable v_nb : signed(23 downto 0);
    begin
        if rising_edge(clk) then
            e_gq <= g_avid;
            e_go <= '0';
            if s_zone = '1' and s_zc = 370 then
                e_y8 <= to_signed(-276480, 20);
                e_bl <= c_b0;
            elsif s_zone = '1' and s_zc = 371 then
                e_go <= '1';
            elsif e_gq = '1' and g_avid = '0' then
                e_y8 <= e_y8 + signed(resize(s_ys & '0', 20));
                e_bl <= e_bl + signed(resize(c_bst, 24));
                e_go <= '1';
            end if;
            if e_go = '1' then
                v_yh := e_y8(19 downto 8);
                e_ma <= resize(unsigned(abs(v_yh)), 22);
                e_mb <= resize(unsigned(abs(v_yh)), 11);
                e_acc <= (others => '0');
                e_cnt <= (others => '0');
                e_run <= '1';
                v_nb := abs(e_bl);
                if s_sal = '1' then l_b <= -v_nb; else l_b <= e_bl; end if;   -- DSG3 Abs: -|B| (the ramp is |A| + l_b)
            elsif e_run = '1' then
                if e_mb(0) = '1' then e_acc <= e_acc + e_ma; end if;
                e_ma <= e_ma(20 downto 0) & '0';
                e_mb <= '0' & e_mb(10 downto 1);
                e_cnt <= e_cnt + 1;
                if e_cnt = 11 then
                    l_q <= resize(e_acc, 23) + resize(c_x0q, 23);
                    e_run <= '0';
                end if;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1): the ramp A = sA X and x^2 + y^2 by second differences
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_q : signed(23 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_a  <= c_a0;
                b_q  <= l_q;
                b_dx <= c_dx0;
            elsif g_avid = '1' then
                b_a  <= b_a + signed(resize(c_ast, 24));
                v_q  := signed(resize(b_q, 24)) + resize(b_dx, 24);
                b_q  <= unsigned(v_q(22 downto 0));
                b_dx <= b_dx + signed(resize(c_ddx, 16));
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_aa  : signed(23 downto 0);
        variable v_r   : signed(24 downto 0);
        variable v_g   : unsigned(22 downto 0);
        variable v_y4  : unsigned(9 downto 0);
        variable v_cs  : signed(11 downto 0);
        variable v_hs  : unsigned(15 downto 0);
        variable v_x   : signed(17 downto 0);
        variable v_lev : unsigned(2 downto 0);
        variable v_yf  : unsigned(9 downto 0);
        variable v_dy  : signed(12 downto 0);
        variable v_m   : unsigned(12 downto 0);
        variable v_c   : signed(13 downto 0);
        variable v_fl  : std_logic;
    begin
        if rising_edge(clk) then
            -- 2: the ramp's x term (|A| under DSG3 Abs) ; the clamped figure gradient ; key compares ; dome
            v_aa := abs(b_a);
            if s_sal = '1' then p2_a <= v_aa; else p2_a <= b_a; end if;
            v_g := C_RGQ - b_q;
            if b_q < C_RGQ then p2_g <= v_g(21 downto 10); else p2_g <= (others => '0'); end if;
            if b_q(22 downto 6) < c_tlo(22 downto 6) then p2_kin <= '1'; else p2_kin <= '0'; end if;
            if b_q(22 downto 6) > c_thi(22 downto 6) then p2_kout <= '1'; else p2_kout <= '0'; end if;
            p2_dm <= to_unsigned(176, 8) - resize(b_q(22 downto 15), 8);
            -- 3: the ramp (DSG3 Abs: |A| - |B| ; Angles: A + B) ; K3 x gradient partial products ; the patch's Y
            --    (K4 + dome + key step) and camera - patch
            p3_r  <= resize(p2_a, 25) + resize(l_b, 25);
            p3_gl <= resize(p2_g * c_kr4(4 downto 0), 17);
            p3_gh <= resize(p2_g * c_kr4(9 downto 5), 17);
            if p2_kin = '1' then v_y4 := c_y4b; elsif p2_kout = '1' then v_y4 := c_y4; else v_y4 := c_y4a; end if;
            p3_yp <= resize(v_y4, 11) + resize(p2_dm, 11);
            v_cs  := signed(resize(s_camy, 12)) - signed(resize(v_y4, 12));
            p3_d  <= v_cs - signed(resize(p2_dm, 12));
            p3_key <= not p2_kout;
            -- 4: |ramp| (Abs) to a 16-bit phase ; K3 x gradient ; crossfade partial products
            if s_sal = '1' then v_r := abs(p3_r); else v_r := p3_r; end if;
            p4_r16 <= unsigned(v_r(19 downto 4));
            p4_rp  <= resize(p3_gl, 22) + shift_left(resize(p3_gh, 22), 5);
            p4_xl  <= resize(p3_d * signed('0' & c_f(2 downto 0)), 15);
            p4_xh  <= resize(p3_d * signed('0' & c_f(5 downto 3)), 15);
            p4_yp <= p3_yp; p4_key <= p3_key;
            -- 5: phase = ramp +- rings ; Y = patch + fade x (camera - patch)
            if z_n3 = '1' then p5_t <= p4_r16 - p4_rp(17 downto 2); else p5_t <= p4_r16 + p4_rp(17 downto 2); end if;
            v_x  := resize(p4_xl, 18) + shift_left(resize(p4_xh, 18), 3);
            p5_y <= signed(resize(p4_yp, 13)) + v_x(17 downto 6);
            p5_key <= p4_key;
            -- 6: the read phases (the keyed 180 deg flip toggles bit 15) ; Ribbons (seven comparators) ; Proc soft clip
            v_fl  := p5_key and not s_l2c;
            v_hs  := p5_t + c_hu;  p6_au <= (v_hs(15) xor v_fl) & v_hs(14 downto 0);
            v_hs  := p5_t + c_hv;  p6_av <= (v_hs(15) xor v_fl) & v_hs(14 downto 0);
            v_hs  := p5_t + c_hoff; p6_ph <= (v_hs(15) xor v_fl) & v_hs(14 downto 11);
            v_lev := (others => '0');
            if p5_y >= 467 then v_lev := v_lev + 1; end if;
            if p5_y >= 514 then v_lev := v_lev + 1; end if;
            if p5_y >= 560 then v_lev := v_lev + 1; end if;
            if p5_y >= 607 then v_lev := v_lev + 1; end if;
            if p5_y >= 653 then v_lev := v_lev + 1; end if;
            if p5_y >= 700 then v_lev := v_lev + 1; end if;
            if p5_y >= 746 then v_lev := v_lev + 1; end if;
            p6_lev <= v_lev;
            if p5_y > 1080 then p6_ys <= to_unsigned(940, 10);
            elsif p5_y > 800 then p6_ys <= unsigned(p5_y(10 downto 1)) + 400;
            elsif p5_y < 0 then p6_ys <= (others => '0');
            else p6_ys <= unsigned(p5_y(9 downto 0)); end if;
            -- 7: sine ROM addresses (or the zone's LFO reads) ; the Y path (Ribbons' DAC or Proc) ; gamut table
            --    address {S7, Y >> 5, hue >> 11}
            if z_lfsel = '1' then
                r_su_a <= z_lfa; r_sv_a <= z_lfb;
            else
                r_su_a <= f_fold(p6_au(15 downto 6)); r_sv_a <= f_fold(p6_av(15 downto 6));
            end if;
            p7_un <= p6_au(15); p7_vn <= p6_av(15) xor s_qout;
            if s_rib = '1' then
                case to_integer(p6_lev) is                    -- Ribbons' DAC (the eight levels of C_SC_DAC)
                    when 0 => v_yf := C_SC_DAC(0); when 1 => v_yf := C_SC_DAC(1);
                    when 2 => v_yf := C_SC_DAC(2); when 3 => v_yf := C_SC_DAC(3);
                    when 4 => v_yf := C_SC_DAC(4); when 5 => v_yf := C_SC_DAC(5);
                    when 6 => v_yf := C_SC_DAC(6); when others => v_yf := C_SC_DAC(7);
                end case;
            else
                v_yf := p6_ys;
            end if;
            p7_yf <= v_yf;
            r_g_a <= s_qout & v_yf(9 downto 5) & p6_ph;
            -- 8: ROM and table (EBR)
            p8_un <= p7_un; p8_vn <= p7_vn; p8_yf <= p7_yf;
            -- 9: fabric copies (the sine field by S7: full / I axis)
            if s_qout = '1' then
                pm_u(9) <= unsigned(r_su_q(7 downto 0)); pm_v(9) <= unsigned(r_sv_q(7 downto 0));
            else
                pm_u(9) <= unsigned(r_su_q(15 downto 8)); pm_v(9) <= unsigned(r_sv_q(15 downto 8));
            end if;
            pn_u(9) <= p8_un; pn_v(9) <= p8_vn;
            p9_l  <= unsigned(r_g_q);
            p9_yf <= p8_yf;
            -- 10: saturation = min(Factors gain, gamut limit)
            if c_sf < p9_l then p10_s <= c_sf; else p10_s <= p9_l; end if;
            p10_yf <= p9_yf;
            -- 11: s x |sin| partial products (nibbles of s) ; Luma To Chroma input
            p11_ulo <= resize(p10_s(3 downto 0) * pm_u(10), 12);
            p11_uhi <= resize(p10_s(7 downto 4) * pm_u(10), 12);
            p11_vlo <= resize(p10_s(3 downto 0) * pm_v(10), 12);
            p11_vhi <= resize(p10_s(7 downto 4) * pm_v(10), 12);
            p11_dy  <= signed(resize(p10_yf, 11)) - 517;
            p11_yf  <= p10_yf;
            -- 12: magnitudes ((hi x 16 + lo) >> 7) ; orbit + the Y -> I+ push (I axis: U + 1.22 dY, V - 1.125 dY)
            v_m := resize(p11_ulo(11 downto 4), 13) + resize(p11_uhi, 13);
            p12_mu <= v_m(11 downto 3);
            v_m := resize(p11_vlo(11 downto 4), 13) + resize(p11_vhi, 13);
            p12_mv <= v_m(11 downto 3);
            v_dy := resize(p11_dy, 13);
            if s_l2c = '1' then
                p12_ou <= resize(c_orbu, 13) + v_dy + shift_right(v_dy, 2) - shift_right(v_dy, 5);
                p12_ov <= resize(c_orbv, 13) - v_dy - shift_right(v_dy, 3);
            else
                p12_ou <= resize(c_orbu, 13); p12_ov <= resize(c_orbv, 13);
            end if;
            p12_yf <= p11_yf;
            -- 13: chroma = +- magnitude + orbit (+ push)
            if pn_u(12) = '1' then p13_cu <= p12_ou - signed(resize(p12_mu, 13)); else p13_cu <= p12_ou + signed(resize(p12_mu, 13)); end if;
            if pn_v(12) = '1' then p13_cv <= p12_ov - signed(resize(p12_mv, 13)); else p13_cv <= p12_ov + signed(resize(p12_mv, 13)); end if;
            p13_yf <= p12_yf;
            -- 14: clamp, centre ; black off the generated avid
            if s_gav_sr(11) = '0' then
                s_y14 <= C_KY; s_u14 <= C_KU; s_v14 <= C_KV;
            else
                s_y14 <= p13_yf;
                v_c := resize(p13_cu, 14) + 512;
                if v_c < 16 then s_u14 <= to_unsigned(16, 10); elsif v_c > 1008 then s_u14 <= to_unsigned(1008, 10);
                else s_u14 <= unsigned(v_c(9 downto 0)); end if;
                v_c := resize(p13_cv, 14) + 512;
                if v_c < 16 then s_v14 <= to_unsigned(16, 10); elsif v_c > 1008 then s_v14 <= to_unsigned(1008, 10);
                else s_v14 <= unsigned(v_c(9 downto 0)); end if;
            end if;
            pm_u(10) <= pm_u(9); pm_v(10) <= pm_v(9);
            for k in 10 to 12 loop
                pn_u(k) <= pn_u(k - 1); pn_v(k) <= pn_v(k - 1);
            end loop;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(11);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output filter stores
    ---------------------------------------------------------------------------
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            s_dp <= s_dp + 1; s_dr <= s_dr + 1;
            s_qa <= da(to_integer(s_dr));
            da(to_integer(s_dp)) <= "00" & std_logic_vector(s_y14) & std_logic_vector(s_u14) & std_logic_vector(s_v14);
            s_qb <= db(to_integer(s_dr));
            db(to_integer(s_dp)) <= "00" & std_logic_vector(s_y16) & std_logic_vector(s_u16) & std_logic_vector(s_v16);
            s_fa <= s_qa; s_fb <= s_qb;                 -- = the box inputs 8 pixels back
        end if;
    end process p_dl;

    p_vs : process(clk)
        variable v_s : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if s_gav_sr(15) = '1' then v_k17 <= v_k17 + 1; else v_k17 <= (others => '0'); end if;   -- column of the stage-17 pixel
            if s_gav_sr(17) = '1' then v_k19 <= v_k19 + 1; else v_k19 <= (others => '0'); end if;   -- column of the stage-19 pixel
            v_qy <= vy(to_integer(v_k17(10 downto 1)));
            v_qc <= vc(to_integer(v_k17(10 downto 2)));
            v_fy <= v_qy; v_fc <= v_qc;
            v_py <= s_y19; v_pu <= s_u19; v_pv <= s_v19;
            v_wye <= s_gav_sr(17) and v_k19(0);
            v_wya <= v_k19(10 downto 1);
            v_s := resize(s_y19, 11) + resize(v_py, 11); v_wyd <= std_logic_vector(v_s(10 downto 3));
            if s_gav_sr(17) = '1' and v_k19(1 downto 0) = "01" then v_wce <= '1'; else v_wce <= '0'; end if;
            v_wca <= v_k19(10 downto 2);
            v_s := resize(s_u19, 11) + resize(v_pu, 11); v_wcd(15 downto 8) <= std_logic_vector(v_s(10 downto 3));
            v_s := resize(s_v19, 11) + resize(v_pv, 11); v_wcd(7 downto 0) <= std_logic_vector(v_s(10 downto 3));
            if v_wye = '1' then vy(to_integer(v_wya)) <= v_wyd; end if;
            if v_wce = '1' then vc(to_integer(v_wca)) <= v_wcd; end if;
        end if;
    end process p_vs;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the rack's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16, a vertical [1 1]/2 with the line above (no
    -- single-row lines), and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_mx : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to the black (the off-avid input of both boxes) once
            -- 8 blank samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 15 sums, stage 16 output
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KU & "000"; s_bav <= C_KV & "000";
            else
                s_bay <= s_bay + s_y14 - unsigned(s_fa(29 downto 20));
                s_bau <= s_bau + s_u14 - unsigned(s_fa(19 downto 10));
                s_bav <= s_bav + s_v14 - unsigned(s_fa(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(13) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y16 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u16 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v16 <= v_sc(13 downto 4);
            else
                s_y16 <= C_KY; s_u16 <= C_KU; s_v16 <= C_KV;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(13);
            -- box B: stage 17 sums, stage 18 output
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= C_KU & "000"; s_bbv <= C_KV & "000";
            else
                s_bby <= s_bby + s_y16 - unsigned(s_fb(29 downto 20));
                s_bbu <= s_bbu + s_u16 - unsigned(s_fb(19 downto 10));
                s_bbv <= s_bbv + s_v16 - unsigned(s_fb(9 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y18 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u18 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v18 <= v_sc(13 downto 4);
            -- 19: luma 13/16
            if s_y18 > 64 then v_dl := s_y18 - 64; else v_dl := (others => '0'); end if;
            s_y19 <= s_y18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u19 <= s_u18; s_v19 <= s_v18;
            -- 20: vertical mix with the line above
            v_mx := resize(s_y19, 11) + resize(unsigned(v_fy) & "10", 11);             s_y20 <= v_mx(10 downto 1);
            v_mx := resize(s_u19, 11) + resize(unsigned(v_fc(15 downto 8)) & "10", 11); s_u20 <= v_mx(10 downto 1);
            v_mx := resize(s_v19, 11) + resize(unsigned(v_fc(7 downto 0)) & "10", 11);  s_v20 <= v_mx(10 downto 1);
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

end architecture subcarrier;
