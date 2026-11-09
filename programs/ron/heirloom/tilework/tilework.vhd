-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Tilework v0.1.0 -- Heirloom (video phase), from the clip WIN_20230318_12_24_18 at 4.0 s.
--
-- The still: a grid of flat, saturated colour blocks.  Seven rows (108.75 lines each) climb a 3-bit staircase
-- blue, orange, pink, lime, yellow, cyan into a long clipped green band; the centre column (from ~745 to ~1460)
-- carries every row inverted (all three bits flipped: cyan, yellow, lime, pink, orange, blue, magenta).  The column
-- edges lean right going down (a keystone) and each one is a rainbow fringe: the hue wheel sweeping the long way
-- round from one colour to the other.  In the clip Ron turns one knob that slows both oscillators: the grid grows
-- out of the top-left corner (2.0 -> 4.4 s), and the same patch at full speed is a fine green/magenta mesh.
--
-- The patch, as fitted (tilework.py has every constant):
--   V      = triangle, v = o + fold((y - y_s) / step), period 16.1 levels; L = clamp(floor(v), 0, 7); reset at
--            y_s = step - 32 lines, above it the wave holds the previous field's last value
--   H      = square wave, phi = (x + 97)(1 - kappa y) / h - 0.184 half-periods, reset per line
--   colour = hue wheel: W = L xor 1 (side columns), 7 - W (centre column); at each column edge the wheel
--            sweeps the long way (5 or 7 steps), the half turn first, a dwell, then the rest
--
--   K1  Rows      row height x1/4 .. x4 (the V oscillator)
--   K2  Columns   column width (the H oscillator): wider (one edge) .. x1/5.7 (narrow columns)
--   K3  Level     the staircase's offset: which colours appear, how long the clipped bands are
--   K4  Sweep     the zoom breathing (Ron's hand on the rate knob): depth and rate; 0 = still
--   K5  Skew      the keystone lean of the column edges: fanned left .. the still (512) .. fanned right
--   K6  Bend      how far the camera (or the Osc) bends the rows
--   S7  Plaid     the columns become 4-step staircases too (a tartan)
--   S8  Stairs    each row shifts the column edges by its level: stepped chevrons
--   S9  Prism     the rainbow fringes 8x wider
--   S10 Colours   the hue wheel turned 135 degrees
--   S11 Source    Video / Osc
--   P12 ZOOM      both oscillators (Ron's knob): giant tiles (0) .. the still (246) .. the fine mesh (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, exp2 ROM), a per-line engine kicked at each
-- line start (computes the NEXT line: the V staircase level, the H start phase and step with the keystone), a
-- phase accumulator per pixel, a 20-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).
-- Generated avid; output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16) before
-- the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.tilework_pkg.all;
use work.all;

architecture tilework of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant C_A   : integer := 8441037;     -- triangle amplitude, 8.05 levels (Q20)
    constant C_A2  : integer := 16882074;    -- period, 16.1 levels
    constant C_ONE : integer := 1048576;     -- 1 level (Q20)
    constant C_ST  : integer := 2013266;     -- Stairs: 0.12 half-period per level (Q24)
    constant C_P0  : signed(39 downto 0) := to_signed(-790273982, 40);   -- H phase at the reset, -0.184 (Q32)

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
    -- the V zoom ceiling (Q8 octaves): a row never shorter than 2 real lines
    function f_evmax(t : std_logic_vector(3 downto 0)) return signed is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_signed(924, 12);
            when 4          => return to_signed(1176, 12);
            when 8          => return to_signed(987, 12);
            when 12         => return to_signed(1243, 12);
            when 5 | 6 | 14 => return to_signed(1326, 12);
            when 1 | 2 | 10 => return to_signed(1219, 12);
            when others     => return to_signed(1475, 12);
        end case;
    end function;
    -- 2^23 * (virtual lines per line) / 1080, for the keystone step
    function f_kys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(34528, 16);
            when 4          => return to_unsigned(17476, 16);
            when 8          => return to_unsigned(29127, 16);
            when 12         => return to_unsigned(14564, 16);
            when 5 | 6 | 14 => return to_unsigned(11651, 16);
            when 1 | 2 | 10 => return to_unsigned(15534, 16);
            when others     => return to_unsigned(7767, 16);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- bipolar knob with a +-8 deadband around 512: -503 .. 503
    function f_db(k : unsigned(9 downto 0)) return signed is
        variable d : signed(10 downto 0);
    begin
        d := signed(resize(k, 11)) - 512;
        if d > -9 and d < 9 then
            return to_signed(0, 11);
        elsif d > 0 then
            return d - 8;
        else
            return d + 8;
        end if;
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
    signal r_ex_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q  : unsigned(15 downto 0) := (others => '0');
    signal r_pal_a : unsigned(7 downto 0) := (others => '0');
    signal r_pal_q : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_xa   : unsigned(9 downto 0) := to_unsigned(218, 10);
    signal s_evmax : signed(11 downto 0) := to_signed(1475, 12);
    signal s_kys  : unsigned(15 downto 0) := to_unsigned(7767, 16);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(246 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_plaid, s_stairs, s_prism, s_col, s_src : std_logic := '0';

    -- per-field values
    signal d_k1, d_k2, d_k3, d_p : signed(10 downto 0) := (others => '0');
    signal c_kap  : signed(16 downto 0) := to_signed(1573, 17);       -- keystone, Q16 (0.024 = the still)
    signal c_lfo  : unsigned(15 downto 0) := (others => '0');          -- sweep LFO phase
    signal c_tri  : signed(15 downto 0) := (others => '0');            -- its triangle, Q14
    signal c_atri : unsigned(14 downto 0) := (others => '0');
    signal c_depth : unsigned(9 downto 0) := (others => '0');          -- sweep depth, Q8 octaves
    signal c_esw, c_ep, c_e55, c_ek2 : signed(12 downto 0) := (others => '0');
    signal c_ez, c_evr, c_ehr : signed(12 downto 0) := (others => '0');
    signal c_ev, c_eh, c_ehn : signed(11 downto 0) := (others => '0');
    signal z_mv, z_mh, z_mhn : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_shv, z_shh, z_shn : signed(3 downto 0) := (others => '0');
    signal z_st   : unsigned(18 downto 0) := (others => '0');
    signal c_lst  : unsigned(9 downto 0) := to_unsigned(33, 10);
    signal c_kv   : unsigned(19 downto 0) := to_unsigned(9642, 20);   -- levels per virtual line, Q20
    signal c_dr   : unsigned(21 downto 0) := to_unsigned(9642, 22);   -- levels per line, Q20
    signal c_r0   : signed(27 downto 0) := (others => '0');            -- V ramp at line 0
    signal c_rt   : signed(27 downto 0) := (others => '0');
    signal c_rf   : std_logic := '0';
    signal c_inch : unsigned(26 downto 0) := to_unsigned(3019946, 27); -- half-periods per h, Q32
    signal c_fg    : unsigned(11 downto 0) := to_unsigned(1422, 12);   -- half-period / fringe width, Q6
    signal c_t1   : unsigned(19 downto 0) := (others => '0');
    signal c_dinc : signed(19 downto 0) := (others => '0');            -- keystone step per line, Q32
    signal c_dphs : signed(28 downto 0) := (others => '0');            -- start-phase step per line, Q32
    signal c_phi0 : signed(39 downto 0) := (others => '0');            -- start phase of line 0, Q32
    signal c_o    : signed(24 downto 0) := to_signed(C_ONE, 25);      -- staircase offset, Q20
    signal c_z6   : unsigned(9 downto 0) := (others => '0');
    signal c_bg    : unsigned(9 downto 0) := (others => '0');           -- bend gain, Q6 per luma step
    signal c_ctrg : unsigned(16 downto 0) := (others => '0');          -- Osc centring, 111 * g

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
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
    signal e_lc   : unsigned(3 downto 0) := to_unsigned(15, 4);
    signal e_zk   : std_logic := '0';
    signal e_r, e_ra : signed(27 downto 0) := (others => '0');
    signal e_top  : std_logic := '0';
    signal e_tri  : unsigned(23 downto 0) := (others => '0');
    signal e_v, e_vlast : signed(25 downto 0) := (others => '0');
    signal e_L    : unsigned(2 downto 0) := (others => '0');
    signal e_incl : signed(28 downto 0) := (others => '0');
    signal e_phs  : signed(39 downto 0) := (others => '0');
    signal e_st   : signed(24 downto 0) := (others => '0');
    signal e_ipx  : signed(31 downto 0) := (others => '0');
    signal n_vq8, e_vq : signed(13 downto 0) := (others => '0');
    signal n_phis : signed(31 downto 0) := (others => '0');
    signal n_inc  : signed(31 downto 0) := (others => '0');

    -- beam
    signal b_phi, b_inc : signed(31 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_vq8 : signed(13 downto 0) := (others => '0');

    -- pixel path
    signal p2_sq, p2_act0, p2_bc : std_logic := '0';
    signal p2_d10 : signed(9 downto 0) := (others => '0');
    signal p2_qf  : unsigned(1 downto 0) := (others => '0');
    signal p3_sq, p3_act0, p3_bc : std_logic := '0';
    signal p3_qf  : unsigned(1 downto 0) := (others => '0');
    signal p3_ta, p3_tb : signed(16 downto 0) := (others => '0');
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p4_sq, p4_act0, p4_bc : std_logic := '0';
    signal p4_qf  : unsigned(1 downto 0) := (others => '0');
    signal p4_tp  : signed(22 downto 0) := (others => '0');
    signal p4_pa, p4_pb : unsigned(13 downto 0) := (others => '0');
    signal p5_sq, p5_act, p5_bc : std_logic := '0';
    signal p5_qf  : unsigned(1 downto 0) := (others => '0');
    signal p5_tq  : unsigned(4 downto 0) := (others => '0');
    signal p5_bs  : unsigned(17 downto 0) := (others => '0');
    signal p6_sq, p6_act, p6_bc : std_logic := '0';
    signal p6_qf  : unsigned(1 downto 0) := (others => '0');
    signal p6_tq  : unsigned(4 downto 0) := (others => '0');
    signal p6_vp  : signed(13 downto 0) := (others => '0');
    signal p7_sq, p7_act, p7_bc : std_logic := '0';
    signal p7_qf  : unsigned(1 downto 0) := (others => '0');
    signal p7_tq  : unsigned(4 downto 0) := (others => '0');
    signal p7_L   : unsigned(2 downto 0) := (others => '0');
    signal p8_act : std_logic := '0';
    signal p8_tq  : unsigned(4 downto 0) := (others => '0');
    signal p8_wb, p8_wn : unsigned(2 downto 0) := (others => '0');
    signal p9_act, p9_neg : std_logic := '0';
    signal p9_wb, p9_wn : unsigned(2 downto 0) := (others => '0');
    signal p9_off : unsigned(7 downto 0) := (others => '0');
    signal p10_w, p11_w : unsigned(7 downto 0) := (others => '0');
    signal p14_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y15 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cby : t_u10(0 to 7) := (others => to_unsigned(64, 10));
    signal s_cau, s_cav, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bay, s_bayd, s_bby, s_bbyd : unsigned(12 downto 0) := to_unsigned(512, 13);
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bbu, s_bbv, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 inputs of boxes A, B
    signal s_y17, s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17, s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
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
            r_ex_q  <= C_TW_EXP2(to_integer(r_ex_a));
            r_pal_q <= C_TW_PAL(to_integer(r_pal_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_t  : signed(16 downto 0);
        variable v_a  : signed(15 downto 0);
        variable v_u  : unsigned(23 downto 0);
        variable v_sf : integer range 0 to 31;
        variable v_g  : unsigned(11 downto 0);
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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_plaid  <= registers_in(6)(0);
                s_stairs <= registers_in(6)(1);
                s_prism  <= registers_in(6)(2);
                s_col    <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_xs     <= s_xsr;
                s_evmax  <= f_evmax(s_tid);
                s_kys    <= f_kys(s_tid);
                case to_integer(s_xsr) is
                    -- H reset 97 virtual px left of the picture (+ the SD crop), + 12 pixels for the output boxes'
                    -- delay, so the column edges land where the still has them
                    when 4      => s_xa <= to_unsigned(722, 10); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_xa <= to_unsigned(230, 10); s_oxs <= to_unsigned(153, 9);
                    when others => s_xa <= to_unsigned(218, 10); s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

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

            -- serial products: slot k loads at zc = 40 + 24 k, 16 iterations, result stable from
            -- load + 17 (stores at load + 18).  The slot number is decoded at 38 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 39 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 38  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 62  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 86  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 110 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 134 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 158 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 182 => z_sel <= to_unsigned(6, 5);  z_pre <= '1';
                    when 206 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 230 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 254 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 278 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 302 => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when 326 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 350 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 374 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 398 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 422 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_g4(12 downto 3), 36);  z_fb <= resize(s_g4(12 downto 3), 16);  -- k4^2
                    when 1  => z_fa <= resize(c_atri, 36);              z_fb <= resize(c_depth, 16);           -- sweep
                    when 2  => z_fa <= resize(unsigned(abs(d_p)), 36);  z_fb <= to_unsigned(1382, 16);         -- zoom
                    when 3  => z_fa <= resize(unsigned(abs(c_ez)), 36); z_fb <= to_unsigned(141, 16);          -- x 0.55
                    when 4  =>                                                                                 -- columns
                        z_fa <= resize(unsigned(abs(d_k2)), 36);
                        if d_k2(10) = '0' then z_fb <= to_unsigned(325, 16); else z_fb <= to_unsigned(104, 16); end if;
                    when 5  => z_fa <= to_unsigned(9642, 36);           z_fb <= z_mv;                          -- kv
                    when 6  => z_fa <= resize(c_kv, 36);                z_fb <= resize(s_ys, 16);              -- dr
                    when 7  => z_fa <= to_unsigned(188747, 36);         z_fb <= z_mh;                          -- inc
                    when 8  => z_fa <= to_unsigned(1422, 36);           z_fb <= z_mhn;                         -- G
                    when 9  => z_fa <= resize(c_inch(26 downto 8), 36); z_fb <= resize(unsigned(abs(c_kap)), 16);
                    when 10 => z_fa <= resize(c_t1, 36);                z_fb <= s_kys;
                    when 11 => z_fa <= resize(unsigned(abs(c_dinc)), 36); z_fb <= resize(s_xa, 16);
                    when 12 => z_fa <= resize(c_inch, 36);              z_fb <= resize(s_xa, 16);
                    when 13 => z_fa <= resize(unsigned(abs(d_k3)), 36); z_fb <= to_unsigned(7282, 16);         -- level
                    when 14 => z_fa <= resize(s_g6(12 downto 3), 36);  z_fb <= resize(s_g6(12 downto 3), 16);  -- k6^2
                    when 15 => z_fa <= resize(c_z6, 36);                z_fb <= to_unsigned(276, 16);
                    when others => z_fa <= resize(c_bg, 36);             z_fb <= to_unsigned(111, 16);          -- Osc centre
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- knob decodes (the glide is done by zc 30), the sweep LFO's triangle
                    when 32 =>
                        d_k1 <= f_db(s_g1(12 downto 3));
                        d_k2 <= f_db(s_g2(12 downto 3));
                        d_k3 <= f_db(s_g3(12 downto 3));
                        d_p  <= signed(resize(s_gp(12 downto 3), 11)) - 246;
                        if c_lfo < x"4000" then
                            c_tri <= signed(c_lfo);
                        elsif c_lfo < x"C000" then
                            c_tri <= signed(x"8000" - c_lfo);
                        else
                            c_tri <= signed(c_lfo);       -- c_lfo - 0x10000, two's complement
                        end if;
                    when 33 =>
                        v_a := abs(c_tri);
                        c_atri <= unsigned(v_a(14 downto 0));
                        v_t := resize(f_db(s_g5(12 downto 3)), 17);
                        c_kap <= shift_left(v_t, 6) + v_t + 1573;   -- 0.024 + kd * 65 / 65536

                    -- sweep: depth = 3 k4^2 >> 12 (Q8 octaves), zoom OUT first
                    when 58 =>
                        v_u := resize(f_acc(21 downto 0), 24) + shift_left(resize(f_acc(21 downto 0), 24), 1);
                        c_depth <= v_u(21 downto 12);
                    when 82 =>
                        if c_tri(15) = '0' then c_esw <= -signed(resize(f_acc(23 downto 14), 13));
                        else c_esw <= signed(resize(f_acc(23 downto 14), 13)); end if;
                    -- zoom: e_p = (P12 - 246) * 1382 / 1024; e_v = e_p + sweep + K1; e_h = 0.55 (e_p + sweep) + K2
                    when 106 =>
                        if d_p(10) = '0' then c_ep <= signed(resize(f_acc(20 downto 10), 13));
                        else c_ep <= -signed(resize(f_acc(20 downto 10), 13)); end if;
                    when 107 => c_ez <= c_ep + c_esw;
                    when 108 => c_evr <= c_ez + resize(d_k1, 13);
                    when 109 =>
                        if c_evr < -768 then c_ev <= to_signed(-768, 12);
                        elsif c_evr > resize(s_evmax, 13) then c_ev <= s_evmax;
                        else c_ev <= c_evr(11 downto 0); end if;
                    when 110 => r_ex_a <= unsigned(c_ev(7 downto 0)); z_shv <= c_ev(11 downto 8);
                    when 112 => z_mv <= r_ex_q;
                    when 130 =>
                        if c_ez(12) = '0' then c_e55 <= signed(resize(f_acc(20 downto 8), 13));
                        else c_e55 <= -signed(resize(f_acc(20 downto 8), 13)); end if;
                    when 154 =>
                        if d_k2(10) = '0' then c_ek2 <= signed(resize(f_acc(17 downto 8), 13));
                        else c_ek2 <= -signed(resize(f_acc(17 downto 8), 13)); end if;
                    when 155 => c_ehr <= c_e55 + c_ek2;
                    when 156 =>
                        if c_ehr < -384 then c_eh <= to_signed(-384, 12);
                        elsif c_ehr > 1280 then c_eh <= to_signed(1280, 12);
                        else c_eh <= c_ehr(11 downto 0); end if;
                    when 157 => r_ex_a <= unsigned(c_eh(7 downto 0)); z_shh <= c_eh(11 downto 8); c_ehn <= -c_eh;
                    when 158 => r_ex_a <= unsigned(c_ehn(7 downto 0)); z_shn <= c_ehn(11 downto 8);
                    when 159 => z_mh <= r_ex_q;
                    when 160 => z_mhn <= r_ex_q;
                    -- kv = 9642 * 2^e_v (Q20 levels per virtual line), the V ramp at line 0
                    when 178 =>
                        case to_integer(z_shv) is
                            when -3     => c_kv <= resize(f_acc(35 downto 18), 20);
                            when -2     => c_kv <= resize(f_acc(35 downto 17), 20);
                            when -1     => c_kv <= f_acc(35 downto 16);
                            when 0      => c_kv <= f_acc(34 downto 15);
                            when 1      => c_kv <= f_acc(33 downto 14);
                            when 2      => c_kv <= f_acc(32 downto 13);
                            when 3      => c_kv <= f_acc(31 downto 12);
                            when 4      => c_kv <= f_acc(30 downto 11);
                            when others => c_kv <= f_acc(29 downto 10);
                        end case;
                    when 179 => c_r0 <= signed(resize(shift_left(resize(c_kv, 26), 5), 28)) - C_ONE;
                    when 202 => c_dr <= f_acc(29 downto 8);
                    -- the reset less than one line below the top: no 1-line top band (r0 > -dr -> 0), in 3 steps
                    when 203 => c_rt <= c_r0 + signed(resize(c_dr, 28));
                    when 204 =>
                        if c_r0(27) = '1' and c_rt(27) = '0' and c_rt /= 0 then c_rf <= '1'; else c_rf <= '0'; end if;
                    when 205 =>
                        if c_rf = '1' then c_r0 <= (others => '0'); end if;
                    -- H: half-periods per h (Q32), the fringe gain
                    when 226 =>
                        case to_integer(z_shh) is
                            when -2     => c_inch <= resize(f_acc(35 downto 13), 27);
                            when -1     => c_inch <= resize(f_acc(35 downto 12), 27);
                            when 0      => c_inch <= resize(f_acc(35 downto 11), 27);
                            when 1      => c_inch <= resize(f_acc(35 downto 10), 27);
                            when 2      => c_inch <= f_acc(35 downto 9);
                            when 3      => c_inch <= f_acc(34 downto 8);
                            when 4      => c_inch <= f_acc(33 downto 7);
                            when others => c_inch <= f_acc(32 downto 6);
                        end case;
                    when 250 =>
                        v_sf := 15 - to_integer(z_shn);
                        if s_prism = '1' then v_sf := v_sf + 3; end if;
                        case v_sf is
                            when 14     => c_fg <= f_acc(25 downto 14);
                            when 15     => c_fg <= f_acc(26 downto 15);
                            when 16     => c_fg <= f_acc(27 downto 16);
                            when 17     => c_fg <= f_acc(28 downto 17);
                            when 18     => c_fg <= f_acc(29 downto 18);
                            when 19     => c_fg <= f_acc(30 downto 19);
                            when 20     => c_fg <= f_acc(31 downto 20);
                            when 21     => c_fg <= f_acc(32 downto 21);
                            when 22     => c_fg <= f_acc(33 downto 22);
                            when others => c_fg <= f_acc(34 downto 23);
                        end case;
                    -- keystone: dinc = inc * kappa * (lines per line) / 1080, dphs = Xa * dinc, phi0 = P0 + Xa * inc
                    when 274 => c_t1 <= f_acc(33 downto 14);
                    when 298 =>
                        if c_kap(16) = '0' then c_dinc <= signed(resize(f_acc(35 downto 17), 20));
                        else c_dinc <= -signed(resize(f_acc(35 downto 17), 20)); end if;
                    when 322 =>
                        if c_dinc(19) = '0' then c_dphs <= signed(resize(f_acc(27 downto 0), 29));
                        else c_dphs <= -signed(resize(f_acc(27 downto 0), 29)); end if;
                    when 346 => c_phi0 <= C_P0 + signed(resize(f_acc, 40));
                    -- staircase offset o = 1 + 3.5 kd / 504 levels
                    when 370 =>
                        if d_k3(10) = '0' then c_o <= to_signed(C_ONE, 25) + signed(resize(f_acc(21 downto 0), 25));
                        else c_o <= to_signed(C_ONE, 25) - signed(resize(f_acc(21 downto 0), 25)); end if;
                    -- bend gain: (k6^2 >> 10) * 276 >> 10, x3 for the Osc
                    when 394 => c_z6 <= f_acc(19 downto 10);
                    when 418 =>
                        v_g := resize(f_acc(19 downto 10), 12);
                        if s_src = '1' then v_g := v_g + shift_left(v_g, 1); end if;
                        c_bg <= v_g(9 downto 0);
                    when 442 =>
                        if s_src = '1' then c_ctrg <= f_acc(16 downto 0); else c_ctrg <= (others => '0'); end if;
                    -- next field's LFO phase: step = 33 + k4 * 273 / 1024
                    when 445 => z_st <= shift_left(resize(s_g4(12 downto 3), 19), 8) + shift_left(resize(s_g4(12 downto 3), 19), 4);
                    when 446 => z_st <= z_st + resize(s_g4(12 downto 3), 19);
                    when 447 => c_lst <= resize(z_st(18 downto 10), 10) + 33;
                    when 448 => c_lfo <= c_lfo + c_lst;
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
            if s_src = '1' then s_lb_wd <= std_logic_vector(s_osum);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line in 8 clocks: the V ramp r (wrapped to one
    -- period), the triangle and the staircase value v (held above the reset),
    -- the keystone step and the start phase (+ Stairs)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d : signed(27 downto 0);
        variable v_iu : unsigned(31 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 460 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_r    <= c_r0 - signed(resize(c_dr, 28));
                e_incl <= signed(resize(c_inch, 29)) + resize(c_dinc, 29);
                e_phs  <= c_phi0 + resize(c_dphs, 40);
                e_lc   <= (others => '0');
            elsif g_start = '1' then
                e_lc <= (others => '0');
            elsif e_lc /= 15 then
                e_lc <= e_lc + 1;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    e_ra <= e_r + signed(resize(c_dr, 28));
                when 2 =>
                    if e_ra >= C_A2 then e_r <= e_ra - C_A2; else e_r <= e_ra; end if;
                when 3 =>
                    e_top <= e_r(27);
                    v_d := C_A2 - e_r;      -- 0 .. 8.05 levels: SLICE, a narrowing resize would fold it
                    if e_r < C_A then e_tri <= unsigned(e_r(23 downto 0));
                    else e_tri <= unsigned(v_d(23 downto 0)); end if;
                when 4 =>
                    if e_top = '1' then e_v <= e_vlast;
                    else e_v <= resize(c_o, 26) + signed(resize(e_tri, 26)); end if;
                when 5 =>
                    e_vlast <= e_v;
                    e_vq <= e_v(25 downto 12);
                    if e_v < 0 then e_L <= "000";
                    elsif e_v >= 7 * C_ONE then e_L <= "111";
                    else e_L <= unsigned(e_v(22 downto 20)); end if;
                when 6 =>
                    e_incl <= e_incl - resize(c_dinc, 29);
                    e_phs  <= e_phs - resize(c_dphs, 40);
                    n_vq8  <= e_vq - signed(resize(c_ctrg(16 downto 6), 14));     -- the Osc's centring, per line
                when 7 =>
                    if s_stairs = '1' then
                        case to_integer(e_L) is
                            when 0      => e_st <= to_signed(-4 * C_ST, 25);
                            when 1      => e_st <= to_signed(-3 * C_ST, 25);
                            when 2      => e_st <= to_signed(-2 * C_ST, 25);
                            when 3      => e_st <= to_signed(-C_ST, 25);
                            when 4      => e_st <= to_signed(0, 25);
                            when 5      => e_st <= to_signed(C_ST, 25);
                            when 6      => e_st <= to_signed(2 * C_ST, 25);
                            when others => e_st <= to_signed(3 * C_ST, 25);
                        end case;
                    else
                        e_st <= (others => '0');
                    end if;
                    -- the step is always positive (keystone <= 0.524): zero-extend, so no LUT sees the sign bit twice
                    -- (a duplicated input hangs router1)
                    v_iu := resize(unsigned(e_incl(27 downto 0)), 32);
                    case to_integer(s_xs) is
                        when 4      => e_ipx <= signed(shift_left(v_iu, 2));
                        when 3      => e_ipx <= signed(shift_left(v_iu, 1) + v_iu);
                        when others => e_ipx <= signed(shift_left(v_iu, 1));
                    end case;
                when 8 =>
                    n_phis <= e_phs(39 downto 8) + resize(e_st, 32);
                    n_inc  <= resize(e_ipx(31 downto 8), 32);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the H phase of each pixel (Q24 half-periods), from the line's start
    -- phase and step
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_phi <= n_phis;
                b_inc <= n_inc;
                l_vq8 <= n_vq8;
                b_px  <= (others => '0');
            elsif g_avid = '1' then
                b_phi <= b_phi + b_inc;
                b_px  <= b_px + 1;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = the beam phase of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ip : signed(7 downto 0);
        variable v_nr : signed(8 downto 0);
        variable v_tr : signed(11 downto 0);
        variable v_ws, v_wc : unsigned(2 downto 0);
        variable v_w  : unsigned(7 downto 0);
        variable v_fa : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: integer / fraction of the phase: started (past the first edge), square, nearest edge,
            --    signed distance to it (Q10 half-periods), plaid quarter (mirrored on odd half-periods)
            v_ip := b_phi(31 downto 24);
            if b_phi(23) = '1' then v_nr := resize(v_ip, 9) + 1; else v_nr := resize(v_ip, 9); end if;
            if v_ip > 0 then p2_sq <= b_phi(24); else p2_sq <= '0'; end if;
            if v_nr > 0 then p2_act0 <= '1'; else p2_act0 <= '0'; end if;
            p2_bc  <= not v_nr(0);
            p2_d10 <= b_phi(23 downto 14);
            if b_phi(24) = '1' then p2_qf <= not unsigned(b_phi(23 downto 22));
            else p2_qf <= unsigned(b_phi(23 downto 22)); end if;
            -- 3: fringe position partials d * G (G split 6 / 6 bits); luma from the line buffer
            p3_ta <= p2_d10 * signed('0' & c_fg(5 downto 0));
            p3_tb <= p2_d10 * signed('0' & c_fg(11 downto 6));
            p3_lm <= unsigned(s_lb_q);
            p3_sq <= p2_sq; p3_act0 <= p2_act0; p3_bc <= p2_bc; p3_qf <= p2_qf;
            -- 4: d * G; bend partials (luma nibbles x gain)
            p4_tp <= resize(p3_ta, 23) + shift_left(resize(p3_tb, 23), 6);
            p4_pa <= resize(p3_lm(3 downto 0) * c_bg, 14);
            p4_pb <= resize(p3_lm(7 downto 4) * c_bg, 14);
            p4_sq <= p3_sq; p4_act0 <= p3_act0; p4_bc <= p3_bc; p4_qf <= p3_qf;
            -- 5: t = 16 + d G / 2048 across the edge (0 .. 31 inside it); bend sum
            v_tr := p4_tp(22 downto 11);
            if p4_act0 = '1' and v_tr > -17 and v_tr < 16 then p5_act <= '1'; else p5_act <= '0'; end if;
            if v_tr < -16 then p5_tq <= "00000";
            elsif v_tr > 15 then p5_tq <= "11111";
            else p5_tq <= unsigned(v_tr(4 downto 0)) + 16; end if;
            p5_bs <= resize(p4_pa, 18) + shift_left(resize(p4_pb, 18), 4);
            p5_sq <= p4_sq; p5_bc <= p4_bc; p5_qf <= p4_qf;
            -- 6: v = the line's staircase value (less the Osc centring) + bend (Q8 levels)
            p6_vp <= l_vq8 + signed(resize(p5_bs(17 downto 6), 14));
            p6_sq <= p5_sq; p6_act <= p5_act; p6_bc <= p5_bc; p6_qf <= p5_qf; p6_tq <= p5_tq;
            -- 7: the quantizer, L = clamp(floor(v), 0, 7)
            if p6_vp < 0 then p7_L <= "000";
            elsif p6_vp >= 1792 then p7_L <= "111";
            else p7_L <= unsigned(p6_vp(10 downto 8)); end if;
            p7_sq <= p6_sq; p7_act <= p6_act; p7_bc <= p6_bc; p7_qf <= p6_qf; p7_tq <= p6_tq;
            -- 8: wheel positions: side W = L xor 1 (xor the plaid quarter), centre = 7 - W; the colour
            --    before the nearest edge and the current one
            v_ws := p7_L xor "001";
            if s_plaid = '1' then v_ws := v_ws xor ('0' & p7_qf); end if;
            v_wc := not v_ws;
            if p7_bc = '1' then p8_wb <= v_wc; else p8_wb <= v_ws; end if;
            if p7_sq = '1' then p8_wn <= v_wc; else p8_wn <= v_ws; end if;
            p8_act <= p7_act; p8_tq <= p7_tq;
            -- 9: the long way round: Wb = 0 / 1 -> +7 / +5 steps, 2 / 3 -> -5 / -7 (mod 4); fringe offset ROM
            v_fa := (not (p8_wb(1) xor p8_wb(0))) & p8_tq;
            p9_off <= C_TW_FR(to_integer(v_fa));
            p9_neg <= p8_wb(1);
            p9_wb <= p8_wb; p9_wn <= p8_wn; p9_act <= p8_act;
            -- 10: wheel index (32 per step, mod 8 steps)
            if p9_act = '1' then
                if p9_neg = '1' then p10_w <= (p9_wb & "00000") - p9_off;
                else p10_w <= (p9_wb & "00000") + p9_off; end if;
            else
                p10_w <= p9_wn & "00000";
            end if;
            -- 11: Colours turns the wheel 3 steps
            if s_col = '1' then v_w := p10_w + 96; else v_w := p10_w; end if;
            p11_w <= v_w;
            -- 12: palette address; 13: palette ROM (EBR); 14: fabric copy
            r_pal_a <= p11_w;
            p14_q <= r_pal_q;
            -- 15: compose; black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            else
                s_y15 <= unsigned(p14_q(31 downto 22)); s_u15 <= unsigned(p14_q(21 downto 12)); s_v15 <= unsigned(p14_q(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the still's soft column edges, and the HD
    -- HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to black / neutral (the off-avid input of both boxes) once
            -- 8 blank samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 16 sums, stage 17 output
            s_cay(0) <= s_y15; s_cau(0) <= s_u15; s_cav(0) <= s_v15;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= to_unsigned(64 * 8, 13); s_bau <= to_unsigned(512 * 8, 13); s_bav <= to_unsigned(512 * 8, 13);
            else
                s_bay <= s_bay + s_y15 - s_cay(7);
                s_bau <= s_bau + s_u15 - s_cau(7);
                s_bav <= s_bav + s_v15 - s_cav(7);
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black / neutral off the generated avid (what box A gives there anyway)
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            -- box B: stage 18 sums, stage 19 output
            s_cby(0) <= s_y17; s_cbu(0) <= s_u17; s_cbv(0) <= s_v17;
            for i in 1 to 7 loop s_cby(i) <= s_cby(i - 1); s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(64 * 8, 13); s_bbu <= to_unsigned(512 * 8, 13); s_bbv <= to_unsigned(512 * 8, 13);
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

end architecture tilework;
