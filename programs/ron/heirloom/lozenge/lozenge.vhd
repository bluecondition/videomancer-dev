-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Lozenge v0.5.0 -- Heirloom #24, rebuilt from the photo WIN_20230410_01_10_37.
--
-- The photo: nested diamonds (centre 993, 518.6; 1.8 px of width per line of height), thick black
-- bands with a navy / grey ring inside, hot pink and olive bands between them, cut by horizontal
-- stripes that swap pink <-> olive (and navy <-> grey) and squeeze from 71 to 27 lines going down,
-- and thin "road marking" lines on every black band's leading edge (yellow, light pink, or a double
-- white line after the grey ring, which the stripes turn into dashes).
--
-- The patch, as fitted (lozenge.py has every constant):
--   t      = diamond field (H triangle + V triangle), held at its x = 66 value at the line start
--   u      = frac((t + Etch * luma - flow) / period)      a sawtooth colouriser
--   band   = black / ring / A / B / A / black over each period, a fixed eye at the centre (keyed
--            on the un-etched field, so the picture bends the rings but not the pupil)
--   stripe = a vertical square wave, frequency gliding 1/142.6 -> 1/53.5 cycles per line; swaps the pairs
--   lines  = one-shot profiles (ROM) at each black band's leading edge, in scan order
--
--   K1  Stripes   stripe density (x0.25 .. x4)
--   K2  Chirp     how much the stripes squeeze going down (bipolar: up / even / down; the photo = 772)
--   K3  Aspect    tall .. flat diamonds
--   K4  Speed     bipolar: rings stream out of (or into) the centre, the stripes roll
--   K5  Black     width of the black bands
--   K6  Etch      how far the picture (or the Osc) bends the diamonds
--   S7  Stripes   On (the photo) / Off (plain nested diamonds)
--   S8  Marks     Road (the photo's edge lines) / Glow (a long luminous decay behind each edge)
--   S9  Twin      one lozenge / two side by side
--   S10 Colours   Pink Olive (the photo) / Cyan Rust
--   S11 Source    Video / Osc
--   P12 RINGS     ring density: one giant lozenge .. a tunnel of fine rings (379 = the photo)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at
-- each line start (computes the NEXT line: the vertical term times the ring gain, the stripe phase),
-- a beam DDA that carries t * gain exactly (reflecting at the folds), a 21-stage pixel path.  Virtual
-- 1080-line coordinates (SD shows the centre 4:3 of the picture).  Generated avid; output
-- conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16) before the
-- blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.lozenge_pkg.all;
use work.all;

architecture lozenge of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- fitted constants (h = half virtual px; phase: 2^24 per colouriser period)
    constant C_XC2  : integer := 1986;      -- diamond centre x (h)
    constant C_XH2  : integer := 132;       -- line-start hold point (h)
    constant C_CY8  : integer := 132762;    -- diamond centre y, Q8 virtual lines
    constant C_EYE  : integer := 1319880;   -- eye: t * gain < 0.0787 period
    constant C_PH0  : integer := 10026062;  -- stripe phase at line 0 (frac -0.4024), Q24
    constant C_U1   : integer := 17111;     -- black band 1 centre (Q16 phase)
    constant C_U2   : integer := 64448;     -- black band 2 centre
    constant C_UB   : integer := 40750;     -- wide band B centre
    constant C_HB   : integer := 12105;     -- wide band B half-width

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
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a, r_el_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_el_q : unsigned(15 downto 0) := (others => '0');
    signal r_sg_a : unsigned(7 downto 0) := (others => '0');
    signal r_sg_q : signed(11 downto 0) := (others => '0');
    signal r_pf_a : unsigned(9 downto 0) := (others => '0');
    signal r_pf_q : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_zc   : unsigned(9 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(772 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(320 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(379 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(680, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_flipoff, s_glow, s_twin, s_pal, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(11 downto 0) := (others => '0');     -- exponent scratch (Q8 octaves)
    signal z_neg  : std_logic := '0';
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_a    : unsigned(19 downto 0) := (others => '0');
    signal c_g    : unsigned(15 downto 0) := to_unsigned(9166, 16);   -- ring gain per h (period 2^24)
    signal c_gl   : unsigned(15 downto 0) := to_unsigned(9259, 16);   -- left-side gain (x 1.01025)
    signal c_ga, c_gd : unsigned(16 downto 0) := to_unsigned(9166, 17); -- ascending / descending gain (twin: 2G)
    signal c_kat  : unsigned(15 downto 0) := to_unsigned(14746, 16);  -- 2 * 1.800 * aspect, Q12
    signal c_kab  : unsigned(15 downto 0) := to_unsigned(14664, 16);  -- 2 * 1.790 * aspect, Q12
    signal c_chg  : unsigned(9 downto 0) := to_unsigned(362, 10);     -- |chirp| * 1.415, Q8 octaves
    signal c_chn  : std_logic := '0';
    signal c_fsb  : unsigned(21 downto 0) := to_unsigned(192026, 22);
    signal c_fs   : unsigned(23 downto 0) := to_unsigned(192026, 24); -- stripe base frequency per line, Q24
    signal c_vsq  : unsigned(17 downto 0) := (others => '0');
    signal c_dfl  : unsigned(18 downto 0) := (others => '0');
    signal c_drl  : unsigned(17 downto 0) := (others => '0');
    signal c_fl   : unsigned(23 downto 0) := (others => '0');          -- ring flow, Q24 periods
    signal c_rl   : unsigned(23 downto 0) := (others => '0');          -- stripe roll, Q24 cycles
    signal c_hk   : unsigned(15 downto 0) := to_unsigned(6305, 16);   -- black half-width, Q16
    signal c_egq  : unsigned(12 downto 0) := (others => '0');          -- etch gain per luma step >> 6
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);     -- beam start x (h)
    signal c_xhs  : unsigned(11 downto 0) := to_unsigned(132, 12);    -- hold end x (h)
    signal c_h01  : std_logic := '1';
    signal c_p0   : unsigned(29 downto 0) := (others => '0');
    signal c_sta  : unsigned(29 downto 0) := (others => '0');          -- + xs * GA
    signal c_std  : unsigned(29 downto 0) := (others => '0');          -- - xs * GD (two's complement)
    signal c_rg1, c_rg2, c_rg3, c_rg4 : unsigned(29 downto 0) := (others => '0');   -- r * GA
    signal c_bmx  : unsigned(29 downto 0) := (others => '0');          -- twin peak value 960 * GA
    signal c_ep0, c_ep1, c_ep2 : unsigned(11 downto 0) := to_unsigned(4095, 12);
    signal c_ky, c_ku, c_kv : unsigned(9 downto 0) := to_unsigned(512, 10);   -- black of the palette

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
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_dq   : unsigned(17 downto 0) := (others => '0');
    signal e_top  : std_logic := '0';
    signal e_dy8  : unsigned(20 downto 0) := (others => '0');
    signal e_sg   : unsigned(10 downto 0) := (others => '0');
    signal e_sgn  : std_logic := '0';
    signal e_e    : signed(10 downto 0) := (others => '0');
    signal e_sh   : signed(2 downto 0) := (others => '0');
    signal e_ph   : unsigned(23 downto 0) := (others => '0');
    signal m_a    : unsigned(37 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(37 downto 0) := (others => '0');
    signal n_dyg  : unsigned(28 downto 0) := (others => '0');
    signal n_s    : std_logic := '0';

    -- beam
    signal b_x, b_x1, b_x2 : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_p   : unsigned(29 downto 0) := (others => '0');
    signal b_up  : std_logic := '0';
    signal b_fh, b_fc : std_logic := '0';
    signal b_ev  : unsigned(1 downto 0) := (others => '0');
    signal b_pos : unsigned(11 downto 0) := to_unsigned(4095, 12);
    signal b_mx  : std_logic := '0';
    signal b_rg  : unsigned(29 downto 0) := (others => '0');
    signal l_dyg : unsigned(28 downto 0) := (others => '0');
    signal l_s   : std_logic := '0';

    -- pixel path
    signal p2_pd, p3_pd, p4_pd, p5_pd : unsigned(29 downto 0) := (others => '0');
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(16 downto 0) := (others => '0');
    signal p5_le  : unsigned(20 downto 0) := (others => '0');
    signal p6_u   : unsigned(29 downto 0) := (others => '0');
    signal p7_u   : unsigned(23 downto 0) := (others => '0');
    signal p6_eye, p7_eye : std_logic := '0';
    signal p8_v1  : signed(16 downto 0) := (others => '0');
    signal p8_v2  : signed(15 downto 0) := (others => '0');
    signal p8_vb  : signed(16 downto 0) := (others => '0');
    signal p8_ring, p8_eye : std_logic := '0';
    signal p9_k, p9_b, p9_ring, p9_eye : std_logic := '0';
    signal p10_b, p11_b, p11_pv : unsigned(2 downto 0) := (others => '0');
    signal p11_c  : unsigned(8 downto 0) := (others => '0');
    signal p12_act, p13_act, p14_act : std_logic := '0';
    signal p12_bs : unsigned(4 downto 0) := (others => '0');
    signal p13_base, p14_base : std_logic_vector(31 downto 0) := (others => '0');
    signal p14_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
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
            r_ex_q <= C_LZ_EXP2(to_integer(r_ex_a));
            r_el_q <= C_LZ_EXP2(to_integer(r_el_a));
            r_sg_q <= C_LZ_SIG(to_integer(r_sg_a));
            r_pf_q <= C_LZ_PROF(to_integer(r_pf_a));
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
        variable v_t  : unsigned(29 downto 0);
        variable v_k  : std_logic_vector(31 downto 0);
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
            elsif s_zc /= 1023 then
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
                s_flipoff <= registers_in(6)(0);
                s_glow    <= registers_in(6)(1);
                s_twin    <= registers_in(6)(2);
                s_pal     <= registers_in(6)(3);
                s_src     <= registers_in(6)(4);
                s_xs      <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
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
                    when 414 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 438 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when 462 => z_sel <= to_unsigned(18, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_gp(12 downto 3), 36); z_fb <= to_unsigned(1049, 16);     -- rings exponent
                    when 1  => z_fa <= to_unsigned(9166, 36);          z_fb <= z_mt;                      -- G
                    when 2  => z_fa <= resize(c_g, 36);                z_fb <= to_unsigned(4138, 16);     -- GL
                    when 3  => z_fa <= resize(z_a(9 downto 0), 36);    z_fb <= to_unsigned(769, 16);      -- aspect exponent
                    when 4  => z_fa <= to_unsigned(14746, 36);         z_fb <= z_mt;                      -- KAT
                    when 5  => z_fa <= to_unsigned(14664, 36);         z_fb <= z_mt;                      -- KAB
                    when 6  => z_fa <= resize(z_a(9 downto 0), 36);    z_fb <= to_unsigned(1474, 16);     -- |chirp|
                    when 7  => z_fa <= to_unsigned(192026, 36);        z_fb <= z_mt;                      -- stripes
                    when 8  => z_fa <= resize(c_fsb, 36);              z_fb <= resize(s_ys, 16);          -- x line step
                    when 9  => z_fa <= resize(s_vm, 36);               z_fb <= resize(s_vm, 16);          -- speed^2
                    when 10 => z_fa <= resize(c_vsq, 36);              z_fb <= to_unsigned(1131, 16);     -- flow step
                    when 11 => z_fa <= resize(c_vsq, 36);              z_fb <= to_unsigned(566, 16);      -- roll step
                    when 12 =>                                                                            -- black
                        if s_g5(12 downto 3) > 512 then
                            z_fa <= resize(s_g5(12 downto 3) - 512, 36); z_fb <= to_unsigned(12634, 16);
                        else
                            z_fa <= resize(s_g5(12 downto 3), 36);       z_fb <= to_unsigned(5359, 16);
                        end if;
                    when 13 => z_fa <= resize(s_g6(12 downto 3), 36); z_fb <= resize(s_g6(12 downto 3), 16);  -- etch^2
                    when 14 => z_fa <= resize(z_a, 36);                z_fb <= c_g;                       -- etch^2 * G
                    when 15 => z_fa <= resize(z_a, 36);                z_fb <= to_unsigned(7496, 16);
                    when 16 =>                                                                            -- P0
                        z_fa <= resize(c_gd, 36);
                        if s_twin = '1' then z_fb <= resize(to_unsigned(960, 12) - c_xhs, 16);
                        else z_fb <= resize(to_unsigned(C_XC2, 12) - c_xhs, 16); end if;
                    when 17 => z_fa <= resize(c_ga, 36);               z_fb <= to_unsigned(960, 16);      -- twin peak
                    when others => z_fa <= resize(c_ga, 36);           z_fb <= resize(s_xs, 16);          -- step
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
                    -- motion (last field's steps); beam origin
                    when 20 =>
                        if s_vneg = '1' then c_fl <= c_fl - resize(c_dfl, 24); c_rl <= c_rl + resize(c_drl, 24);
                        else c_fl <= c_fl + resize(c_dfl, 24); c_rl <= c_rl - resize(c_drl, 24); end if;
                        c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
                    when 21 =>
                        if c_xb0 > C_XH2 then c_xhs <= c_xb0; else c_xhs <= to_unsigned(C_XH2, 12); end if;
                        if s_twin = '1' then
                            c_ep0 <= to_unsigned(960, 12); c_ep1 <= to_unsigned(1920, 12); c_ep2 <= to_unsigned(2880, 12);
                        else
                            c_ep0 <= to_unsigned(C_XC2, 12); c_ep1 <= to_unsigned(4095, 12); c_ep2 <= to_unsigned(4095, 12);
                        end if;
                        when 22 =>
                        if c_xb0 + s_xs <= c_xhs then c_h01 <= '1'; else c_h01 <= '0'; end if;
                        if s_pal = '1' then v_k := C_LZ_BASE(16); else v_k := C_LZ_BASE(0); end if;
                        c_ky <= unsigned(v_k(31 downto 22)); c_ku <= unsigned(v_k(21 downto 12)); c_kv <= unsigned(v_k(11 downto 2));

                    -- rings: e = -388 + kp * 1049 >> 10
                    when 50  => z_e <= signed(resize(f_acc(20 downto 10), 12)) - 388;
                    when 51  => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 53  => z_mt <= r_ex_q;
                    when 74  =>                       -- G = 9166 * mt >> (15 - sh), sh = -2 .. 2
                        case to_integer(z_sh) is
                            when -2     => c_g <= f_acc(32 downto 17);
                            when -1     => c_g <= f_acc(31 downto 16);
                            when 0      => c_g <= f_acc(30 downto 15);
                            when 1      => c_g <= f_acc(29 downto 14);
                            when others => c_g <= f_acc(28 downto 13);
                        end case;
                    when 98  => c_gl <= f_acc(27 downto 12);
                    when 99  =>
                        if s_twin = '1' then
                            c_ga <= c_g & '0'; c_gd <= c_g & '0';
                        else
                            c_ga <= resize(c_g, 17); c_gd <= resize(c_gl, 17);
                        end if;
                        -- aspect: |k3 - 512|
                        if s_g3(12 downto 3) >= 512 then z_a <= resize(s_g3(12 downto 3) - 512, 20); z_neg <= '0';
                        else z_a <= resize(512 - s_g3(12 downto 3), 20); z_neg <= '1'; end if;
                    when 122 =>
                        v_e := signed(resize(f_acc(19 downto 10), 11));
                        if z_neg = '1' then v_e := -v_e; end if;
                        z_e <= resize(v_e, 12);
                    when 123 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 125 => z_mt <= r_ex_q;
                    when 146 | 170 =>                 -- KAT / KAB = C * mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when -2     => v_t := resize(f_acc(35 downto 17), 30);
                            when -1     => v_t := resize(f_acc(35 downto 16), 30);
                            when 0      => v_t := resize(f_acc(35 downto 15), 30);
                            when others => v_t := resize(f_acc(35 downto 14), 30);
                        end case;
                        if s_zc = 146 then c_kat <= v_t(15 downto 0); else c_kab <= v_t(15 downto 0); end if;
                    when 171 =>                       -- chirp: |k2 - 512| - 8
                        if s_g2(12 downto 3) >= 520 then z_a <= resize(s_g2(12 downto 3) - 520, 20); z_neg <= '0';
                        elsif s_g2(12 downto 3) <= 504 then z_a <= resize(504 - s_g2(12 downto 3), 20); z_neg <= '1';
                        else z_a <= (others => '0'); z_neg <= '0'; end if;
                    when 194 => c_chg <= f_acc(19 downto 10); c_chn <= z_neg;
                    when 195 =>                       -- stripes exponent = k1 - 512
                        v_e := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 197 => z_mt <= r_ex_q;
                    when 218 =>                       -- FSB = 192026 * mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when -2     => c_fsb <= resize(f_acc(35 downto 17), 22);
                            when -1     => c_fsb <= resize(f_acc(35 downto 16), 22);
                            when 0      => c_fsb <= resize(f_acc(35 downto 15), 22);
                            when others => c_fsb <= f_acc(35 downto 14);
                        end case;
                    when 242 => c_fs  <= f_acc(31 downto 8);
                    when 266 => c_vsq <= f_acc(17 downto 0);
                    when 290 => c_dfl <= f_acc(28 downto 10);
                    when 314 => c_drl <= f_acc(27 downto 10);
                    when 338 =>
                        if s_g5(12 downto 3) > 512 then c_hk <= to_unsigned(6305, 16) + f_acc(25 downto 10);
                        else c_hk <= to_unsigned(946, 16) + f_acc(24 downto 9); end if;
                    when 362 => z_a <= f_acc(19 downto 0);
                    when 386 => z_a <= f_acc(35 downto 16);
                    when 410 => c_egq <= f_acc(32 downto 20);
                    when 434 => c_p0  <= f_acc(29 downto 0);
                    when 458 => c_bmx <= f_acc(29 downto 0);
                    when 482 =>
                        c_sta <= f_acc(29 downto 0);
                        c_rg1 <= resize(c_ga, 30);
                        c_rg2 <= shift_left(resize(c_ga, 30), 1);
                        c_rg4 <= shift_left(resize(c_ga, 30), 2);
                    when 483 =>
                        c_rg3 <= c_rg1 + c_rg2;
                        -- descending step = xs * GD, negated
                        case to_integer(s_xs) is
                            when 4      => v_t := shift_left(resize(c_gd, 30), 2);
                            when 3      => v_t := shift_left(resize(c_gd, 30), 1) + resize(c_gd, 30);
                            when others => v_t := shift_left(resize(c_gd, 30), 1);
                        end case;
                        c_std <= (not v_t) + 1;
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
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line: DYG = (|vy - cy| * KA >> 12) * G >> 8 and the
    -- stripe bit / phase step, on one serial multiplier (16 iterations, result
    -- at load + 17)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d : signed(19 downto 0);
        variable v_e : signed(10 downto 0);
        variable v_g : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;
            if s_zone = '1' and s_zc = 590 then e_ph <= to_unsigned(C_PH0, 24) + c_rl; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(36 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    v_d := signed(resize(e_vy, 20)) - C_CY8;
                    if v_d < 0 then v_d := -v_d; e_top <= '1'; else e_top <= '0'; end if;
                    e_dq <= unsigned(v_d(17 downto 0));
                    r_sg_a <= e_vy(18 downto 11);
                    n_s <= not e_ph(23);
                when 2 =>
                    m_a <= resize(e_dq, 38); m_acc <= (others => '0');
                    if e_top = '1' then m_b <= c_kat; else m_b <= c_kab; end if;
                when 4 =>
                    v_g := r_sg_q;
                    if v_g < 0 then v_g := -v_g; e_sgn <= '1'; else e_sgn <= '0'; end if;
                    e_sg <= unsigned(v_g(10 downto 0));
                when 19 =>
                    e_dy8 <= m_acc(32 downto 12);
                    m_a <= resize(m_acc(32 downto 12), 38); m_b <= c_g; m_acc <= (others => '0');
                when 36 =>
                    n_dyg <= m_acc(36 downto 8);
                    m_a <= resize(c_chg, 38); m_b <= resize(e_sg, 16); m_acc <= (others => '0');
                when 53 =>
                    v_e := signed(resize(m_acc(20 downto 12), 11));
                    if (c_chn xor e_sgn) = '1' then v_e := -v_e; end if;
                    e_e <= v_e;
                when 54 =>
                    r_el_a <= unsigned(e_e(7 downto 0)); e_sh <= e_e(10 downto 8);
                when 56 =>
                    m_a <= resize(c_fs, 38); m_b <= r_el_q; m_acc <= (others => '0');
                when 73 =>
                    case to_integer(e_sh) is
                        when -2     => e_ph <= e_ph + resize(m_acc(37 downto 17), 24);
                        when -1     => e_ph <= e_ph + resize(m_acc(37 downto 16), 24);
                        when 0      => e_ph <= e_ph + resize(m_acc(37 downto 15), 24);
                        when others => e_ph <= e_ph + m_acc(37 downto 14);
                    end case;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: P = t_x * gain, exact, as a DDA in h units: held left of the hold
    -- point, descending to the centre fold (or the twin minima), reflecting
    -- there (P = r * GA), and at the twin peak (P = 960 GA - r GA).  The event
    -- flags for the next step are registered a clock ahead.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_op : unsigned(29 downto 0);
        variable v_pa : unsigned(29 downto 0);
        variable v_r  : unsigned(2 downto 0);
        variable v_nx : unsigned(11 downto 0);
        variable v_ev : unsigned(1 downto 0);
        variable v_mx : std_logic;
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= c_xb0;
                b_x1 <= c_xb0 + s_xs;
                b_x2 <= c_xb0 + shift_left(resize(s_xs, 12), 1);
                b_px <= (others => '0');
                b_p  <= c_p0;
                b_up <= '0';
                b_fh <= c_h01;
                b_fc <= '0';
                b_ev <= "00";
                b_pos <= c_ep0;
                b_mx <= '0';
                l_dyg <= n_dyg; l_s <= n_s and not s_flipoff;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x1;
                b_x1 <= b_x2;
                b_x2 <= b_x2 + s_xs;
                -- P for the next pixel
                if b_fh = '1' then
                    v_pa := b_p; v_op := (others => '0');
                elsif b_fc = '1' then
                    if b_mx = '1' then v_pa := c_bmx; v_op := (not b_rg) + 1;
                    else v_pa := (others => '0'); v_op := b_rg; end if;
                elsif b_up = '1' then
                    v_pa := b_p; v_op := c_sta;
                else
                    v_pa := b_p; v_op := c_std;
                end if;
                b_p <= v_pa + v_op;
                -- consume the event
                v_ev := b_ev; v_nx := b_pos; v_mx := b_mx;
                if b_fc = '1' then
                    b_up <= not b_mx;
                    v_ev := b_ev + 1;
                    case to_integer(b_ev) is
                        when 0      => v_nx := c_ep1; v_mx := s_twin;
                        when 1      => v_nx := c_ep2; v_mx := '0';
                        when others => v_nx := to_unsigned(4095, 12); v_mx := '0';
                    end case;
                end if;
                b_ev <= v_ev; b_pos <= v_nx; b_mx <= v_mx;
                -- flags for the step after next: x(n+1) = b_x1, x(n+2) = b_x2
                if b_x2 <= c_xhs then b_fh <= '1'; else b_fh <= '0'; end if;
                if b_x1 <= b_pos and b_x2 > b_pos and b_fc = '0' then b_fc <= '1'; else b_fc <= '0'; end if;
                v_r := b_x2(2 downto 0) - b_pos(2 downto 0);
                case to_integer(v_r) is
                    when 1      => b_rg <= c_rg1;
                    when 2      => b_rg <= c_rg2;
                    when 3      => b_rg <= c_rg3;
                    when others => b_rg <= c_rg4;
                end case;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_u16 : unsigned(15 downto 0);
        variable v_ty  : unsigned(1 downto 0);
        variable v_c   : unsigned(9 downto 0);
        variable v_o   : unsigned(5 downto 0);
        variable v_act : std_logic;
    begin
        if rising_edge(clk) then
            -- 2: t * gain = x term + y term
            p2_pd <= b_p + resize(l_dyg, 30);
            -- 3: luma from the line buffer
            p3_pd <= p2_pd;
            p3_lm <= unsigned(s_lb_q);
            -- 4: etch partials (luma nibbles x gain)
            p4_ea <= resize(p3_lm(3 downto 0) * c_egq, 17);
            p4_eb <= resize(p3_lm(7 downto 4) * c_egq, 17);
            p4_pd <= p3_pd;
            -- 5: etch
            p5_le <= resize(p4_ea, 21) + shift_left(resize(p4_eb, 21), 4);
            p5_pd <= p4_pd;
            -- 6: U = t * gain + luma * etch gain; the eye keys the un-etched field (a fixed pupil)
            p6_u <= p5_pd + shift_left(resize(p5_le, 30), 6);
            if p5_pd < C_EYE then p6_eye <= '1'; else p6_eye <= '0'; end if;
            -- 7: phase (minus the flow)
            p7_u <= p6_u(23 downto 0) - c_fl;
            p7_eye <= p6_eye;
            -- 8: distances to the band centres
            v_u16 := p7_u(23 downto 8);
            p8_v1 <= signed(resize(v_u16, 17)) - C_U1;
            p8_v2 <= signed(v_u16 - to_unsigned(C_U2, 16));
            p8_vb <= signed(resize(v_u16, 17)) - C_UB;
            if v_u16 < C_U1 or v_u16 >= C_U2 then p8_ring <= '1'; else p8_ring <= '0'; end if;
            p8_eye <= p7_eye;
            -- 9: band tests
            if (p8_v1 > -signed(resize(c_hk, 17)) and p8_v1 < signed(resize(c_hk, 17))) or
               (resize(p8_v2, 17) > -signed(resize(c_hk, 17)) and resize(p8_v2, 17) < signed(resize(c_hk, 17))) then
                p9_k <= '1';
            else
                p9_k <= '0';
            end if;
            if p8_vb > -C_HB and p8_vb < C_HB then p9_b <= '1'; else p9_b <= '0'; end if;
            p9_ring <= p8_ring; p9_eye <= p8_eye;
            -- 10: band code (0 black, 1 eye, 2 ring, 3 A, 4 B)
            if p9_eye = '1' then p10_b <= "001";
            elsif p9_k = '1' then p10_b <= "000";
            elsif p9_ring = '1' then p10_b <= "010";
            elsif p9_b = '1' then p10_b <= "100";
            else p10_b <= "011"; end if;
            -- 11: runs: band before the run, offset since the run start (h units, saturating at 511)
            p11_b <= p10_b;
            if s_gav_sr(8) = '1' and s_gav_sr(9) = '0' then
                p11_pv <= "111"; p11_c <= (others => '0');
            elsif p10_b /= p11_b then
                p11_pv <= p11_b; p11_c <= (others => '0');
            else
                v_c := resize(p11_c, 10) + s_xs;
                if v_c > 511 then p11_c <= (others => '1'); else p11_c <= v_c(8 downto 0); end if;
            end if;
            -- 12: line type and ROM address; base colour index
            if p11_b = "000" and (p11_pv = "001" or p11_pv = "010" or p11_pv = "011" or p11_pv = "100") then
                v_act := '1';
            else
                v_act := '0';
            end if;
            if p11_pv = "011" then v_ty := '1' & l_s;
            elsif p11_pv = "100" then v_ty := '1' & not l_s;
            else v_ty := '0' & l_s; end if;
            p12_act <= v_act;
            if s_glow = '1' then v_o := p11_c(8 downto 3);            -- glow: 4 virtual px per entry
            elsif p11_c(8 downto 7) /= "00" then v_o := (others => '1');
            else v_o := p11_c(6 downto 1); end if;                       -- road: 1 virtual px per entry
            r_pf_a <= s_pal & s_glow & v_ty & v_o;
            p12_bs <= s_pal & p11_b & l_s;
            -- 13: ROM data (EBR); base colour
            p13_act <= p12_act;
            p13_base <= C_LZ_BASE(to_integer(p12_bs));
            -- 14: fabric copy
            p14_q <= r_pf_q; p14_act <= p13_act; p14_base <= p13_base;
            -- 15: compose; black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= c_ky; s_u15 <= c_ku; s_v15 <= c_kv;
            elsif p14_act = '1' then
                s_y15 <= unsigned(p14_q(31 downto 22)); s_u15 <= unsigned(p14_q(21 downto 12)); s_v15 <= unsigned(p14_q(11 downto 2));
            else
                s_y15 <= unsigned(p14_base(31 downto 22)); s_u15 <= unsigned(p14_base(21 downto 12)); s_v15 <= unsigned(p14_base(11 downto 2));
            end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the photo's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- box A: stage 16 sums, stage 17 output
            s_cay(0) <= s_y15; s_cau(0) <= s_u15; s_cav(0) <= s_v15;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            s_bay <= s_bay + s_y15 - s_cay(7);
            s_bau <= s_bau + s_u15 - s_cau(7);
            s_bav <= s_bav + s_v15 - s_cav(7);
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
            v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            -- box B: stage 18 sums, stage 19 output
            s_cby(0) <= s_y17; s_cbu(0) <= s_u17; s_cbv(0) <= s_v17;
            for i in 1 to 7 loop s_cby(i) <= s_cby(i - 1); s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            s_bby <= s_bby + s_y17 - s_cby(7);
            s_bbu <= s_bbu + s_u17 - s_cbu(7);
            s_bbv <= s_bbv + s_v17 - s_cbv(7);
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

end architecture lozenge;
