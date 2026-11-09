-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Carpet v0.1.0 -- Heirloom phase 3 (video clips), from WIN_20230722_01_18_41 at 5.5 s.
--
-- The still: a red / white / orange kaleidoscopic carpet on black.  Tall black-cored diamonds ringed in white and
-- red stand in two columns (x 538 / 1483, period 945 px) above and below a mirror row (y 516); flat black lenses
-- sit between them (x 64 / 1012); where the diamonds' and the lenses' rings meet they make X crossings (538, 516)
-- and small bow-tie stars (1012, 253); V-zigzag bands run across the top and bottom.  Thin dark lines edge every
-- white band, yellow fringes the outer edges, and H-sync jitter streaks the diamonds.  In the clip Ron sweeps the
-- ring gain: rings are born on the diamond / lens boundaries and flow into the black cores.
--
-- The patch, as rebuilt (carpet.py has the palette):
--   P1  = s1 g du + g kA (258 - yy) above / g kB (yy - 258) below      tall diamonds: H triangle + asymmetric V
--   P2  = k2x g dl + g k2b w (below the ridge row 253) / g (Ft - qa dr - qb dr^2) above      flat lenses
--   F   = min(P1, P2)  (S8 Max: max)  + Etch * luma
--   t   = F - gap/2;  q = frac(t + flow) / (1 - gap)  (phase units: 4096 per ring cycle)
--   lenses: q < 1 -> A[q], else black;  diamonds: A[q], a dark line, a second copy A[q - 1 - dl], then black;
--   black in the diamond cores (t < 0) and beyond ND cycles.
--
--   K1  Diamonds  diamond width (the H slope of P1)          K2  Zigzag   lens / band angle (the H slope of P2)
--   K3  Tiles     diamond columns across (1 .. 6)            K4  Speed    bipolar ring flow
--   K5  Jitter    H-sync jitter streaks                       K6  Etch     camera / Osc luma bends the field
--   S7  Rows      Mirror (the photo) / Double                 S8  Weave    Min (the photo) / Max
--   S9  Palette   Ember / Ice                                  S10 Stagger  lens rows on the diamond rows
--   S11 Source    Video / Osc                                  P12 FOLDS    ring gain and black duty (the clip's knob)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, exp2 ROM), a per-line engine kicked at each
-- line start (the next line's V terms), two sawtooth DDAs folded into the H triangles (no pixel multiplier for the
-- field), a 21-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output
-- conditioning (Y, U, V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the running-sum resync, luma 13/16) before the
-- blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.carpet_pkg.all;
use work.all;

architecture carpet of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant C_XH2 : integer := 190;        -- line-start hold (h = half virtual px): x < 95 px is flat
    constant C_PHC : integer := 1826;       -- H phase at x = 960, Q12 (0.4456)
    constant C_QB  : integer := 262144 + 18 * 1024;   -- second diamond copy starts at q = 1 + dark line (Q18)

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

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_ki_a : unsigned(7 downto 0) := (others => '0');
    signal r_ki_q : unsigned(7 downto 0) := (others => '0');
    signal r_pl_a : unsigned(8 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

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

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(320 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(409 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(680, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_rows2, s_max, s_pal, s_stag, s_src : std_logic := '0';

    -- per-field scratch and constants
    signal z_a    : unsigned(19 downto 0) := (others => '0');
    signal z_hi, z_n1, z_n2, z_n3 : std_logic := '0';
    signal z_lg, z_l1, z_l2, z_l3 : signed(11 downto 0) := (others => '0');
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_m0, z_mt : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_s0, z_s1, z_s2, z_s3, z_s4 : signed(3 downto 0) := (others => '0');
    signal z_a5, z_a6 : unsigned(11 downto 0) := (others => '0');
    signal z_sp : unsigned(17 downto 0) := (others => '0');
    signal z_ph : unsigned(11 downto 0) := (others => '0');
    signal c_gapq : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal c_nd   : unsigned(3 downto 0) := to_unsigned(1, 4);
    signal c_mg   : unsigned(15 downto 0) := to_unsigned(4096, 16);   -- 2^lg, Q12
    signal c_g1, c_g2 : unsigned(16 downto 0) := to_unsigned(5191, 17); -- H slopes, Q8 phase units per vpx
    signal c_t    : unsigned(16 downto 0) := to_unsigned(17766, 17);   -- H phase per vpx, Q24
    signal c_p    : unsigned(16 downto 0) := to_unsigned(15109, 17);   -- H period, Q4 vpx
    signal c_w1, c_w2   : unsigned(27 downto 0) := (others => '0');    -- sawtooth wraps (Q8 phase units)
    signal c_st1, c_st2 : unsigned(27 downto 0) := (others => '0');    -- steps per pixel
    signal c_sw1, c_sw2 : unsigned(27 downto 0) := (others => '0');    -- step - wrap (two's complement)
    signal c_r10, c_r20 : unsigned(27 downto 0) := (others => '0');    -- line-start values
    signal c_gka, c_gkb, c_gk2b, c_gqa, c_gqb : unsigned(15 downto 0) := (others => '0');
    signal c_gft  : unsigned(15 downto 0) := (others => '0');
    signal c_j1, c_j2, c_j3 : unsigned(13 downto 0) := (others => '0');
    signal c_e    : unsigned(7 downto 0) := (others => '0');
    signal c_dfl  : unsigned(17 downto 0) := (others => '0');
    signal c_off  : unsigned(19 downto 0) := (others => '0');          -- ring flow, Q20 cycles
    signal c_gapo : unsigned(11 downto 0) := to_unsigned(1024, 12);    -- gap / 2, Q12
    signal c_km   : unsigned(5 downto 0) := to_unsigned(32, 6);        -- 1 / (1 - gap) = km << ke (Q8)
    signal c_ke   : unsigned(1 downto 0) := "01";
    signal c_ky, c_ku, c_kv, c_dy, c_du, c_dv : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- field serial multiplier (16 iterations, 36-bit accumulator)
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
    signal e_y1   : unsigned(15 downto 0) := (others => '0');
    signal e_y2   : unsigned(15 downto 0) := (others => '0');
    signal e_yy, e_yy2 : unsigned(13 downto 0) := (others => '0');
    signal e_w2   : unsigned(13 downto 0) := (others => '0');
    signal e_ady  : unsigned(13 downto 0) := (others => '0');
    signal e_top  : std_logic := '0';
    signal e_dr   : unsigned(11 downto 0) := (others => '0');
    signal e_up   : std_logic := '0';
    signal e_tb   : unsigned(15 downto 0) := (others => '0');
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(31 downto 0) := (others => '0');
    signal n_v1   : signed(19 downto 0) := (others => '0');
    signal n_v2   : signed(19 downto 0) := (others => '0');

    -- beam
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_x2  : unsigned(11 downto 0) := (others => '0');
    signal b_h   : std_logic := '0';
    signal b_r1, b_r2 : unsigned(27 downto 0) := (others => '0');
    signal b_jt  : std_logic := '0';
    signal b_j   : signed(2 downto 0) := (others => '0');
    signal b_lf  : std_logic_vector(15 downto 0) := x"ACE1";
    signal l_v1, l_v2 : signed(19 downto 0) := (others => '0');

    -- pixel path
    signal p2_f1, p2_f2 : unsigned(19 downto 0) := (others => '0');
    signal p2_up : std_logic := '0';
    signal p2_j  : signed(2 downto 0) := (others => '0');
    signal p3_x1, p3_x2 : signed(20 downto 0) := (others => '0');
    signal p3_lm : unsigned(7 downto 0) := (others => '0');
    signal p4_p1, p4_p2 : signed(20 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(11 downto 0) := (others => '0');
    signal p5_f  : signed(20 downto 0) := (others => '0');
    signal p5_dom : std_logic := '0';
    signal p5_le : unsigned(15 downto 0) := (others => '0');
    signal p6_t  : signed(21 downto 0) := (others => '0');
    signal p6_dom : std_logic := '0';
    signal p7_fr : unsigned(9 downto 0) := (others => '0');
    signal p7_kill, p7_dom : std_logic := '0';
    signal p8_m  : unsigned(15 downto 0) := (others => '0');
    signal p8_kill, p8_dom : std_logic := '0';
    signal p9_q  : unsigned(21 downto 0) := (others => '0');
    signal p9_kill, p9_dom : std_logic := '0';
    signal p10_q, p10_qb : signed(22 downto 0) := (others => '0');
    signal p10_kill, p10_dom : std_logic := '0';
    signal p11_a, p11_b : unsigned(7 downto 0) := (others => '0');
    signal p11_lt, p11_bn, p11_blt, p11_kill, p11_dom : std_logic := '0';
    signal p12_c, p13_c, p14_c : unsigned(1 downto 0) := (others => '0');   -- 0 ROM, 1 black, 2 dark
    signal p14_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    -- the boxes' 8-back taps come from two 256 x 32 block-RAM rings (write at wp, read at wp - 6, plus the EBR
    -- output register and a fabric copy = the sample 8 clocks back); the read never meets the write address
    type t_ring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal ra_ram, rb_ram : t_ring := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of ra_ram : signal is true;
    attribute no_rw_check of rb_ram : signal is true;
    signal r_wp : unsigned(7 downto 0) := (others => '0');
    signal ra_q, rb_q, ra_f, rb_f : std_logic_vector(31 downto 0) := (others => '0');
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
            r_ex_q <= C_CP_EXP2(to_integer(r_ex_a));
            r_ki_q <= C_CP_KINV(to_integer(r_ki_a));
            r_pl_q <= C_CP_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_p  : unsigned(9 downto 0);
        variable v_t  : unsigned(39 downto 0);
        variable v_k  : std_logic_vector(31 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_g  : unsigned(16 downto 0);
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
                s_rows2 <= registers_in(6)(0);
                s_max   <= registers_in(6)(1);
                s_pal   <= registers_in(6)(2);
                s_stag  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

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

            -- serial products: slot k is decoded at zc = 30 + 32 k, pre-muxed at 31 + 32 k, loaded at 32 + 32 k;
            -- 16 iterations, the result is stable from 49 + 32 k (stored at 58 + 32 k)
            z_pre <= '0';
            if s_zone = '1' and s_zc(4 downto 0) = "11110" and s_zc < 830 then
                z_sel <= resize(shift_right(s_zc - 30, 5), 5); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  =>                                                                   -- folds exponent
                        z_fa <= resize(z_a(9 downto 0), 36);
                        if z_hi = '1' then z_fb <= to_unsigned(192, 16); else z_fb <= to_unsigned(184, 16); end if;
                    when 1  => z_fa <= resize(z_a(19 downto 10), 36); z_fb <= to_unsigned(169, 16);   -- diamonds
                    when 2  => z_fa <= resize(z_ph(9 downto 0), 36); z_fb <= to_unsigned(205, 16);    -- zigzag
                    when 3  =>                                                                   -- tiles
                        z_fa <= resize(unsigned(z_e(9 downto 0)), 36);
                        if z_n3 = '0' then z_fb <= to_unsigned(203, 16); else z_fb <= to_unsigned(128, 16); end if;
                    when 4  => z_fa <= resize(s_g5(12 downto 3), 36); z_fb <= resize(s_g5(12 downto 3), 16);   -- jitter^2
                    when 5  => z_fa <= resize(s_g6(12 downto 3), 36); z_fb <= resize(s_g6(12 downto 3), 16);   -- etch^2
                    when 6  => z_fa <= resize(s_vm, 36);              z_fb <= resize(s_vm, 16);                -- speed^2
                    when 7  => z_fa <= to_unsigned(5191, 36);         z_fb <= z_mt;                            -- G1
                    when 8  => z_fa <= to_unsigned(1298, 36);         z_fb <= z_mt;                            -- G2
                    when 9  => z_fa <= to_unsigned(17766, 36);        z_fb <= z_mt;                            -- T
                    when 10 => z_fa <= to_unsigned(15109, 36);        z_fb <= z_mt;                            -- P
                    when 11 => z_fa <= resize(c_g1, 36);              z_fb <= c_p(15 downto 0);                -- W1
                    when 12 => z_fa <= resize(c_g2, 36);              z_fb <= c_p(15 downto 0);                -- W2
                    when 13 =>                                                                   -- (960 - x0) T
                        z_fa <= resize(c_t, 36);
                        if s_xs = 4 then z_fb <= to_unsigned(704, 16); else z_fb <= to_unsigned(865, 16); end if;
                    when 14 => z_fa <= resize(c_w1(27 downto 4), 36); z_fb <= resize(z_ph, 16);                -- R1 start
                    when 15 => z_fa <= resize(c_w2(27 downto 4), 36); z_fb <= resize(z_ph, 16);                -- R2 start
                    when 16 => z_fa <= to_unsigned(3582, 36);         z_fb <= c_mg;                            -- g kA
                    when 17 => z_fa <= to_unsigned(2160, 36);         z_fb <= c_mg;                            -- g kB
                    when 18 => z_fa <= to_unsigned(6464, 36);         z_fb <= c_mg;                            -- g k2b
                    when 19 => z_fa <= to_unsigned(4091, 36);         z_fb <= c_mg;                            -- g qa
                    when 20 => z_fa <= to_unsigned(6351, 36);         z_fb <= c_mg;                            -- g qb
                    when 21 => z_fa <= to_unsigned(6640, 36);         z_fb <= c_mg;                            -- g Ft
                    when 22 => z_fa <= resize(c_g1, 36);              z_fb <= resize(z_a5, 16);                -- jitter
                    when 23 => z_fa <= resize(z_a6, 36);              z_fb <= c_mg;                            -- etch
                    when others => z_fa <= resize(z_sp, 36);          z_fb <= to_unsigned(106, 16);            -- flow
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

            -- exp2 lookups: z_e (Q8 log2) -> mantissa (Q15) and integer shift
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 20 =>
                        if s_vneg = '1' then c_off <= c_off - resize(c_dfl, 20); else c_off <= c_off + resize(c_dfl, 20); end if;
                        if s_pal = '1' then v_k := C_CP_BASE(2); else v_k := C_CP_BASE(0); end if;
                        c_ky <= unsigned(v_k(31 downto 22)); c_ku <= unsigned(v_k(21 downto 12)); c_kv <= unsigned(v_k(11 downto 2));
                    when 21 =>
                        if s_pal = '1' then v_k := C_CP_BASE(3); else v_k := C_CP_BASE(1); end if;
                        c_dy <= unsigned(v_k(31 downto 22)); c_du <= unsigned(v_k(21 downto 12)); c_dv <= unsigned(v_k(11 downto 2));
                    when 24 =>
                        -- P12: gap (Q8), ND, and the exponent operand
                        v_p := s_gp(12 downto 3);
                        if v_p <= 409 then
                            z_a <= resize(v_p, 20); z_hi <= '0';
                            c_nd <= to_unsigned(1, 4);
                        else
                            z_a <= resize(v_p - 409, 20); z_hi <= '1';
                            v_g := resize(v_p - 409, 17) + shift_left(resize(v_p - 409, 17), 1);   -- 3 (p - 409)
                            c_nd <= resize(v_g(11 downto 8), 4) + 1;
                        end if;
                    when 25 =>
                        -- gap: 200 -> 128 over 0..409 (x 45/256), 128 -> 0 over 409..1023 (x 53/256)
                        if z_hi = '0' then
                            v_g := shift_left(resize(z_a(9 downto 0), 17), 5) + shift_left(resize(z_a(9 downto 0), 17), 3)
                                   + shift_left(resize(z_a(9 downto 0), 17), 2) + resize(z_a(9 downto 0), 17);
                            c_gapq <= to_unsigned(200, 8) - v_g(15 downto 8);
                        else
                            v_g := shift_left(resize(z_a(9 downto 0), 17), 5) + shift_left(resize(z_a(9 downto 0), 17), 4)
                                   + shift_left(resize(z_a(9 downto 0), 17), 2) + resize(z_a(9 downto 0), 17);
                            if v_g(15 downto 8) >= 128 then c_gapq <= (others => '0');
                            else c_gapq <= to_unsigned(128, 8) - v_g(15 downto 8); end if;
                        end if;
                        -- K1 (diamonds): l1 = (512 - k1) * 169 / 256;  K2 (zigzag): l2 = (k2 - 512) * 205 / 256
                        if s_g1(12 downto 3) <= 512 then
                            z_a(19 downto 10) <= to_unsigned(512, 10) - s_g1(12 downto 3); z_n1 <= '0';
                        else
                            z_a(19 downto 10) <= s_g1(12 downto 3) - 512; z_n1 <= '1';
                        end if;
                        if s_g2(12 downto 3) >= 512 then
                            z_ph <= resize(s_g2(12 downto 3) - 512, 12); z_n2 <= '0';
                        else
                            z_ph <= resize(512 - s_g2(12 downto 3), 12); z_n2 <= '1';
                        end if;
                        -- K3 (tiles): above 512: (k3 - 512) * 203 / 256 (3^e), below: (512 - k3) / 2
                        if s_g3(12 downto 3) >= 512 then
                            z_e <= signed(resize(s_g3(12 downto 3) - 512, 12)); z_n3 <= '0';
                        else
                            z_e <= signed(resize(512 - s_g3(12 downto 3), 12)); z_n3 <= '1';
                        end if;
                        -- gap -> K = 1 / (1 - gap) (ROM), gap / 2 in Q12
                    when 26 =>
                        r_ki_a <= c_gapq;
                        c_gapo <= resize(c_gapq & "000", 12);
                    when 28 =>
                        c_km <= r_ki_q(5 downto 0); c_ke <= r_ki_q(7 downto 6);

                    -- slot results
                    when 58  =>                       -- lg (Q8) = r / 256 - 294 (p <= 409) or r / 256
                        if z_hi = '0' then z_lg <= signed(resize(f_acc(17 downto 8), 12)) - 294;
                        else z_lg <= signed(resize(f_acc(17 downto 8), 12)); end if;
                    when 90  =>
                        if z_n1 = '0' then z_l1 <= signed(resize(f_acc(17 downto 8), 12));
                        else z_l1 <= -signed(resize(f_acc(17 downto 8), 12)); end if;
                    when 122 =>
                        if z_n2 = '0' then z_l2 <= signed(resize(f_acc(17 downto 8), 12));
                        else z_l2 <= -signed(resize(f_acc(17 downto 8), 12)); end if;
                    when 154 =>
                        if z_n3 = '0' then z_l3 <= signed(resize(f_acc(17 downto 8), 12));
                        else z_l3 <= -signed(resize(f_acc(17 downto 8), 12)); end if;
                    when 186 => z_a5 <= f_acc(19 downto 8);
                    when 218 => z_a6 <= f_acc(19 downto 8);
                    when 250 => z_sp <= f_acc(17 downto 0);

                    -- exp lookups (2-clock ROM), each just before its slot: lg and l1 + lg (slot 7), l2 + lg (8),
                    -- l3 (9), -l3 (10)
                    when 251 => v_e := z_lg;          r_ex_a <= unsigned(v_e(7 downto 0)); z_s0 <= v_e(11 downto 8);
                    when 252 => v_e := z_l1 + z_lg;   r_ex_a <= unsigned(v_e(7 downto 0)); z_s1 <= v_e(11 downto 8);
                    when 253 => z_m0 <= r_ex_q;
                    when 254 => z_mt <= r_ex_q;
                    when 255 =>
                                -- Mg = 2^lg in Q12: mantissa Q15 >> (3 - s0), s0 = -2 .. 1
                                case to_integer(z_s0) is
                                    when -2     => c_mg <= resize(z_m0(15 downto 5), 16);
                                    when -1     => c_mg <= resize(z_m0(15 downto 4), 16);
                                    when 0      => c_mg <= resize(z_m0(15 downto 3), 16);
                                    when others => c_mg <= resize(z_m0(15 downto 2), 16);
                                end case;
                    when 284 => v_e := z_l2 + z_lg;   r_ex_a <= unsigned(v_e(7 downto 0)); z_s2 <= v_e(11 downto 8);
                    when 286 => z_mt <= r_ex_q;
                    when 316 => v_e := z_l3;          r_ex_a <= unsigned(v_e(7 downto 0)); z_s3 <= v_e(11 downto 8);
                    when 318 => z_mt <= r_ex_q;
                    when 348 => v_e := -z_l3;         r_ex_a <= unsigned(v_e(7 downto 0)); z_s4 <= v_e(11 downto 8);
                    when 350 => z_mt <= r_ex_q;
                    -- G1 = 5191 m1 >> (15 - s1), s1 = -3 .. 3 (Q8 phase units per vpx)
                    when 282 =>
                        case to_integer(z_s1) is
                            when -4 | -3 => c_g1 <= resize(f_acc(33 downto 18), 17);
                            when -2     => c_g1 <= resize(f_acc(33 downto 17), 17);
                            when -1     => c_g1 <= resize(f_acc(32 downto 16), 17);
                            when 0      => c_g1 <= f_acc(31 downto 15);
                            when 1      => c_g1 <= f_acc(30 downto 14);
                            when 2      => c_g1 <= f_acc(29 downto 13);
                            when others => c_g1 <= f_acc(28 downto 12);
                        end case;
                    when 314 =>
                        case to_integer(z_s2) is
                            when -4 | -3 => c_g2 <= resize(f_acc(33 downto 18), 17);
                            when -2     => c_g2 <= resize(f_acc(33 downto 17), 17);
                            when -1     => c_g2 <= resize(f_acc(32 downto 16), 17);
                            when 0      => c_g2 <= f_acc(31 downto 15);
                            when 1      => c_g2 <= f_acc(30 downto 14);
                            when 2      => c_g2 <= f_acc(29 downto 13);
                            when others => c_g2 <= f_acc(28 downto 12);
                        end case;
                    when 346 =>                       -- T (Q24 phase per vpx), s3 = -1 .. 1
                        case to_integer(z_s3) is
                            when -1     => c_t <= resize(f_acc(32 downto 16), 17);
                            when 0      => c_t <= f_acc(31 downto 15);
                            when others => c_t <= f_acc(30 downto 14);
                        end case;
                    when 378 =>                       -- P (Q4 vpx), s4 = -2 .. 1
                        case to_integer(z_s4) is
                            when -2     => c_p <= resize(f_acc(33 downto 17), 17);
                            when -1     => c_p <= resize(f_acc(32 downto 16), 17);
                            when 0      => c_p <= f_acc(31 downto 15);
                            when others => c_p <= f_acc(30 downto 14);
                        end case;
                    when 410 => c_w1 <= f_acc(31 downto 4);                        -- W1 = G1 P / 16 (Q8)
                    when 442 => c_w2 <= f_acc(31 downto 4);
                    when 474 =>                       -- start phase = 0.4456 - (960 - x0) T, Q12
                        z_ph <= to_unsigned(C_PHC, 12) - f_acc(23 downto 12);
                    when 506 => c_r10 <= f_acc(35 downto 8);                       -- R1 start = phase W1
                    when 538 => c_r20 <= f_acc(35 downto 8);
                    when 570 => c_gka  <= f_acc(27 downto 12);
                    when 602 => c_gkb  <= f_acc(27 downto 12);
                    when 634 => c_gk2b <= f_acc(27 downto 12);
                    when 666 => c_gqa  <= f_acc(27 downto 12);
                    when 698 => c_gqb  <= f_acc(27 downto 12);
                    when 730 => c_gft  <= f_acc(27 downto 12);
                    when 762 => c_j1   <= f_acc(30 downto 17);                     -- jitter step (Q12 per unit j)
                    when 763 => c_j2 <= c_j1(12 downto 0) & '0';
                    when 764 => c_j3 <= c_j1 + c_j2;
                    when 794 => c_e    <= f_acc(25 downto 18);                     -- etch gain per luma, >> 1 in the path
                    when 826 => c_dfl  <= f_acc(27 downto 10);                     -- flow step per field (Q20)
                    when 830 =>                       -- steps per pixel = G xs / 2 (Q20 = G Q8 << 12 ... in R units)
                        case to_integer(s_xs) is
                            when 4      => c_st1 <= shift_left(resize(c_g1, 28), 1); c_st2 <= shift_left(resize(c_g2, 28), 1);
                            when 3      => c_st1 <= resize(c_g1, 28) + shift_right(resize(c_g1, 28), 1);
                                           c_st2 <= resize(c_g2, 28) + shift_right(resize(c_g2, 28), 1);
                            when others => c_st1 <= resize(c_g1, 28); c_st2 <= resize(c_g2, 28);
                        end case;
                    when 831 =>
                        c_sw1 <= c_st1 - c_w1; c_sw2 <= c_st2 - c_w2;
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
    -- Generated avid (parity-locked start 2/3 clocks after the source's avid rise, field-latched width)
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
    -- Source side: oscillator (2-D triangle lattice, drifting), line-buffer write of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
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
    -- Line engine: kicked at each line start (and once in the zone for line 0), computes the NEXT line's
    -- V terms (Q12 phase units) on one serial multiplier (24 iterations, result at load + 25):
    --   V1 = g kA |dys| (above the diamond row) or g kB dys (below)
    --   V2 = g k2b w2 (below the ridge row) or g Ft - g qa dr - g qb dr^2 (above)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y : unsigned(16 downto 0);
        variable v_d : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 900 then e_zk <= '1'; end if;

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
            m_a <= m_a(30 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>                     -- y (Q4 virtual lines); Rows Double: 2y
                    if s_rows2 = '1' then e_y1 <= resize(e_vy(18 downto 3), 16);
                    else e_y1 <= resize(e_vy(18 downto 4), 16); end if;
                when 2 =>                     -- mod 1032 lines (16512 Q4)
                    if e_y1 >= 33024 then e_y1 <= e_y1 - 33024;
                    elsif e_y1 >= 16512 then e_y1 <= e_y1 - 16512; end if;
                when 3 =>
                    if e_y1 >= 16512 then e_y1 <= e_y1 - 16512; end if;
                when 4 =>                     -- yy = 516 - |y - 516|; Stagger: the lens rows half a cell on
                    if e_y1 >= 8256 then e_yy <= resize(16512 - e_y1, 14); else e_yy <= e_y1(13 downto 0); end if;
                    if s_stag = '1' then v_y := resize(e_y1, 17) + 4128; else v_y := resize(e_y1, 17); end if;
                    if v_y >= 16512 then v_y := v_y - 16512; end if;
                    e_y2 <= v_y(15 downto 0);
                when 5 =>
                    if e_y2 >= 8256 then e_yy2 <= resize(16512 - e_y2, 14); else e_yy2 <= e_y2(13 downto 0); end if;
                    if e_yy < 4128 then e_ady <= 4128 - e_yy; e_top <= '1'; else e_ady <= e_yy - 4128; e_top <= '0'; end if;
                when 6 =>
                    e_w2 <= 8256 - e_yy2;
                    v_d := signed(resize(to_unsigned(4048, 14), 16)) - signed(resize(e_yy2, 16));
                    if v_d > 0 then e_up <= '1'; e_dr <= unsigned(v_d(11 downto 0)); else e_up <= '0'; e_dr <= (others => '0'); end if;
                    m_acc <= (others => '0');
                    m_b <= resize(e_ady, 16);
                    if e_top = '1' then m_a <= resize(c_gka, 32); else m_a <= resize(c_gkb, 32); end if;
                when 31 =>                    -- the V terms carry - gap / 2
                    n_v1 <= signed(resize(m_acc(31 downto 12), 20)) - signed(resize(c_gapo, 20));
                    m_acc <= (others => '0');
                    if e_up = '1' then m_a <= resize(e_dr, 32); m_b <= resize(e_dr, 16);
                    else m_a <= resize(c_gk2b, 32); m_b <= resize(e_w2, 16); end if;
                when 56 =>
                    if e_up = '0' then
                        n_v2 <= signed(resize(m_acc(31 downto 12), 20)) - signed(resize(c_gapo, 20));
                    else
                        m_b <= m_acc(23 downto 8); m_a <= resize(c_gqb, 32); m_acc <= (others => '0');
                    end if;
                when 81 =>
                    if e_up = '1' then
                        e_tb <= m_acc(31 downto 16);
                        m_a <= resize(c_gqa, 32); m_b <= resize(e_dr, 16); m_acc <= (others => '0');
                    end if;
                when 106 =>
                    if e_up = '1' then
                        n_v2 <= signed(resize(c_gft, 20)) - signed(resize(m_acc(31 downto 12), 20)) - signed(resize(e_tb, 20));
                    end if;
                when 107 =>
                    if e_up = '1' then n_v2 <= n_v2 - signed(resize(c_gapo, 20)); end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: two sawtooth DDAs (R1, R2 in [0, W)) at the H phase, held left of x = 95 px; jitter j redrawn every
    -- 16 virtual px from an LFSR
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_a, v_b : unsigned(27 downto 0);
    begin
        if rising_edge(clk) then
            b_lf <= b_lf(14 downto 0) & (b_lf(15) xor b_lf(13) xor b_lf(12) xor b_lf(10));
            if g_start = '1' then
                b_r1 <= c_r10; b_r2 <= c_r20;
                -- the beam runs 8 pixels ahead (pixel n is drawn at x0 + (n + 8) xs): the two output boxes
                -- delay the picture by 8 pixels
                b_x2 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3) + shift_left(resize(s_xs, 12), 1);
                if resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3) + s_xs < C_XH2 then b_h <= '1'; else b_h <= '0'; end if;
                b_px <= (others => '0');
                l_v1 <= n_v1; l_v2 <= n_v2;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                if b_h = '0' then
                    v_a := b_r1 + c_sw1; v_b := b_r1 + c_st1;
                    if v_a(27) = '0' then b_r1 <= v_a; else b_r1 <= v_b; end if;
                    v_a := b_r2 + c_sw2; v_b := b_r2 + c_st2;
                    if v_a(27) = '0' then b_r2 <= v_a; else b_r2 <= v_b; end if;
                end if;
                if b_x2 < C_XH2 then b_h <= '1'; else b_h <= '0'; end if;
                b_x2 <= b_x2 + s_xs;
                b_jt <= b_x2(5);
                if b_jt /= b_x2(5) then b_j <= signed(b_lf(2 downto 0)); end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_d1, v_d2 : unsigned(27 downto 0);
        variable v_jd : unsigned(13 downto 0);
        variable v_fr : unsigned(11 downto 0);
        variable v_lt : std_logic;
    begin
        if rising_edge(clk) then
            -- 2: fold the sawtooths into the H triangles (the two share their phase, so R1 decides both)
            v_d1 := c_w1 - b_r1; v_d2 := c_w2 - b_r2;
            if b_r1 < ('0' & c_w1(27 downto 1)) then
                p2_f1 <= b_r1(27 downto 8); p2_f2 <= b_r2(27 downto 8); p2_up <= '1';
            else
                p2_f1 <= v_d1(27 downto 8); p2_f2 <= v_d2(27 downto 8); p2_up <= '0';
            end if;
            p2_j <= b_j;
            -- 3: P1x +- jitter (the H oscillator jitters: +j on the rising side, -j on the falling side),
            --    P2x = W2 / 2 - fold(R2); luma from the line buffer
            case to_integer(p2_j) is
                when 1 | -1 => v_jd := c_j1;
                when 2 | -2 => v_jd := c_j2;
                when 3 | -3 => v_jd := c_j3;
                when -4     => v_jd := c_j2(12 downto 0) & '0';
                when others => v_jd := (others => '0');
            end case;
            if (p2_j(2) xor p2_up) = '1' then
                p3_x1 <= signed(resize(p2_f1, 21)) + signed(resize(v_jd, 21));
            else
                p3_x1 <= signed(resize(p2_f1, 21)) - signed(resize(v_jd, 21));
            end if;
            p3_x2 <= signed(resize(c_w2(27 downto 9), 21)) - signed(resize(p2_f2, 21));
            p3_lm <= unsigned(s_lb_q);
            -- 4: + the line's V terms (which carry - gap / 2); etch nibble products
            p4_p1 <= p3_x1 + resize(l_v1, 21);
            p4_p2 <= p3_x2 + resize(l_v2, 21);
            p4_ea <= resize(p3_lm(3 downto 0) * c_e, 12);
            p4_eb <= resize(p3_lm(7 downto 4) * c_e, 12);
            -- 5: Min (the photo): F = min(P1, P2), diamonds where P1 wins; Max: F = max
            if p4_p1 <= p4_p2 then v_lt := '1'; else v_lt := '0'; end if;
            v_lt := v_lt xor s_max;
            if v_lt = '1' then p5_f <= p4_p1; else p5_f <= p4_p2; end if;
            p5_dom <= v_lt;
            p5_le <= resize(p4_ea, 16) + shift_left(resize(p4_eb, 16), 4);
            -- 6: t = F + etch (- gap / 2 is in the V terms)
            p6_t <= resize(p5_f, 22) + signed(resize(p5_le(15 downto 1), 22));
            p6_dom <= p5_dom;
            -- 7: ring group n = t >> 12 (diamonds: black in the core and beyond ND); frac(t + flow)
            v_fr := unsigned(p6_t(11 downto 0)) + c_off(19 downto 8);
            p7_fr <= v_fr(11 downto 2);
            if p6_dom = '1' and (p6_t(21) = '1' or unsigned(p6_t(20 downto 12)) >= resize(c_nd, 9)) then p7_kill <= '1';
            else p7_kill <= '0'; end if;
            p7_dom <= p6_dom;
            -- 8, 9: q = frac / (1 - gap) = (frac * km) << ke (Q18: 1 = 262144)
            p8_m <= p7_fr * c_km;
            p8_kill <= p7_kill; p8_dom <= p7_dom;
            case c_ke is
                when "00"   => p9_q <= shift_left(resize(p8_m, 22), 3);
                when "01"   => p9_q <= shift_left(resize(p8_m, 22), 4);
                when others => p9_q <= shift_left(resize(p8_m, 22), 5);
            end case;
            p9_kill <= p8_kill; p9_dom <= p8_dom;
            -- 10: the second diamond copy's index
            p10_q  <= signed(resize(p9_q, 23));
            p10_qb <= signed(resize(p9_q, 23)) - C_QB;
            p10_kill <= p9_kill; p10_dom <= p9_dom;
            -- 11: range flags and both ROM addresses
            if p10_q < 262144 then p11_lt <= '1'; else p11_lt <= '0'; end if;
            p11_bn <= p10_qb(22);
            if p10_qb < 262144 then p11_blt <= '1'; else p11_blt <= '0'; end if;
            p11_a <= unsigned(p10_q(17 downto 10)); p11_b <= unsigned(p10_qb(17 downto 10));
            p11_kill <= p10_kill; p11_dom <= p10_dom;
            -- 12: colour code and palette address
            if p11_kill = '1' then
                p12_c <= "01";
            elsif p11_lt = '1' then
                p12_c <= "00"; r_pl_a <= s_pal & p11_a;
            elsif p11_dom = '0' then
                p12_c <= "01";
            elsif p11_bn = '1' then
                p12_c <= "10";
            elsif p11_blt = '1' then
                p12_c <= "00"; r_pl_a <= s_pal & p11_b;
            else
                p12_c <= "01";
            end if;
            -- 13: palette ROM (EBR); 14: fabric copy
            p13_c <= p12_c;
            p14_q <= r_pl_q; p14_c <= p13_c;
            -- 15: compose; palette black outside the generated avid
            if s_gav_sr(12) = '0' or p14_c = "01" then
                s_y15 <= c_ky; s_u15 <= c_ku; s_v15 <= c_kv;
            elsif p14_c = "10" then
                s_y15 <= c_dy; s_u15 <= c_du; s_v15 <= c_dv;
            else
                s_y15 <= unsigned(p14_q(31 downto 22)); s_u15 <= unsigned(p14_q(21 downto 12)); s_v15 <= unsigned(p14_q(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (each two adjacent 8-sums; the
    -- photo's horizontal softness and the HD HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_ra : process(clk)
    begin
        if rising_edge(clk) then
            ra_q <= ra_ram(to_integer(r_wp - 6));
            ra_ram(to_integer(r_wp)) <= std_logic_vector(s_y15) & std_logic_vector(s_u15) & std_logic_vector(s_v15) & "00";
        end if;
    end process p_ra;

    p_rb : process(clk)
    begin
        if rising_edge(clk) then
            rb_q <= rb_ram(to_integer(r_wp - 6));
            rb_ram(to_integer(r_wp)) <= std_logic_vector(s_y17) & std_logic_vector(s_u17) & std_logic_vector(s_v17) & "00";
        end if;
    end process p_rb;

    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to the palette black once 8 blank samples are in (resync).
            r_wp <= r_wp + 1;
            ra_f <= ra_q; rb_f <= rb_q;
            if s_bah = "00000000" then
                s_bay <= c_ky & "000"; s_bau <= c_ku & "000"; s_bav <= c_kv & "000";
            else
                s_bay <= s_bay + s_y15 - unsigned(ra_f(31 downto 22));
                s_bau <= s_bau + s_u15 - unsigned(ra_f(21 downto 12));
                s_bav <= s_bav + s_v15 - unsigned(ra_f(11 downto 2));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= c_ky; s_u17 <= c_ku; s_v17 <= c_kv;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            if s_bbh = "00000000" then
                s_bby <= c_ky & "000"; s_bbu <= c_ku & "000"; s_bbv <= c_kv & "000";
            else
                s_bby <= s_bby + s_y17 - unsigned(rb_f(31 downto 22));
                s_bbu <= s_bbu + s_u17 - unsigned(rb_f(21 downto 12));
                s_bbv <= s_bbv + s_v17 - unsigned(rb_f(11 downto 2));
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

end architecture carpet;
