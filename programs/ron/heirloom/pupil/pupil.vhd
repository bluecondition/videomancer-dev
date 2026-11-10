-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Pupil v0.1.0 -- Heirloom (video phase), rebuilt from the clip WIN_20230408_23_11_38 (still at 3.5 s).
--
-- The clip: a column of three almond "eyes" on flat green.  The left half of each eye is concentric cream /
-- pale-green / teal ring arcs cut into brick-like dashes (a moire), with a soft dark-green halo outside; the
-- middle is a blue core with a violet ball, a navy rim and a dark crescent, keyed only on the rows where the eye
-- is deep enough at its centre (flat top and bottom).  The eye snaps open on every beat of a rhythm (attack
-- ~1 field, sweeping down the frame) and closes in ~0.25 s.
--
-- The patch, as fitted (pupil.py has the palettes):
--   e     = R(y) + z(y) - Kv t(y)^2 - |x - cx|        eye depth: H triangle + V parabola-tile (squared triangle)
--           t = frac((y - 521)/522 + 0.5) - 0.5, Kv = 1730 px, cx = 1077
--   R     = dilation (P12) + the beat envelope (per line) ; z = the moire: bands of 10 virtual lines (period 20)
--           where the eye is dilated by 16..31 px
--   core  = rows where R + z - Kv t^2 >= 362 (scaled by Rings): the blue core
--   colour= mix(PAL_R(core, e), PAL_L(e), A), A = clamp((K_line - x) / 128 px): rings on the left, fading right
--   video = camera luma (or the Osc lattice) added to e: the rings and the core bend around the picture
--
--   K1  Eyes      eyes per frame height: 0.73 (one giant eye) .. 2.07 (the clip) .. 5.8
--   K2  Squint    the eye's vertical curvature: merged wavy column .. the clip .. flat slits
--   K3  Rings     ring scale: fat rings filling the eye .. the clip .. fine rings round a big core
--   K4  Beat      the rhythm's step: 0.8 s .. 0.2 s (the clip) .. 0.05 s (Ron's 32-step hit pattern)
--   K5  Pulse     how wide each beat throws the eye open: 0 .. 440 px (the clip) .. 880 px
--   K6  Bend      how far the camera (or the Osc) bends the eye
--   S7  Slip      Off / sync slips: whole bands of the picture thrown sideways
--   S8  Core      Blue (the clip) / Rings (no core: ring moire through the whole eye, the clip's rest look)
--   S9  Twin      one column / two mirrored columns gazing at each other
--   S10 Colours   Green (the clip) / Magenta (complementary chroma)
--   S11 Source    Video / Osc
--   P12 DILATE    the pupil's opening: closed seeds .. the clip (343) .. the eye swallowing the screen
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, exp2 ROM), a per-line engine kicked at each
-- line start (computes the NEXT line: phase, t^2, Kv t^2, the beat and its envelope, the moire band, the slip
-- band, the core test), a 15-stage pixel path (two palette ROMs, one blend), C_LAT 20.  Virtual 1080-line
-- coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (Y, U and V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes with the running-sum resync, luma 13/16) before the blanking gate.  Colours in
-- the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.pupil_pkg.all;
use work.all;

architecture pupil of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- Ron's hit pattern from the clip: steps 0, 1, 5, 8, 14, 16, 17, 21, 24 of a 32-step loop
    constant C_PAT : std_logic_vector(31 downto 0) := x"01234123";

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

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_l_a, r_r_a : unsigned(8 downto 0) := (others => '0');
    signal r_l_q, r_r_q : std_logic_vector(31 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 160 x 135 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(9 downto 0) := to_unsigned(408, 10);
    signal s_oys : unsigned(11 downto 0) := to_unsigned(484, 12);
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

    -- knobs: glided (Q3), raw beat, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(354 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(343 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_slip, s_coreoff, s_twin, s_pal, s_src : std_logic := '0';

    -- per-field constants
    signal z_v8   : signed(10 downto 0) := (others => '0');   -- exponent scratch (Q8 octaves)
    signal z_sh1, z_sh2, z_sh4, z_shi : signed(2 downto 0) := (others => '0');
    signal z_m1, z_m2, z_m4, z_mi : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal c_sh3  : signed(2 downto 0) := (others => '0');
    signal c_m6   : unsigned(5 downto 0) := to_unsigned(32, 6);  -- ring scale mantissa (32..63)
    signal c_cth  : unsigned(16 downto 0) := to_unsigned(23168, 17);
    signal c_bias : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal c_fvl  : unsigned(12 downto 0) := to_unsigned(2009, 13);   -- eye phase per virtual line (Q20)
    signal c_fs   : unsigned(15 downto 0) := to_unsigned(2009, 16);   -- per line
    signal c_ph0  : unsigned(19 downto 0) := (others => '0');
    signal c_kv   : unsigned(13 downto 0) := to_unsigned(3460, 14);   -- Kv in h
    signal c_g6   : unsigned(7 downto 0) := (others => '0');
    signal c_g6e  : unsigned(9 downto 0) := (others => '0');          -- bend gain (Osc x2)
    signal c_lb8  : unsigned(23 downto 0) := to_unsigned(12960 * 256, 24);   -- beat step, Q8 virtual lines
    signal c_rb   : unsigned(10 downto 0) := to_unsigned(476, 11);    -- dilation, px
    signal c_dep  : unsigned(21 downto 0) := to_unsigned(512 * 3523, 22);  -- pulse depth, Q12 px
    signal c_dcap : unsigned(10 downto 0) := to_unsigned(220, 11);
    signal c_atk  : unsigned(12 downto 0) := to_unsigned(1728, 13);
    signal c_dcon : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal c_cxh  : unsigned(11 downto 0) := to_unsigned(2154, 12);
    signal c_zph0 : unsigned(12 downto 0) := (others => '0');          -- moire phase at line 0 (Q8 lines)
    signal c_zn0  : unsigned(15 downto 0) := (others => '0');
    signal c_fcnt : unsigned(2 downto 0) := (others => '0');
    signal c_seed : unsigned(7 downto 0) := (others => '0');
    signal c_sd77 : unsigned(15 downto 0) := (others => '0');
    signal z_t    : unsigned(15 downto 0) := (others => '0');

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
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
    signal e_zk, e_first : std_logic := '0';
    signal e_ph   : unsigned(19 downto 0) := (others => '0');
    signal e_zacc : unsigned(13 downto 0) := (others => '0');
    signal e_zn   : unsigned(15 downto 0) := (others => '0');
    signal e_sacc : unsigned(16 downto 0) := (others => '0');
    signal e_sn   : unsigned(15 downto 0) := (others => '0');
    signal e_slen : unsigned(16 downto 0) := to_unsigned(32 * 256, 17);
    signal e_bacc : unsigned(24 downto 0) := (others => '0');
    signal e_bstep : unsigned(4 downto 0) := (others => '1');
    signal e_hit, e_atk : std_logic := '0';
    signal e_dr   : unsigned(22 downto 0) := (others => '0');
    signal e_dec  : unsigned(19 downto 0) := (others => '0');
    signal e_tq   : unsigned(15 downto 0) := (others => '0');
    signal e_t2q  : unsigned(15 downto 0) := (others => '0');
    signal e_vh   : unsigned(12 downto 0) := (others => '0');
    signal e_hz, e_hs : unsigned(15 downto 0) := (others => '0');
    signal e_z    : unsigned(5 downto 0) := (others => '0');
    signal e_drop, e_core : std_logic := '0';
    signal e_rh1, e_rh : unsigned(11 downto 0) := (others => '0');
    signal e_evh, e_ev0 : signed(14 downto 0) := (others => '0');
    signal e_xo   : signed(10 downto 0) := (others => '0');
    signal e_x0   : signed(13 downto 0) := (others => '0');
    signal m_a    : unsigned(39 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(39 downto 0) := (others => '0');
    signal n_evb, n_k : signed(14 downto 0) := (others => '0');
    signal n_core, n_cr, n_dn : std_logic := '0';
    signal n_x0   : signed(13 downto 0) := (others => '0');
    signal n_am   : unsigned(12 downto 0) := (others => '0');

    -- beam (stage 1) and line registers
    signal b_x    : signed(13 downto 0) := (others => '0');
    signal b_am   : unsigned(12 downto 0) := (others => '0');
    signal b_dn   : std_logic := '0';
    signal b_px   : unsigned(10 downto 0) := (others => '0');
    signal l_evb, l_k : signed(14 downto 0) := (others => '0');
    signal l_core, l_cr : std_logic := '0';

    -- pixel path
    signal p2_xf  : signed(13 downto 0) := (others => '0');
    signal p3_d, p3_a : signed(14 downto 0) := (others => '0');
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p4_ad  : unsigned(13 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(13 downto 0) := (others => '0');
    signal p5_e0  : signed(15 downto 0) := (others => '0');
    signal p5_bd  : unsigned(17 downto 0) := (others => '0');
    signal p6_e   : unsigned(12 downto 0) := (others => '0');
    signal p7_ph, p7_pl : unsigned(15 downto 0) := (others => '0');
    signal p8_s   : unsigned(18 downto 0) := (others => '0');
    type t_a5 is array (natural range <>) of unsigned(4 downto 0);
    signal p_a    : t_a5(4 to 12) := (others => (others => '0'));   -- A16 from stage 4 to stage 12
    signal p_cr   : std_logic_vector(4 to 8) := (others => '0');     -- core-row R region, stages 4..8
    signal p12_dy, p12_du, p12_dv : signed(10 downto 0) := (others => '0');
    signal p12_ry, p12_ru, p12_rv : unsigned(9 downto 0) := (others => '0');
    signal p12_a  : unsigned(4 downto 0) := (others => '0');
    signal p11_l, p11_r : std_logic_vector(31 downto 0) := (others => '0');
    signal p13_hy, p13_hu, p13_hv : signed(13 downto 0) := (others => '0');
    signal p13_ly, p13_lu, p13_lv : signed(12 downto 0) := (others => '0');
    signal p13_ry, p13_ru, p13_rv : unsigned(9 downto 0) := (others => '0');
    signal p14_y, p14_u, p14_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    constant C_KY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_KC : unsigned(9 downto 0) := to_unsigned(512, 10);
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
            r_ex_q <= C_PU_EXP2(to_integer(r_ex_a));
            r_l_q  <= C_PU_L(to_integer(r_l_a));
            r_r_q  <= C_PU_R(to_integer(r_r_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_e  : signed(10 downto 0);
        variable v_t  : unsigned(35 downto 0);
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
                s_slip    <= registers_in(6)(0);
                s_coreoff <= registers_in(6)(1);
                s_twin    <= registers_in(6)(2);
                s_pal     <= registers_in(6)(3);
                s_src     <= registers_in(6)(4);
                s_k4      <= unsigned(registers_in(3));
                s_xs      <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(816, 10);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(612, 10);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(408, 10);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 21) * to_unsigned(484, 9), 8), 12);

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

            -- serial products: slot k loads at zc = 200 + 24 k, 16 iterations, result stable from
            -- load + 17 (stored at load + 18).  The slot number is decoded at 198 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 199 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 198 => z_sel <= to_unsigned(0, 4); z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(1, 4); z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(2, 4); z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(3, 4); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(4, 4); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(5, 4); z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(6, 4); z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(7, 4); z_pre <= '1';
                    when 390 => z_sel <= to_unsigned(8, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(2009, 36);            z_fb <= z_m1;                       -- eye phase / vline
                    when 1  => z_fa <= resize(c_fvl, 36);                z_fb <= resize(s_ys, 16);           -- per line
                    when 2  => z_fa <= resize(c_fvl, 36);                z_fb <= to_unsigned(521, 16);       -- phase at line 0
                    when 3  => z_fa <= to_unsigned(3460, 36);            z_fb <= z_m2;                       -- Kv
                    when 4  => z_fa <= resize(s_g6(12 downto 3), 36);    z_fb <= resize(s_g6(12 downto 3), 16);  -- bend^2
                    when 5  => z_fa <= to_unsigned(12960, 36);           z_fb <= z_m4;                       -- beat step
                    when 6  =>                                                                               -- dilation
                        if s_gp(12 downto 3) < 343 then
                            z_fa <= resize(s_gp(12 downto 3), 36);       z_fb <= to_unsigned(973, 16);
                        else
                            z_fa <= resize(s_gp(12 downto 3) - 343, 36); z_fb <= to_unsigned(1250, 16);
                        end if;
                    when 7  => z_fa <= resize(s_g5(12 downto 3), 36);    z_fb <= to_unsigned(3523, 16);      -- pulse depth
                    when others => z_fa <= resize(c_dep(21 downto 12), 36); z_fb <= resize(s_ys, 16);       -- decay cap
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
                    -- switches' constants, motion of the moire and the slip seed
                    when 20 =>
                        if s_twin = '1' then c_cxh <= to_unsigned(1200, 12); else c_cxh <= to_unsigned(2154, 12); end if;
                        if c_zph0 < 171 then
                            c_zph0 <= c_zph0 + 4949; c_zn0 <= c_zn0 - 1;
                        else
                            c_zph0 <= c_zph0 - 171;
                        end if;
                        c_fcnt <= c_fcnt + 1;
                        if c_fcnt = 7 then c_seed <= c_seed + 1; end if;
                    when 21 =>
                        c_sd77 <= shift_left(resize(c_seed, 16), 6) + shift_left(resize(c_seed, 16), 3);
                        z_t <= shift_left(resize(s_ys, 16), 5) - shift_left(resize(s_ys, 16), 2);
                    when 22 =>
                        c_sd77 <= c_sd77 + shift_left(resize(c_seed, 16), 2) + resize(c_seed, 16);
                        z_t <= z_t - resize(s_ys, 16);
                    when 23 =>
                        c_atk <= resize(shift_right(z_t, 2), 13);                 -- ys * 27 / 4 (Q12 px per line)
                        c_dcon <= resize(shift_right(s_ys, 5), 7);

                    -- exponents: K1 eyes (a = 0.75 (k - 512)), K2 squint (b = k - 512), K3 rings
                    -- (c = 0.75 (k - 512)) and its inverse, K4 beat (512 - k); 256 = one octave
                    when 30 =>
                        v_e := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        z_v8 <= v_e + shift_right(v_e, 1);                        -- 1.5 (k - 512)
                    when 31 =>
                        z_v8 <= shift_right(z_v8, 1);                             -- 0.75 (k - 512)
                    when 32 =>
                        r_ex_a <= unsigned(z_v8(7 downto 0)); z_sh1 <= z_v8(10 downto 8);
                        z_v8 <= signed(resize(s_g2(12 downto 3), 11)) - 512;
                    when 33 =>
                        r_ex_a <= unsigned(z_v8(7 downto 0)); z_sh2 <= z_v8(10 downto 8);
                        v_e := signed(resize(s_g3(12 downto 3), 11)) - 512;
                        z_v8 <= v_e + shift_right(v_e, 1);
                    when 34 =>
                        z_m1 <= r_ex_q;
                        z_v8 <= shift_right(z_v8, 1);
                    when 35 =>
                        z_m2 <= r_ex_q;
                        r_ex_a <= unsigned(z_v8(7 downto 0)); c_sh3 <= z_v8(10 downto 8);
                        z_v8 <= -z_v8;
                    when 36 =>
                        r_ex_a <= unsigned(z_v8(7 downto 0)); z_shi <= z_v8(10 downto 8);
                        z_v8 <= 512 - signed(resize(s_k4, 11));
                    when 37 =>
                        r_ex_a <= unsigned(z_v8(7 downto 0)); z_sh4 <= z_v8(10 downto 8);
                        c_m6 <= r_ex_q(15 downto 10);
                        -- core threshold: e * m6 >= 23168 >> sh3 (362 px of scaled depth)
                        case to_integer(c_sh3) is
                            when -2     => c_cth <= to_unsigned(92672, 17);
                            when -1     => c_cth <= to_unsigned(46336, 17);
                            when 0      => c_cth <= to_unsigned(23168, 17);
                            when others => c_cth <= to_unsigned(11584, 17);
                        end case;
                    when 38 =>
                        z_mi <= r_ex_q;
                    when 39 =>
                        z_m4 <= r_ex_q;
                    when 40 =>                       -- bias = 256 / s (h): mi >> (7 - shi)
                        case to_integer(z_shi) is
                            when -2     => c_bias <= resize(shift_right(z_mi, 9), 11);
                            when -1     => c_bias <= resize(shift_right(z_mi, 8), 11);
                            when 0      => c_bias <= resize(shift_right(z_mi, 7), 11);
                            when others => c_bias <= resize(shift_right(z_mi, 6), 11);
                        end case;

                    -- products (slot k stored at 200 + 24 k + 18)
                    when 218 =>                      -- fvl = 2009 m1 >> (15 - sh1)
                        case to_integer(z_sh1) is
                            when -2     => c_fvl <= resize(shift_right(f_acc, 17), 13);
                            when -1     => c_fvl <= resize(shift_right(f_acc, 16), 13);
                            when 0      => c_fvl <= resize(shift_right(f_acc, 15), 13);
                            when others => c_fvl <= resize(shift_right(f_acc, 14), 13);
                        end case;
                    when 242 => c_fs <= resize(shift_right(f_acc, 8), 16);
                    when 266 => c_ph0 <= to_unsigned(524288, 20) - f_acc(19 downto 0);
                    when 290 =>                      -- Kv = 3460 m2 >> (15 - sh2)
                        case to_integer(z_sh2) is
                            when -2     => c_kv <= resize(shift_right(f_acc, 17), 14);
                            when -1     => c_kv <= resize(shift_right(f_acc, 16), 14);
                            when 0      => c_kv <= resize(shift_right(f_acc, 15), 14);
                            when others => c_kv <= resize(shift_right(f_acc, 14), 14);
                        end case;
                    when 314 => c_g6 <= f_acc(19 downto 12);
                    when 315 =>
                        if s_src = '1' then c_g6e <= shift_left(resize(c_g6, 10), 1);              -- Osc: x2
                        else c_g6e <= resize(c_g6, 10); end if;
                    when 338 =>                      -- beat step = 12960 m4 >> (15 - sh4) virtual lines, Q8
                        case to_integer(z_sh4) is
                            when -2     => v_t := shift_right(f_acc, 17);
                            when -1     => v_t := shift_right(f_acc, 16);
                            when 0      => v_t := shift_right(f_acc, 15);
                            when 1      => v_t := shift_right(f_acc, 14);
                            when others => v_t := shift_right(f_acc, 13);
                        end case;
                        c_lb8 <= v_t(15 downto 0) & x"00";
                    when 362 =>
                        if s_gp(12 downto 3) < 343 then c_rb <= resize(shift_right(f_acc, 10), 11) + 150;
                        else c_rb <= resize(shift_right(f_acc, 10), 11) + 476; end if;
                    when 386 => c_dep <= f_acc(21 downto 0);
                    when 410 => c_dcap <= resize(shift_right(f_acc, 9), 11);
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
    -- 0), computes the NEXT line on one serial multiplier (16 iterations,
    -- result at load + 17): the eye phase t, Kv t^2, the beat, the envelope,
    -- the moire band, the slip band, the core test.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d  : signed(20 downto 0);
        variable v_u  : unsigned(16 downto 0);
        variable v_h  : unsigned(15 downto 0);
        variable v_s  : signed(15 downto 0);
        variable v_m  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_lc <= (others => '0'); e_first <= '1';
            elsif g_start = '1' then
                e_lc <= (others => '0'); e_first <= '0';
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(38 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    if e_first = '1' then
                        e_ph <= c_ph0; e_zacc <= resize(c_zph0, 14); e_zn <= c_zn0;
                        e_sacc <= (others => '0'); e_sn <= (others => '0');
                    else
                        e_ph <= e_ph + c_fs;
                        e_zacc <= e_zacc + resize(s_ys, 14);
                        e_sacc <= e_sacc + resize(s_ys, 17);
                    end if;
                    e_bacc <= e_bacc + resize(s_ys, 25);
                when 2 =>
                    v_d := signed(resize(e_ph, 21)) - 524288;
                    if v_d < 0 then v_d := -v_d; end if;
                    e_tq <= unsigned(v_d(19 downto 4));
                    if e_bacc >= resize(c_lb8, 25) then
                        e_bacc <= e_bacc - resize(c_lb8, 25); e_bstep <= e_bstep + 1; e_hit <= '1';
                    end if;
                    if e_zacc >= 5120 then e_zacc <= e_zacc - 5120; e_zn <= e_zn + 1; end if;
                    if e_first = '0' and e_sacc >= e_slen then e_sacc <= e_sacc - e_slen; e_sn <= e_sn + 1; end if;
                when 3 =>
                    m_a <= resize(e_tq, 40); m_b <= e_tq; m_acc <= (others => '0');
                    if e_hit = '1' and C_PAT(to_integer(e_bstep)) = '1' then e_atk <= '1'; end if;
                    e_hit <= '0';
                when 20 =>
                    e_t2q <= m_acc(31 downto 16);
                    m_a <= resize(c_kv, 40); m_b <= m_acc(31 downto 16); m_acc <= (others => '0');
                when 37 =>
                    e_vh <= m_acc(28 downto 16);
                    m_a <= resize(e_dr, 40); m_b <= resize(s_ys, 16); m_acc <= (others => '0');
                when 54 =>
                    e_dec <= resize(m_acc(39 downto 20), 20) + resize(c_dcon, 20);
                when 55 =>
                    if e_dec > resize(c_dcap, 20) then e_dec <= resize(c_dcap, 20); end if;
                when 56 =>
                    if e_atk = '1' then
                        if resize(e_dr, 24) + resize(c_atk, 24) >= resize(c_dep, 24) then
                            e_dr <= resize(c_dep, 23); e_atk <= '0';
                        else
                            e_dr <= e_dr + resize(c_atk, 23);
                        end if;
                    else
                        if e_dr > resize(e_dec, 23) then e_dr <= e_dr - resize(e_dec, 23);
                        else e_dr <= (others => '0'); end if;
                    end if;
                    m_a <= resize(e_zn, 40); m_b <= to_unsigned(40503, 16); m_acc <= (others => '0');
                when 73 =>
                    v_h := m_acc(15 downto 0);
                    e_hz <= v_h xor shift_right(v_h, 8);
                    m_a <= resize(e_sn + c_sd77, 40); m_b <= to_unsigned(40503, 16); m_acc <= (others => '0');
                when 74 =>
                    if e_zacc < 2560 then e_z <= resize(e_hz(3 downto 0), 6) + 16; else e_z <= (others => '0'); end if;
                    if e_zacc < 1280 and e_hz(6 downto 4) = "000" then e_drop <= '1'; else e_drop <= '0'; end if;
                    e_rh1 <= resize(c_rb, 12) + resize(e_dr(22 downto 12), 12);
                when 75 =>
                    e_rh <= e_rh1 + resize(e_z, 12);
                when 76 =>
                    e_evh <= signed(resize(e_rh & '0', 15)) - signed(resize(e_vh, 15));
                    e_ev0 <= signed(resize(e_rh1 & '0', 15)) - signed(resize(e_vh, 15));   -- the core test ignores the moire
                when 90 =>
                    v_h := m_acc(15 downto 0);
                    v_h := v_h xor shift_right(v_h, 8);
                    e_hs <= v_h;
                    if e_ev0 > 0 then m_a <= resize(unsigned(e_ev0), 40); else m_a <= (others => '0'); end if;
                    m_b <= resize(c_m6, 16); m_acc <= (others => '0');
                when 91 =>
                    v_u := resize(shift_left(resize(e_hs(3 downto 0), 17), 3) + 32, 17);
                    e_slen <= v_u(8 downto 0) & x"00";
                    v_m := shift_left(resize(e_hs(10 downto 5), 10), 3) + shift_left(resize(e_hs(10 downto 5), 10), 1);
                    if s_slip = '0' or e_hs(4) = '1' then e_xo <= (others => '0');
                    elsif e_hs(11) = '1' then e_xo <= -signed(resize(v_m, 11));
                    else e_xo <= signed(resize(v_m, 11)); end if;
                when 92 =>
                    e_x0 <= signed(resize(s_x0, 14)) + resize(e_xo, 14);
                when 93 =>
                    n_x0 <= e_x0;
                    v_s := resize(e_x0, 16) - 1920;
                    if v_s < 0 then n_am <= unsigned(resize(-v_s, 13)); n_dn <= '1';
                    else n_am <= unsigned(resize(v_s, 13)); n_dn <= '0'; end if;
                when 107 =>
                    if e_ev0 > 0 and s_coreoff = '0' and m_acc(19 downto 0) >= resize(c_cth, 20) then e_core <= '1';
                    else e_core <= '0'; end if;
                when 108 =>
                    n_evb <= e_evh + signed(resize(c_bias, 15));
                    n_core <= e_core;
                    n_cr <= e_core and not e_drop;
                    if e_core = '1' then n_k <= signed(resize(c_cxh, 15)) + 120;
                    else n_k <= e_evh + signed(resize(c_cxh, 15)) - 160; end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: x in h units (unfolded, starting at the line's slip offset) and
    -- |x - 1920| as a reflecting DDA for the Twin fold.  Line registers load
    -- at the line start.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= n_x0;
                b_am <= n_am;
                b_dn <= n_dn;
                b_px <= (others => '0');
                l_evb <= n_evb; l_k <= n_k; l_core <= n_core; l_cr <= n_cr;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x + signed(resize(s_xs, 14));
                if b_dn = '1' then
                    if b_am > s_xs then b_am <= b_am - s_xs;
                    else b_am <= resize(s_xs, 13) - b_am; b_dn <= '0'; end if;
                else
                    b_am <= b_am + s_xs;
                end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_e  : signed(16 downto 0);
        variable v_v  : unsigned(12 downto 0);
        variable v_w  : unsigned(11 downto 0);
        variable v_a  : signed(14 downto 0);
        variable v_t  : signed(16 downto 0);
        variable v_o  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: folded x (Twin: 1920 - |x - 1920|)
            if s_twin = '1' then p2_xf <= 1920 - signed(resize(b_am, 14)); else p2_xf <= b_x; end if;
            -- 3: distance to the eye centre, the fade ramp, the luma
            p3_d  <= resize(p2_xf, 15) - signed(resize(c_cxh, 15));
            p3_a  <= l_k - resize(p2_xf, 15);
            p3_lm <= unsigned(s_lb_q);
            -- 4: |d|, A16 = clamp((K - x) / 16, 0, 16), bend partials (luma nibbles x gain)
            if p3_d < 0 then p4_ad <= unsigned(resize(-p3_d, 14)); else p4_ad <= unsigned(resize(p3_d, 14)); end if;
            v_a := shift_right(p3_a, 4);
            if v_a < 0 then p_a(4) <= (others => '0');
            elsif v_a > 16 then p_a(4) <= to_unsigned(16, 5);
            else p_a(4) <= unsigned(v_a(4 downto 0)); end if;
            p4_ea <= resize(p3_lm(3 downto 0) * c_g6e, 14);
            p4_eb <= resize(p3_lm(7 downto 4) * c_g6e, 14);
            p_cr(4) <= l_cr;
            -- 5: biased depth (line term - |d|), bend
            p5_e0 <= resize(l_evb, 16) - signed(resize(p4_ad, 16));
            p5_bd <= resize(p4_ea, 18) + shift_left(resize(p4_eb, 18), 4);
            -- 6: e = depth + bend / 64, clamped 0 .. 8191
            v_e := resize(p5_e0, 17) + signed(resize(shift_right(p5_bd, 6), 17));
            if v_e < 0 then p6_e <= (others => '0');
            elsif v_e > 8191 then p6_e <= (others => '1');
            else p6_e <= unsigned(v_e(12 downto 0)); end if;
            -- 7: ring scale partials (e x m6, split 3 + 3 bits)
            p7_ph <= resize(p6_e * c_m6(5 downto 3), 16);
            p7_pl <= resize(p6_e * c_m6(2 downto 0), 16);
            -- 8: e * m6
            p8_s <= shift_left(resize(p7_ph, 19), 3) + resize(p7_pl, 19);
            -- 9: index = e m6 >> (7 - sh3); palette addresses (Rings wrap; the deep pupil repeats)
            case to_integer(c_sh3) is
                when -2     => v_v := resize(shift_right(p8_s, 9), 13);
                when -1     => v_v := resize(shift_right(p8_s, 8), 13);
                when 0      => v_v := resize(shift_right(p8_s, 7), 13);
                when others => v_v := resize(shift_right(p8_s, 6), 13);
            end case;
            if s_coreoff = '1' then
                if v_v < 64 then r_l_a <= v_v(8 downto 0);
                else r_l_a <= "0" & (not v_v(6)) & v_v(6) & v_v(5 downto 0); end if;
            else
                if v_v < 448 then r_l_a <= v_v(8 downto 0);
                else r_l_a <= "11" & v_v(6 downto 0); end if;
            end if;
            if v_v(12 downto 9) /= "0000" then v_w := (others => '1'); else v_w := resize(v_v(8 downto 1), 12); end if;
            r_r_a <= p_cr(8) & v_w(7 downto 0);
            -- 10: ROMs (EBR), 11: fabric copy
            p11_l <= r_l_q; p11_r <= r_r_q;
            -- 12: L - R per channel
            p12_dy <= signed(resize(unsigned(p11_l(31 downto 22)), 11)) - signed(resize(unsigned(p11_r(31 downto 22)), 11));
            p12_du <= signed(resize(unsigned(p11_l(21 downto 12)), 11)) - signed(resize(unsigned(p11_r(21 downto 12)), 11));
            p12_dv <= signed(resize(unsigned(p11_l(11 downto 2)), 11)) - signed(resize(unsigned(p11_r(11 downto 2)), 11));
            p12_ry <= unsigned(p11_r(31 downto 22)); p12_ru <= unsigned(p11_r(21 downto 12)); p12_rv <= unsigned(p11_r(11 downto 2));
            p12_a  <= p_a(11);
            -- 13: (L - R) x A16 partials: A(4:2) (0..4) and A(1:0) (0..3)
            case to_integer(p12_a(4 downto 2)) is
                when 0      => p13_hy <= (others => '0'); p13_hu <= (others => '0'); p13_hv <= (others => '0');
                when 1      => p13_hy <= resize(p12_dy, 14); p13_hu <= resize(p12_du, 14); p13_hv <= resize(p12_dv, 14);
                when 2      => p13_hy <= shift_left(resize(p12_dy, 14), 1); p13_hu <= shift_left(resize(p12_du, 14), 1);
                               p13_hv <= shift_left(resize(p12_dv, 14), 1);
                when 3      => p13_hy <= shift_left(resize(p12_dy, 14), 1) + resize(p12_dy, 14);
                               p13_hu <= shift_left(resize(p12_du, 14), 1) + resize(p12_du, 14);
                               p13_hv <= shift_left(resize(p12_dv, 14), 1) + resize(p12_dv, 14);
                when others => p13_hy <= shift_left(resize(p12_dy, 14), 2); p13_hu <= shift_left(resize(p12_du, 14), 2);
                               p13_hv <= shift_left(resize(p12_dv, 14), 2);
            end case;
            case to_integer(p12_a(1 downto 0)) is
                when 0      => p13_ly <= (others => '0'); p13_lu <= (others => '0'); p13_lv <= (others => '0');
                when 1      => p13_ly <= resize(p12_dy, 13); p13_lu <= resize(p12_du, 13); p13_lv <= resize(p12_dv, 13);
                when 2      => p13_ly <= shift_left(resize(p12_dy, 13), 1); p13_lu <= shift_left(resize(p12_du, 13), 1);
                               p13_lv <= shift_left(resize(p12_dv, 13), 1);
                when others => p13_ly <= shift_left(resize(p12_dy, 13), 1) + resize(p12_dy, 13);
                               p13_lu <= shift_left(resize(p12_du, 13), 1) + resize(p12_du, 13);
                               p13_lv <= shift_left(resize(p12_dv, 13), 1) + resize(p12_dv, 13);
            end case;
            p13_ry <= p12_ry; p13_ru <= p12_ru; p13_rv <= p12_rv;
            -- 14: out = R + ((hi << 2) + lo) / 16
            v_t := shift_right(shift_left(resize(p13_hy, 17), 2) + resize(p13_ly, 17), 4);
            v_o := signed(resize(p13_ry, 12)) + resize(v_t, 12);
            p14_y <= unsigned(v_o(9 downto 0));
            v_t := shift_right(shift_left(resize(p13_hu, 17), 2) + resize(p13_lu, 17), 4);
            v_o := signed(resize(p13_ru, 12)) + resize(v_t, 12);
            p14_u <= unsigned(v_o(9 downto 0));
            v_t := shift_right(shift_left(resize(p13_hv, 17), 2) + resize(p13_lv, 17), 4);
            v_o := signed(resize(p13_rv, 12)) + resize(v_t, 12);
            p14_v <= unsigned(v_o(9 downto 0));
            -- 15: compose: Colours = complementary chroma; black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= C_KY; s_u15 <= C_KC; s_v15 <= C_KC;
            elsif s_pal = '1' then
                s_y15 <= p14_y;
                if p14_u = 0 then s_u15 <= (others => '1'); else s_u15 <= (not p14_u) + 1; end if;
                if p14_v = 0 then s_v15 <= (others => '1'); else s_v15 <= (not p14_v) + 1; end if;
            else
                s_y15 <= p14_y; s_u15 <= p14_u; s_v15 <= p14_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
            -- flag pipes
            for i in 5 to 12 loop p_a(i) <= p_a(i - 1); end loop;
            for i in 5 to 8 loop p_cr(i) <= p_cr(i - 1); end loop;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the clip's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to black (the off-avid input of both boxes) once 8 blank
            -- samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 16 sums, stage 17 output
            s_cay(0) <= s_y15; s_cau(0) <= s_u15; s_cav(0) <= s_v15;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KC & "000"; s_bav <= C_KC & "000";
            else
                s_bay <= s_bay + s_y15 - s_cay(7);
                s_bau <= s_bau + s_u15 - s_cau(7);
                s_bav <= s_bav + s_v15 - s_cav(7);
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= C_KY; s_u17 <= C_KC; s_v17 <= C_KC;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            -- box B: stage 18 sums, stage 19 output
            s_cby(0) <= s_y17; s_cbu(0) <= s_u17; s_cbv(0) <= s_v17;
            for i in 1 to 7 loop s_cby(i) <= s_cby(i - 1); s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= C_KC & "000"; s_bbv <= C_KC & "000";
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

end architecture pupil;
