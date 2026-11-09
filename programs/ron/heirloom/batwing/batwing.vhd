-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Batwing v0.1.0 -- Heirloom (video phase), from the clip WIN_20230927_23_58_07 at 2.5 s.
--
-- The still: three mirrored tiles of a magenta / yellow / violet "bat wing": in each tile a lilac bowtie
-- (camera-textured, ragged edges) inside nested contour bands -- magenta, a violet ramp, dark green, violet,
-- yellow / dark-green pairs, compressing toward the bowtie -- with magenta columns at the seams that pinch into a
-- dark-green waist, and a magenta dome under every bowtie.  In the clip the vertical oscillator rolls the bands
-- down 160 lines/s (one period of 1117 lines per 7 s); the seams and the bowtie's sides stay put.
--
-- The patch, as fitted (batwing.py has every constant):
--   xp  = mirrored horizontal triangle: distance from the seam, 0 .. 511 (the tile centre)
--   sA  = TA(xp + waist(y) (1 - xp/184)) - 64      the wing sides: a sqrt-like ramp off the seam, bent by a
--                                                   static frame-locked hump (the waist)
--   sB  = C0 - C1 Dv + ws (KE (511 - xp) + KM Dv (511 - xpb))    the bowtie's top / bottom: the rolling vertical
--                                                   oscillator's waveform Dv(phase) times the horizontal ramp
--   s   = min(sA, sB) (chamfered) + camera -> palette (the photo's band sequence)
--   key = xp > 202 and Dv - 155 < KW (511 - xp): the lilac bowtie (two per-line limits), frayed per 8-line strip
--
--   K1  Tiles     tiles across (x0.35 .. x2.8; 2.02 = the photo)
--   K2  Rows      the vertical oscillator's frequency (x0.35 .. x2.8; 1117 lines = the photo)
--   K3  Roll      bipolar roll speed (+ = down; 772 = the clip's 160 lines/s)
--   K4  Bands     band density (x0.33 .. x3)
--   K5  Waist     the static hump that pinches the seams (0 .. 2x)
--   K6  Etch      how far the camera (or the Osc) bends the bands and frays the bowtie
--   S7  Mirror    On (the photo) / Off (sawtooth: one-sided wings, hard seams)
--   S8  Zigzag    Off / On (no wing sides: chevron zigzag bands and a zigzag lilac strip, the alt clip)
--   S9  Colours   Magenta (the photo) / Meadow (the green / yellow alt clip)
--   S10 Invert    Off / On (lilac fills the low regions, rings fill the bowtie)
--   S11 Source    Video / Osc
--   P12 WINGSPAN  the horizontal coupling: folded flat (0: horizontal bands) .. the photo (384) .. 3x chevrons
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line start
-- (computes the NEXT line: the waveform Dv from a table at the oscillator's phase, the B line terms, the waist, the
-- fray), a horizontal phase accumulator, a 16-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre
-- 4:3).  Generated avid; output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two 8-boxes with the resync, luma
-- [1 2 1]/4, luma 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.batwing_pkg.all;
use work.all;

architecture batwing of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant C_PHC : integer := 15561206;   -- tile phase at the screen centre: (960 - 1029) / 952 tile, Q24 mod 1
    constant C_R0  : integer := 1764790;    -- the oscillator phase of line 0 at power-up (the still: peak row 441)

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
    -- x step in half virtual px per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc lattice)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ta_a : unsigned(8 downto 0) := (others => '0');
    signal r_ta_q : unsigned(8 downto 0) := (others => '0');
    signal r_dv_a : unsigned(9 downto 0) := (others => '0');
    signal r_dv_q : unsigned(9 downto 0) := (others => '0');
    signal r_rn_a : unsigned(8 downto 0) := (others => '0');
    signal r_rn_q : unsigned(7 downto 0) := (others => '0');
    signal r_pa_a : unsigned(9 downto 0) := (others => '0');
    signal r_pa_q : std_logic_vector(23 downto 0) := (others => '0');
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');

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
    signal s_fc   : unsigned(10 downto 0) := (others => '0');

    -- knobs: glided (Q3), raw roll speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(120 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(384 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_k3 : unsigned(9 downto 0) := to_unsigned(772, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg, s_vzero : std_logic := '0';
    signal s_mir, s_zig, s_col, s_inv, s_src : std_logic := '0';

    -- per-field constants
    signal z_sh1, z_sh2, z_sh4 : signed(2 downto 0) := (others => '0');
    signal z_mt1, z_mt2 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_e    : signed(10 downto 0) := (others => '0');
    signal c_hsh  : unsigned(16 downto 0) := to_unsigned(8812, 17);     -- tile phase per half virtual px, Q24
    signal c_hpx  : unsigned(23 downto 0) := to_unsigned(17624, 24);    -- per pixel
    signal c_hp0  : unsigned(23 downto 0) := (others => '0');           -- at the line start
    signal c_kv   : unsigned(16 downto 0) := to_unsigned(15020, 17);    -- oscillator phase per virtual line, Q24
    signal c_kvl  : unsigned(23 downto 0) := to_unsigned(15020, 24);    -- per line
    signal c_vsq  : unsigned(17 downto 0) := (others => '0');
    signal c_spd  : unsigned(17 downto 0) := (others => '0');           -- roll per field, Q24
    signal c_R    : unsigned(23 downto 0) := to_unsigned(C_R0, 24);     -- phase of line 0
    signal c_G    : unsigned(8 downto 0) := to_unsigned(128, 9);        -- Bands gain, Q7
    signal c_cw   : unsigned(7 downto 0) := to_unsigned(96, 8);         -- waist (xp units x 256 / 184)
    signal c_eg   : unsigned(7 downto 0) := to_unsigned(3, 8);          -- etch gain
    signal c_ws   : unsigned(9 downto 0) := to_unsigned(256, 10);       -- wingspan, Q8
    signal c_wsg  : unsigned(18 downto 0) := (others => '0');
    signal c_le   : unsigned(12 downto 0) := to_unsigned(525, 13);      -- KE ws G, Q12 per xp unit
    signal c_kmf  : unsigned(13 downto 0) := to_unsigned(916, 14);      -- KM ws G, Q8
    signal c_kw   : unsigned(9 downto 0) := to_unsigned(95, 10);        -- bowtie slope, Q8 Dv per xp unit
    signal c_sdk  : unsigned(15 downto 0) := to_unsigned(45, 16);       -- fray seed term
    signal c_lil  : std_logic_vector(23 downto 0) := C_BW_LIL0;
    signal c_shd  : std_logic_vector(23 downto 0) := C_BW_SHD0;

    -- field serial multiplier
    signal f_a   : unsigned(29 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(29 downto 0) := (others => '0');
    signal z_fa  : unsigned(29 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
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

    -- line engine
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_ph   : unsigned(23 downto 0) := (others => '0');
    signal e_ady  : unsigned(9 downto 0) := (others => '0');
    signal e_st   : unsigned(7 downto 0) := (others => '0');
    signal e_dv   : unsigned(9 downto 0) := (others => '0');
    signal e_t2   : unsigned(18 downto 0) := (others => '0');
    signal e_t    : unsigned(16 downto 0) := (others => '0');
    signal e_tn   : std_logic := '0';
    signal e_h    : unsigned(15 downto 0) := (others => '0');
    signal e_fr   : signed(8 downto 0) := (others => '0');
    signal e_dsq  : unsigned(19 downto 0) := (others => '0');
    signal m_a    : unsigned(27 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(27 downto 0) := (others => '0');
    signal n_lb0  : signed(13 downto 0) := (others => '0');
    signal n_l1   : unsigned(11 downto 0) := (others => '0');
    signal n_cs   : unsigned(7 downto 0) := (others => '0');
    signal n_kb   : signed(11 downto 0) := (others => '0');
    signal n_kth  : signed(9 downto 0) := to_signed(202, 10);
    signal n_vr1, n_vr2 : unsigned(7 downto 0) := (others => '0');
    signal l_lb0  : signed(13 downto 0) := (others => '0');
    signal l_l1   : unsigned(11 downto 0) := (others => '0');
    signal l_cs   : unsigned(7 downto 0) := (others => '0');
    signal l_kb   : signed(11 downto 0) := (others => '0');
    signal l_kth  : signed(9 downto 0) := to_signed(202, 10);
    signal l_vr1, l_vr2 : unsigned(7 downto 0) := (others => '0');

    -- beam (stage 1)
    signal b_hp  : unsigned(23 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    signal p2_xp, p3_xp, p4_xp, p5_xp, p6_xp, p7_xp : unsigned(8 downto 0) := (others => '0');
    signal p3_ux, p4_ux, p5_ux, p6_ux, p7_ux : unsigned(8 downto 0) := (others => '0');
    signal p3_wf  : unsigned(7 downto 0) := (others => '0');
    signal p4_cw  : unsigned(15 downto 0) := (others => '0');
    signal p4_l8, p5_l8, p6_l8, p7_l8, p8_l8 : unsigned(7 downto 0) := (others => '0');
    signal p5_w   : unsigned(8 downto 0) := (others => '0');
    signal p5_rn  : unsigned(7 downto 0) := (others => '0');
    signal p5_lb  : unsigned(7 downto 0) := (others => '0');
    signal p6_camp : unsigned(15 downto 0) := (others => '0');
    signal p6_xpb : unsigned(9 downto 0) := (others => '0');
    signal p7_cam, p8_cam, p9_cam : unsigned(9 downto 0) := (others => '0');
    signal p7_uxb : signed(10 downto 0) := (others => '0');
    signal p7_kw  : unsigned(18 downto 0) := (others => '0');
    signal p8_k1l, p8_k1h : signed(17 downto 0) := (others => '0');
    signal p8_kel : unsigned(14 downto 0) := (others => '0');
    signal p8_keh : unsigned(15 downto 0) := (others => '0');
    signal p8_ta  : unsigned(8 downto 0) := (others => '0');
    signal p8_t   : signed(8 downto 0) := (others => '0');
    signal p9_k1  : signed(26 downto 0) := (others => '0');
    signal p9_ke  : unsigned(21 downto 0) := (others => '0');
    signal p9_lt, p9_lc : signed(12 downto 0) := (others => '0');
    signal p8_kA, p8_kB, p9_kA, p9_kB, p10_kA, p10_kB, p11_kA, p11_kB, p12_kA, p12_kB, p13_kA, p13_kB : std_logic := '0';
    signal p9_sa  : signed(19 downto 0) := (others => '0');
    signal p10_sum : signed(27 downto 0) := (others => '0');
    signal p10_sa, p11_sa : signed(13 downto 0) := (others => '0');
    signal p10_lbc : signed(15 downto 0) := (others => '0');
    signal p10_ly, p11_ly, p12_ly, p13_ly : signed(12 downto 0) := (others => '0');
    signal p11_sb : signed(15 downto 0) := (others => '0');
    signal p12_d, p12_dn : signed(16 downto 0) := (others => '0');
    signal p12_mn : signed(15 downto 0) := (others => '0');
    signal p13_s  : signed(16 downto 0) := (others => '0');
    signal p13_ad, p14_ad, p15_ad : std_logic := '0';
    signal p14_key, p15_key : std_logic := '0';
    signal p14_ly, p15_ly : unsigned(9 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_ld : t_u10(0 to 3) := (others => to_unsigned(64, 10));
    type t_ra is array (0 to 255) of std_logic_vector(31 downto 0);
    type t_rb is array (0 to 255) of std_logic_vector(19 downto 0);
    signal o_ra : t_ra := (others => (others => '0'));
    signal o_rb : t_rb := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of o_ra : signal is true;
    attribute no_rw_check of o_rb : signal is true;
    signal o_ra_w, o_ra_r : unsigned(7 downto 0) := (others => '0');
    signal o_ra_q, o_fa : std_logic_vector(31 downto 0) := (others => '0');
    signal o_rb_q, o_fb : std_logic_vector(19 downto 0) := (others => '0');
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bbu, s_bbv, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
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
            r_ta_q <= C_BW_TA(to_integer(r_ta_a));
            r_dv_q <= C_BW_DV(to_integer(r_dv_a));
            r_rn_q <= C_BW_RND(to_integer(r_rn_a));
            r_pa_q <= C_BW_PAL(to_integer(r_pa_a));
            r_ex_q <= C_BW_EXP2(to_integer(r_ex_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(12 downto 0);
        variable v_e2 : signed(19 downto 0);
        variable v_t  : unsigned(23 downto 0);
        variable v_g  : unsigned(15 downto 0);
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
            -- roll speed: bipolar, +-8 deadband
            s_k3 <= unsigned(registers_in(2));
            v_s := signed(resize(s_k3, 11)) - 512;
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
                s_mir <= registers_in(6)(0);
                s_zig <= registers_in(6)(1);
                s_col <= registers_in(6)(2);
                s_inv <= registers_in(6)(3);
                s_src <= registers_in(6)(4);
                s_xs  <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run from the zone
            -- start by its own phase / channel counters (no zc decode on these enables)
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g4; s_gr <= unsigned(registers_in(3));
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
                            when 2      => s_g4 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 40 + 24 k, 16 iterations, result stable from load + 17
            -- (stores at load + 18).  The slot number is decoded at 38 + 24 k and the operands are pre-muxed
            -- into z_fa / z_fb at 39 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 38  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 62  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 86  => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 110 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 134 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 158 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 182 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 206 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 230 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 254 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 278 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 302 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when 326 => z_sel <= to_unsigned(12, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(8812, 30);         z_fb <= z_mt1;                          -- tile step
                    when 1  => z_fa <= resize(c_hsh, 30);             z_fb <= resize(to_unsigned(1920, 12) - s_x0, 16);
                    when 2  => z_fa <= to_unsigned(15020, 30);        z_fb <= z_mt2;                          -- rows
                    when 3  => z_fa <= resize(c_kv, 30);              z_fb <= resize(s_ys, 16);               -- per line
                    when 4  => z_fa <= resize(s_vm, 30);              z_fb <= resize(s_vm, 16);               -- speed^2
                    when 5  => z_fa <= resize(c_vsq, 30);             z_fb <= to_unsigned(648, 16);
                    when 6  => z_fa <= resize(s_g5(12 downto 3), 30); z_fb <= to_unsigned(193, 16);           -- waist
                    when 7  => z_fa <= resize(s_g6(12 downto 3), 30); z_fb <= resize(s_g6(12 downto 3), 16);  -- etch^2
                    when 8  =>                                                                               -- wingspan
                        if s_gp(12 downto 3) <= 384 then
                            z_fa <= resize(s_gp(12 downto 3), 30);       z_fb <= to_unsigned(683, 16);
                        else
                            z_fa <= resize(s_gp(12 downto 3) - 384, 30); z_fb <= to_unsigned(820, 16);
                        end if;
                    when 9  => z_fa <= resize(c_ws, 30);              z_fb <= resize(c_G, 16);
                    when 10 => z_fa <= resize(c_wsg, 30);             z_fb <= to_unsigned(525, 16);
                    when 11 => z_fa <= resize(c_wsg, 30);             z_fb <= to_unsigned(916, 16);
                    when others => z_fa <= resize(c_ws, 30);          z_fb <= to_unsigned(95, 16);
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(28 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's speed)
                    when 20 =>
                        if s_vneg = '1' then c_R <= c_R + resize(c_spd, 24);
                        else c_R <= c_R - resize(c_spd, 24); end if;
                        s_fc <= s_fc + 1;
                        if s_col = '1' then c_lil <= C_BW_LIL1; c_shd <= C_BW_SHD1;
                        else c_lil <= C_BW_LIL0; c_shd <= C_BW_SHD0; end if;
                    when 21 =>                                    -- fray seed: seed * 59 + 45, seed = fields / 8
                        v_g := resize(s_fc(10 downto 3), 16);
                        c_sdk <= shift_left(v_g, 6) - shift_left(v_g, 2) - v_g + 45;
                    -- exponents: tiles and rows 0.75 (k - 512) (+-1.5 octaves), bands 0.791 (k - 512)
                    when 22 =>
                        v_e := resize(signed(resize(s_g1(12 downto 3), 11)) - 512, 13);
                        v_e := shift_right(v_e + shift_left(v_e, 1), 2);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh1 <= v_e(10 downto 8);
                    when 24 => z_mt1 <= r_ex_q;
                    when 26 =>
                        v_e := resize(signed(resize(s_g2(12 downto 3), 11)) - 512, 13);
                        v_e := shift_right(v_e + shift_left(v_e, 1), 2);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh2 <= v_e(10 downto 8);
                    when 28 => z_mt2 <= r_ex_q;
                    when 30 =>
                        z_e <= signed(resize(s_g4(12 downto 3), 11)) - 512;
                    when 31 =>                                    -- (k - 512) * 405 >> 9
                        v_e2 := resize(z_e, 20);
                        v_e2 := shift_right(shift_left(v_e2, 8) + shift_left(v_e2, 7) + shift_left(v_e2, 4) + shift_left(v_e2, 2) + v_e2, 9);
                        r_ex_a <= unsigned(v_e2(7 downto 0)); z_sh4 <= v_e2(10 downto 8);
                    when 33 =>                                    -- G = 128 mt >> (15 - sh) = mt >> (8 - sh)
                        case to_integer(z_sh4) is
                            when -2     => c_G <= resize(r_ex_q(15 downto 10), 9);
                            when -1     => c_G <= resize(r_ex_q(15 downto 9), 9);
                            when 0      => c_G <= resize(r_ex_q(15 downto 8), 9);
                            when others => c_G <= r_ex_q(15 downto 7);
                        end case;
                    when 58 =>                                    -- tile step = 8812 mt >> (15 - sh)
                        case to_integer(z_sh1) is
                            when -2     => c_hsh <= resize(f_acc(29 downto 17), 17);
                            when -1     => c_hsh <= resize(f_acc(29 downto 16), 17);
                            when 0      => c_hsh <= resize(f_acc(29 downto 15), 17);
                            when others => c_hsh <= resize(f_acc(29 downto 14), 17);
                        end case;
                    when 82 =>                                    -- line start phase; per-pixel step
                        c_hp0 <= to_unsigned(C_PHC, 24) - f_acc(23 downto 0);
                        case to_integer(s_xs) is
                            when 4      => c_hpx <= resize(shift_left(resize(c_hsh, 20), 2), 24);
                            when 3      => c_hpx <= resize(shift_left(resize(c_hsh, 20), 1) + resize(c_hsh, 20), 24);
                            when others => c_hpx <= resize(shift_left(resize(c_hsh, 20), 1), 24);
                        end case;
                    when 106 =>                                   -- rows: 15020 mt >> (15 - sh)
                        case to_integer(z_sh2) is
                            when -2     => c_kv <= resize(f_acc(29 downto 17), 17);
                            when -1     => c_kv <= resize(f_acc(29 downto 16), 17);
                            when 0      => c_kv <= resize(f_acc(29 downto 15), 17);
                            when others => c_kv <= resize(f_acc(29 downto 14), 17);
                        end case;
                    when 130 => c_kvl <= resize(f_acc(29 downto 8), 24);
                    when 154 => c_vsq <= f_acc(17 downto 0);
                    when 178 => c_spd <= f_acc(27 downto 10);
                    when 202 => c_cw  <= f_acc(17 downto 10);
                    when 226 =>                                   -- etch k6^2 >> 12; the Osc at 4x + 16
                        if s_src = '1' then
                            v_t := resize(f_acc(19 downto 12), 24);
                            v_t := shift_left(v_t, 2) + 16;
                            if v_t > 255 then c_eg <= (others => '1'); else c_eg <= v_t(7 downto 0); end if;
                        else
                            c_eg <= f_acc(19 downto 12);
                        end if;
                    when 250 =>
                        if s_gp(12 downto 3) <= 384 then c_ws <= f_acc(19 downto 10);
                        else c_ws <= f_acc(19 downto 10) + 256; end if;
                    when 274 => c_wsg <= f_acc(18 downto 0);
                    when 298 => c_le  <= f_acc(27 downto 15);
                    when 322 => c_kmf <= f_acc(28 downto 15);
                    when 346 => c_kw  <= f_acc(17 downto 8);
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
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (16 iterations, result at
    -- load + 17): the oscillator waveform Dv, the B line terms LB0 and L1, the
    -- waist (a clipped parabola of the virtual line), the fray, the ripple phases
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_dy : signed(11 downto 0);
        variable v_h  : unsigned(15 downto 0);
        variable v_x  : unsigned(15 downto 0);
        variable v_s8 : signed(10 downto 0);
        variable v_t  : signed(19 downto 0);
        variable v_f  : signed(9 downto 0);
        variable v_vy : unsigned(10 downto 0);
        variable v_r  : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 400 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_ph <= c_R;
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_ph <= e_ph + c_kvl;
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(26 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            v_vy := e_vy(18 downto 8);
            case to_integer(e_lc) is
                when 1 =>
                    r_dv_a <= e_ph(23 downto 14);
                    v_dy := signed(resize(v_vy, 12)) - 704;
                    if v_dy < 0 then v_dy := -v_dy; end if;
                    e_ady <= unsigned(v_dy(9 downto 0));
                    e_st  <= e_vy(18 downto 11);
                when 2 =>
                    m_a <= resize(e_ady, 28); m_b <= resize(e_ady, 16); m_acc <= (others => '0');
                    v_x := resize(e_st, 16);                       -- strip * 159 + seed term
                    e_h <= shift_left(v_x, 7) + shift_left(v_x, 5) - v_x + c_sdk;
                when 3 =>
                    e_dv <= r_dv_q;
                    e_h <= e_h xor shift_left(e_h, 5);
                when 4 =>
                    e_t2 <= shift_left(resize(e_dv, 19), 8) + shift_left(resize(e_dv, 19), 4) - shift_left(resize(e_dv, 19), 1);  -- 270 Dv
                    v_x := resize(e_h(15 downto 7), 16);
                    e_h <= e_h + shift_left(v_x, 3) + shift_left(v_x, 2) + v_x;    -- + (h >> 7) * 13
                when 5 =>
                    v_t := to_signed(107494, 20) - signed(resize(e_t2, 20));
                    if v_t < 0 then e_t <= unsigned(resize(-v_t, 17)); e_tn <= '1';
                    else e_t <= unsigned(v_t(16 downto 0)); e_tn <= '0'; end if;
                    e_h <= e_h xor shift_right(e_h, 4);
                when 6 =>
                    e_fr <= signed(resize(e_h(7 downto 0), 9)) - 128;
                when 7 =>
                    v_f := resize(e_fr, 10);
                    v_f := shift_left(v_f, 3) + shift_left(v_f, 2);              -- fr * 12
                    n_kth <= to_signed(202, 10) - resize(shift_right(v_f, 7), 10);   -- side-limit threshold, frayed
                    n_kb <= signed(resize(e_dv, 12)) - 155;       -- (the fray tears only the side edges)
                when 8 =>
                    v_r := resize(v_vy, 13) + shift_left(resize(v_vy, 13), 2);   -- vy * 5 >> 2
                    n_vr1 <= v_r(9 downto 2);
                    v_r := resize(v_vy, 13) + shift_left(resize(v_vy, 13), 1);   -- vy * 3 >> 1
                    n_vr2 <= v_r(8 downto 1);
                when 19 =>
                    e_dsq <= m_acc(19 downto 0);
                    m_a <= resize(e_dv, 28); m_b <= resize(c_kmf, 16); m_acc <= (others => '0');
                when 36 =>
                    n_l1 <= m_acc(22 downto 11);
                    m_a <= resize(e_dsq, 28); m_b <= to_unsigned(115, 16); m_acc <= (others => '0');
                when 53 =>
                    v_s8 := to_signed(294, 11) - signed(resize(m_acc(26 downto 16), 11));
                    if v_s8 < 0 then v_s8 := (others => '0'); elsif v_s8 > 256 then v_s8 := to_signed(256, 11); end if;
                    m_a <= resize(c_cw, 28); m_b <= resize(unsigned(v_s8(8 downto 0)), 16); m_acc <= (others => '0');
                when 70 =>
                    n_cs <= m_acc(15 downto 8);
                    m_a <= resize(e_t, 28); m_b <= resize(c_G, 16); m_acc <= (others => '0');
                when 87 =>
                    if e_tn = '1' then n_lb0 <= -signed(resize(m_acc(27 downto 15), 14));
                    else n_lb0 <= signed(resize(m_acc(27 downto 15), 14)); end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the horizontal tile phase (stage 1); loads the line terms at g_start
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_hp <= c_hp0;
                b_px <= (others => '0');
                l_lb0 <= n_lb0; l_l1 <= n_l1; l_cs <= n_cs; l_kb <= n_kb; l_kth <= n_kth;
                l_vr1 <= n_vr1; l_vr2 <= n_vr2;
            elsif g_avid = '1' then
                b_hp <= b_hp + c_hpx;
                b_px <= b_px + 1;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_xp : unsigned(8 downto 0);
        variable v_w  : unsigned(9 downto 0);
        variable v_lb : unsigned(9 downto 0);
        variable v_ad : signed(16 downto 0);
        variable v_ch : signed(16 downto 0);
        variable v_s  : signed(16 downto 0);
        variable v_ix : unsigned(8 downto 0);
        variable v_ka : signed(12 downto 0);
        variable v_kb : signed(12 downto 0);
        variable v_r1, v_r2 : unsigned(7 downto 0);
        variable v_t  : signed(8 downto 0);
        variable v_lt : signed(13 downto 0);
        variable v_lc : signed(13 downto 0);
        variable v_w24 : std_logic_vector(23 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: fold the tile phase into xp (0 = the seam, 511 = the tile centre); Mirror off = sawtooth
            if s_mir = '1' then
                v_xp := not b_hp(23 downto 15);
            elsif b_hp(23) = '1' then
                v_xp := b_hp(22 downto 14);
            else
                v_xp := not b_hp(22 downto 14);
            end if;
            p2_xp <= v_xp;
            s_lb_ra <= b_px;
            -- 3: distances; waist weight; fold-rounding address
            p3_xp <= p2_xp;
            p3_ux <= not p2_xp;
            if p2_xp < 184 then p3_wf <= resize(to_unsigned(184, 9) - p2_xp, 8); else p3_wf <= (others => '0'); end if;
            r_rn_a <= not p2_xp;
            -- 4: waist product; luma
            p4_cw <= l_cs * p3_wf;
            p4_xp <= p3_xp; p4_ux <= p3_ux;
            p4_l8 <= unsigned(s_lb_q);
            -- 5: TA index; rounding (fabric copy); the camera luma box-filtered over 4 px (it bends, not shreds)
            v_w := resize(p4_xp, 10) + resize(p4_cw(15 downto 8), 10);
            if v_w > 511 then p5_w <= (others => '1'); else p5_w <= v_w(8 downto 0); end if;
            p5_rn <= r_rn_q;
            v_lb := resize(p4_l8, 10) + resize(p5_l8, 10) + resize(p6_l8, 10) + resize(p7_l8, 10);
            p5_lb <= v_lb(9 downto 2);
            p5_xp <= p4_xp; p5_ux <= p4_ux; p5_l8 <= p4_l8;
            -- 6: TA address; xpb; camera product
            r_ta_a <= p5_w;
            p6_xpb <= resize(p5_xp, 10) + resize(p5_rn, 10);
            p6_camp <= p5_lb * c_eg;
            p6_xp <= p5_xp; p6_ux <= p5_ux; p6_l8 <= p5_l8;
            -- 7: 511 - xpb; bowtie slope product
            p7_uxb <= to_signed(511, 11) - signed(resize(p6_xpb, 11));
            p7_kw <= p6_ux * c_kw;
            p7_cam <= p6_camp(15 downto 6);
            p7_xp <= p6_xp; p7_ux <= p6_ux; p7_l8 <= p6_l8;
            -- 8: KM and KE partial products (L1 split 6 / 6 bits, LE split 6 / 7 bits); TA (fabric copy); the
            -- bowtie key's two limits; the ripple phase
            p8_k1l <= p7_uxb * signed('0' & l_l1(5 downto 0));
            p8_k1h <= p7_uxb * signed('0' & l_l1(11 downto 6));
            p8_kel <= p7_ux * c_le(5 downto 0);
            p8_keh <= p7_ux * c_le(12 downto 6);
            p8_ta <= r_ta_q;
            v_kb := signed(resize(p7_kw(18 downto 8), 13)) + signed(resize(p7_cam(9 downto 2), 13));
            if resize(l_kb, 13) < v_kb then p8_kB <= '1'; else p8_kB <= '0'; end if;
            v_ka := signed(resize(p7_xp, 13)) + signed(resize(p7_cam(9 downto 2), 13));
            if v_ka >= resize(l_kth, 13) then p8_kA <= '1'; else p8_kA <= '0'; end if;
            v_r1 := resize(shift_left(resize(p7_xp, 10), 1), 8) + l_vr1;
            v_r2 := l_vr2 - resize(shift_right(resize(p7_xp, 11) + shift_left(resize(p7_xp, 11), 1), 2), 8);
            p8_t <= signed(resize(f_tri7(v_r1), 9)) + signed(resize(f_tri7(v_r2), 9)) - 127;
            p8_cam <= p7_cam; p8_l8 <= p7_l8;
            -- 9: sA x Bands; KM and KE sums; the lilac luma's ripple and camera terms
            p9_sa <= (signed(resize(p8_ta, 10)) - 64) * signed('0' & c_G);
            p9_k1 <= resize(p8_k1l, 27) + shift_left(resize(p8_k1h, 27), 6);
            p9_ke <= resize(p8_kel, 22) + shift_left(resize(p8_keh, 22), 6);
            v_lt := resize(p8_t, 14);
            p9_lt <= resize(shift_right(shift_left(v_lt, 5) + shift_left(v_lt, 1), 7), 13);         -- t * 34 >> 7
            v_lc := signed(resize(p8_l8, 14)) - 110;
            p9_lc <= resize(shift_right(shift_left(v_lc, 4) + shift_left(v_lc, 3) + shift_left(v_lc, 1), 6), 13);  -- * 26 >> 6
            p9_kA <= p8_kA; p9_kB <= p8_kB; p9_cam <= p8_cam;
            -- 10: sA + camera; the B sum; LB0 + camera; the lilac luma
            p10_sa <= resize(shift_right(p9_sa, 7), 14) + signed(resize(p9_cam, 14));
            p10_sum <= signed(resize(p9_ke, 28)) + shift_left(resize(p9_k1, 28), 3);
            p10_lbc <= resize(l_lb0, 16) + signed(resize(p9_cam, 16));
            p10_ly <= signed(resize(unsigned(c_lil(23 downto 16)) & "00", 13)) + p9_lt + p9_lc;
            p10_kA <= p9_kA; p10_kB <= p9_kB;
            -- 11: sB
            p11_sa <= p10_sa;
            p11_sb <= p10_lbc + resize(shift_right(p10_sum, 12), 16);
            p11_kA <= p10_kA; p11_kB <= p10_kB; p11_ly <= p10_ly;
            -- 12: the min (Zigzag: sB alone); both differences for the chamfer
            if s_zig = '1' then
                p12_d <= to_signed(1000, 17); p12_dn <= to_signed(-1000, 17);
                p12_mn <= p11_sb;
            else
                p12_d <= resize(p11_sa, 17) - resize(p11_sb, 17);
                p12_dn <= resize(p11_sb, 17) - resize(p11_sa, 17);
                if resize(p11_sa, 16) < p11_sb then p12_mn <= resize(p11_sa, 16); else p12_mn <= p11_sb; end if;
            end if;
            p12_kA <= p11_kA; p12_kB <= p11_kB; p12_ly <= p11_ly;
            -- 13: chamfer the corners (the camera is already in both terms)
            if p12_d < 0 then v_ad := p12_dn; else v_ad := p12_d; end if;
            if v_ad < 16 then v_ch := shift_right(16 - v_ad, 1); else v_ch := (others => '0'); end if;
            p13_s <= resize(p12_mn, 17) - v_ch;
            if p12_d < 0 then p13_ad <= '1'; else p13_ad <= '0'; end if;
            p13_kA <= p12_kA; p13_kB <= p12_kB; p13_ly <= p12_ly;
            -- 14: palette address; the key; clip the lilac luma
            if p13_s < -128 then
                v_ix := (others => '0');
            elsif p13_s < 256 then
                v_s := p13_s + 128;
                v_ix := unsigned(v_s(8 downto 0));
            else
                v_ix := "11" & unsigned(p13_s(6 downto 0));
            end if;
            r_pa_a <= s_col & v_ix;
            if s_inv = '1' then
                if p13_s < 28 then p14_key <= '1'; else p14_key <= '0'; end if;
            elsif s_zig = '1' then
                p14_key <= p13_kB;
            else
                p14_key <= p13_kA and p13_kB;
            end if;
            if p13_ly < 64 then p14_ly <= to_unsigned(64, 10);
            elsif p13_ly > 940 then p14_ly <= to_unsigned(940, 10);
            else p14_ly <= unsigned(p13_ly(9 downto 0)); end if;
            p14_ad <= p13_ad;
            -- 15: palette ROM (EBR)
            p15_key <= p14_key; p15_ad <= p14_ad; p15_ly <= p14_ly;
            -- 16: compose from the palette word (shade the violet bands where the wing-side term wins); neutral
            -- outside the avid
            if p15_ad = '1' and r_pa_q(8) = '1' then v_w24 := c_shd; else v_w24 := r_pa_q; end if;
            if s_gav_sr(13) = '0' then
                s_y16 <= to_unsigned(64, 10); s_u16 <= to_unsigned(512, 10); s_v16 <= to_unsigned(512, 10);
            elsif p15_key = '1' then
                s_y16 <= p15_ly;
                s_u16 <= unsigned(c_lil(15 downto 8)) & "00";
                s_v16 <= unsigned(c_lil(7 downto 0)) & "00";
            else
                s_y16 <= unsigned(v_w24(23 downto 16)) & "00";
                s_u16 <= unsigned(v_w24(15 downto 8)) & "00";
                s_v16 <= unsigned(v_w24(7 downto 0)) & "00";
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(13);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (each
    -- two adjacent 8-sums, with the resync), luma [1 2 1]/4 on the matching delay,
    -- luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    -- 16-clock ring buffers in block RAM (written every clock, read 8 samples back through the EBR output
    -- register and a fabric register; the read address never meets the write address: no_rw_check):
    -- ring A carries box A's input (Y, U, V of stage 16), ring B box B's (U, V of stage 18)
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            o_ra_q <= o_ra(to_integer(o_ra_r));
            o_ra(to_integer(o_ra_w)) <= "00" & std_logic_vector(s_y16) & std_logic_vector(s_u16) & std_logic_vector(s_v16);
            o_rb_q <= o_rb(to_integer(o_ra_r));
            o_rb(to_integer(o_ra_w)) <= std_logic_vector(s_u18) & std_logic_vector(s_v18);
        end if;
    end process p_ring;

    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_yl : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            o_ra_w <= o_ra_w + 1;
            o_ra_r <= o_ra_w - 5;              -- read issue = write - 6: the sample written 8 clocks before its use
            o_fa <= o_ra_q;                    -- fabric copies of the ring outputs
            o_fb <= o_rb_q;
            -- The running 8-sums snap back to neutral (the off-avid input of both boxes) once 8 blank samples
            -- are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 17 sums, stage 18 output
            if s_bah = "00000000" then
                s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bau <= s_bau + s_u16 - unsigned(o_fa(19 downto 10));
                s_bav <= s_bav + s_v16 - unsigned(o_fa(9 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(15) = '1' then
                v_sc := resize(s_bau, 14) + s_baud; s_u18 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v18 <= v_sc(13 downto 4);
            else
                s_u18 <= to_unsigned(512, 10); s_v18 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(15);
            -- box B: stage 19 sums, stage 20 output
            if s_bbh = "00000000" then
                s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bbu <= s_bbu + s_u18 - unsigned(o_fb(19 downto 10));
                s_bbv <= s_bbv + s_v18 - unsigned(o_fb(9 downto 0));
            end if;
            s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bbu, 14) + s_bbud; s_u20 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v20 <= v_sc(13 downto 4);
            -- luma: [1 2 1] centred 8 pixels back (the boxes' group delay): ring A's Y (8 back) + 4 registers
            s_ld(0) <= unsigned(o_fa(29 downto 20));
            for i in 1 to 3 loop s_ld(i) <= s_ld(i - 1); end loop;
            v_yl := resize(s_ld(1), 12) + shift_left(resize(s_ld(2), 12), 1) + resize(s_ld(3), 12);
            s_y20 <= v_yl(11 downto 2);
            -- 21: luma 13/16 and the blanking gate
            if s_y20 > 64 then v_dl := s_y20 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y21 <= s_y20 - shift_right(v_dl, 3) - shift_right(v_dl, 4); s_u21 <= s_u20; s_v21 <= s_v20;
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

end architecture batwing;
