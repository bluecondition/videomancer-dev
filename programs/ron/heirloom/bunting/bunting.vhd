-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Bunting v0.5.0 -- Heirloom #35, rebuilt from the photo WIN_20230521_23_37_25.
--
-- The photo: a white ground cut into yellow and cyan pennants by two bow-tie "X"s (centres x 527
-- and 1471, y 516: one H triangle oscillator, period 945 px), a middle band that flares open to
-- the right (half-height 178 + 6.9e-5 (x - 42)^2 lines, mirrored about y 519) and, over it, a
-- 4.3 x 4.6 grid of identical 8 x 3-block glyph marks (pitch 450 x 233) keyed into the ground:
-- red on yellow, blue on cyan, and on white a black one-shot (33 px from each run start) that
-- charges up to grey.  Everything is softened horizontally (~16 px); the first 65 px of each line
-- hold the X oscillator.
--
-- The patch, as fitted (.heirloom_work/rb_bunting/fit2.py, gedge3.py, oneshot.py):
--   Tx  = H triangle (0 at the X centres, 1 midway), Ty = |y - 516.2|
--   K   = Ty < 309 s (Tx - 0.106)  or  Ty > 70.1 + 876 Tx / s        (the bow-tie wedges)
--   mid = |y - 519.1| < z0 + kz max(x - 42.5, 0)^2                   (the flaring band)
--   ground = mid ? (K ? cyan : white) : (K ? white : yellow)
--   glyph ink: yellow -> red (mask dilated 3.8 px), cyan -> blue (eroded 3.8 px),
--              white -> black for 33 px of each run, then an RC slew from dark grey to the source
--
--   K1  Flags    X oscillator frequency: 0.75 .. 6 bow-ties across (2.03 = the photo)
--   K2  Band     the middle band's height at the left edge (178 lines = the photo)
--   K3  Flare    how fast the band opens to the right (0 = straight, 3x the photo at 100 %)
--   K4  Speed    bipolar (centre +-8 still): the X's drift sideways, the glyph grid scrolls back
--   K5  Glyphs   glyph grid size: 1.5 .. 9 columns (4.26 = the photo)
--   K6  Bend     how far the picture (or the Osc) bends the X oscillator's phase
--   S7  Shot     On (the photo's black one-shot in the grey glyphs) / Off (plain grey)
--   S8  Swap     yellow and cyan exchanged (inks follow: red on yellow, blue on cyan)
--   S9  Flip     the glyph marks mirrored
--   S10 Edges    Soft (the photo's horizontal softness) / Hard
--   S11 Source   Video / Osc
--   P12 SPREAD   the bow-ties' opening: 0 % furled slivers, 50 % the photo, 100 % wide fans with a
--                yellow zig-zag string
--
-- Engine: vblank-zone sequencer (one serial multiplier, one serial divider, shared glide), a
-- per-line engine kicked at each line start (computes the NEXT line's wedge thresholds, band
-- limit and glyph row), beam DDAs (X phase, band parabola (x - 42.5)^2 exact, glyph phase), a
-- 20-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid;
-- soft [1 2 .. 2 1] box on Y/U/V; output conditioning (chroma [1 2 .. 2 1]/16, luma [1 2 1],
-- luma 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture bunting of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- fitted constants (h = half virtual px; Q8 = virtual lines * 256; Q12 = fraction of the half period)
    constant C_CY8  : integer := 132140;    -- X centre y 516.17
    constant C_CYZ8 : integer := 132882;    -- band axis y 519.07
    constant C_V08  : integer := 17946;     -- V wedge apex 70.1 lines from the centre
    constant C_H0N  : integer := 433;       -- H wedge apex 0.1057 of the half period from the centre
    constant C_XH   : integer := 130;       -- line-start hold (65 virtual px)
    constant C_XZ   : integer := 85;        -- band parabola vertex (42.5 virtual px)
    constant C_UANC   : integer := 14011240;  -- glyph x phase at the anchor (+64/4096 offset), Q24
    constant C_VANC   : integer := 11540376;  -- glyph y phase at the anchor, Q24
    constant C_DIL  : integer := 35;        -- glyph ink dilation (3.8 px), Q12 of the pitch

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

    -- rings (block RAM): soft box {Y, U, V}, shell chroma box {U, V}
    type t_sring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal sr_ram : t_sring := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    type t_cring is array (0 to 255) of std_logic_vector(19 downto 0);
    signal cr_ram : t_cring := (others => "1000000000" & "1000000000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of sr_ram : signal is true;
    attribute no_rw_check of cr_ram : signal is true;
    signal s_wp, sr_ra, cr_ra : unsigned(7 downto 0) := (others => '0');
    signal sr_wd : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";
    signal sr_q, sr_q2 : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";
    signal cr_wd : std_logic_vector(19 downto 0) := "1000000000" & "1000000000";
    signal cr_q, cr_q2 : std_logic_vector(19 downto 0) := "1000000000" & "1000000000";

    -- oscillator source (Source = Osc): 2-D triangle lattice, ~2520 x 2160 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(8 downto 0) := (others => '0');
    signal s_oxs : unsigned(5 downto 0) := to_unsigned(26, 6);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(61, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width / mode
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xh0  : unsigned(8 downto 0) := (others => '0');
    signal s_xd   : unsigned(10 downto 0) := to_unsigned(1868, 11);   -- 1998 - start of the X phase
    signal s_xg   : unsigned(10 downto 0) := to_unsigned(1898, 11);   -- 1898 - line start (glyph anchor)
    signal s_s0   : unsigned(17 downto 0) := to_unsigned(7225, 18);
    signal s_d0   : signed(12 downto 0) := to_signed(-336, 13);
    signal s_dd   : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_hd   : std_logic := '1';                                 -- 1080 class (soft box 16, slew >> 5)
    signal s_short : std_logic := '0';                                -- 720p / SD: the short chroma box
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(9 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(506 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(507 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(591 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(621 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(340 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(600, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_shot, s_swap, s_flip, s_hard, s_src : std_logic := '0';
    signal s_flat : std_logic := '0';

    -- per-field constants
    signal c_k1sq, c_k3sq, c_k5sq : unsigned(19 downto 0) := (others => '0');
    signal c_inch  : unsigned(14 downto 0) := to_unsigned(12160, 15);   -- X phase per h (2^24 / period)
    signal c_incp  : unsigned(16 downto 0) := to_unsigned(24320, 17);   -- X phase per pixel
    signal c_po    : unsigned(25 downto 0) := (others => '0');          -- (1998 - xs) * inc_h
    signal c_s     : unsigned(12 downto 0) := to_unsigned(4096, 13);    -- spread, Q12
    signal c_is    : unsigned(17 downto 0) := to_unsigned(65536, 18);   -- 1 / s, Q16
    signal c_iar, c_ial : unsigned(17 downto 0) := (others => '0');
    signal c_sb    : unsigned(15 downto 0) := (others => '0');
    signal c_ikz   : unsigned(22 downto 0) := (others => '0');
    signal c_z0    : unsigned(16 downto 0) := (others => '0');
    signal c_duh   : unsigned(15 downto 0) := to_unsigned(18628, 16);
    signal c_dup   : unsigned(17 downto 0) := (others => '0');
    signal c_uo    : unsigned(26 downto 0) := (others => '0');
    signal c_dvv   : unsigned(17 downto 0) := (others => '0');
    signal c_dvl   : unsigned(19 downto 0) := (others => '0');
    signal c_vo    : unsigned(26 downto 0) := (others => '0');
    signal c_drs, c_scs : unsigned(16 downto 0) := (others => '0');
    signal c_dr, c_sc : unsigned(23 downto 0) := (others => '0');       -- drift / scroll, Q24
    signal c_phi0  : unsigned(23 downto 0) := (others => '0');
    signal c_u0    : unsigned(23 downto 0) := (others => '0');
    signal c_v0    : unsigned(23 downto 0) := (others => '0');
    signal c_g6    : unsigned(7 downto 0) := to_unsigned(85, 8);

    -- field serial multiplier (a 20 x b 18 bits, 18 iterations) and divider (35 / 20 bits)
    signal f_a   : unsigned(37 downto 0) := (others => '0');
    signal f_b   : unsigned(17 downto 0) := (others => '0');
    signal f_acc : unsigned(37 downto 0) := (others => '0');
    signal z_fa  : unsigned(19 downto 0) := (others => '0');
    signal z_fb  : unsigned(17 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal z_neg : std_logic := '0';
    signal d_num : unsigned(34 downto 0) := (others => '0');
    signal d_den : unsigned(19 downto 0) := (others => '1');
    signal d_rem : unsigned(20 downto 0) := (others => '0');
    signal d_q   : unsigned(24 downto 0) := (others => '0');
    signal d_i   : unsigned(5 downto 0) := (others => '0');
    signal d_run, d_ld : std_logic := '0';
    signal d_job : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_lc   : unsigned(6 downto 0) := to_unsigned(127, 7);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(19 downto 0) := (others => '0');
    signal e_vg   : unsigned(23 downto 0) := (others => '0');
    signal e_ty, e_tz : unsigned(17 downto 0) := (others => '0');
    signal e_dz   : signed(18 downto 0) := (others => '0');
    signal e_tvz  : std_logic := '0';
    signal e_row  : unsigned(1 downto 0) := "11";
    signal m_a    : unsigned(40 downto 0) := (others => '0');
    signal m_b    : unsigned(17 downto 0) := (others => '0');
    signal m_acc  : unsigned(40 downto 0) := (others => '0');
    signal n_thr, n_thl, n_tv : unsigned(11 downto 0) := (others => '0');
    signal n_midl : std_logic := '0';
    signal n_r2   : unsigned(23 downto 0) := (others => '1');
    signal n_s1, n_e1, n_s2, n_e2 : unsigned(11 downto 0) := to_unsigned(4095, 12);
    signal n_rok  : std_logic := '0';

    -- beam
    signal b_xh, b_x1 : unsigned(11 downto 0) := (others => '0');
    signal b_phi : unsigned(23 downto 0) := (others => '0');
    signal b_u   : unsigned(23 downto 0) := (others => '0');
    signal b_s   : unsigned(23 downto 0) := (others => '0');
    signal b_d   : signed(16 downto 0) := (others => '0');
    signal l_thr, l_thl, l_tv : unsigned(11 downto 0) := (others => '0');
    signal l_midl : std_logic := '0';
    signal l_r2   : unsigned(23 downto 0) := (others => '1');
    signal l_s1, l_e1, l_s2, l_e2 : unsigned(11 downto 0) := to_unsigned(4095, 12);
    signal l_rok  : std_logic := '0';

    -- pixel path
    signal p1_l8  : unsigned(7 downto 0) := (others => '0');
    signal p1_os  : signed(8 downto 0) := (others => '0');
    signal p1_phi, p2_phi, p3_phi, p4_phi : unsigned(23 downto 0) := (others => '0');
    signal p1_zs, p1_zx : std_logic := '0';
    signal p1_ur  : unsigned(11 downto 0) := (others => '0');
    signal p1_urp : unsigned(12 downto 0) := (others => '0');
    signal p1_urm : signed(12 downto 0) := (others => '0');
    signal p2_bin : signed(9 downto 0) := (others => '0');
    signal p2_src : unsigned(7 downto 0) := (others => '0');
    signal p2_gw, p2_gy, p2_gc, p2_mid : std_logic := '0';
    signal p3_pl, p3_ph : signed(14 downto 0) := (others => '0');
    signal p4_bp  : signed(18 downto 0) := (others => '0');
    signal p5_phib : unsigned(23 downto 0) := (others => '0');
    signal p6_txn : unsigned(11 downto 0) := (others => '0');
    signal p6_rt  : std_logic := '0';
    signal p7_k   : std_logic := '0';
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    signal p_sv   : t_u8a(3 to 8) := (others => (others => '0'));
    type t_sla is array (natural range <>) of std_logic;
    signal p_gw, p_gy, p_gc, p_mid : t_sla(3 to 7) := (others => '0');
    signal p8_cls : unsigned(1 downto 0) := "00";
    signal p8_wg, p8_red, p8_blu : std_logic := '0';
    signal p8_sa  : unsigned(11 downto 0) := (others => '0');
    signal p9_cls : unsigned(1 downto 0) := "00";
    signal p9_wg, p9_red, p9_blu, p9_blk : std_logic := '0';
    signal p9_g   : unsigned(7 downto 0) := (others => '0');
    signal x_cnt  : unsigned(6 downto 0) := (others => '0');
    signal x_g    : unsigned(11 downto 0) := to_unsigned(1520, 12);
    signal x_seg  : unsigned(6 downto 0) := (others => '0');
    signal x_lf   : std_logic_vector(15 downto 0) := x"ACE1";
    signal x_nz   : signed(3 downto 0) := (others => '0');
    signal p10_code : unsigned(2 downto 0) := (others => '0');
    signal p10_g  : unsigned(7 downto 0) := (others => '0');
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_hw1  : std_logic_vector(29 downto 0) := (others => '0');
    signal s_sy, s_su, s_sv, s_syd, s_sud, s_svd : unsigned(14 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10a is array (natural range <>) of unsigned(9 downto 0);
    signal s_ly : t_u10a(0 to 6) := (others => to_unsigned(64, 10));
    signal s_usum, s_vsum, s_ud, s_vd : unsigned(13 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Rings (block RAM; the read never meets the write address)
    ---------------------------------------------------------------------------
    p_sring : process(clk)
    begin
        if rising_edge(clk) then
            sr_ram(to_integer(s_wp)) <= sr_wd;
            sr_q <= sr_ram(to_integer(sr_ra));
            sr_q2 <= sr_q;
        end if;
    end process p_sring;

    p_cring : process(clk)
    begin
        if rising_edge(clk) then
            cr_ram(to_integer(s_wp)) <= cr_wd;
            cr_q <= cr_ram(to_integer(cr_ra));
            cr_q2 <= cr_q;
        end if;
    end process p_cring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_r  : unsigned(20 downto 0);
        variable v_t  : unsigned(23 downto 0);
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
            -- speed: |k4 - 512| - 8 (deadband +-8), sign
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
                s_shot <= not registers_in(6)(0);
                s_swap <= registers_in(6)(1);
                s_flip <= registers_in(6)(2);
                s_hard <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4 =>                                         -- SD: the centre 1440 virtual px
                        s_xh0 <= to_unsigned(480, 9); s_xd <= to_unsigned(1518, 11); s_xg <= to_unsigned(1418, 11);
                        s_s0 <= to_unsigned(156025, 18); s_d0 <= to_signed(3176, 13); s_dd <= to_unsigned(32, 6);
                        s_hd <= '0'; s_short <= '1'; s_oxs <= to_unsigned(52, 6);
                    when 3 =>                                         -- 720p
                        s_xh0 <= (others => '0'); s_xd <= to_unsigned(1868, 11); s_xg <= to_unsigned(1898, 11);
                        s_s0 <= to_unsigned(7225, 18); s_d0 <= to_signed(-501, 13); s_dd <= to_unsigned(18, 6);
                        s_hd <= '0'; s_short <= '1'; s_oxs <= to_unsigned(39, 6);
                    when others =>                                    -- 1080
                        s_xh0 <= (others => '0'); s_xd <= to_unsigned(1868, 11); s_xg <= to_unsigned(1898, 11);
                        s_s0 <= to_unsigned(7225, 18); s_d0 <= to_signed(-336, 13); s_dd <= to_unsigned(8, 6);
                        s_hd <= '1'; s_short <= '0'; s_oxs <= to_unsigned(26, 6);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 18) * to_unsigned(61, 6), 8), 10);

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

            -- serial products: slot k pre-selected at zc = 40 + 32 k, loaded at +2, 18 iterations,
            -- result stored at +24
            z_pre <= '0';
            if s_zone = '1' and s_zc(4 downto 0) = "01000" and s_zc >= 40 and s_zc <= 552 then
                z_sel <= resize(shift_right(s_zc - 40, 5), 5);
                z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_g1(12 downto 3), 20); z_fb <= resize(s_g1(12 downto 3), 18);   -- k1^2
                    when 1  => z_fa <= c_k1sq;                         z_fb <= to_unsigned(22984, 18);         -- inc_h
                    when 2  => z_fa <= resize(c_inch, 20);             z_fb <= resize(s_xd, 18);              -- X phase origin
                    when 3  =>                                                                               -- spread
                        if s_gp(12) = '0' then
                            z_fa <= resize(s_gp(12 downto 3), 20); z_fb <= to_unsigned(4920, 18); z_neg <= '0';
                        else
                            z_fa <= resize(s_gp(11 downto 3), 20); z_fb <= to_unsigned(6560, 18); z_neg <= '1';
                        end if;
                    when 4  =>                                                                               -- k3^2 (floor 64)
                        if s_g3(12 downto 9) = "0000" then
                            z_fa <= to_unsigned(64, 20); z_fb <= to_unsigned(64, 18);
                        else
                            z_fa <= resize(s_g3(12 downto 3), 20); z_fb <= resize(s_g3(12 downto 3), 18);
                        end if;
                    when 5  => z_fa <= resize(s_g5(12 downto 3), 20); z_fb <= resize(s_g5(12 downto 3), 18);   -- k5^2
                    when 6  => z_fa <= c_k5sq;                         z_fb <= to_unsigned(32833, 18);         -- du_h
                    when 7  => z_fa <= resize(c_duh, 20);              z_fb <= resize(s_xg, 18);              -- glyph x origin
                    when 8  => z_fa <= resize(c_duh, 20);              z_fb <= to_unsigned(63339, 18);         -- dv per virtual line
                    when 9  => z_fa <= resize(c_dvv, 20);              z_fb <= resize(s_ys, 18);              -- dv per line
                    when 10 => z_fa <= resize(c_dvv, 20);              z_fb <= to_unsigned(136115, 18) - resize(s_ys(11 downto 1), 18);  -- glyph y origin
                    when 11 => z_fa <= resize(s_vm, 20);               z_fb <= to_unsigned(141, 18);           -- drift step
                    when 12 => z_fa <= resize(s_vm, 20);               z_fb <= to_unsigned(148, 18);           -- scroll step
                    when 13 => z_fa <= resize(c_is, 20);               z_fb <= to_unsigned(54917, 18);         -- 1 / (A_R s)
                    when 14 => z_fa <= resize(c_is, 20);               z_fb <= to_unsigned(53722, 18);         -- 1 / (A_L s)
                    when 15 => z_fa <= resize(c_s, 20);                z_fb <= to_unsigned(38295, 18);         -- s / BN
                    when others => z_fa <= resize(s_g2(12 downto 3), 20); z_fb <= to_unsigned(90, 18);         -- z0
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 38); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(36 downto 0) & '0';
                f_b <= '0' & f_b(17 downto 1);
            end if;

            -- serial divider (restoring, one dividend bit per clock, 35 clocks)
            d_ld <= '0';
            if d_ld = '1' then
                d_rem <= (others => '0'); d_q <= (others => '0'); d_i <= (others => '0'); d_run <= '1';
            elsif d_run = '1' then
                v_r := d_rem(19 downto 0) & d_num(34);
                if v_r >= ('0' & d_den) then
                    d_rem <= v_r - ('0' & d_den); d_q <= d_q(23 downto 0) & '1';
                else
                    d_rem <= v_r; d_q <= d_q(23 downto 0) & '0';
                end if;
                d_num <= d_num(33 downto 0) & '0';
                if d_i = 34 then d_run <= '0'; end if;
                d_i <= d_i + 1;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's steps)
                    when 20 =>
                        if s_vneg = '1' then c_dr <= c_dr - resize(c_drs, 24); c_sc <= c_sc - resize(c_scs, 24);
                        else c_dr <= c_dr + resize(c_drs, 24); c_sc <= c_sc + resize(c_scs, 24); end if;
                        c_g6 <= s_g6(12 downto 5);
                    -- products
                    when 64  => c_k1sq <= f_acc(19 downto 0);
                    when 96  => c_inch <= to_unsigned(3277, 15) + f_acc(34 downto 20);
                    when 97  =>
                        case to_integer(s_xs) is
                            when 4      => c_incp <= shift_left(resize(c_inch, 17), 2);
                            when 3      => c_incp <= shift_left(resize(c_inch, 17), 1) + resize(c_inch, 17);
                            when others => c_incp <= shift_left(resize(c_inch, 17), 1);
                        end case;
                    when 128 => c_po <= f_acc(25 downto 0);
                    when 160 =>
                        if z_neg = '1' then c_s <= to_unsigned(4096, 13) + f_acc(22 downto 10);
                        else c_s <= to_unsigned(1638, 13) + f_acc(22 downto 10); end if;
                    when 192 => c_k3sq <= f_acc(19 downto 0);
                    when 224 => c_k5sq <= f_acc(19 downto 0);
                    when 256 => c_duh <= to_unsigned(6554, 16) + f_acc(35 downto 20);
                    when 257 =>
                        case to_integer(s_xs) is
                            when 4      => c_dup <= shift_left(resize(c_duh, 18), 2);
                            when 3      => c_dup <= shift_left(resize(c_duh, 18), 1) + resize(c_duh, 18);
                            when others => c_dup <= shift_left(resize(c_duh, 18), 1);
                        end case;
                    when 288 => c_uo  <= f_acc(26 downto 0);
                    when 320 => c_dvv <= f_acc(31 downto 14);
                    when 352 => c_dvl <= f_acc(27 downto 8);
                    when 384 => c_vo  <= f_acc(34 downto 8);
                    when 416 => c_drs <= f_acc(16 downto 0);
                    when 448 => c_scs <= f_acc(16 downto 0);
                    when 480 => c_iar <= f_acc(33 downto 16);
                    when 512 => c_ial <= f_acc(33 downto 16);
                    when 544 => c_sb  <= f_acc(28 downto 13);
                    when 576 => c_z0  <= f_acc(16 downto 0);
                    -- divider A: 1 / s (s ready at 160); B: 4 / kz (k3^2 ready at 192)
                    when 200 =>
                        d_num <= to_unsigned(2 ** 28, 35); d_den <= resize(c_s, 20); d_ld <= '1'; d_job <= '0';
                    when 250 => c_is <= d_q(17 downto 0);
                    when 260 =>
                        d_num <= "10010110110011001011110010000000000"; d_den <= c_k3sq; d_ld <= '1'; d_job <= '1';   -- 20240000000
                    when 310 => c_ikz <= d_q(22 downto 0);
                    -- origins
                    when 600 =>
                        c_phi0 <= to_unsigned(2 ** 23, 24) - c_dr - c_po(23 downto 0);
                        v_t := to_unsigned(C_UANC, 24) - c_uo(23 downto 0);
                        c_u0 <= v_t + c_sc;
                        c_v0 <= to_unsigned(C_VANC, 24) - c_vo(23 downto 0);
                        if s_g3(12 downto 9) = "0000" then s_flat <= '1'; else s_flat <= '0'; end if;
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
    -- Source side: oscillator lattice (two triangles, drifting)
    ---------------------------------------------------------------------------
    p_src : process(clk)
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + resize(s_oxs, 16);
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                    -- ot * 128
                s_oy <= s_oy + resize(s_oys, 16);
            end if;
            if s_zone = '1' then s_oy <= to_unsigned(0, 16) - shift_left(resize(s_ot, 16), 6); end if;   -- -ot * 64
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            s_osum <= resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line's wedge thresholds, band limit and glyph row on one
    -- serial multiplier (18 iterations, result at load + 19)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d  : signed(20 downto 0);
        variable v_t  : unsigned(20 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 700 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= resize(s_ys(11 downto 1), 20);
                e_vg <= c_v0;
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 20);
                e_vg <= e_vg + resize(c_dvl, 24);
                e_lc <= (others => '0');
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(39 downto 0) & '0';
            m_b <= '0' & m_b(17 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    v_d := signed(resize(e_vy, 21)) - C_CY8;
                    if v_d < 0 then v_d := -v_d; end if;
                    e_ty <= unsigned(v_d(17 downto 0));
                    v_d := signed(resize(e_vy, 21)) - C_CYZ8;
                    if v_d < 0 then v_d := -v_d; end if;
                    e_tz <= unsigned(v_d(17 downto 0));
                    if e_vg(23 downto 12) < 510 then e_row <= "00";
                    elsif e_vg(23 downto 12) < 1021 then e_row <= "01";
                    elsif e_vg(23 downto 12) < 1540 then e_row <= "10";
                    else e_row <= "11"; end if;
                when 2 =>
                    e_dz <= signed(resize(e_tz, 19)) - signed(resize(c_z0, 19));
                    if e_ty < C_V08 then e_tvz <= '1'; else e_tvz <= '0'; end if;
                    m_a <= resize(c_iar, 41); m_b <= e_ty; m_acc <= (others => '0');
                    -- glyph runs {s1, e1, s2, e2} (Q12 of the pitch, +64) for the row (S9 Flip: mirrored)
                    case to_integer(s_flip & e_row) is
                        when 0 => n_s1 <= to_unsigned(64, 12);   n_e1 <= to_unsigned(1067, 12); n_s2 <= to_unsigned(1683, 12); n_e2 <= to_unsigned(2648, 12);
                        when 1 => n_s1 <= to_unsigned(1014, 12); n_e1 <= to_unsigned(2659, 12); n_s2 <= to_unsigned(4095, 12); n_e2 <= to_unsigned(4095, 12);
                        when 2 => n_s1 <= to_unsigned(64, 12);   n_e1 <= to_unsigned(2681, 12); n_s2 <= to_unsigned(4095, 12); n_e2 <= to_unsigned(4095, 12);
                        when 4 => n_s1 <= to_unsigned(97, 12);   n_e1 <= to_unsigned(1062, 12); n_s2 <= to_unsigned(1678, 12); n_e2 <= to_unsigned(2681, 12);
                        when 5 => n_s1 <= to_unsigned(86, 12);   n_e1 <= to_unsigned(1731, 12); n_s2 <= to_unsigned(4095, 12); n_e2 <= to_unsigned(4095, 12);
                        when 6 => n_s1 <= to_unsigned(64, 12);   n_e1 <= to_unsigned(2681, 12); n_s2 <= to_unsigned(4095, 12); n_e2 <= to_unsigned(4095, 12);
                        when others => n_s1 <= to_unsigned(4095, 12); n_e1 <= to_unsigned(4095, 12); n_s2 <= to_unsigned(4095, 12); n_e2 <= to_unsigned(4095, 12);
                    end case;
                    if e_row = "11" then n_rok <= '0'; else n_rok <= '1'; end if;
                when 3 =>
                    n_midl <= e_dz(18);
                when 21 =>
                    v_t := m_acc(40 downto 20);
                    if v_t > 3662 then n_thr <= to_unsigned(4095, 12); else n_thr <= resize(v_t + C_H0N, 12); end if;
                    m_a <= resize(c_ial, 41); m_b <= e_ty; m_acc <= (others => '0');
                when 40 =>
                    v_t := m_acc(40 downto 20);
                    if v_t > 3662 then n_thl <= to_unsigned(4095, 12); else n_thl <= resize(v_t + C_H0N, 12); end if;
                    m_a <= resize(c_sb, 41); m_acc <= (others => '0');
                    if e_tvz = '1' then m_b <= (others => '0'); else m_b <= e_ty - C_V08; end if;
                when 59 =>
                    v_t := m_acc(40 downto 20);
                    if e_tvz = '1' then n_tv <= (others => '0');
                    elsif v_t > 4095 then n_tv <= to_unsigned(4095, 12);
                    else n_tv <= v_t(11 downto 0); end if;
                    m_a <= resize(c_ikz, 41); m_acc <= (others => '0');
                    if e_dz(18) = '1' then m_b <= (others => '0'); else m_b <= resize(unsigned(e_dz(17 downto 2)), 18); end if;
                when 78 =>
                    if s_flat = '1' or m_acc(40 downto 30) /= 0 then n_r2 <= (others => '1');
                    else n_r2 <= m_acc(29 downto 6); end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: X phase (held left of the line-start hold point), band parabola
    -- S = (x - 42.5)^2 in h^2 exactly (S += D; D += 2k^2), glyph x phase
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_xh  <= resize(s_xh0, 12);
                b_x1  <= resize(s_xh0, 12) + s_xs;
                b_phi <= c_phi0;
                b_u   <= c_u0;
                b_s   <= resize(s_s0, 24);
                b_d   <= resize(s_d0, 17);
                l_thr <= n_thr; l_thl <= n_thl; l_tv <= n_tv;
                l_midl <= n_midl; l_r2 <= n_r2;
                l_s1 <= n_s1; l_e1 <= n_e1; l_s2 <= n_s2; l_e2 <= n_e2; l_rok <= n_rok;
            elsif g_avid = '1' then
                b_xh <= b_x1;
                b_x1 <= b_x1 + s_xs;
                if b_x1 > C_XH then b_phi <= b_phi + c_incp; end if;
                b_s <= unsigned(signed(b_s) + resize(b_d, 24));
                b_d <= b_d + signed(resize(s_dd, 17));
                b_u <= b_u + resize(c_dup, 24);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage k register = the beam pixel of k clocks before)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_d   : unsigned(9 downto 0);
        variable v_l   : unsigned(9 downto 0);
        variable v_os  : signed(9 downto 0);
        variable v_ur  : unsigned(11 downto 0);
        variable v_th  : unsigned(11 downto 0);
        variable v_k   : std_logic;
        variable v_c   : unsigned(1 downto 0);
        variable v_g   : unsigned(11 downto 0);
        variable v_n   : unsigned(7 downto 0);
        variable v_gs  : signed(9 downto 0);
        variable v_cnt : unsigned(7 downto 0);
        variable v_sg  : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: video tap (luma -> 0..255, a few px early: invisible in a bend); band compare;
            --    glyph phase +- dilation
            if unsigned(data_in.y) > 64 then v_d := unsigned(data_in.y) - 64; else v_d := (others => '0'); end if;
            v_l := shift_right(v_d, 2) + shift_right(v_d, 4);
            if v_l > 255 then p1_l8 <= (others => '1'); else p1_l8 <= v_l(7 downto 0); end if;
            p1_os <= signed(resize(s_osum, 9)) - 127;
            p1_phi <= b_phi;
            if b_s > l_r2 then p1_zs <= '1'; else p1_zs <= '0'; end if;
            if b_xh >= C_XZ then p1_zx <= '1'; else p1_zx <= '0'; end if;
            v_ur := b_u(23 downto 12);
            p1_ur  <= v_ur;
            p1_urp <= resize(v_ur, 13) + C_DIL;
            p1_urm <= signed(resize(v_ur, 13)) - C_DIL;

            -- 2: source (bend input, grey target); glyph masks; band
            if s_src = '0' then
                p2_bin <= signed(resize(p1_l8, 10));
                p2_src <= to_unsigned(40, 8) + resize(p1_l8(7 downto 1), 8) + resize(p1_l8(7 downto 3), 8) + resize(p1_l8(7 downto 4), 8);
            else
                v_os := resize(p1_os, 10);
                p2_bin <= shift_right(v_os, 2);
                v_gs := to_signed(158, 10) + shift_right(v_os, 2);
                p2_src <= unsigned(v_gs(7 downto 0));
            end if;
            if l_rok = '1' and (((p1_ur >= l_s1) and (p1_ur < l_e1)) or ((p1_ur >= l_s2) and (p1_ur < l_e2))) then
                p2_gw <= '1'; else p2_gw <= '0'; end if;
            if l_rok = '1' and (((p1_urp >= l_s1) and (p1_urm < signed(resize(l_e1, 13)))) or
                                ((p1_urp >= l_s2) and (p1_urm < signed(resize(l_e2, 13))))) then
                p2_gy <= '1'; else p2_gy <= '0'; end if;
            if l_rok = '1' and (((p1_urm >= signed(resize(l_s1, 13))) and (p1_urp < l_e1)) or
                                ((p1_urm >= signed(resize(l_s2, 13))) and (p1_urp < l_e2))) then
                p2_gc <= '1'; else p2_gc <= '0'; end if;
            p2_mid <= l_midl or (p1_zx and p1_zs);
            p2_phi <= p1_phi;

            -- 3: bend partial products (bend input x gain nibbles)
            p3_pl <= resize(p2_bin * signed('0' & c_g6(3 downto 0)), 15);
            p3_ph <= resize(p2_bin * signed('0' & c_g6(7 downto 4)), 15);
            p3_phi <= p2_phi;
            -- 4: bend product
            p4_bp <= resize(p3_pl, 19) + shift_left(resize(p3_ph, 19), 4);
            p4_phi <= p3_phi;
            -- 5: bent X phase
            p5_phib <= p4_phi + unsigned(shift_left(resize(p4_bp, 24), 6));
            -- 6: fold: Txn = 0 at the X centres, 1 midway; right of a centre = first half
            if p5_phib(23) = '1' then p6_txn <= not p5_phib(22 downto 11); p6_rt <= '0';
            else p6_txn <= p5_phib(22 downto 11); p6_rt <= '1'; end if;
            -- 7: wedge key
            if p6_rt = '1' then v_th := l_thr; else v_th := l_thl; end if;
            if p6_txn > v_th or p6_txn < l_tv then p7_k <= '1'; else p7_k <= '0'; end if;

            -- delay lines for source and masks
            p_sv(3) <= p2_src;
            for i in 4 to 8 loop p_sv(i) <= p_sv(i - 1); end loop;
            p_gw(3) <= p2_gw; p_gy(3) <= p2_gy; p_gc(3) <= p2_gc; p_mid(3) <= p2_mid;
            for i in 4 to 7 loop
                p_gw(i) <= p_gw(i - 1); p_gy(i) <= p_gy(i - 1); p_gc(i) <= p_gc(i - 1); p_mid(i) <= p_mid(i - 1);
            end loop;

            -- 8: ground class (0 white, 1 yellow, 2 cyan; swap), ink flags; slew input
            if p_mid(7) = '1' then
                if p7_k = '1' then v_c := "10"; else v_c := "00"; end if;
            else
                if p7_k = '1' then v_c := "00"; else v_c := "01"; end if;
            end if;
            if s_swap = '1' and v_c /= "00" then v_c := not v_c; end if;
            p8_cls <= v_c;
            if v_c = "00" and p_gw(7) = '1' and s_gav_sr(6) = '1' then p8_wg <= '1'; else p8_wg <= '0'; end if;
            if v_c = "01" and p_gy(7) = '1' then p8_red <= '1'; else p8_red <= '0'; end if;
            if v_c = "10" and p_gc(7) = '1' then p8_blu <= '1'; else p8_blu <= '0'; end if;
            if s_hd = '1' then p8_sa <= resize(p_sv(7)(7 downto 1), 12);  -- src * 16 >> 5
            else p8_sa <= resize(p_sv(7), 12); end if;                   -- src * 16 >> 4

            -- 9: one-shot (black for 66 h from each white-glyph run start) and the RC slew
            p9_cls <= p8_cls; p9_red <= p8_red; p9_blu <= p8_blu; p9_wg <= p8_wg;
            p9_g <= x_g(11 downto 4);
            if p8_wg = '1' and s_shot = '1' and x_cnt < 66 then p9_blk <= '1'; else p9_blk <= '0'; end if;
            if p8_wg = '0' then
                x_cnt <= (others => '0');
                if s_shot = '1' then x_g <= to_unsigned(1520, 12); else x_g <= shift_left(resize(p_sv(8), 12), 4); end if;
            else
                v_cnt := resize(x_cnt, 8) + s_xs;
                if v_cnt > 127 then x_cnt <= (others => '1'); else x_cnt <= v_cnt(6 downto 0); end if;
                if s_shot = '1' and x_cnt < 66 then
                    x_g <= to_unsigned(1520, 12);
                else
                    if s_hd = '1' then v_g := x_g - shift_right(x_g, 5); else v_g := x_g - shift_right(x_g, 4); end if;
                    x_g <= v_g + p8_sa;
                end if;
            end if;
            -- streak noise: a new value every 64 h (32 virtual px)
            v_sg := x_seg + s_xs;
            x_seg <= '0' & v_sg(5 downto 0);
            if v_sg(6) = '1' then
                x_lf <= x_lf(14 downto 0) & (x_lf(15) xor x_lf(13) xor x_lf(12) xor x_lf(10));
                x_nz <= signed('0' & x_lf(2 downto 0)) - 4;
            end if;

            -- 10: colour code (0 W, 1 Y, 2 C, 3 R, 4 B, 5 K, 6 grey); grey + noise
            if p9_blk = '1' then p10_code <= "101";
            elsif p9_wg = '1' then p10_code <= "110";
            elsif p9_red = '1' then p10_code <= "011";
            elsif p9_blu = '1' then p10_code <= "100";
            else p10_code <= resize(p9_cls, 3); end if;
            v_gs := signed(resize(p9_g, 10)) + resize(x_nz, 10);
            if v_gs < 0 then p10_g <= (others => '0');
            elsif v_gs > 205 then p10_g <= to_unsigned(205, 8);
            else p10_g <= unsigned(v_gs(7 downto 0)); end if;

            -- 11: colour {Y, U = Cr, V = Cb}; luma pre-scaled so the shell's 13/16 restores the photo
            case to_integer(p10_code) is
                when 0 => s_y11 <= to_unsigned(940, 10); s_u11 <= to_unsigned(511, 10); s_v11 <= to_unsigned(499, 10);
                when 1 => s_y11 <= to_unsigned(865, 10); s_u11 <= to_unsigned(644, 10); s_v11 <= to_unsigned(121, 10);
                when 2 => s_y11 <= to_unsigned(785, 10); s_u11 <= to_unsigned(76, 10);  s_v11 <= to_unsigned(649, 10);
                when 3 => s_y11 <= to_unsigned(397, 10); s_u11 <= to_unsigned(939, 10); s_v11 <= to_unsigned(358, 10);
                when 4 => s_y11 <= to_unsigned(405, 10); s_u11 <= to_unsigned(325, 10); s_v11 <= to_unsigned(844, 10);
                when 5 => s_y11 <= to_unsigned(170, 10); s_u11 <= to_unsigned(525, 10); s_v11 <= to_unsigned(516, 10);
                when others =>
                    s_y11 <= to_unsigned(64, 10) + shift_left(resize(p10_g, 10), 2) + resize(p10_g(7 downto 2), 10);
                    s_u11 <= to_unsigned(512, 10); s_v11 <= to_unsigned(512, 10);
            end case;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Soft box (the photo's horizontal softness): [1 2 .. 2 1] / 2N on Y, U, V,
    -- N = 16 at 1080, 8 at 720p / SD (S10 Hard bypasses it); then the output
    -- conditioning: chroma [1 2 2 2 2 2 2 2 1] / 16 (4-tap pair at 720p / SD),
    -- luma [1 2 1], luma 13/16 and the blanking gate.  Ring inputs are forced
    -- neutral outside the generated avid and the sums re-seeded in the zone.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(15 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_iy, v_iu, v_iv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_wp <= s_wp + 1;
            if s_hd = '1' then sr_ra <= s_wp - 13; else sr_ra <= s_wp - 5; end if;
            if s_short = '1' then cr_ra <= s_wp - 1; else cr_ra <= s_wp - 5; end if;

            -- 12: soft ring input
            if s_zone = '1' or s_gav_sr(10) = '0' then
                v_iy := to_unsigned(64, 10); v_iu := to_unsigned(512, 10); v_iv := to_unsigned(512, 10);
            else
                v_iy := s_y11; v_iu := s_u11; v_iv := s_v11;
            end if;
            sr_wd <= std_logic_vector(v_iy) & std_logic_vector(v_iu) & std_logic_vector(v_iv) & "00";
            -- 13: box sums
            if s_zone = '1' and s_zc = 40 then
                if s_hd = '1' then
                    s_sy <= to_unsigned(1024, 15); s_su <= to_unsigned(8192, 15); s_sv <= to_unsigned(8192, 15);
                else
                    s_sy <= to_unsigned(512, 15); s_su <= to_unsigned(4096, 15); s_sv <= to_unsigned(4096, 15);
                end if;
            else
                s_sy <= s_sy + unsigned(sr_wd(31 downto 22)) - unsigned(sr_q2(31 downto 22));
                s_su <= s_su + unsigned(sr_wd(21 downto 12)) - unsigned(sr_q2(21 downto 12));
                s_sv <= s_sv + unsigned(sr_wd(11 downto 2)) - unsigned(sr_q2(11 downto 2));
            end if;
            s_hw1 <= sr_wd(31 downto 2);
            -- 14: box pair (or the hard bypass)
            s_syd <= s_sy; s_sud <= s_su; s_svd <= s_sv;
            if s_hard = '1' then
                s_y14 <= unsigned(s_hw1(29 downto 20)); s_u14 <= unsigned(s_hw1(19 downto 10)); s_v14 <= unsigned(s_hw1(9 downto 0));
            elsif s_hd = '1' then
                v_sc := resize(s_sy, 16) + s_syd; s_y14 <= v_sc(14 downto 5);
                v_sc := resize(s_su, 16) + s_sud; s_u14 <= v_sc(14 downto 5);
                v_sc := resize(s_sv, 16) + s_svd; s_v14 <= v_sc(14 downto 5);
            else
                v_sc := resize(s_sy, 16) + s_syd; s_y14 <= v_sc(13 downto 4);
                v_sc := resize(s_su, 16) + s_sud; s_u14 <= v_sc(13 downto 4);
                v_sc := resize(s_sv, 16) + s_svd; s_v14 <= v_sc(13 downto 4);
            end if;

            -- 15: chroma ring input; luma chain
            if s_zone = '1' or s_gav_sr(13) = '0' then
                cr_wd <= std_logic_vector(to_unsigned(512, 10)) & std_logic_vector(to_unsigned(512, 10));
                s_ly(0) <= to_unsigned(64, 10);
            else
                cr_wd <= std_logic_vector(s_u14) & std_logic_vector(s_v14);
                s_ly(0) <= s_y14;
            end if;
            for k in 1 to 6 loop s_ly(k) <= s_ly(k - 1); end loop;
            -- 16: chroma box sums
            if s_zone = '1' and s_zc = 40 then
                if s_short = '1' then s_usum <= to_unsigned(2048, 14); s_vsum <= to_unsigned(2048, 14);
                else s_usum <= to_unsigned(4096, 14); s_vsum <= to_unsigned(4096, 14); end if;
            else
                s_usum <= s_usum + unsigned(cr_wd(19 downto 10)) - unsigned(cr_q2(19 downto 10));
                s_vsum <= s_vsum + unsigned(cr_wd(9 downto 0)) - unsigned(cr_q2(9 downto 0));
            end if;
            -- 17: chroma box pair; luma [1 2 1] centred on the same pixel
            s_ud <= s_usum; s_vd <= s_vsum;
            v_sc := resize(s_usum, 16) + s_ud;
            if s_short = '1' then s_u17 <= v_sc(12 downto 3); else s_u17 <= v_sc(13 downto 4); end if;
            v_sc := resize(s_vsum, 16) + s_vd;
            if s_short = '1' then s_v17 <= v_sc(12 downto 3); else s_v17 <= v_sc(13 downto 4); end if;
            if s_short = '1' then
                v_sy := resize(s_ly(2), 12) + shift_left(resize(s_ly(3), 12), 1) + resize(s_ly(4), 12);
            else
                v_sy := resize(s_ly(4), 12) + shift_left(resize(s_ly(5), 12), 1) + resize(s_ly(6), 12);
            end if;
            s_y17 <= v_sy(11 downto 2);
            -- 18: luma toward black (white 940 -> 777); blanking gate
            if s_y17 > 64 then v_dl := s_y17 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(17) = '1' then
                s_y18 <= s_y17 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u18 <= s_u17; s_v18 <= s_v17;
            else
                s_y18 <= to_unsigned(64, 10); s_u18 <= to_unsigned(512, 10); s_v18 <= to_unsigned(512, 10);
            end if;
            -- 19, 20
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture bunting;
