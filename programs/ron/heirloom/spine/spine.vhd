-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Spine v0.1.0 -- Heirloom phase 3 (video clips), from the clip WIN_20230906_00_33_30 at 11.0 s.
--
-- The still: a magenta -> plum vertical gradient cut by a diagonal band; outside the band, a purple "O X O X"
-- lattice and green wavy lines (the contour rings of an H + V sine sum).  The band's two edges are staircases of
-- hooked teeth (the spine): pointed teeth, round bays, each tooth's left flank lit by a bright magenta rim and a
-- slewed dark-violet shadow trailing right of it; where the band narrows the bays close into vertebra holes.
-- In the clip the whole patch rolls up ~30 lines/s, its oscillators at slightly different rates.
--
-- The patch, as fitted (spine.py has every constant):
--   key   K = tri(x - xc(y)) - hw + Ah hk(x) + Av cos(2 pi (y - 378) / 700) (+ etch)  > 0 -> texture
--         tri = |.| folded every 4096 px, xc = 325 + s (y - 540), hk = round arches meeting in pointed teeth
--         (period 230 px)
--   rings r = nt (cos(2 pi (x - 26) / 230) + 9 cos(2 pi (y - 155) / 1250)) (+ etch): a two-ring colouriser,
--         purple on the O rows, green between
--   bg    a vertical palette over a V wave of 1100 lines
--   rim   a magenta one-shot over the 22 px before each texture edge (the key is evaluated 28 px ahead);
--         shadow: a dark-violet slew after each texture edge, its length jittered per line
--
--   K1  Teeth       the H oscillator: teeth / vertebrae and lattice O's across, x1/4 .. x4 (512 = the photo)
--   K2  Band        the band's width: narrow (a chain of vertebra holes) .. the photo (512) .. wide open
--   K3  Slant       the band's angle: tilted back .. vertical .. the photo's 1.25 (512) .. nearly flat
--   K4  Speed       bipolar roll: up (+, 700 = the clip's ~30 lines/s) / down (-), +-8 deadband
--   K5  Rings       the texture's ring density, x1/4 .. x4
--   K6  Etch        how far the camera (or the Osc) bends the band edges and the rings
--   S7  Mirror      the picture mirrored about the screen centre (two spines meeting)
--   S8  Palette     Magenta (the photo) / Verdant (chroma complement: green field, magenta lattice)
--   S9  Invert      the key inverted: texture inside the band, the gradient outside
--   S10 Tear        band tears: 72-line bands shifted +-128 px, redrawn every 0.75 s
--   S11 Source      Video / Osc
--   P12 VERTEBRAE   the teeth's depth: a clean band (0) .. the photo (410) .. a field of rimmed holes (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line
-- start (computes the NEXT line: three V cosines, the background colour, the band start, the H phase), a
-- 20-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3), line-paired like the clip.
-- Generated avid; output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the running-sum
-- resync, luma 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.spine_pkg.all;
use work.all;

architecture spine of program_top is
    constant C_LAT : integer := 48;   -- = processing_delay_clks (20 + the key's 28-pixel head start)
    constant C_D   : integer := 28;   -- key look-ahead (pixels): the key beam starts C_D clocks before the colour beam

    constant C_KX0  : integer := 36472;     -- H phase per h (half virtual px) at K1 = 512: 2^24 / 460
    constant C_KYK  : integer := 23967;     -- key V phase per virtual line: 2^24 / 700
    constant C_KYT  : integer := 13422;     -- ring V phase per virtual line: 2^24 / 1250
    constant C_KYB  : integer := 15252;     -- background phase per virtual line: 2^24 / 1100

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

    -- line buffer: 4-pixel box of (luma >> 2) - 16
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    attribute no_rw_check : boolean;
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    signal s_l0, s_l1, s_l2, s_l3 : unsigned(7 downto 0) := (others => '0');
    signal s_lav   : std_logic := '0';

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_cs_a : unsigned(9 downto 0) := (others => '0');
    signal r_cs_q : signed(15 downto 0) := (others => '0');
    signal r_bg_a : unsigned(7 downto 0) := (others => '0');
    signal r_bg_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_hk_a : unsigned(7 downto 0) := (others => '0');
    signal r_hk_q : signed(7 downto 0) := (others => '0');
    signal r_ch_a : unsigned(7 downto 0) := (others => '0');
    signal r_ch_q : signed(7 downto 0) := (others => '0');
    signal r_tx_a : unsigned(8 downto 0) := (others => '0');
    signal r_tx_q : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(410 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_mir, s_pal, s_inv, s_tear, s_src : std_logic := '0';
    signal s_k6 : unsigned(9 downto 0) := to_unsigned(300, 10);

    -- per-field constants
    signal z_e    : signed(10 downto 0) := (others => '0');
    signal z_sh1, z_sh5 : signed(2 downto 0) := (others => '0');
    signal z_mt1, z_mt5 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_phi  : std_logic := '0';
    signal z_bhi  : std_logic := '0';
    signal z_sp   : unsigned(17 downto 0) := (others => '0');
    signal z_lst  : unsigned(17 downto 0) := (others => '0');
    signal z_tf   : unsigned(5 downto 0) := (others => '0');
    signal z_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal c_kx   : unsigned(17 downto 0) := to_unsigned(C_KX0, 18);   -- H phase per h (Q24 cycles)
    signal c_pstep : unsigned(19 downto 0) := to_unsigned(2 * C_KX0, 20);
    signal c_nt   : unsigned(9 downto 0) := to_unsigned(246, 10);      -- ring density (Q8 rings per unit cos)
    signal c_at   : unsigned(13 downto 0) := to_unsigned(2214, 14);    -- ring V amplitude (Q8 rings)
    signal c_ah   : unsigned(10 downto 0) := to_unsigned(256, 11);     -- tooth depth (h per 64 hk units)
    signal c_av   : unsigned(9 downto 0) := to_unsigned(320, 10);      -- key V amplitude (h)
    signal c_hw   : unsigned(11 downto 0) := to_unsigned(1290, 12);    -- band half width (h)
    signal c_sq   : signed(10 downto 0) := to_signed(320, 11);         -- band slope (Q8)
    signal c_srk, c_srt, c_srb : unsigned(16 downto 0) := (others => '0');
    signal c_bst  : unsigned(12 downto 0) := (others => '0');          -- band roll per field (Q8 h)
    signal c_rk   : unsigned(23 downto 0) := to_unsigned(7717690, 24); -- roll phases (Q24)
    signal c_rt   : unsigned(23 downto 0) := to_unsigned(14696806, 24);
    signal c_rb   : unsigned(23 downto 0) := to_unsigned(13574304, 24);
    signal c_bb   : unsigned(20 downto 0) := (others => '0');          -- band roll (Q8 h, mod 8192 h)
    signal c_tseed : unsigned(15 downto 0) := x"5A3C";
    signal c_nc   : unsigned(10 downto 0) := to_unsigned(960, 11);     -- Mirror: the centre pixel

    -- field serial multiplier (20-bit b operand, 20 iterations)
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(19 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
    signal z_fb  : unsigned(19 downto 0) := (others => '0');
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
    signal e_tb   : unsigned(14 downto 0) := (others => '0');
    signal e_tl   : unsigned(15 downto 0) := x"5A3C";
    signal e_jl   : unsigned(15 downto 0) := x"1D87";
    signal e_th   : signed(9 downto 0) := (others => '0');
    signal e_vyq  : unsigned(10 downto 0) := (others => '0');
    signal e_ph   : unsigned(23 downto 0) := (others => '0');
    signal e_ck, e_ct : signed(15 downto 0) := (others => '0');
    signal e_vb   : signed(11 downto 0) := (others => '0');
    signal e_dx   : signed(11 downto 0) := (others => '0');
    signal m_a    : unsigned(33 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(33 downto 0) := (others => '0');
    -- next line's values (loaded with the line at g_start)
    signal n_dh0  : unsigned(12 downto 0) := (others => '0');
    signal n_pc0  : unsigned(23 downto 0) := (others => '0');
    signal n_cl   : signed(13 downto 0) := (others => '0');
    signal n_vt   : unsigned(8 downto 0) := (others => '0');
    signal n_zn   : unsigned(2 downto 0) := (others => '0');
    signal n_bgy, n_bgu, n_bgv : unsigned(9 downto 0) := (others => '0');
    signal n_dsy, n_dsu, n_dsv : signed(10 downto 0) := (others => '0');
    signal n_jit  : signed(5 downto 0) := (others => '0');

    -- beam
    signal b_dh  : unsigned(12 downto 0) := (others => '0');
    signal b_pk, b_pc : unsigned(23 downto 0) := (others => '0');
    signal b_px, b_cpx : unsigned(10 downto 0) := (others => '0');
    signal g_cdc : unsigned(4 downto 0) := (others => '0');
    signal b_back, b_kback : std_logic := '0';
    signal l_cl  : signed(13 downto 0) := (others => '0');
    signal l_vt  : unsigned(8 downto 0) := (others => '0');
    signal l_zn  : unsigned(2 downto 0) := (others => '0');
    signal l_bgy, l_bgu, l_bgv : unsigned(9 downto 0) := (others => '0');
    signal l_dsy, l_dsu, l_dsv : signed(10 downto 0) := (others => '0');
    signal l_jit : signed(5 downto 0) := (others => '0');

    -- pixel path
    signal p2_tri, p3_tri : unsigned(12 downto 0) := (others => '0');
    signal p4_hk  : signed(7 downto 0) := (others => '0');
    signal p4_ch  : signed(7 downto 0) := (others => '0');
    signal p4_s   : unsigned(7 downto 0) := (others => '0');
    signal p4_a, p5_a : signed(14 downto 0) := (others => '0');
    signal p5_kh  : signed(19 downto 0) := (others => '0');
    signal p5_rc  : signed(18 downto 0) := (others => '0');
    signal p5_e   : unsigned(17 downto 0) := (others => '0');
    signal p6_k   : signed(15 downto 0) := (others => '0');
    signal p6_r   : unsigned(8 downto 0) := (others => '0');
    signal s_sr   : std_logic_vector(C_D downto 0) := (others => '0');
    signal p8_key, p9_key, p10_key, p11_key : std_logic := '0';
    signal p8_n   : unsigned(4 downto 0) := (others => '0');
    signal p8_cnt : unsigned(7 downto 0) := (others => '1');
    signal p9_nh  : unsigned(6 downto 0) := (others => '0');
    signal p9_cj  : unsigned(5 downto 0) := (others => '1');
    signal p11_tx : std_logic_vector(31 downto 0) := (others => '0');
    signal p7_ta, p8_ta : unsigned(8 downto 0) := (others => '0');
    signal p10_as : unsigned(4 downto 0) := (others => '0');
    signal p10_rim, p11_rim : std_logic := '0';
    signal p11_my, p11_mu, p11_mv : signed(15 downto 0) := (others => '0');
    signal s_y12, s_u12, s_v12, s_y13, s_u13, s_v13, s_y14, s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: the boxes' 8-back taps come from block-RAM rings (read 6 back + two registers)
    type t_ring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal r_ra, r_rb : t_ring := (others => x"10200800");
    attribute no_rw_check of r_ra : signal is true;
    attribute no_rw_check of r_rb : signal is true;
    signal r_wp  : unsigned(7 downto 0) := (others => '0');
    signal r_aq, r_aq2, r_bq, r_bq2 : std_logic_vector(31 downto 0) := x"10200800";
    -- sync ring (hsync, vsync, field): read C_LAT - 2 back + two registers = C_LAT clocks
    type t_sring is array (0 to 255) of std_logic_vector(3 downto 0);
    signal r_rs  : t_sring := (others => (others => '1'));
    attribute no_rw_check of r_rs : signal is true;
    signal r_sq, r_sq2 : std_logic_vector(3 downto 0) := (others => '1');
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
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
            r_ex_q <= C_SP_EXP2(to_integer(r_ex_a));
            r_cs_q <= C_SP_COS(to_integer(r_cs_a));
            r_bg_q <= C_SP_BG(to_integer(r_bg_a));
            r_hk_q <= C_SP_HK(to_integer(r_hk_a));
            r_ch_q <= C_SP_CH(to_integer(r_ch_a));
            r_tx_q <= C_SP_TEX(to_integer(r_tx_a));
        end if;
    end process p_rom;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            r_wp <= r_wp + 1;
            r_rs(to_integer(r_wp)) <= '1' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            r_sq <= r_rs(to_integer(r_wp - (C_LAT - 2)));
            r_sq2 <= r_sq;
            r_ra(to_integer(r_wp)) <= std_logic_vector(s_y15) & std_logic_vector(s_u15) & std_logic_vector(s_v15) & "00";
            r_aq <= r_ra(to_integer(r_wp - 6));
            r_aq2 <= r_aq;
            r_rb(to_integer(r_wp)) <= std_logic_vector(s_y17) & std_logic_vector(s_u17) & std_logic_vector(s_v17) & "00";
            r_bq <= r_rb(to_integer(r_wp - 6));
            r_bq2 <= r_bq;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(10 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_v  : signed(10 downto 0);
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
                s_mir  <= registers_in(6)(0);
                s_pal  <= registers_in(6)(1);
                s_inv  <= registers_in(6)(2);
                s_tear <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(408, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(306, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(204, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 20) * to_unsigned(242, 8), 8), 11);

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

            -- serial products: slot k loads at zc = 72 + 24 k, 20 iterations, result stable from
            -- load + 21 (stores at load + 21).  The slot number is decoded at 70 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 71 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 70  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 94  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 118 => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 142 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 166 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 190 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 214 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 238 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 262 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 286 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(C_KX0, 36);          z_fb <= resize(z_mt1, 20);        -- KX
                    when 1  => z_fa <= to_unsigned(246, 36);            z_fb <= resize(z_mt5, 20);        -- Nt
                    when 2  =>                                                                            -- Ah
                        if z_phi = '1' then
                            z_fa <= resize(s_gp(12 downto 3) - 410, 36); z_fb <= to_unsigned(3422, 20);
                        else
                            z_fa <= resize(s_gp(12 downto 3), 36);       z_fb <= to_unsigned(2558, 20);
                        end if;
                    when 3  =>                                                                            -- hw
                        if z_bhi = '1' then
                            z_fa <= resize(s_g2(12 downto 3) - 512, 36); z_fb <= to_unsigned(1513, 20);
                        else
                            z_fa <= resize(s_g2(12 downto 3), 36);       z_fb <= to_unsigned(990, 20);
                        end if;
                    when 4  => z_fa <= resize(s_vm, 36);                z_fb <= resize(s_vm, 20);         -- speed^2
                    when 5  => z_fa <= resize(z_sp, 36);                z_fb <= to_unsigned(328, 20);     -- key roll
                    when 6  => z_fa <= resize(z_sp, 36);                z_fb <= to_unsigned(226, 20);     -- ring roll
                    when 7  => z_fa <= resize(z_sp, 36);                z_fb <= to_unsigned(305, 20);     -- bg roll
                    when 8  => z_fa <= resize(z_sp, 36);                z_fb <= to_unsigned(690, 20);     -- band lines
                    when others => z_fa <= resize(unsigned(abs(c_sq)), 36);  z_fb <= resize(z_lst, 20);    -- band h
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(19 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 40 =>                       -- teeth: e = k1 - 512 (Q8 octaves)
                        v_e := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh1 <= v_e(10 downto 8);
                    when 42 => z_mt1 <= r_ex_q;
                    when 44 =>                       -- rings: e = k5 - 512
                        v_e := signed(resize(s_g5(12 downto 3), 11)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh5 <= v_e(10 downto 8);
                    when 46 => z_mt5 <= r_ex_q;
                    when 48 =>
                        if s_gp(12 downto 3) > 410 then z_phi <= '1'; else z_phi <= '0'; end if;
                        if s_g2(12 downto 3) >= 512 then z_bhi <= '1'; else z_bhi <= '0'; end if;
                        -- slant: s = 1.25 + 2.25 (k3 - 512) / 512 (Q8: 320 + v + v / 8)
                        v_v := signed(resize(s_g3(12 downto 3), 11)) - 512;
                        c_sq <= to_signed(320, 11) + v_v + shift_right(v_v, 3);
                        s_k6 <= s_g6(12 downto 3);
                    when 93 =>                       -- KX = 36472 mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh1) is
                            when -2     => c_kx <= resize(f_acc(35 downto 17), 18);
                            when -1     => c_kx <= resize(f_acc(35 downto 16), 18);
                            when 0      => c_kx <= resize(f_acc(35 downto 15), 18);
                            when others => c_kx <= resize(f_acc(35 downto 14), 18);
                        end case;
                    when 117 =>                      -- Nt = 246 mt >> (15 - sh)
                        case to_integer(z_sh5) is
                            when -2     => c_nt <= resize(f_acc(35 downto 17), 10);
                            when -1     => c_nt <= resize(f_acc(35 downto 16), 10);
                            when 0      => c_nt <= resize(f_acc(35 downto 15), 10);
                            when others => c_nt <= resize(f_acc(35 downto 14), 10);
                        end case;
                    when 141 =>
                        if z_phi = '1' then c_ah <= resize(f_acc(23 downto 12), 11) + 256;
                        else c_ah <= resize(f_acc(23 downto 12), 11); end if;
                    when 142 =>
                        c_av <= resize(c_ah + shift_right(c_ah, 2), 10);
                        c_at <= shift_left(resize(c_nt, 14), 3) + resize(c_nt, 14);
                    when 165 =>
                        if z_bhi = '1' then c_hw <= resize(f_acc(20 downto 9), 12) + 1290;
                        else c_hw <= resize(f_acc(20 downto 9), 12) + 300; end if;
                        case to_integer(s_xs) is
                            when 4      => c_pstep <= shift_left(resize(c_kx, 20), 2);
                            when 3      => c_pstep <= shift_left(resize(c_kx, 20), 1) + resize(c_kx, 20);
                            when others => c_pstep <= shift_left(resize(c_kx, 20), 1);
                        end case;
                    when 189 => z_sp <= f_acc(17 downto 0);
                    when 213 => c_srk <= f_acc(26 downto 10);
                    when 237 => c_srt <= f_acc(26 downto 10);
                    when 261 => c_srb <= f_acc(26 downto 10);
                    when 285 => z_lst <= f_acc(27 downto 10);
                    when 309 => c_bst <= f_acc(27 downto 15);
                    when 358 =>                      -- motion: everything rolls (up for Speed > 0)
                        if s_vneg = '1' then
                            c_rk <= c_rk - resize(c_srk, 24);
                            c_rt <= c_rt - resize(c_srt, 24);
                            c_rb <= c_rb - resize(c_srb, 24);
                        else
                            c_rk <= c_rk + resize(c_srk, 24);
                            c_rt <= c_rt + resize(c_srt, 24);
                            c_rb <= c_rb + resize(c_srb, 24);
                        end if;
                        if (s_vneg xor c_sq(10)) = '1' then
                            c_bb <= c_bb - resize(c_bst, 21);
                        else
                            c_bb <= c_bb + resize(c_bst, 21);
                        end if;
                        c_nc <= s_w(11 downto 1);
                        -- tear seed: redrawn every 45 fields
                        z_lfsr <= z_lfsr(14 downto 0) & (z_lfsr(15) xor z_lfsr(13) xor z_lfsr(12) xor z_lfsr(10));
                        if z_tf = 44 then
                            z_tf <= (others => '0'); c_tseed <= z_lfsr;
                        else
                            z_tf <= z_tf + 1;
                        end if;
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
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
    -- write of a 4-pixel box of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_t : unsigned(9 downto 0);
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
            if s_src = '1' then v_l := s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then v_l := v_l - 16; else v_l := (others => '0'); end if;
            end if;
            s_lav <= data_in.avid;
            s_l0 <= v_l; s_l1 <= s_l0; s_l2 <= s_l1; s_l3 <= s_l2;
            v_t := resize(s_l0, 10) + resize(s_l1, 10) + resize(s_l2, 10) + resize(s_l3, 10);
            s_lb_wd <= std_logic_vector(v_t(9 downto 2));
            s_lb_we <= s_lav;
            s_lb_wa <= s_wcol;
            if s_lav = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line on one serial multiplier (16 iterations,
    -- result at load + 17): the key V cosine x Av, the ring V cosine x AT and
    -- its zone, the background colour and its shadow / rim deltas, the band
    -- start (xc = 325 + s (y - 540) + roll), the H phase start; tear and jitter.  The key beam loads the
    -- n_* key values at g_start, the colour beam the colour values C_D clocks later (all written after that)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_p  : unsigned(23 downto 0);
        variable v_q  : unsigned(18 downto 0);
        variable v_x  : signed(13 downto 0);
        variable v_c  : unsigned(12 downto 0);
        variable v_d  : signed(11 downto 0);
        variable v_z  : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 700 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0'); e_tb <= (others => '0'); e_tl <= c_tseed; e_jl <= c_tseed xor x"47B1";
                e_lc <= (others => '0');
            elsif g_start = '1' then
                v_q := e_vy + resize(s_ys, 19);
                e_vy <= v_q;
                -- tear bands of 72 virtual lines: the band LFSR steps at each boundary
                if resize(e_tb, 16) + resize(s_ys, 16) >= 72 * 256 then
                    e_tb <= resize(resize(e_tb, 16) + resize(s_ys, 16) - 72 * 256, 15);
                    e_tl <= e_tl(14 downto 0) & (e_tl(15) xor e_tl(13) xor e_tl(12) xor e_tl(10));
                else
                    e_tb <= e_tb + resize(s_ys, 15);
                end if;
                if v_q(9) /= e_vy(9) then
                    e_jl <= e_jl(14 downto 0) & (e_jl(15) xor e_jl(13) xor e_jl(12) xor e_jl(10));
                end if;
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(32 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>                     -- line-paired virtual line; tear offset (h)
                    e_vyq <= e_vy(18 downto 9) & '0';
                    if s_tear = '1' then e_th <= resize(signed(e_tl(8 downto 0)), 10); else e_th <= (others => '0'); end if;
                when 2 =>
                    m_a <= to_unsigned(C_KYK, 34); m_b <= resize(e_vyq, 16); m_acc <= (others => '0');
                    e_vb <= signed(resize(e_vyq, 12)) - 540;
                    -- H phase start offset: dx = x0 + tear - 52 (h)
                    v_d := signed(resize(s_x0, 12)) + resize(e_th, 12) - 52;
                    e_dx <= v_d;
                when 19 =>                    -- key V phase
                    v_p := m_acc(23 downto 0) + c_rk;
                    r_cs_a <= v_p(23 downto 14);
                    m_a <= to_unsigned(C_KYT, 34); m_b <= resize(e_vyq, 16); m_acc <= (others => '0');
                when 21 =>
                    e_ck <= r_cs_q;
                when 22 => null;
                when 36 =>                    -- ring V phase
                    v_p := m_acc(23 downto 0) + c_rt;
                    r_cs_a <= v_p(23 downto 14);
                    m_a <= resize(unsigned(abs(e_ck)), 34); m_b <= resize(c_av, 16); m_acc <= (others => '0');
                when 38 =>
                    e_ct <= r_cs_q;
                when 39 =>                    -- zone: purple where ct > 0.35 (8 steps of 1/32)
                    v_z := resize(e_ct, 17) - 11469;
                    if v_z < 0 then n_zn <= "000";
                    elsif v_z >= 7168 then n_zn <= "111";
                    else n_zn <= unsigned(v_z(12 downto 10)); end if;
                when 53 =>                    -- C = +-Av ck >> 15 - hw
                    if e_ck(15) = '1' then
                        n_cl <= -signed(resize(m_acc(24 downto 15), 14)) - signed(resize(c_hw, 14));
                    else
                        n_cl <= signed(resize(m_acc(24 downto 15), 14)) - signed(resize(c_hw, 14));
                    end if;
                    m_a <= to_unsigned(C_KYB, 34); m_b <= resize(e_vyq, 16); m_acc <= (others => '0');
                when 70 =>                    -- background phase
                    v_p := m_acc(23 downto 0) + c_rb;
                    r_bg_a <= v_p(23 downto 16);
                    m_a <= resize(unsigned(abs(e_ct)), 34); m_b <= resize(c_at, 16); m_acc <= (others => '0');
                when 72 =>
                    n_bgy <= unsigned(r_bg_q(31 downto 22));
                    n_bgu <= unsigned(r_bg_q(21 downto 12));
                    n_bgv <= unsigned(r_bg_q(11 downto 2));
                when 73 =>
                    n_dsy <= to_signed(C_SP_SHD_Y, 11) - signed(resize(n_bgy, 11));
                    n_dsu <= to_signed(C_SP_SHD_U, 11) - signed(resize(n_bgu, 11));
                    n_dsv <= to_signed(C_SP_SHD_V, 11) - signed(resize(n_bgv, 11));
                when 87 =>                    -- V_t = +-AT ct >> 15 (ring Q8, mod 2 rings)
                    if e_ct(15) = '1' then
                        n_vt <= (not m_acc(23 downto 15)) + 1;
                    else
                        n_vt <= m_acc(23 downto 15);
                    end if;
                    m_a <= resize(unsigned(abs(c_sq)), 34); m_b <= resize(unsigned(abs(e_vb)), 16); m_acc <= (others => '0');
                when 104 =>                   -- xc (h) = 650 + 2 s (y - 540) + roll; dh0 = x0 + tear + 28 xs - xc
                    if (c_sq(10) xor e_vb(11)) = '1' then
                        v_c := to_unsigned(650, 13) - m_acc(19 downto 7) + c_bb(20 downto 8);
                    else
                        v_c := to_unsigned(650, 13) + m_acc(19 downto 7) + c_bb(20 downto 8);
                    end if;
                    n_dh0 <= resize(s_x0, 13) + unsigned(resize(e_th, 13)) - v_c;
                    n_jit <= signed(resize(e_jl(4 downto 0), 6)) - 16;
                    m_a <= resize(c_kx, 34); m_b <= resize(unsigned(abs(e_dx)), 16); m_acc <= (others => '0');
                when 121 =>                   -- H phase start = dx KX (Q24); key = + 28 px
                    if e_dx(11) = '1' then
                        v_p := (not m_acc(23 downto 0)) + 1;
                    else
                        v_p := m_acc(23 downto 0);
                    end if;
                    n_pc0 <= v_p;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beams: the key beam (band distance mod 8192 h, H phase) runs from the
    -- line start; the colour beam (H phase for the rings) starts C_D clocks
    -- later, so the key is always 28 px ahead of the colour.  Under Mirror
    -- every DDA runs back after the centre pixel.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then g_cdc <= to_unsigned(1, 5);
            elsif g_cdc = C_D then g_cdc <= (others => '0');
            elsif g_cdc /= 0 then g_cdc <= g_cdc + 1; end if;
            -- key beam: from the line start
            if g_start = '1' then
                b_dh <= n_dh0;
                b_pk <= n_pc0;
                b_px <= (others => '0');
                b_kback <= '0';
                l_cl <= n_cl;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                if b_kback = '1' then
                    b_pk <= b_pk - resize(c_pstep, 24);
                    b_dh <= b_dh - resize(s_xs, 13);
                else
                    b_pk <= b_pk + resize(c_pstep, 24);
                    b_dh <= b_dh + resize(s_xs, 13);
                end if;
                if s_mir = '1' and b_px + 1 >= c_nc then b_kback <= '1'; end if;
            end if;
            -- colour beam: C_D clocks behind
            if g_cdc = C_D then
                b_pc <= n_pc0;
                b_cpx <= (others => '0');
                b_back <= '0';
                l_vt <= n_vt; l_zn <= n_zn; l_jit <= n_jit;
                l_bgy <= n_bgy; l_bgu <= n_bgu; l_bgv <= n_bgv;
                l_dsy <= n_dsy; l_dsu <= n_dsu; l_dsv <= n_dsv;
            elsif s_gav_sr(C_D - 1) = '1' then
                b_cpx <= b_cpx + 1;
                if b_back = '1' then b_pc <= b_pc - resize(c_pstep, 24);
                else b_pc <= b_pc + resize(c_pstep, 24); end if;
                if s_mir = '1' and b_cpx + 1 >= c_nc then b_back <= '1'; end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_k  : signed(15 downto 0);
        variable v_r  : unsigned(8 downto 0);
        variable v_n  : unsigned(4 downto 0);
        variable v_c  : unsigned(8 downto 0);
        variable v_j  : signed(9 downto 0);
        variable v_y, v_u, v_v : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: folded band distance (h, period 8192); ROM addresses
            if b_dh(12) = '1' then p2_tri <= (not b_dh) + 1; else p2_tri <= b_dh; end if;
            r_hk_a <= b_pk(23 downto 16);
            r_ch_a <= b_pc(23 downto 16);
            -- 3: (ROM data)
            p3_tri <= p2_tri;
            -- 4: fabric copies; band + line term
            p4_hk <= r_hk_q;
            p4_ch <= r_ch_q;
            p4_s  <= unsigned(s_lb_q);
            p4_a  <= signed(resize(p3_tri, 15)) + resize(l_cl, 15);
            -- 5: products
            p5_kh <= signed('0' & c_ah) * p4_hk;
            p5_rc <= signed('0' & c_nt) * p4_ch;
            p5_e  <= p4_s * s_k6;
            p5_a  <= p4_a;
            -- 6: the key (28 px ahead) and the ring phase (mod 2 rings)
            v_k := resize(p5_a, 16) + resize(shift_right(p5_kh, 6), 16) + signed(resize(p5_e(17 downto 8), 16));
            p6_k <= v_k;
            v_r := l_vt + unsigned(p5_rc(15 downto 7)) + p5_e(15 downto 7);
            p6_r <= v_r;
            -- 7: key bit into the look-ahead register; texture palette address
            if (p6_k > 0) xor (s_inv = '1') then
                s_sr <= s_sr(C_D - 1 downto 0) & '1';
            else
                s_sr <= s_sr(C_D - 1 downto 0) & '0';
            end if;
            p7_ta <= l_zn & p6_r(8 downto 3);
            -- 8: key of the pixel; distance to the next texture pixel; distance since the last (h)
            p8_key <= s_sr(C_D);
            p8_ta <= p7_ta;
            v_n := (others => '0');
            for j in C_D downto 1 loop
                if s_sr(C_D - j) = '1' then v_n := to_unsigned(j, 5); end if;
            end loop;
            p8_n <= v_n;
            if s_gav_sr(6 + C_D) = '0' then           -- outside the line: no texture seen yet
                p8_cnt <= (others => '1');
            elsif s_sr(C_D) = '1' then
                p8_cnt <= (others => '0');
            else
                v_c := resize(p8_cnt, 9) + s_xs;
                if v_c(8) = '1' then p8_cnt <= (others => '1'); else p8_cnt <= v_c(7 downto 0); end if;
            end if;
            -- 9: rim distance (h); jittered shadow distance (virtual px); texture address (issued late)
            r_tx_a <= p8_ta;
            p9_key <= p8_key;
            case to_integer(s_xs) is
                when 4      => p9_nh <= resize(shift_left(resize(p8_n, 7), 2), 7);
                when 3      => p9_nh <= resize(shift_left(resize(p8_n, 7), 1), 7) + resize(p8_n, 7);
                when others => p9_nh <= resize(shift_left(resize(p8_n, 7), 1), 7);
            end case;
            v_j := signed(resize(p8_cnt, 10)) + resize(l_jit, 10);
            if v_j < 0 then p9_cj <= (others => '0');
            elsif v_j >= 126 then p9_cj <= (others => '1');
            else p9_cj <= unsigned(v_j(6 downto 1)); end if;
            -- 10: rim = texture 2 .. 44 h ahead (1 .. 22 virtual px); shadow alpha
            p10_key <= p9_key;
            if p9_nh >= 2 and p9_nh <= 44 then p10_rim <= '1'; else p10_rim <= '0'; end if;
            p10_as <= C_SP_SHA(to_integer(p9_cj));
            -- 11: the shadow over the line's background: delta x alpha; texture ROM data (fabric)
            p11_tx <= r_tx_q; p11_key <= p10_key; p11_rim <= p10_rim;
            p11_my <= resize(l_dsy * signed('0' & p10_as), 16);
            p11_mu <= resize(l_dsu * signed('0' & p10_as), 16);
            p11_mv <= resize(l_dsv * signed('0' & p10_as), 16);
            -- 12: texture, the rim, or the background with its shadow
            if p11_key = '1' then
                s_y12 <= unsigned(p11_tx(31 downto 22)); s_u12 <= unsigned(p11_tx(21 downto 12)); s_v12 <= unsigned(p11_tx(11 downto 2));
            elsif p11_rim = '1' then
                s_y12 <= to_unsigned(C_SP_RIM_Y, 10); s_u12 <= to_unsigned(C_SP_RIM_U, 10); s_v12 <= to_unsigned(C_SP_RIM_V, 10);
            else
                v_y := signed(resize(l_bgy, 12)) + resize(shift_right(p11_my, 4), 12);
                v_u := signed(resize(l_bgu, 12)) + resize(shift_right(p11_mu, 4), 12);
                v_v := signed(resize(l_bgv, 12)) + resize(shift_right(p11_mv, 4), 12);
                s_y12 <= unsigned(v_y(9 downto 0)); s_u12 <= unsigned(v_u(9 downto 0)); s_v12 <= unsigned(v_v(9 downto 0));
            end if;
            -- 13: compose (neutral black outside the generated avid)
            if s_gav_sr(10 + C_D) = '0' then
                s_y13 <= to_unsigned(64, 10); s_u13 <= to_unsigned(512, 10); s_v13 <= to_unsigned(512, 10);
            else
                s_y13 <= s_y12; s_u13 <= s_u12; s_v13 <= s_v12;
            end if;
            -- 14: Verdant = the chroma complement
            s_y14 <= s_y13;
            if s_pal = '1' then s_u14 <= not s_u13; s_v14 <= not s_v13; else s_u14 <= s_u13; s_v14 <= s_v13; end if;
            -- 15: copy (box A input)
            s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12 + C_D);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the HD HDMI bar fix), luma 13/16 and the
    -- blanking gate.  The running 8-sums snap back to neutral once 8 blank
    -- samples are in (the per-line resync).
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- box A: stage 16 sums, stage 17 output
            if s_bah = "00000000" then
                s_bay <= to_unsigned(64 * 8, 13); s_bau <= to_unsigned(512 * 8, 13); s_bav <= to_unsigned(512 * 8, 13);
            else
                s_bay <= s_bay + s_y15 - unsigned(r_aq2(31 downto 22));
                s_bau <= s_bau + s_u15 - unsigned(r_aq2(21 downto 12));
                s_bav <= s_bav + s_v15 - unsigned(r_aq2(11 downto 2));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: neutral off the generated avid
            if s_gav_sr(14 + C_D) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14 + C_D);
            -- box B: stage 18 sums, stage 19 output
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(64 * 8, 13); s_bbu <= to_unsigned(512 * 8, 13); s_bbv <= to_unsigned(512 * 8, 13);
            else
                s_bby <= s_bby + s_y17 - unsigned(r_bq2(31 downto 22));
                s_bbu <= s_bbu + s_u17 - unsigned(r_bq2(21 downto 12));
                s_bbv <= s_bbv + s_v17 - unsigned(r_bq2(11 downto 2));
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
    data_out.hsync_n <= r_sq2(0);
    data_out.vsync_n <= r_sq2(1);
    data_out.field_n <= r_sq2(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture spine;
