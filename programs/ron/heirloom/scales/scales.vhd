-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Scales v0.1.0 -- Heirloom (phase 3, video clips), rebuilt from the clip WIN_20230712_00_46_37 at 2.5 s.
--
-- The still: ten-plus "(" crescent scales across the frame, each one period of a horizontal sawtooth whose
-- frequency is bent by a vertical parabola (the left curves nearly straight, the right ones wrapping hard).
-- Scanned left to right every scale runs pink / mauve -> salmon -> tan -> (a cyan or dark teal crescent in
-- the key lobes) -> a dark maroon line, with red fringes in the olive bands at the top, the seam and the
-- bottom.  Cyan diamonds sit in the lower half (vertex y ~ 775, paired: alternate scales differ), dim teal
-- teardrops at the top left.  In the clip the cyan lobes swell and fade, lower then upper, on a 5.35 s cycle.
--
-- The patch, as fitted (scales.py has the colour tables, notes.md every number):
--   phase  = f(y) (x + 194.8) + 0.455 + bend      a saw VCO hard-synced 194.8 px before the picture
--   f(y)   = (1/177) (1 - B d^2)                   V parabola FM, d = (y - 494.8) / 540, B .272 up / .206 down
--   key    = cyan where saw > k,  k = Kc - .592 tri(lobes) - .40 sin(wave) + .263 x/1000 +- .093 (pairs)
--            wave = a long V sine (2800 lines) travelling up at 523 lines/s (the clip's 5.35 s swell cycle);
--            the fill is the ramp (saw - k), dimmed by an opacity 2 (1 - k at the lobe tip) - .25 x/1000
--   bend   = -.137 clamp(1 - k_row): the key CV also pulls the VCO phase (the kinks at the lobe vertex)
--   colour = ROWS[P][saw], P = 82 + 106 |sin(pi (y - 533) / 510)| - 43 x/1000 +- 8 (pink / olive bands)
--   camera = luma bends the phase and opens the key
--
--   K1  Scales     scale count x1/3 .. x3 (the still at 512)
--   K2  Bow        curvature: reversed ")" (0) .. straight (300) .. the still (512) .. x3.4 (1023)
--   K3  Lobes      key lobe period: 0.43 .. 7 lobes per frame (1.74 at 512)
--   K4  Drift      bipolar: the travelling wave's speed and direction (768 = the clip, 512 = still)
--   K5  Bands      pink / olive band count 0.5 .. 8.5 humps (2.1 at 512)
--   K6  Warp       how far the camera (or the Osc) bends the scales and opens the cyan
--   S7  Wave       Saw (the still) / Tri: every scale mirrored into a lens with a cyan eye
--   S8  Mirror     Off / On: the right half mirrored onto the left, the scales facing out of a seam
--   S9  Rows       One (the still) / Tiled: three stacked rows of scales (fish-scale armour)
--   S10 Colours    Koi (the still) / Reef (the complementary hue: blue-green scales, coral crescents)
--   S11 Source     Video / Osc
--   P12 CRESCENTS  key level: none (0) .. the still (460) .. cyan flooding every scale (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, exp2 ROM), a per-line engine kicked at
-- each line start (computes the NEXT line on one serial multiplier: d^2, the parabola, f, the line-start
-- phase, the lobe / wave / band terms), a 15-stage pixel path (phase / key / band / opacity DDAs at stage 5,
-- luma warp, the band row and the key chosen against a 16-px ordered dither that the output boxes average
-- into a smooth lerp / blend).  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated
-- avid; output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes, their 8-back taps in two
-- block-RAM rings, with the running-sum resync; luma 13/16) before the blanking gate.  Colours in the
-- hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.scales_pkg.all;
use work.all;

architecture scales of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- fitted constants
    constant C_FB    : integer := 47394;     -- 1/177.0 cycles per virtual px, Q24 per h (half px)
    constant C_YC    : integer := 495;       -- parabola vertex (virtual line)
    constant C_X0H   : integer := 390;       -- sync point: 194.8 px before the picture, in h
    constant C_C0    : integer := 7635376;   -- 0.4551 cycles, Q24
    constant C_DK    : integer := 38797;     -- lobe depth 0.592, Q16
    constant C_TS    : integer := 27060;     -- 2^24 / 620 (lobe period), Q24 per virtual line
    constant C_TY    : integer := 768;       -- lobe centre
    constant C_BS    : integer := 16448;     -- 2^24 / (2 * 510) (band half-period 510), Q24 per virtual line
    constant C_BY    : integer := 533;       -- band seam
    constant C_VA0   : integer := 2160066;   -- 82.4 / 40 rows, Q20
    constant C_VA1   : integer := 2773483;   -- 105.8 / 40 rows, Q20
    constant C_ALTV  : integer := 49;        -- pairs: +-0.19 rows, Q8
    constant C_ALTK  : integer := 381;       -- pairs: +-0.093 cycles, Q12

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
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_sn_a : unsigned(7 downto 0) := (others => '0');
    signal r_sn_q : signed(15 downto 0) := (others => '0');
    signal r_c_a  : unsigned(9 downto 0) := (others => '0');
    signal r_c_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_k_a  : unsigned(6 downto 0) := (others => '0');
    signal r_k_q  : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(460 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(768, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_wave, s_mir, s_rows, s_pal, s_src : std_logic := '0';

    -- per-field scratch
    signal z_e    : signed(10 downto 0) := (others => '0');
    signal z_sh1, z_sh3, z_sh5 : signed(2 downto 0) := (others => '0');
    signal z_mt1, z_mt3, z_mt5 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_tsv, z_bsv : unsigned(19 downto 0) := (others => '0');
    signal z_kneg, z_bneg : std_logic := '0';
    signal z_ab   : unsigned(9 downto 0) := (others => '0');            -- |bow| knob distance
    signal z_av, z_aw : unsigned(19 downto 0) := (others => '0');      -- speed^2, warp^2

    -- per-field constants
    signal c_f0    : unsigned(17 downto 0) := to_unsigned(C_FB, 18);       -- VCO base frequency, Q24 per h
    signal c_bu, c_bd : unsigned(15 downto 0) := (others => '0');         -- |bow| coefficients (d^2 Cb >> 18 = Q16)
    signal c_bneg  : std_logic := '0';
    signal c_tstep, c_tst0 : unsigned(23 downto 0) := (others => '0');    -- lobe phase per line, at line 0 (Q24)
    signal c_bstep, c_bst0 : unsigned(23 downto 0) := (others => '0');    -- band phase
    signal c_wstep : unsigned(23 downto 0) := (others => '0');            -- travelling wave phase per line
    signal c_wph   : unsigned(23 downto 0) := to_unsigned(16#E28F6F#, 24); -- wave phase at line 0 (the still's)
    signal c_wrate : unsigned(23 downto 0) := to_unsigned(52264, 24);     -- wave phase per field (Q24)
    signal c_kc    : signed(19 downto 0) := to_signed(69927, 20);          -- key level, Q16
    signal c_gw    : unsigned(11 downto 0) := (others => '0');            -- warp per luma step, Q24 >> 5
    signal c_xsh   : unsigned(11 downto 0) := (others => '0');            -- x' at pixel 0 (h)
    signal c_xsp   : unsigned(12 downto 0) := to_unsigned(C_X0H, 13);     -- x' + X0 at pixel 0 (h)
    signal c_kx0, c_px0 : signed(23 downto 0) := (others => '0');         -- H ramps at pixel 0 (Q20)
    signal c_kx, c_px : signed(15 downto 0) := (others => '0');           -- H ramp steps per pixel (Q20)
    signal c_ctr   : unsigned(10 downto 0) := to_unsigned(959, 11);       -- mirror: last pixel stepping down

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld1, z_ld : std_logic := '0';
    signal z_oh  : std_logic_vector(16 downto 0) := (others => '0');
    signal z_kd  : unsigned(9 downto 0) := (others => '0');
    signal z_ks  : unsigned(15 downto 0) := (others => '0');

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
    signal e_vy   : unsigned(19 downto 0) := (others => '0');           -- next line, Q8 virtual lines
    signal e_rw   : unsigned(16 downto 0) := (others => '0');           -- tiled rows: (y - Yc + 180) mod 360, Q8
    signal e_tph, e_bph, e_wph : unsigned(23 downto 0) := (others => '0');
    signal e_dr, e_dv, e_dd : signed(12 downto 0) := (others => '0');
    signal e_d    : unsigned(10 downto 0) := (others => '0');            -- |d| (virtual lines)
    signal e_dneg : std_logic := '0';
    signal e_tneg, e_wneg : std_logic := '0';
    signal e_tm   : unsigned(15 downto 0) := (others => '0');            -- |T| Q15
    signal e_wm   : unsigned(15 downto 0) := (others => '0');            -- |W| Q15
    signal e_bs   : unsigned(15 downto 0) := (others => '0');            -- |sin| bands, Q15
    signal e_o    : signed(21 downto 0) := (others => '0');
    signal e_one  : unsigned(15 downto 0) := (others => '0');            -- (1 - q) Q15
    signal e_f    : unsigned(19 downto 0) := (others => '0');            -- f, Q24 per h
    signal e_inc  : unsigned(20 downto 0) := (others => '0');
    signal e_p0   : unsigned(24 downto 0) := (others => '0');
    signal e_dkt, e_aw : signed(19 downto 0) := (others => '0');
    signal e_kb, e_km : signed(20 downto 0) := (others => '0');
    signal e_c    : unsigned(16 downto 0) := (others => '0');
    signal e_x    : signed(21 downto 0) := (others => '0');
    signal e_e    : signed(21 downto 0) := (others => '0');
    signal e_cw   : unsigned(24 downto 0) := (others => '0');
    signal e_pa   : signed(23 downto 0) := (others => '0');
    signal m_a    : unsigned(39 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(39 downto 0) := (others => '0');
    signal n_ph0  : unsigned(24 downto 0) := (others => '0');
    signal n_inc  : unsigned(20 downto 0) := (others => '0');
    signal n_k0, n_p0 : signed(23 downto 0) := (others => '0');
    signal n_e    : signed(15 downto 0) := (others => '0');             -- opacity reference, Q12

    -- beam: pixel count (stage 1); phase / key / band / opacity DDAs (stage 5), mirrored about the centre on S8
    signal b_gsr, b_fsr : std_logic_vector(3 downto 0) := (others => '0');
    signal b_ph, b_sph : unsigned(24 downto 0) := (others => '0');
    signal b_inc : unsigned(20 downto 0) := (others => '0');
    signal b_k, b_p : signed(23 downto 0) := (others => '0');
    signal b_sk, b_sp : signed(15 downto 0) := (others => '0');
    signal b_e   : signed(15 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    signal p6_ph  : unsigned(24 downto 0) := (others => '0');
    signal p6_k, p7_k : signed(15 downto 0) := (others => '0');      -- key, Q12
    signal p6_p, p7_p : signed(11 downto 0) := (others => '0');      -- band row, Q8
    signal p6_e   : signed(16 downto 0) := (others => '0');         -- opacity reference - key, Q12
    signal p7_op  : signed(18 downto 0) := (others => '0');         -- 3 x that
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(15 downto 0) := (others => '0');
    signal p5_lw  : unsigned(24 downto 0) := (others => '0');
    signal p6_lk  : unsigned(13 downto 0) := (others => '0');
    signal p7_s, p8_s : unsigned(11 downto 0) := (others => '0');
    signal p7_par : std_logic := '0';
    signal p8_k12 : signed(16 downto 0) := (others => '0');
    signal p8_pp  : signed(12 downto 0) := (others => '0');
    signal p8_op5, p9_op5, p10_op5, p11_op5 : unsigned(5 downto 0) := (others => '0');
    signal p9_d   : signed(16 downto 0) := (others => '0');
    signal p9_pc  : unsigned(10 downto 0) := (others => '0');
    signal p9_sh  : unsigned(8 downto 0) := (others => '0');
    signal p9_s6  : unsigned(5 downto 0) := (others => '0');
    signal p10_d5 : signed(19 downto 0) := (others => '0');
    signal p11_m5 : unsigned(5 downto 0) := (others => '0');
    signal p12_key : std_logic := '0';
    signal p12_c, p12_kq : std_logic_vector(31 downto 0) := (others => '0');
    signal p13_y, p13_u, p13_v : unsigned(9 downto 0) := (others => '0');
    signal p14_y, p14_u, p14_v : unsigned(9 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    -- the boxes' 8-back taps come from two block-RAM rings (read pointer = write - 6, then a fabric register)
    type t_ring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal o_ra, o_rb : t_ring := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of o_ra : signal is true;
    attribute no_rw_check of o_rb : signal is true;
    signal o_wp : unsigned(7 downto 0) := (others => '0');
    signal o_rp : unsigned(7 downto 0) := to_unsigned(250, 8);
    signal o_aq, o_aq2, o_bq, o_bq2 : std_logic_vector(31 downto 0) := (others => '0');
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 inputs of boxes A, B
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
            r_ex_q <= C_SC_EXP2(to_integer(r_ex_a));
            r_sn_q <= C_SC_SIN(to_integer(r_sn_a));
            r_c_q  <= C_SC_LUT(to_integer(r_c_a));
            r_k_q  <= C_SC_KEY(to_integer(r_k_a));
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
        variable v_fa : unsigned(35 downto 0);
        variable v_fb : unsigned(15 downto 0);
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

            -- K4 Drift: bipolar, +-8 deadband, |v| - 8 (square law on the multiplier)
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
                s_wave <= registers_in(6)(0);
                s_mir  <= registers_in(6)(1);
                s_rows <= registers_in(6)(2);
                s_pal  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
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

            -- serial products: slot k loads at zc = 33 + 24 k, 16 iterations, result read at 50 + 24 k.  The
            -- slot number is set at 30 + 24 k, decoded one-hot at 31, the operands fetched at 32.
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
                    when others => null;
                end case;
            end if;
            -- operand fetch: the slot number is decoded into a registered one-hot, then a flat AND-OR builds
            -- the operands every clock (no enables, no decode on the multiplier's path); the load follows
            -- two clocks after the slot is set.
            z_ld1 <= z_pre; z_ld <= z_ld1;
            for i in 0 to 16 loop
                if z_sel = i then z_oh(i) <= '1'; else z_oh(i) <= '0'; end if;
            end loop;
            v_fa := (others => '0'); v_fb := (others => '0');
            if z_oh(0)  = '1' then v_fa := v_fa or to_unsigned(C_FB, 36);           v_fb := v_fb or z_mt1;                   end if;  -- F0
            if z_oh(1)  = '1' then v_fa := v_fa or resize(z_ab, 36);                v_fb := v_fb or to_unsigned(303, 16);    end if;  -- bow up
            if z_oh(2)  = '1' then v_fa := v_fa or resize(z_ab, 36);                v_fb := v_fb or to_unsigned(229, 16);    end if;  -- bow down
            if z_oh(3)  = '1' then v_fa := v_fa or to_unsigned(C_TS, 36);           v_fb := v_fb or z_mt3;                   end if;  -- lobes / vline
            if z_oh(4)  = '1' then v_fa := v_fa or resize(z_tsv, 36);               v_fb := v_fb or resize(s_ys, 16);        end if;
            if z_oh(5)  = '1' then v_fa := v_fa or resize(z_tsv, 36);               v_fb := v_fb or to_unsigned(C_TY, 16);   end if;
            if z_oh(6)  = '1' then v_fa := v_fa or to_unsigned(C_BS, 36);           v_fb := v_fb or z_mt5;                   end if;  -- bands / vline
            if z_oh(7)  = '1' then v_fa := v_fa or resize(z_bsv, 36);               v_fb := v_fb or resize(s_ys, 16);        end if;
            if z_oh(8)  = '1' then v_fa := v_fa or resize(z_bsv, 36);               v_fb := v_fb or to_unsigned(C_BY, 16);   end if;
            if z_oh(9)  = '1' then v_fa := v_fa or resize(s_vm, 36);                v_fb := v_fb or resize(s_vm, 16);        end if;  -- speed^2
            if z_oh(10) = '1' then v_fa := v_fa or resize(z_av(17 downto 0), 36);   v_fb := v_fb or to_unsigned(870, 16);    end if;  -- wave rate
            if z_oh(11) = '1' then v_fa := v_fa or to_unsigned(5992, 36);           v_fb := v_fb or resize(s_ys, 16);        end if;  -- wave / line
            if z_oh(12) = '1' then v_fa := v_fa or resize(z_kd, 36);                v_fb := v_fb or z_ks;                    end if;  -- key level
            if z_oh(13) = '1' then v_fa := v_fa or resize(s_g6(12 downto 3), 36);   v_fb := v_fb or resize(s_g6(12 downto 3), 16); end if;  -- warp^2
            if z_oh(14) = '1' then v_fa := v_fa or resize(z_aw, 36);                v_fb := v_fb or to_unsigned(360, 16);    end if;
            if z_oh(15) = '1' then v_fa := v_fa or resize(c_xsh, 36);               v_fb := v_fb or to_unsigned(138, 16);    end if;  -- k ramp at x'
            if z_oh(16) = '1' then v_fa := v_fa or resize(c_xsh, 36);               v_fb := v_fb or to_unsigned(566, 16);    end if;  -- P ramp
            z_fa <= v_fa; z_fb <= v_fb;
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
                    -- motion (once per field), exponents and exp2 lookups (address at k, data at k + 2)
                    when 20 =>
                        if s_vneg = '1' then c_wph <= c_wph - c_wrate; else c_wph <= c_wph + c_wrate; end if;
                        v_e := signed(resize(s_g1(12 downto 3), 11)) - 512;            -- scales: +-log2(3) octaves
                        z_e <= v_e - shift_right(v_e, 2) + shift_right(v_e, 5);        -- x 0.7813 (Q8 octaves)
                        -- x' at pixel 0 (h) and its sum with the sync point
                        if s_mir = '1' then c_xsh <= to_unsigned(3840, 12) - resize(s_x0, 12);
                        else c_xsh <= resize(s_x0, 12); end if;
                    when 21 =>
                        r_ex_a <= unsigned(z_e(7 downto 0)); z_sh1 <= z_e(10 downto 8);
                        z_e <= signed(resize(s_g3(12 downto 3), 11)) - 512;             -- lobes: +-2 octaves
                        c_xsp <= resize(c_xsh, 13) + to_unsigned(C_X0H, 13);
                    when 23 =>
                        z_mt1 <= r_ex_q;
                        r_ex_a <= unsigned(z_e(7 downto 0)); z_sh3 <= z_e(10 downto 8);
                        z_e <= signed(resize(s_g5(12 downto 3), 11)) - 512;             -- bands: +-2 octaves
                    when 25 =>
                        z_mt3 <= r_ex_q;
                        r_ex_a <= unsigned(z_e(7 downto 0)); z_sh5 <= z_e(10 downto 8);
                        -- bow: g = (k2 - 300) / 212
                        if s_g2(12 downto 3) >= 300 then
                            z_ab <= s_g2(12 downto 3) - 300; z_bneg <= '0';
                        else
                            z_ab <= 300 - s_g2(12 downto 3); z_bneg <= '1';
                        end if;
                    when 28 =>                    -- key level: |p - 460| and the slope of its side
                        if s_gp(12 downto 3) < 460 then
                            z_kd <= to_unsigned(460, 10) - s_gp(12 downto 3); z_ks <= to_unsigned(142, 16);
                        else
                            z_kd <= s_gp(12 downto 3) - to_unsigned(460, 10); z_ks <= to_unsigned(122, 16);
                        end if;
                    when 27 =>
                        z_mt5 <= r_ex_q;
                        -- mirror reversal point; the H ramp steps per pixel (Q20; negative ramps stored negated)
                        c_ctr <= resize(s_w(11 downto 1), 11) - 1;
                        case to_integer(s_xs) is
                            when 4 =>      c_kx <= to_signed(552, 16);  c_px <= to_signed(-2264, 16);
                            when 3 =>      c_kx <= to_signed(414, 16);  c_px <= to_signed(-1698, 16);
                            when others => c_kx <= to_signed(276, 16);  c_px <= to_signed(-1132, 16);
                        end case;

                    -- slot results
                    when 50 =>                    -- F0 = C_FB * mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh1) is
                            when -2     => c_f0 <= resize(f_acc(35 downto 17), 18);
                            when -1     => c_f0 <= resize(f_acc(35 downto 16), 18);
                            when 0      => c_f0 <= resize(f_acc(35 downto 15), 18);
                            when others => c_f0 <= resize(f_acc(35 downto 14), 18);
                        end case;
                    when 74 => c_bu <= f_acc(17 downto 2); c_bneg <= z_bneg;
                    when 98 => c_bd <= f_acc(17 downto 2);
                    when 122 =>                   -- lobe phase per virtual line = C_TS * mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh3) is
                            when -2     => z_tsv <= resize(f_acc(35 downto 17), 20);
                            when -1     => z_tsv <= resize(f_acc(35 downto 16), 20);
                            when 0      => z_tsv <= resize(f_acc(35 downto 15), 20);
                            when others => z_tsv <= resize(f_acc(35 downto 14), 20);
                        end case;
                    when 146 => c_tstep <= f_acc(31 downto 8);
                    when 170 => c_tst0 <= (not f_acc(23 downto 0)) + 1;                  -- -768 / Pk
                    when 194 =>                   -- band phase per virtual line
                        case to_integer(z_sh5) is
                            when -2     => z_bsv <= resize(f_acc(35 downto 17), 20);
                            when -1     => z_bsv <= resize(f_acc(35 downto 16), 20);
                            when 0      => z_bsv <= resize(f_acc(35 downto 15), 20);
                            when others => z_bsv <= resize(f_acc(35 downto 14), 20);
                        end case;
                    when 218 => c_bstep <= f_acc(31 downto 8);
                    when 242 => c_bst0 <= (not f_acc(23 downto 0)) + 1;                  -- -533 / (2 Ph)
                    when 266 => z_av <= f_acc(19 downto 0);                                -- speed^2 (18 bits)
                    when 290 => c_wrate <= resize(f_acc(33 downto 10), 24);               -- speed^2 * 870 >> 10
                    when 314 => c_wstep <= f_acc(31 downto 8);
                    when 338 =>                   -- Kc (Q16): 69927 (1.067) at 460; +1.0 at 0, -1.05 at 1023
                        if s_gp(12 downto 3) < 460 then
                            c_kc <= to_signed(69927, 20) + signed(resize(f_acc(18 downto 0), 20));
                        else
                            c_kc <= to_signed(69927, 20) - signed(resize(f_acc(18 downto 0), 20));
                        end if;
                    when 362 => z_aw <= f_acc(19 downto 0);                                -- warp^2
                    when 386 => c_gw <= f_acc(28 downto 17);                              -- warp^2 * 360 >> 17
                    when 410 => c_kx0 <= signed(resize(f_acc(22 downto 0), 24));
                    when 434 => c_px0 <= -signed(resize(f_acc(22 downto 0), 24));
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
    -- load + 17): d^2, the bow, f, f * (x' + X0), the lobe depth, the wave, the
    -- band level; ~125 clocks, so it fits SD's 858-clock line
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_q  : signed(20 downto 0);
        variable v_o  : signed(21 downto 0);
        variable v_t  : unsigned(15 downto 0);
        variable v_c  : signed(21 downto 0);
        variable v_rw : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy  <= (others => '0');
                e_rw  <= to_unsigned(11571, 17);            -- (0 - 494.8 + 180) mod 360 = 45.2 lines, Q8
                e_tph <= c_tst0;
                e_bph <= c_bst0;
                e_wph <= c_wph;
                e_lc  <= (others => '0');
            elsif g_start = '1' then
                e_vy  <= e_vy + resize(s_ys, 20);
                v_rw  := e_rw + resize(s_ys, 17);
                if v_rw >= 92160 then e_rw <= v_rw - 92160; else e_rw <= v_rw; end if;
                e_tph <= e_tph + c_tstep;
                e_bph <= e_bph + c_bstep;
                e_wph <= e_wph + c_wstep;
                e_lc  <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(38 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    -- d: virtual lines from the vertex (Tiled: 3 x the distance to the row centre)
                    e_dr <= signed(resize(e_rw(16 downto 8), 13)) - 180;
                    e_dv <= signed(resize(e_vy(19 downto 8), 13)) - C_YC;
                    r_sn_a <= e_wph(23 downto 16);               -- the travelling wave
                    -- lobe triangle: T = 1 - 4 min(u, 1 - u), Q15 magnitude and sign
                    if e_tph(23) = '1' then v_t := not e_tph(23 downto 8); else v_t := e_tph(23 downto 8); end if;
                    if v_t(14 downto 0) > 16383 then
                        e_tneg <= '1'; e_tm <= resize(shift_left(resize(v_t(14 downto 0), 16), 1) - 32767, 16);
                    else
                        e_tneg <= '0'; e_tm <= resize(32767 - shift_left(resize(v_t(14 downto 0), 16), 1), 16);
                    end if;
                    e_pa <= to_signed(C_VA0, 24) + c_px0;
                when 2 =>
                    if s_rows = '1' then e_dd <= e_dr + shift_left(e_dr, 1); else e_dd <= e_dv; end if;
                    r_sn_a <= e_bph(23 downto 16);               -- band sine
                when 3 =>
                    if e_dd < 0 then e_d <= unsigned(resize(-e_dd, 11)); e_dneg <= '1';
                    else e_d <= unsigned(e_dd(10 downto 0)); e_dneg <= '0'; end if;
                    if r_sn_q < 0 then e_wneg <= '1'; e_wm <= unsigned(-r_sn_q); else e_wneg <= '0'; e_wm <= unsigned(r_sn_q); end if;
                when 4 =>
                    m_a <= resize(e_d, 40); m_b <= resize(e_d, 16); m_acc <= (others => '0');
                    if r_sn_q < 0 then e_bs <= unsigned(-r_sn_q); else e_bs <= unsigned(r_sn_q); end if;
                when 21 =>
                    m_a <= resize(m_acc(19 downto 0), 40); m_acc <= (others => '0');
                    if e_dneg = '1' then m_b <= c_bu; else m_b <= c_bd; end if;
                when 38 =>                    -- q = d^2 Cb >> 18 (Q16), signed by the bow; one = 1 - q, Q15, clamped
                    v_q := signed(resize(m_acc(38 downto 18), 21));
                    if c_bneg = '1' then v_o := to_signed(65536, 22) + resize(v_q, 22);
                    else v_o := to_signed(65536, 22) - resize(v_q, 22); end if;
                    e_o <= v_o;
                when 39 =>
                    if e_o < 4096 then e_one <= to_unsigned(2048, 16);
                    elsif e_o > 131071 then e_one <= to_unsigned(65535, 16);
                    else e_one <= unsigned(e_o(16 downto 1)); end if;
                when 40 =>
                    m_a <= resize(c_f0, 40); m_b <= e_one; m_acc <= (others => '0');
                when 57 =>                    -- f = F0 (1 - q), Q24 cycles per h
                    e_f <= m_acc(34 downto 15);
                    m_a <= resize(m_acc(34 downto 15), 40); m_b <= resize(c_xsp, 16); m_acc <= (others => '0');
                when 58 =>                    -- per-pixel step = f * xs
                    case to_integer(s_xs) is
                        when 4      => e_inc <= resize(shift_left(resize(e_f, 21), 2), 21);
                        when 3      => e_inc <= resize(shift_left(resize(e_f, 21), 1), 21) + resize(e_f, 21);
                        when others => e_inc <= resize(shift_left(resize(e_f, 21), 1), 21);
                    end case;
                when 74 =>                    -- phase at pixel 0, before the bend: f (x' + X0) + c0
                    e_p0 <= m_acc(24 downto 0) + to_unsigned(C_C0, 25);
                    m_a <= resize(e_tm, 40); m_b <= to_unsigned(C_DK, 16); m_acc <= (others => '0');
                when 91 =>                    -- Dk T (Q16)
                    if e_tneg = '1' then e_dkt <= -signed(resize(m_acc(34 downto 15), 20));
                    else e_dkt <= signed(resize(m_acc(34 downto 15), 20)); end if;
                    m_a <= resize(e_wm, 40); m_b <= to_unsigned(26214, 16); m_acc <= (others => '0');   -- A = 0.40
                when 92 =>
                    e_kb <= resize(c_kc, 21) - resize(e_dkt, 21);
                    e_km <= resize(c_kc, 21) - C_DK;
                when 108 =>                   -- A W (Q16)
                    if e_wneg = '1' then e_aw <= -signed(resize(m_acc(34 downto 15), 20));
                    else e_aw <= signed(resize(m_acc(34 downto 15), 20)); end if;
                    m_a <= to_unsigned(C_VA1, 40); m_b <= e_bs; m_acc <= (others => '0');
                when 109 =>                   -- Kb = Kc - Dk T - A W; kmin = Kc - Dk - A W
                    e_kb <= e_kb - resize(e_aw, 21);
                    e_km <= e_km - resize(e_aw, 21);
                when 110 =>                   -- bend term: clamp(1 - Kb, 0, 1)
                    v_c := to_signed(65536, 22) - resize(e_kb, 22);
                    if v_c < 0 then e_c <= (others => '0');
                    elsif v_c > 65536 then e_c <= to_unsigned(65536, 17);
                    else e_c <= unsigned(v_c(16 downto 0)); end if;
                    e_x <= to_signed(65536, 22) - resize(e_km, 22);                   -- 1 - kmin
                    n_k0 <= resize(shift_left(resize(e_kb, 24), 4), 24) + c_kx0;     -- key at pixel 0 (Q20)
                when 111 =>
                    e_cw <= shift_left(resize(e_c, 25), 5) + shift_left(resize(e_c, 25), 1);
                    e_e <= shift_right(e_x, 1) + shift_right(e_x, 3);              -- (2/3)(1 - kmin), in five adds
                when 112 =>
                    e_cw <= e_cw + resize(e_c, 25);                                 -- 0.1373 c, Q16 -> Q24 (x 35)
                    e_e <= e_e + shift_right(e_x, 5);
                when 113 =>
                    n_ph0 <= e_p0 - e_cw;
                    n_inc <= e_inc;
                    e_e <= e_e + shift_right(e_x, 7);
                when 114 =>
                    e_e <= e_e + shift_right(e_x, 9);
                when 115 =>                   -- opacity reference: op(x) = 3 (E - k(x)) = 2 (1 - kmin) - 0.79 x/1000
                    v_c := resize(e_kb, 22) + e_e;
                    n_e <= v_c(19 downto 4);
                when 125 =>                   -- band level P (rows, Q20) = VA0 + tilt + ramp at x' + VA1 |sin|
                    n_p0 <= e_pa + signed(resize(m_acc(37 downto 15), 24));
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam.  Stage 1 only counts pixels (the line-buffer read address and the
    -- mirror point); the VCO phase and the three H ramps (key, band, opacity)
    -- run as DDAs at stage 5, where the camera's warp term arrives, so nothing
    -- is carried through stages 2 .. 5.  Mirror runs them down to the centre and
    -- back up (x' = 960 + |x - 960|); the steps are registers, negated a pixel ahead.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_px <= (others => '0');
            elsif g_avid = '1' then
                b_px <= b_px + 1;
            end if;
            s_lb_ra <= b_px;
            b_gsr <= b_gsr(2 downto 0) & g_start;
            if g_avid = '1' and b_px = c_ctr then b_fsr <= b_fsr(2 downto 0) & '1'; else b_fsr <= b_fsr(2 downto 0) & '0'; end if;

            if b_gsr(3) = '1' then
                b_ph <= n_ph0;
                b_k <= n_k0; b_p <= n_p0; b_e <= n_e;
                b_inc <= n_inc;
                if s_mir = '1' then
                    b_sph <= (not resize(n_inc, 25)) + 1;
                    b_sk <= -c_kx; b_sp <= -c_px;
                else
                    b_sph <= resize(n_inc, 25);
                    b_sk <= c_kx; b_sp <= c_px;
                end if;
            elsif s_gav_sr(3) = '1' then
                b_ph <= b_ph + b_sph;
                b_k  <= b_k + b_sk; b_p <= b_p + b_sp;
                if b_fsr(3) = '1' then
                    b_sph <= resize(b_inc, 25);
                    b_sk <= c_kx; b_sp <= c_px;
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_s12 : unsigned(11 downto 0);
        variable v_kl  : signed(16 downto 0);
        variable v_sh  : signed(16 downto 0);
        variable v_m   : signed(17 downto 0);
        variable v_j   : unsigned(2 downto 0);
        variable v_th  : unsigned(3 downto 0);
        variable v_w   : unsigned(5 downto 0);
        variable v_c   : std_logic_vector(31 downto 0);
    begin
        if rising_edge(clk) then
            -- 3: luma from the line buffer
            p3_lm <= unsigned(s_lb_q);
            -- 4: warp partials (luma nibbles x gain)
            p4_ea <= p3_lm(3 downto 0) * c_gw;
            p4_eb <= p3_lm(7 downto 4) * c_gw;
            -- 5: luma x warp (Q24 cycles)
            if s_src = '1' then          -- Osc: twice the depth, so the generator's bends read at any Warp setting
                p5_lw <= shift_left(resize(p4_ea, 25) + shift_left(resize(p4_eb, 25), 4), 6);
            else
                p5_lw <= shift_left(resize(p4_ea, 25) + shift_left(resize(p4_eb, 25), 4), 5);
            end if;
            -- 6: the camera bends the phase (the DDAs are the stage-5 beam); 1.5 x the same amount opens the key
            p6_ph <= b_ph + p5_lw;
            p6_lk <= resize(p5_lw(24 downto 12), 14) + resize(p5_lw(24 downto 13), 14);
            p6_k <= b_k(23 downto 8); p6_p <= b_p(23 downto 12);
            p6_e <= resize(b_e, 17) - resize(b_k(23 downto 8), 17);          -- opacity = 3 (E - k), from the key ramp
            -- 7: saw (Tri: folded into a lens), key, opacity x 3
            if s_wave = '1' then
                if p6_ph(23) = '1' then v_s12 := not p6_ph(22 downto 11); else v_s12 := p6_ph(22 downto 11); end if;
            else
                v_s12 := p6_ph(23 downto 12);
            end if;
            p7_s <= v_s12;
            p7_par <= p6_ph(24);
            p7_k <= p6_k - signed(resize(p6_lk, 16));
            p7_op <= resize(p6_e, 19) + shift_left(resize(p6_e, 19), 1);
            p7_p <= p6_p;
            -- 8: pairs (alternate scales: key +- 0.093, band +- 0.19 row), opacity (Q5)
            if p7_par = '1' then
                p8_k12 <= resize(p7_k, 17) + C_ALTK;
                p8_pp  <= resize(p7_p, 13) + C_ALTV;
            else
                p8_k12 <= resize(p7_k, 17) - C_ALTK;
                p8_pp  <= resize(p7_p, 13) - C_ALTV;
            end if;
            if p7_op < 0 then p8_op5 <= (others => '0');                     -- Q12 -> Q5, clamped to 0 .. 1
            elsif p7_op(18 downto 7) > 32 then p8_op5 <= to_unsigned(32, 6);
            else p8_op5 <= unsigned(p7_op(12 downto 7)); end if;
            p8_s <= p7_s;
            -- 9: d = saw - k; band row clamp; 4 (1 - saw)
            p9_d <= signed(resize(p8_s, 17)) - p8_k12;
            if p8_pp < 0 then p9_pc <= (others => '0');
            elsif p8_pp > 1279 then p9_pc <= to_unsigned(1279, 11);
            else p9_pc <= unsigned(p8_pp(10 downto 0)); end if;
            p9_sh <= resize(shift_right(to_unsigned(4096, 13) - resize(p8_s, 13), 4), 9);
            p9_s6 <= p8_s(11 downto 6);
            p9_op5 <= p8_op5;
            -- 10: 5 d; key level min(d, 4 (1 - saw)) (Q6); the band row, dithered between the two neighbours
            --     along the line on a 16-pixel ordered pattern (the output boxes average it into a smooth
            --     lerp); palette and key ROM addresses
            p10_d5 <= resize(p9_d, 20) + shift_left(resize(p9_d, 20), 2);
            v_kl := shift_right(p9_d, 6);
            v_sh := signed(resize(p9_sh, 17));
            if v_sh < v_kl then v_kl := v_sh; end if;
            if v_kl < 0 then v_kl := (others => '0'); elsif v_kl > 63 then v_kl := to_signed(63, 17); end if;
            v_th := o_wp(0) & o_wp(1) & o_wp(2) & o_wp(3);
            if p9_pc(7 downto 4) > v_th then v_j := p9_pc(10 downto 8) + 1; else v_j := p9_pc(10 downto 8); end if;
            r_c_a <= s_pal & v_j & p9_s6;
            r_k_a <= s_pal & unsigned(v_kl(5 downto 0));
            p10_op5 <= p9_op5;
            -- 11: ROM data (EBR); blend weight 0.5 + d / 0.2 (Q5)
            v_m := to_signed(16, 18) + resize(shift_right(p10_d5, 7), 18);
            if v_m < 0 then p11_m5 <= (others => '0');
            elsif v_m > 32 then p11_m5 <= to_unsigned(32, 6);
            else p11_m5 <= unsigned(v_m(5 downto 0)); end if;
            p11_op5 <= p10_op5;
            -- 12: fabric copies; key where min(blend, opacity) beats the dither (Q5 against 2 x 0 .. 15)
            p12_c <= r_c_q; p12_kq <= r_k_q;
            v_th := o_wp(3 downto 0) + 2;                       -- (read here, the pointer is already 2 past stage 10)
            v_th := (v_th(0) & v_th(1) & v_th(2) & v_th(3)) xor "0101";
            if p11_m5 < p11_op5 then v_w := p11_m5; else v_w := p11_op5; end if;
            if v_w > (v_th & '0') then p12_key <= '1'; else p12_key <= '0'; end if;
            -- 13: band or key colour
            if p12_key = '1' then v_c := p12_kq; else v_c := p12_c; end if;
            p13_y <= unsigned(v_c(31 downto 22)); p13_u <= unsigned(v_c(21 downto 12)); p13_v <= unsigned(v_c(11 downto 2));
            -- 14: carry
            p14_y <= p13_y; p14_u <= p13_u; p14_v <= p13_v;
            -- 15: compose; black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            else
                s_y15 <= p14_y; s_u15 <= p14_u; s_v15 <= p14_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the still's softness, and the HD HDMI bar fix),
    -- luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            o_wp <= o_wp + 1; o_rp <= o_rp + 1;
            o_ra(to_integer(o_wp)) <= std_logic_vector(s_y15) & std_logic_vector(s_u15) & std_logic_vector(s_v15) & "00";
            o_aq <= o_ra(to_integer(o_rp));
            o_aq2 <= o_aq;
        end if;
    end process p_ring;

    p_ringb : process(clk)
    begin
        if rising_edge(clk) then
            o_rb(to_integer(o_wp)) <= std_logic_vector(s_y17) & std_logic_vector(s_u17) & std_logic_vector(s_v17) & "00";
            o_bq <= o_rb(to_integer(o_rp));
            o_bq2 <= o_bq;
        end if;
    end process p_ringb;

    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to black (the off-avid input of both boxes) once 8 blank samples
            -- are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 16 sums, stage 17 output
            if s_bah = "00000000" then
                s_bay <= to_unsigned(512, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y15 - unsigned(o_aq2(31 downto 22));
                s_bau <= s_bau + s_u15 - unsigned(o_aq2(21 downto 12));
                s_bav <= s_bav + s_v15 - unsigned(o_aq2(11 downto 2));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            -- box B: stage 18 sums, stage 19 output
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(512, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y17 - unsigned(o_bq2(31 downto 22));
                s_bbu <= s_bbu + s_u17 - unsigned(o_bq2(21 downto 12));
                s_bbv <= s_bbv + s_v17 - unsigned(o_bq2(11 downto 2));
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

end architecture scales;
