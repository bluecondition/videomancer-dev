-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Kachina v0.1.1 -- Heirloom (video phase) from clip WIN_20231227_22_52_47 at 3.5 s.
--
-- v0.1.1: Etch default 480 -> 380 (a busy camera no longer shreds the side masks); the Osc lattice enters u at 3x
-- the camera's level (the etch gain is tripled per field in Osc mode), so Source = Osc visibly ripples the totem.
--
-- The still: a four-way mirrored totem mask -- a magenta ground with nested ovals top and bottom, blue bands
-- with red rims sweeping into a big X, yellow spindles and crescents, black cavities, and shaded red masks with
-- slit eyes (two big side masks, a top/bottom mask, small masks riding the X arms).  The clip shows that every
-- frame is the SAME top-left quadrant T(u, v) (u = |x - 990|, v = table row) read through a different vertical
-- mapping v(y): a soft-triangle V oscillator through a bipolar VCA (a 11.8 s triangle LFO, +-290 rows) that
-- scrolls up at ~128 lines/s.  No horizontal motion at all.
--
-- The patch, as built:
--   T      = the still's quadrant as a run table (kachina.py): 256 v slots -> topology-matched keyrow pairs,
--            each row a list of runs (length, 64-entry chain colour, slope); a row between the keyrows has
--            every boundary and colour interpolated
--   v(y)   = C + L * W(phi),  phi = phase of the V oscillator (period 1148 lines at the default),
--            W = asymmetric triangle + measured correction, L = A(P12) * breath LFO, folded into 0..587
--   u      = |x - 990| * width + etch * luma (camera or Osc), folded at the table edge (1024)
--   colour = chain palette (Kachina or Frost) at the expanded index + colour rotation
--
--   K1  Width     the H ramp's gain: one wide totem (0.35x) .. the still (512) .. four folded totems (4x)
--   K2  Period    the V oscillator's period: x4 (a giant stretched totem) .. the still .. x1/4 (four stacked)
--   K3  Breath    the amplitude LFO: steady (0) .. the clip's bipolar 11.8 s breath (512) .. 2.4 s (1023)
--   K4  Scroll    bipolar: the V wave scrolls down / still / up (700 = the clip's 128 lines/s up)
--   K5  Etch      how far the camera (or, at 3x, the Osc lattice) bends the totem's bands
--   K6  Colour    colouriser rotation along the chain, +-28 entries
--   S7  Wave      Triangle (mirrored top/bottom, the clip) / Saw (the top half repeated, hard seams)
--   S8  Centre    Cross (the still: the X at the centre) / Masks (inside-out: the side masks meet in the middle)
--   S9  Tear      Off / On (sync-tear bands of 8..71 virtual lines shifted 40..198 px)
--   S10 Palette   Kachina / Frost (the 23_03_24 clip's cyan, white and deep blue)
--   S11 Source    Video / Osc
--   P12 TOTEM     the VCA gain on the V oscillator: 0 = vertical stripes (the clip's 0.5 s look), 512 = the
--                 still, 1023 = 4x: the table folded four times per half wave, a stacked totem pole
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide); a line engine kicked at each line start
-- that computes the NEXT line's row (V wave, VCA, fold, span lookup, interpolation factor) and expands its run
-- list into one bank of a 2 x 1024 x 8 line buffer (radix-4 serial multiplier, a 9-step fetch pipeline and a
-- one-entry-per-clock writer); a 16-stage pixel path (C_LAT 16 in every mode).  Virtual 1080-line coordinates
-- (SD shows the centre 4:3).  Generated avid; output conditioning (U and V through a [1 2 2 2 2 2 2 2 1]/16 box
-- with the running-sum resync, Y through [1 2 1], luma 13/16) before the blanking gate.  Colours in the hardware
-- convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.kachina_pkg.all;
use work.all;

architecture kachina of program_top is
    constant C_LAT : integer := 16;   -- = processing_delay_clks

    constant C_PHM : integer := 8578449;    -- trough phase 587/1148 (Q24)
    constant C_KF  : integer := 16021;      -- falling slope: 8192 * 2^24 / C_PHM >> 8
    constant C_KR  : integer := 16763;      -- rising slope
    constant C_CV  : integer := 4696;       -- V centre 293.5 rows (Q4)

    type t_exp is array (0 to 63) of unsigned(8 downto 0);
    constant C_EXP : t_exp := (
        to_unsigned(256, 9), to_unsigned(259, 9), to_unsigned(262, 9), to_unsigned(264, 9), to_unsigned(267, 9), to_unsigned(270, 9), to_unsigned(273, 9), to_unsigned(276, 9),
        to_unsigned(279, 9), to_unsigned(282, 9), to_unsigned(285, 9), to_unsigned(288, 9), to_unsigned(292, 9), to_unsigned(295, 9), to_unsigned(298, 9), to_unsigned(301, 9),
        to_unsigned(304, 9), to_unsigned(308, 9), to_unsigned(311, 9), to_unsigned(314, 9), to_unsigned(318, 9), to_unsigned(321, 9), to_unsigned(325, 9), to_unsigned(328, 9),
        to_unsigned(332, 9), to_unsigned(336, 9), to_unsigned(339, 9), to_unsigned(343, 9), to_unsigned(347, 9), to_unsigned(350, 9), to_unsigned(354, 9), to_unsigned(358, 9),
        to_unsigned(362, 9), to_unsigned(366, 9), to_unsigned(370, 9), to_unsigned(374, 9), to_unsigned(378, 9), to_unsigned(382, 9), to_unsigned(386, 9), to_unsigned(391, 9),
        to_unsigned(395, 9), to_unsigned(399, 9), to_unsigned(403, 9), to_unsigned(408, 9), to_unsigned(412, 9), to_unsigned(417, 9), to_unsigned(421, 9), to_unsigned(426, 9),
        to_unsigned(431, 9), to_unsigned(435, 9), to_unsigned(440, 9), to_unsigned(445, 9), to_unsigned(450, 9), to_unsigned(454, 9), to_unsigned(459, 9), to_unsigned(464, 9),
        to_unsigned(470, 9), to_unsigned(475, 9), to_unsigned(480, 9), to_unsigned(485, 9), to_unsigned(490, 9), to_unsigned(496, 9), to_unsigned(501, 9), to_unsigned(506, 9));

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
    -- 2^(e/64) in Q8 for e = -128 .. 191 (e: signed Q6 octaves)
    function f_pow(e : signed(9 downto 0)) return unsigned is
        variable m : unsigned(11 downto 0);
    begin
        m := resize(C_EXP(to_integer(unsigned(e(5 downto 0)))), 12);
        case to_integer(e(9 downto 6)) is
            when -2     => return shift_right(m, 2);
            when -1     => return shift_right(m, 1);
            when 0      => return m;
            when 1      => return shift_left(m, 1);
            when others => return shift_left(m, 2);
        end case;
    end function;

    -- line buffer: two banks of 1024 chain indices (6.2), written by the line engine, read by the pixel path;
    -- the write bank is always the other one, so reads never meet the write address
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    signal s_lbw_we : std_logic := '0';
    signal s_lbw_a  : unsigned(10 downto 0) := (others => '0');
    signal s_lbw_d  : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_ra  : unsigned(10 downto 0) := (others => '0');
    signal s_lb_q   : std_logic_vector(7 downto 0) := x"00";

    -- ROMs (registered address and data, then a fabric copy)
    signal r_tab_a : unsigned(11 downto 0) := (others => '0');
    signal r_tab_q : std_logic_vector(15 downto 0) := (others => '0');
    signal x_tq    : std_logic_vector(15 downto 0) := (others => '0');
    signal r_pal_a : unsigned(8 downto 0) := (others => '0');
    signal r_pal_q : std_logic_vector(31 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_lm  : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_sd   : std_logic := '0';
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw scroll, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(380 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_saw, s_inside, s_tear, s_pal, s_src : std_logic := '0';

    -- per-field constants
    signal z_e1, z_e2, z_e3, z_es : signed(9 downto 0) := (others => '0');
    signal c_pw  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal z_a   : unsigned(19 downto 0) := (others => '0');
    signal z_neg : std_logic := '0';
    signal c_kx  : unsigned(11 downto 0) := to_unsigned(256, 12);    -- width gain, Q8
    signal c_dd  : unsigned(12 downto 0) := to_unsigned(256, 13);    -- u step per pixel, Q8 px
    signal c_d0  : signed(21 downto 0) := to_signed(-982 * 256, 22); -- u DDA start (x0 - 990) * kx + 8 px, Q8
    signal c_kpm : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal c_kp  : unsigned(15 downto 0) := to_unsigned(14614, 16);  -- V phase per virtual line (Q24)
    signal c_kpl : unsigned(17 downto 0) := to_unsigned(14614, 18);  -- per output line
    signal c_aykp : unsigned(23 downto 0) := to_unsigned(8578418, 24); -- 587 * kp
    signal c_spd : unsigned(12 downto 0) := (others => '0');         -- scroll, Q8 virtual lines per field
    signal c_dsc : unsigned(23 downto 0) := (others => '0');         -- scroll phase step per field (Q24)
    signal c_rm  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal c_rate : unsigned(15 downto 0) := to_unsigned(93, 16);    -- breath phase step per field (Q16)
    signal c_dep : unsigned(9 downto 0) := to_unsigned(512, 10);     -- breath depth, Q8 (512 = 2: bipolar)
    signal c_brq : signed(17 downto 0) := to_signed(32768, 18);      -- breath triangle, +-32768
    signal c_be  : signed(17 downto 0) := to_signed(32768, 18);      -- 1 - depth/2 (1 - B)
    signal c_amp : unsigned(14 downto 0) := to_unsigned(C_CV, 15);  -- VCA amplitude A, Q4 rows
    signal c_L   : signed(16 downto 0) := to_signed(C_CV, 17);       -- A * breath, Q4 rows
    signal c_La  : unsigned(15 downto 0) := to_unsigned(C_CV, 16);  -- |L|
    signal c_La3 : unsigned(17 downto 0) := to_unsigned(3 * C_CV, 18);
    signal c_Ln  : std_logic := '0';
    signal c_eg  : unsigned(9 downto 0) := to_unsigned(141, 10);     -- etch gain
    signal c_egx : unsigned(11 downto 0) := to_unsigned(141, 12);    -- etch gain applied (x3 in Osc mode)
    signal c_rot : unsigned(7 downto 0) := (others => '0');          -- colour rotation (6.2)
    signal c_phs : unsigned(23 downto 0) := (others => '0');         -- scroll phase (Q24)
    signal c_brp : unsigned(15 downto 0) := (others => '0');         -- breath LFO phase (Q16)
    signal c_phtop : unsigned(23 downto 0) := to_unsigned(31, 24);   -- V phase of virtual line 0

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
    signal e_lc   : unsigned(5 downto 0) := to_unsigned(63, 6);
    signal e_zk   : std_logic := '0';
    signal e_phi  : unsigned(23 downto 0) := (others => '0');
    signal e_rise : std_logic := '0';
    signal e_corr : signed(15 downto 0) := (others => '0');
    signal e_w    : signed(14 downto 0) := (others => '0');
    signal e_ms   : std_logic := '0';
    signal e_v    : signed(16 downto 0) := (others => '0');
    signal e_t1   : unsigned(23 downto 0) := (others => '0');
    signal e_sl   : unsigned(7 downto 0) := (others => '0');
    signal e_pa, e_pb : unsigned(11 downto 0) := (others => '0');
    signal e_va   : unsigned(9 downto 0) := (others => '0');
    signal e_inv  : unsigned(15 downto 0) := (others => '0');
    signal e_f    : unsigned(7 downto 0) := (others => '0');
    signal e_fneg : std_logic := '0';
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_a3   : unsigned(31 downto 0) := (others => '0');
    signal e_pd, e_pf, e_ma : unsigned(15 downto 0) := (others => '0');
    signal t_abs  : unsigned(7 downto 0) := (others => '0');
    signal t_abs3 : unsigned(9 downto 0) := (others => '0');
    signal e_dv   : signed(16 downto 0) := (others => '0');
    signal e_t3   : unsigned(15 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(31 downto 0) := (others => '0');
    signal m_sh   : unsigned(5 downto 0) := (others => '0');
    -- tear
    signal t_lfsr : unsigned(31 downto 0) := x"2545F491";
    signal t_seed : unsigned(31 downto 0) := x"9E3779B9";
    signal t_fc   : unsigned(1 downto 0) := (others => '0');
    signal t_cnt  : signed(16 downto 0) := (others => '0');
    signal t_on   : std_logic := '0';
    signal t_off  : signed(8 downto 0) := (others => '0');
    signal t_tn   : std_logic := '0';
    signal n_d0   : signed(21 downto 0) := to_signed(-982 * 256, 22);
    -- run expansion: fetch pipeline (x_*), next-run register (nx_*), writer (w_*)
    signal x_bank : std_logic := '0';
    signal l_bank : std_logic := '0';
    signal x_run, x_last : std_logic := '0';
    signal x_fs   : unsigned(3 downto 0) := (others => '0');
    signal x_pa, x_pb : unsigned(11 downto 0) := (others => '0');
    signal x_posa, x_posb : unsigned(10 downto 0) := (others => '0');
    signal x_ia, x_ib : unsigned(5 downto 0) := (others => '0');
    signal x_sla  : std_logic_vector(2 downto 0) := (others => '0');
    signal x_dp   : signed(11 downto 0) := (others => '0');
    signal x_di   : signed(6 downto 0) := (others => '0');
    signal x_pl, x_ph : signed(16 downto 0) := (others => '0');
    signal x_pil, x_pih : signed(11 downto 0) := (others => '0');
    signal x_pe   : signed(21 downto 0) := (others => '0');
    signal x_pi   : signed(15 downto 0) := (others => '0');
    signal nx_v   : std_logic := '0';
    signal nx_e   : unsigned(10 downto 0) := (others => '0');
    signal nx_acc : signed(13 downto 0) := (others => '0');
    signal nx_sl  : signed(7 downto 0) := (others => '0');
    signal w_run, w_on : std_logic := '0';
    signal w_u, w_e : unsigned(10 downto 0) := (others => '0');
    signal w_acc  : signed(13 downto 0) := (others => '0');
    signal w_sl   : signed(7 downto 0) := (others => '0');

    -- pixel path
    signal b_ua   : signed(21 downto 0) := (others => '0');
    signal p1_u, p2_u, p3_u : unsigned(12 downto 0) := (others => '0');
    signal p1_lm  : unsigned(7 downto 0) := (others => '0');
    signal p2_ea, p2_eb : unsigned(15 downto 0) := (others => '0');
    signal p3_e   : unsigned(19 downto 0) := (others => '0');
    signal p4_u   : unsigned(13 downto 0) := (others => '0');
    signal p5_f   : unsigned(9 downto 0) := (others => '0');
    signal p8_v   : unsigned(7 downto 0) := (others => '0');
    signal p11_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cau, s_cav : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer and ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lbw_we = '1' then lb(to_integer(s_lbw_a)) <= s_lbw_d; end if;
        end if;
    end process p_lb;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_tab_q <= C_KA_TAB(to_integer(r_tab_a));
            x_tq    <= r_tab_q;
            r_pal_q <= C_KA_PAL(to_integer(r_pal_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_t  : signed(18 downto 0);
        variable v_e  : signed(10 downto 0);
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
            -- scroll: bipolar, +-8 deadband
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
                s_saw    <= registers_in(6)(0);
                s_inside <= registers_in(6)(1);
                s_tear   <= registers_in(6)(2);
                s_pal    <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_xs     <= s_xsr;
                if s_xsr = 4 then s_sd <= '1'; else s_sd <= '0'; end if;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
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

            -- serial products: slot k loads at zc = 72 + 24 k, 16 iterations, result stable from load + 17
            -- (stored at load + 18).  The slot number is decoded at 70 + 24 k and the operands pre-muxed at
            -- 71 + 24 k, so no zc decode sits on the multiplier's path.
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
                    when 310 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 334 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  =>                                                            -- (x0 - 990) * kx
                        if s_sd = '1' then z_fa <= to_unsigned(750, 36); else z_fa <= to_unsigned(990, 36); end if;
                        z_fb <= resize(c_kx, 16);
                    when 1  => z_fa <= to_unsigned(14614, 36);       z_fb <= resize(c_kpm, 16);  -- kp
                    when 2  => z_fa <= resize(c_kp, 36);             z_fb <= resize(s_ys, 16);   -- kp per line
                    when 3  => z_fa <= resize(c_kp, 36);             z_fb <= to_unsigned(587, 16); -- 587 kp
                    when 4  => z_fa <= resize(s_vm, 36);             z_fb <= resize(s_vm, 16);   -- scroll^2
                    when 5  => z_fa <= resize(z_a, 36);              z_fb <= to_unsigned(69, 16);
                    when 6  => z_fa <= resize(c_spd, 36);            z_fb <= c_kp;               -- scroll step
                    when 7  => z_fa <= to_unsigned(93, 36);          z_fb <= resize(c_rm, 16);   -- breath rate
                    when 8  => z_fa <= resize(unsigned(to_signed(32768, 19) - resize(c_brq, 19)), 36);
                               z_fb <= resize(c_dep, 16);                                       -- depth (1 - B)
                    when 9  =>                                                                    -- A
                        if s_gp(12 downto 3) > 512 then
                            z_fa <= resize(s_gp(12 downto 3) - 512, 36); z_fb <= to_unsigned(1765, 16);
                        else
                            z_fa <= resize(s_gp(12 downto 3), 36);       z_fb <= to_unsigned(587, 16);
                        end if;
                    when 10 =>                                                                    -- A * |breath|
                        z_fa <= resize(c_amp, 36);
                        if c_be < 0 then z_fb <= resize(unsigned(-c_be), 16); else z_fb <= resize(unsigned(c_be), 16); end if;
                    when others => z_fa <= resize(s_g5(12 downto 3), 36); z_fb <= resize(s_g5(12 downto 3), 16); -- etch^2
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

            c_pw <= f_pow(z_es);                 -- the one 2^x table, time-shared by K1 / K2 / K3
            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's steps)
                    when 20 =>
                        if s_vneg = '1' then c_phs <= c_phs - c_dsc; else c_phs <= c_phs + c_dsc; end if;
                        c_brp <= c_brp + c_rate;
                    -- knob curves (Q6 octaves)
                    when 22 =>
                        v_k := s_g1(12 downto 3);
                        if v_k >= 512 then
                            z_e1 <= signed(resize(shift_right(v_k - 512, 2), 10));            -- 0 .. 2 oct
                        else
                            z_e1 <= -signed(resize(shift_right((512 - v_k) * to_unsigned(3, 2), 4), 10));  -- 0 .. -1.5 oct
                        end if;
                        v_e := signed(resize(s_g2(12 downto 3), 11)) - 512;
                        z_e2 <= resize(shift_right(v_e, 2), 10);                                 -- +-2 oct
                        v_k := s_g3(12 downto 3);
                        if v_k > 512 then
                            z_e3 <= signed(resize(shift_right((v_k - 512) * to_unsigned(147, 8), 9), 10));  -- 0 .. 2.3 oct
                            c_dep <= to_unsigned(512, 10);
                        else
                            z_e3 <= (others => '0');
                            c_dep <= v_k;
                        end if;
                        v_e := signed(resize(s_g6(12 downto 3), 11)) - 512;
                        if v_e > -9 and v_e < 9 then
                            c_rot <= (others => '0');
                        else
                            v_t := resize(v_e, 19);
                            v_t := shift_left(v_t, 3) - v_t;
                            c_rot <= unsigned(v_t(12 downto 5));                                 -- (k6 - 512) * 7 / 32, mod 256
                        end if;
                    when 23 =>
                        z_es <= z_e1;
                        -- breath triangle: B = 32768 - 2 |frac(phase + 1/2) - 1/2|
                        v_t := signed(resize(c_brp + 32768, 19)) - 32768;
                        if v_t < 0 then v_t := -v_t; end if;
                        c_brq <= to_signed(32768, 18) - resize(shift_left(v_t, 1), 18);
                    when 24 => z_es <= z_e2;
                    when 25 => c_kx <= c_pw; z_es <= z_e3;
                    when 26 => c_kpm <= c_pw;
                    when 27 => c_rm <= c_pw;
                    when 28 =>
                        case to_integer(s_xs) is
                            when 4      => c_dd <= resize(shift_left(c_kx, 1), 13);
                            when 3      => c_dd <= resize(c_kx, 13) + resize(shift_right(c_kx, 1), 13);
                            when others => c_dd <= resize(c_kx, 13);
                        end case;
                    -- products
                    when 88  => c_d0  <= -signed(resize(f_acc(20 downto 0), 22));
                    when 89  => c_d0  <= c_d0 + signed(resize(shift_left(resize(c_dd, 17), 3), 22));    -- + 8 pixels (output lag)
                    when 112 => c_kp  <= f_acc(23 downto 8);
                    when 136 => c_kpl <= f_acc(25 downto 8);
                    when 160 => c_aykp <= f_acc(23 downto 0);
                    when 184 => z_a <= f_acc(19 downto 0);
                    when 208 => c_spd <= f_acc(24 downto 12);
                    when 232 => c_dsc <= f_acc(31 downto 8);
                    when 256 => c_rate <= f_acc(23 downto 8);
                    when 280 => c_be <= to_signed(32768, 18) - signed(resize(f_acc(26 downto 9), 18));
                    when 304 =>
                        if s_gp(12 downto 3) > 512 then
                            c_amp <= to_unsigned(C_CV, 15) + f_acc(20 downto 6);
                        else
                            c_amp <= f_acc(20 downto 6);
                        end if;
                    when 328 =>
                        if c_be < 0 then c_L <= -signed(resize(f_acc(31 downto 15), 17));
                        else c_L <= signed(resize(f_acc(31 downto 15), 17)); end if;
                    when 330 =>
                        c_Ln <= c_L(16);
                        if c_L < 0 then c_La <= resize(unsigned(-c_L), 16); else c_La <= resize(unsigned(c_L), 16); end if;
                    when 331 => c_La3 <= resize(c_La, 18) + shift_left(resize(c_La, 18), 1);
                    when 352 => c_eg <= f_acc(19 downto 10);
                    when 353 =>                                   -- Osc: the lattice enters at 3x the camera's level
                        if s_src = '1' then c_egx <= resize(c_eg, 12) + shift_left(resize(c_eg, 12), 1);
                        else c_egx <= resize(c_eg, 12); end if;
                    when 354 => c_phtop <= to_unsigned(C_PHM, 24) - c_aykp;
                    when 355 => c_phtop <= c_phtop + c_phs;
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
    -- Source side: oscillator (2-D triangle lattice, drifting) or camera luma
    -- (Y >> 2) - 16, one register, used only as the etch operand
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
            if s_src = '1' then
                s_lm <= s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if data_in.avid = '1' and v_l > 16 then s_lm <= v_l - 16; else s_lm <= (others => '0'); end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line: V phase -> wave W -> v = C + L W, folded -> slot ->
    -- span (keyrows A, B, va, 1/len) -> f; then expands the row (boundaries and
    -- colours interpolated at f) into the free line-buffer bank.  Radix-4 serial
    -- multiplier: load, 8 iterations (16-bit b), result at load + 9.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d  : signed(24 downto 0);
        variable v_w  : signed(15 downto 0);
        variable v_p  : signed(17 downto 0);
        variable v_r  : unsigned(31 downto 0);
        variable v_c  : unsigned(21 downto 0);
        variable v_x  : std_logic_vector(15 downto 0);
        variable v_sq : signed(7 downto 0);
        variable v_q  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            -- radix-4 serial multiplier
            case m_b(1 downto 0) is
                when "01"   => m_acc <= m_acc + m_a;
                when "10"   => m_acc <= m_acc + (m_a(30 downto 0) & '0');
                when "11"   => m_acc <= m_acc + m_a3;
                when others => null;
            end case;
            m_a  <= m_a(29 downto 0) & "00";
            m_a3 <= m_a3(29 downto 0) & "00";
            m_b  <= "00" & m_b(15 downto 2);

            s_lbw_we <= '0';

            if e_zk = '1' or g_start = '1' then
                e_lc <= (others => '0');
                if e_zk = '1' then
                    e_phi <= c_phtop;
                    t_cnt <= (others => '0'); t_on <= '0'; t_off <= (others => '0');
                    t_fc <= t_fc + 1;                                           -- the tear pattern holds 4 fields
                    if t_fc = 3 then t_seed <= t_lfsr; else t_lfsr <= t_seed; end if;
                else
                    e_phi <= e_phi + resize(c_kpl, 24);
                    l_bank <= x_bank;
                    x_bank <= not x_bank;
                    -- tear bands (heights in virtual lines, Q8)
                    t_lfsr <= t_lfsr xor shift_left(t_lfsr, 13);
                    if t_cnt <= signed(resize(s_ys, 17)) then
                        if t_on = '1' then
                            t_on <= '0'; t_off <= (others => '0');
                            t_cnt <= signed(resize(t_lfsr(7 downto 0) & x"00", 17)) + 2048;       -- gap 8 .. 263 lines
                        else
                            t_on <= '1';
                            v_c := resize(t_lfsr(14 downto 8), 22) + resize(t_lfsr(20 downto 16), 22) + 40;  -- 40 .. 198 px
                            if t_lfsr(21) = '1' then t_off <= -signed(resize(v_c(8 downto 0), 9));
                            else t_off <= signed(resize(v_c(8 downto 0), 9)); end if;
                            t_cnt <= signed(resize(t_lfsr(29 downto 24) & x"00", 17)) + 2048;     -- 8 .. 71 lines
                        end if;
                    else
                        t_cnt <= t_cnt - signed(resize(s_ys, 17));
                    end if;
                end if;
                x_run <= '0'; w_run <= '0'; nx_v <= '0'; x_fs <= (others => '0');
            else
                -- xorshift second and third steps (one per clock, settle long before the next line)
                if e_lc = 1 then t_lfsr <= t_lfsr xor shift_right(t_lfsr, 17); end if;
                if e_lc = 2 then t_lfsr <= t_lfsr xor shift_left(t_lfsr, 5); end if;
                if e_lc /= 63 then e_lc <= e_lc + 1; end if;
                case to_integer(e_lc) is
                    when 0 =>
                        r_tab_a <= to_unsigned(256, 12) + resize(e_phi(23 downto 16), 12);  -- CORR
                        if e_phi < C_PHM then e_rise <= '0'; else e_rise <= '1'; end if;
                        e_pf <= e_phi(23 downto 8);
                        e_pd <= e_phi(23 downto 8) - to_unsigned(C_PHM / 256, 16);
                    when 1 =>
                        if e_rise = '1' then e_ma <= e_pd; else e_ma <= e_pf; end if;
                    when 2 =>
                        m_a <= resize(e_ma, 32); m_a3 <= resize(e_ma, 32) + shift_left(resize(e_ma, 32), 1);
                        if e_rise = '1' then m_b <= to_unsigned(C_KR, 16); else m_b <= to_unsigned(C_KF, 16); end if;
                        m_acc <= (others => '0');
                    when 3 =>
                        e_corr <= signed(x_tq);
                    when 4 =>
                        if t_off < 0 then t_abs <= resize(unsigned(-t_off), 8); else t_abs <= resize(unsigned(t_off), 8); end if;
                        t_tn <= t_off(8);
                    when 5 =>
                        t_abs3 <= resize(t_abs, 10) + shift_left(resize(t_abs, 10), 1);
                    when 11 =>
                        -- W (Q12): asymmetric triangle + correction, or the saw (two falls per period)
                        if s_saw = '1' then
                            v_w := to_signed(4096, 16) - signed(resize(shift_right(e_phi(22 downto 0) & '0', 11), 16));
                        elsif e_rise = '1' then
                            v_w := to_signed(-4096, 16) + signed(resize(m_acc(28 downto 16), 16)) + e_corr;
                        else
                            v_w := to_signed(4096, 16) - signed(resize(m_acc(28 downto 16), 16)) + e_corr;
                        end if;
                        e_w <= v_w(14 downto 0);
                    when 12 =>
                        -- |L| * |W|
                        m_a <= resize(c_La, 32); m_a3 <= resize(c_La3, 32);
                        if e_w < 0 then m_b <= resize(unsigned(-e_w), 16); else m_b <= resize(unsigned(e_w), 16); end if;
                        e_ms <= c_Ln xor e_w(14);
                        m_acc <= (others => '0');
                    when 21 =>
                        if e_ms = '1' then
                            e_v <= to_signed(C_CV, 17) - signed(resize(m_acc(31 downto 12), 17));
                        else
                            e_v <= to_signed(C_CV, 17) + signed(resize(m_acc(31 downto 12), 17));
                        end if;
                    when 22 =>   -- fold into one period of 1176 rows (Q4 18816)
                        if e_v < 0 then e_v <= e_v + 18816;
                        elsif e_v >= 18816 then e_v <= e_v - 18816; end if;
                    when 23 =>   -- mirror at 588 rows
                        if e_v >= 9408 then e_v <= 18815 - e_v; end if;
                    when 24 =>   -- slot = v * 445 >> 14 (445 = 512 - 64 - 3)
                        e_t1 <= shift_left(resize(unsigned(e_v(13 downto 0)), 24), 9) - shift_left(resize(unsigned(e_v(13 downto 0)), 24), 6);
                        e_t3 <= resize(unsigned(e_v(13 downto 0)), 16) + shift_left(resize(unsigned(e_v(13 downto 0)), 16), 1);
                    when 25 =>
                        v_r := resize(e_t1, 32) - resize(e_t3, 32);
                        e_sl <= v_r(21 downto 14);
                    when 26 => r_tab_a <= resize(e_sl, 12);
                    when 29 => r_tab_a <= to_unsigned(512, 12) + shift_left(resize(unsigned(x_tq(7 downto 0)), 12), 2);
                    when 30 => r_tab_a <= r_tab_a + 1;
                    when 31 => r_tab_a <= r_tab_a + 1;
                    when 32 => r_tab_a <= r_tab_a + 1; e_pa <= unsigned(x_tq(11 downto 0));
                    when 33 => e_pb <= unsigned(x_tq(11 downto 0));
                    when 34 => e_va <= unsigned(x_tq(9 downto 0));
                    when 35 =>
                        e_inv <= unsigned(x_tq);
                        e_dv <= e_v - signed(resize(shift_left(resize(e_va, 14), 4), 17));
                    when 36 =>
                        if e_dv < 0 then
                            e_fneg <= '1'; m_a <= (others => '0'); m_a3 <= (others => '0');
                        else
                            e_fneg <= '0'; m_a <= resize(unsigned(e_dv(15 downto 0)), 32);
                            m_a3 <= resize(unsigned(e_dv(15 downto 0)), 32) + shift_left(resize(unsigned(e_dv(15 downto 0)), 32), 1);
                        end if;
                        m_b <= e_inv; m_acc <= (others => '0');
                    when 45 =>
                        if e_fneg = '1' then e_f <= (others => '0');
                        elsif m_acc(31 downto 20) /= 0 then e_f <= (others => '1');
                        else e_f <= m_acc(19 downto 12); end if;
                    when 46 =>
                        -- start the expansion; tear product |off| * kx on the multiplier meanwhile
                        x_pa <= e_pa; x_pb <= e_pb; x_posa <= (others => '0'); x_posb <= (others => '0');
                        x_fs <= (others => '0'); x_last <= '0'; x_run <= '1'; nx_v <= '0';
                        w_u <= (others => '0'); w_on <= '0'; w_run <= '1';
                        m_a <= resize(t_abs, 32); m_a3 <= resize(t_abs3, 32);
                        m_b <= resize(c_kx, 16); m_acc <= (others => '0');
                    when 55 =>
                        if s_tear = '0' then n_d0 <= c_d0;
                        elsif t_tn = '1' then n_d0 <= c_d0 - signed(resize(m_acc(20 downto 0), 22));
                        else n_d0 <= c_d0 + signed(resize(m_acc(20 downto 0), 22)); end if;
                    when others => null;
                end case;

                -- fetch pipeline: one run (A and B words) at a time
                if x_run = '1' then
                    case to_integer(x_fs) is
                        when 0 =>
                            if x_last = '0' then r_tab_a <= x_pa; x_pa <= x_pa + 1; x_fs <= to_unsigned(1, 4); end if;
                        when 1 => r_tab_a <= x_pb; x_pb <= x_pb + 1; x_fs <= to_unsigned(2, 4);
                        when 2 => x_fs <= to_unsigned(3, 4);
                        when 3 =>
                            x_posa <= x_posa + resize(unsigned(x_tq(15 downto 9)), 11) + 1;
                            x_ia <= unsigned(x_tq(8 downto 3)); x_sla <= x_tq(2 downto 0);
                            x_fs <= to_unsigned(4, 4);
                        when 4 =>
                            x_posb <= x_posb + resize(unsigned(x_tq(15 downto 9)), 11) + 1;
                            x_ib <= unsigned(x_tq(8 downto 3));
                            if x_posa(10) = '1' then x_last <= '1'; end if;
                            x_fs <= to_unsigned(5, 4);
                        when 5 =>
                            x_dp <= signed(resize(x_posb, 12)) - signed(resize(x_posa, 12));
                            x_di <= signed(resize(x_ib, 7)) - signed(resize(x_ia, 7));
                            x_fs <= to_unsigned(6, 4);
                        when 6 =>
                            x_pl  <= resize(x_dp * signed('0' & e_f(3 downto 0)), 17);
                            x_ph  <= resize(x_dp * signed('0' & e_f(7 downto 4)), 17);
                            x_pil <= resize(x_di * signed('0' & e_f(3 downto 0)), 12);
                            x_pih <= resize(x_di * signed('0' & e_f(7 downto 4)), 12);
                            x_fs <= to_unsigned(7, 4);
                        when 7 =>
                            x_pe <= resize(x_pl, 22) + shift_left(resize(x_ph, 22), 4) + 128;
                            x_pi <= resize(x_pil, 16) + shift_left(resize(x_pih, 16), 4);
                            x_fs <= to_unsigned(8, 4);
                        when others =>
                            if nx_v = '0' then
                                v_q := signed(resize(x_posa, 12)) + resize(shift_right(x_pe, 8), 12);
                                nx_e <= unsigned(v_q(10 downto 0));
                                v_p := signed(resize(shift_left(resize(x_ia, 8), 2), 18)) + resize(shift_right(x_pi, 6), 18);
                                nx_acc <= resize(shift_left(v_p, 4), 14);
                                case x_sla is
                                    when "001"  => v_sq := to_signed(8, 8);
                                    when "010"  => v_sq := to_signed(16, 8);
                                    when "011"  => v_sq := to_signed(32, 8);
                                    when "100"  => v_sq := to_signed(-8, 8);
                                    when "101"  => v_sq := to_signed(-16, 8);
                                    when "110"  => v_sq := to_signed(-32, 8);
                                    when "111"  => v_sq := to_signed(4, 8);
                                    when others => v_sq := to_signed(0, 8);
                                end case;
                                if s_sd = '1' then nx_sl <= shift_left(v_sq, 1); else nx_sl <= v_sq; end if;
                                nx_v <= '1';
                                x_fs <= (others => '0');
                            end if;
                    end case;
                end if;

                -- writer: one line-buffer entry per clock (two table px per entry in SD)
                if w_run = '1' then
                    if w_u(10) = '1' then
                        w_run <= '0'; x_run <= '0';
                    elsif w_on = '1' and w_u < w_e then
                        s_lbw_we <= '1';
                        if s_sd = '1' then s_lbw_a <= x_bank & '0' & w_u(9 downto 1);
                        else s_lbw_a <= x_bank & w_u(9 downto 0); end if;
                        if w_acc < 0 then s_lbw_d <= (others => '0');
                        elsif w_acc(13 downto 12) /= "00" then s_lbw_d <= (others => '1');
                        else s_lbw_d <= std_logic_vector(w_acc(11 downto 4)); end if;
                        w_acc <= w_acc + resize(w_sl, 14);
                        if s_sd = '1' then w_u <= w_u + 2; else w_u <= w_u + 1; end if;
                    elsif nx_v = '1' then
                        w_e <= nx_e; w_acc <= nx_acc; w_sl <= nx_sl; w_on <= '1';
                        nx_v <= '0';
                    end if;
                end if;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 0 = the u DDA of the pixel, aligned with g_avid)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a  : unsigned(21 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_i  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 0: u DDA (Q8 px from the axis, signed)
            if g_start = '1' then
                b_ua <= n_d0;
            elsif g_avid = '1' then
                b_ua <= b_ua + signed(resize(c_dd, 22));
            end if;
            -- 1: |u|, camera / Osc luma
            if b_ua < 0 then v_a := unsigned(-b_ua); else v_a := unsigned(b_ua); end if;
            p1_u  <= v_a(20 downto 8);
            p1_lm <= s_lm;
            -- 2: etch partials (luma nibbles x gain)
            p2_ea <= p1_lm(3 downto 0) * c_egx;
            p2_eb <= p1_lm(7 downto 4) * c_egx;
            p2_u  <= p1_u;
            -- 3: etch
            p3_e <= resize(p2_ea, 20) + shift_left(resize(p2_eb, 20), 4);
            p3_u <= p2_u;
            -- 4: u + etch (u px; the fold below only uses the low 11 bits)
            p4_u <= resize(p3_u, 14) + resize(p3_e(19 downto 9), 14);
            -- 5: fold at the table edge (1024)
            if p4_u(10) = '1' then p5_f <= not p4_u(9 downto 0); else p5_f <= p4_u(9 downto 0); end if;
            -- 6: line-buffer address (Centre = Masks: inside-out, |989 - u|)
            if s_inside = '1' then
                v_d := to_signed(989, 11) - signed(resize(p5_f, 11));
                if v_d < 0 then v_d := -v_d; end if;
                v_i := unsigned(v_d(9 downto 0));
            else
                v_i := p5_f;
            end if;
            if s_sd = '1' then s_lb_ra <= l_bank & '0' & v_i(9 downto 1);
            else s_lb_ra <= l_bank & v_i; end if;
            -- 7: line buffer (EBR); 8: fabric copy
            p8_v <= unsigned(s_lb_q);
            -- 9: palette address (+ colour rotation)
            r_pal_a <= s_pal & (p8_v + c_rot);
            -- 10: palette (EBR); 11: fabric copy
            p11_q <= r_pal_q;
            -- 12: compose; black outside the generated avid
            if s_gav_sr(10) = '0' then
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            else
                s_y15 <= unsigned(p11_q(31 downto 22)); s_u15 <= unsigned(p11_q(21 downto 12)); s_v15 <= unsigned(p11_q(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(10);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through a [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums, with the running-sum resync), Y through [1 2 1] centred on
    -- the same sample, luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_yy : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 13 sums, stage 14 output; the 8-sums snap back to neutral once 8 blank samples are in
            s_cay(0) <= s_y15; s_cau(0) <= s_u15; s_cav(0) <= s_v15;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bau <= s_bau + s_u15 - s_cau(7);
                s_bav <= s_bav + s_v15 - s_cav(7);
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            -- luma [1 2 1] centred on the box's centre sample (4 back)
            v_yy := resize(s_cay(3), 12) + shift_left(resize(s_cay(4), 12), 1) + resize(s_cay(5), 12);
            s_y17 <= v_yy(11 downto 2);
            -- 15: luma 13/16
            if s_y17 > 64 then v_dl := s_y17 - 64; else v_dl := (others => '0'); end if;
            s_y20 <= s_y17 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u20 <= s_u17; s_v20 <= s_v17;
            -- 16: blanking gate
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

end architecture kachina;
