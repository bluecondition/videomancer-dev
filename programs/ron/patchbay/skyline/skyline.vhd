-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Skyline v0.1.1 -- Patchbay: a sample-and-hold bar graph that dances like a spectrum analyser.
--
-- The patch:
--   slow noise (an LFSR, resampled per column at the Speed clock) x two drifting formant LFOs
--     + source (camera luma, or the Osc lattice: the mean of each column over the field, AC-coupled)
--     + Level (a DC offset)                 -> mixer
--   mixer x Drive                           -> H-rate sample-and-hold (one value per column, with droop)
--   S&H -> envelope follower (fast attack, slow release)       = the bar height
--       -> peak-hold (hold, then a fall under gravity)         = the cap
--   V ramp (from the horizon, mirrored below it) vs bar height -> comparator = the bar key
--   V ramp (+ a little H ramp + the overdrive heat)            -> colouriser (green / yellow / red ladder)
--   bar key -> VCA (lit / dim ladder / reflection fade), the cap keyed from the complement of the wheel
--
--   K1  Columns   8 .. 96 columns across the width
--   K2  Segments  LED segment pitch 32 .. 283 lines, solid bars at the top of the travel
--   K3  Level     DC offset into the mixer: sparse towers .. the default .. a high wall
--   K4  Speed     bipolar: the S&H clock rate and the LFO drift (left / right)
--   K5  Colour    rotates the colouriser: cool ladder .. green / yellow / red (512) .. hot ladder
--   K6  Camera    source strength (camera or Osc) into the mixer: 0 = none (the pure noise analyser),
--                 25 % = v0.1.0's fixed x4, 50 % (default) = x8, 100 % = x16.  (Decay is fixed at
--                 v0.1.0's default 600: release 12 lines a field, caps hold 0.7 s.)
--   S7  Shape     Spectrum (noise under two formant humps) / Wave (a travelling sine plus noise)
--   S8  Mirror    Reflection (a dim rippled mirror below the horizon) / Centre (bars grow both ways)
--   S9  Peaks     Caps / Trails (the space under the peak glows instead)
--   S10 Gap       LEDs (gaps between the columns) / Solid (a fused skyline)
--   S11 Source    Video / Osc
--   P12 DRIVE     gain into the bars: a murmur .. everything pinned and white hot
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide); a column engine in the zone
-- (64 clocks per column: state RAM read, three serial products, envelope / peak update, display RAM
-- write); a per-line engine (the mirrored V ramp, its segment phase, the per-line constants); a column
-- phase accumulator per pixel and a 15-stage pixel path; a video accumulator (row sum per column,
-- read-modify-write into a block RAM on 128 rows of the field).  Virtual 1080-line coordinates; the
-- columns span the real width.  Generated avid; output conditioning (Y, U and V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16) before the blanking gate.  Colours in the hardware
-- convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K5 = Colour (was K6); K5 Decay fixed at 600 (C_KFIX_5).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.skyline_pkg.all;
use work.all;

architecture skyline of program_top is
    constant C_KFIX_5 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(600, 10));   -- K6 camera pass: fixed knob
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant C_HY0R : integer := 199680;    -- Reflection: the horizon, 780 virtual lines (Q8)
    constant C_HY0C : integer := 190080;    -- Centre: 540 lines x 1.375 (Q8)
    constant C_HOT  : integer := 148;       -- the ladder index saturates here (white hot)
    constant C_HZ   : integer := 160;       -- horizon line's wheel index
    constant C_KY   : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_KC   : unsigned(9 downto 0) := to_unsigned(512, 10);

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
    -- x step in h per pixel: 1080 = 2, 720p = 3, SD = 4
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

    -- block RAMs: display (h, p, heat per column), engine state (2 words per column), video sums
    type t_r128 is array (0 to 127) of std_logic_vector(31 downto 0);
    type t_r256 is array (0 to 255) of std_logic_vector(31 downto 0);
    signal dram : t_r128 := (others => (others => '0'));
    signal vram : t_r128 := (others => (others => '0'));
    signal sram : t_r256 := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of dram : signal is true;
    attribute no_rw_check of vram : signal is true;
    attribute no_rw_check of sram : signal is true;
    signal d_ra : unsigned(6 downto 0) := (others => '0');
    signal d_rq : std_logic_vector(31 downto 0) := (others => '0');
    signal d_we : std_logic := '0';
    signal v_ra, v_wa : unsigned(6 downto 0) := (others => '0');
    signal v_rq, v_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal v_rd : unsigned(23 downto 0) := (others => '0');
    signal v_we : std_logic := '0';
    signal s_ra, s_wa : unsigned(7 downto 0) := (others => '0');
    signal s_rq, s_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_we : std_logic := '0';

    -- ROMs (registered address and data)
    signal r_tb_a : unsigned(8 downto 0) := (others => '0');
    signal r_tb_q : unsigned(15 downto 0) := (others => '0');
    signal r_pf_a : unsigned(8 downto 0) := (others => '0');
    signal r_pf_q : std_logic_vector(31 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_lum  : unsigned(7 downto 0) := (others => '0');

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
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(300 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(100 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gk : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);   -- K6 camera (glided x8)
    signal c_vsc : unsigned(13 downto 0) := to_unsigned(580, 14);      -- column-sum scale x camera / 256
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(680, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_wave, s_centre, s_trail, s_fuse, s_src : std_logic := '0';

    -- per-field constants
    signal c_n    : unsigned(6 downto 0) := to_unsigned(34, 7);       -- columns
    signal c_rinc : unsigned(12 downto 0) := to_unsigned(2185, 13);   -- 2^22 / width
    signal c_inc  : unsigned(19 downto 0) := to_unsigned(74290, 20);  -- column phase per pixel, Q22
    signal c_rcp  : unsigned(15 downto 0) := to_unsigned(1928, 16);   -- 65536 / columns
    signal z_t    : unsigned(10 downto 0) := (others => '0');
    signal z_p9   : unsigned(13 downto 0) := (others => '0');
    signal z_o7   : unsigned(12 downto 0) := (others => '0');
    signal z_dk   : unsigned(12 downto 0) := (others => '0');
    signal c_prob : unsigned(9 downto 0) := to_unsigned(62, 10);
    signal c_d1, c_d2, c_dw : unsigned(11 downto 0) := (others => '0');
    -- Decay fixed at 600 (K6 camera pass): release (423^2 >> 10) + 16 = 190 Q4 lines a field, hold 37 + 6
    -- fields, gravity 190 / 8 + 2 = 25
    constant c_rel  : unsigned(10 downto 0) := to_unsigned(190, 11);
    constant c_hold : unsigned(6 downto 0) := to_unsigned(43, 7);
    constant c_grav : unsigned(7 downto 0) := to_unsigned(25, 8);
    signal c_g    : unsigned(10 downto 0) := to_unsigned(461, 11);
    signal c_bias : unsigned(8 downto 0) := (others => '0');
    signal c_ofs  : signed(10 downto 0) := (others => '0');
    signal c_koff : unsigned(7 downto 0) := (others => '0');
    signal c_hzi  : unsigned(7 downto 0) := to_unsigned(C_HZ, 8);
    signal c_pitch : unsigned(8 downto 0) := to_unsigned(60, 9);
    signal c_pgapa, c_pgapb : unsigned(8 downto 0) := to_unsigned(45, 9);   -- gap threshold above / below the axis
    signal c_gma, c_gmb : unsigned(4 downto 0) := to_unsigned(2, 5);        -- 2.25 raster rows, in hy lines
    signal c_cta, c_ctb : unsigned(4 downto 0) := to_unsigned(12, 5);       -- solid cap thickness
    signal z_g2a, z_g2b : unsigned(13 downto 0) := (others => '0');
    signal z_gxa, z_gxb : unsigned(6 downto 0) := (others => '0');
    signal z_ona, z_onb : std_logic := '1';
    signal c_solid : std_logic := '0';
    signal c_dka, c_dkb : unsigned(12 downto 0) := to_unsigned(256, 13);
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);     -- beam start x (h)
    signal c_fld  : unsigned(7 downto 0) := (others => '0');
    signal c_vmean : unsigned(11 downto 0) := (others => '0');

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

    -- column engine
    signal e_run, e_first, e_fin : std_logic := '0';
    signal e_boot : std_logic := '1';
    signal e_cs   : unsigned(5 downto 0) := (others => '0');
    signal e_ci   : unsigned(6 downto 0) := (others => '0');
    signal e_pc   : unsigned(4 downto 0) := (others => '0');
    signal e_lf   : unsigned(31 downto 0) := x"ACE1F00D";
    signal e_mreq : std_logic := '0';
    signal e_msel : unsigned(1 downto 0) := (others => '0');
    signal e_ph1, e_ph2, e_phw : unsigned(15 downto 0) := (others => '0');
    signal e_xf, e_a1, e_a2 : unsigned(15 downto 0) := (others => '0');
    signal e_s1, e_s2 : unsigned(7 downto 0) := (others => '0');
    signal e_r    : unsigned(9 downto 0) := (others => '0');
    signal e_v    : unsigned(11 downto 0) := (others => '0');
    signal e_hd   : unsigned(6 downto 0) := (others => '0');
    signal e_h, e_p : unsigned(13 downto 0) := (others => '0');
    signal e_ht   : unsigned(3 downto 0) := (others => '0');
    signal e_e1   : unsigned(9 downto 0) := (others => '0');
    signal e_e2   : unsigned(10 downto 0) := (others => '0');
    signal e_e3   : signed(11 downto 0) := (others => '0');
    signal e_t    : unsigned(8 downto 0) := (others => '0');
    signal e_a    : unsigned(9 downto 0) := (others => '0');
    signal e_b    : unsigned(9 downto 0) := (others => '0');
    signal e_m    : unsigned(11 downto 0) := (others => '0');   -- 4 x mean luma x camera / 256
    signal e_vsum : unsigned(18 downto 0) := (others => '0');
    signal e_sr   : signed(13 downto 0) := (others => '0');
    signal e_src  : signed(11 downto 0) := (others => '0');
    signal e_tt   : unsigned(10 downto 0) := (others => '0');
    signal e_rs1, e_rs : signed(13 downto 0) := (others => '0');
    signal e_rw, e_raw : unsigned(12 downto 0) := (others => '0');
    signal e_d    : unsigned(15 downto 0) := (others => '0');
    signal e_tg, e_ct : unsigned(13 downto 0) := (others => '0');
    signal e_tq  : unsigned(3 downto 0) := (others => '0');
    signal e_dh, e_hr, e_pf : signed(14 downto 0) := (others => '0');
    signal e_ha   : unsigned(13 downto 0) := (others => '0');
    signal e_vn   : unsigned(11 downto 0) := (others => '0');
    signal e_cge  : std_logic := '0';
    signal e_vclr : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine: the mirrored V ramp hy (Q8 virtual lines above / below the axis), its segment phase
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal y_init : std_logic := '0';
    signal y_oc   : unsigned(1 downto 0) := (others => '0');
    signal y_hy   : unsigned(17 downto 0) := to_unsigned(C_HY0R, 18);
    signal y_ph   : signed(18 downto 0) := (others => '0');
    signal y_hq   : unsigned(9 downto 0) := (others => '0');
    signal y_t    : signed(18 downto 0) := (others => '0');
    signal y_dn, y_refl, y_sp : std_logic := '1';
    signal y_ix   : unsigned(7 downto 0) := (others => '0');
    signal y_lbr  : signed(5 downto 0) := (others => '0');
    signal y_rip  : std_logic := '0';
    signal n_cmp, n_cmp2, n_cmp3 : signed(11 downto 0) := (others => '0');
    signal n_idx  : unsigned(7 downto 0) := (others => '0');
    signal n_lb   : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal n_gap, n_top, n_refl, n_hz, n_samp : std_logic := '0';
    signal l_cmp, l_cmp2, l_cmp3 : signed(11 downto 0) := (others => '0');
    signal l_idx  : unsigned(7 downto 0) := (others => '0');
    signal l_lb   : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal l_gap, l_top, l_refl, l_hz, l_samp : std_logic := '0';

    -- beam
    signal b_acc : unsigned(28 downto 0) := (others => '0');
    signal b_x   : unsigned(11 downto 0) := (others => '0');
    signal g_st1, g_st2 : std_logic := '0';

    -- pixel path
    signal q2_col : unsigned(6 downto 0) := (others => '0');
    signal q2_fr, q3_fr, q4_fr : unsigned(3 downto 0) := (others => '0');
    signal q2_tilt, q3_tilt, q4_tilt : unsigned(3 downto 0) := (others => '0');
    signal q2_gap, q3_blank, q4_blank, q5_blank, q6_blank, q7_blank : std_logic := '0';
    signal q4_h, q4_p, q5_p : unsigned(9 downto 0) := (others => '0');
    signal q4_ht, q5_ht : unsigned(3 downto 0) := (others => '0');
    signal q5_he  : signed(10 downto 0) := (others => '0');
    signal q5_idx : unsigned(7 downto 0) := (others => '0');
    signal q5_sh, q6_sh : unsigned(2 downto 0) := (others => '0');
    signal q6_lit, q6_bp, q6_ac2, q6_bp3 : std_logic := '0';
    signal q6_td  : signed(11 downto 0) := (others => '0');
    signal q6_idx : unsigned(8 downto 0) := (others => '0');
    signal q7_lit, q7_bp, q7_cap : std_logic := '0';
    signal q7_blit, q7_tl, q7_bun : unsigned(4 downto 0) := (others => '0');
    signal q7_idx : unsigned(7 downto 0) := (others => '0');
    signal q8_b, q9_b, q10_b : unsigned(4 downto 0) := (others => '0');
    signal p10_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal p13_y  : unsigned(13 downto 0) := (others => '0');
    signal p13_u, p13_v : signed(14 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- video accumulator
    signal a_col, a_fc : unsigned(6 downto 0) := (others => '0');
    signal a_sum, a_fs : unsigned(15 downto 0) := (others => '0');
    signal a_fl, a_f1, a_f2, a_f3, a_gq : std_logic := '0';

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
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
    -- Block RAMs (one write statement each; reads never use the cell being
    -- written) and ROMs
    ---------------------------------------------------------------------------
    p_dram : process(clk)
    begin
        if rising_edge(clk) then
            d_rq <= dram(to_integer(d_ra));
            if d_we = '1' then dram(to_integer(e_ci)) <= s_wd; end if;      -- the state word 0, as it is written
        end if;
    end process p_dram;

    p_vram : process(clk)
    begin
        if rising_edge(clk) then
            v_rq <= vram(to_integer(v_ra));
            if v_we = '1' then vram(to_integer(v_wa)) <= v_wd; end if;
        end if;
    end process p_vram;

    p_sram : process(clk)
    begin
        if rising_edge(clk) then
            s_rq <= sram(to_integer(s_ra));
            if s_we = '1' then sram(to_integer(s_wa)) <= s_wd; end if;
        end if;
    end process p_sram;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_tb_q <= C_SK_TAB(to_integer(r_tb_a));
            r_pf_q <= C_SK_PAL(to_integer(r_pf_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
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
                s_wave   <= registers_in(6)(0);
                s_centre <= registers_in(6)(1);
                s_trail  <= registers_in(6)(2);
                s_fuse   <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_xs     <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9); c_rinc <= to_unsigned(5825, 13);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9); c_rinc <= to_unsigned(3277, 13);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9); c_rinc <= to_unsigned(2185, 13);
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
                            when 3      => s_gm <= s_gk; s_gr <= unsigned(registers_in(5));
                            when 4      => s_gm <= s_g6; s_gr <= unsigned(registers_in(4));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        -- deadband +-2 knob codes (+-4 on the camera channel, 3)
                        if s_gd > 32 or s_gd < -32 or (z_gc /= 3 and (s_gd > 16 or s_gd < -16)) then
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
                            when 3      => s_gk <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products.  Zone slot k loads at zc = 32 + 24 k (decoded at 30 + 24 k, operands
            -- pre-muxed at 31 + 24 k), 16 iterations, stable from load + 17.  The column engine asks
            -- for its products with e_mreq / e_msel (slots 8 .. 11) once the zone slots are done.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 160 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when others => null;
                end case;
            end if;
            if e_mreq = '1' then
                z_sel <= "010" & e_msel; z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_g1(12 downto 3), 36);       z_fb <= to_unsigned(89, 16);                 -- columns
                    when 1  => z_fa <= resize(c_n, 36);                     z_fb <= resize(c_rinc, 16);                  -- column phase step
                    when 2  => z_fa <= resize(s_vm, 36);                    z_fb <= resize(s_vm, 16);                    -- speed^2
                    when 3  => z_fa <= resize(c_inc(19 downto 8), 36);      z_fb <= resize(s_gk(12 downto 3), 16);       -- column-sum scale x camera
                    when 4  => z_fa <= resize(s_gp(12 downto 3), 36);       z_fb <= resize(s_gp(12 downto 3), 16);       -- drive^2
                    when 5  => z_fa <= resize(s_g6(12 downto 3), 36);       z_fb <= to_unsigned(51, 16);                 -- colour rotation
                    when 8  => z_fa <= resize(unsigned(v_rq(23 downto 0)), 36); z_fb <= resize(c_vsc, 16);              -- column sum -> mean x camera
                    when 9  => z_fa <= resize(e_r, 36);                     z_fb <= resize(e_a, 16);                     -- noise x envelope
                    when 10 => z_fa <= resize(e_raw, 36);                   z_fb <= resize(c_g, 16);                     -- mixer x drive
                    when others => z_fa <= resize(e_vsum, 36);              z_fb <= c_rcp;                               -- field mean
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
                    when 20  => c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
                    when 50  => c_n <= to_unsigned(8, 7) + f_acc(16 downto 10);
                    when 74  => c_inc <= f_acc(19 downto 0);
                    when 98  =>                       -- speed^2 >> 8 = f_acc(17 downto 8)
                        c_prob <= resize(f_acc(17 downto 9), 10) + 12;
                        c_d1 <= resize(f_acc(17 downto 9), 12) + 16;
                        c_d2 <= resize(f_acc(17 downto 8), 12) + 24;
                        c_dw <= resize(f_acc(17 downto 8) & '0', 12) + 32;
                    when 122 => c_vsc <= f_acc(21 downto 8);                         -- c_inc(19:8) x k6 / 256
                    when 146 => z_t <= resize(f_acc(19 downto 10), 11) + f_acc(19 downto 11);
                    when 151 =>
                    when 152 =>
                        c_g <= z_t + 77;
                        if s_gp(12 downto 3) > 768 then
                            c_bias <= resize(s_gp(10 downto 3) & '0', 9);     -- (p - 768) * 2
                        else
                            c_bias <= (others => '0');
                        end if;
                    when 153 =>
                        z_o7 <= (s_g3(12 downto 3) & "000") - resize(s_g3(12 downto 3), 13);      -- 7 k3
                        z_p9 <= resize(s_g2(12 downto 3), 14) + shift_left(resize(s_g2(12 downto 3), 14), 3);
                    when 154 =>
                        c_ofs <= signed(resize(z_o7(12 downto 3), 11)) - 448;                     -- (7 (k3 - 512)) >> 3
                        c_pitch <= z_p9(13 downto 5) + 32;
                        if s_g2(12 downto 3) >= 896 then c_solid <= '1'; else c_solid <= '0'; end if;
                    when 155 =>
                        z_dk <= resize(s_ys, 13) + s_ys(11 downto 2);
                    when 156 =>
                        if s_centre = '1' then
                            c_dka <= z_dk + s_ys(11 downto 3); c_dkb <= z_dk + s_ys(11 downto 3);
                        else
                            c_dka <= resize(s_ys, 13); c_dkb <= s_ys & '0';
                        end if;
                    when 157 =>
                        z_g2a <= resize(c_dka & '0', 14) + c_dka(12 downto 2);
                        z_g2b <= resize(c_dkb & '0', 14) + c_dkb(12 downto 2);
                    when 158 =>
                        c_gma <= z_g2a(12 downto 8); c_gmb <= z_g2b(12 downto 8);
                    when 159 =>
                        -- segment gap = a quarter of the pitch, at least 2.25 raster rows; dropped where a
                        -- segment could not hold 2.25 lit rows as well (SD reflections at fine pitch)
                        if c_pitch(8 downto 2) > c_gma then z_gxa <= c_pitch(8 downto 2); else z_gxa <= resize(c_gma, 7); end if;
                        if c_pitch(8 downto 2) > c_gmb then z_gxb <= c_pitch(8 downto 2); else z_gxb <= resize(c_gmb, 7); end if;
                        if c_pitch >= (c_gma & '0') then z_ona <= '1'; else z_ona <= '0'; end if;
                        if c_pitch >= (c_gmb & '0') then z_onb <= '1'; else z_onb <= '0'; end if;
                        if c_gma > 12 then c_cta <= c_gma; else c_cta <= to_unsigned(12, 5); end if;
                        if c_gmb > 12 then c_ctb <= c_gmb; else c_ctb <= to_unsigned(12, 5); end if;
                    when 161 =>
                        if z_ona = '1' then c_pgapa <= c_pitch - z_gxa; else c_pgapa <= (others => '1'); end if;
                        if z_onb = '1' then c_pgapb <= c_pitch - z_gxb; else c_pgapb <= (others => '1'); end if;
                    when 180 => c_koff <= f_acc(15 downto 8) - 102;                                   -- ((k6 - 512) * 51) >> 8, mod 256
                    when 181 => c_hzi <= c_koff + C_HZ;
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
    -- Column engine (vblank zone, from zc 512): 64 clocks per column.
    --   0 .. 5   read the state words, the column's video sum and two sines; resample the noise
    --   3 .. 19  column sum x scale -> mean; 19 .. 33 noise x envelope; 37 .. 52 mixer x drive
    --   53 .. 57 envelope follower, heat, peak-hold with gravity
    --   58 .. 60 write the state / display words, clear the video sum, step the column phases
    -- The LFSR steps on every clock of the run.  Started by the zone only (a field start), never
    -- free-running.
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_x : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            e_mreq <= '0';
            s_we <= '0'; d_we <= '0'; e_vclr <= '0';

            if s_zone = '1' and e_run = '0' then
                case to_integer(s_zc) is
                    when 160 => r_tb_a <= "10" & c_n;
                    when 163 => c_rcp <= r_tb_q;
                    when 182 =>
                        if s_vneg = '1' then
                            e_ph1 <= e_ph1 - c_d1; e_ph2 <= e_ph2 + c_d2; e_phw <= e_phw + c_dw;
                        else
                            e_ph1 <= e_ph1 + c_d1; e_ph2 <= e_ph2 - c_d2; e_phw <= e_phw - c_dw;
                        end if;
                        c_fld <= c_fld + 1;
                    when 190 =>
                        e_xf <= '0' & c_rcp(15 downto 1);
                        if s_wave = '1' then e_a1 <= e_phw + c_rcp;
                        else e_a1 <= e_ph1 + c_rcp(15 downto 1); end if;
                        e_a2 <= e_ph2 + c_rcp;
                    when 191 => e_a2 <= e_a2 + c_rcp(15 downto 3);
                    when 512 =>
                        e_run <= '1'; e_cs <= (others => '0'); e_ci <= (others => '0');
                        e_vsum <= (others => '0'); e_first <= e_boot;
                    when others => null;
                end case;
            end if;

            -- field mean of the column means (AC coupling for the next field)
            if e_fin = '1' then
                e_pc <= e_pc + 1;
                if e_pc = 0 then e_mreq <= '1'; e_msel <= "11"; end if;
                if e_pc = 24 then
                    c_vmean <= f_acc(27 downto 16);
                    e_fin <= '0';
                end if;
            end if;

            -- Free-running terms: each is one operation on values that hold still for the steps
            -- around the one that reads it (the step numbers are in the table above the case).
            -- envelope: 260 + 2 s1 + 1.5 s2 - xf / 256, clipped to 0 .. 1023  (Wave: a = 128, b = 3 s1)
            e_e1 <= resize(e_s1 & '0', 10) + 260;
            e_t  <= resize(e_s2, 9) + e_s2(7 downto 1);
            e_e2 <= resize(e_e1, 11) + e_t;
            e_e3 <= signed(resize(e_e2, 12)) - signed(resize(e_xf(15 downto 8), 12));
            if s_wave = '1' then
                e_a <= to_unsigned(128, 10);
                e_b <= resize(e_s1 & '0', 10) + e_s1;
            else
                if e_e3 < 0 then e_a <= (others => '0');
                elsif e_e3 > 1023 then e_a <= (others => '1');
                else e_a <= unsigned(e_e3(9 downto 0)); end if;
                e_b <= (others => '0');
            end if;
            -- source: (column mean - field mean) x 4, clipped
            e_sr <= signed(resize(e_m, 14)) - signed(resize(c_vmean, 14));
            if e_sr > 511 then e_src <= to_signed(2047, 12);
            elsif e_sr < -511 then e_src <= to_signed(-2047, 12);
            else e_src <= shift_left(resize(e_sr, 12), 2); end if;
            -- mixer: shaped noise + source + level, clipped at 0, + bias
            e_rs1 <= signed(resize(e_tt, 14)) + resize(e_src, 14);
            e_rs  <= e_rs1 + resize(c_ofs, 14);
            if e_rs < 0 then e_rw <= (others => '0');
            elsif e_rs > 4095 then e_rw <= to_unsigned(4095, 13);
            else e_rw <= unsigned(e_rs(12 downto 0)); end if;
            e_raw <= e_rw + c_bias;
            -- targets: bar (Q4 lines, full scale 640), heat (overdrive), cap
            if e_d >= 1024 then
                e_tg <= to_unsigned(10240, 14);
                v_x := e_d - 1024;
                if v_x(15 downto 13) /= "000" then e_tq <= (others => '1'); else e_tq <= v_x(12 downto 9); end if;
            else
                e_tg <= shift_left(resize(e_d(9 downto 0), 14), 3) + shift_left(resize(e_d(9 downto 0), 14), 1);
                e_tq <= (others => '0');
            end if;
            e_ct <= e_tg + shift_left(resize(e_tq, 14), 6);
            e_dh <= signed(resize(e_tg, 15)) - signed(resize(e_h, 15));
            e_ha <= e_h + unsigned(resize(shift_right(e_dh + 1, 1), 14));           -- h + ((dh + 1) >> 1), dh > 0
            e_hr <= signed(resize(e_h, 15)) - signed(resize(c_rel, 15));
            if e_ct >= e_p then e_cge <= '1'; else e_cge <= '0'; end if;
            e_vn <= e_v + c_grav;
            e_pf <= signed(resize(e_p, 15)) - signed(resize(e_vn, 15));

            if e_run = '1' then
                e_cs <= e_cs + 1;
                if e_lf(0) = '1' then e_lf <= ('0' & e_lf(31 downto 1)) xor x"A3000000";
                else e_lf <= '0' & e_lf(31 downto 1); end if;

                --  0      addresses: state word 1, sine 1 (the video sum is read at v_ra = column all along)
                --  2      state word 1 -> r, v, hold; sine 1; addresses: state word 0, sine 2
                --  3      resample r from the LFSR; ask for sum x scale (ready at 19)
                --  4      state word 0 -> h, p, heat; sine 2
                --  19     column mean; ask for r x envelope (ready at 33)
                --  20     sum of the means
                --  33     shaped noise; the mixer terms settle by 38
                --  37     ask for mixer x drive (ready at 52)
                --  53     d; targets settle by 57
                --  56-58  heat, bar, cap
                --  59-61  write state / display, clear the video sum, step the column phases
                case to_integer(e_cs) is
                    when 0 =>
                        s_ra <= '1' & e_ci;
                        r_tb_a <= '0' & e_a1(15 downto 8);
                    when 2 =>
                        e_r  <= unsigned(s_rq(9 downto 0));
                        e_v  <= unsigned(s_rq(21 downto 10));
                        e_hd <= unsigned(s_rq(28 downto 22));
                        e_s1 <= r_tb_q(7 downto 0);
                        s_ra <= '0' & e_ci;
                        r_tb_a <= '0' & e_a2(15 downto 8);
                    when 3 =>
                        if e_lf(9 downto 0) < c_prob or e_first = '1' then e_r <= e_lf(19 downto 10); end if;
                        e_mreq <= '1'; e_msel <= "00";
                    when 4 =>
                        e_h  <= unsigned(s_rq(31 downto 18));
                        e_p  <= unsigned(s_rq(17 downto 4));
                        e_ht <= unsigned(s_rq(3 downto 0));
                        e_s2 <= r_tb_q(7 downto 0);
                    when 19 =>
                        if f_acc(35 downto 31) /= 0 then e_m <= (others => '1'); else e_m <= f_acc(30 downto 19); end if;
                        e_mreq <= '1'; e_msel <= "01";
                    when 20 => e_vsum <= e_vsum + e_m;
                    when 33 => e_tt <= f_acc(20 downto 10) + e_b;
                    when 37 => e_mreq <= '1'; e_msel <= "10";
                    when 53 => e_d <= f_acc(23 downto 8);
                    when 56 =>
                        if e_tq > e_ht then e_ht <= e_tq;
                        elsif e_ht /= 0 then e_ht <= e_ht - 1; end if;
                    when 57 =>
                        if e_dh > 0 then e_h <= e_ha;
                        elsif e_hr < signed(resize(e_tg, 15)) then e_h <= e_tg;
                        else e_h <= unsigned(e_hr(13 downto 0)); end if;
                    when 58 =>
                        if e_cge = '1' then
                            e_p <= e_ct; e_v <= (others => '0'); e_hd <= c_hold;
                        elsif e_hd /= 0 then
                            e_hd <= e_hd - 1;
                        elsif e_pf <= signed(resize(e_ct, 15)) then
                            e_p <= e_ct; e_v <= (others => '0');
                        else
                            e_p <= unsigned(e_pf(13 downto 0)); e_v <= e_vn;
                        end if;
                    when 59 =>
                        s_we <= '1'; s_wa <= '0' & e_ci;
                        s_wd <= std_logic_vector(e_h) & std_logic_vector(e_p) & std_logic_vector(e_ht);
                        d_we <= '1';
                        e_vclr <= '1';
                    when 60 =>
                        s_we <= '1'; s_wa <= '1' & e_ci;
                        s_wd <= "000" & std_logic_vector(e_hd) & std_logic_vector(e_v) & std_logic_vector(e_r);
                        e_xf <= e_xf + c_rcp;
                        if s_wave = '1' then e_a1 <= e_a1 + (c_rcp(14 downto 0) & '0');
                        else e_a1 <= e_a1 + c_rcp; end if;
                        e_a2 <= e_a2 + (c_rcp(14 downto 0) & '0');
                    when 61 => e_a2 <= e_a2 + c_rcp(15 downto 2);
                    when 63 =>
                        if e_ci + 1 = c_n then
                            e_run <= '0'; e_fin <= '1'; e_pc <= (others => '0'); e_boot <= '0';
                        else
                            e_ci <= e_ci + 1;
                        end if;
                    when others => null;
                end case;
            end if;
        end if;
    end process p_eng;

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
    -- Source side: oscillator (2-D triangle lattice, drifting) or the camera's
    -- (luma >> 2) - 16, into the column accumulators
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
                s_lum <= s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lum <= v_l - 16; else s_lum <= (others => '0'); end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Video accumulator: the luma summed along the row while the beam stays in a
    -- column; at each column change (and the line end) the row sum is added to
    -- the column's cell (read, fabric copy, add, write: 5 clocks, a column is at
    -- least 7 px wide).  Only on the 128 sample rows of the field.  The column
    -- engine reads and clears the cells in the zone.
    ---------------------------------------------------------------------------
    p_vacc : process(clk)
    begin
        if rising_edge(clk) then
            a_fl <= '0';
            a_gq <= s_gav_sr(2);
            if s_gav_sr(2) = '1' then
                if q2_col /= a_col then
                    a_fc <= a_col; a_fs <= a_sum; a_fl <= l_samp;
                    a_sum <= resize(s_lum, 16); a_col <= q2_col;
                else
                    a_sum <= a_sum + s_lum;
                end if;
            elsif a_gq = '1' then
                a_fc <= a_col; a_fs <= a_sum; a_fl <= l_samp;
                a_sum <= (others => '0');
            end if;
            if a_fl = '1' and a_fc < c_n then a_f1 <= '1'; else a_f1 <= '0'; end if;
            a_f2 <= a_f1; a_f3 <= a_f2;
            if e_run = '1' then v_ra <= e_ci; else v_ra <= a_fc; end if;
            v_rd <= unsigned(v_rq(23 downto 0));
            v_we <= '0';
            if e_vclr = '1' then
                v_we <= '1'; v_wa <= e_ci; v_wd <= (others => '0');
            elsif a_f3 = '1' then
                v_we <= '1'; v_wa <= a_fc; v_wd <= x"00" & std_logic_vector(v_rd + a_fs);
            end if;
        end if;
    end process p_vacc;

    ---------------------------------------------------------------------------
    -- Line engine: hy = the V ramp measured from the axis (the horizon, or the
    -- mid-line), Q8 virtual lines: it descends to the axis and climbs again below
    -- it (twice as fast under the horizon: the squeezed reflection).  hq / ph
    -- split it into LED segments.  Kicked in the zone for line 0 and at each line
    -- start for the NEXT line.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_hyi : unsigned(9 downto 0);
        variable v_n   : signed(18 downto 0);
        variable v_l   : signed(5 downto 0);
        variable v_gm, v_ct : unsigned(4 downto 0);
        variable v_pg  : unsigned(8 downto 0);
        variable v_c   : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if s_zone = '1' and s_zc = 400 then
                if s_centre = '1' then
                    y_hy <= to_unsigned(C_HY0C, 18); y_ph <= to_signed(C_HY0C, 19);
                else
                    y_hy <= to_unsigned(C_HY0R, 18); y_ph <= to_signed(C_HY0R, 19);
                end if;
                y_hq <= (others => '0');
                y_dn <= '1'; y_refl <= '0'; y_init <= '1'; y_sp <= '1';
                e_vy <= (others => '0');
            elsif y_init = '1' then
                if unsigned(y_ph(17 downto 8)) >= resize(c_pitch, 10) then
                    y_ph <= y_ph - signed(resize(c_pitch & x"00", 19));
                    y_hq <= y_hq + c_pitch;
                else
                    y_init <= '0'; y_oc <= "01";
                end if;
            else
                case to_integer(e_lc) is
                    when 1 =>
                        y_t <= signed(resize(y_hy, 19)) - signed(resize(c_dka, 19));
                    when 2 =>
                        if y_dn = '1' then
                            if y_t >= 0 then
                                y_hy <= unsigned(y_t(17 downto 0));
                                y_ph <= y_ph - signed(resize(c_dka, 19));
                            else
                                y_dn <= '0';
                                v_n := -y_t;
                                if s_centre = '0' then v_n := shift_left(v_n, 1); y_refl <= '1'; end if;
                                y_hy <= unsigned(v_n(17 downto 0)); y_ph <= v_n; y_hq <= (others => '0');
                            end if;
                        else
                            y_hy <= y_hy + c_dkb;
                            y_ph <= y_ph + signed(resize(c_dkb, 19));
                        end if;
                    when 3 =>
                        if y_dn = '1' then
                            if y_ph < 0 then
                                y_ph <= y_ph + signed(resize(c_pitch & x"00", 19));
                                y_hq <= y_hq - c_pitch;
                            end if;
                        elsif unsigned(y_ph(17 downto 8)) >= resize(c_pitch, 10) then
                            y_ph <= y_ph - signed(resize(c_pitch & x"00", 19));
                            y_hq <= y_hq + c_pitch;
                        end if;
                        y_oc <= "01";
                    when others => null;
                end case;
            end if;

            -- the per-line constants of the pixel path, three steps after hy settles
            v_hyi := y_hy(17 downto 8);
            if y_dn = '1' then v_gm := c_gma; v_ct := c_cta; v_pg := c_pgapa;
            else v_gm := c_gmb; v_ct := c_ctb; v_pg := c_pgapb; end if;
            v_c := unsigned(n_cmp(9 downto 0));
            if y_oc = "01" then
                -- the comparator level of this line: hy (solid; nothing under 2.25 raster rows), or the
                -- middle of its LED segment
                if c_solid = '0' then n_cmp <= signed(resize(y_hq, 12)) + signed(resize(c_pitch(8 downto 1), 12));
                elsif v_hyi < v_gm then n_cmp <= signed(resize(v_gm, 12));
                else n_cmp <= signed(resize(v_hyi, 12)); end if;
                if y_refl = '1' and v_hyi < 20 then n_hz <= '1'; else n_hz <= '0'; end if;
                if c_solid = '0' and unsigned(y_ph(17 downto 8)) >= resize(v_pg, 10) then n_gap <= '1'; else n_gap <= '0'; end if;
                y_lbr <= to_signed(11, 6) - signed(resize(v_hyi(9 downto 6), 6));
                n_refl <= y_refl;
                if e_vy(11) /= y_sp and e_vy(18) = '0' then n_samp <= '1'; else n_samp <= '0'; end if;
                y_sp <= e_vy(11);
                y_oc <= "10";
            elsif y_oc = "10" then
                -- cap window: one segment (LEDs), or from 8 lines under the peak (it sits on the drooping
                -- bar top) to the cap thickness above it (solid)
                if c_solid = '1' then
                    n_cmp2 <= n_cmp - signed(resize(v_ct, 12)); n_cmp3 <= n_cmp + 8;
                else
                    n_cmp2 <= n_cmp - signed(resize(c_pitch, 12)); n_cmp3 <= n_cmp;
                end if;
                -- the colouriser, the ceiling and the ripple see the same level (one colour per LED)
                y_ix <= resize(v_c(9 downto 3), 8) + v_c(9 downto 5);
                if v_c > 720 then n_top <= '1'; else n_top <= '0'; end if;
                if (v_c(6 downto 5) + c_fld(3 downto 2)) = "00" then y_rip <= '1'; else y_rip <= '0'; end if;
                y_oc <= "11";
            elsif y_oc = "11" then
                n_idx <= y_ix + v_c(9 downto 7);
                v_l := y_lbr;
                if y_rip = '1' then v_l := v_l - 2; end if;
                if n_refl = '0' then n_lb <= to_unsigned(16, 5);
                elsif v_l < 0 then n_lb <= (others => '0');
                else n_lb <= unsigned(v_l(4 downto 0)); end if;
                y_oc <= "00";
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the column phase (7.22: column, position in the column) and the
    -- virtual x (the H ramp that leaks into the colouriser)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            -- the pixel path is 13 stages deep; the beam starts 2 clocks after the generated avid so its
            -- output meets the shell's stage 15
            g_st1 <= g_start; g_st2 <= g_st1;
            if g_st2 = '1' then
                b_acc <= resize(c_inc & "000", 29);      -- 8 * inc: the output boxes lag 8 px
                b_x   <= c_xb0;
            elsif s_gav_sr(1) = '1' then
                b_acc <= b_acc + c_inc;
                b_x   <= b_x + s_xs;
            end if;
            if g_start = '1' then
                l_cmp <= n_cmp; l_cmp2 <= n_cmp2; l_cmp3 <= n_cmp3; l_idx <= n_idx; l_lb <= n_lb;
                l_gap <= n_gap; l_top <= n_top; l_refl <= n_refl; l_hz <= n_hz; l_samp <= n_samp;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel = the shell's stage 3)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_sh : unsigned(5 downto 0);
        variable v_tl : unsigned(4 downto 0);
        variable v_b  : signed(5 downto 0);
        variable v_yx : unsigned(9 downto 0);
        variable v_ux, v_vx : signed(9 downto 0);
        variable v_g1, v_g2 : unsigned(11 downto 0);
        variable v_g3, v_g4 : unsigned(13 downto 0);
        variable v_h1, v_h2 : signed(11 downto 0);
        variable v_h3, v_h4 : signed(13 downto 0);
        variable v_su, v_sv : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: column -> display RAM address; position in the column; the H ramp
            d_ra    <= b_acc(28 downto 22);
            q2_col  <= b_acc(28 downto 22);
            q2_fr   <= b_acc(21 downto 18);
            if b_acc(21 downto 20) = "11" and s_fuse = '0' then q2_gap <= '1'; else q2_gap <= '0'; end if;
            q2_tilt <= b_x(11 downto 8);
            -- 3: RAM read
            q3_fr <= q2_fr; q3_tilt <= q2_tilt;
            if q2_gap = '1' or q2_col >= c_n then q3_blank <= '1'; else q3_blank <= '0'; end if;
            -- 4: fabric copy: bar height, peak (virtual lines), heat
            q4_h  <= unsigned(d_rq(31 downto 22));
            q4_p  <= unsigned(d_rq(17 downto 8));
            q4_ht <= unsigned(d_rq(3 downto 0));
            q4_fr <= q3_fr; q4_tilt <= q3_tilt; q4_blank <= q3_blank;
            -- 5: S&H droop across the column; ladder index + H ramp; column shade
            q5_he  <= signed(resize(q4_h, 11)) - signed(resize(q4_fr(3 downto 1), 11));
            q5_p   <= q4_p;
            q5_idx <= l_idx + q4_tilt;
            q5_ht  <= q4_ht;
            v_sh   := resize(q4_fr & '0', 6) + q4_fr;
            q5_sh  <= v_sh(5 downto 3);
            q5_blank <= q4_blank;
            -- 6: the comparators against the V ramp
            if resize(q5_he, 12) >= l_cmp then q6_lit <= '1'; else q6_lit <= '0'; end if;
            if signed(resize(q5_p, 12)) >= l_cmp then q6_bp <= '1'; else q6_bp <= '0'; end if;
            if signed(resize(q5_p, 12)) >= l_cmp2 then q6_ac2 <= '1'; else q6_ac2 <= '0'; end if;
            if signed(resize(q5_p, 12)) >= l_cmp3 then q6_bp3 <= '1'; else q6_bp3 <= '0'; end if;
            q6_td  <= signed(resize(q5_p, 12)) - l_cmp;
            q6_idx <= resize(q5_idx, 9) + (q5_ht & "000");
            q6_sh  <= q5_sh; q6_blank <= q5_blank;
            -- 7: the candidate levels (1/16 units)
            if q6_td(11 downto 3) >= 6 then v_tl := to_unsigned(14, 5);
            else v_tl := to_unsigned(8, 5) + unsigned(q6_td(5 downto 3)); end if;
            if l_refl = '1' then
                v_b := signed(resize(l_lb, 6)) - signed(resize(q6_sh(2 downto 1), 6));
                if v_b < 0 then q7_blit <= (others => '0'); else q7_blit <= unsigned(v_b(4 downto 0)); end if;
                q7_tl  <= '0' & v_tl(4 downto 1);
                q7_bun <= (others => '0');
            else
                q7_blit <= to_unsigned(16, 5) - q6_sh;
                q7_tl  <= v_tl;
                q7_bun <= to_unsigned(2, 5);
            end if;
            q7_lit <= q6_lit; q7_bp <= q6_bp; q7_cap <= q6_ac2 and not q6_bp3;
            q7_blank <= q6_blank or l_gap or l_top;
            if q6_idx > C_HOT then q7_idx <= to_unsigned(C_HOT, 8); else q7_idx <= q6_idx(7 downto 0); end if;
            -- 8: the keyer: level, wheel variant, colouriser address
            r_pf_a <= '0' & (q7_idx + c_koff);
            if l_hz = '1' then
                q8_b <= to_unsigned(7, 5); r_pf_a <= '0' & c_hzi;
            elsif q7_blank = '1' then
                q8_b <= (others => '0');
            elsif q7_lit = '1' then
                q8_b <= q7_blit;
            elsif s_trail = '1' and q7_bp = '1' then
                q8_b <= q7_tl; r_pf_a(8) <= '1';
            elsif s_trail = '0' and q7_cap = '1' then
                q8_b <= l_lb; r_pf_a(8) <= '1';
            else
                q8_b <= q7_bun;
            end if;
            -- 9: ROM data; 10: fabric copy
            q9_b <= q8_b;
            p10_q <= r_pf_q; q10_b <= q9_b;
            -- 11 (shell stage 13): VCA: colour x level (level 16 = unity)
            v_yx := unsigned(p10_q(31 downto 22));
            v_ux := signed(p10_q(21 downto 12));
            v_vx := signed(p10_q(11 downto 2));
            v_g1 := (others => '0'); v_g2 := (others => '0'); v_g3 := (others => '0'); v_g4 := (others => '0');
            if q10_b(0) = '1' then v_g1 := resize(v_yx, 12); end if;
            if q10_b(1) = '1' then v_g2 := resize(v_yx & '0', 12); end if;
            if q10_b(2) = '1' then v_g3 := resize(v_yx & "00", 14); end if;
            if q10_b(3) = '1' then v_g4 := resize(v_yx & "000", 14); end if;
            if q10_b(4) = '1' then p13_y <= v_yx & "0000";
            else p13_y <= resize(v_g1 + v_g2, 14) + (v_g3 + v_g4); end if;
            v_h1 := (others => '0'); v_h2 := (others => '0'); v_h3 := (others => '0'); v_h4 := (others => '0');
            if q10_b(0) = '1' then v_h1 := resize(v_ux, 12); end if;
            if q10_b(1) = '1' then v_h2 := shift_left(resize(v_ux, 12), 1); end if;
            if q10_b(2) = '1' then v_h3 := shift_left(resize(v_ux, 14), 2); end if;
            if q10_b(3) = '1' then v_h4 := shift_left(resize(v_ux, 14), 3); end if;
            if q10_b(4) = '1' then p13_u <= shift_left(resize(v_ux, 15), 4);
            else p13_u <= resize(v_h1 + v_h2, 15) + resize(v_h3 + v_h4, 15); end if;
            v_h1 := (others => '0'); v_h2 := (others => '0'); v_h3 := (others => '0'); v_h4 := (others => '0');
            if q10_b(0) = '1' then v_h1 := resize(v_vx, 12); end if;
            if q10_b(1) = '1' then v_h2 := shift_left(resize(v_vx, 12), 1); end if;
            if q10_b(2) = '1' then v_h3 := shift_left(resize(v_vx, 14), 2); end if;
            if q10_b(3) = '1' then v_h4 := shift_left(resize(v_vx, 14), 3); end if;
            if q10_b(4) = '1' then p13_v <= shift_left(resize(v_vx, 15), 4);
            else p13_v <= resize(v_h1 + v_h2, 15) + resize(v_h3 + v_h4, 15); end if;
            -- 12 (shell stage 14): back to codes
            s_y14 <= to_unsigned(64, 10) + p13_y(13 downto 4);
            v_su := p13_u(14 downto 4); v_sv := p13_v(14 downto 4);
            v_su := v_su + 512; v_sv := v_sv + 512;
            s_u14 <= unsigned(v_su(9 downto 0)); s_v14 <= unsigned(v_sv(9 downto 0));
            -- 15: compose; black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= C_KY; s_u15 <= C_KC; s_v15 <= C_KC;
            else
                s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the rack's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to black (the off-avid input of both boxes) once
            -- 8 blank samples are in, so a glitched sample can't offset the colours for good.
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

end architecture skyline;
