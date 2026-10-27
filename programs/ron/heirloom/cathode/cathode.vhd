-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Cathode v0.5.0 -- Heirloom #23 (2026-10-27 rebuild), from WIN_20230407_23_48_05.
--
-- The photo: a sample-and-hold colour mosaic (42.2 px x 13.9-line cells) through a CYCLIC
-- four-colour colourizer -- green -> pink -> yellow -> mint -> green, a 2-bit Gray code --
-- whose phase ramps down every column at the column's own rate (0, +-1/32, +-1/8, +-1/4,
-- +-1/4+-1/32 of a cycle per row: a vertical ramp oscillator times a horizontal S&H
-- waveform), a blue fringe on the leading edge of every green cell, and two purple
-- "flames" keyed over it: each a group of three shaded tubes (a teardrop, a body
-- with a magenta left edge, a thin left arm) tapering down to a tip, the pair being
-- two periods (947 px) of one horizontal pattern.  Inside the flames' envelope the
-- mosaic loses its green (magenta / black blocks in the gaps between the tubes) and
-- a dark-green halo fades it back over ~80 px.
--
-- The program draws exactly that.  Osc = the photo's per-column ramp table (64 x 16 ROM)
-- plus hashed R-bit noise with the photo's class statistics.  Video = the camera is the
-- mosaic's R bit (cell-sampled luma above its running mean; the ramp keeps the other
-- bit) and the purple of the tubes (a live duotone picture inside the flames).
--
--   K1  Cells   cell width 160 .. 12 px (42 = the photo)
--   K2  Rows    row height 64 .. 4 lines (13.9 = the photo)
--   K3  Cycle   column-ramp gain 0 .. 3x (1x = the photo)
--   K4  Speed   bipolar (still in the centre): the ramps roll, cells re-roll, flames sway
--   K5  Flames  flames across, 1 .. 4 (2.03 = the photo)
--   K6  Patch   RGB cable patch: RGB / RBG / GRB / GBR / BRG / BGR
--   S7  Fringe  Blue / Off
--   S8  Glow    On (dark-green halo) / Off (hard key)
--   S9  Core    On / Off (the teardrop)
--   S10 Flip    Down / Up
--   S11 Source  Video / Osc
--   P12 FLAMES  length x0.35 .. x1 (photo, 50 %) .. x2.2, width x0.75 .. x1.25
--
-- Virtual 1080-line coordinates (timing-ID steps); generated avid; output conditioning
-- ([1 2 1] luma, [1 2 2 2 2 2 2 2 1]/16 chroma, luma 13/16); blanking gate.  Latency 24
-- clocks in every mode.  Colours: u = Cr, v = Cb (hardware convention).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture cathode of program_top is
    constant C_LAT : integer := 24;   -- matches processing_delay_clks

    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
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

    -- column table (photo-fitted, colfit3.py): ph6 & rate6 (signed, 1/32 cycle per row) & namp4
    type t_col is array (0 to 63) of std_logic_vector(15 downto 0);
    constant C_COL : t_col := (
        x"C00A", x"C00A", x"8000", x"8000", x"0810", x"8387", x"0000", x"6040",
        x"6040", x"6040", x"6040", x"6040", x"6040", x"6040", x"6040", x"6040",
        x"6040", x"6040", x"6040", x"0810", x"4B9F", x"089A", x"707C", x"4B9F",
        x"F37B", x"C084", x"0817", x"089D", x"838B", x"8819", x"6040", x"6040",
        x"6040", x"6040", x"6040", x"6040", x"6040", x"6040", x"6040", x"6040",
        x"6040", x"6040", x"838F", x"881C", x"C08A", x"6BF8", x"C00A", x"6040",
        x"6040", x"707C", x"8819", x"6040", x"881C", x"8387", x"6040", x"0810",
        x"0817", x"6040", x"6040", x"8000", x"6040", x"6040", x"4B9F", x"6040");
    -- flame tables (tubefit2.py), 16-line steps: tw 0..79, bc 80.., bw 160.., -ac 240.., aw 320..
    type t_ft is array (0 to 511) of std_logic_vector(7 downto 0);
    constant C_FT : t_ft := (
        x"52", x"52", x"53", x"53", x"53", x"54", x"54", x"53", x"52", x"51", x"50", x"4F", x"4E", x"4D", x"4D", x"4C",
        x"4C", x"4B", x"4B", x"4B", x"4A", x"49", x"47", x"46", x"44", x"42", x"3F", x"3D", x"3A", x"38", x"35", x"32",
        x"2F", x"2B", x"26", x"22", x"1E", x"1A", x"16", x"12", x"0D", x"06", x"00", x"00", x"00", x"00", x"00", x"00",
        x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00",
        x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00",
        x"A8", x"A8", x"A7", x"A7", x"A7", x"A6", x"A6", x"A6", x"A5", x"A5", x"A4", x"A3", x"A3", x"A2", x"A1", x"A0",
        x"9F", x"9E", x"9D", x"9B", x"9A", x"99", x"98", x"97", x"95", x"94", x"93", x"92", x"90", x"8F", x"8E", x"8C",
        x"8A", x"86", x"81", x"7D", x"79", x"77", x"75", x"73", x"6F", x"6B", x"67", x"63", x"60", x"5D", x"5A", x"57",
        x"52", x"4D", x"47", x"43", x"3F", x"3B", x"37", x"32", x"2C", x"27", x"21", x"1C", x"18", x"14", x"10", x"0C",
        x"09", x"06", x"04", x"02", x"01", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00",
        x"39", x"39", x"3A", x"3A", x"3A", x"3B", x"3B", x"3B", x"3C", x"3C", x"3C", x"3C", x"3C", x"3C", x"3C", x"3C",
        x"3C", x"3C", x"3C", x"3C", x"3C", x"3C", x"3C", x"3C", x"3C", x"3C", x"3B", x"3B", x"3B", x"3A", x"3A", x"39",
        x"39", x"3B", x"3D", x"3F", x"40", x"3F", x"3F", x"3E", x"3F", x"41", x"44", x"46", x"46", x"45", x"44", x"44",
        x"45", x"45", x"46", x"46", x"45", x"44", x"43", x"42", x"41", x"40", x"3F", x"3E", x"3C", x"3B", x"39", x"33",
        x"2E", x"28", x"20", x"14", x"09", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00",
        x"B8", x"B8", x"B7", x"B7", x"B7", x"B6", x"B6", x"B6", x"B5", x"B5", x"B4", x"B4", x"B3", x"B2", x"AF", x"AC",
        x"A8", x"A5", x"A3", x"A1", x"9F", x"9D", x"9C", x"9A", x"99", x"96", x"94", x"91", x"8E", x"8D", x"8C", x"8B",
        x"89", x"85", x"81", x"7D", x"78", x"74", x"70", x"6C", x"68", x"65", x"62", x"5F", x"5B", x"56", x"50", x"4C",
        x"4A", x"47", x"45", x"43", x"41", x"3F", x"3D", x"3B", x"3A", x"39", x"38", x"38", x"38", x"38", x"38", x"38",
        x"38", x"38", x"38", x"38", x"38", x"38", x"38", x"38", x"38", x"38", x"38", x"38", x"38", x"38", x"38", x"38",
        x"14", x"14", x"14", x"14", x"14", x"15", x"15", x"15", x"15", x"15", x"15", x"15", x"16", x"16", x"17", x"18",
        x"18", x"19", x"1A", x"1C", x"1D", x"1E", x"1F", x"21", x"22", x"23", x"25", x"28", x"2A", x"2A", x"2B", x"2C",
        x"2C", x"2C", x"2C", x"2C", x"2B", x"2A", x"28", x"27", x"25", x"24", x"23", x"22", x"21", x"20", x"1F", x"1E",
        x"1D", x"1D", x"1C", x"1A", x"16", x"12", x"0A", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00",
        x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00",
        x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00",
        x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00",
        x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00",
        x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00",
        x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00",
        x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00",
        x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00");
    type t_sin is array (0 to 63) of integer range -128 to 127;
    constant C_SIN : t_sin := (
        0, 12, 25, 37, 49, 60, 71, 81, 90, 98, 106, 112, 117, 122, 125, 126,
        127, 126, 125, 122, 117, 112, 106, 98, 90, 81, 71, 60, 49, 37, 25, 12,
        0, -12, -25, -37, -49, -60, -71, -81, -90, -98, -106, -112, -117, -122, -125, -126,
        -127, -126, -125, -122, -117, -112, -106, -98, -90, -81, -71, -60, -49, -37, -25, -12);

    -- cell RAM: the camera luma held per cell (written on a row's first line)
    type t_cram is array (0 to 63) of std_logic_vector(7 downto 0);
    signal cram : t_cram := (others => x"80");
    attribute no_rw_check : boolean;
    attribute no_rw_check of cram : signal is true;
    signal s_cwe : std_logic := '0';
    signal s_cwa : unsigned(5 downto 0) := (others => '0');
    signal s_cwd : std_logic_vector(7 downto 0) := x"80";
    signal s_cra : unsigned(5 downto 0) := (others => '0');
    signal s_crq : std_logic_vector(7 downto 0) := x"80";

    -- timing / zone / width
    signal s_tid : std_logic_vector(3 downto 0) := (others => '1');
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- glided knobs (10.3), switches
    signal g1, g2, g12 : unsigned(12 downto 0) := (others => '0');
    signal k1v, k2v, k3v, k5v, p12v : unsigned(9 downto 0) := (others => '0');
    signal s_sp : signed(10 downto 0) := (others => '0');
    signal s_asp : unsigned(9 downto 0) := (others => '0');
    signal s_patch : unsigned(2 downto 0) := (others => '0');
    signal s_nofr, s_hard, s_nocore, s_flip, s_osc : std_logic := '0';
    signal s_warm : std_logic := '0';

    -- per-field state
    signal s_roll : unsigned(19 downto 0) := (others => '0');                -- Q8 rows
    signal s_sacc : unsigned(12 downto 0) := (others => '0');
    signal s_seed : unsigned(7 downto 0) := (others => '0');
    signal s_th : unsigned(15 downto 0) := (others => '0');
    signal s_inc : unsigned(12 downto 0) := to_unsigned(1552, 13);           -- Q16 per virtual px
    signal s_incr : unsigned(13 downto 0) := to_unsigned(1552, 14);          -- per real px
    signal s_cp0 : unsigned(15 downto 0) := (others => '0');
    signal s_rinc : unsigned(14 downto 0) := to_unsigned(4715, 15);          -- Q16 per virtual line
    signal s_rincr : unsigned(16 downto 0) := to_unsigned(4715, 17);         -- per real line
    signal s_racc0 : unsigned(15 downto 0) := (others => '0');
    signal s_cyc8 : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_sy8 : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_isy : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_sx8 : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal s_wv : unsigned(11 downto 0) := to_unsigned(1920, 12);            -- virtual width
    signal s_pf : unsigned(10 downto 0) := to_unsigned(946, 11);             -- flame period (virtual px)
    signal s_pf2, s_pf2m : unsigned(11 downto 0) := to_unsigned(1892, 12);
    signal s_fxs : unsigned(10 downto 0) := (others => '0');
    signal s_fxt : signed(12 downto 0) := (others => '0');
    signal s_sway : signed(6 downto 0) := (others => '0');
    -- field sequencer (one serial multiplier, one serial divider)
    signal f_st : unsigned(5 downto 0) := (others => '0');
    signal f_ma : unsigned(24 downto 0) := (others => '0');
    signal f_mb : unsigned(11 downto 0) := (others => '0');
    signal f_acc : unsigned(24 downto 0) := (others => '0');
    signal f_mc : unsigned(4 downto 0) := (others => '0');
    signal f_dn : unsigned(20 downto 0) := (others => '0');                  -- dividend (shifts out)
    signal f_dd : unsigned(11 downto 0) := (others => '0');                  -- divisor
    signal f_rem : unsigned(12 downto 0) := (others => '0');
    signal f_dc : unsigned(4 downto 0) := (others => '0');
    signal s_linit : std_logic := '0';

    -- per-line engine (computes the NEXT line, kicked at the source avid rise)
    signal e_st : unsigned(5 downto 0) := (others => '0');
    signal e_ma : unsigned(21 downto 0) := (others => '0');
    signal e_mb : unsigned(11 downto 0) := (others => '0');
    signal e_acc : unsigned(21 downto 0) := (others => '0');
    signal e_mc : unsigned(3 downto 0) := (others => '0');
    signal e_vy8 : unsigned(19 downto 0) := (others => '0');                 -- virtual y, Q8
    signal e_racc : unsigned(15 downto 0) := (others => '0');
    signal e_j : unsigned(6 downto 0) := (others => '0');
    signal e_vy, e_yv : unsigned(10 downto 0) := (others => '0');
    signal e_ti : unsigned(6 downto 0) := (others => '0');
    signal e_fr : unsigned(3 downto 0) := (others => '0');
    signal e_p : unsigned(2 downto 0) := (others => '0');
    signal e_base : unsigned(8 downto 0) := (others => '0');
    signal e_v0, e_v1 : unsigned(7 downto 0) := (others => '0');
    signal e_d : signed(8 downto 0) := (others => '0');
    signal e_t1, e_t2 : signed(12 downto 0) := (others => '0');
    signal e_dfr : signed(12 downto 0) := (others => '0');
    signal e_v : unsigned(7 downto 0) := (others => '0');
    signal e_vyq : unsigned(11 downto 0) := (others => '0');
    signal e_fx : signed(12 downto 0) := (others => '0');
    signal e_aL, e_tL, e_bL, e_aR, e_tR, e_bR, e_m1, e_m2 : signed(11 downto 0) := (others => '0');
    signal s_fta : unsigned(8 downto 0) := (others => '0');
    signal s_ftq : std_logic_vector(7 downto 0) := (others => '0');
    -- next-line values
    signal n_tick : std_logic := '1';
    signal n_j : unsigned(6 downto 0) := (others => '0');
    signal n_jk : unsigned(12 downto 0) := (others => '0');
    signal n_tw, n_bc, n_bw, n_an, n_aw : unsigned(7 downto 0) := (others => '0');
    signal n_l, n_r : signed(11 downto 0) := (others => '0');
    signal n_alive, n_low : std_logic := '0';
    signal n_fxs : unsigned(10 downto 0) := (others => '0');
    -- committed line values
    signal l_tick : std_logic := '1';
    signal l_j : unsigned(6 downto 0) := (others => '0');
    signal l_jk : unsigned(12 downto 0) := (others => '0');
    signal l_tw, l_bc, l_bw, l_an, l_aw : unsigned(7 downto 0) := (others => '0');
    signal l_l, l_r : signed(11 downto 0) := (others => '0');
    signal l_alive, l_low : std_logic := '0';
    signal l_fxs : unsigned(10 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- beam (stage 0)
    signal b_cacc : unsigned(15 downto 0) := (others => '0');
    signal b_ci : unsigned(5 downto 0) := (others => '0');
    signal b_cs : std_logic := '0';
    signal b_pxv : unsigned(4 downto 0) := (others => '0');
    signal b_fx2 : unsigned(11 downto 0) := (others => '0');

    -- video luma
    signal v_y1 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal v_d2 : unsigned(9 downto 0) := (others => '0');
    signal v_l3, v_l4, v_l5 : unsigned(7 downto 0) := (others => '0');
    signal s_hold : unsigned(7 downto 0) := x"80";
    signal s_avg : unsigned(15 downto 0) := x"8000";
    signal s_adf : signed(16 downto 0) := (others => '0');
    signal s_adv : std_logic := '0';

    -- pixel pipeline
    type t_u6 is array (natural range <>) of unsigned(5 downto 0);
    type t_u5 is array (natural range <>) of unsigned(4 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal p_ci : t_u6(1 to 3) := (others => (others => '0'));
    signal p_cs : t_sl(1 to 7) := (others => '0');
    signal p_pxv : t_u5(1 to 7) := (others => (others => '0'));
    signal p_dx1 : signed(11 downto 0) := (others => '0');
    signal h2, h3, h4, h5, h6 : unsigned(15 downto 0) := (others => '0');
    signal p_colq2, p_col3 : std_logic_vector(15 downto 0) := (others => '0');
    signal p_ram3 : std_logic_vector(7 downto 0) := x"80";
    signal p_pa4, p_pb4 : unsigned(12 downto 0) := (others => '0');
    signal p_ph4, p_ph5 : unsigned(5 downto 0) := (others => '0');
    signal p_neg4, p_neg5 : std_logic := '0';
    signal p_nl4, p_nl5 : unsigned(7 downto 0) := (others => '0');
    signal p_p5 : unsigned(12 downto 0) := (others => '0');
    signal p_t1, p_t2, p_t3, p_t4 : unsigned(7 downto 0) := (others => '0');
    signal p_c46 : unsigned(1 downto 0) := (others => '0');
    signal p_lc4, p_lc5 : unsigned(7 downto 0) := (others => '0');
    signal p_ld6 : signed(8 downto 0) := (others => '0');
    signal p_idx7 : unsigned(2 downto 0) := (others => '0');
    signal s_prev, s_cur : unsigned(2 downto 0) := (others => '0');
    signal p_idx8 : unsigned(2 downto 0) := (others => '0');
    signal p_fr8 : std_logic := '0';
    signal p_rgb9, p_sr1, p_m10 : t_u8(0 to 2) := (others => (others => '0'));
    -- flame path
    signal p_uT2, p_uB2, p_uA2, p_dl2, p_dr2 : signed(12 downto 0) := (others => '0');
    signal p_dT3, p_dB3, p_dA3 : unsigned(11 downto 0) := (others => '0');
    signal p_lft3, p_lft4, p_lft5 : std_logic := '0';
    signal p_do3 : signed(12 downto 0) := (others => '0');
    signal p_eT4, p_eB4, p_eA4 : signed(12 downto 0) := (others => '0');
    signal p_k3a4 : unsigned(13 downto 0) := (others => '0');
    signal p_kin4 : std_logic := '0';
    signal p_inT5, p_inB5, p_inA5 : std_logic := '0';
    signal p_ebT5, p_ebB5, p_ebA5 : unsigned(5 downto 0) := (others => '0');
    signal p_e6T5, p_e4B5, p_e1A5 : unsigned(6 downto 0) := (others => '0');
    signal p_n22B5 : std_logic := '0';
    signal p_k5 : unsigned(5 downto 0) := (others => '0');
    signal p_rT6, p_rB6, p_rA6 : unsigned(7 downto 0) := (others => '0');
    signal p_inT6, p_inB6, p_inA6 : std_logic := '0';
    signal p_ebT6, p_ebB6, p_ebA6 : unsigned(5 downto 0) := (others => '0');
    signal p_r7 : unsigned(7 downto 0) := (others => '0');
    signal p_eb7 : unsigned(5 downto 0) := (others => '0');
    signal p_any : t_sl(7 to 12) := (others => '0');
    signal p_k : t_u6(6 to 11) := (others => (others => '0'));
    signal p_bt : t_u8(8 to 12) := (others => (others => '0'));
    signal p_rt : t_u8(8 to 12) := (others => (others => '0'));
    -- composite
    signal p_mr11, p_mg11, p_mb11 : unsigned(7 downto 0) := (others => '0');
    signal p_db11 : signed(8 downto 0) := (others => '0');
    signal p_kn11 : unsigned(5 downto 0) := (others => '0');
    signal p_gp12 : unsigned(13 downto 0) := (others => '0');
    signal p_bp12 : signed(15 downto 0) := (others => '0');
    signal p_mr12, p_mb12 : unsigned(7 downto 0) := (others => '0');
    signal p_g13, p_b13, p_mr13 : unsigned(7 downto 0) := (others => '0');
    signal p_rt13, p_bt13 : unsigned(7 downto 0) := (others => '0');
    signal p_any13, p_dk12 : std_logic := '0';
    signal p_o14 : t_u8(0 to 2) := (others => (others => '0'));
    signal p_yr15, p_yg15, p_yb15 : unsigned(15 downto 0) := (others => '0');
    signal p_ur15, p_ug15, p_ub15, p_vr15, p_vg15, p_vb15 : signed(16 downto 0) := (others => '0');
    signal p_ys16 : unsigned(16 downto 0) := (others => '0');
    signal p_us16, p_vs16 : signed(16 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y18, s_y19, s_y20, s_y21, s_y22, s_y23x, s_yf23 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_cram : process(clk)
    begin
        if rising_edge(clk) then
            s_crq <= cram(to_integer(s_cra));
            if s_cwe = '1' then cram(to_integer(s_cwa)) <= s_cwd; end if;
        end if;
    end process p_cram;

    p_from : process(clk)
    begin
        if rising_edge(clk) then
            s_ftq <= C_FT(to_integer(s_fta));
        end if;
    end process p_from;

    p_crom : process(clk)
    begin
        if rising_edge(clk) then
            p_colq2 <= C_COL(to_integer(p_ci(1)));
        end if;
    end process p_crom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s : signed(10 downto 0);
        variable v_t : unsigned(12 downto 0);
        variable v_r : unsigned(13 downto 0);
        variable v_sp : signed(19 downto 0);
        variable v_a : unsigned(12 downto 0);
        variable v_sy : unsigned(10 downto 0);
        variable v_x : signed(12 downto 0);
        variable v_c16 : unsigned(15 downto 0);
        procedure glide(signal g : inout unsigned(12 downto 0); k : std_logic_vector(9 downto 0); w : std_logic) is
            variable d : signed(14 downto 0);
            variable n : signed(14 downto 0);
        begin
            d := signed(resize(unsigned(k) & "000", 15)) - signed(resize(g, 15));
            if w = '0' then
                g <= unsigned(k) & "000";
            elsif d > 31 or d < -31 then
                n := signed(resize(g, 15)) + shift_right(d, 3);
                g <= unsigned(n(12 downto 0));
            end if;
        end procedure;
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_xs <= f_vstep_x(s_tid);
            s_ys <= f_vstep_y(s_tid);

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            k1v <= g1(12 downto 3); k2v <= g2(12 downto 3); p12v <= g12(12 downto 3);
            g_wm1 <= s_w - 1;
            s_linit <= '0';

            if s_zgo = '1' then
                s_warm <= '1';
                glide(g1, registers_in(0), s_warm);
                glide(g2, registers_in(1), s_warm);
                k3v <= unsigned(registers_in(2));
                k5v <= unsigned(registers_in(4));
                glide(g12, registers_in(7), s_warm);
                v_s := signed(resize(unsigned(registers_in(3)), 11)) - 512;          -- K4 Speed
                if v_s > -8 and v_s < 8 then v_s := (others => '0'); end if;
                s_sp <= v_s;
                if v_s < 0 then s_asp <= unsigned(resize(-v_s, 10)); else s_asp <= unsigned(v_s(9 downto 0)); end if;
                -- K6 patch: (k6 * 3) >> 9 -> 0..5
                v_t := resize(unsigned(registers_in(5)), 13) + resize(unsigned(registers_in(5)) & '0', 13);
                if unsigned(registers_in(5)) < 6 then            -- a labelled default arrives as its index
                    s_patch <= unsigned(registers_in(5)(2 downto 0));
                else
                    s_patch <= v_t(11 downto 9);
                end if;
                s_nofr   <= registers_in(6)(0);
                s_hard   <= registers_in(6)(1);
                s_nocore <= registers_in(6)(2);
                s_flip   <= registers_in(6)(3);
                s_osc    <= registers_in(6)(4);
                if s_wmax /= 0 then s_w <= s_wmax; end if;
                s_wmax <= (others => '0');
                -- motion: the ramps roll (sp/8 + sp/32 rows/256 per field), cells re-seed every
                -- 8192/|sp| fields, flames sway
                v_sp := resize(s_sp, 20);
                s_roll <= unsigned(signed(s_roll) + shift_right(v_sp, 3) + shift_right(v_sp, 5));
                v_r := resize(s_sacc, 14) + resize(s_asp, 14);
                if v_r(13) = '1' then s_seed <= s_seed + 1; end if;
                s_sacc <= v_r(12 downto 0);
                s_th  <= s_th + resize(s_asp & '0', 16);
                f_st <= to_unsigned(1, 6);
            end if;

            -- per-field sequencer: one op per state; the serial units step while their
            -- counters are non-zero and the state machine waits.  Products < 2^25.
            if f_mc /= 0 then
                if f_mb(0) = '1' then f_acc <= f_acc + f_ma; end if;
                f_ma <= f_ma(23 downto 0) & '0';
                f_mb <= '0' & f_mb(11 downto 1);
                f_mc <= f_mc - 1;
            elsif f_dc /= 0 then
                -- restoring division; the quotient shifts in behind the dividend
                v_a := f_rem(11 downto 0) & f_dn(20);
                if v_a >= resize(f_dd, 13) then
                    f_rem <= v_a - resize(f_dd, 13);
                    f_dn <= f_dn(19 downto 0) & '1';
                else
                    f_rem <= v_a;
                    f_dn <= f_dn(19 downto 0) & '0';
                end if;
                f_dc <= f_dc - 1;
            else
                case to_integer(f_st) is
                    when 1 =>       -- inc = 410 + k1 (k1 * 5051 >> 10) >> 10
                        f_ma <= to_unsigned(5051, 25); f_mb <= resize(k1v, 12); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 5);
                        f_st <= f_st + 1;
                    when 2 =>
                        f_ma <= resize(f_acc(22 downto 10), 25); f_mb <= resize(k1v, 12); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 5);
                        f_st <= f_st + 1;
                    when 3 =>
                        s_inc <= to_unsigned(410, 13) + resize(f_acc(22 downto 10), 13);
                        f_st <= f_st + 1;
                    when 4 =>       -- rinc = 1024 + k2 (k2 * 15360 >> 10) >> 10
                        f_ma <= to_unsigned(15360, 25); f_mb <= resize(k2v, 12); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 5);
                        f_st <= f_st + 1;
                    when 5 =>
                        f_ma <= resize(f_acc(23 downto 10), 25); f_mb <= resize(k2v, 12); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 5);
                        f_st <= f_st + 1;
                    when 6 =>
                        s_rinc <= to_unsigned(1024, 15) + resize(f_acc(23 downto 10), 15);
                        f_st <= f_st + 1;
                    when 7 =>       -- rinc * ys / 256
                        f_ma <= resize(s_rinc, 25); f_mb <= s_ys; f_acc <= (others => '0'); f_mc <= to_unsigned(12, 5);
                        f_st <= f_st + 1;
                    when 8 =>
                        s_rincr <= f_acc(24 downto 8);
                        f_st <= f_st + 1;
                    when 9 =>       -- sy8 from P12
                        if p12v(9) = '0' then
                            f_ma <= to_unsigned(333, 25); f_mb <= resize(p12v, 12);
                        else
                            f_ma <= to_unsigned(614, 25); f_mb <= resize(p12v(8 downto 0), 12);
                        end if;
                        f_acc <= (others => '0'); f_mc <= to_unsigned(10, 5);
                        f_st <= f_st + 1;
                    when 10 =>
                        if p12v(9) = '0' then
                            v_sy := to_unsigned(90, 11) + resize(f_acc(18 downto 10), 11);
                        else
                            v_sy := to_unsigned(256, 11) + resize(f_acc(19 downto 10), 11);
                        end if;
                        s_sy8 <= v_sy(9 downto 0);
                        -- isy = 65536 / sy8 (dividend 2^16 left-aligned in 21 bits)
                        f_dn <= shift_left(to_unsigned(65536, 21), 4); f_dd <= resize(v_sy, 12);
                        f_rem <= (others => '0'); f_dc <= to_unsigned(17, 5);
                        f_st <= f_st + 1;
                    when 11 =>
                        s_isy <= f_dn(9 downto 0);
                        s_sx8 <= to_unsigned(192, 9) + resize(p12v(9 downto 3), 9);
                        -- virtual width
                        case to_integer(s_xs) is
                            when 4      => s_wv <= s_w(10 downto 0) & '0';
                            when 3      => s_wv <= s_w + ('0' & s_w(11 downto 1));
                            when others => s_wv <= s_w;
                        end case;
                        f_st <= f_st + 1;
                    when 12 =>      -- Pf = Wv * 1024 / (1024 + 3 k5)
                        f_dn <= resize(s_wv, 11) & "0000000000";
                        f_dd <= to_unsigned(1024, 12) + resize(k5v, 12) + resize(k5v & '0', 12);
                        f_rem <= (others => '0'); f_dc <= to_unsigned(21, 5);
                        f_st <= f_st + 1;
                    when 13 =>
                        s_pf <= f_dn(10 downto 0);
                        f_st <= f_st + 1;
                    when 14 =>
                        s_pf2 <= s_pf & '0';
                        s_fxt <= signed(resize(s_pf, 13)) - signed(resize(s_wv(11 downto 1), 13)) - 48;
                        f_st <= f_st + 1;
                    when 15 | 16 | 17 =>
                        s_pf2m <= s_pf2 - resize(s_xs, 12);
                        if s_fxt < 0 then s_fxt <= s_fxt + signed(resize(s_pf, 13)); end if;
                        f_st <= f_st + 1;
                    when 18 =>
                        if s_fxt >= signed(resize(s_pf, 13)) then
                            v_x := s_fxt - signed(resize(s_pf, 13));
                        else
                            v_x := s_fxt;
                        end if;
                        s_fxs <= unsigned(v_x(10 downto 0));
                        -- sway amplitude (px at vy = 1024): 40 sin / 127 ~ (32 + 8) sin / 128
                        v_x := to_signed(C_SIN(to_integer(s_th(15 downto 10))), 13);
                        v_x := shift_right(v_x, 2) + shift_right(v_x, 4);
                        s_sway <= resize(v_x, 7);
                        f_st <= f_st + 1;
                    when 19 =>
                        -- cell grid anchor: first boundary at virtual x = 12
                        v_c16 := shift_left(resize(s_inc, 16), 3) + shift_left(resize(s_inc, 16), 2);
                        s_cp0 <= to_unsigned(0, 16) - v_c16;
                        case to_integer(s_xs) is
                            when 4      => s_incr <= s_inc & '0';
                            when 3      => s_incr <= resize(s_inc, 14) + resize(s_inc(12 downto 1), 14);
                            when others => s_incr <= resize(s_inc, 14);
                        end case;
                        -- rows: first boundary at virtual y = 10
                        s_racc0 <= to_unsigned(0, 16) - resize(s_rinc & "000", 16) - resize(s_rinc & '0', 16);
                        s_cyc8 <= resize(k3v(9 downto 1), 10) + resize(k3v(9 downto 2), 10);
                        f_st <= f_st + 1;
                    when 20 =>
                        s_linit <= '1';            -- start the line engine on line 0
                        f_st <= (others => '0');
                    when others =>
                        null;
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
    -- Generated avid (parity-locked start, field-latched width)
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
    -- Per-line engine: at the source avid rise it commits the values computed for this
    -- line and starts on the next one (~170 clocks, inside SD's 858-clock line).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_n : unsigned(16 downto 0);
        variable v_q : unsigned(31 downto 0);
        variable v_f : signed(12 downto 0);
        variable v_w : signed(11 downto 0);
        variable v_big : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' and s_avq = '0' then
                -- commit this line
                l_tick <= n_tick; l_j <= n_j; l_jk <= n_jk;
                l_tw <= n_tw; l_bc <= n_bc; l_bw <= n_bw; l_an <= n_an; l_aw <= n_aw;
                l_l <= n_l; l_r <= n_r; l_alive <= n_alive; l_low <= n_low; l_fxs <= n_fxs;
            end if;

            if s_linit = '1' then
                e_vy8 <= (others => '0'); e_racc <= s_racc0; e_j <= (others => '0');
                n_tick <= '1';
                e_st <= to_unsigned(2, 6);
            elsif data_in.avid = '1' and s_avq = '0' then
                e_st <= to_unsigned(1, 6);
            elsif e_mc /= 0 then
                if e_mb(0) = '1' then e_acc <= e_acc + e_ma; end if;
                e_ma <= e_ma(20 downto 0) & '0';
                e_mb <= '0' & e_mb(11 downto 1);
                e_mc <= e_mc - 1;
            else
                case to_integer(e_st) is
                    when 1 =>       -- advance to the next line
                        e_vy8 <= e_vy8 + resize(s_ys, 20);
                        v_n := resize(e_racc, 17) + s_rincr;
                        e_racc <= v_n(15 downto 0);
                        n_tick <= v_n(16);
                        if v_n(16) = '1' then e_j <= e_j + 1; end if;
                        e_st <= e_st + 1;
                    when 2 =>       -- Jk = ((j << 8) + roll) * cyc / 256
                        e_vy <= resize(e_vy8(19 downto 8), 11);
                        e_ma <= resize((resize(e_j, 12) & x"00") + s_roll, 22);
                        e_mb <= resize(s_cyc8, 12); e_acc <= (others => '0'); e_mc <= to_unsigned(10, 4);
                        e_st <= e_st + 1;
                    when 3 =>
                        n_jk <= e_acc(20 downto 8);
                        n_j <= e_j;
                        if e_vy > 1079 then e_vy <= to_unsigned(1079, 11); end if;
                        e_st <= e_st + 1;
                    when 4 =>
                        if s_flip = '1' then e_yv <= to_unsigned(1079, 11) - e_vy; else e_yv <= e_vy; end if;
                        e_st <= e_st + 1;
                    when 5 =>       -- q = yv * isy
                        e_ma <= resize(e_yv, 22); e_mb <= resize(s_isy, 12); e_acc <= (others => '0'); e_mc <= to_unsigned(10, 4);
                        e_st <= e_st + 1;
                    when 6 =>
                        if e_acc(21 downto 12) >= 79 then
                            e_ti <= to_unsigned(79, 7); e_fr <= (others => '0');
                        else
                            e_ti <= e_acc(18 downto 12); e_fr <= e_acc(11 downto 8);
                        end if;
                        e_st <= e_st + 1;
                    when 7 =>       -- vy^2
                        e_ma <= resize(e_vy, 22); e_mb <= resize(e_vy, 12); e_acc <= (others => '0'); e_mc <= to_unsigned(11, 4);
                        e_st <= e_st + 1;
                    when 8 =>
                        e_vyq <= e_acc(21 downto 10);
                        e_st <= e_st + 1;
                    when 9 =>       -- sway at this line: |S| * vy^2 / 2^20
                        e_ma <= resize(e_vyq, 22);
                        if s_sway < 0 then e_mb <= unsigned(resize(-s_sway, 12)); else e_mb <= unsigned(resize(s_sway, 12)); end if;
                        e_acc <= (others => '0'); e_mc <= to_unsigned(7, 4);
                        e_st <= e_st + 1;
                    when 10 =>
                        if s_sway < 0 then
                            e_fx <= signed(resize(s_fxs, 13)) + signed(resize(e_acc(16 downto 10), 13));
                        else
                            e_fx <= signed(resize(s_fxs, 13)) - signed(resize(e_acc(16 downto 10), 13));
                        end if;
                        e_p <= (others => '0'); e_base <= (others => '0');
                        e_st <= e_st + 1;
                    when 11 =>
                        if e_fx < 0 then
                            e_fx <= e_fx + signed(resize(s_pf, 13));
                        elsif e_fx >= signed(resize(s_pf, 13)) then
                            e_fx <= e_fx - signed(resize(s_pf, 13));
                        end if;
                        e_st <= e_st + 1;
                    when 12 =>      -- flame table: read entries ti and ti + 1
                        n_fxs <= unsigned(e_fx(10 downto 0));
                        if e_ti >= 45 then n_low <= '1'; else n_low <= '0'; end if;   -- lower flame: dark gaps
                        s_fta <= e_base + resize(e_ti, 9);
                        e_st <= e_st + 1;
                    when 13 =>
                        if e_ti /= 79 then s_fta <= e_base + resize(e_ti, 9) + 1; end if;
                        e_st <= e_st + 1;
                    when 14 =>
                        e_v0 <= unsigned(s_ftq);
                        e_st <= e_st + 1;
                    when 15 =>
                        e_v1 <= unsigned(s_ftq);
                        e_st <= e_st + 1;
                    when 16 =>
                        e_d <= signed(resize(e_v1, 9)) - signed(resize(e_v0, 9));
                        e_st <= e_st + 1;
                    when 17 =>
                        v_f := resize(e_d, 13);
                        if e_fr(0) = '1' then e_t1 <= v_f; else e_t1 <= (others => '0'); end if;
                        if e_fr(2) = '1' then e_t2 <= shift_left(v_f, 2); else e_t2 <= (others => '0'); end if;
                        e_st <= e_st + 1;
                    when 18 =>
                        v_f := resize(e_d, 13);
                        if e_fr(1) = '1' then e_t1 <= e_t1 + shift_left(v_f, 1); end if;
                        if e_fr(3) = '1' then e_t2 <= e_t2 + shift_left(v_f, 3); end if;
                        e_st <= e_st + 1;
                    when 19 =>
                        e_dfr <= e_t1 + e_t2;
                        e_st <= e_st + 1;
                    when 20 =>
                        v_f := signed(resize(e_v0, 13)) + shift_right(e_dfr, 4);
                        e_v <= unsigned(v_f(7 downto 0));
                        e_st <= e_st + 1;
                    when 21 =>      -- scale by sx
                        e_ma <= resize(e_v, 22); e_mb <= resize(s_sx8, 12); e_acc <= (others => '0'); e_mc <= to_unsigned(9, 4);
                        e_st <= e_st + 1;
                    when 22 =>
                        case to_integer(e_p) is
                            when 0      => n_tw <= e_acc(15 downto 8);
                            when 1      => n_bc <= e_acc(15 downto 8);
                            when 2      => n_bw <= e_acc(15 downto 8);
                            when 3      => n_an <= e_acc(15 downto 8);
                            when others => n_aw <= e_acc(15 downto 8);
                        end case;
                        e_p <= e_p + 1;
                        e_base <= e_base + 80;
                        if e_p = 4 then e_st <= to_unsigned(23, 6); else e_st <= to_unsigned(12, 6); end if;
                    when 23 =>
                        if s_nocore = '1' then n_tw <= (others => '0'); end if;
                        e_st <= e_st + 1;
                    when 24 =>      -- envelope candidates (tc = 4)
                        v_big := to_signed(2047, 12);
                        if n_aw /= 0 then e_aL <= -signed(resize(n_an, 12)) - signed(resize(n_aw, 12));
                                          e_aR <= -signed(resize(n_an, 12)) + signed(resize(n_aw, 12));
                        else e_aL <= v_big; e_aR <= -v_big; end if;
                        if n_tw /= 0 then e_tL <= 4 - signed(resize(n_tw, 12));
                                          e_tR <= 4 + signed(resize(n_tw, 12));
                        else e_tL <= v_big; e_tR <= -v_big; end if;
                        if n_bw /= 0 then e_bL <= signed(resize(n_bc, 12)) - signed(resize(n_bw, 12));
                                          e_bR <= signed(resize(n_bc, 12)) + signed(resize(n_bw, 12));
                        else e_bL <= v_big; e_bR <= -v_big; end if;
                        e_st <= e_st + 1;
                    when 25 =>
                        if e_aL < e_tL then e_m1 <= e_aL; else e_m1 <= e_tL; end if;
                        if e_aR > e_tR then e_m2 <= e_aR; else e_m2 <= e_tR; end if;
                        e_st <= e_st + 1;
                    when 26 =>
                        if e_m1 < e_bL then n_l <= e_m1 - 16; else n_l <= e_bL - 16; end if;
                        if e_m2 > e_bR then n_r <= e_m2 + 16; else n_r <= e_bR + 16; end if;
                        if n_tw /= 0 or n_bw /= 0 or n_aw /= 0 then n_alive <= '1'; else n_alive <= '0'; end if;
                        e_st <= (others => '0');
                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 0): cell phase and number, flame coordinate (Q1)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_c : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            b_cs <= '0';
            if g_start = '1' then
                b_cacc <= s_cp0;
                b_ci <= (others => '0');
                b_cs <= '1';
                b_pxv <= (others => '0');
                b_fx2 <= l_fxs & '0';
            elsif g_avid = '1' then
                v_c := resize(b_cacc, 17) + resize(s_incr, 17);
                b_cacc <= v_c(15 downto 0);
                if v_c(16) = '1' then
                    b_ci <= b_ci + 1;
                    b_cs <= '1';
                    b_pxv <= (others => '0');
                elsif b_pxv < 27 then
                    b_pxv <= b_pxv + resize(s_xs, 5);
                end if;
                if b_fx2 >= s_pf2m then
                    b_fx2 <= b_fx2 + resize(s_xs, 12) - s_pf2;
                else
                    b_fx2 <= b_fx2 + resize(s_xs, 12);
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Video luma (L8 = 0..255 over black .. white) and its running mean
    ---------------------------------------------------------------------------
    p_vid : process(clk)
        variable v_l : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            v_y1 <= unsigned(data_in.y);
            if v_y1 > 64 then v_d2 <= v_y1 - 64; else v_d2 <= (others => '0'); end if;
            v_l := resize(v_d2(9 downto 2), 11) + resize(v_d2(9 downto 5), 11) + resize(v_d2(9 downto 6), 11);
            if v_l > 255 then v_l3 <= (others => '1'); else v_l3 <= v_l(7 downto 0); end if;
            v_l4 <= v_l3; v_l5 <= v_l4;
            -- mean of the cell samples: avg += (L << 8 - avg) / 1024
            s_adv <= s_cwe;
            s_adf <= signed(resize(unsigned(s_cwd) & x"00", 17)) - signed(resize(s_avg, 17));
            if s_adv = '1' then
                s_avg <= unsigned(signed(s_avg) + resize(shift_right(s_adf, 10), 16));
            end if;
        end if;
    end process p_vid;

    ---------------------------------------------------------------------------
    -- Pixel pipeline
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_h : unsigned(15 downto 0);
        variable v_r : unsigned(4 downto 0);
        variable v_rs : signed(5 downto 0);
        variable v_j : unsigned(12 downto 0);
        variable v_g : unsigned(12 downto 0);
        variable v_ph : unsigned(12 downto 0);
        variable v_c4 : unsigned(1 downto 0);
        variable v_idx : unsigned(2 downto 0);
        variable v_a, v_b : std_logic;
        variable v_prev : unsigned(2 downto 0);
        variable v_e : signed(12 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_r8 : unsigned(8 downto 0);
        variable v_s9 : unsigned(8 downto 0);
        variable v_rt : unsigned(11 downto 0);
        variable v_b17 : signed(16 downto 0);
        variable v_y : unsigned(16 downto 0);
        variable v_cc : signed(16 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_rr, v_gg, v_bv : unsigned(7 downto 0);
        variable v_o : t_u8(0 to 2);
    begin
        if rising_edge(clk) then
            -------------------------------------------------------------------
            -- 1: cell number, flame dx, hash round 1
            p_ci(1) <= b_ci; p_cs(1) <= b_cs; p_pxv(1) <= b_pxv;
            v_d := signed(resize(b_fx2, 13)) - signed(resize(s_pf, 13));
            p_dx1 <= resize(shift_right(v_d, 1), 12);
            s_cra <= b_ci;
            v_h := ((b_ci & l_j & "000") xor (s_seed & s_seed(4 downto 0) & s_seed(7 downto 5))) xor x"A5C3";
            h2 <= v_h + shift_left(v_h, 5);

            -- 2: tube offsets (teardrop centre 4), envelope distances; hash round 2
            p_ci(2) <= p_ci(1);
            p_uT2 <= resize(p_dx1, 13) - 4;
            p_uB2 <= resize(p_dx1, 13) - signed(resize(l_bc, 13));
            p_uA2 <= resize(p_dx1, 13) + signed(resize(l_an, 13));
            p_dl2 <= resize(l_l, 13) - resize(p_dx1, 13);
            p_dr2 <= resize(p_dx1, 13) - resize(l_r, 13);
            v_h := h2 xor shift_right(h2, 7);
            h3 <= v_h + shift_left(v_h, 3);

            -- 3: |offsets|, outside distance; column ROM / cell RAM to fabric; hash round 3; RAM write
            p_ci(3) <= p_ci(2);
            p_col3 <= p_colq2;
            p_ram3 <= s_crq;
            if p_uT2 < 0 then p_dT3 <= unsigned(resize(-p_uT2, 12)); else p_dT3 <= unsigned(resize(p_uT2, 12)); end if;
            if p_uB2 < 0 then p_dB3 <= unsigned(resize(-p_uB2, 12)); else p_dB3 <= unsigned(resize(p_uB2, 12)); end if;
            if p_uA2 < 0 then p_dA3 <= unsigned(resize(-p_uA2, 12)); else p_dA3 <= unsigned(resize(p_uA2, 12)); end if;
            if p_uB2 < 0 then p_lft3 <= '1'; else p_lft3 <= '0'; end if;
            if p_dl2 > p_dr2 then p_do3 <= p_dl2; else p_do3 <= p_dr2; end if;
            v_h := h3 xor shift_right(h3, 9);
            h4 <= v_h + shift_left(v_h, 11);
            s_cwe <= '0';
            if l_tick = '1' and p_cs(3) = '1' then
                s_cwe <= '1'; s_cwa <= p_ci(3); s_cwd <= std_logic_vector(v_l3);
            end if;

            -- 4: rate x Jk partial sums; noise level; tube edge distances; halo partial; cell luma
            v_rs := abs(signed(p_col3(9 downto 4)));
            v_r := unsigned(v_rs(4 downto 0));
            v_j := l_jk;
            if v_r(0) = '1' then v_g := v_j; else v_g := (others => '0'); end if;
            if v_r(1) = '1' then v_g := v_g + shift_left(v_j, 1); end if;
            if v_r(2) = '1' then v_g := v_g + shift_left(v_j, 2); end if;
            p_pa4 <= v_g;
            if v_r(3) = '1' then v_g := shift_left(v_j, 3); else v_g := (others => '0'); end if;
            if v_r(4) = '1' then v_g := v_g + shift_left(v_j, 4); end if;
            p_pb4 <= v_g;
            p_neg4 <= p_col3(9);
            p_ph4 <= unsigned(p_col3(15 downto 10));
            if p_col3 = x"6040" and l_j >= 48 then
                p_nl4 <= to_unsigned(170, 8);                    -- flame columns break up low down
            else
                p_nl4 <= unsigned(p_col3(3 downto 0)) & unsigned(p_col3(3 downto 0));     -- namp * 17
            end if;
            p_eT4 <= signed(resize(l_tw, 13)) - signed(resize(p_dT3, 13));
            p_eB4 <= signed(resize(l_bw, 13)) - signed(resize(p_dB3, 13));
            p_eA4 <= signed(resize(l_aw, 13)) - signed(resize(p_dA3, 13));
            p_lft4 <= p_lft3;
            if p_do3 <= 0 then
                p_kin4 <= '1'; p_k3a4 <= (others => '0');
            else
                p_kin4 <= '0';
                if s_hard = '1' then
                    if p_do3 > 3 then p_k3a4 <= to_unsigned(16383, 14);
                    else p_k3a4 <= resize(unsigned(p_do3(1 downto 0)) & "000000", 14); end if;
                elsif p_do3 > 1000 then
                    p_k3a4 <= to_unsigned(16383, 14);
                else
                    p_k3a4 <= resize(unsigned(p_do3(9 downto 0)) & '0', 14) + resize(unsigned(p_do3(9 downto 0)), 14) + resize(unsigned(p_do3(9 downto 2)), 14);
                end if;
            end if;
            if l_tick = '1' then
                if p_cs(3) = '1' then p_lc4 <= v_l3; s_hold <= v_l3; else p_lc4 <= s_hold; end if;
            else
                p_lc4 <= unsigned(p_ram3);
            end if;
            h5 <= h4 xor shift_right(h4, 8);

            -- 5: product; tube membership, edge ramp, profile inputs; halo K (0..32)
            p_p5 <= p_pa4 + p_pb4;
            p_neg5 <= p_neg4; p_ph5 <= p_ph4; p_nl5 <= p_nl4;
            if p_eT4 > -16 and l_tw /= 0 then p_inT5 <= '1'; else p_inT5 <= '0'; end if;
            if p_eB4 > -16 and l_bw /= 0 then p_inB5 <= '1'; else p_inB5 <= '0'; end if;
            if p_eA4 > -16 and l_aw /= 0 then p_inA5 <= '1'; else p_inA5 <= '0'; end if;
            v_e := p_eT4 + 16; if v_e < 0 then v_e := (others => '0'); elsif v_e > 32 then v_e := to_signed(32, 13); end if;
            p_ebT5 <= unsigned(v_e(5 downto 0));
            v_e := p_eB4 + 16; if v_e < 0 then v_e := (others => '0'); elsif v_e > 32 then v_e := to_signed(32, 13); end if;
            p_ebB5 <= unsigned(v_e(5 downto 0));
            v_e := p_eA4 + 16; if v_e < 0 then v_e := (others => '0'); elsif v_e > 32 then v_e := to_signed(32, 13); end if;
            p_ebA5 <= unsigned(v_e(5 downto 0));
            v_e := p_eT4; if v_e < 0 then v_e := (others => '0'); elsif v_e > 64 then v_e := to_signed(64, 13); end if;
            p_e6T5 <= unsigned(v_e(6 downto 0));
            v_e := p_eB4; if v_e < 0 then v_e := (others => '0'); elsif v_e > 48 then v_e := to_signed(48, 13); end if;
            p_e4B5 <= unsigned(v_e(6 downto 0));
            v_e := p_eA4; if v_e < 0 then v_e := (others => '0'); elsif v_e > 16 then v_e := to_signed(16, 13); end if;
            p_e1A5 <= unsigned(v_e(6 downto 0));
            if p_eB4 < 22 then p_n22B5 <= '1'; else p_n22B5 <= '0'; end if;
            p_lft5 <= p_lft4;
            if l_alive = '0' then
                p_k5 <= (others => '0');
            elsif p_kin4 = '1' then
                p_k5 <= to_unsigned(32, 6);
            elsif p_k3a4 >= 256 then
                p_k5 <= (others => '0');
            else
                p_k5 <= to_unsigned(32, 6) - resize(p_k3a4(7 downto 3), 6);
            end if;
            p_lc5 <= p_lc4;
            h6 <= h5;

            -- 6: colourizer phase -> quarter; thresholds; tube profiles; cell luma vs its mean
            if p_neg5 = '1' then v_ph := (p_ph5 & "0000000") - p_p5; else v_ph := (p_ph5 & "0000000") + p_p5; end if;
            p_c46 <= v_ph(12 downto 11);
            p_t1 <= p_nl5 - shift_right(p_nl5, 2);
            p_t2 <= shift_right(p_nl5, 1) - shift_right(p_nl5, 4);
            p_t3 <= shift_right(p_nl5, 2) + shift_right(p_nl5, 3);
            p_t4 <= shift_right(p_nl5, 2) + shift_right(p_nl5, 4);
            p_ld6 <= signed(resize(p_lc5, 9)) - signed(resize(s_avg(15 downto 8), 9));
            if s_osc = '0' then
                v_r8 := resize(v_l5, 9) - resize(v_l5(7 downto 2), 9) + resize(v_l5(7 downto 4), 9) + 31;
                p_rT6 <= v_r8(7 downto 0); p_rB6 <= v_r8(7 downto 0); p_rA6 <= v_r8(7 downto 0);
            else
                p_rT6 <= to_unsigned(64, 8) + resize(p_e6T5, 8) + resize(p_e6T5(6 downto 2), 8) + resize(p_e6T5(6 downto 5), 8);
                if p_lft5 = '1' then
                    if p_n22B5 = '1' then p_rB6 <= to_unsigned(219, 8); else p_rB6 <= to_unsigned(107, 8); end if;
                else
                    p_rB6 <= to_unsigned(56, 8) + resize(p_e4B5, 8) + resize(p_e4B5(6 downto 3), 8);
                end if;
                p_rA6 <= to_unsigned(46, 8) + resize(p_e1A5 & '0', 8) + resize(p_e1A5, 8) + resize(p_e1A5(6 downto 1), 8);
            end if;
            p_inT6 <= p_inT5; p_inB6 <= p_inB5; p_inA6 <= p_inA5;
            p_ebT6 <= p_ebT5; p_ebB6 <= p_ebB5; p_ebA6 <= p_ebA5;
            p_k(6) <= p_k5;

            -- 7: colour index (Osc noise rules / Video R bit); tube priority
            v_c4 := p_c46;
            if s_osc = '1' then
                if v_c4 = 3 and h6(7 downto 0) < p_t1 then v_c4 := "10";
                elsif v_c4 = 1 and h6(7 downto 0) < p_t2 then v_c4 := "00"; end if;
                v_idx := v_c4 & '0';
                if v_c4 = 2 and h6(15 downto 8) < p_t3 then
                    if (h6(0) xor h6(15)) = '1' then v_idx := "011"; else v_idx := "101"; end if;
                elsif v_c4 = 1 and h6(15 downto 8) < p_t4 then
                    v_idx := "001";
                end if;
            else
                v_a := p_c46(1);
                if p_ld6 > 0 then v_b := '1'; else v_b := '0'; end if;
                -- Gray: (a, b) = 00 G, 01 P, 11 Y, 10 M
                v_idx := (v_a & (v_a xor v_b)) & '0';
                if p_ld6 > -10 and p_ld6 < 10 then
                    if v_a = '0' then v_idx := "011"; else v_idx := "101"; end if;
                end if;
            end if;
            p_idx7 <= v_idx;
            if p_inT6 = '1' then p_r7 <= p_rT6; p_eb7 <= p_ebT6;
            elsif p_inB6 = '1' then p_r7 <= p_rB6; p_eb7 <= p_ebB6;
            else p_r7 <= p_rA6; p_eb7 <= p_ebA6; end if;
            p_any(7) <= p_inT6 or p_inB6 or p_inA6;

            -- 8: previous cell's colour and the blue fringe; tube R and B
            if p_cs(7) = '1' then
                v_prev := s_cur;
                s_prev <= s_cur; s_cur <= p_idx7;
            else
                v_prev := s_prev;
            end if;
            p_idx8 <= p_idx7;
            if s_nofr = '0' and p_idx7 = 0 and v_prev /= 0 and p_pxv(7) < 14 then p_fr8 <= '1'; else p_fr8 <= '0'; end if;
            v_rt := p_r7 * p_eb7(5 downto 2);
            p_rt(8) <= v_rt(10 downto 3);
            if p_eb7(5) = '1' then p_bt(8) <= to_unsigned(255, 8); else p_bt(8) <= p_eb7(4 downto 0) & "000"; end if;

            -- 9: palette
            if p_fr8 = '1' then
                p_rgb9(0) <= to_unsigned(13, 8); p_rgb9(1) <= to_unsigned(38, 8); p_rgb9(2) <= to_unsigned(255, 8);
            else
                case to_integer(p_idx8) is
                    when 0 => p_rgb9(0) <= to_unsigned(0, 8);   p_rgb9(1) <= to_unsigned(255, 8); p_rgb9(2) <= to_unsigned(0, 8);
                    when 1 => p_rgb9(0) <= to_unsigned(232, 8); p_rgb9(1) <= to_unsigned(10, 8);  p_rgb9(2) <= to_unsigned(255, 8);
                    when 2 => p_rgb9(0) <= to_unsigned(255, 8); p_rgb9(1) <= to_unsigned(13, 8);  p_rgb9(2) <= to_unsigned(189, 8);
                    when 3 => p_rgb9(0) <= to_unsigned(242, 8); p_rgb9(1) <= to_unsigned(117, 8); p_rgb9(2) <= to_unsigned(54, 8);
                    when 4 => p_rgb9(0) <= to_unsigned(255, 8); p_rgb9(1) <= to_unsigned(240, 8); p_rgb9(2) <= to_unsigned(0, 8);
                    when 5 => p_rgb9(0) <= to_unsigned(130, 8); p_rgb9(1) <= to_unsigned(229, 8); p_rgb9(2) <= to_unsigned(31, 8);
                    when 6 => p_rgb9(0) <= to_unsigned(3, 8);   p_rgb9(1) <= to_unsigned(250, 8); p_rgb9(2) <= to_unsigned(135, 8);
                    when others => p_rgb9(0) <= to_unsigned(5, 8); p_rgb9(1) <= to_unsigned(245, 8); p_rgb9(2) <= to_unsigned(60, 8);
                end case;
            end if;

            -- 10: 2-px box on the mosaic
            for c in 0 to 2 loop
                p_sr1(c) <= p_rgb9(c);
                v_s9 := resize(p_rgb9(c), 9) + resize(p_sr1(c), 9);
                p_m10(c) <= v_s9(8 downto 1);
            end loop;

            -- 11: keyed mosaic: G * (1 - K), B -> lerp(B, max(R, blue fringe), K); K in 32nds
            p_mr11 <= p_m10(0); p_mg11 <= p_m10(1); p_mb11 <= p_m10(2);
            if p_m10(2) > 153 and p_m10(0) < 77 and p_m10(1) < 128 and p_m10(2) > p_m10(0) then
                v_rr := p_m10(2);
            else
                v_rr := p_m10(0);
            end if;
            p_db11 <= signed(resize(v_rr, 9)) - signed(resize(p_m10(2), 9));
            p_kn11 <= to_unsigned(32, 6) - p_k(10);

            -- 12: products
            p_gp12 <= p_mg11 * p_kn11;
            p_bp12 <= resize(p_db11 * signed('0' & p_k(11)), 16);
            p_mr12 <= p_mr11; p_mb12 <= p_mb11;
            if l_low = '1' and p_k(11) = 32 then p_dk12 <= '1'; else p_dk12 <= '0'; end if;

            -- 13: sums
            v_b17 := shift_right(resize(p_bp12, 17), 5) + signed(resize(p_mb12, 17));
            if p_dk12 = '1' then
                p_g13 <= (others => '0'); p_b13 <= (others => '0'); p_mr13 <= (others => '0');
            else
                p_g13 <= p_gp12(12 downto 5);
                if v_b17 < 0 then p_b13 <= (others => '0'); elsif v_b17 > 255 then p_b13 <= (others => '1'); else p_b13 <= unsigned(v_b17(7 downto 0)); end if;
                p_mr13 <= p_mr12;
            end if;
            p_any13 <= p_any(12); p_rt13 <= p_rt(12); p_bt13 <= p_bt(12);

            -- 14: tubes over the keyed mosaic, then the RGB patch
            if p_any13 = '1' then
                v_o(0) := p_rt13; v_o(1) := (others => '0'); v_o(2) := p_bt13;
            else
                v_o(0) := p_mr13; v_o(1) := p_g13; v_o(2) := p_b13;
            end if;
            case to_integer(s_patch) is
                when 1      => p_o14(0) <= v_o(0); p_o14(1) <= v_o(2); p_o14(2) <= v_o(1);
                when 2      => p_o14(0) <= v_o(1); p_o14(1) <= v_o(0); p_o14(2) <= v_o(2);
                when 3      => p_o14(0) <= v_o(1); p_o14(1) <= v_o(2); p_o14(2) <= v_o(0);
                when 4      => p_o14(0) <= v_o(2); p_o14(1) <= v_o(0); p_o14(2) <= v_o(1);
                when 5      => p_o14(0) <= v_o(2); p_o14(1) <= v_o(1); p_o14(2) <= v_o(0);
                when others => p_o14 <= v_o;
            end case;

            -- 15..17: RGB -> 10-bit YUV (u = Cr, v = Cb), Q6, luma pre-boosted 16/13:
            -- Y = 64 + (81 R + 159 G + 31 B) / 64, Cr = 512 + (113 R - 94 G - 18 B) / 64,
            -- Cb = 512 + (-38 R - 75 G + 113 B) / 64
            v_rr := p_o14(0); v_gg := p_o14(1); v_bv := p_o14(2);
            p_yr15 <= shift_left(resize(v_rr, 16), 6) + shift_left(resize(v_rr, 16), 4) + resize(v_rr, 16);
            p_yg15 <= shift_left(resize(v_gg, 16), 7) + shift_left(resize(v_gg, 16), 5) - resize(v_gg, 16);
            p_yb15 <= shift_left(resize(v_bv, 16), 5) - resize(v_bv, 16);
            p_ur15 <= signed(shift_left(resize(v_rr, 17), 7) - shift_left(resize(v_rr, 17), 4) + resize(v_rr, 17));
            p_ug15 <= -signed(shift_left(resize(v_gg, 17), 6) + shift_left(resize(v_gg, 17), 5) - shift_left(resize(v_gg, 17), 1));
            p_ub15 <= -signed(shift_left(resize(v_bv, 17), 4) + shift_left(resize(v_bv, 17), 1));
            p_vr15 <= -signed(shift_left(resize(v_rr, 17), 5) + shift_left(resize(v_rr, 17), 2) + shift_left(resize(v_rr, 17), 1));
            p_vg15 <= -signed(shift_left(resize(v_gg, 17), 6) + shift_left(resize(v_gg, 17), 3) + shift_left(resize(v_gg, 17), 1) + resize(v_gg, 17));
            p_vb15 <= signed(shift_left(resize(v_bv, 17), 7) - shift_left(resize(v_bv, 17), 4) + resize(v_bv, 17));
            p_ys16 <= resize(p_yr15, 17) + resize(p_yg15, 17) + resize(p_yb15, 17);
            p_us16 <= p_ur15 + p_ug15 + p_ub15;
            p_vs16 <= p_vr15 + p_vg15 + p_vb15;
            v_y := shift_right(p_ys16, 6) + 64;
            if v_y > 940 then s_y17 <= to_unsigned(940, 10); else s_y17 <= v_y(9 downto 0); end if;
            v_cc := shift_right(p_us16, 6) + 512;
            s_u17 <= unsigned(v_cc(9 downto 0));
            v_cc := shift_right(p_vs16, 6) + 512;
            s_v17 <= unsigned(v_cc(9 downto 0));

            -- 18..23: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent
            -- 8-sums, luma [1 2 1]; centre at stage 22 -> 23)
            s_csu(0) <= s_u17; s_csv(0) <= s_v17;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u17 - s_csu(7);
            s_boxv <= s_boxv + s_v17 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v23 <= v_sc(13 downto 4);
            s_y18 <= s_y17; s_y19 <= s_y18; s_y20 <= s_y19; s_y21 <= s_y20;
            s_y22 <= s_y21; s_y23x <= s_y22;
            v_sc := resize(s_y21, 15) + shift_left(resize(s_y22, 15), 1) + resize(s_y23x, 15);
            s_yf23 <= v_sc(11 downto 2);
            -- 24: luma toward black (white 940 -> 777); blanking gate
            if s_yf23 > 64 then v_dl := s_yf23 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(22) = '1' then
                s_y24 <= s_yf23 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u24 <= s_u23; s_v24 <= s_v23;
            else
                s_y24 <= to_unsigned(64, 10); s_u24 <= to_unsigned(512, 10); s_v24 <= to_unsigned(512, 10);
            end if;

            -- delays
            for i in 2 to 7 loop p_cs(i) <= p_cs(i - 1); p_pxv(i) <= p_pxv(i - 1); end loop;
            for i in 8 to 12 loop p_any(i) <= p_any(i - 1); end loop;
            for i in 7 to 11 loop p_k(i) <= p_k(i - 1); end loop;
            for i in 9 to 12 loop p_bt(i) <= p_bt(i - 1); p_rt(i) <= p_rt(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture cathode;
