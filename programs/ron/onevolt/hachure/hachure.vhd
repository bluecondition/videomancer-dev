-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Hachure v0.1.0 -- Onevolt 9: a figure drawn in four hatchings.
--
-- The rack:
--   H / V ramps -> Navigator (rotation, K4) -> Angles (0, 45, 90, 135 degrees) -> four Stairs (Steps, K3)
--        or four Keychain keys of the same ramps (S9) -> BSO Scanner inputs 1 .. 4     (S8: four angles / four pitches)
--   DSG3 Average or Absolute (S7) -> Proc (bias K1 Scan, gain P12 Span) + camera luma (K6; S11 Osc) -> Scanner SCAN
--   Scanner crossfade width (K2 Blend: 0 = the Quarterizer's hard switching .. the full interpolation)
--   Scanner out -> luma; the scan level's zone picks one of four Matte colours through SMX3 (K5 Colour,
--   S10 Zones / Tint)
--
--   K1  Scan    Proc bias on SCAN: a full turn cycles the four textures through every zone
--   K2  Blend   Scanner crossfade width: 0 = hard switching (crisp borders) .. full interpolation
--   K3  Pitch   Stairs Steps: stripe period 240 .. 10 virtual px (faded near the pixel limit)
--   K4  Angle   Navigator rotation of all four directions, 0 .. 180 degrees
--   K5  Colour  eight designed four-colour Matte sets, detented, blended between
--   K6  Camera  camera luma into SCAN, 0 .. 2 passes per full luma
--   S7  Gradient  Average (diamond) / Absolute (saltire)
--   S8  Textures  Angles (0 / 45 / 90 / 135) / Pitches (one angle, x1/2, x1, x2, x4)
--   S9  Shaper    Stairs (soft triangle stripes) / Keychain (hard stripes, 70 % colour, 1-px edges)
--   S10 Colour    Zones / Tint
--   S11 Source    Video / Osc
--   P12 SPAN    Proc gain on SCAN: half a pass (two broad zones) .. 2 passes (centre) .. 6 passes (a nest of rings)
--
-- Engine: vblank-zone sequencer (one 32-bit serial multiply-accumulate, 57 slots: tent steps, stripe frequency,
-- Navigator sin / cos products, per-texture pixel / line steps and top-left phases, camera gain, the K5 blend of the
-- Matte colours; shared knob glide), then a table writer (one DDA engine and a restoring divider) that rewrites the
-- crossfade table and the four shaper tables every field; per-line DDAs (the V tent, the four texture line phases);
-- per-pixel DDAs (the H tent, four texture phases); a 14-stage pixel path.  Virtual 1080-line coordinates (the scan
-- diamond spans the width).  Generated avid; output conditioning (chroma through two [1 2 2 2 2 2 2 2 1]/16 boxes,
-- luma [1 2 1]/4 one pixel ahead of the chroma centre, luma 13/16) before the blanking gate.
-- Colours in the hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.hachure_pkg.all;
use work.all;

architecture hachure of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

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
    -- the V tent's step: round(ys x 2^16 / 270)
    function f_cv(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(276214, 19);
            when 4          => return to_unsigned(139810, 19);
            when 8          => return to_unsigned(233017, 19);
            when 12         => return to_unsigned(116508, 19);
            when 5 | 6 | 14 => return to_unsigned(93207, 19);
            when 1 | 2 | 10 => return to_unsigned(124276, 19);
            when others     => return to_unsigned(62138, 19);
        end case;
    end function;
    -- the Osc lattice's line step: (ys x 121) >> 8
    function f_oys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(537, 10);
            when 4          => return to_unsigned(272, 10);
            when 8          => return to_unsigned(453, 10);
            when 12         => return to_unsigned(226, 10);
            when 5 | 6 | 14 => return to_unsigned(181, 10);
            when 1 | 2 | 10 => return to_unsigned(242, 10);
            when others     => return to_unsigned(121, 10);
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
    function f_tri8(p : unsigned(15 downto 0)) return unsigned is
    begin
        if p(15) = '1' then return not p(14 downto 7); else return p(14 downto 7); end if;
    end function;
    -- zone-colour slots: the slot about to start (n + 1 for an address issued in slot n) -> {zone, channel}
    function f_pidx(n : unsigned(5 downto 0)) return unsigned is
    begin
        case to_integer(n) is
            when 32 | 33 => return "0000";  when 34 | 35 => return "0001";  when 36 | 37 => return "0010";
            when 38 | 39 => return "0100";  when 40 | 41 => return "0101";  when 42 | 43 => return "0110";
            when 44 | 45 => return "1000";  when 46 | 47 => return "1001";  when 48 | 49 => return "1010";
            when 50 | 51 => return "1100";  when 52 | 53 => return "1101";  when others => return "1110";
        end case;
    end function;
    function f_zidx(n : unsigned(5 downto 0)) return integer is
    begin
        case to_integer(n) is
            when 34 => return 0;  when 36 => return 1;  when 38 => return 2;  when 40 => return 3;
            when 42 => return 4;  when 44 => return 5;  when 46 => return 6;  when 48 => return 7;
            when 50 => return 8;  when 52 => return 9;  when 54 => return 10; when others => return 11;
        end case;
    end function;
    -- quarter-wave fold of a 10-bit angle: ROM address, the n = 256 flag, the sign
    function f_sa(a : unsigned(9 downto 0)) return unsigned is
        variable v_n : unsigned(8 downto 0);
    begin
        if a(8) = '1' then v_n := to_unsigned(256, 9) - resize(a(7 downto 0), 9);
        else v_n := resize(a(7 downto 0), 9); end if;
        return v_n(7 downto 0);
    end function;
    function f_sval(a : unsigned(9 downto 0); q : unsigned(15 downto 0)) return signed is
        variable v_m : signed(10 downto 0);
    begin
        if a(8) = '1' and a(7 downto 0) = 0 then v_m := to_signed(1023, 11);
        else v_m := signed(resize(q(9 downto 0), 11)); end if;
        if a(9) = '1' then return -v_m; else return v_m; end if;
    end function;

    -- luma cell store: one 12-bit vertical-slew state per 8-pixel cell (read a cell ahead of the write)
    type t_lb is array (0 to 255) of std_logic_vector(15 downto 0);
    signal lb : t_lb := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    signal s_lb_wa, s_lb_ra : unsigned(7 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(15 downto 0) := x"0000";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(15 downto 0) := x"0000";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    signal s_lin   : unsigned(7 downto 0) := (others => '0');
    signal s_lav, s_f1, s_f2 : std_logic := '0';
    signal s_lc    : unsigned(2 downto 0) := (others => '0');
    signal s_lcell, s_fcell, s_wcell : unsigned(7 downto 0) := (others => '0');
    signal s_acc, s_sum8 : unsigned(10 downto 0) := (others => '0');
    signal s_cur, s_old, s_new : unsigned(11 downto 0) := (others => '0');
    signal s_lv    : unsigned(11 downto 0) := (others => '0');
    signal s_h2 : unsigned(15 downto 0) := (others => '0');
    signal s_la, s_lbh : unsigned(13 downto 0) := (others => '0');
    signal s_lterm : unsigned(12 downto 0) := (others => '0');

    -- the four shaper tables (Stairs / Keychain transfer of the folded ramp), written in the zone
    type t_sh is array (0 to 255) of std_logic_vector(7 downto 0);
    signal sh0, sh1, sh2, sh3 : t_sh := (others => x"80");
    attribute no_rw_check of sh0 : signal is true;
    attribute no_rw_check of sh1 : signal is true;
    attribute no_rw_check of sh2 : signal is true;
    attribute no_rw_check of sh3 : signal is true;
    signal r_sa0, r_sa1, r_sa2, r_sa3 : unsigned(7 downto 0) := (others => '0');
    signal r_sq0, r_sq1, r_sq2, r_sq3 : std_logic_vector(7 downto 0) := x"80";
    signal w_she : std_logic_vector(3 downto 0) := "0000";
    -- the scanner crossfade table t(u)
    type t_tl is array (0 to 127) of std_logic_vector(7 downto 0);
    signal tl : t_tl := (others => x"00");
    attribute no_rw_check of tl : signal is true;
    signal r_t_a  : unsigned(6 downto 0) := (others => '0');
    signal r_t_q  : std_logic_vector(7 downto 0) := x"00";
    signal w_te   : std_logic := '0';
    -- ROMs (registered address and data)
    signal r_s_a  : unsigned(7 downto 0) := (others => '0');
    signal r_s_q  : unsigned(15 downto 0) := (others => '0');
    signal r_pa_a : unsigned(7 downto 0) := (others => '0');
    signal r_pa_q : unsigned(15 downto 0) := (others => '0');

    -- output filter stores (as the shell)
    type t_dl is array (0 to 15) of std_logic_vector(31 downto 0);
    signal da, db : t_dl := (others => (others => '0'));
    attribute no_rw_check of da : signal is true;
    attribute no_rw_check of db : signal is true;
    signal s_dp : unsigned(3 downto 0) := to_unsigned(0, 4);
    signal s_dr : unsigned(3 downto 0) := to_unsigned(10, 4);
    signal s_qa, s_qb, s_fa, s_fb : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_cv   : unsigned(18 downto 0) := to_unsigned(62138, 19);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(12 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_half : unsigned(10 downto 0) := to_unsigned(960, 11);

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(0, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_abs, s_pit, s_key, s_tnt, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- LFOs, field counter
    signal z_lb, z_ls : unsigned(15 downto 0) := x"4000";
    signal z_fc   : unsigned(15 downto 0) := (others => '0');
    type t_u20x4 is array (0 to 3) of unsigned(17 downto 0);

    -- per-field values
    signal z_bias : unsigned(9 downto 0) := (others => '0');
    signal z_span : unsigned(10 downto 0) := to_unsigned(512, 11);
    signal z_e3   : unsigned(10 downto 0) := (others => '0');
    signal z_th   : unsigned(16 downto 0) := (others => '0');
    signal z_i5, z_i51 : unsigned(2 downto 0);
    signal z_bl8  : unsigned(7 downto 0);
    signal z_ang  : unsigned(9 downto 0) := (others => '0');
    signal z_fn   : unsigned(2 downto 0) := (others => '0');
    signal z_sf1, z_sf2 : std_logic_vector(1 downto 0) := "00";
    signal z_ff   : std_logic_vector(3 downto 0) := "0000";
    signal z_ip   : std_logic_vector(3 downto 0) := "0000";
    signal z_fv1  : std_logic := '0';
    signal z_r0, z_r1, z_rb : signed(10 downto 0) := (others => '0');
    signal z_di   : signed(4 downto 0) := (others => '0');
    signal z_sn, z_cs : signed(15 downto 0) := (others => '0');
    signal z_dh   : unsigned(11 downto 0) := (others => '0');
    signal z_dv   : unsigned(12 downto 0) := (others => '0');
    signal z_fq   : unsigned(16 downto 0) := (others => '0');
    signal z_A, z_B, z_ABr, z_BAr : signed(17 downto 0) := (others => '0');
    signal z_fv   : unsigned(21 downto 0) := (others => '0');
    signal z_sx, z_sy : t_u20x4 := (others => (others => '0'));
    signal z_p0we : std_logic_vector(3 downto 0) := "0000";
    signal z_cgl  : unsigned(9 downto 0) := (others => '0');
    signal z_b8e  : unsigned(8 downto 0) := to_unsigned(1, 9);

    -- field serial multiply: acc = +- a x b (mod 2^32)
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal f_sub : std_logic := '0';
    signal z_fa  : signed(24 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_ng  : std_logic := '0';
    signal z_sl  : unsigned(5 downto 0) := (others => '0');
    signal z_ph  : unsigned(4 downto 0) := (others => '0');
    signal z_srun : std_logic := '0';
    signal z_pre, z_ld, z_keep : std_logic := '0';

    -- table writer: phase 0 idle, 1 crossfade table, 2 shapers
    signal w_go   : std_logic := '0';
    signal w_ph   : unsigned(2 downto 0) := (others => '0');
    signal w_st   : unsigned(1 downto 0) := (others => '0');
    signal w_c    : unsigned(8 downto 0) := (others => '0');
    signal w_k    : unsigned(1 downto 0) := (others => '0');
    signal w_acc  : signed(25 downto 0) := (others => '0');
    signal w_ws   : unsigned(15 downto 0) := (others => '0');
    signal w_rn, w_lo, w_hi : unsigned(8 downto 0);
    signal w_wa, w_va : unsigned(7 downto 0) := (others => '0');
    signal w_vv   : signed(17 downto 0) := (others => '0');
    signal w_vshe : std_logic_vector(3 downto 0) := "0000";
    signal w_vte  : std_logic := '0';
    signal w_wd   : std_logic_vector(7 downto 0) := x"00";
    signal w_f20  : unsigned(21 downto 0) := (others => '0');
    signal w_a    : unsigned(8 downto 0) := (others => '0');
    signal w_cap  : std_logic := '0';
    signal d_r    : unsigned(22 downto 0) := (others => '0');
    signal d_x    : unsigned(21 downto 0) := (others => '0');
    signal d_q    : unsigned(16 downto 0) := (others => '0');
    signal d_inj  : std_logic := '0';
    signal d_n    : unsigned(4 downto 0) := (others => '0');
    -- the four zone colours (K5 blend of two Matte sets), Y U V each
    type t_zcol is array (0 to 11) of unsigned(7 downto 0);
    signal z_zc   : t_zcol := (others => (others => '0'));

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine and beam
    signal e_gv  : unsigned(19 downto 0) := (others => '0');
    signal e_vq  : unsigned(18 downto 0) := (others => '0');
    signal e_L   : t_u20x4 := (others => (others => '0'));
    signal l_gv  : unsigned(19 downto 0) := (others => '0');
    signal b_ph  : t_u20x4 := (others => (others => '0'));
    signal b_gh  : unsigned(19 downto 0) := (others => '0');

    -- pixel path
    signal p2_dif : signed(21 downto 0) := (others => '0');
    signal p3_sd  : unsigned(12 downto 0) := (others => '0');
    signal p3_bc  : unsigned(12 downto 0) := (others => '0');
    signal p4_s   : unsigned(9 downto 0) := (others => '0');
    signal p4_t0, p4_t1, p4_t2, p4_t3 : unsigned(7 downto 0) := (others => '0');
    signal p5_a, p5_b : unsigned(7 downto 0) := (others => '0');
    signal p6_d   : signed(8 downto 0) := (others => '0');
    signal p6_a : unsigned(7 downto 0) := (others => '0');
    signal p7_d   : signed(8 downto 0) := (others => '0');
    signal p7_a, p7_t : unsigned(7 downto 0) := (others => '0');
    signal p5_z, p6_z, p7_z, p8_z, p9_z, p10_z, p11_z : unsigned(1 downto 0) := (others => '0');
    signal p8_pr  : signed(17 downto 0) := (others => '0');
    signal p8_a : unsigned(7 downto 0) := (others => '0');
    signal p12_col : std_logic_vector(29 downto 0) := (others => '0');
    signal p9_pair : unsigned(7 downto 0) := (others => '0');
    signal p10_pair, p11_pair : unsigned(7 downto 0) := (others => '0');
    signal p12_mono : unsigned(7 downto 0) := (others => '0');
    signal p13_py : unsigned(17 downto 0) := (others => '0');
    signal p13_pu, p13_pv : signed(15 downto 0) := (others => '0');
    signal s_y14 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bbu, s_bbv, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_l1, s_l2, s_l3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y18 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_KU : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KV : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories (one write statement each, registered reads)
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_sh0 : process(clk)
    begin
        if rising_edge(clk) then
            r_sq0 <= sh0(to_integer(r_sa0));
            if w_she(0) = '1' then sh0(to_integer(w_wa)) <= w_wd; end if;
        end if;
    end process p_sh0;
    p_sh1 : process(clk)
    begin
        if rising_edge(clk) then
            r_sq1 <= sh1(to_integer(r_sa1));
            if w_she(1) = '1' then sh1(to_integer(w_wa)) <= w_wd; end if;
        end if;
    end process p_sh1;
    p_sh2 : process(clk)
    begin
        if rising_edge(clk) then
            r_sq2 <= sh2(to_integer(r_sa2));
            if w_she(2) = '1' then sh2(to_integer(w_wa)) <= w_wd; end if;
        end if;
    end process p_sh2;
    p_sh3 : process(clk)
    begin
        if rising_edge(clk) then
            r_sq3 <= sh3(to_integer(r_sa3));
            if w_she(3) = '1' then sh3(to_integer(w_wa)) <= w_wd; end if;
        end if;
    end process p_sh3;

    p_tl : process(clk)
    begin
        if rising_edge(clk) then
            r_t_q <= tl(to_integer(r_t_a));
            if w_te = '1' then tl(to_integer(w_wa(6 downto 0))) <= w_wd; end if;
        end if;
    end process p_tl;



    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_s_q  <= C_HA_SIN(to_integer(r_s_a));
            r_pa_q <= C_HA_PAL(to_integer(r_pa_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_t  : unsigned(9 downto 0);
        variable v_a  : unsigned(13 downto 0);
        variable v_x  : signed(18 downto 0);
        variable v_ab : signed(16 downto 0);
        variable v_ang : unsigned(9 downto 0);
        variable v_n  : unsigned(8 downto 0);
        variable v_k  : integer range 0 to 3;
        variable v_sub : integer range 0 to 7;
        variable v_ms : unsigned(15 downto 0);
        variable v_pi : unsigned(3 downto 0);
        variable v_q, v_e2 : unsigned(9 downto 0);
        variable v_sp : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            s_cv  <= f_cv(s_tid);

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
            elsif s_zc /= 8191 then
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
                s_abs <= registers_in(6)(0);
                s_pit <= registers_in(6)(1);
                s_key <= registers_in(6)(2);
                s_tnt <= registers_in(6)(3);
                s_src <= registers_in(6)(4);
                s_xs  <= s_xsr;
                z_lb  <= z_lb + 68;
                z_ls  <= z_ls + 38;
                z_fc  <= z_fc + 1;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1  <= s_w - 1;
            s_half <= s_w(11 downto 1);
            s_oys  <= f_oys(s_tid);

            -- shared glide (1/8 of the gap per field, +-2 deadband; +-4 on the camera channel): four clocks
            -- per channel, run from the zone start by its own phase / channel counters
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g4; s_gr <= unsigned(registers_in(3));
                            when 4      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 5      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if s_gd > 32 or s_gd < -32 or (z_gc /= 5 and (s_gd > 16 or s_gd < -16)) then
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
                            when 3      => s_g4 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 5      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 6 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- decodes (zone clock 30, after the glide) and the Navigator's sine / cosine: one quarter-wave fold,
            -- the four ROM reads a, a + 1, a + 256, a + 257, then one interpolator (sin, then cos)
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30 =>
                        v_t := (s_g1(12 downto 3) xor "1000000000") + resize(unsigned(f_tri8(z_lb)), 10);
                        z_bias <= v_t;
                        v_a := resize(s_g4, 14) + shift_left(resize(unsigned(f_tri8(z_ls)), 14), 1);
                        z_th <= resize(v_a & "000", 17) - 2048;           -- K4 x 64 + the sway (tri - 128) x 16
                        z_e3 <= resize(s_g3(12 downto 3), 11) + resize(s_g3(12 downto 6), 11);
                        -- Span (P12): e = p below the centre, 512 + 3/4 (p - 512) above; 0.5 x 2^(e / 256) passes
                        if s_gp(12) = '1' then
                            v_q := '0' & s_gp(11 downto 3);
                            v_e2 := to_unsigned(512, 10) + v_q - shift_right(v_q, 2);
                        else
                            v_e2 := s_gp(12 downto 3);
                        end if;
                        v_sp := shift_left(resize('1' & v_e2(7 downto 0), 12), to_integer(v_e2(9 downto 8)));
                        z_span <= v_sp(11 downto 1);
                    when others => null;
                end case;
            end if;
            -- the fetch: angle register stepped +1 / +255 / +1 from zone clock 31
            if s_zone = '1' and s_zc = 31 then
                z_ang <= z_th(16 downto 7); z_fn <= "001";
            elsif z_fn /= 0 then
                if z_fn = 2 then z_ang <= z_ang + 255; else z_ang <= z_ang + 1; end if;
                if z_fn = 4 then z_fn <= "000"; else z_fn <= z_fn + 1; end if;
            end if;
            v_ang := z_ang;
            if v_ang(8) = '1' then v_n := to_unsigned(256, 9) - resize(v_ang(7 downto 0), 9);
            else v_n := resize(v_ang(7 downto 0), 9); end if;
            r_s_a <= v_n(7 downto 0);
            z_sf1 <= v_n(8) & v_ang(9);            -- (n = 256, negative) of the address just issued
            z_sf2 <= z_sf1;
            z_ff <= z_ff(2 downto 0) & z_fv1;
            z_fv1 <= '0'; if z_fn /= 0 then z_fv1 <= '1'; end if;
            if z_ff(0) = '1' then                  -- a value arrives: shift it in
                if z_sf2(1) = '1' then z_r1 <= to_signed(1023, 11); else z_r1 <= signed(resize(r_s_q(9 downto 0), 11)); end if;
                if z_sf2(0) = '1' then
                    if z_sf2(1) = '1' then z_r1 <= to_signed(-1023, 11); else z_r1 <= -signed(resize(r_s_q(9 downto 0), 11)); end if;
                end if;
                z_r0 <= z_r1;
            end if;
            -- interpolate (after the 2nd and the 4th value): x = (v0 128 + (v1 - v0) fr) >> 3
            z_ip <= z_ip(2 downto 0) & '0';
            if s_zone = '1' and (s_zc = 35 or s_zc = 37) then z_ip(0) <= '1'; end if;
            if z_ip(0) = '1' then z_di <= resize(z_r1 - z_r0, 5); z_rb <= z_r0; end if;
            if z_ip(1) = '1' then
                v_x := shift_left(resize(z_rb, 19), 7) + resize(z_di * signed('0' & z_th(6 downto 0)), 19);
                z_cs <= resize(shift_right(v_x, 3), 16);
                z_sn <= z_cs;                       -- sin first, then cos
            end if;

            -- serial multiply-accumulate slots: slot k (0 .. 56) runs 20 zone clocks from 40 + 20 k.  Phase 0
            -- arms, phase 1 selects the operands, phase 2 loads (clearing, or keeping the sum), 16 iterations,
            -- the result is taken at phase 19.
            if s_zone = '1' and s_zc = 39 then
                z_sl <= (others => '0'); z_ph <= (others => '0'); z_srun <= '1';
            elsif z_srun = '1' then
                if z_ph = 19 then
                    z_ph <= (others => '0');
                    if z_sl = 56 then z_srun <= '0'; end if;
                    z_sl <= z_sl + 1;
                else
                    z_ph <= z_ph + 1;
                end if;
            end if;
            -- texture slots 10 .. 29: texture k = (slot - 10) / 5, sub-slot = (slot - 10) mod 5
            v_k := 0; v_sub := 7;
            case to_integer(z_sl) is
                when 10 => v_k := 0; v_sub := 0;  when 11 => v_k := 0; v_sub := 1;  when 12 => v_k := 0; v_sub := 2;
                when 13 => v_k := 0; v_sub := 3;  when 14 => v_k := 0; v_sub := 4;
                when 15 => v_k := 1; v_sub := 0;  when 16 => v_k := 1; v_sub := 1;  when 17 => v_k := 1; v_sub := 2;
                when 18 => v_k := 1; v_sub := 3;  when 19 => v_k := 1; v_sub := 4;
                when 20 => v_k := 2; v_sub := 0;  when 21 => v_k := 2; v_sub := 1;  when 22 => v_k := 2; v_sub := 2;
                when 23 => v_k := 2; v_sub := 3;  when 24 => v_k := 2; v_sub := 4;
                when 25 => v_k := 3; v_sub := 0;  when 26 => v_k := 3; v_sub := 1;  when 27 => v_k := 3; v_sub := 2;
                when 28 => v_k := 3; v_sub := 3;  when 29 => v_k := 3; v_sub := 4;
                when others => null;
            end case;
            z_pre <= '0';
            if z_srun = '1' and z_ph = 0 then z_pre <= '1'; end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_ng <= '0'; z_keep <= '0';
                if v_sub /= 7 then
                    -- S_k / C_k: angle set (A, B) (ABr, BAr) (B, -A) (BAr, -ABr); pitch set (ABr, BAr) / 2 x 2^k
                    -- (the 2^k goes on the other operand)
                    case v_sub is
                        when 0 => v_ms := resize(s_xs, 16); z_ng <= '1';                       -- x step = -S xs / 2
                        when 1 => v_ms := resize(s_ys, 16);                                    -- line step = C ys / 256
                        when 2 =>                                                              -- creep = fc x rate
                            case v_k is
                                when 0 => v_ms := to_unsigned(1936, 16);
                                when 1 => v_ms := to_unsigned(1456, 16);
                                when 2 => v_ms := to_unsigned(1168, 16);
                                when others => v_ms := to_unsigned(976, 16);
                            end case;
                        when 3 => z_keep <= '1';                                               -- + S x centre
                                  if s_xs = 4 then v_ms := to_unsigned(720, 16); else v_ms := to_unsigned(960, 16); end if;
                        when others => v_ms := to_unsigned(540, 16); z_keep <= '1'; z_ng <= '1';  -- - C x 540
                    end case;
                    if s_pit = '1' and v_sub /= 2 then z_fb <= shift_left(v_ms, v_k); else z_fb <= v_ms; end if;
                    if v_sub = 2 then
                        z_fa <= signed(resize(z_fc, 25));
                    elsif v_sub = 0 or v_sub = 3 then                                          -- S
                        if s_pit = '1' then z_fa <= resize(shift_right(z_ABr, 1), 25);
                        else
                            case v_k is
                                when 0 => z_fa <= resize(z_A, 25);   when 1 => z_fa <= resize(z_ABr, 25);
                                when 2 => z_fa <= resize(z_B, 25);   when others => z_fa <= resize(z_BAr, 25);
                            end case;
                        end if;
                    else                                                                       -- C (negated for k >= 2)
                        if s_pit = '1' then z_fa <= resize(shift_right(z_BAr, 1), 25);
                        else
                            case v_k is
                                when 0 => z_fa <= resize(z_B, 25);   when 1 => z_fa <= resize(z_BAr, 25);
                                when 2 => z_fa <= resize(z_A, 25);   when others => z_fa <= resize(z_ABr, 25);
                            end case;
                            if v_k >= 2 then
                                if v_sub = 1 then z_ng <= '1'; else z_ng <= '0'; end if;
                            end if;
                        end if;
                    end if;
                else
                    case to_integer(z_sl) is
                        when 0  =>                                                             -- the H tent's step
                            case to_integer(s_xs) is
                                when 4      => z_fa <= to_signed(23302, 25);
                                when 3      => z_fa <= to_signed(13107, 25);
                                when others => z_fa <= to_signed(8738, 25);
                            end case;
                            z_fb <= resize(z_span, 16);
                        when 1  => z_fa <= signed(resize(s_cv, 25));   z_fb <= resize(z_span, 16);   -- the V tent's step
                        when 2  => z_fa <= to_signed(4369, 25);                                        -- F = 4369 x mant x 2^oct / 256
                                   z_fb <= shift_left(resize('1' & z_e3(7 downto 0), 16), to_integer(z_e3(10 downto 8)));
                        when 3  => z_fa <= signed(resize(z_fq, 25)); z_ng <= z_sn(15);                 -- A = F sin
                                   v_ab := abs(resize(z_sn, 17)); z_fb <= unsigned(v_ab(15 downto 0));
                        when 4  => z_fa <= signed(resize(z_fq, 25)); z_ng <= z_cs(15);                 -- B = F cos
                                   v_ab := abs(resize(z_cs, 17)); z_fb <= unsigned(v_ab(15 downto 0));
                        when 5  => z_fa <= resize(z_A, 25); z_fb <= to_unsigned(11585, 16);            -- ABr = (A + B) / rt2
                        when 6  => z_fa <= resize(z_B, 25); z_fb <= to_unsigned(11585, 16); z_keep <= '1';
                        when 7  => z_fa <= resize(z_A, 25); z_fb <= to_unsigned(11585, 16); z_ng <= '1'; -- BAr = (B - A) / rt2
                        when 8  => z_fa <= resize(z_B, 25); z_fb <= to_unsigned(11585, 16); z_keep <= '1';
                        when 9  => z_fa <= signed(resize(z_fq, 25)); z_fb <= resize(s_ys, 16);        -- the fade source
                        when 30 => z_fa <= to_signed(599, 25);     z_fb <= resize(s_g6(12 downto 3), 16); -- camera gain
                        when 31 | 32 => z_fa <= (others => '0'); z_fb <= (others => '0');            -- (idle)
                        when others =>                                                         -- zone colours:
                            z_fa <= signed(resize(r_pa_q(9 downto 0), 25));                    -- A (256 - b) + B b
                            if z_sl(0) = '0' then z_fb <= resize(z_bl8, 16); z_keep <= '1';
                            else z_fb <= to_unsigned(256, 16) - resize(z_bl8, 16); end if;
                    end case;
                end if;
            end if;
            if z_ld = '1' then
                if z_keep = '0' then f_acc <= (others => '0'); end if;
                f_a <= unsigned(resize(z_fa, 32)); f_sub <= z_ng;
                f_b <= z_fb;
            else
                if f_b(0) = '1' then
                    if f_sub = '1' then f_acc <= f_acc - f_a; else f_acc <= f_acc + f_a; end if;
                end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- the Matte ROM address for the next slot (zone-colour slots 33 .. 56: set i5 then set i5 + 1)
            if z_srun = '1' and z_ph = 17 and z_sl >= 32 and z_sl <= 55 then
                v_pi := f_pidx(z_sl);
                if z_sl(0) = '0' then r_pa_a <= z_i5 & s_tnt & v_pi;
                else r_pa_a <= z_i51 & s_tnt & v_pi; end if;
            end if;

            -- results (phase 19)
            z_p0we <= "0000";
            if z_srun = '1' and z_ph = 19 then
                if v_sub /= 7 then
                    case v_sub is
                        when 0 => z_sx(v_k) <= f_acc(20 downto 3);
                        when 1 => z_sy(v_k) <= f_acc(27 downto 10);
                        when 4 => z_p0we(v_k) <= '1';                  -- the line accumulator takes f_acc(19 .. 0)
                        when others => null;
                    end case;
                else
                    case to_integer(z_sl) is
                        when 0  => z_dh <= f_acc(25 downto 14);
                        when 1  => z_dv <= f_acc(28 downto 16);
                        when 2  =>
                            z_fq <= f_acc(24 downto 8);
                            -- the crossfade floor: one pixel of the scan gradient (dV >> 8, x2 for the saltire)
                            if s_abs = '1' then v_a := resize(z_dv(12 downto 7), 14); else v_a := resize(z_dv(12 downto 8), 14); end if;
                            if resize(s_g2(12 downto 5), 14) > v_a then v_a := resize(s_g2(12 downto 5), 14); end if;
                            if v_a = 0 then v_a := to_unsigned(1, 14); end if;
                            z_b8e <= v_a(8 downto 0);
                        when 3  => z_A   <= signed(f_acc(31 downto 14));
                        when 4  => z_B   <= signed(f_acc(31 downto 14));
                        when 6  => z_ABr <= signed(f_acc(31 downto 14));
                        when 8  => z_BAr <= signed(f_acc(31 downto 14));
                        when 9  => z_fv  <= f_acc(29 downto 8);
                        when 30 => z_cgl <= f_acc(19 downto 10);
                        when 31 | 32 => null;
                        when others =>
                            if z_sl >= 33 and z_sl(0) = '0' then z_zc(f_zidx(z_sl)) <= f_acc(17 downto 10); end if;
                    end case;
                end if;
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
    -- Table writer (from zone clock 1200).  One DDA table engine (set-up with a
    -- restoring divider, rewind, write) builds the crossfade table t(u) and the
    -- four shaper tables; then, per channel, the four zone colours (K5 blend of
    -- two Matte sets) and the colour table over the scan level, on one
    -- free-running 11 x 8 multiplier.
    ---------------------------------------------------------------------------
    w_rn <= to_unsigned(255, 9) - resize(z_b8e(7 downto 0), 9) when w_ph = 1 else
            to_unsigned(77, 9) when s_key = '1' else to_unsigned(128, 9);
    w_lo <= to_unsigned(0, 9) when w_ph = 1 or s_key = '0' else to_unsigned(128, 9) - resize(w_a(8 downto 1), 9);
    w_hi <= to_unsigned(128, 9) when w_ph = 1 else to_unsigned(255, 9) when s_key = '0' or w_a(8) = '1' else
            to_unsigned(128, 9) + resize(w_a(8 downto 1), 9);

    p_wr : process(clk)
        variable v_v  : signed(17 downto 0);
        variable v_r  : unsigned(22 downto 0);
        variable v_u  : unsigned(7 downto 0);
        variable v_c  : signed(11 downto 0);
        variable v_j  : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            w_go <= '0';
            if s_zone = '1' and s_zc = 1200 then w_go <= '1'; end if;
            -- the write stage: clamp the value issued last clock to [lo, hi]
            v_v := w_vv;
            if v_v < signed(resize(w_lo, 18)) then v_v := signed(resize(w_lo, 18));
            elsif v_v > signed(resize(w_hi, 18)) then v_v := signed(resize(w_hi, 18)); end if;
            w_wd <= std_logic_vector(v_v(7 downto 0));
            w_wa <= w_va; w_she <= w_vshe; w_te <= w_vte;
            w_vshe <= "0000"; w_vte <= '0';

            -- divider: 17 iterations, quotient bits 16 .. 0
            if d_n /= 0 then
                v_r := shift_left(d_r, 1);
                if d_inj = '1' then v_r(0) := '1'; end if;
                d_inj <= '0';
                if v_r >= resize(d_x, 23) then
                    d_r <= v_r - resize(d_x, 23); d_q <= d_q(15 downto 0) & '1';
                else
                    d_r <= v_r; d_q <= d_q(15 downto 0) & '0';
                end if;
                d_n <= d_n - 1;
            end if;

            if w_go = '1' then
                w_ph <= "001"; w_st <= "00"; w_c <= (others => '0'); w_k <= "00";
            else
                case to_integer(w_ph) is
                    when 1 | 2 =>                           -- DDA tables: 1 = the crossfade table, 2 = shaper k
                        w_c <= w_c + 1;
                        case to_integer(w_st) is
                            when 0 =>                       -- set-up: the divider, the fade
                                if w_c = 0 then
                                    if w_ph = 1 then
                                        d_r <= (others => '0'); d_x <= resize(z_b8e, 22); d_inj <= '1';
                                    else
                                        d_r <= to_unsigned(1024, 23); d_inj <= '0';
                                        if s_pit = '1' then
                                            case to_integer(w_k) is
                                                when 0      => d_x <= shift_right(z_fv, 1); w_f20 <= shift_right(z_fv, 1);
                                                when 1      => d_x <= z_fv; w_f20 <= z_fv;
                                                when 2      => d_x <= shift_left(z_fv, 1); w_f20 <= shift_left(z_fv, 1);
                                                when others => d_x <= shift_left(z_fv, 2); w_f20 <= shift_left(z_fv, 2);
                                            end case;
                                        else
                                            d_x <= z_fv; w_f20 <= z_fv;
                                        end if;
                                    end if;
                                    d_n <= to_unsigned(17, 5); d_q <= (others => '0');
                                elsif w_c = 2 then
                                    if w_f20(21 downto 9) >= 512 then w_a <= (others => '0');
                                    elsif w_f20(21 downto 9) <= 256 then w_a <= to_unsigned(256, 9);
                                    else w_a <= resize(to_unsigned(512, 10) - resize(w_f20(17 downto 9), 10), 9); end if;
                                    if w_f20(21 downto 11) = 0 then w_cap <= '1'; else w_cap <= '0'; end if;
                                elsif w_c = 19 then
                                    w_acc <= (others => '0');
                                    w_st <= "01"; w_c <= (others => '0');
                                    if w_ph = 1 then        -- t(u) = clamp((u + 1/2 - u_on) / b, 0, 1/2): Rb = 2^16 / b
                                        if d_q(16) = '1' then w_ws <= x"FFFF"; else w_ws <= d_q(15 downto 0); end if;
                                    elsif s_key = '1' then  -- Keychain: 2^27 / f
                                        if w_cap = '1' or d_q(16) = '1' then w_ws <= x"FFFF"; else w_ws <= d_q(15 downto 0); end if;
                                    else                    -- Stairs: the fade
                                        w_ws <= resize(w_a, 16);
                                    end if;
                                end if;
                            when 1 =>                       -- rewind (255 clocks): t 255 - b steps of Rb / 2; Keychain 77, Stairs 128
                                if w_c < w_rn then
                                    if w_ph = 1 then w_acc <= w_acc - signed(resize(w_ws(15 downto 1), 26));
                                    else w_acc <= w_acc - signed(resize(w_ws, 26)); end if;
                                end if;
                                if w_c = 254 then w_st <= "10"; w_c <= (others => '0'); end if;
                            when 2 =>                       -- write: t clamp(acc >> 8, 0, 128); shaper clamp(acc >> 8 + 128, lo, hi)
                                if w_ph = 2 then w_vv <= w_acc(25 downto 8) + 128; else w_vv <= w_acc(25 downto 8); end if;
                                w_va <= w_c(7 downto 0);            -- (clamped and written next clock)
                                if w_ph = 1 then
                                    w_vte <= '1';
                                else
                                    case to_integer(w_k) is
                                        when 0      => w_vshe <= "0001";
                                        when 1      => w_vshe <= "0010";
                                        when 2      => w_vshe <= "0100";
                                        when others => w_vshe <= "1000";
                                    end case;
                                end if;
                                w_acc <= w_acc + signed(resize(w_ws, 26));
                                if (w_ph = 1 and w_c(6 downto 0) = 127) or w_c(7 downto 0) = 255 then w_st <= "11"; end if;
                            when others =>                  -- next table
                                w_st <= "00"; w_c <= (others => '0');
                                if w_ph = 1 then
                                    w_ph <= "010"; w_k <= "00";
                                else
                                    w_k <= w_k + 1;
                                    if w_k = 3 then w_ph <= "000"; end if;
                                end if;
                        end case;
                    when others => null;
                end case;
            end if;
        end if;
    end process p_wr;

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

    -- knob decodes read straight from the glide registers (static during the zone)
    z_i5   <= s_g5(12 downto 10);
    z_i51  <= s_g5(12 downto 10) + 1;
    z_bl8  <= s_g5(8 downto 3) & "00" when s_g5(9) = '1' else (others => '0');

    ---------------------------------------------------------------------------
    -- Source side: oscillator (2-D triangle lattice, drifting) or camera luma
    -- ((Y >> 2) - 16), averaged over 8-pixel cells, each cell slewed down the
    -- screen (one pole, 1/16 per line; the state lives in the cell store), then
    -- slewed across by two one-pole filters (1/8 per pixel) and scaled by the
    -- source gain (Osc at twice the camera's).  All three filters decay, so none
    -- needs a resync.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_e : signed(16 downto 0);
        variable v_p : unsigned(17 downto 0);
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

            -- 1: sample
            if s_src = '1' then
                s_lin <= s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lin <= v_l - 16; else s_lin <= (others => '0'); end if;
            end if;
            s_lav   <= data_in.avid;
            s_lc    <= s_wcol(2 downto 0);
            s_lcell <= s_wcol(10 downto 3);
            if data_in.avid = '1' then
                s_wcol  <= s_wcol + 1;
                s_lb_ra <= s_wcol(10 downto 3) + 1;
            else
                s_wcol  <= (others => '0');
                s_lb_ra <= (others => '0');
            end if;
            -- 2: sum eight; at the eighth, take the cell's state (read a cell ahead) for the update
            s_f1 <= '0';
            if s_lav = '0' then
                s_cur <= unsigned(s_lb_q(11 downto 0));
            else
                if s_lc = 0 then s_acc <= resize(s_lin, 11); else s_acc <= s_acc + s_lin; end if;
                if s_lc = 7 then
                    s_sum8  <= s_acc + s_lin;
                    s_old   <= s_cur;
                    s_cur   <= unsigned(s_lb_q(11 downto 0));
                    s_fcell <= s_lcell;
                    s_f1    <= '1';
                end if;
            end if;
            -- 3: state += (cell average * 16 - state) / 16
            v_d := signed(resize(s_sum8 & '0', 13)) - signed(resize(s_old, 13));
            v_d := signed(resize(s_old, 13)) + shift_right(v_d, 4);
            s_new   <= unsigned(v_d(11 downto 0));
            s_wcell <= s_fcell;
            s_f2    <= s_f1;
            -- 4: write
            s_lb_we <= s_f2;
            s_lb_wa <= s_wcell;
            s_lb_wd <= "0000" & std_logic_vector(s_new);

            -- pixel side: the cell stream (the line above's state), two slews across, gain
            s_lv <= unsigned(s_lb_q(11 downto 0));
            v_e  := signed(resize(s_lv & "0000", 17)) - signed(resize(s_h2, 17));
            v_e  := signed(resize(s_h2, 17)) + shift_right(v_e, 3);
            s_h2 <= unsigned(v_e(15 downto 0));
            s_la  <= resize(s_h2(11 downto 8) * z_cgl(9 downto 2), 14);
            s_lbh <= resize(s_h2(15 downto 12) * z_cgl(9 downto 2), 14);
            v_p := resize(s_la, 18) + shift_left(resize(s_lbh, 18), 4);   -- luma x gain
            if s_src = '1' then s_lterm <= resize(v_p(15 downto 3), 13); else s_lterm <= resize(v_p(15 downto 4), 13); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine and beam: the V tent (reflected at the centre line), the four
    -- texture line phases; per pixel the H tent (reflected at the centre
    -- column) and the four texture phases
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 620 then
                e_gv <= (others => '0'); e_vq <= (others => '0');
            elsif g_start = '1' then
                if e_vq < 138240 then
                    e_gv <= e_gv + resize(z_dv, 20);
                elsif e_gv < resize(z_dv, 20) then
                    e_gv <= (others => '0');
                else
                    e_gv <= e_gv - resize(z_dv, 20);
                end if;
                e_vq <= e_vq + resize(s_ys, 19);
            end if;
            for k in 0 to 3 loop
                if z_p0we(k) = '1' then e_L(k) <= f_acc(19 downto 2);         -- the top-left phase, from the MAC
                elsif g_start = '1' then e_L(k) <= e_L(k) + z_sy(k); end if;
            end loop;
            if g_start = '1' then
                l_gv <= e_gv;
                b_gh <= (others => '0');
                for k in 0 to 3 loop b_ph(k) <= e_L(k); end loop;
            elsif g_avid = '1' then
                if resize(g_cnt, 12) < resize(s_half, 12) then
                    b_gh <= b_gh + resize(z_dh, 20);
                elsif b_gh < resize(z_dh, 20) then
                    b_gh <= (others => '0');
                else
                    b_gh <= b_gh - resize(z_dh, 20);
                end if;
                for k in 0 to 3 loop b_ph(k) <= b_ph(k) + z_sx(k); end loop;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_d  : signed(21 downto 0);
        variable v_s  : unsigned(12 downto 0);
        variable v_z, v_n : unsigned(1 downto 0);
        variable v_ta, v_tb : unsigned(7 downto 0);
        variable v_p  : signed(9 downto 0);
        variable v_c  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: the four folded ramps -> the shaper tables; the scan tent sum / difference
            if b_ph(0)(17) = '1' then r_sa0 <= not b_ph(0)(16 downto 9); else r_sa0 <= b_ph(0)(16 downto 9); end if;
            if b_ph(1)(17) = '1' then r_sa1 <= not b_ph(1)(16 downto 9); else r_sa1 <= b_ph(1)(16 downto 9); end if;
            if b_ph(2)(17) = '1' then r_sa2 <= not b_ph(2)(16 downto 9); else r_sa2 <= b_ph(2)(16 downto 9); end if;
            if b_ph(3)(17) = '1' then r_sa3 <= not b_ph(3)(16 downto 9); else r_sa3 <= b_ph(3)(16 downto 9); end if;
            if s_abs = '1' then p2_dif <= signed(resize(b_gh, 22)) - signed(resize(l_gv, 22));
            else p2_dif <= signed(resize(b_gh, 22)) + signed(resize(l_gv, 22)); end if;
            -- 3: the DSG3 output (Average or Absolute); Proc bias + the camera
            if s_abs = '1' then
                v_d := abs(p2_dif);
                p3_sd <= unsigned(v_d(19 downto 7));
            else
                p3_sd <= unsigned(p2_dif(20 downto 8));
            end if;
            p3_bc <= resize(z_bias, 13) + s_lterm;
            -- 4: the scan level; the shaped textures (fabric copies)
            v_s := p3_sd + p3_bc;
            p4_s <= v_s(9 downto 0);
            p4_t0 <= unsigned(r_sq0); p4_t1 <= unsigned(r_sq1); p4_t2 <= unsigned(r_sq2); p4_t3 <= unsigned(r_sq3);
            -- 5: zone, neighbour, distance from the zone centre -> crossfade / colour tables; select the pair
            v_z := p4_s(9 downto 8);
            if p4_s(7) = '1' then
                v_n := v_z + 1; r_t_a <= p4_s(6 downto 0);
            else
                v_n := v_z - 1; r_t_a <= not p4_s(6 downto 0);
            end if;
            p5_z <= v_z;
            case to_integer(v_z) is
                when 0      => v_ta := p4_t0;
                when 1      => v_ta := p4_t1;
                when 2      => v_ta := p4_t2;
                when others => v_ta := p4_t3;
            end case;
            case to_integer(v_n) is
                when 0      => v_tb := p4_t0;
                when 1      => v_tb := p4_t1;
                when 2      => v_tb := p4_t2;
                when others => v_tb := p4_t3;
            end case;
            p5_a <= v_ta; p5_b <= v_tb;
            -- 6: B - A
            p6_d <= signed(resize(p5_b, 9)) - signed(resize(p5_a, 9));
            p6_a <= p5_a; p6_z <= p5_z;
            -- 7: crossfade and colour (fabric copies)
            p7_d <= p6_d; p7_a <= p6_a;
            p7_t <= unsigned(r_t_q); p7_z <= p6_z;
            -- 8: (B - A) x t
            p8_pr <= resize(p7_d * signed('0' & p7_t(7 downto 1)), 18);
            p8_a <= p7_a; p8_z <= p7_z;
            -- 9: the pair
            v_c := signed(resize(p8_a, 12)) + resize(shift_right(p8_pr, 7), 12);
            p9_pair <= unsigned(v_c(7 downto 0));
            p9_z <= p8_z;
            -- 10, 11: (delay)
            p10_pair <= p9_pair; p10_z <= p9_z;
            p11_pair <= p10_pair; p11_z <= p10_z;
            -- 12: the scanner's mono output
            p12_mono <= p11_pair;
            case to_integer(p11_z) is                   -- the Matte colour of the zone (SMX3 pick)
                when 0      => p12_col <= std_logic_vector(z_zc(0)) & "00" & std_logic_vector(z_zc(1)) & "00" & std_logic_vector(z_zc(2)) & "00";
                when 1      => p12_col <= std_logic_vector(z_zc(3)) & "00" & std_logic_vector(z_zc(4)) & "00" & std_logic_vector(z_zc(5)) & "00";
                when 2      => p12_col <= std_logic_vector(z_zc(6)) & "00" & std_logic_vector(z_zc(7)) & "00" & std_logic_vector(z_zc(8)) & "00";
                when others => p12_col <= std_logic_vector(z_zc(9)) & "00" & std_logic_vector(z_zc(10)) & "00" & std_logic_vector(z_zc(11)) & "00";
            end case;
            -- 13: the colour x the mono signal
            p13_py <= resize((unsigned(p12_col(29 downto 20)) - 64) * p12_mono(7 downto 1), 18);
            p13_pu <= resize((signed(resize(unsigned(p12_col(19 downto 10)), 11)) - 512) * signed('0' & p12_mono(7 downto 5)), 16);
            p13_pv <= resize((signed(resize(unsigned(p12_col(9 downto 0)), 11)) - 512) * signed('0' & p12_mono(7 downto 5)), 16);
            -- 14: offsets; black off the generated avid
            if s_gav_sr(11) = '0' then
                s_y14 <= C_KY; s_u14 <= C_KU; s_v14 <= C_KV;
            else
                s_y14 <= 64 + p13_py(16 downto 7);
                v_c := resize(shift_right(p13_pu, 3), 12) + 512; s_u14 <= unsigned(v_c(9 downto 0));
                v_c := resize(shift_right(p13_pv, 3), 12) + 512; s_v14 <= unsigned(v_c(9 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(11);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output filter stores
    ---------------------------------------------------------------------------
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            s_dp <= s_dp + 1; s_dr <= s_dr + 1;
            s_qa <= da(to_integer(s_dr));
            da(to_integer(s_dp)) <= "00" & std_logic_vector(s_y14) & std_logic_vector(s_u14) & std_logic_vector(s_v14);
            s_qb <= db(to_integer(s_dr));
            db(to_integer(s_dp)) <= "00" & std_logic_vector(C_KY) & std_logic_vector(s_u16) & std_logic_vector(s_v16);
            s_fa <= s_qa; s_fb <= s_qb;                 -- = the box inputs 8 pixels back
        end if;
    end process p_dl;



    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the HD HDMI bar fix), Y through [1 2 1]/4 one
    -- pixel ahead of the chroma centre (the hatching stays sharp), luma 13/16,
    -- and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_ly : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to black (the off-avid input of both boxes) once
            -- 8 blank samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 15 sums, stage 16 output
            if s_bah = "00000000" then
                s_bau <= C_KU & "000"; s_bav <= C_KV & "000";
            else
                s_bau <= s_bau + s_u14 - unsigned(s_fa(19 downto 10));
                s_bav <= s_bav + s_v14 - unsigned(s_fa(9 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(13) = '1' then
                v_sc := resize(s_bau, 14) + s_baud; s_u16 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v16 <= v_sc(13 downto 4);
            else
                s_u16 <= C_KU; s_v16 <= C_KV;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(13);
            -- box B: stage 17 sums, stage 18 output
            if s_bbh = "00000000" then
                s_bbu <= C_KU & "000"; s_bbv <= C_KV & "000";
            else
                s_bbu <= s_bbu + s_u16 - unsigned(s_fb(19 downto 10));
                s_bbv <= s_bbv + s_v16 - unsigned(s_fb(9 downto 0));
            end if;
            s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bbu, 14) + s_bbud; s_u18 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v18 <= v_sc(13 downto 4);
            -- luma: [1 2 1] of the box A input 10 / 11 / 12 pixels back (the chroma's centre is 12 back)
            s_l1 <= unsigned(s_fa(29 downto 20)); s_l2 <= s_l1; s_l3 <= s_l2;
            v_ly := resize(s_l1, 12) + resize(s_l3, 12) + shift_left(resize(s_l2, 12), 1);
            s_y18 <= v_ly(11 downto 2);
            -- 19: luma 13/16
            if s_y18 > 64 then v_dl := s_y18 - 64; else v_dl := (others => '0'); end if;
            s_y19 <= s_y18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u19 <= s_u18; s_v19 <= s_v18;
            -- 20: (a plain stage: the latency stays C_LAT)
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
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

end architecture hachure;
