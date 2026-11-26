-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Casement v0.1.0 -- Onevolt: three sliding, tiling windows with priority.
--
-- The rack (LZX): Scrolls (two ramps, Waveform = Mirror, scaled by Tiles) -> three Procs (a moving offset per
-- layer, from Scrolls' LFOs) -> [S7: Angles, 45 / 135 degree ramps] -> Stacker (three window keys, Size per
-- channel, Ratio on all three; S8: DSG3 parabolas + Keychain thresholds = discs) -> Switcher (priority: layer 1
-- over 2 over 3 over the ground; S10: SMX3, the raw keys summed) with Matte colours (K5) striped by Stairs
-- (S9: each layer's own Proc'd ramp folded into luma bars: layer 1 across, layer 2 down, layer 3 along the
-- diagonal sum, so the bars tile, mirror and slide with the windows; or flat Matte), the camera (TBC2) keyed
-- over layer 3's striped Matte by its own luma in an FKG3 (K6 = the key gain), and an Angles colour field as
-- the ground.
--
--   K1  Size 1     Stacker channel 1 (the top window): a dot .. larger than the cell
--   K2  Size 2     channel 2 (the frame)
--   K3  Size 3     channel 3 (the camera's window)
--   K4  Ratio      all three: wide slots .. squares .. tall slots (full-width bars / full-height columns at the ends)
--   K5  Colour     Matte: eight designed sets of three colours plus ground, blended
--   K6  Camera     FKG3 key gain: 0 = Matte C only (the black-input picture) .. 100 % = the camera at full level
--   S7  Ramps      Straight / Angles (45 degrees: rhombi, the tiling turns diagonal)
--   S8  Keys       Stacker (windows) / DSG3 + Keychain (discs and rings)
--   S9  Fills      Stairs (bars in each layer's luma, its Matte hue kept) / Matte (flat)
--   S10 Mixer      Switcher (priority) / SMX3 (the raw keys added)
--   S11 Source     Video / Osc
--   P12 TILES      Scrolls ramp scale: 0 % = one great set of windows .. 50 % = 4 cells across .. 100 % = 12
--
-- Engine: vblank-zone sequencer (one 30-bit serial multiplier, 37 slots, shared knob glide, exp2 ROM, Matte set
-- ROM); per pixel two folded phase accumulators, six window compares, six parabola-ROM reads, three Stairs
-- folds (layer 2's bars coarsen to half amplitude, then stop, as their half-cycle nears four field rows); the source
-- (camera or Osc) is sampled three registers deep for the key and five deep for the colour.  Generated avid; one latency (C_LAT 20); output
-- conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16) before the blanking gate.  Virtual 1080-line coordinates.  Colours in the hardware convention
-- (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.casement_pkg.all;
use work.all;

architecture casement of program_top is
    constant C_LAT  : integer := 20;   -- = processing_delay_clks
    constant C_SNAP : boolean := false; -- sim-only copies set this: glides jump straight to the knob

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
    -- triangle LFO: 16-bit phase -> -32767 .. 32767
    function f_tw(p : unsigned(15 downto 0)) return signed is
        variable v : unsigned(14 downto 0);
    begin
        if p(15) = '1' then v := not p(14 downto 0); else v := p(14 downto 0); end if;
        return signed(resize(v, 17) & '0') - to_signed(32767, 18);
    end function;
    -- colour item (0 .. 11) -> ROM low nibble (colour * 4 + channel)
    function f_cit(i : unsigned(3 downto 0)) return unsigned is
    begin
        case to_integer(i) is
            when 0 => return x"0"; when 1 => return x"1"; when 2  => return x"2";
            when 3 => return x"4"; when 4 => return x"5"; when 5  => return x"6";
            when 6 => return x"8"; when 7 => return x"9"; when 8  => return x"A";
            when 9 => return x"C"; when 10 => return x"D"; when others => return x"E";
        end case;
    end function;
    -- one-hot 2^k (k = 0 .. 15)
    function f_pow2(k : integer) return unsigned is
        variable r : unsigned(15 downto 0);
    begin
        r := (others => '0');
        r(k) := '1';
        return r;
    end function;
    -- |u| by ones' complement (u in -4095 .. 4095; a negative u reads one less), and its top 8 bits (the
    -- square-table address, 1/256 cell)
    function f_abs12(u : signed(12 downto 0)) return unsigned is
    begin
        if u(12) = '1' then return unsigned(not u(11 downto 0)); end if;
        return unsigned(u(11 downto 0));
    end function;
    function f_ix(u : signed(12 downto 0)) return unsigned is
    begin
        if u(12) = '1' then return unsigned(not u(11 downto 4)); end if;
        return unsigned(u(11 downto 4));
    end function;
    -- a compositor term: the fill if its gate is open
    function f_gate(f : signed(11 downto 0); g : std_logic) return signed is
    begin
        if g = '0' then return to_signed(0, 14); end if;
        return resize(f, 14);
    end function;
    -- d x alpha (alpha = 0 .. 8, in eighths): 8 d, or (4 a2 + 2 a1 + a0) d
    function f_p8(d : signed(11 downto 0); a : unsigned(3 downto 0)) return signed is
        variable r : signed(15 downto 0);
    begin
        if a(3) = '1' then return shift_left(resize(d, 16), 3); end if;
        r := (others => '0');
        if a(2) = '1' then r := r + shift_left(resize(d, 16), 2); end if;
        if a(1) = '1' then r := r + shift_left(resize(d, 16), 1); end if;
        if a(0) = '1' then r := r + resize(d, 16); end if;
        return r;
    end function;

    -- a Stairs bar: the fold v (0 .. 127) about its middle, (v - 64) x 4 (h: x 2, layer 2's coarse bars)
    function f_bar(v : unsigned(6 downto 0); h : std_logic) return signed is
        variable t : unsigned(6 downto 0);
    begin
        t := v;
        t(6) := not v(6);
        if h = '1' then return shift_left(resize(signed(t), 12), 1); end if;
        return shift_left(resize(signed(t), 12), 2);
    end function;
    -- the fold of an 8-bit slice (a Stairs tap); 64 (= no bar) when the bars are off
    function f_fold(v : unsigned(7 downto 0); off : std_logic) return unsigned is
    begin
        if off = '1' then return to_unsigned(64, 7); end if;
        if v(7) = '1' then return not v(6 downto 0); end if;
        return v(6 downto 0);
    end function;

    -- round-key square tables (the DSG3 parabolas), one ROM per layer and axis
    signal q_a1, q_a2, q_a3, q_b1, q_b2, q_b3 : unsigned(11 downto 0) := (others => '0');
    signal ra_a1, ra_a2, ra_a3, ra_b1, ra_b2, ra_b3 : unsigned(7 downto 0) := (others => '0');
    -- ROMs (registered address and data)
    signal z_ra  : unsigned(7 downto 0) := (others => '0');
    signal r_e_q : unsigned(15 downto 0) := (others => '0');
    signal z_ca  : unsigned(6 downto 0) := (others => '0');
    signal r_c_q : unsigned(15 downto 0) := (others => '0');

    -- output filter stores (as the shell)
    type t_dl is array (0 to 15) of std_logic_vector(31 downto 0);
    signal da, db : t_dl := (others => (others => '0'));
    attribute no_rw_check : boolean;
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

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(11 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(560 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(520 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(880 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(293 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(680 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_ang, s_rnd, s_flt, s_smx, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- LFO phases (Scrolls motion paths and the size breath), advanced once per field
    signal z_po, z_p2x, z_p2y, z_p3 : unsigned(15 downto 0) := (others => '0');

    -- per-field temporaries
    signal z_tw  : signed(17 downto 0) := (others => '0');
    signal z_pc, z_pq : unsigned(15 downto 0);
    signal z_wa  : std_logic := '0';
    signal z_x12, z_x2 : unsigned(11 downto 0) := (others => '0');
    signal z_e12, z_n12 : unsigned(14 downto 0) := (others => '0');
    signal z_n   : unsigned(2 downto 0) := (others => '0');
    signal z_f   : unsigned(12 downto 0) := (others => '0');
    signal z_f2  : unsigned(12 downto 0) := (others => '0');
    signal z_fs  : unsigned(13 downto 0) := (others => '0');
    signal z_w1, z_w2, z_w3 : unsigned(11 downto 0) := (others => '0');
    signal z_ep  : unsigned(14 downto 0) := (others => '0');
    signal z_dl  : unsigned(12 downto 0) := (others => '0');      -- delta: cell per virtual px, 2^-19
    signal z_gi  : unsigned(2 downto 0) := (others => '0');                  -- gap-rule loop
    signal z_gph : unsigned(1 downto 0) := (others => '0');
    signal z_grn : std_logic := '0';
    signal z_gcc : unsigned(12 downto 0) := (others => '0');
    signal z_gw  : unsigned(12 downto 0) := (others => '0');
    signal z_gd0, z_gd1 : signed(13 downto 0) := (others => '0');
    signal z_shw, z_shn : unsigned(3 downto 0) := (others => '0');            -- 2^e, 2^-n scales
    signal z_qa, z_qb : unsigned(15 downto 0) := to_unsigned(1024, 16);       -- Ratio q (Q10)
    signal z_eq  : unsigned(15 downto 0) := (others => '0');
    signal z_cq  : unsigned(15 downto 0) := (others => '0');
    signal z_c0, z_c0h : unsigned(9 downto 0) := (others => '0');
    signal z_cd  : signed(10 downto 0) := (others => '0');
    signal z_vc  : signed(11 downto 0) := (others => '0');
    signal z_k6  : unsigned(9 downto 0) := (others => '0');

    -- per-field values used by the pixel path
    signal c_ca1, c_cb1, c_ca2, c_cb2, c_ca3, c_cb3 : unsigned(11 downto 0) := to_unsigned(2048, 12);
    signal c_wa1, c_wb1, c_wa2, c_wb2, c_wa3, c_wb3 : unsigned(12 downto 0) := (others => '0');
    signal c_t1, c_t2, c_t3 : unsigned(11 downto 0) := (others => '0');
    signal c_ta0, c_tb0 : unsigned(19 downto 0) := (others => '0');
    signal c_dax, c_day, c_dbx, c_dby : unsigned(19 downto 0) := (others => '0');
    signal c_gap : unsigned(7 downto 0) := (others => '0');
    signal c_ay, c_au, c_av, c_by, c_bu, c_bv, c_cy, c_cu, c_cv, c_gy, c_gu, c_gv : unsigned(9 downto 0) :=
        to_unsigned(512, 10);
    signal c_km  : unsigned(3 downto 0) := (others => '0');       -- camera key gain: mantissa, shift
    signal c_ksh : unsigned(3 downto 0) := to_unsigned(15, 4);

    -- field serial multiply: acc = a x b (mod 2^32)
    signal f_a   : unsigned(29 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(29 downto 0) := (others => '0');
    signal z_fa  : signed(17 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(5 downto 0) := (others => '0');
    signal z_tick : unsigned(4 downto 0) := (others => '0');
    signal z_tgo  : std_logic := '0';
    signal z_f12, z_f13, z_f15, z_f16, z_f20, z_f21, z_f22 : std_logic := '0';   -- tick flags (registered)
    signal z_wcol, z_wtw : std_logic := '0';                                       -- slot windows (registered)
    signal z_pre, z_ld : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line accumulators and the beam
    signal l_ta, l_tb : unsigned(19 downto 0) := (others => '0');
    signal b_ta, b_tb : unsigned(19 downto 0) := (others => '0');
    signal s_gst : std_logic_vector(7 downto 0) := (others => '0');   -- s_gst(k) = g_start delayed k + 1
    signal p9_gu : signed(21 downto 0) := (others => '0');
    signal c_gxu : signed(11 downto 0) := (others => '0');

    -- source chain: luma for the key from stage 3, the colour from stage 6
    signal c3_yd : unsigned(10 downto 0) := (others => '0');
    signal c4_pl, c4_ph : unsigned(12 downto 0) := (others => '0');
    signal c5_p  : unsigned(14 downto 0) := (others => '0');
    signal c6_al : unsigned(3 downto 0) := (others => '0');
    signal c6_y, c6_u, c6_v : unsigned(10 downto 0) := (others => '0');
    signal c7_al : unsigned(3 downto 0) := (others => '0');
    signal c7_dy, c7_du, c7_dv : signed(11 downto 0) := (others => '0');
    signal c8_py, c8_pu, c8_pv : signed(15 downto 0) := (others => '0');

    -- pixel path
    signal p2_ma, p2_mb : unsigned(11 downto 0) := (others => '0');
    signal p3_ua1, p3_ub1, p3_ua2, p3_ub2, p3_ua3, p3_ub3 : signed(12 downto 0) := (others => '0');
    signal p4_aa1, p4_ab1, p4_aa2, p4_ab2, p4_aa3, p4_ab3 : unsigned(11 downto 0) := (others => '0');
    signal p5_r  : std_logic_vector(5 downto 0) := (others => '0');
    signal p6_rk, p7_rk : std_logic_vector(2 downto 0) := (others => '0');
    signal p6_a1, p6_a2, p6_a3, p6_b1, p6_b2, p6_b3 : unsigned(11 downto 0) := (others => '0');
    signal p7_s1, p7_s2, p7_s3 : unsigned(12 downto 0) := (others => '0');
    signal p8_k  : std_logic_vector(2 downto 0) := (others => '0');
    signal p9_k  : std_logic_vector(2 downto 0) := (others => '0');
    signal p9_g  : std_logic_vector(2 downto 0) := (others => '0');
    signal p9_g0 : std_logic := '0';
    signal p9_cy, p9_cu, p9_cv : signed(11 downto 0) := (others => '0');
    -- Stairs bars: the folds (stage 5), carried to stage 8; layers 1 and 2's striped luma at stage 9, layer 3's
    -- (the FKG3 background) from stage 6
    signal p5_v1, p5_v2, p5_v3, p6_v1, p6_v2, p7_v1, p7_v2, p8_v1, p8_v2 : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal p6_c3, p7_c3, p8_c3 : signed(11 downto 0) := (others => '0');
    signal p9_ay, p9_by : signed(11 downto 0) := (others => '0');
    signal p10_sy, p10_su, p10_sv, p10_ty, p10_tu, p10_tv : signed(13 downto 0) := (others => '0');
    signal p11_y, p11_u, p11_v : signed(12 downto 0) := (others => '0');
    signal p12_y, p12_u, p12_v : signed(12 downto 0) := (others => '0');
    signal p13_y, p13_u, p13_v : unsigned(9 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
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
    -- Memories: the DSG3 parabola ROMs (one per layer and axis), exp2, Matte sets
    ---------------------------------------------------------------------------
    p_sqa1 : process(clk) begin if rising_edge(clk) then q_a1 <= C_CM_SQ(to_integer(ra_a1)); end if; end process;
    p_sqa2 : process(clk) begin if rising_edge(clk) then q_a2 <= C_CM_SQ(to_integer(ra_a2)); end if; end process;
    p_sqa3 : process(clk) begin if rising_edge(clk) then q_a3 <= C_CM_SQ(to_integer(ra_a3)); end if; end process;
    p_sqb1 : process(clk) begin if rising_edge(clk) then q_b1 <= C_CM_SQ(to_integer(ra_b1)); end if; end process;
    p_sqb2 : process(clk) begin if rising_edge(clk) then q_b2 <= C_CM_SQ(to_integer(ra_b2)); end if; end process;
    p_sqb3 : process(clk) begin if rising_edge(clk) then q_b3 <= C_CM_SQ(to_integer(ra_b3)); end if; end process;

    z_pc <= z_po + 16384;
    z_pq <= z_p3 + 16384;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_e_q <= C_CM_EXP(to_integer(z_ra));
            r_c_q <= C_CM_COL(to_integer(z_ca));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s   : signed(17 downto 0);
        variable v_u   : unsigned(22 downto 0);
        variable v_e   : unsigned(15 downto 0);
        variable v_c   : signed(11 downto 0);
        variable v_k   : integer range 0 to 63;
        variable v_w : unsigned(12 downto 0);
        variable v_x : unsigned(12 downto 0);
        variable v_o : signed(17 downto 0);
        variable v_ph : unsigned(15 downto 0);
        variable v_b  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            z_eq  <= r_e_q;
            z_cq  <= r_c_q;

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
            elsif s_zc /= 4095 then
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
                s_ang <= registers_in(6)(0);
                s_rnd <= registers_in(6)(1);
                s_flt <= registers_in(6)(2);
                s_smx <= registers_in(6)(3);
                s_src <= registers_in(6)(4);
                s_xs  <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                                   c_gxu <= to_signed(2 * C_CM_GXU, 12);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                                   c_gxu <= to_signed((3 * C_CM_GXU) / 2, 12);
                    when others => s_oxs <= to_unsigned(102, 9);
                                   c_gxu <= to_signed(C_CM_GXU, 12);
                end case;
                -- Scrolls LFOs: corner path 11 s (two triangles in quadrature), ping-pong 17 s / 23 s, drift 41 s
                -- (its quadrature is the size breath)
                z_po  <= z_po + 99;  z_p2x <= z_p2x + 64; z_p2y <= z_p2y + 47;
                z_p3  <= z_p3 + 27;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= f_oys(s_tid);

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
                        if C_SNAP then
                            null;
                        elsif s_gd > 32 or s_gd < -32 or (z_gc /= 5 and (s_gd > 16 or s_gd < -16)) then
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

            -- serial multiply slots: a 24-clock tick runs from zc 38; slot k pre-muxes its operands at tick 1
            -- (zc 39 + 24 k), loads at tick 2, and its result is stored at tick 20 (zc 58 + 24 k), when
            -- z_sel = k + 1.  37 slots (0 .. 36).
            if s_zone = '1' and s_zc = 37 then
                z_tgo <= '1';
            elsif s_zone = '0' or (z_tick = 23 and z_sel = 37) then
                z_tgo <= '0';
            end if;
            if s_zone = '1' and s_zc = 37 then
                z_tick <= (others => '0');
            elsif z_tgo = '1' then
                if z_tick = 23 then z_tick <= (others => '0'); else z_tick <= z_tick + 1; end if;
            end if;
            z_pre <= '0';
            if z_tgo = '1' and z_tick = 0 and z_sel /= 37 then z_pre <= '1'; end if;
            -- the tick and window decodes, one clock ahead (each action then waits on a single flag)
            if z_tgo = '1' and z_tick = 11 then z_f12 <= '1'; else z_f12 <= '0'; end if;
            if z_tgo = '1' and z_tick = 12 then z_f13 <= '1'; else z_f13 <= '0'; end if;
            if z_tgo = '1' and z_tick = 14 then z_f15 <= '1'; else z_f15 <= '0'; end if;
            if z_tgo = '1' and z_tick = 15 then z_f16 <= '1'; else z_f16 <= '0'; end if;
            if z_tgo = '1' and z_tick = 19 then z_f20 <= '1'; else z_f20 <= '0'; end if;
            if z_tgo = '1' and z_tick = 20 then z_f21 <= '1'; else z_f21 <= '0'; end if;
            if z_tgo = '1' and z_tick = 21 then z_f22 <= '1'; else z_f22 <= '0'; end if;
            if z_sel >= 25 and z_sel <= 36 then z_wcol <= '1'; else z_wcol <= '0'; end if;
            if z_sel >= 1 and z_sel <= 6 then z_wtw <= '1'; else z_wtw <= '0'; end if;
            z_ld <= z_pre;
            if s_zone = '1' and s_zc = 37 then
                z_sel <= (others => '0');
            elsif z_ld = '1' then
                z_sel <= z_sel + 1;
            end if;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_signed(203, 18);                    z_fb <= resize(s_gp, 16);
                    when 1  => z_fa <= z_tw;                                  z_fb <= to_unsigned(287, 16);     -- 0.035 cell
                    when 2  => z_fa <= z_tw;                                  z_fb <= to_unsigned(410, 16);     -- 0.05
                    when 3  => z_fa <= z_tw;                                  z_fb <= to_unsigned(819, 16);     -- 0.10
                    when 4  => z_fa <= z_tw;                                  z_fb <= to_unsigned(655, 16);     -- 0.08
                    when 5  => z_fa <= z_tw;                                  z_fb <= to_unsigned(410, 16);     -- 0.05
                    when 6  => z_fa <= z_tw;                                  z_fb <= to_unsigned(328, 16);     -- 0.04
                    when 7  => z_fa <= signed(resize(z_x12, 18));             z_fb <= resize(z_x12, 16);
                    when 8  => z_fa <= signed(resize(z_eq, 18));              z_fb <= to_unsigned(5825, 16);
                    when 9  => z_fa <= signed(resize(z_dl, 18));              z_fb <= resize(s_ys, 16);
                    when 10 => z_fa <= signed(resize(z_dl, 18));              -- x0 = the half width, virtual px
                               if s_xs = 4 then z_fb <= to_unsigned(720, 16); else z_fb <= to_unsigned(960, 16); end if;
                    when 11 => z_fa <= signed(resize(z_dl, 18));              z_fb <= to_unsigned(540, 16);
                    when 12 => z_fa <= signed(resize(z_eq, 18));              z_fb <= f_pow2(to_integer(z_shw));
                    when 13 => z_fa <= signed(resize(z_eq, 18));              z_fb <= f_pow2(to_integer(z_shn));
                    when 14 => z_fa <= signed(resize(z_w1, 18));              z_fb <= z_qa;
                    when 15 => z_fa <= signed(resize(z_w2, 18));              z_fb <= z_qa;
                    when 16 => z_fa <= signed(resize(z_w3, 18));              z_fb <= z_qa;
                    when 17 => z_fa <= signed(resize(z_w1, 18));              z_fb <= z_qb;
                    when 18 => z_fa <= signed(resize(z_w2, 18));              z_fb <= z_qb;
                    when 19 => z_fa <= signed(resize(z_w3, 18));              z_fb <= z_qb;
                    when 20 => z_fa <= signed(resize(z_w1, 18));              z_fb <= resize(z_w1, 16);
                    when 21 => z_fa <= signed(resize(z_w2, 18));              z_fb <= resize(z_w2, 16);
                    when 22 => z_fa <= signed(resize(z_w3, 18));              z_fb <= resize(z_w3, 16);
                    when 23 => z_fa <= signed(resize(z_f, 18));               z_fb <= resize(z_f, 16);
                    when 24 => z_fa <= signed(resize(z_f2, 18));              z_fb <= to_unsigned(24576, 16) - shift_left(resize(z_f, 16), 1);
                    when others => z_fa <= resize(z_cd, 18);                 z_fb <= resize(z_fs, 16);  -- colours
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= unsigned(resize(z_fa, 30));
                f_b <= z_fb;
                z_c0h <= z_c0;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(28 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- just-in-time operands at tick 22 of the slot before (and zc 35): the Scrolls LFO values (one
            -- triangle), the exp2 reads for Tiles and Ratio
            case to_integer(z_sel) is
                when 1      => v_ph := z_po;
                when 2      => v_ph := z_pc;
                when 3      => v_ph := z_p2x;
                when 4      => v_ph := z_p2y;
                when 5 | 6  => v_ph := z_p3;
                when others => v_ph := z_pq;
            end case;
            if (z_f22 = '1' and z_wtw = '1') or (s_zone = '1' and s_zc = 35) then
                z_tw <= f_tw(v_ph);
            end if;
            if z_f22 = '1' then
                if z_sel = 8 then z_ra <= z_ep(12 downto 5); end if;                           -- 2^frac (Tiles)
                if z_sel = 12 then z_ra <= z_e12(11 downto 4); end if;                         -- 2^e (fraction)
                if z_sel = 13 then z_ra <= to_unsigned(0, 8) - z_n12(11 downto 4); end if;     -- 2^(1 - frac n)
            end if;
            -- Matte set reads for the colour slots (25 .. 36): during slot k - 1 (z_sel = k) read item
            -- k - 25 of set n (tick 12) and of set n + 1 (tick 13); the difference is ready at tick 16
            if z_wcol = '1' then
                if z_f12 = '1' then
                    z_ca <= z_n & f_cit(resize(z_sel - 25, 4));
                elsif z_f13 = '1' then
                    z_ca <= (z_n + 1) & f_cit(resize(z_sel - 25, 4));
                elsif z_f15 = '1' then
                    z_c0 <= z_cq(9 downto 0);
                elsif z_f16 = '1' then
                    z_cd <= signed(resize(z_cq(9 downto 0), 11)) - signed(resize(z_c0, 11));
                end if;
            end if;

            -- slot results (tick 20; z_sel = k + 1)
            if z_f20 = '1' then
                v_k := to_integer(z_sel);
                -- a window half-width (Q12, 4096 = always); the gap rule runs after the captures
                if f_acc(29 downto 22) /= 0 then v_w := to_unsigned(4096, 13); else v_w := resize(f_acc(21 downto 10), 13); end if;
                -- an offset product (Q12 cells), and the Proc's centre for the offset slots
                v_o := resize(signed(f_acc(29 downto 16)), 18);
                case v_k is
                    when 2      => v_s := to_signed(900, 18) - v_o;
                    when 3      => v_s := to_signed(1000, 18) - v_o;
                    when 4      => v_s := to_signed(2300, 18) - v_o;
                    when 5      => v_s := to_signed(2400, 18) - v_o;
                    when 6      => v_s := to_signed(2048, 18) - v_o;
                    when others => v_s := to_signed(2048, 18) + v_o;
                end case;
                -- a Matte colour (the lerp; S9's inversion follows at tick 21)
                z_vc <= signed(resize(z_c0h, 12)) + resize(signed(f_acc(23 downto 13)), 12);
                case v_k is
                    when 1  => z_ep <= f_acc(20 downto 6);
                    when 2  => c_cb1 <= unsigned(v_s(11 downto 0));
                    when 3  => c_ca1 <= unsigned(v_s(11 downto 0));
                    when 4  => c_ca2 <= unsigned(v_s(11 downto 0));
                    when 5  => c_cb2 <= unsigned(v_s(11 downto 0));
                    when 6  => c_ca3 <= unsigned(v_s(11 downto 0));
                    when 7  => c_cb3 <= unsigned(v_s(11 downto 0));
                    when 8  => z_x2 <= f_acc(23 downto 12);
                    when 9  =>
                        case to_integer(z_ep(14 downto 13)) is
                            when 0      => z_dl <= resize(f_acc(29 downto 19), 13);
                            when 1      => z_dl <= resize(f_acc(29 downto 18), 13);
                            when 2      => z_dl <= f_acc(29 downto 17);
                            when others => z_dl <= f_acc(28 downto 16);
                        end case;
                    when 10 =>                                      -- the line step (B; S7 also A); the gap threshold
                        c_dby <= resize(f_acc(22 downto 8), 20);
                        if s_ang = '1' then c_day <= resize(f_acc(22 downto 8), 20); else c_day <= (others => '0'); end if;
                        c_gap <= f_acc(21 downto 14);
                    when 11 =>
                        c_ta0 <= to_unsigned(262144, 20) - f_acc(19 downto 0);
                        if s_ang = '1' then c_tb0 <= to_unsigned(262144, 20) + f_acc(19 downto 0);
                        else c_tb0 <= to_unsigned(262144, 20); end if;
                    when 12 =>
                        if s_ang = '1' then c_ta0 <= c_ta0 - f_acc(19 downto 0); end if;
                        c_tb0 <= c_tb0 - f_acc(19 downto 0);
                    when 13 =>                                      -- q of the widened axis (Q10)
                        if f_acc(29 downto 21) /= 0 then v_e := to_unsigned(65535, 16); else v_e := f_acc(20 downto 5); end if;
                        if z_wa = '1' then z_qa <= v_e; else z_qb <= v_e; end if;
                    when 14 =>                                      -- q of the narrowed axis (Q10)
                        if z_wa = '1' then z_qb <= resize(f_acc(29 downto 16), 16); else z_qa <= resize(f_acc(29 downto 16), 16); end if;
                    when 15 => c_wa1 <= v_w;
                    when 16 => c_wa2 <= v_w;
                    when 17 => c_wa3 <= v_w;
                    when 18 => c_wb1 <= v_w;
                    when 19 => c_wb2 <= v_w;
                    when 20 => c_wb3 <= v_w;
                    when 21 => c_t1 <= f_acc(23 downto 12);
                    when 22 => c_t2 <= f_acc(23 downto 12);
                    when 23 => c_t3 <= f_acc(23 downto 12);
                    when 24 => z_f2 <= f_acc(25 downto 13);
                    when 25 => z_fs <= f_acc(26 downto 13);
                    when others => null;
                end case;
            end if;
            -- the Matte colour (tick 21)
            if z_f21 = '1' then
                v_k := to_integer(z_sel);
                v_c := z_vc;
                case v_k is
                    when 26 => c_ay <= unsigned(v_c(9 downto 0)); when 27 => c_au <= unsigned(v_c(9 downto 0));
                    when 28 => c_av <= unsigned(v_c(9 downto 0)); when 29 => c_by <= unsigned(v_c(9 downto 0));
                    when 30 => c_bu <= unsigned(v_c(9 downto 0)); when 31 => c_bv <= unsigned(v_c(9 downto 0));
                    when 32 => c_cy <= unsigned(v_c(9 downto 0)); when 33 => c_cu <= unsigned(v_c(9 downto 0));
                    when 34 => c_cv <= unsigned(v_c(9 downto 0)); when 35 => c_gy <= unsigned(v_c(9 downto 0));
                    when 36 => c_gu <= unsigned(v_c(9 downto 0)); when 37 => c_gv <= unsigned(v_c(9 downto 0));
                    when others => null;
                end case;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- knob decodes (after the glide)
                    when 32 =>
                        if s_g4 < 4092 then
                            z_wa <= '1'; v_e := to_unsigned(4092, 16) - resize(s_g4, 16);
                        else
                            z_wa <= '0'; v_e := resize(s_g4, 16) - 4092;
                        end if;
                        if v_e > 4095 then z_x12 <= to_unsigned(4095, 12); else z_x12 <= v_e(11 downto 0); end if;
                        v_e := shift_left(resize(s_g5, 16), 3) - resize(s_g5, 16);
                        z_n <= v_e(15 downto 13); z_f <= v_e(12 downto 0);
                        z_k6 <= s_g6(12 downto 3);
                    when 34 =>                                      -- the camera key gain
                        -- K6 ~ m 2^(e - 3), m = 8 .. 15; the key alpha (8ths) = (luma m) >> (19 - e)
                        if z_k6(9) = '1' then    c_km <= z_k6(9 downto 6); c_ksh <= to_unsigned(10, 4);
                        elsif z_k6(8) = '1' then c_km <= z_k6(8 downto 5); c_ksh <= to_unsigned(11, 4);
                        elsif z_k6(7) = '1' then c_km <= z_k6(7 downto 4); c_ksh <= to_unsigned(12, 4);
                        elsif z_k6(6) = '1' then c_km <= z_k6(6 downto 3); c_ksh <= to_unsigned(13, 4);
                        elsif z_k6(5) = '1' then c_km <= z_k6(5 downto 2); c_ksh <= to_unsigned(14, 4);
                        else                     c_km <= z_k6(4 downto 1); c_ksh <= to_unsigned(15, 4); end if;
                    when 36 =>                                      -- Stacker Size (Q12 half-width): 0.02 + 0.5 k + breath
                        v_b := resize(shift_right(z_tw, 10), 12);
                        z_w1 <= unsigned(to_signed(82, 12) + signed(resize(s_g1(12 downto 3) & '0', 12)) + v_b);
                        z_w2 <= unsigned(to_signed(82, 12) + signed(resize(s_g2(12 downto 3) & '0', 12)) + v_b);
                        z_w3 <= unsigned(to_signed(82, 12) + signed(resize(s_g3(12 downto 3) & '0', 12)) + v_b);
                    when 230 =>                                     -- Ratio exponent e = 6 x^2 (Q12)
                        z_e12 <= shift_left(resize(z_x2, 15), 2) + shift_left(resize(z_x2, 15), 1);
                    when 231 =>
                        z_n12 <= resize(z_e12(14 downto 3), 15) + resize(z_e12(14 downto 5), 15) + resize(z_e12(14 downto 7), 15);
                        z_shw <= resize(z_e12(14 downto 12), 4);
                    when 232 =>                                     -- 2^-n = 2^(1 - frac) 2^-(int + 1); frac 0 -> 2 x 32768
                        if z_n12(11 downto 4) = 0 then z_shn <= to_unsigned(11, 4) - resize(z_n12(14 downto 12), 4);
                        else z_shn <= to_unsigned(10, 4) - resize(z_n12(14 downto 12), 4); end if;
                    when 255 =>                                     -- beam steps along the line (S7: Angles)
                        case to_integer(s_xs) is
                            when 4      => v_x := shift_left(z_dl, 1);
                            when 3      => v_x := z_dl + resize(z_dl(12 downto 1), 13);
                            when others => v_x := z_dl;
                        end case;
                        c_dax <= resize(v_x, 20);
                        if s_ang = '1' then c_dbx <= to_unsigned(0, 20) - resize(v_x, 20); else c_dbx <= (others => '0'); end if;
                    when others => null;
                end case;
            end if;

            -- the gap rule (zc 520 .. 543, four clocks per window): a mullion under two field rows at either fold
            -- closes (the half-width grows to the fold; the far side grows by under two rows)
            if s_zone = '1' and s_zc = 519 then
                z_grn <= '1'; z_gi <= (others => '0'); z_gph <= "00";
            elsif z_grn = '1' then
                z_gph <= z_gph + 1;
                case z_gph is
                    when "00" =>
                        case to_integer(z_gi) is
                            when 0      => z_gcc <= resize(c_ca1, 13); z_gw <= c_wa1;
                            when 1      => z_gcc <= resize(c_ca2, 13); z_gw <= c_wa2;
                            when 2      => z_gcc <= resize(c_ca3, 13); z_gw <= c_wa3;
                            when 3      => z_gcc <= resize(c_cb1, 13); z_gw <= c_wb1;
                            when 4      => z_gcc <= resize(c_cb2, 13); z_gw <= c_wb2;
                            when others => z_gcc <= resize(c_cb3, 13); z_gw <= c_wb3;
                        end case;
                    when "01" =>
                        z_gd0 <= signed(resize(z_gcc, 14)) - signed(resize(z_gw, 14));
                        z_gd1 <= to_signed(4096, 14) - signed(resize(z_gcc, 14)) - signed(resize(z_gw, 14));
                    when "10" =>
                        if z_gd0 >= 0 and z_gd0 < signed(resize(c_gap, 14)) then z_gw <= z_gcc + 1;
                        elsif z_gd1 > 0 and z_gd1 < signed(resize(c_gap, 14)) then z_gw <= to_unsigned(4096, 13) - z_gcc; end if;
                    when others =>
                        case to_integer(z_gi) is
                            when 0      => c_wa1 <= z_gw;
                            when 1      => c_wa2 <= z_gw;
                            when 2      => c_wa3 <= z_gw;
                            when 3      => c_wb1 <= z_gw;
                            when 4      => c_wb2 <= z_gw;
                            when others => c_wb3 <= z_gw;
                        end case;
                        if z_gi = 5 then z_grn <= '0'; end if;
                        z_gi <= z_gi + 1;
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
    -- Source side: the Osc (2-D triangle lattice, drifting, fed at twice the
    -- camera's level, its phases as chroma) or the camera (TBC2), sampled and
    -- held through blanking.  The FKG3 soft luma key: alpha = min(1.7 K6 luma,
    -- 1) in eighths, from the luma sampled at stage 3; layer 3's fill = C +
    -- alpha (src - C), with the colour sampled at stage 5 (two pixels on).
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_s  : unsigned(7 downto 0);
        variable v_tx, v_ty : unsigned(6 downto 0);
        variable v_p  : unsigned(14 downto 0);
        variable v_yd : unsigned(10 downto 0);
        variable v_yp : unsigned(11 downto 0);
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
            v_tx := f_tri7(s_ox(15 downto 8)); v_ty := f_tri7(s_oy(15 downto 8));
            v_s := resize(v_tx, 8) + resize(v_ty, 8);
            v_s := v_s - shift_right(v_s, 3);
            if s_src = '1' then
                v_yd := shift_left(resize(v_s, 11), 3);
            elsif unsigned(data_in.y) > 64 then
                v_yd := resize(unsigned(data_in.y) - 64, 11);
            else
                v_yd := (others => '0');
            end if;

            -- the key: stage 3 samples the luma above black (held through blanking), 4 and 5 multiply by the
            -- gain mantissa, 6 shifts and clips
            if data_in.avid = '1' then c3_yd <= v_yd; end if;
            c4_pl <= resize(c3_yd * c_km(1 downto 0), 13);
            c4_ph <= resize(c3_yd * c_km(3 downto 2), 13);
            c5_p  <= resize(c4_pl, 15) + shift_left(resize(c4_ph, 15), 2);
            v_p := shift_right(c5_p, to_integer(c_ksh));
            if v_p > 8 then c6_al <= to_unsigned(8, 4); else c6_al <= v_p(3 downto 0); end if;
            -- the colour: stage 5 samples (luma above black, chroma), 6 pre-scales the luma 16/13 and takes
            -- chroma minus C, 7 luma minus C
            if data_in.avid = '1' then
                c6_y <= v_yd;
                if s_src = '1' then
                    c6_u <= to_unsigned(256, 11) + shift_left(resize(v_tx, 11), 2);
                    c6_v <= to_unsigned(256, 11) + shift_left(resize(v_ty, 11), 2);
                else
                    c6_u <= resize(unsigned(data_in.u), 11); c6_v <= resize(unsigned(data_in.v), 11);
                end if;
            end if;
            v_yp := to_unsigned(64, 12) + resize(c6_y, 12) + resize(c6_y(10 downto 2), 12) - resize(c6_y(10 downto 6), 12);
            if v_yp > 1023 then v_yp := to_unsigned(1023, 12); end if;
            c7_dy <= signed(resize(v_yp, 12)) - p6_c3;
            c7_du <= signed(resize(c6_u, 12)) - signed(resize(c_cu, 12));
            c7_dv <= signed(resize(c6_v, 12)) - signed(resize(c_cv, 12));
            c7_al <= c6_al;
            -- stage 8: (src - C) x alpha;  stage 9: layer 3's fill
            c8_py <= f_p8(c7_dy, c7_al); c8_pu <= f_p8(c7_du, c7_al); c8_pv <= f_p8(c7_dv, c7_al);
            p9_cy <= p8_c3 + resize(shift_right(c8_py, 3), 12);
            p9_cu <= signed(resize(c_cu, 12)) + resize(shift_right(c8_pu, 3), 12);
            p9_cv <= signed(resize(c_cv, 12)) + resize(shift_right(c8_pv, 3), 12);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line accumulators (loaded in the zone, advanced at each active line start):
    -- the two Scrolls phases
    ---------------------------------------------------------------------------
    p_line : process(clk)
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 1100 then
                l_ta <= c_ta0; l_tb <= c_tb0;
            elsif g_start = '1' then
                l_ta <= l_ta + c_day; l_tb <= l_tb + c_dby;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the two Scrolls phases (2^20 per mirror cycle of two cells); a
    -- delayed copy of the beam's control runs the Angles field's U at stage 9
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_ta <= l_ta; b_tb <= l_tb;
            elsif g_avid = '1' then
                b_ta <= b_ta + c_dax; b_tb <= b_tb + c_dbx;
            end if;
            s_gst <= s_gst(6 downto 0) & g_start;
            -- the Angles field's U across the line (stage 9)
            if s_gst(7) = '1' then
                p9_gu <= shift_left(signed(resize(c_gu, 22)), 10);
            elsif s_gav_sr(7) = '1' then
                p9_gu <= p9_gu + resize(c_gxu, 22);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_m   : unsigned(18 downto 0);
        variable v_k   : signed(12 downto 0);
        variable v_n   : unsigned(1 downto 0);
        variable v_y, v_u, v_w : signed(13 downto 0);
        variable v_s9  : unsigned(9 downto 0);
        variable v_sl  : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: the mirror (Scrolls Waveform = Mirror): triangle of each phase, 12 bits per cell
            if b_ta(19) = '1' then v_m := not b_ta(18 downto 0); else v_m := b_ta(18 downto 0); end if;
            p2_ma <= v_m(18 downto 7);
            if b_tb(19) = '1' then v_m := not b_tb(18 downto 0); else v_m := b_tb(18 downto 0); end if;
            p2_mb <= v_m(18 downto 7);
            -- 3: the Procs' offsets (u = m - c)
            p3_ua1 <= signed(resize(p2_ma, 13)) - signed(resize(c_ca1, 13));
            p3_ub1 <= signed(resize(p2_mb, 13)) - signed(resize(c_cb1, 13));
            p3_ua2 <= signed(resize(p2_ma, 13)) - signed(resize(c_ca2, 13));
            p3_ub2 <= signed(resize(p2_mb, 13)) - signed(resize(c_cb2, 13));
            p3_ua3 <= signed(resize(p2_ma, 13)) - signed(resize(c_ca3, 13));
            p3_ub3 <= signed(resize(p2_mb, 13)) - signed(resize(c_cb3, 13));
            -- 4: |u|; square-table addresses
            p4_aa1 <= f_abs12(p3_ua1); p4_ab1 <= f_abs12(p3_ub1);
            p4_aa2 <= f_abs12(p3_ua2); p4_ab2 <= f_abs12(p3_ub2);
            p4_aa3 <= f_abs12(p3_ua3); p4_ab3 <= f_abs12(p3_ub3);
            ra_a1 <= f_ix(p3_ua1); ra_b1 <= f_ix(p3_ub1);
            ra_a2 <= f_ix(p3_ua2); ra_b2 <= f_ix(p3_ub2);
            ra_a3 <= f_ix(p3_ua3); ra_b3 <= f_ix(p3_ub3);
            -- 5: the Stacker's window compares (|u| < half-width); tables (EBR)
            if resize(p4_aa1, 13) < c_wa1 then p5_r(0) <= '1'; else p5_r(0) <= '0'; end if;
            if resize(p4_ab1, 13) < c_wb1 then p5_r(1) <= '1'; else p5_r(1) <= '0'; end if;
            if resize(p4_aa2, 13) < c_wa2 then p5_r(2) <= '1'; else p5_r(2) <= '0'; end if;
            if resize(p4_ab2, 13) < c_wb2 then p5_r(3) <= '1'; else p5_r(3) <= '0'; end if;
            if resize(p4_aa3, 13) < c_wa3 then p5_r(4) <= '1'; else p5_r(4) <= '0'; end if;
            if resize(p4_ab3, 13) < c_wb3 then p5_r(5) <= '1'; else p5_r(5) <= '0'; end if;
            -- 5: the Stairs folds of each layer's own |u| from its window centre (a dark bar there): layer 1
            --    |uA| across (1/8 cell); layer 2 |uB| down (1/16 cell while that is over four field rows per
            --    half-cycle, then 1/8 cell at half amplitude, off once that would be four rows or under: c_gap =
            --    two rows); layer 3 |uA| + |uB| (1/8 cell, diamonds; under S7, where the sum steps twice per row,
            --    off under two rows); S9 Matte: none
            p5_v1 <= f_fold(p4_aa1(8 downto 1), s_flt);
            if c_gap(6) = '1' then v_sl := p4_ab2(8 downto 1); else v_sl := p4_ab2(7 downto 0); end if;
            p5_v2 <= f_fold(v_sl, s_flt or c_gap(7));
            v_s9  := resize(p4_aa3(8 downto 0), 10) + resize(p4_ab3(8 downto 0), 10);
            p5_v3 <= f_fold(v_s9(8 downto 1), s_flt or (s_ang and c_gap(7)));   -- S7: two rows' step per row
            p6_v1 <= p5_v1; p6_v2 <= p5_v2; p7_v1 <= p6_v1; p7_v2 <= p6_v2; p8_v1 <= p7_v1; p8_v2 <= p7_v2;
            -- 6: layer 3's striped Matte C luma (the FKG3 background), carried to stage 8
            p6_c3 <= signed(resize(c_cy, 12)) + f_bar(p5_v3, '0');
            p7_c3 <= p6_c3; p8_c3 <= p7_c3;
            -- 6: fabric copies of the tables; the window keys
            p6_a1 <= q_a1; p6_a2 <= q_a2; p6_a3 <= q_a3;
            p6_b1 <= q_b1; p6_b2 <= q_b2; p6_b3 <= q_b3;
            p6_rk(0) <= p5_r(0) and p5_r(1); p6_rk(1) <= p5_r(2) and p5_r(3); p6_rk(2) <= p5_r(4) and p5_r(5);
            -- 7: the DSG3 average (sum of the two parabolas)
            p7_s1 <= resize(p6_a1, 13) + resize(p6_b1, 13);
            p7_s2 <= resize(p6_a2, 13) + resize(p6_b2, 13);
            p7_s3 <= resize(p6_a3, 13) + resize(p6_b3, 13);
            p7_rk <= p6_rk;
            -- 8: the keys: Keychain thresholds on the DSG3 discs (still clipped by the Ratio's window), or the
            --    Stacker windows; layer 1's DWO3 ripple
            if s_rnd = '1' then
                if p7_s1 < resize(c_t1, 13) and p7_rk(0) = '1' then p8_k(0) <= '1'; else p8_k(0) <= '0'; end if;
                if p7_s2 < resize(c_t2, 13) and p7_rk(1) = '1' then p8_k(1) <= '1'; else p8_k(1) <= '0'; end if;
                if p7_s3 < resize(c_t3, 13) and p7_rk(2) = '1' then p8_k(2) <= '1'; else p8_k(2) <= '0'; end if;
            else
                p8_k <= p7_rk;
            end if;
            -- 9: the gates: Switcher = the priority (one layer, or the ground), SMX3 = every raw key and the
            --    ground; layers 1 and 2's striped luma
            p9_ay <= signed(resize(c_ay, 12)) + f_bar(p8_v1, '0');
            p9_by <= signed(resize(c_by, 12)) + f_bar(p8_v2, c_gap(6));
            p9_k <= p8_k;
            if s_smx = '1' then
                p9_g <= p8_k; p9_g0 <= '1';
            else
                p9_g(0) <= p8_k(0);
                p9_g(1) <= p8_k(1) and not p8_k(0);
                p9_g(2) <= p8_k(2) and not p8_k(1) and not p8_k(0);
                p9_g0 <= not (p8_k(0) or p8_k(1) or p8_k(2));
            end if;
            -- 10: one adder tree for both compositors: the ground (less the black of each SMX3 term) plus
            --     each gated fill (whole for the Switcher, half for SMX3)
            -- base: the ground (Switcher, when no key); twice the ground less the black of each key (SMX3)
            v_n := ("0" & p9_k(0)) + ("0" & p9_k(1)) + ("0" & p9_k(2));
            if s_smx = '1' then
                p10_sy <= shift_left(signed(resize(c_gy, 14)), 1) - signed(resize(v_n & "000000", 14)) + f_gate(p9_ay, p9_g(0));
                p10_su <= shift_left(resize(p9_gu(21 downto 10), 14), 1) - signed(resize(v_n & "000000000", 14)) + f_gate(signed(resize(c_au, 12)), p9_g(0));
                p10_sv <= shift_left(signed(resize(c_gv, 14)), 1) - signed(resize(v_n & "000000000", 14)) + f_gate(signed(resize(c_av, 12)), p9_g(0));
            elsif p9_g0 = '1' then
                p10_sy <= signed(resize(c_gy, 14)); p10_su <= resize(p9_gu(21 downto 10), 14); p10_sv <= signed(resize(c_gv, 14));
            else
                p10_sy <= f_gate(p9_ay, p9_g(0));
                p10_su <= f_gate(signed(resize(c_au, 12)), p9_g(0));
                p10_sv <= f_gate(signed(resize(c_av, 12)), p9_g(0));
            end if;
            p10_ty <= f_gate(p9_by, p9_g(1)) + f_gate(p9_cy, p9_g(2));
            p10_tu <= f_gate(signed(resize(c_bu, 12)), p9_g(1)) + f_gate(p9_cu, p9_g(2));
            p10_tv <= f_gate(signed(resize(c_bv, 12)), p9_g(1)) + f_gate(p9_cv, p9_g(2));
            -- 11: the sum (SMX3: halved)
            v_y := p10_sy + p10_ty; v_u := p10_su + p10_tu; v_w := p10_sv + p10_tv;
            if s_smx = '1' then
                p11_y <= resize(shift_right(v_y, 1), 13); p11_u <= resize(shift_right(v_u, 1), 13); p11_v <= resize(shift_right(v_w, 1), 13);
            else
                p11_y <= resize(v_y, 13); p11_u <= resize(v_u, 13); p11_v <= resize(v_w, 13);
            end if;
            -- 12: the encoder's soft knee on luma above 860
            if p11_y > 860 then v_k := to_signed(860, 13) + shift_right(p11_y - 860, 1); else v_k := p11_y; end if;
            p12_y <= v_k; p12_u <= p11_u; p12_v <= p11_v;
            -- 13: clip (luma 64 .. 940, chroma 32 .. 992)
            if p12_y < 64 then p13_y <= to_unsigned(64, 10);
            elsif p12_y > 940 then p13_y <= to_unsigned(940, 10);
            else p13_y <= unsigned(p12_y(9 downto 0)); end if;
            if p12_u < 32 then p13_u <= to_unsigned(32, 10);
            elsif p12_u > 992 then p13_u <= to_unsigned(992, 10);
            else p13_u <= unsigned(p12_u(9 downto 0)); end if;
            if p12_v < 32 then p13_v <= to_unsigned(32, 10);
            elsif p12_v > 992 then p13_v <= to_unsigned(992, 10);
            else p13_v <= unsigned(p12_v(9 downto 0)); end if;
            -- 14: black off the generated avid
            if s_gav_sr(11) = '0' then
                s_y14 <= C_KY; s_u14 <= C_KU; s_v14 <= C_KV;
            else
                s_y14 <= p13_y; s_u14 <= p13_u; s_v14 <= p13_v;
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
            db(to_integer(s_dp)) <= "00" & std_logic_vector(s_y16) & std_logic_vector(s_u16) & std_logic_vector(s_v16);
            s_fa <= s_qa; s_fb <= s_qb;                 -- = the box inputs 8 pixels back
        end if;
    end process p_dl;


    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the rack's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16, and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to the black (the off-avid input of both boxes) once
            -- 8 blank samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 15 sums, stage 16 output
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KU & "000"; s_bav <= C_KV & "000";
            else
                s_bay <= s_bay + s_y14 - unsigned(s_fa(29 downto 20));
                s_bau <= s_bau + s_u14 - unsigned(s_fa(19 downto 10));
                s_bav <= s_bav + s_v14 - unsigned(s_fa(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(13) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y16 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u16 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v16 <= v_sc(13 downto 4);
            else
                s_y16 <= C_KY; s_u16 <= C_KU; s_v16 <= C_KV;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(13);
            -- box B: stage 17 sums, stage 18 output
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= C_KU & "000"; s_bbv <= C_KV & "000";
            else
                s_bby <= s_bby + s_y16 - unsigned(s_fb(29 downto 20));
                s_bbu <= s_bbu + s_u16 - unsigned(s_fb(19 downto 10));
                s_bbv <= s_bbv + s_v16 - unsigned(s_fb(9 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y18 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u18 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v18 <= v_sc(13 downto 4);
            -- 19: luma 13/16
            if s_y18 > 64 then v_dl := s_y18 - 64; else v_dl := (others => '0'); end if;
            s_y19 <= s_y18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u19 <= s_u18; s_v19 <= s_v18;
            -- 20: delay (keeps the shell's stage count)
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

end architecture casement;
