-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Starburst v0.5.0 -- Heirloom #3, rebuilt from WIN_20221122_23_37_36.
--
-- The photo: a four-fold mirror kaleidoscope (cells 473 x 264 px, about
-- 4.1 per screen height).  Inside each mirror cell a source pattern is
-- colourized through a CMYK cycle: white / lilac / grey / charcoal / black
-- bowties (the big white X, the grey X at the centre, the grey arrows) and
-- yellow / magenta / lavender / cyan / white chevrons, a ring of beads
-- round the small diamonds.  The horizontal band edges are cut into
-- yellow-and-black sawtooth teeth (a horizontal oscillator thrown into the
-- vertical ramp), the grey X is feathered by the vertical oscillator thrown
-- into the horizontal ramp, and a yellow starburst sparks off its arms.
--
-- The program:
--   * the source pattern is an 81 x 49 knot field fitted to the photo
--     (starburst_pkg, split by row parity), folded by H and V triangle
--     ramps (the mirrors);
--   * a per-line engine (kicked at each line start, computing the NEXT
--     line) displaces each knot column vertically by the tooth sawtooth,
--     samples the field there (two rows per clock, lerp) and writes the
--     row into a ping-pong row RAM;
--   * the pixel path displaces u by the feather saw + fur noise (masked to
--     the grey X and the starburst), lerps the row, adds the camera (or
--     Osc) luma, applies the colourizer gain / breathing and looks up the
--     band and the palette colour.
--
--   K1  Cells    mirror cells per screen height, 2 .. 8 (4.1 = the photo)
--   K2  Bands    colourizer gain: 0.5x .. 3x band density
--   K3  Teeth    tooth count on the serrated edges (and the feathers), 0.5x .. 2x
--   K4  Speed    bipolar: teeth crawl / feathers roll / bands breathe; centre still
--   K5  Etch     how hard the picture (Video) or the oscillator (Osc) bends the bands
--   K6  Palette  CMYK / Ember / Ice / Acid / Rose / Mono / Neon / Gold
--   S7  Centre   Grey X / White X at the screen centre
--   S8  Greys    the bowties Grey / Colour
--   S9  Source   Video / Osc
--   S10 Drift    the pattern drifts under the mirrors
--   S11 Mirror   Fold (kaleidoscope) / Tile (ramps, no mirror)
--   P12 Burst    teeth depth, feathers and fur: 0 = calm bands .. 100 % shredded
--
-- Geometry in virtual 1080-line units (round cells at SD, 720p, 1080p).
-- Per-field constants on one serial multiplier in the vblank zone; the
-- line engine has its own serial multiplier for its three per-line products.
-- Generated avid; output conditioning (chroma 9-tap, luma [1 2 1] and
-- 13/16) before the blanking gate.  Latency 36 clocks in every mode.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.starburst_pkg.all;
use work.all;

architecture starburst of program_top is
    constant C_LAT : integer := 36;

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
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- 20-bit mirror phase -> 16-bit cell coordinate (fold) or ramp (tile)
    function f_tri20(p : unsigned(19 downto 0); tile : std_logic) return unsigned is
        variable r : unsigned(18 downto 0);
    begin
        if p(19) = '1' and tile = '0' then r := not p(18 downto 0); else r := p(18 downto 0); end if;
        return r(18 downto 3);
    end function;
    -- fold a signed coordinate (range -65536 .. 131071) back into 0 .. 65535
    function f_fold(s : signed(17 downto 0)) return unsigned is
        variable a : unsigned(15 downto 0);
    begin
        -- bit 16 is set both below 0 and above 65535: invert the low bits there
        a := unsigned(s(15 downto 0));
        if s(16) = '1' then a := not a; end if;
        return a;
    end function;
    -- glide: ease 1/8 of the gap, +-3 hold, single steps when close (d = target - g)
    function f_glide(g : unsigned(9 downto 0); d : signed(10 downto 0)) return unsigned is
        variable r : signed(10 downto 0);
    begin
        if d >= 8 or d <= -8 then
            r := signed('0' & g) + shift_right(d, 3);
        elsif d >= 4 then
            r := signed('0' & g) + 1;
        elsif d <= -4 then
            r := signed('0' & g) - 1;
        else
            r := signed('0' & g);
        end if;
        return unsigned(r(9 downto 0));
    end function;

    ---------------------------------------------------------------------------
    -- RAMs / ROMs
    ---------------------------------------------------------------------------
    -- field ROMs (even / odd knot rows)
    signal s_fe_a, s_fo_a : unsigned(10 downto 0) := (others => '0');
    signal s_fe_q, s_fo_q : std_logic_vector(7 downto 0) := (others => '0');
    -- row RAMs (even / odd knot columns), {bank, col/2}, F8.4
    type t_row is array (0 to 127) of std_logic_vector(11 downto 0);
    signal ram_re, ram_ro : t_row := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of ram_re : signal is true;
    attribute no_rw_check of ram_ro : signal is true;
    signal s_rw_a : unsigned(6 downto 0) := (others => '0');
    signal s_rw_d : std_logic_vector(11 downto 0) := (others => '0');
    signal s_rw_e, s_rw_o : std_logic := '0';
    signal s_rr_e, s_rr_o : unsigned(6 downto 0) := (others => '0');
    signal s_rq_e, s_rq_o : std_logic_vector(11 downto 0) := (others => '0');
    -- luma ring (source x -> beam x, ~17 clocks later)
    type t_ring is array (0 to 255) of std_logic_vector(7 downto 0);
    signal ram_lu : t_ring := (others => x"10");
    attribute no_rw_check of ram_lu : signal is true;
    signal s_lu_wa, s_lu_ra : unsigned(7 downto 0) := (others => '0');
    signal s_lu_wd : std_logic_vector(7 downto 0) := x"10";
    signal s_lu_we : std_logic := '0';
    signal s_lu_q : std_logic_vector(7 downto 0) := x"10";
    -- band / colour ROMs
    signal s_bd_a : unsigned(7 downto 0) := (others => '0');
    signal s_bd_c : std_logic_vector(1 to 11) := (others => '0');
    signal s_cl_a : unsigned(7 downto 0) := (others => '0');
    signal s_cl_q : std_logic_vector(29 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- timing, controls, per-field constants
    ---------------------------------------------------------------------------
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_g1, s_g2, s_g3, s_g5, s_gp, s_gl_g : unsigned(9 downto 0) := (others => '0');
    signal s_gl_t : std_logic_vector(9 downto 0) := (others => '0');
    signal s_gl_r : unsigned(9 downto 0) := (others => '0');
    signal s_gl_d : signed(10 downto 0) := (others => '0');
    signal s_j6 : unsigned(2 downto 0) := (others => '0');
    signal s_live6 : std_logic := '0';
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_ctr, s_grey, s_src, s_drift, s_tile : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := (others => '0');
    signal s_spd : signed(10 downto 0) := (others => '0');
    signal s_crawl, s_crawlv : unsigned(15 downto 0) := (others => '0');
    signal s_lu_ph, s_lv_ph, s_br_ph : unsigned(15 downto 0) := to_unsigned(16384, 16);
    signal s_n16 : unsigned(7 downto 0) := to_unsigned(65, 8);
    signal s_ts8 : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_phi : unsigned(9 downto 0) := (others => '0');        -- max(0, p - 512)
    signal s_u1c : unsigned(6 downto 0) := to_unsigned(40, 7);     -- tooth mask end (columns)
    signal s_vu0 : unsigned(15 downto 0) := to_unsigned(51118, 16);
    signal s_dep : unsigned(5 downto 0) := (others => '0');
    signal s_dpx : unsigned(12 downto 0) := to_unsigned(1101, 13);
    signal s_px0 : unsigned(19 downto 0) := (others => '0');
    signal s_t3 : unsigned(17 downto 0) := (others => '0');
    signal s_dpy : unsigned(14 downto 0) := to_unsigned(1972, 15);
    signal s_py0 : unsigned(19 downto 0) := (others => '0');
    signal s_d0 : unsigned(15 downto 0) := (others => '0');
    signal s_dd : unsigned(8 downto 0) := (others => '0');
    signal s_vnq : unsigned(12 downto 0) := (others => '0');
    signal s_pp : unsigned(9 downto 0) := (others => '0');
    signal s_amp : unsigned(14 downto 0) := (others => '0');
    signal s_vamp : unsigned(12 downto 0) := (others => '0');
    signal s_fur : unsigned(10 downto 0) := (others => '0');
    signal s_gq : unsigned(9 downto 0) := to_unsigned(328, 10);
    signal s_pu, s_pv : signed(15 downto 0) := (others => '0');
    signal s_br : signed(11 downto 0) := (others => '0');
    -- field serial multiplier
    signal m_a, m_acc : unsigned(31 downto 0) := (others => '0');
    signal m_b : unsigned(15 downto 0) := (others => '0');
    signal m_neg, m_ld, m_stp, m_st : std_logic := '0';
    signal m_slot : unsigned(4 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- generated avid
    ---------------------------------------------------------------------------
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    ---------------------------------------------------------------------------
    -- source side (oscillator, luma ring)
    ---------------------------------------------------------------------------
    signal s_ox : unsigned(9 downto 0) := (others => '0');
    signal s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_wcol : unsigned(7 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- line engine
    ---------------------------------------------------------------------------
    signal e_kick : std_logic := '0';
    signal e_st : unsigned(3 downto 0) := (others => '0');
    signal e_wb, e_last, s_rbank : std_logic := '0';
    signal s_pyn : unsigned(19 downto 0) := (others => '0');
    signal e_v : unsigned(15 downto 0) := (others => '0');
    signal e_sv : signed(17 downto 0) := (others => '0');
    signal e_tv : unsigned(8 downto 0) := (others => '0');
    signal e_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal e_run : unsigned(2 downto 0) := (others => '0');
    signal e_nl : signed(7 downto 0) := (others => '0');
    signal e_ma : unsigned(28 downto 0) := (others => '0');
    signal e_mb : unsigned(12 downto 0) := (others => '0');
    signal e_macc : unsigned(28 downto 0) := (others => '0');
    signal e_mi : unsigned(3 downto 0) := (others => '0');
    signal e_sgn : std_logic := '0';
    signal e_ret : unsigned(3 downto 0) := (others => '0');
    signal n_a : signed(12 downto 0) := (others => '0');
    signal n_b : signed(11 downto 0) := (others => '0');
    signal n_tv : unsigned(8 downto 0) := (others => '0');
    signal l_a : signed(12 downto 0) := (others => '0');
    signal l_b : signed(11 downto 0) := (others => '0');
    signal l_tv : unsigned(8 downto 0) := (others => '0');
    -- column pipeline
    signal c_run : std_logic := '0';
    signal c_i : unsigned(6 downto 0) := (others => '0');
    signal c_phi, c_d : unsigned(15 downto 0) := (others => '0');
    signal c_a : unsigned(14 downto 0) := (others => '0');
    signal c_amp8 : unsigned(11 downto 0) := (others => '0');
    signal c_fcnt : unsigned(6 downto 0) := (others => '0');
    signal c_fade : std_logic := '0';
    signal c_v : std_logic_vector(13 downto 0) := (others => '0');   -- valid per stage (2b counts as one)
    signal c_i6, c_i11 : unsigned(6 downto 0) := (others => '0');
    signal c1_a : unsigned(14 downto 0) := (others => '0');
    signal c1_s8 : signed(8 downto 0) := (others => '0');
    signal c2_d : signed(16 downto 0) := (others => '0');
    signal c2_pl : signed(14 downto 0) := (others => '0');
    signal c2_ph : signed(14 downto 0) := (others => '0');
    signal c3_vv : signed(17 downto 0) := (others => '0');
    signal c4_v : unsigned(15 downto 0) := (others => '0');
    signal c5_iv : unsigned(5 downto 0) := (others => '0');
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    signal c_fv : t_u8(5 to 11) := (others => (others => '0'));
    signal c_sw : std_logic_vector(5 to 10) := (others => '0');
    signal c6_re81, c6_ro81 : unsigned(10 downto 0) := (others => '0');
    signal c9_e, c9_o : unsigned(7 downto 0) := (others => '0');
    signal c10_lo : unsigned(7 downto 0) := (others => '0');
    signal c10_df : signed(8 downto 0) := (others => '0');
    signal c11_lo : unsigned(7 downto 0) := (others => '0');
    signal c11_p : signed(17 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- pixel path
    ---------------------------------------------------------------------------
    signal s_px : unsigned(19 downto 0) := (others => '0');
    type t_u16 is array (natural range <>) of unsigned(15 downto 0);
    signal p_ua : t_u16(3 to 10) := (others => (others => '0'));
    signal p1_u : unsigned(15 downto 0) := (others => '0');
    signal p2_s : signed(17 downto 0) := (others => '0');
    signal p4_d1, p4_d2, p4_d3 : signed(16 downto 0) := (others => '0');
    signal p5_r1, p5_r2 : unsigned(8 downto 0) := (others => '0');
    signal p5_d3 : unsigned(10 downto 0) := (others => '0');
    signal p6_r1, p6_r2, p6_t : unsigned(8 downto 0) := (others => '0');
    signal p7_m1 : unsigned(9 downto 0) := (others => '0');
    signal p7_m2 : unsigned(10 downto 0) := (others => '0');
    signal p8_p1 : signed(18 downto 0) := (others => '0');
    signal p8_p2 : signed(17 downto 0) := (others => '0');
    signal p9_dl : signed(15 downto 0) := (others => '0');
    signal p10_s : signed(17 downto 0) := (others => '0');
    signal p11_ub : unsigned(15 downto 0) := (others => '0');
    signal p12_iu : unsigned(6 downto 0) := (others => '0');
    signal p_fu : t_u8(12 to 16) := (others => (others => '0'));
    signal p_sw : std_logic_vector(13 to 15) := (others => '0');
    signal p15_e, p15_o : unsigned(11 downto 0) := (others => '0');
    signal p16_lo : unsigned(11 downto 0) := (others => '0');
    signal p16_df : signed(12 downto 0) := (others => '0');
    signal p17_lo : unsigned(11 downto 0) := (others => '0');
    signal p17_p : signed(19 downto 0) := (others => '0');
    signal p15_lu : unsigned(7 downto 0) := x"10";
    signal p16_ls : signed(9 downto 0) := (others => '0');
    signal p17_vp : signed(16 downto 0) := (others => '0');
    signal p18_f : signed(13 downto 0) := (others => '0');
    signal p18_vp : signed(16 downto 0) := (others => '0');
    signal p19_fd : signed(14 downto 0) := (others => '0');
    signal p20_g : signed(21 downto 0) := (others => '0');
    signal p23_b, p24_b : std_logic_vector(3 downto 0) := (others => '0');
    signal p26_y, p26_u, p26_v, p27_y, p27_u, p27_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_rx : unsigned(7 downto 0) := (others => '0');
    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y13, s_y14, s_y15, s_y16, s_y17, s_y18x, s_yf18 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_from : process(clk)
    begin
        if rising_edge(clk) then
            s_fe_q <= C_FE(to_integer(s_fe_a));
            s_fo_q <= C_FO(to_integer(s_fo_a));
        end if;
    end process p_from;

    p_rowe : process(clk)
    begin
        if rising_edge(clk) then
            s_rq_e <= ram_re(to_integer(s_rr_e));
            if s_rw_e = '1' then ram_re(to_integer(s_rw_a)) <= s_rw_d; end if;
        end if;
    end process p_rowe;

    p_rowo : process(clk)
    begin
        if rising_edge(clk) then
            s_rq_o <= ram_ro(to_integer(s_rr_o));
            if s_rw_o = '1' then ram_ro(to_integer(s_rw_a)) <= s_rw_d; end if;
        end if;
    end process p_rowo;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_lu_q <= ram_lu(to_integer(s_lu_ra));
            if s_lu_we = '1' then ram_lu(to_integer(s_lu_wa)) <= s_lu_wd; end if;
        end if;
    end process p_ring;

    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            s_cl_q <= C_COL(to_integer(s_cl_a));
        end if;
    end process p_pal;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one serial multiplier)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_slot : integer range 0 to 31;
        variable v_x : unsigned(16 downto 0);
        variable v_t : unsigned(15 downto 0);
        variable v_d : signed(16 downto 0);
        variable v_r : unsigned(31 downto 0);
        variable v_h : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            if s_zone = '0' then
                s_zc <= (others => '0');
            elsif s_zc /= 1023 then
                s_zc <= s_zc + 1;
            end if;

            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- K6 Palette (8 labels; the firmware loads the default as the index)
            v_k := unsigned(registers_in(5));
            if v_k >= 8 then s_live6 <= '1'; end if;
            if s_live6 = '0' and v_k < 8 then
                s_j6 <= v_k(2 downto 0);
            elsif s_warm /= 15 or ((v_k(6 downto 0) >= 6 or v_k(9 downto 7) = 0) and
                                   (v_k(6 downto 0) <= 121 or v_k(9 downto 7) = 7)) then
                s_j6 <= v_k(9 downto 7);
            end if;

            -- zone clock 1: glides, switches, speed, phases
            if s_zone = '1' and s_zc = 1 then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_ctr   <= registers_in(6)(0);
                s_grey  <= registers_in(6)(1);
                s_src   <= registers_in(6)(2);
                s_drift <= registers_in(6)(3);
                s_tile  <= registers_in(6)(4);
                s_pal <= registers_in(6)(1) & s_j6;
                v_s := signed(resize(unsigned(registers_in(3)), 11)) - 512;      -- K4 Speed
                if v_s > -9 and v_s < 9 then v_s := (others => '0'); end if;
                s_spd <= v_s;
                s_crawl  <= s_crawl + unsigned(resize(shift_left(resize(s_spd, 16), 3), 16));
                s_crawlv <= s_crawlv + unsigned(resize(shift_left(resize(s_spd, 16), 2), 16))
                                     + unsigned(resize(shift_left(resize(s_spd, 16), 1), 16));
                s_lu_ph <= s_lu_ph + 109;
                s_lv_ph <= s_lv_ph + 78;
                if s_spd < 0 then
                    s_br_ph <= s_br_ph + unsigned(resize(-s_spd, 16));
                else
                    s_br_ph <= s_br_ph + unsigned(resize(s_spd, 16));
                end if;
            end if;
            -- zone clocks 4 .. 23: glides, four clocks per channel on one shared datapath
            -- (load, difference, ease, store); channel = zc(4:2) = 1 .. 5
            if s_zone = '1' and s_zc(9 downto 5) = 0 and s_zc(4 downto 2) /= 0 and
               s_zc(4 downto 2) /= 6 and s_zc(4 downto 2) /= 7 then
                case to_integer(s_zc(1 downto 0)) is
                    when 0 =>
                        case to_integer(s_zc(4 downto 2)) is
                            when 1      => s_gl_g <= s_g1; s_gl_t <= registers_in(0);
                            when 2      => s_gl_g <= s_g2; s_gl_t <= registers_in(1);
                            when 3      => s_gl_g <= s_g3; s_gl_t <= registers_in(2);
                            when 4      => s_gl_g <= s_g5; s_gl_t <= registers_in(4);
                            when others => s_gl_g <= s_gp; s_gl_t <= registers_in(7);
                        end case;
                    when 1 =>
                        s_gl_d <= signed('0' & unsigned(s_gl_t)) - signed('0' & s_gl_g);
                    when 2 =>
                        if s_warm = 1 then s_gl_r <= unsigned(s_gl_t); else s_gl_r <= f_glide(s_gl_g, s_gl_d); end if;
                    when others =>
                        case to_integer(s_zc(4 downto 2)) is
                            when 1      => s_g1 <= s_gl_r;
                            when 2      => s_g2 <= s_gl_r;
                            when 3      => s_g3 <= s_gl_r;
                            when 4      => s_g5 <= s_gl_r;
                            when others => s_gp <= s_gl_r;
                        end case;
                end case;
            end if;
            -- zone clock 24: direct per-field terms
            if s_zone = '1' and s_zc = 24 then
                v_h := resize(s_g1, 12) + shift_left(resize(s_g1, 12), 1);       -- 3 g1
                s_n16 <= to_unsigned(32, 8) + resize(v_h(11 downto 5), 8);
                v_h := resize(s_g3, 12) + shift_left(resize(s_g3, 12), 1);
                s_ts8 <= to_unsigned(32, 8) + resize(v_h(11 downto 5), 8);
                if s_gp(9) = '1' then s_phi <= '0' & s_gp(8 downto 0); else s_phi <= (others => '0'); end if;
                s_dep <= s_g5(9 downto 4);
            end if;
            if s_zone = '1' and s_zc = 25 then
                -- tooth mask end U1c = 40 + phi * 48 / 512; V-teeth mask start 0.78 - phi * 64
                v_h := shift_left(resize(s_phi, 12), 1) + resize(s_phi, 12);    -- 3 phi
                s_u1c <= to_unsigned(40, 7) + resize(v_h(11 downto 5), 7);       -- 3 phi / 32 = 48 phi / 512
                s_vu0 <= to_unsigned(51118, 16) - shift_left(resize(s_phi, 16), 6);
            end if;

            -- serial multiplier slots: slot = zc / 32, operands at sub 0,
            -- 16 shift-add steps, result at sub 20
            -- (decode registered a clock ahead)
            m_slot <= s_zc(9 downto 5);
            if s_zone = '1' and s_zc(4 downto 0) = 0 and s_zc(9 downto 5) /= 0 and s_zc(9) = '0' then
                m_ld <= '1'; else m_ld <= '0'; end if;
            if s_zone = '1' and s_zc(4 downto 0) >= 1 and s_zc(4 downto 0) <= 16 then
                m_stp <= '1'; else m_stp <= '0'; end if;
            if s_zone = '1' and s_zc(4 downto 0) = 20 and s_zc(9 downto 5) /= 0 and s_zc(9) = '0' then
                m_st <= '1'; else m_st <= '0'; end if;
            v_slot := to_integer(m_slot);
            if m_ld = '1' then
                m_acc <= (others => '0');
                m_neg <= '0';
                case v_slot is
                    when 1 =>
                        case to_integer(s_xs) is
                            when 4      => m_a <= to_unsigned(34712, 32);
                            when 3      => m_a <= to_unsigned(26034, 32);
                            when others => m_a <= to_unsigned(17356, 32);
                        end case;
                        m_b <= resize(s_n16, 16);
                    when 2 => m_a <= resize(s_w, 32); m_b <= resize(s_dpx, 16);
                    when 3 => m_a <= resize(s_ys, 32); m_b <= resize(s_n16, 16);
                    when 4 => m_a <= resize(s_t3, 32); m_b <= to_unsigned(7767, 16);
                    when 5 => m_a <= to_unsigned(132, 32); m_b <= resize(s_ts8, 16);
                    when 6 => m_a <= to_unsigned(215, 32); m_b <= resize(s_ts8, 16);
                    when 7 => m_a <= to_unsigned(403, 32); m_b <= resize(s_ts8, 16);
                    when 8 => m_a <= resize(s_gp, 32); m_b <= resize(s_gp, 16);
                    when 9 =>
                        if s_gp(9) = '0' then m_a <= to_unsigned(22282, 32); m_b <= resize(s_pp, 16);
                        else m_a <= to_unsigned(15729, 32); m_b <= resize(s_phi, 16); end if;
                    when 10 =>
                        if s_gp(9) = '0' then m_a <= to_unsigned(13107, 32); m_b <= resize(s_pp, 16);
                        else m_a <= to_unsigned(3277, 32); m_b <= resize(s_phi, 16); end if;
                    when 11 => m_a <= to_unsigned(945, 32); m_b <= resize(s_pp, 16);
                    when 12 => m_a <= to_unsigned(205, 32); m_b <= resize(s_g2, 16);
                    when 13 | 14 | 15 =>
                        -- LFOs: a triangle of the phase by bit logic (the phases start at
                        -- a quarter turn, so the LFO is 0 at t = 0), signed, x amplitude
                        if v_slot = 13 then v_t := s_lu_ph; m_a <= to_unsigned(14418, 32);
                        elsif v_slot = 14 then v_t := s_lv_ph; m_a <= to_unsigned(9175, 32);
                        else v_t := s_br_ph; m_a <= to_unsigned(1638, 32); end if;
                        if v_t(15) = '1' then v_x := resize(not v_t(14 downto 0), 17);
                        else v_x := resize(v_t(14 downto 0), 17); end if;            -- 0 .. 32767
                        m_neg <= not v_x(14);
                        if v_x(14) = '1' then m_b <= resize(v_x(13 downto 0), 16);
                        else m_b <= resize(not v_x(13 downto 0), 16); end if;
                    when others => m_a <= (others => '0'); m_b <= (others => '0');
                end case;
            elsif m_stp = '1' then
                if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
                m_a <= m_a(30 downto 0) & '0';
                m_b <= '0' & m_b(15 downto 1);
            elsif m_st = '1' then
                v_r := m_acc;
                case v_slot is
                    when 1 => s_dpx <= v_r(22 downto 10);
                    when 2 =>
                        if s_ctr = '1' then
                            s_px0 <= to_unsigned(0, 20) - v_r(20 downto 1);
                        else
                            s_px0 <= to_unsigned(524288, 20) - v_r(20 downto 1);
                        end if;
                    when 3 => s_t3 <= v_r(17 downto 0);
                    when 4 => s_dpy <= v_r(30 downto 16);
                    when 5 => s_d0 <= v_r(15 downto 0);
                    when 6 => s_dd <= v_r(14 downto 6);
                    when 7 => s_vnq <= v_r(15 downto 3);
                    when 8 => s_pp <= v_r(19 downto 10);
                    when 9 =>
                        if s_gp(9) = '0' then s_amp <= to_unsigned(1311, 15) + v_r(24 downto 10);
                        else s_amp <= to_unsigned(6881, 15) + v_r(24 downto 10); end if;
                    when 10 =>
                        if s_gp(9) = '0' then s_vamp <= to_unsigned(328, 13) + v_r(22 downto 10);
                        else s_vamp <= to_unsigned(3604, 13) + v_r(22 downto 10); end if;
                    when 11 =>
                        if v_r(19 downto 8) > 1136 then s_fur <= to_unsigned(1136, 11);
                        else s_fur <= v_r(18 downto 8); end if;
                    when 12 => s_gq <= to_unsigned(164, 10) + v_r(17 downto 8);
                    when 13 =>
                        if s_drift = '0' then s_pu <= (others => '0');
                        elsif m_neg = '1' then s_pu <= -signed(resize(v_r(29 downto 14), 16));
                        else s_pu <= signed(resize(v_r(29 downto 14), 16)); end if;
                    when 14 =>
                        if s_drift = '0' then s_pv <= (others => '0');
                        elsif m_neg = '1' then s_pv <= -signed(resize(v_r(29 downto 14), 16));
                        else s_pv <= signed(resize(v_r(29 downto 14), 16)); end if;
                    when others =>
                        if m_neg = '1' then s_br <= -signed(resize(v_r(25 downto 14), 12));
                        else s_br <= signed(resize(v_r(25 downto 14), 12)); end if;
                end case;
            end if;
            -- slot 16: the first line's phase (Py0 = centre - N16 * 2^14)
            if s_zone = '1' and s_zc = 512 then
                if s_ctr = '1' then
                    s_py0 <= to_unsigned(0, 20) - shift_left(resize(s_n16, 20), 14);
                else
                    s_py0 <= to_unsigned(524288, 20) - shift_left(resize(s_n16, 20), 14);
                end if;
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            g_wm1 <= s_w - 1;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Generated avid
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
    -- Source side: oscillator and the luma ring
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then s_ox <= s_ox + 3; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 5; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zone = '1' and s_zc = 1 then s_ot <= s_ot + 3; end if;
            s_osum <= resize(f_tri8(s_ox(9 downto 2) + s_ot), 8) + resize(f_tri8(s_oy(9 downto 2) - s_ot), 8);
            -- Osc luma = 502 + (sum - 127) * 7/4, written as 8 bits (luma / 4)
            v_l := to_unsigned(502 - 222, 10) + resize(s_osum, 10) + resize(s_osum(7 downto 2), 10)
                   + resize(s_osum(7 downto 1), 10);
            s_lu_we <= data_in.avid;
            s_lu_wa <= s_wcol;
            if s_src = '1' then
                s_lu_wd <= std_logic_vector(v_l(9 downto 2));
            else
                s_lu_wd <= data_in.y(9 downto 2);
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: the next line's row (81 knot columns) into the row RAM
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_sv : signed(17 downto 0);
        variable v_t : signed(17 downto 0);
        variable v_ph : unsigned(15 downto 0);
        variable v_s : signed(16 downto 0);
        variable v_p : signed(25 downto 0);
        variable v_r48 : unsigned(21 downto 0);
        variable v_re, v_ro : unsigned(4 downto 0);
        variable v_w : unsigned(11 downto 0);
        variable v_q : signed(17 downto 0);
        variable v_fa : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            -- kicks: once in the zone for line 0, then at every line start
            e_kick <= '0';
            if s_zone = '1' and s_zc = 520 then
                s_pyn <= s_py0;
                e_kick <= '1';
                -- the fur noise restarts every field: a still texture, moved only by the teeth / feathers
                e_lfsr <= x"ACE1";
                e_run <= (others => '0');
            elsif g_start = '1' then
                e_kick <= '1';
            end if;
            if g_start = '1' then
                l_a <= n_a; l_b <= n_b; l_tv <= n_tv;
                s_rbank <= e_last;
            end if;

            case to_integer(e_st) is
                when 0 =>
                    if e_kick = '1' then
                        e_v  <= f_tri20(s_pyn, s_tile);
                        s_pyn <= s_pyn + s_dpy;
                        e_wb <= not e_wb;
                        -- fur noise: xorshift per line, held 2 .. 6 lines
                        e_lfsr <= e_lfsr xor shift_left(e_lfsr, 7);
                        e_st <= to_unsigned(1, 4);
                    end if;
                when 1 =>
                    e_lfsr <= e_lfsr xor shift_right(e_lfsr, 9);
                    e_sv <= signed(resize(e_v, 18)) + resize(s_pv, 18);
                    e_st <= to_unsigned(2, 4);
                when 2 =>
                    e_lfsr <= e_lfsr xor shift_left(e_lfsr, 8);
                    e_v <= f_fold(e_sv);
                    e_st <= to_unsigned(3, 4);
                when 3 =>
                    if e_run = 0 then
                        e_nl <= signed(e_lfsr(7 downto 0));
                        e_run <= resize(e_lfsr(9 downto 8), 3) + 1 + resize(e_lfsr(10 downto 10), 3);
                    else
                        e_run <= e_run - 1;
                    end if;
                    -- tent (v part): 256 - |v - 0.46| * 4
                    v_t := signed(resize(e_v, 18)) - 30147;
                    if v_t < 0 then v_t := -v_t; end if;
                    if v_t(17 downto 6) >= 256 then e_tv <= (others => '0');
                    else e_tv <= to_unsigned(256, 9) - unsigned(v_t(14 downto 6)); end if;
                    -- v x VNQ on the engine's serial multiplier
                    e_ma <= resize(e_v, 29); e_mb <= s_vnq; e_macc <= (others => '0'); e_mi <= (others => '0');
                    e_ret <= to_unsigned(5, 4);
                    e_st <= to_unsigned(4, 4);
                when 4 =>
                    -- 13-step serial multiply, then back to e_ret
                    if e_mb(0) = '1' then e_macc <= e_macc + e_ma; end if;
                    e_ma <= e_ma(27 downto 0) & '0';
                    e_mb <= '0' & e_mb(12 downto 1);
                    if e_mi = 12 then e_st <= e_ret; end if;
                    e_mi <= e_mi + 1;
                when 5 =>
                    -- V-teeth saw: 0.5 - frac(v * VN + crawl); then |saw| x feather depth
                    v_ph := e_macc(23 downto 8) + s_crawlv;
                    v_s := 32768 - signed(resize(v_ph, 17));
                    if v_s(16 downto 8) > 127 then v_s(16 downto 8) := to_signed(127, 9); end if;
                    e_sgn <= v_s(16);
                    e_ma <= resize(s_vamp, 29);
                    if v_s(16) = '1' then e_mb <= unsigned(resize(-v_s(16 downto 8), 13));
                    else e_mb <= unsigned(resize(v_s(16 downto 8), 13)); end if;
                    e_macc <= (others => '0'); e_mi <= (others => '0');
                    e_ret <= to_unsigned(6, 4);
                    e_st <= to_unsigned(4, 4);
                when 6 =>
                    -- A = vamp * saw / 256; then |nl| x fur depth
                    if e_sgn = '1' then n_a <= -signed(resize(e_macc(20 downto 8), 13));
                    else n_a <= signed(resize(e_macc(20 downto 8), 13)); end if;
                    e_sgn <= e_nl(7);
                    e_ma <= resize(s_fur, 29);
                    if e_nl(7) = '1' then e_mb <= unsigned(resize(-resize(e_nl, 9), 13));
                    else e_mb <= unsigned(resize(e_nl, 13)); end if;
                    e_macc <= (others => '0'); e_mi <= (others => '0');
                    e_ret <= to_unsigned(7, 4);
                    e_st <= to_unsigned(4, 4);
                when 7 =>
                    -- B = fur * nl / 128
                    if e_sgn = '1' then n_b <= -signed(resize(e_macc(18 downto 7), 12));
                    else n_b <= signed(resize(e_macc(18 downto 7), 12)); end if;
                    n_tv <= e_tv;
                    -- start the column loop
                    c_run <= '1';
                    c_i <= (others => '0');
                    c_i6 <= (others => '0');
                    c_i11 <= (others => '0');
                    c_phi <= to_unsigned(49807, 16) + s_crawl;
                    c_d <= s_d0;
                    c_a <= s_amp;
                    c_amp8 <= s_amp(14 downto 3);
                    c_fcnt <= s_u1c - 9;
                    c_fade <= '0';
                    e_st <= to_unsigned(8, 4);
                when others =>
                    if c_run = '0' and c_v = (c_v'range => '0') then
                        e_last <= e_wb;
                        e_st <= to_unsigned(0, 4);
                    end if;
            end case;

            -- column stage 0: DDA
            c_v(1) <= c_run;
            if c_run = '1' then
                c1_a <= c_a;
                v_s := 32768 - signed(resize(c_phi, 17));
                if v_s(16 downto 8) > 127 then c1_s8 <= to_signed(127, 9);
                else c1_s8 <= v_s(16 downto 8); end if;
                c_phi <= c_phi + c_d;
                c_d <= c_d + resize(s_dd, 16);
                -- the tooth depth fades to 0 over 8 columns ending at the mask column
                if c_fcnt = 0 then
                    c_fade <= '1';
                else
                    c_fcnt <= c_fcnt - 1;
                end if;
                if c_fade = '1' then
                    v_fa := signed('0' & c_a) - signed('0' & resize(c_amp8, 15));
                    if v_fa < 0 then c_a <= (others => '0'); else c_a <= unsigned(v_fa(14 downto 0)); end if;
                end if;
                if c_i = 80 then c_run <= '0'; end if;
                c_i <= c_i + 1;
            end if;
            for k in 2 to 13 loop c_v(k) <= c_v(k - 1); end loop;
            -- 2: d = a * s8 / 256, as (a / 64) * s8 / 4: partial products on the s8 nibbles
            c2_pl <= signed('0' & c1_a(14 downto 6)) * signed('0' & c1_s8(3 downto 0));
            c2_ph <= signed('0' & c1_a(14 downto 6)) * c1_s8(8 downto 4);
            -- 2b: sum
            c2_d <= resize(shift_right(shift_left(resize(c2_ph, 17), 4) + resize(c2_pl, 17), 2), 17);
            -- 3: v + d
            c3_vv <= signed(resize(e_v, 18)) + resize(c2_d, 18);
            -- 4: fold
            c4_v <= f_fold(c3_vv);
            -- 5: v * 48 -> row, fraction
            v_r48 := shift_left(resize(c4_v, 22), 5) + shift_left(resize(c4_v, 22), 4);
            c5_iv <= v_r48(21 downto 16);
            c_fv(5) <= v_r48(15 downto 8);
            c_sw(5) <= v_r48(16);
            for k in 6 to 11 loop c_fv(k) <= c_fv(k - 1); end loop;
            for k in 6 to 10 loop c_sw(k) <= c_sw(k - 1); end loop;
            -- 6: row bases x 81
            v_re := resize(shift_right(resize(c5_iv, 7) + 1, 1), 5);
            v_ro := resize(shift_right(c5_iv, 1), 5);
            c6_re81 <= shift_left(resize(v_re, 11), 6) + shift_left(resize(v_re, 11), 4) + resize(v_re, 11);
            c6_ro81 <= shift_left(resize(v_ro, 11), 6) + shift_left(resize(v_ro, 11), 4) + resize(v_ro, 11);
            -- 7: ROM addresses (c_i6 counts the columns passing stage 6)
            s_fe_a <= c6_re81 + resize(c_i6, 11);
            s_fo_a <= c6_ro81 + resize(c_i6, 11);
            if c_v(7) = '1' then c_i6 <= c_i6 + 1; end if;
            -- 8: ROM out; 9: fabric
            c9_e <= unsigned(s_fe_q);
            c9_o <= unsigned(s_fo_q);
            -- 10: lo / hi by row parity
            if c_sw(9) = '1' then
                c10_lo <= c9_o; c10_df <= signed(resize(c9_e, 9)) - signed(resize(c9_o, 9));
            else
                c10_lo <= c9_e; c10_df <= signed(resize(c9_o, 9)) - signed(resize(c9_e, 9));
            end if;
            -- 11: diff x fv
            c11_lo <= c10_lo;
            c11_p <= c10_df * signed(resize(c_fv(10), 9));
            -- 12: R = lo * 16 + p / 16, written to the row RAM
            v_q := signed(resize(c11_lo & "0000", 18)) + resize(shift_right(c11_p, 4), 18);
            if v_q < 0 then v_w := (others => '0');
            elsif v_q > 4095 then v_w := (others => '1');
            else v_w := unsigned(v_q(11 downto 0)); end if;
            s_rw_d <= std_logic_vector(v_w);
            s_rw_a <= e_wb & c_i11(6 downto 1);
            s_rw_e <= c_v(12) and not c_i11(0);
            s_rw_o <= c_v(12) and c_i11(0);
            if c_v(12) = '1' then c_i11 <= c_i11 + 1; end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_s : signed(17 downto 0);
        variable v_r : signed(20 downto 0);
        variable v_u80 : unsigned(22 downto 0);
        variable v_iu1 : unsigned(7 downto 0);
        variable v_g : signed(24 downto 0);
        variable v_i : unsigned(15 downto 0);
        variable v_t : signed(10 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_n : integer range 0 to 11;
    begin
        if rising_edge(clk) then
            -- 0: mirror phase (aligned with g_avid)
            if g_start = '1' then
                s_px <= s_px0;
            elsif g_avid = '1' then
                s_px <= s_px + resize(s_dpx, 20);
            end if;
            -- 1: fold / tile
            p1_u <= f_tri20(s_px, s_tile);
            -- 2: pan (drift)
            p2_s <= signed(resize(p1_u, 18)) + resize(s_pu, 18);
            -- 3: fold
            p_ua(3) <= f_fold(p2_s);
            for k in 4 to 10 loop p_ua(k) <= p_ua(k - 1); end loop;
            -- 4: mask distances
            p4_d1 <= signed(resize(p_ua(3), 17)) - signed(resize(s_vu0, 17));
            p4_d2 <= signed(resize(p_ua(3), 17)) - 29491;
            p4_d3 <= signed(resize(p_ua(3), 17)) - 49807;
            -- 5: ramps (Q8): r1 = (u - vu0) * 10 / 256, r2 = (u - 0.45) / 32, |u - 0.76| / 32
            v_r := shift_left(resize(p4_d1, 21), 3) + shift_left(resize(p4_d1, 21), 1);
            if v_r < 0 then p5_r1 <= (others => '0');
            elsif v_r(20 downto 8) >= 256 then p5_r1 <= to_unsigned(256, 9);
            else p5_r1 <= unsigned(v_r(16 downto 8)); end if;
            if p4_d2 < 0 then p5_r2 <= (others => '0');
            elsif p4_d2(16 downto 5) >= 256 then p5_r2 <= to_unsigned(256, 9);
            else p5_r2 <= unsigned(p4_d2(13 downto 5)); end if;
            if p4_d3 < 0 then p5_d3 <= resize(unsigned(-p4_d3(16 downto 5)), 11);
            else p5_d3 <= resize(unsigned(p4_d3(16 downto 5)), 11); end if;
            -- 6: tent = tv - |u - 0.76| * 8
            p6_r1 <= p5_r1; p6_r2 <= p5_r2;
            if resize(p5_d3, 12) >= resize(l_tv, 12) then p6_t <= (others => '0');
            else p6_t <= l_tv - resize(p5_d3, 9); end if;
            -- 7: masks
            p7_m1 <= resize(p6_r1, 10) + shift_left(resize(p6_t, 10), 1);
            p7_m2 <= resize(p6_r2, 11) + shift_left(resize(p6_t, 11), 2);
            -- 8: products on narrowed operands: (A/8)(M1/4) and (B/8)(M2/8)
            p8_p1 <= l_a(12 downto 3) * signed('0' & p7_m1(9 downto 2));
            p8_p2 <= l_b(11 downto 3) * signed('0' & p7_m2(10 downto 3));
            -- 9: displacement = (A M1 + B M2) / 256
            p9_dl <= resize(shift_right(resize(p8_p1, 20) + shift_left(resize(p8_p2, 20), 1), 3), 16);
            -- 10: u + displacement
            p10_s <= signed(resize(p_ua(9), 18)) + resize(p9_dl, 18);
            -- 11: fold
            p11_ub <= f_fold(p10_s);
            -- 12: u * 80 -> column, fraction
            v_u80 := shift_left(resize(p11_ub, 23), 6) + shift_left(resize(p11_ub, 23), 4);
            p12_iu <= v_u80(22 downto 16);
            p_fu(12) <= v_u80(15 downto 8);
            for k in 13 to 16 loop p_fu(k) <= p_fu(k - 1); end loop;
            -- 13: row RAM addresses: even RAM holds column iu or iu+1
            v_iu1 := resize(p12_iu, 8) + 1;
            s_rr_e <= s_rbank & v_iu1(6 downto 1);
            s_rr_o <= s_rbank & p12_iu(6 downto 1);
            p_sw(13) <= p12_iu(0);
            p_sw(14) <= p_sw(13); p_sw(15) <= p_sw(14);
            -- 14: RAM out; 15: fabric
            p15_e <= unsigned(s_rq_e);
            p15_o <= unsigned(s_rq_o);
            -- 16: lo / hi
            if p_sw(15) = '1' then
                p16_lo <= p15_o; p16_df <= signed(resize(p15_e, 13)) - signed(resize(p15_o, 13));
            else
                p16_lo <= p15_e; p16_df <= signed(resize(p15_o, 13)) - signed(resize(p15_e, 13));
            end if;
            -- 17: diff x fu; video product
            p17_lo <= p16_lo;
            p17_p <= p16_df * signed('0' & p_fu(16)(7 downto 2));
            p15_lu <= unsigned(s_lu_q);
            p16_ls <= signed(resize(p15_lu, 10)) - 126;
            p17_vp <= p16_ls * signed('0' & s_dep);
            -- 18: F
            p18_f <= signed(resize(p17_lo, 14)) + resize(shift_right(p17_p, 6), 14);
            p18_vp <= p17_vp;
            -- 19: F + video - pivot
            p19_fd <= resize(p18_f, 15) + resize(shift_right(p18_vp, 3), 15) - 2080;
            -- 20: gain, (Fd / 4)(gq / 4)
            p20_g <= p19_fd(14 downto 2) * signed('0' & s_gq(9 downto 2));
            -- 21: colourizer index (one cycle = 256)
            v_i := unsigned(p20_g(15 downto 0)) + to_unsigned(32768, 16) + unsigned(resize(s_br, 16));
            s_bd_a <= v_i(15 downto 8);
            -- 22: band thresholds; 23: band = how many are passed
            for k in 1 to 11 loop
                if s_bd_a >= C_BT(k) then s_bd_c(k) <= '1'; else s_bd_c(k) <= '0'; end if;
            end loop;
            v_n := 0;
            for k in 1 to 11 loop
                if s_bd_c(k) = '1' then v_n := v_n + 1; end if;
            end loop;
            p23_b <= std_logic_vector(to_unsigned(v_n, 4));
            -- 24: colour ROM address
            s_cl_a <= s_pal & unsigned(p23_b);
            -- 25: colour ROM out; 26: fabric; 27, 28: pads to the compose stage
            p26_y <= unsigned(s_cl_q(29 downto 20));
            p26_u <= unsigned(s_cl_q(19 downto 10));
            p26_v <= unsigned(s_cl_q(9 downto 0));
            p27_y <= p26_y; p27_u <= p26_u; p27_v <= p26_v;
            s_y12 <= p27_y; s_u12 <= p27_u; s_v12 <= p27_v;

            -- luma ring read address (beam x at stage 14)
            if s_gav_sr(11) = '1' then s_rx <= s_rx + 1; else s_rx <= (others => '0'); end if;
            s_lu_ra <= s_rx;

            -- output conditioning (centre of the chroma box at C_LAT - 3)
            s_csu(0) <= s_u12; s_csv(0) <= s_v12;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u12 - s_csu(7);
            s_boxv <= s_boxv + s_v12 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u18 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v18 <= v_sc(13 downto 4);
            s_y13 <= s_y12; s_y14 <= s_y13; s_y15 <= s_y14; s_y16 <= s_y15;
            s_y17 <= s_y16; s_y18x <= s_y17;
            v_sy := resize(s_y16, 12) + shift_left(resize(s_y17, 12), 1) + resize(s_y18x, 12);
            s_yf18 <= v_sy(11 downto 2);
            -- luma toward black; blanking gate
            if s_yf18 > 64 then v_dl := s_yf18 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(C_LAT - 3) = '1' then
                s_y19 <= s_yf18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u19 <= s_u18; s_v19 <= s_v18;
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture starburst;
