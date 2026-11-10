-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Butterfly v0.1.1 -- Heirloom (video clips), rebuilt from the still WIN_20231008_20_41_21 t0.5.
--
-- v0.1.1: the camera bends the rings smoothly: a vertical IIR on the line-buffer write ((previous + new) / 2,
-- a second 2048 x 8 block RAM) and a horizontal IIR on the read (h += (f - h) / 4), and the default Bend is a
-- quarter of v0.1.0's (K6 224, square law); the Osc multiplier rises 4x so Source = Osc flips exactly as before.
--
-- The still: a magenta / blue / black kaleidoscope.  Big magenta lenses pinched at dark X crossings, a blue band
-- bowed over and under each lens, hot-magenta hearts above and below every X (tip at the X, notch toward a nested
-- blue diamond), blue arcs bracketing the diamonds, and a bow-tie glow between the arcs.  The lattice repeats every
-- 940 px across and 524.5 lines down, mirrored four ways.  The clip held perfectly still for 8.5 s.
--
-- The patch, as fitted (butterfly.py has every constant):
--   u, v   = folded H / V phase (cell coordinates: 0 .. 470 px from an X column, 0 .. 262 lines from a lens row)
--   F1     = k1 (ln v - ln sin(pi u / 940) - ln t0)     the lens field: a pencil of curves through each X point
--   F2     = gain (table(u, v) + offset)                the heart / diamond / arc rings (33 x 17 bilinear nodes)
--   knot   = near each X, F1 is blended toward a constant-width dark arm (the camera's blur of the pencil)
--   |g|    = min(|F1 + cam|, |F2 + cam|)               whichever field is nearer its blue line wins
--   colour = palette(|g|): 0 blue, 1 black, 2.45+ magenta
--
--   K1  Columns   H oscillator rate: butterflies across (x0.5 .. x3; 2.04 in the still)
--   K2  Rows      V oscillator rate: butterfly rows (x0.5 .. x3)
--   K3  Hearts    heart size: small lobes in rings (0) .. the still (512) .. hearts swallowing the diamonds (1023)
--   K4  Flap      wing beat: the lenses open while the hearts shrink, and back (0 = still, as in the clip)
--   K5  Rings     band weight of every ring: wide soft bands (0) .. the still .. hairlines (1023)
--   K6  Bend      how far the camera (or the Osc, 16x and centred) bends the rings
--   S7  Echo      Off (the still) / On: every magenta field repeats the ring sequence
--   S8  Mix       Darker (the still: nearer blue wins) / Interfere (the difference of the two fields)
--   S9  Palette   Magenta (the still) / Ember (amber, teal bands)
--   S10 Twin      Off / On: hearts and diamonds also on the lens columns
--   S11 Source    Video / Osc
--   P12 WINGS     lens width with heart size: closed slivers (0) .. the still (512) .. open wings and flooding hearts
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line start
-- (computes the NEXT line: ln v, and the F2 row interpolated across the 33 table columns into a ping-pong block RAM),
-- a 25-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output
-- conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16) before the blanking gate.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.butterfly_pkg.all;
use work.all;

architecture butterfly of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

    constant C_PHX_C : integer := 7844173;    -- H phase at the screen centre (x = 960): (960 - 520.5) / 940 * 2^24
    constant C_PHY_C : integer := 9068205;    -- V phase at y = 540: (540 - 256.5) / 524.5 * 2^24
    constant C_LN174 : integer := 1321;       -- 256 ln 174

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
    -- fold a 24-bit phase to 15 bits (0 at phase 0, 32767 at half a period)
    function f_fold(p : unsigned(23 downto 0)) return unsigned is
    begin
        if p(23) = '1' then return not p(22 downto 8); else return p(22 downto 8); end if;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    -- vertical IIR state: the smoothed luma of the previous source line, read 3 clocks ahead of its rewrite
    signal fr : t_lb := (others => x"00");
    signal s_fr_ra : unsigned(10 downto 0) := (others => '0');
    signal s_fr_q  : std_logic_vector(7 downto 0) := x"00";
    signal w1_l, w2_l, w3_l, w3_f : unsigned(7 downto 0) := (others => '0');
    signal w1_v, w2_v, w3_v : std_logic := '0';
    signal s_wcol2 : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_lp_a  : unsigned(9 downto 0) := (others => '0');
    signal r_lp_q  : unsigned(11 downto 0) := (others => '0');
    signal r_lv_a  : unsigned(8 downto 0) := (others => '0');
    signal r_lv_q  : signed(11 downto 0) := (others => '0');
    signal r_tb_a  : unsigned(9 downto 0) := (others => '0');
    signal r_tb_q  : signed(7 downto 0) := (others => '0');
    signal r_pl_a  : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_ex_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q  : unsigned(15 downto 0) := (others => '0');

    -- F2 row ping-pong RAM: {row (12, Q2 G units), d = next - row (10)} at bank * 32 + i
    type t_rw is array (0 to 255) of std_logic_vector(31 downto 0);
    signal rw : t_rw := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of rw : signal is true;
    attribute no_rw_check of fr : signal is true;
    signal rw_we : std_logic := '0';
    signal rw_wa, rw_ra : unsigned(7 downto 0) := (others => '0');
    signal rw_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal rw_q  : std_logic_vector(31 downto 0) := (others => '0');

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

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(224 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := (others => '0');
    signal s_echo, s_mix, s_pal, s_twin, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_neg  : std_logic := '0';
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_a    : unsigned(15 downto 0) := (others => '0');
    signal z_kxh  : unsigned(16 downto 0) := to_unsigned(8924, 17);
    signal z_kyl  : unsigned(17 downto 0) := to_unsigned(31987, 18);
    signal z_l0p  : signed(12 downto 0) := (others => '0');
    signal z_lfa  : signed(10 downto 0) := (others => '0');
    signal z_lfb  : signed(12 downto 0) := (others => '0');
    signal z_d3   : signed(10 downto 0) := (others => '0');
    signal s_lph  : unsigned(15 downto 0) := (others => '0');       -- flap LFO phase
    signal s_lfo  : signed(9 downto 0) := (others => '0');          -- -256 .. 255
    signal c_lstep : unsigned(11 downto 0) := (others => '0');
    signal c_kstep : unsigned(16 downto 0) := to_unsigned(17848, 17);
    signal c_phx0 : unsigned(23 downto 0) := (others => '0');
    signal c_kys  : unsigned(19 downto 0) := to_unsigned(31987, 20);
    signal c_phy0 : unsigned(23 downto 0) := (others => '0');
    signal c_k1q  : unsigned(8 downto 0) := to_unsigned(150, 9);
    signal c_g2q  : unsigned(8 downto 0) := to_unsigned(104, 9);
    signal c_l0   : signed(12 downto 0) := to_signed(C_LN174, 13);
    signal c_off  : signed(12 downto 0) := (others => '0');
    signal c_gq   : unsigned(7 downto 0) := (others => '0');
    signal c_moff : unsigned(7 downto 0) := (others => '0');
    signal c_ky, c_ku, c_kv : unsigned(9 downto 0) := to_unsigned(512, 10);   -- the palette's black

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

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_zk   : std_logic := '0';
    signal e_lc   : unsigned(6 downto 0) := to_unsigned(127, 7);
    signal e_phy  : unsigned(23 downto 0) := (others => '0');
    signal e_v    : unsigned(14 downto 0) := (others => '0');
    signal e_t131 : unsigned(21 downto 0) := (others => '0');
    signal e_bv   : unsigned(8 downto 0) := (others => '0');
    signal e_tb0, e_tb1 : unsigned(9 downto 0) := (others => '0');
    signal e_fv   : unsigned(5 downto 0) := (others => '0');
    signal e_bank : std_logic := '0';
    signal e_lv   : signed(11 downto 0) := (others => '0');
    signal e_i    : unsigned(5 downto 0) := (others => '0');
    signal e_ra   : std_logic := '0';           -- this clock's table read is the A (row j) node
    signal e_va   : std_logic_vector(2 downto 0) := (others => '0');   -- data valid pipeline: tA at 2, tB at 3
    signal e_tA, e_tAh, e_tB : signed(7 downto 0) := (others => '0');
    signal e_x1v, e_x2v, e_x3v, e_x4v, e_x5v, e_x6v, e_x7v : std_logic := '0';
    signal e_x1d  : signed(8 downto 0) := (others => '0');
    signal e_x1a, e_x2a : signed(7 downto 0) := (others => '0');
    signal e_x2p  : signed(15 downto 0) := (others => '0');
    signal e_x3l  : signed(14 downto 0) := (others => '0');
    signal e_x4s  : signed(15 downto 0) := (others => '0');
    signal e_x5a  : signed(21 downto 0) := (others => '0');
    signal e_x5b  : signed(20 downto 0) := (others => '0');
    signal e_x6p  : signed(25 downto 0) := (others => '0');
    signal e_x7r  : signed(11 downto 0) := (others => '0');
    signal e_prev : signed(11 downto 0) := (others => '0');
    signal e_wi   : unsigned(5 downto 0) := (others => '0');
    signal e_havep : std_logic := '0';
    signal p_bank : std_logic := '0';
    signal l_lvl  : signed(12 downto 0) := (others => '0');    -- ln v - L0 of the line
    signal l_bv   : unsigned(8 downto 0) := (others => '0');

    -- beam
    signal b_phx : unsigned(23 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    signal q2_u, q3_u : unsigned(14 downto 0) := (others => '0');
    signal q2_fu, q3_fu, q4_fu : unsigned(5 downto 0) := (others => '0');
    signal q3_t1  : unsigned(22 downto 0) := (others => '0');
    signal q3_t2  : unsigned(16 downto 0) := (others => '0');
    signal q4_a   : unsigned(8 downto 0) := (others => '0');
    signal q4_lp  : unsigned(11 downto 0) := (others => '0');
    signal q4_row : signed(11 downto 0) := (others => '0');
    signal q4_d   : signed(9 downto 0) := (others => '0');
    signal q4_lm  : unsigned(7 downto 0) := (others => '0');
    signal q5_l   : signed(12 downto 0) := (others => '0');
    signal q5_dfu : signed(16 downto 0) := (others => '0');
    signal q5_row : signed(11 downto 0) := (others => '0');
    signal q5_mx, q5_mn : unsigned(8 downto 0) := (others => '0');
    signal q5_m   : unsigned(15 downto 0) := (others => '0');
    signal q6_f1a : signed(18 downto 0) := (others => '0');
    signal q6_f1b : signed(17 downto 0) := (others => '0');
    signal q6_f2q : signed(12 downto 0) := (others => '0');
    signal q6_mx  : unsigned(8 downto 0) := (others => '0');
    signal q6_m38 : unsigned(7 downto 0) := (others => '0');
    signal q6_m, q7_m : signed(9 downto 0) := (others => '0');
    signal q7_f1p : signed(22 downto 0) := (others => '0');
    signal q7_f2m : signed(11 downto 0) := (others => '0');
    signal q7_rr, q8_rr, q9_rr : unsigned(8 downto 0) := (others => '0');
    signal q8_f1m : signed(15 downto 0) := (others => '0');
    signal q8_b, q9_b, q10_b, q11_b, q12_b, q13_b, q14_b : unsigned(7 downto 0) := (others => '0');
    signal q9_a   : unsigned(9 downto 0) := (others => '0');
    signal q9_ta  : unsigned(16 downto 0) := (others => '0');
    signal q9_tb  : unsigned(12 downto 0) := (others => '0');
    signal q10_pl : unsigned(13 downto 0) := (others => '0');
    signal q10_ph : unsigned(13 downto 0) := (others => '0');
    signal q10_ac, q11_ac, q12_ac, q13_ac : unsigned(6 downto 0) := (others => '0');
    signal q10_c39, q11_c39 : signed(7 downto 0) := (others => '0');
    signal q10_f8p : unsigned(8 downto 0) := (others => '0');
    signal q11_prod : unsigned(18 downto 0) := (others => '0');
    signal q11_f8, q12_f8 : unsigned(7 downto 0) := (others => '0');
    signal q12_df : signed(7 downto 0) := (others => '0');
    signal q13_fd : signed(16 downto 0) := (others => '0');
    signal q14_g1 : unsigned(7 downto 0) := (others => '0');
    signal q15_ag : unsigned(7 downto 0) := (others => '0');
    signal q18_c  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23x, s_u23x, s_v23x : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_fr : process(clk)
    begin
        if rising_edge(clk) then
            s_fr_q <= fr(to_integer(s_fr_ra));
            if s_lb_we = '1' then fr(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_fr;

    p_rw : process(clk)
    begin
        if rising_edge(clk) then
            rw_q <= rw(to_integer(rw_ra));
            if rw_we = '1' then rw(to_integer(rw_wa)) <= rw_wd; end if;
        end if;
    end process p_rw;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_lp_q <= C_BF_LP(to_integer(r_lp_a));
            r_lv_q <= C_BF_LV(to_integer(r_lv_a));
            r_tb_q <= C_BF_TAB(to_integer(r_tb_a));
            r_pl_q <= C_BF_PAL(to_integer(r_pl_a));
            r_ex_q <= C_BF_EXP2(to_integer(r_ex_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_d  : signed(10 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_t  : unsigned(35 downto 0);
        variable v_s  : signed(14 downto 0);
        variable v_k  : std_logic_vector(31 downto 0);
        variable v_l  : signed(9 downto 0);
        variable v_ph : unsigned(15 downto 0);
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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_echo <= registers_in(6)(0);
                s_mix  <= registers_in(6)(1);
                s_pal  <= registers_in(6)(2);
                s_twin <= registers_in(6)(3);
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

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from load + 17
            -- (stored at load + 18).  The slot is decoded at 30 + 24 k, operands pre-muxed at 31 + 24 k.
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
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(z_a, 36);                 z_fb <= to_unsigned(203, 16);     -- K1 exponent
                    when 1  => z_fa <= to_unsigned(8924, 36);           z_fb <= z_mt;                      -- kxh
                    when 2  => z_fa <= resize(z_a, 36);                 z_fb <= to_unsigned(203, 16);     -- K2 exponent
                    when 3  => z_fa <= to_unsigned(31987, 36);          z_fb <= z_mt;                      -- kyl
                    when 4  => z_fa <= resize(z_kyl, 36);               z_fb <= resize(s_ys, 16);          -- kys
                    when 5  => z_fa <= resize(z_kyl, 36);               z_fb <= to_unsigned(540, 16);      -- 540 kyl
                    when 6  =>                                -- (1920 - x0 - 8 xs) kxh: 8 px ahead, the output boxes' delay
                        z_fa <= resize(z_kxh, 36);
                        case to_integer(s_xs) is
                            when 4      => z_fb <= to_unsigned(1408, 16);
                            when 3      => z_fb <= to_unsigned(1896, 16);
                            when others => z_fb <= to_unsigned(1904, 16);
                        end case;
                    when 7  =>                                                                             -- K5 exponent
                        z_fa <= resize(z_a, 36);
                        if z_neg = '1' then z_fb <= to_unsigned(115, 16); else z_fb <= to_unsigned(166, 16); end if;
                    when 8  => z_fa <= to_unsigned(150, 36);            z_fb <= z_mt;                      -- k1q
                    when 9  => z_fa <= to_unsigned(209, 36);            z_fb <= z_mt;                      -- g2q
                    when 10 => z_fa <= resize(z_a, 36);                 z_fb <= to_unsigned(204, 16);     -- P12 wings
                    when 11 => z_fa <= resize(z_a, 36);                 z_fb <= to_unsigned(89, 16);      -- |lfo| * 89
                    when 12 => z_fa <= resize(z_a, 36);                 z_fb <= to_unsigned(819, 16);     -- |lfo| * 819
                    when 13 => z_fa <= resize(s_g6(12 downto 3), 36);   z_fb <= resize(s_g6(12 downto 3), 16);  -- bend^2
                    when 14 => z_fa <= resize(s_k4, 36);                z_fb <= resize(s_k4, 16);          -- flap^2
                    when others => z_fa <= resize(c_gq, 36);            z_fb <= to_unsigned(111, 16);      -- Osc centre
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
                    -- flap LFO: advance by last field's step, triangle -256 .. 255
                    when 10 =>
                        s_lph <= s_lph + resize(c_lstep, 16);
                    when 11 =>                                -- a quarter period ahead, so phase 0 = rest (0)
                        v_ph := s_lph + 16384;
                        if v_ph(15) = '0' then
                            s_lfo <= signed(resize(v_ph(14 downto 6), 10)) - 256;
                        else
                            s_lfo <= 255 - signed(resize(v_ph(14 downto 6), 10));
                        end if;
                    when 12 =>
                        -- K1: d = k - 512; down = d / 2 (one octave), up = d * 203 / 256 (1.585 octaves)
                        v_d := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        z_neg <= v_d(10);
                        if v_d(10) = '1' then z_a <= (others => '0'); z_e <= resize(shift_right(v_d, 1), 12);
                        else z_a <= resize(unsigned(v_d(9 downto 0)), 16); end if;
                    when 13 =>
                        if s_pal = '1' then v_k := C_BF_PAL(128 + 26); else v_k := C_BF_PAL(26); end if;
                        c_ky <= unsigned(v_k(31 downto 22)); c_ku <= unsigned(v_k(21 downto 12)); c_kv <= unsigned(v_k(11 downto 2));
                    -- K1 exponent -> kxh = 8924 * 2^e
                    when 50 =>
                        if z_neg = '0' then z_e <= signed(resize(f_acc(17 downto 8), 12)); end if;
                    when 51 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 53 => z_mt <= r_ex_q;
                    when 74 =>                                -- kxh = 8924 mt >> (15 - sh), sh = -1 .. 1
                        case to_integer(z_sh) is
                            when -1     => z_kxh <= f_acc(32 downto 16);
                            when 0      => z_kxh <= f_acc(31 downto 15);
                            when others => z_kxh <= f_acc(30 downto 14);
                        end case;
                        v_d := signed(resize(s_g2(12 downto 3), 11)) - 512;
                        z_neg <= v_d(10);
                        if v_d(10) = '1' then z_a <= (others => '0'); z_e <= resize(shift_right(v_d, 1), 12);
                        else z_a <= resize(unsigned(v_d(9 downto 0)), 16); end if;
                    when 75 =>
                        case to_integer(s_xs) is
                            when 4      => c_kstep <= shift_left(z_kxh, 2);
                            when 3      => c_kstep <= shift_left(z_kxh, 1) + z_kxh;
                            when others => c_kstep <= shift_left(z_kxh, 1);
                        end case;
                    -- K2 exponent -> kyl = 31987 * 2^e
                    when 98 =>
                        if z_neg = '0' then z_e <= signed(resize(f_acc(17 downto 8), 12)); end if;
                    when 99  => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 101 => z_mt <= r_ex_q;
                    when 122 =>
                        case to_integer(z_sh) is
                            when -1     => z_kyl <= f_acc(33 downto 16);
                            when 0      => z_kyl <= f_acc(32 downto 15);
                            when others => z_kyl <= f_acc(31 downto 14);
                        end case;
                    when 146 => c_kys <= f_acc(27 downto 8);                                -- kyl * ys >> 8
                    when 170 => c_phy0 <= to_unsigned(C_PHY_C, 24) - f_acc(23 downto 0);    -- - 540 kyl
                    when 194 =>
                        c_phx0 <= to_unsigned(C_PHX_C, 24) - f_acc(23 downto 0);           -- - (1920 - x0) kxh
                        -- K5: rings exponent e = (k - 512) * 166 / 256 (+-1.3 octaves)
                        v_d := signed(resize(s_g5(12 downto 3), 11)) - 512;
                        z_neg <= v_d(10);
                        if v_d(10) = '1' then z_a <= resize(unsigned(-v_d), 16);
                        else z_a <= resize(unsigned(v_d), 16); end if;
                    when 218 =>
                        if z_neg = '1' then z_e <= -signed(resize(f_acc(17 downto 8), 12));
                        else z_e <= signed(resize(f_acc(17 downto 8), 12)); end if;
                    when 219 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 221 => z_mt <= r_ex_q;
                    when 242 =>                               -- k1q = 150 mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when -2     => c_k1q <= f_acc(25 downto 17);
                            when -1     => c_k1q <= f_acc(24 downto 16);
                            when 0      => c_k1q <= f_acc(23 downto 15);
                            when others => c_k1q <= f_acc(22 downto 14);
                        end case;
                    when 266 =>                               -- g2q = 209 mt >> (16 - sh)
                        case to_integer(z_sh) is
                            when -2     => c_g2q <= f_acc(26 downto 18);
                            when -1     => c_g2q <= f_acc(25 downto 17);
                            when 0      => c_g2q <= f_acc(24 downto 16);
                            when others => c_g2q <= f_acc(23 downto 15);
                        end case;
                        -- P12: L0 term (p - 512) * 204 / 256
                        v_d := signed(resize(s_gp(12 downto 3), 11)) - 512;
                        z_neg <= v_d(10);
                        if v_d(10) = '1' then z_a <= resize(unsigned(-v_d), 16);
                        else z_a <= resize(unsigned(v_d), 16); end if;
                    when 290 =>
                        if z_neg = '1' then z_l0p <= -signed(resize(f_acc(17 downto 8), 13));
                        else z_l0p <= signed(resize(f_acc(17 downto 8), 13)); end if;
                        z_neg <= s_lfo(9);
                        if s_lfo(9) = '1' then z_a <= resize(unsigned(-s_lfo), 16);
                        else z_a <= resize(unsigned(s_lfo), 16); end if;
                    when 314 =>
                        if z_neg = '1' then z_lfa <= -signed(resize(f_acc(16 downto 8), 11));
                        else z_lfa <= signed(resize(f_acc(16 downto 8), 11)); end if;
                    when 338 =>
                        if z_neg = '1' then z_lfb <= -signed(resize(f_acc(19 downto 8), 13));
                        else z_lfb <= signed(resize(f_acc(19 downto 8), 13)); end if;
                        -- K3 hearts: +-8 deadband
                        v_d := signed(resize(s_g3(12 downto 3), 11)) - 512;
                        if v_d > -8 and v_d < 8 then z_d3 <= (others => '0');
                        elsif v_d > 0 then z_d3 <= v_d - 8;
                        else z_d3 <= v_d + 8; end if;
                    when 339 =>
                        c_l0 <= to_signed(C_LN174, 13) + z_l0p + resize(z_lfa, 13);
                        -- off = -d3 * 5 - (p - 512) * 3 + lfo * 819 / 256
                        v_d := signed(resize(s_gp(12 downto 3), 11)) - 512;
                        v_s := z_lfb - resize(shift_left(resize(z_d3, 15), 2) + resize(z_d3, 15), 15)
                               - resize(shift_left(resize(v_d, 15), 1) + resize(v_d, 15), 15);
                        if v_s > 4095 then c_off <= to_signed(4095, 13);
                        elsif v_s < -4095 then c_off <= to_signed(-4095, 13);
                        else c_off <= v_s(12 downto 0); end if;
                    when 362 =>                               -- bend gain = k6^2 >> 13 (Osc: x16, centred)
                        if s_src = '1' then
                            if f_acc(20 downto 17) /= "0000" then c_gq <= (others => '1'); else c_gq <= f_acc(16 downto 9); end if;
                        else
                            c_gq <= f_acc(20 downto 13);
                        end if;
                    when 386 => c_lstep <= f_acc(20 downto 9);                 -- flap step = k4^2 >> 9
                    when 410 =>                               -- Osc: subtract its mid level (111) so it bends both ways
                        if s_src = '1' then c_moff <= f_acc(14 downto 7); else c_moff <= (others => '0'); end if;
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
    -- Generated avid (parity-locked start 3/4 clocks after the source's avid rise, field-latched width)
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
    -- Source side: oscillator (2-D triangle lattice, drifting), line-buffer write of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_s9 : unsigned(8 downto 0);
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
            -- W1: the source sample; read the vertical IIR state of its column
            if s_src = '1' then w1_l <= s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then w1_l <= v_l - 16; else w1_l <= (others => '0'); end if;
            end if;
            w1_v <= data_in.avid;
            s_fr_ra <= s_wcol;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
            -- W2: EBR read; W3: fabric copy
            w2_l <= w1_l; w2_v <= w1_v;
            w3_l <= w2_l; w3_v <= w2_v; w3_f <= unsigned(s_fr_q);
            -- W4: f = (previous line + new) / 2, written to the line buffer and the IIR state
            s_lb_we <= w3_v;
            s_lb_wa <= s_wcol2;
            v_s9 := resize(w3_f, 9) + resize(w3_l, 9);
            s_lb_wd <= std_logic_vector(v_s9(8 downto 1));
            if w3_v = '1' then s_wcol2 <= s_wcol2 + 1; else s_wcol2 <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0); computes the NEXT line:
    -- v (folded V phase), ln v, the knot's v distance, and the F2 row: for each of the 33 table columns,
    -- lerp the two table rows by the v fraction, add the hearts offset, times the rings gain, clamp; written
    -- as {row(i), row(i+1) - row(i)} into the bank the pixel path is not reading.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_r : signed(15 downto 0);
        variable v_dd : signed(12 downto 0);
        variable v_dc : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;

            rw_we <= '0';
            if e_zk = '1' then
                e_phy <= c_phy0;
                e_lc  <= (others => '0');
            elsif g_start = '1' then
                e_phy <= e_phy + resize(c_kys, 24);
                e_lc  <= (others => '0');
                e_bank <= not e_bank;
                p_bank <= e_bank;
                l_lvl <= resize(e_lv, 13) - c_l0;
                l_bv  <= e_bv;
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    e_v <= f_fold(e_phy);
                when 2 =>
                    r_lv_a <= e_v(14 downto 6);
                    e_fv   <= e_v(10 downto 5);
                    e_tb0  <= shift_left(resize(e_v(14 downto 11), 10), 5) + resize(e_v(14 downto 11), 10);
                    e_t131 <= shift_left(resize(e_v, 22), 7) + shift_left(resize(e_v, 22), 1);
                when 3 =>
                    e_tb1  <= e_tb0 + 33;
                    e_bv   <= resize(shift_right(e_t131 + resize(e_v, 22), 14), 9);
                    e_i    <= (others => '0');
                    e_ra   <= '1';
                    e_havep <= '0';
                when 5 =>
                    e_lv <= r_lv_q;
                when others => null;
            end case;

            -- table reads: A node at even clocks, B node at odd clocks, i = 0 .. 32
            e_va <= e_va(1 downto 0) & '0';
            if e_lc >= 4 and e_lc < 70 then
                if e_ra = '1' then
                    r_tb_a <= e_tb0 + resize(e_i, 10);
                    e_va(0) <= '1';
                else
                    r_tb_a <= e_tb1 + resize(e_i, 10);
                    e_i <= e_i + 1;
                end if;
                e_ra <= not e_ra;
            end if;
            -- r_tb_q holds A two clocks after e_va(0) was set (e_va(1)), B one clock later (e_va(2))
            if e_va(1) = '1' then e_tA <= r_tb_q; end if;
            e_x1v <= e_va(2);
            if e_va(2) = '1' then
                e_tB <= r_tb_q;
                e_x1d <= resize(r_tb_q, 9) - resize(e_tA, 9);
                e_x1a <= e_tA;
            end if;
            -- x2: (tB - tA) * fv
            e_x2v <= e_x1v;
            e_x2p <= resize(e_x1d * signed('0' & e_fv), 16);
            e_x2a <= e_x1a;
            -- x3: lerp, Q6
            e_x3v <= e_x2v;
            e_x3l <= resize(shift_left(resize(e_x2a, 15), 6), 15) + resize(e_x2p, 15);
            -- x4: + offset
            e_x4v <= e_x3v;
            e_x4s <= resize(e_x3l, 16) + resize(c_off, 16);
            -- x5: * rings gain, split lo / hi (16 x 5, 16 x 4)
            e_x5v <= e_x4v;
            e_x5a <= resize(e_x4s * signed(resize(c_g2q(4 downto 0), 6)), 22);
            e_x5b <= resize(e_x4s * signed(resize(c_g2q(8 downto 5), 5)), 21);
            -- x6: sum
            e_x6v <= e_x5v;
            e_x6p <= resize(e_x5a, 26) + shift_left(resize(e_x5b, 26), 5);
            -- x7: >> 10, clamp +-2047
            e_x7v <= e_x6v;
            v_r := e_x6p(25 downto 10);
            if v_r > 2047 then e_x7r <= to_signed(2047, 12);
            elsif v_r < -2047 then e_x7r <= to_signed(-2047, 12);
            else e_x7r <= v_r(11 downto 0); end if;
            -- x8: d = row - prev, clamp +-511; write entry (i - 1) = {prev, d}
            if e_x7v = '1' then
                e_prev <= e_x7r;
                e_havep <= '1';
                if e_havep = '1' then
                    v_dd := resize(e_x7r, 13) - resize(e_prev, 13);
                    if v_dd > 511 then v_dc := to_signed(511, 10);
                    elsif v_dd < -511 then v_dc := to_signed(-511, 10);
                    else v_dc := v_dd(9 downto 0); end if;
                    rw_we <= '1';
                    rw_wa <= "00" & e_bank & e_wi(4 downto 0);
                    rw_wd <= "0000000000" & std_logic_vector(e_prev) & std_logic_vector(v_dc);
                    e_wi <= e_wi + 1;
                end if;
            end if;
            if e_lc = 3 then e_wi <= (others => '0'); end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: H phase per pixel
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_phx <= c_phx0;
                b_px  <= (others => '0');
            elsif g_avid = '1' then
                b_phx <= b_phx + resize(c_kstep, 24);
                b_px  <= b_px + 1;
            else
                b_px  <= (others => '0');
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam phase of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_u  : unsigned(14 downto 0);
        variable v_ut : unsigned(14 downto 0);
        variable v_tw : unsigned(23 downto 0);
        variable v_f1 : signed(15 downto 0);
        variable v_x  : signed(15 downto 0);
        variable v_x2 : signed(11 downto 0);
        variable v_a  : unsigned(15 downto 0);
        variable v_b  : unsigned(11 downto 0);
        variable v_g  : unsigned(18 downto 0);
        variable v_h  : unsigned(9 downto 0);
        variable v_e  : unsigned(6 downto 0);
        variable v_d9 : signed(8 downto 0);
        variable v_hd, v_hs : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: fold; LP and row addresses
            v_u := f_fold(b_phx);
            v_tw := b_phx(22 downto 0) & '0';
            if s_twin = '1' then v_ut := f_fold(v_tw); else v_ut := v_u; end if;
            q2_u <= v_u;
            r_lp_a <= v_u(14 downto 5);
            rw_ra <= "00" & p_bank & v_ut(14 downto 10);
            q2_fu <= v_ut(9 downto 4);
            -- 3: memories; u * 235 partials
            q3_u <= q2_u; q3_fu <= q2_fu;
            q3_t1 <= shift_left(resize(q2_u, 23), 8) - shift_left(resize(q2_u, 23), 4);
            q3_t2 <= resize(shift_left(resize(q2_u, 17), 2), 17) + resize(q2_u, 17);
            -- 4: fabric copies; a = u * 235 >> 14 (knot x distance, cell px)
            q4_lp <= r_lp_q;
            q4_row <= signed(rw_q(21 downto 10));
            q4_d <= signed(rw_q(9 downto 0));
            q4_fu <= q3_fu;
            q4_a <= resize(shift_right(q3_t1 - resize(q3_t2, 23), 14), 9);
            -- horizontal IIR on the camera / Osc luma: h += (f - h) / 4 (tracks the line's first value off avid)
            if s_gav_sr(1) = '0' then
                q4_lm <= unsigned(s_lb_q);
            else
                v_hd := signed(resize(unsigned(s_lb_q), 10)) - signed(resize(q4_lm, 10));
                v_hs := signed(resize(q4_lm, 10)) + shift_right(v_hd, 2);
                q4_lm <= unsigned(v_hs(7 downto 0));
            end if;
            -- 5: L = ln v - L0 + (-ln sin); d * fu; knot max / min; camera * gain
            q5_l <= l_lvl + signed(resize(q4_lp, 13));
            q5_dfu <= resize(q4_d * signed(resize(q4_fu, 7)), 17);
            q5_row <= q4_row;
            if q4_a > l_bv then q5_mx <= q4_a; q5_mn <= l_bv; else q5_mx <= l_bv; q5_mn <= q4_a; end if;
            q5_m <= q4_lm * c_gq;
            -- 6: F1 = L * k1 split lo / hi; F2 (Q2); 3/8 min; m
            q6_f1a <= resize(q5_l * signed(resize(c_k1q(4 downto 0), 6)), 19);
            q6_f1b <= resize(q5_l * signed(resize(c_k1q(8 downto 5), 5)), 18);
            q6_f2q <= resize(q5_row, 13) + resize(shift_right(q5_dfu, 6), 13);
            q6_mx <= q5_mx;
            q6_m38 <= resize(shift_right(q5_mn, 2), 8) + resize(shift_right(q5_mn, 3), 8);
            q6_m <= signed(resize(q5_m(15 downto 7), 10)) - signed(resize(c_moff, 10));
            -- 7: F1 sum (Q8); F2 + camera; knot radius
            q7_f1p <= resize(q6_f1a, 23) + shift_left(resize(q6_f1b, 23), 5);
            q7_f2m <= resize(shift_right(q6_f2q, 2), 12) + resize(q6_m, 12);
            v_a := resize(q6_mx, 16) + resize(q6_m38, 16);
            if v_a > 511 then q7_rr <= (others => '1'); else q7_rr <= v_a(8 downto 0); end if;
            q7_m <= q6_m;
            -- 8: F1 + camera; |F2| (clamp 255, one's complement)
            q8_f1m <= resize(shift_right(q7_f1p, 8), 16) + resize(q7_m, 16);
            if q7_f2m(11) = '1' then v_x2 := not q7_f2m; else v_x2 := q7_f2m; end if;
            if v_x2 > 255 then q8_b <= (others => '1'); else q8_b <= unsigned(v_x2(7 downto 0)); end if;
            q8_rr <= q7_rr;
            -- 9: |F1| (clamp 1023, one's complement); rr * 233 partials
            if q8_f1m(15) = '1' then v_f1 := not q8_f1m; else v_f1 := q8_f1m; end if;
            if v_f1 > 1023 then q9_a <= (others => '1'); else q9_a <= unsigned(v_f1(9 downto 0)); end if;
            q9_rr <= q8_rr;
            q9_ta <= shift_left(resize(q8_rr, 17), 8) - shift_left(resize(q8_rr, 17), 5);
            q9_tb <= shift_left(resize(q8_rr, 13), 3) + resize(q8_rr, 13);
            q9_b <= q8_b;
            -- 10: knot: rr * |F1| split lo / hi; Ac = min(|F1|, 104), 39 - Ac; f8 pre
            q10_pl <= q9_rr * q9_a(4 downto 0);
            q10_ph <= q9_rr * q9_a(9 downto 5);
            if q9_a > 104 then q10_ac <= to_unsigned(104, 7); else q10_ac <= q9_a(6 downto 0); end if;
            if q9_a > 104 then q10_c39 <= to_signed(39 - 104, 8); else q10_c39 <= 39 - signed(resize(q9_a(6 downto 0), 8)); end if;
            v_g := resize(q9_ta, 19) + resize(q9_tb, 19);
            q10_f8p <= v_g(16 downto 8);
            q10_b <= q9_b;
            -- 11: prod; f8 = max(0, 255 - rr * 233 >> 8)
            q11_prod <= resize(q10_pl, 19) + shift_left(resize(q10_ph, 19), 5);
            if q10_f8p > 255 then q11_f8 <= (others => '0'); else q11_f8 <= 255 - q10_f8p(7 downto 0); end if;
            q11_ac <= q10_ac; q11_c39 <= q10_c39; q11_b <= q10_b;
            -- 12: gl - Ac = (39 - Ac) + min(24, prod >> 9)
            if q11_prod(18 downto 9) > 24 then
                q12_df <= q11_c39 + 24;
            else
                q12_df <= q11_c39 + signed(resize(q11_prod(13 downto 9), 8));
            end if;
            q12_f8 <= q11_f8; q12_ac <= q11_ac; q12_b <= q11_b;
            -- 13: f8 * (gl - Ac)
            q13_fd <= signed(resize(q12_f8, 9)) * q12_df;
            q13_ac <= q12_ac; q13_b <= q12_b;
            -- 14: g1 = Ac + f8 (gl - Ac) / 256
            v_x := signed(resize(q13_ac, 16)) + resize(shift_right(q13_fd, 8), 16);
            if v_x < 0 then q14_g1 <= (others => '0'); else q14_g1 <= unsigned(v_x(7 downto 0)); end if;
            q14_b <= q13_b;
            -- 15: Darker (min of the two fields) or Interfere (|difference|, one's complement)
            v_d9 := signed(resize(q14_g1, 9)) - signed(resize(q14_b, 9));
            if s_mix = '1' then
                if v_d9(8) = '1' then q15_ag <= unsigned(not v_d9(7 downto 0)); else q15_ag <= unsigned(v_d9(7 downto 0)); end if;
            elsif q14_g1 < q14_b then
                q15_ag <= q14_g1;
            else
                q15_ag <= q14_b;
            end if;
            -- 16: palette index (Echo: the ring sequence repeats every 128 G units, two cycles)
            if s_echo = '1' then
                if q15_ag > 224 then v_h := to_unsigned(224, 10); else v_h := resize(q15_ag, 10); end if;
                if v_h(6) = '1' then v_e := not v_h(6 downto 0); else v_e := v_h(6 downto 0); end if;
            else
                if q15_ag > 127 then v_e := (others => '1'); else v_e := q15_ag(6 downto 0); end if;
            end if;
            r_pl_a <= s_pal & v_e;
            -- 17: palette ROM (EBR); 18: fabric copy
            q18_c <= r_pl_q;
            -- 19: compose; the palette black outside the generated avid
            if s_gav_sr(16) = '0' then
                s_y19 <= c_ky; s_u19 <= c_ku; s_v19 <= c_kv;
            else
                s_y19 <= unsigned(q18_c(31 downto 22)); s_u19 <= unsigned(q18_c(21 downto 12)); s_v19 <= unsigned(q18_c(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(16);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (each two adjacent 8-sums;
    -- the still's softness and the HD HDMI bar fix), luma 13/16 and the blanking gate.  The running sums snap
    -- back to the palette black once 8 blank samples are in (resync).
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- box A: stage 20 sums, stage 21 output
            s_cay(0) <= s_y19; s_cau(0) <= s_u19; s_cav(0) <= s_v19;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= c_ky & "000"; s_bau <= c_ku & "000"; s_bav <= c_kv & "000";
            else
                s_bay <= s_bay + s_y19 - s_cay(7);
                s_bau <= s_bau + s_u19 - s_cau(7);
                s_bav <= s_bav + s_v19 - s_cav(7);
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(18) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y21 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u21 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v21 <= v_sc(13 downto 4);
            else
                s_y21 <= c_ky; s_u21 <= c_ku; s_v21 <= c_kv;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(18);
            -- box B: stage 22 sums, stage 23 output
            s_cby(0) <= s_y21; s_cbu(0) <= s_u21; s_cbv(0) <= s_v21;
            for i in 1 to 7 loop s_cby(i) <= s_cby(i - 1); s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bby <= c_ky & "000"; s_bbu <= c_ku & "000"; s_bbv <= c_kv & "000";
            else
                s_bby <= s_bby + s_y21 - s_cby(7);
                s_bbu <= s_bbu + s_u21 - s_cbu(7);
                s_bbv <= s_bbv + s_v21 - s_cbv(7);
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y23x <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u23x <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v23x <= v_sc(13 downto 4);
            -- 24: luma 13/16
            if s_y23x > 64 then v_dl := s_y23x - 64; else v_dl := (others => '0'); end if;
            s_y24 <= s_y23x - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u24 <= s_u23x; s_v24 <= s_v23x;
            -- 25: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y25 <= s_y24; s_u25 <= s_u24; s_v25 <= s_v24;
            else
                s_y25 <= to_unsigned(64, 10); s_u25 <= to_unsigned(512, 10); s_v25 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y25);
    data_out.u       <= std_logic_vector(s_u25);
    data_out.v       <= std_logic_vector(s_v25);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture butterfly;
