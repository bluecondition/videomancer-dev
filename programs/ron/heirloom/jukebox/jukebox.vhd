-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Jukebox v0.5.0 -- Heirloom #16, rebuilt from the photo WIN_20230315_00_28_42.
--
-- The photo: a wall of tall rainbow capsules, 9.15 columns by 2.08 rows.  In
-- each column the contour bands run flat across and bend into rounded noses
-- at the column's left side, right after a hard cut; violet/blue cores on the
-- row centres (lines 259 / 779), red/yellow seams between the rows.  The
-- columns alternate in two kinds (red-topped / yellow-topped, half a band
-- apart).
--
-- The patch, as measured (jukebox.py has every constant and table):
--   vertical    dn = |y - row centre| / half-row; V = 0.0078 + 3.625 dn + 1.711 dn^2
--               fold units, with a 0.77 dip on the seam.
--   horizontal  a 2-column oscillator; every column starts with a reset
--               transient.  Its measured waveforms (column tables, 2 kinds x
--               256 positions) feed the three colour channels:
--   G  = 10 + 233 (o + c tri(a Bands (V + etch luma) + E - t))   the rainbow contours
--   R  = r + 36.5 (V + tint) - 0.09 G                            red rises to the seams
--   B  = 184 + bo - 79 bg (V - tint) - 0.086 G                   blue fills the cores
--   then BT.601 to 10-bit YUV (luma pre-scaled 16/13).
--
--   K1  Columns   capsule columns across (2 .. 24; 9.15 = the photo)
--   K2  Rows      capsule rows per frame height (1 .. 6; 2.08 = the photo)
--   K3  Nose      how far the rounded noses reach into each column
--   K4  Speed     bipolar band flow (centre +-8 still; right = bands stream out of the cores)
--   K5  Tint      red / blue balance of the capsules (centre = the photo)
--   K6  Etch      the picture's luma pushed into the contours
--   S7  Mirror    every other column reversed: closed capsules
--   S8  Pairs     On = the photo's two alternating column kinds / Off = all alike
--   S9  Palette   Photo / Swap (red and blue exchanged: red cores, blue seams)
--   S10 Stack     Mirror (the photo) / Saw (rows repeat, cut flat at every seam)
--   S11 Source    Video / Osc
--   P12 BANDS     contour bands per capsule: 0 = one soft gradient, 100 % = fine filigree
--
-- Engine: a vblank-zone sequencer (one serial multiplier, fixed slots, a shared
-- glide) sets the per-field constants; a per-line engine (one serial
-- multiplier, kicked at each line start, computes the NEXT line) sets the
-- vertical field values; the beam runs the 2-column phase.  Column tables and
-- the colour matrix live in block RAM.  Generated avid (parity-locked);
-- output conditioning (chroma 9-tap, luma [1 2 1], luma 13/16) before the
-- blanking gate.  Latency 28 clocks in every mode.  Colours in the hardware
-- convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.jukebox_pkg.all;
use work.all;

architecture jukebox of program_top is
    constant C_LAT  : integer := 28;          -- = processing_delay_clks
    constant C_HC   : integer := 1175078;     -- 2-column phase at the screen centre (Q24)
    constant C_VPH0 : integer := 8406359;     -- vertical phase of the first line (Q24)

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
    -- width class: 0 = 1920, 1 = 1280, 2 = 720
    function f_cls(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(2, 2);
            when 5 | 6 | 14     => return to_unsigned(1, 2);
            when others         => return to_unsigned(0, 2);
        end case;
    end function;
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0, of the line being drawn
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');


    -- oscillator source (Source = Osc)
    signal s_ox : unsigned(9 downto 0) := (others => '0');
    signal s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_oty : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_clsr : unsigned(1 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(8 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_half : unsigned(10 downto 0) := to_unsigned(960, 11);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(583 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(475 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(460 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(406 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');                 -- shared glide: selected knob
    signal s_gd : signed(14 downto 0) := (others => '0');                   -- and its step
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(600, 10);
    signal s_vm : unsigned(9 downto 0) := (others => '0');                  -- |speed| - 8
    signal s_vneg, s_vstill : std_logic := '1';
    signal s_mirror, s_pairsoff, s_swap, s_saw, s_src : std_logic := '0';
    signal s_cls  : unsigned(1 downto 0) := (others => '0');

    -- per-field constants
    signal s_tmp   : unsigned(23 downto 0) := (others => '0');
    signal s_k1sq, s_k2sq, s_psq, s_k6sq : unsigned(19 downto 0) := (others => '0');
    signal s_hd    : unsigned(16 downto 0) := to_unsigned(39955, 17);
    signal s_hstep : unsigned(17 downto 0) := to_unsigned(39955, 18);
    signal s_hp0   : unsigned(23 downto 0) := (others => '0');
    signal s_rq    : unsigned(12 downto 0) := to_unsigned(2127, 13);
    signal s_vstep : unsigned(19 downto 0) := to_unsigned(32270, 20);
    signal s_bands : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal s_etch  : unsigned(9 downto 0) := (others => '0');
    signal s_be    : unsigned(11 downto 0) := (others => '0');
    signal s_vsq   : unsigned(17 downto 0) := (others => '0');
    signal s_spd   : signed(15 downto 0) := (others => '0');
    signal s_t17   : unsigned(16 downto 0) := (others => '0');
    signal s_t9    : unsigned(8 downto 0) := (others => '0');
    signal s_tv    : unsigned(9 downto 0) := (others => '0');                -- |tint knob| - 8
    signal s_tneg, s_tstill : std_logic := '1';
    signal s_tm    : unsigned(13 downto 0) := (others => '0');
    signal s_tint  : signed(9 downto 0) := (others => '0');
    signal s_nose  : unsigned(7 downto 0) := to_unsigned(64, 8);

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');

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
    signal e_ph   : unsigned(23 downto 0) := to_unsigned(C_VPH0, 24);
    signal e_dn   : unsigned(11 downto 0) := (others => '0');
    signal e_dn2  : unsigned(11 downto 0) := (others => '0');
    signal e_t1   : unsigned(21 downto 0) := (others => '0');               -- 928 dn
    signal e_v2   : unsigned(20 downto 0) := (others => '0');               -- 438 dn^2
    signal e_w    : unsigned(12 downto 0) := (others => '0');
    signal e_x1   : unsigned(23 downto 0) := (others => '0');
    signal e_wx   : unsigned(12 downto 0) := (others => '0');
    signal e_dp   : unsigned(20 downto 0) := (others => '0');
    signal e_dip  : unsigned(7 downto 0) := (others => '0');
    signal e_dq   : unsigned(15 downto 0) := (others => '0');
    signal e_v0, e_vg, e_vr : unsigned(10 downto 0) := (others => '0');
    signal e_rv1, e_bv1 : signed(11 downto 0) := (others => '0');
    signal e_rv2, e_bv2 : signed(19 downto 0) := (others => '0');
    signal m_a    : unsigned(23 downto 0) := (others => '0');
    signal m_b    : unsigned(11 downto 0) := (others => '0');
    signal m_acc  : unsigned(23 downto 0) := (others => '0');
    signal n_bvg  : unsigned(12 downto 0) := (others => '0');
    signal n_rv, n_bv : signed(9 downto 0) := (others => '0');
    signal l_bvg  : unsigned(12 downto 0) := (others => '0');
    signal l_rv, l_bv : signed(9 downto 0) := (others => '0');

    -- beam
    signal b_hp : unsigned(23 downto 0) := (others => '0');
    signal b_px : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    signal p1_typ : std_logic := '0';
    signal p1_pos : unsigned(7 downto 0) := (others => '0');
    signal p2_typ, p2_ge : std_logic := '0';
    signal p2_pos, p2_d : unsigned(7 downto 0) := (others => '0');
    signal p3_typ, p3_ge : std_logic := '0';
    signal p3_pos : unsigned(7 downto 0) := (others => '0');
    signal p3_nl, p3_nh : unsigned(11 downto 0) := (others => '0');
    signal p3_y8  : unsigned(7 downto 0) := (others => '0');
    signal p4_ad  : unsigned(8 downto 0) := (others => '0');
    signal p4_bla, p4_blb : unsigned(15 downto 0) := (others => '0');
    signal p5_bl  : unsigned(11 downto 0) := (others => '0');
    signal t5_0, t5_1, t5_2, t5_3 : std_logic_vector(15 downto 0) := (others => '0');
    signal t6_0, t6_1, t6_2, t6_3 : std_logic_vector(15 downto 0) := (others => '0');
    signal p6_gs  : unsigned(13 downto 0) := (others => '0');
    signal p7_gal, p7_gah : unsigned(17 downto 0) := (others => '0');
    signal p7_bbl, p7_bbh : signed(14 downto 0) := (others => '0');
    signal p7_e   : unsigned(8 downto 0) := (others => '0');
    signal p7_c, p8_c, p9_c : unsigned(7 downto 0) := (others => '0');
    signal p7_o, p8_o, p9_o, p10_o, p11_o : signed(7 downto 0) := (others => '0');
    signal p7_r, p8_r, p9_r, p10_r, p11_r, p12_r, p13_r, p14_r, p15_r : unsigned(7 downto 0) := (others => '0');
    signal p7_bo, p8_bo, p9_bo, p10_bo, p11_bo, p12_bo, p13_bo, p14_bo, p15_bo : unsigned(7 downto 0) := (others => '0');
    signal p8_ga9 : unsigned(8 downto 0) := (others => '0');
    signal p8_e   : unsigned(8 downto 0) := (others => '0');
    signal p8_bb, p9_bb, p10_bb, p11_bb, p12_bb, p13_bb, p14_bb, p15_bb : signed(10 downto 0) := (others => '0');
    signal p9_tri : unsigned(7 downto 0) := (others => '0');
    signal p10_cl, p10_ch : unsigned(11 downto 0) := (others => '0');
    signal p11_ct : unsigned(8 downto 0) := (others => '0');
    signal p12_g8, p13_g8 : signed(9 downto 0) := (others => '0');
    signal p13_gm : signed(14 downto 0) := (others => '0');
    signal p14_g, p15_g : unsigned(7 downto 0) := (others => '0');
    signal p15_gr, p15_gb : unsigned(12 downto 0) := (others => '0');
    signal p16_r, p16_g, p16_b : unsigned(7 downto 0) := (others => '0');
    signal p17_ar, p17_ag, p17_ab : unsigned(7 downto 0) := (others => '0');
    signal p18_qr, p18_qg, p18_qb : std_logic_vector(31 downto 0) := (others => '0');
    signal p19_qr, p19_qg, p19_qb : std_logic_vector(31 downto 0) := (others => '0');
    signal p20_ya, p20_yb : unsigned(11 downto 0) := (others => '0');
    signal p20_ua, p20_ub, p20_va, p20_vb : signed(11 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y22, s_y23, s_y24, s_y25, s_y26, s_y27x, s_yf27 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept)
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer.  Every zone-clock step is one
    -- small operation (the zone is quasi-static but timed at the pixel clock).
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_k  : integer range 0 to 63;
    begin
        if rising_edge(clk) then
            s_tid  <= registers_in(8)(3 downto 0);
            s_ys   <= f_vstep_y(s_tid);
            s_clsr <= f_cls(s_tid);

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
            elsif s_zc /= 511 then
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
            -- speed: |v| - 8 and the still zone, registered free-running
            v_s := signed(resize(s_k4, 11)) - 512;
            if v_s > -9 and v_s < 9 then s_vstill <= '1'; else s_vstill <= '0'; end if;
            s_vneg <= v_s(10);
            if v_s(10) = '1' then v_m := unsigned(not v_s(9 downto 0)) - 7; else v_m := unsigned(v_s(9 downto 0)) - 8; end if;
            s_vm <= v_m;
            -- tint: |v| - 8 of the glided K5 and the centre zone
            v_s := signed(resize(s_g5(12 downto 3), 11)) - 512;
            if v_s > -9 and v_s < 9 then s_tstill <= '1'; else s_tstill <= '0'; end if;
            s_tneg <= v_s(10);
            if v_s(10) = '1' then v_m := unsigned(not v_s(9 downto 0)) - 7; else v_m := unsigned(v_s(9 downto 0)) - 8; end if;
            s_tv <= v_m;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_mirror   <= registers_in(6)(0);
                s_pairsoff <= registers_in(6)(1);
                s_swap     <= registers_in(6)(2);
                s_saw      <= registers_in(6)(3);
                s_src      <= registers_in(6)(4);
                s_cls      <= s_clsr;
            end if;
            s_half <= s_w(11 downto 1);
            g_wm1 <= s_w - 1;

            -- shared glide (1/8 of the gap per field, +-2 deadband): knob k on
            -- zone clocks 3k+1 (select, distance), 3k+2 (step), 3k+3 (update)
            if s_zone = '1' and s_zc >= 1 and s_zc <= 18 then
                case to_integer(s_zc) is
                    when 1  => s_gm <= s_g1; s_gd <= signed(resize(unsigned(registers_in(0)) & "000", 15)) - signed(resize(s_g1, 15));
                    when 4  => s_gm <= s_g2; s_gd <= signed(resize(unsigned(registers_in(1)) & "000", 15)) - signed(resize(s_g2, 15));
                    when 7  => s_gm <= s_g3; s_gd <= signed(resize(unsigned(registers_in(2)) & "000", 15)) - signed(resize(s_g3, 15));
                    when 10 => s_gm <= s_g5; s_gd <= signed(resize(unsigned(registers_in(4)) & "000", 15)) - signed(resize(s_g5, 15));
                    when 13 => s_gm <= s_g6; s_gd <= signed(resize(unsigned(registers_in(5)) & "000", 15)) - signed(resize(s_g6, 15));
                    when 16 => s_gm <= s_gp; s_gd <= signed(resize(unsigned(registers_in(7)) & "000", 15)) - signed(resize(s_gp, 15));
                    when 2 | 5 | 8 | 11 | 14 | 17 =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                    when 3  => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 6  => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 9  => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 12 => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 15 => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                end case;
            end if;

            -- serial products: one slot every 24 zone clocks (k = 1, 4, 7 ..
            -- 40): load at 8k, 16 iterations, result stable from 8k + 17.
            v_k := to_integer(s_zc(8 downto 3));
            if s_zc(2 downto 0) = "000" and (v_k = 1 or v_k = 4 or v_k = 7 or v_k = 10 or v_k = 13 or v_k = 16 or
               v_k = 19 or v_k = 22 or v_k = 25 or v_k = 28 or v_k = 31 or v_k = 34 or v_k = 37 or v_k = 40) then
                f_acc <= (others => '0');
                case v_k is
                    when 1  => f_a <= resize(s_g1(12 downto 3), 36);  f_b <= resize(s_g1(12 downto 3), 16);
                    when 4  => f_a <= to_unsigned(96308, 36);          f_b <= s_k1sq(19 downto 4);
                    when 7  => f_a <= resize(s_hstep, 36);             f_b <= resize(s_half, 16);
                    when 10 => f_a <= resize(s_g2(12 downto 3), 36);  f_b <= resize(s_g2(12 downto 3), 16);
                    when 13 => f_a <= to_unsigned(5130, 36);           f_b <= s_k2sq(19 downto 4);
                    when 16 => f_a <= resize(s_rq, 36);                f_b <= resize(s_ys, 16);
                    when 19 => f_a <= resize(s_tmp, 36);               f_b <= to_unsigned(3884, 16);
                    when 22 => f_a <= resize(s_gp(12 downto 3), 36);  f_b <= resize(s_gp(12 downto 3), 16);
                    when 25 => f_a <= resize(s_psq, 36);               f_b <= to_unsigned(1222, 16);
                    when 28 => f_a <= resize(s_g6(12 downto 3), 36);  f_b <= resize(s_g6(12 downto 3), 16);
                    when 31 => f_a <= resize(s_k6sq, 36);              f_b <= to_unsigned(769, 16);
                    when 34 => f_a <= resize(s_bands, 36);             f_b <= resize(s_etch, 16);
                    when 37 => f_a <= resize(s_vm, 36);                f_b <= resize(s_vm, 16);
                    when others => f_a <= resize(s_vsq, 36);           f_b <= to_unsigned(66, 16);
                end case;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;
            -- stores and shift-adds (one operation per zone clock)
            case to_integer(s_zc) is
                when 26  => s_k1sq <= f_acc(19 downto 0);
                when 50  => s_hd <= to_unsigned(8738, 17) + f_acc(32 downto 16);
                when 51  =>
                    case to_integer(s_cls) is
                        when 1      => s_hstep <= resize(s_hd, 18) + resize(s_hd(16 downto 1), 18);
                        when 2      => s_hstep <= s_hd & '0';
                        when others => s_hstep <= resize(s_hd, 18);
                    end case;
                when 74  => s_hp0 <= to_unsigned(C_HC, 24) - f_acc(23 downto 0);
                when 98  => s_k2sq <= f_acc(19 downto 0);
                when 122 => s_rq <= to_unsigned(1024, 13) + f_acc(28 downto 16);
                when 146 => s_tmp <= f_acc(23 downto 0);
                when 170 => s_vstep <= f_acc(35 downto 16);
                when 194 => s_psq <= f_acc(19 downto 0);
                when 218 => s_bands <= to_unsigned(64, 11) + f_acc(30 downto 20);
                when 242 => s_k6sq <= f_acc(19 downto 0);
                when 266 => s_etch <= f_acc(29 downto 20);
                when 290 => s_be <= f_acc(19 downto 8);
                when 314 => s_vsq <= f_acc(17 downto 0);
                when 338 =>
                    if s_vstill = '1' then
                        s_spd <= (others => '0');
                    elsif s_vneg = '1' then
                        s_spd <= -signed(resize(f_acc(23 downto 10), 16));
                    else
                        s_spd <= signed(resize(f_acc(23 downto 10), 16));
                    end if;
                when 339 => s_t17 <= unsigned(signed(s_t17) + resize(s_spd, 17));
                when 340 => s_t9 <= s_t17(16 downto 8);
                -- tint = sign ((|v| - 8) 13 >> 5)
                when 341 => s_tm <= shift_left(resize(s_tv, 14), 3) + shift_left(resize(s_tv, 14), 2);
                when 342 => s_tm <= s_tm + resize(s_tv, 14);
                when 343 =>
                    if s_tstill = '1' then
                        s_tint <= (others => '0');
                    elsif s_tneg = '1' then
                        s_tint <= -signed(resize(s_tm(13 downto 5), 10));
                    else
                        s_tint <= signed(resize(s_tm(13 downto 5), 10));
                    end if;
                when 344 => s_nose <= C_JB_NOSE(to_integer(s_g3(12 downto 7)));
                when others => null;
            end case;

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
    -- Source side: oscillator, line-buffer write of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then s_ox <= s_ox + 3; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 5; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zgo = '1' then s_ot <= s_ot + 3; end if;
            s_oty <= to_unsigned(64, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot) & '0', 10)
                     + resize(f_tri8(s_ox(9 downto 2) + s_ot), 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot), 10);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then v_l := s_oty(9 downto 2); else v_l := unsigned(data_in.y(9 downto 2)); end if;
            if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line's vertical field values: dn (the vertical
    -- triangle, or the saw for Stack), V0 = 2 + 928 dn + 438 dn^2 (Q8 fold
    -- units, dn in Q12), the seam dip, Bands x Vg, the red and blue ramps.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_p2 : unsigned(23 downto 0);
        variable v_a  : unsigned(23 downto 0);
        variable v_x  : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 360 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_ph <= to_unsigned(C_VPH0, 24);
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_ph <= e_ph + resize(s_vstep, 24);
                e_lc <= (others => '0');
            elsif e_lc /= 63 then
                e_lc <= e_lc + 1;
            end if;
            -- the pixel path takes the previous computation at the line start
            if g_start = '1' then
                l_bvg <= n_bvg; l_rv <= n_rv; l_bv <= n_bv;
            end if;

            -- multiplier (iterates every clock it is not loaded)
            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(22 downto 0) & '0';
            m_b <= '0' & m_b(11 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    v_p2 := e_ph + to_unsigned(8388608, 24);
                    if s_saw = '1' then
                        e_dn <= v_p2(23 downto 12);
                    else
                        if v_p2(23) = '1' then v_a := '0' & v_p2(22 downto 0);
                        else v_a := to_unsigned(8388608, 24) - ('0' & v_p2(22 downto 0)); end if;
                        if v_a(23) = '1' then e_dn <= (others => '1'); else e_dn <= v_a(22 downto 11); end if;
                    end if;
                when 2 =>
                    m_a <= resize(e_dn, 24); m_b <= e_dn; m_acc <= (others => '0');
                when 3 =>
                    e_t1 <= shift_left(resize(e_dn, 22), 10) - shift_left(resize(e_dn, 22), 6);
                    e_w  <= to_unsigned(4096, 13) - resize(e_dn, 13);
                when 4 =>
                    e_t1 <= e_t1 - shift_left(resize(e_dn, 22), 5);
                    e_x1 <= shift_left(resize(e_w, 24), 11) - shift_left(resize(e_w, 24), 5);
                when 5 =>
                    e_x1 <= e_x1 - resize(e_w, 24);
                when 6 =>
                    v_x := e_x1(23 downto 6);
                    if v_x < 4096 then e_wx <= to_unsigned(4096, 13) - v_x(12 downto 0); else e_wx <= (others => '0'); end if;
                when 7 =>
                    e_dp <= shift_left(resize(e_wx, 21), 7) + shift_left(resize(e_wx, 21), 6);
                when 8 =>
                    e_dp <= e_dp + shift_left(resize(e_wx, 21), 2) + resize(e_wx, 21);
                when 9 =>
                    e_dip <= e_dp(19 downto 12);
                when 10 =>
                    e_dq <= shift_left(resize(e_dip, 16), 6) + shift_left(resize(e_dip, 16), 4);
                when 11 =>
                    e_dq <= e_dq + shift_left(resize(e_dip, 16), 3) + shift_left(resize(e_dip, 16), 1);
                when 15 =>
                    e_dn2 <= m_acc(23 downto 12);
                when 16 =>
                    e_v2 <= shift_left(resize(e_dn2, 21), 9) - shift_left(resize(e_dn2, 21), 6);
                when 17 =>
                    e_v2 <= e_v2 - shift_left(resize(e_dn2, 21), 3) - shift_left(resize(e_dn2, 21), 1);
                when 18 =>
                    e_v0 <= to_unsigned(2, 11) + resize(e_t1(21 downto 12), 11) + resize(e_v2(20 downto 12), 11);
                when 19 =>
                    e_vg <= e_v0 - resize(e_dip, 11);
                    e_vr <= e_v0 - resize(e_dq(15 downto 8), 11);
                when 20 =>
                    m_a <= resize(e_vg, 24); m_b <= '0' & s_bands; m_acc <= (others => '0');
                    e_rv1 <= signed(resize(e_vr, 12)) + resize(s_tint, 12);
                    e_bv1 <= signed(resize(e_v0, 12)) - resize(s_tint, 12);
                when 21 =>
                    e_rv2 <= shift_left(resize(e_rv1, 20), 6) + shift_left(resize(e_rv1, 20), 3);
                    e_bv2 <= shift_left(resize(e_bv1, 20), 6) + shift_left(resize(e_bv1, 20), 4);
                when 22 =>
                    e_rv2 <= e_rv2 + resize(e_rv1, 20);
                    e_bv2 <= e_bv2 - resize(e_bv1, 20);
                when 34 =>
                    n_bvg <= m_acc(20 downto 8);
                    n_rv  <= e_rv2(18 downto 9);
                    n_bv  <= e_bv2(17 downto 8);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the 2-column phase
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_hp <= s_hp0;
                b_px <= (others => '0');
            elsif g_avid = '1' then
                b_hp <= b_hp + resize(s_hstep, 24);
                b_px <= b_px + 1;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Column tables (one registered read each) and colour-matrix ROMs
    ---------------------------------------------------------------------------
    p_tab : process(clk)
    begin
        if rising_edge(clk) then
            t5_0 <= C_JB_T0(to_integer(p4_ad));
            t5_1 <= C_JB_T1(to_integer(p4_ad));
            t5_2 <= C_JB_T2(to_integer(p4_ad));
            t5_3 <= C_JB_T3(to_integer(p4_ad));
            p18_qr <= C_JB_RGB_R(to_integer(p17_ar));
            p18_qg <= C_JB_RGB_G(to_integer(p17_ag));
            p18_qb <= C_JB_RGB_B(to_integer(p17_ab));
        end if;
    end process p_tab;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_pos : unsigned(7 downto 0);
        variable v_n   : unsigned(15 downto 0);
        variable v_pw  : unsigned(9 downto 0);
        variable v_bl  : unsigned(19 downto 0);
        variable v_ga  : unsigned(21 downto 0);
        variable v_bb  : signed(18 downto 0);
        variable v_f   : unsigned(8 downto 0);
        variable v_ct  : unsigned(15 downto 0);
        variable v_g   : signed(11 downto 0);
        variable v_r   : signed(11 downto 0);
        variable v_b   : signed(12 downto 0);
        variable v_y   : unsigned(11 downto 0);
        variable v_u   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: column kind and position (Pairs Off: all kind A; Mirror: B reversed)
            if s_pairsoff = '1' then p1_typ <= '1'; else p1_typ <= b_hp(23); end if;
            v_pos := b_hp(22 downto 15);
            if s_mirror = '1' and b_hp(23) = '0' then v_pos := not v_pos; end if;
            p1_pos <= v_pos;

            -- 2: nose: distance into the tail
            p2_typ <= p1_typ; p2_pos <= p1_pos;
            if p1_pos >= 48 then p2_ge <= '1'; p2_d <= p1_pos - 48; else p2_ge <= '0'; p2_d <= (others => '0'); end if;

            -- 3: tail stretch partial products; luma
            p3_typ <= p2_typ; p3_pos <= p2_pos; p3_ge <= p2_ge;
            p3_nl <= p2_d * s_nose(3 downto 0);
            p3_nh <= p2_d * s_nose(7 downto 4);
            p3_y8 <= unsigned(s_lb_q);

            -- 4: table address; etch partial products
            if p3_ge = '1' then
                v_n := resize(p3_nl, 16) + shift_left(resize(p3_nh, 16), 4);
                v_pw := to_unsigned(48, 10) + resize(v_n(15 downto 6), 10);
                if v_pw > 255 then p4_ad <= p3_typ & x"FF"; else p4_ad <= p3_typ & v_pw(7 downto 0); end if;
            else
                p4_ad <= p3_typ & p3_pos;
            end if;
            p4_bla <= s_be * p3_y8(3 downto 0);
            p4_blb <= s_be * p3_y8(7 downto 4);

            -- 5: tables out of block RAM (p_tab); etch sum
            v_bl := resize(p4_bla, 20) + shift_left(resize(p4_blb, 20), 4);
            p5_bl <= v_bl(19 downto 8);

            -- 6: fabric copies; G-field vertical term
            t6_0 <= t5_0; t6_1 <= t5_1; t6_2 <= t5_2; t6_3 <= t5_3;
            p6_gs <= resize(l_bvg, 14) + resize(p5_bl, 14);

            -- 7: gain partial products (G: a x gs; B: bg x Bv); side values
            p7_gal <= unsigned(t6_1(11 downto 8)) * p6_gs;
            p7_gah <= unsigned(t6_1(15 downto 12)) * p6_gs;
            p7_bbl <= signed('0' & t6_2(3 downto 0)) * l_bv;
            p7_bbh <= signed('0' & t6_2(7 downto 4)) * l_bv;
            p7_e  <= unsigned(t6_0(12 downto 4));
            p7_c  <= unsigned(t6_1(7 downto 0));
            p7_o  <= signed(t6_2(15 downto 8));
            p7_r  <= unsigned(t6_3(15 downto 8));
            p7_bo <= unsigned(t6_3(7 downto 0));

            -- 8: products
            v_ga := resize(p7_gal, 22) + shift_left(resize(p7_gah, 22), 4);
            p8_ga9 <= v_ga(15 downto 7);
            v_bb := resize(p7_bbl, 19) + shift_left(resize(p7_bbh, 19), 4);
            p8_bb <= v_bb(17 downto 7);
            p8_e <= p7_e; p8_c <= p7_c; p8_o <= p7_o; p8_r <= p7_r; p8_bo <= p7_bo;

            -- 9: fold phase (mod 2 folds) and the triangle fold
            v_f := p8_ga9 + p8_e - s_t9;
            if v_f(8) = '1' then p9_tri <= not v_f(7 downto 0); else p9_tri <= v_f(7 downto 0); end if;
            p9_c <= p8_c; p9_o <= p8_o; p9_r <= p8_r; p9_bo <= p8_bo; p9_bb <= p8_bb;

            -- 10: contrast partial products
            p10_cl <= p9_c(3 downto 0) * p9_tri;
            p10_ch <= p9_c(7 downto 4) * p9_tri;
            p10_o <= p9_o; p10_r <= p9_r; p10_bo <= p9_bo; p10_bb <= p9_bb;

            -- 11: contrast
            v_ct := resize(p10_cl, 16) + shift_left(resize(p10_ch, 16), 4);
            p11_ct <= v_ct(15 downto 7);
            p11_o <= p10_o; p11_r <= p10_r; p11_bo <= p10_bo; p11_bb <= p10_bb;

            -- 12: G level before the 233/256 scale
            p12_g8 <= resize(p11_o, 10) + signed(resize(p11_ct, 10));
            p12_r <= p11_r; p12_bo <= p11_bo; p12_bb <= p11_bb;

            -- 13: g8 * 23
            p13_gm <= shift_left(resize(p12_g8, 15), 4) + shift_left(resize(p12_g8, 15), 3) - resize(p12_g8, 15);
            p13_g8 <= p12_g8;
            p13_r <= p12_r; p13_bo <= p12_bo; p13_bb <= p12_bb;

            -- 14: G = 10 + g8 * 233 / 256, clipped
            v_g := to_signed(10, 12) + resize(p13_g8, 12) - resize(shift_right(p13_gm, 8), 12);
            if v_g < 0 then p14_g <= (others => '0');
            elsif v_g > 255 then p14_g <= (others => '1');
            else p14_g <= unsigned(v_g(7 downto 0)); end if;
            p14_r <= p13_r; p14_bo <= p13_bo; p14_bb <= p13_bb;

            -- 15: crosstalk terms G * 23, G * 22
            p15_gr <= shift_left(resize(p14_g, 13), 4) + shift_left(resize(p14_g, 13), 3) - resize(p14_g, 13);
            p15_gb <= shift_left(resize(p14_g, 13), 4) + shift_left(resize(p14_g, 13), 2) + shift_left(resize(p14_g, 13), 1);
            p15_g <= p14_g; p15_r <= p14_r; p15_bo <= p14_bo; p15_bb <= p14_bb;

            -- 16: red and blue, clipped
            v_r := signed(resize(p15_r, 12)) + resize(l_rv, 12) - signed(resize(p15_gr(12 downto 8), 12));
            if v_r < 0 then p16_r <= (others => '0');
            elsif v_r > 255 then p16_r <= (others => '1');
            else p16_r <= unsigned(v_r(7 downto 0)); end if;
            v_b := to_signed(184, 13) + signed(resize(p15_bo, 13)) - resize(p15_bb, 13) - signed(resize(p15_gb(12 downto 8), 13));
            if v_b < 0 then p16_b <= (others => '0');
            elsif v_b > 255 then p16_b <= (others => '1');
            else p16_b <= unsigned(v_b(7 downto 0)); end if;
            p16_g <= p15_g;

            -- 17: Palette Swap; colour-matrix ROM addresses
            if s_swap = '1' then p17_ar <= p16_b; p17_ab <= p16_r; else p17_ar <= p16_r; p17_ab <= p16_b; end if;
            p17_ag <= p16_g;

            -- 18: ROMs (p_tab); 19: fabric copies
            p19_qr <= p18_qr; p19_qg <= p18_qg; p19_qb <= p18_qb;

            -- 20: partial sums
            p20_ya <= resize(unsigned(p19_qr(31 downto 22)), 12) + resize(unsigned(p19_qg(31 downto 22)), 12);
            p20_yb <= to_unsigned(64, 12) + resize(unsigned(p19_qb(31 downto 22)), 12);
            p20_ua <= resize(signed(p19_qr(21 downto 12)), 12) + resize(signed(p19_qg(21 downto 12)), 12);
            p20_ub <= to_signed(512, 12) + resize(signed(p19_qb(21 downto 12)), 12);
            p20_va <= resize(signed(p19_qr(11 downto 2)), 12) + resize(signed(p19_qg(11 downto 2)), 12);
            p20_vb <= to_signed(512, 12) + resize(signed(p19_qb(11 downto 2)), 12);

            -- 21: Y U V, clamped (drawn luma <= 940 -> ~777 after the 13/16 scale)
            v_y := p20_ya + p20_yb;
            if v_y > 940 then s_y21 <= to_unsigned(940, 10); else s_y21 <= v_y(9 downto 0); end if;
            v_u := p20_ua + p20_ub;
            if v_u < 0 then s_u21 <= (others => '0'); elsif v_u > 1023 then s_u21 <= (others => '1'); else s_u21 <= unsigned(v_u(9 downto 0)); end if;
            v_u := p20_va + p20_vb;
            if v_u < 0 then s_v21 <= (others => '0'); elsif v_u > 1023 then s_v21 <= (others => '1'); else s_v21 <= unsigned(v_u(9 downto 0)); end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent
    -- 8-sums, luma [1 2 1]; centre at stage 26 -> 27), luma 13/16 and the
    -- blanking gate at stage 28
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_csu(0) <= s_u21; s_csv(0) <= s_v21;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u21 - s_csu(7);
            s_boxv <= s_boxv + s_v21 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u27 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v27 <= v_sc(13 downto 4);
            s_y22 <= s_y21; s_y23 <= s_y22; s_y24 <= s_y23; s_y25 <= s_y24;
            s_y26 <= s_y25; s_y27x <= s_y26;
            v_sy := resize(s_y25, 12) + shift_left(resize(s_y26, 12), 1) + resize(s_y27x, 12);
            s_yf27 <= v_sy(11 downto 2);
            if s_yf27 > 64 then v_dl := s_yf27 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(26) = '1' then
                s_y28 <= s_yf27 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u28 <= s_u27; s_v28 <= s_v27;
            else
                s_y28 <= to_unsigned(64, 10); s_u28 <= to_unsigned(512, 10); s_v28 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(s_u28);
    data_out.v       <= std_logic_vector(s_v28);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture jukebox;
