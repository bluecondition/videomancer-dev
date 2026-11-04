-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Grommets v0.1.0 -- Heirloom (from the video clips), built from the still WIN_20230216_01_27_34 @ 2.5 s.
--
-- The still: two big black keyhole holes (x 538 / 1474, 545 px wide, 511 lines tall, round sides, pointed top and
-- bottom) over a purple / blue lattice of contours, a bright cyan X of contours crossing between the holes, a reddish
-- "eye" of rings at the bottom centre, thin black stems leaving the holes' bottom tips and fanning out going down
-- (a keystoned ridge), the whole built of 9.49-line colour bands (dark line, indigo, blue, violet, magenta, red ..),
-- and a dark green rim on the left of every black edge, dark red on the right.  In the clip the holes scroll down at
-- 224 px/s (period 1152 lines); the stems and the bands stay put.
--
-- The patch, as fitted (grommets.py has the tables, .heirloom_work/rb_grommets/notes.md the measurements):
--   F      = 0.805 tri(x; period 936 px, trough at the centre x 1006) + cos(y - v t; period 1152, max at 319.5)
--            + stem bump (2.10 max(0, 1 - |x' - xs(y)| / 15 px), x' = |x - 1006|, xs = 381.3 + 0.0422 y + 2.15e-4 y^2)
--   zone   = (F * Lattice + Holes) over 128 zones (F -1.75 .. 1.75); zones >= 95 are the black holes
--   phase  = a 9.49-line sawtooth stepped per line pair, 8 steps
--   colour = TABLE[palette][zone][phase]; U (= Cr) -/+ R by the sign of dF/dx (green entering black, red leaving)
--
--   K1  Lattice   colouriser gain: x0.25 .. x4 (512 = the still): flat purple .. black diamonds and thin contours
--   K2  Columns   hole spacing: 1 .. 4 holes across (512 = the still's 2.05); the stems follow
--   K3  Rows      hole row spacing: 2304 .. 576 lines (512 = the still's 1152)
--   K4  Speed     bipolar scroll (square law, +-8 deadband): 764 = the still's 224 px/s down; left = up
--   K5  Stems     stem strength 0 .. 4x (square law; 512 = the still)
--   K6  Etch      how far the camera (or the Osc) bends the lattice: 0 .. 2 F per full luma (square law; 512 = 0.5)
--   S7  Bands     Fine (9.49 lines, the still) / Bold (19 lines)
--   S8  Palette   Violet (the still) / Rainbow (the 01_16_43 clip: three hue cycles of rings)
--   S9  Shape     Round (cosine rows, the still) / Diamond (triangle rows: diamond holes, straight contours)
--   S10 Negative  Off / On (the zone index inverted: black at the lattice minima, the holes coloured)
--   S11 Source    Video / Osc
--   P12 HOLES     the key level: 0 % = no black at all .. 40 % = the still .. 100 % = black floods all but the eyes
--
-- Engine: vblank-zone sequencer (one serial multiplier, 25 slots, shared glide), a per-line engine (V wave ROM x gain,
-- the stem DDA, the band phase), a beam DDA carrying the H triangle x gain exactly in half-pixel units (reloaded at
-- every fold), a 17-stage pixel path, the 2048 x 32 colour table in block RAM.  Virtual 1080-line coordinates (SD
-- shows the centre 4:3).  Generated avid; output conditioning (Y, U, V through two [1 2 2 2 2 2 2 2 1]/16 boxes with
-- the per-line resync, luma 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.grommets_pkg.all;
use work.all;

architecture grommets of program_top is
    constant C_LAT : integer := 16;   -- = processing_delay_clks (the pixel path is 17 stages: the gate is stage C_LAT + 1)

    constant C_XC_H : integer := 2012;      -- mirror axis / lattice trough, h (half virtual px)
    constant C_GV0  : integer := 9362;      -- zone units per F unit at gain 1 (256 per zone, 128 zones over 3.5 F)
    constant C_BP0  : integer := 41367;     -- band phase at line 0 (Q16 per band)
    constant C_ZT   : integer := 95;        -- first black zone (the still's key level)

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
    -- (a * mt) >> (15 - sh) from the 36-bit accumulator, sh = -2 .. 1
    function f_shq(acc : unsigned(35 downto 0); sh : signed(2 downto 0)) return unsigned is
    begin
        case to_integer(sh) is
            when -2     => return acc(35 downto 17);
            when -1     => return acc(34 downto 16);
            when 0      => return acc(33 downto 15);
            when others => return acc(32 downto 14);
        end case;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0 (read-before-write kept: collisions possible)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q  : unsigned(15 downto 0) := (others => '0');
    signal r_cs_a  : unsigned(8 downto 0) := (others => '0');
    signal r_cs_q  : signed(11 downto 0) := (others => '0');
    signal r_tb_q  : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_x0   : unsigned(9 downto 0) := (others => '0');
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
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(409 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(764, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_bold, s_pal, s_diam, s_neg, s_src : std_logic := '0';

    -- per-field constants
    signal z_mt1, z_mt2, z_mt2n, z_mt3n : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_sh1, z_sh2, z_sh2n, z_sh3n : signed(2 downto 0) := (others => '0');
    signal z_a    : unsigned(21 downto 0) := (others => '0');
    signal z_c1s, z_c2s : unsigned(15 downto 0) := (others => '0');
    signal z_sp8  : unsigned(11 downto 0) := (others => '0');
    signal z_fold : unsigned(13 downto 0) := to_unsigned(C_XC_H, 14);
    signal z_n    : std_logic := '0';
    signal z_fd   : unsigned(11 downto 0) := (others => '0');
    signal c_gv   : unsigned(15 downto 0) := to_unsigned(C_GV0, 16);
    signal c_gh   : unsigned(15 downto 0) := to_unsigned(7536, 16);
    signal c_hp   : unsigned(11 downto 0) := to_unsigned(936, 12);     -- half period, h
    signal c_slp8 : unsigned(15 downto 0) := to_unsigned(4122, 16);    -- Q8 units per h
    signal c_slpx : unsigned(17 downto 0) := to_unsigned(8244, 18);    -- per pixel
    signal c_rg1, c_rg2, c_rg3, c_rg4 : unsigned(17 downto 0) := (others => '0');
    signal c_fold0 : unsigned(13 downto 0) := to_unsigned(140, 14);
    signal c_kind0 : std_logic := '0';
    signal c_p0   : signed(24 downto 0) := (others => '0');
    signal c_ghq  : signed(24 downto 0) := (others => '0');            -- GH * 256
    signal c_m    : unsigned(15 downto 0) := to_unsigned(14563, 16);
    signal c_incv : unsigned(17 downto 0) := to_unsigned(14563, 18);
    signal c_dv   : unsigned(18 downto 0) := (others => '0');
    signal c_scr  : unsigned(23 downto 0) := (others => '0');          -- scroll phase (Q24)
    signal c_ph0  : unsigned(23 downto 0) := (others => '0');
    signal c_binc : unsigned(14 downto 0) := to_unsigned(13813, 15);
    signal c_l0   : unsigned(27 downto 0) := to_unsigned(49976320, 28);  -- stem base, Q16 h
    signal c_d0   : unsigned(17 downto 0) := to_unsigned(5562, 18);
    signal c_dd   : unsigned(11 downto 0) := to_unsigned(56, 12);
    signal c_gs   : unsigned(18 downto 0) := to_unsigned(19659, 19);
    signal c_ss   : unsigned(15 downto 0) := to_unsigned(655, 16);
    signal c_eg   : unsigned(12 downto 0) := to_unsigned(18, 13);
    signal c_off  : unsigned(15 downto 0) := to_unsigned(16384, 16);
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);
    constant C_KY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_KC : unsigned(9 downto 0) := to_unsigned(512, 10);

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
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_vyn  : unsigned(18 downto 0) := (others => '0');
    signal e_ph   : unsigned(23 downto 0) := (others => '0');          -- V phase, Q24
    signal e_bp   : unsigned(15 downto 0) := to_unsigned(C_BP0, 16);   -- band phase, Q16
    signal e_xs   : unsigned(27 downto 0) := (others => '0');          -- stem x', Q16 h
    signal e_xd   : unsigned(17 downto 0) := (others => '0');
    signal e_tri  : signed(11 downto 0) := (others => '0');
    signal e_wn   : std_logic := '0';
    signal m_a    : unsigned(27 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(27 downto 0) := (others => '0');
    signal n_fy   : signed(16 downto 0) := (others => '0');
    signal n_xl, n_xr : unsigned(12 downto 0) := (others => '0');
    signal n_ph   : unsigned(2 downto 0) := (others => '0');

    -- beam
    signal b_x, b_x1, b_x2 : unsigned(12 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_p   : signed(24 downto 0) := (others => '0');
    signal b_up  : std_logic := '0';
    signal b_fc  : std_logic := '0';
    signal b_pos : unsigned(13 downto 0) := (others => '0');
    signal b_kind : std_logic := '0';
    signal b_rg  : unsigned(17 downto 0) := (others => '0');
    signal l_fy  : signed(16 downto 0) := (others => '0');
    signal l_xl, l_xr : unsigned(12 downto 0) := (others => '0');
    signal l_ph  : unsigned(2 downto 0) := (others => '0');

    -- pixel path
    signal p2_s   : signed(20 downto 0) := (others => '0');
    signal q2_d1, q2_d2 : signed(13 downto 0) := (others => '0');
    signal p2_up  : std_logic := '0';
    signal p3_s   : signed(20 downto 0) := (others => '0');
    signal q3_a1, q3_a2 : unsigned(4 downto 0) := (others => '0');
    signal q3_i1, q3_i2, q3_r1, q3_r2 : std_logic := '0';
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p3_up  : std_logic := '0';
    signal p4_s   : signed(20 downto 0) := (others => '0');
    signal q4_dm  : unsigned(4 downto 0) := (others => '0');
    signal q4_bs  : signed(24 downto 0) := (others => '0');            -- stem slope term, Q8 per h
    signal p4_ea, p4_eb : unsigned(16 downto 0) := (others => '0');
    signal p4_up  : std_logic := '0';
    signal p5_s   : signed(20 downto 0) := (others => '0');
    signal q5_pr  : unsigned(20 downto 0) := (others => '0');
    signal q5_ds  : signed(24 downto 0) := (others => '0');
    signal p5_le  : unsigned(20 downto 0) := (others => '0');
    signal p6_s   : signed(20 downto 0) := (others => '0');
    signal q6_bm  : unsigned(18 downto 0) := (others => '0');
    signal p6_sn  : std_logic := '0';
    signal p7_s   : signed(20 downto 0) := (others => '0');
    signal p7_sn  : std_logic := '0';
    signal p8_sn, p9_sn, p10_sn : std_logic := '0';
    signal p9_q, p10_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y13, s_u13, s_v13 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y17 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer and ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_rom : process(clk)
        variable v_z : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            r_ex_q <= C_GR_EXP2(to_integer(r_ex_a));
            r_cs_q <= C_GR_COS(to_integer(r_cs_a));
            -- colour table: address from the stage-7 sum (clamped to 0 .. 32767, zone = bits 14..8)
            if p7_s < 0 then
                v_z := (others => '0');
            elsif p7_s > 32767 then
                v_z := (others => '1');
            else
                v_z := unsigned(p7_s(14 downto 8));
            end if;
            if s_neg = '1' then v_z := not v_z; end if;
            r_tb_q <= C_GR_TAB(to_integer(unsigned'(s_pal & v_z & l_ph)));
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
        variable v_o  : signed(16 downto 0);
        variable v_t  : unsigned(18 downto 0);
        variable v_en : signed(10 downto 0);
        variable v_q  : unsigned(18 downto 0);
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
                s_bold <= registers_in(6)(0);
                s_pal  <= registers_in(6)(1);
                s_diam <= registers_in(6)(2);
                s_neg  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(960, 10); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');      s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');      s_oxs <= to_unsigned(102, 9);
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

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from
            -- load + 17 (stores at load + 18).  The slot number is decoded at 30 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 31 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
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
                    when 438 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when 462 => z_sel <= to_unsigned(18, 5); z_pre <= '1';
                    when 486 => z_sel <= to_unsigned(19, 5); z_pre <= '1';
                    when 510 => z_sel <= to_unsigned(20, 5); z_pre <= '1';
                    when 534 => z_sel <= to_unsigned(21, 5); z_pre <= '1';
                    when 558 => z_sel <= to_unsigned(22, 5); z_pre <= '1';
                    when 582 => z_sel <= to_unsigned(23, 5); z_pre <= '1';
                    when 606 => z_sel <= to_unsigned(24, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(C_GV0, 36);          z_fb <= z_mt1;                     -- GV
                    when 1  => z_fa <= resize(c_gv, 36);                z_fb <= to_unsigned(26378, 16);    -- GH = 0.805 GV
                    when 2  => z_fa <= resize(c_gh, 36);                z_fb <= to_unsigned(17925, 16);    -- GH * 1024/1872
                    when 3  => z_fa <= resize(z_a, 36);                 z_fb <= z_mt2n;                    -- SLP8
                    when 4  => z_fa <= to_unsigned(936, 36);            z_fb <= z_mt2;                     -- HP
                    when 6  => z_fa <= resize(c_slp8, 36);              z_fb <= resize(z_fd, 16);          -- SLP8 * d
                    when 7  => z_fa <= to_unsigned(14563, 36);          z_fb <= z_mt3n;                    -- M = 2^24 / Py
                    when 8  => z_fa <= resize(c_m, 36);                 z_fb <= resize(s_ys, 16);          -- INCV
                    when 9  => z_fa <= resize(s_vm, 36);                z_fb <= resize(s_vm, 16);          -- speed^2
                    when 10 => z_fa <= resize(z_a, 36);                 z_fb <= to_unsigned(990, 16);      -- lines per field, Q8
                    when 11 => z_fa <= resize(z_sp8, 36);               z_fb <= c_m;                       -- DV
                    when 12 => z_fa <= to_unsigned(81792, 36);          z_fb <= c_m;                       -- 319.5 lines * M
                    when 13 => z_fa <= to_unsigned(195220, 36);         z_fb <= z_mt2;                     -- stem base L0, Q8 h
                    when 14 => z_fa <= resize(s_ys, 36);                z_fb <= to_unsigned(22139, 16);    -- c1 step
                    when 15 => z_fa <= resize(s_ys, 36);                z_fb <= resize(s_ys, 16);          -- ys^2
                    when 16 => z_fa <= resize(z_a, 36);                 z_fb <= to_unsigned(28, 16);       -- c2 step
                    when 17 => z_fa <= resize(z_c1s, 36) + resize(z_c2s, 36); z_fb <= z_mt2;              -- D0
                    when 18 => z_fa <= resize(z_c2s & '0', 36);         z_fb <= z_mt2;                     -- DD
                    when 19 => z_fa <= resize(s_g5(12 downto 3), 36);   z_fb <= resize(s_g5(12 downto 3), 16); -- k5^2
                    when 20 => z_fa <= resize(z_a, 36);                 z_fb <= c_gv;                      -- k5^2 GV
                    when 21 => z_fa <= resize(z_a, 36);                 z_fb <= to_unsigned(34406, 16);    -- GS
                    when 22 => z_fa <= resize(c_gs, 36);                z_fb <= to_unsigned(2185, 16);     -- SS = GS / 30
                    when 23 => z_fa <= resize(s_g6(12 downto 3), 36);   z_fb <= resize(s_g6(12 downto 3), 16); -- k6^2
                    when others => z_fa <= resize(z_a(21 downto 4), 36); z_fb <= c_gv;                     -- EG
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
                    -- scroll (last field's step); beam origin (+8 px: the output boxes' group delay)
                    when 20 =>
                        if s_vneg = '1' then c_scr <= c_scr - resize(c_dv, 24); else c_scr <= c_scr + resize(c_dv, 24); end if;
                        c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
                        -- exponent 1: gain 2^((k1 - 512) / 256), Q8 octaves = k1 - 512
                        v_e := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh1 <= v_e(10 downto 8);
                    when 22 => z_mt1 <= r_ex_q;
                    when 23 =>
                        -- holes offset: 16384 + (p12 - 409) * 24
                        v_o := signed(resize(s_gp(12 downto 3), 17)) - 409;
                        v_o := shift_left(v_o, 4) + shift_left(v_o, 3);
                        c_off <= unsigned(resize(v_o + 16384, 16));
                        if s_bold = '1' then c_binc <= to_unsigned(6907, 15); else c_binc <= to_unsigned(13813, 15); end if;
                    when 50  => v_q := f_shq(f_acc, z_sh1); c_gv <= v_q(15 downto 0);
                    when 74  => c_gh <= f_acc(30 downto 15);
                    when 98  =>
                        z_a <= resize(f_acc(30 downto 15), 22);
                        -- exponent 2: columns, Q8 octaves = (512 - k2) / 2; read -e2 now, +e2 at 124
                        v_e := shift_right(512 - signed(resize(s_g2(12 downto 3), 11)), 1);
                        v_en := -v_e;
                        r_ex_a <= unsigned(v_en(7 downto 0)); z_sh2n <= v_en(10 downto 8);
                    when 100 => z_mt2n <= r_ex_q;
                    when 122 => v_q := f_shq(f_acc, z_sh2n); c_slp8 <= v_q(15 downto 0);
                    when 123 =>
                        v_e := shift_right(512 - signed(resize(s_g2(12 downto 3), 11)), 1);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh2 <= v_e(10 downto 8);
                        case to_integer(s_xs) is
                            when 4      => c_slpx <= shift_left(resize(c_slp8, 18), 2);
                            when 3      => c_slpx <= shift_left(resize(c_slp8, 18), 1) + resize(c_slp8, 18);
                            when others => c_slpx <= shift_left(resize(c_slp8, 18), 1);
                        end case;
                        c_rg1 <= resize(c_slp8, 18);
                        c_rg2 <= shift_left(resize(c_slp8, 18), 1);
                        c_rg4 <= shift_left(resize(c_slp8, 18), 2);
                    when 124 => c_rg3 <= c_rg1 + c_rg2;
                    when 125 => z_mt2 <= r_ex_q;
                    when 146 => v_q := f_shq(f_acc, z_sh2); c_hp <= v_q(11 downto 0);
                    -- first fold at or right of the beam origin: subtract HP from the centre while it stays >= xb0
                    when 147 => z_fold <= to_unsigned(C_XC_H, 14); z_n <= '0';
                    when 148 | 149 | 150 | 151 | 152 | 153 | 154 | 155 | 156 =>
                        if z_fold >= resize(c_hp, 14) + resize(c_xb0, 14) then
                            z_fold <= z_fold - resize(c_hp, 14); z_n <= not z_n;
                        end if;
                    when 157 => c_fold0 <= z_fold; c_kind0 <= z_n; z_fd <= resize(z_fold - resize(c_xb0, 14), 12);
                    when 158 => c_ghq <= signed(resize(c_gh & x"00", 25));
                    when 194 =>
                        if c_kind0 = '0' then c_p0 <= -c_ghq + signed(resize(f_acc(26 downto 0), 25));
                        else c_p0 <= c_ghq - signed(resize(f_acc(26 downto 0), 25)); end if;
                        -- exponent 3: rows, -e3 = (k3 - 512) / 2
                        v_e := shift_right(signed(resize(s_g3(12 downto 3), 11)) - 512, 1);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh3n <= v_e(10 downto 8);
                    when 196 => z_mt3n <= r_ex_q;
                    when 218 => v_q := f_shq(f_acc, z_sh3n); c_m <= v_q(15 downto 0);
                    when 242 => c_incv <= f_acc(25 downto 8);
                    when 266 => z_a <= resize(f_acc(19 downto 0), 22);
                    when 290 => z_sp8 <= f_acc(27 downto 16);
                    when 314 => c_dv <= f_acc(26 downto 8);
                    when 338 => c_ph0 <= c_scr - f_acc(31 downto 8);
                    when 362 => v_q := f_shq(f_acc, z_sh2); c_l0 <= resize(v_q, 20) & x"00";
                    when 386 => z_c1s <= f_acc(25 downto 10);
                    when 410 => z_a <= f_acc(21 downto 0);
                    when 434 => z_c2s <= f_acc(31 downto 16);
                    when 458 => v_q := f_shq(f_acc, z_sh2); c_d0 <= v_q(17 downto 0);
                    when 482 => v_q := f_shq(f_acc, z_sh2); c_dd <= v_q(11 downto 0);
                    when 506 => z_a <= resize(f_acc(19 downto 0), 22);
                    when 530 => z_a <= resize(f_acc(35 downto 18), 22);
                    when 554 => c_gs <= f_acc(32 downto 14);
                    when 578 => c_ss <= f_acc(31 downto 16);
                    when 602 => z_a <= resize(f_acc(19 downto 0), 22);
                    when 626 => c_eg <= f_acc(35 downto 23);
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
            s_osum <= shift_right(v_s - shift_right(v_s, 3), 1);     -- half amplitude: a gentle bend at the default Etch
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
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line: the V wave (cosine ROM or triangle) times the
    -- gain on a serial multiplier, the stem x' DDA, the band phase
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_a12 : unsigned(11 downto 0);
        variable v_d   : signed(12 downto 0);
        variable v_pr  : unsigned(1 downto 0);
        variable v_w   : signed(11 downto 0);
        variable v_t   : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 700 then e_zk <= '1'; end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(26 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_ph <= c_ph0;
                e_bp <= to_unsigned(C_BP0, 16);
                e_xs <= c_l0;
                e_xd <= c_d0;
                e_lc <= (others => '0');
            elsif g_start = '1' then
                -- advance to the next line
                e_vy <= e_vyn;
                e_ph <= e_ph + resize(c_incv, 24);
                e_xs <= e_xs + resize(e_xd, 28);
                e_xd <= e_xd + resize(c_dd, 18);
                v_pr := e_vyn(10 downto 9) - e_vy(10 downto 9);
                case to_integer(v_pr) is
                    when 0      => null;
                    when 1      => e_bp <= e_bp + resize(c_binc, 16);
                    when 2      => e_bp <= e_bp + resize(c_binc & '0', 16);
                    when others => e_bp <= e_bp + resize(c_binc & '0', 16) + resize(c_binc, 16);
                end case;
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;
            e_vyn <= e_vy + resize(s_ys, 19);

            case to_integer(e_lc) is
                when 1 =>
                    -- cosine address (half wave, mirrored) and the triangle alternative
                    if e_ph(23) = '1' then r_cs_a <= not e_ph(22 downto 14); else r_cs_a <= e_ph(22 downto 14); end if;
                    v_a12 := e_ph(23 downto 12);
                    v_d := signed(resize(v_a12, 13)) - 2048;
                    if v_d < 0 then v_d := -v_d; end if;
                    v_t := 2047 - v_d;
                    e_tri <= v_t(11 downto 0);
                    n_ph <= e_bp(15 downto 13);
                    if e_xs(27 downto 16) >= C_XC_H then n_xl <= (others => '0');
                    else n_xl <= to_unsigned(C_XC_H, 13) - resize(e_xs(27 downto 16), 13); end if;
                    n_xr <= to_unsigned(C_XC_H, 13) + resize(e_xs(27 downto 16), 13);
                when 3 =>
                    if s_diam = '1' then v_w := e_tri; else v_w := r_cs_q; end if;
                    if v_w < 0 then v_w := -v_w; e_wn <= '1'; else e_wn <= '0'; end if;
                    m_a <= resize(unsigned(v_w), 28); m_b <= c_gv; m_acc <= (others => '0');
                when 20 =>
                    if e_wn = '1' then n_fy <= -signed(resize(m_acc(27 downto 11), 17));
                    else n_fy <= signed(resize(m_acc(27 downto 11), 17)); end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: P = the H triangle times the gain, exact, as a Q8 DDA in h units:
    -- +- SLPX per pixel, reloaded to -+ GH at every fold (trough / peak, every
    -- half period).  The fold event for the step after next is registered a
    -- clock ahead.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_r  : unsigned(2 downto 0);
        variable v_np : signed(24 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= resize(c_xb0, 13);
                b_x1 <= resize(c_xb0, 13) + s_xs;
                b_x2 <= resize(c_xb0, 13) + shift_left(resize(s_xs, 13), 1);
                b_px <= (others => '0');
                b_p  <= c_p0;
                b_up <= c_kind0;
                b_fc <= '0';
                b_pos <= c_fold0;
                b_kind <= c_kind0;
                l_fy <= n_fy; l_xl <= n_xl; l_xr <= n_xr; l_ph <= n_ph;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x1;
                b_x1 <= b_x2;
                b_x2 <= b_x2 + s_xs;
                -- P for the next pixel
                if b_fc = '1' then
                    if b_kind = '0' then v_np := -c_ghq + signed(resize(b_rg, 25)); b_up <= '1';
                    else v_np := c_ghq - signed(resize(b_rg, 25)); b_up <= '0'; end if;
                    b_pos <= b_pos + resize(c_hp, 14);
                    b_kind <= not b_kind;
                elsif b_up = '1' then
                    v_np := b_p + signed(resize(c_slpx, 25));
                else
                    v_np := b_p - signed(resize(c_slpx, 25));
                end if;
                b_p <= v_np;
                -- fold event for the step after next: x(n+1) = b_x1, x(n+2) = b_x2
                if resize(b_x1, 14) <= b_pos and resize(b_x2, 14) > b_pos and b_fc = '0' then b_fc <= '1'; else b_fc <= '0'; end if;
                v_r := b_x2(2 downto 0) - b_pos(2 downto 0);
                case to_integer(v_r) is
                    when 1      => b_rg <= c_rg1;
                    when 2      => b_rg <= c_rg2;
                    when 3      => b_rg <= c_rg3;
                    when others => b_rg <= c_rg4;
                end case;
            end if;
            s_lb_ra <= b_px + 8;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a   : signed(13 downto 0);
        variable v_u   : signed(9 downto 0);
        variable v_bm  : signed(20 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: H + V terms; distances to the stems
            p2_s  <= resize(b_p(24 downto 8), 21) + resize(l_fy, 21);
            q2_d1 <= signed(resize(b_x, 14)) - signed(resize(l_xl, 14));
            q2_d2 <= signed(resize(b_x, 14)) - signed(resize(l_xr, 14));
            p2_up <= b_up;
            -- 3: + holes offset; |d| clamped to 31, inside (< 30) and rising (d < 0) flags; luma
            p3_s <= p2_s + signed(resize(c_off, 21));
            v_a := abs(q2_d1);
            if v_a > 31 then q3_a1 <= (others => '1'); else q3_a1 <= unsigned(v_a(4 downto 0)); end if;
            if v_a < 30 then q3_i1 <= '1'; else q3_i1 <= '0'; end if;
            q3_r1 <= q2_d1(13);
            v_a := abs(q2_d2);
            if v_a > 31 then q3_a2 <= (others => '1'); else q3_a2 <= unsigned(v_a(4 downto 0)); end if;
            if v_a < 30 then q3_i2 <= '1'; else q3_i2 <= '0'; end if;
            q3_r2 <= q2_d2(13);
            p3_lm <= unsigned(s_lb_q);
            p3_up <= p2_up;
            -- 4: nearest stem distance; the stem's slope term; etch partials (luma nibbles x gain)
            if q3_a1 < q3_a2 then q4_dm <= q3_a1; else q4_dm <= q3_a2; end if;
            if q3_i1 = '1' then
                if q3_r1 = '1' then q4_bs <= signed(resize(c_ss & x"00", 25)); else q4_bs <= -signed(resize(c_ss & x"00", 25)); end if;
            elsif q3_i2 = '1' then
                if q3_r2 = '1' then q4_bs <= signed(resize(c_ss & x"00", 25)); else q4_bs <= -signed(resize(c_ss & x"00", 25)); end if;
            else
                q4_bs <= (others => '0');
            end if;
            p4_ea <= resize(p3_lm(3 downto 0) * c_eg, 17);
            p4_eb <= resize(p3_lm(7 downto 4) * c_eg, 17);
            p4_s <= p3_s;
            p4_up <= p3_up;
            -- 5: stem product; etch; the field's x slope (H triangle direction + the stem)
            q5_pr <= resize(q4_dm * c_ss, 21);
            p5_le <= resize(p4_ea, 21) + shift_left(resize(p4_eb, 21), 4);
            if p4_up = '1' then q5_ds <= signed(resize(c_slp8, 25)) + q4_bs;
            else q5_ds <= -signed(resize(c_slp8, 25)) + q4_bs; end if;
            p5_s <= p4_s;
            -- 6: bump = max(GS - product, 0); + etch; slope sign
            v_bm := signed(resize(c_gs, 21)) - signed(resize(q5_pr, 21));
            if v_bm < 0 then q6_bm <= (others => '0'); else q6_bm <= unsigned(v_bm(18 downto 0)); end if;
            p6_s <= p5_s + signed(resize(p5_le, 21));
            p6_sn <= q5_ds(24);
            -- 7: + bump (the table address is formed from p7_s in p_rom; the table data is stage 8)
            p7_s <= p6_s + signed(resize(q6_bm, 21));
            p7_sn <= p6_sn;
            -- 8: table (EBR); 9, 10: fabric copies
            p8_sn <= p7_sn;
            p9_q <= r_tb_q; p9_sn <= p8_sn;
            p10_q <= p9_q; p10_sn <= p9_sn;
            -- 11: compose: U (= Cr) -/+ the rim term by the slope sign; black outside the generated avid
            v_u := signed(resize(unsigned(p10_q(23 downto 16)), 10));
            if p10_sn = '1' then v_u := v_u + resize(signed(p10_q(7 downto 0)), 10);
            else v_u := v_u - resize(signed(p10_q(7 downto 0)), 10); end if;
            if v_u < 0 then v_u := (others => '0'); elsif v_u > 255 then v_u := to_signed(255, 10); end if;
            if s_gav_sr(8) = '0' then
                s_y11 <= C_KY; s_u11 <= C_KC; s_v11 <= C_KC;
            else
                s_y11 <= unsigned(p10_q(31 downto 24)) & "00";
                s_u11 <= unsigned(v_u(7 downto 0)) & "00";
                s_v11 <= unsigned(p10_q(15 downto 8)) & "00";
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(8);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (the HD HDMI bar fix and the capture's horizontal softness), luma 13/16
    -- and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to black (the off-avid input of both boxes) once 8 blank
            -- samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 12 sums, stage 13 output
            s_cay(0) <= s_y11; s_cau(0) <= s_u11; s_cav(0) <= s_v11;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KC & "000"; s_bav <= C_KC & "000";
            else
                s_bay <= s_bay + s_y11 - s_cay(7);
                s_bau <= s_bau + s_u11 - s_cau(7);
                s_bav <= s_bav + s_v11 - s_cav(7);
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(10) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y13 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u13 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v13 <= v_sc(13 downto 4);
            else
                s_y13 <= C_KY; s_u13 <= C_KC; s_v13 <= C_KC;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(10);
            -- box B: stage 14 sums, stage 15 output
            s_cby(0) <= s_y13; s_cbu(0) <= s_u13; s_cbv(0) <= s_v13;
            for i in 1 to 7 loop s_cby(i) <= s_cby(i - 1); s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= C_KC & "000"; s_bbv <= C_KC & "000";
            else
                s_bby <= s_bby + s_y13 - s_cby(7);
                s_bbu <= s_bbu + s_u13 - s_cbu(7);
                s_bbv <= s_bbv + s_v13 - s_cbv(7);
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y15 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u15 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v15 <= v_sc(13 downto 4);
            -- 16: luma 13/16
            if s_y15 > 64 then v_dl := s_y15 - 64; else v_dl := (others => '0'); end if;
            s_y16 <= s_y15 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u16 <= s_u15; s_v16 <= s_v15;
            -- 17: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y17 <= s_y16; s_u17 <= s_u16; s_v17 <= s_v16;
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y17);
    data_out.u       <= std_logic_vector(s_u17);
    data_out.v       <= std_logic_vector(s_v17);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture grommets;
