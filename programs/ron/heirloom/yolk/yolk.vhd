-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Yolk v0.1.0 -- Heirloom (video phase), from the clip WIN_20240111_01_01_24 at 0.5 s.
--
-- The still: a row of four red eggs with black yolks across the middle (x 320, 726, 1240, 1663: mirrored about
-- x 993), yellow between them going green on the right, under a band of white flames with magenta U-lines and
-- over a band of white lens rings drawn in magenta (static bands: y < 264 and y >= 770), the left 90 px torn into
-- horizontal colour bars and every key edge broken into dashes.  In the clip the egg rows rise (~160 lines/s, one
-- row every 6.5 s) while the white bands stay put.
--
-- The patch, as fitted (yolk.py has every table):
--   T      = mirrored H triangle (period 425 px), 0 on the egg columns, a half-depth notch at the centre, held at
--            its x = 90 value at the line start
--   VG, VR = the vertical waves of the G and R mixes (one 1055-line period, rolling)
--   F_G    = T Gm + (VG - cK) gm + cK - yolk       F_R = (T Gm + (VR - c0) gm - yolk) * lean(x) + c0
--   key    = static window (top / bottom bands) and F_G' > cK  -> the white layer (W gain = sqrt of the Rings gain)
--   colour = R(F_R) G(F_G) B(F_G) palette ROMs (RGB colouriser), then a constant RGB -> YUV matrix
--
--   K1  Eggs     eggs across: x0.33 .. x3 (512 = the still's 4.5)
--   K2  Rows     vertical period: x0.31 .. x3.25 rows (512 = the still's one row per 1055 lines)
--   K3  Yolk     the yolks: shrunk to red seeds (0) .. the still (512) .. swollen black yolks (1023)
--   K4  Rise     bipolar: the egg rows rise (+) or sink (-); 680 = the clip's 160 lines/s
--   K5  Bands    height of the white bands: none (0) .. the still (512) .. nearly the whole frame (1023)
--   K6  Etch     how far the camera (or the Osc, x6) bends the field
--   S7  Mirror   the frame folded about its centre line: the flames reflected below, rows of eggs facing
--   S8  Invert   the field inverted: the ring centres become yolks, the eggs become ringed bulbs
--   S9  Tear     Dash (the still's dashes) / Slip: bands of 16..64 virtual lines slide sideways
--   S10 Colours  Yolk (the still) / Lagoon (the sibling clip: red ground, green / cyan / blue rings)
--   S11 Source   Video / Osc
--   P12 RINGS    the colouriser gain: soft eggs (0 %) .. the still (38 %) .. nested targets in every egg (100 %)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line
-- start (computes the NEXT line: the V phase, the V ROM read, the gained line terms, the noise seed, the bar /
-- slip bands), a beam DDA (|acc| = the mirrored H phase), a 20-stage pixel path.  Virtual 1080-line coordinates
-- (SD shows the centre 4:3).  Generated avid; output conditioning (Y, U and V through one [1 2 2 2 2 2 2 2 1]/16
-- box, its taps and the syncs in block-RAM rings) and luma 13/16 before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.yolk_pkg.all;
use work.all;

architecture yolk of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant C_XC2  : integer := 1987;        -- mirror centre (h = half virtual px)
    constant C_XLH  : integer := 180;         -- line-start hold (90 px)
    constant C_KH0  : integer := 19738;       -- 2^24 / 850 h: H phase per h at the still's period
    constant C_KV0  : integer := 15903;       -- 2^24 / 1055: V phase per virtual line
    constant C_UC   : integer := 6559891;     -- 0.391 * 2^24: H phase at the centre
    constant C_CPL  : integer := 1828717;     -- (0.5 - 0.391) * 2^24: the centre notch
    constant C_CK   : integer := 563;         -- key level 0.55 (Q10)
    constant C_C0   : integer := 410;         -- R pivot 0.40 (Q10)

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
    function f_xs16(x : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := x xor shift_left(x, 7);
        v := v xor shift_right(v, 9);
        v := v xor shift_left(v, 8);
        return v;
    end function;
    -- palette fold: a triangle of period 1024 over the raw index (mirrors at 0 and 512, no carries)
    function f_fold(v : signed(11 downto 0)) return unsigned is
    begin
        if v(9) = '1' then return not unsigned(v(8 downto 0)); else return unsigned(v(8 downto 0)); end if;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc lattice)
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
    signal r_v_a  : unsigned(8 downto 0) := (others => '0');
    signal r_v_q  : std_logic_vector(23 downto 0) := (others => '0');
    signal r_r_a, r_gb_a : unsigned(10 downto 0) := (others => '0');
    signal r_r_q  : std_logic_vector(7 downto 0) := (others => '0');
    signal r_gb_q : std_logic_vector(15 downto 0) := (others => '0');

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
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(380 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(390 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(680, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_mir, s_inv, s_slip, s_pal, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_neg  : std_logic := '0';
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_shp  : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_a    : unsigned(9 downto 0) := (others => '0');
    signal z_hi   : std_logic := '0';
    signal c_kh   : unsigned(15 downto 0) := to_unsigned(C_KH0, 16);   -- H phase per h
    signal c_kv   : unsigned(15 downto 0) := to_unsigned(C_KV0, 16);   -- V phase per virtual line
    signal c_gm   : unsigned(10 downto 0) := to_unsigned(256, 11);     -- Rings gain (Q8)
    signal c_xst  : unsigned(11 downto 0) := to_unsigned(C_XLH, 12);   -- first stepped x (h)
    signal c_a0   : signed(28 downto 0) := (others => '0');            -- H acc at c_xst
    signal c_ks   : unsigned(17 downto 0) := (others => '0');          -- H acc step per pixel
    signal c_scr  : unsigned(23 downto 0) := (others => '0');          -- scroll phase
    signal c_ps   : unsigned(23 downto 0) := (others => '0');          -- psi at virtual line 0
    signal z_t    : unsigned(19 downto 0) := (others => '0');          -- sequencer scratch
    signal c_spd  : unsigned(21 downto 0) := (others => '0');          -- scroll step per field
    signal c_yn   : std_logic := '0';                                  -- yolk < 0 (shrink)
    signal c_ygm  : signed(13 downto 0) := (others => '0');            -- yolk * gm (Q10)
    signal c_ki3m : signed(15 downto 0) := to_signed(6246, 16);        -- Invert: index offsets KI + 3072 (Q10)
    signal c_ki3r : signed(15 downto 0) := to_signed(5274, 16);
    signal c_kcmp : signed(15 downto 0) := to_signed(C_CK, 16);        -- key compare constant (on F xor inv)
    signal c_eg   : unsigned(13 downto 0) := (others => '0');          -- etch gain per luma step (Q8, Q10 out)
    signal c_em   : unsigned(6 downto 0) := (others => '0');           -- ... as mantissa << c_ee
    signal c_ee   : unsigned(2 downto 0) := (others => '0');
    signal c_wt   : unsigned(10 downto 0) := to_unsigned(264, 11);     -- window: top band ends
    signal c_wb   : unsigned(10 downto 0) := to_unsigned(770, 11);     -- bottom band starts
    signal c_fc   : unsigned(15 downto 0) := to_unsigned(1, 16);       -- field counter (noise seeds)
    signal c_ssd  : unsigned(15 downto 0) := to_unsigned(4321, 16);    -- slip seed (changes every 6 fields)
    signal c_s6   : unsigned(2 downto 0) := (others => '0');

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
    signal e_lc   : unsigned(6 downto 0) := to_unsigned(127, 7);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');          -- next line, Q8 virtual lines
    signal e_vf   : unsigned(18 downto 0) := (others => '0');          -- folded (Mirror)
    signal e_ln   : std_logic := '0';
    signal e_l4   : unsigned(1 downto 0) := (others => '0');
    signal e_seed : unsigned(15 downto 0) := to_unsigned(1, 16);
    signal e_brs  : unsigned(15 downto 0) := to_unsigned(77, 16);
    signal e_srs  : unsigned(15 downto 0) := to_unsigned(99, 16);
    signal e_bct  : signed(13 downto 0) := (others => '0');            -- bar band: Q8 virtual lines left
    signal e_bar  : signed(10 downto 0) := (others => '0');
    signal e_sct  : signed(14 downto 0) := (others => '0');            -- slip band: Q8 virtual lines left
    signal e_sh   : signed(10 downto 0) := (others => '0');            -- slip shift (h)
    signal e_vg, e_vr : signed(11 downto 0) := (others => '0');
    signal e_sg   : std_logic := '0';
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    signal n_vgm, n_vr : signed(15 downto 0) := (others => '0');
    signal n_a0   : signed(28 downto 0) := (others => '0');
    signal n_dt, n_db : signed(6 downto 0) := (others => '0');
    signal n_seed : unsigned(15 downto 0) := to_unsigned(1, 16);
    signal n_bar  : signed(10 downto 0) := (others => '0');

    -- beam (stage 1)
    signal b_x    : unsigned(11 downto 0) := (others => '0');
    signal b_px   : unsigned(10 downto 0) := (others => '0');
    signal b_acc  : signed(28 downto 0) := (others => '0');
    signal b_ga   : unsigned(7 downto 0) := (others => '0');            -- R lean: x / 240 h
    signal b_gk   : unsigned(3 downto 0) := (others => '0');
    signal b_hold : std_logic := '0';
    signal b_lf   : unsigned(15 downto 0) := to_unsigned(1, 16);
    signal b_rc   : signed(8 downto 0) := (others => '0');
    signal b_n    : signed(9 downto 0) := (others => '0');
    signal l_vgm, l_vr : signed(15 downto 0) := (others => '0');
    signal l_dt, l_db : signed(6 downto 0) := (others => '0');
    signal l_bar  : signed(10 downto 0) := (others => '0');
    signal l_inv  : std_logic := '0';
    signal p6_h, p7_h : std_logic := '0';

    -- pixel path
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    type t_u4 is array (natural range <>) of unsigned(3 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal p_lm   : t_u8(0 to 3) := (others => (others => '0'));
    signal p2_aa  : unsigned(27 downto 0) := (others => '0');
    signal p2_n, p3_n, p4_n, p5_n : signed(9 downto 0) := (others => '0');
    signal p2_ns  : signed(6 downto 0) := (others => '0');
    signal p2_h, p3_h, p4_h, p5_h : std_logic := '0';
    signal p3_u   : unsigned(23 downto 0) := (others => '0');
    signal p3_pl, p3_win : std_logic := '0';
    signal p3_ls0, p3_ls1 : unsigned(8 downto 0) := (others => '0');
    signal p4_t   : unsigned(9 downto 0) := (others => '0');
    signal p4_ls  : unsigned(9 downto 0) := (others => '0');
    signal p5_p1a : unsigned(15 downto 0) := (others => '0');
    signal p5_p1b : unsigned(14 downto 0) := (others => '0');
    signal p5_lm  : signed(8 downto 0) := (others => '0');
    signal p6_p1  : unsigned(20 downto 0) := (others => '0');
    signal p6_ea  : unsigned(14 downto 0) := (others => '0');
    signal p6_en  : std_logic := '0';
    signal p6_bg, p6_br : signed(11 downto 0) := (others => '0');
    signal p7_p1  : signed(15 downto 0) := (others => '0');
    signal p7_e   : signed(13 downto 0) := (others => '0');
    signal p7_bg, p7_br : signed(11 downto 0) := (others => '0');
    signal p8_qgm, p8_qr, p8_eg, p8_er : signed(15 downto 0) := (others => '0');
    signal p9_fgm, p9_fr : signed(15 downto 0) := (others => '0');
    signal p_st   : t_sl(4 to 14) := (others => '0');
    signal p_win  : t_sl(4 to 9) := (others => '0');
    signal p_gk   : t_u4(2 to 9) := (others => (others => '0'));
    signal p10_key, p11_key, p12_key, p13_key, p14_key : std_logic := '0';
    signal p10_igm : signed(11 downto 0) := (others => '0');
    signal p10_fa : unsigned(14 downto 0) := (others => '0');
    signal p10_kf : unsigned(18 downto 0) := (others => '0');
    signal p10_fn : std_logic := '0';
    signal p11_igm : unsigned(8 downto 0) := (others => '0');
    signal p11_fr : signed(15 downto 0) := (others => '0');
    signal p14_r  : unsigned(7 downto 0) := (others => '0');
    signal p14_gb : std_logic_vector(15 downto 0) := (others => '0');
    signal p15_yr, p15_yg, p15_yb, p15_cr, p15_cg, p15_cb, p15_br, p15_bg, p15_bb : unsigned(18 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: one [1 2 2 2 2 2 2 2 1]/16 box on Y, U and V; the 8-back taps from a block-RAM ring;
    -- the syncs ride a second ring of C_LAT
    type t_cr is array (0 to 255) of std_logic_vector(29 downto 0);
    signal cr : t_cr := (others => (others => '0'));
    type t_sr is array (0 to 255) of std_logic_vector(2 downto 0);
    signal sr : t_sr := (others => (others => '1'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of cr : signal is true;
    attribute no_rw_check of sr : signal is true;
    signal rc     : unsigned(7 downto 0) := (others => '0');
    signal cr_q, cr_f : std_logic_vector(29 downto 0) := (others => '0');
    signal sr_q, sr_f : std_logic_vector(2 downto 0) := (others => '1');
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_boy, s_bou, s_bov : unsigned(13 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19, s_y20, s_u20, s_v20, s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
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
            r_ex_q <= C_YK_EXP2(to_integer(r_ex_a));
            r_v_q  <= C_YK_V(to_integer(r_v_a));
            r_r_q  <= C_YK_R(to_integer(r_r_a));
            r_gb_q <= C_YK_GB(to_integer(r_gb_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_t  : unsigned(35 downto 0);
        variable v_u  : unsigned(9 downto 0);
        variable v_ab : signed(10 downto 0);
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
            -- Rise: bipolar, +-8 deadband
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
                s_mir  <= registers_in(6)(0);
                s_inv  <= registers_in(6)(1);
                s_slip <= registers_in(6)(2);
                s_pal  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                c_fc   <= c_fc + 1;
                if c_s6 = 5 then c_s6 <= (others => '0'); c_ssd <= f_xs16(c_ssd); else c_s6 <= c_s6 + 1; end if;
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
            -- (stores at load + 18).  The slot number is decoded at 30 + 24 k and the operands pre-muxed at
            -- 31 + 24 k, so no zc decode sits on the multiplier's path.
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
                    when 366 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 390 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 414 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 438 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when 462 => z_sel <= to_unsigned(18, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(z_a, 36);                z_fb <= to_unsigned(205, 16);      -- eggs exponent
                    when 1  => z_fa <= to_unsigned(C_KH0, 36);         z_fb <= z_mt;                      -- KH
                    when 2  => z_fa <= resize(z_a, 36);                z_fb <= to_unsigned(218, 16);      -- rows exponent
                    when 3  => z_fa <= to_unsigned(C_KV0, 36);         z_fb <= z_mt;                      -- KV
                    when 4  =>                                                                            -- rings exponent
                        z_fa <= resize(z_a, 36);
                        if z_hi = '1' then z_fb <= to_unsigned(207, 16); else z_fb <= to_unsigned(193, 16); end if;
                    when 5  => z_fa <= resize(c_kh, 36);               z_fb <= resize(C_XC2 - c_xst, 16); -- a0
                    when 6  => z_fa <= resize(c_kv, 36);               z_fb <= to_unsigned(950, 16);      -- 950 KV
                    when 7  => z_fa <= resize(s_vm, 36);               z_fb <= resize(s_vm, 16);          -- d^2
                    when 8  => z_fa <= resize(z_t(17 downto 0), 36);   z_fb <= to_unsigned(1748, 16);
                    when 9  => z_fa <= resize(z_t(14 downto 0), 36);   z_fb <= c_kv;                      -- speed
                    when 10 =>                                                                            -- |yolk|
                        z_fa <= resize(z_a, 36);
                        if z_hi = '1' then z_fb <= to_unsigned(922, 16); else z_fb <= to_unsigned(1434, 16); end if;
                    when 11 => z_fa <= resize(z_t(10 downto 0), 36);   z_fb <= resize(c_gm, 16);          -- yolk gm
                    when 12 => z_fa <= to_unsigned(922, 36);           z_fb <= resize(c_gm, 16);          -- KIm
                    when 14 => z_fa <= to_unsigned(1382, 36);          z_fb <= resize(c_gm, 16);          -- KIr
                    when 15 => z_fa <= resize(s_g6(12 downto 3), 36);  z_fb <= resize(s_g6(12 downto 3), 16); -- k6^2
                    when 16 => z_fa <= resize(z_t(19 downto 4), 36);   z_fb <= to_unsigned(105, 16);      -- etch gain
                    when 17 =>                                                                            -- top band
                        if z_hi = '1' then z_fa <= resize(z_a, 36); z_fb <= to_unsigned(473, 16);
                        else z_fa <= resize(z_a, 36); z_fb <= to_unsigned(264, 16); end if;
                    when others =>                                                                        -- bottom band
                        if z_hi = '1' then z_fa <= resize(z_a, 36); z_fb <= to_unsigned(421, 16);
                        else z_fa <= resize(z_a, 36); z_fb <= to_unsigned(310, 16); end if;
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
                    -- scroll (last field's step); line-start constants
                    when 20 =>
                        if s_vneg = '1' then c_scr <= c_scr - resize(c_spd, 24);
                        else c_scr <= c_scr + resize(c_spd, 24); end if;
                        if s_x0 > C_XLH then c_xst <= resize(s_x0, 12); else c_xst <= to_unsigned(C_XLH, 12); end if;
                    when 28 =>                        -- eggs: a = |k1 - 512|
                        v_s := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        z_neg <= v_s(10); v_ab := abs(v_s); z_a <= unsigned(v_ab(9 downto 0));
                    when 50  =>                       -- e = +-(a * 205 >> 8): +-1.6 octaves
                        v_e := signed(resize(f_acc(17 downto 8), 12));
                        if z_neg = '1' then v_e := -v_e; end if;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 53  => z_mt <= r_ex_q;
                        v_s := signed(resize(s_g2(12 downto 3), 11)) - 512;
                        z_neg <= v_s(10); v_ab := abs(v_s); z_a <= unsigned(v_ab(9 downto 0));
                    when 74  =>                       -- KH = KH0 mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when -2     => c_kh <= f_acc(32 downto 17);
                            when -1     => c_kh <= f_acc(31 downto 16);
                            when 0      => c_kh <= f_acc(30 downto 15);
                            when others => c_kh <= f_acc(29 downto 14);
                        end case;
                    when 98  =>                       -- rows: e = +-(a * 218 >> 8): +-1.7 octaves
                        v_e := signed(resize(f_acc(17 downto 8), 12));
                        if z_neg = '1' then v_e := -v_e; end if;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 101 => z_mt <= r_ex_q;
                        -- rings: a = |p - 390|, slope by side
                        v_s := signed(resize(s_gp(12 downto 3), 11)) - 390;
                        z_neg <= v_s(10); v_ab := abs(v_s); z_a <= unsigned(v_ab(9 downto 0));
                        z_hi <= not v_s(10);
                    when 122 =>
                        case to_integer(z_sh) is
                            when -2     => c_kv <= f_acc(32 downto 17);
                            when -1     => c_kv <= f_acc(31 downto 16);
                            when 0      => c_kv <= f_acc(30 downto 15);
                            when others => c_kv <= f_acc(29 downto 14);
                        end case;
                    when 146 =>                       -- e (Q8 octaves) = -294 .. 512
                        v_e := signed(resize(f_acc(17 downto 8), 12));
                        if z_neg = '1' then v_e := -v_e; end if;
                        z_e <= v_e;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_shp <= v_e(10 downto 8);
                    when 149 =>                       -- gm = mt >> (7 - sh): 115 .. 1024 (Q8)
                        case to_integer(z_shp) is
                            when -2     => c_gm <= resize(r_ex_q(15 downto 9), 11);
                            when -1     => c_gm <= resize(r_ex_q(15 downto 8), 11);
                            when 0      => c_gm <= resize(r_ex_q(15 downto 7), 11);
                            when 1      => c_gm <= resize(r_ex_q(15 downto 6), 11);
                            when others => c_gm <= r_ex_q(15 downto 5);
                        end case;
                    when 172 =>
                        case to_integer(s_xs) is
                            when 4      => c_ks <= shift_left(resize(c_kh, 18), 2);
                            when 3      => c_ks <= shift_left(resize(c_kh, 18), 1) + resize(c_kh, 18);
                            when others => c_ks <= shift_left(resize(c_kh, 18), 1);
                        end case;
                    when 170 =>                       -- a0 = (c_xst - XC2) KH
                        c_a0 <= -signed(resize(f_acc(27 downto 0), 29));
                    when 194 => c_ps <= c_scr - f_acc(23 downto 0);                   -- psi at line 0
                    when 218 => z_t <= resize(f_acc(17 downto 0), 20);
                    when 242 => z_t <= resize(f_acc(28 downto 14), 20);
                    when 266 =>
                        c_spd <= f_acc(31 downto 10);
                        -- yolk: a = |k3 - 512|; left = shrink (yolk < 0)
                        v_s := signed(resize(s_g3(12 downto 3), 11)) - 512;
                        c_yn <= v_s(10); v_ab := abs(v_s); z_a <= unsigned(v_ab(9 downto 0));
                        z_hi <= not v_s(10);
                    when 290 => z_t <= resize(f_acc(20 downto 10), 20);
                    when 314 =>
                        if c_yn = '1' then c_ygm <= -signed(resize(f_acc(21 downto 8), 14));
                        else c_ygm <= signed(resize(f_acc(21 downto 8), 14)); end if;
                    when 338 =>
                        c_ki3m <= signed(resize(f_acc(20 downto 8), 16)) + 4198;                 -- KIm + 3072
                        if s_inv = '1' then c_kcmp <= not (signed(resize(f_acc(20 downto 8), 16)) + 563);  -- KIm - cK
                        else c_kcmp <= to_signed(C_CK, 16); end if;
                    when 386 => c_ki3r <= signed(resize(f_acc(21 downto 8), 16)) + 3892;     -- KIr + 2 c0 + 3072
                    when 410 => z_t <= f_acc(19 downto 0);
                    when 434 =>                       -- etch gain (Q8 per luma step); the Osc x6
                        if s_src = '1' then c_eg <= resize(f_acc(23 downto 11), 14) + shift_left(resize(f_acc(23 downto 11), 14), 1);
                        else c_eg <= resize(f_acc(23 downto 12), 14); end if;
                        c_ee <= (others => '0');
                        -- bands: k5 <= 512 scales the still's bands, above grows them
                        if s_g5(12 downto 3) > 512 then z_a <= s_g5(12 downto 3) - 512; z_hi <= '1';
                        else z_a <= s_g5(12 downto 3); z_hi <= '0'; end if;
                    when 436 | 437 | 438 | 439 | 440 | 441 | 442 =>   -- normalise the etch gain to 7 bits << ee
                        if c_eg(13 downto 7) /= "0000000" then c_eg <= shift_right(c_eg, 1); c_ee <= c_ee + 1; end if;
                    when 443 => c_em <= c_eg(6 downto 0);
                    when 458 =>
                        if z_hi = '1' then c_wt <= 264 + resize(f_acc(19 downto 10), 11);
                        else c_wt <= resize(f_acc(19 downto 9), 11); end if;
                    when 482 =>
                        if z_hi = '1' then c_wb <= 770 - resize(f_acc(19 downto 10), 11);
                        else c_wb <= 1080 - resize(f_acc(19 downto 9), 11); end if;
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
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
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;
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
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line on one serial multiplier (16 iterations,
    -- result at load + 17):
    --   psi = vy' KV + ps (vy' folded for Mirror) -> V ROM -> VG, VR
    --   l_vgm = (VG - cK) gm + cK - yolk gm,  l_vr = (VR - c0) gm - yolk gm
    --   (Invert: the constants KI - l and the pixel path subtracts), the slip shift times KH
    --   the window distances, the noise seed (pairs of lines), the bar and slip bands (virtual lines)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d  : signed(11 downto 0);
        variable v_w  : signed(12 downto 0);
        variable v_p  : signed(15 downto 0);
        variable v_r  : unsigned(15 downto 0);
        variable v_i  : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_ln <= '0'; e_l4 <= "00";
                e_seed <= c_fc xor x"5a3c";
                e_brs  <= f_xs16(c_fc + 77);
                e_srs  <= c_ssd;
                e_bct  <= (others => '0');
                e_sct  <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                e_ln <= not e_ln; e_l4 <= e_l4 + 1;
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    -- fold for Mirror (about virtual line 540)
                    if s_mir = '1' and e_vy > 138240 then e_vf <= 276480 - e_vy; else e_vf <= e_vy; end if;
                    -- noise seed: a new one every fourth line
                    if e_l4 = "00" then e_seed <= f_xs16(e_seed); end if;
                    -- bar band (6 .. 37 virtual lines, changing on even lines only): a new offset when it runs out
                    if e_ln = '0' then e_brs <= f_xs16(e_brs); end if;
                    if e_ln = '1' then
                        null;
                    elsif e_bct <= 0 then
                        e_bct <= signed(resize(shift_left(resize(e_brs(4 downto 0), 13), 8), 14))
                                 + to_signed(1536, 14) - signed(resize(s_ys, 14));
                        v_i := signed(resize(e_brs(15 downto 8), 10)) + signed(resize(e_brs(7 downto 0), 10)) - 255;
                        e_bar <= resize(shift_left(resize(v_i, 11), 2), 11);
                    else
                        e_bct <= e_bct - signed(resize(s_ys & '0', 14));
                    end if;
                    -- slip band (16 .. 79 virtual lines, changing on even lines only)
                    if e_ln = '0' then e_srs <= f_xs16(e_srs); end if;
                    if e_ln = '1' then
                        null;
                    elsif e_sct <= 0 then
                        e_sct <= signed(resize(shift_left(resize(e_srs(5 downto 0), 14), 8), 15))
                                 + to_signed(4096, 15) - signed(resize(s_ys, 15));
                        if e_srs(15 downto 13) < 5 then
                            v_i := signed(resize(e_srs(13 downto 7), 10)) + signed(resize(e_srs(6 downto 0), 10)) - 127;
                            e_sh <= resize(shift_left(resize(v_i, 11), 2), 11);
                        else
                            e_sh <= (others => '0');
                        end if;
                    else
                        e_sct <= e_sct - signed(resize(s_ys & '0', 15));
                    end if;
                when 2 =>
                    m_a <= resize(e_vf, 36); m_b <= c_kv; m_acc <= (others => '0');
                    -- window distances (clamped to +-63 lines), per pair of lines (no 1-row slivers at the edges)
                    if e_ln = '0' then
                        v_w := signed(resize(c_wt, 13)) - signed(resize(e_vf(18 downto 8), 13));
                        if v_w > 63 then n_dt <= to_signed(63, 7); elsif v_w < -64 then n_dt <= to_signed(-64, 7);
                        else n_dt <= v_w(6 downto 0); end if;
                        v_w := signed(resize(c_wb, 13)) - signed(resize(e_vf(18 downto 8), 13));
                        if v_w > 63 then n_db <= to_signed(63, 7); elsif v_w < -64 then n_db <= to_signed(-64, 7);
                        else n_db <= v_w(6 downto 0); end if;
                    end if;
                    n_seed <= e_seed;
                    n_bar  <= e_bar;
                when 19 =>
                    v_r := m_acc(31 downto 16) + c_ps(23 downto 8);
                    r_v_a <= v_r(15 downto 7);
                when 22 =>
                    e_vg <= signed(r_v_q(23 downto 12));
                    e_vr <= signed(r_v_q(11 downto 0));
                when 23 =>
                    v_d := e_vg - C_CK;
                    e_sg <= v_d(11);
                    m_a <= resize(unsigned(abs(v_d)), 36); m_b <= resize(c_gm, 16); m_acc <= (others => '0');
                when 40 =>
                    v_p := signed(resize(m_acc(23 downto 8), 16));
                    if e_sg = '1' then v_p := -v_p; end if;
                    v_p := v_p + C_CK - resize(c_ygm, 16);
                    n_vgm <= v_p;
                    v_d := e_vr - C_C0;
                    e_sg <= v_d(11);
                    m_a <= resize(unsigned(abs(v_d)), 36); m_b <= resize(c_gm, 16); m_acc <= (others => '0');
                when 57 =>
                    v_p := signed(resize(m_acc(23 downto 8), 16));
                    if e_sg = '1' then v_p := -v_p; end if;
                    v_p := v_p - resize(c_ygm, 16);
                    n_vr <= v_p;
                    e_sg <= e_sh(10);
                    m_a <= resize(unsigned(abs(e_sh)), 36); m_b <= c_kh; m_acc <= (others => '0');
                when 74 =>
                    if s_slip = '0' then
                        n_a0 <= c_a0;
                    elsif e_sg = '1' then
                        n_a0 <= c_a0 - signed(resize(m_acc(27 downto 0), 29));
                    else
                        n_a0 <= c_a0 + signed(resize(m_acc(27 downto 0), 29));
                    end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: x (h), the H accumulator acc = (x - XC2) KH (held left of the
    -- hold point; |acc| is the mirrored phase), the R lean level (x / 240 h,
    -- 16 steps), the noise runs (an xorshift per pixel, a new value every
    -- 8 .. 39 virtual px; the seed is held for 4 lines)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_rc : signed(8 downto 0);
        variable v_n  : signed(9 downto 0);
        variable v_ga : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x   <= resize(s_x0, 12);
                b_px  <= (others => '0');
                b_acc <= n_a0;
                b_ga  <= (others => '0');
                if s_x0 = 0 then b_gk <= (others => '0'); else b_gk <= to_unsigned(2, 4); end if;
                if s_x0 < C_XLH then b_hold <= '1'; else b_hold <= '0'; end if;
                b_lf  <= n_seed;
                b_rc  <= (others => '0');
                b_n   <= (others => '0');
                l_vgm <= n_vgm; l_vr <= n_vr;
                l_dt <= n_dt; l_db <= n_db; l_bar <= n_bar; l_inv <= s_inv;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x + s_xs;
                if b_x + s_xs > c_xst then b_acc <= b_acc + signed(resize(c_ks, 29)); end if;
                if b_x + s_xs < C_XLH then b_hold <= '1'; else b_hold <= '0'; end if;
                v_ga := resize(b_ga, 9) + s_xs;
                if v_ga >= 240 then
                    b_ga <= resize(v_ga - 240, 8);
                    if b_gk /= 15 then b_gk <= b_gk + 1; end if;
                else
                    b_ga <= v_ga(7 downto 0);
                end if;
                b_lf <= f_xs16(b_lf);
                v_rc := b_rc - signed(resize(s_xs, 9));
                if v_rc <= 0 then
                    b_rc <= signed(resize(b_lf(15 downto 11) & '0', 9)) + 16;
                    v_n := signed(resize(b_lf(4 downto 0), 10)) + signed(resize(b_lf(9 downto 5), 10)) - 31;
                    if b_lf(13 downto 10) = "0000" then v_n := shift_left(v_n, 2); end if;
                    if s_slip = '1' then v_n := shift_left(v_n, 1); end if;
                    b_n <= v_n;
                else
                    b_rc <= v_rc;
                end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_t   : unsigned(9 downto 0);
        variable v_ns  : signed(9 downto 0);
        variable v_e21 : unsigned(21 downto 0);
        variable v_es  : signed(15 downto 0);
        variable v_ng  : signed(11 downto 0);
        variable v_aa  : signed(28 downto 0);
        variable v_al  : signed(8 downto 0);
        variable v_af  : signed(15 downto 0);
        variable v_msk : signed(15 downto 0);
        variable v_off : signed(15 downto 0);
        variable v_s   : signed(15 downto 0);
        variable v_rm  : unsigned(20 downto 0);
        variable v_ra  : unsigned(10 downto 0);
        variable v_r, v_g, v_b : unsigned(7 downto 0);
        variable v_y   : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            v_msk := (others => l_inv);
            -- 2: |acc|, the noise (and its window raggedness: +-12 lines), luma history (camera: 0 off the line,
            -- the Osc: its mean)
            v_aa := abs(b_acc);
            p2_aa <= unsigned(v_aa(27 downto 0));
            p2_n  <= b_n;
            if b_n > 48 then v_ns := to_signed(48, 10); elsif b_n < -48 then v_ns := to_signed(-48, 10); else v_ns := b_n; end if;
            p2_ns <= resize(shift_right(v_ns, 2), 7);
            p2_h  <= b_hold;
            p_gk(2) <= b_gk;
            for i in 3 to 9 loop p_gk(i) <= p_gk(i - 1); end loop;
            if s_gav_sr(0) = '1' then p_lm(0) <= unsigned(s_lb_q);
            elsif s_src = '1' then p_lm(0) <= to_unsigned(110, 8); else p_lm(0) <= (others => '0'); end if;
            for i in 1 to 3 loop p_lm(i) <= p_lm(i - 1); end loop;
            -- 3: phase, the centre notch, the window, luma 4-box (pairs)
            p3_u  <= p2_aa(23 downto 0) + to_unsigned(C_UC, 24);
            if p2_aa < C_CPL then p3_pl <= '1'; else p3_pl <= '0'; end if;
            if p2_ns < l_dt or p2_ns >= l_db then p3_win <= '1'; else p3_win <= '0'; end if;
            p3_ls0 <= resize(p_lm(0), 9) + p_lm(1); p3_ls1 <= resize(p_lm(2), 9) + p_lm(3);
            p3_h <= p2_h; p3_n <= p2_n;
            -- 4: T (0 on the egg columns, 1023 between), half-depth notch at the centre; the yellow-strip band
            if p3_u(23) = '0' then v_t := p3_u(22 downto 13); else v_t := not p3_u(22 downto 13); end if;
            if p3_pl = '1' then v_t := resize(shift_right(resize(v_t, 11) + 1023, 1), 10); end if;
            p4_t <= v_t;
            if v_t >= 676 and v_t < 819 then p_st(4) <= '1'; else p_st(4) <= '0'; end if;
            for i in 5 to 14 loop p_st(i) <= p_st(i - 1); end loop;
            p4_ls <= resize(p3_ls0, 10) + p3_ls1;
            p4_h <= p3_h; p4_n <= p3_n;
            p_win(4) <= p3_win;
            for i in 5 to 9 loop p_win(i) <= p_win(i - 1); end loop;
            -- 5: T gm partial products; luma (camera adds: black = the still; the Osc: centred)
            p5_p1a <= resize(p4_t * c_gm(5 downto 0), 16);
            p5_p1b <= resize(p4_t * c_gm(10 downto 6), 15);
            if s_src = '1' then p5_lm <= signed(resize(p4_ls(9 downto 2), 9)) - 110;
            else p5_lm <= signed(resize(p4_ls(9 downto 2), 9)); end if;
            p5_h <= p4_h; p5_n <= p4_n;
            -- 6: product; etch partials (|luma| x gain); noise / bar terms (the hold zone: the bar offset only)
            p6_p1 <= resize(p5_p1a, 21) + shift_left(resize(p5_p1b, 21), 6);
            v_al := abs(p5_lm);
            p6_ea <= unsigned(v_al(7 downto 0)) * c_em;
            p6_en <= p5_lm(8);
            p6_h <= p5_h;
            v_ng := resize(p5_n, 12);
            if p_win(5) = '0' then v_ng := shift_right(v_ng, 2) + shift_right(v_ng, 4); end if;
            if p5_h = '1' then
                p6_bg <= resize(l_bar, 12);
                p6_br <= resize(l_bar, 12);
            else
                p6_bg <= v_ng;
                p6_br <= shift_right(resize(p5_n, 12), 2);
            end if;
            -- 7: E = +-(luma gain) >> 8; T gm >> 8 (Q10)
            v_e21 := shift_left(resize(p6_ea, 22), to_integer(c_ee));
            v_es := signed(resize(v_e21(21 downto 8), 16));
            if p6_en = '1' then v_es := -v_es; end if;
            p7_e  <= v_es(13 downto 0);
            p7_p1 <= signed(resize(p6_p1(20 downto 8), 16));
            p7_bg <= p6_bg; p7_br <= p6_br; p7_h <= p6_h;
            -- 8: T term + the line terms (the hold zone: the held T and the bar offset only, so its bars change on
            -- line pairs and are 2 rows at least); E + noise / bar
            if p7_h = '1' then
                p8_qgm <= p7_p1; p8_qr <= p7_p1;
            else
                p8_qgm <= p7_p1 + l_vgm; p8_qr <= p7_p1 + l_vr;
            end if;
            p8_eg <= resize(p7_e, 16) + resize(p7_bg, 16);
            p8_er <= resize(p7_e, 16) + resize(p7_br, 16);
            -- 9: the fields
            p9_fgm <= p8_qgm + p8_eg; p9_fr <= p8_qr + p8_er;
            -- 10: the key (Invert: KI - F > cK, as not F > not (KI - cK)); the G index (Invert: KI3 + not F);
            -- R lean partials (|F_R rel| x level)
            if p_win(9) = '1' and (p9_fgm xor v_msk) > c_kcmp then p10_key <= '1'; else p10_key <= '0'; end if;
            if l_inv = '1' then v_off := c_ki3m; else v_off := to_signed(3072, 16); end if;
            v_s := (p9_fgm xor v_msk) + v_off;
            p10_igm <= v_s(15 downto 4);
            v_af := abs(p9_fr);
            p10_fa <= unsigned(v_af(14 downto 0));
            p10_kf <= unsigned(v_af(14 downto 0)) * p_gk(9);
            p10_fn <= p9_fr(15);
            -- 11: fold the G index (a triangle of period 1024: no carries); F_R = +-|F| (17 + 2k) / 32 + c0
            p11_igm <= f_fold(p10_igm);
            v_rm := shift_left(resize(p10_fa, 21), 4) + resize(p10_fa, 21) + shift_left(resize(p10_kf, 21), 1);
            v_s := signed(resize(v_rm(20 downto 5), 16));
            if p10_fn = '1' then v_s := -v_s; end if;
            p11_fr <= v_s + C_C0;
            p11_key <= p10_key;
            -- 12: R index and fold; ROM addresses (the white layer: R, G, B all read the G field)
            if l_inv = '1' then v_off := c_ki3r; else v_off := to_signed(3072, 16); end if;
            v_s := (p11_fr xor v_msk) + v_off;
            v_ra(10) := s_pal; v_ra(9) := p11_key;
            if p11_key = '1' then
                v_ra(8 downto 0) := p11_igm;
                r_r_a <= v_ra; r_gb_a <= v_ra;
            else
                v_ra(8 downto 0) := f_fold(v_s(15 downto 4));
                r_r_a <= v_ra;
                v_ra(8 downto 0) := p11_igm;
                r_gb_a <= v_ra;
            end if;
            p12_key <= p11_key;
            -- 13: ROM data (EBR); 14: fabric copy
            p13_key <= p12_key;
            p14_r <= unsigned(r_r_q); p14_gb <= r_gb_q; p14_key <= p13_key;
            -- 15: the white layer's yellow strips (B x 3/8 where T sits in the strip band and G is high);
            -- RGB -> YUV terms (x256, white-balanced: Y 256 512 96, Cr 448 -384 -64, Cb -160 -288 448), the
            -- chroma offsets 512 x 256 folded in as bit 8 of R and B
            v_r := p14_r; v_g := unsigned(p14_gb(15 downto 8)); v_b := unsigned(p14_gb(7 downto 0));
            if p14_key = '1' and p_st(14) = '1' and v_g > 160 then
                v_b := shift_right(v_b, 2) + shift_right(v_b, 3);
            end if;
            p15_yr <= shift_left(resize(v_r, 19), 8);
            p15_yg <= shift_left(resize(v_g, 19), 9);
            p15_yb <= shift_left(resize(v_b, 19), 6) + shift_left(resize(v_b, 19), 5);
            p15_cr <= shift_left(resize('1' & v_r, 19), 9) - shift_left(resize(v_r, 19), 6);
            p15_cg <= shift_left(resize(v_g, 19), 8) + shift_left(resize(v_g, 19), 7);
            p15_cb <= shift_left(resize(v_b, 19), 6);
            p15_bb <= shift_left(resize('1' & v_b, 19), 9) - shift_left(resize(v_b, 19), 6);
            p15_br <= shift_left(resize(v_r, 19), 7) + shift_left(resize(v_r, 19), 5);
            p15_bg <= shift_left(resize(v_g, 19), 8) + shift_left(resize(v_g, 19), 5);
            -- 16: sums >> 8 (in range by construction: Y 64 .. 924, chroma 66 .. 958); black off the generated avid
            if s_gav_sr(13) = '0' then
                s_y16 <= to_unsigned(64, 10); s_u16 <= to_unsigned(512, 10); s_v16 <= to_unsigned(512, 10);
            else
                v_rm := resize(p15_yr, 21) + p15_yg + p15_yb;
                s_y16 <= v_rm(17 downto 8) + 64;
                v_rm := resize(p15_cr, 21) - p15_cg - p15_cb;
                s_u16 <= v_rm(17 downto 8);
                v_rm := resize(p15_bb, 21) - p15_br - p15_bg;
                s_v16 <= v_rm(17 downto 8);
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(13);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Rings: the box's 8-back taps (written at rc + 6, RAM register, fabric
    -- register = the input of 8 clocks before; never the write address) and the
    -- syncs (written at rc + 18: C_LAT clocks)
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rc <= rc + 1;
            cr(to_integer(rc + 6)) <= std_logic_vector(s_y16) & std_logic_vector(s_u16) & std_logic_vector(s_v16);
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            sr(to_integer(rc + 18)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through one [1 2 2 2 2 2 2 2 1]/16 box
    -- (two adjacent 8-sums; the still's softness and the HD HDMI bar fix),
    -- luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 17: running 8-sums (snap back to black once 8 blank samples are in, so a glitched sample can't
            -- offset the colours for good)
            if s_bah = "00000000" then
                s_bay <= to_unsigned(512, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y16 - unsigned(cr_f(29 downto 20));
                s_bau <= s_bau + s_u16 - unsigned(cr_f(19 downto 10));
                s_bav <= s_bav + s_v16 - unsigned(cr_f(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- 18: [1 2 2 2 2 2 2 2 1] = two adjacent 8-sums
            s_boy <= resize(s_bay, 14) + s_bayd;
            s_bou <= resize(s_bau, 14) + s_baud;
            s_bov <= resize(s_bav, 14) + s_bavd;
            -- 19: / 16; black off the generated avid
            if s_gav_sr(16) = '1' then
                s_y19 <= s_boy(13 downto 4); s_u19 <= s_bou(13 downto 4); s_v19 <= s_bov(13 downto 4);
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
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
    data_out.hsync_n <= sr_f(2);
    data_out.vsync_n <= sr_f(1);
    data_out.field_n <= sr_f(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture yolk;
