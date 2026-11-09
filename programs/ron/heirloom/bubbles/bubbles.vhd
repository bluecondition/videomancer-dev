-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Bubbles v0.1.0 -- Heirloom (video phase), from the clip WIN_20240105_20_46_57 at 0.5 s.
--
-- The still: three columns of rainbow bubble lenses (937 px apart) stacked over a mint-to-cyan ground, each
-- ringed by a soft dark halo, with the clip's wavy rainbow curtain just entering the right-hand corners.  It is
-- one analog field: a screen-fixed cusped vertical parabola wave (period 536 lines, seams at 242 / 778), a slow
-- vertical cosine (1050 lines) that rolls UP at ~132 px/s, and a horizontal parabola per column:
--   F1 = 1.98 + Vs + Vf - dx^2 / 50000      bubble where F1 > 1, halo dip at F1 = 0, ground red = Vs
--   F2 = Vf + 1.12 - dx^2 / 125000           the bubble (and the curtain) are RGB-inverted where F2 > 0
--   curtain = x > 1995 - 591 cos(2 pi (y - 10) / 1050)   (screen-fixed; P12 sweeps it across)
-- The rainbow inside is a screen-fixed 512 px colour cycle, bent per line by the same slow cosine.  As the slow
-- cosine rolls up through the fixed slots the bubbles swell, shrink to a dark bell and come back (the clip).
--
--   K1  Bubbles   bubble level: none (0) .. the still (512) .. huge merged lenses (1023)
--   K2  Halo      halo ring distance: hugging the rim (0) .. the still (512) .. far out (1023)
--   K3  Columns   column spacing x2.5 (one column) .. 937 px (512) .. x0.4 (five columns)
--   K4  Rise      bipolar swell roll (+-8 deadband, square law): 756 = the clip's 2.2 lines / field up
--   K5  Rainbow   colour-cycle width x4 .. 512 px (512) .. x1/4 (candy stripes); the curtain's bands follow
--   K6  Camera    how far the source (camera luma or the Osc) swells the bubbles and bends the rainbow (256)
--   S7  Lens      Normal / Flip: the inversion key flipped (cyan/blue lenses, red crescents)
--   S8  Rings     Off / On: the halo repeats into nested contour rings over the ground
--   S9  Mirror    Off / On: the whole field read upside down (the lenses hang, the swell sinks)
--   S10 Ground    Mint / Night (a dark ground with glowing halos)
--   S11 Source    Video / Osc
--   P12 WAVE      the rainbow curtain: none (0) .. the still's corners (196) .. a full wavy curtain (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line
-- start (the next line's slot parabola, both cosines, the rainbow bend, the curtain edge, the ground colour,
-- the two phase starts; one serial multiplier), a beam second-order DDA for c dx^2 with a column wrap, a pixel
-- path of RGB arithmetic and an RGB -> YUV matrix.  Virtual 1080-line coordinates (SD shows the centre 4:3).
-- Generated avid; output conditioning (U and V through a [1 2 2 2 2 2 2 2 1]/16 box with the running-sum
-- resync, Y through [1 2 1]/4 on the same centre, luma 13/16) before the blanking gate.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.bubbles_pkg.all;
use work.all;

architecture bubbles of program_top is
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
    function f_sat16(v : signed) return signed is
    begin
        if v > 32767 then return to_signed(32767, 16);
        elsif v < -32768 then return to_signed(-32768, 16);
        else return v(15 downto 0); end if;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_rb_a : unsigned(8 downto 0) := (others => '0');
    signal r_rr_q, r_rg_q, r_rb_q : unsigned(7 downto 0) := (others => '0');
    signal r_cos_a : unsigned(7 downto 0) := (others => '0');
    signal r_cos_q : unsigned(15 downto 0) := (others => '0');
    signal r_ex_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q  : unsigned(15 downto 0) := (others => '0');
    signal r_hm_a  : unsigned(6 downto 0) := (others => '0');
    signal r_hm_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_mr_a, r_mg_a, r_mb_a : unsigned(7 downto 0) := (others => '0');
    signal r_mr_q, r_mg_q, r_mb_q : std_logic_vector(31 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 214 x 180 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(9 downto 0) := to_unsigned(306, 10);
    signal s_oys : unsigned(10 downto 0) := to_unsigned(363, 11);
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

    -- knobs: glided (Q3), raw rise, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(196 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(756, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_flip, s_rings, s_mirror, s_night, s_src : std_logic := '0';

    -- per-field constants
    signal c_lb   : signed(15 downto 0) := to_signed(4096, 16);      -- bubble level (Q12)
    signal c_lhb  : signed(15 downto 0) := to_signed(-4096, 16);     -- halo level - bubble level
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_neg  : std_logic := '0';
    signal c_ph   : unsigned(12 downto 0) := to_unsigned(1874, 13);  -- column period (h)
    signal c_mp   : unsigned(11 downto 0) := to_unsigned(937, 12);   -- column period (pixels)
    signal c_mh1  : signed(12 downto 0) := to_signed(468, 13);       -- last m before the wrap
    signal c_mw   : signed(12 downto 0) := to_signed(-468, 13);      -- m after the wrap
    signal c_m0   : signed(12 downto 0) := to_signed(-53, 13);       -- m at the line start
    signal c_he   : unsigned(6 downto 0) := to_unsigned(21, 7);      -- c * xs^2 (the DDA curvature)
    signal c_a0, c_aw : unsigned(25 downto 0) := (others => '0');
    signal c_d0, c_dw : signed(25 downto 0) := (others => '0');
    signal c_kr   : unsigned(16 downto 0) := to_unsigned(16384, 17); -- rainbow phase per h (2^24 per cycle)
    signal c_sr   : unsigned(18 downto 0) := to_unsigned(32768, 19); -- per pixel
    signal c_kw   : unsigned(17 downto 0) := to_unsigned(20992, 18); -- curtain phase per h
    signal c_sw   : unsigned(19 downto 0) := to_unsigned(41984, 20);
    signal c_q    : unsigned(9 downto 0) := (others => '0');         -- P12 above the still
    signal c_xw   : signed(13 downto 0) := to_signed(3989, 14);      -- curtain edge (h)
    signal c_gw   : unsigned(10 downto 0) := to_unsigned(1182, 11);  -- curtain swing (h)
    signal c_gk   : unsigned(8 downto 0) := (others => '0');         -- curtain kinks (h per unit Vf)
    signal c_vss  : unsigned(16 downto 0) := to_unsigned(15978, 17); -- cosine phase per output line
    signal c_vsn  : signed(17 downto 0) := to_signed(15978, 18);     -- the same, negated under Mirror
    signal c_phs  : unsigned(23 downto 0) := (others => '0');        -- swell roll phase
    signal c_vst  : unsigned(17 downto 0) := to_unsigned(35028, 18); -- roll step per field (24-bit phase)
    signal c_g6   : unsigned(5 downto 0) := to_unsigned(16, 6);      -- camera gain

    -- field serial multiplier
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : unsigned(23 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal z_d   : unsigned(8 downto 0) := (others => '0');
    signal z_dneg : std_logic := '0';
    signal z_t   : unsigned(23 downto 0) := (others => '0');

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
    signal e_vs, e_vq : unsigned(23 downto 0) := (others => '0');
    signal e_v    : signed(11 downto 0) := (others => '0');
    signal e_ssn, e_sqn : std_logic := '0';
    signal e_cs, e_cq : unsigned(15 downto 0) := (others => '0');
    signal e_vf, e_vsv : signed(15 downto 0) := (others => '0');
    signal e_tau  : signed(11 downto 0) := (others => '0');
    signal e_xb   : signed(13 downto 0) := (others => '0');
    signal e_tx   : signed(13 downto 0) := (others => '0');
    signal e_txn  : std_logic := '0';
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(31 downto 0) := (others => '0');
    signal n_s1, n_s2, n_sh : signed(15 downto 0) := (others => '0');
    signal n_xb   : signed(13 downto 0) := to_signed(8191, 14);
    signal n_phb, n_phw : unsigned(23 downto 0) := (others => '0');
    signal n_br, n_bg, n_bb : unsigned(7 downto 0) := (others => '0');

    -- beam
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_m   : signed(12 downto 0) := (others => '0');
    signal b_a   : unsigned(25 downto 0) := (others => '0');
    signal b_d   : signed(25 downto 0) := (others => '0');
    signal b_xh  : signed(13 downto 0) := (others => '0');
    signal b_phb, b_phw : unsigned(23 downto 0) := (others => '0');   -- the stage-5 pixel's phases
    signal l_s1, l_s2, l_sh : signed(15 downto 0) := (others => '0');
    signal l_xb  : signed(13 downto 0) := to_signed(8191, 14);
    signal l_br, l_bg, l_bb : unsigned(7 downto 0) := (others => '0');

    -- pixel path (the number is the stage)
    signal p2_a  : unsigned(14 downto 0) := (others => '0');
    signal p2_l  : unsigned(7 downto 0) := (others => '0');
    signal p3_gl : unsigned(13 downto 0) := (others => '0');
    signal p3_a  : unsigned(14 downto 0) := (others => '0');
    signal p4_ac : signed(16 downto 0) := (others => '0');
    signal p4_a04 : unsigned(14 downto 0) := (others => '0');
    signal p4_g4 : unsigned(11 downto 0) := (others => '0');
    signal p4_pc : unsigned(7 downto 0) := (others => '0');
    signal p5_f1, p5_f2, p5_d : signed(17 downto 0) := (others => '0');
    signal p5_pc : unsigned(7 downto 0) := (others => '0');
    type t_u2 is array (2 to 10) of unsigned(1 downto 0);
    signal pf_rz : t_u2 := (others => (others => '0'));
    type t_flag is array (2 to 10) of std_logic;
    signal pf_kw : t_flag := (others => '0');
    type t_flag2 is array (6 to 10) of std_logic;
    signal pf_k1, pf_inv, pf_ob : t_flag2 := (others => '0');
    type t_u2b is array (6 to 9) of unsigned(1 downto 0);
    signal pf_w : t_u2b := (others => (others => '0'));
    signal p8_cr, p8_cg, p8_cb : unsigned(7 downto 0) := (others => '0');
    signal p8_hm : std_logic_vector(26 downto 0) := (others => '0');
    signal p9_dr, p9_dg, p9_db : signed(8 downto 0) := (others => '0');
    signal p9_cr, p9_cg, p9_cb : unsigned(7 downto 0) := (others => '0');
    signal p9_hr, p9_hg, p9_hb : unsigned(7 downto 0) := (others => '0');
    signal p10_mr, p10_mg, p10_mb : unsigned(7 downto 0) := (others => '0');
    signal p10_cr, p10_cg, p10_cb : unsigned(7 downto 0) := (others => '0');
    signal p10_hr, p10_hg, p10_hb : unsigned(7 downto 0) := (others => '0');
    signal p13_mr, p13_mg, p13_mb : std_logic_vector(31 downto 0) := (others => '0');
    signal p14_y1, p14_yb : unsigned(11 downto 0) := (others => '0');
    signal p14_u1, p14_ub, p14_v1, p14_vg : signed(11 downto 0) := (others => '0');
    signal p15_y : unsigned(12 downto 0) := (others => '0');
    signal p15_u, p15_v : signed(12 downto 0) := (others => '0');
    signal s_y16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y17 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent 8-sums), luma [1 2 1]/4 on the same centre
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_ly : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_cau, s_cav : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y19 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
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
            r_rr_q  <= C_BB_RR(to_integer(r_rb_a));
            r_rg_q  <= C_BB_RG(to_integer(r_rb_a));
            r_rb_q  <= C_BB_RB(to_integer(r_rb_a));
            r_cos_q <= C_BB_COS(to_integer(r_cos_a));
            r_ex_q  <= C_BB_EXP(to_integer(r_ex_a));
            r_hm_q  <= C_BB_HM(to_integer(r_hm_a));
            r_mr_q  <= C_BB_MR(to_integer(r_mr_a));
            r_mg_q  <= C_BB_MG(to_integer(r_mg_a));
            r_mb_q  <= C_BB_MB(to_integer(r_mb_a));
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
        variable v_k  : signed(11 downto 0);
        variable v_t  : signed(15 downto 0);
        variable v_p  : unsigned(31 downto 0);
        variable v_x  : signed(25 downto 0);
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
            -- rise: |k4 - 512| - 8 (deadband +-8), sign
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
                s_flip  <= registers_in(6)(0);
                s_rings <= registers_in(6)(1);
                s_mirror <= registers_in(6)(2);
                s_night <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(612, 10);
                                   c_he <= to_unsigned(84, 7);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(459, 10);
                                   c_he <= to_unsigned(47, 7);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(306, 10);
                                   c_he <= to_unsigned(21, 7);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 21) * to_unsigned(363, 9), 8), 11);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters
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
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 4 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 48 + 24 k, 16 iterations, result stable from
            -- load + 17 (stored at load + 18).  The slot number is decoded at 46 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 47 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 46  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 70  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 94  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 142 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 166 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 190 => z_sel <= to_unsigned(6, 5);  z_pre <= '1';
                    when 214 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 238 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 262 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 286 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 334 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 358 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 382 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 406 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 430 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(z_d, 24);          z_fb <= to_unsigned(169, 16);         -- |512 - k3| * 169
                    when 1  => z_fa <= to_unsigned(1874, 24);    z_fb <= r_ex_q;                       -- period
                    when 2  => z_fa <= resize(c_ph, 24);         z_fb <= to_unsigned(21846, 16);       -- / 3
                    when 4  => z_fa <= resize(unsigned(abs(c_m0)), 24); z_fb <= resize(unsigned(abs(c_m0)), 16);
                    when 5  => z_fa <= z_t;                      z_fb <= resize(c_he, 16);             -- he m0^2
                    when 6  => z_fa <= resize(unsigned(abs(shift_left(resize(c_m0, 14), 1) + 1)), 24);
                               z_fb <= resize(c_he, 16);                                               -- he (2 m0 + 1)
                    when 7  => z_fa <= resize(unsigned(abs(c_mw)), 24); z_fb <= resize(unsigned(abs(c_mw)), 16);
                    when 8  => z_fa <= z_t;                      z_fb <= resize(c_he, 16);             -- he mw^2
                    when 9  => z_fa <= resize(unsigned(abs(shift_left(resize(c_mw, 14), 1) + 1)), 24);
                               z_fb <= resize(c_he, 16);                                               -- he (2 mw + 1)
                    when 10 => z_fa <= resize(c_kr, 24);         z_fb <= to_unsigned(328, 16);         -- kw = kr * 1.28
                    when 12 => z_fa <= resize(s_gp(12 downto 3), 24); z_fb <= to_unsigned(461, 16);    -- curtain edge
                    when 13 => z_fa <= resize(c_q, 24);          z_fb <= to_unsigned(817, 16);         -- curtain swing
                    when 14 => z_fa <= resize(c_q, 24);          z_fb <= to_unsigned(396, 16);         -- curtain kinks
                    when 15 => z_fa <= resize(s_ys, 24);         z_fb <= to_unsigned(15978, 16);       -- cosine step
                    when others => z_fa <= resize(s_vm, 24);     z_fb <= resize(s_vm, 16);             -- rise^2
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 32); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion: roll the swell (last field's step)
                    when 20 =>
                        if s_vneg = '1' then c_phs <= c_phs - resize(c_vst, 24);
                        else c_phs <= c_phs + resize(c_vst, 24); end if;
                    -- bubble and halo levels (Q12)
                    when 30 =>
                        v_k := signed(resize(s_g1(12 downto 3), 12)) - 512;
                        v_t := shift_left(resize(v_k, 16), 4) + shift_left(resize(v_k, 16), 1);
                        c_lb <= to_signed(4096, 16) - v_t - resize(v_k, 16);
                    when 31 =>
                        v_k := signed(resize(s_g2(12 downto 3), 12)) - 512;
                        if v_k < 0 then
                            v_t := shift_left(resize(-v_k, 16), 4) - resize(-v_k, 16);          -- 15 (512 - k2)
                            c_lhb <= to_signed(-4096, 16) + v_t;
                        else
                            v_t := shift_left(resize(v_k, 16), 4) + resize(v_k, 16);   -- 17 (k2 - 512)
                            c_lhb <= to_signed(-4096, 16) - v_t;
                        end if;
                        -- columns exponent: |512 - k3| for the * 169 slot
                        v_k := to_signed(512, 12) - signed(resize(s_g3(12 downto 3), 12));
                        z_neg <= v_k(11);
                        if v_k(11) = '1' then v_k := -v_k; end if;
                        z_d <= unsigned(v_k(8 downto 0));
                        -- rainbow width: exp ROM address for kr
                        v_e := signed(resize(s_g5(12 downto 3), 12)) - 512;
                        z_e <= v_e;
                        -- P12 above the still
                        if s_gp(12 downto 3) > 196 then c_q <= s_gp(12 downto 3) - 196;
                        else c_q <= (others => '0'); end if;
                        c_g6 <= unsigned(registers_in(5)(9 downto 4));
                    when 66 =>                         -- e8 = +-(|512 - k3| * 169) >> 8 (floor)
                        if z_neg = '1' then v_e := -signed(resize(f_acc(19 downto 8), 12)) - 1;
                        else v_e := signed(resize(f_acc(19 downto 8), 12)); end if;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 90 =>                         -- P (h) = 1874 mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when -2     => c_ph <= resize(f_acc(29 downto 17), 13);
                            when -1     => c_ph <= resize(f_acc(29 downto 16), 13);
                            when 0      => c_ph <= f_acc(27 downto 15);
                            when others => c_ph <= f_acc(26 downto 14);
                        end case;
                    when 114 =>                        -- Mp = P / xs
                        case to_integer(s_xs) is
                            when 4      => c_mp <= resize(c_ph(12 downto 2), 12);
                            when 3      => c_mp <= f_acc(27 downto 16);
                            when others => c_mp <= resize(c_ph(12 downto 1), 12);
                        end case;
                    when 115 =>
                        v_x := resize(signed('0' & c_mp), 26) + 1;
                        c_mh1 <= v_x(13 downto 1) - 1;                         -- Mh - 1
                        c_mw  <= v_x(13 downto 1) - signed(resize(c_mp, 13));
                        case to_integer(s_xs) is
                            when 4      => c_m0 <= to_signed(-375, 13);
                            when 3      => c_m0 <= to_signed(-660, 13);
                            when others => c_m0 <= to_signed(-990, 13);
                        end case;
                    when 116 | 117 | 118 | 119 | 120 | 121 | 122 | 123 =>   -- reduce m0 into [mw, Mh - 1]
                        if c_m0 < c_mw then
                            c_m0 <= c_m0 + signed(resize(c_mp, 13));
                        end if;
                    when 162 => z_t <= f_acc(23 downto 0);                   -- m0^2
                    when 186 => c_a0 <= f_acc(25 downto 0);                  -- he m0^2
                    when 210 =>                                              -- he (2 m0 + 1)
                        if c_m0 < 0 then c_d0 <= -signed(f_acc(25 downto 0));
                        else c_d0 <= signed(f_acc(25 downto 0)); end if;
                    when 234 => z_t <= f_acc(23 downto 0);                   -- mw^2
                    when 236 =>                                              -- kr: exp ROM
                        r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(10 downto 8);
                    when 240 =>
                        case to_integer(z_sh) is
                            when -2     => c_kr <= resize(r_ex_q(15 downto 3), 17);
                            when -1     => c_kr <= resize(r_ex_q(15 downto 2), 17);
                            when 0      => c_kr <= resize(r_ex_q(15 downto 1), 17);
                            when others => c_kr <= resize(r_ex_q, 17);
                        end case;
                    when 241 =>
                        case to_integer(s_xs) is
                            when 4      => c_sr <= shift_left(resize(c_kr, 19), 2);
                            when 3      => c_sr <= shift_left(resize(c_kr, 19), 1) + resize(c_kr, 19);
                            when others => c_sr <= shift_left(resize(c_kr, 19), 1);
                        end case;
                    when 258 => c_aw <= f_acc(25 downto 0);                  -- he mw^2
                    when 282 => c_dw <= -signed(f_acc(25 downto 0));          -- he (2 mw + 1), 2 mw + 1 < 0
                    when 306 => c_kw <= f_acc(25 downto 8);                  -- kr * 328 >> 8
                    when 307 =>
                        case to_integer(s_xs) is
                            when 4      => c_sw <= shift_left(resize(c_kw, 20), 2);
                            when 3      => c_sw <= shift_left(resize(c_kw, 20), 1) + resize(c_kw, 20);
                            when others => c_sw <= shift_left(resize(c_kw, 20), 1);
                        end case;
                    when 354 => c_xw <= to_signed(5400, 14) - signed(resize(f_acc(19 downto 6), 14));
                    when 378 => c_gw <= to_unsigned(1182, 11) - resize(f_acc(19 downto 10), 11);
                    when 402 => c_gk <= resize(f_acc(18 downto 10), 9);
                    when 426 => c_vss <= f_acc(24 downto 8);
                    when 427 =>
                        if s_mirror = '1' then c_vsn <= -signed(resize(c_vss, 18)); else c_vsn <= signed(resize(c_vss, 18)); end if;
                    when 450 => z_t <= f_acc(23 downto 0);                   -- rise^2
                    when 451 =>                                              -- step = rise^2 * 161 >> 8 (24-bit phase)
                        v_p := shift_left(resize(z_t, 32), 7) + shift_left(resize(z_t, 32), 5) + resize(z_t, 32);
                        c_vst <= v_p(25 downto 8);
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
    -- 0), computes the NEXT line: the slot parabola Vf, the swell cosine Vs,
    -- the screen-fixed cosine Vq -> the rainbow bend tau and the curtain edge,
    -- the ground colour, the field levels and the two phase starts.  Mirror
    -- reads the field at 1079 - y.  One serial multiplier (16 iterations,
    -- result at load + 17); ~145 clocks, so it fits SD's 858-clock line.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y  : signed(11 downto 0);
        variable v_u  : signed(11 downto 0);
        variable v_i  : unsigned(9 downto 0);
        variable v_s  : signed(17 downto 0);
        variable v_r  : signed(15 downto 0);
        variable v_p  : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 520 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                if s_mirror = '1' then
                    e_vs <= c_phs + to_unsigned(15498660, 24);       -- (1079 - 109) lines
                    e_vq <= to_unsigned(303266, 24);                 -- (1079 - 10) lines
                else
                    e_vs <= c_phs - to_unsigned(1741602, 24);        -- -109 lines
                    e_vq <= to_unsigned(16617436, 24);               -- -10 lines
                end if;
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_vs <= unsigned(signed(e_vs) + resize(c_vsn, 24));
                e_vq <= unsigned(signed(e_vq) + resize(c_vsn, 24));
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(30 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    v_y := signed(resize(e_vy(18 downto 8), 12));
                    if s_mirror = '1' then v_y := 1079 - v_y; end if;
                    e_v <= v_y - 242;
                    v_i := e_vs(23 downto 14);
                    if v_i(8) = '1' then r_cos_a <= not v_i(7 downto 0); else r_cos_a <= v_i(7 downto 0); end if;
                    e_ssn <= v_i(9) xor v_i(8);
                when 2 =>                                            -- (y - 242) mod 536
                    if e_v < 0 then e_v <= e_v + 536;
                    elsif e_v >= 536 then e_v <= e_v - 536; end if;
                when 3 =>
                    e_cs <= r_cos_q;
                    v_i := e_vq(23 downto 14);
                    if v_i(8) = '1' then r_cos_a <= not v_i(7 downto 0); else r_cos_a <= v_i(7 downto 0); end if;
                    e_sqn <= v_i(9) xor v_i(8);
                when 5 =>                                            -- u^2
                    e_cq <= r_cos_q;
                    v_u := e_v - 268;
                    if v_u < 0 then v_u := -v_u; end if;
                    m_a <= resize(unsigned(v_u), 32); m_b <= resize(unsigned(v_u), 16); m_acc <= (others => '0');
                when 22 =>                                           -- u^2 * 425
                    m_a <= m_acc; m_b <= to_unsigned(425, 16); m_acc <= (others => '0');
                when 39 =>                                           -- Vf = -(u^2 * 425) >> 12; |cos| * 6226
                    e_vf <= -signed(resize(m_acc(26 downto 12), 16));
                    m_a <= resize(e_cs, 32); m_b <= to_unsigned(6226, 16); m_acc <= (others => '0');
                when 56 =>                                           -- Vs; |cq| * 300
                    if e_ssn = '1' then e_vsv <= -signed(resize(m_acc(30 downto 15), 16));
                    else e_vsv <= signed(resize(m_acc(30 downto 15), 16)); end if;
                    m_a <= resize(e_cq, 32); m_b <= to_unsigned(300, 16); m_acc <= (others => '0');
                when 57 =>                                           -- field levels (in range by construction)
                    v_s := to_signed(8110, 18) + resize(e_vsv, 18) + resize(e_vf, 18) - resize(c_lb, 18);
                    n_s1 <= v_s(15 downto 0);
                    v_s := resize(e_vf, 18) + 4301;
                    n_s2 <= v_s(15 downto 0);
                when 58 =>
                    v_s := resize(n_s1, 18) - resize(c_lhb, 18);
                    n_sh <= v_s(15 downto 0);
                    -- ground colour (Mint: 112 + Vs/64, 244 - 6 Vs/4096, 214 - 20 Vs/4096; Night: a dim navy)
                    if s_night = '1' then
                        v_r := to_signed(12, 16) + shift_right(e_vsv, 10);
                        n_br <= unsigned(v_r(7 downto 0));
                        v_r := to_signed(40, 16) + shift_right(e_vsv, 10) + shift_right(e_vsv, 11);
                        n_bg <= unsigned(v_r(7 downto 0));
                        v_r := to_signed(50, 16) + shift_right(e_vsv, 9);
                        n_bb <= unsigned(v_r(7 downto 0));
                    else
                        v_r := to_signed(112, 16) + shift_right(e_vsv, 6);
                        n_br <= unsigned(v_r(7 downto 0));
                        v_r := to_signed(244, 16) - shift_right(e_vsv, 10) - shift_right(e_vsv, 11);
                        n_bg <= unsigned(v_r(7 downto 0));
                        v_r := to_signed(214, 16) - shift_right(e_vsv, 8) - shift_right(e_vsv, 10);
                        n_bb <= unsigned(v_r(7 downto 0));
                    end if;
                when 73 =>                                           -- tau = 268 -+ tq (h); |cq| * Gw
                    if e_sqn = '1' then e_tau <= to_signed(268, 12) + signed(resize(m_acc(24 downto 15), 12));
                    else e_tau <= to_signed(268, 12) - signed(resize(m_acc(24 downto 15), 12)); end if;
                    m_a <= resize(e_cq, 32); m_b <= resize(c_gw, 16); m_acc <= (others => '0');
                when 90 =>                                           -- xb = Xw -+ gq; |Vf| * Gk
                    if e_sqn = '1' then e_xb <= c_xw + signed(resize(m_acc(26 downto 15), 14));
                    else e_xb <= c_xw - signed(resize(m_acc(26 downto 15), 14)); end if;
                    m_a <= resize(unsigned(-e_vf), 32); m_b <= resize(c_gk, 16); m_acc <= (others => '0');
                when 91 =>
                    n_xb <= e_xb;
                    e_tx <= signed(resize(s_x0, 14)) - 1620 - resize(e_tau, 14);
                when 107 =>                                          -- xbk = xb - (Gk |Vf|) >> 12; kr |tx|
                    e_xb <= e_xb - signed(resize(m_acc(21 downto 12), 14));
                    m_a <= resize(c_kr, 32); m_b <= resize(unsigned(-e_tx), 16); m_acc <= (others => '0');
                when 108 =>
                    v_p := signed(resize(s_x0, 14)) - e_xb;
                    e_txn <= v_p(13);
                    if v_p(13) = '1' then v_p := -v_p; end if;
                    e_tx <= v_p;
                when 124 =>                                          -- rainbow phase start (tx < 0); kw |x0 - xbk|
                    n_phb <= unsigned(-signed(m_acc(23 downto 0)));
                    m_a <= resize(c_kw, 32); m_b <= resize(unsigned(e_tx), 16); m_acc <= (others => '0');
                when 141 =>                                          -- curtain phase start
                    if e_txn = '1' then n_phw <= unsigned(-signed(m_acc(23 downto 0)));
                    else n_phw <= m_acc(23 downto 0); end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: c dx^2 by a second-order DDA (A += D, D += 2 he) with the column
    -- wrap, x (h); the two phase accumulators run late (they hold the stage-5
    -- pixel's phases).  The line's engine values are loaded at g_start.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_px  <= (others => '0');
                b_m   <= c_m0;
                b_a   <= c_a0;
                b_d   <= c_d0;
                b_xh  <= signed(resize(s_x0, 14));
                b_phb <= n_phb;
                b_phw <= n_phw;
                l_s1 <= n_s1; l_s2 <= n_s2; l_sh <= n_sh;
                l_xb <= n_xb; l_br <= n_br; l_bg <= n_bg; l_bb <= n_bb;
            else
                if g_avid = '1' then
                    b_px <= b_px + 1;
                    if b_m = c_mh1 then
                        b_m <= c_mw; b_a <= c_aw; b_d <= c_dw;
                    else
                        b_m <= b_m + 1;
                        b_a <= unsigned(signed(b_a) + b_d);
                        b_d <= b_d + signed(resize(c_he & '0', 26));
                    end if;
                    b_xh <= b_xh + signed(resize(s_xs, 14));
                end if;
                if s_gav_sr(3) = '1' then
                    b_phb <= b_phb + resize(c_sr, 24);
                    b_phw <= b_phw + resize(c_sw, 24);
                end if;
            end if;
            s_lb_ra <= b_px + 2;                                     -- read ahead: the luma lands at stage 2
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a   : unsigned(17 downto 0);
        variable v_f   : signed(17 downto 0);
        variable v_d   : signed(17 downto 0);
        variable v_w   : signed(17 downto 0);
        variable v_t   : unsigned(17 downto 0);
        variable v_ad  : unsigned(8 downto 0);
        variable v_inv : std_logic;
        variable v_h   : signed(9 downto 0);
        variable v_x   : signed(9 downto 0);
        variable v_r, v_g, v_b : unsigned(7 downto 0);
        variable v_y   : unsigned(12 downto 0);
        variable v_c   : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: c dx^2 (Q12, saturated), camera luma, rim zone, curtain key
            v_a := b_a(25 downto 8);
            if v_a > 32767 then p2_a <= to_unsigned(32767, 15); else p2_a <= v_a(14 downto 0); end if;
            p2_l <= unsigned(s_lb_q);
            if b_m(12) = '0' and b_m(11 downto 5) /= "0000000" then pf_rz(2) <= "10";      -- m >= 32
            elsif b_m(12) = '1' and b_m(11 downto 5) /= "1111111" then pf_rz(2) <= "00";   -- m <= -33
            else pf_rz(2) <= "01"; end if;
            if b_xh > l_xb then pf_kw(2) <= '1'; else pf_kw(2) <= '0'; end if;
            -- 3: camera gain product, 0.4 a partials
            p3_gl <= p2_l * c_g6;
            p3_a  <= p2_a;

            -- 4: the camera folded into a
            p4_ac  <= signed(resize(p3_a, 17)) - signed(resize(shift_right(p3_gl, 1), 17));
            p4_a04 <= shift_right(p3_a, 2) + shift_right(p3_a, 3);          -- 0.375 a
            p4_g4  <= p3_gl(13 downto 2);
            p4_pc  <= p3_gl(13 downto 6);
            -- 5: the fields
            p5_f1 <= resize(l_s1, 18) - resize(p4_ac, 18);
            p5_d  <= resize(l_sh, 18) - resize(p4_ac, 18);
            p5_f2 <= resize(l_s2, 18) + signed(resize(p4_g4, 18)) - signed(resize(p4_a04, 18));
            p5_pc <= p4_pc;
            for k in 3 to 10 loop
                pf_rz(k) <= pf_rz(k - 1); pf_kw(k) <= pf_kw(k - 1);
            end loop;
            -- 6: keys, outline band, rim weight, halo index, rainbow address
            v_f := p5_f1;
            if v_f > 0 then pf_k1(6) <= '1'; else pf_k1(6) <= '0'; end if;
            if p5_f2 > 0 then v_inv := not s_flip; else v_inv := s_flip; end if;
            pf_inv(6) <= v_inv;
            if v_f <= 0 and v_f > -369 then pf_ob(6) <= '1'; else pf_ob(6) <= '0'; end if;
            if v_f <= 0 or v_f >= 4096 then
                pf_w(6) <= (others => '0');
            else
                v_w := to_signed(4096, 18) - v_f;
                pf_w(6) <= unsigned(v_w(11 downto 10));
            end if;
            if s_rings = '1' then
                v_d := resize(signed(p5_d(12 downto 0)), 18);            -- ((d + 1) mod 2) - 1
            else
                v_d := p5_d;
            end if;
            if v_d < 0 then v_d := -v_d; end if;
            v_t := unsigned(v_d);
            if v_t(17 downto 12) /= 0 then r_hm_a <= s_night & "111111";
            else r_hm_a <= s_night & v_t(11 downto 6); end if;
            if pf_kw(5) = '1' then
                if v_inv = '1' then v_ad := b_phw(23 downto 15); else v_ad := b_phw(23 downto 15) + 271; end if;
            else
                v_ad := b_phb(23 downto 15);
            end if;
            r_rb_a <= v_ad + resize(p5_pc, 9);
            for k in 7 to 10 loop
                pf_k1(k) <= pf_k1(k - 1); pf_inv(k) <= pf_inv(k - 1); pf_ob(k) <= pf_ob(k - 1);
            end loop;
            for k in 7 to 9 loop pf_w(k) <= pf_w(k - 1); end loop;
            -- 7: rainbow and halo ROMs (EBR); 8: fabric copies
            p8_cr <= r_rr_q; p8_cg <= r_rg_q; p8_cb <= r_rb_q;
            p8_hm <= r_hm_q(26 downto 0);
            -- 9: rim delta, halo = clamp(ground - dip)
            case pf_rz(8) is
                when "00"   => p9_dr <= 245 - signed(resize(p8_cr, 9)); p9_dg <= 228 - signed(resize(p8_cg, 9));
                               p9_db <= 240 - signed(resize(p8_cb, 9));
                when "10"   => p9_dr <= 150 - signed(resize(p8_cr, 9)); p9_dg <= 245 - signed(resize(p8_cg, 9));
                               p9_db <= 225 - signed(resize(p8_cb, 9));
                when others => p9_dr <= 198 - signed(resize(p8_cr, 9)); p9_dg <= 236 - signed(resize(p8_cg, 9));
                               p9_db <= 232 - signed(resize(p8_cb, 9));
            end case;
            p9_cr <= p8_cr; p9_cg <= p8_cg; p9_cb <= p8_cb;
            v_h := signed(resize(l_br, 10)) - resize(signed(p8_hm(26 downto 18)), 10);
            if v_h < 0 then p9_hr <= (others => '0'); elsif v_h > 255 then p9_hr <= (others => '1'); else p9_hr <= unsigned(v_h(7 downto 0)); end if;
            v_h := signed(resize(l_bg, 10)) - resize(signed(p8_hm(17 downto 9)), 10);
            if v_h < 0 then p9_hg <= (others => '0'); elsif v_h > 255 then p9_hg <= (others => '1'); else p9_hg <= unsigned(v_h(7 downto 0)); end if;
            v_h := signed(resize(l_bb, 10)) - resize(signed(p8_hm(8 downto 0)), 10);
            if v_h < 0 then p9_hb <= (others => '0'); elsif v_h > 255 then p9_hb <= (others => '1'); else p9_hb <= unsigned(v_h(7 downto 0)); end if;
            -- 10: rim mix c + w/4 (rim - c), w = 0..3
            v_x := signed(resize(p9_cr, 10));
            if pf_w(9)(1) = '1' then v_x := v_x + resize(shift_right(p9_dr, 1), 10); end if;
            if pf_w(9)(0) = '1' then v_x := v_x + resize(shift_right(p9_dr, 2), 10); end if;
            p10_mr <= unsigned(v_x(7 downto 0));
            v_x := signed(resize(p9_cg, 10));
            if pf_w(9)(1) = '1' then v_x := v_x + resize(shift_right(p9_dg, 1), 10); end if;
            if pf_w(9)(0) = '1' then v_x := v_x + resize(shift_right(p9_dg, 2), 10); end if;
            p10_mg <= unsigned(v_x(7 downto 0));
            v_x := signed(resize(p9_cb, 10));
            if pf_w(9)(1) = '1' then v_x := v_x + resize(shift_right(p9_db, 1), 10); end if;
            if pf_w(9)(0) = '1' then v_x := v_x + resize(shift_right(p9_db, 2), 10); end if;
            p10_mb <= unsigned(v_x(7 downto 0));
            p10_cr <= p9_cr; p10_cg <= p9_cg; p10_cb <= p9_cb;
            p10_hr <= p9_hr; p10_hg <= p9_hg; p10_hb <= p9_hb;
            -- 11: select (lens: rim-shaded in a bubble, plain in the curtain; negative; halo), then the
            -- outline (darker on the trailing side) -> matrix ROM addresses
            if pf_k1(10) = '1' or pf_kw(10) = '1' then
                if pf_inv(10) = '1' then
                    if pf_kw(10) = '1' then v_r := p10_cr; v_g := p10_cg; v_b := p10_cb;
                    else v_r := p10_mr; v_g := p10_mg; v_b := p10_mb; end if;
                else
                    v_r := not p10_cr; v_g := not p10_cg; v_b := not p10_cb;
                end if;
            else
                v_r := p10_hr; v_g := p10_hg; v_b := p10_hb;
            end if;
            if pf_ob(10) = '1' then
                if pf_rz(10) = "10" then
                    v_r := shift_right(v_r, 2); v_g := shift_right(v_g, 2); v_b := shift_right(v_b, 2);
                else
                    v_r := shift_right(v_r, 1); v_g := shift_right(v_g, 1); v_b := shift_right(v_b, 1);
                end if;
            end if;
            r_mr_a <= v_r; r_mg_a <= v_g; r_mb_a <= v_b;
            -- 12: matrix ROM data (EBR); 13: fabric copy
            p13_mr <= r_mr_q; p13_mg <= r_mg_q; p13_mb <= r_mb_q;
            -- 14: partial sums  {Y 31..21, |Cr| 20..11, |Cb| 10..1}
            p14_y1 <= resize(unsigned(p13_mr(31 downto 21)), 12) + resize(unsigned(p13_mg(31 downto 21)), 12);
            p14_yb <= resize(unsigned(p13_mb(31 downto 21)), 12);
            p14_u1 <= signed(resize(unsigned(p13_mr(20 downto 11)), 12)) - signed(resize(unsigned(p13_mg(20 downto 11)), 12));
            p14_ub <= signed(resize(unsigned(p13_mb(20 downto 11)), 12));
            p14_v1 <= signed(resize(unsigned(p13_mb(10 downto 1)), 12)) - signed(resize(unsigned(p13_mr(10 downto 1)), 12));
            p14_vg <= signed(resize(unsigned(p13_mg(10 downto 1)), 12));
            -- 15: sums
            p15_y <= resize(p14_y1, 13) + resize(p14_yb, 13);
            p15_u <= resize(p14_u1, 13) - resize(p14_ub, 13);
            p15_v <= resize(p14_v1, 13) - resize(p14_vg, 13);
            -- 16: 10-bit Y (pre-scaled 16/13, capped at 930), Cr, Cb
            v_y := shift_right(p15_y, 1) + 64;
            if v_y > 930 then s_y16 <= to_unsigned(930, 10); else s_y16 <= v_y(9 downto 0); end if;
            v_c := shift_right(p15_u, 1) + 512;
            if v_c < 0 then s_u16 <= (others => '0'); elsif v_c > 1023 then s_u16 <= (others => '1');
            else s_u16 <= unsigned(v_c(9 downto 0)); end if;
            v_c := shift_right(p15_v, 1) + 512;
            if v_c < 0 then s_v16 <= (others => '0'); elsif v_c > 1023 then s_v16 <= (others => '1');
            else s_v16 <= unsigned(v_c(9 downto 0)); end if;
            -- 17: compose (neutral black outside the generated avid)
            if s_gav_sr(14) = '0' then
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            else
                s_y17 <= s_y16; s_u17 <= s_u16; s_v17 <= s_v16;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(14);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through a [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums), Y through [1 2 1]/4 on the same centre (4 px back): the
    -- HD HDMI bar fix; luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_ls : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 18: chroma 8-sums (they snap back to neutral once 8 blank samples are in, so a glitched
            -- sample can't offset the colours for good); luma taps
            s_cau(0) <= s_u17; s_cav(0) <= s_v17;
            for i in 1 to 7 loop s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bau <= s_bau + s_u17 - s_cau(7);
                s_bav <= s_bav + s_v17 - s_cav(7);
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            s_ly(0) <= s_y17;
            for i in 1 to 5 loop s_ly(i) <= s_ly(i - 1); end loop;
            -- 19: filter outputs
            v_sc := resize(s_bau, 14) + s_baud; s_u19 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v19 <= v_sc(13 downto 4);
            v_ls := resize(s_ly(3), 12) + shift_left(resize(s_ly(4), 12), 1) + resize(s_ly(5), 12);
            s_y19 <= v_ls(11 downto 2);
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

end architecture bubbles;
