-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Harlequin v0.1.1 -- Heirloom (video phase), from the clip WIN_20231218_00_10_10 at 0.5 s.
--
-- v0.1.1: Etch on a square law (default 320 = a quarter of v0.1.0's bend, full = the same flood), the camera luma
-- through an 8-pixel box and a vertical smoothing, the Osc fed at 16x the camera gain (the same flip as v0.1.0);
-- the per-pixel camera product is a 4-bit mantissa times the luma, shifted (timing).
--
-- The still: two tall orange "lemon" diamonds (olive centre band, red-orange shoulders) in a field of violet / red
-- vertical stripes.  The stripes are cut by each diamond's left side and hug its right side, compressed there and
-- relaxing into gentle waves further right.  A big soft "glass" diamond washes the stripes lavender / olive around
-- each lemon, and a thin outline (the lemon key's edge) crosses the stripes.  Below, a torn seam and a band where the
-- same patch runs squashed: flat orange diamonds half a period over, with chevron stripes.  In the clip the whole
-- picture rolls up (V-hold) at ~350 lines / s.
--
-- The patch, as modelled (harlequin.py has every table):
--   hp        horizontal oscillator, one diamond period = 942 virtual px; u = signed phase about a diamond centre
--   v         vertical position in a 1150-line rolling cycle (main row, seam, squashed band, seam)
--   core      |u| < hb(v)          orange key (B channel off), R = a horizontal V ramp, G = the lemon / glass level
--   lemon     |u| < hg(v)          G key, with a dark / bright edge line
--   glass     soft |u| < gl(v)     G added: violet -> lavender, red -> olive
--   stripes   S = F d + A E(d), d = phase distance from the core's right edge (hard sync), E = 1 - exp(-d / tau):
--             the stripe oscillator restarts at each core's trailing edge with a decaying frequency kick
--   seam      strips of the cycle slip sideways and shear the stripe phase
--   camera    luma (or the Osc lattice) widens the keys and bends the stripe phase
--
--   K1  Diamonds  diamonds across (0.7 .. 2.04 .. 5.8)
--   K2  Stripes   stripe frequency (x0.35 .. x1 .. x2.8)
--   K3  Waist     the orange cores' width (none .. the still .. x2)
--   K4  Bend      the stripes' kick after each core (x-1 .. x1 .. x4)
--   K5  Glass     the glass diamonds' size (none .. the still .. x2)
--   K6  Etch      how far the camera (or the Osc) widens the keys and bends the stripes (square law; the camera
--                 luma is an 8-px box)
--   S7  Shape     Lemon (the still's fitted V shapers) / Rhombus (triangle V shapers: straight diamonds)
--   S8  Tear      Calm (the still's seam) / Torn (32-line bands slip sideways all over)
--   S9  Core      Orange (the still) / Stripes (the hidden stripes run hot through the cores)
--   S10 Patch     RGB (the still) / GBR (the colour patch rotated)
--   S11 Source    Video / Osc
--   P12 V-HOLD    0 = locked (main row centred) .. the clip's roll (400) .. fast roll, 2.5 cycles per screen
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each line start
-- (computes the NEXT line from the vertical table: interpolated widths, stripe frequency and kick, seam slips), a
-- 25-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output
-- conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the running-sum resync, luma 13/16)
-- before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.harlequin_pkg.all;
use work.all;

architecture harlequin of program_top is
    constant C_LAT  : integer := 24;        -- = processing_delay_clks
    constant C_PVQ  : integer := 294400;    -- vertical cycle, Q8 virtual lines (1150)
    constant C_HOME : integer := 261120;    -- locked roll: the main row centred (1020 * 256)
    constant C_V40  : integer := 10240;     -- still row 0 = cycle line -40
    constant C_HC24 : integer := 7747435;   -- phase of the screen centre, Q24 (435 / 942)

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

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc lattice)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    type t_u8 is array (0 to 7) of unsigned(7 downto 0);
    signal s_bd    : t_u8 := (others => (others => '0'));    -- camera box: the last 8 samples
    signal s_bsum  : unsigned(10 downto 0) := (others => '0');
    signal s_lv    : unsigned(7 downto 0) := (others => '0');
    signal s_lva   : std_logic := '0';
    signal s_lvc   : unsigned(10 downto 0) := (others => '0');

    -- ROMs (registered address and data)
    signal r_vt_a  : unsigned(8 downto 0) := (others => '0');
    signal r_vt_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_ex_a  : unsigned(8 downto 0) := (others => '0');
    signal r_ex_q  : unsigned(7 downto 0) := (others => '0');
    signal r_e2_a  : unsigned(7 downto 0) := (others => '0');
    signal r_e2_q  : unsigned(15 downto 0) := (others => '0');
    signal r_pal_a : unsigned(9 downto 0) := (others => '0');
    signal r_pal_q : std_logic_vector(31 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 320 x 270 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(204, 9);
    signal s_oys : unsigned(10 downto 0) := to_unsigned(242, 11);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_x0   : unsigned(8 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(8 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_shape, s_torn, s_fill, s_patch, s_src : std_logic := '0';

    -- per-field constants
    signal z_e1, z_e2, z_ef : signed(11 downto 0) := (others => '0');
    signal z_m1 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_p  : unsigned(9 downto 0) := to_unsigned(400, 10);
    signal z_d  : signed(20 downto 0) := (others => '0');
    signal c_hd   : unsigned(15 downto 0) := to_unsigned(8905, 16);   -- Q24 phase per h
    signal c_hds  : unsigned(17 downto 0) := to_unsigned(17810, 18);  -- per pixel
    signal c_hp0  : unsigned(23 downto 0) := (others => '0');
    signal c_fk   : unsigned(15 downto 0) := to_unsigned(4096, 16);   -- Q12 stripe factor
    signal c_w3   : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal c_gls  : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal c_bendp : unsigned(10 downto 0) := to_unsigned(512, 11);   -- bend + 256 (Q8)
    signal c_et   : unsigned(8 downto 0) := to_unsigned(25, 9);
    signal c_em   : unsigned(3 downto 0) := to_unsigned(12, 4);   -- et ~ em << es (4-bit mantissa)
    signal c_es   : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal c_spd  : unsigned(13 downto 0) := (others => '0');
    signal c_sq   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal c_vstep : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal c_roll : unsigned(18 downto 0) := (others => '0');
    signal c_vv0  : unsigned(18 downto 0) := (others => '0');
    signal c_seed : unsigned(15 downto 0) := (others => '0');
    signal c_fcnt : unsigned(2 downto 0) := (others => '0');

    -- field serial multiplier
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : unsigned(15 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(2 downto 0) := (others => '0');
    signal z_k6  : unsigned(9 downto 0) := to_unsigned(320, 10);
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
    signal e_vv   : unsigned(18 downto 0) := (others => '0');
    signal e_n    : unsigned(8 downto 0) := (others => '0');
    signal e_k    : unsigned(6 downto 0) := (others => '0');
    signal e_fr3  : unsigned(2 downto 0) := (others => '0');
    signal e_fr4  : unsigned(3 downto 0) := (others => '0');
    signal e_vl   : unsigned(10 downto 0) := (others => '0');
    signal e_wa, e_wb, e_fa, e_fb : std_logic_vector(31 downto 0) := (others => '0');
    signal e_band : std_logic := '0';
    signal e_seam : unsigned(2 downto 0) := (others => '0');
    signal e_hb, e_hg, e_gl : unsigned(9 downto 0) := (others => '0');
    signal e_f    : unsigned(11 downto 0) := (others => '0');
    signal e_df   : unsigned(9 downto 0) := (others => '0');
    signal e_t1, e_t2, e_t3 : signed(11 downto 0) := (others => '0');
    signal e_hb2, e_df2, e_gl2 : unsigned(10 downto 0) := (others => '0');
    signal e_f2   : unsigned(14 downto 0) := (others => '0');
    signal e_a    : signed(16 downto 0) := (others => '0');
    signal e_a2   : signed(12 downto 0) := (others => '0');
    signal e_phi  : unsigned(7 downto 0) := (others => '0');
    signal e_hx   : unsigned(15 downto 0) := (others => '0');
    signal e_slb  : signed(16 downto 0) := (others => '0');
    signal e_slip : signed(17 downto 0) := (others => '0');
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(31 downto 0) := (others => '0');
    -- next line's start phase (the other results are loaded straight from e_* at g_start: the engine's next
    -- run rewrites them only after that)
    signal n_hp0  : unsigned(23 downto 0) := (others => '0');

    -- beam and the line's values
    signal b_hp  : unsigned(23 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_hb16, l_hg16, l_gl16 : unsigned(16 downto 0) := (others => '0');
    signal l_band : std_logic := '0';
    signal l_f    : unsigned(11 downto 0) := (others => '0');
    signal l_a    : signed(12 downto 0) := (others => '0');
    signal l_phi  : unsigned(7 downto 0) := (others => '0');

    -- pixel path
    signal p2_um, p2_uk : signed(15 downto 0) := (others => '0');
    signal p3_dum, p3_duk : unsigned(15 downto 0) := (others => '0');
    signal p3_uk  : signed(15 downto 0) := (others => '0');
    signal p3_ur  : signed(16 downto 0) := (others => '0');
    signal p4_duk, p5_duk : unsigned(15 downto 0) := (others => '0');
    signal p4_t, p5_t : unsigned(15 downto 0) := (others => '0');
    signal p4_gd  : signed(17 downto 0) := (others => '0');
    signal p4_r5  : unsigned(4 downto 0) := (others => '0');
    signal p5_cam : signed(14 downto 0) := (others => '0');   -- luma x mantissa (shifted by c_es at 6)
    signal p5_g2  : unsigned(1 downto 0) := (others => '0');
    signal p5_r5  : unsigned(4 downto 0) := (others => '0');
    signal p6_dk  : signed(17 downto 0) := (others => '0');
    signal p6_dn  : unsigned(15 downto 0) := (others => '0');
    signal p6_cs  : unsigned(7 downto 0) := (others => '0');
    signal p7_d   : unsigned(15 downto 0) := (others => '0');
    signal p7_e   : signed(17 downto 0) := (others => '0');
    signal p8_ph, p8_pl : unsigned(17 downto 0) := (others => '0');
    signal p9_fd, p10_q, p11_q, p12_q, p12_ae, p13_s : unsigned(7 downto 0) := (others => '0');
    signal p10_e  : unsigned(7 downto 0) := (others => '0');
    signal p11_ah, p11_al : signed(17 downto 0) := (others => '0');
    signal p14_t  : unsigned(6 downto 0) := (others => '0');
    signal s_cs7, s_cs8, s_cs9 : unsigned(7 downto 0) := (others => '0');   -- camera stripe term + the line phase
    -- carried flags: (kb, kg, ol, g2(2), r5(5)) by stage
    type t_fl is array (6 to 15) of std_logic_vector(9 downto 0);
    signal s_fl : t_fl := (others => (others => '0'));
    signal p17_q, p18_q : std_logic_vector(31 downto 0) := (others => '0');
    -- vertical smoothing of the camera luma: 1024 x 8 half-resolution state (read with the stage-1 pixel, written back
    -- with the stage-5 pixel: the addresses never meet)
    type t_lbv is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbv : t_lbv := (others => x"00");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lbv : signal is true;
    signal s_vb_ra, s_vb_wa : unsigned(9 downto 0) := (others => '0');
    signal s_vb_q, s_vb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_vb_we : std_logic := '0';
    signal p2_va, p3_va, p4_va : unsigned(10 downto 0) := (others => '0');
    signal p3_l, p3_vb, p4_s : unsigned(7 downto 0) := (others => '0');

    -- output conditioning
    -- each box's 8-sample delay is a 256 x 32 block-RAM ring: written at wp, read at wp - 5 (address reg + RAM
    -- reg + fabric reg = the sample 8 clocks back); the read never meets the write address (no_rw_check)
    type t_ring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal rga, rgb : t_ring := (others => (others => '0'));
    attribute no_rw_check of rga : signal is true;
    attribute no_rw_check of rgb : signal is true;
    signal s_rwp : unsigned(7 downto 0) := (others => '0');
    signal s_rra, s_rrb : unsigned(7 downto 0) := (others => '0');
    signal s_rqa, s_rqb : std_logic_vector(31 downto 0) := (others => '0');
    signal s_rqa2, s_rqb2 : std_logic_vector(31 downto 0) := (others => '0');
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
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
    -- Line buffer (collisions possible: read-before-write kept) and ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_lbv : process(clk)
    begin
        if rising_edge(clk) then
            s_vb_q <= lbv(to_integer(s_vb_ra));
            if s_vb_we = '1' then lbv(to_integer(s_vb_wa)) <= s_vb_wd; end if;
        end if;
    end process p_lbv;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_vt_q  <= C_HQ_VT(to_integer(r_vt_a));
            r_ex_q  <= C_HQ_EX(to_integer(r_ex_a));
            r_e2_q  <= C_HQ_EXP2(to_integer(r_e2_a));
            r_pal_q <= C_HQ_PAL(to_integer(r_pal_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_e  : signed(11 downto 0);
        variable v_r  : unsigned(19 downto 0);
        variable v_d  : signed(20 downto 0);
        variable v_t  : unsigned(17 downto 0);
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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_shape <= registers_in(6)(0);
                s_torn  <= registers_in(6)(1);
                s_fill  <= registers_in(6)(2);
                s_patch <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(408, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(306, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(204, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 20) * to_unsigned(242, 8), 8), 11);

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

            -- serial products: slot k loads at zc = 50 + 24 k, 16 iterations, result stable from
            -- load + 17.  The slot number is decoded two clocks ahead and the operands are pre-muxed.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 48  => z_sel <= to_unsigned(0, 3); z_pre <= '1';
                    when 72  => z_sel <= to_unsigned(1, 3); z_pre <= '1';
                    when 96  => z_sel <= to_unsigned(2, 3); z_pre <= '1';
                    when 120 => z_sel <= to_unsigned(3, 3); z_pre <= '1';
                    when 146 => z_sel <= to_unsigned(4, 3); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0 =>      z_fa <= to_unsigned(8905, 16); z_fb <= z_m1;
                    when 1 =>      z_fa <= resize(to_unsigned(1920, 11) - resize(s_x0, 11), 16); z_fb <= c_hd;
                    when 2 =>      z_fa <= resize(z_p, 16); z_fb <= resize(z_p, 16);
                    when 3 =>      z_fa <= resize(s_ys, 16); z_fb <= resize(c_sq, 16);
                    when others => z_fa <= resize(z_k6, 16); z_fb <= resize(z_k6, 16);
                end case;
            end if;
            z_ld <= z_pre;
            if z_ld = '1' then
                f_a <= resize(z_fa, 32); f_b <= z_fb; f_acc <= (others => '0');
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 38 =>
                        c_w3  <= s_g3(12 downto 4);
                        c_gls <= s_g5(12 downto 4);
                        if s_g4(12) = '0' then
                            c_bendp <= resize(s_g4(12 downto 3), 11);
                        else
                            c_bendp <= to_unsigned(512, 11) + resize(s_g4(11 downto 3), 11) + resize(s_g4(11 downto 4), 11);
                        end if;
                        z_k6 <= s_g6(12 downto 3);
                        z_p <= s_gp(12 downto 3);
                    when 39 =>
                        -- e = v - (v >> 2), v = k - 512: Q8 octaves (+-1.5 octaves over the travel)
                        v_e := signed(resize(s_g1(12 downto 3), 12)) - 512;
                        z_e1 <= v_e - shift_right(v_e, 2);
                        v_e := signed(resize(s_g2(12 downto 3), 12)) - 512;
                        z_e2 <= v_e - shift_right(v_e, 2);
                    when 40 =>
                        r_e2_a <= unsigned(z_e1(7 downto 0));
                        z_ef <= z_e2 - z_e1;
                    when 41 =>
                        if z_ef > 767 then z_ef <= to_signed(767, 12);
                        elsif z_ef < -768 then z_ef <= to_signed(-768, 12); end if;
                    when 42 =>
                        z_m1 <= r_e2_q;                       -- e1 mantissa (address set at 40)
                        r_e2_a <= unsigned(z_ef(7 downto 0));
                    when 45 =>
                        -- c_fk = 4096 * 2^(ef/256) = m >> (3 - sh)
                        case to_integer(z_ef(11 downto 8)) is
                            when -3     => c_fk <= shift_right(r_e2_q, 6);
                            when -2     => c_fk <= shift_right(r_e2_q, 5);
                            when -1     => c_fk <= shift_right(r_e2_q, 4);
                            when 0      => c_fk <= shift_right(r_e2_q, 3);
                            when 1      => c_fk <= shift_right(r_e2_q, 2);
                            when others => c_fk <= shift_right(r_e2_q, 1);
                        end case;
                    when 67 =>
                        -- c_hd = 8905 * m1 >> (15 - sh1)
                        case to_integer(z_e1(11 downto 8)) is
                            when -2     => c_hd <= resize(f_acc(31 downto 17), 16);
                            when -1     => c_hd <= f_acc(31 downto 16);
                            when 0      => c_hd <= f_acc(30 downto 15);
                            when others => c_hd <= f_acc(29 downto 14);
                        end case;
                    when 70 =>
                        case to_integer(s_xs) is
                            when 4      => c_hds <= shift_left(resize(c_hd, 18), 2);
                            when 3      => c_hds <= shift_left(resize(c_hd, 18), 1) + resize(c_hd, 18);
                            when others => c_hds <= shift_left(resize(c_hd, 18), 1);
                        end case;
                    when 91 =>
                        c_hp0 <= to_unsigned(C_HC24, 24) - f_acc(23 downto 0);
                    when 115 =>
                        v_r := f_acc(19 downto 0);
                        c_spd <= resize(shift_right(v_r, 7), 14) + resize(shift_right(v_r, 9), 14);
                    when 116 =>
                        if z_p(9) = '0' or z_p = 512 then
                            c_sq <= to_unsigned(256, 10);
                        else
                            v_t := resize(z_p - 512, 18);
                            c_sq <= to_unsigned(256, 10) + resize(v_t - shift_right(v_t, 2), 10);
                        end if;
                    when 139 =>
                        c_vstep <= f_acc(19 downto 8);
                    when 140 =>
                        -- roll: lock toward the home phase below 48, else advance by the speed
                        v_d := to_signed(C_HOME, 21) - signed(resize(c_roll, 21));
                        z_d <= v_d;
                    when 141 =>
                        if z_d > C_PVQ / 2 then z_d <= z_d - C_PVQ;
                        elsif z_d < -(C_PVQ / 2) then z_d <= z_d + C_PVQ; end if;
                        c_fcnt <= c_fcnt + 1;
                        if c_fcnt = 5 then c_fcnt <= (others => '0'); c_seed <= c_seed + 949; end if;
                    when 142 =>
                        if z_p < 48 then
                            v_d := signed(resize(c_roll, 21)) + shift_right(z_d, 3);
                        else
                            v_d := signed(resize(c_roll, 21)) + signed(resize(c_spd, 21));
                        end if;
                        z_d <= v_d;
                    when 143 =>
                        if z_d >= C_PVQ then z_d <= z_d - C_PVQ;
                        elsif z_d < 0 then z_d <= z_d + C_PVQ; end if;
                    when 144 =>
                        c_roll <= unsigned(z_d(18 downto 0));
                        z_d <= z_d - C_V40;
                    when 145 =>
                        v_d := z_d + C_PVQ;
                        if z_d < 0 then c_vv0 <= unsigned(v_d(18 downto 0));
                        else c_vv0 <= unsigned(z_d(18 downto 0)); end if;
                    when 165 =>
                        -- Etch, square law: et = k6^2 >> 12 (default 320 -> 25, full -> 255); Osc 16x, capped
                        if s_src = '1' then
                            if f_acc(19 downto 17) /= 0 then c_et <= to_unsigned(511, 9);
                            else c_et <= f_acc(16 downto 12) & "0000"; end if;
                        else
                            c_et <= resize(f_acc(19 downto 12), 9);
                        end if;
                    when 166 =>
                        -- the pixel path multiplies by a 4-bit mantissa and shifts (no 10 x 10 product per pixel)
                        if c_et(8) = '1' then    c_es <= to_unsigned(5, 3); c_em <= c_et(8 downto 5);
                        elsif c_et(7) = '1' then c_es <= to_unsigned(4, 3); c_em <= c_et(7 downto 4);
                        elsif c_et(6) = '1' then c_es <= to_unsigned(3, 3); c_em <= c_et(6 downto 3);
                        elsif c_et(5) = '1' then c_es <= to_unsigned(2, 3); c_em <= c_et(5 downto 2);
                        elsif c_et(4) = '1' then c_es <= to_unsigned(1, 3); c_em <= c_et(4 downto 1);
                        else                     c_es <= to_unsigned(0, 3); c_em <= c_et(3 downto 0); end if;
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
        variable v_b : unsigned(10 downto 0);
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
            -- luma (or the lattice), then an 8-sample running box written to the line buffer (the camera bends
            -- the picture smoothly, not pixel by pixel); the box empties at each line start
            if s_src = '1' then s_lv <= s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lv <= v_l - 16; else s_lv <= (others => '0'); end if;
            end if;
            s_lva <= data_in.avid; s_lvc <= s_wcol;
            if s_lva = '1' then
                v_b := s_bsum + s_lv - s_bd(7);
                s_bsum <= v_b;
                s_bd(0) <= s_lv;
                for i in 1 to 7 loop s_bd(i) <= s_bd(i - 1); end loop;
                s_lb_wd <= std_logic_vector(v_b(10 downto 3));
            else
                s_bsum <= (others => '0');
                s_bd <= (others => (others => '0'));
            end if;
            s_lb_we <= s_lva;
            s_lb_wa <= s_lvc;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (shift-add, one multiplier
    -- bit per clock; signed multiplicands are sign-extended to 40 bits)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_a, v_b : unsigned(9 downto 0);
        variable v_d  : signed(11 downto 0);
        variable v_x  : unsigned(15 downto 0);
        variable v_s  : signed(17 downto 0);
        variable v_w  : signed(32 downto 0);
        variable v_s24 : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 160 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vv <= c_vv0;
                e_lc <= (others => '0');
            elsif g_start = '1' then
                if e_vv + c_vstep >= C_PVQ then
                    e_vv <= e_vv + c_vstep - C_PVQ;
                else
                    e_vv <= e_vv + c_vstep;
                end if;
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(30 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    e_n <= e_vv(18 downto 10); e_fr3 <= e_vv(9 downto 7);
                    e_k <= e_vv(18 downto 12); e_fr4 <= e_vv(11 downto 8);
                    e_vl <= e_vv(18 downto 8);
                    r_vt_a <= e_vv(18 downto 10);
                when 2 =>
                    if e_n = 287 then r_vt_a <= (others => '0'); else r_vt_a <= e_n + 1; end if;
                when 3 =>
                    e_wa <= r_vt_q;
                    r_vt_a <= to_unsigned(288, 9) + resize(e_k, 9);
                when 4 =>
                    e_wb <= r_vt_q;
                    if e_k = 71 then r_vt_a <= to_unsigned(288, 9); else r_vt_a <= to_unsigned(289, 9) + resize(e_k, 9); end if;
                when 5 =>
                    e_fa <= r_vt_q;
                when 6 =>
                    e_fb <= r_vt_q;
                    e_band <= e_wa(27);
                    e_seam <= unsigned(e_wa(30 downto 28));
                -- interpolations: a * 8 + (b - a) * fr, then >> 3 (>> 4 for F and DF)
                when 7 =>
                    v_a := '0' & unsigned(e_wa(8 downto 0)); v_b := '0' & unsigned(e_wb(8 downto 0));
                    m_acc <= resize(v_a & "000", 32);
                    v_d := signed(resize(v_b, 12)) - signed(resize(v_a, 12));
                    m_a <= unsigned(resize(v_d, 32)); m_b <= resize(e_fr3, 16);
                when 11 =>
                    e_hb <= m_acc(12 downto 3);
                    v_a := '0' & unsigned(e_wa(17 downto 9)); v_b := '0' & unsigned(e_wb(17 downto 9));
                    m_acc <= resize(v_a & "000", 32);
                    v_d := signed(resize(v_b, 12)) - signed(resize(v_a, 12));
                    m_a <= unsigned(resize(v_d, 32)); m_b <= resize(e_fr3, 16);
                when 15 =>
                    e_hg <= m_acc(12 downto 3);
                    v_a := '0' & unsigned(e_wa(26 downto 18)); v_b := '0' & unsigned(e_wb(26 downto 18));
                    m_acc <= resize(v_a & "000", 32);
                    v_d := signed(resize(v_b, 12)) - signed(resize(v_a, 12));
                    m_a <= unsigned(resize(v_d, 32)); m_b <= resize(e_fr3, 16);
                when 19 =>
                    e_gl <= m_acc(12 downto 3);
                    m_acc <= resize(unsigned(e_fa(11 downto 0)) & "0000", 32);
                    v_s := signed(resize(unsigned(e_fb(11 downto 0)), 18)) - signed(resize(unsigned(e_fa(11 downto 0)), 18));
                    m_a <= unsigned(resize(v_s, 32)); m_b <= resize(e_fr4, 16);
                when 24 =>
                    e_f <= m_acc(15 downto 4);
                    v_a := '0' & unsigned(e_fa(20 downto 12)); v_b := '0' & unsigned(e_fb(20 downto 12));
                    m_acc <= resize(v_a & "0000", 32);
                    v_d := signed(resize(v_b, 12)) - signed(resize(v_a, 12));
                    m_a <= unsigned(resize(v_d, 32)); m_b <= resize(e_fr4, 16);
                when 29 =>
                    e_df <= m_acc(13 downto 4);
                    -- Rhombus: distances to the row centres (wrapping over the 1150-line cycle)
                    e_t1 <= signed(resize(e_vl, 12)) - 370;
                    e_t2 <= signed(resize(e_vl, 12)) - 940;
                    e_t3 <= signed(resize(e_vl, 12)) - 362;
                when 30 =>
                    e_t1 <= abs(e_t1); e_t2 <= abs(e_t2); e_t3 <= abs(e_t3);
                when 31 =>
                    if e_t1 > 575 then e_t1 <= 1150 - e_t1; end if;
                    if e_t2 > 575 then e_t2 <= 1150 - e_t2; end if;
                    if e_t3 > 575 then e_t3 <= 1150 - e_t3; end if;
                when 32 =>
                    if e_band = '1' then e_t1 <= 150 - e_t2; else e_t1 <= 420 - e_t1; end if;
                    e_t3 <= 492 - e_t3;
                when 33 =>
                    if e_t1 < 0 then e_t1 <= (others => '0'); end if;
                    if e_t3 < 0 then e_t3 <= (others => '0'); end if;
                when 34 =>
                    m_a <= resize(unsigned(e_t1(9 downto 0)), 32); m_acc <= (others => '0');
                    if e_band = '1' then m_b <= to_unsigned(325, 16); else m_b <= to_unsigned(172, 16); end if;
                when 45 =>
                    if s_shape = '1' then e_hb <= m_acc(17 downto 8); end if;
                    m_a <= resize(unsigned(e_t1(9 downto 0)), 32); m_acc <= (others => '0');
                    if e_band = '1' then m_b <= to_unsigned(352, 16); else m_b <= to_unsigned(156, 16); end if;
                when 56 =>
                    if s_shape = '1' then e_hg <= m_acc(17 downto 8); end if;
                    m_a <= resize(unsigned(e_t3(9 downto 0)), 32); m_acc <= (others => '0');
                    m_b <= to_unsigned(223, 16);
                when 66 =>
                    if s_shape = '1' then e_gl <= m_acc(17 downto 8); end if;
                -- control scaling
                when 67 =>
                    m_a <= resize(e_hb, 32); m_b <= resize(c_w3, 16); m_acc <= (others => '0');
                when 78 =>
                    e_hb2 <= m_acc(18 downto 8);
                    m_a <= resize(e_df, 32); m_b <= resize(c_w3, 16); m_acc <= (others => '0');
                when 89 =>
                    e_df2 <= m_acc(18 downto 8);
                    m_a <= resize(e_gl, 32); m_b <= resize(c_gls, 16); m_acc <= (others => '0');
                when 100 =>
                    e_gl2 <= m_acc(18 downto 8);
                    m_a <= resize(e_f, 32); m_b <= c_fk; m_acc <= (others => '0');
                when 118 =>
                    if m_acc(31 downto 27) /= 0 then e_f2 <= (others => '1');
                    else e_f2 <= m_acc(26 downto 12); end if;
                when 119 =>
                    v_s := signed(resize(e_hb2, 18)) - signed(resize(e_df2, 18));
                    m_a <= unsigned(resize(v_s, 32)); m_b <= resize(e_f2, 16); m_acc <= (others => '0');
                when 136 =>
                    e_a <= signed(m_acc(26 downto 10));
                when 137 =>
                    m_a <= unsigned(resize(e_a, 32)); m_b <= resize(c_bendp, 16); m_acc <= (others => '0');
                when 150 =>
                    v_w := signed(resize(signed(m_acc(31 downto 8)), 33)) - resize(e_a, 33);
                    if v_w > 4095 then e_a2 <= to_signed(4095, 13);
                    elsif v_w < -4096 then e_a2 <= to_signed(-4096, 13);
                    else e_a2 <= v_w(12 downto 0); end if;
                -- seam shear: (v - v0) * |alpha|
                when 151 =>
                    case to_integer(e_seam) is
                        when 1      => m_a <= resize(e_vl - 728, 32);  m_b <= to_unsigned(9, 16);
                        when 2      => m_a <= resize(e_vl - 760, 32);  m_b <= to_unsigned(5, 16);
                        when 3      => m_a <= resize(e_vl - 1040, 32); m_b <= to_unsigned(8, 16);
                        when others => m_a <= (others => '0');         m_b <= (others => '0');
                    end case;
                    m_acc <= (others => '0');
                    -- slip hash input: Torn = the 32-line band and the seed, Calm = the strip and seed / 8
                    if s_torn = '1' then
                        v_x := resize(e_vl(10 downto 5), 8) & resize(e_vl(10 downto 5), 8);
                        e_hx <= v_x + c_seed;
                    else
                        v_x := resize(e_seam, 8) & resize(e_seam, 8);
                        e_hx <= v_x + ("000" & c_seed(15 downto 3));
                    end if;
                when 152 =>
                    e_hx <= e_hx xor (e_hx(10 downto 0) & e_hx(15 downto 11));
                when 153 =>
                    e_hx <= e_hx + x"6D2B";
                when 154 =>
                    e_hx <= e_hx xor ("0000000" & e_hx(15 downto 7));
                when 157 =>
                    if e_seam = 2 then e_phi <= 0 - m_acc(7 downto 0); else e_phi <= m_acc(7 downto 0); end if;
                    m_a <= unsigned(resize(signed('0' & e_hx(7 downto 0)) - 128, 32));
                    if s_torn = '1' then m_b <= to_unsigned(70, 16); else m_b <= to_unsigned(8, 16); end if;
                    m_acc <= (others => '0');
                    case to_integer(e_seam) is
                        when 1      => e_slb <= to_signed(6261, 17);
                        when 2      => e_slb <= to_signed(-4174, 17);
                        when 3      => e_slb <= to_signed(4870, 17);
                        when 4      => e_slb <= to_signed(-2783, 17);
                        when others => e_slb <= (others => '0');
                    end case;
                when 166 =>
                    v_s := resize(e_slb, 18);
                    if s_torn = '1' then
                        if e_hx(8) = '1' then v_s := v_s + signed(m_acc(17 downto 0)); end if;
                        if e_seam /= 0 then v_s := shift_left(v_s, 1); end if;
                    elsif e_seam /= 0 then
                        v_s := v_s + signed(m_acc(17 downto 0));
                    end if;
                    e_slip <= v_s;
                when 167 =>
                    v_s24  := unsigned(resize(e_slip, 24));
                    n_hp0  <= c_hp0 + (v_s24(15 downto 0) & x"00");
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the diamond phase (Q24 per period) of the pixel in stage 1
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_px : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            v_px := b_px;
            if g_start = '1' then
                b_hp <= n_hp0;
                v_px := (others => '0');
                l_hb16 <= e_hb2 & "000000"; l_hg16 <= "0" & e_hg & "000000"; l_gl16 <= e_gl2 & "000000";
                l_band <= e_band; l_f <= e_f2(14 downto 3); l_a <= e_a2; l_phi <= e_phi;
            elsif g_avid = '1' then
                b_hp <= b_hp + c_hds;
                v_px := b_px + 1;
            end if;
            -- the line buffers are addressed with the stage-1 pixel (data at stage 2, fabric copies at stage 3)
            b_px    <= v_px;
            s_lb_ra <= v_px;
            s_vb_ra <= v_px(10 downto 1);
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (signal pK_* = the stage-K pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ls  : signed(9 downto 0);
        variable v_vd, v_vs : signed(9 downto 0);
        variable v_cam : signed(14 downto 0);
        variable v_c   : signed(19 downto 0);
        variable v_dk  : signed(17 downto 0);
        variable v_r   : signed(16 downto 0);
        variable v_g   : signed(17 downto 0);
        variable v_t   : signed(7 downto 0);
        variable v_s3  : unsigned(2 downto 0);
        variable v_lo  : std_logic_vector(6 downto 0);
        variable v_rg  : std_logic_vector(1 downto 0);
        variable v_ae  : signed(20 downto 0);
        variable v_fd  : unsigned(23 downto 0);
        variable v_ad  : std_logic_vector(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: signed phase about the main diamond centre, and about the key centre (band: half a period over)
            p2_um <= signed(b_hp(23 downto 8));
            if l_band = '1' then
                p2_uk <= signed((not b_hp(23)) & b_hp(22 downto 8));
            else
                p2_uk <= signed(b_hp(23 downto 8));
            end if;
            -- 3: distances
            if p2_um = -32768 then p3_dum <= to_unsigned(32767, 16);
            else p3_dum <= unsigned(abs(p2_um)); end if;
            if p2_uk = -32768 then p3_duk <= to_unsigned(32767, 16);
            else p3_duk <= unsigned(abs(p2_uk)); end if;
            p3_uk <= p2_uk;
            p3_ur <= resize(p2_um, 17) + 2783;
            p2_va <= b_px;
            p3_va <= p2_va;
            p3_l  <= unsigned(s_lb_q);
            p3_vb <= unsigned(s_vb_q);
            -- 4: luma (fabric copy of the line buffer), the hard-sync distance before the camera, glass, ramp
            -- 4 (cont.): vertical smoothing of the (boxed) camera luma: s = prev + (l - prev) / 4, the previous lines'
            -- state kept at half resolution and written back on even pixels
            v_vd := signed(resize(p3_l, 10)) - signed(resize(p3_vb, 10));
            v_vs := signed(resize(p3_vb, 10)) + shift_right(v_vd, 2);
            p4_s   <= unsigned(v_vs(7 downto 0));
            p4_va  <= p3_va;
            p4_duk <= p3_duk;
            p4_t   <= unsigned(p3_uk) - l_hb16(15 downto 0);
            p4_gd  <= signed(resize(l_gl16, 18)) - signed(resize(p3_dum, 18));
            v_r := abs(p3_ur);
            if v_r(15 downto 14) /= "00" then p4_r5 <= (others => '1');
            else p4_r5 <= unsigned(v_r(13 downto 9)); end if;
            -- 5: camera product (Video: luma 0..; Osc: lattice - 110, bipolar)
            -- 5: write the smoothed state back (even pixels of an active line; the read 4 pixels ahead never meets
            -- it), camera product (Video: luma 0..; Osc: lattice - 110, bipolar)
            s_vb_we <= p4_va(0) nor (not s_gav_sr(2));
            s_vb_wa <= p4_va(10 downto 1);
            s_vb_wd <= std_logic_vector(p4_s);
            if s_src = '1' then v_ls := signed(resize(p4_s, 10)) - 110;
            else v_ls := signed(resize(p4_s, 10)); end if;
            v_cam := v_ls * signed('0' & c_em);
            p5_cam <= v_cam(14 downto 0);
            p5_duk <= p4_duk;
            p5_t   <= p4_t;
            if p4_gd < 0 then p5_g2 <= "00";
            elsif p4_gd(17 downto 12) /= 0 then p5_g2 <= "11";
            else p5_g2 <= unsigned(p4_gd(11 downto 10)); end if;
            p5_r5  <= p4_r5;
            -- 6: key distance with the camera, the sync distance with the camera, the stripe term
            case to_integer(c_es) is
                when 0      => v_c := resize(p5_cam, 20);
                when 1      => v_c := shift_left(resize(p5_cam, 20), 1);
                when 2      => v_c := shift_left(resize(p5_cam, 20), 2);
                when 3      => v_c := shift_left(resize(p5_cam, 20), 3);
                when 4      => v_c := shift_left(resize(p5_cam, 20), 4);
                when others => v_c := shift_left(resize(p5_cam, 20), 5);
            end case;
            p6_dk <= signed(resize(p5_duk, 18)) - resize(shift_right(v_c, 2), 18);
            p6_dn <= p5_t - unsigned(v_c(17 downto 2));
            p6_cs <= unsigned(v_c(13 downto 6));
            s_fl(6) <= "000" & std_logic_vector(p5_g2) & std_logic_vector(p5_r5);
            -- 7: keys
            p7_d <= p6_dn;
            s_cs7 <= p6_cs + l_phi;
            if p6_dk < signed(resize(l_hb16, 18)) then s_fl(7)(9) <= '1'; else s_fl(7)(9) <= '0'; end if;
            if p6_dk < signed(resize(l_hg16, 18)) then s_fl(7)(8) <= '1'; else s_fl(7)(8) <= '0'; end if;
            p7_e <= p6_dk - signed(resize(l_hg16, 18));
            s_fl(7)(7 downto 0) <= s_fl(6)(7 downto 0);
            -- 8: E address, F * d (d12 split 6 / 6), outline
            r_ex_a <= p7_d(15 downto 7);
            p8_ph <= p7_d(15 downto 10) * l_f;
            p8_pl <= p7_d(9 downto 4) * l_f;
            s_fl(8) <= s_fl(7);
            if p7_e > -600 and p7_e < 600 and l_hg16 /= 0 then s_fl(8)(7) <= '1'; else s_fl(8)(7) <= '0'; end if;
            s_cs8 <= s_cs7;
            -- 9: (F >> 3) * d >> 9 (mod one cycle)
            v_fd := shift_left(resize(p8_ph, 24), 6) + resize(p8_pl, 24);
            p9_fd <= v_fd(16 downto 9);
            s_fl(9) <= s_fl(8); s_cs9 <= s_cs8;
            -- 10: E (fabric copy), + the line's shear phase
            p10_e <= r_ex_q;
            p10_q <= p9_fd + s_cs9;
            s_fl(10) <= s_fl(9);
            -- 11: A * E (E split 4 / 4), + the camera term
            p11_ah <= l_a * signed('0' & p10_e(7 downto 4));
            p11_al <= l_a * signed('0' & p10_e(3 downto 0));
            p11_q <= p10_q;
            s_fl(11) <= s_fl(10);
            -- 12: A * E >> 8 (mod one cycle)
            v_ae := shift_left(resize(p11_ah, 21), 4) + resize(p11_al, 21);
            p12_ae <= unsigned(v_ae(15 downto 8));
            p12_q <= p11_q;
            s_fl(12) <= s_fl(11);
            -- 13: stripe phase
            p13_s <= p12_q + p12_ae;
            s_fl(13) <= s_fl(12);
            -- 14: triangle (127 at phase 0)
            if p13_s(7) = '1' then p14_t <= p13_s(6 downto 0); else p14_t <= not p13_s(6 downto 0); end if;
            s_fl(14) <= s_fl(13);
            -- 15: soft square and the palette address {patch, region, low}
            v_t := signed('0' & p14_t) - 36;
            if v_t < 0 then v_s3 := "000";
            elsif v_t(6) = '1' or v_t(5 downto 3) = "111" then v_s3 := "111";
            else v_s3 := unsigned(v_t(5 downto 3)); end if;
            if s_fl(14)(9) = '1' then
                if s_fill = '1' then
                    v_rg := "10";
                    v_lo := std_logic_vector(v_s3) & s_fl(14)(4 downto 3) & s_fl(14)(8) & s_fl(14)(7);
                else
                    v_rg := "01";
                    v_lo := s_fl(14)(4 downto 0) & s_fl(14)(8) & s_fl(14)(7);
                end if;
            else
                v_rg := "00";
                v_lo := std_logic_vector(v_s3) & s_fl(14)(6 downto 5) & s_fl(14)(8) & s_fl(14)(7);
            end if;
            v_ad := s_patch & v_rg & v_lo;
            r_pal_a <= unsigned(v_ad);
            -- 16: palette ROM; 17, 18 fabric copies
            p17_q <= r_pal_q;
            p18_q <= p17_q;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the still's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16 and the blanking gate.  The box inputs are
    -- neutral black off the generated avid, and each running sum reloads
    -- 8 x black once 8 blank samples are in (the resync).
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rga(to_integer(s_rwp)) <= "00" & std_logic_vector(s_y19) & std_logic_vector(s_u19) & std_logic_vector(s_v19);
            s_rqa <= rga(to_integer(s_rra));
        end if;
    end process p_ring;

    p_ringb : process(clk)
    begin
        if rising_edge(clk) then
            rgb(to_integer(s_rwp)) <= "00" & std_logic_vector(s_y21) & std_logic_vector(s_u21) & std_logic_vector(s_v21);
            s_rqb <= rgb(to_integer(s_rrb));
        end if;
    end process p_ringb;

    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 19: box A input
            if s_gav_sr(16) = '1' then
                s_y19 <= unsigned(p18_q(29 downto 20)); s_u19 <= unsigned(p18_q(19 downto 10)); s_v19 <= unsigned(p18_q(9 downto 0));
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
            -- box A: stage 20 sums, stage 21 output
            s_rwp <= s_rwp + 1;
            s_rra <= s_rwp - 5; s_rrb <= s_rwp - 5;
            s_rqa2 <= s_rqa; s_rqb2 <= s_rqb;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(16);      -- the avid of the sample entering the box this clock
            if s_bah = "00000000" then
                s_bay <= to_unsigned(512, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y19 - unsigned(s_rqa2(29 downto 20));
                s_bau <= s_bau + s_u19 - unsigned(s_rqa2(19 downto 10));
                s_bav <= s_bav + s_v19 - unsigned(s_rqa2(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(18) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y21 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u21 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v21 <= v_sc(13 downto 4);
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
            -- box B: stage 22 sums, stage 23 output
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(18);
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(512, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y21 - unsigned(s_rqb2(29 downto 20));
                s_bbu <= s_bbu + s_u21 - unsigned(s_rqb2(19 downto 10));
                s_bbv <= s_bbv + s_v21 - unsigned(s_rqb2(9 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y23 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v23 <= v_sc(13 downto 4);
            -- 24: luma 13/16
            if s_y23 > 64 then v_dl := s_y23 - 64; else v_dl := (others => '0'); end if;
            s_y24 <= s_y23 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u24 <= s_u23; s_v24 <= s_v23;
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

end architecture harlequin;
