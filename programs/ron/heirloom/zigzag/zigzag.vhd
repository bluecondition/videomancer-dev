-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Zigzag v0.1.0 -- Heirloom (phase 3), built from the clip still WIN_20230205_00_16_05 at 0.5 s.
--
-- The still: a bright green ground, two rounded-top red zigzag bands (period 940.5 px, apexes at x 520 and 1460)
-- and pale blue / white ribbons down every branch, meeting in V's at x 990; a smoky texture (camera / feedback
-- video) keyed into the ribbons as yellow blobs and pink / cyan smears.
--
-- The patch, as fitted (zigzag.py has every constant; .heirloom_work/rb_zigzag/notes.md the measurements):
--   f      = u(t) + pitch * y / H + offset        one horizontal oscillator (a rectified rounded triangle,
--                                                 u(t) = (1 - t) + 0.3 t (1 - t), from a 1024-entry table)
--                                                 plus a vertical ramp; 1 unit = 4096, the shapers repeat every 2.0
--   R, G   = the still's red and green shapers of f (1024-entry tables)
--   B      = a window comparator (half-width 0.175 about 0.69, soft edge 1/32) on f delayed 36 virtual px
--   smoke  = texture * key(f): the camera's luma band-pass (16-px box minus the box 40 px earlier) or three
--            drifting sines; added to the shapers' input inside the ribbon only, B with 8x the R / G gain
--
--   K1  Zigzags   0.5 .. 8 periods across (2.04 = the still)
--   K2  Ribbon    the blue window's width, x4 .. x1/4 of the still's
--   K3  Lines     the red shaper's rate, x0.4 .. x2.5 (more red lines; the table is re-indexed every field)
--   K4  Speed     bipolar: the colours stream down the zigzag (or up), +-8 deadband
--   K5  Smoke     how hard the texture bites into the ribbons (0 = clean ribbons)
--   K6  Palette   which shaper drives which gun: Still, 5 permutations, Negative, Negative GBR
--   S7  Fold      Zigzag (rectified) / Slant (a sawtooth: diagonal bands)
--   S8  Mirror    the ramp folded at the centre line: diamonds
--   S9  Ground    Green / Black (the green shaper only inside the ribbon)
--   S10 Tone      Neon / Pastel (chroma halved, luma lifted)
--   S11 Source    Video / Osc
--   P12 PITCH     the vertical ramp: vertical stripes (0 %) .. the still (41 %) .. two nested zigzags (100 %)
--
-- Engine: vblank-zone sequencer (one serial multiplier, the shared glide, a per-field refill of the red table
-- re-indexed by Lines), a per-line multiply for the vertical term, two phase accumulators (R/G and the delayed B
-- field) into two u tables, the texture term of the B field through a block-RAM ring, a 24-stage pixel path with a
-- fixed RGB -> YCbCr matrix.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output
-- conditioning (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4, luma 13/16) before the blanking gate; syncs through
-- a block-RAM ring.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.zigzag_pkg.all;
use work.all;

architecture zigzag of program_top is
    constant C_LAT : integer := 32;   -- = processing_delay_clks (the syncs; the picture path is 24 stages + the filters)
    constant C_FILL : integer := 700;  -- zone clock at which the red table refill starts (1024 entries)

    constant C_RIB : integer := 2826;   -- 0.69: ribbon / window centre, Lines pivot (Q12)
    constant C_KLO : integer := 2007;   -- key window 0.49 .. 0.84
    constant C_KHI : integer := 3441;
    constant C_CY  : integer := 138240; -- centre line, Q8 virtual lines (540 * 256)
    constant C_TRX : integer := 1979;   -- 2 * 989.5: the first trough sits at virtual x 989.5

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
    -- x step in half virtual px per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- triangle of a phase's top 8 bits: +127 at 0, -127 at 1/2 (8-bit signed)
    function f_tri8(v : unsigned(7 downto 0)) return signed is
        variable r : signed(8 downto 0);
    begin
        if v(7) = '0' then
            r := to_signed(127, 9) - signed(resize(v(6 downto 0) & '0', 9));
        else
            r := signed(resize(v(6 downto 0) & '0', 9)) - to_signed(127, 9);
        end if;
        return r(7 downto 0);
    end function;
    -- texture lattice: Q16 cycles per virtual px (x) and per virtual line (y), three sines at 48 / 66 / 84 px
    function f_tdx(i : integer; xs : unsigned(2 downto 0)) return signed is
        variable b : signed(15 downto 0);
    begin
        case i is when 0 => b := to_signed(965, 16); when 1 => b := to_signed(894, 16); when others => b := to_signed(-234, 16); end case;
        case to_integer(xs) is
            when 4      => return shift_left(b, 1);
            when 3      => return b + shift_right(b, 1);
            when others => return b;
        end case;
    end function;
    function f_tdy(i : integer; t : std_logic_vector(3 downto 0)) return signed is
        variable b, r : signed(15 downto 0);
    begin
        case i is when 0 => b := to_signed(965, 16); when 1 => b := to_signed(-433, 16); when others => b := to_signed(744, 16); end case;
        case to_integer(unsigned(t)) is
            when 0          => r := shift_left(b, 2) + shift_right(b, 1) - shift_right(b, 4);          -- 1138/256 = 4.445
            when 4          => r := shift_left(b, 1) + shift_right(b, 2);                                -- 576/256 = 2.25
            when 8          => r := shift_left(b, 1) + b + shift_right(b, 1) + shift_right(b, 2);        -- 960/256 = 3.75
            when 12         => r := b + shift_right(b, 1) + shift_right(b, 2) + shift_right(b, 3);       -- 480/256 = 1.875
            when 5 | 6 | 14 => r := b + shift_right(b, 1);                                               -- 384/256 = 1.5
            when 1 | 2 | 10 => r := shift_left(b, 1);                                                    -- 512/256 = 2
            when others     => r := b;                                                                   -- 256/256 = 1
        end case;
        return r;
    end function;
    function f_clamp8(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else return unsigned(v(7 downto 0)); end if;
    end function;
    function f_key(d1, d2 : signed(14 downto 0)) return unsigned is
        variable m : signed(14 downto 0);
    begin
        if d1 < d2 then m := d1; else m := d2; end if;
        if m < 0 then return to_unsigned(0, 7);
        elsif m > 127 then return to_unsigned(127, 7);
        else return unsigned(m(6 downto 0)); end if;
    end function;

    -- ROMs (one read port each, registered address and data)
    signal r_ua_a, r_ub_a : unsigned(9 downto 0) := (others => '0');
    signal r_ua_q, r_ub_q : unsigned(11 downto 0) := (others => '0');
    signal r_r_a, r_g_a   : unsigned(9 downto 0) := (others => '0');
    signal r_r_q, r_g_q   : unsigned(7 downto 0) := (others => '0');
    -- red table: the R shaper re-indexed by Lines, refilled from the ROM every field in the zone
    type t_rt is array (0 to 1023) of unsigned(7 downto 0);
    signal r_rt  : t_rt := (others => (others => '0'));
    signal t_run, t_we, t_we2 : std_logic := '0';
    signal t_i   : unsigned(9 downto 0) := (others => '0');
    signal t_wa, t_wa2 : unsigned(9 downto 0) := (others => '0');
    signal t_wd  : unsigned(7 downto 0) := (others => '0');
    signal t_acc : unsigned(23 downto 0) := (others => '0');
    signal t_ra  : unsigned(9 downto 0) := (others => '0');
    signal t_rq  : unsigned(7 downto 0) := (others => '0');
    signal r_s1_a, r_s2_a, r_s3_a : unsigned(7 downto 0) := (others => '0');
    signal r_s1_q, r_s2_q, r_s3_q : signed(7 downto 0) := (others => '0');
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');

    -- block-RAM rings (written every clock, read N back: no collision)
    type t_ring16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal r_tr  : t_ring16 := (others => (others => '0'));   -- texture term for the delayed B field
    signal r_v1  : t_ring16 := (others => (others => '0'));   -- source luma (16-px box tap)
    signal r_v2  : t_ring16 := (others => (others => '0'));   -- 16-px box (40-px tap)
    signal r_oc  : t_ring16 := (others => x"8080");          -- output chroma box: U (8) + V (8)
    type t_ring4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal r_sy  : t_ring4 := (others => (others => '1'));    -- syncs
    attribute no_rw_check : boolean;
    attribute no_rw_check of r_tr : signal is true;
    attribute no_rw_check of r_v1 : signal is true;
    attribute no_rw_check of r_v2 : signal is true;
    attribute no_rw_check of r_oc : signal is true;
    attribute no_rw_check of r_sy : signal is true;
    attribute no_rw_check of r_rt : signal is true;
    signal r_wp  : unsigned(7 downto 0) := (others => '0');
    signal r_tq, r_v1q, r_v2q, r_ocq : std_logic_vector(15 downto 0) := (others => '0');
    signal r_tq2 : signed(15 downto 0) := (others => '0');
    signal r_syq, r_syq2 : std_logic_vector(3 downto 0) := (others => '1');
    signal c_dr  : unsigned(7 downto 0) := to_unsigned(36, 8);   -- ring tap for the B delay (36 virtual px)

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
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(520 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(285 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(424 * 8, 13);
    signal s_k6 : unsigned(2 downto 0) := (others => '0');
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(612, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_slant, s_mir, s_blk, s_pas, s_src : std_logic := '0';

    -- per-field constants
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_e    : signed(10 downto 0) := (others => '0');
    signal z_a    : unsigned(19 downto 0) := (others => '0');
    signal z_neg  : std_logic := '0';
    signal c_stv  : unsigned(16 downto 0) := to_unsigned(17858, 17);  -- phase step per virtual px (Q24)
    signal c_stx  : unsigned(17 downto 0) := to_unsigned(17858, 18);  -- per pixel
    signal c_ph0, c_ph0b : unsigned(23 downto 0) := (others => '0');
    signal c_pit  : unsigned(14 downto 0) := (others => '0');
    signal c_pq   : unsigned(15 downto 0) := (others => '0');
    signal c_rate : unsigned(7 downto 0) := (others => '0');
    signal c_off  : unsigned(12 downto 0) := (others => '0');
    signal c_sr   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal c_hwb  : unsigned(11 downto 0) := to_unsigned(717, 12);
    signal c_gain : unsigned(8 downto 0) := (others => '0');
    signal c_ta0  : unsigned(23 downto 0) := (others => '0');         -- red table index start (Q11 entries)
    signal c_tst  : unsigned(12 downto 0) := to_unsigned(2048, 13);   -- red table index step = 8 sR
    type t_s16 is array (0 to 2) of signed(15 downto 0);
    signal t_fld, t_lin, t_pix : t_s16 := (others => (others => '0'));
    constant C_DRF : t_s16 := (to_signed(852, 16), to_signed(-721, 16), to_signed(459, 16));

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
    signal e_dy   : unsigned(18 downto 0) := (others => '0');
    signal m_a    : unsigned(39 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(39 downto 0) := (others => '0');
    signal n_v    : unsigned(12 downto 0) := (others => '0');
    signal l_v    : unsigned(12 downto 0) := (others => '0');

    -- video texture
    signal v_l    : unsigned(9 downto 0) := (others => '0');
    signal v_s16  : unsigned(13 downto 0) := (others => '0');
    signal v_box  : unsigned(9 downto 0) := (others => '0');
    signal v_boxd : unsigned(9 downto 0) := (others => '0');
    signal v_d    : signed(11 downto 0) := (others => '0');
    signal v_t    : signed(8 downto 0) := (others => '0');
    signal v_bl   : unsigned(4 downto 0) := (others => '0');
    signal v_cnt  : unsigned(5 downto 0) := (others => '0');
    signal v_ok1, v_ok2 : std_logic := '0';

    -- pixel path
    signal b_phi, b_phib : unsigned(23 downto 0) := (others => '0');
    signal p3_u, p3_ub : unsigned(11 downto 0) := (others => '0');
    signal p3_s1, p3_s2, p3_s3 : signed(7 downto 0) := (others => '0');
    signal p4_f0, p4_fb0, p5_f0, p5_fb0, p6_f0, p6_fb0, p7_f0, p7_fb0, p8_fb0, p9_fb0 : unsigned(12 downto 0) := (others => '0');
    signal p4_t   : signed(9 downto 0) := (others => '0');
    signal p5_d1, p5_d2 : signed(14 downto 0) := (others => '0');
    signal p5_tg  : signed(19 downto 0) := (others => '0');
    signal p6_key : unsigned(6 downto 0) := (others => '0');
    signal p6_tg  : signed(11 downto 0) := (others => '0');
    signal p7_kt  : signed(19 downto 0) := (others => '0');
    signal p8_f1, p10_fbs : unsigned(12 downto 0) := (others => '0');
    signal p11_db, p12_db : unsigned(12 downto 0) := (others => '0');
    signal p11_r, p12_r, p13_r, p14_r, p11_g, p12_g, p13_g, p14_g, p13_b, p14_b : unsigned(7 downto 0) := (others => '0');
    signal p15_yr, p15_yg, p15_yb : unsigned(15 downto 0) := (others => '0');
    signal p15_rr, p15_rg, p15_rb, p15_br, p15_bg, p15_bb : unsigned(16 downto 0) := (others => '0');
    signal p16_y  : unsigned(16 downto 0) := (others => '0');
    signal p16_cr, p16_cb : signed(17 downto 0) := (others => '0');
    signal p17_yn : unsigned(9 downto 0) := (others => '0');
    signal p17_u, p17_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal t_wa3  : unsigned(9 downto 0) := (others => '0');
    signal t_we3  : std_logic := '0';

    -- output conditioning
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y18d, s_y18dd : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y19, s_y20, s_y21, s_y22, s_y22b, s_y22c, s_y22d, s_y23 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u20, s_v20, s_u21, s_v21, s_u22, s_v22, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs and rings
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ua_q <= C_ZZ_U(to_integer(r_ua_a));
            r_ub_q <= C_ZZ_U(to_integer(r_ub_a));
            r_r_q  <= C_ZZ_R(to_integer(r_r_a));
            r_g_q  <= C_ZZ_G(to_integer(r_g_a));
            t_rq   <= r_rt(to_integer(t_ra));
            if t_we3 = '1' then r_rt(to_integer(t_wa3)) <= t_wd; end if;
            r_s1_q <= C_ZZ_SIN(to_integer(r_s1_a));
            r_s2_q <= C_ZZ_SIN(to_integer(r_s2_a));
            r_s3_q <= C_ZZ_SIN(to_integer(r_s3_a));
            r_ex_q <= C_ZZ_EXP2(to_integer(r_ex_a));
        end if;
    end process p_rom;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            r_wp <= r_wp + 1;
            r_tr(to_integer(r_wp)) <= std_logic_vector(p7_kt(19 downto 4));
            r_tq <= r_tr(to_integer(r_wp - c_dr));
            r_tq2 <= signed(r_tq);
            r_v1(to_integer(r_wp)) <= "000000" & std_logic_vector(v_l);
            r_v1q <= r_v1(to_integer(r_wp - 15));
            r_v2(to_integer(r_wp)) <= "000000" & std_logic_vector(v_box);
            r_v2q <= r_v2(to_integer(r_wp - 39));
            r_oc(to_integer(r_wp)) <= std_logic_vector(s_u18(9 downto 2)) & std_logic_vector(s_v18(9 downto 2));
            r_ocq <= r_oc(to_integer(r_wp - 7));
            r_sy(to_integer(r_wp)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            r_syq <= r_sy(to_integer(r_wp - 30));
            r_syq2 <= r_syq;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(10 downto 0);
        variable v_e8 : signed(18 downto 0);
        variable v_p  : signed(17 downto 0);
        variable v_st : unsigned(35 downto 0);
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
                s_slant <= registers_in(6)(0);
                s_mir   <= registers_in(6)(1);
                s_blk   <= registers_in(6)(2);
                s_pas   <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_k6    <= unsigned(registers_in(5)(9 downto 7));
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(240, 9); c_dr <= to_unsigned(18, 8);
                    when 3      => s_x0 <= (others => '0');     c_dr <= to_unsigned(24, 8);
                    when others => s_x0 <= (others => '0');     c_dr <= to_unsigned(36, 8);
                end case;
                -- texture: field drift (the line phases restart from it in p_line)
                for i in 0 to 2 loop
                    t_fld(i) <= t_fld(i) + C_DRF(i);
                end loop;
            end if;
            g_wm1 <= s_w - 1;

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
                    when 150 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 5);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(4369, 36);           z_fb <= z_mt;                     -- step per virtual px
                    when 1  => z_fa <= resize(c_stv, 36);               z_fb <= resize(s_xs, 16);         -- per pixel
                    when 2  => z_fa <= resize(c_stv, 36);               z_fb <= resize(z_a(11 downto 0), 16);   -- trough anchor
                    when 3  => z_fa <= resize(s_gp(12 downto 3), 36);   z_fb <= resize(s_gp(12 downto 3), 16);  -- p12^2
                    when 4  => z_fa <= resize(z_a, 36);                 z_fb <= to_unsigned(1026, 16);    -- pitch
                    when 5  => z_fa <= resize(c_pit, 36);               z_fb <= to_unsigned(3885, 16);    -- pq
                    when 6  => z_fa <= resize(s_vm, 36);                z_fb <= resize(s_vm, 16);         -- speed^2
                    when 7  => z_fa <= resize(z_a(17 downto 0), 36);    z_fb <= to_unsigned(2715, 16);    -- rate
                    when 8  => z_fa <= to_unsigned(717, 36);            z_fb <= z_mt;                     -- ribbon half-width
                    when 9  => z_fa <= resize(s_g5(12 downto 3), 36);   z_fb <= resize(s_g5(12 downto 3), 16);  -- k5^2
                    when 10 => z_fa <= resize(z_a, 36);                 z_fb <= to_unsigned(2051, 16);    -- gain
                    when others => z_fa <= to_unsigned(C_RIB, 36);      z_fb <= resize(z_a(9 downto 0), 16);   -- table start: c |256 - sR|
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
                    when 18 =>                        -- zigzags: e = k1 / 256 octaves
                        r_ex_a <= s_g1(10 downto 3); z_sh <= signed(resize(s_g1(12 downto 11), 3));
                    when 20 => z_mt <= r_ex_q;
                    when 23 =>
                        if s_xs = 4 then z_a <= to_unsigned(C_TRX - 480, 20); else z_a <= to_unsigned(C_TRX, 20); end if;
                    when 50 =>                        -- step per virtual px = 4369 * mt >> (15 - sh), sh 0..3
                        case to_integer(z_sh) is
                            when 0      => c_stv <= f_acc(31 downto 15);
                            when 1      => c_stv <= f_acc(30 downto 14);
                            when 2      => c_stv <= f_acc(29 downto 13);
                            when others => c_stv <= f_acc(28 downto 12);
                        end case;
                    when 74  => c_stx <= f_acc(18 downto 1);
                    when 98  =>                       -- phi0 = 2^23 - anchor * stv / 2; the B field 36 virtual px back
                        c_ph0  <= to_unsigned(2**23, 24) - f_acc(24 downto 1);
                        c_ph0b <= to_unsigned(2**23, 24) - f_acc(24 downto 1) - resize(shift_left(resize(c_stv, 24), 5) + shift_left(resize(c_stv, 24), 2), 24);
                    when 122 => z_a <= f_acc(19 downto 0);                        -- p12^2
                    when 146 => c_pit <= f_acc(30 downto 16);                     -- pitch Q12 (max 16384)
                    when 170 => c_pq  <= f_acc(25 downto 10);
                        -- speed: |k4 - 512| - 8, squared next
                    when 194 => z_a <= resize(f_acc(17 downto 0), 20);
                    when 218 =>
                        c_rate <= f_acc(29 downto 22);
                        if s_vneg = '1' then c_off <= c_off - resize(c_rate, 13); else c_off <= c_off + resize(c_rate, 13); end if;
                        -- ribbon: e = -(k2 - 512) / 256 octaves
                        v_e := -(signed(resize(s_g2(12 downto 3), 11)) - 512);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 220 => z_mt <= r_ex_q;
                    when 221 =>                       -- lines: e = 1.32 (k3 - 512) / 512 = (k3 - 512) * 169 / 256
                        v_e := signed(resize(s_g3(12 downto 3), 11)) - 512;
                        v_e8 := shift_left(resize(v_e, 19), 7) + shift_left(resize(v_e, 19), 5) + shift_left(resize(v_e, 19), 3) + resize(v_e, 19);
                        z_e <= v_e8(18 downto 8);
                    when 222 => r_ex_a <= unsigned(z_e(7 downto 0));
                    when 242 =>                       -- hwB = 717 * mt >> (15 - sh), sh -2 .. 2
                        case to_integer(z_sh) is
                            when -2     => c_hwb <= f_acc(28 downto 17);
                            when -1     => c_hwb <= f_acc(27 downto 16);
                            when 0      => c_hwb <= f_acc(26 downto 15);
                            when 1      => c_hwb <= f_acc(25 downto 14);
                            when others => c_hwb <= f_acc(24 downto 13);
                        end case;
                        -- sR = 2^e in Q8: mt >> (7 - sh), sh -2 .. 1
                        case to_integer(z_e(10 downto 8)) is
                            when -2     => c_sr <= resize(r_ex_q(15 downto 9), 10);
                            when -1     => c_sr <= resize(r_ex_q(15 downto 8), 10);
                            when 0      => c_sr <= resize(r_ex_q(15 downto 7), 10);
                            when others => c_sr <= r_ex_q(15 downto 6);
                        end case;
                    when 266 => z_a <= f_acc(19 downto 0);                        -- k5^2
                    when 290 => c_gain <= f_acc(30 downto 22);
                        -- red table: entry i reads R at (c (256 - sR) + 8 i sR) >> 11 (Q8 sR); |256 - sR| for the product
                        if c_sr > 256 then z_a <= resize(c_sr - 256, 20); z_neg <= '1';
                        else z_a <= resize(to_unsigned(256, 10) - c_sr, 20); z_neg <= '0'; end if;
                        c_tst <= c_sr & "000";
                    when 314 =>
                        if z_neg = '1' then c_ta0 <= (not resize(f_acc(23 downto 0), 24)) + 1;
                        else c_ta0 <= resize(f_acc(23 downto 0), 24); end if;
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
    -- Video texture: 16-px box of the source luma minus the same box 40 px
    -- earlier (a horizontal band-pass), x1.25, clamped; zero off avid, zero for
    -- the first 56 px of a line, running sum resynced after 16 blank inputs
    ---------------------------------------------------------------------------
    p_vid : process(clk)
        variable v_sum : unsigned(13 downto 0);
        variable v_dd  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                v_l <= unsigned(data_in.y);
                v_bl <= (others => '0');
                if v_cnt /= 63 then v_cnt <= v_cnt + 1; end if;
            else
                v_l <= (others => '0');
                if v_bl /= 31 then v_bl <= v_bl + 1; end if;
                v_cnt <= (others => '0');
            end if;
            if v_bl >= 16 then
                v_s16 <= (others => '0');
            else
                v_s16 <= v_s16 + v_l - unsigned(r_v1q(9 downto 0));
            end if;
            v_box <= v_s16(13 downto 4);
            v_boxd <= unsigned(r_v2q(9 downto 0));
            if v_cnt >= 56 then v_ok1 <= '1'; else v_ok1 <= '0'; end if;
            v_ok2 <= v_ok1;
            v_d <= signed(resize(v_box, 12)) - signed(resize(v_boxd, 12));
            v_dd := v_d + shift_right(v_d, 2);
            if v_ok2 = '0' then v_t <= (others => '0');
            elsif v_dd > 255 then v_t <= to_signed(255, 9);
            elsif v_dd < -255 then v_t <= to_signed(-255, 9);
            else v_t <= v_dd(8 downto 0); end if;
        end if;
    end process p_vid;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line's vertical term v = |vy - cy| (Mirror) or vy, times
    -- pq >> 20, on one serial multiplier (16 iterations, result at load + 17);
    -- steps the texture line phases
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_dd : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if s_zgo = '1' then
                for i in 0 to 2 loop t_lin(i) <= t_fld(i) + C_DRF(i); end loop;
            elsif g_start = '1' then
                for i in 0 to 2 loop t_lin(i) <= t_lin(i) + f_tdy(i, s_tid); end loop;
            end if;
            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(38 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    v_dd := signed(resize(e_vy, 20)) - C_CY;
                    if v_dd < 0 then v_dd := -v_dd; end if;
                    if s_mir = '1' then e_dy <= unsigned(v_dd(18 downto 0)); else e_dy <= e_vy; end if;
                when 2 =>
                    m_a <= resize(e_dy, 40); m_b <= c_pq; m_acc <= (others => '0');
                when 19 =>
                    n_v <= m_acc(32 downto 20);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Red table refill (vblank zone, zc C_FILL .. C_FILL + 1026): entry i = R ROM
    -- at (c (256 - sR) + 8 i sR) >> 11, so Lines re-scales the red shaper about
    -- the ribbon centre with no pixel multiplier
    ---------------------------------------------------------------------------
    p_fill : process(clk)
    begin
        if rising_edge(clk) then
            t_we <= '0';
            if s_zone = '1' and s_zc = C_FILL then
                t_run <= '1'; t_i <= (others => '0'); t_acc <= c_ta0;
            elsif t_run = '1' then
                t_we <= '1'; t_wa <= t_i;
                r_r_a <= t_acc(20 downto 11);
                t_acc <= t_acc + resize(c_tst, 24); t_i <= t_i + 1;
                if t_i = 1023 then t_run <= '0'; end if;
            end if;
            t_we2 <= t_we; t_wa2 <= t_wa;
            t_we3 <= t_we2; t_wa3 <= t_wa2; t_wd <= r_r_q;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 0 = the phase accumulators at g_start)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ts : signed(11 downto 0);     -- 4 s1 + 3 s2 + 2 s3 reaches +-1143: 12 bits
        variable v_f  : unsigned(13 downto 0);
        variable v_fb : unsigned(13 downto 0);
        variable v_bw : signed(14 downto 0);
        variable v_g  : unsigned(7 downto 0);
        variable v_yn : unsigned(10 downto 0);
        variable v_c  : signed(9 downto 0);
        variable v_y8 : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- 0: phases
            if g_start = '1' then
                b_phi  <= c_ph0;
                b_phib <= c_ph0b;
                l_v <= n_v;
                for i in 0 to 2 loop t_pix(i) <= t_lin(i); end loop;
            elsif g_avid = '1' then
                b_phi  <= b_phi + resize(c_stx, 24);
                b_phib <= b_phib + resize(c_stx, 24);
                for i in 0 to 2 loop t_pix(i) <= t_pix(i) + f_tdx(i, s_xs); end loop;
            end if;
            -- 1: fold -> table address (Zigzag); Slant reads the table unfolded (a sawtooth every half period)
            if b_phi(23) = '1' and s_slant = '0' then r_ua_a <= not b_phi(22 downto 13); else r_ua_a <= b_phi(22 downto 13); end if;
            if b_phib(23) = '1' and s_slant = '0' then r_ub_a <= not b_phib(22 downto 13); else r_ub_a <= b_phib(22 downto 13); end if;
            r_s1_a <= unsigned(t_pix(0)(15 downto 8));
            r_s2_a <= unsigned(t_pix(1)(15 downto 8));
            r_s3_a <= unsigned(t_pix(2)(15 downto 8));
            -- 2: ROMs; 3: fabric copies
            p3_u <= r_ua_q; p3_ub <= r_ub_q;
            p3_s1 <= r_s1_q; p3_s2 <= r_s2_q; p3_s3 <= r_s3_q;
            -- 4: the clean fields (mod 2.0, or mod 1.0 for Slant); the texture (Osc: 4 s1 + 3 s2 + 2 s3 >> 2)
            v_f  := resize(p3_u, 14) + resize(l_v, 14) + resize(c_off, 14);
            v_fb := resize(p3_ub, 14) + resize(l_v, 14) + resize(c_off, 14);
            if s_slant = '1' then
                p4_f0 <= '0' & v_f(11 downto 0); p4_fb0 <= '0' & v_fb(11 downto 0);
            else
                p4_f0 <= v_f(12 downto 0); p4_fb0 <= v_fb(12 downto 0);
            end if;
            v_ts := shift_left(resize(p3_s1, 12), 2) + resize(p3_s2, 12) + shift_left(resize(p3_s2, 12), 1) + shift_left(resize(p3_s3, 12), 1);
            v_ts := shift_right(v_ts, 2);
            if s_src = '1' then
                if v_ts > 255 then p4_t <= to_signed(255, 10);
                elsif v_ts < -255 then p4_t <= to_signed(-255, 10);
                else p4_t <= v_ts(9 downto 0); end if;
            else
                p4_t <= resize(v_t, 10);
            end if;
            -- 5: key distances, texture x gain
            p5_d1  <= signed(resize(p4_f0, 15)) - C_KLO;  p5_d2  <= C_KHI - signed(resize(p4_f0, 15));
            p5_tg  <= p4_t * signed('0' & c_gain);
            p5_f0 <= p4_f0; p5_fb0 <= p4_fb0;
            -- 6: key (7-bit over 1/32 unit), the texture in field units / 4 (12 bits)
            p6_key <= f_key(p5_d1, p5_d2);
            p6_tg  <= p5_tg(17 downto 6);
            p6_f0 <= p5_f0; p6_fb0 <= p5_fb0;
            -- 7: key x texture (the B field's copy comes back from the ring 36 virtual px later)
            p7_kt <= signed('0' & p6_key) * p6_tg;
            p7_f0 <= p6_f0; p7_fb0 <= p6_fb0;
            -- 8: the R / G field: + kT / 128
            v_f := unsigned(resize(signed('0' & p7_f0), 14) + resize(p7_kt(19 downto 7), 14));
            if s_slant = '1' then p8_f1 <= '0' & v_f(11 downto 0); else p8_f1 <= v_f(12 downto 0); end if;
            p8_fb0 <= p7_fb0;
            -- 9: table addresses (the red table reads the RAM; the fill owns it in the zone)
            if s_zone = '0' then t_ra <= p8_f1(12 downto 3); else t_ra <= t_wa; end if;
            r_g_a <= p8_f1(12 downto 3);
            p9_fb0 <= p8_fb0;
            -- 10: RAM / ROM; the B field: + its texture term (x8, zero for the first 42 px of the line)
            if g_cnt < 52 then v_fb := resize(p9_fb0, 14);
            else v_fb := unsigned(resize(signed('0' & p9_fb0), 14) + resize(r_tq2, 14)); end if;
            if s_slant = '1' then p10_fbs <= '0' & v_fb(11 downto 0); else p10_fbs <= v_fb(12 downto 0); end if;
            -- 11: fabric copies; |fbs - centre|
            p11_r <= t_rq; p11_g <= r_g_q;
            v_bw := signed(resize(p10_fbs, 15)) - C_RIB;
            if v_bw < 0 then v_bw := -v_bw; end if;
            p11_db <= unsigned(v_bw(12 downto 0));
            -- 12: slant wraps the distance at 1.0
            p12_r <= p11_r; p12_g <= p11_g;
            if s_slant = '1' and p11_db > 2048 then p12_db <= to_unsigned(4096, 13) - p11_db; else p12_db <= p11_db; end if;
            -- 13: B window (soft edge 1/32)
            p13_r <= p12_r; p13_g <= p12_g;
            v_bw := signed(resize(c_hwb, 15)) - signed(resize(p12_db, 15));     -- down to -5365: the x2 needs 15 bits
            p13_b <= f_clamp8(shift_left(v_bw, 1));
            -- 14: Ground Black dims the green shaper outside the blue window; Palette permutes the guns
            if s_blk = '1' and p13_b < 64 then v_g := "000" & p13_g(7 downto 3); else v_g := p13_g; end if;
            case to_integer(s_k6) is
                when 0 => p14_r <= p13_r; p14_g <= v_g;   p14_b <= p13_b;
                when 1 => p14_r <= v_g;   p14_g <= p13_b; p14_b <= p13_r;
                when 2 => p14_r <= p13_b; p14_g <= p13_r; p14_b <= v_g;
                when 3 => p14_r <= p13_r; p14_g <= p13_b; p14_b <= v_g;
                when 4 => p14_r <= v_g;   p14_g <= p13_r; p14_b <= p13_b;
                when 5 => p14_r <= p13_b; p14_g <= v_g;   p14_b <= p13_r;
                when 6 => p14_r <= not p13_r; p14_g <= not v_g; p14_b <= not p13_b;
                when others => p14_r <= not v_g; p14_g <= not p13_b; p14_b <= not p13_r;
            end case;
            -- 15: the matrix partials (constant shift-adds): Y' = (77 R + 150 G + 29 B) >> 8,
            --     Cr = (225 R - 188 G - 36 B) >> 7, Cb = (-76 R - 149 G + 225 B) >> 7
            p15_yr <= shift_left(resize(p14_r, 16), 6) + shift_left(resize(p14_r, 16), 4) - shift_left(resize(p14_r, 16), 2) + resize(p14_r, 16);   -- 77
            p15_yg <= shift_left(resize(p14_g, 16), 7) + shift_left(resize(p14_g, 16), 4) + shift_left(resize(p14_g, 16), 2) + shift_left(resize(p14_g, 16), 1);   -- 150
            p15_yb <= shift_left(resize(p14_b, 16), 5) - shift_left(resize(p14_b, 16), 2) + resize(p14_b, 16);   -- 29
            p15_rr <= shift_left(resize(p14_r, 17), 8) - shift_left(resize(p14_r, 17), 5) + resize(p14_r, 17);   -- 225
            p15_rg <= shift_left(resize(p14_g, 17), 7) + shift_left(resize(p14_g, 17), 6) - shift_left(resize(p14_g, 17), 2);   -- 188
            p15_rb <= shift_left(resize(p14_b, 17), 5) + shift_left(resize(p14_b, 17), 2);   -- 36
            p15_br <= shift_left(resize(p14_r, 17), 6) + shift_left(resize(p14_r, 17), 3) + shift_left(resize(p14_r, 17), 2);   -- 76
            p15_bg <= shift_left(resize(p14_g, 17), 7) + shift_left(resize(p14_g, 17), 4) + shift_left(resize(p14_g, 17), 2) + resize(p14_g, 17);   -- 149
            p15_bb <= shift_left(resize(p14_b, 17), 8) - shift_left(resize(p14_b, 17), 5) + resize(p14_b, 17);   -- 225
            -- 16: sums
            p16_y  <= resize(p15_yr, 17) + resize(p15_yg, 17) + resize(p15_yb, 17);
            p16_cr <= signed(resize(p15_rr, 18)) - signed(resize(p15_rg, 18)) - signed(resize(p15_rb, 18));
            p16_cb <= signed(resize(p15_bb, 18)) - signed(resize(p15_br, 18)) - signed(resize(p15_bg, 18));
            -- 17: luma x4.25 (16/13 prescale) capped at 876; chroma to 10 bits, clamped (Pastel halves it)
            v_y8 := p16_y(15 downto 8);
            v_yn := shift_left(resize(v_y8, 11), 2) + resize(v_y8(7 downto 2), 11);
            if v_yn > 876 then p17_yn <= to_unsigned(876, 10); else p17_yn <= v_yn(9 downto 0); end if;
            if s_pas = '1' then v_c := p16_cr(17 downto 8); else v_c := p16_cr(16 downto 7); end if;
            if v_c < -488 then p17_u <= to_unsigned(24, 10); elsif v_c > 488 then p17_u <= to_unsigned(1000, 10); else p17_u <= to_unsigned(512, 10) + unsigned(v_c); end if;
            if s_pas = '1' then v_c := p16_cb(17 downto 8); else v_c := p16_cb(16 downto 7); end if;
            if v_c < -488 then p17_v <= to_unsigned(24, 10); elsif v_c > 488 then p17_v <= to_unsigned(1000, 10); else p17_v <= to_unsigned(512, 10) + unsigned(v_c); end if;
            -- 18: compose (Pastel lifts the luma); neutral outside the generated avid
            v_yn := resize(p17_yn, 11);
            if s_pas = '1' then v_yn := to_unsigned(320, 11) + resize(shift_right(resize(p17_yn, 13) + shift_left(resize(p17_yn, 13), 2), 3), 11); end if;
            if s_gav_sr(15) = '0' then
                s_y18 <= to_unsigned(64, 10); s_u18 <= to_unsigned(512, 10); s_v18 <= to_unsigned(512, 10);
            else
                s_y18 <= resize(v_yn + 64, 10); s_u18 <= p17_u; s_v18 <= p17_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(15);     -- the same pixel as s_u18 / s_v18 (the compose gate)
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: chroma through [1 2 2 2 2 2 2 2 1]/16 (two adjacent
    -- 8-sums off a block-RAM ring), luma through [1 2 1]/4 delayed to the same
    -- centre (the HD HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(12 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_s3 : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to neutral once 8 blank samples are in, so a glitched sample can't
            -- offset the colours for good.  Stage 19 sums, stage 20 output (centred 4 px back).
            if s_bah = "00000000" then
                s_bau <= to_unsigned(128 * 8, 11); s_bav <= to_unsigned(128 * 8, 11);
            else
                s_bau <= s_bau + s_u18(9 downto 2) - unsigned(r_ocq(15 downto 8));
                s_bav <= s_bav + s_v18(9 downto 2) - unsigned(r_ocq(7 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            v_sc := resize(s_bau, 13) + s_baud; s_u20 <= v_sc(11 downto 4) & "00";
            v_sc := resize(s_bav, 13) + s_bavd; s_v20 <= v_sc(11 downto 4) & "00";
            s_u21 <= s_u20; s_v21 <= s_v20;
            s_u22 <= s_u21; s_v22 <= s_v21;
            -- luma [1 2 1]/4 (centred on the stage-18 pixel one clock back) then six delays, so that at the
            -- luma 13/16 stage it sits on the same pixel as the chroma 9-tap (centred 4 px back)
            s_y18d <= s_y18; s_y18dd <= s_y18d;
            v_s3 := resize(s_y18, 12) + shift_left(resize(s_y18d, 12), 1) + resize(s_y18dd, 12); s_y19 <= v_s3(11 downto 2);
            s_y20 <= s_y19; s_y21 <= s_y20; s_y22 <= s_y21; s_y22b <= s_y22; s_y22c <= s_y22b; s_y22d <= s_y22c;
            -- 23: luma 13/16
            if s_y22d > 64 then v_dl := s_y22d - 64; else v_dl := (others => '0'); end if;
            s_y23 <= s_y22d - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u23 <= s_u22; s_v23 <= s_v22;
            -- 24: blanking gate.  The output avid sits where the input avid was (32 clocks on, an even offset
            -- so the hsync parity holds); the filter-centred picture lands exactly on it.
            if s_gav_sr(25) = '1' then
                s_y24 <= s_y23; s_u24 <= s_u23; s_v24 <= s_v23;
            else
                s_y24 <= to_unsigned(64, 10); s_u24 <= to_unsigned(512, 10); s_v24 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= r_syq2(0);
    data_out.vsync_n <= r_syq2(1);
    data_out.field_n <= r_syq2(2);
    data_out.avid    <= s_gav_sr(26);

end architecture zigzag;
