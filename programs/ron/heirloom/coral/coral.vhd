-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Coral v0.1.1 -- Heirloom from the video clips, built from the still WIN_20230905_00_44_28 t0.5
-- (clip frame 15 at 0.5 s; the whole 17 s clip, and clip 00_44_50, is one patch cycling with an 8.9 s period).
--
-- The still: a red bulb at the bottom centre under a shallow red "v" bar and an eye of nested red chevrons,
-- fans of red / cyan arcs sweeping down from the upper corners, a violet-pink ground with a cyan sky, all
-- mirrored left-right, fine 27-px diagonal ripples (held in 48-px bricks off the red), the line starts held.
--
-- The patch, as rebuilt (coral.py has the tables; .heirloom_work/rb_coral/notes.md the measurements), in
-- virtual 1080-line coordinates, F in bands (Q8):
--   F   = g cos(2 pi (y - yr(t)) / 2215)        a free-running vertical sine, rolling DOWN 248 lines/s (8.9 s)
--       - A (1 - cos(pi |X| / 1000)) / 2         a mirrored horizontal cosine (low at the edges: the arc fans)
--       - D max(0, 1 - |X|/420 - |Y|/ry)          a diamond key, the eye (ry 250 above / 420 below y 430)
--       + camera luma x K6
--   w   = 1 - 1.3 dist(F x rings - 0.815, integer)   a triangle wavefolder into the colouriser: red on every band
--   w   = max(w, ground(cos))                     the ground rises from cyan to violet where the sine is high
--   w   = 1 where F > fz + 1.17                   the travelling outer zone (red tide)
--   w   = 1 inside the still's solid shapes       the triangle, the hooked "v" bar and the hexagon bulb: a per-row
--                                                 table (two runs per row) measured on the still, translated down
--                                                 with the rolling sine (one shape row per virtual line, 2215-line
--                                                 wrap), widened / narrowed by Bulb, Swell and the camera
--
--   K1  Rings     band density (x0.33 .. x3, exponential)
--   K2  Arcs      depth of the horizontal cosine: the corner arc fans (flat .. dense, steep fans)
--   K3  Eye       depth of the diamond key: no eye .. deep nested chevrons and diamonds
--   K4  Speed     bipolar roll of the vertical sine (+-8 deadband); 700 = the clip, 248 lines/s down
--   K5  Bulb      outer-zone level: no bulb .. red floods the frame
--   K6  Camera    how far the picture (or the Osc) bends every band
--   S7  Flip      the patch upside down: the bulb hangs from the top, the tide rolls up
--   S8  Palette   Coral / Lagoon
--   S9  Twin      two corals side by side
--   S10 Bands     Fold (the photo's soft wavefolder) / Steps (three hard posterised levels)
--   S11 Source    Video / Osc
--   P12 SWELL     the rolling sine's amplitude: 0 = a calm, nearly still eye; 360 = the photo; 100 % = a red tide
--
-- Engine: vblank-zone sequencer (shared glide, one serial multiplier, exp2 ring curve), a per-line engine kicked
-- at each line start computing the NEXT line (diamond row, the sine's phase -> quarter-sine ROM -> x g, the
-- ground level, a tear jitter, the ripple phase), a 19-stage pixel path (hold, |X|, twin, H cosine ROM x A,
-- diamond, the sum, x rings, the fold, palette ROM, ripple), the shell's output conditioning (Y, U, V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes with block-RAM taps and the resync, luma 13/16) and the blanking gate.
-- Generated avid; one latency (24) in every mode; colours in the hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.coral_pkg.all;
use work.all;

architecture coral of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks
    constant C_INC : integer := 7574; -- sine phase per virtual line (2^24 / 2215)

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

    -- ROMs (one read port each, registered address and data)
    signal r_pal_a : unsigned(8 downto 0) := (others => '0');
    signal r_pal_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_ht_a  : unsigned(9 downto 0) := (others => '0');
    signal r_ht_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal r_sn_a  : unsigned(8 downto 0) := (others => '0');
    signal r_sh_a  : unsigned(8 downto 0) := (others => '0');
    signal r_sh_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_sn_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal r_ex_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q  : std_logic_vector(15 downto 0) := (others => '0');

    attribute no_rw_check : boolean;

    -- output box rings (the 8-back taps of the two running sums) and the sync delay ring
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal ba1, ba2, bb1, bb2 : t_r16 := (others => x"0000");
    attribute no_rw_check of ba1 : signal is true;
    attribute no_rw_check of ba2 : signal is true;
    attribute no_rw_check of bb1 : signal is true;
    attribute no_rw_check of bb2 : signal is true;
    signal rg_wa, rg_ra, sy_ra : unsigned(7 downto 0) := (others => '0');
    signal ba1_q, ba2_q, bb1_q, bb2_q : std_logic_vector(15 downto 0) := x"0000";
    type t_r4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal syr : t_r4 := (others => "1111");
    attribute no_rw_check of syr : signal is true;
    signal syr_q, syr_c : std_logic_vector(3 downto 0) := "1111";

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
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(700 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(384 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(360 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_flip, s_pal, s_twin, s_step, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field values
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_neg  : std_logic := '0';
    signal f_a    : unsigned(35 downto 0) := (others => '0');
    signal f_b    : unsigned(15 downto 0) := (others => '0');
    signal f_acc  : unsigned(35 downto 0) := (others => '0');
    signal c_g    : unsigned(10 downto 0) := to_unsigned(639, 11);     -- sine amplitude, Q8 bands
    signal c_a    : unsigned(10 downto 0) := to_unsigned(897, 11);     -- H cosine depth, Q8
    signal c_d    : unsigned(10 downto 0) := to_unsigned(768, 11);     -- diamond depth, Q8
    signal c_kx   : unsigned(10 downto 0) := to_unsigned(468, 11);     -- D / 420 per virtual px, Q8.8
    signal c_kyt  : unsigned(10 downto 0) := to_unsigned(786, 11);     -- D / 250 per line, Q8.8
    signal c_kr   : unsigned(9 downto 0) := to_unsigned(256, 10);      -- rings, Q8 (256 = x1)
    signal c_fz   : signed(13 downto 0) := to_signed(620, 14);         -- outer-zone level, Q8
    signal c_dth  : signed(23 downto 0) := to_signed(-31320, 24);      -- sine phase step per field
    signal c_th0  : unsigned(23 downto 0) := to_unsigned(9589190, 24); -- the field's sine phase (the still)
    signal c_cg   : unsigned(5 downto 0) := to_unsigned(24, 6);        -- camera gain, Q4 per luma step
    signal z_gb   : unsigned(10 downto 0) := to_unsigned(154, 11);
    signal c_grow : signed(11 downto 0) := (others => '0');          -- solid-shape widening, virtual px
    signal c_off  : unsigned(11 downto 0) := (others => '0');        -- shape-row offset (the roll since the still), lines
    signal z_dt   : unsigned(23 downto 0) := (others => '0');

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
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_ym, e_ye : unsigned(10 downto 0) := (others => '0');
    signal e_ay   : unsigned(10 downto 0) := (others => '0');
    signal e_neg  : std_logic := '0';
    signal e_th   : unsigned(23 downto 0) := (others => '0');
    signal e_sn   : std_logic := '0';
    signal e_rn   : signed(16 downto 0) := (others => '0');
    signal e_lf   : std_logic_vector(15 downto 0) := x"ACE1";
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    signal n_p    : signed(13 downto 0) := (others => '0');
    signal n_r    : signed(12 downto 0) := (others => '0');
    signal n_wf   : unsigned(7 downto 0) := (others => '0');
    signal n_jit  : signed(3 downto 0) := (others => '0');
    signal n_sp   : unsigned(15 downto 0) := (others => '0');
    signal e_yr   : signed(12 downto 0) := (others => '0');
    signal e_sv   : std_logic := '0';
    signal n_nc8  : unsigned(7 downto 0) := (others => '0');
    signal n_wc2, n_hi2, n_ho2 : unsigned(9 downto 0) := (others => '0');

    -- beam
    signal b_x   : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_sp, b_sph : unsigned(15 downto 0) := (others => '0');
    signal b_blk : unsigned(6 downto 0) := (others => '0');
    signal c_xb0 : unsigned(11 downto 0) := to_unsigned(16, 12);
    signal c_spx : unsigned(12 downto 0) := to_unsigned(2428, 13);
    signal l_p   : signed(13 downto 0) := (others => '0');
    signal l_r   : signed(12 downto 0) := (others => '0');
    signal l_wf  : unsigned(7 downto 0) := (others => '0');
    signal l_jit : signed(3 downto 0) := (others => '0');
    signal l_nc8 : unsigned(7 downto 0) := (others => '0');
    signal l_wc2, l_hi2, l_ho2 : unsigned(9 downto 0) := (others => '0');

    -- pixel path
    signal p2_x   : signed(12 downto 0) := (others => '0');
    signal p3_x   : unsigned(10 downto 0) := (others => '0');
    signal p4_x   : unsigned(9 downto 0) := (others => '0');
    signal p4_lm  : unsigned(7 downto 0) := (others => '0');
    signal p5_k1  : unsigned(15 downto 0) := (others => '0');
    signal p5_k0  : unsigned(14 downto 0) := (others => '0');
    signal p5_cam : signed(15 downto 0) := (others => '0');
    signal p6_kx  : unsigned(21 downto 0) := (others => '0');
    signal p6_cam, p7_cam, p8_cam, p9_cam : signed(11 downto 0) := (others => '0');
    signal p7_ht  : unsigned(8 downto 0) := (others => '0');
    signal p7_pk  : signed(14 downto 0) := (others => '0');
    signal p8_a1  : unsigned(13 downto 0) := (others => '0');
    signal p8_a0  : unsigned(14 downto 0) := (others => '0');
    signal p8_pit, p9_pit : unsigned(12 downto 0) := (others => '0');
    signal p9_hx  : unsigned(11 downto 0) := (others => '0');
    signal p10_f  : signed(14 downto 0) := (others => '0');
    signal p11_g1 : signed(20 downto 0) := (others => '0');
    signal p11_g0 : signed(20 downto 0) := (others => '0');
    signal p11_oz, p12_oz, p13_oz, p14_oz : std_logic := '0';
    signal p5_x, p6_x, p7_x, p8_x : unsigned(9 downto 0) := (others => '0');
    signal p7_g   : signed(12 downto 0) := (others => '0');
    signal p8_xm, p8_xp : signed(12 downto 0) := (others => '0');
    signal p9_sc, p9_so, p10_sk : std_logic := '0';
    signal p12_g  : signed(16 downto 0) := (others => '0');
    signal p13_d  : unsigned(7 downto 0) := (others => '0');
    signal p14_w  : unsigned(7 downto 0) := (others => '0');
    signal p15_w  : unsigned(7 downto 0) := (others => '0');
    signal p16_red, p17_red, p18_red : std_logic := '0';
    signal p18_y, p18_u, p18_v : unsigned(9 downto 0) := (others => '0');
    signal p18_t  : signed(6 downto 0) := (others => '0');
    type t_sp is array (2 to 17) of unsigned(5 downto 0);
    signal q_spl, q_sph : t_sp := (others => (others => '0'));
    signal p16_t  : signed(6 downto 0) := (others => '0');
    signal p17_t  : signed(6 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories: line buffer (collisions possible: read-before-write kept),
    -- ROMs, the box rings, the sync ring
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
            r_pal_q <= C_CO_PAL(to_integer(r_pal_a));
            r_ht_q  <= C_CO_HT(to_integer(r_ht_a));
            r_sn_q  <= C_CO_SIN(to_integer(r_sn_a));
            r_ex_q  <= C_CO_EXP2(to_integer(r_ex_a));
            r_sh_q  <= C_CO_SHP(to_integer(r_sh_a));
        end if;
    end process p_rom;

    p_rings : process(clk)
    begin
        if rising_edge(clk) then
            ba1_q <= ba1(to_integer(rg_ra));
            ba1(to_integer(rg_wa)) <= std_logic_vector(s_y19) & std_logic_vector(s_u19(9 downto 4));
            ba2_q <= ba2(to_integer(rg_ra));
            ba2(to_integer(rg_wa)) <= std_logic_vector(s_u19(3 downto 0)) & std_logic_vector(s_v19) & "00";
            bb1_q <= bb1(to_integer(rg_ra));
            bb1(to_integer(rg_wa)) <= std_logic_vector(s_y21) & std_logic_vector(s_u21(9 downto 4));
            bb2_q <= bb2(to_integer(rg_ra));
            bb2(to_integer(rg_wa)) <= std_logic_vector(s_u21(3 downto 0)) & std_logic_vector(s_v21) & "00";
            syr_q <= syr(to_integer(sy_ra));
            syr(to_integer(rg_wa)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
        end if;
    end process p_rings;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_d  : signed(11 downto 0);
        variable v_e  : signed(11 downto 0);
        variable v_k  : unsigned(9 downto 0);
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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_flip <= registers_in(6)(0);
                s_pal  <= registers_in(6)(1);
                s_twin <= registers_in(6)(2);
                s_step <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9); c_spx <= to_unsigned(4855, 13);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9); c_spx <= to_unsigned(3641, 13);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9); c_spx <= to_unsigned(2427, 13);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);
            c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, seven channels
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

            -- serial multiplier (16 iterations; the product is complete 17 clocks after the load)
            if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
            f_a <= f_a(34 downto 0) & '0';
            f_b <= '0' & f_b(15 downto 1);

            -- per-field values: zc 30 .. 145
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30 =>                                        -- swell: g = 154 + p12 * 1382 / 1024 up to the photo (360),
                        if s_gp(12 downto 3) > 360 then               -- then 639 + (p12 - 360) * 784 / 1024
                            f_a <= to_unsigned(784, 36); f_b <= resize(s_gp(12 downto 3) - 360, 16); z_gb <= to_unsigned(639, 11);
                        else
                            f_a <= to_unsigned(1382, 36); f_b <= resize(s_gp(12 downto 3), 16); z_gb <= to_unsigned(154, 11);
                        end if;
                        f_acc <= (others => '0');
                    when 48 =>
                        c_g <= z_gb + resize(f_acc(20 downto 10), 11);
                        f_a <= to_unsigned(1180, 36); f_b <= resize(s_g2(12 downto 3), 16); f_acc <= (others => '0');
                    when 66 =>                                        -- arcs: A = 307 + k2 * 1180 / 1024; eye: D = 3 k3 / 2
                        c_a <= to_unsigned(307, 11) + resize(f_acc(20 downto 10), 11);
                        c_d <= resize(shift_right(resize(s_g3(12 downto 3), 12) + shift_left(resize(s_g3(12 downto 3), 12), 1), 1), 11);
                    when 67 =>
                        f_a <= to_unsigned(624, 36); f_b <= resize(c_d, 16); f_acc <= (others => '0');
                    when 85 =>
                        c_kx <= resize(f_acc(20 downto 10), 11);
                        f_a <= to_unsigned(1049, 36); f_b <= resize(c_d, 16); f_acc <= (others => '0');
                    when 103 =>                                       -- rings: e = (k1 - 512) * 202 / 256, Q8 octaves
                        c_kyt <= resize(f_acc(20 downto 10), 11);
                        v_d := signed(resize(s_g1(12 downto 3), 12)) - 512;
                        if v_d < 0 then z_neg <= '1'; f_b <= unsigned(resize(-v_d, 16));
                        else z_neg <= '0'; f_b <= unsigned(resize(v_d, 16)); end if;
                        f_a <= to_unsigned(202, 36); f_acc <= (others => '0');
                    when 121 =>
                        if z_neg = '1' then v_e := -signed(resize(f_acc(18 downto 8), 12));
                        else v_e := signed(resize(f_acc(18 downto 8), 12)); end if;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 123 =>
                        z_mt <= unsigned(r_ex_q);
                    when 124 =>                                       -- kr = 2^e * 256 = mt >> (7 - sh)
                        case to_integer(z_sh) is
                            when -2     => c_kr <= resize(z_mt(15 downto 9), 10);
                            when -1     => c_kr <= resize(z_mt(15 downto 8), 10);
                            when 0      => c_kr <= resize(z_mt(15 downto 7), 10);
                            when others => c_kr <= z_mt(15 downto 6);
                        end case;
                        -- bulb: fz = 620 + (512 - k5) * 2; the shapes widen by (k5 - 512) + (g - 639) px
                        v_d := 512 - signed(resize(s_g5(12 downto 3), 12));
                        c_fz <= to_signed(620, 14) + shift_left(resize(v_d, 14), 1);
                        c_grow <= -v_d + signed(resize(c_g, 12)) - 639;
                        -- speed: (k4 - 512) with a +-8 deadband, x 174
                        v_d := signed(resize(s_g4(12 downto 3), 12)) - 512;
                        if v_d > 8 then
                            f_b <= unsigned(resize(v_d - 8, 16)); z_neg <= '1';
                        elsif v_d < -8 then
                            f_b <= unsigned(resize(-8 - v_d, 16)); z_neg <= '0';
                        else
                            f_b <= (others => '0'); z_neg <= '0';
                        end if;
                        f_a <= to_unsigned(174, 36); f_acc <= (others => '0');
                    when 142 =>
                        if z_neg = '1' then c_dth <= -signed(resize(f_acc(19 downto 0), 24));
                        else c_dth <= signed(resize(f_acc(19 downto 0), 24)); end if;
                    when 143 =>
                        c_th0 <= unsigned(signed(c_th0) + c_dth);
                        c_cg <= s_g6(12 downto 7);
                    when 144 =>                                       -- shape offset: (theta_still - theta0) * 2215 lines
                        z_dt <= to_unsigned(9589190, 24) - c_th0;
                    when 145 =>
                        f_a <= to_unsigned(2215, 36); f_b <= z_dt(23 downto 8); f_acc <= (others => '0');
                    when 163 =>
                        c_off <= f_acc(27 downto 16);
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
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
    -- 0), computes the NEXT line: the diamond row P = D - |Y| ky; the sine
    -- phase theta0 + y inc -> quarter-sine ROM (cos) -> R = g cos; the ground
    -- level from cos; a tear jitter (+-2 virtual px on about half the lines);
    -- the ripple phase at the line start.  One serial product at a time
    -- (16 iterations, complete at load + 17).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y  : signed(11 downto 0);
        variable v_p  : signed(14 downto 0);
        variable v_t  : signed(17 downto 0);
        variable v_th : unsigned(23 downto 0);
        variable v_yr : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    if e_vy(18 downto 8) > 1079 then e_ym <= to_unsigned(1079, 11); else e_ym <= e_vy(18 downto 8); end if;
                when 2 =>
                    if s_flip = '1' then e_ye <= 1079 - e_ym; else e_ye <= e_ym; end if;
                    e_lf <= e_lf(14 downto 0) & (e_lf(15) xor e_lf(13) xor e_lf(12) xor e_lf(10));
                when 3 =>
                    v_y := signed('0' & e_ye) - 430;
                    if v_y < 0 then e_ay <= unsigned(resize(-v_y, 11)); e_neg <= '1';
                    else e_ay <= unsigned(v_y(10 downto 0)); e_neg <= '0'; end if;
                    if e_lf(0) = '1' then
                        case e_lf(3 downto 1) is
                            when "000" | "001" => n_jit <= to_signed(-2, 4);
                            when "010"         => n_jit <= to_signed(-1, 4);
                            when "011" | "100" => n_jit <= to_signed(0, 4);
                            when "101"         => n_jit <= to_signed(1, 4);
                            when others        => n_jit <= to_signed(2, 4);
                        end case;
                    else
                        n_jit <= (others => '0');
                    end if;
                when 4 =>
                    m_a <= resize(e_ay, 36);
                    if e_neg = '1' then m_b <= resize(c_kyt, 16); else m_b <= resize(c_kx, 16); end if;
                    m_acc <= (others => '0');
                when 21 =>
                    v_p := signed(resize(c_d, 15)) - signed(resize(m_acc(21 downto 8), 15));
                    if v_p < -8191 then n_p <= to_signed(-8191, 14); else n_p <= v_p(13 downto 0); end if;
                    m_a <= resize(e_ye, 36); m_b <= to_unsigned(C_INC, 16); m_acc <= (others => '0');
                when 38 =>
                    e_th <= c_th0 + m_acc(23 downto 0) + to_unsigned(4194304, 24);   -- + 1/4 cycle: cos
                when 39 =>
                    if e_th(22) = '1' then r_sn_a <= not e_th(21 downto 13); else r_sn_a <= e_th(21 downto 13); end if;
                    e_sn <= e_th(23);
                when 41 =>
                    m_a <= resize(c_g, 36); m_b <= unsigned(r_sn_q); m_acc <= (others => '0');
                    if e_sn = '1' then e_rn <= -signed(resize(unsigned(r_sn_q), 17));
                    else e_rn <= signed(resize(unsigned(r_sn_q), 17)); end if;
                when 42 =>                                            -- ground: 13 + (max(cos + 0.3, 0) * 3) >> 10
                    v_t := resize(e_rn, 18) + 9830;
                    if v_t < 0 then n_wf <= to_unsigned(13, 8);
                    else n_wf <= to_unsigned(13, 8) + resize(shift_right(unsigned(v_t(16 downto 0)) + shift_left(unsigned(v_t(16 downto 0)), 1), 10), 8); end if;
                when 58 =>
                    if e_sn = '1' then n_r <= -signed(resize(m_acc(26 downto 15), 13));
                    else n_r <= signed(resize(m_acc(26 downto 15), 13)); end if;
                    -- ripple phase at the line start: (x0 + y) * 2427, x0 = the beam's first virtual px
                    m_a <= resize(e_ym + resize(shift_right(c_xb0, 1), 11), 36); m_b <= to_unsigned(2427, 16); m_acc <= (others => '0');
                when 75 =>
                    n_sp <= m_acc(15 downto 0);
                when 76 =>                                            -- shape row yr = ye - offset (mod 2215)
                    e_yr <= signed(resize(e_ye, 13)) - signed(resize(c_off, 13));
                when 77 =>
                    if e_yr < 0 then e_yr <= e_yr + 2215; end if;
                when 78 =>
                    if e_yr >= 384 and e_yr < 1408 then
                        e_sv <= '1'; v_yr := e_yr - 384; r_sh_a <= unsigned(v_yr(9 downto 1));
                    else
                        e_sv <= '0';
                    end if;
                when 80 =>
                    if e_sv = '1' then
                        n_nc8 <= unsigned(r_sh_q(31 downto 27)) & "000";
                        n_wc2 <= unsigned(r_sh_q(26 downto 18)) & '0';
                        n_hi2 <= unsigned(r_sh_q(17 downto 9)) & '0';
                        n_ho2 <= unsigned(r_sh_q(8 downto 0)) & '0';
                    else
                        n_nc8 <= (others => '0'); n_wc2 <= (others => '0'); n_hi2 <= (others => '0'); n_ho2 <= (others => '0');
                    end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1): x in h (half virtual px); the line's values latched at
    -- the line start; the ripple phase advances 2427 per virtual px and is
    -- also held for 48-px bricks; the camera read issued from the pixel index
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x   <= c_xb0;
                b_px  <= (others => '0');
                b_sp  <= n_sp;
                b_sph <= n_sp;
                b_blk <= (others => '0');
                l_p   <= n_p;
                l_r   <= n_r;
                l_wf  <= n_wf;
                l_jit <= n_jit;
                l_nc8 <= n_nc8;
                l_wc2 <= n_wc2;
                l_hi2 <= n_hi2;
                l_ho2 <= n_ho2;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x + s_xs;
                b_sp <= b_sp + resize(c_spx, 16);
                if b_blk + s_xs >= 96 then
                    b_blk <= b_blk + s_xs - 96;
                    b_sph <= b_sp + resize(c_spx, 16);
                else
                    b_blk <= b_blk + s_xs;
                end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_x  : signed(12 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_l  : signed(8 downto 0);
        variable v_s  : unsigned(19 downto 0);
        variable v_g  : signed(25 downto 0);
        variable v_w  : unsigned(7 downto 0);
        variable v_p  : unsigned(5 downto 0);
        variable v_t  : signed(6 downto 0);
        variable v_y, v_cu, v_cv : signed(11 downto 0);
        variable v_t5, v_t3 : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: virtual px, the line-start hold (x < 78 reads x = 78) and the line's tear jitter
            v_u := b_x(11 downto 1);
            if v_u < 78 then v_x := to_signed(78, 13); else v_x := signed(resize(v_u, 13)); end if;
            p2_x <= v_x + resize(l_jit, 13);
            q_spl(2) <= b_sp(15 downto 10); q_sph(2) <= b_sph(15 downto 10);
            for i in 3 to 17 loop q_spl(i) <= q_spl(i - 1); q_sph(i) <= q_sph(i - 1); end loop;
            -- 3: X = |x - 960|
            v_x := p2_x - 960;
            if v_x < 0 then p3_x <= unsigned(resize(-v_x, 11)); else p3_x <= unsigned(v_x(10 downto 0)); end if;
            -- 4: Twin folds each half again about |X| = 480; clamp; the camera luma (line buffer copy)
            if s_twin = '1' then
                if p3_x >= 480 then v_u := p3_x - 480; else v_u := 480 - p3_x; end if;
                v_u := v_u(9 downto 0) & '0';
            else
                v_u := p3_x;
            end if;
            if v_u > 1023 then p4_x <= (others => '1'); else p4_x <= v_u(9 downto 0); end if;
            p4_lm <= unsigned(s_lb_q);
            -- 5: H cosine ROM address; X * kx (diamond slope) as two partial products; camera x gain
            r_ht_a <= p4_x;
            p5_x <= p4_x;
            p5_k1 <= p4_x * c_kx(10 downto 5);
            p5_k0 <= p4_x * c_kx(4 downto 0);
            if s_src = '1' then v_l := signed('0' & p4_lm) - 120; else v_l := signed('0' & p4_lm); end if;
            p5_cam <= resize(v_l * signed('0' & c_cg), 16);
            -- 6: X kx sum; camera / 16
            p6_kx <= shift_left(resize(p5_k1, 22), 5) + resize(p5_k0, 22);
            p6_x <= p5_x;
            p6_cam <= resize(shift_right(p5_cam, 4), 12);
            -- 7: H cosine table (fabric copy); diamond row - X kx
            p7_ht <= unsigned(r_ht_q(8 downto 0));
            p7_pk <= resize(l_p, 15) - signed(resize(p6_kx(21 downto 8), 15));
            p7_cam <= p6_cam;
            -- 7: the shapes' widening (Bulb, Swell, camera / 8)
            p7_g <= resize(c_grow, 13) + resize(shift_right(p6_cam, 3), 13);
            p7_x <= p6_x;
            -- 8: A x HT partial products; the diamond key (clamped at 0)
            p8_a1 <= p7_ht * c_a(10 downto 6);
            p8_a0 <= p7_ht * c_a(5 downto 0);
            if p7_pk < 0 then p8_pit <= (others => '0'); else p8_pit <= unsigned(p7_pk(12 downto 0)); end if;
            p8_cam <= p7_cam;
            -- 8: X - grow, X + grow
            p8_xm <= signed(resize(p7_x, 13)) - p7_g;
            p8_xp <= signed(resize(p7_x, 13)) + p7_g;
            p8_x  <= p7_x;
            -- 9: inside the central run [Nc, Wc + grow) or the outer run [Hi - grow, Ho + grow)
            if p8_xm < signed(resize(l_wc2, 13)) and p8_x >= resize(l_nc8, 10) and l_wc2 /= 0 then p9_sc <= '1'; else p9_sc <= '0'; end if;
            if p8_xp >= signed(resize(l_hi2, 13)) and p8_xm < signed(resize(l_ho2, 13)) and l_ho2 /= 0 then p9_so <= '1'; else p9_so <= '0'; end if;
            -- 10: solid shape
            p10_sk <= p9_sc or p9_so;
            -- 9: Hx = A HT / 256
            v_s := shift_left(resize(p8_a1, 20), 6) + resize(p8_a0, 20);
            p9_hx <= v_s(19 downto 8);
            p9_pit <= p8_pit; p9_cam <= p8_cam;
            -- 10: F = R - Hx - diamond + camera   (Q8 bands)
            p10_f <= resize(l_r, 15) - signed(resize(p9_hx, 15)) - signed(resize(p9_pit, 15)) + resize(p9_cam, 15);
            -- 11: F x rings, two partial products; the outer zone
            p11_g1 <= resize(p10_f * signed('0' & c_kr(9 downto 5)), 21);
            p11_g0 <= resize(p10_f * signed('0' & c_kr(4 downto 0)), 21);
            if p10_f > resize(c_fz, 15) or p10_sk = '1' then p11_oz <= '1'; else p11_oz <= '0'; end if;
            -- 12: G = F rings / 256 - 0.815 band
            v_g := shift_left(resize(p11_g1, 26), 5) + resize(p11_g0, 26);
            p12_g <= resize(shift_right(v_g, 8), 17) - 209;
            p12_oz <= p11_oz;
            -- 13: distance to the nearest band centre (0 .. 128)
            if p12_g(7) = '1' then p13_d <= resize(256 - resize(unsigned(p12_g(7 downto 0)), 9), 8);
            else p13_d <= unsigned(p12_g(7 downto 0)); end if;
            p13_oz <= p12_oz;
            -- 14: the wavefolder w = 255 - 2 d (red on the band, cyan half way between bands)
            if p13_d >= 128 then p14_w <= (others => '0'); else p14_w <= not (p13_d(6 downto 0) & '0'); end if;
            p14_oz <= p13_oz;
            -- 15: the ground floor, the outer zone, Steps
            if p14_w < l_wf then v_w := l_wf; else v_w := p14_w; end if;
            if p14_oz = '1' then v_w := to_unsigned(255, 8); end if;
            if s_step = '1' then
                if v_w < 96 then v_w := to_unsigned(40, 8);
                elsif v_w < 200 then v_w := to_unsigned(140, 8);
                else v_w := to_unsigned(255, 8); end if;
            end if;
            p15_w <= v_w;
            -- 16: palette ROM address; the ripple (live on the red, held in 48-px bricks elsewhere)
            r_pal_a <= s_pal & p15_w;
            if p15_w >= 248 then p16_red <= '1'; v_p := q_spl(15); else p16_red <= '0'; v_p := q_sph(15); end if;
            if v_p(5) = '1' then v_t := signed(resize(63 - v_p, 7)) - 16; else v_t := signed(resize(v_p, 7)) - 16; end if;
            p16_t <= v_t;
            -- 17: palette (EBR)
            p17_t <= p16_t; p17_red <= p16_red;
            -- 18: fabric copy
            p18_y <= unsigned(r_pal_q(29 downto 20)); p18_u <= unsigned(r_pal_q(19 downto 10)); p18_v <= unsigned(r_pal_q(9 downto 0));
            p18_t <= p17_t; p18_red <= p17_red;
            -- 19: ripple offsets, clamps, black outside the generated avid
            v_t5 := shift_left(resize(p18_t, 12), 2) + resize(p18_t, 12);
            v_t3 := shift_left(resize(p18_t, 12), 1) + resize(p18_t, 12);
            if p18_red = '1' then
                v_y  := shift_right(v_t5, 3);
                v_cu := -shift_right(v_t3, 3);
                v_cv := -shift_right(resize(p18_t, 12), 2);
            else
                v_y  := -shift_right(v_t3, 2);
                v_cu := shift_right(resize(p18_t, 12), 2);
                v_cv := -shift_right(v_t3, 1);
            end if;
            v_y := signed(resize(p18_y, 12)) + v_y; v_cu := signed(resize(p18_u, 12)) + v_cu; v_cv := signed(resize(p18_v, 12)) + v_cv;
            if s_gav_sr(16) = '0' then
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            else
                if v_y < 64 then s_y19 <= to_unsigned(64, 10); elsif v_y > 940 then s_y19 <= to_unsigned(940, 10); else s_y19 <= unsigned(v_y(9 downto 0)); end if;
                if v_cu < 0 then s_u19 <= (others => '0'); elsif v_cu > 1023 then s_u19 <= (others => '1'); else s_u19 <= unsigned(v_cu(9 downto 0)); end if;
                if v_cv < 0 then s_v19 <= (others => '0'); elsif v_cv > 1023 then s_v19 <= (others => '1'); else s_v19 <= unsigned(v_cv(9 downto 0)); end if;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(16);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the 8-back taps come from block-RAM rings:
    -- the still's analog softness, and the HD HDMI bar fix), luma 13/16 and
    -- the blanking gate; the syncs ride a block-RAM ring of the same latency
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- ring addresses: one write per clock; the read, issued two clocks ahead and copied once,
            -- is the sample written 8 clocks before the one being added
            rg_wa <= rg_wa + 1;
            rg_ra <= rg_wa - 5;
            sy_ra <= rg_wa - 21;
            s_cay <= unsigned(ba1_q(15 downto 6)); s_cau <= unsigned(ba1_q(5 downto 0)) & unsigned(ba2_q(15 downto 12)); s_cav <= unsigned(ba2_q(11 downto 2));
            s_cby <= unsigned(bb1_q(15 downto 6)); s_cbu <= unsigned(bb1_q(5 downto 0)) & unsigned(bb2_q(15 downto 12)); s_cbv <= unsigned(bb2_q(11 downto 2));
            syr_c <= syr_q;
            -- The running 8-sums snap back to neutral (the off-avid input of both boxes) once 8 blank
            -- samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 20 sums, stage 21 output
            if s_bah = "00000000" then
                s_bay <= to_unsigned(64 * 8, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y19 - s_cay;
                s_bau <= s_bau + s_u19 - s_cau;
                s_bav <= s_bav + s_v19 - s_cav;
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: neutral off the generated avid (what box A gives there anyway)
            if s_gav_sr(18) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y21 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u21 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v21 <= v_sc(13 downto 4);
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(18);
            -- box B: stage 22 sums, stage 23 output
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(64 * 8, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y21 - s_cby;
                s_bbu <= s_bbu + s_u21 - s_cbu;
                s_bbv <= s_bbv + s_v21 - s_cbv;
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
    data_out.hsync_n <= syr_c(0);
    data_out.vsync_n <= syr_c(1);
    data_out.field_n <= syr_c(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture coral;
