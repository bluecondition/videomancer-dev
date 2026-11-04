-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Frogspawn v0.1.1 -- Heirloom (video phase), from the clip WIN_20230206_00_35_06 at 0.5 s.
--
-- v0.1.1: the palette's low end (the hearts on the saddle rows, the far-left heart column) refitted from sharp
-- samples of the still (no frame averages); S9 Diamond makes the rows triangular too; S10 folds the palette.
--
-- The picture: a green / black cellular contour lattice.  Round cells (concentric rings: red core, dark
-- green annulus, thin rainbow fringes, bright green ring, a black ring) sit at the maxima of a horizontal
-- oscillator (period 453.5 px, 4.2 cells per line); between two cells a narrow vertical EYE (a lens of
-- nested green rings, 66 x 190 px); under every eye on the saddle rows a pair of nested diamonds (the
-- "hearts").  A gain rising to the right pushes the field into the colouriser's top, where it saturates:
-- the right-hand cells and eyes are solid black ovals.  The whole lattice scrolls down at 150 px/s.
--
-- The patch, as fitted (frogspawn.py has the tables, notes.md the measurements):
--   F(x, y) = soft(r(x) (2 - W(u)) - 2) + V(y) + etch * luma
--   u       = (x - 963) mod 453.5 px; W(u) = min(hump, eye) in [0, 1]:
--             hump = (1 - cos(pi u / 200)) / 2   (1 beyond +-200 px): the round cell
--             eye  = E + (1 - E) |u - 0.465 T| / 62 px (E = 0.3): the narrow eye
--   r(x)    = max(0.45, 1 + a v + b v^2), v = (x - 960)/960 (a = 1.0, b = 0.2): the rising gain
--   soft    = a knee above 0 with slopes 1, 1/2, 1/4, 1/8 per 0.35 units: the saturation (black ovals)
--   V(y)    = cos(2 pi (y - 551)/531) + split cos(4 pi ..), scrolling 150 px/s down
--   colour  = PAL[F]: 1024 entries over F in [-2.6, 1.4), black above 1.03
--
--   K1  Cells   cell period: 1/2.8 .. 2.8 x the photo's (453 px at 512)
--   K2  Rows    row period: 1/2.8 .. 2.8 x the photo's (531 px at 512)
--   K3  Eyes    eye height: none (0) .. the photo (716) .. as tall as the cells (1023)
--   K4  Speed   bipolar, still at the centre (+-8): the lattice scrolls down (+) or up (-); 700 = the clip
--   K5  Tilt    the rising gain: black ovals on the left (0) .. flat (512) .. the photo (768) .. steep (1023)
--   K6  Etch    how far the camera (or the Osc) bends the lattice
--   S7  Twin    Off / On: the field mirrored about the centre (black ovals at both edges)
--   S8  Colours Green (the photo) / Pink (chroma negated: a pink lattice with green fringes)
--   S9  Cells   Round (cosine hump and rows) / Diamond (triangle hump and triangle rows: straight-sided diamonds)
--   S10 Top     Black (saturated field is black, the photo) / Rings (the palette folds about the top: every
--               black oval becomes a nested ring cell)
--   S11 Source  Video / Osc
--   P12 SPLIT   V's octave: 0 = single rows (the photo) .. stacked pairs of cells (the clip at 4 s) .. 100 %
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-field fill of the 1024-entry
-- waveform table W, a per-line engine (V from a quarter-wave cosine ROM), a beam that carries the H phase and
-- the gain DDA, a 15-stage pixel path to the compose, then the shell's output conditioning (two
-- [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync, luma 13/16) and the blanking gate.  Virtual
-- 1080-line coordinates (SD shows the centre 4:3).  Generated avid; one latency (C_LAT 20) in every mode.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.frogspawn_pkg.all;
use work.all;

architecture frogspawn of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant C_KX0  : integer := 18498;  -- H phase step per h at the photo's period (2^24 / (2 * 453.5))
    constant C_KY0  : integer := 31591;  -- V phase step per virtual line, Q24 turns (2^24 / 531)
    constant C_XC2  : integer := 1926;   -- cell centre x (h) = 963 px
    constant C_UE   : integer := 476;    -- eye apex index (0.465 * 1024)
    constant C_HS_R : integer := 290;    -- hump DDA step, round (512 / 452 Q8): cosine index per table index
    constant C_HS_D : integer := 580;    -- hump DDA step, diamond (1024 / 452 Q8)
    constant C_KNEE : integer := 358;    -- soft knee segment, Q10 (0.35)

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
    -- '1' for 50 / 60 field-per-second modes (the scroll step halves), '0' for 1080p
    function f_half(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 3 | 7 | 9 | 11 | 13 | 15 => return '0';
            when others                   => return '1';
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- palette black (entry 1023): the off-avid colour
    constant C_KW : std_logic_vector(31 downto 0) := C_FS_PAL(1023);
    constant C_BKY : unsigned(9 downto 0) := unsigned(C_KW(31 downto 22));
    constant C_BKU : unsigned(9 downto 0) := unsigned(C_KW(21 downto 12));
    constant C_BKV : unsigned(9 downto 0) := unsigned(C_KW(11 downto 2));

    -- line buffer: (luma >> 2) - 16, clipped at 0
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
    signal r_cs_a, t_csa, e_csa : unsigned(7 downto 0) := (others => '0');
    signal r_cs_q : unsigned(11 downto 0) := (others => '0');
    signal r_pal_a : unsigned(9 downto 0) := (others => '0');
    signal r_pal_q : std_logic_vector(31 downto 0) := (others => '0');

    -- waveform table W (Q10 in 0 .. 1023): written by the per-field fill in the vblank zone, read by the
    -- pixel path during active video (never both)
    type t_wt is array (0 to 1023) of unsigned(9 downto 0);
    signal wt : t_wt := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of wt : signal is true;
    signal w_we : std_logic := '0';
    signal w_wa, w_ra : unsigned(9 downto 0) := (others => '0');
    signal w_wd, w_rq : unsigned(9 downto 0) := (others => '0');

    -- fill engine
    signal t_run : std_logic := '0';
    signal t_i   : unsigned(10 downto 0) := (others => '0');
    signal t_ha  : unsigned(19 downto 0) := (others => '0');    -- hump cosine index, Q8
    signal t_ta  : unsigned(20 downto 0) := (others => '0');    -- eye triangle value, Q8 (Q10 value << 8)
    signal t_i1, t_i2, t_i3, t_i4 : unsigned(9 downto 0) := (others => '0');
    signal t_v1, t_v2, t_v3, t_v4 : std_logic := '0';
    signal t_a1  : unsigned(11 downto 0) := (others => '0');
    signal t_t1, t_t2, t_t3, t_t4 : unsigned(9 downto 0) := (others => '0');
    signal t_z2, t_z3 : unsigned(1 downto 0) := (others => '0');
    signal t_h2, t_h3, t_h4 : unsigned(9 downto 0) := (others => '0');
    signal t_csel : std_logic := '0';

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
    signal s_half : std_logic := '0';
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(716 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(768 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := (others => '0');
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_twin, s_pink, s_dia, s_wrap, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_sh1, z_sh2 : signed(2 downto 0) := (others => '0');
    signal z_mt1, z_mt2 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_a    : unsigned(19 downto 0) := (others => '0');
    signal z_neg  : std_logic := '0';
    signal c_kx   : unsigned(19 downto 0) := to_unsigned(C_KX0, 20);   -- H step per h
    signal c_kxp  : unsigned(23 downto 0) := to_unsigned(C_KX0 * 2, 24); -- per clock (x xs)
    signal c_ph0  : unsigned(23 downto 0) := (others => '0');          -- beam start phase
    signal c_ky   : unsigned(23 downto 0) := to_unsigned(C_KY0, 24);   -- V step per virtual line, Q24
    signal c_kyl  : unsigned(23 downto 0) := to_unsigned(C_KY0, 24);   -- V step per line
    signal c_th0  : unsigned(23 downto 0) := to_unsigned(16147791, 24); -- V phase at line 0 (scrolls); power-up: the cell row on line 551, as in the still
    signal c_dth  : unsigned(23 downto 0) := (others => '0');          -- scroll step per field
    signal c_vsq  : unsigned(17 downto 0) := (others => '0');
    signal c_e10  : unsigned(9 downto 0) := to_unsigned(318, 10);      -- eye apex W (Q10)
    signal c_es   : unsigned(11 downto 0) := to_unsigned(1288, 12);    -- eye flank slope per index, Q8
    signal c_tri0 : unsigned(20 downto 0) := (others => '0');          -- eye triangle at index 0, Q8
    signal c_hs   : unsigned(9 downto 0) := to_unsigned(C_HS_R, 10);
    signal c_ra, c_rb : signed(12 downto 0) := (others => '0');        -- tilt a, b (Q10)
    signal c_r0   : signed(29 downto 0) := (others => '0');            -- gain at the beam start, Q26
    signal c_d0   : signed(29 downto 0) := (others => '0');            -- gain step at the start
    signal c_dd   : signed(29 downto 0) := (others => '0');            -- step of the step
    signal c_dfl  : signed(29 downto 0) := (others => '0');            -- twin: step correction at the centre
    signal c_eg   : unsigned(7 downto 0) := (others => '0');           -- etch gain per luma step, Q4
    signal c_b4   : unsigned(9 downto 0) := (others => '0');           -- split, Q10
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);
    signal z_v0q  : std_logic := '0';                                  -- '1' when |v0| = 3/4 (SD)
    signal z_rt   : signed(13 downto 0) := (others => '0');            -- scratch (ra + 2 rb v0)

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
    signal e_th   : unsigned(23 downto 0) := (others => '0');
    signal e_q1, e_q2 : unsigned(1 downto 0) := (others => '0');
    signal e_c1   : signed(12 downto 0) := (others => '0');
    signal e_c2n  : std_logic := '0';
    signal m_a    : unsigned(23 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(23 downto 0) := (others => '0');
    signal n_v    : signed(12 downto 0) := (others => '0');
    signal n_vk   : signed(13 downto 0) := (others => '0');            -- V + 2664 (the index offset 666, x4)
    signal l_v    : signed(13 downto 0) := (others => '0');

    -- beam
    signal b_ph  : unsigned(23 downto 0) := (others => '0');
    signal b_r, b_d : signed(29 downto 0) := (others => '0');
    signal b_x   : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_left : std_logic := '0';

    -- pixel path
    signal p2_r, p3_r, p4_r, p5_r : unsigned(11 downto 0) := (others => '0');
    signal p3_lm, p4_lm, p5_lm, p6_lm : unsigned(7 downto 0) := (others => '0');
    signal p4_wn  : unsigned(9 downto 0) := (others => '0');
    signal p5_km  : unsigned(11 downto 0) := (others => '0');
    signal p6_pa, p6_pb : unsigned(17 downto 0) := (others => '0');
    signal p7_p   : unsigned(23 downto 0) := (others => '0');
    signal p7_ea, p7_eb : unsigned(11 downto 0) := (others => '0');
    signal p8_h   : signed(13 downto 0) := (others => '0');
    signal p8_et  : unsigned(15 downto 0) := (others => '0');
    signal p9_h   : signed(13 downto 0) := (others => '0');
    signal p9_et  : signed(13 downto 0) := (others => '0');
    signal p10_f  : signed(15 downto 0) := (others => '0');
    signal p13_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal p14_y, p14_u, p14_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), ROMs, table
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
            if t_csel = '1' then r_cs_a <= t_csa; else r_cs_a <= e_csa; end if;
            r_ex_q  <= C_FS_EXP2(to_integer(r_ex_a));
            r_cs_q  <= C_FS_COS(to_integer(r_cs_a));
            r_pal_q <= C_FS_PAL(to_integer(r_pal_a));
        end if;
    end process p_rom;

    p_wt : process(clk)
    begin
        if rising_edge(clk) then
            w_rq <= wt(to_integer(w_ra));
            if w_we = '1' then wt(to_integer(w_wa)) <= w_wd; end if;
        end if;
    end process p_wt;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(10 downto 0);
        variable v_r  : signed(13 downto 0);
        variable v_rq : signed(29 downto 0);
        variable v_t  : unsigned(35 downto 0);
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
                s_twin <= registers_in(6)(0);
                s_pink <= registers_in(6)(1);
                s_dia  <= registers_in(6)(2);
                s_wrap <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                s_half <= f_half(s_tid);
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9); z_v0q <= '1';
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9); z_v0q <= '0';
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9); z_v0q <= '0';
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

            -- serial products: slot k loads at zc = 32 + 24 k (decoded at 30 + 24 k, operands pre-muxed at
            -- 31 + 24 k), 16 iterations, result stable from load + 17 (stores at load + 18)
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
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(C_KX0, 36);          z_fb <= z_mt1;                     -- kx
                    when 1  => z_fa <= to_unsigned(C_KY0, 36);          z_fb <= z_mt2;                     -- ky
                    when 2  => z_fa <= resize(c_ky, 36);                z_fb <= resize(s_ys, 16);          -- ky * line step
                    when 3  => z_fa <= resize(s_vm, 36);                z_fb <= resize(s_vm, 16);          -- speed^2
                    when 4  => z_fa <= resize(c_vsq, 36);               z_fb <= to_unsigned(1250, 16);     -- scroll step
                    when 5  => z_fa <= resize(z_a(9 downto 0), 36);     z_fb <= to_unsigned(468, 16);      -- eye slope
                    when 6  => z_fa <= resize(c_es, 36);                z_fb <= to_unsigned(C_UE, 16);     -- slope * 476
                    when 7  => z_fa <= resize(z_a(11 downto 0), 36);    z_fb <= to_unsigned(51, 16);       -- |ra| * 0.2
                    when 8  =>                                                                               -- |d0| = |ra + 2 rb v0| * xs * 546
                        z_fa <= resize(z_a(13 downto 0), 36);
                        case to_integer(s_xs) is
                            when 4      => z_fb <= to_unsigned(2184, 16);
                            when 3      => z_fb <= to_unsigned(1638, 16);
                            when others => z_fb <= to_unsigned(1092, 16);
                        end case;
                    when 9  =>                                                                               -- |dd| = |rb| * xs^2 * 9
                        z_fa <= resize(z_a(11 downto 0), 36);
                        case to_integer(s_xs) is
                            when 4      => z_fb <= to_unsigned(144, 16);
                            when 3      => z_fb <= to_unsigned(81, 16);
                            when others => z_fb <= to_unsigned(36, 16);
                        end case;
                    when 10 =>                                                                               -- |dflip| = |ra| * xs * 1092
                        z_fa <= resize(z_a(11 downto 0), 36);
                        case to_integer(s_xs) is
                            when 4      => z_fb <= to_unsigned(4368, 16);
                            when 3      => z_fb <= to_unsigned(3276, 16);
                            when others => z_fb <= to_unsigned(2184, 16);
                        end case;
                    when 11 => z_fa <= resize(s_g6(12 downto 3), 36);   z_fb <= resize(s_g6(12 downto 3), 16);  -- etch^2
                    when 12 => z_fa <= resize(z_a, 36);                 z_fb <= to_unsigned(192, 16);      -- etch gain
                    when others =>                                                                           -- start phase
                        z_fa <= resize(c_kx, 36);
                        if s_twin = '1' then z_fb <= resize(to_unsigned(1914, 12) - c_xb0, 16);
                        else z_fb <= resize(to_unsigned(C_XC2, 12) - c_xb0, 16); end if;
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
                    when 18 =>                        -- cells exponent: 0.75 (512 - k1), Q8 octaves
                        v_e := 512 - signed(resize(s_g1(12 downto 3), 11));
                        v_e := shift_right(v_e, 1) + shift_right(v_e, 2);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh1 <= v_e(10 downto 8);
                    when 20 =>
                        z_mt1 <= r_ex_q;
                        -- scroll (last field's step: the V phase retreats, so the lattice moves down); beam origin
                        if s_vneg = '1' then c_th0 <= c_th0 + c_dth; else c_th0 <= c_th0 - c_dth; end if;   -- + speed scrolls DOWN (the clip)
                        c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
                    when 22 =>                        -- rows exponent
                        v_e := 512 - signed(resize(s_g2(12 downto 3), 11));
                        v_e := shift_right(v_e, 1) + shift_right(v_e, 2);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh2 <= v_e(10 downto 8);
                    when 24 => z_mt2 <= r_ex_q;
                    when 50 =>                        -- kx = C_KX0 * mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh1) is
                            when -2     => c_kx <= resize(f_acc(35 downto 17), 20);
                            when -1     => c_kx <= f_acc(35 downto 16);
                            when 0      => c_kx <= f_acc(34 downto 15);
                            when others => c_kx <= f_acc(33 downto 14);
                        end case;
                    when 52 =>
                        case to_integer(s_xs) is
                            when 4      => c_kxp <= shift_left(resize(c_kx, 24), 2);
                            when 3      => c_kxp <= shift_left(resize(c_kx, 24), 1) + resize(c_kx, 24);
                            when others => c_kxp <= shift_left(resize(c_kx, 24), 1);
                        end case;
                    when 74 =>                        -- ky
                        case to_integer(z_sh2) is
                            when -2     => c_ky <= resize(f_acc(35 downto 17), 24);
                            when -1     => c_ky <= resize(f_acc(35 downto 16), 24);
                            when 0      => c_ky <= resize(f_acc(35 downto 15), 24);
                            when others => c_ky <= resize(f_acc(35 downto 14), 24);
                        end case;
                    when 98  => c_kyl <= f_acc(31 downto 8);
                    when 122 => c_vsq <= f_acc(17 downto 0);
                    when 146 =>
                        if s_half = '1' then c_dth <= f_acc(32 downto 9); else c_dth <= f_acc(31 downto 8); end if;
                        -- eyes: E10 = 1023 - k3 + k3/64 (the photo at 716: 318); A = 1023 - E10
                        c_e10 <= to_unsigned(1023, 10) - s_g3(12 downto 3) + resize(s_g3(12 downto 9), 10);
                        z_a <= resize(s_g3(12 downto 3) - resize(s_g3(12 downto 9), 10), 20);
                    when 170 => c_es <= f_acc(19 downto 8);
                    when 194 =>
                        c_tri0 <= resize(f_acc(20 downto 0), 21) + shift_left(resize(c_e10, 21), 8);
                        -- tilt: ra = 4 (k5 - 512), Q10 (+-2048)
                        v_r := resize(signed(resize(s_g5(12 downto 3), 11)) - 512, 14);
                        v_r := shift_left(v_r, 2);
                        c_ra <= v_r(12 downto 0);
                        if v_r < 0 then z_a <= resize(unsigned(-v_r), 20); z_neg <= '1';
                        else z_a <= resize(unsigned(v_r), 20); z_neg <= '0'; end if;
                    when 218 =>                       -- rb = 0.2 ra
                        if z_neg = '1' then c_rb <= -signed(resize(f_acc(20 downto 8), 13));
                        else c_rb <= signed(resize(f_acc(20 downto 8), 13)); end if;
                    when 219 =>                       -- ra + 2 rb v0 (v0 = -1 or -3/4; twin: |v0|)
                        if z_v0q = '1' then v_r := resize(c_rb, 14) + shift_right(resize(c_rb, 14), 1);   -- 1.5 rb
                        else v_r := shift_left(resize(c_rb, 14), 1); end if;                                 -- 2 rb
                        if s_twin = '1' then v_r := resize(c_ra, 14) + v_r; else v_r := resize(c_ra, 14) - v_r; end if;
                        z_rt <= v_r;
                        if v_r < 0 then z_a <= resize(unsigned(-v_r), 20); z_neg <= '1';
                        else z_a <= resize(unsigned(v_r), 20); z_neg <= '0'; end if;
                        -- r0 = 1 + ra v0 + rb v0^2 (Q26)
                        if z_v0q = '1' then
                            v_r := resize(c_rb, 14) - shift_right(resize(c_rb, 14), 2) + shift_right(resize(c_rb, 14), 4);  -- 9/16 rb
                            if s_twin = '1' then v_r := v_r + resize(c_ra, 14) - shift_right(resize(c_ra, 14), 2);
                            else v_r := v_r - resize(c_ra, 14) + shift_right(resize(c_ra, 14), 2); end if;
                        else
                            if s_twin = '1' then v_r := resize(c_rb, 14) + resize(c_ra, 14);
                            else v_r := resize(c_rb, 14) - resize(c_ra, 14); end if;
                        end if;
                        v_rq := resize(v_r, 30) + 1024;
                        c_r0 <= shift_left(v_rq, 16);
                    when 242 =>                       -- d0: sign = sign(z_rt), twin negated
                        v_rq := signed(resize(f_acc(31 downto 4), 30));
                        if (z_neg xor s_twin) = '1' then c_d0 <= -v_rq; else c_d0 <= v_rq; end if;
                        if c_rb < 0 then z_a <= resize(unsigned(-c_rb), 20); z_neg <= '1';
                        else z_a <= resize(unsigned(c_rb), 20); z_neg <= '0'; end if;
                    when 266 =>                       -- dd
                        v_rq := signed(resize(f_acc(27 downto 8), 30));
                        if z_neg = '1' then c_dd <= -v_rq; else c_dd <= v_rq; end if;
                        if c_ra < 0 then z_a <= resize(unsigned(-c_ra), 20); z_neg <= '1';
                        else z_a <= resize(unsigned(c_ra), 20); z_neg <= '0'; end if;
                    when 290 =>                       -- dflip (twin): 2 ra * delta
                        v_rq := signed(resize(f_acc(31 downto 4), 30));
                        if z_neg = '1' then c_dfl <= -v_rq; else c_dfl <= v_rq; end if;
                    when 314 => z_a <= f_acc(19 downto 0);
                    when 338 => c_eg <= f_acc(27 downto 20);
                    when 362 =>
                        if s_twin = '1' then c_ph0 <= f_acc(23 downto 0);
                        else c_ph0 <= (not f_acc(23 downto 0)) + 1; end if;
                        if s_dia = '1' then c_hs <= to_unsigned(C_HS_D, 10); else c_hs <= to_unsigned(C_HS_R, 10); end if;
                        c_b4 <= s_gp(12 downto 3);
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
    -- Waveform table fill (vblank zone, zc 400 ..): W(i) = min(hump(|i|), eye(i)), i = 0 .. 1023 with the cell
    -- centre at 0 (|i| folds about 512).  hump: cosine index DDA (ha += hs up to 512, then down) through the
    -- quarter-wave ROM, or a plain ramp (diamond); eye: triangle DDA from c_tri0, slope c_es, apex at 476.
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_a  : unsigned(11 downto 0);
        variable v_q  : unsigned(12 downto 0);
        variable v_h  : unsigned(9 downto 0);
        variable v_tq : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 400 then
                t_run <= '1'; t_i <= (others => '0'); t_ha <= (others => '0'); t_ta <= c_tri0; t_csel <= '1';
            elsif t_run = '1' then
                t_i <= t_i + 1;
                if t_i = 1023 then t_run <= '0'; t_csel <= '0'; end if;
                if t_i < 512 then t_ha <= t_ha + c_hs; else t_ha <= t_ha - c_hs; end if;
                if t_i < C_UE then t_ta <= t_ta - c_es; else t_ta <= t_ta + c_es; end if;
            end if;
            -- stage 1: cosine ROM address from the hump index, triangle value (saturated)
            t_v1 <= t_run; t_i1 <= t_i(9 downto 0);
            v_a := t_ha(19 downto 8);
            t_a1 <= v_a;
            if v_a < 256 then t_csa <= v_a(7 downto 0);
            elsif v_a < 512 then t_csa <= not v_a(7 downto 0);     -- 511 - a
            else t_csa <= (others => '0'); end if;
            if t_ta(20 downto 18) /= "000" then t_t1 <= (others => '1'); else t_t1 <= t_ta(17 downto 8); end if;
            -- stage 2: the ROM address register loads; carry the zone code and the diamond ramp
            t_v2 <= t_v1; t_i2 <= t_i1; t_t2 <= t_t1;
            if t_a1 < 256 then t_z2 <= "00"; elsif t_a1 < 512 then t_z2 <= "01"; else t_z2 <= "10"; end if;
            if t_a1(11 downto 10) /= "00" then t_h2 <= (others => '1');          -- diamond ramp: |u| * 2.265, saturated
            else t_h2 <= t_a1(9 downto 0); end if;
            -- stage 3: ROM data arrives
            t_v3 <= t_v2; t_i3 <= t_i2; t_t3 <= t_t2; t_z3 <= t_z2; t_h3 <= t_h2;
            -- stage 4: hump value
            t_v4 <= t_v3; t_i4 <= t_i3; t_t4 <= t_t3;
            if s_dia = '1' then
                v_h := t_h3;
            else
                case t_z3 is
                    when "00"   => v_q := to_unsigned(2047, 13) - resize(r_cs_q, 13); v_h := v_q(11 downto 2);
                    when "01"   => v_q := to_unsigned(2047, 13) + resize(r_cs_q, 13); v_h := v_q(11 downto 2);
                    when others => v_h := (others => '1');
                end case;
            end if;
            t_h4 <= v_h;
            -- stage 5: write min(hump, eye)
            w_we <= t_v4; w_wa <= t_i4;
            if t_h4 < t_t4 then w_wd <= t_h4; else w_wd <= t_t4; end if;
        end if;
    end process p_fill;

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
    -- Line engine: kicked at each line start (and once in the zone for line 0), computes the NEXT line's
    -- V = cos(th) + split * cos(2 th) from the quarter-wave ROM and one serial product
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_c : signed(12 downto 0);
        variable v_q : unsigned(1 downto 0);
        variable v_p : signed(12 downto 0);
        variable v_u : unsigned(10 downto 0);
        variable v_t : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1500 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_th <= c_th0;
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_th <= e_th + c_kyl;
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(22 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>                      -- cos(th): quadrant fold
                    e_q1 <= e_th(23 downto 22);
                    if e_th(22) = '1' then e_csa <= not e_th(21 downto 14); else e_csa <= e_th(21 downto 14); end if;
                when 2 =>                      -- cos(2 th)
                    e_q2 <= e_th(22 downto 21);
                    if e_th(21) = '1' then e_csa <= not e_th(20 downto 13); else e_csa <= e_th(20 downto 13); end if;
                when 4 =>                      -- Round: cos(th); Diamond: a triangle wave, +2047 at th = 0
                    if s_dia = '1' then
                        v_u := e_th(22 downto 12);
                        v_t := signed(shift_left(resize(v_u, 13), 1)) - 2047;        -- 2u - 2047
                        if e_th(23) = '0' then e_c1 <= -v_t; else e_c1 <= v_t; end if;
                    else
                        v_c := signed(resize(r_cs_q, 13));
                        if e_q1 = "01" or e_q1 = "10" then e_c1 <= -v_c; else e_c1 <= v_c; end if;
                    end if;
                when 5 =>                      -- the octave: |cos 2th| or |tri 2th| and its sign
                    if s_dia = '1' then
                        v_u := e_th(21 downto 11);
                        v_t := signed(shift_left(resize(v_u, 13), 1)) - 2047;
                        if e_th(22) = '0' then v_t := -v_t; end if;
                        if v_t < 0 then
                            e_c2n <= '1'; m_a <= resize(unsigned(-v_t), 24);
                        else
                            e_c2n <= '0'; m_a <= resize(unsigned(v_t), 24);
                        end if;
                    else
                        if e_q2 = "01" or e_q2 = "10" then e_c2n <= '1'; else e_c2n <= '0'; end if;
                        m_a <= resize(r_cs_q, 24);
                    end if;
                    m_b <= resize(c_b4, 16); m_acc <= (others => '0');
                when 22 =>                     -- V = cos/2 +- split * |cos 2th| >> 11 (Q10)
                    v_p := signed(resize(m_acc(21 downto 11), 13));
                    if e_c2n = '1' then n_v <= shift_right(e_c1, 1) - v_p; else n_v <= shift_right(e_c1, 1) + v_p; end if;
                when 23 =>                     -- pre-add the palette offset (666 entries = 2664 in Q10 / 4)
                    n_vk <= resize(n_v, 14) + 2664;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the H phase (+-kxp per clock; twin: backward on the left half) and the gain r by a second-order DDA
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_d : signed(29 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_ph <= c_ph0;
                b_r  <= c_r0;
                b_d  <= c_d0;
                b_x  <= c_xb0;
                b_px <= (others => '0');
                b_left <= s_twin;
                l_v  <= n_vk;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x + s_xs;
                if b_left = '1' then b_ph <= b_ph - c_kxp; else b_ph <= b_ph + c_kxp; end if;
                b_r <= b_r + b_d;
                v_d := b_d + c_dd;
                if b_left = '1' and b_x + s_xs > 1920 then
                    b_left <= '0';
                    v_d := v_d + c_dfl;
                end if;
                b_d <= v_d;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_r  : signed(29 downto 0);
        variable v_e  : signed(13 downto 0);
        variable v_i  : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: table address, gain clamped to [0.45, 4)
            w_ra <= b_ph(23 downto 14);
            v_r := b_r;
            if v_r < to_signed(461 * 65536, 30) then p2_r <= to_unsigned(461, 12);
            elsif v_r(29 downto 28) /= "00" then p2_r <= to_unsigned(4095, 12);
            else p2_r <= unsigned(v_r(27 downto 16)); end if;
            -- 3: table data (EBR); luma from the line buffer
            p3_r <= p2_r; p3_lm <= unsigned(s_lb_q);
            -- 4: fabric copy
            p4_wn <= w_rq; p4_r <= p3_r; p4_lm <= p3_lm;
            -- 5: 2 - W
            p5_km <= to_unsigned(2048, 12) - resize(p4_wn, 12); p5_r <= p4_r; p5_lm <= p4_lm;
            -- 6: partial products r_hi * km, r_lo * km
            p6_pa <= resize(p5_r(11 downto 6) * p5_km, 18);
            p6_pb <= resize(p5_r(5 downto 0) * p5_km, 18);
            p6_lm <= p5_lm;
            -- 7: r (2 - W), Q20; etch partials
            p7_p  <= shift_left(resize(p6_pa, 24), 6) + resize(p6_pb, 24);
            p7_ea <= resize(p6_lm(3 downto 0) * c_eg, 12);
            p7_eb <= resize(p6_lm(7 downto 4) * c_eg, 12);
            -- 8: h = r (2 - W) - 2, Q10; etch sum
            p8_h  <= signed(resize(p7_p(23 downto 10), 14)) - 2048;
            p8_et <= resize(p7_ea, 16) + shift_left(resize(p7_eb, 16), 4);
            -- 9: soft knee above 0 (slopes 1, 1/2, 1/4, 1/8 per knee segment); etch term, Q10
            v_e := p8_h;
            if v_e <= 0 then
                p9_h <= v_e;
            elsif v_e < C_KNEE then
                p9_h <= v_e;
            elsif v_e < 2 * C_KNEE then
                p9_h <= to_signed(C_KNEE, 14) + shift_right(v_e - C_KNEE, 1);
            elsif v_e < 4 * C_KNEE then
                p9_h <= to_signed(C_KNEE + C_KNEE / 2, 14) + shift_right(v_e - 2 * C_KNEE, 2);
            else
                p9_h <= to_signed(2 * C_KNEE, 14) + shift_right(v_e - 4 * C_KNEE, 3);
            end if;
            p9_et <= signed(resize(shift_right(p8_et, 4), 14));      -- luma only adds: black input = the still
            -- 10: F = h + V + etch
            p10_f <= resize(p9_h, 16) + resize(l_v, 16) + resize(p9_et, 16);
            -- 11: palette index = F/4 + 666 (the offset rides in l_v); Rings: fold about the top (entry 929.5, F = 1.03)
            --     so the saturated field reads the palette back down: every black oval becomes a ring cell
            v_i := shift_right(p10_f, 2);
            if s_wrap = '1' and v_i > 929 then v_i := 1859 - v_i; end if;
            if v_i < 0 then
                r_pal_a <= (others => '0');
            elsif v_i > 1023 then
                r_pal_a <= (others => '1');
            else
                r_pal_a <= unsigned(v_i(9 downto 0));
            end if;
            -- 12: ROM data (r_pal_q)
            -- 13: fabric copy
            p13_q <= r_pal_q;
            -- 14: colours (pink: chroma negated)
            p14_y <= unsigned(p13_q(31 downto 22));
            if s_pink = '1' then
                p14_u <= to_unsigned(1024 - 1, 10) - unsigned(p13_q(21 downto 12)) + 1;
                p14_v <= to_unsigned(1024 - 1, 10) - unsigned(p13_q(11 downto 2)) + 1;
            else
                p14_u <= unsigned(p13_q(21 downto 12));
                p14_v <= unsigned(p13_q(11 downto 2));
            end if;
            -- 15: compose; black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= C_BKY; s_u15 <= C_BKU; s_v15 <= C_BKV;
            else
                s_y15 <= p14_y; s_u15 <= p14_u; s_v15 <= p14_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (the HD HDMI bar fix and the
    -- capture's horizontal softness), luma 13/16 and the blanking gate.  The running 8-sums snap back to the
    -- palette black once 8 blank samples are in, so a glitched sample can't offset the colours for good.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- box A: stage 16 sums, stage 17 output
            s_cay(0) <= s_y15; s_cau(0) <= s_u15; s_cav(0) <= s_v15;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= C_BKY & "000"; s_bau <= C_BKU & "000"; s_bav <= C_BKV & "000";
            else
                s_bay <= s_bay + s_y15 - s_cay(7);
                s_bau <= s_bau + s_u15 - s_cau(7);
                s_bav <= s_bav + s_v15 - s_cav(7);
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= C_BKY; s_u17 <= C_BKU; s_v17 <= C_BKV;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            -- box B: stage 18 sums, stage 19 output
            s_cby(0) <= s_y17; s_cbu(0) <= s_u17; s_cbv(0) <= s_v17;
            for i in 1 to 7 loop s_cby(i) <= s_cby(i - 1); s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bby <= C_BKY & "000"; s_bbu <= C_BKU & "000"; s_bbv <= C_BKV & "000";
            else
                s_bby <= s_bby + s_y17 - s_cby(7);
                s_bbu <= s_bbu + s_u17 - s_cbu(7);
                s_bbv <= s_bbv + s_v17 - s_cbv(7);
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y19 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u19 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v19 <= v_sc(13 downto 4);
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

end architecture frogspawn;
