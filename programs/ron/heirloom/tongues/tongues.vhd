-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Tongues v0.1.0 -- Heirloom (video phase), rebuilt from the clip WIN_20230127_01_48_55 at 25.5 s.
--
-- The still: eight columns across the frame (an 8x horizontal sawtooth), a vertical cosine (2.04 cycles
-- per frame), and a WINDOW key on the cosine whose level rises along each column and whose width opens
-- from left to right: thin green lines at the left, red / yellow flame tongues converging on a bright
-- apex in the centre columns, broad green bands on black at the right.  The red is a per-column "flame"
-- hump (bright in the centre columns) riding the same cosine.
--
-- The patch, as fitted (tongues.py has every constant):
--   cos    = cos(2 pi (vy - yc_k - psi_k xl) / P)   P = 530 virtual lines; yc_k / psi_k per column
--   C      = C0_k + Saw * s_k * xl (+ Stairs / drift ramp across the columns, + Etch * luma)
--   w      = Open * (w_k + wr_k xl) + wv (|vy - 530| - 260) / 530 + breath
--   G      = min(8 (cos - C + w), khi (C + w - cos)) clipped       (Fill: the first term only)
--   R      = Blaze * clip(a_k + b_k cos(2 pi (vy - 530) / P))       per column, per line
--   colour = (R, G, 0) "Fire", or the same with its chroma inverted "Ice" (azure flame, magenta bands)
--
--   K1  Cycles    vertical cycles per frame, 0.5 .. 8 (2.04 = the still)
--   K2  Stairs    bipolar ramp of the window level across the columns (0 = the still)
--   K3  Saw       how far the level rises along each column (0 = flat steps, 410 = the still, 2.5x at full)
--   K4  Speed     bipolar: the tilt wobble, the level drift and the width breathing of the clip
--   K5  Open      the key width: hairlines (0) .. the still (512) .. solid bands (1023)
--   K6  Etch      camera (or Osc) luma into the window level: the lines trace the picture
--   S7  Columns   8 / 16
--   S8  Key       Window (the still) / Fill (one-sided: filled tongues)
--   S9  Flip      mirror left <-> right
--   S10 Colours   Fire (the still) / Ice
--   S11 Source    Video / Osc
--   P12 BLAZE     the red flame: none (green on black) .. the still (512) .. the whole frame ablaze
--
-- Engine: vblank-zone sequencer (one serial 18 x 16 multiplier, 58 slots, shared glide, three LFOs from
-- the cosine ROM) writing the per-column tables into two small block RAMs (Flip loads the column END
-- values); a per-line engine (the next line's red for the 4 distinct cosine columns and the vertical width
-- term) reading one of them; a beam with per-column accumulators (phase tilt, level, width) reloaded at
-- every column boundary from values prefetched out of the other, running 12 pixels ahead of the generated
-- avid; a 15-stage pixel path.  Virtual 1080-line coordinates; the 8 columns span the width in every mode.
-- Generated avid; output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes whose 8-back
-- taps come from block-RAM rings, luma 13/16) before the blanking gate.  Colours composed in the hardware
-- convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.tongues_pkg.all;
use work.all;

architecture tongues of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant C_STEP0 : integer := 31690;    -- Q24 phase per virtual line at 2.04 cycles per 1080 lines

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
    -- x step class: 1080 = 2, 720p = 3, SD = 4 (column widths 240 / 160 / 90 source px)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- 2^20 / column width
    function f_invcw(xs : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(xs) is
            when 4      => return to_unsigned(11651, 14);
            when 3      => return to_unsigned(6554, 14);
            when others => return to_unsigned(4370, 14);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    function f_clip8(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else return resize(unsigned(v(7 downto 0)), 8); end if;
    end function;
    -- column of the j-th per-line red product (columns 3 and 7 share 2's and 6's; 5..7 are per field)
    function f_rcol(j : integer) return integer is
    begin
        if j = 3 then return 4; else return j; end if;
    end function;

    type t_u8x4 is array (0 to 3) of unsigned(7 downto 0);

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- per-column tables in block RAM (written in the zone, never read at the written address in the
    -- same clock).  T (beam): word k*8 + {0 off lo, 1 off hi, 2 level, 3 width, 4 level step}.
    -- E (line engine): word j*2 + {0 red a, 1 red b} (Q4) for the 4 per-line red columns.
    type t_ram is array (0 to 255) of std_logic_vector(15 downto 0);
    signal tram : t_ram := (others => x"0000");
    signal eram : t_ram := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of tram : signal is true;
    attribute no_rw_check of eram : signal is true;
    signal t_ra, t_wa, e_ra, e_wa : unsigned(7 downto 0) := (others => '0');
    signal t_wd, e_wd : std_logic_vector(15 downto 0) := x"0000";
    signal t_we, e_we : std_logic := '0';
    signal t_q, e_q : std_logic_vector(15 downto 0) := x"0000";
    signal z_tra : unsigned(7 downto 0) := (others => '0');

    -- output-filter 8-back taps: six 256 x 16 block-RAM rings (Y, U, V of box A and box B), read
    -- pointer = write pointer - 6, then a fabric register
    signal or_ya, or_ua, or_va, or_yb, or_ub, or_vb : t_ram := (others => x"0000");
    attribute no_rw_check of or_ya : signal is true;
    attribute no_rw_check of or_ua : signal is true;
    attribute no_rw_check of or_va : signal is true;
    attribute no_rw_check of or_yb : signal is true;
    attribute no_rw_check of or_ub : signal is true;
    attribute no_rw_check of or_vb : signal is true;
    signal o_wp, o_rp : unsigned(7 downto 0) := (others => '0');
    signal o_qya, o_qua, o_qva, o_qyb, o_qub, o_qvb : std_logic_vector(15 downto 0) := (others => '0');
    signal o_fya, o_fua, o_fva, o_fyb, o_fub, o_fvb : unsigned(9 downto 0) := (others => '0');

    -- ROMs (registered address and data)
    signal r_cs_a : unsigned(9 downto 0) := (others => '0');
    signal r_cs_q : signed(11 downto 0) := (others => '0');
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal z_addr : unsigned(9 downto 0) := (others => '0');
    signal e_addr : unsigned(9 downto 0) := (others => '0');
    signal p2_addr : unsigned(9 downto 0) := (others => '0');
    signal e_ysel : std_logic := '0';

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
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(11 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(410 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(680, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_c16, s_fill, s_flip, s_ice, s_src : std_logic := '0';

    -- per-field constants
    signal c_invcw : unsigned(13 downto 0) := to_unsigned(4370, 14);
    signal c_spd2  : unsigned(17 downto 0) := (others => '0');
    signal c_inct  : signed(15 downto 0) := (others => '0');          -- tilt LFO step (signed by Speed)
    signal c_incb  : signed(15 downto 0) := (others => '0');          -- breath LFO step
    signal c_pht : unsigned(15 downto 0) := x"0000";
    signal c_phd : unsigned(15 downto 0) := x"4000";
    signal c_phb : unsigned(15 downto 0) := x"4000";
    signal c_tiltv, c_driftv, c_breathv : signed(11 downto 0) := (others => '0');
    signal c_tl    : signed(11 downto 0) := (others => '0');          -- tilt, Q6 lines (+-1599)
    signal c_br    : signed(11 downto 0) := (others => '0');          -- breath, Q11
    signal c_lev   : signed(12 downto 0) := (others => '0');          -- stairs + drift, Q11
    signal c_lv7, c_lv5, c_lv3, c_lv1 : signed(15 downto 0) := (others => '0');
    signal z_ei    : signed(2 downto 0) := (others => '0');
    signal c_mant  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal c_step  : unsigned(17 downto 0) := to_unsigned(C_STEP0, 18);
    signal c_stepL : unsigned(23 downto 0) := to_unsigned(C_STEP0, 24);
    signal c_stepc : unsigned(15 downto 0) := (others => '0');
    signal c_incT2, c_incT3 : signed(23 downto 0) := (others => '0');
    signal c_offR  : unsigned(23 downto 0) := (others => '0');
    signal c_r1, c_r2 : unsigned(23 downto 0) := (others => '0');      -- red phase of line 1 / two lines on
    signal c_sawq  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal c_sawc  : unsigned(15 downto 0) := (others => '0');
    signal c_opn   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal c_wrq   : unsigned(15 downto 0) := (others => '0');
    signal c_incWa, c_incWb : signed(12 downto 0) := (others => '0'); -- Q11.4 per px
    signal c_wvq   : unsigned(11 downto 0) := (others => '0');
    signal c_wvs   : unsigned(15 downto 0) := (others => '0');
    signal z_e2    : unsigned(9 downto 0) := (others => '0');
    signal c_etch11 : unsigned(11 downto 0) := (others => '0');
    signal c_etchg4 : unsigned(7 downto 0) := (others => '0');
    signal c_bl8   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal c_flb   : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal c_r5, c_r6 : unsigned(7 downto 0) := (others => '0');      -- per-field red of columns 5, 6 / 7
    signal c_incx  : unsigned(14 downto 0) := to_unsigned(4370, 15);
    signal c_kst   : unsigned(3 downto 0) := (others => '0');          -- first column index of the line

    -- field serial multiplier (18 x 16)
    signal f_a   : unsigned(33 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(33 downto 0) := (others => '0');
    signal z_fa  : unsigned(17 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(5 downto 0) := (others => '0');
    signal z_ctr : unsigned(4 downto 0) := (others => '0');
    signal z_run : std_logic := '0';
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
    signal e_th   : unsigned(23 downto 0) := (others => '0');
    signal e_frn  : signed(11 downto 0) := (others => '0');
    signal e_fa   : unsigned(10 downto 0) := (others => '0');
    signal e_fneg : std_logic := '0';
    signal e_d    : signed(11 downto 0) := (others => '0');
    signal e_da   : unsigned(9 downto 0) := (others => '0');
    signal e_dneg : std_logic := '0';
    signal e_ri, e_rj : unsigned(1 downto 0) := (others => '0');
    signal e_aq   : unsigned(13 downto 0) := (others => '0');           -- red a of the product (fabric copy)
    signal e_t    : signed(19 downto 0) := (others => '0');             -- +- b |cos| >> 11
    signal e_rs   : signed(19 downto 0) := (others => '0');
    signal e_s1, e_s2 : std_logic := '0';
    signal e_pc   : unsigned(4 downto 0) := (others => '0');          -- product phase 0..19
    signal e_run  : std_logic := '0';
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(31 downto 0) := (others => '0');
    signal n_r, l_r : t_u8x4 := (others => (others => '0'));
    signal n_wv, l_wv : signed(11 downto 0) := (others => '0');
    signal y_cnt  : unsigned(3 downto 0) := (others => '0');

    -- beam
    signal b_px   : unsigned(10 downto 0) := (others => '0');
    signal b_cpx  : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal b_xacc : unsigned(19 downto 0) := (others => '0');
    signal b_k    : unsigned(4 downto 0) := (others => '0');
    signal b_tacc : unsigned(23 downto 0) := (others => '0');
    signal b_cacc : signed(19 downto 0) := (others => '0');
    signal b_wacc : signed(19 downto 0) := (others => '0');
    signal b_nk   : unsigned(4 downto 0) := (others => '0');
    signal b_noff : unsigned(23 downto 0) := (others => '0');
    signal b_ncst, b_nwst : signed(15 downto 0) := (others => '0');
    signal b_iT, b_niT : signed(23 downto 0) := (others => '0');
    signal b_iC, b_iW, b_niC, b_niW : signed(12 downto 0) := (others => '0');

    -- pixel path
    signal p2_c, p3_c, p4_c, p5_c : signed(15 downto 0) := (others => '0');
    signal p2_w, p3_w, p4_w : signed(15 downto 0) := (others => '0');
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(11 downto 0) := (others => '0');
    signal p5_f, p6_f, p7_f : signed(11 downto 0) := (others => '0');
    signal p4_wv  : signed(14 downto 0) := (others => '0');
    signal p5_et  : signed(15 downto 0) := (others => '0');
    signal p5_w   : signed(15 downto 0) := (others => '0');
    signal p6_cq, p6_w : signed(15 downto 0) := (others => '0');
    signal p7_lo, p7_hi : signed(16 downto 0) := (others => '0');
    signal p8_d1, p8_d2 : signed(17 downto 0) := (others => '0');
    signal p9_g1, p9_g2 : unsigned(7 downto 0) := (others => '0');
    signal p10_g, p11_g, p11_r : unsigned(7 downto 0) := (others => '0');
    signal p12_rr, p12_gg : unsigned(7 downto 0) := (others => '0');
    signal p13_ya, p13_ua, p13_va : signed(12 downto 0) := (others => '0');
    signal p14_y, p14_u, p14_v : signed(12 downto 0) := (others => '0');
    type t_k is array (0 to 11) of unsigned(2 downto 0);
    signal pk : t_k := (others => (others => '0'));   -- pk(i) = column of the stage-(i+2) pixel
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_KC : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), the table
    -- RAMs, the output-filter rings and the ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_tram : process(clk)
    begin
        if rising_edge(clk) then
            t_q <= tram(to_integer(t_ra));
            if t_we = '1' then tram(to_integer(t_wa)) <= t_wd; end if;
        end if;
    end process p_tram;

    p_eram : process(clk)
    begin
        if rising_edge(clk) then
            e_q <= eram(to_integer(e_ra));
            if e_we = '1' then eram(to_integer(e_wa)) <= e_wd; end if;
        end if;
    end process p_eram;

    p_oring : process(clk)
    begin
        if rising_edge(clk) then
            o_qya <= or_ya(to_integer(o_rp)); or_ya(to_integer(o_wp)) <= std_logic_vector(resize(s_y15, 16));
            o_qua <= or_ua(to_integer(o_rp)); or_ua(to_integer(o_wp)) <= std_logic_vector(resize(s_u15, 16));
            o_qva <= or_va(to_integer(o_rp)); or_va(to_integer(o_wp)) <= std_logic_vector(resize(s_v15, 16));
            o_qyb <= or_yb(to_integer(o_rp)); or_yb(to_integer(o_wp)) <= std_logic_vector(resize(s_y17, 16));
            o_qub <= or_ub(to_integer(o_rp)); or_ub(to_integer(o_wp)) <= std_logic_vector(resize(s_u17, 16));
            o_qvb <= or_vb(to_integer(o_rp)); or_vb(to_integer(o_wp)) <= std_logic_vector(resize(s_v17, 16));
        end if;
    end process p_oring;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            -- one cosine port: the line engine in hblank, the sequencer in the zone, else the pixel path
            if e_ysel = '1' then r_cs_a <= e_addr;
            elsif s_zone = '1' then r_cs_a <= z_addr;
            else r_cs_a <= p2_addr; end if;
            r_cs_q <= C_TG_COS(to_integer(r_cs_a));
            r_ex_q <= C_TG_EXP2(to_integer(r_ex_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s   : signed(10 downto 0);
        variable v_m   : unsigned(9 downto 0);
        variable v_e   : signed(10 downto 0);
        variable v_t18 : signed(17 downto 0);
        variable v_l   : signed(15 downto 0);
        variable v_y   : signed(16 downto 0);
        variable v_x   : integer range 0 to 7;
        variable v_c   : integer range 0 to 7;
        variable v_lm  : signed(15 downto 0);
        variable v_lv  : signed(15 downto 0);
        variable v_rs  : signed(15 downto 0);
        variable v_ic  : signed(15 downto 0);
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
            elsif s_zc /= 4095 then
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
                s_c16  <= registers_in(6)(0);
                s_fill <= registers_in(6)(1);
                s_flip <= registers_in(6)(2);
                s_ice  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                c_invcw <= f_invcw(s_xsr);
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                -- LFO phases advance once per field by the last field's (signed) steps
                c_pht <= unsigned(signed(c_pht) + c_inct);
                c_phd <= unsigned(signed(c_phd) + shift_right(c_inct, 3));
                c_phb <= unsigned(signed(c_phb) + c_incb);
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

            -- serial products on one 18 x 16 multiplier: slot n has z_ctr = 0 at zc = 30 + 24 n; the operands
            -- are pre-muxed at z_ctr = 1, loaded at 2, 16 iterations, the result stored at z_ctr = 20 (the high
            -- half of a phase offset at 19).  A slot counter (no zone-counter decode) steps the slot number.
            -- Slots 8..31 and 46..53 are per-column groups (the column is the slot's low 3 bits).
            if s_zone = '1' and s_zc = 29 then
                z_run <= '1'; z_ctr <= (others => '0'); z_sel <= (others => '0');
            elsif z_run = '1' then
                if z_ctr = 23 then
                    z_ctr <= (others => '0');
                    z_sel <= z_sel + 1;
                    if z_sel = 57 then z_run <= '0'; end if;
                else
                    z_ctr <= z_ctr + 1;
                end if;
            end if;
            if z_run = '1' and z_ctr = 0 then z_pre <= '1'; else z_pre <= '0'; end if;
            if z_pre = '1' then
                v_x := to_integer(z_sel(2 downto 0));
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_g3(12 downto 3), 18);    z_fb <= to_unsigned(160, 16);        -- saw: k3 * 160
                    when 1  => z_fa <= resize(s_g5(12 downto 3), 18);    z_fb <= to_unsigned(230, 16);        -- open: k5 * 230
                    when 2  => z_fa <= resize(s_vm, 18);                 z_fb <= resize(s_vm, 16);            -- speed^2
                    when 3  => z_fa <= c_spd2;                           z_fb <= to_unsigned(2795, 16);       -- tilt LFO step
                    when 4  => z_fa <= to_unsigned(C_STEP0, 18);         z_fb <= c_mant;                      -- step mantissa
                    when 5  => z_fa <= c_step;                           z_fb <= resize(s_ys, 16);            -- step per source line
                    when 6  => z_fa <= c_step;                           z_fb <= resize(c_invcw, 16);         -- step per px per line of tilt
                    when 7  => z_fa <= resize(c_invcw, 18);              z_fb <= resize(c_sawq, 16);          -- saw per px
                    when 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 =>                                               -- column phase offsets
                        z_fa <= c_step;                                  -- (Flip: the column END phase)
                        if s_flip = '1' then v_y := to_signed(C_TG_YCF(v_x) * 64, 17);
                        else v_y := to_signed(C_TG_YC(v_x) * 64, 17); end if;
                        if C_TG_SGN(v_x) < 0 then v_y := v_y - resize(c_tl, 17); else v_y := v_y + resize(c_tl, 17); end if;
                        z_fb <= unsigned(v_y(15 downto 0));
                    when 16 | 17 | 18 | 19 | 20 | 21 | 22 | 23 =>                                             -- level per px
                        z_fa <= to_unsigned(C_TG_S(v_x), 18);            z_fb <= c_sawc;
                    when 24 | 25 | 26 | 27 | 28 | 29 | 30 | 31 =>                                             -- width starts (Flip: ends)
                        if s_flip = '1' then z_fa <= to_unsigned(C_TG_WE(v_x), 18);
                        else z_fa <= to_unsigned(C_TG_W(v_x), 18); end if;
                        z_fb <= resize(c_opn, 16);
                    when 32 | 33 | 34 | 35 =>                                                                 -- red a * blaze
                        z_fa <= to_unsigned(C_TG_A(f_rcol(to_integer(z_sel(1 downto 0)))), 18); z_fb <= resize(c_bl8, 16);
                    when 36 | 37 | 38 | 39 =>                                                                 -- red b * blaze
                        z_fa <= to_unsigned(C_TG_B(f_rcol(to_integer(z_sel(1 downto 0)))), 18); z_fb <= resize(c_bl8, 16);
                    when 40 => z_fa <= to_unsigned(C_TG_A(5), 18);       z_fb <= resize(c_bl8, 16);           -- red of column 5
                    when 41 => z_fa <= to_unsigned(C_TG_A(6), 18);       z_fb <= resize(c_bl8, 16);           -- red of columns 6, 7
                    when 42 => z_fa <= to_unsigned(28, 18);              z_fb <= c_stepc;                     -- tilt per px, column 2 (|psi|)
                    when 43 => z_fa <= to_unsigned(46, 18);              z_fb <= c_stepc;                     -- tilt per px, column 3
                    when 44 => z_fa <= resize(c_invcw, 18);              z_fb <= resize(c_opn, 16);           -- width ramp per px
                    when 45 => z_fa <= resize(s_g6(12 downto 3), 18);    z_fb <= resize(s_g6(12 downto 3), 16); -- etch^2
                    when 46 | 47 | 48 | 49 | 50 | 51 | 52 | 53 =>                                             -- level rise over a column
                        z_fa <= to_unsigned(C_TG_S(v_x), 18);            z_fb <= resize(c_sawq, 16);
                    when 54 => z_fa <= to_unsigned(C_TG_WR(0), 18);      z_fb <= c_wrq;                       -- width per px, a
                    when 55 => z_fa <= to_unsigned(C_TG_WR(2), 18);      z_fb <= c_wrq;                       -- width per px, b
                    when 56 => z_fa <= resize(c_opn, 18);                z_fb <= to_unsigned(164, 16);        -- wv: 0.08 * opn
                    when 57 => z_fa <= resize(c_etch11, 18);             z_fb <= to_unsigned(293, 16);        -- etch per luma unit
                    when others => z_fa <= (others => '0');              z_fb <= (others => '0');
                end case;
            end if;
            z_ld <= z_pre;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 34); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(32 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;
            -- stores
            t_we <= '0'; e_we <= '0';
            v_c := to_integer(z_sel(2 downto 0));
            if z_run = '1' and z_ctr = 19 and z_sel(5 downto 3) = "001" then          -- off hi: word 1
                t_we <= '1'; t_wa <= "00" & z_sel(2 downto 0) & "001"; t_wd <= x"00" & std_logic_vector(f_acc(29 downto 22));
            end if;
            if z_run = '1' and z_ctr = 20 then
                case to_integer(z_sel) is
                    when 0  => c_sawq <= f_acc(17 downto 8);
                    when 1  =>
                        if s_g5(12 downto 3) <= 512 then c_opn <= to_unsigned(26, 10) + f_acc(18 downto 9);
                        else c_opn <= to_unsigned(256, 10) + resize(s_g5(12 downto 3) - 512, 10); end if;
                    when 2  => c_spd2 <= f_acc(17 downto 0);
                    when 3  =>                        -- the tilt LFO step, reversed by Speed's sign
                        v_ic := signed(resize(f_acc(30 downto 16), 16));
                        if s_vneg = '1' then c_inct <= -v_ic; else c_inct <= v_ic; end if;
                    when 4  =>                        -- step = 31690 * mant >> 15 << ei, ei = -2 .. 1
                        case to_integer(z_ei) is
                            when -2     => c_step <= '0' & f_acc(33 downto 17);
                            when -1     => c_step <= f_acc(33 downto 16);
                            when 0      => c_step <= f_acc(32 downto 15);
                            when others => c_step <= f_acc(31 downto 14);
                        end case;
                    when 5  => c_stepL <= f_acc(31 downto 8);
                    when 6  => c_stepc <= "00" & f_acc(33 downto 20);
                    when 7  => c_sawc <= f_acc(23 downto 8);
                    when 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 =>                                               -- off lo: word 0
                        t_we <= '1'; t_wa <= "00" & z_sel(2 downto 0) & "000"; t_wd <= std_logic_vector(f_acc(21 downto 6));
                    when 16 | 17 | 18 | 19 | 20 | 21 | 22 | 23 =>                                             -- level step: word 4
                        t_we <= '1'; t_wa <= "00" & z_sel(2 downto 0) & "100"; t_wd <= std_logic_vector(resize(f_acc(28 downto 16), 16));
                    when 24 | 25 | 26 | 27 | 28 | 29 | 30 | 31 =>                                             -- width start: word 3
                        t_we <= '1'; t_wa <= "00" & z_sel(2 downto 0) & "011"; t_wd <= std_logic_vector(f_acc(23 downto 8));
                    when 32 | 33 | 34 | 35 =>                                                                 -- red a (Q4)
                        e_we <= '1'; e_wa <= "00000" & z_sel(1 downto 0) & '0'; e_wd <= std_logic_vector(resize(f_acc(17 downto 4), 16));
                    when 36 | 37 | 38 | 39 =>                                                                 -- red b (Q4)
                        e_we <= '1'; e_wa <= "00000" & z_sel(1 downto 0) & '1'; e_wd <= std_logic_vector(resize(f_acc(17 downto 4), 16));
                    when 40 =>
                        if f_acc(17 downto 16) /= "00" then c_r5 <= x"FF"; else c_r5 <= f_acc(15 downto 8); end if;
                    when 41 =>
                        if f_acc(17 downto 16) /= "00" then c_r6 <= x"FF"; else c_r6 <= f_acc(15 downto 8); end if;
                    when 42 => c_incT2 <= signed(resize(f_acc(23 downto 0), 24));   -- (negated in the beam)
                    when 43 => c_incT3 <= signed(resize(f_acc(23 downto 0), 24));
                    when 44 => c_wrq <= f_acc(23 downto 8);
                    when 45 => z_e2 <= f_acc(19 downto 10);
                    when 46 | 47 | 48 | 49 | 50 | 51 | 52 | 53 =>                                             -- level start: word 2
                        -- C0 + the ramp weight of (stairs + drift): +7/8, 5/8, 3/8, 1/8, -1/8 .. -7/8 across the
                        -- columns; Flip starts at the column's END (+ its rise)
                        case v_c is
                            when 0 | 7  => v_lm := c_lv7;
                            when 1 | 6  => v_lm := c_lv5;
                            when 2 | 5  => v_lm := c_lv3;
                            when others => v_lm := c_lv1;
                        end case;
                        if z_sel(2) = '1' then v_lv := -v_lm; else v_lv := v_lm; end if;
                        if s_flip = '1' then v_rs := signed(f_acc(23 downto 8)); else v_rs := (others => '0'); end if;
                        t_we <= '1'; t_wa <= "00" & z_sel(2 downto 0) & "010";
                        t_wd <= std_logic_vector(to_signed(C_TG_C0(v_c), 16) + v_lv + v_rs);
                    when 54 => c_incWa <= signed(resize(f_acc(28 downto 16), 13));
                    when 55 => c_incWb <= signed(resize(f_acc(28 downto 16), 13));
                    when 56 => c_wvq <= f_acc(19 downto 8);
                    when 57 => c_etchg4 <= f_acc(19 downto 12);
                    when others => null;
                end case;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- LFO values from the cosine ROM (zc 2..6), knob curves
                    when 2  => z_addr <= c_pht(15 downto 6);
                    when 3  => z_addr <= c_phd(15 downto 6);
                    when 4  => z_addr <= c_phb(15 downto 6);
                    when 5  => c_tiltv <= r_cs_q;
                    when 6  => c_driftv <= r_cs_q;
                    when 7  =>
                        c_breathv <= r_cs_q;
                        -- tilt: 25 lines in Q6 = tiltv * 25 / 32 (18 bits: 25 * 2047 overflows 16)
                        v_t18 := resize(c_tiltv, 18);
                        v_t18 := shift_left(v_t18, 4) + shift_left(v_t18, 3) + v_t18;
                        c_tl <= resize(shift_right(v_t18, 5), 12);
                        -- cycles: e = k1 - 512 in Q8 octaves
                        v_e := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_ei <= v_e(10 downto 8);
                    when 8  =>
                        c_br <= resize(shift_right(c_breathv, 4) + shift_right(c_breathv, 6), 12);
                        -- stairs (K2): (k2 - 512) * 13 / 4, Q11
                        v_e := signed(resize(s_g2(12 downto 3), 11)) - 512;
                        v_l := resize(v_e, 16);
                        v_l := shift_left(v_l, 3) + shift_left(v_l, 2) + v_l;
                        c_lev <= resize(shift_right(v_l, 2), 13) + resize(shift_right(c_driftv, 2), 13);
                        -- blaze (P12): two slopes, Q8
                        if s_gp(12 downto 3) <= 512 then
                            c_bl8 <= resize(s_gp(12 downto 4), 10);
                        else
                            c_bl8 <= to_unsigned(256, 10) + resize(s_gp(12 downto 3) - 512, 10);
                        end if;
                    when 9  =>
                        c_mant <= r_ex_q;
                        c_flb <= resize(shift_right(c_bl8, 2), 8);
                        -- the level ramp's four weights of lev: 7/8, 5/8, 3/8, 1/8
                        c_lv7 <= resize(c_lev, 16) - resize(shift_right(c_lev, 3), 16);
                        c_lv5 <= resize(shift_right(c_lev, 1), 16) + resize(shift_right(c_lev, 3), 16);
                        c_lv3 <= resize(shift_right(c_lev, 2), 16) + resize(shift_right(c_lev, 3), 16);
                        c_lv1 <= resize(shift_right(c_lev, 3), 16);
                    when 10 =>
                        -- columns: 16 doubles the per-px steps
                        if s_c16 = '1' then c_incx <= c_invcw & '0'; else c_incx <= resize(c_invcw, 15); end if;
                        if s_flip = '1' then
                            if s_c16 = '1' then c_kst <= to_unsigned(15, 4); else c_kst <= to_unsigned(7, 4); end if;
                        else
                            c_kst <= (others => '0');
                        end if;
                    when 1140 =>                                                                               -- etch: k6^2 >> 10 x 2.39 -> Q11
                        c_etch11 <= resize(shift_left(resize(z_e2, 12), 1) + shift_right(z_e2, 2) + shift_right(z_e2, 3) + shift_right(z_e2, 6), 12);
                    when 1150 =>                                                                               -- breath LFO step ~0.69 x tilt's
                        c_incb <= shift_right(c_inct, 1) + shift_right(c_inct, 3) + shift_right(c_inct, 4);
                    when 1270 =>                                                                               -- offR = 530 step = (512 + 16 + 2) step
                        c_offR <= resize(shift_left(resize(c_step, 24), 9) + shift_left(resize(c_step, 24), 4) + shift_left(resize(c_step, 24), 1), 24);
                    when 1280 => c_r1 <= c_stepL + not c_offR;                                                 -- theta(1) - offR (theta(n) = n stepL - 1)
                    when 1290 => c_r2 <= shift_left(c_stepL, 1) - c_offR;                                      -- 2 stepL - offR
                    when 1660 => c_wvs <= resize(shift_left(resize(c_wvq, 16), 7) - shift_left(resize(c_wvq, 16), 2), 16);   -- * 124
                    -- the first column's prefetch for the beam (5 reads), done in the zone
                    when 1840 => z_tra <= "00" & c_kst(2 downto 0) & "000";
                    when 1841 => z_tra <= "00" & c_kst(2 downto 0) & "001";
                    when 1842 => z_tra <= "00" & c_kst(2 downto 0) & "010";
                    when 1843 => z_tra <= "00" & c_kst(2 downto 0) & "011";
                    when 1844 => z_tra <= "00" & c_kst(2 downto 0) & "100";
                    -- line-engine cosine reads for lines 0 and 1 (the engine latches them)
                    when 1860 => z_addr <= not c_offR(23 downto 14);                                           -- theta(0) - offR = -1 - offR
                    when 2060 => z_addr <= c_r1(23 downto 14);
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
    -- Line engine: the cosine for the next-but-one line's red is read in
    -- hblank (5 clocks after the generated avid falls); at each line start the
    -- engine (kicked with the cosine read one line earlier) computes the next
    -- line's red for the 4 per-line columns (0, 1, 2 = 3, 4; the table RAM E
    -- read per product) and the vertical width term, five serial products
    -- (20 clocks each), done by clock 100; results are copied in at the
    -- following line start.  Lines 0 and 1 are prepared in the zone.
    -- theta(n) = n stepL - 1 (e_th starts at not stepL).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d  : signed(11 downto 0);
        variable v_t  : signed(19 downto 0);
        variable v_r  : signed(19 downto 0);
        variable v_fl : unsigned(7 downto 0);
        variable v_f  : signed(11 downto 0);
        variable v_i  : integer range 0 to 3;
        variable v_ea : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1880 then e_zk <= '1'; end if;
            -- zone reads: the ROM address is set at 1860 / 2060 by the sequencer, data at +3
            if s_zone = '1' and (s_zc = 1863 or s_zc = 2063) then e_frn <= r_cs_q; end if;
            if s_zone = '1' and s_zc = 1850 then
                e_th <= not c_stepL; e_vy <= (others => '0');
            end if;

            -- hblank read of cos(theta(n + 2) - offR) where theta(n) is the line just finished
            if s_gav_sr(0) = '1' and g_avid = '0' then
                y_cnt <= to_unsigned(1, 4);
            elsif y_cnt /= 0 then
                y_cnt <= y_cnt + 1;
            end if;
            if y_cnt = 4 then v_ea := e_th + c_r2; e_addr <= v_ea(23 downto 14); end if;
            if y_cnt = 5 then e_ysel <= '1'; elsif y_cnt = 6 then e_ysel <= '0'; end if;
            if y_cnt = 8 then e_frn <= r_cs_q; end if;

            if e_zk = '1' then
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_th <= e_th + c_stepL;
                e_lc <= (others => '0');
                l_r <= n_r; l_wv <= n_wv;
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(30 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            -- product phase: e_run from e_lc = 2, e_pc 0..19 per product, e_ri = the product
            if e_lc = 1 then
                e_run <= '1'; e_pc <= (others => '0'); e_ri <= (others => '0');
                v_f := -e_frn;
                if e_frn < 0 then e_fa <= unsigned(v_f(10 downto 0)); e_fneg <= '1';
                else e_fa <= unsigned(e_frn(10 downto 0)); e_fneg <= '0'; end if;
                -- vertical term: |vy - 530| - 260 (virtual lines)
                v_d := signed(resize(e_vy(18 downto 8), 12)) - 530;
                if v_d < 0 then v_d := -v_d; end if;
                e_d <= v_d - 260;
            elsif e_run = '1' then
                if e_pc = 19 then
                    e_pc <= (others => '0');
                    if e_ri = 3 then e_run <= '0'; else e_ri <= e_ri + 1; end if;
                else
                    e_pc <= e_pc + 1;
                end if;
            end if;
            if e_lc = 2 then
                v_d := -e_d;
                if e_d < 0 then e_da <= unsigned(v_d(9 downto 0)); e_dneg <= '1';
                else e_da <= unsigned(e_d(9 downto 0)); e_dneg <= '0'; end if;
            end if;
            -- RAM E reads: red b of product e_ri at phase 0 (data at 2, loaded straight into the multiplier),
            -- red a at phase 5 (data at 7, copied into a fabric register)
            if e_run = '1' and e_pc = 0 then e_ra <= "00000" & e_ri & '1'; end if;
            if e_run = '1' and e_pc = 5 then e_ra <= "00000" & e_ri & '0'; end if;
            if e_run = '1' and e_pc = 7 then e_aq <= unsigned(e_q(13 downto 0)); end if;
            if e_run = '1' and e_pc = 2 then
                m_a <= resize(unsigned(e_q(13 downto 0)), 32); m_b <= resize(e_fa, 16); m_acc <= (others => '0');
            end if;
            -- red = (a4 + sign * (b4 |cos| >> 11)) >> 4, clipped to the floor (columns 0 and 2), in three
            -- clocks: 19 the signed product, +1 the sum, +2 the clip
            e_s1 <= '0';
            if e_run = '1' and e_pc = 19 then
                v_t := signed(resize(m_acc(26 downto 11), 20));
                if e_fneg = '1' then e_t <= -v_t; else e_t <= v_t; end if;
                e_rj <= e_ri; e_s1 <= '1';
                if e_ri = 3 then
                    m_a <= resize(c_wvs, 32); m_b <= resize(e_da, 16); m_acc <= (others => '0');
                end if;
            end if;
            e_s2 <= e_s1;
            if e_s1 = '1' then e_rs <= shift_right(signed(resize(e_aq, 20)) + e_t, 4); end if;
            if e_s2 = '1' then
                v_i := to_integer(e_rj);
                v_r := e_rs;
                if C_TG_FL(f_rcol(v_i)) /= 0 then v_fl := c_flb; else v_fl := (others => '0'); end if;
                if v_r < 0 then n_r(v_i) <= v_fl;
                elsif v_r > 255 then n_r(v_i) <= x"FF";
                elsif unsigned(v_r(7 downto 0)) < v_fl then n_r(v_i) <= v_fl;
                else n_r(v_i) <= unsigned(v_r(7 downto 0)); end if;
            end if;
            -- the width term lands 17 clocks after its load at e_lc = 81
            if e_lc = 100 then
                v_t := signed(resize(m_acc(31 downto 16), 20));
                if e_dneg = '1' then v_t := -v_t; end if;
                n_wv <= resize(v_t, 12);
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: per-pixel column accumulators.  xacc counts the position inside
    -- the column (Q20, its carry = the next column); the phase tilt, level and
    -- width accumulators step by the column's increment and reload at every
    -- column boundary from the next column's values, prefetched out of the
    -- table RAM T at pixels 24..28 of the current column (the zone prefetches
    -- the first column's).  The beam runs 12 pixels AHEAD of the generated avid
    -- (it starts at pixel 12): the group delay of the two output boxes (8) plus
    -- the shell's gate-to-data lag (4), so the picture sits on the avid and the
    -- right edge is not cut; the carry past the last column is ignored.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_x  : unsigned(20 downto 0);
        variable v_k  : unsigned(2 downto 0);
        variable v_iT : signed(23 downto 0);
        variable v_iW : signed(12 downto 0);
        variable v_iC : signed(12 downto 0);
        variable v_tn : std_logic;
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_px   <= to_unsigned(12, 11);
                b_cpx  <= to_unsigned(12, 8);
                b_xacc <= shift_left(resize(c_incx, 20), 3) + shift_left(resize(c_incx, 20), 2);
                b_k    <= resize(c_kst, 5);
                b_tacc <= b_noff;                 -- the first column (0, or 7 under Flip) has no tilt (psi = 0)
                b_cacc <= (b_ncst & "0000") + shift_left(resize(b_niC, 20), 3) + shift_left(resize(b_niC, 20), 2);
                b_wacc <= (b_nwst & "0000") + shift_left(resize(b_niW, 20), 3) + shift_left(resize(b_niW, 20), 2);
                b_iT <= b_niT; b_iC <= b_niC; b_iW <= b_niW;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                v_x := resize(b_xacc, 21) + c_incx;
                b_xacc <= v_x(19 downto 0);
                if v_x(20) = '1' and b_nk(4) = '0' and (s_c16 = '1' or b_nk(3) = '0') then
                    b_k    <= b_nk;
                    b_cpx  <= (others => '0');
                    b_tacc <= b_noff;
                    b_cacc <= b_ncst & "0000";
                    b_wacc <= b_nwst & "0000";
                    b_iT <= b_niT; b_iC <= b_niC; b_iW <= b_niW;
                else
                    if b_cpx /= 255 then b_cpx <= b_cpx + 1; end if;
                    b_tacc <= unsigned(signed(b_tacc) + b_iT);
                    b_cacc <= b_cacc + resize(b_iC, 20);
                    b_wacc <= b_wacc + resize(b_iW, 20);
                end if;
            end if;
            -- the next column and its prefetch (RAM T, 5 words), its increments by column class
            if s_flip = '1' then b_nk <= b_k - 1; else b_nk <= b_k + 1; end if;
            v_k := b_nk(2 downto 0);
            if s_zone = '1' then
                t_ra <= z_tra;
            else
                case to_integer(b_cpx) is
                    when 24 => t_ra <= "00" & v_k & "000";
                    when 25 => t_ra <= "00" & v_k & "001";
                    when 26 => t_ra <= "00" & v_k & "010";
                    when 27 => t_ra <= "00" & v_k & "011";
                    when 28 => t_ra <= "00" & v_k & "100";
                    when others => null;
                end case;
            end if;
            -- the data arrives two clocks after its address (zone: three after z_tra, 1843..1847)
            if b_cpx = 26 or (s_zone = '1' and s_zc = 1843) then b_noff(15 downto 0) <= unsigned(t_q); end if;
            if b_cpx = 27 or (s_zone = '1' and s_zc = 1844) then b_noff(23 downto 16) <= unsigned(t_q(7 downto 0)); end if;
            if b_cpx = 28 or (s_zone = '1' and s_zc = 1845) then b_ncst <= signed(t_q); end if;
            if b_cpx = 29 or (s_zone = '1' and s_zc = 1846) then b_nwst <= signed(t_q); end if;
            if b_cpx = 30 or (s_zone = '1' and s_zc = 1847) then
                v_iC := signed(t_q(12 downto 0));
                if s_c16 = '1' then v_iC := shift_left(v_iC, 1); end if;
                if s_flip = '1' then v_iC := -v_iC; end if;
                b_niC <= v_iC;
            end if;
            if s_zone = '1' then v_k := c_kst(2 downto 0); end if;
            -- tilt: column 2 runs down (psi -28), column 3 up (+46); Flip reverses both
            case to_integer(v_k) is
                when 2      => v_iT := c_incT2; v_tn := not s_flip;
                when 3      => v_iT := c_incT3; v_tn := s_flip;
                when others => v_iT := (others => '0'); v_tn := '0';
            end case;
            if v_k = 2 or v_k = 3 or v_k = 4 then v_iW := c_incWb; else v_iW := c_incWa; end if;
            if s_c16 = '1' then
                v_iT := shift_left(v_iT, 1); v_iW := shift_left(v_iW, 1);
            end if;
            if v_tn = '1' then v_iT := -v_iT; end if;
            if s_flip = '1' then v_iW := -v_iW; end if;
            b_niT <= v_iT; b_niW <= v_iW;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_w  : signed(15 downto 0);
        variable v_13 : signed(21 downto 0);
        variable v_rr, v_gr : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            pk(0) <= b_k(2 downto 0);
            for i in 1 to 11 loop pk(i) <= pk(i - 1); end loop;
            -- 2: cosine address = line phase - column phase - tilt; level and width
            p2_addr <= resize(shift_right(e_th - b_tacc, 14), 10);
            p2_c <= b_cacc(19 downto 4);
            p2_w <= b_wacc(19 downto 4);
            -- 3: ROM read (EBR), luma from the line buffer
            p3_c <= p2_c; p3_w <= p2_w;
            p3_lm <= unsigned(s_lb_q);
            -- 4: etch partials; the vertical width term faded by column pairs: 1, 1/2, 1/4, 1/8
            --    (the cosine ROM's output register is this stage's value; its fabric copy is taken at 5)
            p4_ea <= resize(p3_lm(3 downto 0) * c_etchg4, 12);
            p4_eb <= resize(p3_lm(7 downto 4) * c_etchg4, 12);
            p4_c <= p3_c; p4_w <= p3_w;
            case pk(1)(2 downto 1) is
                when "00"   => p4_wv <= resize(l_wv, 15);
                when "01"   => p4_wv <= resize(shift_right(l_wv, 1), 15);
                when "10"   => p4_wv <= resize(shift_right(l_wv, 2), 15);
                when others => p4_wv <= resize(shift_right(l_wv, 3), 15);
            end case;
            -- 5: etch = gain * luma (Q11.4 -> Q11); width + vertical term + breath (floor 40)
            p5_et <= resize(shift_right(signed(resize(p4_ea, 17)) + shift_left(signed(resize(p4_eb, 17)), 4), 4), 16);
            v_w := p4_w + resize(p4_wv, 16) + resize(c_br, 16);
            if v_w < 40 then p5_w <= to_signed(40, 16); else p5_w <= v_w; end if;
            p5_f <= r_cs_q; p5_c <= p4_c;
            -- 6: level with etch
            p6_cq <= p5_c + p5_et;
            p6_w <= p5_w; p6_f <= p5_f;
            -- 7: window edges
            p7_lo <= resize(p6_cq, 17) - resize(p6_w, 17);
            p7_hi <= resize(p6_cq, 17) + resize(p6_w, 17);
            p7_f <= p6_f;
            -- 8: distances into the window
            p8_d1 <= resize(p7_f, 18) - resize(p7_lo, 18);
            p8_d2 <= resize(p7_hi, 18) - resize(p7_f, 18);
            -- 9: the two edges: sharp (1 per Q11 unit), soft (13/32, column 3: 13/64)
            p9_g1 <= f_clip8(p8_d1);
            v_13 := resize(p8_d2, 22);
            v_13 := shift_left(v_13, 3) + shift_left(v_13, 2) + v_13;
            if pk(6) = 3 then p9_g2 <= f_clip8(shift_right(v_13, 6));
            else p9_g2 <= f_clip8(shift_right(v_13, 5)); end if;
            -- 10: the key
            if s_fill = '1' or p9_g1 < p9_g2 then p10_g <= p9_g1; else p10_g <= p9_g2; end if;
            -- 11: the centre columns' yellow is 13/16 green; the line's red for this column
            if pk(8) = 2 or pk(8) = 3 then
                p11_g <= p10_g - shift_right(p10_g, 3) - shift_right(p10_g, 4);
            else
                p11_g <= p10_g;
            end if;
            case to_integer(pk(8)) is
                when 0      => p11_r <= l_r(0);
                when 1      => p11_r <= l_r(1);
                when 2 | 3  => p11_r <= l_r(2);
                when 4      => p11_r <= l_r(3);
                when 5      => p11_r <= c_r5;
                when others => p11_r <= c_r6;
            end case;
            -- 12: copy
            p12_rr <= p11_r; p12_gg <= p11_g;
            -- 13: RGB (B = 0) -> hardware YUV partial sums (luma pre-scaled 16/13):
            --     Y' = 1.25 R + 2.5 G (+64); Cr = 1.75 R - 1.5 G; Cb = -0.5625 R - 1.125 G
            v_rr := signed(resize(p12_rr, 13)); v_gr := signed(resize(p12_gg, 13));
            p13_ya <= (v_rr + shift_right(v_rr, 2)) + (shift_left(v_gr, 1) + shift_right(v_gr, 1));
            p13_ua <= (v_rr + shift_right(v_rr, 1) + shift_right(v_rr, 2)) - (v_gr + shift_right(v_gr, 1));
            p13_va <= -(shift_right(v_rr, 1) + shift_right(v_rr, 4)) - (v_gr + shift_right(v_gr, 3));
            -- 14: sums; Ice inverts the chroma (azure flame, blue apex, magenta bands)
            p14_y <= to_signed(64, 13) + p13_ya;
            if s_ice = '1' then
                p14_u <= to_signed(512, 13) - p13_ua;
                p14_v <= to_signed(512, 13) - p13_va;
            else
                p14_u <= to_signed(512, 13) + p13_ua;
                p14_v <= to_signed(512, 13) + p13_va;
            end if;
            -- 15: clip and compose; black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= C_KY; s_u15 <= C_KC; s_v15 <= C_KC;
            else
                if p14_y > 940 then s_y15 <= to_unsigned(940, 10); elsif p14_y < 64 then s_y15 <= C_KY; else s_y15 <= unsigned(p14_y(9 downto 0)); end if;
                if p14_u > 1000 then s_u15 <= to_unsigned(1000, 10); elsif p14_u < 24 then s_u15 <= to_unsigned(24, 10); else s_u15 <= unsigned(p14_u(9 downto 0)); end if;
                if p14_v > 1000 then s_v15 <= to_unsigned(1000, 10); elsif p14_v < 24 then s_v15 <= to_unsigned(24, 10); else s_v15 <= unsigned(p14_v(9 downto 0)); end if;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the still's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16 and the blanking gate.  The 8-back samples come
    -- from the block-RAM rings (p_oring), read 6 behind the write pointer plus
    -- a fabric register.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            o_wp <= o_wp + 1;
            o_rp <= o_wp - 5;
            o_fya <= unsigned(o_qya(9 downto 0)); o_fua <= unsigned(o_qua(9 downto 0)); o_fva <= unsigned(o_qva(9 downto 0));
            o_fyb <= unsigned(o_qyb(9 downto 0)); o_fub <= unsigned(o_qub(9 downto 0)); o_fvb <= unsigned(o_qvb(9 downto 0));
            -- The running 8-sums snap back to black (the off-avid input of both boxes) once 8 blank
            -- samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 16 sums, stage 17 output
            if s_bah = "00000000" then
                s_bay <= C_KY & "000"; s_bau <= C_KC & "000"; s_bav <= C_KC & "000";
            else
                s_bay <= s_bay + s_y15 - o_fya;
                s_bau <= s_bau + s_u15 - o_fua;
                s_bav <= s_bav + s_v15 - o_fva;
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= C_KY; s_u17 <= C_KC; s_v17 <= C_KC;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            -- box B: stage 18 sums, stage 19 output
            if s_bbh = "00000000" then
                s_bby <= C_KY & "000"; s_bbu <= C_KC & "000"; s_bbv <= C_KC & "000";
            else
                s_bby <= s_bby + s_y17 - o_fyb;
                s_bbu <= s_bbu + s_u17 - o_fub;
                s_bbv <= s_bbv + s_v17 - o_fvb;
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

end architecture tongues;
