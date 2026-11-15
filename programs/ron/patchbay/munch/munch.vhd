-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Munch v0.1.1 -- Patchbay: a digital-logic patch (munching squares through a 3-channel DAC).
--
-- The patch:
--   H ramp, V ramp   -> (Mirror: both folded about the centre) -> two 8-bit ADCs clocked by the divided
--                       pixel clock (the cell): one code = 3.75 virtual px, a cell holds its code
--   ADC words        -> Code (binary / Gray) -> Skew (V word added to the H word) -> LOGIC (XOR, AND, OR, ADD,
--                       Lattice (A+B)^(A-B), Carry, Mul, Rings A^2+B^2) -> the 8-bit word w
--   frame counter    -> the threshold T (Speed) and a slow drift of both ramps
--   w, T             -> three window comparators (R, G, B): lit when (w - T_c) mod 256 < Window; MUNCH spreads the
--                       three thresholds apart and opens a second window per channel half a range away
--   w bit planes     -> (Bits: rotate) -> three 2-bit DACs, each gated by its comparator; the G and B DACs settle
--                       late (colour fringes at every edge) -> colouriser ROM (Colour: 32 hue mappings)
--   comparators      -> keyer: where no window is open the camera luma shows (Source), or the dimmed quilt (MUNCH 0)
--   camera luma      -> held per cell -> narrows the windows (bright picture = more picture, less quilt)
--
--   K1  Cell     the ADC clock divider: 7.5 .. 120 virtual px (detents at the octaves = clean quilts)
--   K2  Logic    XOR / AND / OR / ADD / Lattice / Carry / Mul / Rings
--   K3  Window   comparator width
--   K4  Speed    bipolar: munch rate and direction, ramp drift
--   K5  Colour   the DAC channel colours, turned through 270 degrees of hue
--   K6  Camera   camera strength: the keyed-through picture fades in to full level at 25 %, and the per-cell
--                luma narrows the windows by g x the v0.1.0 amount, g = K6 / 256 to x 2 at 50 %, then up to x 3
--                at 100 % (the picture takes the bright cells).  Bits (the old K5) is fixed at "7-2".
--   S7  Mirror   On (4-way fold) / Off
--   S8  Code     Binary / Gray
--   S9  Skew     Off / On (diagonals)
--   S10 Planes   Filled / Outline (bit edges only)
--   S11 Source   Video / Osc
--   P12 MUNCH    0 = the whole quilt, one window passing over it .. black keyed munching squares ..
--                100 % = six offset thresholds munching in three colours
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line vertical DDA that runs one
-- line ahead, a beam DDA (cell phase + code accumulator, reflecting at the fold), a 25-stage pixel path.
-- Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (chroma
-- [1 2 2 2 2 2 2 2 1]/16, luma [1 3 4 4 3 1]/16 with a little peaking, luma 13/16) before the blanking gate.
-- Colours in the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K5 = Colour (was K6); K5 Bits fixed at 448 (C_KFIX_5).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.munch_pkg.all;
use work.all;

architecture munch of program_top is
    constant C_KFIX_5 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(448, 10));   -- K6 camera pass: fixed knob
    constant C_LAT     : integer := 24;      -- = processing_delay_clks
    constant C_NOGLIDE : boolean := false;   -- sim only: knobs jump

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
    function f_gray(v : unsigned(7 downto 0)) return unsigned is
    begin
        return v xor ('0' & v(7 downto 1));
    end function;

    attribute no_rw_check : boolean;

    -- source luma line buffer: (luma >> 2) - 16, clipped at 0 (written on the source avid, read late in the pipe)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    attribute no_rw_check of lb : signal is true;
    signal s_lb_wa, l_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal l_q     : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- code line buffer: the DAC code of the last line of the previous cell row (Outline's horizontal edges)
    signal clb : t_lb := (others => x"00");
    attribute no_rw_check of clb : signal is true;
    signal c_ra, c_wa : unsigned(10 downto 0) := (others => '0');
    signal c_wd, c_q, c_qf : std_logic_vector(7 downto 0) := x"00";
    signal k_we : std_logic := '0';

    -- per-cell luma hold, 512 cell columns x 4 banks: two by cell-row parity, and two by field parity for the
    -- top row (its first line reads the samples the last field took there)
    type t_hold is array (0 to 2047) of std_logic_vector(3 downto 0);
    signal hold : t_hold := (others => x"0");
    attribute no_rw_check of hold : signal is true;
    signal h_ra, h_wa : unsigned(10 downto 0) := (others => '0');
    signal h_wd, h_q : std_logic_vector(3 downto 0) := x"0";
    signal h_we : std_logic := '0';

    -- ROMs
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_pa   : unsigned(10 downto 0) := (others => '0');
    signal r_pq   : std_logic_vector(15 downto 0) := (others => '0');

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
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, indices, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(520 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(720, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_fn  : unsigned(2 downto 0) := "000";
    signal s_rot : unsigned(2 downto 0) := "000";
    signal s_pal : unsigned(4 downto 0) := to_unsigned(10, 5);
    signal s_mir : std_logic := '1';
    signal s_gray, s_skew, s_outl, s_src : std_logic := '0';
    signal s_mul, s_rings : std_logic := '0';

    -- mode constants (latched per field)
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);    -- beam start x (h)
    signal c_nf2  : unsigned(10 downto 0) := to_unsigned(950, 11);   -- fold pixel index - 2
    signal c_nfv1 : unsigned(10 downto 0) := to_unsigned(539, 11);   -- fold line index - 1
    signal c_ryk  : unsigned(9 downto 0) := (others => '0');         -- fold line's overshoot (Q8 lines): NTSC 596
    signal c_tz3  : std_logic := '1';                                 -- outline zone: 3 lines (1080p) / 2

    -- per-field constants
    signal z_oct  : unsigned(2 downto 0) := "011";
    signal z_sh   : unsigned(2 downto 0) := "011";
    signal z_f    : unsigned(7 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_mb   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_a    : unsigned(19 downto 0) := (others => '0');
    signal z_ig   : unsigned(8 downto 0) := (others => '0');
    signal z_cx, z_cy : std_logic := '0';
    signal c_step : unsigned(13 downto 0) := to_unsigned(2048, 14);   -- ADC codes per cell, Q8
    signal c_inv  : unsigned(16 downto 0) := to_unsigned(17476, 17);  -- cells per h, Q20
    signal c_dpx, c_dpxn : unsigned(20 downto 0) := (others => '0');  -- cells per pixel (Q20) and its negative
    signal c_dq, c_dqn   : unsigned(20 downto 0) := (others => '0');  -- cells per line
    signal c_dt   : unsigned(18 downto 0) := to_unsigned(41250, 19);  -- threshold step per field, Q16
    signal c_dsx  : unsigned(18 downto 0) := to_unsigned(10999, 19);  -- ramp drift per field, cells Q20
    signal c_dsy  : unsigned(18 downto 0) := to_unsigned(4123, 19);
    signal c_t    : unsigned(23 downto 0) := to_unsigned(166 * 65536, 24);  -- the threshold T, Q8.16
    signal c_sxf  : unsigned(19 downto 0) := to_unsigned(492000, 20);     -- drift, cell fraction
    signal c_syf  : unsigned(19 downto 0) := to_unsigned(1017000, 20);
    signal c_syfe : unsigned(19 downto 0) := (others => '0');             -- V drift fraction in use (0 under Mirror)
    signal c_scx  : unsigned(15 downto 0) := to_unsigned(4096, 16);       -- drift, whole cells in codes (Q8.8)
    signal c_scy  : unsigned(15 downto 0) := (others => '0');
    signal c_fx0, c_fy0 : unsigned(19 downto 0) := (others => '0');
    signal c_xc0, c_yc0 : unsigned(15 downto 0) := (others => '0');
    signal c_ry   : unsigned(19 downto 0) := (others => '0');
    signal c_wid  : unsigned(7 downto 0) := to_unsigned(136, 8);
    signal c_fct  : unsigned(10 downto 0) := to_unsigned(744, 11);
    signal c_we   : unsigned(7 downto 0) := to_unsigned(98, 8);
    signal c_we2  : unsigned(7 downto 0) := (others => '0');
    signal c_w2q  : unsigned(7 downto 0) := (others => '0');
    signal c_o1, c_o2 : unsigned(7 downto 0) := (others => '0');
    signal c_tr, c_tg, c_tb : unsigned(7 downto 0) := (others => '0');
    signal c_g32  : unsigned(4 downto 0) := (others => '0');
    signal c_cg32 : unsigned(5 downto 0) := to_unsigned(32, 6);
    signal s_gc   : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);   -- K6 Camera, glided (Q3)
    signal c_gb5  : unsigned(5 downto 0) := to_unsigned(32, 6);   -- camera picture level, 32 = full (K6 >= 25 %)
    signal c_nw   : unsigned(5 downto 0) := to_unsigned(24, 6);   -- window bite per held-luma step (v0.1.0: 12, Osc 16)
    signal z_cgq  : unsigned(5 downto 0) := to_unsigned(32, 6);
    signal z_cgp  : unsigned(11 downto 0) := (others => '0');
    signal z_nwq  : unsigned(12 downto 0) := (others => '0');
    signal l_ph1  : unsigned(15 downto 0) := x"3000";
    signal l_ph2  : unsigned(15 downto 0) := x"B000";

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(1 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine: the vertical DDA (state = the line after next), A = the next line
    signal e_zk   : std_logic := '0';
    signal e_st   : unsigned(1 downto 0) := "00";
    signal e_fy   : unsigned(19 downto 0) := (others => '0');
    signal e_yc   : unsigned(15 downto 0) := (others => '0');
    signal e_up, e_rc, e_dn, e_fl : std_logic := '0';
    signal e_ln   : unsigned(10 downto 0) := (others => '0');
    signal a_yc   : unsigned(15 downto 0) := (others => '0');
    signal a_rc   : std_logic := '0';
    signal a_yg   : unsigned(7 downto 0) := (others => '0');
    signal a_b2   : unsigned(15 downto 0) := (others => '0');
    -- the current line
    signal l_yg   : unsigned(7 downto 0) := (others => '0');
    signal l_b2   : unsigned(10 downto 0) := (others => '0');
    signal l_rbank, l_last, l_wr, l_first, l_zone, l_seen, l_fpar, l_row0 : std_logic := '0';
    signal l_rb2, l_wb2 : std_logic_vector(1 downto 0) := "00";
    signal l_since : unsigned(1 downto 0) := "00";

    -- beam
    signal b_fr   : unsigned(19 downto 0) := (others => '0');
    signal b_n    : unsigned(10 downto 0) := (others => '0');
    signal b_up, b_fold, b_ev, b_evf, b_evup : std_logic := '0';
    signal x_c    : unsigned(15 downto 0) := (others => '0');
    signal x_col  : unsigned(8 downto 0) := (others => '0');
    signal e_sr   : std_logic_vector(15 downto 0) := (others => '0');

    -- pixel path
    signal p3_xg, p4_a, p5_a : unsigned(7 downto 0) := (others => '0');
    signal p5_xor, p5_and, p5_or, p5_dif : unsigned(7 downto 0) := (others => '0');
    signal p5_sum : unsigned(8 downto 0) := (others => '0');
    signal p5_pl, p5_ph : unsigned(11 downto 0) := (others => '0');
    signal p5_lq, p6_lq : unsigned(2 downto 0) := (others => '0');
    signal p6_p, p7_pr : unsigned(10 downto 0) := (others => '0');
    signal p6_xor, p6_and, p6_or, p6_sum, p6_lat, p6_car, p7_w0 : unsigned(7 downto 0) := (others => '0');
    signal p6_lw : unsigned(8 downto 0) := (others => '0');
    signal p7_wl, p7_wl2, p8_wl, p8_wl2 : unsigned(7 downto 0) := (others => '0');
    signal p8_dr, p8_dg, p8_db : unsigned(7 downto 0) := (others => '0');
    signal p8_lv, p9_lv : std_logic_vector(5 downto 0) := (others => '0');
    signal p9_lit : std_logic_vector(2 downto 0) := (others => '0');
    signal p10_r, p11_r, p11_g, p11_b : std_logic_vector(2 downto 0) := (others => '0');   -- lit & level
    type t_sr3 is array (natural range <>) of std_logic_vector(2 downto 0);
    signal g_sr : t_sr3(0 to 8) := (others => "000");
    signal b_sr : t_sr3(0 to 16) := (others => "000");
    signal p12_code, p13_code : std_logic_vector(5 downto 0) := (others => '0');
    signal p12_bg, p13_bg, p14_bg, p15_bg, p16_bg : std_logic := '1';
    type t_sr6 is array (natural range <>) of std_logic_vector(5 downto 0);
    signal k_sr : t_sr6(0 to 7) := (others => "000000");
    signal n9, n10, n11, n12, n13 : unsigned(10 downto 0) := (others => '0');
    signal p16_w  : std_logic_vector(15 downto 0) := (others => '0');
    signal p16_cam : unsigned(7 downto 0) := (others => '0');
    signal p17_yq : unsigned(11 downto 0) := (others => '0');
    signal p17_us, p17_vs : signed(12 downto 0) := (others => '0');
    signal p17_yc : unsigned(13 downto 0) := (others => '0');
    signal p18_ya, p18_yb : unsigned(11 downto 0) := (others => '0');
    signal p18_u, p18_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal q_acc  : unsigned(11 downto 0) := (others => '0');
    signal q_col  : unsigned(8 downto 0) := (others => '0');
    signal s_y19 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cy : t_u10(0 to 7) := (others => to_unsigned(64, 10));
    signal s_cu, s_cv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_by, s_byd : unsigned(12 downto 0) := to_unsigned(512, 13);
    signal s_bu, s_bv, s_bud, s_bvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_lp, s_lp1, s_lp2 : unsigned(10 downto 0) := to_unsigned(128, 11);
    signal s_lq, s_lq1, s_lq2 : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_lr, s_lrd : unsigned(13 downto 0) := to_unsigned(1024, 14);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21, s_u22, s_v22, s_u23, s_v23, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_ys22 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_yd22 : signed(10 downto 0) := (others => '0');
    signal s_y23, s_y24, s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Block RAMs and ROMs (one write statement each, registered read)
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            l_q <= lb(to_integer(l_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_clb : process(clk)
    begin
        if rising_edge(clk) then
            c_q <= clb(to_integer(c_ra));
            if k_we = '1' then clb(to_integer(c_wa)) <= c_wd; end if;
        end if;
    end process p_clb;

    p_hold : process(clk)
    begin
        if rising_edge(clk) then
            h_q <= hold(to_integer(h_ra));
            if h_we = '1' then hold(to_integer(h_wa)) <= h_wd; end if;
        end if;
    end process p_hold;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_MU_EXP2(to_integer(r_ex_a));
            r_pq   <= C_MU_PAL(to_integer(r_pa));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : unsigned(10 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_lo, v_hi : unsigned(10 downto 0);
        variable v_c  : unsigned(20 downto 0);
        variable v_x  : unsigned(11 downto 0);
        variable v_t  : unsigned(4 downto 0);
        variable v_p  : unsigned(9 downto 0);
        variable v_i  : unsigned(2 downto 0);
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
            -- Speed: bipolar, +-8 deadband
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
                s_mir  <= not registers_in(6)(0);
                s_gray <= registers_in(6)(1);
                s_skew <= registers_in(6)(2);
                s_outl <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => c_xb0 <= to_unsigned(512, 12); s_oxs <= to_unsigned(204, 9); c_nf2 <= to_unsigned(350, 11);
                    when 3      => c_xb0 <= to_unsigned(24, 12);  s_oxs <= to_unsigned(153, 9); c_nf2 <= to_unsigned(630, 11);
                    when others => c_xb0 <= to_unsigned(16, 12);  s_oxs <= to_unsigned(102, 9); c_nf2 <= to_unsigned(950, 11);
                end case;
                c_ryk <= (others => '0'); c_tz3 <= '0';
                case to_integer(s_ys) is
                    when 1138   => c_nfv1 <= to_unsigned(121, 11); c_ryk <= to_unsigned(596, 10);
                    when 576    => c_nfv1 <= to_unsigned(239, 11);
                    when 960    => c_nfv1 <= to_unsigned(143, 11);
                    when 480    => c_nfv1 <= to_unsigned(287, 11);
                    when 384    => c_nfv1 <= to_unsigned(359, 11);
                    when 512    => c_nfv1 <= to_unsigned(269, 11);
                    when others => c_nfv1 <= to_unsigned(539, 11); c_tz3 <= '1';
                end case;
                l_ph1 <= l_ph1 + 34;
                l_ph2 <= l_ph2 + 24;
                -- stepped knobs: move only when the knob sits clear of a step boundary.  A labelled knob's
                -- register holds the label INDEX (0..7) right after the program loads, so values below 8 are taken
                -- as the index itself.
                v_k := unsigned(registers_in(1));
                if v_k < 12 then v_lo := (others => '0'); else v_lo := resize(v_k, 11) - 12; end if;
                v_hi := resize(v_k, 11) + 12; if v_hi > 1023 then v_hi := to_unsigned(1023, 11); end if;
                v_i := v_k(9 downto 7);
                if v_k < 8 then v_i := v_k(2 downto 0); end if;
                if v_k < 8 or v_lo(9 downto 7) = v_hi(9 downto 7) then
                    s_fn <= v_i;
                    if v_i(2 downto 1) = "11" then s_mul <= '1'; else s_mul <= '0'; end if;
                    if v_i = "111" then s_rings <= '1'; else s_rings <= '0'; end if;
                end if;
                v_k := unsigned(C_KFIX_5);
                if v_k < 12 then v_lo := (others => '0'); else v_lo := resize(v_k, 11) - 12; end if;
                v_hi := resize(v_k, 11) + 12; if v_hi > 1023 then v_hi := to_unsigned(1023, 11); end if;
                v_i := v_k(9 downto 7);
                if v_k < 8 then v_i := v_k(2 downto 0); end if;
                if v_k < 8 or v_lo(9 downto 7) = v_hi(9 downto 7) then s_rot <= v_i - 3; end if;
                v_k := unsigned(registers_in(4));
                if v_k < 4 then v_lo := (others => '0'); else v_lo := resize(v_k, 11) - 4; end if;
                v_hi := resize(v_k, 11) + 4; if v_hi > 1023 then v_hi := to_unsigned(1023, 11); end if;
                if v_lo(9 downto 5) = v_hi(9 downto 5) then s_pal <= v_k(9 downto 5); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "00"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 2      => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                            when others => s_gm <= s_gc; s_gr <= unsigned(registers_in(5));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if C_NOGLIDE then
                            null;
                        elsif (z_gc /= 3 and (s_gd > 16 or s_gd < -16)) or s_gd > 32 or s_gd < -32 then   -- Camera: +-4
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gc <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 3 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from
            -- load + 17 (stores at 50 + 24 k).  The slot number is decoded at 30 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 31 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(12, 4); z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(13, 4); z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(14, 4); z_pre <= '1';
                    when 390 => z_sel <= to_unsigned(15, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(139810, 36);        z_fb <= z_mb;                          -- 1 / cell
                    when 1  => z_fa <= resize(c_inv, 36);              z_fb <= resize(s_ys, 16);              -- cells per line
                    when 2  => z_fa <= resize(s_vm, 36);               z_fb <= resize(s_vm, 16);              -- speed^2
                    when 3  => z_fa <= resize(c_inv, 36);              z_fb <= c_dt(18 downto 3);             -- drift
                    when 4  =>                                                                                -- H start phase
                        z_fa <= resize(c_inv, 36);
                        if s_mir = '1' then z_fb <= resize(to_unsigned(1920, 12) - c_xb0, 16); else z_fb <= resize(c_xb0, 16); end if;
                    when 5  => z_fa <= resize(c_step, 36);             z_fb <= resize(z_ig, 16);              -- H start code
                    when 6  =>                                                                                -- V start phase
                        z_fa <= resize(c_inv, 36);
                        if s_mir = '1' then z_fb <= to_unsigned(1080, 16); else z_fb <= (others => '0'); end if;
                    when 7  => z_fa <= resize(c_step, 36);             z_fb <= resize(z_ig, 16);              -- V start code
                    when 8  => z_fa <= resize(c_inv, 36);              z_fb <= resize(c_ryk, 16);             -- fold line overshoot
                    when 9  => z_fa <= resize(s_g3(12 downto 3), 36);  z_fb <= to_unsigned(230, 16);          -- window
                    when 10 => z_fa <= resize(s_gp(12 downto 3), 36);  z_fb <= to_unsigned(717, 16);          -- MUNCH narrows it
                    when 11 => z_fa <= resize(c_wid, 36);              z_fb <= resize(c_fct, 16);
                    when 12 => z_fa <= resize(c_we, 36);               z_fb <= resize(c_w2q, 16);             -- second window
                    when 13 =>                                                                                -- G threshold offset
                        z_fa <= resize(s_gp(12 downto 3), 36);
                        if l_ph1(15) = '1' then v_t := not l_ph1(14 downto 10); else v_t := l_ph1(14 downto 10); end if;
                        z_fb <= resize(v_t, 16) + 69;
                    when 14 =>                                                                                -- B threshold offset
                        z_fa <= resize(s_gp(12 downto 3), 36);
                        if l_ph2(15) = '1' then v_t := not l_ph2(14 downto 10); else v_t := l_ph2(14 downto 10); end if;
                        z_fb <= resize(v_t, 16) + 154;
                    when others =>                                                                            -- keyed-out quilt level
                        if s_gp(12 downto 3) < 358 then z_fa <= resize(to_unsigned(358, 10) - s_gp(12 downto 3), 36);
                        else z_fa <= (others => '0'); end if;
                        z_fb <= to_unsigned(52, 16);
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
                    -- motion (last field's steps): threshold, ramp drift
                    when 20 =>
                        if s_vneg = '1' then
                            c_t <= c_t - resize(c_dt, 24);
                            v_c := ('0' & c_sxf) - resize(c_dsx, 21); c_sxf <= v_c(19 downto 0); z_cx <= v_c(20);
                            v_c := ('0' & c_syf) - resize(c_dsy, 21); c_syf <= v_c(19 downto 0); z_cy <= v_c(20);
                        else
                            c_t <= c_t + resize(c_dt, 24);
                            v_c := ('0' & c_sxf) + resize(c_dsx, 21); c_sxf <= v_c(19 downto 0); z_cx <= v_c(20);
                            v_c := ('0' & c_syf) + resize(c_dsy, 21); c_syf <= v_c(19 downto 0); z_cy <= v_c(20);
                        end if;
                    when 21 =>
                        if z_cx = '1' then
                            if s_vneg = '1' then c_scx <= c_scx - resize(c_step, 16); else c_scx <= c_scx + resize(c_step, 16); end if;
                        end if;
                        if z_cy = '1' then
                            if s_vneg = '1' then c_scy <= c_scy - resize(c_step, 16); else c_scy <= c_scy + resize(c_step, 16); end if;
                        end if;
                    -- cell: e = k1 + 256 (Q8 octaves), detents within 20 of an octave
                    when 22 =>
                        v_e := resize(s_g1(12 downto 3), 11) + 256;
                        if v_e(7 downto 0) < 20 then v_e := v_e(10 downto 8) & x"00";
                        elsif v_e(7 downto 0) > 236 then v_e := (v_e(10 downto 8) + 1) & x"00"; end if;
                        z_oct <= v_e(10 downto 8); z_f <= v_e(7 downto 0);
                        r_ex_a <= v_e(7 downto 0);
                    when 23 =>
                        if z_f = 0 then z_sh <= z_oct; else z_sh <= z_oct + 1; end if;
                        -- Mirror: the V ramp drifts in whole cells only, so the row at the fold never thins
                        -- to a single line
                        if s_mir = '1' then c_syfe <= (others => '0'); else c_syfe <= c_syf; end if;
                    when 24 => z_mt <= r_ex_q; r_ex_a <= (not z_f) + 1;
                    when 25 =>                        -- step = mt 2^oct / 128 (Q8 codes per cell)
                        case to_integer(z_oct) is
                            when 1      => c_step <= resize(z_mt(15 downto 6), 14);
                            when 2      => c_step <= resize(z_mt(15 downto 5), 14);
                            when 3      => c_step <= resize(z_mt(15 downto 4), 14);
                            when 4      => c_step <= resize(z_mt(15 downto 3), 14);
                            when others => c_step <= z_mt(15 downto 2);
                        end case;
                    when 26 => z_mb <= r_ex_q;
                    when 27 =>                        -- second window opens above 60 %: (p - 614) * 160 >> 8
                        v_p := s_gp(12 downto 3);
                        if v_p > 614 then
                            v_p := v_p - 614;
                            v_x := resize(shift_right(shift_left(resize(v_p, 17), 7) + shift_left(resize(v_p, 17), 5), 8), 12);
                            if v_x > 255 then c_w2q <= (others => '1'); else c_w2q <= v_x(7 downto 0); end if;
                        else
                            c_w2q <= (others => '0');
                        end if;
                    when 50 =>                        -- inv = 139810 * b >> (15 + sh)
                        case to_integer(z_sh) is
                            when 1      => c_inv <= f_acc(32 downto 16);
                            when 2      => c_inv <= f_acc(33 downto 17);
                            when 3      => c_inv <= f_acc(34 downto 18);
                            when 4      => c_inv <= f_acc(35 downto 19);
                            when others => c_inv <= resize(f_acc(35 downto 20), 17);
                        end case;
                    when 51 =>                        -- cells per pixel = xs * inv
                        case to_integer(s_xs) is
                            when 4      => c_dpx <= resize(c_inv & "00", 21);
                            when 3      => c_dpx <= resize(c_inv & '0', 21) + resize(c_inv, 21);
                            when others => c_dpx <= resize(c_inv & '0', 21);
                        end case;
                    when 52 => c_dpxn <= (not c_dpx) + 1;
                    when 74 => c_dq <= f_acc(27 downto 7);
                    when 75 => c_dqn <= (not c_dq) + 1;
                    when 98 => z_a <= f_acc(19 downto 0);
                    when 99 => c_dt <= resize(z_a, 19) + resize(z_a(19 downto 5), 19);
                    when 122 => c_dsx <= f_acc(31 downto 13);
                    when 123 => c_dsy <= resize(c_dsx(18 downto 2), 19) + resize(c_dsx(18 downto 3), 19);
                    when 146 => z_a <= f_acc(19 downto 0); z_ig <= f_acc(28 downto 20);
                    when 147 =>
                        v_c := ('0' & z_a) + ('0' & c_sxf);
                        c_fx0 <= v_c(19 downto 0);
                        if v_c(20) = '1' then z_ig <= z_ig + 1; end if;
                    when 170 => c_xc0 <= f_acc(15 downto 0) + c_scx;
                    when 194 => z_a <= f_acc(19 downto 0); z_ig <= f_acc(28 downto 20);
                    when 195 =>
                        v_c := ('0' & z_a) + ('0' & c_syfe);
                        c_fy0 <= v_c(19 downto 0);
                        if v_c(20) = '1' then z_ig <= z_ig + 1; end if;
                    when 218 => c_yc0 <= f_acc(15 downto 0) + c_scy;
                    when 242 => c_ry <= f_acc(26 downto 7);
                    when 266 => c_wid <= resize(f_acc(17 downto 10), 8) + 20;
                    when 290 => c_fct <= to_unsigned(1024, 11) - resize(f_acc(19 downto 10), 11);
                    when 314 => c_we <= f_acc(17 downto 10);
                    when 338 => c_we2 <= f_acc(15 downto 8);
                    when 362 => c_o1 <= f_acc(17 downto 10);
                    when 386 => c_o2 <= f_acc(17 downto 10);
                    when 410 =>
                        if f_acc(19 downto 10) > 18 then c_g32 <= to_unsigned(18, 5); else c_g32 <= f_acc(14 downto 10); end if;
                    when 420 =>
                        c_tr <= c_t(23 downto 16);
                        z_cgq <= to_unsigned(32, 6) - resize(c_g32, 6);
                        -- K6 camera: picture level min(k, 256) / 256 (Q5); window bite per held step 12 g
                        -- (camera) or 16 g (Osc), g = k / 256 up to k = 512 (x 2), then 2 + (k - 512) / 512 (x 3 at
                        -- the top): camera (min(3k, 1536) + 3k) / 128, Osc (min(k, 512) + k) / 32
                        if s_gc(12 downto 11) /= "00" then c_gb5 <= to_unsigned(32, 6);
                        else c_gb5 <= resize(s_gc(10 downto 6), 6); end if;
                        z_cgp <= resize(s_gc(12 downto 3), 12) + resize(s_gc(12 downto 3) & '0', 12);
                    when 421 =>
                        c_tg <= c_tr + c_o1; c_tb <= c_tr + c_o2;
                        if s_src = '1' then
                            if s_gc(12 downto 3) > 512 then z_nwq <= resize(s_gc(12 downto 3), 13) + 512;
                            else z_nwq <= resize(s_gc(12 downto 3) & '0', 13); end if;
                        else
                            if z_cgp > 1536 then z_nwq <= resize(z_cgp, 13) + 1536;
                            else z_nwq <= resize(z_cgp & '0', 13); end if;
                        end if;
                        z_cgp <= z_cgq * c_gb5;
                    when 422 =>
                        c_cg32 <= z_cgp(10 downto 5);
                        if s_src = '1' then c_nw <= z_nwq(10 downto 5); else c_nw <= z_nwq(12 downto 7); end if;
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
            s_osum <= shift_right(v_s, 1) + shift_right(v_s, 2);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then s_lb_wd <= std_logic_vector(s_osum);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: the V ramp's cell phase and ADC code as a DDA, one line
    -- ahead of the beam (so each line knows whether it is the last of its cell
    -- row).  Kicked in the zone for line 0 and at every line start.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_c  : unsigned(20 downto 0);
        variable v_rb : std_logic;
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 430 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_fy <= c_fy0; e_yc <= c_yc0; e_up <= not s_mir; e_rc <= '0'; e_ln <= (others => '0');
                e_st <= "01";
                l_first <= '1'; l_rbank <= '0'; l_seen <= '0'; l_since <= "00"; l_fpar <= not l_fpar;
            elsif g_start = '1' then
                -- the line starting now = A; its successor's row change = e_rc
                v_rb := l_rbank xor a_rc;
                l_rbank <= v_rb;
                l_yg <= a_yg;
                if s_rings = '1' then l_b2 <= a_b2(10 downto 0); else l_b2 <= (others => '0'); end if;
                if a_rc = '1' then
                    l_zone <= '1'; l_seen <= '1'; l_since <= "01";
                else
                    if l_seen = '1' and (l_since < 2 or (l_since = 2 and c_tz3 = '1')) then l_zone <= '1'; else l_zone <= '0'; end if;
                    if l_since /= 3 then l_since <= l_since + 1; end if;
                end if;
                l_last  <= e_rc;
                l_wr    <= l_first or e_rc;
                -- hold banks: read the top row's own pair (line 0: last field's), else this row's parity bank;
                -- write the next row's bank on a row's last line, or the top pair on line 0
                if l_first = '1' then
                    l_row0 <= '1'; l_rb2 <= '1' & (not l_fpar);
                elsif l_row0 = '1' and a_rc = '0' then
                    l_rb2 <= '1' & l_fpar;
                else
                    l_row0 <= '0'; l_rb2 <= '0' & v_rb;
                end if;
                if e_rc = '1' then l_wb2 <= '0' & (not v_rb); else l_wb2 <= '1' & l_fpar; end if;
                l_first <= '0';
                e_st <= "01";
            else
                case e_st is
                    when "01" =>
                        a_yc <= e_yc; a_rc <= e_rc;
                        e_dn <= '0'; e_fl <= '0';
                        if s_mir = '1' and e_ln = c_nfv1 then
                            v_c := ('0' & c_syfe) + ('0' & c_ry);
                            e_fy <= v_c(19 downto 0); e_rc <= v_c(20); e_up <= '1'; e_fl <= '1';
                        elsif e_up = '1' then
                            v_c := ('0' & e_fy) + c_dq;
                            e_fy <= v_c(19 downto 0); e_rc <= v_c(20);
                        else
                            v_c := ('0' & e_fy) + c_dqn;
                            e_fy <= v_c(19 downto 0); e_rc <= v_c(20); e_dn <= '1';
                        end if;
                        e_ln <= e_ln + 1;
                        e_st <= "10";
                    when "10" =>
                        if s_gray = '1' then a_yg <= f_gray(a_yc(15 downto 8)); else a_yg <= a_yc(15 downto 8); end if;
                        if e_fl = '1' then
                            if e_rc = '1' then e_yc <= c_scy + resize(c_step, 16); else e_yc <= c_scy; end if;
                        elsif e_rc = '1' then
                            if e_dn = '1' then e_yc <= e_yc - resize(c_step, 16); else e_yc <= e_yc + resize(c_step, 16); end if;
                        end if;
                        e_st <= "11";
                    when "11" =>
                        a_b2 <= a_yg * a_yg;
                        e_st <= "00";
                    when others => null;
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1) and the H ADC code (stage 2): the cell phase as a 20-bit
    -- fraction stepped per pixel; every carry / borrow is a cell clock, which
    -- steps the code accumulator by one cell's worth of codes.  Mirror runs
    -- the phase down to the fold pixel and reloads there.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_s : unsigned(20 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_fr <= c_fx0; b_up <= not s_mir; b_n <= (others => '0');
                b_fold <= '0'; b_ev <= '0'; b_evf <= '0'; b_evup <= '0';
                x_c <= c_xc0; x_col <= (others => '0');
            elsif g_avid = '1' then
                b_n <= b_n + 1;
                if b_n = c_nf2 then b_fold <= s_mir; else b_fold <= '0'; end if;
                if b_fold = '1' then
                    b_fr <= c_sxf; b_up <= '1'; b_ev <= '0'; b_evf <= '1';
                else
                    if b_up = '1' then v_s := ('0' & b_fr) + c_dpx; else v_s := ('0' & b_fr) + c_dpxn; end if;
                    b_fr <= v_s(19 downto 0); b_ev <= v_s(20); b_evf <= '0';
                end if;
                b_evup <= b_up;
                -- stage 2 from the stage-1 pixel's flags
                if b_evf = '1' then
                    x_c <= c_scx;
                elsif b_ev = '1' then
                    if b_evup = '1' then x_c <= x_c + resize(c_step, 16); else x_c <= x_c - resize(c_step, 16); end if;
                end if;
                if b_ev = '1' then x_col <= x_col + 1; end if;
            end if;
            e_sr <= e_sr(14 downto 0) & (b_ev and g_avid);
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_m   : unsigned(7 downto 0);
        variable v_w   : unsigned(7 downto 0);
        variable v_v   : unsigned(7 downto 0);
        variable v_s9  : unsigned(8 downto 0);
        variable v_lw  : unsigned(7 downto 0);
        variable v_d   : unsigned(7 downto 0);
        variable v_lit : std_logic_vector(2 downto 0);
        variable v_g, v_b : std_logic_vector(2 downto 0);
        variable v_k   : std_logic_vector(5 downto 0);
        variable v_kv  : std_logic;
        variable v_bg  : std_logic;
        variable v_e   : std_logic;
        variable v_gn  : unsigned(5 downto 0);
        variable v_cg  : unsigned(5 downto 0);
        variable v_y14 : unsigned(15 downto 0);
        variable v_yc5 : unsigned(16 downto 0);
        variable v_ys  : unsigned(12 downto 0);
        variable v_us  : signed(5 downto 0);
        variable v_gs  : signed(6 downto 0);
        variable v_ch  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 3: ADC word (binary / Gray); hold RAM address
            if s_gray = '1' then p3_xg <= f_gray(x_c(15 downto 8)); else p3_xg <= x_c(15 downto 8); end if;
            h_ra <= unsigned(l_rb2) & x_col;
            -- 4: Skew: the V word added to the H word
            if s_skew = '1' then p4_a <= p3_xg + l_yg; else p4_a <= p3_xg; end if;
            -- 5: logic candidates; multiplier partial products (A x B, or A x A for Rings); held luma (fabric)
            p5_a   <= p4_a;
            p5_xor <= p4_a xor l_yg;
            p5_and <= p4_a and l_yg;
            p5_or  <= p4_a or l_yg;
            p5_sum <= resize(p4_a, 9) + resize(l_yg, 9);
            p5_dif <= p4_a - l_yg;
            if s_rings = '1' then v_m := p4_a; else v_m := l_yg; end if;
            p5_pl <= p4_a * v_m(3 downto 0);
            p5_ph <= p4_a * v_m(7 downto 4);
            p5_lq <= unsigned(h_q(2 downto 0));
            -- 6: product; Lattice, Carry; the luma's bite out of the window
            p6_p   <= p5_pl(10 downto 0) + (p5_ph(6 downto 0) & "0000");
            p6_xor <= p5_xor; p6_and <= p5_and; p6_or <= p5_or; p6_sum <= p5_sum(7 downto 0);
            p6_lat <= p5_sum(7 downto 0) xor p5_dif;
            v_s9 := p5_sum xor ('0' & p5_a) xor ('0' & l_yg);
            p6_car <= v_s9(8 downto 1);
            p6_lq <= p5_lq;
            p6_lw <= p5_lq * c_nw;   -- K6: the luma's bite, held step x c_nw (12 g; Osc 16 g)
            -- 7: + B^2 (Rings); the other functions; window widths for this cell
            p7_pr <= p6_p + l_b2;
            case to_integer(s_fn) is
                when 0      => p7_w0 <= p6_xor;
                when 1      => p7_w0 <= p6_and;
                when 2      => p7_w0 <= p6_or;
                when 3      => p7_w0 <= p6_sum;
                when 4      => p7_w0 <= p6_lat;
                when others => p7_w0 <= p6_car;
            end case;
            if resize(p6_lw, 10) + 8 < resize(c_we, 10) then p7_wl <= c_we - p6_lw(7 downto 0); else p7_wl <= to_unsigned(8, 8); end if;
            if resize(p6_lw, 10) < resize(c_we2, 10) then p7_wl2 <= c_we2 - p6_lw(7 downto 0); else p7_wl2 <= (others => '0'); end if;
            -- 8: the word; its distance past each threshold; the six DAC bits
            if s_mul = '1' then v_w := p7_pr(10 downto 3); else v_w := p7_w0; end if;
            p8_dr <= v_w - c_tr; p8_dg <= v_w - c_tg; p8_db <= v_w - c_tb;
            case to_integer(s_rot) is
                when 0      => v_v := v_w;
                when 1      => v_v := v_w(6 downto 0) & v_w(7);
                when 2      => v_v := v_w(5 downto 0) & v_w(7 downto 6);
                when 3      => v_v := v_w(4 downto 0) & v_w(7 downto 5);
                when 4      => v_v := v_w(3 downto 0) & v_w(7 downto 4);
                when 5      => v_v := v_w(2 downto 0) & v_w(7 downto 3);
                when 6      => v_v := v_w(1 downto 0) & v_w(7 downto 2);
                when others => v_v := v_w(0) & v_w(7 downto 1);
            end case;
            p8_lv <= v_v(7) & v_v(4) & v_v(6) & v_v(3) & v_v(5) & v_v(2);
            p8_wl <= p7_wl; p8_wl2 <= p7_wl2;
            -- 9: three window comparators (each with its second window half a range away)
            v_lit := "000";
            v_d := p8_dr; if v_d < p8_wl or (v_d xor x"80") < p8_wl2 then v_lit(2) := '1'; end if;
            v_d := p8_dg; if v_d < p8_wl or (v_d xor x"80") < p8_wl2 then v_lit(1) := '1'; end if;
            v_d := p8_db; if v_d < p8_wl or (v_d xor x"80") < p8_wl2 then v_lit(0) := '1'; end if;
            if s_gav_sr(6) = '1' then
                p9_lit <= v_lit; p9_lv <= p8_lv;
            else
                p9_lit <= "000"; p9_lv <= (others => '0');
            end if;
            -- 10: DAC settling: the G and B channels go through delay lines
            p10_r <= p9_lit(2) & p9_lv(5 downto 4);
            g_sr(0) <= p9_lit(1) & p9_lv(3 downto 2);
            b_sr(0) <= p9_lit(0) & p9_lv(1 downto 0);
            for i in 1 to 8 loop g_sr(i) <= g_sr(i - 1); end loop;
            for i in 1 to 16 loop b_sr(i) <= b_sr(i - 1); end loop;
            -- 11: the late taps (same fringe width in virtual px at every resolution)
            p11_r <= p10_r;
            case to_integer(s_xs) is
                when 4      => p11_g <= g_sr(4); p11_b <= b_sr(8);
                when 3      => p11_g <= g_sr(5); p11_b <= b_sr(10);
                when others => p11_g <= g_sr(8); p11_b <= b_sr(16);
            end case;
            -- 12: keyer: no window open = background (the quilt's own code, shown dim / under the camera)
            v_bg := not (p11_r(2) or p11_g(2) or p11_b(2));
            p12_bg <= v_bg;
            if v_bg = '1' or p11_r(2) = '1' then p12_code(5 downto 4) <= p11_r(1 downto 0); else p12_code(5 downto 4) <= "00"; end if;
            if v_bg = '1' or p11_g(2) = '1' then p12_code(3 downto 2) <= p11_g(1 downto 0); else p12_code(3 downto 2) <= "00"; end if;
            if v_bg = '1' or p11_b(2) = '1' then p12_code(1 downto 0) <= p11_b(1 downto 0); else p12_code(1 downto 0) <= "00"; end if;
            -- pixel index of the stage-9 .. 13 pixels; code line buffer: address at 10, data 11, fabric 12;
            -- written at 13
            if s_gav_sr(7) = '1' then n9 <= n9 + 1; else n9 <= (others => '0'); end if;
            n10 <= n9; n11 <= n10; n12 <= n11; n13 <= n12;
            c_ra <= n9;
            c_qf <= c_q;
            c_wd <= "00" & p12_code; c_wa <= n12; k_we <= l_last and s_gav_sr(10);
            -- 13: Planes: Filled, or Outline (a DAC channel is drawn only where it changes: against the pixel
            -- a fringe-width to the left, and on a cell row's first lines against the row above)
            k_sr(0) <= p12_code;
            for i in 1 to 7 loop k_sr(i) <= k_sr(i - 1); end loop;
            case to_integer(s_xs) is
                when 4      => v_k := k_sr(3); v_kv := s_gav_sr(14);
                when 3      => v_k := k_sr(4); v_kv := s_gav_sr(15);
                when others => v_k := k_sr(7); v_kv := s_gav_sr(18);
            end case;
            if s_outl = '1' then
                for c in 0 to 2 loop
                    v_e := '0';
                    if v_kv = '1' and p12_code(2 * c + 1 downto 2 * c) /= v_k(2 * c + 1 downto 2 * c) then v_e := '1'; end if;
                    if l_zone = '1' and p12_code(2 * c + 1 downto 2 * c) /= c_qf(2 * c + 1 downto 2 * c) then v_e := '1'; end if;
                    p13_code(2 * c + 1 downto 2 * c) <= v_e & v_e;
                end loop;
            else
                p13_code <= p12_code;
            end if;
            p13_bg <= p12_bg;
            -- 14: colouriser ROM address; source line buffer address
            r_pa <= s_pal & unsigned(p13_code);
            p14_bg <= p13_bg;
            l_ra <= n13;
            -- 15: ROM / line buffer data (block RAM registers)
            p15_bg <= p14_bg;
            -- 16: fabric copies
            p16_w <= r_pq; p16_bg <= p15_bg;
            p16_cam <= unsigned(l_q);
            -- 17: the DAC reference: full for a lit pixel; background = the quilt at the MUNCH-0 level plus the
            -- camera through the rest (four small multipliers, nothing else in this stage)
            if p16_bg = '1' then v_gn := resize(c_g32, 6); v_cg := c_cg32;
            else v_gn := to_unsigned(32, 6); v_cg := (others => '0'); end if;
            p17_yq <= unsigned(p16_w(15 downto 10)) * v_gn;
            v_us := signed(resize(unsigned(p16_w(9 downto 5)), 6)) - 16;
            v_gs := signed(resize(v_gn, 7));
            p17_us <= v_us * v_gs;
            v_us := signed(resize(unsigned(p16_w(4 downto 0)), 6)) - 16;
            p17_vs <= v_us * v_gs;
            p17_yc <= p16_cam * v_cg;
            -- 18: luma terms (x 14 / 32 and x 5 / 32), chroma
            v_y14 := shift_left(resize(p17_yq, 16), 4) - shift_left(resize(p17_yq, 16), 1);
            p18_ya <= resize(v_y14(15 downto 5), 12);
            v_yc5 := shift_left(resize(p17_yc, 17), 2) + resize(p17_yc, 17);
            p18_yb <= v_yc5(16 downto 5);
            v_ch := p17_us + 512; p18_u <= unsigned(v_ch(9 downto 0));
            v_ch := p17_vs + 512; p18_v <= unsigned(v_ch(9 downto 0));
            -- 19: compose; black outside the generated avid
            v_ys := resize(p18_ya, 13) + resize(p18_yb, 13) + 64;
            if s_gav_sr(16) = '0' then
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            else
                if v_ys > 940 then s_y19 <= to_unsigned(940, 10); else s_y19 <= v_ys(9 downto 0); end if;
                s_u19 <= p18_u; s_v19 <= p18_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(16);

            -- per-cell luma hold (for the NEXT cell row): a leaky average along the line (stage 17), written
            -- at each cell's last pixel on the row's last line, into the other bank
            if s_gav_sr(14) = '1' then
                q_acc <= q_acc - shift_right(q_acc, 4) + p16_cam;
            else
                q_acc <= (others => '0');
            end if;
            if s_gav_sr(14) = '1' and e_sr(14) = '1' then
                q_col <= q_col + 1;
            elsif s_gav_sr(14) = '0' and s_gav_sr(15) = '0' then
                q_col <= (others => '0');
            end if;
            h_we <= l_wr and s_gav_sr(15) and (e_sr(14) or not s_gav_sr(14));
            h_wa <= unsigned(l_wb2) & q_col;
            h_wd <= '0' & std_logic_vector(q_acc(11 downto 9));
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16; luma [1 3 4 4 3 1]/16
    -- plus a quarter of its difference from the 9-tap average (edge peaking);
    -- luma 13/16; blanking gate.  The running 8-sums snap back to black once 8
    -- blank samples are in, so a glitched sample can't offset the picture.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_r  : unsigned(13 downto 0);
        variable v_pk : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 20: 8-sums; luma pair sum
            s_cy(0) <= s_y19; s_cu(0) <= s_u19; s_cv(0) <= s_v19;
            for i in 1 to 7 loop s_cy(i) <= s_cy(i - 1); s_cu(i) <= s_cu(i - 1); s_cv(i) <= s_cv(i - 1); end loop;
            if s_bah = "00000000" then
                s_by <= to_unsigned(512, 13); s_bu <= to_unsigned(4096, 13); s_bv <= to_unsigned(4096, 13);
            else
                s_by <= s_by + s_y19 - s_cy(7);
                s_bu <= s_bu + s_u19 - s_cu(7);
                s_bv <= s_bv + s_v19 - s_cv(7);
            end if;
            s_byd <= s_by; s_bud <= s_bu; s_bvd <= s_bv;
            s_lp <= resize(s_y19, 11) + resize(s_cy(0), 11);
            -- 21: 9-tap outputs; luma 4-sum
            v_sc := resize(s_by, 14) + s_byd; s_y21 <= v_sc(13 downto 4);
            v_sc := resize(s_bu, 14) + s_bud; s_u21 <= v_sc(13 downto 4);
            v_sc := resize(s_bv, 14) + s_bvd; s_v21 <= v_sc(13 downto 4);
            s_lp1 <= s_lp; s_lp2 <= s_lp1;
            s_lq <= resize(s_lp, 12) + resize(s_lp2, 12);
            -- 22: luma [1 2 1] of the 4-sums (one stage later than the 9-tap, so delayed once to pair up)
            s_lq1 <= s_lq; s_lq2 <= s_lq1;
            v_r := resize(s_lq, 14) + shift_left(resize(s_lq1, 14), 1) + resize(s_lq2, 14);
            s_lr <= v_r; s_lrd <= s_lr;
            s_ys22 <= s_lrd(13 downto 4);
            s_yd22 <= signed(resize(s_lrd(13 downto 4), 11)) - signed(resize(s_y21, 11));
            s_u22 <= s_u21; s_v22 <= s_v21;
            -- 23: peaking, clipped
            v_pk := signed(resize(s_ys22, 12)) + resize(shift_right(s_yd22, 2), 12);
            if v_pk < 64 then s_y23 <= to_unsigned(64, 10);
            elsif v_pk > 940 then s_y23 <= to_unsigned(940, 10);
            else s_y23 <= unsigned(v_pk(9 downto 0)); end if;
            s_u23 <= s_u22; s_v23 <= s_v22;
            -- 24: luma 13/16
            v_dl := s_y23 - 64;
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

end architecture munch;
