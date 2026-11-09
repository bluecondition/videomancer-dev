-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Gable v0.1.0 -- Heirloom (video phase), from the clip WIN_20230129_23_23_56 at 0.5 s.
--
-- The still: a fan of pointed cyan chevrons (a gable) on dark, apex top centre, the bands torn into horizontal
-- strokes; red hairlines across the top that thicken going down into red flanks left and right; a red / white
-- core low in the gable with a dark arch under it.  In the clip the stroke pattern rolls down (69 lines/s, one
-- 52.6-line period every 0.76 s) while the chevrons stand still.
--
-- The patch, as fitted (gable.py has every table):
--   phi    = y - (S + K y) |x - 1022|          a V ramp minus a VCA'd H triangle: the chevron fan
--   p      = (y - roll) / 52.6                 the rolling stroke oscillator's phase
--   v      = Bands (phi + tear(p))             the colouriser's input (+ camera / Osc luma)
--   cyan   = Ic(v) where Lc(v) > N(run(p), v/32)          bricks: a fixed threshold per run and cell
--   red    = Ir(v) softkey(Lr(v) - W(p) + vr(y)) s(p, v)  line PWM against the stroke waveform W
--   mix    = red over cyan outside the gable, both inside, red forcing white in the deep core
--
--   K1  Fan       how much the chevrons fan (the VCA depth): bell .. parallel .. the still (512) .. hourglass
--   K2  Bands     the colouriser's gain: x0.25 .. x4 (the still at 512)
--   K3  Strokes   depth of the stroke texture: solid bands (0) .. the still (512) .. thin shreds (1023)
--   K4  Speed     bipolar: the strokes roll down (+, 700 = the clip's 69 lines/s) or up
--   K5  Tear      the per-run sync slips: clean (0) .. the still (512) .. x4 (square law)
--   K6  Etch      how far the camera (or the Osc) bends the chevrons
--   S7  Twin      one gable / two gables side by side
--   S8  Flip      the gable / a valley (the field and the roll flipped top to bottom)
--   S9  Colours   Red Cyan (the still) / Magenta Cyan (the clip 23_50_10)
--   S10 Arch      Pointed (the still) / Round: a parabolic arch
--   S11 Source    Video / Osc
--   P12 PITCH     the roof's slope: flat strata (0) .. the still (384) .. a needle spire (1023)
--
-- Engine: vblank-zone sequencer (shared glide, products on the line engine's serial multiplier), a per-line
-- engine kicked at each line start (computes the NEXT line: stroke phase, slope, tear, the field DDA's start,
-- step and second-order terms, the red threshold), a beam DDA (pointed: reflecting at the folds; round: second
-- order), a 33-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid;
-- output conditioning (U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the running-sum resync, Y through
-- [1 2 1]/4, luma 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.gable_pkg.all;
use work.all;

architecture gable of program_top is
    constant C_LAT : integer := 32;   -- = processing_delay_clks

    constant C_P0I  : integer := 5500;     -- stroke phase at power-up, Q16 (the still's hairline rows)
    constant C_CX   : integer := 2044;     -- gable axis (h): virtual 1022
    constant C_CX1  : integer := 1084;     -- twin axes (h)
    constant C_CX2  : integer := 3004;

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
    -- the red streak levels (eighths)
    function f_s8(s : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(s) is
            when 0      => return to_unsigned(0, 4);
            when 1      => return to_unsigned(4, 4);
            when 2      => return to_unsigned(6, 4);
            when 3 | 4  => return to_unsigned(8, 4);
            when 5      => return to_unsigned(9, 4);
            when 6      => return to_unsigned(10, 4);
            when others => return to_unsigned(3, 4);
        end case;
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
    signal r_ex_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q  : unsigned(15 downto 0) := (others => '0');
    signal r_row_a, r_vr_a : unsigned(5 downto 0) := (others => '0');
    signal r_row_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_vr_q  : std_logic_vector(7 downto 0) := (others => '0');
    signal r_col_a : unsigned(8 downto 0) := (others => '0');
    signal r_col_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_brk_a : unsigned(9 downto 0) := (others => '0');
    signal r_brk_q : std_logic_vector(7 downto 0) := (others => '0');
    signal r_cr_a, r_cc_a : unsigned(7 downto 0) := (others => '0');
    signal r_cr_q, r_cc_q : std_logic_vector(31 downto 0) := (others => '0');

    -- vertical [1 1]/2: the previous line's channel levels {R 7, C 7}, half horizontal resolution (the odd
    -- pixel of each pair).  The read (pixel n, address n/2) never meets the write (pixel n - 3).
    type t_vb is array (0 to 1023) of std_logic_vector(13 downto 0);
    signal vb : t_vb := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of vb : signal is true;
    signal vb_ra, vb_wa : unsigned(9 downto 0) := (others => '0');
    signal vb_wd : std_logic_vector(13 downto 0) := (others => '0');
    signal vb_we : std_logic := '0';
    signal vb_q  : std_logic_vector(13 downto 0) := (others => '0');

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
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(9 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(600 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(384 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_twin, s_flip, s_pal, s_arch, s_src : std_logic := '0';

    -- per-field constants
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_hi   : std_logic := '0';
    signal z_sq   : unsigned(17 downto 0) := (others => '0');
    signal z_e2   : unsigned(15 downto 0) := (others => '0');
    signal c_g8   : unsigned(9 downto 0) := to_unsigned(256, 10);     -- Bands gain, Q8
    signal c_pc   : unsigned(14 downto 0) := to_unsigned(3600, 15);   -- pitch, Q12 rows per virtual px
    signal c_kf   : signed(11 downto 0) := to_signed(504, 12);        -- fan, Q20 rows per px per line
    signal c_tg   : unsigned(8 downto 0) := to_unsigned(128, 9);      -- tear gain, Q7
    signal c_dep  : unsigned(7 downto 0) := to_unsigned(128, 8);      -- stroke depth, Q7
    signal c_pst  : unsigned(13 downto 0) := (others => '0');          -- roll step per field, Q16 cycles
    signal c_p0   : unsigned(15 downto 0) := to_unsigned(C_P0I, 16);  -- stroke phase at line 0
    signal c_eg   : unsigned(12 downto 0) := (others => '0');          -- etch gain, index units per luma step
    signal c_eoff : unsigned(19 downto 0) := (others => '0');          -- Osc centring, index units
    signal c_x1, c_x2 : unsigned(11 downto 0) := (others => '0');   -- x of pixels 1, 2 (h)
    signal c_fx   : unsigned(10 downto 0) := to_unsigned(2044, 11);   -- first fold - start (h)
    signal c_u0sq : unsigned(21 downto 0) := to_unsigned(4177936, 22);-- (start - first axis)^2 (h^2)
    signal c_k0   : unsigned(13 downto 0) := to_unsigned(8172, 14);   -- |2 u0 xs + xs^2|

    -- field products (on the line engine's multiplier, which is idle in the zone)
    signal z_fa  : unsigned(21 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(2 downto 0) := (others => '0');
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
    signal e_yf   : unsigned(10 downto 0) := (others => '0');
    signal e_ady  : unsigned(9 downto 0) := (others => '0');
    signal e_dyn  : std_logic := '0';
    signal e_mn   : std_logic := '0';
    signal e_p    : unsigned(15 downto 0) := (others => '0');
    signal e_w7   : unsigned(6 downto 0) := (others => '0');
    signal e_tear : signed(7 downto 0) := (others => '0');
    signal e_vr   : signed(7 downto 0) := (others => '0');
    signal e_wma  : unsigned(6 downto 0) := (others => '0');
    signal e_wmn  : std_logic := '0';
    signal e_m    : signed(16 downto 0) := (others => '0');
    signal e_ma   : unsigned(15 downto 0) := (others => '0');
    signal e_ms   : std_logic := '0';
    signal e_t2   : signed(10 downto 0) := (others => '0');
    signal e_yt   : signed(12 downto 0) := (others => '0');
    signal e_yta  : unsigned(11 downto 0) := (others => '0');
    signal e_ytn  : std_logic := '0';
    signal e_d1m  : unsigned(25 downto 0) := (others => '0');
    signal e_base : unsigned(27 downto 0) := (others => '0');
    signal e_a0p, e_a0r : unsigned(27 downto 0) := (others => '0');
    signal e_s    : signed(23 downto 0) := (others => '0');
    signal e_d0   : signed(30 downto 0) := (others => '0');
    signal e_dd   : signed(30 downto 0) := (others => '0');
    signal e_j    : signed(30 downto 0) := (others => '0');
    signal e_jm   : unsigned(27 downto 0) := (others => '0');
    signal e_wd   : signed(9 downto 0) := (others => '0');
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    -- next line (loaded at g_start)
    signal n_a0   : unsigned(27 downto 0) := (others => '0');
    signal n_s    : signed(23 downto 0) := (others => '0');
    signal n_d0, n_dd, n_j : signed(30 downto 0) := (others => '0');
    signal n_rthr : signed(9 downto 0) := (others => '0');
    signal n_run  : unsigned(2 downto 0) := (others => '0');

    -- beam
    signal b_a   : unsigned(27 downto 0) := (others => '0');
    signal b_d   : signed(30 downto 0) := (others => '0');
    signal b_xn, b_xnn : unsigned(11 downto 0) := (others => '0');
    signal f_up, f_jmp : std_logic := '0';
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_s   : signed(23 downto 0) := (others => '0');
    signal l_dd, l_j : signed(30 downto 0) := (others => '0');
    signal l_rthr : signed(9 downto 0) := (others => '0');
    signal l_run : unsigned(2 downto 0) := (others => '0');

    -- pixel path (the number is the stage the register holds)
    type t_i20 is array (natural range <>) of unsigned(19 downto 0);
    signal q_i   : t_i20(2 to 8) := (others => (others => '0'));       -- index I = A >> 8 (mod 2^20)
    signal q4_l, q5_l : unsigned(7 downto 0) := (others => '0');
    signal q5_a, q6_a, q7_a : unsigned(8 downto 0) := (others => '0');
    signal q6_lb : unsigned(7 downto 0) := (others => '0');
    signal q7_el, q7_eh : unsigned(16 downto 0) := (others => '0');
    signal q8_et : unsigned(20 downto 0) := (others => '0');
    signal q9_i  : unsigned(19 downto 0) := (others => '0');
    signal q12_col : std_logic_vector(31 downto 0) := (others => '0');
    signal q12_brk : std_logic_vector(7 downto 0) := (others => '0');
    signal q13_lc, q13_ic, q13_ir, q14_ic, q14_ir, q15_ic, q15_ir : unsigned(6 downto 0) := (others => '0');
    signal q13_occ, q13_wht, q14_occ, q14_wht : std_logic := '0';
    signal q13_kd  : signed(10 downto 0) := (others => '0');
    signal q13_np  : unsigned(13 downto 0) := (others => '0');
    signal q13_nn  : std_logic := '0';
    signal q13_s   : unsigned(2 downto 0) := (others => '0');
    signal q14_kc  : std_logic := '0';
    signal q14_kl  : unsigned(2 downto 0) := (others => '0');
    signal q14_s8  : unsigned(3 downto 0) := (others => '0');
    signal q15_rf  : unsigned(5 downto 0) := (others => '0');
    signal q15_cl  : unsigned(2 downto 0) := (others => '0');
    signal q16_r, q16_c : unsigned(7 downto 0) := (others => '0');
    signal q17_ar, q17_ac, q18_br, q18_bc : unsigned(9 downto 0) := (others => '0');
    signal q_cnt : unsigned(10 downto 0) := (others => '0');
    signal q17_n, q18_n, q19_n : unsigned(10 downto 0) := (others => '0');
    signal q19_r7, q19_c7, q20_r7, q20_c7, q21_rv, q21_cv : unsigned(6 downto 0) := (others => '0');
    signal q20_pr : std_logic_vector(13 downto 0) := (others => '0');
    signal q24_cr, q24_cc : std_logic_vector(31 downto 0) := (others => '0');
    signal q25_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q25_u, q25_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y26, s_u26, s_v26 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cau, s_cav, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bbu, s_bbv, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_ly : t_u10(0 to 11) := (others => to_unsigned(64, 10));   -- luma delay (aligns [1 2 1] with the chroma centre)
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 inputs of boxes A, B
    signal s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y30, s_u30, s_v30 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y31, s_u31, s_v31 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y32, s_u32, s_v32 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y33 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u33, s_v33 : unsigned(9 downto 0) := to_unsigned(512, 10);

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

    p_vb : process(clk)
    begin
        if rising_edge(clk) then
            vb_q <= vb(to_integer(vb_ra));
            if vb_we = '1' then vb(to_integer(vb_wa)) <= vb_wd; end if;
        end if;
    end process p_vb;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q  <= C_GB_EXP2(to_integer(r_ex_a));
            r_row_q <= C_GB_ROW(to_integer(r_row_a));
            r_vr_q  <= C_GB_VR(to_integer(r_vr_a));
            r_col_q <= C_GB_COL(to_integer(r_col_a));
            r_brk_q <= C_GB_BRK(to_integer(r_brk_a));
            r_cr_q  <= C_GB_CR(to_integer(r_cr_a));
            r_cc_q  <= C_GB_CC(to_integer(r_cc_a));
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
        variable v_k  : signed(11 downto 0);
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
            elsif s_zc /= 1023 then
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
                s_flip <= registers_in(6)(1);
                s_pal  <= registers_in(6)(2);
                s_arch <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(408, 9);
                    when 3      => s_oxs <= to_unsigned(306, 9);
                    when others => s_oxs <= to_unsigned(204, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(242, 8), 8), 11);

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
            -- load + 17 (stored at load + 18).  The slot number is decoded at 30 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 31 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 3); z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 3); z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 3); z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 3); z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 3); z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 3); z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 3); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0 =>                                                                   -- pitch
                        if z_hi = '1' then z_fa <= resize(s_gp(12 downto 3) - 384, 22); z_fb <= to_unsigned(45, 16);
                        else z_fa <= resize(s_gp(12 downto 3), 22); z_fb <= to_unsigned(361, 16); end if;
                    when 1 => z_fa <= resize(s_g5(12 downto 3), 22); z_fb <= resize(s_g5(12 downto 3), 16);   -- tear^2
                    when 2 => z_fa <= resize(s_vm, 22);              z_fb <= resize(s_vm, 16);                -- speed^2
                    when 3 => z_fa <= resize(z_sq, 22);              z_fb <= to_unsigned(2903, 16);           -- roll step
                    when 4 => z_fa <= resize(s_g6(12 downto 3), 22); z_fb <= resize(s_g6(12 downto 3), 16);   -- etch^2
                    when 5 => z_fa <= resize(z_e2, 22);              z_fb <= resize(c_g8, 16);                -- etch gain
                    when others => z_fa <= resize(c_eg, 22);         z_fb <= to_unsigned(111, 16);            -- Osc centring
                end case;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's step): the strokes roll down for Speed > 0
                    when 20 =>
                        if s_vneg = '1' then c_p0 <= c_p0 + resize(c_pst, 16);
                        else c_p0 <= c_p0 - resize(c_pst, 16); end if;
                        -- Bands exponent (+-2 octaves) into the exp2 table; pitch segment
                        v_e := signed(resize(s_g2(12 downto 3), 11)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                        if s_gp(12 downto 3) > 384 then z_hi <= '1'; else z_hi <= '0'; end if;
                    when 21 =>
                        -- fan = (k1 - 344) * 3 (Q20), depth = k3 / 4 (Q7)
                        v_k := signed(resize(s_g1(12 downto 3), 12)) - 344;
                        c_kf <= v_k + shift_left(v_k, 1);
                        c_dep <= s_g3(12 downto 5);
                        -- start x and the fold constants per mode (twin, x step)
                        if s_xs = 4 then
                            c_x1 <= to_unsigned(484, 12); c_x2 <= to_unsigned(488, 12);
                        elsif s_xs = 3 then
                            c_x1 <= to_unsigned(3, 12); c_x2 <= to_unsigned(6, 12);
                        else
                            c_x1 <= to_unsigned(2, 12); c_x2 <= to_unsigned(4, 12);
                        end if;
                    when 22 =>
                        if s_twin = '1' then
                            if s_xs = 4 then
                                c_fx <= to_unsigned(604, 11); c_u0sq <= to_unsigned(364816, 22); c_k0 <= to_unsigned(4816, 14);
                            elsif s_xs = 3 then
                                c_fx <= to_unsigned(1084, 11); c_u0sq <= to_unsigned(1175056, 22); c_k0 <= to_unsigned(6495, 14);
                            else
                                c_fx <= to_unsigned(1084, 11); c_u0sq <= to_unsigned(1175056, 22); c_k0 <= to_unsigned(4332, 14);
                            end if;
                        else
                            if s_xs = 4 then
                                c_fx <= to_unsigned(1564, 11); c_u0sq <= to_unsigned(2446096, 22); c_k0 <= to_unsigned(12496, 14);
                            elsif s_xs = 3 then
                                c_fx <= to_unsigned(2044, 11); c_u0sq <= to_unsigned(4177936, 22); c_k0 <= to_unsigned(12255, 14);
                            else
                                c_fx <= to_unsigned(2044, 11); c_u0sq <= to_unsigned(4177936, 22); c_k0 <= to_unsigned(8172, 14);
                            end if;
                        end if;
                    when 23 => z_mt <= r_ex_q;
                    when 24 =>                       -- Bands gain = mt >> (7 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when -2     => c_g8 <= resize(z_mt(15 downto 9), 10);
                            when -1     => c_g8 <= resize(z_mt(15 downto 8), 10);
                            when 0      => c_g8 <= resize(z_mt(15 downto 7), 10);
                            when others => c_g8 <= z_mt(15 downto 6);
                        end case;
                    when 50  =>                      -- pitch: 1434 + v * 361 >> 6, or 3600 + (v - 384) * 45 >> 1
                        if z_hi = '1' then c_pc <= to_unsigned(3600, 15) + m_acc(15 downto 1);
                        else c_pc <= to_unsigned(1434, 15) + resize(m_acc(17 downto 6), 15); end if;
                    when 74  => c_tg  <= m_acc(19 downto 11);
                    when 98  => z_sq  <= m_acc(17 downto 0);
                    when 122 => c_pst <= m_acc(29 downto 16);
                    when 146 => z_e2  <= m_acc(19 downto 4);
                    when 170 => c_eg  <= resize(m_acc(25 downto 15), 13);
                    when 171 =>
                        if s_src = '1' then c_eg <= c_eg + shift_right(c_eg, 1); end if;   -- the Osc at 1.5x the camera's gain
                    when 194 =>
                        if s_src = '1' then c_eoff <= m_acc(19 downto 0); else c_eoff <= (others => '0'); end if;
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
    -- 0), computes the NEXT line on one serial multiplier (16 iterations, result
    -- at load + 17):
    --   Yf = y (Flip: 1079 - y), p = p0 + 1246 Yf (Q16), W, tear = ROW[p >> 10], vr = VR[Yf >> 4]
    --   m = pitch + fan (Yf - 540) >> 8, D1 = m G, base = (Yf + tear tg >> 7) G 256 - eoff 256
    --   pointed: A0 = base - D1 fx >> 5, step S = D1 xs / 32
    --   round:   c = D1 >> 10, A0 = base - c u0^2 >> 5, D0 = c |K0|, DD = -2 c xs^2, J = 3840 c xs (Q5)
    --   red threshold = 64 + (W - 64) depth >> 7 - vr
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y  : unsigned(10 downto 0);
        variable v_d  : signed(11 downto 0);
        variable v_t  : unsigned(27 downto 0);
        variable v_u  : unsigned(25 downto 0);
        variable v_q  : unsigned(29 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 420 then e_zk <= '1'; end if;

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
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            -- the per-field sequencer's products (zone only: the line engine is idle then)
            if z_ld = '1' then
                m_a <= resize(z_fa, 36); m_b <= z_fb; m_acc <= (others => '0');
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    if e_vy(18 downto 8) > 1079 then v_y := to_unsigned(1079, 11); else v_y := e_vy(18 downto 8); end if;
                    if s_flip = '1' then e_yf <= to_unsigned(1079, 11) - v_y; else e_yf <= v_y; end if;
                when 2 =>
                    m_a <= resize(e_yf, 36); m_b <= to_unsigned(1246, 16); m_acc <= (others => '0');
                    v_d := signed(resize(e_yf, 12)) - 540;
                    if v_d < 0 then v_d := -v_d; e_dyn <= '1'; else e_dyn <= '0'; end if;
                    e_ady <= unsigned(v_d(9 downto 0));            -- |y - 540| <= 540 (a slice: no signed narrowing)
                when 19 =>
                    e_p <= c_p0 + m_acc(15 downto 0);
                    m_a <= resize(unsigned(abs(c_kf)), 36); m_b <= resize(e_ady, 16); m_acc <= (others => '0');
                    e_mn <= c_kf(11) xor e_dyn;
                when 20 =>
                    r_row_a <= e_p(15 downto 10);
                    if e_yf(10 downto 4) > 63 then r_vr_a <= (others => '1'); else r_vr_a <= e_yf(9 downto 4); end if;
                when 22 =>
                    e_w7   <= unsigned(r_row_q(17 downto 11));
                    e_tear <= signed(r_row_q(10 downto 3));
                    n_run  <= unsigned(r_row_q(2 downto 0));
                    e_vr   <= signed(r_vr_q);
                when 23 =>
                    if e_w7 >= 64 then e_wma <= e_w7 - 64; e_wmn <= '0';
                    else e_wma <= 64 - e_w7; e_wmn <= '1'; end if;
                when 36 =>                    -- m = pitch +- (|fan| |dy|) >> 8
                    if e_mn = '1' then e_m <= signed(resize(c_pc, 17)) - signed(resize(m_acc(20 downto 8), 17));
                    else e_m <= signed(resize(c_pc, 17)) + signed(resize(m_acc(20 downto 8), 17)); end if;
                    m_a <= resize(unsigned(abs(resize(e_tear, 9))), 36); m_b <= resize(c_tg, 16); m_acc <= (others => '0');
                when 37 =>
                    e_ma <= unsigned(resize(abs(e_m), 16)); e_ms <= e_m(16);
                when 53 =>                    -- t2 = +-(|tear| tg) >> 7
                    if e_tear(7) = '1' then e_t2 <= -signed(resize(m_acc(15 downto 7), 11));
                    else e_t2 <= signed(resize(m_acc(15 downto 7), 11)); end if;
                    m_a <= resize(e_ma, 36); m_b <= resize(c_g8, 16); m_acc <= (others => '0');
                when 54 =>
                    e_yt <= signed(resize(e_yf, 13)) + resize(e_t2, 13);
                when 55 =>
                    e_yta <= unsigned(resize(abs(e_yt), 12)); e_ytn <= e_yt(12);
                when 70 =>                    -- |D1| = |m| G
                    e_d1m <= m_acc(25 downto 0);
                    m_a <= resize(e_yta, 36); m_b <= resize(c_g8, 16); m_acc <= (others => '0');
                when 87 =>                    -- base = +-(|Yf + t2| G) << 8 (mod 2^28)
                    v_t := m_acc(19 downto 0) & x"00";
                    if e_ytn = '1' then e_base <= (not v_t) + 1; else e_base <= v_t; end if;
                    m_a <= resize(e_d1m, 36); m_b <= resize(c_fx, 16); m_acc <= (others => '0');
                when 88 =>
                    e_base <= e_base - (c_eoff & x"00");
                when 104 =>                   -- pointed start: A0 = base -+ (|D1| fx) >> 5
                    if e_ms = '1' then e_a0p <= e_base + m_acc(32 downto 5);
                    else e_a0p <= e_base - m_acc(32 downto 5); end if;
                    m_a <= resize(c_u0sq, 36); m_b <= e_d1m(25 downto 10); m_acc <= (others => '0');
                    -- pointed step S = D1 xs / 32
                    case to_integer(s_xs) is
                        when 4      => v_u := shift_right(e_d1m, 3);
                        when 3      => v_u := resize(shift_right(resize(e_d1m, 28) + shift_left(resize(e_d1m, 28), 1), 5), 26);
                        when others => v_u := shift_right(e_d1m, 4);
                    end case;
                    if e_ms = '1' then e_s <= -signed(resize(v_u, 24)); else e_s <= signed(resize(v_u, 24)); end if;
                when 121 =>                   -- round start: A0 = base -+ (u0^2 |c|) >> 5
                    if e_ms = '1' then e_a0r <= e_base + m_acc(32 downto 5);
                    else e_a0r <= e_base - m_acc(32 downto 5); end if;
                    m_a <= resize(c_k0, 36); m_b <= e_d1m(25 downto 10); m_acc <= (others => '0');
                    -- DD = -2 c xs^2 (Q5): 8 / 18 / 32 |c|
                    case to_integer(s_xs) is
                        when 4      => v_q := shift_left(resize(e_d1m(25 downto 10), 30), 5);
                        when 3      => v_q := shift_left(resize(e_d1m(25 downto 10), 30), 4) + shift_left(resize(e_d1m(25 downto 10), 30), 1);
                        when others => v_q := shift_left(resize(e_d1m(25 downto 10), 30), 3);
                    end case;
                    if e_ms = '1' then e_dd <= signed(resize(v_q, 31)); else e_dd <= -signed(resize(v_q, 31)); end if;
                    -- 3840 |c| = (|c| << 12) - (|c| << 8)
                    e_jm <= shift_left(resize(e_d1m(25 downto 10), 28), 12) - shift_left(resize(e_d1m(25 downto 10), 28), 8);
                when 122 =>                   -- J = 3840 c xs (Q5)
                    case to_integer(s_xs) is
                        when 4      => v_t := shift_left(e_jm, 2);
                        when 3      => v_t := shift_left(e_jm, 1) + e_jm;
                        when others => v_t := shift_left(e_jm, 1);
                    end case;
                    if e_ms = '1' then e_j <= -signed(resize(v_t, 31)); else e_j <= signed(resize(v_t, 31)); end if;
                when 138 =>                   -- D0 = c |K0|
                    if e_ms = '1' then e_d0 <= -signed(resize(m_acc(29 downto 0), 31));
                    else e_d0 <= signed(resize(m_acc(29 downto 0), 31)); end if;
                    m_a <= resize(e_wma, 36); m_b <= resize(c_dep, 16); m_acc <= (others => '0');
                when 155 =>                   -- W' = 64 +- (|W - 64| depth) >> 7
                    if e_wmn = '1' then e_wd <= to_signed(64, 10) - signed(resize(m_acc(14 downto 7), 10));
                    else e_wd <= to_signed(64, 10) + signed(resize(m_acc(14 downto 7), 10)); end if;
                when 156 =>
                    n_rthr <= e_wd - resize(e_vr, 10);
                    if s_arch = '1' then n_a0 <= e_a0r; else n_a0 <= e_a0p; end if;
                    n_s <= e_s; n_d0 <= e_d0; n_dd <= e_dd; n_j <= e_j;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: A = the field index x 256 (mod 2^28) as a DDA.  Pointed: +-S,
    -- reversing at the axis (twin: at both axes and the midpoint); Round: a
    -- second-order DDA (A += D >> 5, D += DD; twin: D jumps by J at the
    -- midpoint).  The flags for the next step are registered a clock ahead.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_x : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_a  <= n_a0;
                b_d  <= n_d0;
                b_xn <= c_x1;
                b_xnn <= c_x2;
                f_up <= '1';
                f_jmp <= '0';
                b_px <= (others => '0');
                l_s <= n_s; l_dd <= n_dd; l_j <= n_j; l_rthr <= n_rthr; l_run <= n_run;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                if s_arch = '1' then
                    b_a <= b_a + unsigned(resize(shift_right(b_d, 5), 28));
                    if f_jmp = '1' then b_d <= b_d + l_dd + l_j; else b_d <= b_d + l_dd; end if;
                elsif f_up = '1' then
                    b_a <= b_a + unsigned(resize(l_s, 28));
                else
                    b_a <= b_a - unsigned(resize(l_s, 28));
                end if;
                -- flags for the step into b_xnn (next clock)
                v_x := b_xnn;
                if s_twin = '1' then
                    if v_x <= C_CX1 or (v_x > C_CX and v_x <= C_CX2) then f_up <= '1'; else f_up <= '0'; end if;
                    if v_x > C_CX and b_xn <= C_CX then f_jmp <= '1'; else f_jmp <= '0'; end if;
                else
                    if v_x <= C_CX then f_up <= '1'; else f_up <= '0'; end if;
                    f_jmp <= '0';
                end if;
                b_xn  <= b_xnn;
                b_xnn <= b_xnn + s_xs;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = the beam value of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_n  : signed(7 downto 0);
        variable v_nd : signed(9 downto 0);
        variable v_kd : signed(10 downto 0);
        variable v_cl : unsigned(2 downto 0);
        variable v_r  : unsigned(12 downto 0);
        variable v_y  : unsigned(10 downto 0);
        variable v_u  : signed(11 downto 0);
        variable v_f  : signed(10 downto 0);
        variable v_a  : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- the index rides along
            q_i(2) <= b_a(27 downto 8);
            for i in 3 to 8 loop q_i(i) <= q_i(i - 1); end loop;
            -- 3: line buffer data; 4: fabric copy; 5-6: 4-px box (the camera bends rather than shreds)
            q4_l <= unsigned(s_lb_q);
            q5_l <= q4_l;
            q5_a <= resize(q4_l, 9) + q5_l;                     -- L(n) + L(n-1) at stage 5
            q6_a <= q5_a; q7_a <= q6_a;
            q6_lb <= resize(shift_right(resize(q5_a, 10) + q7_a, 2), 8);   -- (L(n) .. L(n-3)) / 4 at stage 6
            -- 7: etch partials (luma nibbles x gain); 8: etch; 9: I' = I + etch (mod 2^20)
            q7_el <= resize(q6_lb(3 downto 0) * c_eg, 17);
            q7_eh <= resize(q6_lb(7 downto 4) * c_eg, 17);
            q8_et <= resize(q7_el, 21) + shift_left(resize(q7_eh, 21), 4);
            q9_i  <= q_i(8) + q8_et(19 downto 0);
            -- 10: ROM addresses (colouriser: 4 phi units per entry; bricks: stroke run & 32-unit cell)
            r_col_a <= q9_i(18 downto 10);
            r_brk_a <= l_run & q9_i(19 downto 13);
            -- 11: ROM data (EBR); 12: fabric copy
            q12_col <= r_col_q; q12_brk <= r_brk_q;
            -- 13: decode; red key distance; brick threshold product
            q13_lc  <= unsigned(q12_col(30 downto 24));
            q13_ic  <= unsigned(q12_col(23 downto 17));
            q13_ir  <= unsigned(q12_col(8 downto 2));
            q13_occ <= q12_col(1); q13_wht <= q12_col(0);
            q13_kd  <= signed(resize(unsigned(q12_col(16 downto 9)), 11)) - resize(l_rthr, 11);
            v_n := signed(resize(unsigned(q12_brk(7 downto 4)) & "000", 8)) - 60;   -- 8 N - 60
            q13_np <= resize(unsigned(abs(v_n)) * c_dep, 14);
            q13_nn <= v_n(7);
            q13_s  <= unsigned(q12_brk(3 downto 1));
            -- 14: cyan key; red key level (4 steps over 13/128); streak level
            if q13_nn = '1' then v_nd := to_signed(64, 10) - signed(resize(q13_np(13 downto 7), 10));
            else v_nd := to_signed(64, 10) + signed(resize(q13_np(13 downto 7), 10)); end if;
            if signed(resize(q13_lc, 10)) > v_nd then q14_kc <= '1'; else q14_kc <= '0'; end if;
            v_kd := q13_kd;
            if v_kd <= 0 then q14_kl <= "000";
            elsif v_kd >= 13 then q14_kl <= "100";
            else
                v_kd := v_kd + 3;
                q14_kl <= unsigned(v_kd(4 downto 2));
            end if;
            q14_s8 <= f_s8(q13_s);
            q14_ic <= q13_ic; q14_ir <= q13_ir; q14_occ <= q13_occ; q14_wht <= q13_wht;
            -- 15: red factor; cyan level (red over cyan outside, red forcing white in the deep core)
            q15_rf <= resize(q14_kl * q14_s8, 6);
            if q14_wht = '1' then
                if q14_kl = 0 and q14_kc = '1' then v_cl := "001"; else v_cl := q14_kl; end if;
            elsif q14_kc = '1' then
                v_cl := "100";
            else
                v_cl := "000";
            end if;
            if q14_occ = '1' and q14_kl >= 2 then v_cl := "000"; end if;
            q15_cl <= v_cl;
            q15_ic <= q14_ic; q15_ir <= q14_ir;
            -- 16: channel levels
            v_r := q15_ir * q15_rf;
            if v_r(12) = '1' then q16_r <= (others => '1'); else q16_r <= v_r(11 downto 4); end if;
            case to_integer(q15_cl) is
                when 0      => q16_c <= (others => '0');
                when 1      => q16_c <= resize(q15_ic(6 downto 1), 8);
                when 2      => q16_c <= resize(q15_ic, 8);
                when 3      => q16_c <= resize(q15_ic, 8) + resize(q15_ic(6 downto 1), 8);
                when others => q16_c <= q15_ic & '0';
            end case;
            -- 17-18: two one-pole low-passes per channel (alpha 1/4, Q2), reset off the generated avid
            if s_gav_sr(14) = '0' then
                q17_ar <= (others => '0'); q17_ac <= (others => '0');
            else
                v_f := signed(resize(q17_ar, 11)) + shift_right(signed(resize(q16_r & "00", 11)) - signed(resize(q17_ar, 11)), 2);
                q17_ar <= unsigned(v_f(9 downto 0));
                v_f := signed(resize(q17_ac, 11)) + shift_right(signed(resize(q16_c & "00", 11)) - signed(resize(q17_ac, 11)), 2);
                q17_ac <= unsigned(v_f(9 downto 0));
            end if;
            if s_gav_sr(15) = '0' then
                q18_br <= (others => '0'); q18_bc <= (others => '0');
            else
                v_f := signed(resize(q18_br, 11)) + shift_right(signed(resize(q17_ar, 11)) - signed(resize(q18_br, 11)), 2);
                q18_br <= unsigned(v_f(9 downto 0));
                v_f := signed(resize(q18_bc, 11)) + shift_right(signed(resize(q17_ac, 11)) - signed(resize(q18_bc, 11)), 2);
                q18_bc <= unsigned(v_f(9 downto 0));
            end if;
            -- 17-20: the vertical [1 1]/2 store: read address with the pixel entering 17 (index q_cnt), data at
            -- 19, fabric copy at 20; the pixel entering 20 writes its levels (address n / 2, 3 pixels behind)
            if s_gav_sr(14) = '1' then
                vb_ra <= q_cnt(10 downto 1); q17_n <= q_cnt; q_cnt <= q_cnt + 1;
            else
                q_cnt <= (others => '0'); q17_n <= (others => '0');
            end if;
            q18_n <= q17_n; q19_n <= q18_n;
            q19_r7 <= q18_br(9 downto 3); q19_c7 <= q18_bc(9 downto 3);
            q20_r7 <= q19_r7; q20_c7 <= q19_c7; q20_pr <= vb_q;
            vb_we <= s_gav_sr(17); vb_wa <= q19_n(10 downto 1);
            vb_wd <= std_logic_vector(q19_r7) & std_logic_vector(q19_c7);
            -- 21: average with the previous line
            v_a := resize(q20_r7, 8) + resize(unsigned(q20_pr(13 downto 7)), 8) + 1;
            q21_rv <= v_a(7 downto 1);
            v_a := resize(q20_c7, 8) + resize(unsigned(q20_pr(6 downto 0)), 8) + 1;
            q21_cv <= v_a(7 downto 1);
            -- 22: colour ROM addresses (palette & 7-bit level); 23: ROM data (EBR); 24: fabric copy
            r_cr_a <= s_pal & q21_rv;
            r_cc_a <= s_pal & q21_cv;
            q24_cr <= r_cr_q; q24_cc <= r_cc_q;
            -- 25: sum the two channels' contributions (luma capped at 940)
            v_y := resize(unsigned(q24_cr(31 downto 22)), 11) + resize(unsigned(q24_cc(31 downto 22)), 11);
            if v_y > 940 then q25_y <= to_unsigned(940, 10); else q25_y <= v_y(9 downto 0); end if;
            v_u := signed(resize(unsigned(q24_cr(21 downto 12)), 12)) + signed(resize(unsigned(q24_cc(21 downto 12)), 12)) - 512;
            if v_u < 0 then q25_u <= (others => '0'); elsif v_u > 1023 then q25_u <= (others => '1'); else q25_u <= unsigned(v_u(9 downto 0)); end if;
            v_u := signed(resize(unsigned(q24_cr(11 downto 2)), 12)) + signed(resize(unsigned(q24_cc(11 downto 2)), 12)) - 512;
            if v_u < 0 then q25_v <= (others => '0'); elsif v_u > 1023 then q25_v <= (others => '1'); else q25_v <= unsigned(v_u(9 downto 0)); end if;
            -- 26: compose; black outside the generated avid
            if s_gav_sr(23) = '0' then
                s_y26 <= to_unsigned(64, 10); s_u26 <= to_unsigned(512, 10); s_v26 <= to_unsigned(512, 10);
            else
                s_y26 <= q25_y; s_u26 <= q25_u; s_v26 <= q25_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(23);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums), Y through [1 2 1]/4 on the same centre (the
    -- HD HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_yl : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to neutral (the off-avid input of both boxes) once 8 blank
            -- samples are in, so a glitched sample can't offset the colours for good.
            -- chroma box A: stage 27 sums, stage 28 output
            s_cau(0) <= s_u26; s_cav(0) <= s_v26;
            for i in 1 to 7 loop s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bau <= to_unsigned(512 * 8, 13); s_bav <= to_unsigned(512 * 8, 13);
            else
                s_bau <= s_bau + s_u26 - s_cau(7);
                s_bav <= s_bav + s_v26 - s_cav(7);
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: neutral off the generated avid (what box A gives there anyway)
            if s_gav_sr(25) = '1' then
                v_sc := resize(s_bau, 14) + s_baud; s_u28 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v28 <= v_sc(13 downto 4);
            else
                s_u28 <= to_unsigned(512, 10); s_v28 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(25);
            -- chroma box B: stage 29 sums, stage 30 output (centred 8 compose samples back)
            s_cbu(0) <= s_u28; s_cbv(0) <= s_v28;
            for i in 1 to 7 loop s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bbu <= to_unsigned(512 * 8, 13); s_bbv <= to_unsigned(512 * 8, 13);
            else
                s_bbu <= s_bbu + s_u28 - s_cbu(7);
                s_bbv <= s_bbv + s_v28 - s_cbv(7);
            end if;
            s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bbu, 14) + s_bbud; s_u30 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v30 <= v_sc(13 downto 4);
            -- luma [1 2 1] / 4 on the same centre: s_ly(k) holds the compose luma of the pixel k - 2 ahead
            s_ly(0) <= s_y26;
            for i in 1 to 11 loop s_ly(i) <= s_ly(i - 1); end loop;
            v_yl := resize(s_ly(9), 12) + shift_left(resize(s_ly(10), 12), 1) + s_ly(11);
            s_y30 <= v_yl(11 downto 2);
            -- 31: luma 13/16; 32: a register (one latency, a multiple of 4)
            if s_y30 > 64 then v_dl := s_y30 - 64; else v_dl := (others => '0'); end if;
            s_y31 <= s_y30 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u31 <= s_u30; s_v31 <= s_v30;
            s_y32 <= s_y31; s_u32 <= s_u31; s_v32 <= s_v31;
            -- 33: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y33 <= s_y32; s_u33 <= s_u32; s_v33 <= s_v32;
            else
                s_y33 <= to_unsigned(64, 10); s_u33 <= to_unsigned(512, 10); s_v33 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y33);
    data_out.u       <= std_logic_vector(s_u33);
    data_out.v       <= std_logic_vector(s_v33);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture gable;
