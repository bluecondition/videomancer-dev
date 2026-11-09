-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Staircase v0.1.0 -- Heirloom (video clips), from the clip WIN_20230318_14_13_38 at 0.5 s.
--
-- The still: a digital pattern on a coarse cell grid (37.55 virtual px x 30.14 lines, 51 x 36 cells).
-- A cyan / grey checker of bricks 4 cells x 1 row fills the ground; diagonal chains of identical
-- 10 x 10-cell stair glyphs climb up-right across it (8 cells right per 8 rows, chains 32 cells apart).
-- Each glyph: two black rails ending in a black / cyan X, dark-grey caps, silver feet, cyan between the
-- rails, a magenta / yellow / white tread staircase on its left edge, yellow / white rungs on its right.
-- In the clip the picture is static except for single-frame "branch" glitches: below mid-frame the chains
-- carry a second copy 8 cells to the right (the clip's first frame holds that state).
--
-- The patch, as modelled (staircase.py has the tables):
--   cells   cx = floor(X0 + (vx - 960) / CW), cy = floor(Y0 + (vy - 540) / CH)   (cell DDAs, virtual coords)
--   bricks  cyan when floor((cx - 23) / L) and cy have equal parity, else grey
--   glyphs  group m starts at row 17 + Q m - ro, its glyph column 0 sits at 11 - G m + co (mod P);
--           each cell reads the glyph from its own group (A) and from the group above (B, rows Q..15)
--   family 2  Cross: the same lattice mirrored about the centre; Branch: a copy P/4 to the right below
--           the branch row (held, or flickering one or two fields at a time as in the clip)
--   camera  a per-cell sample (one cell row late) shifts the glyph columns by V * E and walks the brick's
--           colour code along C g M Y W S K D by V * ELV (the Osc lattice feeds the same path, 4x / 2x)
--
--   K1  Cells    cell size x1/3 .. x3 (the clip's dividers; 512 = the still)
--   K2  Bricks   brick length 1 .. 16 cells (4 = the still)
--   K3  Lean     cells each glyph steps right per group: -8 .. 23 (8 = the still)
--   K4  Climb    bipolar: the glyphs climb up (or down) the chains, ticking a row at a time
--   K5  Rise     rows per group: 5 .. 20 (8 = the still)
--   K6  Etch     how far the camera (or the Osc) moves the glyphs and recolours the bricks
--   S7  Branch   Flicker (single-frame branches, as in the clip) / Held (the clip's first frame)
--   S8  Bricks   Checker (the still) / Mosaic (random runs of all eight codes, the alt clip)
--   S9  Cross    a mirrored second family of chains crossing the first
--   S10 Colours  Cyan Grey (the still) / Pink Teal (the alt clip's palette)
--   S11 Source   Video / Osc
--   P12 STAIRS   chain spacing 128 .. 11 cells: one lone staircase .. a packed weave (578 = the still)
--
-- Engine: vblank-zone sequencer (one serial multiplier, one divmod loop, shared glide), a per-line engine
-- kicked at each line start (computes the NEXT line's group row, column phases and flags), a cell DDA with
-- seven modulo counters, a 512 x 8 cell-sample RAM (ping-pong by cell-row parity), four glyph ROM reads,
-- a 21-stage pixel path.  Generated avid; output conditioning (Y, U and V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16) before the blanking gate; colours in the hardware convention.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.staircase_pkg.all;
use work.all;

architecture staircase of program_top is
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
    function f_xs16(x : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := x xor shift_left(x, 7);
        v := v xor shift_right(v, 9);
        v := v xor shift_left(v, 8);
        return v;
    end function;

    -- cell-sample RAM: 2 banks (cell-row parity) x 256 cells; bank cy(0) written on a cell row's first line,
    -- bank not cy(0) read: the two never meet
    type t_cr is array (0 to 511) of unsigned(7 downto 0);
    signal cr_ram : t_cr := (others => x"00");
    attribute no_rw_check : boolean;
    attribute no_rw_check of cr_ram : signal is true;
    signal cr_q  : unsigned(7 downto 0) := (others => '0');
    signal cr_ra, cr_wa : unsigned(8 downto 0);
    signal cr_we : std_logic;

    -- ROMs (registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_ga, r_gb, r_g2a, r_g2b : std_logic_vector(4 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_yd1, s_yd2, s_yd3 : unsigned(9 downto 0) := (others => '0');
    signal s_ysum : unsigned(11 downto 0) := (others => '0');
    signal s_vcam, s_vin : unsigned(7 downto 0) := (others => '0');

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

    -- knobs: glided (Q3), raw climb speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(448 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(448 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(224 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(578 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_br, s_mos, s_cross, s_pal, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- field serial multiplier (slot k loads at zc = 32 + 24 k; result stable from load + 17)
    signal f_a   : unsigned(33 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(33 downto 0) := (others => '0');
    signal z_fa  : unsigned(33 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';

    -- per-field scratch and constants
    signal z_kd   : unsigned(9 downto 0) := (others => '0');
    signal z_kn   : std_logic := '0';
    signal z_x    : signed(10 downto 0) := (others => '0');
    signal z_xx   : unsigned(10 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_ip   : unsigned(2 downto 0) := (others => '0');
    signal z_fxa, z_fya : unsigned(27 downto 0) := (others => '0');
    signal z_fxb  : unsigned(21 downto 0) := (others => '0');
    signal z_sq   : unsigned(9 downto 0) := (others => '0');
    signal z_psq  : unsigned(17 downto 0) := (others => '0');
    signal z_e13  : unsigned(21 downto 0) := (others => '0');
    signal z_e4, z_em : unsigned(12 downto 0) := (others => '0');
    signal z_gn   : std_logic := '0';
    signal z_ga   : unsigned(4 downto 0) := (others => '0');
    signal z_rg   : signed(10 downto 0) := (others => '0');
    signal c_kxh  : unsigned(15 downto 0) := to_unsigned(13962, 16);
    signal c_ky   : unsigned(16 downto 0) := to_unsigned(34788, 17);
    signal c_kpx  : unsigned(17 downto 0) := to_unsigned(27924, 18);
    signal c_dfy  : unsigned(18 downto 0) := to_unsigned(34788, 19);
    signal c_fx0  : unsigned(27 downto 0) := (others => '0');
    signal c_f1   : unsigned(19 downto 0) := (others => '0');   -- fraction of pixel 1's fx
    signal c_c1   : std_logic := '0';                            -- pixel 1 starts a new cell
    signal c_fy0  : unsigned(27 downto 0) := (others => '0');
    signal c_p    : unsigned(7 downto 0) := to_unsigned(32, 8);
    signal c_pm1  : unsigned(7 downto 0) := to_unsigned(31, 8);
    signal c_d    : unsigned(7 downto 0) := to_unsigned(8, 8);
    signal c_g    : signed(5 downto 0) := to_signed(8, 6);
    signal c_gp   : unsigned(7 downto 0) := to_unsigned(8, 8);    -- G mod P
    signal c_q    : unsigned(4 downto 0) := to_unsigned(8, 5);
    signal c_qm1  : unsigned(4 downto 0) := to_unsigned(7, 5);
    signal c_l    : unsigned(4 downto 0) := to_unsigned(4, 5);
    signal c_lm1  : unsigned(4 downto 0) := to_unsigned(3, 5);
    signal c_thr  : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal c_e    : unsigned(10 downto 0) := (others => '0');
    signal c_elv  : unsigned(7 downto 0) := (others => '0');
    signal c_inc  : unsigned(13 downto 0) := (others => '0');
    signal c_co   : signed(5 downto 0) := (others => '0');
    signal c_base1, c_base2 : unsigned(7 downto 0) := (others => '0');
    signal c_bpos : unsigned(4 downto 0) := (others => '0');
    signal c_bpar : std_logic := '0';
    signal c_lr0  : unsigned(4 downto 0) := (others => '0');
    signal c_gm0  : unsigned(7 downto 0) := (others => '0');
    signal c_z    : std_logic := '0';
    signal c_gated, c_f2on, c_f2over, c_up : std_logic := '0';
    signal c_brow : unsigned(7 downto 0) := to_unsigned(80, 8);
    signal z_v    : signed(11 downto 0) := (others => '0');

    -- divmod loop: v -> v mod M (and the floor quotient; gm += / -= Gp mod P alongside)
    signal d_v    : signed(11 downto 0) := (others => '0');
    signal d_m    : unsigned(7 downto 0) := to_unsigned(1, 8);
    signal d_q    : signed(8 downto 0) := (others => '0');
    signal d_gm   : unsigned(7 downto 0) := (others => '0');

    -- climb and branch flicker (per field)
    signal fc_ph  : unsigned(15 downto 0) := (others => '0');
    signal fc_ro  : unsigned(4 downto 0) := (others => '0');
    signal fl_r   : unsigned(15 downto 0) := x"ACE1";
    signal fl_cnt : unsigned(3 downto 0) := (others => '0');
    signal fl_row : unsigned(2 downto 0) := (others => '0');
    signal fl_on  : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine (computes the NEXT line)
    signal e_lc   : unsigned(4 downto 0) := to_unsigned(31, 5);
    signal e_kz, e_step, e_new, e_wrap : std_logic := '0';
    signal e_fy   : unsigned(27 downto 0) := (others => '0');
    signal e_cy   : unsigned(7 downto 0) := (others => '0');
    signal e_lr   : unsigned(4 downto 0) := (others => '0');
    signal e_gm   : unsigned(7 downto 0) := (others => '0');
    signal e_k    : unsigned(1 downto 0) := (others => '0');
    signal e_ta, e_t2 : unsigned(7 downto 0) := (others => '0');
    signal e_tb, e_t2b, e_tay, e_tby : signed(8 downto 0) := (others => '0');
    signal e_sd   : unsigned(15 downto 0) := (others => '0');
    signal n_lca, n_lcb, n_l2a, n_l2b : unsigned(6 downto 0) := (others => '0');
    signal n_lr   : unsigned(4 downto 0) := (others => '0');
    signal n_lrq  : unsigned(5 downto 0) := (others => '0');
    signal n_enb, n_top, n_g2, n_par, n_wfl : std_logic := '0';
    signal n_sd   : unsigned(15 downto 0) := x"0001";

    -- per-line values in the pixel path (loaded at g_start)
    signal l_lr   : unsigned(4 downto 0) := (others => '0');
    signal l_lrq  : unsigned(5 downto 0) := (others => '0');
    signal l_enb, l_top, l_g2, l_par, l_wfl : std_logic := '0';

    -- pixel path
    type t_u7 is array (natural range <>) of unsigned(6 downto 0);
    type t_u3 is array (natural range <>) of unsigned(2 downto 0);
    signal p1_frac : unsigned(19 downto 0) := (others => '0');   -- fraction of the NEXT pixel's fx
    signal p1_nc   : std_logic := '0';                           -- the next pixel starts a new cell
    signal p1_cx   : unsigned(7 downto 0) := (others => '0');
    signal p1_lca, p1_lcb, p1_l2a, p1_l2b : unsigned(6 downto 0) := (others => '0');
    signal p1_bpos : unsigned(4 downto 0) := (others => '0');
    signal p1_bpar, p1_cs : std_logic := '0';
    signal p1_lf, p1_lfnx : unsigned(15 downto 0) := x"0001";
    signal p1_mc   : unsigned(2 downto 0) := (others => '0');
    signal d_lca, d_lcb, d_l2a, d_l2b : t_u7(2 to 5) := (others => (others => '0'));
    signal d_bk    : t_u3(2 to 10) := (others => (others => '0'));     -- brick code (checker or mosaic)
    signal d_lv    : t_u3(5 to 10) := (others => (others => '0'));
    signal p3_v    : unsigned(7 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(14 downto 0) := (others => '0');
    signal p4_la, p4_lb : unsigned(11 downto 0) := (others => '0');
    signal p5_s    : unsigned(6 downto 0) := (others => '0');
    signal p6_ta, p6_tb, p6_t2a, p6_t2b : unsigned(7 downto 0) := (others => '0');
    signal p7_ta, p7_tb, p7_t2a, p7_t2b : unsigned(7 downto 0) := (others => '0');
    signal p7_ua, p7_ub, p7_u2a, p7_u2b : signed(8 downto 0) := (others => '0');
    signal p8_aa, p8_ab, p8_a2a, p8_a2b : unsigned(7 downto 0) := (others => '0');
    signal p8_v, p9_v, p10_v : std_logic_vector(3 downto 0) := (others => '0');
    signal p10_ga, p10_gb, p10_g2a, p10_g2b : std_logic_vector(4 downto 0) := (others => '0');
    signal p11_o   : std_logic_vector(3 downto 0) := (others => '0');
    signal p11_ca, p11_cb, p11_c2a, p11_c2b, p11_bk : unsigned(2 downto 0) := (others => '0');
    signal p12_code : unsigned(2 downto 0) := (others => '0');
    signal p13_w, p14_w : std_logic_vector(29 downto 0) := (others => '0');
    signal s_y15 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cby : t_u10(0 to 7) := (others => to_unsigned(64, 10));
    signal s_cau, s_cav, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bay, s_bayd, s_bby, s_bbyd : unsigned(12 downto 0) := to_unsigned(512, 13);
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bbu, s_bbv, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y17, s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17, s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Cell-sample RAM (collision-free: written in bank cy(0), read in the other) and ROMs
    ---------------------------------------------------------------------------
    cr_ra <= (not l_par) & p1_cx;
    cr_wa <= l_par & p1_cx;
    cr_we <= l_wfl and p1_cs and g_avid;
    p_cr : process(clk)
    begin
        if rising_edge(clk) then
            cr_q <= cr_ram(to_integer(cr_ra));
            if cr_we = '1' then cr_ram(to_integer(cr_wa)) <= s_vin; end if;
        end if;
    end process p_cr;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_SC_EXP(to_integer(r_ex_a));
            r_ga   <= C_SC_GLYPH(to_integer(p8_aa));
            r_gb   <= C_SC_GLYPH(to_integer(p8_ab));
            r_g2a  <= C_SC_GLYPH(to_integer(p8_a2a));
            r_g2b  <= C_SC_GLYPH(to_integer(p8_a2b));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_t  : unsigned(16 downto 0);
        variable v_c  : unsigned(16 downto 0);
        variable v_g  : signed(10 downto 0);
        variable v_p  : unsigned(16 downto 0);
        variable v_s20 : unsigned(20 downto 0);
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

            -- climb speed: bipolar, +-8 deadband
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
                s_br    <= registers_in(6)(0);
                s_mos   <= registers_in(6)(1);
                s_cross <= registers_in(6)(2);
                s_pal   <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
                -- climb: a 16-bit phase per field; each wrap ticks the row offset (mod Q)
                if s_vm /= 0 then
                    if s_vneg = '0' then
                        v_c := ('0' & fc_ph) + resize(c_inc, 17);
                        if v_c(16) = '1' then
                            if fc_ro >= c_qm1 then fc_ro <= (others => '0'); else fc_ro <= fc_ro + 1; end if;
                        end if;
                    else
                        v_c := ('0' & fc_ph) - resize(c_inc, 17);
                        if v_c(16) = '1' then
                            if fc_ro = 0 or fc_ro > c_qm1 then fc_ro <= c_qm1; else fc_ro <= fc_ro - 1; end if;
                        end if;
                    end if;
                    fc_ph <= v_c(15 downto 0);
                end if;
                -- branch flicker: a burst on ~1 field in 256, 7 field pairs, each pair lit with p = 3/8
                fl_r <= f_xs16(fl_r);
                if fl_cnt = 0 then
                    fl_on <= '0';
                    if fl_r(7 downto 0) = x"00" then
                        fl_cnt <= to_unsigned(14, 4); fl_row <= fl_r(10 downto 8);
                    end if;
                else
                    fl_cnt <= fl_cnt - 1;
                    if fl_cnt(0) = '0' then
                        if fl_r(12 downto 10) < 3 then fl_on <= '1'; else fl_on <= '0'; end if;
                    end if;
                end if;
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

            -- serial products: slot k loads at zc = 32 + 24 k (decode at 30 + 24 k, operands at 31 + 24 k)
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
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(z_kd, 34);               z_fb <= to_unsigned(406, 16);    -- |K1 - 512| * 406
                    when 1  => z_fa <= to_unsigned(13962, 34);         z_fb <= z_mt;                    -- kxh
                    when 2  => z_fa <= to_unsigned(34788, 34);         z_fb <= z_mt;                    -- ky
                    when 3  => z_fa <= resize(c_ky, 34);               z_fb <= resize(s_ys, 16);        -- dfy
                    when 4  => z_fa <= resize(c_kxh, 34);              z_fb <= to_unsigned(1920, 16) - resize(s_x0, 16);
                    when 5  => z_fa <= resize(c_ky, 34);               z_fb <= to_unsigned(540, 16);
                    when 6  => z_fa <= resize(s_gp(12 downto 3), 34);  z_fb <= to_unsigned(907, 16);    -- P12 exponent
                    when 7  => z_fa <= resize(s_g6(12 downto 3), 34);  z_fb <= resize(s_g6(12 downto 3), 16);
                    when 8  => z_fa <= resize(z_sq, 34);               z_fb <= resize(c_p, 16);
                    when 9  => z_fa <= resize(s_vm, 34);               z_fb <= resize(s_vm, 16);        -- climb^2
                    when others => z_fa <= resize(fc_ro, 34);          z_fb <= resize(z_ga, 16);        -- ro * |G|
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(32 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- divmod loop: one step per clock (results read 30+ clocks after each load)
            if d_v < 0 then
                d_v <= d_v + signed(resize(d_m, 12));
                d_q <= d_q - 1;
                if d_gm < c_gp then d_gm <= d_gm + c_p - c_gp; else d_gm <= d_gm - c_gp; end if;
            elsif d_v >= signed(resize(d_m, 12)) then
                d_v <= d_v - signed(resize(d_m, 12));
                d_q <= d_q + 1;
                if d_gm + c_gp >= c_p then d_gm <= d_gm + c_gp - c_p; else d_gm <= d_gm + c_gp; end if;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 4 =>
                        c_gated <= s_br or fl_on;
                        if s_br = '1' then c_brow <= to_unsigned(80, 8);
                        else c_brow <= to_unsigned(77, 8) + resize(fl_row, 8); end if;
                    when 5 =>
                        c_f2on <= s_cross or c_gated; c_f2over <= c_gated; c_up <= not s_cross;
                        s_ot <= s_ot + 1;
                    when 20 =>                    -- K1: d = |g1 - 512|
                        v_g := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        if v_g < 0 then v_g := -v_g; z_kn <= '1'; else z_kn <= '0'; end if;
                        z_kd <= unsigned(v_g(9 downto 0));
                    when 50 =>                    -- e1 = +-(d * 406 >> 9) (Q8 octaves); x = -e1
                        if z_kn = '1' then z_x <= signed(resize(f_acc(17 downto 9), 11));
                        else z_x <= -signed(resize(f_acc(17 downto 9), 11)); end if;
                    when 51 => r_ex_a <= unsigned(z_x(7 downto 0));
                    when 53 => z_mt <= r_ex_q;
                    when 73 =>                    -- kxh = 13962 * mt >> (15 - ix), ix = x >> 8 = -2 .. 1
                        case to_integer(z_x(10 downto 8)) is
                            when -2     => c_kxh <= f_acc(32 downto 17);
                            when -1     => c_kxh <= f_acc(31 downto 16);
                            when 0      => c_kxh <= f_acc(30 downto 15);
                            when others => c_kxh <= f_acc(29 downto 14);
                        end case;
                    when 97 =>
                        case to_integer(z_x(10 downto 8)) is
                            when -2     => c_ky <= f_acc(33 downto 17);
                            when -1     => c_ky <= f_acc(32 downto 16);
                            when 0      => c_ky <= f_acc(31 downto 15);
                            when others => c_ky <= f_acc(30 downto 14);
                        end case;
                    when 98 =>
                        case to_integer(s_xs) is
                            when 4      => c_kpx <= shift_left(resize(c_kxh, 18), 2);
                            when 3      => c_kpx <= shift_left(resize(c_kxh, 18), 1) + resize(c_kxh, 18);
                            when others => c_kpx <= shift_left(resize(c_kxh, 18), 1);
                        end case;
                    when 121 => c_dfy <= f_acc(26 downto 8);
                    when 145 => z_fxa <= to_unsigned(C_SC_FX0C, 28) - f_acc(27 downto 0);
                    when 146 => z_fxb <= resize(shift_right(c_kpx, 1), 22) + shift_left(resize(c_kpx, 22), 3);
                    when 147 => z_fxb <= z_fxb + shift_left(resize(c_kpx, 22), 2);
                    when 148 => c_fx0 <= z_fxa + resize(z_fxb, 28);          -- + half a pixel + 12 px (boxes, generated-avid lag)
                    when 149 =>
                        v_s20 := ('0' & c_fx0(19 downto 0)) + resize(c_kpx, 21);
                        c_f1 <= v_s20(19 downto 0); c_c1 <= v_s20(20);
                    when 169 => z_fya <= to_unsigned(C_SC_FY0C, 28) - f_acc(27 downto 0);
                    when 170 => c_fy0 <= z_fya + resize(shift_right(c_dfy, 1), 28);
                    when 193 =>                   -- P = 2^(7 - ep), ep = gp * 907 >> 10 (Q8 octaves)
                        z_xx <= to_unsigned(1792, 11) - resize(f_acc(19 downto 10), 11);
                    when 194 => r_ex_a <= z_xx(7 downto 0); z_ip <= z_xx(10 downto 8);
                    when 196 => z_mt <= r_ex_q;
                    when 197 =>
                        case to_integer(z_ip) is
                            when 3      => v_p := shift_right(resize(z_mt, 17) + 2048, 12);
                            when 4      => v_p := shift_right(resize(z_mt, 17) + 1024, 11);
                            when 5      => v_p := shift_right(resize(z_mt, 17) + 512, 10);
                            when 6      => v_p := shift_right(resize(z_mt, 17) + 256, 9);
                            when others => v_p := shift_right(resize(z_mt, 17) + 128, 8);
                        end case;
                        c_p <= v_p(7 downto 0);
                    when 198 =>
                        c_pm1 <= c_p - 1;
                        if c_p < 8 then c_d <= to_unsigned(2, 8); else c_d <= shift_right(c_p, 2); end if;
                    when 217 => z_sq <= f_acc(19 downto 10);
                    when 241 => z_psq <= f_acc(17 downto 0);
                    when 242 => z_e13 <= shift_left(resize(z_psq, 22), 3) + shift_left(resize(z_psq, 22), 2);
                    when 243 => z_e13 <= z_e13 + resize(z_psq, 22);
                    when 244 =>
                        c_e <= z_e13(20 downto 10);
                        if s_src = '1' then c_elv <= resize(s_g6(12 downto 5), 8);
                        else c_elv <= resize(s_g6(12 downto 6), 8); end if;
                        z_e4 <= shift_left(resize(z_e13(20 downto 10), 13), 2);
                        z_em <= shift_left(resize(c_pm1, 13), 4);
                    when 245 =>
                        if s_src = '1' then
                            if z_e4 < z_em then c_e <= z_e4(10 downto 0); else c_e <= z_em(10 downto 0); end if;
                        end if;
                    when 265 => c_inc <= f_acc(17 downto 4);
                    when 266 =>
                        v_g := signed(resize(s_g3(12 downto 3), 11)) - 512;
                        c_g <= resize(shift_right(v_g, 5), 6) + 8;
                        c_q <= C_SC_LUTQ(to_integer(s_g5(12 downto 9)));
                        c_l <= C_SC_LUTL(to_integer(s_g2(12 downto 9)));
                        c_thr <= C_SC_LUTT(to_integer(s_g2(12 downto 9)));
                    when 267 =>
                        c_qm1 <= c_q - 1; c_lm1 <= c_l - 1;
                        if fc_ro >= c_q then fc_ro <= (others => '0'); end if;
                        if c_g < 0 then v_g := -resize(c_g, 11); z_gn <= '1'; else v_g := resize(c_g, 11); z_gn <= '0'; end if;
                        z_ga <= unsigned(v_g(4 downto 0));
                    when 289 =>                   -- ro * G (signed)
                        if z_gn = '1' then z_rg <= -signed(resize(f_acc(9 downto 0), 11));
                        else z_rg <= signed(resize(f_acc(9 downto 0), 11)); end if;
                    -- divmod jobs
                    when 300 => d_v <= resize(c_g, 12); d_m <= c_p; d_q <= (others => '0');              -- G mod P
                    when 335 => c_gp <= unsigned(d_v(7 downto 0));
                                z_v <= resize(z_rg, 12) + signed(resize(shift_left(resize(c_q, 9), 3), 12));
                    when 340 => d_v <= z_v; d_m <= resize(c_q, 8); d_q <= (others => '0');             -- floor(ro G / Q)
                    when 380 => c_co <= resize(d_q - 8, 6);
                    when 381 => z_v <= signed(resize(c_fx0(27 downto 20), 12)) - 139 - resize(c_co, 12);
                    when 385 => d_v <= z_v; d_m <= c_p;                                                -- base1
                    when 420 => c_base1 <= unsigned(d_v(7 downto 0));
                                z_v <= 168 - signed(resize(c_fx0(27 downto 20), 12)) - resize(c_co, 12);
                    when 425 => d_v <= z_v; d_m <= c_p;                                                -- base2
                    when 460 => c_base2 <= unsigned(d_v(7 downto 0));
                                z_v <= signed(resize(c_fx0(27 downto 20), 12)) - 151;
                    when 465 => d_v <= z_v; d_m <= shift_left(resize(c_l, 8), 1);                     -- brick
                    when 560 =>
                        if d_v >= signed(resize(c_l, 12)) then
                            c_bpos <= resize(unsigned(d_v(5 downto 0)) - resize(c_l, 6), 5); c_bpar <= '1';
                        else
                            c_bpos <= unsigned(d_v(4 downto 0)); c_bpar <= '0';
                        end if;
                        z_v <= signed(resize(c_fy0(27 downto 20), 12)) - 81 + signed(resize(fc_ro, 12));
                    when 565 => d_v <= z_v; d_m <= resize(c_q, 8); d_gm <= (others => '0');           -- group / row
                    when 600 =>
                        c_lr0 <= unsigned(d_v(4 downto 0)); c_gm0 <= d_gm;
                        if d_v /= 0 then c_z <= '1'; else c_z <= '0'; end if;
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
    -- Source side: oscillator (2-D triangle lattice, drifting) and the camera's
    -- 4-pixel luma average, (Y >> 2) - 16 scaled to 0..252
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_c : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;  -- ot * 96
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s;
            s_yd1 <= unsigned(data_in.y); s_yd2 <= s_yd1; s_yd3 <= s_yd2;
            s_ysum <= resize(unsigned(data_in.y), 12) + s_yd1 + s_yd2 + s_yd3;
            v_l := s_ysum(11 downto 4);
            if v_l > 16 then v_l := v_l - 16; else v_l := (others => '0'); end if;
            v_c := resize(v_l, 9) + shift_right(v_l, 3) + shift_right(v_l, 5);
            if v_c(8) = '1' then s_vcam <= (others => '1'); else s_vcam <= v_c(7 downto 0); end if;
            if s_src = '1' then s_vin <= s_osum; else s_vin <= s_vcam; end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0,
    -- without a step), computes the NEXT line: the cell row, the group row and
    -- column phase, the four glyph column starts, the flags and the mosaic seed
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_cy : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 620 then
                e_fy <= c_fy0; e_cy <= c_fy0(27 downto 20); e_lr <= c_lr0; e_gm <= c_gm0; e_k <= "00";
            end if;
            e_kz <= '0';
            if s_zone = '1' and s_zc = 630 then e_kz <= '1'; end if;

            if e_kz = '1' then
                e_lc <= to_unsigned(1, 5); e_step <= '0';
            elsif g_start = '1' then
                e_lc <= to_unsigned(1, 5); e_step <= '1';
            elsif e_lc /= 31 then
                e_lc <= e_lc + 1;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    if e_step = '1' then e_fy <= e_fy + resize(c_dfy, 28); end if;
                when 2 =>
                    v_cy := e_fy(27 downto 20);
                    if e_step = '0' or v_cy /= e_cy then e_new <= '1'; else e_new <= '0'; end if;
                    e_cy <= v_cy;
                when 3 =>
                    e_wrap <= '0';
                    if e_step = '1' and e_new = '1' then
                        if e_lr >= c_qm1 then e_lr <= (others => '0'); e_wrap <= '1'; else e_lr <= e_lr + 1; end if;
                    end if;
                when 4 =>
                    if e_wrap = '1' then
                        e_gm <= e_gm + c_gp;
                        if e_k /= 3 then e_k <= e_k + 1; end if;
                    end if;
                when 5 =>
                    if e_gm >= c_p then e_gm <= e_gm - c_p; end if;
                when 6 =>
                    e_ta <= c_base1 + e_gm; e_t2 <= c_base2 + e_gm;
                when 7 =>
                    if e_ta >= c_p then e_ta <= e_ta - c_p; end if;
                    if e_t2 >= c_p then e_t2 <= e_t2 - c_p; end if;
                when 8 =>
                    e_tb  <= signed('0' & e_ta) - signed('0' & c_gp);
                    e_t2b <= signed('0' & e_t2) - signed('0' & c_gp);
                    e_tay <= signed('0' & e_ta) - signed('0' & c_d);
                when 9 =>
                    if e_tb < 0 then e_tb <= e_tb + signed('0' & c_p); end if;
                    if e_t2b < 0 then e_t2b <= e_t2b + signed('0' & c_p); end if;
                    if e_tay < 0 then e_tay <= e_tay + signed('0' & c_p); end if;
                when 10 =>
                    e_tby <= e_tb - signed('0' & c_d);
                when 11 =>
                    if e_tby < 0 then e_tby <= e_tby + signed('0' & c_p); end if;
                when 12 =>
                    n_lca <= e_ta(6 downto 0); n_lcb <= unsigned(e_tb(6 downto 0));
                    if s_cross = '1' then
                        n_l2a <= e_t2(6 downto 0); n_l2b <= unsigned(e_t2b(6 downto 0));
                    else
                        n_l2a <= unsigned(e_tay(6 downto 0)); n_l2b <= unsigned(e_tby(6 downto 0));
                    end if;
                    if c_z = '1' then
                        if e_k >= 2 then n_enb <= '1'; else n_enb <= '0'; end if;
                        if e_k = 0 then n_top <= '1'; else n_top <= '0'; end if;
                    else
                        if e_k >= 1 then n_enb <= '1'; else n_enb <= '0'; end if;
                        n_top <= '0';
                    end if;
                    if c_f2on = '1' and (c_gated = '0' or e_cy >= c_brow) then n_g2 <= '1'; else n_g2 <= '0'; end if;
                    n_par <= e_cy(0); n_wfl <= e_new; n_lr <= e_lr; n_lrq <= resize(e_lr, 6) + resize(c_q, 6);
                    e_sd <= (e_cy & e_cy) + to_unsigned(16#3C6E# + 16#2B71#, 16);
                when 13 | 14 | 15 | 16 =>
                    e_sd <= f_xs16(e_sd);
                when 17 =>
                    if e_sd = 0 then n_sd <= x"0001"; else n_sd <= e_sd; end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = the cell DDA and its counters, loaded at g_start)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_sum : unsigned(20 downto 0);
        variable v_s   : unsigned(18 downto 0);
        variable v_l   : unsigned(15 downto 0);
        variable v_lc  : unsigned(7 downto 0);
        variable v_ok  : std_logic;
        variable v_bk  : unsigned(2 downto 0);
        variable v_f1, v_f2 : unsigned(2 downto 0);
        variable v_h1, v_h2 : std_logic;
        variable v_pw  : std_logic_vector(29 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: cell DDA (cx = fx >> 20) and the counters that follow it
            p1_lfnx <= f_xs16(p1_lf);
            if g_start = '1' then
                p1_frac <= c_f1; p1_nc <= c_c1; p1_cx <= c_fx0(27 downto 20);
                p1_lca <= n_lca; p1_lcb <= n_lcb; p1_l2a <= n_l2a; p1_l2b <= n_l2b;
                p1_bpos <= c_bpos; p1_bpar <= c_bpar;
                p1_lf <= n_sd; p1_mc <= C_SC_MOS(to_integer(n_sd(11 downto 8)));
                p1_cs <= '1';
                l_lr <= n_lr; l_lrq <= n_lrq; l_enb <= n_enb; l_top <= n_top; l_g2 <= n_g2;
                l_par <= n_par; l_wfl <= n_wfl;
            elsif g_avid = '1' then
                -- the accumulator runs one pixel ahead, so the counters only see a registered carry
                v_sum := ('0' & p1_frac) + resize(c_kpx, 21);
                p1_frac <= v_sum(19 downto 0);
                p1_nc <= v_sum(20);
                p1_cs <= p1_nc;
                if p1_nc = '1' then
                    p1_cx <= p1_cx + 1;
                    if resize(p1_lca, 8) = c_pm1 then p1_lca <= (others => '0'); else p1_lca <= p1_lca + 1; end if;
                    if resize(p1_lcb, 8) = c_pm1 then p1_lcb <= (others => '0'); else p1_lcb <= p1_lcb + 1; end if;
                    if c_up = '1' then
                        if resize(p1_l2a, 8) = c_pm1 then p1_l2a <= (others => '0'); else p1_l2a <= p1_l2a + 1; end if;
                        if resize(p1_l2b, 8) = c_pm1 then p1_l2b <= (others => '0'); else p1_l2b <= p1_l2b + 1; end if;
                    else
                        if p1_l2a = 0 then p1_l2a <= c_pm1(6 downto 0); else p1_l2a <= p1_l2a - 1; end if;
                        if p1_l2b = 0 then p1_l2b <= c_pm1(6 downto 0); else p1_l2b <= p1_l2b - 1; end if;
                    end if;
                    if p1_bpos = c_lm1 then p1_bpos <= (others => '0'); p1_bpar <= not p1_bpar;
                    else p1_bpos <= p1_bpos + 1; end if;
                    p1_lf <= p1_lfnx;
                    if ('0' & p1_lfnx(7 downto 0)) < c_thr then p1_mc <= C_SC_MOS(to_integer(p1_lfnx(11 downto 8))); end if;
                end if;
            end if;
            -- brick code of the stage-1 pixel (checker: cyan when the brick parity equals the row parity)
            if s_mos = '1' then d_bk(2) <= p1_mc;
            elsif p1_bpar = l_par then d_bk(2) <= "000";
            else d_bk(2) <= "001"; end if;
            for i in 3 to 10 loop d_bk(i) <= d_bk(i - 1); end loop;
            -- 2: (cell-sample RAM read), counters delayed to stage 5
            d_lca(2) <= p1_lca; d_lcb(2) <= p1_lcb; d_l2a(2) <= p1_l2a; d_l2b(2) <= p1_l2b;
            for i in 3 to 5 loop
                d_lca(i) <= d_lca(i - 1); d_lcb(i) <= d_lcb(i - 1); d_l2a(i) <= d_l2a(i - 1); d_l2b(i) <= d_l2b(i - 1);
            end loop;
            -- 3: fabric copy of the sample
            p3_v <= cr_q;
            -- 4: nibble products (V * E, V * ELV)
            p4_ea <= resize(p3_v(3 downto 0) * c_e, 15);
            p4_eb <= resize(p3_v(7 downto 4) * c_e, 15);
            p4_la <= resize(p3_v(3 downto 0) * c_elv, 12);
            p4_lb <= resize(p3_v(7 downto 4) * c_elv, 12);
            -- 5: shift S = V E >> 12 (< P) and level LV = V ELV >> 12 (mod 8)
            v_s := resize(p4_ea, 19) + shift_left(resize(p4_eb, 19), 4);
            p5_s <= v_s(18 downto 12);
            v_l := resize(p4_la, 16) + shift_left(resize(p4_lb, 16), 4);
            d_lv(5) <= v_l(14 downto 12);
            for i in 6 to 10 loop d_lv(i) <= d_lv(i - 1); end loop;
            -- 6: column + shift
            p6_ta  <= resize(d_lca(5), 8) + resize(p5_s, 8);
            p6_tb  <= resize(d_lcb(5), 8) + resize(p5_s, 8);
            p6_t2a <= resize(d_l2a(5), 8) + resize(p5_s, 8);
            p6_t2b <= resize(d_l2b(5), 8) + resize(p5_s, 8);
            -- 7: minus P
            p7_ta <= p6_ta; p7_tb <= p6_tb; p7_t2a <= p6_t2a; p7_t2b <= p6_t2b;
            p7_ua  <= signed('0' & p6_ta) - signed('0' & c_p);
            p7_ub  <= signed('0' & p6_tb) - signed('0' & c_p);
            p7_u2a <= signed('0' & p6_t2a) - signed('0' & c_p);
            p7_u2b <= signed('0' & p6_t2b) - signed('0' & c_p);
            -- 8: mod P, glyph ROM addresses {row, col} and validity
            if p7_ua(8) = '0' then v_lc := unsigned(p7_ua(7 downto 0)); else v_lc := p7_ta; end if;
            if v_lc(7 downto 4) = "0000" and l_lr(4) = '0' then v_ok := '1'; else v_ok := '0'; end if;
            p8_aa <= l_lr(3 downto 0) & v_lc(3 downto 0); p8_v(0) <= v_ok;
            if p7_ub(8) = '0' then v_lc := unsigned(p7_ub(7 downto 0)); else v_lc := p7_tb; end if;
            if v_lc(7 downto 4) = "0000" and l_lrq(5 downto 4) = "00" and l_enb = '1' then v_ok := '1'; else v_ok := '0'; end if;
            p8_ab <= l_lrq(3 downto 0) & v_lc(3 downto 0); p8_v(1) <= v_ok;
            if p7_u2a(8) = '0' then v_lc := unsigned(p7_u2a(7 downto 0)); else v_lc := p7_t2a; end if;
            if v_lc(7 downto 4) = "0000" and l_lr(4) = '0' and l_g2 = '1' then v_ok := '1'; else v_ok := '0'; end if;
            p8_a2a <= l_lr(3 downto 0) & v_lc(3 downto 0); p8_v(2) <= v_ok;
            if p7_u2b(8) = '0' then v_lc := unsigned(p7_u2b(7 downto 0)); else v_lc := p7_t2b; end if;
            if v_lc(7 downto 4) = "0000" and l_lrq(5 downto 4) = "00" and l_enb = '1' and l_g2 = '1' then v_ok := '1'; else v_ok := '0'; end if;
            p8_a2b <= l_lrq(3 downto 0) & v_lc(3 downto 0); p8_v(3) <= v_ok;
            -- 9: (glyph ROMs), 10: fabric copies
            p9_v <= p8_v;
            p10_v <= p9_v;
            p10_ga <= r_ga; p10_gb <= r_gb; p10_g2a <= r_g2a; p10_g2b <= r_g2b;
            -- 11: opacity (the top group's X cells are cut), codes, brick + camera level
            p11_o(0) <= p10_v(0) and p10_ga(4) and not (l_top and p10_ga(3));
            p11_o(1) <= p10_v(1) and p10_gb(4);
            p11_o(2) <= p10_v(2) and p10_g2a(4) and not (l_top and p10_g2a(3));
            p11_o(3) <= p10_v(3) and p10_g2b(4);
            p11_ca <= unsigned(p10_ga(2 downto 0)); p11_cb <= unsigned(p10_gb(2 downto 0));
            p11_c2a <= unsigned(p10_g2a(2 downto 0)); p11_c2b <= unsigned(p10_g2b(2 downto 0));
            p11_bk <= d_bk(10) + d_lv(10);
            -- 12: priority (family 2 over family 1 for the branch, under it for the cross)
            if p11_o(0) = '1' then v_f1 := p11_ca; v_h1 := '1';
            elsif p11_o(1) = '1' then v_f1 := p11_cb; v_h1 := '1';
            else v_f1 := p11_bk; v_h1 := '0'; end if;
            if p11_o(2) = '1' then v_f2 := p11_c2a; v_h2 := '1';
            elsif p11_o(3) = '1' then v_f2 := p11_c2b; v_h2 := '1';
            else v_f2 := p11_bk; v_h2 := '0'; end if;
            if c_f2over = '1' then
                if v_h2 = '1' then p12_code <= v_f2; else p12_code <= v_f1; end if;
            else
                if v_h1 = '1' then p12_code <= v_f1; else p12_code <= v_f2; end if;
            end if;
            -- 13: palette, 14: copy
            v_pw := C_SC_PAL(to_integer(s_pal & p12_code));
            p13_w <= v_pw;
            p14_w <= p13_w;
            -- 15: black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            else
                s_y15 <= unsigned(p14_w(29 downto 20)); s_u15 <= unsigned(p14_w(19 downto 10)); s_v15 <= unsigned(p14_w(9 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the capture's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to black once 8 blank samples are in, so a glitched sample
            -- can't offset the colours for good.
            -- box A: stage 16 sums, stage 17 output
            s_cay(0) <= s_y15; s_cau(0) <= s_u15; s_cav(0) <= s_v15;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= to_unsigned(512, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
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
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            -- box B: stage 18 sums, stage 19 output
            s_cby(0) <= s_y17; s_cbu(0) <= s_u17; s_cbv(0) <= s_v17;
            for i in 1 to 7 loop s_cby(i) <= s_cby(i - 1); s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(512, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
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

end architecture staircase;
