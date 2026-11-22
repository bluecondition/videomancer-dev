-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Gnomon v0.1.0 -- Straightedge.  A plain of square posts seen from straight above at low sun (the gnomon of a
-- sundial; de Chirico's raking shadows): every post casts a long flat parallelogram shadow, all parallel.  The
-- sun circles, so the shadows swing round together like clock hands; as they lengthen they run under the
-- neighbouring posts and lock into striped bars at the lattice's rational directions.  Every edge on the screen
-- is a straight line.
--
--   The shadow of a post of half size h swept along s = L P (cf, sf) is a hexagon: in post coordinates (u, v)
--     u in [-h, h + k L P cf],  v in [-h, h + k L P sf],  |u sf - v cf| <= h (cf + sf)
--   (the sun's direction folded into the first quadrant by mirroring x and y; k = the post's camera height).
--   A pixel tests the posts lying back along the shadow: three rows of three (four in the middle row, whose
--   posts sit half a cell over in Stagger) -- ten candidate slots.  Shadow = the union (max of the slots' edge
--   distances), the post's own top painted over it.
--
--   K1  X       lattice pan, +-1.25 cells          K2  Y   the same, vertically (up = up)
--   K3  Scale   pitch 50 .. 400 virtual px (exponential)
--   K4  Post    post half size 0.04 .. 0.45 cell: pins to blocks that almost touch
--   K5  Drift   the sun's azimuth turns; centre +-24 = still; 2^-12 .. 2^-6 turn per field; wraps at one turn
--   K6  Camera  each post's height in four steps from the held luma at its centre (bright = low, short shadow)
--   S7  Glide   Off = the knobs act this field;  On = 1/16 of the gap per field (Drift speed 1/8)
--   S8  Form    Grid / Stagger (alternate rows half a cell over)
--   S9  Tone    Grey (grey ground, black shadows, white tops) / Ink (white ground and tops, black shadows)
--   S10 Weight  Bold (filled) / Fine (the shadows' and the posts' outlines: an architect's plan)
--   S11 Source  Video / Osc (the shell's drifting triangle lattice at twice the camera level)
--   P12 Sun     the shadow length 0.15 .. 2 cells
--
-- No multiplier in the pixel path except the Fine tone blend (gated shifts).  All per-field values come from
-- Hardline flute's microcoded engine (gnomon.py: holds, glides, Drift, sin / cos, every constant); the line
-- engine derives the per-line slot constants in the blanking on one small ALU; the beam runs two accumulators
-- (the cell counter F and lx sf).
--
-- Sections (the integer model .straightedge_work/rb_gnomon/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         switches (holds, glides and the Drift speed are microcode)
--   [C] FIELD ENGINE                     Hardline flute's: holds, glides, Drift, the sun's azimuth, pitch,
--                                        size, length, start values -> the field's registers
--   [L] LINE ENGINE                      row stepping; per line the slot constants and the camera rows
--   [K] CAMERA READ                      per cell crossing the next lead column's heights (3 rows), a 4-deep
--                                        shift register per row; prefilled in the blanking
--   [P] PIXEL PATH                       beam -> 10 slots (cross, box) -> min -> max -> coverage -> tone
--   [D] SOURCE FIELD                     luma (or Osc) -> 32-px box -> vertical one-pole per block column ->
--                                        8-bit grid (32 virtual px blocks), two banks ping-ponged per field
--   [F] OUTPUT CONDITIONING              luma [1 2 1] / 4, 13/16 toward black, blanking gate; U = V = 512
--
-- Stage numbering: a register "at stage S" holds pixel n at T + S + n (T = the g_start clock).  Pixels
-- 0 .. W + 1 are rendered; output column j is the [1 2 1] filter over pixels j .. j + 2 and lands on screen
-- column j + 4, so pixel m sits at screen column m + 3.  One latency (28) in every mode.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.gnomon_pkg.all;
use work.all;

architecture gnomon of program_top is
    constant C_LAT : integer := 28;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 512;
    constant C_DEF_K4  : integer := 300;
    constant C_DEF_K5  : integer := 650;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 660;
    constant C_A0      : integer := 87381;   -- rest azimuth: 30 degrees of a 2^20 turn

    ---------------------------------------------------------------------------
    -- [A] timing-ID steps
    ---------------------------------------------------------------------------
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
    -- a + b or a - b on ONE carry chain (the low column makes the carry-in)
    function f_addsub(a, b : unsigned; sub : std_logic) return unsigned is
        variable v_a, v_b, v_r : unsigned(a'length + 1 downto 0);
    begin
        v_a := '0' & a & '1';
        if sub = '1' then v_b := '0' & (not b) & '1'; else v_b := '0' & b & '0'; end if;
        v_r := v_a + v_b;
        return v_r(a'length downto 1);
    end function;
    function f_as(a, b : signed; sub : std_logic) return signed is
        variable v_a, v_b, v_r : signed(a'length downto 0);
        variable v_o : signed(a'length - 1 downto 0);
    begin
        v_a := a & '1';
        if sub = '1' then v_b := (not b) & '1'; else v_b := b & '0'; end if;
        v_r := v_a + v_b;
        v_o := v_r(a'length downto 1);
        return v_o;
    end function;
    -- |x| as x xor sign (one's complement)
    function f_oabs(x : signed) return unsigned is
        variable v_x : signed(x'length - 1 downto 0);
        variable v_o : unsigned(x'length - 2 downto 0);
    begin
        v_x := x;
        if v_x(v_x'high) = '1' then v_o := unsigned(not v_x(v_x'high - 1 downto 0));
        else v_o := unsigned(v_x(v_x'high - 1 downto 0)); end if;
        return v_o;
    end function;
    -- a signed distance (Q3 virtual px) saturated to -64 .. 63 (8 bits): all the coverage needs
    function f_sat(x : signed) return signed is
        variable v_x : signed(x'length - 1 downto 0);
        variable v_s : signed(7 downto 0);
        variable v_hi : boolean;
        variable v_lo : boolean;
    begin
        v_x := x;
        v_hi := false; v_lo := false;
        for i in 6 to v_x'high - 1 loop
            if v_x(i) = '1' then v_hi := true; end if;
            if v_x(i) = '0' then v_lo := true; end if;
        end loop;
        if v_x(v_x'high) = '0' and v_hi then v_s := to_signed(63, 8);
        elsif v_x(v_x'high) = '1' and v_lo then v_s := to_signed(-64, 8);
        else v_s := resize(v_x(6 downto 0), 8); end if;
        return v_s;
    end function;
    function f_min8(a, b : signed(7 downto 0)) return signed is
    begin
        if a < b then return a; else return b; end if;
    end function;
    function f_max8(a, b : signed(7 downto 0)) return signed is
    begin
        if a > b then return a; else return b; end if;
    end function;
    -- coverage 0 .. 16 from a signed distance (Q3 virtual px): clamp(8 + 16 d / pixel), pixel = xs / 2 vpx
    function f_cv(s : signed(9 downto 0); xs : unsigned(2 downto 0)) return unsigned is
        variable v_t : signed(11 downto 0);
        variable v_s : signed(11 downto 0);
        variable v_o : unsigned(4 downto 0);
    begin
        v_s := resize(s, 12);
        if xs = 2 then v_t := shift_left(v_s, 1);
        elsif xs = 4 then v_t := v_s;
        else v_t := v_s + shift_right(v_s, 2) + shift_right(v_s, 4); end if;
        v_t := v_t + 8;
        if v_t(11) = '1' then v_o := (others => '0');
        elsif v_t > 16 then v_o := to_unsigned(16, 5);
        else v_o := unsigned(v_t(4 downto 0)); end if;
        return v_o;
    end function;

    -- coverage 0 .. 16 from 8 + the scaled distance (no carry chain: sign, high bits and the 5-bit window)
    function f_cl16(c : signed(11 downto 0)) return unsigned is
        variable v_o : unsigned(4 downto 0);
    begin
        if c(11) = '1' then v_o := (others => '0');
        elsif c(10 downto 5) /= "000000" or (c(4) = '1' and c(3 downto 0) /= "0000") then v_o := to_unsigned(16, 5);
        else v_o := unsigned(c(4 downto 0)); end if;
        return v_o;
    end function;

    attribute no_rw_check : boolean;
    attribute keep : boolean;

    -- [A] timing / zone / width
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

    -- [A] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');
    signal s_field_q  : std_logic := '1';

    -- [B] switches
    signal s_glide : std_logic := '1';
    signal s_stag, s_ink, s_fine, s_src : std_logic := '0';
    signal s_ish3 : std_logic := '1';                              -- the vertical one-pole's shift: 3 (else 2)
    signal s_bank : std_logic := '0';                              -- the grid bank written this field

    -- ROM: the engine's table (2^(i/256) + quarter sine)
    signal r_ex_a : unsigned(8 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');

    -- [C] the per-field engine (Hardline flute's): 37-bit accumulator, 256 x 32 register file, one adder,
    -- serial 18 x 18 multiplier, serial right shift; microprogram in block RAM
    constant OP_LD   : std_logic_vector(3 downto 0) := x"0";
    constant OP_ADD  : std_logic_vector(3 downto 0) := x"1";
    constant OP_SUB  : std_logic_vector(3 downto 0) := x"2";
    constant OP_MUL  : std_logic_vector(3 downto 0) := x"3";
    constant OP_LDI  : std_logic_vector(3 downto 0) := x"4";
    constant OP_ADDI : std_logic_vector(3 downto 0) := x"5";
    constant OP_SUBI : std_logic_vector(3 downto 0) := x"6";
    constant OP_MULI : std_logic_vector(3 downto 0) := x"7";
    constant OP_ST   : std_logic_vector(3 downto 0) := x"8";
    constant OP_IN   : std_logic_vector(3 downto 0) := x"9";
    constant OP_OUT  : std_logic_vector(3 downto 0) := x"A";
    constant OP_SHR  : std_logic_vector(3 downto 0) := x"B";
    constant OP_SKIP : std_logic_vector(3 downto 0) := x"C";
    constant OP_JMP  : std_logic_vector(3 downto 0) := x"D";
    constant OP_CALL : std_logic_vector(3 downto 0) := x"E";
    constant OP_SPC  : std_logic_vector(3 downto 0) := x"F";      -- 0 return, 1 halt
    signal rf_ram : t_c_gn_rfi := C_GN_RFI;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal e_run : std_logic := '0';
    signal e_ph  : unsigned(1 downto 0) := "11";
    signal e_pc, e_lr : unsigned(9 downto 0) := (others => '0');
    signal e_ir  : std_logic_vector(15 downto 0) := x"F001";      -- the microprogram ROM's output register
    signal e_irf : std_logic_vector(15 downto 0) := x"F001";      -- its fabric copy
    signal x_mul, x_shr, x_halt, x_out, x_st : std_logic := '0';
    signal x_gate, x_subr, x_shift, x_cz : std_logic := '0';
    signal e_acc, e_b : signed(36 downto 0) := (others => '0');
    signal e_cnt : unsigned(4 downto 0) := (others => '0');
    signal e_out : std_logic;
    signal e_td  : std_logic;                                      -- an OUT TD this clock: target e_ta takes e_acc
    signal e_ta  : unsigned(5 downto 0) := (others => '0');
    signal w_in  : unsigned(15 downto 0);

    -- [C] per-field constants
    signal c_P3  : unsigned(11 downto 0) := to_unsigned(1128, 12);  -- the pitch, Q3 virtual px (even)
    signal c_half3 : unsigned(10 downto 0) := to_unsigned(564, 11);
    signal c_h3  : unsigned(10 downto 0) := (others => '0');      -- post half size, Q3
    signal c_W3  : unsigned(11 downto 0) := (others => '0');      -- the cross half width h (cf + sf)
    signal c_sgx, c_sgy : std_logic := '0';                       -- '1': mirrored (negative cos / sin)
    signal c_xst : unsigned(4 downto 0) := to_unsigned(8, 5);
    signal c_Fw, c_Cu : signed(13 downto 0) := (others => '0');
    signal z_ysw : signed(19 downto 0) := (others => '0');         -- ys - P8 (the line engine's row step)
    signal c_lsinc, c_lsincw : signed(22 downto 0) := (others => '0');   -- lx sf per pixel, Q3.10
    signal c_vsi, c_vsiw : signed(29 downto 0) := (others => '0');       -- ly cf per line, Q8.12
    type t_s15x4 is array (0 to 3) of signed(14 downto 0);
    signal c_CxO : t_s15x4 := (others => (others => '0'));
    type t_s15x3 is array (0 to 2) of signed(14 downto 0);
    signal c_CxE : t_s15x3 := (others => (others => '0'));
    signal c_E, c_Ey0, c_nK1, c_nK2, c_Psfp : signed(14 downto 0) := (others => '0');
    signal c_lw3 : unsigned(5 downto 0) := to_unsigned(16, 6);
    type t_u9x3 is array (1 to 3) of unsigned(8 downto 0);
    signal c_thr : t_u9x3 := (others => (others => '1'));
    type t_s14x2 is array (0 to 1) of signed(13 downto 0);
    signal c_F0 : t_s14x2 := (others => (others => '0'));
    type t_s22x2 is array (0 to 1) of signed(21 downto 0);
    signal c_Ls0 : t_s22x2 := (others => (others => '0'));
    type t_s16x2 is array (0 to 1) of signed(15 downto 0);
    signal c_xh : t_s16x2 := (others => (others => '0'));          -- pixel 0's home post (Q3 from the left)
    signal c_bxlo : unsigned(5 downto 0) := (others => '0');
    signal c_bxhi : unsigned(5 downto 0) := to_unsigned(59, 6);
    signal c_kn0 : unsigned(7 downto 0) := to_unsigned(6, 8);      -- the source's second sample's phase in its block
    signal c_col0 : unsigned(5 downto 0) := (others => '0');

    -- [L] line engine
    signal e_ly  : signed(18 downto 0) := (others => '0');        -- local y, Q8 (screen frame)
    signal e_G   : signed(18 downto 0) := (others => '0');        -- the next line's distance past the row's edge
    signal e_Vs  : signed(29 downto 0) := (others => '0');        -- ly cf, Q8.12
    signal e_yh  : signed(19 downto 0) := (others => '0');        -- the row's centre, Q8 from the top
    signal e_par : std_logic := '0';
    signal e_j   : unsigned(4 downto 0) := (others => '1');       -- derive step (31 = idle)
    signal e_wait : unsigned(3 downto 0) := (others => '0');
    signal e_go, e_step : std_logic := '0';
    signal e_v3, e_nvc, l_alu : signed(14 downto 0) := (others => '0');
    signal e_sl  : signed(7 downto 0) := (others => '0');
    signal e_ybm : signed(19 downto 0) := (others => '0');
    -- per-line values used by the pixel path and the camera read
    type t_s15x10 is array (0 to 9) of signed(14 downto 0);
    signal l_Kl : t_s15x10 := (others => (others => '0'));
    type t_s8x3l is array (0 to 2) of signed(7 downto 0);
    signal l_dv : t_s8x3l := (others => (others => '0'));
    signal l_topv : signed(7 downto 0) := (others => '0');
    signal l_var : std_logic := '0';
    signal l_by : unsigned(4 downto 0) := (others => '0');
    -- the source's block rows (the grid write)
    signal n_yq  : unsigned(12 downto 0) := (others => '0');
    signal n_row, l_row : unsigned(6 downto 0) := (others => '0');
    signal n_first, l_first, l_rev : std_logic := '0';

    -- [K] camera read: the home post's step (the tile tone) and the next post's, staged
    signal k_cur, k_stg : unsigned(1 downto 0) := (others => '0');
    signal k_x   : signed(15 downto 0) := (others => '0');
    signal k_rnd : unsigned(1 downto 0) := (others => '0');
    signal k_first : std_logic := '0';
    signal k_ra  : unsigned(11 downto 0) := (others => '0');
    signal k_jv, k_jc : std_logic_vector(1 to 3) := (others => '0');
    signal k_lv  : unsigned(7 downto 0) := (others => '0');
    signal k_pfgo : std_logic := '0';

    -- [P] pixel path
    signal b_F   : signed(13 downto 0) := (others => '0');
    signal b_Ls  : signed(21 downto 0) := (others => '0');
    signal b_act, b_cross, b_xing : std_logic := '0';
    signal s2_u  : signed(13 downto 0) := (others => '0');
    signal s2_ls : signed(11 downto 0) := (others => '0');
    type t_s15x9 is array (0 to 8) of signed(14 downto 0);
    signal s3_x, s4_y : t_s15x9 := (others => (others => '0'));
    type t_s15x6 is array (0 to 5) of signed(14 downto 0);
    signal s3_lo, s4_hi : t_s15x6 := (others => (others => '0'));  -- 0 .. 2 even rows, 3 .. 5 the middle row
    signal s3_au : unsigned(12 downto 0) := (others => '0');
    type t_s8x6 is array (0 to 5) of signed(7 downto 0);
    signal s4_slo, s5_slo, s5_shi, s6_ub : t_s8x6 := (others => (others => '0'));
    signal s4_tu : signed(13 downto 0) := (others => '0');
    type t_s8x9 is array (0 to 8) of signed(7 downto 0);
    signal s5_scr, s6_a, s7_d : t_s8x9 := (others => (others => '0'));
    signal s5_tu, s6_top : signed(7 downto 0) := (others => '0');
    type t_s8x5 is array (0 to 4) of signed(7 downto 0);
    signal s8_m : t_s8x5 := (others => (others => '0'));
    type t_s8x3c is array (0 to 2) of signed(7 downto 0);
    signal s9_m : t_s8x3c := (others => (others => '0'));
    signal s10_a, s10_b, s11_d : signed(7 downto 0) := (others => '0');
    type t_s8p is array (7 to 11) of signed(7 downto 0);
    signal s_topp : t_s8p := (others => (others => '0'));
    type t_u2p is array (2 to 18) of unsigned(1 downto 0);
    signal s_tone : t_u2p := (others => (others => '0'));          -- the home tile's camera step
    signal s12_d, s12_t : unsigned(6 downto 0) := (others => '0');   -- the distance + 64 (0 .. 127)
    signal s12_dt, s12_tt : unsigned(7 downto 0) := (others => '0');
    signal s13_ds, s13_ts : unsigned(8 downto 0) := (others => '0');
    signal s14_cs, s14_ct, s14_csl, s14_ctl : unsigned(4 downto 0) := (others => '0');
    signal s15_mx, s15_cl, s15_pl, s15_ct : unsigned(4 downto 0) := (others => '0');
    signal s16_a, s16_b, s16_q, s16_w : unsigned(8 downto 0) := (others => '0');
    signal s16_mxl, s16_pl : unsigned(4 downto 0) := (others => '0');
    signal s17_m1, s17_fb, s17_fi, s17_iv, s18_iv : unsigned(8 downto 0) := (others => '0');
    signal s17_ws : unsigned(7 downto 0) := (others => '0');
    signal s17_pl : unsigned(4 downto 0) := (others => '0');
    signal s18_m : unsigned(8 downto 0) := (others => '0');
    signal s18_pl : unsigned(4 downto 0) := (others => '0');
    signal s19_pa, s19_pb : unsigned(12 downto 0) := (others => '0');
    signal s19_m, s20_m : unsigned(8 downto 0) := (others => '0');
    signal s21_a, s21_b : unsigned(16 downto 0) := (others => '0');
    signal s22_s : unsigned(9 downto 0) := (others => '0');
    signal c_kc, c_kl : signed(11 downto 0) := (others => '0');     -- the mode's offsets (and the Fine line's)

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_l8, s_sfirst : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_svd : std_logic := '0';
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(5 downto 0) := (others => '0');
    signal s_bsum : unsigned(12 downto 0) := (others => '0');
    signal k_n   : unsigned(7 downto 0) := (others => '0');
    signal k_col, k_evc, e_col : unsigned(5 downto 0) := (others => '0');
    signal k_ev, e_v : std_logic := '0';
    type t_st is array (0 to 255) of unsigned(15 downto 0);
    signal st_ram : t_st := (others => (others => '0'));
    attribute no_rw_check of st_ram : signal is true;
    signal st_q, st_wd : unsigned(15 downto 0) := (others => '0');
    signal st_wa : unsigned(5 downto 0) := (others => '0');
    signal st_we : std_logic := '0';
    signal s_eb : unsigned(7 downto 0) := (others => '0');
    signal s_eq : unsigned(11 downto 0) := (others => '0');
    -- the grid: 2 banks x 32 rows x 64 columns x 8 bits
    type t_gr is array (0 to 4095) of unsigned(7 downto 0);
    signal gr_ram : t_gr := (others => (others => '0'));
    attribute no_rw_check of gr_ram : signal is true;
    signal gr_q, gr_wd : unsigned(7 downto 0) := (others => '0');
    signal gr_wa : unsigned(11 downto 0) := (others => '0');
    signal gr_we : std_logic := '0';

    -- [F] output conditioning
    signal s23_y, s24_y, s25_y, s26_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s26_ya : unsigned(11 downto 0) := (others => '0');
    signal s27_y, s28_y, s29_y : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- slot map: slot s = 3 r + i (row r 0 .. 2 behind, column i 0 .. 2 behind); the column box it uses; its Kl
    type t_int9 is array (0 to 8) of integer;
    constant C_SR : t_int9 := (0, 0, 0, 1, 1, 1, 2, 2, 2);
    constant C_SD : t_int9 := (0, 1, 2, 3, 4, 5, 0, 1, 2);
    constant C_SK : t_int9 := (0, 1, 2, 3, 4, 5, 7, 8, 9);

begin

    ---------------------------------------------------------------------------
    -- ROM and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_GN_TAB(to_integer(r_ex_a));
        end if;
    end process p_rom;

    -- box delay: read 32 samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv = '1' then bx_ram(to_integer(bx_wp)) <= s_l8; end if;
        end if;
    end process p_bxram;

    -- one-pole state: read at the column about to end, written three clocks after it ended
    p_stram : process(clk)
    begin
        if rising_edge(clk) then
            st_q <= st_ram(to_integer(resize(k_col, 8)));
            if st_we = '1' then st_ram(to_integer(resize(st_wa, 8))) <= st_wd; end if;
        end if;
    end process p_stram;

    -- the grid: the camera read takes the bank written last field, the source writes the other
    p_grram : process(clk)
    begin
        if rising_edge(clk) then
            gr_q <= gr_ram(to_integer(k_ra));
            if gr_we = '1' then gr_ram(to_integer(gr_wa)) <= gr_wd; end if;
        end if;
    end process p_grram;

    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            -- [A] timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);

            -- [A] vblank zone: no avid for 1024 clocks
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

            -- [A] width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- [A][B] per-field latches; the grid banks swap here (once per field: the zone needs 1024
            -- clocks without active video, so serrated vsync cannot fire it twice)
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_stag  <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_bank  <= not s_bank;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                                   c_bxlo <= to_unsigned(7, 6); c_bxhi <= to_unsigned(51, 6); c_col0 <= to_unsigned(7, 6);
                                   c_kn0 <= to_unsigned(68 + 8, 8);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                                   c_bxlo <= to_unsigned(0, 6); c_bxhi <= to_unsigned(59, 6); c_col0 <= to_unsigned(0, 6);
                                   c_kn0 <= to_unsigned(3 + 6, 8);
                    when others => s_oxs <= to_unsigned(102, 9);
                                   c_bxlo <= to_unsigned(0, 6); c_bxhi <= to_unsigned(59, 6); c_col0 <= to_unsigned(0, 6);
                                   c_kn0 <= to_unsigned(2 + 4, 8);
                end case;
                if s_ys <= 512 then s_ish3 <= '1'; else s_ish3 <= '0'; end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- [A] sync delay and generated-avid shift registers (field_n is tied high by the core on every
            -- input, so it only gets a register)
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
        end if;
    end process p_ctrl;

    ------------------------------------------------------------------------
    -- [C] THE PER-FIELD ENGINE (Hardline flute's, as in windrose).  Started at
    -- every zone start from address 0 (so a stalled or garbled run cannot
    -- outlive a field); three clocks per micro-op (register-file read,
    -- operand, execute), serial multiply (18 steps) and shift.  ACC = 37 bits;
    -- MUL = ACC(17:0) x operand(17:0), both signed, by add-and-shift-right with
    -- the last step subtracting.  The microprogram (gnomon.py) does the knob
    -- holds and glides, the Drift, the sun's azimuth and its fold, the pitch,
    -- size and length and every start value; OUT TD writes them, in target
    -- order, straight into the registers below (and the line engine's).
    ------------------------------------------------------------------------
    with e_irf(3 downto 0) select w_in <=
        resize(s_ys, 16)            when x"8",
        resize(s_xs, 16)            when x"9",
        r_ex_q                      when x"B",
        (others => '0')             when x"A" | x"C" | x"D" | x"E" | x"F",
        resize(unsigned(registers_in(to_integer(unsigned(e_irf(2 downto 0))))), 16) when others;

    e_out <= '1' when e_run = '1' and e_ph = "10" and x_out = '1' else '0';
    e_td  <= '1' when e_out = '1' and e_irf(1 downto 0) = "10" else '0';

    p_eng : process(clk)
        variable v_op  : std_logic_vector(3 downto 0);
        variable v_arg : std_logic_vector(11 downto 0);
        variable v_shift, v_last, v_cond, v_fetch : std_logic;
        variable v_bx  : signed(36 downto 0);
        variable v_t   : signed(37 downto 0);
        variable v_sum : signed(36 downto 0);
    begin
        if rising_edge(clk) then
            v_op := e_irf(15 downto 12);
            v_arg := e_irf(11 downto 0);
            v_fetch := '0';
            if e_ph = "00" then
                rf_q <= rf_ram(to_integer(unsigned(e_ir(7 downto 0))));
            end if;
            if e_run = '1' and e_ph = "10" and x_st = '1' then
                rf_ram(to_integer(unsigned(e_irf(7 downto 0)))) <= std_logic_vector(e_acc(31 downto 0));
            end if;

            if s_zgo = '1' then
                e_run <= '1'; e_pc <= (others => '0'); e_ph <= "11";
            elsif e_run = '1' then
                case e_ph is
                    when "11" =>                          -- fetch (first micro-op)
                        v_fetch := '1';
                        e_ph <= "00";
                    when "00" =>                          -- register-file read
                        e_irf <= e_ir;
                        e_ph <= "01";
                    when "01" =>                          -- operand, accumulator clear, next address
                        case v_op is
                            when OP_LD | OP_ADD | OP_SUB => e_b <= resize(signed(rf_q), 37);
                            when OP_LDI | OP_ADDI | OP_SUBI => e_b <= resize(signed(v_arg), 37);
                            when OP_IN   => e_b <= signed(resize(w_in, 37));
                            when OP_MUL  => e_b <= shift_left(resize(signed(rf_q(17 downto 0)), 37), 18);
                            when OP_MULI => e_b <= shift_left(resize(signed(v_arg), 37), 18);
                            when others  => null;
                        end case;
                        if v_op = OP_LD or v_op = OP_LDI or v_op = OP_IN then
                            e_acc <= (others => '0');
                        elsif v_op = OP_MUL or v_op = OP_MULI then
                            e_acc(36 downto 18) <= (others => '0');
                        end if;
                        if v_op = OP_MUL or v_op = OP_MULI then e_cnt <= to_unsigned(17, 5);
                        elsif v_op = OP_SHR then e_cnt <= unsigned(v_arg(4 downto 0)) - 1; end if;
                        case v_arg(2 downto 0) is
                            when "000"  => v_cond := e_acc(36);
                            when "010"  => v_cond := s_stag;
                            when others => v_cond := s_glide;
                        end case;
                        v_cond := v_cond xor v_arg(3);
                        if v_op = OP_SKIP and v_cond = '1' then e_pc <= e_pc + 2;
                        elsif v_op = OP_JMP then e_pc <= unsigned(v_arg(9 downto 0));
                        elsif v_op = OP_CALL then e_pc <= unsigned(v_arg(9 downto 0)); e_lr <= e_pc + 1;
                        elsif v_op = OP_SPC and v_arg(1 downto 0) = "00" then e_pc <= e_lr;
                        else e_pc <= e_pc + 1; end if;
                        x_mul <= '0'; x_shr <= '0'; x_halt <= '0'; x_out <= '0'; x_st <= '0';
                        x_gate <= '0'; x_subr <= '0'; x_shift <= '0'; x_cz <= '0';
                        case v_op is
                            when OP_LD | OP_ADD | OP_LDI | OP_ADDI | OP_IN => x_gate <= '1';
                            when OP_SUB | OP_SUBI => x_gate <= '1'; x_subr <= '1';
                            when OP_MUL | OP_MULI => x_mul <= '1'; x_shift <= '1'; x_gate <= e_acc(0);
                            when OP_SHR  =>
                                x_shr <= '1'; x_shift <= '1';
                                if v_arg(4 downto 0) = "00001" then x_cz <= '1'; end if;
                            when OP_ST   => x_st <= '1';
                            when OP_OUT  => x_out <= '1';
                            when OP_SPC  => x_halt <= v_arg(0);
                            when others  => null;
                        end case;
                        e_ph <= "10";
                    when others =>                        -- execute (the adder sees registers only)
                        v_shift := x_shift; v_last := '1';
                        if x_gate = '0' then v_bx := (others => '0');
                        elsif x_subr = '1' then v_bx := not e_b;
                        else v_bx := e_b; end if;
                        v_t := (e_acc & '1') + (v_bx & (x_subr and x_gate));
                        v_sum := v_t(37 downto 1);
                        if x_mul = '1' or x_shr = '1' then
                            if x_cz = '0' then v_last := '0'; end if;
                            e_cnt <= e_cnt - 1;
                            if e_cnt = 1 then x_cz <= '1'; x_subr <= x_mul; end if;   -- the next step is the last
                            if x_mul = '1' then x_gate <= v_sum(1); end if;           -- the next multiplier bit
                        end if;
                        if x_halt = '1' then e_run <= '0'; end if;
                        if v_shift = '1' then e_acc <= v_sum(36) & v_sum(36 downto 1);
                        else e_acc <= v_sum; end if;
                        if v_last = '1' then
                            v_fetch := '1';
                            e_ph <= "00";
                        end if;
                end case;
            end if;
            if v_fetch = '1' then
                e_ir <= C_GN_UC(to_integer(e_pc));
            end if;

            -- the engine's outputs: the table ROM's address; the field's registers (OUT TD, target e_ta)
            if e_out = '1' then
                case e_irf(1 downto 0) is
                    when "00"   => r_ex_a <= unsigned(e_acc(8 downto 0));
                    when "01"   => e_ta <= unsigned(e_acc(5 downto 0));
                    when others => e_ta <= e_ta + 1;
                end case;
            end if;
            if e_td = '1' then
                case to_integer(e_ta) is
                    when 0  => c_P3 <= unsigned(e_acc(11 downto 0));
                    when 1  => c_half3 <= unsigned(e_acc(10 downto 0));
                    when 2  => c_h3 <= unsigned(e_acc(10 downto 0));
                    when 3  => c_W3 <= unsigned(e_acc(11 downto 0));
                    when 4  => c_sgx <= e_acc(0); c_sgy <= e_acc(1);
                    when 5  => c_xst <= unsigned(e_acc(4 downto 0));
                    when 6  => c_Fw <= e_acc(13 downto 0);
                    when 7  => c_Cu <= e_acc(13 downto 0);
                    when 8  => c_lsinc <= e_acc(22 downto 0);
                    when 9  => c_lsincw <= e_acc(22 downto 0);
                    when 10 => c_vsi <= e_acc(29 downto 0);
                    when 11 => c_vsiw <= e_acc(29 downto 0);
                    when 12 => z_ysw <= e_acc(19 downto 0);
                    when 13 => c_CxE(0) <= e_acc(14 downto 0);
                    when 14 => c_CxE(1) <= e_acc(14 downto 0);
                    when 15 => c_CxE(2) <= e_acc(14 downto 0);
                    when 16 => c_CxO(0) <= e_acc(14 downto 0);
                    when 17 => c_CxO(1) <= e_acc(14 downto 0);
                    when 18 => c_CxO(2) <= e_acc(14 downto 0);
                    when 19 => c_CxO(3) <= e_acc(14 downto 0);
                    when 20 => c_E <= e_acc(14 downto 0);
                    when 21 => c_Ey0 <= e_acc(14 downto 0);
                    when 22 => c_nK1 <= e_acc(14 downto 0);
                    when 23 => c_nK2 <= e_acc(14 downto 0);
                    when 24 => c_Psfp <= e_acc(14 downto 0);
                    when 25 => c_lw3 <= unsigned(e_acc(5 downto 0));
                    when 26 => c_thr(1) <= unsigned(e_acc(8 downto 0));
                    when 27 => c_thr(2) <= unsigned(e_acc(8 downto 0));
                    when 28 => c_thr(3) <= unsigned(e_acc(8 downto 0));
                    when 29 => c_F0(0) <= e_acc(13 downto 0);
                    when 30 => c_F0(1) <= e_acc(13 downto 0);
                    when 31 => c_Ls0(0) <= e_acc(21 downto 0);
                    when 32 => c_Ls0(1) <= e_acc(21 downto 0);
                    when 33 => c_xh(0) <= e_acc(15 downto 0);
                    when 34 => c_xh(1) <= e_acc(15 downto 0);
                    when others => null;              -- 35 .. 40: the line engine's (p_line)
                end case;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- [A] Generated avid (parity-locked start 3/4 clocks after the source's
    -- avid rise, field-latched width)
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
    -- [P] the beam enters a new cell at the end of this clock
    b_xing <= '1' when g_start = '0' and b_act = '1' and b_F(13) = '0' else '0';

    ---------------------------------------------------------------------------
    -- [L] LINE ENGINE.  Line 0 comes from the microprogram (G, ly cf, the row
    -- centre) and the reduction (ly, parity) at c_kick; every later line is
    -- stepped from the one before at its end (the generated avid's fall, then
    -- 8 clocks for the line's last pixels to clear the slot stages).  Then 18
    -- derive steps on one 15-bit ALU write the per-line slot constants: the
    -- cross offsets Kl (signed by sgx), the v-box distance per row, the top's
    -- v part; and the camera's block row.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y  : unsigned(13 downto 0);
        variable v_a, v_b : signed(14 downto 0);
        variable v_sub : std_logic;
        variable v_v3, v_vc : signed(14 downto 0);
        variable v_j : integer range 0 to 31;
    begin
        if rising_edge(clk) then
            e_go <= '0';
            -- the end of a line: wait, then step to the next line and derive
            if s_gav_sr(0) = '1' and g_avid = '0' then
                e_wait <= to_unsigned(8, 4);
            elsif e_wait /= 0 then
                e_wait <= e_wait - 1;
            end if;
            e_step <= '0';
            if e_wait = 1 then e_step <= '1'; end if;
            -- line 0's values from the engine (targets 35 .. 39; 40 starts the derive)
            if e_td = '1' then
                case to_integer(e_ta) is
                    when 35     => e_ly <= e_acc(18 downto 0);
                    when 36     => e_par <= e_acc(0);
                    when 37     => e_G <= e_acc(18 downto 0);
                    when 38     => e_Vs <= e_acc(29 downto 0);
                    when 39     => e_yh <= e_acc(19 downto 0);
                    when others => null;
                end case;
            end if;
            if e_td = '1' and e_ta = 40 then
                e_go <= '1';
            elsif e_step = '1' and s_zone = '0' then
                e_go <= '1';
                if e_G(18) = '0' then
                    e_ly <= e_ly + resize(z_ysw, 19);
                    e_G  <= e_G + resize(z_ysw, 19);
                    e_Vs <= e_Vs + c_vsiw;
                    e_par <= not e_par;
                    e_yh <= e_yh + signed(resize(c_P3 & "00000", 20));
                else
                    e_ly <= e_ly + signed(resize(s_ys, 19));
                    e_G  <= e_G + signed(resize(s_ys, 19));
                    e_Vs <= e_Vs + c_vsi;
                end if;
            end if;

            -- derive
            if e_go = '1' then
                e_j <= (others => '0');
            elsif e_j /= 31 then
                if e_j = 18 then e_j <= (others => '1'); else e_j <= e_j + 1; end if;
            end if;
            v_j := to_integer(e_j);
            if v_j = 0 then
                v_v3 := resize(shift_right(e_ly, 5), 15);
                v_vc := resize(shift_right(e_Vs, 17), 15);
                if c_sgy = '1' then e_v3 <= -v_v3; else e_v3 <= v_v3; end if;
                if c_sgx = c_sgy then e_nvc <= -v_vc; else e_nvc <= v_vc; end if;
                l_var <= e_par and s_stag;
                -- the camera row: the home row's 32-vpx block row
                if e_yh(19) = '1' then l_by <= (others => '0');
                elsif e_yh(18) = '1' then l_by <= (others => '1');
                else l_by <= unsigned(e_yh(17 downto 13)); end if;
            end if;
            -- the ALU: l_alu = A + B or A - B
            v_a := l_alu; v_b := c_Psfp; v_sub := '0';
            case v_j is
                when 1      => v_a := e_nvc; v_b := (others => '0');
                when 4      => v_a := e_nvc; v_b := c_nK1;
                when 8      => v_a := e_nvc; v_b := c_nK2;
                when 11     => v_a := e_v3; v_b := c_CxE(0);
                when 13     => v_a := e_v3; v_b := c_CxE(1);
                when 15     => v_a := e_v3; v_b := c_CxE(2);
                when 12 | 14 | 16 => v_a := c_Ey0; v_b := l_alu; v_sub := '1';
                when 17     => v_a := signed(resize(c_h3, 15)); v_b := signed(resize(f_oabs(e_v3), 15)); v_sub := '1';
                when others => null;
            end case;
            if v_j >= 1 and v_j <= 17 then l_alu <= f_as(v_a, v_b, v_sub); end if;
            -- the results (the step after their ALU step)
            for q in 0 to 9 loop
                if v_j = q + 2 then l_Kl(q) <= l_alu; end if;
            end loop;
            if v_j = 12 or v_j = 14 or v_j = 16 then e_sl <= f_sat(l_alu); end if;
            for r in 0 to 2 loop
                if v_j = 13 + 2 * r then l_dv(r) <= f_min8(e_sl, f_sat(l_alu)); end if;
            end loop;
            if v_j = 18 then l_topv <= f_sat(l_alu); end if;

            -- [D]'s block rows (the source's 32-vpx grid rows), taken at the line start, stepped right after
            if s_zone = '1' then
                if s_zc = 306 then
                    n_yq <= to_unsigned(1024, 13) + resize(s_ys(11 downto 1), 13);
                    n_row <= (others => '1');
                    n_first <= '1';
                end if;
            elsif g_start = '1' then
                l_row <= n_row; l_first <= n_first;
                n_first <= '0';
                v_y := resize(n_yq, 14) + resize(s_ys, 14);
                n_yq <= v_y(12 downto 0);
                l_rev <= v_y(13);
                if v_y(13) = '1' then n_row <= n_row + 1; end if;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [K] CAMERA READ.  The tile tone is the camera step of the pixel's home
    -- post.  k_cur holds it; k_stg holds the next post's (one cell right).  At
    -- every cell crossing k_cur takes k_stg and one read stages the next; in
    -- the blanking two reads load both (after the derive wrote the block row).
    ---------------------------------------------------------------------------
    p_cam : process(clk)
        variable v_bx : unsigned(5 downto 0);
        variable v_st : unsigned(1 downto 0);
    begin
        if rising_edge(clk) then
            k_pfgo <= '0';
            if e_j = 2 then k_pfgo <= '1'; end if;
            k_jv(1) <= '0';
            if k_pfgo = '1' then
                if l_var = '1' then k_x <= c_xh(1); else k_x <= c_xh(0); end if;
                k_rnd <= "10"; k_first <= '1';
            elsif b_cross = '1' then
                k_rnd <= "01"; k_first <= '0';
            elsif k_rnd /= 0 then
                if k_x(15) = '1' then v_bx := c_bxlo;
                elsif unsigned(k_x(14 downto 8)) > resize(c_bxhi, 7) then v_bx := c_bxhi;
                elsif unsigned(k_x(13 downto 8)) < c_bxlo then v_bx := c_bxlo;
                else v_bx := unsigned(k_x(13 downto 8)); end if;
                k_ra <= (not s_bank) & l_by & v_bx;
                k_x <= k_x + signed(resize(c_P3, 16));
                k_jv(1) <= '1'; k_jc(1) <= k_first; k_first <= '0';
                k_rnd <= k_rnd - 1;
            end if;
            -- the read pipeline: address (1), RAM register (2), fabric register (3) -> the step
            k_jv(2) <= k_jv(1); k_jc(2) <= k_jc(1);
            k_jv(3) <= k_jv(2); k_jc(3) <= k_jc(2);
            k_lv <= gr_q;
            v_st := "00";
            if resize(k_lv, 9) >= c_thr(1) then v_st := "01"; end if;
            if resize(k_lv, 9) >= c_thr(2) then v_st := "10"; end if;
            if resize(k_lv, 9) >= c_thr(3) then v_st := "11"; end if;
            if b_xing = '1' then k_cur <= k_stg; end if;
            if k_jv(3) = '1' then
                if k_jc(3) = '1' then k_cur <= v_st; else k_stg <= v_st; end if;
            end if;
        end if;
    end process p_cam;

    ---------------------------------------------------------------------------
    -- [P] PIXEL PATH
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_cs  : unsigned(4 downto 0);
        variable v_t   : signed(5 downto 0);
        variable v_g0, v_g1, v_g2, v_g3, v_g4 : unsigned(12 downto 0);
        variable v_iv  : unsigned(9 downto 0);
        variable v_y   : unsigned(16 downto 0);
        variable v_kl  : signed(14 downto 0);
        variable v_cx  : signed(14 downto 0);
        variable v_neg : boolean;
        variable v_ws  : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: the beam.  b_F is the NEXT pixel's distance past the cell's far edge (screen frame): when it is
            -- not negative the next pixel is in the next cell, lx sf steps back by one cell there and the camera
            -- takes the next post's step (p_cam, on b_xing).
            if g_start = '1' then
                if l_var = '1' then b_F <= c_F0(1); b_Ls <= c_Ls0(1);
                else b_F <= c_F0(0); b_Ls <= c_Ls0(0); end if;
                b_act <= '1';
            else
                if b_F(13) = '0' then
                    b_F <= b_F + c_Fw; b_Ls <= b_Ls + resize(c_lsincw, 22);
                else
                    b_F <= b_F + signed(resize(c_xst, 14)); b_Ls <= b_Ls + resize(c_lsinc, 22);
                end if;
                if s_gav_sr(1) = '1' and g_avid = '0' then b_act <= '0'; end if;
            end if;
            b_cross <= b_xing;

            -- 2: u' = sgx lx (folded local x), lx sf, the tile's camera step
            s2_u <= f_as(c_Cu, b_F, c_sgx);
            s2_ls <= resize(shift_right(b_Ls, 10), 12);
            s_tone(2) <= k_cur;
            for q in 3 to 18 loop s_tone(q) <= s_tone(q - 1); end loop;
            -- 3: the slots' cross terms x' = lx sf + sgx Kl; the column boxes' near sides; in the Stagger middle
            -- row the three posts nearest the pixel (by the sign of u')
            v_neg := s_stag = '1' and s2_u(13) = '1';
            for q in 0 to 8 loop
                if C_SR(q) = 1 and v_neg then v_kl := l_Kl(C_SK(q) + 1); else v_kl := l_Kl(C_SK(q)); end if;
                s3_x(q) <= resize(s2_ls, 15) + v_kl;
            end loop;
            for i in 0 to 2 loop
                s3_lo(i) <= resize(s2_u, 15) + c_CxE(i);
                if v_neg then v_cx := c_CxO(i + 1); else v_cx := c_CxO(i); end if;
                s3_lo(3 + i) <= resize(s2_u, 15) + v_cx;
            end loop;
            s3_au <= f_oabs(s2_u);
            -- 4: the cross distance W - |x'|, the boxes' far sides
            for q in 0 to 8 loop
                s4_y(q) <= signed(resize(c_W3, 15)) - signed(resize(f_oabs(s3_x(q)), 15));
            end loop;
            for c in 0 to 5 loop
                s4_hi(c) <= c_E - s3_lo(c);
                s4_slo(c) <= f_sat(s3_lo(c));
            end loop;
            s4_tu <= signed(resize(c_h3, 14)) - signed(resize(s3_au, 14));
            -- 5 .. 7: the slot's distance = min(cross, v box, u box)
            for q in 0 to 8 loop s5_scr(q) <= f_sat(s4_y(q)); end loop;
            for c in 0 to 5 loop s5_shi(c) <= f_sat(s4_hi(c)); end loop;
            s5_slo <= s4_slo;
            s5_tu <= f_sat(s4_tu);
            for q in 0 to 8 loop s6_a(q) <= f_min8(s5_scr(q), l_dv(C_SR(q))); end loop;
            for c in 0 to 5 loop s6_ub(c) <= f_min8(s5_slo(c), s5_shi(c)); end loop;
            s6_top <= f_min8(s5_tu, l_topv);
            for q in 0 to 8 loop s7_d(q) <= f_min8(s6_a(q), s6_ub(C_SD(q))); end loop;
            -- 8 .. 11: the union
            for q in 0 to 3 loop s8_m(q) <= f_max8(s7_d(2 * q), s7_d(2 * q + 1)); end loop;
            s8_m(4) <= s7_d(8);
            s9_m(0) <= f_max8(s8_m(0), s8_m(1));
            s9_m(1) <= f_max8(s8_m(2), s8_m(3));
            s9_m(2) <= s8_m(4);
            s10_a <= f_max8(s9_m(0), s9_m(1));
            s10_b <= s9_m(2);
            s11_d <= f_max8(s10_a, s10_b);
            s_topp(7) <= s6_top;
            for q in 8 to 11 loop s_topp(q) <= s_topp(q - 1); end loop;
            -- 12 .. 14: coverages (and the Fine lines' inner edges): clamp(8 + 16 d / pixel) with the distance
            -- scaled per mode (2 d at 1080, d + d/4 + d/16 at 720p, d at SD), the line weight scaled the same.
            -- Unsigned (d + 64: the sign bit flipped), so no adder sees one net on both inputs.
            s12_d <= unsigned((not s11_d(6)) & s11_d(5 downto 0));
            s12_t <= unsigned((not s_topp(11)(6)) & s_topp(11)(5 downto 0));
            s12_dt <= resize(unsigned((not s11_d(6)) & s11_d(5 downto 0)), 8) + resize(unsigned(s11_d(6 downto 2)) xor "10000", 8);
            s12_tt <= resize(unsigned((not s_topp(11)(6)) & s_topp(11)(5 downto 0)), 8) + resize(unsigned(s_topp(11)(6 downto 2)) xor "10000", 8);
            case to_integer(s_xs) is
                when 2      => s13_ds <= resize(s12_d & '0', 9); s13_ts <= resize(s12_t & '0', 9);
                when 4      => s13_ds <= resize(s12_d, 9); s13_ts <= resize(s12_t, 9);
                when others => s13_ds <= resize(s12_dt, 9) + resize(s12_d(6 downto 4), 9);
                               s13_ts <= resize(s12_tt, 9) + resize(s12_t(6 downto 4), 9);
            end case;
            s14_cs <= f_cl16(signed(resize(s13_ds, 12)) - c_kc);
            s14_ct <= f_cl16(signed(resize(s13_ts, 12)) - c_kc);
            s14_csl <= f_cl16(signed(resize(s13_ds, 12)) - c_kl);
            s14_ctl <= f_cl16(signed(resize(s13_ts, 12)) - c_kl);
            -- 15: max(cs, ct); the Fine line coverages cl = cs - csl, pl = ct - ctl
            if s14_cs > s14_ct then s15_mx <= s14_cs; else s15_mx <= s14_ct; end if;
            v_t := signed(resize(s14_cs, 6)) - signed(resize(s14_csl, 6));
            if v_t(5) = '1' then s15_cl <= (others => '0'); else s15_cl <= unsigned(v_t(4 downto 0)); end if;
            v_t := signed(resize(s14_ct, 6)) - signed(resize(s14_ctl, 6));
            if v_t(5) = '1' then s15_pl <= (others => '0'); else s15_pl <= unsigned(v_t(4 downto 0)); end if;
            s15_ct <= s14_ct;
            -- 16 .. 18: the tone.  Bold Grey: 16 ct + 160 - 10 max + 2 st (16 - ct) [st: the tile's camera step];
            -- Bold Ink: 16 ct + 256 - 16 max (a bright tile white);  Fine Grey: 160 + 32 st - 10 cl;
            -- Fine Ink: 256 - 16 (st bright ? pl : max(cl, pl))
            if s_ink = '1' then
                s16_a <= resize(s15_ct & "0000", 9) + to_unsigned(256, 9);
                s16_b <= resize(s15_mx & "0000", 9);
            else
                s16_a <= resize(s15_ct & "0000", 9) + to_unsigned(160, 9);
                s16_b <= resize(s15_mx & "000", 9) + resize(s15_mx & '0', 9);
            end if;
            s16_q <= resize(s15_cl & "000", 9) + resize(s15_cl & '0', 9);
            s16_w <= to_unsigned(16, 9) - resize(s15_ct, 9);
            if s15_cl > s15_pl and s_tone(15)(1) = '0' then s16_mxl <= s15_cl; else s16_mxl <= s15_pl; end if;
            s16_pl <= s15_pl;
            s17_m1 <= s16_a - s16_b;
            v_ws := (others => '0');
            if s_tone(16)(0) = '1' then v_ws := v_ws + resize(s16_w, 8); end if;
            if s_tone(16)(1) = '1' then v_ws := v_ws + resize(s16_w & '0', 8); end if;
            s17_ws <= v_ws;
            s17_fb <= to_unsigned(160, 9) + resize(s_tone(16) & "00000", 9) - s16_q;
            s17_fi <= to_unsigned(256, 9) - resize(s16_mxl & "0000", 9);
            s17_iv <= to_unsigned(96, 9) - resize(s_tone(16) & "00000", 9) + s16_q;        -- 256 - fb
            s17_pl <= s16_pl;
            if s_fine = '0' then
                if s_ink = '1' then
                    if s_tone(17)(1) = '1' then s18_m <= to_unsigned(256, 9); else s18_m <= s17_m1; end if;
                else
                    s18_m <= s17_m1 + resize(s17_ws & '0', 9);
                end if;
            elsif s_ink = '1' then
                s18_m <= s17_fi;
            else
                s18_m <= s17_fb;
            end if;
            s18_pl <= s17_pl;
            s18_iv <= s17_iv;
            -- 19, 20: Fine Grey: the white post lines over that, m += pl (256 - m) / 16 (gated shifts)
            v_g0 := (others => '0'); v_g1 := (others => '0'); v_g2 := (others => '0'); v_g3 := (others => '0'); v_g4 := (others => '0');
            v_iv := resize(s18_iv, 10);
            if s_fine = '1' and s_ink = '0' then
                if s18_pl(0) = '1' then v_g0 := resize(v_iv, 13); end if;
                if s18_pl(1) = '1' then v_g1 := resize(v_iv & '0', 13); end if;
                if s18_pl(2) = '1' then v_g2 := resize(v_iv & "00", 13); end if;
                if s18_pl(3) = '1' then v_g3 := resize(v_iv & "000", 13); end if;
                if s18_pl(4) = '1' then v_g4 := resize(v_iv & "0000", 13); end if;
            end if;
            s19_pa <= v_g0 + v_g1 + v_g2;
            s19_pb <= v_g3 + v_g4;
            s19_m <= s18_m;
            v_y := resize(s19_pa, 17) + resize(s19_pb, 17);
            s20_m <= s19_m + v_y(12 downto 4);
            -- 21, 22: luma = m * 219 / 64 (white 876): m 256 - m 32 - m 4 - m
            s21_a <= resize(s20_m & "00000000", 17) - resize(s20_m & "00000", 17);
            s21_b <= resize(s20_m & "00", 17) + resize(s20_m, 17);
            v_y := s21_a - s21_b;
            s22_s <= v_y(15 downto 6);
            -- per field: the mode's offset (the +64 bias scaled, less 8) and the same plus the Fine line weight
            case to_integer(s_xs) is
                when 2      => c_kc <= to_signed(120, 12);
                               c_kl <= to_signed(120, 12) + signed(resize(c_lw3 & '0', 12));
                when 4      => c_kc <= to_signed(56, 12);
                               c_kl <= to_signed(56, 12) + signed(resize(c_lw3, 12));
                when others => c_kc <= to_signed(76, 12);
                               c_kl <= to_signed(76, 12) + signed(resize(c_lw3, 12)) + signed(resize(c_lw3(5 downto 2), 12))
                                       + signed(resize(c_lw3(5 downto 4), 12));
            end case;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice, doubled) -> box of 32 samples,
    -- edge-replicated and reloaded at every line start -> at each 32-virtual-px
    -- block's last pixel a vertical one-pole per block column (reloaded on the
    -- field's first line) -> on each block row's last line its top eight bits go
    -- into the grid bank being written.  The camera read takes the other bank.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_u : signed(12 downto 0);
        variable v_q : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting; both phases restart from this
            -- field's drift in the zone (riley's lattice carried the last field's x drift on line 0)
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);   -- ot * 96
            end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- the source sample: (luma >> 2) - 16, or the lattice at twice the level
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then
                if s_osum(7) = '1' then s_l8 <= (others => '1'); else s_l8 <= s_osum(6 downto 0) & '0'; end if;
            elsif v_l > 16 then s_l8 <= v_l - 16;
            else s_l8 <= (others => '0'); end if;
            s_sv <= data_in.avid; s_svd <= s_sv;

            -- box: sum of the last 32 samples; the samples before the line start count as the first one
            if s_sv = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(225, 8); end if;
            if s_bcnt(5) = '0' then v_tap := s_sfirst; else v_tap := bx_q; end if;
            if s_sv = '1' and s_svd = '0' then
                s_sfirst <= s_l8;
                s_bsum <= s_l8 & "00000";
                s_bcnt <= to_unsigned(1, 6);
            elsif s_sv = '1' then
                s_bsum <= s_bsum + s_l8 - v_tap;
                if s_bcnt(5) = '0' then s_bcnt <= s_bcnt + 1; end if;
            end if;

            -- block columns: k_n is the next sample's phase in its block (bit 7: it starts a new block, so
            -- this sample is the block's last)
            k_ev <= '0';
            if s_sv = '0' then
                k_n <= c_kn0; k_col <= c_col0;
            elsif k_n(7) = '1' then
                k_n <= resize(k_n(6 downto 0), 8) + resize(s_xs & '0', 8);
                k_ev <= '1'; k_evc <= k_col; k_col <= k_col + 1;
            else
                k_n <= k_n + resize(s_xs & '0', 8);
            end if;

            -- the one-pole at the block's last pixel: q += (box - q) >> sh, Q8.4 (first line: q = box)
            s_eb <= s_bsum(12 downto 5); s_eq <= st_q(11 downto 0);
            e_v <= k_ev; e_col <= k_evc;
            v_d := signed(resize(s_eb & "0000", 13)) - signed(resize(s_eq, 13));
            if s_ish3 = '1' then v_u := shift_right(v_d, 3); else v_u := shift_right(v_d, 2); end if;
            if l_first = '1' then v_q := s_eb & "0000";
            else v_q := unsigned(signed(s_eq) + v_u(11 downto 0)); end if;
            st_wd <= resize(v_q, 16); st_wa <= e_col; st_we <= e_v;
            gr_wd <= v_q(11 downto 4);
            gr_wa <= s_bank & l_row(4 downto 0) & e_col;
            if e_v = '1' and l_rev = '1' and l_row(6 downto 5) = "00" then gr_we <= '1'; else gr_we <= '0'; end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: black outside pixels 0 .. W + 1 (23), luma
    -- [1 2 1] / 4 (26, 27), 13/16 toward black (28), blanking gate (29).
    -- Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_gav_sr(20) = '1' or s_gav_sr(22) = '1' then s23_y <= to_unsigned(64, 10) + s22_s; else s23_y <= to_unsigned(64, 10); end if;
            s24_y <= s23_y; s25_y <= s24_y;
            s26_ya <= resize(s23_y, 12) + resize(s25_y, 12) + 2;
            s26_yb <= s24_y;
            v_y := s26_ya + shift_left(resize(s26_yb, 12), 1);
            s27_y <= v_y(11 downto 2);
            if s27_y > 64 then v_dl := s27_y - 64; else v_dl := (others => '0'); end if;
            s28_y <= s27_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if s_gav_sr(C_LAT - 2) = '1' then s29_y <= s28_y; else s29_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s29_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture gnomon;
