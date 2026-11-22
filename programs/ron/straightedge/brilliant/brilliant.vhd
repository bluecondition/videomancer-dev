-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Brilliant v0.1.0 -- Straightedge.  The gemmologist's facet diagram of the round brilliant cut seen from above:
-- an octagonal table (the pavilion showing through it as an eight-rayed star), eight star facets, eight kites,
-- sixteen upper girdle facets; every facet one flat tone from a light that circles the stone.  Luma only
-- (U = V = 512).  Every edge is the zero of an affine function.
--
--   X, Y    = the pixel in the stone's turned frame (one DDA pair); Pave: the same pair wraps per cell of a square
--             lattice of stones (whole-pixel cells, exact)
--   fold    = |X|, |Y|, swap, reflection in the 22.5-degree line (p - 2 d n, exact on the fold lines)
--             -> wedge coordinates (a, b) and the sector code (sign X, sign Y, swap, reflected) -> sector j
--   edges   E1 = a + 106/256 b - t R (table)     E4 = a + 53/256 b - R (girdle, from the same product)
--           E2 = a + k2 b - g2 (star / kite)    E3 = a + k3 b - g3 (kite / girdle): k b in radix 4 (3 k per
--           field); the distance x alpha = 1 / sqrt(1 + k^2) (snapped to 8, 6, 4, 3, 2 / 8)
--   tone    = the sector word (read from the table every pixel): kite, star, girdle, girdle across the fold,
--             pavilion ray and its two neighbours, 2 bits each = base (the diagram's 3-colouring) + the flash
--             of the circling light (+ the per-facet camera step); a ramp (by Fire, Tone, Weight) -> luma
--   painter = kite with star or girdle (the girdle pair across its fold), the pavilion's alternate sectors across
--             theirs, then the table or the ground; Fine lines (white) or Ink (XOR)
--   camera  = the luma (or Osc) in a 32 x 32 virtual-px block grid (two banks): Stone: each facet's step from
--             its anchor block (the engine) and a halftone of glittering lozenge dots around the stone (dot size
--             per 32-vpx cell); Pave: each stone's girdle shrinks with its cell's luma below the picture's mean
--
--   K1  X        the stone's centre / the lattice's pan, +-640 virtual px                  (glided)
--   K2  Y        +-360 virtual px, up = up                                                  (glided)
--   K3  Scale    Stone R = 225 .. 520 .. 1200 virtual px; Pave cell 130 .. 266 .. 546       (glided)
--   K4  Fire     the flash: broad soft modelling .. rare hard flashes on a dark stone      (glided)
--   K5  Drift    the light circles (2^-11 .. 2^-5 turn per field), the stone turns -1/4 as fast
--   K6  Camera   0 .. 100 %                                                                 (glided)
--   S7  Glide    Off / On       S8  Form  Stone / Pave      S9  Tone  Grey / Ink
--   S10 Weight   Bold / Fine    S11 Source  Video / Osc
--   P12 CUT      table and star points: rose cut (0 %) .. brilliant (50 %) .. starburst (100 %)  (glided)
--
-- Sections (the integer model .straightedge_work/rb_brilliant/imodel.py mirrors each one):
--   [A] TIMING     timing-ID steps, vblank zone, width, line count, generated avid, sync delay (block RAM)
--   [B] CONTROLS   switches (latched per field; the pixel path takes its own from the table)
--   [C] ENGINE     Hardline flute's microcoded engine (brilliant.py) + a four-deep link stack + a grid port
--   [D] SOURCE     luma or the Osc lattice -> block sums -> vertical one-pole -> grid (two banks, raw luma)
--   [E] DDAS       X, Y with their increment registers, the table loader, the lattice counters, the per-cell
--                  camera fetch
--   [P] FACETS     fold, edges, coverages, the sector word; the glitter dots (a counter 13 clocks late)
--   [G] BLEND      two levels of selected blends, then Fine lines / Ink
--   [F] OUTPUT     luma, [1 2 1] / 4, 13/16 toward black, blanking gate
--
-- Timing: T = the g_start clock.  The DDAs hold DDA index i at T + 5 + i; stage Sk holds index i at T + 5 + k + i;
-- the luma is S23; output column c is the [1 2 1] filter centred on DDA index c + C_XOFS.  One latency (36).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.brilliant_pkg.all;
use work.all;

architecture brilliant of program_top is
    constant C_LAT  : integer := 36;   -- = processing_delay_clks

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
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    function f_il(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8 | 1 | 2 | 10 => return '1';
            when others             => return '0';
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- the Osc sample: (lattice - 64) * 3, clamped (three times the camera's level about the lattice's middle)
    function f_osc3(v : unsigned(7 downto 0)) return unsigned is
        variable d : unsigned(9 downto 0);
    begin
        if v < 64 then return to_unsigned(0, 8); end if;
        d := resize(v - 64, 10);
        d := shift_left(d, 1) + d;
        if d > 255 then return to_unsigned(255, 8); else return d(7 downto 0); end if;
    end function;
    -- a sample times a weight 0 .. 4
    function f_mulw(l : unsigned(7 downto 0); w : unsigned(2 downto 0)) return unsigned is
        variable a, b : unsigned(10 downto 0);
    begin
        if w(2) = '1' then a := resize(l & "00", 11);
        elsif w(1) = '1' then a := resize(l & '0', 11);
        else a := (others => '0'); end if;
        if w(0) = '1' then b := resize(l, 11); else b := (others => '0'); end if;
        return a + b;
    end function;
    -- |x| by one's complement (14 bits of a 15-bit signed value)
    function f_oc14(v : signed(14 downto 0)) return unsigned is
        variable r : unsigned(13 downto 0);
    begin
        for i in 0 to 13 loop r(i) := v(i) xor v(14); end loop;
        return r;
    end function;
    -- saturate to 0 .. 255
    function f_sat8(v : unsigned) return unsigned is
    begin
        if v > 255 then return to_unsigned(255, 8); else return resize(v, 8); end if;
    end function;
    -- saturate to 0 .. 31
    function f_sat5(v : unsigned) return unsigned is
    begin
        if v > 31 then return to_unsigned(31, 5); else return resize(v, 5); end if;
    end function;
    -- saturate to 0 .. 511
    function f_sat9(v : unsigned) return unsigned is
    begin
        if v > 511 then return to_unsigned(511, 9); else return resize(v, 9); end if;
    end function;
    -- saturate a signed value to +-512 (10 bits) by its top bits (no carry chain)
    function f_s10(v : signed) return signed is
        alias w : signed(v'length - 1 downto 0) is v;
        variable all0, all1 : boolean;
    begin
        all0 := true; all1 := true;
        for i in 9 to v'length - 1 loop
            if w(i) = '1' then all0 := false; else all1 := false; end if;
        end loop;
        if all0 or all1 then return w(9 downto 0);
        elsif w(v'length - 1) = '0' then return to_signed(511, 10);
        else return to_signed(-512, 10); end if;
    end function;
    -- coverage clamp(4 - e, 0, 8) of a 10-bit distance (1/8 unit), short carry chains only
    function f_cm(e : signed(9 downto 0)) return unsigned is
        variable v : signed(5 downto 0);
    begin
        if e(9 downto 4) = "000000" or e(9 downto 4) = "111111" then
            v := to_signed(4, 6) - e(5 downto 0);
            if v(5) = '1' then return to_unsigned(0, 4);
            elsif v(4) = '1' or unsigned(v(3 downto 0)) > 8 then return to_unsigned(8, 4);
            else return unsigned(v(3 downto 0)); end if;
        elsif e(9) = '0' then return to_unsigned(0, 4);
        else return to_unsigned(8, 4); end if;
    end function;
    -- clamp(4 + e, 0, 8)
    function f_cp(e : signed(9 downto 0)) return unsigned is
        variable v : signed(5 downto 0);
    begin
        if e(9 downto 4) = "000000" or e(9 downto 4) = "111111" then
            v := to_signed(4, 6) + e(5 downto 0);
            if v(5) = '1' then return to_unsigned(0, 4);
            elsif v(4) = '1' or unsigned(v(3 downto 0)) > 8 then return to_unsigned(8, 4);
            else return unsigned(v(3 downto 0)); end if;
        elsif e(9) = '0' then return to_unsigned(8, 4);
        else return to_unsigned(0, 4); end if;
    end function;
    -- a Fine line's coverage clamp(hw + 4 - |d|, 0, 8), |d| saturated to 63
    function f_ln(hw4 : unsigned(5 downto 0); d : unsigned(8 downto 0)) return unsigned is
        variable s : unsigned(5 downto 0);
        variable v : signed(6 downto 0);
    begin
        if d(8 downto 6) /= "000" then s := (others => '1'); else s := d(5 downto 0); end if;
        v := signed('0' & hw4) - signed('0' & s);
        if v(6) = '1' then return to_unsigned(0, 4);
        elsif v(5 downto 3) /= "000" then return to_unsigned(8, 4);
        else return unsigned(v(3 downto 0)); end if;
    end function;
    -- clamp to 0 .. 8
    function f_cl8(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 4);
        elsif v > 8 then return to_unsigned(8, 4);
        else return resize(unsigned(v), 4); end if;
    end function;
    -- |x| of a 10-bit signed value by one's complement
    function f_oc10(v : signed(9 downto 0)) return unsigned is
        variable r : unsigned(8 downto 0);
    begin
        for i in 0 to 8 loop r(i) := v(i) xor v(9); end loop;
        return r;
    end function;
    -- x * a / 8 for a in {8, 6, 4, 3, 2} (the engine snaps alpha to these), x 10-bit signed
    -- 3 x is built from the magnitude bits (3 xu - 1536 xs, x = xu - 512 xs): a 2 x + x on the sign-extended x put
    -- the sign net on both inputs of the top carry LUTs (dupin: 2 duplicate-input LUTs; router2 stalled at
    -- overused = 2 on HD HDMI, and router1 would hang on them).  Exact: same values as before.
    function f_mula(x : signed(9 downto 0); a : unsigned(3 downto 0)) return signed is
        variable p  : signed(13 downto 0);
        variable xu : unsigned(10 downto 0);
        variable t3 : signed(13 downto 0);
        variable sb : signed(13 downto 0);
    begin
        xu := resize(unsigned(x(8 downto 0)), 11);
        sb := (others => '0');
        if x(9) = '1' then sb := to_signed(-1536, 14); end if;
        t3 := signed(resize(shift_left(xu, 1) + xu, 14)) + sb;
        case to_integer(a) is
            when 8      => p := shift_left(resize(x, 14), 3);
            when 6      => p := shift_left(t3, 1);
            when 4      => p := shift_left(resize(x, 14), 2);
            when 3      => p := t3;
            when others => p := shift_left(resize(x, 14), 1);
        end case;
        return resize(shift_right(p, 3), 10);
    end function;
    -- a pair of 2-bit entries (sectors 2 m, 2 m + 1) of a 32-bit table word
    function f_t4(w : std_logic_vector(31 downto 0); m : unsigned(2 downto 0)) return unsigned is
        variable r : unsigned(3 downto 0);
    begin
        r := "0000";
        for i in 0 to 7 loop
            if to_integer(m) = i then r := unsigned(w(4 * i + 3 downto 4 * i)); end if;
        end loop;
        return r;
    end function;
    -- 2-bit entry k of a 32-bit table word
    function f_t2(w : std_logic_vector(31 downto 0); k : unsigned(3 downto 0)) return unsigned is
        variable r : unsigned(1 downto 0);
    begin
        r := "00";
        for i in 0 to 15 loop
            if to_integer(k) = i then r := unsigned(w(2 * i + 1 downto 2 * i)); end if;
        end loop;
        return r;
    end function;
    -- the low six bits of a blend result (always 0 .. 63)
    function f_y6(v : signed(10 downto 0)) return unsigned is
    begin
        return unsigned(v(5 downto 0));
    end function;
    -- the cell index: col + 3 row (4 bits): the Pave stones' light offsets and the dots' twinkles sweep in bands
    function f_hash(col, row : unsigned(7 downto 0)) return unsigned is
    begin
        return col(3 downto 0) + row(3 downto 0) + (row(2 downto 0) & '0');
    end function;
    attribute no_rw_check : boolean;

    -- radix-4 partial product: 0, k, 2 k or 3 k (k Q9, 12 bits; k3 = 3 k, 14 bits)
    function f_r4(k : signed(11 downto 0); k3 : signed(13 downto 0); d : unsigned(1 downto 0)) return signed is
    begin
        case d is
            when "00"   => return to_signed(0, 14);
            when "01"   => return resize(k, 14);
            when "10"   => return shift_left(resize(k, 14), 1);
            when others => return k3;
        end case;
    end function;
    -- a carry as a signed 0 / 1
    function f_c1(c : std_logic) return signed is
    begin
        if c = '1' then return to_signed(1, 18); else return to_signed(0, 18); end if;
    end function;


    -- [A] timing / zone / width / lines
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ilr, s_il : std_logic := '0';
    signal s_ush  : unsigned(1 downto 0) := "00";
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_lcnt, s_hn : unsigned(11 downto 0) := (others => '0');
    signal s_hv, z_hbig : std_logic := '0';
    signal s_bank : std_logic := '0';                              -- the bank being written this field

    -- [A] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- sync delay line in block RAM: hsync, vsync, the generated avid, C_LAT - 2 clocks (+ the output register)
    type t_sd is array (0 to 63) of std_logic_vector(2 downto 0);
    signal sd_ram : t_sd := (others => "011");
    attribute no_rw_check of sd_ram : signal is true;
    signal sd_wa, sd_ra : unsigned(5 downto 0) := (others => '0');
    signal sd_q : std_logic_vector(2 downto 0) := "011";
    signal sd_o : std_logic_vector(2 downto 0) := "011";
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 3) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] switches (the engine's conditions; the pixel path takes its own from the table)
    signal s_glide : std_logic := '1';
    signal s_pv, s_src : std_logic := '0';

    -- ROM: the engine's table
    signal r_ex_a : unsigned(9 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');

    -- [C] the per-field engine
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
    constant OP_SPC  : std_logic_vector(3 downto 0) := x"F";
    signal rf_ram : t_c_br_rfi := C_BR_RFI;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal e_run : std_logic := '0';
    signal e_ph  : unsigned(1 downto 0) := "11";
    signal e_pc  : unsigned(10 downto 0) := (others => '0');
    type t_lr is array (0 to 3) of unsigned(10 downto 0);
    signal e_lr  : t_lr := (others => (others => '0'));
    signal e_ir  : std_logic_vector(15 downto 0) := x"F001";
    signal e_irf : std_logic_vector(15 downto 0) := x"F001";
    signal x_mul, x_shr, x_halt, x_out, x_st : std_logic := '0';
    signal x_gate, x_subr, x_shift, x_cz : std_logic := '0';
    signal e_acc, e_b : signed(36 downto 0) := (others => '0');
    signal e_cnt : unsigned(4 downto 0) := (others => '0');
    signal e_out : std_logic;
    signal w_in  : unsigned(15 downto 0);
    -- the grid port's mailbox
    signal mb_addr : unsigned(10 downto 0) := (others => '0');
    signal mb_pend, mb_rdy : std_logic := '0';
    signal mb_data : unsigned(7 downto 0) := (others => '0');
    signal gq_tag  : std_logic_vector(1 downto 0) := "00";       -- engine reads in flight (address, data)
    signal gp_tag  : std_logic_vector(1 downto 0) := "00";       -- pixel-path reads in flight

    -- [C][E] the table: two banks of 64 words; the engine writes bank s_bank, the loader reads the other
    type t_dt is array (0 to 127) of std_logic_vector(31 downto 0);
    signal dt_ram : t_dt := (others => (others => '0'));
    attribute no_rw_check of dt_ram : signal is true;
    signal dt_q : std_logic_vector(31 downto 0) := (others => '0');
    signal dt_ra, dt_wa : unsigned(5 downto 0) := (others => '0');
    signal dt_we : std_logic := '0';

    -- [E] the field's constants
    signal c_xc0, c_wcm1, c_yc0, c_hcm1 : unsigned(11 downto 0) := (others => '0');
    signal c_col0, c_row0 : unsigned(7 downto 0) := (others => '0');
    signal c_bx0, c_by0 : signed(15 downto 0) := (others => '0');
    signal c_bxs, c_bys : unsigned(15 downto 0) := (others => '0');
    signal c_g1, c_g4 : unsigned(15 downto 0) := (others => '0');
    signal c_g2, c_g3 : signed(15 downto 0) := (others => '0');
    signal c_k2, c_k3 : signed(11 downto 0) := (others => '0');                 -- kappa, Q9
    signal c_k2x3, c_k3x3 : signed(13 downto 0) := (others => '0');             -- 3 kappa
    signal c_dy0, c_dyr, c_dx0, c_dxc : signed(15 downto 0) := (others => '0');
    signal c_al2, c_al3 : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal c_hw  : unsigned(5 downto 0) := (others => '0');
    signal c_dst : unsigned(7 downto 0) := (others => '0');
    signal c_don, c_pave, c_ink, c_fine : std_logic := '0';
    signal c_tph : unsigned(3 downto 0) := (others => '0');
    type t_u6x4 is array (0 to 3) of unsigned(5 downto 0);
    signal c_ramp : t_u6x4 := (others => (others => '0'));
    signal c_dl0, c_dl1 : unsigned(5 downto 0) := (others => '0');
    type t_u9x4 is array (0 to 3) of unsigned(8 downto 0);
    signal c_thr : t_u9x4 := (others => (others => '1'));
    signal f_run, f_v1, f_v2 : std_logic := '0';
    signal f_t, f_i1, f_i2 : unsigned(5 downto 0) := (others => '0');

    -- [E] DDAs (0 X, 1 Y) and their loader
    type t_s24x2 is array (0 to 1) of signed(23 downto 0);
    signal ca_inc, ca_incc, ca_acc : t_s24x2 := (others => (others => '0'));
    signal ca_ld, ca_en : std_logic_vector(1 downto 0) := (others => '0');
    signal ca_rst : std_logic := '0';
    signal l_kick, l_run, l_seed, l_dw : std_logic := '0';
    signal l_dly : unsigned(3 downto 0) := (others => '0');
    signal l_t : unsigned(1 downto 0) := (others => '0');
    signal l_wb : std_logic := '0';                                -- this line's wrap: the cell-row one
    signal l_v1, l_v2, l_o1, l_o2 : std_logic := '0';
    signal l_c1, l_c2 : std_logic := '0';
    signal b_run, b_end : std_logic := '0';
    signal b_cnt, b_mm1 : unsigned(11 downto 0) := (others => '0');
    signal xc   : unsigned(11 downto 0) := (others => '0');       -- pixels left in this cell after this one
    signal yc   : unsigned(11 downto 0) := (others => '0');       -- lines left in this cell row after this one
    signal x_wr : std_logic := '0';                                -- the first pixel of a new cell
    signal k_col, k_row : unsigned(7 downto 0) := (others => '0');
    signal k_bx  : signed(17 downto 0) := (others => '0');
    signal k_by  : signed(17 downto 0) := (others => '0');
    signal k_sq  : unsigned(2 downto 0) := (others => '0');       -- the blanking fetch of the line's first two cells
    signal k_sqr : std_logic := '0';
    signal k_r60 : unsigned(10 downto 0) := (others => '0');      -- the cell row's grid row x 60
    signal k_dys : signed(15 downto 0) := (others => '0');        -- the glitter's vertical offset, 1/512 vpx
    signal k_dy8 : unsigned(7 downto 0) := (others => '0');
    -- [E] per-cell camera fetch: the current cell and the next
    signal k_rq  : std_logic := '0';                               -- a pixel-path read this clock
    signal k_ra  : unsigned(10 downto 0) := (others => '0');
    signal k_cn  : std_logic := '0';                               -- the read is for the current cell (blanking)
    signal k_hc  : unsigned(7 downto 0) := (others => '0');       -- the cell column whose hash is taken
    signal kq_cn : std_logic_vector(1 downto 0) := "00";
    signal gr_f  : unsigned(7 downto 0) := (others => '0');
    signal kf_v, kf_cn : std_logic := '0';
    signal lev_c, lev_n : unsigned(2 downto 0) := (others => '0');
    signal h_c, h_n, h_k : unsigned(3 downto 0) := (others => '0');
    signal x_new : std_logic;
    signal xn_sr : std_logic_vector(1 to 15) := (others => '0');
    signal pl_c  : unsigned(4 downto 0) := (others => '0');       -- the first cell's copies: 24 clocks after the
    signal pl_r, pl_p : std_logic := '0';                          -- line end / kick (blanking)
    signal cl2  : unsigned(2 downto 0) := (others => '0');        -- level and hash a cell on (the first copy)
    signal ch2  : unsigned(3 downto 0) := (others => '0');
    signal cr16 : unsigned(10 downto 0) := (others => '0');       -- dot radius at S16
    signal cz17 : std_logic := '1';                                -- level 0 at S17
    signal ch17 : unsigned(3 downto 0) := (others => '0');        -- hash at S17
    signal cg9  : unsigned(15 downto 0) := (others => '0');       -- girdle offset at S9
    signal ch13 : unsigned(3 downto 0) := (others => '0');        -- hash at S13

    -- [P] fold
    signal p1_au, p1_av : unsigned(13 downto 0) := (others => '0');
    signal p1_q : std_logic_vector(2 downto 0) := (others => '0');            -- su, sv, sw
    signal p2_a0, p2_b0 : unsigned(13 downto 0) := (others => '0');
    signal p2_ka : unsigned(20 downto 0) := (others => '0');
    signal p2_kb : unsigned(17 downto 0) := (others => '0');
    signal p2_q, p3_q, p4_q : std_logic_vector(2 downto 0) := (others => '0');
    signal p3_k1, p4_k1 : unsigned(13 downto 0) := (others => '0');
    signal p3_a0, p3_b0, p4_a0, p4_b0, p5_a0, p6_a0, p7_a0 : unsigned(13 downto 0) := (others => '0');
    signal p5_bs, p6_bs, p7_bs : unsigned(13 downto 0) := (others => '0');   -- b0, or k1 when reflected
    signal p4_d  : signed(15 downto 0) := (others => '0');
    signal p5_m, p6_m : unsigned(13 downto 0) := (others => '0');
    signal p5_ma : unsigned(20 downto 0) := (others => '0');
    signal p5_mb : unsigned(17 downto 0) := (others => '0');
    signal p6_mab : unsigned(21 downto 0) := (others => '0');
    signal p7_k2 : unsigned(13 downto 0) := (others => '0');
    signal p5_rf, p6_rf, p7_rf : std_logic := '0';
    signal p5_d22, p6_d22, p7_d22, p8_d22, p9_d22, p10_d22, p11_d22, p12_d22 : unsigned(4 downto 0) := (others => '0');
    type t_q4 is array (5 to 12) of std_logic_vector(3 downto 0);
    signal pq : t_q4 := (others => (others => '0'));
    signal p8_a  : unsigned(14 downto 0) := (others => '0');
    signal p8_b  : unsigned(13 downto 0) := (others => '0');
    signal p9_d0, p10_d0, p11_d0, p12_d0 : unsigned(4 downto 0) := (others => '0');
    -- [P] edges
    signal p9_e1a : unsigned(19 downto 0) := (others => '0');
    signal p9_e1b : unsigned(16 downto 0) := (others => '0');
    signal p9_ag1, p9_ag4, p9_ag2, p9_ag3, p10_ag1, p10_ag4 : signed(17 downto 0) := (others => '0');
    signal p9_s01k2, p9_s23k2, p9_s45k2, p9_s01k3, p9_s23k3, p9_s45k3 : signed(16 downto 0) := (others => '0');
    signal p10_e1 : unsigned(20 downto 0) := (others => '0');
    signal p10_s2, p10_s3 : signed(20 downto 0) := (others => '0');
    signal p10_w2, p10_w3 : signed(19 downto 0) := (others => '0');
    signal p11_e1, p11_e4, p11_e2, p11_e3 : signed(17 downto 0) := (others => '0');
    signal p12_e1, p12_e4, p12_e2, p12_e3 : signed(9 downto 0) := (others => '0');
    signal p13_e2t, p13_e3t : signed(9 downto 0) := (others => '0');
    signal p13_ctab, p13_cbg, p13_c22, p13_c0n : unsigned(3 downto 0) := (others => '0');
    signal p13_l1, p13_l4, p13_l22 : unsigned(3 downto 0) := (others => '0');
    signal p13_e3p, p13_e4n, p13_e1n : std_logic := '0';
    signal p13_n0 : std_logic := '0';
    signal p13_j : unsigned(3 downto 0) := (others => '0');
    signal p14_cst, p14_cgd, p14_ctab, p14_cbg, p14_c22, p14_c0n : unsigned(3 downto 0) := (others => '0');
    signal p14_l1, p14_l2, p14_l3, p14_l4, p14_l22 : unsigned(3 downto 0) := (others => '0');
    signal p14_n0, p15_n0 : std_logic := '0';
    signal p15_lca, p15_lcb : unsigned(3 downto 0) := (others => '0');
    type t_c4 is array (15 to 16) of unsigned(3 downto 0);
    signal pc_st, pc_gd, pc_tab, pc_bg, pc_22, pc_0n : t_c4 := (others => (others => '0'));
    signal p16_lc : unsigned(3 downto 0) := (others => '0');
    signal p16_yk, p16_ys, p16_yg, p16_yg2, p16_yr, p16_yrn : unsigned(5 downto 0) := (others => '0');
    -- [P] glitter
    signal gr_sr : std_logic_vector(1 to 13) := (others => '0');
    signal xcg : unsigned(11 downto 0) := (others => '0');
    signal gu : signed(15 downto 0) := (others => '0');
    signal gdy : unsigned(7 downto 0) := (others => '0');
    signal q1_x, q1_y : unsigned(8 downto 0) := (others => '0');
    signal q2_s : unsigned(9 downto 0) := (others => '0');
    signal q3_eo : signed(11 downto 0) := (others => '0');
    signal q4_cd : unsigned(3 downto 0) := (others => '0');
    signal q4_yd : unsigned(5 downto 0) := (others => '0');
    signal qg18 : unsigned(5 downto 0) := (others => '0');

    -- [G] blend
    signal g17_da, g17_dt, g19_d, g21_d : signed(6 downto 0) := (others => '0');
    signal g17_ya, g17_yt : unsigned(5 downto 0) := (others => '0');
    signal g17_ca, g17_ct, g17_ctab, g17_cbg, g17_lc : unsigned(3 downto 0) := (others => '0');
    signal g18_ya, g18_yt : unsigned(5 downto 0) := (others => '0');
    signal g18_ctab, g18_cbg, g18_lc : unsigned(3 downto 0) := (others => '0');
    signal g19_y0 : unsigned(5 downto 0) := (others => '0');
    signal g19_c, g19_lc, g20_lc : unsigned(3 downto 0) := (others => '0');
    signal g20_y : unsigned(5 downto 0) := (others => '0');
    signal g21_y0 : unsigned(5 downto 0) := (others => '0');
    signal g21_c : unsigned(3 downto 0) := (others => '0');
    signal g21_ink : std_logic := '0';
    signal g22_y : unsigned(5 downto 0) := (others => '0');

    -- [D] source field and the grid
    signal r_vy : unsigned(12 downto 0) := (others => '0');
    signal r_rb : unsigned(10 downto 0) := (others => '0');        -- the source line's block row x 60
    signal r_first : std_logic := '1';
    type t_gr is array (0 to 4095) of unsigned(7 downto 0);
    signal gr_ram : t_gr := (others => (others => '0'));
    attribute no_rw_check of gr_ram : signal is true;
    signal gr_ra, gr_wa : unsigned(11 downto 0) := (others => '0');
    signal gr_q, gr_wd : unsigned(7 downto 0) := (others => '0');
    signal gr_we : std_logic := '0';
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl  : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_fl, s_seen : std_logic := '0';
    signal w_ph  : unsigned(5 downto 0) := (others => '0');
    signal w_l   : unsigned(7 downto 0) := (others => '0');
    signal w_wa, w_wb : unsigned(2 downto 0) := (others => '0');
    signal w_cy, w_v1, w_cy2, w_v2, w_dv, w_v4, w_v5, w_v6 : std_logic := '0';
    signal w_pa, w_pb : unsigned(10 downto 0) := (others => '0');
    signal w_acc, w_dump, w_dd, w_qd, w_qn : unsigned(13 downto 0) := (others => '0');
    signal w_col, w_dcol, w_c4, w_c5, w_c6 : unsigned(5 downto 0) := (others => '0');
    signal w_diff : signed(14 downto 0) := (others => '0');
    type t_iq is array (0 to 63) of unsigned(15 downto 0);
    signal iq_ram : t_iq := (others => (others => '0'));
    attribute no_rw_check of iq_ram : signal is true;
    signal iq_q, iq_f : unsigned(15 downto 0) := (others => '0');
    signal iq_we : std_logic := '0';
    signal w_qr : unsigned(7 downto 0) := (others => '0');

    -- [F] output conditioning
    signal s_y23, s_yd1, s_yd2, s_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ya : unsigned(11 downto 0) := (others => '0');
    signal s_y26, s_y27, s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_BR_TAB(to_integer(r_ex_a));
        end if;
    end process p_rom;

    p_dtram : process(clk)
    begin
        if rising_edge(clk) then
            dt_q <= dt_ram(to_integer((not s_bank) & dt_ra));
            if dt_we = '1' then dt_ram(to_integer(s_bank & dt_wa)) <= std_logic_vector(e_acc(31 downto 0)); end if;
        end if;
    end process p_dtram;

    -- camera grid: 2 banks x 34 rows x 60 blocks (address row x 60 + column)
    p_grram : process(clk)
    begin
        if rising_edge(clk) then
            gr_q <= gr_ram(to_integer(gr_ra));
            if gr_we = '1' then gr_ram(to_integer(gr_wa)) <= gr_wd; end if;
        end if;
    end process p_grram;

    -- the sync delay line: written every clock, read C_LAT - 2 writes later (never the address being written)
    p_sdram : process(clk)
    begin
        if rising_edge(clk) then
            sd_q <= sd_ram(to_integer(sd_ra));
            sd_ram(to_integer(sd_wa)) <= g_avid & data_in.vsync_n & data_in.hsync_n;
            sd_wa <= sd_wa + 1;
            sd_ra <= sd_wa - (C_LAT - 3);
            sd_o <= sd_q;
        end if;
    end process p_sdram;

    p_iqram : process(clk)
    begin
        if rising_edge(clk) then
            iq_q <= iq_ram(to_integer(w_col));
            if iq_we = '1' then iq_ram(to_integer(w_c5)) <= resize(w_qn, 16); end if;
        end if;
    end process p_iqram;

    ---------------------------------------------------------------------------
    -- [A] TIMING + [B] CONTROLS
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_vy : unsigned(13 downto 0);
        variable v_hd : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            s_ilr <= f_il(s_tid);

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            -- width (max over the field) and lines per field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;
            if g_start = '1' then s_lcnt <= s_lcnt + 1; end if;
            v_hd := signed(resize(s_lcnt, 13)) - signed(resize(s_hn, 13));
            if v_hd > 1 or v_hd < -1 then z_hbig <= '1'; else z_hbig <= '0'; end if;

            -- per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                if s_hv = '0' or z_hbig = '1' then s_hn <= s_lcnt; end if;
                s_hv <= '1'; s_lcnt <= (others => '0');
                s_bank  <= not s_bank;
                s_glide <= registers_in(6)(0);
                s_pv    <= registers_in(6)(1);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_il    <= s_ilr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                if s_xsr = 2 then s_ush <= '0' & s_ilr;
                elsif s_ilr = '1' then s_ush <= "10";
                else s_ush <= "01"; end if;
            end if;
            g_wm1 <= s_w - 1;
            b_mm1 <= s_w + (C_NEXT - 1);
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- [D] the source line's block row (x 60): 32 virtual lines per row
            if s_zone = '1' then
                r_vy <= (others => '0'); r_rb <= (others => '0'); r_first <= '1';
            elsif g_start = '1' then
                if r_first = '1' then
                    r_first <= '0';
                else
                    v_vy := ('0' & r_vy) + resize(s_ys, 14);
                    r_vy <= v_vy(12 downto 0);
                    if v_vy(13) = '1' then r_rb <= r_rb + 60; end if;
                end if;
            end if;

            -- sync and generated-avid delay (field_n is tied high by the core, so it only gets a register)
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 2);
        end if;
    end process p_ctrl;

    ------------------------------------------------------------------------
    -- [C] THE PER-FIELD ENGINE (Hardline flute's).  Started at every zone start from address 0; three clocks
    -- per micro-op, serial multiply (18 steps) and shift.  CALL pushes a four-deep link stack, RET pops it.
    -- OUT GA starts a read of the camera grid (the bank the pixel path reads) through a mailbox that takes
    -- the port on a clock the pixel path leaves free; SKIP GRDY waits for it; IN GD returns the byte.
    ------------------------------------------------------------------------
    with e_irf(3 downto 0) select w_in <=
        resize(s_ys, 16)            when x"8",
        resize(s_xs, 16)            when x"9",
        "000000000000000" & s_il    when x"A",
        r_ex_q                      when x"B",
        resize(s_w, 16)             when x"C",
        resize(mb_data, 16)         when x"D",
        resize(s_hn, 16)            when x"E",
        (others => '0')             when x"F",
        resize(unsigned(registers_in(to_integer(unsigned(e_irf(2 downto 0))))), 16) when others;

    e_out <= '1' when e_run = '1' and e_ph = "10" and x_out = '1' else '0';

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
                    when "11" =>
                        v_fetch := '1';
                        e_ph <= "00";
                    when "00" =>
                        e_irf <= e_ir;
                        e_ph <= "01";
                    when "01" =>
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
                        case v_arg(1 downto 0) is
                            when "00"   => v_cond := e_acc(36);
                            when "01"   => v_cond := s_glide;
                            when "10"   => v_cond := s_pv;
                            when others => v_cond := mb_rdy;
                        end case;
                        v_cond := v_cond xor v_arg(3);
                        if v_op = OP_SKIP and v_cond = '1' then e_pc <= e_pc + 2;
                        elsif v_op = OP_JMP then e_pc <= unsigned(v_arg(10 downto 0));
                        elsif v_op = OP_CALL then
                            e_pc <= unsigned(v_arg(10 downto 0));
                            e_lr <= (e_pc + 1) & e_lr(0 to 2);
                        elsif v_op = OP_SPC and v_arg(1 downto 0) = "00" then
                            e_pc <= e_lr(0);
                            e_lr <= e_lr(1 to 3) & e_lr(3);
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
                    when others =>
                        v_shift := x_shift; v_last := '1';
                        if x_gate = '0' then v_bx := (others => '0');
                        elsif x_subr = '1' then v_bx := not e_b;
                        else v_bx := e_b; end if;
                        v_t := (e_acc & '1') + (v_bx & (x_subr and x_gate));
                        v_sum := v_t(37 downto 1);
                        if x_mul = '1' or x_shr = '1' then
                            if x_cz = '0' then v_last := '0'; end if;
                            e_cnt <= e_cnt - 1;
                            if e_cnt = 1 then x_cz <= '1'; x_subr <= x_mul; end if;
                            if x_mul = '1' then x_gate <= v_sum(1); end if;
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
                e_ir <= C_BR_UC(to_integer(e_pc));
            end if;

            -- outputs: the ROM address, the table's write address and data, the grid mailbox
            dt_we <= '0';
            if dt_we = '1' then dt_wa <= dt_wa + 1; end if;
            if e_out = '1' then
                case e_irf(1 downto 0) is
                    when "00"   => r_ex_a <= unsigned(e_acc(9 downto 0));
                    when "01"   => dt_wa <= unsigned(e_acc(5 downto 0));
                    when "10"   => dt_we <= '1';                       -- (the write takes e_acc, unchanged a clock on)
                    when others => mb_addr <= unsigned(e_acc(10 downto 0)); mb_pend <= '1'; mb_rdy <= '0';
                end case;
            end if;
            -- the grid port: the pixel path's read has priority; the mailbox read takes a free clock
            gq_tag <= gq_tag(0) & '0';
            if k_rq = '1' then
                gr_ra <= (not s_bank) & k_ra;
            elsif mb_pend = '1' then
                gr_ra <= (not s_bank) & mb_addr;
                mb_pend <= '0'; gq_tag(0) <= '1';
            end if;
            if gq_tag(1) = '1' then mb_data <= gr_q; mb_rdy <= '1'; end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- [A] Generated avid (parity-locked start 3/4 clocks after the source's avid rise, field-latched width)
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
    -- [E] DDAS AND LATTICE.  Four accumulators (Q12 of the anti-aliasing unit), each with its own increment
    -- register: the pixel step on every clock of the line window (width + C_NEXT steps; the lattice's u, v take
    -- the step at a cell wrap on the clock that crosses into the next cell), then one add of the line wrap
    -- parked in the same register.  The zone start loads the field's constants and seeds every accumulator
    -- from the bank the engine finished during the field before.
    ---------------------------------------------------------------------------
    -- a new cell for the stage-aligned copies: every cell wrap, and the line's first cell in the blanking (pl_p, 24
    -- clocks after the line end or the kick: the last line's pixels are past S17, the blanking fetch has set
    -- lev_c / h_c).  (Copying the first cell at the line start failed when it is shorter than the 15-clock copy
    -- chain: 720p / NTSC put a 4-pixel first glitter cell at the left edge, which took the second cell's twinkle.)
    x_new <= pl_p or x_wr;

    p_load : process(clk)
        variable v_col : unsigned(5 downto 0);
        variable v_bx : signed(17 downto 0);
        variable v_lv : unsigned(2 downto 0);
        variable v_r  : unsigned(10 downto 0);
        variable v_row : unsigned(5 downto 0);
        variable v_dy : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            -- line window
            b_end <= '0';
            if s_st(3) = '1' then
                b_run <= '1'; b_cnt <= (others => '0');
            elsif b_run = '1' then
                b_cnt <= b_cnt + 1;
                if b_cnt = b_mm1 then b_run <= '0'; b_end <= '1'; end if;
            end if;

            -- the zone start: words 0 .. 44 (the constants), then the seeds
            if s_zgo = '1' then
                f_run <= '1'; f_t <= (others => '0');
            elsif f_run = '1' then
                f_t <= f_t + 1;
                if f_t = 44 then f_run <= '0'; end if;
            end if;
            f_v1 <= f_run; f_i1 <= f_t; f_v2 <= f_v1; f_i2 <= f_i1;
            l_kick <= '0';
            if f_v2 = '1' then
                case to_integer(f_i2) is
                    when 1  => ca_incc(0) <= signed(dt_q(23 downto 0));
                    when 9  => ca_incc(1) <= signed(dt_q(23 downto 0));
                    when 24 => c_xc0 <= unsigned(dt_q(11 downto 0)); c_wcm1 <= unsigned(dt_q(23 downto 12));
                    when 25 => c_yc0 <= unsigned(dt_q(11 downto 0)); c_hcm1 <= unsigned(dt_q(23 downto 12));
                    when 26 => c_col0 <= unsigned(dt_q(7 downto 0)); c_row0 <= unsigned(dt_q(15 downto 8));
                    when 27 => c_bx0 <= signed(dt_q(15 downto 0)); c_bxs <= unsigned(dt_q(31 downto 16));
                    when 28 => c_by0 <= signed(dt_q(15 downto 0)); c_bys <= unsigned(dt_q(31 downto 16));
                    when 29 => c_g1 <= unsigned(dt_q(15 downto 0)); c_g4 <= unsigned(dt_q(31 downto 16));
                    when 30 => c_g2 <= signed(dt_q(15 downto 0)); c_g3 <= signed(dt_q(31 downto 16));
                    when 31 => c_k2 <= signed(dt_q(11 downto 0)); c_k3 <= signed(dt_q(27 downto 16));
                    when 32 =>
                        c_al2 <= unsigned(dt_q(3 downto 0)); c_al3 <= unsigned(dt_q(7 downto 4));
                        c_hw <= unsigned(dt_q(13 downto 8)); c_dst <= unsigned(dt_q(21 downto 14));
                        c_don <= dt_q(22); c_pave <= dt_q(23); c_ink <= dt_q(24); c_fine <= dt_q(25);
                        for i in 0 to 3 loop c_thr(i)(8) <= dt_q(26 + i); end loop;
                    when 38 => c_tph <= unsigned(dt_q(3 downto 0));
                    when 39 =>
                        c_ramp(0) <= unsigned(dt_q(5 downto 0)); c_ramp(1) <= unsigned(dt_q(13 downto 8));
                        c_ramp(2) <= unsigned(dt_q(21 downto 16)); c_ramp(3) <= unsigned(dt_q(29 downto 24));
                    when 40 =>
                        c_dl0 <= unsigned(dt_q(5 downto 0)); c_dl1 <= unsigned(dt_q(13 downto 8));
                    when 41 =>
                        c_thr(0)(7 downto 0) <= unsigned(dt_q(7 downto 0)); c_thr(1)(7 downto 0) <= unsigned(dt_q(15 downto 8));
                        c_thr(2)(7 downto 0) <= unsigned(dt_q(23 downto 16)); c_thr(3)(7 downto 0) <= unsigned(dt_q(31 downto 24));
                    when 42 => c_k2x3 <= signed(dt_q(13 downto 0)); c_k3x3 <= signed(dt_q(29 downto 16));
                    when 43 => c_dy0 <= signed(dt_q(15 downto 0)); c_dyr <= signed(dt_q(31 downto 16));
                    when 44 => c_dx0 <= signed(dt_q(15 downto 0)); c_dxc <= signed(dt_q(31 downto 16)); l_kick <= '1';
                    when others => null;
                end case;
            end if;

            -- lattice: the cell row (at the kick, then at every line end); the glitter's vertical offset
            if l_kick = '1' then
                yc <= c_yc0; k_row <= c_row0; k_by <= resize(c_by0, 18); l_wb <= '0'; k_dys <= c_dy0;
            elsif b_end = '1' then
                if yc = 0 then
                    l_wb <= '1'; yc <= c_hcm1; k_row <= k_row + 1; k_by <= k_by + signed(resize(c_bys, 18));
                    k_dys <= c_dyr;
                else
                    l_wb <= '0'; yc <= yc - 1;
                    k_dys <= k_dys + signed(resize(s_ys & '0', 16));
                end if;
            end if;
            if k_by(17) = '1' then v_row := (others => '0');
            elsif unsigned(k_by(16 downto 10)) > 33 then v_row := to_unsigned(33, 6);
            else v_row := resize(unsigned(k_by(16 downto 10)), 6); end if;
            k_r60 <= resize(v_row & "000000", 11) - resize(v_row & "00", 11);
            if k_dys(15) = '1' then v_dy := -k_dys; else v_dy := k_dys; end if;
            k_dy8 <= f_sat8(unsigned(v_dy(14 downto 6)));

            -- lattice: the cell column along the line
            x_wr <= '0';
            if b_run = '0' then
                xc <= c_xc0; k_col <= c_col0; k_bx <= resize(c_bx0, 18);
            elsif xc = 0 then
                xc <= c_wcm1; k_col <= k_col + 1; k_bx <= k_bx + signed(resize(c_bxs, 18)); x_wr <= '1';
            else
                xc <= xc - 1;
            end if;

            -- the per-cell camera fetch: in the line's blanking the first two cells (3 and 4 clocks after the
            -- line end or the kick, when the cell row is settled), during the line the next cell at xc = 5
            if l_kick = '1' or b_end = '1' then k_sqr <= '1'; k_sq <= (others => '0');
            elsif k_sqr = '1' then
                k_sq <= k_sq + 1;
                if k_sq = 4 then k_sqr <= '0'; end if;
            end if;
            pl_p <= '0';
            if l_kick = '1' or b_end = '1' then pl_r <= '1'; pl_c <= (others => '0');
            elsif pl_r = '1' then
                pl_c <= pl_c + 1;
                if pl_c = 22 then pl_r <= '0'; pl_p <= '1'; end if;
            end if;
            k_rq <= '0'; k_cn <= '0';
            if k_sqr = '1' and k_sq = 3 then
                v_bx := resize(c_bx0, 18); k_rq <= '1'; k_cn <= '1'; k_hc <= c_col0;
            elsif k_sqr = '1' and k_sq = 4 then
                v_bx := resize(c_bx0, 18) + signed(resize(c_bxs, 18)); k_rq <= '1'; k_hc <= c_col0 + 1;
            else
                v_bx := k_bx + signed(resize(c_bxs, 18));
                if b_run = '1' and xc = 5 then k_rq <= '1'; k_hc <= k_col + 1; end if;
            end if;
            if v_bx(17) = '1' then v_col := (others => '0');
            elsif unsigned(v_bx(16 downto 10)) > 59 then v_col := to_unsigned(59, 6);
            else v_col := resize(unsigned(v_bx(16 downto 10)), 6); end if;
            k_ra <= k_r60 + resize(v_col, 11);
            h_k <= f_hash(k_hc, k_row);                      -- (the index of the cell read)
            gp_tag <= gp_tag(0) & k_rq;
            kq_cn <= kq_cn(0) & k_cn;
            gr_f <= gr_q; kf_v <= gp_tag(1); kf_cn <= kq_cn(1);
            if kf_v = '1' then
                v_lv := (others => '0');
                for i in 0 to 3 loop
                    if resize(gr_f, 9) >= c_thr(i) then v_lv := v_lv + 1; end if;
                end loop;
                if kf_cn = '1' then lev_c <= v_lv; else lev_n <= v_lv; end if;
            end if;
            if gp_tag(0) = '1' then                          -- (h_k is the index of the cell requested a clock ago)
                if kq_cn(0) = '1' then h_c <= h_k; else h_n <= h_k; end if;
            end if;
            if b_run = '1' and xc = 0 then lev_c <= lev_n; h_c <= h_n; end if;
            -- stage-aligned copies (a cell lasts at least 16 clocks)
            xn_sr <= x_new & xn_sr(1 to 14);
            if xn_sr(2) = '1' then cl2 <= lev_c; ch2 <= h_c; end if;
            v_r := resize(cl2 * to_unsigned(41, 6), 11);           -- the dot radius, 41 / 8 virtual px per level (x sqrt 2)
            if xn_sr(14) = '1' then cr16 <= v_r; end if;
            if xn_sr(15) = '1' then
                if cl2 = 0 then cz17 <= '1'; else cz17 <= '0'; end if;
                ch17 <= ch2;
            end if;
            if xn_sr(7) = '1' then
                case to_integer(lev_c) is
                    when 0      => cg9 <= (others => '0');
                    when 1      => cg9 <= shift_right(c_g4, 2);
                    when 2      => cg9 <= shift_right(c_g4, 1);
                    when 3      => cg9 <= shift_right(c_g4, 1) + shift_right(c_g4, 2);
                    when others => cg9 <= c_g4;
                end case;
                if c_pave = '0' then cg9 <= c_g4; end if;
            end if;
            if xn_sr(11) = '1' then ch13 <= h_c; end if;

            -- loader: (seed or wrap), (step) for X then Y
            ca_rst <= l_kick;
            -- (the line's loads wait 16 clocks: until then the pixel path reads its sector words from the table)
            if b_end = '1' then l_dly <= (others => '0'); l_dw <= '1';
            elsif l_dw = '1' then l_dly <= l_dly + 1; if l_dly = 15 then l_dw <= '0'; end if; end if;
            if l_kick = '1' then
                l_run <= '1'; l_t <= (others => '0'); l_seed <= '1';
            elsif l_dw = '1' and l_dly = 15 then
                l_run <= '1'; l_t <= (others => '0'); l_seed <= '0';
            elsif l_run = '1' then
                l_t <= l_t + 1;
                if l_t = 3 then l_run <= '0'; end if;
            end if;
            if f_run = '1' then
                dt_ra <= f_t;
            elsif l_run = '0' then
                dt_ra <= "11" & p13_j;                                     -- the pixel's sector word (48 + j)
            elsif l_t(0) = '1' then
                dt_ra <= "00" & l_t(1) & "000";                              -- the step: 0 / 8
            elsif l_seed = '1' then
                dt_ra <= "00" & l_t(1) & "100";                              -- the seed: 4 / 12
            else
                dt_ra <= "00" & l_t(1) & "01" & l_wb;                        -- the wrap: 2 or 3 / 10 or 11
            end if;
            l_v1 <= l_run; l_c1 <= l_t(1); l_o1 <= l_t(0);
            l_v2 <= l_v1;  l_c2 <= l_c1; l_o2 <= l_o1;
            for k in 0 to 1 loop
                if l_v1 = '1' and l_c1 = to_unsigned(k, 1)(0) then ca_ld(k) <= '1'; else ca_ld(k) <= '0'; end if;
                if l_v2 = '1' and l_o2 = '0' and l_c2 = to_unsigned(k, 1)(0) then ca_en(k) <= '1'; else ca_en(k) <= '0'; end if;
            end loop;
        end if;
    end process p_load;

    g_dda : for d in 0 to 1 generate
        p_dda : process(clk)
            variable v_i : signed(23 downto 0);
        begin
            if rising_edge(clk) then
                if ca_ld(d) = '1' then ca_inc(d) <= signed(dt_q(23 downto 0)); end if;
                if b_run = '1' and xc = 0 then v_i := ca_incc(d); else v_i := ca_inc(d); end if;
                if ca_rst = '1' then
                    ca_acc(d) <= (others => '0');
                elsif b_run = '1' or ca_en(d) = '1' then
                    ca_acc(d) <= ca_acc(d) + v_i;
                end if;
            end if;
        end process p_dda;
    end generate g_dda;

    ---------------------------------------------------------------------------
    -- [P] FACETS.  S1 .. S8: the fold to wedge coordinates (1/8 unit); S9 .. S14: the five edges, their
    -- coverages (9 levels) and the Fine lines; the tone path (sector -> angular index -> the table).
    ---------------------------------------------------------------------------
    p_fold : process(clk)
        variable v_x, v_y : signed(14 downto 0);
        variable v_au, v_av : unsigned(13 downto 0);
        variable v_d : signed(14 downto 0);
        variable v_a0 : unsigned(13 downto 0);
        variable v_m : unsigned(13 downto 0);
        variable v_bb : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            -- S1: |X|, |Y| and which is larger (Pave: the lattice's u, v; Stone: the great stone's U, V)
            v_x := ca_acc(0)(23 downto 9); v_y := ca_acc(1)(23 downto 9);
            v_au := f_oc14(v_x); v_av := f_oc14(v_y);
            p1_au <= v_au; p1_av <= v_av;
            v_d := signed('0' & v_au) - signed('0' & v_av);
            p1_q <= v_x(14) & v_y(14) & v_d(14);
            -- S2: a0 = max, b0 = min; 106 a0 in two halves
            if p1_q(0) = '1' then v_a0 := p1_av; p2_b0 <= p1_au; else v_a0 := p1_au; p2_b0 <= p1_av; end if;
            p2_a0 <= v_a0;
            p2_ka <= shift_left(resize(v_a0, 21), 6) + shift_left(resize(v_a0, 21), 5);
            p2_kb <= shift_left(resize(v_a0, 18), 3) + shift_left(resize(v_a0, 18), 1);
            p2_q <= p1_q;
            -- S3: k1 = 106 a0 / 256
            p3_k1 <= resize(shift_right(p2_ka + resize(p2_kb, 21), 8), 14);
            p3_a0 <= p2_a0; p3_b0 <= p2_b0; p3_q <= p2_q;
            -- S4: D = k1 - b0 (the signed distance to the 22.5-degree line, x 1 / cos 22.5)
            p4_d <= signed(resize(p3_k1, 16)) - signed(resize(p3_b0, 16));
            p4_k1 <= p3_k1; p4_a0 <= p3_a0; p4_b0 <= p3_b0; p4_q <= p3_q;
            -- S5: reflected when D < 0; M = -D - 1; 181 M in two halves; |D|
            v_m := unsigned(not p4_d(13 downto 0));
            p5_m <= v_m; p5_rf <= p4_d(15);
            p5_ma <= shift_left(resize(v_m, 21), 7) + shift_left(resize(v_m, 21), 5);
            p5_mb <= shift_left(resize(v_m, 18), 4) + shift_left(resize(v_m, 18), 2);
            if p4_d(15) = '1' then
                p5_d22 <= f_sat5(resize(v_m, 15));
            else
                p5_d22 <= f_sat5(unsigned(p4_d(14 downto 0)));
            end if;
            p5_a0 <= p4_a0;
            if p4_d(15) = '1' then p5_bs <= p4_k1; else p5_bs <= p4_b0; end if;
            pq(5) <= p4_q & p4_d(15);
            -- S6, S7: k2 = 181 M / 256
            p6_mab <= resize(p5_ma, 22) + resize(p5_mb, 22);
            p6_m <= p5_m; p6_rf <= p5_rf; p6_d22 <= p5_d22; p6_bs <= p5_bs; p6_a0 <= p5_a0;
            p7_k2 <= resize(shift_right(p6_mab + resize(p6_m, 22), 8), 14);
            p7_rf <= p6_rf; p7_d22 <= p6_d22; p7_bs <= p6_bs; p7_a0 <= p6_a0;
            -- S8: the reflection p - 2 d n: a = a0 + k2, b = k1 - k2 (at least 0)
            if p7_rf = '1' then
                p8_a <= resize(p7_a0, 15) + resize(p7_k2, 15);
                v_bb := signed(resize(p7_bs, 16)) - signed(resize(p7_k2, 16));
                if v_bb(15) = '1' then p8_b <= (others => '0'); else p8_b <= unsigned(v_bb(13 downto 0)); end if;
            else
                p8_a <= resize(p7_a0, 15); p8_b <= p7_bs;
            end if;
            p8_d22 <= p7_d22;
            for k in 6 to 12 loop pq(k) <= pq(k - 1); end loop;
        end if;
    end process p_fold;

    p_edge : process(clk)
        variable v_bs : unsigned(11 downto 0);
        variable v_jl, v_j, v_j0 : unsigned(3 downto 0);
        variable v_q : std_logic_vector(3 downto 0);
        variable v_sp : std_logic;
        variable v_l, v_m : unsigned(3 downto 0);
        variable v_hw4 : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            -- S9: the edges' parts (a minus the offsets; 106 b and 51 b in halves; kappa b in radix 4)
            p9_e1a <= shift_left(resize(p8_b, 20), 6) + shift_left(resize(p8_b, 20), 5);
            p9_e1b <= shift_left(resize(p8_b, 17), 3) + shift_left(resize(p8_b, 17), 1);
            p9_ag1 <= signed(resize(p8_a, 18)) - signed(resize(c_g1, 18));
            p9_ag4 <= signed(resize(p8_a, 18)) - signed(resize(cg9, 18));
            p9_ag2 <= signed(resize(p8_a, 18)) - resize(c_g2, 18);
            p9_ag3 <= signed(resize(p8_a, 18)) - resize(c_g3, 18);
            if p8_b > 4095 then v_bs := (others => '1'); else v_bs := p8_b(11 downto 0); end if;
            p9_s01k2 <= resize(f_r4(c_k2, c_k2x3, v_bs(1 downto 0)), 17) + shift_left(resize(f_r4(c_k2, c_k2x3, v_bs(3 downto 2)), 17), 2);
            p9_s23k2 <= resize(f_r4(c_k2, c_k2x3, v_bs(5 downto 4)), 17) + shift_left(resize(f_r4(c_k2, c_k2x3, v_bs(7 downto 6)), 17), 2);
            p9_s45k2 <= resize(f_r4(c_k2, c_k2x3, v_bs(9 downto 8)), 17) + shift_left(resize(f_r4(c_k2, c_k2x3, v_bs(11 downto 10)), 17), 2);
            p9_s01k3 <= resize(f_r4(c_k3, c_k3x3, v_bs(1 downto 0)), 17) + shift_left(resize(f_r4(c_k3, c_k3x3, v_bs(3 downto 2)), 17), 2);
            p9_s23k3 <= resize(f_r4(c_k3, c_k3x3, v_bs(5 downto 4)), 17) + shift_left(resize(f_r4(c_k3, c_k3x3, v_bs(7 downto 6)), 17), 2);
            p9_s45k3 <= resize(f_r4(c_k3, c_k3x3, v_bs(9 downto 8)), 17) + shift_left(resize(f_r4(c_k3, c_k3x3, v_bs(11 downto 10)), 17), 2);
            p9_d22 <= p8_d22;
            p9_d0 <= f_sat5(p8_b);
            -- S10: kappa b = s0123 + s45 2^8; E = ag + floor(kappa b / 512) = (w >> 1) + (s0123 >> 9) + (w(0) and s0123(8))
            -- with w = 2 ag + s45
            p10_e1 <= resize(p9_e1a, 21) + resize(p9_e1b, 21);
            p10_s2 <= resize(p9_s01k2, 21) + shift_left(resize(p9_s23k2, 21), 4);
            p10_s3 <= resize(p9_s01k3, 21) + shift_left(resize(p9_s23k3, 21), 4);
            p10_w2 <= shift_left(resize(p9_ag2, 20), 1) + resize(p9_s45k2, 20);
            p10_w3 <= shift_left(resize(p9_ag3, 20), 1) + resize(p9_s45k3, 20);
            p10_ag1 <= p9_ag1; p10_ag4 <= p9_ag4;
            p10_d22 <= p9_d22; p10_d0 <= p9_d0;
            -- S11
            p11_e1 <= p10_ag1 + signed(resize(shift_right(p10_e1, 8), 18));
            p11_e4 <= p10_ag4 + signed(resize(shift_right(p10_e1, 9), 18));   -- the girdle: 53 / 256 b (11.7 degrees)
            p11_e2 <= resize(shift_right(p10_w2, 1), 18) + resize(shift_right(p10_s2, 9), 18) + f_c1(p10_w2(0) and p10_s2(8));
            p11_e3 <= resize(shift_right(p10_w3, 1), 18) + resize(shift_right(p10_s3, 9), 18) + f_c1(p10_w3(0) and p10_s3(8));
            p11_d22 <= p10_d22; p11_d0 <= p10_d0;
            -- S12: saturate to +-64 units
            p12_e1 <= f_s10(p11_e1); p12_e4 <= f_s10(p11_e4); p12_e2 <= f_s10(p11_e2); p12_e3 <= f_s10(p11_e3);
            p12_d22 <= p11_d22; p12_d0 <= p11_d0;
            -- S13: kappa edges' distances (x alpha); the other coverages and lines; the angular sector
            p13_e2t <= f_mula(p12_e2, c_al2);
            p13_e3t <= f_mula(p12_e3, c_al3);
            p13_ctab <= f_cm(p12_e1);
            p13_cbg  <= f_cp(p12_e4);
            p13_c22  <= f_cm(signed(resize(p12_d22, 10)));
            if p12_d0 < p12_d22 then
                p13_n0 <= '1'; p13_c0n <= f_cm(signed(resize(p12_d0, 10)));
            else
                p13_n0 <= '0'; p13_c0n <= f_cm(signed(resize(p12_d22, 10)));
            end if;
            v_hw4 := c_hw + 4;
            p13_l1  <= f_ln(v_hw4, f_oc10(p12_e1));
            p13_l4  <= f_ln(v_hw4, f_oc10(p12_e4));
            p13_l22 <= f_ln(v_hw4, resize(p12_d22, 9));
            if p12_e3(9) = '0' and p12_e3 /= 0 then p13_e3p <= '1'; else p13_e3p <= '0'; end if;
            p13_e4n <= p12_e4(9); p13_e1n <= p12_e1(9);
            -- the angular sector j = 4 quadrant + octant / wedge order, + the Pave stone's own offset
            v_q := pq(12);                                   -- su, sv, sw, rf
            v_sp := v_q(1) xor (v_q(3) xor v_q(2));
            v_jl := "00" & v_sp & (v_q(0) xor v_sp);
            v_j := unsigned'(v_q(2) & (v_q(3) xor v_q(2)) & "00") + v_jl;
            if c_pave = '1' then v_j := v_j + (ch13(2 downto 0) & '0'); end if;
            p13_j <= v_j;
            -- S14
            v_hw4 := c_hw + 4;
            p14_cst <= f_cm(p13_e2t);
            p14_cgd <= f_cp(p13_e3t);
            p14_l2 <= f_ln(v_hw4, f_oc10(p13_e2t));
            p14_l3 <= f_ln(v_hw4, f_oc10(p13_e3t));
            p14_l1 <= p13_l1; p14_l4 <= p13_l4;
            if (p13_e3p = '1' and p13_e4n = '1') or p13_e1n = '1' then p14_l22 <= p13_l22; else p14_l22 <= (others => '0'); end if;
            p14_ctab <= p13_ctab; p14_cbg <= p13_cbg; p14_c22 <= p13_c22; p14_c0n <= p13_c0n;
            p14_n0 <= p13_n0;                                -- (the sector word of p13_j is read: dt_q at S15)
            -- S15: the tones; the lines' union (two halves)
            p15_n0 <= p14_n0;
            if c_fine = '1' then
                v_l := p14_l1; if p14_l2 > v_l then v_l := p14_l2; end if;
                if p14_l3 > v_l then v_l := p14_l3; end if;
                v_m := p14_l4; if p14_l22 > v_m then v_m := p14_l22; end if;
            elsif c_ink = '1' then
                v_l := (others => '0'); v_m := p14_l4;
            else
                v_l := (others => '0'); v_m := (others => '0');
            end if;
            p15_lca <= v_l; p15_lcb <= v_m;
            pc_st(15) <= p14_cst; pc_gd(15) <= p14_cgd; pc_tab(15) <= p14_ctab; pc_bg(15) <= p14_cbg;
            pc_22(15) <= p14_c22; pc_0n(15) <= p14_c0n;
            -- S16: lumas through the ramp
            -- (the sector word: kite, star, girdle, girdle j^1, ray, ray j^1, ray j0)
            p16_yk <= c_ramp(to_integer(unsigned(dt_q(1 downto 0)))); p16_ys <= c_ramp(to_integer(unsigned(dt_q(3 downto 2))));
            p16_yg <= c_ramp(to_integer(unsigned(dt_q(5 downto 4)))); p16_yg2 <= c_ramp(to_integer(unsigned(dt_q(7 downto 6))));
            p16_yr <= c_ramp(to_integer(unsigned(dt_q(9 downto 8))));
            if p15_n0 = '1' then p16_yrn <= c_ramp(to_integer(unsigned(dt_q(13 downto 12))));
            else p16_yrn <= c_ramp(to_integer(unsigned(dt_q(11 downto 10)))); end if;
            if p15_lca > p15_lcb then p16_lc <= p15_lca; else p16_lc <= p15_lcb; end if;
            pc_st(16) <= pc_st(15); pc_gd(16) <= pc_gd(15); pc_tab(16) <= pc_tab(15); pc_bg(16) <= pc_bg(15);
            pc_22(16) <= pc_22(15); pc_0n(16) <= pc_0n(15);
        end if;
    end process p_edge;

    -- the glitter dots (Stone with a camera): an octagon per lattice cell, its size the cell's camera level
    p_glit : process(clk)
        variable v_u : signed(15 downto 0);
        variable v_p : unsigned(9 downto 0);
        variable v_e : signed(11 downto 0);
        variable v_tw : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- S13: a replica of the cell counter 13 clocks late, and the dot's horizontal offset (1/8 vpx):
            -- (wc - 1 - 2 xc) 2 xs, stepping 4 xs a pixel, from (1 - wc) 2 xs at each cell's first pixel
            gr_sr <= b_run & gr_sr(1 to 12);
            if gr_sr(13) = '0' then
                xcg <= c_xc0; gu <= c_dx0;
            else
                if xcg = 0 then xcg <= c_wcm1; gu <= c_dxc;
                else xcg <= xcg - 1; gu <= gu + signed(resize(s_xs & "00", 16)); end if;
            end if;
            if gr_sr(13) = '0' then gdy <= k_dy8; end if;            -- the line's vertical offset
            -- S14: |dx|, dy
            if gu(15) = '1' then v_u := -gu; else v_u := gu; end if;
            q1_x <= f_sat9(unsigned(v_u(14 downto 0)));
            q1_y <= resize(gdy, 9);
            -- S15: a diamond (lozenge): |dx| + |dy|
            q2_s <= resize(q1_x, 10) + resize(q1_y, 10);
            -- S16: less the dot radius, in anti-aliasing units (1/8)
            v_e := signed(resize(q2_s, 12)) - signed(resize(cr16, 12));
            case s_ush is
                when "00"   => q3_eo <= v_e;
                when "01"   => q3_eo <= shift_right(v_e, 1);
                when others => q3_eo <= shift_right(v_e, 2);
            end case;
            -- S17
            if cz17 = '1' or c_don = '0' or c_pave = '1' then q4_cd <= (others => '0');
            else q4_cd <= f_cm(f_s10(q3_eo)); end if;
            v_tw := ch17 - c_tph;
            if v_tw(3 downto 1) = "000" then q4_yd <= c_dl1; else q4_yd <= c_dl0; end if;
            -- S18: the ground's luma
            v_p := resize(q4_yd * q4_cd, 10);
            qg18 <= v_p(8 downto 3);
        end if;
    end process p_glit;

    ---------------------------------------------------------------------------
    -- [G] BLEND (two stages per level: the difference, then y0 + d c / 8).  L1: the girdle pair and the
    -- pavilion ray across their fold lines; L2: kite with star or girdle, the tri with the ray; L3: that with
    -- the table or the ground (the table edge and the girdle never share a pixel); L4: Fine lines / Ink.
    ---------------------------------------------------------------------------
    p_blend : process(clk)
        variable v_p : signed(10 downto 0);
        variable v_y : signed(10 downto 0);
        variable v_l : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S17: L1, kite with the star or the girdle (inside the girdle: the girdle pair across its fold line);
            -- the pavilion ray with its neighbour across the nearer fold line
            if pc_gd(16) = 8 and pc_22(16) /= 0 then
                g17_da <= signed(resize(p16_yg2, 7)) - signed(resize(p16_yg, 7)); g17_ya <= p16_yg; g17_ca <= pc_22(16);
            elsif pc_gd(16) /= 0 then
                g17_da <= signed(resize(p16_yg, 7)) - signed(resize(p16_yk, 7)); g17_ya <= p16_yk; g17_ca <= pc_gd(16);
            else
                g17_da <= signed(resize(p16_ys, 7)) - signed(resize(p16_yk, 7)); g17_ya <= p16_yk; g17_ca <= pc_st(16);
            end if;
            g17_dt <= signed(resize(p16_yrn, 7)) - signed(resize(p16_yr, 7)); g17_yt <= p16_yr; g17_ct <= pc_0n(16);
            g17_ctab <= pc_tab(16); g17_cbg <= pc_bg(16); g17_lc <= p16_lc;
            -- S18
            v_p := resize(g17_da * signed('0' & g17_ca), 11);
            g18_ya <= f_y6(signed(resize(g17_ya, 11)) + shift_right(v_p, 3));
            v_p := resize(g17_dt * signed('0' & g17_ct), 11);
            g18_yt <= f_y6(signed(resize(g17_yt, 11)) + shift_right(v_p, 3));
            g18_ctab <= g17_ctab; g18_cbg <= g17_cbg; g18_lc <= g17_lc;
            -- S19: L2, the table, or the ground outside the girdle (the table edge and the girdle never share a pixel)
            if g18_cbg /= 0 then
                g19_d <= signed(resize(qg18, 7)) - signed(resize(g18_ya, 7)); g19_c <= g18_cbg;
            else
                g19_d <= signed(resize(g18_yt, 7)) - signed(resize(g18_ya, 7)); g19_c <= g18_ctab;
            end if;
            g19_y0 <= g18_ya; g19_lc <= g18_lc;
            -- S20
            v_p := resize(g19_d * signed('0' & g19_c), 11);
            g20_y <= f_y6(signed(resize(g19_y0, 11)) + shift_right(v_p, 3));
            g20_lc <= g19_lc;
            -- S21: L3, Grey Fine: white lines over; Ink: |y - line|; Grey Bold: as it is
            v_l := resize(g20_lc * to_unsigned(63, 6), 10);
            if c_ink = '1' then
                g21_d <= signed(resize(v_l(8 downto 3), 7)); g21_ink <= '1';
            else
                g21_d <= to_signed(63, 7) - signed(resize(g20_y, 7)); g21_ink <= '0';
            end if;
            g21_y0 <= g20_y; g21_c <= g20_lc;
            -- S22
            if g21_ink = '1' then
                v_y := signed(resize(g21_y0, 11)) - resize(g21_d, 11);
                if v_y < 0 then v_y := -v_y; end if;
                g22_y <= unsigned(v_y(5 downto 0));
            else
                v_p := resize(g21_d * signed('0' & g21_c), 11);
                g22_y <= f_y6(signed(resize(g21_y0, 11)) + shift_right(v_p, 3));
            end if;
            -- S23: luma = 64 + 14 y - y / 8 (white 939)
            s_y23 <= to_unsigned(64, 10) + resize(g22_y & "0000", 10) - resize(g22_y & '0', 10)
                     - resize(g22_y(5 downto 3), 10);
        end if;
    end process p_blend;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD -> the camera grid.  luma (or the Osc lattice x 3) -> exact block sums over 64 h
    -- (32 virtual px; a straddling pixel is split by its h weights) -> a vertical one-pole per block column
    -- (1/16 per line at 1080p, 1/8 at 2-px units, 1/4 at 480i / 576i; loaded on the field's first line) ->
    -- x gain -> grid bank s_bank, rewritten on every line so the row keeps its last line's value.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_np : unsigned(6 downto 0);
        variable v_r : unsigned(2 downto 0);
        variable v_dq : signed(14 downto 0);
        variable v_n : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);
            end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= f_osc3(s_osum);
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid;

            if s_zone = '1' then
                s_fl <= '1'; s_seen <= '0';
            elsif w_v1 = '1' and w_v2 = '0' then
                if s_seen = '1' then s_fl <= '0'; end if;
                s_seen <= '1';
            end if;

            if s_sv = '1' then
                v_np := ('0' & w_ph) + resize(s_xs, 7);
                w_ph <= v_np(5 downto 0);
                if v_np(6) = '1' then
                    v_r := v_np(2 downto 0);
                    w_cy <= '1'; w_wa <= s_xs - v_r; w_wb <= v_r;
                else
                    w_cy <= '0'; w_wa <= s_xs; w_wb <= (others => '0');
                end if;
            else
                w_ph <= (others => '0'); w_cy <= '0';
            end if;
            w_l <= s_sl; w_v1 <= s_sv;
            w_pa <= f_mulw(w_l, w_wa); w_pb <= f_mulw(w_l, w_wb);
            w_cy2 <= w_cy and w_v1; w_v2 <= w_v1;
            w_dv <= '0';
            if w_v2 = '1' then
                if w_cy2 = '1' then
                    w_dump <= w_acc + resize(w_pa, 14); w_acc <= resize(w_pb, 14);
                    w_dv <= '1'; w_dcol <= w_col; w_col <= w_col + 1;
                else
                    w_acc <= w_acc + resize(w_pa, 14);
                end if;
            else
                w_acc <= (others => '0'); w_col <= (others => '0');
            end if;
            iq_f <= iq_q;
            w_diff <= signed(resize(w_dump, 15)) - signed(resize(iq_f(13 downto 0), 15));
            w_dd <= w_dump; w_qd <= iq_f(13 downto 0); w_v4 <= w_dv; w_c4 <= w_dcol;
            case s_ush is
                when "00"   => v_dq := shift_right(w_diff, 4);
                when "01"   => v_dq := shift_right(w_diff, 3);
                when others => v_dq := shift_right(w_diff, 2);
            end case;
            v_n := signed(resize(w_qd, 16)) + resize(v_dq, 16);
            if s_fl = '1' then w_qn <= w_dd; else w_qn <= unsigned(v_n(13 downto 0)); end if;
            iq_we <= w_v4; w_v5 <= w_v4; w_c5 <= w_c4;
            w_v6 <= w_v5; w_c6 <= w_c5; w_qr <= w_qn(13 downto 6);
            gr_wd <= w_qr;                                   -- the raw block luma (the engine scales by K6)
            gr_we <= w_v6;
            gr_wa <= s_bank & (r_rb + resize(w_c6, 11));
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: luma [1 2 1]/4, luma 13/16 toward black, blanking gate.  Chroma 512.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_yd1 <= s_y23; s_yd2 <= s_yd1;
            s_ya <= resize(s_y23, 12) + resize(s_yd2, 12) + 2;
            s_yb <= s_yd1;
            v_y := s_ya + shift_left(resize(s_yb, 12), 1);
            s_y26 <= v_y(11 downto 2);
            if s_y26 > 64 then v_dl := s_y26 - 64; else v_dl := (others => '0'); end if;
            s_y27 <= s_y26 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if sd_q(2) = '1' then
                s_y28 <= s_y27;
            else
                s_y28 <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= sd_o(0);
    data_out.vsync_n <= sd_o(1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= sd_o(2);

end architecture brilliant;
