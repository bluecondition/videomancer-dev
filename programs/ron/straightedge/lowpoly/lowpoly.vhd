-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Lowpoly v0.1.0 -- Straightedge.  Low-polygon rendering, a sheet of crumpled paper lit from the side, a cut
-- gemstone, Ron Resch's folded paper reliefs: the screen is a faceted sheet, an irregular mesh of triangles,
-- every facet ONE flat grey given by its slope toward a light.  Luma only: U = V = 512 on every pixel.
--
--   columns of constant width along a sheared axis (xs = x + lean * y); the vertex (column c, row j) keeps its
--   column and is displaced only in height:  y(c, j) = j * rh + A * tri(phase(c, j)),  phase = hash / 2 + a wave
--   travelling across the sheet + the Drift phase; its height z(c, j) follows the same wave with its own hash.
--   A row boundary is a polyline, straight inside a column; the quad (c, band) is split by one diagonal whose
--   direction alternates.  Every edge is the zero of an affine function: one accumulator per edge, reloaded at
--   each column from the vertex tables (E = base - step * x), one subtraction per pixel.
--
--   K1  X        the light, across: tone = 1/2 + lx dz/dx + ly dz/dy (+ the camera)
--   K2  Y        the light, down
--   K3  Scale    facet size: columns 32 .. 384 virtual px (exponential), rows 7/8 of that
--   K4  Lean     shear of the columns, bipolar, up to 45 degrees either way
--   K5  Drift    bipolar speed (Hypnos): the ripple travels right (+) or left (-), 2^-12 .. 2^-5 cycle per field
--   K6  Camera   the luma held per facet (a 32 x 32 virtual-px grid, one field old) adds to its flat tone,
--                relative to the picture's mean level
--   S7  Glide    Off / On      S8 Facet  Triangles / Quads      S9 Tone  Grey (six steps) / Ink
--   S10 Weight   Bold (filled facets) / Fine (the mesh edges, white lines over dim facets; Ink: XOR)
--   S11 Source   Video / Osc
--   P12 RELIEF   0 = a regular lattice of studs, 50 % = the crumple, 100 % = wild shards
--
-- The chassis is the Kinetic riley shell (via Vellum rosette):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID constants, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         switches; the knobs' holds, glides and the Drift speed (Hypnos) are
--                                        in the microprogram
--   [C] ENGINE                           the per-field microcoded engine (lowpoly.py PROGRAM: knob holds and
--                                        glides, Drift, every per-field constant, line 0's values), the
--                                        vertex and gain table fills, the line engine
--   [D] SOURCE FIELD                     luma (or the Osc lattice) -> box -> vertical IIR -> gain -> camera grid
--   [E] MESH                             column fetch (hash, vertex tables, grid), edge accumulators, distances,
--                                        tones, blends
--   [F] OUTPUT CONDITIONING              tone [1 2 1], 13/16 toward black (white 771), blanking gate
--
-- Stage numbering: stage 1 is the generated avid's own clock; a register "at stage S" holds pixel n at
-- T + S + n (T = the g_start clock).  Beam pixel n is screen column n.  The output register is at stage
-- C_LAT - 3 and the generated avid lands 4 clocks after the delayed source avid (the shell's parity-locked
-- start), so output column j shows pixel j + 4 on screen column j + 4.  s_gav_sr(S - 2) flags pixels
-- 0 .. W - 1 at stage S.  One latency (40) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.lowpoly_pkg.all;
use work.all;

architecture lowpoly of program_top is
    constant C_LAT : integer := 40;   -- = processing_delay_clks

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
    -- the class of an L1 gradient norm w (Q4): e = position of its leading one (w >= 4), q = the two bits under it
    function f_class(w : unsigned(7 downto 0)) return unsigned is
    begin
        if w(7) = '1' then return "101" & w(6 downto 5);
        elsif w(6) = '1' then return "100" & w(5 downto 4);
        elsif w(5) = '1' then return "011" & w(4 downto 3);
        elsif w(4) = '1' then return "010" & w(3 downto 2);
        elsif w(3) = '1' then return "001" & w(2 downto 1);
        elsif w(2) = '1' then return "000" & w(1 downto 0);
        else return "00000";
        end if;
    end function;
    -- t = min(u >> e, 2047)
    function f_bar(u : unsigned(14 downto 0); e : unsigned(2 downto 0)) return unsigned is
        variable v : unsigned(14 downto 0);
    begin
        case to_integer(e) is
            when 0      => v := u;
            when 1      => v := "0" & u(14 downto 1);
            when 2      => v := "00" & u(14 downto 2);
            when 3      => v := "000" & u(14 downto 3);
            when 4      => v := "0000" & u(14 downto 4);
            when others => v := "00000" & u(14 downto 5);
        end case;
        if v(14 downto 11) /= "0000" then return to_unsigned(2047, 11); end if;
        return v(10 downto 0);
    end function;
    -- the distance (Q3 px, 0 .. 63) = t * (7/8, 3/4, 5/8, 9/16) >> 4
    function f_dist(t : unsigned(10 downto 0); q : unsigned(1 downto 0)) return unsigned is
        variable v_a, v_b, v_r : unsigned(10 downto 0);
    begin
        case to_integer(q) is
            when 0      => v_a := t;                    v_b := "000" & t(10 downto 3);
            when 1      => v_a := t;                    v_b := "00" & t(10 downto 2);
            when 2      => v_a := "0" & t(10 downto 1); v_b := "000" & t(10 downto 3);
            when others => v_a := "0" & t(10 downto 1); v_b := "0000" & t(10 downto 4);
        end case;
        v_r := f_addsub(v_a, v_b, not q(1));
        if v_r(10) = '1' then return to_unsigned(63, 6); end if;
        return v_r(9 downto 4);
    end function;
    -- a facet's tone from its 3-bit step: six steps of 51 (Bold, Ink) or of 16 (the dim facets under Fine lines)
    function f_tone(c : unsigned(2 downto 0); dim : std_logic) return unsigned is
    begin
        if dim = '1' then
            case to_integer(c) is
                when 0 => return to_unsigned(0, 8);
                when 1 => return to_unsigned(16, 8);
                when 2 => return to_unsigned(32, 8);
                when 3 => return to_unsigned(48, 8);
                when 4 => return to_unsigned(64, 8);
                when others => return to_unsigned(80, 8);
            end case;
        end if;
        case to_integer(c) is
            when 0 => return to_unsigned(0, 8);
            when 1 => return to_unsigned(51, 8);
            when 2 => return to_unsigned(102, 8);
            when 3 => return to_unsigned(153, 8);
            when 4 => return to_unsigned(204, 8);
            when others => return to_unsigned(255, 8);
        end case;
    end function;
    -- max(0, 4 - d) for a Q3 distance: the neighbour's share of the pixel (0 .. 4 eighths)
    function f_cov(d : unsigned(5 downto 0)) return unsigned is
    begin
        if d >= 4 then return to_unsigned(0, 3); end if;
        return to_unsigned(4, 3) - d(2 downto 0);
    end function;

    attribute no_rw_check : boolean;

    type t_s24x3 is array (0 to 2) of signed(23 downto 0);
    type t_s17x3 is array (0 to 2) of signed(16 downto 0);
    type t_s16x3 is array (0 to 2) of signed(15 downto 0);
    type t_s15x3 is array (0 to 2) of signed(14 downto 0);
    type t_s10x3 is array (0 to 2) of signed(9 downto 0);
    type t_s10x2 is array (0 to 1) of signed(9 downto 0);
    type t_s9x4  is array (0 to 3) of signed(8 downto 0);
    type t_s11x4 is array (0 to 3) of signed(10 downto 0);
    type t_s12x4 is array (0 to 3) of signed(11 downto 0);
    type t_u3x4  is array (0 to 3) of unsigned(2 downto 0);
    type t_u5x4  is array (0 to 3) of unsigned(4 downto 0);

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_ysl  : unsigned(11 downto 0) := to_unsigned(256, 12);     -- the line step, latched at the field start
    signal c_sd   : std_logic := '0';                                  -- SD: the centre 1440 virtual px
    signal c_rs   : signed(14 downto 0) := to_signed(7168, 15);        -- row height / column width, Q13 (engine)
    signal c_mean : unsigned(7 downto 0) := (others => '0');           -- the camera grid's mean (last field)

    -- [A] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    type t_sd is array (0 to 255) of std_logic_vector(1 downto 0);
    signal sd_ram : t_sd := (others => "11");                           -- the sync delay line (C_LAT clocks)
    attribute no_rw_check of sd_ram : signal is true;
    signal sd_wp  : unsigned(7 downto 0) := (others => '0');
    signal sd_q   : std_logic_vector(1 downto 0) := "11";
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k): pixels 0 .. W - 1 at stage k + 2
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 15) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] switches (latched at the zone start)
    signal s_glide : std_logic := '1';
    signal s_quad, s_ink, s_fine, s_src : std_logic := '0';

    -- [C] THE PER-FIELD ENGINE (after Hardline flute, via Vellum cassini): 37-bit accumulator, a 256 x 32-bit
    -- register file in block RAM (power-up values: the knob holds and glides at the TOML defaults, and the
    -- constants), one adder, a serial 18 x 18 signed multiplier and a serial right shift; micro-op = opcode
    -- (15..12) & operand (11..0)
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
    signal rf_ram : t_c_lp_rfi := C_LP_RFI;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal u_run : std_logic := '0';
    signal u_ph  : unsigned(1 downto 0) := "11";
    signal u_pc, u_lr : unsigned(9 downto 0) := (others => '0');
    signal u_ir  : std_logic_vector(15 downto 0) := x"F001";      -- the microprogram ROM's output register
    signal u_irf : std_logic_vector(15 downto 0) := x"F001";      -- its fabric copy (from phase 1 on)
    signal u_mul, u_shr, u_halt, u_xo, u_xst : std_logic := '0';  -- decoded at phase 1
    signal u_gate, u_subr, u_shift, u_cz : std_logic := '0';      -- the adder's controls, as registers
    signal u_acc, u_b : signed(36 downto 0) := (others => '0');
    signal u_cnt : unsigned(4 downto 0) := (others => '0');
    signal u_out : std_logic;                                     -- the ROM address port's strobe
    signal u_outr : std_logic := '0';                             -- every other port's, one clock later (acc and op hold)
    signal w_in  : unsigned(15 downto 0);
    signal ft_adr : unsigned(7 downto 0) := (others => '0');
    signal ft_q  : std_logic_vector(15 downto 0) := (others => '0');

    -- [C] per-field constants (the engine's outputs)
    signal c_cwp  : unsigned(14 downto 0) := to_unsigned(4096, 15);    -- column width, px Q6
    signal c_rhl  : unsigned(14 downto 0) := to_unsigned(3584, 15);    -- row height, lines Q6
    signal c_hr2  : unsigned(14 downto 0) := to_unsigned(1792, 15);    -- the row phase's wrap point
    signal c_cwh  : unsigned(15 downto 0) := to_unsigned(2048, 16);    -- column width, half virtual px Q4
    signal c_c4   : unsigned(13 downto 0) := (others => '0');          -- 1/4 and 3/4 of it (the camera samples)
    signal c_c12  : unsigned(15 downto 0) := (others => '0');
    signal c_LR   : signed(14 downto 0) := (others => '0');            -- shear of one row, half virtual px Q4
    signal c_lpl  : signed(14 downto 0) := (others => '0');            -- lean, px per line, Q12
    signal c_lplq : signed(6 downto 0) := (others => '0');             -- the same, Q4
    signal c_wcc  : unsigned(4 downto 0) := "01000";                   -- the column border's gradient class
    signal c_hw4  : unsigned(5 downto 0) := to_unsigned(16, 6);        -- Fine: half line width, Q4 px
    signal c_rhv  : unsigned(12 downto 0) := (others => '0');          -- row height, virtual lines Q4
    signal c_base : signed(7 downto 0) := to_signed(38, 8);            -- 38 - the grid's mean / 2 (units of 4; 6 = the rounding)
    signal c_ph8  : unsigned(7 downto 0) := (others => '0');           -- the drift phase's index offset
    signal c_ox, c_oy : signed(7 downto 0) := (others => '0');         -- the regular studs: odd vertices' z offset

    -- [C] table fills: one accumulator, five passes launched by the engine
    signal f_i0, f_i1, f_acc : signed(25 downto 0) := (others => '0');
    signal f_m    : unsigned(16 downto 0) := (others => '0');
    signal ft_run : std_logic := '0';
    signal ft_k   : unsigned(7 downto 0) := (others => '0');
    signal ft_code : unsigned(4 downto 0) := (others => '0');
    signal ft_wey, ft_wes, ft_wex, ft_wez, ft_weg : std_logic := '0';
    signal ft_wa  : unsigned(7 downto 0) := (others => '0');
    signal ft_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal ft_wb  : unsigned(7 downto 0) := (others => '0');

    -- [C] vertex tables, gain table
    type t_r16x256 is array (0 to 255) of std_logic_vector(15 downto 0);
    type t_r8x256  is array (0 to 255) of unsigned(7 downto 0);
    signal ty_ram, ts_ram : t_r16x256 := (others => (others => '0'));
    signal tzx_ram, tzy_ram, gn_ram : t_r8x256 := (others => (others => '0'));
    attribute no_rw_check of ty_ram : signal is true;
    attribute no_rw_check of ts_ram : signal is true;
    attribute no_rw_check of tzx_ram : signal is true;
    attribute no_rw_check of tzy_ram : signal is true;
    attribute no_rw_check of gn_ram : signal is true;
    signal ty_q, ts_q, ty_f, ts_f : std_logic_vector(15 downto 0) := (others => '0');
    signal tzx_q, tzy_q, tzx_f, tzy_f : unsigned(7 downto 0) := (others => '0');
    signal gn_q, gn_f : unsigned(7 downto 0) := (others => '0');

    -- [C] line engine: this line's row phase, row, first column and its phase, sample points
    signal e_lc   : unsigned(5 downto 0) := (others => '1');
    signal l_vr   : signed(16 downto 0) := (others => '0');            -- line - row r, lines Q6
    signal l_r    : unsigned(7 downto 0) := (others => '0');           -- row
    signal l_X0   : signed(22 downto 0) := (others => '0');            -- pixel 0 into its column, px Q12
    signal l_ci   : unsigned(7 downto 0) := (others => '0');           -- first column
    signal l_XC   : signed(18 downto 0) := (others => '0');            -- lattice x of the first column, Q4
    signal l_BX1, l_BX0 : signed(18 downto 0) := (others => '0');      -- sample x offsets of quads r, r - 1
    signal l_YA   : signed(16 downto 0) := (others => '0');            -- virtual y of row r's middle, Q4
    signal l_rinc, l_cinc, l_cdec : std_logic := '0';
    signal l_gr0, l_gr1 : unsigned(4 downto 0) := (others => '0');     -- grid rows of the quads r - 1 and r
    signal pp_go  : std_logic := '0';

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_p, s_pfirst : unsigned(7 downto 0) := (others => '0');
    signal s_avd : std_logic_vector(1 to 5) := (others => '0');    -- data_in.avid delayed
    signal s_col, s_pcol, s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal c_b32 : std_logic := '1';                               -- box length 32 / 16 pixels
    signal c_brp : unsigned(7 downto 0) := to_unsigned(225, 8);
    signal c_iir : unsigned(2 downto 0) := to_unsigned(5, 3);
    signal bx_ram : t_r8x256 := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(5 downto 0) := (others => '0');
    signal s_bsum : unsigned(12 downto 0) := (others => '0');
    signal s_ev, s_fl : std_logic := '0';
    signal s_eb, s_eq : unsigned(11 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(11 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_wd : unsigned(11 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    -- the camera grid: 32 rows x 64 columns of 32 x 32 virtual px, two banks
    type t_gr is array (0 to 2047) of unsigned(5 downto 0);
    signal gr0_ram, gr1_ram : t_gr := (others => (others => '0'));
    attribute no_rw_check of gr0_ram : signal is true;
    attribute no_rw_check of gr1_ram : signal is true;
    signal gr0_q, gr1_q : unsigned(5 downto 0) := (others => '0');
    signal w_bank : std_logic := '0';                              -- the bank being written this field
    signal w_vy   : unsigned(18 downto 0) := (others => '0');      -- this line's virtual y, Q8
    signal w_bp   : signed(6 downto 0) := (others => '1');         -- the previous line's grid row
    signal w_b    : signed(6 downto 0) := (others => '1');
    signal w_samp : std_logic := '0';                              -- this line is a grid row's sample line
    signal w_x    : unsigned(11 downto 0) := (others => '0');      -- virtual x of the store's next write, h
    signal w_lb   : unsigned(5 downto 0) := (others => '1');       -- grid column of the last write
    signal w1_we, w2_we, w1_new, w2_new : std_logic := '0';
    signal w1_a, w2_a : unsigned(10 downto 0) := (others => '0');
    signal w_sum  : unsigned(17 downto 0) := (others => '0');

    -- [E] the beam's column counters (A at stage 2 drives the fetch, B at stage 16 the pixel path)
    signal x_X, b_X : unsigned(14 downto 0) := (others => '0');
    signal x_cross, b_cross : std_logic := '0';
    signal x_ah   : unsigned(1 downto 0) := (others => '0');
    signal p_c    : unsigned(7 downto 0) := (others => '0');
    signal p_XC   : signed(18 downto 0) := (others => '0');
    -- column fetch: six vertex reads and four grid reads
    signal f_n    : unsigned(2 downto 0) := (others => '0');
    signal f_ev, f_ls : std_logic := '0';
    signal f1_x, f2_x, f3_x : unsigned(15 downto 0) := (others => '0');
    signal f1_w, f2_w, f3_w : unsigned(7 downto 0) := (others => '0');
    signal f1_par, f2_par, f3_par, f4_par, f5_par, f6_par : std_logic := '0';
    signal f1_v, f2_v, f3_v, f4_v, f5_v, f6_v : std_logic := '0';
    signal f1_k, f2_k, f3_k, f4_k, f5_k, f6_k : unsigned(2 downto 0) := (others => '0');
    signal f4_ad, f4_az : unsigned(7 downto 0) := (others => '0');
    signal g1_t   : signed(18 downto 0) := (others => '0');
    signal g1_v, g2_v, g3_v, g4_v : std_logic := '0';
    signal g1_j, g2_j, g3_j, g4_j : unsigned(1 downto 0) := (others => '0');
    signal g2_a   : unsigned(10 downto 0) := (others => '0');
    signal g4_d   : unsigned(5 downto 0) := (others => '0');
    -- the column's derived values
    signal d_B    : t_s17x3 := (others => (others => '0'));        -- edge bases, lines Q6 (polyline, diag r, diag r - 1)
    signal d_S    : t_s15x3 := (others => (others => '0'));        -- edge steps, lines per px Q13
    signal d_TX   : t_s10x3 := (others => (others => '0'));        -- lx dz along the three row edges
    signal d_L, d_R : t_s10x2 := (others => (others => '0'));      -- ly dz along the left / right borders
    signal d_G    : t_s9x4 := (others => (others => '0'));         -- 32 - mean / 2 + camera, per facet (A0 B0 A1 B1)
    signal t_sr   : std_logic_vector(0 to 2) := (others => '0');
    signal t_ls   : std_logic_vector(0 to 2) := (others => '0');
    signal t_S    : t_s12x4 := (others => (others => '0'));        -- the facets' tone sums (A0 B0 A1 B1), units of 4
    signal t_code, a_code : t_u3x4 := (others => (others => '0'));
    -- edge accumulators (stage 17)
    signal e_P    : t_s16x3 := (others => (others => '0'));
    signal e_E, e_st : t_s24x3 := (others => (others => '0'));
    signal e_xs   : unsigned(14 downto 0) := (others => '0');
    signal e_sn   : unsigned(3 downto 0) := (others => '0');
    signal e_fin  : std_logic := '0';
    -- column-border distance (stages 17 .. 22) and flags
    signal c17_xn : unsigned(9 downto 0) := (others => '0');
    signal c18_u : unsigned(14 downto 0) := (others => '0');
    signal c19_t : unsigned(10 downto 0) := (others => '0');
    signal c20_d, c21_d, c22_d : unsigned(5 downto 0) := (others => '0');
    signal cr_sr  : std_logic_vector(17 to 30) := (others => '0');   -- the pixel is the first of its column
    signal lf_sr  : std_logic_vector(17 to 30) := (others => '0');   -- the left border is the nearer one
    -- pixel path
    type t_s16x2 is array (0 to 1) of signed(15 downto 0);
    signal s18_E  : t_s16x2 := (others => (others => '0'));
    signal s18_pp, s18_pd : signed(8 downto 0) := (others => '0');
    signal s18_ap, s18_ad : unsigned(3 downto 0) := (others => '0');
    signal s19_up, s19_ud, s20_up, s20_ud : unsigned(14 downto 0) := (others => '0');
    signal s21_tp, s21_td : unsigned(10 downto 0) := (others => '0');
    signal s19_wp, s19_wd : unsigned(7 downto 0) := (others => '0');
    signal s20_cp, s20_cd : unsigned(4 downto 0) := (others => '0');
    signal s21_qp, s21_qd : unsigned(1 downto 0) := (others => '0');
    signal s22_dp, s22_dd : unsigned(5 downto 0) := (others => '0');
    signal b1_sr  : std_logic_vector(18 to 22) := (others => '0');   -- the pixel is in band r (else r - 1)
    signal sb_sr  : std_logic_vector(19 to 22) := (others => '0');   -- below its quad's diagonal
    signal s23_own, s23_od, s23_op : unsigned(7 downto 0) := (others => '0');
    signal s23_cd, s23_cp : unsigned(2 downto 0) := (others => '0');
    type t_u5d is array (23 to 30) of unsigned(3 downto 0);
    signal c3_d   : t_u5d := (others => (others => '0'));
    signal s24_df, s27_df, s31_df : signed(8 downto 0) := (others => '0');
    signal s24_own, s24_op, s25_own, s25_op, s26_t, s26_op, s27_t, s28_t, s29_t, s30_t, s31_t, s32_t, s33_t : unsigned(7 downto 0) := (others => '0');
    signal s24_cd, s24_cp, s25_cp, s26_cp, s27_cp : unsigned(2 downto 0) := (others => '0');
    signal s25_m, s28_m : signed(12 downto 0) := (others => '0');
    signal s31_c  : unsigned(3 downto 0) := (others => '0');
    signal s32_m  : signed(13 downto 0) := (others => '0');

    -- [F] output conditioning
    signal s34_t, s35_t : unsigned(7 downto 0) := (others => '0');
    signal s35_s  : unsigned(9 downto 0) := (others => '0');
    signal s36_y, s37_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            ft_q <= C_LP_EXP(to_integer(ft_adr));
        end if;
    end process p_rom;

    -- sync delay: written every clock, read C_LAT - 1 addresses behind, + the output register = C_LAT clocks
    p_sdram : process(clk)
    begin
        if rising_edge(clk) then
            sd_q <= sd_ram(to_integer(sd_wp - (C_LAT - 1)));
            sd_ram(to_integer(sd_wp)) <= data_in.vsync_n & data_in.hsync_n;
            sd_wp <= sd_wp + 1;
        end if;
    end process p_sdram;

    -- box delay: read (box length) samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_avd(2) = '1' then bx_ram(to_integer(bx_wp)) <= s_p; end if;
        end if;
    end process p_bxram;

    -- camera-field line store (half horizontal resolution): read 3 pixels ahead of the write
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_pcol(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    -- vertex tables: written in the vblank zone only, read by the column fetch
    p_tyram : process(clk) begin
        if rising_edge(clk) then
            ty_q <= ty_ram(to_integer(f4_ad));
            if ft_wey = '1' then ty_ram(to_integer(ft_wa)) <= ft_wd; end if;
        end if;
    end process p_tyram;
    p_tsram : process(clk) begin
        if rising_edge(clk) then
            ts_q <= ts_ram(to_integer(f4_ad));
            if ft_wes = '1' then ts_ram(to_integer(ft_wa)) <= ft_wd; end if;
        end if;
    end process p_tsram;
    p_tzxram : process(clk) begin
        if rising_edge(clk) then
            tzx_q <= tzx_ram(to_integer(f4_az));
            if ft_wex = '1' then tzx_ram(to_integer(ft_wa)) <= ft_wb; end if;
        end if;
    end process p_tzxram;
    p_tzyram : process(clk) begin
        if rising_edge(clk) then
            tzy_q <= tzy_ram(to_integer(f4_az));
            if ft_wez = '1' then tzy_ram(to_integer(ft_wa)) <= ft_wb; end if;
        end if;
    end process p_tzyram;

    -- camera gain table: written in the zone, read by the grid write (the store's write data)
    p_gnram : process(clk) begin
        if rising_edge(clk) then
            gn_q <= gn_ram(to_integer(lb_wd(11 downto 4)));
            if ft_weg = '1' then gn_ram(to_integer(ft_wa)) <= ft_wb; end if;
        end if;
    end process p_gnram;

    -- camera grid, two banks of 6-bit levels: the bank written this field is read during the next one
    p_gr0 : process(clk) begin
        if rising_edge(clk) then
            gr0_q <= gr0_ram(to_integer(g2_a));
            if w2_we = '1' and w_bank = '0' then gr0_ram(to_integer(w2_a)) <= gn_f(7 downto 2); end if;
        end if;
    end process p_gr0;
    p_gr1 : process(clk) begin
        if rising_edge(clk) then
            gr1_q <= gr1_ram(to_integer(g2_a));
            if w2_we = '1' and w_bank = '1' then gr1_ram(to_integer(w2_a)) <= gn_f(7 downto 2); end if;
        end if;
    end process p_gr1;

    -- the engine's inputs: 8 x the knobs (0 .. 7), the pixel and line steps, the table ROM, the grid's mean
    with u_irf(3 downto 0) select w_in <=
        resize(s_xs, 16)            when x"8",
        resize(s_ysl, 16)           when x"9",
        unsigned(ft_q)              when x"B",
        resize(c_mean, 16)          when x"F",
        resize(unsigned(registers_in(to_integer(unsigned(u_irf(2 downto 0))))) & "000", 16) when others;

    u_out <= '1' when u_run = '1' and u_ph = "10" and u_xo = '1' else '0';

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] switches
    ---------------------------------------------------------------------------
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

            -- [A] width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_quad  <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is                  -- Osc step, box length, its read offset
                    when 4      => s_oxs <= to_unsigned(204, 9); c_b32 <= '0'; c_brp <= to_unsigned(241, 8); c_sd <= '1';
                    when 3      => s_oxs <= to_unsigned(153, 9); c_b32 <= '1'; c_brp <= to_unsigned(225, 8); c_sd <= '0';
                    when others => s_oxs <= to_unsigned(102, 9); c_b32 <= '1'; c_brp <= to_unsigned(225, 8); c_sd <= '0';
                end case;
                s_ysl <= s_ys;
                case to_integer(s_ys) is                   -- the Osc lattice's line step: 121 virtual lines' worth
                    when 1138   => s_oys <= to_unsigned(537, 10);
                    when 960    => s_oys <= to_unsigned(453, 10);
                    when 576    => s_oys <= to_unsigned(272, 10);
                    when 480    => s_oys <= to_unsigned(226, 10);
                    when 512    => s_oys <= to_unsigned(242, 10);
                    when 384    => s_oys <= to_unsigned(181, 10);
                    when others => s_oys <= to_unsigned(121, 10);
                end case;
                -- the vertical IIR's shift: about 32 .. 48 virtual lines in every mode
                if s_ys >= 960 then c_iir <= to_unsigned(3, 3);
                elsif s_ys >= 480 then c_iir <= to_unsigned(4, 3);
                else c_iir <= to_unsigned(5, 3); end if;
            end if;
            g_wm1 <= s_w - 1;
            -- Fine's half line width: 1/256 of the column, 1 .. 3 px (Q4)
            if c_cwp(14 downto 8) < 16 then c_hw4 <= to_unsigned(16, 6);
            elsif c_cwp(14 downto 8) > 48 then c_hw4 <= to_unsigned(48, 6);
            else c_hw4 <= c_cwp(13 downto 8); end if;

            -- [A] sync delay, generated-avid and start-pulse shift registers (field_n is tied high by the
            -- core on every input, so it only gets a register)
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 14);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [C] THE PER-FIELD ENGINE.  Started at every zone start (once per field);
    -- a micro-op takes 3 clocks (register-file read, operand, execute), a
    -- multiply 20, a shift by n: n + 2.  lowpoly.py holds the program and its
    -- emulator; the integer model runs the same words.
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_op  : std_logic_vector(3 downto 0);
        variable v_arg : std_logic_vector(11 downto 0);
        variable v_gate, v_sub, v_shift, v_last, v_cond, v_fetch : std_logic;
        variable v_bx  : signed(36 downto 0);
        variable v_t   : signed(37 downto 0);
        variable v_sum : signed(36 downto 0);
    begin
        if rising_edge(clk) then
            v_op := u_irf(15 downto 12);
            v_arg := u_irf(11 downto 0);
            v_fetch := '0';
            if u_ph = "00" then
                rf_q <= rf_ram(to_integer(unsigned(u_ir(7 downto 0))));
            end if;
            if u_run = '1' and u_ph = "10" and u_xst = '1' then
                rf_ram(to_integer(unsigned(u_irf(7 downto 0)))) <= std_logic_vector(u_acc(31 downto 0));
            end if;

            if s_zgo = '1' then
                u_run <= '1'; u_pc <= (others => '0'); u_ph <= "11";
            elsif u_run = '1' then
                case u_ph is
                    when "11" =>                          -- fetch (first micro-op)
                        v_fetch := '1';
                        u_ph <= "00";
                    when "00" =>                          -- register-file read
                        u_irf <= u_ir;
                        u_ph <= "01";
                    when "01" =>                          -- operand, accumulator clear, next address
                        case v_op is
                            when OP_LD | OP_ADD | OP_SUB => u_b <= resize(signed(rf_q), 37);
                            when OP_LDI | OP_ADDI | OP_SUBI => u_b <= resize(signed(v_arg), 37);
                            when OP_IN   => u_b <= signed(resize(w_in, 37));
                            when OP_MUL  => u_b <= shift_left(resize(signed(rf_q(17 downto 0)), 37), 18);
                            when OP_MULI => u_b <= shift_left(resize(signed(v_arg), 37), 18);
                            when others  => null;
                        end case;
                        if v_op = OP_LD or v_op = OP_LDI or v_op = OP_IN then
                            u_acc <= (others => '0');
                        elsif v_op = OP_MUL or v_op = OP_MULI then
                            u_acc(36 downto 18) <= (others => '0');
                        end if;
                        if v_op = OP_MUL or v_op = OP_MULI then u_cnt <= to_unsigned(17, 5);
                        elsif v_op = OP_SHR then u_cnt <= unsigned(v_arg(4 downto 0)) - 1; end if;
                        case v_arg(2 downto 0) is
                            when "000"  => v_cond := u_acc(36);
                            when "001"  => v_cond := s_glide;
                            when "010"  => v_cond := ft_run;
                            when others => v_cond := s_src;
                        end case;
                        v_cond := v_cond xor v_arg(3);
                        if v_op = OP_SKIP and v_cond = '1' then u_pc <= u_pc + 2;
                        elsif v_op = OP_JMP then u_pc <= unsigned(v_arg(9 downto 0));
                        elsif v_op = OP_CALL then u_pc <= unsigned(v_arg(9 downto 0)); u_lr <= u_pc + 1;
                        elsif v_op = OP_SPC and v_arg(0) = '0' then u_pc <= u_lr;
                        else u_pc <= u_pc + 1; end if;
                        u_mul <= '0'; u_shr <= '0'; u_halt <= '0'; u_xo <= '0'; u_xst <= '0';
                        u_gate <= '0'; u_subr <= '0'; u_shift <= '0'; u_cz <= '0';
                        case v_op is
                            when OP_LD | OP_ADD | OP_LDI | OP_ADDI | OP_IN => u_gate <= '1';
                            when OP_SUB | OP_SUBI => u_gate <= '1'; u_subr <= '1';
                            when OP_MUL | OP_MULI => u_mul <= '1'; u_shift <= '1'; u_gate <= u_acc(0);
                            when OP_SHR  =>
                                u_shr <= '1'; u_shift <= '1';
                                if v_arg(4 downto 0) = "00001" then u_cz <= '1'; end if;
                            when OP_ST   => u_xst <= '1';
                            when OP_OUT  => u_xo <= '1';
                            when OP_SPC  => u_halt <= v_arg(0);
                            when others  => null;
                        end case;
                        u_ph <= "10";
                    when others =>                        -- execute (the adder sees registers only)
                        v_gate := u_gate; v_sub := u_subr; v_shift := u_shift; v_last := '1';
                        if v_gate = '0' then v_bx := (others => '0');
                        elsif v_sub = '1' then v_bx := not u_b;
                        else v_bx := u_b; end if;
                        v_t := (u_acc & '1') + (v_bx & (v_sub and v_gate));
                        v_sum := v_t(37 downto 1);
                        if u_mul = '1' or u_shr = '1' then
                            if u_cz = '0' then v_last := '0'; end if;
                            u_cnt <= u_cnt - 1;
                            if u_cnt = 1 then u_cz <= '1'; u_subr <= u_mul; end if;   -- the next step is the last
                            if u_mul = '1' then u_gate <= v_sum(1); end if;           -- the next multiplier bit
                        end if;
                        if u_halt = '1' then u_run <= '0'; end if;
                        if v_shift = '1' then u_acc <= v_sum(36) & v_sum(36 downto 1);
                        else u_acc <= v_sum; end if;
                        if v_last = '1' then
                            v_fetch := '1';
                            u_ph <= "00";
                        end if;
                end case;
            end if;
            if v_fetch = '1' then
                u_ir <= C_LP_UC(to_integer(u_pc));
            end if;

            -- the engine's output registers (the zone has no active lines)
            u_outr <= u_out;
            if u_out = '1' and u_irf(4 downto 0) = "00000" then ft_adr <= unsigned(u_acc(7 downto 0)); end if;
            if u_outr = '1' then
                case to_integer(unsigned(u_irf(4 downto 0))) is
                    when 1  => c_cwp  <= unsigned(u_acc(14 downto 0));
                    when 2  => c_rhl  <= unsigned(u_acc(14 downto 0));
                    when 3  => c_hr2  <= unsigned(u_acc(14 downto 0));
                    when 4  => c_cwh  <= unsigned(u_acc(15 downto 0));
                    when 5  => c_c4   <= unsigned(u_acc(13 downto 0));
                    when 6  => c_c12  <= unsigned(u_acc(15 downto 0));
                    when 7  => c_LR   <= u_acc(14 downto 0);
                    when 8  => c_lpl  <= u_acc(14 downto 0);
                    when 9  => c_lplq <= u_acc(6 downto 0);
                    when 10 => c_wcc  <= f_class(unsigned(u_acc(7 downto 0)));
                    when 11 => c_rs   <= u_acc(14 downto 0);
                    when 12 => c_rhv  <= unsigned(u_acc(12 downto 0));
                    when 13 => c_base <= u_acc(7 downto 0);
                    when 24 => c_ph8  <= unsigned(u_acc(7 downto 0));
                    when 25 => f_i0   <= u_acc(25 downto 0);
                    when 26 => f_i1   <= u_acc(25 downto 0);
                    when 27 => f_m    <= unsigned(u_acc(16 downto 0));
                    when 30 => c_ox   <= u_acc(7 downto 0);
                    when 31 => c_oy   <= u_acc(7 downto 0);
                    when others => null;          -- 14 .. 23 the line engine's values, 28 the fill, 29 the prep
                end case;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- [C] TABLE FILLS: one accumulator, one pass of 256 entries per launch
    -- (engine port 28; code = destination + 16 * the first half falls + 32 *
    -- the second half falls).  v(0) = FI0, v(128) = FI1 (the gain ramp: no
    -- reload), else v +- FM.
    --   Y  (0): TY(i) = v >> 6    vertex displacement A tri(i + f), lines Q6 (tri: -1 at 0, +1 at 128)
    --   S  (1): TS(i) = v >> 9    its slope share TY / column width, lines per px Q13
    --   ZX (2), ZY (3): a byte    128 + (lx, ly) / 4 * Zr tri(i + f): the light's dot product parts
    --   G  (4): min(255, v >> 16) the camera gain (half tone units by level)
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_dn : std_logic;
        variable v_n  : signed(25 downto 0);
    begin
        if rising_edge(clk) then
            ft_wey <= '0'; ft_wes <= '0'; ft_wex <= '0'; ft_wez <= '0'; ft_weg <= '0';
            if u_outr = '1' and u_irf(4 downto 0) = "11100" then
                ft_run <= '1'; ft_k <= (others => '0');
                ft_code <= unsigned(u_acc(5 downto 4)) & unsigned(u_acc(2 downto 0));
            elsif ft_run = '1' then
                if ft_k(7) = '1' then v_dn := ft_code(4); else v_dn := ft_code(3); end if;
                if ft_k = 0 then v_n := f_i0;
                elsif ft_k = 128 and ft_code(2) = '0' then v_n := f_i1;
                else v_n := signed(f_addsub(unsigned(f_acc), resize(f_m, 26), v_dn)); end if;
                f_acc <= v_n;
                ft_wa <= ft_k;
                if ft_code(0) = '1' then ft_wd <= std_logic_vector(v_n(24 downto 9));
                else ft_wd <= std_logic_vector(v_n(21 downto 6)); end if;
                if ft_code(2) = '0' then ft_wb <= unsigned(v_n(21 downto 14));
                elsif v_n(25 downto 24) /= "00" then ft_wb <= (others => '1');
                else ft_wb <= unsigned(v_n(23 downto 16)); end if;
                case to_integer(ft_code(2 downto 0)) is
                    when 0      => ft_wey <= '1';
                    when 1      => ft_wes <= '1';
                    when 2      => ft_wex <= '1';
                    when 3      => ft_wez <= '1';
                    when others => ft_weg <= '1';
                end case;
                ft_k <= ft_k + 1;
                if ft_k = 255 then ft_run <= '0'; end if;
            end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- [C] THE LINE ENGINE.  The engine's ports load line 0's values in the
    -- zone; they then advance once per ACTIVE line, 20 clocks after the
    -- generated avid falls (the line's last column fetch has finished): the
    -- row phase steps one line (a new row when it passes the wrap point), the
    -- first column's phase steps by the lean (a column in or out when it leaves
    -- 0 .. column width); a new row moves the camera sample points with it.
    -- Step 24 (and the engine's last port) fetches the line's first column.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_p : integer range 0 to 31;
        variable v_vo : unsigned(16 downto 0);
        variable v_vn, v_yn : signed(16 downto 0);
        variable v_xo : unsigned(22 downto 0);
        variable v_xn : signed(22 downto 0);
        variable v_xs : std_logic;
        variable v_g  : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            pp_go <= '0';
            v_p := to_integer(unsigned(u_irf(4 downto 0)));
            if u_outr = '1' and v_p = 29 then
                e_lc <= to_unsigned(24, 6);
            elsif s_gav_sr(0) = '1' and g_avid = '0' then
                e_lc <= (others => '0');
            elsif e_lc /= 63 then
                e_lc <= e_lc + 1;
            end if;

            if u_outr = '1' then
                case v_p is
                    when 14 => l_vr  <= u_acc(16 downto 0);
                    when 15 => l_r   <= unsigned(u_acc(7 downto 0));
                    when 16 => l_X0  <= signed(resize(unsigned(u_acc(14 downto 0)) & "000000", 23));
                    when 17 => l_ci  <= unsigned(u_acc(7 downto 0));
                    when 18 => l_XC  <= u_acc(18 downto 0);
                    when 19 => l_BX1 <= u_acc(18 downto 0);
                    when 20 => l_BX0 <= u_acc(18 downto 0);
                    when 21 => l_YA  <= u_acc(16 downto 0);
                    when 23 => l_gr0 <= l_gr1; l_gr1 <= unsigned(u_acc(4 downto 0));
                    when others => null;
                end case;
            end if;

            -- one adder each for the row phase (+ a line, then - a row past the wrap point) and the column
            -- phase (+ the lean, then +- a column outside 0 .. column width)
            if e_lc = 20 then v_vo := to_unsigned(64, 17); else v_vo := resize(c_rhl, 17); end if;
            v_vn := signed(f_addsub(unsigned(l_vr), v_vo, e_lc(0)));
            if e_lc = 20 then v_xo := unsigned(resize(c_lpl, 23)); v_xs := '0';
            else v_xo := resize(c_cwp & "000000", 23); v_xs := not l_X0(22); end if;
            v_xn := signed(f_addsub(unsigned(l_X0), v_xo, v_xs));
            v_yn := l_YA + signed(resize(c_rhv, 17));
            if v_yn(16) = '1' then v_g := (others => '0');
            elsif v_yn(15 downto 14) /= "00" then v_g := (others => '1');
            else v_g := unsigned(v_yn(13 downto 9)); end if;
            case to_integer(e_lc) is
                when 20 =>
                    l_vr <= v_vn;
                    l_X0 <= v_xn;
                when 21 =>
                    l_rinc <= '0'; l_cinc <= '0'; l_cdec <= '0';
                    if l_vr >= signed(resize(c_hr2, 17)) then l_vr <= v_vn; l_rinc <= '1'; end if;
                    if l_X0(22) = '1' then l_X0 <= v_xn; l_cdec <= '1';
                    elsif v_xn(22) = '0' then l_X0 <= v_xn; l_cinc <= '1'; end if;
                when 22 =>
                    if l_rinc = '1' then
                        l_r <= l_r + 1;
                        l_BX0 <= l_BX1; l_BX1 <= l_BX1 - resize(c_LR, 19);
                        l_YA <= v_yn; l_gr0 <= l_gr1; l_gr1 <= v_g;
                    end if;
                    if l_cinc = '1' or l_cdec = '1' then
                        l_XC <= signed(f_addsub(unsigned(l_XC), resize(c_cwh, 19), l_cdec));
                    end if;
                    if l_cinc = '1' then l_ci <= l_ci + 1; elsif l_cdec = '1' then l_ci <= l_ci - 1; end if;
                when 24 =>
                    pp_go <= '1';
                when others => null;
            end case;
        end if;
    end process p_line;

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

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> box over 32 pixels (16 at
    -- SD: 32 virtual px), edge-replicated and reloaded at every line start (the
    -- box's resync) -> vertical one-pole IIR in the line store (half horizontal
    -- resolution; the first line of a field loads it) -> gain table -> the
    -- camera grid: on the first line 48 virtual lines or more under a grid
    -- row's top, every store write lands in that row's cell of its column (the
    -- last write of a block stays), so a cell holds the box mean of its own
    -- block.  The first write of every block of the centre 32 columns also
    -- adds to the field's sum (1024 cells): its mean is the camera's zero.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l   : unsigned(7 downto 0);
        variable v_s   : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_d   : signed(12 downto 0);
        variable v_dd  : signed(12 downto 0);
        variable v_n   : signed(12 downto 0);
        variable v_full : std_logic;
        variable v_b   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting; both phases restart in the
            -- zone (Hardline dazzle's fix of riley's line-0 phase)
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

            -- 1: the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_avd <= data_in.avid & s_avd(1 to 4);
            if s_avd(1) = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- 2: a plain register (keeps the shell's timing)
            s_p <= s_sl;
            s_pcol <= s_col;

            -- 3: box: sum of the last N samples; the samples before the line start count as the first one
            if s_avd(2) = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= c_brp; end if;
            if (c_b32 = '1' and s_bcnt = 32) or (c_b32 = '0' and s_bcnt = 16) then v_full := '1'; else v_full := '0'; end if;
            if v_full = '0' then v_tap := s_pfirst; else v_tap := bx_q; end if;
            if s_avd(2) = '1' and s_avd(3) = '0' then
                s_pfirst <= s_p;
                if c_b32 = '1' then s_bsum <= shift_left(resize(s_p, 13), 5);
                else s_bsum <= shift_left(resize(s_p, 13), 4); end if;
                s_bcnt <= to_unsigned(1, 6);
            elsif s_avd(2) = '1' then
                s_bsum <= s_bsum + s_p - v_tap;
                if v_full = '0' then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bcol <= s_pcol;

            -- 4: fabric copies (the box mean x 16); 5: vertical IIR on the odd columns, q += (box - q) >> k
            -- (the first line: q = box)
            if c_b32 = '1' then s_eb <= s_bsum(12 downto 1); else s_eb <= s_bsum(11 downto 0); end if;
            s_eq <= lb_q1;
            s_ev <= s_avd(3) and s_bcol(0); s_ecol <= s_bcol;
            if s_zone = '1' then s_fl <= '1';
            elsif s_avd(5) = '1' and s_avd(4) = '0' then s_fl <= '0'; end if;
            v_d := signed(resize(s_eb, 13)) - signed(resize(s_eq, 13));
            case to_integer(c_iir) is
                when 3      => v_dd := shift_right(v_d, 3);
                when 4      => v_dd := shift_right(v_d, 4);
                when others => v_dd := shift_right(v_d, 5);
            end case;
            v_n := signed(resize(s_eq, 13)) + v_dd;
            if s_fl = '1' then lb_wd <= s_eb; else lb_wd <= unsigned(v_n(11 downto 0)); end if;
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            -- the grid's rows: this line's virtual y (Q8) steps after every active line's last store write; its
            -- grid row is (y - 48) >> 5, and the line is a sample line when that is new and in 0 .. 31
            if s_zone = '1' then
                w_vy <= (others => '0'); w_bp <= (others => '1');
            elsif s_avd(5) = '1' and s_avd(4) = '0' then
                w_vy <= w_vy + resize(s_ys, 19);
                w_bp <= w_b;
            end if;
            v_b := signed(resize(w_vy(18 downto 8), 12)) - 48;
            w_b <= v_b(11 downto 5);
            if w_b /= w_bp and w_b(6 downto 5) = "00" then w_samp <= '1'; else w_samp <= '0'; end if;
            -- the grid's columns: virtual x (h) of the store's next write, from the first odd column
            if s_avd(5) = '0' then
                if c_sd = '1' then w_x <= to_unsigned(480, 12) + resize(s_xs, 12); else w_x <= resize(s_xs, 12); end if;
                w_lb <= (others => '1');
            elsif lb_we = '1' then
                w_x <= w_x + resize(s_xs & '0', 12);
                w_lb <= w_x(11 downto 6);
            end if;
            -- W0: the store's write (gain table address = its data); W1: the table's register; W2: fabric,
            -- the grid write and the mean's sum
            w1_we <= lb_we and w_samp;
            w1_a  <= unsigned(w_b(4 downto 0)) & w_x(11 downto 6);
            if w_x(11 downto 6) /= w_lb and w_x(11 downto 6) >= 14 and w_x(11 downto 6) <= 45 then w1_new <= '1'; else w1_new <= '0'; end if;
            w2_we <= w1_we; w2_a <= w1_a; w2_new <= w1_new and w1_we;
            gn_f <= gn_q;
            if s_zgo = '1' then
                c_mean <= w_sum(17 downto 10);
                w_sum <= (others => '0');
                w_bank <= not w_bank;
            elsif w2_new = '1' then
                w_sum <= w_sum + resize(gn_f, 18);
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] MESH, the column machine.  det = the clock in which x_cross shows the
    -- first pixel of a new column at stage 2 (or pp_go for a line's first
    -- column).  Six vertex reads (rows r - 1, r, r + 1 of the column's left and
    -- right borders) go through the hash (three shift-and-add rounds) -> table
    -- address -> block RAM -> fabric -> the derived registers (complete at
    -- det + 13); four grid reads fetch the facets' camera levels.  The products
    -- step * (sub-pixel start) follow at det + 14, the edge accumulators load at
    -- det + 15 = stage 17, where the second column counter (stage 16) shows the
    -- same crossing.  A line's first column is fetched in the blanking and its
    -- accumulators are walked to pixel 0 bit-serially (step * the pixel's place
    -- in the column).
    ---------------------------------------------------------------------------
    p_col : process(clk)
        variable v_x   : unsigned(15 downto 0);
        variable v_row : unsigned(1 downto 0);
        variable v_cs, v_js : unsigned(7 downto 0);
        variable v_m, v_xa, v_xb, v_xc, v_xf : unsigned(15 downto 0);
        variable v_hd, v_hz : unsigned(7 downto 0);
        variable v_ty  : signed(16 downto 0);
        variable v_ts  : signed(14 downto 0);
        variable v_zx, v_zy : signed(9 downto 0);
        variable v_o17 : signed(16 downto 0);
        variable v_o15 : signed(14 downto 0);
        variable v_en, v_sb : std_logic;
        variable v_j   : unsigned(1 downto 0);
        variable v_evq, v_isa : std_logic;
        variable v_bx  : signed(18 downto 0);
        variable v_gs  : signed(18 downto 0);
        variable v_gc  : unsigned(5 downto 0);
        variable v_ea, v_eb : signed(23 downto 0);
        variable v_ya, v_yb : signed(9 downto 0);
        variable v_su  : unsigned(10 downto 0);
        variable v_s5  : unsigned(11 downto 0);
        variable v_lv  : unsigned(7 downto 0);
        variable v_p, v_q : signed(15 downto 0);
        variable v_clr, v_ini : std_logic;
    begin
        if rising_edge(clk) then
            -- stage 2: column counter A
            if s_st(0) = '1' then
                x_X <= unsigned(l_X0(20 downto 6)); x_cross <= '0';
            elsif s_gav_sr(0) = '1' or s_gav_sr(8) = '1' then
                v_x := resize(x_X, 16) + 64;
                if v_x >= resize(c_cwp, 16) then
                    v_x := v_x - resize(c_cwp, 16); x_cross <= '1';
                else
                    x_cross <= '0';
                end if;
                x_X <= v_x(14 downto 0);
            else
                x_cross <= '0';
            end if;
            -- stage 16: column counter B (the same count, 14 clocks later)
            if s_st(14) = '1' then
                b_X <= unsigned(l_X0(20 downto 6)); b_cross <= '0';
            elsif s_gav_sr(14) = '1' or s_gav_sr(22) = '1' then
                v_x := resize(b_X, 16) + 64;
                if v_x >= resize(c_cwp, 16) then
                    v_x := v_x - resize(c_cwp, 16); b_cross <= '1';
                else
                    b_cross <= '0';
                end if;
                b_X <= v_x(14 downto 0);
            else
                b_cross <= '0';
            end if;

            -- fetch start
            if x_cross = '1' then
                p_c <= p_c + 1;
                p_XC <= p_XC + signed(resize(c_cwh, 19));
                x_ah <= x_X(5 downto 4);
                f_n <= to_unsigned(1, 3); f_ev <= p_c(0) xor l_r(0); f_ls <= '0';
            elsif pp_go = '1' then
                p_c <= l_ci;
                p_XC <= l_XC;
                f_n <= to_unsigned(1, 3); f_ev <= not (l_ci(0) xor l_r(0)); f_ls <= '1';
            elsif f_n /= 0 then
                if f_n = 6 then f_n <= (others => '0'); else f_n <= f_n + 1; end if;
            end if;

            -- F1: the vertex (column, row) and the hash's first round; its wave phase 16 c + 8 j + drift.
            -- k: 0 Lm, 1 L0, 2 Lp, 3 Rm, 4 R0, 5 Rp
            f1_v <= '0';
            if f_n /= 0 then
                f1_v <= '1';
                f1_k <= f_n - 1;
                if f_n >= 4 then v_cs := p_c + 1; v_row := resize(f_n - 4, 2);
                else v_cs := p_c; v_row := resize(f_n - 1, 2); end if;
                case to_integer(v_row) is
                    when 0      => v_js := l_r - 1;
                    when 1      => v_js := l_r;
                    when others => v_js := l_r + 1;
                end case;
                v_m := v_cs & v_js;
                v_xa := v_m xor ("0000000" & v_m(15 downto 7));
                f1_x <= v_xa + (v_xa(12 downto 0) & "000");
                f1_w <= (v_cs(3 downto 0) & "0000") + (v_js(4 downto 0) & "000") + c_ph8;
                f1_par <= v_cs(0) xor v_js(0);
            end if;
            -- F2, F3: the second and third rounds
            v_xb := f1_x xor ("00000" & f1_x(15 downto 5));
            f2_x <= v_xb + (v_xb(13 downto 0) & "00");
            f2_w <= f1_w; f2_par <= f1_par; f2_k <= f1_k; f2_v <= f1_v;
            v_xc := f2_x xor ("000000" & f2_x(15 downto 6));
            f3_x <= v_xc + (v_xc(11 downto 0) & "0000");
            f3_w <= f2_w; f3_par <= f2_par; f3_k <= f2_k; f3_v <= f2_v;
            -- F4: table addresses: displacement phase = hash / 2 + wave, z phase = its own hash / 2 + wave + 1/4
            v_xf := f3_x xor (x"00" & f3_x(15 downto 8));
            v_hd := v_xf(11 downto 4) xor (v_xf(6 downto 0) & '0');
            v_hz := v_xf(7 downto 0) xor v_xf(10 downto 3);
            f4_ad <= ('0' & v_hd(7 downto 1)) + f3_w;
            f4_az <= ('0' & v_hz(7 downto 1)) + f3_w + 64;
            f4_par <= f3_par; f4_k <= f3_k; f4_v <= f3_v;
            -- F5 (block RAM), F6 (fabric)
            f5_k <= f4_k; f5_v <= f4_v; f5_par <= f4_par;
            ty_f <= ty_q; ts_f <= ts_q; tzx_f <= tzx_q; tzy_f <= tzy_q;
            f6_k <= f5_k; f6_v <= f5_v; f6_par <= f5_par;

            -- the derived registers: cleared at det + 1, given their constants at det + 2 (the row phase for
            -- the bases, +- the row slope for the diagonals' steps), then one add or subtract per read
            if f_n = 1 then v_clr := '1'; else v_clr := '0'; end if;
            if f1_v = '1' and f1_k = 0 then v_ini := '1'; else v_ini := '0'; end if;
            v_ty := resize(signed(ty_f), 17);
            v_ts := signed(ts_f(14 downto 0));
            v_zx := signed(resize(tzx_f, 10));
            v_zy := signed(resize(tzy_f, 10));
            if f6_par = '1' then v_zx := v_zx + resize(c_ox, 10); v_zy := v_zy + resize(c_oy, 10); end if;
            -- bases: polyline (vr - Ty(L0)); diagonal of quad r (even: vr - Ty(L0), odd: vr - rhl - Ty(Lp));
            -- diagonal of quad r - 1 (even: vr - Ty(L0), odd: vr + rhl - Ty(Lm))
            if v_clr = '1' then d_B(0) <= (others => '0');
            elsif v_ini = '1' then d_B(0) <= d_B(0) + l_vr;
            elsif f6_v = '1' and f6_k = 1 then d_B(0) <= d_B(0) - v_ty; end if;
            v_en := '0'; v_o17 := v_ty; v_sb := '1';
            if v_ini = '1' then v_en := '1'; v_o17 := l_vr; v_sb := '0';
            elsif f6_v = '1' and ((f6_k = 1 and f_ev = '1') or (f6_k = 2 and f_ev = '0')) then v_en := '1';
            elsif f6_v = '1' and f6_k = 5 and f_ev = '0' then v_en := '1'; v_o17 := signed(resize(c_rhl, 17)); end if;
            if v_clr = '1' then d_B(1) <= (others => '0');
            elsif v_en = '1' then d_B(1) <= signed(f_addsub(unsigned(d_B(1)), unsigned(v_o17), v_sb)); end if;
            v_en := '0'; v_o17 := v_ty; v_sb := '1';
            if v_ini = '1' then v_en := '1'; v_o17 := l_vr; v_sb := '0';
            elsif f6_v = '1' and ((f6_k = 1 and f_ev = '1') or (f6_k = 0 and f_ev = '0')) then v_en := '1';
            elsif f6_v = '1' and f6_k = 3 and f_ev = '0' then v_en := '1'; v_o17 := signed(resize(c_rhl, 17)); v_sb := '0'; end if;
            if v_clr = '1' then d_B(2) <= (others => '0');
            elsif v_en = '1' then d_B(2) <= signed(f_addsub(unsigned(d_B(2)), unsigned(v_o17), v_sb)); end if;
            -- steps: polyline (Ts(R0) - Ts(L0)); diagonal of quad r (even: rs + Ts(Rp) - Ts(L0), odd: -rs +
            -- Ts(R0) - Ts(Lp)); diagonal of quad r - 1 (even: -rs + Ts(Rm) - Ts(L0), odd: rs + Ts(R0) - Ts(Lm))
            if v_clr = '1' then d_S(0) <= (others => '0');
            elsif f6_v = '1' and f6_k = 1 then d_S(0) <= d_S(0) - v_ts;
            elsif f6_v = '1' and f6_k = 4 then d_S(0) <= d_S(0) + v_ts; end if;
            v_en := '0'; v_o15 := v_ts; v_sb := '0';
            if v_ini = '1' then v_en := '1'; v_o15 := c_rs; v_sb := not f_ev;
            elsif f6_v = '1' and ((f6_k = 1 and f_ev = '1') or (f6_k = 2 and f_ev = '0')) then v_en := '1'; v_sb := '1';
            elsif f6_v = '1' and ((f6_k = 5 and f_ev = '1') or (f6_k = 4 and f_ev = '0')) then v_en := '1'; end if;
            if v_clr = '1' then d_S(1) <= (others => '0');
            elsif v_en = '1' then d_S(1) <= signed(f_addsub(unsigned(d_S(1)), unsigned(v_o15), v_sb)); end if;
            v_en := '0'; v_o15 := v_ts; v_sb := '0';
            if v_ini = '1' then v_en := '1'; v_o15 := c_rs; v_sb := f_ev;
            elsif f6_v = '1' and ((f6_k = 1 and f_ev = '1') or (f6_k = 0 and f_ev = '0')) then v_en := '1'; v_sb := '1';
            elsif f6_v = '1' and ((f6_k = 3 and f_ev = '1') or (f6_k = 4 and f_ev = '0')) then v_en := '1'; end if;
            if v_clr = '1' then d_S(2) <= (others => '0');
            elsif v_en = '1' then d_S(2) <= signed(f_addsub(unsigned(d_S(2)), unsigned(v_o15), v_sb)); end if;
            -- tone parts: lx dz along the three row edges (right - left), ly dz along the two borders
            for i in 0 to 2 loop
                if v_clr = '1' then d_TX(i) <= (others => '0');
                elsif f6_v = '1' and (f6_k = i or f6_k = i + 3) then
                    d_TX(i) <= signed(f_addsub(unsigned(d_TX(i)), unsigned(v_zx), not f6_k(2) and not (f6_k(1) and f6_k(0))));
                end if;
            end loop;
            for i in 0 to 1 loop
                if v_clr = '1' then d_L(i) <= (others => '0');
                elsif f6_v = '1' and f6_k = i then d_L(i) <= d_L(i) - v_zy;
                elsif f6_v = '1' and f6_k = i + 1 then d_L(i) <= d_L(i) + v_zy; end if;
                if v_clr = '1' then d_R(i) <= (others => '0');
                elsif f6_v = '1' and f6_k = i + 3 then d_R(i) <= d_R(i) - v_zy;
                elsif f6_v = '1' and f6_k = i + 4 then d_R(i) <= d_R(i) + v_zy; end if;
            end loop;
            if f6_v = '1' and f6_k = 5 then t_sr(0) <= '1'; else t_sr(0) <= '0'; end if;
            t_sr(1 to 2) <= t_sr(0 to 1);
            t_ls(0) <= f_ls; t_ls(1 to 2) <= t_ls(0 to 1);

            -- the grid reads: facet j = 0 A0, 1 B0, 2 A1, 3 B1 (quad r - 1, quad r; A = the upper triangle).
            -- G1: lattice x of the sample (1/4 or 3/4 of the column, by the diagonal's direction)
            g1_v <= '0';
            if f_n /= 0 and f_n <= 4 then
                g1_v <= '1';
                v_j := resize(f_n - 1, 2);
                g1_j <= v_j;
                v_evq := f_ev xor (not v_j(1));
                v_isa := not v_j(0);
                if v_evq = v_isa then g1_t <= p_XC + signed(resize(c_c12, 19));
                else g1_t <= p_XC + signed(resize(c_c4, 19)); end if;
            end if;
            -- G2: + the row's shear and the screen centre -> grid column (clamped), with the facet's grid row
            if g1_j(1) = '1' then v_bx := l_BX1; else v_bx := l_BX0; end if;
            v_gs := g1_t + v_bx;
            if v_gs(18) = '1' then v_gc := (others => '0');
            elsif v_gs(17 downto 16) /= "00" then v_gc := (others => '1');
            else v_gc := unsigned(v_gs(15 downto 10)); end if;
            if c_sd = '1' then
                if v_gc < 7 then v_gc := to_unsigned(7, 6); elsif v_gc > 52 then v_gc := to_unsigned(52, 6); end if;
            elsif v_gc > 59 then
                v_gc := to_unsigned(59, 6);
            end if;
            if g1_j(1) = '1' then g2_a <= l_gr1 & v_gc; else g2_a <= l_gr0 & v_gc; end if;
            g2_j <= g1_j; g2_v <= g1_v;
            -- G3 (block RAM), G4 (fabric: the bank written last field), G5: 32 - mean / 2 + the level (units of 4)
            g3_j <= g2_j; g3_v <= g2_v;
            if w_bank = '1' then g4_d <= gr0_q; else g4_d <= gr1_q; end if;
            g4_j <= g3_j; g4_v <= g3_v;
            if g4_v = '1' then
                d_G(to_integer(g4_j)) <= resize(c_base, 9) + signed(resize(g4_d & '0', 9));
            end if;

            -- det + 13: the sub-pixel products step * a (a = the crossing pixel's quarters of a pixel past the
            -- border)
            for k in 0 to 2 loop
                v_p := (others => '0'); v_q := (others => '0');
                if x_ah(1) = '1' then v_p := resize(shift_right(d_S(k), 1), 16); end if;
                if x_ah(0) = '1' then v_q := resize(shift_right(d_S(k), 2), 16); end if;
                e_P(k) <= v_p + v_q;
            end loop;
            -- det + 13: the facets' tone sums (A0, B0, A1, B1) = relief + base + camera; det + 14: the six steps
            -- (Ink: two): step = (5 S) >> 6, the rounding half is in the base
            if f_ev = '1' then v_ya := d_L(0); v_yb := d_R(0); else v_ya := d_R(0); v_yb := d_L(0); end if;
            t_S(0) <= resize(d_TX(0), 12) + resize(v_ya, 12) + resize(d_G(0), 12);
            t_S(1) <= resize(d_TX(1), 12) + resize(v_yb, 12) + resize(d_G(1), 12);
            if f_ev = '1' then v_ya := d_R(1); v_yb := d_L(1); else v_ya := d_L(1); v_yb := d_R(1); end if;
            t_S(2) <= resize(d_TX(1), 12) + resize(v_ya, 12) + resize(d_G(2), 12);
            t_S(3) <= resize(d_TX(2), 12) + resize(v_yb, 12) + resize(d_G(3), 12);
            if t_sr(1) = '1' then
                for j in 0 to 3 loop
                    -- on S + 1024 (unsigned: no sign-extended self-add), so the step comes out + 80
                    v_su := (not t_S(j)(10)) & unsigned(t_S(j)(9 downto 0));
                    v_s5 := ('0' & v_su) + ("000" & v_su(10 downto 2));
                    v_lv := v_s5(11 downto 4);
                    if s_ink = '1' then
                        if v_su >= 1062 then t_code(j) <= "101"; else t_code(j) <= "000"; end if;
                    elsif v_lv < 80 then t_code(j) <= "000";
                    elsif v_lv > 85 then t_code(j) <= "101";
                    else t_code(j) <= resize(v_lv - 80, 3); end if;
                end loop;
            end if;
            -- the tones in use: taken where the pixel path meets the new column (stage 22), or at once for a
            -- line's first column
            if (t_sr(2) = '1' and t_ls(2) = '1') or cr_sr(21) = '1' then a_code <= t_code; end if;

            -- stage 17: the edge accumulators.  E = base - step * (px into the column); the step registers
            -- double as the serial multiplier's shifting operand at a line start.
            for k in 0 to 2 loop
                v_ea := e_E(k); v_eb := e_st(k);
                if t_sr(1) = '1' then
                    v_ea := shift_left(resize(d_B(k), 24), 7);
                    if t_ls(1) = '1' then v_eb := (others => '0'); else v_eb := resize(e_P(k), 24); end if;
                end if;
                if t_sr(1) = '1' or (e_sn /= 0 and e_xs(14) = '1') or (e_sn = 0 and e_fin = '0' and (s_gav_sr(15) = '1' or s_gav_sr(23) = '1')) then
                    e_E(k) <= v_ea - v_eb;
                end if;
                if t_sr(1) = '1' then
                    if t_ls(1) = '1' then e_st(k) <= shift_left(resize(d_S(k), 24), 8);
                    else e_st(k) <= resize(d_S(k), 24); end if;
                elsif e_sn /= 0 then
                    e_st(k) <= shift_right(e_st(k), 1);
                elsif e_fin = '1' then
                    e_st(k) <= resize(d_S(k), 24);
                end if;
            end loop;
            if t_sr(1) = '1' then
                e_fin <= '0';
                if t_ls(1) = '1' then e_xs <= unsigned(l_X0(20 downto 6)); e_sn <= to_unsigned(15, 4);
                else e_sn <= (others => '0'); end if;
            elsif e_sn /= 0 then
                e_xs <= e_xs(13 downto 0) & '0';
                e_sn <= e_sn - 1;
                if e_sn = 1 then e_fin <= '1'; end if;
            else
                e_fin <= '0';
            end if;
        end if;
    end process p_col;

    ---------------------------------------------------------------------------
    -- [E] MESH, the pixel path (stages 17 .. 33)
    --   18  the edge functions saturated to +-32 lines; step * lean for the gradient norms
    --   19  |E| of the polyline and of the pixel's own quad's diagonal; w = |dE/dx| + |dE/dy|
    --   20  gradient class        21  |E| >> class exponent        22  distance, Q3 px
    --   23  the pixel's facet tone, its neighbours across the diagonal and the polyline, their shares
    --   24 .. 26  blend across the diagonal     27 .. 29  blend across the polyline
    --   30 .. 33  blend across the column border with the pixel beside (Fine: the line over the facet)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_xr  : unsigned(14 downto 0);
        variable v_lf  : std_logic;
        variable v_sq  : signed(3 downto 0);
        variable v_b1  : std_logic;
        variable v_e, v_n : signed(15 downto 0);
        variable v_e24 : signed(23 downto 0);
        variable v_gy  : signed(9 downto 0);
        variable v_ag  : unsigned(7 downto 0);
        variable v_ca, v_cb, v_co, v_cd, v_cp : unsigned(2 downto 0);
        variable v_dm  : unsigned(5 downto 0);
        variable v_l   : signed(7 downto 0);
        variable v_nb  : unsigned(7 downto 0);
        variable v_ul, v_ur : std_logic;
        variable v_t10 : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 17 .. 22: the column border: distance to the nearer side (within 8 px), scaled by 1 / (1 + |lean|)
            v_xr := c_cwp - b_X;
            if b_X(14 downto 9) = "000000" then v_lf := '1'; else v_lf := '0'; end if;
            if v_lf = '1' then c17_xn <= '0' & b_X(8 downto 0);
            elsif v_xr(14 downto 9) = "000000" then c17_xn <= '0' & v_xr(8 downto 0);
            else c17_xn <= (others => '1'); end if;
            cr_sr(17) <= b_cross; lf_sr(17) <= v_lf;
            cr_sr(18 to 30) <= cr_sr(17 to 29); lf_sr(18 to 30) <= lf_sr(17 to 29);
            if c17_xn(9) = '1' then c18_u <= (others => '1'); else c18_u <= "000" & c17_xn(8 downto 0) & "000"; end if;
            c19_t <= f_bar(c18_u, c_wcc(4 downto 2));
            c20_d <= f_dist(c19_t, c_wcc(1 downto 0));
            c21_d <= c20_d; c22_d <= c21_d;

            -- 18
            v_b1 := not e_E(0)(23);
            b1_sr(18) <= v_b1; b1_sr(19 to 22) <= b1_sr(18 to 21);
            for k in 0 to 1 loop                        -- 0 the polyline, 1 the diagonal of the pixel's own quad
                if k = 0 then v_e24 := e_E(0); elsif v_b1 = '1' then v_e24 := e_E(1); else v_e24 := e_E(2); end if;
                if v_e24(23) = '0' and v_e24(22 downto 18) /= "00000" then s18_E(k) <= to_signed(16383, 16);
                elsif v_e24(23) = '1' and v_e24(22 downto 18) /= "11111" then s18_E(k) <= to_signed(-16384, 16);
                else s18_E(k) <= v_e24(19 downto 4); end if;
            end loop;
            v_sq := e_st(0)(14 downto 11);
            s18_pp <= v_sq * c_lplq(6 downto 2);
            if v_sq(3) = '1' then s18_ap <= unsigned(-v_sq); else s18_ap <= unsigned(v_sq); end if;
            if v_b1 = '1' then v_sq := e_st(1)(14 downto 11); else v_sq := e_st(2)(14 downto 11); end if;
            s18_pd <= v_sq * c_lplq(6 downto 2);
            if v_sq(3) = '1' then s18_ad <= unsigned(-v_sq); else s18_ad <= unsigned(v_sq); end if;
            -- 19
            v_n := -s18_E(0);
            if s18_E(0)(15) = '1' then s19_up <= unsigned(v_n(14 downto 0)); else s19_up <= unsigned(s18_E(0)(14 downto 0)); end if;
            v_e := s18_E(1);
            v_n := -v_e;
            if v_e(15) = '1' then s19_ud <= unsigned(v_n(14 downto 0)); else s19_ud <= unsigned(v_e(14 downto 0)); end if;
            sb_sr(19) <= not v_e(15); sb_sr(20 to 22) <= sb_sr(19 to 21);
            v_gy := to_signed(16, 10) - resize(s18_pp, 10);
            if v_gy(9) = '1' then v_ag := unsigned(resize(-v_gy, 8)); else v_ag := unsigned(v_gy(7 downto 0)); end if;
            s19_wp <= resize(s18_ap & "00", 8) + v_ag;
            v_gy := to_signed(16, 10) - resize(s18_pd, 10);
            if v_gy(9) = '1' then v_ag := unsigned(resize(-v_gy, 8)); else v_ag := unsigned(v_gy(7 downto 0)); end if;
            s19_wd <= resize(s18_ad & "00", 8) + v_ag;
            -- 20
            s20_up <= s19_up; s20_ud <= s19_ud;
            s20_cp <= f_class(s19_wp); s20_cd <= f_class(s19_wd);
            -- 21
            s21_tp <= f_bar(s20_up, s20_cp(4 downto 2)); s21_qp <= s20_cp(1 downto 0);
            s21_td <= f_bar(s20_ud, s20_cd(4 downto 2)); s21_qd <= s20_cd(1 downto 0);
            -- 22
            s22_dp <= f_dist(s21_tp, s21_qp);
            s22_dd <= f_dist(s21_td, s21_qd);

            -- 23: tones and shares
            if b1_sr(22) = '1' then v_ca := a_code(2); v_cb := a_code(3); v_cp := a_code(1);
            else v_ca := a_code(0); v_cb := a_code(1); v_cp := a_code(2); end if;
            if s_quad = '1' then                        -- Quads: the whole quad takes its upper triangle's tone
                v_cb := v_ca;
                if b1_sr(22) = '1' then v_cp := a_code(0); end if;
            end if;
            if sb_sr(22) = '1' then v_co := v_cb; v_cd := v_ca; else v_co := v_ca; v_cd := v_cb; end if;
            s23_own <= f_tone(v_co, s_fine and not s_ink);
            s23_od  <= f_tone(v_cd, s_fine and not s_ink);
            s23_op  <= f_tone(v_cp, s_fine and not s_ink);
            if s_fine = '1' then
                s23_cd <= (others => '0'); s23_cp <= (others => '0');
                v_dm := s22_dp;
                if c22_d < v_dm then v_dm := c22_d; end if;
                if s_quad = '0' and s22_dd < v_dm then v_dm := s22_dd; end if;
                v_l := signed(resize(c_hw4(5 downto 1), 8)) + 4 - signed(resize(v_dm, 8));
                if v_l(7) = '1' then c3_d(23) <= (others => '0');
                elsif v_l > 8 then c3_d(23) <= to_unsigned(8, 4);
                else c3_d(23) <= unsigned(v_l(3 downto 0)); end if;
            else
                if s_quad = '1' then s23_cd <= (others => '0'); else s23_cd <= f_cov(s22_dd); end if;
                s23_cp <= f_cov(s22_dp);
                c3_d(23) <= '0' & f_cov(c22_d);
            end if;
            for i in 24 to 30 loop c3_d(i) <= c3_d(i - 1); end loop;
            -- 24 .. 26: across the diagonal
            s24_df <= signed(resize(s23_od, 9)) - signed(resize(s23_own, 9));
            s24_own <= s23_own; s24_op <= s23_op; s24_cd <= s23_cd; s24_cp <= s23_cp;
            s25_m <= s24_df * signed('0' & s24_cd);
            s25_own <= s24_own; s25_op <= s24_op; s25_cp <= s24_cp;
            v_t10 := signed(resize(s25_own, 10)) + resize(shift_right(s25_m, 3), 10);
            s26_t <= unsigned(v_t10(7 downto 0));
            s26_op <= s25_op; s26_cp <= s25_cp;
            -- 27 .. 29: across the polyline
            s27_df <= signed(resize(s26_op, 9)) - signed(resize(s26_t, 9));
            s27_t <= s26_t; s27_cp <= s26_cp;
            s28_m <= s27_df * signed('0' & s27_cp);
            s28_t <= s27_t;
            v_t10 := signed(resize(s28_t, 10)) + resize(shift_right(s28_m, 3), 10);
            s29_t <= unsigned(v_t10(7 downto 0));
            -- 30: the window's centre (29 = the pixel after, 31 = the pixel before)
            s30_t <= s29_t;
            -- 31: the neighbour across the column border (Fine: the line's tone) and its share
            v_ul := cr_sr(30) and lf_sr(30);
            v_ur := cr_sr(29) and not lf_sr(30);
            if s_fine = '1' then
                if s_ink = '1' then v_nb := not s30_t; else v_nb := (others => '1'); end if;
                s31_c <= c3_d(30);
            else
                if v_ul = '1' then v_nb := s31_t; else v_nb := s29_t; end if;
                if v_ul = '1' or v_ur = '1' then s31_c <= c3_d(30); else s31_c <= (others => '0'); end if;
            end if;
            s31_df <= signed(resize(v_nb, 9)) - signed(resize(s30_t, 9));
            s31_t <= s30_t;
            -- 32, 33
            s32_m <= s31_df * signed('0' & s31_c);
            s32_t <= s31_t;
            v_t10 := signed(resize(s32_t, 10)) + resize(shift_right(s32_m, 3), 10);
            s33_t <= unsigned(v_t10(7 downto 0));
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING (stages 34 .. 37): [1 2 1] on the tone, then the
    -- luma 64 + 876 * 13/16 * tone / 255 as shift-adds of the tap sum (white
    -- 771), blanking gate.  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_y : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 34, 35: taps t(n + 1) = s33, t(n) = s34, t(n - 1) = s35
            s34_t <= s33_t; s35_t <= s34_t;
            s35_s <= resize(s33_t, 10) + resize(s34_t & '0', 10) + resize(s35_t, 10);
            -- 36
            v_y := to_unsigned(64, 10) + resize(s35_s(9 downto 1), 10) + resize(s35_s(9 downto 3), 10)
                   + resize(s35_s(9 downto 4), 10) + resize(s35_s(9 downto 7), 10);
            s36_y <= v_y;
            -- 37: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s37_y <= s36_y;
            else
                s37_y <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s37_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= sd_q(0);
    data_out.vsync_n <= sd_q(1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture lowpoly;
