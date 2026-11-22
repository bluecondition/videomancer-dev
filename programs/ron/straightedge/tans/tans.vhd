-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Tans v0.1.0 -- Straightedge.  The tangram: seven flat pieces cut from one square (two large triangles, a medium
-- triangle, two small triangles, a square and a parallelogram), each its own flat grey on black, assembled into
-- the square or one of seven silhouettes with hairline black seams.  Every few seconds the pieces glide and turn,
-- rigidly, into the next figure.  Black, white and greys only (U = V = 512).
--
--   pieces    23 straight edges, each the zero of an affine function e = d S - n . (p - P) in pixel widths (Q12):
--             one accumulator per edge, one add per pixel.  A piece's coverage comes from the minimum of its
--             edges (clamped to 0 .. 127 sixteenths): Bold = min(m, 16), Fine = a 3 virtual-px outline.  The edges
--             sit 1.5 virtual px inside the true outline, so assembled pieces show a clean black seam.
--   tone      Grey: each piece its own grey; overlapping pieces (in flight, scattered) = the brightest plus a
--             quarter of the rest (a lighter film).  Ink: white pieces, overlaps XOR.
--   motion    a per-field microcoded engine (tans.py) interpolates every piece between figure A and the next
--             figure B (position linearly, angle the short way, smoothstep ease after half a step of dwell) and
--             writes the 23 edge functions (pixel step, line wrap, field seed) into the edge table.
--   camera    the box-filtered luma, sampled once per 32 x 32 virtual-px block into a two-bank block-RAM grid as a
--             level 0..3 (0 = black); read back the next field and held across the block: inside the pieces the
--             level replaces the piece grey by K6 with one of three flat greys, on the ground it lays two dark steps.
--   mirror    S8: the screen folded left-right at its centre (a line buffer of the right half, one line late).
--
--   K1 X / K2 Y  the figure's position       K3 Scale (exponential)        K4 Figure (8, stepped)
--   K5 Drift     bipolar speed of the cycle  K6 Camera                     S7 Glide   S8 Form   S9 Tone
--   S10 Weight   S11 Source                  P12 SCATTER: 0 = the figure, 100 % = a composed scatter
--
-- Sections (the integer model .straightedge_work/rb_tans/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [C] PER-FIELD ENGINE                 microcoded (tans.py), sine unit, edge-table writes
--   [D] SOURCE FIELD                     Osc lattice, box, vertical one-pole in a line store, block grid
--   [E] EDGES                            23 affine DDAs and their loader, coverage, tone, camera, mirror
--   [F] OUTPUT CONDITIONING              luma [1 2 1]/4, luma 13/16 toward black, blanking gate
--
-- Timing: T = the g_start clock.  The DDAs hold DDA pixel i at T + 6 + i; a register "at stage S" holds
-- pixel i at T + 6 + i + S; the luma is stage 15, the output register stage 20.  One latency (26) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.tans_pkg.all;
use work.all;

architecture tans of program_top is
    constant C_LAT : integer := 26;   -- = processing_delay_clks
    constant NE    : integer := 23;   -- edges

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
    -- interlaced modes (field lines are two pixels tall): 480i, 576i, 1080i
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
    function f_min6(a, b : unsigned(5 downto 0)) return unsigned is
    begin
        if a < b then return a; else return b; end if;
    end function;
    function f_max8(a, b : unsigned(7 downto 0)) return unsigned is
    begin
        if a > b then return a; else return b; end if;
    end function;
    -- coverage (0 .. 16) times a constant grey (sixteenths), explicit shift-adds
    function f_cg(c : unsigned(4 downto 0); g : integer) return unsigned is
        variable v : unsigned(7 downto 0);
    begin
        v := resize(c, 8);
        case g is
            when 4      => return shift_left(v, 2);
            when 6      => return shift_left(v, 2) + shift_left(v, 1);
            when 7      => return shift_left(v, 3) - v;
            when 9      => return shift_left(v, 3) + v;
            when 10     => return shift_left(v, 3) + shift_left(v, 1);
            when 11     => return shift_left(v, 3) + shift_left(v, 1) + v;
            when 12     => return shift_left(v, 3) + shift_left(v, 2);
            when 13     => return shift_left(v, 3) + shift_left(v, 2) + v;
            when 14     => return shift_left(v, 4) - shift_left(v, 1);
            when others => return shift_left(v, 4) - v;          -- 15
        end case;
    end function;
    -- the camera level of a block luma (8 bits): 0 below 5 (black), 1, 2 from 100, 3 from 149
    function f_level(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v >= 149 then return "11";
        elsif v >= 100 then return "10";
        elsif v >= 5 then return "01";
        else return "00"; end if;
    end function;

    -- radix-4 Booth: the multiple of B for the multiplier bits (b1, b0, b-1), inverted for a subtraction (the +1
    -- is the adder's carry-in, returned in bit 38)
    function f_booth(b : signed(37 downto 0); b1, b0, bm : std_logic) return signed is
        variable v : signed(38 downto 0);
        variable d : std_logic_vector(2 downto 0);
    begin
        d := b1 & b0 & bm;
        case d is
            when "001" | "010" => v := '0' & b;
            when "011"         => v := '0' & b(36 downto 0) & '0';
            when "100"         => v := '1' & not (b(36 downto 0) & '0');
            when "101" | "110" => v := '1' & not b;
            when others        => v := (others => '0');
        end case;
        return v;
    end function;

    type t_int7 is array (0 to 6) of integer;
    constant C_GREY  : t_int7 := (13, 9, 6, 11, 7, 15, 4);     -- = tans.py GREY
    -- each piece's first edge (triangles 3 edges, the square and the parallelogram 4)
    constant C_E0    : t_int7 := (0, 3, 6, 9, 12, 15, 19);

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ilr, s_il : std_logic := '0';
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal z_m    : unsigned(11 downto 0) := to_unsigned(1928, 12);
    signal z_rq   : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal b_mm1  : unsigned(11 downto 0) := to_unsigned(1927, 12);
    signal c_x0   : unsigned(11 downto 0) := (others => '0');
    signal c_w    : unsigned(5 downto 0) := to_unsigned(47, 6);    -- Fine: the outline width in coverage units
    signal c_half : unsigned(9 downto 0) := to_unsigned(960, 10);  -- W / 2 (the mirror's fold)

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
    signal s_st       : std_logic_vector(0 to 16) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- switches (latched at the zone start)
    signal s_glide : std_logic := '1';
    signal s_mir, s_ink, s_fine, s_src : std_logic := '0';

    ---------------------------------------------------------------------------
    -- [C] THE PER-FIELD ENGINE (after Vellum cassini / Hardline flute): 38-bit accumulator, a 64 x 32-bit register
    -- file in block RAM (power-up values: the knob holds and glides at the TOML defaults), one adder, a radix-4
    -- Booth multiplier (18 x 18 signed, 9 steps) and a right shift in 2-bit steps; micro-op = opcode & operand
    ---------------------------------------------------------------------------
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
    signal rf_ram : t_c_tn_rfi := C_TN_RFI;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal u_run : std_logic := '0';
    signal u_ph  : unsigned(1 downto 0) := "11";
    signal u_pc, u_lr : unsigned(9 downto 0) := (others => '0');
    signal u_ir  : std_logic_vector(15 downto 0) := x"F001";      -- the microprogram ROM's output register
    signal u_irf : std_logic_vector(15 downto 0) := x"F001";      -- its fabric copy (from phase 1 on)
    signal u_mul, u_shr, u_halt, u_xo, u_xst : std_logic := '0';  -- decoded at phase 1
    signal u_acc, u_b : signed(37 downto 0) := (others => '0');
    signal u_op  : signed(37 downto 0) := (others => '0');       -- the adder's operand for this clock, prepared
    signal u_cin : std_logic := '0';                              -- one clock ahead (decode / the previous Booth step)
    signal u_cnt : unsigned(4 downto 0) := (others => '0');
    signal u_out : std_logic;
    signal w_in  : signed(37 downto 0);
    signal ft_adr : unsigned(9 downto 0) := (others => '0');
    signal ft_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal l_kick : std_logic := '0';

    -- [C] the sine unit: angle (65536 per turn) -> sin, cos (Q12) from a quarter sine, interpolated
    signal sn_go  : std_logic := '0';
    signal sn_st  : unsigned(3 downto 0) := (others => '0');
    signal sn_th  : unsigned(15 downto 0) := (others => '0');
    signal sn_is, sn_ic : unsigned(8 downto 0) := (others => '0');
    signal sn_fs, sn_fc : unsigned(5 downto 0) := (others => '0');
    signal sn_ns, sn_nc : std_logic := '0';
    signal sn_adr : unsigned(7 downto 0) := (others => '0');
    signal sn_ov  : std_logic := '0';
    signal sn_ov1, sn_ov2 : std_logic := '0';
    signal sn_q, sn_qf : std_logic_vector(15 downto 0) := (others => '0');
    signal sn_t0, sn_t1 : unsigned(12 downto 0) := (others => '0');
    signal sn_d   : unsigned(4 downto 0) := (others => '0');
    signal sn_p   : unsigned(10 downto 0) := (others => '0');
    signal sn_sin, sn_cos : signed(13 downto 0) := (others => '0');

    -- [C] the engine's outputs: the camera's tones
    signal c_kc  : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal c_vk1, c_vk2, c_vk3, c_gg2, c_gg3 : unsigned(7 downto 0) := (others => '0');
    signal c_kx  : std_logic := '1';

    -- [E] edge table: (edge, 0) = the pixel step, (edge, 1) = the line wrap, (edge, 2) = the field seed
    type t_cr is array (0 to 127) of std_logic_vector(31 downto 0);
    signal cr_ram : t_cr := (others => (others => '0'));
    attribute no_rw_check of cr_ram : signal is true;
    signal cr_ra, cr_wa, e_ea : unsigned(6 downto 0) := (others => '0');
    signal cr_q, cr_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal cr_we : std_logic := '0';

    -- [E] edge DDAs and their loader
    type t_s26 is array (0 to NE - 1) of signed(25 downto 0);
    signal ca_inc, ca_acc : t_s26 := (others => (others => '0'));
    signal ca_ld, ca_en : std_logic_vector(NE - 1 downto 0) := (others => '0');
    signal ca_rst : std_logic := '0';
    signal l_run, l_seed : std_logic := '0';
    signal l_t : unsigned(5 downto 0) := (others => '0');
    signal l_v1, l_v2, l_v3, l_o1, l_o2, l_o3 : std_logic := '0';
    signal l_c1, l_c2, l_c3 : unsigned(4 downto 0) := (others => '0');
    -- the table word re-registered in fabric (the block RAM's own output fanned out to all 23 DDAs across the
    -- die was the critical path)
    signal cr_qf : std_logic_vector(25 downto 0) := (others => '0');
    signal b_run, b_end, b_e1, b_e2, b_e3 : std_logic := '0';
    signal b_cnt : unsigned(11 downto 0) := (others => '0');

    -- [E] coverage and tone
    type t_u6x23 is array (0 to NE - 1) of unsigned(5 downto 0);
    signal x1_es : t_u6x23 := (others => (others => '0'));
    type t_u6x7 is array (0 to 6) of unsigned(5 downto 0);
    signal x2_a, x2_b, x3_m : t_u6x7 := (others => (others => '0'));
    type t_u5x7 is array (0 to 6) of unsigned(4 downto 0);
    signal x4_cb, x4_cf, x5_c : t_u5x7 := (others => (others => '0'));
    type t_u8x7 is array (0 to 6) of unsigned(7 downto 0);
    signal x5_t : t_u8x7 := (others => (others => '0'));
    type t_u8x4 is array (0 to 3) of unsigned(7 downto 0);
    signal x6_m : t_u8x4 := (others => (others => '0'));
    type t_u9x4 is array (0 to 3) of unsigned(8 downto 0);
    signal x6_s : t_u9x4 := (others => (others => '0'));
    type t_u6x4 is array (0 to 3) of unsigned(5 downto 0);
    signal x5_bp, x6_c : t_u6x4 := (others => (others => '0'));       -- Bold coverage pair sums (one stage early)
    signal x6_bq0, x6_bq1, x7_bs : unsigned(6 downto 0) := (others => '0');
    signal x7_m0, x7_m1 : unsigned(7 downto 0) := (others => '0');
    signal x7_s0, x7_s1 : unsigned(9 downto 0) := (others => '0');
    signal x7_c0, x7_c1 : unsigned(6 downto 0) := (others => '0');
    signal x8_tm : unsigned(7 downto 0) := (others => '0');
    signal x8_ts : unsigned(10 downto 0) := (others => '0');
    signal x8_cb, x8_cc : unsigned(6 downto 0) := (others => '0');
    signal x9_t8 : unsigned(7 downto 0) := (others => '0');
    signal x9_cbf : unsigned(5 downto 0) := (others => '0');
    signal x9_cb16, x9_c16, x9_v : unsigned(4 downto 0) := (others => '0');
    signal x9_ikl, x9_icb, x9_dcc : unsigned(4 downto 0) := (others => '0');   -- 16 - kl, 16 - Cb16, Cb16 - C16
    signal x9_vk, x9_gg : unsigned(7 downto 0) := (others => '0');
    signal x9_fl : std_logic := '0';
    signal x10_p1 : unsigned(11 downto 0) := (others => '0');
    signal x10_p2 : unsigned(13 downto 0) := (others => '0');
    signal x10_p3 : unsigned(11 downto 0) := (others => '0');
    signal x10_ik : unsigned(7 downto 0) := (others => '0');
    signal x11_t, x12_t, x13_t, x14_t : unsigned(7 downto 0) := (others => '0');

    -- [E] the camera grid (read side) and its pipeline
    signal m_yv   : unsigned(18 downto 0) := (others => '0');      -- this line's virtual y, Q8
    signal m_rb0  : std_logic := '0';
    signal m_base : unsigned(10 downto 0) := (others => '0');      -- 60 * block row
    signal m_xr   : unsigned(11 downto 0) := (others => '0');      -- the DDA pixel's virtual x, h
    signal m_bank : std_logic := '0';
    type t_grid is array (0 to 4095) of unsigned(1 downto 0);
    signal gr_ram : t_grid := (others => (others => '0'));
    attribute no_rw_check of gr_ram : signal is true;
    signal gr_ra, gr_wa : unsigned(11 downto 0) := (others => '0');
    signal gr_q, gr_f, gr_wd : unsigned(1 downto 0) := (others => '0');
    signal gr_we : std_logic := '0';
    signal m_xw   : unsigned(11 downto 0) := (others => '0');
    type t_u2x9 is array (4 to 9) of unsigned(1 downto 0);
    signal x_l : t_u2x9 := (others => (others => '0'));

    -- [E] the mirror's line buffer (the right half of the tone, read back folded on the next line)
    type t_mb is array (0 to 1023) of unsigned(7 downto 0);
    signal mb_ram : t_mb := (others => (others => '0'));
    attribute no_rw_check of mb_ram : signal is true;
    signal mb_rp, mb_wa : unsigned(9 downto 0) := (others => '0');
    signal mb_dn, mb_we : std_logic := '0';
    signal mb_q, mb_f, mb_wd : unsigned(7 downto 0) := (others => '0');
    signal r_run : std_logic_vector(1 to 12) := (others => '0');   -- b_run delayed to stage 12
    signal m_first, m_l0 : std_logic := '0';

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_t0, s_tv, s_tvd, s_sfirst : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_v1, s_v2, s_v2d, s_v3, s_v4, s_ld, s_fl, s_seen, s_efl, s_ffl : std_logic := '0';
    signal s_col, s_c2, s_bcol, s_ecol, s_fcol : unsigned(10 downto 0) := (others => '0');
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q, bx_f : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(5 downto 0) := (others => '0');
    signal s_b32 : std_logic := '1';                                  -- 32-px box on the 1920-wide rasters, else 16
    signal s_df   : signed(8 downto 0) := (others => '0');
    signal s_bsum : unsigned(13 downto 0) := (others => '0');
    signal s_ev, s_fv : std_logic := '0';
    signal s_eb, s_eq, s_fq : unsigned(15 downto 0) := (others => '0');
    signal s_dd : signed(16 downto 0) := (others => '0');
    signal c_vsh  : unsigned(1 downto 0) := "00";                    -- one-pole shift: 4 - c_vsh
    type t_lq is array (0 to 511) of unsigned(15 downto 0);
    signal lq_ram : t_lq := (others => (others => '0'));
    attribute no_rw_check of lq_ram : signal is true;
    signal lq_q1, lq_wd : unsigned(15 downto 0) := (others => '0');
    signal lq_wa : unsigned(8 downto 0) := (others => '0');
    signal lq_we : std_logic := '0';

    -- [F] output conditioning
    signal s_y15, s_yd1, s_yd2, s_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ya : unsigned(11 downto 0) := (others => '0');
    signal s_y26, s_y27, s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- RAMs and ROMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    -- edge table: written by the engine in the zone, read by the loader
    p_crram : process(clk)
    begin
        if rising_edge(clk) then
            cr_q <= cr_ram(to_integer(cr_ra));
            if cr_we = '1' then cr_ram(to_integer(cr_wa)) <= cr_wd; end if;
        end if;
    end process p_crram;

    -- box delay: read 30 (or 14) samples behind the write (+ two registers = 32 or 16)
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_v2 = '1' then bx_ram(to_integer(bx_wp)) <= s_tv; end if;
        end if;
    end process p_bxram;

    -- source line store (quarter horizontal resolution): the one-pole's own read, ahead of its write
    p_lqram : process(clk)
    begin
        if rising_edge(clk) then
            lq_q1 <= lq_ram(to_integer(s_c2(10 downto 2)));
            if lq_we = '1' then lq_ram(to_integer(lq_wa)) <= lq_wd; end if;
        end if;
    end process p_lqram;

    -- the block grid: two banks of 34 x 60 levels; this field writes one bank and reads the other
    p_grram : process(clk)
    begin
        if rising_edge(clk) then
            gr_q <= gr_ram(to_integer(gr_ra));
            if gr_we = '1' then gr_ram(to_integer(gr_wa)) <= gr_wd; end if;
        end if;
    end process p_grram;

    -- the mirror's line buffer: every address is read (for both halves) before this line overwrites it
    p_mbram : process(clk)
    begin
        if rising_edge(clk) then
            mb_q <= mb_ram(to_integer(mb_rp));
            if mb_we = '1' then mb_ram(to_integer(mb_wa)) <= mb_wd; end if;
        end if;
    end process p_mbram;

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES, switches, per-field constants
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            -- timing ID -> steps (decoded free-running)
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

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_mir   <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_il    <= s_ilr;
                if s_xsr = 2 then s_b32 <= '1'; else s_b32 <= '0'; end if;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); c_x0 <= to_unsigned(480, 12);
                    when 3      => s_oxs <= to_unsigned(153, 9); c_x0 <= to_unsigned(0, 12);
                    when others => s_oxs <= to_unsigned(102, 9); c_x0 <= to_unsigned(0, 12);
                end case;
            end if;
            g_wm1  <= s_w - 1;
            b_mm1  <= s_w + 7;
            z_m    <= s_w + 8;
            c_half <= s_w(10 downto 1);
            s_oys  <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);
            case to_integer(s_xs) is               -- line height in pixel widths, Q10
                when 4      => z_rq <= resize(s_ys & '0', 12);
                when 3      => z_rq <= to_unsigned(1024, 12);
                when others => z_rq <= resize(s_ys & "00", 12);
            end case;
            -- Fine: the outline width (3 virtual px, at least 1.5 px; 47/16 at 1080p so that an interior pixel,
            -- clamped at 63, is never outline) in coverage units (eighths when interlaced)
            if s_xs = 4 then
                if s_il = '1' then c_w <= to_unsigned(12, 6); else c_w <= to_unsigned(24, 6); end if;
            elsif s_xs = 3 then c_w <= to_unsigned(32, 6);
            elsif s_il = '1' then c_w <= to_unsigned(24, 6);
            else c_w <= to_unsigned(47, 6); end if;
            -- the one-pole's rate, by raster
            if s_xs = 4 then
                if s_il = '1' then c_vsh <= "10"; else c_vsh <= "01"; end if;
            else
                if s_il = '1' then c_vsh <= "01"; else c_vsh <= "00"; end if;
            end if;

            -- sync delay, generated-avid and start-pulse shift registers (field_n is tied high by the core on
            -- every input, so it only gets a register)
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 15);
        end if;
    end process p_ctrl;

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
    -- [C] THE PER-FIELD ENGINE.  Started at every zone start from address 0 (a
    -- stalled or garbled run cannot outlive a field); three clocks per micro-op
    -- (register-file read, operand / decode / next address, execute), MUL in
    -- 9 radix-4 Booth steps, SHR in 2-bit steps.  OUT ED writes the edge table;
    -- HALT kicks the DDA loader (the field seeds).
    ---------------------------------------------------------------------------
    with u_irf(3 downto 0) select w_in <=
        signed(resize(s_xs, 38))            when x"8",
        signed(resize(z_rq, 38))            when x"9",
        signed(resize(z_m, 38))             when x"A",
        signed(resize(unsigned(ft_q), 38))  when x"B" | x"F",
        resize(signed(ft_q), 38)            when x"C" | x"6",
        resize(sn_sin, 38)                  when x"D",
        resize(sn_cos, 38)                  when x"E",
        signed(resize(unsigned(registers_in(to_integer(unsigned(u_irf(2 downto 0))))), 38)) when others;

    u_out <= '1' when u_run = '1' and u_ph = "10" and u_xo = '1' else '0';

    p_eng : process(clk)
        variable v_op  : std_logic_vector(3 downto 0);
        variable v_arg : std_logic_vector(11 downto 0);
        variable v_last, v_cond, v_fetch : std_logic;
        variable v_v   : signed(37 downto 0);
        variable v_bo  : signed(38 downto 0);
        variable v_t   : signed(38 downto 0);
        variable v_sum : signed(37 downto 0);
    begin
        if rising_edge(clk) then
            v_op := u_irf(15 downto 12);
            v_arg := u_irf(11 downto 0);
            v_fetch := '0';
            l_kick <= '0';
            ft_q <= C_TN_FT(to_integer(ft_adr));
            if u_ph = "00" then
                rf_q <= rf_ram(to_integer(unsigned(u_ir(5 downto 0))));
            end if;
            if u_run = '1' and u_ph = "10" and u_xst = '1' then
                rf_ram(to_integer(unsigned(u_irf(5 downto 0)))) <= std_logic_vector(u_acc(31 downto 0));
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
                        -- the operand (inverted, carry-in set, for a subtraction; the first Booth multiple of B
                        -- for a multiply)
                        case v_op is
                            when OP_LD | OP_ADD | OP_SUB => v_v := resize(signed(rf_q), 38);
                            when OP_LDI | OP_ADDI | OP_SUBI => v_v := resize(signed(v_arg), 38);
                            when OP_IN   => v_v := w_in;
                            when OP_MUL  => v_v := shift_left(resize(signed(rf_q(17 downto 0)), 38), 18);
                            when others  => v_v := shift_left(resize(signed(v_arg), 38), 18);
                        end case;
                        u_b <= v_v;
                        v_bo := f_booth(v_v, u_acc(1), u_acc(0), '0');
                        case v_op is
                            when OP_LD | OP_ADD | OP_LDI | OP_ADDI | OP_IN => u_op <= v_v; u_cin <= '0';
                            when OP_SUB | OP_SUBI => u_op <= not v_v; u_cin <= '1';
                            when OP_MUL | OP_MULI => u_op <= v_bo(37 downto 0); u_cin <= v_bo(38);
                            when others => u_op <= (others => '0'); u_cin <= '0';
                        end case;
                        if v_op = OP_LD or v_op = OP_LDI or v_op = OP_IN then
                            u_acc <= (others => '0');
                        elsif v_op = OP_MUL or v_op = OP_MULI then
                            u_acc(37 downto 18) <= (others => '0');
                        end if;
                        if v_op = OP_MUL or v_op = OP_MULI then u_cnt <= to_unsigned(8, 5);
                        elsif v_op = OP_SHR then u_cnt <= unsigned(v_arg(4 downto 0)); end if;
                        -- IN ROMI / ROMSI read the table and step its address
                        if v_op = OP_IN and (v_arg(3 downto 0) = x"F" or v_arg(3 downto 0) = x"6") then
                            ft_adr <= ft_adr + 1;
                        end if;
                        case v_arg(2 downto 0) is
                            when "000"  => v_cond := u_acc(37);
                            when "001"  => v_cond := s_glide;
                            when "010"  => v_cond := s_mir;
                            when others => v_cond := s_il;
                        end case;
                        v_cond := v_cond xor v_arg(3);
                        if v_op = OP_SKIP and v_cond = '1' then u_pc <= u_pc + 2;
                        elsif v_op = OP_JMP then u_pc <= unsigned(v_arg(9 downto 0));
                        elsif v_op = OP_CALL then u_pc <= unsigned(v_arg(9 downto 0)); u_lr <= u_pc + 1;
                        elsif v_op = OP_SPC and v_arg(0) = '0' then u_pc <= u_lr;
                        else u_pc <= u_pc + 1; end if;
                        u_mul <= '0'; u_shr <= '0'; u_halt <= '0'; u_xo <= '0'; u_xst <= '0';
                        case v_op is
                            when OP_MUL | OP_MULI => u_mul <= '1';
                            when OP_SHR  => u_shr <= '1';
                            when OP_ST   => u_xst <= '1';
                            when OP_OUT  => u_xo <= '1';
                            when OP_SPC  => u_halt <= v_arg(0);
                            when others  => null;
                        end case;
                        u_ph <= "10";
                    when others =>                        -- execute (the adder sees registers only)
                        v_last := '1';
                        v_t := (u_acc & '1') + (u_op & u_cin);
                        v_sum := v_t(38 downto 1);
                        if u_mul = '1' then
                            -- the next Booth step's multiple, from the multiplier bits that shift down to 1, 0, -1
                            v_bo := f_booth(u_b, u_acc(3), u_acc(2), u_acc(1));
                            u_op <= v_bo(37 downto 0); u_cin <= v_bo(38);
                            u_acc <= v_sum(37) & v_sum(37) & v_sum(37 downto 2);
                            if u_cnt /= 0 then v_last := '0'; end if;
                            u_cnt <= u_cnt - 1;
                        elsif u_shr = '1' then
                            if u_cnt(4 downto 1) /= "0000" then
                                u_acc <= v_sum(37) & v_sum(37) & v_sum(37 downto 2); u_cnt <= u_cnt - 2;
                            else
                                u_acc <= v_sum(37) & v_sum(37 downto 1); u_cnt <= u_cnt - 1;
                            end if;
                            if u_cnt > 2 then v_last := '0'; end if;
                        else
                            u_acc <= v_sum;
                        end if;
                        if u_halt = '1' then u_run <= '0'; l_kick <= '1'; end if;
                        if v_last = '1' then
                            v_fetch := '1';
                            u_ph <= "00";
                        end if;
                end case;
            end if;
            if v_fetch = '1' then
                u_ir <= C_TN_UC(to_integer(u_pc));
            end if;

            -- the engine's output ports
            cr_we <= '0'; sn_go <= '0';
            if u_out = '1' then
                case to_integer(unsigned(u_irf(4 downto 0))) is
                    when 0  => ft_adr <= unsigned(u_acc(9 downto 0));
                    when 1  => sn_th <= unsigned(u_acc(15 downto 0)); sn_go <= '1';
                    when 2  => e_ea <= unsigned(u_acc(6 downto 0));
                    when 3  => cr_we <= '1'; cr_wa <= e_ea; cr_wd <= std_logic_vector(u_acc(31 downto 0)); e_ea <= e_ea + 1;
                    when 4  => c_kc  <= unsigned(u_acc(4 downto 0));
                    when 5  => c_vk1 <= unsigned(u_acc(7 downto 0));
                    when 6  => c_vk2 <= unsigned(u_acc(7 downto 0));
                    when 7  => c_vk3 <= unsigned(u_acc(7 downto 0));
                    when 8  => c_gg2 <= unsigned(u_acc(7 downto 0));
                    when 9  => c_gg3 <= unsigned(u_acc(7 downto 0));
                    when 10 => c_kx  <= not u_acc(37);
                    when others => null;
                end case;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- [C] SINE UNIT: OUT ANG -> fold both quadrants -> read the quarter sine at
    -- the index and the next one, interpolate with the 6-bit fraction, sign:
    -- the sine, then the cosine through the same registers.  sin ready 10
    -- clocks after OUT ANG, cos 15 (the microprogram reads sin 4 micro-ops on,
    -- cos 6).  Entry 256 (= 4096) is made here.
    ---------------------------------------------------------------------------
    p_sine : process(clk)
        variable v_q : unsigned(1 downto 0);
        variable v_p : unsigned(14 downto 0);
        variable v_t : unsigned(15 downto 0);
        variable v_s : unsigned(12 downto 0);
        variable v_i : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            sn_q  <= C_TN_SIN(to_integer(sn_adr));
            sn_qf <= sn_q;
            sn_ov1 <= sn_ov; sn_ov2 <= sn_ov1;
            if sn_go = '1' then
                sn_st <= to_unsigned(1, 4);
            elsif sn_st /= 0 then
                sn_st <= sn_st + 1;
            end if;
            case to_integer(sn_st) is
                when 1 =>                                  -- fold the sine and the cosine
                    v_q := sn_th(15 downto 14);
                    if v_q(0) = '1' then v_p := to_unsigned(16384, 15) - resize(sn_th(13 downto 0), 15);
                    else v_p := resize(sn_th(13 downto 0), 15); end if;
                    sn_is <= v_p(14 downto 6); sn_fs <= v_p(5 downto 0); sn_ns <= v_q(1);
                    v_t := sn_th + 16384;
                    v_q := v_t(15 downto 14);
                    if v_q(0) = '1' then v_p := to_unsigned(16384, 15) - resize(v_t(13 downto 0), 15);
                    else v_p := resize(v_t(13 downto 0), 15); end if;
                    sn_ic <= v_p(14 downto 6); sn_fc <= v_p(5 downto 0); sn_nc <= v_q(1);
                when 2 => sn_adr <= sn_is(7 downto 0); sn_ov <= sn_is(8);
                when 3 =>
                    v_i := sn_is + 1;
                    sn_adr <= v_i(7 downto 0); sn_ov <= v_i(8);
                when 7 => sn_adr <= sn_ic(7 downto 0); sn_ov <= sn_ic(8);
                when 8 =>
                    v_i := sn_ic + 1;
                    sn_adr <= v_i(7 downto 0); sn_ov <= v_i(8);
                when others => null;
            end case;
            -- the table value (fabric copy) two clocks after its address; entry 256 = 4096
            if sn_ov2 = '1' then v_s := to_unsigned(4096, 13); else v_s := unsigned(sn_qf(12 downto 0)); end if;
            case to_integer(sn_st) is
                when 5 | 10 => sn_t0 <= v_s;
                when 6 | 11 => sn_t1 <= v_s;
                when 7 | 12 => sn_d <= resize(sn_t1 - sn_t0, 5);
                when 8      => sn_p <= sn_d * sn_fs;
                when 13     => sn_p <= sn_d * sn_fc;
                when 9  =>
                    v_s := sn_t0 + resize(sn_p(10 downto 6), 13);
                    if sn_ns = '1' then sn_sin <= -signed(resize(v_s, 14)); else sn_sin <= signed(resize(v_s, 14)); end if;
                when 14 =>
                    v_s := sn_t0 + resize(sn_p(10 downto 6), 13);
                    if sn_nc = '1' then sn_cos <= -signed(resize(v_s, 14)); else sn_cos <= signed(resize(v_s, 14)); end if;
                when others => null;
            end case;
            if sn_st = 14 then sn_st <= (others => '0'); end if;
        end if;
    end process p_sine;

    ---------------------------------------------------------------------------
    -- [E] EDGES.  23 affine DDAs in pixel widths (Q12): each adds its own
    -- increment register on every clock of the line window (W + 8 steps), then
    -- once more with the line wrap parked in that register, so the adder has no
    -- input mux.  The words come from the edge table one per clock (stride 3:
    -- step, wrap, seed).  The kick (HALT) clears the accumulators and adds the
    -- field seeds the same way.
    ---------------------------------------------------------------------------
    p_load : process(clk)
        variable v_e : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- line window
            b_end <= '0';
            if s_st(4) = '1' then
                b_run <= '1'; b_cnt <= (others => '0');
            elsif b_run = '1' then
                b_cnt <= b_cnt + 1;
                if b_cnt = b_mm1 then b_run <= '0'; b_end <= '1'; end if;
            end if;
            b_e1 <= b_end; b_e2 <= b_e1; b_e3 <= b_e2;

            -- loader: 46 table reads (wrap or seed, step) per edge
            ca_rst <= l_kick;
            if l_kick = '1' then
                l_run <= '1'; l_t <= (others => '0'); l_seed <= '1';
            elsif b_end = '1' then
                l_run <= '1'; l_t <= (others => '0'); l_seed <= '0';
            elsif l_run = '1' then
                l_t <= l_t + 1;
                if l_t = 45 then l_run <= '0'; end if;
            end if;
            v_e := shift_left(resize(l_t(5 downto 1), 7), 1) + resize(l_t(5 downto 1), 7);
            if l_t(0) = '1' then cr_ra <= v_e;
            elsif l_seed = '1' then cr_ra <= v_e + 2;
            else cr_ra <= v_e + 1; end if;
            -- the word addressed now is in cr_q two clocks on and in its fabric copy three on: it goes into the
            -- edge's increment register then (ca_ld), and a wrap / seed word is added one clock after that
            -- (ca_en), on the same edge that puts the pixel step back
            l_v1 <= l_run; l_c1 <= l_t(5 downto 1); l_o1 <= l_t(0);
            l_v2 <= l_v1;  l_c2 <= l_c1;            l_o2 <= l_o1;
            l_v3 <= l_v2;  l_c3 <= l_c2;            l_o3 <= l_o2;
            cr_qf <= cr_q(25 downto 0);
            for k in 0 to NE - 1 loop
                if l_v2 = '1' and l_c2 = k then ca_ld(k) <= '1'; else ca_ld(k) <= '0'; end if;
                if l_v3 = '1' and l_o3 = '0' and l_c3 = k then ca_en(k) <= '1'; else ca_en(k) <= '0'; end if;
            end loop;

            -- the line's virtual y and block row (shared by the grid's write and read sides): stepped after
            -- the line's last pixel has left both; the DDA pixel's virtual x
            if s_zone = '1' then
                m_yv <= (others => '0'); m_rb0 <= '0'; m_base <= (others => '0');
            elsif b_e2 = '1' then
                m_yv <= m_yv + resize(s_ys, 19);
            elsif b_e3 = '1' then
                m_rb0 <= m_yv(13);
                if m_yv(13) /= m_rb0 then m_base <= m_base + 60; end if;
            end if;
            if s_zgo = '1' then m_bank <= not m_bank; end if;
            if b_run = '1' then m_xr <= m_xr + resize(s_xs, 12); else m_xr <= c_x0; end if;
        end if;
    end process p_load;

    g_edge : for k in 0 to NE - 1 generate
        p_edge : process(clk)
            variable v_hi : std_logic;
        begin
            if rising_edge(clk) then
                if ca_ld(k) = '1' then ca_inc(k) <= signed(cr_qf); end if;
                if ca_rst = '1' then
                    ca_acc(k) <= (others => '0');
                elsif b_run = '1' or ca_en(k) = '1' then
                    ca_acc(k) <= ca_acc(k) + ca_inc(k);
                end if;
                -- stage 1: the edge value in sixteenths of a pixel (eighths when interlaced), clamped to 0 .. 63
                if s_il = '1' then
                    if ca_acc(k)(24 downto 15) /= "0000000000" then v_hi := '1'; else v_hi := '0'; end if;
                else
                    if ca_acc(k)(24 downto 14) /= "00000000000" then v_hi := '1'; else v_hi := '0'; end if;
                end if;
                if ca_acc(k)(25) = '1' then x1_es(k) <= (others => '0');
                elsif v_hi = '1' then x1_es(k) <= (others => '1');
                elsif s_il = '1' then x1_es(k) <= unsigned(ca_acc(k)(14 downto 9));
                else x1_es(k) <= unsigned(ca_acc(k)(13 downto 8)); end if;
            end if;
        end process p_edge;
    end generate g_edge;

    -- stages 2 .. 5 per piece: the minimum of its edges (its inner distance), Bold / Fine coverage, its grey
    g_piece : for p in 0 to 6 generate
        p_piece : process(clk)
            variable v_mw : unsigned(5 downto 0);
            variable v_cl : unsigned(4 downto 0);
        begin
            if rising_edge(clk) then
                x2_a(p) <= f_min6(x1_es(C_E0(p)), x1_es(C_E0(p) + 1));
                if p < 5 then
                    x2_b(p) <= x1_es(C_E0(p) + 2);
                else
                    x2_b(p) <= f_min6(x1_es(C_E0(p) + 2), x1_es(C_E0(p) + 3));
                end if;
                x3_m(p) <= f_min6(x2_a(p), x2_b(p));
                -- 4: Bold = min(m, 16); Fine = Bold - clamp(m - w, 0, 16)
                if x3_m(p)(5 downto 4) /= "00" then x4_cb(p) <= to_unsigned(16, 5);
                else x4_cb(p) <= resize(x3_m(p)(3 downto 0), 5); end if;
                if x3_m(p) < c_w then v_cl := (others => '0');
                else
                    v_mw := x3_m(p) - c_w;
                    if v_mw(5 downto 4) /= "00" then v_cl := to_unsigned(16, 5); else v_cl := resize(v_mw(3 downto 0), 5); end if;
                end if;
                if x3_m(p)(5 downto 4) /= "00" then x4_cf(p) <= to_unsigned(16, 5) - v_cl;
                else x4_cf(p) <= resize(x3_m(p)(3 downto 0), 5) - v_cl; end if;
                -- 5: the selected coverage and its grey
                if s_fine = '1' then
                    x5_c(p) <= x4_cf(p); x5_t(p) <= resize(x4_cf(p), 8);       -- Fine: one outline grey (stage 9)
                else
                    x5_c(p) <= x4_cb(p); x5_t(p) <= f_cg(x4_cb(p), C_GREY(p));
                end if;
            end if;
        end process p_piece;
    end generate g_piece;

    ---------------------------------------------------------------------------
    -- [E] stages 6 .. 15: the films (brightest + a quarter of the rest), the
    -- camera, Grey / Ink / Fine, the mirror, luma
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_col : unsigned(5 downto 0);
        variable v_T   : unsigned(10 downto 0);
        variable v_c16, v_cc : unsigned(6 downto 0);
        variable v_t5  : unsigned(4 downto 0);
        variable v_v   : unsigned(4 downto 0);
        variable v_sum : unsigned(14 downto 0);
        variable v_sel : unsigned(7 downto 0);
        variable v_y   : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            -- camera, stage 1: the grid address of the pixel's block (the bank written last field)
            if m_xr(11 downto 8) = "1111" then v_col := to_unsigned(59, 6); else v_col := m_xr(11 downto 6); end if;
            gr_ra <= (not m_bank) & (m_base + resize(v_col, 11));
            -- 2: gr_q (the RAM's register); 3: its fabric copy; 4 .. 9: the level travels with the pixel
            gr_f <= gr_q;
            x_l(4) <= gr_f;
            for s in 5 to 9 loop x_l(s) <= x_l(s - 1); end loop;

            -- 6: maxima and sums of pairs (the grey terms, the Bold coverages, the selected coverages)
            x6_m(0) <= f_max8(x5_t(0), x5_t(1)); x6_m(1) <= f_max8(x5_t(2), x5_t(3));
            x6_m(2) <= f_max8(x5_t(4), x5_t(5)); x6_m(3) <= x5_t(6);
            x6_s(0) <= resize(x5_t(0), 9) + resize(x5_t(1), 9); x6_s(1) <= resize(x5_t(2), 9) + resize(x5_t(3), 9);
            x6_s(2) <= resize(x5_t(4), 9) + resize(x5_t(5), 9); x6_s(3) <= resize(x5_t(6), 9);
            -- (the Bold coverages are summed one stage early, straight from stage 4)
            x5_bp(0) <= resize(x4_cb(0), 6) + resize(x4_cb(1), 6); x5_bp(1) <= resize(x4_cb(2), 6) + resize(x4_cb(3), 6);
            x5_bp(2) <= resize(x4_cb(4), 6) + resize(x4_cb(5), 6); x5_bp(3) <= resize(x4_cb(6), 6);
            x6_bq0 <= resize(x5_bp(0), 7) + resize(x5_bp(1), 7); x6_bq1 <= resize(x5_bp(2), 7) + resize(x5_bp(3), 7);
            x6_c(0) <= resize(x5_c(0), 6) + resize(x5_c(1), 6); x6_c(1) <= resize(x5_c(2), 6) + resize(x5_c(3), 6);
            x6_c(2) <= resize(x5_c(4), 6) + resize(x5_c(5), 6); x6_c(3) <= resize(x5_c(6), 6);
            -- 7
            x7_m0 <= f_max8(x6_m(0), x6_m(1)); x7_m1 <= f_max8(x6_m(2), x6_m(3));
            x7_s0 <= resize(x6_s(0), 10) + resize(x6_s(1), 10); x7_s1 <= resize(x6_s(2), 10) + resize(x6_s(3), 10);
            x7_bs <= x6_bq0 + x6_bq1;
            x7_c0 <= resize(x6_c(0), 7) + resize(x6_c(1), 7); x7_c1 <= resize(x6_c(2), 7) + resize(x6_c(3), 7);
            -- 8
            x8_tm <= f_max8(x7_m0, x7_m1);
            x8_ts <= resize(x7_s0, 11) + resize(x7_s1, 11);
            x8_cb <= x7_bs;
            x8_cc <= x7_c0 + x7_c1;
            -- 9: the film T (clipped to 255), the Bold coverage capped and filmed, Ink's triangle, the level's tones
            v_T := resize(x8_tm, 11) + shift_right(x8_ts - resize(x8_tm, 11), 2);
            if s_fine = '1' then                                     -- Fine: the outline film x 16 (x 14/16 below)
                if v_T(10 downto 4) /= "0000000" then x9_t8 <= to_unsigned(255, 8); else x9_t8 <= v_T(3 downto 0) & "0000"; end if;
            elsif v_T(10 downto 8) /= "000" then x9_t8 <= to_unsigned(255, 8); else x9_t8 <= v_T(7 downto 0); end if;
            if x8_cb(6 downto 4) /= "000" then
                v_c16 := to_unsigned(16, 7);
            else
                v_c16 := resize(x8_cb(3 downto 0), 7);
            end if;
            x9_cb16 <= resize(v_c16, 5);
            x9_icb  <= resize(to_unsigned(16, 7) - v_c16, 5);
            x9_cbf  <= resize(v_c16 + shift_right(x8_cb - v_c16, 2), 6);
            if x8_cc(6 downto 4) /= "000" then v_cc := to_unsigned(16, 7); else v_cc := resize(x8_cc(3 downto 0), 7); end if;
            x9_c16 <= resize(v_cc, 5);
            if v_c16 > v_cc then x9_dcc <= resize(v_c16 - v_cc, 5); else x9_dcc <= (others => '0'); end if;
            v_t5 := x8_cc(4 downto 0);
            if v_t5 > 16 then x9_v <= (not v_t5) + 1; else x9_v <= v_t5; end if;      -- 32 - t
            if s_fine = '1' then x9_ikl <= to_unsigned(14, 5);
            elsif x_l(8) /= "00" then x9_ikl <= to_unsigned(16, 5) - c_kc; else x9_ikl <= to_unsigned(16, 5); end if;
            case to_integer(x_l(8)) is
                when 1      => x9_vk <= c_vk1; x9_gg <= (others => '0');
                when 2      => x9_vk <= c_vk2; x9_gg <= c_gg2;
                when 3      => x9_vk <= c_vk3; x9_gg <= c_gg3;
                when others => x9_vk <= (others => '0'); x9_gg <= (others => '0');
            end case;
            x9_fl <= x_l(8)(1) and c_kx;
            -- 10: the three products (Grey), Ink's tone
            x10_p1 <= resize(x9_t8 * x9_ikl, 12);
            if s_fine = '1' then
                x10_p2 <= resize(shift_right(x9_vk, 1) * resize(x9_dcc, 6), 14);
            else
                x10_p2 <= resize(x9_vk * x9_cbf, 14);
            end if;
            x10_p3 <= resize(x9_gg * x9_icb, 12);
            if x9_fl = '1' then v_v := to_unsigned(16, 5) - x9_v; else v_v := x9_v; end if;
            if v_v(4) = '1' then x10_ik <= to_unsigned(255, 8); else x10_ik <= v_v(3 downto 0) & "0000"; end if;
            -- 11: the tone, clipped
            if s_ink = '1' then
                v_sum := resize(x10_ik, 15);
            else
                v_sum := resize(shift_right(x10_p1, 4), 15) + resize(shift_right(x10_p2, 4), 15)
                         + resize(shift_right(x10_p3, 4), 15);
            end if;
            if v_sum(14 downto 8) /= "0000000" then x11_t <= to_unsigned(255, 8); else x11_t <= v_sum(7 downto 0); end if;
            -- 12: the tone; the mirror's line buffer is read (and, one clock on, written) from here
            x12_t <= x11_t;
            -- 13, 14: the direct path beside the buffer's register and fabric copy
            x13_t <= x12_t; x14_t <= x13_t;
            mb_f <= mb_q;
            -- 15: luma = 64 + tone * 55 / 16 (Mirror: the buffer, black on the field's first line)
            if s_mir = '1' then
                if m_l0 = '1' then v_sel := (others => '0'); else v_sel := mb_f; end if;
            else
                v_sel := x14_t;
            end if;
            v_y := shift_left(resize(v_sel, 14), 6) - shift_left(resize(v_sel, 14), 3) - resize(v_sel, 14);
            s_y15 <= to_unsigned(64, 10) + resize(shift_right(v_y, 4), 10);
        end if;
    end process p_pix;

    -- the mirror's addresses at stage 12: DDA pixel i reads W / 2 - i on the left half, i - W / 2 - 1 on the
    -- right (a counter that runs down to 0, holds one clock, then up); the right half is written one clock later
    p_mirror : process(clk)
    begin
        if rising_edge(clk) then
            r_run <= b_run & r_run(1 to 11);
            if s_st(16) = '1' then
                mb_rp <= c_half; mb_dn <= '1';
                m_l0 <= m_first; m_first <= '0';
            elsif r_run(12) = '1' then
                if mb_dn = '1' then
                    if mb_rp = 0 then mb_dn <= '0'; else mb_rp <= mb_rp - 1; end if;
                else
                    mb_rp <= mb_rp + 1;
                end if;
            end if;
            if s_zone = '1' then m_first <= '1'; end if;
            mb_we <= r_run(12) and not mb_dn and s_mir;
            mb_wa <= mb_rp; mb_wd <= x12_t;
        end if;
    end process p_mirror;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> box of 32 px (16 px off the
    -- 1920-wide rasters), edge-replicated and reloaded at every line start ->
    -- vertical one-pole (1/16 per line; 1/8 at 1080i and progressive SD, 1/4 at
    -- 480i / 576i) in the line store at quarter horizontal resolution -> the
    -- block grid: on the lower half of every block row each stored column
    -- overwrites its block's cell with its camera level, so the cell ends as the
    -- block's last stored column and line.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_dl : signed(16 downto 0);
        variable v_n : signed(17 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then   -- both phases restart from this field's drift
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);    -- ot * 96
            end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid;
            -- two registers on: s_tv is the sample of column s_col
            s_v1 <= s_sv; s_v2 <= s_v1; s_v2d <= s_v2;
            s_t0 <= s_sl; s_tv <= s_t0;
            if s_v2 = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 32 (16) samples; the samples before the line start count as the first one
            if s_v2 = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            elsif s_b32 = '1' then bx_wp <= (others => '0'); bx_rp <= to_unsigned(226, 8);
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(242, 8); end if;
            bx_f <= bx_q;
            if s_bcnt(5) = '0' and (s_b32 = '1' or s_bcnt(4) = '0') then v_tap := s_sfirst; else v_tap := bx_f; end if;
            s_df <= signed(resize(s_tv, 9)) - signed(resize(v_tap, 9));
            s_tvd <= s_tv; s_v3 <= s_v2; s_c2 <= s_col;
            s_ld <= s_v2 and not s_v2d;
            if s_v2 = '1' and s_v2d = '0' then
                s_sfirst <= s_tv;
                s_bcnt <= to_unsigned(1, 6);
            elsif s_v2 = '1' then
                if s_bcnt(5) = '0' then s_bcnt <= s_bcnt + 1; end if;
            end if;
            if s_ld = '1' then
                if s_b32 = '1' then s_bsum <= shift_left(resize(s_tvd, 14), 5);
                else s_bsum <= shift_left(resize(s_tvd, 14), 4); end if;
            elsif s_v3 = '1' then
                s_bsum <= unsigned(signed(s_bsum) + resize(s_df, 14));
            end if;
            s_v4 <= s_v3; s_bcol <= s_c2;

            -- the first line of a field loads the one-pole instead of stepping it
            if s_zone = '1' then
                s_fl <= '1'; s_seen <= '0';
            elsif s_ld = '1' then
                if s_seen = '1' then s_fl <= '0'; end if;
                s_seen <= '1';
            end if;

            -- vertical one-pole on columns 3, 7, 11 ...: q += (box - q) >> 4 (3, 2), Q8.8
            if s_b32 = '1' then s_eb <= s_bsum(12 downto 0) & "000"; else s_eb <= s_bsum(11 downto 0) & "0000"; end if;
            s_eq <= lq_q1;
            s_ev <= s_v4 and s_bcol(1) and s_bcol(0); s_ecol <= s_bcol; s_efl <= s_fl;
            s_dd <= signed(resize(s_eb, 17)) - signed(resize(s_eq, 17));
            if s_efl = '1' then s_fq <= s_eb; else s_fq <= s_eq; end if;
            s_fv <= s_ev; s_fcol <= s_ecol; s_ffl <= s_efl;
            if s_ffl = '1' then v_dl := (others => '0');
            elsif c_vsh = "00" then v_dl := shift_right(s_dd, 4);
            elsif c_vsh = "01" then v_dl := shift_right(s_dd, 3);
            else v_dl := shift_right(s_dd, 2); end if;
            v_n := signed(resize(s_fq, 18)) + resize(v_dl, 18);
            lq_wd <= unsigned(v_n(15 downto 0));
            lq_we <= s_fv; lq_wa <= s_fcol(10 downto 2);

            -- the block grid's write side: m_xw = the stored column's virtual x (h)
            if s_ld = '1' then
                m_xw <= c_x0 + resize(s_xs, 12) + shift_left(resize(s_xs, 12), 1);
            elsif lq_we = '1' then
                m_xw <= m_xw + shift_left(resize(s_xs, 12), 2);
            end if;
            gr_we <= lq_we and m_yv(12);
            gr_wa <= m_bank & (m_base + resize(m_xw(11 downto 6), 11));
            gr_wd <= f_level(lq_wd(15 downto 8));
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: luma [1 2 1]/4, luma 13/16 toward black, blanking
    -- gate.  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_yd1 <= s_y15; s_yd2 <= s_yd1;
            s_ya <= resize(s_y15, 12) + resize(s_yd2, 12) + 2;
            s_yb <= s_yd1;
            v_y := s_ya + shift_left(resize(s_yb, 12), 1);
            s_y26 <= v_y(11 downto 2);
            if s_y26 > 64 then v_dl := s_y26 - 64; else v_dl := (others => '0'); end if;
            s_y27 <= s_y26 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y28 <= s_y27;
            else
                s_y28 <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture tans;
