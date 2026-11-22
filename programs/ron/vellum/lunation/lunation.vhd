-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Lunation v0.1.0 -- Vellum.  The lunar-phase calendar: a wall chart of moons in rows, each a day older than
-- the last (Galileo's wash drawings of the phases).  Luma only: U = V = 512 on every pixel.
--
--   cell pitch P, disc radius R = d P; moon (i, j) has phase phi = phi0 + s n(i, j)
--     Calendar: square grid, n = i + 7 j          Brick: odd rows offset P/2, n = 2 i + (j & 1) + 2 j
--   in a cell: u, v from the cell centre (horizontal px), w = sqrt(R^2 - v^2) (per line, serial square root)
--     limb       cov = clamp(1/2 + (w^2 - u^2) / 2R)        (squares from a per-field table, w^2 per line)
--     terminator u = c w, c = cos(2 pi phi) (+ camera), lit u > c w waxing, -u > c w waning;
--                cov = clamp(1/2 + kt (s u - c w)), kt = w / sqrt(w^2 + c^2 v^2) from a ROM by (w / R, |c|)
--     Grey: ground black, dark part 3/16 (earthshine), lit white.  Ink: lit part only.
--     Fine: the limb and the terminator as lines (R / 14 wide, at least 2 virtual px); Grey adds a lit film.
--
--   K1  X       pan across          K2  Y      pan up / down          K3  Scale   pitch 640 .. 137 virtual px
--   K4  Disc    radius 0.14 .. 0.5 of the pitch                        K5  Drift   phi0 speed (Hypnos law)
--   K6  Camera  box-filtered luma pulls each moon's c toward the luma's own phase (lit area = luma), inside the
--               band that keeps new and full moons as they are (so nothing flips when a moon passes new / full)
--   S7  Glide   Off / On     S8  Form  Calendar / Brick     S9  Tone  Grey / Ink     S10  Weight  Bold / Fine
--   S11 Source  Video / Osc
--   P12 SPAN    the phase step per moon: 0 = all alike, 34 % = one lunation in 28 moons, 100 % = 27/56 (full and
--               new alternate with a slow standing beat)
--
-- The chassis is the Kinetic riley shell via Vellum rosette:
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         +-1-count holds, glides, switches, the Drift speed (Hypnos)
--   [C] ENGINES                          per-field sequencer (one serial multiplier, one serial divider, the
--                                        cell reductions), squares-table fill, per-line engine (v, w by serial
--                                        square root, the limb's w^2 / 2R, w / R), per-cell engine (phase ->
--                                        cos ROM, camera, c w by a radix-4 serial multiplier, kt ROM)
--   [D] SOURCE FIELD                     luma (or the Osc lattice) -> box -> vertical IIR line store (+ a copy
--                                        the cell engine reads at each cell centre, one line old)
--   [E] MOONS                            u DDA with cell crossings, squares RAM, coverages, tone
--   [F] OUTPUT CONDITIONING              luma [1 2 1]/4, 13/16 toward black, blanking gate
--
-- Stage numbering: stage 1 is the generated avid's own clock; a register "at stage S" holds pixel n at
-- T + S + n (T = the g_start clock).  s_st(k) is high during T + k + 1, so a per-line constant read by a
-- stage-S register is loaded on s_st(S - 3).  s_gav_sr(S - 2) flags pixels 0 .. W - 1 at stage S.  The
-- luma is at stage 15, the output register at stage C_LAT - 3.  One latency (24) in every mode.
--
-- Line timing: at the end of each generated line (t_go) the next line's values become current, the cell
-- engine computes that line's first two cells (2 x 25 clocks, inside the 64-clock hblank of the image
-- tester and the 138+ of the hardware) and the per-line engine starts on the line after it.  Inside a line
-- the cell engine computes the next cell at every cell crossing (a cell is at least 68 pixels).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.lunation_pkg.all;
use work.all;

architecture lunation of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

    -- TOML defaults: the glides and holds start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 573;
    constant C_DEF_K4  : integer := 870;
    constant C_DEF_K5  : integer := 680;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 352;

    constant C_NSLOT  : integer := 12;     -- serial-product slots 0 .. 12 (10, 11: the reductions run)
    constant C_SQ_ZC  : integer := 320;    -- squares fill start (after slot 9 has KA)
    constant C_LI_ZC  : integer := 390;    -- line-0 init (after slot 12)

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
    -- coverage clamp(8 + x, 0, 16) of a signed x
    function f_cov(x : signed) return unsigned is
        variable v_z, v_o : std_logic;
        variable v_x : signed(x'length - 1 downto 0);
    begin
        v_x := x;
        v_z := '1'; v_o := '1';
        for i in 4 to x'length - 1 loop
            if v_x(i) = '1' then v_z := '0'; else v_o := '0'; end if;
        end loop;
        if v_z = '1' then                                  -- 0 .. 15
            if v_x(3) = '1' then return to_unsigned(16, 5); else return "01" & unsigned(v_x(2 downto 0)); end if;
        elsif v_o = '1' then                               -- -16 .. -1
            if v_x(3) = '0' then return to_unsigned(0, 5); else return "00" & unsigned(v_x(2 downto 0)); end if;
        elsif v_x(x'length - 1) = '1' then
            return to_unsigned(0, 5);
        else
            return to_unsigned(16, 5);
        end if;
    end function;
    function f_min5(a, b : unsigned(4 downto 0)) return unsigned is
    begin
        if a < b then return a; else return b; end if;
    end function;

    attribute no_rw_check : boolean;

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
    signal s_st       : std_logic_vector(0 to 9) := (others => '0');

    -- [A] per-field mode constants
    signal c_kdx  : unsigned(15 downto 0) := to_unsigned(35139, 16);    -- 137.26 v-px in Q8 px
    signal c_xo   : signed(19 downto 0) := to_signed(-245760, 20);      -- pixel 0's x - 960 v-px, Q8 px
    signal c_kx   : unsigned(15 downto 0) := to_unsigned(61440, 16);    -- pan gain (x 2^-13)
    signal c_yo   : signed(19 downto 0) := to_signed(-138240, 20);      -- line 0's y - 540 v-px
    signal c_ky   : unsigned(15 downto 0) := to_unsigned(34560, 16);
    signal c_dyp  : unsigned(9 downto 0) := to_unsigned(256, 10);       -- line step, Q8 px
    signal c_il   : std_logic := '0';                                   -- interlaced: AA unit 1.5 px
    signal c_hmin : unsigned(4 downto 0) := to_unsigned(16, 5);

    -- [B] knobs: +-1-count holds, glide accumulators (Q10.6), switches, the Drift speed glide (Q6)
    signal s_h1 : unsigned(9 downto 0) := to_unsigned(C_DEF_K1, 10);
    signal s_h2 : unsigned(9 downto 0) := to_unsigned(C_DEF_K2, 10);
    signal s_h3 : unsigned(9 downto 0) := to_unsigned(C_DEF_K3, 10);
    signal s_h4 : unsigned(9 downto 0) := to_unsigned(C_DEF_K4, 10);
    signal s_h6 : unsigned(9 downto 0) := to_unsigned(C_DEF_K6, 10);
    signal s_hp : unsigned(9 downto 0) := to_unsigned(C_DEF_P12, 10);
    signal s_a1 : unsigned(15 downto 0) := to_unsigned(C_DEF_K1 * 64, 16);
    signal s_a2 : unsigned(15 downto 0) := to_unsigned(C_DEF_K2 * 64, 16);
    signal s_a3 : unsigned(15 downto 0) := to_unsigned(C_DEF_K3 * 64, 16);
    signal s_a4 : unsigned(15 downto 0) := to_unsigned(C_DEF_K4 * 64, 16);
    signal s_a6 : unsigned(15 downto 0) := to_unsigned(C_DEF_K6 * 64, 16);
    signal s_ap : unsigned(15 downto 0) := to_unsigned(C_DEF_P12 * 64, 16);
    signal s_gm : unsigned(15 downto 0) := (others => '0');
    signal s_gr, s_gh, s_gt : unsigned(9 downto 0) := (others => '0');
    signal s_hd : signed(10 downto 0) := (others => '0');
    signal s_gd : signed(16 downto 0) := (others => '0');
    signal z_gp : unsigned(2 downto 0) := (others => '0');
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_glide : std_logic := '1';
    signal s_brick, s_ink, s_fine, s_src : std_logic := '0';
    signal s_k5  : unsigned(9 downto 0) := to_unsigned(C_DEF_K5, 10);
    signal s_vtg : signed(16 downto 0) := (others => '0');
    signal s_vgs : signed(16 downto 0) := to_signed((C_DEF_K5 - 512) * 64, 17);
    signal s_vneg, s_vz : std_logic := '0';
    signal s_av  : unsigned(9 downto 0) := (others => '0');
    signal s_u   : unsigned(9 downto 0) := (others => '0');
    signal s_ls  : unsigned(9 downto 0) := (others => '0');
    signal s_scnt : unsigned(2 downto 0) := (others => '0');
    signal c_spd : unsigned(19 downto 0) := (others => '0');       -- drift step per field, Q20 lunations
    signal c_sneg : std_logic := '0';
    signal c_phi : unsigned(19 downto 0) := (others => '0');       -- phi0, Q20 lunations (wraps exactly)
    signal s_g   : unsigned(9 downto 0) := to_unsigned(1023, 10);  -- camera gain (K6 glided)

    -- [C] EXP ROM
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');

    -- [C] serial multiplier (per field and per line), slot counter and scratch
    signal f_a, z_fa : unsigned(21 downto 0) := (others => '0');       -- multiplicand (static)
    signal f_hi   : unsigned(22 downto 0) := (others => '0');          -- product high part
    signal f_b, z_fb : unsigned(19 downto 0) := (others => '0');       -- multiplier -> product low part
    signal f_n    : unsigned(4 downto 0) := (others => '0');
    signal f_prod : unsigned(42 downto 0);
    signal z_ld   : std_logic := '0';
    signal z_ph   : unsigned(4 downto 0) := (others => '0');
    signal z_slot : unsigned(3 downto 0) := (others => '0');
    signal z_run  : std_logic := '0';
    signal z_oct  : unsigned(9 downto 0) := (others => '0');
    signal z_d3, z_d7 : unsigned(15 downto 0) := (others => '0');
    signal c_q32  : signed(23 downto 0) := (others => '0');            -- 32 P - Ph
    signal c_q31  : signed(23 downto 0) := (others => '0');            -- 31 P (= 32 P - 2 Ph)
    signal c_xlim : signed(20 downto 0) := (others => '0');            -- -256 (W - 1): the camera column's right clamp
    signal z_d16  : unsigned(15 downto 0) := (others => '0');
    signal z_o5   : unsigned(10 downto 0) := (others => '0');
    signal z_X    : unsigned(20 downto 0) := (others => '0');
    signal z_s16  : unsigned(15 downto 0) := (others => '0');
    signal z_oxn  : std_logic := '0';
    signal z_oxm  : unsigned(14 downto 0) := (others => '0');
    signal z_h    : unsigned(7 downto 0) := (others => '0');
    signal z_n0   : unsigned(15 downto 0) := (others => '0');

    -- [C] per-field constants
    signal c_P    : unsigned(17 downto 0) := to_unsigned(46080, 18);   -- pitch, Q8 px
    signal c_Ph   : unsigned(16 downto 0) := to_unsigned(23040, 17);   -- P / 2
    signal c_mP   : signed(19 downto 0) := (others => '0');            -- -P
    signal c_th   : signed(19 downto 0) := (others => '0');            -- Ph - 256 (crossing threshold)
    signal c_k2   : signed(19 downto 0) := (others => '0');            -- 256 - P (the crossing step)
    signal c_R    : unsigned(16 downto 0) := (others => '0');          -- radius, Q8 px
    signal c_R4   : unsigned(12 downto 0) := to_unsigned(1280, 13);    -- radius, Q4 px
    signal c_R4s  : unsigned(25 downto 0) := (others => '0');          -- R4^2
    signal c_KA   : unsigned(19 downto 0) := (others => '0');          -- 1 / (2 R4 a), Q28
    signal c_Kr   : unsigned(17 downto 0) := (others => '0');          -- 32 / R4, Q20
    signal c_Sx   : unsigned(15 downto 0) := (others => '0');          -- phase step per cell, Q16
    signal c_D1, c_D2 : unsigned(15 downto 0) := (others => '0');      -- row steps: to an odd row, to an even row
    signal c_u0e, c_u0o : signed(19 downto 0) := (others => '0');      -- pixel 0's u: even rows, odd rows
    signal c_v0   : signed(19 downto 0) := (others => '0');            -- line 0's v
    signal c_par0 : std_logic := '0';
    signal c_phf  : unsigned(15 downto 0) := (others => '0');          -- line 0's first-cell phase
    signal c_h16  : unsigned(7 downto 0) := (others => '0');           -- Fine: half line width, 1/16 AA px

    -- [C] serial divider: inv = floor(2^32 / R4)
    signal dv_rem : unsigned(13 downto 0) := (others => '0');
    signal dv_q   : unsigned(25 downto 0) := (others => '0');
    signal dv_n   : unsigned(5 downto 0) := (others => '0');

    -- [C] reductions of pixel 0 / line 0 into their cells (one shared unit, x then y)
    signal rd_x, rd_y : signed(20 downto 0) := (others => '0');
    signal rd_a   : signed(23 downto 0) := (others => '0');
    signal rd_n   : signed(7 downto 0) := (others => '0');
    signal rd_ph  : unsigned(1 downto 0) := (others => '0');
    signal rd_xr, rd_yr : signed(19 downto 0) := (others => '0');
    signal rd_i, rd_j : signed(7 downto 0) := (others => '0');
    signal rd_dl  : std_logic := '0';                                  -- odd rows start one cell left

    -- [C] squares fill
    signal sf_run : std_logic := '0';
    signal sf_i   : unsigned(9 downto 0) := (others => '0');
    signal sf_S   : unsigned(39 downto 0) := (others => '0');
    signal sf_D   : unsigned(30 downto 0) := (others => '0');
    signal sq_we  : std_logic := '0';
    signal sq_wa  : unsigned(9 downto 0) := (others => '0');
    signal sq_wd  : unsigned(11 downto 0) := (others => '0');

    -- [C] per-line engine (the running line state = the next line), square root, current line
    signal l_run, l_init : std_logic := '0';
    signal l_ph   : unsigned(6 downto 0) := (others => '0');
    signal l_v    : signed(19 downto 0) := (others => '0');
    signal l_par  : std_logic := '0';
    signal l_phr  : unsigned(15 downto 0) := (others => '0');
    signal l_ur   : signed(19 downto 0) := (others => '0');
    signal l_av4  : unsigned(13 downto 0) := (others => '0');
    signal l_w2   : signed(26 downto 0) := (others => '0');
    signal l_al   : signed(13 downto 0) := (others => '0');
    signal l_rho  : unsigned(4 downto 0) := (others => '0');
    signal q_rad  : unsigned(25 downto 0) := (others => '0');
    signal q_rem  : unsigned(15 downto 0) := (others => '0');
    signal q_root : unsigned(12 downto 0) := (others => '0');
    signal q_n    : unsigned(3 downto 0) := (others => '0');
    signal t_zf, t_go, g_avd : std_logic := '0';
    signal cur_ur : signed(19 downto 0) := (others => '0');
    signal cur_w  : unsigned(12 downto 0) := (others => '0');
    signal cur_al : signed(13 downto 0) := (others => '0');
    signal cur_rho : unsigned(4 downto 0) := (others => '0');
    signal cur_phr : unsigned(15 downto 0) := (others => '0');

    -- [C] per-cell engine
    signal e_run  : std_logic := '0';
    signal e_st   : unsigned(4 downto 0) := (others => '0');
    signal e_ls   : unsigned(1 downto 0) := (others => '0');            -- line start: 1 = cell 0, 2 = cell 1
    signal e_dcs  : std_logic := '0';                                   -- this run's result goes to the current cell
    signal e_tp   : unsigned(15 downto 0) := (others => '0');
    signal e_xn   : signed(20 downto 0) := (others => '0');            -- minus the target cell's centre (Q8 px)
    signal e_col  : unsigned(10 downto 0) := (others => '0');
    signal e_sin_a : unsigned(9 downto 0) := (others => '0');
    signal e_sin_q, e_sin_f : unsigned(10 downto 0) := (others => '0');
    signal e_qs, e_wax : std_logic := '0';
    signal e_cam_a : unsigned(9 downto 0) := (others => '0');
    signal e_cam_q, e_cam_f : unsigned(7 downto 0) := (others => '0');
    signal e_c    : signed(12 downto 0) := (others => '0');
    signal e_ts1  : unsigned(12 downto 0) := (others => '0');
    signal e_ts2  : unsigned(7 downto 0) := (others => '0');
    signal e_t    : signed(13 downto 0) := (others => '0');
    signal e_tsum : unsigned(13 downto 0) := (others => '0');
    signal e_d    : signed(13 downto 0) := (others => '0');
    signal e_ad   : unsigned(12 downto 0) := (others => '0');
    signal e_sd   : std_logic := '0';
    signal e_lo, e_hi : signed(13 downto 0) := (others => '0');
    signal e_c1   : signed(14 downto 0) := (others => '0');
    signal e_ce   : signed(13 downto 0) := (others => '0');
    signal e_sce  : std_logic := '0';
    signal e_kt_a : unsigned(9 downto 0) := (others => '0');
    signal e_kt_q, e_kt_f : unsigned(5 downto 0) := (others => '0');
    signal m_a, m_hi : unsigned(12 downto 0) := (others => '0');       -- radix-4, right-shifting
    signal m_a3   : unsigned(14 downto 0) := (others => '0');
    signal m_b    : unsigned(11 downto 0) := (others => '0');
    signal e_ov   : std_logic := '0';
    signal e_os   : std_logic := '0';
    signal e_ocw  : unsigned(17 downto 0) := (others => '0');          -- |c w|, Q8 px
    signal e_osc  : std_logic := '0';                                  -- sign of c
    signal e_okt  : unsigned(5 downto 0) := (others => '0');
    signal e_odcs : std_logic := '0';

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_p, s_pfirst : unsigned(7 downto 0) := (others => '0');
    signal s_avd : std_logic_vector(1 to 5) := (others => '0');
    signal s_col, s_pcol, s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal c_b64 : std_logic := '1';
    signal c_brp : unsigned(7 downto 0) := to_unsigned(193, 8);
    signal c_iir : unsigned(2 downto 0) := to_unsigned(5, 3);
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(6 downto 0) := (others => '0');
    signal s_bsum : unsigned(13 downto 0) := (others => '0');
    signal s_ev, s_fl : std_logic := '0';
    signal s_eb, s_eq : unsigned(11 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(11 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_wd : unsigned(11 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    type t_cm is array (0 to 1023) of unsigned(7 downto 0);
    signal cm_ram : t_cm := (others => (others => '0'));
    attribute no_rw_check of cm_ram : signal is true;

    -- [E] squares table and the pixel path
    type t_sq is array (0 to 1023) of unsigned(11 downto 0);
    signal sq_ram : t_sq := (others => (others => '0'));
    attribute no_rw_check of sq_ram : signal is true;
    signal sq_q, sq_f : unsigned(11 downto 0) := (others => '0');
    signal cs_s, nx_s, cs_sc, nx_sc : std_logic := '0';
    signal cs_cw, nx_cw : unsigned(17 downto 0) := (others => '0');
    signal cs_kt, nx_kt : unsigned(5 downto 0) := (others => '0');
    signal u3     : signed(19 downto 0) := (others => '0');
    signal x3     : std_logic := '0';                                  -- a cell crossing at stage 3
    signal au4    : unsigned(18 downto 0) := (others => '0');
    signal ex4    : signed(20 downto 0) := (others => '0');            -- u -+ |c w|: s (s u - c w)
    signal s4     : std_logic := '0';
    signal kt4, kt5 : unsigned(5 downto 0) := (others => '0');
    signal idx5   : unsigned(9 downto 0) := (others => '0');
    signal mg5    : unsigned(7 downto 0) := (others => '0');           -- |dt| in 1/16 px (sat 255)
    signal ng5, ng6, ng7, ng8 : std_logic := '0';                      -- dt < 0
    signal pr6    : unsigned(13 downto 0) := (others => '0');
    signal mk7, mk8 : unsigned(7 downto 0) := (others => '0');         -- |dt| kt
    signal al_px  : signed(13 downto 0) := (others => '0');
    signal dl8    : signed(14 downto 0) := (others => '0');
    signal adl9   : unsigned(11 downto 0) := (others => '0');
    signal ak9    : unsigned(7 downto 0) := (others => '0');
    signal cl9, cl10 : unsigned(4 downto 0) := (others => '0');
    signal ct9    : unsigned(4 downto 0) := (others => '0');
    signal rg10, mx11 : unsigned(4 downto 0) := (others => '0');
    signal tl10   : unsigned(4 downto 0) := (others => '0');
    signal a11, c11 : unsigned(6 downto 0) := (others => '0');
    signal b11    : unsigned(7 downto 0) := (others => '0');
    signal li10, li11 : unsigned(4 downto 0) := (others => '0');
    signal x12    : unsigned(8 downto 0) := (others => '0');
    signal y13, y14, y15 : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- [F] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_yd : t_u10(1 to 3) := (others => to_unsigned(64, 10));
    signal s_ya : unsigned(11 downto 0) := (others => '0');
    signal s_yb : unsigned(9 downto 0) := (others => '0');
    signal s_y1, s_y2, s_y3 : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q  <= C_LN_EXP(to_integer(r_ex_a));
            r_exf   <= r_ex_q;
            e_sin_q <= C_LN_SIN(to_integer(e_sin_a));
            e_kt_q  <= C_LN_KT(to_integer(e_kt_a));
            y13     <= C_LN_Y(to_integer(x12));
        end if;
    end process p_rom;

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

    -- its 8-bit copy, read by the cell engine at the cell centres (always ahead of the write: one line old)
    p_cmram : process(clk)
    begin
        if rising_edge(clk) then
            e_cam_q <= cm_ram(to_integer(e_cam_a));
            if lb_we = '1' then cm_ram(to_integer(lb_wa)) <= lb_wd(11 downto 4); end if;
        end if;
    end process p_cmram;

    -- squares table: written in the vblank zone only, read by the pixel path
    p_sqram : process(clk)
    begin
        if rising_edge(clk) then
            sq_q <= sq_ram(to_integer(idx5));
            if sq_we = '1' then sq_ram(to_integer(sq_wa)) <= sq_wd; end if;
        end if;
    end process p_sqram;

    ---------------------------------------------------------------------------
    -- [A] TIMING + [B] CONTROLS + [C] per-field sequencer, divider, reductions,
    -- per-line engine (one small operation per clock; quasi-static but timed
    -- at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_st  : signed(16 downto 0);
        variable v_v   : signed(10 downto 0);
        variable v_16  : unsigned(16 downto 0);
        variable v_r   : unsigned(14 downto 0);
        variable v_ph  : unsigned(18 downto 0);
        variable v_qr  : unsigned(15 downto 0);
        variable v_qt  : unsigned(15 downto 0);
        variable v_nv  : signed(19 downto 0);
        variable v_al  : unsigned(16 downto 0);
        variable v_s   : unsigned(23 downto 0);
        variable v_ni, v_nj, v_nn : signed(10 downto 0);
        variable v_d3, v_d7 : unsigned(15 downto 0);
        variable v_op  : signed(19 downto 0);
        variable v_ri, v_rk : signed(23 downto 0);
        variable v_rgo : std_logic;
        variable v_rr  : signed(19 downto 0);
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

            -- [A][B] per-field latches and mode constants
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_brick <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_k5    <= unsigned(registers_in(4));
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4 =>
                        s_oxs <= to_unsigned(204, 9); c_b64 <= '0'; c_brp <= to_unsigned(225, 8);
                        c_kdx <= to_unsigned(17570, 16); c_xo <= to_signed(-92160, 20); c_kx <= to_unsigned(30720, 16);
                        c_yo <= to_signed(-69120, 20); c_ky <= to_unsigned(17280, 16);
                        c_dyp <= s_ys(10 downto 1);
                        if s_ys >= 960 then c_il <= '1'; c_hmin <= to_unsigned(5, 5);
                        else c_il <= '0'; c_hmin <= to_unsigned(8, 5); end if;
                    when 3 =>
                        s_oxs <= to_unsigned(153, 9); c_b64 <= '0'; c_brp <= to_unsigned(225, 8);
                        c_kdx <= to_unsigned(23426, 16); c_xo <= to_signed(-163840, 20); c_kx <= to_unsigned(40960, 16);
                        c_yo <= to_signed(-92160, 20); c_ky <= to_unsigned(23040, 16);
                        c_dyp <= to_unsigned(256, 10);
                        c_il <= '0'; c_hmin <= to_unsigned(11, 5);
                    when others =>
                        s_oxs <= to_unsigned(102, 9); c_b64 <= '1'; c_brp <= to_unsigned(193, 8);
                        c_kdx <= to_unsigned(35139, 16); c_xo <= to_signed(-245760, 20); c_kx <= to_unsigned(61440, 16);
                        c_yo <= to_signed(-138240, 20); c_ky <= to_unsigned(34560, 16);
                        c_dyp <= s_ys(9 downto 0);
                        if s_ys = 512 then c_il <= '1'; c_hmin <= to_unsigned(10, 5);
                        else c_il <= '0'; c_hmin <= to_unsigned(16, 5); end if;
                end case;
                -- the vertical IIR's shift: about 32 .. 48 virtual lines in every mode
                if s_ys >= 960 then c_iir <= to_unsigned(3, 3);
                elsif s_ys >= 480 then c_iir <= to_unsigned(4, 3);
                else c_iir <= to_unsigned(5, 3); end if;
            end if;
            g_wm1 <= s_w - 1;
            case to_integer(unsigned(s_tid)) is                              -- (ys * 121) >> 8
                when 0          => s_oys <= to_unsigned(537, 10);
                when 4          => s_oys <= to_unsigned(272, 10);
                when 8          => s_oys <= to_unsigned(453, 10);
                when 12         => s_oys <= to_unsigned(226, 10);
                when 5 | 6 | 14 => s_oys <= to_unsigned(181, 10);
                when 1 | 2 | 10 => s_oys <= to_unsigned(242, 10);
                when others     => s_oys <= to_unsigned(121, 10);
            end case;

            -- [B] shared hold + glide, six clocks per channel (X, Y, Scale, Disc, Camera, Span):
            --   hold: the knob is re-read only when it has moved more than one count (pot jitter)
            --   glide On: 1/16 of the gap per field, never slower than one count, landing exactly
            --   glide Off: the accumulator takes the held value of THIS field start (no easing, no lag)
            if s_zgo = '1' then
                z_gp <= "000"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                z_gp <= z_gp + 1;
                case to_integer(z_gp) is
                    when 0 =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_a1; s_gh <= s_h1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_a2; s_gh <= s_h2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_a3; s_gh <= s_h3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_a4; s_gh <= s_h4; s_gr <= unsigned(registers_in(3));
                            when 4      => s_gm <= s_a6; s_gh <= s_h6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_ap; s_gh <= s_hp; s_gr <= unsigned(registers_in(7));
                        end case;
                    when 1 =>
                        s_hd <= signed(resize(s_gr, 11)) - signed(resize(s_gh, 11));
                    when 2 =>
                        if s_hd > 1 or s_hd < -1 then s_gt <= s_gr; else s_gt <= s_gh; end if;
                    when 3 =>
                        s_gd <= signed(resize(s_gt & "000000", 17)) - signed(resize(s_gm, 17));
                        case to_integer(z_gc) is
                            when 0      => s_h1 <= s_gt;
                            when 1      => s_h2 <= s_gt;
                            when 2      => s_h3 <= s_gt;
                            when 3      => s_h4 <= s_gt;
                            when 4      => s_h6 <= s_gt;
                            when others => s_hp <= s_gt;
                        end case;
                    when 4 =>
                        if s_glide = '0' or (s_gd <= 64 and s_gd >= -64) then
                            null;                                          -- the whole gap
                        elsif s_gd < 1024 and s_gd > -1024 then
                            if s_gd(16) = '1' then s_gd <= to_signed(-64, 17); else s_gd <= to_signed(64, 17); end if;
                        else
                            s_gd <= shift_right(s_gd, 4);
                        end if;
                    when others =>
                        v_st := signed(resize(s_gm, 17)) + s_gd;
                        case to_integer(z_gc) is
                            when 0      => s_a1 <= unsigned(v_st(15 downto 0));
                            when 1      => s_a2 <= unsigned(v_st(15 downto 0));
                            when 2      => s_a3 <= unsigned(v_st(15 downto 0));
                            when 3      => s_a4 <= unsigned(v_st(15 downto 0));
                            when 4      => s_a6 <= unsigned(v_st(15 downto 0));
                            when others => s_ap <= unsigned(v_st(15 downto 0));
                        end case;
                        z_gp <= "000";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [B] Drift (Hypnos v1.1.0 SPEED): glide 1/8 (Off: this field's knob), |v| < 24 still, else
            -- 2^-12 .. 2^-5 lunation per field (Q20), exponential (mantissa from the 2^(i/256) table, then a
            -- serial shift by the octave); phi0 advances once per field and wraps exactly one lunation
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 8  => s_vtg <= shift_left(resize(signed('0' & s_k5) - 512, 17), 6);
                    when 9  =>
                        if s_glide = '1' then s_vgs <= s_vgs + shift_right(s_vtg - s_vgs, 3);
                        else s_vgs <= s_vtg; end if;
                    when 10 =>
                        v_v := resize(shift_right(s_vgs, 6), 11);
                        s_vneg <= v_v(10);
                        if v_v(10) = '1' then s_av <= unsigned(resize(-v_v, 10)); else s_av <= unsigned(v_v(9 downto 0)); end if;
                    when 11 =>
                        if s_av < 24 then s_vz <= '1'; s_u <= (others => '0');
                        else s_vz <= '0'; s_u <= s_av - 24; end if;
                    when 12 =>
                        s_ls <= s_u - shift_right(s_u, 4) - shift_right(s_u, 5);
                    when 13 =>
                        r_ex_a <= s_ls(5 downto 0) & "00";
                    when 30 =>
                        c_phi <= f_addsub(c_phi, c_spd, c_sneg);
                    when others => null;
                end case;
            end if;
            if s_zone = '1' and s_zc = 16 then
                if s_vz = '1' then c_spd <= (others => '0');
                else c_spd <= resize(r_exf(15 downto 7), 20); end if;
                c_sneg <= s_vneg;
                s_scnt <= s_ls(8 downto 6);
            elsif s_zone = '1' and s_zc > 16 and s_zc < 26 and s_scnt /= 0 then
                c_spd <= shift_left(c_spd, 1);
                s_scnt <= s_scnt - 1;
            end if;
            if s_zone = '1' and s_zc = 40 then
                -- the pans' signs and magnitudes (|o| = g - 32768 or 32767 - g, a one's complement)
                z_oxn <= not s_a1(15);
                if s_a1(15) = '1' then z_oxm <= s_a1(14 downto 0); else z_oxm <= not s_a1(14 downto 0); end if;
                s_g <= s_a6(15 downto 6);
            end if;

            -- [C] serial products, one slot every 26 zone clocks from zc 46: per-slot ROM address at phase 0,
            -- operands muxed at phase 3, loaded at phase 4, 20 iterations, stored at phase 25.
            if s_zone = '0' then
                z_run <= '0';
            elsif s_zc = 46 then
                z_run <= '1'; z_ph <= (others => '0'); z_slot <= (others => '0');
            elsif z_run = '1' then
                if z_ph = 25 then
                    z_ph <= (others => '0');
                    if z_slot = C_NSLOT then z_run <= '0'; else z_slot <= z_slot + 1; end if;
                else
                    z_ph <= z_ph + 1;
                end if;
            end if;
            z_ld <= '0';
            if z_run = '1' then
                case to_integer(z_ph) is
                    when 0 =>
                        if z_slot = 1 then r_ex_a <= z_oct(7 downto 0); end if;                  -- 2^(f/256) mantissa
                        if z_slot = 6 then r_ex_a <= z_o5(7 downto 0); end if;
                    when 3 =>
                        z_ld <= '1';
                        case to_integer(z_slot) is
                            when 0  => z_fa <= resize(not s_a3, 22); z_fb <= to_unsigned(570, 20);        -- octaves
                            when 1  => z_fa <= resize(c_kdx, 22); z_fb <= resize(r_exf, 20);              -- pitch
                            when 2  => z_fa <= resize(s_a4, 22);  z_fb <= to_unsigned(23616, 20);         -- disc
                            when 3  => z_fa <= resize(c_P, 22);   z_fb <= resize(z_d16, 20);              -- radius
                            when 4  => z_fa <= resize(c_R4, 22);  z_fb <= resize(c_R4, 20);               -- R4^2
                            when 5  => z_fa <= resize(s_ap, 22);  z_fb <= to_unsigned(1281, 20);          -- span octaves
                            when 6  => z_fa <= resize(r_exf, 22); z_fb <= to_unsigned(32617, 20);
                            when 7 | 8 => z_fa <= resize(z_oxm, 22); z_fb <= resize(c_kx, 20);            -- pans
                            when 9  => z_fa <= resize(dv_q(24 downto 5), 22); z_fb <= to_unsigned(43691, 20);   -- 1/(2 R4 a)
                            when others => z_fa <= resize(z_s16, 22); z_fb <= resize(z_n0, 20);           -- first phase
                        end case;
                    when 25 =>
                        case to_integer(z_slot) is
                            when 0  =>
                                z_oct <= f_prod(25 downto 16);
                            when 1  =>
                                case to_integer(z_oct(9 downto 8)) is                 -- >> (15 - e), even
                                    when 0      => c_P <= resize(f_prod(30 downto 16) & '0', 18);
                                    when 1      => c_P <= resize(f_prod(30 downto 15) & '0', 18);
                                    when others => c_P <= f_prod(30 downto 14) & '0';
                                end case;
                            when 2  => z_d16 <= to_unsigned(9175, 16) + f_prod(31 downto 16);
                            when 3  => c_R <= f_prod(32 downto 16);
                            when 4  => c_R4s <= f_prod(25 downto 0);
                            when 5  => z_o5 <= f_prod(26 downto 16);
                            when 6  =>                                                           -- (mant 32617 >> (20 - e)) - 1019
                                case to_integer(z_o5(10 downto 8)) is
                                    when 0      => z_s16 <= resize(f_prod(30 downto 20), 16) - 1019;
                                    when 1      => z_s16 <= resize(f_prod(30 downto 19), 16) - 1019;
                                    when 2      => z_s16 <= resize(f_prod(30 downto 18), 16) - 1019;
                                    when 3      => z_s16 <= resize(f_prod(30 downto 17), 16) - 1019;
                                    when others => z_s16 <= f_prod(31 downto 16) - 1019;
                                end case;
                            when 7  =>
                                rd_x <= signed(f_addsub(unsigned(resize(c_xo, 21)), resize(f_prod(31 downto 13), 21), not z_oxn));
                            when 8  =>
                                rd_y <= signed(f_addsub(unsigned(resize(c_yo, 21)), resize(f_prod(31 downto 13), 21), z_oxn));
                            when 9  =>
                                if c_il = '1' then c_KA <= f_prod(35 downto 16); else c_KA <= dv_q(24 downto 5); end if;
                                c_Kr <= dv_q(24 downto 7);
                            when others =>
                                c_phf <= c_phi(19 downto 4) + f_prod(15 downto 0);
                        end case;
                    when others => null;
                end case;
            end if;
            -- slot 3 done: P / 2 and the DDA thresholds; slot 4: R4
            if z_run = '1' and z_slot = 2 and z_ph = 0 then
                c_Ph <= c_P(17 downto 1);
            end if;
            if z_run = '1' and z_slot = 2 and z_ph = 2 then
                c_mP <= -signed(resize(c_P, 20));
                c_th <= signed(resize(c_Ph, 20)) - 256;
                c_k2 <= to_signed(256, 20) - signed(resize(c_P, 20));
            end if;
            if z_run = '1' and z_slot = 4 and z_ph = 0 then
                c_R4 <= c_R(16 downto 4);
                c_q32 <= signed(resize(c_P & "00000", 24)) - signed(resize(c_Ph, 24));
                c_q31 <= signed(resize(c_P & "00000", 24)) - signed(resize(c_P, 24));
                c_xlim <= -signed(resize(g_wm1 & "00000000", 21));
                -- Fine: half line width 16 R / 28.4 px (2/3 interlaced), at least 1 virtual px
                z_h <= resize(c_R(16 downto 9), 8) + resize(c_R(16 downto 12), 8);
            end if;
            if z_run = '1' and z_slot = 4 and z_ph = 1 then
                if c_il = '1' then v_16 := resize(z_h - shift_right(z_h, 2) - shift_right(z_h, 4), 17);
                else v_16 := resize(z_h, 17); end if;
                if s_fine = '0' then c_h16 <= (others => '0');
                elsif v_16 < c_hmin then c_h16 <= resize(c_hmin, 8);
                else c_h16 <= v_16(7 downto 0); end if;
            end if;
            -- slot 8: the y pan's operand and gain (the x pan's registers, reused)
            if z_run = '1' and z_slot = 8 and z_ph = 0 then
                z_oxn <= not s_a2(15);
                if s_a2(15) = '1' then z_oxm <= s_a2(14 downto 0); else z_oxm <= not s_a2(14 downto 0); end if;
                c_kx <= c_ky;
            end if;
            -- slot 7: the span's steps (per cell; 3 s and 7 s for the rows)
            if z_run = '1' and z_slot = 7 and z_ph = 0 then
                if s_brick = '1' then c_Sx <= z_s16(14 downto 0) & '0'; else c_Sx <= z_s16; end if;
                z_d3 <= z_s16 + shift_left(z_s16, 1);
            end if;
            if z_run = '1' and z_slot = 7 and z_ph = 1 then
                z_d7 <= shift_left(z_s16, 3) - z_s16;
            end if;

            -- [C] serial multiplier: load / 20 shift-add iterations (shared by the sequencer and the per-line
            -- engine, which never run together)
            if z_ld = '1' then
                f_hi <= (others => '0'); f_n <= to_unsigned(20, 5);
                f_a <= z_fa; f_b <= z_fb;
            elsif f_n /= 0 then
                if f_b(0) = '1' then v_s := resize(f_hi, 24) + resize(f_a, 24); else v_s := resize(f_hi, 24); end if;
                f_hi <= v_s(23 downto 1);
                f_b <= v_s(0) & f_b(19 downto 1);
                f_n <= f_n - 1;
            end if;

            -- [C] serial divider (slot 5): inv = floor(2^32 / R4), restoring, one quotient bit per clock
            if z_run = '1' and z_slot = 5 and z_ph = 0 then
                dv_rem <= (others => '0'); dv_q <= (others => '0'); dv_n <= to_unsigned(33, 6);
            elsif dv_n /= 0 then
                if dv_n = 33 then v_r := to_unsigned(1, 15); else v_r := dv_rem & '0'; end if;
                if v_r >= resize(c_R4, 15) then
                    dv_rem <= resize(v_r - resize(c_R4, 15), 14); dv_q <= dv_q(24 downto 0) & '1';
                else
                    dv_rem <= v_r(13 downto 0); dv_q <= dv_q(24 downto 0) & '0';
                end if;
                dv_n <= dv_n - 1;
            end if;

            -- [C] reductions (slots 9 .. 11): pixel 0 / line 0 into [Ph - P, Ph), counting cells
            -- (rd_a holds the value minus Ph, so the step decision is its sign; one adder for the starts and
            -- the steps; the result is rd_a + Ph, registered at the end of each axis)
            v_ri := rd_a; v_rk := resize(c_mP, 24); v_rgo := '0';
            if z_run = '1' and z_slot = 9 and z_ph = 0 then
                v_ri := resize(rd_x, 24); v_rk := c_q32; v_rgo := '1';                -- x + 32 P - Ph
                rd_ph <= "01"; rd_n <= to_signed(-29, 8);
            elsif rd_ph = "01" or rd_ph = "10" then
                if rd_a(23) = '0' then
                    v_rgo := '1'; rd_n <= rd_n + 1;
                elsif rd_ph = "01" then
                    rd_i <= rd_n;
                    rd_ph <= "11"; rd_n <= to_signed(-30, 8);
                    v_ri := resize(rd_y, 24); v_rk := c_q31; v_rgo := '1';            -- y + 32 P - Ph - Ph
                else
                    rd_j <= rd_n;
                    rd_ph <= "00";
                end if;
            elsif rd_ph = "11" then
                rd_ph <= "10";
            end if;
            if v_rgo = '1' then rd_a <= v_ri + v_rk; end if;
            -- the reduced values: rd_a + Ph when an axis ends (x: rd_ph 01 -> 11, y: 10 -> 00)
            v_rr := resize(rd_a, 20) + signed(resize(c_Ph, 20));
            if rd_ph = "01" and rd_a(23) = '1' then rd_xr <= v_rr; end if;
            if rd_ph = "10" and rd_a(23) = '1' then rd_yr <= v_rr; end if;
            if z_run = '1' and z_slot = 12 and z_ph = 0 then
                -- odd rows: shifted right by P / 2 (P is even: the range is [-Ph, Ph))
                c_u0e <= rd_xr;
                c_u0o <= signed(f_addsub(unsigned(rd_xr), resize(c_Ph, 20), not rd_xr(19)));
                rd_dl <= rd_xr(19);
                c_v0 <= rd_yr;
                c_par0 <= rd_j(0);
            end if;
            if z_run = '1' and z_slot = 12 and z_ph = 1 then
                -- the first-cell index of line 0 (mod 2^16) and the row steps
                v_ni := resize(rd_i, 11); v_nj := resize(rd_j, 11);
                v_d3 := z_d3; v_d7 := z_d7;
                if s_brick = '1' then
                    v_nn := shift_left(v_ni, 1) + shift_left(v_nj, 1);
                    if rd_j(0) = '1' and rd_dl = '1' then v_nn := v_nn - 1;
                    elsif rd_j(0) = '1' then v_nn := v_nn + 1; end if;
                    if rd_dl = '1' then c_D1 <= z_s16; c_D2 <= v_d3;
                    else c_D1 <= v_d3; c_D2 <= z_s16; end if;
                else
                    v_nn := v_ni + shift_left(v_nj, 3) - v_nj;
                    c_D1 <= v_d7; c_D2 <= v_d7;
                end if;
                z_n0 <= unsigned(resize(v_nn, 16));
            end if;

            -- [C] per-line engine.  Init (line 0, in the zone) or step (at t_go): v += dyp, wrap into the next
            -- row (parity, first-cell phase, pixel 0's u); then v^2 -> w^2 = R4^2 - v^2 -> w = sqrt (serial)
            -- -> the limb's A = w^2 / (2 R4 a) (Q28 -> 1/16 px) -> rho = 32 w / R4 for the kt ROM.
            t_zf <= '0';
            if s_zone = '1' and s_zc = C_LI_ZC then
                l_run <= '1'; l_init <= '1'; l_ph <= (others => '0');
            elsif t_go = '1' then
                l_run <= '1'; l_init <= '0'; l_ph <= (others => '0');
            elsif l_run = '1' then
                l_ph <= l_ph + 1;
                case to_integer(l_ph) is
                    when 0 =>
                        if l_init = '1' then
                            l_v <= c_v0; l_par <= c_par0; l_phr <= c_phf;
                            if c_par0 = '1' and s_brick = '1' then l_ur <= c_u0o; else l_ur <= c_u0e; end if;
                        end if;
                    when 1 =>
                        if l_init = '0' and l_v >= signed(resize(c_Ph, 20)) then
                            l_par <= not l_par;
                            if l_par = '0' then l_phr <= l_phr + c_D1; else l_phr <= l_phr + c_D2; end if;
                            if l_par = '0' and s_brick = '1' then l_ur <= c_u0o; else l_ur <= c_u0e; end if;
                        end if;
                    when 2 =>
                        v_nv := shift_right(l_v, 4);
                        if v_nv(19) = '1' then l_av4 <= unsigned(resize(-v_nv, 14)); else l_av4 <= unsigned(v_nv(13 downto 0)); end if;
                    when 3 =>
                        z_ld <= '1'; z_fa <= resize(l_av4, 22); z_fb <= resize(l_av4, 20);
                    when 25 =>
                        l_w2 <= signed(resize(c_R4s, 27)) - signed(resize(f_prod(26 downto 0), 27));
                    when 26 =>
                        z_ld <= '1'; z_fb <= c_KA;
                        if l_w2(26) = '1' then z_fa <= resize(shift_right(unsigned(not l_w2), 4), 22);   -- |w^2| - 1
                        else z_fa <= resize(shift_right(unsigned(l_w2), 4), 22); end if;
                    when 48 =>
                        v_al := f_prod(40 downto 24);
                        if l_w2(26) = '1' then
                            if v_al(16 downto 13) /= 0 then l_al <= to_signed(-8192, 14);
                            else l_al <= -signed(resize(v_al(12 downto 0), 14)); end if;
                        else
                            if v_al(16 downto 12) /= 0 then l_al <= to_signed(4095, 14);
                            else l_al <= signed(resize(v_al(11 downto 0), 14)); end if;
                        end if;
                    when 49 =>
                        z_ld <= '1'; z_fa <= resize(q_root, 22); z_fb <= resize(c_Kr, 20);
                    when 71 =>
                        if f_prod(35 downto 20) >= 31 then l_rho <= to_unsigned(31, 5);
                        else l_rho <= f_prod(24 downto 20); end if;
                        l_run <= '0';
                        if l_init = '1' then t_zf <= '1'; end if;
                    when others => null;
                end case;
            end if;
            -- the step (phase 0) and the row wrap (phase 1) on one adder
            if l_run = '1' and l_init = '0' and (l_ph = 0 or (l_ph = 1 and l_v >= signed(resize(c_Ph, 20)))) then
                if l_ph = 0 then v_op := signed(resize(c_dyp, 20)); else v_op := c_mP; end if;
                l_v <= l_v + v_op;
            end if;
            -- the square root: 13 digit pairs of max(w^2, 0)
            if l_run = '1' and l_ph = 26 then
                if l_w2(26) = '1' then q_rad <= (others => '0'); else q_rad <= unsigned(l_w2(25 downto 0)); end if;
                q_rem <= (others => '0'); q_root <= (others => '0'); q_n <= to_unsigned(13, 4);
            elsif q_n /= 0 then
                v_qr := q_rem(13 downto 0) & q_rad(25 downto 24);
                v_qt := resize(q_root & "01", 16);
                q_rad <= q_rad(23 downto 0) & "00";
                if v_qr >= v_qt then
                    q_rem <= v_qr - v_qt; q_root <= q_root(11 downto 0) & '1';
                else
                    q_rem <= v_qr; q_root <= q_root(11 downto 0) & '0';
                end if;
                q_n <= q_n - 1;
            end if;

            -- [C] line trigger: the end of each generated line (or the zone's line-0 init) makes the next line
            -- current; the cell engine and the per-line engine start from it
            g_avd <= g_avid;
            t_go <= (g_avd and not g_avid) or t_zf;
            if t_go = '1' then
                cur_ur <= l_ur; cur_w <= q_root; cur_al <= l_al; cur_rho <= l_rho; cur_phr <= l_phr;
            end if;

            -- [A] sync delay, generated-avid and start-pulse shift registers (field_n is tied high by the
            -- core on every input, so it only gets a register)
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 8);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [C] SQUARES FILL (vblank zone): SQ(i) = min(4095, KA i^2 >> 22) = the limb's u^2 / (2 R a) in 1/16 px
    -- at u = i / 2 px, by second differences
    ---------------------------------------------------------------------------
    p_fill : process(clk)
    begin
        if rising_edge(clk) then
            sq_we <= '0';
            if s_zone = '1' and s_zc = C_SQ_ZC then
                sf_run <= '1'; sf_i <= (others => '0');
                sf_S <= (others => '0'); sf_D <= resize(c_KA, 31);
            elsif sf_run = '1' then
                sq_we <= '1'; sq_wa <= sf_i;
                if sf_S(39 downto 34) /= 0 then sq_wd <= (others => '1'); else sq_wd <= sf_S(33 downto 22); end if;
                sf_S <= sf_S + resize(sf_D, 40);
                sf_D <= sf_D + resize(c_KA & '0', 31);
                sf_i <= sf_i + 1;
                if sf_i = 1023 then sf_run <= '0'; end if;
            end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- [C] CELL ENGINE (25 clocks a cell): phase -> cos ROM; the camera level at the cell centre; m = min(4L,
    -- 255) G; c1 = c + m (t - c) with t = 1 - 2.2 L; c = clamp(c1, 2c - 1, 2c + 1); c w; kt ROM by (rho, |c|).
    -- Line start: cells 0 and 1 of the new line (to the current and next cell); then one cell per crossing.
    ---------------------------------------------------------------------------
    p_cell : process(clk)
        variable v_p   : unsigned(15 downto 0);
        variable v_cx  : signed(12 downto 0);
        variable v_t   : signed(14 downto 0);
        variable v_c2  : signed(14 downto 0);
        variable v_pp  : signed(14 downto 0);
        variable v_a   : unsigned(9 downto 0);
        variable v_sel : unsigned(14 downto 0);
        variable v_s   : unsigned(14 downto 0);
        variable v_ac  : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            e_ov <= '0';
            if t_go = '1' then
                -- line start: the new line's first cell (its phase and pixel 0's u are the line state's)
                e_tp <= l_phr; e_xn <= resize(l_ur, 21);
                e_ls <= "01"; e_dcs <= '1'; e_run <= '1'; e_st <= (others => '0');
            elsif x3 = '1' and e_run = '0' then
                e_dcs <= '0'; e_run <= '1'; e_st <= (others => '0');
            elsif e_run = '1' then
                e_st <= e_st + 1;
                case to_integer(e_st) is
                    when 0 =>
                        -- the cell centre's column (the camera), the cos ROM address, the waxing flag
                        -- col = clamp(-(xn >> 8), 0, W - 1): the right clamp is xn < -256 (W - 1), the left xn >= 256
                        v_cx := -resize(shift_right(e_xn, 8), 13);
                        if e_xn < c_xlim then e_col <= g_wm1(10 downto 0);
                        elsif e_xn(20) = '0' and e_xn(19 downto 8) /= 0 then e_col <= (others => '0');
                        else e_col <= unsigned(v_cx(10 downto 0)); end if;
                        v_p := e_tp + 16384;
                        if v_p(14) = '1' then e_sin_a <= not v_p(13 downto 4); else e_sin_a <= v_p(13 downto 4); end if;
                        e_qs <= v_p(15);
                        e_wax <= not e_tp(15);
                    when 1 =>
                        e_cam_a <= e_col(10 downto 1);
                    when 2 =>
                        e_sin_f <= e_sin_q;
                    when 3 =>
                        if e_qs = '1' then e_c <= -signed(resize(e_sin_f, 13)); else e_c <= signed(resize(e_sin_f, 13)); end if;
                        e_cam_f <= e_cam_q;
                    when 4 =>
                        -- m = G x min(4 L, 255) >> 8
                        if e_cam_f(7 downto 6) /= "00" then v_a := to_unsigned(255, 10);
                        else v_a := resize(e_cam_f(5 downto 0) & "00", 10); end if;
                        m_a <= resize(s_g, 13); m_b <= resize(v_a, 12); m_hi <= (others => '0');
                        e_ts1 <= resize(e_cam_f & "0000", 13) + resize(e_cam_f & "00", 13);
                        e_ts2 <= resize(e_cam_f(7 downto 1), 8) + resize(e_cam_f(7 downto 4), 8);
                        -- the band that keeps new and full moons: [max(-1, 2c - 1), min(1, 2c + 1)]
                        v_c2 := shift_left(resize(e_c, 15), 1);
                        if v_c2 < 0 then e_lo <= to_signed(-2048, 14); else e_lo <= resize(v_c2 - 2048, 14); end if;
                        if v_c2 > 0 then e_hi <= to_signed(2048, 14); else e_hi <= resize(v_c2 + 2048, 14); end if;
                    when 5 =>
                        e_tsum <= resize(e_ts1, 14) + resize(e_ts2, 14);
                    when 6 =>
                        v_t := to_signed(2048, 15) - signed(resize(e_tsum, 15));
                        if e_tsum > 4096 then e_t <= to_signed(-2048, 14); else e_t <= resize(v_t, 14); end if;
                    when 7 =>
                        e_d <= e_t - resize(e_c, 14);
                    when 8 =>
                        if e_d(13) = '1' then e_ad <= unsigned(resize(-e_d, 13)); else e_ad <= unsigned(e_d(12 downto 0)); end if;
                        e_sd <= e_d(13);
                    when 10 =>
                        -- p = m |d| >> 10
                        m_a <= e_ad; m_b <= resize(m_hi(9 downto 0), 12); m_hi <= (others => '0');
                    when 17 =>
                        v_pp := signed(resize(m_hi, 15));
                        if e_sd = '1' then e_c1 <= resize(e_c, 15) - v_pp; else e_c1 <= resize(e_c, 15) + v_pp; end if;
                    when 18 =>
                        if e_c1 < resize(e_lo, 15) then e_ce <= e_lo;
                        elsif e_c1 > resize(e_hi, 15) then e_ce <= e_hi;
                        else e_ce <= resize(e_c1, 14); end if;
                    when 19 =>
                        -- c w = |c| w >> 7; the kt ROM by (rho, |c|)
                        if e_ce(13) = '1' then v_ac := unsigned(resize(-e_ce, 13)); else v_ac := unsigned(e_ce(12 downto 0)); end if;
                        e_sce <= e_ce(13);
                        if v_ac(12 downto 11) /= "00" then e_kt_a <= cur_rho & "11111";
                        else e_kt_a <= cur_rho & v_ac(10 downto 6); end if;
                        m_a <= cur_w; m_b <= v_ac(11 downto 0); m_hi <= (others => '0');
                    when 21 =>
                        e_kt_f <= e_kt_q;
                    when 27 =>
                        e_ov <= '1'; e_os <= e_wax; e_okt <= e_kt_f; e_odcs <= e_dcs;
                        e_ocw <= m_hi & m_b(11 downto 7); e_osc <= e_sce;
                        e_tp <= e_tp + c_Sx;
                        e_xn <= e_xn - signed(resize(c_P, 21));
                        e_st <= (others => '0');
                        if e_ls = "01" then
                            e_ls <= "10"; e_dcs <= '0';
                        else
                            e_ls <= "00"; e_run <= '0';
                        end if;
                    when others => null;
                end case;
                -- 3 a for the radix-4 steps (5, 11, 20), then the steps: (6 .. 9) G x a, (12 .. 16) |d| x m,
                -- (21 .. 26) w x |c|; {hi, b} shifts right two bits a step, so b ends with the product's low bits
                if e_st = 5 or e_st = 11 or e_st = 20 then
                    m_a3 <= resize(m_a, 15) + resize(m_a & '0', 15);
                end if;
                if (e_st >= 6 and e_st <= 9) or (e_st >= 12 and e_st <= 16) or (e_st >= 21 and e_st <= 26) then
                    case m_b(1 downto 0) is
                        when "00"   => v_sel := (others => '0');
                        when "01"   => v_sel := resize(m_a, 15);
                        when "10"   => v_sel := resize(m_a & '0', 15);
                        when others => v_sel := m_a3;
                    end case;
                    v_s := resize(m_hi, 15) + v_sel;
                    m_hi <= v_s(14 downto 2);
                    m_b <= v_s(1 downto 0) & m_b(11 downto 2);
                end if;
            end if;
        end if;
    end process p_cell;

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
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> box over 64 virtual px
    -- (48 at 720p), edge-replicated and reloaded at every line start (the box's
    -- resync) -> vertical one-pole IIR in the line store (half horizontal
    -- resolution; the first line of a field loads it).
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l   : unsigned(7 downto 0);
        variable v_s   : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_d   : signed(12 downto 0);
        variable v_dd  : signed(12 downto 0);
        variable v_n   : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting; both phases restart in the
            -- zone (Hardline dazzle's fix of riley's line-0 phase)
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
            if (c_b64 = '1' and s_bcnt /= 64) or (c_b64 = '0' and s_bcnt /= 32) then v_tap := s_pfirst; else v_tap := bx_q; end if;
            if s_avd(2) = '1' and s_avd(3) = '0' then
                s_pfirst <= s_p;
                if c_b64 = '1' then s_bsum <= shift_left(resize(s_p, 14), 6);
                else s_bsum <= shift_left(resize(s_p, 14), 5); end if;
                s_bcnt <= to_unsigned(1, 7);
            elsif s_avd(2) = '1' then
                s_bsum <= s_bsum + s_p - v_tap;
                if (c_b64 = '1' and s_bcnt /= 64) or (c_b64 = '0' and s_bcnt /= 32) then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bcol <= s_pcol;

            -- 4: fabric copies (the box mean x 16); 5: vertical IIR on the odd columns, q += (box - q) >> k
            -- (the first line: q = box)
            if c_b64 = '1' then s_eb <= s_bsum(13 downto 2); else s_eb <= s_bsum(12 downto 1); end if;
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
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] MOONS (stages 3 .. 19)
    --   3  u DDA (Q8 px, +1 px a pixel; a crossing steps back one pitch and takes the next cell)
    --   4  |u|, s u        5  squares index round(2 |u|), dt = s u - c w        6  RAM, dt in 1/16 px (sat)
    --   7  fabric, dt x kt          8  limb dl = A - SQ, dtk          9  limb cover, interlace 2/3, |dl|
    --   10 ring, terminator line, lit = max(0, limb + terminator - 1)      11 the line clipped to the disc
    --   12 tone x      13 luma ROM: 64 + 876 x / 256      14 fabric      15 delay
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ix  : unsigned(18 downto 0);
        variable v_x   : unsigned(9 downto 0);
        variable v_r   : signed(13 downto 0);
        variable v_z, v_o : std_logic;
        variable v_n   : signed(12 downto 0);
        variable v_l6  : unsigned(5 downto 0);
        variable v_tc  : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            -- the cell values: written by the engine (line start: current and next), next -> current at a crossing
            if e_ov = '1' and e_odcs = '1' then cs_s <= e_os; cs_sc <= e_osc; cs_cw <= e_ocw; cs_kt <= e_okt; end if;
            if e_ov = '1' and e_odcs = '0' then nx_s <= e_os; nx_sc <= e_osc; nx_cw <= e_ocw; nx_kt <= e_okt; end if;

            -- 3: u DDA
            x3 <= '0';
            if s_st(0) = '1' then
                u3 <= cur_ur - 256;
            elsif s_gav_sr(0) = '1' then
                if u3 >= c_th then
                    u3 <= u3 + c_k2; x3 <= '1';
                    cs_s <= nx_s; cs_sc <= nx_sc; cs_cw <= nx_cw; cs_kt <= nx_kt;
                else
                    u3 <= u3 + 256;
                end if;
            end if;
            -- 4: |u|; ex = u + |c w| when s = sign(c), else u - |c w|  (= dt waxing, -dt waning)
            if u3(19) = '1' then au4 <= unsigned(not u3(18 downto 0)); else au4 <= unsigned(u3(18 downto 0)); end if;   -- |u| (- 1 when u < 0)
            ex4 <= signed(f_addsub(unsigned(resize(u3, 21)), resize(cs_cw, 21), cs_s xor cs_sc));
            s4 <= cs_s; kt4 <= cs_kt;
            -- 5: squares index; |dt| >> 4 saturated; the sign of dt
            v_ix := au4 + 64;
            if v_ix(18 downto 17) /= "00" then idx5 <= (others => '1'); else idx5 <= v_ix(16 downto 7); end if;
            v_z := '1'; v_o := '1';
            for i in 12 to 20 loop
                if ex4(i) = '1' then v_z := '0'; else v_o := '0'; end if;
            end loop;
            v_n := -ex4(12 downto 0);
            if v_z = '1' then mg5 <= unsigned(ex4(11 downto 4));
            elsif v_o = '1' and v_n(12) = '0' then mg5 <= unsigned(v_n(11 downto 4));
            else mg5 <= (others => '1'); end if;
            if s4 = '1' then ng5 <= ex4(20);
            elsif ex4(20) = '0' and ex4 /= 0 then ng5 <= '1';
            else ng5 <= '0'; end if;
            kt5 <= kt4;
            -- 6 (RAM): |dt| x kt
            pr6 <= mg5 * kt5;
            ng6 <= ng5;
            if s_st(3) = '1' then al_px <= cur_al; end if;
            -- 7
            sq_f <= sq_q;
            mk7 <= pr6(13 downto 6);
            ng7 <= ng6;
            -- 8: the limb distance; interlaced: 2/3 of the terminator's
            dl8 <= resize(al_px, 15) - signed(resize(sq_f, 15));
            if c_il = '1' then mk8 <= mk7 - shift_right(mk7, 2) - shift_right(mk7, 4); else mk8 <= mk7; end if;
            ng8 <= ng7;
            -- 9: limb cover, |dl|, terminator cover 8 -+ mk
            cl9 <= f_cov(dl8);
            if dl8(14) = '1' then
                if dl8 < -4095 then adl9 <= to_unsigned(4095, 12); else adl9 <= unsigned(resize(-dl8, 12)); end if;
            else
                if dl8 > 4095 then adl9 <= to_unsigned(4095, 12); else adl9 <= unsigned(dl8(11 downto 0)); end if;
            end if;
            if mk8(7 downto 3) /= 0 then
                if ng8 = '1' then ct9 <= (others => '0'); else ct9 <= to_unsigned(16, 5); end if;
            elsif ng8 = '1' then ct9 <= to_unsigned(8, 5) - resize(mk8(2 downto 0), 5);
            else ct9 <= resize(mk8(2 downto 0), 5) + 8; end if;
            ak9 <= mk8;
            -- 10: the ring and the terminator line; lit = min(limb, terminator)
            v_r := signed(resize(c_h16, 14)) - signed(resize(adl9, 14));
            rg10 <= f_cov(v_r);
            v_r := signed(resize(c_h16, 14)) - signed(resize(ak9, 14));
            tl10 <= f_cov(v_r);
            v_l6 := resize(cl9, 6) + resize(ct9, 6);                                         -- lit = max(0, cl + ct - 16)
            if v_l6(5 downto 4) = "00" then li10 <= (others => '0');
            elsif v_l6(5) = '1' then li10 <= to_unsigned(16, 5);
            else li10 <= '0' & v_l6(3 downto 0); end if;
            cl10 <= cl9;
            -- 11: the terminator line clipped to the disc; the line = max(ring, that); tone terms
            v_tc := f_min5(tl10, cl10);
            if rg10 > v_tc then mx11 <= rg10; else mx11 <= v_tc; end if;
            a11 <= resize(cl10, 7) + resize(cl10 & '0', 7);                                  -- 3 cover
            b11 <= resize(li10 & "000", 8) + resize(li10 & "00", 8) + resize(li10, 8);       -- 13 lit
            c11 <= resize(li10 & "00", 7) + resize(li10, 7);                                 -- 5 lit
            li11 <= li10;
            -- 12: tone x: Grey Bold 3 cover + 13 lit | Ink Bold 16 lit | Grey Fine min(256, 16 line + 5 lit) |
            -- Ink Fine 16 line
            if s_fine = '0' then
                if s_ink = '0' then x12 <= resize(a11, 9) + resize(b11, 9); else x12 <= li11 & "0000"; end if;
            else
                if s_ink = '0' then
                    v_x := resize(mx11 & "0000", 10) + resize(c11, 10);
                    if v_x(9 downto 8) /= "00" and v_x(7 downto 0) /= 0 then x12 <= to_unsigned(256, 9);
                    else x12 <= v_x(8 downto 0); end if;
                else
                    x12 <= mx11 & "0000";
                end if;
            end if;
            -- 13 (ROM), 14: Y = 64 + (876 x) >> 8; 15
            y14 <= y13;
            y15 <= y14;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING (stages 16 .. 21): luma [1 2 1]/4, luma 13/16
    -- toward black (white 776), blanking gate.  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_yd(1) <= y15;
            for i in 2 to 3 loop s_yd(i) <= s_yd(i - 1); end loop;
            s_ya <= resize(s_yd(1), 12) + resize(s_yd(3), 12) + 2;
            s_yb <= s_yd(2);
            v_y := s_ya + shift_left(resize(s_yb, 12), 1);
            s_y1 <= v_y(11 downto 2);
            if s_y1 > 64 then v_dl := s_y1 - 64; else v_dl := (others => '0'); end if;
            s_y2 <= s_y1 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y3 <= s_y2;
            else
                s_y3 <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_out;

    f_prod <= f_hi & f_b;

    data_out.y       <= std_logic_vector(s_y3);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture lunation;
