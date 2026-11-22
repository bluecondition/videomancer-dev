-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Steiner v0.1.0 -- Vellum.  After the Steiner chain and Steiner's porism, and the Japanese sangaku tablets
-- (circles packed between circles, drawn in ink and hung in temples): a great circle, a smaller circle inside it
-- and off centre, and between them a closed chain of n discs, each touching both circles and both neighbours.
-- The chain turns: each disc swells round the wide side and shrinks past the narrow side, and every tangency
-- holds the whole way.  Nested draws the same construction again inside the inner circle, turning the other way.
--
-- Construction (unit great circle, eccentricity a, chain phase al): the concentric chain (inner radius
-- rho0 = (1 - s) / (1 + s), s = sin(pi / n)) carried by the disc map M(z) = (z + a) / (1 + a z); chain disc k has
--   E = 1 + 2 a cos al_k + a^2,  r = (1 - a^2) / ((1 - a^2) + E / s),
--   centre (1 - r) (((1 + a^2) cos al_k + 2 a) / E, (1 - a^2) sin al_k / E),     al_k = al + k / n
-- and the figure turns by beta about the great circle's centre.  The per-field engine (steiner.py) computes every
-- circle; per pixel each circle is a slot: P = |z - c|^2 - r^2 by second differences (one shared quadratic about
-- the great circle's centre minus the slot's own linear step), its edge distance P / 2r by a shift (with a 3/4
-- step), coverage over one pixel.
--
--   K1  X        the great circle's centre, +-960 virtual px    K2  Y   the same, +-540 virtual px
--   K3  Scale    the great circle's radius 227 .. 1100 virtual px, exponential (the top runs off the screen)
--   K4  Count    n = 3 .. 12 discs (stepped, hysteresis, the two ends kept)
--   K5  Drift    the chain circulates (bipolar: +-24 still, 2^-12 .. 2^-5.8 turn per field); the eccentricity axis
--                turns the other way at a quarter of the rate
--   K6  Camera   0 .. 100 %: the luma (box + vertical one-pole, a per-field l^2 table) is added to every rho^2:
--                discs dent away from the light, the great circle swells into it (x 8)
--   S7  Glide    On: the knobs ease 1/16 of the gap per field (Drift's speed 1/8); Off: no easing
--   S8  Form     Chain / Nested (a second chain inside the inner circle, n1 + n2 <= 12, turning the other way)
--   S9  Tone     Grey (gap dark, inner disc mid-grey, chain discs light in four steps by size) / Ink (white chain
--                discs and inner disc inside a white frame line, overlaps XOR)
--   S10 Weight   Bold (discs) / Fine (every circle a thin line; Grey: over a wash)
--   S11 Source   Video / Osc (the shell's drifting triangle lattice)
--   P12 ECCENTRIC  a = 0 .. 0.85: a symmetric ring of equal discs .. one enormous disc and a string of pearls
--
-- Sections (the integer model .vellum_work/rb_steiner/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         switches, the Count steps, the mode constants
--   [C] FIELD ENGINE                     Hardline flute's microcoded engine (steiner.py) with a two-deep return
--                                        stack; runs through the field and writes the next field's circles into
--                                        the parameter RAM
--   [Q] FIELD START COPY + LINE ENGINE   the copy moves the parameters into the line RAMs and the slots; at each
--                                        line end the line engine steps V += U, U += vv and loads every slot
--   [P] PIXEL PATH (x 15 slots)          P DDA -> windowed edge distance -> coverage 0 .. 8
--   [M] COMPOSITE                        chain sum, class by priority, tone ROM
--   [D] SOURCE FIELD                     luma (or Osc) -> per-field l^2 table -> box -> vertical one-pole
--   [F] OUTPUT CONDITIONING              luma [1 2 1] / 4, 13/16 toward black, blanking gate; U = V = 512
--
-- Stage numbering (the Hardline shell's): a register "at stage S" holds pixel n at T + S + n (T = the g_start
-- clock).  Pixels 0 .. W + 1 are rendered; output column j is the [1 2 1] filter over pixels j .. j + 2.
-- s_gav_sr(S - 3) flags pixels 0 .. W - 1 on their way into a stage-S register.  One latency (24) in every mode.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.steiner_pkg.all;
use work.all;

architecture steiner of program_top is
    constant C_LAT  : integer := 24;   -- = processing_delay_clks
    constant NSLOT  : integer := 14;   -- 0 the great circle, 1 the inner circle, 2 .. 13 the chain (Nested: 2 = the
                                       -- nested inner circle)

    ---------------------------------------------------------------------------
    -- [A] timing-ID steps
    ---------------------------------------------------------------------------
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

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_xsr  : unsigned(3 downto 0) := to_unsigned(4, 4);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
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

    -- [B] switches, the Count steps
    signal s_glide : std_logic := '1';
    signal s_nest, s_ink, s_fine, s_src : std_logic := '0';
    signal s_zn : unsigned(3 downto 0) := to_unsigned(4, 4);       -- n - 3
    signal z_ok : std_logic := '0';
    signal z_kz : unsigned(3 downto 0) := to_unsigned(4, 4);
    signal s_k4 : unsigned(9 downto 0) := (others => '0');
    signal z_p  : unsigned(13 downto 0) := (others => '0');

    -- [B] mode constants (per field)
    signal c_xs   : unsigned(3 downto 0) := to_unsigned(4, 4);      -- units (1/4 virtual px) per pixel
    signal c_dd   : unsigned(7 downto 0) := to_unsigned(32, 8);     -- 2 xs^2
    signal c_ys2  : unsigned(6 downto 0) := to_unsigned(16, 7);     -- Q2 units per field line
    signal c_vv   : unsigned(13 downto 0) := to_unsigned(512, 14);  -- 2 ys2^2
    signal c_xpre : signed(12 downto 0) := to_signed(-3846, 13);    -- the pre-pixel's x (units)
    signal c_y0   : signed(14 downto 0) := to_signed(-8632, 15);    -- line 0's y (Q2 units)
    signal c_hw   : unsigned(4 downto 0) := to_unsigned(16, 5);     -- Fine half line (1/8 px)
    signal c_hwf  : unsigned(4 downto 0) := to_unsigned(24, 5);     -- Ink frame half line
    signal s_lg6  : std_logic := '1';                               -- box: 64 samples (1080) / 32
    signal s_ish  : unsigned(2 downto 0) := to_unsigned(4, 3);      -- the vertical one-pole's shift

    -- ROMs: the engine's table (2^(i/256) + quarter sine), tone
    signal r_ex_a : unsigned(8 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal yr_a   : unsigned(8 downto 0) := (others => '0');
    signal yr_q   : signed(11 downto 0) := (others => '0');

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
    constant OP_SPC  : std_logic_vector(3 downto 0) := x"F";      -- 0 return, 1 halt
    signal rf_ram : t_c_st_rfi := C_ST_RFI;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal e_run : std_logic := '0';
    signal e_ph  : unsigned(1 downto 0) := "11";
    signal e_pc, e_lr, e_lr2 : unsigned(9 downto 0) := (others => '0');
    signal e_ir  : std_logic_vector(15 downto 0) := x"F001";
    signal e_irf : std_logic_vector(15 downto 0) := x"F001";
    signal x_mul, x_shr, x_halt, x_out, x_st : std_logic := '0';
    signal x_gate, x_subr, x_shift, x_cz : std_logic := '0';
    signal e_acc, e_b : signed(36 downto 0) := (others => '0');
    signal e_cnt : unsigned(4 downto 0) := (others => '0');
    signal e_out : std_logic;
    signal w_in  : signed(16 downto 0);
    signal e_pa  : unsigned(6 downto 0) := (others => '0');

    -- [Q] the parameter RAM (written by the engine, read by the copy at the field start)
    type t_pr is array (0 to 127) of std_logic_vector(31 downto 0);
    signal pr_ram : t_pr := (others => (others => '0'));
    attribute no_rw_check of pr_ram : signal is true;
    signal pr_q, pr_qf : std_logic_vector(31 downto 0) := (others => '0');
    signal pr_we : std_logic;
    signal pr_ra : unsigned(6 downto 0) := (others => '0');
    signal cp_run : std_logic := '0';
    signal cp_a1, cp_a2 : unsigned(6 downto 0) := (others => '0');
    signal cp_v1, cp_v2 : std_logic := '0';
    signal cp_done : std_logic := '0';
    signal cp_ctl_we : std_logic;
    signal cp_ctl_j : unsigned(4 downto 0);
    signal cp_ctl : std_logic_vector(31 downto 0);
    signal c_dq0 : signed(20 downto 0) := (others => '0');
    signal c_gt  : unsigned(13 downto 0) := (others => '0');
    signal c_sc  : unsigned(3 downto 0) := to_unsigned(6, 4);
    signal tf_go : std_logic := '0';

    -- [Q] line RAMs (V, U per slot, Q4) and the line engine
    type t_lr is array (0 to 31) of std_logic_vector(31 downto 0);
    signal lv_ram, lu_ram : t_lr := (others => (others => '0'));
    attribute no_rw_check of lv_ram : signal is true;
    attribute no_rw_check of lu_ram : signal is true;
    signal lv_q, lu_q, lv_f, lu_f : std_logic_vector(31 downto 0) := (others => '0');
    signal lv_we, lu_we : std_logic;
    signal l_wa : unsigned(4 downto 0);
    signal l_ra : unsigned(4 downto 0) := (others => '0');
    signal lv_wd, lu_wd : std_logic_vector(31 downto 0);
    signal le_go, le_act, le_a1, le_a2, le_a3 : std_logic := '0';
    signal le_j, le_j1, le_j2, le_j3, le_jl : unsigned(4 downto 0) := (others => '0');
    signal le_ld : std_logic := '0';
    signal le_nv : signed(31 downto 0) := (others => '0');
    signal s_gq  : std_logic := '0';

    -- [P] shared pixel-path signals
    signal q_dqx : signed(20 downto 0) := (others => '0');
    signal s_st1 : std_logic := '0';
    signal s1_cs, s1_csd : unsigned(12 downto 0) := (others => '0');
    signal s2_dqc : signed(22 downto 0) := (others => '0');
    signal s_run : std_logic := '0';
    type t_cov is array (0 to NSLOT - 1) of unsigned(3 downto 0);
    signal s8_cov : t_cov := (others => (others => '0'));
    signal s8_in  : std_logic_vector(0 to NSLOT - 1) := (others => '0');
    signal c_nin  : std_logic := '0';                               -- slot 2 is the nested inner circle
    signal s8_near : std_logic_vector(0 to NSLOT - 1) := (others => '0');
    -- per-slot field constants (the copy loads them)
    type t_u2 is array (0 to NSLOT - 1) of unsigned(1 downto 0);
    signal c_cls : t_u2 := (others => (others => '0'));
    signal c_lvl, c_en : std_logic_vector(0 to NSLOT - 1) := (others => '0');

    -- [M] composite
    type t_u6 is array (0 to 4) of unsigned(5 downto 0);
    signal s9_sg : t_u6 := (others => (others => '0'));
    type t_u3 is array (0 to 4) of unsigned(2 downto 0);
    signal s9_ch, s9_cn : t_u3 := (others => (others => '0'));
    signal s9_fh, s9_fn : std_logic_vector(0 to 4) := (others => '0');
    signal s9_co, s9_ci, s9_cn2 : unsigned(3 downto 0) := (others => '0');
    signal s9_in : std_logic_vector(0 to 2) := (others => '0');
    signal s9_near : std_logic := '0';
    signal s10_sa, s10_sb : unsigned(6 downto 0) := (others => '0');
    signal s10_sc : unsigned(5 downto 0) := (others => '0');
    signal s10_cl : unsigned(2 downto 0) := (others => '0');
    signal s10_co, s10_ci, s10_cn : unsigned(3 downto 0) := (others => '0');
    signal s10_in : std_logic_vector(0 to 2) := (others => '0');
    signal s10_near : std_logic := '0';
    signal s11_s : unsigned(6 downto 0) := (others => '0');
    signal s11_cl : unsigned(2 downto 0) := (others => '0');
    signal s11_co, s11_ci, s11_cn : unsigned(3 downto 0) := (others => '0');
    signal s11_rg : unsigned(1 downto 0) := (others => '0');
    signal s12_abc : signed(11 downto 0) := (others => '0');
    signal s13_abc, s14_abc : signed(11 downto 0) := (others => '0');
    signal s14_y : signed(11 downto 0) := (others => '0');
    signal s15_y, s16_y : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_l8  : unsigned(7 downto 0) := (others => '0');
    type t_tb is array (0 to 255) of unsigned(15 downto 0);
    signal tb_ram : t_tb := (others => (others => '0'));
    attribute no_rw_check of tb_ram : signal is true;
    signal tb_q, tb_wd : unsigned(15 downto 0) := (others => '0');
    signal tb_wa, tf_i : unsigned(7 downto 0) := (others => '0');
    signal tb_we, tf_run : std_logic := '0';
    signal tf_acc : unsigned(29 downto 0) := (others => '0');
    signal tf_d   : unsigned(23 downto 0) := (others => '0');
    signal s_p, s_sfirst : unsigned(12 downto 0) := (others => '0');
    signal s_sv, s_sv1, s_svp, s_svd, s_bv, s_ev : std_logic := '0';
    signal s_col, s_col1, s_colp, s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    type t_bx is array (0 to 255) of unsigned(15 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(15 downto 0) := (others => '0');
    signal s_bcnt : unsigned(7 downto 0) := (others => '0');
    signal s_bsum : unsigned(18 downto 0) := (others => '0');
    signal s_eb  : unsigned(12 downto 0) := (others => '0');
    signal s_eq  : unsigned(15 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal c_a, c_b, s_Ls : unsigned(12 downto 0) := (others => '0');
    signal c_new : std_logic := '0';

    -- [F] output conditioning
    signal s19_y, s20_y, s21_y, s22_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s22_ya : unsigned(11 downto 0) := (others => '0');
    signal s23_y, s24_y, s25_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s17_y, s18_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_ST_TAB(to_integer(r_ex_a));
            yr_q   <= C_ST_Y(to_integer(yr_a));
        end if;
    end process p_rom;

    -- the parameter RAM: the engine writes during the field, the copy reads at the field start (never together)
    p_prram : process(clk)
    begin
        if rising_edge(clk) then
            pr_q <= pr_ram(to_integer(pr_ra));
            if pr_we = '1' then pr_ram(to_integer(e_pa)) <= std_logic_vector(e_acc(31 downto 0)); end if;
        end if;
    end process p_prram;

    -- the line RAMs: the copy writes the field's start values, the line engine steps them (a different address
    -- from the one it reads in the same clock)
    p_lram : process(clk)
    begin
        if rising_edge(clk) then
            lv_q <= lv_ram(to_integer(l_ra));
            lu_q <= lu_ram(to_integer(l_ra));
            if lv_we = '1' then lv_ram(to_integer(l_wa)) <= lv_wd; end if;
            if lu_we = '1' then lu_ram(to_integer(l_wa)) <= lu_wd; end if;
        end if;
    end process p_lram;

    -- the camera's gain table: written in the vblank zone only, read during active lines
    p_tbram : process(clk)
    begin
        if rising_edge(clk) then
            tb_q <= tb_ram(to_integer(s_l8));
            if tb_we = '1' then tb_ram(to_integer(tb_wa)) <= tb_wd; end if;
        end if;
    end process p_tbram;

    -- box delay: read N samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_svp = '1' then bx_ram(to_integer(bx_wp)) <= resize(s_p, 16); end if;
        end if;
    end process p_bxram;

    -- luma line store (half horizontal resolution)
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q <= lb_ram(to_integer(s_colp(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_z : unsigned(3 downto 0);
        variable v_f : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            case to_integer(unsigned(s_tid)) is
                when 0 | 4 | 8 | 12 => s_xsr <= to_unsigned(8, 4);
                when 5 | 6 | 14     => s_xsr <= to_unsigned(6, 4);
                when others         => s_xsr <= to_unsigned(4, 4);
            end case;

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

            -- [B] Count: 10 steps of 102.4 counts; a new step is taken only 8 counts inside it, except at the two
            -- ends of the range (a knob resting at 0 or 1023 reaches the first / last step)
            s_k4 <= unsigned(registers_in(3));
            z_p  <= resize(s_k4 & "000", 14) + resize(s_k4 & '0', 14);         -- k4 * 10
            v_z := z_p(13 downto 10);
            v_f := z_p(9 downto 0);
            z_kz <= v_z;
            if (v_f >= 80 and v_f <= 943) or (v_z = 0 and v_f <= 943) or (v_z = 9 and v_f >= 80) then
                z_ok <= '1';
            else
                z_ok <= '0';
            end if;

            -- [A][B] per-field latches and the mode constants
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_nest  <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                if z_ok = '1' and z_kz <= 9 then s_zn <= z_kz; end if;
                c_xs <= s_xsr;
                case to_integer(s_xsr) is
                    when 8 =>
                        s_oxs <= to_unsigned(204, 9); c_xpre <= to_signed(-2892, 13); c_dd <= to_unsigned(128, 8);
                        c_hw <= to_unsigned(8, 5); c_hwf <= to_unsigned(12, 5);
                    when 6 =>
                        s_oxs <= to_unsigned(153, 9); c_xpre <= to_signed(-3849, 13); c_dd <= to_unsigned(72, 8);
                        c_hw <= to_unsigned(11, 5); c_hwf <= to_unsigned(16, 5);
                    when others =>
                        s_oxs <= to_unsigned(102, 9); c_xpre <= to_signed(-3846, 13); c_dd <= to_unsigned(32, 8);
                        c_hw <= to_unsigned(16, 5); c_hwf <= to_unsigned(24, 5);
                end case;
                if s_xsr = 4 then s_lg6 <= '1'; else s_lg6 <= '0'; end if;
                case to_integer(unsigned(s_tid)) is
                    when 0 =>                                   -- 480i (field line 4.4375 virtual px)
                        c_ys2 <= to_unsigned(71, 7); c_vv <= to_unsigned(10082, 14); c_y0 <= to_signed(-8591, 15);
                        s_oys <= to_unsigned(537, 10); s_ish <= to_unsigned(2, 3);
                    when 4 =>                                   -- 576p
                        c_ys2 <= to_unsigned(30, 7); c_vv <= to_unsigned(1800, 14); c_y0 <= to_signed(-8625, 15);
                        s_oys <= to_unsigned(272, 10); s_ish <= to_unsigned(3, 3);
                    when 8 =>                                   -- 576i
                        c_ys2 <= to_unsigned(60, 7); c_vv <= to_unsigned(7200, 14); c_y0 <= to_signed(-8610, 15);
                        s_oys <= to_unsigned(453, 10); s_ish <= to_unsigned(2, 3);
                    when 12 =>                                  -- 480p
                        c_ys2 <= to_unsigned(36, 7); c_vv <= to_unsigned(2592, 14); c_y0 <= to_signed(-8622, 15);
                        s_oys <= to_unsigned(226, 10); s_ish <= to_unsigned(3, 3);
                    when 5 | 6 | 14 =>                          -- 720p
                        c_ys2 <= to_unsigned(24, 7); c_vv <= to_unsigned(1152, 14); c_y0 <= to_signed(-8628, 15);
                        s_oys <= to_unsigned(181, 10); s_ish <= to_unsigned(4, 3);
                    when 1 | 2 | 10 =>                          -- 1080i
                        c_ys2 <= to_unsigned(32, 7); c_vv <= to_unsigned(2048, 14); c_y0 <= to_signed(-8624, 15);
                        s_oys <= to_unsigned(242, 10); s_ish <= to_unsigned(3, 3);
                    when others =>                              -- 1080p
                        c_ys2 <= to_unsigned(16, 7); c_vv <= to_unsigned(512, 14); c_y0 <= to_signed(-8632, 15);
                        s_oys <= to_unsigned(121, 10); s_ish <= to_unsigned(4, 3);
                end case;
            end if;
            g_wm1 <= s_w - 1;

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
    -- [C] THE PER-FIELD ENGINE (Hardline flute's, with a two-deep return
    -- stack).  Started from address 0 after every field-start copy (so a
    -- stalled or garbled run cannot outlive a field); three clocks per
    -- micro-op, serial multiply (18 steps) and shift.  It runs through the
    -- field (about 50 .. 90 k clocks) and writes the next field's circles
    -- into the parameter RAM.
    ------------------------------------------------------------------------
    with e_irf(3 downto 0) select w_in <=
        signed(resize(c_xs, 17))         when x"8",
        signed(resize(c_ys2, 17))        when x"9",
        resize(c_xpre, 17)               when x"A",
        signed(resize(r_ex_q, 17))       when x"B",
        resize(c_y0, 17)                 when x"C",
        signed(resize(s_zn, 17))         when x"D",
        (others => '0')                  when x"E" | x"F",
        signed(resize(unsigned(registers_in(to_integer(unsigned(e_irf(2 downto 0))))), 17)) when others;

    e_out <= '1' when e_run = '1' and e_ph = "10" and x_out = '1' else '0';
    pr_we <= '1' when e_out = '1' and e_irf(1 downto 0) = "10" else '0';

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

            if cp_done = '1' then
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
                            when OP_IN   => e_b <= resize(w_in, 37);
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
                            when "010"  => v_cond := s_nest;
                            when others => v_cond := s_glide;
                        end case;
                        v_cond := v_cond xor v_arg(3);
                        if v_op = OP_SKIP and v_cond = '1' then e_pc <= e_pc + 2;
                        elsif v_op = OP_JMP then e_pc <= unsigned(v_arg(9 downto 0));
                        elsif v_op = OP_CALL then
                            e_pc <= unsigned(v_arg(9 downto 0)); e_lr <= e_pc + 1; e_lr2 <= e_lr;
                        elsif v_op = OP_SPC and v_arg(1 downto 0) = "00" then
                            e_pc <= e_lr; e_lr <= e_lr2;
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
                e_ir <= C_ST_UC(to_integer(e_pc));
            end if;

            -- the engine's outputs: the table address, the parameter RAM's address (its write port: p_prram)
            if e_out = '1' then
                case e_irf(1 downto 0) is
                    when "00" => r_ex_a <= unsigned(e_acc(8 downto 0));
                    when "01" => e_pa <= unsigned(e_acc(6 downto 0));
                    when others => e_pa <= e_pa + 1;
                end case;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- [Q] THE FIELD-START COPY (words 0 .. 98 of the parameter RAM: per slot V0,
    -- U0, CTL; then dq0, GT, SC) and THE LINE ENGINE (after the copy and at
    -- every line end: V += U, U += vv for every slot, and each slot's DDA loads
    -- the new V; the DDAs then hold until the line starts)
    ---------------------------------------------------------------------------
    p_copy : process(clk)
        variable v_a : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            cp_done <= '0'; tf_go <= '0'; le_go <= '0';
            if s_zgo = '1' then
                cp_run <= '1'; pr_ra <= (others => '0');
            elsif cp_run = '1' then
                if pr_ra = 98 then cp_run <= '0'; else pr_ra <= pr_ra + 1; end if;
            end if;
            cp_v1 <= cp_run; cp_a1 <= pr_ra;
            cp_v2 <= cp_v1;  cp_a2 <= cp_a1;
            pr_qf <= pr_q;
            if cp_v2 = '1' and cp_v1 = '0' then
                cp_done <= '1'; tf_go <= '1'; le_go <= '1';
            end if;
            v_a := cp_a2;
            if cp_v2 = '1' then
                if v_a = 96 then c_dq0 <= signed(pr_qf(20 downto 0)); end if;
                if v_a = 97 then c_gt <= unsigned(pr_qf(13 downto 0)); end if;
                if v_a = 98 then c_sc <= unsigned(pr_qf(3 downto 0)); end if;
            end if;
        end if;
    end process p_copy;

    cp_ctl_we <= '1' when cp_v2 = '1' and cp_a2 < 60 and cp_a2(1 downto 0) = "10" else '0';
    cp_ctl_j  <= cp_a2(6 downto 2);
    cp_ctl    <= pr_qf;

    p_line : process(clk)
    begin
        if rising_edge(clk) then
            -- trigger at every line end, once all the DDAs have stepped past the last pixel
            s_gq <= s_gav_sr(5);
            if le_go = '1' or (s_gq = '1' and s_gav_sr(5) = '0') then
                le_act <= '1'; le_j <= (others => '0');
            elsif le_act = '1' then
                if le_j = NSLOT - 1 then le_act <= '0'; else le_j <= le_j + 1; end if;
            end if;
            -- issue (t), address register (t + 1), RAM out (t + 2), fabric (t + 3), new V / write back (t + 4)
            l_ra <= le_j;
            le_a1 <= le_act; le_j1 <= le_j;
            le_a2 <= le_a1;  le_j2 <= le_j1;
            le_a3 <= le_a2;  le_j3 <= le_j2;
            lv_f <= lv_q; lu_f <= lu_q;
            le_nv <= signed(lv_f) + signed(lu_f);
            le_ld <= le_a3; le_jl <= le_j3;
        end if;
    end process p_line;

    -- the line RAMs' write ports: the copy's start values (V at word 4j, U at 4j + 1), or the line engine's step
    -- (written at t + 3, while the read is three slots ahead)
    lv_we <= '1' when (cp_v2 = '1' and cp_a2 < 60 and cp_a2(1 downto 0) = "00") or le_a3 = '1' else '0';
    lu_we <= '1' when (cp_v2 = '1' and cp_a2 < 60 and cp_a2(1 downto 0) = "01") or le_a3 = '1' else '0';
    l_wa  <= cp_a2(6 downto 2) when cp_v2 = '1' else le_j3;
    lv_wd <= pr_qf when cp_v2 = '1' else std_logic_vector(signed(lv_f) + signed(lu_f));
    lu_wd <= pr_qf when cp_v2 = '1' else std_logic_vector(signed(lu_f) + signed(resize(c_vv, 32)));

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
    -- [P] the shared horizontal quadratic about the great circle's centre
    -- (dq(j) = dq0 + j dd, stage 1), the camera's step dC(j) = (s(j) - s(j-1))
    -- << SC (s(-1) = 0), the chain bus dqc = dq + dC and the great circle's bus
    -- dqo = dq - 8 dC (stage 2), the DDAs' run window (stages 3 ..), and the
    -- camera's slope class for the great circle
    ---------------------------------------------------------------------------
    p_bus : process(clk)
        variable v_dc : signed(13 downto 0);
        variable v_s1 : signed(22 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then q_dqx <= c_dq0;
            else q_dqx <= q_dqx + signed(resize(c_dd, 21)); end if;
            s_st1 <= g_start;
            s1_cs <= s_Ls; s1_csd <= s1_cs;
            if s_st1 = '1' then v_dc := signed(resize(s1_cs, 14));
            else v_dc := signed(resize(s1_cs, 14)) - signed(resize(s1_csd, 14)); end if;
            case to_integer(c_sc) is
                when 4      => v_s1 := shift_left(resize(v_dc, 23), 4);
                when 5      => v_s1 := shift_left(resize(v_dc, 23), 5);
                when 6      => v_s1 := shift_left(resize(v_dc, 23), 6);
                when 7      => v_s1 := shift_left(resize(v_dc, 23), 7);
                when others => v_s1 := shift_left(resize(v_dc, 23), 8);
            end case;
            s2_dqc <= resize(q_dqx, 23) + v_s1;
            -- the DDAs step for pixels 0 .. W + 1
            s_run <= s_gav_sr(0) or s_gav_sr(2);
        end if;
    end process p_bus;

    ---------------------------------------------------------------------------
    -- [P] PIXEL PATH, one slot per circle.  The DDA P = |z - c|^2 - r^2 (q =
    -- units^2, 28 bits, wrapping: the chain is drawn inside the great circle
    -- only) steps by dk = bus - delta (stage 3) from the line's load (stage 4);
    -- the edge distance in 1/8 px is the window P(sh + 9 .. sh + 4) (5), x 3/4
    -- when m (6); Bold 4 - d, Fine hw + 4 - |d| (7); coverage clamped to 0 .. 8,
    -- or full / empty where the window saturates (8).  Slot 0 (the great circle) runs on its
    -- own bus with the camera's slope class added to its shift.
    ---------------------------------------------------------------------------
    g_slot : for j in 0 to NSLOT - 1 generate
        signal sl_dl : signed(17 downto 0) := (others => '0');
        signal sl_sh : unsigned(3 downto 0) := (others => '0');
        signal sl_m, sl_en : std_logic := '0';
        signal b_dk : signed(24 downto 0) := (others => '0');
        signal b_P  : signed(27 downto 0) := (others => '0');
        signal s5_w : signed(5 downto 0) := (others => '0');
        signal s5_sat, s5_sg, s5_in, s6_sat, s6_sg, s6_in, s7_sat, s7_full, s7_in : std_logic := '0';
        signal s6_d : signed(5 downto 0) := (others => '0');
        signal s7_t : signed(7 downto 0) := (others => '0');
        signal s7_nr : std_logic := '0';
    begin
        p_slot : process(clk)
            variable v_p   : signed(23 downto 0);
            variable v_x1  : signed(12 downto 0);
            variable v_x2  : signed(8 downto 0);
            variable v_x3  : signed(6 downto 0);
            variable v_w   : signed(5 downto 0);
            variable v_sat, v_g : std_logic;
            variable v_s   : unsigned(3 downto 0);
            variable v_b   : unsigned(7 downto 0);
            variable v_ln  : std_logic;
            variable v_hw  : unsigned(7 downto 0);
        begin
            if rising_edge(clk) then
                -- the field's constants (the copy)
                if cp_ctl_we = '1' and cp_ctl_j = j then
                    sl_dl <= signed(cp_ctl(27 downto 10));
                    sl_sh <= unsigned(cp_ctl(3 downto 0));
                    sl_m  <= cp_ctl(4);
                    if cp_ctl(9 downto 7) /= "000" then sl_en <= '1'; else sl_en <= '0'; end if;
                end if;
                -- 3: the step
                b_dk <= resize(s2_dqc - resize(sl_dl, 23), 25);
                -- 4: the DDA (loaded by the line engine, held until the line starts)
                if le_ld = '1' and le_jl = j then
                    b_P <= le_nv(31 downto 4);
                elsif s_run = '1' then
                    b_P <= b_P + resize(b_dk, 28);
                end if;
                -- 5: the window (a four-level shifter); saturated when a bit it drops off the top differs from the sign
                v_p := b_P(27 downto 4);
                v_s := sl_sh;
                v_g := v_p(23);
                v_sat := '0';
                if v_s(3) = '1' then
                    v_x1 := v_p(20 downto 8);
                    if v_p(22 downto 21) /= (v_g & v_g) then v_sat := '1'; end if;
                else
                    v_x1 := v_p(12 downto 0);
                    for i in 13 to 22 loop
                        if v_p(i) /= v_g then v_sat := '1'; end if;
                    end loop;
                end if;
                if v_s(2) = '1' then v_x2 := v_x1(12 downto 4);
                else
                    v_x2 := v_x1(8 downto 0);
                    for i in 9 to 12 loop
                        if v_x1(i) /= v_g then v_sat := '1'; end if;
                    end loop;
                end if;
                if v_s(1) = '1' then v_x3 := v_x2(8 downto 2);
                else
                    v_x3 := v_x2(6 downto 0);
                    if v_x2(8 downto 7) /= (v_g & v_g) then v_sat := '1'; end if;
                end if;
                if v_s(0) = '1' then v_w := v_x3(6 downto 1);
                else
                    v_w := v_x3(5 downto 0);
                    if v_x3(6) /= v_g then v_sat := '1'; end if;
                end if;
                if v_w(5) /= v_g then v_sat := '1'; end if;
                s5_w <= v_w; s5_sat <= v_sat; s5_sg <= v_p(23); s5_in <= b_P(27);
                -- 6: the edge distance (1/8 px), x 3/4 when m
                if sl_m = '1' then s6_d <= s5_w - shift_right(s5_w, 2); else s6_d <= s5_w; end if;
                s6_sat <= s5_sat; s6_sg <= s5_sg; s6_in <= s5_in;
                -- 7: Bold 4 - d, Fine (and the great circle's Ink frame) hw + 4 - |d| (|d| as the one's
                --    complement); saturated: Bold full inside / empty outside, lines empty
                if j = 0 then
                    v_ln := s_fine or s_ink;
                    if s_fine = '1' then v_hw := resize(c_hw, 8) + 4; else v_hw := resize(c_hwf, 8) + 4; end if;
                else
                    v_ln := s_fine; v_hw := resize(c_hw, 8) + 4;
                end if;
                if v_ln = '0' then v_hw := to_unsigned(4, 8); end if;
                if v_ln = '1' and s6_d(5) = '1' then v_b := not unsigned(resize(s6_d, 8));
                else v_b := unsigned(resize(s6_d, 8)); end if;
                s7_t <= signed(f_addsub(v_hw, v_b, '1'));
                s7_sat <= s6_sat; s7_full <= s6_sg and not v_ln;
                if s6_sat = '1' then s7_nr <= s6_sg;
                elsif s6_d < 4 then s7_nr <= '1'; else s7_nr <= '0'; end if;
                s7_in <= s6_in;
                -- 8: coverage 0 .. 8
                if sl_en = '0' then s8_cov(j) <= (others => '0');
                elsif s7_sat = '1' then
                    if s7_full = '1' then s8_cov(j) <= to_unsigned(8, 4); else s8_cov(j) <= (others => '0'); end if;
                elsif s7_t < 0 then s8_cov(j) <= (others => '0');
                elsif s7_t > 8 then s8_cov(j) <= to_unsigned(8, 4);
                else s8_cov(j) <= unsigned(s7_t(3 downto 0)); end if;
                s8_in(j) <= s7_in and sl_en;
                s8_near(j) <= s7_nr;
            end if;
        end process p_slot;
    end generate g_slot;

    -- the per-slot class and level (the copy)
    p_cls : process(clk)
    begin
        if rising_edge(clk) then
            if cp_ctl_we = '1' then
                for j in 0 to NSLOT - 1 loop
                    if cp_ctl_j = j then
                        c_cls(j) <= unsigned(cp_ctl(6 downto 5));
                        if cp_ctl(9 downto 7) = "101" then c_lvl(j) <= '1'; else c_lvl(j) <= '0'; end if;
                    end if;
                end loop;
                if cp_ctl_j = 2 then
                    if cp_ctl(9 downto 7) = "011" then c_nin <= '1'; else c_nin <= '0'; end if;
                end if;
            end if;
        end if;
    end process p_cls;

    ---------------------------------------------------------------------------
    -- [M] COMPOSITE (stages 9 .. 14).  Chain sum S over slots 2 .. 13 (3 groups
    -- of 4; slot 2 is the nested inner circle in Nested); the chain class / level from the first slot covering half the
    -- pixel, else the first covering any; S, the inner coverages gated to the
    -- great circle (it bounds the chain's wrap).  Tone (stage 12 address, 13
    -- ROM, 14 fabric): Grey Bold = A(cov_out) + B(cov_in) + C(cov_nin) +
    -- ROM(level, class, min(S, 8)); Ink Bold = ROM(max(tri(S + in + nin),
    -- cov_out)); Fine = ROM(region, min(S + in + nin + out, 8)).
    ---------------------------------------------------------------------------
    p_comp : process(clk)
        variable v_s : unsigned(5 downto 0);
        variable v_h, v_n : std_logic;
        variable v_ch, v_cn : unsigned(2 downto 0);
        variable v_e : unsigned(7 downto 0);
        variable v_l, v_tr : unsigned(3 downto 0);
        variable v_f : unsigned(7 downto 0);
        variable v_a : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 9: group sums and group priorities (groups of 4 chain slots; slot 2 counts only when it is a chain disc)
            for g in 0 to 2 loop
                v_s := (others => '0'); v_h := '0'; v_n := '0'; v_ch := (others => '0'); v_cn := (others => '0');
                for i in 3 downto 0 loop
                    if not (g = 0 and i = 0 and c_nin = '1') then
                        v_s := v_s + resize(s8_cov(2 + 4 * g + i), 6);
                        if s8_cov(2 + 4 * g + i)(3 downto 2) /= "00" then
                            v_h := '1'; v_ch := c_lvl(2 + 4 * g + i) & c_cls(2 + 4 * g + i);
                        end if;
                        if s8_cov(2 + 4 * g + i) /= 0 then
                            v_n := '1'; v_cn := c_lvl(2 + 4 * g + i) & c_cls(2 + 4 * g + i);
                        end if;
                    end if;
                end loop;
                s9_sg(g) <= v_s; s9_fh(g) <= v_h; s9_fn(g) <= v_n; s9_ch(g) <= v_ch; s9_cn(g) <= v_cn;
            end loop;
            s9_co <= s8_cov(0); s9_ci <= s8_cov(1);
            if c_nin = '1' then s9_cn2 <= s8_cov(2); else s9_cn2 <= (others => '0'); end if;
            s9_in(0) <= s8_in(0); s9_in(1) <= s8_in(1); s9_in(2) <= s8_in(2) and c_nin;
            s9_near <= s8_near(0);
            -- 10
            s10_sa <= resize(s9_sg(0), 7) + resize(s9_sg(1), 7);
            s10_sc <= s9_sg(2);
            if s9_fh(0) = '1' then s10_cl <= s9_ch(0);
            elsif s9_fh(1) = '1' then s10_cl <= s9_ch(1);
            elsif s9_fh(2) = '1' then s10_cl <= s9_ch(2);
            elsif s9_fn(0) = '1' then s10_cl <= s9_cn(0);
            elsif s9_fn(1) = '1' then s10_cl <= s9_cn(1);
            else s10_cl <= s9_cn(2); end if;
            s10_co <= s9_co; s10_in <= s9_in;
            if s9_near = '1' then s10_ci <= s9_ci; s10_cn <= s9_cn2; else s10_ci <= (others => '0'); s10_cn <= (others => '0'); end if;
            s10_near <= s9_near;
            -- 11
            if s10_near = '1' then s11_s <= s10_sa + resize(s10_sc, 7); else s11_s <= (others => '0'); end if;
            s11_cl <= s10_cl; s11_co <= s10_co; s11_ci <= s10_ci; s11_cn <= s10_cn;
            if s10_in(0) = '0' then s11_rg <= "00";
            elsif s10_in(1) = '0' then s11_rg <= "01";
            elsif s10_in(2) = '0' then s11_rg <= "10";
            else s11_rg <= "11"; end if;
            -- 12: the tone's address and the Grey Bold terms
            v_e := resize(s11_s, 8) + resize(s11_ci, 8) + resize(s11_cn, 8);
            if s_fine = '0' and s_ink = '0' then
                if s11_s > 8 then v_l := to_unsigned(8, 4); else v_l := s11_s(3 downto 0); end if;
                yr_a <= "00" & s11_cl & v_l;
                v_a := C_ST_A(to_integer(s11_co)) + C_ST_B(to_integer(s11_ci)) + C_ST_C(to_integer(s11_cn));
                s12_abc <= v_a;
            elsif s_fine = '0' then
                if v_e(3) = '1' then v_tr := to_unsigned(8, 4) - resize(v_e(2 downto 0), 4);
                else v_tr := resize(v_e(2 downto 0), 4); end if;
                if s11_co > v_tr then v_tr := s11_co; end if;
                yr_a <= "01000" & v_tr;
                s12_abc <= (others => '0');
            else
                v_f := v_e + resize(s11_co, 8);
                if v_f > 8 then v_l := to_unsigned(8, 4); else v_l := v_f(3 downto 0); end if;
                if s_ink = '1' then yr_a <= "11000" & v_l;
                else yr_a <= "10" & '0' & s11_rg & v_l; end if;
                s12_abc <= (others => '0');
            end if;
            -- 13: ROM out; 14: fabric register and the sum
            s13_abc <= s12_abc;
            s14_abc <= s13_abc;
            s14_y <= yr_q;
            v_a := s14_y + s14_abc;
            s15_y <= unsigned(v_a(9 downto 0));
            -- 16: black outside the rendered pixels (0 .. W + 1)
            if s_gav_sr(13) = '1' or s_gav_sr(15) = '1' then s16_y <= s15_y; else s16_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_comp;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> the per-field table
    -- T(l) = min(GT l^2 >> 16, 8191) -> box of 64 (1080) or 32 samples, edge-
    -- replicated and reloaded at every line start -> vertical one-pole in the
    -- line store (half horizontal resolution, Q13.3).  The pixel path takes the
    -- one-pole's stream as it is written (the camera is never shown): s, and
    -- |its x step| + |its y step| for the great circle's anti-aliasing.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(12 downto 0);
        variable v_d : signed(16 downto 0);
        variable v_u, v_n : signed(16 downto 0);
        variable v_df : unsigned(12 downto 0);
        variable v_wc : unsigned(13 downto 0);
        variable v_mn : unsigned(13 downto 0);
        variable v_ta : unsigned(29 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting; both phases restart in the
            -- vblank zone from this field's drift (the Hardline dazzle fix)
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

            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_l8 <= v_s;
            elsif v_l > 16 then s_l8 <= v_l - 16;
            else s_l8 <= (others => '0'); end if;
            s_sv <= data_in.avid;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;
            s_sv1 <= s_sv; s_col1 <= s_col;
            s_p <= tb_q(12 downto 0);
            s_svp <= s_sv1; s_colp <= s_col1;
            s_svd <= s_svp;

            if s_svp = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else
                bx_wp <= (others => '0');
                if s_lg6 = '1' then bx_rp <= to_unsigned(193, 8); else bx_rp <= to_unsigned(225, 8); end if;
            end if;
            if (s_lg6 = '1' and s_bcnt(7 downto 6) = "00") or (s_lg6 = '0' and s_bcnt(7 downto 5) = "000") then v_tap := s_sfirst;
            else v_tap := bx_q(12 downto 0); end if;
            if s_svp = '1' and s_svd = '0' then
                s_sfirst <= s_p;
                if s_lg6 = '1' then s_bsum <= resize(s_p & "000000", 19); else s_bsum <= resize(s_p & "00000", 19); end if;
                s_bcnt <= to_unsigned(1, 8);
            elsif s_svp = '1' then
                s_bsum <= s_bsum + s_p - v_tap;
                if (s_lg6 = '1' and s_bcnt(7 downto 6) = "00") or (s_lg6 = '0' and s_bcnt(7 downto 5) = "000") then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bv <= s_svp; s_bcol <= s_colp;

            if s_lg6 = '1' then s_eb <= s_bsum(18 downto 6); else s_eb <= s_bsum(17 downto 5); end if;
            s_eq <= lb_q;
            s_ev <= s_bv and s_bcol(0); s_ecol <= s_bcol;
            v_d := signed(resize(s_eb & "000", 17)) - signed(resize(s_eq, 17));
            case to_integer(s_ish) is
                when 2      => v_u := shift_right(v_d, 2);
                when 3      => v_u := shift_right(v_d, 3);
                when others => v_u := shift_right(v_d, 4);
            end case;
            lb_wd <= unsigned(signed(s_eq) + v_u(15 downto 0));
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            c_new <= lb_we;
            if lb_we = '1' then
                c_b <= c_a; c_a <= lb_wd(15 downto 3);
            end if;
            v_mn := resize(c_a, 14) + resize(c_b, 14);
            if c_new = '1' then s_Ls <= v_mn(13 downto 1); else s_Ls <= c_a; end if;

            -- the table fill (256 clocks, started after the field-start copy): T(l) = min(GT l^2 >> 16, 8191)
            -- (Osc: 2 GT, so the lattice bends the circles about twice as hard as a camera's mean level)
            tb_we <= '0';
            if tf_go = '1' then
                tf_run <= '1'; tf_i <= (others => '0'); tf_acc <= (others => '0');
                if s_src = '1' then tf_d <= resize(c_gt & '0', 24); else tf_d <= resize(c_gt, 24); end if;
            elsif tf_run = '1' then
                tb_we <= '1'; tb_wa <= tf_i;
                if tf_acc(29) = '1' or tf_acc(28 downto 16) = "1111111111111" then tb_wd <= to_unsigned(8191, 16);
                else tb_wd <= resize(tf_acc(28 downto 16), 16); end if;
                v_ta := tf_acc + resize(tf_d, 30);
                tf_acc <= v_ta;
                if s_src = '1' then tf_d <= tf_d + resize(c_gt & "00", 24); else tf_d <= tf_d + resize(c_gt & '0', 24); end if;
                tf_i <= tf_i + 1;
                if tf_i = 255 then tf_run <= '0'; end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: (padding 17, 18) luma [1 2 1] / 4 (19 .. 23),
    -- 13/16 toward black (24), blanking gate (25).  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s17_y <= s16_y; s18_y <= s17_y; s19_y <= s18_y;
            s20_y <= s19_y; s21_y <= s20_y;
            s22_ya <= resize(s19_y, 12) + resize(s21_y, 12) + 2;
            s22_yb <= s20_y;
            v_y := s22_ya + shift_left(resize(s22_yb, 12), 1);
            s23_y <= v_y(11 downto 2);
            if s23_y > 64 then v_dl := s23_y - 64; else v_dl := (others => '0'); end if;
            s24_y <= s23_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if s_gav_sr(C_LAT - 2) = '1' then s25_y <= s24_y; else s25_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s25_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture steiner;
