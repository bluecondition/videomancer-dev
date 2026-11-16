-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Eclipse v0.1.0 -- Vellum.  After Kandinsky's "Several Circles", Albers' overlapping translucent planes and a
-- Venn diagram drawn by a Bauhaus printer: a few large discs, each a translucent film of light on black, orbit a
-- formation point at their own rates, so the overlaps slide and change shape like slow eclipses.
--
-- Disc i (0 .. n - 1, n = 2 .. 4) has radius R_i = S k_i (k = 1, .88, .78, .70) and centre
--   c_i = F + rho (1.2 cos phi_i, -0.8 sin phi_i),   phi_i = theta_i + 0.1 + i / n,   rho = S sigma(P12, n)
-- and covers a pixel by its edge distance d = (r^2 - R^2) / 2R (one-pixel anti-aliasing, no square root):
--   Flat   Bold: clamp(1/2 - (d - swell)),  Fine: clamp(1/2 + (w/2 - |d - swell|))   (swell = the camera)
--   Rings  min(the disc edge, a ring band) on discs 0 .. 2: Fresnel zone-plate rings, phase d / 24 v px + camera
--          (rings 24 v px apart at the rim, opening toward the centre: lenses)
-- Grey adds the coverages as films (1/n each; 1/max(2, n - 1) for Fine, 1/2 for Rings), Ink takes them even-odd
-- (a triangle of the sum, so the XOR is anti-aliased too).
--
--   K1  X        formation point, +-960 virtual px        K2  Y      formation point, +-540 virtual px
--   K3  Scale    S = 185 .. 1090 virtual px, exponential (the orbits scale with it)
--   K4  Discs    2 / 3 / 4 (stepped, hysteresis)
--   K5  Drift    orbit speed, bipolar: +-24 still, 2^-12 .. 2^-5 turn per field (rates +1, -3/4, +2/3, -1/2)
--   K6  Camera   0 .. 100 %: Flat swells the disc edges by 500 v px * K6 * luma^2; Rings adds 3 rings to the phase
--   S7  Glide    On: the knobs ease 1/16 of the gap per field (Drift's speed 1/8); Off: no easing
--   S8  Fill     Flat / Rings            S9  Tone   Grey / Ink           S10 Weight  Bold / Fine
--   S11 Source   Video / Osc (the shell's drifting triangle lattice, about twice the camera's level)
--   P12 SPREAD   orbit radius: nearly concentric (crescents) .. the Venn (50 %) .. just touching (100 %)
--
-- Sections (the integer model .vellum_work/rb_eclipse/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         switches, the Discs steps, the mode constants
--   [C] FIELD ENGINE                     Hardline flute's microcoded engine (eclipse.py holds the program and its
--                                        emulator): holds + glides, scale, spread, formation, speed, camera
--                                        gain; per disc the orbit angle, sin / cos, centre, radius, the scales
--                                        and every DDA start value
--   [V] VERTICAL DDAs                    per disc r^2 - R^2 at the line's pre-pixel (Q2 of q^2, q = 1/4 h), exact
--   [P] PIXEL PATH (x 4)                 r^2 - R^2 DDA -> normalised edge distance -> edge / ring coverage
--                                        (17 steps) -> sum -> tone ROM
--   [D] SOURCE FIELD                     luma (or Osc) -> per-field K6 * l^2 table -> box -> vertical one-pole
--   [F] OUTPUT CONDITIONING              luma [1 2 1] / 4, 13/16 toward black, blanking gate; U = V = 512
--
-- Stage numbering (the Hardline shell's): a register "at stage S" holds pixel n at T + S + n (T = the g_start
-- clock).  Pixels 0 .. W + 1 are rendered; output column j is the [1 2 1] filter over pixels j .. j + 2 and lands
-- on screen column j + 4, so pixel m sits at x = xq + (m + 1) xs q (q = 1/4 h, h = half a virtual px).
-- s_gav_sr(S - 3) flags pixels 0 .. W - 1 on their way into a stage-S register.  One latency (24) in every mode.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.eclipse_pkg.all;
use work.all;

architecture eclipse of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks
    -- (the knob defaults, the disc sizes, orbit rates and phase offsets live in the engine's microprogram and its
    -- register file at power-up: eclipse.py)

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
    function f_one(e : unsigned(2 downto 0)) return unsigned is
    begin
        return shift_left(to_unsigned(1, 21), to_integer(e));
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
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zgd  : std_logic_vector(1 downto 0) := "00";
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_st0  : std_logic := '0';

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

    -- [B] switches, discs
    signal s_glide : std_logic := '1';
    signal s_rings, s_ink, s_fine, s_src : std_logic := '0';
    signal s_zn : unsigned(1 downto 0) := to_unsigned(1, 2);       -- discs - 2
    signal z_up, z_dn : std_logic := '0';
    signal z_kz : unsigned(1 downto 0) := to_unsigned(1, 2);
    signal s_k4 : unsigned(9 downto 0) := (others => '0');

    -- [B] mode constants (per field)
    signal c_ysd  : unsigned(10 downto 0) := to_unsigned(256, 11);  -- field line in Q8 virtual lines (480i: 1136)
    signal c_vv   : unsigned(13 downto 0) := to_unsigned(512, 14);  -- the vertical DDA's second step (Q2 q^2)
    signal c_lpx  : unsigned(9 downto 0) := to_unsigned(256, 10);   -- 256 log2(the AA pixel in h)
    signal c_gr   : unsigned(8 downto 0) := to_unsigned(21, 9);     -- ring slope per AA pixel (ring units)
    signal c_ec   : unsigned(1 downto 0) := (others => '0');        -- ring units -> 1/16 AA px: 2^-ec ...
    signal c_mc   : std_logic := '1';                               -- ... x 3/4
    signal c_hw   : unsigned(6 downto 0) := to_unsigned(64, 7);     -- half the Fine line (4 v px), 1/16 AA px
    signal c_xq   : unsigned(10 downto 0) := to_unsigned(20, 11);   -- the pre-pixel's x, q
    signal c_xsq  : unsigned(4 downto 0) := to_unsigned(8, 5);      -- the pixel step, q
    signal c_dd   : unsigned(9 downto 0) := to_unsigned(128, 10);   -- the pixel DDA's second step
    signal c_th1, c_th2, c_th3 : unsigned(10 downto 0) := (others => '1');
    signal c_sel  : unsigned(1 downto 0) := (others => '0');
    signal s_lg6  : std_logic := '1';                               -- box: 64 samples (1080) / 32
    signal s_ish  : unsigned(2 downto 0) := to_unsigned(4, 3);      -- the vertical one-pole's shift

    -- ROMs: the engine's table (2^(i/256) + quarter sine), tone -> luma
    signal r_ex_a : unsigned(8 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal yr_q   : unsigned(9 downto 0) := to_unsigned(64, 10);

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
    signal rf_ram : t_c_ec_rfi := C_EC_RFI;
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
    signal w_in  : unsigned(15 downto 0);
    signal e_di  : unsigned(2 downto 0) := (others => '0');
    signal c_Gt  : unsigned(13 downto 0) := (others => '0');
    signal tf_go : std_logic := '0';

    -- [C][V] per-disc field constants and the vertical DDAs
    type t_u16x5 is array (0 to 4) of unsigned(15 downto 0);
    type t_s22x5 is array (0 to 3) of signed(21 downto 0);
    type t_u3x5 is array (0 to 3) of unsigned(2 downto 0);
    type t_s34x5 is array (0 to 3) of signed(33 downto 0);
    type t_s24x5 is array (0 to 3) of signed(23 downto 0);
    type t_u15x3 is array (0 to 2) of unsigned(14 downto 0);
    type t_u6x3 is array (0 to 2) of unsigned(5 downto 0);
    signal c_D0 : t_s22x5 := (others => (others => '0'));          -- the pixel DDA's first step
    signal c_e  : t_u3x5 := (others => to_unsigned(1, 3));         -- edge scale: 2^-(9 + e) ...
    signal c_m  : std_logic_vector(0 to 3) := (others => '0');     -- ... x 3/4 (disc 3)
    signal c_s  : t_u3x5 := (others => to_unsigned(3, 3));         -- ring scale: 2^-(8 + s) ...
    signal c_mu : t_u6x3 := (others => to_unsigned(48, 6));        -- ... x mu / 64 (discs 0 .. 2)
    signal c_t1, c_t2 : t_u15x3 := (others => (others => '1'));    -- the r / R classes (discs 0 .. 2)
    signal s_V  : t_s34x5 := (others => (others => '0'));          -- r^2 - R^2 at the line's pre-pixel, Q2 of q^2
    signal s_U  : t_s24x5 := (others => (others => '0'));          -- its step to the next line
    signal s_vk, s_vki : unsigned(2 downto 0) := (others => '1');  -- the shared vertical step: disc counter
    signal s_vr : signed(33 downto 0) := (others => '0');
    signal s_ur : signed(23 downto 0) := (others => '0');
    signal s_wv, s_wu, s_av : std_logic := '0';

    -- [P] shared pixel-path signals
    type t_c5 is array (0 to 3) of unsigned(4 downto 0);
    signal s11_cov : t_c5 := (others => (others => '0'));
    signal s3_cam, s4_cam, s5_cam : unsigned(12 downto 0) := (others => '0');
    signal s2_kg, s2_kgf : unsigned(1 downto 0) := (others => '0');
    type t_kg is array (3 to 9) of unsigned(1 downto 0);
    signal s_kgd : t_kg := (others => (others => '0'));
    signal s6_kgf : unsigned(1 downto 0) := (others => '0');
    type t_sw is array (4 to 7) of unsigned(12 downto 0);
    signal s_swp : t_sw := (others => (others => '0'));
    type t_hw is array (3 to 8) of unsigned(6 downto 0);
    signal s_hwp : t_hw := (others => (others => '0'));
    signal s_rng, s_fn : std_logic := '0';
    signal s12_a, s12_b : unsigned(5 downto 0) := (others => '0');
    signal s13_sum : unsigned(6 downto 0) := (others => '0');
    signal s14_ya, s15_ya, s16_ya, s17_ya : unsigned(8 downto 0) := (others => '0');

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
    signal tf_d   : unsigned(22 downto 0) := (others => '0');
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
    signal s_upd, c_dy : unsigned(13 downto 0) := (others => '0');
    signal c_a, c_b, s_Ls, s_wcam : unsigned(12 downto 0) := (others => '0');
    signal c_new : std_logic := '0';

    -- [F] output conditioning
    signal s19_y, s20_y, s21_y, s22_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s22_ya : unsigned(11 downto 0) := (others => '0');
    signal s23_y, s24_y, s25_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_EC_TAB(to_integer(r_ex_a));
            yr_q   <= C_EC_Y(to_integer(s17_ya));
        end if;
    end process p_rom;

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

    -- luma line store (half horizontal resolution): the one-pole's own read, at its write address one sample
    -- earlier (the write lands two clocks later, when the read address has moved on)
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q <= lb_ram(to_integer(s_colp(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS (switches, the Discs
    -- steps, the mode constants) + [V] the vertical DDAs
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_lo, v_hi : unsigned(10 downto 0);
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
            s_zgd <= s_zgd(0) & s_zgo;

            -- [A] width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- [B] Discs: 3 steps (0 .. 340, 341 .. 681, 682 ..); a step is left only 8 counts past its edge (the
            -- outer steps are entered from inside, so a knob resting at either end reaches them).  The comparison
            -- runs free; the latch is per field.
            s_k4 <= unsigned(registers_in(3));
            case to_integer(s_zn) is
                when 0      => v_lo := to_unsigned(0, 11);   v_hi := to_unsigned(341 + 8, 11);
                when 1      => v_lo := to_unsigned(341, 11); v_hi := to_unsigned(682 + 8, 11);
                when others => v_lo := to_unsigned(682, 11); v_hi := to_unsigned(2047, 11);
            end case;
            if resize(s_k4, 11) >= v_hi then z_up <= '1'; else z_up <= '0'; end if;
            if resize(s_k4, 11) + 8 < v_lo then z_dn <= '1'; else z_dn <= '0'; end if;
            if s_k4 < 341 then z_kz <= "00"; elsif s_k4 < 682 then z_kz <= "01"; else z_kz <= "10"; end if;

            -- [A][B] per-field latches and the mode constants
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_rings <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                if z_up = '1' or z_dn = '1' then s_zn <= z_kz; end if;
                case to_integer(s_xsr) is
                    when 4 =>
                        s_oxs <= to_unsigned(204, 9); c_xq <= to_unsigned(1960, 11);
                        c_xsq <= to_unsigned(16, 5); c_dd <= to_unsigned(512, 10);
                    when 3 =>
                        s_oxs <= to_unsigned(153, 9); c_xq <= to_unsigned(30, 11);
                        c_xsq <= to_unsigned(12, 5); c_dd <= to_unsigned(288, 10);
                    when others =>
                        s_oxs <= to_unsigned(102, 9); c_xq <= to_unsigned(20, 11);
                        c_xsq <= to_unsigned(8, 5); c_dd <= to_unsigned(128, 10);
                end case;
                if s_xsr = 2 then s_lg6 <= '1'; else s_lg6 <= '0'; end if;
                case to_integer(unsigned(s_tid)) is
                    when 0 =>                                   -- 480i (1136: an exact Q2 step, 0.2 % short)
                        c_ysd <= to_unsigned(1136, 11); c_vv <= to_unsigned(10082, 14); s_oys <= to_unsigned(537, 10);
                        c_lpx <= to_unsigned(807, 10); c_gr <= to_unsigned(95, 9); c_ec <= to_unsigned(2, 2); c_mc <= '1'; c_hw <= to_unsigned(16, 7);
                        s_ish <= to_unsigned(2, 3);
                    when 4 =>                                   -- 576p
                        c_ysd <= to_unsigned(576, 11); c_vv <= to_unsigned(2592, 14); s_oys <= to_unsigned(272, 10);
                        c_lpx <= to_unsigned(555, 10); c_gr <= to_unsigned(48, 9); c_ec <= to_unsigned(1, 2); c_mc <= '1'; c_hw <= to_unsigned(32, 7);
                        s_ish <= to_unsigned(3, 3);
                    when 8 =>                                   -- 576i
                        c_ysd <= to_unsigned(960, 11); c_vv <= to_unsigned(7200, 14); s_oys <= to_unsigned(453, 10);
                        c_lpx <= to_unsigned(744, 10); c_gr <= to_unsigned(80, 9); c_ec <= to_unsigned(2, 2); c_mc <= '1'; c_hw <= to_unsigned(16, 7);
                        s_ish <= to_unsigned(2, 3);
                    when 12 =>                                  -- 480p
                        c_ysd <= to_unsigned(480, 11); c_vv <= to_unsigned(1800, 14); s_oys <= to_unsigned(226, 10);
                        c_lpx <= to_unsigned(512, 10); c_gr <= to_unsigned(43, 9); c_ec <= to_unsigned(1, 2); c_mc <= '1'; c_hw <= to_unsigned(32, 7);
                        s_ish <= to_unsigned(3, 3);
                    when 5 | 6 | 14 =>                          -- 720p
                        c_ysd <= to_unsigned(384, 11); c_vv <= to_unsigned(1152, 14); s_oys <= to_unsigned(181, 10);
                        c_lpx <= to_unsigned(406, 10); c_gr <= to_unsigned(32, 9); c_ec <= to_unsigned(1, 2); c_mc <= '0'; c_hw <= to_unsigned(48, 7);
                        s_ish <= to_unsigned(4, 3);
                    when 1 | 2 | 10 =>                          -- 1080i
                        c_ysd <= to_unsigned(512, 11); c_vv <= to_unsigned(2048, 14); s_oys <= to_unsigned(242, 10);
                        c_lpx <= to_unsigned(512, 10); c_gr <= to_unsigned(43, 9); c_ec <= to_unsigned(1, 2); c_mc <= '1'; c_hw <= to_unsigned(32, 7);
                        s_ish <= to_unsigned(3, 3);
                    when others =>                              -- 1080p
                        c_ysd <= to_unsigned(256, 11); c_vv <= to_unsigned(512, 14); s_oys <= to_unsigned(121, 10);
                        c_lpx <= to_unsigned(256, 10); c_gr <= to_unsigned(21, 9); c_ec <= to_unsigned(0, 2); c_mc <= '1'; c_hw <= to_unsigned(64, 7);
                        s_ish <= to_unsigned(4, 3);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            -- the camera's slope classes (Flat: in 1/16 px per px; Rings: the ring's own slope x 1/2, 2, 4.5) and
            -- the films: 1/n (Flat Bold), 1/max(2, n - 1) (Fine), 1/2 (Rings: at most 3 discs)
            if s_zgd(1) = '1' then
                if s_rings = '1' then
                    c_th1 <= resize(c_gr(8 downto 1), 11);
                    c_th2 <= resize(c_gr & '0', 11);
                    c_th3 <= resize(c_gr & "00", 11) + resize(c_gr(8 downto 1), 11);
                    c_sel <= "00";
                else
                    c_th1 <= to_unsigned(6, 11); c_th2 <= to_unsigned(29, 11); c_th3 <= to_unsigned(74, 11);
                    if s_fine = '1' then
                        if s_zn(1) = '0' then c_sel <= "00"; else c_sel <= s_zn - 1; end if;
                    else
                        c_sel <= s_zn;
                    end if;
                end if;
            end if;

            -- [V] vertical DDAs: one shared adder.  After each line's start it steps disc 0 .. 3 in turn
            -- (read V, U; write V + U, U + vv); in the zone the engine's OUT V / OUT U go through the same adder
            -- (with a zero operand)
            if g_start = '1' then s_vk <= (others => '0');
            elsif s_vk /= 7 then s_vk <= s_vk + 1; end if;
            if s_vk < 4 then
                s_vr <= s_V(to_integer(s_vk(1 downto 0))); s_ur <= s_U(to_integer(s_vk(1 downto 0)));
                s_vki <= s_vk; s_wv <= '1'; s_wu <= '1'; s_av <= '1';
            elsif e_out = '1' and e_irf(3 downto 0) = x"2" then
                s_vr <= e_acc(33 downto 0); s_ur <= (others => '0'); s_vki <= e_di; s_wv <= '1'; s_wu <= '0'; s_av <= '0';
            elsif e_out = '1' and e_irf(3 downto 0) = x"3" then
                s_vr <= (others => '0'); s_ur <= e_acc(23 downto 0); s_vki <= e_di; s_wv <= '0'; s_wu <= '1'; s_av <= '0';
            else
                s_wv <= '0'; s_wu <= '0';
            end if;
            for i in 0 to 3 loop
                if s_wv = '1' and s_vki = i then s_V(i) <= s_vr + resize(s_ur, 34); end if;
                if s_wu = '1' and s_vki = i then
                    if s_av = '1' then s_U(i) <= s_ur + signed(resize(c_vv, 24)); else s_U(i) <= s_ur; end if;
                end if;
            end loop;
            s_st0 <= g_start;

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
    -- [C] THE PER-FIELD ENGINE (Hardline flute's).  Started at every zone start
    -- from address 0 (so a stalled or garbled run cannot outlive a field);
    -- three clocks per micro-op (register-file read, operand, execute), serial
    -- multiply (18 steps) and shift.  ACC = 37 bits; MUL = ACC(17:0) x
    -- operand(17:0), both signed, by add-and-shift-right with the last step
    -- subtracting.  The microprogram (eclipse.py) does the knob holds and
    -- glides, the scale, spread, formation and speed, the camera gain, and per
    -- disc the orbit angle, sin / cos, centre, radius and every DDA start value.
    ------------------------------------------------------------------------
    with e_irf(3 downto 0) select w_in <=
        resize(c_ysd, 16)           when x"8",
        resize(c_xq, 16)            when x"9",
        resize(c_xsq, 16)           when x"A",
        r_ex_q                      when x"B",
        resize(c_lpx, 16)           when x"D",
        resize(s_zn, 16)            when x"E",
        (others => '0')             when x"C" | x"F",
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
                            when "010"  => v_cond := s_rings;
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
                e_ir <= C_EC_UC(to_integer(e_pc));
            end if;

            -- the engine's output registers (V and U: the vertical DDAs, p_ctrl)
            tf_go <= '0';
            if e_out = '1' then
                case to_integer(unsigned(e_irf(3 downto 0))) is
                    when 0 => r_ex_a <= unsigned(e_acc(8 downto 0));
                    when 1 => e_di <= unsigned(e_acc(2 downto 0));
                    when 4 =>
                        for i in 0 to 3 loop
                            if e_di = i then c_D0(i) <= e_acc(21 downto 0); end if;
                        end loop;
                    when 6 =>
                        for i in 0 to 3 loop
                            if e_di = i then c_e(i) <= unsigned(e_acc(2 downto 0)); c_m(i) <= e_acc(3); end if;
                        end loop;
                    when 7 => c_Gt <= unsigned(e_acc(13 downto 0)); tf_go <= '1';
                    when 9 =>
                        for i in 0 to 2 loop
                            if e_di = i then c_s(i) <= unsigned(e_acc(2 downto 0)); c_mu(i) <= unsigned(e_acc(8 downto 3)); end if;
                        end loop;
                    when 10 =>
                        for i in 0 to 2 loop
                            if e_di = i then c_t1(i) <= unsigned(e_acc(14 downto 0)); end if;
                        end loop;
                    when 11 =>
                        for i in 0 to 2 loop
                            if e_di = i then c_t2(i) <= unsigned(e_acc(14 downto 0)); end if;
                        end loop;
                    when others => null;
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

    ---------------------------------------------------------------------------
    -- [P] PIXEL PATH, one per disc, from P = r^2 - R^2 (the DDA, q^2).
    --   Discs 0 .. 2: d_r = (P 2^-(8 + s)) mu / 64, the edge distance in ring units
    --   (24 virtual px = 512); the edge in 1/16 AA px is d_r 3/4^mc 2^-ec, divided by
    --   the camera's slope class 2^kg (Flat); x = that - swell; Bold t = -x, Fine
    --   t = hw - |x|; coverage clamp(8 + t).  Rings: the Fresnel ring phase is d_r +
    --   the camera (mod 512), the band's coverage by its distance to the band's
    --   middle over the ring's slope (the rim's, x 2 or x 4 by the r / R class from
    --   two thresholds, and the camera's class), and the smaller of the two counts.
    --   Disc 3: d16 = P 2^-(9 + e + kg) (x 3/4) directly; Rings does not draw it.
    -- Stages 3 .. 10; stage 11 drops undrawn discs.
    ---------------------------------------------------------------------------
    g_disc : for i in 0 to 3 generate
        signal b_P   : signed(31 downto 0) := (others => '0');
        signal b_D   : signed(21 downto 0) := (others => '0');
        signal s3_v  : signed(14 downto 0) := (others => '0');
        signal s10_covf, s10_covr : unsigned(4 downto 0) := (others => '0');
    begin
        p_dda : process(clk)
        begin
            if rising_edge(clk) then
                -- 2: the DDA, loaded with the line's pre-pixel
                if g_start = '1' then
                    b_P <= s_V(i)(33 downto 2); b_D <= c_D0(i);
                else
                    b_P <= b_P + resize(b_D, 32);
                    b_D <= b_D + signed(resize(c_dd, 22));
                end if;
            end if;
        end process p_dda;

        p_cov : process(clk)
        begin
            if rising_edge(clk) then
                -- 11: the disc's coverage (Rings: the smaller of edge and band; 0 when the disc is not drawn)
                if (s_rng = '0' and to_unsigned(i, 3) > resize(s_zn, 3) + 1) or
                   (s_rng = '1' and (i > 2 or to_unsigned(i, 3) > resize(s_zn, 3) + 1)) then
                    s11_cov(i) <= (others => '0');
                elsif s_rng = '1' and s10_covr < s10_covf then
                    s11_cov(i) <= s10_covr;
                else
                    s11_cov(i) <= s10_covf;
                end if;
            end if;
        end process p_cov;

        g_ring : if i < 3 generate
            signal s4_pa, s4_pb : signed(20 downto 0) := (others => '0');
            signal s5_dr, s6_de, s7_de : signed(14 downto 0) := (others => '0');
            signal s6_ph : unsigned(8 downto 0) := (others => '0');
            signal s6_cl, s7_cl : unsigned(1 downto 0) := (others => '0');
            signal s7_dph, s8_a : signed(9 downto 0) := (others => '0');
            signal s8_x, s9_t : signed(15 downto 0) := (others => '0');
            signal s8_sh : signed(3 downto 0) := (others => '0');
            signal s9_dr : signed(5 downto 0) := (others => '0');
        begin
            p_rg : process(clk)
                variable v_w  : signed(31 downto 0);
                variable v_g  : signed(20 downto 0);
                variable v_s  : signed(20 downto 0);
                variable v_sh : unsigned(2 downto 0);
                variable v_a  : unsigned(15 downto 0);
                variable v_t  : signed(15 downto 0);
                variable v_f  : unsigned(8 downto 0);
                variable v_r  : signed(9 downto 0);
                variable v_q  : signed(12 downto 0);
                variable v_c  : signed(7 downto 0);
            begin
                if rising_edge(clk) then
                    -- 3: P 2^-(8 + s), saturated to 15 bits
                    case to_integer(c_s(i)) is
                        when 0      => v_w := shift_right(b_P, 8);
                        when 1      => v_w := shift_right(b_P, 9);
                        when 2      => v_w := shift_right(b_P, 10);
                        when 3      => v_w := shift_right(b_P, 11);
                        when others => v_w := shift_right(b_P, 12);
                    end case;
                    if v_w(31 downto 14) = 0 or v_w(31 downto 14) = -1 then s3_v <= v_w(14 downto 0);
                    elsif v_w(31) = '1' then s3_v <= to_signed(-16384, 15);
                    else s3_v <= to_signed(16383, 15); end if;
                    -- 4, 5: x mu / 64 (mu = 32 .. 63: 1 + five gated terms)
                    v_s := shift_left(resize(s3_v, 21), 5);
                    if c_mu(i)(4) = '1' then v_s := v_s + shift_left(resize(s3_v, 21), 4); end if;
                    v_g := (others => '0');
                    if c_mu(i)(3) = '1' then v_g := shift_left(resize(s3_v, 21), 3); end if;
                    s4_pa <= v_s + v_g;
                    v_s := (others => '0');
                    if c_mu(i)(2) = '1' then v_s := shift_left(resize(s3_v, 21), 2); end if;
                    v_g := (others => '0');
                    if c_mu(i)(1) = '1' then v_g := shift_left(resize(s3_v, 21), 1); end if;
                    if c_mu(i)(0) = '1' then v_g := v_g + resize(s3_v, 21); end if;
                    s4_pb <= v_s + v_g;
                    v_s := s4_pa + s4_pb;
                    s5_dr <= v_s(20 downto 6);
                    -- 6: 3/4; the ring phase (+ the camera, mod 512); the r / R class
                    if c_mc = '1' then s6_de <= s5_dr - shift_right(s5_dr, 2); else s6_de <= s5_dr; end if;
                    s6_ph <= unsigned(s5_dr(8 downto 0)) + s5_cam(8 downto 0);
                    if s5_dr < -signed(resize(c_t1(i), 16)) then s6_cl <= "10";
                    elsif s5_dr < -signed(resize(c_t2(i), 16)) then s6_cl <= "01";
                    else s6_cl <= "00"; end if;
                    -- 7: the edge in 1/16 AA px / 2^kg; duty / 2 - |f - 1/2| (ring units, one carry chain)
                    v_sh := resize(c_ec, 3) + resize(s6_kgf, 3);
                    case to_integer(v_sh) is
                        when 0      => s7_de <= s6_de;
                        when 1      => s7_de <= shift_right(s6_de, 1);
                        when 2      => s7_de <= shift_right(s6_de, 2);
                        when 3      => s7_de <= shift_right(s6_de, 3);
                        when 4      => s7_de <= shift_right(s6_de, 4);
                        when others => s7_de <= shift_right(s6_de, 5);
                    end case;
                    v_f := s6_ph;
                    if s_fine = '1' then
                        if v_f(8) = '1' then v_r := signed(f_addsub(to_unsigned(299, 10), resize(v_f, 10), '1'));
                        else v_r := signed(f_addsub(resize(v_f, 10), to_unsigned(213, 10), '1')); end if;
                    else
                        if v_f(8) = '1' then v_r := signed(f_addsub(to_unsigned(384, 10), resize(v_f, 10), '1'));
                        else v_r := signed(f_addsub(resize(v_f, 10), to_unsigned(128, 10), '1')); end if;
                    end if;
                    s7_dph <= v_r; s7_cl <= s6_cl;
                    -- 8: minus the swell; 3/4 of the band distance; its shift ec + kg - class
                    s8_x <= resize(s7_de, 16) - signed(resize(s_swp(7), 16));
                    if c_mc = '1' then s8_a <= s7_dph - shift_right(s7_dph, 2); else s8_a <= s7_dph; end if;
                    s8_sh <= signed(resize(c_ec, 4)) + signed(resize(s_kgd(7), 4)) - signed(resize(s7_cl, 4));
                    -- 9: Bold 0 - x, Fine hw - |x|; the band distance in 1/16 AA px, saturated
                    if s_fn = '1' then v_a := f_addsub(resize(s_hwp(8), 16), unsigned(s8_x), not s8_x(15));
                    else v_a := f_addsub(to_unsigned(0, 16), unsigned(s8_x), '1'); end if;
                    s9_t <= signed(v_a);
                    case to_integer(s8_sh) is
                        when -2     => v_q := shift_left(resize(s8_a, 13), 2);
                        when -1     => v_q := shift_left(resize(s8_a, 13), 1);
                        when 0      => v_q := resize(s8_a, 13);
                        when 1      => v_q := resize(shift_right(s8_a, 1), 13);
                        when 2      => v_q := resize(shift_right(s8_a, 2), 13);
                        when 3      => v_q := resize(shift_right(s8_a, 3), 13);
                        when 4      => v_q := resize(shift_right(s8_a, 4), 13);
                        when others => v_q := resize(shift_right(s8_a, 5), 13);
                    end case;
                    if v_q > 31 then s9_dr <= to_signed(31, 6);
                    elsif v_q < -32 then s9_dr <= to_signed(-32, 6);
                    else s9_dr <= v_q(5 downto 0); end if;
                    -- 10: coverages 0 .. 16 (the camera too steep for the band: its mean)
                    v_t := s9_t + 8;
                    if v_t < 0 then s10_covf <= (others => '0');
                    elsif v_t > 16 then s10_covf <= to_unsigned(16, 5);
                    else s10_covf <= unsigned(v_t(4 downto 0)); end if;
                    v_c := resize(s9_dr, 8) + 8;
                    if s_kgd(9) = "11" then
                        if s_fine = '1' then s10_covr <= to_unsigned(3, 5); else s10_covr <= to_unsigned(8, 5); end if;
                    elsif v_c < 0 then s10_covr <= (others => '0');
                    elsif v_c > 16 then s10_covr <= to_unsigned(16, 5);
                    else s10_covr <= unsigned(v_c(4 downto 0)); end if;
                end if;
            end process p_rg;
        end generate g_ring;

        g_flat : if i >= 3 generate
            signal s4_d16 : signed(14 downto 0) := (others => '0');
            signal s5_x, s6_t : signed(15 downto 0) := (others => '0');
            type t_cd is array (7 to 9) of unsigned(4 downto 0);
            signal s_cd : t_cd := (others => (others => '0'));
        begin
            p_fl : process(clk)
                variable v_sh : unsigned(4 downto 0);
                variable v_w  : signed(31 downto 0);
                variable v_a  : unsigned(15 downto 0);
                variable v_t  : signed(15 downto 0);
            begin
                if rising_edge(clk) then
                    -- 3: P 2^-(9 + e + kg), saturated to 15 bits
                    v_sh := resize(c_e(i), 5) + resize(s2_kgf, 5);
                    case to_integer(v_sh) is
                        when 0      => v_w := shift_right(b_P, 9);
                        when 1      => v_w := shift_right(b_P, 10);
                        when 2      => v_w := shift_right(b_P, 11);
                        when 3      => v_w := shift_right(b_P, 12);
                        when 4      => v_w := shift_right(b_P, 13);
                        when 5      => v_w := shift_right(b_P, 14);
                        when 6      => v_w := shift_right(b_P, 15);
                        when 7      => v_w := shift_right(b_P, 16);
                        when 8      => v_w := shift_right(b_P, 17);
                        when 9      => v_w := shift_right(b_P, 18);
                        when others => v_w := shift_right(b_P, 19);
                    end case;
                    if v_w(31 downto 14) = 0 or v_w(31 downto 14) = -1 then s3_v <= v_w(14 downto 0);
                    elsif v_w(31) = '1' then s3_v <= to_signed(-16384, 15);
                    else s3_v <= to_signed(16383, 15); end if;
                    -- 4: x 3/4; 5: minus the swell; 6: Bold 0 - x, Fine hw - |x|; 7: coverage; 8 .. 10: delay
                    if c_m(i) = '1' then s4_d16 <= s3_v - shift_right(s3_v, 2); else s4_d16 <= s3_v; end if;
                    s5_x <= resize(s4_d16, 16) - signed(resize(s_swp(4), 16));
                    if s_fn = '1' then v_a := f_addsub(resize(s_hwp(5), 16), unsigned(s5_x), not s5_x(15));
                    else v_a := f_addsub(to_unsigned(0, 16), unsigned(s5_x), '1'); end if;
                    s6_t <= signed(v_a);
                    v_t := s6_t + 8;
                    if v_t < 0 then s_cd(7) <= (others => '0');
                    elsif v_t > 16 then s_cd(7) <= to_unsigned(16, 5);
                    else s_cd(7) <= unsigned(v_t(4 downto 0)); end if;
                    s_cd(8) <= s_cd(7); s_cd(9) <= s_cd(8); s10_covf <= s_cd(9);
                end if;
            end process p_fl;
            s10_covr <= (others => '1');
        end generate g_flat;
    end generate g_disc;

    ---------------------------------------------------------------------------
    -- [P] the camera stream into the pixel path (the slope class at stage 2, the
    -- swell and the rings' phase from stage 3), the summed coverage and the tone
    -- (12 .. 17)
    ---------------------------------------------------------------------------
    p_sum : process(clk)
        variable v_kg : unsigned(1 downto 0);
        variable v_tr : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            s_rng <= s_rings;
            s_fn  <= s_fine and not s_rings;
            v_kg := "00";
            if s_wcam > resize(c_th3, 13) then v_kg := "11";
            elsif s_wcam > resize(c_th2, 13) then v_kg := "10";
            elsif s_wcam > resize(c_th1, 13) then v_kg := "01"; end if;
            s2_kg <= v_kg;
            if s_rings = '1' then s2_kgf <= "00"; else s2_kgf <= v_kg; end if;
            s_kgd(3) <= s2_kg;
            for k in 4 to 9 loop s_kgd(k) <= s_kgd(k - 1); end loop;
            if s_rng = '1' then s6_kgf <= "00"; else s6_kgf <= s_kgd(5); end if;
            s3_cam <= s_Ls; s4_cam <= s3_cam; s5_cam <= s4_cam;
            -- the swell and the Fine half width, divided by the slope class
            if s_rng = '1' then s_swp(4) <= (others => '0');
            else s_swp(4) <= shift_right(s3_cam, to_integer(s_kgd(3))); end if;
            for k in 5 to 7 loop s_swp(k) <= s_swp(k - 1); end loop;
            s_hwp(3) <= shift_right(c_hw, to_integer(s2_kg));
            for k in 4 to 8 loop s_hwp(k) <= s_hwp(k - 1); end loop;
            -- 12, 13: the sum
            s12_a <= resize(s11_cov(0), 6) + resize(s11_cov(1), 6);
            s12_b <= resize(s11_cov(2), 6) + resize(s11_cov(3), 6);
            s13_sum <= resize(s12_a, 7) + resize(s12_b, 7);
            -- 14: the tone's address: Grey = film & sum; Ink = the triangle of the sum (as 2 x on film 1/2)
            if s_ink = '1' then
                if s13_sum(4) = '1' then v_tr := to_unsigned(16, 5) - resize(s13_sum(3 downto 0), 5);
                else v_tr := resize(s13_sum(3 downto 0), 5); end if;
                s14_ya <= resize(v_tr & '0', 9);
            else
                s14_ya <= c_sel & s13_sum;
            end if;
            s15_ya <= s14_ya; s16_ya <= s15_ya; s17_ya <= s16_ya;
        end if;
    end process p_sum;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> the per-field table
    -- T(l) = K6-gain * l^2 -> box of 64 (1080) or 32 samples, edge-replicated and
    -- reloaded at every line start -> vertical one-pole in the line store (half
    -- horizontal resolution, Q13.3).  The pixel path takes the one-pole's stream
    -- as it is written (the camera is never shown): L, and |its x step| + |its y
    -- step| for the anti-aliasing.
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
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);   -- ot * 96
            end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);

            -- the source sample: (luma >> 2) - 16, or the lattice (0 .. 254) -> the table (p_tbram) -> s_p
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

            -- box: sum of the last N samples; the samples before the line start count as the first one
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

            -- vertical one-pole on the odd columns: q += (box - q) >> sh
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
            v_n := -v_u;
            if v_u(16) = '1' then s_upd <= unsigned(v_n(13 downto 0)); else s_upd <= unsigned(v_u(13 downto 0)); end if;

            -- the stream to the pixel path: the latest and the previous column's values, their mean on the
            -- clock in between, and the slope
            c_new <= lb_we;
            if lb_we = '1' then
                c_b <= c_a; c_a <= lb_wd(15 downto 3); c_dy <= s_upd;
            end if;
            v_mn := resize(c_a, 14) + resize(c_b, 14);
            if c_new = '1' then s_Ls <= v_mn(13 downto 1); else s_Ls <= c_a; end if;
            if c_a > c_b then v_df := c_a - c_b; else v_df := c_b - c_a; end if;
            v_wc := resize(v_df(12 downto 1), 14) + resize(c_dy(13 downto 3), 14);
            if v_wc(13) = '1' then s_wcam <= (others => '1'); else s_wcam <= v_wc(12 downto 0); end if;

            -- the table fill (256 clocks, started when the engine outputs the gain): T(l) = min(Gt l^2 >> 16, 8191) by second differences
            tb_we <= '0';
            if tf_go = '1' then
                tf_run <= '1'; tf_i <= (others => '0'); tf_acc <= (others => '0'); tf_d <= resize(c_Gt, 23);
            elsif tf_run = '1' then
                tb_we <= '1'; tb_wa <= tf_i;
                if tf_acc(29) = '1' then tb_wd <= to_unsigned(8191, 16);
                else tb_wd <= resize(tf_acc(28 downto 16), 16); end if;
                v_ta := tf_acc + resize(tf_d, 30);
                tf_acc <= v_ta;
                tf_d <= tf_d + resize(c_Gt & '0', 23);
                tf_i <= tf_i + 1;
                if tf_i = 255 then tf_run <= '0'; end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: tone -> luma (ROM, stage 18), black outside pixels
    -- 0 .. W + 1 (19), luma [1 2 1] / 4 (22, 23), 13/16 toward black (24),
    -- blanking gate (25).  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_gav_sr(16) = '1' or s_gav_sr(18) = '1' then s19_y <= yr_q; else s19_y <= to_unsigned(64, 10); end if;
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

end architecture eclipse;
