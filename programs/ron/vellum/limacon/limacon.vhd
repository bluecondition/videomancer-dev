-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Limacon v0.1.0 -- Vellum.  The cardioid drawn as an envelope of circles: up to 24 translucent discs, their
-- centres evenly spaced round a base circle, every one passing through the same pole.  Their outer edges crowd
-- into a cardioid (the pole on the base circle), a limacon of Pascal (the pole inside: a rosette; outside: the
-- inner loop) or, in Nephroid form, every disc tangent to a diameter (the nephroid, Huygens' construction).
-- Grey: the number of discs over a pixel, as flat grey steps cycling black, dark, light, white every four discs;
-- Ink: their parity (a black-and-white lace of crescents).  The discs roll round the base circle and the pole
-- orbits the other way at a quarter of the rate.
--
--   K1  X        the figure, +-960 virtual px          K2  Y      the figure, +-540 virtual px
--   K3  Scale    base radius 99 .. 581 virtual px, exponential (240 at 50 %)
--   K4  Count    8 / 12 / 16 / 24 discs (stepped, hysteresis)
--   K5  Drift    bipolar: +-24 still, 2^-9 .. 2^-2 disc steps per field; the pole orbits back at 1/4 the rate
--   K6  Camera   box-filtered luma^2 added to every disc's F (the discs shrink away from the light)
--   S7  Glide    On: the knobs ease 1/16 of the gap per field (Drift's speed 1/8); Off: no easing
--   S8  Form     Cardioid (discs through the pole) / Nephroid (discs tangent to a diameter)
--   S9  Tone     Grey / Ink             S10 Weight  Bold (discs) / Fine (each circle a 2-pixel ring)
--   S11 Source   Video / Osc (the shell's drifting triangle lattice)
--   P12 POLE     the pole's distance from the base centre: 0.15 R (rosette) .. R at 60 % (the cardioid, cusp
--                and all) .. 2.2 R (the limacon's inner loop); Nephroid: the radius rule 0.5 .. 1.0 .. 1.8
--
-- Sections (the integer model .vellum_work/rb_limacon/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         switches, the Count steps
--   [C] FIELD ENGINE                     Hardline flute's microcoded engine (limacon.py holds the program and its
--                                        emulator): holds + glides, scale, pole, drift, camera gain, and per
--                                        circle the centre, the line-start / line-end / slope constants and the
--                                        edge scale, written into the inactive half of the bank RAM
--   [S] SEQUENCER                        per field (zone start): swap banks, load every circle's F and D; per line
--                                        (after the last pixel step): F += KC + Ysh, D = D0 (one shared adder)
--   [P] PIXEL PATH (x 24)                F += D, D += bus (second difference + camera), all circles in parallel;
--                                        F -> 8-position window -> coverage (17 steps) -> sum + crowd -> tone ROM
--   [D] SOURCE FIELD                     luma (or Osc) -> per-field K6 * l^2 table -> box -> vertical one-pole
--   [F] OUTPUT CONDITIONING              luma [1 2 1] / 4, 13/16 toward black, blanking gate; U = V = 512
--
-- Stage numbering (the Hardline shell's): a register "at stage S" holds pixel n at T + S + n (T = the g_start
-- clock).  The circles' F registers hold pixel m at stage 7 (they step from pixel -2 to W + 2).  One latency
-- (24) in every mode.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.limacon_pkg.all;
use work.all;

architecture limacon of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks
    constant NC    : integer := 24;   -- circle slots

    ---------------------------------------------------------------------------
    -- [A] timing-ID steps
    ---------------------------------------------------------------------------
    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1152, 12);     -- 480i (4.5 virtual lines, 1 % tall)
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

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ysr  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
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

    -- [B] switches, count
    signal s_glide : std_logic := '1';
    signal s_neph, s_ink, s_fine, s_src : std_logic := '0';
    signal s_zn : unsigned(1 downto 0) := to_unsigned(3, 2);       -- 0..3 = 8, 12, 16, 24 discs
    signal z_up, z_dn : std_logic := '0';
    signal z_kz : unsigned(1 downto 0) := to_unsigned(3, 2);
    signal s_k4 : unsigned(9 downto 0) := (others => '0');
    signal s_xc : unsigned(5 downto 0) := to_unsigned(8, 6);        -- 2 xs^2: the pixel DDA's second step

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
    signal rf_ram : t_c_lm_rfi := C_LM_RFI;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal e_run : std_logic := '0';
    signal e_ph  : unsigned(1 downto 0) := "11";
    signal e_pc, e_lr : unsigned(10 downto 0) := (others => '0');
    signal e_ir  : std_logic_vector(15 downto 0) := x"F001";      -- the microprogram ROM's output register
    signal e_irf : std_logic_vector(15 downto 0) := x"F001";      -- its fabric copy
    signal x_mul, x_shr, x_halt, x_out, x_st : std_logic := '0';
    signal x_gate, x_subr, x_shift, x_cz : std_logic := '0';
    signal e_acc, e_b : signed(36 downto 0) := (others => '0');
    signal e_cnt : unsigned(4 downto 0) := (others => '0');
    signal e_out : std_logic;
    signal w_in  : unsigned(15 downto 0);
    signal e_ci  : unsigned(4 downto 0) := (others => '0');
    signal e_em  : std_logic_vector(4 downto 0) := (others => '0');
    signal c_Gt  : unsigned(13 downto 0) := (others => '0');
    signal tf_go : std_logic := '0';
    signal s_eb  : std_logic := '0';                                -- the bank the engine writes (displayed: not)

    -- [S] the circle-constant banks: 2 x (32 slots x 4 words): F00, KC, D0 | EM, (slot 24: dY0, A_Y, -, W | f0)
    signal bk_ram : t_c_lm_bki := C_LM_BKI;
    attribute no_rw_check of bk_ram : signal is true;
    signal bk_q, bk_f : std_logic_vector(31 downto 0) := (others => '0');
    signal bk_we : std_logic := '0';
    signal bk_wa, bk_ra : unsigned(7 downto 0) := (others => '0');
    signal bk_wd : std_logic_vector(31 downto 0) := (others => '0');

    -- [S] sequencer
    signal q_ph  : unsigned(3 downto 0) := (others => '0');
    signal q_k   : unsigned(4 downto 0) := (others => '0');
    signal q_fsr : std_logic := '0';                                -- a field-start pass is due
    signal r_v0, r_v1, r_v2, r_v3 : std_logic := '0';               -- read pipeline: valid
    signal r_s0, r_s1, r_s2, r_s3 : unsigned(1 downto 0) := (others => '0');
    signal r_k0, r_k1, r_k2, r_k3 : unsigned(4 downto 0) := (others => '0');
    signal r_h0, r_h1, r_h2, r_h3 : std_logic := '0';               -- a shared-word read
    signal r_f0, r_f1, r_f2, r_f3 : std_logic := '0';               -- F += D lands with this load
    signal c_frst, c_drst, c_fsq : std_logic := '0';
    signal c_ld, c_lem : std_logic := '0';
    signal c_lk  : unsigned(4 downto 0) := (others => '0');
    signal c_bus : signed(26 downto 0) := (others => '0');
    signal c_bem : std_logic_vector(4 downto 0) := (others => '0');
    signal c_step : std_logic := '0';
    signal y_f   : unsigned(11 downto 0) := (others => '0');
    signal y_dy, y_ay : signed(31 downto 0) := (others => '0');
    signal y_sh  : signed(19 downto 0) := (others => '0');
    signal s_wb  : unsigned(11 downto 0) := to_unsigned(1920, 12);   -- the bank's pixel count
    signal st_gd : std_logic_vector(3 downto 0) := (others => '0');
    signal st_n  : unsigned(11 downto 0) := (others => '0');

    -- [P] the camera's second difference into the bus
    signal p_c0, p_c1, p_c2, p_c2d : signed(16 downto 0) := (others => '0');
    signal p_d1 : signed(18 downto 0) := (others => '0');

    -- [P] coverage of every circle (stage 12) and their near flags
    type t_c5 is array (0 to NC - 1) of unsigned(4 downto 0);
    signal s10_t : t_c5 := (others => (others => '0'));
    signal s10_n : std_logic_vector(0 to NC - 1) := (others => '0');
    type t_u6x12 is array (0 to 11) of unsigned(5 downto 0);
    type t_u7x6 is array (0 to 5) of unsigned(6 downto 0);
    type t_u8x3 is array (0 to 2) of unsigned(7 downto 0);
    type t_u2x12 is array (0 to 11) of unsigned(1 downto 0);
    type t_u3x6 is array (0 to 5) of unsigned(2 downto 0);
    type t_u4x3 is array (0 to 2) of unsigned(3 downto 0);
    signal s13_a : t_u6x12 := (others => (others => '0'));
    signal s14_a : t_u7x6 := (others => (others => '0'));
    signal s15_a : t_u8x3 := (others => (others => '0'));
    signal s16_a : unsigned(8 downto 0) := (others => '0');
    signal s16_b : unsigned(7 downto 0) := (others => '0');
    signal s17_s : unsigned(8 downto 0) := (others => '0');
    signal s13_n : t_u2x12 := (others => (others => '0'));
    signal s14_n : t_u3x6 := (others => (others => '0'));
    signal s15_n : t_u4x3 := (others => (others => '0'));
    signal s16_na : unsigned(4 downto 0) := (others => '0');
    signal s16_nb : unsigned(3 downto 0) := (others => '0');
    signal s17_n : unsigned(4 downto 0) := (others => '0');
    signal s18_ya : unsigned(9 downto 0) := (others => '0');
    signal s_pf, s_pi : std_logic := '0';
    signal s_pk : unsigned(4 downto 0) := to_unsigned(8, 5);         -- 8 (Bold) or 24 (Fine)

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
    signal s_ebx : unsigned(12 downto 0) := (others => '0');
    signal s_eq  : unsigned(15 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal c_a, c_b, s_Lw : unsigned(15 downto 0) := (others => '0');
    signal c_new : std_logic := '0';
    signal s_lg6 : std_logic := '1';
    signal s_ish : unsigned(2 downto 0) := to_unsigned(4, 3);

    -- [F] output conditioning
    signal s18_y, s19_y, s20_y, s21_y, s22_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s22_ya : unsigned(11 downto 0) := (others => '0');
    signal s23_y, s24_y, s25_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_LM_TAB(to_integer(r_ex_a));
            yr_q   <= C_LM_Y(to_integer(s18_ya));
        end if;
    end process p_rom;

    -- the circle-constant banks: the engine writes one half, the sequencer reads the other
    p_bkram : process(clk)
    begin
        if rising_edge(clk) then
            bk_q <= bk_ram(to_integer(bk_ra));
            if bk_we = '1' then bk_ram(to_integer(bk_wa)) <= bk_wd; end if;
        end if;
    end process p_bkram;

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
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_lo, v_hi : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- [A] timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ysr <= f_vstep_y(s_tid);
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

            -- [B] Count: 4 steps (0 .. 255, 256 .. 511, 512 .. 767, 768 ..); a step is left only 8 counts past its
            -- edge (the outer steps are entered from inside, so a knob resting at either end reaches them).  The
            -- comparison runs free; the latch is per field.
            s_k4 <= unsigned(registers_in(3));
            case to_integer(s_zn) is
                when 0      => v_lo := to_unsigned(0, 11);   v_hi := to_unsigned(256 + 8, 11);
                when 1      => v_lo := to_unsigned(256, 11); v_hi := to_unsigned(512 + 8, 11);
                when 2      => v_lo := to_unsigned(512, 11); v_hi := to_unsigned(768 + 8, 11);
                when others => v_lo := to_unsigned(768, 11); v_hi := to_unsigned(2047, 11);
            end case;
            if resize(s_k4, 11) >= v_hi then z_up <= '1'; else z_up <= '0'; end if;
            if resize(s_k4, 11) + 8 < v_lo then z_dn <= '1'; else z_dn <= '0'; end if;
            z_kz <= s_k4(9 downto 8);

            -- [A][B] per-field latches and the mode constants
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_neph  <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_ys    <= s_ysr;
                if z_up = '1' or z_dn = '1' then s_zn <= z_kz; end if;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                if s_xsr = 2 then s_lg6 <= '1'; else s_lg6 <= '0'; end if;
                case to_integer(s_ysr) is
                    when 1152   => s_oys <= to_unsigned(544, 10); s_ish <= to_unsigned(2, 3);
                    when 960    => s_oys <= to_unsigned(453, 10); s_ish <= to_unsigned(2, 3);
                    when 576    => s_oys <= to_unsigned(272, 10); s_ish <= to_unsigned(3, 3);
                    when 512    => s_oys <= to_unsigned(242, 10); s_ish <= to_unsigned(3, 3);
                    when 480    => s_oys <= to_unsigned(226, 10); s_ish <= to_unsigned(3, 3);
                    when 384    => s_oys <= to_unsigned(181, 10); s_ish <= to_unsigned(4, 3);
                    when others => s_oys <= to_unsigned(121, 10); s_ish <= to_unsigned(4, 3);
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
    -- [C] THE PER-FIELD ENGINE (Hardline flute's, as in Vellum eclipse).
    -- Started at every zone start from address 0 (so a stalled or garbled
    -- run cannot outlive a field); three clocks per micro-op (register-file
    -- read, operand, execute), serial multiply (18 steps) and shift.  ACC =
    -- 37 bits; MUL = ACC(17:0) x operand(17:0), both signed.  It writes the
    -- next field's circle constants into its half of the bank RAM; the halves
    -- swap at the next zone start when it has halted.
    ------------------------------------------------------------------------
    with e_irf(3 downto 0) select w_in <=
        resize(s_ys, 16)            when x"8",
        resize(s_xs, 16)            when x"9",
        resize(s_w, 16)             when x"A",
        r_ex_q                      when x"B",
        resize(s_zn, 16)            when x"D",
        (others => '0')             when x"C" | x"E" | x"F",
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
                if e_run = '0' then s_eb <= not s_eb; end if;   -- a finished run's bank becomes the displayed one
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
                            when "010"  => v_cond := s_neph;
                            when others => v_cond := s_glide;
                        end case;
                        v_cond := v_cond xor v_arg(3);
                        if v_op = OP_SKIP and v_cond = '1' then e_pc <= e_pc + 2;
                        elsif v_op = OP_JMP then e_pc <= unsigned(v_arg(10 downto 0));
                        elsif v_op = OP_CALL then e_pc <= unsigned(v_arg(10 downto 0)); e_lr <= e_pc + 1;
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
                e_ir <= C_LM_UC(to_integer(e_pc));
            end if;

            -- the engine's outputs: the table address, the circle index, the camera gain, the EM bits, and the
            -- bank writes (slot e_ci, word = the OUT code - 4; word 2 carries the EM bits on top)
            tf_go <= '0';
            bk_we <= '0';
            if e_out = '1' then
                case to_integer(unsigned(e_irf(3 downto 0))) is
                    when 0 => r_ex_a <= unsigned(e_acc(8 downto 0));
                    when 1 => e_ci <= unsigned(e_acc(4 downto 0));
                    when 2 => c_Gt <= unsigned(e_acc(13 downto 0)); tf_go <= '1';
                    when 3 => e_em <= std_logic_vector(e_acc(4 downto 0));
                    when 4 | 5 | 6 | 7 =>
                        bk_we <= '1';
                        bk_wa <= s_eb & e_ci & unsigned(e_irf(1 downto 0));
                        if e_irf(1 downto 0) = "10" then bk_wd <= e_em & std_logic_vector(e_acc(26 downto 0));
                        else bk_wd <= std_logic_vector(e_acc(31 downto 0)); end if;
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
    -- [S] SEQUENCER.  Field start (zone start + 1): read the shared words (dY0,
    -- A_Y, W | f0), F = D = 0, D_k += F00_k (k = 0 .. 23), F += D, D = 0,
    -- D_k += D0_k and EM_k = its top bits, one Y step.  Line end (its first
    -- load lands just after the last pixel step): D_k += KC'_k + Ysh (D was
    -- D0_k + 2 xs^2 (W + 4), so it becomes KC_k + Ysh), then D_k += P2_k - Ysh
    -- (back to D0_k) with F += D on the first of these loads; one Y step (Ysh =
    -- the floor difference of the y quadratic).  Reads are issued one per
    -- clock and reach the circles 5 clocks later; the line end takes 57
    -- clocks (the tester's fake 64-clock blanking fits).
    -- Pixel steps: W + 4 clocks from stage 5 (F: pixel -2 .. W + 2).
    ---------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_t : signed(32 downto 0);
        variable v_fs : std_logic;
    begin
        if rising_edge(clk) then
            c_frst <= '0'; c_drst <= '0';
            r_v0 <= '0'; r_h0 <= '0'; r_f0 <= '0';
            v_fs := '0';
            if s_zgo = '1' then q_fsr <= '1'; end if;

            -- pixel steps: W + 4 clocks from stage 5 (st_gd(3) = the g_start clock + 4)
            st_gd <= st_gd(2 downto 0) & g_start;
            if st_gd(3) = '1' and q_ph = 0 and q_fsr = '0' then
                c_step <= '1'; st_n <= s_wb + 3;
            elsif c_step = '1' then
                if st_n = 0 then c_step <= '0'; else st_n <= st_n - 1; end if;
            end if;

            case to_integer(q_ph) is
                when 0 =>                                  -- idle
                    q_k <= (others => '0');
                    if q_fsr = '1' and c_step = '0' and s_zgo = '0' then
                        q_fsr <= '0'; q_ph <= to_unsigned(1, 4);
                    elsif c_step = '1' and st_n = 5 then   -- the line end: its first load lands just after
                        q_ph <= to_unsigned(4, 4);         -- the last pixel step
                    end if;
                when 1 =>                                  -- field start: the shared words (slot 24: 0, 1, 3)
                    r_v0 <= '1'; r_h0 <= '1'; r_k0 <= to_unsigned(24, 5);
                    case to_integer(q_k) is
                        when 0      => r_s0 <= "00";
                        when 1      => r_s0 <= "01";
                        when others => r_s0 <= "11";
                    end case;
                    if q_k = 2 then q_k <= (others => '0'); q_ph <= to_unsigned(2, 4); c_frst <= '1'; c_drst <= '1';
                    else q_k <= q_k + 1; end if;
                when 2 =>                                  -- field start: D_k += F00_k
                    r_v0 <= '1'; r_k0 <= q_k; r_s0 <= "00";
                    if q_k = NC - 1 then q_k <= (others => '0'); q_ph <= to_unsigned(5, 4);
                    else q_k <= q_k + 1; end if;
                when 5 =>                                  -- field start: drain (5 clocks), then F += D
                    if q_k = 4 then q_k <= (others => '0'); q_ph <= to_unsigned(6, 4); v_fs := '1';
                    else q_k <= q_k + 1; end if;
                when 6 =>                                  -- field start: D = 0
                    c_drst <= '1'; q_ph <= to_unsigned(7, 4);
                when 7 =>                                  -- field start: D_k += D0_k (and EM_k)
                    r_v0 <= '1'; r_k0 <= q_k; r_s0 <= "10";
                    if q_k = NC - 1 then q_k <= (others => '0'); q_ph <= to_unsigned(8, 4);
                    else q_k <= q_k + 1; end if;
                when 4 =>                                  -- line end, pass 1: D_k += KC'_k + Ysh (-> KC_k + Ysh)
                    r_v0 <= '1'; r_k0 <= q_k; r_s0 <= "01";
                    if q_k = NC - 1 then q_k <= (others => '0'); q_ph <= to_unsigned(9, 4);
                    else q_k <= q_k + 1; end if;
                when 9 =>                                  -- line end, pass 2: D_k += P2_k - Ysh (-> D0_k); F += D
                    r_v0 <= '1'; r_k0 <= q_k; r_s0 <= "11";  -- lands with the first of these loads
                    if q_k = 0 then r_f0 <= '1'; end if;
                    if q_k = NC - 1 then q_k <= (others => '0'); q_ph <= to_unsigned(8, 4);
                    else q_k <= q_k + 1; end if;
                when 8 =>                                  -- drain, then one Y step
                    if q_k = 4 then
                        q_k <= (others => '0'); q_ph <= (others => '0');
                        v_t := resize(signed('0' & y_f), 33) + resize(y_dy, 33);
                        y_sh <= resize(shift_right(v_t, 12), 20);
                        y_f  <= unsigned(v_t(11 downto 0));
                        y_dy <= y_dy + y_ay;
                    else q_k <= q_k + 1; end if;
                when others =>
                    q_ph <= (others => '0');
            end case;

            -- the read pipeline: address (with tag 1), RAM register (2), fabric register (3), then the bus and the
            -- circles' enables
            bk_ra <= (not s_eb) & r_k0 & r_s0;
            r_v1 <= r_v0; r_s1 <= r_s0; r_k1 <= r_k0; r_h1 <= r_h0; r_f1 <= r_f0;
            r_v2 <= r_v1; r_s2 <= r_s1; r_k2 <= r_k1; r_h2 <= r_h1; r_f2 <= r_f1;
            bk_f <= bk_q;
            r_v3 <= r_v2; r_s3 <= r_s2; r_k3 <= r_k2; r_h3 <= r_h2; r_f3 <= r_f2;
            c_ld <= '0'; c_lem <= '0';
            c_fsq <= v_fs or r_f3;
            if r_v3 = '1' and r_h3 = '1' then
                case to_integer(r_s3) is
                    when 0      => y_dy <= signed(bk_f);
                    when 1      => y_ay <= signed(bk_f);
                    when others => s_wb <= unsigned(bk_f(11 downto 0)); y_f <= unsigned(bk_f(27 downto 16));
                end case;
            elsif r_v3 = '1' then
                c_ld <= '1'; c_lk <= r_k3;
                if r_s3 = "10" then c_lem <= '1'; end if;
                c_bem <= bk_f(31 downto 27);
            end if;
            if r_v3 = '1' and r_h3 = '0' then
                if r_s3 = "01" then
                    c_bus <= signed(bk_f(26 downto 0)) + resize(y_sh, 27);
                elsif r_s3 = "11" then
                    c_bus <= signed(bk_f(26 downto 0)) - resize(y_sh, 27);
                else
                    c_bus <= signed(bk_f(26 downto 0));
                end if;
            else
                -- the pixel steps: 2 xs^2 + the camera's second difference
                c_bus <= shift_left(resize(p_d1, 27) + resize(p_c2d, 27), 6) + signed(resize(s_xc, 27));
            end if;
        end if;
    end process p_seq;

    ---------------------------------------------------------------------------
    -- [P] the camera into the bus: c(i) = the stream (x 64) for pixels 0 .. W - 1
    -- (stage 3), d1 = c(i) - 2 c(i - 1) and c(i - 2) (stage 4), the bus at stage 5
    -- holds c(i) - 2 c(i - 1) + c(i - 2) + 2 xs^2 = the second difference for
    -- the step from pixel i - 2 (F at stage 7)
    ---------------------------------------------------------------------------
    p_cam : process(clk)
    begin
        if rising_edge(clk) then
            s_xc <= resize(shift_left(resize(s_xs, 6) * resize(s_xs, 3), 1), 6);
            if s_gav_sr(0) = '1' then p_c0 <= signed(resize(s_Lw, 17));
            else p_c0 <= (others => '0'); end if;
            p_c1 <= p_c0; p_c2 <= p_c1;
            p_d1 <= resize(p_c0, 19) - shift_left(resize(p_c1, 19), 1);
            p_c2d <= p_c2;
        end if;
    end process p_cam;

    ---------------------------------------------------------------------------
    -- [P] PIXEL PATH, one per circle.  F (27 bits, 1/4 virtual px^2 units,
    -- offset by 2^(10 + e)) steps by D, D by the bus.  Coverage: the window
    -- F~ >> (5 + e) (6 bits, e = 0 .. 3) and the zero test of F~ >> (11 + e),
    -- in two mux stages; x 3/4 if m; Bold clamp(8 - H), Fine clamp(24 - |H|);
    -- outside the window Bold = 16 below (inside the disc), 0 above.
    ---------------------------------------------------------------------------
    g_circ : for i in 0 to NC - 1 generate
        signal b_F, b_D : signed(26 downto 0) := (others => '0');
        signal b_em : std_logic_vector(4 downto 0) := "00001";      -- (static during a field)
        signal s8_w : unsigned(6 downto 0) := (others => '0');
        signal s8_z : std_logic_vector(1 downto 0) := (others => '0');
        signal s8_g, s9_g, s9_n : std_logic := '0';
        signal s9_h : signed(6 downto 0) := (others => '0');
    begin
        p_dda : process(clk)
        begin
            if rising_edge(clk) then
                if c_frst = '1' then b_F <= (others => '0');
                elsif c_step = '1' or c_fsq = '1' then b_F <= b_F + b_D; end if;
                if c_drst = '1' then b_D <= (others => '0');
                elsif c_step = '1' or (c_ld = '1' and c_lk = i) then b_D <= b_D + c_bus; end if;
                if c_lem = '1' and c_lk = i then b_em <= c_bem; end if;
            end if;
        end process p_dda;

        p_cov : process(clk)
            variable v_z14, v_z13, v_z12, v_z11 : std_logic;
            variable v_ht, v_hb : unsigned(5 downto 0);
            variable v_h : signed(6 downto 0);
            variable v_v : signed(7 downto 0);
        begin
            if rising_edge(clk) then
                -- 8: the window F~(5 + 2 e1 .. 11 + 2 e1) and the zero tests of F~(26 .. 11 + 2 e1 + r), r = 0, 1
                if b_F(26 downto 14) = 0 then v_z14 := '1'; else v_z14 := '0'; end if;
                v_z13 := v_z14 and not b_F(13);
                v_z12 := v_z13 and not b_F(12);
                v_z11 := v_z12 and not b_F(11);
                if b_em(3) = '1' then
                    s8_w <= unsigned(b_F(13 downto 7)); s8_z <= v_z14 & v_z13;
                else
                    s8_w <= unsigned(b_F(11 downto 5)); s8_z <= v_z12 & v_z11;
                end if;
                s8_g <= b_F(26);
                -- 9: shift by e0: the window (6 bits) - 32 (the top bit flipped) and its zero test; x 3/4 if m;
                -- Fine: |H| (ones' complement)
                if b_em(2) = '1' then v_ht := s8_w(6 downto 1); s9_n <= s8_z(1);
                else v_ht := s8_w(5 downto 0); s9_n <= s8_z(0); end if;
                v_hb := v_ht; v_hb(5) := not v_ht(5);
                v_h := resize(signed(v_hb), 7);
                if b_em(1) = '1' then v_h := v_h - shift_right(v_h, 2); end if;
                if s_pf = '1' and v_h(6) = '1' then s9_h <= not v_h; else s9_h <= v_h; end if;
                s9_g <= s8_g;
                -- 10: coverage 0 .. 16: Bold 8 - H, Fine 24 - |H|, clamped; outside the window Bold 16 below, 0
                -- above, Fine 0
                v_v := signed(resize(s_pk, 8)) - resize(s9_h, 8);
                if b_em(0) = '1' then s10_t(i) <= (others => '0');
                elsif s9_n = '0' then
                    if s9_g = '1' and s_pf = '0' then s10_t(i) <= to_unsigned(16, 5);
                    else s10_t(i) <= (others => '0'); end if;
                elsif v_v(7) = '1' then s10_t(i) <= (others => '0');
                elsif v_v(6 downto 4) /= "000" then s10_t(i) <= to_unsigned(16, 5);
                else s10_t(i) <= unsigned(v_v(4 downto 0)); end if;
                s10_n(i) <= s9_n and not b_em(0);
            end if;
        end process p_cov;
    end generate g_circ;

    ---------------------------------------------------------------------------
    -- [P] the summed coverage, the crowd count and the tone (stages 11 .. 16:
    -- the signals keep the names s13 .. s18)
    ---------------------------------------------------------------------------
    p_sum : process(clk)
        variable v_w : unsigned(4 downto 0);
        variable v_s6 : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            s_pf <= s_fine; s_pi <= s_ink;
            if s_fine = '1' then s_pk <= to_unsigned(24, 5); else s_pk <= to_unsigned(8, 5); end if;
            for j in 0 to 11 loop
                s13_a(j) <= resize(s10_t(2 * j), 6) + resize(s10_t(2 * j + 1), 6);
                s13_n(j) <= ('0' & s10_n(2 * j)) + ('0' & s10_n(2 * j + 1));
            end loop;
            for j in 0 to 5 loop
                s14_a(j) <= resize(s13_a(2 * j), 7) + resize(s13_a(2 * j + 1), 7);
                s14_n(j) <= resize(s13_n(2 * j), 3) + resize(s13_n(2 * j + 1), 3);
            end loop;
            for j in 0 to 2 loop
                s15_a(j) <= resize(s14_a(2 * j), 8) + resize(s14_a(2 * j + 1), 8);
                s15_n(j) <= resize(s14_n(2 * j), 4) + resize(s14_n(2 * j + 1), 4);
            end loop;
            s16_a <= resize(s15_a(0), 9) + resize(s15_a(1), 9); s16_b <= s15_a(2);
            s16_na <= resize(s15_n(0), 5) + resize(s15_n(1), 5); s16_nb <= s15_n(2);
            s17_s <= s16_a + resize(s16_b, 9);
            s17_n <= s16_na + resize(s16_nb, 5);
            -- 18: the tone's address: ink & fine & crowd (n - 2, 0 .. 3) & the sum (Fine Grey: at most 63)
            if s17_n < 3 then v_w := (others => '0');
            elsif s17_n > 5 then v_w := to_unsigned(3, 5);
            else v_w := s17_n - 2; end if;
            if s_pf = '1' and s_pi = '0' then
                if s17_s > 63 then v_s6 := to_unsigned(63, 6); else v_s6 := s17_s(5 downto 0); end if;
            else
                v_s6 := s17_s(5 downto 0);
            end if;
            s18_ya <= s_pi & s_pf & v_w(1 downto 0) & v_s6;
        end if;
    end process p_sum;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or 3/4 of the Osc lattice) -> the per-field table
    -- T(l) = K6-gain * l^2 -> box of 64 (1080) or 32 samples, edge-replicated and
    -- reloaded at every line start -> vertical one-pole in the line store (half
    -- horizontal resolution, Q13.3).  The pixel path takes the one-pole's stream
    -- as it is written (16 bits): the camera is never shown.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(12 downto 0);
        variable v_d : signed(16 downto 0);
        variable v_u : signed(16 downto 0);
        variable v_mn : unsigned(16 downto 0);
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

            -- the source sample: (luma >> 2) - 16, or 3/4 of the lattice -> the table (p_tbram) -> s_p
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_l8 <= v_s - shift_right(v_s, 2);
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
            if s_lg6 = '1' then s_ebx <= s_bsum(18 downto 6); else s_ebx <= s_bsum(17 downto 5); end if;
            s_eq <= lb_q;
            s_ev <= s_bv and s_bcol(0); s_ecol <= s_bcol;
            v_d := signed(resize(s_ebx & "000", 17)) - signed(resize(s_eq, 17));
            case to_integer(s_ish) is
                when 2      => v_u := shift_right(v_d, 2);
                when 3      => v_u := shift_right(v_d, 3);
                when others => v_u := shift_right(v_d, 4);
            end case;
            lb_wd <= unsigned(signed(s_eq) + v_u(15 downto 0));
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            -- the stream to the pixel path: the latest and the previous column's values, their mean on the
            -- clock in between
            c_new <= lb_we;
            if lb_we = '1' then
                c_b <= c_a; c_a <= lb_wd;
            end if;
            v_mn := resize(c_a, 17) + resize(c_b, 17);
            if c_new = '1' then s_Lw <= v_mn(16 downto 1); else s_Lw <= c_a; end if;

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
    -- [F] OUTPUT CONDITIONING: tone -> luma (ROM, stage 17; fabric 18), black
    -- outside pixels 0 .. W + 1 (19), luma [1 2 1] / 4 (22, 23), 13/16 toward
    -- black (24), blanking gate (25).  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s18_y <= yr_q;
            if s_gav_sr(16) = '1' or s_gav_sr(18) = '1' then s19_y <= s18_y; else s19_y <= to_unsigned(64, 10); end if;
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

end architecture limacon;
