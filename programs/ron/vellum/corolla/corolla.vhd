-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Corolla v0.1.0 -- Vellum.  The overlapping-circles grid (the "Flower of Life", Leonardo's studies in the
-- Codex Atlanticus; Japanese shippo): translucent vellum discs, one on every point of a triangular (or square)
-- lattice, their overlaps building petals.  Luma only: U = V = 512 on every pixel.
--
--   lattice: hex pitch P (K3) or square spacing sqrt2 P, turned by theta, stretched along x by K4, panned by
--   K1 / K2; disc radius R (P12 Bloom, R = P at the detent) moved by the camera about its field mean
--   (u, w)  = lattice coordinates (22-bit DDAs over 3 pitches x 1 cell (hex) / 2 x 2 spacings (square))
--   fold    = the quarter cell (mirror at the half cell in u and w); 10 candidate centres (hex) / 6 (square)
--   d_k^2   = (a_k - u)^2 + (c_k - w)^2 = Q(u) + K_k -+ m_k L(u) + the same in w; Q = G u^2, L = G u from
--             two per-field tables (512 x 32), G = 8 P_a / R (edge units: 1/16 AA px, per lattice unit^2)
--   e_k     = T(L) - d_k^2  (T = the disc's scaled r^2 (+ h^2) by camera level)
--   Bold    cov_k = clamp(8 + e_k)          Fine  cov_k = clamp(8 + h - |e_k|)      (17 levels per edge)
--   Grey    tone = triangle(sum w_k cov_k / 128): the films of the three sublattices weigh 1, 3, 4 (square:
--           the checkerboard's two weigh 1, 3), so every overlap count folds into four stepped greys
--   Ink     tone = triangle(sum cov_k / 32)   (the parity of the overlap count: XOR, anti-aliased)
--
--   K1  X        pan +-512 v-px (the turn's centre moves with it)      K2  Y   the same, down
--   K3  Scale    pitch 48 .. 384 virtual px (exponential)
--   K4  Oval     horizontal stretch 1 .. 2 (exponential): discs to ovals, petals lean as the lattice turns
--   K5  Drift    bipolar speed (Hypnos): glide along the lattice's own row (wraps exactly one period of the
--                tile and its colouring) and a slow turn (wraps exactly 60 / 90 degrees)
--   K6  Camera   the box-filtered luma moves R about the field's mean level (0.45 pitch per full swing at
--                100 %): brighter than the mean = bigger discs, darker = smaller; a black input moves nothing
--   S7  Glide    Off / On      S8 Lattice  Hex / Square      S9 Tone  Grey / Ink      S10 Weight  Bold / Fine
--   S11 Source   Video / Osc
--   P12 BLOOM    R / pitch 0.35 (dots) .. 0.5 (kissing coins) .. 1.0 (the Flower of Life, a detent) .. 1.3 (lace)
--
-- The chassis is the Kinetic riley shell (via Vellum rosette), with Hardline flute's per-field engine:
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B][C] ENGINE                        flute's microcoded per-field engine (37-bit accumulator, register file
--                                        in block RAM, serial 18 x 18 multiplier; the program in corolla.py does
--                                        the holds, glides, Drift speed / phase / turn and every constant), the
--                                        table fills, line DDAs and shared pixel offsets
--   [D] SOURCE FIELD                     luma (or the Osc lattice) -> box -> vertical IIR -> camera level
--   [E] DISCS                            fold, tables, candidate distances, coverage, films, tone
--   [F] OUTPUT CONDITIONING              luma [1 2 1]/4, 13/16 toward black, blanking gate
--
-- Stage numbering: stage 1 is the generated avid's own clock; a register "at stage S" holds pixel n at
-- T + S + n (T = the g_start clock).  Pixel n sits at x = x0 + n xs half virtual px.  The output register is
-- at stage C_LAT - 3 and the generated avid lands 4 clocks after the delayed source avid (the shell's
-- parity-locked start), so output column j shows pixel j + 4 on screen column j + 4.  One latency (32) in
-- every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.corolla_pkg.all;
use work.all;

architecture corolla of program_top is
    constant C_LAT : integer := 32;   -- = processing_delay_clks

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
    -- the folded lattice fraction: mirrored at the half cell into 0 .. 511 (index i stands for (i + 1/2) / 1024)
    function f_fold(f : unsigned(9 downto 0)) return unsigned is
    begin
        if f(9) = '1' then return not f(8 downto 0); end if;
        return f(8 downto 0);
    end function;
    -- coverage clamp(8 + x, 0, 16) of a 12-bit signed edge distance (1/16 px)
    function f_cov(x : signed(11 downto 0)) return unsigned is
    begin
        if x(11) = '1' and x(10 downto 3) /= "11111111" then return to_unsigned(0, 5);
        elsif x(11) = '0' and x(10 downto 3) /= "00000000" then return to_unsigned(16, 5);
        else return unsigned(x(4 downto 0)) + 8;
        end if;
    end function;
    -- saturate an 18-bit signed edge distance to 11 bits (+-1023: far outside every ring and edge window)
    function f_sat11(x : signed(17 downto 0)) return signed is
    begin
        if x(17) = '0' and x(16 downto 10) /= "0000000" then return to_signed(1023, 11);
        elsif x(17) = '1' and x(16 downto 10) /= "1111111" then return to_signed(-1024, 11);
        else return x(10 downto 0);
        end if;
    end function;

    attribute no_rw_check : boolean;

    type t_s11x10 is array (0 to 9) of signed(10 downto 0);
    type t_s18x6  is array (0 to 5) of signed(17 downto 0);
    type t_s18x5  is array (0 to 4) of signed(17 downto 0);
    type t_s18x4  is array (0 to 3) of signed(17 downto 0);
    type t_s18x3  is array (0 to 2) of signed(17 downto 0);
    type t_u5x10  is array (0 to 9) of unsigned(4 downto 0);
    type t_u2x10  is array (0 to 9) of unsigned(1 downto 0);
    type t_u7x10  is array (0 to 9) of unsigned(6 downto 0);
    type t_u7x4   is array (0 to 3) of unsigned(6 downto 0);
    type t_fl     is array (13 to 16) of std_logic_vector(4 downto 0);    -- I(1:0), J, mu, mw by stage

    -- candidate k uses X variant XA(k) and W variant WB(k); the square lattice uses SQON(k) only
    type t_int10 is array (0 to 9) of integer;
    constant XA   : t_int10 := (0, 1, 3, 2, 0, 3, 1, 2, 4, 5);
    constant WB   : t_int10 := (0, 1, 0, 1, 2, 2, 3, 3, 0, 1);
    constant SQON : std_logic_vector(0 to 9) := "1010111010";

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_wc   : unsigned(1 downto 0) := (others => '0');     -- AA unit: 1 / 1.5 / 2 / 3 virtual px
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal w_dxm  : unsigned(10 downto 0);     -- the beam's pre-pixel: its distance left of the centre (h)

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
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k): pixels 0 .. W - 1 at stage k + 2
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 9) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- switches (latched at the zone start)
    signal s_glide : std_logic := '1';
    signal s_sq, s_ink, s_fine, s_src : std_logic := '0';
    signal c_Lm   : unsigned(7 downto 0) := (others => '0');       -- mean camera level of the last fields

    -- [B][C] THE PER-FIELD ENGINE (Hardline flute's): micro-op = 4-bit opcode + 12-bit operand
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
    constant OP_SPC  : std_logic_vector(3 downto 0) := x"F";      -- 0 return, 1 halt, 2 normalise
    signal rf_ram : t_c_co_rfi := C_CO_RFI;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal e_run : std_logic := '0';
    signal e_ph  : unsigned(1 downto 0) := "11";
    signal e_pc, e_lr : unsigned(9 downto 0) := (others => '0');
    signal e_ir  : std_logic_vector(15 downto 0) := x"F001";      -- the microprogram ROM's output register
    signal e_irf : std_logic_vector(15 downto 0) := x"F001";      -- its fabric copy (from phase 1 on)
    signal x_mul, x_shr, x_nrm, x_halt, x_out, x_st : std_logic := '0';   -- decoded at phase 1
    signal x_gate, x_subr, x_shift, x_cz : std_logic := '0';              -- the adder's controls, as registers
    signal e_acc, e_b : signed(36 downto 0) := (others => '0');
    signal e_cnt : unsigned(4 downto 0) := (others => '0');
    signal e_out : std_logic;
    signal w_in  : unsigned(15 downto 0);
    signal ft_adr : unsigned(9 downto 0) := (others => '0');
    signal ft_q  : std_logic_vector(15 downto 0) := (others => '0');

    -- the engine's outputs: DDA steps (Q22), the candidates' constants, the fills' constants
    signal c_SA, c_SB, c_TA, c_TB : unsigned(21 downto 0) := (others => '0');
    signal c_cq   : unsigned(17 downto 0) := (others => '0');      -- G: edge units per lattice unit^2
    signal c_Ke2, c_Kw1, c_Kw2 : unsigned(17 downto 0) := (others => '0');   -- 2.25 G; w constants (Kw2 = Gw)
    signal c_hb   : unsigned(11 downto 0) := (others => '0');      -- Fine: h in 1/16 px (Bold: 0)
    signal c_Acs, c_H2s, c_Gs : unsigned(15 downto 0) := (others => '0');

    -- [C] table fills (started by the engine's T0 output)
    signal sf_run, sf_ph : std_logic := '0';
    signal sf_i   : unsigned(8 downto 0) := (others => '0');
    signal sf_S   : unsigned(37 downto 0) := (others => '0');
    signal sf_D   : unsigned(29 downto 0) := (others => '0');
    signal sf_L   : unsigned(26 downto 0) := (others => '0');
    signal tu_we, tw_we : std_logic := '0';
    signal t_wa   : unsigned(8 downto 0) := (others => '0');
    signal t_wd   : unsigned(31 downto 0) := (others => '0');
    signal rf_run : std_logic := '0';
    signal rf_acc : signed(25 downto 0) := (others => '0');
    signal rt_we  : std_logic := '0';
    signal rt_wa  : unsigned(7 downto 0) := (others => '0');
    signal rt_wd  : unsigned(15 downto 0) := (others => '0');

    -- [C] line DDAs, shared pixel offsets, the lattice coordinates (top 14 bits)
    signal l_U, l_W : unsigned(21 downto 0) := (others => '0');
    signal l_first : std_logic := '0';
    signal o_A, o_B : unsigned(21 downto 0) := (others => '0');
    signal s10_u, s10_w : unsigned(13 downto 0) := (others => '0');

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
    signal c_b64 : std_logic := '1';                               -- box length 64 (1080 lines) / 32 pixels
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
    signal h_cur, h_prev, s8_q : unsigned(11 downto 0) := (others => '0');
    signal h_ph : std_logic := '0';
    signal lm_acc : unsigned(27 downto 0) := (others => '0');      -- running mean of the camera level, Q20

    -- [E] the two per-field tables (Q = G t^2 in the high half, L = G t (u) / 3G t, 2G t (w) in the low half)
    type t_tab is array (0 to 511) of unsigned(31 downto 0);
    signal tu_ram, tw_ram : t_tab := (others => (others => '0'));
    attribute no_rw_check of tu_ram : signal is true;
    attribute no_rw_check of tw_ram : signal is true;
    signal tu_q, tw_q, tu_f, tw_f : unsigned(31 downto 0) := (others => '0');
    type t_rt is array (0 to 255) of unsigned(15 downto 0);
    signal rt_ram : t_rt := (others => (others => '0'));
    attribute no_rw_check of rt_ram : signal is true;
    signal rt_a : unsigned(7 downto 0) := (others => '0');
    signal rt_q, rt_f, rt_g, s13_T : unsigned(15 downto 0) := (others => '0');

    -- [E] pixel path
    signal s11_ux : unsigned(15 downto 0) := (others => '0');
    signal s11_wa : unsigned(8 downto 0) := (others => '0');
    signal s11_mw, s11_J : std_logic := '0';
    signal s12_fl : std_logic_vector(4 downto 0) := (others => '0');
    signal s_fl_d : t_fl := (others => (others => '0'));
    signal s14_P  : t_s18x5 := (others => (others => '0'));       -- P1 .. P5
    signal s14_D  : signed(17 downto 0) := (others => '0');
    signal s14_V  : t_s18x3 := (others => (others => '0'));       -- V1, V2, V3
    signal s14_Qw : signed(17 downto 0) := (others => '0');
    signal s15_X  : t_s18x6 := (others => (others => '0'));       -- X'0 .. X'5
    signal s15_W  : t_s18x4 := (others => (others => '0'));
    signal s16_e  : t_s11x10 := (others => (others => '0'));
    signal s17_c  : t_u5x10 := (others => (others => '0'));
    signal s17_col : t_u2x10 := (others => (others => '0'));
    signal s18_w  : t_u7x10 := (others => (others => '0'));
    signal s19_s  : t_u7x4 := (others => (others => '0'));
    signal s20_T  : unsigned(6 downto 0) := (others => '0');
    signal tn_a   : unsigned(5 downto 0) := (others => '0');
    signal tn_q, s23_y : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- [F] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_yd : t_u10(1 to 3) := (others => to_unsigned(64, 10));
    signal s26_ya : unsigned(11 downto 0) := (others => '0');
    signal s26_yb : unsigned(9 downto 0) := (others => '0');
    signal s27_y, s28_y, s29_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            tn_q   <= C_CO_TONE(to_integer(tn_a));
        end if;
    end process p_rom;

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

    -- the u and w tables: written in the vblank zone only, read by the pixel path (stage-11 folded addresses)
    p_tu : process(clk) begin
        if rising_edge(clk) then
            tu_q <= tu_ram(to_integer(f_fold(s11_ux(13 downto 4))));
            if tu_we = '1' then tu_ram(to_integer(t_wa)) <= t_wd; end if;
        end if;
    end process p_tu;
    p_tw : process(clk) begin
        if rising_edge(clk) then
            tw_q <= tw_ram(to_integer(s11_wa));
            if tw_we = '1' then tw_ram(to_integer(t_wa)) <= t_wd; end if;
        end if;
    end process p_tw;

    -- camera table: written in the vblank zone only, read by the pixel path (stage 9 address)
    p_rtram : process(clk)
    begin
        if rising_edge(clk) then
            rt_q <= rt_ram(to_integer(rt_a));
            if rt_we = '1' then rt_ram(to_integer(rt_wa)) <= rt_wd; end if;
        end if;
    end process p_rtram;

    -- [A] the beam's pre-pixel by video mode
    w_dxm <= to_unsigned(1444, 11) when s_xs = 4 else to_unsigned(1923, 11) when s_xs = 3 else to_unsigned(1922, 11);

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES, per-field latches
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

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_sq    <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                c_Lm    <= lm_acc(27 downto 20);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is                  -- Osc step, box length, its read offset
                    when 4      => s_oxs <= to_unsigned(204, 9); c_b64 <= '0'; c_brp <= to_unsigned(225, 8);
                    when 3      => s_oxs <= to_unsigned(153, 9); c_b64 <= '0'; c_brp <= to_unsigned(225, 8);
                    when others => s_oxs <= to_unsigned(102, 9); c_b64 <= '1'; c_brp <= to_unsigned(193, 8);
                end case;
                -- the anti-aliasing unit: 1 v-px at 1080p, 1.5 at 1080i / 720p, 2 at 480p / 576p, 3 at 480i / 576i
                if s_xsr = 4 then
                    if s_ys >= 960 then s_wc <= "11"; else s_wc <= "10"; end if;
                elsif s_xsr = 3 or s_ys >= 512 then s_wc <= "01";
                else s_wc <= "00"; end if;
                -- the vertical IIR's shift: about 32 .. 48 virtual lines in every mode
                if s_ys >= 960 then c_iir <= to_unsigned(3, 3);
                elsif s_ys >= 480 then c_iir <= to_unsigned(4, 3);
                else c_iir <= to_unsigned(5, 3); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

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
    -- [B][C] THE PER-FIELD ENGINE (Hardline flute's).  Started at every zone
    -- start from address 0 (so a stalled or garbled run cannot outlive a
    -- field); three clocks per micro-op (register-file read, the ROM word
    -- copied to fabric / operand, decode and next address / execute), serial
    -- multiply (18 steps) and shift.  ACC = 37 bits; MUL = ACC(17:0) x
    -- operand(17:0), both signed, by add-and-shift-right with the last step
    -- subtracting.  The microprogram (corolla.py) does the knob holds and
    -- glides, the Drift speed, phase and turn, and every per-field constant.
    ---------------------------------------------------------------------------
    with e_irf(3 downto 0) select w_in <=
        resize(s_xs, 16)            when x"8",
        resize(s_ys, 16)            when x"9",
        resize(s_wc, 16)            when x"A",
        unsigned(ft_q)              when x"B",
        resize(e_cnt, 16)           when x"C",
        resize(c_Lm, 16)            when x"D",
        resize(w_dxm, 16)           when x"E",
        (others => '0')             when x"F",
        resize(unsigned(registers_in(to_integer(unsigned(e_irf(2 downto 0))))), 16) when others;

    e_out <= '1' when e_run = '1' and e_ph = "10" and x_out = '1' else '0';

    p_eng : process(clk)
        variable v_op  : std_logic_vector(3 downto 0);
        variable v_arg : std_logic_vector(11 downto 0);
        variable v_gate, v_sub, v_shift, v_last, v_cond, v_fetch, v_nz : std_logic;
        variable v_bx  : signed(36 downto 0);
        variable v_t   : signed(37 downto 0);
        variable v_sum : signed(36 downto 0);
    begin
        if rising_edge(clk) then
            v_op := e_irf(15 downto 12);
            v_arg := e_irf(11 downto 0);
            v_fetch := '0';
            ft_q <= C_CO_FT(to_integer(ft_adr));
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
                        elsif v_op = OP_SHR then e_cnt <= unsigned(v_arg(4 downto 0)) - 1;
                        elsif v_op = OP_SPC then e_cnt <= (others => '0'); end if;
                        case v_arg(2 downto 0) is
                            when "000"  => v_cond := e_acc(36);
                            when "001"  => v_cond := s_glide;
                            when "010"  => v_cond := s_sq;
                            when others => v_cond := s_fine;
                        end case;
                        v_cond := v_cond xor v_arg(3);
                        if v_op = OP_SKIP and v_cond = '1' then e_pc <= e_pc + 2;
                        elsif v_op = OP_JMP then e_pc <= unsigned(v_arg(9 downto 0));
                        elsif v_op = OP_CALL then e_pc <= unsigned(v_arg(9 downto 0)); e_lr <= e_pc + 1;
                        elsif v_op = OP_SPC and v_arg(1 downto 0) = "00" then e_pc <= e_lr;
                        else e_pc <= e_pc + 1; end if;
                        x_mul <= '0'; x_shr <= '0'; x_nrm <= '0'; x_halt <= '0'; x_out <= '0'; x_st <= '0';
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
                            when OP_SPC  => x_nrm <= v_arg(1); x_halt <= v_arg(0) and not v_arg(1);
                            when others  => null;
                        end case;
                        e_ph <= "10";
                    when others =>                        -- execute (the adder sees registers only)
                        v_gate := x_gate; v_sub := x_subr; v_shift := x_shift; v_last := '1';
                        if e_acc(36 downto 5) = 0 then v_nz := '0'; else v_nz := '1'; end if;
                        if v_gate = '0' then v_bx := (others => '0');
                        elsif v_sub = '1' then v_bx := not e_b;
                        else v_bx := e_b; end if;
                        v_t := (e_acc & '1') + (v_bx & (v_sub and v_gate));
                        v_sum := v_t(37 downto 1);
                        if x_mul = '1' or x_shr = '1' then
                            if x_cz = '0' then v_last := '0'; end if;
                            e_cnt <= e_cnt - 1;
                            if e_cnt = 1 then x_cz <= '1'; x_subr <= x_mul; end if;   -- the next step is the last
                            if x_mul = '1' then x_gate <= v_sum(1); end if;           -- the next multiplier bit
                        elsif x_nrm = '1' and v_nz = '1' then          -- normalise: shift right until < 32
                            v_shift := '1'; v_last := '0';
                            e_cnt <= e_cnt + 1;
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
                e_ir <= C_CO_UC(to_integer(e_pc));
            end if;

            -- the engine's output registers (U0 / W0 load the line DDAs, T0 starts the fills: p_dda, p_fill)
            if e_out = '1' then
                case to_integer(unsigned(e_irf(4 downto 0))) is
                    when 0  => ft_adr <= unsigned(e_acc(9 downto 0));
                    when 1  => c_SA   <= unsigned(e_acc(21 downto 0));
                    when 2  => c_SB   <= unsigned(e_acc(21 downto 0));
                    when 3  => c_TA   <= unsigned(e_acc(21 downto 0));
                    when 4  => c_TB   <= unsigned(e_acc(21 downto 0));
                    when 7  => c_cq   <= unsigned(e_acc(17 downto 0));
                    when 8  => c_Ke2  <= unsigned(e_acc(17 downto 0));
                    when 9  => c_Kw1  <= unsigned(e_acc(17 downto 0));
                    when 10 => c_Kw2  <= unsigned(e_acc(17 downto 0));
                    when 11 => c_hb   <= unsigned(e_acc(11 downto 0));
                    when 13 => c_Gs   <= unsigned(e_acc(15 downto 0));
                    when 14 => c_H2s  <= unsigned(e_acc(15 downto 0));
                    when 15 => c_Acs  <= unsigned(e_acc(15 downto 0));
                    when others => null;
                end case;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- [C] TABLE FILLS (vblank zone, after the slots), by second differences:
    --   u: Q(i) = G (2i + 1)^2 >> 22,  L(i) = G (2i + 1) >> 11           (t = (i + 1/2) / 1024 cells)
    --   w: Q(i) = Gw (2i + 1)^2 >> 22, L(i) = Gw (2i + 1) >> 11 (hex) / >> 10 (square); Gw = 3G (hex) / G
    --   camera: T(L) = clamp(T0 + L Gs, h^2, cap^2 + h^2) (scaled), the radius moved about the mean level
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_t : signed(17 downto 0);
        variable v_g : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            tu_we <= '0'; tw_we <= '0';
            if e_out = '1' and e_irf(4 downto 0) = "01100" then
                sf_run <= '1'; sf_ph <= '0'; sf_i <= (others => '0');
                sf_S <= resize(c_cq, 38); sf_D <= resize(c_cq & "000", 30); sf_L <= resize(c_cq, 27);
            elsif sf_run = '1' then
                t_wa <= sf_i;
                if sf_ph = '1' and s_sq = '1' then t_wd <= sf_S(37 downto 22) & sf_L(25 downto 10);
                else t_wd <= sf_S(37 downto 22) & sf_L(26 downto 11); end if;
                tu_we <= not sf_ph; tw_we <= sf_ph;
                sf_i <= sf_i + 1;
                if sf_i = 511 then
                    sf_S <= resize(c_Kw2, 38); sf_D <= resize(c_Kw2 & "000", 30); sf_L <= resize(c_Kw2, 27);
                    sf_ph <= '1';
                    if sf_ph = '1' then sf_run <= '0'; end if;
                else
                    if sf_ph = '1' then v_g := c_Kw2; else v_g := c_cq; end if;
                    sf_S <= sf_S + sf_D; sf_D <= sf_D + resize(v_g & "000", 30); sf_L <= sf_L + resize(v_g & '0', 27);
                end if;
            end if;

            rt_we <= '0';
            if e_out = '1' and e_irf(4 downto 0) = "01100" then
                rf_run <= '1'; rt_wa <= (others => '1');
                rf_acc <= e_acc(25 downto 0);
            elsif rf_run = '1' then
                v_t := resize(shift_right(rf_acc, 8), 18);
                rt_we <= '1'; rt_wa <= rt_wa + 1;
                if v_t < signed(resize(c_H2s, 18)) then rt_wd <= c_H2s;
                elsif v_t > signed(resize(c_Acs, 18)) then rt_wd <= c_Acs;
                else rt_wd <= unsigned(v_t(15 downto 0)); end if;
                rf_acc <= rf_acc + signed(resize(c_Gs, 26));
                if rt_wa = 254 then rf_run <= '0'; end if;
            end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- [C] ENGINE, line DDAs and pixel offsets.  l_U / l_W hold this line's
    -- lattice coordinates at the pre-pixel: loaded with line 0 from the
    -- start-value accumulators in the zone, stepped at every line start but
    -- the field's first (so they hold through the line).  The two pixel
    -- offsets restart at the pre-pixel; pixel 0 is at stage 9.  Stage 10 adds
    -- them on the top 14 bits (Q22, modulo the span): u + n SA, w - n SB.
    ---------------------------------------------------------------------------
    p_dda : process(clk)
    begin
        if rising_edge(clk) then
            if s_zone = '1' then l_first <= '1'; elsif s_st(0) = '1' then l_first <= '0'; end if;
            if e_out = '1' and e_irf(4 downto 0) = "00101" then l_U <= unsigned(e_acc(21 downto 0));
            elsif s_st(0) = '1' and l_first = '0' then l_U <= l_U + c_TA; end if;
            if e_out = '1' and e_irf(4 downto 0) = "00110" then l_W <= unsigned(e_acc(21 downto 0));
            elsif s_st(0) = '1' and l_first = '0' then l_W <= l_W + c_TB; end if;

            if s_st(6) = '1' then
                o_A <= (others => '0'); o_B <= (others => '0');
            else
                o_A <= o_A + c_SA; o_B <= o_B + c_SB;
            end if;

            -- 10: the lattice coordinates (14 bits; the low 8 bits of the sum are dropped)
            s10_u <= l_U(21 downto 8) + o_A(21 downto 8);
            s10_w <= l_W(21 downto 8) - o_B(21 downto 8);
        end if;
    end process p_dda;

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
    -- resolution; the first line of a field loads it) -> camera level 0 .. 255.
    -- The pixel path taps the store's write data (the camera is never shown, so
    -- it need not be aligned to the pixel).
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l   : unsigned(7 downto 0);
        variable v_s   : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_d   : signed(12 downto 0);
        variable v_dd  : signed(12 downto 0);
        variable v_n   : signed(12 downto 0);
        variable v_m   : unsigned(12 downto 0);
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

            -- the pixel path's tap: the last two columns written; stage 8 interpolates between them on the
            -- clock after a write; stage 9 addresses the camera table
            if lb_we = '1' then
                h_cur <= lb_wd; h_prev <= h_cur; h_ph <= '1';
            else
                h_ph <= '0';
            end if;
            v_m := resize(h_cur, 13) + resize(h_prev, 13);
            if h_ph = '1' then s8_q <= v_m(12 downto 1); else s8_q <= h_cur; end if;
            rt_a <= s8_q(11 downto 4);
            -- the camera's running mean level (about one field): the camera moves the radius about it
            if s_gav_sr(7) = '1' then lm_acc <= lm_acc + rt_a - lm_acc(27 downto 20); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] DISCS (stages 11 .. 23); edge distances in 1/16 AA px
    --   11  u3 = 3u (hex) / 2u (square): the u-cell index and fraction; the folded w address
    --   12  tables (RAM)  13 fabric        (the camera table: 9 address, 10 RAM, 11 .. 13 fabric)
    --   14  P = K -+ m L(u), D = T - Q(u), V = K -+ m L(w)
    --   15  X'a = D - Pa (a = 0 .. 5), W = Q(w) + V
    --   16  e_k = X'a(k) - Wb(k), saturated to 11 bits
    --   17  Bold x = e, Fine x = h - |e|; coverage clamp(8 + x); the candidate's film (colour)
    --   18  weighted coverage           19, 20  sum modulo 128
    --   21  tone (Grey: triangle of the 7 bits; Ink: of the low 5)            22  tone ROM     23  fabric
    ---------------------------------------------------------------------------
    p_disc : process(clk)
        variable v_ux  : unsigned(15 downto 0);
        variable v_w10 : unsigned(9 downto 0);
        variable v_Lu, v_Lw, v_Qu, v_Qw : signed(17 downto 0);
        variable v_o   : unsigned(1 downto 0);
        variable v_3   : unsigned(2 downto 0);
        variable v_f   : integer range 0 to 3;
        variable v_c   : unsigned(6 downto 0);
        variable v_b   : std_logic;
        variable v_x   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 11
            v_ux := resize(s10_u, 16) + shift_left(resize(s10_u, 16), 1);     -- (a shift for the square, no self-add)
            if s_sq = '1' then s11_ux <= shift_left(resize(s10_u, 16), 1); else s11_ux <= v_ux; end if;
            if s_sq = '1' then v_w10 := s10_w(12 downto 3); s11_J <= s10_w(13);
            else v_w10 := s10_w(13 downto 4); s11_J <= '0'; end if;
            s11_wa <= f_fold(v_w10);
            s11_mw <= v_w10(9);
            -- 12 (RAM): flags I(1:0), J, mu, mw
            if s_sq = '1' then s12_fl(4 downto 3) <= '0' & s11_ux(14);
            else s12_fl(4 downto 3) <= std_logic_vector(s11_ux(15 downto 14)); end if;
            s12_fl(2) <= s11_J; s12_fl(1) <= s11_ux(13); s12_fl(0) <= s11_mw;
            -- 13
            tu_f <= tu_q; tw_f <= tw_q;
            rt_f <= rt_q; rt_g <= rt_f; s13_T <= rt_g;
            s_fl_d(13) <= s12_fl;
            for i in 14 to 16 loop s_fl_d(i) <= s_fl_d(i - 1); end loop;
            -- 14
            v_Qu := signed(resize(tu_f(31 downto 16), 18)); v_Lu := signed(resize(tu_f(15 downto 0), 18));
            v_Qw := signed(resize(tw_f(31 downto 16), 18)); v_Lw := signed(resize(tw_f(15 downto 0), 18));
            if s_sq = '1' then s14_P(0) <= (others => '0');                                   -- square: X1 = u^2
            else s14_P(0) <= signed(resize(shift_right(c_cq, 2), 18)) - v_Lu; end if;
            s14_P(1) <= signed(resize(shift_right(c_cq, 2), 18)) + v_Lu;
            s14_P(2) <= signed(resize(c_cq, 18)) - shift_left(v_Lu, 1);
            s14_P(3) <= signed(resize(c_cq, 18)) + shift_left(v_Lu, 1);
            s14_P(4) <= signed(resize(c_Ke2, 18)) - v_Lu - shift_left(v_Lu, 1);
            s14_D <= signed(resize(s13_T, 18)) - v_Qu;
            s14_V(0) <= signed(resize(c_Kw1, 18)) - v_Lw;
            if s_sq = '1' then s14_V(1) <= signed(resize(c_Kw2, 18)) - v_Lw;
            else s14_V(1) <= signed(resize(c_Kw2, 18)) - shift_left(v_Lw, 1); end if;
            s14_V(2) <= signed(resize(c_Kw1, 18)) + v_Lw;
            s14_Qw <= v_Qw;
            -- 15
            s15_X(0) <= s14_D;
            for a in 1 to 5 loop s15_X(a) <= s14_D - s14_P(a - 1); end loop;
            s15_W(0) <= s14_Qw;
            for b in 1 to 3 loop s15_W(b) <= s14_Qw + s14_V(b - 1); end loop;
            -- 16
            for k in 0 to 9 loop s16_e(k) <= f_sat11(s15_X(XA(k)) - s15_W(WB(k))); end loop;
            -- 17: Bold x = 0 + e, Fine x = h -+ e (- outside the ring's centre); coverage; the film
            v_f := to_integer(unsigned(s_fl_d(16)(1 downto 0)));
            for k in 0 to 9 loop
                v_x := signed(f_addsub(c_hb(11 downto 0), unsigned(resize(s16_e(k), 12)), s_fine and not s16_e(k)(10)));
                if s_sq = '1' and SQON(k) = '0' then s17_c(k) <= (others => '0');
                else s17_c(k) <= f_cov(v_x); end if;
                if s_sq = '1' then
                    case v_f is
                        when 0      => v_b := C_CO_COLS(k)(0);
                        when 1      => v_b := C_CO_COLS(k)(1);
                        when 2      => v_b := C_CO_COLS(k)(2);
                        when others => v_b := C_CO_COLS(k)(3);
                    end case;
                    s17_col(k) <= '0' & (s_fl_d(16)(3) xor s_fl_d(16)(2) xor v_b);
                else
                    case v_f is
                        when 0      => v_o := unsigned(C_CO_COLH(k)(1 downto 0));
                        when 1      => v_o := unsigned(C_CO_COLH(k)(3 downto 2));
                        when 2      => v_o := unsigned(C_CO_COLH(k)(5 downto 4));
                        when others => v_o := unsigned(C_CO_COLH(k)(7 downto 6));
                    end case;
                    v_3 := resize(unsigned(s_fl_d(16)(4 downto 3)), 3) + resize(v_o, 3);
                    if v_3 >= 3 then v_3 := v_3 - 3; end if;
                    s17_col(k) <= v_3(1 downto 0);
                end if;
            end loop;
            -- 18: the films weigh 1, 3, 4 (Ink: all 1)
            for k in 0 to 9 loop
                v_c := resize(s17_c(k), 7);
                if s_ink = '1' or s17_col(k) = "00" then s18_w(k) <= v_c;
                elsif s17_col(k) = "01" then s18_w(k) <= v_c + shift_left(v_c, 1);
                else s18_w(k) <= shift_left(v_c, 2); end if;
            end loop;
            -- 19, 20: the sum, modulo 128
            s19_s(0) <= s18_w(0) + s18_w(1) + s18_w(2);
            s19_s(1) <= s18_w(3) + s18_w(4) + s18_w(5);
            s19_s(2) <= s18_w(6) + s18_w(7) + s18_w(8);
            s19_s(3) <= s18_w(9);
            s20_T <= (s19_s(0) + s19_s(1)) + (s19_s(2) + s19_s(3));
            -- 21: the tone: Grey = triangle over 128 (6 bits), Ink = triangle over 32 (4 bits, widened)
            if s_ink = '1' then
                if s20_T(4) = '1' then tn_a <= (not s20_T(3 downto 0)) & (not s20_T(3 downto 2));
                else tn_a <= s20_T(3 downto 0) & s20_T(3 downto 2); end if;
            else
                if s20_T(6) = '1' then tn_a <= not s20_T(5 downto 0);
                else tn_a <= s20_T(5 downto 0); end if;
            end if;
            -- 22 (ROM), 23
            s23_y <= tn_q;
        end if;
    end process p_disc;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING (stages 24 .. 29): luma [1 2 1]/4, luma 13/16
    -- toward black (white 776), blanking gate.  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 24 .. 26: taps y(n + 1), y(n), y(n - 1)
            s_yd(1) <= s23_y;
            for i in 2 to 3 loop s_yd(i) <= s_yd(i - 1); end loop;
            s26_ya <= resize(s_yd(1), 12) + resize(s_yd(3), 12) + 2;
            s26_yb <= s_yd(2);
            -- 27
            v_y := s26_ya + shift_left(resize(s26_yb, 12), 1);
            s27_y <= v_y(11 downto 2);
            -- 28: luma 13/16 toward black
            if s27_y > 64 then v_dl := s27_y - 64; else v_dl := (others => '0'); end if;
            s28_y <= s27_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            -- 29: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s29_y <= s28_y;
            else
                s29_y <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s29_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture corolla;
