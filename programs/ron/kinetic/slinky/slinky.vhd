-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Slinky v0.1.0 -- Kinetic.
--
-- One outline shape followed by a train of graded copies along a swinging path (motion-graphics echo tubes,
-- the Slinky): an extruded, coiling tube.
--
--   element  copy j = 0 .. C_N-1 (C_N = 13; the head is j = 0, drawn on top)
--   state    its centre = path(phase - Spread * j);  path = (Ax sin(a), Ay sin(b))
--              Swing: a = theta, b = 2 theta (figure-eight);  Orbit: a = theta + 1/4, b = theta (ellipse)
--              Snap: theta dwells on 8 stations per cycle and eases between them (75 % dwell)
--   size     R_j = R0 + j * dR (Shape: cone .. cylinder .. flare)
--   test     per copy and pixel  f = x^2 - L_j,  L_j = 2 x cx - cx^2 + H2 + M  (one linear accumulator per
--            copy against the shared x^2);  H2 per copy and line from the line engine:
--              Ring    H2 = R^2 - dy^2
--              Square  H2 = R^2 on the body rows, R^2 - T + (R^2 - dy^2) on the cap rows (rounded corners)
--            outer hit f < 0, inner hit f + T + 2M < 0 (T = outline = 3/32 Rmax^2 + 4 Rmax, shared);
--            M = 2^n > Rmax is the AA half-ramp, so coverage = (-f) >> (n - 2) in eighths
--   colour   grade (a ROM from slinky.py): accent (head) -> B -> A -> halfway to the ground (tail); a per-field
--            table T(copy, coverage) = ground + (grade - ground) * coverage / 8;  Solid: the fill of the top
--            copy, its outline in the ground colour over the copy below;  Wire: the outlines alone, see-through
--   video    the picture's luma (or the Osc lattice) is added inside the drawn copies, times the coverage
--
--   K1  Scale    head radius 32 .. 255 virtual px (exponential, glided)
--   K2  Shape    taper along the train: cone (tail = 1/2) .. cylinder .. flare (tail = 2x)
--   K3  Depth    how far the path swings (0 .. 900 px across, half that down)
--   K4  Rate     bipolar, centre = still; 60 s per cycle .. 2 cycles per second
--   K5  Spread   phase step per copy 0 .. 1/8 cycle (square law)
--   K6  Colour   the 16 shared palettes
--   S7  Wave     Smooth / Snap
--   S8  Field    Swing / Orbit
--   S9  Form     Ring / Square
--   S10 Body     Solid / Wire
--   S11 Source   Video / Osc
--   P12 EXTRUDE  1 .. C_N copies, the newest fading in (glided)
--
-- Fit: one copy costs about 143 logic cells (a 23-bit accumulator, its 23-bit step register, two 20-bit
-- tests, its share of the priority mux), so C_N is 13 where the spec asked for up to 24.
--
-- Sections (the Kinetic chassis, lifted from louver):
--   [A] timing ID -> virtual-1080 steps, vblank zone, width, knob glides, per-field sequencer (serial multiplier)
--   [B] generated avid
--   [C] source: Osc lattice / luma, delay ring
--   [D] the copies: per-field loop, per-line engine, hblank transfer, per-pixel tests
--   [E] priority, coverage, colour table
--   [F] output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1] with resync, luma 13/16, blanking gate
-- One latency (C_LAT = 16) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.slinky_pkg.all;
use work.all;

architecture slinky of program_top is
    constant C_N        : integer := C_SK_N;  -- copies (slinky.py)
    constant C_G        : integer := (C_N + 7) / 8;
    constant C_LAT      : integer := 16;      -- = processing_delay_clks
    constant C_SIM_SNAP : boolean := false;   -- sim-only copies set this: knob glides jump straight to the knob
    constant C_DRK      : integer := 2048 / (C_N - 1);   -- taper per copy: Q6 step = R0 |k2| C_DRK >> 14

    -- [A] virtual-1080 steps from the timing ID: y step per field line (Q8), x step in half virtual px
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
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    function f_fold(a : unsigned(11 downto 0)) return unsigned is
    begin
        if a(10) = '1' then return not a(9 downto 0); else return a(9 downto 0); end if;
    end function;
    function f_gu(x : unsigned; b : std_logic; w : natural) return unsigned is
        variable r : unsigned(w - 1 downto 0) := (others => '0');
    begin
        if b = '1' then r := resize(x, w); end if;
        return r;
    end function;
    function f_gs(x : signed; b : std_logic; w : natural) return signed is
        variable r : signed(w - 1 downto 0) := (others => '0');
    begin
        if b = '1' then r := resize(x, w); end if;
        return r;
    end function;
    -- index of a one-hot vector
    function f_enc(h : unsigned(C_N - 1 downto 0)) return unsigned is
        variable r : unsigned(4 downto 0) := (others => '0');
    begin
        for j in 0 to C_N - 1 loop
            if h(j) = '1' then r := r or to_unsigned(j, 5); end if;
        end loop;
        return r;
    end function;
    -- coverage in eighths from a window: w(6 downto 0) = bits 7 .. 1 of the magnitude, w(7) = any higher bit
    function f_cov(w : std_logic_vector(7 downto 0); s : unsigned(1 downto 0)) return unsigned is
        variable v   : unsigned(2 downto 0);
        variable sat : std_logic;
    begin
        case to_integer(s) is
            when 0      => v := unsigned(w(2 downto 0)); sat := w(7) or w(6) or w(5) or w(4) or w(3);
            when 1      => v := unsigned(w(3 downto 1)); sat := w(7) or w(6) or w(5) or w(4);
            when 2      => v := unsigned(w(4 downto 2)); sat := w(7) or w(6) or w(5);
            when others => v := unsigned(w(5 downto 3)); sat := w(7) or w(6);
        end case;
        if sat = '1' then return to_unsigned(8, 4); else return '0' & v; end if;
    end function;

    attribute no_rw_check : boolean;

    -- ROMs (registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_w_a  : unsigned(9 downto 0) := (others => '0');
    signal r_w_q  : unsigned(15 downto 0) := (others => '0');
    signal r_pl_a : unsigned(8 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    -- [D] per-copy store: word (j, w): 0 = E = R^2 - dy^2, 1 = dy, 2 = P, 3 = R^2, 4 = line start (bit 31 = body), 5 = x step
    type t_r32 is array (0 to 255) of std_logic_vector(31 downto 0);
    signal cs : t_r32 := (others => (others => '0'));
    attribute no_rw_check of cs : signal is true;
    signal cs_ra, fw_a, lw_a : unsigned(7 downto 0) := (others => '0');
    signal cs_q, cs_f, fw_d, lw_d : std_logic_vector(31 downto 0) := (others => '0');
    signal fw_we, lw_we : std_logic := '0';
    -- [E] colour table, refilled every field: T(copy, al) = ground + (grade - ground) al / 8; {Y 10, U 8, V 8}
    type t_tt is array (0 to 511) of std_logic_vector(31 downto 0);
    signal tt : t_tt := (others => (others => '0'));
    attribute no_rw_check of tt : signal is true;
    signal tt_ra, tt_wa : unsigned(8 downto 0) := (others => '0');
    signal tt_q, tt_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal tt_we : std_logic := '0';
    -- delay rings (write at rc + d, read at rc)
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal vr : t_r16 := (others => (others => '0'));          -- source luma
    attribute no_rw_check of vr : signal is true;
    signal vr_q, vr_f : std_logic_vector(15 downto 0) := (others => '0');
    signal cr : t_r16 := (others => (others => '0'));          -- chroma 8 back for the running 8-sums
    attribute no_rw_check of cr : signal is true;
    signal cr_q, cr_f : std_logic_vector(15 downto 0) := (others => '0');
    type t_r4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal sr : t_r4 := (others => (others => '1'));           -- syncs: C_LAT
    attribute no_rw_check of sr : signal is true;
    signal sr_q, sr_f : std_logic_vector(3 downto 0) := (others => '1');
    signal rc   : unsigned(7 downto 0) := (others => '0');

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_sd   : std_logic := '0';
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw rate, switches, palette
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(620 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(870 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_orb, s_sq, s_wire, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(4, 4);
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- field serial multiplier (14 iterations), sequencer scratch
    signal m_a, m_acc, z_fa : unsigned(31 downto 0) := (others => '0');
    signal m_b, z_fb : unsigned(13 downto 0) := (others => '0');
    signal z_ld  : std_logic := '0';
    signal z_e   : unsigned(10 downto 0) := (others => '0');
    signal z_sh  : unsigned(2 downto 0) := (others => '0');
    signal z_shn : unsigned(2 downto 0) := (others => '0');
    signal z_mt  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_z   : unsigned(17 downto 0) := (others => '0');
    signal z_fl  : std_logic := '0';

    -- per-field constants
    signal c_r0   : unsigned(8 downto 0) := to_unsigned(99, 9);        -- head radius
    signal c_rmax : unsigned(8 downto 0) := to_unsigned(99, 9);
    signal c_drq  : unsigned(10 downto 0) := (others => '0');           -- |radius step per copy|, Q6
    signal c_drneg : std_logic := '0';
    signal c_ax, c_ay : unsigned(9 downto 0) := (others => '0');       -- path amplitudes
    signal c_spr  : unsigned(10 downto 0) := (others => '0');          -- phase step per copy, 14 bits per cycle
    signal c_cntq : unsigned(7 downto 0) := to_unsigned(8, 8);         -- copies, Q3
    signal c_nfull, c_nact : unsigned(4 downto 0) := to_unsigned(1, 5);
    signal c_frac : unsigned(2 downto 0) := (others => '0');
    signal c_shm  : unsigned(1 downto 0) := "01";                      -- coverage shift - 1
    signal c_m    : unsigned(9 downto 0) := to_unsigned(128, 10);      -- AA half-ramp M = 2^n >= Rmax
    signal c_t    : unsigned(14 downto 0) := to_unsigned(1000, 15);    -- outline: R^2 - Rin^2
    signal c_tu   : unsigned(11 downto 0) := to_unsigned(157, 12);     -- (T + 2 M) / 8
    signal c_xh   : unsigned(11 downto 0) := to_unsigned(1920, 12);    -- half the width in half virtual px
    signal c_x0sq : unsigned(21 downto 0) := (others => '0');
    signal c_dx0  : signed(15 downto 0) := (others => '0');
    signal c_ddx  : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal c_ph   : unsigned(25 downto 0) := (others => '0');          -- wave phase, Q26 cycles
    signal c_dph  : unsigned(21 downto 0) := (others => '0');
    signal c_gy   : unsigned(9 downto 0) := to_unsigned(64, 10);       -- ground colour (chroma in 8 bits)
    signal c_gu, c_gv : unsigned(7 downto 0) := to_unsigned(128, 8);

    -- per-field copy loop
    signal p_run, p_done : std_logic := '0';
    signal p_j    : unsigned(4 downto 0) := (others => '0');
    signal p_u    : unsigned(6 downto 0) := (others => '0');
    signal p_th, p_thp : unsigned(13 downto 0) := (others => '0');
    signal p_racc : unsigned(14 downto 0) := (others => '0');
    signal p_rj   : unsigned(8 downto 0) := (others => '0');
    signal p_en, p_nx, p_ny : std_logic := '0';
    signal p_ev, p_sx, p_sy, p_cxm, p_cym, p_yc : unsigned(9 downto 0) := (others => '0');
    signal p_ay   : unsigned(11 downto 0) := (others => '0');
    signal p_xc   : unsigned(11 downto 0) := (others => '0');
    signal p_r2   : unsigned(17 downto 0) := (others => '0');

    -- [B] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_avd : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- [C] source
    signal s_ox, s_oy : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_v9  : unsigned(8 downto 0) := (others => '0');

    -- [D] line engine, transfer, the copies
    signal l_yf   : unsigned(7 downto 0) := (others => '0');
    signal le_dl  : unsigned(2 downto 0) := (others => '0');
    signal le_run, le_zero : std_logic := '0';
    signal le_j   : unsigned(4 downto 0) := (others => '0');
    signal le_u   : unsigned(3 downto 0) := (others => '0');
    signal le_e, le_et, le_h2, le_ls : signed(22 downto 0) := (others => '0');
    signal le_dy  : signed(12 downto 0) := (others => '0');
    signal le_v   : signed(13 downto 0) := (others => '0');
    signal le_m   : signed(17 downto 0) := (others => '0');
    signal le_r2  : unsigned(17 downto 0) := (others => '0');
    signal le_body, le_fm, le_fz : std_logic := '0';
    signal t_run, t_ph : std_logic := '0';
    signal t_cnt  : unsigned(5 downto 0) := (others => '0');
    signal t_tk   : std_logic_vector(C_N - 1 downto 0) := (others => '0');
    signal x_sq   : unsigned(21 downto 0) := (others => '0');
    signal x_dx   : signed(15 downto 0) := (others => '0');
    signal x_s    : unsigned(16 downto 0) := (others => '0');
    signal x_t    : unsigned(19 downto 0) := (others => '0');
    type t_s23 is array (0 to C_N - 1) of signed(22 downto 0);
    signal a_l, a_st : t_s23 := (others => (others => '0'));
    signal a_body : std_logic_vector(C_N - 1 downto 0) := (others => '0');
    type t_d is array (0 to C_N - 1) of std_logic_vector(16 downto 0);
    signal q_d    : t_d := (others => (others => '0'));       -- stage 2: {ib, g hi, g window 7, f hi, f window 7}
    signal q_h    : std_logic_vector(C_N - 1 downto 0) := (others => '0');

    -- [E] priority, coverage, colour
    type t_m is array (0 to C_G - 1) of std_logic_vector(16 downto 0);
    signal r3_m   : t_m := (others => (others => '0'));
    signal r3_hot, r3_rest, r3_rib, r4_hot2, r4_h2ib : unsigned(C_N - 1 downto 0) := (others => '0');
    signal r3_any, r4_any1, r4_any2, r5_any1, r5_any2, r5_uf, r5_ib : std_logic := '0';
    signal r4_d   : std_logic_vector(16 downto 0) := (others => '0');
    signal r4_j, r5_j, r5_k : unsigned(4 downto 0) := (others => '0');
    signal r5_co, r5_ai, r6_al, r7_al : unsigned(3 downto 0) := (others => '0');
    signal r8_t   : std_logic_vector(25 downto 0) := (others => '0');
    signal r8_vp  : unsigned(8 downto 0) := (others => '0');
    signal q9_y   : unsigned(9 downto 0) := to_unsigned(64, 10);
    type t_u8 is array (1 to 2) of unsigned(7 downto 0);
    signal q9_c   : t_u8 := (others => to_unsigned(128, 8));

    -- colour table fill
    signal f_run, f_w : std_logic := '0';
    signal f_st   : unsigned(1 downto 0) := (others => '0');
    signal f_j    : unsigned(4 downto 0) := (others => '0');
    signal f_al   : unsigned(3 downto 0) := (others => '0');
    signal f_g    : std_logic_vector(25 downto 0) := (others => '0');
    signal f_ey   : signed(13 downto 0) := (others => '0');
    signal f_eu, f_ev : signed(11 downto 0) := (others => '0');

    -- [F] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_ysr  : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv, s_bud, s_bvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y31, s_y32, s_y33 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u31, s_v31, s_u32, s_v32, s_u33, s_v33 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Block RAM: ROMs, the per-copy store, the colour table, delay rings (one
    -- write statement each; none ever reads the address it is writing)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_SK_EXP2(to_integer(r_ex_a));
            r_w_q  <= C_SK_SIN(to_integer(r_w_a));
            r_pl_q <= C_SK_GRD(to_integer(r_pl_a));
        end if;
    end process p_rom;

    p_cs : process(clk)
        variable v_a : unsigned(7 downto 0);
        variable v_d : std_logic_vector(31 downto 0);
    begin
        if rising_edge(clk) then
            cs_q <= cs(to_integer(cs_ra));
            cs_f <= cs_q;
            if lw_we = '1' then v_a := lw_a; v_d := lw_d; else v_a := fw_a; v_d := fw_d; end if;
            if (lw_we or fw_we) = '1' then cs(to_integer(v_a)) <= v_d; end if;
        end if;
    end process p_cs;

    p_tt : process(clk)
    begin
        if rising_edge(clk) then
            tt_q <= tt(to_integer(tt_ra));
            if tt_we = '1' then tt(to_integer(tt_wa)) <= tt_wd; end if;
        end if;
    end process p_tt;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            vr(to_integer(rc + 7)) <= "0000000" & std_logic_vector(s_v9);
            vr_q <= vr(to_integer(rc));
            vr_f <= vr_q;
            cr(to_integer(rc + 6)) <= std_logic_vector(q9_c(1)) & std_logic_vector(q9_c(2));
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            sr(to_integer(rc + (C_LAT - 2))) <= '1' & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
            rc <= rc + 1;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- [A] Controls, vblank zone, glides, per-field sequencer (one small
    -- operation per zone clock; quasi-static but timed at the pixel clock),
    -- then the per-copy loop (C_N copies, 90 clocks each)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_b  : unsigned(25 downto 0);
        variable v_a27 : unsigned(26 downto 0);
        variable v_x  : unsigned(9 downto 0);
        variable v_w  : unsigned(13 downto 0);
        variable v_r  : unsigned(9 downto 0);
        variable v_a12, v_t12, v_ax : unsigned(11 downto 0);
        variable v_e11 : unsigned(10 downto 0);
        variable v_p  : signed(22 downto 0);
        variable v_c  : signed(13 downto 0);
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

            -- Rate: bipolar with a +-8 deadband
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
                s_snap <= registers_in(6)(0);
                s_orb  <= registers_in(6)(1);
                s_sq   <= registers_in(6)(2);
                s_wire <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); s_sd <= '1';
                    when 3      => s_oxs <= to_unsigned(153, 9); s_sd <= '0';
                    when others => s_oxs <= to_unsigned(102, 9); s_sd <= '0';
                end case;
                -- Colour: 16 palettes with hysteresis
                v_d := signed(resize(unsigned(registers_in(5)), 11)) - signed(resize(s_pal & "100000", 11));
                if v_d > 40 or v_d < -40 then s_pal <= unsigned(registers_in(5)(9 downto 6)); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide for Scale and Extrude (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        if z_gc(0) = '0' then s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                        else s_gm <= s_gp; s_gr <= unsigned(registers_in(7)); end if;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if C_SIM_SNAP then
                            null;
                        elsif s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        if z_gc(0) = '0' then s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        else s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0)); end if;
                        z_gp <= "00";
                        if z_gc = 1 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial multiplier: a state sets z_fa, z_fb, z_ld; the product is readable 16 clocks later
            if z_ld = '1' then
                m_acc <= (others => '0');
                m_a <= z_fa; m_b <= z_fb;
            else
                if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
                m_a <= m_a(30 downto 0) & '0';
                m_b <= '0' & m_b(13 downto 1);
            end if;
            z_ld <= '0';
            z_fa <= (others => '0'); z_fb <= (others => '0');

            -- serial shifter for the rate's 2^n
            if z_shn /= 0 then
                z_shn <= z_shn - 1;
                c_dph <= c_dph(20 downto 0) & '0';
            end if;

            fw_we <= '0';
            p_done <= '0';

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- wave phase (last field's step): one adder, add or subtract
                    when 20 =>
                        v_b := resize(c_dph, 26);
                        if s_vneg = '1' then v_b := not v_b; end if;
                        v_a27 := (c_ph & '1') + (v_b & s_vneg);
                        c_ph <= v_a27(26 downto 1);
                    -- half the width in half virtual px
                    when 30 =>
                        case to_integer(s_xs) is
                            when 4      => c_xh <= s_w(10 downto 0) & '0';
                            when 3      => c_xh <= resize(shift_right(resize(s_w, 14) + shift_left(resize(s_w, 14), 1), 1), 12);
                            when others => c_xh <= s_w;
                        end case;

                    -- head radius: e = k1 * 3 octaves (Q8), R0 = 32 * 2^e
                    when 40 => z_fa(12 downto 0) <= s_g1; z_fb <= to_unsigned(6144, 14); z_ld <= '1';
                    when 56 =>
                        z_e <= resize(m_acc(25 downto 16), 11);
                        z_fa(9 downto 0) <= unsigned(registers_in(2)); z_fb <= to_unsigned(900, 14); z_ld <= '1';    -- depth
                    when 57 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 59 => z_mt <= r_ex_q;
                    when 60 =>
                        case to_integer(z_sh) is
                            when 0      => c_r0 <= "000" & z_mt(15 downto 10);
                            when 1      => c_r0 <= "00" & z_mt(15 downto 9);
                            when others => c_r0 <= '0' & z_mt(15 downto 8);
                        end case;
                    when 72 =>
                        v_x := m_acc(19 downto 10);
                        if s_sd = '1' then v_x := v_x - shift_right(v_x, 2); end if;
                        c_ax <= v_x; c_ay <= shift_right(v_x, 1);
                        z_fa(9 downto 0) <= unsigned(registers_in(4)); z_fb(9 downto 0) <= unsigned(registers_in(4)); z_ld <= '1';   -- spread^2
                    when 88 =>
                        c_spr <= m_acc(19 downto 9);
                        z_fa(13 downto 0) <= resize(s_gp, 14) + 8; z_fb <= to_unsigned(C_N - 1, 14); z_ld <= '1';    -- copies
                    when 104 =>
                        c_cntq <= m_acc(17 downto 10) + 8;
                        z_fa(8 downto 0) <= c_r0; z_ld <= '1';                                                    -- taper
                        z_fl <= registers_in(1)(9);
                        if registers_in(1)(9) = '1' then z_fb(8 downto 0) <= unsigned(registers_in(1)(8 downto 0));
                        else z_fb(9 downto 0) <= to_unsigned(512, 10) - unsigned(registers_in(1)(8 downto 0)); end if;
                    when 105 =>
                        c_nfull <= c_cntq(7 downto 3); c_frac <= c_cntq(2 downto 0);
                        if c_cntq(2 downto 0) = 0 then c_nact <= c_cntq(7 downto 3); else c_nact <= c_cntq(7 downto 3) + 1; end if;
                    when 120 =>
                        z_z <= m_acc(17 downto 0);
                        z_fa(17 downto 0) <= m_acc(17 downto 0); z_fb <= to_unsigned(C_DRK, 14); z_ld <= '1';
                    when 121 =>
                        if z_fl = '1' then c_rmax <= c_r0 + z_z(17 downto 9); else c_rmax <= c_r0; end if;
                    when 136 =>
                        if z_fl = '1' then c_drq <= m_acc(24 downto 14); else c_drq <= '0' & m_acc(24 downto 15); end if;
                        c_drneg <= not z_fl;
                        z_fa(8 downto 0) <= c_rmax; z_fb(8 downto 0) <= c_rmax; z_ld <= '1';                       -- Rmax^2
                    when 137 =>
                        if c_rmax(8) = '1' then    c_shm <= "11"; c_m <= to_unsigned(512, 10);
                        elsif c_rmax(7) = '1' then c_shm <= "10"; c_m <= to_unsigned(256, 10);
                        elsif c_rmax(6) = '1' then c_shm <= "01"; c_m <= to_unsigned(128, 10);
                        else                       c_shm <= "00"; c_m <= to_unsigned(64, 10); end if;
                    when 152 =>
                        c_t <= (resize(m_acc(17 downto 4), 15) + resize(m_acc(17 downto 5), 15)) + resize(c_rmax & "00", 15);
                        z_fa(11 downto 0) <= c_xh; z_fb(11 downto 0) <= c_xh; z_ld <= '1';                         -- x0^2
                    when 153 => c_tu <= resize(shift_right(resize(c_t, 16) + resize(c_m & '0', 16), 3), 12);
                    when 168 =>
                        c_x0sq <= m_acc(21 downto 0);
                        z_fa(8 downto 0) <= s_vm; z_fb <= to_unsigned(900, 14); z_ld <= '1';                       -- rate exponent
                    when 169 =>
                        case to_integer(s_xs) is
                            when 4      => c_dx0 <= 16 - signed(resize(c_xh & "000", 16)); c_ddx <= to_unsigned(32, 6);
                            when 3      => c_dx0 <= 9 - signed(resize(c_xh & "00", 16)) - signed(resize(c_xh & '0', 16)); c_ddx <= to_unsigned(18, 6);
                            when others => c_dx0 <= 4 - signed(resize(c_xh & "00", 16)); c_ddx <= to_unsigned(8, 6);
                        end case;
                    -- rate: 2^(m * 6.9 / 503) / 3600 cycles per field
                    when 184 => z_e <= m_acc(18 downto 8);
                    when 185 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 187 => z_mt <= r_ex_q;
                    when 188 => z_fa(15 downto 0) <= z_mt; z_fb <= to_unsigned(9321, 14); z_ld <= '1';
                    when 204 =>
                        if s_vm = 0 then c_dph <= (others => '0'); else c_dph <= resize(m_acc(29 downto 14), 22); end if;
                        z_shn <= z_sh;
                    when 220 => p_run <= '1'; p_j <= (others => '0'); p_u <= (others => '0');
                    when others => null;
                end case;
            end if;

            -- the per-copy loop: centre, radius, the line engine's start values
            if p_run = '1' then
                p_u <= p_u + 1;
                fw_a <= p_j & "000";
                case to_integer(p_u) is
                    when 0 =>
                        if p_j = 0 then
                            p_th <= c_ph(25 downto 12);
                            p_racc <= c_r0 & "000000";
                        else
                            p_th <= p_th - resize(c_spr, 14);
                            if c_drneg = '1' then p_racc <= p_racc - resize(c_drq, 15);
                            else p_racc <= p_racc + resize(c_drq, 15); end if;
                        end if;
                    when 1 =>                                    -- snap easing: cos over the last quarter of the station
                        v_a12 := resize(p_th(8 downto 0) & "00", 12) + 1024;
                        r_w_a <= f_fold(v_a12); p_en <= v_a12(11);
                        p_rj <= p_racc(14 downto 6);
                    when 3 => p_ev <= r_w_q(9 downto 0);
                    when 4 =>
                        if p_en = '1' then v_e11 := to_unsigned(1024, 11) + resize(p_ev, 11);
                        else v_e11 := to_unsigned(1024, 11) - resize(p_ev, 11); end if;
                        if s_snap = '0' then p_thp <= p_th;
                        elsif p_th(10 downto 9) /= "11" then p_thp <= p_th(13 downto 11) & "00000000000";
                        else p_thp <= p_th(13 downto 11) & v_e11; end if;
                    when 5 =>
                        v_t12 := p_thp(13 downto 2);
                        if s_orb = '1' then v_ax := v_t12 + 1024; p_ay <= v_t12;
                        else v_ax := v_t12; p_ay <= p_thp(12 downto 1); end if;
                        r_w_a <= f_fold(v_ax); p_nx <= v_ax(11);
                    when 6 => r_w_a <= f_fold(p_ay); p_ny <= p_ay(11);
                    when 7 => p_sx <= r_w_q(9 downto 0);
                    when 8 =>
                        p_sy <= r_w_q(9 downto 0);
                        z_fa(9 downto 0) <= c_ax; z_fb(9 downto 0) <= p_sx; z_ld <= '1';
                    when 24 =>
                        p_cxm <= m_acc(19 downto 10);
                        z_fa(9 downto 0) <= c_ay; z_fb(9 downto 0) <= p_sy; z_ld <= '1';
                    when 40 =>
                        p_cym <= m_acc(19 downto 10);
                        z_fa(8 downto 0) <= p_rj; z_fb(8 downto 0) <= p_rj; z_ld <= '1';
                    when 41 =>
                        if p_nx = '1' then p_xc <= c_xh - resize(p_cxm, 12); else p_xc <= c_xh + resize(p_cxm, 12); end if;
                        if p_ny = '1' then p_yc <= to_unsigned(540, 10) - p_cym; else p_yc <= to_unsigned(540, 10) + p_cym; end if;
                    when 56 =>
                        p_r2 <= m_acc(17 downto 0);
                        fw_we <= '1'; fw_a <= p_j & "011"; fw_d <= "00000000000000" & std_logic_vector(m_acc(17 downto 0));
                        z_fa(11 downto 0) <= p_xc; z_fb(9 downto 0) <= p_cxm; z_ld <= '1';
                    when 72 =>                                   -- P = -cx (cx + xh) + M
                        if p_nx = '1' then v_p := signed(resize(c_m, 23)) + signed(m_acc(22 downto 0));
                        else v_p := signed(resize(c_m, 23)) - signed(m_acc(22 downto 0)); end if;
                        fw_we <= '1'; fw_a <= p_j & "010"; fw_d <= std_logic_vector(resize(v_p, 32));
                        z_fa(9 downto 0) <= p_yc; z_fb(9 downto 0) <= p_yc; z_ld <= '1';
                    when 73 =>                                   -- x step of L = cx * (half px per pixel)
                        case to_integer(s_xs) is
                            when 4      => v_c := signed(resize(p_cxm & "00", 14));
                            when 3      => v_c := signed(resize(p_cxm & '0', 14)) + signed(resize(p_cxm, 14));
                            when others => v_c := signed(resize(p_cxm & '0', 14));
                        end case;
                        if p_nx = '1' then v_c := -v_c; end if;
                        fw_we <= '1'; fw_a <= p_j & "101"; fw_d <= std_logic_vector(resize(v_c, 32));
                    when 74 =>                                   -- dy at line 0 = -(540 + cy)
                        fw_we <= '1'; fw_a <= p_j & "001"; fw_d <= std_logic_vector(resize(-signed(resize(p_yc, 13)), 32));
                    when 88 =>                                   -- E at line 0 = R^2 - dy^2
                        v_p := signed(resize(p_r2, 23)) - signed(resize(m_acc(19 downto 0), 23));
                        fw_we <= '1'; fw_a <= p_j & "000"; fw_d <= std_logic_vector(resize(v_p, 32));
                    when 89 =>
                        p_u <= (others => '0');
                        if p_j = C_N - 1 then p_run <= '0'; p_done <= '1';
                        else p_j <= p_j + 1; end if;
                    when others => null;
                end case;
            end if;
            if s_zone = '0' then p_run <= '0'; end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Colour table fill (zone; adders only).  The ground colour and each
    -- copy's grade colour come from the grade ROM; row j holds
    -- ground + (grade_j - ground) * al / 8 for al = 0 .. 8.
    ---------------------------------------------------------------------------
    p_fill : process(clk)
    begin
        if rising_edge(clk) then
            tt_we <= '0';
            f_g <= r_pl_q(31 downto 22) & r_pl_q(21 downto 14) & r_pl_q(11 downto 4);
            if s_zone = '1' and s_zc = 22 then
                r_pl_a <= s_pal & "11111";
            elsif s_zone = '1' and s_zc = 26 then
                c_gy <= unsigned(f_g(25 downto 16)); c_gu <= unsigned(f_g(15 downto 8)); c_gv <= unsigned(f_g(7 downto 0));
            elsif s_zone = '1' and s_zc = 110 then
                f_run <= '1'; f_st <= "00"; f_j <= (others => '0');
            elsif f_run = '1' then
                case f_st is
                    when "00" => r_pl_a <= s_pal & f_j; f_st <= "01";
                    when "01" => f_st <= "10";
                    when "10" =>
                        f_st <= "11"; f_al <= (others => '0'); f_w <= '1';
                        f_ey <= signed(resize(c_gy & "000", 14));
                        f_eu <= signed(resize(c_gu & "000", 12)); f_ev <= signed(resize(c_gv & "000", 12));
                    when others =>
                        if f_w = '1' then
                            f_w <= '0';                         -- f_g now holds the grade colour
                        else
                            tt_we <= '1'; tt_wa <= f_j & f_al;
                            tt_wd <= "000000" & std_logic_vector(f_ey(12 downto 3)) & std_logic_vector(f_eu(10 downto 3)) & std_logic_vector(f_ev(10 downto 3));
                            f_ey <= f_ey + signed(resize(unsigned(f_g(25 downto 16)), 14)) - signed(resize(c_gy, 14));
                            f_eu <= f_eu + signed(resize(unsigned(f_g(15 downto 8)), 12)) - signed(resize(c_gu, 12));
                            f_ev <= f_ev + signed(resize(unsigned(f_g(7 downto 0)), 12)) - signed(resize(c_gv, 12));
                            f_al <= f_al + 1;
                            if f_al = 8 then
                                f_st <= "00"; f_j <= f_j + 1;
                                if f_j = C_N - 1 then f_run <= '0'; end if;
                            end if;
                        end if;
                end case;
            end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- [B] Generated avid (parity-locked start 3/4 clocks after the source's
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
    -- [C] Source: the picture's luma (half scale), or the Osc: a two-axis
    -- triangle lattice (640 x 540 virtual px) drifting diagonally, at full scale
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_tx, v_ty : unsigned(6 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_o  : unsigned(7 downto 0);
        variable v_l  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;  -- ot * 96
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_tx := f_tri7(s_ox(15 downto 8)); v_ty := f_tri7(s_oy(15 downto 8));
            v_s := resize(v_tx, 8) + resize(v_ty, 8);
            v_o := v_s - shift_right(v_s, 3);

            if data_in.avid = '1' then
                if s_src = '1' then
                    s_v9 <= v_o & '0';
                else
                    v_l := unsigned(data_in.y);
                    if v_l > 64 then v_l := v_l - 64; else v_l := (others => '0'); end if;
                    s_v9 <= v_l(9 downto 1);
                end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [D] Line engine: during each active line it steps every copy's
    -- E = R^2 - dy^2 to the next line (second differences: E -= d (2 dy + d),
    -- dy += d, d = virtual lines per field line) and writes that line's start
    -- value P + H2.  Run once more at the end of the zone without stepping, for
    -- line 0.  Transfer: after each line (and after the zone pass) the start
    -- value and the x step of every copy go through the copy's step register
    -- into its accumulator, two clocks per copy.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y9 : unsigned(8 downto 0);
        variable v_dl : unsigned(2 downto 0);
        variable v_ja : unsigned(4 downto 0);
        variable v_on : std_logic;
    begin
        if rising_edge(clk) then
            lw_we <= '0';
            g_avd <= g_avid;
            if le_zero = '1' then v_dl := "000"; else v_dl := le_dl; end if;

            if s_zgo = '1' then
                l_yf <= (others => '0');
            elsif g_start = '1' then
                v_y9 := resize(l_yf, 9) + resize(s_ys(7 downto 0), 9);
                l_yf <= v_y9(7 downto 0);
                if v_y9(8) = '1' then le_dl <= s_ys(10 downto 8) + 1; else le_dl <= s_ys(10 downto 8); end if;
            end if;

            if g_start = '1' then
                le_run <= '1'; le_zero <= '0'; le_j <= (others => '0'); le_u <= (others => '0');
            elsif p_done = '1' then
                le_run <= '1'; le_zero <= '1'; le_j <= (others => '0'); le_u <= (others => '0');
            elsif le_run = '1' then
                le_u <= le_u + 1;
                case to_integer(le_u) is
                    when 0 => cs_ra <= le_j & "000";
                    when 1 => cs_ra <= le_j & "001";
                    when 2 => null;
                    when 3 => cs_ra <= le_j & "011"; le_e <= signed(cs_f(22 downto 0));
                    when 4 => le_dy <= signed(cs_f(12 downto 0));
                    when 5 => le_v <= shift_left(resize(le_dy, 14), 1) + signed(resize(v_dl, 14));
                    when 6 =>
                        le_r2 <= unsigned(cs_f(17 downto 0));
                        le_m <= f_gs(le_v, v_dl(0), 18) + shift_left(f_gs(le_v, v_dl(1), 18), 1);
                    when 7 => le_m <= le_m + shift_left(f_gs(le_v, v_dl(2), 18), 2);
                    when 8 =>
                        le_e <= le_e - resize(le_m, 23);
                        le_dy <= le_dy + signed(resize(v_dl, 13));
                        cs_ra <= le_j & "010";
                    when 9 =>
                        lw_we <= '1'; lw_a <= le_j & "000"; lw_d <= std_logic_vector(resize(le_e, 32));
                        le_et <= le_e - signed(resize(c_t, 23));
                        -- E + M < 0 (M = 2^n): E negative and its bits from n up not all ones
                        v_on := '1';
                        for i in 9 to 21 loop v_on := v_on and le_e(i); end loop;
                        if c_shm /= 3 then v_on := v_on and le_e(8); end if;
                        if c_shm(1) = '0' then v_on := v_on and le_e(7); end if;
                        if c_shm = 0 then v_on := v_on and le_e(6); end if;
                        le_fm <= le_e(22) and not v_on;
                        if le_e <= 0 then le_fz <= '1'; else le_fz <= '0'; end if;
                    when 10 =>
                        lw_we <= '1'; lw_a <= le_j & "001"; lw_d <= std_logic_vector(resize(le_dy, 32));
                        le_body <= '1';
                        if le_j >= c_nact then
                            le_h2 <= -signed(resize(c_m, 23)) - 8;
                        elsif s_sq = '0' then
                            if le_fm = '1' then le_h2 <= -signed(resize(c_m, 23)) - 8;
                            else le_h2 <= le_e; end if;
                        elsif le_fz = '1' then
                            le_h2 <= -signed(resize(c_m, 23)) - 8;
                        elsif le_et(22) = '1' then
                            le_h2 <= le_et + signed(resize(le_r2, 23)); le_body <= '0';
                        else
                            le_h2 <= signed(resize(le_r2, 23));
                        end if;
                    when 11 => le_ls <= signed(cs_f(22 downto 0)) + le_h2;
                    when others =>
                        lw_we <= '1'; lw_a <= le_j & "100"; lw_d <= le_body & "00000000" & std_logic_vector(le_ls);
                        le_u <= (others => '0');
                        if le_j = C_N - 1 then le_run <= '0'; else le_j <= le_j + 1; end if;
                end case;
            end if;

            -- transfer
            t_ph <= not t_ph;
            if t_ph = '1' then t_tk <= t_tk(C_N - 2 downto 0) & '0'; end if;
            if (g_avid = '0' and g_avd = '1') or (le_run = '1' and le_zero = '1' and le_u = 12 and le_j = C_N - 1) then
                t_run <= '1'; t_cnt <= (others => '0');
            elsif t_run = '1' then
                v_ja := t_cnt(5 downto 1);
                if t_cnt < 2 * C_N then cs_ra <= v_ja & "10" & t_cnt(0); end if;
                if t_cnt = 2 then
                    t_tk <= (others => '0'); t_tk(0) <= '1'; t_ph <= '0';
                end if;
                t_cnt <= t_cnt + 1;
                if t_cnt = 2 * C_N + 3 then t_run <= '0'; end if;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- The copies.  Stage 1: x^2 (one pixel ahead of the copies, so its two
    -- views are registered) and each copy's linear accumulator.  Stage 2: the
    -- outer and inner tests in units of 8 px^2 and their coverage windows.
    ---------------------------------------------------------------------------
    p_cp : process(clk)
        variable v_la, v_lb : std_logic;
        variable v_lu, v_f, v_g : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            if (g_start or g_avid) = '1' then
                x_sq <= unsigned(signed(x_sq) + resize(x_dx, 22));
                x_dx <= x_dx + signed(resize(c_ddx, 16));
            else
                x_sq <= c_x0sq; x_dx <= c_dx0;
            end if;
            x_s <= x_sq(21 downto 5);
            x_t <= resize(x_sq(21 downto 5), 20) + resize(c_tu, 20);

            for j in 0 to C_N - 1 loop
                v_la := t_tk(j) and not t_ph; v_lb := t_tk(j) and t_ph;
                if (v_la or v_lb) = '1' then a_st(j) <= signed(cs_f(22 downto 0)); end if;
                if v_la = '1' then
                    a_l(j) <= (others => '0'); a_body(j) <= cs_f(31);
                elsif (v_lb or g_avid) = '1' then
                    a_l(j) <= a_l(j) + a_st(j);
                end if;

                v_lu := a_l(j)(22 downto 3);
                v_f := signed(resize(x_s, 20)) - v_lu;
                v_g := signed(x_t) - v_lu;
                q_d(j)(6 downto 0) <= not std_logic_vector(v_f(7 downto 1));
                if v_f(18 downto 8) = "11111111111" then q_d(j)(7) <= '0'; else q_d(j)(7) <= '1'; end if;
                q_d(j)(14 downto 8) <= std_logic_vector(v_g(7 downto 1));
                if v_g(18 downto 8) /= 0 or a_body(j) = '0' then q_d(j)(15) <= '1'; else q_d(j)(15) <= '0'; end if;
                q_d(j)(16) <= v_g(19) and a_body(j);
                q_h(j) <= v_f(19) and not (s_wire and v_g(19) and a_body(j));
            end loop;
        end if;
    end process p_cp;

    ---------------------------------------------------------------------------
    -- [E] Priority (the head on top), coverage, colour.
    --   3: the top hit (one-hot), its windows muxed in groups of 8; the hits below it
    --   4: windows, the top copy's number; the second hit
    --   5: coverages: co = outer, ai = 8 - inner; the second copy's number and whether the pixel is in its fill
    --   6: Wire: line = min(co, ai) of the top copy.  Solid: on the top copy's rim (co < 8) the copy below shows
    --      through, (8 - co) of its fill; inside, the top copy's fill by its inner coverage
    --   7, 8: colour table; 8: source luma * coverage; 9: sum, clamp
    ---------------------------------------------------------------------------
    p_sel : process(clk)
        variable v_h, v_hot, v_rest : unsigned(C_N - 1 downto 0);
        variable v_ib : unsigned(C_N - 1 downto 0);
        variable v_m  : std_logic_vector(16 downto 0);
        variable v_co, v_ai, v_al : unsigned(3 downto 0);
        variable v_ix : unsigned(4 downto 0);
        variable v_v  : unsigned(8 downto 0);
        variable v_p  : unsigned(11 downto 0);
        variable v_y  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 3
            v_h := unsigned(q_h);
            v_hot := v_h and ((not v_h) + 1);
            v_rest := v_h and not v_hot;
            for j in 0 to C_N - 1 loop v_ib(j) := q_d(j)(16); end loop;
            r3_hot <= v_hot; r3_rest <= v_rest; r3_rib <= v_rest and v_ib;
            if v_h /= 0 then r3_any <= '1'; else r3_any <= '0'; end if;
            for g in 0 to C_G - 1 loop
                v_m := (others => '0');
                for j in 8 * g to 8 * g + 7 loop
                    if j < C_N then
                        if v_hot(j) = '1' then v_m := v_m or q_d(j); end if;
                    end if;
                end loop;
                r3_m(g) <= v_m;
            end loop;
            -- 4
            v_m := (others => '0');
            for g in 0 to C_G - 1 loop v_m := v_m or r3_m(g); end loop;
            r4_d <= v_m;
            r4_j <= f_enc(r3_hot);
            r4_any1 <= r3_any;
            v_hot := r3_rest and ((not r3_rest) + 1);
            r4_hot2 <= v_hot; r4_h2ib <= v_hot and r3_rib;
            if r3_rest /= 0 then r4_any2 <= '1'; else r4_any2 <= '0'; end if;
            -- 5
            r5_co <= f_cov(r4_d(7 downto 0), c_shm);
            r5_ai <= f_cov(r4_d(15 downto 8), c_shm);
            r5_ib <= r4_d(16);
            r5_j <= r4_j; r5_any1 <= r4_any1; r5_any2 <= r4_any2;
            r5_k <= f_enc(r4_hot2);
            if r4_h2ib /= 0 then r5_uf <= '1'; else r5_uf <= '0'; end if;
            -- 6
            if r5_ib = '1' then v_ai := (others => '0'); else v_ai := r5_ai; end if;
            v_co := r5_co;
            v_ix := r5_j;
            if r5_any1 = '0' then
                v_al := (others => '0');
            elsif s_wire = '1' then
                if v_co < v_ai then v_al := v_co; else v_al := v_ai; end if;
            elsif v_co(3) = '0' then
                v_ix := r5_k;
                if (r5_any2 and r5_uf) = '1' then v_al := 8 - v_co; else v_al := (others => '0'); end if;
            else
                v_al := 8 - v_ai;
            end if;
            if v_ix = c_nfull and v_al > ('0' & c_frac) then v_al := '0' & c_frac; end if;   -- the newest copy fades in
            tt_ra <= v_ix & v_al;
            r6_al <= v_al; r7_al <= r6_al;
            -- 8
            r8_t <= tt_q(25 downto 0);
            v_v := unsigned(vr_f(8 downto 0));
            if r7_al(3) = '1' then
                r8_vp <= v_v;
            else
                v_p := (f_gu(v_v, r7_al(0), 12) + shift_left(f_gu(v_v, r7_al(1), 12), 1)) + shift_left(f_gu(v_v, r7_al(2), 12), 2);
                r8_vp <= v_p(11 downto 3);
            end if;
            -- 9: the ground colour outside the generated avid
            v_y := resize(unsigned(r8_t(25 downto 16)), 11) + resize(r8_vp, 11);
            if s_gav_sr(6) = '0' then
                q9_y <= c_gy; q9_c(1) <= c_gu; q9_c(2) <= c_gv;
            else
                if v_y > 1023 then q9_y <= (others => '1'); else q9_y <= v_y(9 downto 0); end if;
                q9_c(1) <= unsigned(r8_t(15 downto 8)); q9_c(2) <= unsigned(r8_t(7 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(6);
        end if;
    end process p_sel;

    ---------------------------------------------------------------------------
    -- [F] Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent
    -- 8-sums of the 8-bit chroma, resynced every line: 10 bits out), luma [1 2 1]/4 delayed to the chroma's
    -- group delay, luma 13/16, blanking gate.  The filters add 4 pixels of
    -- delay: the output register is effective stage 17 = C_LAT + 1.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 10: running 8-sums (the sample of 8 clocks before comes back from its delay ring); they snap
            -- back to 8 x ground once 8 blank samples are in, so a glitched sample can't offset the colours for good
            if s_bah = "00000000" then
                s_bu <= c_gu & "000"; s_bv <= c_gv & "000";
            else
                s_bu <= s_bu + q9_c(1) - unsigned(cr_f(15 downto 8));
                s_bv <= s_bv + q9_c(2) - unsigned(cr_f(7 downto 0));
            end if;
            s_bud <= s_bu; s_bvd <= s_bv;
            s_ysr(0) <= q9_y;
            for i in 1 to 5 loop s_ysr(i) <= s_ysr(i - 1); end loop;
            -- 11: filtered
            v_sc := resize(s_bu, 12) + s_bud; s_u31 <= v_sc(11 downto 2);
            v_sc := resize(s_bv, 12) + s_bvd; s_v31 <= v_sc(11 downto 2);
            v_y := resize(s_ysr(3), 12) + resize(s_ysr(4) & '0', 12) + resize(s_ysr(5), 12);
            s_y31 <= v_y(11 downto 2);
            -- 12: luma 13/16 toward black
            if s_y31 > 64 then v_dl := s_y31 - 64; else v_dl := (others => '0'); end if;
            s_y32 <= s_y31 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u32 <= s_u31; s_v32 <= s_v31;
            -- 13: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y33 <= s_y32; s_u33 <= s_u32; s_v33 <= s_v32;
            else
                s_y33 <= to_unsigned(64, 10); s_u33 <= to_unsigned(512, 10); s_v33 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y33);
    data_out.u       <= std_logic_vector(s_u33);
    data_out.v       <= std_logic_vector(s_v33);
    data_out.hsync_n <= sr_f(2);
    data_out.vsync_n <= sr_f(1);
    data_out.field_n <= sr_f(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture slinky;
