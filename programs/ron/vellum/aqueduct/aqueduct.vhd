-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Aqueduct v0.1.0 -- Vellum.  Two arcades at two depths, flat greys sliding past each other with parallax
-- (de Chirico's arcades, the Pont du Gard, painted stage flats).  Luma only (U = V = 512).
--
--   tier k (0 far, 1 near):  scale s = 1 (near), 1/g (far), g = 3.2^(P12/1023);  pitch p_k = s_k P
--   arch centres  x = x0_k + (theta + j) p_k,  x0_k = 960 + Xpan (1 + s_k) / 2      (theta = the drift phase)
--   vertical      horizon yh = y0 - 1.234 P;  top_k = yh + 1.078 p_k;  base_k = top_k + 1.898 p_k
--                 spring_k = top_k + 0.18 p_k + apex      (each tier is solid below its base: stage flats)
--   opening       N = Rc^2 - dd^2 - X^2,  d = N / (2 Rc)    (dd per line from the line unit; X = |dx| + e - swell
--                 per pixel through one ROM squarer per tier; the division is a shift and a 5-bit mantissa);
--                 the gable squares the L1 distance |dx| + |dd| instead
--   wall          min(y - top, -min(d, base - y));  one line per tier: the archivolt (Grey), the opening's
--                 outline (Ink) or the wall's outline (Lattice); coverage over 17 steps
--   camera        luma -> gain table -> 128 / 64-px box -> vertical one-pole: the swell, taken off |dx|, so
--                 bright areas open the arches sideways into rounded rectangles with hairline piers
--
--   K1  X        pan, the near tier by Xpan, the far one by Xpan (1 + s) / 2                (glided)
--   K2  Y        the whole structure up / down                                            (glided)
--   K3  Scale    P = 160 .. 1280 virtual px, exponential                                  (glided)
--   K4  Arch     Round / Pointed / Horseshoe / Stadium / Porthole / Gable (hysteresis)
--   K5  Drift    bipolar speed (Hypnos law), 2^-12 .. 2^-3 cycle of each tier's pitch per field
--   K6  Camera   swell gain, 0 .. 0.22 P at white, at most the pier's half-width           (glided)
--   S7  Glide    Off / On      S8 Build  Solid / Lattice      S9 Tone  Grey / Ink
--   S10 Weight   Bold / Fine   S11 Source  Video / Osc
--   P12 DEPTH    0 = one arcade, 50 % = two tiers (g 1.79), 100 % = g 3.2                (glided)
--
-- Sections (the integer model .vellum_work/rb_aqueduct/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         switches, Arch hysteresis, the drift phase (hold and glide: [C])
--   [C] FIELD ENGINE                     a microcoded accumulator machine (program, constants and register
--                                        file in block RAM, one serial multiplier) runs once per field: knob
--                                        hold and glide, drift speed, the per-tier constants (shifted into
--                                        chains) and the line unit's (stored in the register file);
--                                        LINE UNIT: the per-line head term and edges, from the register file
--   [D] SOURCE FIELD                     Osc lattice, gain table, box, vertical one-pole in a line store
--   [E] TIERS                            per tier: DDA, swell, the squarer, distance normaliser, coverages
--   [F] COMPOSITE                        Solid: two "over" blends; Lattice: additive lines or XOR
--   [G] OUTPUT CONDITIONING              luma [1 2 1]/4, luma 13/16 toward black, blanking gate
--
-- Timing: T = the g_start clock; a register "at stage S" holds DDA pixel i at T + i + S.  The tier DDAs are
-- at stage 20, the coverages at 34, the tone at 40, the output register at 47; output column j is the [1 2 1]
-- filter centred on DDA pixel j + 1, whose virtual x is the centre of screen column j + 4.  One latency (48)
-- in every mode.  The anti-alias width is 1 virtual px at 1080p, 2 at 720p, 1080i and 480p / 576p, 4 at
-- 480i / 576i, so every unit conversion is a shift.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.aqueduct_pkg.all;
use work.all;

architecture aqueduct of program_top is
    constant C_LAT : integer := 48;   -- = processing_delay_clks

    -- (the TOML defaults the knob state starts at are in aqueduct_pkg: C_AQ_RFINIT)

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
    -- mode index for the field engine: 0 1080p, 1 720p, 2 1080i, 3 480p / 576p, 4 480i / 576i
    function f_mi(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8          => return to_unsigned(4, 3);
            when 4 | 12         => return to_unsigned(3, 3);
            when 5 | 6 | 14     => return to_unsigned(1, 3);
            when 1 | 2 | 10     => return to_unsigned(2, 3);
            when others         => return to_unsigned(0, 3);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- saturate a signed value to 13 bits (-4096 .. 4095) by its top bits
    function f_s13(v : signed) return signed is
        variable t : signed(v'length - 1 downto 0);
        variable ok : boolean;
    begin
        t := v;
        ok := true;
        for i in t'high - 1 downto 12 loop
            if t(i) /= t(t'high) then ok := false; end if;
        end loop;
        if ok then return t(12 downto 0);
        elsif t(t'high) = '1' then return to_signed(-4096, 13);
        else return to_signed(4095, 13); end if;
    end function;
    -- saturate an unsigned value to 14 bits
    function f_u14(v : unsigned) return unsigned is
        variable t : unsigned(v'length - 1 downto 0);
    begin
        t := v;
        if t(t'high downto 14) /= 0 then return to_unsigned(16383, 14); else return t(13 downto 0); end if;
    end function;
    -- coverage 0 .. 16 of a distance in sixteenths of the anti-alias width
    function f_cov(d : signed) return unsigned is
        variable v : signed(d'length downto 0);
    begin
        v := resize(d, d'length + 1) + 8;
        if v(v'high) = '1' then return to_unsigned(0, 5);
        elsif v(v'high - 1 downto 5) /= 0 or (v(4) = '1' and v(3 downto 0) /= 0) then return to_unsigned(16, 5);
        else return unsigned(v(4 downto 0)); end if;
    end function;
    function f_min(a, b : signed) return signed is
    begin
        if a < b then return a; else return b; end if;
    end function;
    function f_max5(a, b : unsigned(4 downto 0)) return unsigned is
    begin
        if a > b then return a; else return b; end if;
    end function;
    -- one row of diff * coverage: diff << j when coverage bit j is set
    function f_rows19(d : signed(13 downto 0); a : unsigned(4 downto 0); j : integer) return signed is
    begin
        if a(j) = '1' then return shift_left(resize(d, 19), j); else return to_signed(0, 19); end if;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ysr  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_mir, s_mi : unsigned(2 downto 0) := (others => '0');
    signal s_A    : unsigned(1 downto 0) := (others => '0');
    signal s_idle : unsigned(8 downto 0) := (others => '0');
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
    signal s_st       : std_logic_vector(0 to 19) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] switches, the arch step, the drift phase (the knob hold and glide live in the field engine)
    signal q_go : std_logic := '0';
    signal s_glide : std_logic := '1';
    signal s_lat, s_ink, s_fine, s_src : std_logic := '0';
    signal s_zn  : unsigned(2 downto 0) := (others => '0');            -- arch shape 0 .. 5
    signal z_v6  : unsigned(12 downto 0) := (others => '0');
    signal z_up, z_dn : std_logic := '0';
    signal s_th   : unsigned(17 downto 0) := (others => '0');          -- the drift phase (Q18 cycles)

    -- [C] field engine
    type t_eng is (E_IDLE, E_W1, E_DEC, E_M1, E_M2, E_EX, E_EX2, E_SH, E_MUL, E_NORM);
    signal e_s    : t_eng := E_IDLE;
    signal e_pc   : unsigned(8 downto 0) := (others => '0');
    signal pg_a   : unsigned(8 downto 0) := (others => '0');
    signal pg_q   : unsigned(15 downto 0) := (others => '0');
    signal e_op   : unsigned(4 downto 0) := (others => '0');
    signal e_a    : unsigned(7 downto 0) := (others => '0');
    signal e_acc  : signed(23 downto 0) := (others => '0');
    signal e_cnt  : unsigned(4 downto 0) := (others => '0');
    signal e_tk   : unsigned(2 downto 0) := (others => '0');
    signal e_x    : unsigned(2 downto 0) := (others => '0');
    signal e_f    : std_logic := '0';
    signal ex_x, ex_y : signed(23 downto 0) := (others => '0');
    signal ex_c   : std_logic := '0';
    signal ex_s   : signed(23 downto 0) := (others => '0');
    signal e_sl   : std_logic := '0';                                   -- shift direction: '1' = left
    signal rf_ram : t_aq_rf := C_AQ_RFINIT;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_ra, rf_wa : unsigned(7 downto 0) := (others => '0');
    signal rf_q, rf_d, rf_wd : signed(23 downto 0) := (others => '0');
    signal rf_we : std_logic := '0';
    signal dr_a   : unsigned(8 downto 0) := (others => '0');
    signal dr_q, dr_d : unsigned(15 downto 0) := (others => '0');
    signal m_a    : unsigned(23 downto 0) := (others => '0');
    signal m_p    : unsigned(40 downto 0) := (others => '0');
    signal m_sgn  : std_logic := '0';
    signal e_out  : std_logic_vector(0 to 13) := (others => '0');
    signal rf_rx  : unsigned(7 downto 0);
    signal e_gl   : std_logic := '0';

    -- [C] per-field constants: chains of two (written near, far: c(k) = tier k, 0 far, 1 near)
    type t_u17x2 is array (0 to 1) of unsigned(16 downto 0);
    type t_u16x2 is array (0 to 1) of unsigned(15 downto 0);
    type t_u15x2 is array (0 to 1) of unsigned(14 downto 0);
    type t_u14x2 is array (0 to 1) of unsigned(13 downto 0);
    type t_u13x2 is array (0 to 1) of unsigned(12 downto 0);
    type t_s18x2 is array (0 to 1) of signed(17 downto 0);
    type t_s22x2 is array (0 to 1) of signed(21 downto 0);
    type t_u4x2  is array (0 to 1) of unsigned(3 downto 0);
    type t_u5x2  is array (0 to 1) of unsigned(4 downto 0);
    type t_u2x2  is array (0 to 1) of unsigned(1 downto 0);
    type t_s13x2 is array (0 to 1) of signed(12 downto 0);
    signal c_a0  : t_u17x2 := (others => (others => '0'));
    signal c_stp : t_s18x2 := (others => to_signed(-5000, 18));
    signal c_hp5 : t_u16x2 := (others => to_unsigned(2560, 16));
    signal c_e   : t_u14x2 := (others => (others => '0'));
    signal c_qq  : t_u4x2 := (others => (others => '0'));
    signal c_M   : t_u5x2 := (others => to_unsigned(16, 5));
    signal c_hb, c_cb : t_s13x2 := (others => (others => '0'));
    signal c_ssh : t_u2x2 := (others => (others => '0'));
    signal c_smax : t_u13x2 := (others => (others => '0'));
    signal c_gl  : unsigned(16 downto 0) := (others => '0');
    signal c_spd : unsigned(16 downto 0) := (others => '0');
    signal c_vneg, c_spdz : std_logic := '0';
    -- [C] the line unit's results for the current line, per tier
    signal l_e8  : t_s22x2 := (others => (others => '0'));
    signal l_el  : t_u15x2 := (others => (others => '0'));
    signal l_ndt, l_db, l_dct : t_s13x2 := (others => (others => '0'));
    -- per-field flags and steps (decoded from the latched switches and mode)
    signal c_st  : unsigned(6 downto 0) := to_unsigned(32, 7);
    signal c_sky : unsigned(12 downto 0) := to_unsigned(3584, 13);
    signal c_sflag, c_gable, c_cutinf, c_ltc, c_ink, c_noclip : std_logic := '0';

    -- [C] line values
    signal e_first : std_logic := '1';
    signal e_ly  : unsigned(19 downto 0) := (others => '0');
    signal e_y4  : unsigned(15 downto 0) := (others => '0');
    signal e_y16 : unsigned(15 downto 0) := (others => '0');

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl  : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_v1, s_v2, s_v2d, s_v3, s_v4, s_ld, s_fl, s_seen, s_efl, s_ffl : std_logic := '0';
    type t_gt is array (0 to 255) of unsigned(15 downto 0);
    signal gt_ram : t_gt := (others => (others => '0'));
    attribute no_rw_check of gt_ram : signal is true;
    signal gt_q, s_tv, s_tvd, s_sfirst, gt_wd : unsigned(15 downto 0) := (others => '0');
    signal gt_wa, gt_i : unsigned(7 downto 0) := (others => '0');
    signal gt_we, gt_run : std_logic := '0';
    signal gt_acc : unsigned(23 downto 0) := (others => '0');
    signal s_col, s_c2, s_bcol, s_ecol, s_fcol : unsigned(10 downto 0) := (others => '0');
    signal bx_ram : t_gt := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q, bx_f : unsigned(15 downto 0) := (others => '0');
    signal s_bcnt : unsigned(7 downto 0) := (others => '0');
    signal s_b128 : std_logic := '1';
    signal s_ik   : unsigned(1 downto 0) := (others => '0');           -- one-pole shift: 0 = 5, 1 = 6, 2 = 7
    signal s_df   : signed(16 downto 0) := (others => '0');
    signal s_bsum : unsigned(22 downto 0) := (others => '0');
    signal s_ev, s_fv : std_logic := '0';
    signal s_eb, s_eq, s_fq : unsigned(15 downto 0) := (others => '0');
    signal s_dd : signed(16 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_q2, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal lb_rc : unsigned(10 downto 0) := (others => '0');

    -- [C] line unit
    signal lu_c  : unsigned(4 downto 0) := (others => '1');
    signal lu_ra : unsigned(7 downto 0) := (others => '0');
    signal lu_rd : signed(23 downto 0) := (others => '0');
    signal lu_D, lu_ma, lu_mb, lu_dd : signed(17 downto 0) := (others => '0');
    signal lu_rc2 : signed(21 downto 0) := (others => '0');
    signal lu_y2 : unsigned(13 downto 0) := (others => '0');
    signal lu_sq, lu_sqf : unsigned(15 downto 0) := (others => '0');
    signal lu_p01, lu_p23, lu_p45, lu_cr : signed(15 downto 0) := (others => '0');
    signal lu_s8 : signed(21 downto 0) := (others => '0');
    signal lu_ndt : signed(12 downto 0) := (others => '0');
    signal lu_e4, lu_e5, lu_e6 : signed(17 downto 0) := (others => '0');
    signal lu_dt, lu_dc : signed(12 downto 0) := (others => '0');

    -- [E] shared pixel stages 16 .. 20
    signal x_q0, x_q1 : unsigned(15 downto 0) := (others => '0');
    signal x19_lq : unsigned(15 downto 0) := (others => '0');
    signal x20_l4 : unsigned(13 downto 0) := (others => '0');

    -- [E] tier outputs (stage 34)
    signal t_aw, t_al : t_u5x2 := (others => (others => '0'));

    -- [F] composite
    signal p35_a, p35_b, p38_a, p38_b : signed(18 downto 0) := (others => '0');
    signal p35_t0, p35_t1 : unsigned(12 downto 0) := (others => '0');
    signal p35_x : unsigned(5 downto 0) := (others => '0');
    signal p36_ls, p37_lt, p38_lt, p39_lt : unsigned(12 downto 0) := (others => '0');
    signal p35_aw1, p35_al1, p36_aw1, p37_aw1 : unsigned(4 downto 0) := (others => '0');
    signal p36_t, p37_t, p38_t, p39_t, p40_t : unsigned(12 downto 0) := (others => '0');
    signal p36_c1 : unsigned(12 downto 0) := (others => '0');
    signal p37_d : signed(13 downto 0) := (others => '0');

    -- [G] output conditioning
    signal s_y41, s_y42, s_y43, s_yd1, s_yd2, s_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ya : unsigned(11 downto 0) := (others => '0');
    signal s_y45, s_y46, s_y47 : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    -- field engine: program, data ROM (exponentials, mantissas, constants), register file
    p_pgrom : process(clk)
    begin
        if rising_edge(clk) then pg_q <= C_AQ_PROG(to_integer(pg_a)); end if;
    end process p_pgrom;
    p_drom : process(clk)
    begin
        if rising_edge(clk) then dr_q <= C_AQ_ROM(to_integer(dr_a)); end if;
    end process p_drom;
    p_rf : process(clk)
    begin
        if rising_edge(clk) then
            rf_q <= rf_ram(to_integer(rf_rx));
            if rf_we = '1' then rf_ram(to_integer(rf_wa)) <= rf_wd; end if;
        end if;
    end process p_rf;

    -- gain table: written in the zone, read by the source path
    p_gtram : process(clk)
    begin
        if rising_edge(clk) then
            gt_q <= gt_ram(to_integer(s_sl));
            if gt_we = '1' then gt_ram(to_integer(gt_wa)) <= gt_wd; end if;
        end if;
    end process p_gtram;

    -- box delay: read 126 (or 62) samples behind the write (+ two registers = 128 or 64)
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_v2 = '1' then bx_ram(to_integer(bx_wp)) <= s_tv; end if;
        end if;
    end process p_bxram;

    -- source line store (half horizontal resolution): port 1 = the one-pole's own read (ahead of its
    -- write), port 2 = the pixel path (behind it)
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_c2(10 downto 1)));
            lb_q2 <= lb_ram(to_integer(lb_rc(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    -- the register file's read port: the field engine, or the line unit while the engine is idle
    rf_rx <= lu_ra when e_s = E_IDLE else rf_ra;

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k   : unsigned(9 downto 0);
        variable v_d   : signed(10 downto 0);
        variable v_lo, v_hi : unsigned(13 downto 0);
        variable v_s14 : signed(13 downto 0);
        variable v_th  : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            -- [A] timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ysr <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            s_mir <= f_mi(s_tid);

            -- [A] vblank zone: no avid for 512 clocks (hblank is 138 .. 280)
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 511 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 511 then s_zone <= '1'; else s_zone <= '0'; end if;
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

            -- [A][B] per-field latches; the drift phase advances by last field's speed
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_lat   <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_ys    <= s_ysr;
                s_mi    <= s_mir;
                if s_xsr = 2 then s_b128 <= '1'; else s_b128 <= '0'; end if;
                case to_integer(s_mir) is
                    when 0      => s_ik <= to_unsigned(2, 2); s_A <= to_unsigned(0, 2);   -- 1080p: 1/128 per line
                    when 4      => s_ik <= to_unsigned(0, 2); s_A <= to_unsigned(2, 2);   -- 480i / 576i: 1/32
                    when others => s_ik <= to_unsigned(1, 2); s_A <= to_unsigned(1, 2);   -- 1/64
                end case;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                -- Arch: 6 steps with hysteresis (enter a step only 8 counts past its edge; the two ends are
                -- reached because the band lies on the entry side)
                if z_up = '1' or z_dn = '1' then s_zn <= z_v6(12 downto 10); end if;
                if c_spdz = '0' then                   -- one carry chain: the low column makes the carry-in
                    v_th := (s_th & '1') + ((resize(c_spd, 18) xor (17 downto 0 => c_vneg)) & c_vneg);
                    s_th <= v_th(18 downto 1);
                end if;
            end if;
            -- (the comparison itself runs free, one step per clock, so no knob carry chain sits on the latch)
            v_k := unsigned(registers_in(3));
            z_v6 <= shift_left(resize(v_k, 13), 2) + shift_left(resize(v_k, 13), 1);
            v_lo := resize(s_zn & "0000000000", 14);
            v_hi := v_lo + 1072;
            if resize(z_v6, 14) >= v_hi then z_up <= '1'; else z_up <= '0'; end if;
            if resize(z_v6, 14) + 48 < v_lo then z_dn <= '1'; else z_dn <= '0'; end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);
            -- per-field flags (from the latched switches)
            c_st <= resize(s_xs & "0000", 7);
            if s_ink = '1' then c_sky <= to_unsigned(4096, 13); else c_sky <= to_unsigned(3584, 13); end if;
            c_ltc <= s_lat; c_ink <= s_ink;
            if s_zn = 3 or s_zn = 4 then c_sflag <= '1'; c_cutinf <= '1'; else c_sflag <= '0'; c_cutinf <= '0'; end if;
            if s_zn = 5 then c_gable <= '1'; else c_gable <= '0'; end if;
            c_noclip <= s_lat or s_ink;

            -- [B] the knob hold and glide run in the field engine; it starts one clock after the zone does
            q_go <= s_zgo;

            -- [A] sync delay, generated-avid and start-pulse shift registers (field_n is tied high by the
            -- core on every input, so it only gets a register)
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 18);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [C] FIELD ENGINE.  One instruction = DEC (the word is in pg_q: latch it,
    -- issue the register-file / ROM read, prefetch the next word) [, M1 (block
    -- RAM read), M2 (fabric copy)], EX (one adder; shifts 4 bits a clock,
    -- multiplies 16 steps).  Ops: aqueduct.py.  OUT shifts the accumulator into
    -- that item's chain of three.
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_op  : unsigned(4 downto 0);
        variable v_a   : unsigned(7 downto 0);
        variable v_sel : unsigned(2 downto 0);
        variable v_b   : signed(23 downto 0);
        variable v_x, v_y : signed(23 downto 0);
        variable v_c   : std_logic;
        variable v_sum : signed(23 downto 0);
        variable v_s33 : signed(24 downto 0);
        variable v_ng  : signed(23 downto 0);
        variable v_nr  : unsigned(9 downto 0);
        variable v_mem : boolean;
        variable v_nx  : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            rf_we <= '0';
            e_out <= (others => '0');
            e_gl <= '0';
            -- the shared adder (EX): its operands were registered one clock earlier (ex_x, ex_y, ex_c: in DEC for
            -- the ops without a memory read, in M2 for the others), so EX is only the carry chain and the select
            if e_op = 1 or e_op = 7 or e_op = 17 or e_op = 27 then v_b := resize(signed(dr_d), 24);   -- CT: signed
            elsif e_op = 16 then v_b := signed(resize(dr_d, 24));                         -- EXR: unsigned
            else v_b := rf_d; end if;
            v_s33 := (ex_x & '1') + (ex_y & ex_c);       -- one carry chain; the low column makes the carry-in
            v_sum := v_s33(24 downto 1);

            case e_s is
                when E_IDLE =>
                    if q_go = '1' then
                        e_pc <= (others => '0'); pg_a <= (others => '0'); e_tk <= to_unsigned(C_AQ_TK0, 3); e_s <= E_W1;
                    end if;
                when E_W1 =>
                    e_s <= E_DEC;
                when E_DEC =>
                    v_op := pg_q(15 downto 11); v_a := pg_q(7 downto 0);
                    case to_integer(pg_q(10 downto 8)) is
                        when 1      => v_sel := s_zn;
                        when 2      => v_sel := s_mi;
                        when 3      => v_sel := "00" & s_fine;
                        when 4      => v_sel := '0' & s_lat & s_fine;
                        when 5      => v_sel := "00" & s_src;
                        when 6      => v_sel := "00" & s_glide;
                        when 7      => if s_lat = '1' then v_sel := "010"; else v_sel := "00" & s_ink; end if;
                        when others => v_sel := "000";
                    end case;
                    e_op <= v_op; e_a <= v_a; e_x <= pg_q(10 downto 8);
                    -- operands of the ops that go straight to EX: MRD, MRD8 (+- the product), ABS (- acc)
                    ex_y <= (others => '0');
                    if v_op = 24 then v_x := signed(m_p(31 downto 8)); else v_x := signed(m_p(23 downto 0)); end if;
                    if v_op = 20 then
                        ex_x <= not e_acc; ex_c <= '1';
                    elsif m_sgn = '1' then
                        ex_x <= not v_x; ex_c <= '1';
                    else
                        ex_x <= v_x; ex_c <= '0';
                    end if;
                    if (v_op = 2 or v_op = 3) and pg_q(8) = '1' then     -- LD / ST, x = 1: the tier / channel bank
                        rf_ra <= '1' & e_tk & v_a(3 downto 0);
                    else
                        rf_ra <= v_a;
                    end if;
                    if v_op = 16 then dr_a <= unsigned(e_acc(8 downto 0));
                    else dr_a <= to_unsigned(288, 9) + resize(v_a, 9) + resize(v_sel, 9); end if;
                    -- next word
                    v_nx := e_pc + 1;
                    if v_op = 23 and e_tk /= 0 then v_nx := '0' & v_a; e_tk <= e_tk - 1; end if;
                    if v_op = 28 then e_tk <= v_a(2 downto 0); end if;                       -- SETK
                    e_pc <= v_nx; pg_a <= v_nx;
                    case to_integer(v_op) is
                        when 1 | 2 | 4 | 5 | 6 | 7 | 11 | 12 | 13 | 14 | 16 | 17 | 18 | 19 | 26 | 27 => v_mem := true;
                        when others => v_mem := false;
                    end case;
                    if v_op = 0 then e_s <= E_IDLE;
                    elsif v_op = 23 then e_s <= E_W1;
                    elsif v_mem then e_s <= E_M1;
                    else e_s <= E_EX; end if;
                when E_M1 =>
                    e_s <= E_M2;
                when E_M2 =>
                    rf_d <= rf_q; dr_d <= dr_q;
                    -- operands of the memory ops (the EBR outputs go only through a mux before this register)
                    if e_op = 1 or e_op = 7 or e_op = 17 or e_op = 27 then v_y := resize(signed(dr_q), 24);
                    else v_y := rf_q; end if;
                    case to_integer(e_op) is
                        when 5 | 17 | 18 | 27 => ex_x <= e_acc; ex_y <= not v_y; ex_c <= '1';     -- SUB MAXC CSUB MINC
                        when 6                => ex_x <= not e_acc; ex_y <= v_y; ex_c <= '1';     -- RSUB
                        when 13 | 14          => ex_x <= not e_acc; ex_y <= (others => '0'); ex_c <= '1';   -- |acc|
                        when others           => ex_x <= e_acc; ex_y <= v_y; ex_c <= '0';         -- ADD ADDC ADDN
                    end case;
                    e_s <= E_EX;
                when E_EX =>
                    e_s <= E_DEC;
                    case to_integer(e_op) is
                        when 1 | 2 | 16 => e_acc <= v_b;                                          -- LDC LD EXR
                        when 4 | 5 | 6 | 7 | 15 | 24 => e_acc <= v_sum;                           -- ADD SUB RSUB ADDC MRD MRD8
                        when 17 | 27 | 18 =>                                                       -- MAXC MINC CSUB:
                            ex_s <= v_sum; e_s <= E_EX2;                                           -- select next clock
                        when 26 => if e_f = '1' then e_acc <= v_b; end if;                        -- LDF
                        when 25 =>                                                                 -- TST
                            case to_integer(e_x) is
                                when 0      => e_f <= not e_acc(23);
                                when 1      => e_f <= e_f or e_acc(23);
                                when others => e_f <= e_f and not e_acc(23);
                            end case;
                        when 19 | 20 => if e_acc(23) = '1' then e_acc <= v_sum; end if;           -- ADDN ABS
                        when 3 =>                                                                  -- ST
                            rf_we <= '1'; rf_wd <= e_acc;
                            if e_x(0) = '1' then rf_wa <= '1' & e_tk & e_a(3 downto 0);        -- the bank
                            else rf_wa <= e_a; end if;
                        when 8 =>                                                                  -- KNB
                            if e_a(3) = '0' then e_acc <= signed(resize(s_th, 24));               -- the drift phase
                            else
                                case to_integer(e_tk) is                                           -- loop channel's knob
                                    when 5      => e_acc <= signed(resize(unsigned(registers_in(4)), 24));
                                    when 4      => e_acc <= signed(resize(unsigned(registers_in(0)), 24));
                                    when 3      => e_acc <= signed(resize(unsigned(registers_in(1)), 24));
                                    when 2      => e_acc <= signed(resize(unsigned(registers_in(2)), 24));
                                    when 1      => e_acc <= signed(resize(unsigned(registers_in(5)), 24));
                                    when others => e_acc <= signed(resize(unsigned(registers_in(7)), 24));
                                end case;
                            end if;
                        when 9 | 10 =>                                                             -- SRA SHL
                            e_cnt <= e_a(4 downto 0);
                            if e_op = 10 then e_sl <= '1'; else e_sl <= '0'; end if;
                            if e_a(4 downto 0) /= 0 then e_s <= E_SH; end if;
                        when 11 | 12 =>                                                            -- SRAV SHLV
                            e_cnt <= unsigned(rf_d(4 downto 0));
                            if e_op = 12 then e_sl <= '1'; else e_sl <= '0'; end if;
                            if rf_d(4 downto 0) /= "00000" then e_s <= E_SH; end if;
                        when 13 | 14 =>                                                            -- MULR MULC
                            if e_acc(23) = '1' then m_a <= unsigned(v_sum); else m_a <= unsigned(e_acc); end if;
                            if e_op = 13 then m_p <= resize(unsigned(rf_d(15 downto 0)), 41);
                            else m_p <= resize(dr_d, 41); end if;
                            m_sgn <= e_acc(23);
                            e_cnt <= to_unsigned(16, 5);
                            e_s <= E_MUL;
                        when 21 =>                                                                 -- NORM
                            e_cnt <= to_unsigned(17, 5);
                            e_s <= E_NORM;
                        when 22 =>                                                                 -- OUT
                            if e_a < 14 then e_out(to_integer(e_a(3 downto 0))) <= '1'; end if;
                            if e_a = 10 then e_gl <= '1'; end if;
                        when others => null;
                    end case;
                when E_EX2 =>                                                              -- the conditional select
                    e_s <= E_DEC;
                    case to_integer(e_op) is
                        when 17     => if ex_s(23) = '1' then e_acc <= v_b; end if;               -- MAXC
                        when 27     => if ex_s(23) = '0' then e_acc <= v_b; end if;               -- MINC
                        when others => if ex_s(23) = '0' then e_acc <= ex_s; end if;              -- CSUB
                    end case;
                when E_SH =>
                    if e_cnt >= 4 then
                        if e_sl = '1' then e_acc <= shift_left(e_acc, 4); else e_acc <= shift_right(e_acc, 4); end if;
                        e_cnt <= e_cnt - 4;
                        if e_cnt = 4 then e_s <= E_DEC; end if;
                    else
                        if e_sl = '1' then e_acc <= shift_left(e_acc, 1); else e_acc <= shift_right(e_acc, 1); end if;
                        e_cnt <= e_cnt - 1;
                        if e_cnt = 1 then e_s <= E_DEC; end if;
                    end if;
                when E_MUL =>
                    e_cnt <= e_cnt - 1;
                    if e_cnt = 1 then e_s <= E_DEC; end if;
                when E_NORM =>
                    if e_acc(17) = '0' then
                        e_acc <= shift_left(e_acc, 1); e_cnt <= e_cnt - 1;
                    else
                        v_nr := e_cnt & unsigned(e_acc(16 downto 12));
                        e_acc <= signed(resize(v_nr, 24));
                        e_s <= E_DEC;
                    end if;
            end case;

            -- the serial multiplier: m_p = [partial product | multiplier], add at bit 16, shift right (16 steps)
            if e_s = E_MUL then
                if m_p(0) = '1' then
                    m_p <= '0' & (m_p(40 downto 16) + resize(m_a, 25)) & m_p(15 downto 1);
                else
                    m_p <= '0' & m_p(40 downto 1);
                end if;
            end if;
        end if;
    end process p_eng;

    -- OUT: the per-tier chains (written near, then far) and the global results
    p_outs : process(clk)
    begin
        if rising_edge(clk) then
            if e_out(0) = '1' then c_a0 <= unsigned(e_acc(16 downto 0)) & c_a0(0 to 0); end if;
            if e_out(1) = '1' then c_stp <= e_acc(17 downto 0) & c_stp(0 to 0); end if;
            if e_out(2) = '1' then c_hp5 <= unsigned(e_acc(15 downto 0)) & c_hp5(0 to 0); end if;
            if e_out(3) = '1' then c_e <= unsigned(e_acc(13 downto 0)) & c_e(0 to 0); end if;
            if e_out(4) = '1' then c_qq <= unsigned(e_acc(3 downto 0)) & c_qq(0 to 0); end if;
            if e_out(5) = '1' then c_M <= unsigned(e_acc(4 downto 0)) & c_M(0 to 0); end if;
            if e_out(6) = '1' then c_hb <= e_acc(12 downto 0) & c_hb(0 to 0); end if;
            if e_out(7) = '1' then c_cb <= e_acc(12 downto 0) & c_cb(0 to 0); end if;
            if e_out(8) = '1' then c_ssh <= unsigned(e_acc(1 downto 0)) & c_ssh(0 to 0); end if;
            if e_out(9) = '1' then c_smax <= unsigned(e_acc(12 downto 0)) & c_smax(0 to 0); end if;
            if e_out(10) = '1' then c_gl <= unsigned(e_acc(16 downto 0)); end if;
            if e_out(11) = '1' then c_spd <= unsigned(e_acc(16 downto 0)); end if;
            if e_out(12) = '1' then c_vneg <= e_acc(23); end if;
            if e_out(13) = '1' then c_spdz <= e_acc(23); end if;
        end if;
    end process p_outs;

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
    -- [C] line values: y (Q4 virtual lines) and y16 = y >> AA of the current line
    ---------------------------------------------------------------------------
    p_line : process(clk)
    begin
        if rising_edge(clk) then
            if s_zone = '1' then
                e_first <= '1';
            elsif g_start = '1' then
                e_first <= '0';
                if e_first = '1' then e_ly <= resize(s_ys(11 downto 1), 20);
                else e_ly <= e_ly + resize(s_ys, 20); end if;
            end if;
            e_y4 <= e_ly(19 downto 4);
            e_y16 <= shift_right(e_y4, to_integer(s_A));
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [C] LINE UNIT: in the first 21 clocks of every line (the field engine is
    -- idle), for each tier, from that tier's bank of the register file (spring,
    -- K, Ls, (Rc^2 - 2730) >> 8, cornice, base, archivolt cut): |dd|, the head's
    -- vertical term E8 = Rc^2 - dd^2 (the gable keeps Rc^2 and takes |dd| into
    -- the pixel squarer through e), and the edges in sixteenths of the aa width.
    -- Tier k reads its seven words at clocks 1 + 7k .. 7 + 7k.
    ---------------------------------------------------------------------------
    p_sqlu : process(clk)
    begin
        if rising_edge(clk) then lu_sq <= C_AQ_SQ(to_integer(lu_y2(13 downto 6))); end if;
    end process p_sqlu;

    p_lu : process(clk)
        variable v_j  : integer range 0 to 31;
        variable v_k  : integer range 0 to 1;
        variable v_i  : integer range 0 to 15;
        variable v_dl : signed(17 downto 0);
        variable v_a  : unsigned(17 downto 0);
        variable v_h  : unsigned(8 downto 0);
        variable v_l  : std_logic_vector(5 downto 0);
        variable v_dc, v_dt : signed(12 downto 0);
        function f_xr(h : unsigned(8 downto 0); l : std_logic; j : integer) return signed is
        begin
            if l = '1' then return shift_left(signed(resize(h, 16)), j); else return to_signed(0, 16); end if;
        end function;
    begin
        if rising_edge(clk) then
            if g_start = '1' then lu_c <= (others => '0');
            elsif lu_c /= 31 then lu_c <= lu_c + 1; end if;
            v_j := to_integer(lu_c);
            -- reads: clock 1 + 7k + i -> word i of tier k; the fabric copy arrives 3 clocks later
            if v_j >= 1 and v_j <= 14 then
                if v_j <= 7 then v_k := 0; v_i := v_j - 1; else v_k := 1; v_i := v_j - 8; end if;
                lu_ra <= "100" & to_unsigned(v_k, 1) & to_unsigned(v_i, 4);
            end if;
            lu_rd <= rf_q;
            -- processing: tier k's word i is in lu_rd at clock 4 + 7k + i (each step below is keyed on its own clock,
            -- so tier 0's late steps overlap tier 1's early ones)
            for k in 0 to 1 loop
                if v_j = 4 + 7 * k then                                -- spring: D = y - spring
                    lu_D <= signed(resize(e_y4, 18)) - lu_rd(17 downto 0);
                end if;
                if v_j = 5 + 7 * k then                                -- K: min(D, K)
                    if lu_D < lu_rd(17 downto 0) then lu_ma <= lu_D; else lu_ma <= lu_rd(17 downto 0); end if;
                end if;
                if v_j = 6 + 7 * k then                                -- Ls: max(D - Ls, 0) for the stadium / porthole
                    v_dl := lu_D - lu_rd(17 downto 0);
                    if c_sflag = '1' and v_dl(17) = '0' then lu_mb <= v_dl; else lu_mb <= (others => '0'); end if;
                end if;
                if v_j = 7 + 7 * k then                                -- Rc^2 term; dd
                    lu_rc2 <= lu_rd(21 downto 0);
                    lu_dd <= lu_ma + lu_mb;
                end if;
                if v_j = 8 + 7 * k then                                -- cornice: top - y16 (raw); |dd|
                    lu_e4 <= lu_rd(17 downto 0) - signed(resize(e_y16, 18));
                    if lu_dd(17) = '1' then v_a := unsigned(-lu_dd); else v_a := unsigned(lu_dd); end if;
                    lu_y2 <= f_u14(v_a);
                end if;
                if v_j = 9 + 7 * k then                                -- base (raw); -dt; e (+ |dd| for the gable)
                    lu_e5 <= lu_rd(17 downto 0) - signed(resize(e_y16, 18));
                    lu_ndt <= f_s13(lu_e4);
                    if c_gable = '1' then l_el(k) <= resize(c_e(k), 15) + resize(lu_y2, 15);
                    else l_el(k) <= resize(c_e(k), 15); end if;
                end if;
                if v_j = 10 + 7 * k then                               -- cut (raw); write -dt and the base
                    lu_e6 <= lu_rd(17 downto 0) - signed(resize(e_y16, 18));
                    l_ndt(k) <= lu_ndt;
                    l_db(k) <= f_s13(lu_e5);
                    if lu_ndt = -4096 then lu_dt <= to_signed(4095, 13); else lu_dt <= -lu_ndt; end if;
                end if;
                if v_j = 11 + 7 * k then
                    lu_dc <= f_s13(lu_e6);
                end if;
                if v_j = 12 + 7 * k then                               -- the archivolt clip: min(cut - y16, dt); E8
                    if c_noclip = '1' then l_dct(k) <= to_signed(4095, 13);
                    elsif c_cutinf = '1' or lu_dt < lu_dc then l_dct(k) <= lu_dt;
                    else l_dct(k) <= lu_dc; end if;
                    if c_gable = '1' then l_e8(k) <= lu_rc2;
                    else l_e8(k) <= lu_rc2 - lu_s8; end if;
                end if;
            end loop;
            -- the squarer on |dd| (ROM at clock i = 5, rows; 6: fabric copy, cross term; 7: >> 8)
            v_h := lu_y2(13 downto 6) & '1'; v_l := std_logic_vector(lu_y2(5 downto 0));
            lu_p01 <= f_xr(v_h, v_l(0), 0) + f_xr(v_h, v_l(1), 1);
            lu_p23 <= f_xr(v_h, v_l(2), 2) + f_xr(v_h, v_l(3), 3);
            lu_p45 <= f_xr(v_h, v_l(4), 4) - f_xr(v_h, not v_l(5), 5);
            lu_sqf <= lu_sq;
            lu_cr <= lu_p01 + lu_p23 + lu_p45;
            lu_s8 <= signed(shift_left(resize(lu_sqf, 22), 4)) + resize(shift_right(lu_cr, 2), 22);
        end if;
    end process p_lu;

    ---------------------------------------------------------------------------
    -- [E] shared pixel stages: the store read (stage 16), its interpolation, the
    -- swell L4 (stage 20)
    ---------------------------------------------------------------------------
    p_pre : process(clk)
        variable v_s : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            if s_st(13) = '1' or s_st(14) = '1' then lb_rc <= (others => '0');
            elsif lb_rc /= g_wm1(10 downto 0) then lb_rc <= lb_rc + 1; end if;
            -- 17: lb_q2;  18: this and the last read;  19: their mean (Q2);  20: L4 (Q4 virtual px)
            x_q0 <= lb_q2; x_q1 <= x_q0;
            v_s := resize(x_q0, 17) + resize(x_q1, 17) + 1;
            x19_lq <= v_s(16 downto 1);
            x20_l4 <= x19_lq(15 downto 2);
        end if;
    end process p_pre;

    ---------------------------------------------------------------------------
    -- [E] TIERS (stages 20 .. 34), one copy per tier
    ---------------------------------------------------------------------------
    g_tier : for k in 0 to 1 generate
        signal a20  : unsigned(16 downto 0) := (others => '0');
        signal ax21 : unsigned(16 downto 0) := (others => '0');
        signal sw21 : unsigned(12 downto 0) := (others => '0');
        signal x22  : unsigned(13 downto 0) := (others => '0');
        signal sqx_q, sqx_f : unsigned(15 downto 0) := (others => '0');
        signal px01, px23, px45, crx : signed(15 downto 0) := (others => '0');
        signal n25  : signed(21 downto 0) := (others => '0');
        signal ns26 : signed(12 downto 0) := (others => '0');
        signal pa27, pb27 : signed(17 downto 0) := (others => '0');
        signal dh28, dh29, do29, mw30, xg30 : signed(12 downto 0) := (others => '0');
        signal bl31, dl32, dl33 : signed(13 downto 0) := (others => '0');
        signal aw31, aw32, aw33 : unsigned(4 downto 0) := (others => '0');
        function f_xrow(h : unsigned(8 downto 0); l : std_logic; j : integer) return signed is
        begin
            if l = '1' then return shift_left(signed(resize(h, 16)), j); else return to_signed(0, 16); end if;
        end function;
        function f_mrow(v : signed(12 downto 0); m : unsigned(4 downto 0); j : integer) return signed is
        begin
            if m(j) = '1' then return shift_left(resize(v, 18), j); else return to_signed(0, 18); end if;
        end function;
    begin
        -- the squarer's high part (one ROM per tier)
        p_sqx : process(clk)
        begin
            if rising_edge(clk) then sqx_q <= C_AQ_SQ(to_integer(x22(13 downto 6))); end if;
        end process p_sqx;

        p_tier : process(clk)
            variable v_s1 : unsigned(16 downto 0);
            variable v_s2 : signed(17 downto 0);
            variable v_v  : signed(17 downto 0);
            variable v_ab, v_ac : unsigned(18 downto 0);
            variable v_n4 : signed(17 downto 0);
            variable v_w  : unsigned(13 downto 0);
            variable v_x  : signed(18 downto 0);
            variable v_h  : unsigned(8 downto 0);
            variable v_l  : std_logic_vector(5 downto 0);
            variable v_n  : signed(21 downto 0);
            variable v_eq : std_logic_vector(12 to 21);
            variable v_ov : std_logic;
            variable v_win : signed(12 downto 0);
            variable v_t  : signed(17 downto 0);
            variable v_b  : signed(13 downto 0);
            variable v_s  : std_logic;
            variable v_y  : signed(14 downto 0);
        begin
            if rising_edge(clk) then
                -- 20: the DDA, a = (a0 + i st) mod p (Q5), compare-subtract wrap
                if s_st(18) = '1' then
                    a20 <= c_a0(k);
                else
                    v_s1 := a20 + resize(c_st, 17);
                    v_s2 := signed(resize(a20, 18)) + c_stp(k);
                    if v_s2(17) = '0' then a20 <= unsigned(v_s2(16 downto 0)); else a20 <= v_s1; end if;
                end if;
                -- 21: |a - p/2| (Q5); the camera's swell for this tier, min(L4 >> SSH, SMAX) (Q4)
                v_v := signed(resize(a20, 18)) - signed(resize(c_hp5(k), 18));
                v_ab := unsigned(v_v xor (17 downto 0 => v_v(17))) & '1';     -- |v| = (v xor s) + s: the low
                v_ac := (others => '0'); v_ac(0) := v_v(17);                     -- column makes the carry-in
                v_ab := v_ab + v_ac;
                ax21 <= v_ab(17 downto 1);
                v_w := shift_right(x20_l4, to_integer(c_ssh(k)));
                if v_w > resize(c_smax(k), 14) then sw21 <= c_smax(k); else sw21 <= v_w(12 downto 0); end if;
                -- 22: X = |dx| + e - swell (Q4; the gable adds |dd|), clamped to 0 .. 1024 virtual px
                v_x := signed(resize(ax21(16 downto 1), 19)) + signed(resize(l_el(k), 19)) - signed(resize(sw21, 19));
                if v_x(18) = '1' then x22 <= (others => '0');
                elsif v_x(17 downto 14) /= "0000" then x22 <= (others => '1');
                else x22 <= unsigned(v_x(13 downto 0)); end if;
                -- 23: ROM read (p_sqx); the cross term (2h + 1)(l - 32) as gated rows
                v_h := x22(13 downto 6) & '1'; v_l := std_logic_vector(x22(5 downto 0));
                px01 <= f_xrow(v_h, v_l(0), 0) + f_xrow(v_h, v_l(1), 1);
                px23 <= f_xrow(v_h, v_l(2), 2) + f_xrow(v_h, v_l(3), 3);
                px45 <= f_xrow(v_h, v_l(4), 4) - f_xrow(v_h, not v_l(5), 5);
                -- 24: fabric copy of the ROM; the cross term
                sqx_f <= sqx_q;
                crx <= px01 + px23 + px45;
                -- 25: N8 = E8 - (X^2 - 1365) >> 8 = E8 - 16 (h^2 + h) - ((2h + 1)(l - 32)) >> 2
                n25 <= l_e8(k) - signed(shift_left(resize(sqx_f, 22), 4)) - resize(shift_right(crx, 2), 22);
                -- 26: N8 >> qq, saturated to 13 bits (a window and an overflow test)
                v_n := n25;
                for j in 12 to 21 loop                       -- bits j .. 21 all equal to the sign (parallel tests)
                    v_eq(j) := '1';
                    for i in j to 20 loop
                        if v_n(i) /= v_n(21) then v_eq(j) := '0'; end if;
                    end loop;
                end loop;
                case to_integer(c_qq(k)(3 downto 2)) is
                    when 0      => v_n4 := v_n(17 downto 0);
                    when 1      => v_n4 := v_n(21 downto 4);
                    when others => v_n4 := resize(v_n(21 downto 8), 18);
                end case;
                case to_integer(c_qq(k)(1 downto 0)) is
                    when 0      => v_win := v_n4(12 downto 0);
                    when 1      => v_win := v_n4(13 downto 1);
                    when 2      => v_win := v_n4(14 downto 2);
                    when others => v_win := v_n4(15 downto 3);
                end case;
                v_ov := not v_eq(12 + to_integer(c_qq(k)));
                if v_ov = '0' then ns26 <= v_win;
                elsif v_n(21) = '1' then ns26 <= to_signed(-4096, 13);
                else ns26 <= to_signed(4095, 13); end if;
                -- 27, 28: d = n M / 32 (M = 16 .. 31: the top row is always there)
                pa27 <= shift_left(resize(ns26, 18), 4) + f_mrow(ns26, c_M(k), 3) + f_mrow(ns26, c_M(k), 0);
                pb27 <= f_mrow(ns26, c_M(k), 2) + f_mrow(ns26, c_M(k), 1);
                v_t := pa27 + pb27;
                dh28 <= v_t(17 downto 5);
                -- 29: the opening = the head and above the base (this line's edges from the line unit)
                do29 <= f_min(dh28, l_db(k));
                dh29 <= dh28;
                -- 30: minus the wall distance = max(-dt, opening); the line's input: Grey the head, Ink the opening
                if l_ndt(k) > do29 then mw30 <= l_ndt(k); else mw30 <= do29; end if;
                if c_ink = '1' then xg30 <= do29; else xg30 <= dh29; end if;
                -- 31: wall coverage; the line's offset (Lattice: the wall edge)
                aw31 <= to_unsigned(16, 5) - f_cov(mw30);
                if c_ltc = '1' then bl31 <= resize(mw30, 14) - resize(c_cb(k), 14);
                else bl31 <= resize(xg30, 14) - resize(c_cb(k), 14); end if;
                -- 32: hb - |offset| on one carry chain
                v_s := bl31(13);
                v_b := bl31;
                if v_s = '0' then v_b := not bl31; end if;
                v_y := (resize(c_hb(k), 14) & '1') + (v_b & (not v_s));
                dl32 <= v_y(14 downto 1);
                aw32 <= aw31;
                -- 33: clipped at the spring and the cornice (Grey)
                dl33 <= f_min(dl32, resize(l_dct(k), 14));
                aw33 <= aw32;
                -- 34: the tier's two coverages
                t_al(k) <= f_cov(dl33);
                t_aw(k) <= aw33;
            end if;
        end process p_tier;
    end generate g_tier;

    ---------------------------------------------------------------------------
    -- [F] COMPOSITE (stages 35 .. 40).  Solid: tone = sky, the far tier "over" it
    -- by its wall coverage (a flat grey, or black in Ink), then the near tier
    -- (black with a white archivolt; Ink: white with a black outline).  Lattice:
    -- the two tiers' lines, additive (Grey) or XOR (Ink), chosen at the end.
    ---------------------------------------------------------------------------
    p_comp : process(clk)
        variable v_s  : signed(19 downto 0);
        variable v_y  : unsigned(12 downto 0);
        variable v_d0 : signed(13 downto 0);
        variable v_l  : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            -- 35: far rows (sky -> far grey by aw0); lattice terms (128 and 224 per line), the XOR count
            if c_ink = '1' then v_d0 := to_signed(-4096, 14); else v_d0 := to_signed(-1024, 14); end if;
            if t_aw(0)(4) = '1' then
                p35_a <= shift_left(resize(v_d0, 19), 4); p35_b <= (others => '0');
            else
                p35_a <= f_rows19(v_d0, t_aw(0), 0) + f_rows19(v_d0, t_aw(0), 1);
                p35_b <= f_rows19(v_d0, t_aw(0), 2) + f_rows19(v_d0, t_aw(0), 3);
            end if;
            p35_t0 <= shift_left(resize(t_al(0), 13), 7);
            p35_t1 <= shift_left(resize(t_al(1), 13), 8) - shift_left(resize(t_al(1), 13), 5);
            p35_x <= resize(t_al(0), 6) + resize(t_al(1), 6);
            p35_aw1 <= t_aw(1); p35_al1 <= t_al(1);
            -- 36: far tone; the near paint; the lattice tone
            v_s := shift_left(signed(resize(c_sky, 20)), 4) + resize(p35_a, 20) + resize(p35_b, 20);
            p36_t <= unsigned(v_s(16 downto 4));
            if c_ink = '1' then p36_c1 <= to_unsigned(4096, 13) - shift_left(resize(p35_al1, 13), 8);
            else p36_c1 <= shift_left(resize(p35_al1, 13), 8); end if;
            p36_aw1 <= p35_aw1;
            if c_ink = '1' then
                if p35_x(4 downto 0) <= 16 then p36_ls <= shift_left(resize(p35_x(4 downto 0), 13), 8);
                else p36_ls <= shift_left(resize(32 - p35_x(4 downto 0), 13), 8); end if;
            else
                v_l := resize(p35_t0, 14) + resize(p35_t1, 14);
                if v_l > 4096 then p36_ls <= to_unsigned(4096, 13); else p36_ls <= v_l(12 downto 0); end if;
            end if;
            -- 37 .. 39: the near blend (forced through in Lattice)
            p37_d <= signed(resize(p36_c1, 14)) - signed(resize(p36_t, 14));
            p37_t <= p36_t; p37_aw1 <= p36_aw1; p37_lt <= p36_ls;
            if p37_aw1(4) = '1' then
                p38_a <= shift_left(resize(p37_d, 19), 4); p38_b <= (others => '0');
            else
                p38_a <= f_rows19(p37_d, p37_aw1, 0) + f_rows19(p37_d, p37_aw1, 1);
                p38_b <= f_rows19(p37_d, p37_aw1, 2) + f_rows19(p37_d, p37_aw1, 3);
            end if;
            p38_t <= p37_t; p38_lt <= p37_lt;
            v_s := shift_left(signed(resize(p38_t, 20)), 4) + resize(p38_a, 20) + resize(p38_b, 20);
            p39_t <= unsigned(v_s(16 downto 4)); p39_lt <= p38_lt;
            -- 40: Solid or Lattice
            if c_ltc = '1' then p40_t <= p39_lt; else p40_t <= p39_t; end if;
            -- 41: tone -> luma (white 944, 779 after the output scaling)
            v_y := p40_t;
            s_y41 <= resize(to_unsigned(64, 13) + shift_right(v_y, 2) - shift_right(v_y, 5) - shift_right(v_y, 8), 10);
        end if;
    end process p_comp;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> gain table (K6: the lift in
    -- Q2 sixteenths of the anti-alias width) -> box of 128 px (64 px at 720p and
    -- SD), edge-replicated and reloaded at every line start -> vertical one-pole
    -- (1/128 per line at 1080p, 1/32 at 480i / 576i, 1/64 otherwise) in the line
    -- store at half horizontal resolution.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(15 downto 0);
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
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);   -- ot * 96
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
            -- gain table (p_gtram), then its fabric copy: s_tv is the sample of column s_col
            s_v1 <= s_sv; s_v2 <= s_v1; s_v2d <= s_v2;
            s_tv <= gt_q;
            if s_v2 = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 128 (64) samples; the samples before the line start count as the first one
            if s_v2 = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            elsif s_b128 = '1' then bx_wp <= (others => '0'); bx_rp <= to_unsigned(130, 8);
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(194, 8); end if;
            bx_f <= bx_q;
            if s_bcnt(7) = '0' and (s_b128 = '1' or s_bcnt(6) = '0') then v_tap := s_sfirst; else v_tap := bx_f; end if;
            s_df <= signed(resize(s_tv, 17)) - signed(resize(v_tap, 17));
            s_tvd <= s_tv; s_v3 <= s_v2; s_c2 <= s_col;
            s_ld <= s_v2 and not s_v2d;
            if s_v2 = '1' and s_v2d = '0' then
                s_sfirst <= s_tv;
                s_bcnt <= to_unsigned(1, 8);
            elsif s_v2 = '1' then
                if s_bcnt(7) = '0' then s_bcnt <= s_bcnt + 1; end if;
            end if;
            if s_ld = '1' then
                if s_b128 = '1' then s_bsum <= shift_left(resize(s_tvd, 23), 7);
                else s_bsum <= shift_left(resize(s_tvd, 23), 6); end if;
            elsif s_v3 = '1' then
                s_bsum <= unsigned(signed(s_bsum) + resize(s_df, 23));
            end if;
            s_v4 <= s_v3; s_bcol <= s_c2;

            -- the first line of a field loads the one-pole instead of stepping it
            if s_zone = '1' then
                s_fl <= '1'; s_seen <= '0';
            elsif s_ld = '1' then
                if s_seen = '1' then s_fl <= '0'; end if;
                s_seen <= '1';
            end if;

            -- vertical one-pole on the odd columns: q += (box - q) >> k
            if s_b128 = '1' then s_eb <= s_bsum(22 downto 7); else s_eb <= s_bsum(21 downto 6); end if;
            s_eq <= lb_q1;
            s_ev <= s_v4 and s_bcol(0); s_ecol <= s_bcol; s_efl <= s_fl;
            s_dd <= signed(resize(s_eb, 17)) - signed(resize(s_eq, 17));
            if s_efl = '1' then s_fq <= s_eb; else s_fq <= s_eq; end if;
            s_fv <= s_ev; s_fcol <= s_ecol; s_ffl <= s_efl;
            if s_ffl = '1' then v_dl := (others => '0');
            elsif s_ik = 2 then v_dl := shift_right(s_dd, 7);
            elsif s_ik = 1 then v_dl := shift_right(s_dd, 6);
            else v_dl := shift_right(s_dd, 5); end if;
            v_n := signed(resize(s_fq, 18)) + resize(v_dl, 18);
            lb_wd <= unsigned(v_n(15 downto 0));
            lb_we <= s_fv; lb_wa <= s_fcol(10 downto 1);

            -- gain table fill (zone, after slot 6): T(i) = (i * Gl) >> 8 by accumulation
            gt_we <= '0';
            if e_gl = '1' then
                gt_run <= '1'; gt_i <= (others => '0'); gt_acc <= (others => '0');
            elsif gt_run = '1' then
                gt_we <= '1'; gt_wa <= gt_i;
                gt_wd <= gt_acc(23 downto 8);
                gt_acc <= gt_acc + resize(c_gl, 24); gt_i <= gt_i + 1;
                if gt_i = 255 then gt_run <= '0'; end if;
            end if;
        end if;
    end process p_src;
    ---------------------------------------------------------------------------
    -- [G] OUTPUT CONDITIONING: two delay registers (latency 48, a multiple of
    -- 4), luma [1 2 1]/4, luma 13/16 toward black, blanking gate.  Chroma is 512
    -- on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_y42 <= s_y41; s_y43 <= s_y42;
            s_yd1 <= s_y43; s_yd2 <= s_yd1;
            s_ya <= resize(s_y43, 12) + resize(s_yd2, 12) + 2;
            s_yb <= s_yd1;
            v_y := s_ya + shift_left(resize(s_yb, 12), 1);
            s_y45 <= v_y(11 downto 2);
            if s_y45 > 64 then v_dl := s_y45 - 64; else v_dl := (others => '0'); end if;
            s_y46 <= s_y45 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y47 <= s_y46;
            else
                s_y47 <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y47);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture aqueduct;
