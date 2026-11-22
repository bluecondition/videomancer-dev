-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Miura v0.1.0 -- Kinetic wave 2.
--
-- A Miura-ori sheet (Koryo Miura's map fold; the origami tessellations of Ron Resch and Tomohiro Tachi):
-- horizontal creases cut the screen into ROWS, zigzag creases cut every row into parallelogram facets leaning
-- right in one row and left in the next, lit from the side so the facets take four tones.  A wave of folding
-- runs down the sheet: a folding row closes (its height H shrinks), its creases lean harder (t = A / H, so they
-- still meet on the shared crease) and its facets split into a bright and a dark tone.
--
--   element  a row j (row 0 centred on the screen centre; the rows above are stacked once per field)
--   state    fold z_j = F + Depth * wave(phase - Spread * index_j) + camera row mean   (logit, Q6)
--            phi_j = 1/8 turn * (1 + sin(pi/2 * clamp(z / 4)))            (an S-curve: 0 = flat, 1/4 = shut)
--   geometry H_j = max(12, H0 cos phi_j) virtual lines; crease x offset = A v / H (even rows), A - A v / H (odd)
--   tones    T(row, column parity) = sin phi_j * (+-cx -+ cy)  (Light Side / Top), -32 .. 31 table steps
--   colour   paper A / B on alternate rows through a per-field table (toward black below 0, toward white above),
--            AA across the facet edge in tone space; creases on top
--
--   K1  Scale    facet pitch 56 .. 317 virtual px (exponential, glided); flat row height = the pitch
--   K2  Shape    zigzag amplitude A = 0 .. 0.6 pitch (straight pleats .. sharp herringbone)
--   K3  Depth    fold swing 0 .. 3 logit (square law)
--   K4  Rate     bipolar, centre = still; 60 s per cycle .. 2 cycles per second
--   K5  Colour   the 16 shared palettes (A, B = the two papers, accent = mountain creases); Spread frozen 0.085
--   K6  Camera   camera (and Osc) strength: per-facet luma steps the facet's tone, the row mean adds to the fold
--   S7  Wave     Smooth / Snap
--   S8  Field    Linear (the wave runs down the sheet) / Radial (it folds from the middle out)
--   S9  Creases  Off / On
--   S10 Light    Side (zigzag columns) / Top (mountain / valley rows)
--   S11 Source   Video / Osc
--   P12 FOLD     the mean fold: almost flat .. nearly shut (glided)
--
-- Chassis sections (from louver / riley): [A] timing ID -> virtual-1080 steps, vblank zone, width, glides, palette;
-- [B] generated avid; [C] source oscillator and luma tap; [F] output conditioning.  New: [E] the engine (per-field
-- constants, the row walk, the row unit, one serial multiplier + one serial divider), [L] the line engine (its own
-- multiplier), [K] the camera writer (its own multiplier), [P] the pixel path (18 stages).
-- One latency (C_LAT = 25) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.miura_pkg.all;
use work.all;

architecture miura of program_top is
    constant C_LAT      : integer := 25;      -- = processing_delay_clks
    constant C_SIM_SNAP : boolean := false;   -- sim-only copies set this: knob glides jump straight to the knob
    constant C_SRC_D    : integer := 4;       -- luma tap delay to the pixel path's stage 2
    constant C_S14      : unsigned(13 downto 0) := to_unsigned(1393, 14);    -- frozen Spread: 0.085 cycle per row

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
    -- 8 / ys (Q16): the horizontal crease's ramp of two field lines (three on interlaced SD)
    function f_irv(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(307, 12);
            when 4          => return to_unsigned(910, 12);
            when 8          => return to_unsigned(364, 12);
            when 12         => return to_unsigned(1092, 12);
            when 5 | 6 | 14 => return to_unsigned(1365, 12);
            when 1 | 2 | 10 => return to_unsigned(1024, 12);
            when others     => return to_unsigned(2048, 12);
        end case;
    end function;
    -- per x step: 32 / xs (Q12), 1 / xs (Q12), half the real width in px, the camera row gains per unit K6
    function f_rx(x : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(x) is
            when 4 => return to_unsigned(32768, 22);
            when 3 => return to_unsigned(43691, 22);
            when others => return to_unsigned(65536, 22);
        end case;
    end function;
    function f_rxh(x : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(x) is
            when 4 => return to_unsigned(1024, 16);
            when 3 => return to_unsigned(1365, 16);
            when others => return to_unsigned(2048, 16);
        end case;
    end function;
    function f_wrh(x : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(x) is
            when 4 => return to_unsigned(360, 10);
            when 3 => return to_unsigned(640, 10);
            when others => return to_unsigned(960, 10);
        end case;
    end function;
    function f_kgr(x : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(x) is
            when 4 => return to_unsigned(6817, 22);
            when 3 => return to_unsigned(3834, 22);
            when others => return to_unsigned(2556, 22);
        end case;
    end function;
    function f_krr(x : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(x) is
            when 4 => return to_unsigned(2727, 22);
            when 3 => return to_unsigned(1534, 22);
            when others => return to_unsigned(1023, 22);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- gated operands: every per-pixel multiply in this file is a sum of these
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

    attribute no_rw_check : boolean;
    attribute fsm_encoding : string;

    -- ROMs (one reader each: the engine)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_sn_a : unsigned(9 downto 0) := (others => '0');
    signal r_sn_q : unsigned(15 downto 0) := (others => '0');
    signal r_pl_a : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    -- facet table, refilled every field: address = paper & tone (6 bits, two's complement); word = {Y 10, U 8, V 8}
    -- (three RAMs, one per channel: the fill writes one channel at a time)
    type t_fy is array (0 to 127) of std_logic_vector(15 downto 0);
    type t_fc is array (0 to 127) of std_logic_vector(7 downto 0);
    signal fty : t_fy := (others => (others => '0'));
    signal ftu, ftv : t_fc := (others => (others => '0'));
    attribute no_rw_check of fty : signal is true;
    attribute no_rw_check of ftu : signal is true;
    attribute no_rw_check of ftv : signal is true;
    signal ft_we : std_logic_vector(2 downto 0) := "000";
    signal ft_wa : unsigned(6 downto 0) := (others => '0');
    signal ft_wd : std_logic_vector(9 downto 0) := (others => '0');
    signal ft1_a : unsigned(6 downto 0) := (others => '0');
    signal fty_q : std_logic_vector(15 downto 0) := (others => '0');
    signal ftu_q, ftv_q : std_logic_vector(7 downto 0) := (others => '0');
    signal ft1_q : std_logic_vector(25 downto 0);
    -- camera values per facet column, two banks by row parity
    type t_cv is array (0 to 511) of std_logic_vector(7 downto 0);
    signal cvm : t_cv := (others => x"00");
    attribute no_rw_check of cvm : signal is true;
    signal cv_ra, cv_wa, cv_pa : unsigned(8 downto 0) := (others => '0');
    signal cv_q, cv_qf, cv_wd : std_logic_vector(7 downto 0) := x"00";
    signal cv_we : std_logic := '0';
    -- camera fold term per row (row index mod 256)
    type t_rc is array (0 to 255) of std_logic_vector(7 downto 0);
    signal rcm : t_rc := (others => x"00");
    attribute no_rw_check of rcm : signal is true;
    signal rc_ra, rc_wa : unsigned(7 downto 0) := (others => '0');
    signal rc_q, rc_wd : std_logic_vector(7 downto 0) := x"00";
    signal rc_we : std_logic := '0';
    -- delay rings
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
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
    signal c_irv  : unsigned(11 downto 0) := to_unsigned(2048, 12);
    signal c_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo, s_z24, s_z40 : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw rate, switches, palette
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(560 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(460 * 8, 13);
    signal s_gk : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_cre, s_top, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(10, 4);
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- palette: papers A, B and the accent (luma 10 bits, chroma 8 bits)
    signal c_pay, c_acy : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal c_pau, c_pav, c_acu, c_acv : unsigned(7 downto 0) := to_unsigned(128, 8);

    -- [E] engine: per-field constants
    signal c_wc4   : unsigned(12 downto 0) := to_unsigned(2311, 13);      -- facet pitch, Q4 virtual px (= H0)
    signal c_wpx8  : unsigned(16 downto 0) := to_unsigned(24650, 17);     -- facet pitch, Q8 pixels
    signal c_apx8  : unsigned(15 downto 0) := (others => '0');            -- zigzag amplitude, Q8 pixels
    signal c_d6    : unsigned(7 downto 0) := (others => '0');             -- depth, Q6 logit
    signal c_f6    : signed(9 downto 0) := (others => '0');               -- fold, Q6 logit
    signal c_ph    : unsigned(25 downto 0) := (others => '0');            -- wave phase, Q26 cycles
    signal c_dph   : unsigned(21 downto 0) := (others => '0');
    signal c_gk16  : unsigned(13 downto 0) := (others => '0');
    signal c_gkr   : unsigned(16 downto 0) := (others => '0');
    signal c_gr    : unsigned(15 downto 0) := (others => '0');
    signal c_p0    : unsigned(16 downto 0) := (others => '0');            -- column phase of pixel 0, Q8 px
    signal c_k0    : unsigned(7 downto 0) := (others => '0');             -- its column (mod 256)
    signal c_wm512 : unsigned(16 downto 0) := (others => '0');
    signal c_256mw : unsigned(16 downto 0) := (others => '0');            -- 256 - Wpx8 (mod 2^17)

    -- [E] engine state (the row unit works in the nxt_* registers; so does the walk)
    signal e_st   : unsigned(5 downto 0) := (others => '0');
    attribute fsm_encoding of e_st : signal is "none";
    signal e_row, e_first, e_init, e_init2, e_kick : std_logic := '0';
    signal e_sh   : unsigned(2 downto 0) := (others => '0');
    signal e_a4   : unsigned(11 downto 0) := (others => '0');
    signal e_st4  : unsigned(2 downto 0) := (others => '0');
    signal e_cwb  : unsigned(14 downto 0) := (others => '0');
    signal e_q    : signed(19 downto 0) := (others => '0');
    signal e_k    : unsigned(7 downto 0) := (others => '0');
    signal e_cnt  : unsigned(7 downto 0) := (others => '0');
    signal e_neg, e_zneg : std_logic := '0';
    signal e_rc   : unsigned(7 downto 0) := (others => '0');
    signal e_dw   : unsigned(7 downto 0) := (others => '0');
    signal e_z    : signed(9 downto 0) := (others => '0');
    signal e_su   : unsigned(9 downto 0) := (others => '0');
    signal e_ipc  : unsigned(9 downto 0) := (others => '0');
    signal e_sin  : unsigned(9 downto 0) := (others => '0');
    signal e_shift, e_wait, e_run : std_logic := '0';   -- e_run: this field's rows are set up (no engine runs from configuration)
    -- engine serial multiplier (16 iterations, op-code operand fetch) and divider (28 iterations, quotient in d_num)
    signal m_go, m_busy, m_done : std_logic := '0';
    signal m_op   : unsigned(4 downto 0) := (others => '0');
    signal m_oa   : unsigned(21 downto 0) := (others => '0');
    signal m_hi   : unsigned(22 downto 0) := (others => '0');
    signal m_lo   : unsigned(15 downto 0) := (others => '0');
    signal m_n    : unsigned(4 downto 0) := (others => '0');
    signal m_p    : unsigned(38 downto 0);
    signal d_go, d_busy, d_done : std_logic := '0';
    signal d_op   : unsigned(1 downto 0) := (others => '0');
    signal d_od   : unsigned(16 downto 0) := (others => '0');
    signal d_num  : unsigned(27 downto 0) := (others => '0');
    signal d_rem  : unsigned(17 downto 0) := (others => '0');
    signal d_n    : unsigned(4 downto 0) := (others => '0');

    -- rows: previous / current / next (the row unit writes next; a shift moves next -> cur -> prev)
    signal cur_j, nxt_j : unsigned(7 downto 0) := (others => '0');
    signal cur_Y, nxt_Y : signed(20 downto 0) := (others => '0');
    signal cur_H, nxt_H : unsigned(16 downto 0) := to_unsigned(3072, 17);
    signal nxt_th : unsigned(13 downto 0) := (others => '0');
    signal cur_T0, cur_T1, nxt_T0, nxt_T1 : signed(5 downto 0) := (others => '0');
    signal cur_tq, nxt_tq : unsigned(15 downto 0) := (others => '0');
    signal cur_invR, nxt_invR : unsigned(8 downto 0) := (others => '0');
    signal cur_hwR, nxt_hwR : unsigned(6 downto 0) := (others => '0');
    signal cur_cw, nxt_cw : unsigned(11 downto 0) := (others => '0');

    -- [L] line engine: computes the NEXT line's registers (nl_*); copied into l_* at the line start
    signal le_st  : unsigned(4 downto 0) := (others => '0');
    attribute fsm_encoding of le_st : signal is "none";
    signal le_k0, le_first, le_vb, le_seen, le_shift : std_logic := '0';
    signal le_yc  : unsigned(19 downto 0) := (others => '0');
    signal le_end : signed(20 downto 0) := (others => '0');
    signal le_v8  : unsigned(16 downto 0) := (others => '0');
    signal le_hv  : unsigned(16 downto 0) := (others => '0');
    signal le_pos : signed(17 downto 0) := (others => '0');
    signal le_hn  : signed(17 downto 0) := (others => '0');
    signal lm_go, lm_busy, lm_done, le_wait : std_logic := '0';
    signal lm_op  : unsigned(1 downto 0) := (others => '0');
    signal lm_oa  : signed(20 downto 0) := (others => '0');
    signal lm_hi  : signed(21 downto 0) := (others => '0');
    signal lm_lo  : unsigned(16 downto 0) := (others => '0');
    signal lm_n   : unsigned(4 downto 0) := (others => '0');
    signal lm_p   : signed(38 downto 0);
    signal le_ft  : std_logic := '0';
    signal le_fa  : unsigned(6 downto 0) := (others => '0');
    signal nl_pos0 : unsigned(16 downto 0) := (others => '0');
    signal nl_kl, nl_tj : unsigned(7 downto 0) := (others => '0');
    signal nl_wrap0, nl_smp, nl_tbank, le_pl : std_logic := '0';
    signal nl_hc  : unsigned(4 downto 0) := (others => '0');
    signal l_tj   : unsigned(7 downto 0) := (others => '0');
    signal l_smp, l_tbank, l_camen, l_pc, l_jbp, l_bank : std_logic := '0';
    signal l_Tc0, l_Tc1 : signed(5 downto 0) := (others => '0');
    signal l_cv   : std_logic_vector(25 downto 0) := (others => '0');          -- this row's valley crease colour
    signal l_hc   : unsigned(4 downto 0) := (others => '0');
    signal l_invR : unsigned(8 downto 0) := (others => '0');
    signal l_hwR  : unsigned(6 downto 0) := (others => '0');
    signal l_crow : unsigned(7 downto 0) := (others => '0');

    -- [B] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
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
    signal s_sv  : unsigned(7 downto 0) := (others => '0');
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    signal s_sd  : t_u8a(1 to C_SRC_D) := (others => (others => '0'));

    -- [K] camera writer
    signal k_acc  : unsigned(16 downto 0) := (others => '0');
    signal k_tot  : unsigned(19 downto 0) := (others => '0');
    signal k_end, kl_go, kl_op, k_tb : std_logic := '0';
    signal k_ph   : unsigned(1 downto 0) := (others => '0');
    signal k_tj   : unsigned(7 downto 0) := (others => '0');
    signal km_go, km_busy : std_logic := '0';
    signal km_a   : unsigned(19 downto 0) := (others => '0');
    signal km_hi  : unsigned(20 downto 0) := (others => '0');
    signal km_lo  : unsigned(16 downto 0) := (others => '0');
    signal km_n   : unsigned(4 downto 0) := (others => '0');
    signal km_k   : unsigned(7 downto 0) := (others => '0');
    signal km_done : std_logic := '0';
    signal k_last  : std_logic := '0';
    signal k_lk    : unsigned(7 downto 0) := (others => '0');
    signal k_fst   : std_logic := '0';
    signal km_ed, k_xe : std_logic_vector(1 downto 0) := "00";   -- the column is the line's last (1) / first (0)
    signal cam_crow : unsigned(7 downto 0) := (others => '0');

    -- [P] pixel path
    signal a_pos  : unsigned(16 downto 0) := (others => '0');
    signal a_k    : unsigned(7 downto 0) := (others => '0');
    signal a_wrap : std_logic := '0';
    signal w_c, w_n : unsigned(7 downto 0) := (others => '0');
    signal q2_pos, q2_dr : unsigned(11 downto 0) := (others => '0');
    signal q2_left, q2_wrap : std_logic := '0';
    signal q2_k   : unsigned(7 downto 0) := (others => '0');
    signal k7     : unsigned(7 downto 0) := (others => '0');
    signal sb_wr  : std_logic_vector(3 to 7) := (others => '0');
    signal q3_d   : unsigned(11 downto 0) := (others => '0');
    signal q3_cp, q3_kcp, q3_hmp : std_logic := '0';
    signal q4_pa, q4_pb, q4_pc : unsigned(14 downto 0) := (others => '0');
    signal q8_co  : signed(7 downto 0) := (others => '0');
    signal q8_kcp, q8_hmp : std_logic := '0';
    signal q5_s1  : unsigned(18 downto 0) := (others => '0');
    signal q5_pc  : unsigned(14 downto 0) := (others => '0');
    signal q6_pr  : unsigned(20 downto 0) := (others => '0');
    signal q7_u   : unsigned(6 downto 0) := (others => '0');
    signal q8_cov : unsigned(4 downto 0) := (others => '0');
    signal q8_zc  : unsigned(4 downto 0) := (others => '0');
    signal sb_kcp, sb_hmp : std_logic_vector(4 to 8) := (others => '0');
    signal sb_cp  : std_logic_vector(4 to 8) := (others => '0');
    signal q9_toc, q9_tnc : signed(8 downto 0) := (others => '0');
    signal q9_cc  : unsigned(4 downto 0) := (others => '0');
    signal q9_cov : unsigned(4 downto 0) := (others => '0');
    signal sb_cs  : std_logic_vector(9 to 15) := (others => '0');     -- the crease: 0 mountain (accent), 1 valley
    signal q10_dc : signed(9 downto 0) := (others => '0');
    signal q10_tc : signed(8 downto 0) := (others => '0');
    signal q10_cov : unsigned(4 downto 0) := (others => '0');
    signal q10_cc : unsigned(4 downto 0) := (others => '0');
    signal q11_pac, q11_pbc : signed(14 downto 0) := (others => '0');
    signal q11_tc : signed(8 downto 0) := (others => '0');
    signal q11_cc : unsigned(4 downto 0) := (others => '0');
    signal q12_tc : signed(10 downto 0) := (others => '0');
    type t_u5a is array (natural range <>) of unsigned(4 downto 0);
    signal sc_cc  : t_u5a(12 to 16) := (others => (others => '0'));
    signal q15_f1 : std_logic_vector(25 downto 0) := (others => '0');
    signal q16_dy : signed(10 downto 0) := (others => '0');
    signal q16_du, q16_dv : signed(8 downto 0) := (others => '0');
    signal q16_y  : unsigned(9 downto 0) := (others => '0');
    signal q16_u, q16_v : unsigned(7 downto 0) := (others => '0');
    signal q17_py0, q17_py1 : signed(15 downto 0) := (others => '0');
    signal q17_pu, q17_pv : signed(11 downto 0) := (others => '0');
    signal q17_y  : unsigned(9 downto 0) := (others => '0');
    signal q17_u, q17_v : unsigned(7 downto 0) := (others => '0');
    signal q18_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    type t_u8 is array (1 to 2) of unsigned(7 downto 0);
    signal q18_c  : t_u8 := (others => to_unsigned(128, 8));

    -- [F] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_ysr  : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv, s_bud, s_bvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y31, s_y32, s_y33 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u31, s_v31, s_u32, s_v32, s_u33, s_v33 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

    -- fill (facet tables)
    signal f_st   : unsigned(3 downto 0) := (others => '0');
    signal f_pp, f_dir : std_logic := '0';
    signal f_ch   : unsigned(1 downto 0) := (others => '0');
    signal f_cnt  : unsigned(4 downto 0) := (others => '0');
    signal f_adr  : unsigned(5 downto 0) := (others => '0');
    signal fm_a   : signed(10 downto 0) := (others => '0');
    signal fm_hi  : signed(11 downto 0) := (others => '0');
    signal fm_lo  : unsigned(7 downto 0) := (others => '0');
    signal fm_n   : unsigned(3 downto 0) := (others => '0');
    signal f_s    : signed(19 downto 0) := (others => '0');
    signal f_a    : signed(23 downto 0) := (others => '0');
    signal f_p    : unsigned(9 downto 0) := (others => '0');

begin

    m_p  <= m_hi(22 downto 0) & m_lo;
    lm_p <= lm_hi(21 downto 0) & signed(lm_lo);

    ---------------------------------------------------------------------------
    -- Block RAM: ROMs, tables, camera stores, delay rings (one write statement
    -- each; none ever reads the address it is writing, except as noted)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_MI_EXP2(to_integer(r_ex_a));
            r_sn_q <= C_MI_SIN(to_integer(r_sn_a));
            r_pl_q <= C_MI_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    p_fty : process(clk)
    begin
        if rising_edge(clk) then
            fty_q <= fty(to_integer(ft1_a));
            if ft_we(0) = '1' then fty(to_integer(ft_wa)) <= "000000" & ft_wd; end if;
        end if;
    end process p_fty;
    p_ftu : process(clk)
    begin
        if rising_edge(clk) then
            ftu_q <= ftu(to_integer(ft1_a));
            if ft_we(1) = '1' then ftu(to_integer(ft_wa)) <= ft_wd(7 downto 0); end if;
        end if;
    end process p_ftu;
    p_ftv : process(clk)
    begin
        if rising_edge(clk) then
            ftv_q <= ftv(to_integer(ft1_a));
            if ft_we(2) = '1' then ftv(to_integer(ft_wa)) <= ft_wd(7 downto 0); end if;
        end if;
    end process p_ftv;
    ft1_q <= fty_q(9 downto 0) & ftu_q & ftv_q;

    -- camera store: the writer fills the bank of the next row while the pixel path reads the current row's bank
    -- (line 0 reads and writes one bank, but its reads are discarded)
    p_cv : process(clk)
    begin
        if rising_edge(clk) then
            cv_q <= cvm(to_integer(cv_ra));
            cv_qf <= cv_q;
            if cv_we = '1' then cvm(to_integer(cv_wa)) <= cv_wd; end if;
        end if;
    end process p_cv;

    p_rc : process(clk)
    begin
        if rising_edge(clk) then
            rc_q <= rcm(to_integer(rc_ra));
            if rc_we = '1' then rcm(to_integer(rc_wa)) <= rc_wd; end if;
        end if;
    end process p_rc;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            cr(to_integer(rc + 6)) <= std_logic_vector(q18_c(1)) & std_logic_vector(q18_c(2));
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            sr(to_integer(rc + (C_LAT - 2))) <= '1' & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
            rc <= rc + 1;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- [A] Controls, vblank zone, glides, palette (the zone sequencer's fixed slots)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
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
            if s_zone = '1' and s_zc = 23 then s_z24 <= '1'; else s_z24 <= '0'; end if;   -- registered strobes: zone clocks 24, 40
            if s_zone = '1' and s_zc = 39 then s_z40 <= '1'; else s_z40 <= '0'; end if;
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
            v_s := signed(resize(s_k4, 11)) - to_signed(512, 11);
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
                s_rad  <= registers_in(6)(1);
                s_cre  <= registers_in(6)(2);
                s_top  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                c_ys   <= s_ys;
                c_irv  <= f_irv(s_tid);
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                -- Colour: 16 palettes with hysteresis (the two end zones select from rest)
                v_d := signed(resize(unsigned(registers_in(4)), 11)) - signed(resize(s_pal & "100000", 11));
                if v_d > 40 or v_d < -40 then s_pal <= unsigned(registers_in(4)(9 downto 6)); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide for Scale, Fold and Camera (1/8 of the gap per field, +-2 deadband; camera +-4)
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        if z_gc(1) = '1' then s_gm <= s_gk; s_gr <= unsigned(registers_in(5));
                        elsif z_gc(0) = '0' then s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                        else s_gm <= s_gp; s_gr <= unsigned(registers_in(7)); end if;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if C_SIM_SNAP then
                            null;
                        elsif (s_gd > 16 or s_gd < -16) and (z_gc(1) = '0' or s_gd > 32 or s_gd < -32) then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        if z_gc(1) = '1' then s_gk <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        elsif z_gc(0) = '0' then s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        else s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0)); end if;
                        z_gp <= "00";
                        if z_gc = 2 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Facet-table fill (zone clock 40 on; its own 8-step serial multiplier).
    -- Twelve passes, one channel at a time: paper A light Y U V, A dark Y U V, B light, B dark.  Light: tone
    -- t = 0 .. 31 at addresses 0 .. 31, P + (P->white) * 0.55 t / 32; dark: t = -1 .. -32 at 63 .. 32,
    -- P + (P->black) * 0.8 |t| / 32.  The valley crease colour is the dark end (t = -32) of each paper.
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_s : signed(11 downto 0);
        variable v_w : signed(10 downto 0);
        variable v_t : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            ft_we <= "000";
            if fm_n /= 0 then
                if fm_lo(0) = '1' then v_s := fm_hi + resize(fm_a, 12); else v_s := fm_hi; end if;
                fm_hi <= shift_right(v_s, 1);
                fm_lo <= v_s(0) & fm_lo(7 downto 1);
                fm_n <= fm_n - 1;
            end if;
            if s_z24 = '1' then
                f_st <= "1000"; f_pp <= '0'; f_dir <= '0'; f_ch <= "00";
            else
                case to_integer(f_st) is
                    -- the palette: paper A and the accent for the pixel path (the papers' channels come per pass)
                    when 8  => r_pl_a <= "00" & s_pal & "01"; f_st <= "1001";
                    when 9  => r_pl_a <= "00" & s_pal & "11"; f_st <= "1010";
                    when 10 =>
                        c_pay <= unsigned(r_pl_q(31 downto 22)); c_pau <= unsigned(r_pl_q(21 downto 14)); c_pav <= unsigned(r_pl_q(11 downto 4));
                        f_st <= "1011";
                    when 11 =>
                        c_acy <= unsigned(r_pl_q(31 downto 22)); c_acu <= unsigned(r_pl_q(21 downto 14)); c_acv <= unsigned(r_pl_q(11 downto 4));
                        f_st <= "0110";
                    when 6 =>                     -- pass setup: this paper's palette word
                        r_pl_a <= "00" & s_pal & f_pp & (not f_pp);
                        f_st <= "0111";
                    when 7 => f_st <= "0001";
                    when 1 =>                     -- its channel
                        case to_integer(f_ch) is
                            when 0      => f_p <= unsigned(r_pl_q(31 downto 22));
                            when 1      => f_p <= resize(unsigned(r_pl_q(21 downto 14)), 10);
                            when others => f_p <= resize(unsigned(r_pl_q(11 downto 4)), 10);
                        end case;
                        f_st <= "0010";
                    when 2 =>
                        if f_ch /= 0 then v_t := to_signed(128, 11);
                        elsif f_dir = '0' then v_t := to_signed(940, 11);
                        else v_t := to_signed(64, 11); end if;
                        fm_a <= v_t - signed(resize(f_p, 11));
                        fm_hi <= (others => '0'); fm_n <= to_unsigned(8, 4);
                        if f_dir = '0' then fm_lo <= to_unsigned(141, 8); else fm_lo <= to_unsigned(205, 8); end if;
                        f_st <= "0011";
                    when 3 =>                     -- step = (target - P) * K
                        if fm_n = 0 then
                            f_s <= resize(fm_hi(10 downto 0) & signed(fm_lo), 20);
                            f_st <= "0100";
                        end if;
                    when 4 =>
                        if f_dir = '0' then f_a <= (others => '0'); f_adr <= (others => '0');
                        else f_a <= resize(f_s, 24); f_adr <= (others => '1'); end if;
                        f_cnt <= (others => '0');
                        f_st <= "0101";
                    when 5 =>                     -- one entry per clock
                        v_w := signed(resize(f_p, 11)) + f_a(23 downto 13);
                        ft_we(to_integer(f_ch)) <= '1';
                        ft_wa <= f_pp & f_adr;
                        ft_wd <= std_logic_vector(v_w(9 downto 0));
                        f_a <= f_a + resize(f_s, 24);
                        if f_dir = '0' then f_adr <= f_adr + 1; else f_adr <= f_adr - 1; end if;
                        f_cnt <= f_cnt + 1;
                        if f_cnt = 31 then
                            f_st <= "0110";
                            if f_ch /= 2 then
                                f_ch <= f_ch + 1;
                            else
                                f_ch <= "00";
                                if f_dir = '0' then f_dir <= '1';
                                elsif f_pp = '0' then f_dir <= '0'; f_pp <= '1';
                                else f_st <= "0000"; end if;
                            end if;
                        end if;
                    when others => null;
                end case;
            end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- [E] Engine multiplier: 16-step right-shifting shift-add; the operands are
    -- fetched by op code on the go clock (states write only m_op + m_go).  Ops
    -- that take the previous product read m_p, which holds until the next go.
    ---------------------------------------------------------------------------
    p_emul : process(clk)
        variable v_s : unsigned(22 downto 0);
    begin
        if rising_edge(clk) then
            m_done <= '0';
            if m_go = '1' then
                m_busy <= '1';
                m_hi <= (others => '0'); m_n <= to_unsigned(16, 5);
                m_oa <= (others => '0'); m_lo <= (others => '0');
                case to_integer(m_op) is
                    when 1  => m_oa <= resize(r_ex_q, 22);             m_lo <= to_unsigned(896, 16);  -- pitch
                    when 2  => m_oa <= f_rx(s_xs);                     m_lo <= resize(c_wc4, 16);     -- pitch in pixels
                    when 3  => m_oa <= resize(c_wc4, 22);              m_lo <= resize(unsigned(registers_in(1)), 16);  -- pitch * shape
                    when 5  => m_oa <= f_rx(s_xs);                     m_lo <= resize(e_a4, 16);      -- amplitude in pixels
                    when 6  => m_oa <= resize(unsigned(registers_in(2)), 22); m_lo <= resize(unsigned(registers_in(2)), 16);  -- depth^2
                    when 8  => m_oa <= to_unsigned(436, 22);           m_lo <= resize(s_gp, 16);      -- fold
                    when 9  => m_oa <= to_unsigned(900, 22);           m_lo <= resize(s_vm, 16);      -- rate exponent
                    when 10 => m_oa <= to_unsigned(1193046, 22);       m_lo <= r_ex_q;                -- rate
                    when 11 => m_oa <= to_unsigned(38364, 22);         m_lo <= resize(s_gk(12 downto 3), 16);  -- camera gain
                    when 13 => m_oa <= f_kgr(s_xs);                    m_lo <= resize(s_gk(12 downto 3), 16);
                    when 15 => m_oa <= f_krr(s_xs);                    m_lo <= resize(s_gk(12 downto 3), 16);
                    when 17 => m_oa <= resize(r_sn_q(9 downto 0), 22); m_lo <= resize(c_d6, 16);      -- depth * wave
                    when 18 => m_oa <= resize(c_wc4, 22);              m_lo <= resize(r_sn_q(9 downto 0), 16);  -- height
                    when 19 =>                                                                       -- tone, column 0
                        m_oa <= resize(e_sin, 22);
                        if nxt_j(0) = '1' then m_lo <= to_unsigned(31, 16);
                        elsif s_top = '0' then m_lo <= to_unsigned(23, 16);
                        else m_lo <= to_unsigned(17, 16); end if;
                    when 20 =>                                                                       -- tone, column 1
                        m_oa <= resize(e_sin, 22);
                        if nxt_j(0) = '0' then m_lo <= to_unsigned(31, 16);
                        elsif s_top = '0' then m_lo <= to_unsigned(23, 16);
                        else m_lo <= to_unsigned(17, 16); end if;
                    when 21 => m_oa <= resize(nxt_tq, 22);             m_lo <= resize(c_ys, 16);      -- lean per field line
                    when others => m_oa <= resize(nxt_cw, 22);         m_lo <= f_rxh(s_xs);           -- crease half-width
                end case;
            elsif m_n /= 0 then
                if m_lo(0) = '1' then v_s := m_hi + resize(m_oa, 23); else v_s := m_hi; end if;
                m_hi <= '0' & v_s(22 downto 1);
                m_lo <= v_s(0) & m_lo(15 downto 1);
                m_n <= m_n - 1;
                if m_n = 1 then m_busy <= '0'; m_done <= '1'; end if;
            end if;
        end if;
    end process p_emul;

    -- engine divider: 28-step restoring division; the quotient shifts into the numerator register
    p_ediv : process(clk)
        variable v_r : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            d_done <= '0';
            if d_go = '1' then
                d_busy <= '1'; d_rem <= (others => '0'); d_n <= to_unsigned(28, 5);
                case to_integer(d_op) is
                    when 0      => d_num <= resize(c_apx8 & x"000", 28); d_od <= nxt_H;                           -- lean A / H
                    when 1      => d_num <= to_unsigned(65536, 28);      d_od <= resize(m_p(27 downto 12), 17) + 256;  -- 1 / (1 + lean)
                    when others => d_num <= resize(m_p(25 downto 0), 28); d_od <= c_wpx8;                          -- camera gain / pitch
                end case;
            elsif d_n /= 0 then
                v_r := d_rem(16 downto 0) & d_num(27);
                if v_r >= resize(d_od, 18) then
                    d_rem <= v_r - resize(d_od, 18); d_num <= d_num(26 downto 0) & '1';
                else
                    d_rem <= v_r; d_num <= d_num(26 downto 0) & '0';
                end if;
                d_n <= d_n - 1;
                if d_n = 1 then d_busy <= '0'; d_done <= '1'; end if;
            end if;
        end if;
    end process p_ediv;

    ---------------------------------------------------------------------------
    -- [E] Engine: per-field constants (zone clock 40 on), the row walk (row 0
    -- centred on line 540, stacked upward until a row top passes the screen
    -- top), the first two rows, the line-0 kick; then one row per line-engine
    -- shift.  Restarted from state 1 at every vblank (the watchdog).  The row
    -- unit and the walk both work in the nxt_* registers.
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_z   : signed(11 downto 0);
        variable v_au  : unsigned(8 downto 0);
        variable v_pq  : unsigned(10 downto 0);
        variable v_t   : unsigned(4 downto 0);
        variable v_h   : unsigned(18 downto 0);
        variable v_th  : unsigned(13 downto 0);
        variable v_e   : unsigned(9 downto 0);
        variable v_sq  : unsigned(21 downto 0);
        variable v_a4  : unsigned(25 downto 0);
        variable v_cw  : unsigned(14 downto 0);
        variable v_b   : unsigned(25 downto 0);
        variable v_a27 : unsigned(26 downto 0);
    begin
        if rising_edge(clk) then
            m_go <= '0'; d_go <= '0'; e_shift <= '0'; le_k0 <= '0';
            if le_shift = '1' then e_kick <= '1'; end if;
            if m_done = '1' or d_done = '1' then e_wait <= '0'; end if;
            if s_zgo = '1' then
                e_st <= (others => '0'); e_kick <= '0'; e_wait <= '0'; e_run <= '0';
            elsif s_z40 = '1' then
                e_st <= to_unsigned(1, 6);
            elsif e_wait = '0' then          -- a state that starts a product or a division waits for its done pulse
                e_st <= e_st + 1;
                case to_integer(e_st) is
                    when 0 =>
                        if e_kick = '1' then e_kick <= '0'; e_st <= to_unsigned(57, 6); else e_st <= (others => '0'); end if;
                    -- per-field constants ----------------------------------------------------------------
                    when 1 =>                                      -- pitch exponent e = Scale * 2.5 octaves (Q8)
                        v_e := resize(s_g1(12 downto 4), 10) + resize(s_g1(12 downto 6), 10);
                        r_ex_a <= v_e(7 downto 0); e_sh <= resize(v_e(9 downto 8), 3);
                    when 3 => m_op <= to_unsigned(1, 5); m_go <= '1'; e_wait <= '1';
                    when 4 =>
                        case to_integer(e_sh(1 downto 0)) is
                            when 0      => c_wc4 <= m_p(27 downto 15);
                            when 1      => c_wc4 <= m_p(26 downto 14);
                            when others => c_wc4 <= m_p(25 downto 13);
                        end case;
                        m_op <= to_unsigned(2, 5); m_go <= '1'; e_wait <= '1';
                    when 5 => c_wpx8 <= m_p(28 downto 12); m_op <= to_unsigned(3, 5); m_go <= '1'; e_wait <= '1';
                    when 6 =>                                      -- amplitude A = 5/8 pitch * shape (Q4)
                        v_a4 := resize(m_p(22 downto 0), 26) + resize(m_p(22 downto 0) & "00", 26);
                        e_a4 <= v_a4(24 downto 13);
                    when 7 => m_op <= to_unsigned(5, 5); m_go <= '1'; e_wait <= '1';
                    when 8 => c_apx8 <= m_p(27 downto 12); m_op <= to_unsigned(6, 5); m_go <= '1'; e_wait <= '1';
                    when 9 =>
                        v_sq := resize(m_p(19 downto 0), 22) + resize(m_p(19 downto 0) & '0', 22);   -- depth^2 * 3
                        c_d6 <= v_sq(21 downto 14);
                        m_op <= to_unsigned(8, 5); m_go <= '1'; e_wait <= '1';
                    when 10 =>
                        c_f6 <= signed(resize(m_p(21 downto 13), 10)) - to_signed(218, 10);
                        m_op <= to_unsigned(9, 5); m_go <= '1'; e_wait <= '1';
                    when 11 => r_ex_a <= m_p(15 downto 8); e_sh <= m_p(18 downto 16);
                    when 13 => m_op <= to_unsigned(10, 5); m_go <= '1'; e_wait <= '1';
                    when 14 => c_dph <= resize(m_p(36 downto 21), 22);
                    when 15 =>                                     -- serial shift by the exponent's octave
                        if e_sh /= 0 then
                            c_dph <= c_dph(20 downto 0) & '0'; e_sh <= e_sh - 1; e_st <= to_unsigned(15, 6);
                        else
                            if s_vm = 0 then c_dph <= (others => '0'); end if;
                            m_op <= to_unsigned(11, 5); m_go <= '1'; e_wait <= '1';
                        end if;
                    when 16 => d_op <= "10"; d_go <= '1'; e_wait <= '1';
                    when 17 => c_gk16 <= d_num(13 downto 0);
                    when 18 =>
                        if s_src = '1' then c_gk16 <= c_gk16(12 downto 0) & '0'; end if;   -- Osc x2
                        m_op <= to_unsigned(13, 5); m_go <= '1'; e_wait <= '1';
                    when 19 => c_gkr <= m_p(23 downto 7);
                    when 20 =>
                        if s_src = '1' then c_gkr <= c_gkr(15 downto 0) & '0'; end if;
                        m_op <= to_unsigned(15, 5); m_go <= '1'; e_wait <= '1';
                    when 21 => c_gr <= m_p(23 downto 8);
                    when 22 =>                                     -- column origin: pixel 0's centre in the crease grid
                        if s_src = '1' then c_gr <= c_gr(14 downto 0) & '0'; end if;
                        e_q <= to_signed(128, 20) - signed(resize(f_wrh(s_xs) & x"00", 20)) + signed(resize(c_apx8(15 downto 1), 20));
                        e_k <= (others => '0');
                        c_wm512 <= c_wpx8 - 512;
                        c_256mw <= to_unsigned(256, 17) - c_wpx8;
                    when 23 =>
                        if e_q(19) = '1' then
                            e_q <= e_q + signed(resize(c_wpx8, 20)); e_k <= e_k - 1; e_st <= to_unsigned(23, 6);
                        else
                            c_p0 <= unsigned(e_q(16 downto 0)); c_k0 <= e_k;
                        end if;
                    -- the walk ----------------------------------------------------------------------------
                    when 24 =>
                        nxt_th <= c_ph(25 downto 12); nxt_j <= (others => '0'); e_cnt <= (others => '0');
                        e_first <= '1'; rc_ra <= (others => '0'); e_row <= '0'; e_init <= '0'; e_init2 <= '0';
                    -- 25, 26: the rowcam read
                    -- row_h: wave -> fold -> S-curve -> sin, cos -> height -------------------------------
                    when 27 =>
                        v_th := nxt_th;
                        e_neg <= v_th(13);
                        if v_th(12) = '1' then v_th(11 downto 0) := not v_th(11 downto 0); end if;
                        if s_snap = '0' then r_sn_a <= v_th(11 downto 2);
                        elsif v_th(11 downto 10) /= "00" then r_sn_a <= (others => '1');
                        else r_sn_a <= v_th(9 downto 0); end if;
                    when 29 => e_rc <= unsigned(rc_q); m_op <= to_unsigned(17, 5); m_go <= '1'; e_wait <= '1';
                    when 30 => e_dw <= m_p(17 downto 10);
                    when 31 =>
                        if e_neg = '1' then
                            v_z := resize(c_f6, 12) - signed(resize(e_dw, 12)) + signed(resize(e_rc, 12));
                        else
                            v_z := resize(c_f6, 12) + signed(resize(e_dw, 12)) + signed(resize(e_rc, 12));
                        end if;
                        if v_z < -256 then e_z <= to_signed(-256, 10);
                        elsif v_z > 255 then e_z <= to_signed(255, 10);
                        else e_z <= v_z(9 downto 0); end if;
                    when 32 =>
                        if e_z(9) = '1' then
                            if e_z = -256 then v_au := to_unsigned(255, 9); else v_au := unsigned(-e_z(8 downto 0)); end if;
                        else
                            v_au := unsigned(e_z(8 downto 0));
                        end if;
                        r_sn_a <= v_au(7 downto 0) & "00";
                        e_zneg <= e_z(9);
                    when 34 => e_su <= r_sn_q(9 downto 0);
                    when 35 =>
                        if e_zneg = '1' then v_pq := to_unsigned(1024, 11) - resize(e_su, 11);
                        else v_pq := to_unsigned(1024, 11) + resize(e_su, 11); end if;
                        r_sn_a <= v_pq(10 downto 1);
                        e_ipc <= not v_pq(10 downto 1);
                    when 36 => r_sn_a <= e_ipc;
                    when 37 => e_sin <= r_sn_q(9 downto 0);
                    when 38 => m_op <= to_unsigned(18, 5); m_go <= '1'; e_wait <= '1';
                    when 39 =>
                        if m_p(28 downto 6) < 3072 then nxt_H <= to_unsigned(3072, 17);
                        else nxt_H <= m_p(22 downto 6); end if;
                        if e_row = '1' then e_st <= to_unsigned(42, 6); end if;
                    -- walk: stack upward ----------------------------------------------------------------
                    when 40 =>
                        if e_first = '1' then
                            nxt_Y <= to_signed(138240, 21) - signed(resize(nxt_H(16 downto 1), 21)); e_first <= '0';
                        else
                            nxt_Y <= nxt_Y - signed(resize(nxt_H, 21));
                        end if;
                    when 41 =>
                        if nxt_Y(20) = '1' or nxt_Y = 0 or e_cnt = 255 then
                            rc_ra <= nxt_j; e_row <= '1'; e_init <= '1';           -- the top row: compute it in full
                        else
                            nxt_j <= nxt_j - 1; rc_ra <= nxt_j - 1; e_cnt <= e_cnt + 1;
                            if s_rad = '1' then nxt_th <= nxt_th - C_S14; else nxt_th <= nxt_th + C_S14; end if;
                        end if;
                        e_st <= to_unsigned(25, 6);
                    -- the rest of the row unit ------------------------------------------------------------
                    when 42 =>
                        v_h := resize(nxt_H & "00", 19) + resize(nxt_H, 19);      -- 5 H, then / 16
                        if v_h(18 downto 4) > 2048 then nxt_cw <= to_unsigned(2048, 12); else nxt_cw <= v_h(15 downto 4); end if;
                        v_t := resize(shift_right(resize(e_sin, 11) + 128, 8), 5);  -- crease strength 4 .. 8 (eighths)
                        e_st4 <= v_t(2 downto 0);
                        e_cwb <= (others => '0');
                        m_op <= to_unsigned(19, 5); m_go <= '1'; e_wait <= '1';
                    when 43 =>
                        v_t := m_p(14 downto 10);
                        if s_top = '1' and nxt_j(0) = '0' then nxt_T0 <= -signed(resize(v_t, 6)); else nxt_T0 <= signed(resize(v_t, 6)); end if;
                        m_op <= to_unsigned(20, 5); m_go <= '1'; e_wait <= '1';
                    when 44 =>
                        v_t := m_p(14 downto 10);
                        if s_top = '1' and nxt_j(0) = '1' then nxt_T1 <= signed(resize(v_t, 6)); else nxt_T1 <= -signed(resize(v_t, 6)); end if;
                        d_op <= "00"; d_go <= '1'; e_wait <= '1';
                    when 45 =>                                      -- crease weight * strength / 8, in two adds
                        nxt_tq <= d_num(15 downto 0); m_op <= to_unsigned(21, 5); m_go <= '1'; e_wait <= '1';
                        e_cwb <= f_gu(nxt_cw, e_st4(0), 15) + shift_left(f_gu(nxt_cw, e_st4(1), 15), 1);
                    when 46 =>
                        d_op <= "01"; d_go <= '1'; e_wait <= '1';
                        v_cw := resize(nxt_cw & "00", 15) + e_cwb + shift_left(f_gu(nxt_cw, e_st4(2), 15), 2);
                        nxt_cw <= v_cw(14 downto 3);
                    when 47 =>                                      -- crease half-width in sixteenths of the ramp
                        nxt_invR <= d_num(8 downto 0);
                        m_op <= to_unsigned(22, 5); m_go <= '1'; e_wait <= '1';
                        e_st <= to_unsigned(54, 6);
                    when 54 =>
                        if m_p(27 downto 16) > 127 then nxt_hwR <= to_unsigned(127, 7); else nxt_hwR <= m_p(22 downto 16); end if;
                        if e_init = '1' then e_st <= to_unsigned(55, 6);
                        elsif e_init2 = '1' then e_st <= to_unsigned(58, 6);
                        else e_st <= (others => '0'); end if;
                    when 55 =>                                      -- the top row is in next: shift it to current
                        e_shift <= '1'; e_init <= '0'; e_init2 <= '1';
                    when 57 =>                                      -- the row after it (next still holds it)
                        nxt_j <= nxt_j + 1; rc_ra <= nxt_j + 1;
                        if s_rad = '1' and nxt_j(7) = '1' then nxt_th <= nxt_th + C_S14; else nxt_th <= nxt_th - C_S14; end if;
                        nxt_Y <= nxt_Y + signed(resize(nxt_H, 21));
                        e_row <= '1';
                        e_st <= to_unsigned(25, 6);
                    when 58 =>                                      -- line 0, then the phase for the next field
                        le_k0 <= '1'; e_run <= '1';                 -- the line engine and camera run this field
                        v_b := resize(c_dph, 26);
                        if s_vneg = '1' then v_b := not v_b; end if;
                        v_a27 := (c_ph & '1') + (v_b & s_vneg);
                        c_ph <= v_a27(26 downto 1);
                        e_init2 <= '0'; e_kick <= '0';
                        e_st <= (others => '0');
                    when others => null;
                end case;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- Row registers: a shift moves next -> cur -> prev (the engine writes next)
    ---------------------------------------------------------------------------
    p_rows : process(clk)
    begin
        if rising_edge(clk) then
            if le_shift = '1' or e_shift = '1' then
                cur_j <= nxt_j; cur_Y <= nxt_Y; cur_H <= nxt_H;
                cur_T0 <= nxt_T0; cur_T1 <= nxt_T1; cur_tq <= nxt_tq; cur_invR <= nxt_invR;
                cur_hwR <= nxt_hwR; cur_cw <= nxt_cw;
            end if;
        end if;
    end process p_rows;

    ---------------------------------------------------------------------------
    -- [L] Line engine: kicked at every line start (and once in vblank for line
    -- 0), computes the NEXT line: row advance, sample-line flag, the crease x
    -- offset -> column start, the row-edge weight, the horizontal crease; then,
    -- once this line has left the pixel path, reads the neighbouring row's two
    -- colours from the facet table and preloads the camera window.
    ---------------------------------------------------------------------------
    p_lmul : process(clk)
        variable v_s : signed(21 downto 0);
    begin
        if rising_edge(clk) then
            lm_done <= '0';
            if lm_go = '1' then
                lm_busy <= '1'; lm_hi <= (others => '0'); lm_n <= to_unsigned(17, 5);
                case to_integer(lm_op) is
                    when 0      => lm_oa <= signed(resize(cur_tq, 21)); lm_lo <= le_v8;                       -- offset
                    when others => lm_oa <= resize(le_hn, 21);          lm_lo <= resize(c_irv, 17);            -- crease
                end case;
            elsif kl_go = '1' then                         -- the camera writer's row products, at the line end
                lm_busy <= '1'; lm_hi <= (others => '0'); lm_n <= to_unsigned(17, 5);
                lm_oa <= signed(resize(k_tot, 21));
                if kl_op = '0' then lm_lo <= c_gkr; else lm_lo <= resize(c_gr, 17); end if;
            elsif lm_n /= 0 then
                if lm_lo(0) = '1' then v_s := lm_hi + resize(lm_oa, 22); else v_s := lm_hi; end if;
                lm_hi <= shift_right(v_s, 1);
                lm_lo <= v_s(0) & lm_lo(16 downto 1);
                lm_n <= lm_n - 1;
                if lm_n = 1 then lm_busy <= '0'; lm_done <= '1'; end if;
            end if;
        end if;
    end process p_lmul;

    p_line : process(clk)
        variable v_e   : signed(21 downto 0);
        variable v_off : unsigned(16 downto 0);
        variable v_hf  : unsigned(16 downto 0);
        variable v_dy  : unsigned(16 downto 0);
        variable v_a   : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            lm_go <= '0'; le_shift <= '0';
            if g_avid = '1' then le_seen <= '1'; end if;
            if lm_done = '1' then le_wait <= '0'; end if;
            if s_zgo = '1' then
                le_st <= (others => '0'); le_ft <= '0'; le_pl <= '0'; le_wait <= '0';
            elsif le_k0 = '1' or (g_start = '1' and e_run = '1') then
                if le_k0 = '1' then
                    le_yc <= resize(c_ys(11 downto 1), 20); le_first <= '1'; le_vb <= '1';
                else
                    le_yc <= le_yc + resize(c_ys, 20); le_first <= '0'; le_vb <= '0';
                end if;
                le_seen <= '0'; le_ft <= '0'; le_pl <= '0';
                le_st <= to_unsigned(1, 5);
            elsif le_st /= 0 and le_wait = '0' then     -- states after a product wait for its done pulse
                le_st <= le_st + 1;
                case to_integer(le_st) is
                    when 1 => le_end <= cur_Y + signed(resize(cur_H, 21));
                    when 2 => if signed(resize(le_yc, 21)) >= le_end then le_shift <= '1'; end if;
                    when 4 =>
                        le_end <= cur_Y + signed(resize(cur_H, 21));
                        le_v8 <= resize(unsigned(signed(resize(le_yc, 21)) - cur_Y), 17);
                    when 5 =>
                        v_e := signed(resize(le_yc, 22)) + signed(resize(c_ys, 22));
                        if v_e >= resize(le_end, 22) then
                            nl_smp <= '1'; nl_tj <= cur_j + 1; nl_tbank <= not cur_j(0);
                        else
                            nl_smp <= le_first; nl_tj <= cur_j; nl_tbank <= cur_j(0);
                        end if;
                        le_hv <= cur_H - le_v8;
                        lm_op <= "00"; lm_go <= '1'; le_wait <= '1';
                    when 6 =>
                        v_off := unsigned(lm_p(28 downto 12));
                        if cur_j(0) = '0' then
                            le_pos <= signed(resize(c_p0, 18)) - signed(resize(v_off, 18));
                        else
                            le_pos <= signed(resize(c_p0, 18)) - signed(resize(c_apx8, 18)) + signed(resize(v_off, 18));
                        end if;
                    when 7 =>
                        if le_pos(17) = '1' then
                            nl_pos0 <= unsigned(le_pos(16 downto 0) + signed(c_wpx8)); nl_kl <= c_k0 - 1;
                        else
                            nl_pos0 <= unsigned(le_pos(16 downto 0)); nl_kl <= c_k0;
                        end if;
                    when 8 =>
                        if nl_pos0 + 256 >= c_wpx8 then nl_wrap0 <= '1'; else nl_wrap0 <= '0'; end if;
                    when 9 =>
                        if le_v8 < le_hv then v_dy := le_v8; else v_dy := le_hv; end if;
                        le_hn <= signed(resize(cur_cw(11 downto 1), 18)) - signed(resize(v_dy, 18));
                    when 10 => lm_op <= "10"; lm_go <= '1'; le_wait <= '1';
                    when 11 =>
                        v_a := resize(lm_p(31 downto 16), 16) + 8;                           -- |product| / 2^16 < 2^14
                        if v_a(15) = '1' then nl_hc <= (others => '0');
                        elsif v_a > 16 then nl_hc <= to_unsigned(16, 5);
                        else nl_hc <= unsigned(v_a(4 downto 0)); end if;
                    when 12 =>                                     -- wait for this line to leave the pixel path, then
                        -- the row registers the pixel path reads straight from the current row
                        if le_vb = '1' or (le_seen = '1' and g_avid = '0' and s_gav_sr(17) = '0') then
                            l_Tc0 <= cur_T0; l_Tc1 <= cur_T1; l_pc <= cur_j(0); l_bank <= cur_j(0);
                            l_invR <= cur_invR; l_hwR <= cur_hwR;
                            if le_v8 < le_hv then l_jbp <= cur_j(0); else l_jbp <= not cur_j(0); end if;
                            l_camen <= not le_first;
                            cv_pa <= cur_j(0) & nl_kl; le_pl <= '1';
                            le_fa <= cur_j(0) & "100000"; le_ft <= '1';          -- valley crease: this paper at -32
                        else
                            le_st <= to_unsigned(12, 5);
                        end if;
                    -- cv_pa -> cv_ra (p_win) -> RAM -> cv_qf: a camera value is back 4 clocks after its address;
                    -- le_fa -> ft1_a (p_pix) -> RAM -> ft1_q: a table word 3 clocks after
                    when 13 => cv_pa <= l_bank & (nl_kl + 1);
                    when 14 => cv_pa <= l_bank & (nl_kl + 2); le_ft <= '0';
                    when 15 => l_cv <= ft1_q;
                    when 16 => null;                                     -- p_win loads w_c
                    when 17 => le_st <= (others => '0');                -- w_n (le_pl holds until the line starts)
                    when others => null;
                end case;
            end if;
        end if;
    end process p_line;

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
    -- [C] Source: the luma (less black, 8 bits) or the Osc lattice (640 x 540
    -- virtual px, drifting diagonally), delayed to the pixel path's stage 2
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_tx, v_ty : unsigned(6 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_l  : unsigned(7 downto 0);
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
            if data_in.avid = '1' then
                if s_src = '1' then
                    s_sv <= v_s - shift_right(v_s, 3);
                else
                    v_l := unsigned(data_in.y(9 downto 2));
                    if v_l > 16 then s_sv <= v_l - 16; else s_sv <= (others => '0'); end if;
                end if;
            end if;
            s_sd(1) <= s_sv;
            for i in 2 to C_SRC_D loop s_sd(i) <= s_sd(i - 1); end loop;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [K] Camera writer: on a sample line (the last line before a row, or line
    -- 0) the luma is summed per facet column (stage 2) and each column's sum
    -- times the camera gain goes to the next row's bank; at the line end the
    -- line total gives the row's camera mean (crow, subtracted from every
    -- facet) and its fold term (rowcam RAM, used by the next field's rows).
    ---------------------------------------------------------------------------
    p_cam : process(clk)
        variable v_a : unsigned(16 downto 0);
        variable v_s : unsigned(20 downto 0);
        variable v_p : unsigned(37 downto 0);
    begin
        if rising_edge(clk) then
            cv_we <= '0'; rc_we <= '0'; km_go <= '0'; km_done <= '0'; kl_go <= '0';
            -- the column multiplier: km_a * gk over 17 steps
            if km_go = '1' then
                km_busy <= '1'; km_hi <= (others => '0'); km_n <= to_unsigned(17, 5);
                km_lo <= resize(c_gk16, 17);
            elsif km_n /= 0 then
                if km_lo(0) = '1' then v_s := km_hi + resize(km_a, 21); else v_s := km_hi; end if;
                km_hi <= '0' & v_s(20 downto 1);
                km_lo <= v_s(0) & km_lo(16 downto 1);
                km_n <= km_n - 1;
                if km_n = 1 then km_done <= '1'; km_busy <= '0'; end if;
            end if;
            v_p := km_hi(20 downto 0) & km_lo;
            if km_done = '1' then                                -- a column: value = sum * gk / 2^16
                cv_we <= '1'; cv_wa <= k_tb & km_k;
                if v_p(37 downto 24) /= 0 then cv_wd <= x"FF"; else cv_wd <= std_logic_vector(v_p(23 downto 16)); end if;
                k_xe <= km_ed;                                   -- the line's first / last column: copy it outward
            elsif k_xe /= "00" then                              -- (a sliver column entering at an edge later in the row)
                cv_we <= '1';
                if k_xe(0) = '1' then cv_wa <= k_tb & (km_k - 1); k_xe(0) <= '0';
                else cv_wa <= k_tb & (km_k + 1); k_xe(1) <= '0'; end if;
            end if;
            -- the row products on the line multiplier (idle at the line end): the row mean, then the fold term
            if k_ph = "01" and kl_go = '0' and lm_busy = '0' then
                if lm_p(37 downto 32) /= 0 then cam_crow <= x"FF"; else cam_crow <= unsigned(lm_p(31 downto 24)); end if;
                kl_op <= '1'; kl_go <= '1'; k_ph <= "10";
            elsif k_ph = "10" and kl_go = '0' and lm_busy = '0' then
                rc_we <= '1'; rc_wa <= k_tj;
                if lm_p(37 downto 32) /= 0 then rc_wd <= x"FF"; else rc_wd <= std_logic_vector(lm_p(31 downto 24)); end if;
                k_ph <= "00";
            end if;

            if g_start = '1' then
                k_acc <= (others => '0'); k_tot <= (others => '0'); k_last <= '0'; k_fst <= '1';
            elsif s_gav_sr(0) = '1' and l_smp = '1' and e_run = '1' then
                v_a := k_acc + resize(s_sd(C_SRC_D), 17);
                k_tot <= k_tot + resize(s_sd(C_SRC_D), 20);
                k_acc <= v_a;
                if q2_wrap = '1' then                            -- the last pixel of a column
                    km_a <= resize(v_a, 20); km_k <= q2_k; km_go <= '1';
                    km_ed <= (not g_avid) & k_fst;
                    k_acc <= (others => '0'); k_fst <= '0';
                end if;
                if q2_wrap = '1' or g_avid = '0' then k_tb <= l_tbank; end if;
                if g_avid = '0' then                             -- the line ends with this pixel: its column waits in k_acc;
                    k_last <= not q2_wrap; k_lk <= q2_k; k_end <= '1';   -- the targets are held for the late writes
                    k_tb <= l_tbank; k_tj <= l_tj;
                end if;
            elsif k_end = '1' then                               -- the line total is complete: start the row products
                k_end <= '0'; kl_op <= '0'; kl_go <= '1'; k_ph <= "01";
            end if;
            if k_last = '1' and km_busy = '0' and km_go = '0' and s_gav_sr(0) = '0' then
                km_a <= resize(k_acc, 20); km_k <= k_lk; km_go <= '1'; k_last <= '0'; km_ed <= '1' & k_fst;
            end if;
        end if;
    end process p_cam;

    ---------------------------------------------------------------------------
    -- [P] Pixel path
    ---------------------------------------------------------------------------
    -- 1: column accumulator (pos += 1 px, wrap at the pitch); the remaining line registers
    p_acc : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                a_pos <= nl_pos0; a_k <= nl_kl; a_wrap <= nl_wrap0;
                l_tj <= nl_tj; l_smp <= nl_smp; l_tbank <= nl_tbank;
                l_hc <= nl_hc;
                l_crow <= cam_crow;
            elsif g_avid = '1' then
                if a_wrap = '1' then
                    a_pos <= a_pos + c_256mw; a_k <= a_k + 1; a_wrap <= '0';
                else
                    a_pos <= a_pos + 256;
                    if a_pos >= c_wm512 then a_wrap <= '1'; else a_wrap <= '0'; end if;
                end if;
            end if;
        end if;
    end process p_acc;

    -- the camera window (columns k7, k7 + 1 of the stage-7 pixel, read by stage 8): shifted when a column ends at
    -- stage 7, preloaded by the line engine in blanking; its RAM read address runs at k7 + 2
    p_win : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                k7 <= nl_kl;
            elsif s_gav_sr(5) = '1' and sb_wr(7) = '1' then
                k7 <= k7 + 1;
                w_c <= w_n; w_n <= unsigned(cv_qf);
            elsif s_gav_sr(5) = '0' then
                if le_st = 16 then w_c <= unsigned(cv_qf); end if;
                if le_st = 17 then w_n <= unsigned(cv_qf); end if;
            end if;
            if le_pl = '1' then cv_ra <= cv_pa; else cv_ra <= l_bank & (k7 + 2); end if;
        end if;
    end process p_win;

    p_pix : process(clk)
        variable v_c   : signed(9 downto 0);
        variable v_zc  : signed(8 downto 0);
        variable v_m   : unsigned(4 downto 0);
        variable v_q   : unsigned(2 downto 0);
        variable v_ta, v_tb : signed(5 downto 0);
        variable v_t   : signed(10 downto 0);
        variable v_y   : signed(11 downto 0);
        variable v_u   : signed(9 downto 0);
        variable v_cy  : unsigned(9 downto 0);
        variable v_cu, v_cv : unsigned(7 downto 0);
        variable v_w   : unsigned(7 downto 0);
        variable v_dr  : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: distance to the right crease, nearer crease side
            q2_pos <= a_pos(15 downto 4); v_dr := c_wpx8 - a_pos; q2_dr <= v_dr(15 downto 4);
            if (a_pos & '0') < ('0' & c_wpx8) then q2_left <= '1'; else q2_left <= '0'; end if;
            q2_k <= a_k; q2_wrap <= a_wrap;
            -- 3: distance to the nearer crease (Q4 px); parities
            if q2_left = '1' then q3_d <= q2_pos; else q3_d <= q2_dr; end if;
            q3_cp <= q2_k(0);
            q3_kcp <= q2_k(0) xor not q2_left;
            q3_hmp <= q2_k(0) xor l_jbp;
            -- 4: distance / ramp as gated partial sums
            q4_pa <= f_gu(q3_d, l_invR(2), 15) + shift_left(f_gu(q3_d, l_invR(3), 15), 1) + shift_left(f_gu(q3_d, l_invR(4), 15), 2);
            q4_pb <= f_gu(q3_d, l_invR(5), 15) + shift_left(f_gu(q3_d, l_invR(6), 15), 1) + shift_left(f_gu(q3_d, l_invR(7), 15), 2);
            q4_pc <= f_gu(q3_d, l_invR(8), 15);
            sb_cp(4) <= q3_cp; sb_kcp(4) <= q3_kcp; sb_hmp(4) <= q3_hmp;
            for i in 5 to 8 loop sb_kcp(i) <= sb_kcp(i - 1); sb_hmp(i) <= sb_hmp(i - 1); end loop;
            for i in 5 to 8 loop sb_cp(i) <= sb_cp(i - 1); end loop;
            sb_wr(3) <= q2_wrap;
            for i in 4 to 7 loop sb_wr(i) <= sb_wr(i - 1); end loop;
            -- 5, 6: the product
            q5_s1 <= resize(q4_pa, 19) + shift_left(resize(q4_pb, 19), 3);
            q5_pc <= q4_pc;
            q6_pr <= resize(q5_s1, 21) + shift_left(resize(q5_pc, 21), 6);
            -- 7: u = distance in sixteenths of the ramp (7 bits): d (Q4 px) * (1 / ramp, Q6) / 64
            if q6_pr(20 downto 13) /= 0 then q7_u <= to_unsigned(127, 7); else q7_u <= q6_pr(12 downto 6); end if;
            -- 8: own-facet coverage (8 .. 16), zigzag crease coverage; the camera (own facet, neighbour) less the
            -- row mean, / 4
            if q7_u > 8 then q8_cov <= to_unsigned(16, 5); else q8_cov <= resize(q7_u, 5) + 8; end if;
            v_zc := signed(resize(l_hwR, 9)) + to_signed(8, 9) - signed(resize(q7_u, 9));
            if v_zc < 0 then q8_zc <= (others => '0');
            elsif v_zc > 16 then q8_zc <= to_unsigned(16, 5);
            else q8_zc <= unsigned(v_zc(4 downto 0)); end if;
            v_c := signed(resize(w_c, 10)) - signed(resize(l_crow, 10));
            if l_camen = '1' then q8_co <= v_c(9 downto 2); else q8_co <= (others => '0'); end if;
            q8_kcp <= sb_kcp(7); q8_hmp <= sb_hmp(7);
            -- 9: tones (own facet, neighbour facet); the overlay: crease (the stronger of zigzag and horizontal)
            -- or, Creases Off, the row edge toward the neighbouring row
            if sb_cp(8) = '0' then v_ta := l_Tc0; v_tb := l_Tc1; else v_ta := l_Tc1; v_tb := l_Tc0; end if;
            q9_toc <= resize(v_ta, 9) + resize(q8_co, 9);
            q9_tnc <= resize(v_tb, 9) + resize(q8_co, 9);
            if s_cre = '0' then
                q9_cc <= (others => '0'); sb_cs(9) <= '0';
            elsif q8_zc >= l_hc then
                q9_cc <= q8_zc; sb_cs(9) <= q8_kcp;
            else
                q9_cc <= l_hc; sb_cs(9) <= q8_hmp;
            end if;
            q9_cov <= q8_cov;
            for i in 10 to 15 loop sb_cs(i) <= sb_cs(i - 1); end loop;
            -- 10: tone difference
            q10_dc <= resize(q9_toc, 10) - resize(q9_tnc, 10);
            q10_tc <= q9_tnc; q10_cov <= q9_cov; q10_cc <= q9_cc;
            -- 11: difference * coverage as gated partial sums
            q11_pac <= f_gs(q10_dc, q10_cov(0), 15) + shift_left(f_gs(q10_dc, q10_cov(1), 15), 1) + shift_left(f_gs(q10_dc, q10_cov(2), 15), 2);
            q11_pbc <= shift_left(f_gs(q10_dc, q10_cov(3), 15), 3) + shift_left(f_gs(q10_dc, q10_cov(4), 15), 4);
            q11_tc <= q10_tc;
            q11_cc <= q10_cc;
            -- 12: the AA'd tone
            q12_tc <= resize(q11_tc, 11) + resize(shift_right(q11_pac + q11_pbc, 4), 11);
            sc_cc(12) <= q11_cc;
            for i in 13 to 16 loop sc_cc(i) <= sc_cc(i - 1); end loop;
            -- 13: clamp to the table (-32 .. 31): address (the line engine borrows the port in blanking)
            v_t := q12_tc;
            if v_t < -32 then v_t := to_signed(-32, 11); elsif v_t > 31 then v_t := to_signed(31, 11); end if;
            if le_ft = '1' then ft1_a <= le_fa; else ft1_a <= l_pc & unsigned(v_t(5 downto 0)); end if;
            -- 14: tables; 15: fabric copy
            q15_f1 <= ft1_q;
            -- 16: the crease colour (mountain = accent, valley = this paper at -32) less the facet colour
            if sb_cs(15) = '0' then v_cy := c_acy; v_cu := c_acu; v_cv := c_acv;
            else v_cy := unsigned(l_cv(25 downto 16)); v_cu := unsigned(l_cv(15 downto 8)); v_cv := unsigned(l_cv(7 downto 0)); end if;
            q16_dy <= signed(resize(v_cy, 11)) - signed(resize(unsigned(q15_f1(25 downto 16)), 11));
            q16_du <= signed(resize(v_cu, 9)) - signed(resize(unsigned(q15_f1(15 downto 8)), 9));
            q16_dv <= signed(resize(v_cv, 9)) - signed(resize(unsigned(q15_f1(7 downto 0)), 9));
            q16_y <= unsigned(q15_f1(25 downto 16)); q16_u <= unsigned(q15_f1(15 downto 8)); q16_v <= unsigned(q15_f1(7 downto 0));
            -- 17: * the overlay weight: luma in sixteenths, chroma in quarters
            v_m := sc_cc(16); v_q := v_m(4 downto 2);
            q17_py0 <= f_gs(q16_dy, v_m(0), 16) + shift_left(f_gs(q16_dy, v_m(1), 16), 1) + shift_left(f_gs(q16_dy, v_m(2), 16), 2);
            q17_py1 <= shift_left(f_gs(q16_dy, v_m(3), 16), 3) + shift_left(f_gs(q16_dy, v_m(4), 16), 4);
            q17_pu <= f_gs(q16_du, v_q(0), 12) + shift_left(f_gs(q16_du, v_q(1), 12), 1) + shift_left(f_gs(q16_du, v_q(2), 12), 2);
            q17_pv <= f_gs(q16_dv, v_q(0), 12) + shift_left(f_gs(q16_dv, v_q(1), 12), 1) + shift_left(f_gs(q16_dv, v_q(2), 12), 2);
            q17_y <= q16_y; q17_u <= q16_u; q17_v <= q16_v;
            -- 18: the pixel (a blend of two in-range colours needs no clamp); paper A outside the generated avid
            v_y := signed(resize(q17_y, 12)) + resize(shift_right(q17_py0 + q17_py1, 4), 12);
            if s_gav_sr(15) = '0' then q18_y <= c_pay; else q18_y <= unsigned(v_y(9 downto 0)); end if;
            v_u := signed(resize(q17_u, 10)) + resize(shift_right(q17_pu, 2), 10);
            if s_gav_sr(15) = '0' then q18_c(1) <= c_pau; else q18_c(1) <= unsigned(v_u(7 downto 0)); end if;
            v_u := signed(resize(q17_v, 10)) + resize(shift_right(q17_pv, 2), 10);
            if s_gav_sr(15) = '0' then q18_c(2) <= c_pav; else q18_c(2) <= unsigned(v_u(7 downto 0)); end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(15);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [F] Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent
    -- 8-sums of the 8-bit chroma, resynced every line: 10 bits out), luma [1 2 1]/4 delayed to the chroma's
    -- group delay, luma 13/16, blanking gate.  The filters add 4 pixels of
    -- delay: the output register is effective stage 26 = C_LAT + 1.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 19: running 8-sums (the sample of 8 clocks before comes back from its delay ring); they snap
            -- back to 8 x paper A once 8 blank samples are in, so a glitched sample can't offset the colours for good
            if s_bah = "00000000" then
                s_bu <= c_pau & "000"; s_bv <= c_pav & "000";
            else
                s_bu <= s_bu + q18_c(1) - unsigned(cr_f(15 downto 8));
                s_bv <= s_bv + q18_c(2) - unsigned(cr_f(7 downto 0));
            end if;
            s_bud <= s_bu; s_bvd <= s_bv;
            s_ysr(0) <= q18_y;
            for i in 1 to 5 loop s_ysr(i) <= s_ysr(i - 1); end loop;
            -- 20: filtered
            v_sc := resize(s_bu, 12) + s_bud; s_u31 <= v_sc(11 downto 2);
            v_sc := resize(s_bv, 12) + s_bvd; s_v31 <= v_sc(11 downto 2);
            v_y := resize(s_ysr(3), 12) + resize(s_ysr(4) & '0', 12) + resize(s_ysr(5), 12);
            s_y31 <= v_y(11 downto 2);
            -- 21: luma 13/16 toward black
            if s_y31 > 64 then v_dl := s_y31 - 64; else v_dl := (others => '0'); end if;
            s_y32 <= s_y31 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u32 <= s_u31; s_v32 <= s_v31;
            -- 22: blanking gate
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

end architecture miura;
