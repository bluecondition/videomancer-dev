-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Triplicate v0.5.0 -- Heirloom #53 (2026-10-21), rebuilt from selfie.jpg (a paper collage).
--
-- The collage: a bear with a skull for a face and a TI home computer on its lap, photocopied in
-- red, green and blue ink at three enlargements and cascaded diagonally down to the right over a
-- grey wall of tiled skull photographs, the full-colour original (smallest) in front.
--
-- The program: the camera picture (or, with Source = Osc, a generated skull-faced bear with a
-- computer) is written into a 128 x 128 x 4-bit store (box-filtered; 32 x 32 chroma).  Four
-- cut-outs in the collage's measured bear outline (a 128-row interval ROM, interpolated per
-- line) cascade over the frame at the collage's measured sizes and positions; each shows the
-- store bilinear-magnified, the back three printed as one-colour photocopies (ink over tinted
-- paper by a photocopy density curve, with toner grain), the front one in colour.  Behind them
-- a wall of square greyscale prints of the store, each a different window (head-biased), with
-- per-print exposure, thin paper seams and per-row misalignment.
--
--   K1  Recede   size step between the copies (equal .. the collage .. strong)
--   K2  Size     figure size
--   K3  Ink      photocopy contrast (faint prints .. the collage .. solid silhouettes)
--   K4  Tiles    print size on the wall
--   K5  Wall     wall exposure
--   K6  Speed    bipolar: the cascade breathes and the wall scrolls (centre = still)
--   S7  Order    RGB / BGR (which ink is at the back)
--   S8  Inks     RGB / CMY
--   S9  Front    Colour / Ink (a fourth, black plate)
--   S10 Flip     Off / On (the 2nd and 4th copies mirrored)
--   S11 Source   Video / Osc
--   P12 CASCADE  0 = the copies nested on one spot, 50 % = the collage, 100 % = fanned across
--
-- Geometry in virtual 1080-line units (timing-ID steps).  Engines: a per-field register machine
-- (register file in one EBR; one shift-right multiplier, one restoring divider, one post-adder)
-- that also runs the knob glides; a per-line engine in hblank that steps the copies' rows, reads
-- and interpolates the outline ROM, loads the copies' column DDAs and steps the wall rows.
-- Generated avid; output conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma to
-- ~777) before the blanking gate.  Latency 21, all modes.  Colours: hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.triplicate_pkg.all;
use work.all;

architecture triplicate of program_top is
    constant C_LAT : integer := 21;

    ---------------------------------------------------------------------------
    -- mode tables (timing ID)
    ---------------------------------------------------------------------------
    function f_xs(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    function f_ys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 11);
            when 4          => return to_unsigned(576, 11);
            when 8          => return to_unsigned(960, 11);
            when 12         => return to_unsigned(480, 11);
            when 5 | 6 | 14 => return to_unsigned(384, 11);
            when 1 | 2 | 10 => return to_unsigned(512, 11);
            when others     => return to_unsigned(256, 11);
        end case;
    end function;
    function f_kr(t : std_logic_vector(3 downto 0)) return unsigned is   -- store rows per line, Q.14
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(8632, 14);
            when 4          => return to_unsigned(4369, 14);
            when 8          => return to_unsigned(7282, 14);
            when 12         => return to_unsigned(3641, 14);
            when 5 | 6 | 14 => return to_unsigned(2913, 14);
            when 1 | 2 | 10 => return to_unsigned(3884, 14);
            when others     => return to_unsigned(1942, 14);
        end case;
    end function;
    function f_nb(t : std_logic_vector(3 downto 0)) return unsigned is   -- log2 lines per store row
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(0, 2);
            when 4 | 8      => return to_unsigned(1, 2);
            when 12 | 5 | 6 | 14 | 1 | 2 | 10 => return to_unsigned(2, 2);
            when others     => return to_unsigned(3, 2);
        end case;
    end function;

    ---------------------------------------------------------------------------
    -- register machine.  res = (+/-)((A * B) >> sh) + K  (post: clamp / min(.,K) / max(.,K)),
    -- or 2^n / A; written to the register file and/or an output register.
    ---------------------------------------------------------------------------
    -- constants
    constant RF_ZERO : integer := 0;   constant RF_ONE : integer := 1;   constant RF_TWO : integer := 2;
    constant RF_3482 : integer := 3;   constant RF_205 : integer := 4;   constant RF_13115 : integer := 5;
    constant RF_230 : integer := 6;    constant RF_220 : integer := 7;   constant RF_4496 : integer := 8;
    constant RF_FIVE : integer := 9;   constant RF_256 : integer := 10;  constant RF_410 : integer := 11;
    constant RF_115 : integer := 12;   constant RF_688 : integer := 13;  constant RF_BM : integer := 14;
    constant RF_307 : integer := 15;   constant RF_M717 : integer := 16; constant RF_614 : integer := 17;
    constant RF_717 : integer := 18;   constant RF_512 : integer := 19;
    constant RF_DH : integer := 20;    constant RF_DB : integer := 24;   constant RF_DX : integer := 28;
    constant RF_M290 : integer := 32;
    -- live inputs (loaded each field)
    constant RF_T12 : integer := 40;   constant RF_T1 : integer := 41;   constant RF_T2 : integer := 42;
    constant RF_T4 : integer := 43;    constant RF_K3 : integer := 44;   constant RF_K5 : integer := 45;
    constant RF_W : integer := 46;     constant RF_TRI : integer := 47;  constant RF_SC : integer := 48;
    -- glide state (knob * 4, persists)
    constant RF_G12 : integer := 50;   constant RF_G1 : integer := 51;   constant RF_G2 : integer := 52;
    constant RF_G4 : integer := 53;
    -- scratch
    constant RF_D : integer := 56;     constant RF_CLO : integer := 57;  constant RF_C : integer := 58;
    constant RF_CV : integer := 59;    constant RF_CP : integer := 60;   constant RF_SLO : integer := 61;
    constant RF_S : integer := 62;     constant RF_RLO : integer := 63;  constant RF_R : integer := 64;
    constant RF_SH : integer := 65;    constant RF_SR : integer := 66;   constant RF_TQ : integer := 67;
    constant RF_T : integer := 68;     constant RF_KT : integer := 69;   constant RF_KT5 : integer := 70;
    constant RF_X0G : integer := 71;   constant RF_WV16 : integer := 72; constant RF_H : integer := 73;
    constant RF_G : integer := 74;     constant RF_TOP : integer := 75;  constant RF_LEFT : integer := 76;
    constant RF_KUC : integer := 77;   constant RF_LR : integer := 78;
    -- line-engine inputs (per copy at +ci; wide values: high word at +4)
    constant RF_KV : integer := 80;    constant RF_U0 : integer := 84;   constant RF_V0 : integer := 92;
    constant RF_KTV : integer := 100;  constant RF_G0 : integer := 101;  constant RF_VT0 : integer := 102;
    -- per-mode tables (indexed by the timing ID)
    constant RF_YS : integer := 128;   constant RF_YS5 : integer := 144; constant RF_KX : integer := 160;
    constant RF_RX : integer := 176;   constant RF_XS512 : integer := 192; constant RF_XS : integer := 208;
    constant RF_XS4 : integer := 224;
    type t_rf is array (0 to 255) of std_logic_vector(15 downto 0);
    function f_rf_init return t_rf is
        variable r : t_rf := (others => (others => '0'));
        variable xs, ys : integer;
        procedure set(a : integer; v : integer) is
        begin
            r(a) := std_logic_vector(to_signed(v, 16));
        end procedure;
    begin
        set(RF_ONE, 1); set(RF_TWO, 2); set(RF_3482, 3482); set(RF_205, 205); set(RF_13115, 13115);
        set(RF_230, 230); set(RF_220, 220); set(RF_4496, 4496); set(RF_FIVE, 5); set(RF_256, 256);
        set(RF_410, 410); set(RF_115, 115); set(RF_688, 688); set(RF_BM, 15686); set(RF_307, 307);
        set(RF_M717, -717); set(RF_614, 614); set(RF_717, 717); set(RF_512, 512); set(RF_M290, -290);
        set(RF_DH, 0); set(RF_DH + 1, 1936); set(RF_DH + 2, 2972); set(RF_DH + 3, 6445);       -- size drop
        set(RF_DB, -1585); set(RF_DB + 1, -480); set(RF_DB + 2, 1007); set(RF_DB + 3, 1058);  -- bottoms
        set(RF_DX, -2890); set(RF_DX + 1, -989); set(RF_DX + 2, 1102); set(RF_DX + 3, 2778);  -- ear-centre x
        set(RF_G12, 2048); set(RF_G1, 2048); set(RF_G2, 2048); set(RF_G4, 1868);
        for t in 0 to 15 loop
            xs := to_integer(f_xs(std_logic_vector(to_unsigned(t, 4))));
            ys := to_integer(f_ys(std_logic_vector(to_unsigned(t, 4))));
            set(RF_YS + t, ys); set(RF_YS5 + t, 5 * ys); set(RF_KX + t, 205 * xs); set(RF_RX + t, 8192 / xs);
            set(RF_XS512 + t, 512 * xs); set(RF_XS + t, xs); set(RF_XS4 + t, 4 * xs);
        end loop;
        return r;
    end function;
    -- live sources
    constant L_T12 : integer := 0; constant L_T1 : integer := 1; constant L_T2 : integer := 2;
    constant L_T4 : integer := 3;  constant L_K3 : integer := 4; constant L_K5 : integer := 5;
    constant L_W : integer := 6;   constant L_TRI : integer := 7; constant L_SC : integer := 8;
    -- output registers
    constant O_NONE : integer := 0; constant O_HI : integer := 1; constant O_LO : integer := 2;
    constant O_KTR : integer := 3;  constant O_KU : integer := 4; constant O_T : integer := 5;
    -- address modifiers
    constant M_NO : integer := 0; constant M_CI : integer := 1; constant M_TID : integer := 2;
    type t_uc is record
        op : integer range 0 to 2;          -- 0 multiply, 1 divide (2^b / A), 2 load live(a)
        a, b, k, d : integer range 0 to 255;
        am, bm : integer range 0 to 2;
        dci, wide : std_logic;              -- d + copy index; also write the high word at d + 4
        sh : integer range 0 to 15;
        ng : std_logic;
        post : integer range 0 to 4;        -- 1 clamp [0, 1024], 2 clamp >= 0, 3 min(., K), 4 max(., K)
        o : integer range 0 to 5;
    end record;
    function mu(a, b, sh : integer; ng : std_logic; k, d : integer; o : integer := 0; am, bm : integer := 0;
                post : integer := 0; dci, wide : std_logic := '0') return t_uc is
        variable r : t_uc;
    begin
        r.op := 0; r.a := a; r.b := b; r.k := k; r.d := d; r.am := am; r.bm := bm; r.dci := dci;
        r.wide := wide; r.sh := sh; r.ng := ng; r.post := post; r.o := o;
        return r;
    end function;
    function dv(a, n, d : integer) return t_uc is
        variable r : t_uc;
    begin
        r := mu(a, n, 0, '0', RF_ZERO, d); r.op := 1;
        return r;
    end function;
    function ld(l, d : integer) return t_uc is
        variable r : t_uc;
    begin
        r := mu(l, 0, 0, '0', RF_ZERO, d); r.op := 2;
        return r;
    end function;
    constant ST_COPY : integer := 43;      -- first per-copy step
    constant ST_LAST : integer := 53;
    function f_uc(st : unsigned(5 downto 0)) return t_uc is
    begin
        case to_integer(st) is
            when 0  => return ld(L_T12, RF_T12);   when 1  => return ld(L_T1, RF_T1);
            when 2  => return ld(L_T2, RF_T2);     when 3  => return ld(L_T4, RF_T4);
            when 4  => return ld(L_K3, RF_K3);     when 5  => return ld(L_K5, RF_K5);
            when 6  => return ld(L_W, RF_W);       when 7  => return ld(L_TRI, RF_TRI);
            when 8  => return ld(L_SC, RF_SC);
            -- glides: G += (T - G) / 8
            when 9  => return mu(RF_G12, RF_ONE, 0, '1', RF_T12, RF_D);
            when 10 => return mu(RF_D, RF_ONE, 3, '0', RF_G12, RF_G12);
            when 11 => return mu(RF_G1, RF_ONE, 0, '1', RF_T1, RF_D);
            when 12 => return mu(RF_D, RF_ONE, 3, '0', RF_G1, RF_G1);
            when 13 => return mu(RF_G2, RF_ONE, 0, '1', RF_T2, RF_D);
            when 14 => return mu(RF_D, RF_ONE, 3, '0', RF_G2, RF_G2);
            when 15 => return mu(RF_G4, RF_ONE, 0, '1', RF_T4, RF_D);
            when 16 => return mu(RF_D, RF_ONE, 3, '0', RF_G4, RF_G4);
            -- mode: X0G = W xs / 4 - 290 (print grid origin), WV16 = W xs * 4 (Q.4 half width)
            when 17 => return mu(RF_W, RF_XS, 2, '0', RF_M290, RF_X0G, bm => M_TID);
            when 18 => return mu(RF_W, RF_XS4, 0, '0', RF_ZERO, RF_WV16, bm => M_TID);
            -- cascade c (Q.10) = max(2 g, 3.4 g - 717); breathing; clamps
            when 19 => return mu(RF_G12, RF_ONE, 1, '0', RF_ZERO, RF_CLO);
            when 20 => return mu(RF_G12, RF_3482, 12, '0', RF_M717, RF_C);
            when 21 => return mu(RF_C, RF_ONE, 0, '0', RF_CLO, RF_C, post => 4);
            when 22 => return mu(RF_TRI, RF_205, 15, '0', RF_C, RF_C);
            when 23 => return mu(RF_C, RF_ONE, 0, '0', RF_ZERO, RF_CV, post => 1);
            when 24 => return mu(RF_C, RF_ONE, 0, '0', RF_ZERO, RF_CP, post => 2);
            -- size S (Q.10) = min(614 + 0.8 k2, 717 + 0.6 k2); recession R = min(2 k1, k1 + 512)
            when 25 => return mu(RF_G2, RF_410, 11, '0', RF_614, RF_SLO);
            when 26 => return mu(RF_G2, RF_307, 11, '0', RF_717, RF_S);
            when 27 => return mu(RF_S, RF_ONE, 0, '0', RF_SLO, RF_S, post => 3);
            when 28 => return mu(RF_G1, RF_ONE, 1, '0', RF_ZERO, RF_RLO);
            when 29 => return mu(RF_G1, RF_ONE, 2, '0', RF_512, RF_R);
            when 30 => return mu(RF_R, RF_ONE, 0, '0', RF_RLO, RF_R, post => 3);
            when 31 => return mu(RF_S, RF_13115, 10, '0', RF_ZERO, RF_SH);
            when 32 => return mu(RF_S, RF_R, 10, '0', RF_ZERO, RF_SR);
            -- ink threshold, wall offset
            when 33 => return mu(RF_K3, RF_230, 10, '0', RF_115, 0, O_HI);
            when 34 => return mu(RF_K5, RF_220, 10, '0', RF_ZERO, 0, O_LO);
            -- prints: T = 43 + 281 k^2 lines (Q.4), KT = 64 cols / T, row/column steps, origin
            when 35 => return mu(RF_G4, RF_G4, 14, '0', RF_ZERO, RF_TQ);
            when 36 => return mu(RF_TQ, RF_4496, 10, '0', RF_688, RF_T, O_T);
            when 37 => return dv(RF_T, 22, RF_KT);
            when 38 => return mu(RF_KT, RF_YS5, 11, '0', RF_ZERO, RF_KTV, bm => M_TID);
            when 39 => return mu(RF_KT, RF_XS512, 10, '0', RF_ZERO, 0, O_KTR, bm => M_TID);
            when 40 => return mu(RF_KT, RF_FIVE, 0, '0', RF_ZERO, RF_KT5);
            when 41 => return mu(RF_X0G, RF_KT, 2, '1', RF_ZERO, RF_G0);
            when 42 => return mu(RF_SC, RF_KT5, 9, '0', RF_ZERO, RF_VT0);
            -- per copy
            when 43 => return mu(RF_DH, RF_SR, 10, '1', RF_SH, RF_H, am => M_CI);
            when 44 => return dv(RF_H, 26, RF_G);
            when 45 => return mu(RF_G, RF_YS, 10, '0', RF_ZERO, RF_KV, bm => M_TID, dci => '1');
            when 46 => return mu(RF_G, RF_KX, 11, '0', RF_ZERO, RF_KUC, O_KU, bm => M_TID);
            when 47 => return mu(RF_DB, RF_CV, 10, '0', RF_BM, RF_TOP, am => M_CI);
            when 48 => return mu(RF_H, RF_ONE, 0, '1', RF_TOP, RF_TOP);
            when 49 => return mu(RF_TOP, RF_G, 5, '1', RF_ZERO, RF_V0, dci => '1', wide => '1');
            when 50 => return mu(RF_H, RF_256, 10, '1', RF_WV16, RF_LEFT);
            when 51 => return mu(RF_DX, RF_CP, 10, '0', RF_LEFT, RF_LEFT, am => M_CI);
            when 52 => return mu(RF_LEFT, RF_RX, 12, '0', RF_ZERO, RF_LR, bm => M_TID);
            when others => return mu(RF_LR, RF_KUC, 4, '1', RF_ZERO, RF_U0, dci => '1', wide => '1');
        end case;
    end function;
    function f_am(base : integer; m : integer; ci : unsigned(1 downto 0); tid : std_logic_vector(3 downto 0)) return unsigned is
        variable v : unsigned(7 downto 0);
    begin
        v := to_unsigned(base, 8);
        case m is
            when M_CI  => v(1 downto 0) := ci;
            when M_TID => v(3 downto 0) := unsigned(tid);
            when others => null;
        end case;
        return v;
    end function;

    -- one-stage mixer for the print hash (Weyl sequence in, xorshift-add out)
    function f_mix1(w : unsigned(15 downto 0)) return unsigned is
        variable a : unsigned(15 downto 0);
    begin
        a := w xor shift_right(w, 7);
        return a + shift_left(a, 3);
    end function;
    constant C_KT  : unsigned(15 downto 0) := x"9E37";   -- per-print step
    constant C_KR  : unsigned(15 downto 0) := x"6A09";   -- per-row step

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 64 then return to_unsigned(64, 10);
        elsif v > 940 then return to_unsigned(940, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;

    ---------------------------------------------------------------------------
    -- memories
    ---------------------------------------------------------------------------
    type t_bank is array (0 to 4095) of std_logic_vector(3 downto 0);
    signal bk0, bk1, bk2, bk3 : t_bank := (others => (others => '0'));   -- bank = row(0) & col(0)
    attribute no_rw_check : boolean;
    attribute no_rw_check of bk0 : signal is true;
    attribute no_rw_check of bk1 : signal is true;
    attribute no_rw_check of bk2 : signal is true;
    attribute no_rw_check of bk3 : signal is true;
    type t_crom is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal cst : t_crom := (others => (others => '0'));                  -- chroma U4 & V4 (signed)
    attribute no_rw_check of cst : signal is true;
    type t_acc is array (0 to 127) of std_logic_vector(15 downto 0);
    signal accr : t_acc := (others => (others => '0'));
    attribute no_rw_check of accr : signal is true;
    signal rf : t_rf := f_rf_init;
    attribute no_rw_check of rf : signal is true;

    signal b_we : std_logic_vector(3 downto 0) := (others => '0');
    signal b_wa : unsigned(11 downto 0) := (others => '0');
    signal b_wd : std_logic_vector(3 downto 0) := (others => '0');
    signal b_ra0, b_ra1, b_ra2, b_ra3 : unsigned(11 downto 0) := (others => '0');
    signal b_q0, b_q1, b_q2, b_q3 : std_logic_vector(3 downto 0) := (others => '0');
    signal c_we : std_logic := '0';
    signal c_wa, c_ra : unsigned(9 downto 0) := (others => '0');
    signal c_wd, c_q : std_logic_vector(7 downto 0) := (others => '0');
    signal a_we : std_logic := '0';
    signal a_wa, a_ra : unsigned(6 downto 0) := (others => '0');
    signal a_wd, a_q : std_logic_vector(15 downto 0) := (others => '0');
    signal o_ra : unsigned(7 downto 0) := (others => '0');
    signal o_q : std_logic_vector(15 downto 0) := (others => '0');
    signal rf_ra, q_ra, l_ra, rf_wa : unsigned(7 downto 0) := (others => '0');
    signal rf_we : std_logic := '0';
    signal rf_wd, rf_q, rf_qq : std_logic_vector(15 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- timing, controls
    ---------------------------------------------------------------------------
    signal s_tid : std_logic_vector(3 downto 0) := (others => '0');
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_kr : unsigned(13 downto 0) := to_unsigned(1942, 14);
    signal s_nb : unsigned(1 downto 0) := to_unsigned(3, 2);
    signal s_kw : unsigned(10 downto 0) := to_unsigned(777, 11);
    signal s_xco : unsigned(8 downto 0) := to_unsigned(338, 9);
    signal s_na : unsigned(1 downto 0) := to_unsigned(2, 2);
    signal s_idle : unsigned(10 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w : unsigned(10 downto 0) := to_unsigned(1920, 11);
    signal s_xc, s_xca : unsigned(10 downto 0) := to_unsigned(622, 11);
    signal s_spr, s_sp : signed(9 downto 0) := (others => '0');
    signal s_order, s_cmy, s_fink, s_flip, s_osc : std_logic := '0';

    ---------------------------------------------------------------------------
    -- register machine
    ---------------------------------------------------------------------------
    signal q_run, q_done : std_logic := '0';
    signal q_st : unsigned(5 downto 0) := (others => '0');
    signal q_ci : unsigned(1 downto 0) := (others => '0');
    signal q_cnt : unsigned(5 downto 0) := (others => '0');
    signal q_uc : t_uc := ld(0, 0);
    signal m_a, m_k : signed(15 downto 0) := (others => '0');
    signal m_p : unsigned(36 downto 0) := (others => '0');          -- multiply {acc, multiplier}
    signal m_rem : unsigned(15 downto 0) := (others => '0');        -- divide
    signal m_dq : unsigned(26 downto 0) := (others => '0');
    signal m_ps, m_res, m_sum : signed(27 downto 0) := (others => '0');
    signal m_lt, m_neg, m_big : std_logic := '0';
    signal m_wr, m_wr2 : std_logic := '0';
    signal r_hi : unsigned(8 downto 0) := to_unsigned(230, 9);
    signal r_lo : unsigned(9 downto 0) := to_unsigned(110, 10);
    signal r_t : unsigned(12 downto 0) := to_unsigned(1618, 13);
    signal r_ktr : unsigned(13 downto 0) := to_unsigned(2592, 14);
    signal r_tri : signed(15 downto 0) := (others => '0');
    signal r_sc : signed(15 downto 0) := (others => '0');          -- wall scroll, Q.4 lines
    signal r_wb, r_sp : unsigned(15 downto 0) := (others => '0');
    signal r_ls : unsigned(12 downto 0) := (others => '0');
    type t_u14x4 is array (0 to 3) of unsigned(13 downto 0);
    signal p_ku : t_u14x4 := (others => (others => '0'));

    ---------------------------------------------------------------------------
    -- line engine (display)
    ---------------------------------------------------------------------------
    signal l_run, l_init, l_fkick, l_kickp : std_logic := '0';
    signal l_dly : unsigned(3 downto 0) := (others => '0');
    signal l_cnt : unsigned(5 downto 0) := (others => '0');
    type t_s27x4 is array (0 to 3) of signed(26 downto 0);
    signal l_v : t_s27x4 := (others => (others => '0'));
    signal l_vadd : std_logic_vector(3 downto 0) := (others => '0');
    type t_u7x4 is array (0 to 3) of unsigned(6 downto 0);
    type t_u4x4 is array (0 to 3) of unsigned(3 downto 0);
    signal l_vr : t_u7x4 := (others => (others => '0'));
    signal l_vf : t_u4x4 := (others => (others => '0'));
    signal l_vin : std_logic_vector(3 downto 0) := (others => '0');
    type t_u10x16 is array (0 to 15) of unsigned(9 downto 0);
    signal l_e : t_u10x16 := (others => (others => '0'));
    signal l_lo : std_logic_vector(15 downto 0) := (others => '0');
    signal l_u0 : signed(25 downto 0) := (others => '0');
    signal l_ld : std_logic_vector(3 downto 0) := (others => '0');
    signal l_j : unsigned(3 downto 0) := (others => '0');
    signal l_jv : std_logic := '0';
    signal l_tg1, l_tg2, l_tg3 : unsigned(3 downto 0) := (others => '0');
    signal l_tv1, l_tv2, l_tv3 : std_logic := '0';
    signal l_e0, l_oq : unsigned(15 downto 0) := (others => '0');
    signal l_d0, l_d1 : signed(8 downto 0) := (others => '0');
    signal l_b0, l_b1, l_bb0, l_bb1 : unsigned(7 downto 0) := (others => '0');
    signal l_p0, l_p1 : signed(13 downto 0) := (others => '0');
    signal l_wv2 : std_logic := '0';
    signal l_wp2 : unsigned(2 downto 0) := (others => '0');
    signal l_f : unsigned(3 downto 0) := (others => '0');
    signal l_wp : unsigned(2 downto 0) := (others => '0');
    signal l_wv : std_logic := '0';
    signal l_vt : unsigned(17 downto 0) := (others => '0');
    signal l_wrow : unsigned(15 downto 0) := (others => '0');
    signal l_ut0 : unsigned(17 downto 0) := (others => '0');
    signal l_hl : unsigned(15 downto 0) := (others => '0');
    signal l_seamt : std_logic := '0';

    ---------------------------------------------------------------------------
    -- generated avid
    ---------------------------------------------------------------------------
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_avd : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    ---------------------------------------------------------------------------
    -- pixel path
    ---------------------------------------------------------------------------
    type t_s26x4 is array (0 to 3) of signed(25 downto 0);
    signal p_u : t_s26x4 := (others => (others => '0'));
    signal p_ut : unsigned(17 downto 0) := (others => '0');
    signal p_tw, p_hc, p_hn : unsigned(15 downto 0) := (others => '0');
    signal p_cmp : std_logic_vector(15 downto 0) := (others => '0');
    signal p_ok2 : std_logic_vector(3 downto 0) := (others => '0');
    type t_u11x4 is array (0 to 3) of unsigned(10 downto 0);
    signal p_cf2 : t_u11x4 := (others => (others => '0'));
    signal p_tc2, p_tr2 : unsigned(6 downto 0) := (others => '0');
    signal p_tfx2, p_tfy2 : unsigned(3 downto 0) := (others => '0');
    signal p_tsm2 : std_logic := '0';
    signal p_tg2 : unsigned(1 downto 0) := (others => '0');
    signal p_ly3 : unsigned(2 downto 0) := (others => '0');
    signal p_c03, p_r03 : unsigned(6 downto 0) := (others => '0');
    signal p_fx3, p_fy3 : unsigned(3 downto 0) := (others => '0');
    signal p_tsm3 : std_logic := '0';
    signal p_tg3 : unsigned(1 downto 0) := (others => '0');
    type t_side is record
        ly : unsigned(2 downto 0);
        cp, rp : std_logic;
        fx, fy : unsigned(3 downto 0);
        tg : unsigned(1 downto 0);
        ts : std_logic;
    end record;
    constant C_SIDE0 : t_side := ("100", '0', '0', "0000", "0000", "00", '0');
    type t_sidev is array (4 to 13) of t_side;
    signal p_sd : t_sidev := (others => C_SIDE0);
    signal p_q0, p_q1, p_q2, p_q3 : unsigned(3 downto 0) := (others => '0');
    type t_cq is array (6 to 13) of std_logic_vector(7 downto 0);
    signal p_cq : t_cq := (others => (others => '0'));
    signal p_tl : unsigned(3 downto 0) := (others => '0');
    signal p_dt, p_dtl, p_ddt : signed(5 downto 0) := (others => '0');
    signal p_top8, p_dv8 : signed(10 downto 0) := (others => '0');
    signal p_l9 : unsigned(7 downto 0) := (others => '0');
    signal p_dd10 : unsigned(7 downto 0) := (others => '0');
    signal p_ix10 : unsigned(2 downto 0) := "111";
    signal p_l10 : unsigned(7 downto 0) := (others => '0');
    signal p_pa11 : unsigned(7 downto 0) := (others => '0');
    signal p_ph12, p_pl12, p_ph13, p_pl13 : std_logic_vector(15 downto 0) := (others => '0');
    signal p_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal p_wo13, p_yw14 : signed(11 downto 0) := (others => '0');
    signal p_yr14 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p_wl14, p_ts14 : std_logic := '0';
    signal p_u14, p_v14, p_u15, p_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p_y15 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- conditioning
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    signal s_csu, s_csv : t_u8(0 to 8) := (others => to_unsigned(128, 8));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_y16, s_y17, s_y18, s_y19, s_y20, s_y21x, s_yf21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21x, s_v21x : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y22 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- sync / avid delay line: {hsync_n, vsync_n, field_n, generated avid} through one EBR
    type t_dly is array (0 to 255) of std_logic_vector(3 downto 0);
    signal s_dly : t_dly := (others => "1110");
    attribute no_rw_check of s_dly : signal is true;
    signal s_dwp, s_drp : unsigned(7 downto 0) := (others => '0');
    signal s_dq, s_dg, s_do : std_logic_vector(3 downto 0) := "1110";

    ---------------------------------------------------------------------------
    -- write path (source side)
    ---------------------------------------------------------------------------
    signal w_sx : unsigned(11 downto 0) := (others => '0');
    signal w_wu, w_wua : unsigned(19 downto 0) := (others => '0');
    signal w_win, w_nw, w_smp, w_winp : std_logic := '0';
    signal w_j : unsigned(2 downto 0) := (others => '0');
    signal w_y1 : unsigned(7 downto 0) := (others => '0');
    signal w_u1, w_v1 : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal w_wv, w_wvn : unsigned(21 downto 0) := (others => '0');
    signal w_row, w_rown : unsigned(6 downto 0) := (others => '0');
    signal w_lrkn : unsigned(3 downto 0) := (others => '0');
    signal w_rok, w_lsmp, w_lfst, w_llst, w_lchr : std_logic := '0';
    signal w_hs : unsigned(12 downto 0) := (others => '0');
    signal w_dn, w_sv7 : std_logic := '0';
    signal w_dc, w_colq : unsigned(6 downto 0) := (others => '0');
    signal w_dhs : unsigned(12 downto 0) := (others => '0');
    signal w_aq : unsigned(15 downto 0) := (others => '0');
    signal w_sum : unsigned(15 downto 0) := (others => '0');
    signal w_sc7, w_sc8 : unsigned(6 downto 0) := (others => '0');
    signal w_st8 : std_logic := '0';
    signal w_avg8 : unsigned(8 downto 0) := (others => '0');
    -- Osc generator
    signal o_row : std_logic_vector(21 downto 0) := (others => '0');
    signal o_t1 : unsigned(19 downto 0) := (others => '0');
    signal o_lg, o_hbq, o_hbq3 : std_logic := '0';
    signal o_adu : unsigned(6 downto 0) := (others => '0');
    signal o_cf1, o_cf2, o_cf3 : std_logic_vector(10 downto 0) := (others => '0');
    signal o_d50, o_d39, o_d62, o_adu0 : unsigned(6 downto 0) := (others => '0');
    signal o_hbq0 : std_logic := '0';
    signal o_jw2, o_ey2 : std_logic := '0';
    signal o_sq : unsigned(13 downto 0) := (others => '0');
    signal o_jw3, o_mk3 : std_logic := '0';
    signal o_r1 : unsigned(20 downto 0) := (others => '0');
    signal o_sk4, o_mk4 : std_logic := '0';
    signal o_fl4 : std_logic_vector(4 downto 0) := (others => '0');
    signal o_gy : unsigned(9 downto 0) := (others => '0');
    signal o_gu, o_gv : unsigned(9 downto 0) := to_unsigned(512, 10);

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_bank0 : process(clk)
    begin
        if rising_edge(clk) then
            b_q0 <= bk0(to_integer(b_ra0));
            if b_we(0) = '1' then bk0(to_integer(b_wa)) <= b_wd; end if;
        end if;
    end process;
    p_bank1 : process(clk)
    begin
        if rising_edge(clk) then
            b_q1 <= bk1(to_integer(b_ra1));
            if b_we(1) = '1' then bk1(to_integer(b_wa)) <= b_wd; end if;
        end if;
    end process;
    p_bank2 : process(clk)
    begin
        if rising_edge(clk) then
            b_q2 <= bk2(to_integer(b_ra2));
            if b_we(2) = '1' then bk2(to_integer(b_wa)) <= b_wd; end if;
        end if;
    end process;
    p_bank3 : process(clk)
    begin
        if rising_edge(clk) then
            b_q3 <= bk3(to_integer(b_ra3));
            if b_we(3) = '1' then bk3(to_integer(b_wa)) <= b_wd; end if;
        end if;
    end process;
    p_cst : process(clk)
    begin
        if rising_edge(clk) then
            c_q <= cst(to_integer(c_ra));
            if c_we = '1' then cst(to_integer(c_wa)) <= c_wd; end if;
        end if;
    end process;
    p_accr : process(clk)
    begin
        if rising_edge(clk) then
            a_q <= accr(to_integer(a_ra));
            if a_we = '1' then accr(to_integer(a_wa)) <= a_wd; end if;
        end if;
    end process;
    p_sdly : process(clk)
    begin
        if rising_edge(clk) then
            s_dq <= s_dly(to_integer(s_drp));
            s_dly(to_integer(s_dwp)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n & g_avid;
        end if;
    end process;
    p_orom : process(clk)
    begin
        if rising_edge(clk) then
            o_q <= C_OUTLINE(to_integer(o_ra));
        end if;
    end process;
    -- register file: the machine reads it in vblank, the line engine in hblank
    rf_ra <= q_ra when q_run = '1' else l_ra;
    p_rf : process(clk)
    begin
        if rising_edge(clk) then
            rf_q <= rf(to_integer(rf_ra));
            if rf_we = '1' then rf(to_integer(rf_wa)) <= rf_wd; end if;
            rf_qq <= rf_q;
        end if;
    end process p_rf;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, width, sync delays
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_xs <= f_xs(s_tid);
            s_kr <= f_kr(s_tid);
            s_nb <= f_nb(s_tid);
            case to_integer(s_xs) is
                when 4 =>      s_kw <= to_unsigned(1553, 11); s_xco <= to_unsigned(169, 9); s_na <= "01";
                when 3 =>      s_kw <= to_unsigned(1165, 11); s_xco <= to_unsigned(225, 9); s_na <= "01";
                when others => s_kw <= to_unsigned(777, 11);  s_xco <= to_unsigned(338, 9); s_na <= "10";
            end case;

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle(10) = '0' then
                s_idle <= s_idle + 1;
            end if;
            s_zone <= s_idle(10);
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

            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;      -- K6 Speed, deadband
            if v_s > -16 and v_s < 16 then s_spr <= (others => '0'); else s_spr <= v_s(9 downto 0); end if;
            if s_zgo = '1' then
                s_sp <= s_spr;
                s_order <= registers_in(6)(0);
                s_cmy   <= registers_in(6)(1);
                s_fink  <= registers_in(6)(2);
                s_flip  <= registers_in(6)(3);
                s_osc   <= registers_in(6)(4);
                if s_wmax /= 0 then s_w <= s_wmax(10 downto 0); end if;
                s_wmax <= (others => '0');
            end if;
            g_wm1 <= resize(s_w, 12) - 1;
            s_xc <= ('0' & s_w(10 downto 1)) - resize(s_xco, 11);
            s_xca <= s_xc - 5;

            -- delay line: written each clock, read 18 entries back (s_dq = input delayed 19),
            -- then two registers (delay 20, 21 = C_LAT)
            s_dwp <= s_dwp + 1;
            s_drp <= s_dwp - 17;
            s_dg <= s_dq; s_do <= s_dg;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Register machine (vblank).  Load steps: 4 clocks.  Arithmetic steps: 48 clocks:
    -- cnt 1..3 operand addresses (A, B, K; data 2 clocks later), 6..21 multiply then up to
    -- 15 arithmetic shifts / 6..32 divide, 38 sign, 39 add / compare, 40 post, 41 write (42 high word).
    ---------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_l : signed(15 downto 0);
        variable v_hi : signed(20 downto 0);
        variable v_r2 : unsigned(15 downto 0);
        variable v_ps, v_res : signed(27 downto 0);
        variable v_tri : unsigned(15 downto 0);
        variable v_sc : signed(16 downto 0);
        variable v_end : std_logic;
        variable v_d : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if r_sp(15) = '1' then v_tri := r_sp - 32768; else v_tri := 32768 - r_sp; end if;
            r_tri <= signed(v_tri) - 16384;
            q_done <= '0'; rf_we <= '0'; m_wr <= '0'; m_wr2 <= m_wr and q_uc.wide;

            if s_zgo = '1' then
                q_run <= '1'; q_st <= (others => '0'); q_ci <= (others => '0'); q_cnt <= (others => '0');
                -- motion: wall scroll (Q.4 lines, wraps one print), breathing phase, Osc logo
                v_sc := resize(r_sc, 17) + resize(shift_right(s_sp, 6), 17);
                if v_sc >= signed(resize(r_t, 17)) then
                    r_sc <= resize(v_sc - signed(resize(r_t, 17)), 16); r_wb <= r_wb + C_KR;
                elsif v_sc < 0 then
                    r_sc <= resize(v_sc + signed(resize(r_t, 17)), 16); r_wb <= r_wb - C_KR;
                else
                    r_sc <= resize(v_sc, 16);
                end if;
                r_sp <= r_sp + unsigned(resize(shift_right(s_sp, 1), 16));
                if r_ls + 33 >= 5888 then r_ls <= r_ls + 33 - 5888; else r_ls <= r_ls + 33; end if;
            elsif q_run = '1' then
                q_cnt <= q_cnt + 1;
                v_end := '0';
                if q_cnt = 0 then
                    q_uc <= f_uc(q_st);
                elsif q_uc.op = 2 then
                    if q_cnt = 1 then
                        case q_uc.a is
                            when L_T12 => v_l := signed(resize(unsigned(registers_in(7)) & "00", 16));
                            when L_T1  => v_l := signed(resize(unsigned(registers_in(0)) & "00", 16));
                            when L_T2  => v_l := signed(resize(unsigned(registers_in(1)) & "00", 16));
                            when L_T4  => v_l := signed(resize(unsigned(registers_in(3)) & "00", 16));
                            when L_K3  => v_l := signed(resize(unsigned(registers_in(2)), 16));
                            when L_K5  => v_l := signed(resize(unsigned(registers_in(4)), 16));
                            when L_W   => v_l := signed(resize(s_w, 16));
                            when L_TRI => v_l := r_tri;
                            when others => v_l := r_sc;
                        end case;
                        rf_we <= '1'; rf_wa <= to_unsigned(q_uc.d, 8); rf_wd <= std_logic_vector(v_l);
                    elsif q_cnt = 3 then
                        v_end := '1';
                    end if;
                else
                    if q_cnt = 1 then
                        q_ra <= f_am(q_uc.a, q_uc.am, q_ci, s_tid);
                    elsif q_cnt = 2 then
                        q_ra <= f_am(q_uc.b, q_uc.bm, q_ci, s_tid);
                    elsif q_cnt = 3 then
                        q_ra <= to_unsigned(q_uc.k, 8);
                    end if;
                    if q_cnt = 4 then
                        m_a <= signed(rf_qq);
                    elsif q_cnt = 5 then
                        m_p <= resize(unsigned(rf_qq), 37);
                        m_rem <= (others => '0'); m_dq <= (others => '0');
                        if q_uc.op = 1 then
                            case q_uc.b is
                                when 22 => m_dq(22) <= '1';
                                when others => m_dq(26) <= '1';
                            end case;
                        end if;
                    elsif q_cnt = 6 then
                        m_k <= signed(rf_qq);
                    end if;
                    -- multiply 6..21, then sh arithmetic shifts; divide 6..32
                    if q_cnt >= 6 and q_cnt <= 32 then
                        v_r2 := m_rem(14 downto 0) & m_dq(26);
                        if v_r2 >= unsigned(m_a) then
                            m_rem <= v_r2 - unsigned(m_a); m_dq <= m_dq(25 downto 0) & '1';
                        else
                            m_rem <= v_r2; m_dq <= m_dq(25 downto 0) & '0';
                        end if;
                    end if;
                    if q_cnt >= 6 and q_cnt <= 21 then
                        if m_p(0) = '1' then
                            v_hi := signed(m_p(36 downto 16)) + resize(m_a, 21);
                        else
                            v_hi := signed(m_p(36 downto 16));
                        end if;
                        m_p(36 downto 15) <= unsigned(resize(v_hi, 22));
                        m_p(14 downto 0) <= m_p(15 downto 1);
                    elsif q_cnt >= 22 and q_cnt <= 36 then
                        if q_cnt - 22 < q_uc.sh then
                            m_p <= unsigned(shift_right(signed(m_p), 1));
                        end if;
                    end if;
                    if q_cnt = 38 then
                        if q_uc.op = 1 then v_ps := signed(resize(m_dq(15 downto 0), 28));
                        else v_ps := signed(m_p(27 downto 0)); end if;
                        if q_uc.ng = '1' then m_ps <= -v_ps; else m_ps <= v_ps; end if;
                    elsif q_cnt = 39 then
                        m_sum <= m_ps + resize(m_k, 28);
                        if m_ps < resize(m_k, 28) then m_lt <= '1'; else m_lt <= '0'; end if;
                        if m_ps < 0 then m_neg <= '1'; else m_neg <= '0'; end if;
                        if m_ps > 1024 then m_big <= '1'; else m_big <= '0'; end if;
                    elsif q_cnt = 40 then
                        case q_uc.post is
                            when 1 =>
                                if m_neg = '1' then v_res := (others => '0');
                                elsif m_big = '1' then v_res := to_signed(1024, 28);
                                else v_res := m_ps; end if;
                            when 2 =>
                                if m_neg = '1' then v_res := (others => '0'); else v_res := m_ps; end if;
                            when 3 =>
                                if m_lt = '1' then v_res := m_ps; else v_res := resize(m_k, 28); end if;
                            when 4 =>
                                if m_lt = '0' then v_res := m_ps; else v_res := resize(m_k, 28); end if;
                            when others =>
                                v_res := m_sum;
                        end case;
                        m_res <= v_res;
                        m_wr <= '1';
                    elsif q_cnt = 44 then
                        v_end := '1';
                    end if;
                end if;
                -- write-back (cnt 41; the high word of a wide result at cnt 42, address + 4)
                v_d := to_unsigned(q_uc.d, 8);
                if q_uc.dci = '1' then v_d(1 downto 0) := q_ci; end if;
                if m_wr = '1' then
                    if q_uc.d /= 0 then
                        rf_we <= '1'; rf_wa <= v_d; rf_wd <= std_logic_vector(m_res(15 downto 0));
                    end if;
                    case q_uc.o is
                        when O_HI  => r_hi <= unsigned(m_res(8 downto 0));
                        when O_LO  => r_lo <= unsigned(m_res(9 downto 0));
                        when O_KTR => r_ktr <= unsigned(m_res(13 downto 0));
                        when O_T   => r_t <= unsigned(m_res(12 downto 0));
                        when O_KU  =>
                            for i in 0 to 3 loop
                                if q_ci = i then p_ku(i) <= unsigned(m_res(13 downto 0)); end if;
                            end loop;
                        when others => null;
                    end case;
                elsif m_wr2 = '1' then
                    rf_we <= '1'; rf_wa <= v_d + 4; rf_wd <= std_logic_vector(resize(m_res(27 downto 16), 16));
                end if;
                if v_end = '1' then
                    q_cnt <= (others => '0');
                    if q_st = ST_LAST then
                        if q_ci = 3 then q_run <= '0'; q_done <= '1';
                        else q_ci <= q_ci + 1; q_st <= to_unsigned(ST_COPY, 6); end if;
                    else
                        q_st <= q_st + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_seq;

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 2/3 clocks after the source avid rise)
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
            g_avd <= g_avid;
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- Line engine: runs in hblank (8 clocks after the generated avid falls, or once after the
    -- register machine at the field start) and prepares the NEXT line.  Register-file reads
    -- (data readable 3 clocks later): t0..3 KV(i) (field start: t0..7 V0 lo/hi), t8..15 U0 lo/hi,
    -- t16 G0, t17 KTV, t18 VT0.  Outline ROM reads t12..27.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_ci : integer range 0 to 3;
        variable v_w : std_logic;
        variable v_row : unsigned(6 downto 0);
        variable v_e, v_e1 : signed(12 downto 0);
        variable v_t : integer range 0 to 63;
    begin
        if rising_edge(clk) then
            l_ld <= (others => '0');
            l_kickp <= '0';
            if q_done = '1' then l_fkick <= '1'; end if;
            if g_avd = '1' and g_avid = '0' then
                l_dly <= to_unsigned(1, 4);
            elsif l_dly /= 0 then
                l_dly <= l_dly + 1;
                if l_dly = 8 then l_kickp <= '1'; l_dly <= (others => '0'); end if;
            end if;
            if (l_kickp = '1' or (l_fkick = '1' and g_avid = '0' and q_run = '0')) and l_run = '0' then
                l_run <= '1'; l_cnt <= (others => '0');
                l_init <= l_fkick and not l_kickp;
                if l_kickp = '0' then l_fkick <= '0'; end if;
            elsif l_run = '1' then
                l_cnt <= l_cnt + 1;
                if l_cnt = 34 then l_run <= '0'; end if;
            end if;
            v_t := to_integer(l_cnt);
            -- one-hot add enables, a clock ahead (KV(i) readable at t = 3 + i)
            l_vadd <= (others => '0');
            if l_run = '1' and l_init = '0' and v_t >= 2 and v_t <= 5 then
                l_vadd(v_t - 2) <= '1';
            end if;

            -- register-file read addresses
            if l_run = '1' then
                if v_t <= 7 then
                    if l_init = '1' then
                        l_ra <= to_unsigned(RF_V0, 8) + to_unsigned(v_t / 2, 8) + shift_left(to_unsigned(v_t mod 2, 8), 2);
                    else
                        l_ra <= to_unsigned(RF_KV, 8) + to_unsigned(v_t mod 4, 8);
                    end if;
                elsif v_t <= 15 then
                    l_ra <= to_unsigned(RF_U0, 8) + to_unsigned((v_t - 8) / 2, 8) + shift_left(to_unsigned((v_t - 8) mod 2, 8), 2);
                elsif v_t = 16 then
                    l_ra <= to_unsigned(RF_KTV, 8);
                elsif v_t = 17 then
                    l_ra <= to_unsigned(RF_VT0, 8);
                else
                    l_ra <= to_unsigned(RF_G0, 8);
                end if;
            end if;
            -- v: per line v += KV * 2 (shared adder, t3..6); field start: v = V0 (lo t3,5,7,9; hi t4..10)
            if l_run = '1' then
                for i in 0 to 3 loop
                    if l_vadd(i) = '1' then
                        l_v(i) <= l_v(i) + signed(resize(shift_left(resize(unsigned(rf_qq), 17), 1), 27));
                    end if;
                end loop;
                if l_init = '1' and v_t >= 3 and v_t <= 10 then
                    if v_t mod 2 = 1 then
                        l_lo <= rf_qq;
                    else
                        v_ci := (v_t - 4) / 2;
                        l_v(v_ci) <= signed(std_logic_vector'(rf_qq(10 downto 0) & l_lo));
                    end if;
                end if;
                -- t11: rows, fractions, inside flags
                if v_t = 11 then
                    for i in 0 to 3 loop
                        if l_v(i) >= 0 and l_v(i) < 2 ** 21 then l_vin(i) <= '1'; else l_vin(i) <= '0'; end if;
                        l_vr(i) <= unsigned(l_v(i)(20 downto 14));
                        l_vf(i) <= unsigned(l_v(i)(13 downto 10));
                    end loop;
                end if;
                -- U0 (lo t11,13,15,17; hi t12..18): load copy i's column DDA
                if v_t >= 11 and v_t <= 18 then
                    if v_t mod 2 = 1 then
                        l_lo <= rf_qq;
                    else
                        l_u0 <= signed(std_logic_vector'(rf_qq(9 downto 0) & l_lo));
                        l_ld((v_t - 12) / 2) <= '1';
                    end if;
                end if;
                -- wall rows: KTV (t19, wrap t20) / VT0 at the field start (t20), G0 (t22)
                if v_t = 19 and l_init = '0' then
                    l_vt <= l_vt + unsigned(rf_qq);
                end if;
                if v_t = 20 and l_init = '0' and l_vt >= 163840 then
                    l_vt <= l_vt - 163840; l_wrow <= l_wrow + C_KR;
                end if;
                if v_t = 20 and l_init = '1' then
                    l_vt <= shift_left(resize(unsigned(rf_qq), 18), 2); l_wrow <= r_wb;
                end if;
                if v_t = 21 then
                    l_hl <= f_mix1(l_wrow);
                    if l_vt < 2048 then l_seamt <= '1'; else l_seamt <= '0'; end if;
                end if;
                if v_t = 22 then
                    l_ut0 <= shift_left(resize(unsigned(rf_qq), 18), 2) + shift_left(resize(l_hl(15 downto 11), 18), 10) - 16384;
                end if;
            end if;

            -- outline ROM reads (t12..27), j = 0..15: copy j/4, word (j/2)%2, row +(j%2)
            if l_run = '1' and v_t = 12 then
                l_j <= (others => '0'); l_jv <= '1';
            elsif l_jv = '1' then
                if l_j = 15 then l_jv <= '0'; else l_j <= l_j + 1; end if;
            end if;
            v_ci := to_integer(l_j(3 downto 2));
            v_w := l_j(1);
            v_row := l_vr(v_ci);
            if l_j(0) = '1' and v_row /= 127 then v_row := v_row + 1; end if;
            o_ra <= v_row & v_w;
            l_tg1 <= l_j; l_tv1 <= l_jv;
            l_tg2 <= l_tg1; l_tv2 <= l_tv1;
            l_tg3 <= l_tg2; l_tv3 <= l_tv2;
            l_oq <= unsigned(o_q);
            l_wv <= '0';
            if l_tv3 = '1' then
                if l_tg3(0) = '0' then
                    l_e0 <= l_oq;
                else
                    l_d0 <= signed(resize(l_oq(7 downto 0), 9)) - signed(resize(l_e0(7 downto 0), 9));
                    l_d1 <= signed(resize(l_oq(15 downto 8), 9)) - signed(resize(l_e0(15 downto 8), 9));
                    l_b0 <= l_e0(7 downto 0); l_b1 <= l_e0(15 downto 8);
                    l_f <= l_vf(to_integer(l_tg3(3 downto 2)));
                    l_wp <= l_tg3(3 downto 1);
                    l_wv <= '1';
                end if;
            end if;
            -- lerp: the products (one stage), then e = 4 b + (d f) / 4 and the write
            l_p0 <= l_d0 * signed('0' & l_f); l_p1 <= l_d1 * signed('0' & l_f);
            l_bb0 <= l_b0; l_bb1 <= l_b1; l_wv2 <= l_wv; l_wp2 <= l_wp;
            v_e := signed(shift_left(resize(l_bb0, 13), 2)) + resize(shift_right(l_p0, 2), 13);
            v_e1 := signed(shift_left(resize(l_bb1, 13), 2)) + resize(shift_right(l_p1, 2), 13);
            for k in 0 to 7 loop
                if l_wv2 = '1' and l_wp2 = k then
                    l_e(2 * k) <= unsigned(v_e(9 downto 0));
                    l_e(2 * k + 1) <= unsigned(v_e1(9 downto 0));
                end if;
            end loop;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Pixel path (S1 = the register loaded for pixel 0 at the g_start edge)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ue : unsigned(18 downto 0);
        variable v_uc : unsigned(9 downto 0);
        variable v_ut : unsigned(18 downto 0);
        variable v_ti : unsigned(5 downto 0);
        variable v_in : std_logic_vector(3 downto 0);
        variable v_c1 : unsigned(7 downto 0);
        variable v_er : unsigned(6 downto 0);
        variable v_tl, v_tr, v_bl, v_br : unsigned(3 downto 0);
        variable v_d : signed(18 downto 0);
        variable v_dd : unsigned(7 downto 0);
        variable v_dx : signed(10 downto 0);
        variable v_ix : unsigned(2 downto 0);
        variable v_m : unsigned(8 downto 0);
        variable v_y, v_u, v_v : signed(20 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- S1: copy column DDAs (loaded in hblank by the line engine), print DDA and hash
            for i in 0 to 3 loop
                if l_ld(i) = '1' then
                    p_u(i) <= l_u0;
                elsif g_avid = '1' then
                    p_u(i) <= p_u(i) + signed(resize(p_ku(i), 26));
                end if;
            end loop;
            p_hn <= f_mix1(p_tw + C_KT);
            if g_start = '1' then
                p_ut <= l_ut0; p_tw <= l_wrow; p_hc <= l_hl;
            else
                v_ut := resize(p_ut, 19) + resize(r_ktr, 19);
                p_ut <= v_ut(17 downto 0);
                if v_ut(18) = '1' then
                    p_tw <= p_tw + C_KT; p_hc <= p_hn;
                end if;
            end if;

            -- S2: flip, outline compares; print window
            for i in 0 to 3 loop
                if s_flip = '1' and (i = 1 or i = 3) then v_ue := not unsigned(p_u(i)(18 downto 0));
                else v_ue := unsigned(p_u(i)(18 downto 0)); end if;
                v_uc := v_ue(18 downto 9);
                for k in 0 to 3 loop
                    if v_uc >= l_e(i * 4 + k) then p_cmp(i * 4 + k) <= '1'; else p_cmp(i * 4 + k) <= '0'; end if;
                end loop;
                if p_u(i)(25 downto 19) = "0000000" and l_vin(i) = '1' then p_ok2(i) <= '1'; else p_ok2(i) <= '0'; end if;
                p_cf2(i) <= v_ue(18 downto 8);
            end loop;
            v_ti := p_ut(17 downto 12);
            if p_hc(11) = '1' then
                p_tc2 <= resize(p_hc(5 downto 0), 7) + resize(63 - v_ti, 7);
                p_tfx2 <= not p_ut(11 downto 8);
            else
                p_tc2 <= resize(p_hc(5 downto 0), 7) + resize(v_ti, 7);
                p_tfx2 <= p_ut(11 downto 8);
            end if;
            p_tr2 <= resize(p_hc(10 downto 6), 7) + resize(l_vt(17 downto 12), 7);
            p_tfy2 <= l_vt(11 downto 8);
            if (p_hc(14) = '1' and p_hc(12) = '1' and v_ti = 0) or (p_hc(15) = '1' and p_hc(13) = '1' and l_seamt = '1') then
                p_tsm2 <= '1';
            else
                p_tsm2 <= '0';
            end if;
            p_tg2 <= p_hc(13 downto 12);

            -- S3: inside flags; the front-most copy wins, else the wall
            for i in 0 to 3 loop
                if p_ok2(i) = '1' and ((p_cmp(i * 4) = '1' and p_cmp(i * 4 + 1) = '0') or
                                      (p_cmp(i * 4 + 2) = '1' and p_cmp(i * 4 + 3) = '0')) then
                    v_in(i) := '1';
                else
                    v_in(i) := '0';
                end if;
            end loop;
            if v_in(3) = '1' then
                p_ly3 <= "011"; p_c03 <= p_cf2(3)(10 downto 4); p_fx3 <= p_cf2(3)(3 downto 0);
                p_r03 <= l_vr(3); p_fy3 <= l_vf(3);
            elsif v_in(2) = '1' then
                p_ly3 <= "010"; p_c03 <= p_cf2(2)(10 downto 4); p_fx3 <= p_cf2(2)(3 downto 0);
                p_r03 <= l_vr(2); p_fy3 <= l_vf(2);
            elsif v_in(1) = '1' then
                p_ly3 <= "001"; p_c03 <= p_cf2(1)(10 downto 4); p_fx3 <= p_cf2(1)(3 downto 0);
                p_r03 <= l_vr(1); p_fy3 <= l_vf(1);
            elsif v_in(0) = '1' then
                p_ly3 <= "000"; p_c03 <= p_cf2(0)(10 downto 4); p_fx3 <= p_cf2(0)(3 downto 0);
                p_r03 <= l_vr(0); p_fy3 <= l_vf(0);
            else
                p_ly3 <= "100"; p_c03 <= p_tc2; p_fx3 <= p_tfx2; p_r03 <= p_tr2; p_fy3 <= p_tfy2;
            end if;
            p_tsm3 <= p_tsm2; p_tg3 <= p_tg2;

            -- S4: bank addresses (even banks hold c0+1 / r0+1 when c0 / r0 is odd)
            v_c1 := resize(p_c03, 8) + 1;
            v_er := resize(p_r03(6 downto 1), 7) + resize(p_r03(0 downto 0), 7);
            if v_er(6) = '1' then v_er := to_unsigned(63, 7); end if;
            b_ra0 <= v_er(5 downto 0) & v_c1(6 downto 1);
            b_ra1 <= v_er(5 downto 0) & p_c03(6 downto 1);
            b_ra2 <= p_r03(6 downto 1) & v_c1(6 downto 1);
            b_ra3 <= p_r03(6 downto 1) & p_c03(6 downto 1);
            c_ra <= p_r03(6 downto 2) & p_c03(6 downto 2);
            p_sd(4) <= (ly => p_ly3, cp => p_c03(0), rp => p_r03(0), fx => p_fx3, fy => p_fy3, tg => p_tg3, ts => p_tsm3);
            for k in 5 to 13 loop p_sd(k) <= p_sd(k - 1); end loop;
            -- S5: RAM output registers.  S6: fabric copies
            p_q0 <= unsigned(b_q0); p_q1 <= unsigned(b_q1); p_q2 <= unsigned(b_q2); p_q3 <= unsigned(b_q3);
            p_cq(6) <= c_q;
            for k in 7 to 13 loop p_cq(k) <= p_cq(k - 1); end loop;

            -- S7: corners by parity; differences for both lerps
            if p_sd(6).rp = '0' then
                if p_sd(6).cp = '0' then v_tl := p_q0; v_tr := p_q1; v_bl := p_q2; v_br := p_q3;
                else v_tl := p_q1; v_tr := p_q0; v_bl := p_q3; v_br := p_q2; end if;
            else
                if p_sd(6).cp = '0' then v_tl := p_q2; v_tr := p_q3; v_bl := p_q0; v_br := p_q1;
                else v_tl := p_q3; v_tr := p_q2; v_bl := p_q1; v_br := p_q0; end if;
            end if;
            p_tl <= v_tl;
            p_dt <= signed(resize(v_tr, 6)) - signed(resize(v_tl, 6));
            p_dtl <= signed(resize(v_bl, 6)) - signed(resize(v_tl, 6));
            p_ddt <= (signed(resize(v_br, 6)) - signed(resize(v_bl, 6))) - (signed(resize(v_tr, 6)) - signed(resize(v_tl, 6)));
            -- S8: top row and the vertical difference, both lerped across (x16)
            p_top8 <= signed(shift_left(resize(p_tl, 11), 4)) + resize(p_dt * signed('0' & p_sd(7).fx), 11);
            p_dv8 <= shift_left(resize(p_dtl, 11), 4) + resize(p_ddt * signed('0' & p_sd(7).fx), 11);
            -- S9: L8 = (top*16 + dv*fy) / 16
            v_d := shift_left(resize(p_top8, 19), 4) + resize(p_dv8 * signed('0' & p_sd(8).fy), 19);
            if v_d < 0 then p_l9 <= (others => '0'); else p_l9 <= unsigned(v_d(11 downto 4)); end if;
            -- S10: density 2 (HI - L8) clamped; palette index
            if p_lfsr(0) = '1' then p_lfsr <= shift_right(p_lfsr, 1) xor x"B400"; else p_lfsr <= shift_right(p_lfsr, 1); end if;
            v_dx := signed(resize(r_hi, 11)) - signed(resize(p_l9, 11));
            if v_dx < 0 then p_dd10 <= (others => '0');
            elsif v_dx >= 128 then p_dd10 <= (others => '1');
            else p_dd10 <= unsigned(v_dx(6 downto 0)) & '0'; end if;
            case p_sd(9).ly is
                when "000" => if s_order = '1' then v_ix := "010"; else v_ix := "000"; end if;
                when "001" => v_ix := "001";
                when "010" => if s_order = '1' then v_ix := "000"; else v_ix := "010"; end if;
                when "011" => if s_fink = '1' then v_ix := "110"; else v_ix := "111"; end if;
                when others => v_ix := "111";
            end case;
            if s_cmy = '1' and v_ix(2 downto 1) /= "11" then v_ix := v_ix + 3; end if;
            p_ix10 <= v_ix; p_l10 <= p_l9;
            -- S11: toner grain; palette address (level dithered to 5 bits)
            v_dd := p_dd10;
            if p_lfsr(7 downto 0) >= 253 then
                v_dd := v_dd - shift_right(v_dd, 2) - shift_right(v_dd, 3);
            elsif p_lfsr(1) = '1' then
                v_dd := v_dd - shift_right(v_dd, 4);
            end if;
            if p_ix10 = "111" then v_m := resize(p_l10, 9); else v_m := resize(v_dd, 9); end if;
            v_m := v_m + resize(p_lfsr(10 downto 8), 9);
            if v_m(8) = '1' then v_m := to_unsigned(255, 9); end if;
            p_pa11 <= p_ix10 & v_m(7 downto 3);
            -- S12: palette ROM.  S13: fabric register
            p_ph12 <= C_PALH(to_integer(p_pa11)); p_pl12 <= C_PALL(to_integer(p_pa11));
            p_ph13 <= p_ph12; p_pl13 <= p_pl12;
            -- (S13) the wall's luma offset: Wall + print exposure - 24
            p_wo13 <= signed(resize(r_lo, 12)) + signed(resize(shift_left(resize(p_sd(12).tg, 6), 4), 12)) - 24;
            -- S14: compose. Wall = 7/8 of the luma ramp + the offset; chroma final (front colour
            -- 512 + 32 cq stays inside 256..736, palette entries are in range: no clamps)
            v_y := signed(resize(unsigned(p_ph13(15 downto 6)), 21));
            p_yr14 <= unsigned(p_ph13(15 downto 6));
            p_yw14 <= resize(v_y - shift_right(v_y, 3), 12) + p_wo13;
            p_ts14 <= p_sd(13).ts;
            if p_sd(13).ly = "100" then p_wl14 <= '1'; else p_wl14 <= '0'; end if;
            if p_sd(13).ly = "100" then
                p_u14 <= to_unsigned(512, 10); p_v14 <= to_unsigned(512, 10);
            elsif p_sd(13).ly = "011" and s_fink = '0' then
                v_u := to_signed(512, 21) + shift_left(resize(signed(p_cq(13)(7 downto 4)), 21), 5);
                v_v := to_signed(512, 21) + shift_left(resize(signed(p_cq(13)(3 downto 0)), 21), 5);
                p_u14 <= unsigned(v_u(9 downto 0)); p_v14 <= unsigned(v_v(9 downto 0));
            else
                p_u14 <= unsigned(p_ph13(5 downto 0)) & unsigned(p_pl13(15 downto 12));
                p_v14 <= unsigned(p_pl13(11 downto 2));
            end if;
            -- S15: the wall's seams (half + 400) and clamp
            if p_wl14 = '1' then
                v_y := resize(p_yw14, 21);
                if p_ts14 = '1' then v_y := shift_right(v_y, 1) + 400; end if;
                p_y15 <= f_clamp10(v_y);
            else
                p_y15 <= p_yr14;
            end if;
            p_u15 <= p_u14; p_v15 <= p_v14;

            -- 16..21: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent 8-sums
            -- on 8-bit chroma, luma [1 2 1]); 22: luma toward black (white 940 -> 777), the gate
            s_csu(0) <= p_u15(9 downto 2); s_csv(0) <= p_v15(9 downto 2);
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + resize(p_u15(9 downto 2), 11) - resize(s_csu(7), 11);
            s_boxv <= s_boxv + resize(p_v15(9 downto 2), 11) - resize(s_csv(7), 11);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 12) + s_boxud; s_u21x <= v_sc(11 downto 2);
            v_sc := resize(s_boxv, 12) + s_boxvd; s_v21x <= v_sc(11 downto 2);
            s_y16 <= p_y15; s_y17 <= s_y16; s_y18 <= s_y17; s_y19 <= s_y18;
            s_y20 <= s_y19; s_y21x <= s_y20;
            v_sy := resize(s_y19, 12) + shift_left(resize(s_y20, 12), 1) + resize(s_y21x, 12);
            s_yf21 <= v_sy(11 downto 2);
            if s_yf21 > 64 then v_dl := s_yf21 - 64; else v_dl := (others => '0'); end if;
            if s_dg(0) = '1' then
                s_y22 <= s_yf21 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u22 <= s_u21x; s_v22 <= s_v21x;
            else
                s_y22 <= to_unsigned(64, 10); s_u22 <= to_unsigned(512, 10); s_v22 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Write path: source pixels -> box sums -> store (and chroma store)
    ---------------------------------------------------------------------------
    p_wr : process(clk)
        variable v_wu : unsigned(20 downto 0);
        variable v_j : unsigned(2 downto 0);
        variable v_y : unsigned(7 downto 0);
        variable v_cu, v_cv : signed(8 downto 0);
        variable v_lv : signed(9 downto 0);
        variable v_q : signed(17 downto 0);
        variable v_ca : signed(8 downto 0);
        variable v_c4u, v_c4v : signed(3 downto 0);
        variable v_ad : unsigned(4 downto 0);
        variable v_ua : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 1: pixel index, the store-column DDA, new column / sample rank;
            -- a second DDA runs 4 pixels ahead for the Osc generator
            if data_in.avid = '1' then w_sx <= w_sx + 1; else w_sx <= (others => '0'); end if;
            w_winp <= w_win;
            if data_in.avid = '1' and w_sx = resize(s_xc, 12) then
                w_wu <= (others => '0'); w_win <= '1'; w_nw <= '1'; v_j := (others => '0');
            elsif data_in.avid = '1' and w_win = '1' then
                v_wu := resize(w_wu, 21) + resize(s_kw, 21);
                w_wu <= v_wu(19 downto 0);
                if v_wu(19) = '1' then w_win <= '0'; end if;
                if v_wu(18 downto 12) /= w_wu(18 downto 12) then
                    w_nw <= '1'; v_j := (others => '0');
                else
                    w_nw <= '0'; if w_j /= 7 then v_j := w_j + 1; else v_j := w_j; end if;
                end if;
            else
                w_win <= '0'; w_nw <= '0'; v_j := w_j;
            end if;
            w_j <= v_j;
            if v_j < shift_left(to_unsigned(1, 3), to_integer(s_na)) then w_smp <= '1'; else w_smp <= '0'; end if;
            if data_in.avid = '1' and w_sx = resize(s_xca, 12) then
                w_wua <= (others => '0');
            else
                w_wua <= w_wua + resize(s_kw, 20);
            end if;
            w_y1 <= unsigned(data_in.y(9 downto 2));
            w_u1 <= unsigned(data_in.u(9 downto 3));
            w_v1 <= unsigned(data_in.v(9 downto 3));

            -- Osc generator from the look-ahead column (aligned to stage 1 five clocks later)
            -- g0: column flags; distances from the jaw / eye / nose centres; squarer input
            -- (|2u - 97|, or in hblank the next line's |2r - 39|)
            v_ua := w_wua(18 downto 12);
            o_cf1 <= f_osc_col(v_ua);
            if data_in.avid = '0' and w_win = '0' then
                v_lv := signed(shift_left(resize(w_wvn(20 downto 14), 10), 1)) - 39; o_hbq0 <= '1';
            else
                v_lv := signed(shift_left(resize(v_ua, 10), 1)) - 97; o_hbq0 <= '0';
            end if;
            if v_lv < 0 then v_lv := -v_lv; end if;
            if v_lv > 127 then o_adu0 <= (others => '1'); else o_adu0 <= unsigned(v_lv(6 downto 0)); end if;
            v_lv := signed(resize(v_ua, 10)) - 50; if v_lv < 0 then v_lv := -v_lv; end if;
            o_d50 <= unsigned(v_lv(6 downto 0));
            v_lv := signed(resize(v_ua, 10)) - 39; if v_lv < 0 then v_lv := -v_lv; end if;
            o_d39 <= unsigned(v_lv(6 downto 0));
            v_lv := signed(resize(v_ua, 10)) - 62; if v_lv < 0 then v_lv := -v_lv; end if;
            o_d62 <= unsigned(v_lv(6 downto 0));
            -- g1: jaw / eye / nose tests against this line's widths
            o_cf2 <= o_cf1; o_adu <= o_adu0; o_hbq <= o_hbq0;
            if o_d50 < resize(unsigned(o_row(21 downto 17)), 7) then o_jw2 <= '1'; else o_jw2 <= '0'; end if;
            if o_d50 < resize(unsigned(o_row(12 downto 10)), 7)
               or o_d39 < resize(unsigned(o_row(16 downto 13)), 7)
               or o_d62 < resize(unsigned(o_row(16 downto 13)), 7) then
                o_ey2 <= '1';
            else
                o_ey2 <= '0';
            end if;
            -- g2: square; marks
            o_sq <= o_adu * o_adu; o_hbq3 <= o_hbq;
            o_cf3 <= o_cf2; o_jw3 <= o_jw2;
            if o_ey2 = '1'
               or (o_row(0) = '1' and o_cf2(0) = '1') or (o_row(1) = '1' and o_cf2(1) = '1')
               or (o_row(2) = '1' and o_cf2(2) = '1') or (o_row(3) = '1' and o_cf2(3) = '1')
               or (o_row(4) = '1' and o_cf2(4) = '1') or (o_row(5) = '1' and o_cf2(5) = '1') then
                o_mk3 <= '1';
            else
                o_mk3 <= '0';
            end if;
            if o_hbq3 = '1' then
                o_t1 <= shift_left(resize(o_sq, 20), 5) + shift_left(resize(o_sq, 20), 3);   -- x40
            end if;
            -- g3: cranium radius r1 = du^2 * 24 + dv^2 * 40; region flags
            o_r1 <= shift_left(resize(o_sq, 21), 4) + shift_left(resize(o_sq, 21), 3) + resize(o_t1, 21);
            o_sk4 <= o_jw3; o_mk4 <= o_mk3;
            o_fl4(0) <= o_row(6) and o_cf3(6);                       -- monitor
            o_fl4(1) <= o_row(7) and o_cf3(7);                       -- screen
            o_fl4(2) <= o_row(7) and o_cf3(8) and o_lg;              -- logo
            o_fl4(3) <= o_row(8) and o_cf3(9);                       -- console
            o_fl4(4) <= o_row(9) and o_cf3(10);                      -- keys
            -- g4: compose (skull shading 852 - r1 (1/256 + 1/512), floor 400)
            if o_r1(20 downto 16) /= "00000" or o_r1(15 downto 9) > 150 then
                v_q := to_signed(400, 18);
            else
                v_q := to_signed(852, 18) - signed(resize(o_r1(15 downto 9) & '0', 18)) - signed(resize(o_r1(15 downto 9), 18));
            end if;
            o_gy <= to_unsigned(179, 10) + resize(p_lfsr(13 downto 8), 10); o_gu <= to_unsigned(505, 10); o_gv <= to_unsigned(534, 10);
            if o_r1 < 65536 or o_sk4 = '1' then
                if o_mk4 = '1' then
                    o_gy <= to_unsigned(176, 10); o_gu <= to_unsigned(531, 10); o_gv <= to_unsigned(499, 10);
                else
                    o_gy <= unsigned(v_q(9 downto 0)); o_gu <= to_unsigned(575, 10); o_gv <= to_unsigned(411, 10);
                end if;
            end if;
            if o_fl4(0) = '1' then o_gy <= to_unsigned(760, 10); o_gu <= to_unsigned(521, 10); o_gv <= to_unsigned(459, 10); end if;
            if o_fl4(1) = '1' then
                if o_fl4(2) = '1' then o_gy <= to_unsigned(160, 10); o_gu <= to_unsigned(492, 10); o_gv <= to_unsigned(575, 10);
                else o_gy <= to_unsigned(409, 10); o_gu <= to_unsigned(411, 10); o_gv <= to_unsigned(686, 10); end if;
            end if;
            if o_fl4(3) = '1' then
                if o_fl4(4) = '1' then o_gy <= to_unsigned(273, 10); o_gu <= to_unsigned(510, 10); o_gv <= to_unsigned(503, 10);
                else o_gy <= to_unsigned(770, 10); o_gu <= to_unsigned(524, 10); o_gv <= to_unsigned(462, 10); end if;
            end if;
            -- per-line generator terms from the committed store row
            o_row <= f_osc_row(w_wv(20 downto 14));
            v_lv := signed(resize(w_wv(20 downto 14), 10)) - 67 - signed(resize(r_ls(12 downto 8), 10));
            if v_lv < 0 then v_lv := v_lv + 23; end if;
            if v_lv >= 9 and v_lv < 14 then o_lg <= '1'; else o_lg <= '0'; end if;

            -- stage 2: column sums and column-done events
            if s_osc = '1' then
                v_y := o_gy(9 downto 2);
                v_cu := signed(resize(o_gu(9 downto 3), 9)) - 64; v_cv := signed(resize(o_gv(9 downto 3), 9)) - 64;
            else
                v_y := w_y1;
                v_cu := signed(resize(w_u1, 9)) - 64; v_cv := signed(resize(w_v1, 9)) - 64;
            end if;
            w_dn <= '0';
            if (w_win = '1' and w_nw = '1' and w_winp = '1') or (w_win = '0' and w_winp = '1') then
                w_dn <= '1'; w_dc <= w_colq; w_dhs <= w_hs;
            end if;
            if w_win = '1' then w_colq <= w_wu(18 downto 12); end if;
            if w_nw = '1' then
                if w_smp = '1' then w_hs <= resize(v_y, 13); else w_hs <= (others => '0'); end if;
                a_ra <= w_wu(18 downto 12);
            elsif w_win = '1' and w_smp = '1' then
                w_hs <= w_hs + resize(v_y, 13);
            end if;
            -- chroma: one sample (the first of each 4 x 4 block), in units of 32 codes
            c_we <= '0';
            if w_nw = '1' and w_wu(13 downto 12) = "00" and w_lchr = '1' and w_rok = '1' then
                for k in 0 to 1 loop
                    if k = 0 then v_ca := v_cu; else v_ca := v_cv; end if;
                    v_ca := shift_right(v_ca + 2, 2);
                    if v_ca < -8 then v_ca := to_signed(-8, 9); elsif v_ca > 7 then v_ca := to_signed(7, 9); end if;
                    if k = 0 then v_c4u := v_ca(3 downto 0); else v_c4v := v_ca(3 downto 0); end if;
                end loop;
                c_we <= '1'; c_wa <= w_row(6 downto 2) & w_wu(18 downto 14);
                c_wd <= std_logic_vector(v_c4u) & std_logic_vector(v_c4v);
            end if;
            w_aq <= unsigned(a_q);
            -- stage 3: add the earlier lines' partial sum
            w_sv7 <= '0';
            if w_dn = '1' and w_lsmp = '1' and w_rok = '1' then
                w_sv7 <= '1'; w_sc7 <= w_dc;
                if w_lfst = '1' then w_sum <= resize(w_dhs, 16); else w_sum <= w_aq + resize(w_dhs, 16); end if;
            end if;
            -- stage 4: partial sum back, or the finished store pixel
            a_we <= '0'; b_we <= (others => '0'); w_st8 <= '0';
            if w_sv7 = '1' then
                if w_llst = '1' then
                    w_avg8 <= resize(shift_right(w_sum, to_integer(s_na) + to_integer(s_nb)), 9);
                    w_st8 <= '1'; w_sc8 <= w_sc7;
                else
                    a_we <= '1'; a_wa <= w_sc7; a_wd <= std_logic_vector(w_sum);
                end if;
            end if;
            -- stage 5: quantize to 4 bits and write the store
            if w_st8 = '1' then
                v_q := signed(resize(w_avg8, 18)) - 16;
                v_q := shift_right(v_q + shift_right(v_q, 3), 4);                             -- x 9/128
                if v_q < 0 then b_wd <= "0000";
                elsif v_q > 15 then b_wd <= "1111";
                else b_wd <= std_logic_vector(v_q(3 downto 0)); end if;
                b_wa <= w_row(6 downto 1) & w_sc8(6 downto 1);
                case std_logic_vector'(w_row(0) & w_sc8(0)) is
                    when "00" => b_we <= "0001";
                    when "01" => b_we <= "0010";
                    when "10" => b_we <= "0100";
                    when others => b_we <= "1000";
                end case;
            end if;
            -- line state: the next row advances at the source avid fall (from its own register),
            -- is committed at the rise (the commit wins over the zone, which is still high for one
            -- clock at a field's first rise), and is reset in the zone
            if data_in.avid = '1' and s_avq = '0' then
                w_wv <= w_wvn;
                w_row <= w_wvn(20 downto 14);
                w_rok <= not w_wvn(21);
                if w_wvn(20 downto 14) /= w_rown then
                    v_ad := (others => '0');
                elsif w_lrkn = 15 then
                    v_ad := to_unsigned(15, 5);
                else
                    v_ad := resize(w_lrkn + 1, 5);
                end if;
                w_lrkn <= v_ad(3 downto 0); w_rown <= w_wvn(20 downto 14);
                if v_ad(3 downto 0) < shift_left(to_unsigned(1, 4), to_integer(s_nb)) then w_lsmp <= '1'; else w_lsmp <= '0'; end if;
                if v_ad = 0 then w_lfst <= '1'; else w_lfst <= '0'; end if;
                if v_ad(3 downto 0) = shift_left(to_unsigned(1, 4), to_integer(s_nb)) - 1 then w_llst <= '1'; else w_llst <= '0'; end if;
                if v_ad = 0 and w_wvn(15 downto 14) = "00" then w_lchr <= '1'; else w_lchr <= '0'; end if;
            elsif s_zone = '1' then
                w_wvn <= (others => '0'); w_lrkn <= (others => '0'); w_rown <= (others => '1');
            elsif data_in.avid = '0' and s_avq = '1' then
                w_wvn <= w_wvn + resize(s_kr, 22);
            end if;
        end if;
    end process p_wr;

    data_out.y       <= std_logic_vector(s_y22);
    data_out.u       <= std_logic_vector(s_u22);
    data_out.v       <= std_logic_vector(s_v22);
    data_out.hsync_n <= s_do(3);
    data_out.vsync_n <= s_do(2);
    data_out.field_n <= s_do(1);
    data_out.avid    <= s_do(0);

end architecture triplicate;
