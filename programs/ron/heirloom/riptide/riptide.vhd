-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Riptide v0.5.0 -- Heirloom #15, rebuilt from WIN_20230309_00_28_35.
--
-- The photo: a band of psychedelic swirl (rows 327..712 of 1080) MIRRORED top/bottom about the
-- centre line, cut into strips by glitch zones of torn lines, with torn edges, between two blue
-- "fingerprint" textures of streaky beads and tilted stripes with crimson-beaded columns every 375 px.
-- Every colour sits on the RGB-cube faces (HSV at full saturation): one hue/value colourizer
-- draws all of it.
--
-- The program:
--   * band: a 256 x 32 knot grid of (hue H6, value V4) fitted to the photo's top half-band
--     (riptide_pkg, even / odd knot rows), folded about the axis; a micro-coded per-line engine
--     (one two-phase 22-bit ALU, kicked at each line start, computing the NEXT line) lerps the
--     line's two knot rows into a row RAM (hue on the shortest path); the pixel path lerps the row
--     along a per-line column DDA (strip slides in the glitch zones, tear jitter, torn edges);
--   * texture: the same engine fills the row from lattice noise (one pipelined 24-bit hash: a fine
--     8 px x 2.4 line lattice and a coarse 16 x 8 one, sheared), a tilted 10.8-line stripe and a
--     32-entry zone table over the 375-px column period (mean hue, hue spread, mean / spread of
--     value); where the hue noise passes the zone's threshold the column becomes a crimson bead;
--   * pixel: hue + Hue knob + drift + etch, value x gain (float), a 256-entry colour ROM
--     (hue -> Y, Cr, Cb) and three radix-4 products.
--
--   K1  Band     band half-height 24 .. 540 lines (193 = the photo); past 193 the field refolds
--   K2  Etch     the picture (Video) or the oscillator (Osc) rotates the hue
--   K3  Hue      colourizer phase (0 = the photo)
--   K4  Speed    bipolar: hue drift, strip slides, stripe crawl, column drift; centre still
--   K5  Gain     value gain 0.4 .. 2.5 (dark ink .. posterized plateaus)
--   K6  Ridges   fingerprint stripe spacing 0.37 .. 2.7 x
--   S7  Mirror   Fold (the photo) / Repeat (strips stacked, no reflection)
--   S8  Quad     the band also mirrored left / right
--   S9  Palette  Photo / Opposite (hue + 180)
--   S10 Twin     one band / two bands
--   S11 Source   Video / Osc
--   P12 Rip      0 = clean mirrored band .. photo tears (default) .. 100 % strips sliding apart,
--                every line torn, edges shredded into the texture
--
-- Geometry in virtual 1080-line units (SD shows the centre 1440 of the 1920 virtual columns).
-- Per-field constants on a 9-slot serial multiplier in the vblank zone.  Generated avid; output
-- conditioning (chroma 9-tap box, luma [1 2 1] and 13/16) before the blanking gate.
-- Latency 28 clocks in every mode.  Colours in the hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.riptide_pkg.all;
use work.all;

architecture riptide of program_top is
    constant C_LAT : integer := 28;

    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
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
    -- glide: ease 1/8 of the gap, +-3 hold, single steps when close (d = target - g)
    function f_glide(g : unsigned(9 downto 0); d : signed(10 downto 0)) return unsigned is
        variable r : signed(10 downto 0);
    begin
        if d >= 8 or d <= -8 then
            r := signed('0' & g) + shift_right(d, 3);
        elsif d >= 4 then
            r := signed('0' & g) + 1;
        elsif d <= -4 then
            r := signed('0' & g) - 1;
        else
            r := signed('0' & g);
        end if;
        return unsigned(r(9 downto 0));
    end function;
    -- triangle of a 16-bit phase: 0 .. 32767
    function f_tri16(p : unsigned(15 downto 0)) return unsigned is
    begin
        if p(15) = '1' then return not p(14 downto 0); else return p(14 downto 0); end if;
    end function;
    -- leading-one float of a 10-bit value: value ~ M << e (M = the top 4 bits, exact below 16)
    function f_fe(v : unsigned(9 downto 0)) return unsigned is
    begin
        if v(9) = '1' then return to_unsigned(6, 3);
        elsif v(8) = '1' then return to_unsigned(5, 3);
        elsif v(7) = '1' then return to_unsigned(4, 3);
        elsif v(6) = '1' then return to_unsigned(3, 3);
        elsif v(5) = '1' then return to_unsigned(2, 3);
        elsif v(4) = '1' then return to_unsigned(1, 3);
        else return to_unsigned(0, 3); end if;
    end function;
    function f_fm(v : unsigned(9 downto 0)) return unsigned is
    begin
        if v(9) = '1' then return v(9 downto 6);
        elsif v(8) = '1' then return v(8 downto 5);
        elsif v(7) = '1' then return v(7 downto 4);
        elsif v(6) = '1' then return v(6 downto 3);
        elsif v(5) = '1' then return v(5 downto 2);
        elsif v(4) = '1' then return v(4 downto 1);
        else return v(3 downto 0); end if;
    end function;
    -- signed x times an unsigned m by conditional adds
    function f_mul(x : signed; m : unsigned) return signed is
        variable r : signed(x'length + m'length - 1 downto 0);
        variable xx : signed(x'length + m'length - 1 downto 0);
    begin
        r := (others => '0');
        xx := resize(x, r'length);
        for i in 0 to m'length - 1 loop
            if m(m'low + i) = '1' then r := r + shift_left(xx, i); end if;
        end loop;
        return r;
    end function;

    -- x times a 2-bit digit (0, x, 2x, 3x) given x3 = 3x
    function f_dig(x : signed; x3 : signed; d : unsigned(1 downto 0)) return signed is
    begin
        case d is
            when "00"   => return to_signed(0, x3'length);
            when "01"   => return resize(x, x3'length);
            when "10"   => return resize(shift_left(resize(x, x3'length), 1), x3'length);
            when others => return x3;
        end case;
    end function;

    -- lerp of two 8-bit values by a 2-bit fraction: lo, (3 lo + hi) / 4, (lo + hi) / 2, (lo + 3 hi) / 4
    function f_lerp4(lo : signed(7 downto 0); hi : signed(7 downto 0); f : unsigned(1 downto 0)) return signed is
        variable s1 : signed(10 downto 0);
    begin
        s1 := resize(lo, 11) + resize(hi, 11);
        case f is
            when "00"   => return resize(lo, 9);
            when "01"   => return resize(shift_right(s1 + shift_left(resize(lo, 11), 1), 2), 9);
            when "10"   => return resize(shift_right(s1, 1), 9);
            when others => return resize(shift_right(s1 + shift_left(resize(hi, 11), 1), 2), 9);
        end case;
    end function;

    type t_i16 is array (natural range <>) of integer;
    constant C_SPH0 : t_i16(0 to 6) := (0, 17733, 32331, 45893, 55281, 9387, 27119);
    -- source line r -> {glitch zone, strip index}
    type t_gz is array (0 to 255) of integer range 0 to 15;
    constant C_GZ : t_gz := (
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 8, 8, 8, 9, 9, 9, 9, 9, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 9, 9,
        9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2, 10, 10, 10, 10, 11, 11, 11, 11, 11, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 11, 11, 11, 11, 11, 12, 12, 12, 12, 4, 4,
        4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 5, 5, 13, 13,
        13, 13, 14, 14, 14, 14, 14, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
        6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6);

    ---------------------------------------------------------------------------
    -- memories
    ---------------------------------------------------------------------------
    signal s_ka_ea, s_ka_oa : unsigned(11 downto 0) := (others => '0');
    signal s_kae_q, s_kao_q : std_logic_vector(7 downto 0) := (others => '0');
    signal s_kbe_q, s_kbo_q : std_logic_vector(1 downto 0) := (others => '0');
    type t_row is array (0 to 255) of std_logic_vector(15 downto 0);
    signal ram_re, ram_ro : t_row := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of ram_re : signal is true;
    attribute no_rw_check of ram_ro : signal is true;
    signal s_rw_a : unsigned(7 downto 0) := (others => '0');
    signal s_rw_d : std_logic_vector(15 downto 0) := (others => '0');
    signal s_rw_e, s_rw_o : std_logic := '0';
    signal s_rr_e, s_rr_o : unsigned(7 downto 0) := (others => '0');
    signal s_rq_e, s_rq_o : std_logic_vector(15 downto 0) := (others => '0');
    type t_ring is array (0 to 255) of std_logic_vector(7 downto 0);
    signal ram_lu : t_ring := (others => x"80");
    attribute no_rw_check of ram_lu : signal is true;
    signal s_lu_wa, s_lu_ra : unsigned(7 downto 0) := (others => '0');
    signal s_lu_wd : std_logic_vector(7 downto 0) := x"80";
    signal s_lu_we : std_logic := '0';
    signal s_lu_q : std_logic_vector(7 downto 0) := x"80";
    -- sync delay ring {gav, field, vsync, hsync}
    type t_sring is array (0 to 255) of std_logic_vector(3 downto 0);
    signal ram_sy : t_sring := (others => "0111");
    attribute no_rw_check of ram_sy : signal is true;
    signal s_sy_wa : unsigned(7 downto 0) := (others => '0');
    signal s_sy_q, s_sy_r, s_sy_d1, s_sy_d2, s_sy_d3 : std_logic_vector(3 downto 0) := "0111";
    -- colourizer ROM
    signal s_cl_a : unsigned(7 downto 0) := (others => '0');
    signal s_cl_q : std_logic_vector(23 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- timing, controls, per-field constants
    ---------------------------------------------------------------------------
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone : std_logic := '0';
    signal s_zc : unsigned(10 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_g1, s_g2, s_g3, s_g5, s_g6, s_gp, s_gl_g : unsigned(9 downto 0) := (others => '0');
    signal s_gl_t : std_logic_vector(9 downto 0) := (others => '0');
    signal s_gl_r : unsigned(9 downto 0) := (others => '0');
    signal s_gl_d : signed(10 downto 0) := (others => '0');
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_rep, s_quad, s_opp, s_twin, s_src : std_logic := '0';
    signal s_sp : signed(9 downto 0) := (others => '0');            -- speed, -504 .. 503
    signal s_spm : unsigned(8 downto 0) := (others => '0');         -- |speed|
    signal s_hh : unsigned(17 downto 0) := to_unsigned(49408, 18);  -- band half-height, Q8 lines
    signal s_p2, s_p3 : unsigned(9 downto 0) := (others => '0');
    signal s_jit : unsigned(5 downto 0) := (others => '0');         -- glitch-zone jitter (px)
    signal s_jn : unsigned(6 downto 0) := (others => '0');          -- near-edge jitter (px)
    signal s_gjit : unsigned(4 downto 0) := (others => '0');        -- global jitter (px)
    signal s_ttear : unsigned(1 downto 0) := (others => '0');       -- texture tear (lattice columns)
    signal s_slide : unsigned(8 downto 0) := (others => '0');       -- strip slide amplitude (px)
    signal s_edge : unsigned(17 downto 0) := (others => '0');       -- torn edge depth, Q8 lines
    signal s_edgen : unsigned(17 downto 0) := (others => '0');      -- 0.7 x edge
    signal s_k5q : unsigned(9 downto 0) := (others => '0');
    signal s_gq : unsigned(9 downto 0) := to_unsigned(64, 10);      -- gain Q6
    signal s_gg : unsigned(17 downto 0) := (others => '0');
    signal s_eh : unsigned(5 downto 0) := (others => '0');          -- etch hue
    signal s_rowcl : unsigned(9 downto 0) := to_unsigned(32, 10);
    signal s_rowfl : unsigned(10 downto 0) := to_unsigned(107, 11);
    signal s_strp : unsigned(14 downto 0) := to_unsigned(6068, 15);
    signal s_strpl : unsigned(16 downto 0) := to_unsigned(6068, 17);
    signal s_mph : unsigned(15 downto 0) := (others => '0');        -- master strip phase
    signal s_th : unsigned(15 downto 0) := (others => '0');         -- hue drift accumulator
    signal s_th0 : unsigned(11 downto 0) := (others => '0');        -- field hue angle (4096 / turn)
    signal s_spt : unsigned(15 downto 0) := (others => '0');        -- stripe roll
    signal s_zdt : unsigned(23 downto 0) := (others => '0');        -- zone drift
    signal s_xsu : unsigned(11 downto 0) := (others => '0');        -- first pixel, half-px units
    signal s_bstep : unsigned(12 downto 0) := to_unsigned(546, 13); -- band column step / px (Q12)
    signal s_tstep : unsigned(12 downto 0) := to_unsigned(512, 13); -- texture column step / px
    signal s_jb4 : unsigned(9 downto 0) := (others => '0');         -- first lattice column of a texture row
    signal s_tcst : unsigned(11 downto 0) := (others => '0');
    signal s_ncol : unsigned(8 downto 0) := to_unsigned(250, 9);
    signal s_zl : unsigned(23 downto 0) := (others => '0');         -- zone phase of the first lattice column
    signal s_xcol, s_sca, s_scb : unsigned(15 downto 0) := (others => '0');
    signal s_sc0 : unsigned(15 downto 0) := (others => '0');        -- stripe phase offset of the first lattice column
    signal s_cx0 : unsigned(21 downto 0) := (others => '0');        -- band column of the first pixel (Q12)
    signal s_tyf0, s_tyc0 : unsigned(20 downto 0) := (others => '0');   -- lattice row shift of the first column
    -- field serial multiplier: P = A x B, B shifted right, accumulator shifted right (A 20 bits, B 16 bits)
    signal m_a : unsigned(19 downto 0) := (others => '0');
    signal m_b : unsigned(15 downto 0) := (others => '0');
    signal m_acc : unsigned(35 downto 0) := (others => '0');
    signal m_neg, m_ld, m_stp, m_st : std_logic := '0';
    signal m_slot : unsigned(5 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- generated avid
    ---------------------------------------------------------------------------
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    ---------------------------------------------------------------------------
    -- source side (oscillator, luma ring)
    ---------------------------------------------------------------------------
    signal s_ox : unsigned(9 downto 0) := (others => '0');
    signal s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(9 downto 0) := (others => '0');
    signal s_osum : unsigned(9 downto 0) := (others => '0');
    signal s_wcol : unsigned(7 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- line engine
    ---------------------------------------------------------------------------
    signal e_kick : std_logic := '0';
    signal e_st : unsigned(5 downto 0) := (others => '0');
    signal e_wb, e_last, s_rbank : std_logic := '0';
    signal e_t, r1, r2 : signed(21 downto 0) := (others => '0');
    signal e_rr : unsigned(16 downto 0) := (others => '0');         -- source row, Q8
    signal e_lfsr : unsigned(31 downto 0) := x"ACE1F00D";
    signal e_typ : unsigned(1 downto 0) := (others => '0');         -- 0 tex, 1 band, 2 near, 3 far
    signal e_gz, e_near, e_ts : std_logic := '0';
    signal e_si : unsigned(2 downto 0) := (others => '0');
    signal e_n1, e_n2 : signed(7 downto 0) := (others => '0');
    signal e_tear : signed(2 downto 0) := (others => '0');
    signal pm_x : signed(17 downto 0) := (others => '0');
    signal pm_m : unsigned(6 downto 0) := (others => '0');
    signal pm_c : unsigned(2 downto 0) := (others => '0');
    signal e_soff, e_j1, e_j2, e_off : signed(10 downto 0) := (others => '0');
    signal e_k0 : unsigned(4 downto 0) := (others => '0');
    signal e_fy : unsigned(3 downto 0) := (others => '0');
    signal e_yf, e_yc : unsigned(20 downto 0) := (others => '0');   -- texture lattice rows, Q8
    signal e_sl : unsigned(15 downto 0) := (others => '0');         -- stripe phase of the line
    signal e_zc : unsigned(23 downto 0) := (others => '0');         -- zone phase of the fill column
    signal e_scp : unsigned(15 downto 0) := (others => '0');        -- stripe phase of the fill column
    signal e_scw : unsigned(15 downto 0) := (others => '0');        -- the same + phase wander
    signal w_col : unsigned(7 downto 0) := (others => '0');          -- row RAM write column
    signal a_r, b_r : signed(21 downto 0) := (others => '0');
    signal cin_r, e_ph, e_kp : std_logic := '0';
    signal s_sl0 : unsigned(15 downto 0) := (others => '0');
    -- per-line handover (n_ computed for the next line, l_ in use)
    signal n_band, n_fold : std_logic := '0';
    signal n_cst : signed(21 downto 0) := (others => '0');
    signal n_cstp : unsigned(12 downto 0) := (others => '0');
    signal l_band, l_fold : std_logic := '0';
    signal l_cst : signed(21 downto 0) := (others => '0');
    signal l_cstp : unsigned(12 downto 0) := (others => '0');
    -- band column pipeline
    signal b_run : std_logic := '0';
    signal b_c : unsigned(8 downto 0) := (others => '0');
    signal b_v : std_logic_vector(1 to 6) := (others => '0');
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    signal b_ca : t_u8a(1 to 6) := (others => (others => '0'));
    signal b_ae, b_ao : std_logic_vector(7 downto 0) := (others => '0');
    signal b_be, b_bo : std_logic_vector(1 downto 0) := (others => '0');
    signal b4_h, b4_v : unsigned(7 downto 0) := (others => '0');
    signal b4_dh, b4_dh3 : signed(7 downto 0) := (others => '0');
    signal b4_dv, b4_dv3 : signed(6 downto 0) := (others => '0');
    signal b5_h : unsigned(7 downto 0) := (others => '0');
    signal b5_v : unsigned(8 downto 0) := (others => '0');
    -- texture column machine + hash pipe
    signal t_run, t_pro : std_logic := '0';
    signal t_j : unsigned(8 downto 0) := (others => '0');
    signal t_slot : unsigned(1 downto 0) := (others => '0');
    signal t_jf : unsigned(9 downto 0) := (others => '0');
    signal t_yf, t_yc : unsigned(20 downto 0) := (others => '0');
    signal h_v : std_logic_vector(0 to 4) := (others => '0');
    type t_u24a is array (natural range <>) of unsigned(23 downto 0);
    signal h_h : t_u24a(0 to 4) := (others => (others => '0'));
    type t_tag is array (natural range <>) of std_logic_vector(14 downto 0);
    signal h_tag : t_tag(0 to 4) := (others => (others => '0'));
    signal c_f0p, c_f0q : signed(7 downto 0) := (others => '0');
    signal c_c0p, c_c0q : signed(7 downto 0) := (others => '0');
    signal c_fp, c_fq : signed(8 downto 0) := (others => '0');
    signal c_cnp, c_cnq, c_cpp, c_cpq : signed(8 downto 0) := (others => '0');
    signal c_out, c_oddo : std_logic := '0';
    signal c_ocol : unsigned(7 downto 0) := (others => '0');
    signal c2_out : std_logic := '0';
    signal c2_col : unsigned(7 downto 0) := (others => '0');
    signal c2_p, c2_q : signed(11 downto 0) := (others => '0');
    -- texture colour pipeline (per lattice column)
    signal tv : std_logic_vector(1 to 6) := (others => '0');
    type t_u8b is array (natural range <>) of unsigned(7 downto 0);
    signal tc : t_u8b(1 to 6) := (others => (others => '0'));
    signal t1_p, t1_q : signed(6 downto 0) := (others => '0');
    signal t1_st : signed(6 downto 0) := (others => '0');
    signal t1_zi : unsigned(4 downto 0) := (others => '0');
    signal t2_zt : std_logic_vector(34 downto 0) := (others => '0');
    signal t2_p, t2_q, t2_p3, t2_q3 : signed(9 downto 0) := (others => '0');
    signal t2_st : signed(6 downto 0) := (others => '0');
    signal t3_ahl, t3_ahh, t3_avl, t3_avh : signed(13 downto 0) := (others => '0');
    signal t3_sh, t3_sv : signed(13 downto 0) := (others => '0');
    signal t3_mh, t3_mv : unsigned(7 downto 0) := (others => '0');
    signal t3_bd : std_logic := '0';
    signal t2_qr : signed(6 downto 0) := (others => '0');
    signal t4_ah, t4_av, t4_sh, t4_sv : signed(13 downto 0) := (others => '0');
    signal t4_mh, t4_mv : unsigned(7 downto 0) := (others => '0');
    signal t5_h : unsigned(7 downto 0) := (others => '0');
    signal t5_v : unsigned(7 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- pixel path
    ---------------------------------------------------------------------------
    signal s_col : signed(21 downto 0) := (others => '0');
    signal p1_c : unsigned(9 downto 0) := (others => '0');
    signal p1_f : unsigned(3 downto 0) := (others => '0');
    signal p2_c, p3_c : unsigned(8 downto 0) := (others => '0');
    type t_u4a is array (natural range <>) of unsigned(3 downto 0);
    signal p_f : t_u4a(2 to 7) := (others => (others => '0'));
    signal p_par : std_logic_vector(4 to 6) := (others => '0');
    signal p_bnd : std_logic_vector(1 to 20) := (others => '0');
    signal p6_e, p6_o : std_logic_vector(15 downto 0) := (others => '0');
    signal p7_h : unsigned(7 downto 0) := (others => '0');
    signal p7_v : unsigned(7 downto 0) := (others => '0');
    signal p7_dh : signed(7 downto 0) := (others => '0');
    signal p7_dv : signed(8 downto 0) := (others => '0');
    signal p8_h : unsigned(11 downto 0) := (others => '0');
    signal p8_v : unsigned(11 downto 0) := (others => '0');
    signal s_lu1 : signed(7 downto 0) := (others => '0');
    signal p9_a : unsigned(11 downto 0) := (others => '0');
    signal p9_v : unsigned(11 downto 0) := (others => '0');
    signal p9_he : signed(13 downto 0) := (others => '0');
    signal p10_a : unsigned(11 downto 0) := (others => '0');
    signal p10_v : unsigned(11 downto 0) := (others => '0');
    signal p10_g : unsigned(9 downto 0) := (others => '0');
    signal p11_v : unsigned(11 downto 0) := (others => '0');
    signal p11_ge : unsigned(2 downto 0) := (others => '0');
    signal p11_gm : unsigned(3 downto 0) := (others => '0');
    signal p12_vg : unsigned(15 downto 0) := (others => '0');
    signal p12_ge : unsigned(2 downto 0) := (others => '0');
    signal p13_cl : std_logic_vector(23 downto 0) := (others => '0');
    signal p13_v : unsigned(7 downto 0) := (others => '0');
    signal p14_y, p14_u, p14_w : signed(8 downto 0) := (others => '0');
    signal p14_y3, p14_u3, p14_w3 : signed(10 downto 0) := (others => '0');
    signal p14_v : unsigned(7 downto 0) := (others => '0');
    signal p15_yl, p15_yh, p15_ul, p15_uh, p15_wl, p15_wh : signed(14 downto 0) := (others => '0');
    signal p16_y, p16_u, p16_w : signed(18 downto 0) := (others => '0');
    signal p17_y, p17_u, p17_v, p18_y, p18_u, p18_v, p19_y, p19_u, p19_v : unsigned(9 downto 0) := (others => '0');
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_rx : unsigned(7 downto 0) := (others => '0');
    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_cring is array (0 to 255) of std_logic_vector(15 downto 0);
    signal ram_cd : t_cring := (others => x"8080");
    attribute no_rw_check of ram_cd : signal is true;
    signal s_cd_wa : unsigned(7 downto 0) := (others => '0');
    signal s_cd_q, s_cd_r : std_logic_vector(15 downto 0) := x"8080";
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(11 downto 0) := to_unsigned(1024, 12);
    signal s_y13, s_y14, s_y15, s_y16, s_y17, s_y18x, s_yf18 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- short FF taps of the generated avid (the long delay is the sync ring)
    signal s_gav_sr : std_logic_vector(0 to 8) := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_kae : process(clk) begin if rising_edge(clk) then s_kae_q <= C_RIP_KAE(to_integer(s_ka_ea)); end if; end process;
    p_kao : process(clk) begin if rising_edge(clk) then s_kao_q <= C_RIP_KAO(to_integer(s_ka_oa)); end if; end process;
    p_kbe : process(clk) begin if rising_edge(clk) then s_kbe_q <= C_RIP_KBE(to_integer(s_ka_ea)); end if; end process;
    p_kbo : process(clk) begin if rising_edge(clk) then s_kbo_q <= C_RIP_KBO(to_integer(s_ka_oa)); end if; end process;
    p_col : process(clk) begin if rising_edge(clk) then s_cl_q <= C_RIP_COL(to_integer(s_cl_a)); end if; end process;

    p_rowe : process(clk)
    begin
        if rising_edge(clk) then
            s_rq_e <= ram_re(to_integer(s_rr_e));
            if s_rw_e = '1' then ram_re(to_integer(s_rw_a)) <= s_rw_d; end if;
        end if;
    end process p_rowe;

    p_rowo : process(clk)
    begin
        if rising_edge(clk) then
            s_rq_o <= ram_ro(to_integer(s_rr_o));
            if s_rw_o = '1' then ram_ro(to_integer(s_rw_a)) <= s_rw_d; end if;
        end if;
    end process p_rowo;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_lu_q <= ram_lu(to_integer(s_lu_ra));
            if s_lu_we = '1' then ram_lu(to_integer(s_lu_wa)) <= s_lu_wd; end if;
        end if;
    end process p_ring;

    -- sync delay ring: written every clock, read C_LAT - 5 entries behind (+ EBR register + fabric register)
    p_sring : process(clk)
    begin
        if rising_edge(clk) then
            s_sy_q <= ram_sy(to_integer(s_sy_wa - to_unsigned(C_LAT - 5, 8)));
            ram_sy(to_integer(s_sy_wa)) <= g_avid & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            s_sy_wa <= s_sy_wa + 1;
        end if;
    end process p_sring;

    -- chroma delay ring for the 9-tap box (u12, v12 at 8 bits)
    p_cring : process(clk)
    begin
        if rising_edge(clk) then
            s_cd_q <= ram_cd(to_integer(s_cd_wa - 6));
            ram_cd(to_integer(s_cd_wa)) <= std_logic_vector(s_u12(9 downto 2)) & std_logic_vector(s_v12(9 downto 2));
            s_cd_wa <= s_cd_wa + 1;
        end if;
    end process p_cring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one serial multiplier)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s : signed(10 downto 0);
        variable v_h : unsigned(13 downto 0);
        variable v_j : unsigned(19 downto 0);
        variable v_slot : integer range 0 to 63;
        variable v_r : unsigned(35 downto 0);
        variable v_w : unsigned(13 downto 0);
        variable v_x : signed(14 downto 0);
        variable v_gi : integer range 0 to 15;
        variable v_sum : unsigned(20 downto 0);
        variable v_t : unsigned(14 downto 0);
        variable v_i : integer range 0 to 7;
        variable v_q : unsigned(31 downto 0);
    begin
        if rising_edge(clk) then
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            if s_zone = '0' then
                s_zc <= (others => '0');
            elsif s_zc /= 2047 then
                s_zc <= s_zc + 1;
            end if;

            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- zone clock 1: width, switches, speed
            if s_zone = '1' and s_zc = 1 then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_rep  <= registers_in(6)(0);
                s_quad <= registers_in(6)(1);
                s_opp  <= registers_in(6)(2);
                s_twin <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                v_s := signed(resize(unsigned(registers_in(3)), 11)) - 512;      -- K4 Speed
                if v_s > -9 and v_s < 9 then v_s := (others => '0');
                elsif v_s > 0 then v_s := v_s - 8;
                else v_s := v_s + 8; end if;
                s_sp <= v_s(9 downto 0);
                if v_s < 0 then s_spm <= unsigned(resize(-v_s, 9)); else s_spm <= unsigned(v_s(8 downto 0)); end if;
            end if;
            -- zone clocks 4 .. 27: glides, four clocks per channel on one shared datapath (channel = zc(4:2) 1..6)
            if s_zone = '1' and s_zc(10 downto 5) = 0 and s_zc(4 downto 2) /= 0 and s_zc(4 downto 2) /= 7 then
                case to_integer(s_zc(1 downto 0)) is
                    when 0 =>
                        case to_integer(s_zc(4 downto 2)) is
                            when 1      => s_gl_g <= s_g1; s_gl_t <= registers_in(0);
                            when 2      => s_gl_g <= s_g2; s_gl_t <= registers_in(1);
                            when 3      => s_gl_g <= s_g3; s_gl_t <= registers_in(2);
                            when 4      => s_gl_g <= s_g5; s_gl_t <= registers_in(4);
                            when 5      => s_gl_g <= s_g6; s_gl_t <= registers_in(5);
                            when others => s_gl_g <= s_gp; s_gl_t <= registers_in(7);
                        end case;
                    when 1 =>
                        s_gl_d <= signed('0' & unsigned(s_gl_t)) - signed('0' & s_gl_g);
                    when 2 =>
                        if s_warm = 1 then s_gl_r <= unsigned(s_gl_t); else s_gl_r <= f_glide(s_gl_g, s_gl_d); end if;
                    when others =>
                        case to_integer(s_zc(4 downto 2)) is
                            when 1      => s_g1 <= s_gl_r;
                            when 2      => s_g2 <= s_gl_r;
                            when 3      => s_g3 <= s_gl_r;
                            when 4      => s_g5 <= s_gl_r;
                            when 5      => s_g6 <= s_gl_r;
                            when others => s_gp <= s_gl_r;
                        end case;
                end case;
            end if;
            -- zone clock 28: geometry of the mode; grain table; tear
            if s_zone = '1' and s_zc = 28 then
                case to_integer(s_xs) is
                    when 4      => v_w := resize(s_w & "00", 14);
                    when 3      => v_w := resize(s_w, 14) + resize(s_w & '0', 14);
                    when others => v_w := resize(s_w & '0', 14);
                end case;
                v_x := to_signed(3840, 15) - signed('0' & v_w);
                if v_x < 0 then v_x := (others => '0'); end if;
                s_xsu <= unsigned(v_x(12 downto 1));
                case to_integer(s_xs) is
                    when 4      => s_bstep <= to_unsigned(1092, 13); s_tstep <= to_unsigned(1024, 13);
                    when 3      => s_bstep <= to_unsigned(819, 13);  s_tstep <= to_unsigned(768, 13);
                    when others => s_bstep <= to_unsigned(546, 13);  s_tstep <= to_unsigned(512, 13);
                end case;
                -- texture columns (8 virtual px): visible / 16 + 2, + 8 for the tear margin
                s_ncol <= resize(v_w(13 downto 4), 9) + 10;
                v_gi := to_integer(s_g6(9 downto 6));
                s_strp <= to_unsigned(C_RIP_GSTRP(v_gi), 15);
                if s_gp(9) = '1' then s_ttear <= s_gp(8 downto 7); else s_ttear <= (others => '0'); end if;
                -- speed: hue drift sp / 2, stripe roll 4 sp, zone drift 64 sp, strips 1.625 sp per field
                s_th <= s_th + unsigned(resize(shift_right(s_sp, 1), 16));
                s_spt <= s_spt + unsigned(shift_left(resize(s_sp, 16), 2));
                s_zdt <= s_zdt + unsigned(shift_left(resize(s_sp, 24), 6));
                v_x := resize(s_sp, 15) + shift_right(resize(s_sp, 15), 1) + shift_right(resize(s_sp, 15), 3);
                s_mph <= s_mph + unsigned(resize(v_x, 16));
            end if;

            -- zone clocks 29 .. 34: direct per-field terms of the geometry and the controls
            if s_zone = '1' and s_zc = 29 then
                s_jb4 <= resize(s_xsu(11 downto 4), 10) - 4; s_tcst <= s_xsu(3 downto 0) & x"00";   -- texture column of the first pixel
                s_cx0 <= shift_left(resize(s_xsu, 22), 8) + shift_left(resize(s_xsu, 22), 4) + resize(s_xsu, 22) + resize(s_xsu(11 downto 4), 22);
                s_eh <= s_g2(9 downto 4);
                -- strips slide apart in the upper half of P12: 300 px x (2p - 1)
                if s_gp(9) = '1' then
                    v_h := resize(s_gp(8 downto 0) & '0', 14);
                    v_j := shift_left(resize(v_h, 20), 8) + shift_left(resize(v_h, 20), 5) + shift_left(resize(v_h, 20), 3) + shift_left(resize(v_h, 20), 2);
                    s_slide <= v_j(18 downto 10);
                    v_j := shift_left(resize(v_h, 20), 5) - shift_left(resize(v_h, 20), 1);            -- 30 x
                    s_gjit <= v_j(14 downto 10);
                else
                    s_slide <= (others => '0'); s_gjit <= (others => '0');
                end if;
            end if;
            if s_zone = '1' and s_zc = 30 then
                s_xcol <= shift_left(resize(s_xsu(11 downto 4), 16), 3);     -- virtual x of lattice column jb4 + 4
            end if;
            if s_zone = '1' and s_zc = 31 then
                s_sca <= shift_left(resize(s_xcol, 16), 8) + shift_left(resize(s_xcol, 16), 6);
                s_scb <= shift_left(resize(s_xcol, 16), 4) + resize(s_xcol, 16);
            end if;
            if s_zone = '1' and s_zc = 32 then
                s_sc0 <= to_unsigned(0, 16) - s_sca - s_scb;                                       -- -337 x
            end if;

            -- serial multiplier: slot = zc / 32 (2 .. 10); A at sub 0, 16 steps, result at sub 20
            m_slot <= s_zc(10 downto 5);
            if s_zone = '1' and s_zc(4 downto 0) = 0 and s_zc(10 downto 5) >= 2 and s_zc(10 downto 5) <= 10 then
                m_ld <= '1'; else m_ld <= '0'; end if;
            if s_zone = '1' and s_zc(4 downto 0) >= 1 and s_zc(4 downto 0) <= 16 then
                m_stp <= '1'; else m_stp <= '0'; end if;
            if s_zone = '1' and s_zc(4 downto 0) = 20 and s_zc(10 downto 5) >= 2 and s_zc(10 downto 5) <= 10 then
                m_st <= '1'; else m_st <= '0'; end if;
            v_slot := to_integer(m_slot);
            if m_ld = '1' then
                m_acc <= (others => '0');
                case v_slot is
                    when 2      => m_a <= resize(s_g1, 20);  m_b <= resize(s_g1, 16);
                    when 3      => m_a <= resize(s_gp, 20);  m_b <= resize(s_gp, 16);
                    when 4      => m_a <= resize(s_g5, 20);  m_b <= resize(s_g5, 16);
                    when 5      => m_a <= to_unsigned(C_RIP_ZPX, 20); m_b <= s_xcol;
                    when 6      => m_a <= to_unsigned(C_RIP_SHF, 20); m_b <= resize(s_jb4, 16);
                    when 7      => m_a <= to_unsigned(C_RIP_SHC, 20); m_b <= resize(s_jb4(9 downto 1), 16);
                    when 8      => m_a <= to_unsigned(C_RIP_ROWF, 20); m_b <= resize(s_ys, 16);
                    when 9      => m_a <= to_unsigned(C_RIP_ROWC, 20); m_b <= resize(s_ys, 16);
                    when others => m_a <= resize(s_strp, 20); m_b <= resize(s_ys, 16);
                end case;
            elsif m_stp = '1' then
                if m_b(0) = '1' then v_sum := ('0' & m_acc(35 downto 16)) + ('0' & m_a);
                else v_sum := '0' & m_acc(35 downto 16); end if;
                m_acc <= v_sum & m_acc(15 downto 1);
                m_b <= '0' & m_b(15 downto 1);
            elsif m_st = '1' then
                v_r := m_acc;
                case v_slot is
                    when 2      => s_hh <= to_unsigned(6144, 18) + resize(v_r(19 downto 3), 18) + resize(v_r(19 downto 10), 18) + resize(v_r(19 downto 11), 18);
                    when 3      => s_p2 <= v_r(19 downto 10);
                    when 4      => s_k5q <= v_r(19 downto 10);
                    when 5      => s_zl <= v_r(23 downto 0) + s_zdt - to_unsigned(32 * C_RIP_ZPX, 24);
                    when 6      => s_tyf0 <= to_unsigned(0, 21) - v_r(20 downto 0);
                    when 7      => s_tyc0 <= to_unsigned(0, 21) - v_r(20 downto 0);
                    when 8      => s_rowfl <= v_r(18 downto 8);
                    when 9      => s_rowcl <= resize(v_r(17 downto 8), 10);
                    when others => s_strpl <= v_r(24 downto 8);
                end case;
            end if;
            -- terms from the products
            if s_zone = '1' and s_zc = 380 then
                v_j := shift_left(resize(s_p2, 20), 5) + shift_left(resize(s_p2, 20), 3);              -- 40 p2
                s_jit <= v_j(15 downto 10);
                s_jn <= to_unsigned(2, 7) + resize(v_j(15 downto 10), 7) + resize(v_j(15 downto 11), 7);
                -- torn edge depth 64 p2 (Q8 lines), near part 3/4 of it
                s_edge <= resize(s_p2 & "000000", 18);
                s_edgen <= resize(s_p2 & "000000", 18) - resize(s_p2 & "0000", 18);
                v_j := shift_left(resize(s_k5q, 20), 7) + shift_left(resize(s_k5q, 20), 2) + shift_left(resize(s_k5q, 20), 1);   -- 134 k5q
                s_gq <= to_unsigned(26, 10) + v_j(19 downto 10);
            end if;
            if s_zone = '1' and s_zc = 383 then
                s_sl0 <= s_spt + s_sc0;
                v_q := resize(s_g3 & "000000", 32) + resize(s_th, 32);
                if s_opp = '1' then v_q := v_q + 32768; end if;
                s_th0 <= v_q(15 downto 4);
            end if;

            -- avid taps and the sync ring outputs
            s_gav_sr(0) <= g_avid;
            for i in 1 to 8 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
            s_sy_r <= s_sy_q; s_sy_d1 <= s_sy_r; s_sy_d2 <= s_sy_d1; s_sy_d3 <= s_sy_d2;
            g_wm1 <= s_w - 1;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Generated avid
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
    -- Source side: oscillator and the luma ring (8-bit luma / 4)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_a, v_b : unsigned(9 downto 0);
        variable v_ta, v_tb : unsigned(8 downto 0);
        variable v_l : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then s_ox <= s_ox + 3; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 4; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zone = '1' and s_zc = 1 then s_ot <= s_ot + 5; end if;
            -- Osc: two slow triangles summed, small (+-0.08 of the swing round mid grey)
            v_a := s_ox + s_ot; v_b := s_oy - s_ot;
            if v_a(9) = '1' then v_ta := not v_a(8 downto 0); else v_ta := v_a(8 downto 0); end if;
            if v_b(9) = '1' then v_tb := not v_b(8 downto 0); else v_tb := v_b(8 downto 0); end if;
            s_osum <= resize(v_ta, 10) + resize(v_tb, 10);
            v_l := signed(resize(s_osum, 12)) - 512;
            s_lu_we <= data_in.avid;
            s_lu_wa <= s_wcol;
            if s_src = '1' then
                -- 128 + (sum - 512) * 72 / 2048
                s_lu_wd <= std_logic_vector(to_unsigned(128, 8) + unsigned(resize(shift_right(v_l * 9, 8), 8)));
            else
                s_lu_wd <= data_in.y(9 downto 2);
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: the next line's row into the row RAM (no multiplier: products are per field)
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_a, v_b, v_res : signed(21 downto 0);
        variable v_res2 : signed(22 downto 0);
        variable v_inv, v_cin : std_logic;
        variable v_st : integer range 0 to 63;
        variable v_g : integer range 0 to 15;
        variable v_t : unsigned(14 downto 0);
        variable v_tr : signed(2 downto 0);
        variable v_h6l, v_h6h, v_d6 : unsigned(5 downto 0);
        variable v_v4l, v_v4h : unsigned(3 downto 0);
        variable v_dv : signed(4 downto 0);
        variable v_m : signed(11 downto 0);
        variable v_hk : unsigned(23 downto 0);
        variable v_h : unsigned(23 downto 0);
        variable v_kind : std_logic_vector(1 downto 0);
        variable v_fr2 : unsigned(1 downto 0);
        variable v_p, v_qq, v_l : signed(11 downto 0);
        variable v_cc : unsigned(9 downto 0);
        variable v_kr : unsigned(12 downto 0);
        variable v_bp, v_bq : signed(7 downto 0);
        variable v_sti : unsigned(15 downto 0);
        variable v_bh : signed(13 downto 0);
        variable v_tv : signed(11 downto 0);
        variable v_v8 : unsigned(8 downto 0);
        variable v_hn : signed(9 downto 0);
        variable v_st7 : signed(6 downto 0);
        variable v_hs, v_vs : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            -- kicks: once in the zone for line 0, then at every line start
            e_kick <= '0';
            if s_zone = '1' and s_zc = 1320 then
                if s_twin = '1' then e_t <= to_signed(128 - (269 * 256 + 128), 22);
                else e_t <= to_signed(128 - (519 * 256 + 128), 22); end if;
                e_yf <= s_tyf0; e_yc <= s_tyc0;                  -- lattice rows of the first texture column
                e_sl <= s_sl0;                                    -- stripe phase of the first texture column
                e_kick <= '1';
            elsif g_start = '1' then
                e_kick <= '1';
            end if;
            if e_kick = '1' then e_kp <= '1'; end if;
            if g_start = '1' then
                l_band <= n_band; l_fold <= n_fold; l_cst <= n_cst; l_cstp <= n_cstp;
                s_rbank <= e_last;
            end if;

            -------------------------------------------------------------------
            -- line program: one 22-bit ALU, res = a + (b xor inv) + cin, one destination per state
            -------------------------------------------------------------------
            v_a := (others => '0'); v_b := (others => '0'); v_inv := '0'; v_cin := '0';
            v_st := to_integer(e_st);
            case v_st is
                when 0  => v_a := e_t; v_b := to_signed(69120, 22); v_inv := '1'; v_cin := '1';
                when 1  => v_a := e_t; if s_twin = '1' and r1 >= 0 then v_b := to_signed(138240, 22); end if; v_inv := '1'; v_cin := '1';
                when 2  => if r1 < 0 then v_b := r1; v_inv := '1'; v_cin := '1'; else v_a := r1; end if;
                when 3  => v_a := signed(resize(s_hh, 22)) - 128; v_b := r1; v_inv := '1'; v_cin := '1';
                when 4  => if r1 >= 0 then
                               v_a := r1; if s_rep = '1' then v_b := to_signed(49408, 22); else v_b := to_signed(98816, 22); end if;
                           else v_b := r1; end if;
                           v_inv := '1'; v_cin := '1';
                when 5  => if r2 >= 0 then v_a := r2; else v_a := r1; end if;
                when 6  => v_a := r1; if s_rep = '1' then v_b := to_signed(49408, 22); else v_b := to_signed(49281, 22); end if; v_inv := '1'; v_cin := '1';
                when 7  => if r2 >= 0 then
                               if s_rep = '1' then v_a := r2; else v_a := to_signed(98560, 22); v_b := r1; v_inv := '1'; v_cin := '1'; end if;
                           else v_a := r1; end if;
                when 8  => v_a := signed(resize(shift_right(e_rr, 3), 22)); v_b := signed(resize(shift_right(e_rr, 5), 22));
                when 9  => v_a := r1; v_b := signed(resize(shift_right(e_rr, 8), 22));
                when 10 => v_a := r1; v_b := signed(resize(shift_right(e_rr, 9), 22));
                when 11 => -- strip phase: M(si) x master as two shifted terms
                           case to_integer(e_si) is
                               when 0 | 5  => v_a := signed(resize(shift_left(resize(s_mph, 18), 2), 22));
                               when 1      => v_a := signed(resize(shift_left(resize(s_mph, 18), 2), 22)); v_b := signed(resize(shift_left(resize(s_mph, 18), 1), 22));
                               when 2      => v_a := signed(resize(shift_left(resize(s_mph, 18), 1), 22)); v_b := signed(resize(s_mph, 22));
                               when 3 | 6  => v_a := signed(resize(shift_left(resize(s_mph, 18), 2), 22)); v_b := signed(resize(s_mph, 22));
                               when others => v_a := signed(resize(shift_left(resize(s_mph, 18), 3), 22)); v_b := signed(resize(s_mph, 22)); v_inv := '1'; v_cin := '1';
                           end case;
                when 12 => v_a := to_signed(C_SPH0(to_integer(e_si)), 22); v_b := r2; v_inv := e_si(0); v_cin := e_si(0);
                when 14 | 16 | 18 => v_a := r2; if pm_m(0) = '1' then v_b := resize(pm_x, 22); end if;
                when 20 => if e_typ = "01" then v_a := resize(e_soff, 22); v_b := resize(e_j2, 22); else v_b := resize(e_j1, 22); end if;
                when 21 => v_a := r1; if e_typ = "01" and e_gz = '1' then v_b := resize(e_j1, 22); end if;
                when 22 => v_a := signed(s_cx0); v_b := shift_left(resize(e_off, 22), 9);
                when 23 => v_a := r1; v_b := shift_left(resize(e_off, 22), 5);
                when 24 => v_a := r1; v_b := shift_left(resize(e_off, 22), 1);
                when 30 => v_a := r2; v_b := signed(resize(s_edgen, 22)); v_inv := '1';
                when 31 => v_a := r2; v_b := signed(resize(s_edge, 22)); v_inv := '1';
                when 40 => v_a := signed(resize(s_tcst, 22)); v_b := shift_left(resize(e_tear, 22) + 4, 12);
                when 50 => v_a := e_t; v_b := signed(resize(s_ys, 22));
                when 51 => v_a := signed(resize(e_yf, 22)); v_b := signed(resize(s_rowfl, 22));
                when 52 => v_a := signed(resize(e_yc, 22)); v_b := signed(resize(s_rowcl, 22));
                when 53 => v_a := signed(resize(e_sl, 22)); v_b := signed(resize(s_strpl, 22));
                when others => null;
            end case;
            if v_inv = '1' then v_b := not v_b; end if;
            a_r <= v_a; b_r <= v_b; cin_r <= v_cin;
            v_res2 := (a_r & '1') + (b_r & cin_r);
            v_res := v_res2(22 downto 1);

            -- two clocks per micro-op: phase 0 fetches the operands, phase 1 executes
            if e_ph = '0' then
                if v_st /= 0 or e_kp = '1' then e_ph <= '1'; end if;
            else
                e_ph <= '0';
            case v_st is
                when 0 =>
                    if e_kp = '1' then
                        e_kp <= '0';
                        e_wb <= not e_wb;
                        e_lfsr <= e_lfsr xor shift_left(e_lfsr, 13);
                        r1 <= v_res;
                        e_st <= to_unsigned(1, 6);
                    end if;
                when 1 =>
                    e_lfsr <= e_lfsr xor shift_right(e_lfsr, 17);
                    r1 <= v_res; e_st <= to_unsigned(2, 6);
                when 2 =>
                    e_lfsr <= e_lfsr xor shift_left(e_lfsr, 5);
                    r1 <= v_res; e_st <= to_unsigned(3, 6);
                when 3 =>
                    e_n1 <= resize(shift_right(signed('0' & e_lfsr(7 downto 0)) + signed('0' & e_lfsr(15 downto 8)) - 255, 1), 8);
                    e_n2 <= resize(shift_right(signed('0' & e_lfsr(23 downto 16)) + signed('0' & e_lfsr(31 downto 24)) - 255, 1), 8);
                    v_tr := signed(e_lfsr(26 downto 24));
                    case to_integer(s_ttear) is
                        when 0      => v_tr := (others => '0');
                        when 1      => v_tr := shift_right(v_tr, 2);
                        when 2      => v_tr := shift_right(v_tr, 1);
                        when others => null;
                    end case;
                    e_tear <= v_tr;
                    r1 <= v_res; e_st <= to_unsigned(4, 6);
                when 4 =>
                    r2 <= v_res;
                    if r1 >= 0 then e_typ <= "01"; e_st <= to_unsigned(5, 6);
                    else e_typ <= "00"; e_st <= to_unsigned(30, 6); end if;
                when 5 => r1 <= v_res; e_st <= to_unsigned(6, 6);
                when 6 => r2 <= v_res; e_st <= to_unsigned(7, 6);
                when 7 => r1 <= v_res; e_rr <= unsigned(v_res(16 downto 0)); e_st <= to_unsigned(8, 6);
                when 8 =>
                    v_g := C_GZ(to_integer(e_rr(15 downto 8)));
                    if v_g >= 8 then e_gz <= '1'; else e_gz <= '0'; end if;
                    e_si <= to_unsigned(v_g mod 8, 3);
                    r1 <= v_res; e_st <= to_unsigned(9, 6);
                when 9  => r1 <= v_res; e_st <= to_unsigned(10, 6);
                when 10 =>
                    if v_res(12 downto 8) >= 31 then e_k0 <= to_unsigned(30, 5); e_fy <= to_unsigned(15, 4);
                    else e_k0 <= unsigned(v_res(12 downto 8)); e_fy <= unsigned(v_res(7 downto 4)); end if;
                    e_st <= to_unsigned(11, 6);
                when 11 => r2 <= v_res; e_st <= to_unsigned(12, 6);
                when 12 => r2 <= v_res; e_st <= to_unsigned(13, 6);
                when 13 =>
                    -- strip triangle (7-bit magnitude, sign) x slide: serial multiply on the ALU
                    v_t := f_tri16(unsigned(r2(15 downto 0)));
                    if v_t(14) = '1' then pm_m <= v_t(13 downto 7); e_ts <= '0'; else pm_m <= not v_t(13 downto 7); e_ts <= '1'; end if;
                    pm_x <= signed(resize(s_slide, 18)); r2 <= (others => '0'); pm_c <= (others => '0');
                    e_st <= to_unsigned(14, 6);
                when 14 | 16 | 18 =>
                    r2 <= v_res; pm_x <= shift_left(pm_x, 1); pm_m <= '0' & pm_m(6 downto 1);
                    pm_c <= pm_c + 1;
                    if pm_c = 6 then e_st <= e_st + 1; end if;
                when 15 =>
                    if e_ts = '1' then e_soff <= -resize(shift_right(r2, 7), 11); else e_soff <= resize(shift_right(r2, 7), 11); end if;
                    pm_x <= resize(e_n1, 18); r2 <= (others => '0'); pm_c <= (others => '0');
                    if e_typ = "10" then pm_m <= s_jn; else pm_m <= '0' & s_jit; end if;
                    e_st <= to_unsigned(16, 6);
                when 17 =>
                    e_j1 <= resize(shift_right(r2, 6), 11);
                    pm_x <= resize(e_n2, 18); r2 <= (others => '0'); pm_c <= (others => '0'); pm_m <= "00" & s_gjit;
                    e_st <= to_unsigned(18, 6);
                when 19 =>
                    e_j2 <= resize(shift_right(r2, 6), 11);
                    e_st <= to_unsigned(20, 6);
                when 20 => r1 <= v_res; e_st <= to_unsigned(21, 6);
                when 21 => e_off <= resize(v_res, 11); e_st <= to_unsigned(22, 6);
                when 22 => r1 <= v_res; e_st <= to_unsigned(23, 6);
                when 23 => r1 <= v_res; e_st <= to_unsigned(24, 6);
                when 24 =>
                    n_cst <= v_res; n_cstp <= s_bstep; n_band <= '1'; n_fold <= '1';
                    b_run <= '1'; b_c <= (others => '0'); w_col <= (others => '0');
                    e_st <= to_unsigned(25, 6);
                when 25 =>
                    if b_run = '0' and b_v = "000000" then e_st <= to_unsigned(50, 6); end if;
                when 30 => r1 <= v_res; e_st <= to_unsigned(31, 6);
                when 31 =>
                    -- near: e <= edgen (e - edgen - 1 < 0), 235/256 of the lines
                    if r1 < 0 and e_lfsr(7 downto 0) < 235 then e_near <= '1'; else e_near <= '0'; end if;
                    r1 <= v_res; e_st <= to_unsigned(32, 6);
                when 32 =>
                    if e_near = '1' then
                        e_typ <= "10";
                        if r2 > 49152 then e_rr <= to_unsigned(49152, 17); else e_rr <= unsigned(r2(16 downto 0)); end if;
                        e_st <= to_unsigned(8, 6);
                    else
                        e_typ <= "00";
                        e_st <= to_unsigned(40, 6);
                    end if;
                when 40 =>
                    n_cst <= v_res; n_cstp <= s_tstep; n_band <= '0'; n_fold <= '0';
                    t_yf <= e_yf; t_yc <= e_yc; e_scp <= e_sl; e_zc <= s_zl;
                    t_jf <= s_jb4; t_j <= (others => '0'); t_pro <= '1'; t_slot <= (others => '0'); t_run <= '1'; w_col <= (others => '0');
                    e_st <= to_unsigned(44, 6);
                when 44 =>
                    if t_run = '0' and h_v = "00000" and c_out = '0' and c2_out = '0' and tv = "000000" then e_st <= to_unsigned(50, 6); end if;
                when 50 => e_last <= e_wb; e_t <= v_res; e_st <= to_unsigned(51, 6);
                when 51 => e_yf <= unsigned(v_res(20 downto 0)); e_st <= to_unsigned(52, 6);
                when 52 => e_yc <= unsigned(v_res(20 downto 0)); e_st <= to_unsigned(53, 6);
                when 53 => e_sl <= unsigned(v_res(15 downto 0)); e_st <= to_unsigned(0, 6);
                when others => e_st <= to_unsigned(0, 6);
            end case;
            end if;

            -------------------------------------------------------------------
            -- band row fill: c = 0 .. 255
            -------------------------------------------------------------------
            if b_run = '1' then
                s_ka_ea <= resize(shift_right(e_k0 + 1, 1), 4) & b_c(7 downto 0);
                s_ka_oa <= resize(shift_right(e_k0, 1), 4) & b_c(7 downto 0);
                if b_c = 255 then b_run <= '0'; end if;
                b_c <= b_c + 1;
            end if;
            b_v(1) <= b_run;
            for k in 2 to 6 loop b_v(k) <= b_v(k - 1); end loop;
            -- 2: ROM out (registered in the ROM process); 3: fabric
            b_ae <= s_kae_q; b_ao <= s_kao_q; b_be <= s_kbe_q; b_bo <= s_kbo_q;
            -- 4: knots lo / hi (lo = row k0): H6 = KA(7:2), V4 = KA(1:0) & KB; hue difference on the shortest path
            if e_k0(0) = '0' then
                v_h6l := unsigned(b_ae(7 downto 2)); v_v4l := (unsigned(b_ae(1 downto 0)) & unsigned(b_be));
                v_h6h := unsigned(b_ao(7 downto 2)); v_v4h := (unsigned(b_ao(1 downto 0)) & unsigned(b_bo));
            else
                v_h6l := unsigned(b_ao(7 downto 2)); v_v4l := (unsigned(b_ao(1 downto 0)) & unsigned(b_bo));
                v_h6h := unsigned(b_ae(7 downto 2)); v_v4h := (unsigned(b_ae(1 downto 0)) & unsigned(b_be));
            end if;
            v_d6 := v_h6h - v_h6l;
            b4_h <= v_h6l & "00"; b4_dh <= resize(signed(v_d6), 8); b4_dh3 <= resize(signed(v_d6), 8) + shift_left(resize(signed(v_d6), 8), 1);
            b4_v <= "0000" & v_v4l;
            v_dv := signed('0' & v_v4h) - signed('0' & v_v4l);
            b4_dv <= resize(v_dv, 7); b4_dv3 <= shift_left(resize(v_dv, 7), 2) - resize(v_dv, 7);   -- 3 dv as 4 dv - dv (dv + 2 dv shares its sign bits: a router1 self-add)
            -- 5: H8 = 4 H6 + dH fy / 4 (wraps), V16 = 16 V4 + dV fy
            v_m := resize(f_dig(b4_dh, b4_dh3, e_fy(1 downto 0)), 12) + shift_left(resize(f_dig(b4_dh, b4_dh3, e_fy(3 downto 2)), 12), 2);
            b5_h <= b4_h + unsigned(v_m(9 downto 2));
            v_m := resize(f_dig(b4_dv, b4_dv3, e_fy(1 downto 0)), 12) + shift_left(resize(f_dig(b4_dv, b4_dv3, e_fy(3 downto 2)), 12), 2);
            b5_v <= unsigned(resize(shift_left(resize(signed('0' & b4_v(3 downto 0)), 12), 4) + v_m, 9));

            -------------------------------------------------------------------
            -- texture row fill: one 24-bit hash per clock.  Per row column j (lattice column jf):
            --   even jf: F0, F1 (fine rows kf, kf + 1); the coarse value is the current C
            --   odd jf:  F0, F1, C0, C1 (coarse point (jf + 1) / 2); the coarse value is the
            --            average of the previous and the new C
            --   prologue: C0, C1 of coarse point jf0 / 2 (no output)
            -- tag: prologue(14) odd(13) kind(12:11: 00 F0, 01 F1, 10 C0, 11 C1) frac(10:8) col(7:0)
            -------------------------------------------------------------------
            h_v(0) <= '0';
            if t_run = '1' then
                h_v(0) <= '1';
                if t_pro = '1' or t_slot(1) = '1' then
                    if t_pro = '1' then v_cc := '0' & t_jf(9 downto 1); else v_cc := resize(shift_right(t_jf + 1, 1), 10); end if;
                    v_kr := t_yc(20 downto 8) + resize(t_slot(0 downto 0), 13);
                    v_hk := v_kr & v_cc(9 downto 0) & '1';
                    h_tag(0) <= t_pro & t_jf(0) & '1' & t_slot(0) & std_logic_vector(t_yc(7 downto 5)) & std_logic_vector(t_j(7 downto 0));
                else
                    v_kr := t_yf(20 downto 8) + resize(t_slot(0 downto 0), 13);
                    v_hk := v_kr & t_jf & '0';
                    h_tag(0) <= '0' & t_jf(0) & '0' & t_slot(0) & std_logic_vector(t_yf(7 downto 5)) & std_logic_vector(t_j(7 downto 0));
                end if;
                h_h(0) <= v_hk + x"5A5A5A";
                if t_pro = '1' then
                    if t_slot(0) = '1' then
                        t_pro <= '0'; t_slot <= "00";
                        t_yc <= t_yc - to_unsigned(C_RIP_SHC, 21);   -- the next coarse point
                    else
                        t_slot <= "01";
                    end if;
                elsif (t_slot = "01" and t_jf(0) = '0') or t_slot = "11" then
                    t_slot <= "00";
                    t_j <= t_j + 1; t_jf <= t_jf + 1;
                    t_yf <= t_yf - to_unsigned(C_RIP_SHF, 21);
                    if t_slot = "11" then t_yc <= t_yc - to_unsigned(C_RIP_SHC, 21); end if;
                    if t_j = s_ncol then t_run <= '0'; end if;
                else
                    t_slot <= t_slot + 1;
                end if;
            end if;
            -- hash rounds (one adder + one xor per stage)
            for k in 1 to 4 loop h_v(k) <= h_v(k - 1); h_tag(k) <= h_tag(k - 1); end loop;
            v_h := h_h(0); v_h := v_h + shift_left(v_h, 10); h_h(1) <= v_h xor shift_right(v_h, 6);
            v_h := h_h(1); v_h := v_h + shift_left(v_h, 3);  h_h(2) <= v_h xor shift_right(v_h, 11);
            v_h := h_h(2); v_h := v_h + shift_left(v_h, 15); h_h(3) <= v_h xor shift_right(v_h, 7);
            v_h := h_h(3); h_h(4) <= v_h + shift_left(v_h, 5);
            -- collector: lerps by the 2-bit row fraction (candidates in parallel, then a mux)
            c_out <= '0';
            if h_v(4) = '1' then
                v_kind := h_tag(4)(12 downto 11);
                v_fr2 := unsigned(h_tag(4)(10 downto 9));
                v_bp := signed(h_h(4)(15 downto 8) xor x"80"); v_bq := signed(h_h(4)(7 downto 0) xor x"80");
                case v_kind is
                    when "00" =>
                        c_f0p <= v_bp; c_f0q <= v_bq;
                    when "01" =>
                        c_fp <= f_lerp4(c_f0p, v_bp, v_fr2); c_fq <= f_lerp4(c_f0q, v_bq, v_fr2);
                        if h_tag(4)(13) = '0' then c_out <= '1'; c_oddo <= '0'; end if;
                    when "10" =>
                        c_c0p <= v_bp; c_c0q <= v_bq;
                    when others =>
                        c_cpp <= c_cnp; c_cpq <= c_cnq;
                        c_cnp <= f_lerp4(c_c0p, v_bp, v_fr2); c_cnq <= f_lerp4(c_c0q, v_bq, v_fr2);
                        if h_tag(4)(14) = '0' then c_out <= '1'; c_oddo <= '1'; end if;
                end case;
            end if;
            -- combine: n' = (C + F / 2 + F / 8) / 4
            c2_out <= c_out;
            if c_oddo = '1' then
                v_p := resize(c_cpp, 12) + resize(c_cnp, 12); v_qq := resize(c_cpq, 12) + resize(c_cnq, 12);
            else
                v_p := shift_left(resize(c_cnp, 12), 1); v_qq := shift_left(resize(c_cnq, 12), 1);
            end if;
            c2_p <= v_p + resize(c_fp, 12) + resize(shift_right(c_fp, 2), 12);
            c2_q <= v_qq + resize(c_fq, 12) + resize(shift_right(c_fq, 2), 12);

            -------------------------------------------------------------------
            -- texture colour per lattice column: zone, stripe, H = muH + (q AH + st KS) / 64 (bead: HB + st KS / 64),
            -- V = muV + (p AV + st KV) / 64
            -------------------------------------------------------------------
            tv(1) <= c2_out;
            for k in 2 to 6 loop tv(k) <= tv(k - 1); end loop;
            -- T1: noise (7 bits), zone bin, stripe triangle; column DDAs
            v_l := shift_right(c2_p, 3); v_p := shift_right(c2_q, 3);
            if v_l > 63 then v_l := to_signed(63, 12); elsif v_l < -64 then v_l := to_signed(-64, 12); end if;
            if v_p > 63 then v_p := to_signed(63, 12); elsif v_p < -64 then v_p := to_signed(-64, 12); end if;
            t1_p <= v_l(6 downto 0); t1_q <= v_p(6 downto 0);
            t1_zi <= e_zc(23 downto 19);
            v_sti := e_scw;
            if v_sti >= 32768 then v_sti := v_sti - 32768; else v_sti := 32768 - v_sti; end if;
            v_st7 := resize(signed('0' & v_sti(15 downto 9)) - 32, 7);
            if v_st7 < C_RIP_SC then t1_st <= to_signed(C_RIP_SC, 7); else t1_st <= v_st7; end if;   -- stripe floor (one-sided ridges)
            -- ridge phase wander: the previous column's hue noise (columns come every other clock)
            e_scw <= e_scp + unsigned(shift_left(resize(t1_q, 16), C_RIP_SWSH));
            if c2_out = '1' then
                e_zc <= e_zc + to_unsigned(8 * C_RIP_ZPX, 24);
                e_scp <= e_scp - 2696;
            end if;
            -- T2: zone row; hue noise h = q, q + p or q - p (the zone's hue / value correlation); 3 p, 3 h
            t2_zt <= C_RIP_ZT(to_integer(t1_zi));
            case C_RIP_ZS(to_integer(t1_zi)) is
                when 1      => v_hn := resize(t1_q, 10) + resize(t1_p, 10);
                when 2      => v_hn := resize(t1_q, 10) - resize(t1_p, 10);
                when others => v_hn := resize(t1_q, 10);
            end case;
            t2_qr <= t1_q;
            t2_p <= resize(t1_p, 10); t2_q <= v_hn; t2_st <= t1_st;
            t2_p3 <= resize(t1_p, 10) + shift_left(resize(t1_p, 10), 1);
            t2_q3 <= v_hn + shift_left(v_hn, 1);
            -- T3: two digits of each 6-bit coefficient; stripe terms
            t3_ahl <= resize(f_dig(t2_q, t2_q3, unsigned(t2_zt(22 downto 21))), 14) + shift_left(resize(f_dig(t2_q, t2_q3, unsigned(t2_zt(24 downto 23))), 14), 2);
            -- red bead: the hue noise q above the zone's threshold 2 T + 1 (T signed 6-bit, row bits 20:15)
            if t2_qr > signed(t2_zt(20 downto 15) & '1') then t3_bd <= '1'; else t3_bd <= '0'; end if;
            t3_avl <= resize(f_dig(t2_p, t2_p3, unsigned(t2_zt(1 downto 0))), 14) + shift_left(resize(f_dig(t2_p, t2_p3, unsigned(t2_zt(3 downto 2))), 14), 2);
            t3_sh <= resize(t2_st * to_signed(C_RIP_KS, 7), 14);
            t3_sv <= resize(t2_st * to_signed(C_RIP_KV, 7), 14);
            t3_mh <= unsigned(t2_zt(34 downto 27)); t3_mv <= unsigned(t2_zt(13 downto 6));
            -- T4: products
            if t3_bd = '1' then
                t4_ah <= (others => '0'); t4_mh <= to_unsigned(C_RIP_HB, 8); t4_mv <= t3_mv + to_unsigned(C_RIP_VB mod 256, 8);
            else
                t4_ah <= t3_ahl; t4_mh <= t3_mh; t4_mv <= t3_mv;
            end if;
            t4_av <= t3_avl;
            t4_sh <= t3_sh; t4_sv <= t3_sv;
            -- T5: H8 = muH + (h AH 2^AHSH + st KS) / 16 (wraps), V8 = muV + (p AV + st KV) / 8 (clamped); beads take HB, muV + VB
            v_hs := shift_left(resize(t4_ah, 15), C_RIP_AHSH) + resize(t4_sh, 15);
            t5_h <= t4_mh + unsigned(v_hs(11 downto 4));
            v_vs := shift_left(resize(t4_av, 15), 2) + resize(t4_sv, 15);
            v_tv := signed(resize(t4_mv, 12)) + resize(shift_right(v_vs, 3), 12);
            if v_tv < 0 then t5_v <= (others => '0'); elsif v_tv > 255 then t5_v <= (others => '1'); else t5_v <= unsigned(v_tv(7 downto 0)); end if;

            -------------------------------------------------------------------
            -- row RAM write: band (b_*) or texture (c2_*)
            -------------------------------------------------------------------
            s_rw_e <= '0'; s_rw_o <= '0';
            if b_v(6) = '1' then
                v_v8 := b5_v + resize(b5_v(8 downto 5), 9);
                if v_v8 > 255 then v_v8 := to_unsigned(255, 9); end if;
                s_rw_d <= std_logic_vector(b5_h) & std_logic_vector(v_v8(7 downto 0));
                s_rw_a <= e_wb & w_col(7 downto 1);
                s_rw_e <= not w_col(0); s_rw_o <= w_col(0);
                w_col <= w_col + 1;
            elsif tv(6) = '1' then
                s_rw_d <= std_logic_vector(t5_h) & std_logic_vector(t5_v);
                s_rw_a <= e_wb & w_col(7 downto 1);
                s_rw_e <= not w_col(0); s_rw_o <= w_col(0);
                w_col <= w_col + 1;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- Pixel path (stage k: the register holds pixel n at clock t0(n) + k)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_c : signed(21 downto 0);
        variable v_u : unsigned(9 downto 0);
        variable v_ci : integer range 0 to 255;
        variable v_hl, v_hh, v_vl, v_vh : unsigned(7 downto 0);
        variable v_d8 : unsigned(7 downto 0);
        variable v_x3 : signed(10 downto 0);
        variable v_m : signed(13 downto 0);
        variable v_e3 : signed(9 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vg : unsigned(22 downto 0);
        variable v_vm : unsigned(7 downto 0);
        variable v_yy : signed(13 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable p11_v3 : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            -- 0: column DDA (aligned with g_avid)
            if g_start = '1' then
                s_col <= l_cst;
            elsif g_avid = '1' then
                s_col <= s_col + signed(resize(l_cstp, 22));
            end if;
            p_bnd(1) <= l_band;
            for k in 2 to 20 loop p_bnd(k) <= p_bnd(k - 1); end loop;
            -- 1: |col| (the field edge at 0 is a mirror on band lines)
            if s_col < 0 and l_fold = '1' then v_c := -s_col; else v_c := s_col; end if;
            if v_c < 0 then v_c := (others => '0'); end if;
            p1_c <= unsigned(v_c(21 downto 12));
            p1_f <= unsigned(v_c(11 downto 8));
            -- 2: the right field edge is a mirror (band lines)
            if p1_c > 254 and l_fold = '1' then
                v_u := to_unsigned(509, 10) - p1_c;
            elsif p1_c > 254 then
                v_u := to_unsigned(254, 10);
            else
                v_u := p1_c;
            end if;
            p2_c <= v_u(8 downto 0);
            p_f(2) <= p1_f;
            for k in 3 to 7 loop p_f(k) <= p_f(k - 1); end loop;
            -- 3: quad mirror at the centre column (band lines)
            if s_quad = '1' and p_bnd(2) = '1' and p2_c >= 128 then
                p3_c <= to_unsigned(255, 9) - p2_c;
            else
                p3_c <= p2_c;
            end if;
            -- 4: row RAM addresses: entry c in the even RAM if c even, c + 1 in the other
            if p3_c > 254 then v_ci := 254; else v_ci := to_integer(p3_c(7 downto 0)); end if;
            s_rr_e <= s_rbank & to_unsigned((v_ci + 1) / 2, 7);
            s_rr_o <= s_rbank & to_unsigned(v_ci / 2, 7);
            p_par(4) <= to_unsigned(v_ci, 8)(0);
            p_par(5) <= p_par(4); p_par(6) <= p_par(5);
            -- 5: RAM out; 6: fabric
            p6_e <= s_rq_e; p6_o <= s_rq_o;
            -- 7: lo, differences (hue on the shortest path)
            if p_par(6) = '0' then
                v_hl := unsigned(p6_e(15 downto 8)); v_vl := unsigned(p6_e(7 downto 0));
                v_hh := unsigned(p6_o(15 downto 8)); v_vh := unsigned(p6_o(7 downto 0));
            else
                v_hl := unsigned(p6_o(15 downto 8)); v_vl := unsigned(p6_o(7 downto 0));
                v_hh := unsigned(p6_e(15 downto 8)); v_vh := unsigned(p6_e(7 downto 0));
            end if;
            v_d8 := v_hh - v_hl;
            p7_h <= v_hl; p7_dh <= signed(v_d8);
            p7_v <= v_vl; p7_dv <= signed('0' & v_vh) - signed('0' & v_vl);
            -- 8: H12 = 16 H + dH f (wraps), V12 = 16 V + dV f
            v_x3 := resize(p7_dh, 11) + shift_left(resize(p7_dh, 11), 1);
            v_m := resize(f_dig(resize(p7_dh, 11), v_x3, p_f(7)(1 downto 0)), 14) + shift_left(resize(f_dig(resize(p7_dh, 11), v_x3, p_f(7)(3 downto 2)), 14), 2);
            p8_h <= (p7_h & "0000") + unsigned(v_m(11 downto 0));
            v_x3 := resize(p7_dv, 11) + shift_left(resize(p7_dv, 11), 1);
            v_m := resize(f_dig(resize(p7_dv, 11), v_x3, p_f(7)(1 downto 0)), 14) + shift_left(resize(f_dig(resize(p7_dv, 11), v_x3, p_f(7)(3 downto 2)), 14), 2);
            v_m := shift_left(resize(signed('0' & p7_v), 14), 4) + v_m;
            if v_m < 0 then p8_v <= (others => '0'); else p8_v <= unsigned(v_m(11 downto 0)); end if;
            s_lu1 <= signed(unsigned(s_lu_q) xor x"80");
            -- 9: hue + field angle; etch products l8 eh (6 bits), l8 eg (5 bits)
            p9_a <= p8_h + s_th0;
            p9_v <= p8_v;
            v_e3 := resize(s_lu1, 10) + shift_left(resize(s_lu1, 10), 1);
            p9_he <= resize(f_dig(resize(s_lu1, 10), v_e3, s_eh(1 downto 0)), 14) + shift_left(resize(f_dig(resize(s_lu1, 10), v_e3, s_eh(3 downto 2)), 14), 2)
                   + shift_left(resize(f_dig(resize(s_lu1, 10), v_e3, s_eh(5 downto 4)), 14), 4);
            -- 10: hue + etch; gain = base + etch
            p10_a <= p9_a + unsigned(resize(shift_right(p9_he, 2), 12));
            p10_v <= p9_v;
            p10_g <= s_gq;
            -- 11: colour ROM address; gain float
            s_cl_a <= p10_a(11 downto 4);
            p11_v <= p10_v;
            p11_ge <= f_fe(p10_g); p11_gm <= f_fm(p10_g);
            -- 12: ROM out; V x M
            p11_v3 := resize(signed('0' & p11_v), 15) + shift_left(resize(signed('0' & p11_v), 15), 1);
            p12_vg <= unsigned(resize(f_dig(resize(signed('0' & p11_v), 15), p11_v3, p11_gm(1 downto 0)), 16)
                    + shift_left(resize(f_dig(resize(signed('0' & p11_v), 15), p11_v3, p11_gm(3 downto 2)), 16), 2));
            p12_ge <= p11_ge;
            -- 13: ROM fabric register; V' = (V M) << e >> 10, clamped to 255 - floor
            p13_cl <= s_cl_q;
            v_vg := shift_left(resize(p12_vg, 23), to_integer(p12_ge));
            if p_bnd(12) = '1' then v_vm := to_unsigned(247, 8); else v_vm := to_unsigned(255 - C_RIP_TFL, 8); end if;
            if v_vg(22 downto 10) > v_vm then p13_v <= v_vm; else p13_v <= v_vg(17 downto 10); end if;
            -- 14: 3 x ROM terms
            p14_y <= signed('0' & p13_cl(23 downto 16)); p14_u <= resize(signed(p13_cl(15 downto 8)), 9); p14_w <= resize(signed(p13_cl(7 downto 0)), 9);
            p14_y3 <= resize(signed('0' & p13_cl(23 downto 16)), 11) + shift_left(resize(signed('0' & p13_cl(23 downto 16)), 11), 1);
            p14_u3 <= resize(signed(p13_cl(15 downto 8)), 11) + shift_left(resize(signed(p13_cl(15 downto 8)), 11), 1);
            p14_w3 <= resize(signed(p13_cl(7 downto 0)), 11) + shift_left(resize(signed(p13_cl(7 downto 0)), 11), 1);
            p14_v <= p13_v;
            -- 15: partial products by the value digits
            p15_yl <= resize(f_dig(resize(p14_y, 11), p14_y3, p14_v(1 downto 0)), 15) + shift_left(resize(f_dig(resize(p14_y, 11), p14_y3, p14_v(3 downto 2)), 15), 2);
            p15_yh <= resize(f_dig(resize(p14_y, 11), p14_y3, p14_v(5 downto 4)), 15) + shift_left(resize(f_dig(resize(p14_y, 11), p14_y3, p14_v(7 downto 6)), 15), 2);
            p15_ul <= resize(f_dig(resize(p14_u, 11), p14_u3, p14_v(1 downto 0)), 15) + shift_left(resize(f_dig(resize(p14_u, 11), p14_u3, p14_v(3 downto 2)), 15), 2);
            p15_uh <= resize(f_dig(resize(p14_u, 11), p14_u3, p14_v(5 downto 4)), 15) + shift_left(resize(f_dig(resize(p14_u, 11), p14_u3, p14_v(7 downto 6)), 15), 2);
            p15_wl <= resize(f_dig(resize(p14_w, 11), p14_w3, p14_v(1 downto 0)), 15) + shift_left(resize(f_dig(resize(p14_w, 11), p14_w3, p14_v(3 downto 2)), 15), 2);
            p15_wh <= resize(f_dig(resize(p14_w, 11), p14_w3, p14_v(5 downto 4)), 15) + shift_left(resize(f_dig(resize(p14_w, 11), p14_w3, p14_v(7 downto 6)), 15), 2);
            -- 16: products
            p16_y <= resize(p15_yl, 19) + shift_left(resize(p15_yh, 19), 4);
            p16_u <= resize(p15_ul, 19) + shift_left(resize(p15_uh, 19), 4);
            p16_w <= resize(p15_wl, 19) + shift_left(resize(p15_wh, 19), 4);
            -- 17: Y = 64 + floor + p / 64, U / V = 512 + p / 64
            if p_bnd(16) = '1' then v_yy := resize(shift_right(p16_y, 6), 14) + 91;
            else v_yy := resize(shift_right(p16_y, 6), 14) + (64 + (C_RIP_TFL * 876 + 127) / 255); end if;
            if v_yy > 1000 then p17_y <= to_unsigned(1000, 10); else p17_y <= unsigned(v_yy(9 downto 0)); end if;
            v_yy := resize(shift_right(p16_u, 6), 14) + 512;
            if v_yy < 4 then p17_u <= to_unsigned(4, 10); elsif v_yy > 1019 then p17_u <= to_unsigned(1019, 10); else p17_u <= unsigned(v_yy(9 downto 0)); end if;
            v_yy := resize(shift_right(p16_w, 6), 14) + 512;
            if v_yy < 4 then p17_v <= to_unsigned(4, 10); elsif v_yy > 1019 then p17_v <= to_unsigned(1019, 10); else p17_v <= unsigned(v_yy(9 downto 0)); end if;
            -- 18, 19: pads; 20: to the conditioning
            p18_y <= p17_y; p18_u <= p17_u; p18_v <= p17_v;
            p19_y <= p18_y; p19_u <= p18_u; p19_v <= p18_v;
            s_y12 <= p19_y; s_u12 <= p19_u; s_v12 <= p19_v;

            -- luma ring read address (beam x at stage 7)
            if s_gav_sr(4) = '1' then s_rx <= s_rx + 1; else s_rx <= (others => '0'); end if;
            s_lu_ra <= s_rx;

            -- output conditioning (centre of the chroma box at C_LAT - 3)
            -- (8-bit chroma in an EBR delay ring: the value 8 clocks back = read 6 behind + EBR and fabric registers)
            s_cd_r <= s_cd_q;
            s_boxu <= s_boxu + resize(s_u12(9 downto 2), 12) - resize(unsigned(s_cd_r(15 downto 8)), 12);
            s_boxv <= s_boxv + resize(s_v12(9 downto 2), 12) - resize(unsigned(s_cd_r(7 downto 0)), 12);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u18 <= v_sc(11 downto 2);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v18 <= v_sc(11 downto 2);
            s_y13 <= s_y12; s_y14 <= s_y13; s_y15 <= s_y14; s_y16 <= s_y15;
            s_y17 <= s_y16; s_y18x <= s_y17;
            v_sy := resize(s_y16, 12) + shift_left(resize(s_y17, 12), 1) + resize(s_y18x, 12);
            s_yf18 <= v_sy(11 downto 2);
            -- luma toward black; blanking gate on the generated avid (delay C_LAT - 2 from the sync ring)
            if s_yf18 > 64 then v_dl := s_yf18 - 64; else v_dl := (others => '0'); end if;
            if s_sy_d1(3) = '1' then
                s_y19 <= s_yf18 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u19 <= s_u18; s_v19 <= s_v18;
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_sy_d3(0);
    data_out.vsync_n <= s_sy_d3(1);
    data_out.field_n <= s_sy_d3(2);
    data_out.avid    <= s_sy_d3(3);

end architecture riptide;
