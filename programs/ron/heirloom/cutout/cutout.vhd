-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Cutout v0.5.0 -- Heirloom #55 (2026-10-28), rebuilt from tezcatlipoca.jpg (a paper collage).
--
-- The collage: a skull-like head in profile, facing left, cut out of a flat cadmium-yellow
-- ground and filled with overlapping torn magazine scraps -- blues, teals, glossy blacks, a sea,
-- skies, aerial landscapes -- with a teacup of red soup for the eye, a plate of pastries and
-- gravy on the cheek, a black torn band across the face, a row of grey stone teeth, a "Love."
-- label and a black jaw.
--
-- The program: the head and its 53 measured scraps live in a 54 x 74 cell map (scrap id +
-- distance to the nearest other id per cell, 4 bank ROMs); the pixel path interpolates the signed
-- distance bilinearly, so the scrap edges come back at sub-cell precision, and adds a tear noise
-- (two triangle plane waves + a positional hash) to it: ragged torn edges, white paper-core
-- fringes on a third of them, a clean painted head edge.  Every scrap prints the camera picture
-- (Source = Osc: a generated magazine photo) from a 128 x 64 x 4-bit store at its own crop, scale
-- and flip, through its own ink ramp (16 luma quantiles of that scrap family in the photo).  The
-- soup and its cream rim are chord quadratics (one squarer), the cup, saucer, garnish, plate, plate
-- rim, gravy and pastries come from the map (rings from the distance), the teeth and the "Love."
-- glyph from a 4096 x 2 detail ROM, the forks are per-line bars.
--
--   K1  Rag      torn-edge raggedness: scissor-clean .. the photo .. shredded
--   K2  Zoom     the size of the pictures in the scraps (3 octaves, 32 steps)
--   K3  Ink      texture contrast of the scraps: flat paper .. the photo .. hard
--   K4  Jaw      the jaw drops open (0 = closed, the photo)
--   K5  Paint    the ground: yellow (the photo) and seven more poster paints
--   K6  Speed    bipolar: the crops pan, the Osc photo drifts, the torn strips sway (centre = still)
--   S7  Facing   Left (the photo) / Right (mirrored)
--   S8  Fringe   On / Off (the white torn paper core)
--   S9  Chatter  Off / On (the jaw chatters, at least a third open)
--   S10 Eye      Soup / Mirror (the soup becomes Tezcatlipoca's obsidian mirror: glossy black)
--   S11 Source   Video / Osc
--   P12 TEAR     0 = the intact collage; up: the head tears along its horizontal tears into eight
--                strips that slide apart and sway; 100 % = flung across the frame
--
-- Geometry in map units (0.75 virtual px, 1080 virtual lines) from the timing ID: the same
-- picture at SD, 720p and 1080p, the cup and plate round on 4:3.  Engines: a field sequencer
-- (one serial multiplier: zoom products, jaw, eight strip offsets into a strip RAM that also
-- holds the zoom / rag / paint tables) and a line prep in hblank (row DDAs, jaw, strip, map
-- rows, cup chord, teeth / glyph rows).  Generated avid; output conditioning (luma [1 2 1],
-- chroma [1 2 2 2 2 2 2 2 1]/16, luma to ~777) before the blanking gate.  Latency 32 in every
-- mode.  Colours in the hardware convention (u = Cr, v = Cb), see cutout.py.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.cutout_pkg.all;
use work.all;

architecture cutout of program_top is
    constant C_LAT : integer := 32;

    ---------------------------------------------------------------------------
    -- mode tables (timing ID): map units (Q.8) per pixel / per line, line-start x, store sampling
    ---------------------------------------------------------------------------
    function f_sd(t : std_logic_vector(3 downto 0)) return boolean is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return true;
            when others => return false;
        end case;
    end function;
    function f_720(t : std_logic_vector(3 downto 0)) return boolean is
    begin
        case to_integer(unsigned(t)) is
            when 5 | 6 | 14 => return true;
            when others => return false;
        end case;
    end function;
    function f_sy(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1517, 11);
            when 4          => return to_unsigned(768, 11);
            when 8          => return to_unsigned(1280, 11);
            when 12         => return to_unsigned(640, 11);
            when 5 | 6 | 14 => return to_unsigned(512, 11);
            when 1 | 2 | 10 => return to_unsigned(683, 11);
            when others     => return to_unsigned(341, 11);
        end case;
    end function;
    function f_kr(t : std_logic_vector(3 downto 0)) return unsigned is   -- store rows per line, Q.14
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(4315, 13);
            when 4          => return to_unsigned(2185, 13);
            when 8          => return to_unsigned(3641, 13);
            when 12         => return to_unsigned(1820, 13);
            when 5 | 6 | 14 => return to_unsigned(1456, 13);
            when 1 | 2 | 10 => return to_unsigned(1942, 13);
            when others     => return to_unsigned(971, 13);
        end case;
    end function;
    function f_nb(t : std_logic_vector(3 downto 0)) return unsigned is   -- log2 lines summed per row
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1, 3);
            when 4 | 8      => return to_unsigned(2, 3);
            when 12 | 5 | 6 | 14 | 1 | 2 | 10 => return to_unsigned(3, 3);
            when others     => return to_unsigned(4, 3);
        end case;
    end function;

    -- strips (Tear): boundaries in Q.4 map units, amplitudes (Q.4 mu at full tear), sway phases
    function f_sb(i : unsigned(2 downto 0)) return signed is
    begin
        case to_integer(i) is
            when 0 => return to_signed(2909, 17);  when 1 => return to_signed(5285, 17);
            when 2 => return to_signed(8770, 17);  when 3 => return to_signed(11477, 17);
            when 4 => return to_signed(12557, 17); when 5 => return to_signed(14645, 17);
            when 6 => return to_signed(16805, 17); when others => return to_signed(65535, 17);
        end case;
    end function;
    function f_amp(i : unsigned(2 downto 0)) return signed is
    begin
        case to_integer(i) is
            when 0 => return to_signed(-10757, 16); when 1 => return to_signed(6574, 16);
            when 2 => return to_signed(-4183, 16);  when 3 => return to_signed(8964, 16);
            when 4 => return to_signed(-7171, 16);  when 5 => return to_signed(5379, 16);
            when 6 => return to_signed(-11952, 16); when others => return to_signed(9562, 16);
        end case;
    end function;
    function f_sph(i : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(i) is
            when 0 => return to_unsigned(0, 16);     when 1 => return to_unsigned(17800, 16);
            when 2 => return to_unsigned(32480, 16); when 3 => return to_unsigned(46100, 16);
            when 4 => return to_unsigned(9420, 16);  when 5 => return to_unsigned(26200, 16);
            when 6 => return to_unsigned(54400, 16); when others => return to_unsigned(39800, 16);
        end case;
    end function;

    function f_tri8(p : unsigned(15 downto 0)) return unsigned is   -- 0 .. 255 .. 0 over one period
    begin
        if p(15) = '1' then return not p(14 downto 7); else return p(14 downto 7); end if;
    end function;
    type t_g5 is array (0 to 31) of unsigned(4 downto 0);
    constant C_G5 : t_g5 := (to_unsigned(0, 5), to_unsigned(1, 5), to_unsigned(1, 5), to_unsigned(1, 5), to_unsigned(2, 5),
        to_unsigned(2, 5), to_unsigned(2, 5), to_unsigned(2, 5), to_unsigned(3, 5), to_unsigned(3, 5), to_unsigned(3, 5),
        to_unsigned(4, 5), to_unsigned(4, 5), to_unsigned(4, 5), to_unsigned(5, 5), to_unsigned(5, 5), to_unsigned(5, 5),
        to_unsigned(6, 5), to_unsigned(6, 5), to_unsigned(7, 5), to_unsigned(7, 5), to_unsigned(8, 5), to_unsigned(8, 5),
        to_unsigned(9, 5), to_unsigned(10, 5), to_unsigned(10, 5), to_unsigned(11, 5), to_unsigned(12, 5), to_unsigned(13, 5),
        to_unsigned(14, 5), to_unsigned(15, 5), to_unsigned(15, 5));   -- K1 Rag -> tear-noise gain
    function f_mix1(w : unsigned(15 downto 0)) return unsigned is
        variable a : unsigned(15 downto 0);
    begin
        a := w xor shift_right(w, 7);
        return a + shift_left(a, 3);
    end function;

    function f_hyst(k : unsigned(9 downto 0); h : unsigned(4 downto 0)) return unsigned is
        variable c : unsigned(10 downto 0);
    begin
        c := resize(h & "10000", 11);
        if resize(k, 11) > c + 24 or resize(k, 11) + 24 < c then return k(9 downto 5); end if;
        return h;
    end function;
    function f_t8(p : unsigned(7 downto 0)) return unsigned is   -- 8-bit phase triangle
    begin
        if p(7) = '1' then return 255 - shift_left(resize(p(6 downto 0), 8), 1);
        else return shift_left(resize(p(6 downto 0), 8), 1); end if;
    end function;

    ---------------------------------------------------------------------------
    -- memories
    ---------------------------------------------------------------------------
    type t_store is array (0 to 8191) of std_logic_vector(3 downto 0);
    signal st : t_store := (others => (others => '0'));                  -- 64 rows x 128 columns
    attribute no_rw_check : boolean;
    attribute no_rw_check of st : signal is true;
    type t_acc is array (0 to 127) of std_logic_vector(15 downto 0);
    signal accr : t_acc := (others => (others => '0'));
    attribute no_rw_check of accr : signal is true;
    type t_dly is array (0 to 255) of std_logic_vector(3 downto 0);
    signal s_dly : t_dly := (others => "1110");
    attribute no_rw_check of s_dly : signal is true;
    type t_srm is array (0 to 255) of std_logic_vector(15 downto 0);
    function f_srm_init return t_srm is      -- 0..15 strip offsets (written); 32 zoom, 64 rag gain, 96 paints
        variable r : t_srm := (others => (others => '0'));
        variable c : std_logic_vector(31 downto 0);
    begin
        for i in 0 to 31 loop
            r(32 + i) := std_logic_vector(resize(C_ZT(i), 16));
            r(64 + i) := std_logic_vector(resize(C_G5(i), 16));
        end loop;
        for i in 0 to 7 loop
            c := C_PAINT(i);
            r(96 + 4 * i) := std_logic_vector(resize(signed(resize(unsigned(c(29 downto 20)), 12)) - 941, 16));
            r(97 + 4 * i) := std_logic_vector(resize(signed(resize(unsigned(c(19 downto 10)), 12)) - 576, 16));
            r(98 + 4 * i) := std_logic_vector(resize(signed(resize(unsigned(c(9 downto 0)), 12)) - 124, 16));
        end loop;
        return r;
    end function;
    signal r_ram : t_srm := f_srm_init;
    attribute no_rw_check of r_ram : signal is true;

    signal b_we : std_logic := '0';
    signal b_wa, b_ra : unsigned(12 downto 0) := (others => '0');
    signal b_wd, b_q : std_logic_vector(3 downto 0) := (others => '0');
    signal a_we : std_logic := '0';
    signal a_wa, a_ra : unsigned(6 downto 0) := (others => '0');
    signal a_wd, a_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s_dwp, s_drp : unsigned(7 downto 0) := (others => '0');
    signal s_dq, s_dg, s_do : std_logic_vector(3 downto 0) := "1110";
    signal m_a0, m_a1, m_a2, m_a3 : unsigned(9 downto 0) := (others => '0');
    signal m_q0, m_q1, m_q2, m_q3 : std_logic_vector(7 downto 0) := (others => '0');
    signal m_r0, m_r1, m_r2, m_r3 : std_logic_vector(7 downto 0) := (others => '0');
    signal t_a : unsigned(8 downto 0) := (others => '0');
    signal t_qy, t_qu, t_qv, t_ry, t_ru, t_rv : std_logic_vector(7 downto 0) := (others => '0');
    signal d_a : unsigned(11 downto 0) := (others => '0');
    signal d_q, d_r : std_logic_vector(1 downto 0) := (others => '0');
    signal r_we : std_logic := '0';
    signal r_wa, r_ra : unsigned(7 downto 0) := (others => '0');
    signal r_wd, r_q, r_rq : std_logic_vector(15 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- timing, controls, field values
    ---------------------------------------------------------------------------
    signal s_tid : std_logic_vector(3 downto 0) := "0111";
    signal s_sx : unsigned(9 downto 0) := to_unsigned(341, 10);
    signal s_sy : unsigned(10 downto 0) := to_unsigned(341, 11);
    signal s_x0 : signed(20 downto 0) := to_signed(-229530, 21);
    signal s_kw : unsigned(9 downto 0) := to_unsigned(273, 10);
    signal s_kr : unsigned(12 downto 0) := to_unsigned(971, 13);
    signal s_na : unsigned(1 downto 0) := to_unsigned(3, 2);
    signal s_nb : unsigned(2 downto 0) := to_unsigned(4, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(10 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_mir, s_frg, s_cht, s_eye, s_osc : std_logic := '0';
    signal s_spd : signed(9 downto 0) := (others => '0');
    -- glides (Q.3): P12, K1, K2, K3, K4, K5
    signal s_g12 : unsigned(12 downto 0) := (others => '0');                      -- P12 glide (Q.3)
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal s_h1, s_h2, s_h3 : unsigned(4 downto 0) := to_unsigned(16, 5);           -- power up at the defaults
    signal s_h5 : unsigned(4 downto 0) := to_unsigned(0, 5);
    signal s_hn1, s_hn2, s_hn3, s_hn5 : unsigned(4 downto 0) := (others => '0');
    signal s_kr1, s_kr2, s_kr3, s_kr5, s_kr12 : unsigned(9 downto 0) := (others => '0');
    signal f_tdr, f_tsw, f_tos, f_tch : unsigned(15 downto 0) := (others => '0');
    signal f_zq : unsigned(12 downto 0) := to_unsigned(1536, 13);
    signal f_c16 : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal f_g5 : unsigned(4 downto 0) := to_unsigned(5, 5);
    signal f_jk, f_jkin : unsigned(9 downto 0) := (others => '0');
    signal f_j4 : unsigned(11 downto 0) := (others => '0');
    signal f_jl2 : signed(16 downto 0) := to_signed(14645, 17);
    signal f_trich : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal f_pcy, f_pcu, f_pcv : signed(10 downto 0) := (others => '0');
    signal f_dspx : signed(15 downto 0) := to_signed(128, 16);
    signal f_s0, f_sy0 : signed(20 downto 0) := (others => '0');
    signal f_syz, f_jz : signed(15 downto 0) := (others => '0');
    signal f_ni1, f_ni2, f_ni3, f_nx1, f_nx2, f_nx3, f_binc, f_bxk : unsigned(15 downto 0) := (others => '0');
    signal f_ny1, f_ny2, f_ny3, f_byi : unsigned(15 downto 0) := (others => '0');
    signal f_sxs : signed(10 downto 0) := to_signed(341, 11);
    signal f_xs0 : signed(20 downto 0) := to_signed(-229530, 21);

    ---------------------------------------------------------------------------
    -- field sequencer
    ---------------------------------------------------------------------------
    signal q_run, q_wr : std_logic := '0';
    signal q_op, q_ph : unsigned(4 downto 0) := (others => '0');
    signal q_s : unsigned(2 downto 0) := (others => '0');
    signal q_j : unsigned(1 downto 0) := (others => '0');
    signal q_sw : unsigned(9 downto 0) := (others => '0');
    signal q_sel : std_logic_vector(8 downto 0) := (others => '0');
    signal q_amp, q_as, q_off : signed(15 downto 0) := (others => '0');
    signal m_a : signed(17 downto 0) := (others => '0');
    signal m_b : unsigned(15 downto 0) := (others => '0');
    signal m_p : signed(34 downto 0) := (others => '0');
    signal m_r : signed(23 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- line prep and the live per-line values
    ---------------------------------------------------------------------------
    signal g_avd, pr_run, pr_first, pr_sq, pr_below : std_logic := '0';
    signal pr_w, pr_t : unsigned(4 downto 0) := (others => '0');
    signal pr_ym8 : signed(19 downto 0) := to_signed(-30643, 20);
    signal pr_syb : signed(20 downto 0) := (others => '0');
    signal pr_pb, pr_yh1 : unsigned(15 downto 0) := (others => '0');
    signal pr_yq4, pr_ye4 : signed(16 downto 0) := (others => '0');
    signal pr_dy : signed(10 downto 0) := (others => '0');
    signal pr_sa : unsigned(2 downto 0) := (others => '0');
    signal pr_dyq : signed(7 downto 0) := (others => '0');
    signal pr_r0 : signed(8 downto 0) := (others => '0');
    signal pr_re, pr_ro : signed(7 downto 0) := (others => '0');
    signal pr_trow : signed(10 downto 0) := (others => '0');
    signal l_ye4, l_b4, l_gv0 : signed(16 downto 0) := (others => '0');
    signal l_yei : signed(12 downto 0) := (others => '0');
    signal l_gap, l_rv0, l_rv1, l_r0p, l_tok, l_cupok : std_logic := '0';
    signal l_oa4, l_ob4 : signed(15 downto 0) := (others => '0');
    signal l_dsa, l_dsb : signed(19 downto 0) := (others => '0');
    signal l_rb0, l_rb1 : unsigned(9 downto 0) := (others => '0');
    signal l_fy : unsigned(3 downto 0) := (others => '0');
    signal l_xc4 : signed(13 downto 0) := (others => '0');
    signal l_e : unsigned(11 downto 0) := (others => '0');
    signal l_dy : signed(10 downto 0) := (others => '0');
    signal l_fx0, l_fx1 : signed(10 downto 0) := (others => '0');
    signal l_tb : unsigned(11 downto 0) := (others => '0');
    signal l_sy8 : signed(20 downto 0) := (others => '0');
    signal l_yh, l_ph1, l_ph2, l_ph3, l_pb : unsigned(15 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- generated avid
    ---------------------------------------------------------------------------
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';
    signal g_dl : std_logic_vector(15 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- pixel path
    ---------------------------------------------------------------------------
    signal p_n1, p_n2, p_n3, p_nb : unsigned(15 downto 0) := (others => '0');
    signal p_t1, p_t2, p_t3, p_tb14 : unsigned(7 downto 0) := (others => '0');
    signal p_sel2 : std_logic := '0';
    signal p_ns2, p_ns3, p_ns4, p_ns5, p_ns6, p_ns7 : signed(10 downto 0) := (others => '0');
    signal p_x8 : signed(20 downto 0) := (others => '0');
    signal p_xt3 : signed(16 downto 0) := (others => '0');
    type t_bits is array (natural range <>) of std_logic;
    signal p_sel : t_bits(3 to 15) := (others => '0');
    signal p_fk, p_wv : t_bits(4 to 15) := (others => '0');
    signal p_c04 : signed(8 downto 0) := (others => '0');
    signal p_fx4, p_fx5, p_fx6, p_fx7, p_fx8, p_fx9 : unsigned(3 downto 0) := (others => '0');
    signal p_xti4 : signed(12 downto 0) := (others => '0');
    signal p_gx4 : unsigned(11 downto 0) := (others => '0');
    signal p_gok4, p_gok5 : std_logic := '0';
    signal p_tc4, p_tc5 : signed(10 downto 0) := (others => '0');
    signal p_hw4, p_h5, p_h6, p_h7 : unsigned(15 downto 0) := (others => '0');
    signal p_ok5, p_ok6, p_ok7 : std_logic_vector(1 downto 0) := (others => '0');
    signal p_cp5, p_cp6, p_cp7 : std_logic := '0';
    signal p_uu5, p_uu6, p_uu7 : signed(7 downto 0) := (others => '0');
    signal p_gv5 : signed(16 downto 0) := (others => '0');
    signal p_gu5 : unsigned(5 downto 0) := (others => '0');
    signal p_sq6 : unsigned(13 downto 0) := (others => '0');
    signal p_gl6, p_gl7, p_gl8, p_tt6, p_tt7, p_tt8, p_gs6, p_gs7, p_gs8 : std_logic := '0';
    signal p_qs7 : unsigned(14 downto 0) := (others => '0');
    signal p_i00, p_i01, p_i10, p_i11 : unsigned(4 downto 0) := (others => '0');
    signal p_d00, p_d01, p_d10, p_d11 : unsigned(2 downto 0) := (others => '0');
    signal p_raw8 : signed(11 downto 0) := (others => '0');
    type t_s7v is array (natural range <>) of signed(6 downto 0);
    signal p_pn : t_s7v(8 to 13) := (others => (others => '0'));
    type t_ft is record
        rim, soup : std_logic;
        sh   : signed(8 downto 0);
        uu   : signed(6 downto 0);
        code : std_logic_vector(1 downto 0);
        glyb : std_logic;
        hb   : unsigned(7 downto 0);
        hf   : std_logic;
    end record;
    constant C_FT0 : t_ft := ('0', '0', (others => '0'), (others => '0'), "00", '0', (others => '0'), '0');
    type t_ftv is array (8 to 14) of t_ft;
    signal ft : t_ftv := (others => C_FT0);
    signal p_iref9, p_ioth9, p_iref10, p_ioth10, p_iref11, p_ioth11, p_iref12, p_ioth12 : unsigned(4 downto 0) := (others => '0');
    signal p_s00, p_s01, p_s10, p_s11 : signed(3 downto 0) := (others => '0');
    signal p_n9, p_n10, p_n11 : signed(11 downto 0) := (others => '0');
    signal p_top10, p_bot10 : signed(7 downto 0) := (others => '0');
    signal p_f11 : signed(13 downto 0) := (others => '0');
    signal p_fs12 : signed(12 downto 0) := (others => '0');
    signal p_pid13, p_oth13, p_pid14 : unsigned(4 downto 0) := (others => '0');
    signal p_fo13, p_fo14 : unsigned(11 downto 0) := (others => '0');
    signal p_fp13, p_fp14 : std_logic := '0';
    signal p_fam14 : unsigned(4 downto 0) := (others => '0');
    signal p_sh14 : signed(9 downto 0) := (others => '0');
    signal p_k14, p_k15, p_k16 : unsigned(1 downto 0) := (others => '0');
    signal p_fl14, p_fl15, p_fl16, p_fl17 : std_logic_vector(1 downto 0) := (others => '0');
    signal p_ou14, p_ou15, p_ou16, p_ou17 : unsigned(6 downto 0) := (others => '0');
    signal p_ov14, p_ov15, p_ov16, p_ov17 : unsigned(5 downto 0) := (others => '0');
    signal p_dn14, p_frok14, p_inpl14, p_fe14, p_gr14 : std_logic := '0';
    signal p_dn15, p_dn16, p_dn17, p_eye15, p_eye16, p_eye17 : std_logic := '0';
    signal p_uu15, p_uu16, p_uu17 : signed(6 downto 0) := (others => '0');
    type t_cb is record
        fam  : unsigned(4 downto 0);
        off  : signed(9 downto 0);
        outg : std_logic;
        frg  : std_logic;
    end record;
    constant C_CB0 : t_cb := ((others => '0'), (others => '0'), '0', '0');
    type t_cbv is array (16 to 23) of t_cb;
    signal cb : t_cbv := (others => C_CB0);
    signal p_s8, p_sxs16 : signed(16 downto 0) := (others => '0');
    signal p_fam15c : unsigned(4 downto 0) := (others => '0');
    signal p_sh15c : signed(9 downto 0) := (others => '0');
    signal p_gl15 : signed(10 downto 0) := (others => '0');
    signal p_out15c, p_hf15, p_frg15 : std_logic := '0';
    signal p_hb15 : unsigned(7 downto 0) := (others => '0');
    signal p_bu16 : signed(14 downto 0) := (others => '0');
    signal p_sys16 : signed(15 downto 0) := (others => '0');
    signal p_q19 : unsigned(3 downto 0) := (others => '0');
    signal p_cx20 : signed(10 downto 0) := (others => '0');
    signal p_y24, p_u24, p_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- conditioning
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    signal s_csu, s_csv : t_u8(0 to 8) := (others => to_unsigned(128, 8));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_y25, s_y26, s_y27, s_y28, s_y29, s_y30x, s_yf30 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u30, s_v30 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y31, s_y32 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u31, s_v31, s_u32, s_v32 : unsigned(9 downto 0) := to_unsigned(512, 10);

    ---------------------------------------------------------------------------
    -- write path (source -> store)
    ---------------------------------------------------------------------------
    signal w_sx : unsigned(11 downto 0) := (others => '0');
    signal w_wu : unsigned(19 downto 0) := (others => '0');
    signal w_win, w_nw, w_smp, w_winp : std_logic := '0';
    signal w_j : unsigned(3 downto 0) := (others => '0');
    signal w_y1 : unsigned(7 downto 0) := (others => '0');
    signal w_wv, w_wvn : unsigned(20 downto 0) := (others => '0');
    signal w_row, w_rown : unsigned(5 downto 0) := (others => '0');
    signal w_lrkn : unsigned(4 downto 0) := (others => '0');
    signal w_rok, w_lsmp, w_lfst, w_llst : std_logic := '0';
    signal w_hs : unsigned(13 downto 0) := (others => '0');
    signal w_dn, w_sv7 : std_logic := '0';
    signal w_dc, w_colq : unsigned(6 downto 0) := (others => '0');
    signal w_dhs : unsigned(13 downto 0) := (others => '0');
    signal w_aq : unsigned(15 downto 0) := (others => '0');
    signal w_sum : unsigned(15 downto 0) := (others => '0');
    signal w_sc7, w_sc8 : unsigned(6 downto 0) := (others => '0');
    signal w_st8 : std_logic := '0';
    signal w_avg8 : unsigned(8 downto 0) := (others => '0');
    signal w_row8 : unsigned(5 downto 0) := (others => '0');
    signal o_p1, o_p2, o_p3, o_p4, o_p5 : unsigned(7 downto 0) := (others => '0');
    signal o_s4 : signed(10 downto 0) := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- Memories and ROMs
    ---------------------------------------------------------------------------
    p_store : process(clk)
    begin
        if rising_edge(clk) then
            b_q <= st(to_integer(b_ra));
            if b_we = '1' then st(to_integer(b_wa)) <= b_wd; end if;
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
    p_map : process(clk)
    begin
        if rising_edge(clk) then
            m_q0 <= C_MAP0(to_integer(m_a0)); m_q1 <= C_MAP1(to_integer(m_a1));
            m_q2 <= C_MAP2(to_integer(m_a2)); m_q3 <= C_MAP3(to_integer(m_a3));
            m_r0 <= m_q0; m_r1 <= m_q1; m_r2 <= m_q2; m_r3 <= m_q3;
        end if;
    end process;
    p_tint : process(clk)
    begin
        if rising_edge(clk) then
            t_qy <= C_TY(to_integer(t_a)); t_qu <= C_TU(to_integer(t_a)); t_qv <= C_TV(to_integer(t_a));
            t_ry <= t_qy; t_ru <= t_qu; t_rv <= t_qv;
        end if;
    end process;
    p_det : process(clk)
    begin
        if rising_edge(clk) then
            d_q <= C_DET(to_integer(d_a));
            d_r <= d_q;
        end if;
    end process;

    p_srm : process(clk)
    begin
        if rising_edge(clk) then
            r_q <= r_ram(to_integer(r_ra));
            if r_we = '1' then r_ram(to_integer(r_wa)) <= r_wd; end if;
            r_rq <= r_q;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, width, glides, field counters, sync delay
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_x : signed(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_t : std_logic_vector(3 downto 0);
        variable v_c : std_logic_vector(31 downto 0);
        variable v_jk, v_k, v_gg, v_g10 : unsigned(9 downto 0);
        variable v_p20 : unsigned(39 downto 0);
        variable v_p16 : unsigned(31 downto 0);
        variable v_j14 : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            v_t := registers_in(8)(3 downto 0);
            s_tid <= v_t;
            if f_sd(s_tid) then
                s_sx <= to_unsigned(683, 10); s_kw <= to_unsigned(728, 10); s_na <= "10";
                if s_mir = '1' then s_x0 <= to_signed(343910, 21); else s_x0 <= to_signed(-147610, 21); end if;
            elsif f_720(s_tid) then
                s_sx <= to_unsigned(512, 10); s_kw <= to_unsigned(410, 10); s_na <= "11";
                if s_mir = '1' then s_x0 <= to_signed(425830, 21); else s_x0 <= to_signed(-229530, 21); end if;
            else
                s_sx <= to_unsigned(341, 10); s_kw <= to_unsigned(273, 10); s_na <= "11";
                if s_mir = '1' then s_x0 <= to_signed(425830, 21); else s_x0 <= to_signed(-229530, 21); end if;
            end if;
            s_sy <= f_sy(s_tid); s_kr <= f_kr(s_tid); s_nb <= f_nb(s_tid);

            -- vblank zone: no avid for 1024 clocks; zone clock
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
            if f_sd(s_tid) then g_wm1 <= to_unsigned(719, 12);
            elsif f_720(s_tid) then g_wm1 <= to_unsigned(1279, 12);
            else g_wm1 <= to_unsigned(1919, 12); end if;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                v_x := signed(resize(unsigned(registers_in(5)), 11)) - 512;          -- K6 Speed
                if v_x > -16 and v_x < 16 then s_spd <= (others => '0'); else s_spd <= v_x(9 downto 0); end if;
                s_mir <= registers_in(6)(0);
                s_frg <= registers_in(6)(1);
                s_cht <= registers_in(6)(2);
                s_eye <= registers_in(6)(3);
                s_osc <= registers_in(6)(4);
                -- field counters
                f_tdr <= f_tdr + unsigned(resize(shift_right(s_spd, 3), 16));
                f_tsw <= f_tsw + unsigned(resize(shift_left(resize(s_spd, 16), 1), 16));
                f_tos <= f_tos + unsigned(resize(shift_right(s_spd, 6), 16));
                f_tch <= f_tch + 2184;
            end if;
            -- P12 glide: ease 1/8 of the gap per field (deadband 4, the tail snaps), zone clocks 4..6
            if s_zone = '1' and s_zc = 4 then
                gl_d <= signed(resize(shift_left(resize(s_kr12, 13), 3), 14)) - signed(resize(s_g12, 14));
            elsif s_zone = '1' and s_zc = 5 then
                v_ng := signed(resize(s_g12, 14)) + shift_right(gl_d, 3);
                if gl_d > -32 and gl_d < 32 then null;
                elsif gl_d > -64 and gl_d < 64 then s_g12 <= shift_left(resize(s_kr12, 13), 3);
                else s_g12 <= unsigned(v_ng(12 downto 0)); end if;
            end if;
            -- K1 Rag, K2 Zoom, K3 Ink, K5 Paint: 32-step indices with hysteresis (no ADC flicker)
            s_kr1 <= unsigned(registers_in(0)); s_kr2 <= unsigned(registers_in(1));
            s_kr3 <= unsigned(registers_in(2)); s_kr5 <= unsigned(registers_in(4));
            s_kr12 <= unsigned(registers_in(7));
            s_hn1 <= f_hyst(s_kr1, s_h1); s_hn2 <= f_hyst(s_kr2, s_h2);
            s_hn3 <= f_hyst(s_kr3, s_h3); s_hn5 <= f_hyst(s_kr5, s_h5);
            if s_zgo = '1' then
                s_h1 <= s_hn1; s_h2 <= s_hn2; s_h3 <= s_hn3; s_h5 <= s_hn5;
            end if;
            -- field values (settle long before the field sequencer reads them)
            v_p16 := resize(s_h3 & "10000", 16) * to_unsigned(3, 16);
            f_c16 <= to_unsigned(4, 5) + resize(v_p16(13 downto 7), 5);               -- K3 Ink, Q.4 (4..27)
            v_jk := unsigned(registers_in(3));
            if s_cht = '1' then
                if v_jk < 358 then v_jk := to_unsigned(358, 10); end if;
                f_trich <= resize(f_tri8(f_tch), 9);
            else
                f_trich <= to_unsigned(256, 9);
            end if;
            f_jkin <= v_jk;
            v_j14 := shift_left(resize(f_jk, 14), 3) + resize(f_jk, 14);
            f_j4 <= v_j14(13 downto 2);                                                -- JK * 9/4 (Q.4 mu)
            f_jl2 <= to_signed(14645, 17) + signed(resize(f_j4, 17));
            if s_mir = '1' then f_sxs <= -signed(resize(s_sx, 11)); else f_sxs <= signed(resize(s_sx, 11)); end if;
            f_xs0 <= s_x0;

            -- sync / generated-avid delay: written each clock, read 28 entries back (s_dq = input
            -- delayed 30, the gate's), then two registers (delay 32 = C_LAT)
            s_dwp <= s_dwp + 1;
            s_drp <= s_dwp - 28;
            s_dg <= s_dq; s_do <= s_dg;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Field sequencer (vblank, from zone clock 64): one serial multiplier, 30 ops in 24-clock
    -- slots (0 operand select, 1 load, 2..17 multiply, 18 result shift, 19 write):
    --   0 JK = Jaw * chatter triangle;  1 DSPX = sx * zq;  2 S0 = x0 * zq;  3 SYZ = sy * zq;
    --   4 SY0 = y0 * zq;  5 JZ = J4 * zq;  then per strip s = 0..7:
    --   AS = amplitude(s) * sway(s);  OFF = AS * P12 -> strip RAM 2s;  OFFS = OFF * zq -> 2s + 1
    ---------------------------------------------------------------------------
    p_fseq : process(clk)
        variable v_hi : signed(18 downto 0);
        variable v_r : signed(23 downto 0);
        variable v_a : signed(17 downto 0);
        variable v_b : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            q_wr <= '0'; r_we <= '0';
            if s_zone = '1' and s_zc = 64 then
                q_run <= '1'; q_op <= (others => '0'); q_ph <= (others => '0'); q_s <= (others => '0'); q_j <= (others => '0');
            elsif q_run = '1' then
                if q_ph = 23 then
                    q_ph <= (others => '0');
                    if q_op = 29 then q_run <= '0'; end if;
                    q_op <= q_op + 1;
                    if q_op >= 6 then
                        if q_j = 2 then q_j <= (others => '0'); q_s <= q_s + 1; else q_j <= q_j + 1; end if;
                    end if;
                else
                    q_ph <= q_ph + 1;
                end if;
            end if;
            -- phase 0: operand selects (registered)
            if q_run = '1' and q_ph = 0 then
                q_sw <= 768 + resize(f_tri8(f_sph(q_s) + f_tsw), 10);
                q_amp <= f_amp(q_s);
                q_sel <= (others => '0');
                if q_op < 6 then q_sel(to_integer(q_op(2 downto 0))) <= '1';
                else q_sel(6 + to_integer(q_j)) <= '1'; end if;
            end if;
            -- phase 1: load
            if q_run = '1' and q_ph = 1 then
                m_p <= (others => '0');
                v_a := (others => '0'); v_b := (others => '0');
                if q_sel(0) = '1' then v_a := v_a or signed(resize(f_jkin, 18)); v_b := v_b or shift_left(resize(f_trich, 16), 2); end if;
                if q_sel(1) = '1' then v_a := v_a or resize(f_sxs, 18); v_b := v_b or resize(f_zq(12 downto 2), 16); end if;
                if q_sel(2) = '1' then v_a := v_a or resize(shift_right(f_xs0, 4), 18); v_b := v_b or shift_left(resize(f_zq, 16), 2); end if;
                if q_sel(3) = '1' then v_a := v_a or signed(resize(s_sy, 18)); v_b := v_b or resize(f_zq(12 downto 2), 16); end if;
                if q_sel(4) = '1' then v_a := v_a or to_signed(-1916, 18); v_b := v_b or shift_left(resize(f_zq, 16), 2); end if;
                if q_sel(5) = '1' then v_a := v_a or signed(shift_left(resize(f_j4, 18), 2)); v_b := v_b or resize(f_zq, 16); end if;
                if q_sel(6) = '1' then v_a := v_a or resize(q_amp, 18); v_b := v_b or resize(q_sw, 16); end if;
                if q_sel(7) = '1' then v_a := v_a or resize(q_as, 18); v_b := v_b or resize(s_g12(12 downto 3), 16); end if;
                if q_sel(8) = '1' then v_a := v_a or resize(q_off, 18); v_b := v_b or resize(f_zq, 16); end if;
                m_a <= v_a; m_b <= v_b;
            elsif q_run = '1' and q_ph >= 2 and q_ph <= 17 then
                if m_b(0) = '1' then v_hi := m_p(34 downto 16) + resize(m_a, 19);
                else v_hi := m_p(34 downto 16); end if;
                m_p <= v_hi(18) & v_hi & m_p(15 downto 1);
                m_b <= '0' & m_b(15 downto 1);
            elsif q_run = '1' and q_ph = 18 then
                m_r <= m_p(33 downto 10);
                q_wr <= '1';
            end if;
            -- phase 19: write
            if q_wr = '1' then
                if q_op = 0 then f_jk <= unsigned(m_r(9 downto 0));
                elsif q_op = 1 then f_dspx <= m_r(15 downto 0);
                elsif q_op = 2 then f_s0 <= m_r(20 downto 0);
                elsif q_op = 3 then f_syz <= m_r(15 downto 0);
                elsif q_op = 4 then f_sy0 <= m_r(20 downto 0);
                elsif q_op = 5 then f_jz <= m_r(15 downto 0);
                elsif q_j = 0 then q_as <= m_r(15 downto 0);
                elsif q_j = 1 then
                    q_off <= m_r(15 downto 0);
                    r_we <= '1'; r_wa <= resize(q_s & '0', 8); r_wd <= std_logic_vector(m_r(15 downto 0));
                else
                    r_we <= '1'; r_wa <= resize(q_s & '1', 8); r_wd <= std_logic_vector(m_r(17 downto 2));   -- Q.6 store px
                end if;
            end if;
        end if;
    end process p_fseq;

    ---------------------------------------------------------------------------
    -- Line prep: in hblank (20 clocks after the generated avid falls, when the last pixel has
    -- passed every stage that reads l_*), or for line 0 at zone clock 850.  Writes the next
    -- line's values straight into l_*: row DDAs (map y, store y, tear-noise and brush phases),
    -- jaw, strip and its offsets (strip RAM), map rows, cup chord, teeth / glyph rows, forks.
    ---------------------------------------------------------------------------
    p_prep : process(clk)
        variable v_sb : signed(16 downto 0);
        variable v_v : signed(16 downto 0);
        variable v_yi : signed(12 downto 0);
        variable v_d : signed(19 downto 0);
        variable v_t : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            g_avd <= g_avid;
            if g_avd = '1' and g_avid = '0' then pr_w <= to_unsigned(1, 5);
            elsif pr_w /= 0 then pr_w <= pr_w + 1; end if;
            if s_zone = '1' and s_zc = 850 then
                pr_run <= '1'; pr_t <= (others => '0'); pr_first <= '1';
            elsif pr_w = 20 then
                pr_run <= '1'; pr_t <= (others => '0'); pr_first <= '0';
            elsif pr_run = '1' then
                if pr_t = 20 then pr_run <= '0'; end if;
                pr_t <= pr_t + 1;
            end if;
            pr_sq <= '0';
            -- vblank table reads (zone clocks 40..44, data 3 clocks later): zoom, rag gain, paint
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 40 => r_ra <= "001" & s_h2;
                    when 41 => r_ra <= "010" & s_h1;
                    when 42 => r_ra <= "011" & s_h5(4 downto 2) & "00";
                    when 43 => r_ra <= "011" & s_h5(4 downto 2) & "01"; f_zq <= unsigned(r_rq(12 downto 0));
                    when 44 => r_ra <= "011" & s_h5(4 downto 2) & "10"; f_g5 <= unsigned(r_rq(4 downto 0));
                    when 45 => f_pcy <= signed(r_rq(10 downto 0));
                    when 46 => f_pcu <= signed(r_rq(10 downto 0));
                    when 47 => f_pcv <= signed(r_rq(10 downto 0));
                    when others => null;
                end case;
            end if;
            if pr_run = '1' then
                case to_integer(pr_t) is
                    when 0 =>
                        if pr_first = '1' then
                            pr_ym8 <= to_signed(-30643, 20); pr_syb <= f_sy0;
                            l_ph1 <= f_nx1 + 34553; l_ph2 <= f_nx2 + 6368; l_ph3 <= f_nx3 + 47018; pr_pb <= f_bxk + 4942;
                        else
                            pr_ym8 <= pr_ym8 + signed(resize(s_sy, 20)); pr_syb <= pr_syb + resize(f_syz, 21);
                            l_ph1 <= l_ph1 + f_ny1; l_ph2 <= l_ph2 + f_ny2; l_ph3 <= l_ph3 + f_ny3; pr_pb <= pr_pb + f_byi;
                        end if;
                    when 1 =>
                        pr_yq4 <= resize(shift_right(pr_ym8, 4), 17);
                        pr_dy <= resize(shift_right(pr_ym8, 8), 11) - 440;
                    when 2 =>
                        if pr_yq4 >= f_jl2 then
                            pr_ye4 <= pr_yq4 - signed(resize(f_j4, 17)); l_gap <= '0'; pr_below <= '1';
                        elsif pr_yq4 >= 14645 then
                            pr_ye4 <= pr_yq4; l_gap <= '1'; pr_below <= '0';
                        else
                            pr_ye4 <= pr_yq4; l_gap <= '0'; pr_below <= '0';
                        end if;
                        pr_sa <= (others => '0');
                        if pr_dy > -90 and pr_dy < 90 then l_cupok <= '1'; else l_cupok <= '0'; end if;
                        v_d := shift_left(resize(pr_dy, 20), 7) + shift_left(resize(pr_dy, 20), 5) + resize(pr_dy, 20);   -- dy * 161
                        pr_dyq <= resize(shift_right(v_d, 8), 8);
                        v_d := shift_left(resize(pr_dy, 20), 6) + shift_left(resize(pr_dy, 20), 4) + shift_left(resize(pr_dy, 20), 2)
                               + resize(pr_dy, 20);                                                                         -- dy * 85 / 32
                        l_xc4 <= to_signed(2030, 14) - resize(shift_right(v_d, 5), 14);
                        l_dy <= pr_dy;
                    when 3 | 4 | 5 | 6 | 7 | 8 | 9 =>
                        v_sb := f_sb(resize(pr_t - 3, 3));
                        if v_sb <= pr_ye4 - 64 then pr_sa <= resize(pr_t - 2, 3); end if;
                        if pr_t = 3 then
                            l_ye4 <= pr_ye4; l_yei <= pr_ye4(16 downto 4);
                            v_v := pr_ye4 - 128;
                            pr_r0 <= resize(shift_right(v_v, 8), 9);
                            l_fy <= unsigned(v_v(7 downto 4));
                            v_t := pr_ye4 - 13752;
                            pr_trow <= resize(shift_right(v_t, 6), 11);
                            l_gv0 <= pr_ye4 - 15667;
                            if pr_below = '1' then l_sy8 <= pr_syb - resize(f_jz, 21); else l_sy8 <= pr_syb; end if;
                            pr_sq <= '1';
                        elsif pr_t = 4 then
                            if pr_r0(0) = '1' then pr_re <= resize(shift_right(pr_r0 + 1, 1), 8); l_r0p <= '1';
                            else pr_re <= resize(shift_right(pr_r0, 1), 8); l_r0p <= '0'; end if;
                            pr_ro <= resize(shift_right(pr_r0, 1), 8);
                            pr_sq <= '1';
                        elsif pr_t = 5 then
                            if pr_re >= 0 and pr_re <= 36 then
                                l_rv0 <= '1';
                                l_rb0 <= shift_left(resize(unsigned(pr_re(5 downto 0)), 10), 5) - shift_left(resize(unsigned(pr_re(5 downto 0)), 10), 2)
                                         - resize(unsigned(pr_re(5 downto 0)), 10);
                            else
                                l_rv0 <= '0'; l_rb0 <= (others => '0');
                            end if;
                            if pr_ro >= 0 and pr_ro <= 36 then
                                l_rv1 <= '1';
                                l_rb1 <= shift_left(resize(unsigned(pr_ro(5 downto 0)), 10), 5) - shift_left(resize(unsigned(pr_ro(5 downto 0)), 10), 2)
                                         - resize(unsigned(pr_ro(5 downto 0)), 10);
                            else
                                l_rv1 <= '0'; l_rb1 <= (others => '0');
                            end if;
                            if pr_trow >= 0 and pr_trow < 28 then
                                l_tok <= '1';
                                l_tb <= shift_left(resize(unsigned(pr_trow(4 downto 0)), 12), 6) + shift_left(resize(unsigned(pr_trow(4 downto 0)), 12), 5)
                                        + shift_left(resize(unsigned(pr_trow(4 downto 0)), 12), 3) + resize(unsigned(pr_trow(4 downto 0)), 12);
                            else
                                l_tok <= '0'; l_tb <= (others => '0');
                            end if;
                        elsif pr_t = 6 then
                            v_yi := pr_ye4(16 downto 4);
                            if v_yi >= 454 and v_yi < 470 then l_fx0 <= to_signed(511, 11); l_fx1 <= to_signed(642, 11);
                            elsif v_yi >= 440 and v_yi < 454 then l_fx0 <= to_signed(313, 11); l_fx1 <= to_signed(408, 11);
                            elsif v_yi >= 311 and v_yi < 384 then l_fx0 <= to_signed(407, 11); l_fx1 <= to_signed(421, 11);
                            else l_fx0 <= to_signed(0, 11); l_fx1 <= to_signed(0, 11); end if;
                            pr_yh1 <= f_mix1(unsigned(resize(shift_right(v_yi, 1), 16)) xor x"3C6F");
                            -- the squarer (shared with the pixel path) returned |dyq|^2
                            if p_sq6(13 downto 12) /= "00" then l_e <= (others => '1'); else l_e <= p_sq6(11 downto 0); end if;
                        elsif pr_t = 7 then
                            l_yh <= f_mix1(pr_yh1 + x"9E37");
                        end if;
                    when 10 =>
                        l_b4 <= f_sb(pr_sa);
                        r_ra <= resize(pr_sa & '0', 8);
                    when 11 => r_ra <= resize(pr_sa & '1', 8);
                    when 12 =>
                        if pr_sa = 7 then r_ra <= resize(pr_sa & '0', 8); else r_ra <= resize((pr_sa + 1) & '0', 8); end if;
                    when 13 =>
                        if pr_sa = 7 then r_ra <= resize(pr_sa & '1', 8); else r_ra <= resize((pr_sa + 1) & '1', 8); end if;
                        l_oa4 <= signed(r_rq);
                    when 14 => l_dsa <= shift_left(resize(signed(r_rq), 20), 2);
                    when 15 => l_ob4 <= signed(r_rq);
                    when 16 => l_dsb <= shift_left(resize(signed(r_rq), 20), 2);
                    when 17 => l_pb <= pr_pb;
                    when others => null;
                end case;
            end if;
        end if;
    end process p_prep;

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
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- Mode tables for the noise / brush DDAs (pixel increments, line-start phases, line steps)
    ---------------------------------------------------------------------------
    p_tab : process(s_tid, s_mir, s_sy)
        variable v_c : integer range 0 to 5;
    begin
            if f_sd(s_tid) then v_c := 0; elsif f_720(s_tid) then v_c := 2; else v_c := 4; end if;
            if s_mir = '1' then v_c := v_c + 1; end if;
            case v_c is
                when 0 => f_ni1 <= to_unsigned(5701, 16);  f_ni2 <= to_unsigned(3198, 16);  f_ni3 <= to_unsigned(1958, 16);
                          f_nx1 <= to_unsigned(12986, 16); f_nx2 <= to_unsigned(49211, 16); f_nx3 <= to_unsigned(9312, 16);
                          f_binc <= to_unsigned(626, 16);  f_bxk <= to_unsigned(61106, 16);
                when 1 => f_ni1 <= to_unsigned(59834, 16); f_ni2 <= to_unsigned(62337, 16); f_ni3 <= to_unsigned(63577, 16);
                          f_nx1 <= to_unsigned(52794, 16); f_nx2 <= to_unsigned(57531, 16); f_nx3 <= to_unsigned(42336, 16);
                          f_binc <= to_unsigned(64909, 16); f_bxk <= to_unsigned(53554, 16);
                when 2 => f_ni1 <= to_unsigned(4274, 16);  f_ni2 <= to_unsigned(2398, 16);  f_ni3 <= to_unsigned(1468, 16);
                          f_nx1 <= to_unsigned(50042, 16); f_nx2 <= to_unsigned(58747, 16); f_nx3 <= to_unsigned(36576, 16);
                          f_binc <= to_unsigned(470, 16);  f_bxk <= to_unsigned(51442, 16);
                when 3 => f_ni1 <= to_unsigned(61262, 16); f_ni2 <= to_unsigned(63138, 16); f_ni3 <= to_unsigned(64068, 16);
                          f_nx1 <= to_unsigned(15738, 16); f_nx2 <= to_unsigned(47995, 16); f_nx3 <= to_unsigned(15072, 16);
                          f_binc <= to_unsigned(65066, 16); f_bxk <= to_unsigned(63218, 16);
                when 4 => f_ni1 <= to_unsigned(2846, 16);  f_ni2 <= to_unsigned(1597, 16);  f_ni3 <= to_unsigned(977, 16);
                          f_nx1 <= to_unsigned(50042, 16); f_nx2 <= to_unsigned(58747, 16); f_nx3 <= to_unsigned(36576, 16);
                          f_binc <= to_unsigned(313, 16);  f_bxk <= to_unsigned(51442, 16);
                when others =>
                          f_ni1 <= to_unsigned(62689, 16); f_ni2 <= to_unsigned(63938, 16); f_ni3 <= to_unsigned(64558, 16);
                          f_nx1 <= to_unsigned(15738, 16); f_nx2 <= to_unsigned(47995, 16); f_nx3 <= to_unsigned(15072, 16);
                          f_binc <= to_unsigned(65222, 16); f_bxk <= to_unsigned(63218, 16);
            end case;
            case to_integer(s_sy) is
                when 512  => f_ny1 <= to_unsigned(1612, 16); f_ny2 <= to_unsigned(62146, 16); f_ny3 <= to_unsigned(5782, 16); f_byi <= to_unsigned(1012, 16);
                when 640  => f_ny1 <= to_unsigned(2015, 16); f_ny2 <= to_unsigned(61298, 16); f_ny3 <= to_unsigned(7227, 16); f_byi <= to_unsigned(1265, 16);
                when 683  => f_ny1 <= to_unsigned(2150, 16); f_ny2 <= to_unsigned(61013, 16); f_ny3 <= to_unsigned(7713, 16); f_byi <= to_unsigned(1349, 16);
                when 768  => f_ny1 <= to_unsigned(2418, 16); f_ny2 <= to_unsigned(60451, 16); f_ny3 <= to_unsigned(8673, 16); f_byi <= to_unsigned(1518, 16);
                when 1280 => f_ny1 <= to_unsigned(4030, 16); f_ny2 <= to_unsigned(57061, 16); f_ny3 <= to_unsigned(14455, 16); f_byi <= to_unsigned(2530, 16);
                when 1517 => f_ny1 <= to_unsigned(4776, 16); f_ny2 <= to_unsigned(55491, 16); f_ny3 <= to_unsigned(17131, 16); f_byi <= to_unsigned(2998, 16);
                when others => f_ny1 <= to_unsigned(1073, 16); f_ny2 <= to_unsigned(63278, 16); f_ny3 <= to_unsigned(3850, 16); f_byi <= to_unsigned(674, 16);
            end case;
    end process p_tab;

    ---------------------------------------------------------------------------
    -- Pixel path.  S0 = the tear-noise DDAs (loaded for pixel 0 at the g_start edge, aligned with
    -- g_avid); the map-x DDA runs 2 clocks later (S2), the brush DDA 13 (S13), the store-x DDA 15
    -- (S15).  Sk is k clocks after S0.  l_* are stable while a line is in flight.
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_u : signed(16 downto 0);
        variable v_c0, v_ce, v_co : signed(8 downto 0);
        variable v_g : signed(13 downto 0);
        variable v_gx : unsigned(19 downto 0);
        variable v_xi : signed(12 downto 0);
        variable v_w0, v_w1, v_w2, v_w3 : std_logic_vector(7 downto 0);
        variable v_tl, v_tr, v_bl, v_br : std_logic_vector(7 downto 0);
        variable v_rim : signed(15 downto 0);
        variable v_q : unsigned(14 downto 0);
        variable v_hx, v_hy : boolean;
        variable v_iref, v_ih, v_iv, v_idg, v_ioth : unsigned(4 downto 0);
        variable v_n : signed(12 downto 0);
        variable v_nm : signed(11 downto 0);
        variable v_fs : signed(12 downto 0);
        variable v_prm : std_logic_vector(23 downto 0);
        variable v_fam : unsigned(4 downto 0);
        variable v_sh : signed(9 downto 0);
        variable v_k : unsigned(1 downto 0);
        variable v_fl : std_logic_vector(1 downto 0);
        variable v_ou : unsigned(6 downto 0);
        variable v_ov : unsigned(5 downto 0);
        variable v_dn, v_feat, v_out, v_eye : std_logic;
        variable v_tx : signed(9 downto 0);
        variable v_off : signed(9 downto 0);
        variable v_f3 : unsigned(13 downto 0);
        variable v_au : signed(7 downto 0);
        variable v_ring : unsigned(7 downto 0);
        variable v_rp : unsigned(15 downto 0);
        variable v_gl : signed(10 downto 0);
        variable v_su : signed(14 downto 0);
        variable v_sv : signed(13 downto 0);
        variable v_st : signed(16 downto 0);
        variable v_l9 : signed(8 downto 0);
        variable v_cu, v_c1 : unsigned(6 downto 0);
        variable v_cv, v_er : unsigned(5 downto 0);
        variable v_qtl, v_qtr, v_qbl, v_qbr : unsigned(3 downto 0);
        variable v_l : signed(18 downto 0);
        variable v_lv : signed(11 downto 0);
        variable v_li : unsigned(3 downto 0);
        variable v_y, v_cu2, v_cv2 : signed(11 downto 0);
        variable v_sc : unsigned(11 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            g_dl <= g_dl(14 downto 0) & g_start;
            -- S0: tear-noise DDAs
            if g_start = '1' then
                p_n1 <= l_ph1; p_n2 <= l_ph2;
            else
                p_n1 <= p_n1 + f_ni1; p_n2 <= p_n2 + f_ni2;
            end if;
            -- S1: triangles
            p_t1 <= f_tri8(p_n1); p_t2 <= f_tri8(p_n2);
            -- S2: which side of the line's strip boundary (+-4 map units of rag); noise sum; map x
            if resize(l_ye4, 18) + resize(shift_right(signed(resize(p_t1, 10)) + signed(resize(p_t2, 10)) - 256, 2), 18)
               >= resize(l_b4, 18) then p_sel2 <= '1'; else p_sel2 <= '0'; end if;
            p_ns2 <= signed(resize(p_t1, 11)) + signed(resize(p_t2, 11)) - 256;
            if g_dl(1) = '1' then p_x8 <= f_xs0; else p_x8 <= p_x8 + resize(f_sxs, 21); end if;
            -- S3: torn map x (Q.4)
            if p_sel2 = '1' then p_xt3 <= resize(shift_right(p_x8, 4), 17) - resize(l_ob4, 17);
            else p_xt3 <= resize(shift_right(p_x8, 4), 17) - resize(l_oa4, 17); end if;
            p_sel(3) <= p_sel2; p_ns3 <= p_ns2;
            for k in 4 to 15 loop p_sel(k) <= p_sel(k - 1); end loop;
            -- S4: map cell / fraction, integer x, glyph / teeth x, hash round 1, forks, weave
            v_u := p_xt3 - 128;
            p_c04 <= resize(shift_right(v_u, 8), 9);
            p_fx4 <= unsigned(v_u(7 downto 4));
            v_xi := resize(shift_right(p_xt3, 4), 13);
            p_xti4 <= v_xi;
            v_u := p_xt3 - 8006;
            if v_u >= 0 and v_u < 2048 then p_gx4 <= unsigned(v_u(11 downto 0)); p_gok4 <= '1';
            else p_gx4 <= (others => '0'); p_gok4 <= '0'; end if;
            p_tc4 <= resize(shift_right(p_xt3 - 720, 6), 11);
            p_hw4 <= f_mix1(unsigned(resize(shift_right(p_xt3, 5), 16)) xor l_yh);
            if v_xi >= l_fx0 and v_xi < l_fx1 then p_fk(4) <= '1'; else p_fk(4) <= '0'; end if;
            p_ns4 <= p_ns3;
            for k in 5 to 15 loop p_fk(k) <= p_fk(k - 1); end loop;
            -- S5: map bank addresses into the ROMs; column validity; cup offset (or the prep's dyq);
            -- glyph v; hash round 2a
            v_c0 := p_c04;
            if v_c0(0) = '1' then v_ce := shift_right(v_c0 + 1, 1); else v_ce := shift_right(v_c0, 1); end if;
            v_co := shift_right(v_c0, 1);
            m_a0 <= l_rb0 + resize(unsigned(v_ce(4 downto 0)), 10);
            m_a1 <= l_rb0 + resize(unsigned(v_co(4 downto 0)), 10);
            m_a2 <= l_rb1 + resize(unsigned(v_ce(4 downto 0)), 10);
            m_a3 <= l_rb1 + resize(unsigned(v_co(4 downto 0)), 10);
            if v_ce >= 0 and v_ce <= 26 then p_ok5(0) <= '1'; else p_ok5(0) <= '0'; end if;
            if v_co >= 0 and v_co <= 26 then p_ok5(1) <= '1'; else p_ok5(1) <= '0'; end if;
            p_cp5 <= v_c0(0); p_fx5 <= p_fx4;
            if pr_sq = '1' then
                p_uu5 <= pr_dyq;
            else
                v_g := resize(p_xti4, 14) - resize(shift_right(l_xc4, 4), 14);
                if v_g > 127 then p_uu5 <= to_signed(127, 8); elsif v_g < -127 then p_uu5 <= to_signed(-127, 8);
                else p_uu5 <= v_g(7 downto 0); end if;
            end if;
            v_gx := shift_left(resize(p_gx4, 20), 2) + resize(p_gx4, 20);
            p_gv5 <= l_gv0 + signed(resize(v_gx(19 downto 5), 17));
            p_gu5 <= p_gx4(10 downto 5); p_gok5 <= p_gok4;
            p_tc5 <= p_tc4;
            p_h5 <= p_hw4 + x"6A09";
            p_ns5 <= p_ns4;
            -- S6: square, detail-ROM address (glyph right of the teeth), hash round 2b
            v_au := abs(p_uu5);
            p_sq6 <= unsigned(v_au(6 downto 0)) * unsigned(v_au(6 downto 0));
            p_uu6 <= p_uu5;
            if p_gok5 = '1' then
                d_a <= to_unsigned(2944, 12) + shift_left(resize(unsigned(p_gv5(9 downto 5)), 12), 5) + resize(p_gu5(5 downto 1), 12);
            else
                d_a <= l_tb + resize(unsigned(p_tc5(6 downto 0)), 12);
            end if;
            if p_gok5 = '1' and p_gv5 >= 0 and p_gv5 < 640 then p_gl6 <= '1'; else p_gl6 <= '0'; end if;
            if p_gok5 = '0' and l_tok = '1' and p_tc5 >= 0 and p_tc5 < 105 then p_tt6 <= '1'; else p_tt6 <= '0'; end if;
            p_gs6 <= p_gu5(0);
            p_h6 <= f_mix1(p_h5);
            p_ok6 <= p_ok5; p_cp6 <= p_cp5; p_fx6 <= p_fx5; p_ns6 <= p_ns5;
            -- S7: soup quadratic; (map bank data m_r* is S7-aligned)
            p_qs7 <= resize(p_sq6, 15) + resize(l_e, 15);
            p_uu7 <= p_uu6; p_h7 <= p_h6;
            p_gl7 <= p_gl6; p_tt7 <= p_tt6; p_gs7 <= p_gs6;
            p_ok7 <= p_ok6; p_cp7 <= p_cp6; p_fx7 <= p_fx6; p_ns7 <= p_ns6;
            -- S8: corners by row / column parity (invalid cells read as ground, d = 7); cup tests and
            -- shade; noise raw; the hash bits that ride on
            v_w0 := m_r0; v_w1 := m_r1; v_w2 := m_r2; v_w3 := m_r3;
            if l_rv0 = '0' or p_ok7(0) = '0' then v_w0 := "00000111"; end if;
            if l_rv0 = '0' or p_ok7(1) = '0' then v_w1 := "00000111"; end if;
            if l_rv1 = '0' or p_ok7(0) = '0' then v_w2 := "00000111"; end if;
            if l_rv1 = '0' or p_ok7(1) = '0' then v_w3 := "00000111"; end if;
            if l_r0p = '0' then
                if p_cp7 = '0' then v_tl := v_w0; v_tr := v_w1; v_bl := v_w2; v_br := v_w3;
                else v_tl := v_w1; v_tr := v_w0; v_bl := v_w3; v_br := v_w2; end if;
            else
                if p_cp7 = '0' then v_tl := v_w2; v_tr := v_w3; v_bl := v_w0; v_br := v_w1;
                else v_tl := v_w3; v_tr := v_w2; v_bl := v_w1; v_br := v_w0; end if;
            end if;
            p_i00 <= unsigned(v_tl(7 downto 3)); p_d00 <= unsigned(v_tl(2 downto 0));
            p_i01 <= unsigned(v_tr(7 downto 3)); p_d01 <= unsigned(v_tr(2 downto 0));
            p_i10 <= unsigned(v_bl(7 downto 3)); p_d10 <= unsigned(v_bl(2 downto 0));
            p_i11 <= unsigned(v_br(7 downto 3)); p_d11 <= unsigned(v_br(2 downto 0));
            p_fx8 <= p_fx7;
            p_raw8 <= resize(p_ns7, 12) - resize(shift_right(p_ns7, 3), 12)
                      + resize(shift_right(signed(resize(p_h7(7 downto 0), 10)) - 128, 2), 12);
            v_rim := signed(resize(p_qs7, 16)) + shift_left(resize(p_uu7, 16), 3) + shift_left(resize(p_uu7, 16), 1) + resize(p_uu7, 16);
            ft(8) <= C_FT0;
            if v_rim < 2901 and l_cupok = '1' then
                ft(8).rim <= '1';
                if p_qs7 < 1870 then ft(8).soup <= '1'; end if;
            end if;
            if p_qs7 <= 1870 then ft(8).sh <= to_signed(24, 9);                         -- rim shade
            elsif p_qs7 >= 1870 + 1024 then ft(8).sh <= to_signed(-40, 9);
            else v_q := p_qs7 - 1870; ft(8).sh <= 24 - signed(resize(v_q(9 downto 4), 9)); end if;
            if p_qs7 < 1870 then                                                        -- soup shade
                if s_eye = '1' then
                    v_q := resize(p_qs7, 15) + shift_left(resize(p_qs7, 15), 1);
                    ft(8).sh <= -signed(resize(v_q(14 downto 8), 9));
                else
                    v_q := shift_left(resize(p_qs7, 15), 2) + resize(p_qs7, 15);
                    ft(8).sh <= 16 - signed(resize(v_q(14 downto 8), 9));
                end if;
            end if;
            ft(8).uu <= p_uu7(6 downto 0);
            ft(8).hb <= p_h7(15 downto 8);
            if p_h7(5 downto 4) /= "00" then ft(8).hf <= '1'; else ft(8).hf <= '0'; end if;
            p_gl8 <= p_gl7; p_tt8 <= p_tt7; p_gs8 <= p_gs7;
            -- S9: reference (nearest) cell, the 'other' id, signed distances; rag noise; detail code
            v_hx := p_fx8 >= 8; v_hy := l_fy >= 8;
            if v_hy then
                if v_hx then v_iref := p_i11; v_ih := p_i10; v_iv := p_i01; v_idg := p_i00;
                else v_iref := p_i10; v_ih := p_i11; v_iv := p_i00; v_idg := p_i01; end if;
            else
                if v_hx then v_iref := p_i01; v_ih := p_i00; v_iv := p_i11; v_idg := p_i10;
                else v_iref := p_i00; v_ih := p_i01; v_iv := p_i10; v_idg := p_i11; end if;
            end if;
            if v_ih /= v_iref then v_ioth := v_ih; else v_ioth := v_iv; end if;
            p_iref9 <= v_iref; p_ioth9 <= v_ioth;
            if p_i00 = v_iref then p_s00 <= signed(resize(p_d00, 4)); else p_s00 <= -signed(resize(p_d00, 4)); end if;
            if p_i01 = v_iref then p_s01 <= signed(resize(p_d01, 4)); else p_s01 <= -signed(resize(p_d01, 4)); end if;
            if p_i10 = v_iref then p_s10 <= signed(resize(p_d10, 4)); else p_s10 <= -signed(resize(p_d10, 4)); end if;
            if p_i11 = v_iref then p_s11 <= signed(resize(p_d11, 4)); else p_s11 <= -signed(resize(p_d11, 4)); end if;
            v_n := resize(shift_right(p_raw8, 4), 8) * signed('0' & f_g5(3 downto 0));
            p_n9 <= resize(v_n, 12);
            p_fx9 <= p_fx8;
            ft(9) <= ft(8);
            if p_tt8 = '1' then ft(9).code <= d_r; else ft(9).code <= "00"; end if;
            if p_gl8 = '1' and ((p_gs8 = '1' and d_r(1) = '1') or (p_gs8 = '0' and d_r(0) = '1')) then ft(9).glyb <= '1';
            else ft(9).glyb <= '0'; end if;
            for k in 10 to 14 loop ft(k) <= ft(k - 1); end loop;
            -- S10: lerp across (x16)
            p_top10 <= shift_left(resize(p_s00, 8), 3) + resize((resize(p_s01, 5) - resize(p_s00, 5)) * signed('0' & p_fx9(3 downto 1)), 8);
            p_bot10 <= shift_left(resize(p_s10, 8), 3) + resize((resize(p_s11, 5) - resize(p_s10, 5)) * signed('0' & p_fx9(3 downto 1)), 8);
            p_n10 <= p_n9; p_iref10 <= p_iref9; p_ioth10 <= p_ioth9;
            -- S11: lerp down (x256)
            p_f11 <= shift_left(resize(p_top10, 14), 3) + resize((resize(p_bot10, 9) - resize(p_top10, 9)) * signed('0' & l_fy(3 downto 1)), 14);
            p_n11 <= p_n10; p_iref11 <= p_iref10; p_ioth11 <= p_ioth10;
            -- S12: + tear noise (x1/4 on the painted head edge, x1/8 on the scissor-cut features)
            if p_iref11 = 0 or p_ioth11 = 0 then v_nm := shift_right(p_n11, 2);
            elsif p_iref11 >= 23 or p_ioth11 >= 23 then v_nm := shift_right(p_n11, 3);
            else v_nm := p_n11; end if;
            p_fs12 <= resize(p_f11, 13) + resize(v_nm, 13);
            p_iref12 <= p_iref11; p_ioth12 <= p_ioth11;
            -- S13: the scrap at this pixel, the one across the nearest edge, the distance
            if p_fs12 >= 0 then
                p_pid13 <= p_iref12; p_oth13 <= p_ioth12; p_fo13 <= unsigned(p_fs12(11 downto 0)); p_fp13 <= '1';
            else
                v_fs := -p_fs12;
                p_pid13 <= p_ioth12; p_oth13 <= p_iref12; p_fo13 <= unsigned(v_fs(11 downto 0)); p_fp13 <= '0';
            end if;
            -- S14: scrap parameters; edge rings of the map features
            v_prm := C_PRM(to_integer(p_pid13));
            v_fam := unsigned(v_prm(4 downto 0)); v_sh := (others => '0');
            if p_pid13 = 24 and p_oth13 /= 24 and p_oth13 /= 25 and p_fo13 < 58 then v_fam := to_unsigned(20, 5); v_sh := to_signed(-48, 10); end if;
            if p_pid13 = 23 and p_oth13 < 23 and p_fo13 < 86 then v_fam := to_unsigned(21, 5); v_sh := to_signed(64, 10); end if;
            if p_pid13 = 26 and p_oth13 = 27 and p_fo13 < 32 then v_fam := to_unsigned(27, 5); v_sh := to_signed(32, 10); end if;
            if p_pid13 = 25 and p_fo13 >= 130 then v_fam := to_unsigned(23, 5); end if;                       -- garnish core
            if p_pid13 = 30 then
                v_f3 := resize(p_fo13, 14) + shift_left(resize(p_fo13, 14), 1);
                if v_f3(13 downto 8) >= 48 then v_sh := to_signed(16, 10);
                else v_sh := signed(resize(v_f3(13 downto 8), 10)) - 32; end if;
            end if;
            if p_pid13 = 29 then
                if p_fo13 >= 256 then v_sh := to_signed(16, 10); else v_sh := signed(resize(p_fo13(7 downto 3), 10)) - 16; end if;
            end if;
            if p_pid13 = 28 then v_sh := to_signed(-16, 10); end if;
            p_fam14 <= v_fam; p_sh14 <= v_sh;
            p_k14 <= unsigned(v_prm(6 downto 5)); p_fl14 <= v_prm(8 downto 7);
            p_ou14 <= unsigned(v_prm(15 downto 9)); p_ov14 <= unsigned(v_prm(21 downto 16));
            p_dn14 <= v_prm(22); p_frok14 <= v_prm(23);
            if p_pid13 >= 26 and p_pid13 <= 30 then p_inpl14 <= '1'; else p_inpl14 <= '0'; end if;
            p_pid14 <= p_pid13; p_fo14 <= p_fo13; p_fp14 <= p_fp13;
            if p_oth13 /= 0 and p_oth13 < 23 then p_fe14 <= '1'; else p_fe14 <= '0'; end if;
            if p_pid13 = 0 and p_fo13 < 192 then p_gr14 <= '1'; else p_gr14 <= '0'; end if;
            -- S13/S14: the brush DDA (S13) and its triangle (S14)
            if g_dl(12) = '1' then p_nb <= l_pb; else p_nb <= p_nb + f_binc; end if;
            p_tb14 <= f_tri8(p_nb);
            -- S15: drawn features over the map (forks, cup, teeth, glyph); ground, fringe, texture;
            -- the store-x DDA
            v_fam := p_fam14; v_sh := p_sh14; v_k := p_k14; v_fl := p_fl14; v_ou := p_ou14; v_ov := p_ov14;
            v_dn := p_dn14; v_feat := '0'; v_eye := '0';
            if p_inpl14 = '1' and p_fk(14) = '1' then v_fam := to_unsigned(27, 5); v_sh := to_signed(16, 10); end if;
            if ft(14).rim = '1' then
                v_feat := '1'; v_fam := to_unsigned(19, 5); v_sh := resize(ft(14).sh, 10);
                if ft(14).soup = '1' then
                    if s_eye = '1' then v_fam := to_unsigned(0, 5);                         -- obsidian
                    else v_fam := to_unsigned(18, 5); end if;
                end if;
            end if;
            if ft(14).code /= "00" then
                v_feat := '1'; v_fam := to_unsigned(28, 5); v_k := "00"; v_fl := "00";
                v_ou := to_unsigned(5, 7); v_ov := to_unsigned(38, 6); v_dn := '0';
                case ft(14).code is
                    when "01" => v_sh := to_signed(-96, 10);
                    when "11" => v_sh := to_signed(48, 10);
                    when others => v_sh := (others => '0');
                end case;
            end if;
            if ft(14).glyb = '1' and p_pid14 = 31 then v_fam := to_unsigned(30, 5); end if;
            if (p_pid14 = 0 and v_feat = '0') or l_gap = '1' then v_out := '1'; else v_out := '0'; end if;
            p_k15 <= v_k; p_fl15 <= v_fl; p_ou15 <= v_ou; p_ov15 <= v_ov; p_dn15 <= v_dn; p_eye15 <= v_eye;
            p_uu15 <= ft(14).uu;
            -- ground level: brush wave + paint rings round the head edge
            if p_gr14 = '1' then
                v_rp := shift_left(resize(p_fo14, 16), 8) + shift_left(resize(p_fo14, 16), 10);
                v_ring := resize(shift_right(f_tri8(v_rp), 3), 8);
            else
                v_ring := (others => '0');
            end if;
            p_gl15 <= -16 + signed(resize(shift_right(signed(resize(p_tb14, 10)) - 128, 1), 11)) + signed(resize(v_ring, 11));
            p_fam15c <= v_fam; p_sh15c <= v_sh; p_out15c <= v_out; p_hb15 <= ft(14).hb; p_hf15 <= ft(14).hf;
            if s_frg = '0' and p_fp14 = '1' and p_fo14 < 35 + resize(ft(14).hb(7 downto 4), 12) and v_out = '0' and v_feat = '0'
               and p_pid14 < 23 and p_pid14 /= 0 and p_frok14 = '1' and p_fe14 = '1' and ft(14).hf = '1' then
                p_frg15 <= '1';
            else
                p_frg15 <= '0';
            end if;
            -- S16: surface texture by family (grain / specks), level offset (+ dither)
            case C_TEX(to_integer(p_fam15c)) is
                when "001" | "010" | "011" | "101" =>
                    v_tx := signed(resize(shift_left(resize(p_hb15(2 downto 0), 6), 3) - resize(p_hb15(2 downto 0), 6), 10)) - 25;
                when "100" => if p_hb15(7 downto 3) = "00000" and p_hf15 = '0' then v_tx := to_signed(112, 10); else v_tx := (others => '0'); end if;
                when others => v_tx := (others => '0');
            end case;
            if p_out15c = '1' then v_off := resize(p_gl15, 10); else v_off := p_sh15c + v_tx; end if;
            cb(16).fam <= p_fam15c; cb(16).outg <= p_out15c; cb(16).frg <= p_frg15;
            cb(16).off <= v_off + signed(resize(p_hb15(5 downto 3) & '0', 10));          -- + dither
            for k in 17 to 23 loop cb(k) <= cb(k - 1); end loop;
            if g_dl(14) = '1' then p_s8 <= f_s0(16 downto 0); else p_s8 <= p_s8 + resize(f_dspx, 17); end if;
            -- S16: torn store x at the scrap's scale; store y at its scale
            if p_sel(15) = '1' then v_st := p_s8 - l_dsb(16 downto 0); else v_st := p_s8 - l_dsa(16 downto 0); end if;
            p_sxs16 <= shift_right(v_st, to_integer(p_k15));
            p_sys16 <= shift_right(l_sy8(15 downto 0), to_integer(p_k15));
            p_fl16 <= p_fl15; p_ov16 <= p_ov15;
            if p_dn15 = '1' then p_bu16 <= signed(shift_left(resize(p_ou15, 15), 8)) - signed(f_tdr(14 downto 0));
            else p_bu16 <= signed(shift_left(resize(p_ou15, 15), 8)) + signed(f_tdr(14 downto 0)); end if;
            -- S17: flips, crop offsets, drift -> the store address (nearest)
            if p_fl16(0) = '1' then v_su := p_bu16 - p_sxs16(14 downto 0); else v_su := p_bu16 + p_sxs16(14 downto 0); end if;
            if p_fl16(1) = '1' then v_sv := signed(shift_left(resize(p_ov16, 14), 8)) - p_sys16(13 downto 0);
            else v_sv := signed(shift_left(resize(p_ov16, 14), 8)) + p_sys16(13 downto 0); end if;
            b_ra <= unsigned(v_sv(13 downto 8)) & unsigned(v_su(14 downto 8));
            -- S18: RAM.  S19: fabric
            p_q19 <= unsigned(b_q);
            -- S20: ink contrast
            v_l9 := signed(resize(p_q19 & "000", 9)) - 64;                                   -- (L - 128) / 2
            p_cx20 <= resize(shift_right(v_l9 * signed('0' & f_c16), 3), 11);
            -- S21: level = 128 + contrast + offset (shade, texture, dither; the ground's brush) -> ink index
            if cb(20).outg = '1' then v_lv := 128 + resize(cb(20).off, 12);
            else v_lv := 128 + resize(p_cx20, 12) + resize(cb(20).off, 12); end if;
            if v_lv < 0 then v_li := (others => '0'); elsif v_lv > 255 then v_li := (others => '1');
            else v_li := unsigned(v_lv(7 downto 4)); end if;
            if cb(20).outg = '1' then t_a <= "11111" & v_li; else t_a <= cb(20).fam & v_li; end if;
            -- S22: ink ROM.  S23: fabric.  S24: compose (ground + paint offset; white torn fringe)
            v_y := signed(resize(unsigned(t_ry) & "00", 12)); v_cu2 := signed(resize(unsigned(t_ru) & "00", 12));
            v_cv2 := signed(resize(unsigned(t_rv) & "00", 12));
            if cb(23).outg = '1' then
                v_y := v_y + resize(f_pcy, 12); v_cu2 := v_cu2 + resize(f_pcu, 12); v_cv2 := v_cv2 + resize(f_pcv, 12);
            end if;
            if cb(23).frg = '1' then
                v_y := to_signed(881, 12); v_cu2 := to_signed(500, 12); v_cv2 := to_signed(530, 12);
            end if;
            if v_y < 64 then p_y24 <= to_unsigned(64, 10); elsif v_y > 1020 then p_y24 <= to_unsigned(1020, 10);
            else p_y24 <= unsigned(v_y(9 downto 0)); end if;
            if v_cu2 < 16 then p_u24 <= to_unsigned(16, 10); elsif v_cu2 > 1008 then p_u24 <= to_unsigned(1008, 10);
            else p_u24 <= unsigned(v_cu2(9 downto 0)); end if;
            if v_cv2 < 16 then p_v24 <= to_unsigned(16, 10); elsif v_cv2 > 1008 then p_v24 <= to_unsigned(1008, 10);
            else p_v24 <= unsigned(v_cv2(9 downto 0)); end if;

            -- 25..30: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent 8-sums on
            -- 8-bit chroma, luma [1 2 1]); 31: luma toward black (white 940 -> 777), the gate; 32: pad
            s_csu(0) <= p_u24(9 downto 2); s_csv(0) <= p_v24(9 downto 2);
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + resize(p_u24(9 downto 2), 11) - resize(s_csu(7), 11);
            s_boxv <= s_boxv + resize(p_v24(9 downto 2), 11) - resize(s_csv(7), 11);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 12) + s_boxud; s_u30 <= v_sc(11 downto 2);
            v_sc := resize(s_boxv, 12) + s_boxvd; s_v30 <= v_sc(11 downto 2);
            s_y25 <= p_y24; s_y26 <= s_y25; s_y27 <= s_y26; s_y28 <= s_y27;
            s_y29 <= s_y28; s_y30x <= s_y29;
            v_sy := resize(s_y28, 12) + shift_left(resize(s_y29, 12), 1) + resize(s_y30x, 12);
            s_yf30 <= v_sy(11 downto 2);
            if s_yf30 > 64 then v_dl := s_yf30 - 64; else v_dl := (others => '0'); end if;
            if s_dq(0) = '1' then
                s_y31 <= s_yf30 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u31 <= s_u30; s_v31 <= s_v30;
            else
                s_y31 <= to_unsigned(64, 10); s_u31 <= to_unsigned(512, 10); s_v31 <= to_unsigned(512, 10);
            end if;
            s_y32 <= s_y31; s_u32 <= s_u31; s_v32 <= s_v31;
        end if;
    end process p_pix;

    p_wr : process(clk)
        variable v_wu : unsigned(20 downto 0);
        variable v_j : unsigned(3 downto 0);
        variable v_q : signed(17 downto 0);
        variable v_ad : unsigned(4 downto 0);
        variable v_cc, v_r : unsigned(7 downto 0);
        variable v_g : signed(11 downto 0);
        variable v_t1, v_t2, v_t3, v_t4, v_t5 : unsigned(7 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_x : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 1: store-column DDA over the whole line, sample rank
            w_winp <= w_win;
            if data_in.avid = '1' and s_avq = '0' then
                w_wu <= (others => '0'); w_win <= '1'; w_nw <= '1'; v_j := (others => '0');
            elsif data_in.avid = '1' and w_win = '1' then
                v_wu := resize(w_wu, 21) + resize(s_kw, 21);
                w_wu <= v_wu(19 downto 0);
                if v_wu(19) = '1' then w_win <= '0'; end if;
                if v_wu(18 downto 12) /= w_wu(18 downto 12) then
                    w_nw <= '1'; v_j := (others => '0');
                else
                    w_nw <= '0'; if w_j /= 15 then v_j := w_j + 1; else v_j := w_j; end if;
                end if;
            else
                w_win <= '0'; w_nw <= '0'; v_j := w_j;
            end if;
            w_j <= v_j;
            if v_j < shift_left(to_unsigned(1, 4), to_integer(s_na)) then w_smp <= '1'; else w_smp <= '0'; end if;
            w_y1 <= unsigned(data_in.y(9 downto 2));

            -- stage 2: column sums and column-done events
            w_dn <= '0';
            if (w_win = '1' and w_nw = '1' and w_winp = '1') or (w_win = '0' and w_winp = '1') then
                w_dn <= '1'; w_dc <= w_colq; w_dhs <= w_hs;
            end if;
            if w_win = '1' then w_colq <= w_wu(18 downto 12); end if;
            if w_nw = '1' then
                if w_smp = '1' then w_hs <= resize(w_y1, 14); else w_hs <= (others => '0'); end if;
                a_ra <= w_wu(18 downto 12);
            elsif w_win = '1' and w_smp = '1' then
                w_hs <= w_hs + resize(w_y1, 14);
            end if;
            w_aq <= unsigned(a_q);

            -- Osc photo, stage 3 (aligned with w_sc7): phases of five triangle waves and a hash
            v_cc := ('0' & w_dc) + f_tos(11 downto 4);
            v_r := "00" & w_row;
            o_p1 <= shift_left(v_cc, 2) + v_cc + shift_left(v_r, 2) + shift_left(v_r, 1);
            o_p2 <= shift_left(v_cc, 3) - v_cc - shift_left(v_r, 3) - shift_left(v_r, 1) - v_r + 77;
            -- stage 4 (aligned with w_sc8): triangles, the broad terms, the blob product
            v_t1 := f_t8(o_p1); v_t2 := f_t8(o_p2);
            v_x := resize(signed(resize(v_t1, 11)) + signed(resize(v_t2, 11)) - 256, 15);
            v_s := resize(shift_right(shift_left(v_x, 4) + v_x, 6), 11);
            o_s4 <= v_s;

            -- stage 3: add the earlier lines' partial sum
            w_sv7 <= '0';
            if w_dn = '1' and w_lsmp = '1' and w_rok = '1' then
                w_sv7 <= '1'; w_sc7 <= w_dc;
                if w_lfst = '1' then w_sum <= resize(w_dhs, 16); else w_sum <= w_aq + resize(w_dhs, 16); end if;
            end if;
            -- stage 4: partial sum back, or the finished store pixel
            a_we <= '0'; b_we <= '0'; w_st8 <= '0';
            if w_sv7 = '1' then
                if w_llst = '1' then
                    w_avg8 <= resize(shift_right(w_sum, to_integer(s_na) + to_integer(s_nb)), 9);
                    w_st8 <= '1'; w_sc8 <= w_sc7; w_row8 <= w_row;
                else
                    a_we <= '1'; a_wa <= w_sc7; a_wd <= std_logic_vector(w_sum);
                end if;
            end if;
            -- stage 5: quantize to 4 bits (or the Osc photo) and write the store
            if w_st8 = '1' then
                if s_osc = '1' then
                    v_g := 108 + resize(o_s4, 12);
                    if v_g < 0 then b_wd <= "0000"; elsif v_g > 255 then b_wd <= "1111";
                    else b_wd <= std_logic_vector(v_g(7 downto 4)); end if;
                else
                    v_q := signed(resize(w_avg8, 18)) - 16;
                    v_q := shift_right(v_q + shift_right(v_q, 3), 4);                          -- x 9/128
                    if v_q < 0 then b_wd <= "0000";
                    elsif v_q > 15 then b_wd <= "1111";
                    else b_wd <= std_logic_vector(v_q(3 downto 0)); end if;
                end if;
                b_wa <= w_row8 & w_sc8; b_we <= '1';
            end if;
            -- line state: the next row advances at the source avid fall, is committed at the rise,
            -- and is reset in the zone
            if data_in.avid = '1' and s_avq = '0' then
                w_wv <= w_wvn;
                w_row <= w_wvn(19 downto 14);
                w_rok <= not w_wvn(20);
                if w_wvn(19 downto 14) /= w_rown then
                    v_ad := (others => '0');
                elsif w_lrkn = 31 then
                    v_ad := to_unsigned(31, 5);
                else
                    v_ad := w_lrkn + 1;
                end if;
                w_lrkn <= v_ad; w_rown <= w_wvn(19 downto 14);
                if v_ad < shift_left(to_unsigned(1, 5), to_integer(s_nb)) then w_lsmp <= '1'; else w_lsmp <= '0'; end if;
                if v_ad = 0 then w_lfst <= '1'; else w_lfst <= '0'; end if;
                if v_ad = shift_left(to_unsigned(1, 5), to_integer(s_nb)) - 1 then w_llst <= '1'; else w_llst <= '0'; end if;
            elsif s_zone = '1' then
                w_wvn <= (others => '0'); w_lrkn <= (others => '0'); w_rown <= (others => '1');
            elsif data_in.avid = '0' and s_avq = '1' then
                w_wvn <= w_wvn + resize(s_kr, 21);
            end if;
        end if;
    end process p_wr;

    data_out.y       <= std_logic_vector(s_y32);
    data_out.u       <= std_logic_vector(s_u32);
    data_out.v       <= std_logic_vector(s_v32);
    data_out.hsync_n <= s_do(3);
    data_out.vsync_n <= s_do(2);
    data_out.field_n <= s_do(1);
    data_out.avid    <= s_do(0);

end architecture cutout;
