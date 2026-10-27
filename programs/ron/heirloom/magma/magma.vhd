-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Magma v0.5.0 -- Heirloom #29, rebuilt from the photo WIN_20230515_00_46_28.
--
-- The photo: fanned cyan/pink streaks sweeping down-right in the top half and blue/magenta ")"
-- bands below, red pockets stacked in the bands, and big lava blobs -- yellow with red rims at
-- the top, red at the bottom -- in a lattice with a row of pointed red lenses across the middle
-- and a red band along the bottom edge.  It is a three-channel colourizer on one band field:
--   * the bands are ONE line-locked sawtooth oscillator (phase 0 at 247 virtual px left of the
--     picture) whose frequency is swept per line by a vertical curve: 134 px period at the top,
--     198 px at y ~ 700, 181 px at the bottom, so the stripes fan out from the left edge;
--   * R and B are keys of that field (blue/cyan < magenta < red), G falls with it and with the
--     height (cyan sky at the top, deep blue below);
--   * the lava is a separate key whose shape was fitted to the photo, with its own green
--     channel (yellow cores, red rims).
-- The program: the band oscillator, a static band level BG, the lava key FR and its green GB
-- (quadratic B-splines on a 36-virtual-px grid, magma.py), red pockets riding the bands (a
-- per-band, per-slot table, rising like bubbles), a fine contour ripple and a per-line comb
-- jitter; the colourizer, the lava mix, RGB -> YUV.
--
--   K1  Stripes  how many bands (0.5x .. 2x; centre = the photo)
--   K2  Fan      how far the stripes fan out (0 = straight vertical bands; centre = the photo)
--   K3  Flow     band drift, bipolar (centre +-8 = still): + streams the stripes right
--   K4  Pockets  the red pockets in the bands (0 = none, centre = the photo, max = every slot)
--   K5  Melt     how far the video (or Osc) bends the bands and swells the lava
--   K6  Sky      the cyan/blue horizon: low = night blue/red, high = cyan/yellow
--   S7  Cores    Hot (yellow cores, the photo) / Flat (red lava)
--   S8  Ripple   Fine (the photo's fingerprint contours) / Coarse (wide topographic rings)
--   S9  Mirror   Off / On: the stripes mirrored about the centre (chevrons)
--   S10 Palette  Magma / Ice
--   S11 Source   Video / Osc
--   P12 LAVA     0 = no lava (a calm striped flow), 50 % = the photo, 100 % = the lava floods
--
-- Virtual 1080-line geometry (the same picture at SD, 720p and 1080; SD shows the central 4:3).
-- Generated avid; output conditioning (luma [1 2 1], chroma box pair [1 2 .. 2 1], luma to
-- ~777) before the blanking gate.  Latency 28 clocks in every mode.
-- Generated colours in the hardware convention (U carries Cr, V carries Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.magma_pkg.all;
use work.all;

architecture magma of program_top is
    constant C_LAT : integer := 28;   -- matches processing_delay_clks

    -- per-timing-ID constants
    -- kx: spline cells (36 virtual px) per pixel, Q16; c0: cell coordinate of pixel 0, Q16
    function f_kx(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(3641, 12);
            when 5 | 6 | 14     => return to_unsigned(2731, 12);
            when others         => return to_unsigned(1820, 12);
        end case;
    end function;
    function f_c0(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(469675, 23);
            when others         => return to_unsigned(32768, 23);
        end case;
    end function;
    -- ky: cells per line, Q16
    function f_ky(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(8091, 14);
            when 4          => return to_unsigned(4096, 14);
            when 8          => return to_unsigned(6827, 14);
            when 12         => return to_unsigned(3413, 14);
            when 5 | 6 | 14 => return to_unsigned(2731, 14);
            when 1 | 2 | 10 => return to_unsigned(3641, 14);
            when others     => return to_unsigned(1820, 14);
        end case;
    end function;
    -- vq: virtual lines per line, Q8
    function f_vq(t : std_logic_vector(3 downto 0)) return unsigned is
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
    -- ksl: pocket slots (44 virtual lines) per line, Q16
    function f_ksl(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(6620, 13);
            when 4          => return to_unsigned(3351, 13);
            when 8          => return to_unsigned(5585, 13);
            when 12         => return to_unsigned(2793, 13);
            when 5 | 6 | 14 => return to_unsigned(2234, 13);
            when 1 | 2 | 10 => return to_unsigned(2979, 13);
            when others     => return to_unsigned(1489, 13);
        end case;
    end function;
    -- mode class: 0 = 1080, 1 = 720p, 2 = SD
    function f_cls(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(2, 2);
            when 5 | 6 | 14     => return to_unsigned(1, 2);
            when others         => return to_unsigned(0, 2);
        end case;
    end function;
    -- Osc spatial steps (Q16 cycles per pixel / per line)
    function f_kax(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(89, 9);
            when 5 | 6 | 14     => return to_unsigned(67, 9);
            when others         => return to_unsigned(44, 9);
        end case;
    end function;
    function f_kay(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(270, 9);
            when 4          => return to_unsigned(137, 9);
            when 8          => return to_unsigned(228, 9);
            when 12         => return to_unsigned(114, 9);
            when 5 | 6 | 14 => return to_unsigned(91, 9);
            when 1 | 2 | 10 => return to_unsigned(121, 9);
            when others     => return to_unsigned(61, 9);
        end case;
    end function;
    -- triangle of a 16-bit phase -> signed -127..127
    function f_tri(p : unsigned(15 downto 0)) return signed is
        variable v : unsigned(15 downto 0);
    begin
        if p(15) = '1' then v := p - 32768; else v := 32768 - p; end if;
        return to_signed(127, 9) - signed(resize(v(15 downto 7), 9));
    end function;
    -- 16-bit Galois LFSR step (taps 0xB400)
    function f_lstep(s : unsigned(15 downto 0)) return unsigned is
    begin
        if s(0) = '1' then return ('0' & s(15 downto 1)) xor x"B400";
        else return '0' & s(15 downto 1); end if;
    end function;

    -- timing
    signal t_kx   : unsigned(11 downto 0) := to_unsigned(1820, 12);
    signal t_c0   : unsigned(22 downto 0) := to_unsigned(32768, 23);
    signal t_ky   : unsigned(13 downto 0) := to_unsigned(1820, 14);
    signal t_vq   : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal t_ksl  : unsigned(12 downto 0) := to_unsigned(1489, 13);
    signal t_cls  : unsigned(1 downto 0) := (others => '0');
    signal t_i0   : unsigned(5 downto 0) := (others => '0');
    signal t_kay : unsigned(8 downto 0) := (others => '0');
    signal t_b8   : std_logic := '0';   -- 720p / SD: the short chroma box

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_half : unsigned(10 downto 0) := to_unsigned(959, 11);

    -- glides: K1, K2, P12 (Q.3), power up at the defaults; K4..K6 are field-latched raw
    type t_g is array(0 to 2) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');

    -- per-field serial products
    signal q_p : unsigned(4 downto 0) := (others => '0');
    signal q_ld, q_st : std_logic := '0';
    signal q_wr : std_logic_vector(2 to 5) := (others => '0');
    signal m_a  : unsigned(19 downto 0) := (others => '0');
    signal m_b  : unsigned(9 downto 0) := (others => '0');
    signal m_h  : unsigned(19 downto 0) := (others => '0');
    signal m_l  : unsigned(9 downto 0) := (others => '0');

    -- per-field values
    signal f_ks9   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal f_fan9  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal f_spm   : unsigned(8 downto 0) := (others => '0');     -- |K3 - 512| - 8
    signal f_sneg  : std_logic := '0';
    signal f_spd   : unsigned(10 downto 0) := (others => '0');    -- Q16 cycles per field
    signal f_drift : unsigned(15 downto 0) := (others => '0');
    signal f_k4    : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal f_po    : signed(8 downto 0) := (others => '0');            -- pocket offset, Q7
    signal f_m5    : unsigned(4 downto 0) := to_unsigned(12, 5);
    signal f_b9    : unsigned(10 downto 0) := to_unsigned(512, 11);  -- band slope ks * fan, Q9
    signal f_qq    : unsigned(9 downto 0) := (others => '0');
    signal f_al    : signed(18 downto 0) := (others => '0');          -- band offset, Q24
    signal f_fpos  : std_logic := '0';
    signal f_ssh   : signed(10 downto 0) := (others => '0');
    signal f_off5  : signed(7 downto 0) := (others => '0');
    signal f_hc8   : unsigned(6 downto 0) := (others => '0');
    signal f_lph   : unsigned(15 downto 0) := (others => '0');
    signal f_lfo5  : signed(3 downto 0) := (others => '0');
    signal f_sl0   : unsigned(23 downto 0) := (others => '0');
    signal f_sd    : unsigned(15 downto 0) := x"ACE1";
    signal f_ep    : unsigned(2 downto 0) := (others => '0');
    signal f_tb : unsigned(15 downto 0) := (others => '0');
    signal f_flat, f_norip, f_mir, f_ice, f_osc : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- per-line engine: one MAC lane, the three fields in turn; writes cell-boundary values
    -- B[i] = (D[i] + D[i+1]) / 2 of the line's knot row (FR Q7, BG Q8, GB Q7)
    signal e_V    : unsigned(21 downto 0) := (others => '0');     -- Q16 cells
    signal e_vl   : unsigned(18 downto 0) := (others => '0');     -- Q8 virtual lines
    signal e_sl   : unsigned(23 downto 0) := (others => '0');     -- Q16 slots
    signal e_ln   : unsigned(10 downto 0) := (others => '0');
    signal e_seed : unsigned(15 downto 0) := x"ACE1";
    signal e_k5   : unsigned(2 downto 0) := (others => '0');
    signal e_wbk  : std_logic := '0';
    signal e_st   : unsigned(3 downto 0) := (others => '0');
    signal e_oi   : unsigned(5 downto 0) := (others => '0');
    signal e_iss  : std_logic := '0';
    signal e_i    : unsigned(5 downto 0) := (others => '0');
    signal e_r, e_f : unsigned(1 downto 0) := (others => '0');
    signal e_j    : unsigned(5 downto 0) := (others => '0');
    signal e_t8   : unsigned(7 downto 0) := (others => '0');
    signal e_t2   : unsigned(15 downto 0) := (others => '0');
    signal e_tq   : unsigned(7 downto 0) := (others => '0');
    signal e_tm   : unsigned(15 downto 0) := (others => '0');
    signal e_base : unsigned(10 downto 0) := (others => '0');
    signal e_abi  : unsigned(10 downto 0) := (others => '0');
    signal e_w0, e_w1, e_w2 : unsigned(8 downto 0) := (others => '0');
    signal r_ca   : unsigned(10 downto 0) := (others => '0');
    signal cs_q, cs_q2 : std_logic_vector(19 downto 0) := (others => '0');
    type t_u2a is array (natural range <>) of unsigned(1 downto 0);
    type t_u6a is array (natural range <>) of unsigned(5 downto 0);
    signal e_tv : std_logic_vector(0 to 5) := (others => '0');
    signal e_tr, e_tf : t_u2a(0 to 5) := (others => (others => '0'));
    signal e_cv : signed(7 downto 0) := (others => '0');
    signal e_wc : unsigned(8 downto 0) := (others => '0');
    signal e_ph, e_pl : signed(13 downto 0) := (others => '0');
    signal e_p  : signed(17 downto 0) := (others => '0');
    signal e_aa, e_ab, e_ac : signed(17 downto 0) := (others => '0');
    signal e_dv : std_logic := '0';
    signal e_df : unsigned(1 downto 0) := (others => '0');
    signal e_d  : signed(10 downto 0) := (others => '0');
    signal e_da, e_db : signed(10 downto 0) := (others => '0');   -- this knot's FR, BG
    signal e_pa, e_pb, e_pc : signed(10 downto 0) := (others => '0');   -- previous knot's D

    -- per-line scalars: fl = alpha + beta * T(v); ph0 = fl * (247 + u0) (one serial multiplier)
    signal p_st  : unsigned(5 downto 0) := (others => '0');
    signal p_idx : unsigned(8 downto 0) := (others => '0');
    signal p_ids : unsigned(7 downto 0) := (others => '0');
    signal r_fy  : unsigned(8 downto 0) := (others => '0');
    signal r_fa  : unsigned(7 downto 0) := (others => '0');
    signal fy_q, fy_q2 : std_logic_vector(16 downto 0) := (others => '0');
    signal fp_q, fp_q2 : std_logic_vector(25 downto 0) := (others => '0');
    signal p_fl  : unsigned(17 downto 0) := (others => '0');     -- Q24 cycles / virtual px
    signal u_a  : signed(18 downto 0) := (others => '0');
    signal u_b  : unsigned(10 downto 0) := (others => '0');
    signal u_h  : signed(19 downto 0) := (others => '0');
    signal u_l  : unsigned(10 downto 0) := (others => '0');
    -- next-line values, handed to the line registers at hsync
    signal n_dph : unsigned(14 downto 0) := (others => '0');
    signal n_ph0 : unsigned(23 downto 0) := (others => '0');
    signal n_po  : signed(8 downto 0) := (others => '0');
    signal n_gc, n_kg : unsigned(7 downto 0) := (others => '0');
    signal n_f07 : signed(7 downto 0) := (others => '0');
    signal n_sl  : unsigned(12 downto 0) := (others => '0');
    signal n_bank : std_logic := '0';
    signal l_dph : unsigned(14 downto 0) := (others => '0');
    signal l_ph0 : unsigned(23 downto 0) := (others => '0');
    signal l_po  : signed(8 downto 0) := (others => '0');
    signal l_gc, l_kg, l_bm : unsigned(7 downto 0) := (others => '0');
    signal l_f07 : signed(7 downto 0) := (others => '0');
    signal l_off : unsigned(3 downto 0) := (others => '0');
    signal l_sl  : unsigned(12 downto 0) := (others => '0');
    signal l_bank : std_logic := '0';
    signal l_pb  : unsigned(15 downto 0) := (others => '0');

    -- cell-boundary RAM (ping-pong by line): {FR Q7 11 | BG Q8 10 | GB Q7 9}
    type t_kr is array (0 to 127) of std_logic_vector(29 downto 0);
    signal kr_ram : t_kr := (others => (others => '0'));
    signal t_we : std_logic := '0';
    signal t_wa : unsigned(6 downto 0) := (others => '0');
    signal t_wd : std_logic_vector(29 downto 0) := (others => '0');
    signal kr_ra : unsigned(6 downto 0) := (others => '0');
    signal kr_q, kr_q2 : std_logic_vector(29 downto 0) := (others => '0');
    attribute no_rw_check : boolean;
    attribute no_rw_check of kr_ram : signal is true;
    signal s_pl : unsigned(2 downto 0) := (others => '0');

    -- ROM ports
    signal r_wa : unsigned(7 downto 0) := (others => '0');
    signal wv_q, wv_q2 : std_logic_vector(7 downto 0) := (others => '0');
    signal r_pa : unsigned(9 downto 0) := (others => '0');
    signal pk_q, pk_q2 : std_logic_vector(7 downto 0) := (others => '0');

    -- beam
    signal b_c  : unsigned(22 downto 0) := (others => '0');       -- Q16 cells
    signal b_ph : unsigned(23 downto 0) := (others => '0');       -- Q19 band phase

    -- video tap
    type t_u8v is array (natural range <>) of unsigned(7 downto 0);
    signal s_vy : t_u8v(1 to 2) := (others => to_unsigned(128, 8));
    signal s_vs : signed(9 downto 0) := (others => '0');

    -- window (stage 2): cell-boundary values B[ci], B[ci + 1]
    signal w0, w1 : std_logic_vector(29 downto 0) := (others => '0');

    -- pixel path (stage N = N registers after the beam)
    signal q1_ci : unsigned(6 downto 0) := (others => '0');
    signal q1_x  : std_logic := '0';
    signal q1_sf, q2_sf : unsigned(7 downto 0) := (others => '0');
    signal q1_ph : unsigned(23 downto 0) := (others => '0');
    signal q1_osc, q2_src, q3_src : signed(9 downto 0) := (others => '0');
    signal q2_phj : unsigned(7 downto 0) := (others => '0');
    signal q2_n5 : unsigned(4 downto 0) := (others => '0');
    signal q2_fq : unsigned(7 downto 0) := (others => '0');
    signal q3_a0, q3_a1 : signed(10 downto 0) := (others => '0');   -- FR B0, B1 - B0 (Q7)
    signal q3_b0, q3_b1 : signed(9 downto 0) := (others => '0');    -- BG (Q8)
    signal q3_c0, q3_c1 : signed(8 downto 0) := (others => '0');    -- GB (Q7)
    signal q3_s5 : unsigned(4 downto 0) := (others => '0');
    signal q3_s4 : unsigned(3 downto 0) := (others => '0');
    signal q3_n5 : unsigned(4 downto 0) := (others => '0');
    signal q3_ro, q3_d : unsigned(7 downto 0) := (others => '0');
    signal q4_ap : signed(16 downto 0) := (others => '0');
    signal q4_bp : signed(14 downto 0) := (others => '0');
    signal q4_cp : signed(13 downto 0) := (others => '0');
    signal q4_a0 : signed(10 downto 0) := (others => '0');
    signal q4_b0 : signed(9 downto 0) := (others => '0');
    signal q4_c0 : signed(8 downto 0) := (others => '0');
    signal q4_fy, q4_hx : unsigned(7 downto 0) := (others => '0');
    signal q4_m  : signed(15 downto 0) := (others => '0');
    signal q5_fr : signed(10 downto 0) := (others => '0');
    signal q5_bg : signed(9 downto 0) := (others => '0');
    signal q5_gb : signed(8 downto 0) := (others => '0');
    signal q5_hx, q5_vy, q6_sh : unsigned(7 downto 0) := (others => '0');
    signal q5_mt : signed(9 downto 0) := (others => '0');
    signal q6_s, q7_s, q8_s, q9_s : signed(11 downto 0) := (others => '0');
    signal q7_a : signed(7 downto 0) := (others => '0');
    signal q6_frt : signed(9 downto 0) := (others => '0');
    signal q6_gb, q7_gb : signed(8 downto 0) := (others => '0');
    signal q7_sh : unsigned(7 downto 0) := (others => '0');
    signal q7_kb : unsigned(3 downto 0) := (others => '0');
    signal q7_h : unsigned(6 downto 0) := (others => '0');
    signal q8_hcp : unsigned(13 downto 0) := (others => '0');
    signal q8_gb : signed(8 downto 0) := (others => '0');
    signal q9_gk : unsigned(7 downto 0) := (others => '0');
    type t_u5a is array (natural range <>) of unsigned(4 downto 0);
    type t_u4a is array (natural range <>) of unsigned(3 downto 0);
    signal s_kbd : t_u4a(1 to 7) := (others => (others => '0'));
    signal q8_ta : signed(8 downto 0) := (others => '0');
    signal q9_pk : signed(12 downto 0) := (others => '0');
    signal q10_f, q11_f, q12_f, q13_f : signed(12 downto 0) := (others => '0');
    signal q11_tr : unsigned(7 downto 0) := (others => '0');
    signal q11_g, q11_c : std_logic := '0';
    signal q12_rp : signed(8 downto 0) := (others => '0');
    signal q14_xr : signed(13 downto 0) := (others => '0');
    signal q14_bp : signed(13 downto 0) := (others => '0');
    signal q14_dg : unsigned(6 downto 0) := (others => '0');
    signal q15_r, q15_b, q16_r, q16_b, q16_g : unsigned(7 downto 0) := (others => '0');
    signal q15_gp : unsigned(14 downto 0) := (others => '0');
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    signal s_gk : t_u8a(10 to 16) := (others => (others => '0'));
    signal s_kr : t_u4a(9 to 16) := (others => (others => '0'));
    signal s_kbl : t_u4a(8 to 16) := (others => (others => '0'));
    signal q17_rm, q17_bm : unsigned(11 downto 0) := (others => '0');
    signal q17_gm : signed(12 downto 0) := (others => '0');
    signal q17_r, q17_g, q17_b : unsigned(7 downto 0) := (others => '0');
    signal q18_r, q18_g, q18_b : unsigned(7 downto 0) := (others => '0');
    signal q19_yrg, q19_ugb, q19_vrg : unsigned(16 downto 0) := (others => '0');
    signal q19_yb, q19_ur, q19_vb : unsigned(15 downto 0) := (others => '0');
    signal q20_y : unsigned(16 downto 0) := (others => '0');
    signal q20_u, q20_v : signed(17 downto 0) := (others => '0');
    signal q21_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q21_u, q21_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
    type t_u10a is array (natural range <>) of unsigned(9 downto 0);
    type t_ring is array (0 to 255) of std_logic_vector(19 downto 0);
    signal rb_ram : t_ring := (others => "1000000000" & "1000000000");
    attribute no_rw_check of rb_ram : signal is true;
    signal rb_ra : unsigned(7 downto 0) := (others => '0');
    signal rb_wd : std_logic_vector(19 downto 0) := "1000000000" & "1000000000";
    signal rb_q, rb_q2 : std_logic_vector(19 downto 0) := "1000000000" & "1000000000";
    signal s_u8, s_v8, s_u8p, s_v8p : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_y1, s_y2 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_yf : t_u10a(23 to 27) := (others => to_unsigned(64, 10));
    signal q28_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q28_u, q28_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- sync ring
    type t_sring is array (0 to 255) of std_logic_vector(2 downto 0);
    signal rs_ram : t_sring := (others => "111");
    attribute no_rw_check of rs_ram : signal is true;
    signal s_wp, rs_ra : unsigned(7 downto 0) := (others => '0');
    signal rs_q, rs_q2 : std_logic_vector(2 downto 0) := "111";

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs, the knot RAM, the sync ring
    ---------------------------------------------------------------------------
    p_crom : process(clk)
    begin
        if rising_edge(clk) then
            cs_q <= C_CS(to_integer(r_ca));
            fp_q <= C_FP(to_integer(r_fa));
            fy_q <= C_FY(to_integer(r_fy));
        end if;
    end process p_crom;

    p_prom : process(clk)
    begin
        if rising_edge(clk) then
            wv_q <= C_WV(to_integer(r_wa));
            pk_q <= C_PK(to_integer(r_pa));
        end if;
    end process p_prom;

    p_kr : process(clk)
    begin
        if rising_edge(clk) then
            if t_we = '1' then kr_ram(to_integer(t_wa)) <= t_wd; end if;
            kr_q <= kr_ram(to_integer(kr_ra));
        end if;
    end process p_kr;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rb_ram(to_integer(s_wp)) <= rb_wd;
            rb_q <= rb_ram(to_integer(rb_ra));
            rb_q2 <= rb_q;
        end if;
    end process p_ring;

    p_sring : process(clk)
    begin
        if rising_edge(clk) then
            rs_ram(to_integer(s_wp)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            rs_q <= rs_ram(to_integer(rs_ra));
            rs_q2 <= rs_q;
        end if;
    end process p_sring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, per-field values
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_x  : signed(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_p  : integer range 0 to 31;
        variable v_ph : integer range 0 to 31;
        variable v_s  : unsigned(20 downto 0);
        variable v_pr : unsigned(29 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_t  : unsigned(15 downto 0);
        variable v_tr : signed(10 downto 0);
        variable v_q  : unsigned(22 downto 0);
    begin
        if rising_edge(clk) then
            t_kx  <= f_kx(registers_in(8)(3 downto 0));
            t_c0  <= f_c0(registers_in(8)(3 downto 0));
            t_ky  <= f_ky(registers_in(8)(3 downto 0));
            t_vq  <= f_vq(registers_in(8)(3 downto 0));
            t_ksl <= f_ksl(registers_in(8)(3 downto 0));
            t_cls <= f_cls(registers_in(8)(3 downto 0));
            t_kay <= f_kay(registers_in(8)(3 downto 0));
            if t_cls = 0 then t_b8 <= '0'; else t_b8 <= '1'; end if;
            t_i0 <= t_c0(21 downto 16);

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
            elsif s_zc /= 1023 then
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
            g_wm1 <= s_w - 1;
            s_half <= s_w(11 downto 1) - 1;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                -- K3 Flow: bipolar, deadband 8 (raw: the speed shows no ADC noise)
                v_x := signed(resize(unsigned(registers_in(2)), 11)) - 512;
                if v_x > 8 then f_sneg <= '0'; v_u := unsigned(v_x) - 8; f_spm <= v_u(8 downto 0);
                elsif v_x < -8 then f_sneg <= '1'; v_u := unsigned(-v_x) - 8; f_spm <= v_u(8 downto 0);
                else f_sneg <= '0'; f_spm <= (others => '0'); end if;
                f_k4  <= unsigned(registers_in(3));
                f_m5  <= unsigned(registers_in(4)(9 downto 5));                   -- K5 Melt
                f_ssh <= signed(resize(unsigned(registers_in(5)), 11)) - 512;    -- K6 Sky
                f_flat  <= registers_in(6)(0);
                f_norip <= registers_in(6)(1);
                f_mir   <= registers_in(6)(2);
                f_ice   <= registers_in(6)(3);
                f_osc   <= registers_in(6)(4);
                -- + flow streams the stripes right (phase down)
                if f_sneg = '1' then f_drift <= f_drift + resize(f_spd, 16);
                else f_drift <= f_drift - resize(f_spd, 16); end if;
                f_lph <= f_lph + 121;
                f_sl0 <= f_sl0 + 447;
                f_ep <= f_ep + 1;
                if f_ep = 7 then f_sd <= f_lstep(f_lstep(f_lstep(f_sd))); end if;
                f_tb <= f_tb - 113;
            end if;

            -- glides: ease 1/8 of the gap per field (deadband 4); one channel per four zone clocks (4..27)
            if s_zone = '1' and s_zc >= 4 and s_zc < 16 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when others => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(2);
                        end case;
                    when 1 =>
                        gl_d <= signed(resize(gl_k, 14)) - signed(resize(gl_o, 14));
                    when 2 =>
                        v_ng := signed(resize(gl_o, 14)) + shift_right(gl_d, 3);
                        gl_n <= unsigned(v_ng(12 downto 0));
                        if gl_d > 31 or gl_d < -31 then gl_w <= '1'; else gl_w <= '0'; end if;
                    when others =>
                        if gl_w = '1' then
                            case to_integer(q_gi) is
                                when 0 => s_g(0) <= gl_n;
                                when 1 => s_g(1) <= gl_n;
                                when others => s_g(2) <= gl_n;
                            end case;
                        end if;
                end case;
            end if;

            -- knob curves (settle long before the sequencers read them)
            v_k := s_g(0)(12 downto 3);                                   -- K1 Stripes
            if v_k(9) = '0' then f_ks9 <= to_unsigned(256, 10) + resize(v_k(9 downto 1), 10);
            else f_ks9 <= v_k; end if;
            f_fan9 <= s_g(1)(12 downto 3);                                -- K2 Fan
            f_fpos <= s_g(1)(12);
            v_k := s_g(2)(12 downto 3);                                   -- P12 Lava
            -- pocket offset: (K4 - 512) / 4 + (P12 - 512) / 8 (Q7)
            f_po <= resize(signed(resize(f_k4(9 downto 2), 10)) - 128, 9) + resize(signed(resize(v_k(9 downto 3), 9)) - 64, 9);
            if v_k(9) = '0' then
                -- -((512 - p) * 7) >> 5
                v_u := to_unsigned(512, 11) - resize(v_k, 11);
                f_off5 <= -signed(resize(shift_right(shift_left(resize(v_u, 14), 3) - resize(v_u, 14), 5), 8));
                f_hc8 <= (others => '0');
            else
                -- ((p - 512) * 9) >> 6
                v_u := resize(v_k, 11) - 512;
                f_off5 <= signed(resize(shift_right(shift_left(resize(v_u, 14), 3) + resize(v_u, 14), 6), 8));
                f_hc8 <= v_u(8 downto 2);
            end if;
            -- breathe LFO: triangle of the phase, +-4 Q5 key units
            if f_lph(15) = '1' then v_t := f_lph - 32768; else v_t := 32768 - f_lph; end if;
            v_tr := signed(resize(v_t(15 downto 6), 11)) - 256;
            f_lfo5 <= resize(shift_right(v_tr, 6), 4);

            -- serial products: slot p = zc/32 (2..5), phase 0 load, 1..10 step, 12 store
            --   2 spd = sp^2 >> 9   3 beta = (ks fan) >> 9
            --   4 qq = (ks |512 - fan|) >> 9   5 alpha = +-(f0 qq) >> 9
            v_p  := to_integer(s_zc(9 downto 5));
            v_ph := to_integer(s_zc(4 downto 0));
            q_p <= s_zc(9 downto 5);
            if s_zone = '1' and v_p >= 2 and v_p <= 5 and v_ph = 0 then q_ld <= '1'; else q_ld <= '0'; end if;
            if s_zone = '1' and v_p >= 2 and v_p <= 5 and v_ph >= 1 and v_ph <= 10 then q_st <= '1'; else q_st <= '0'; end if;
            for k in 2 to 5 loop
                if s_zone = '1' and v_p = k and v_ph = 12 then q_wr(k) <= '1'; else q_wr(k) <= '0'; end if;
            end loop;
            if q_ld = '1' then
                m_h <= (others => '0'); m_l <= (others => '0');
                case to_integer(q_p) is
                    when 2 => m_a <= resize(f_spm, 20); m_b <= resize(f_spm, 10);
                    when 3 => m_a <= resize(f_ks9, 20); m_b <= f_fan9;
                    when 4 =>
                        m_a <= resize(f_ks9, 20);
                        if f_fpos = '1' then m_b <= f_fan9 - 512; else m_b <= 512 - f_fan9; end if;
                    when others => m_a <= to_unsigned(96483, 20); m_b <= f_qq;
                end case;
            end if;
            if q_st = '1' then
                if m_b(0) = '1' then v_s := resize(m_h, 21) + resize(m_a, 21); else v_s := resize(m_h, 21); end if;
                m_h <= v_s(20 downto 1);
                m_l <= v_s(0) & m_l(9 downto 1);
                m_b <= '0' & m_b(9 downto 1);
            end if;
            v_pr := m_h & m_l;
            if q_wr(2) = '1' then f_spd <= resize(v_pr(17 downto 9), 11); end if;      -- sp^2 >> 9
            if q_wr(3) = '1' then f_b9 <= v_pr(19 downto 9); end if;
            if q_wr(4) = '1' then f_qq <= v_pr(18 downto 9); end if;
            if q_wr(5) = '1' then
                if f_fpos = '1' then f_al <= -signed(resize(v_pr(27 downto 9), 19));
                else f_al <= signed(resize(v_pr(27 downto 9), 19)); end if;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 3/4 clocks after the source's avid
    -- rise, field-latched width)
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
    -- Per-line engine: at the source avid rise of line n, blend control rows
    -- j..j+2 of line n+1 into the 56 knots D[i] of FR, BG and GB (one MAC
    -- lane, the three fields in turn: 504 clocks) and write the 55 cell-
    -- boundary values B[i] = (D[i] + D[i+1]) / 2 into the bank line n+1 reads.
    -- Line 0 is computed in the vblank zone.
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_jr : unsigned(5 downto 0);
        variable v_w  : unsigned(9 downto 0);
        variable v_s  : signed(18 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_d12 : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- kick
            if s_zone = '1' and s_zc = 200 then
                e_V <= to_unsigned(32768, 22); e_vl <= (others => '0'); e_sl <= f_sl0;
                e_ln <= (others => '0'); e_seed <= f_sd; e_k5 <= (others => '0');
                e_wbk <= not l_bank; e_st <= to_unsigned(1, 4);
            elsif data_in.avid = '1' and s_avq = '0' then
                e_V <= e_V + resize(t_ky, 22); e_vl <= e_vl + resize(t_vq, 19); e_sl <= e_sl + resize(t_ksl, 24);
                e_ln <= e_ln + 1;
                if e_ln(0) = '1' then e_k5 <= to_unsigned(5, 3); else e_k5 <= (others => '0'); end if;
                e_wbk <= not l_bank; e_st <= to_unsigned(1, 4);
            else
                if e_st /= 0 then
                    if e_st = 12 then e_st <= (others => '0'); else e_st <= e_st + 1; end if;
                end if;
                -- line seed: 5 LFSR steps on every even line
                if e_k5 /= 0 then e_seed <= f_lstep(e_seed); e_k5 <= e_k5 - 1; end if;
            end if;

            -- setup: row, fraction, weights (w0 + w1 + w2 = 256)
            case to_integer(e_st) is
                when 1 =>
                    v_jr := e_V(21 downto 16);
                    if v_jr > 30 then e_j <= to_unsigned(30, 6); e_t8 <= (others => '1');
                    else e_j <= v_jr; e_t8 <= e_V(15 downto 8); end if;
                when 2 =>
                    e_t2 <= to_unsigned(256, 16); e_tq <= e_t8; e_tm <= resize(e_t8, 16);
                    e_base <= shift_left(resize(e_j, 11), 5) + shift_left(resize(e_j, 11), 4)
                              + shift_left(resize(e_j, 11), 3);                       -- j * 56
                when 3 to 10 =>
                    -- t8^2 + 256, one multiplier bit per clock
                    if e_tq(0) = '1' then e_t2 <= e_t2 + e_tm; end if;
                    e_tq <= '0' & e_tq(7 downto 1); e_tm <= shift_left(e_tm, 1);
                when 11 =>
                    e_w2 <= resize(e_t2(15 downto 9), 9);
                    e_w0 <= to_unsigned(128, 9) - resize(e_t8, 9) + resize(e_t2(15 downto 9), 9);
                when 12 =>
                    e_w1 <= to_unsigned(256, 9) - e_w0 - e_w2;
                    e_iss <= '1'; e_i <= (others => '0'); e_r <= (others => '0'); e_f <= (others => '0');
                    e_abi <= e_base; e_oi <= (others => '0');
                when others => null;
            end case;

            -- issue: 56 knots x 3 rows x 3 fields; one ROM read per (knot, row)
            e_tv(0) <= e_iss; e_tr(0) <= e_r; e_tf(0) <= e_f;
            if e_iss = '1' then
                if e_f = 0 then
                    case to_integer(e_r) is
                        when 0 => r_ca <= e_abi;
                        when 1 => r_ca <= e_abi + 56;
                        when others => r_ca <= e_abi + 112;
                    end case;
                end if;
                if e_f = 2 then
                    e_f <= (others => '0');
                    if e_r = 2 then
                        e_r <= (others => '0');
                        e_abi <= e_abi + 1;
                        if e_i = 55 then e_iss <= '0'; else e_i <= e_i + 1; end if;
                    else
                        e_r <= e_r + 1;
                    end if;
                else
                    e_f <= e_f + 1;
                end if;
            end if;
            -- tags: 0 = address, 1 = ROM out, 2 = fabric copy, 3 = field / weight select,
            --       4 = products, 5 = product sum, then accumulate
            for k in 1 to 5 loop
                e_tv(k) <= e_tv(k - 1); e_tr(k) <= e_tr(k - 1); e_tf(k) <= e_tf(k - 1);
            end loop;
            if e_tf(1) = 0 then cs_q2 <= cs_q; end if;
            case to_integer(e_tf(2)) is
                when 0 => e_cv <= signed(cs_q2(19 downto 12));
                when 1 => e_cv <= resize(signed(cs_q2(11 downto 5)), 8);
                when others => e_cv <= signed(resize(unsigned(cs_q2(4 downto 0)), 8));
            end case;
            case to_integer(e_tr(2)) is
                when 0 => e_wc <= e_w0;
                when 1 => e_wc <= e_w1;
                when others => e_wc <= e_w2;
            end case;
            e_ph <= e_cv * signed('0' & e_wc(8 downto 4));
            e_pl <= resize(e_cv * signed('0' & e_wc(3 downto 0)), 14);
            e_p <= shift_left(resize(e_ph, 18), 4) + resize(e_pl, 18);
            -- accumulate over the 3 rows per field; D = (sum + 32) >> 6
            e_dv <= '0';
            if e_tv(5) = '1' then
                if e_tr(5) = 0 then
                    case to_integer(e_tf(5)) is
                        when 0 => e_aa <= e_p;
                        when 1 => e_ab <= e_p;
                        when others => e_ac <= e_p;
                    end case;
                else
                    case to_integer(e_tf(5)) is
                        when 0 => v_s := resize(e_aa, 19) + resize(e_p, 19);
                        when 1 => v_s := resize(e_ab, 19) + resize(e_p, 19);
                        when others => v_s := resize(e_ac, 19) + resize(e_p, 19);
                    end case;
                    case to_integer(e_tf(5)) is
                        when 0 => e_aa <= v_s(17 downto 0);
                        when 1 => e_ab <= v_s(17 downto 0);
                        when others => e_ac <= v_s(17 downto 0);
                    end case;
                    if e_tr(5) = 2 then
                        e_dv <= '1'; e_df <= e_tf(5);
                        e_d <= resize(shift_right(v_s + 32, 6), 11);
                    end if;
                end if;
            end if;
            -- collect D per knot; write B[i-1] = (D[i-1] + D[i]) / 2 when the knot's GB is done
            t_we <= '0';
            if e_dv = '1' then
                case to_integer(e_df) is
                    when 0 => e_da <= e_d;
                    when 1 => e_db <= e_d;
                    when others =>
                        e_pa <= e_da; e_pb <= e_db; e_pc <= e_d;
                        e_oi <= e_oi + 1;
                        if e_oi /= 0 then
                            t_we <= '1';
                            t_wa <= e_wbk & (e_oi - 1);
                            v_d12 := shift_right(resize(e_pa, 12) + resize(e_da, 12), 1);
                            t_wd(29 downto 19) <= std_logic_vector(v_d12(10 downto 0));
                            v_d := shift_right(e_pb + e_db, 1);
                            t_wd(18 downto 9) <= std_logic_vector(v_d(9 downto 0));
                            v_d := shift_right(e_pc + e_d, 1);
                            t_wd(8 downto 0) <= std_logic_vector(v_d(8 downto 0));
                        end if;
                end case;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- Per-line scalars (one serial multiplier, 11 steps per product):
    --   fl  = alpha + (beta * T[v/2]) >> 9          (band frequency: Stripes x Fan, Q24)
    --   ph0 = (fl * (247 + u0)) >> 5 + drift + jitter  (phase at the left edge, Q19)
    --   dph = (fl * su / 2) >> 5                       (per pixel, Q19)
    --   colour row {Gc, F0, kG, pocket gate} at the Sky-shifted line
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_i   : unsigned(10 downto 0);
        variable v_ls  : signed(12 downto 0);
        variable v_sh  : signed(12 downto 0);
        variable v_s   : signed(19 downto 0);
        variable v_r   : signed(30 downto 0);
        variable v_cat : std_logic_vector(30 downto 0);
        variable v_fl  : signed(19 downto 0);
        variable v_d   : unsigned(18 downto 0);
        variable v_p0  : unsigned(25 downto 0);
    begin
        if rising_edge(clk) then
            fp_q2 <= fp_q; fy_q2 <= fy_q;
            if e_st = 1 then
                p_st <= to_unsigned(1, 6);
            elsif p_st /= 0 then
                if p_st = 40 then p_st <= (others => '0'); else p_st <= p_st + 1; end if;
            end if;
            -- serial multiplier: steps while p_st in 7..17 and 22..32
            if (p_st >= 7 and p_st <= 17) or (p_st >= 22 and p_st <= 32) then
                if u_b(0) = '1' then v_s := u_h + resize(u_a, 20); else v_s := u_h; end if;
                u_h <= shift_right(v_s, 1);
                u_l <= v_s(0) & u_l(10 downto 1);
                u_b <= '0' & u_b(10 downto 1);
            end if;
            v_cat := std_logic_vector(u_h(19 downto 0)) & std_logic_vector(u_l);
            v_r := signed(v_cat);
            case to_integer(p_st) is
                when 1 =>
                    v_i := e_vl(18 downto 8);                         -- virtual line
                    if v_i(10 downto 1) > 511 then p_idx <= to_unsigned(511, 9); else p_idx <= v_i(9 downto 1); end if;
                    v_ls := signed(resize(v_i, 13)) - resize(f_ssh, 13);
                    v_sh := shift_right(v_ls, 3);
                    if v_sh < 0 then p_ids <= (others => '0');
                    elsif v_sh > 135 then p_ids <= to_unsigned(135, 8);
                    else p_ids <= unsigned(v_sh(7 downto 0)); end if;
                when 2 => r_fy <= p_idx; r_fa <= p_ids;
                when 5 =>
                    -- beta * T
                    u_a <= signed(resize(unsigned(fy_q2), 19)); u_b <= f_b9; u_h <= (others => '0'); u_l <= (others => '0');
                    n_gc <= unsigned(fp_q2(25 downto 18)); n_f07 <= signed(fp_q2(17 downto 10)); n_kg <= unsigned(fp_q2(9 downto 2));
                    case fp_q2(1 downto 0) is
                        when "01" => n_po <= f_po - 16;
                        when "10" => n_po <= f_po - 32;
                        when others => n_po <= f_po;
                    end case;
                when 19 =>
                    v_fl := resize(shift_right(v_r, 9), 20) + resize(f_al, 20);
                    p_fl <= unsigned(v_fl(17 downto 0));
                when 20 =>
                    -- fl * (247 + u0)
                    u_a <= signed(resize(p_fl, 19)); u_h <= (others => '0'); u_l <= (others => '0');
                    if t_cls = 2 then u_b <= to_unsigned(487, 11); else u_b <= to_unsigned(247, 11); end if;
                    -- dph = (fl * su / 2) >> 5
                    case to_integer(t_cls) is
                        when 0 => v_d := resize(p_fl, 19);
                        when 1 => v_d := resize(p_fl, 19) + resize(p_fl(17 downto 1), 19);
                        when others => v_d := shift_left(resize(p_fl, 19), 1);
                    end case;
                    n_dph <= resize(v_d(18 downto 5), 15);
                when 34 =>
                    v_p0 := unsigned(std_logic_vector(v_r(30 downto 5)));
                    n_ph0 <= v_p0(23 downto 0) + shift_left(resize(f_drift, 24), 3)
                             + unsigned(resize(shift_left(resize(signed(not e_seed(7) & e_seed(6 downto 0)), 17), 8), 24));
                    n_sl <= e_sl(20 downto 8); n_bank <= e_wbk;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Line hand-over at hsync; window preload; beam
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_hsr = '1' then
                l_dph <= n_dph; l_ph0 <= n_ph0; l_po <= n_po; l_gc <= n_gc; l_kg <= n_kg;
                l_bm <= to_unsigned(255, 8) - resize(n_gc(7 downto 4), 8) - resize(n_gc(7 downto 5), 8);
                l_f07 <= n_f07; l_off <= n_gc(7 downto 4); l_sl <= n_sl; l_bank <= n_bank;
                s_pl <= to_unsigned(1, 3);
            elsif s_pl /= 0 then
                s_pl <= s_pl + 1;
            end if;
            -- RAM read address: preload B[i0], B[i0 + 1], then prefetch B[ci + 2]
            case to_integer(s_pl) is
                when 1 => kr_ra <= l_bank & t_i0;
                when 2 => kr_ra <= l_bank & (t_i0 + 1);
                when others => kr_ra <= l_bank & (q1_ci(5 downto 0) + 2);
            end case;
            kr_q2 <= kr_q;

            -- Osc phases
            if s_zgo = '1' then
                l_pb <= f_tb;
            elsif data_in.avid = '1' and s_avq = '0' then
                l_pb <= l_pb + resize(t_kay, 16);
            end if;
            if g_start = '1' then
                b_c  <= t_c0;
                b_ph <= l_ph0;
            elsif g_avid = '1' then
                b_c <= b_c + resize(t_kx, 23);
                if f_mir = '1' and g_cnt(10 downto 0) >= s_half then
                    if g_cnt(10 downto 0) /= s_half then b_ph <= b_ph - resize(l_dph, 24); end if;
                else
                    b_ph <= b_ph + resize(l_dph, 24);
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ci  : unsigned(6 downto 0);
        variable v_q   : unsigned(12 downto 0);
        variable v_a0, v_a1 : signed(11 downto 0);
        variable v_b0, v_b1 : signed(10 downto 0);
        variable v_c0, v_c1 : signed(9 downto 0);
        variable v_dd  : signed(8 downto 0);
        variable v_hx  : signed(11 downto 0);
        variable v_ys  : unsigned(12 downto 0);
        variable v_pa  : signed(9 downto 0);
        variable v_fr  : signed(11 downto 0);
        variable v_kb  : signed(13 downto 0);
        variable v_h   : signed(10 downto 0);
        variable v_gk  : signed(11 downto 0);
        variable v_ta  : signed(16 downto 0);
        variable v_pk  : signed(16 downto 0);
        variable v_f   : signed(13 downto 0);
        variable v_r7  : unsigned(6 downto 0);
        variable v_tr  : signed(8 downto 0);
        variable v_rr  : signed(15 downto 0);
        variable v_x   : signed(13 downto 0);
        variable v_dg  : signed(13 downto 0);
        variable v_gp  : signed(18 downto 0);
        variable v_m   : signed(14 downto 0);
        variable v_r, v_gg, v_b : unsigned(15 downto 0);
        variable v_y   : unsigned(16 downto 0);
        variable v_cs  : signed(17 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_dl  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- video tap: 2-px luma average -> signed (Y - 512) / 2 (a few px early: invisible in a bend)
            s_vy(1) <= unsigned(data_in.y(9 downto 2)); s_vy(2) <= s_vy(1);
            s_vs <= signed(resize(resize(s_vy(1), 9) + resize(s_vy(2), 9), 10)) - 256;

            -- 1: cell index / fraction; cell crossing; jitter product; Osc
            v_ci := b_c(22 downto 16);
            if g_avid = '1' then
                if v_ci /= q1_ci then q1_x <= '1'; else q1_x <= '0'; end if;
                q1_ci <= v_ci;
            else
                q1_x <= '0';
                q1_ci <= '0' & t_i0;
            end if;
            q1_sf <= b_c(15 downto 8);
            q1_ph <= b_ph;
            q1_osc <= resize(f_tri(b_c(18 downto 3)), 10) + resize(f_tri(l_pb), 10);

            -- 2: window (preload / shift); band phase; pocket band / phase; source select
            if s_pl = 4 then w0 <= kr_q2;
            elsif s_pl = 5 then w1 <= kr_q2;
            elsif q1_x = '1' then
                w0 <= w1; w1 <= kr_q2;
            end if;
            q2_sf <= q1_sf;
            q2_phj <= q1_ph(18 downto 11);
            v_q := q1_ph(23 downto 11) - 141;
            q2_n5 <= v_q(12 downto 8); q2_fq <= v_q(7 downto 0);
            if f_osc = '1' then q2_src <= q1_osc; else q2_src <= s_vs; end if;

            -- 3: linear cell terms (B0, B1 - B0); wave ROM address; pocket slot offset and distance
            v_a0 := resize(signed(w0(29 downto 19)), 12); v_a1 := resize(signed(w1(29 downto 19)), 12);
            q3_a0 <= signed(w0(29 downto 19));
            q3_a1 <= resize(v_a1 - v_a0, 11);
            v_b0 := resize(signed(w0(18 downto 9)), 11); v_b1 := resize(signed(w1(18 downto 9)), 11);
            q3_b0 <= signed(w0(18 downto 9));
            q3_b1 <= resize(v_b1 - v_b0, 10);
            v_c0 := resize(signed(w0(8 downto 0)), 10); v_c1 := resize(signed(w1(8 downto 0)), 10);
            q3_c0 <= signed(w0(8 downto 0));
            q3_c1 <= resize(v_c1 - v_c0, 9);
            q3_s5 <= q2_sf(7 downto 3);
            q3_s4 <= q2_sf(7 downto 4);
            r_wa <= q2_phj;
            q3_ro <= unsigned(C_RO(to_integer(q2_n5)));
            q3_n5 <= q2_n5;
            v_dd := signed('0' & q2_fq) - 141;
            if v_dd < 0 then q3_d <= unsigned(resize(-v_dd, 8)); else q3_d <= unsigned(v_dd(7 downto 0)); end if;
            q3_src <= q2_src;

            -- 4: cell products; melt product; pocket slot -> ROM address; hx
            q4_ap <= resize(q3_a1 * signed('0' & q3_s5), 17);
            q4_bp <= resize(q3_b1 * signed('0' & q3_s4), 15);
            q4_cp <= resize(q3_c1 * signed('0' & q3_s4), 14);
            q4_a0 <= q3_a0; q4_b0 <= q3_b0; q4_c0 <= q3_c0;
            q4_m <= q3_src * signed('0' & f_m5);
            v_ys := l_sl + resize(q3_ro, 13);
            q4_fy <= v_ys(7 downto 0);
            r_pa <= q3_n5 & v_ys(12 downto 8);
            v_hx := shift_left(to_signed(72, 12) - signed(resize(q3_d, 12)), 3) - (to_signed(72, 12) - signed(resize(q3_d, 12)));
            if v_hx < 0 then q4_hx <= (others => '0'); elsif v_hx > 255 then q4_hx <= (others => '1');
            else q4_hx <= unsigned(v_hx(7 downto 0)); end if;

            -- 5: FR, BG, GB; vy; wave (fabric copy); melt
            q5_fr <= q4_a0 + resize(shift_right(q4_ap, 5), 11);
            q5_bg <= q4_b0 + resize(shift_right(q4_bp, 4), 10);
            q5_gb <= q4_c0 + resize(shift_right(q4_cp, 4), 9);
            if q4_fy(7) = '0' then q5_vy <= q4_fy(6 downto 0) & '0';
            else q5_vy <= (not q4_fy(6 downto 0)) & '0'; end if;
            q5_hx <= q4_hx;
            wv_q2 <= wv_q;
            q5_mt <= resize(shift_right(q4_m, 6), 10);

            -- 6: pocket ROM (fabric copy); shape = min(hx, vy); lava key field; band partial sum
            pk_q2 <= pk_q;
            if q5_hx < q5_vy then q6_sh <= q5_hx; else q6_sh <= q5_vy; end if;
            v_fr := resize(shift_right(q5_fr, 2), 12) + resize(f_off5, 12) + resize(f_lfo5, 12) + resize(shift_right(q5_mt, 3), 12);
            q6_frt <= resize(v_fr, 10);
            q6_s <= signed(resize(unsigned(wv_q2), 12)) + resize(shift_right(q5_bg, 1), 12) + resize(q5_mt, 12);
            q6_gb <= q5_gb;

            -- 7: pocket amplitude + offset (positive entries only); lava key (blanked outside the picture);
            --    hot-core product
            v_pa := resize(signed(pk_q2), 10) + resize(l_po, 10);
            if signed(pk_q2) <= 0 then q7_a <= signed(pk_q2);
            elsif v_pa < 0 then q7_a <= (others => '0');
            elsif v_pa > 127 then q7_a <= to_signed(127, 8);
            else q7_a <= v_pa(7 downto 0); end if;
            q7_sh <= q6_sh;
            v_kb := resize(shift_right(resize(q6_frt, 14) * 13, 4), 14) + 4;
            if s_gav_sr(5) = '0' or v_kb < 0 then q7_kb <= (others => '0');
            elsif v_kb > 8 then q7_kb <= to_unsigned(8, 4);
            else q7_kb <= unsigned(v_kb(3 downto 0)); end if;
            v_h := resize(q6_frt, 11) - 10;
            if v_h < 0 then q7_h <= (others => '0'); elsif v_h > 96 then q7_h <= to_unsigned(96, 7);
            else q7_h <= unsigned(v_h(6 downto 0)); end if;
            q7_s <= q6_s; q7_gb <= q6_gb;

            -- 8: ta = a * shape >> 8; hot-core product
            v_ta := q7_a * signed('0' & q7_sh);
            q8_ta <= resize(shift_right(v_ta, 8), 9);
            q8_hcp <= resize(q7_h * f_hc8, 14);
            q8_s <= q7_s; q8_gb <= q7_gb;

            -- 9: pocket = 2.5 ta; lava green
            q9_pk <= shift_left(resize(q8_ta, 13), 1) + resize(shift_right(q8_ta, 1), 13);
            q9_s <= q8_s;
            if f_flat = '1' then
                q9_gk <= to_unsigned(10, 8);
            else
                v_gk := shift_left(resize(q8_gb, 12), 1) + signed(resize(shift_right(q8_hcp, 6), 12));
                if v_gk < 0 then q9_gk <= (others => '0'); elsif v_gk > 255 then q9_gk <= (others => '1');
                else q9_gk <= unsigned(v_gk(7 downto 0)); end if;
            end if;

            -- 10: the band field F (Q7)
            q10_f <= resize(q9_s, 13) + q9_pk;

            -- 11: ripple phase (F * 12 mod 1) -> triangle; gate F > 20
            v_f := resize(q10_f, 14);
            if f_norip = '1' then v_r7 := unsigned(std_logic_vector(resize(shift_left(v_f, 1) + v_f, 7)));      -- Coarse: F * 3
            else v_r7 := unsigned(std_logic_vector(resize(shift_left(v_f, 3) + shift_left(v_f, 2), 7))); end if; -- Fine: F * 12
            v_tr := signed('0' & v_r7 & '0') - 128;
            if v_tr < 0 then q11_tr <= unsigned(resize(-v_tr, 8)); else q11_tr <= unsigned(v_tr(7 downto 0)); end if;
            if q10_f > 20 then q11_g <= '1'; else q11_g <= '0'; end if;
            q11_c <= f_norip;
            q11_f <= q10_f;

            -- 12: ripple = (tri - 64) / 4 where F > 20 (S8 Coarse: / 2)
            if q11_g = '0' then q12_rp <= (others => '0');
            elsif q11_c = '1' then q12_rp <= shift_right(signed(resize(q11_tr, 9)) - 64, 1);
            else q12_rp <= shift_right(signed(resize(q11_tr, 9)) - 64, 2); end if;
            q12_f <= q11_f;

            -- 13: F + ripple
            q13_f <= q12_f + resize(q12_rp, 13);

            -- 14: colourizer terms
            v_x := resize(q13_f, 14) - signed(resize(l_off, 14)) - 64;
            q14_xr <= v_x;
            q14_bp <= 243 - resize(q13_f, 14);
            v_dg := resize(q13_f, 14) - resize(l_f07, 14);
            if v_dg < 0 then q14_dg <= (others => '0'); elsif v_dg > 127 then q14_dg <= (others => '1');
            else q14_dg <= unsigned(v_dg(6 downto 0)); end if;

            -- 15: R, B clamps; G product
            v_rr := resize(q14_xr, 16) + shift_left(resize(q14_xr, 16), 1) + 121;
            if v_rr < 0 then q15_r <= (others => '0'); elsif v_rr > 242 then q15_r <= to_unsigned(242, 8);
            else q15_r <= unsigned(v_rr(7 downto 0)); end if;
            v_rr := shift_left(resize(q14_bp, 16), 3);
            if v_rr < 0 then q15_b <= (others => '0'); elsif v_rr > signed(resize(l_bm, 16)) then q15_b <= l_bm;
            else q15_b <= unsigned(v_rr(7 downto 0)); end if;
            q15_gp <= q14_dg * l_kg;

            -- 16: G
            v_gp := signed(resize(l_gc, 19)) - signed(resize(shift_right(q15_gp, 7), 19));
            if v_gp < 0 then q16_g <= (others => '0'); elsif v_gp > 255 then q16_g <= (others => '1');
            else q16_g <= unsigned(v_gp(7 downto 0)); end if;
            q16_r <= q15_r; q16_b <= q15_b;

            -- lava key / green pipes: kB from stage 7, kR = kB of the pixel dR to the left
            s_kbd(1) <= q7_kb;
            for k in 2 to 7 loop s_kbd(k) <= s_kbd(k - 1); end loop;
            s_kbl(8) <= q7_kb;
            for k in 9 to 16 loop s_kbl(k) <= s_kbl(k - 1); end loop;
            case to_integer(t_cls) is
                when 0 => s_kr(9) <= s_kbd(7);
                when 1 => s_kr(9) <= s_kbd(5);
                when others => s_kr(9) <= s_kbd(4);
            end case;
            for k in 10 to 16 loop s_kr(k) <= s_kr(k - 1); end loop;
            s_gk(10) <= q9_gk;
            for k in 11 to 16 loop s_gk(k) <= s_gk(k - 1); end loop;

            -- 17: lava mix products
            q17_rm <= s_kr(16) * (255 - q16_r);
            q17_bm <= s_kbl(16) * q16_b;
            v_m := signed(resize(s_gk(16), 15)) - signed(resize(q16_g, 15));
            q17_gm <= resize(v_m * signed('0' & s_kbl(16)), 13);
            q17_r <= q16_r; q17_g <= q16_g; q17_b <= q16_b;

            -- 18: mix sums; S10 Ice palette (R, G, B) <- (G, B, R)
            v_r := resize(q17_r, 16) + resize(shift_right(q17_rm, 3), 16);
            v_b := resize(q17_b, 16) - resize(shift_right(q17_bm, 3), 16);
            v_gg := unsigned(resize(signed(resize(q17_g, 16)) + resize(shift_right(q17_gm, 3), 16), 16));
            if f_ice = '1' then
                q18_r <= v_gg(7 downto 0); q18_g <= v_b(7 downto 0); q18_b <= v_r(7 downto 0);
            else
                q18_r <= v_r(7 downto 0); q18_g <= v_gg(7 downto 0); q18_b <= v_b(7 downto 0);
            end if;

            -- 19: RGB -> YUV terms, Q6 (hardware convention: U = Cr, V = Cb)
            --     Y = 66 R + 129 G + 25 B;  U = 112 R - (96 G + 16 B);  V = 112 B - (40 R + 72 G)
            v_r := resize(q18_r, 16); v_gg := resize(q18_g, 16); v_b := resize(q18_b, 16);
            q19_yrg <= resize(shift_left(v_r, 6), 17) + resize(shift_left(v_r, 1), 17)
                       + resize(shift_left(v_gg, 7), 17) + resize(v_gg, 17);
            q19_yb  <= shift_left(v_b, 4) + shift_left(v_b, 3) + v_b;
            q19_ur  <= shift_left(v_r, 7) - shift_left(v_r, 4);
            q19_ugb <= resize(shift_left(v_gg, 6), 17) + resize(shift_left(v_gg, 5), 17) + resize(shift_left(v_b, 4), 17);
            q19_vb  <= shift_left(v_b, 7) - shift_left(v_b, 4);
            q19_vrg <= resize(shift_left(v_r, 5), 17) + resize(shift_left(v_r, 3), 17)
                       + resize(shift_left(v_gg, 6), 17) + resize(shift_left(v_gg, 3), 17);
            -- 20: sums
            q20_y <= q19_yrg + resize(q19_yb, 17);
            q20_u <= signed(resize(q19_ur, 18)) - signed(resize(q19_ugb, 18));
            q20_v <= signed(resize(q19_vb, 18)) - signed(resize(q19_vrg, 18));
            -- 21: Y, U, V (10 bit)
            v_y := shift_right(q20_y, 6);
            q21_y <= to_unsigned(64, 10) + v_y(9 downto 0);
            v_cs := shift_right(q20_u, 6) + 512;
            if v_cs < 0 then q21_u <= (others => '0'); elsif v_cs > 1023 then q21_u <= (others => '1');
            else q21_u <= unsigned(v_cs(9 downto 0)); end if;
            v_cs := shift_right(q20_v, 6) + 512;
            if v_cs < 0 then q21_v <= (others => '0'); elsif v_cs > 1023 then q21_v <= (others => '1');
            else q21_v <= unsigned(v_cs(9 downto 0)); end if;

            -- 22..28: output conditioning.  Chroma: box-8 pair [1 2 2 2 2 2 2 2 1]/16 in every mode over a
            -- block-RAM ring (the sums see samples 1..8 back), re-seeded neutral in the vblank zone, its
            -- centre at stage 28 -- summed straight into the output register.  Luma [1 2 1] (stage 23),
            -- delayed 4 to stage 27, then scaled toward black (white 940 -> 777) into stage 28.
            if s_zone = '1' then rb_wd <= std_logic_vector(to_unsigned(512, 10)) & std_logic_vector(to_unsigned(512, 10));
            else rb_wd <= std_logic_vector(q21_u) & std_logic_vector(q21_v); end if;
            rb_ra <= s_wp - 5;
            if s_zone = '1' and s_zc = 40 then
                s_u8 <= to_unsigned(4096, 13); s_v8 <= to_unsigned(4096, 13);
            else
                s_u8 <= s_u8 + unsigned(rb_wd(19 downto 10)) - unsigned(rb_q2(19 downto 10));
                s_v8 <= s_v8 + unsigned(rb_wd(9 downto 0)) - unsigned(rb_q2(9 downto 0));
            end if;
            s_u8p <= s_u8; s_v8p <= s_v8;
            s_y1 <= q21_y; s_y2 <= s_y1;
            v_sy := resize(q21_y, 12) + shift_left(resize(s_y1, 12), 1) + resize(s_y2, 12);
            s_yf(23) <= v_sy(11 downto 2);
            for k in 24 to 27 loop s_yf(k) <= s_yf(k - 1); end loop;

            -- 28: blanking gate
            if s_yf(27) > 64 then v_dl := s_yf(27) - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(26) = '1' then
                q28_y <= s_yf(27) - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                v_y := resize(s_u8, 17) + resize(s_u8p, 17); q28_u <= v_y(13 downto 4);
                v_y := resize(s_v8, 17) + resize(s_v8p, 17); q28_v <= v_y(13 downto 4);
            else
                q28_y <= to_unsigned(64, 10); q28_u <= to_unsigned(512, 10); q28_v <= to_unsigned(512, 10);
            end if;

            -- sync ring read pointer
            s_wp <= s_wp + 1;
            rs_ra <= s_wp - (C_LAT - 3);
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(q28_y);
    data_out.u       <= std_logic_vector(q28_u);
    data_out.v       <= std_logic_vector(q28_v);
    data_out.hsync_n <= rs_q2(2);
    data_out.vsync_n <= rs_q2(1);
    data_out.field_n <= rs_q2(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture magma;
