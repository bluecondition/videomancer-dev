-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Marionette v0.1.0 -- Heirloom (video phase), from the clip WIN_20231003_00_48_14 at 1.5 s.
--
-- The still: a curtain of hanging figures, 4.7 across.  Each figure is a tall magenta body (a tapered panel with a
-- thin red line down its left edge and a cyan / white line down its right), set off from its neighbours by a
-- violet rim, a black band and a green gap, the gap deepening downward into a white "arch" line; the body's hem
-- glows red on the left and white on the right.  Above the bodies a crown band of pink / white heads in bright green,
-- torn into 85-line strips that slip vertically.  In the clip the whole curtain rolls UP (~220 lines / s) and Ron
-- was lengthening the bodies (crown 660 -> 325 lines in 1.5 s).
--
-- The patch, as fitted (marionette.py has every table):
--   one level field per pixel, s = P_line[t] + W_line[t] (ribs + tex) + lower tilt + source; colour =
--   PAL[night][slope variant][s >> 4]  (an RGB colouriser: rising slopes fringe red, falling slopes cyan / white)
--   P_line[t], 256 entries over the signed tile phase, built by the line engine for every line:
--     body  = A(nu) - K(nu) f - D(nu) dip(f) - m(nu) max(0, body - Lp(f))     (taper, gap arches, hem glow)
--     rim   = RIM(e), e = distance beyond the body's top / bottom (neck: the left part reaches higher / lower)
--     crown = heads profile - drop toward the rim;   P = max(body - rim, crown); W = crown texture weight
--     (fading in past the crossing; W = 1 flags the rim, where the gap-arch palette band is stepped over)
--
--   K1  Figures   how many figures across (1.8 .. 12, 4.7 = the clip)
--   K2  Waist     the bodies' taper and the gap arches: straight columns (0) .. the clip (512) .. pinched wedges
--   K3  Heads     the heads' size and the necks: no heads (0) .. the clip (512) .. big heads, tall necks
--   K4  Roll      bipolar: the curtain rolls up (+) or down (-); 772 = the clip's ~220 lines / s
--   K5  Glow      the hem glow: none (0) .. the clip (512) .. red / white flames up the whole body
--   K6  Etch      how far the camera (or the Osc) bends the figures
--   S7  Twin      two rows of figures
--   S8  Tear      Calm (the clip's strip slips) / Torn: 4x slips and sideways tears through the whole curtain
--   S9  Light     Day (the clip) / Night (the sibling clip: black ground, green, blue rims, pink)
--   S10 Sway      the figures swing side to side
--   S11 Source    Video / Osc
--   P12 CURTAIN   the bodies' length: a crowd of heads (0) .. the clip (688) .. the curtain closed (100 %)
--
-- Engine: vblank-zone sequencer (one serial divider, shared glide; the serial multiplier is shared with the line
-- engine, which never runs during the zone slots), a per-line engine kicked
-- at each line start (computes the NEXT line: its vertical terms on a serial multiplier, then the 256-entry profile
-- into a block-RAM bank), a 16-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).
-- Generated avid; output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4, luma 13/16) before the
-- blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.marionette_pkg.all;
use work.all;

architecture marionette of program_top is
    constant C_LAT : integer := 16;   -- = processing_delay_clks

    constant C_UC   : integer := 2161424;    -- tile phase at the screen centre (Q20 tile units: 2 tiles + 0.0613)
    constant C_TOP  : integer := 292;        -- the bodies' top (right part), figure lines
    constant C_PER  : integer := 276480;     -- the figure period, 1080 lines in Q8

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
    -- crown texture phase steps per pixel (234 / 558 per h) and the SD x offsets (480 h)
    function f_t1s(x : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(x) is
            when 4 => return to_unsigned(936, 16); when 3 => return to_unsigned(702, 16);
            when others => return to_unsigned(468, 16);
        end case;
    end function;
    function f_t2s(x : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(x) is
            when 4 => return to_unsigned(2232, 16); when 3 => return to_unsigned(1674, 16);
            when others => return to_unsigned(1116, 16);
        end case;
    end function;
    function f_t1x(x : unsigned(2 downto 0)) return unsigned is
    begin
        if x = 4 then return to_unsigned(46784, 16); else return to_unsigned(0, 16); end if;
    end function;
    function f_t2x(x : unsigned(2 downto 0)) return unsigned is
    begin
        if x = 4 then return to_unsigned(5696, 16); else return to_unsigned(0, 16); end if;
    end function;

    -- triangle of a 16-bit phase: 2 a - 512 with a = |ph/64 - 512| (one's complement on the falling half), -512 .. 510
    function f_tri10(p : unsigned(15 downto 0)) return signed is
        variable a   : unsigned(8 downto 0);
        variable v10 : unsigned(9 downto 0);
    begin
        if p(15) = '1' then a := p(14 downto 6); else a := not p(14 downto 6); end if;
        v10 := a & '0';
        return signed(resize(v10, 11)) - 512;
    end function;

    ---------------------------------------------------------------------------
    -- memories
    ---------------------------------------------------------------------------
    -- line buffer: the source value (camera: -luma, Osc: lattice - 111), half horizontal resolution
    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- profile table: two banks of 256 x {P 12, W 4}; the engine writes one bank while the pixel path reads the other
    type t_lut is array (0 to 511) of std_logic_vector(15 downto 0);
    signal lut : t_lut := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lut : signal is true;
    signal lut_wa, lut_ra : unsigned(8 downto 0) := (others => '0');
    signal lut_wd : std_logic_vector(15 downto 0) := x"0000";
    signal lut_we : std_logic := '0';
    signal lut_q  : std_logic_vector(15 downto 0) := x"0000";

    -- strip table (16 x {off8, hs8}), written in the zone, read by the line engine
    type t_strip is array (0 to 255) of std_logic_vector(15 downto 0);
    function f_strip_init return t_strip is
        variable v : t_strip := (others => x"0000");
    begin
        for i in 0 to 15 loop v(i) := C_MN_STRIP(i); end loop;
        return v;
    end function;
    signal strp : t_strip := f_strip_init;
    attribute no_rw_check of strp : signal is true;
    signal st_wa, st_ra : unsigned(7 downto 0) := (others => '0');
    signal st_wd : std_logic_vector(15 downto 0) := x"0000";
    signal st_we : std_logic := '0';
    signal st_q  : std_logic_vector(15 downto 0) := x"0000";

    -- ROMs (registered address and data)
    signal r_pal_a : unsigned(9 downto 0) := (others => '0');
    signal r_pal_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_e1_a, r_e2_a, r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_e1_q, r_e2_q, r_ex_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_ln_a  : unsigned(8 downto 0) := (others => '0');
    signal r_ln_q  : std_logic_vector(7 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- timing / zone / width
    ---------------------------------------------------------------------------
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_x0   : unsigned(8 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs (glided, Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(220 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(688 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(772, 10);
    signal s_dd : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_twin, s_torn, s_night, s_sway, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(10 downto 0) := (others => '0');
    signal z_sh   : signed(1 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_a    : unsigned(17 downto 0) := (others => '0');
    signal c_duh  : unsigned(11 downto 0) := to_unsigned(1283, 12);    -- tile phase per h, Q4 (u in Q20 per tile)
    signal c_dupx : unsigned(13 downto 0) := to_unsigned(2566, 14);    -- per pixel
    signal c_u0   : unsigned(21 downto 0) := (others => '0');          -- tile phase of the first pixel
    signal c_spd  : unsigned(11 downto 0) := (others => '0');          -- roll, Q8 lines per field
    signal c_roll : unsigned(18 downto 0) := (others => '0');          -- figure line at screen line 0, Q8
    signal c_swp  : unsigned(15 downto 0) := (others => '0');          -- sway time phase
    signal c_g    : unsigned(7 downto 0) := (others => '0');           -- source gain
    signal c_l    : unsigned(9 downto 0) := to_unsigned(724, 10);      -- body length, lines
    signal c_invl : unsigned(15 downto 0) := to_unsigned(11586, 16);   -- 2^23 / L
    signal c_nu0  : signed(8 downto 0) := to_signed(64, 9);            -- glow onset (nu7)
    signal c_gmin : unsigned(7 downto 0) := to_unsigned(255, 8);       -- glow strength
    signal c_neck : unsigned(7 downto 0) := to_unsigned(62, 8);        -- neck, lines
    signal c_hpk  : unsigned(8 downto 0) := to_unsigned(256, 9);       -- heads level (Q8)
    signal c_den  : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal c_hs   : unsigned(9 downto 0) := to_unsigned(256, 10);      -- heads index step (Q8)
    signal c_ci0  : signed(17 downto 0) := to_signed(128, 18);         -- heads index start (Q8)
    signal c_step : unsigned(12 downto 0) := to_unsigned(256, 13);     -- figure lines per output line, Q8
    signal c_wst  : unsigned(9 downto 0) := to_unsigned(512, 10);      -- waist (Q9)
    signal z_lfsr : unsigned(31 downto 0) := x"6D2B79F5";
    signal z_div  : unsigned(10 downto 0) := (others => '0');          -- divider remainder
    signal z_dq   : unsigned(23 downto 0) := (others => '0');
    signal z_dd   : unsigned(9 downto 0) := (others => '0');
    signal z_dn   : unsigned(4 downto 0) := (others => '0');
    signal z_dsel : std_logic := '0';

    -- field serial multiplier
    signal z_fa  : unsigned(31 downto 0);
    signal z_fb  : unsigned(15 downto 0);
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- oscillator source
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- line engine
    signal e_lc    : unsigned(8 downto 0) := (others => '1');
    signal e_zk    : std_logic := '0';
    signal e_phi   : unsigned(18 downto 0) := (others => '0');
    signal e_pi    : unsigned(10 downto 0) := (others => '0');
    signal e_pos0  : signed(11 downto 0) := (others => '0');
    signal e_off   : signed(9 downto 0) := (others => '0');
    signal e_hs    : signed(7 downto 0) := (others => '0');
    signal e_phs   : signed(11 downto 0) := (others => '0');
    signal e_pos   : signed(11 downto 0) := (others => '0');
    signal e_inp   : std_logic := '0';
    signal e_bot   : std_logic := '0';
    signal e_nu    : unsigned(6 downto 0) := (others => '0');
    signal e_dt, e_db : signed(11 downto 0) := (others => '0');
    signal e_pq8   : unsigned(19 downto 0) := (others => '0');
    signal e_phu   : signed(11 downto 0) := (others => '0');
    signal e_da, e_dk, e_dtb, e_sm : unsigned(7 downto 0) := (others => '0');
    signal e_a     : signed(13 downto 0) := (others => '0');
    signal e_k     : unsigned(10 downto 0) := (others => '0');
    signal e_d8    : unsigned(7 downto 0) := (others => '0');
    signal e_m8    : unsigned(7 downto 0) := (others => '0');
    signal e_tri   : signed(10 downto 0) := (others => '0');
    signal e_xsw   : signed(18 downto 0) := (others => '0');
    signal e_lnsel : std_logic := '0';
    signal e_lna   : unsigned(8 downto 0) := (others => '0');
    signal m_a     : unsigned(31 downto 0) := (others => '0');
    signal m_b     : unsigned(15 downto 0) := (others => '0');
    signal m_acc   : unsigned(31 downto 0) := (others => '0');
    signal n_u0    : unsigned(21 downto 0) := (others => '0');
    signal n_rib, n_t1, n_t2 : unsigned(15 downto 0) := (others => '0');
    signal n_m8    : unsigned(7 downto 0) := (others => '0');
    signal w_bank, r_bank : std_logic := '0';

    -- profile build pipeline (one entry per clock)
    signal q_run   : std_logic := '0';
    signal q_i     : unsigned(7 downto 0) := (others => '0');
    signal q_kacc  : unsigned(18 downto 0) := (others => '0');
    signal q_ci    : signed(17 downto 0) := (others => '0');
    signal q_ci2   : signed(17 downto 0) := (others => '0');
    signal q8_cr8  : unsigned(7 downto 0) := (others => '0');
    signal q8_cw4, q9a_cw : unsigned(3 downto 0) := (others => '0');
    signal q_v     : std_logic_vector(12 downto 0) := (others => '0');   -- valid per stage
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    signal q_wi    : unsigned(7 downto 0) := (others => '0');
    signal q1b_fi  : unsigned(7 downto 0) := (others => '0');
    signal q1b_kf  : unsigned(10 downto 0) := (others => '0');
    signal e_stq   : std_logic_vector(15 downto 0) := (others => '0');
    signal e_lnf   : std_logic_vector(7 downto 0) := (others => '0');
    signal q_rla   : unsigned(6 downto 0) := (others => '0');
    signal q8b_b2  : signed(13 downto 0) := (others => '0');
    signal q8b_e   : unsigned(8 downto 0) := (others => '0');
    signal q8b_top : std_logic := '0';
    signal q9_rim8 : unsigned(7 downto 0) := (others => '0');
    signal q1_fi, q2_fi : unsigned(7 downto 0) := (others => '0');
    signal q1_kf, q2_kf : unsigned(10 downto 0) := (others => '0');
    signal q2_nk, q2_dp : unsigned(7 downto 0) := (others => '0');
    signal q9_cw, q10_cw : unsigned(3 downto 0) := (others => '0');
    signal q3_ddl, q3_ddh, q3_shl, q3_shh : unsigned(11 downto 0) := (others => '0');
    signal q3_crl, q3_crh : unsigned(12 downto 0) := (others => '0');   -- (crown level partials, stage 9)
    signal q6_mdl, q6_mdh : unsigned(15 downto 0) := (others => '0');
    signal q3_b0   : signed(13 downto 0) := (others => '0');
    signal q4_lp : signed(12 downto 0) := (others => '0');
    signal q3_fi : unsigned(7 downto 0) := (others => '0');
    signal q4_b1   : signed(13 downto 0) := (others => '0');
    signal q4_sh   : unsigned(7 downto 0) := (others => '0');
    signal q5_b1, q6_b1 : signed(13 downto 0) := (others => '0');
    signal q5_df   : unsigned(11 downto 0) := (others => '0');
    signal q5_et, q5_eb : signed(12 downto 0) := (others => '0');
    signal q6_e, q7_e, q8_e : unsigned(8 downto 0) := (others => '0');
    signal q6_top, q7_top, q8_top, q9_top : std_logic := '0';
    signal q7_b2, q8_b2, q9_b2 : signed(13 downto 0) := (others => '0');
    signal q9_cd   : unsigned(12 downto 0) := (others => '0');
    signal q9_cr   : unsigned(12 downto 0) := (others => '0');
    signal q10_crn, q10_b2, q11_pan, q11_crn : signed(13 downto 0) := (others => '0');
    signal q10_rl  : unsigned(11 downto 0) := (others => '0');
    signal q11_cw  : unsigned(3 downto 0) := (others => '0');
    signal q9_rf, q10_rf, q11_rf, q12_rf : std_logic := '0';
    signal q12_d   : signed(14 downto 0) := (others => '0');
    signal q12_crn, q12_pan : signed(13 downto 0) := (others => '0');
    signal q12_cw  : unsigned(3 downto 0) := (others => '0');

    -- beam
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_u   : unsigned(21 downto 0) := (others => '0');
    signal b_t1, b_t2 : unsigned(15 downto 0) := (others => '0');
    signal l_rib : unsigned(15 downto 0) := (others => '0');
    signal l_m8  : unsigned(7 downto 0) := (others => '0');

    -- pixel path
    signal p1_i  : unsigned(7 downto 0) := (others => '0');
    signal p1_tk : unsigned(1 downto 0) := (others => '0');
    signal p1_rb, p1_t1, p1_t2 : unsigned(15 downto 0) := (others => '0');
    signal p2_tr, p2_t1, p2_t2 : signed(10 downto 0) := (others => '0');
    signal p2_tc : signed(7 downto 0) := (others => '0');
    signal p2_tk, p3_tk, p4_tk : unsigned(1 downto 0) := (others => '0');
    signal p3_p  : unsigned(11 downto 0) := (others => '0');
    signal p3_w  : unsigned(3 downto 0) := (others => '0');
    signal p3_ts, p3_t2 : signed(11 downto 0) := (others => '0');
    signal p3_tc : signed(7 downto 0) := (others => '0');
    signal p3_sv : signed(7 downto 0) := (others => '0');
    signal p4_p, p5_p : unsigned(11 downto 0) := (others => '0');
    signal p4_w  : unsigned(3 downto 0) := (others => '0');
    signal p4_rf, p5_rf, p6_rf, p7_rf : std_logic := '0';
    signal p4_ts : signed(11 downto 0) := (others => '0');
    signal p4_tll, p4_tlh : signed(12 downto 0) := (others => '0');
    signal p4_srl, p4_srh : signed(12 downto 0) := (others => '0');
    signal p5_wt : signed(15 downto 0) := (others => '0');
    signal p5_tl, p5_sr : signed(13 downto 0) := (others => '0');
    signal p6_a, p6_b : signed(14 downto 0) := (others => '0');
    signal p7_s  : unsigned(11 downto 0) := (others => '0');
    type t_u12a is array (natural range <>) of unsigned(11 downto 0);
    type t_sdring is array (0 to 255) of std_logic_vector(11 downto 0);
    signal sdr : t_sdring := (others => (others => '0'));
    signal sd_q, sd_f : std_logic_vector(11 downto 0) := (others => '0');
    signal p8_var : std_logic := '1';
    signal p_vc  : unsigned(3 downto 0) := (others => '0');
    signal p10_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent 8-sums, the 8-back tap from a block-RAM
    -- ring), luma [1 2 1]/4 on taps aligned with the chroma centre, luma 13/16
    type t_cring is array (0 to 255) of std_logic_vector(19 downto 0);
    signal cr : t_cring := (others => (others => '0'));
    signal cr_q, cr_f : std_logic_vector(19 downto 0) := (others => '0');
    type t_sring is array (0 to 255) of std_logic_vector(2 downto 0);
    signal sr : t_sring := (others => (others => '1'));
    signal sr_q, sr_f : std_logic_vector(2 downto 0) := (others => '1');
    attribute no_rw_check of cr : signal is true;
    attribute no_rw_check of sr : signal is true;
    attribute no_rw_check of sdr : signal is true;
    signal rc     : unsigned(7 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_ly   : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y13, s_u13, s_v13 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories: line buffer (collisions possible: read-before-write kept),
    -- profile banks and strip table (collision-free: no_rw_check), ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_lut : process(clk)
    begin
        if rising_edge(clk) then
            lut_q <= lut(to_integer(lut_ra));
            if lut_we = '1' then lut(to_integer(lut_wa)) <= lut_wd; end if;
        end if;
    end process p_lut;

    p_strp : process(clk)
    begin
        if rising_edge(clk) then
            st_q <= strp(to_integer(st_ra));
            if st_we = '1' then strp(to_integer(st_wa)) <= st_wd; end if;
        end if;
    end process p_strp;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_pal_q <= C_MN_PAL(to_integer(r_pal_a));
            r_e1_q  <= C_MN_E1(to_integer(r_e1_a));
            r_e2_q  <= C_MN_E2(to_integer(r_e2_a));
            r_ex_q  <= C_MN_EXP2(to_integer(r_ex_a));
            r_ln_q  <= C_MN_LN(to_integer(r_ln_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_e  : signed(14 downto 0);
        variable v_r  : unsigned(19 downto 0);
        variable v_rm : unsigned(10 downto 0);
        variable v_b7 : unsigned(9 downto 0);
        variable v_k  : unsigned(9 downto 0);
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
            -- roll: bipolar, +-8 deadband
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
            s_vneg <= v_s(10);
            if v_s > -9 and v_s < 9 then
                s_dd <= (others => '0');
            elsif v_s(10) = '1' then
                v_k := unsigned(not v_s(9 downto 0)) - 7;      -- |v| - 8
                s_dd <= v_k(8 downto 0);
            else
                v_k := unsigned(v_s(9 downto 0)) - 8;
                s_dd <= v_k(8 downto 0);
            end if;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_twin  <= registers_in(6)(0);
                s_torn  <= registers_in(6)(1);
                s_night <= registers_in(6)(2);
                s_sway  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4 =>
                        s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3 =>
                        s_x0 <= (others => '0'); s_oxs <= to_unsigned(153, 9);
                    when others =>
                        s_x0 <= (others => '0'); s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);
            if s_twin = '1' then c_step <= s_ys & '0'; else c_step <= '0' & s_ys; end if;

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 4      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k pre-muxes its operands at zc = 30 + 24 k, loads at 31 + 24 k, the product is
            -- read at 50 + 24 k
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(14, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;

            -- serial divider (restoring, 24 steps): invL = 2^23 / L, then heads step = 2^16 / den
            st_we <= '0';
            if s_zone = '1' and (s_zc = 280 or s_zc = 320) then
                if s_zc = 280 then
                    z_dq <= to_unsigned(8388608, 24); z_dd <= c_l;
                else
                    z_dq <= to_unsigned(65536, 24); z_dd <= '0' & c_den;
                end if;
                z_div <= (others => '0');
                z_dn <= to_unsigned(24, 5);
            elsif z_dn /= 0 then
                v_rm := z_div(9 downto 0) & z_dq(23);
                if v_rm >= resize(z_dd, 11) then
                    z_div <= v_rm - resize(z_dd, 11);
                    z_dq <= z_dq(22 downto 0) & '1';
                else
                    z_div <= v_rm;
                    z_dq <= z_dq(22 downto 0) & '0';
                end if;
                z_dn <= z_dn - 1;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's speed), strip slips, tile exponent
                    when 20 =>
                        if s_vneg = '1' then
                            if c_roll < resize(c_spd, 19) then c_roll <= c_roll + to_unsigned(C_PER, 19) - resize(c_spd, 19);
                            else c_roll <= c_roll - resize(c_spd, 19); end if;
                        else
                            if c_roll + resize(c_spd, 19) >= C_PER then c_roll <= c_roll + resize(c_spd, 19) - to_unsigned(C_PER, 19);
                            else c_roll <= c_roll + resize(c_spd, 19); end if;
                        end if;
                        c_swp <= c_swp + 382;
                        z_lfsr <= z_lfsr xor shift_left(z_lfsr, 13);
                        v_d := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        v_e := shift_left(resize(v_d, 15), 3) + shift_left(resize(v_d, 15), 1) + resize(v_d, 15);   -- 11 d
                        z_e <= v_e(14 downto 4);
                        c_wst <= s_g2(12 downto 3);
                    when 21 =>
                        r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= z_e(9 downto 8);
                        z_lfsr <= z_lfsr xor shift_right(z_lfsr, 17);
                    when 22 =>
                        z_lfsr <= z_lfsr xor shift_left(z_lfsr, 5);
                    when 23 =>
                        z_mt <= unsigned(r_ex_q);
                        if z_lfsr(1 downto 0) = "00" then
                            v_b7 := resize(z_lfsr(14 downto 8), 10);
                            v_b7 := shift_right(shift_left(v_b7, 2) + v_b7, 3);                 -- 0 .. 79
                            st_we <= '1';
                            st_wa <= "0000" & z_lfsr(7 downto 4);
                            st_wd <= std_logic_vector(resize(signed(resize(v_b7, 11)) - 40, 8)) & std_logic_vector(z_lfsr(23 downto 16));
                        end if;

                    when 50 =>                       -- du_h = 1283 * mt >> (15 - sh)
                        case to_integer(z_sh) is
                            when -2     => c_duh <= m_acc(28 downto 17);
                            when -1     => c_duh <= m_acc(27 downto 16);
                            when 0      => c_duh <= m_acc(26 downto 15);
                            when others => c_duh <= m_acc(25 downto 14);
                        end case;
                    when 74 =>
                        c_u0 <= to_unsigned(C_UC, 22) - m_acc(21 downto 0);
                        case to_integer(s_xs) is
                            when 4      => c_dupx <= c_duh & "00";
                            when 3      => c_dupx <= resize(c_duh, 14) + resize(c_duh & '0', 14);
                            when others => c_dupx <= '0' & c_duh & '0';
                        end case;
                    when 122 => z_a <= m_acc(17 downto 0);
                    when 146 => c_spd <= m_acc(21 downto 10);
                    when 170 =>
                        if m_acc(19 downto 11) > 255 then c_g <= (others => '1'); else c_g <= m_acc(18 downto 11); end if;
                    when 194 => c_l <= 160 + m_acc(19 downto 10);
                    when 218 =>
                        if s_g5(12 downto 3) < 512 then c_nu0 <= 64 + signed(resize(m_acc(16 downto 9), 9));
                        else c_nu0 <= 64 - signed(resize(m_acc(16 downto 9), 9)); end if;
                        if s_g5(12) = '1' then c_gmin <= (others => '1'); else c_gmin <= s_g5(11 downto 4); end if;
                    when 242 =>
                        if s_g3(12 downto 3) <= 512 then c_neck <= m_acc(16 downto 9);
                        else c_neck <= 62 + m_acc(16 downto 9); end if;
                        if s_g3(12 downto 3) > 640 then c_hpk <= to_unsigned(320, 9); else c_hpk <= s_g3(12 downto 4); end if;
                    when 266 => c_den <= 141 + m_acc(17 downto 9);
                    when 306 => c_invl <= z_dq(15 downto 0);
                    when 346 => c_hs <= z_dq(9 downto 0);
                    when 386 => c_ci0 <= to_signed(32896, 18) - signed(resize(m_acc(18 downto 1), 18));
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    -- operands of the shared serial multiplier for the zone slots (combinational from the registered slot number;
    -- the line engine owns the multiplier outside the zone slots, see p_line)
    p_zop : process(all)
    begin
        z_fa <= (others => '0'); z_fb <= (others => '0');
        case to_integer(z_sel) is
            when 0  => z_fa <= to_unsigned(1283, 32);         z_fb <= z_mt;                                 -- tile step
            when 1  => z_fa <= resize(c_duh, 32);             z_fb <= resize(1920 - resize(s_x0, 11), 16);  -- u0
            when 3  => z_fa <= resize(s_dd, 32);              z_fb <= resize(s_dd, 16);                     -- roll^2
            when 4  => z_fa <= resize(z_a, 32);               z_fb <= to_unsigned(15, 16);
            when 5  => z_fa <= resize(s_g6(12 downto 3), 32); z_fb <= resize(s_g6(12 downto 3), 16);       -- etch^2
            when 6  => z_fa <= resize(s_gp(12 downto 3), 32); z_fb <= to_unsigned(840, 16);                 -- length
            when 7  =>                                                                                      -- glow onset
                if s_g5(12 downto 3) < 512 then
                    z_fa <= resize(512 - s_g5(12 downto 3), 32); z_fb <= to_unsigned(51, 16);
                else
                    z_fa <= resize(s_g5(12 downto 3) - 512, 32); z_fb <= to_unsigned(96, 16);
                end if;
            when 8  =>                                                                                      -- neck
                if s_g3(12 downto 3) <= 512 then
                    z_fa <= resize(s_g3(12 downto 3), 32); z_fb <= to_unsigned(62, 16);
                else
                    z_fa <= resize(s_g3(12 downto 3) - 512, 32); z_fb <= to_unsigned(110, 16);
                end if;
            when 9  =>                                                                                      -- heads width
                if s_g3(12 downto 3) > 819 then z_fa <= to_unsigned(819, 32);
                else z_fa <= resize(s_g3(12 downto 3), 32); end if;
                z_fb <= to_unsigned(115, 16);
            when others => z_fa <= resize(c_hs, 32);           z_fb <= to_unsigned(255, 16);                -- heads start
        end case;
    end process p_zop;

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
    -- Source side: oscillator (2-D triangle lattice, drifting) or the camera,
    -- written at half resolution: camera -luma (0 .. -128), Osc lattice - 111
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_c : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);
            s_lb_we <= data_in.avid and not s_wcol(0);
            s_lb_wa <= s_wcol(10 downto 1);
            if s_src = '1' then
                s_lb_wd <= std_logic_vector(s_osum - 111);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 235 then v_l := to_unsigned(219, 8);
                elsif v_l > 16 then v_l := v_l - 16;
                else v_l := (others => '0'); end if;
                v_c := shift_left(resize(v_l, 16), 7) + shift_left(resize(v_l, 16), 4) + shift_left(resize(v_l, 16), 2)
                       + shift_left(resize(v_l, 16), 1);                                                   -- 150 c
                s_lb_wd <= std_logic_vector(0 - v_c(15 downto 8));
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line: its vertical terms on one serial multiplier
    -- (16 iterations, the product at load + 17), then builds the line's
    -- 256-entry profile into the bank the pixel path is not reading
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_p   : unsigned(19 downto 0);
        variable v_12  : unsigned(11 downto 0);
        variable v_o   : signed(9 downto 0);
        variable v_x   : signed(15 downto 0);
        variable v_n   : unsigned(15 downto 0);
        variable v_sw  : signed(19 downto 0);
        variable v_ht  : signed(19 downto 0);
        variable v_u   : unsigned(21 downto 0);
        variable v_fi  : unsigned(7 downto 0);
        variable v_ci  : signed(17 downto 0);
        variable v_df  : signed(14 downto 0);
        variable v_et, v_eb : signed(12 downto 0);
        variable v_e   : unsigned(8 downto 0);
        variable v_cx  : unsigned(5 downto 0);
        variable v_c7  : unsigned(6 downto 0);
        variable v_w   : unsigned(3 downto 0);
        variable v_pn  : signed(14 downto 0);
        variable v_rl  : unsigned(11 downto 0);
        variable v_16  : unsigned(15 downto 0);
        variable v_17  : unsigned(16 downto 0);
        variable v_20  : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 420 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_phi <= c_roll;
                e_lc <= (others => '0');
            elsif g_start = '1' then
                v_p := resize(e_phi, 20) + resize(c_step, 20);
                if v_p >= C_PER then v_p := v_p - C_PER; end if;
                e_phi <= v_p(18 downto 0);
                e_lc <= (others => '0');
                r_bank <= w_bank;
                w_bank <= not w_bank;
            elsif e_lc /= 511 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(30 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);
            if z_ld = '1' then                              -- a zone slot of the sequencer
                m_a <= z_fa; m_b <= z_fb; m_acc <= (others => '0');
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    e_pi <= e_phi(18 downto 8);
                when 2 =>
                    v_12 := resize(e_pi, 12) + resize(e_pi & '0', 12);
                    st_ra <= "0000" & v_12(11 downto 8);
                    e_pos0 <= signed(resize(e_pi, 12)) - C_TOP;
                when 3 =>
                    if e_pos0 < 0 then e_pos0 <= e_pos0 + 1080; end if;
                when 4 =>
                    e_stq <= st_q;
                when 5 =>
                    v_o := resize(signed(e_stq(15 downto 8)), 10);
                    if s_torn = '1' then v_o := shift_left(v_o, 2); end if;          -- Torn: 4x the slips
                    e_off <= v_o;
                    e_hs <= signed(e_stq(7 downto 0));
                    if e_pos0 < signed(resize(c_l, 12)) then e_inp <= '1'; else e_inp <= '0'; end if;
                    if shift_left(resize(e_pos0, 13), 1) < 1080 + signed(resize(c_l, 13)) then e_bot <= '1'; else e_bot <= '0'; end if;
                when 6 =>
                    e_phs <= signed(resize(e_pi, 12)) + resize(e_off, 12);
                when 7 =>
                    e_phu <= e_phs + 256;                      -- unwrapped (texture phases stay continuous in a strip)
                    if e_phs < 0 then e_phs <= e_phs + 1080;
                    elsif e_phs >= 1080 then e_phs <= e_phs - 1080; end if;
                    m_a <= resize(unsigned(e_pos0), 32); m_b <= c_invl; m_acc <= (others => '0');
                when 8 =>
                    e_pos <= e_phs - C_TOP;
                    e_pq8 <= unsigned(e_phu(11 downto 0)) & e_phi(7 downto 0);
                when 9 =>
                    if e_pos < 0 then e_pos <= e_pos + 1080; end if;
                when 10 =>
                    if e_pos < signed(resize(c_l, 12)) then
                        e_dt <= -e_pos;
                    else
                        e_dt <= 1080 - e_pos;
                    end if;
                    e_db <= e_pos - signed(resize(c_l, 12));
                when 24 =>
                    if e_inp = '1' then
                        if m_acc(31 downto 23) /= 0 then e_nu <= (others => '1'); else e_nu <= m_acc(22 downto 16); end if;
                    elsif e_bot = '1' then
                        e_nu <= (others => '1');
                    else
                        e_nu <= (others => '0');
                    end if;
                    m_a <= resize(e_pq8, 32); m_b <= to_unsigned(565, 16); m_acc <= (others => '0');
                when 28 => e_da <= unsigned(e_lnf);
                when 29 => e_dk <= unsigned(e_lnf);
                when 30 => e_dtb <= unsigned(e_lnf);
                when 41 =>
                    n_rib <= m_acc(23 downto 8);
                    m_a <= resize(e_pq8, 32); m_b <= to_unsigned(936, 16); m_acc <= (others => '0');
                when 58 =>
                    n_t1 <= m_acc(23 downto 8) + f_t1x(s_xs);
                    m_a <= resize(e_pq8, 32); m_b <= to_unsigned(1394, 16); m_acc <= (others => '0');
                when 75 =>
                    n_t2 <= m_acc(23 downto 8) - f_t2x(s_xs);
                    m_a <= resize(c_wst, 32); m_b <= resize(e_da, 16); m_acc <= (others => '0');
                when 92 =>
                    e_a <= to_signed(3222, 14) - signed(resize(m_acc(17 downto 9) & "00", 14));
                    m_a <= resize(c_wst, 32); m_b <= resize(e_dk, 16); m_acc <= (others => '0');
                when 109 =>
                    e_k <= to_unsigned(1758, 11) - resize(m_acc(17 downto 9) & "00", 11);
                    m_a <= resize(c_wst, 32); m_b <= resize(e_dtb, 16); m_acc <= (others => '0');
                when 126 =>
                    if m_acc(17 downto 9) > 255 then e_d8 <= (others => '1'); else e_d8 <= m_acc(16 downto 9); end if;
                when 130 =>
                    m_a <= resize(c_gmin, 32); m_b <= resize(e_sm, 16); m_acc <= (others => '0');
                when 147 =>
                    e_m8 <= m_acc(15 downto 8);
                    m_a <= resize(e_phi, 32); m_b <= to_unsigned(345, 16); m_acc <= (others => '0');
                when 164 =>
                    v_n := m_acc(25 downto 10) - c_swp;
                    e_tri <= f_tri10(v_n);
                when 165 =>
                    m_a <= to_unsigned(351, 32); m_b <= resize(unsigned(abs(e_tri)), 16); m_acc <= (others => '0');   -- 0.17 tile
                when 182 =>
                    if s_sway = '0' then
                        e_xsw <= (others => '0');
                    elsif e_tri < 0 then
                        e_xsw <= -signed(resize(m_acc(17 downto 0), 19));
                    else
                        e_xsw <= signed(resize(m_acc(17 downto 0), 19));
                    end if;
                when 183 =>
                    v_sw := resize(e_xsw, 20);
                    if s_torn = '1' then
                        v_ht := shift_left(resize(e_hs, 20), 10) + shift_left(resize(e_hs, 20), 8) + shift_left(resize(e_hs, 20), 5);
                    else
                        v_ht := (others => '0');
                    end if;
                    v_u := c_u0 - unsigned(resize(v_sw, 22)) - unsigned(resize(v_ht, 22));
                    n_u0 <= v_u;
                    n_m8 <= e_m8;
                when others => null;
            end case;

            -- LN port: the engine's vertical-table reads, then (during the build) the rim reads
            e_lnf <= r_ln_q;
            if q_run = '1' or q_v(8 downto 0) /= "000000000" then
                r_ln_a <= "00" & q_rla;
            else
                case to_integer(e_lc) is
                    when 25     => r_ln_a <= "011" & e_nu(6 downto 1);
                    when 26     => r_ln_a <= "100" & e_nu(6 downto 1);
                    when 27     => r_ln_a <= "101" & e_nu(6 downto 1);
                    when 126 =>
                        v_x := shift_left(resize(signed(resize(e_nu, 9)) - c_nu0, 16), 6) + shift_left(resize(signed(resize(e_nu, 9)) - c_nu0, 16), 3)
                               - resize(signed(resize(e_nu, 9)) - c_nu0, 16);                     -- 71 x
                        v_x := shift_right(v_x, 6);
                        if v_x < 0 then v_cx := (others => '0');
                        elsif v_x > 63 then v_cx := (others => '1');
                        else v_cx := unsigned(v_x(5 downto 0)); end if;
                        r_ln_a <= "010" & v_cx(5 downto 0);
                    when others => null;
                end case;
            end if;
            if e_lc = 129 then e_sm <= unsigned(e_lnf); end if;

            -------------------------------------------------------------------
            -- profile build: entry i at Q0 (lc 185 + i), written at Q11
            -------------------------------------------------------------------
            if e_lc = 185 then
                q_run <= '1';
                q_i <= (others => '0');
                q_kacc <= shift_left(resize(e_k, 19), 8) - resize(e_k, 19);      -- K * 255
            elsif q_run = '1' then
                q_i <= q_i + 1;
                if q_i < 127 then
                    q_kacc <= q_kacc - shift_left(resize(e_k, 19), 1);
                elsif q_i > 127 then
                    q_kacc <= q_kacc + shift_left(resize(e_k, 19), 1);
                end if;
                if q_i = 255 then q_run <= '0'; end if;
            end if;
            q_v <= q_v(11 downto 0) & q_run;
            -- Q0 -> Q1
            r_e1_a <= q_i;
            if q_i(7) = '1' then v_fi := q_i(6 downto 0) & '1'; else v_fi := (not q_i(6 downto 0)) & '1'; end if;
            q1_fi <= v_fi;
            q1_kf <= q_kacc(18 downto 8);
            -- Q1 -> Q2: wait for the ROM data
            q1b_fi <= q1_fi; q1b_kf <= q1_kf;
            -- Q2 -> Q3: ROM data to fabric; Lp = 2226 - 154 fi / 256
            q2_nk <= unsigned(r_e1_q(15 downto 8)); q2_dp <= unsigned(r_e1_q(7 downto 0));
            q2_kf <= q1b_kf;
            q2_fi <= q1b_fi;
            -- Q2 -> Q3: products, body0 = A - K f
            q3_ddl <= q2_dp * e_d8(3 downto 0); q3_ddh <= q2_dp * e_d8(7 downto 4);
            q3_shl <= q2_nk * c_neck(3 downto 0); q3_shh <= q2_nk * c_neck(7 downto 4);

            q3_b0 <= e_a - signed(resize(q2_kf, 14));
            q3_fi <= q2_fi;
            -- Q3 -> Q4: body1 = body0 - D dip
            v_16 := resize(q3_ddl, 16) + shift_left(resize(q3_ddh, 16), 4);
            q4_b1 <= q3_b0 - signed(resize(v_16(15 downto 6), 14));
            v_16 := resize(q3_shl, 16) + shift_left(resize(q3_shh, 16), 4);
            q4_sh <= v_16(15 downto 8);
            q4_lp <= to_signed(2226, 13) - signed(resize(shift_right(q3_fi, 1), 13)) - signed(resize(shift_right(q3_fi, 3), 13));
            -- Q4 -> Q5: plateau excess, rim distances (foot = 5/8 of the neck)
            v_df := resize(q4_b1, 15) - resize(q4_lp, 15);
            if v_df < 0 then q5_df <= (others => '0');
            elsif v_df > 4095 then q5_df <= (others => '1');
            else q5_df <= unsigned(v_df(11 downto 0)); end if;
            q5_et <= resize(e_dt, 13) - signed(resize(q4_sh, 13));
            q5_eb <= resize(e_db, 13) - signed(resize(shift_right(resize(q4_sh, 11) + shift_left(resize(q4_sh, 11), 2), 3), 13));
            q5_b1 <= q4_b1;
            -- Q5 -> Q6: m * excess; e = distance beyond the nearer end
            q6_mdl <= q5_df * e_m8(3 downto 0); q6_mdh <= q5_df * e_m8(7 downto 4);
            if q5_et < 0 then v_et := (others => '0'); else v_et := q5_et; end if;
            if q5_eb < 0 then v_eb := (others => '0'); else v_eb := q5_eb; end if;
            if v_et <= v_eb then
                q6_top <= '1';
                if v_et > 511 then v_e := (others => '1'); else v_e := unsigned(v_et(8 downto 0)); end if;
            else
                q6_top <= '0';
                if v_eb > 511 then v_e := (others => '1'); else v_e := unsigned(v_eb(8 downto 0)); end if;
            end if;
            if s_twin = '1' then v_e := '0' & v_e(8 downto 1); end if;        -- Twin: rims as thick on screen
            q6_e <= v_e;
            q6_b1 <= q5_b1;
            -- crown ROM address for this entry (stage 6): heads-scaled index from the second accumulator
            if e_lc = 191 then q_ci2 <= c_ci0; else q_ci2 <= q_ci2 + signed(resize(c_hs, 18)); end if;
            v_ci := shift_right(q_ci2, 8);
            if v_ci < 0 then r_e2_a <= (others => '0');
            elsif v_ci > 255 then r_e2_a <= (others => '1');
            else r_e2_a <= unsigned(v_ci(7 downto 0)); end if;
            -- Q6 -> Q7: body2; rim ROM address
            v_20 := resize(q6_mdl, 20) + shift_left(resize(q6_mdh, 20), 4);
            q7_b2 <= q6_b1 - signed(resize(v_20(19 downto 8), 14));
            if q6_e > 127 then q_rla <= (others => '1'); else q_rla <= q6_e(6 downto 0); end if;
            q7_e <= q6_e; q7_top <= q6_top;
            -- (rim ROM address, then ROM read)
            q8_b2 <= q7_b2; q8_e <= q7_e; q8_top <= q7_top;
            q8b_b2 <= q8_b2; q8b_e <= q8_e; q8b_top <= q8_top;
            -- crown ROM data to fabric (stage 8), heads level product (stage 9), sum (stage 10)
            q8_cr8 <= unsigned(r_e2_q(11 downto 4)); q8_cw4 <= unsigned(r_e2_q(3 downto 0));
            q3_crl <= c_hpk * q8_cr8(3 downto 0); q3_crh <= c_hpk * q8_cr8(7 downto 4); q9a_cw <= q8_cw4;
            v_17 := resize(q3_crl, 17) + shift_left(resize(q3_crh, 17), 4);
            q9_cr <= v_17(16 downto 4); q9_cw <= q9a_cw;
            -- rim to fabric; crown drop toward the rim, 34 lu per line inside 50 (top) / 66 (foot) lines
            q9_rim8 <= unsigned(r_ln_q);
            if q8b_top = '1' then
                if q8b_e < 50 then v_c7 := resize(50 - q8b_e, 7); else v_c7 := (others => '0'); end if;
            else
                if q8b_e < 66 then v_c7 := resize(66 - q8b_e, 7); else v_c7 := (others => '0'); end if;
            end if;
            q9_cd <= shift_left(resize(v_c7, 13), 5) + shift_left(resize(v_c7, 13), 1);
            q9_b2 <= q8b_b2; q9_top <= q8b_top;
            if q8b_e /= 0 then q9_rf <= '1'; else q9_rf <= '0'; end if;
            -- rim in lu (foot rim x 21/32); crown = heads - drop
            if q9_top = '1' then
                q10_rl <= shift_left(resize(q9_rim8, 12), 4);
            else
                q10_rl <= resize(shift_right(shift_left(resize(q9_rim8, 13), 4) + shift_left(resize(q9_rim8, 13), 2) + resize(q9_rim8, 13), 1), 12);
            end if;
            q10_b2 <= q9_b2;
            q10_crn <= signed(resize(q9_cr, 14)) - signed(resize(q9_cd, 14));
            q10_cw <= q9_cw; q10_rf <= q9_rf;
            -- panel = body - rim
            q11_pan <= q10_b2 - signed(resize(q10_rl, 14));
            q11_crn <= q10_crn; q11_cw <= q10_cw; q11_rf <= q10_rf;
            -- crown margin over the panel (the crown texture fades in over 320 lu past the crossing)
            q12_d <= resize(q11_crn, 15) - resize(q11_pan, 15);
            q12_crn <= q11_crn; q12_pan <= q11_pan; q12_cw <= q11_cw; q12_rf <= q11_rf;
            -- P = max(panel, crown), clamp, write
            lut_we <= q_v(12);
            lut_wa <= w_bank & q_wi;
            if e_lc = 185 then q_wi <= (others => '0'); elsif q_v(12) = '1' then q_wi <= q_wi + 1; end if;
            if q12_d > 0 then
                v_pn := resize(q12_crn, 15);
                if unsigned(q12_d(13 downto 5)) < resize(q12_cw, 9) then v_w := unsigned(q12_d(8 downto 5)); else v_w := q12_cw; end if;
                if v_w = 1 then v_w := to_unsigned(2, 4); end if;
                lut_wd(3 downto 0) <= std_logic_vector(v_w);
            else
                v_pn := resize(q12_pan, 15);
                lut_wd(3 downto 0) <= "000" & q12_rf;          -- W = 1 flags the rim: no arch band there
            end if;
            if v_pn < 0 then lut_wd(15 downto 4) <= (others => '0');
            elsif v_pn > 4095 then lut_wd(15 downto 4) <= (others => '1');
            else lut_wd(15 downto 4) <= std_logic_vector(v_pn(11 downto 0)); end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: tile phase (Q20 per tile, 2 tile-count bits on top), texture phases
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_px <= (others => '0');
                b_u  <= n_u0;
                b_t1 <= n_t1;
                b_t2 <= n_t2;
                l_rib <= n_rib;
                l_m8 <= n_m8;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_u  <= b_u + resize(c_dupx, 22);
                b_t1 <= b_t1 + f_t1s(s_xs);
                b_t2 <= b_t2 - f_t2s(s_xs);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 0 = the beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_u16 : unsigned(17 downto 0);
        variable v_t   : signed(8 downto 0);
        variable v_s   : signed(15 downto 0);
        variable v_var : std_logic;
        variable v_tl, v_sr : signed(16 downto 0);
        variable v_ix  : unsigned(7 downto 0);
        variable v_sd  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: profile address, rib phase (base - 5/4 u), texture phases, line-buffer address
            lut_ra <= r_bank & b_u(19 downto 12);
            p1_i  <= b_u(19 downto 12);
            p1_tk <= b_u(21 downto 20);
            v_u16 := resize(b_u(19 downto 4), 18) + shift_left(resize(b_u(19 downto 4), 18), 2);
            p1_rb <= l_rib - v_u16(17 downto 2);
            p1_t1 <= b_t1; p1_t2 <= b_t2;
            s_lb_ra <= b_px(10 downto 1);
            -- 2: triangles; the lower tilt's tile phase (clipped to +-0.2 tile)
            p2_tr <= f_tri10(p1_rb); p2_t1 <= f_tri10(p1_t1); p2_t2 <= f_tri10(p1_t2);
            v_t := signed(resize(p1_i, 9)) - 128;
            if v_t > 51 then p2_tc <= to_signed(51, 8);
            elsif v_t < -51 then p2_tc <= to_signed(-51, 8);
            else p2_tc <= v_t(7 downto 0); end if;
            p2_tk <= p1_tk;
            -- 3: profile to fabric; texture partial sums; source to fabric
            p3_p <= unsigned(lut_q(15 downto 4)); p3_w <= unsigned(lut_q(3 downto 0));
            p3_ts <= resize(p2_tr, 12) + resize(shift_right(p2_t1, 1), 12);
            p3_t2 <= resize(shift_right(p2_t2, 2), 12);
            p3_tc <= p2_tc; p3_tk <= p2_tk;
            p3_sv <= signed(s_lb_q);
            -- 4: products (lower tilt, source gain)
            p4_ts <= p3_ts + p3_t2;
            p4_p <= p3_p; p4_tk <= p3_tk;
            if p3_w = "0001" then p4_w <= "0000"; p4_rf <= '1'; else p4_w <= p3_w; p4_rf <= '0'; end if;
            p4_tll <= signed('0' & l_m8(3 downto 0)) * p3_tc; p4_tlh <= signed('0' & l_m8(7 downto 4)) * p3_tc;
            p4_srl <= signed('0' & c_g(3 downto 0)) * p3_sv; p4_srh <= signed('0' & c_g(7 downto 4)) * p3_sv;
            -- 5: crown texture x weight; tilt sign by the tile (1 falling, 3 rising); source scale (Osc 2x)
            p5_wt <= resize(signed('0' & p4_w) * p4_ts, 16);
            v_tl := resize(p4_tll, 17) + shift_left(resize(p4_tlh, 17), 4);
            case to_integer(p4_tk) is
                when 1      => p5_tl <= -resize(shift_right(v_tl, 7), 14);
                when 3      => p5_tl <= resize(shift_right(v_tl, 7), 14);
                when others => p5_tl <= (others => '0');
            end case;
            v_sr := resize(p4_srl, 17) + shift_left(resize(p4_srh, 17), 4);
            if s_src = '1' then p5_sr <= resize(shift_right(v_sr, 3), 14);
            else p5_sr <= resize(shift_right(v_sr, 4), 14); end if;
            p5_p <= p4_p; p5_rf <= p4_rf;
            -- 6, 7: the level
            p6_a <= signed(resize(p5_p, 15)) + resize(shift_right(p5_wt, 4), 15);
            p6_b <= resize(p5_tl, 15) + resize(p5_sr, 15);
            p6_rf <= p5_rf;
            v_s := resize(p6_a, 16) + resize(p6_b, 16);
            if v_s < 0 then p7_s <= (others => '0');
            elsif v_s > 4095 then p7_s <= (others => '1');
            else p7_s <= unsigned(v_s(11 downto 0)); end if;
            p7_rf <= p6_rf;
            -- 8: slope variant (rising / falling against the level 8 pixels back, by more than 3 lu; held on
            -- flats, so camera noise or a flat hem can't sparkle, and for the first pixels of the line); palette
            -- address
            v_var := p8_var;
            v_sd := signed(resize(p7_s, 13)) - signed(resize(unsigned(sd_f), 13));
            if s_gav_sr(6) = '0' then
                p_vc <= (others => '0');
                v_var := '1';
            else
                if p_vc /= 15 then p_vc <= p_vc + 1; end if;
                if p_vc >= 8 then
                    if v_sd > 3 then v_var := '0';
                    elsif v_sd < -3 then v_var := '1'; end if;
                end if;
            end if;
            p8_var <= v_var;
            -- the rim crosses the gap-arch band horizontally: step over it there (no thin arch rows)
            v_ix := p7_s(11 downto 4);
            if p7_rf = '1' and v_ix >= 73 and v_ix <= 78 then
                if v_ix <= 75 then v_ix := to_unsigned(72, 8); else v_ix := to_unsigned(79, 8); end if;
            end if;
            r_pal_a <= s_night & v_var & v_ix;
            -- 9: palette ROM; 10: fabric copy
            p10_q <= r_pal_q;
            -- 11: compose; black outside the generated avid
            if s_gav_sr(9) = '0' then
                s_y11 <= to_unsigned(64, 10); s_u11 <= to_unsigned(512, 10); s_v11 <= to_unsigned(512, 10);
            else
                s_y11 <= unsigned(p10_q(31 downto 22)); s_u11 <= unsigned(p10_q(21 downto 12)); s_v11 <= unsigned(p10_q(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(9);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: chroma through one [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums; the 8-back tap from a block-RAM ring), luma [1 2 1]/4 on
    -- taps aligned with the chroma centre (the HD HDMI bar fix), luma 13/16 and
    -- the blanking gate.  The syncs ride a block-RAM ring of C_LAT.
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rc <= rc + 1;
            -- chroma ring: written at rc + 6, read at rc, RAM register, fabric register = the input of 8 clocks
            -- before (never the write address)
            cr(to_integer(rc + 6)) <= std_logic_vector(s_u11) & std_logic_vector(s_v11);
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            -- sync ring: written at rc + C_LAT - 2, RAM register, fabric register = C_LAT clocks
            sr(to_integer(rc + (C_LAT - 2))) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            -- level ring for the slope variant: written at rc + 6, read 8 clocks later
            sdr(to_integer(rc + 6)) <= std_logic_vector(p7_s);
            sd_q <= sdr(to_integer(rc));
            sd_f <= sd_q;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
        end if;
    end process p_ring;

    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_l  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 12: chroma running 8-sums (snap back to neutral once 8 blank samples are in, so a glitched sample
            -- can't offset the colours for good); luma taps
            if s_bah = "00000000" then
                s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bau <= s_bau + s_u11 - unsigned(cr_f(19 downto 10));
                s_bav <= s_bav + s_v11 - unsigned(cr_f(9 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            s_ly(0) <= s_y11;
            for i in 1 to 5 loop s_ly(i) <= s_ly(i - 1); end loop;
            -- 13: chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4 (both centred 4 pixels back)
            v_sc := resize(s_bau, 14) + s_baud; s_u13 <= v_sc(13 downto 4);
            v_sc := resize(s_bav, 14) + s_bavd; s_v13 <= v_sc(13 downto 4);
            v_l := resize(s_ly(3), 12) + shift_left(resize(s_ly(4), 12), 1) + resize(s_ly(5), 12);
            s_y13 <= v_l(11 downto 2);
            -- 14: luma 13/16
            if s_y13 > 64 then v_dl := s_y13 - 64; else v_dl := (others => '0'); end if;
            s_y14 <= s_y13 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u14 <= s_u13; s_v14 <= s_v13;
            -- 15: pipeline
            s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14;
            -- 16: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;
            else
                s_y16 <= to_unsigned(64, 10); s_u16 <= to_unsigned(512, 10); s_v16 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= sr_f(2);
    data_out.vsync_n <= sr_f(1);
    data_out.field_n <= sr_f(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture marionette;
