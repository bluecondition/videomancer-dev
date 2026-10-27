-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Plume v0.5.0 -- Heirloom #19, rebuilt from the photo WIN_20230315_00_58_54.
--
-- The photo: a red plume pointing right (a key rising at xL ~ 410-520 and falling at a
-- chevron-shaped edge xF), folded top/bottom about the screen centre.  Outside it, horizontal
-- green stripes wobble over a vertical blue gradient with two black ribbons each side; inside,
-- the same stripes and gradient are cut by vertical "seams": a chirped x oscillator whose phase
-- per line draws the hooked chevrons.  B is a pulse of the seam phase, the G stripes are
-- sheared by its sawtooth (the yellow/pink ripples), and the outer band carries fine "<"
-- chevrons.  A second red ramp (the pink wedge) rises 820 px right of the fall.  The top and
-- bottom bands tear line by line.
--
-- The program: every per-line quantity is the fitted function of the folded line y' (plume.py:
-- one 256 x 240 line ROM, a word per two lines plus odd-line deltas).  A per-line engine (one serial
-- multiply-accumulate, 18 slots, ~370 clocks, computing the NEXT line) turns the line's quadratic seam
-- phase into a second-order forward-difference DDA (Q32), scales it by Seams, places the line's tear,
-- starts the wiggle-phase accumulators and derives the stripe phases.  The line ROM's output register
-- holds the next line's constants; they are copied to the line registers at the line start.  The pixel
-- path runs the DDA, reads the pulse / saw / cos ROMs, builds the stripe phases (parabolic wiggle sines,
-- shear, video bend), applies the channel laws, the plume window, the red key with its teeth, and
-- RGB -> YUV (luma pre-compensated 16/13 for the output scaling).
--
--   K1  Seams    seam count across the plume (x0.5 .. x2; centre = the photo)
--   K2  Reach    plume length: how far the red chevron reaches right (x0.4 .. x1.6)
--   K3  Stripes  green stripe density (x0.5 .. x2)
--   K4  Flow     bipolar, still at the centre +-8: the seams stream along the plume (default slow)
--   K5  Bend     how far the video (or Osc) bends the seams and the stripes
--   K6  Tear     line tearing in the top / bottom bands (0 = clean lines)
--   S7  Twin     a second plume mirrored from the right edge
--   S8  Wedge    the pink wedge on the right: On (the photo) / Off
--   S9  Ribbons  the black ribbons: Dark (the photo) / Filled with stripes
--   S10 Palette  Photo / Swap (red and blue exchanged)
--   S11 Source   Video / Osc
--   P12 WAKE     chevron depth: 0 = the stripes run straight, 50 % = the photo, 100 % = deep hooks
--
-- Virtual 1080-line geometry (SD shows the central 1440 virtual px).  Generated avid; output
-- conditioning (luma box-4 + [1 2 1], chroma 8-box pair, luma to ~777) before the blanking gate.
-- Latency 24 clocks in every mode.  Generated colours in the hardware convention (U = Cr, V = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.plume_pkg.all;
use work.all;

architecture plume of program_top is
    constant C_LAT : integer := 24;   -- matches processing_delay_clks

    -- virtual lines per line (Q8), by timing ID
    function f_ys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 11);
            when 4          => return to_unsigned(576, 11);
            when 8          => return to_unsigned(960, 11);
            when 12         => return to_unsigned(480, 11);
            when 1 | 2 | 10 => return to_unsigned(512, 11);
            when 5 | 6 | 14 => return to_unsigned(384, 11);
            when others     => return to_unsigned(256, 11);
        end case;
    end function;
    -- virtual px per px, Q2 (k = 4 / 6 / 8)
    function f_k(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(8, 4);
            when 5 | 6 | 14     => return to_unsigned(6, 4);
            when others         => return to_unsigned(4, 4);
        end case;
    end function;
    -- Osc spatial steps (Q16 cycles per pixel / per line)
    function f_kax(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(356, 11);
            when 5 | 6 | 14     => return to_unsigned(268, 11);
            when others         => return to_unsigned(176, 11);
        end case;
    end function;
    function f_kay(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1080, 11);
            when 4          => return to_unsigned(548, 11);
            when 8          => return to_unsigned(912, 11);
            when 12         => return to_unsigned(456, 11);
            when 5 | 6 | 14 => return to_unsigned(364, 11);
            when 1 | 2 | 10 => return to_unsigned(484, 11);
            when others     => return to_unsigned(244, 11);
        end case;
    end function;
    -- 8-bit triangle -> signed -127..127
    function f_tri(p : unsigned(15 downto 0)) return signed is
        variable v : unsigned(15 downto 0);
    begin
        if p(15) = '1' then v := p - 32768; else v := 32768 - p; end if;
        return to_signed(127, 9) - signed(resize(v(15 downto 7), 9));
    end function;
    -- knob curve 0..1023 -> Q8 0.5 .. 1 .. 2 (piecewise linear)
    function f_x2(g : unsigned(9 downto 0)) return unsigned is
    begin
        if g(9) = '0' then return to_unsigned(128, 10) + resize(g(8 downto 2), 10);
        else return to_unsigned(256, 10) + resize(g(8 downto 1), 10); end if;
    end function;
    -- clamp a signed value to 0..255
    function f_c8(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else return unsigned(v(7 downto 0)); end if;
    end function;

    -- timing
    signal t_ys  : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal t_k   : unsigned(3 downto 0) := to_unsigned(4, 4);
    signal t_k128 : unsigned(10 downto 0) := (others => '0');         -- 128 k
    signal t_kk   : unsigned(6 downto 0) := (others => '0');          -- k^2
    signal t_s1k, t_s2k : unsigned(11 downto 0) := (others => '0');   -- wiggle phase steps 262 k, 160 k
    signal t_kax, t_kay : unsigned(10 downto 0) := (others => '0');

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- glides: K1, K2, K3, K5, K6, P12 (Q.3), power up at the defaults
    type t_g is array(0 to 4) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13), to_unsigned(512 * 8, 13),
                         to_unsigned(120 * 8, 13), to_unsigned(512 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');

    -- per-field values
    signal f_sig, f_st : unsigned(9 downto 0) := to_unsigned(256, 10);    -- Q8
    signal f_lam  : unsigned(8 downto 0) := to_unsigned(256, 9);          -- Q8
    signal f_wake : unsigned(9 downto 0) := to_unsigned(256, 10);         -- Q8, 0 .. 3
    signal f_b5   : unsigned(4 downto 0) := to_unsigned(3, 5);
    signal f_tg   : unsigned(5 downto 0) := to_unsigned(32, 6);
    signal f_tk   : unsigned(3 downto 0) := to_unsigned(3, 4);            -- teeth depth (x 1/8 px per pulse unit)
    signal f_x0   : unsigned(13 downto 0) := (others => '0');             -- virtual x of pixel 0, Q2
    signal f_fm   : unsigned(8 downto 0) := (others => '0');
    signal f_fneg : std_logic := '0';
    signal f_spd  : unsigned(12 downto 0) := (others => '0');             -- Q16 cycles per field
    signal f_flow, f_gflow : unsigned(15 downto 0) := (others => '0');
    signal f_twin, f_nowedge, f_fill, f_swap, f_osc : std_logic := '0';
    signal f_ta, f_tb : unsigned(15 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start, g_start_d : std_logic := '0';

    -- per-line engine
    signal e_cnt  : unsigned(8 downto 0) := (others => '1');
    signal e_yv   : unsigned(19 downto 0) := (others => '0');             -- Q8 virtual line
    signal e_ypq  : unsigned(19 downto 0) := (others => '0');
    signal e_yp   : unsigned(9 downto 0) := (others => '0');
    signal e_lfsr : unsigned(31 downto 0) := x"01234567";
    signal e_lc   : unsigned(1 downto 0) := (others => '0');
    signal e_hold : signed(1 downto 0) := (others => '0');
    signal ln_a   : unsigned(7 downto 0) := (others => '0');
    signal ln_q   : std_logic_vector(239 downto 0) := (others => '0');
    signal e_odd  : std_logic := '0';
    signal e_a0   : unsigned(9 downto 0) := (others => '0');
    signal e_pg   : unsigned(15 downto 0) := (others => '0');
    -- multiply-accumulate slots: P = PRE + A * B (B two's complement, 15 bits)
    signal e_sl   : unsigned(4 downto 0) := (others => '0');
    signal e_ph   : unsigned(4 downto 0) := (others => '0');
    signal e_run  : std_logic := '0';
    signal m_a    : signed(41 downto 0) := (others => '0');
    signal m_b    : signed(14 downto 0) := (others => '0');
    signal m_acc  : signed(41 downto 0) := (others => '0');
    -- engine scratch
    signal e_tri  : signed(9 downto 0) := (others => '0');
    signal e_xs   : signed(14 downto 0) := (others => '0');                -- Q2 virtual x of pixel 0
    signal e_s    : signed(13 downto 0) := (others => '0');                -- s0, Q12
    signal e_s2h  : signed(14 downto 0) := (others => '0');                -- 2 s0 + h, Q12
    signal e_r1p, e_r2p : signed(22 downto 0) := (others => '0');          -- a1 Q14, a2 Q13
    signal e_spd  : unsigned(12 downto 0) := (others => '0');
    signal e_kx   : unsigned(10 downto 0) := (others => '0');              -- xF - xL
    -- next-line registers (written by the engine)
    signal n_th0, n_d1 : unsigned(31 downto 0) := (others => '0');
    signal n_d2   : signed(23 downto 0) := (others => '0');
    signal n_xs   : signed(14 downto 0) := (others => '0');
    signal n_xfp  : unsigned(11 downto 0) := (others => '0');
    signal n_pgs, n_dlt : unsigned(15 downto 0) := (others => '0');
    signal n_sgw6 : unsigned(5 downto 0) := (others => '0');
    signal n_p1, n_p2 : unsigned(17 downto 0) := (others => '0');
    -- current-line registers (pixel path)
    signal l_d2   : signed(23 downto 0) := (others => '0');
    signal l_x1   : unsigned(12 downto 0) := (others => '1');
    signal l_wb   : unsigned(2 downto 0) := (others => '0');
    signal l_mb, l_go : unsigned(7 downto 0) := (others => '0');
    signal l_ab6, l_gk6 : unsigned(5 downto 0) := (others => '0');
    signal l_xl   : unsigned(10 downto 0) := (others => '0');
    signal l_xfp  : unsigned(11 downto 0) := (others => '0');
    signal l_bb   : unsigned(7 downto 0) := (others => '0');
    signal l_bk3, l_wa3 : unsigned(2 downto 0) := (others => '0');
    signal l_pgs, l_dlt : unsigned(15 downto 0) := (others => '0');
    signal l_e1   : unsigned(7 downto 0) := (others => '0');
    signal l_e26  : unsigned(6 downto 0) := (others => '0');
    signal l_sgw6 : unsigned(5 downto 0) := (others => '0');
    signal l_rl   : unsigned(7 downto 0) := (others => '0');
    signal l_pb   : unsigned(15 downto 0) := (others => '0');

    -- beam (stage 0)
    signal b_xq   : signed(14 downto 0) := (others => '0');
    signal b_th, b_d1 : unsigned(31 downto 0) := (others => '0');
    signal b_p1, b_p2 : unsigned(17 downto 0) := (others => '0');
    signal b_frz  : std_logic := '0';
    signal b_pa   : unsigned(15 downto 0) := (others => '0');

    -- video tap
    type t_u8v is array (natural range <>) of unsigned(7 downto 0);
    signal s_vy : t_u8v(1 to 2) := (others => to_unsigned(128, 8));
    signal s_vs : signed(9 downto 0) := (others => '0');

    -- pixel path (stage N = N registers after the beam)
    signal q0_src : signed(9 downto 0) := (others => '0');
    signal q1_vm  : signed(14 downto 0) := (others => '0');
    signal q1_xi  : signed(12 downto 0) := (others => '0');
    signal q1_th  : unsigned(9 downto 0) := (others => '0');
    signal q1_v1, q1_v2 : unsigned(14 downto 0) := (others => '0');
    signal q1_n1, q1_n2 : std_logic := '0';
    signal q2_thf : unsigned(9 downto 0) := (others => '0');
    signal q2_vg, q3_vg, q4_vg : signed(10 downto 0) := (others => '0');
    signal q2_s1, q2_s2 : signed(7 downto 0) := (others => '0');
    signal q2_xw, q3_xw : signed(12 downto 0) := (others => '0');
    signal pw_a   : unsigned(9 downto 0) := (others => '0');
    signal sc_a   : unsigned(7 downto 0) := (others => '0');
    signal pw_q   : std_logic_vector(7 downto 0) := (others => '0');
    signal sc_q   : std_logic_vector(15 downto 0) := (others => '0');
    signal q3_wsum : signed(9 downto 0) := (others => '0');
    signal q4_wv  : signed(12 downto 0) := (others => '0');
    signal q4_d, q4_e : signed(13 downto 0) := (others => '0');
    signal q5_gw, q6_gw : unsigned(15 downto 0) := (others => '0');
    signal q5_pw, q6_pw : signed(7 downto 0) := (others => '0');
    signal q5_saw : unsigned(7 downto 0) := (others => '0');
    signal q5_cg  : signed(7 downto 0) := (others => '0');
    signal q5_el, q5_er, q5_e2 : unsigned(7 downto 0) := (others => '0');
    signal q5_v8, q6_v8, q7_v8, q8_v8, q9_v8 : unsigned(7 downto 0) := (others => '0');
    signal q5_sd, q6_sd, q7_sd, q8_sd, q9_sd : std_logic := '0';
    signal q5_fd, q6_fd : signed(13 downto 0) := (others => '0');
    signal q6_sw  : unsigned(13 downto 0) := (others => '0');
    signal q6_tth : signed(11 downto 0) := (others => '0');
    signal q6_ab  : signed(12 downto 0) := (others => '0');
    signal q6_et4, q7_et4, q8_et4, q9_et4, q10_et4, q11_et4, q12_et4 : unsigned(3 downto 0) := (others => '0');
    signal q7_bp, q8_bp, q9_bp, q10_bp : unsigned(7 downto 0) := (others => '0');
    signal q7_fd  : signed(13 downto 0) := (others => '0');
    signal cs1_a, cs2_a : unsigned(7 downto 0) := (others => '0');
    signal cs1_q, cs2_q : std_logic_vector(7 downto 0) := (others => '0');
    signal q8_frp : signed(19 downto 0) := (others => '0');
    signal q8_cgg : signed(7 downto 0) := (others => '0');
    signal q9_fr  : unsigned(7 downto 0) := (others => '0');
    signal q9_cgp : signed(7 downto 0) := (others => '0');
    signal q9_e2p : unsigned(13 downto 0) := (others => '0');
    signal q9_bk  : signed(11 downto 0) := (others => '0');
    signal q10_r, q11_r, q12_r, q13_r : unsigned(7 downto 0) := (others => '0');
    signal q10_gk : signed(13 downto 0) := (others => '0');
    signal q10_gg, q11_gg, q12_gg, q13_gg : unsigned(7 downto 0) := (others => '0');
    signal q10_bg, q11_bg, q12_bg : unsigned(7 downto 0) := (others => '0');
    signal q11_gp : unsigned(7 downto 0) := (others => '0');
    signal q11_db : signed(8 downto 0) := (others => '0');
    signal q12_dg : signed(8 downto 0) := (others => '0');
    signal q12_pb : signed(12 downto 0) := (others => '0');
    signal q13_pg : signed(12 downto 0) := (others => '0');
    signal q13_b  : unsigned(7 downto 0) := (others => '0');
    signal q14_r, q14_g, q14_b : unsigned(7 downto 0) := (others => '0');
    signal q15_yrg, q15_ugb, q15_vrg : unsigned(16 downto 0) := (others => '0');
    signal q15_yb, q15_ur, q15_vb : unsigned(15 downto 0) := (others => '0');
    signal q16_y  : unsigned(16 downto 0) := (others => '0');
    signal q16_u, q16_v : signed(17 downto 0) := (others => '0');
    signal q17_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q17_u, q17_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- ROM / RAM storage
    type t_ring is array (0 to 255) of std_logic_vector(29 downto 0);
    signal rb_ram : t_ring := (others => "0000000000" & "1000000000" & "1000000000");
    type t_sring is array (0 to 255) of std_logic_vector(2 downto 0);
    signal rs_ram : t_sring := (others => "111");
    attribute no_rw_check : boolean;
    attribute no_rw_check of rb_ram : signal is true;
    attribute no_rw_check of rs_ram : signal is true;
    signal s_wp, rbr_a, rs_ra : unsigned(7 downto 0) := (others => '0');
    signal rb_wd : std_logic_vector(29 downto 0) := "0000000000" & "1000000000" & "1000000000";
    signal rbr_q, rbr_q2 : std_logic_vector(29 downto 0) := "0000000000" & "1000000000" & "1000000000";
    signal rs_q, rs_q2 : std_logic_vector(2 downto 0) := "111";
    signal s_ysum, s_usum, s_vsum, s_ud, s_vd : unsigned(13 downto 0) := (others => '0');
    type t_yd is array (0 to 3) of std_logic_vector(9 downto 0);
    signal s_yd : t_yd := (others => (others => '0'));
    signal q19_y, q20_y, q21_y, q21_yf : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q20_u, q20_v, q21_u, q21_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal q23_y, q24_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q22_u, q22_v, q23_u, q23_v, q24_u, q24_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs
    ---------------------------------------------------------------------------
    p_rowrom : process(clk)
    begin
        if rising_edge(clk) then
            ln_q <= C_LN(to_integer(ln_a));
        end if;
    end process p_rowrom;

    p_pixrom : process(clk)
    begin
        if rising_edge(clk) then
            pw_q  <= C_PW(to_integer(pw_a));
            sc_q  <= C_SC(to_integer(sc_a));
            cs1_q <= C_CS(to_integer(cs1_a));
            cs2_q <= C_CS(to_integer(cs2_a));
        end if;
    end process p_pixrom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, per-field values
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_x  : signed(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_g  : unsigned(9 downto 0);
        variable v_t  : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            t_ys  <= f_ys(registers_in(8)(3 downto 0));
            t_k   <= f_k(registers_in(8)(3 downto 0));
            case to_integer(t_k) is
                when 8 => t_k128 <= to_unsigned(1024, 11); t_kk <= to_unsigned(64, 7); t_s1k <= to_unsigned(2096, 12); t_s2k <= to_unsigned(1280, 12);
                when 6 => t_k128 <= to_unsigned(768, 11);  t_kk <= to_unsigned(36, 7); t_s1k <= to_unsigned(1572, 12); t_s2k <= to_unsigned(960, 12);
                when others => t_k128 <= to_unsigned(512, 11); t_kk <= to_unsigned(16, 7); t_s1k <= to_unsigned(1048, 12); t_s2k <= to_unsigned(640, 12);
            end case;
            t_kax <= f_kax(registers_in(8)(3 downto 0));
            t_kay <= f_kay(registers_in(8)(3 downto 0));

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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                -- K4 Flow: bipolar, deadband 8
                v_x := signed(resize(unsigned(registers_in(3)), 11)) - 512;
                if v_x > 8 then f_fneg <= '0'; v_u := unsigned(v_x) - 8; f_fm <= v_u(8 downto 0);
                elsif v_x < -8 then f_fneg <= '1'; v_u := unsigned(-v_x) - 8; f_fm <= v_u(8 downto 0);
                else f_fneg <= '0'; f_fm <= (others => '0'); end if;
                f_twin   <= registers_in(6)(0);
                f_nowedge <= registers_in(6)(1);
                f_fill   <= registers_in(6)(2);
                f_swap   <= registers_in(6)(3);
                f_osc    <= registers_in(6)(4);
                f_tg     <= unsigned(registers_in(5)(9 downto 4));
                -- phases: + flow streams the seams right (phase down); the stripes drift at 1/4
                if f_fneg = '1' then
                    f_flow <= f_flow + resize(f_spd, 16); f_gflow <= f_gflow + resize(f_spd(12 downto 2), 16);
                else
                    f_flow <= f_flow - resize(f_spd, 16); f_gflow <= f_gflow - resize(f_spd(12 downto 2), 16);
                end if;
                f_ta <= f_ta + 151; f_tb <= f_tb - 113;
            end if;

            f_spd <= e_spd;

            -- glides: ease 1/8 of the gap per field (deadband 4); one channel per four zone clocks (32..55)
            if s_zone = '1' and s_zc >= 32 and s_zc < 52 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2); q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(2)) & "000"; gl_o <= s_g(2);
                            when 3 => gl_k <= unsigned(registers_in(4)) & "000"; gl_o <= s_g(3);
                            when others => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(4);
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
                                when 2 => s_g(2) <= gl_n;
                                when 3 => s_g(3) <= gl_n;
                                when others => s_g(4) <= gl_n;
                            end case;
                        end if;
                end case;
            end if;

            -- knob curves (settle long before the line engine reads them)
            f_sig <= f_x2(s_g(0)(12 downto 3));
            v_g := s_g(1)(12 downto 3);
            f_lam <= to_unsigned(96, 9) + resize(v_g(9 downto 2), 9) + resize(v_g(9 downto 4), 9);
            f_st  <= f_x2(s_g(2)(12 downto 3));
            f_b5 <= s_g(3)(12 downto 8);
            v_g := s_g(4)(12 downto 3);
            if v_g(9) = '0' then f_wake <= resize(v_g(8 downto 1), 10);
            else f_wake <= to_unsigned(256, 10) + resize(v_g(8 downto 0), 10); end if;
            f_tk <= resize(f_wake(9 downto 7), 4);
            -- virtual x of pixel 0 (Q2): 0 for 1920 x 4 / 1280 x 6; SD: 3840 - 720 x 4 = 960 (240 px: the central 1440)
            if t_k = 8 then f_x0 <= to_unsigned(960, 14); else f_x0 <= (others => '0'); end if;

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
            g_start_d <= g_start;
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- Per-line engine: kicked one clock after the line's g_start (line 0: in the
    -- vblank zone), computes the NEXT line into the n_* registers.
    --   1..3    y' and ROM / knot addresses, LFSR step
    --   4..37   knot interpolation (14 functions, one every 2 clocks), row ROM fields
    --   48..299 14 multiply slots of 18 clocks: 0 load, 1..14 shift-add, 15 store, 16..17 derived
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_y   : signed(20 downto 0);
        variable v_c   : unsigned(8 downto 0);
        variable v_f   : unsigned(4 downto 0);
        variable v_kp  : signed(22 downto 0);
        variable v_x   : unsigned(31 downto 0);
        variable v_k6  : unsigned(9 downto 0);
        variable v_a   : signed(25 downto 0);
        variable v_b   : signed(14 downto 0);
        variable v_pre : signed(41 downto 0);
    begin
        if rising_edge(clk) then
            -- kick
            if s_zone = '1' and s_zc = 200 then
                e_yv <= resize(t_ys(10 downto 1), 20) - 128;
                e_lc <= (others => '0');
                e_cnt <= to_unsigned(1, 9);
            elsif g_start_d = '1' then
                e_yv <= e_yv + resize(t_ys, 20);
                if e_lc = 2 then e_lc <= (others => '0'); else e_lc <= e_lc + 1; end if;
                e_cnt <= to_unsigned(1, 9);
            elsif e_cnt /= 511 then
                e_cnt <= e_cnt + 1;
            end if;
            v_c := e_cnt;

            -- 1..3: y' (folded at 520), addresses, LFSR
            if v_c = 1 then
                v_y := signed(resize(e_yv, 21)) - 132992;
                if v_y < 0 then e_ypq <= unsigned(resize(-v_y, 20)); else e_ypq <= unsigned(resize(v_y, 20)); end if;
            end if;
            if v_c = 2 then
                if e_ypq(19 downto 18) /= "00" then e_yp <= to_unsigned(520, 10);
                elsif e_ypq(17 downto 8) > 520 then v_k6 := resize(to_unsigned(1040, 11) - resize(e_ypq(17 downto 8), 11), 10); e_yp <= v_k6;
                else e_yp <= e_ypq(17 downto 8); end if;
            end if;
            if v_c = 3 then
                if e_yp(9) = '1' then ln_a <= (others => '1'); else ln_a <= e_yp(8 downto 1); end if;
                e_odd <= e_yp(0);
                v_x := e_lfsr;
                v_x := v_x xor shift_left(v_x, 13);
                v_x := v_x xor shift_right(v_x, 17);
                v_x := v_x xor shift_left(v_x, 5);
                e_lfsr <= v_x;
                if e_lc = 0 then
                    case to_integer(v_x(17 downto 16)) is
                        when 0 => e_hold <= to_signed(-1, 2);
                        when 2 => e_hold <= to_signed(1, 2);
                        when others => e_hold <= to_signed(0, 2);
                    end case;
                end if;
            end if;
            if v_c = 4 then
                e_tri <= signed(resize(e_lfsr(7 downto 0), 10)) + signed(resize(e_lfsr(15 downto 8), 10)) - 255;
            end if;

            -- 7: line-ROM fields that feed the multiply slots (address at 3, data at 5)
            if v_c = 7 then
                if e_odd = '1' then
                    e_a0 <= unsigned(ln_q(F_A0O_H downto F_A0O_L));
                    e_kx <= unsigned(signed(ln_q(F_KX_H downto F_KX_L)) + resize(signed(ln_q(F_DKX_H downto F_DKX_L)), 11));
                    e_pg <= unsigned(ln_q(F_PG_H downto F_PG_L)) + resize(unsigned(ln_q(F_DPG_H downto F_DPG_L)), 16);
                else
                    e_a0 <= unsigned(ln_q(F_A0E_H downto F_A0E_L));
                    e_kx <= unsigned(ln_q(F_KX_H downto F_KX_L));
                    e_pg <= unsigned(ln_q(F_PG_H downto F_PG_L));
                end if;
            end if;

            -- 48 ..: 18 MAC slots of 18 clocks (a slot may take the previous slot's product as its A or PRE): 0 load, 1..15 bits of B (15 = sign: subtract), 16 P, 17 store
            if v_c = 48 then
                e_run <= '1'; e_sl <= (others => '0'); e_ph <= (others => '0');
            elsif e_run = '1' then
                if e_ph = 17 then
                    e_ph <= (others => '0');
                    if e_sl = 17 then e_run <= '0'; else e_sl <= e_sl + 1; end if;
                else
                    e_ph <= e_ph + 1;
                end if;
            end if;
            if e_run = '1' and e_ph = 0 then
                v_a := (others => '0'); v_b := (others => '0'); v_pre := (others => '0');
                case to_integer(e_sl) is
                    when 0  => v_a := resize(e_tri, 26);  v_b := signed(resize(f_tg, 15));
                               if e_hold = 1 then v_pre := shift_left(resize(signed('0' & f_tg(5 downto 2)), 42), 8);
                               elsif e_hold = -1 then v_pre := -shift_left(resize(signed('0' & f_tg(5 downto 2)), 42), 8); end if;
                    when 1  => v_a := resize(m_acc(17 downto 8), 26); v_b := signed(resize(unsigned(ln_q(F_TZ_H downto F_TZ_L)), 15));
                    when 2  => v_a := resize(signed(ln_q(F_A1_H downto F_A1_L)), 26); v_b := signed(resize(f_sig, 15));
                    when 3  => v_a := resize(signed(ln_q(F_A2_H downto F_A2_L)), 26); v_b := signed(resize(f_sig, 15));
                    when 4  => v_a := resize(e_r2p, 26);  v_b := resize(e_s, 15);   v_pre := shift_left(resize(e_r1p, 42), 11);
                    when 5  => v_a := m_acc(37 downto 12);  v_b := resize(e_s, 15);
                               v_pre := signed(resize(e_a0 & "000000000000000", 42)) + signed(resize(f_flow & "000000000", 42));
                    when 6  => v_a := resize(e_r2p, 26);  v_b := e_s2h;             v_pre := shift_left(resize(e_r1p, 42), 11);
                    when 7  => v_a := m_acc(37 downto 12);  v_b := signed(resize(t_k128, 15));
                    when 8  => v_a := resize(e_r2p, 26);  v_b := signed(resize(t_kk, 15));
                    when 9  => v_a := signed(resize(e_kx, 26)); v_b := signed(resize(f_lam, 15));
                               v_pre := signed(resize(unsigned(ln_q(F_XL_H downto F_XL_L)) & x"00", 42));
                    when 10 => v_a := signed(resize(e_pg, 26)); v_b := signed(resize(f_st, 15));
                               v_pre := signed(resize(f_gflow & x"0", 42));
                    when 11 => v_a := signed(resize(unsigned(ln_q(F_DLT_H downto F_DLT_L)), 26)); v_b := signed(resize(f_st, 15));
                    when 12 => v_a := signed(resize(unsigned(ln_q(F_SG_H downto F_SG_L)), 26)); v_b := signed(resize(f_wake, 15));
                    when 13 => v_a := signed(resize(e_yp, 26)); v_b := to_signed(164, 15);
                    when 14 => v_a := resize(e_xs, 26);   v_b := to_signed(262, 15);  v_pre := shift_left(m_acc, 2);
                    when 15 => v_a := signed(resize(e_yp, 26)); v_b := to_signed(-385, 15); v_pre := to_signed(11469, 42);
                    when 16 => v_a := resize(e_xs, 26);   v_b := to_signed(160, 15);  v_pre := shift_left(m_acc, 2);
                    when others => v_a := signed(resize(f_fm, 26)); v_b := signed(resize(f_fm, 15));
                end case;
                m_a <= resize(v_a, 42); m_b <= v_b; m_acc <= v_pre;
            elsif e_run = '1' and e_ph >= 1 and e_ph <= 15 then
                if m_b(0) = '1' then
                    if e_ph = 15 then m_acc <= m_acc - m_a; else m_acc <= m_acc + m_a; end if;
                end if;
                m_a <= shift_left(m_a, 1); m_b <= shift_right(m_b, 1);
            end if;
            -- stores (one destination per slot)
            if e_run = '1' and e_ph = 17 then
                case to_integer(e_sl) is
                    when 1  => e_xs <= signed(resize(f_x0, 15)) + shift_left(resize(m_acc(17 downto 8), 15), 2);
                    when 2  => e_r1p <= m_acc(22 downto 0); e_s <= resize(e_xs - 4000, 14);
                    when 3  => e_r2p <= m_acc(22 downto 0); e_s2h <= shift_left(resize(e_s, 15), 1) + signed(resize(t_k, 15));
                    when 5  => n_th0 <= unsigned(m_acc(24 downto 0)) & "0000000";
                    when 7  => n_d1 <= unsigned(m_acc(31 downto 0));
                    when 8  => n_d2 <= m_acc(27 downto 4);
                    when 9  => n_xfp <= unsigned(m_acc(19 downto 8));
                    when 10 => n_pgs <= unsigned(m_acc(19 downto 4));
                    when 11 => n_dlt <= unsigned(m_acc(19 downto 4));
                    when 12 => n_sgw6 <= unsigned(m_acc(15 downto 10));
                    when 14 => n_p1 <= unsigned(m_acc(17 downto 0));
                    when 16 => n_p2 <= unsigned(m_acc(17 downto 0));
                    when 17 => e_spd <= unsigned(m_acc(17 downto 5)); n_xs <= e_xs;
                    when others => null;
                end case;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- Beam: x (virtual, Q2), the seam-phase DDA, Osc phases; line registers
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_bb, v_e1 : unsigned(7 downto 0);
        variable v_e26 : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            if s_zgo = '1' then
                l_pb <= f_tb;
            elsif data_in.avid = '1' and s_avq = '0' then
                l_pb <= l_pb + resize(t_kay, 16);
            end if;
            if g_start = '1' then
                b_xq <= n_xs; b_th <= n_th0; b_d1 <= n_d1; b_frz <= '0'; b_p1 <= n_p1; b_p2 <= n_p2;
                b_pa <= f_ta;
                l_d2 <= n_d2; l_x1 <= unsigned(ln_q(F_XQ_H downto F_XQ_L)) & "0000";
                l_wb <= unsigned(ln_q(F_WB_H downto F_WB_L)); l_mb <= unsigned(ln_q(F_MB_H downto F_MB_L));
                l_ab6 <= unsigned(ln_q(F_AB6_H downto F_AB6_L)); l_go <= unsigned(ln_q(F_GO_H downto F_GO_L));
                l_gk6 <= unsigned(ln_q(F_GK6_H downto F_GK6_L));
                l_xl <= unsigned(ln_q(F_XL_H downto F_XL_L)); l_xfp <= n_xfp;
                l_bk3 <= unsigned(ln_q(F_BK3_H downto F_BK3_L)); l_pgs <= n_pgs; l_dlt <= n_dlt;
                l_wa3 <= resize(unsigned(ln_q(F_WA3M_H downto F_WA3M_L)), 3) + 1; l_sgw6 <= n_sgw6;
                l_rl <= not resize(unsigned(ln_q(F_RL5_H downto F_RL5_L)) & '0', 8);
                if e_odd = '1' then
                    v_bb := unsigned(signed(ln_q(F_BB_H downto F_BB_L)) + resize(signed(ln_q(F_DBB_H downto F_DBB_L)), 8));
                    v_e1 := unsigned(signed(ln_q(F_E1_H downto F_E1_L)) + resize(signed(ln_q(F_DE1_H downto F_DE1_L)), 8));
                    v_e26 := unsigned(signed(ln_q(F_E26_H downto F_E26_L)) + resize(signed(ln_q(F_DE26_H downto F_DE26_L)), 7));
                else
                    v_bb := unsigned(ln_q(F_BB_H downto F_BB_L));
                    v_e1 := unsigned(ln_q(F_E1_H downto F_E1_L));
                    v_e26 := unsigned(ln_q(F_E26_H downto F_E26_L));
                end if;
                l_bb <= v_bb;
                if f_fill = '1' then
                    l_e1 <= to_unsigned(231, 8);
                    if v_e1(7) = '0' then l_e26 <= to_unsigned(57, 7); else l_e26 <= v_e26; end if;
                else
                    l_e1 <= v_e1; l_e26 <= v_e26;
                end if;
            elsif g_avid = '1' then
                b_xq <= b_xq + signed(resize(t_k, 15));
                b_th <= b_th + b_d1;
                if b_frz = '1' or (b_xq >= 0 and unsigned(b_xq(12 downto 0)) >= l_x1) then
                    b_frz <= '1';
                else
                    b_d1 <= b_d1 + unsigned(resize(l_d2, 32));
                end if;
                b_pa <= b_pa + resize(t_kax, 16);
                b_p1 <= b_p1 + resize(t_s1k, 18); b_p2 <= b_p2 + resize(t_s2k, 18);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Rings (block RAM): the box window {Y, U, V} and the sync delay
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rb_ram(to_integer(s_wp)) <= rb_wd;
            rbr_q <= rb_ram(to_integer(rbr_a));
            rbr_q2 <= rbr_q;
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
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_s11 : signed(11 downto 0);
        variable v_e9  : unsigned(8 downto 0);
        variable v_k   : unsigned(7 downto 0);
        variable v_c9  : signed(8 downto 0);
        variable v_r20 : signed(19 downto 0);
        variable v_u16 : unsigned(15 downto 0);
        variable v_y   : unsigned(16 downto 0);
        variable v_cs  : signed(17 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_dl  : unsigned(9 downto 0);
        variable v_r, v_gg, v_b : unsigned(15 downto 0);
        variable v_iy, v_iu, v_iv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- video tap: 2-px luma average -> signed (Y - 512) / 2 (a few px early: invisible in a bend)
            s_vy(1) <= unsigned(data_in.y(9 downto 2)); s_vy(2) <= s_vy(1);
            s_vs <= signed(resize(resize(s_vy(1), 9) + resize(s_vy(2), 9), 10)) - 256;

            -- 0: source (Osc: an x triangle phase-modulated by a y triangle; skew of a few px is invisible)
            if f_osc = '1' then q0_src <= resize(f_tri(b_pa), 10) + resize(f_tri(l_pb), 10);
            else q0_src <= s_vs; end if;

            -- 1: bend product; x; theta; wiggle parabolas (sin ~ 4 h (128 - h))
            q1_vm <= resize(q0_src * signed('0' & f_b5), 15);
            q1_xi <= b_xq(14 downto 2);
            q1_th <= b_th(31 downto 22);
            q1_v1 <= b_p1(16 downto 10) * (to_unsigned(128, 8) - resize(b_p1(16 downto 10), 8)); q1_n1 <= b_p1(17);
            q1_v2 <= b_p2(16 downto 10) * (to_unsigned(128, 8) - resize(b_p2(16 downto 10), 8)); q1_n2 <= b_p2(17);
            -- 2: theta + bend; sines; Twin fold
            q2_thf <= q1_th + unsigned(resize(shift_right(q1_vm, 2), 10));
            q2_vg <= q1_vm(10 downto 0);
            if q1_v1(14 downto 5) > 127 then v_k := to_unsigned(127, 8); else v_k := q1_v1(12 downto 5); end if;
            if q1_n1 = '1' then q2_s1 <= -signed(v_k); else q2_s1 <= signed(v_k); end if;
            if q1_v2(14 downto 5) > 127 then v_k := to_unsigned(127, 8); else v_k := q1_v2(12 downto 5); end if;
            if q1_n2 = '1' then q2_s2 <= -signed(v_k); else q2_s2 <= signed(v_k); end if;
            if f_twin = '1' and q1_xi > 960 then q2_xw <= 1920 - q1_xi; else q2_xw <= q1_xi; end if;
            -- 3: seam ROM addresses; wiggle sum
            pw_a <= l_wb & q2_thf(9 downto 3);
            sc_a <= q2_thf(9 downto 2);
            v_s11 := shift_left(resize(q2_s2, 12), 3) + shift_left(resize(q2_s2, 12), 1) + resize(q2_s2, 12);   -- x11
            q3_wsum <= resize(q2_s1, 10) + resize(shift_right(v_s11, 4), 10);
            q3_vg <= q2_vg; q3_xw <= q2_xw;
            -- 4: (ROM read) wiggle amplitude; the two distances every edge is measured from
            q4_wv <= resize(q3_wsum * signed('0' & l_wa3), 13);
            q4_vg <= q3_vg;
            q4_d <= resize(q3_xw, 14) - signed(resize(l_xl, 14));        -- x - xL
            q4_e <= signed(resize(l_xfp, 14)) - resize(q3_xw, 14);       -- xF' - x
            -- 5: ground stripe phase; fabric copies; ramps (window, red rise / wedge, fall distance)
            q5_gw <= l_pgs + unsigned(resize(q4_wv, 16) sll 6) + unsigned(resize(q4_vg, 16) sll 5);
            q5_pw <= signed(pw_q); q5_saw <= unsigned(sc_q(15 downto 8)); q5_cg <= signed(sc_q(7 downto 0));
            q5_el <= f_c8(shift_left(resize(q4_d, 17) - 64, 3));
            q5_er <= f_c8(shift_left(resize(q4_e, 17) + 330, 2));
            if f_nowedge = '1' then q5_e2 <= (others => '0');
            else q5_e2 <= f_c8(shift_left(-resize(q4_e, 17) - 780, 1)); end if;            -- x - xr + 40
            -- red rise (x - xL + 85, x 1.5) left of the plume, the wedge ramp (x - xr + 100, x 1.25) right of it
            if q4_e < -400 then
                q5_sd <= '1'; v_r20 := -resize(q4_e, 20) - 720;
                v_r20 := v_r20 + shift_right(v_r20, 2);
                if f_nowedge = '1' then q5_v8 <= (others => '0'); else q5_v8 <= f_c8(v_r20); end if;
            else
                q5_sd <= '0'; v_r20 := resize(q4_d, 20) + 85;
                q5_v8 <= f_c8(v_r20 + shift_right(v_r20, 1));
            end if;
            q5_fd <= q4_e + 85;
            -- 6: cos address (ground stripes); narrow products; window -> 4-bit blend weight
            cs1_a <= q5_gw(15 downto 8);
            q6_gw <= q5_gw;
            q6_sw <= q5_saw * l_sgw6;
            q6_tth <= resize(shift_right(q5_pw, 2) * signed('0' & f_tk(2 downto 0)), 12);
            q6_ab <= resize(shift_right(q5_pw, 1) * signed('0' & l_ab6), 13);
            if q5_el < q5_er then v_k := q5_el; else v_k := q5_er; end if;
            if q5_e2 > v_k then v_k := q5_e2; end if;
            v_e9 := resize(v_k, 9) + 16;
            q6_et4 <= v_e9(8 downto 5);
            q6_v8 <= q5_v8; q6_sd <= q5_sd; q6_fd <= q5_fd; q6_pw <= q5_pw;
            -- 7: plume stripe phase (shear) -> cos address; B plume; teeth; seam G term
            v_u16 := q6_gw + l_dlt - (q6_sw(11 downto 0) & "0000");
            cs2_a <= v_u16(15 downto 8);
            q7_bp <= f_c8(signed(resize(l_mb, 12)) + resize(shift_right(q6_ab, 4), 12));
            q7_fd <= q6_fd + resize(q6_tth, 14);
            q7_et4 <= q6_et4; q7_v8 <= q6_v8; q7_sd <= q6_sd;
            -- 8: fall ramp (x41/32 for a 200-px fall, x27/16 for 150 px); ground cos copy
            q8_frp <= resize(q7_fd, 20) + shift_right(resize(q7_fd, 20), 1);
            q8_cgg <= signed(cs1_q);
            q8_bp <= q7_bp; q8_et4 <= q7_et4; q8_v8 <= q7_v8; q8_sd <= q7_sd;
            -- 9: fall clamp; plume cos copy; ground products
            q9_fr <= f_c8(q8_frp);
            q9_cgp <= signed(cs2_q);
            v_c9 := to_signed(127, 9) - resize(q8_cgg, 9);
            q9_e2p <= unsigned(v_c9(7 downto 1)) * l_e26;
            q9_bk <= resize(q8_cgg * signed('0' & l_bk3), 12);
            q9_bp <= q8_bp; q9_et4 <= q8_et4; q9_v8 <= q8_v8; q9_sd <= q8_sd;
            -- 10: red key; plume G product; ground G and B
            if q9_sd = '1' then
                if q9_fr > q9_v8 then v_k := q9_fr; else v_k := q9_v8; end if;
            else
                if q9_fr < q9_v8 then v_k := q9_fr; else v_k := q9_v8; end if;
            end if;
            if l_rl < v_k then v_k := l_rl; end if;
            q10_r <= v_k;
            q10_gk <= resize(shift_right(q9_cgp, 1) * signed('0' & l_gk6), 14);
            q10_gg <= f_c8(signed(resize(l_e1, 11)) - signed(resize(q9_e2p(13 downto 5), 11)));
            q10_bg <= f_c8(signed(resize(l_bb, 11)) - resize(shift_right(q9_bk, 3), 11));
            q10_bp <= q9_bp; q10_et4 <= q9_et4;
            -- 11: plume G; B difference
            q11_gp <= f_c8(signed(resize(l_go, 11)) + resize(shift_right(q10_gk, 5), 11));
            q11_db <= signed(resize(q10_bp, 9)) - signed(resize(q10_bg, 9));
            q11_gg <= q10_gg; q11_bg <= q10_bg; q11_r <= q10_r; q11_et4 <= q10_et4;
            -- 12: G difference; B blend product
            q12_dg <= signed(resize(q11_gp, 9)) - signed(resize(q11_gg, 9));
            q12_pb <= resize(q11_db * signed('0' & q11_et4), 13);
            q12_gg <= q11_gg; q12_bg <= q11_bg; q12_r <= q11_r; q12_et4 <= q11_et4;
            -- 13: G blend product; B
            q13_pg <= resize(q12_dg * signed('0' & q12_et4), 13);
            q13_b <= f_c8(signed(resize(q12_bg, 11)) + resize(shift_right(q12_pb, 3), 11));
            q13_gg <= q12_gg; q13_r <= q12_r;
            -- 14: G; palette (Swap exchanges red and blue)
            q14_g <= f_c8(signed(resize(q13_gg, 11)) + resize(shift_right(q13_pg, 3), 11));
            if f_swap = '1' then q14_r <= q13_b; q14_b <= q13_r; else q14_r <= q13_r; q14_b <= q13_b; end if;
            -- 15: RGB -> YUV terms, Q6 (hardware convention: U = Cr, V = Cb); luma x16/13
            --     Y = 81 R + 159 G + 31 B;  U = 112 R - (94 G + 18 B);  V = 112 B - (38 R + 75 G)
            v_r := resize(q14_r, 16); v_gg := resize(q14_g, 16); v_b := resize(q14_b, 16);
            q15_yrg <= resize(shift_left(v_r, 6), 17) + resize(shift_left(v_r, 4), 17) + resize(v_r, 17)
                       + resize(shift_left(v_gg, 7), 17) + resize(shift_left(v_gg, 5), 17) - resize(v_gg, 17);
            q15_yb  <= shift_left(v_b, 5) - v_b;
            q15_ur  <= shift_left(v_r, 7) - shift_left(v_r, 4);
            q15_ugb <= resize(shift_left(v_gg, 6), 17) + resize(shift_left(v_gg, 5), 17) - resize(shift_left(v_gg, 1), 17)
                       + resize(shift_left(v_b, 4), 17) + resize(shift_left(v_b, 1), 17);
            q15_vb  <= shift_left(v_b, 7) - shift_left(v_b, 4);
            q15_vrg <= resize(shift_left(v_r, 5), 17) + resize(shift_left(v_r, 2), 17) + resize(shift_left(v_r, 1), 17)
                       + resize(shift_left(v_gg, 6), 17) + resize(shift_left(v_gg, 3), 17) + resize(shift_left(v_gg, 1), 17)
                       + resize(v_gg, 17);
            -- 16: sums
            q16_y <= q15_yrg + resize(q15_yb, 17);
            q16_u <= signed(resize(q15_ur, 18)) - signed(resize(q15_ugb, 18));
            q16_v <= signed(resize(q15_vb, 18)) - signed(resize(q15_vrg, 18));
            -- 17: Y, U, V (10 bit); luma capped at 940 (-> ~777 after the output scaling)
            v_y := shift_right(q16_y, 6);
            if v_y > 876 then q17_y <= to_unsigned(940, 10); else q17_y <= to_unsigned(64, 10) + v_y(9 downto 0); end if;
            v_cs := shift_right(q16_u, 6) + 512;
            if v_cs < 0 then q17_u <= (others => '0'); elsif v_cs > 1023 then q17_u <= (others => '1');
            else q17_u <= unsigned(v_cs(9 downto 0)); end if;
            v_cs := shift_right(q16_v, 6) + 512;
            if v_cs < 0 then q17_v <= (others => '0'); elsif v_cs > 1023 then q17_v <= (others => '1');
            else q17_v <= unsigned(v_cs(9 downto 0)); end if;

            -- 18: box sums: chroma over an 8-tap ring window (box pair = [1 2 2 2 2 2 2 2 1]/16, the HD HDMI
            --     bar fix), luma over 4 taps from a short delay line (then [1 2 1]).
            --     The ring's input is forced to black/neutral in the vblank zone and the sums re-seeded there.
            s_wp <= s_wp + 1;
            rbr_a <= s_wp - 5;
            s_yd(0) <= rb_wd(29 downto 20); s_yd(1) <= s_yd(0); s_yd(2) <= s_yd(1); s_yd(3) <= s_yd(2);
            rs_ra <= s_wp - (C_LAT - 3);
            if s_zone = '1' then
                v_iy := (others => '0'); v_iu := to_unsigned(512, 10); v_iv := to_unsigned(512, 10);
            else
                v_iy := q17_y; v_iu := q17_u; v_iv := q17_v;
            end if;
            rb_wd <= std_logic_vector(v_iy) & std_logic_vector(v_iu) & std_logic_vector(v_iv);
            if s_zone = '1' and s_zc = 40 then
                s_ysum <= (others => '0'); s_usum <= to_unsigned(4096, 14); s_vsum <= to_unsigned(4096, 14);
            else
                s_ysum <= s_ysum + unsigned(rb_wd(29 downto 20)) - unsigned(s_yd(3));
                s_usum <= s_usum + unsigned(rb_wd(19 downto 10)) - unsigned(rbr_q2(19 downto 10));
                s_vsum <= s_vsum + unsigned(rb_wd(9 downto 0)) - unsigned(rbr_q2(9 downto 0));
            end if;
            -- 19..20: luma box out; chroma box pair
            q19_y <= s_ysum(11 downto 2);
            s_ud <= s_usum; s_vd <= s_vsum;
            v_sc := resize(s_usum, 15) + s_ud;
            q20_u <= v_sc(13 downto 4);
            v_sc := resize(s_vsum, 15) + s_vd;
            q20_v <= v_sc(13 downto 4);
            -- 20..21: luma [1 2 1]
            q20_y <= q19_y; q21_y <= q20_y;
            v_sy := resize(q19_y, 12) + shift_left(resize(q20_y, 12), 1) + resize(q21_y, 12);
            q21_yf <= v_sy(11 downto 2);
            q21_u <= q20_u; q21_v <= q20_v;
            q22_u <= q21_u; q22_v <= q21_v;
            -- 23: luma toward black (white 940 -> 777); blanking gate
            if q21_yf > 64 then v_dl := q21_yf - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(C_LAT - 2) = '1' then
                q23_y <= q21_yf - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                q23_u <= q22_u; q23_v <= q22_v;
            else
                q23_y <= to_unsigned(64, 10); q23_u <= to_unsigned(512, 10); q23_v <= to_unsigned(512, 10);
            end if;
            -- 24
            q24_y <= q23_y; q24_u <= q23_u; q24_v <= q23_v;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(q24_y);
    data_out.u       <= std_logic_vector(q24_u);
    data_out.v       <= std_logic_vector(q24_v);
    data_out.hsync_n <= rs_q2(2);
    data_out.vsync_n <= rs_q2(1);
    data_out.field_n <= rs_q2(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture plume;
