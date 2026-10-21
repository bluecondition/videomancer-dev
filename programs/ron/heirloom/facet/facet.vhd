-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Facet v0.5.0 -- Heirloom #40 (rebuilt 2026-10-21), from WIN_20230602_01_26_17.
--
-- The photo: a mirrored row of green "eyes" (lens shapes |V| < h(H-osc),
-- pointed tips, rounded tops) on a blue -> teal -> green Lambda gradient; a
-- purple wedge from the top and a purple V floor, both edged with white
-- lines and dark shadows; where the wedge / V cut an eye they turn red (the
-- red dome and cone of the centre eye, the red caps of the half-eyes at the
-- frame edges); a light-green halo beside the red with pale rim highlights;
-- the wedge continues below the centre eye as a black cone tip.
--
-- The program redraws that patch from its fields:
--   lens   phase = |X - axL| * n (folded oscillator), h = Hl * s(t) from a
--          512-entry shape ROM, eye where h - |Y - 518| > 0
--   wedge  eA = |X - axA| + kA (Y - 942)          (tip fixed at Y = 942)
--   floor  eB = (B0 - Y) / kB - |X - axA|
--   colour five 256-entry ramps (bg, purple, red, light, dark green) over
--          D = Y - 1.05 |X - axA|, fitted to the photo
--   edges  an x-order edge processor: dark shadow left of each wedge / floor
--          edge, white line on it, blue after-glow; pale rims inside the eye
--          in the halo zone; soft lens edges with the blue leading
--
--   K1  Eyes     eyes across: 0.75 .. 6 (2.03 = the photo)
--   K2  Wedge    wedge opening about its tip: needle .. wide fan
--   K3  Floor    floor V: sunk off-screen .. risen to mid-screen
--   K4  Speed    breathing motion (eye opening, wedge sway, floor bob,
--                lateral eye sway, colour drift, Osc drift); 0 = still
--   K5  Halo     width of the light-green band beside the red
--   K6  Picture  source depth: bends the eye / wedge edges, shades D
--   S7  Lines    white edge lines + shadows (On)
--   S8  Glow     pale rim highlights in the halo (On)
--   S9  Twin     eye phase + half a period: two eyes flank the centre
--   S10 Palette  Jewel (the photo) / Ember (chroma complement)
--   S11 Source   Video / Osc (a slow triangle-plaid oscillator)
--   P12 OPEN     eye height: shut slits .. eyes filling the frame
--
-- Heirloom v0.2.0 shell: timing-ID virtual-1080 steps, vblank zone with
-- per-field latches, measured width, generated avid (beam initialised on
-- g_start), source line buffer, output conditioning (chroma
-- [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], luma 13/16), blanking gate on the
-- latency-aligned generated avid.  One serial shift-add multiplier runs the
-- per-field sequencer (zone) and the per-line jobs (after each line).
-- Latency 32 clocks in every mode.  Colours in hardware convention
-- (u = Cr, v = Cb); luma stored x16/13 so the conditioning lands on the photo.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.facet_pkg.all;
use work.all;

architecture facet of program_top is
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
    -- triangle of a 16-bit phase -> 7 bits
    function f_tri7(v : unsigned(15 downto 0)) return unsigned is
    begin
        if v(15) = '1' then return not v(14 downto 8); else return v(14 downto 8); end if;
    end function;
    -- 127 sin(2 pi i / 256), quarter-wave table
    type t_q is array (0 to 63) of integer;
    constant C_QS : t_q := (
        2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47,
        50, 53, 56, 59, 62, 64, 67, 70, 72, 75, 77, 80, 82, 85, 87, 89,
        91, 94, 96, 98, 100, 102, 104, 105, 107, 109, 110, 112, 113, 115, 116, 117,
        118, 119, 120, 121, 122, 123, 124, 125, 125, 126, 126, 126, 127, 127, 127, 127);
    function f_sin8(p : unsigned(7 downto 0)) return signed is
        variable k : integer;
        variable q : unsigned(5 downto 0);
    begin
        if p(6) = '1' then q := not p(5 downto 0); else q := p(5 downto 0); end if;
        k := C_QS(to_integer(q));
        if p(7) = '1' then return to_signed(-k, 8); else return to_signed(k, 8); end if;
    end function;

    ---------------------------------------------------------------------------
    -- ROMs / RAM
    ---------------------------------------------------------------------------
    signal rom_co : t_co := C_CO;
    signal rom_ci : t_ci := C_CI;
    signal rom_tg : t_tg := C_TG;
    signal rom_ls : t_ls := C_LS;
    signal rom_inc : t_inc := C_INC;
    signal s_co_ra, s_tg_ra : unsigned(8 downto 0) := (others => '0');
    signal s_ci_ra : unsigned(9 downto 0) := (others => '0');
    signal q_co, q_ci, q_tg : std_logic_vector(31 downto 0) := (others => '0');
    signal s_ls_ra : unsigned(8 downto 0) := (others => '0');
    signal q_ls : std_logic_vector(15 downto 0) := (others => '0');
    signal s_inc_ra : unsigned(7 downto 0) := (others => '0');
    signal q_inc : std_logic_vector(15 downto 0) := (others => '0');

    type t_lb is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"6E");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    signal s_lb_wa, s_lb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"6E";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"6E";

    ---------------------------------------------------------------------------
    -- timing / controls
    ---------------------------------------------------------------------------
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    -- glides (value << 3)
    signal g_k1, g_k2, g_k3, g_k5, g_p12 : unsigned(12 downto 0) := (others => '0');
    signal s_spd : unsigned(9 downto 0) := (others => '0');
    signal s_warm : unsigned(1 downto 0) := (others => '0');
    signal gl_cnt : unsigned(4 downto 0) := (others => '0');
    signal gl_on : std_logic := '0';
    signal gl_t, gl_c, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal j_slot : unsigned(4 downto 0) := (others => '0');
    signal j_n : unsigned(4 downto 0) := (others => '0');
    signal j_on : std_logic := '0';
    -- registered zone-counter decodes (one clock ahead)
    signal zf_s0, zf_s1, zf_s2, zf_s3, zf_s4, zf_inc, zf_vw, zf_ox, zf_hm, zf_gl, zf_j, zf_l0 : std_logic := '0';
    signal s_lines, s_glow, s_twin, s_ember, s_osc : std_logic := '0';
    -- LFOs
    signal s_lm : unsigned(19 downto 0) := (others => '0');      -- master LFO phase
    signal s_linc : unsigned(15 downto 0) := (others => '0');
    signal s_sidx : unsigned(7 downto 0) := (others => '0');
    signal s_sval : signed(7 downto 0) := (others => '0');
    -- serial multiplier: a (signed 26) x b (unsigned 16)
    signal m_a : signed(26 downto 0) := (others => '0');
    signal m_b : unsigned(15 downto 0) := (others => '0');
    signal m_acc : signed(43 downto 0) := (others => '0');
    signal m_i : unsigned(4 downto 0) := (others => '0');
    signal m_run, m_done : std_logic := '0';
    signal m_jid : unsigned(4 downto 0) := (others => '0');
    signal m_ld : std_logic := '0';
    -- per-field constants
    signal f_k2sq : unsigned(19 downto 0) := (others => '0');
    signal f_kA, f_kAf : unsigned(12 downto 0) := to_unsigned(1247, 13);
    signal f_Hl : unsigned(9 downto 0) := to_unsigned(308, 10);
    signal f_Hlf : unsigned(10 downto 0) := to_unsigned(308, 11);
    signal f_B0, f_B0f : unsigned(10 downto 0) := to_unsigned(1086, 11);
    signal f_hs, f_hm : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal f_vw48 : unsigned(12 downto 0) := to_unsigned(1968, 13);
    signal f_oALf, f_oARf, f_oB : unsigned(8 downto 0) := (others => '0');
    signal f_iAR, f_iBL, f_iBR : unsigned(5 downto 0) := (others => '0');
    signal f_kb : unsigned(15 downto 0) := (others => '0');
    signal f_lph : unsigned(15 downto 0) := (others => '0');
    signal f_doff : signed(7 downto 0) := (others => '0');
    signal f_inc : unsigned(14 downto 0) := to_unsigned(8856, 15);
    signal f_incp : unsigned(16 downto 0) := (others => '0');
    signal f_ph0, f_phx : unsigned(23 downto 0) := (others => '0');
    signal f_npre : unsigned(10 downto 0) := to_unsigned(984, 11);
    signal f_r0 : unsigned(2 downto 0) := (others => '0');
    signal f_dCA : signed(31 downto 0) := (others => '0');
    signal f_CA0 : signed(31 downto 0) := (others => '0');
    signal f_dCB : unsigned(19 downto 0) := (others => '0');
    signal f_CB0 : signed(31 downto 0) := (others => '0');
    signal f_depth : unsigned(7 downto 0) := (others => '0');
    signal f_oyinc : unsigned(7 downto 0) := to_unsigned(61, 8);
    signal f_oxinc : unsigned(7 downto 0) := to_unsigned(51, 8);
    -- per-line engine (computes the NEXT line)
    signal l_yq8 : unsigned(18 downto 0) := (others => '0');
    signal l_CA : signed(31 downto 0) := (others => '0');
    signal l_CB : signed(31 downto 0) := (others => '0');
    signal l_step : unsigned(2 downto 0) := (others => '0');
    signal l_fall1, l_go : std_logic := '0';
    signal l_wait : unsigned(4 downto 0) := (others => '0');
    signal l_av2 : unsigned(19 downto 0) := (others => '0');
    -- per-line values (written ~16 clocks after the line, before the next one)
    signal a_cA : signed(12 downto 0) := (others => '0');
    signal a_cB : signed(13 downto 0) := (others => '0');
    signal a_aex : std_logic := '1';
    signal a_av : unsigned(9 downto 0) := (others => '0');
    signal a_oAL, a_oAR : unsigned(10 downto 0) := (others => '0');
    signal a_dl : signed(11 downto 0) := (others => '0');
    signal a_jx : signed(2 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- source side
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_ox, s_oy : unsigned(15 downto 0) := (others => '0');
    signal s_osc8 : unsigned(7 downto 0) := to_unsigned(110, 8);
    signal s_vid8 : unsigned(7 downto 0) := to_unsigned(110, 8);

    -- beam
    signal b_x : signed(12 downto 0) := (others => '0');
    signal b_ph : unsigned(23 downto 0) := (others => '0');
    signal b_cnt : unsigned(10 downto 0) := (others => '0');
    signal b_xl : std_logic := '0';
    signal b_px : unsigned(10 downto 0) := (others => '0');
    signal b_rng : unsigned(15 downto 0) := x"BEEF";

    ---------------------------------------------------------------------------
    -- pixel path
    ---------------------------------------------------------------------------
    type t_sl is array (natural range <>) of std_logic;
    type t_c3 is array (0 to 2) of unsigned(9 downto 0);
    type t_d3 is array (0 to 2) of signed(10 downto 0);
    type t_p3 is array (0 to 2) of signed(15 downto 0);
    -- 1 .. 7: fields
    signal p1_xa : signed(13 downto 0) := (others => '0');
    signal p1_lf : std_logic := '0';
    type t_u13 is array (natural range <>) of unsigned(12 downto 0);
    type t_u6 is array (natural range <>) of unsigned(5 downto 0);
    signal d_rx : t_u13(2 to 5) := (others => (others => '0'));
    signal d_c  : t_u6(3 to 6) := (others => (others => '0'));
    signal p3_s : signed(9 downto 0) := (others => '0');
    signal p3_src : signed(8 downto 0) := (others => '0');
    signal p4_plo, p4_phi : signed(16 downto 0) := (others => '0');
    signal p4_slo, p4_shi : signed(13 downto 0) := (others => '0');
    signal p5_h : signed(11 downto 0) := (others => '0');
    signal p5_sm : signed(8 downto 0) := (others => '0');
    signal p6_m : signed(12 downto 0) := (others => '0');
    signal p6_sme : signed(8 downto 0) := (others => '0');
    signal p6_smD : signed(11 downto 0) := (others => '0');
    signal p6_eA : signed(13 downto 0) := (others => '0');
    signal p6_eB : signed(14 downto 0) := (others => '0');
    signal p6_kda : unsigned(11 downto 0) := (others => '0');
    signal p6_kdb : unsigned(5 downto 0) := (others => '0');
    signal p7_mc : signed(19 downto 0) := (others => '0');
    signal p7_eA : signed(13 downto 0) := (others => '0');
    signal p7_eB : signed(14 downto 0) := (others => '0');
    signal p7_D : signed(12 downto 0) := (others => '0');
    -- 8 .. 11: keys, distances, ROM addresses
    signal p8_hdA, p8_hdB : signed(6 downto 0) := (others => '0');
    signal p8_eA0 : signed(12 downto 0) := (others => '0');
    signal p8_eB3 : signed(16 downto 0) := (others => '0');
    type t_m7 is array (natural range <>) of signed(6 downto 0);
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    signal d_mp : t_m7(8 to 10) := (others => (others => '0'));
    signal d_idx : t_u8(8 to 10) := (others => (others => '0'));
    signal p9_ahA, p9_ahB : unsigned(6 downto 0) := (others => '0');
    signal p9_hdA, p9_hdB : signed(6 downto 0) := (others => '0');
    signal p9_a1, p9_a2, p9_b1, p9_b2 : signed(13 downto 0) := (others => '0');
    signal p10_hdi : signed(5 downto 0) := (others => '0');
    signal p10_hA, p10_hB : signed(13 downto 0) := (others => '0');
    -- 11 .. 12: weights
    signal p11_w, p11_redol, p11_al, p11_alv, p11_rim, p11_dip, p11_dd : unsigned(4 downto 0) := (others => '0');
    signal p11_tail : unsigned(5 downto 0) := (others => '0');
    signal p11_halo : std_logic := '0';
    signal p12_in, p12_isr : std_logic := '0';
    signal p12_w2 : unsigned(4 downto 0) := (others => '0');
    signal p12_tail : unsigned(5 downto 0) := (others => '0');
    signal p12_hz : std_logic := '0';
    signal s_st : signed(4 downto 0) := (others => '0');
    type t_u9 is array (natural range <>) of unsigned(8 downto 0);
    signal d_tga : t_u9(11 to 13) := (others => (others => '0'));
    signal d_in, d_isr : t_sl(14 to 16) := (others => '0');
    signal d_hz : t_sl(13 to 18) := (others => '0');
    signal p16_tg : t_c3 := (others => (others => '0'));
    type t_w5 is array (natural range <>) of unsigned(4 downto 0);
    signal d_al, d_alv : t_w5(12 to 14) := (others => (others => '0'));
    -- 13 .. 20: colours
    signal p13_co, p13_ci : t_c3 := (others => (others => '0'));
    signal p13_in, p13_isr : std_logic := '0';
    signal p13_w2 : unsigned(4 downto 0) := (others => '0');
    signal p13_tail : unsigned(5 downto 0) := (others => '0');
    signal p14_co : t_c3 := (others => (others => '0'));
    signal p14_da : t_d3 := (others => (others => '0'));
    signal p15_co : t_c3 := (others => (others => '0'));
    signal p15_pa : t_p3 := (others => (others => '0'));
    signal p16_x : t_c3 := (others => (others => '0'));
    signal p17_x : t_c3 := (others => (others => '0'));
    signal p17_db : t_d3 := (others => (others => '0'));
    signal p18_x : t_c3 := (others => (others => '0'));
    signal p18_pb : t_p3 := (others => (others => '0'));
    signal d_w2 : t_w5(14 to 17) := (others => (others => '0'));
    type t_u6b is array (natural range <>) of unsigned(5 downto 0);
    signal d_tail : t_u6b(14 to 18) := (others => (others => '0'));
    type t_s5 is array (natural range <>) of signed(4 downto 0);
    signal p19_y, p19_u, p19_v : unsigned(9 downto 0) := (others => '0');
    signal p20_y, p20_u, p20_v : unsigned(9 downto 0) := (others => '0');
    -- delay lines (index = the stage the value is aligned with)
    signal d_lf : t_sl(2 to 10) := (others => '0');
    signal d_sg : t_sl(2 to 8) := (others => '0');
    signal d_ab : t_sl(8 to 11) := (others => '0');
    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y21, s_y22, s_y23, s_y24, s_y25, s_y26x, s_yf26 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u26, s_v26 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y27, s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u27, s_v27, s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

    -- a + (b - a) w / 8 with p = (b - a) w, w = 0 .. 8: stays between a and b, no clamp
    function f_lerp(a : unsigned(9 downto 0); p : signed(15 downto 0)) return unsigned is
        variable v : signed(11 downto 0);
    begin
        v := signed(resize(a, 12)) + resize(shift_right(p, 3), 12);
        return unsigned(v(9 downto 0));
    end function;
    function f_diff(a, b : unsigned(9 downto 0)) return signed is
    begin
        return signed(resize(b, 11)) - signed(resize(a, 11));
    end function;
    -- (b - a) x w for w = 0 .. 8: one add of gated operands, or the full shift
    function f_mulw(d : signed(10 downto 0); w : unsigned(4 downto 0)) return signed is
        variable g0, g1, g2 : signed(15 downto 0);
    begin
        if w(3) = '1' then return shift_left(resize(d, 16), 3); end if;
        g0 := (others => '0'); g1 := (others => '0'); g2 := (others => '0');
        if w(0) = '1' then g0 := resize(d, 16); end if;
        if w(1) = '1' then g1 := shift_left(resize(d, 16), 1); end if;
        if w(2) = '1' then g2 := shift_left(resize(d, 16), 2); end if;
        return g0 + g1 + g2;
    end function;
    function f_unpack(w : std_logic_vector(31 downto 0)) return t_c3 is
        variable r : t_c3;
    begin
        r(0) := unsigned(w(31 downto 22)); r(1) := unsigned(w(21 downto 12)); r(2) := unsigned(w(11 downto 2));
        return r;
    end function;
    function f_max5(a, b : unsigned(4 downto 0)) return unsigned is
    begin
        if a > b then return a; else return b; end if;
    end function;

begin

    ---------------------------------------------------------------------------
    -- ROMs and the source line buffer
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            q_co <= rom_co(to_integer(s_co_ra));
            q_ci <= rom_ci(to_integer(s_ci_ra));
            q_tg <= rom_tg(to_integer(s_tg_ra));
            q_ls <= rom_ls(to_integer(s_ls_ra));
            q_inc <= rom_inc(to_integer(s_inc_ra));
        end if;
    end process p_rom;

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, LFOs, sequencer, per-line engine
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_vw : unsigned(13 downto 0);
        variable v_d : signed(13 downto 0);
        variable v_t : unsigned(10 downto 0);
        variable v_r : signed(43 downto 0);
        variable v_p : signed(28 downto 0);
        variable v_g : unsigned(12 downto 0);
        variable v_k : unsigned(9 downto 0);
        variable v_m : unsigned(11 downto 0);
        variable v_x : unsigned(23 downto 0);
        variable v_y : signed(19 downto 0);
        variable v_e14 : signed(13 downto 0);
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
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;
            if s_zone = '0' then
                s_zc <= (others => '0');
            elsif s_zc /= 1023 then
                s_zc <= s_zc + 1;
            end if;

            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            if unsigned(registers_in(3)) < 16 then s_spd <= (others => '0'); else s_spd <= unsigned(registers_in(3)); end if;
            s_inc_ra <= g_k1(12 downto 5);

            -- per field: switches, master LFO phase
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                if s_warm /= 3 then s_warm <= s_warm + 1; end if;
                s_lines <= registers_in(6)(0);
                s_glow  <= registers_in(6)(1);
                s_twin  <= registers_in(6)(2);
                s_ember <= registers_in(6)(3);
                s_osc   <= registers_in(6)(4);
                f_depth <= unsigned(registers_in(5)(9 downto 2));
                s_lm <= s_lm + resize(s_linc, 20);
            end if;

            -- glides, one shared datapath pipelined over 4 clocks per channel
            -- (zc 20 .. 39): 1/8 of the gap per field, snap within 1 (and on the
            -- first fields)
            -- registered decodes of the zone counter (each fires on zc = N)
            if s_zone = '1' and s_zc = 133 then zf_s0 <= '1'; else zf_s0 <= '0'; end if;
            if s_zone = '1' and s_zc = 157 then zf_s1 <= '1'; else zf_s1 <= '0'; end if;
            if s_zone = '1' and s_zc = 205 then zf_s2 <= '1'; else zf_s2 <= '0'; end if;
            if s_zone = '1' and s_zc = 421 then zf_s4 <= '1'; else zf_s4 <= '0'; end if;
            if s_zone = '1' and s_zc = 445 then zf_s3 <= '1'; else zf_s3 <= '0'; end if;
            if s_zone = '1' and s_zc = 11 then zf_inc <= '1'; else zf_inc <= '0'; end if;
            if s_zone = '1' and s_zc = 12 then zf_vw <= '1'; else zf_vw <= '0'; end if;
            if s_zone = '1' and s_zc = 13 then zf_ox <= '1'; else zf_ox <= '0'; end if;
            if s_zone = '1' and s_zc = 299 then zf_hm <= '1'; else zf_hm <= '0'; end if;
            if s_zone = '1' and s_zc = 18 then zf_gl <= '1'; else zf_gl <= '0'; end if;
            if s_zone = '1' and s_zc = 13 then zf_j <= '1'; else zf_j <= '0'; end if;
            if s_zone = '1' and s_zc = 699 then zf_l0 <= '1'; else zf_l0 <= '0'; end if;

            if zf_gl = '1' then gl_cnt <= (others => '0'); gl_on <= '1';
            elsif gl_on = '1' then
                if gl_cnt = 19 then gl_on <= '0'; end if;
                gl_cnt <= gl_cnt + 1;
            end if;
            case to_integer(gl_cnt(4 downto 2)) is
                when 0 => v_k := unsigned(registers_in(0)); v_g := g_k1;
                when 1 => v_k := unsigned(registers_in(1)); v_g := g_k2;
                when 2 => v_k := unsigned(registers_in(2)); v_g := g_k3;
                when 3 => v_k := unsigned(registers_in(4)); v_g := g_k5;
                when others => v_k := unsigned(registers_in(7)); v_g := g_p12;
            end case;
            gl_t <= v_k & "000"; gl_c <= v_g;                                  -- phase 0
            gl_d <= signed(resize(gl_t, 14)) - signed(resize(gl_c, 14));       -- phase 1
            if (gl_d > -8 and gl_d < 8) or s_warm /= 3 then gl_n <= gl_t;       -- phase 2
            else
                v_d := signed(resize(gl_c, 14)) + shift_right(gl_d, 3);
                gl_n <= unsigned(v_d(12 downto 0));
            end if;
            if gl_on = '1' and gl_cnt(1 downto 0) = 3 then                     -- phase 3
                case to_integer(gl_cnt(4 downto 2)) is
                    when 0 => g_k1 <= gl_n;
                    when 1 => g_k2 <= gl_n;
                    when 2 => g_k3 <= gl_n;
                    when 3 => g_k5 <= gl_n;
                    when others => g_p12 <= gl_n;
                end case;
            end if;

            -- five LFOs from the master phase, ratios 16 : 11 : 9 : 14 : 6 (/16)
            v_m := s_lm(19 downto 8);
            if zf_s0 = '1' then s_sidx <= v_m(11 downto 4); end if;                                          -- job 5
            if zf_s1 = '1' then v_m := shift_left(v_m, 3) + shift_left(v_m, 1) + v_m; s_sidx <= v_m(11 downto 4); end if;  -- job 6
            v_m := s_lm(19 downto 8);
            if zf_s2 = '1' then v_m := shift_left(v_m, 3) + v_m; s_sidx <= v_m(11 downto 4); end if;         -- job 8
            v_m := s_lm(19 downto 8);
            if zf_s3 = '1' then v_m := shift_left(v_m, 4) - shift_left(v_m, 1); s_sidx <= v_m(11 downto 4); end if;  -- job 18
            v_m := s_lm(19 downto 8);
            if zf_s4 = '1' then v_m := shift_left(v_m, 2) + shift_left(v_m, 1); s_sidx <= v_m(11 downto 4); end if;  -- job 17
            s_sval <= f_sin8(s_sidx);
            if zf_inc = '1' then f_inc <= unsigned(q_inc(14 downto 0)); end if;
            if zf_vw = '1' then f_vw48 <= resize(s_vw, 13) + 48; end if;
            if zf_hm = '1' then
                if f_hs > 256 then f_hm <= to_unsigned(256, 10); else f_hm <= f_hs; end if;
            end if;
            if zf_ox = '1' then
                    case to_integer(s_xs) is
                        when 4      => f_oxinc <= to_unsigned(102, 8);
                        when 3      => f_oxinc <= to_unsigned(77, 8);
                        when others => f_oxinc <= to_unsigned(51, 8);
                    end case;
            end if;

            -- serial multiplier P = a x b (a signed 27 bits, b unsigned 16 bits):
            -- 16 steps of P = (P + b0 a 2^16) / 2.  Job n is announced at
            -- zc = 15 + 24 n and its operands fetched one clock later.
            -- job slots: job n announced every 24 clocks from zc 15 (n = 0 .. 27)
            m_ld <= '0';
            if zf_j = '1' then
                j_on <= '1'; j_slot <= to_unsigned(23, 5); j_n <= (others => '1');
            elsif j_on = '1' then
                if j_slot = 23 then
                    j_slot <= (others => '0');
                    if j_n = 27 then j_on <= '0';
                    else
                        j_n <= j_n + 1;
                        m_ld <= '1'; m_jid <= j_n + 1;
                    end if;
                else
                    j_slot <= j_slot + 1;
                end if;
            end if;
            if l_step = 2 then m_ld <= '1'; m_jid <= to_unsigned(30, 5); end if;
            if l_step = 4 then m_ld <= '1'; m_jid <= to_unsigned(31, 5); end if;

            m_done <= '0';
            if m_ld = '1' then
                m_acc <= (others => '0'); m_i <= (others => '0'); m_run <= '1';
                case to_integer(m_jid) is
                    when 0 =>     -- pixels left of the lens axis: ceil((vw + 48) / xs)
                        m_a <= signed(resize(f_vw48, 27)) + signed(resize(s_xs, 27)) - 1;
                        case to_integer(s_xs) is
                            when 4 =>      m_b <= to_unsigned(16384, 16);
                            when 3 =>      m_b <= to_unsigned(21846, 16);
                            when others => m_b <= to_unsigned(32768, 16);
                        end case;
                    when 1  => m_a <= signed(resize(s_spd, 27));           m_b <= to_unsigned(355, 16);
                    when 2  => m_a <= signed(resize(g_k2(12 downto 3), 27)); m_b <= resize(g_k2(12 downto 3), 16);
                    when 3  => m_a <= signed(resize(f_k2sq, 27));          m_b <= to_unsigned(4751, 16);
                    when 4  => m_a <= signed(resize(g_p12(12 downto 3), 27)); m_b <= to_unsigned(628, 16);
                    when 5  => m_a <= resize(s_sval, 27);                m_b <= resize(f_Hl, 16);
                    when 6  => m_a <= resize(s_sval, 27);                m_b <= resize(f_kA, 16);
                    when 7  => m_a <= signed(resize(g_k3(12 downto 3), 27)); m_b <= to_unsigned(900, 16);
                    when 8  => m_a <= resize(s_sval, 27);                m_b <= to_unsigned(25, 16);
                    when 9  => m_a <= signed(resize(g_k5(12 downto 3), 27)); m_b <= to_unsigned(602, 16);
                    when 10 => m_a <= signed(resize(f_hs, 27));            m_b <= to_unsigned(160, 16);
                    when 11 => m_a <= signed(resize(f_hs, 27));            m_b <= to_unsigned(130, 16);
                    when 12 => m_a <= signed(resize(f_hs, 27));            m_b <= to_unsigned(176, 16);
                    when 13 => m_a <= signed(resize(f_hs, 27));            m_b <= to_unsigned(72, 16);
                    when 14 => m_a <= signed(resize(f_hm, 27));            m_b <= to_unsigned(38, 16);
                    when 15 => m_a <= signed(resize(f_hm, 27));            m_b <= to_unsigned(30, 16);
                    when 16 => m_a <= signed(resize(f_hm, 27));            m_b <= to_unsigned(6, 16);
                    when 17 => m_a <= resize(s_sval, 27);                m_b <= to_unsigned(25, 16);
                    when 18 => m_a <= resize(s_sval, 27);                m_b <= to_unsigned(31, 16);
                    when 19 => m_a <= signed(resize(f_inc, 27));           m_b <= resize(s_xs, 16);
                    when 20 => m_a <= signed(resize(f_vw48, 27));          m_b <= resize(f_inc, 16);
                    when 21 => m_a <= signed(resize(f_npre, 27));          m_b <= resize(s_xs, 16);
                    when 22 => m_a <= signed(resize(f_inc, 27));           m_b <= resize(f_r0, 16);
                    when 23 => m_a <= signed(resize(s_ys, 27));            m_b <= resize(f_kAf, 16);
                    when 24 => m_a <= to_signed(241152, 27);               m_b <= resize(f_kAf, 16);
                    when 25 => m_a <= signed(resize(s_ys, 27));            m_b <= to_unsigned(686, 16);
                    when 26 => m_a <= to_signed(175616, 27);               m_b <= resize(f_B0f, 16);
                    when 27 => m_a <= signed(resize(s_ys, 27));            m_b <= to_unsigned(61, 16);
                    when 30 => m_a <= signed(resize(a_av, 27));            m_b <= resize(a_av, 16);
                    when 31 => m_a <= signed(resize(l_av2, 27));           m_b <= f_kb;
                    when others => m_a <= (others => '0');                  m_b <= (others => '0');
                end case;
            elsif m_run = '1' then
                if m_b(0) = '1' then v_p := resize(m_acc(43 downto 16), 29) + resize(m_a, 29);
                else v_p := resize(m_acc(43 downto 16), 29); end if;
                m_acc <= v_p(28 downto 0) & m_acc(15 downto 1);
                m_b <= '0' & m_b(15 downto 1);
                if m_i = 15 then m_run <= '0'; m_done <= '1'; end if;
                m_i <= m_i + 1;
            end if;

            if m_done = '1' then
                v_r := m_acc;
                v_x := unsigned(v_r(23 downto 0)) + (f_lph & x"00");      -- shared by jobs 20, 22
                case to_integer(m_jid) is
                    when 0  => f_npre <= unsigned(v_r(26 downto 16));
                    when 1  => s_linc <= unsigned(v_r(21 downto 6));
                    when 2  => f_k2sq <= unsigned(v_r(19 downto 0));
                    when 3  => f_kA <= to_unsigned(164, 13) + unsigned(v_r(32 downto 20));
                    when 4  => f_Hl <= to_unsigned(12, 10) + unsigned(v_r(19 downto 10));
                    when 5  =>                                                                       -- +- 6 %
                        v_e14 := signed(resize(f_Hl, 14)) + resize(v_r(21 downto 11), 14);
                        f_Hlf <= unsigned(v_e14(10 downto 0));
                    when 6  =>                                                                       -- +- 12 %
                        v_e14 := signed(resize(f_kA, 14)) + resize(v_r(22 downto 10), 14);
                        f_kAf <= unsigned(v_e14(12 downto 0));
                    when 7  => f_B0 <= to_unsigned(1386, 11) - unsigned(v_r(20 downto 10));
                    when 8  =>                                                                       -- +- 25 lines
                        v_e14 := signed(resize(f_B0, 14)) + resize(v_r(17 downto 7), 14);
                        f_B0f <= unsigned(v_e14(10 downto 0));
                    when 9  => f_hs <= to_unsigned(38, 10) + unsigned(v_r(19 downto 10));
                    when 10 => f_oALf <= unsigned(v_r(16 downto 8));
                    when 11 => f_oARf <= unsigned(v_r(16 downto 8));
                    when 12 => f_oB <= unsigned(v_r(16 downto 8));
                    when 13 => f_kb <= unsigned(v_r(15 downto 0));
                    when 14 => f_iAR <= unsigned(v_r(13 downto 8));
                    when 15 => f_iBL <= unsigned(v_r(13 downto 8));
                    when 16 => f_iBR <= unsigned(v_r(13 downto 8));
                    when 17 => f_doff <= v_r(13 downto 6);                                          -- +- 49
                    when 18 =>                                                                      -- eye sway, + half a period (Twin)
                        f_lph <= unsigned(v_r(16 downto 1));
                        f_lph(15) <= v_r(16) xor s_twin;
                    when 19 => f_incp <= unsigned(v_r(16 downto 0));
                    when 20 => f_ph0 <= v_x;
                    when 21 => f_r0 <= unsigned(v_r(2 downto 0)) - f_vw48(2 downto 0);
                    when 22 => f_phx <= v_x;
                    when 23 => f_dCA <= v_r(31 downto 0);
                    when 24 => f_CA0 <= -v_r(31 downto 0);
                    when 25 => f_dCB <= unsigned(v_r(19 downto 0));
                    when 26 => f_CB0 <= v_r(31 downto 0);
                    when 27 => f_oyinc <= unsigned(v_r(15 downto 8));
                    when 30 => l_av2 <= unsigned(v_r(19 downto 0));
                    when 31 => a_oAL <= resize(f_oALf, 11) + unsigned(v_r(34 downto 24));
                               a_oAR <= resize(f_oARf, 11) + unsigned(v_r(34 downto 24));
                    when others => null;
                end case;
            end if;

            -- per-line engine: the zone sets up line 0; after each active line,
            -- step the DDAs to the next line, then the two halo-bow jobs.
            -- wait 16 clocks after the source line so the last pixels of the
            -- line have passed the stages that use the per-line values
            l_fall1 <= '0';
            if data_in.avid = '0' and s_avq = '1' then l_fall1 <= '1'; end if;
            l_go <= '0';
            if l_fall1 = '1' then l_wait <= to_unsigned(16, 5);
            elsif l_wait /= 0 then
                l_wait <= l_wait - 1;
                if l_wait = 1 then l_go <= '1'; end if;
            end if;
            if zf_l0 = '1' then
                l_yq8 <= (others => '0');
                l_CA <= f_CA0;
                l_CB <= f_CB0;
                l_step <= to_unsigned(1, 3);
            elsif l_go = '1' and s_zone = '0' then
                l_yq8 <= l_yq8 + resize(s_ys, 19);
                l_CA <= l_CA + f_dCA;
                l_CB <= l_CB - signed(resize(f_dCB, 32));
                l_step <= to_unsigned(1, 3);
                if b_rng(2 downto 0) = "000" and a_jx /= 2 then a_jx <= a_jx + 1;
                elsif b_rng(2 downto 0) = "001" and a_jx /= -2 then a_jx <= a_jx - 1; end if;
            elsif l_step = 1 then
                v_y := signed(resize(l_yq8, 20)) - 132608;
                if v_y < 0 then v_y := -v_y; end if;
                a_av <= resize(unsigned(v_y(19 downto 8)), 10);
                v_r := resize(shift_right(l_CA, 19), 44);
                a_cA <= v_r(12 downto 0);
                v_r := resize(shift_right(l_CB, 15), 44);
                a_cB <= v_r(13 downto 0);
                if l_yq8 < 241152 then a_aex <= '1'; else a_aex <= '0'; end if;
                a_dl <= signed(resize(l_yq8(18 downto 8), 12)) + resize(f_doff, 12);
                l_step <= to_unsigned(2, 3);
            elsif l_step = 2 then
                l_step <= to_unsigned(3, 3);
            elsif l_step = 3 then
                if m_done = '1' then l_step <= to_unsigned(4, 3); end if;
            elsif l_step = 4 then
                l_step <= to_unsigned(5, 3);
            elsif l_step = 5 then
                if m_done = '1' then l_step <= to_unsigned(0, 3); end if;
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;   -- field_n is tied high on every input
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
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
    -- Source side: line-buffer write (video luma or the oscillator)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_t : signed(10 downto 0);
        variable v_y : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + resize(f_oxinc, 16);
            elsif s_avq = '1' then
                s_ox <= s_lm(19 downto 4);
                s_oy <= s_oy + resize(f_oyinc, 16);
            end if;
            if s_zone = '1' then s_oy <= not s_lm(18 downto 3); s_ox <= s_lm(19 downto 4); end if;
            v_t := signed(resize(f_tri7(s_ox), 11)) + signed(resize(f_tri7(s_oy), 11)) - 127;
            v_t := shift_right(shift_left(v_t, 1) + v_t, 3);
            v_t := v_t + 110;
            s_osc8 <= unsigned(v_t(7 downto 0));
            v_y := signed(resize(unsigned(data_in.y(9 downto 2)), 11)) - 16;
            if v_y < 0 then s_vid8 <= (others => '0');
            elsif v_y > 219 then s_vid8 <= to_unsigned(219, 8);
            else s_vid8 <= unsigned(v_y(7 downto 0)); end if;
            s_lb_we <= s_avq and not s_wcol(0);
            s_lb_wa <= s_wcol(10 downto 1);
            if s_osc = '1' then s_lb_wd <= std_logic_vector(s_osc8);
            else s_lb_wd <= std_logic_vector(s_vid8); end if;
            if s_avq = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam: virtual X (Q.1, centred), folded lens-phase DDA
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x <= -signed(resize(s_vw, 13));
                b_ph <= f_ph0;
                b_cnt <= f_npre;
                b_xl <= '1';
                b_px <= (others => '0');
            elsif g_avid = '1' then
                b_x <= b_x + signed(resize(s_xs, 13));
                b_px <= b_px + 1;
                if b_cnt >= 2 then
                    b_ph <= b_ph - resize(f_incp, 24);
                elsif b_cnt = 1 then
                    b_ph <= f_phx;
                else
                    b_ph <= b_ph + resize(f_incp, 24);
                end if;
                if b_cnt >= 2 then b_xl <= '1'; else b_xl <= '0'; end if;
                if b_cnt /= 0 then b_cnt <= b_cnt - 1; end if;
            end if;
            b_rng <= b_rng(14 downto 0) & (b_rng(15) xor b_rng(14) xor b_rng(12) xor b_rng(3));
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_s : signed(21 downto 0);
        variable v_mp : signed(19 downto 0);
        variable v_h : signed(14 downto 0);
        variable v_h7 : signed(6 downto 0);
        variable v_a8 : signed(7 downto 0);
        variable v_hd : signed(14 downto 0);
        variable v_d : signed(13 downto 0);
        variable v_ls : signed(10 downto 0);
        variable v_w : unsigned(4 downto 0);
        variable v_cls : unsigned(1 downto 0);
        variable v_t : unsigned(10 downto 0);
        variable v_e : signed(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_c : t_c3;
    begin
        if rising_edge(clk) then
            -- 1: shape-ROM address (folded phase), A/B axis offset, line-buffer read
            if b_ph(23) = '1' then s_ls_ra <= not b_ph(22 downto 14); else s_ls_ra <= b_ph(22 downto 14); end if;
            p1_lf <= b_ph(23) xor b_xl;
            p1_xa <= resize(b_x, 14) + resize(a_jx, 14) - 62;
            s_lb_ra <= b_px(10 downto 1);

            -- 2: |X - axA| and side (left of the axis)
            if p1_xa < 0 then d_rx(2) <= unsigned(resize(-p1_xa, 13)); d_sg(2) <= '1';
            else d_rx(2) <= unsigned(resize(p1_xa, 13)); d_sg(2) <= '0'; end if;
            d_lf(2) <= p1_lf;

            -- 3: ROM / video fabric copies
            p3_s <= signed(q_ls(15 downto 6));
            d_c(3) <= unsigned(q_ls(5 downto 0));
            p3_src <= signed(resize(unsigned(s_lb_q), 9)) - 110;

            -- 4: h = Hl * s as two partial products; source x depth
            p4_plo <= resize(signed('0' & f_Hlf) * signed('0' & p3_s(4 downto 0)), 17);
            p4_phi <= resize(signed('0' & f_Hlf) * p3_s(9 downto 5), 17);
            p4_slo <= resize(p3_src * signed('0' & f_depth(3 downto 0)), 14);
            p4_shi <= resize(p3_src * signed('0' & f_depth(7 downto 4)), 14);

            -- 5
            v_s := shift_left(resize(p4_phi, 22), 5) + resize(p4_plo, 22);
            p5_h <= resize(shift_right(v_s, 9), 12);
            p5_sm <= resize(shift_right(shift_left(resize(p4_shi, 18), 4) + resize(p4_slo, 18), 8), 9);

            -- 6: eye margin (+ source), wedge and floor fields, D shift-adds
            p6_m <= resize(p5_h, 13) - signed(resize(a_av, 13)) + resize(shift_right(p5_sm, 2), 13);
            p6_sme <= shift_right(p5_sm, 2) + shift_right(p5_sm, 3);
            p6_smD <= shift_left(resize(p5_sm, 12), 2) + resize(shift_right(p5_sm, 1), 12) + resize(shift_right(p5_sm, 2), 12);
            p6_eA <= signed(resize(d_rx(5), 14)) + resize(a_cA, 14);
            p6_eB <= resize(a_cB, 15) - signed(resize(d_rx(5), 15));
            p6_kda <= resize(d_rx(5)(12 downto 1), 12) + resize(d_rx(5)(12 downto 5), 12);
            p6_kdb <= resize(d_rx(5)(12 downto 7), 6);

            -- 7: margin x perpendicular factor; the source bends the wedge, floor and D
            p7_mc <= resize(p6_m * signed('0' & d_c(6)), 20);
            p7_eA <= p6_eA + resize(p6_sme, 14);
            p7_eB <= p6_eB + resize(p6_sme, 15);
            p7_D <= (resize(a_dl, 13) + resize(p6_smD, 13)) - (signed(resize(p6_kda, 13)) - signed(resize(p6_kdb, 13)));
            for k in 3 to 5 loop d_rx(k) <= d_rx(k - 1); end loop;
            for k in 4 to 6 loop d_c(k) <= d_c(k - 1); end loop;
            for k in 3 to 8 loop d_sg(k) <= d_sg(k - 1); end loop;
            for k in 3 to 10 loop d_lf(k) <= d_lf(k - 1); end loop;

            -- 8: keys, edge distances, palette index, perpendicular margin
            v_mp := shift_right(p7_mc, 6);
            if v_mp < -32 then d_mp(8) <= to_signed(-32, 7);
            elsif v_mp > 63 then d_mp(8) <= to_signed(63, 7);
            else d_mp(8) <= resize(v_mp, 7); end if;
            if p7_eA < 0 or p7_eB < 0 then d_ab(8) <= '1'; else d_ab(8) <= '0'; end if;
            -- signed horizontal distances to the wedge / floor edges, clamped to
            -- the profiles' reach (+-32 px = +-64 in Q.1)
            if d_sg(7) = '1' then v_hd := -resize(p7_eA, 15); else v_hd := resize(p7_eA, 15); end if;
            if a_aex = '0' or v_hd > 63 then p8_hdA <= to_signed(63, 7);
            elsif v_hd < -64 then p8_hdA <= to_signed(-64, 7);
            else p8_hdA <= resize(v_hd, 7); end if;
            if d_sg(7) = '1' then v_hd := p7_eB; else v_hd := -p7_eB; end if;
            if v_hd > 63 then p8_hdB <= to_signed(63, 7);
            elsif v_hd < -64 then p8_hdB <= to_signed(-64, 7);
            else p8_hdB <= resize(v_hd, 7); end if;
            p8_eA0 <= resize(shift_right(p7_eA, 1), 13);
            p8_eB3 <= shift_left(resize(p7_eB, 17), 1) + resize(p7_eB, 17);
            v_d := resize(p7_D, 14) + 1024;
            if v_d < 0 then d_idx(8) <= (others => '0');
            elsif v_d > 2047 then d_idx(8) <= (others => '1');
            else d_idx(8) <= unsigned(v_d(10 downto 3)); end if;

            -- 9: |hd|; halo distances
            p9_hdA <= p8_hdA; p9_hdB <= p8_hdB;
            v_a8 := abs(resize(p8_hdA, 8)); p9_ahA <= unsigned(v_a8(6 downto 0));
            v_a8 := abs(resize(p8_hdB, 8)); p9_ahB <= unsigned(v_a8(6 downto 0));
            v_ls := resize(shift_right(p8_eB3, 4), 11);
            if d_sg(8) = '1' then
                p9_a1 <= -resize(p8_eA0, 14);
                p9_a2 <= resize(p8_eA0, 14) - signed(resize(a_oAL, 14));
                p9_b1 <= signed(resize(f_iBL, 14)) - resize(v_ls, 14);
            else
                p9_a1 <= signed(resize(f_iAR, 14)) - resize(p8_eA0, 14);
                p9_a2 <= resize(p8_eA0, 14) - signed(resize(a_oAR, 14));
                p9_b1 <= signed(resize(f_iBR, 14)) - resize(v_ls, 14);
            end if;
            p9_b2 <= resize(v_ls, 14) - signed(resize(f_oB, 14));
            d_mp(9) <= d_mp(8); d_idx(9) <= d_idx(8); d_ab(9) <= d_ab(8);

            -- 10: edge index (nearer edge, px); halo maxima
            if p9_ahA <= p9_ahB then v_h7 := p9_hdA; else v_h7 := p9_hdB; end if;
            p10_hdi <= resize(shift_right(v_h7, 1), 6);
            if p9_a1 > p9_a2 then p10_hA <= p9_a1; else p10_hA <= p9_a2; end if;
            if p9_b1 > p9_b2 then p10_hB <= p9_b1; else p10_hB <= p9_b2; end if;
            d_mp(10) <= d_mp(9); d_idx(10) <= d_idx(9); d_ab(10) <= d_ab(9);

            -- 11: halo class, profiles, ROM addresses {class, D index}
            if p10_hA < p10_hB then v_d := p10_hA; else v_d := p10_hB; end if;
            if v_d < 0 then p11_halo <= '1'; else p11_halo <= '0'; end if;
            if d_ab(10) = '1' then v_cls := "10";
            elsif v_d < -5 then v_cls := "01";
            elsif v_d < 0 then v_cls := "11";
            else v_cls := "00"; end if;
            s_co_ra <= d_ab(10) & d_idx(10);
            s_ci_ra <= v_cls & d_idx(10);
            if p10_hdi < -3 then d_tga(11) <= '1' & d_idx(10); else d_tga(11) <= '0' & d_idx(10); end if;
            p11_w <= resize(f_edge(p10_hdi), 5);
            p11_tail <= resize(f_tail(p10_hdi), 6);
            p11_redol <= resize(f_redol(p10_hdi), 5);
            p11_al <= resize(f_alpha(d_mp(10)), 5);
            p11_alv <= resize(f_alphav(d_mp(10)), 5);
            if d_lf(10) = '1' then
                p11_rim <= resize(f_riml(d_mp(10)), 5);
                p11_dip <= resize(f_dipl(d_mp(10)), 5);
            else
                p11_rim <= resize(f_rimr(d_mp(10)), 5);
                p11_dip <= (others => '0');
            end if;
            p11_dd <= resize(f_dd(d_mp(10)), 5);
            d_ab(11) <= d_ab(10);

            -- 12: second-lerp target / weight (inside the eye: rim or outline;
            --     outside: the x-order shadow / white line)
            if p11_al >= 4 then p12_in <= '1'; else p12_in <= '0'; end if;
            p12_isr <= '0';
            if s_glow = '1' and p11_halo = '1' and d_ab(11) = '0' then
                v_w := f_max5(p11_redol, p11_dip);
                if p11_rim /= 0 then p12_isr <= '1'; end if;
            else
                v_w := p11_redol;
            end if;
            if d_ab(11) = '0' and p11_halo = '0' then v_w := f_max5(v_w, p11_dd); end if;
            if p11_al >= 4 then
                if s_glow = '1' and p11_halo = '1' and d_ab(11) = '0' and p11_rim /= 0 then
                    p12_w2 <= p11_rim;
                else
                    p12_w2 <= v_w;
                end if;
                p12_tail <= (others => '0');
            elsif s_lines = '1' then
                p12_w2 <= p11_w; p12_tail <= p11_tail;
            else
                p12_w2 <= (others => '0'); p12_tail <= (others => '0');
            end if;
            if p11_halo = '1' and d_ab(11) = '0' and p11_al >= 4 then p12_hz <= '1'; else p12_hz <= '0'; end if;
            d_al(12) <= p11_al; d_alv(12) <= p11_alv; d_tga(12) <= d_tga(11);

            -- 13: ROM fabric copies
            p13_co <= f_unpack(q_co); p13_ci <= f_unpack(q_ci);
            p13_in <= p12_in; p13_isr <= p12_isr; p13_w2 <= p12_w2; p13_tail <= p12_tail; d_hz(13) <= p12_hz;
            d_tga(13) <= d_tga(12);
            d_al(13) <= d_al(12); d_alv(13) <= d_alv(12);

            -- 14: first lerp difference (Co -> Ci); second-lerp target
            for c in 0 to 2 loop p14_da(c) <= f_diff(p13_co(c), p13_ci(c)); end loop;
            s_tg_ra <= d_tga(13);
            d_in(14) <= p13_in; d_isr(14) <= p13_isr;
            p14_co <= p13_co;
            d_al(14) <= d_al(13); d_alv(14) <= d_alv(13);
            d_w2(14) <= p13_w2; d_tail(14) <= p13_tail;

            -- 15
            p15_pa(0) <= f_mulw(p14_da(0), d_al(14));
            p15_pa(1) <= f_mulw(p14_da(1), d_al(14));
            p15_pa(2) <= f_mulw(p14_da(2), d_alv(14));
            p15_co <= p14_co;

            -- 16: X = the eye blended into the ground
            for c in 0 to 2 loop p16_x(c) <= f_lerp(p15_co(c), p15_pa(c)); end loop;
            p16_tg <= f_unpack(q_tg);
            for k in 15 to 16 loop d_in(k) <= d_in(k - 1); d_isr(k) <= d_isr(k - 1); end loop;

            -- 17
            -- second-lerp target: the shadow / white line outside, rim white or outline dark inside
            if d_in(16) = '0' then v_c := p16_tg;
            elsif d_isr(16) = '1' then
                v_c(0) := to_unsigned(C_RIMW_Y, 10); v_c(1) := to_unsigned(462, 10); v_c(2) := to_unsigned(496, 10);
            else
                v_c(0) := to_unsigned(C_DKC_Y, 10); v_c(1) := to_unsigned(500, 10); v_c(2) := to_unsigned(500, 10);
            end if;
            for c in 0 to 2 loop p17_db(c) <= f_diff(p16_x(c), v_c(c)); end loop;
            p17_x <= p16_x;

            -- 18
            for c in 0 to 2 loop p18_pb(c) <= f_mulw(p17_db(c), d_w2(17)); end loop;
            p18_x <= p17_x;
            for k in 15 to 17 loop d_w2(k) <= d_w2(k - 1); end loop;
            for k in 15 to 18 loop d_tail(k) <= d_tail(k - 1); end loop;
            for k in 14 to 18 loop d_hz(k) <= d_hz(k - 1); end loop;
            if b_px(2 downto 0) = "000" then s_st <= signed(b_rng(4 downto 0)); end if;

            -- 19: out = X + (T - X) w2; blue after-glow; light-zone streaks
            for c in 0 to 2 loop v_c(c) := f_lerp(p18_x(c), p18_pb(c)); end loop;
            if d_hz(18) = '1' then v_e := signed(resize(v_c(0), 12)) + resize(shift_right(s_st, 1), 12);
            else v_e := signed(resize(v_c(0), 12)); end if;
            p19_y <= unsigned(v_e(9 downto 0));
            p19_u <= v_c(1);
            v_t := resize(v_c(2), 11) + resize(d_tail(18), 11);
            if v_t > 1023 then p19_v <= to_unsigned(1023, 10); else p19_v <= v_t(9 downto 0); end if;

            -- 20: Ember = chroma complement
            p20_y <= p19_y;
            if s_ember = '1' then p20_u <= not p19_u; p20_v <= not p19_v;
            else p20_u <= p19_u; p20_v <= p19_v; end if;

            -- 21..26: output conditioning (centre at stage 25 -> 26)
            s_csu(0) <= p20_u; s_csv(0) <= p20_v;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + p20_u - s_csu(7);
            s_boxv <= s_boxv + p20_v - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u26 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v26 <= v_sc(13 downto 4);
            s_y21 <= p20_y; s_y22 <= s_y21; s_y23 <= s_y22; s_y24 <= s_y23;
            s_y25 <= s_y24; s_y26x <= s_y25;
            v_sy := resize(s_y24, 12) + shift_left(resize(s_y25, 12), 1) + resize(s_y26x, 12);
            s_yf26 <= v_sy(11 downto 2);
            -- 27: luma toward black (~777 white); blanking gate
            if s_yf26 > 64 then v_dl := s_yf26 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(C_LAT - 3) = '1' then
                s_y27 <= s_yf26 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u27 <= s_u26; s_v27 <= s_v26;
            else
                s_y27 <= to_unsigned(64, 10); s_u27 <= to_unsigned(512, 10); s_v27 <= to_unsigned(512, 10);
            end if;
            -- 28
            s_y28 <= s_y27; s_u28 <= s_u27; s_v28 <= s_v27;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(s_u28);
    data_out.v       <= std_logic_vector(s_v28);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture facet;
