-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Facet v0.6.2 -- Heirloom #40 (rebuilt 2026-10-21; phase 2 2026-11-01 / 11-02), from
-- WIN_20230602_01_26_17.
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
--   lens   phase = |X - axL| * n (folded oscillator); eye index i counts the
--          pairs out from the centre; h = Hl_i * s(t) from a 1024-entry shape
--          ROM (lens or diamond bank), eye where h - |Y - 518 - yo_i| > 0;
--          Hl_i / yo_i come from an 8-entry eye table rewritten every field
--   wedge  eA = |X - axA| + kA (Y - 942)          (tip fixed at Y = 942)
--   floor  eB = (B0 - Y) / kB - |X - axA|        (Peak: apex B0 - 370, + |X - axA|);
--          the slope kB rides the height (K3)
--   overlap where wedge and floor overlap: an XOR mix of purple and ground (red
--          and light in the eyes) in gradient bands flipped by the source's luma
--   colour five 256-entry ramps (bg, purple, red, light, dark green) over
--          D = Y - 1.05 |X - axA|, fitted to the photo
--   edges  an x-order edge processor: dark shadow left of each wedge / floor
--          edge, white line on it, blue after-glow; pale rims inside the eye
--          in the halo zone; soft lens edges with the blue leading
--
--   K1  Eyes     eyes across: 0.75 .. 6 (2.03 = the photo)
--   K2  Wedge    wedge opening about its tip, cubic: needle (0.02) .. the photo
--                (0.30 at 38 %) .. nearly flat (5.0)
--   K3  Floor    floor apex (V or peak): sunk off-screen with shallow sides (kB
--                0.21) .. the photo (0.373) .. risen to mid-screen, steep (1.2)
--   K4  Speed    rate and depth of every motion, square law: still at 0, the
--                v0.5 motion at 50 %, 4x deeper at 100 %; the eye-pair ripple
--                (a wave running out from the centre) and the wedge / floor
--                slide across the eyes deepen fastest (+-4 .. +-253 lines/px)
--   K5  Spread   per-pair eye heights G_i = 1 + i sigma (the centre eye on P12):
--                left the pairs shrink outward to slits, centre equal (the
--                photo), right they swell outward (x2.5, x4, ...)
--   K6  Picture  source depth: bends the eye / wedge edges, shades D
--   S7  Floor    V (the photo) / Peak (a pyramid rising toward the wedge tip)
--   S8  Eyes     Lens (the photo) / Diamond (straight flanks)
--   S9  Twin     eye phase + half a period: two eyes flank the centre
--   S10 Palette  Jewel (the photo) / Ember (chroma complement)
--   S11 Source   Video / Osc (a slow triangle-plaid oscillator)
--   P12 OPEN     eye height and halo width: shut slits .. eyes filling the frame
--
-- Heirloom v0.2.0 shell: timing-ID virtual-1080 steps, vblank zone with
-- per-field latches, measured width, generated avid (beam initialised on
-- g_start), source line buffer, output conditioning (chroma
-- [1 2 2 2 2 2 2 2 1]/16 on 8-bit chroma from a block-RAM ring, luma [1 2 1],
-- luma 13/16), blanking gate on the latency-aligned generated avid.  One
-- serial shift-add multiplier runs the per-field sequencer (zone, 54 jobs
-- including the eye table) and the per-line jobs (after each line).
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
    signal s_ls_ra : unsigned(9 downto 0) := (others => '0');
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
    -- eye table: {Hl_i (11), yo_i (signed 10)} per eye pair, written by the zone sequencer
    type t_eye is array (0 to 255) of std_logic_vector(31 downto 0);
    signal ram_eye : t_eye := (others => "00100110100" & "0000000000" & "00000000000");
    attribute no_rw_check of ram_eye : signal is true;
    signal s_eye_ra : unsigned(2 downto 0) := (others => '0');
    signal q_eye : std_logic_vector(31 downto 0) := "00100110100" & "0000000000" & "00000000000";
    signal e_we : std_logic;
    signal e_wa : unsigned(2 downto 0);
    signal e_wd : std_logic_vector(31 downto 0);
    -- the chroma box filter's 8-back tap: a 256 x 16 ring (u8 & v8), read 7 behind the write
    type t_cring is array (0 to 255) of std_logic_vector(15 downto 0);
    signal s_cring : t_cring := (others => x"8080");
    attribute no_rw_check of s_cring : signal is true;
    signal s_cwp : unsigned(7 downto 0) := (others => '0');
    signal s_crp : unsigned(7 downto 0) := to_unsigned(249, 8);
    signal s_crq : std_logic_vector(15 downto 0) := x"8080";

    ---------------------------------------------------------------------------
    -- timing / controls
    ---------------------------------------------------------------------------
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(10 downto 0) := (others => '0');
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
    signal j_nx : unsigned(5 downto 0) := (others => '0');
    signal j_on : std_logic := '0';
    -- registered zone-counter decodes (one clock ahead)
    signal zf_inc, zf_vw, zf_ox, zf_gl, zf_j, zf_l0 : std_logic := '0';
    signal s_peak, s_diam, s_twin, s_ember, s_osc : std_logic := '0';
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
    signal m_jid : unsigned(5 downto 0) := (others => '0');
    -- operand select codes, registered with m_jid
    constant OA_ZERO : integer := 0;  constant OA_VWX : integer := 1;   constant OA_SPD : integer := 2;
    constant OA_K2 : integer := 3;    constant OA_K2SQ : integer := 4;  constant OA_P12 : integer := 5;
    constant OA_DEP : integer := 6;   constant OA_TMP : integer := 7;   constant OA_SVAL : integer := 8;
    constant OA_K3 : integer := 9;    constant OA_HS : integer := 10;   constant OA_LM : integer := 11;
    constant OA_INC : integer := 12;  constant OA_VW48 : integer := 13; constant OA_NPRE : integer := 14;
    constant OA_YS : integer := 15;   constant OA_C241 : integer := 16; constant OA_B0F : integer := 17;
    constant OA_K5 : integer := 18;   constant OA_AV : integer := 19;   constant OA_AV2 : integer := 20;
    constant OA_HLF : integer := 21;
    constant OB_CST : integer := 0;   constant OB_SPD : integer := 1;   constant OB_K2 : integer := 2;
    constant OB_DEP : integer := 3;   constant OB_HL : integer := 4;    constant OB_KA : integer := 5;
    constant OB_INC : integer := 6;   constant OB_XS : integer := 7;    constant OB_R0 : integer := 8;
    constant OB_KAF : integer := 9;   constant OB_KF : integer := 10;   constant OB_DEP3 : integer := 11;
    constant OB_AV : integer := 12;   constant OB_KB : integer := 13;   constant OB_G : integer := 14;
    constant OB_K3 : integer := 15;
    signal a_sel : unsigned(4 downto 0) := (others => '0');
    signal b_sel : unsigned(3 downto 0) := (others => '0');
    signal b_cst : unsigned(15 downto 0) := (others => '0');
    signal m_ld : std_logic := '0';
    -- per-field constants
    signal f_k2sq : unsigned(19 downto 0) := (others => '0');
    signal f_kA, f_kAf : unsigned(14 downto 0) := to_unsigned(1247, 15);
    signal f_kf : unsigned(10 downto 0) := to_unsigned(686, 11);          -- 2 / floor slope, Q7
    signal f_Hl : unsigned(9 downto 0) := to_unsigned(308, 10);
    signal f_Hlf : unsigned(10 downto 0) := to_unsigned(308, 11);
    signal f_B0, f_B0f : unsigned(10 downto 0) := to_unsigned(1086, 11);
    signal f_hs : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal f_vw48 : unsigned(12 downto 0) := to_unsigned(1968, 13);
    signal f_vwx : unsigned(12 downto 0) := to_unsigned(1969, 13);       -- vw + 48 + xs - 1
    signal f_oALf, f_oARf, f_oB : unsigned(8 downto 0) := (others => '0');
    signal f_iAR : unsigned(6 downto 0) := (others => '0');
    signal f_iBL, f_iBR : unsigned(5 downto 0) := (others => '0');
    signal f_kb : unsigned(15 downto 0) := (others => '0');
    signal f_lph : signed(16 downto 0) := (others => '0');                -- eye sway (+ Twin), 1/65536 period
    signal f_doff : signed(8 downto 0) := (others => '0');
    -- speed law (square), depths, slide, spread, eye-table accumulator
    signal f_dep : unsigned(8 downto 0) := (others => '0');
    signal f_dep3 : unsigned(12 downto 0) := (others => '0');
    signal f_slide : signed(9 downto 0) := (others => '0');
    signal f_sig : signed(13 downto 0) := (others => '0');
    signal f_G : unsigned(15 downto 0) := to_unsigned(4096, 16);
    -- scratch: dep^2 (job 6), the LFO sine x depth (jobs 8 .. 25), then Hl_i in the eye loop
    signal s_tmp : signed(11 downto 0) := to_signed(308, 12);
    signal f_inc : unsigned(14 downto 0) := to_unsigned(8856, 15);
    signal f_incp : unsigned(16 downto 0) := (others => '0');
    signal f_ph0, f_phx : unsigned(26 downto 0) := (others => '0');
    signal f_npre : unsigned(10 downto 0) := to_unsigned(984, 11);
    signal f_r0 : unsigned(2 downto 0) := (others => '0');
    signal f_dCA : signed(25 downto 0) := (others => '0');
    signal f_CA0 : signed(27 downto 0) := (others => '0');
    signal f_dCB : unsigned(16 downto 0) := (others => '0');
    signal f_CB0 : signed(25 downto 0) := (others => '0');
    signal f_depth : unsigned(7 downto 0) := (others => '0');
    signal f_oyinc : unsigned(7 downto 0) := to_unsigned(61, 8);
    signal f_oxinc : unsigned(7 downto 0) := to_unsigned(51, 8);
    signal f_x62 : unsigned(6 downto 0) := to_unsigned(68, 7);
    -- per-line engine (computes the NEXT line)
    signal l_yq8 : unsigned(18 downto 0) := (others => '0');
    signal l_CA : signed(27 downto 0) := (others => '0');
    signal l_CB : signed(25 downto 0) := (others => '0');
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
    signal a_vy : signed(11 downto 0) := (others => '0');
    signal a_axo : signed(11 downto 0) := to_signed(-62, 12);

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
    signal b_ph : unsigned(26 downto 0) := (others => '0');
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
    signal p4_xa : signed(13 downto 0) := (others => '0');
    signal p1_lf : std_logic := '0';
    type t_u13 is array (natural range <>) of unsigned(12 downto 0);
    type t_u6 is array (natural range <>) of unsigned(5 downto 0);
    signal d_rx5 : unsigned(12 downto 0) := (others => '0');
    signal d_c  : t_u6(3 to 6) := (others => (others => '0'));
    signal p3_s : signed(9 downto 0) := (others => '0');
    signal p3_src : signed(8 downto 0) := (others => '0');
    signal p3_hl : unsigned(10 downto 0) := to_unsigned(308, 11);
    signal p3_yo : signed(9 downto 0) := (others => '0');
    signal p4_dy : signed(11 downto 0) := (others => '0');
    signal p5_av : unsigned(9 downto 0) := (others => '0');
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
    signal d_sg : t_sl(5 to 8) := (others => '0');
    signal d_ab : t_sl(8 to 11) := (others => '0');
    signal d_ov : t_sl(8 to 10) := (others => '0');                     -- wedge and floor overlap
    signal d_par : t_sl(3 to 10) := (others => '0');                    -- source luma bit parity (32-level contours)
    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    -- the syncs ride a RAM ring, so the box gate gets its own avid delay: s_gav(k) = generated avid of the
    -- stage-(k+1) pixel (s_gav(18) gates p20); s_bxh = that gate over the last 8 box inputs
    signal s_gav : std_logic_vector(18 downto 0) := (others => '0');
    signal s_bxh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y21, s_y22, s_y23, s_y24, s_y25, s_y26x, s_yf26 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u26, s_v26 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y27, s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u27, s_v27, s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- sync delay: {hsync, vsync, generated avid} through a 256 x 4 ring read 25 behind
    -- the write (= tap C_LAT - 3, the gate), then two FFs to C_LAT - 1
    type t_sring is array (0 to 255) of std_logic_vector(3 downto 0);
    signal s_sring : t_sring := (others => "1100");
    attribute no_rw_check of s_sring : signal is true;
    signal s_srp : unsigned(7 downto 0) := to_unsigned(231, 8);
    signal s_srq, s_sr1, s_sr2 : std_logic_vector(3 downto 0) := "1100";
    signal s_field0 : std_logic := '1';

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

    p_eye : process(clk)
    begin
        if rising_edge(clk) then
            q_eye <= ram_eye(to_integer("00000" & s_eye_ra));
            if e_we = '1' then ram_eye(to_integer("00000" & e_wa)) <= e_wd; end if;
        end if;
    end process p_eye;

    p_sring : process(clk)
    begin
        if rising_edge(clk) then
            s_srq <= s_sring(to_integer(s_srp));                            -- = the input 25 clocks ago
            s_sring(to_integer(s_cwp)) <= data_in.hsync_n & data_in.vsync_n & g_avid & '0';
            s_srp <= s_srp + 1;
            s_sr1 <= s_srq; s_sr2 <= s_sr1;                                 -- 27 clocks ago
        end if;
    end process p_sring;

    p_cring : process(clk)
    begin
        if rising_edge(clk) then
            s_crq <= s_cring(to_integer(s_crp));
            s_cring(to_integer(s_cwp)) <= std_logic_vector(p20_u(9 downto 2)) & std_logic_vector(p20_v(9 downto 2));
            s_cwp <= s_cwp + 1; s_crp <= s_crp + 1;
        end if;
    end process p_cring;

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
        variable v_e17 : signed(16 downto 0);
        variable v_m10 : unsigned(9 downto 0);
        variable v_gs : signed(17 downto 0);
        variable v_ph : unsigned(26 downto 0);
        variable v_ldj : std_logic;
        variable v_id : unsigned(5 downto 0);
        variable v_k5 : signed(9 downto 0);
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
                s_peak  <= registers_in(6)(0);
                s_diam  <= registers_in(6)(1);
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
            if s_zone = '1' and s_zc = 11 then zf_inc <= '1'; else zf_inc <= '0'; end if;
            if s_zone = '1' and s_zc = 12 then zf_vw <= '1'; else zf_vw <= '0'; end if;
            if s_zone = '1' and s_zc = 13 then zf_ox <= '1'; else zf_ox <= '0'; end if;
            if s_zone = '1' and s_zc = 18 then zf_gl <= '1'; else zf_gl <= '0'; end if;
            if s_zone = '1' and s_zc = 13 then zf_j <= '1'; else zf_j <= '0'; end if;
            if s_zone = '1' and s_zc = 1560 then zf_l0 <= '1'; else zf_l0 <= '0'; end if;

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

            -- LFO sine index: the ratio jobs write it (phase x ratio, below); the eye ripple
            -- (x2, a quarter wave per pair, running outward) is set at slot 21 before each lift job
            if j_on = '1' and j_slot = 21 and j_nx(5 downto 4) = "11" and j_nx(0) = '1' then
                v_m10 := s_lm(19 downto 10);
                s_sidx <= v_m10(8 downto 1) - (j_nx(2 downto 1) & "000000");
            end if;
            s_sval <= f_sin8(s_sidx);
            if zf_inc = '1' then f_inc <= unsigned(q_inc(14 downto 0)); end if;
            if zf_vw = '1' then f_vw48 <= resize(s_vw, 13) + 48; f_vwx <= resize(s_vw, 13) + resize(s_xs, 13) + 47; end if;
            if zf_ox = '1' then
                    case to_integer(s_xs) is                                   -- x steps; axis offset 62 + 3 xs (X at stage 4)
                        when 4      => f_oxinc <= to_unsigned(102, 8); f_x62 <= to_unsigned(74, 7);
                        when 3      => f_oxinc <= to_unsigned(77, 8);  f_x62 <= to_unsigned(71, 7);
                        when others => f_oxinc <= to_unsigned(51, 8);  f_x62 <= to_unsigned(68, 7);
                    end case;
            end if;

            -- serial multiplier P = a x b (a signed 27 bits, b unsigned 16 bits):
            -- 16 steps of P = (P + b0 a 2^16) / 2.  Zone jobs every 24 clocks from
            -- zc 15: ids 0 .. 45, then the eye table 48 .. 63 (Hl_i at 48 + 2i,
            -- yo_i at 49 + 2i); the per-line jobs are 46 / 47.  Each LFO is a ratio job
            -- (master phase x ratio -> sine index), a sine x depth job, then its use.
            -- job load: m_jid and the operand select codes are registered together
            -- (a_sel: the m_a source, b_sel: the m_b source, b_cst: the job's constant),
            -- so the operand fetch one clock later is a shallow mux on registered codes
            m_ld <= '0';
            v_ldj := '0'; v_id := j_nx;
            if zf_j = '1' then
                j_on <= '1'; j_slot <= to_unsigned(23, 5); j_nx <= (others => '0');
            elsif j_on = '1' then
                if j_slot = 23 then
                    j_slot <= (others => '0');
                    v_ldj := '1';
                    if j_nx = 63 then j_on <= '0';
                    elsif j_nx = 45 then j_nx <= to_unsigned(48, 6);
                    else j_nx <= j_nx + 1; end if;
                else
                    j_slot <= j_slot + 1;
                end if;
            end if;
            if l_step = 2 then v_ldj := '1'; v_id := to_unsigned(46, 6); end if;
            if l_step = 4 then v_ldj := '1'; v_id := to_unsigned(47, 6); end if;
            if v_ldj = '1' then
                m_ld <= '1'; m_jid <= v_id;
                a_sel <= to_unsigned(OA_ZERO, 5); b_sel <= to_unsigned(OB_CST, 4); b_cst <= (others => '0');
                if v_id(5 downto 4) = "11" then
                    if v_id(0) = '0' then                                 -- eye i height: Hlf x G_i, G_i = 1 + i sigma
                        a_sel <= to_unsigned(OA_HLF, 5);
                        if v_id(3 downto 1) = 0 then b_cst <= to_unsigned(4096, 16);
                        else b_sel <= to_unsigned(OB_G, 4); end if;
                    else                                                  -- eye i lift: the ripple wave x depth
                        a_sel <= to_unsigned(OA_SVAL, 5); b_sel <= to_unsigned(OB_DEP3, 4);
                    end if;
                else
                case to_integer(v_id) is
                    when 0 =>     -- pixels left of the lens axis: ceil((vw + 48) / xs)
                        a_sel <= to_unsigned(OA_VWX, 5);
                        case to_integer(s_xs) is
                            when 4 =>      b_cst <= to_unsigned(16384, 16);
                            when 3 =>      b_cst <= to_unsigned(21846, 16);
                            when others => b_cst <= to_unsigned(32768, 16);
                        end case;
                    when 1  => a_sel <= to_unsigned(OA_SPD, 5);  b_sel <= to_unsigned(OB_SPD, 4);    -- speed^2
                    when 2  => a_sel <= to_unsigned(OA_K2, 5);   b_sel <= to_unsigned(OB_K2, 4);     -- K2^2
                    when 3  => a_sel <= to_unsigned(OA_K2SQ, 5); b_sel <= to_unsigned(OB_K2, 4);     -- K2^3
                    when 4  => a_sel <= to_unsigned(OA_TMP, 5);  b_cst <= to_unsigned(638, 16);      -- kA
                    when 5  => a_sel <= to_unsigned(OA_P12, 5);  b_cst <= to_unsigned(628, 16);      -- Hl
                    when 6  => a_sel <= to_unsigned(OA_DEP, 5);  b_sel <= to_unsigned(OB_DEP, 4);    -- dep^2
                    when 7  => a_sel <= to_unsigned(OA_TMP, 5);  b_sel <= to_unsigned(OB_DEP, 4);    -- dep^3
                    when 8 | 43 => a_sel <= to_unsigned(OA_LM, 5); b_cst <= to_unsigned(1, 16);      -- breath / slide phase
                    when 11 => a_sel <= to_unsigned(OA_LM, 5);   b_cst <= to_unsigned(11, 16);       -- wedge phase
                    when 15 => a_sel <= to_unsigned(OA_LM, 5);   b_cst <= to_unsigned(9, 16);        -- floor phase
                    when 28 => a_sel <= to_unsigned(OA_LM, 5);   b_cst <= to_unsigned(6, 16);        -- colour phase
                    when 31 => a_sel <= to_unsigned(OA_LM, 5);   b_cst <= to_unsigned(14, 16);       -- sway phase
                    when 9 | 12 | 16 | 29 | 32 =>                                                    -- the LFO sine x depth
                               a_sel <= to_unsigned(OA_SVAL, 5); b_sel <= to_unsigned(OB_DEP, 4);
                    when 10 => a_sel <= to_unsigned(OA_TMP, 5);  b_sel <= to_unsigned(OB_HL, 4);     -- Hlf
                    when 13 => a_sel <= to_unsigned(OA_TMP, 5);  b_sel <= to_unsigned(OB_KA, 4);     -- kAf
                    when 14 => a_sel <= to_unsigned(OA_K3, 5);   b_cst <= to_unsigned(900, 16);      -- B0
                    when 17 | 30 => a_sel <= to_unsigned(OA_TMP, 5); b_cst <= to_unsigned(25, 16);   -- B0f / doff
                    when 18 => a_sel <= to_unsigned(OA_K3, 5);   b_cst <= to_unsigned(922, 16);      -- floor slope (Horner)
                    when 19 => a_sel <= to_unsigned(OA_TMP, 5);  b_sel <= to_unsigned(OB_K3, 4);
                    when 20 => a_sel <= to_unsigned(OA_P12, 5);  b_cst <= to_unsigned(464, 16);      -- halo width
                    when 21 => a_sel <= to_unsigned(OA_HS, 5);   b_cst <= to_unsigned(160, 16);
                    when 22 => a_sel <= to_unsigned(OA_HS, 5);   b_cst <= to_unsigned(130, 16);
                    when 23 => a_sel <= to_unsigned(OA_HS, 5);   b_cst <= to_unsigned(176, 16);
                    when 24 => a_sel <= to_unsigned(OA_HS, 5);   b_cst <= to_unsigned(72, 16);
                    when 25 => a_sel <= to_unsigned(OA_HS, 5);   b_cst <= to_unsigned(38, 16);
                    when 26 => a_sel <= to_unsigned(OA_HS, 5);   b_cst <= to_unsigned(30, 16);
                    when 27 => a_sel <= to_unsigned(OA_HS, 5);   b_cst <= to_unsigned(6, 16);
                    when 33 => a_sel <= to_unsigned(OA_TMP, 5);  b_cst <= to_unsigned(31, 16);       -- eye sway
                    when 34 => a_sel <= to_unsigned(OA_INC, 5);  b_sel <= to_unsigned(OB_XS, 4);
                    when 35 => a_sel <= to_unsigned(OA_VW48, 5); b_sel <= to_unsigned(OB_INC, 4);
                    when 36 => a_sel <= to_unsigned(OA_NPRE, 5); b_sel <= to_unsigned(OB_XS, 4);
                    when 37 => a_sel <= to_unsigned(OA_INC, 5);  b_sel <= to_unsigned(OB_R0, 4);
                    when 38 => a_sel <= to_unsigned(OA_YS, 5);   b_sel <= to_unsigned(OB_KAF, 4);
                    when 39 => a_sel <= to_unsigned(OA_C241, 5); b_sel <= to_unsigned(OB_KAF, 4);
                    when 40 => a_sel <= to_unsigned(OA_YS, 5);   b_sel <= to_unsigned(OB_KF, 4);
                    when 41 => a_sel <= to_unsigned(OA_B0F, 5);  b_sel <= to_unsigned(OB_KF, 4);
                    when 42 => a_sel <= to_unsigned(OA_YS, 5);   b_cst <= to_unsigned(61, 16);
                    when 44 => a_sel <= to_unsigned(OA_SVAL, 5); b_sel <= to_unsigned(OB_DEP3, 4);   -- slide
                    when 45 =>    -- K5 Spread sigma: -0.93 .. 0 (deadband 496 .. 527) .. +1.55 per pair
                        a_sel <= to_unsigned(OA_K5, 5);
                        if g_k5(12) = '0' then b_cst <= to_unsigned(476, 16); else b_cst <= to_unsigned(794, 16); end if;
                    when 46 => a_sel <= to_unsigned(OA_AV, 5);   b_sel <= to_unsigned(OB_AV, 4);
                    when 47 => a_sel <= to_unsigned(OA_AV2, 5);  b_sel <= to_unsigned(OB_KB, 4);
                    when others => null;
                end case;
                end if;
            end if;

            m_done <= '0';
            if m_ld = '1' then
                m_acc <= (others => '0'); m_i <= (others => '0'); m_run <= '1';
                v_k5 := signed(g_k5(12 downto 3)); v_k5(9) := not g_k5(12);       -- K5 - 512
                case to_integer(a_sel) is
                    when OA_VWX  => m_a <= signed(resize(f_vwx, 27));
                    when OA_SPD  => m_a <= signed(resize(s_spd, 27));
                    when OA_K2   => m_a <= signed(resize(g_k2(12 downto 3), 27));
                    when OA_K2SQ => m_a <= signed(resize(f_k2sq, 27));
                    when OA_P12  => m_a <= signed(resize(g_p12(12 downto 3), 27));
                    when OA_DEP  => m_a <= signed(resize(f_dep, 27));
                    when OA_TMP  => m_a <= resize(s_tmp, 27);
                    when OA_SVAL => m_a <= resize(s_sval, 27);
                    when OA_K3   => m_a <= signed(resize(g_k3(12 downto 3), 27));
                    when OA_HS   => m_a <= signed(resize(f_hs, 27));
                    when OA_INC  => m_a <= signed(resize(f_inc, 27));
                    when OA_VW48 => m_a <= signed(resize(f_vw48, 27));
                    when OA_NPRE => m_a <= signed(resize(f_npre, 27));
                    when OA_YS   => m_a <= signed(resize(s_ys, 27));
                    when OA_C241 => m_a <= to_signed(241152, 27);
                    when OA_B0F  => m_a <= signed(resize(f_B0f, 27));
                    when OA_K5   => m_a <= resize(v_k5, 27);
                    when OA_AV   => m_a <= signed(resize(a_av, 27));
                    when OA_AV2  => m_a <= signed(resize(l_av2, 27));
                    when OA_HLF  => m_a <= signed(resize(f_Hlf, 27));
                    when OA_LM   => m_a <= signed(resize(s_lm(19 downto 10), 27));
                    when others => m_a <= (others => '0');
                end case;
                case to_integer(b_sel) is
                    when OB_SPD  => m_b <= resize(s_spd, 16);
                    when OB_K2   => m_b <= resize(g_k2(12 downto 3), 16);
                    when OB_DEP  => m_b <= resize(f_dep, 16);
                    when OB_HL   => m_b <= resize(f_Hl, 16);
                    when OB_KA   => m_b <= resize(f_kA, 16);
                    when OB_INC  => m_b <= resize(f_inc, 16);
                    when OB_XS   => m_b <= resize(s_xs, 16);
                    when OB_R0   => m_b <= resize(f_r0, 16);
                    when OB_KAF  => m_b <= resize(f_kAf, 16);
                    when OB_DEP3 => m_b <= resize(f_dep3, 16);
                    when OB_AV   => m_b <= resize(a_av, 16);
                    when OB_KB   => m_b <= f_kb;
                    when OB_G    => m_b <= f_G;
                    when OB_K3   => m_b <= resize(g_k3(12 downto 3), 16);
                    when OB_KF   => m_b <= resize(f_kf, 16);
                    when others => m_b <= b_cst;
                end case;
                if m_jid(5 downto 4) = "11" and m_jid(0) = '0' then        -- G_(i+1) = G_i + sigma, >= 1/32
                    if m_jid(3 downto 1) = 0 then v_gs := to_signed(4096, 18) + resize(f_sig, 18);
                    else v_gs := signed(resize(f_G, 18)) + resize(f_sig, 18); end if;
                    if v_gs(17) = '1' or v_gs(16 downto 7) = 0 then f_G <= to_unsigned(128, 16);   -- <= 48469 above
                    else f_G <= unsigned(v_gs(15 downto 0)); end if;
                end if;
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
                -- lens phase at the left edge / the axis (jobs 35 / 37)
                v_ph := unsigned(v_r(26 downto 0)) + unsigned(shift_left(resize(f_lph, 27), 8));
                if m_jid(5 downto 4) = "11" and m_jid(0) = '0' then                              -- Hl_i, clamped
                    if v_r(26 downto 23) /= 0 then s_tmp <= to_signed(2047, 12);
                    else s_tmp <= signed('0' & v_r(22 downto 12)); end if;
                end if;
                case to_integer(m_jid) is
                    when 0  => f_npre <= unsigned(v_r(26 downto 16));
                    when 1  => f_dep <= unsigned(v_r(19 downto 11));                               -- dep = speed^2 / 2048: 128 at 50 %
                               s_linc <= resize(unsigned(v_r(19 downto 7)), 16);                   -- rate = speed^2 / 128: x4 at 100 %
                    when 2  => f_k2sq <= unsigned(v_r(19 downto 0));
                    when 3  => s_tmp <= signed('0' & v_r(29 downto 19));                           -- K2^3 / 2^19 (<= 2045)
                    when 4  => f_kA <= to_unsigned(82, 15) + unsigned(v_r(20 downto 6));          -- kA 0.02 .. 5.0, cubic
                    when 5  => f_Hl <= to_unsigned(12, 10) + unsigned(v_r(19 downto 10));
                    when 6  => s_tmp <= signed('0' & v_r(17 downto 7));                            -- dep^2 / 128 (<= 2040)
                    when 7  => f_dep3 <= unsigned(v_r(19 downto 7));                               -- dep^3: 128 at 50 %, 8144 at 100 %
                    when 8 | 11 | 15 | 28 | 31 => s_sidx <= unsigned(v_r(9 downto 2));            -- LFO phase x ratio
                    when 43 => s_sidx <= unsigned(v_r(9 downto 2)) + 64;                           -- slide: the breath + a quarter wave
                    when 9 | 12 | 16 | 29 | 32 => s_tmp <= resize(v_r(17 downto 7), 12);           -- sine x dep / 128
                    when 10 =>                                                                       -- +- 6 % at 50 %
                        v_e14 := signed(resize(f_Hl, 14)) + resize(v_r(21 downto 11), 14);
                        f_Hlf <= unsigned(v_e14(10 downto 0));
                    when 13 =>                                                                       -- +- 12 % at 50 %
                        v_e17 := signed(resize(f_kA, 17)) + resize(v_r(25 downto 10), 17);
                        f_kAf <= unsigned(v_e17(14 downto 0));
                    when 14 =>                                                                       -- Peak: apex 370 lines above the V's
                        if s_peak = '1' then f_B0 <= to_unsigned(1016, 11) - unsigned(v_r(20 downto 10));
                        else f_B0 <= to_unsigned(1386, 11) - unsigned(v_r(20 downto 10)); end if;
                    when 17 =>                                                                       -- +- 25 lines at 50 %
                        v_e14 := signed(resize(f_B0, 14)) + resize(v_r(17 downto 7), 14);
                        f_B0f <= unsigned(v_e14(10 downto 0));
                    when 18 => s_tmp <= to_signed(1936, 12) - signed('0' & v_r(19 downto 10));     -- 15.125 - 7.2 x (Q7)
                    when 19 => f_kf <= to_unsigned(1228, 11) - unsigned(v_r(20 downto 10));        -- 2 / floor slope (Q7): 9.6 .. 5.36 .. 1.67
                    when 20 => f_hs <= to_unsigned(38, 10) + unsigned(v_r(19 downto 10));          -- halo width on P12
                    when 21 => f_oALf <= unsigned(v_r(16 downto 8));
                    when 22 => f_oARf <= unsigned(v_r(16 downto 8));
                    when 23 => f_oB <= unsigned(v_r(16 downto 8));
                    when 24 => f_kb <= unsigned(v_r(15 downto 0));
                    when 25 => f_iAR <= unsigned(v_r(14 downto 8));
                    when 26 => f_iBL <= unsigned(v_r(13 downto 8));
                    when 27 => f_iBR <= unsigned(v_r(13 downto 8));
                    when 30 => f_doff <= v_r(14 downto 6);                                          -- +- 49 at 50 %
                    when 33 =>                                                                      -- eye sway, + half a period (Twin)
                        if s_twin = '1' then f_lph <= resize(v_r(16 downto 1), 17) + 32768;
                        else f_lph <= resize(v_r(16 downto 1), 17); end if;
                    when 34 => f_incp <= unsigned(v_r(16 downto 0));
                    when 35 => f_ph0 <= v_ph;
                    when 36 => f_r0 <= unsigned(v_r(2 downto 0)) - f_vw48(2 downto 0);
                    when 37 => f_phx <= v_ph;
                    when 38 => f_dCA <= v_r(31 downto 6);
                    when 39 => f_CA0 <= -v_r(33 downto 6);
                    when 40 => f_dCB <= unsigned(v_r(20 downto 4));
                    when 41 => f_CB0 <= v_r(21 downto 0) & "0000";
                    when 42 => f_oyinc <= unsigned(v_r(15 downto 8));
                    when 44 => f_slide <= v_r(20 downto 11);                                        -- +- 4 .. 253 px (Q.1 x2)
                    when 45 =>                                                                      -- Q12 per pair
                        if g_k5(12 downto 7) = "011111" or g_k5(12 downto 7) = "100000" then f_sig <= (others => '0');
                        else f_sig <= v_r(19 downto 6); end if;
                    when 46 => l_av2 <= unsigned(v_r(19 downto 0));
                    when 47 => a_oAL <= resize(f_oALf, 11) + unsigned(v_r(34 downto 24));
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
                l_CA <= l_CA + resize(f_dCA, 28);
                l_CB <= l_CB - signed(resize(f_dCB, 26));
                l_step <= to_unsigned(1, 3);
                if b_rng(2 downto 0) = "000" and a_jx /= 2 then a_jx <= a_jx + 1;
                elsif b_rng(2 downto 0) = "001" and a_jx /= -2 then a_jx <= a_jx - 1; end if;
            elsif l_step = 1 then
                v_y := signed(resize(l_yq8, 20)) - 132608;
                if v_y < 0 then v_y := -v_y; end if;
                a_av <= resize(unsigned(v_y(19 downto 8)), 10);
                if l_yq8 < 132608 then a_vy <= -signed(resize(v_y(19 downto 8), 12));
                else a_vy <= signed(resize(v_y(19 downto 8), 12)); end if;
                a_axo <= resize(a_jx, 12) + resize(f_slide, 12) - signed(resize(f_x62, 12));
                -- the wide wedge / steep floor overrun the field widths: saturate (|X| < 2600 decides)
                v_r := resize(shift_right(l_CA, 13), 44);
                if v_r > 4095 then a_cA <= to_signed(4095, 13);
                elsif v_r < -4096 then a_cA <= to_signed(-4096, 13);
                else a_cA <= v_r(12 downto 0); end if;
                v_r := resize(shift_right(l_CB, 11), 44);
                if v_r > 8191 then a_cB <= to_signed(8191, 14);
                elsif v_r < -8192 then a_cB <= to_signed(-8192, 14);
                else a_cB <= v_r(13 downto 0); end if;
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

            s_field0 <= data_in.field_n;                                    -- field_n is tied high on every input
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
    -- Beam: virtual X (Q.1, centred), folded lens-phase DDA (27 bits: the top
    -- three count eye periods, for the eye index)
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
            else
                b_x <= b_x + signed(resize(s_xs, 13));                     -- runs on, so stage 4 can take X - 3 xs
            end if;
            if g_start = '1' then
                null;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                if b_cnt >= 2 then
                    b_ph <= b_ph - resize(f_incp, 27);
                elsif b_cnt = 1 then
                    b_ph <= f_phx;
                else
                    b_ph <= b_ph + resize(f_incp, 27);
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
        variable v_dy : signed(11 downto 0);
        variable v_ov : std_logic;
    begin
        if rising_edge(clk) then
            -- 1: shape-ROM address (bank, folded phase), eye index (pairs out from the
            --    centre: floor(phase + 1/2) mod 8), A/B
            --    axis offset (jitter + slide), line-buffer read
            if b_ph(23) = '1' then s_ls_ra <= s_diam & not b_ph(22 downto 14); else s_ls_ra <= s_diam & b_ph(22 downto 14); end if;
            s_eye_ra <= b_ph(26 downto 24) + ("00" & b_ph(23));
            p1_lf <= b_ph(23) xor b_xl;
            s_lb_ra <= b_px(10 downto 1);

            -- 2
            d_lf(2) <= p1_lf;

            -- 3: ROM / eye table / video fabric copies
            p3_s <= signed(q_ls(15 downto 6));
            p3_hl <= unsigned(q_eye(31 downto 21));
            p3_yo <= signed(q_eye(20 downto 11));
            d_c(3) <= unsigned(q_ls(5 downto 0));
            p3_src <= signed(resize(unsigned(s_lb_q), 9)) - 110;
            d_par(3) <= s_lb_q(7) xor s_lb_q(6) xor s_lb_q(5);

            -- 4: h = Hl_i * s as two partial products; Y - 518 - yo_i; source x depth
            p4_plo <= resize(signed('0' & p3_hl) * signed('0' & p3_s(4 downto 0)), 17);
            p4_phi <= resize(signed('0' & p3_hl) * p3_s(9 downto 5), 17);
            p4_dy <= a_vy - resize(p3_yo, 12);
            p4_xa <= resize(b_x, 14) + resize(a_axo, 14);                   -- X of this pixel (b_x is 3 xs on)
            p4_slo <= resize(p3_src * signed('0' & f_depth(3 downto 0)), 14);
            p4_shi <= resize(p3_src * signed('0' & f_depth(7 downto 4)), 14);

            -- 5
            v_s := shift_left(resize(p4_phi, 22), 5) + resize(p4_plo, 22);
            p5_h <= resize(shift_right(v_s, 9), 12);
            if p4_dy < 0 then v_dy := -p4_dy; else v_dy := p4_dy; end if;
            p5_av <= unsigned(v_dy(9 downto 0));                                       -- |dy| <= 815
            -- |X - axA| and side (left of the axis)
            if p4_xa < 0 then d_rx5 <= unsigned(resize(-p4_xa, 13)); d_sg(5) <= '1';
            else d_rx5 <= unsigned(resize(p4_xa, 13)); d_sg(5) <= '0'; end if;
            p5_sm <= resize(shift_right(shift_left(resize(p4_shi, 18), 4) + resize(p4_slo, 18), 8), 9);

            -- 6: eye margin (+ source), wedge and floor fields, D shift-adds
            p6_m <= resize(p5_h, 13) - signed(resize(p5_av, 13)) + resize(shift_right(p5_sm, 2), 13);
            p6_sme <= shift_right(p5_sm, 2) + shift_right(p5_sm, 3);
            p6_smD <= shift_left(resize(p5_sm, 12), 2) + resize(shift_right(p5_sm, 1), 12) + resize(shift_right(p5_sm, 2), 12);
            p6_eA <= signed(resize(d_rx5, 14)) + resize(a_cA, 14);
            if s_peak = '1' then p6_eB <= resize(a_cB, 15) + signed(resize(d_rx5, 15));
            else p6_eB <= resize(a_cB, 15) - signed(resize(d_rx5, 15)); end if;
            p6_kda <= resize(d_rx5(12 downto 1), 12) + resize(d_rx5(12 downto 5), 12);
            p6_kdb <= resize(d_rx5(12 downto 7), 6);

            -- 7: margin x perpendicular factor; the source bends the wedge, floor and D
            p7_mc <= resize(p6_m * signed('0' & d_c(6)), 20);
            p7_eA <= p6_eA + resize(p6_sme, 14);
            p7_eB <= p6_eB + resize(p6_sme, 15);
            p7_D <= (resize(a_dl, 13) + resize(p6_smD, 13)) - (signed(resize(p6_kda, 13)) - signed(resize(p6_kdb, 13)));
            for k in 4 to 6 loop d_c(k) <= d_c(k - 1); end loop;
            for k in 6 to 8 loop d_sg(k) <= d_sg(k - 1); end loop;
            for k in 4 to 10 loop d_par(k) <= d_par(k - 1); end loop;
            for k in 3 to 10 loop d_lf(k) <= d_lf(k - 1); end loop;

            -- 8: keys, edge distances, palette index, perpendicular margin
            v_mp := shift_right(p7_mc, 6);
            if v_mp < -32 then d_mp(8) <= to_signed(-32, 7);
            elsif v_mp > 63 then d_mp(8) <= to_signed(63, 7);
            else d_mp(8) <= resize(v_mp, 7); end if;
            if p7_eA < 0 or p7_eB < 0 then d_ab(8) <= '1'; else d_ab(8) <= '0'; end if;
            if p7_eA < 0 and p7_eB < 0 then d_ov(8) <= '1'; else d_ov(8) <= '0'; end if;
            -- signed horizontal distances to the wedge / floor edges, clamped to
            -- the profiles' reach (+-32 px = +-64 in Q.1)
            if d_sg(7) = '1' then v_hd := -resize(p7_eA, 15); else v_hd := resize(p7_eA, 15); end if;
            if a_aex = '0' or v_hd > 63 then p8_hdA <= to_signed(63, 7);
            elsif v_hd < -64 then p8_hdA <= to_signed(-64, 7);
            else p8_hdA <= resize(v_hd, 7); end if;
            if (d_sg(7) xor s_peak) = '1' then v_hd := p7_eB; else v_hd := -p7_eB; end if;
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
            else
                p9_a1 <= signed(resize(f_iAR, 14)) - resize(p8_eA0, 14);
                p9_a2 <= resize(p8_eA0, 14) - signed(resize(a_oAR, 14));
            end if;
            if (d_sg(8) xor s_peak) = '1' then p9_b1 <= signed(resize(f_iBL, 14)) - resize(v_ls, 14);
            else p9_b1 <= signed(resize(f_iBR, 14)) - resize(v_ls, 14); end if;
            p9_b2 <= resize(v_ls, 14) - signed(resize(f_oB, 14));
            d_mp(9) <= d_mp(8); d_idx(9) <= d_idx(8); d_ab(9) <= d_ab(8); d_ov(9) <= d_ov(8);

            -- 10: edge index (nearer edge, px); halo maxima
            if p9_ahA <= p9_ahB then v_h7 := p9_hdA; else v_h7 := p9_hdB; end if;
            p10_hdi <= resize(shift_right(v_h7, 1), 6);
            if p9_a1 > p9_a2 then p10_hA <= p9_a1; else p10_hA <= p9_a2; end if;
            if p9_b1 > p9_b2 then p10_hB <= p9_b1; else p10_hB <= p9_b2; end if;
            d_mp(10) <= d_mp(9); d_idx(10) <= d_idx(9); d_ab(10) <= d_ab(9); d_ov(10) <= d_ov(9);

            -- 11: halo class, profiles, ROM addresses {class, D index}
            if p10_hA < p10_hB then v_d := p10_hA; else v_d := p10_hB; end if;
            if v_d < 0 then p11_halo <= '1'; else p11_halo <= '0'; end if;
            -- where the wedge and floor overlap, an XOR mix: purple / ground outside the eyes,
            -- red / light inside, in bands of the gradient flipped by the picture's luma contours
            v_ov := d_idx(10)(5) xor d_par(10);
            if d_ov(10) = '1' then
                if v_ov = '1' then v_cls := "10"; else v_cls := "01"; end if;
            elsif d_ab(10) = '1' then v_cls := "10";
            elsif v_d < -5 then v_cls := "01";
            elsif v_d < 0 then v_cls := "11";
            else v_cls := "00"; end if;
            if d_ov(10) = '1' then s_co_ra <= v_ov & d_idx(10); else s_co_ra <= d_ab(10) & d_idx(10); end if;
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
            if p11_halo = '1' and d_ab(11) = '0' then
                v_w := f_max5(p11_redol, p11_dip);
                if p11_rim /= 0 then p12_isr <= '1'; end if;
            else
                v_w := p11_redol;
            end if;
            if d_ab(11) = '0' and p11_halo = '0' then v_w := f_max5(v_w, p11_dd); end if;
            if p11_al >= 4 then
                if p11_halo = '1' and d_ab(11) = '0' and p11_rim /= 0 then
                    p12_w2 <= p11_rim;
                else
                    p12_w2 <= v_w;
                end if;
                p12_tail <= (others => '0');
            else
                p12_w2 <= p11_w; p12_tail <= p11_tail;
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

            -- 20: Ember = chroma complement; chroma neutral off the generated avid, so the 8-sums below
            -- resync in every blanking interval
            p20_y <= p19_y;
            s_gav <= s_gav(17 downto 0) & g_avid;
            if s_gav(18) = '0' then p20_u <= to_unsigned(512, 10); p20_v <= to_unsigned(512, 10);
            elsif s_ember = '1' then p20_u <= not p19_u; p20_v <= not p19_v;
            else p20_u <= p19_u; p20_v <= p19_v; end if;
            s_bxh <= s_bxh(6 downto 0) & s_gav(18);

            -- 21..26: output conditioning (centre at stage 25 -> 26); chroma as two
            -- adjacent 8-sums of 8-bit chroma, the 8-back tap from the ring.  The running 8-sums snap back
            -- to neutral once 8 blank samples are in, so a glitched sample can't offset the chroma for good.
            if s_bxh = "00000000" then
                s_boxu <= to_unsigned(1024, 11); s_boxv <= to_unsigned(1024, 11);
            else
                s_boxu <= s_boxu + resize(p20_u(9 downto 2), 11) - resize(unsigned(s_crq(15 downto 8)), 11);   -- s_crq = p20 of 8 clocks ago
                s_boxv <= s_boxv + resize(p20_v(9 downto 2), 11) - resize(unsigned(s_crq(7 downto 0)), 11);
            end if;
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u26 <= v_sc(11 downto 2);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v26 <= v_sc(11 downto 2);
            s_y21 <= p20_y; s_y22 <= s_y21; s_y23 <= s_y22; s_y24 <= s_y23;
            s_y25 <= s_y24; s_y26x <= s_y25;
            v_sy := resize(s_y24, 12) + shift_left(resize(s_y25, 12), 1) + resize(s_y26x, 12);
            s_yf26 <= v_sy(11 downto 2);
            -- 27: luma toward black (~777 white); blanking gate
            if s_yf26 > 64 then v_dl := s_yf26 - 64; else v_dl := (others => '0'); end if;
            if s_srq(1) = '1' then                                                       -- tap C_LAT - 3
                s_y27 <= s_yf26 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u27 <= s_u26; s_v27 <= s_v26;
            else
                s_y27 <= to_unsigned(64, 10); s_u27 <= to_unsigned(512, 10); s_v27 <= to_unsigned(512, 10);
            end if;
            -- 28
            s_y28 <= s_y27; s_u28 <= s_u27; s_v28 <= s_v27;
        end if;
    end process p_pix;

    -- eye table write: {Hl_i, yo_i (+-253 lines)} as the yo_i job completes
    e_we <= '1' when m_done = '1' and m_jid(5 downto 4) = "11" and m_jid(0) = '1' else '0';
    e_wa <= m_jid(3 downto 1);
    e_wd <= std_logic_vector(s_tmp(10 downto 0)) & std_logic_vector(m_acc(21 downto 12)) & "00000000000";

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(s_u28);
    data_out.v       <= std_logic_vector(s_v28);
    data_out.hsync_n <= s_sr2(3);
    data_out.vsync_n <= s_sr2(2);
    data_out.field_n <= s_field0;
    data_out.avid    <= s_sr2(1);

end architecture facet;
