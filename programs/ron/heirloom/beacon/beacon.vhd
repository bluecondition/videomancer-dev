-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Beacon v0.6.1 -- Heirloom #26, rebuilt from WIN_20230424_23_53_49 (phase 2: wider
-- ranges, the flame centred on the tower, Dusk squeeze, Speed swell, more video; v0.6.1:
-- a blue flame and beam against amber bars, the camera warped into the flame).
--
-- The photo is two layers keyed together:
--   * a white layer (grey-green in the mid tones): a candle flame (an
--     asymmetric tent, 2.4 / 2.0 per px, on a peak profile with a pointed
--     tip), a neck, a bell whose outline is a hyperbola (bright line outside,
--     dark line inside) widening into a straight-edged cone shaded by a
--     horizontal V (the vertical shading bands), and white ramps at the
--     screen sides whose reach follows the vertical band cycle;
--   * a blue key over it: rows of class A (blue bands / rows) show blue in the
--     wings (x < 880, x >= 1648); rows of class B (black rows) show blue only
--     in the low part of a 391-px horizontal pulse wave.  The blue fades out
--     around the beacon (a fitted 32 x 31 knot fade per class), rises with an
--     RC lag after each cut and rings with a bright pre-shoot before it, and
--     the fade edges are ragged per line.  The vertical band cycle (742
--     lines: blue band, a 27-line stripe zone, a thin blue band, the black
--     band) restarts after a 70-line hold at the top of the frame.
--
-- The program:
--   * a micro engine (program in C_UROM, assembled by masm.py; a 24-bit
--     accumulator, a register file in block RAM, one serial multiplier):
--     per field the control glides and curves, the column phase, the flame
--     gain / height, the LFOs and the band roll; per line (kicked at each
--     line start, computing the NEXT line) the band class, the fade knots of
--     the line (a knot loop writes them to a ping-pong knot RAM), the flame
--     peak and slopes, the bell half-widths and profiles, the edge-glow
--     offset and the ragged-edge jitter.  The line values are staged in a
--     block RAM and copied into the line registers in the blanking;
--   * per pixel: layout / shape / column / flame DDAs, the fade lerp, the
--     key logic with its x-order RC lag, the bell profile (Gaussian line,
--     ramp interior), the colour ROMs and the keyed mix.
--
--   K1  Bars     vertical band-cycle frequency (stripe density), x0.25 .. x4
--   K2  Columns  horizontal column period (the blue/black checker columns), 1106 .. 138 px
--   K3  Dusk     below the centre the blue closes in on the tower (a lift outside a
--                squeeze radius that shrinks onto the tower outline); above it the dark deepens
--   K4  Speed    bipolar: the bars roll and swell in a travelling wave whose depth
--                follows |speed|; the flame flickers faster
--   K5  Beam     the bell / cone spread, x0.35 .. x2.83 (the wing cone follows it)
--   K6  Halo     reach of the white ramps at the screen sides; at 100 % they close
--                in on the tower
--   S7  Bands    Stripes (the photo) / Solid bands
--   S8  Bell     Solid / Outline (the bell drawn as its outline only)
--   S9  Palette  Blue (a blue flame, beam and halo against amber bars) / Ember (red bars, warm white)
--   S10 Tear     ragged fade edges and tracking lines on / clean
--   S11 Source   Video (the camera fills the beam, shows through the dark outside the
--                tower, and a 64 x 64 thumbnail of it is warped into the flame) / Osc
--   P12 Flame    0 = a small ember .. 40 % = the photo .. 100 % a towering
--                flame that burns through the bars
--
-- Geometry in virtual 1080-line units: shapes sized to the height, the
-- column / window layout spanning the width.  Generated avid; output
-- conditioning (chroma 9-tap, luma [1 2 1] and 13/16) before the blanking
-- gate.  Latency C_LAT in every mode.  Colours in the hardware convention
-- (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.beacon_pkg.all;
use work.all;

architecture beacon of program_top is
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
    -- flame texture capture: rows advance 65536 / (active lines per field) per line
    function f_trow(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(269, 9);
            when 4          => return to_unsigned(136, 9);
            when 8          => return to_unsigned(227, 9);
            when 12         => return to_unsigned(113, 9);
            when 5 | 6 | 14 => return to_unsigned(91, 9);
            when 1 | 2 | 10 => return to_unsigned(121, 9);
            when others     => return to_unsigned(60, 9);
        end case;
    end function;
    function f_clip8(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else return unsigned(v(7 downto 0)); end if;
    end function;
    function f_umax(a, b : unsigned(7 downto 0)) return unsigned is
    begin
        if a > b then return a; else return b; end if;
    end function;

    ---------------------------------------------------------------------------
    -- memories
    ---------------------------------------------------------------------------
    signal s_fe_a, s_fo_a : unsigned(9 downto 0) := (others => '0');
    signal s_fe_q, s_fo_q : std_logic_vector(7 downto 0) := (others => '0');
    signal s_vt_a : unsigned(8 downto 0) := (others => '0');
    signal s_vt_q : std_logic_vector(7 downto 0) := (others => '0');
    signal s_pk_a : unsigned(7 downto 0) := (others => '0');
    signal s_pk_q : std_logic_vector(15 downto 0) := (others => '0');
    signal s_y2_a : unsigned(7 downto 0) := (others => '0');
    signal s_y2_q : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lc_a, s_bc_a : unsigned(7 downto 0) := (others => '0');
    signal s_lc_q, s_bc_q : std_logic_vector(31 downto 0) := (others => '0');
    -- knot RAM: {bank, knot} -> {K(i), K(i+1)}
    type t_knot is array (0 to 63) of std_logic_vector(15 downto 0);
    signal ram_kn : t_knot := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of ram_kn : signal is true;
    signal s_kw_a, s_kr_a : unsigned(5 downto 0) := (others => '0');
    signal s_kw_d : std_logic_vector(15 downto 0) := (others => '0');
    signal s_kw_e : std_logic := '0';
    signal s_kr_q : std_logic_vector(15 downto 0) := (others => '0');
    -- camera luma ring
    type t_ring is array (0 to 255) of std_logic_vector(7 downto 0);
    signal ram_lu : t_ring := (others => x"10");
    attribute no_rw_check of ram_lu : signal is true;
    signal s_lu_wa, s_lu_ra : unsigned(7 downto 0) := (others => '0');
    signal s_lu_wd : std_logic_vector(7 downto 0) := x"10";
    signal s_lu_we : std_logic := '0';
    signal s_lu_q : std_logic_vector(7 downto 0) := x"10";
    signal s_wcol, s_rx : unsigned(7 downto 0) := (others => '0');
    -- flame texture: a 64 x 64 x 4-bit thumbnail of the camera ({row, column}), written by the capture,
    -- read through the flame's warp (rows never written and read at once except at the very top)
    type t_tex is array (0 to 4095) of std_logic_vector(3 downto 0);
    signal ram_tx : t_tex := (others => "0000");
    attribute no_rw_check of ram_tx : signal is true;
    signal s_tx_a : unsigned(11 downto 0) := (others => '0');
    signal s_tw_a : unsigned(11 downto 0);
    signal s_tw_d : std_logic_vector(3 downto 0);
    signal s_tw_e : std_logic;
    signal s_tx_q : std_logic_vector(3 downto 0) := (others => '0');
    signal s_tcc : unsigned(15 downto 0) := (others => '0');       -- capture column accumulator
    signal s_tcs : unsigned(6 downto 0) := to_unsigned(34, 7);     -- its step: 65536 / line width
    signal s_tcd : unsigned(5 downto 0) := (others => '1');        -- the previous pixel's column
    signal s_tbs : unsigned(7 downto 0) := (others => '0');        -- 4-pixel sum (6-bit samples)
    signal s_tsum : unsigned(7 downto 0);
    signal s_tbc : unsigned(1 downto 0) := (others => '0');
    signal s_trc : unsigned(16 downto 0) := (others => '0');       -- capture row accumulator
    signal s_trs : unsigned(8 downto 0) := to_unsigned(60, 9);
    signal s_tav : std_logic := '0';

    ---------------------------------------------------------------------------
    -- timing, controls, per-field constants
    ---------------------------------------------------------------------------
    signal s_vx : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_sy : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_stripes, s_outline, s_ember, s_tear, s_osc : std_logic := '0';
    signal s_sx, s_sv : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal s_xv0 : signed(16 downto 0) := to_signed(-16480, 17);   -- shape x origin: the tower (flame and bell) axis
    signal s_xdp : unsigned(7 downto 0) := to_unsigned(146, 8);
    signal s_hinc : unsigned(23 downto 0) := to_unsigned(42912, 24);
    signal s_ph0 : unsigned(23 downto 0) := (others => '0');
    signal s_ts1, s_ts2 : unsigned(9 downto 0) := (others => '1');
    signal s_d3n : signed(8 downto 0) := (others => '0');           -- Dusk: darkening (<= 0)
    signal s_d3p : unsigned(7 downto 0) := (others => '0');         -- Dusk: lift outside the squeeze radius
    signal s_xec : unsigned(10 downto 0) := to_unsigned(930, 11);   -- Halo: the ramps' centre
    signal s_tl : unsigned(5 downto 0) := to_unsigned(36, 6);

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
    -- line engine
    ---------------------------------------------------------------------------
    signal e_kick, e_fkick, e_wb, e_last, s_rbank, e_done : std_logic := '0';
    signal e_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal e_a : std_logic := '0';
    signal e_krow : unsigned(4 downto 0) := (others => '0');
    signal e_t5 : unsigned(4 downto 0) := (others => '0');
    -- micro engine: F (uROM) / D (RF address) / R (RF, IN) / E (execute)
    signal u_run, u_fva, u_fvq, u_irv, u_irv2 : std_logic := '0';
    signal u_pc, u_ua, u_ret : unsigned(8 downto 0) := (others => '0');
    signal u_q, u_ir, u_ir2 : std_logic_vector(31 downto 0) := (others => '0');
    type t_rf is array (0 to 255) of std_logic_vector(31 downto 0);
    signal ram_rf : t_rf := t_rf(C_RFINIT);
    signal u_ix : unsigned(7 downto 0) := (others => '0');
    attribute no_rw_check of ram_rf : signal is true;
    signal u_ra : unsigned(7 downto 0);                                   -- combinational (from the uROM word)
    signal u_rq : std_logic_vector(31 downto 0) := (others => '0');       -- RF block RAM out (read in D, into u_b in R)
    signal u_b : signed(23 downto 0) := (others => '0');                  -- the B operand, selected in R
    signal u_acc : signed(23 downto 0) := (others => '0');
    -- multiply / shift unit: mode 0 idle, 1 multiply steps, 2 barrel, 3 sign
    signal u_mode : unsigned(1 downto 0) := (others => '0');
    signal u_adv : std_logic := '1';            -- = (u_mode = "00"), registered: the pipeline advances
    -- OUT, one clock after E: valid, port, data
    signal o_v : std_logic := '0';
    signal o_p : unsigned(5 downto 0) := (others => '0');
    signal o_d : signed(23 downto 0) := (others => '0');
    signal u_mcnt : unsigned(2 downto 0) := (others => '0');
    signal u_ma : unsigned(15 downto 0) := (others => '0');
    signal u_ma3 : unsigned(17 downto 0) := (others => '0');
    signal u_mh : unsigned(16 downto 0) := (others => '0');
    signal u_ml : unsigned(15 downto 0) := (others => '0');
    signal u_mneg : std_logic := '0';
    signal u_mx, u_mxt : std_logic := '0';      -- MAX / MIN: select in the next clock (u_mxt = take B)
    signal u_sh : unsigned(4 downto 0) := (others => '0');
    signal u_bin : unsigned(31 downto 0) := (others => '0');
    signal u_bx : std_logic := '0';
    -- the line values: OUT writes them to a staging RAM (address = port); the copier moves them into the
    -- current line's (l_) registers in the blanking after the program has halted
    type t_stg is array (0 to 63) of std_logic_vector(31 downto 0);
    signal ram_st : t_stg := (others => (others => '0'));
    attribute no_rw_check of ram_st : signal is true;
    signal s_st_q : std_logic_vector(31 downto 0) := (others => '0');
    signal e_pend, cp_run, cp_v : std_logic := '0';
    signal cp_i, cp_k : unsigned(5 downto 0) := (others => '0');
    signal cp_idle : unsigned(4 downto 0) := (others => '0');
    signal n_a, l_a, l_bell, l_wingy, l_cap, l_tev : std_logic := '0';
    signal l_tx0 : unsigned(11 downto 0) := (others => '0');
    signal l_tu0 : unsigned(15 downto 0) := (others => '0');         -- flame texture: u at the line start (mod 2^16)
    signal l_tdu : unsigned(13 downto 0) := (others => '0');         -- its step per pixel
    signal l_tv : unsigned(5 downto 0) := (others => '0');           -- the line's texel row
    signal l_j : signed(7 downto 0) := (others => '0');
    signal l_nl : signed(3 downto 0) := (others => '0');
    signal l_sli : unsigned(11 downto 0) := (others => '0');
    signal l_rsq : unsigned(11 downto 0) := (others => '1');        -- Dusk squeeze radius (tower half-width + margin)
    signal l_swl : signed(8 downto 0) := (others => '0');
    signal l_lf0 : signed(22 downto 0) := (others => '0');
    signal l_hw, l_hww : unsigned(9 downto 0) := (others => '0');
    signal l_iy, l_lp, l_vr : unsigned(5 downto 0) := (others => '0');
    signal l_dl : unsigned(5 downto 0) := (others => '0');
    signal l_hcl : signed(6 downto 0) := (others => '0');
    signal l_e0 : signed(12 downto 0) := (others => '0');
    -- knot loop
    signal c_run : std_logic := '0';
    signal c_i : unsigned(4 downto 0) := (others => '0');
    signal c_v : std_logic_vector(1 to 6) := (others => '0');
    signal c_par : std_logic_vector(1 to 3) := (others => '0');
    type t_u5 is array (natural range <>) of unsigned(4 downto 0);
    signal c_wi : unsigned(4 downto 0) := (others => '0');
    signal c2_e, c2_o : unsigned(7 downto 0) := (others => '0');
    signal c3_a : unsigned(7 downto 0) := (others => '0');
    signal c3_d : signed(8 downto 0) := (others => '0');
    signal c4_a : unsigned(7 downto 0) := (others => '0');
    signal c4_p : signed(14 downto 0) := (others => '0');
    signal c5_k, c_prev : unsigned(7 downto 0) := (others => '0');
    signal c_tail : std_logic := '0';

    ---------------------------------------------------------------------------
    -- pixel path
    ---------------------------------------------------------------------------
    signal s_xs : signed(16 downto 0) := (others => '0');
    signal s_xv : signed(16 downto 0) := (others => '0');
    signal s_ph : unsigned(23 downto 0) := (others => '0');
    signal s_lf : signed(22 downto 0) := (others => '0');
    signal s_tu : unsigned(15 downto 0) := (others => '0');          -- flame texture u (256 per texel, 8192 at the axis)
    signal p1_tc : unsigned(5 downto 0) := (others => '0');
    signal p4_tex : unsigned(3 downto 0) := (others => '0');
    signal s_xd, s_xd0 : signed(16 downto 0) := (others => '0');    -- xv - SWL (its sign picks the flame slope)
    signal s_nsri : signed(12 downto 0) := (others => '0');
    signal p1_xs : unsigned(11 downto 0) := (others => '0');
    signal p1_xv, p1_xj : signed(12 downto 0) := (others => '0');
    signal p1_lf : signed(14 downto 0) := (others => '0');
    signal p1_hl : std_logic := '0';
    signal p1_spb : unsigned(1 downto 0) := (others => '0');
    signal p2_kf : unsigned(5 downto 0) := (others => '0');
    signal p2_xb, p2_xd, p2_ae : signed(12 downto 0) := (others => '0');
    signal p2_flm : unsigned(7 downto 0) := (others => '0');
    signal p2_brn : unsigned(2 downto 0) := (others => '0');
    signal p3_kf : unsigned(5 downto 0) := (others => '0');
    signal p3_adb, p3_adx, p3_ae : unsigned(11 downto 0) := (others => '0');
    signal p3_flm : unsigned(7 downto 0) := (others => '0');
    signal p4_k0, p4_k1 : unsigned(7 downto 0) := (others => '0');
    signal p4_kf : unsigned(5 downto 0) := (others => '0');
    signal p4_tt, p4_dw, p4_hcl, p4_c5 : signed(12 downto 0) := (others => '0');
    signal s_hw5 : signed(12 downto 0) := (others => '0');          -- HW + 5 (static over the line)
    signal p4_d8 : unsigned(7 downto 0) := (others => '0');
    signal p4_dr : signed(12 downto 0) := (others => '0');          -- |x - axis| - squeeze radius
    signal p4_egm : unsigned(11 downto 0) := (others => '0');
    signal p4_flm : unsigned(7 downto 0) := (others => '0');
    signal p5_k0 : unsigned(7 downto 0) := (others => '0');
    signal p5_df : signed(8 downto 0) := (others => '0');
    signal p5_kf : unsigned(5 downto 0) := (others => '0');
    signal p5_att : unsigned(7 downto 0) := (others => '0');
    signal p5_ttp : std_logic := '0';
    signal p5_r8a : signed(12 downto 0) := (others => '0');
    signal p5_cc : unsigned(7 downto 0) := (others => '0');
    signal p5_wm, p5_eg, p5_flm, p5_hc, p5_dr8 : unsigned(7 downto 0) := (others => '0');
    signal p6_k0 : unsigned(7 downto 0) := (others => '0');
    signal p6_pr : signed(15 downto 0) := (others => '0');
    signal p6_gi : unsigned(5 downto 0) := (others => '0');
    signal p6_r, p6_ka, p6_wa, p6_lx, p6_lift : unsigned(7 downto 0) := (others => '0');
    signal p7_f8 : signed(9 downto 0) := (others => '0');
    signal p7_gau, p7_ka, p7_wa : unsigned(7 downto 0) := (others => '0');
    signal p7_iyr : unsigned(15 downto 0) := (others => '0');
    signal p7_ycam : unsigned(7 downto 0) := x"10";
    signal p7_dl : signed(9 downto 0) := (others => '0');           -- the Dusk offset of the pixel
    signal p8_fd, p8_line, p8_s8, p8_int1, p8_ka, p8_wa, p8_bg : unsigned(7 downto 0) := (others => '0');
    signal p9_kraw, p9_int, p9_line, p9_wing : unsigned(7 downto 0) := (others => '0');
    signal p10_kk : unsigned(11 downto 0) := (others => '0');
    signal p10_bw : unsigned(7 downto 0) := (others => '0');
    signal p11_k9 : unsigned(8 downto 0) := (others => '0');
    signal p11_l, p12_l : unsigned(7 downto 0) := (others => '0');
    signal p12_k9 : unsigned(8 downto 0) := (others => '0');
    signal p13_kb : unsigned(8 downto 0) := (others => '0');
    signal p13_l : unsigned(7 downto 0) := (others => '0');
    -- late flags out of the chroma ring's spare bits (k -> k + 7, then one fabric register)
    signal r10_sp : std_logic_vector(1 downto 0) := (others => '0');
    signal r10_op : std_logic := '0';
    signal r11_lc, r11_tk : std_logic := '0';
    signal r12_brn : unsigned(2 downto 0) := (others => '0');
    signal p14_km, p14_l, p15_km, p15_l, p16_km, p16_l, p17_l : unsigned(7 downto 0) := (others => '0');
    signal p17_inv, p18_inv : unsigned(8 downto 0) := (others => '0');
    signal p17_ly, p18_ly : unsigned(9 downto 0) := (others => '0');
    signal p17_lu, p17_lv, p17_by, p17_bu, p17_bv : signed(10 downto 0) := (others => '0');
    signal p18_by, p18_bu, p18_bv : signed(10 downto 0) := (others => '0');
    signal p18_lu, p18_lv : signed(7 downto 0) := (others => '0');       -- white-layer chroma / 2
    signal p19_py : unsigned(18 downto 0) := (others => '0');
    signal p19_pu, p19_pv : signed(13 downto 0) := (others => '0');
    signal p19_by, p19_bu, p19_bv : signed(10 downto 0) := (others => '0');
    -- per-pixel values riding the pipe (index = stage)
    type t_flags is array (natural range <>) of std_logic_vector(11 downto 0);
    signal p_fl : t_flags(2 to 9) := (others => (others => '0'));
    type t_u8a is array (natural range <>) of std_logic_vector(7 downto 0);
    signal p_lx : t_u8a(7 to 10) := (others => (others => '0'));
    signal p_d8 : t_u8a(5 to 8) := (others => (others => '0'));
    type t_u3a is array (natural range <>) of std_logic_vector(2 downto 0);
    signal p_brn : t_u3a(3 to 4) := (others => (others => '0'));
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_cring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal ram_cr : t_cring := (others => x"00080200");
    attribute no_rw_check of ram_cr : signal is true;
    signal s_cw : unsigned(7 downto 0) := (others => '0');
    signal s_cr : unsigned(7 downto 0) := to_unsigned(250, 8);
    signal s_crq : std_logic_vector(31 downto 0) := x"00080200";
    signal s_cru, s_crv : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_sring is array (0 to 255) of std_logic_vector(3 downto 0);
    signal ram_sr : t_sring := (others => "0111");
    attribute no_rw_check of ram_sr : signal is true;
    signal s_sw : unsigned(7 downto 0) := (others => '0');
    signal s_sr : unsigned(7 downto 0) := to_unsigned(233, 8);
    signal s_srq, s_srf, s_sr26, s_sr27 : std_logic_vector(3 downto 0) := "0111";
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y13, s_y14, s_y15, s_y16, s_y17, s_y18x, s_yf18 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- syncs: one input register, then the EBR delay (taps C_LAT - 3 and C_LAT - 1); the early avid taps in FFs
    type t_bit_shift is array (0 to 9) of std_logic;
    signal s_hs0, s_vs0, s_fd0 : std_logic := '1';
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_roms : process(clk)
    begin
        if rising_edge(clk) then
            s_fe_q <= C_FE(to_integer(s_fe_a));
            s_fo_q <= C_FO(to_integer(s_fo_a));
            s_vt_q <= C_VT(to_integer(s_vt_a));
            s_pk_q <= C_PK(to_integer(s_pk_a));
            s_y2_q <= C_Y2(to_integer(s_y2_a));
        end if;
    end process p_roms;

    p_cols : process(clk)
    begin
        if rising_edge(clk) then
            s_lc_q <= C_LC(to_integer(s_lc_a));
            s_bc_q <= C_BC(to_integer(s_bc_a));
        end if;
    end process p_cols;

    p_knot : process(clk)
    begin
        if rising_edge(clk) then
            s_kr_q <= ram_kn(to_integer(s_kr_a));
            if s_kw_e = '1' then ram_kn(to_integer(s_kw_a)) <= s_kw_d; end if;
        end if;
    end process p_knot;

    p_tex : process(clk)
    begin
        if rising_edge(clk) then
            s_tx_q <= ram_tx(to_integer(s_tx_a));
            if s_tw_e = '1' then ram_tx(to_integer(s_tw_a)) <= s_tw_d; end if;
        end if;
    end process p_tex;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_lu_q <= ram_lu(to_integer(s_lu_ra));
            if s_lu_we = '1' then ram_lu(to_integer(s_lu_wa)) <= s_lu_wd; end if;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one serial multiplier)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s : signed(10 downto 0);
        variable v_slot : integer range 0 to 31;
        variable v_d : signed(16 downto 0);
        variable v_i : unsigned(5 downto 0);
        variable v_g : unsigned(9 downto 0);
        variable v_sum : unsigned(16 downto 0);
        variable v_p : unsigned(31 downto 0);
        variable v_sl : unsigned(19 downto 0);
        variable v_rs : signed(20 downto 0);
        variable v_ip : signed(20 downto 0);
        variable v_t : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            s_vx <= f_vstep_x(registers_in(8)(3 downto 0));
            s_sy <= f_vstep_y(registers_in(8)(3 downto 0));
            s_trs <= f_trow(registers_in(8)(3 downto 0));

            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
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

            -- zone clock 1: switches, timing constants
            if s_zone = '1' and s_zc = 1 then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_stripes <= not registers_in(6)(0);
                s_outline <= registers_in(6)(1);
                s_ember   <= registers_in(6)(2);
                s_tear    <= not registers_in(6)(3);
                s_osc     <= registers_in(6)(4);
                case to_integer(s_vx) is
                    -- shape x is measured from the tower axis (the flame sits on the bell's axis);
                    when 4 =>
                        s_sx <= to_unsigned(43, 6); s_sv <= to_unsigned(32, 6);
                        s_xv0 <= to_signed(-12356, 17); s_xdp <= to_unsigned(109, 8); s_tcs <= to_unsigned(91, 7);
                    when 3 =>
                        s_sx <= to_unsigned(24, 6); s_sv <= to_unsigned(24, 6);
                        s_xv0 <= to_signed(-16480, 17); s_xdp <= to_unsigned(146, 8); s_tcs <= to_unsigned(51, 7);
                    when others =>
                        s_sx <= to_unsigned(16, 6); s_sv <= to_unsigned(16, 6);
                        s_xv0 <= to_signed(-16480, 17); s_xdp <= to_unsigned(146, 8); s_tcs <= to_unsigned(34, 7);
                end case;
            end if;
            s_hs0 <= data_in.hsync_n; s_vs0 <= data_in.vsync_n; s_fd0 <= data_in.field_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to 9 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
            g_wm1 <= s_w - 1;
        end if;
    end process p_ctrl;

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
    -- Source side: the camera luma ring (Video mode)
    ---------------------------------------------------------------------------
    p_src : process(clk)
    begin
        if rising_edge(clk) then
            s_lu_we <= data_in.avid;
            s_lu_wa <= s_wcol;
            s_lu_wd <= data_in.y(9 downto 2);
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;

            -- flame texture capture: each texel is the mean of the first 4 pixels of its 1/64 of the
            -- line; one row per 1/64 of the field (every line of the band rewrites it, the last wins)
            s_tav <= data_in.avid;
            if data_in.avid = '1' then
                s_tcc <= s_tcc + s_tcs;
                s_tcd <= s_tcc(15 downto 10);
                if s_tcc(15 downto 10) /= s_tcd then
                    s_tbs <= resize(unsigned(data_in.y(9 downto 4)), 8); s_tbc <= "01";
                elsif s_tbc /= "00" then
                    s_tbs <= s_tsum; s_tbc <= s_tbc + 1;
                end if;
            else
                s_tcc <= (others => '0'); s_tcd <= (others => '1'); s_tbc <= "00";
                if s_tav = '1' then s_trc <= s_trc + s_trs; end if;
            end if;
            if s_zone = '1' then s_trc <= (others => '0'); end if;
        end if;
    end process p_src;
    -- the 4th sample of a texel column completes it: write {row, column} straight from the sum
    s_tsum <= s_tbs + unsigned(data_in.y(9 downto 4));
    s_tw_e <= '1' when data_in.avid = '1' and s_tcc(15 downto 10) = s_tcd and s_tbc = "11" and s_trc(16) = '0' else '0';
    s_tw_a <= s_trc(15 downto 10) & s_tcd;
    s_tw_d <= std_logic_vector(s_tsum(7 downto 4));
    ---------------------------------------------------------------------------
    -- Micro engine: the per-field constants and the next line's values (program in C_UROM,
    -- assembled by masm.py); and the knot loop that writes the line's fade knots
    ---------------------------------------------------------------------------
    p_urom : process(clk)
    begin
        if rising_edge(clk) then
            if u_adv = '1' then u_q <= C_UROM(to_integer(u_ua)); end if;   -- held while the pipeline stalls
        end if;
    end process p_urom;

    p_rf : process(clk)
    begin
        if rising_edge(clk) then
            if u_adv = '1' then u_rq <= ram_rf(to_integer(u_ra)); end if;   -- read in D, held while the pipeline stalls
            -- ST writes straight from the execute stage
            if u_run = '1' and u_irv2 = '1' and u_adv = '1' and u_ir2(31 downto 28) = "1001" then
                ram_rf(to_integer(unsigned(u_ir2(17 downto 10)))) <= std_logic_vector(resize(u_acc, 32));
            end if;
        end if;
    end process p_rf;

    p_stg : process(clk)
    begin
        if rising_edge(clk) then
            s_st_q <= ram_st(to_integer(cp_i));
            -- OUT writes the staging RAM at its port number (the copier reads ports 12 .. 36)
            if o_v = '1' then
                ram_st(to_integer(o_p)) <= std_logic_vector(resize(o_d, 32));
            end if;
        end if;
    end process p_stg;

    -- the RF read address comes straight from the uROM word (the RF is read in D; its output is
    -- re-registered in fabric in R): an ST is not read by the next TWO instructions (masm inserts NOPs)
    u_ra <= unsigned(u_q(17 downto 10)) + u_ix when u_q(27 downto 26) = "11" else unsigned(u_q(17 downto 10));

    p_eng : process(clk)
        variable v_op : integer range 0 to 15;
        variable v_b : signed(23 downto 0);
        variable v_d : signed(24 downto 0);
        variable v_a : unsigned(15 downto 0);
        variable v_bb : unsigned(15 downto 0);
        variable v_t : unsigned(18 downto 0);
        variable v_pp : unsigned(17 downto 0);
        variable v_x : unsigned(31 downto 0);
        variable v_stall, v_take : std_logic;
        variable v_tgt : unsigned(8 downto 0);
        variable v_c10 : signed(9 downto 0);
        variable v_e : unsigned(15 downto 0);
        variable v_m : signed(23 downto 0);
        variable v_in : signed(23 downto 0);
    begin
        if rising_edge(clk) then
            -- kicks: the field program in the zone (it ends with line 0), then one line program per line start
            e_kick <= '0';
            if s_zone = '1' and s_zc = 40 then e_fkick <= '1'; end if;
            if g_start = '1' then
                e_kick <= '1';
                s_rbank <= e_last;
            end if;

            -- copier: once the program has halted and the pixel pipe is idle (31 clocks after the line),
            -- move the staged values of ports 12 .. 36 into the l_ registers (one per clock)
            if g_avid = '1' then cp_idle <= (others => '0'); elsif cp_idle /= 31 then cp_idle <= cp_idle + 1; end if;
            cp_v <= cp_run; cp_k <= cp_i;
            if cp_run = '1' then
                if cp_i = 36 then cp_run <= '0'; end if;
                cp_i <= cp_i + 1;
            elsif e_pend = '1' and cp_idle = 31 then
                cp_run <= '1'; cp_i <= to_unsigned(12, 6); e_pend <= '0';
            end if;
            if cp_v = '1' then
                case to_integer(cp_k) is
                    when 12 => l_a <= s_st_q(0);
                    when 13 => l_j <= signed(s_st_q(7 downto 0));
                    when 14 => l_nl <= signed(s_st_q(3 downto 0));
                    when 15 => l_sli <= unsigned(s_st_q(11 downto 0));
                    when 17 => l_swl <= signed(s_st_q(8 downto 0));
                    when 18 => l_lf0 <= signed(s_st_q(22 downto 0));
                    when 19 => l_bell <= s_st_q(0);
                    when 20 => l_hw <= unsigned(s_st_q(9 downto 0));
                    when 21 => l_hww <= unsigned(s_st_q(9 downto 0));
                    when 22 => l_iy <= unsigned(s_st_q(7 downto 2));
                    when 23 => l_lp <= unsigned(s_st_q(7 downto 2));
                    when 24 => l_dl <= unsigned(s_st_q(5 downto 0));
                    when 25 => l_cap <= s_st_q(0);
                    when 26 => l_hcl <= signed(s_st_q(6 downto 0));
                    when 27 => l_vr <= unsigned(s_st_q(7 downto 2));
                    when 28 => l_wingy <= s_st_q(0);
                    when 29 => l_e0 <= signed(s_st_q(12 downto 0));
                    when 30 => l_tev <= s_st_q(0);
                    when 31 => l_tx0 <= unsigned(s_st_q(11 downto 0));
                    when 33 => l_rsq <= unsigned(s_st_q(11 downto 0));
                    when 34 => l_tdu <= unsigned(s_st_q(13 downto 0));
                    when 35 => l_tu0 <= unsigned(s_st_q(15 downto 0));
                    when 36 => l_tv <= unsigned(s_st_q(5 downto 0));
                    when others => null;
                end case;
            end if;

            -- OUT decode (one clock after E)
            o_v <= '0';
            if o_v = '1' then
                case to_integer(o_p) is
                    when 1  => s_vt_a <= unsigned(o_d(8 downto 0));
                    when 2  => s_pk_a <= unsigned(o_d(7 downto 0));
                    when 3  => s_y2_a <= unsigned(o_d(7 downto 0));
                    when 4  => s_hinc <= unsigned(o_d);
                    when 5  => s_ph0 <= unsigned(o_d);
                    when 6  => s_ts1 <= unsigned(o_d(9 downto 0));
                    when 7  => s_ts2 <= unsigned(o_d(9 downto 0));
                    when 8  => s_xec <= unsigned(o_d(10 downto 0));
                    when 9  => s_d3n <= o_d(8 downto 0);
                    when 10 => s_d3p <= unsigned(o_d(7 downto 0));
                    when 11 => s_tl <= unsigned(o_d(5 downto 0));
                    when 12 => n_a <= o_d(0);       -- (ports 12 .. 36 also go to the staging RAM)
                    when 41 => u_ix <= unsigned(o_d(7 downto 0));
                    when 40 =>
                        -- KNOT: start the knot loop for row ACC(12:8), weight ACC(7:0), class n_a
                        e_krow <= unsigned(o_d(12 downto 8));
                        e_t5 <= unsigned(o_d(7 downto 3));
                        e_a <= n_a;
                        c_run <= '1';
                        c_i <= (others => '0');
                        c_wi <= (others => '0');
                    when others => null;
                end case;
            end if;

            v_stall := '0';
            if u_adv = '0' then v_stall := '1'; end if;

            if u_run = '0' then
                if e_fkick = '1' or e_kick = '1' then
                    -- start: one LFSR step per program run (xorshift 7, 9, 8)
                    v_e := e_lfsr xor shift_left(e_lfsr, 7);
                    v_e := v_e xor shift_right(v_e, 9);
                    e_lfsr <= v_e xor shift_left(v_e, 8);
                    e_wb <= not e_wb;
                    u_run <= '1';
                    if e_fkick = '1' then u_pc <= to_unsigned(C_U_FIELD, 9); else u_pc <= to_unsigned(C_U_LINE, 9); end if;
                    e_fkick <= '0';
                end if;
                u_fva <= '0'; u_fvq <= '0'; u_irv <= '0'; u_irv2 <= '0';
            elsif v_stall = '0' then
                -- F: fetch
                u_ua <= u_pc;
                u_pc <= u_pc + 1;
                u_fva <= '1';
                u_fvq <= u_fva;
                -- D: instruction word, RF address
                u_ir <= u_q; u_irv <= u_fvq;
                -- R: the B operand (the RF word, an IN port or the immediate) into one register, the
                -- instruction moves to E
                u_ir2 <= u_ir; u_irv2 <= u_irv;
                case to_integer(unsigned(u_ir(14 downto 10))) is
                    when 0  => v_in := signed(resize(unsigned(registers_in(0)), 24));
                    when 1  => v_in := signed(resize(unsigned(registers_in(1)), 24));
                    when 2  => v_in := signed(resize(unsigned(registers_in(2)), 24));
                    when 3  => v_in := signed(resize(unsigned(registers_in(4)), 24));
                    when 4  => v_in := signed(resize(unsigned(registers_in(5)), 24));
                    when 5  => v_in := signed(resize(unsigned(registers_in(7)), 24));
                    when 6  => v_in := signed(resize(s_sy, 24));
                    when 7  => v_in := signed(resize(s_sx, 24));
                    when 8  => v_in := signed(resize(s_sv, 24));
                    when 10 => v_in := resize(s_xv0, 24);
                    when 11 => v_in := signed(resize(unsigned(registers_in(3)), 24));
                    when 12 => v_in := signed(resize(unsigned(s_vt_q), 24));
                    when 13 => v_in := signed(resize(unsigned(s_pk_q(8 downto 0)), 24));
                    when 14 => v_in := signed(resize(unsigned(s_y2_q(26 downto 16)), 24));
                    when 15 => v_in := signed(resize(unsigned(s_y2_q(15 downto 8)), 24));
                    when 16 => v_in := signed(resize(unsigned(s_y2_q(7 downto 0)), 24));
                    when 17 => v_in := resize(signed(e_lfsr(7 downto 0)), 24);
                    when 18 => v_in := resize(signed(e_lfsr(11 downto 8)), 24);
                    when 19 => if s_tear = '1' then v_in := to_signed(1, 24); else v_in := (others => '0'); end if;
                    when 20 => if s_stripes = '1' then v_in := (others => '0'); else v_in := to_signed(1, 24); end if;
                    when 21 => v_in := signed(resize(e_lfsr, 24));
                    when 22 => v_in := signed(resize(unsigned(s_pk_q(15 downto 9)), 24));
                    when others => v_in := (others => '0');
                end case;
                case u_ir(27 downto 26) is
                    when "01"   => u_b <= v_in;
                    when "10"   => u_b <= resize(signed(u_ir(25 downto 10)), 24);
                    when others => u_b <= signed(u_rq(23 downto 0));
                end case;
                -- E: execute
                if u_irv2 = '1' then
                    v_op := to_integer(unsigned(u_ir2(31 downto 28)));
                    v_b := u_b;
                    v_d := resize(u_acc, 25) - resize(v_b, 25);
                    v_take := '0';
                    v_tgt := unsigned(u_ir2(18 downto 10));
                    case v_op is
                        when 1  => u_acc <= v_b;
                        when 2  => u_acc <= u_acc + v_b;
                        when 3  => u_acc <= v_d(23 downto 0);
                        when 4  => u_acc <= u_acc and v_b;
                        when 5 | 6 =>
                            -- MAX / MIN: compare now, select in the unit's last state (one stall clock)
                            if v_op = 5 then u_mxt <= v_d(24); else u_mxt <= not v_d(24); end if;
                            u_bin <= unsigned(resize(v_b, 32));
                            u_mx <= '1';
                            u_mode <= "11"; u_adv <= '0';
                        when 7  =>
                            -- MUL: |acc| x |b| (16 bits each), radix-4, then the post-shift and the sign
                            if u_acc < 0 then v_m := -u_acc; else v_m := u_acc; end if;
                            v_a := unsigned(v_m(15 downto 0));
                            if v_b < 0 then v_m := -v_b; else v_m := v_b; end if;
                            v_bb := unsigned(v_m(15 downto 0));
                            u_ma <= v_a;
                            u_ma3 <= resize(v_a, 18) + shift_left(resize(v_a, 18), 1);
                            u_mh <= (others => '0');
                            u_ml <= v_bb;
                            u_mneg <= u_acc(23) xor v_b(23);
                            u_sh <= unsigned(u_ir2(9 downto 5));
                            u_mcnt <= (others => '0');
                            u_mode <= "01"; u_adv <= '0';
                        when 8  =>
                            -- SHR: through the barrel
                            u_bin <= unsigned(resize(u_acc, 32));
                            u_bx <= u_acc(23);
                            u_mneg <= '0';
                            u_sh <= unsigned(u_ir2(9 downto 5));
                            u_mode <= "10"; u_adv <= '0';
                        when 9  => null;    -- ST: the RF write port (p_rf) acts on this stage directly
                        when 10 =>
                            -- OUT: port and data go out one clock later (o_*)
                            o_v <= '1'; o_p <= unsigned(u_ir2(15 downto 10)); o_d <= u_acc;
                        when 11 => if u_acc(23) = '1' then v_take := '1'; end if;
                        when 12 => if u_acc(23) = '0' then v_take := '1'; end if;
                        when 13 => v_take := '1';
                        when 14 =>
                            v_take := '1';
                            if u_ir2(5) = '1' then v_tgt := u_ret; else u_ret <= u_pc - 3; end if;
                        when 15 =>
                            u_run <= '0'; e_done <= '1'; e_pend <= '1';
                            u_fva <= '0'; u_fvq <= '0'; u_irv <= '0'; u_irv2 <= '0';
                        when others => null;
                    end case;
                    if v_take = '1' then
                        -- taken branch: refetch from the target, drop the three instructions in flight
                        u_ua <= v_tgt; u_pc <= v_tgt + 1;
                        u_fva <= '1'; u_fvq <= '0'; u_irv <= '0'; u_irv2 <= '0';
                    end if;
                end if;
            end if;

            -- multiply / shift unit (the pipeline is stalled while it runs)
            case u_mode is
                when "01" =>
                    case u_ml(1 downto 0) is
                        when "00"   => v_pp := (others => '0');
                        when "01"   => v_pp := resize(u_ma, 18);
                        when "10"   => v_pp := shift_left(resize(u_ma, 18), 1);
                        when others => v_pp := u_ma3;
                    end case;
                    v_t := resize(u_mh, 19) + resize(v_pp, 19);
                    u_mh <= v_t(18 downto 2);
                    u_ml <= v_t(1 downto 0) & u_ml(15 downto 2);
                    if u_mcnt = 7 then
                        u_bin <= v_t(17 downto 2) & v_t(1 downto 0) & u_ml(15 downto 2);
                        u_bx <= '0';
                        u_mode <= "10";
                    end if;
                    u_mcnt <= u_mcnt + 1;
                when "10" =>
                    -- serial arithmetic right shift: 8 or 1 places per clock (SHR inputs are sign-extended)
                    if u_sh(4 downto 3) /= "00" then
                        u_bin <= unsigned(std_logic_vector'(u_bx & u_bx & u_bx & u_bx & u_bx & u_bx & u_bx & u_bx)) & u_bin(31 downto 8);
                        u_sh <= u_sh - 8;
                    elsif u_sh(2 downto 0) /= "000" then
                        u_bin <= u_bx & u_bin(31 downto 1);
                        u_sh <= u_sh - 1;
                    else
                        u_mode <= "11";
                    end if;
                when "11" =>
                    if u_mx = '1' then
                        if u_mxt = '1' then u_acc <= signed(u_bin(23 downto 0)); end if;
                    elsif u_mneg = '1' then
                        u_acc <= -signed(u_bin(23 downto 0));
                    else
                        u_acc <= signed(u_bin(23 downto 0));
                    end if;
                    u_mx <= '0';
                    u_mode <= "00"; u_adv <= '1';
                when others => null;
            end case;

            -- the line's knot bank is published once the program has halted and the knots are written
            if e_done = '1' and c_run = '0' and c_v = "000000" and c_tail = '0' then
                e_last <= e_wb;
                e_done <= '0';
            end if;

            -- knot loop: 32 knots of the line's fade into the knot RAM bank e_wb
            c_v(1) <= c_run;
            if c_run = '1' then
                if e_krow(0) = '0' then
                    s_fe_a <= e_a & e_krow(4 downto 1) & c_i;
                else
                    s_fe_a <= e_a & (e_krow(4 downto 1) + 1) & c_i;
                end if;
                s_fo_a <= e_a & e_krow(4 downto 1) & c_i;
                if c_i = 31 then c_run <= '0'; end if;
                c_i <= c_i + 1;
            end if;
            c_par(1) <= e_krow(0);
            for k in 2 to 6 loop c_v(k) <= c_v(k - 1); end loop;
            for k in 2 to 3 loop c_par(k) <= c_par(k - 1); end loop;
            c2_e <= unsigned(s_fe_q);
            c2_o <= unsigned(s_fo_q);
            if c_par(3) = '0' then
                c3_a <= c2_e; c3_d <= signed(resize(c2_o, 9)) - signed(resize(c2_e, 9));
            else
                c3_a <= c2_o; c3_d <= signed(resize(c2_e, 9)) - signed(resize(c2_o, 9));
            end if;
            c4_a <= c3_a;
            c4_p <= c3_d * signed('0' & e_t5);
            v_c10 := signed(resize(c4_a, 10)) + resize(shift_right(c4_p, 5), 10);
            c5_k <= unsigned(v_c10(7 downto 0));
            s_kw_e <= '0';
            c_tail <= '0';
            if c_v(6) = '1' then
                c_prev <= c5_k;
                c_wi <= c_wi + 1;
                if c_wi /= 0 then
                    s_kw_e <= '1';
                    s_kw_a <= e_wb & (c_wi - 1);
                    s_kw_d <= std_logic_vector(c_prev) & std_logic_vector(c5_k);
                end if;
                if c_wi = 31 then c_tail <= '1'; end if;
            elsif c_tail = '1' then
                s_kw_e <= '1';
                s_kw_a <= e_wb & "11111";
                s_kw_d <= std_logic_vector(c_prev) & std_logic_vector(c_prev);
            end if;
        end if;
    end process p_eng;


    ---------------------------------------------------------------------------
    -- Delay rings in block RAM: the chroma box's 8-sample delay and the sync delay
    ---------------------------------------------------------------------------
    p_rings : process(clk)
    begin
        if rising_edge(clk) then
            s_crq <= ram_cr(to_integer(s_cr));
            -- spare bits 26 .. 20 carry pipeline flags 7 stages on (stage k in -> s_crq acts as stage k + 7)
            ram_cr(to_integer(s_cw)) <= "00000" & p_brn(4) & p_fl(3)(4) & p_fl(3)(3) & p_fl(2)(9 downto 8)
                                        & std_logic_vector(s_u12) & std_logic_vector(s_v12);
            s_srq <= ram_sr(to_integer(s_sr));
            ram_sr(to_integer(s_sw)) <= s_gav_sr(0) & s_hs0 & s_vs0 & s_fd0;
            s_cw <= s_cw + 1; s_cr <= s_cr + 1;
            s_sw <= s_sw + 1; s_sr <= s_sr + 1;
            s_cru <= unsigned(s_crq(19 downto 10)); s_crv <= unsigned(s_crq(9 downto 0));
            r10_sp <= s_crq(21 downto 20);           -- p_fl(2)(9:8) at stage 10
            r11_lc <= s_crq(22);                     -- p_fl(3)(3) at stage 11
            r11_tk <= s_crq(23);                     -- p_fl(3)(4) at stage 11
            r12_brn <= unsigned(s_crq(26 downto 24)); -- p_brn(4) at stage 12
            s_srf <= s_srq; s_sr26 <= s_srf; s_sr27 <= s_sr26;
        end if;
    end process p_rings;

    ---------------------------------------------------------------------------
    -- Pixel path (stage k registers hold the pixel when s_gav_sr(k - 1) = '1');
    -- geometry in pixel units of the 1080-line frame from stage 1 on
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_neg : std_logic;
        variable v_inc : signed(12 downto 0);
        variable v_s : signed(13 downto 0);
        variable v_e : unsigned(15 downto 0);
        variable v_w : unsigned(15 downto 0);
        variable v_u : unsigned(17 downto 0);
        variable v_a : unsigned(10 downto 0);
        variable v_k : unsigned(12 downto 0);
        variable v_p : signed(21 downto 0);
        variable v_x : unsigned(7 downto 0);
        variable v_l : signed(9 downto 0);
        variable v_r : unsigned(12 downto 0);
        variable v_sp : unsigned(6 downto 0);
        variable v_f, v_f5 : std_logic_vector(11 downto 0);
        variable v_c : unsigned(7 downto 0);
        variable v_g : unsigned(4 downto 0);
        variable v_d13 : signed(12 downto 0);
        variable v_ft : unsigned(12 downto 0);
        variable v_tg : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            -- 0: DDAs (layout x, shape x, column phase, flame)
            s_xd0 <= s_xv0 - resize(l_swl, 17);          -- static over the line
            s_nsri <= -signed(resize(l_sli, 13));                -- the flame is symmetric: one slope
            if g_start = '1' then
                s_xs <= (others => '0');
                s_xv <= s_xv0;
                s_xd <= s_xd0;
                s_ph <= s_ph0;
                s_lf <= l_lf0;
                s_tu <= l_tu0;
            elsif g_avid = '1' then
                s_tu <= s_tu + l_tdu;
                if s_xd(16) = '1' then v_inc := signed(resize(l_sli, 13)); else v_inc := s_nsri; end if;
                s_xs <= s_xs + signed(resize(s_sx, 17));
                s_xv <= s_xv + signed(resize(s_sv, 17));
                s_xd <= s_xd + signed(resize(s_sv, 17));
                s_ph <= s_ph + s_hinc;
                s_lf <= s_lf + resize(v_inc, 23);
            end if;

            -- 1: pixel units, the fade position, the flame level, the column phase, the texel column
            case s_tu(15 downto 14) is
                when "00"   => p1_tc <= s_tu(13 downto 8);
                when "11"   => p1_tc <= (others => '0');         -- left of the thumbnail
                when others => p1_tc <= (others => '1');         -- right of it
            end case;
            p1_xs <= unsigned(s_xs(15 downto 4));
            p1_xv <= s_xv(16 downto 4);
            p1_xj <= signed('0' & s_xs(15 downto 4)) + resize(l_j, 13);
            p1_lf <= resize(shift_right(s_lf, 8), 15);
            if s_ph(23 downto 16) >= 161 then p1_hl <= '1'; else p1_hl <= '0'; end if;
            if s_ph(23 downto 14) >= s_ts1 then p1_spb <= "10";
            elsif s_ph(23 downto 14) >= s_ts2 then p1_spb <= "01";
            else p1_spb <= "00"; end if;

            -- 2: knot RAM address, offsets, compares, the flame toe index
            if p1_xj < 0 then v_s := (others => '0');
            elsif p1_xj > 1983 then v_s := to_signed(1983, 14);
            else v_s := resize(p1_xj, 14); end if;
            s_kr_a <= s_rbank & unsigned(v_s(10 downto 6));
            p2_kf <= unsigned(v_s(5 downto 0));
            p2_xb <= p1_xv;                                -- shape x from the tower axis
            p2_xd <= p1_xv - signed(resize(s_xdp, 13));
            p2_ae <= signed('0' & p1_xs) - signed(resize(s_xec, 13));
            v_f := (others => '0');
            if p1_xs < 880 or p1_xs >= 1648 then v_f(0) := '1'; end if;                       -- w1a
            v_f(1) := p1_hl;                                                                   -- hlow
            if (l_a = '1' and p1_xs < 880) or (l_a = '0' and p1_xs < 862) then
                v_f(2) := '1'; end if;                                                         -- left wing
            if p1_xs < 81 then v_f(3) := '1'; end if;                                          -- left column
            v_d13 := signed('0' & p1_xs) - signed('0' & l_tx0);
            if l_tev = '1' and v_d13(12 downto 9) = "0000" then v_f(4) := '1'; end if;         -- tracking line (512 px)
            if p1_xv < 0 then v_f(5) := '1'; end if;                                           -- xb < 0
            if p1_xv > 0 then v_f(6) := '1'; end if;                                           -- xb > 0
            if l_wingy = '1' and p1_xs < 900 and p1_xv < 0 then v_f(7) := '1'; end if;
            if l_a = '1' then
                if p1_xs < 880 and p1_xs >= 877 then v_f(9 downto 8) := "10";
                elsif p1_xs < 877 and p1_xs >= 874 then v_f(9 downto 8) := "01"; end if;
            else
                v_f(9 downto 8) := std_logic_vector(p1_spb);
            end if;
            p_fl(2) <= v_f;
            s_tx_a <= l_tv & p1_tc;                        -- texel read (data at stage 3)
            -- flame toe: below TL the level falls at half slope; burn level for the blue
            v_s := resize(p1_lf, 14) + signed(resize(s_tl, 14));
            if p1_lf >= signed(resize(s_tl, 15)) then
                p2_flm <= f_clip8(resize(p1_lf, 16));
            elsif v_s < 0 then
                p2_flm <= (others => '0');
            else
                p2_flm <= unsigned(v_s(8 downto 1));
            end if;
            v_c := f_clip8(resize(p1_lf, 16) - 300);
            p2_brn <= v_c(7 downto 5);

            -- 3: magnitudes, exp lookup (the knot RAM reads)
            p3_kf <= p2_kf;
            if p2_xb < 0 then p3_adb <= unsigned(resize(-p2_xb, 12)); else p3_adb <= unsigned(resize(p2_xb, 12)); end if;
            if p2_xd < 0 then p3_adx <= unsigned(resize(-p2_xd, 12)); else p3_adx <= unsigned(resize(p2_xd, 12)); end if;
            if p2_ae < 0 then p3_ae <= unsigned(resize(-p2_ae, 12)); else p3_ae <= unsigned(resize(p2_ae, 12)); end if;
            p3_flm <= p2_flm;
            p_brn(3) <= std_logic_vector(p2_brn); p_fl(3) <= p_fl(2);

            -- 4: knots in; bell distances, shading, edge-glow slope; the texel into fabric
            p4_tex <= unsigned(s_tx_q);
            p4_k0 <= unsigned(s_kr_q(15 downto 8));
            p4_k1 <= unsigned(s_kr_q(7 downto 0));
            p4_kf <= p3_kf;
            if l_bell = '1' then p4_tt <= signed(resize(p3_adb, 13)) - signed(resize(l_hw, 13));
            else p4_tt <= to_signed(2047, 13); end if;
            s_hw5 <= signed(resize(l_hw, 13)) + 5;
            p4_c5 <= s_hw5 - signed(resize(p3_adb, 13));                -- = 5 - tt on bell lines (cap only matters there)
            p4_dw <= signed(resize(l_hww, 13)) - signed(resize(p3_adb, 13));
            p4_hcl <= signed(resize(p3_adb, 13)) - resize(l_hcl, 13);
            v_u := resize(shift_left(resize(p3_adx, 18), 3) - resize(p3_adx, 18), 18);
            if v_u(17 downto 2) > 227 then p4_d8 <= to_unsigned(255, 8); else p4_d8 <= resize(28 + v_u(9 downto 2), 8); end if;
            v_u := shift_left(resize(p3_ae, 18), 6) + shift_left(resize(p3_ae, 18), 1) + resize(p3_ae, 18);
            p4_egm <= resize(v_u(17 downto 5), 12);
            p4_flm <= p3_flm;
            p4_dr <= signed(resize(p3_adb, 13)) - signed(resize(l_rsq, 13));
            p_fl(4) <= p_fl(3); p_brn(4) <= p_brn(3);

            -- 5: knot difference, clipped masks, glow, toe
            p5_df <= signed(resize(p4_k1, 9)) - signed(resize(p4_k0, 9));
            p5_k0 <= p4_k0;
            p5_kf <= p4_kf;
            if p4_tt < 0 then v_s := resize(-p4_tt, 14); else v_s := resize(p4_tt, 14); end if;
            if v_s > 255 then p5_att <= to_unsigned(255, 8); else p5_att <= unsigned(v_s(7 downto 0)); end if;
            if p4_tt > 0 then p5_ttp <= '1'; else p5_ttp <= '0'; end if;
            p5_r8a <= -p4_tt - signed(resize(l_dl, 13));
            p5_cc <= f_clip8(shift_left(resize(p4_c5, 18), 4) + shift_left(resize(p4_c5, 18), 3) + resize(p4_c5, 18));
            if p_fl(4)(7) = '1' then
                p5_wm <= f_clip8(shift_left(resize(p4_dw, 18), 4) + shift_left(resize(p4_dw, 18), 1));
            else
                p5_wm <= (others => '0');
            end if;
            if l_a = '1' then
                p5_eg <= (others => '0');
            else
                p5_eg <= f_clip8(signed(resize(p4_egm, 14)) + resize(l_e0, 14));
            end if;
            -- the flame carrying the camera thumbnail: x (texel + 4) / 16 in Video, x 1 in Osc
            if s_osc = '1' then v_tg := to_unsigned(16, 5); else v_tg := resize(p4_tex, 5) + 4; end if;
            v_ft := p4_flm * v_tg;
            if v_ft(12) = '1' then p5_flm <= (others => '1'); else p5_flm <= v_ft(11 downto 4); end if;
            p5_hc <= f_clip8(shift_left(resize(p4_hcl, 18), 4) + shift_left(resize(p4_hcl, 18), 3) + resize(p4_hcl, 18));
            -- Dusk squeeze: outside the radius the A-row window opens; the lift ramps in over D3P / 4 px
            p5_dr8 <= f_clip8(shift_left(resize(p4_dr, 16), 2));
            v_f5 := p_fl(4);
            if p4_dr(12) = '0' then v_f5(0) := '1'; v_f5(11) := '1'; end if;
            p_fl(5) <= v_f5; p_d8(5) <= std_logic_vector(p4_d8);

            -- 6: knot product, Gaussian indexes, the bell ramp, the key candidates in the cones
            p6_pr <= resize(p5_df * signed('0' & p5_kf), 16);
            p6_k0 <= p5_k0;
            if p5_ttp = '1' then
                if p_fl(5)(6) = '1' then v_g := to_unsigned(15, 5); else v_g := to_unsigned(18, 5); end if;
            else
                v_g := to_unsigned(23, 5);
            end if;
            v_r := resize(shift_right(p5_att * v_g, 4), 13);
            if v_r > 63 then p6_gi <= to_unsigned(63, 6); else p6_gi <= v_r(5 downto 0); end if;
            if l_cap = '1' then
                if p5_hc < p5_cc then p6_r <= p5_hc; else p6_r <= p5_cc; end if;
            else
                p6_r <= f_clip8(shift_right(shift_left(resize(p5_r8a, 18), 4) + resize(p5_r8a, 18), 1));
            end if;
            v_w := shift_left(resize(p5_wm, 16), 6) + shift_left(resize(p5_wm, 16), 5) + resize(p5_wm, 16);
            p6_ka <= v_w(15 downto 8);
            v_w := shift_left(resize(p5_wm, 16), 7) - shift_left(resize(p5_wm, 16), 5) - resize(p5_wm, 16);
            p6_wa <= v_w(15 downto 8);
            -- the (textured) flame with the edge glow (one value rides to stage 12)
            p6_lx <= f_umax(p5_flm, p5_eg);
            if p5_dr8 < s_d3p then p6_lift <= p5_dr8; else p6_lift <= s_d3p; end if;
            v_f5 := p_fl(5); v_f5(10) := p5_ttp;                                           -- outside the bell
            p_fl(6) <= v_f5; p_d8(6) <= p_d8(5);

            -- 7: fade value, Gaussian lookups, interior level, camera luma
            p7_f8 <= signed(resize(p6_k0, 10)) + resize(shift_right(p6_pr, 6), 10);
            p7_gau <= to_unsigned(C_GAUSS(to_integer(p6_gi)), 8);
            p7_iyr <= resize(l_iy * p6_r, 16);
            p7_ycam <= unsigned(s_lu_q);
            p7_dl <= signed(resize(p6_lift, 10)) + resize(s_d3n, 10);
            p7_ka <= p6_ka; p7_wa <= p6_wa;
            p_lx(7) <= std_logic_vector(p6_lx);

            -- 8: dusk offset, line products, the beam source
            p8_fd <= f_clip8(resize(p7_f8, 12) + resize(p7_dl, 12));
            v_w := resize(l_lp * p7_gau, 16); p8_line <= v_w(13 downto 6);
            p8_int1 <= p7_iyr(13 downto 6);
            if s_osc = '1' then
                p8_s8 <= unsigned(p_d8(7));
            else
                p8_s8 <= p7_ycam;
            end if;
            -- Video: the camera at 1/2 shows through the white layer outside the bell
            if s_osc = '0' and p_fl(7)(10) = '1' then
                p8_bg <= shift_right(p7_ycam, 1);
            else
                p8_bg <= (others => '0');
            end if;
            p8_ka <= p7_ka; p8_wa <= p7_wa;

            -- 9: raw key, interior, wing
            if l_a = '1' then
                if p_fl(8)(0) = '1' then p9_kraw <= f_umax(p8_fd, p8_ka); else p9_kraw <= (others => '0'); end if;
            else
                if p_fl(8)(1) = '1' then p9_kraw <= p8_fd; else p9_kraw <= (others => '0'); end if;
            end if;
            v_w := p8_int1 * p8_s8;
            if s_outline = '1' then p9_int <= (others => '0'); else p9_int <= v_w(15 downto 8); end if;
            if p_fl(8)(7) = '0' then p9_wing <= (others => '0');
            elsif l_a = '1' then p9_wing <= p8_wa;
            else v_w := resize(p8_wa * l_vr, 16); p9_wing <= v_w(13 downto 6); end if;
            p9_line <= p8_line;

            -- 10: the key's x-order RC lag (a one-clock loop; a line starts from 0); the bell
            if s_gav_sr(8) = '1' and s_gav_sr(9) = '0' then v_k := (others => '0'); else v_k := resize(p10_kk, 13); end if;
            v_r := shift_left(resize(p9_kraw, 13), 4);
            if v_r(11 downto 0) < v_k(11 downto 0) then
                p10_kk <= v_r(11 downto 0);
            elsif l_a = '1' then
                p10_kk <= v_k(11 downto 0) + shift_right(v_r(11 downto 0) - v_k(11 downto 0), 4);
            else
                p10_kk <= v_k(11 downto 0) + shift_right(v_r(11 downto 0) - v_k(11 downto 0), 3);
            end if;
            -- the white layer's region value: the wing cone in the left wing, the bell elsewhere
            if p_fl(9)(2) = '1' then p10_bw <= p9_wing;
            elsif l_bell = '1' then p10_bw <= f_umax(p9_line, p9_int);
            else p10_bw <= (others => '0'); end if;

            -- 11: pre-shoot spike
            v_x := p10_kk(11 downto 4);
            case r10_sp is
                when "10"   => if l_a = '1' then v_sp := to_unsigned(64, 7); else v_sp := to_unsigned(56, 7); end if;
                when "01"   => if l_a = '1' then v_sp := to_unsigned(32, 7); else v_sp := to_unsigned(28, 7); end if;
                when others => v_sp := (others => '0');
            end case;
            if r10_op = '1' and l_a = '1' then v_sp := (others => '0'); end if;   -- no A-row cut where Dusk opened the window
            if v_x > 13 then p11_k9 <= resize(v_x, 9) + resize(v_sp, 9); else p11_k9 <= resize(v_x, 9); end if;
            p11_l <= f_umax(unsigned(p_lx(10)), p10_bw);

            -- 12: forced blue (left column, tracking lines); white-layer maxima
            v_k := resize(p11_k9, 13);
            if r11_lc = '1' and v_k < 255 then v_k := to_unsigned(255, 13); end if;
            if r11_tk = '1' and v_k < 140 then v_k := to_unsigned(140, 13); end if;
            p12_k9 <= v_k(8 downto 0);
            p12_l <= p11_l;

            -- 13: burn-through
            v_k := resize(shift_right(p12_k9 * r12_brn, 3), 13);
            p13_kb <= p12_k9 - v_k(8 downto 0);
            p13_l <= p12_l;

            -- 14: mix weight, line noise
            if p13_kb > 255 then p14_km <= to_unsigned(255, 8); else p14_km <= p13_kb(7 downto 0); end if;
            if p13_l >= 16 then
                v_l := signed(resize(p13_l, 10)) + resize(l_nl, 10);
                p14_l <= f_clip8(v_l);
            else
                p14_l <= p13_l;
            end if;

            -- 15: colour ROM addresses; 16: ROM out; 17: fabric
            s_lc_a <= p14_l; s_bc_a <= p14_km;
            p15_km <= p14_km; p15_l <= p14_l;
            p16_km <= p15_km; p16_l <= p15_l;
            p17_inv <= 256 - resize(p16_km, 9);
            p17_ly <= unsigned(s_lc_q(29 downto 20));
            p17_lu <= resize(signed(s_lc_q(19 downto 10)), 11);
            p17_lv <= resize(signed(s_lc_q(9 downto 0)), 11);
            p17_by <= signed(resize(unsigned(s_bc_q(29 downto 20)), 11));
            p17_bu <= resize(signed(s_bc_q(19 downto 10)), 11);
            p17_bv <= resize(signed(s_bc_q(9 downto 0)), 11);
            p17_l <= p16_l;

            -- 18: palette.  Blue: the ROMs (a blue white layer whose chroma is stored halved, an amber
            -- key).  Ember: the key turned crimson (Y / 2, Cb / 4) and the white layer warm (Cr +L/4, Cb -L/4)
            p18_ly <= p17_ly;
            p18_bu <= p17_bu;
            if s_ember = '1' then
                p18_lu <= signed(resize(shift_right(p17_l, 3), 8));
                p18_lv <= -signed(resize(shift_right(p17_l, 3), 8));
                p18_by <= shift_right(p17_by, 1);
                p18_bv <= shift_right(p17_bv, 2);
            else
                p18_lu <= p17_lu(7 downto 0);
                p18_lv <= p17_lv(7 downto 0);
                p18_by <= p17_by;
                p18_bv <= p17_bv;
            end if;
            p18_inv <= p17_inv;

            -- 19: white layer x (1 - k) (its chroma x 2 at stage 20)
            p19_py <= p18_ly * p18_inv;
            p19_pu <= p18_lu * signed('0' & p18_inv(8 downto 4));     -- chroma: a 5-bit weight
            p19_pv <= p18_lv * signed('0' & p18_inv(8 downto 4));
            p19_by <= p18_by; p19_bu <= p18_bu; p19_bv <= p18_bv;

            -- 20: + blue layer
            v_p := signed(resize(p19_py(18 downto 8), 22)) + resize(p19_by, 22) + 64;
            if v_p < 0 then s_y12 <= (others => '0'); elsif v_p > 1023 then s_y12 <= (others => '1');
            else s_y12 <= unsigned(v_p(9 downto 0)); end if;
            v_p := resize(shift_right(p19_pu, 3), 22) + resize(p19_bu, 22) + 512;
            if v_p < 0 then s_u12 <= (others => '0'); elsif v_p > 1023 then s_u12 <= (others => '1');
            else s_u12 <= unsigned(v_p(9 downto 0)); end if;
            v_p := resize(shift_right(p19_pv, 3), 22) + resize(p19_bv, 22) + 512;
            if v_p < 0 then s_v12 <= (others => '0'); elsif v_p > 1023 then s_v12 <= (others => '1');
            else s_v12 <= unsigned(v_p(9 downto 0)); end if;

            -- pipes
            for k in 7 to 9 loop p_fl(k) <= p_fl(k - 1); end loop;
            p_lx(8) <= p_lx(7);
            p_lx(9) <= std_logic_vector(f_umax(unsigned(p_lx(8)), p8_bg));
            p_lx(10) <= p_lx(9);
            r10_op <= p_fl(9)(11);
            for k in 7 to 8 loop p_d8(k) <= p_d8(k - 1); end loop;

            -- camera luma ring read address (arrives at stage 7)
            if s_gav_sr(3) = '1' then s_rx <= s_rx + 1; else s_rx <= (others => '0'); end if;
            s_lu_ra <= s_rx;

            -- output conditioning (centre of the chroma box at C_LAT - 3)
            s_boxu <= s_boxu + s_u12 - s_cru;
            s_boxv <= s_boxv + s_v12 - s_crv;
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_u := resize(s_boxu, 18) + s_boxud; s_u18 <= v_u(13 downto 4);
            v_u := resize(s_boxv, 18) + s_boxvd; s_v18 <= v_u(13 downto 4);
            s_y13 <= s_y12; s_y14 <= s_y13; s_y15 <= s_y14; s_y16 <= s_y15;
            s_y17 <= s_y16; s_y18x <= s_y17;
            v_u := resize(s_y16, 18) + shift_left(resize(s_y17, 18), 1) + resize(s_y18x, 18);
            s_yf18 <= v_u(11 downto 2);
            -- luma toward black; blanking gate
            if s_yf18 > 64 then v_a := resize(s_yf18 - 64, 11); else v_a := (others => '0'); end if;
            if s_srf(3) = '1' then
                s_y19 <= s_yf18 - resize(shift_right(v_a, 3), 10) - resize(shift_right(v_a, 4), 10);
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
    data_out.hsync_n <= s_sr27(2);
    data_out.vsync_n <= s_sr27(1);
    data_out.field_n <= s_sr27(0);
    data_out.avid    <= s_sr27(3);

end architecture beacon;
