-- overlook.vhd  (v1.2 -- the Overlook Hotel corridor carpet, full controls)
--
-- David Hicks' "Hexagon" -- the Shining carpet -- from the user's exact
-- vector spec.  The orange figure is a set of identical horizontal
-- SERPENTINE polylines stacked vertically at the same phase; their upward
-- peaks and downward V-tips interlock without touching, and the hexagon
-- "cells" are pure negative space.  Solid red hexagons float in the dark
-- field touching nothing.
--
-- Tile: user 100x142 px scaled x10.24 -> 1024 x 1454 texture units
-- (u period = 1024 = the free 16-bit wrap; stroke 12.5px = 128 exactly).
-- Half-period polyline (x-fold symmetric about 0 and 512):
--   A(0,0) B(374,210) C(374,650) D(143,773) E(143,1213) F(512,1423)
-- stroked with half-width 64 and MITER joins.  Red hexagons: regular
-- pointy-top, circumradius 169, at (0,430) and (512,993).
--
-- Rendering: per-segment slabs clipped by the miter lines at each join --
-- 24 half-plane tests fed by FIVE shared x-products (9/16, 17/32, 73/128,
-- 19/32, 37/64 of the folded x, single-truncation) and five adders; all
-- coordinates carry 4 fractional bits so edges resolve at 1/16 texel.
-- The slab half-widths and hex bounds are per-frame REGISTERS (Weight and
-- Hex Size knobs); the miter clip lines pass through the polyline
-- vertices and never move.
--
-- Controls:
--   K1 Drift X / K2 Drift Y : bipolar texture drift (deadbanded)
--   K3 Glide                : zoom smoothing strength 1/2..1/256 per frame
--   K4 Weight               : stroke half-width ~20..84 texels
--   K5 Hex Size             : red hexagons ~60..140%
--   K6 Scheme               : five stepped palettes: Classic / Gold Room /
--                             Lobby Blue / Deep Night / Mono Amber
--   S7 Glide on/off   S8 Weave (texture-anchored fibre dither)
--   S9 Video Ground (dark field = dimmed input video)
--   S10 Breathe (slow +-3% zoom triangle, ~17 s cycle)
--   S11 Night (field/orange dimmed, reds full)
--   P12 Zoom (exponential, continuous octave law, centre-anchored, glided)
--
-- All colour and threshold math runs per frame in the vblank sequencer
-- (6-bit, 63 steps, one shared registered multiplier) -- zero pixel-rate
-- cost.  Palette stored U/V (Cb/Cr) swapped for the Videomancer output
-- convention.
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture overlook of program_top is

    constant C_LATENCY : integer := 5;

    -- texture tile
    constant C_P       : integer := 1454;    -- row pitch (v period)
    constant C_VWRAP   : integer := 93056;   -- 1454 * 64 (6 fractional bits)

    ----------------------------------------------------------------------
    -- frame-latched controls
    ----------------------------------------------------------------------
    signal s_k1, s_k2, s_k3, s_k4, s_k5, s_k6 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_sw    : std_logic_vector(4 downto 0) := "00001";  -- S7..S11 (glide on)
    signal s_p12   : unsigned(9 downto 0) := to_unsigned(719, 10);

    signal s_prev_vsync_n : std_logic := '1';
    signal s_vs_pulse     : std_logic := '0';
    signal s_fpar         : std_logic := '0';
    signal s_ilace        : std_logic := '0';

    -- measured raster
    signal s_wcnt : unsigned(11 downto 0) := (others => '0');
    signal s_w2   : unsigned(10 downto 0) := to_unsigned(360, 11);
    signal s_hcnt : unsigned(10 downto 0) := (others => '0');
    signal s_h2   : unsigned(10 downto 0) := to_unsigned(120, 11);

    ----------------------------------------------------------------------
    -- vblank sequencer: one shared registered multiplier
    ----------------------------------------------------------------------
    signal s_seq  : unsigned(5 downto 0) := (others => '0');
    signal s_t    : unsigned(9 downto 0) := (others => '0');
    signal s_oct  : unsigned(2 downto 0) := (others => '0');
    signal s_mant : unsigned(7 downto 0) := (others => '0');
    signal s_dut  : unsigned(13 downto 0) := to_unsigned(128, 14);
    signal s_du   : unsigned(13 downto 0) := to_unsigned(128, 14);
    signal s_due  : unsigned(13 downto 0) := to_unsigned(128, 14);

    signal s_ma   : unsigned(10 downto 0) := (others => '0');
    signal s_mb   : unsigned(7 downto 0)  := (others => '0');
    signal s_mp   : unsigned(18 downto 0) := (others => '0');
    signal s_ph   : unsigned(16 downto 0) := (others => '0');

    signal s_offu : unsigned(23 downto 0) := (others => '0');
    signal s_offv : unsigned(23 downto 0) := (others => '0');
    signal s_modc : unsigned(23 downto 0) := (others => '0');

    signal s_u0    : unsigned(15 downto 0) := (others => '0');
    signal s_v0    : unsigned(16 downto 0) := (others => '0');
    signal s_v0o   : unsigned(16 downto 0) := (others => '0');
    signal s_dline : unsigned(14 downto 0) := to_unsigned(128, 15);

    -- drift origin accumulators
    signal s_ucen : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal s_vcen : unsigned(16 downto 0) := to_unsigned(63552, 17);

    -- glide pipeline scratch
    signal s_gdd  : signed(14 downto 0) := (others => '0');
    signal s_gsh  : signed(14 downto 0) := (others => '0');

    -- breathe oscillator
    signal s_bph  : unsigned(9 downto 0) := (others => '0');
    signal s_bam  : unsigned(8 downto 0) := (others => '0');
    signal s_bsg  : std_logic := '0';

    -- resolved palette (Classic defaults)
    signal s_pbg_y : unsigned(9 downto 0) := to_unsigned(184, 10);
    signal s_pbg_u : unsigned(9 downto 0) := to_unsigned(555, 10);
    signal s_pbg_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_por_y : unsigned(9 downto 0) := to_unsigned(504, 10);
    signal s_por_u : unsigned(9 downto 0) := to_unsigned(793, 10);
    signal s_por_v : unsigned(9 downto 0) := to_unsigned(357, 10);
    signal s_pre_y : unsigned(9 downto 0) := to_unsigned(331, 10);
    signal s_pre_u : unsigned(9 downto 0) := to_unsigned(768, 10);
    signal s_pre_v : unsigned(9 downto 0) := to_unsigned(467, 10);

    -- per-frame pattern thresholds (Weight / Hex Size), x16 fixed point
    signal s_hs    : unsigned(13 downto 0) := to_unsigned(1024, 14);
    signal s_hd    : signed(15 downto 0)   := to_signed(1168, 16);
    signal s_c1dl  : signed(15 downto 0)   := to_signed(22096, 16);
    signal s_c1dh  : signed(15 downto 0)   := to_signed(24432, 16);
    signal s_c3l   : signed(15 downto 0)   := to_signed(12408, 16);
    signal s_c3h   : signed(15 downto 0)   := to_signed(14744, 16);
    signal s_c5l   : signed(15 downto 0)   := to_signed(16936, 16);
    signal s_c5h   : signed(15 downto 0)   := to_signed(19272, 16);
    signal s_c5ul  : signed(15 downto 0)   := to_signed(-6328, 16);
    signal s_c5uh  : signed(15 downto 0)   := to_signed(-3992, 16);
    signal s_c2xl  : unsigned(13 downto 0) := to_unsigned(4960, 14);
    signal s_c2xh  : unsigned(13 downto 0) := to_unsigned(7008, 14);
    signal s_c4xl  : unsigned(13 downto 0) := to_unsigned(1264, 14);
    signal s_c4xh  : unsigned(13 downto 0) := to_unsigned(3312, 14);
    signal s_hr    : signed(16 downto 0)   := to_signed(2704, 17);
    signal s_hwr   : unsigned(13 downto 0) := to_unsigned(2336, 14);
    signal s_hx2t  : unsigned(13 downto 0) := to_unsigned(5856, 14);

    ----------------------------------------------------------------------
    -- texture accumulators
    ----------------------------------------------------------------------
    signal s_accu   : unsigned(15 downto 0) := (others => '0');
    signal s_linv   : unsigned(16 downto 0) := (others => '0');
    signal s_vline  : unsigned(16 downto 0) := (others => '0');
    signal s_avid_p : std_logic := '0';

    ----------------------------------------------------------------------
    -- pixel pipeline (all pattern coordinates carry 4 fractional bits)
    ----------------------------------------------------------------------
    signal r1_xf : unsigned(13 downto 0) := (others => '0');
    signal r1_v  : unsigned(14 downto 0) := (others => '0');
    signal r1_wh : unsigned(2 downto 0)  := (others => '0');

    signal r2_xf   : unsigned(13 downto 0) := (others => '0');
    signal r2_v    : unsigned(14 downto 0) := (others => '0');
    signal r2_p36  : unsigned(13 downto 0) := (others => '0');
    signal r2_p17  : unsigned(13 downto 0) := (others => '0');
    signal r2_p73  : unsigned(13 downto 0) := (others => '0');
    signal r2_p19  : unsigned(13 downto 0) := (others => '0');
    signal r2_h1   : unsigned(15 downto 0) := (others => '0');
    signal r2_h2   : unsigned(15 downto 0) := (others => '0');
    signal r2_wh   : unsigned(2 downto 0)  := (others => '0');

    signal r3_xf    : unsigned(13 downto 0) := (others => '0');
    signal r3_vm36  : signed(15 downto 0) := (others => '0');
    signal r3_vp17  : signed(15 downto 0) := (others => '0');
    signal r3_vm73  : signed(15 downto 0) := (others => '0');
    signal r3_vp19  : signed(15 downto 0) := (others => '0');
    signal r3_vm19  : signed(15 downto 0) := (others => '0');
    signal r3_h1    : unsigned(15 downto 0) := (others => '0');
    signal r3_h2    : unsigned(15 downto 0) := (others => '0');
    signal r3_hx1, r3_hx2 : std_logic := '0';
    signal r3_wh    : unsigned(2 downto 0) := (others => '0');

    signal r4_code : unsigned(1 downto 0) := (others => '0');
    signal r4_wh   : unsigned(2 downto 0) := (others => '0');

    -- input video delay (for S9 Video Ground)
    type t_vd is array (0 to 3) of unsigned(9 downto 0);
    signal vd_y, vd_u, vd_v : t_vd := (others => (others => '0'));

    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- sync delay
    ----------------------------------------------------------------------
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- raster measurement (height updated on active lines, never at vsync)
    ------------------------------------------------------------------------
    p_meas : process(clk)
        variable v_h : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_wcnt <= s_wcnt + 1;
            elsif s_avid_p = '1' then
                s_w2   <= s_wcnt(11 downto 1);
                s_wcnt <= (others => '0');
            end if;

            if s_vs_pulse = '1' then
                s_hcnt <= (others => '0');
            elsif data_in.avid = '1' and s_avid_p = '0' then
                v_h    := s_hcnt + 1;
                s_hcnt <= v_h;
                s_h2   <= '0' & v_h(10 downto 1);
            end if;
        end if;
    end process p_meas;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer (63 steps).
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_h2e : unsigned(10 downto 0);
        variable v_vo  : unsigned(17 downto 0);
        variable v_dd  : signed(14 downto 0);
        variable v_st  : signed(14 downto 0);
        variable v_gk  : integer range 1 to 8;
        variable v_tri : signed(10 downto 0);
        variable v_dl  : unsigned(13 downto 0);
        variable v_dk  : signed(10 downto 0);
        variable v_d9  : signed(9 downto 0);
        variable v_c19 : signed(18 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;
            s_mp <= s_ma * s_mb;

            s_vs_pulse <= '0';

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_vs_pulse <= '1';

                s_k1  <= unsigned(registers_in(0));
                s_k2  <= unsigned(registers_in(1));
                s_k3  <= unsigned(registers_in(2));
                s_k4  <= unsigned(registers_in(3));
                s_k5  <= unsigned(registers_in(4));
                s_k6  <= unsigned(registers_in(5));
                s_sw  <= registers_in(6)(4 downto 0);
                s_p12 <= unsigned(registers_in(7));

                s_fpar <= data_in.field_n;
                if data_in.field_n /= s_fpar then
                    s_ilace <= '1';
                else
                    s_ilace <= '0';
                end if;

                s_seq <= to_unsigned(63, 6);

            elsif s_seq /= 0 and data_in.avid = '0' then
                s_seq <= s_seq - 1;

                if s_ilace = '1' then
                    v_h2e := s_h2(9 downto 0) & '0';
                else
                    v_h2e := s_h2;
                end if;

                case to_integer(s_seq) is

                    -- ==== zoom target: continuous octave law ====
                    when 63 => s_t <= to_unsigned(1023, 10) - s_p12;
                    when 62 => s_oct  <= s_t(9 downto 7);
                               s_mant <= to_unsigned(128, 8) + resize(s_t(6 downto 0), 8);
                    when 61 => s_dut <= resize(shift_right(
                                   shift_left(resize(s_mant, 16), to_integer(s_oct)), 2), 14);

                    -- ==== glide: variable-strength IIR (K3), S7 bypass,
                    -- pipelined over three steps (diff / shift / apply) so
                    -- the barrel shifter never chains with the adders ====
                    when 60 =>
                        s_gdd <= signed(resize(s_dut, 15)) - signed(resize(s_du, 15));
                    when 59 =>
                        v_gk := 1 + to_integer(s_k3(9 downto 7));
                        s_gsh <= shift_right(s_gdd, v_gk);
                    when 58 =>
                        if s_sw(0) = '0' then
                            s_du <= s_dut;
                        else
                            v_st := s_gsh;
                            if v_st = 0 and s_gdd /= 0 then
                                if s_gdd > 0 then v_st := to_signed(1, 15);
                                else              v_st := to_signed(-1, 15);
                                end if;
                            end if;
                            s_du <= unsigned(resize(signed(resize(s_du, 15)) + v_st, 14));
                        end if;

                    -- ==== breathe: slow +-3% triangle on du (S10) ====
                    when 57 =>
                        if s_sw(3) = '1' then
                            s_bph <= s_bph + 1;
                        end if;
                        if s_bph(9) = '0' then
                            v_tri := signed(resize(s_bph(8 downto 0), 11)) - to_signed(256, 11);
                        else
                            v_tri := to_signed(256, 11) - signed(resize(s_bph(8 downto 0), 11));
                        end if;
                        if v_tri < 0 then
                            s_bsg <= '1';
                            s_bam <= resize(unsigned(-v_tri), 9);
                        else
                            s_bsg <= '0';
                            s_bam <= resize(unsigned(v_tri), 9);
                        end if;
                    when 56 => s_ma <= resize(s_bam, 11);
                               s_mb <= "00" & s_du(13 downto 8);
                    when 55 => s_ma <= resize(s_bam, 11);
                               s_mb <= s_du(7 downto 0);
                    when 54 => s_ph <= resize(s_mp, 17);
                    when 53 =>
                        v_dl := resize(shift_right(shift_left(resize(s_ph, 22), 8)
                                                   + resize(s_mp, 22), 13), 14);
                        if s_sw(3) = '0' then
                            s_due <= s_du;
                        elsif s_bsg = '0' then
                            s_due <= s_du + v_dl;
                        else
                            s_due <= s_du - v_dl;
                        end if;

                    -- ==== off_u = due * W/2; u0 anchors to the drift origin ====
                    when 52 => s_ma <= s_w2;
                               s_mb <= "00" & s_due(13 downto 8);
                    when 51 => s_ma <= s_w2;
                               s_mb <= s_due(7 downto 0);
                    when 50 => s_ph <= resize(s_mp, 17);
                    when 49 => s_offu <= shift_left(resize(s_ph, 24), 8) + resize(s_mp, 24);
                    when 48 => s_u0 <= s_ucen - s_offu(15 downto 0);

                    -- ==== off_v = due * H/2 ====
                    when 47 => s_ma <= v_h2e;
                               s_mb <= "00" & s_due(13 downto 8);
                    when 46 => s_ma <= v_h2e;
                               s_mb <= s_due(7 downto 0);
                    when 45 => s_ph <= resize(s_mp, 17);
                    when 44 => s_offv <= shift_left(resize(s_ph, 24), 8) + resize(s_mp, 24);
                               s_modc <= to_unsigned(C_VWRAP * 128, 24);

                    -- offv mod 93056: binary conditional-subtract ladder
                    when 43 | 42 | 41 | 40 | 39 | 38 | 37 | 36 =>
                        if s_offv >= s_modc then
                            s_offv <= s_offv - s_modc;
                        end if;
                        s_modc <= '0' & s_modc(23 downto 1);

                    when 35 =>
                        if s_offv(16 downto 0) > s_vcen then
                            v_vo := resize(s_vcen, 18) + to_unsigned(C_VWRAP, 18)
                                    - resize(s_offv(16 downto 0), 18);
                            s_v0 <= v_vo(16 downto 0);
                        else
                            s_v0 <= s_vcen - s_offv(16 downto 0);
                        end if;

                    when 34 =>
                        v_vo := resize(s_v0, 18) + resize(s_due, 18);
                        if v_vo >= to_unsigned(C_VWRAP, 18) then
                            v_vo := v_vo - to_unsigned(C_VWRAP, 18);
                        end if;
                        s_v0o <= v_vo(16 downto 0);

                    when 33 =>
                        if s_ilace = '1' then
                            s_dline <= s_due & '0';
                        else
                            s_dline <= '0' & s_due;
                        end if;

                    -- ==== drift accumulators (K1/K2, deadband 8) ====
                    when 32 =>
                        v_dk := signed('0' & s_k1) - to_signed(512, 11);
                        if v_dk > 8 then
                            v_d9 := resize(shift_right(v_dk - 8, 1), 10);
                        elsif v_dk < -8 then
                            v_d9 := resize(shift_right(v_dk + 8, 1), 10);
                        else
                            v_d9 := (others => '0');
                        end if;
                        s_ucen <= unsigned(signed(s_ucen) + resize(v_d9, 16));
                    when 31 =>
                        v_dk := signed('0' & s_k2) - to_signed(512, 11);
                        if v_dk > 8 then
                            v_d9 := resize(shift_right(v_dk - 8, 1), 10);
                        elsif v_dk < -8 then
                            v_d9 := resize(shift_right(v_dk + 8, 1), 10);
                        else
                            v_d9 := (others => '0');
                        end if;
                        v_c19 := resize(signed('0' & s_vcen), 19) + resize(v_d9, 19);
                        if v_c19 < 0 then
                            v_c19 := v_c19 + to_signed(C_VWRAP, 19);
                        elsif v_c19 >= to_signed(C_VWRAP, 19) then
                            v_c19 := v_c19 - to_signed(C_VWRAP, 19);
                        end if;
                        s_vcen <= unsigned(v_c19(16 downto 0));

                    -- ==== Weight (K4): stroke half-width + slab bounds ====
                    when 30 => s_hs <= to_unsigned(320, 14) + resize(s_k4, 14)
                                       + resize(s_k4(9 downto 2), 14);
                    when 29 => s_hd <= signed(resize(s_hs + shift_right(s_hs, 3)
                                       + shift_right(s_hs, 6), 16));
                    when 28 => s_c2xl <= to_unsigned(5984, 14) - s_hs;
                               s_c2xh <= to_unsigned(5984, 14) + s_hs;
                    when 27 => s_c4xl <= to_unsigned(2288, 14) - s_hs;
                               s_c4xh <= to_unsigned(2288, 14) + s_hs;
                    when 26 => s_c1dl <= to_signed(23264, 16) - s_hd;
                               s_c1dh <= to_signed(23264, 16) + s_hd;
                    when 25 => s_c5l <= to_signed(18104, 16) - s_hd;
                               s_c5h <= to_signed(18104, 16) + s_hd;
                    when 24 => s_c5ul <= to_signed(-5160, 16) - s_hd;
                               s_c5uh <= to_signed(-5160, 16) + s_hd;
                    when 23 => s_c3l <= to_signed(13576, 16) - s_hd;
                               s_c3h <= to_signed(13576, 16) + s_hd;

                    -- ==== Hex Size (K5): hr = 1622 + 2.125*K5 ====
                    when 22 => s_hr <= signed(resize(to_unsigned(1622, 15)
                                       + resize(s_k5 & '0', 15)
                                       + resize(s_k5(9 downto 3), 15), 17));
                    when 21 => s_hwr <= resize(unsigned(s_hr(14 downto 0))
                                        - shift_right(unsigned(s_hr(14 downto 0)), 3)
                                        - shift_right(unsigned(s_hr(14 downto 0)), 6), 14);
                    when 20 => s_hx2t <= to_unsigned(8192, 14) - s_hwr;

                    -- ==== Scheme select (K6): five stepped palettes ====
                    when 19 =>
                        if s_k6 < 102 then                       -- Classic
                            s_pbg_y <= to_unsigned(184, 10);
                            s_pbg_u <= to_unsigned(555, 10);
                            s_pbg_v <= to_unsigned(512, 10);
                            s_por_y <= to_unsigned(504, 10);
                            s_por_u <= to_unsigned(793, 10);
                            s_por_v <= to_unsigned(357, 10);
                            s_pre_y <= to_unsigned(331, 10);
                            s_pre_u <= to_unsigned(768, 10);
                            s_pre_v <= to_unsigned(467, 10);
                        elsif s_k6 < 307 then                    -- Gold Room
                            s_pbg_y <= to_unsigned(159, 10);
                            s_pbg_u <= to_unsigned(538, 10);
                            s_pbg_v <= to_unsigned(477, 10);
                            s_por_y <= to_unsigned(709, 10);
                            s_por_u <= to_unsigned(673, 10);
                            s_por_v <= to_unsigned(255, 10);
                            s_pre_y <= to_unsigned(282, 10);
                            s_pre_u <= to_unsigned(729, 10);
                            s_pre_v <= to_unsigned(465, 10);
                        elsif s_k6 < 512 then                    -- Lobby Blue
                            s_pbg_y <= to_unsigned(136, 10);
                            s_pbg_u <= to_unsigned(500, 10);
                            s_pbg_v <= to_unsigned(546, 10);
                            s_por_y <= to_unsigned(514, 10);
                            s_por_u <= to_unsigned(414, 10);
                            s_por_v <= to_unsigned(645, 10);
                            s_pre_y <= to_unsigned(315, 10);
                            s_pre_u <= to_unsigned(755, 10);
                            s_pre_v <= to_unsigned(470, 10);
                        elsif s_k6 < 768 then                    -- Deep Night
                            s_pbg_y <= to_unsigned(100, 10);
                            s_pbg_u <= to_unsigned(521, 10);
                            s_pbg_v <= to_unsigned(519, 10);
                            s_por_y <= to_unsigned(239, 10);
                            s_por_u <= to_unsigned(625, 10);
                            s_por_v <= to_unsigned(451, 10);
                            s_pre_y <= to_unsigned(346, 10);
                            s_pre_u <= to_unsigned(782, 10);
                            s_pre_v <= to_unsigned(464, 10);
                        else                                     -- Mono Amber
                            s_pbg_y <= to_unsigned( 93, 10);
                            s_pbg_u <= to_unsigned(516, 10);
                            s_pbg_v <= to_unsigned(507, 10);
                            s_por_y <= to_unsigned(728, 10);
                            s_por_u <= to_unsigned(599, 10);
                            s_por_v <= to_unsigned(366, 10);
                            s_pre_y <= to_unsigned(453, 10);
                            s_pre_u <= to_unsigned(579, 10);
                            s_pre_v <= to_unsigned(406, 10);
                        end if;

                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- texture accumulators
    ------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_vn : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            s_avid_p <= data_in.avid;

            if s_vs_pulse = '1' then
                if s_ilace = '1' and data_in.field_n = '1' then
                    s_linv <= s_v0o;
                else
                    s_linv <= s_v0;
                end if;
            elsif data_in.avid = '1' and s_avid_p = '0' then
                s_accu  <= s_u0;
                s_vline <= s_linv;
                v_vn := resize(s_linv, 18) + resize(s_dline, 18);
                if v_vn >= to_unsigned(C_VWRAP, 18) then
                    v_vn := v_vn - to_unsigned(C_VWRAP, 18);
                end if;
                s_linv <= v_vn(16 downto 0);
            elsif data_in.avid = '1' then
                s_accu <= s_accu + resize(s_due, 16);
            end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- input video delay (aligns the source with the pattern latency)
    ------------------------------------------------------------------------
    p_vd : process(clk)
    begin
        if rising_edge(clk) then
            vd_y(0) <= unsigned(data_in.y);
            vd_u(0) <= unsigned(data_in.u);
            vd_v(0) <= unsigned(data_in.v);
            for i in 1 to 3 loop
                vd_y(i) <= vd_y(i - 1);
                vd_u(i) <= vd_u(i - 1);
                vd_v(i) <= vd_v(i - 1);
            end loop;
        end if;
    end process p_vd;

    ------------------------------------------------------------------------
    -- R1: fold u to |x| (14-bit, 4 frac bits), raw v, weave hash bits.
    ------------------------------------------------------------------------
    p_r1 : process(clk)
        variable v_u : unsigned(13 downto 0);
        variable v_s : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            v_u := s_accu(15 downto 2);
            if v_u(13) = '1' then
                v_s := to_signed(16384, 15) - signed('0' & v_u);
                r1_xf <= unsigned(v_s(13 downto 0));
            else
                r1_xf <= v_u;
            end if;
            r1_v <= s_vline(16 downto 2);

            r1_wh <= (s_accu(8 downto 6) xor s_accu(11 downto 9))
                     xor (s_vline(8 downto 6) xor s_vline(11 downto 9));
        end if;
    end process p_r1;

    ------------------------------------------------------------------------
    -- R2: the five shared x-products (single truncation) + hex sums.
    ------------------------------------------------------------------------
    p_r2 : process(clk)
        variable v_x   : unsigned(13 downto 0);
        variable v_x2  : unsigned(13 downto 0);
        variable v_w   : unsigned(20 downto 0);
        variable v_d1, v_d2 : signed(15 downto 0);
        variable v_a1, v_a2 : unsigned(14 downto 0);
        variable v_p37 : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            v_x := r1_xf;
            r2_xf <= v_x;
            r2_v  <= r1_v;
            r2_wh <= r1_wh;

            v_w := shift_left(resize(v_x, 21), 5) + shift_left(resize(v_x, 21), 2);
            r2_p36 <= resize(shift_right(v_w, 6), 14);
            v_w := shift_left(resize(v_x, 21), 4) + resize(v_x, 21);
            r2_p17 <= resize(shift_right(v_w, 5), 14);
            v_w := shift_left(resize(v_x, 21), 6) + shift_left(resize(v_x, 21), 3)
                   + resize(v_x, 21);
            r2_p73 <= resize(shift_right(v_w, 7), 14);
            v_w := shift_left(resize(v_x, 21), 4) + shift_left(resize(v_x, 21), 1)
                   + resize(v_x, 21);
            r2_p19 <= resize(shift_right(v_w, 5), 14);

            v_d1 := signed('0' & r1_v) - to_signed(6880, 16);
            if v_d1 < 0 then v_d1 := -v_d1; end if;
            v_a1 := unsigned(v_d1(14 downto 0));
            v_w := shift_left(resize(v_x, 21), 5) + shift_left(resize(v_x, 21), 2)
                   + resize(v_x, 21);
            v_p37 := resize(shift_right(v_w, 6), 14);
            r2_h1 <= resize(v_a1, 16) + resize(v_p37, 16);

            v_x2 := to_unsigned(8192, 14) - v_x;
            v_d2 := signed('0' & r1_v) - to_signed(15888, 16);
            if v_d2 < 0 then v_d2 := -v_d2; end if;
            v_a2 := unsigned(v_d2(14 downto 0));
            v_w := shift_left(resize(v_x2, 21), 5) + shift_left(resize(v_x2, 21), 2)
                   + resize(v_x2, 21);
            v_p37 := resize(shift_right(v_w, 6), 14);
            r2_h2 <= resize(v_a2, 16) + resize(v_p37, 16);
        end if;
    end process p_r2;

    ------------------------------------------------------------------------
    -- R3: the five signed plane sums + hex bound tests.
    ------------------------------------------------------------------------
    p_r3 : process(clk)
        variable v_sv : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            v_sv := signed('0' & r2_v);
            r3_vm36 <= v_sv - signed(resize(r2_p36, 16));
            r3_vp17 <= v_sv + signed(resize(r2_p17, 16));
            r3_vm73 <= v_sv - signed(resize(r2_p73, 16));
            r3_vp19 <= v_sv + signed(resize(r2_p19, 16));
            r3_vm19 <= v_sv - signed(resize(r2_p19, 16));

            r3_h1 <= r2_h1;
            r3_h2 <= r2_h2;
            if r2_xf <= s_hwr then r3_hx1 <= '1'; else r3_hx1 <= '0'; end if;
            if r2_xf >= s_hx2t then r3_hx2 <= '1'; else r3_hx2 <= '0'; end if;

            r3_xf <= r2_xf;
            r3_wh <= r2_wh;
        end if;
    end process p_r3;

    ------------------------------------------------------------------------
    -- R4: the serpentine (per-frame slab bounds, fixed miter clips) and
    -- the red hexagons.  Red > orange > ground.
    ------------------------------------------------------------------------
    p_r4 : process(clk)
        variable s1, s1d, s2, s3, s4, s5, s5u : std_logic;
        variable v_org, v_red : std_logic;
    begin
        if rising_edge(clk) then
            s1 := '0'; s1d := '0'; s2 := '0'; s3 := '0';
            s4 := '0'; s5 := '0'; s5u := '0';

            if r3_vm36 >= -s_hd and r3_vm36 <= s_hd
               and r3_vp19 <= to_signed(6912, 16) then
                s1 := '1';
            end if;
            if r3_vm36 >= s_c1dl and r3_vm36 <= s_c1dh
               and r3_vp19 <= to_signed(30176, 16) then
                s1d := '1';
            end if;
            if r3_xf >= s_c2xl and r3_xf <= s_c2xh
               and r3_vp19 >= to_signed(6912, 16)
               and r3_vm19 <= to_signed(6848, 16) then
                s2 := '1';
            end if;
            if r3_vp17 >= s_c3l and r3_vp17 <= s_c3h
               and r3_vm19 >= to_signed(6848, 16)
               and r3_vm19 <= to_signed(11008, 16) then
                s3 := '1';
            end if;
            if r3_xf >= s_c4xl and r3_xf <= s_c4xh
               and r3_vm19 >= to_signed(11008, 16)
               and r3_vp19 <= to_signed(20768, 16) then
                s4 := '1';
            end if;
            if r3_vm73 >= s_c5l and r3_vm73 <= s_c5h
               and r3_vp19 >= to_signed(20768, 16) then
                s5 := '1';
            end if;
            if r3_vm73 >= s_c5ul and r3_vm73 <= s_c5uh then
                s5u := '1';
            end if;

            v_org := s1 or s1d or s2 or s3 or s4 or s5 or s5u;

            v_red := '0';
            if (r3_hx1 = '1' and signed('0' & r3_h1) <= s_hr)
               or (r3_hx2 = '1' and signed('0' & r3_h2) <= s_hr) then
                v_red := '1';
            end if;

            if v_red = '1' then
                r4_code <= "10";
            elsif v_org = '1' then
                r4_code <= "01";
            else
                r4_code <= "00";
            end if;

            r4_wh <= r3_wh;
        end if;
    end process p_r4;

    ------------------------------------------------------------------------
    -- R5: paint.  Scheme-resolved palette; Night dims field and orange;
    -- Video Ground swaps the field for dimmed input; Weave adds a
    -- texture-anchored fibre dither to luma.
    ------------------------------------------------------------------------
    p_r5 : process(clk)
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_ys : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            case to_integer(r4_code) is
                when 1 =>
                    if s_sw(4) = '1' then    -- Night: orange at half
                        v_y := '0' & s_por_y(9 downto 1);
                        v_u := ('0' & s_por_u(9 downto 1)) + to_unsigned(256, 10);
                        v_v := ('0' & s_por_v(9 downto 1)) + to_unsigned(256, 10);
                    else
                        v_y := s_por_y; v_u := s_por_u; v_v := s_por_v;
                    end if;
                when 2 =>
                    v_y := s_pre_y; v_u := s_pre_u; v_v := s_pre_v;
                when others =>
                    if s_sw(2) = '1' then    -- Video Ground
                        v_y := ('0' & vd_y(3)(9 downto 1)) + to_unsigned(92, 10);
                        v_u := ('0' & vd_u(3)(9 downto 1)) + to_unsigned(256, 10);
                        v_v := ('0' & vd_v(3)(9 downto 1)) + to_unsigned(256, 10);
                    elsif s_sw(4) = '1' then -- Night: field at quarter
                        v_y := "00" & s_pbg_y(9 downto 2);
                        v_u := ("00" & s_pbg_u(9 downto 2)) + to_unsigned(384, 10);
                        v_v := ("00" & s_pbg_v(9 downto 2)) + to_unsigned(384, 10);
                    else
                        v_y := s_pbg_y; v_u := s_pbg_u; v_v := s_pbg_v;
                    end if;
            end case;

            if s_sw(1) = '1' then            -- Weave fibre dither on luma
                v_ys := signed(resize(v_y, 12)) + signed(resize(r4_wh & "00", 12))
                        - to_signed(14, 12);
                if v_ys < 0 then
                    v_ys := (others => '0');
                end if;
                v_y := unsigned(v_ys(9 downto 0));
            end if;

            s_out_y <= v_y;
            s_out_u <= v_u;
            s_out_v <= v_v;
        end if;
    end process p_r5;

    ------------------------------------------------------------------------
    -- sync delay + output
    ------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= data_in.avid    & s_avid_sr   (0 to C_LATENCY - 2);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture overlook;
