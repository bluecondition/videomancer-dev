-- Videomancer SDK - Open source FPGA-based video effects development kit
-- Copyright (C) 2025 LZX Industries LLC
-- File: wrinkle.vhd - Wrinkle for Videomancer
-- License: GNU General Public License v3.0
-- https://github.com/lzxindustries/videomancer-sdk
--
-- This file is free software: you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.
--
-- This program is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with this program. If not, see <https://www.gnu.org/licenses/>.
--
-- Program Name:        Wrinkle
-- Author:              Adam Pflanzer
-- Overview:
--   Per-channel wavefolding effect with selectable waveform shape, continuous
--   fold frequency, phase offset, symmetry control, and optional RGB colorspace
--   conversion. Inspired by audio wavefolding / Buchla-style processing applied
--   to video signals.
--
--   Fold frequency is continuous (1x to ~64x) via inline multiply. Each channel
--   value is multiplied by a frequency factor, and the product bits encode the
--   fold position and direction for triangle/sine/saw/square shape output.
--
--   Parameters are latched on vsync to prevent mid-frame tearing.
--
-- Wrinkle Lab (czartroniks fork of homegrownvhs/wrinkle v2.1.0):
--   * v2.2.0: input LPF restored to a true first-order IIR. v2.1.0's
--     two-stage timing split made it ring undamped at a 6-pixel period at
--     Smoothing 0 (see p_lpf). Blanking gate at the output register.
--   * v2.4.0: wet-path output guard (HD HDMI thick-bar fix): legal-range
--     clamps, [1 2 2 2 2 2 2 2 1]/16 on U/V, [1 2 1]/4 on Y, luma knee
--     (white ~767). HW bisect on one placement (diag 2, 2026-10-20): the
--     WIDE CHROMA smoothing is what stops the bars; [1 2 1] chroma barred
--     (so did damier's guard), while a wider luma filter or a dimmer luma
--     scale made no difference.
--
-- Resources:
--   0 BRAM
--
-- Pipeline (clocks after data_in):
--   IIR LPF (pre-shift, loop) + source register     T+3
--   Stage 0 (CSC forward)                            T+4
--   Stage 1 (phase + symmetry offset)                T+5
--   Stage 2 (frequency multiply)                     T+6
--   Stage 2.5 (sin_cos register, timing fix)         T+7
--   Stage 3 (shape select)                           T+8
--   Stage 4a/4b (CSC reverse; brightness + clamp)    T+10
--   Guard (window centre, 3 sum stages, luma scale)  T+18
--   interpolator_u x3 (wet/dry mix)                  T+22
--   Output register (bypass mux + blanking gate)     T+23
--   Pad register                                     T+24
--
-- Submodules:
--   inline IIR LPF x3: line-reset IIR LPF (18-bit precision, no cross-line bleed)
--   sin_cos_full_lut_10x10 x3: sine lookup, combinational
--   interpolator_u x3: linear blend for dry/wet mix, 4 clocks each
--
-- Parameters:
--   Pot 1  (registers_in(0)):   Fold Amount (continuous 1x-64x)
--   Pot 2  (registers_in(1)):   Phase (0-360 degrees)
--   Pot 3  (registers_in(2)):   Symmetry (DC offset before fold, center=none)
--   Pot 4  (registers_in(3)):   Chroma Offset (chroma freq offset, center=same)
--   Pot 5  (registers_in(4)):   Smoothing (LPF cutoff on source video, 0=off)
--   Pot 6  (registers_in(5)):   Brightness (post-fold Y offset, center=none)
--   Tog 7  (registers_in(6)(0)): Shape bit 0 (00=Tri,01=Sin,10=Saw,11=Sqr)
--   Tog 8  (registers_in(6)(1)): Shape bit 1
--   Tog 9  (registers_in(6)(2)): Cspace bit 0 (00=YUV,01=RGB,10=XYZ,11=Y-only)
--   Tog 10 (registers_in(6)(3)): Cspace bit 1
--   Tog 11 (registers_in(6)(4)): Bypass
--   Fader  (registers_in(7)):    Mix (dry/wet)
--
-- Timing:
--   One latency in every mode, bypass included: 24 clocks
--   (processing_delay_clks = 24 in the TOML).

--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture wrinkle of program_top is

    -- Output stream record (output register + pads)
    type t_io_align_stream is record
        y       : std_logic_vector(10-1 downto 0);
        u       : std_logic_vector(10-1 downto 0);
        v       : std_logic_vector(10-1 downto 0);
        avid    : std_logic;
        hsync_n : std_logic;
        vsync_n : std_logic;
        field_n : std_logic;
    end record;
    signal s_out   : t_io_align_stream;   -- output register, T+23
    signal s_pad_0 : t_io_align_stream;   -- T+24 = processing_delay_clks

    constant C_VIDEO_DATA_WIDTH : integer := 10;
    -- Tap latencies from data_in, in clocks
    constant C_YONLY_UV_CLKS : integer := 8;   -- dry chroma read by stage 4a (Y-only)
    constant C_DRY_CLKS      : integer := 18;  -- dry video into the mix, beside the guarded wet
    constant C_TAP_CLKS      : integer := 22;  -- syncs + bypass video into the output register

    -- Frequency encoding: freq_factor = (knob >> 2) + 4 = 4..259 (8 bits)
    -- product = channel * freq_factor (10x8 = 18 bits)
    -- Normalized: product >> 2, at factor=4 gives 1x, at factor=259 gives ~65x
    constant C_FREQ_OFFSET : unsigned(7 downto 0) := to_unsigned(4, 8);

    -- Vsync-latched parameters (prevents mid-frame tearing)
    signal r_freq_knob       : unsigned(9 downto 0) := to_unsigned(128, 10);
    signal r_phase           : unsigned(9 downto 0) := (others => '0');
    signal r_symmetry        : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_chroma_offset   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_smoothing       : unsigned(7 downto 0) := (others => '0');
    signal r_brightness      : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal r_shape           : std_logic_vector(1 downto 0) := "00";
    signal r_csc_mode        : std_logic_vector(1 downto 0) := "00";
    signal s_bypass          : std_logic;
    signal s_mix             : unsigned(9 downto 0);

    -- Computed frequency factors
    signal r_luma_freq   : unsigned(7 downto 0) := to_unsigned(36, 8);
    signal r_chroma_freq : unsigned(7 downto 0) := to_unsigned(36, 8);

    -- Vsync edge detection
    signal s_prev_vsync : std_logic := '1';

    -- Stage 0 outputs: input register + optional CSC forward
    signal s0_ch0 : unsigned(9 downto 0) := (others => '0');
    signal s0_ch1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s0_ch2 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Stage 1 outputs: phase + symmetry offset
    signal s1_ch0 : unsigned(9 downto 0) := (others => '0');
    signal s1_ch1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s1_ch2 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Stage 2 outputs: frequency multiply products (18 bits each)
    signal s2_prod0 : unsigned(17 downto 0) := (others => '0');
    signal s2_prod1 : unsigned(17 downto 0) := (others => '0');
    signal s2_prod2 : unsigned(17 downto 0) := (others => '0');

    -- Stage 3 outputs: fold + shape select
    signal s3_ch0 : unsigned(9 downto 0) := (others => '0');
    signal s3_ch1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s3_ch2 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Stage 4 outputs: invert + CSC reverse + clamp
    signal s4_y : unsigned(9 downto 0) := (others => '0');
    signal s4_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s4_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Stage 4a: CSC sum-of-shifted-terms result (signed 12-bit, before
    -- brightness add and chroma +512 offset). Splitting stage 4 into 4a
    -- (CSC compute) and 4b (brightness + offset + clamp) breaks the deep
    -- multi-add chain that failed HD timing on the hd_hdmi build (path
    -- was retimed from s3_ch* through stage-4 logic into the interpolator's
    -- stage-0 input registers).
    signal s4a_y : signed(11 downto 0) := (others => '0');
    signal s4a_u : signed(11 downto 0) := (others => '0');
    signal s4a_v : signed(11 downto 0) := (others => '0');
    -- '1' in CSC modes 01/10 -> stage 4b adds +512 to U/V before clamp.
    signal s4a_chroma_offset : std_logic := '0';

    -- Triangle fold from product bits (combinational)
    signal s_tri0 : unsigned(9 downto 0);
    signal s_tri1 : unsigned(9 downto 0);
    signal s_tri2 : unsigned(9 downto 0);

    -- Sine LUT outputs (combinational)
    signal s_sin0_raw : signed(9 downto 0);
    signal s_sin1_raw : signed(9 downto 0);
    signal s_sin2_raw : signed(9 downto 0);

    -- Stage 2.5: registered pipeline to break long sin_cos combinational path
    signal s2p5_sin0  : signed(9 downto 0)   := (others => '0');
    signal s2p5_sin1  : signed(9 downto 0)   := (others => '0');
    signal s2p5_sin2  : signed(9 downto 0)   := (others => '0');
    signal s2p5_tri0  : unsigned(9 downto 0) := (others => '0');
    signal s2p5_tri1  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s2p5_tri2  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s2p5_saw0  : unsigned(9 downto 0) := (others => '0');
    signal s2p5_saw1  : unsigned(9 downto 0) := (others => '0');
    signal s2p5_saw2  : unsigned(9 downto 0) := (others => '0');
    signal s2p5_sign0 : std_logic := '0';
    signal s2p5_sign1 : std_logic := '0';
    signal s2p5_sign2 : std_logic := '0';

    -- Dry taps, bypass + sync taps (T+22), mix outputs (T+22)
    signal s_dry_y        : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_dry_u        : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_dry_v        : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_dry8_u       : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_dry8_v       : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bypass_y     : std_logic_vector(9 downto 0) := (others => '0');
    signal s_bypass_u     : std_logic_vector(9 downto 0) := (others => '0');
    signal s_bypass_v     : std_logic_vector(9 downto 0) := (others => '0');
    signal s_tap_hsync_n  : std_logic := '1';
    signal s_tap_vsync_n  : std_logic := '1';
    signal s_tap_field_n  : std_logic := '1';
    signal s_tap_avid     : std_logic := '0';
    signal s_mix_y        : unsigned(9 downto 0);
    signal s_mix_u        : unsigned(9 downto 0);
    signal s_mix_v        : unsigned(9 downto 0);

    -- Output guard on the wet path: 9-pixel window w(1..8) behind s4 (= w(0)),
    -- centre w(4); sum stages A, B, C; luma knee -> c2 (T+18)
    type t_win is array (1 to 8) of unsigned(9 downto 0);
    signal w_y : t_win := (others => to_unsigned(64, 10));
    signal w_u, w_v : t_win := (others => to_unsigned(512, 10));
    signal ya_121, yb_121, yc_121 : unsigned(11 downto 0) := (others => '0');
    signal ua_12, ua_35, ua_67, ua_08 : unsigned(10 downto 0) := (others => '0');
    signal va_12, va_35, va_67, va_08 : unsigned(10 downto 0) := (others => '0');
    signal ua_4, va_4 : unsigned(9 downto 0) := (others => '0');
    signal ub_r1, ub_r2, vb_r1, vb_r2 : unsigned(11 downto 0) := (others => '0');
    signal ub_08, vb_08 : unsigned(10 downto 0) := (others => '0');
    signal uc_s9, vc_s9 : unsigned(13 downto 0) := (others => '0');
    signal c2_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal c2_u, c2_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Inline IIR LPF with line-boundary reset and extended precision:
    -- 18-bit unsigned state (10 integer + 8 fractional) per channel.
    constant C_IIR_FRAC  : integer := 8;
    constant C_IIR_WIDTH : integer := 10 + C_IIR_FRAC;  -- 18 bits
    signal s_iir_y          : unsigned(C_IIR_WIDTH - 1 downto 0) := (others => '0');
    signal s_iir_u          : unsigned(C_IIR_WIDTH - 1 downto 0) := (others => '0');
    signal s_iir_v          : unsigned(C_IIR_WIDTH - 1 downto 0) := (others => '0');
    -- Stage A: the input pre-shifted by this pixel's k, the k itself, valid
    signal s_xk_y           : unsigned(C_IIR_WIDTH - 1 downto 0) := (others => '0');
    signal s_xk_u           : unsigned(C_IIR_WIDTH - 1 downto 0) := (others => '0');
    signal s_xk_v           : unsigned(C_IIR_WIDTH - 1 downto 0) := (others => '0');
    signal s_k_use_q        : natural range 0 to 15 := 0;
    signal s_lpf_valid      : std_logic := '0';
    -- k for the next pixel (sigma-delta between k_coarse and k_coarse + 1)
    signal s_k_cur          : natural range 0 to 15 := 0;
    signal s_line_first_pix : std_logic := '1';
    signal s_prev_hsync_lpf : std_logic := '1';
    signal s_lpf_mix_acc    : unsigned(3 downto 0) := (others => '0');
    -- Registered source: breaks the s_iir -> CSC -> s0_ch* chain
    signal s_src_y          : unsigned(9 downto 0) := (others => '0');
    signal s_src_u          : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_src_v          : unsigned(9 downto 0) := to_unsigned(512, 10);

begin

    -- Bypass and mix are NOT latched (need immediate response)
    s_bypass <= registers_in(6)(4);
    s_mix    <= unsigned(registers_in(7)(9 downto 0));

    -- =========================================================================
    -- Input LPF: first-order IIR, one pixel per clock, reset per line.
    --     I <= I + X/2^k - I/2^k        (k = 0 passes X straight through)
    -- v2.1.0 split the loop for timing into an error register and an update
    -- (E <= X - I; I <= I + E>>k), so each update used the error of the pixel
    -- before. At k = 0 (Smoothing 0, the default) that is
    --     I(n) = I(n-1) - I(n-2) + X(n-1)
    -- with poles on the unit circle: every edge launched an undamped 6-pixel
    -- ring that ran to the end of the line, overflowing past 1023 on hard
    -- edges. Here the input's barrel shift sits outside the loop (stage A)
    -- and the loop is (I + X>>k) - (I>>k). The first pixel of each line uses
    -- k = 0, an exact snap (I = X), so nothing bleeds across lines.
    -- Sigma-delta mixing of k and k+1 from the cutoff's low 4 bits gives
    -- 256 smooth cutoff steps.
    -- =========================================================================
    p_lpf : process(clk)
        variable v_k        : natural range 0 to 15;
        variable v_k_coarse : natural range 0 to 15;
        variable v_acc_next : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            -- Stage A: this pixel's k and the input pre-shifted by it
            if s_line_first_pix = '1' then
                v_k := 0;
            else
                v_k := s_k_cur;
            end if;
            s_xk_y      <= shift_right(unsigned(data_in.y) & to_unsigned(0, C_IIR_FRAC), v_k);
            s_xk_u      <= shift_right(unsigned(data_in.u) & to_unsigned(0, C_IIR_FRAC), v_k);
            s_xk_v      <= shift_right(unsigned(data_in.v) & to_unsigned(0, C_IIR_FRAC), v_k);
            s_k_use_q   <= v_k;
            s_lpf_valid <= data_in.avid;

            if data_in.avid = '1' then
                s_line_first_pix <= '0';
                v_k_coarse := to_integer(r_smoothing(7 downto 4));
                v_acc_next := resize(s_lpf_mix_acc, 5) + resize(r_smoothing(3 downto 0), 5);
                s_lpf_mix_acc <= v_acc_next(3 downto 0);
                if v_acc_next(4) = '1' and v_k_coarse < 15 then
                    s_k_cur <= v_k_coarse + 1;
                else
                    s_k_cur <= v_k_coarse;
                end if;
            end if;

            -- Stage B: the loop, one clock
            if s_lpf_valid = '1' then
                s_iir_y <= (s_iir_y + s_xk_y) - shift_right(s_iir_y, s_k_use_q);
                s_iir_u <= (s_iir_u + s_xk_u) - shift_right(s_iir_u, s_k_use_q);
                s_iir_v <= (s_iir_v + s_xk_v) - shift_right(s_iir_v, s_k_use_q);
            end if;

            -- Detect line boundaries
            s_prev_hsync_lpf <= data_in.hsync_n;
            if s_prev_hsync_lpf = '1' and data_in.hsync_n = '0' then
                s_line_first_pix <= '1';
            end if;
        end if;
    end process p_lpf;

    -- Source register (integer part of the IIR state; it never exceeds 1023)
    p_src_reg : process(clk)
    begin
        if rising_edge(clk) then
            s_src_y <= s_iir_y(C_IIR_WIDTH - 1 downto C_IIR_FRAC);
            s_src_u <= s_iir_u(C_IIR_WIDTH - 1 downto C_IIR_FRAC);
            s_src_v <= s_iir_v(C_IIR_WIDTH - 1 downto C_IIR_FRAC);
        end if;
    end process p_src_reg;

    -- =========================================================================
    -- Parameter latch on vsync (prevents mid-frame tearing)
    -- =========================================================================
    p_param_latch : process(clk)
        variable v_luma_freq_wide : unsigned(8 downto 0);  -- 9 bits to avoid overflow
        variable v_chroma_off_s   : signed(9 downto 0);
        variable v_chroma_freq_s  : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            if s_prev_vsync = '1' and data_in.vsync_n = '0' then
                r_freq_knob     <= unsigned(registers_in(0)(9 downto 0));
                r_phase         <= unsigned(registers_in(1)(9 downto 0));
                r_symmetry      <= unsigned(registers_in(2)(9 downto 0));
                r_chroma_offset <= unsigned(registers_in(3)(9 downto 0));
                r_smoothing     <= unsigned(registers_in(4)(9 downto 2));
                r_brightness    <= unsigned(registers_in(5)(9 downto 0));
                r_shape         <= registers_in(6)(1) & registers_in(6)(0);
                r_csc_mode      <= registers_in(6)(3) & registers_in(6)(2);

                -- Luma frequency factor: (knob >> 2) + 4 = 4..259
                -- Use 9-bit to avoid overflow, clamp to 255
                v_luma_freq_wide := resize(unsigned(registers_in(0)(9 downto 2)), 9)
                                  + resize(C_FREQ_OFFSET, 9);
                if v_luma_freq_wide > 255 then
                    r_luma_freq <= to_unsigned(255, 8);
                else
                    r_luma_freq <= v_luma_freq_wide(7 downto 0);
                end if;

                -- Chroma frequency: luma + offset from center
                -- chroma_knob centered at 512: offset = (chroma_knob >> 1) - 256
                v_chroma_off_s := signed('0' & registers_in(3)(9 downto 1)) -
                                  to_signed(256, 10);
                v_chroma_freq_s := signed(resize(v_luma_freq_wide, 10)) +
                                   resize(v_chroma_off_s, 10);
                if v_chroma_freq_s < 4 then
                    r_chroma_freq <= to_unsigned(4, 8);
                elsif v_chroma_freq_s > 255 then
                    r_chroma_freq <= to_unsigned(255, 8);
                else
                    r_chroma_freq <= unsigned(v_chroma_freq_s(7 downto 0));
                end if;
            end if;
        end if;
    end process p_param_latch;

    -- =========================================================================
    -- Stage 0 (T+1): Input register + optional CSC forward
    -- 00=YUV (pass), 01=RGB, 10=XYZ, 11=Y-only
    -- =========================================================================
    p_stage0 : process(clk)
        variable v_u_s : signed(10 downto 0);
        variable v_v_s : signed(10 downto 0);
        variable v_r   : signed(11 downto 0);
        variable v_g   : signed(11 downto 0);
        variable v_b   : signed(11 downto 0);
        variable v_x   : signed(11 downto 0);
        variable v_yx  : signed(11 downto 0);
        variable v_z   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            case r_csc_mode is
                when "01" =>
                    -- RGB mode: Approximate BT.601 YUV -> RGB
                    v_u_s := signed(resize(s_src_u, 11)) - to_signed(512, 11);
                    v_v_s := signed(resize(s_src_v, 11)) - to_signed(512, 11);

                    v_r := signed(resize(s_src_y, 12)) +
                           resize(v_v_s, 12) +
                           resize(shift_right(v_v_s, 2), 12) +
                           resize(shift_right(v_v_s, 3), 12);
                    v_g := signed(resize(s_src_y, 12)) -
                           resize(shift_right(v_u_s, 2), 12) -
                           resize(shift_right(v_u_s, 4), 12) -
                           resize(shift_right(v_v_s, 1), 12) -
                           resize(shift_right(v_v_s, 3), 12);
                    v_b := signed(resize(s_src_y, 12)) +
                           resize(v_u_s, 12) +
                           resize(shift_right(v_u_s, 1), 12) +
                           resize(shift_right(v_u_s, 2), 12);

                    if v_r < 0 then s0_ch0 <= (others => '0');
                    elsif v_r > 1023 then s0_ch0 <= to_unsigned(1023, 10);
                    else s0_ch0 <= unsigned(v_r(9 downto 0)); end if;

                    if v_g < 0 then s0_ch1 <= (others => '0');
                    elsif v_g > 1023 then s0_ch1 <= to_unsigned(1023, 10);
                    else s0_ch1 <= unsigned(v_g(9 downto 0)); end if;

                    if v_b < 0 then s0_ch2 <= (others => '0');
                    elsif v_b > 1023 then s0_ch2 <= to_unsigned(1023, 10);
                    else s0_ch2 <= unsigned(v_b(9 downto 0)); end if;

                when "10" =>
                    -- XYZ mode: Approximate YUV -> CIE XYZ
                    v_u_s := signed(resize(s_src_u, 11)) - to_signed(512, 11);
                    v_v_s := signed(resize(s_src_v, 11)) - to_signed(512, 11);

                    -- X ~ Y + 3/16*U_s + 5/16*V_s
                    v_x := signed(resize(s_src_y, 12)) +
                           resize(shift_right(v_u_s, 3), 12) +
                           resize(shift_right(v_u_s, 4), 12) +
                           resize(shift_right(v_v_s, 2), 12) +
                           resize(shift_right(v_v_s, 4), 12);
                    -- Y_xyz ~ Y - U_s/8 - V_s/4
                    v_yx := signed(resize(s_src_y, 12)) -
                            resize(shift_right(v_u_s, 3), 12) -
                            resize(shift_right(v_v_s, 2), 12);
                    -- Z ~ Y + 13/8*U_s - V_s/16
                    v_z := signed(resize(s_src_y, 12)) +
                           resize(v_u_s, 12) +
                           resize(shift_right(v_u_s, 1), 12) +
                           resize(shift_right(v_u_s, 3), 12) -
                           resize(shift_right(v_v_s, 4), 12);

                    if v_x < 0 then s0_ch0 <= (others => '0');
                    elsif v_x > 1023 then s0_ch0 <= to_unsigned(1023, 10);
                    else s0_ch0 <= unsigned(v_x(9 downto 0)); end if;

                    if v_yx < 0 then s0_ch1 <= (others => '0');
                    elsif v_yx > 1023 then s0_ch1 <= to_unsigned(1023, 10);
                    else s0_ch1 <= unsigned(v_yx(9 downto 0)); end if;

                    if v_z < 0 then s0_ch2 <= (others => '0');
                    elsif v_z > 1023 then s0_ch2 <= to_unsigned(1023, 10);
                    else s0_ch2 <= unsigned(v_z(9 downto 0)); end if;

                when "11" =>
                    -- Y-only mode: fold luma only, neutral chroma
                    s0_ch0 <= s_src_y;
                    s0_ch1 <= to_unsigned(512, 10);
                    s0_ch2 <= to_unsigned(512, 10);

                when others =>
                    -- YUV mode (00): direct passthrough
                    s0_ch0 <= s_src_y;
                    s0_ch1 <= s_src_u;
                    s0_ch2 <= s_src_v;
            end case;
        end if;
    end process p_stage0;

    -- =========================================================================
    -- Stage 1 (T+2): Phase + symmetry offset
    -- =========================================================================
    p_stage1 : process(clk)
        variable v_sym : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_sym := r_symmetry - to_unsigned(512, 10);

            s1_ch0 <= s0_ch0 + r_phase + v_sym;
            s1_ch1 <= s0_ch1 + r_phase + v_sym;
            s1_ch2 <= s0_ch2 + r_phase + v_sym;
        end if;
    end process p_stage1;

    -- =========================================================================
    -- Stage 2 (T+3): Frequency multiply
    -- =========================================================================
    p_stage2 : process(clk)
    begin
        if rising_edge(clk) then
            s2_prod0 <= s1_ch0 * r_luma_freq;
            s2_prod1 <= s1_ch1 * r_chroma_freq;
            s2_prod2 <= s1_ch2 * r_chroma_freq;
        end if;
    end process p_stage2;

    -- =========================================================================
    -- Triangle fold (combinational from stage 2 products)
    -- Direction: product(12), Position: product(11:2)
    -- =========================================================================
    s_tri0 <= to_unsigned(1023, 10) - unsigned(s2_prod0(11 downto 2))
              when s2_prod0(12) = '1'
              else unsigned(s2_prod0(11 downto 2));

    s_tri1 <= to_unsigned(1023, 10) - unsigned(s2_prod1(11 downto 2))
              when s2_prod1(12) = '1'
              else unsigned(s2_prod1(11 downto 2));

    s_tri2 <= to_unsigned(1023, 10) - unsigned(s2_prod2(11 downto 2))
              when s2_prod2(12) = '1'
              else unsigned(s2_prod2(11 downto 2));

    -- =========================================================================
    -- Sine LUTs (combinational): sawtooth phase as angle input
    -- =========================================================================
    u_sin0 : entity work.sin_cos_full_lut_10x10
        port map (angle_in => std_logic_vector(s2_prod0(11 downto 2)),
                  sin_out  => s_sin0_raw, cos_out => open);

    u_sin1 : entity work.sin_cos_full_lut_10x10
        port map (angle_in => std_logic_vector(s2_prod1(11 downto 2)),
                  sin_out  => s_sin1_raw, cos_out => open);

    u_sin2 : entity work.sin_cos_full_lut_10x10
        port map (angle_in => std_logic_vector(s2_prod2(11 downto 2)),
                  sin_out  => s_sin2_raw, cos_out => open);

    -- =========================================================================
    -- Stage 2.5 (T+4): Register sin_cos and fold outputs
    --   Breaks the long combinational path through the sin_cos LUT tables
    --   to improve timing on high-utilization builds.
    -- =========================================================================
    p_stage2p5 : process(clk)
    begin
        if rising_edge(clk) then
            s2p5_sin0  <= s_sin0_raw;
            s2p5_sin1  <= s_sin1_raw;
            s2p5_sin2  <= s_sin2_raw;
            s2p5_tri0  <= s_tri0;
            s2p5_tri1  <= s_tri1;
            s2p5_tri2  <= s_tri2;
            s2p5_saw0  <= unsigned(s2_prod0(11 downto 2));
            s2p5_saw1  <= unsigned(s2_prod1(11 downto 2));
            s2p5_saw2  <= unsigned(s2_prod2(11 downto 2));
            s2p5_sign0 <= s2_prod0(12);
            s2p5_sign1 <= s2_prod1(12);
            s2p5_sign2 <= s2_prod2(12);
        end if;
    end process p_stage2p5;

    -- =========================================================================
    -- Stage 3 (T+5): Shape select
    --   00=Triangle, 01=Sine, 10=Sawtooth, 11=Square
    -- =========================================================================
    p_stage3 : process(clk)
        variable v_sin0 : unsigned(9 downto 0);
        variable v_sin1 : unsigned(9 downto 0);
        variable v_sin2 : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_sin0 := unsigned(std_logic_vector(s2p5_sin0 + to_signed(512, 10)));
            v_sin1 := unsigned(std_logic_vector(s2p5_sin1 + to_signed(512, 10)));
            v_sin2 := unsigned(std_logic_vector(s2p5_sin2 + to_signed(512, 10)));

            case r_shape is
                when "00" =>
                    s3_ch0 <= s2p5_tri0;
                    s3_ch1 <= s2p5_tri1;
                    s3_ch2 <= s2p5_tri2;
                when "01" =>
                    s3_ch0 <= v_sin0;
                    s3_ch1 <= v_sin1;
                    s3_ch2 <= v_sin2;
                when "10" =>
                    s3_ch0 <= s2p5_saw0;
                    s3_ch1 <= s2p5_saw1;
                    s3_ch2 <= s2p5_saw2;
                when others =>
                    if s2p5_sign0 = '0' then s3_ch0 <= to_unsigned(1023, 10);
                    else s3_ch0 <= to_unsigned(0, 10); end if;
                    if s2p5_sign1 = '0' then s3_ch1 <= to_unsigned(1023, 10);
                    else s3_ch1 <= to_unsigned(0, 10); end if;
                    if s2p5_sign2 = '0' then s3_ch2 <= to_unsigned(1023, 10);
                    else s3_ch2 <= to_unsigned(0, 10); end if;
            end case;
        end if;
    end process p_stage3;

    -- =========================================================================
    -- Stage 4 (T+5): CSC reverse + brightness + clamp
    -- 00=YUV, 01=RGB->YUV, 10=XYZ->YUV, 11=Y-only (pass original chroma)
    -- =========================================================================
    -- Stage 4a: CSC compute (sum-of-shifted-terms only). Output is signed
    -- 12-bit, no brightness add, no +512 chroma offset, no clamp -- those
    -- happen in stage 4b. This split breaks the deep multi-add chain that
    -- caused the hd_hdmi build to fail HD timing.
    p_stage4a : process(clk)
        variable v0 : unsigned(9 downto 0);
        variable v1 : unsigned(9 downto 0);
        variable v2 : unsigned(9 downto 0);
        variable v_r_s : signed(11 downto 0);
        variable v_g_s : signed(11 downto 0);
        variable v_b_s : signed(11 downto 0);
        variable v_ch0_s : signed(11 downto 0);
        variable v_ch1_s : signed(11 downto 0);
        variable v_ch2_s : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            v0 := s3_ch0;
            v1 := s3_ch1;
            v2 := s3_ch2;

            case r_csc_mode is
                when "01" =>
                    -- RGB -> YUV reverse (no brightness, no +512)
                    v_r_s := signed(resize(v0, 12));
                    v_g_s := signed(resize(v1, 12));
                    v_b_s := signed(resize(v2, 12));

                    s4a_y <= shift_right(v_r_s, 2) +
                             shift_right(v_g_s, 1) + shift_right(v_g_s, 4) +
                             shift_right(v_b_s, 3);
                    s4a_u <= -shift_right(v_r_s, 3) -
                              shift_right(v_g_s, 2) +
                              shift_right(v_b_s, 1);
                    s4a_v <= shift_right(v_r_s, 1) -
                             shift_right(v_g_s, 2) - shift_right(v_g_s, 3) -
                             shift_right(v_b_s, 4);
                    s4a_chroma_offset <= '1';

                when "10" =>
                    -- XYZ -> YUV reverse (approximate inverse)
                    v_ch0_s := signed(resize(v0, 12));
                    v_ch1_s := signed(resize(v1, 12));
                    v_ch2_s := signed(resize(v2, 12));

                    -- Y ~ 3/8*X + 5/8*Y_xyz
                    s4a_y <= shift_right(v_ch0_s, 2) + shift_right(v_ch0_s, 3) +
                             shift_right(v_ch1_s, 1) + shift_right(v_ch1_s, 3);
                    -- U_s ~ -3/16*X - 1/2*Y_xyz + 5/8*Z
                    s4a_u <= -shift_right(v_ch0_s, 3) - shift_right(v_ch0_s, 4) -
                              shift_right(v_ch1_s, 1) +
                              shift_right(v_ch2_s, 1) + shift_right(v_ch2_s, 3);
                    -- V_s ~ 2*X - 3/2*Y_xyz - 3/8*Z
                    s4a_v <= shift_left(v_ch0_s, 1) -
                             v_ch1_s - shift_right(v_ch1_s, 1) -
                             shift_right(v_ch2_s, 2) - shift_right(v_ch2_s, 3);
                    s4a_chroma_offset <= '1';

                when "11" =>
                    -- Y-only: fold luma, preserve original chroma from dry tap
                    s4a_y <= signed(resize(v0, 12));
                    s4a_u <= signed(resize(s_dry8_u, 12));
                    s4a_v <= signed(resize(s_dry8_v, 12));
                    s4a_chroma_offset <= '0';

                when others =>
                    -- YUV mode (00): brightness on Y, pass U/V
                    s4a_y <= signed(resize(v0, 12));
                    s4a_u <= signed(resize(v1, 12));
                    s4a_v <= signed(resize(v2, 12));
                    s4a_chroma_offset <= '0';
            end case;
        end if;
    end process p_stage4a;

    -- Stage 4b: brightness add (Y), conditional +512 chroma offset (U/V), clamp
    -- to the legal range: Y 64..1023 (the knee in p_guard caps white), U/V
    -- 64..960. The fold drives every channel rail to rail; chroma code 0 is
    -- saturated green and sub-black luma is not picture.
    p_stage4b : process(clk)
        variable v_brt : signed(11 downto 0);
        variable v_y   : signed(11 downto 0);
        variable v_u   : signed(11 downto 0);
        variable v_v   : signed(11 downto 0);
        variable v_off : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            v_brt := signed(resize(r_brightness, 12)) - to_signed(512, 12);
            if s4a_chroma_offset = '1' then
                v_off := to_signed(512, 12);
            else
                v_off := to_signed(0, 12);
            end if;

            v_y := s4a_y + v_brt;
            v_u := s4a_u + v_off;
            v_v := s4a_v + v_off;

            if v_y < 64 then s4_y <= to_unsigned(64, 10);
            elsif v_y > 1023 then s4_y <= to_unsigned(1023, 10);
            else s4_y <= unsigned(v_y(9 downto 0)); end if;

            if v_u < 64 then s4_u <= to_unsigned(64, 10);
            elsif v_u > 960 then s4_u <= to_unsigned(960, 10);
            else s4_u <= unsigned(v_u(9 downto 0)); end if;

            if v_v < 64 then s4_v <= to_unsigned(64, 10);
            elsif v_v > 960 then s4_v <= to_unsigned(960, 10);
            else s4_v <= unsigned(v_v(9 downto 0)); end if;
        end if;
    end process p_stage4b;

    -- =========================================================================
    -- Output guard on the wet path (the HD HDMI thick-bar fix). The fold makes
    -- sharp near-full-swing pixel steps in motion, which trigger the bars, and
    -- for Wrinkle it is the CHROMA swing that matters (HW bisect 2026-10-20:
    -- [1 2 1] on U/V barred, wider luma filtering or dimming changed nothing):
    --   U/V [1 2 2 2 2 2 2 2 1]/16      (w0 .. w8)
    --   Y   [1 2 1]/4                   (w3 .. w5)
    --   Y   knee: y >= 512 -> 512 + (y - 512)/2, white 1023 -> 767
    -- Dry video is left untouched (Mix 0% is still the source).
    -- =========================================================================
    p_guard : process(clk)
        variable v_y : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- window: w(0) = s4 (newest), w(k) = k pixels older; centre w(4) T+14
            w_y(1) <= s4_y;  w_u(1) <= s4_u;  w_v(1) <= s4_v;
            for k in 2 to 8 loop
                w_y(k) <= w_y(k - 1);  w_u(k) <= w_u(k - 1);  w_v(k) <= w_v(k - 1);
            end loop;

            -- A (T+15)
            ya_121 <= resize(w_y(3), 12) + resize(w_y(5), 12) + shift_left(resize(w_y(4), 12), 1);
            ua_12 <= resize(w_u(1), 11) + resize(w_u(2), 11);
            ua_35 <= resize(w_u(3), 11) + resize(w_u(5), 11);
            ua_67 <= resize(w_u(6), 11) + resize(w_u(7), 11);
            ua_08 <= resize(s4_u, 11) + resize(w_u(8), 11);
            ua_4  <= w_u(4);
            va_12 <= resize(w_v(1), 11) + resize(w_v(2), 11);
            va_35 <= resize(w_v(3), 11) + resize(w_v(5), 11);
            va_67 <= resize(w_v(6), 11) + resize(w_v(7), 11);
            va_08 <= resize(s4_v, 11) + resize(w_v(8), 11);
            va_4  <= w_v(4);

            -- B (T+16)
            yb_121 <= ya_121;
            ub_r1 <= resize(ua_12, 12) + resize(ua_35, 12);
            ub_r2 <= resize(ua_67, 12) + resize(ua_4, 12);
            ub_08 <= ua_08;
            vb_r1 <= resize(va_12, 12) + resize(va_35, 12);
            vb_r2 <= resize(va_67, 12) + resize(va_4, 12);
            vb_08 <= va_08;

            -- C (T+17): the x16 sums
            yc_121 <= yb_121;
            uc_s9 <= shift_left(resize(ub_r1, 14) + resize(ub_r2, 14), 1) + resize(ub_08, 14);
            vc_s9 <= shift_left(resize(vb_r1, 14) + resize(vb_r2, 14), 1) + resize(vb_08, 14);

            -- T+18: normalise, luma knee (pure wiring)
            v_y := yc_121(11 downto 2);
            if v_y(9) = '1' then
                c2_y(9 downto 8) <= "10";
                c2_y(7 downto 0) <= v_y(8 downto 1);
            else
                c2_y <= v_y;
            end if;
            c2_u <= uc_s9(13 downto 4);
            c2_v <= vc_s9(13 downto 4);
        end if;
    end process p_guard;

    -- =========================================================================
    -- Interpolators: fader-controlled dry/wet mix, T+18 in -> T+22 out
    -- (always enabled; the output register gates blanking)
    -- =========================================================================
    u_interp_y : entity work.interpolator_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (clk => clk, enable => '1',
                  a => s_dry_y, b => c2_y, t => s_mix,
                  result => s_mix_y, valid => open);

    u_interp_u : entity work.interpolator_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (clk => clk, enable => '1',
                  a => s_dry_u, b => c2_u, t => s_mix,
                  result => s_mix_u, valid => open);

    u_interp_v : entity work.interpolator_u
        generic map (G_WIDTH => C_VIDEO_DATA_WIDTH, G_FRAC_BITS => C_VIDEO_DATA_WIDTH,
                     G_OUTPUT_MIN => 0, G_OUTPUT_MAX => 1023)
        port map (clk => clk, enable => '1',
                  a => s_dry_v, b => c2_v, t => s_mix,
                  result => s_mix_v, valid => open);

    -- =========================================================================
    -- Delays: syncs + bypass video to T+22, dry video to T+18 (mix) and
    -- T+8 (Y-only chroma). One raw-video chain; the dry taps share it.
    -- =========================================================================
    p_delay : process(clk)
        type t_sync_delay is array (0 to C_TAP_CLKS - 1) of std_logic;
        variable v_hsync_n : t_sync_delay := (others => '1');
        variable v_vsync_n : t_sync_delay := (others => '1');
        variable v_field_n : t_sync_delay := (others => '1');
        variable v_avid    : t_sync_delay := (others => '0');
        type t_data_delay is array (0 to C_TAP_CLKS - 1)
            of std_logic_vector(9 downto 0);
        variable v_y : t_data_delay := (others => (others => '0'));
        variable v_u : t_data_delay := (others => (others => '0'));
        variable v_v : t_data_delay := (others => (others => '0'));
    begin
        if rising_edge(clk) then
            v_hsync_n := data_in.hsync_n & v_hsync_n(0 to C_TAP_CLKS - 2);
            v_vsync_n := data_in.vsync_n & v_vsync_n(0 to C_TAP_CLKS - 2);
            v_field_n := data_in.field_n & v_field_n(0 to C_TAP_CLKS - 2);
            v_avid    := data_in.avid    & v_avid   (0 to C_TAP_CLKS - 2);
            v_y := data_in.y & v_y(0 to C_TAP_CLKS - 2);
            v_u := data_in.u & v_u(0 to C_TAP_CLKS - 2);
            v_v := data_in.v & v_v(0 to C_TAP_CLKS - 2);
            s_tap_hsync_n <= v_hsync_n(C_TAP_CLKS - 1);
            s_tap_vsync_n <= v_vsync_n(C_TAP_CLKS - 1);
            s_tap_field_n <= v_field_n(C_TAP_CLKS - 1);
            s_tap_avid    <= v_avid   (C_TAP_CLKS - 1);
            s_bypass_y <= v_y(C_TAP_CLKS - 1);
            s_bypass_u <= v_u(C_TAP_CLKS - 1);
            s_bypass_v <= v_v(C_TAP_CLKS - 1);
            s_dry_y  <= unsigned(v_y(C_DRY_CLKS - 1));
            s_dry_u  <= unsigned(v_u(C_DRY_CLKS - 1));
            s_dry_v  <= unsigned(v_v(C_DRY_CLKS - 1));
            s_dry8_u <= unsigned(v_u(C_YONLY_UV_CLKS - 1));
            s_dry8_v <= unsigned(v_v(C_YONLY_UV_CLKS - 1));
        end if;
    end process p_delay;

    -- =========================================================================
    -- Output register (T+23): bypass passes the raw input untouched;
    -- otherwise the mix, gated to neutral (Y 64, U/V 512) outside avid.
    -- Then a pad register -> T+24.
    -- =========================================================================
    p_out : process(clk)
    begin
        if rising_edge(clk) then
            if s_bypass = '1' then
                s_out.y <= s_bypass_y;
                s_out.u <= s_bypass_u;
                s_out.v <= s_bypass_v;
            elsif s_tap_avid = '0' then
                s_out.y <= std_logic_vector(to_unsigned(64, 10));
                s_out.u <= std_logic_vector(to_unsigned(512, 10));
                s_out.v <= std_logic_vector(to_unsigned(512, 10));
            else
                s_out.y <= std_logic_vector(s_mix_y);
                s_out.u <= std_logic_vector(s_mix_u);
                s_out.v <= std_logic_vector(s_mix_v);
            end if;
            s_out.avid    <= s_tap_avid;
            s_out.hsync_n <= s_tap_hsync_n;
            s_out.vsync_n <= s_tap_vsync_n;
            s_out.field_n <= s_tap_field_n;
            s_pad_0 <= s_out;
        end if;
    end process p_out;

    data_out.y       <= s_pad_0.y;
    data_out.u       <= s_pad_0.u;
    data_out.v       <= s_pad_0.v;
    data_out.avid    <= s_pad_0.avid;
    data_out.hsync_n <= s_pad_0.hsync_n;
    data_out.vsync_n <= s_pad_0.vsync_n;
    data_out.field_n <= s_pad_0.field_n;

end architecture wrinkle;
