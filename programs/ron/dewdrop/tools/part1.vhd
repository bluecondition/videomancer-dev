-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- dewdrop.vhd  (v0.1)
--
-- DEWDROP: glossy lens-drops drifting over a waving rainbow band field.
--
-- Eight circles (four big, four at 0.6x) orbit the screen on Lissajous
-- paths.  Where a circle covers the screen the band field is seen
-- MAGNIFIED about its centre (Refract), so the rainbow bands bulge and
-- wrap inside each drop; a dispersion term rotates the hue toward the
-- rim.  The union of the drops wears a dark rim lit from the top-left
-- (white highlight, black shadow) with a thin full-spectrum fringe on its
-- outer edge.  Outside the drops the same field shows as a pastel (or
-- dark, or a flat mint) ground.  P12 Swell sets the radius: a fizz of
-- small lenses up to screen-filling discs whose arcs cross the picture.
--
-- GEOMETRY: per circle j the signed field m_j = (x-cx)^2 + (y-cy)^2 - R_j^2
-- runs along the line by second differences (m += incx, incx += 2; two
-- adders per circle, nothing else per pixel).  The line-start value
-- M0_j + dy^2 is computed for the NEXT line during the current active
-- line by a small FSM (one serial 16x16 square per circle, dy in Q4
-- horizontal-pixel units so circles are round in every raster) into a
-- time-shared circle-table EBR, and loaded into the eight accumulators in
-- hblank.  The pixel owner is argmin m_j (3-stage tree carrying m, dx, dy
-- and the size class); m < 0 = inside, 0 <= m < D = rim.
--
-- FIELD: idx = ((a + A*sin(b*fw + ph)) * bs) >> 8 + disp + scroll, with
-- (a,b) = (x',y') displaced by the lens (or swapped for horizontal
-- stripes); a 256-entry sine EBR (time-shared with the vblank sequencer
-- for the orbit phases) and three 256x10 palette EBRs (yellow, red,
-- magenta, dark, blue, cyan, green, black, wrap).  Palette authored in the
-- HW-verified convention (u register = BT.601 Cr) at 0.8125 intensity
-- (white Y ~775).  Output gets the [1 2 1] Y/U/V conditioning before the
-- blanking gate (HD HDMI thick-bar rule).  Late flags ride two EBR delay
-- lines instead of FF chains.
--
-- MOTION: cx = W/2 + ax*sin(th), cy = H/2 + ay*sin(ph), per-circle
-- amplitudes and phase rates; Drift scales the rates (never fully still).
-- Flow turns the wave phase and scrolls the bands (square law, centre
-- still).  Every per-frame product runs on one serial multiplier in the
-- vblank sequencer.
--
-- CONTROLS
--   K1  Bands     band width (one palette cycle over 240 .. 3840 px)
--   K2  Wave      sideways bend of the bands: flat -> half the screen
--   K3  Refract   magnification inside the drops (+ rim dispersion)
--   K4  Rim       rim thickness
--   K5  Drift     orbit speed (never fully still)
--   K6  Flow      wave phase + band scroll, bipolar, centre = still
--   S7  Ground    Pastel / Dark
--   S8  Drops     Pairs (small drop hangs under each big one) / Solo
--   S9  Bands     Vertical / Horizontal
--   S10 Backdrop  Field / Mint
--   S11 Colour    Rainbow / Chrome (grey bands, spectral fringe stays)
--   P12 SWELL     drop radius: W/32 .. W/2
--
-- Generated avid (cubist recipe), latency 28 clocks in all modes, 7 EBR.
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture dewdrop of program_top is
    constant C_PROCESSING_DELAY_CLKS : natural := 28;
    constant C_LAT : integer := 28;
    constant C_N   : integer := 8;           -- circles: 0..3 big, 4..7 small

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);

    type t_rom10 is array(0 to 255) of unsigned(9 downto 0);
