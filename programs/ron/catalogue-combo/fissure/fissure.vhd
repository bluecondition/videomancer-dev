-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Fissure v0.1.0: catalogue combo (2026-10-20) built from two catalogue
-- entries no combo had used -- "kintsugi-style crack overlays" and
-- "mycelium / growth patterns".  The picture's own edges start cracks; each
-- crack grows down the screen as an angular fracture that kinks, forks and
-- stops where it runs into another, steered toward the picture's dark (or
-- light) lines, and the network is filled with bevelled gold that the
-- picture lights.  With the ground taken to black it becomes a gold drawing
-- of the picture made only of veins.
--
--   K1 Spawn     how readily edges start cracks (threshold + density)
--   K2 Branch    fork rate: single fissures .. bushy deltas
--   K3 Wander    kink rate and slope: straight fractures .. lightning
--   K4 Seam      gold width, hairline .. bold
--   K5 Ground    the picture between the veins, full .. black
--   K6 Metal     Gold / Silver / Copper / Prism (a hue per crack)
--   S7  Seek     kinks turn toward the Dark / Light side of the picture
--   S8  Seeds    Edges (picture edges) / Scatter (a rain over lit areas)
--   S9  Mask     Free / Lit: cracks stop where the picture goes dark
--   S10 Shadow   a dark inset groove round every seam
--   S11 Glint    a highlight sweeping across the gold
--   P12 Reach    how far cracks run: splinters at the edges .. the whole
--                picture shattered (immortal at the top).  Glided.
--
-- Engine.  Each line holds a SORTED LIST of crack tips (512 x 64 RAM, two
-- 256-entry banks by line parity): {p 11, pf 2, v 7 (Q4.2 px/line), mn 11,
-- mx 11 (the span the tip swept since the line above), hue 4, rs 16 (the
-- crack's own xorshift16 state), end}.  Line n draws every tip as its span
-- [mn, mx] widened by the seam.  A walk pointer steps through the list as
-- the raster passes: cur = the first entry whose span ends at or after x.
-- The entries' starts and ends both increase strictly and their ends are
-- >= C_G - 1 apart, so one pointer step per 7+ clocks (a 3-clock fetch)
-- draws the list exactly.  The pointer is armed from the line start until
-- the next list is primed in blanking.  When cur retires it becomes an
-- event: the crack steps its state three times (die; kink with probability,
-- magnitude r^2 * vmax, direction from the picture gradient at the tip;
-- fork), and the result is written into the other bank.  Writes are kept sorted by dropping any candidate
-- that would land within C_G of the last one written: that is how cracks
-- merge into each other.  Seeds (picture edges, content-hashed so a crack
-- follows its seed when the picture moves) fill the gaps between spans.
-- All randomness comes from the per-crack state, so a crack's shape is a
-- function of its seed, not of the screen position.
-- A per-column RAM {countdown 6, luma 6, hue 4} gives shallow strokes their
-- thickness (a core pixel restarts a T-line countdown below it), the
-- Shadow groove, and the luma of the line above for the vertical edge term.
-- The first line of every field starts empty: the network is regrown from
-- the picture every frame.
--
-- Hardware contracts: vblank zone (no avid for 1024 clocks) for every
-- per-field event; the picture is carried in a 256-word EBR ring, not an FF
-- pipe; generated colours swap U/V on lines whose hsync->avid parity is off
-- the voted usual one (the source avid passes through, so the picture
-- double-swaps and stays right); output conditioning (luma [1 2 1], chroma
-- [1 2 2 2 2 2 2 2 1]/16, luma x13/16); blanking gate; no_rw_check on every
-- RAM (all collision-free).  Latency 32 clocks, all modes.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture fissure of program_top is
    -- data_in -> data_out latency (multiple of 4); matches processing_delay_clks.
    constant C_PROCESSING_DELAY_CLKS : natural := 32;
    constant C_LAT   : integer := 32;
    constant C_G     : integer := 8;    -- min spacing of crack tips (px); their span ends >= C_G - 1 apart
    constant C_EDGE  : integer := 16;   -- tips die this close to either end of the line
    constant C_SEDGE : integer := 24;   -- no seeds this close
    constant C_DEAD  : integer := 40;   -- steering deadband (box-sum units = 4 x luma8)

    -- K1: seed density of 256 (8 .. 64, exponential)
    type t_dens is array (0 to 63) of integer range 0 to 255;
    constant C_DENS : t_dens := (
        8, 8, 9, 9, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 13, 13, 14, 14, 14, 15, 15, 16, 17, 17, 18, 18, 19, 20,
        20, 21, 22, 22, 23, 24, 25, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 37, 38, 39, 40, 42, 43, 45, 46,
        48, 49, 51, 53, 54, 56, 58, 60, 62, 64);

    function f_c(y, u, v : integer) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(y, 10)) & std_logic_vector(to_unsigned(u, 10)) &
               std_logic_vector(to_unsigned(v, 10)) & "00";
    end function;
    function f_h(u, v : integer) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(u, 10)) & std_logic_vector(to_unsigned(v, 10));
    end function;
    type t_metal is array (0 to 255) of std_logic_vector(31 downto 0);
    type t_prism is array (0 to 15) of std_logic_vector(19 downto 0);
    -- {Y, U, V}, U/V SWAPPED for the hardware (U carries Cr); level 0 = dark edge
    -- generated by tools/gen_tables.py: gold 0-63, silver 64-127, copper 128-191, prism luma 192-255
    constant C_METAL : t_metal := (
        f_c(230, 540, 447), f_c(238, 541, 443), f_c(247, 543, 440), f_c(256, 544, 437),
        f_c(264, 546, 433), f_c(273, 547, 430), f_c(282, 548, 426), f_c(290, 550, 423),
        f_c(299, 551, 420), f_c(308, 553, 416), f_c(316, 554, 413), f_c(325, 556, 409),
        f_c(334, 557, 406), f_c(342, 559, 403), f_c(351, 560, 399), f_c(360, 562, 396),
        f_c(368, 563, 393), f_c(377, 564, 389), f_c(385, 566, 386), f_c(394, 567, 382),
        f_c(403, 569, 379), f_c(411, 570, 376), f_c(420, 572, 372), f_c(429, 573, 369),
        f_c(437, 575, 365), f_c(446, 576, 362), f_c(455, 577, 359), f_c(463, 579, 355),
        f_c(472, 580, 352), f_c(481, 582, 348), f_c(489, 583, 345), f_c(498, 585, 342),
        f_c(507, 586, 338), f_c(515, 588, 335), f_c(524, 589, 331), f_c(532, 591, 328),
        f_c(541, 592, 325), f_c(550, 593, 321), f_c(558, 595, 318), f_c(567, 596, 314),
        f_c(576, 598, 311), f_c(584, 599, 308), f_c(593, 601, 304), f_c(602, 602, 301),
        f_c(610, 604, 297), f_c(619, 605, 294), f_c(634, 605, 295), f_c(652, 603, 299),
        f_c(669, 601, 303), f_c(686, 600, 307), f_c(702, 598, 311), f_c(718, 596, 316),
        f_c(733, 594, 320), f_c(748, 592, 325), f_c(762, 589, 330), f_c(776, 587, 336),
        f_c(789, 585, 341), f_c(802, 582, 347), f_c(814, 580, 353), f_c(825, 577, 359),
        f_c(836, 574, 366), f_c(846, 572, 372), f_c(856, 569, 379), f_c(860, 566, 386),
        f_c(253, 509, 517), f_c(263, 509, 518), f_c(273, 508, 518), f_c(283, 508, 518),
        f_c(292, 508, 518), f_c(302, 508, 519), f_c(312, 508, 519), f_c(322, 508, 519),
        f_c(332, 507, 519), f_c(342, 507, 520), f_c(352, 507, 520), f_c(361, 507, 520),
        f_c(371, 507, 521), f_c(381, 507, 521), f_c(391, 507, 521), f_c(401, 506, 521),
        f_c(411, 506, 522), f_c(421, 506, 522), f_c(431, 506, 522), f_c(440, 506, 522),
        f_c(450, 506, 523), f_c(460, 505, 523), f_c(470, 505, 523), f_c(480, 505, 524),
        f_c(490, 505, 524), f_c(500, 505, 524), f_c(509, 505, 524), f_c(519, 504, 525),
        f_c(529, 504, 525), f_c(539, 504, 525), f_c(549, 504, 525), f_c(559, 504, 526),
        f_c(569, 504, 526), f_c(578, 503, 526), f_c(588, 503, 527), f_c(598, 503, 527),
        f_c(608, 503, 527), f_c(618, 503, 527), f_c(628, 503, 528), f_c(638, 502, 528),
        f_c(647, 502, 528), f_c(657, 502, 528), f_c(667, 502, 529), f_c(677, 502, 529),
        f_c(687, 502, 529), f_c(697, 501, 530), f_c(711, 501, 530), f_c(728, 502, 529),
        f_c(744, 502, 529), f_c(759, 502, 529), f_c(774, 502, 528), f_c(788, 502, 528),
        f_c(802, 503, 527), f_c(815, 503, 527), f_c(827, 503, 527), f_c(839, 503, 526),
        f_c(850, 504, 526), f_c(860, 504, 525), f_c(860, 504, 525), f_c(860, 505, 524),
        f_c(860, 505, 524), f_c(860, 505, 523), f_c(860, 506, 523), f_c(860, 506, 522),
        f_c(183, 554, 475), f_c(189, 556, 474), f_c(196, 559, 472), f_c(202, 561, 470),
        f_c(208, 563, 468), f_c(214, 565, 466), f_c(220, 567, 464), f_c(227, 570, 462),
        f_c(233, 572, 460), f_c(239, 574, 458), f_c(245, 576, 456), f_c(251, 578, 454),
        f_c(258, 581, 453), f_c(264, 583, 451), f_c(270, 585, 449), f_c(276, 587, 447),
        f_c(283, 589, 445), f_c(289, 592, 443), f_c(295, 594, 441), f_c(301, 596, 439),
        f_c(307, 598, 437), f_c(314, 600, 435), f_c(320, 603, 434), f_c(326, 605, 432),
        f_c(332, 607, 430), f_c(338, 609, 428), f_c(345, 611, 426), f_c(351, 614, 424),
        f_c(357, 616, 422), f_c(363, 618, 420), f_c(370, 620, 418), f_c(376, 622, 416),
        f_c(382, 625, 414), f_c(388, 627, 413), f_c(394, 629, 411), f_c(401, 631, 409),
        f_c(407, 633, 407), f_c(413, 636, 405), f_c(419, 638, 403), f_c(425, 640, 401),
        f_c(432, 642, 399), f_c(438, 644, 397), f_c(444, 647, 395), f_c(450, 649, 394),
        f_c(457, 651, 392), f_c(463, 653, 390), f_c(478, 653, 390), f_c(499, 650, 392),
        f_c(519, 648, 395), f_c(539, 645, 397), f_c(558, 642, 399), f_c(577, 639, 402),
        f_c(596, 636, 404), f_c(614, 633, 407), f_c(632, 630, 410), f_c(650, 626, 413),
        f_c(667, 623, 416), f_c(683, 619, 420), f_c(700, 615, 423), f_c(716, 611, 426),
        f_c(731, 607, 430), f_c(746, 602, 434), f_c(761, 598, 438), f_c(776, 593, 442),
        f_c(283, 512, 512), f_c(293, 512, 512), f_c(302, 512, 512), f_c(312, 512, 512),
        f_c(322, 512, 512), f_c(332, 512, 512), f_c(341, 512, 512), f_c(351, 512, 512),
        f_c(361, 512, 512), f_c(371, 512, 512), f_c(380, 512, 512), f_c(390, 512, 512),
        f_c(400, 512, 512), f_c(410, 512, 512), f_c(419, 512, 512), f_c(429, 512, 512),
        f_c(439, 512, 512), f_c(448, 512, 512), f_c(458, 512, 512), f_c(468, 512, 512),
        f_c(478, 512, 512), f_c(487, 512, 512), f_c(497, 512, 512), f_c(507, 512, 512),
        f_c(517, 512, 512), f_c(526, 512, 512), f_c(536, 512, 512), f_c(546, 512, 512),
        f_c(556, 512, 512), f_c(565, 512, 512), f_c(575, 512, 512), f_c(585, 512, 512),
        f_c(594, 512, 512), f_c(604, 512, 512), f_c(614, 512, 512), f_c(624, 512, 512),
        f_c(633, 512, 512), f_c(643, 512, 512), f_c(653, 512, 512), f_c(663, 512, 512),
        f_c(672, 512, 512), f_c(682, 512, 512), f_c(692, 512, 512), f_c(702, 512, 512),
        f_c(711, 512, 512), f_c(721, 512, 512), f_c(731, 512, 512), f_c(740, 512, 512),
        f_c(750, 512, 512), f_c(760, 512, 512), f_c(770, 512, 512), f_c(779, 512, 512),
        f_c(789, 512, 512), f_c(799, 512, 512), f_c(809, 512, 512), f_c(818, 512, 512),
        f_c(828, 512, 512), f_c(838, 512, 512), f_c(848, 512, 512), f_c(857, 512, 512),
        f_c(860, 512, 512), f_c(860, 512, 512), f_c(860, 512, 512), f_c(860, 512, 512));
    constant C_PRISM : t_prism := (
        f_h(841, 401), f_h(767, 289), f_h(654, 210), f_h(519, 178), f_h(384, 196), f_h(268, 263), f_h(189, 367), f_h(159, 494),
        f_h(183, 623), f_h(257, 735), f_h(370, 814), f_h(505, 846), f_h(640, 828), f_h(756, 761), f_h(835, 657), f_h(865, 530));

    -- x step per pixel (Q.1 virtual units: 2 / 3 / 4 = 1080 / 720 / SD)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- y step in Q.8 virtual units per FIELD line
    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 12);  -- 480i
            when 4          => return to_unsigned(576, 12);   -- 480p
            when 8          => return to_unsigned(960, 12);   -- 576i
            when 12         => return to_unsigned(480, 12);   -- 576p
            when 5 | 6 | 14 => return to_unsigned(384, 12);   -- 720p
            when 1 | 2 | 10 => return to_unsigned(512, 12);   -- 1080i
            when others     => return to_unsigned(256, 12);   -- 1080p
        end case;
    end function;
    -- v x (virtual lines per field line): per-line probabilities and accelerations
    function f_sy(v : unsigned(11 downto 0); t : std_logic_vector(3 downto 0)) return unsigned is
        variable a : unsigned(14 downto 0);
    begin
        a := resize(v, 15);
        case to_integer(unsigned(t)) is
            when 0          => return shift_left(a, 2) + shift_right(a, 1);   -- x4.5
            when 4          => return shift_left(a, 1) + shift_right(a, 2);   -- x2.25
            when 8          => return shift_left(a, 2) - shift_right(a, 2);   -- x3.75
            when 12         => return shift_left(a, 1) - shift_right(a, 3);   -- x1.875
            when 5 | 6 | 14 => return a + shift_right(a, 1);                  -- x1.5
            when 1 | 2 | 10 => return shift_left(a, 1);                       -- x2
            when others     => return a;
        end case;
    end function;
    -- n virtual lines -> field lines (n x 256 / ystep), at least 1
    function f_ln(n : unsigned(5 downto 0); t : std_logic_vector(3 downto 0)) return unsigned is
        variable r : unsigned(5 downto 0);
    begin
        case to_integer(unsigned(t)) is
            when 0          => r := shift_right(n, 2) - shift_right(n, 5);                       -- x0.22
            when 4          => r := shift_right(n, 1) - shift_right(n, 4);                       -- x0.44
            when 8          => r := shift_right(n, 2) + shift_right(n, 5);                       -- x0.28
            when 12         => r := shift_right(n, 1) + shift_right(n, 5);                       -- x0.53
            when 5 | 6 | 14 => r := shift_right(n, 1) + shift_right(n, 3) + shift_right(n, 5);  -- x0.66
            when 1 | 2 | 10 => r := shift_right(n, 1);
            when others     => r := n;
        end case;
        if r = 0 then r := to_unsigned(1, 6); end if;
        return r;
    end function;
    -- n virtual px -> real px (n x 2 / xs), at least 1
    function f_sx(n : unsigned(6 downto 0); xs : unsigned(2 downto 0)) return unsigned is
        variable r : unsigned(6 downto 0);
    begin
        if xs = 4 then
            r := shift_right(n, 1);
        elsif xs = 3 then
            r := shift_right(n, 1) + shift_right(n, 3) + shift_right(n, 5);
        else
            r := n;
        end if;
        if r = 0 then r := to_unsigned(1, 7); end if;
        return r;
    end function;
    -- xorshift16 (7, 9, 8), full period: one step of a crack's random state
    function f_xs16(s : std_logic_vector(15 downto 0)) return std_logic_vector is
        variable x : std_logic_vector(15 downto 0);
    begin
        x := s;
        x := x xor (x(8 downto 0) & "0000000");
        x := x xor ("000000000" & x(15 downto 9));
        x := x xor (x(7 downto 0) & "00000000");
        return x;
    end function;

    -- list word: p(63:53) pf(52:51) v(50:44) mn(43:33) mx(32:22) hue(21:18) rs(17:2) spare(1) end(0)
    -- (mn, mx = the span this tip swept since the line above)
    constant C_SENT  : std_logic_vector(63 downto 0) := "11111111111" & "00" & "0000000" & "11111111111" &
                                                         "11111111111" & "0000" & x"0001" & "0" & "1";
    constant C_BLACK : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    -- RAMs
    type t_list is array (0 to 511) of std_logic_vector(63 downto 0);
    type t_col  is array (0 to 2047) of std_logic_vector(15 downto 0);   -- {cnt 6, luma 6, hue 4}
    type t_ring is array (0 to 255) of std_logic_vector(31 downto 0);    -- {y, u, v, 00}
    signal lram   : t_list := (others => C_SENT);
    signal colram : t_col  := (others => (others => '0'));
    signal ring   : t_ring := (others => C_BLACK);
    attribute no_rw_check : boolean;
    attribute no_rw_check of lram, colram, ring : signal is true;


    signal s_l_ra, s_l_wa : unsigned(8 downto 0) := (others => '0');
    signal s_l_q  : std_logic_vector(63 downto 0) := C_SENT;
    signal s_l_wd : std_logic_vector(63 downto 0) := C_SENT;
    signal s_l_we : std_logic := '0';
    signal s_col_ra, s_col_wa : unsigned(10 downto 0) := (others => '0');
    signal s_col_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal s_col_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal s_col_we : std_logic := '0';
    signal s_vw, s_rw_a, s_rr_a : unsigned(7 downto 0) := (others => '0');
    signal s_rw_d : std_logic_vector(31 downto 0) := C_BLACK;
    signal s_rr_q : std_logic_vector(31 downto 0) := C_BLACK;
    signal s_rom_a : unsigned(7 downto 0) := (others => '0');
    signal s_rom_q : std_logic_vector(31 downto 0) := C_BLACK;

    -- controls
    signal s_tid : std_logic_vector(3 downto 0) := (others => '0');
    signal s_xs  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_j1, s_j2, s_j3 : unsigned(5 downto 0) := (others => '0');
    signal s_j4, s_j5 : unsigned(3 downto 0) := (others => '0');
    signal s_metk : unsigned(1 downto 0) := "00";
    signal s_live6 : std_logic := '0';
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');      -- glided P12, Q.2
    -- field-latched
    signal s_l1, s_l2, s_l3 : unsigned(5 downto 0) := (others => '0');
    signal s_l4, s_l5 : unsigned(3 downto 0) := (others => '0');
    signal s_metal : unsigned(1 downto 0) := "00";
    signal s_seek, s_scat, s_mask, s_shad, s_glint : std_logic := '0';
    signal s_tg : unsigned(9 downto 0) := (others => '0');
    -- derived (settle within a few clocks of the latch)
    signal s_thr, s_dens : unsigned(7 downto 0) := (others => '0');
    signal s_dens4 : unsigned(9 downto 0) := (others => '0');
    signal s_clv : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal s_j2sq, s_usq : unsigned(11 downto 0) := (others => '0');
    signal s_u6 : unsigned(5 downto 0) := (others => '0');
    signal s_pbv, s_pkv, s_pdv : unsigned(11 downto 0) := (others => '0');
    signal s_vmax : unsigned(5 downto 0) := to_unsigned(4, 6);
    signal s_vmaxs : signed(7 downto 0) := to_signed(4, 8);
    signal s_wv, s_hv : unsigned(5 downto 0) := to_unsigned(1, 6);
    signal s_f : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal s_pb, s_pk : unsigned(9 downto 0) := (others => '0');
    signal s_pd : unsigned(11 downto 0) := (others => '0');
    signal s_wr, s_hr, s_wh : signed(11 downto 0) := to_signed(1, 12);
    signal s_clr : signed(11 downto 0) := to_signed(64, 12);
    signal s_T, s_Th, s_Tt : unsigned(5 downto 0) := to_unsigned(2, 6);
    signal s_ks : unsigned(1 downto 0) := "00";
    signal s_hi, s_hi8, s_shi : unsigned(10 downto 0) := (others => '0');

    -- line
    signal s_hsq, s_ph, s_upar, s_swl : std_logic := '0';
    signal s_upc : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal s_w, s_wmax : unsigned(10 downto 0) := (others => '0');
    signal s_vl : unsigned(19 downto 0) := (others => '0');      -- virtual line, Q.8

    -- pixel features
    type t_u8a  is array (natural range <>) of unsigned(7 downto 0);
    type t_u10a is array (natural range <>) of unsigned(9 downto 0);
    type t_u11a is array (natural range <>) of unsigned(10 downto 0);
    signal s_y1 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u1, s_v1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_rx : t_u11a(1 to 16) := (others => (others => '0'));
    signal s_y8 : t_u8a(2 to 17) := (others => (others => '0'));
    signal s_box2, s_boxb : unsigned(9 downto 0) := (others => '0');
    signal s_gx3, s_gxd, s_gxa : signed(10 downto 0) := (others => '0');
    signal s_agx, s_agy : unsigned(7 downto 0) := (others => '0');
    signal s_cq10 : std_logic_vector(15 downto 0) := (others => '0');
    signal s_ca11, s_ca12, s_ca13 : std_logic_vector(9 downto 0) := (others => '0');   -- {cnt 6, hue 4}
    signal s_hk, s_h1, s_h2, s_h3, s_h4 : unsigned(23 downto 0) := (others => '0');
    signal s_sdc, s_gate, s_srng, s_sgap : std_logic := '0';   -- seed sub-tests, registered for T(x)

    -- walk: cur / next / prev entries
    signal c_p, c_mn, c_mx : unsigned(10 downto 0) := (others => '1');
    signal c_pf : std_logic_vector(1 downto 0) := "00";
    signal c_v : signed(6 downto 0) := (others => '0');
    signal c_rs : std_logic_vector(15 downto 0) := x"0001";
    signal c_hue : std_logic_vector(3 downto 0) := (others => '0');
    signal c_end, c_new : std_logic := '1';
    signal c_sl, c_sr, c_slh, c_srh, c_mnc, c_mxc : signed(11 downto 0) := (others => '0');
    signal c_gs : signed(10 downto 0) := (others => '0');
    signal c_lum : unsigned(7 downto 0) := (others => '0');
    signal n_p, n_mn, n_mx : unsigned(10 downto 0) := (others => '1');
    signal n_pf : std_logic_vector(1 downto 0) := "00";
    signal n_v : signed(6 downto 0) := (others => '0');
    signal n_rs : std_logic_vector(15 downto 0) := x"0001";
    signal n_hue : std_logic_vector(3 downto 0) := (others => '0');
    signal n_end : std_logic := '1';
    signal p_ok : std_logic := '0';
    signal p_sr, p_srh, p_mxc : signed(11 downto 0) := (others => '0');
    signal p_hue : std_logic_vector(3 downto 0) := (others => '0');
    signal s_advn : std_logic := '0';
    -- fetch pipe (tag: bit 1 valid, bit 0 target = cur)
    signal s_lf : std_logic_vector(63 downto 0) := C_SENT;
    signal s_tgi, s_tq, s_tf : std_logic_vector(1 downto 0) := "00";
    signal s_ridx : unsigned(7 downto 0) := (others => '0');
    signal s_rb, s_sent, s_lres, s_first, s_arm : std_logic := '0';
    signal s_hb : unsigned(5 downto 0) := (others => '1');

    -- events (E0 from the walk, E1..E7 in p_event)
    signal e0_v, e0_sd : std_logic := '0';
    signal e0_p : unsigned(10 downto 0) := (others => '0');
    signal e0_pf : std_logic_vector(1 downto 0) := "00";
    signal e0_vv : signed(6 downto 0) := (others => '0');
    signal e0_rs : std_logic_vector(15 downto 0) := x"0001";
    signal e0_hue : std_logic_vector(3 downto 0) := (others => '0');
    signal e0_gs : signed(10 downto 0) := (others => '0');
    signal e0_lum : unsigned(7 downto 0) := (others => '0');
    type t_ev is record
        v, sd, die, kill, kink, neg, br, sten, sneg : std_logic;
        p    : unsigned(10 downto 0);
        pf   : std_logic_vector(1 downto 0);
        vv   : signed(6 downto 0);
        s    : std_logic_vector(15 downto 0);
        hue  : std_logic_vector(3 downto 0);
        hueb : std_logic_vector(3 downto 0);
        msq  : unsigned(7 downto 0);
        dv   : unsigned(3 downto 0);
        vk   : unsigned(5 downto 0);
    end record;
    constant C_EV0 : t_ev := ('0', '0', '0', '0', '0', '0', '0', '0', '0', (others => '0'), "00",
                              (others => '0'), x"0001", "0000", "0000", (others => '0'),
                              (others => '0'), (others => '0'));
    signal e1, e2, e3, e4, e5 : t_ev := C_EV0;
    signal e6_v, e6_live, e6_br : std_logic := '0';
    signal e6_p : unsigned(10 downto 0) := (others => '0');
    signal e6_pos : signed(14 downto 0) := (others => '0');
    signal e6_vA, e6_vB : signed(6 downto 0) := (others => '0');
    signal e6_s : std_logic_vector(15 downto 0) := x"0001";
    signal e6_hue, e6_hueb : std_logic_vector(3 downto 0) := (others => '0');
    signal e7_v, e7_live, e7_br, e7_okA, e7_okB : std_logic := '0';
    signal e7_pA, e7_pB, e7_mnA, e7_mxA, e7_mnB, e7_mxB : unsigned(10 downto 0) := (others => '0');
    signal e7_pf : std_logic_vector(1 downto 0) := "00";
    signal e7_vA, e7_vB : signed(6 downto 0) := (others => '0');
    signal e7_s : std_logic_vector(15 downto 0) := x"0001";
    signal e7_hueA, e7_hueB : std_logic_vector(3 downto 0) := (others => '0');
    -- write port state
    signal s_widx : unsigned(7 downto 0) := (others => '0');
    signal s_lpg, s_lmg : unsigned(11 downto 0) := (others => '0');
    signal b_pend, b_ok : std_logic := '0';
    signal b_p, b_mx : unsigned(10 downto 0) := (others => '0');
    signal b_wd : std_logic_vector(63 downto 0) := C_SENT;
    -- draw / compose (suffix = stage)
    signal s_core13, s_inc13, s_inp13, s_hlh13 : std_logic := '0';
    signal s_dlc13, s_drp13 : signed(11 downto 0) := (others => '0');
    signal s_huec13, s_huep13 : std_logic_vector(3 downto 0) := (others => '0');
    signal s_cnt14 : unsigned(5 downto 0) := (others => '0');
    signal s_huew14, s_hue14, s_hue15, s_hue16, s_hue17, s_hue18 : std_logic_vector(3 downto 0) := (others => '0');
    signal s_bc14, s_bp14, s_bh15, s_bv15 : unsigned(4 downto 0) := (others => '0');
    signal s_gv14, s_hlh14, s_gv15, s_gh15, s_halo15 : std_logic := '0';
    signal s_gold16, s_halo16, s_glint16 : std_logic := '0';
    signal s_shade16 : unsigned(3 downto 0) := (others => '0');
    signal s_L16 : unsigned(4 downto 0) := (others => '0');
    signal s_gold17, s_halo17, s_gold18, s_halo18, s_gold19, s_halo19 : std_logic := '0';
    signal s_gold20, s_halo20, s_gold21 : std_logic := '0';
    signal s_rom19, s_pic19 : std_logic_vector(31 downto 0) := C_BLACK;
    signal s_ph19 : std_logic_vector(19 downto 0) := (others => '0');
    signal s_gy20, s_gu20, s_gv20, s_gy21, s_gu21, s_gv21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_py20, s_pu20, s_pv20 : signed(15 downto 0) := (others => '0');
    signal s_by21, s_bu21, s_bv21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y22 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y23, s_y24, s_y25, s_y26, s_y27, s_y28x, s_yf28 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y29, s_y30, s_y31, s_y32 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u29, s_v29, s_u30, s_v30, s_u31, s_v31, s_u32, s_v32 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');   -- s_avid_sr(k) = avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- RAMs: one write statement, registered read, no conditional read
    ---------------------------------------------------------------------------
    p_list : process(clk)
    begin
        if rising_edge(clk) then
            s_l_q <= lram(to_integer(s_l_ra));
            if s_l_we = '1' then lram(to_integer(s_l_wa)) <= s_l_wd; end if;
        end if;
    end process p_list;

    p_col : process(clk)
    begin
        if rising_edge(clk) then
            s_col_q <= colram(to_integer(s_col_ra));
            if s_col_we = '1' then colram(to_integer(s_col_wa)) <= s_col_wd; end if;
        end if;
    end process p_col;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_rr_q <= ring(to_integer(s_rr_a));
            ring(to_integer(s_rw_a)) <= s_rw_d;
        end if;
    end process p_ring;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_rom_q <= C_METAL(to_integer(s_rom_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches and derived constants
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_s : unsigned(14 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_xs <= f_vstep_x(s_tid);
            s_ys <= f_vstep_y(s_tid);

            -- vblank zone: no avid for 1024 clocks (hblank is at most 370)
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            -- K1..K3: 64 zones, +-3 count deadband after warm-up
            v_k := unsigned(registers_in(0));
            if s_warm /= 15 or ((v_k(3 downto 0) >= 3 or v_k(9 downto 4) = 0) and
                                (v_k(3 downto 0) <= 12 or v_k(9 downto 4) = 63)) then
                s_j1 <= v_k(9 downto 4);
            end if;
            v_k := unsigned(registers_in(1));
            if s_warm /= 15 or ((v_k(3 downto 0) >= 3 or v_k(9 downto 4) = 0) and
                                (v_k(3 downto 0) <= 12 or v_k(9 downto 4) = 63)) then
                s_j2 <= v_k(9 downto 4);
            end if;
            v_k := unsigned(registers_in(2));
            if s_warm /= 15 or ((v_k(3 downto 0) >= 3 or v_k(9 downto 4) = 0) and
                                (v_k(3 downto 0) <= 12 or v_k(9 downto 4) = 63)) then
                s_j3 <= v_k(9 downto 4);
            end if;
            -- K4, K5: 16 zones
            v_k := unsigned(registers_in(3));
            if s_warm /= 15 or ((v_k(5 downto 0) >= 6 or v_k(9 downto 6) = 0) and
                                (v_k(5 downto 0) <= 57 or v_k(9 downto 6) = 15)) then
                s_j4 <= v_k(9 downto 6);
            end if;
            v_k := unsigned(registers_in(4));
            if s_warm /= 15 or ((v_k(5 downto 0) >= 6 or v_k(9 downto 6) = 0) and
                                (v_k(5 downto 0) <= 57 or v_k(9 downto 6) = 15)) then
                s_j5 <= v_k(9 downto 6);
            end if;
            -- K6 Metal (4 labels): the firmware loads the default as the label
            -- INDEX; honour it until the knob first reads a real position
            v_k := unsigned(registers_in(5));
            if v_k >= 4 then s_live6 <= '1'; end if;
            if s_live6 = '0' and v_k < 4 then
                s_metk <= v_k(1 downto 0);
            elsif s_warm /= 15 or ((v_k(7 downto 0) >= 12 or v_k(9 downto 8) = 0) and
                                   (v_k(7 downto 0) <= 243 or v_k(9 downto 8) = 3)) then
                s_metk <= v_k(9 downto 8);
            end if;
            -- P12 Reach glide (1/8 of the gap a field)
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            -- per field (zone entry: exactly once per vertical blank)
            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_l1 <= s_j1; s_l2 <= s_j2; s_l3 <= s_j3; s_l4 <= s_j4; s_l5 <= s_j5;
                s_metal <= s_metk;
                s_seek  <= registers_in(6)(0);
                s_scat  <= registers_in(6)(1);
                s_mask  <= registers_in(6)(2);
                s_shad  <= registers_in(6)(3);
                s_glint <= registers_in(6)(4);
                s_tg <= s_tg + 7;
            end if;

            -- derived constants (one op per register)
            s_thr  <= to_unsigned(120, 8) - resize(s_l1, 8) - resize(s_l1(5 downto 3), 8);
            s_dens <= to_unsigned(C_DENS(to_integer(s_l1)), 8);
            s_dens4 <= resize(s_dens, 10);
            s_clv  <= to_unsigned(64, 7) - resize(s_l1(5 downto 1), 7) - resize(s_l1(5 downto 3), 7);
            s_j2sq <= s_l2 * s_l2;
            s_pbv  <= to_unsigned(2, 12) + resize(s_j2sq(11 downto 7), 12);
            s_pkv  <= to_unsigned(10, 12) + resize(s_l3, 12) + resize(s_l3(5 downto 2), 12) + resize(s_l3(5 downto 4), 12);
            s_vmax <= to_unsigned(4, 6) + resize(s_l3(5 downto 1), 6);
            s_vmaxs <= signed(resize(s_vmax, 8));
            s_wv   <= to_unsigned(1, 6) + resize(s_l4, 6) - resize(s_l4(3 downto 2), 6);
            s_hv   <= resize(s_wv(5 downto 1), 6) + 2;
            if s_l5 = 15 then s_f <= (others => '0'); else s_f <= to_unsigned(16, 5) - resize(s_l5, 5); end if;
            -- death probability (of 4096) per virtual line: 102 (1 - reach)^2
            s_u6  <= to_unsigned(63, 6) - s_g(11 downto 6);
            s_usq <= s_u6 * s_u6;
            s_pdv <= resize(s_usq(11 downto 5), 12) - resize(s_usq(11 downto 8), 12) - resize(s_usq(11 downto 9), 12);
            -- per field line
            v_s := f_sy(s_pbv, s_tid); s_pb <= v_s(9 downto 0);
            v_s := f_sy(s_pkv, s_tid); s_pk <= v_s(9 downto 0);
            v_s := f_sy(s_pdv, s_tid); s_pd <= v_s(11 downto 0);
            s_wr  <= signed(resize(f_sx(resize(s_wv, 7), s_xs), 12));
            s_hr  <= signed(resize(f_sx(resize(s_hv, 7), s_xs), 12));
            s_wh  <= s_wr + s_hr;
            s_clr <= signed(resize(f_sx(s_clv, s_xs), 12));
            s_T   <= f_ln(s_wv(4 downto 0) & '0', s_tid);
            if s_shad = '1' then s_Th <= f_ln(s_hv, s_tid); else s_Th <= (others => '0'); end if;
            s_Tt  <= s_T + s_Th;
            if s_wr <= 2 then s_ks <= "11"; elsif s_wr <= 4 then s_ks <= "10"; else s_ks <= "01"; end if;
            if s_w > 64 then
                s_hi  <= s_w - C_EDGE;
                s_hi8 <= s_w - C_EDGE - C_G;
                s_shi <= s_w - C_SEDGE;
            else
                s_hi  <= (others => '0');
                s_hi8 <= (others => '0');
                s_shi <= (others => '0');
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Line events: hsync->avid parity vote (generated colours swap U/V on
    -- off-parity lines), width latched per field, virtual line counter
    ---------------------------------------------------------------------------
    p_line : process(clk)
    begin
        if rising_edge(clk) then
            s_hsq <= data_in.hsync_n;
            if s_hsq = '1' and data_in.hsync_n = '0' then s_ph <= '0'; else s_ph <= not s_ph; end if;
            if data_in.avid = '1' and s_avid_sr(0) = '0' then
                s_swl <= s_ph xor s_upar;
                if s_ph = '1' then
                    if s_upc /= 15 then s_upc <= s_upc + 1; end if;
                else
                    if s_upc /= 0 then s_upc <= s_upc - 1; end if;
                end if;
            end if;
            if s_upc = 15 then s_upar <= '1'; elsif s_upc = 0 then s_upar <= '0'; end if;

            if s_zone = '1' then
                s_vl <= (others => '0');
            elsif data_in.avid = '0' and s_avid_sr(0) = '1' then
                s_vl <= s_vl + s_ys;
            end if;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
            elsif data_in.avid = '0' and s_avid_sr(0) = '1' then
                if s_rx(1) + 1 > s_wmax then s_wmax <= s_rx(1) + 1; end if;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Walk (stage 12 = the walk clock T(x)): pointer through the sorted tip
    -- list, decode pipe, line-end sentinel / bank swap / priming, events
    ---------------------------------------------------------------------------
    p_walk : process(clk)
        variable v_x   : unsigned(10 downto 0);
        variable v_tgi : std_logic_vector(1 downto 0);
        variable v_seed : std_logic;
        variable v_mn, v_mx : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            v_x := s_rx(12);

            -- line end at the walk stage
            if s_avid_sr(11) = '0' and s_avid_sr(12) = '1' then
                s_hb <= (others => '0');
            elsif s_hb /= 63 then
                s_hb <= s_hb + 1;
            end if;
            if s_hb = 12 then s_sent <= '1'; else s_sent <= '0'; end if;
            if s_hb = 8 then s_first <= '0'; end if;
            -- armed from the walk-stage line start until the priming of the next
            -- list (in blanking the column keeps counting past every entry)
            if s_avid_sr(10) = '1' and s_avid_sr(11) = '0' then s_arm <= '1'; end if;

            -- fetch pipe: RAM -> fabric register -> n1 (or cur when priming)
            s_lf <= s_l_q;
            s_tq <= s_tgi; s_tf <= s_tq;

            v_tgi := "00";
            c_new <= '0';
            e0_v <= '0';
            s_lres <= '0';
            -- next-clock advance flag and seed gap test
            s_advn <= '0';
            s_sgap <= '0';

            if s_zgo = '1' then
                -- field start: the list is empty, nothing carries between frames
                c_end <= '1'; n_end <= '1';
                p_ok <= '0';
                s_lres <= '1';
                s_first <= '1';
            elsif s_hb = 14 then
                -- the bank written this line becomes the read bank; prime cur / n1
                s_arm <= '0';
                s_rb <= not s_rb;
                s_lres <= '1';
                p_ok <= '0';
                c_end <= '1';
                s_l_ra <= (not s_rb) & x"00";
                v_tgi := "11";
            elsif s_hb = 15 then
                s_l_ra <= s_rb & x"01";
                s_ridx <= to_unsigned(2, 8);
                v_tgi := "10";
            elsif s_advn = '1' then
                -- cur retires: event; cur <= n1 (seam edges computed here); prev <= cur;
                -- fetch the next entry
                e0_v <= '1'; e0_sd <= '0';
                e0_p <= c_p; e0_pf <= c_pf; e0_vv <= c_v; e0_rs <= c_rs; e0_hue <= c_hue;
                e0_gs <= c_gs; e0_lum <= c_lum;
                p_ok <= '1'; p_sr <= c_sr; p_srh <= c_srh; p_mxc <= c_mxc; p_hue <= c_hue;
                c_p <= n_p; c_pf <= n_pf; c_v <= n_v; c_rs <= n_rs; c_hue <= n_hue; c_end <= n_end;
                c_mn <= n_mn; c_mx <= n_mx;
                v_mn := signed('0' & n_mn); v_mx := signed('0' & n_mx);
                c_sl <= v_mn - s_wr; c_slh <= v_mn - s_wh; c_mnc <= v_mn - s_clr;
                c_sr <= v_mx + s_wr; c_srh <= v_mx + s_wh; c_mxc <= v_mx + s_clr;
                c_new <= '1';
                s_l_ra <= s_rb & s_ridx;
                s_ridx <= s_ridx + 1;
                v_tgi := "10";
            else
                if s_arm = '1' and c_end = '0' and s_rx(11) >= c_mx then s_advn <= '1'; end if;
                -- next column clear of cur and prev (cur cannot change for 7 clocks after a step)
                if (c_end = '1' or signed('0' & s_rx(11)) < c_mnc) and (p_ok = '0' or signed('0' & s_rx(11)) > p_mxc) then
                    s_sgap <= '1';
                end if;
                -- steering sample: the gradient at the tip (or where cur came in)
                if v_x <= c_p or c_new = '1' then
                    c_gs <= s_gxa;
                    c_lum <= s_y8(12);
                end if;
                -- seed into a gap between spans (all four tests registered for this column)
                v_seed := s_sdc and s_gate and s_srng and s_sgap;
                if v_seed = '1' then
                    e0_v <= '1'; e0_sd <= '1';
                    e0_p <= v_x; e0_pf <= "10"; e0_vv <= (others => '0');
                    e0_rs <= std_logic_vector(s_h4(23 downto 9)) & '1';
                    e0_hue <= std_logic_vector(s_h4(11 downto 8));
                    e0_gs <= (others => '0'); e0_lum <= s_y8(12);
                end if;
            end if;
            s_tgi <= v_tgi;

            -- fetched entry lands in n1 (or straight in cur when priming)
            if s_tf(1) = '1' then
                if s_tf(0) = '1' then
                    c_p <= unsigned(s_lf(63 downto 53)); c_pf <= s_lf(52 downto 51); c_v <= signed(s_lf(50 downto 44));
                    c_mn <= unsigned(s_lf(43 downto 33)); c_mx <= unsigned(s_lf(32 downto 22));
                    c_hue <= s_lf(21 downto 18); c_rs <= s_lf(17 downto 2); c_end <= s_lf(0);
                    v_mn := signed('0' & s_lf(43 downto 33)); v_mx := signed('0' & s_lf(32 downto 22));
                    c_sl <= v_mn - s_wr; c_slh <= v_mn - s_wh; c_mnc <= v_mn - s_clr;
                    c_sr <= v_mx + s_wr; c_srh <= v_mx + s_wh; c_mxc <= v_mx + s_clr;
                    c_new <= '1';
                else
                    n_p <= unsigned(s_lf(63 downto 53)); n_pf <= s_lf(52 downto 51); n_v <= signed(s_lf(50 downto 44));
                    n_mn <= unsigned(s_lf(43 downto 33)); n_mx <= unsigned(s_lf(32 downto 22));
                    n_hue <= s_lf(21 downto 18); n_rs <= s_lf(17 downto 2); n_end <= s_lf(0);
                end if;
            end if;
        end if;
    end process p_walk;

    ---------------------------------------------------------------------------
    -- Event pipe: step each retiring crack / place each seed, write sorted
    ---------------------------------------------------------------------------
    p_event : process(clk)
        variable v_m  : unsigned(13 downto 0);
        variable v_k  : signed(6 downto 0);
        variable v_pos : signed(14 downto 0);
        variable v_u  : unsigned(14 downto 0);
        variable v_a, v_b : signed(7 downto 0);
        variable v_pa : unsigned(10 downto 0);
        variable v_ok : std_logic;
    begin
        if rising_edge(clk) then
            -- E1: first state step; steering direction; Mask
            e1 <= C_EV0;
            e1.v <= e0_v; e1.sd <= e0_sd; e1.p <= e0_p; e1.pf <= e0_pf; e1.vv <= e0_vv; e1.hue <= e0_hue;
            if e0_sd = '1' then e1.s <= e0_rs; else e1.s <= f_xs16(e0_rs); end if;
            if e0_gs > C_DEAD or e0_gs < -C_DEAD then e1.sten <= '1'; end if;
            if s_seek = '0' then e1.sneg <= not e0_gs(10); else e1.sneg <= e0_gs(10); end if;   -- Dark: go down the slope
            if s_mask = '1' and e0_sd = '0' and e0_lum < 96 then e1.kill <= '1'; end if;

            -- E2: second step; death; the fork child's hue
            e2 <= e1;
            e2.s <= f_xs16(e1.s);
            if e1.sd = '0' and unsigned(e1.s(11 downto 0)) < s_pd then e2.die <= '1'; else e2.die <= '0'; end if;
            e2.hueb <= e1.s(15 downto 12);

            -- E3: third step; kink, its direction and r^2
            e3 <= e2;
            e3.s <= f_xs16(e2.s);
            if unsigned(e2.s(9 downto 0)) < s_pk then e3.kink <= '1'; else e3.kink <= '0'; end if;
            if e2.sten = '1' and e2.sd = '0' then e3.neg <= e2.sneg; else e3.neg <= e2.s(14); end if;
            e3.msq <= unsigned(e2.s(13 downto 10)) * unsigned(e2.s(13 downto 10));

            -- E4: kink magnitude r^2 vmax (Q4.2); fork
            e4 <= e3;
            v_m := e3.msq * s_vmax;
            e4.vk <= v_m(13 downto 8);
            if e3.sd = '0' and unsigned(e3.s(9 downto 0)) < s_pb then e4.br <= '1'; else e4.br <= '0'; end if;
            e4.dv <= resize(unsigned(e3.s(12 downto 10)), 4) + 2;

            -- E5: new velocity
            e5 <= e4;
            if e4.sd = '1' then
                v_k := signed(resize(e4.vk(5 downto 1), 7));
            else
                v_k := signed(resize(e4.vk, 7));
            end if;
            if e4.sd = '1' or e4.kink = '1' then
                if e4.neg = '1' then e5.vv <= -v_k; else e5.vv <= v_k; end if;
            end if;

            -- E6: position; fork velocities (clamped to +-vmax)
            e6_v <= e5.v; e6_br <= e5.br; e6_p <= e5.p; e6_s <= e5.s; e6_hue <= e5.hue; e6_hueb <= e5.hueb;
            e6_live <= e5.v and not e5.die and not e5.kill;
            v_u := shift_left(resize(e5.p, 15), 2);
            if e5.sd = '1' then
                v_pos := signed(v_u) + 2;
            else
                v_pos := signed(v_u) + signed(resize(unsigned(e5.pf), 15)) + resize(e5.vv, 15);
            end if;
            e6_pos <= v_pos;
            v_a := resize(e5.vv, 8) - signed(resize(e5.dv, 8));
            v_b := resize(e5.vv, 8) + signed(resize(e5.dv, 8));
            if v_a < -s_vmaxs then v_a := -s_vmaxs; end if;
            if v_b > s_vmaxs then v_b := s_vmaxs; end if;
            if e5.br = '1' then e6_vA <= v_a(6 downto 0); else e6_vA <= e5.vv; end if;
            e6_vB <= v_b(6 downto 0);

            -- E7: both candidates' spans.  A swept from p (its last position)
            -- to pA; the fork child B sits C_G right of A and starts C_G - 1
            -- right of p.
            e7_v <= e6_v; e7_live <= e6_live; e7_br <= e6_br;
            e7_vA <= e6_vA; e7_vB <= e6_vB;
            e7_s <= e6_s; e7_hueA <= e6_hue; e7_hueB <= e6_hueb;
            e7_pf <= std_logic_vector(e6_pos(1 downto 0));
            v_pa := unsigned(e6_pos(12 downto 2));
            v_ok := '0';
            if e6_pos >= 0 and e6_pos < 8192 and v_pa >= C_EDGE and v_pa <= s_hi then v_ok := '1'; end if;
            e7_okA <= v_ok;
            v_ok := '0';
            if e6_pos >= 0 and e6_pos < 8192 - 4 * C_G and v_pa >= C_EDGE and v_pa <= s_hi8 then v_ok := '1'; end if;
            e7_okB <= v_ok;
            e7_pA <= v_pa;
            e7_pB <= v_pa + C_G;
            if v_pa >= e6_p then
                e7_mnA <= e6_p; e7_mxA <= v_pa;
            else
                e7_mnA <= v_pa; e7_mxA <= e6_p;
            end if;
            if v_pa + 1 >= e6_p then
                e7_mnB <= e6_p + (C_G - 1); e7_mxB <= v_pa + C_G;
            else
                e7_mnB <= v_pa + C_G; e7_mxB <= e6_p + (C_G - 1);
            end if;

            -- E8: sorted write (drop = merge into the crack before).  Address and
            -- data load every clock from registered selects; only the enable,
            -- the index and the last-written bounds depend on the compares.
            s_l_wa <= (not s_rb) & s_widx;
            if s_sent = '1' then
                s_l_wd <= C_SENT;
            elsif b_pend = '1' then
                s_l_wd <= b_wd;
            else
                s_l_wd <= std_logic_vector(e7_pA) & e7_pf & std_logic_vector(e7_vA) &
                          std_logic_vector(e7_mnA) & std_logic_vector(e7_mxA) & e7_hueA & e7_s & "00";
            end if;
            s_l_we <= '0';
            if s_lres = '1' then
                s_widx <= (others => '0');
                s_lpg <= (others => '0');
                s_lmg <= (others => '0');
                b_pend <= '0';
            elsif s_sent = '1' then
                s_l_we <= '1';
            elsif b_pend = '1' then
                b_pend <= '0';
                if b_ok = '1' and resize(b_p, 12) >= s_lpg and resize(b_mx, 12) >= s_lmg and s_widx < 254 then
                    s_l_we <= '1';
                    s_widx <= s_widx + 1;
                    s_lpg <= resize(b_p, 12) + C_G;
                    s_lmg <= resize(b_mx, 12) + (C_G - 1);
                end if;
            elsif e7_v = '1' and e7_live = '1' then
                if e7_okA = '1' and resize(e7_pA, 12) >= s_lpg and resize(e7_mxA, 12) >= s_lmg and s_widx < 254 then
                    s_l_we <= '1';
                    s_widx <= s_widx + 1;
                    s_lpg <= resize(e7_pA, 12) + C_G;
                    s_lmg <= resize(e7_mxA, 12) + (C_G - 1);
                end if;
                b_pend <= e7_br;
            end if;
            -- the fork child, held for the next clock
            b_ok <= e7_okB;
            b_p  <= e7_pB;
            b_mx <= e7_mxB;
            b_wd <= std_logic_vector(e7_pB) & e7_pf & std_logic_vector(e7_vB) &
                    std_logic_vector(e7_mnB) & std_logic_vector(e7_mxB) & e7_hueB &
                    e7_s(7 downto 0) & e7_s(15 downto 8) & "00";
        end if;
    end process p_event;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_y6  : unsigned(5 downto 0);
        variable v_ag  : signed(10 downto 0);
        variable v_d   : signed(7 downto 0);
        variable v_ca  : std_logic_vector(9 downto 0);
        variable v_cn  : unsigned(5 downto 0);
        variable v_b1, v_b2 : signed(11 downto 0);
        variable v_bv  : signed(6 downto 0);
        variable v_bev : unsigned(4 downto 0);
        variable v_sh  : unsigned(8 downto 0);
        variable v_xv  : unsigned(11 downto 0);
        variable v_ph  : unsigned(11 downto 0);
        variable v_lv  : unsigned(6 downto 0);
        variable v_gu, v_gv : std_logic_vector(9 downto 0);
        variable v_t   : unsigned(23 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_dl  : unsigned(9 downto 0);
        variable v_r   : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: input; column (keeps counting through blanking for the walk)
            s_y1 <= unsigned(data_in.y);
            s_u1 <= unsigned(data_in.u);
            s_v1 <= unsigned(data_in.v);
            if data_in.avid = '1' and s_avid_sr(0) = '0' then
                s_rx(1) <= (others => '0');
            elsif s_rx(1) /= 2046 then
                s_rx(1) <= s_rx(1) + 1;
            end if;
            for i in 2 to 16 loop s_rx(i) <= s_rx(i - 1); end loop;

            -- 2: picture into the ring; 4-pixel luma box
            s_rw_a <= s_vw;
            s_vw <= s_vw + 1;
            s_rw_d <= std_logic_vector(s_y1) & std_logic_vector(s_u1) & std_logic_vector(s_v1) & "00";
            s_y8(2) <= s_y1(9 downto 2);
            for i in 3 to 17 loop s_y8(i) <= s_y8(i - 1); end loop;
            s_box2 <= s_box2 + resize(s_y1(9 downto 2), 10) - resize(s_y8(5), 10);    -- stages 2..5
            s_boxb <= s_boxb + resize(s_y8(13), 10) - resize(s_y8(17), 10);           -- stages 14..17

            -- 3: horizontal gradient box(p) - box(p - 12), centred ~7 px back;
            --    s_gxa is centred on the walk column at T(x)
            s_gx3 <= signed(resize(s_box2, 11)) - signed(resize(s_boxb, 11));
            s_gxd <= s_gx3;
            s_gxa <= s_gxd;
            v_ag := abs(s_gx3);
            s_agx <= unsigned(v_ag(9 downto 2));

            -- content hash, aligned at T(x): key at T-3 from luma at x+4, x, x-4
            -- (Scatter: luma at x and the 32-px cell)
            v_t := (others => '0');
            if s_scat = '0' then
                v_t(14 downto 10) := s_y8(3)(7 downto 3);
                v_t(9 downto 5)   := s_y8(7)(7 downto 3);
                v_t(4 downto 0)   := s_y8(11)(7 downto 3);
            else
                v_t(16 downto 12) := s_y8(7)(7 downto 3);
                v_t(11 downto 6)  := s_rx(7)(10 downto 5);
                v_t(5 downto 0)   := s_vl(17 downto 12);
            end if;
            s_hk <= v_t xor x"9E3779";
            s_h1 <= s_hk + shift_left(s_hk, 10);
            v_t := s_h1 xor shift_right(s_h1, 6);
            s_h2 <= v_t + shift_left(v_t, 3);
            v_t := s_h2 xor shift_right(s_h2, 11);
            s_h3 <= v_t + shift_left(v_t, 9);
            s_h4 <= s_h3;
            -- seed gate for T(x), from the hash at T-1
            if s_scat = '0' then
                if s_h3(7 downto 0) < s_dens then s_gate <= '1'; else s_gate <= '0'; end if;
            else
                if s_h3(9 downto 0) < s_dens4 then s_gate <= '1'; else s_gate <= '0'; end if;
            end if;

            -- 8..10: column RAM {cnt, luma above, hue} for this column
            s_col_ra <= s_rx(7);
            s_cq10 <= s_col_q;
            -- 11: vertical edge term; column word (first line of a field: none)
            v_y6 := s_y8(10)(7 downto 2);
            if s_first = '1' then
                s_agy <= (others => '0');
                s_ca11 <= (others => '0');
            else
                v_d := signed(resize(v_y6, 8)) - signed(resize(unsigned(s_cq10(9 downto 4)), 8));
                if v_d < 0 then v_d := -v_d; end if;
                s_agy <= unsigned(v_d(5 downto 0)) & "00";
                s_ca11 <= s_cq10(15 downto 10) & s_cq10(3 downto 0);
            end if;
            s_ca12 <= s_ca11;
            s_ca13 <= s_ca12;
            -- 12: seed candidate for the walk (edge strength / lit) and its column range
            if s_scat = '0' then
                if resize(s_agx, 9) + resize(s_agy, 9) > s_thr then s_sdc <= '1'; else s_sdc <= '0'; end if;
            else
                if s_y8(11) > 48 then s_sdc <= '1'; else s_sdc <= '0'; end if;
            end if;
            if s_avid_sr(10) = '1' and s_rx(11) >= C_SEDGE and s_rx(11) <= s_shi then s_srng <= '1'; else s_srng <= '0'; end if;

            -- 13: draw tests against cur / prev (walk clock of this pixel)
            if c_end = '0' and s_rx(12) >= c_mn then s_core13 <= '1'; else s_core13 <= '0'; end if;
            if c_end = '0' and signed('0' & s_rx(12)) >= c_sl then s_inc13 <= '1'; else s_inc13 <= '0'; end if;
            if p_ok = '1' and signed('0' & s_rx(12)) <= p_sr then s_inp13 <= '1'; else s_inp13 <= '0'; end if;
            if (c_end = '0' and signed('0' & s_rx(12)) >= c_slh) or (p_ok = '1' and signed('0' & s_rx(12)) <= p_srh) then
                s_hlh13 <= '1';
            else
                s_hlh13 <= '0';
            end if;
            s_dlc13 <= signed('0' & s_rx(12)) - c_sl;      -- cur's seam runs on past x (x <= its end)
            s_drp13 <= p_sr - signed('0' & s_rx(12));      -- prev's seam began before x
            s_huec13 <= c_hue;
            s_huep13 <= p_hue;

            -- 14: column countdown (a core pixel restarts T + Th lines below it);
            --     horizontal bevel of the cur / prev seams
            v_ca := s_ca13;
            v_cn := unsigned(v_ca(9 downto 4));
            if s_core13 = '1' then
                s_cnt14 <= s_Tt;
                s_huew14 <= s_huec13;
            else
                if v_cn /= 0 then s_cnt14 <= v_cn - 1; else s_cnt14 <= (others => '0'); end if;
                s_huew14 <= v_ca(3 downto 0);
            end if;
            if s_inc13 = '1' then
                s_hue14 <= s_huec13;
            elsif s_inp13 = '1' then
                s_hue14 <= s_huep13;
            else
                s_hue14 <= v_ca(3 downto 0);
            end if;
            v_b1 := s_dlc13;
            v_b2 := s_drp13;
            if s_inc13 = '0' or v_b1 < 0 then s_bc14 <= (others => '0');
            elsif v_b1 > 31 then s_bc14 <= (others => '1');
            else s_bc14 <= unsigned(v_b1(4 downto 0)); end if;
            if s_inp13 = '0' or v_b2 < 0 then s_bp14 <= (others => '0');
            elsif v_b2 > 31 then s_bp14 <= (others => '1');
            else s_bp14 <= unsigned(v_b2(4 downto 0)); end if;
            s_gv14 <= s_inc13 or s_inp13;
            s_hlh14 <= s_hlh13;

            -- 15: vertical gold / bevel; Shadow halo; column write-back
            if s_cnt14 > s_Th then s_gh15 <= '1'; else s_gh15 <= '0'; end if;
            s_gv15 <= s_gv14;
            if s_bc14 > s_bp14 then s_bh15 <= s_bc14; else s_bh15 <= s_bp14; end if;
            v_bv := signed(resize(s_Tt, 7)) - signed(resize(s_cnt14, 7));                 -- from the top
            if signed(resize(s_cnt14, 7)) - signed(resize(s_Th, 7)) - 1 < v_bv then
                v_bv := signed(resize(s_cnt14, 7)) - signed(resize(s_Th, 7)) - 1;          -- from the bottom
            end if;
            if v_bv < 0 then s_bv15 <= (others => '0'); else s_bv15 <= unsigned(v_bv(4 downto 0)); end if;
            if s_shad = '1' and (s_hlh14 = '1' or s_cnt14 /= 0) then s_halo15 <= '1'; else s_halo15 <= '0'; end if;
            s_hue15 <= s_hue14;
            s_col_wa <= s_rx(14);
            s_col_wd <= std_logic_vector(s_cnt14) & std_logic_vector(s_y8(14)(7 downto 2)) & s_huew14;
            s_col_we <= s_avid_sr(13);

            -- 16: bevel -> shade 0..15; glint band; picture light
            v_bev := (others => '0');
            if s_gv15 = '1' then v_bev := s_bh15; end if;
            if s_gh15 = '1' and s_bv15 > v_bev then v_bev := s_bv15; end if;
            v_sh := shift_left(resize(v_bev, 9), to_integer(s_ks));
            if v_sh > 15 then s_shade16 <= (others => '1'); else s_shade16 <= v_sh(3 downto 0); end if;
            s_gold16 <= s_gv15 or s_gh15;
            s_halo16 <= s_halo15 and not (s_gv15 or s_gh15);
            if s_xs = 4 then
                v_xv := shift_left(resize(s_rx(15), 12), 1);
            elsif s_xs = 3 then
                v_xv := resize(s_rx(15), 12) + resize(s_rx(15)(10 downto 1), 12);
            else
                v_xv := resize(s_rx(15), 12);
            end if;
            v_ph := v_xv + resize(s_vl(18 downto 8), 12) + shift_left(resize(s_tg, 12), 1);
            if s_glint = '1' and v_ph(9 downto 0) < 80 then s_glint16 <= '1'; else s_glint16 <= '0'; end if;
            s_L16 <= s_y8(15)(7 downto 3);
            s_hue16 <= s_hue15;

            -- 17: metal level 0..63 = 2 x bevel + picture light (+ glint); ROM
            --     address; ring read for this pixel (written at stage 2, 15 clocks back)
            v_lv := shift_left(resize(s_shade16, 7), 1) + resize(s_L16, 7);
            if s_glint16 = '1' then v_lv := v_lv + 16; end if;
            if v_lv > 63 then v_lv := to_unsigned(63, 7); end if;
            s_rom_a <= s_metal & v_lv(5 downto 0);
            s_rr_a <= s_vw - 15;
            s_gold17 <= s_gold16; s_halo17 <= s_halo16; s_hue17 <= s_hue16;

            -- 18: ROM / ring outputs (EBR registers)
            s_gold18 <= s_gold17; s_halo18 <= s_halo17; s_hue18 <= s_hue17;

            -- 19: fabric registers; Prism hue
            s_rom19 <= s_rom_q;
            s_pic19 <= s_rr_q;
            s_ph19 <= C_PRISM(to_integer(unsigned(s_hue18)));
            s_gold19 <= s_gold18; s_halo19 <= s_halo18;

            -- 20: metal colour (U/V swapped on off-parity lines); ground scaling
            s_gy20 <= unsigned(s_rom19(31 downto 22));
            if s_metal = 3 then
                v_gu := s_ph19(19 downto 10); v_gv := s_ph19(9 downto 0);
            else
                v_gu := s_rom19(21 downto 12); v_gv := s_rom19(11 downto 2);
            end if;
            if s_swl = '1' then
                s_gu20 <= unsigned(v_gv); s_gv20 <= unsigned(v_gu);
            else
                s_gu20 <= unsigned(v_gu); s_gv20 <= unsigned(v_gv);
            end if;
            s_py20 <= resize((signed(resize(unsigned(s_pic19(31 downto 22)), 11)) - 64) * signed(resize(s_f, 6)), 16);
            s_pu20 <= resize((signed(resize(unsigned(s_pic19(21 downto 12)), 11)) - 512) * signed(resize(s_f, 6)), 16);
            s_pv20 <= resize((signed(resize(unsigned(s_pic19(11 downto 2)), 11)) - 512) * signed(resize(s_f, 6)), 16);
            s_gold20 <= s_gold19; s_halo20 <= s_halo19;

            -- 21: ground (picture x f/16, halved in the Shadow groove)
            if s_halo20 = '1' then
                v_r := shift_right(s_py20, 5) + 64;  s_by21 <= unsigned(v_r(9 downto 0));
                v_r := shift_right(s_pu20, 5) + 512; s_bu21 <= unsigned(v_r(9 downto 0));
                v_r := shift_right(s_pv20, 5) + 512; s_bv21 <= unsigned(v_r(9 downto 0));
            else
                v_r := shift_right(s_py20, 4) + 64;  s_by21 <= unsigned(v_r(9 downto 0));
                v_r := shift_right(s_pu20, 4) + 512; s_bu21 <= unsigned(v_r(9 downto 0));
                v_r := shift_right(s_pv20, 4) + 512; s_bv21 <= unsigned(v_r(9 downto 0));
            end if;
            s_gy21 <= s_gy20; s_gu21 <= s_gu20; s_gv21 <= s_gv20;
            s_gold21 <= s_gold20;

            -- 22: compose
            if s_gold21 = '1' then
                s_y22 <= s_gy21; s_u22 <= s_gu21; s_v22 <= s_gv21;
            else
                s_y22 <= s_by21; s_u22 <= s_bu21; s_v22 <= s_bv21;
            end if;

            -- 23..28: output conditioning.  Chroma [1 2 2 2 2 2 2 2 1]/16 as
            -- two adjacent 8-sums (centre at stage 27 -> s_u28); luma [1 2 1]
            -- centred at stage 27 -> s_yf28.
            s_csu(0) <= s_u22; s_csv(0) <= s_v22;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u22 - s_csu(7);
            s_boxv <= s_boxv + s_v22 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u28 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v28 <= v_sc(13 downto 4);
            s_y23 <= s_y22; s_y24 <= s_y23; s_y25 <= s_y24; s_y26 <= s_y25;
            s_y27 <= s_y26; s_y28x <= s_y27;
            v_sy := resize(s_y26, 12) + shift_left(resize(s_y27, 12), 1) + resize(s_y28x, 12);
            s_yf28 <= v_sy(11 downto 2);
            -- 29: luma toward black, 64 + 13/16 (y - 64); blanking gate
            if s_yf28 > 64 then v_dl := s_yf28 - 64; else v_dl := (others => '0'); end if;
            if s_avid_sr(27) = '1' then
                s_y29 <= s_yf28 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u29 <= s_u28; s_v29 <= s_v28;
            else
                s_y29 <= to_unsigned(64, 10); s_u29 <= to_unsigned(512, 10); s_v29 <= to_unsigned(512, 10);
            end if;
            -- 30..32: pad to the 32-clock latency
            s_y30 <= s_y29; s_u30 <= s_u29; s_v30 <= s_v29;
            s_y31 <= s_y30; s_u31 <= s_u30; s_v31 <= s_v30;
            s_y32 <= s_y31; s_u32 <= s_u31; s_v32 <= s_v31;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y32);
    data_out.u       <= std_logic_vector(s_u32);
    data_out.v       <= std_logic_vector(s_v32);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture fissure;
