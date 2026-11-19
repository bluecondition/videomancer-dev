-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Ticker v0.6.1 diag -- Heirloom #20 (2026-10-21 rebuild; phase 2 2026-11-09), from WIN_20230319_01_27_56.
--
-- The photo: camera video run through a sample-and-hold mosaic.  A horizontal
-- S&H locked to H (45.9 px cells, every cell split into a 24 px "A" half and a
-- darker 22 px "B" half), a line hold re-sampled by the union of two free-running
-- vertical clocks (19.8 and 23.3 lines; slivers where they nearly coincide),
-- held for 56 / 87 / 99 lines in three bands in the lower half where a vertical
-- sine rises above a falling ramp (the tall bars), a luma window key that turns
-- A cells between 0.63 and 0.80 into saturated cyan and above it into white,
-- and soft analogue edges (16 px) with a pink luma->Cr fringe on warm greys.  The
-- camera saw a dark inverted triangle at the top centre (row-striped, two dim
-- dash columns).
--
-- The program: Video = the camera through exactly that processing.  Osc = a
-- generator with the photo's statistics: hashed greys through a 256-entry
-- quantile ROM fitted to the photo, highlights gated by a 77 px oscillator
-- (bright cells 2-3 cells apart, never adjacent), values re-rolled on staggered
-- 32-line steps per sample, and the dark wedge with its stripes and dashes.
--
--   K1  Cells   the H clock: x0.25 .. x1 (50 %, 41.8 across = the photo) .. x4, exponential
--   K2  Rows    vertical clock rate, 8 .. 263 rows per frame (54.6 = the photo)
--   K3  Cyan    key window position: low = the window drops and widens until cyan floods the
--               greys, high = only whites turn cyan
--   K4  Speed   bipolar (still in the centre): rows roll, highlights hop, the bars drift; past
--               the default speed either way it steepens and every row scrolls sideways at its
--               own rate (the ticker tape; none in 384..640)
--   K5  Gaps    B half-cell gain, black .. 0.87 (0.44 = the photo), and the A/B duty with it:
--               A = 25 % (slits) .. 53 % (the photo) .. 81 % of the cell
--   K6  Camera  how strongly the source shows: its contrast about a mid grey (C_PIV) before the
--               cell hold, 0 = flat grey cells, 50 % = the v0.6 look, 100 % = 2x (Osc wedge exempt)
--   S7  Picks   Cyan / Magenta key colour
--   S8  Grid    Even (the photo) / Mixed (every row its own cell size, x0.5 .. x2)
--   S9  Shadow  Photo / Deep (the old K6 at 512 / C_KSH_DEEP): Osc wedge size, Video dark-crush
--   S10 Mirror  Off / On (the right half replays the left half's cells mirrored)
--   S11 Source  Video / Osc
--   P12 HOLD    0 = no held bars .. photo (three bands at the bottom) .. 100 % = the
--               whole lower frame melts into tall bars
--
-- Virtual 1080-line coordinates (timing ID steps): cells and rows keep their
-- size at SD / 720p / 1080p, the grid and the wedge are centred.  Generated avid,
-- output conditioning ([1 2 1] luma, [1 2 2 2 2 2 2 2 1]/16 chroma, luma 13/16),
-- blanking gate.  Latency 36 clocks in every mode.  8 EBR.  Colours: u = Cr.
-- Phase 2: the per-row engine (5 serial products per tick line on the shared multiplier) gives
-- every row its own cell clock, centre-anchored start phase, scroll offset, hash key base and
-- highlight-gate offset; all of them reduce to the v0.5 values at the defaults.
--
-- K6 camera pass 2026-11-15 (v0.6.1 diag): K6 = Camera (new); the old K6 Shadow moved to S9 as two
-- positions; S9's old Edges switch (Soft / Hard, the weakest control here) is fixed at Soft.  The two
-- K6 stages (source x gain, + pivot term) sit before the cell RAM: latency 34 -> 36 in every mode
-- (even, U/V parity unchanged).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture ticker of program_top is
    constant C_LAT : integer := 36;   -- matches processing_delay_clks (v0.6.0: 34; +2 for the K6 camera stages)

    constant C_AB   : unsigned(15 downto 0) := to_unsigned(34996, 16);   -- A/B split of a cell
    constant C_BLK  : unsigned(9 downto 0) := to_unsigned(77, 10);       -- black, L units (1024 = 1.0)
    -- K6 camera pass: contrast pivot (L units) and the S9 Deep shadow setting (old K6 scale, 512 = 1x)
    constant C_PIV  : integer := 320;
    constant C_KSH_PHOTO : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_KSH_DEEP  : unsigned(9 downto 0) := to_unsigned(800, 10);
    constant KP : integer := 512;   -- the same two settings as integers: the wedge / crush registers take
    constant KD : integer := 800;   -- per-state constants (no adders on copies of one switch bit)
    function f_ss(f : std_logic; d, q : integer; w : natural) return signed is
    begin
        if f = '1' then return to_signed(d, w); else return to_signed(q, w); end if;
    end function;
    function f_su(f : std_logic; d, q : integer; w : natural) return unsigned is
    begin
        if f = '1' then return to_unsigned(d, w); else return to_unsigned(q, w); end if;
    end function;
    constant C_PHC  : unsigned(15 downto 0) := to_unsigned(39190, 16);   -- cell phase at the centre
    constant C_P1   : unsigned(19 downto 0) := to_unsigned(305660, 20);  -- clock phases at the top
    constant C_P2   : unsigned(19 downto 0) := to_unsigned(385876, 20);
    constant C_PH   : unsigned(15 downto 0) := to_unsigned(24183, 16);   -- hold sine phase at the top

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
    -- per-timing line constants: rows scale (x971/256), hold phase, hold ramp, gate phases
    function f_ysc(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 => return to_unsigned(4316, 13); when 4 => return to_unsigned(2184, 13);
            when 8 => return to_unsigned(3641, 13); when 12 => return to_unsigned(1820, 13);
            when 5 | 6 | 14 => return to_unsigned(1456, 13); when 1 | 2 | 10 => return to_unsigned(1942, 13);
            when others => return to_unsigned(971, 13);
        end case;
    end function;
    function f_dh(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 => return to_unsigned(1733, 16); when 4 => return to_unsigned(877, 16);
            when 8 => return to_unsigned(1462, 16); when 12 => return to_unsigned(731, 16);
            when 5 | 6 | 14 => return to_unsigned(585, 16); when 1 | 2 | 10 => return to_unsigned(780, 16);
            when others => return to_unsigned(390, 16);
        end case;
    end function;
    function f_yth(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 => return to_unsigned(3494, 12); when 4 => return to_unsigned(1768, 12);
            when 8 => return to_unsigned(2947, 12); when 12 => return to_unsigned(1473, 12);
            when 5 | 6 | 14 => return to_unsigned(1179, 12); when 1 | 2 | 10 => return to_unsigned(1572, 12);
            when others => return to_unsigned(786, 12);
        end case;
    end function;
    function f_da(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 => return to_unsigned(11793, 16); when 4 => return to_unsigned(5969, 16);
            when 8 => return to_unsigned(9948, 16); when 12 => return to_unsigned(4974, 16);
            when 5 | 6 | 14 => return to_unsigned(3979, 16); when 1 | 2 | 10 => return to_unsigned(5306, 16);
            when others => return to_unsigned(2653, 16);
        end case;
    end function;
    function f_db(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 => return to_unsigned(3005, 16); when 4 => return to_unsigned(1521, 16);
            when 8 => return to_unsigned(2535, 16); when 12 => return to_unsigned(1267, 16);
            when 5 | 6 | 14 => return to_unsigned(1014, 16); when 1 | 2 | 10 => return to_unsigned(1352, 16);
            when others => return to_unsigned(676, 16);
        end case;
    end function;
    -- triangle of a 16-bit phase: +2048 at 0, -2048 at 1/2
    function f_tri12(p : unsigned(15 downto 0)) return signed is
        variable u : signed(13 downto 0);
    begin
        u := signed(resize(p(15 downto 4), 14)) - 2048;
        if u < 0 then u := -u; end if;
        return resize(shift_left(u, 1) - 2048, 13);
    end function;

    -- grey quantile ROM (photo-fitted), L units
    type t_rom is array (0 to 255) of integer range 0 to 1023;
    constant C_GREY : t_rom := (
        77, 79, 80, 81, 81, 81, 83, 84, 84, 85, 85, 86, 87, 88, 89, 89,
        89, 89, 90, 90, 92, 93, 93, 94, 96, 97, 97, 99, 101, 101, 101, 102,
        105, 106, 108, 109, 112, 112, 113, 114, 116, 117, 117, 120, 120, 121, 121, 122,
        124, 124, 124, 125, 125, 125, 126, 126, 126, 127, 127, 127, 128, 128, 128, 128,
        129, 129, 129, 129, 130, 130, 131, 131, 131, 132, 132, 133, 133, 134, 134, 135,
        136, 137, 138, 139, 141, 142, 143, 145, 146, 148, 150, 152, 154, 156, 159, 161,
        163, 165, 166, 169, 172, 175, 178, 181, 184, 187, 191, 194, 197, 200, 203, 205,
        208, 212, 215, 218, 221, 225, 229, 233, 237, 241, 245, 248, 250, 252, 254, 256,
        258, 259, 261, 262, 264, 266, 267, 269, 270, 272, 274, 276, 277, 279, 281, 283,
        287, 290, 293, 296, 300, 303, 306, 309, 312, 315, 318, 321, 324, 326, 328, 330,
        332, 334, 336, 339, 342, 345, 348, 350, 353, 356, 359, 362, 365, 368, 371, 374,
        377, 379, 382, 384, 387, 391, 395, 398, 401, 403, 406, 410, 414, 417, 420, 422,
        425, 428, 432, 436, 440, 444, 448, 452, 457, 461, 465, 468, 472, 476, 479, 482,
        486, 490, 494, 497, 500, 504, 507, 510, 513, 515, 517, 519, 521, 523, 525, 527,
        529, 532, 535, 538, 541, 544, 547, 550, 553, 555, 557, 560, 563, 566, 569, 572,
        575, 578, 581, 584, 587, 590, 593, 596, 599, 602, 605, 608, 611, 614, 617, 621);
    signal s_grey_a : unsigned(7 downto 0) := (others => '0');
    signal s_grey_q : unsigned(9 downto 0) := to_unsigned(77, 10);
    signal s_grey_r : unsigned(9 downto 0) := to_unsigned(77, 10);

    -- per-row hash ROM (row index -> size factor bits 2..0, scroll rate bits 4..3)
    type t_rh is array (0 to 255) of integer range 0 to 255;
    constant C_RH : t_rh := (
        94, 250, 148, 70, 217, 190, 93, 4, 238, 48, 105, 252, 109, 190, 82, 163,
        154, 189, 216, 204, 111, 214, 44, 96, 94, 129, 232, 68, 152, 162, 182, 91,
        236, 71, 96, 170, 29, 74, 121, 30, 199, 125, 155, 77, 230, 201, 251, 13,
        230, 53, 127, 0, 23, 101, 59, 25, 206, 66, 221, 56, 99, 152, 246, 212,
        184, 218, 109, 128, 105, 152, 83, 170, 19, 189, 104, 191, 234, 182, 217, 215,
        104, 254, 27, 207, 242, 151, 114, 9, 96, 177, 199, 84, 198, 221, 179, 162,
        189, 114, 124, 115, 252, 199, 228, 238, 253, 241, 148, 48, 163, 20, 65, 127,
        232, 204, 18, 179, 17, 0, 223, 5, 175, 93, 75, 188, 42, 12, 147, 177,
        16, 227, 79, 61, 70, 74, 56, 213, 111, 97, 70, 88, 251, 2, 75, 230,
        133, 200, 225, 246, 111, 14, 130, 17, 91, 134, 111, 53, 87, 250, 203, 197,
        113, 199, 46, 195, 215, 14, 2, 127, 162, 136, 252, 169, 94, 15, 114, 99,
        71, 240, 83, 1, 48, 126, 20, 191, 17, 77, 222, 164, 10, 216, 130, 188,
        67, 24, 39, 8, 107, 164, 111, 102, 146, 118, 29, 182, 188, 226, 0, 117,
        17, 167, 197, 11, 92, 111, 244, 175, 228, 65, 168, 251, 95, 202, 102, 157,
        225, 192, 154, 110, 216, 106, 207, 177, 111, 160, 47, 76, 51, 185, 243, 161,
        28, 121, 202, 128, 101, 255, 197, 75, 191, 94, 132, 45, 49, 254, 143, 49);
    signal s_rha : unsigned(7 downto 0) := (others => '0');
    signal s_rh, s_rh2 : unsigned(7 downto 0) := (others => '0');

    -- cell RAM: the held sample of every half cell
    type t_cram is array (0 to 511) of std_logic_vector(15 downto 0);
    signal cram : t_cram := (others => x"004D");
    attribute no_rw_check : boolean;
    attribute no_rw_check of cram : signal is true;
    signal s_cwe : std_logic := '0';
    signal s_cwa, s_cra : unsigned(8 downto 0) := (others => '0');
    signal s_cwd : std_logic_vector(15 downto 0) := x"004D";
    signal s_crq : std_logic_vector(15 downto 0) := x"004D";
    signal s_crq2 : unsigned(9 downto 0) := to_unsigned(77, 10);

    -- timing / zone / width
    signal s_tid : std_logic_vector(3 downto 0) := (others => '1');
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(6 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- glided knobs (10.3) and their integer parts
    signal g1, g2, g3, g5, g6, g12 : unsigned(12 downto 0) := (others => '0');
    signal k1v, k2v, k3v, k5v, k6v, p12v : unsigned(9 downto 0) := (others => '0');
    signal s_sp : signed(11 downto 0) := (others => '0');
    signal s_magenta, s_mixed, s_hard, s_mirror, s_src : std_logic := '0';
    signal s_ab   : unsigned(15 downto 0) := C_AB;                         -- A/B split (K5)
    signal s_abd  : signed(15 downto 0) := (others => '0');
    signal s_kwx  : unsigned(10 downto 0) := to_unsigned(174, 11);         -- key window width (K3)
    signal s_T    : signed(23 downto 0) := (others => '0');                -- scroll time, Q4 virtual px
    signal s_rb   : unsigned(7 downto 0) := (others => '0');               -- row index at the top
    signal s_warm : std_logic := '0';

    -- per-field derived
    signal s_incv : unsigned(13 downto 0) := to_unsigned(1427, 14);
    signal s_ginc : unsigned(10 downto 0) := to_unsigned(851, 11);
    signal s_sq   : unsigned(19 downto 0) := (others => '0');
    signal s_ysc  : unsigned(12 downto 0) := to_unsigned(971, 13);
    signal s_yth  : unsigned(11 downto 0) := to_unsigned(786, 12);
    signal s_rq8  : unsigned(16 downto 0) := to_unsigned(2048, 17);
    signal s_d1, s_d2 : unsigned(19 downto 0) := (others => '0');
    signal s_d2a, s_d2b : unsigned(20 downto 0) := (others => '0');
    signal s_tha1, s_tha2, s_thb1, s_thb2 : unsigned(12 downto 0) := (others => '0');
    signal s_kca, s_kcb : unsigned(10 downto 0) := (others => '0');
    signal s_dxra : unsigned(6 downto 0) := (others => '0');
    signal s_m10  : unsigned(9 downto 0) := (others => '0');
    signal s_dth  : unsigned(15 downto 0) := (others => '0');
    signal s_tha, s_thb : unsigned(12 downto 0) := (others => '0');
    signal s_ths  : signed(13 downto 0) := (others => '0');
    signal s_kc, s_kw : unsigned(10 downto 0) := (others => '0');
    signal s_gb8  : unsigned(7 downto 0) := (others => '0');
    signal s_crush : unsigned(9 downto 0) := to_unsigned(77, 10);
    signal s_xl0, s_xr0, s_xrmin : signed(15 downto 0) := (others => '0');   -- Q4, from the centre
    signal s_dxl, s_dxr : unsigned(6 downto 0) := (others => '0');
    signal s_yb, s_ylim : unsigned(10 downto 0) := (others => '0');
    signal s_dx1, s_dx2 : signed(12 downto 0) := (others => '0');
    signal s_won : std_logic := '0';
    signal s_xst : signed(12 downto 0) := (others => '0');                  -- beam start x, Q1, from the centre
    signal s_wc : unsigned(10 downto 0) := to_unsigned(943, 11);            -- grid anchor x (real px)
    -- per-field phases
    signal s_p10 : unsigned(19 downto 0) := C_P1;
    signal s_p20 : unsigned(19 downto 0) := C_P2;
    signal s_ph0, s_gd, s_boil : unsigned(15 downto 0) := (others => '0');
    -- serial multiplier (shared: the zone slots in vblank, the per-row engine on active lines)
    signal s_ma : unsigned(30 downto 0) := (others => '0');
    signal s_mb : unsigned(16 downto 0) := (others => '0');
    signal s_macc : unsigned(30 downto 0) := (others => '0');


    -- per-line engine
    signal s_ls : unsigned(2 downto 0) := (others => '0');
    signal s_first : std_logic := '1';
    signal s_p1, s_p2 : unsigned(19 downto 0) := (others => '0');
    signal s_c1, s_c2 : std_logic := '0';
    signal s_phh, s_pa, s_pb : unsigned(15 downto 0) := (others => '0');
    signal s_th : signed(21 downto 0) := (others => '0');                   -- Q8
    signal s_yv8 : unsigned(19 downto 0) := (others => '0');
    signal s_xl, s_xr : signed(15 downto 0) := (others => '0');
    signal s_trih, s_trib : signed(12 downto 0) := (others => '0');
    signal s_xrc : signed(15 downto 0) := (others => '0');
    signal s_yv : unsigned(10 downto 0) := (others => '0');
    signal s_gate : std_logic := '0';
    signal s_tb, s_tba, s_tbb : signed(15 downto 0) := (others => '0');
    signal s_gl1 : unsigned(15 downto 0) := (others => '0');
    signal s_d3n : signed(12 downto 0) := (others => '0');
    signal s_ylok : std_logic := '0';
    signal s_ybln : unsigned(10 downto 0) := (others => '0');
    signal s_tickn, s_rparn : std_logic := '0';
    signal s_gln : unsigned(15 downto 0) := (others => '0');
    -- per-row engine
    signal s_rowc : unsigned(7 downto 0) := (others => '0');
    signal s_rq : unsigned(5 downto 0) := (others => '0');
    signal s_o : signed(25 downto 0) := (others => '0');                    -- row offset, Q4 virtual px
    signal s_ivr : unsigned(12 downto 0) := to_unsigned(1427, 13);          -- row cell clock (virtual)
    signal s_s0 : unsigned(15 downto 0) := (others => '0');
    signal r_inc : unsigned(14 downto 0) := to_unsigned(1427, 15);
    signal r_phi, r_gof : unsigned(15 downto 0) := (others => '0');
    signal r_eb : unsigned(7 downto 0) := (others => '0');
    -- committed line values
    signal l_tick, l_even, l_ylok : std_logic := '0';
    signal l_gl : unsigned(15 downto 0) := (others => '0');
    signal l_ybl : unsigned(10 downto 0) := (others => '0');
    signal l_d3 : signed(12 downto 0) := (others => '0');
    signal l_xl, l_xr : signed(12 downto 0) := (others => '0');


    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- beam (stage 0)
    signal b_ph : unsigned(15 downto 0) := (others => '0');
    signal b_x  : signed(12 downto 0) := (others => '0');
    signal b_g  : unsigned(15 downto 0) := (others => '0');
    signal b_first : std_logic := '0';

    -- video sample path
    signal v_y1 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal v_d2 : unsigned(9 downto 0) := (others => '0');
    signal v_a3, v_b3 : unsigned(10 downto 0) := (others => '0');
    signal v_l4, v_l5 : unsigned(9 downto 0) := (others => '0');
    signal v_p5, v_p6, v_p7 : unsigned(10 downto 0) := (others => '0');
    signal v_q6 : unsigned(11 downto 0) := (others => '0');
    signal v_vid : unsigned(9 downto 0) := to_unsigned(77, 10);

    -- pixel pipeline (pNN = stage-NN register)
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    type t_s14 is array (natural range <>) of signed(13 downto 0);
    type t_u9 is array (natural range <>) of unsigned(8 downto 0);
    signal p_a, p_st : t_sl(1 to 18) := (others => '0');                   -- A half / segment start
    signal p_e : t_u9(1 to 18) := (others => (others => '0'));            -- half-cell entry (A even, B odd)
    signal p_k2 : unsigned(7 downto 0) := (others => '0');                 -- hash entry key (entry + row base)
    signal p_rt : t_sl(2 to 7) := (others => '0');                         -- right of the screen centre
    signal s_ec : unsigned(8 downto 0) := (others => '0');                 -- entry at the centre
    signal p_ma8, p_ma9, p_ma10 : unsigned(8 downto 0) := (others => '0'); -- mirrored entry
    signal p_mf : t_sl(8 to 13) := (others => '0');                        -- mirrored pixel
    signal p_gt : t_sl(1 to 7) := (others => '0');                         -- osc highlight gate
    signal p_x1 : signed(13 downto 0) := (others => '0');                  -- virtual px from the centre
    signal p_dl2, p_dr2, p_ua2, p_ub2 : signed(13 downto 0) := (others => '0');
    signal p_j2 : unsigned(9 downto 0) := (others => '0');
    signal p_m : t_s14(3 to 7) := (others => (others => '0'));            -- wedge margin
    signal p_dash : t_sl(3 to 10) := (others => '0');
    signal h3, h4, h5, h6, h7 : unsigned(23 downto 0) := (others => '0');
    signal p_br, p_in, p_lite : t_sl(8 to 10) := (others => '0');
    signal p_lat8, p_lat9, p_lat10 : unsigned(9 downto 0) := (others => '0');
    signal p_vo11 : unsigned(9 downto 0) := to_unsigned(77, 10);
    -- K6 camera stages: 12 = source x gain (Q8), 13 = + pivot term, clamped (the Osc wedge passes)
    signal p_wk11, p_wk12 : std_logic := '0';
    signal p_vo12 : unsigned(9 downto 0) := to_unsigned(77, 10);
    signal p_gm12 : unsigned(18 downto 0) := (others => '0');
    signal p_vb13 : unsigned(9 downto 0) := to_unsigned(77, 10);
    signal s_cg   : unsigned(8 downto 0) := to_unsigned(256, 9);        -- camera gain, Q8 (256 = v0.6.0)
    signal s_cpa  : signed(19 downto 0) := (others => '0');             -- C_PIV * cg (Q8)
    signal s_cpv  : signed(19 downto 0) := to_signed(0, 20);            -- C_PIV * (256 - cg), Q8
    signal s_sdeep : std_logic := '0';                -- shadow (S9)
    signal s_hold : unsigned(9 downto 0) := to_unsigned(77, 10);
    signal p_val12 : unsigned(9 downto 0) := to_unsigned(77, 10);
    signal p_dv13 : unsigned(9 downto 0) := (others => '0');
    signal p_v13, p_v14, p_v15 : unsigned(9 downto 0) := to_unsigned(77, 10);
    signal p_cy13, p_wh13, p_cy14, p_wh14, p_cy15, p_wh15 : std_logic := '0';
    signal p_pl14, p_ph14 : unsigned(13 downto 0) := (others => '0');
    signal p_pb15 : unsigned(17 downto 0) := (others => '0');
    signal p_lo16 : unsigned(9 downto 0) := to_unsigned(77, 10);
    signal p_c16, p_c17, p_c18 : std_logic := '0';
    signal p_t17 : unsigned(10 downto 0) := (others => '0');
    signal p_s17 : unsigned(2 downto 0) := (others => '0');
    signal p_y18 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ysr0 : unsigned(9 downto 0) := to_unsigned(64, 10);             -- stage 19 (box input)
    -- block-RAM delay rings (one write address, every clock; no_rw_check: reads trail the write)
    type t_ring16 is array (0 to 255) of std_logic_vector(15 downto 0);
    type t_ring32 is array (0 to 255) of std_logic_vector(31 downto 0);
    signal ry16 : t_ring16 := (others => x"0040");                           -- luma box input, read 16 back
    signal ry9  : t_ring16 := (others => x"0040");                           -- luma box input, read 9 back
    signal rc8  : t_ring32 := (others => x"00080200");                       -- u27 / v27, read 8 back
    attribute no_rw_check of ry16 : signal is true;
    attribute no_rw_check of ry9 : signal is true;
    attribute no_rw_check of rc8 : signal is true;
    signal rg_wa, rg_a16, rg_a9, rg_a8 : unsigned(7 downto 0) := (others => '0');
    signal rg_q16, rg_q9 : std_logic_vector(15 downto 0) := x"0040";
    signal rg_q8 : std_logic_vector(31 downto 0) := x"00080200";
    signal s_y16d, s_y9d : unsigned(9 downto 0) := to_unsigned(64, 10);      -- = old s_ysr(16), s_ysr(9)
    signal s_u8d, s_v8d : unsigned(9 downto 0) := to_unsigned(512, 10);      -- = old s_csu(7), s_csv(7)
    signal s_csr : std_logic_vector(0 to 12) := (others => '0');
    signal s_ysum : unsigned(13 downto 0) := to_unsigned(1024, 14);         -- stage 20
    signal s_csum : unsigned(3 downto 0) := (others => '0');
    signal s_y19h : std_logic_vector(15 downto 0) := (others => '0');      -- generated avid of the last 16 s_ysr(0) / s_csr(0)
    signal s_ys21 : t_u10(0 to 8) := (others => to_unsigned(64, 10));
    signal s_c21 : t_u8(0 to 4) := (others => (others => '0'));
    signal p_dy22 : signed(10 downto 0) := (others => '0');
    signal p_yc22, p_y23, p_y24, p_y25, p_y26 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p_c22, p_c23, p_c24, p_c25 : unsigned(3 downto 0) := (others => '0');
    signal p_fr23 : signed(10 downto 0) := (others => '0');
    signal p_d23 : unsigned(9 downto 0) := (others => '0');
    signal p_u24, p_v24, p_u25, p_v25, p_u26, p_v26 : signed(11 downto 0) := to_signed(512, 12);
    signal p_du25, p_dv25 : signed(11 downto 0) := (others => '0');
    signal p_pu26, p_pv26 : signed(16 downto 0) := (others => '0');
    signal s_y27, s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_bxh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 box inputs
    signal s_y28, s_y29, s_y30, s_y31, s_y32, s_y33x, s_yf33 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u33, s_v33 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y34 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u34, s_v34 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_cram : process(clk)
    begin
        if rising_edge(clk) then
            s_crq <= cram(to_integer(s_cra));
            if s_cwe = '1' then cram(to_integer(s_cwa)) <= s_cwd; end if;
        end if;
    end process p_cram;

    -- delay rings: written every clock at rg_wa with the stage-19 luma (box input) and the stage-27
    -- chroma; read data registered in the EBR, then re-registered in fabric.  Read address = write
    -- address - (delay - 3): data written in cycle n is back in fabric in cycle n + delay.
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            rg_wa <= rg_wa + 1;
            rg_a16 <= rg_wa - 13; rg_a9 <= rg_wa - 6; rg_a8 <= rg_wa - 5;
            ry16(to_integer(rg_wa)) <= "000000" & std_logic_vector(s_ysr0);
            ry9(to_integer(rg_wa))  <= "000000" & std_logic_vector(s_ysr0);
            rc8(to_integer(rg_wa))  <= "000000000000" & std_logic_vector(s_u27) & std_logic_vector(s_v27);
            rg_q16 <= ry16(to_integer(rg_a16));
            rg_q9  <= ry9(to_integer(rg_a9));
            rg_q8  <= rc8(to_integer(rg_a8));
            s_y16d <= unsigned(rg_q16(9 downto 0));
            s_y9d  <= unsigned(rg_q9(9 downto 0));
            s_u8d  <= unsigned(rg_q8(19 downto 10));
            s_v8d  <= unsigned(rg_q8(9 downto 0));
        end if;
    end process p_ring;

    p_grom : process(clk)
    begin
        if rising_edge(clk) then
            s_grey_q <= to_unsigned(C_GREY(to_integer(s_grey_a)), 10);
        end if;
    end process p_grom;

    p_rhrom : process(clk)
    begin
        if rising_edge(clk) then
            s_rh <= to_unsigned(C_RH(to_integer(s_rha)), 8);
        end if;
    end process p_rhrom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, per-field constants and phases
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_t : unsigned(14 downto 0);
        variable v_kc : unsigned(10 downto 0);
        variable v_e : signed(11 downto 0);
        variable v_p : signed(21 downto 0);
        procedure glide(signal g : inout unsigned(12 downto 0); k : std_logic_vector(9 downto 0); w : std_logic) is
            variable d : signed(14 downto 0);
            variable n : signed(14 downto 0);
        begin
            d := signed(resize(unsigned(k) & "000", 15)) - signed(resize(g, 15));
            if w = '0' then
                g <= unsigned(k) & "000";
            elsif d > 31 or d < -31 then
                n := signed(resize(g, 15)) + shift_right(d, 3);
                g <= unsigned(n(12 downto 0));
            end if;
        end procedure;
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_xs <= f_vstep_x(s_tid);
            s_ys <= f_vstep_y(s_tid);

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
            elsif s_zc /= 127 then
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

            if s_zgo = '1' then
                s_warm <= '1';
                glide(g1, registers_in(0), s_warm);
                glide(g2, registers_in(1), s_warm);
                glide(g3, registers_in(2), s_warm);
                glide(g5, registers_in(4), s_warm);
                glide(g6, registers_in(5), s_warm);
                glide(g12, registers_in(7), s_warm);
                -- K4 Speed: deadband +-8; past +-128 (the default speed) the slope triples and the
                -- excess drives the ticker scroll (zero inside, so the default picture is the photo's)
                v_s := signed(resize(unsigned(registers_in(3)), 11)) - 512;
                if v_s > 128 or v_s < -128 then
                    if v_s(10) = '0' then v_e := resize(v_s, 12) - 128; else v_e := resize(v_s, 12) + 128; end if;
                else
                    v_e := (others => '0');
                end if;
                if v_s > -8 and v_s < 8 then
                    s_sp <= (others => '0');
                else
                    s_sp <= resize(v_s, 12) + shift_left(v_e, 1);
                end if;
                if v_e = 0 then
                    s_T <= (others => '0');                                          -- photo zone: rows realign
                else
                    s_T <= s_T + resize(shift_right(v_e, 2), 24) + resize(shift_right(v_e, 4), 24);   -- 5/16 per field
                end if;
                s_magenta <= registers_in(6)(0);
                s_mixed   <= registers_in(6)(1);
                s_hard    <= '0';                    -- K6 camera pass: Edges fixed at Soft
                s_sdeep <= registers_in(6)(2);    -- S9 Shadow: Photo (old K6 512) / Deep (old K6 800)
                s_mirror  <= registers_in(6)(3);
                s_src     <= registers_in(6)(4);
                if s_wmax /= 0 then s_w <= s_wmax; end if;
                s_wmax <= (others => '0');
                -- motion: clocks roll, hold sine drifts, highlights hop, values boil; the row index
                -- base follows clock 1's wraps so per-row sizes / rates travel with their rows
                v_p := signed(resize(s_p10, 22)) + resize(shift_left(resize(s_sp, 22), 6) - shift_left(resize(s_sp, 22), 1), 22);
                s_p10 <= unsigned(v_p(19 downto 0));
                if v_p < 0 then s_rb <= s_rb - 1; elsif v_p(21 downto 20) /= "00" then s_rb <= s_rb + 1; end if;
                s_p20 <= unsigned(signed(s_p20) + resize(shift_left(resize(s_sp, 20), 5) + shift_left(resize(s_sp, 20), 3) + shift_left(resize(s_sp, 20), 2), 20));
                s_ph0 <= unsigned(signed(s_ph0) + resize(shift_right(resize(s_sp, 16) + shift_left(resize(s_sp, 16), 1), 3), 16));
                s_gd  <= unsigned(signed(s_gd) + shift_left(resize(s_sp, 16), 2) + shift_left(resize(s_sp, 16), 1));
                s_boil <= unsigned(signed(s_boil) + resize(shift_right(s_sp, 2), 16));
            end if;
            if s_warm = '0' then
                s_ph0 <= C_PH;
            end if;

            -- integer knob values; derived per-field constants (quasi-static: every sum is
            -- at most two adders deep per register, the chains settle within a few clocks)
            k1v <= g1(12 downto 3); k2v <= g2(12 downto 3); k3v <= g3(12 downto 3);
            k5v <= g5(12 downto 3); k6v <= g6(12 downto 3); p12v <= g12(12 downto 3);
            case to_integer(s_xs) is
                when 4      => s_ginc <= to_unsigned(1702, 11);
                when 3      => s_ginc <= to_unsigned(1276, 11);
                when others => s_ginc <= to_unsigned(851, 11);
            end case;
            s_rq8 <= to_unsigned(2048, 17) + resize(s_sq(19 downto 4), 17);
            s_ysc <= f_ysc(s_tid); s_yth <= f_yth(s_tid);
            -- d2 = d1 * 217 / 256 = d1 - d1/8 - d1/32 + d1/256
            s_d2a <= resize(s_d1, 21) - resize(s_d1(19 downto 3), 21);
            s_d2b <= resize(s_d1(19 downto 8), 21) - resize(s_d1(19 downto 5), 21);
            s_d2 <= resize(s_d2a + s_d2b, 20);
            if p12v(9) = '1' then s_m10 <= (others => '1'); else s_m10 <= p12v(8 downto 0) & '0'; end if;
            -- hold ramp start: 3277 - p12 * 4.60 + m10 * 1.83
            s_tha1 <= resize(shift_left(resize(p12v, 13), 2), 13) + resize(p12v(9 downto 1), 13);
            s_tha2 <= resize(p12v(9 downto 4), 13) + resize(p12v(9 downto 5), 13) + resize(p12v(9 downto 7), 13);
            s_tha <= s_tha1 + s_tha2;
            s_thb1 <= resize(s_m10, 13) + resize(s_m10(9 downto 1), 13);
            s_thb2 <= resize(s_m10(9 downto 2), 13) + resize(s_m10(9 downto 4), 13) + resize(s_m10(9 downto 6), 13);
            s_thb <= s_thb1 + s_thb2;
            s_ths <= to_signed(3277, 14) - signed(resize(s_tha, 14)) + signed(resize(s_thb, 14));
            -- key window: the photo's law at and above 50 %; below it the window drops one for one
            -- and widens by half the distance (k3 = 0: 0.13 .. 0.55, cyan floods the greys)
            s_kca <= to_unsigned(307, 11) + resize(k3v(9 downto 1), 11);
            s_kcb <= resize(k3v(9 downto 3), 11) + resize(k3v(9 downto 5), 11);
            v_kc := s_kca + s_kcb;
            if k3v(9) = '0' then
                s_kc <= resize(k3v, 11) + 131;
                s_kwx <= to_unsigned(174, 11) + resize(not k3v(8 downto 1), 11);
            else
                if v_kc > 870 then s_kc <= to_unsigned(870, 11); else s_kc <= v_kc; end if;
                s_kwx <= to_unsigned(174, 11);
            end if;
            s_kw <= s_kc + s_kwx;
            v_t := resize(k5v & "00", 15) + resize(k5v & '0', 15) + resize(k5v, 15);   -- 7 k5
            s_gb8 <= v_t(12 downto 5);
            -- A/B duty: split 34996 + 36 (k5 - 512): A = 25 % .. 53 % (the photo) .. 81 % of a cell
            s_abd <= shift_left(signed(resize(k5v, 16)) - 512, 5) + shift_left(signed(resize(k5v, 16)) - 512, 2);
            s_ab <= unsigned(signed(C_AB) + s_abd);
            -- Video dark crush: the photo's law to 50 %, then +0.56 per step more (0.51 luma at max)
            -- (the steeper law above 512: + (k - 512) / 2 + (k - 512) / 16)
            s_crush <= f_su(s_sdeep, 77 + KD / 8 + KD / 32 + (KD - 512) / 2 + (KD - 512) / 16,
                                     77 + KP / 8 + KP / 32 + (KP - 512) / 2 + (KP - 512) / 16, 10);
            -- wedge (virtual px from the screen centre; edges Q4)
            s_xl0 <= f_ss(s_sdeep, 80 - 18 * KD, 80 - 18 * KP, 16);
            s_xr0 <= f_ss(s_sdeep, 80 + 18 * KD + KD / 2, 80 + 18 * KP + KP / 2, 16);
            s_xrmin <= f_ss(s_sdeep, 80 - 8 * KD + KD, 80 - 8 * KP + KP, 16);
            s_dxl <= resize(s_ys(11 downto 6), 7);
            s_dxra <= resize(s_ys(11 downto 4), 7) + resize(s_ys(11 downto 5), 7);
            s_dxr <= s_dxra + resize(s_ys(11 downto 7), 7) + resize(s_ys(11 downto 9), 7);
            s_yb <= f_su(s_sdeep, KD + KD / 16 + KD / 64, KP + KP / 16 + KP / 64, 11);
            s_ylim <= f_su(s_sdeep, KD - KD / 4 + KD / 16, KP - KP / 4 + KP / 16, 11);
            s_dx1 <= f_ss(s_sdeep, 5 - KD / 2 + KD / 16 + KD / 64, 5 - KP / 2 + KP / 16 + KP / 64, 13);
            s_dx2 <= f_ss(s_sdeep, 5 + KD / 4 + KD / 16, 5 + KP / 4 + KP / 16, 13);
            s_won <= '1';                        -- the wedge is never off (both Shadow settings draw it)
            -- K6 camera: gain Q8 = k / 2 (0 .. 2x, 512 = 1x = v0.6.0), pivot term C_PIV (256 - cg)
            s_cg  <= k6v(9 downto 1);
            s_cpa <= signed(resize(shift_left(resize(s_cg, 20), 8) + shift_left(resize(s_cg, 20), 6), 20));   -- 320 cg
            s_cpv <= to_signed(C_PIV * 256, 20) - s_cpa;
            g_wm1 <= s_w - 1;
            s_wc <= s_w(11 downto 1) - 17;        -- grid anchor: the centre, less the boxes' 17-px pipeline lag
            case to_integer(s_xs) is
                when 4      => s_xst <= -signed(resize(s_w & '0', 13));                                 -- -w*xs/2, Q1
                when 3      => s_xst <= -signed(resize(s_w, 13)) - signed(resize(s_w(11 downto 1), 13));
                when others => s_xst <= -signed(resize(s_w, 13));
            end case;

            -- serial product results (the multiplier and its loads live in p_line)
            if s_zc = 19 then
                s_sq <= s_macc(19 downto 0);
            elsif s_zc = 39 then
                if s_macc(30 downto 28) /= "000" then s_d1 <= (others => '1'); else s_d1 <= s_macc(27 downto 8); end if;
            elsif s_zc = 57 then
                s_dth <= s_macc(25 downto 10);
            elsif s_zc = 75 then
                case k1v(9 downto 8) is
                    when "11"   => s_incv <= s_macc(20 downto 7);
                    when "10"   => s_incv <= s_macc(21 downto 8);
                    when "01"   => s_incv <= s_macc(22 downto 9);
                    when others => s_incv <= s_macc(23 downto 10);
                end case;
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start, field-latched width)
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
    -- Per-line engine: computes the next line at the source avid fall (line 0
    -- in the zone), commits at the source avid rise.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_n1, v_n2 : unsigned(20 downto 0);
        variable v_m8 : unsigned(4 downto 0);
        variable v_iv : unsigned(13 downto 0);
        variable v_S : unsigned(23 downto 0);
        variable v_ld : std_logic;
        variable v_a : unsigned(30 downto 0);
        variable v_b : unsigned(16 downto 0);
        variable v_oa : signed(25 downto 0);
    begin
        if rising_edge(clk) then
            s_rha <= s_rowc;
            s_rh2 <= s_rh;
            if s_zc = 80 then
                -- line 0 of the field
                s_rowc <= s_rb;
                s_p1 <= s_p10; s_p2 <= s_p20; s_c1 <= '0'; s_c2 <= '0';
                s_phh <= s_ph0;
                s_th <= shift_left(resize(s_ths, 22), 8);
                s_yv8 <= (others => '0'); s_pa <= (others => '0'); s_pb <= (others => '0');
                s_xl <= s_xl0; s_xr <= s_xr0;
                s_first <= '1'; s_rparn <= '0';
                s_ls <= to_unsigned(2, 3);
            elsif data_in.avid = '0' and s_avq = '1' then
                -- advance to the next line
                v_n1 := resize(s_p1, 21) + s_d1; s_p1 <= v_n1(19 downto 0); s_c1 <= v_n1(20);
                if v_n1(20) = '1' then s_rowc <= s_rowc + 1; end if;   -- every clock-1 wrap, ticked or held
                v_n2 := resize(s_p2, 21) + s_d2; s_p2 <= v_n2(19 downto 0); s_c2 <= v_n2(20);
                s_phh <= s_phh + f_dh(s_tid);
                s_th <= s_th - signed(resize(s_dth, 22));
                s_yv8 <= s_yv8 + s_ys;
                s_pa <= s_pa + f_da(s_tid); s_pb <= s_pb + f_db(s_tid);
                s_xl <= s_xl + signed(resize(s_dxl, 16)); s_xr <= s_xr - signed(resize(s_dxr, 16));
                s_ls <= to_unsigned(2, 3);
            elsif s_ls = 2 then
                s_trih <= f_tri12(s_phh);
                s_trib <= f_tri12(s_pb + 16384);
                if s_xr < s_xrmin then s_xrc <= s_xrmin; else s_xrc <= s_xr; end if;
                s_yv <= s_yv8(18 downto 8);
                s_ls <= to_unsigned(3, 3);
            elsif s_ls = 3 then
                if resize(s_trih, 22) > shift_right(s_th, 8) then s_gate <= '1'; else s_gate <= '0'; end if;
                s_tba <= shift_left(resize(s_trib, 16), 2) + shift_left(resize(s_trib, 16), 1);
                s_tbb <= shift_right(resize(s_trib, 16), 1) + shift_right(resize(s_trib, 16), 2);
                s_gl1 <= s_pa + s_gd;
                s_d3n <= signed(resize(s_yb, 13)) - signed(resize(s_yv, 13));
                if s_yv < s_ylim then s_ylok <= '1'; else s_ylok <= '0'; end if;
                s_ybln <= s_yv + s_boil(15 downto 8);
                s_ls <= to_unsigned(4, 3);
            elsif s_ls = 4 then
                if s_first = '1' then
                    s_tickn <= '1';
                elsif (s_c1 = '1' or s_c2 = '1') and s_gate = '0' then
                    s_tickn <= '1';
                else
                    s_tickn <= '0';
                end if;
                s_tb <= s_tba + s_tbb;
                s_ls <= to_unsigned(5, 3);
            elsif s_ls = 5 then
                s_gln <= s_gl1 + unsigned(s_tb);
                if s_tickn = '1' and s_first = '0' then s_rparn <= not s_rparn; end if;
                s_first <= '0';
                s_ls <= (others => '0');
                if s_tickn = '1' then s_rq <= to_unsigned(1, 6); end if;   -- a new row: run the row engine
            end if;

            -- per-row engine (a tick line only; ~58 clocks after the avid fall, inside SD's 138-clock
            -- hblank, and after the beam has finished the line): the row's cell clock (Mixed size factor),
            -- its centre-anchored start phase, its scroll offset (rate x T) as a grid phase, a hash-key base
            -- and a highlight-gate offset.  The row registers r_* feed the beam directly; they change only
            -- in hblank.
            v_ld := '0'; v_a := (others => '0'); v_b := (others => '0');
            if s_rq /= 0 then
                s_rq <= s_rq + 1;
                case to_integer(s_rq) is
                    when 1 =>
                        -- size factor x8: 4 5 6 8 10 12 14 16 (x0.5 .. x2), 8 when Even
                        if s_mixed = '0' then
                            v_m8 := to_unsigned(8, 5);
                        else
                            case to_integer(s_rh2(2 downto 0)) is
                                when 0 => v_m8 := to_unsigned(4, 5);  when 1 => v_m8 := to_unsigned(5, 5);
                                when 2 => v_m8 := to_unsigned(6, 5);  when 3 => v_m8 := to_unsigned(8, 5);
                                when 4 => v_m8 := to_unsigned(10, 5); when 5 => v_m8 := to_unsigned(12, 5);
                                when 6 => v_m8 := to_unsigned(14, 5); when others => v_m8 := to_unsigned(16, 5);
                            end case;
                        end if;
                        v_ld := '1'; v_a := resize(s_incv, 31); v_b := resize(v_m8, 17);
                        s_o <= resize(s_T, 26);
                    when 2 | 3 =>
                        -- scroll rate 1..4 (bits 4..3; K4 sets the direction): o = T + b3 T + b4 2T (one adder, two passes)
                        v_oa := (others => '0');
                        if s_rq(0) = '0' and s_rh2(3) = '1' then v_oa := resize(s_T, 26);
                        elsif s_rq(0) = '1' and s_rh2(4) = '1' then v_oa := shift_left(resize(s_T, 26), 1); end if;
                        s_o <= s_o + v_oa;
                    when 8 =>
                        v_iv := s_macc(16 downto 3);
                        if v_iv > 8191 then s_ivr <= to_unsigned(8191, 13); else s_ivr <= v_iv(12 downto 0); end if;
                    when 9 =>
                        case to_integer(s_xs) is
                            when 4      => r_inc <= resize(s_ivr & '0', 15);
                            when 3      => r_inc <= resize(s_ivr, 15) + resize(s_ivr(12 downto 1), 15);
                            when others => r_inc <= resize(s_ivr, 15);
                        end case;
                    when 10 =>
                        v_ld := '1'; v_a := resize(r_inc, 31); v_b := resize(s_wc, 17);
                    when 24 =>
                        s_s0 <= C_PHC - s_macc(15 downto 0);
                        v_ld := '1'; v_a := unsigned(resize(s_o, 31)); v_b := resize(s_ivr, 17);
                    when 40 =>
                        v_S := resize(s_s0, 24) - s_macc(27 downto 4);
                        r_phi <= v_S(15 downto 0);
                        r_eb <= v_S(22 downto 16) & '0';
                        v_ld := '1'; v_a := unsigned(resize(s_o, 31)); v_b := to_unsigned(851, 17);
                    when 53 =>
                        r_gof <= s_macc(19 downto 4);
                        s_rq <= (others => '0');
                    when others => null;
                end case;
            end if;

            -- shared serial shift-add multiplier: the zone slots (k2^2, rows x ysc, m10 x yth and the K1
            -- exponential 1427 (1 + frac) 2^oct) in vblank, the row engine on active lines; it steps every
            -- clock until its multiplier operand is used up
            if s_zc = 4 then
                s_ma <= resize(k2v, 31); s_mb <= resize(k2v, 17); s_macc <= (others => '0');
            elsif s_zc = 22 then
                s_ma <= resize(s_rq8, 31); s_mb <= resize(s_ysc, 17); s_macc <= (others => '0');
            elsif s_zc = 42 then
                s_ma <= resize(s_yth, 31); s_mb <= resize(s_m10, 17); s_macc <= (others => '0');
            elsif s_zc = 58 then
                s_ma <= to_unsigned(1427, 31); s_mb <= resize('1' & k1v(7 downto 0), 17); s_macc <= (others => '0');
            elsif v_ld = '1' then
                s_ma <= v_a; s_mb <= v_b; s_macc <= (others => '0');
            else
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(29 downto 0) & '0';
                s_mb <= '0' & s_mb(16 downto 1);
            end if;

            if data_in.avid = '1' and s_avq = '0' then
                l_tick <= s_tickn;
                l_even <= not s_rparn;
                l_ybl <= s_ybln;
                l_d3 <= s_d3n;
                l_ylok <= s_ylok;
                l_xl <= resize(shift_right(s_xl, 4), 13);
                l_xr <= resize(shift_right(s_xrc, 4), 13);
                l_gl <= s_gln - r_gof;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 0): cell phase, virtual x (Q1) from the centre, highlight phase
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            b_first <= '0';
            if g_start = '1' then
                b_ph <= r_phi;
                b_x <= s_xst;
                b_g <= l_gl;
                b_first <= '1';
            elsif g_avid = '1' then
                b_ph <= b_ph + resize(r_inc, 16);
                b_x <= b_x + signed(resize(s_xs, 13));
                b_g <= b_g + s_ginc;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Video sample path (the camera luma in L units, 4-px average, crushed)
    ---------------------------------------------------------------------------
    p_vid : process(clk)
        variable v_l : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            v_y1 <= unsigned(data_in.y);
            if v_y1 > 64 then v_d2 <= v_y1 - 64; else v_d2 <= (others => '0'); end if;
            v_a3 <= resize(v_d2, 11) + resize(v_d2(9 downto 3), 11);
            v_b3 <= resize(v_d2(9 downto 5), 11) + resize(v_d2(9 downto 7), 11) + resize(v_d2(9 downto 9), 11);
            v_l := resize(v_a3, 12) + resize(v_b3, 12);
            if v_l > 1023 then v_l4 <= (others => '1'); else v_l4 <= v_l(9 downto 0); end if;
            v_l5 <= v_l4;
            v_p5 <= resize(v_l4, 11) + resize(v_l5, 11);
            v_p6 <= v_p5; v_p7 <= v_p6;
            v_q6 <= resize(v_p5, 12) + resize(v_p7, 12);
            if v_q6(11 downto 2) < s_crush or v_q6(11 downto 2) < C_BLK then
                v_vid <= C_BLK;
            else
                v_vid <= v_q6(11 downto 2);
            end if;
        end if;
    end process p_vid;

    ---------------------------------------------------------------------------
    -- Pixel pipeline
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a : std_logic;
        variable v_e : unsigned(8 downto 0);
        variable v_mm : signed(10 downto 0);
        variable v_o : unsigned(4 downto 0);
        variable v_u : unsigned(11 downto 0);
        variable v_m : signed(13 downto 0);
        variable v_h : unsigned(23 downto 0);
        variable v_lat : unsigned(10 downto 0);
        variable v_val : unsigned(9 downto 0);
        variable v_lo : unsigned(9 downto 0);
        variable v_y : signed(12 downto 0);
        variable v_d : unsigned(9 downto 0);
        variable v_u0, v_v0 : signed(11 downto 0);
        variable v_cu, v_cv : signed(11 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_fr : signed(10 downto 0);
        variable v_cs : signed(20 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: half cell (A/B), segment start and entry; virtual x; highlight gate
            if b_ph < s_ab then v_a := '1'; else v_a := '0'; end if;
            p_a(1) <= v_a;
            if b_first = '1' then
                p_st(1) <= '1';
                if v_a = '1' then p_e(1) <= (others => '0'); else p_e(1) <= to_unsigned(1, 9); end if;
            elsif v_a /= p_a(1) then
                p_st(1) <= '1';
                p_e(1) <= p_e(1) + 1;
            else
                p_st(1) <= '0';
            end if;
            p_x1 <= resize(shift_right(b_x, 1), 14);
            v_u := b_g(15 downto 4);
            if v_u <= 682 or v_u >= 3414 then p_gt(1) <= '1'; else p_gt(1) <= '0'; end if;

            -- 2: wedge distances; stagger offset and hash key j = (y + boil + off) / 32
            p_dl2 <= p_x1 - resize(l_xl, 14);
            p_dr2 <= resize(l_xr, 14) - p_x1;
            p_ua2 <= p_x1 - resize(s_dx1, 14) + 22;
            p_ub2 <= p_x1 - resize(s_dx2, 14) + 22;
            v_e := p_e(1);
            v_o := v_e(0) & v_e(1) & v_e(2) & v_e(3) & v_e(4);
            v_u := resize(l_ybl, 12) + resize(v_o, 12);
            p_j2 <= resize(v_u(11 downto 5), 10);
            p_k2 <= v_e(7 downto 0) + r_eb;                  -- row scroll: the entry's global key
            -- Mirror: the entry at the screen centre (first pixel right of it)
            if p_x1 >= 0 then
                if p_rt(2) = '0' then s_ec <= v_e; end if;
                p_rt(2) <= '1';
            else
                p_rt(2) <= '0';
            end if;

            -- 3: inside margin before the ragged edge; dash columns; hash round 1
            if p_dl2 < p_dr2 then v_m := p_dl2; else v_m := p_dr2; end if;
            if resize(l_d3, 14) < v_m then v_m := resize(l_d3, 14); end if;
            p_m(3) <= v_m;
            if ((p_ua2 >= 0 and p_ua2 < 45) or (p_ub2 >= 0 and p_ub2 < 45)) and p_a(2) = '1'
               and l_even = '1' and l_ylok = '1' then
                p_dash(3) <= '1';
            else
                p_dash(3) <= '0';
            end if;
            v_h(23 downto 16) := p_k2; v_h(15 downto 6) := p_j2; v_h(5 downto 1) := (others => '0'); v_h(0) := p_e(2)(8);
            v_h := v_h xor x"75A3C9";
            h3 <= v_h + shift_left(v_h, 7);
            -- 4..7: hash rounds
            v_h := h3 xor shift_right(h3, 9);
            h4 <= v_h + shift_left(v_h, 3);
            v_h := h4 xor shift_right(h4, 13);
            h5 <= v_h + shift_left(v_h, 11);
            v_h := h5 xor shift_right(h5, 7);
            h6 <= v_h + shift_left(v_h, 5);
            h7 <= h6 xor shift_right(h6, 12);

            -- 8: grey ROM address; highlight; latent bright level; inside / lite
            s_grey_a <= h7(7 downto 0);
            if p_gt(7) = '1' and h7(15 downto 8) > 97 then p_br(8) <= '1'; else p_br(8) <= '0'; end if;
            v_lat := to_unsigned(645, 11) + resize(h7(23 downto 16), 11) + resize(h7(23 downto 18), 11) + resize(h7(23 downto 21), 11);
            p_lat8 <= v_lat(9 downto 0);
            v_m := p_m(7) - signed(resize(h7(7 downto 4), 14)) - signed(resize(h7(7 downto 5), 14));
            if v_m > 0 and s_won = '1' then p_in(8) <= '1'; else p_in(8) <= '0'; end if;
            if l_even = '1' and h7(7 downto 0) < 31 then p_lite(8) <= '1'; else p_lite(8) <= '0'; end if;
            -- Mirror (right of the centre entry): read the entry the same distance left of it
            v_mm := signed(resize(s_ec & '0', 11)) - signed(resize(p_e(7), 11));
            if v_mm < 0 then p_ma8 <= (others => '0'); else p_ma8 <= unsigned(v_mm(8 downto 0)); end if;
            if s_mirror = '1' and p_rt(7) = '1' and p_e(7) > s_ec then p_mf(8) <= '1'; else p_mf(8) <= '0'; end if;
            -- 9: grey ROM output (EBR); 10: fabric copy
            s_grey_r <= s_grey_q;
            p_lat9 <= p_lat8; p_lat10 <= p_lat9;

            -- 11: the source value of this pixel (p_wk11: an Osc wedge value, exempt from K6)
            p_wk11 <= s_src and p_in(10);
            p_ma9 <= p_ma8; p_ma10 <= p_ma9;
            if s_src = '0' then
                p_vo11 <= v_vid;
            elsif p_in(10) = '1' then
                if p_dash(10) = '1' then p_vo11 <= to_unsigned(461, 10);
                elsif p_lite(10) = '1' then p_vo11 <= to_unsigned(205, 10);
                elsif l_even = '1' then p_vo11 <= to_unsigned(118, 10);
                else p_vo11 <= C_BLK; end if;
            elsif p_br(10) = '1' then
                p_vo11 <= p_lat10;
            else
                p_vo11 <= s_grey_r;
            end if;

            -- 12..13 (K6 camera pass): source x gain, then + C_PIV (1 - gain), clamped to black .. 1023
            p_gm12 <= p_vo11 * s_cg;
            p_vo12 <= p_vo11; p_wk12 <= p_wk11;
            v_cs := signed(resize(p_gm12, 21)) + resize(s_cpv, 21);
            if p_wk12 = '1' then p_vb13 <= p_vo12;
            elsif v_cs(20 downto 8) < signed(resize(C_BLK, 13)) then p_vb13 <= C_BLK;
            elsif v_cs(20 downto 18) /= "000" then p_vb13 <= (others => '1');
            else p_vb13 <= unsigned(v_cs(17 downto 8)); end if;

            -- cell RAM: held value read (address 11, EBR 12, fabric 13); on tick
            -- lines the first pixel of every half cell writes its new sample
            if p_mf(10) = '1' then s_cra <= p_ma10; else s_cra <= p_e(10); end if;
            s_crq2 <= unsigned(s_crq(9 downto 0));
            s_cwe <= '0';
            if l_tick = '1' and p_st(13) = '1' then
                s_cwe <= '1';
                s_cwa <= p_e(13);
                s_cwd <= "000000" & std_logic_vector(p_vb13);
            end if;

            -- 14 (v0.6.0: 12): the displayed value (mirrored pixels always replay the cell RAM; their
            -- entries were written earlier in the line on a tick line); every stage below is two later
            -- than its v0.6.0 number
            if p_mf(13) = '1' then
                v_val := s_crq2;
            elsif l_tick = '1' then
                if p_st(13) = '1' then v_val := p_vb13; else v_val := s_hold; end if;
                s_hold <= v_val;
            else
                v_val := s_crq2;
            end if;
            p_val12 <= v_val;

            -- 13: key window; 13..15: B half attenuation (val - black) x gain, split hi/lo
            p_dv13 <= p_val12 - C_BLK;
            p_v13 <= p_val12;
            if resize(p_val12, 11) >= s_kc and resize(p_val12, 11) < s_kw then p_cy13 <= '1'; else p_cy13 <= '0'; end if;
            if resize(p_val12, 11) >= s_kw then p_wh13 <= '1'; else p_wh13 <= '0'; end if;
            p_pl14 <= p_dv13 * s_gb8(3 downto 0);
            p_ph14 <= p_dv13 * s_gb8(7 downto 4);
            p_v14 <= p_v13; p_cy14 <= p_cy13; p_wh14 <= p_wh13;
            p_pb15 <= resize(p_pl14, 18) + shift_left(resize(p_ph14, 18), 4);
            p_v15 <= p_v14; p_cy15 <= p_cy14; p_wh15 <= p_wh14;

            -- 16: luma (L units) and the key-colour flag
            if p_a(17) = '1' then
                if p_cy15 = '1' then
                    if s_magenta = '1' then v_lo := to_unsigned(486, 10); else v_lo := to_unsigned(731, 10); end if;
                elsif p_wh15 = '1' then v_lo := to_unsigned(851, 10);
                else v_lo := p_v15; end if;
                p_c16 <= p_cy15;
            else
                v_lo := C_BLK + p_pb15(17 downto 8);
                p_c16 <= '0';
            end if;
            p_lo16 <= v_lo;

            -- 17..18: drawn luma 64 + L (1 + 1/16 - 1/128)
            p_t17 <= resize(p_lo16, 11) + resize(p_lo16(9 downto 4), 11);
            p_s17 <= p_lo16(9 downto 7);
            p_c17 <= p_c16;
            v_y := signed(resize(p_t17, 13)) + 64 - signed(resize(p_s17, 13));
            if v_y > 1023 then p_y18 <= (others => '1'); else p_y18 <= unsigned(v_y(9 downto 0)); end if;
            p_c18 <= p_c17;

            -- 19..20: soft edges: 16-px box on luma, 8-px box on the key flag (centres aligned)
            -- (luma 64 / key 0 off the generated avid, so both sums resync every blanking interval)
            if s_gav_sr(19) = '1' then
                s_ysr0 <= p_y18; s_csr(0) <= p_c18;
            else
                s_ysr0 <= to_unsigned(64, 10); s_csr(0) <= '0';
            end if;
            s_y19h <= s_y19h(14 downto 0) & s_gav_sr(19);
            for i in 1 to 12 loop s_csr(i) <= s_csr(i - 1); end loop;
            if s_y19h = "0000000000000000" then
                s_ysum <= to_unsigned(1024, 14); s_csum <= (others => '0');
            else
                s_ysum <= s_ysum + s_ysr0 - s_y16d;
                if s_csr(4) = '1' and s_csr(12) = '0' then s_csum <= s_csum + 1;
                elsif s_csr(4) = '0' and s_csr(12) = '1' then s_csum <= s_csum - 1; end if;
            end if;

            -- 21: smoothed (or hard) luma / key amount; shift registers for the fringe
            if s_hard = '1' then
                s_ys21(0) <= s_y9d;
                if s_csr(9) = '1' then s_c21(0) <= to_unsigned(8, 8); else s_c21(0) <= (others => '0'); end if;
            else
                s_ys21(0) <= s_ysum(13 downto 4);
                s_c21(0) <= resize(s_csum, 8);
            end if;
            for i in 1 to 8 loop s_ys21(i) <= s_ys21(i - 1); end loop;
            for i in 1 to 4 loop s_c21(i) <= s_c21(i - 1); end loop;

            -- 22: luma -> Cr fringe (Y[x+4] - Y[x-4]) at the centre tap
            p_dy22 <= signed(resize(s_ys21(0), 11)) - signed(resize(s_ys21(8), 11));
            p_yc22 <= s_ys21(4);
            p_c22 <= s_c21(4)(3 downto 0);

            -- 23..24: warm tint + fringe
            if p_yc22 > 64 then p_d23 <= p_yc22 - 64; else p_d23 <= (others => '0'); end if;
            p_fr23 <= shift_right(p_dy22, 4) + shift_right(p_dy22, 5) + shift_right(p_dy22, 7);
            p_y23 <= p_yc22; p_c23 <= p_c22;
            p_u24 <= to_signed(528, 12) - signed(resize(p_d23(9 downto 6), 12)) + resize(p_fr23, 12);
            p_v24 <= to_signed(498, 12) + signed(resize(p_d23(9 downto 6), 12)) + signed(resize(p_d23(9 downto 8), 12));
            p_y24 <= p_y23; p_c24 <= p_c23;

            -- 25: differences to the key colour; 26: products; 27: mix (clamped)
            if s_magenta = '1' then v_cu := to_signed(790, 12); v_cv := to_signed(720, 12);
            else v_cu := to_signed(139, 12); v_cv := to_signed(641, 12); end if;
            p_du25 <= v_cu - p_u24; p_dv25 <= v_cv - p_v24;
            p_u25 <= p_u24; p_v25 <= p_v24; p_y25 <= p_y24; p_c25 <= p_c24;
            p_pu26 <= resize(p_du25 * signed(resize(p_c25, 5)), 17);
            p_pv26 <= resize(p_dv25 * signed(resize(p_c25, 5)), 17);
            p_u26 <= p_u25; p_v26 <= p_v25; p_y26 <= p_y25;
            v_u0 := p_u26 + resize(shift_right(p_pu26, 3), 12);
            v_v0 := p_v26 + resize(shift_right(p_pv26, 3), 12);
            if s_gav_sr(27) = '0' then s_u27 <= to_unsigned(512, 10);
            elsif v_u0 < 16 then s_u27 <= to_unsigned(16, 10); elsif v_u0 > 1008 then s_u27 <= to_unsigned(1008, 10);
            else s_u27 <= unsigned(v_u0(9 downto 0)); end if;
            if s_gav_sr(27) = '0' then s_v27 <= to_unsigned(512, 10);
            elsif v_v0 < 16 then s_v27 <= to_unsigned(16, 10); elsif v_v0 > 1008 then s_v27 <= to_unsigned(1008, 10);
            else s_v27 <= unsigned(v_v0(9 downto 0)); end if;
            s_bxh <= s_bxh(6 downto 0) & s_gav_sr(27);
            s_y27 <= p_y26;

            -- 28..33: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two
            -- adjacent 8-sums, luma [1 2 1]; centre at stage 32 -> 33)
            -- (chroma neutral off the generated avid, so the sums resync every blanking interval)
            if s_bxh = "00000000" then
                s_boxu <= to_unsigned(4096, 14); s_boxv <= to_unsigned(4096, 14);
            else
                s_boxu <= s_boxu + s_u27 - s_u8d;
                s_boxv <= s_boxv + s_v27 - s_v8d;
            end if;
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u33 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v33 <= v_sc(13 downto 4);
            s_y28 <= s_y27; s_y29 <= s_y28; s_y30 <= s_y29; s_y31 <= s_y30;
            s_y32 <= s_y31; s_y33x <= s_y32;
            v_sy := resize(s_y31, 12) + shift_left(resize(s_y32, 12), 1) + resize(s_y33x, 12);
            s_yf33 <= v_sy(11 downto 2);
            -- 34: luma toward black (white 940 -> 777); blanking gate (the output register).
            if s_yf33 > 64 then v_dl := s_yf33 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(34) = '1' then
                s_y34 <= s_yf33 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u34 <= s_u33; s_v34 <= s_v33;
            else
                s_y34 <= to_unsigned(64, 10); s_u34 <= to_unsigned(512, 10); s_v34 <= to_unsigned(512, 10);
            end if;

            -- delays
            for i in 2 to 18 loop p_a(i) <= p_a(i - 1); p_st(i) <= p_st(i - 1); p_e(i) <= p_e(i - 1); end loop;
            for i in 2 to 7 loop p_gt(i) <= p_gt(i - 1); end loop;
            for i in 4 to 10 loop p_dash(i) <= p_dash(i - 1); end loop;
            for i in 4 to 7 loop p_m(i) <= p_m(i - 1); end loop;
            for i in 9 to 10 loop p_br(i) <= p_br(i - 1); p_in(i) <= p_in(i - 1); p_lite(i) <= p_lite(i - 1); end loop;
            for i in 3 to 7 loop p_rt(i) <= p_rt(i - 1); end loop;
            for i in 9 to 13 loop p_mf(i) <= p_mf(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y34);
    data_out.u       <= std_logic_vector(s_u34);
    data_out.v       <= std_logic_vector(s_v34);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture ticker;
