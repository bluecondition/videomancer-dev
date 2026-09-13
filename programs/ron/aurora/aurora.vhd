-- aurora.vhd  (v1.1)
--
-- AURORA: a soft horizontal rainbow ribbon that drifts, undulates and -- the
-- signature gesture -- FOLDS BACK OVER ITSELF like silk in slow-moving water.
--
-- ARCHITECTURE (no framebuffer; true self-overlap per scanline)
--
--   A y(x) band cannot double back, and an implicit field welds overlapping
--   folds together instead of layering them.  So the ribbon is PARAMETRIC:
--   during every line period a marcher walks the curve
--
--       x(u) = x0 + u/4 + Fx*cos(thf)                (prolate cycloid: when
--       y(u) = P + Fd*sin(thf)                        Fx*wf > 1/4 the path
--              + W*(sin(th1) + sin(th2)/2)            runs BACKWARD -> loops)
--
--   and stamps {hue[9], alpha[5]} into a line buffer at half-pixel columns
--   for the NEXT display row: alpha = feather(|y(u) - R|), and hue runs
--   ACROSS the ribbon's thickness (hue = phc - (y-R)*hscale, ROYGBV top to
--   bottom, hscale = spectrum/H from a per-frame divide) so the rainbow
--   stripes ride WITH the curve and fold with it.  Stamps land through a
--   max-alpha read-modify-write on the stamp bank's otherwise idle read
--   port: where the ribbon crosses itself the more-solid (and on ties the
--   later) pass wins per pixel -- fabric doubling over, cleanly layered.
--   P12 raises Fd first (waves -> S-curves), then Fx past the backward-
--   slope threshold (loops); loop width eases off at extreme Height to
--   respect the write-bandwidth wall.
--
--   Budget: one stamp per clock, one line period per row.  An ADAPTIVE STEP
--   makes that fit every raster: far from the target row the marcher jumps
--   u by 16 units (phases are linear in u -- jumping is a shifted add), and
--   tiers down through 8/4/2 as |y-R| approaches the band (thresholds carry
--   a worst-slope * pipeline-staleness guard, precomputed per frame).  In
--   band it paces by predicted |dx/du| so columns are never skipped; a
--   1-clock bridge write covers the rare 2-column hop.  A hard clock cap
--   (shortest line of the raster class minus margin) bounds pathological
--   corners.
--
--   Line buffer: 2 banks x 1024 x 14b (4 EBR each).  While row R displays
--   from bank R&1 (each word cleared behind the beam), row R+1 is stamped
--   into the other bank; the march is triggered by avid_start so the write
--   window is exactly one line period.  Post-active vblank lines stamp row
--   0 into bank 0 (identical params -> idempotent restamps).  Six 256x9
--   quarter-wave sine ROMs (1 EBR each): 4 for the marcher, 2 for the beam.
--
--   Beam: word -> hue -> (Solid: quantize to 6 flag stripes) -> vectorscope
--   angle -> sin/cos ROMs -> chroma at radius 440, Y = 764 - Cb/4 (yellow
--   bright, blue deep) -> alpha-composite over black or the live picture.
--   Chroma is authored DIRECTLY in the verified hardware convention (the u
--   register carries BT.601 Cr -- phosphor-palette anchors), so the sim
--   image shows mirrored hues; the scope and the screen show ROYGBV.
--
--   Geometry is computed in FRAME rows on every raster; interlaced fields
--   compare against R*2+parity (bottom field = field_n '0'), so both fields
--   sample the same curve with correct phase.  All motion phases advance
--   once per frame at vsync; K1/K2/K3/K6/P12 are slewed (1-pole, ~250 ms)
--   so performance moves are glitch-free.  Stillness is exact: K3 at zero
--   freezes the wiggle/fold phase clocks, K5 has a true center deadband,
--   and S9 hard-freezes everything.
--
-- CONTROLS
--   K1  Height     ribbon thickness: whisper stripe -> ~40%-screen sash
--   K2  Position   vertical home line
--   K3  Wiggle     undulation amplitude; fully CCW = dead still
--   K4  Rate       undulation speed (no effect while Wiggle is zero)
--   K5  Flow       rainbow drift along the ribbon; center = stopped
--   K6  Spectrum   rainbow cycles across the ribbon, 0.5 .. 4
--   S7  Color      Gradient (continuous) / Solid (6 flag stripes)
--   S8  Backdrop   Black / live video (ribbon composites on top)
--   S9  Freeze     hard-stop all animation at its current phase
--   P12 FOLD       the gesture: flat -> waves -> S-curves -> full loops
--
-- Latency: 9 clocks, constant across all modes.  EBR: 14.  No DSP.
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture aurora of program_top is

    constant C_LAT : integer := 8;

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- quarter-wave sine: qsin(i) = round(511*sin((i+0.5)*pi/512))
    type t_qsin is array(0 to 255) of unsigned(8 downto 0);
    constant C_QSIN : t_qsin := (
        to_unsigned(  2, 9), to_unsigned(  5, 9), to_unsigned(  8, 9), to_unsigned( 11, 9),
        to_unsigned( 14, 9), to_unsigned( 17, 9), to_unsigned( 20, 9), to_unsigned( 24, 9),
        to_unsigned( 27, 9), to_unsigned( 30, 9), to_unsigned( 33, 9), to_unsigned( 36, 9),
        to_unsigned( 39, 9), to_unsigned( 42, 9), to_unsigned( 45, 9), to_unsigned( 49, 9),
        to_unsigned( 52, 9), to_unsigned( 55, 9), to_unsigned( 58, 9), to_unsigned( 61, 9),
        to_unsigned( 64, 9), to_unsigned( 67, 9), to_unsigned( 70, 9), to_unsigned( 73, 9),
        to_unsigned( 77, 9), to_unsigned( 80, 9), to_unsigned( 83, 9), to_unsigned( 86, 9),
        to_unsigned( 89, 9), to_unsigned( 92, 9), to_unsigned( 95, 9), to_unsigned( 98, 9),
        to_unsigned(101, 9), to_unsigned(104, 9), to_unsigned(107, 9), to_unsigned(110, 9),
        to_unsigned(113, 9), to_unsigned(117, 9), to_unsigned(120, 9), to_unsigned(123, 9),
        to_unsigned(126, 9), to_unsigned(129, 9), to_unsigned(132, 9), to_unsigned(135, 9),
        to_unsigned(138, 9), to_unsigned(141, 9), to_unsigned(144, 9), to_unsigned(147, 9),
        to_unsigned(150, 9), to_unsigned(153, 9), to_unsigned(156, 9), to_unsigned(159, 9),
        to_unsigned(162, 9), to_unsigned(165, 9), to_unsigned(168, 9), to_unsigned(171, 9),
        to_unsigned(174, 9), to_unsigned(177, 9), to_unsigned(180, 9), to_unsigned(182, 9),
        to_unsigned(185, 9), to_unsigned(188, 9), to_unsigned(191, 9), to_unsigned(194, 9),
        to_unsigned(197, 9), to_unsigned(200, 9), to_unsigned(203, 9), to_unsigned(206, 9),
        to_unsigned(209, 9), to_unsigned(211, 9), to_unsigned(214, 9), to_unsigned(217, 9),
        to_unsigned(220, 9), to_unsigned(223, 9), to_unsigned(226, 9), to_unsigned(228, 9),
        to_unsigned(231, 9), to_unsigned(234, 9), to_unsigned(237, 9), to_unsigned(239, 9),
        to_unsigned(242, 9), to_unsigned(245, 9), to_unsigned(248, 9), to_unsigned(251, 9),
        to_unsigned(253, 9), to_unsigned(256, 9), to_unsigned(259, 9), to_unsigned(261, 9),
        to_unsigned(264, 9), to_unsigned(267, 9), to_unsigned(269, 9), to_unsigned(272, 9),
        to_unsigned(275, 9), to_unsigned(277, 9), to_unsigned(280, 9), to_unsigned(283, 9),
        to_unsigned(285, 9), to_unsigned(288, 9), to_unsigned(290, 9), to_unsigned(293, 9),
        to_unsigned(296, 9), to_unsigned(298, 9), to_unsigned(301, 9), to_unsigned(303, 9),
        to_unsigned(306, 9), to_unsigned(308, 9), to_unsigned(311, 9), to_unsigned(313, 9),
        to_unsigned(316, 9), to_unsigned(318, 9), to_unsigned(321, 9), to_unsigned(323, 9),
        to_unsigned(325, 9), to_unsigned(328, 9), to_unsigned(330, 9), to_unsigned(333, 9),
        to_unsigned(335, 9), to_unsigned(337, 9), to_unsigned(340, 9), to_unsigned(342, 9),
        to_unsigned(344, 9), to_unsigned(347, 9), to_unsigned(349, 9), to_unsigned(351, 9),
        to_unsigned(353, 9), to_unsigned(356, 9), to_unsigned(358, 9), to_unsigned(360, 9),
        to_unsigned(362, 9), to_unsigned(365, 9), to_unsigned(367, 9), to_unsigned(369, 9),
        to_unsigned(371, 9), to_unsigned(373, 9), to_unsigned(375, 9), to_unsigned(378, 9),
        to_unsigned(380, 9), to_unsigned(382, 9), to_unsigned(384, 9), to_unsigned(386, 9),
        to_unsigned(388, 9), to_unsigned(390, 9), to_unsigned(392, 9), to_unsigned(394, 9),
        to_unsigned(396, 9), to_unsigned(398, 9), to_unsigned(400, 9), to_unsigned(402, 9),
        to_unsigned(404, 9), to_unsigned(406, 9), to_unsigned(408, 9), to_unsigned(410, 9),
        to_unsigned(411, 9), to_unsigned(413, 9), to_unsigned(415, 9), to_unsigned(417, 9),
        to_unsigned(419, 9), to_unsigned(420, 9), to_unsigned(422, 9), to_unsigned(424, 9),
        to_unsigned(426, 9), to_unsigned(427, 9), to_unsigned(429, 9), to_unsigned(431, 9),
        to_unsigned(433, 9), to_unsigned(434, 9), to_unsigned(436, 9), to_unsigned(437, 9),
        to_unsigned(439, 9), to_unsigned(441, 9), to_unsigned(442, 9), to_unsigned(444, 9),
        to_unsigned(445, 9), to_unsigned(447, 9), to_unsigned(448, 9), to_unsigned(450, 9),
        to_unsigned(451, 9), to_unsigned(453, 9), to_unsigned(454, 9), to_unsigned(456, 9),
        to_unsigned(457, 9), to_unsigned(459, 9), to_unsigned(460, 9), to_unsigned(461, 9),
        to_unsigned(463, 9), to_unsigned(464, 9), to_unsigned(465, 9), to_unsigned(467, 9),
        to_unsigned(468, 9), to_unsigned(469, 9), to_unsigned(470, 9), to_unsigned(472, 9),
        to_unsigned(473, 9), to_unsigned(474, 9), to_unsigned(475, 9), to_unsigned(476, 9),
        to_unsigned(477, 9), to_unsigned(478, 9), to_unsigned(480, 9), to_unsigned(481, 9),
        to_unsigned(482, 9), to_unsigned(483, 9), to_unsigned(484, 9), to_unsigned(485, 9),
        to_unsigned(486, 9), to_unsigned(487, 9), to_unsigned(488, 9), to_unsigned(489, 9),
        to_unsigned(489, 9), to_unsigned(490, 9), to_unsigned(491, 9), to_unsigned(492, 9),
        to_unsigned(493, 9), to_unsigned(494, 9), to_unsigned(495, 9), to_unsigned(495, 9),
        to_unsigned(496, 9), to_unsigned(497, 9), to_unsigned(498, 9), to_unsigned(498, 9),
        to_unsigned(499, 9), to_unsigned(500, 9), to_unsigned(500, 9), to_unsigned(501, 9),
        to_unsigned(501, 9), to_unsigned(502, 9), to_unsigned(503, 9), to_unsigned(503, 9),
        to_unsigned(504, 9), to_unsigned(504, 9), to_unsigned(505, 9), to_unsigned(505, 9),
        to_unsigned(506, 9), to_unsigned(506, 9), to_unsigned(507, 9), to_unsigned(507, 9),
        to_unsigned(507, 9), to_unsigned(508, 9), to_unsigned(508, 9), to_unsigned(508, 9),
        to_unsigned(509, 9), to_unsigned(509, 9), to_unsigned(509, 9), to_unsigned(509, 9),
        to_unsigned(510, 9), to_unsigned(510, 9), to_unsigned(510, 9), to_unsigned(510, 9),
        to_unsigned(510, 9), to_unsigned(511, 9), to_unsigned(511, 9), to_unsigned(511, 9),
        to_unsigned(511, 9), to_unsigned(511, 9), to_unsigned(511, 9), to_unsigned(511, 9));

    -- 13-bit quarter-wave sine for the coil sweep (Ax up to ~500 half-cols
    -- amplifies every LSB, so the swing needs more amplitude resolution
    -- than the 9-bit wiggle table; still one 256x16 EBR)
    type t_qsin13 is array(0 to 255) of unsigned(11 downto 0);
    constant C_QSIN13 : t_qsin13 := (
        to_unsigned(  13, 12), to_unsigned(  38, 12), to_unsigned(  63, 12), to_unsigned(  88, 12),
        to_unsigned( 113, 12), to_unsigned( 138, 12), to_unsigned( 163, 12), to_unsigned( 188, 12),
        to_unsigned( 213, 12), to_unsigned( 239, 12), to_unsigned( 264, 12), to_unsigned( 289, 12),
        to_unsigned( 314, 12), to_unsigned( 339, 12), to_unsigned( 364, 12), to_unsigned( 389, 12),
        to_unsigned( 414, 12), to_unsigned( 439, 12), to_unsigned( 464, 12), to_unsigned( 489, 12),
        to_unsigned( 514, 12), to_unsigned( 539, 12), to_unsigned( 564, 12), to_unsigned( 588, 12),
        to_unsigned( 613, 12), to_unsigned( 638, 12), to_unsigned( 663, 12), to_unsigned( 688, 12),
        to_unsigned( 712, 12), to_unsigned( 737, 12), to_unsigned( 762, 12), to_unsigned( 787, 12),
        to_unsigned( 811, 12), to_unsigned( 836, 12), to_unsigned( 860, 12), to_unsigned( 885, 12),
        to_unsigned( 909, 12), to_unsigned( 934, 12), to_unsigned( 958, 12), to_unsigned( 983, 12),
        to_unsigned(1007, 12), to_unsigned(1032, 12), to_unsigned(1056, 12), to_unsigned(1080, 12),
        to_unsigned(1104, 12), to_unsigned(1128, 12), to_unsigned(1153, 12), to_unsigned(1177, 12),
        to_unsigned(1201, 12), to_unsigned(1225, 12), to_unsigned(1249, 12), to_unsigned(1273, 12),
        to_unsigned(1296, 12), to_unsigned(1320, 12), to_unsigned(1344, 12), to_unsigned(1368, 12),
        to_unsigned(1391, 12), to_unsigned(1415, 12), to_unsigned(1439, 12), to_unsigned(1462, 12),
        to_unsigned(1485, 12), to_unsigned(1509, 12), to_unsigned(1532, 12), to_unsigned(1555, 12),
        to_unsigned(1579, 12), to_unsigned(1602, 12), to_unsigned(1625, 12), to_unsigned(1648, 12),
        to_unsigned(1671, 12), to_unsigned(1694, 12), to_unsigned(1717, 12), to_unsigned(1739, 12),
        to_unsigned(1762, 12), to_unsigned(1785, 12), to_unsigned(1807, 12), to_unsigned(1830, 12),
        to_unsigned(1852, 12), to_unsigned(1875, 12), to_unsigned(1897, 12), to_unsigned(1919, 12),
        to_unsigned(1941, 12), to_unsigned(1964, 12), to_unsigned(1986, 12), to_unsigned(2007, 12),
        to_unsigned(2029, 12), to_unsigned(2051, 12), to_unsigned(2073, 12), to_unsigned(2094, 12),
        to_unsigned(2116, 12), to_unsigned(2137, 12), to_unsigned(2159, 12), to_unsigned(2180, 12),
        to_unsigned(2201, 12), to_unsigned(2223, 12), to_unsigned(2244, 12), to_unsigned(2265, 12),
        to_unsigned(2285, 12), to_unsigned(2306, 12), to_unsigned(2327, 12), to_unsigned(2348, 12),
        to_unsigned(2368, 12), to_unsigned(2389, 12), to_unsigned(2409, 12), to_unsigned(2429, 12),
        to_unsigned(2449, 12), to_unsigned(2470, 12), to_unsigned(2490, 12), to_unsigned(2509, 12),
        to_unsigned(2529, 12), to_unsigned(2549, 12), to_unsigned(2569, 12), to_unsigned(2588, 12),
        to_unsigned(2608, 12), to_unsigned(2627, 12), to_unsigned(2646, 12), to_unsigned(2665, 12),
        to_unsigned(2684, 12), to_unsigned(2703, 12), to_unsigned(2722, 12), to_unsigned(2741, 12),
        to_unsigned(2759, 12), to_unsigned(2778, 12), to_unsigned(2796, 12), to_unsigned(2815, 12),
        to_unsigned(2833, 12), to_unsigned(2851, 12), to_unsigned(2869, 12), to_unsigned(2887, 12),
        to_unsigned(2904, 12), to_unsigned(2922, 12), to_unsigned(2940, 12), to_unsigned(2957, 12),
        to_unsigned(2974, 12), to_unsigned(2992, 12), to_unsigned(3009, 12), to_unsigned(3026, 12),
        to_unsigned(3043, 12), to_unsigned(3059, 12), to_unsigned(3076, 12), to_unsigned(3093, 12),
        to_unsigned(3109, 12), to_unsigned(3125, 12), to_unsigned(3141, 12), to_unsigned(3157, 12),
        to_unsigned(3173, 12), to_unsigned(3189, 12), to_unsigned(3205, 12), to_unsigned(3221, 12),
        to_unsigned(3236, 12), to_unsigned(3251, 12), to_unsigned(3267, 12), to_unsigned(3282, 12),
        to_unsigned(3297, 12), to_unsigned(3311, 12), to_unsigned(3326, 12), to_unsigned(3341, 12),
        to_unsigned(3355, 12), to_unsigned(3370, 12), to_unsigned(3384, 12), to_unsigned(3398, 12),
        to_unsigned(3412, 12), to_unsigned(3426, 12), to_unsigned(3439, 12), to_unsigned(3453, 12),
        to_unsigned(3466, 12), to_unsigned(3480, 12), to_unsigned(3493, 12), to_unsigned(3506, 12),
        to_unsigned(3519, 12), to_unsigned(3532, 12), to_unsigned(3544, 12), to_unsigned(3557, 12),
        to_unsigned(3569, 12), to_unsigned(3581, 12), to_unsigned(3594, 12), to_unsigned(3606, 12),
        to_unsigned(3617, 12), to_unsigned(3629, 12), to_unsigned(3641, 12), to_unsigned(3652, 12),
        to_unsigned(3663, 12), to_unsigned(3675, 12), to_unsigned(3686, 12), to_unsigned(3696, 12),
        to_unsigned(3707, 12), to_unsigned(3718, 12), to_unsigned(3728, 12), to_unsigned(3739, 12),
        to_unsigned(3749, 12), to_unsigned(3759, 12), to_unsigned(3769, 12), to_unsigned(3778, 12),
        to_unsigned(3788, 12), to_unsigned(3798, 12), to_unsigned(3807, 12), to_unsigned(3816, 12),
        to_unsigned(3825, 12), to_unsigned(3834, 12), to_unsigned(3843, 12), to_unsigned(3851, 12),
        to_unsigned(3860, 12), to_unsigned(3868, 12), to_unsigned(3876, 12), to_unsigned(3884, 12),
        to_unsigned(3892, 12), to_unsigned(3900, 12), to_unsigned(3908, 12), to_unsigned(3915, 12),
        to_unsigned(3922, 12), to_unsigned(3929, 12), to_unsigned(3936, 12), to_unsigned(3943, 12),
        to_unsigned(3950, 12), to_unsigned(3957, 12), to_unsigned(3963, 12), to_unsigned(3969, 12),
        to_unsigned(3975, 12), to_unsigned(3981, 12), to_unsigned(3987, 12), to_unsigned(3993, 12),
        to_unsigned(3998, 12), to_unsigned(4004, 12), to_unsigned(4009, 12), to_unsigned(4014, 12),
        to_unsigned(4019, 12), to_unsigned(4023, 12), to_unsigned(4028, 12), to_unsigned(4033, 12),
        to_unsigned(4037, 12), to_unsigned(4041, 12), to_unsigned(4045, 12), to_unsigned(4049, 12),
        to_unsigned(4053, 12), to_unsigned(4056, 12), to_unsigned(4059, 12), to_unsigned(4063, 12),
        to_unsigned(4066, 12), to_unsigned(4069, 12), to_unsigned(4071, 12), to_unsigned(4074, 12),
        to_unsigned(4076, 12), to_unsigned(4079, 12), to_unsigned(4081, 12), to_unsigned(4083, 12),
        to_unsigned(4085, 12), to_unsigned(4087, 12), to_unsigned(4088, 12), to_unsigned(4089, 12),
        to_unsigned(4091, 12), to_unsigned(4092, 12), to_unsigned(4093, 12), to_unsigned(4093, 12),
        to_unsigned(4094, 12), to_unsigned(4095, 12), to_unsigned(4095, 12), to_unsigned(4095, 12));

    -- Precomputed ribbon color LUT: the seven HW-verified phosphor colors
    -- (scope-confirmed palette) interpolated to 512 steps and packed
    -- {Y[9:0], U[9:0], V[9:0]} -- the u register carries BT.601 Cr on this
    -- hardware, so these are the raw register values.  As a constant this
    -- infers EBR, replacing ~400 LC of runtime palette muxes/multiplies.
    -- Segment starts (addr = k*73) hold the PURE colors, which is what
    -- Solid mode addresses.
    type t_clut is array(0 to 511) of std_logic_vector(31 downto 0);
    constant C_CLUT : t_clut := (
        x"148F0168", x"14BEF565", x"14FEE963", x"152EE161",
        x"156ED55F", x"159ECD5D", x"15DEC15B", x"160EB959",
        x"164EAD57", x"167EA155", x"16BE9953", x"16EE8D51",
        x"172E854F", x"175E794D", x"179E714B", x"17CE6549",
        x"180E5947", x"183E5145", x"187E4543", x"18AE3D41",
        x"18EE313F", x"191E293D", x"195E1D3B", x"198E1139",
        x"19CE0937", x"19FDFD35", x"1A3DF533", x"1A6DE931",
        x"1AADE12F", x"1ADDD52D", x"1B1DC92B", x"1B4DC129",
        x"1B8DB527", x"1BBDAD25", x"1BFDA123", x"1C2D9921",
        x"1C6D8D1F", x"1C9D811D", x"1CDD791B", x"1D0D6D19",
        x"1D4D6517", x"1D7D5915", x"1DBD5113", x"1DED4510",
        x"1E2D390E", x"1E5D310C", x"1E9D250A", x"1ECD1D08",
        x"1F0D1106", x"1F3D0904", x"1F7CFD02", x"1FACF100",
        x"1FECE8FE", x"201CDCFC", x"205CD4FA", x"208CC8F8",
        x"20CCC0F6", x"20FCB4F4", x"213CA8F2", x"216CA0F0",
        x"21AC94EE", x"21DC8CEC", x"221C80EA", x"224C78E8",
        x"228C6CE6", x"22BC60E4", x"22FC58E2", x"232C4CE0",
        x"236C44DE", x"239C38DC", x"23DC30DA", x"240C24D8",
        x"244C18D6", x"247C10D4", x"24BC04D2", x"24EBFCD0",
        x"252BF0CE", x"255BE8CC", x"259BDCCA", x"25CBD0C8",
        x"260BC8C6", x"263BBCC4", x"267BB4C2", x"26ABA8C0",
        x"26EBA0BE", x"271B94BC", x"275B88B9", x"278B80B7",
        x"27CB74B5", x"27FB6CB3", x"283B60B1", x"286B58AF",
        x"28AB4CAD", x"28DB40AB", x"291B38A9", x"294B2CA7",
        x"298B24A5", x"29BB18A3", x"29FB10A1", x"2A2B049F",
        x"2A6AF89D", x"2A9AF09B", x"2ADAE499", x"2B0ADC97",
        x"2B4AD095", x"2B7AC893", x"2BBABC91", x"2BEAB08F",
        x"2C2AA88D", x"2C5A9C8B", x"2C9A9489", x"2CCA8887",
        x"2D0A8085", x"2D3A7483", x"2D7A6881", x"2DAA607F",
        x"2DEA547D", x"2E1A4C7B", x"2E5A4079", x"2E8A3877",
        x"2ECA2C75", x"2EFA2073", x"2F3A1871", x"2F6A0C6F",
        x"2FAA046D", x"2FD9F86B", x"3019F069", x"3049E467",
        x"3089DC65", x"30B9D062", x"30F9C460", x"3129BC5E",
        x"3169B05C", x"3199A85A", x"31D99C58", x"32099456",
        x"32498854", x"32797C52", x"32B97450", x"32E9684E",
        x"3329604C", x"3359544A", x"33994C48", x"33C94046",
        x"34093444", x"34392C42", x"34792040", x"34590C41",
        x"3418F443", x"33E8DC45", x"33A8C447", x"3378AC49",
        x"3338944B", x"3308784D", x"32C86050", x"32984852",
        x"32583054", x"32181856", x"31E80058", x"31A7E85A",
        x"3177D05C", x"3137B45E", x"31079C60", x"30C78462",
        x"30976C64", x"30575466", x"30173C68", x"2FE7246B",
        x"2FA70C6D", x"2F76F06F", x"2F36D871", x"2F06C073",
        x"2EC6A875", x"2E969077", x"2E567879", x"2E16607B",
        x"2DE6487D", x"2DA62C7F", x"2D761481", x"2D35FC83",
        x"2D05E486", x"2CC5CC88", x"2C95B48A", x"2C559C8C",
        x"2C15848E", x"2BE56890", x"2BA55092", x"2B753894",
        x"2B352096", x"2B050898", x"2AC4F09A", x"2A94D89C",
        x"2A54C09F", x"2A14A4A1", x"29E48CA3", x"29A474A5",
        x"29745CA7", x"293444A9", x"29042CAB", x"28C414AD",
        x"2893FCAF", x"2853E0B1", x"2813C8B3", x"27E3B0B5",
        x"27A398B7", x"277380BA", x"273368BC", x"270350BE",
        x"26C338C0", x"26931CC2", x"265304C4", x"2612ECC6",
        x"25E2D4C8", x"25A2BCCA", x"2572A4CC", x"25328CCE",
        x"250274D0", x"24C258D2", x"249240D5", x"245228D7",
        x"240228DD", x"23B238E7", x"235248F2", x"22F258FC",
        x"22A26D06", x"22427D10", x"21E28D1A", x"21829D25",
        x"2132AD2F", x"20D2BD39", x"2072CD43", x"2022E14D",
        x"1FC2F157", x"1F630162", x"1F13116C", x"1EB32176",
        x"1E533180", x"1E03458A", x"1DA35594", x"1D43659F",
        x"1CF375A9", x"1C9385B3", x"1C3395BD", x"1BD3A5C7",
        x"1B83B9D1", x"1B23C9DC", x"1AC3D9E6", x"1A73E9F0",
        x"1A13F9FA", x"19B40A04", x"19641E0E", x"19042E19",
        x"18A43E23", x"18544E2D", x"17F45E37", x"17946E41",
        x"1744824C", x"16E49256", x"1684A260", x"1624B26A",
        x"15D4C274", x"1574D27E", x"1514E289", x"14C4F693",
        x"1465069D", x"140516A7", x"13B526B1", x"135536BB",
        x"12F546C6", x"12A55AD0", x"12456ADA", x"11E57AE4",
        x"11958AEE", x"11359AF8", x"10D5AB03", x"1075BB0D",
        x"1025CF17", x"0FC5DF21", x"0F65EF2B", x"0F15FF35",
        x"0EB60F40", x"0E561F4A", x"0E063354", x"0DA6435E",
        x"0D465368", x"0CF66372", x"0C96737D", x"0C368387",
        x"0BE69791", x"0B86A79B", x"0B26B7A5", x"0AC6C7B0",
        x"0A76D7BA", x"0A46E3BE", x"0A46EFBA", x"0A46F7B7",
        x"0A56FFB3", x"0A570BB0", x"0A6713AC", x"0A671BA8",
        x"0A6727A5", x"0A772FA1", x"0A77379D", x"0A773F9A",
        x"0A874B96", x"0A875393", x"0A975B8F", x"0A97678B",
        x"0A976F88", x"0AA77784", x"0AA78381", x"0AB78B7D",
        x"0AB79379", x"0AB79B76", x"0AC7A772", x"0AC7AF6F",
        x"0AC7B76B", x"0AD7C367", x"0AD7CB64", x"0AE7D360",
        x"0AE7DF5D", x"0AE7E759", x"0AF7EF55", x"0AF7F752",
        x"0B08034E", x"0B080B4A", x"0B081347", x"0B181F43",
        x"0B182740", x"0B182F3C", x"0B283738", x"0B284335",
        x"0B384B31", x"0B38532E", x"0B385F2A", x"0B486726",
        x"0B486F23", x"0B587B1F", x"0B58831C", x"0B588B18",
        x"0B689314", x"0B689F11", x"0B68A70D", x"0B78AF09",
        x"0B78BB06", x"0B88C302", x"0B88CAFF", x"0B88D6FB",
        x"0B98DEF7", x"0B98E6F4", x"0B98EEF0", x"0BA8FAED",
        x"0BA902E9", x"0BB90AE5", x"0BB916E2", x"0BB91EDE",
        x"0BC926DB", x"0BC92ED7", x"0BD93AD3", x"0BD942D0",
        x"0BD94ACC", x"0BE956C9", x"0BE95EC5", x"0BE966C1",
        x"0BF972BE", x"0BF97ABA", x"0C0982B8", x"0C298ABA",
        x"0C4992BC", x"0C799ABF", x"0C99A2C1", x"0CB9AAC3",
        x"0CD9B2C5", x"0CF9BAC7", x"0D19C2C9", x"0D39CACB",
        x"0D69D2CD", x"0D89DAD0", x"0DA9E2D2", x"0DC9EAD4",
        x"0DE9F2D6", x"0E09FAD8", x"0E2A02DA", x"0E5A0ADC",
        x"0E7A16DF", x"0E9A1EE1", x"0EBA26E3", x"0EDA2EE5",
        x"0EFA36E7", x"0F1A3EE9", x"0F4A46EB", x"0F6A4EED",
        x"0F8A56F0", x"0FAA5EF2", x"0FCA66F4", x"0FEA6EF6",
        x"101A76F8", x"103A7EFA", x"105A86FC", x"107A8EFE",
        x"109A9701", x"10BA9F03", x"10DAA705", x"110AAF07",
        x"112AB709", x"114ABF0B", x"116AC70D", x"118ACF10",
        x"11AAD712", x"11CADF14", x"11FAE716", x"121AEF18",
        x"123AF71A", x"125AFF1C", x"127B071E", x"129B0F21",
        x"12BB1723", x"12EB1F25", x"130B2727", x"132B2F29",
        x"134B372B", x"136B3F2D", x"138B4730", x"13AB4F32",
        x"13DB5734", x"13FB5F36", x"141B6738", x"143B733A",
        x"145B7B3C", x"147B833E", x"149B8B41", x"14CB9343",
        x"14EB9B45", x"150BA347", x"152BAB49", x"154BB34B",
        x"156BBB4D", x"159BC350", x"15BBCB52", x"15CBD353",
        x"15CBDF4C", x"15CBE745", x"15CBF33E", x"15BBFF38",
        x"15BC0B31", x"15BC172A", x"15AC1F23", x"15AC2B1D",
        x"15AC3716", x"15AC430F", x"159C4F09", x"159C5702",
        x"159C62FB", x"158C6EF4", x"158C7AEE", x"158C86E7",
        x"158C8EE0", x"157C9AD9", x"157CA6D3", x"157CB2CC",
        x"156CBAC5", x"156CC6BF", x"156CD2B8", x"156CDEB1",
        x"155CEAAA", x"155CF2A4", x"155CFE9D", x"154D0A96",
        x"154D168F", x"154D2289", x"154D2A82", x"153D367B",
        x"153D4275", x"153D4E6E", x"152D5A67", x"152D6260",
        x"152D6E5A", x"152D7A53", x"151D864C", x"151D8E45",
        x"151D9A3F", x"150DA638", x"150DB231", x"150DBE2B",
        x"150DC624", x"14FDD21D", x"14FDDE16", x"14FDEA10",
        x"14EDF609", x"14EDFE02", x"14EE09FB", x"14EE15F5",
        x"14DE21EE", x"14DE2DE7", x"14DE35E1", x"14CE41DA",
        x"14CE4DD3", x"14CE59CC", x"14CE61C6", x"14BE6DBF",
        x"14BE79B8", x"14BE85B1", x"14AE91AB", x"14AE99A4",
        x"14AEA59D", x"14AEB197", x"149EBD90", x"149EC989",
        x"149ED182", x"148EDD7C", x"148EE975", x"148EF56E");

    -- 4096/sqrt(1+slope^2) with slope in Q3, as (value, delta-to-next) so a
    -- single read gives everything needed to INTERPOLATE.  Converts vertical
    -- distance into distance perpendicular to the curve, keeping the ribbon
    -- one constant width through every bend.
    --
    -- Only 16 entries because the factor is clamped at 1.5x (slope 1.12): the
    -- estimate is really distance to the TANGENT LINE, which underestimates
    -- where the curve bends tighter than the ribbon is thick, and unclamped
    -- it grows horns above every crest.
    --
    -- The interpolation is what keeps the edges SMOOTH.  Indexed on the raw
    -- Q3 slope alone, each index step moved the ribbon edge by ~3 rows and
    -- the top and bottom edges came out visibly toothed.
    -- Two flat arrays rather than an array of records: a single record
    -- constant read in two statements is the array-of-arrays pattern this
    -- toolchain mishandles, and it produced garbage factors on scattered
    -- columns (each one a solid vertical line).
    type t_rfv is array(0 to 15) of unsigned(11 downto 0);
    type t_rfd is array(0 to 15) of signed(9 downto 0);
    constant C_RFV : t_rfv := (
        to_unsigned(4095, 12), to_unsigned(4064, 12), to_unsigned(3974, 12), to_unsigned(3835, 12),
        to_unsigned(3664, 12), to_unsigned(3473, 12), to_unsigned(3277, 12), to_unsigned(3083, 12),
        to_unsigned(2896, 12), to_unsigned(2730, 12), to_unsigned(2730, 12), to_unsigned(2730, 12),
        to_unsigned(2730, 12), to_unsigned(2730, 12), to_unsigned(2730, 12), to_unsigned(2730, 12));
    constant C_RFD : t_rfd := (
        to_signed( -31, 10), to_signed( -90, 10), to_signed(-139, 10), to_signed(-171, 10),
        to_signed(-191, 10), to_signed(-196, 10), to_signed(-194, 10), to_signed(-187, 10),
        to_signed(-166, 10), to_signed(   0, 10), to_signed(   0, 10), to_signed(   0, 10),
        to_signed(   0, 10), to_signed(   0, 10), to_signed(   0, 10), to_signed(   0, 10));



    -- ==== timing / raster ====
    signal s_timing  : t_video_timing_port;
    signal s_hcnt    : unsigned(11 downto 0) := (others => '0');
    signal s_vcnt    : unsigned(11 downto 0) := (others => '0');
    signal s_meas_h  : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_meas_v  : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal s_ilace   : std_logic := '0';
    signal s_botf    : std_logic := '0';

    -- ==== per-raster class constants (latched at vsync) ====
    signal s_cols  : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_marg  : unsigned(6 downto 0)  := to_unsigned(64, 7);
    signal s_cap   : unsigned(11 downto 0) := to_unsigned(2180, 12);
    signal s_kwav  : unsigned(11 downto 0) := to_unsigned(68, 12);  -- 65536/cols
    signal s_kenv  : unsigned(11 downto 0) := to_unsigned(34, 12);  -- 32768/cols
    signal s_kw7   : unsigned(14 downto 0) := to_unsigned(476, 15); -- kwav * 7
    signal s_fh    : unsigned(11 downto 0) := to_unsigned(1080, 12); -- frame rows

    -- ==== controls (latched at vsync; slewed where noted) ====
    signal s_solid  : std_logic := '0';   -- S7
    signal s_bgvid  : std_logic := '0';   -- S8
    signal s_freeze : std_logic := '0';   -- S9
    signal s_k4     : unsigned(9 downto 0) := (others => '0');
    signal s_k5     : unsigned(9 downto 0) := C_MID;

    -- slew accumulators, Q10.4
    signal s_a1  : unsigned(13 downto 0) := to_unsigned(300*16, 14);
    signal s_a2  : unsigned(13 downto 0) := to_unsigned(512*16, 14);
    signal s_a3  : unsigned(13 downto 0) := to_unsigned(250*16, 14);
    signal s_a6  : unsigned(13 downto 0) := to_unsigned(146*16, 14);
    signal s_a12 : unsigned(13 downto 0) := (others => '0');
    signal s_k1s, s_k2s, s_k3s, s_k6s, s_p12s : unsigned(9 downto 0) := (others => '0');

    -- motion phases (Q10.6 angle; hue/flow Q16 = one hue cycle)
    signal s_pht1 : unsigned(15 downto 0) := (others => '0');
    signal s_phf  : unsigned(15 downto 0) := to_unsigned(9000, 16);
    signal s_phc  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal s_inc1 : unsigned(10 downto 0) := to_unsigned(64, 11);
    signal s_flow : signed(11 downto 0) := (others => '0');
    signal s_wigon : std_logic := '1';
    signal s_fad   : unsigned(9 downto 0) := (others => '0');

    -- ==== per-frame parameters (written by the vblank sequencer) ====
    signal s_cyc   : unsigned(10 downto 0) := to_unsigned(255, 11);
    signal s_H31   : unsigned(12 downto 0) := to_unsigned(150, 13);  -- H_q2 + 31
    signal s_Pq2   : unsigned(13 downto 0) := to_unsigned(2160, 14);
    signal s_W     : unsigned(11 downto 0) := (others => '0');   -- wiggle amp, q2
    signal s_Amax  : unsigned(12 downto 0) := to_unsigned(1000, 13); -- on-screen cap
    signal s_Ap    : unsigned(12 downto 0) := (others => '0');   -- meander from P12
    signal s_amp1  : unsigned(11 downto 0) := (others => '0');   -- primary amp, q2
    signal s_k1inc : unsigned(11 downto 0) := to_unsigned(34, 12);
    signal s_k2inc : unsigned(11 downto 0) := to_unsigned(80, 12);
    signal s_kp    : unsigned(12 downto 0) := (others => '0');   -- k from P12
    signal s_hscale : unsigned(11 downto 0) := to_unsigned(116, 12); -- hue/q2-row
    signal s_fneg  : std_logic := '0';


    -- shared sequential multiplier + divider (all inside blanking)
    signal s_op   : integer range 0 to 31 := 31;
    signal s_bit  : integer range 0 to 24 := 0;
    signal s_mA   : unsigned(21 downto 0) := (others => '0');
    signal s_mB   : unsigned(10 downto 0) := (others => '0');
    signal s_acc  : unsigned(21 downto 0) := (others => '0');
    signal s_dn   : unsigned(23 downto 0) := (others => '0');    -- divide numerator
    signal s_dr   : unsigned(14 downto 0) := (others => '0');    -- remainder
    signal s_dq   : unsigned(23 downto 0) := (others => '0');    -- quotient
    -- divisor is REGISTERED per divide: feeding a 3-way mux straight into
    -- the 15-bit borrow chain made this vblank-only divider the design's
    -- critical path (nextpnr times it at the full pixel clock)
    signal s_dvsr : unsigned(14 downto 0) := to_unsigned(1, 15);

    -- ==== scanner (monotone left-to-right, one column per clock) ====
    signal s_run    : std_logic := '0';
    signal s_Rq2    : unsigned(13 downto 0) := (others => '0');
    signal s_nrow    : unsigned(11 downto 0) := (others => '0');
    signal s_trig_ok : std_logic := '0';
    signal s_vb_ok   : std_logic := '0';
    signal s_rq2n    : unsigned(13 downto 0) := (others => '0');
    signal s_rq2m    : unsigned(13 downto 0) := (others => '0');
    signal s_col    : unsigned(11 downto 0) := (others => '0');
    signal s_wcol   : unsigned(11 downto 0) := (others => '0');
    signal s_th1, s_th2, s_the : unsigned(15 downto 0) := (others => '0');


    signal s_v : std_logic_vector(1 to 18) := (others => '0');

    -- m1: quarter-wave fold -- both meander terms (idx and idx+1, so both
    -- are interpolated) plus the end-anchor envelope
    signal m1_if, m1_ic, m1_i1, m1_i2 : unsigned(7 downto 0) := (others => '0');
    signal m1_ie, m1_ie2 : unsigned(7 downto 0) := (others => '0');
    signal m1_sf, m1_sc, m1_s1, m1_s2 : std_logic := '0';
    signal m1_fr, m1_fr2, m1_fre : unsigned(5 downto 0) := (others => '0');
    -- m2: ROM data
    signal m2_qf, m2_qc, m2_q1, m2_q2 : unsigned(11 downto 0) := (others => '0');
    signal m2_qe, m2_qe2 : unsigned(11 downto 0) := (others => '0');
    signal m2_sf, m2_sc, m2_s1, m2_s2 : std_logic := '0';
    signal m2_fr, m2_fr2, m2_fre : unsigned(5 downto 0) := (others => '0');
    -- m2b: registered signed ROM data + neighbour deltas
    signal m2b_s0, m2b_g0 : signed(12 downto 0) := (others => '0');
    signal m2b_sd, m2b_gd : signed(7 downto 0) := (others => '0');
    signal m2b_fr, m2b_fr2 : unsigned(5 downto 0) := (others => '0');
    signal m2b_e0  : unsigned(11 downto 0) := (others => '0');
    signal m2b_ed  : signed(8 downto 0) := (others => '0');
    signal m2b_fre : unsigned(5 downto 0) := (others => '0');
    -- m3: interpolated sines; m3b: lobes summed + envelope applied to the
    -- amplitude (the envelope is what pins both ends of the ribbon)
    signal m3_sf, m3_sg : signed(12 downto 0) := (others => '0');
    signal m3_env : unsigned(11 downto 0) := (others => '0');
    signal m3_sum : signed(13 downto 0) := (others => '0');
    signal m3b_ahi : unsigned(17 downto 0) := (others => '0');
    signal m3b_alo : unsigned(17 downto 0) := (others => '0');
    signal m3b_sum : signed(13 downto 0) := (others => '0');
    -- m4: enveloped amplitude, then the amplitude x meander multiply
    signal m4_aeff : unsigned(11 downto 0) := (others => '0');
    signal m4_sum  : signed(13 downto 0) := (others => '0');
    signal m4_hi  : signed(19 downto 0) := (others => '0');
    signal m4_lo  : signed(20 downto 0) := (others => '0');
    signal m4_p1  : signed(26 downto 0) := (others => '0');
    -- m5: meander height for this column.  The slope is taken from the
    -- FULL-PRECISION product difference (12 bits finer than the rounded
    -- height), which is what makes a smooth edge possible.
    signal m5_y   : signed(15 downto 0) := (others => '0');
    -- only the top bits of the product matter for the slope (the low 8 can
    -- shift the answer by at most one Q7 step), so the difference is taken
    -- on 19 bits and split across two stages -- as one 27-bit
    -- subtract-then-negate it was the critical path
    signal s_p1prev : signed(18 downto 0) := (others => '0');
    signal m5_d   : signed(18 downto 0) := (others => '0');
    signal m5_dp  : unsigned(18 downto 0) := (others => '0');  -- prev |slope|
    signal m5_dv0 : std_logic := '0';
    signal m5_idx : unsigned(10 downto 0) := (others => '0');   -- slope, Q7
    -- m6: vertical distance, signed offset (hue), slope index
    signal m6_dv  : unsigned(14 downto 0) := (others => '0');
    signal m6_off : signed(11 downto 0) := (others => '0');
    signal m6_i   : unsigned(3 downto 0) := (others => '0');    -- table entry
    signal m6_fr  : unsigned(3 downto 0) := (others => '0');    -- lerp fraction
    -- m6b: table entry; m6c: the interpolated correction factor
    signal m6b_v  : unsigned(11 downto 0) := (others => '0');
    signal m6b_d  : signed(9 downto 0) := (others => '0');
    signal m6b_fr : unsigned(3 downto 0) := (others => '0');
    signal m6b_dv : unsigned(11 downto 0) := (others => '0');
    signal m6b_off : signed(11 downto 0) := (others => '0');
    signal m6c_rf : unsigned(11 downto 0) := (others => '0');
    signal m6c_dv : unsigned(11 downto 0) := (others => '0');
    signal m6c_off : signed(11 downto 0) := (others => '0');
    -- m7: distance x correction (split), m8: perpendicular distance
    signal m7_ph, m7_pl : unsigned(17 downto 0) := (others => '0');
    signal m7_off : signed(11 downto 0) := (others => '0');
    signal m8_dp  : unsigned(12 downto 0) := (others => '0');
    signal m8_off : signed(11 downto 0) := (others => '0');
    -- m8b: the SIGNED perpendicular offset, registered before it feeds the
    -- hue multiply (a negate feeding a multiply in one stage was the path)
    signal m8b_po : signed(13 downto 0) := (others => '0');
    signal m8b_dp : unsigned(12 downto 0) := (others => '0');
    -- m9: alpha + hue partials, m10: recombined hue
    signal m9_a   : unsigned(5 downto 0) := (others => '0');
    signal m9_wr  : std_logic := '0';
    signal m9_hhi, m9_hlo : signed(20 downto 0) := (others => '0');
    signal m10_a  : unsigned(5 downto 0) := (others => '0');
    signal m10_wr : std_logic := '0';
    signal m10_hp : unsigned(15 downto 0) := (others => '0');

    signal s_st_we   : std_logic := '0';
    signal s_st_addr : unsigned(10 downto 0) := (others => '0');
    signal s_st_data : std_logic_vector(15 downto 0) := (others => '0');


    -- ==== line buffer: ONE bank, 2048 x 16, one writer, one reader ====
    -- 16 LIVE bits (hue10 & alpha6) so the word is a power of two wide: a
    -- 14-bit word at 2048 deep forced a 7-block 2048x2 EBR mapping, and a
    -- word split across an odd number of blocks is the documented
    -- posterise-on-hardware-while-sim-is-clean trap.
    type t_lb is array(0 to 2047) of std_logic_vector(15 downto 0);
    signal s_lb     : t_lb := (others => (others => '0'));
    signal s_rd     : std_logic_vector(15 downto 0) := (others => '0');
    signal s_raddr  : unsigned(10 downto 0) := (others => '0');
    signal s_rar    : unsigned(10 downto 0) := (others => '0');

    -- ==== beam pipeline (color from the packed phosphor LUT) ====
    signal b2_addr : unsigned(8 downto 0) := (others => '0');
    signal b2_a    : unsigned(5 downto 0) := (others => '0');
    signal b3_rom  : std_logic_vector(31 downto 0) := (others => '0');
    signal b3_a    : unsigned(5 downto 0) := (others => '0');
    signal b5_y, b5_u, b5_v : unsigned(9 downto 0) := (others => '0');
    signal b5_a    : unsigned(5 downto 0) := (others => '0');
    signal b6_dy, b6_du, b6_dv : signed(11 downto 0) := (others => '0');
    signal b6_g    : unsigned(6 downto 0) := (others => '0');
    signal b7_py, b7_pu, b7_pv : signed(19 downto 0) := (others => '0');

    -- ==== sync / video delay (u carries Cr on this hardware; kept verbatim) ====
    type t_dsr is array(1 to C_LAT - 1) of unsigned(9 downto 0);
    type t_bsr is array(1 to C_LAT - 1) of std_logic;
    signal d_y, d_u, d_v : t_dsr := (others => (others => '0'));
    signal d_hs, d_vs, d_fl, d_av : t_bsr := (others => '0');

    signal s_io : t_video_stream_yuv444_30b
        := ((others => '0'), (others => '0'), (others => '0'), '0', '1', '1', '1');

begin

    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    -- ====================================================================
    -- Raster measurement + per-frame class constants
    -- ====================================================================
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_hcnt > 0 then s_meas_h <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            elsif s_timing.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            end if;

            if s_timing.vsync_start = '1' then
                if s_vcnt > 0 then s_meas_v <= s_vcnt; end if;
                s_vcnt  <= (others => '0');
                s_ilace <= s_timing.is_interlaced;
                s_botf  <= not s_timing.field_n;      -- bottom field = field_n '0'

                if s_timing.is_interlaced = '1' then
                    s_fh <= s_meas_v(10 downto 0) & '0';
                else
                    s_fh <= s_meas_v;
                end if;
                if s_meas_h >= to_unsigned(1600, 12) then       -- 1080p / 1080i
                    s_cols <= to_unsigned(1920, 11); s_marg <= to_unsigned(64, 7);
                    s_cap  <= to_unsigned(2180, 12);
                    s_kwav <= to_unsigned(34, 12);
                    s_kenv <= to_unsigned(17, 12);
                elsif s_meas_h >= to_unsigned(1000, 12) then    -- 720p
                    s_cols <= to_unsigned(1280, 11); s_marg <= to_unsigned(64, 7);
                    s_cap  <= to_unsigned(1630, 12);
                    s_kwav <= to_unsigned(51, 12);
                    s_kenv <= to_unsigned(26, 12);
                else                                -- SD: quarter-pixel columns
                    s_cols <= to_unsigned(720, 11);  s_marg <= to_unsigned(16, 7);
                    s_cap  <= to_unsigned(830, 12);
                    s_kwav <= to_unsigned(91, 12);
                    s_kenv <= to_unsigned(46, 12);
                end if;
            elsif s_timing.avid_start = '1' then
                s_vcnt <= s_vcnt + 1;
            end if;
        end if;
    end process p_measure;

    -- ====================================================================
    -- Per-frame control latch, slews, motion phases
    -- ====================================================================
    p_frame : process(clk)
        variable v_fd  : signed(11 downto 0);
        variable v_fa  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_solid  <= registers_in(6)(0);
                s_bgvid  <= registers_in(6)(1);
                s_freeze <= registers_in(6)(2);
                s_k4 <= unsigned(registers_in(3));
                s_k5 <= unsigned(registers_in(4));

                -- phase clocks: wiggle/fold gated by K3 > 0, all by Freeze
                if s_freeze = '0' and s_wigon = '1' then
                    s_pht1 <= s_pht1 + resize(s_inc1, 16);
                    s_phf  <= s_phf  + resize(shift_right(s_inc1, 3), 16);
                end if;
                if s_freeze = '0' then
                    s_phc <= unsigned(signed(s_phc) + resize(s_flow, 16));
                end if;
            end if;

            s_k1s  <= s_a1(13 downto 4);
            s_k2s  <= s_a2(13 downto 4);
            s_k3s  <= s_a3(13 downto 4);
            s_k6s  <= s_a6(13 downto 4);
            s_p12s <= s_a12(13 downto 4);
            if s_a3 > to_unsigned(128, 14) then s_wigon <= '1';
            else                                s_wigon <= '0';
            end if;

            s_cyc <= to_unsigned(128, 11)
                     + resize(shift_right(resize(s_k6s * 7, 13), 3), 11);
            -- vertical headroom for the meander: half the screen minus the
            -- ribbon's own half-thickness and a margin, all in q2 rows
            if resize(s_fh & '0', 13) > resize(s_H31, 13) + 96 then
                s_Amax <= resize(s_fh & '0', 13) - resize(s_H31, 13) - 96;
            else
                s_Amax <= (others => '0');
            end if;
            -- kwav * 6: at full Coil this gives ~6 waves across the
            -- screen instead of ~2.7, so the ribbon bunches up much more
            s_kw7 <= resize(s_kwav & "00", 15);

            -- color flow: center deadband +-48 (square law in the sequencer)
            v_fd := resize(signed('0' & s_k5), 12) - to_signed(512, 12);
            if v_fd < 0 then v_fa := resize(unsigned(-v_fd), 11); s_fneg <= '1';
            else             v_fa := resize(unsigned(v_fd), 11);  s_fneg <= '0'; end if;
            if v_fa > to_unsigned(48, 11) then
                s_fad <= resize(v_fa - 48, 10);
            else
                s_fad <= (others => '0');
            end if;
        end if;
    end process p_frame;

    -- ====================================================================
    -- Per-frame parameter sequencer: 5 knob slews, then 11 products on ONE
    -- shared shift-add multiplier (12 cycles each), then the step-tier
    -- thresholds.  ~150 cycles, all inside the blanking interval.
    -- ====================================================================
    p_seq : process(clk)
        variable v_t   : unsigned(13 downto 0);
        variable v_d   : signed(14 downto 0);
        variable v_n   : signed(15 downto 0);
        variable v_dr  : unsigned(14 downto 0);
        variable v_a, v_cl : unsigned(13 downto 0);
        variable v_k   : unsigned(12 downto 0);
        variable v_g   : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_op  <= 0;
                s_bit <= 0;
            elsif s_timing.avid = '0' then
                case s_op is
                    -- 1-pole slews (Q10.4): acc += (target - acc) / 16.
                    -- 16-bit intermediate, then SLICE: a signed resize here
                    -- mangles values >= 8192 (the v1.0 upper-half-range bug)
                    when 0 =>
                        v_t := unsigned(registers_in(0)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a1);
                        v_n := resize(signed('0' & s_a1), 16) + resize(shift_right(v_d, 4), 16);
                        s_a1 <= unsigned(v_n(13 downto 0));
                        s_op <= 1;
                    when 1 =>
                        v_t := unsigned(registers_in(1)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a2);
                        v_n := resize(signed('0' & s_a2), 16) + resize(shift_right(v_d, 4), 16);
                        s_a2 <= unsigned(v_n(13 downto 0));
                        s_op <= 2;
                    when 2 =>
                        v_t := unsigned(registers_in(2)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a3);
                        v_n := resize(signed('0' & s_a3), 16) + resize(shift_right(v_d, 4), 16);
                        s_a3 <= unsigned(v_n(13 downto 0));
                        s_op <= 3;
                    when 3 =>
                        v_t := unsigned(registers_in(5)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a6);
                        v_n := resize(signed('0' & s_a6), 16) + resize(shift_right(v_d, 4), 16);
                        s_a6 <= unsigned(v_n(13 downto 0));
                        s_op <= 4;
                    when 4 =>
                        v_t := unsigned(registers_in(7)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a12);
                        v_n := resize(signed('0' & s_a12), 16) + resize(shift_right(v_d, 4), 16);
                        s_a12 <= unsigned(v_n(13 downto 0));
                        s_op <= 5;

                    -- shared multiplier ops (12-bit mB, load at 0, store one
                    -- cycle AFTER the last partial lands -- storing at the
                    -- last add drops bit 10, e.g. any multiply by 1024)
                    -- shared multiplier ops (store one cycle AFTER the
                    -- last partial lands -- storing at the last add drops
                    -- bit 10, e.g. any multiply by exactly 1024)
                    when 5 to 12 =>
                        if s_bit = 0 then
                            s_acc <= (others => '0');
                            case s_op is
                                when 5      => s_mA <= resize(s_fh, 22);   s_mB <= resize(s_k1s, 11);
                                when 6      => s_mA <= resize(s_fh, 22);   s_mB <= resize(s_k2s, 11);
                                when 7      => s_mA <= resize(s_fh, 22);   s_mB <= resize(s_k3s, 11);
                                when 8      => s_mA <= resize(s_k4, 22);   s_mB <= resize(s_k4, 11);
                                when 9      => s_mA <= resize(s_fad, 22);  s_mB <= resize(s_fad, 11);
                                when 10     => s_mA <= resize(s_Amax, 22); s_mB <= resize(s_p12s, 11);
                                when 11     => s_mA <= resize(s_kw7, 22);  s_mB <= resize(s_p12s, 11);
                                -- H x Coil: the ribbon thins ~25% as it
                                -- stretches.  Past a point the curve bends
                                -- tighter than the ribbon is thick, which no
                                -- constant-width non-overlapping ribbon can
                                -- represent; thinning keeps the extreme clean.
                                when others => s_mA <= resize(s_H31, 22);  s_mB <= resize(s_p12s, 11);
                            end case;
                            s_bit <= 1;
                        else
                            if s_mB(0) = '1' then
                                s_acc <= s_acc + s_mA;
                            end if;
                            s_mA <= s_mA(20 downto 0) & '0';
                            s_mB <= '0' & s_mB(10 downto 1);
                            if s_bit = 12 then
                                s_bit <= 0;
                                s_op  <= s_op + 1;
                                case s_op is
                                    -- height at 3/4 scale (max ~40% of screen)
                                    when 5      => s_H31 <= to_unsigned(43, 13)
                                                            + resize(s_acc(21 downto 10), 13)
                                                            - resize(s_acc(21 downto 12), 13);
                                    when 6      => s_Pq2 <= resize(s_acc(21 downto 8), 14);
                                    when 7      => s_W   <= resize(s_acc(21 downto 11), 12);
                                    when 8      => s_inc1 <= to_unsigned(64, 11)
                                                            + resize(s_acc(19 downto 10), 11);
                                    when 9      => if s_fneg = '1' then
                                                       s_flow <= -signed('0' & s_acc(19 downto 9));
                                                   else
                                                       s_flow <=  signed('0' & s_acc(19 downto 9));
                                                   end if;
                                    when 10     => s_Ap <= resize(s_acc(21 downto 10), 13);
                                    when 11     => s_kp <= resize(s_acc(21 downto 10), 13);
                                    when others => s_H31 <= s_H31
                                                            - resize(s_acc(21 downto 12), 13);
                                end case;
                            else
                                s_bit <= s_bit + 1;
                            end if;
                        end if;

                    -- combine: amplitude (clamped so the ribbon always stays
                    -- on screen) and meander wavenumber
                    when 13 =>
                        v_a := resize(s_Ap, 14) + resize(s_W, 14);
                        v_cl := resize(shift_right(s_Amax, 1), 14)
                                + resize(shift_right(s_Amax, 2), 14);   -- 0.75*Amax
                        if v_a > v_cl then v_a := v_cl; end if;
                        s_amp1 <= resize(v_a, 12);
                        s_op <= 14;
                    when 14 =>
                        -- 0.5 wave across the screen at Coil 0, ~4 at full
                        v_k := resize(shift_right(s_kwav, 2), 13) + resize(s_kp, 13);
                        s_k1inc <= resize(v_k, 12);
                        -- second term at 2.375x: non-harmonic, so the shape
                        -- never looks like a plain sine
                        s_k2inc <= resize(v_k + shift_right(v_k, 1), 12);
                        s_op <= 21;

                    -- hue across thickness: hscale = cyc*128 / H31
                    when 21 =>
                        s_dn  <= resize(s_cyc & "0000000", 24);
                        s_dvsr <= resize(s_H31, 15);
                        s_dr  <= (others => '0');
                        s_dq  <= (others => '0');
                        s_bit <= 0;
                        s_op  <= 22;
                    when 22 =>
                        v_dr := s_dr(13 downto 0) & s_dn(23);
                        s_dn <= s_dn(22 downto 0) & '0';
                        if v_dr >= s_dvsr then
                            s_dr <= v_dr - s_dvsr;
                            s_dq <= s_dq(22 downto 0) & '1';
                        else
                            s_dr <= v_dr;
                            s_dq <= s_dq(22 downto 0) & '0';
                        end if;
                        if s_bit = 23 then
                            s_bit <= 0;
                            s_op  <= 23;
                        else
                            s_bit <= s_bit + 1;
                        end if;
                    when 23 =>
                        if s_dq > to_unsigned(4095, 24) then
                            s_hscale <= to_unsigned(4095, 12);
                        else
                            s_hscale <= resize(s_dq, 12);
                        end if;
                        s_op <= 31;
                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process p_seq;

    -- ====================================================================
    -- Marcher: trigger + adaptive-step curve pipeline + stamp writer
    -- ====================================================================
    -- The scan never stalls: exactly one column per clock, so there are no
    -- gaps to fill and no overlap to arbitrate.

    p_march : process(clk)
        variable v_ph   : unsigned(9 downto 0);
        variable v_s0, v_s1 : signed(12 downto 0);
        variable v_yn   : signed(15 downto 0);
        variable v_dv   : signed(15 downto 0);
        variable v_du   : unsigned(18 downto 0);
        variable v_al   : signed(13 downto 0);
        variable v_dp   : unsigned(25 downto 0);
        variable v_rf13 : signed(12 downto 0);
        variable v_env13 : signed(12 downto 0);
        variable v_hpp  : signed(25 downto 0);
        variable v_h16  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            s_st_we <= '0';

            -- registered next-row helpers (fed by a stable s_vcnt)
            s_nrow <= s_vcnt + 1;
            if (s_vcnt + 1) < s_meas_v then s_trig_ok <= '1';
            else                            s_trig_ok <= '0'; end if;
            if s_vcnt >= s_meas_v then s_vb_ok <= '1';
            else                       s_vb_ok <= '0'; end if;
            if s_ilace = '1' then
                if s_botf = '1' then
                    s_rq2n <= resize(s_nrow(10 downto 0) & "000", 14) + 4;
                else
                    s_rq2n <= resize(s_nrow(10 downto 0) & "000", 14);
                end if;
            else
                s_rq2n <= resize(s_nrow(11 downto 0) & "00", 14);
            end if;
            s_rq2m <= s_Rq2;

            -- ---- triggers ----
            -- A MONOTONE left-to-right sweep, one column per clock, every
            -- column exactly once: y is single-valued in x, so the ribbon
            -- provably cannot cross itself and nothing can be skipped.
            if s_run = '0' and s_timing.avid_start = '1' and s_trig_ok = '1' then
                s_Rq2  <= s_rq2n;
                s_run  <= '1';
                s_col  <= (others => '0');
                s_wcol <= (others => '0');
                s_th1  <= s_pht1;
                s_th2  <= s_phf;
                s_the  <= (others => '0');
                s_v    <= (others => '0');
                m9_wr  <= '0';
                m10_wr <= '0';
            elsif s_run = '0' and s_timing.hsync_start = '1'
                  and s_vb_ok = '1' then
                if s_ilace = '1' and s_botf = '1' then
                    s_Rq2 <= to_unsigned(4, 14);
                else
                    s_Rq2 <= (others => '0');
                end if;
                s_run  <= '1';
                s_col  <= (others => '0');
                s_wcol <= (others => '0');
                s_th1  <= s_pht1;
                s_th2  <= s_phf;
                s_the  <= (others => '0');
                s_v    <= (others => '0');
                m9_wr  <= '0';
                m10_wr <= '0';
            elsif s_run = '1' then
                s_col <= s_col + 1;
                s_th1 <= s_th1 + resize(s_k1inc, 16);
                s_th2 <= s_th2 + resize(s_k2inc, 16);
                s_the <= s_the + resize(s_kenv, 16);   -- half a cycle edge to edge
                s_v(1) <= '1';
                s_v(2 to 18) <= s_v(1 to 17);
                if s_col >= resize(s_cols, 12) + 28 then
                    s_run <= '0';
                    s_v   <= (others => '0');
                end if;
            end if;

            -- ---- ribbon pipeline ----
            if s_run = '1' then
                -- m1: quarter-wave fold for both lobes and the envelope
                v_ph := s_th1(15 downto 6);
                m1_sf <= v_ph(9);
                if v_ph(8) = '1' then m1_if <= not v_ph(7 downto 0);
                else                  m1_if <= v_ph(7 downto 0); end if;
                v_ph := s_th1(15 downto 6) + 1;
                m1_sc <= v_ph(9);
                if v_ph(8) = '1' then m1_ic <= not v_ph(7 downto 0);
                else                  m1_ic <= v_ph(7 downto 0); end if;
                m1_fr <= s_th1(5 downto 0);
                v_ph := s_th2(15 downto 6);
                m1_s1 <= v_ph(9);
                if v_ph(8) = '1' then m1_i1 <= not v_ph(7 downto 0);
                else                  m1_i1 <= v_ph(7 downto 0); end if;
                v_ph := s_th2(15 downto 6) + 1;
                m1_s2 <= v_ph(9);
                if v_ph(8) = '1' then m1_i2 <= not v_ph(7 downto 0);
                else                  m1_i2 <= v_ph(7 downto 0); end if;
                m1_fr2 <= s_th2(5 downto 0);
                -- Envelope: exactly half a sine cycle across the screen, so
                -- it is 0 at both edges and 1 mid-screen.  Read at idx AND
                -- idx+1: its index only advances ~0.53 entries per column,
                -- so an un-interpolated envelope steps on ALTERNATE COLUMNS,
                -- and since it scales the amplitude that showed up as a
                -- ~2.5-row chop at high Coil.
                v_ph := s_the(15 downto 6);
                if v_ph(8) = '1' then m1_ie <= not v_ph(7 downto 0);
                else                  m1_ie <= v_ph(7 downto 0); end if;
                v_ph := s_the(15 downto 6) + 1;
                if v_ph(8) = '1' then m1_ie2 <= not v_ph(7 downto 0);
                else                  m1_ie2 <= v_ph(7 downto 0); end if;
                m1_fre <= s_the(5 downto 0);

                -- m2: ROM data settles in the p_rom* processes
                m2_sf <= m1_sf;  m2_sc <= m1_sc;
                m2_s1 <= m1_s1;  m2_s2 <= m1_s2;
                m2_fr <= m1_fr;  m2_fr2 <= m1_fr2;  m2_fre <= m1_fre;

                -- m2b: signs + neighbour deltas (EBR data into fabric FFs
                -- before any arithmetic)
                if m2_sf = '1' then v_s0 := -signed('0' & m2_qf);
                else                v_s0 :=  signed('0' & m2_qf); end if;
                if m2_sc = '1' then v_s1 := -signed('0' & m2_qc);
                else                v_s1 :=  signed('0' & m2_qc); end if;
                m2b_s0 <= v_s0;
                m2b_sd <= resize(v_s1 - v_s0, 8);
                m2b_fr <= m2_fr;
                if m2_s1 = '1' then v_s0 := -signed('0' & m2_q1);
                else                v_s0 :=  signed('0' & m2_q1); end if;
                if m2_s2 = '1' then v_s1 := -signed('0' & m2_q2);
                else                v_s1 :=  signed('0' & m2_q2); end if;
                m2b_g0 <= v_s0;
                m2b_gd <= resize(v_s1 - v_s0, 8);
                m2b_fr2 <= m2_fr2;
                m2b_e0  <= m2_qe;
                m2b_ed  <= resize(signed('0' & m2_qe2) - signed('0' & m2_qe), 9);
                m2b_fre <= m2_fre;

                -- m3: interpolate both lobes
                m3_sf <= m2b_s0 + resize(shift_right(m2b_sd * signed('0' & m2b_fr), 6), 13);
                m3_sg <= m2b_g0 + resize(shift_right(m2b_gd * signed('0' & m2b_fr2), 6), 13);
                v_env13 := signed('0' & m2b_e0)
                           + resize(shift_right(m2b_ed * signed('0' & m2b_fre), 6), 13);
                if v_env13 < 0 then
                    m3_env <= (others => '0');
                else
                    m3_env <= unsigned(v_env13(11 downto 0));
                end if;

                -- m3b: fold the lobes together (the second is exactly A/8 --
                -- at A/4 its slope contribution notched the peaks once the
                -- wave count went up)
                -- and start the envelope x amplitude multiply
                m3_sum <= resize(m3_sf, 14) + resize(shift_right(m3_sg, 3), 14);
                m3b_ahi <= s_amp1 * m3_env(11 downto 6);
                m3b_alo <= s_amp1 * m3_env(5 downto 0);
                m3b_sum <= m3_sum;

                -- m4a: the enveloped amplitude -- zero at both screen edges,
                -- so the ribbon's ends stay put and all the added length
                -- goes into the middle
                m4_aeff <= resize(shift_right(shift_left(resize(m3b_ahi, 24), 6)
                                              + resize(m3b_alo, 24), 12), 12);
                m4_sum  <= m3b_sum;

                -- m4b: amplitude x meander, split hi/lo
                m4_hi <= signed('0' & m4_aeff) * m4_sum(13 downto 7);
                m4_lo <= signed('0' & m4_aeff) * signed('0' & m4_sum(6 downto 0));

                -- m4c: recombine
                m4_p1 <= shift_left(resize(m4_hi, 27), 7) + resize(m4_lo, 27);

                -- m5: the ribbon's centre height at this column
                v_yn := signed('0' & resize(s_Pq2, 15))
                        + resize(shift_right(m4_p1, 12), 16);
                m5_y  <= v_yn;
                -- slope straight from the un-rounded product: 12 bits finer
                -- than differencing the quarter-row heights, which is what
                -- made the edges tooth
                s_p1prev <= m4_p1(25 downto 7);
                m5_d   <= m4_p1(25 downto 7) - s_p1prev;
                m5_dv0 <= s_v(10);      -- predecessor valid?

                -- m5b: magnitude, averaged over two columns.  The width
                -- correction is a steep function of slope, so un-smoothed
                -- slope noise near a crest turns into a comb on the edge.
                if m5_d < 0 then v_du := unsigned(-m5_d);
                else             v_du := unsigned(m5_d); end if;
                m5_dp <= v_du;
                v_du := shift_right(v_du, 1) + shift_right(m5_dp, 1);
                -- one column == one pixel on every raster now, so the slope
                -- scale is the same for all classes
                if m5_dv0 = '0' then
                    m5_idx <= (others => '0');   -- no predecessor yet
                elsif v_du > 2047 then
                    m5_idx <= to_unsigned(2047, 11);
                else
                    m5_idx <= resize(v_du, 11);
                end if;

                -- m6: vertical distance, hue offset, and the local slope
                -- (one column apart, so this IS dy/dx)
                v_dv := m5_y - signed('0' & resize(s_rq2m, 15));
                if v_dv < 0 then m6_dv <= resize(unsigned(-v_dv), 15);
                else             m6_dv <= resize(unsigned(v_dv), 15); end if;
                m6_off <= v_dv(11 downto 0);
                if m5_idx(10 downto 4) > 15 then
                    m6_i <= to_unsigned(15, 4);   -- past the clamp: flat
                    m6_fr <= (others => '0');
                else
                    m6_i  <= m5_idx(7 downto 4);
                    m6_fr <= m5_idx(3 downto 0);
                end if;

                -- m6b: table entry arrives (read in p_romrf)
                if m6_dv > 4095 then m6b_dv <= to_unsigned(4095, 12);
                else                 m6b_dv <= resize(m6_dv, 12); end if;
                m6b_off <= m6_off;
                m6b_fr  <= m6_fr;

                -- m6c: interpolate between entries -- without this each table
                -- step jumps the ribbon edge ~3 rows and the edges tooth
                v_rf13 := signed('0' & m6b_v)
                          + resize(shift_right(m6b_d * signed('0' & m6b_fr), 4), 13);
                -- unity is 4096, which does NOT fit the 12-bit field: left
                -- unclamped it wrapped to zero on every flat spot and lit
                -- the whole column
                if v_rf13 > 4095 then
                    m6c_rf <= to_unsigned(4095, 12);
                else
                    m6c_rf <= unsigned(v_rf13(11 downto 0));
                end if;
                m6c_dv <= m6b_dv;
                m6c_off <= m6b_off;

                -- m7: distance x correction, split into two small multiplies
                m7_ph <= m6c_dv * m6c_rf(11 downto 6);
                m7_pl <= m6c_dv * m6c_rf(5 downto 0);
                m7_off <= m6c_off;

                -- m8: distance PERPENDICULAR to the curve -- this is what
                -- keeps the ribbon one constant width through every bend
                v_dp := shift_left(resize(m7_ph, 26), 6) + resize(m7_pl, 26);
                m8_dp <= resize(shift_right(v_dp, 12), 13);
                m8_off <= m7_off;

                -- m8b: sign the perpendicular offset (colour follows the
                -- ribbon), and carry the magnitude for alpha
                if m8_off < 0 then m8b_po <= -signed('0' & m8_dp);
                else               m8b_po <=  signed('0' & m8_dp); end if;
                m8b_dp <= m8_dp;

                -- m9: alpha from the perpendicular distance + hue partials
                v_al := signed('0' & resize(s_H31, 13)) - signed('0' & m8b_dp);
                if v_al <= 0 then
                    m9_a <= (others => '0');
                elsif v_al > 31 then
                    m9_a <= to_unsigned(63, 6);
                else
                    m9_a <= unsigned(v_al(4 downto 0)) & '0';
                end if;
                -- EVERY column is stamped now, alpha 0 included: that is what
                -- retires the clear-behind-beam write port and lets one bank
                -- serve both the beam and the stamp.
                if s_v(17) = '1' then m9_wr <= '1';
                else                  m9_wr <= '0'; end if;
                -- Colour is keyed to the signed PERPENDICULAR offset, so the
                -- rainbow bands stay square to the ribbon through every bend.
                m9_hhi <= m8b_po * signed('0' & s_hscale(11 downto 6));
                m9_hlo <= m8b_po * signed('0' & s_hscale(5 downto 0));

                -- m10: recombine the hue product
                v_hpp := shift_left(resize(m9_hhi, 26), 6) + resize(m9_hlo, 26);
                m10_hp <= unsigned(v_hpp(15 downto 0));
                m10_a  <= m9_a;
                m10_wr <= m9_wr;
            end if;

            -- ---- writer: one write per column, in order ----
            v_h16 := s_phc - m10_hp;     -- hue rises downward: R..V top-down
            if s_run = '1' and m10_wr = '1' and s_wcol < s_cols then
                s_st_we   <= '1';
                s_st_addr <= s_wcol(10 downto 0);
                s_st_data <= std_logic_vector(v_h16(15 downto 6))
                             & std_logic_vector(m10_a);
            end if;
            if s_run = '1' and s_v(18) = '1' then
                s_wcol <= s_wcol + 1;
            end if;
        end if;
    end process p_march;

    -- ==== meander sine ROMs: 4 reads (two terms, each at idx and idx+1
    -- so both are interpolated) -- one inferred EBR each ====
    p_romf : process(clk)
    begin
        if rising_edge(clk) then
            m2_qf <= C_QSIN13(to_integer(m1_if));
        end if;
    end process p_romf;

    p_romc : process(clk)
    begin
        if rising_edge(clk) then
            m2_qc <= C_QSIN13(to_integer(m1_ic));
        end if;
    end process p_romc;

    p_rom1 : process(clk)
    begin
        if rising_edge(clk) then
            m2_q1 <= C_QSIN13(to_integer(m1_i1));
        end if;
    end process p_rom1;

    p_rom2 : process(clk)
    begin
        if rising_edge(clk) then
            m2_q2 <= C_QSIN13(to_integer(m1_i2));
        end if;
    end process p_rom2;

    -- envelope sine, read at idx and idx+1 so it can be interpolated
    p_rome : process(clk)
    begin
        if rising_edge(clk) then
            m2_qe <= C_QSIN13(to_integer(m1_ie));
        end if;
    end process p_rome;

    p_rome2 : process(clk)
    begin
        if rising_edge(clk) then
            m2_qe2 <= C_QSIN13(to_integer(m1_ie2));
        end if;
    end process p_rome2;

    -- perpendicular-width correction factor
    p_romrf : process(clk)
    begin
        if rising_edge(clk) then
            m6b_v <= C_RFV(to_integer(m6_i));
            m6b_d <= C_RFD(to_integer(m6_i));
        end if;
    end process p_romrf;

    -- ==== ribbon color LUT (one read port -> one inferred ROM) ====
    p_clut : process(clk)
    begin
        if rising_edge(clk) then
            b3_rom <= C_CLUT(to_integer(b2_addr));
        end if;
    end process p_clut;

    -- ====================================================================
    -- Line buffer: ONE bank.  The stamp for row R+1 writes column c about
    -- 18 clocks AFTER the beam has read column c of row R, so a single
    -- buffer serves both -- the read of every column always precedes its
    -- overwrite by the depth of the marcher pipeline.  Because every column
    -- is stamped (alpha 0 outside the ribbon) there is nothing to clear, so
    -- the second write source is gone: one write statement, one read, and
    -- half the EBR.
    s_raddr <= s_hcnt(10 downto 0);

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_rar <= s_raddr;
            if s_st_we = '1' then
                s_lb(to_integer(s_st_addr)) <= s_st_data;
            end if;
            s_rd <= s_lb(to_integer(s_rar));
        end if;
    end process p_lb;

    -- ====================================================================
    -- Beam: readout, hue -> YUV, alpha composite (C_LAT = 9, all modes)
    -- ====================================================================
    p_beam : process(clk)
        variable v_w    : std_logic_vector(15 downto 0);
        variable v_hue  : unsigned(9 downto 0);
        variable v_m6   : unsigned(12 downto 0);
        variable v_k    : unsigned(2 downto 0);
        variable v_yr, v_ur, v_vr : unsigned(9 downto 0);
        variable v_by, v_bu, v_bv : unsigned(9 downto 0);
        variable v_g    : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- b2: decode word -> color LUT address.  Gradient reads the hue
            -- directly; Solid snaps to the segment start (k*73 = k*64+k*8+k),
            -- where the LUT holds the pure phosphor color.
            v_w := s_rd;
            b2_a <= unsigned(v_w(5 downto 0));
            v_hue := unsigned(v_w(15 downto 6));
            v_m6  := resize(v_hue & "00", 13) + resize(v_hue & '0', 13)
                     + resize(v_hue, 13);                    -- hue9 * 7
            if s_solid = '1' then
                v_k := v_m6(12 downto 10);
                b2_addr <= resize(v_k & "000000", 9) + resize(v_k & "000", 9)
                           + resize(v_k, 9);
            else
                b2_addr <= v_hue(9 downto 1);
            end if;

            -- b3: LUT read happens in p_clut

            -- b4 (named b5_*): ROM data into fabric FFs before any arithmetic
            b5_y <= unsigned(b3_rom(29 downto 20));
            b5_u <= unsigned(b3_rom(19 downto 10));
            b5_v <= unsigned(b3_rom(9 downto 0));
            b3_a  <= b2_a;
            b5_a  <= b3_a;

            -- b6: background select, differences, gain
            v_yr := b5_y;  v_ur := b5_u;  v_vr := b5_v;
            if s_bgvid = '1' then
                v_by := d_y(5);
                v_bu := d_u(5);
                v_bv := d_v(5);
            else
                v_by := (others => '0');
                v_bu := C_MID;
                v_bv := C_MID;
            end if;
            if d_av(5) = '0' then
                v_g := (others => '0');
            elsif b5_a = to_unsigned(63, 6) then
                v_g := to_unsigned(64, 7);
            else
                v_g := resize(b5_a, 7);
            end if;
            b6_dy <= resize(signed('0' & v_yr), 12) - resize(signed('0' & v_by), 12);
            b6_du <= resize(signed('0' & v_ur), 12) - resize(signed('0' & v_bu), 12);
            b6_dv <= resize(signed('0' & v_vr), 12) - resize(signed('0' & v_bv), 12);
            b6_g  <= v_g;

            -- b7: alpha products
            b7_py <= b6_dy * signed('0' & b6_g);
            b7_pu <= b6_du * signed('0' & b6_g);
            b7_pv <= b6_dv * signed('0' & b6_g);

            -- b8 -> s_io: composite over the SAME background sample, re-tapped
            -- from the delay line two stages later (convex blend: no clamp)
            if s_bgvid = '1' then
                v_by := d_y(7);
                v_bu := d_u(7);
                v_bv := d_v(7);
            else
                v_by := (others => '0');
                v_bu := C_MID;
                v_bv := C_MID;
            end if;
            s_io.y <= std_logic_vector(resize(unsigned(
                signed('0' & v_by) + resize(shift_right(b7_py, 6), 11)), 10));
            s_io.u <= std_logic_vector(resize(unsigned(
                signed('0' & v_bu) + resize(shift_right(b7_pu, 6), 11)), 10));
            s_io.v <= std_logic_vector(resize(unsigned(
                signed('0' & v_bv) + resize(shift_right(b7_pv, 6), 11)), 10));
            s_io.hsync_n <= d_hs(C_LAT - 1);
            s_io.vsync_n <= d_vs(C_LAT - 1);
            s_io.field_n <= d_fl(C_LAT - 1);
            s_io.avid    <= d_av(C_LAT - 1);
        end if;
    end process p_beam;

    -- ====================================================================
    -- Sync / video delay
    -- ====================================================================
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            d_y(1) <= unsigned(data_in.y);
            d_u(1) <= unsigned(data_in.u);
            d_v(1) <= unsigned(data_in.v);
            d_hs(1) <= data_in.hsync_n;
            d_vs(1) <= data_in.vsync_n;
            d_fl(1) <= data_in.field_n;
            d_av(1) <= data_in.avid;
            for i in 2 to C_LAT - 1 loop
                d_y(i)  <= d_y(i - 1);
                d_u(i)  <= d_u(i - 1);
                d_v(i)  <= d_v(i - 1);
                d_hs(i) <= d_hs(i - 1);
                d_vs(i) <= d_vs(i - 1);
                d_fl(i) <= d_fl(i - 1);
                d_av(i) <= d_av(i - 1);
            end loop;
        end if;
    end process p_delay;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture aurora;
