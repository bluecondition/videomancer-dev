-- spiroscope.vhd
--
-- Spiroscope v6 -- a full-resolution spirograph / harmonograph drawing machine.
--
-- The figure is a POLYLINE, regenerated whole every field during vertical
-- blanking, so every knob is live and continuous (no zones, no clear-and-
-- redraw, no eraser):
--
--   GENERATOR (vblank, 8 clocks/point): Spiro = 2-arm epicycle; Harmonograph
--     = decaying 2-pendulum Lissajous (Arm blends lateral -> rotary, pendulum
--     A detuned one turn per swing in y).  Frequencies are chosen so every
--     figure closes exactly on the ring.  Twist (K4) is the y-phase of arm B:
--     it morphs epi <-> hypo through tilted "3D" lattices.  Point RAM
--     4096x16 (16 EBR): 8192 points as two 4-bit-delta points per word for
--     thin crisp 1080p Spiro, else 4096 (2048 thick, 1024 SD) as 8-bit
--     deltas; chunk table every 32/16 points {x0, y0, y-extent (centre
--     offsets), age} (3 EBR).  Trace's top zone = whole figure, blooming,
--     its twist drifting.
--     Kaleido writes 8 D4 copies (mirrors / quarter turns) of the pen.  One
--     shared 10x10 multiplier; quarter-wave cosine ROM (10-bit angle).
--   RENDERER (one row ahead of the beam): a prescan culls chunks by y-extent;
--     the vertex scan integrates deltas and turns each segment within reach
--     of the row into a job {words, E0, |u|}: |E0 - k|u|| = 2*distance to the
--     segment's line in Q6 at pixel k (u, v = 2*direction/L from a ROM).  The
--     stamper writes 4-pixel words of a 4-bit closeness field (max = crisp,
--     saturating add = glow light) plus a 4-bit age per 4:2:2 pixel pair,
--     through a pipelined read-modify-write with 3-deep forwarding.  Two
--     line-buffer banks (row parity); the beam clears behind itself.
--   BEAM: weight threshold with half-pixel AA (Engrave: weight follows the
--     input luma), glow light curve with white-hot overlaps, age gradient
--     palettes and comet fade, black or live-video ground.  Chroma: one blend that
--     carries U on even pixels and V on odd ones (what the 4:2:2 packer keeps).
--
-- Contracts: processing type (Engrave reads input luma); generated avid
-- (cubist/aurora parity recipe); output gated to 64/512/512 outside it; one
-- latency (C_L) in every mode; field work keyed on "avid absent 4096
-- clocks", never on vsync; stroke luma capped (~750, glow cores ~900 white).
--
--   K1 Gear A (1-16)      S7  Machine: Spiro / Harmonograph
--   K2 Gear B (-16..16)   S8  Engrave (line weight follows the video)
--   K3 Arm                S9  Kaleido (8 mirrored pens)
--   K4 Twist              S10 Glow (additive light)
--   K5 Colour (8 ramps)   S11 Ground: Black / Video
--   K6 Weight (1-8 px)    P12 Trace: comet -> full figure -> bloom (drifts)
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture spiroscope of program_top is

    constant C_L : integer := 8;      -- S0 (beam column) -> data_out

    -- ==== ROM tables (proto/gen_tables.py) ====
    type t_rom9  is array(0 to 255) of unsigned(8 downto 0);
    type t_rom16 is array(0 to 255) of unsigned(15 downto 0);
    type t_pal16 is array(0 to 127) of unsigned(15 downto 0);
    type t_hrec  is array(0 to 15) of unsigned(16 downto 0);
    constant C_COSQ : t_rom9 := (
        to_unsigned(511, 9), to_unsigned(511, 9), to_unsigned(511, 9), to_unsigned(511, 9), to_unsigned(511, 9), to_unsigned(511, 9), to_unsigned(511, 9), to_unsigned(510, 9),
        to_unsigned(510, 9), to_unsigned(510, 9), to_unsigned(510, 9), to_unsigned(510, 9), to_unsigned(509, 9), to_unsigned(509, 9), to_unsigned(509, 9), to_unsigned(509, 9),
        to_unsigned(508, 9), to_unsigned(508, 9), to_unsigned(508, 9), to_unsigned(507, 9), to_unsigned(507, 9), to_unsigned(507, 9), to_unsigned(506, 9), to_unsigned(506, 9),
        to_unsigned(505, 9), to_unsigned(505, 9), to_unsigned(504, 9), to_unsigned(504, 9), to_unsigned(503, 9), to_unsigned(503, 9), to_unsigned(502, 9), to_unsigned(501, 9),
        to_unsigned(501, 9), to_unsigned(500, 9), to_unsigned(500, 9), to_unsigned(499, 9), to_unsigned(498, 9), to_unsigned(498, 9), to_unsigned(497, 9), to_unsigned(496, 9),
        to_unsigned(495, 9), to_unsigned(495, 9), to_unsigned(494, 9), to_unsigned(493, 9), to_unsigned(492, 9), to_unsigned(491, 9), to_unsigned(490, 9), to_unsigned(489, 9),
        to_unsigned(489, 9), to_unsigned(488, 9), to_unsigned(487, 9), to_unsigned(486, 9), to_unsigned(485, 9), to_unsigned(484, 9), to_unsigned(483, 9), to_unsigned(482, 9),
        to_unsigned(481, 9), to_unsigned(480, 9), to_unsigned(478, 9), to_unsigned(477, 9), to_unsigned(476, 9), to_unsigned(475, 9), to_unsigned(474, 9), to_unsigned(473, 9),
        to_unsigned(472, 9), to_unsigned(470, 9), to_unsigned(469, 9), to_unsigned(468, 9), to_unsigned(467, 9), to_unsigned(465, 9), to_unsigned(464, 9), to_unsigned(463, 9),
        to_unsigned(461, 9), to_unsigned(460, 9), to_unsigned(459, 9), to_unsigned(457, 9), to_unsigned(456, 9), to_unsigned(454, 9), to_unsigned(453, 9), to_unsigned(451, 9),
        to_unsigned(450, 9), to_unsigned(448, 9), to_unsigned(447, 9), to_unsigned(445, 9), to_unsigned(444, 9), to_unsigned(442, 9), to_unsigned(441, 9), to_unsigned(439, 9),
        to_unsigned(437, 9), to_unsigned(436, 9), to_unsigned(434, 9), to_unsigned(433, 9), to_unsigned(431, 9), to_unsigned(429, 9), to_unsigned(427, 9), to_unsigned(426, 9),
        to_unsigned(424, 9), to_unsigned(422, 9), to_unsigned(420, 9), to_unsigned(419, 9), to_unsigned(417, 9), to_unsigned(415, 9), to_unsigned(413, 9), to_unsigned(411, 9),
        to_unsigned(410, 9), to_unsigned(408, 9), to_unsigned(406, 9), to_unsigned(404, 9), to_unsigned(402, 9), to_unsigned(400, 9), to_unsigned(398, 9), to_unsigned(396, 9),
        to_unsigned(394, 9), to_unsigned(392, 9), to_unsigned(390, 9), to_unsigned(388, 9), to_unsigned(386, 9), to_unsigned(384, 9), to_unsigned(382, 9), to_unsigned(380, 9),
        to_unsigned(378, 9), to_unsigned(375, 9), to_unsigned(373, 9), to_unsigned(371, 9), to_unsigned(369, 9), to_unsigned(367, 9), to_unsigned(365, 9), to_unsigned(362, 9),
        to_unsigned(360, 9), to_unsigned(358, 9), to_unsigned(356, 9), to_unsigned(353, 9), to_unsigned(351, 9), to_unsigned(349, 9), to_unsigned(347, 9), to_unsigned(344, 9),
        to_unsigned(342, 9), to_unsigned(340, 9), to_unsigned(337, 9), to_unsigned(335, 9), to_unsigned(333, 9), to_unsigned(330, 9), to_unsigned(328, 9), to_unsigned(325, 9),
        to_unsigned(323, 9), to_unsigned(321, 9), to_unsigned(318, 9), to_unsigned(316, 9), to_unsigned(313, 9), to_unsigned(311, 9), to_unsigned(308, 9), to_unsigned(306, 9),
        to_unsigned(303, 9), to_unsigned(301, 9), to_unsigned(298, 9), to_unsigned(296, 9), to_unsigned(293, 9), to_unsigned(290, 9), to_unsigned(288, 9), to_unsigned(285, 9),
        to_unsigned(283, 9), to_unsigned(280, 9), to_unsigned(277, 9), to_unsigned(275, 9), to_unsigned(272, 9), to_unsigned(269, 9), to_unsigned(267, 9), to_unsigned(264, 9),
        to_unsigned(261, 9), to_unsigned(259, 9), to_unsigned(256, 9), to_unsigned(253, 9), to_unsigned(251, 9), to_unsigned(248, 9), to_unsigned(245, 9), to_unsigned(242, 9),
        to_unsigned(239, 9), to_unsigned(237, 9), to_unsigned(234, 9), to_unsigned(231, 9), to_unsigned(228, 9), to_unsigned(226, 9), to_unsigned(223, 9), to_unsigned(220, 9),
        to_unsigned(217, 9), to_unsigned(214, 9), to_unsigned(211, 9), to_unsigned(209, 9), to_unsigned(206, 9), to_unsigned(203, 9), to_unsigned(200, 9), to_unsigned(197, 9),
        to_unsigned(194, 9), to_unsigned(191, 9), to_unsigned(188, 9), to_unsigned(185, 9), to_unsigned(182, 9), to_unsigned(180, 9), to_unsigned(177, 9), to_unsigned(174, 9),
        to_unsigned(171, 9), to_unsigned(168, 9), to_unsigned(165, 9), to_unsigned(162, 9), to_unsigned(159, 9), to_unsigned(156, 9), to_unsigned(153, 9), to_unsigned(150, 9),
        to_unsigned(147, 9), to_unsigned(144, 9), to_unsigned(141, 9), to_unsigned(138, 9), to_unsigned(135, 9), to_unsigned(132, 9), to_unsigned(129, 9), to_unsigned(126, 9),
        to_unsigned(123, 9), to_unsigned(120, 9), to_unsigned(117, 9), to_unsigned(113, 9), to_unsigned(110, 9), to_unsigned(107, 9), to_unsigned(104, 9), to_unsigned(101, 9),
        to_unsigned(98, 9), to_unsigned(95, 9), to_unsigned(92, 9), to_unsigned(89, 9), to_unsigned(86, 9), to_unsigned(83, 9), to_unsigned(80, 9), to_unsigned(77, 9),
        to_unsigned(73, 9), to_unsigned(70, 9), to_unsigned(67, 9), to_unsigned(64, 9), to_unsigned(61, 9), to_unsigned(58, 9), to_unsigned(55, 9), to_unsigned(52, 9),
        to_unsigned(49, 9), to_unsigned(45, 9), to_unsigned(42, 9), to_unsigned(39, 9), to_unsigned(36, 9), to_unsigned(33, 9), to_unsigned(30, 9), to_unsigned(27, 9),
        to_unsigned(24, 9), to_unsigned(20, 9), to_unsigned(17, 9), to_unsigned(14, 9), to_unsigned(11, 9), to_unsigned(8, 9), to_unsigned(5, 9), to_unsigned(2, 9));
    constant C_DIRUV : t_rom16 := (
        to_unsigned(0, 16), to_unsigned(32768, 16), to_unsigned(32768, 16), to_unsigned(32768, 16), to_unsigned(32768, 16), to_unsigned(32768, 16), to_unsigned(32768, 16), to_unsigned(32768, 16),
        to_unsigned(32768, 16), to_unsigned(32768, 16), to_unsigned(32768, 16), to_unsigned(32768, 16), to_unsigned(32768, 16), to_unsigned(32768, 16), to_unsigned(32768, 16), to_unsigned(32768, 16),
        to_unsigned(128, 16), to_unsigned(23387, 16), to_unsigned(29241, 16), to_unsigned(31016, 16), to_unsigned(31775, 16), to_unsigned(32281, 16), to_unsigned(32277, 16), to_unsigned(32530, 16),
        to_unsigned(32528, 16), to_unsigned(32526, 16), to_unsigned(32525, 16), to_unsigned(32524, 16), to_unsigned(32779, 16), to_unsigned(32778, 16), to_unsigned(32777, 16), to_unsigned(32777, 16),
        to_unsigned(128, 16), to_unsigned(14706, 16), to_unsigned(23387, 16), to_unsigned(27463, 16), to_unsigned(29241, 16), to_unsigned(30512, 16), to_unsigned(31016, 16), to_unsigned(31523, 16),
        to_unsigned(31775, 16), to_unsigned(32028, 16), to_unsigned(32281, 16), to_unsigned(32279, 16), to_unsigned(32277, 16), to_unsigned(32531, 16), to_unsigned(32530, 16), to_unsigned(32529, 16),
        to_unsigned(128, 16), to_unsigned(10361, 16), to_unsigned(18283, 16), to_unsigned(23387, 16), to_unsigned(26189, 16), to_unsigned(28226, 16), to_unsigned(29241, 16), to_unsigned(30258, 16),
        to_unsigned(30765, 16), to_unsigned(31016, 16), to_unsigned(31525, 16), to_unsigned(31522, 16), to_unsigned(31775, 16), to_unsigned(32029, 16), to_unsigned(32027, 16), to_unsigned(32281, 16),
        to_unsigned(128, 16), to_unsigned(8060, 16), to_unsigned(14706, 16), to_unsigned(19814, 16), to_unsigned(23387, 16), to_unsigned(25680, 16), to_unsigned(27463, 16), to_unsigned(28480, 16),
        to_unsigned(29241, 16), to_unsigned(30004, 16), to_unsigned(30512, 16), to_unsigned(30764, 16), to_unsigned(31016, 16), to_unsigned(31270, 16), to_unsigned(31523, 16), to_unsigned(31777, 16),
        to_unsigned(128, 16), to_unsigned(6526, 16), to_unsigned(12407, 16), to_unsigned(17006, 16), to_unsigned(20580, 16), to_unsigned(23387, 16), to_unsigned(25170, 16), to_unsigned(26698, 16),
        to_unsigned(27972, 16), to_unsigned(28734, 16), to_unsigned(29241, 16), to_unsigned(30005, 16), to_unsigned(30257, 16), to_unsigned(30510, 16), to_unsigned(31019, 16), to_unsigned(31016, 16),
        to_unsigned(128, 16), to_unsigned(5502, 16), to_unsigned(10361, 16), to_unsigned(14706, 16), to_unsigned(18283, 16), to_unsigned(21090, 16), to_unsigned(23387, 16), to_unsigned(24915, 16),
        to_unsigned(26189, 16), to_unsigned(27463, 16), to_unsigned(28226, 16), to_unsigned(28733, 16), to_unsigned(29241, 16), to_unsigned(29750, 16), to_unsigned(30258, 16), to_unsigned(30512, 16),
        to_unsigned(128, 16), to_unsigned(4735, 16), to_unsigned(9083, 16), to_unsigned(12918, 16), to_unsigned(16495, 16), to_unsigned(19048, 16), to_unsigned(21345, 16), to_unsigned(23387, 16),
        to_unsigned(24660, 16), to_unsigned(25935, 16), to_unsigned(26953, 16), to_unsigned(27717, 16), to_unsigned(28480, 16), to_unsigned(28989, 16), to_unsigned(29241, 16), to_unsigned(29750, 16),
        to_unsigned(128, 16), to_unsigned(4223, 16), to_unsigned(8060, 16), to_unsigned(11640, 16), to_unsigned(14706, 16), to_unsigned(17517, 16), to_unsigned(19814, 16), to_unsigned(21600, 16),
        to_unsigned(23387, 16), to_unsigned(24661, 16), to_unsigned(25680, 16), to_unsigned(26699, 16), to_unsigned(27463, 16), to_unsigned(27971, 16), to_unsigned(28480, 16), to_unsigned(28988, 16),
        to_unsigned(128, 16), to_unsigned(3711, 16), to_unsigned(7293, 16), to_unsigned(10361, 16), to_unsigned(13429, 16), to_unsigned(15984, 16), to_unsigned(18283, 16), to_unsigned(20325, 16),
        to_unsigned(21856, 16), to_unsigned(23387, 16), to_unsigned(24406, 16), to_unsigned(25425, 16), to_unsigned(26189, 16), to_unsigned(26953, 16), to_unsigned(27717, 16), to_unsigned(28226, 16),
        to_unsigned(128, 16), to_unsigned(3455, 16), to_unsigned(6526, 16), to_unsigned(9595, 16), to_unsigned(12407, 16), to_unsigned(14706, 16), to_unsigned(17006, 16), to_unsigned(18793, 16),
        to_unsigned(20580, 16), to_unsigned(22111, 16), to_unsigned(23387, 16), to_unsigned(24406, 16), to_unsigned(25170, 16), to_unsigned(25934, 16), to_unsigned(26698, 16), to_unsigned(27463, 16),
        to_unsigned(128, 16), to_unsigned(3199, 16), to_unsigned(6014, 16), to_unsigned(8827, 16), to_unsigned(11384, 16), to_unsigned(13685, 16), to_unsigned(15728, 16), to_unsigned(17772, 16),
        to_unsigned(19304, 16), to_unsigned(20835, 16), to_unsigned(22111, 16), to_unsigned(23387, 16), to_unsigned(24150, 16), to_unsigned(25171, 16), to_unsigned(25935, 16), to_unsigned(26444, 16),
        to_unsigned(128, 16), to_unsigned(2944, 16), to_unsigned(5502, 16), to_unsigned(8060, 16), to_unsigned(10361, 16), to_unsigned(12662, 16), to_unsigned(14706, 16), to_unsigned(16495, 16),
        to_unsigned(18283, 16), to_unsigned(19814, 16), to_unsigned(21090, 16), to_unsigned(22110, 16), to_unsigned(23387, 16), to_unsigned(24151, 16), to_unsigned(24915, 16), to_unsigned(25680, 16),
        to_unsigned(128, 16), to_unsigned(2688, 16), to_unsigned(4991, 16), to_unsigned(7549, 16), to_unsigned(9850, 16), to_unsigned(11895, 16), to_unsigned(13940, 16), to_unsigned(15729, 16),
        to_unsigned(17261, 16), to_unsigned(18793, 16), to_unsigned(20069, 16), to_unsigned(21346, 16), to_unsigned(22366, 16), to_unsigned(23387, 16), to_unsigned(24151, 16), to_unsigned(24916, 16),
        to_unsigned(128, 16), to_unsigned(2432, 16), to_unsigned(4735, 16), to_unsigned(7037, 16), to_unsigned(9083, 16), to_unsigned(11129, 16), to_unsigned(12918, 16), to_unsigned(14706, 16),
        to_unsigned(16495, 16), to_unsigned(17772, 16), to_unsigned(19048, 16), to_unsigned(20325, 16), to_unsigned(21345, 16), to_unsigned(22366, 16), to_unsigned(23387, 16), to_unsigned(24151, 16),
        to_unsigned(128, 16), to_unsigned(2432, 16), to_unsigned(4479, 16), to_unsigned(6526, 16), to_unsigned(8572, 16), to_unsigned(10361, 16), to_unsigned(12407, 16), to_unsigned(13940, 16),
        to_unsigned(15473, 16), to_unsigned(17006, 16), to_unsigned(18283, 16), to_unsigned(19559, 16), to_unsigned(20580, 16), to_unsigned(21601, 16), to_unsigned(22366, 16), to_unsigned(23387, 16));
    constant C_PALC : t_pal16 := (
        to_unsigned(24633, 16), to_unsigned(21567, 16), to_unsigned(18244, 16), to_unsigned(15177, 16), to_unsigned(11854, 16), to_unsigned(8788, 16), to_unsigned(4434, 16), to_unsigned(336, 16),
        to_unsigned(61518, 16), to_unsigned(57165, 16), to_unsigned(53067, 16), to_unsigned(49988, 16), to_unsigned(47165, 16), to_unsigned(44087, 16), to_unsigned(41008, 16), to_unsigned(38185, 16),
        to_unsigned(20198, 16), to_unsigned(20704, 16), to_unsigned(21210, 16), to_unsigned(21716, 16), to_unsigned(22222, 16), to_unsigned(22984, 16), to_unsigned(21188, 16), to_unsigned(19392, 16),
        to_unsigned(17595, 16), to_unsigned(16055, 16), to_unsigned(14259, 16), to_unsigned(12216, 16), to_unsigned(10428, 16), to_unsigned(8385, 16), to_unsigned(6089, 16), to_unsigned(4047, 16),
        to_unsigned(60507, 16), to_unsigned(58711, 16), to_unsigned(56915, 16), to_unsigned(55375, 16), to_unsigned(53579, 16), to_unsigned(51783, 16), to_unsigned(51519, 16), to_unsigned(51511, 16),
        to_unsigned(51246, 16), to_unsigned(50982, 16), to_unsigned(50718, 16), to_unsigned(53784, 16), to_unsigned(56594, 16), to_unsigned(59405, 16), to_unsigned(61704, 16), to_unsigned(64004, 16),
        to_unsigned(28890, 16), to_unsigned(20930, 16), to_unsigned(12713, 16), to_unsigned(4253, 16), to_unsigned(60829, 16), to_unsigned(51369, 16), to_unsigned(41654, 16), to_unsigned(40155, 16),
        to_unsigned(38400, 16), to_unsigned(36902, 16), to_unsigned(44862, 16), to_unsigned(53079, 16), to_unsigned(61040, 16), to_unsigned(4963, 16), to_unsigned(14423, 16), to_unsigned(24138, 16),
        to_unsigned(10195, 16), to_unsigned(10702, 16), to_unsigned(11209, 16), to_unsigned(11717, 16), to_unsigned(11968, 16), to_unsigned(12475, 16), to_unsigned(11708, 16), to_unsigned(10684, 16),
        to_unsigned(9661, 16), to_unsigned(8638, 16), to_unsigned(7362, 16), to_unsigned(6346, 16), to_unsigned(5330, 16), to_unsigned(4313, 16), to_unsigned(3296, 16), to_unsigned(2279, 16),
        to_unsigned(42719, 16), to_unsigned(42474, 16), to_unsigned(42484, 16), to_unsigned(42239, 16), to_unsigned(41738, 16), to_unsigned(41749, 16), to_unsigned(45086, 16), to_unsigned(48423, 16),
        to_unsigned(52017, 16), to_unsigned(55354, 16), to_unsigned(58691, 16), to_unsigned(62532, 16), to_unsigned(836, 16), to_unsigned(4420, 16), to_unsigned(8260, 16), to_unsigned(12101, 16),
        to_unsigned(47316, 16), to_unsigned(47316, 16), to_unsigned(47316, 16), to_unsigned(47316, 16), to_unsigned(47316, 16), to_unsigned(47316, 16), to_unsigned(47316, 16), to_unsigned(47316, 16),
        to_unsigned(47316, 16), to_unsigned(47316, 16), to_unsigned(47316, 16), to_unsigned(47316, 16), to_unsigned(47316, 16), to_unsigned(47316, 16), to_unsigned(47316, 16), to_unsigned(47316, 16),
        to_unsigned(0, 16), to_unsigned(0, 16), to_unsigned(0, 16), to_unsigned(0, 16), to_unsigned(0, 16), to_unsigned(0, 16), to_unsigned(0, 16), to_unsigned(0, 16),
        to_unsigned(0, 16), to_unsigned(0, 16), to_unsigned(0, 16), to_unsigned(0, 16), to_unsigned(0, 16), to_unsigned(0, 16), to_unsigned(0, 16), to_unsigned(0, 16));
    constant C_PALY : t_pal16 := (
        to_unsigned(26420, 16), to_unsigned(25651, 16), to_unsigned(25138, 16), to_unsigned(24369, 16), to_unsigned(23600, 16), to_unsigned(23087, 16), to_unsigned(23856, 16), to_unsigned(24626, 16),
        to_unsigned(25395, 16), to_unsigned(26164, 16), to_unsigned(26934, 16), to_unsigned(30010, 16), to_unsigned(32831, 16), to_unsigned(35908, 16), to_unsigned(38985, 16), to_unsigned(41806, 16),
        to_unsigned(15907, 16), to_unsigned(18472, 16), to_unsigned(21292, 16), to_unsigned(23856, 16), to_unsigned(26421, 16), to_unsigned(29241, 16), to_unsigned(31549, 16), to_unsigned(33857, 16),
        to_unsigned(36165, 16), to_unsigned(38729, 16), to_unsigned(41036, 16), to_unsigned(43601, 16), to_unsigned(46165, 16), to_unsigned(47961, 16), to_unsigned(47966, 16), to_unsigned(47970, 16),
        to_unsigned(14112, 16), to_unsigned(16933, 16), to_unsigned(19754, 16), to_unsigned(22831, 16), to_unsigned(25651, 16), to_unsigned(28728, 16), to_unsigned(32318, 16), to_unsigned(35908, 16),
        to_unsigned(39498, 16), to_unsigned(43344, 16), to_unsigned(46934, 16), to_unsigned(47962, 16), to_unsigned(47966, 16), to_unsigned(47969, 16), to_unsigned(47973, 16), to_unsigned(47977, 16),
        to_unsigned(20780, 16), to_unsigned(31805, 16), to_unsigned(42831, 16), to_unsigned(47969, 16), to_unsigned(47960, 16), to_unsigned(42575, 16), to_unsigned(37190, 16), to_unsigned(39241, 16),
        to_unsigned(41293, 16), to_unsigned(43600, 16), to_unsigned(32574, 16), to_unsigned(21548, 16), to_unsigned(10522, 16), to_unsigned(16164, 16), to_unsigned(21805, 16), to_unsigned(27190, 16),
        to_unsigned(24113, 16), to_unsigned(27446, 16), to_unsigned(30523, 16), to_unsigned(33857, 16), to_unsigned(37190, 16), to_unsigned(40524, 16), to_unsigned(42319, 16), to_unsigned(44114, 16),
        to_unsigned(46165, 16), to_unsigned(47960, 16), to_unsigned(47963, 16), to_unsigned(47965, 16), to_unsigned(47967, 16), to_unsigned(47970, 16), to_unsigned(47972, 16), to_unsigned(47974, 16),
        to_unsigned(35652, 16), to_unsigned(35908, 16), to_unsigned(36165, 16), to_unsigned(36421, 16), to_unsigned(36677, 16), to_unsigned(36934, 16), to_unsigned(35652, 16), to_unsigned(34369, 16),
        to_unsigned(33087, 16), to_unsigned(31805, 16), to_unsigned(30267, 16), to_unsigned(30267, 16), to_unsigned(30011, 16), to_unsigned(30010, 16), to_unsigned(29754, 16), to_unsigned(29754, 16),
        to_unsigned(43577, 16), to_unsigned(43577, 16), to_unsigned(43577, 16), to_unsigned(43577, 16), to_unsigned(43577, 16), to_unsigned(43577, 16), to_unsigned(43577, 16), to_unsigned(43577, 16),
        to_unsigned(43577, 16), to_unsigned(43577, 16), to_unsigned(43577, 16), to_unsigned(43577, 16), to_unsigned(43577, 16), to_unsigned(43577, 16), to_unsigned(43577, 16), to_unsigned(43577, 16),
        to_unsigned(47899, 16), to_unsigned(47899, 16), to_unsigned(47899, 16), to_unsigned(47899, 16), to_unsigned(47899, 16), to_unsigned(47899, 16), to_unsigned(47899, 16), to_unsigned(47899, 16),
        to_unsigned(47899, 16), to_unsigned(47899, 16), to_unsigned(47899, 16), to_unsigned(47899, 16), to_unsigned(47899, 16), to_unsigned(47899, 16), to_unsigned(47899, 16), to_unsigned(47899, 16));
    constant C_HREC : t_hrec := (
        to_unsigned(73728, 17), to_unsigned(36864, 17), to_unsigned(24576, 17), to_unsigned(18432, 17), to_unsigned(14746, 17), to_unsigned(12288, 17), to_unsigned(10533, 17), to_unsigned(9216, 17),
        to_unsigned(8192, 17), to_unsigned(7373, 17), to_unsigned(6703, 17), to_unsigned(6144, 17), to_unsigned(5671, 17), to_unsigned(5266, 17), to_unsigned(4915, 17), to_unsigned(4608, 17));
    -- harmonograph pendulum scale k = round(18 / max(a,|b|))
    type t_hk is array(0 to 15) of unsigned(4 downto 0);
    constant C_HK : t_hk := (
        to_unsigned(18, 5), to_unsigned(9, 5), to_unsigned(6, 5), to_unsigned(4, 5),
        to_unsigned(4, 5),  to_unsigned(3, 5), to_unsigned(3, 5), to_unsigned(2, 5),
        to_unsigned(2, 5),  to_unsigned(2, 5), to_unsigned(2, 5), to_unsigned(2, 5),
        to_unsigned(1, 5),  to_unsigned(1, 5), to_unsigned(1, 5), to_unsigned(1, 5));

    constant C_PENUP : std_logic_vector(4 downto 0) := "10000";
    constant C_ENV1  : unsigned(17 downto 0) := to_unsigned(131072, 18);   -- 1.0

    -- ==== helpers ====
    -- quarter-wave cosine: 10-bit angle -> 8-bit ROM index (odd quadrant mirrors)
    function f_qaddr(th : unsigned(19 downto 0)) return unsigned is
        variable i : unsigned(7 downto 0);
    begin
        i := th(17 downto 10);
        if th(18) = '1' then i := not i; end if;
        return i;
    end function;

    -- D4 copy k of (x, y): returns x' (sel_y = '0') or y' (sel_y = '1')
    --   k: 0 (x,y) 1 (-x,y) 2 (x,-y) 3 (-x,-y) 4 (y,x) 5 (-y,x) 6 (y,-x) 7 (-y,-x)
    function f_d4(k : unsigned(2 downto 0); x, y : signed; sel_y : std_logic) return signed is
        variable r : signed(x'length - 1 downto 0);
    begin
        if sel_y = '0' then
            if k(2) = '0' then r := x; else r := y; end if;
            if k(0) = '1' then r := -r; end if;
        else
            if k(2) = '0' then r := y; else r := x; end if;
            if k(1) = '1' then r := -r; end if;
        end if;
        return r;
    end function;

    -- saturate a 12-bit delta to +-7 (dense ring) or +-127, never the pen code
    function f_sat(d : signed(11 downto 0); dens : unsigned(1 downto 0)) return signed is
        variable r : signed(7 downto 0);
    begin
        if dens = "00" then
            if d(11 downto 3) = "000000000" or d(11 downto 3) = "111111111" then
                r := d(7 downto 0);
            elsif d(11) = '1' then
                r := to_signed(-7, 8);
            else
                r := to_signed(7, 8);
            end if;
            if r = -8 then r := to_signed(-7, 8); end if;
        else
            if d(11 downto 7) = "00000" or d(11 downto 7) = "11111" then
                r := d(7 downto 0);
            elsif d(11) = '1' then
                r := to_signed(-127, 8);
            else
                r := to_signed(127, 8);
            end if;
            if r = -128 then r := to_signed(-127, 8); end if;
        end if;
        return r;
    end function;

    -- ==== RAMs ====
    type t_vram is array(0 to 4095) of std_logic_vector(15 downto 0);
    type t_cram is array(0 to 255) of std_logic_vector(47 downto 0);
    type t_lbuf is array(0 to 511) of std_logic_vector(23 downto 0);
    signal vram : t_vram;
    signal cram : t_cram;
    signal lb0, lb1 : t_lbuf;
    attribute no_rw_check : boolean;
    attribute no_rw_check of vram : signal is true;
    attribute no_rw_check of cram : signal is true;
    attribute no_rw_check of lb0  : signal is true;
    attribute no_rw_check of lb1  : signal is true;

    signal vw_we   : std_logic;
    signal vw_addr : unsigned(11 downto 0);
    signal vw_data : std_logic_vector(15 downto 0);
    signal vr_addr : unsigned(11 downto 0) := (others => '0');
    signal vr_q    : std_logic_vector(15 downto 0) := (others => '0');

    signal cw_we   : std_logic;
    signal cw_addr : unsigned(7 downto 0);
    signal cw_data : std_logic_vector(47 downto 0);
    signal cr_addr : unsigned(7 downto 0) := (others => '0');
    signal cr_q    : std_logic_vector(47 downto 0) := (others => '0');

    signal l0_we, l1_we : std_logic;
    signal l0_wa, l1_wa, l0_ra, l1_ra : unsigned(8 downto 0);
    signal l0_wd, l1_wd : std_logic_vector(23 downto 0);
    signal l0_q, l1_q   : std_logic_vector(23 downto 0) := (others => '0');

    -- ==== measurement / field trigger ====
    signal r_av, r_av1 : std_logic := '0';
    signal m_wcnt : unsigned(11 downto 0) := (others => '0');
    signal m_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal m_hcnt : unsigned(10 downto 0) := (others => '0');
    signal m_noav : unsigned(11 downto 0) := (others => '0');
    signal f_trig : std_logic := '0';
    signal f_W    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal f_Hf   : unsigned(10 downto 0) := to_unsigned(1080, 11);

    -- ==== generated avid ====
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal bcol  : unsigned(10 downto 0) := (others => '0');

    -- ==== per-field controls / raster class ====
    signal k_zA   : unsigned(3 downto 0) := to_unsigned(4, 4);    -- a = zA + 1
    signal k_zB   : unsigned(5 downto 0) := to_unsigned(8, 6);    -- b = zB - 16
    signal k_pal  : unsigned(2 downto 0) := (others => '0');
    signal k_wz   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal k_harm, k_eng, k_kal, k_glow, k_vid : std_logic := '0';
    signal gl_arm : unsigned(11 downto 0) := to_unsigned(400 * 4, 12);   -- 10.2 glides
    signal gl_tw  : unsigned(11 downto 0) := (others => '0');
    signal gl_tr  : unsigned(11 downto 0) := to_unsigned(760 * 4, 12);
    signal gd_a, gd_t, gd_r : signed(12 downto 0) := (others => '0');

    signal c_cls  : unsigned(1 downto 0) := "00";                 -- 1080 / 720 / SD
    signal c_il   : std_logic := '0';                             -- interlaced
    signal c_cx   : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal c_cy   : unsigned(10 downto 0) := to_unsigned(540, 11);
    signal c_rb   : unsigned(8 downto 0)  := to_unsigned(500, 9);
    signal c_Wm1  : unsigned(10 downto 0) := to_unsigned(1919, 11);
    signal c_Wwm1 : unsigned(8 downto 0)  := to_unsigned(479, 9);
    signal c_pen  : unsigned(2 downto 0)  := to_unsigned(4, 3);
    -- ring density: points = 8192 >> dens.  dens 0 ("dense"): two points per
    -- 16-bit word, 4-bit deltas, 32-point chunks; otherwise one point per
    -- word, 8-bit deltas, 16-point chunks
    signal c_dens : unsigned(1 downto 0) := "01";
    signal c_nchm : unsigned(7 downto 0)  := to_unsigned(255, 8); -- chunks/figure - 1
    signal c_nm   : unsigned(12 downto 0) := to_unsigned(4095, 13); -- points/figure - 1

    -- ==== generator: field setup ====
    signal gs, gs_ret : integer range 0 to 31 := 0;
    signal gen_done : std_logic := '0';
    signal ga, gb_abs : unsigned(4 downto 0) := (others => '0');
    signal gb_neg : std_logic := '0';
    signal gx, gy, gz : unsigned(4 downto 0) := (others => '0');
    signal gca, gcb   : unsigned(4 downto 0) := (others => '0');
    signal kk     : unsigned(4 downto 0) := (others => '0');
    signal tAk    : unsigned(9 downto 0) := (others => '0');
    signal sm_acc, sm_a : unsigned(20 downto 0) := (others => '0');
    signal sm_b   : unsigned(9 downto 0) := (others => '0');
    signal sm_n   : unsigned(3 downto 0) := (others => '0');
    signal incA, incYA, incB : unsigned(19 downto 0) := (others => '0');   -- incB signed
    signal R1, R2 : unsigned(9 downto 0) := (others => '0');
    signal g_bloom : unsigned(3 downto 0) := (others => '0');
    signal drift  : unsigned(19 downto 0) := (others => '0');
    signal nkt256 : unsigned(8 downto 0) := (others => '0');
    signal nk     : unsigned(8 downto 0) := (others => '0');      -- chunks per copy
    signal nb     : unsigned(13 downto 0) := (others => '0');     -- points per copy
    signal gend   : unsigned(13 downto 0) := (others => '0');     -- nb + 2
    signal ncm1   : unsigned(2 downto 0) := (others => '0');      -- copies - 1
    signal h_q    : unsigned(9 downto 0) := (others => '0');      -- pen, 1/4 chunks
    signal s_ch   : unsigned(7 downto 0) := (others => '0');
    signal h_acc  : unsigned(9 downto 0) := (others => '0');
    signal h_cur  : unsigned(3 downto 0) := (others => '0');
    signal env_n  : unsigned(7 downto 0) := (others => '0');

    -- ==== generator: point loop ====
    signal gj     : unsigned(13 downto 0) := (others => '0');
    signal gt     : unsigned(2 downto 0)  := (others => '0');
    signal thA, thB, thyA, thyB : unsigned(19 downto 0) := (others => '0');
    signal g_thsel : unsigned(19 downto 0);
    signal cq_addr : unsigned(7 downto 0);
    signal cq_q, cq_r : unsigned(8 downto 0) := (others => '0');
    signal sgn_sr : std_logic_vector(3 downto 0) := (others => '0');
    signal mul_a, mul_b : unsigned(9 downto 0) := (others => '0');
    signal mul_p  : unsigned(19 downto 0) := (others => '0');
    signal X_acc, Y_acc : signed(19 downto 0) := (others => '0');
    signal env    : unsigned(17 downto 0) := C_ENV1;
    signal env10v : unsigned(9 downto 0) := (others => '0');
    signal xsg, ysg : std_logic := '0';
    signal ai     : unsigned(12 downto 0) := (others => '0');
    signal wrap_f, wrap_b : std_logic := '0';
    signal bx, by : signed(10 downto 0) := (others => '0');
    signal Px, Py : signed(10 downto 0) := (others => '0');
    signal ddx, ddy : signed(11 downto 0) := (others => '0');
    signal dX, dY   : signed(7 downto 0) := (others => '0');
    signal b_first, b_pen, b_chunk, b_live : std_logic := '0';
    signal b_q    : unsigned(13 downto 0) := (others => '0');
    signal wdx, wdy : signed(7 downto 0) := (others => '0');
    signal hdx, hdy : signed(3 downto 0) := (others => '0');       -- dense: even point held
    signal wpen   : std_logic := '0';
    signal wv_act : std_logic := '0';
    signal wk     : unsigned(2 downto 0) := (others => '0');
    signal va     : unsigned(11 downto 0) := (others => '0');
    signal bxmin, bxmax, bymin, bymax : signed(10 downto 0) := (others => '0');
    signal csx, csy : signed(10 downto 0) := (others => '0');
    signal co_sx, co_sy, co_xmin, co_xmax, co_ymin, co_ymax : signed(10 downto 0) := (others => '0');
    signal co_H   : unsigned(3 downto 0) := (others => '0');
    signal ch_act : std_logic := '0';
    signal ck     : unsigned(2 downto 0) := (others => '0');
    signal ca     : unsigned(7 downto 0) := (others => '0');

    -- per-field constants for the renderer / beam
    signal s_nt    : unsigned(8 downto 0) := (others => '0');     -- chunks to scan
    signal s_dense : std_logic := '0';
    signal s_reach : unsigned(3 downto 0) := to_unsigned(3, 4);
    signal s_K     : unsigned(4 downto 0) := to_unsigned(15, 5);
    signal s_cmax  : unsigned(3 downto 0) := to_unsigned(15, 4);
    signal s_eth   : unsigned(4 downto 0) := to_unsigned(4, 5);
    signal b_w     : unsigned(3 downto 0) := to_unsigned(3, 4);
    signal b_depth : unsigned(4 downto 0) := (others => '0');

    -- ==== renderer ====
    signal vis_row  : unsigned(10 downto 0) := (others => '0');
    signal rc       : integer range 0 to 3 := 3;
    signal pend     : std_logic := '0';
    signal pend_row : unsigned(10 downto 0) := (others => '0');
    signal vr2      : unsigned(10 downto 0) := to_unsigned(2, 11);
    signal pend_ok, st_go : std_logic := '0';
    signal st_row   : unsigned(10 downto 0) := (others => '0');
    signal e_run    : std_logic := '0';
    signal e_R, e_Rlo, e_Rhi : signed(12 downto 0) := (others => '0');   -- centre offsets
    signal rbank    : std_logic := '0';
    signal ev_line  : std_logic := '0';
    signal cl_run   : std_logic := '0';

    signal ps_c   : unsigned(8 downto 0) := (others => '0');
    signal p1_v, p2_v, p3_v : std_logic := '0';
    signal p1_c, p2_c, p3_c : unsigned(7 downto 0) := (others => '0');
    signal cr_r   : std_logic_vector(47 downto 0) := (others => '0');
    signal hb_full : std_logic := '0';
    signal hb_c   : unsigned(7 downto 0) := (others => '0');
    signal hb_e   : std_logic_vector(25 downto 0) := (others => '0');

    signal stall, stall_b, stall_c, stall_d : std_logic := '0';
    attribute keep : boolean;
    attribute keep of stall, stall_b, stall_c, stall_d : signal is true;
    signal vs_cnt : unsigned(4 downto 0) := (others => '0');
    signal vs_act : std_logic := '0';
    signal cur_c  : unsigned(7 downto 0) := (others => '0');
    signal cur_e  : std_logic_vector(25 downto 0) := (others => '0');   -- H & y0 & x0
    signal t1_v, t2_v, t3_v, t1_f, t2_f, t3_f, t1_h, t2_h, t3_h : std_logic := '0';
    signal vd     : std_logic_vector(15 downto 0) := (others => '0');
    signal ix, iy : signed(11 downto 0) := (others => '0');
    signal int_H  : std_logic_vector(3 downto 0) := (others => '0');
    -- c3 -> s5: integrated segment extents (x screen, y centre offset)
    signal s5_v, s5_pen, s5_sdx, s5_sdy : std_logic := '0';
    signal s5_xlo, s5_xhi, s5_ylo, s5_yhi, s5_sx, s5_eyR : signed(11 downto 0) := (others => '0');
    signal s5_H   : std_logic_vector(3 downto 0) := (others => '0');
    signal uv_addr : unsigned(7 downto 0) := (others => '0');
    signal uv_q, uv_r : unsigned(15 downto 0) := (others => '0');
    -- c4 -> s6: row hit, word range, |x|,|y| offsets
    signal s6_v, s6_act, s6_par : std_logic := '0';
    signal s6_wl, s6_wh : signed(9 downto 0) := (others => '0');
    signal s6_aex, s6_aey : unsigned(7 downto 0) := (others => '0');
    signal s6_H   : std_logic_vector(3 downto 0) := (others => '0');
    -- c5 -> s8: products
    signal s8_v, s8_act, s8_par : std_logic := '0';
    signal s8_wl, s8_wh : signed(9 downto 0) := (others => '0');
    signal s8_pu, s8_pv : unsigned(15 downto 0) := (others => '0');
    signal s8_um  : unsigned(7 downto 0) := (others => '0');
    signal s8_H   : std_logic_vector(3 downto 0) := (others => '0');

    -- job FIFO {wl 10, wh 10, E0 16, |u| 8, H 4} = 48 bits, 2 deep (word
    -- indices are signed: a segment may start left of the screen)
    type t_fifo is array(0 to 1) of std_logic_vector(47 downto 0);
    signal f_mem : t_fifo := (others => (others => '0'));
    signal f_wp, f_rp : std_logic := '0';
    signal f_cnt : unsigned(1 downto 0) := (others => '0');

    -- word iterator: pixel k of a word sits at E - k|u|
    signal it_v   : std_logic := '0';
    signal it_w, it_wh : signed(9 downto 0) := (others => '0');
    signal it_E   : signed(15 downto 0) := (others => '0');
    signal it_u1, it_u3, it_u4 : unsigned(9 downto 0) := (others => '0');
    signal it_H   : std_logic_vector(3 downto 0) := (others => '0');

    -- stamp pipeline
    type t_e16 is array(0 to 3) of signed(15 downto 0);
    type t_e5  is array(0 to 3) of unsigned(4 downto 0);
    type t_c4  is array(0 to 3) of unsigned(3 downto 0);
    signal w1_v   : std_logic := '0';
    signal w1_a   : unsigned(8 downto 0) := (others => '0');
    signal w1_E   : t_e16 := (others => (others => '0'));
    signal w1_H   : std_logic_vector(3 downto 0) := (others => '0');
    signal w2_v   : std_logic := '0';
    signal w2_a   : unsigned(8 downto 0) := (others => '0');
    signal w2_e   : t_e5 := (others => (others => '0'));
    signal w2_H   : std_logic_vector(3 downto 0) := (others => '0');
    signal w3_v   : std_logic := '0';
    signal w3_a   : unsigned(8 downto 0) := (others => '0');
    signal w3_c   : t_c4 := (others => (others => '0'));
    signal w3_tk, w3_any : std_logic_vector(1 downto 0) := (others => '0');
    signal w3_H   : std_logic_vector(3 downto 0) := (others => '0');
    signal w3_s1, w3_s2, w3_s3 : std_logic := '0';
    signal rd_r   : std_logic_vector(23 downto 0) := (others => '0');
    signal st_we  : std_logic := '0';
    signal st_wa  : unsigned(8 downto 0) := (others => '0');
    signal st_wd  : std_logic_vector(23 downto 0) := (others => '0');
    signal h1_v, h2_v : std_logic := '0';
    signal h1_a, h2_a : unsigned(8 downto 0) := (others => '0');
    signal h1_d, h2_d : std_logic_vector(23 downto 0) := (others => '0');

    -- ==== beam ====
    signal gav_d   : std_logic := '0';
    signal clr_we, clr_bank : std_logic := '0';
    signal clr_addr : unsigned(8 downto 0) := (others => '0');
    signal cl_ptr  : unsigned(8 downto 0) := (others => '0');
    signal cl_bank : std_logic := '0';
    signal vy1, vy2 : unsigned(9 downto 0) := (others => '0');
    signal s1_p   : unsigned(1 downto 0) := (others => '0');
    signal s1_av, s1_bk : std_logic := '0';
    signal s3_av, s3_pe : std_logic := '0';
    signal s3_Ie, s3_Io, s3_H : unsigned(3 downto 0) := (others => '0');
    signal s3_lum  : unsigned(7 downto 0) := (others => '0');
    signal pal_addr : unsigned(6 downto 0);
    signal pc_q, pc_r, py_q, py_r : unsigned(15 downto 0) := (others => '0');
    signal s4_av, s4_pe : std_logic := '0';
    signal s4b_Ie, s4b_Io, s4b_H, s4_lum4 : unsigned(3 downto 0) := (others => '0');
    signal s4_weff : unsigned(6 downto 0) := (others => '0');
    signal s5_av, s5_pe : std_logic := '0';
    signal s5_ae, s5_ao, s5_he, s5_ho, s5_lum4 : unsigned(3 downto 0) := (others => '0');
    signal s5_fade : unsigned(4 downto 0) := (others => '0');
    signal s6_av, s6_pe : std_logic := '0';
    signal s6_apx, s6_apr, s6_hpx, s6_hpr : unsigned(3 downto 0) := (others => '0');
    signal s6_fl  : unsigned(4 downto 0) := (others => '0');
    signal s6_dY, s6_dC : signed(10 downto 0) := (others => '0');
    signal s7_av, s7_pe : std_logic := '0';
    signal s7_axp, s7_axr : unsigned(4 downto 0) := (others => '0');
    signal s7_hpx, s7_hpr : unsigned(3 downto 0) := (others => '0');
    signal s7_dY, s7_dC : signed(10 downto 0) := (others => '0');
    signal s8_av, s8_csh : std_logic := '0';
    signal s8_pY, s8_pC : signed(16 downto 0) := (others => '0');
    signal s8_yg, s8_cg : signed(12 downto 0) := (others => '0');
    signal o_y, o_c : unsigned(9 downto 0) := (others => '0');

    type t_bsr is array(1 to C_L) of std_logic;
    signal d_hs, d_vs, d_fl : t_bsr := (others => '1');
    signal d_gv : t_bsr := (others => '0');

begin

    -- ====================================================================
    -- RAMs / ROMs
    -- ====================================================================
    p_vram : process(clk)
    begin
        if rising_edge(clk) then
            if vw_we = '1' then
                vram(to_integer(vw_addr)) <= vw_data;
            end if;
            if stall = '0' then
                vr_q <= vram(to_integer(vr_addr));
            end if;
        end if;
    end process p_vram;

    p_cram : process(clk)
    begin
        if rising_edge(clk) then
            if cw_we = '1' then
                cram(to_integer(cw_addr)) <= cw_data;
            end if;
            cr_q <= cram(to_integer(cr_addr));
        end if;
    end process p_cram;

    p_lb0 : process(clk)
    begin
        if rising_edge(clk) then
            if l0_we = '1' then
                lb0(to_integer(l0_wa)) <= l0_wd;
            end if;
            l0_q <= lb0(to_integer(l0_ra));
        end if;
    end process p_lb0;

    p_lb1 : process(clk)
    begin
        if rising_edge(clk) then
            if l1_we = '1' then
                lb1(to_integer(l1_wa)) <= l1_wd;
            end if;
            l1_q <= lb1(to_integer(l1_ra));
        end if;
    end process p_lb1;

    -- line-buffer ports: the render bank belongs to the stamper, the other
    -- to the beam (read + clear-behind)
    l0_ra <= w1_a when rbank = '0' else bcol(10 downto 2);
    l1_ra <= w1_a when rbank = '1' else bcol(10 downto 2);
    l0_we <= (clr_we and not clr_bank) or (st_we and not rbank);
    l1_we <= (clr_we and clr_bank) or (st_we and rbank);
    l0_wa <= clr_addr when (clr_we = '1' and clr_bank = '0') else st_wa;
    l1_wa <= clr_addr when (clr_we = '1' and clr_bank = '1') else st_wa;
    l0_wd <= (others => '0') when (clr_we = '1' and clr_bank = '0') else st_wd;
    l1_wd <= (others => '0') when (clr_we = '1' and clr_bank = '1') else st_wd;

    -- quarter-wave cosine ROM (address from the slot-selected angle)
    with gt select g_thsel <= thA when "000", thB when "001", thyA when "010", thyB when others;
    cq_addr <= f_qaddr(g_thsel);

    p_cos : process(clk)
    begin
        if rising_edge(clk) then
            cq_q <= C_COSQ(to_integer(cq_addr));
            cq_r <= cq_q;
        end if;
    end process p_cos;

    p_mul : process(clk)
    begin
        if rising_edge(clk) then
            mul_p <= mul_a * mul_b;
        end if;
    end process p_mul;

    -- segment direction ROM (stall-gated like the rest of the vertex pipe)
    p_uv : process(clk)
    begin
        if rising_edge(clk) then
            if stall_b = '0' then
                uv_q <= C_DIRUV(to_integer(uv_addr));
                uv_r <= uv_q;
            end if;
        end if;
    end process p_uv;

    -- palette ROMs (age ramp per colour)
    pal_addr <= k_pal & s3_H;
    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            pc_q <= C_PALC(to_integer(pal_addr));
            py_q <= C_PALY(to_integer(pal_addr));
            pc_r <= pc_q;
            py_r <= py_q;
        end if;
    end process p_pal;

    -- generator write ports (combinational from the burst state)
    vw_we   <= wv_act;
    vw_addr <= va;
    -- dense: {even dx4 dy4, odd dx4 dy4} (pen-up = odd dx -8); otherwise
    -- {dx8, dy8} (pen-up = dx -128)
    p_vwd : process(wk, wdx, wdy, hdx, hdy, wpen, c_dens)
        variable v_x8, v_y8 : signed(7 downto 0);
        variable v_x4, v_y4 : signed(3 downto 0);
    begin
        v_x8 := f_d4(wk, wdx, wdy, '0');
        v_y8 := f_d4(wk, wdx, wdy, '1');
        v_x4 := f_d4(wk, hdx, hdy, '0');
        v_y4 := f_d4(wk, hdx, hdy, '1');
        if c_dens = "00" then
            if wpen = '1' then
                vw_data <= std_logic_vector(v_x4) & std_logic_vector(v_y4) & x"80";
            else
                vw_data <= std_logic_vector(v_x4) & std_logic_vector(v_y4)
                         & std_logic_vector(v_x8(3 downto 0)) & std_logic_vector(v_y8(3 downto 0));
            end if;
        else
            if wpen = '1' then
                vw_data <= x"8000";
            else
                vw_data <= std_logic_vector(v_x8) & std_logic_vector(v_y8);
            end if;
        end if;
    end process p_vwd;

    cw_we   <= ch_act;
    cw_addr <= ca;
    p_cwd : process(ck, co_sx, co_sy, co_xmin, co_xmax, co_ymin, co_ymax, co_H)
        variable v_ix, v_iy, v_lo, v_hi, v_x : signed(10 downto 0);
    begin
        v_ix := f_d4(ck, co_sx, co_sy, '0');
        v_iy := f_d4(ck, co_sx, co_sy, '1');
        if ck(2) = '0' then
            v_lo := co_ymin;  v_hi := co_ymax;
        else
            v_lo := co_xmin;  v_hi := co_xmax;
        end if;
        if ck(1) = '1' then
            v_x := -v_hi;  v_hi := -v_lo;  v_lo := v_x;
        end if;
        -- {age, ymax, ymin, y0, x0}: all centre offsets
        cw_data <= std_logic_vector(co_H) & std_logic_vector(v_hi) & std_logic_vector(v_lo)
                 & std_logic_vector(v_iy) & std_logic_vector(v_ix);
    end process p_cwd;

    -- ====================================================================
    -- Measurement + field trigger: "avid absent 4096 clocks" (~1.9 HD lines
    -- after the last active line; immune to vsync serration / doubled edges)
    -- ====================================================================
    p_meas : process(clk)
    begin
        if rising_edge(clk) then
            r_av   <= data_in.avid;
            r_av1  <= r_av;
            f_trig <= '0';
            if r_av = '1' then
                m_wcnt <= m_wcnt + 1;
                m_noav <= (others => '0');
                if r_av1 = '0' then
                    m_hcnt <= m_hcnt + 1;
                end if;
            else
                if r_av1 = '1' then
                    if m_wcnt /= 0 then m_w <= m_wcnt; end if;
                    m_wcnt <= (others => '0');
                end if;
                if m_noav /= 4095 then
                    m_noav <= m_noav + 1;
                end if;
                if m_noav = 4094 then
                    f_trig <= '1';
                    f_W    <= m_w;
                    f_Hf   <= m_hcnt;
                    m_hcnt <= (others => '0');
                end if;
            end if;
        end if;
    end process p_meas;

    -- ====================================================================
    -- Generated avid (cubist / aurora recipe): start 2-3 clocks after the
    -- source's avid rise at the voted hsync parity, end after the field width.
    -- ====================================================================
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;
            if f_trig = '1' then
                g_wm1 <= f_W - 1;
            end if;

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

            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;

            if g_hsr = '1' then
                bcol <= (others => '0');
            elsif g_avid = '1' then
                bcol <= bcol + 1;
            end if;
        end if;
    end process p_gavid;

    -- ====================================================================
    -- Field setup + generator
    -- ====================================================================
    p_gen : process(clk)
        variable v_t    : unsigned(15 downto 0);
        variable v_z    : unsigned(5 downto 0);
        variable v_raw  : unsigned(9 downto 0);
        variable v_w    : unsigned(3 downto 0);
        variable v_n    : unsigned(8 downto 0);
        variable v_m    : unsigned(4 downto 0);
        variable v_q    : unsigned(13 downto 0);
        variable v_ia, v_ib : unsigned(19 downto 0);
        variable v_th   : unsigned(19 downto 0);
        variable v_e    : unsigned(17 downto 0);
        variable v_base : signed(20 downto 0);
        variable v_pp   : signed(20 downto 0);
        variable v_sum  : signed(21 downto 0);
        variable v_ab   : signed(10 downto 0);
        variable v_neg  : std_logic;
        variable v_p9   : signed(10 downto 0);
        variable v_ai   : unsigned(12 downto 0);
        variable v_bx0, v_bx1, v_by0, v_by1 : signed(10 downto 0);
        variable v_s20  : signed(19 downto 0);
        variable v_dd   : signed(4 downto 0);
    begin
        if rising_edge(clk) then
            gen_done <= '0';

            -- shared envelope step: per point (loop) or per chunk (start),
            -- scaled to the ring density so a swing always decays the same:
            -- env *= 1 - 1.5/N per point -> ~0.22 at the swing's end (an
            -- open core: a deeper decay piles the tail into a dense blob)
            case c_dens is
                when "00"   => if gs = 26 then v_e := shift_right(env, 13) + shift_right(env, 14);
                               else          v_e := shift_right(env, 8)  + shift_right(env, 9);  end if;
                when "01"   => if gs = 26 then v_e := shift_right(env, 12) + shift_right(env, 13);
                               else          v_e := shift_right(env, 8)  + shift_right(env, 9);  end if;
                when "10"   => if gs = 26 then v_e := shift_right(env, 11) + shift_right(env, 12);
                               else          v_e := shift_right(env, 7)  + shift_right(env, 8);  end if;
                when others => if gs = 26 then v_e := shift_right(env, 10) + shift_right(env, 11);
                               else          v_e := shift_right(env, 6)  + shift_right(env, 7);  end if;
            end case;
            v_e := env - v_e;

            case gs is
                when 0 =>
                    null;

                -- ------------------ zones (hysteresis), switches, raster class
                when 1 =>
                    -- labelled knobs load their label INDEX at program load
                    v_raw := unsigned(registers_in(0));
                    if v_raw < 16 then
                        k_zA <= v_raw(3 downto 0);
                    elsif v_raw(9 downto 6) /= k_zA and v_raw(5 downto 0) >= 4
                          and v_raw(5 downto 0) <= 59 then
                        k_zA <= v_raw(9 downto 6);
                    end if;
                    v_raw := unsigned(registers_in(1));
                    v_t := shift_left(resize(v_raw, 16), 5) + resize(v_raw, 16);   -- *33
                    v_z := v_t(15 downto 10);
                    if v_z /= k_zB and v_t(9 downto 0) >= 100 and v_t(9 downto 0) < 924 then
                        k_zB <= v_z;
                    end if;
                    v_raw := unsigned(registers_in(4));
                    if v_raw < 8 then
                        k_pal <= v_raw(2 downto 0);
                    elsif v_raw(9 downto 7) /= k_pal and v_raw(6 downto 0) >= 4
                          and v_raw(6 downto 0) < 124 then
                        k_pal <= v_raw(9 downto 7);
                    end if;
                    v_raw := unsigned(registers_in(5));
                    if v_raw < 8 then
                        k_wz <= v_raw(2 downto 0);
                    elsif v_raw(9 downto 7) /= k_wz and v_raw(6 downto 0) >= 4
                          and v_raw(6 downto 0) < 124 then
                        k_wz <= v_raw(9 downto 7);
                    end if;
                    k_harm  <= registers_in(6)(0);
                    k_eng   <= registers_in(6)(1);
                    k_kal   <= registers_in(6)(2);
                    k_glow  <= registers_in(6)(3);
                    k_vid <= registers_in(6)(4);
                    if f_W >= 1600 then                         -- 1080 class
                        c_cls <= "00";
                        c_cx <= to_unsigned(960, 11);  c_Wm1 <= to_unsigned(1919, 11);
                        c_Wwm1 <= to_unsigned(479, 9); c_rb <= to_unsigned(500, 9);
                        c_cy <= to_unsigned(540, 11);
                        if f_Hf >= 800 then
                            c_il <= '0'; c_pen <= to_unsigned(4, 3);
                        else
                            c_il <= '1'; c_pen <= to_unsigned(2, 3);
                        end if;
                    elsif f_W >= 1000 then                      -- 720p
                        c_cls <= "01";
                        c_cx <= to_unsigned(640, 11);  c_Wm1 <= to_unsigned(1279, 11);
                        c_Wwm1 <= to_unsigned(319, 9); c_rb <= to_unsigned(336, 9);
                        c_cy <= to_unsigned(360, 11);
                        c_il <= '0'; c_pen <= to_unsigned(2, 3);
                    else                                        -- SD
                        c_cls <= "10";
                        c_cx <= to_unsigned(360, 11);  c_Wm1 <= to_unsigned(719, 11);
                        c_Wwm1 <= to_unsigned(179, 9);
                        c_pen <= to_unsigned(2, 3);
                        if f_Hf < 270 then                      -- NTSC fields
                            c_il <= '1'; c_cy <= to_unsigned(243, 11); c_rb <= to_unsigned(226, 9);
                        elsif f_Hf < 400 then                   -- PAL fields
                            c_il <= '1'; c_cy <= to_unsigned(288, 11); c_rb <= to_unsigned(270, 9);
                        elsif f_Hf < 528 then                   -- 480p
                            c_il <= '0'; c_cy <= to_unsigned(240, 11); c_rb <= to_unsigned(224, 9);
                        else                                    -- 576p
                            c_il <= '0'; c_cy <= to_unsigned(288, 11); c_rb <= to_unsigned(270, 9);
                        end if;
                    end if;
                    gs <= 2;

                -- ------------- glides (arm, twist, trace): gl += (raw - gl)/4,
                -- one carry chain per state (10.2 fixed point, rounded)
                when 2 =>
                    gd_a <= signed(unsigned'('0' & unsigned(registers_in(2)) & "10")) - signed('0' & gl_arm);
                    gd_t <= signed(unsigned'('0' & unsigned(registers_in(3)) & "10")) - signed('0' & gl_tw);
                    gd_r <= signed(unsigned'('0' & unsigned(registers_in(7)) & "10")) - signed('0' & gl_tr);
                    gs <= 3;
                when 3 =>
                    gl_arm <= unsigned(signed(gl_arm) + resize(shift_right(gd_a, 2), 12));
                    gl_tw  <= unsigned(signed(gl_tw)  + resize(shift_right(gd_t, 2), 12));
                    gl_tr  <= unsigned(signed(gl_tr)  + resize(shift_right(gd_r, 2), 12));
                    gs <= 5;

                -- --------------------------- gear ratio a:b, ring density
                when 5 =>
                    ga <= resize(k_zA, 5) + 1;
                    if k_zB >= 16 then
                        gb_abs <= resize(k_zB - 16, 5); gb_neg <= '0';
                    else
                        gb_abs <= resize(16 - k_zB, 5); gb_neg <= '1';
                    end if;
                    -- ring size vs the per-row budget: 8192 points for thin
                    -- crisp spiro lines at 1080p, 4096
                    -- otherwise, halved again for thick strokes / 720p glow
                    case c_cls is
                        when "00" =>
                            if k_wz >= 4 then
                                c_dens <= "10";
                            elsif k_wz <= 2 and k_glow = '0' and k_harm = '0' and c_il = '0' then
                                c_dens <= "00";
                            else
                                c_dens <= "01";
                            end if;
                        when "01" =>
                            if k_wz >= 4 and k_glow = '1' then c_dens <= "11";
                            elsif k_wz >= 4 or k_glow = '1' then c_dens <= "10";
                            else c_dens <= "01"; end if;
                        when others =>
                            c_dens <= "11";
                    end case;
                    gs <= 6;
                when 6 =>
                    gx <= ga;
                    if gb_abs = 0 then gy <= ga; else gy <= gb_abs; end if;  -- gcd(a,0) = a
                    case c_dens is
                        when "00"   => c_nchm <= to_unsigned(255, 8); c_nm <= to_unsigned(8191, 13);
                        when "01"   => c_nchm <= to_unsigned(255, 8); c_nm <= to_unsigned(4095, 13);
                        when "10"   => c_nchm <= to_unsigned(127, 8); c_nm <= to_unsigned(2047, 13);
                        when others => c_nchm <= to_unsigned(63, 8);  c_nm <= to_unsigned(1023, 13);
                    end case;
                    gs <= 7;
                when 7 =>                                       -- gcd -> gx
                    if gx = gy then
                        gy  <= ga;  gz <= gb_abs;
                        gca <= (others => '0');  gcb <= (others => '0');
                        gs <= 8;
                    elsif gx > gy then
                        gx <= gx - gy;
                    else
                        gy <= gy - gx;
                    end if;
                when 8 =>                                       -- a/g, |b|/g
                    if gy >= gx then gy <= gy - gx; gca <= gca + 1; end if;
                    if gz >= gx then gz <= gz - gx; gcb <= gcb + 1; end if;
                    if gy < gx and gz < gx then
                        gs <= 9;
                    end if;
                when 9 =>
                    -- harmonograph: frequencies a*k, |b|*k, k ~ 18/max(a,|b|),
                    -- so every swing closes on the ring
                    if ga >= gb_abs then v_m := ga; else v_m := gb_abs; end if;
                    kk <= C_HK(to_integer(v_m - 1));
                    gs <= 10;
                when 10 =>
                    sm_a   <= resize(kk, 21);
                    sm_b   <= resize(ga, 10);
                    sm_acc <= (others => '0');
                    sm_n   <= (others => '0');
                    gs_ret <= 11;
                    gs <= 27;
                when 11 =>
                    tAk    <= sm_acc(9 downto 0);
                    sm_a   <= resize(kk, 21);
                    sm_b   <= resize(gb_abs, 10);
                    sm_acc <= (others => '0');
                    sm_n   <= (others => '0');
                    gs_ret <= 12;
                    gs <= 27;
                when 12 =>
                    -- increments per point, 1 turn = 2^20, ring = 8192 >> dens
                    if k_harm = '1' then
                        v_ia := resize(tAk, 20);  v_ib := resize(sm_acc(9 downto 0), 20);
                    else
                        v_ia := resize(gca, 20);  v_ib := resize(gcb, 20);
                    end if;
                    case c_dens is
                        when "00"   => v_ia := shift_left(v_ia, 7);  v_ib := shift_left(v_ib, 7);
                        when "01"   => v_ia := shift_left(v_ia, 8);  v_ib := shift_left(v_ib, 8);
                        when "10"   => v_ia := shift_left(v_ia, 9);  v_ib := shift_left(v_ib, 9);
                        when others => v_ia := shift_left(v_ia, 10); v_ib := shift_left(v_ib, 10);
                    end case;
                    incA <= v_ia;
                    if gb_neg = '1' then
                        incB <= (not v_ib) + 1;
                    else
                        incB <= v_ib;
                    end if;
                    gs <= 13;
                when 13 =>
                    -- pendulum A's y detune: one extra turn per swing
                    if k_harm = '1' then
                        case c_dens is
                            when "00"   => incYA <= incA + to_unsigned(128, 20);
                            when "01"   => incYA <= incA + to_unsigned(256, 20);
                            when "10"   => incYA <= incA + to_unsigned(512, 20);
                            when others => incYA <= incA + to_unsigned(1024, 20);
                        end case;
                    else
                        incYA <= incA;
                    end if;
                    gs <= 29;
                when 29 =>
                    -- arm radii: R2 = rb * (48 + 7/8 arm) / 1024
                    v_t := resize(shift_left(resize(gl_arm(11 downto 2), 13), 3)
                                  - resize(gl_arm(11 downto 2), 13), 16);
                    sm_b   <= resize(shift_right(v_t, 3), 10) + to_unsigned(48, 10);
                    sm_a   <= resize(c_rb, 21);
                    sm_acc <= (others => '0');
                    sm_n   <= (others => '0');
                    gs_ret <= 14;
                    gs <= 27;
                when 14 =>
                    R2 <= sm_acc(19 downto 10);
                    R1 <= resize(c_rb, 10) - sm_acc(19 downto 10);
                    -- trace: comet length (quadratic); the top ~12% is the
                    -- whole figure blooming, its twist drifting
                    if gl_tr(11 downto 2) > 900 then
                        v_t := resize(gl_tr(11 downto 2) - 900, 16);
                        if v_t(15 downto 3) > 15 then
                            g_bloom <= to_unsigned(15, 4);
                        else
                            g_bloom <= v_t(6 downto 3);
                        end if;
                        nkt256 <= to_unsigned(256, 9);
                        gs <= 16;
                    else
                        g_bloom <= (others => '0');
                        -- tr' = 1.125 tr (900 -> 1012); chunks = tr'^2 / 4096
                        v_t := resize(gl_tr(11 downto 2), 16) + resize(gl_tr(11 downto 5), 16);
                        sm_a   <= resize(v_t, 21);
                        sm_b   <= v_t(9 downto 0);
                        sm_acc <= (others => '0');
                        sm_n   <= (others => '0');
                        gs_ret <= 15;
                        gs <= 27;
                    end if;
                when 15 =>
                    if sm_acc(20 downto 12) < 2 then
                        nkt256 <= to_unsigned(2, 9);
                    elsif sm_acc(20) = '1' then
                        nkt256 <= to_unsigned(256, 9);
                    else
                        nkt256 <= sm_acc(20 downto 12);
                    end if;
                    gs <= 16;

                -- ------------------------------------------ window + pen
                when 16 =>
                    case c_dens is
                        when "10"   => v_n := shift_right(nkt256, 1);
                        when "11"   => v_n := shift_right(nkt256, 2);
                        when others => v_n := nkt256;
                    end case;
                    if v_n = 0 then v_n := to_unsigned(1, 9); end if;
                    if k_kal = '1' then
                        v_n := shift_right(v_n, 3);
                        if v_n = 0 then v_n := to_unsigned(1, 9); end if;
                        ncm1 <= "111";
                        s_nt <= shift_left(v_n, 3);
                    else
                        ncm1 <= "000";
                        s_nt <= v_n;
                    end if;
                    nk <= v_n;
                    if c_dens = "00" then                       -- 32-point chunks
                        nb   <= shift_left(resize(v_n, 14), 5);
                        gend <= shift_left(resize(v_n, 14), 5) + 2;
                    else
                        nb   <= shift_left(resize(v_n, 14), 4);
                        gend <= shift_left(resize(v_n, 14), 4) + 2;
                    end if;
                    if g_bloom /= 0 or nkt256(8) = '1' then
                        b_depth <= (others => '0');
                    else
                        b_depth <= to_unsigned(16, 5) - resize(nkt256(7 downto 4), 5);
                    end if;
                    h_q <= h_q + resize(c_pen, 10);
                    v_w := resize(k_wz, 4) + 1;
                    b_w <= v_w;
                    if c_dens = "00" then s_dense <= '1'; else s_dense <= '0'; end if;
                    if k_glow = '1' then
                        s_reach <= resize(shift_right(resize(v_w, 5) + 8, 1), 4);
                        s_K     <= resize(v_w, 5) + 7;
                        s_cmax  <= to_unsigned(7, 4);
                        s_eth   <= resize(v_w, 5);
                    else
                        s_reach <= resize(shift_right(resize(v_w, 5) + 2, 1), 4) + 1;
                        s_K     <= to_unsigned(15, 5);
                        s_cmax  <= to_unsigned(15, 4);
                        s_eth   <= resize(v_w, 5) + 1;
                    end if;
                    gs <= 17;
                when 17 =>
                    s_ch <= (h_q(9 downto 2) - nk(7 downto 0)) and c_nchm;
                    gs <= 18;
                when 18 =>
                    -- start angles: s_ch * 16 * inc (mod 1 turn)
                    sm_a   <= resize(shift_left(incA, 4), 21);
                    sm_b   <= resize(s_ch, 10);
                    sm_acc <= (others => '0');
                    sm_n   <= (others => '0');
                    gs_ret <= 19;
                    gs <= 27;
                when 19 =>
                    thA    <= sm_acc(19 downto 0);
                    sm_a   <= resize(shift_left(incB, 4), 21);
                    sm_b   <= resize(s_ch, 10);
                    sm_acc <= (others => '0');
                    sm_n   <= (others => '0');
                    gs_ret <= 20;
                    gs <= 27;
                when 20 =>
                    thB <= sm_acc(19 downto 0);
                    -- bloom drifts the twist; it eases back home when off
                    if g_bloom /= 0 then
                        drift <= drift + shift_left(resize(g_bloom, 20), 8);
                    else
                        v_s20 := signed(drift);
                        drift <= unsigned(v_s20 - shift_right(v_s20, 4));
                    end if;
                    gs <= 21;
                when 21 =>
                    thyB <= thB + drift;
                    gs <= 22;
                when 22 =>
                    -- y angles: sin = cos(x - quarter turn); pendulum A detune
                    v_th := thA;
                    if k_harm = '1' then                       -- s_ch * chunk * detune
                        case c_dens is
                            when "10"   => v_th(19 downto 13) := v_th(19 downto 13) + s_ch(6 downto 0);
                            when "11"   => v_th(19 downto 14) := v_th(19 downto 14) + s_ch(5 downto 0);
                            when others => v_th(19 downto 12) := v_th(19 downto 12) + s_ch;
                        end case;
                    end if;
                    v_th(19 downto 18) := v_th(19 downto 18) - 1;
                    thyA <= v_th;
                    v_th := thyB;
                    v_th(19 downto 10) := v_th(19 downto 10) + gl_tw(11 downto 2);   -- twist
                    v_th(19 downto 18) := v_th(19 downto 18) - 1;
                    thyB <= v_th;
                    env   <= C_ENV1;
                    env_n <= s_ch;
                    gs <= 23;
                when 23 =>
                    -- envelope at the window start: per-chunk decay steps
                    if env_n = 0 or k_harm = '0' then
                        gs <= 25;
                    else
                        env   <= v_e;
                        env_n <= env_n - 1;
                    end if;
                when 25 =>
                    gj  <= (others => '0');
                    gt  <= (others => '0');
                    if c_dens = "00" then
                        ai <= shift_left(resize(s_ch, 13), 5);
                    else
                        ai <= shift_left(resize(s_ch, 13), 4);
                    end if;
                    h_acc <= (others => '0');
                    h_cur <= (others => '0');
                    wrap_f <= '0';
                    wrap_b <= '0';
                    wv_act <= '0';
                    ch_act <= '0';
                    gs <= 26;

                -- ------------------------------ generation: 8 slots / point
                when 26 =>
                    gt <= gt + 1;
                    if gt = "111" then
                        gj <= gj + 1;
                    end if;
                    if gt(2) = '0' then
                        sgn_sr <= sgn_sr(2 downto 0) & (g_thsel(19) xor g_thsel(18));
                    else
                        sgn_sr <= sgn_sr(2 downto 0) & '0';
                    end if;

                    -- back-end point q = gj - 1 (live for 1 <= gj <= nb + 1)
                    v_q := gj - 1;

                    -- shared abs (slot 6: X, otherwise Y), shared signed env product
                    -- (slot 0: X, otherwise Y), shared accumulate (slots 4..7)
                    if gt = "110" then
                        v_ab := resize(shift_right(X_acc, 9), 11);
                    else
                        v_ab := resize(shift_right(Y_acc, 9), 11);
                    end if;
                    v_neg := v_ab(10);
                    if v_neg = '1' then v_ab := -v_ab; end if;
                    v_p9 := signed(resize(mul_p(17 downto 9), 11));
                    if (gt = "000" and xsg = '1') or (gt /= "000" and ysg = '1') then
                        v_p9 := -v_p9;
                    end if;
                    -- acc +/- product as ONE adder: {base,1} + {p xor s, s}
                    case gt is
                        when "100" | "110" => v_base := (others => '0');
                        when "101"         => v_base := resize(X_acc, 21);
                        when others        => v_base := resize(Y_acc, 21);
                    end case;
                    v_pp := signed('0' & mul_p);
                    if sgn_sr(3) = '1' then v_pp := not v_pp; end if;
                    v_sum := (v_base & '1') + (v_pp & sgn_sr(3));

                    case gt is
                        when "000" =>
                            if k_harm = '1' then
                                bx    <= v_p9;                  -- env * X of point j-1
                                mul_a <= unsigned(v_ab(9 downto 0));
                                mul_b <= env10v;                -- queue env * Y
                                ysg   <= v_neg;
                            else
                                bx <= resize(shift_right(X_acc, 9), 11);
                            end if;
                        when "010" =>
                            mul_a <= resize(R1, 10);
                            mul_b <= resize(cq_r, 10);
                            if k_harm = '1' then
                                by <= v_p9;
                            else
                                by <= resize(shift_right(Y_acc, 9), 11);
                            end if;
                        when "011" =>
                            mul_a <= resize(R2, 10);
                            mul_b <= resize(cq_r, 10);
                            -- back-end: delta to the tracked (stored) position
                            ddx <= resize(bx, 12) - resize(Px, 12);
                            ddy <= resize(by, 12) - resize(Py, 12);
                            b_q <= v_q;
                            if v_q = 0 then b_first <= '1'; else b_first <= '0'; end if;
                            if v_q = nb or (k_harm = '1' and wrap_b = '1' and v_q /= 0) then
                                b_pen <= '1';
                            else
                                b_pen <= '0';
                            end if;
                            if (c_dens = "00" and v_q(4 downto 0) = 0)
                               or (c_dens /= "00" and v_q(3 downto 0) = 0) then
                                b_chunk <= '1';
                            else
                                b_chunk <= '0';
                            end if;
                            if gj /= 0 and v_q <= nb then b_live <= '1'; else b_live <= '0'; end if;
                        when "100" =>
                            X_acc <= v_sum(20 downto 1);
                            if k_harm = '1' then mul_a <= resize(R2, 10); else mul_a <= resize(R1, 10); end if;
                            mul_b <= resize(cq_r, 10);
                            -- saturate the delta to the store (+-7 dense / +-127) by
                            -- sign-extension checks; -8 / -128 are pen-up codes
                            dX <= f_sat(ddx, c_dens);
                            dY <= f_sat(ddy, c_dens);
                        when "101" =>
                            X_acc <= v_sum(20 downto 1);
                            if k_harm = '1' then mul_a <= resize(R1, 10); else mul_a <= resize(R2, 10); end if;
                            mul_b <= resize(cq_r, 10);
                            env10v <= env(17 downto 8);
                            -- back-end: track the stored position, latch the
                            -- delta of segment q-1 (its write burst starts now)
                            if b_live = '1' then
                                if b_first = '1' or b_pen = '1' then
                                    Px <= bx;  Py <= by;
                                else
                                    Px <= Px + resize(dX, 11);
                                    Py <= Py + resize(dY, 11);
                                end if;
                                wdx  <= dX;
                                wdy  <= dY;
                                wpen <= b_pen;
                                -- dense: an even segment waits for its odd partner
                                if b_q(0) = '1' then
                                    hdx <= dX(3 downto 0);
                                    hdy <= dY(3 downto 0);
                                end if;
                            end if;
                        when "110" =>
                            Y_acc <= v_sum(20 downto 1);
                            if k_harm = '1' then
                                xsg   <= v_neg;
                                mul_a <= unsigned(v_ab(9 downto 0));
                                mul_b <= env10v;
                            end if;
                            -- back-end: chunk bookkeeping on the new position
                            if b_live = '1' then
                                if Px < bxmin then v_bx0 := Px; else v_bx0 := bxmin; end if;
                                if Px > bxmax then v_bx1 := Px; else v_bx1 := bxmax; end if;
                                if Py < bymin then v_by0 := Py; else v_by0 := bymin; end if;
                                if Py > bymax then v_by1 := Py; else v_by1 := bymax; end if;
                                if b_chunk = '1' and b_q /= 0 then
                                    if b_pen = '1' then
                                        co_xmin <= bxmin;  co_xmax <= bxmax;
                                        co_ymin <= bymin;  co_ymax <= bymax;
                                    else
                                        co_xmin <= v_bx0;  co_xmax <= v_bx1;
                                        co_ymin <= v_by0;  co_ymax <= v_by1;
                                    end if;
                                    co_sx <= csx;  co_sy <= csy;
                                    co_H  <= h_cur;
                                    if c_dens = "00" then
                                        ca <= b_q(12 downto 5) - 1;
                                    else
                                        ca <= b_q(11 downto 4) - 1;
                                    end if;
                                    ck <= (others => '0');
                                    ch_act <= '1';
                                    bxmin <= Px;  bxmax <= Px;  bymin <= Py;  bymax <= Py;
                                    csx <= Px;  csy <= Py;
                                elsif b_first = '1' then
                                    bxmin <= Px;  bxmax <= Px;  bymin <= Py;  bymax <= Py;
                                    csx <= Px;  csy <= Py;
                                else
                                    bxmin <= v_bx0;  bxmax <= v_bx1;
                                    bymin <= v_by0;  bymax <= v_by1;
                                end if;
                            end if;
                        when "111" =>
                            Y_acc <= v_sum(20 downto 1);
                            -- advance the pen; every figure closes on the ring
                            thA  <= thA + incA;
                            thyA <= thyA + incYA;
                            thB  <= thB + incB;
                            thyB <= thyB + incB;
                            v_ai := (ai + 1) and c_nm;
                            ai <= v_ai;
                            wrap_b <= wrap_f;
                            if v_ai = 0 then
                                wrap_f <= '1';
                                env <= C_ENV1;                  -- harmonograph: new swing
                            else
                                wrap_f <= '0';
                                env <= v_e;
                            end if;
                            if v_q = gend then
                                gs <= 28;
                            end if;
                        when others =>
                            null;
                    end case;

                    -- vertex write burst (combinational port): copies 0..ncm1;
                    -- dense rings write a word per segment PAIR (odd segment)
                    if wv_act = '1' then
                        if c_dens = "00" then
                            va <= va + resize(nb(13 downto 1), 12);
                        else
                            va <= va + resize(nb, 12);
                        end if;
                        wk <= wk + 1;
                        if wk = ncm1 then
                            wv_act <= '0';
                        end if;
                    end if;
                    if gt = "101" and b_live = '1' and b_first = '0'
                       and (c_dens /= "00" or b_q(0) = '0') then
                        wv_act <= '1';
                        wk <= (others => '0');
                        if c_dens = "00" then
                            va <= resize(shift_right(b_q - 1, 1), 12);
                        else
                            va <= resize(b_q - 1, 12);
                        end if;
                    end if;

                    -- chunk age = floor(16 c / nk): +16 per chunk, then count
                    -- off nk per clock (<= 16 steps, a chunk takes 128 clocks)
                    if gt = "110" and b_live = '1' and b_chunk = '1' and b_q /= 0 then
                        h_acc <= h_acc + 16;
                    elsif h_acc >= nk then
                        h_acc <= h_acc - nk;
                        h_cur <= h_cur + 1;
                    end if;

                    -- chunk write burst (combinational port)
                    if ch_act = '1' then
                        ca <= ca + nk(7 downto 0);
                        ck <= ck + 1;
                        if ck = ncm1 then
                            ch_act <= '0';
                        end if;
                    end if;

                when 27 =>
                    -- shared serial multiplier: acc += a * b over 10 steps
                    if sm_b(0) = '1' then sm_acc <= sm_acc + sm_a; end if;
                    sm_a <= shift_left(sm_a, 1);
                    sm_b <= shift_right(sm_b, 1);
                    sm_n <= sm_n + 1;
                    if sm_n = 9 then
                        gs <= gs_ret;
                    end if;

                when 28 =>
                    gen_done <= '1';
                    gs <= 0;

                when others =>
                    gs <= 0;
            end case;

            -- a new field (re)starts the setup
            if f_trig = '1' then
                gs <= 1;
                wv_act <= '0';
                ch_act <= '0';
            end if;
        end if;
    end process p_gen;

    -- ====================================================================
    -- Row controller + renderer (prescan, vertex scan, jobs, stamper)
    -- ====================================================================
    p_render : process(clk)
        variable v_start : std_logic;
        variable v_row   : unsigned(10 downto 0);
        variable v_take  : boolean;
        variable v_hbfree : boolean;
        variable v_hit   : boolean;
        variable v_sx, v_sy, v_ex, v_ey : signed(11 downto 0);
        variable v_dx, v_dy : signed(7 downto 0);
        variable v_b8    : std_logic_vector(7 downto 0);
        variable v_ax8, v_ay8, v_or8 : unsigned(7 downto 0);
        variable v_t12, v_t13 : signed(12 downto 0);
        variable v_ex7   : signed(11 downto 0);
        variable v_ey7   : signed(11 downto 0);
        variable v_E0    : signed(15 downto 0);
        variable v_push, v_pop : boolean;
        variable v_ent   : std_logic_vector(47 downto 0);
        variable v_u     : unsigned(9 downto 0);
        variable v_mag   : signed(15 downto 0);
        variable v_c6    : signed(5 downto 0);
        variable v_old   : std_logic_vector(23 downto 0);
        variable v_new   : std_logic_vector(23 downto 0);
        variable v_o, v_n : unsigned(4 downto 0);
        variable v_busy  : boolean;
        variable v_cnt   : unsigned(1 downto 0);
    begin
        if rising_edge(clk) then
            -- the start decided last clock (registered: the flush fans out wide)
            v_start := st_go;
            v_row   := st_row;

            -- ------------------------------------------- row controller
            st_go <= '0';
            if f_trig = '1' then
                vis_row <= (others => '0');
                vr2     <= to_unsigned(2, 11);
                rc      <= 0;
                pend    <= '0';
                e_run   <= '0';
            else
                case rc is
                    when 0 =>                                   -- wait generator
                        if gen_done = '1' then
                            st_go <= '1';  st_row <= (others => '0');
                            rc <= 1;
                        end if;
                    when 1 =>
                        if e_run = '0' and st_go = '0' and v_start = '0' then
                            st_go <= '1';  st_row <= to_unsigned(1, 11);
                            rc <= 2;
                        end if;
                    when 2 =>
                        if e_run = '0' and st_go = '0' and v_start = '0' then
                            rc <= 3;
                        end if;
                    when others =>
                        if ev_line = '1' then
                            vis_row  <= vis_row + 1;
                            vr2      <= vr2 + 1;
                            pend     <= '1';
                            pend_row <= vr2;
                            if vr2 < f_Hf then pend_ok <= '1'; else pend_ok <= '0'; end if;
                            e_run    <= '0';                    -- abort an overrun
                        elsif pend = '1' and cl_run = '0' then
                            pend <= '0';
                            if pend_ok = '1' then
                                st_go <= '1';  st_row <= pend_row;
                            end if;
                        end if;
                end case;
            end if;
            -- the row's reach window follows the registered row offset
            e_Rlo <= e_R - signed(resize(s_reach, 13));
            e_Rhi <= e_R + signed(resize(s_reach, 13));

            -- ------------------------------------------------ prescan
            p1_v <= '0';
            if e_run = '1' and ps_c < s_nt then
                cr_addr <= ps_c(7 downto 0);
                p1_v <= '1';
                p1_c <= ps_c(7 downto 0);
                ps_c <= ps_c + 1;
            end if;
            p2_v <= p1_v;  p2_c <= p1_c;
            p3_v <= p2_v;  p3_c <= p2_c;
            cr_r <= cr_q;
            -- test + hand-off (cr_r holds the entry of p3)
            v_hit := p3_v = '1' and e_run = '1'
                     and resize(signed(cr_r(32 downto 22)), 13) <= e_Rhi
                     and resize(signed(cr_r(43 downto 33)), 13) >= e_Rlo;
            v_take := (stall = '0') and (vs_act = '0') and (hb_full = '1') and (e_run = '1');
            v_hbfree := (hb_full = '0') or v_take;
            if v_hit then
                if v_hbfree then
                    hb_full <= '1';
                    hb_c    <= p3_c;
                    hb_e    <= cr_r(47 downto 44) & cr_r(21 downto 0);
                else
                    -- hit buffer busy: rewind the prescan to this chunk
                    ps_c <= '0' & p3_c;
                    p1_v <= '0';  p2_v <= '0';  p3_v <= '0';
                end if;
            elsif v_take then
                hb_full <= '0';
            end if;

            -- ------------------------------------------- vertex issue
            if stall = '0' then
                t1_v <= '0';
                if e_run = '1' then
                    if vs_act = '1' then
                        if s_dense = '1' then
                            vr_addr <= cur_c & vs_cnt(4 downto 1);
                            t1_h <= vs_cnt(0);
                        else
                            vr_addr <= cur_c & vs_cnt(3 downto 0);
                            t1_h <= '0';
                        end if;
                        t1_v <= '1';
                        t1_f <= '0';
                        vs_cnt <= vs_cnt + 1;
                        if (s_dense = '1' and vs_cnt = 31) or (s_dense = '0' and vs_cnt = 15) then
                            vs_act <= '0';
                        end if;
                    elsif v_take then
                        cur_c   <= hb_c;
                        cur_e   <= hb_e;
                        vr_addr <= hb_c & "0000";
                        t1_v <= '1';
                        t1_f <= '1';
                        t1_h <= '0';
                        vs_cnt <= to_unsigned(1, 5);
                        vs_act <= '1';
                    end if;
                end if;
                t2_v <= t1_v;  t2_f <= t1_f;  t2_h <= t1_h;
                t3_v <= t2_v;  t3_f <= t2_f;  t3_h <= t2_h;
                -- c2: delta out of the RAM; the direction ROM address goes out
                -- now so {|u|, |v|} land in uv_r with the multiply (c5).  The
                -- ROM is indexed by 4-bit magnitudes: wide deltas are scaled
                -- down by a power of two (direction only), |d| ~ d xor sign.
                vd <= vr_q;
                if s_dense = '1' then
                    if t2_h = '1' then v_b8 := vr_q(7 downto 0); else v_b8 := vr_q(15 downto 8); end if;
                    v_ax8 := unsigned(resize(abs(signed(v_b8(7 downto 4))), 8));
                    v_ay8 := unsigned(resize(abs(signed(v_b8(3 downto 0))), 8));
                else
                    v_ax8 := unsigned(vr_q(15 downto 8));
                    if vr_q(15) = '1' then v_ax8 := not v_ax8; end if;
                    v_ay8 := unsigned(vr_q(7 downto 0));
                    if vr_q(7) = '1' then v_ay8 := not v_ay8; end if;
                end if;
                v_or8 := v_ax8 or v_ay8;
                if v_or8(6) = '1' then
                    uv_addr <= v_ax8(6 downto 3) & v_ay8(6 downto 3);
                elsif v_or8(5) = '1' then
                    uv_addr <= v_ax8(5 downto 2) & v_ay8(5 downto 2);
                elsif v_or8(4) = '1' then
                    uv_addr <= v_ax8(4 downto 1) & v_ay8(4 downto 1);
                else
                    uv_addr <= v_ax8(3 downto 0) & v_ay8(3 downto 0);
                end if;
            end if;

            -- ----- c3: integrate (vd = delta of t3; a chunk's first point
            -- takes the chunk start from cur_e, still that chunk's tag),
            -- extents by delta sign, row offset
            if stall_b = '0' then
                if t3_f = '1' then
                    v_sx := signed('0' & c_cx) + resize(signed(cur_e(10 downto 0)), 12);
                    v_sy := resize(signed(cur_e(21 downto 11)), 12);
                    int_H <= cur_e(25 downto 22);
                    s5_H  <= cur_e(25 downto 22);
                else
                    v_sx := ix;
                    v_sy := iy;
                    s5_H  <= int_H;
                end if;
                if s_dense = '1' then
                    if t3_h = '1' then v_b8 := vd(7 downto 0); else v_b8 := vd(15 downto 8); end if;
                    v_dx := resize(signed(v_b8(7 downto 4)), 8);
                    v_dy := resize(signed(v_b8(3 downto 0)), 8);
                    if v_b8(7 downto 4) = "1000" then s5_pen <= '1'; else s5_pen <= '0'; end if;
                else
                    v_dx := signed(vd(15 downto 8));
                    v_dy := signed(vd(7 downto 0));
                    if vd(15 downto 8) = x"80" then s5_pen <= '1'; else s5_pen <= '0'; end if;
                end if;
                v_ex := v_sx + resize(v_dx, 12);
                v_ey := v_sy + resize(v_dy, 12);
                if t3_v = '1' then
                    ix <= v_ex;
                    iy <= v_ey;
                end if;
                s5_v <= t3_v;
                if v_dx(7) = '1' then
                    s5_xlo <= v_ex;  s5_xhi <= v_sx;
                else
                    s5_xlo <= v_sx;  s5_xhi <= v_ex;
                end if;
                if v_dy(7) = '1' then
                    s5_ylo <= v_ey;  s5_yhi <= v_sy;
                else
                    s5_ylo <= v_sy;  s5_yhi <= v_ey;
                end if;
                s5_sx  <= v_sx;
                s5_eyR <= resize(e_R, 12) - v_sy;
                s5_sdx <= v_dx(7);
                s5_sdy <= v_dy(7);

                -- c4: row hit, word range (4-pixel words, signed: a segment
                -- may start left of the screen -- the iterator skips those
                -- words), |x offset| of the first word's pixel 0, |row
                -- offset|, sign of the |ey||v| term
                s6_v <= s5_v;
                v_t12 := resize(s5_xlo, 13) - signed(resize(s_reach, 13));
                v_t13 := resize(s5_xhi, 13) + signed(resize(s_reach, 13));
                if s5_v = '1' and s5_pen = '0'
                   and resize(s5_ylo, 13) <= e_Rhi and resize(s5_yhi, 13) >= e_Rlo
                   and v_t13 >= 0 and v_t12 <= signed(resize(c_Wm1, 13)) then
                    s6_act <= '1';
                else
                    s6_act <= '0';
                end if;
                s6_wl <= v_t12(11 downto 2);
                v_ex7 := resize(s5_sx - (v_t12(11 downto 0) and to_signed(-4, 12)), 12);
                if v_ex7(11 downto 7) /= 0 then
                    s6_aex <= to_unsigned(127, 8);
                else
                    s6_aex <= unsigned(v_ex7(7 downto 0));
                end if;
                if v_t13 > signed(resize(c_Wm1, 13)) then
                    s6_wh <= signed(resize(c_Wwm1, 10));
                else
                    s6_wh <= v_t13(11 downto 2);
                end if;
                v_ey7 := s5_eyR;
                if v_ey7(11) = '1' then v_ey7 := -v_ey7; end if;
                if v_ey7(11 downto 7) /= 0 then
                    s6_aey <= to_unsigned(127, 8);
                else
                    s6_aey <= unsigned(v_ey7(7 downto 0));
                end if;
                s6_par <= s5_sdy xor s5_eyR(11) xor s5_sdx;
                s6_H   <= s5_H;
            end if;

            if stall_c = '0' then
                -- c5: magnitude products (uv_r = {|u|, |v|} of this segment)
                s8_v <= s6_v;  s8_act <= s6_act;  s8_par <= s6_par;
                s8_wl <= s6_wl;  s8_wh <= s6_wh;
                s8_pu <= s6_aex * uv_r(15 downto 8);
                s8_pv <= s6_aey * uv_r(7 downto 0);
                s8_um <= uv_r(15 downto 8);
                s8_H  <= s6_H;
            end if;

            -- c6: E0 (Q6, 2*distance) at the first word's pixel 0 straight
            -- into the job FIFO; pixel k then sits at E0 - k|u|
            if s8_par = '1' then
                v_E0 := signed('0' & s8_pu(14 downto 0)) - signed('0' & s8_pv(14 downto 0));
            else
                v_E0 := signed('0' & s8_pu(14 downto 0)) + signed('0' & s8_pv(14 downto 0));
            end if;
            v_push := (stall_d = '0') and (s8_v = '1') and (s8_act = '1') and (e_run = '1');
            v_pop  := false;
            if v_push then
                if f_wp = '0' then
                    f_mem(0) <= std_logic_vector(s8_wl) & std_logic_vector(s8_wh)
                              & std_logic_vector(v_E0) & std_logic_vector(s8_um) & s8_H;
                else
                    f_mem(1) <= std_logic_vector(s8_wl) & std_logic_vector(s8_wh)
                              & std_logic_vector(v_E0) & std_logic_vector(s8_um) & s8_H;
                end if;
                f_wp <= not f_wp;
            end if;

            -- ------------------------------------------- word iterator
            if f_rp = '0' then v_ent := f_mem(0); else v_ent := f_mem(1); end if;
            w1_v <= '0';
            if it_v = '1' then
                w1_v <= not it_w(9);                       -- left of the screen: skip
                w1_a <= unsigned(it_w(8 downto 0));
                w1_H <= it_H;
                w1_E(0) <= it_E;
                w1_E(1) <= it_E - signed(resize(it_u1, 16));
                w1_E(2) <= it_E - signed(resize(shift_left(it_u1, 1), 16));
                w1_E(3) <= it_E - signed(resize(it_u3, 16));
                if it_w = it_wh then
                    it_v <= '0';
                else
                    it_w <= it_w + 1;
                    it_E <= it_E - signed(resize(it_u4, 16));
                end if;
            end if;
            if (it_v = '0' or it_w = it_wh) and f_cnt /= 0 then
                v_pop := true;
                f_rp  <= not f_rp;
                it_v  <= '1';
                it_w  <= signed(v_ent(47 downto 38));
                it_wh <= signed(v_ent(37 downto 28));
                it_E  <= signed(v_ent(27 downto 12));
                v_u   := resize(unsigned(v_ent(11 downto 4)), 10);
                it_u1 <= v_u;
                it_u3 <= v_u + shift_left(v_u, 1);
                it_u4 <= shift_left(v_u, 2);
                it_H  <= v_ent(3 downto 0);
            end if;
            v_cnt := f_cnt;
            if v_push then v_cnt := v_cnt + 1; end if;
            if v_pop  then v_cnt := v_cnt - 1; end if;
            f_cnt <= v_cnt;
            if v_cnt >= 2 then
                stall <= '1';  stall_b <= '1';  stall_c <= '1';  stall_d <= '1';
            else
                stall <= '0';  stall_b <= '0';  stall_c <= '0';  stall_d <= '0';
            end if;

            -- ------------------------------------------------ stamper
            -- W1: |E| -> 2*distance in half pixels (16 = beyond reach)
            w2_v <= w1_v;  w2_a <= w1_a;  w2_H <= w1_H;
            for k in 0 to 3 loop
                if w1_E(k)(15) = '1' then
                    v_mag := not w1_E(k);
                else
                    v_mag := w1_E(k);
                end if;
                if v_mag(14 downto 10) /= 0 then
                    w2_e(k) <= to_unsigned(16, 5);
                else
                    w2_e(k) <= '0' & unsigned(v_mag(9 downto 6));
                end if;
            end loop;

            -- W2: closeness profile, age-take flags, forwarding selects
            w3_v <= w2_v;  w3_a <= w2_a;  w3_H <= w2_H;
            for k in 0 to 3 loop
                v_c6 := signed('0' & s_K) - signed('0' & w2_e(k));
                if v_c6 < 0 then
                    w3_c(k) <= (others => '0');
                elsif v_c6 > signed(unsigned'("00" & s_cmax)) then
                    w3_c(k) <= s_cmax;
                else
                    w3_c(k) <= unsigned(v_c6(3 downto 0));
                end if;
            end loop;
            for p in 0 to 1 loop
                if w2_e(2 * p) <= s_eth or w2_e(2 * p + 1) <= s_eth then
                    w3_tk(p) <= '1';
                else
                    w3_tk(p) <= '0';
                end if;
                if w2_e(2 * p) < s_K or w2_e(2 * p + 1) < s_K then
                    w3_any(p) <= '1';
                else
                    w3_any(p) <= '0';
                end if;
            end loop;
            if w3_v = '1' and w3_a = w2_a then w3_s1 <= '1'; else w3_s1 <= '0'; end if;
            if st_we = '1' and st_wa = w2_a then w3_s2 <= '1'; else w3_s2 <= '0'; end if;
            if h1_v = '1' and h1_a = w2_a then w3_s3 <= '1'; else w3_s3 <= '0'; end if;
            if rbank = '0' then rd_r <= l0_q; else rd_r <= l1_q; end if;

            -- W3: read-modify (forwarded) -> write.  Glow adds light across
            -- different parts of the figure (age differs) but takes the max
            -- within one stroke, so a polyline doesn't double at its joints.
            if w3_s1 = '1' then
                v_old := st_wd;
            elsif w3_s2 = '1' then
                v_old := h1_d;
            elsif w3_s3 = '1' then
                v_old := h2_d;
            else
                v_old := rd_r;
            end if;
            for k in 0 to 3 loop
                v_o := '0' & unsigned(v_old(4 * k + 3 downto 4 * k));
                v_n := '0' & w3_c(k);
                if k_glow = '1' and v_old(16 + 4 * (k / 2) + 3 downto 16 + 4 * (k / 2)) /= w3_H then
                    v_o := v_o + v_n;
                    if v_o(4) = '1' then
                        v_new(4 * k + 3 downto 4 * k) := "1111";
                    else
                        v_new(4 * k + 3 downto 4 * k) := std_logic_vector(v_o(3 downto 0));
                    end if;
                else
                    if v_n > v_o then
                        v_new(4 * k + 3 downto 4 * k) := std_logic_vector(v_n(3 downto 0));
                    else
                        v_new(4 * k + 3 downto 4 * k) := std_logic_vector(v_o(3 downto 0));
                    end if;
                end if;
            end loop;
            for p in 0 to 1 loop
                if w3_tk(p) = '1' or (w3_any(p) = '1'
                                      and v_old(8 * p + 7 downto 8 * p) = x"00") then
                    v_new(16 + 4 * p + 3 downto 16 + 4 * p) := w3_H;
                else
                    v_new(16 + 4 * p + 3 downto 16 + 4 * p) := v_old(16 + 4 * p + 3 downto 16 + 4 * p);
                end if;
            end loop;
            st_we <= w3_v;
            st_wa <= w3_a;
            st_wd <= v_new;
            h1_v <= st_we;  h1_a <= st_wa;  h1_d <= st_wd;
            h2_v <= h1_v;   h2_a <= h1_a;   h2_d <= h1_d;

            -- ---------------------------------------- done / (re)start
            v_busy := (ps_c < s_nt) or p1_v = '1' or p2_v = '1' or p3_v = '1'
                      or hb_full = '1' or vs_act = '1' or t1_v = '1' or t2_v = '1' or t3_v = '1'
                      or s5_v = '1' or s6_v = '1' or s8_v = '1'
                      or f_cnt /= 0 or it_v = '1'
                      or w1_v = '1' or w2_v = '1' or w3_v = '1' or st_we = '1';
            if e_run = '1' and not v_busy then
                e_run <= '0';
            end if;

            if v_start = '1' or e_run = '0' then
                -- idle / restart: flush every stage
                p1_v <= '0';  p2_v <= '0';  p3_v <= '0';
                hb_full <= '0';
                vs_act <= '0';
                t1_v <= '0';  t2_v <= '0';  t3_v <= '0';
                s5_v <= '0';  s6_v <= '0';  s8_v <= '0';
                f_cnt <= (others => '0');  f_wp <= '0';  f_rp <= '0';
                it_v <= '0';
                w1_v <= '0';  w2_v <= '0';  w3_v <= '0';
                st_we <= '0';  h1_v <= '0';  h2_v <= '0';
                stall <= '0';  stall_b <= '0';  stall_c <= '0';  stall_d <= '0';
                ps_c <= (others => '0');
            end if;
            if v_start = '1' then
                e_run <= '1';
                rbank <= v_row(0);
                -- row in centre-offset frame coordinates (fields: even lines)
                if c_il = '1' then
                    v_t12 := signed(unsigned'("0" & v_row(10 downto 0) & "0")) - signed(resize(c_cy, 13));
                else
                    v_t12 := signed(unsigned'("00" & v_row)) - signed(resize(c_cy, 13));
                end if;
                e_R   <= v_t12;
            end if;
        end if;
    end process p_render;

    -- ====================================================================
    -- Beam: clear-behind, line events, readout pipeline, output
    -- ====================================================================
    p_beam : process(clk)
        variable v_wd   : std_logic_vector(23 downto 0);
        variable v_lx   : unsigned(7 downto 0);
        variable v_wp   : unsigned(11 downto 0);
        variable v_te   : signed(8 downto 0);
        variable v_f    : unsigned(8 downto 0);
        variable v_fl   : unsigned(8 downto 0);
        variable v_ax   : unsigned(8 downto 0);
        variable v_yi   : signed(10 downto 0);
        variable v_ci   : signed(10 downto 0);
        variable v_y, v_c : signed(12 downto 0);
        variable v_hot  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- ---------------------------- clear behind the beam + events
            gav_d   <= g_avid;
            ev_line <= '0';
            clr_we  <= '0';
            if g_avid = '1' and bcol(1 downto 0) = "11" then
                clr_we   <= '1';
                clr_addr <= bcol(10 downto 2);
                clr_bank <= vis_row(0);
            elsif cl_run = '1' then
                clr_we   <= '1';
                clr_addr <= cl_ptr;
                clr_bank <= cl_bank;
                if cl_ptr >= c_Wwm1 then
                    cl_run <= '0';
                else
                    cl_ptr <= cl_ptr + 1;
                end if;
            end if;
            if gav_d = '1' and g_avid = '0' then
                ev_line <= '1';
                -- a short line leaves unread words: clear them too
                if bcol(10 downto 2) <= c_Wwm1 then
                    cl_run  <= '1';
                    cl_ptr  <= bcol(10 downto 2);
                    cl_bank <= vis_row(0);
                end if;
            end if;

            -- ------------------------------------------- video luma tap
            vy1 <= unsigned(data_in.y);
            vy2 <= vy1;

            -- S0 -> S1
            s1_p  <= bcol(1 downto 0);
            s1_av <= g_avid;
            s1_bk <= vis_row(0);

            -- S1 -> S3: this pixel's pair (closeness/light + age), picked
            -- straight off the line-buffer output (selects only, no logic)
            if s1_bk = '0' then v_wd := l0_q; else v_wd := l1_q; end if;
            if s1_p(1) = '0' then
                s3_Ie <= unsigned(v_wd(3 downto 0));   s3_Io <= unsigned(v_wd(7 downto 4));
                s3_H  <= unsigned(v_wd(19 downto 16));
            else
                s3_Ie <= unsigned(v_wd(11 downto 8));  s3_Io <= unsigned(v_wd(15 downto 12));
                s3_H  <= unsigned(v_wd(23 downto 20));
            end if;
            s3_pe <= not s1_p(0);
            s3_av <= s1_av;
            v_lx := vy2(9 downto 2);                            -- ~luma (black ~16)
            s3_lum <= v_lx;

            -- S3 -> S4: engrave weight (Q2 px)
            s4_av <= s3_av;  s4_pe <= s3_pe;
            s4b_Ie <= s3_Ie;  s4b_Io <= s3_Io;  s4b_H <= s3_H;
            s4_lum4 <= s3_lum(7 downto 4);
            if k_eng = '1' then
                v_wp := b_w * s3_lum;
                s4_weff <= resize(v_wp(11 downto 6), 7);
            else
                s4_weff <= resize(b_w & "00", 7);
            end if;

            -- S4 -> S5: coverage (0..8) per pair pixel, hot overlap, fade
            s5_av <= s4_av;  s5_pe <= s4_pe;  s5_lum4 <= s4_lum4;
            if k_glow = '1' then
                if s4b_Ie > 7 then
                    s5_ae <= x"8";  s5_he <= s4b_Ie - 7;
                else
                    s5_ae <= s4b_Ie + 1 - resize(s4b_Ie(3 downto 3), 4);
                    s5_he <= (others => '0');
                end if;
                if s4b_Io > 7 then
                    s5_ao <= x"8";  s5_ho <= s4b_Io - 7;
                else
                    s5_ao <= s4b_Io + 1 - resize(s4b_Io(3 downto 3), 4);
                    s5_ho <= (others => '0');
                end if;
                if s4b_Ie = 0 then s5_ae <= (others => '0'); end if;
                if s4b_Io = 0 then s5_ao <= (others => '0'); end if;
            else
                v_te := signed(resize(s4b_Ie & "00", 9)) + signed(resize(s4_weff, 9)) - 54;
                if v_te < 0 then s5_ae <= (others => '0');
                elsif v_te > 4 then s5_ae <= x"8";
                else s5_ae <= unsigned(v_te(2 downto 0) & '0'); end if;
                v_te := signed(resize(s4b_Io & "00", 9)) + signed(resize(s4_weff, 9)) - 54;
                if v_te < 0 then s5_ao <= (others => '0');
                elsif v_te > 4 then s5_ao <= x"8";
                else s5_ao <= unsigned(v_te(2 downto 0) & '0'); end if;
                s5_he <= (others => '0');
                s5_ho <= (others => '0');
            end if;
            v_f := (not s4b_H) * b_depth;                        -- (15 - H) * depth
            s5_fade <= to_unsigned(16, 5) - resize(v_f(8 downto 4), 5);

            -- S5 -> S6: palette ink, pair-shared coverage.  Chroma: the 4:2:2
            -- packer keeps U from the even pixel and V from the odd one, so
            -- one blend carries U on even pixels and V on odd ones.
            s6_av <= s5_av;
            s6_pe <= s5_pe;
            if s5_pe = '1' then
                s6_apx <= s5_ae;  s6_hpx <= s5_he;
            else
                s6_apx <= s5_ao;  s6_hpx <= s5_ho;
            end if;
            s6_apr <= resize(shift_right(resize(s5_ae, 5) + resize(s5_ao, 5) + 1, 1), 4);
            if s5_he > s5_ho then s6_hpr <= s5_he; else s6_hpr <= s5_ho; end if;
            if k_eng = '1' and k_glow = '1' then
                v_fl := s5_fade * s5_lum4;
                s6_fl <= v_fl(8 downto 4);
            else
                s6_fl <= s5_fade;
            end if;
            -- the stroke's light above black; on Video it is added over the
            -- picture (the ground swaps in at S7)
            v_yi := signed(unsigned'("0" & py_r(15 downto 8) & "00"))
                    + signed(resize(g_bloom * to_unsigned(10, 4), 11)) - 64;
            if s5_pe = '1' then
                v_ci := shift_left(resize(signed(pc_r(15 downto 8)), 11), 2);
            else
                v_ci := shift_left(resize(signed(pc_r(7 downto 0)), 11), 2);
            end if;
            s6_dY <= v_yi;
            s6_dC <= v_ci;

            -- S6 -> S7: coverage x fade (0..16)
            s7_av <= s6_av;
            s7_pe <= s6_pe;
            v_ax := s6_apx * s6_fl;
            s7_axp <= v_ax(7 downto 3);
            v_ax := s6_apr * s6_fl;
            s7_axr <= v_ax(7 downto 3);
            s7_hpx <= s6_hpx;  s7_hpr <= s6_hpr;
            s7_dY <= s6_dY;  s7_dC <= s6_dC;

            -- S7 -> S8: blend products; ground (+ hot glow overlaps) ready.
            -- Video ground: the input tapped here, late (no deep raw-video
            -- carry: it sits ~10 px left of the source; black where the
            -- source line has already ended)
            s8_av <= s7_av;
            s8_pY <= s7_dY * signed('0' & s7_axp);
            s8_pC <= s7_dC * signed('0' & s7_axr);
            v_hot := resize(signed(unsigned'('0' & s7_hpx & "0000")), 13)
                     + resize(signed(unsigned'('0' & s7_hpx & "00")), 13);       -- hot*20
            if k_vid = '1' and data_in.avid = '1' then
                s8_yg <= signed(unsigned'("000" & unsigned(data_in.y))) + v_hot;
                if s7_pe = '1' then
                    s8_cg <= signed(unsigned'("000" & unsigned(data_in.u)));
                else
                    s8_cg <= signed(unsigned'("000" & unsigned(data_in.v)));
                end if;
            else
                s8_yg <= to_signed(64, 13) + v_hot;
                s8_cg <= to_signed(512, 13);
            end if;
            if s7_hpr >= 4 then s8_csh <= '1'; else s8_csh <= '0'; end if;

            -- S8 -> out: ground + blend, clamp, blanking gate
            v_y := s8_yg + resize(shift_right(s8_pY, 4), 13);
            if s8_csh = '1' then
                v_c := s8_cg + resize(shift_right(s8_pC, 5), 13);
            else
                v_c := s8_cg + resize(shift_right(s8_pC, 4), 13);
            end if;
            if v_y < 64 then v_y := to_signed(64, 13); elsif v_y > 940 then v_y := to_signed(940, 13); end if;
            if v_c < 64 then v_c := to_signed(64, 13); elsif v_c > 960 then v_c := to_signed(960, 13); end if;
            if s8_av = '1' then
                o_y <= unsigned(v_y(9 downto 0));
                o_c <= unsigned(v_c(9 downto 0));
            else
                o_y <= to_unsigned(64, 10);
                o_c <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_beam;

    -- ====================================================================
    -- Sync delay (C_L, every mode) + output
    -- ====================================================================
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            d_hs(1) <= data_in.hsync_n;
            d_vs(1) <= data_in.vsync_n;
            d_fl(1) <= data_in.field_n;
            d_gv(1) <= g_avid;
            for i in 2 to C_L loop
                d_hs(i) <= d_hs(i - 1);
                d_vs(i) <= d_vs(i - 1);
                d_fl(i) <= d_fl(i - 1);
                d_gv(i) <= d_gv(i - 1);
            end loop;
        end if;
    end process p_delay;

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(o_c);
    data_out.v       <= std_logic_vector(o_c);
    data_out.hsync_n <= d_hs(C_L);
    data_out.vsync_n <= d_vs(C_L);
    data_out.field_n <= d_fl(C_L);
    data_out.avid    <= d_gv(C_L);

end architecture spiroscope;
