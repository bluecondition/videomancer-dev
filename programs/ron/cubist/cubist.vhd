-- cubist.vhd
--
-- CUBIST v0.1 -- a photoreal-clean Rubik's cube floating on a soft backdrop.
-- This stage: static SOLVED cube, whole-cube rotation on K1/K2/K3
-- (yaw/pitch/roll, glide-smoothed), auto-zoom framing, satin-black body with
-- rounded cubies, inset rounded stickers, gap AO, silhouette AA with rounded
-- outline corners, gradient+vignette backdrop.  Layer turns, specular rig,
-- MANUAL/AUTO modes and the solver stack on top of this datapath in later
-- stages (see SUMMARY.md).
--
-- ARCHITECTURE (streaming, no line buffer):
--   FRAME FSM (vblank): glide the three angles, rotate the cube basis
--     (quarter-sine ROM + shared multiplier), project the 8 corners
--     (serial divider), auto-zoom, backface-cull the 6 faces by screen
--     winding, and for each visible face (<= 3) write a record to the
--     GEOMETRY EBR: screen edge table + the projective inverse vectors
--       A = (C-O) x V,   B = U x (C-O),   N = U x V     (block-float/face)
--     so that   u = d.A / d.N,  v = d.B / d.N   with  d = (px, -py, -f).
--     uw/vw/w are LINEAR in screen x -- the whole perspective problem
--     collapses to two multiplies per attribute per line + one serial
--     divide per 64 px node (128 px on SD: an 858-clock line budget).
--   LINE FSM (runs during line y for line y+1): reads the geometry EBR,
--     intersects the <=3 spans, walks each span in 64 px segments emitting
--     5-word records (x, u0, v0, du, dv) into the double-buffered DISPLAY
--     LIST EBR (halves by line parity).  Spans are padded +/-2 px so
--     silhouette AA can ramp out; min segment spacing ~32 px guarantees
--     the pixel-side prefetcher always has time.
--   PIXEL PIPE (C_LATENCY stages): ONE u/v DDA fed by the display list,
--     rounded-rect SDFs for sticker / cubie gaps / face bevel + silhouette
--     (all sqrt-free: squared distances linearized near the zero crossing,
--     pipedream band trick), per-face flat lighting (v0.1), backdrop from
--     pure second-difference accumulators, alpha compose, blanking gate.
--
-- Q formats: angles 12b/turn; sin/cos Q12; basis vectors Q2.12; cube spans
-- +/-1.5 units (one cubie = 4096); camera at D = 13.0 (Q12).  UV in Q12
-- cubie units, 0..12288 across a face.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture cubist of program_top is

    constant C_LATENCY : integer := 12;

    ----------------------------------------------------------------------
    -- geometry constants (Q12 cubie units)
    ----------------------------------------------------------------------
    constant C_FACE  : integer := 12288;         -- 3 cubies
    constant C_RB    : integer := 328;           -- outer edge rounding 0.080
    constant C_RB2   : integer := 13448;         -- C_RB^2 >> 3 (ROM domain)
    constant C_GB    : integer := 331;           -- gap+cubie-round zone 0.081
    constant C_CBW2  : integer := 6328;          -- CBW^2 >> 3 (CBW = 225)
    constant C_HSRS  : integer := 1405;          -- sticker |p| inset (HS-RS)
    constant C_RS    : integer := 348;           -- sticker corner radius
    constant C_RS2   : integer := 15138;         -- C_RS^2 >> 3 (ROM domain)
    constant C_INSET : integer := 98;            -- silhouette inset 0.024
    constant C_SEW   : integer := 123;           -- sticker edge bevel 0.030
    constant C_AMB   : integer := 62;            -- ambient, linear Q8
    constant C_SEDEF : integer := 55;            -- sticker rim light deficit
    -- linearization / profile constants
    constant C_RRB   : integer := 25;            -- ~16384/(2*RB)
    constant C_RRS   : integer := 24;            -- ~16384/(2*RS)
    constant C_WBEV  : integer := 331;           -- wbev = s*331 >> 16 (~s/CBW^2)
    constant C_RAO   : integer := 285;           -- ao ramp ~1/230 Q16

    -- camera: D = 13.0 in Q12
    constant C_CAMD  : integer := 53248;

    ----------------------------------------------------------------------
    -- quarter-sine table, Q12: C_SIN(k) = round(4096*sin(k*pi/128))
    ----------------------------------------------------------------------
    -- Quarter-sine, Q12.  Padded to 256x16 so it maps to a block RAM:
    -- at 65x14 yosys leaves it in logic, which cost ~600 cells on a
    -- chip that is over budget on logic and has 23 spare memories.
    type t_sin256 is array (0 to 255) of signed(15 downto 0);
    constant C_SIN : t_sin256 := (
        to_signed(    0,16), to_signed(  101,16), to_signed(  201,16), to_signed(  301,16), to_signed(  401,16), to_signed(  501,16), to_signed(  601,16), to_signed(  700,16),
        to_signed(  799,16), to_signed(  897,16), to_signed(  995,16), to_signed( 1092,16), to_signed( 1189,16), to_signed( 1285,16), to_signed( 1380,16), to_signed( 1474,16),
        to_signed( 1567,16), to_signed( 1660,16), to_signed( 1751,16), to_signed( 1842,16), to_signed( 1931,16), to_signed( 2019,16), to_signed( 2106,16), to_signed( 2191,16),
        to_signed( 2276,16), to_signed( 2359,16), to_signed( 2440,16), to_signed( 2520,16), to_signed( 2598,16), to_signed( 2675,16), to_signed( 2751,16), to_signed( 2824,16),
        to_signed( 2896,16), to_signed( 2967,16), to_signed( 3035,16), to_signed( 3102,16), to_signed( 3166,16), to_signed( 3229,16), to_signed( 3290,16), to_signed( 3349,16),
        to_signed( 3406,16), to_signed( 3461,16), to_signed( 3513,16), to_signed( 3564,16), to_signed( 3612,16), to_signed( 3659,16), to_signed( 3703,16), to_signed( 3745,16),
        to_signed( 3784,16), to_signed( 3822,16), to_signed( 3857,16), to_signed( 3889,16), to_signed( 3920,16), to_signed( 3948,16), to_signed( 3973,16), to_signed( 3996,16),
        to_signed( 4017,16), to_signed( 4036,16), to_signed( 4052,16), to_signed( 4065,16), to_signed( 4076,16), to_signed( 4085,16), to_signed( 4091,16), to_signed( 4095,16),
        to_signed( 4096,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16),
        to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16), to_signed(    0,16) );

    function f_clamp10(v : signed) return unsigned is

    begin
        if v < 0 then return to_unsigned(0, 10);
        elsif v > 1023 then return to_unsigned(1023, 10);
        else return resize(unsigned(v), 10); end if;
    end function;

    ----------------------------------------------------------------------
    -- sticker colors, 10-bit BT.601 (chroma centred 512), by face id
    -- 0=U white  1=D yellow  2=R red  3=L orange  4=F green  5=B blue
    ----------------------------------------------------------------------
    type t_col6 is array (0 to 5) of unsigned(9 downto 0);
    constant C_STY : t_col6 := (to_unsigned(995,10), to_unsigned(807,10), to_unsigned(332,10),
                                to_unsigned(513,10), to_unsigned(416,10), to_unsigned(276,10));
    constant C_STU : t_col6 := (to_unsigned(512,10), to_unsigned( 91,10), to_unsigned(460,10),
                                to_unsigned(244,10), to_unsigned(496,10), to_unsigned(757,10));
    constant C_STV : t_col6 := (to_unsigned(512,10), to_unsigned(654,10), to_unsigned(811,10),
                                to_unsigned(848,10), to_unsigned(238,10), to_unsigned(330,10));
    constant C_PLY_I : integer := 173;   -- satin black, VIDEO luma of 0.020 linear

    -- half-vector H = normalize(Lkey + V), V = world +z (orthographic), Q8
    constant C_HX : integer := -57;
    constant C_HY : integer :=  81;
    constant C_HZ : integer := 236;

    ----------------------------------------------------------------------
    -- MATERIAL ROMs.  Separate constants per read port: GHDL's synth pass
    -- crashes on two parallel reads of one array-of-arrays constant.
    ----------------------------------------------------------------------
    type t_rom16 is array (0 to 255) of unsigned(15 downto 0);
    type t_rom8  is array (0 to 255) of unsigned(7 downto 0);

    -- bevel tilt profile, indexed by penetration>>1 (164 = full 80 deg):
    -- high byte sin(phi), low byte 1-cos(phi), both Q8.
    constant C_TILTA : t_rom16 := (
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(  256,16), to_unsigned(  256,16), to_unsigned(  256,16), to_unsigned(  512,16),
        to_unsigned(  512,16), to_unsigned(  768,16), to_unsigned(  768,16), to_unsigned( 1024,16), to_unsigned( 1024,16), to_unsigned( 1280,16), to_unsigned( 1280,16), to_unsigned( 1536,16),
        to_unsigned( 1792,16), to_unsigned( 2048,16), to_unsigned( 2048,16), to_unsigned( 2304,16), to_unsigned( 2560,16), to_unsigned( 2816,16), to_unsigned( 3072,16), to_unsigned( 3328,16),
        to_unsigned( 3584,16), to_unsigned( 3840,16), to_unsigned( 4096,16), to_unsigned( 4353,16), to_unsigned( 4609,16), to_unsigned( 4865,16), to_unsigned( 5121,16), to_unsigned( 5377,16),
        to_unsigned( 5633,16), to_unsigned( 5889,16), to_unsigned( 6145,16), to_unsigned( 6657,16), to_unsigned( 6913,16), to_unsigned( 7170,16), to_unsigned( 7682,16), to_unsigned( 7938,16),
        to_unsigned( 8194,16), to_unsigned( 8706,16), to_unsigned( 8962,16), to_unsigned( 9219,16), to_unsigned( 9731,16), to_unsigned( 9987,16), to_unsigned(10499,16), to_unsigned(10756,16),
        to_unsigned(11268,16), to_unsigned(11524,16), to_unsigned(12036,16), to_unsigned(12549,16), to_unsigned(12805,16), to_unsigned(13317,16), to_unsigned(13574,16), to_unsigned(14086,16),
        to_unsigned(14598,16), to_unsigned(15111,16), to_unsigned(15367,16), to_unsigned(15880,16), to_unsigned(16392,16), to_unsigned(16905,16), to_unsigned(17161,16), to_unsigned(17674,16),
        to_unsigned(18186,16), to_unsigned(18699,16), to_unsigned(19211,16), to_unsigned(19724,16), to_unsigned(19980,16), to_unsigned(20493,16), to_unsigned(21006,16), to_unsigned(21518,16),
        to_unsigned(22031,16), to_unsigned(22544,16), to_unsigned(23056,16), to_unsigned(23569,16), to_unsigned(24082,16), to_unsigned(24595,16), to_unsigned(25108,16), to_unsigned(25620,16),
        to_unsigned(26133,16), to_unsigned(26646,16), to_unsigned(27159,16), to_unsigned(27672,16), to_unsigned(28185,16), to_unsigned(28698,16), to_unsigned(29467,16), to_unsigned(29980,16),
        to_unsigned(30493,16), to_unsigned(31006,16), to_unsigned(31520,16), to_unsigned(32033,16), to_unsigned(32546,16), to_unsigned(33059,16), to_unsigned(33829,16), to_unsigned(34342,16),
        to_unsigned(34855,16), to_unsigned(35369,16), to_unsigned(35882,16), to_unsigned(36395,16), to_unsigned(36909,16), to_unsigned(37678,16), to_unsigned(38192,16), to_unsigned(38705,16),
        to_unsigned(39219,16), to_unsigned(39733,16), to_unsigned(40246,16), to_unsigned(40760,16), to_unsigned(41530,16), to_unsigned(42044,16), to_unsigned(42557,16), to_unsigned(43071,16),
        to_unsigned(43585,16), to_unsigned(44099,16), to_unsigned(44613,16), to_unsigned(45127,16), to_unsigned(45641,16), to_unsigned(46155,16), to_unsigned(46925,16), to_unsigned(47439,16),
        to_unsigned(47953,16), to_unsigned(48467,16), to_unsigned(48982,16), to_unsigned(49496,16), to_unsigned(50010,16), to_unsigned(50525,16), to_unsigned(50783,16), to_unsigned(51297,16),
        to_unsigned(51812,16), to_unsigned(52326,16), to_unsigned(52841,16), to_unsigned(53355,16), to_unsigned(53870,16), to_unsigned(54129,16), to_unsigned(54643,16), to_unsigned(55158,16),
        to_unsigned(55673,16), to_unsigned(55931,16), to_unsigned(56446,16), to_unsigned(56961,16), to_unsigned(57220,16), to_unsigned(57735,16), to_unsigned(57994,16), to_unsigned(58509,16),
        to_unsigned(58768,16), to_unsigned(59283,16), to_unsigned(59542,16), to_unsigned(60057,16), to_unsigned(60316,16), to_unsigned(60575,16), to_unsigned(61091,16), to_unsigned(61350,16),
        to_unsigned(61609,16), to_unsigned(61868,16), to_unsigned(62128,16), to_unsigned(62387,16), to_unsigned(62646,16), to_unsigned(62906,16), to_unsigned(63165,16), to_unsigned(63425,16),
        to_unsigned(63684,16), to_unsigned(63944,16), to_unsigned(64204,16), to_unsigned(64207,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16) );

    constant C_TILTB : t_rom16 := (
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(  256,16), to_unsigned(  256,16), to_unsigned(  256,16), to_unsigned(  512,16),
        to_unsigned(  512,16), to_unsigned(  768,16), to_unsigned(  768,16), to_unsigned( 1024,16), to_unsigned( 1024,16), to_unsigned( 1280,16), to_unsigned( 1280,16), to_unsigned( 1536,16),
        to_unsigned( 1792,16), to_unsigned( 2048,16), to_unsigned( 2048,16), to_unsigned( 2304,16), to_unsigned( 2560,16), to_unsigned( 2816,16), to_unsigned( 3072,16), to_unsigned( 3328,16),
        to_unsigned( 3584,16), to_unsigned( 3840,16), to_unsigned( 4096,16), to_unsigned( 4353,16), to_unsigned( 4609,16), to_unsigned( 4865,16), to_unsigned( 5121,16), to_unsigned( 5377,16),
        to_unsigned( 5633,16), to_unsigned( 5889,16), to_unsigned( 6145,16), to_unsigned( 6657,16), to_unsigned( 6913,16), to_unsigned( 7170,16), to_unsigned( 7682,16), to_unsigned( 7938,16),
        to_unsigned( 8194,16), to_unsigned( 8706,16), to_unsigned( 8962,16), to_unsigned( 9219,16), to_unsigned( 9731,16), to_unsigned( 9987,16), to_unsigned(10499,16), to_unsigned(10756,16),
        to_unsigned(11268,16), to_unsigned(11524,16), to_unsigned(12036,16), to_unsigned(12549,16), to_unsigned(12805,16), to_unsigned(13317,16), to_unsigned(13574,16), to_unsigned(14086,16),
        to_unsigned(14598,16), to_unsigned(15111,16), to_unsigned(15367,16), to_unsigned(15880,16), to_unsigned(16392,16), to_unsigned(16905,16), to_unsigned(17161,16), to_unsigned(17674,16),
        to_unsigned(18186,16), to_unsigned(18699,16), to_unsigned(19211,16), to_unsigned(19724,16), to_unsigned(19980,16), to_unsigned(20493,16), to_unsigned(21006,16), to_unsigned(21518,16),
        to_unsigned(22031,16), to_unsigned(22544,16), to_unsigned(23056,16), to_unsigned(23569,16), to_unsigned(24082,16), to_unsigned(24595,16), to_unsigned(25108,16), to_unsigned(25620,16),
        to_unsigned(26133,16), to_unsigned(26646,16), to_unsigned(27159,16), to_unsigned(27672,16), to_unsigned(28185,16), to_unsigned(28698,16), to_unsigned(29467,16), to_unsigned(29980,16),
        to_unsigned(30493,16), to_unsigned(31006,16), to_unsigned(31520,16), to_unsigned(32033,16), to_unsigned(32546,16), to_unsigned(33059,16), to_unsigned(33829,16), to_unsigned(34342,16),
        to_unsigned(34855,16), to_unsigned(35369,16), to_unsigned(35882,16), to_unsigned(36395,16), to_unsigned(36909,16), to_unsigned(37678,16), to_unsigned(38192,16), to_unsigned(38705,16),
        to_unsigned(39219,16), to_unsigned(39733,16), to_unsigned(40246,16), to_unsigned(40760,16), to_unsigned(41530,16), to_unsigned(42044,16), to_unsigned(42557,16), to_unsigned(43071,16),
        to_unsigned(43585,16), to_unsigned(44099,16), to_unsigned(44613,16), to_unsigned(45127,16), to_unsigned(45641,16), to_unsigned(46155,16), to_unsigned(46925,16), to_unsigned(47439,16),
        to_unsigned(47953,16), to_unsigned(48467,16), to_unsigned(48982,16), to_unsigned(49496,16), to_unsigned(50010,16), to_unsigned(50525,16), to_unsigned(50783,16), to_unsigned(51297,16),
        to_unsigned(51812,16), to_unsigned(52326,16), to_unsigned(52841,16), to_unsigned(53355,16), to_unsigned(53870,16), to_unsigned(54129,16), to_unsigned(54643,16), to_unsigned(55158,16),
        to_unsigned(55673,16), to_unsigned(55931,16), to_unsigned(56446,16), to_unsigned(56961,16), to_unsigned(57220,16), to_unsigned(57735,16), to_unsigned(57994,16), to_unsigned(58509,16),
        to_unsigned(58768,16), to_unsigned(59283,16), to_unsigned(59542,16), to_unsigned(60057,16), to_unsigned(60316,16), to_unsigned(60575,16), to_unsigned(61091,16), to_unsigned(61350,16),
        to_unsigned(61609,16), to_unsigned(61868,16), to_unsigned(62128,16), to_unsigned(62387,16), to_unsigned(62646,16), to_unsigned(62906,16), to_unsigned(63165,16), to_unsigned(63425,16),
        to_unsigned(63684,16), to_unsigned(63944,16), to_unsigned(64204,16), to_unsigned(64207,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16),
        to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16), to_unsigned(64467,16) );

    -- specular: video-luma DELTA over a nominal lit base, so the add can
    -- happen after gamma.  High byte sticker, low byte plastic.
    constant C_SPEC : t_rom16 := (
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16),
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    1,16), to_unsigned(    1,16),
        to_unsigned(    1,16), to_unsigned(    1,16), to_unsigned(    1,16), to_unsigned(    1,16), to_unsigned(  257,16), to_unsigned(  257,16), to_unsigned(  257,16), to_unsigned(  257,16),
        to_unsigned(  257,16), to_unsigned(  257,16), to_unsigned(  257,16), to_unsigned(  257,16), to_unsigned(  257,16), to_unsigned(  257,16), to_unsigned(  258,16), to_unsigned(  258,16),
        to_unsigned(  258,16), to_unsigned(  258,16), to_unsigned(  258,16), to_unsigned(  258,16), to_unsigned(  258,16), to_unsigned(  514,16), to_unsigned(  514,16), to_unsigned(  515,16),
        to_unsigned(  515,16), to_unsigned(  515,16), to_unsigned(  515,16), to_unsigned(  515,16), to_unsigned(  516,16), to_unsigned(  772,16), to_unsigned(  772,16), to_unsigned(  772,16),
        to_unsigned(  772,16), to_unsigned(  773,16), to_unsigned(  773,16), to_unsigned( 1029,16), to_unsigned( 1030,16), to_unsigned( 1030,16), to_unsigned( 1030,16), to_unsigned( 1287,16),
        to_unsigned( 1287,16), to_unsigned( 1287,16), to_unsigned( 1544,16), to_unsigned( 1544,16), to_unsigned( 1545,16), to_unsigned( 1801,16), to_unsigned( 1801,16), to_unsigned( 1802,16),
        to_unsigned( 2058,16), to_unsigned( 2059,16), to_unsigned( 2315,16), to_unsigned( 2316,16), to_unsigned( 2317,16), to_unsigned( 2573,16), to_unsigned( 2830,16), to_unsigned( 2830,16),
        to_unsigned( 3087,16), to_unsigned( 3088,16), to_unsigned( 3345,16), to_unsigned( 3601,16), to_unsigned( 3602,16), to_unsigned( 3859,16), to_unsigned( 4116,16), to_unsigned( 4373,16),
        to_unsigned( 4373,16), to_unsigned( 4630,16), to_unsigned( 4887,16), to_unsigned( 5144,16), to_unsigned( 5401,16), to_unsigned( 5658,16), to_unsigned( 5915,16), to_unsigned( 6172,16),
        to_unsigned( 6429,16), to_unsigned( 6687,16), to_unsigned( 7200,16), to_unsigned( 7457,16), to_unsigned( 7970,16), to_unsigned( 8228,16), to_unsigned( 8742,16), to_unsigned( 9256,16),
        to_unsigned(10026,16), to_unsigned(11053,16), to_unsigned(12338,16), to_unsigned(14392,16), to_unsigned(17216,16), to_unsigned(21324,16), to_unsigned(27485,16), to_unsigned(35956,16) );

    -- x*x >> 3 for x = 2*index: the squarers become table reads, trading
    -- logic (which is 154% full) for block RAM (which is 21% full).
    constant C_SQA : t_rom16 := (
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    2,16), to_unsigned(    4,16), to_unsigned(    8,16), to_unsigned(   12,16), to_unsigned(   18,16), to_unsigned(   24,16),
        to_unsigned(   32,16), to_unsigned(   40,16), to_unsigned(   50,16), to_unsigned(   60,16), to_unsigned(   72,16), to_unsigned(   84,16), to_unsigned(   98,16), to_unsigned(  112,16),
        to_unsigned(  128,16), to_unsigned(  144,16), to_unsigned(  162,16), to_unsigned(  180,16), to_unsigned(  200,16), to_unsigned(  220,16), to_unsigned(  242,16), to_unsigned(  264,16),
        to_unsigned(  288,16), to_unsigned(  312,16), to_unsigned(  338,16), to_unsigned(  364,16), to_unsigned(  392,16), to_unsigned(  420,16), to_unsigned(  450,16), to_unsigned(  480,16),
        to_unsigned(  512,16), to_unsigned(  544,16), to_unsigned(  578,16), to_unsigned(  612,16), to_unsigned(  648,16), to_unsigned(  684,16), to_unsigned(  722,16), to_unsigned(  760,16),
        to_unsigned(  800,16), to_unsigned(  840,16), to_unsigned(  882,16), to_unsigned(  924,16), to_unsigned(  968,16), to_unsigned( 1012,16), to_unsigned( 1058,16), to_unsigned( 1104,16),
        to_unsigned( 1152,16), to_unsigned( 1200,16), to_unsigned( 1250,16), to_unsigned( 1300,16), to_unsigned( 1352,16), to_unsigned( 1404,16), to_unsigned( 1458,16), to_unsigned( 1512,16),
        to_unsigned( 1568,16), to_unsigned( 1624,16), to_unsigned( 1682,16), to_unsigned( 1740,16), to_unsigned( 1800,16), to_unsigned( 1860,16), to_unsigned( 1922,16), to_unsigned( 1984,16),
        to_unsigned( 2048,16), to_unsigned( 2112,16), to_unsigned( 2178,16), to_unsigned( 2244,16), to_unsigned( 2312,16), to_unsigned( 2380,16), to_unsigned( 2450,16), to_unsigned( 2520,16),
        to_unsigned( 2592,16), to_unsigned( 2664,16), to_unsigned( 2738,16), to_unsigned( 2812,16), to_unsigned( 2888,16), to_unsigned( 2964,16), to_unsigned( 3042,16), to_unsigned( 3120,16),
        to_unsigned( 3200,16), to_unsigned( 3280,16), to_unsigned( 3362,16), to_unsigned( 3444,16), to_unsigned( 3528,16), to_unsigned( 3612,16), to_unsigned( 3698,16), to_unsigned( 3784,16),
        to_unsigned( 3872,16), to_unsigned( 3960,16), to_unsigned( 4050,16), to_unsigned( 4140,16), to_unsigned( 4232,16), to_unsigned( 4324,16), to_unsigned( 4418,16), to_unsigned( 4512,16),
        to_unsigned( 4608,16), to_unsigned( 4704,16), to_unsigned( 4802,16), to_unsigned( 4900,16), to_unsigned( 5000,16), to_unsigned( 5100,16), to_unsigned( 5202,16), to_unsigned( 5304,16),
        to_unsigned( 5408,16), to_unsigned( 5512,16), to_unsigned( 5618,16), to_unsigned( 5724,16), to_unsigned( 5832,16), to_unsigned( 5940,16), to_unsigned( 6050,16), to_unsigned( 6160,16),
        to_unsigned( 6272,16), to_unsigned( 6384,16), to_unsigned( 6498,16), to_unsigned( 6612,16), to_unsigned( 6728,16), to_unsigned( 6844,16), to_unsigned( 6962,16), to_unsigned( 7080,16),
        to_unsigned( 7200,16), to_unsigned( 7320,16), to_unsigned( 7442,16), to_unsigned( 7564,16), to_unsigned( 7688,16), to_unsigned( 7812,16), to_unsigned( 7938,16), to_unsigned( 8064,16),
        to_unsigned( 8192,16), to_unsigned( 8320,16), to_unsigned( 8450,16), to_unsigned( 8580,16), to_unsigned( 8712,16), to_unsigned( 8844,16), to_unsigned( 8978,16), to_unsigned( 9112,16),
        to_unsigned( 9248,16), to_unsigned( 9384,16), to_unsigned( 9522,16), to_unsigned( 9660,16), to_unsigned( 9800,16), to_unsigned( 9940,16), to_unsigned(10082,16), to_unsigned(10224,16),
        to_unsigned(10368,16), to_unsigned(10512,16), to_unsigned(10658,16), to_unsigned(10804,16), to_unsigned(10952,16), to_unsigned(11100,16), to_unsigned(11250,16), to_unsigned(11400,16),
        to_unsigned(11552,16), to_unsigned(11704,16), to_unsigned(11858,16), to_unsigned(12012,16), to_unsigned(12168,16), to_unsigned(12324,16), to_unsigned(12482,16), to_unsigned(12640,16),
        to_unsigned(12800,16), to_unsigned(12960,16), to_unsigned(13122,16), to_unsigned(13284,16), to_unsigned(13448,16), to_unsigned(13612,16), to_unsigned(13778,16), to_unsigned(13944,16),
        to_unsigned(14112,16), to_unsigned(14280,16), to_unsigned(14450,16), to_unsigned(14620,16), to_unsigned(14792,16), to_unsigned(14964,16), to_unsigned(15138,16), to_unsigned(15312,16),
        to_unsigned(15488,16), to_unsigned(15664,16), to_unsigned(15842,16), to_unsigned(16020,16), to_unsigned(16200,16), to_unsigned(16380,16), to_unsigned(16562,16), to_unsigned(16744,16),
        to_unsigned(16928,16), to_unsigned(17112,16), to_unsigned(17298,16), to_unsigned(17484,16), to_unsigned(17672,16), to_unsigned(17860,16), to_unsigned(18050,16), to_unsigned(18240,16),
        to_unsigned(18432,16), to_unsigned(18624,16), to_unsigned(18818,16), to_unsigned(19012,16), to_unsigned(19208,16), to_unsigned(19404,16), to_unsigned(19602,16), to_unsigned(19800,16),
        to_unsigned(20000,16), to_unsigned(20200,16), to_unsigned(20402,16), to_unsigned(20604,16), to_unsigned(20808,16), to_unsigned(21012,16), to_unsigned(21218,16), to_unsigned(21424,16),
        to_unsigned(21632,16), to_unsigned(21840,16), to_unsigned(22050,16), to_unsigned(22260,16), to_unsigned(22472,16), to_unsigned(22684,16), to_unsigned(22898,16), to_unsigned(23112,16),
        to_unsigned(23328,16), to_unsigned(23544,16), to_unsigned(23762,16), to_unsigned(23980,16), to_unsigned(24200,16), to_unsigned(24420,16), to_unsigned(24642,16), to_unsigned(24864,16),
        to_unsigned(25088,16), to_unsigned(25312,16), to_unsigned(25538,16), to_unsigned(25764,16), to_unsigned(25992,16), to_unsigned(26220,16), to_unsigned(26450,16), to_unsigned(26680,16),
        to_unsigned(26912,16), to_unsigned(27144,16), to_unsigned(27378,16), to_unsigned(27612,16), to_unsigned(27848,16), to_unsigned(28084,16), to_unsigned(28322,16), to_unsigned(28560,16),
        to_unsigned(28800,16), to_unsigned(29040,16), to_unsigned(29282,16), to_unsigned(29524,16), to_unsigned(29768,16), to_unsigned(30012,16), to_unsigned(30258,16), to_unsigned(30504,16),
        to_unsigned(30752,16), to_unsigned(31000,16), to_unsigned(31250,16), to_unsigned(31500,16), to_unsigned(31752,16), to_unsigned(32004,16), to_unsigned(32258,16), to_unsigned(32512,16) );

    constant C_SQB : t_rom16 := (
        to_unsigned(    0,16), to_unsigned(    0,16), to_unsigned(    2,16), to_unsigned(    4,16), to_unsigned(    8,16), to_unsigned(   12,16), to_unsigned(   18,16), to_unsigned(   24,16),
        to_unsigned(   32,16), to_unsigned(   40,16), to_unsigned(   50,16), to_unsigned(   60,16), to_unsigned(   72,16), to_unsigned(   84,16), to_unsigned(   98,16), to_unsigned(  112,16),
        to_unsigned(  128,16), to_unsigned(  144,16), to_unsigned(  162,16), to_unsigned(  180,16), to_unsigned(  200,16), to_unsigned(  220,16), to_unsigned(  242,16), to_unsigned(  264,16),
        to_unsigned(  288,16), to_unsigned(  312,16), to_unsigned(  338,16), to_unsigned(  364,16), to_unsigned(  392,16), to_unsigned(  420,16), to_unsigned(  450,16), to_unsigned(  480,16),
        to_unsigned(  512,16), to_unsigned(  544,16), to_unsigned(  578,16), to_unsigned(  612,16), to_unsigned(  648,16), to_unsigned(  684,16), to_unsigned(  722,16), to_unsigned(  760,16),
        to_unsigned(  800,16), to_unsigned(  840,16), to_unsigned(  882,16), to_unsigned(  924,16), to_unsigned(  968,16), to_unsigned( 1012,16), to_unsigned( 1058,16), to_unsigned( 1104,16),
        to_unsigned( 1152,16), to_unsigned( 1200,16), to_unsigned( 1250,16), to_unsigned( 1300,16), to_unsigned( 1352,16), to_unsigned( 1404,16), to_unsigned( 1458,16), to_unsigned( 1512,16),
        to_unsigned( 1568,16), to_unsigned( 1624,16), to_unsigned( 1682,16), to_unsigned( 1740,16), to_unsigned( 1800,16), to_unsigned( 1860,16), to_unsigned( 1922,16), to_unsigned( 1984,16),
        to_unsigned( 2048,16), to_unsigned( 2112,16), to_unsigned( 2178,16), to_unsigned( 2244,16), to_unsigned( 2312,16), to_unsigned( 2380,16), to_unsigned( 2450,16), to_unsigned( 2520,16),
        to_unsigned( 2592,16), to_unsigned( 2664,16), to_unsigned( 2738,16), to_unsigned( 2812,16), to_unsigned( 2888,16), to_unsigned( 2964,16), to_unsigned( 3042,16), to_unsigned( 3120,16),
        to_unsigned( 3200,16), to_unsigned( 3280,16), to_unsigned( 3362,16), to_unsigned( 3444,16), to_unsigned( 3528,16), to_unsigned( 3612,16), to_unsigned( 3698,16), to_unsigned( 3784,16),
        to_unsigned( 3872,16), to_unsigned( 3960,16), to_unsigned( 4050,16), to_unsigned( 4140,16), to_unsigned( 4232,16), to_unsigned( 4324,16), to_unsigned( 4418,16), to_unsigned( 4512,16),
        to_unsigned( 4608,16), to_unsigned( 4704,16), to_unsigned( 4802,16), to_unsigned( 4900,16), to_unsigned( 5000,16), to_unsigned( 5100,16), to_unsigned( 5202,16), to_unsigned( 5304,16),
        to_unsigned( 5408,16), to_unsigned( 5512,16), to_unsigned( 5618,16), to_unsigned( 5724,16), to_unsigned( 5832,16), to_unsigned( 5940,16), to_unsigned( 6050,16), to_unsigned( 6160,16),
        to_unsigned( 6272,16), to_unsigned( 6384,16), to_unsigned( 6498,16), to_unsigned( 6612,16), to_unsigned( 6728,16), to_unsigned( 6844,16), to_unsigned( 6962,16), to_unsigned( 7080,16),
        to_unsigned( 7200,16), to_unsigned( 7320,16), to_unsigned( 7442,16), to_unsigned( 7564,16), to_unsigned( 7688,16), to_unsigned( 7812,16), to_unsigned( 7938,16), to_unsigned( 8064,16),
        to_unsigned( 8192,16), to_unsigned( 8320,16), to_unsigned( 8450,16), to_unsigned( 8580,16), to_unsigned( 8712,16), to_unsigned( 8844,16), to_unsigned( 8978,16), to_unsigned( 9112,16),
        to_unsigned( 9248,16), to_unsigned( 9384,16), to_unsigned( 9522,16), to_unsigned( 9660,16), to_unsigned( 9800,16), to_unsigned( 9940,16), to_unsigned(10082,16), to_unsigned(10224,16),
        to_unsigned(10368,16), to_unsigned(10512,16), to_unsigned(10658,16), to_unsigned(10804,16), to_unsigned(10952,16), to_unsigned(11100,16), to_unsigned(11250,16), to_unsigned(11400,16),
        to_unsigned(11552,16), to_unsigned(11704,16), to_unsigned(11858,16), to_unsigned(12012,16), to_unsigned(12168,16), to_unsigned(12324,16), to_unsigned(12482,16), to_unsigned(12640,16),
        to_unsigned(12800,16), to_unsigned(12960,16), to_unsigned(13122,16), to_unsigned(13284,16), to_unsigned(13448,16), to_unsigned(13612,16), to_unsigned(13778,16), to_unsigned(13944,16),
        to_unsigned(14112,16), to_unsigned(14280,16), to_unsigned(14450,16), to_unsigned(14620,16), to_unsigned(14792,16), to_unsigned(14964,16), to_unsigned(15138,16), to_unsigned(15312,16),
        to_unsigned(15488,16), to_unsigned(15664,16), to_unsigned(15842,16), to_unsigned(16020,16), to_unsigned(16200,16), to_unsigned(16380,16), to_unsigned(16562,16), to_unsigned(16744,16),
        to_unsigned(16928,16), to_unsigned(17112,16), to_unsigned(17298,16), to_unsigned(17484,16), to_unsigned(17672,16), to_unsigned(17860,16), to_unsigned(18050,16), to_unsigned(18240,16),
        to_unsigned(18432,16), to_unsigned(18624,16), to_unsigned(18818,16), to_unsigned(19012,16), to_unsigned(19208,16), to_unsigned(19404,16), to_unsigned(19602,16), to_unsigned(19800,16),
        to_unsigned(20000,16), to_unsigned(20200,16), to_unsigned(20402,16), to_unsigned(20604,16), to_unsigned(20808,16), to_unsigned(21012,16), to_unsigned(21218,16), to_unsigned(21424,16),
        to_unsigned(21632,16), to_unsigned(21840,16), to_unsigned(22050,16), to_unsigned(22260,16), to_unsigned(22472,16), to_unsigned(22684,16), to_unsigned(22898,16), to_unsigned(23112,16),
        to_unsigned(23328,16), to_unsigned(23544,16), to_unsigned(23762,16), to_unsigned(23980,16), to_unsigned(24200,16), to_unsigned(24420,16), to_unsigned(24642,16), to_unsigned(24864,16),
        to_unsigned(25088,16), to_unsigned(25312,16), to_unsigned(25538,16), to_unsigned(25764,16), to_unsigned(25992,16), to_unsigned(26220,16), to_unsigned(26450,16), to_unsigned(26680,16),
        to_unsigned(26912,16), to_unsigned(27144,16), to_unsigned(27378,16), to_unsigned(27612,16), to_unsigned(27848,16), to_unsigned(28084,16), to_unsigned(28322,16), to_unsigned(28560,16),
        to_unsigned(28800,16), to_unsigned(29040,16), to_unsigned(29282,16), to_unsigned(29524,16), to_unsigned(29768,16), to_unsigned(30012,16), to_unsigned(30258,16), to_unsigned(30504,16),
        to_unsigned(30752,16), to_unsigned(31000,16), to_unsigned(31250,16), to_unsigned(31500,16), to_unsigned(31752,16), to_unsigned(32004,16), to_unsigned(32258,16), to_unsigned(32512,16) );


    -- linear Q8 light factor -> video Q8
    constant C_GAM : t_rom8 := (
        to_unsigned(    0,8), to_unsigned(   21,8), to_unsigned(   28,8), to_unsigned(   34,8), to_unsigned(   39,8), to_unsigned(   43,8), to_unsigned(   46,8), to_unsigned(   50,8), to_unsigned(   53,8), to_unsigned(   56,8), to_unsigned(   59,8), to_unsigned(   61,8),
        to_unsigned(   64,8), to_unsigned(   66,8), to_unsigned(   68,8), to_unsigned(   70,8), to_unsigned(   72,8), to_unsigned(   74,8), to_unsigned(   76,8), to_unsigned(   78,8), to_unsigned(   80,8), to_unsigned(   82,8), to_unsigned(   84,8), to_unsigned(   85,8),
        to_unsigned(   87,8), to_unsigned(   89,8), to_unsigned(   90,8), to_unsigned(   92,8), to_unsigned(   93,8), to_unsigned(   95,8), to_unsigned(   96,8), to_unsigned(   98,8), to_unsigned(   99,8), to_unsigned(  101,8), to_unsigned(  102,8), to_unsigned(  103,8),
        to_unsigned(  105,8), to_unsigned(  106,8), to_unsigned(  107,8), to_unsigned(  109,8), to_unsigned(  110,8), to_unsigned(  111,8), to_unsigned(  112,8), to_unsigned(  114,8), to_unsigned(  115,8), to_unsigned(  116,8), to_unsigned(  117,8), to_unsigned(  118,8),
        to_unsigned(  119,8), to_unsigned(  120,8), to_unsigned(  122,8), to_unsigned(  123,8), to_unsigned(  124,8), to_unsigned(  125,8), to_unsigned(  126,8), to_unsigned(  127,8), to_unsigned(  128,8), to_unsigned(  129,8), to_unsigned(  130,8), to_unsigned(  131,8),
        to_unsigned(  132,8), to_unsigned(  133,8), to_unsigned(  134,8), to_unsigned(  135,8), to_unsigned(  136,8), to_unsigned(  137,8), to_unsigned(  138,8), to_unsigned(  139,8), to_unsigned(  140,8), to_unsigned(  141,8), to_unsigned(  142,8), to_unsigned(  143,8),
        to_unsigned(  144,8), to_unsigned(  144,8), to_unsigned(  145,8), to_unsigned(  146,8), to_unsigned(  147,8), to_unsigned(  148,8), to_unsigned(  149,8), to_unsigned(  150,8), to_unsigned(  151,8), to_unsigned(  151,8), to_unsigned(  152,8), to_unsigned(  153,8),
        to_unsigned(  154,8), to_unsigned(  155,8), to_unsigned(  156,8), to_unsigned(  156,8), to_unsigned(  157,8), to_unsigned(  158,8), to_unsigned(  159,8), to_unsigned(  160,8), to_unsigned(  160,8), to_unsigned(  161,8), to_unsigned(  162,8), to_unsigned(  163,8),
        to_unsigned(  164,8), to_unsigned(  164,8), to_unsigned(  165,8), to_unsigned(  166,8), to_unsigned(  167,8), to_unsigned(  167,8), to_unsigned(  168,8), to_unsigned(  169,8), to_unsigned(  170,8), to_unsigned(  170,8), to_unsigned(  171,8), to_unsigned(  172,8),
        to_unsigned(  173,8), to_unsigned(  173,8), to_unsigned(  174,8), to_unsigned(  175,8), to_unsigned(  175,8), to_unsigned(  176,8), to_unsigned(  177,8), to_unsigned(  178,8), to_unsigned(  178,8), to_unsigned(  179,8), to_unsigned(  180,8), to_unsigned(  180,8),
        to_unsigned(  181,8), to_unsigned(  182,8), to_unsigned(  182,8), to_unsigned(  183,8), to_unsigned(  184,8), to_unsigned(  184,8), to_unsigned(  185,8), to_unsigned(  186,8), to_unsigned(  186,8), to_unsigned(  187,8), to_unsigned(  188,8), to_unsigned(  188,8),
        to_unsigned(  189,8), to_unsigned(  190,8), to_unsigned(  190,8), to_unsigned(  191,8), to_unsigned(  192,8), to_unsigned(  192,8), to_unsigned(  193,8), to_unsigned(  194,8), to_unsigned(  194,8), to_unsigned(  195,8), to_unsigned(  195,8), to_unsigned(  196,8),
        to_unsigned(  197,8), to_unsigned(  197,8), to_unsigned(  198,8), to_unsigned(  199,8), to_unsigned(  199,8), to_unsigned(  200,8), to_unsigned(  200,8), to_unsigned(  201,8), to_unsigned(  202,8), to_unsigned(  202,8), to_unsigned(  203,8), to_unsigned(  203,8),
        to_unsigned(  204,8), to_unsigned(  205,8), to_unsigned(  205,8), to_unsigned(  206,8), to_unsigned(  206,8), to_unsigned(  207,8), to_unsigned(  207,8), to_unsigned(  208,8), to_unsigned(  209,8), to_unsigned(  209,8), to_unsigned(  210,8), to_unsigned(  210,8),
        to_unsigned(  211,8), to_unsigned(  212,8), to_unsigned(  212,8), to_unsigned(  213,8), to_unsigned(  213,8), to_unsigned(  214,8), to_unsigned(  214,8), to_unsigned(  215,8), to_unsigned(  215,8), to_unsigned(  216,8), to_unsigned(  217,8), to_unsigned(  217,8),
        to_unsigned(  218,8), to_unsigned(  218,8), to_unsigned(  219,8), to_unsigned(  219,8), to_unsigned(  220,8), to_unsigned(  220,8), to_unsigned(  221,8), to_unsigned(  221,8), to_unsigned(  222,8), to_unsigned(  223,8), to_unsigned(  223,8), to_unsigned(  224,8),
        to_unsigned(  224,8), to_unsigned(  225,8), to_unsigned(  225,8), to_unsigned(  226,8), to_unsigned(  226,8), to_unsigned(  227,8), to_unsigned(  227,8), to_unsigned(  228,8), to_unsigned(  228,8), to_unsigned(  229,8), to_unsigned(  229,8), to_unsigned(  230,8),
        to_unsigned(  230,8), to_unsigned(  231,8), to_unsigned(  231,8), to_unsigned(  232,8), to_unsigned(  232,8), to_unsigned(  233,8), to_unsigned(  233,8), to_unsigned(  234,8), to_unsigned(  234,8), to_unsigned(  235,8), to_unsigned(  235,8), to_unsigned(  236,8),
        to_unsigned(  236,8), to_unsigned(  237,8), to_unsigned(  237,8), to_unsigned(  238,8), to_unsigned(  238,8), to_unsigned(  239,8), to_unsigned(  239,8), to_unsigned(  240,8), to_unsigned(  240,8), to_unsigned(  241,8), to_unsigned(  241,8), to_unsigned(  242,8),
        to_unsigned(  242,8), to_unsigned(  243,8), to_unsigned(  243,8), to_unsigned(  244,8), to_unsigned(  244,8), to_unsigned(  245,8), to_unsigned(  245,8), to_unsigned(  246,8), to_unsigned(  246,8), to_unsigned(  247,8), to_unsigned(  247,8), to_unsigned(  248,8),
        to_unsigned(  248,8), to_unsigned(  249,8), to_unsigned(  249,8), to_unsigned(  249,8), to_unsigned(  250,8), to_unsigned(  250,8), to_unsigned(  251,8), to_unsigned(  251,8), to_unsigned(  252,8), to_unsigned(  252,8), to_unsigned(  253,8), to_unsigned(  253,8),
        to_unsigned(  254,8), to_unsigned(  254,8), to_unsigned(  255,8), to_unsigned(  255,8) );


    ----------------------------------------------------------------------
    -- face tables (single chunk).  Face axes as +/- axis codes 0..5 =
    -- +x,-x,+y,-y,+z,-z.  n = Uv x Vv (matches the python model FACES).
    ----------------------------------------------------------------------
    type t_i6 is array (0 to 5) of integer range 0 to 5;
    constant C_FN : t_i6 := (2, 3, 0, 1, 4, 5);   -- normal axis code
    constant C_FU : t_i6 := (4, 0, 2, 4, 0, 2);   -- U axis code
    constant C_FV : t_i6 := (0, 4, 4, 2, 2, 0);   -- V axis code
    -- cube corner index = sx + 2*sy + 4*sz (sign bits, 1 = +1.5).  Corner
    -- order per face: P0=(u0,v0) P1=(u3,v0) P2=(u3,v3) P3=(u0,v3).
    type t_fc is array (0 to 5, 0 to 3) of integer range 0 to 7;
    constant C_FCORN : t_fc := (
        (2, 6, 7, 3),    -- U (+y): O=(-x,+y,-z), +U=+z, +V=+x
        (0, 1, 5, 4),    -- D (-y): O=(-x,-y,-z), +U=+x, +V=+z
        (1, 3, 7, 5),    -- R (+x): O=(+x,-y,-z), +U=+y, +V=+z
        (0, 4, 6, 2),    -- L (-x): O=(-x,-y,-z), +U=+z, +V=+y
        (4, 5, 7, 6),    -- F (+z): O=(-x,-y,+z), +U=+x, +V=+y
        (1, 0, 2, 3) );  -- B (-z): O=(+x,-y,-z), +U=+y, +V=+x
    -- neighbour face across each edge [u=0, u=3, v=0, v=3] (for sil flags)
    type t_fa is array (0 to 5, 0 to 3) of integer range 0 to 5;
    constant C_FADJ : t_fa := (
        (5, 4, 3, 2),    -- U: -U=-z=B, +U=F, -V=-x=L, +V=R
        (3, 2, 5, 4),    -- D: -U=L, +U=R, -V=B, +V=F
        (1, 0, 5, 4),    -- R: -U=D, +U=U, -V=B, +V=F
        (5, 4, 1, 0),    -- L: -U=B, +U=F, -V=D, +V=U
        (3, 2, 1, 0),    -- F: -U=L, +U=R, -V=D, +V=U
        (1, 0, 3, 2) );  -- B: -U=D, +U=U, -V=L, +V=R

    ----------------------------------------------------------------------
    -- raster measurement / control latch
    ----------------------------------------------------------------------
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse   : std_logic := '0';
    signal s_sawact     : std_logic := '0';
    signal s_fstart     : std_logic := '0';
    signal s_avid_q     : std_logic := '0';
    signal s_avid_r     : std_logic := '0';   -- avid rising strobe
    signal s_avid_f     : std_logic := '0';   -- avid falling strobe
    signal s_hs_q       : std_logic := '0';
    signal s_hs_r       : std_logic := '0';   -- hsync (active low) leading strobe
    signal s_xcnt       : unsigned(11 downto 0) := (others => '0');
    signal s_W          : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_ycnt       : unsigned(10 downto 0) := (others => '0');  -- active line ctr
    signal s_H          : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_ilace      : std_logic := '0';
    signal s_fpar       : std_logic := '0';
    signal s_field      : std_logic := '0';   -- latched field_n at vsync

    signal s_k1, s_k2, s_k3 : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- FRAME FSM
    ----------------------------------------------------------------------
    signal fr_st   : unsigned(7 downto 0) := (others => '0');  -- state
    signal fr_done : std_logic := '0';

    -- glide-smoothed angles (12b) + targets
    signal an_yaw, an_pit, an_rol : unsigned(11 downto 0) := (others => '0');

    -- ONE multiplier + ONE divider pair, shared by the frame and line FSMs
    -- (operands muxed on fr_done in p_dmath).  2-cycle multiply latency.
    signal fm_a  : signed(17 downto 0) := (others => '0');
    signal fm_b  : signed(13 downto 0) := (others => '0');
    signal lm_a  : signed(17 downto 0) := (others => '0');
    signal lm_b  : signed(13 downto 0) := (others => '0');
    signal mu_p  : signed(31 downto 0) := (others => '0');
    -- serial multiplier state; mu_idle means 'no multiply outstanding',
    -- which is what every call site waits on before reading mu_p
    signal mu_go_f, mu_go_l : std_logic := '0';
    signal mu_bsy : std_logic := '0';
    signal mu_ng  : std_logic := '0';
    signal mu_sa  : signed(31 downto 0) := (others => '0');
    signal mu_sb  : unsigned(13 downto 0) := (others => '0');
    signal mu_ac  : signed(31 downto 0) := (others => '0');
    signal mu_cnt : unsigned(3 downto 0) := (others => '0');
    signal mu_idle : std_logic := '1';

    signal dv_na : unsigned(33 downto 0) := (others => '0');
    signal dv_da : unsigned(20 downto 0) := (others => '1');
    signal dv_q  : unsigned(33 downto 0) := (others => '0');
    signal dv_ra : unsigned(21 downto 0) := (others => '0');
    signal dv_cnt : unsigned(5 downto 0) := (others => '0');
    signal dv_bsy : std_logic := '0';
    signal fd_sgn : std_logic := '0';
    -- frame-side request (30/17, right-aligned into the 38-bit divider)
    signal frd_ns : unsigned(31 downto 0) := (others => '0');
    signal frd_ds : unsigned(20 downto 0) := (others => '1');
    signal frd_start : std_logic := '0';

    -- sin/cos scratch (Q12) for the three angles
    signal t_sinA, t_cosA : signed(13 downto 0) := (others => '0');
    signal cs_yaw_s, cs_yaw_c : signed(13 downto 0) := (others => '0');
    signal cs_pit_s, cs_pit_c : signed(13 downto 0) := (others => '0');
    signal cs_rol_s, cs_rol_c : signed(13 downto 0) := (others => '0');

    -- FRAM: per-frame vertex/basis scratch EBR (one read + one write port;
    -- addressing replaces every register-array mux the old FSM had).
    -- Map: 0..8 basis | 18+c*8+k corners | 42+k rx | 50+k ry | 58+k sx | 66+k sy
    signal fr_ra, fr_wa : unsigned(6 downto 0) := (others => '0');
    signal fr_rd2 : std_logic_vector(15 downto 0) := (others => '0');
    signal fr_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal fr_we  : std_logic := '0';
    -- shared quarter-sine ROM port (single instance)
    signal t_sk   : unsigned(7 downto 0) := (others => '0');
    signal t_srd  : signed(13 downto 0) := (others => '0');
    signal t_sneg : std_logic := '0';
    signal t_ang  : unsigned(11 downto 0) := (others => '0');
    signal t_s1   : signed(13 downto 0) := (others => '0');
    -- frame datapath scalars
    signal opA, opB : signed(15 downto 0) := (others => '0');
    signal coX, coY, coZ : signed(16 downto 0) := (others => '0');  -- C-O, Q11
    signal uvc0, uvc1, uvc2 : signed(15 downto 0) := (others => '0');
    signal vvc0, vvc1, vvc2 : signed(15 downto 0) := (others => '0');
    signal t_cacc : signed(17 downto 0) := (others => '0');
    signal bz0, bz1, bz2 : signed(15 downto 0) := (others => '0');
    signal t_ex, t_ey : unsigned(16 downto 0) := (others => '0');
    signal t_px0, t_py0 : signed(15 downto 0) := (others => '0');
    signal t_dx1, t_dx2, t_dy1, t_dy2 : signed(12 downto 0) := (others => '0');
    signal t_det : signed(25 downto 0) := (others => '0');
    signal t_sxa, t_sya : signed(15 downto 0) := (others => '0');
    signal t_dxe : signed(17 downto 0) := (others => '0');
    -- per-slot affine UV gradients (Q8 units per screen pixel) + face origin
    type t_sl3s17 is array (0 to 2) of signed(16 downto 0);
    type t_sl3s13 is array (0 to 2) of signed(12 downto 0);
    signal sl_gux, sl_guy, sl_gvx, sl_gvy : t_sl3s17 := (others => (others => '0'));
    signal sl_px0, sl_py0 : t_sl3s13 := (others => (others => '0'));

    signal f_zoom  : unsigned(11 downto 0) := to_unsigned(256, 12);  -- focal, px
    signal f_zsm   : unsigned(15 downto 0) := to_unsigned(256*16, 16); -- smoothed Q4
    signal s_zfirst : std_logic := '1';
    signal hmax, wmax : unsigned(14 downto 0) := (others => '0');

    -- per-slot face meta for the pixel path (written by frame FSM)
    type t_sl3u8 is array (0 to 2) of unsigned(7 downto 0);
    type t_sl3u4 is array (0 to 2) of std_logic_vector(3 downto 0);
    type t_sl3u3 is array (0 to 2) of unsigned(2 downto 0);
    signal sl_face : t_sl3u3 := (others => (others => '0'));
    signal sl_sil  : t_sl3u4 := (others => (others => '0'));
    type t_sl3u10 is array (0 to 2) of unsigned(9 downto 0);
    signal sl_cu   : t_sl3u10 := (others => to_unsigned(512, 10));
    signal sl_cv   : t_sl3u10 := (others => to_unsigned(512, 10));
    -- UNLIT sticker albedo (video luma).  The plastic albedo is the same on
    -- every face, so it stays a constant; the light itself now arrives per
    -- pixel through the gamma ROM, because the bevels shade.
    signal sl_ly   : t_sl3u10 := (others => (others => '0'));
    -- flat face light, LINEAR Q8 (ambient + key + fill against the face normal)
    signal sl_lf   : t_sl3u8 := (others => (others => '0'));
    -- half-vector H projected on the face basis, Q8: H.n, H.U, H.V.  With an
    -- orthographic camera both L and V are world constants, so H is too, and
    -- these are just the three whole-cube dot products picked and signed per
    -- face by orthonormality.
    type t_sl3s10 is array (0 to 2) of signed(9 downto 0);
    signal sl_hn, sl_hu, sl_hv : t_sl3s10 := (others => (others => '0'));
    signal s_pxs   : unsigned(7 downto 0) := to_unsigned(64, 8);  -- AA slope Q8
    -- AA slope as mantissa (4..7 encoded 0..3) and shift, so stage H needs
    -- only a shift-add tree
    signal s_pxm   : unsigned(1 downto 0) := (others => '0');
    signal s_pxe   : integer range 0 to 5 := 3;   -- actual shift = 3 + s_pxe
    signal s_nslot : unsigned(1 downto 0) := (others => '0');

    -- frame FSM loop counters / scratch
    signal fr_i    : unsigned(3 downto 0) := (others => '0');
    signal fr_j    : unsigned(2 downto 0) := (others => '0');
    signal fr_w    : unsigned(1 downto 0) := (others => '0');
    signal t_s0    : signed(13 downto 0) := (others => '0');
    signal t_zv    : unsigned(16 downto 0) := (others => '1');
    signal t_ac0   : signed(28 downto 0) := (others => '0');
    signal t_ac1   : signed(28 downto 0) := (others => '0');
    signal f_y12   : unsigned(11 downto 0) := (others => '0');
    signal s_fuse  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_hf    : unsigned(12 downto 0) := to_unsigned(480, 13);  -- frame height
    signal s_cx    : unsigned(11 downto 0) := to_unsigned(360, 12);
    signal s_cy    : unsigned(11 downto 0) := to_unsigned(240, 12);
    signal s_fvis  : std_logic_vector(5 downto 0) := (others => '0');
    signal fr_c    : unsigned(3 downto 0) := (others => '0');
    signal t_ymin, t_ymax : signed(15 downto 0) := (others => '0');
    signal t_lacc  : signed(15 downto 0) := (others => '0');
    signal t_lf    : unsigned(7 downto 0) := (others => '0');
    signal fr_t0, fr_t1, fr_t2, fr_t3 : signed(23 downto 0) := (others => '0');
    signal fr_ax, fr_ay, fr_az : signed(23 downto 0) := (others => '0');
    signal fr_bx, fr_by, fr_bz : signed(23 downto 0) := (others => '0');
    signal fr_nx, fr_ny, fr_nz : signed(23 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- GEOMETRY EBR (256x16): frame FSM writes, line FSM reads.
    -- Layout per slot (base = slot*32):
    --   +0  ymin  +1 ymax
    --   +2..+13  four edges x (y0, x0, slope)  (x0 Q11.2, slope Q8 signed)
    --   +14..+22 Ax,Ay,Cu, Bx,By,Cv, Nx,Ny,Cw  (block-float s16)
    --   +23 flags: [11:8] sil, [2:0] face id
    -- word 127: number of valid slots
    ----------------------------------------------------------------------
    type t_gram is array (0 to 255) of std_logic_vector(15 downto 0);
    signal gram : t_gram := (others => (others => '0'));
    attribute ram_style : string;
    attribute ram_style of gram : signal is "block";
    signal g_wa : unsigned(7 downto 0) := (others => '0');
    signal g_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal g_we : std_logic := '0';
    signal g_ra : unsigned(7 downto 0) := (others => '0');
    signal g_rd : std_logic_vector(15 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- DISPLAY LIST EBR (256x16): line FSM writes, pixel path reads.
    -- 5-word records (x/type, u0, v0, du, dv); bank = target line parity.
    ----------------------------------------------------------------------
    type t_dram is array (0 to 255) of std_logic_vector(15 downto 0);
    signal dram : t_dram := (others => (others => '0'));
    attribute ram_style of dram : signal is "block";
    signal d_wa : unsigned(7 downto 0) := (others => '0');
    signal d_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal d_we : std_logic := '0';
    signal d_ra : unsigned(7 downto 0) := (others => '0');
    signal d_rd : std_logic_vector(15 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- LINE FSM
    ----------------------------------------------------------------------
    signal ln_st   : unsigned(5 downto 0) := (others => '0');
    signal ln_busy : std_logic := '0';
    signal ln_y    : unsigned(11 downto 0) := (others => '0');   -- frame-space target y
    signal ln_ynext: unsigned(10 downto 0) := (others => '0');   -- active-line index
    signal ln_bank : std_logic := '0';

    -- line FSM working registers
    signal nslot_l : unsigned(1 downto 0) := (others => '0');
    type t_cy4 is array (0 to 3) of signed(15 downto 0);
    signal cy4 : t_cy4 := (others => (others => '0'));
    signal ln_yc2 : signed(15 downto 0) := (others => '0');
    signal d_wp  : unsigned(7 downto 0) := (others => '0');
    signal ord0, ord1, ord2 : unsigned(1 downto 0) := (others => '0');
    signal ocnt  : unsigned(1 downto 0) := (others => '0');
    signal ln_k  : unsigned(1 downto 0) := (others => '0');
    signal ln_xa, ln_xb : signed(13 downto 0) := (others => '0');
    signal ln_ua, ln_va, ln_ub, ln_vb : signed(15 downto 0) := (others => '0');
    signal ln_du2, ln_dv2 : signed(15 downto 0) := (others => '0');
    signal ln_pxl, ln_py : signed(13 downto 0) := (others => '0');
    signal ln_lg : integer range 6 to 7 := 6;
    signal ln_first2 : std_logic := '0';
    signal ln_w : unsigned(1 downto 0) := (others => '0');
    signal ln_slot : unsigned(1 downto 0) := (others => '0');
    signal ln_dxp, ln_dyp : signed(13 downto 0) := (others => '0');
    signal ln_acc, ln_acc2 : signed(31 downto 0) := (others => '0');
    signal ln_useed, ln_vseed : signed(15 downto 0) := (others => '0');
    -- per-face working set (loaded from geometry EBR)
    signal w_ymin, w_ymax : unsigned(11 downto 0) := (others => '0');
    signal w_ax, w_ay, w_cu : signed(15 downto 0) := (others => '0');
    signal w_bx, w_by, w_cv : signed(15 downto 0) := (others => '0');
    signal w_nx, w_ny, w_cw : signed(15 downto 0) := (others => '0');
    -- span table (3 spans: xl, xr, slot), then x-sorted emission
    type t_sp3 is array (0 to 2) of signed(13 downto 0);
    signal sp_xl, sp_xr : t_sp3 := (others => (others => '0'));
    signal sp_ok : std_logic_vector(2 downto 0) := (others => '0');
    signal ln_i  : unsigned(1 downto 0) := (others => '0');
    signal ln_e  : unsigned(2 downto 0) := (others => '0');
    signal ln_x0, ln_x1 : signed(13 downto 0) := (others => '0');
    signal ln_uw, ln_vw, ln_ww : signed(27 downto 0) := (others => '0');
    signal ln_dua, ln_dva, ln_dwa : signed(27 downto 0) := (others => '0');
    signal ln_un, ln_vn : signed(15 downto 0) := (others => '0');
    signal ln_up, ln_vp : signed(15 downto 0) := (others => '0');
    signal ln_xn : signed(13 downto 0) := (others => '0');
    signal ln_seg : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal ln_edge_y0 : unsigned(11 downto 0) := (others => '0');
    signal ln_edge_x0 : signed(15 downto 0) := (others => '0');
    signal ln_edge_sl : signed(15 downto 0) := (others => '0');
    signal ln_xmin, ln_xmax : signed(15 downto 0) := (others => '0');
    signal ln_t0 : signed(15 downto 0) := (others => '0');
    signal sp_msk : std_logic_vector(2 downto 0) := (others => '0');
    signal ln_ys : signed(15 downto 0) := (others => '0');
    signal ln_have : std_logic := '0';
    signal ln_first : std_logic := '0';

    ----------------------------------------------------------------------
    -- PIXEL PATH
    ----------------------------------------------------------------------
    -- record prefetcher
    signal pf_st  : unsigned(2 downto 0) := (others => '0');
    signal nx_x   : unsigned(11 downto 0) := to_unsigned(4095, 12);
    signal nx_u, nx_v : signed(15 downto 0) := (others => '0');
    signal nx_xe : unsigned(11 downto 0) := (others => '0');
    signal nx_sl  : unsigned(1 downto 0) := (others => '0');
    signal nx_end : std_logic := '1';
    signal uacc, vacc : signed(25 downto 0) := (others => '0');
    signal cu_du, cu_dv : signed(25 downto 0) := (others => '0');
    signal cu_xe  : unsigned(11 downto 0) := (others => '0');
    signal cu_sl  : unsigned(1 downto 0) := (others => '0');
    signal cu_on  : std_logic := '0';
    signal pf_rdp : unsigned(7 downto 0) := (others => '0');

    -- pipeline stage registers (see stage comments in p_pix)
    signal b_u, b_v : signed(15 downto 0) := (others => '0');
    signal b_sl : unsigned(1 downto 0) := (others => '0');
    signal b_on : std_logic := '0';
    signal c_pu, c_pv : signed(13 downto 0) := (others => '0');
    signal c_cu, c_cv : unsigned(1 downto 0) := (others => '0');
    signal c_sl : unsigned(1 downto 0) := (others => '0');
    signal c_on : std_logic := '0';
    signal e_mu, e_mv : unsigned(8 downto 0) := (others => '0');
    signal e_gu, e_gv : unsigned(8 downto 0) := (others => '0');
    signal e_qsu, e_qsv : unsigned(8 downto 0) := (others => '0');
    signal e_stz : std_logic := '0';
    -- bevel tilt: ROM index per axis (penetration>>1, face bevel and cubie
    -- gap rounding rescaled onto one profile) plus the outward direction bit,
    -- which is simply the sign of the in-cubie coordinate in BOTH zones.
    signal e_tu, e_tv : unsigned(7 downto 0) := (others => '0');
    signal e_su, e_sv : std_logic := '0';
    signal e_tmax : unsigned(7 downto 0) := (others => '0');
    signal f_tmax, g_tmax, h_tmax : unsigned(7 downto 0) := (others => '0');
    signal f_su, f_sv : std_logic := '0';
    signal tr_du, tr_dv : unsigned(15 downto 0) := (others => '0');
    signal g_p0, g_p1, g_p2 : signed(18 downto 0) := (others => '0');
    signal h_ndh : unsigned(7 downto 0) := (others => '0');
    signal h_dif : signed(10 downto 0) := (others => '0');
    signal gm_a, gm_ap, gm_d : unsigned(7 downto 0) := (others => '0');
    signal sp_a : unsigned(7 downto 0) := (others => '0');
    signal sp_d : unsigned(15 downto 0) := (others => '0');
    signal k_gam : unsigned(7 downto 0) := (others => '0');
    signal k_spec : unsigned(11 downto 0) := (others => '0');
    signal t_dh0, t_dh1, t_dh2 : signed(11 downto 0) := (others => '0');
    signal e_dst : signed(10 downto 0) := (others => '0');
    signal e_dstr : signed(12 downto 0) := (others => '0');
    signal e_cact : std_logic := '0';
    signal e_silz : std_logic := '0';
    signal e_stc : std_logic := '0';
    signal e_sl : unsigned(1 downto 0) := (others => '0');
    signal e_on : std_logic := '0';
    signal sq_du, sq_dv : unsigned(15 downto 0) := (others => '0');
    signal f_stz : std_logic := '0';
    signal f_dst : signed(10 downto 0) := (others => '0');
    signal f_dstr : signed(12 downto 0) := (others => '0');
    signal f_cact : std_logic := '0';
    signal f_2m : std_logic := '0';
    signal f_silz : std_logic := '0';
    signal f_stc : std_logic := '0';
    signal f_sl : unsigned(1 downto 0) := (others => '0');
    signal f_on : std_logic := '0';
    signal g_sx : unsigned(18 downto 0) := (others => '0');
    signal g_stz : std_logic := '0';
    signal g_dst : signed(10 downto 0) := (others => '0');
    signal g_dstr : signed(12 downto 0) := (others => '0');
    signal g_cact : std_logic := '0';
    signal g_2m : std_logic := '0';
    signal g_silz : std_logic := '0';
    signal g_stc : std_logic := '0';
    signal g_sl2 : unsigned(1 downto 0) := (others => '0');
    signal g_on : std_logic := '0';
    signal h_dlin : signed(11 downto 0) := (others => '0');
    signal h_stz : std_logic := '0';
    signal h_ao : unsigned(7 downto 0) := (others => '0');
    signal h_dst : signed(10 downto 0) := (others => '0');
    signal h_dstr : signed(12 downto 0) := (others => '0');
    signal h_cact : std_logic := '0';
    signal h_2m : std_logic := '0';
    signal h_silz : std_logic := '0';
    signal h_stc : std_logic := '0';
    signal h_sl : unsigned(1 downto 0) := (others => '0');
    signal h_on : std_logic := '0';
    signal i_dsil : signed(13 downto 0) := (others => '0');
    signal i_ds : signed(10 downto 0) := (others => '0');
    signal i_sl : unsigned(1 downto 0) := (others => '0');
    signal i_on : std_logic := '0';
    signal j_asil, j_ast : unsigned(4 downto 0) := (others => '0');
    signal j_sl : unsigned(1 downto 0) := (others => '0');
    signal j_on : std_logic := '0';
    signal k_y : unsigned(9 downto 0) := (others => '0');
    signal k_ston : std_logic := '0';
    signal k_sl2 : unsigned(1 downto 0) := (others => '0');
    signal k_asil : unsigned(4 downto 0) := (others => '0');
    signal m_y : unsigned(9 downto 0) := (others => '0');
    signal m_ston : std_logic := '0';
    signal m_sl2 : unsigned(1 downto 0) := (others => '0');
    signal m_asil : unsigned(4 downto 0) := (others => '0');
    signal n_y, n_u, n_v : unsigned(9 downto 0) := (others => '0');
    signal pf_consume : std_logic := '0';
    signal s_qx0 : unsigned(21 downto 0) := (others => '0');
    signal bg_vy : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- background accumulators (second differences: no multiplies)
    ----------------------------------------------------------------------
    signal bg_ly   : unsigned(17 downto 0) := to_unsigned(644*256, 18); -- line luma Q8
    signal bg_uq   : unsigned(17 downto 0) := to_unsigned(506*256, 18);
    signal bg_vq   : unsigned(17 downto 0) := to_unsigned(516*256, 18);
    signal bg_u    : unsigned(9 downto 0) := to_unsigned(506, 10);
    signal bg_v    : unsigned(9 downto 0) := to_unsigned(516, 10);
    -- delayed to meet pixel pipe stage M
    type t_bg_sr is array (0 to 13) of unsigned(9 downto 0);
    signal bgy_sr : t_bg_sr := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- output + sync delay
    ----------------------------------------------------------------------
    signal o_y, o_u, o_v : unsigned(9 downto 0) := (others => '0');
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');


    ------------------------------------------------------------------------
    -- microcoded geometry engine: register file, program counter, decode
    ------------------------------------------------------------------------
    type t_rf is array (0 to 127) of std_logic_vector(31 downto 0);
    signal rf_a, rf_b : t_rf := (others => (others => '0'));
    attribute ram_style of rf_a : signal is "block";
    attribute ram_style of rf_b : signal is "block";
    signal u_pc  : unsigned(9 downto 0) := (others => '0');
    signal u_st  : unsigned(1 downto 0) := (others => '0');
    signal u_gwt : unsigned(1 downto 0) := (others => '0');
    signal u_ir, u_rom : std_logic_vector(39 downto 0) := (others => '0');
    signal u_ra, u_rb, u_wa, u_xa : unsigned(6 downto 0) := (others => '0');
    signal u_rda, u_rdb, u_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal u_xd, u_slv, u_ctlv : signed(31 downto 0) := (others => '0');
    signal u_we, u_slw : std_logic := '0';
    signal u_done : std_logic := '0';
    signal u_slp : unsigned(4 downto 0) := (others => '0');
    signal u_sls : unsigned(1 downto 0) := (others => '0');
    signal u_gama : unsigned(7 downto 0) := (others => '0');

    type t_ucode is array (0 to 1023) of std_logic_vector(39 downto 0);
    constant C_UCODE : t_ucode := (
        x"7BA0000000", x"7BB0000001", x"CAE0000000", x"2AE5C00002",
        x"1AF5CA0000", x"2305E00004", x"980600000D", x"A005EE800C",
        x"A0074BC00A", x"900000000D", x"0B07600000", x"900000000D",
        x"3307600000", x"12850C0000", x"DA85000FFF", x"CB1000000A",
        x"9806200012", x"9000000013", x"0A85C00000", x"CAE0000001",
        x"2AE5C00002", x"1AF5CA4000", x"2305E00004", x"980600001E",
        x"A005EE801D", x"A0074BC01B", x"900000001E", x"0B07600000",
        x"900000001E", x"3307600000", x"12952C0000", x"DA95200FFF",
        x"CB1000000A", x"9806200023", x"9000000024", x"0A95C00000",
        x"CAE0000002", x"2AE5C00002", x"1AF5CA8000", x"2305E00004",
        x"980600002F", x"A005EE802E", x"A0074BC02C", x"900000002F",
        x"0B07600000", x"900000002F", x"3307600000", x"12A54C0000",
        x"DAA5400FFF", x"CB1000000A", x"9806200034", x"9000000035",
        x"0AA5C00000", x"7AB0000000", x"22E5600001", x"7B40000000",
        x"9805C0003B", x"0B45000000", x"9000000040", x"1AF5CEC000",
        x"9805E0003F", x"0B45200000", x"9000000040", x"0B45400000",
        x"E305600000", x"9806000043", x"9000000046", x"7B10000400",
        x"13468C4000", x"DB46800FFF", x"22E6800004", x"DAE5C0003F",
        x"E2F680000A", x"E36680000B", x"9805E0004E", x"0B55C00000",
        x"1395CEC000", x"9000000051", x"7B00000040", x"1B560B8000",
        x"1B96AEC000", x"7376A00000", x"7387200000", x"9806C00055",
        x"9000000057", x"3376E00000", x"3387000000", x"1AE70DC000",
        x"DAF680000F", x"5005CBC000", x"5B00000004", x"1316EC0000",
        x"7B20000000", x"1B356C8000", x"9806600060", x"0A26200000",
        x"7B20000001", x"1B356C8000", x"9806600064", x"0A36200000",
        x"7B20000002", x"1B356C8000", x"9806600068", x"0A46200000",
        x"7B20000003", x"1B356C8000", x"980660006C", x"0A56200000",
        x"7B20000004", x"1B356C8000", x"9806600070", x"0A66200000",
        x"7B20000005", x"1B356C8000", x"9806600074", x"0A76200000",
        x"12B56EC000", x"7B20000006", x"A0056C8036", x"7800001000",
        x"7810000000", x"7820000000", x"7830000000", x"7840001000",
        x"7850000000", x"7860000000", x"7870000000", x"7880001000",
        x"500009C000", x"5AE000000C", x"5000298000", x"5AF000000C",
        x"5000098000", x"5B0000000C", x"500029C000", x"5B1000000C",
        x"1805CBC000", x"10160C4000", x"500069C000", x"5AE000000C",
        x"5000898000", x"5AF000000C", x"5000698000", x"5B0000000C",
        x"500089C000", x"5B1000000C", x"1835CBC000", x"10460C4000",
        x"5000C9C000", x"5AE000000C", x"5000E98000", x"5AF000000C",
        x"5000C98000", x"5B0000000C", x"5000E9C000", x"5B1000000C",
        x"1865CBC000", x"10760C4000", x"5000294000", x"5AE000000C",
        x"5000490000", x"5AF000000C", x"5000290000", x"5B0000000C",
        x"5000494000", x"5B1000000C", x"1815CBC000", x"10260C4000",
        x"5000894000", x"5AE000000C", x"5000A90000", x"5AF000000C",
        x"5000890000", x"5B0000000C", x"5000A94000", x"5B1000000C",
        x"1845CBC000", x"10560C4000", x"5000E94000", x"5AE000000C",
        x"5001090000", x"5AF000000C", x"5000E90000", x"5B0000000C",
        x"5001094000", x"5B1000000C", x"1875CBC000", x"10860C4000",
        x"500048C000", x"5AE000000C", x"5000088000", x"5AF000000C",
        x"5000488000", x"5B0000000C", x"500008C000", x"5B1000000C",
        x"1825CBC000", x"10060C4000", x"5000A8C000", x"5AE000000C",
        x"5000688000", x"5AF000000C", x"5000A88000", x"5B0000000C",
        x"500068C000", x"5B1000000C", x"1855CBC000", x"10360C4000",
        x"500108C000", x"5AE000000C", x"5000C88000", x"5AF000000C",
        x"5001088000", x"5B0000000C", x"5000C8C000", x"5B1000000C",
        x"1885CBC000", x"10660C4000", x"3AE0000000", x"3AF0600000",
        x"12E5CBC000", x"3AF0C00000", x"12E5CBC000", x"3B00200000",
        x"3AF0800000", x"13060BC000", x"3AF0E00000", x"13060BC000",
        x"22F5C00001", x"13C5CBC000", x"22F6000001", x"13D60BC000",
        x"CAE0000005", x"2AF5C00005", x"2B05C00002", x"1AF5EC0000",
        x"1AF5EB8000", x"6005EF400A", x"6B10000000", x"7B20000FFF",
        x"B3E62C8000", x"CAE0000003", x"2AF5C00005", x"2B05C00001",
        x"1AF5EC0000", x"6005EF000A", x"6B10000000", x"B3162C8000",
        x"43F62F8000", x"2AE7E00004", x"1AF5D00000", x"3B05E00000",
        x"2318000003", x"A0062C0101", x"22F5E00004", x"14080BC000",
        x"9000000102", x"0C05C00000", x"2418000004", x"5000104000",
        x"5AE0000000", x"22F5C00001", x"12E5CBC000", x"20C5C0000E",
        x"5000304000", x"5AE0000000", x"22F5C00001", x"12E5CBC000",
        x"20F5C0000E", x"5000704000", x"5AE0000000", x"22F5C00001",
        x"12E5CBC000", x"20D5C0000E", x"5000904000", x"5AE0000000",
        x"22F5C00001", x"12E5CBC000", x"2105C0000E", x"5000D04000",
        x"5AE0000000", x"22F5C00001", x"12E5CBC000", x"20E5C0000E",
        x"5000F04000", x"5AE0000000", x"22F5C00001", x"12E5CBC000",
        x"2115C0000E", x"CC20000006", x"2C28400002", x"CC30000007",
        x"2C38600002", x"32E1800000", x"1AE5C34000", x"1AE5C38000",
        x"11284B8000", x"32E1E00000", x"1AE5C40000", x"1AE5C44000",
        x"19A86B8000", x"0AE1800000", x"1AE5C34000", x"1AE5C38000",
        x"11384B8000", x"0AE1E00000", x"1AE5C40000", x"1AE5C44000",
        x"19B86B8000", x"32E1800000", x"12E5C34000", x"1AE5C38000",
        x"11484B8000", x"32E1E00000", x"12E5C40000", x"1AE5C44000",
        x"19C86B8000", x"0AE1800000", x"12E5C34000", x"1AE5C38000",
        x"11584B8000", x"0AE1E00000", x"12E5C40000", x"1AE5C44000",
        x"19D86B8000", x"32E1800000", x"1AE5C34000", x"12E5C38000",
        x"11684B8000", x"32E1E00000", x"1AE5C40000", x"12E5C44000",
        x"19E86B8000", x"0AE1800000", x"1AE5C34000", x"12E5C38000",
        x"11784B8000", x"0AE1E00000", x"1AE5C40000", x"12E5C44000",
        x"19F86B8000", x"32E1800000", x"12E5C34000", x"12E5C38000",
        x"11884B8000", x"32E1E00000", x"12E5C40000", x"12E5C44000",
        x"1A086B8000", x"0AE1800000", x"12E5C34000", x"12E5C38000",
        x"11984B8000", x"0AE1E00000", x"12E5C40000", x"12E5C44000",
        x"1A186B8000", x"7B20000040", x"3336400000", x"7C40000000",
        x"A80641416A", x"7C40000001", x"7C50000000", x"A800ACC16D",
        x"7C50000001", x"7C60000000", x"A806408170", x"7C60000001",
        x"7C70000000", x"A8004CC173", x"7C70000001", x"7C80000000",
        x"A806420176", x"7C80000001", x"7C90000000", x"A8010CC179",
        x"7C90000001", x"7CA00007F2", x"7D000004E5", x"7D60000022",
        x"7CB0000948", x"7D10000953", x"7D70000103", x"7CC0000BD9",
        x"7D20000941", x"7D80000110", x"7CD00005A0", x"7D30000065",
        x"7D900000A1", x"7CE0000DEC", x"7D40000053", x"7DA0000084",
        x"7CF0000681", x"7D500004C1", x"7DB0000015", x"7FB0000FA0",
        x"2FBF600003", x"7AE0000001", x"2AE5C0000F", x"12E5CB8000",
        x"1FC5CEC000", x"7AE0000200", x"2AE5C0000A", x"7AF0000200",
        x"12E5CBC000", x"2AE5C0000A", x"7AF00003E3", x"1755CBC000",
        x"7AE000028E", x"2AE5C0000A", x"7AF000005B", x"12E5CBC000",
        x"2AE5C0000A", x"7AF0000327", x"1765CBC000", x"7AE000032B",
        x"2AE5C0000A", x"7AF00001CC", x"12E5CBC000", x"2AE5C0000A",
        x"7AF000014C", x"1775CBC000", x"7AE0000350", x"2AE5C0000A",
        x"7AF00000F4", x"12E5CBC000", x"2AE5C0000A", x"7AF0000201",
        x"1785CBC000", x"7AE00000EE", x"2AE5C0000A", x"7AF00001F0",
        x"12E5CBC000", x"2AE5C0000A", x"7AF00001A0", x"1795CBC000",
        x"7AE000014A", x"2AE5C0000A", x"7AF00002F5", x"12E5CBC000",
        x"2AE5C0000A", x"7AF0000114", x"17A5CBC000", x"7DC0000000",
        x"7AC0000000", x"F2E5800044", x"9805C001C0", x"900000030E",
        x"7AF0000003", x"A80B8BC311", x"F65580004A", x"F665800050",
        x"F675800056", x"DAECA00007", x"F5E5C00012", x"F5F5C0001A",
        x"22FCA00003", x"DAF5E00007", x"F305E00012", x"1B06178000",
        x"2606000002", x"F305E0001A", x"1B0617C000", x"2626000002",
        x"22FCA00009", x"DAF5E00007", x"F305E00012", x"1B06178000",
        x"2616000002", x"F305E0001A", x"1B0617C000", x"2636000002",
        x"500C18C000", x"5E40000000", x"500C584000", x"5AE0000000",
        x"1E4C8B8000", x"0AEC600000", x"12F5CB8000", x"12F5EB8000",
        x"6005F90014", x"6B20000000", x"43265F0000", x"333F800000",
        x"4B264CC000", x"8806570000", x"32EC200000", x"12F5CB8000",
        x"12F5EB8000", x"6005F90014", x"6B20000000", x"43265F0000",
        x"333F800000", x"4B264CC000", x"8806570001", x"32EC400000",
        x"12F5CB8000", x"12F5EB8000", x"6005F90014", x"6B20000000",
        x"43265F0000", x"333F800000", x"4B264CC000", x"8806570002",
        x"0AEC000000", x"12F5CB8000", x"12F5EB8000", x"6005F90014",
        x"6B20000000", x"43265F0000", x"333F800000", x"4B264CC000",
        x"8806570003", x"22EBC00002", x"8805D70004", x"22EBE00002",
        x"8805D70005", x"DAECA00007", x"F685C0001A", x"0E9D000000",
        x"22ECA00003", x"DAE5C00007", x"F2F5C0001A", x"468D0BC000",
        x"4E9D2BC000", x"22ECA00006", x"DAE5C00007", x"F2F5C0001A",
        x"468D0BC000", x"4E9D2BC000", x"22ECA00009", x"DAE5C00007",
        x"F2F5C0001A", x"468D0BC000", x"4E9D2BC000", x"2EDB800005",
        x"22ED000002", x"4AE5CE8000", x"800DAB8000", x"22ED200002",
        x"12E5CEC000", x"4AE5CE8000", x"800DAB8001", x"DAECA00007",
        x"F305C00012", x"F315C0001A", x"22FCA00003", x"DAF5E00007",
        x"F325E00012", x"F335E0001A", x"1F164C0000", x"1F266C4000",
        x"800DAC4002", x"800DAC0003", x"3F3E400000", x"7F40000002",
        x"A80E7D0231", x"A00E4E8230", x"0F2E800000", x"9000000231",
        x"372E800000", x"600E3C8006", x"6F30000000", x"473E7EC000",
        x"374F600000", x"4F3E7D0000", x"800DBCC004", x"22ECA00003",
        x"DAE5C00007", x"F305C00012", x"F315C0001A", x"22FCA00006",
        x"DAF5E00007", x"F325E00012", x"F335E0001A", x"1F164C0000",
        x"1F266C4000", x"800DAC4005", x"800DAC0006", x"3F3E400000",
        x"7F40000002", x"A80E7D024A", x"A00E4E8249", x"0F2E800000",
        x"900000024A", x"372E800000", x"600E3C8006", x"6F30000000",
        x"473E7EC000", x"374F600000", x"4F3E7D0000", x"800DBCC007",
        x"22ECA00006", x"DAE5C00007", x"F305C00012", x"F315C0001A",
        x"22FCA00009", x"DAF5E00007", x"F325E00012", x"F335E0001A",
        x"1F164C0000", x"1F266C4000", x"800DAC4008", x"800DAC0009",
        x"3F3E400000", x"7F40000002", x"A80E7D0263", x"A00E4E8262",
        x"0F2E800000", x"9000000263", x"372E800000", x"600E3C8006",
        x"6F30000000", x"473E7EC000", x"374F600000", x"4F3E7D0000",
        x"800DBCC00A", x"22ECA00009", x"DAE5C00007", x"F305C00012",
        x"F315C0001A", x"DAFCA00007", x"F325E00012", x"F335E0001A",
        x"1F164C0000", x"1F266C4000", x"800DAC400B", x"800DAC000C",
        x"3F3E400000", x"7F40000002", x"A80E7D027B", x"A00E4E827A",
        x"0F2E800000", x"900000027B", x"372E800000", x"600E3C8006",
        x"6F30000000", x"473E7EC000", x"374F600000", x"4F3E7D0000",
        x"800DBCC00D", x"DAECE00007", x"22F5C00001", x"1305EBC000",
        x"13060BC000", x"E315C00000", x"7B20000000", x"13260C8000",
        x"F336400000", x"980620028B", x"900000028C", x"3336600000",
        x"0EE6600000", x"7B20000001", x"13260C8000", x"F336400000",
        x"9806200292", x"9000000293", x"3336600000", x"0EF6600000",
        x"7B20000002", x"13260C8000", x"F336400000", x"9806200299",
        x"900000029A", x"3336600000", x"0F06600000", x"7EA0000000",
        x"7EB0000000", x"7B30003F95", x"500DCCC000", x"5AE000000C",
        x"16AD4B8000", x"7B3000009A", x"500DECC000", x"5AE000000C",
        x"16AD4B8000", x"7B300000B8", x"500E0CC000", x"5AE000000C",
        x"16AD4B8000", x"7B3000008D", x"500DCCC000", x"5AE000000C",
        x"16BD6B8000", x"7B30003F73", x"500DECC000", x"5AE000000C",
        x"16BD6B8000", x"7B300000A1", x"500E0CC000", x"5AE000000C",
        x"16BD6B8000", x"7AE000003E", x"A8075A82BA", x"22FD400002",
        x"1B0D4BC000", x"12E5CC0000", x"A8075AC2BD", x"22FD600002",
        x"12E5CBC000", x"7AF00000FF", x"46C5CBC000", x"8805970006",
        x"7AE0000000", x"DAFCC00007", x"F305E00044", x"98060002C6",
        x"7B10000001", x"12E5CC4000", x"22FCC00003", x"DAF5E00007",
        x"F305E00044", x"98060002CC", x"7B10000002", x"12E5CC4000",
        x"22FCC00006", x"DAF5E00007", x"F305E00044", x"98060002D2",
        x"7B10000004", x"12E5CC4000", x"22FCC00009", x"DAF5E00007",
        x"F305E00044", x"98060002D8", x"7B10000008", x"12E5CC4000",
        x"8805D70007", x"DAECE00007", x"22F5C00001", x"F305E00009",
        x"E315C00000", x"98062002DF", x"90000002E0", x"3306000000",
        x"880617000C", x"22ECE00003", x"DAE5C00007", x"22F5C00001",
        x"F305E00009", x"E315C00000", x"98062002E8", x"90000002E9",
        x"3306000000", x"880617000D", x"22ECE00006", x"DAE5C00007",
        x"22F5C00001", x"F305E00009", x"E315C00000", x"98062002F1",
        x"90000002F2", x"3306000000", x"880617000E", x"F745800075",
        x"DAEE8003FF", x"8805D7000A", x"880D97000B", x"BB2D800000",
        x"22EE80000A", x"DAE5C003FF", x"7AF0000200", x"1AE5CBC000",
        x"5005CC8000", x"5B00000008", x"13060BC000", x"7B100003FF",
        x"B3060C4000", x"8806170008", x"22EE80000A", x"22E5C0000A",
        x"DAE5C003FF", x"7AF0000200", x"1AE5CBC000", x"5005CC8000",
        x"5B00000008", x"13060BC000", x"7B100003FF", x"B3060C4000",
        x"8806170009", x"15CB8EC000", x"12C58EC000", x"7AE0000006",
        x"A0058B81BD", x"CAE0000006", x"5005CB8000", x"5AF0000000",
        x"DB05E03FFF", x"2315E0000E", x"DB162000FF", x"2B1620000E",
        x"13060C4000", x"88060E8010", x"800757007F", x"C000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000",
        x"0000000000", x"0000000000", x"0000000000", x"0000000000" );
begin

    ------------------------------------------------------------------------
    -- raster measurement (width, active-line count, interlace detect)
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            if data_in.vsync_n = '0' and s_prev_vsync = '1' then
                s_vs_pulse <= '1';
            end if;

            s_avid_q <= data_in.avid;
            s_avid_r <= data_in.avid and not s_avid_q;
            s_avid_f <= s_avid_q and not data_in.avid;
            s_hs_q   <= data_in.hsync_n;
            s_hs_r   <= s_hs_q and not data_in.hsync_n;   -- falling edge of _n

            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then
                if s_xcnt > 16 then s_W <= s_xcnt; end if;
                s_xcnt <= (others => '0');
            end if;

            -- count active lines on the ACTIVE edge (vsync height-capture trap:
            -- never latch height at vsync from a serrated counter)
            if s_avid_r = '1' then
                s_ycnt <= s_ycnt + 1;
                s_sawact <= '1';
            end if;
            -- Only the FIRST vsync edge after active video is a real frame
            -- boundary; an analog serrated vsync gives several per field and
            -- would latch a partial line count (and restart the frame FSM).
            s_fstart <= '0';
            if s_vs_pulse = '1' and s_sawact = '1' then
                if s_ycnt > 16 then s_H <= s_ycnt; end if;
                s_ycnt <= (others => '0');
                s_sawact <= '0';
                s_fstart <= '1';
                s_fpar <= data_in.field_n;
                s_field <= data_in.field_n;
                if data_in.field_n /= s_fpar then s_ilace <= '1';
                else                              s_ilace <= '0'; end if;
            end if;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- shared frame multiplier (2-cycle: operands set at N, product at N+2)
    -- and serial restoring divider (dv_q = fd_n / fd_d after fd_cnt cycles)
    ------------------------------------------------------------------------
    ------------------------------------------------------------------------
    -- SHARED FRAME/LINE DATAPATH.  The line FSM only ever runs with
    -- fr_done = '1' and the frame FSM only with fr_done = '0', so both can
    -- drive ONE multiplier array and ONE divider pair through an operand
    -- mux -- half the arithmetic area for free.  Latency is unchanged
    -- (operands registered by the caller at N, product readable at N+2).
    ------------------------------------------------------------------------
    p_dmath : process(clk)
        variable v_ma : signed(17 downto 0);
        variable v_mb : signed(13 downto 0);
        variable v_r  : unsigned(21 downto 0);
        variable v_r2 : unsigned(21 downto 0);
    begin
        if rising_edge(clk) then
            -- SERIAL shift-add multiply.  A parallel 18x14 array costs about
            -- 765 logic cells on a chip with no hardware multipliers; this
            -- costs well under a tenth of that and takes 16 cycles.  Both
            -- callers run during blanking -- the frame FSM in vblank (~99,000
            -- spare clocks) and the line FSM a whole line ahead of the pixel
            -- it feeds -- so the latency is invisible.
            if mu_go_f = '1' or mu_go_l = '1' then
                if fr_done = '0' then v_ma := fm_a; v_mb := fm_b;
                else                  v_ma := lm_a; v_mb := lm_b; end if;
                mu_sa <= resize(v_ma, 32);
                if v_mb < 0 then
                    mu_sb <= unsigned(-v_mb);
                    mu_ng <= '1';
                else
                    mu_sb <= unsigned(v_mb);
                    mu_ng <= '0';
                end if;
                mu_ac  <= (others => '0');
                mu_cnt <= to_unsigned(14, 4);
                mu_bsy <= '1';
            elsif mu_bsy = '1' then
                if mu_cnt = 0 then
                    if mu_ng = '1' then mu_p <= -mu_ac; else mu_p <= mu_ac; end if;
                    mu_bsy <= '0';
                else
                    if mu_sb(0) = '1' then mu_ac <= mu_ac + mu_sa; end if;
                    mu_sa  <= shift_left(mu_sa, 1);
                    mu_sb  <= shift_right(mu_sb, 1);
                    mu_cnt <= mu_cnt - 1;
                end if;
            end if;

            if dv_bsy = '1' then
                v_r := dv_ra(20 downto 0) & dv_na(33);
                if v_r >= resize(dv_da, 22) then
                    dv_ra <= v_r - resize(dv_da, 22);
                    dv_q  <= dv_q(32 downto 0) & '1';
                else
                    dv_ra <= v_r;
                    dv_q  <= dv_q(32 downto 0) & '0';
                end if;
                dv_na <= dv_na(32 downto 0) & '0';
                if dv_cnt = 0 then dv_bsy <= '0';
                else dv_cnt <= dv_cnt - 1; end if;
            end if;

            if frd_start = '1' then
                dv_na  <= resize(frd_ns, 34);
                dv_da  <= frd_ds;
                dv_ra  <= (others => '0');
                dv_q   <= (others => '0');
                dv_cnt <= to_unsigned(33, 6);
                dv_bsy <= '1';
            end if;
        end if;
    end process p_dmath;

    ------------------------------------------------------------------------
    -- FRAME FSM (microcoded).  All per-frame vertex/basis data lives in the
    -- FRAM scratch EBR (one read + one write port), so no state instantiates
    -- register-array muxes; the shared multiplier (fm), divider (fd) and the
    -- single quarter-sine ROM port (t_sk/t_srd) do all the heavy math.
    -- FRAM map: 0..8 basis (vec*3+comp) | 18+c*8+k corners (c=0/1/2=x/y/z)
    --           42+k rx | 50+k ry | 58+k sx | 66+k sy   (k = corner 0..7)
    ------------------------------------------------------------------------
    ------------------------------------------------------------------------
    -- What survives of the old frame FSM: latch the knobs and the raster
    -- measurements at vblank, and clear the first-frame snap once the engine
    -- has consumed it.  Everything else is microcode now.
    ------------------------------------------------------------------------
    p_fsetup : process(clk)
    begin
        if rising_edge(clk) then
            if s_fstart = '1' then
                fr_done <= '0';
                s_k1 <= unsigned(registers_in(0));
                s_k2 <= unsigned(registers_in(1));
                s_k3 <= unsigned(registers_in(2));
                if s_ilace = '1' then s_hf <= resize(s_H & '0', 13);
                else                  s_hf <= resize(s_H, 13); end if;
                s_cx <= '0' & s_W(11 downto 1);
                s_cy <= resize(s_hf(12 downto 1), 12);
                hmax <= to_unsigned(1, 15);
                wmax <= to_unsigned(1, 15);
            else
                fr_done <= u_done;
                if u_done = '1' then s_zfirst <= '0'; end if;
            end if;
        end if;
    end process p_fsetup;

    ------------------------------------------------------------------------
    -- SLW decoder.  Ports 0..14 are per-slot; 15 and 16 are globals, which
    -- is why the port field is five bits and not four.
    ------------------------------------------------------------------------
    p_slotw : process(clk)
        variable v_s : integer range 0 to 2;
    begin
        if rising_edge(clk) then
            -- slot 3 never exists: the loop stops at three visible faces
            if u_slw = '1' and u_sls /= 3 then
                v_s := to_integer(u_sls(1 downto 0));
                case to_integer(u_slp) is
                    when  0 => sl_gux(v_s) <= resize(u_slv, 17);
                    when  1 => sl_guy(v_s) <= resize(u_slv, 17);
                    when  2 => sl_gvx(v_s) <= resize(u_slv, 17);
                    when  3 => sl_gvy(v_s) <= resize(u_slv, 17);
                    when  4 => sl_px0(v_s) <= resize(u_slv, 13);
                    when  5 => sl_py0(v_s) <= resize(u_slv, 13);
                    when  6 => sl_face(v_s) <= resize(unsigned(u_slv(2 downto 0)), 3);
                    when  7 => sl_sil(v_s) <= std_logic_vector(u_slv(3 downto 0));
                    when  8 => sl_cu(v_s) <= resize(unsigned(u_slv(9 downto 0)), 10);
                    when  9 => sl_cv(v_s) <= resize(unsigned(u_slv(9 downto 0)), 10);
                    when 10 => sl_ly(v_s) <= resize(unsigned(u_slv(9 downto 0)), 10);
                    when 11 => sl_lf(v_s) <= resize(unsigned(u_slv(7 downto 0)), 8);
                    when 12 => sl_hn(v_s) <= resize(u_slv, 10);
                    when 13 => sl_hu(v_s) <= resize(u_slv, 10);
                    when 14 => sl_hv(v_s) <= resize(u_slv, 10);
                    when 15 => s_nslot <= resize(unsigned(u_slv(1 downto 0)), 2);
                    when others => s_qx0 <= resize(unsigned(u_slv(21 downto 0)), 22);
                end case;
            end if;
        end if;
    end process p_slotw;

    ------------------------------------------------------------------------
    -- MICROCODED GEOMETRY ENGINE
    --
    -- Replaces ~110 hand-written frame-setup states.  Each of those states
    -- carried its own adders, comparators and address arithmetic, and each
    -- one that touched the multiplier was another input to an 18-bit operand
    -- mux; together that measured 2,666 cells for math that runs once per
    -- frame in blanking with ~99,000 spare clocks.
    --
    -- Here there is ONE ALU, the scratch RAM is the register file (two
    -- copies so an instruction can read both operands in a cycle), and the
    -- program lives in a block RAM.  Three cycles per instruction, which at
    -- ~1,500 dynamic instructions is ~4,500 clocks of the vblank budget.
    ------------------------------------------------------------------------
    p_ueng : process(clk)
        variable v_a, v_b : signed(31 downto 0);
        variable v_r      : signed(31 downto 0);
        variable v_op     : integer range 0 to 31;
        variable v_imm    : signed(13 downto 0);
        variable v_q      : unsigned(16 downto 0);
        variable v_sh     : signed(31 downto 0);
    begin
        if rising_edge(clk) then
            u_we   <= '0';
            g_we   <= '0';
            mu_go_f <= '0';
            frd_start <= '0';

            if s_fstart = '1' then
                u_pc   <= (others => '0');
                u_st   <= "00";
                u_gwt  <= (others => '0');
                u_done <= '0';
            elsif u_done = '0' then
                case to_integer(u_st) is

                    -- FETCH: the ROM answers next cycle
                    when 0 =>
                        u_st <= "01";

                    -- DECODE: latch the instruction, issue both register reads
                    when 1 =>
                        u_ir <= u_rom;
                        u_ra <= unsigned(u_rom(27 downto 21));
                        u_rb <= unsigned(u_rom(20 downto 14));
                        u_st <= "10";

                    -- INDEXED second access (LDX/STX only)
                    when 3 =>
                        u_st <= "00";
                        u_pc <= u_pc + 1;
                        if to_integer(unsigned(u_ir(39 downto 35))) = 30 then
                            u_wa <= unsigned(u_ir(34 downto 28));
                            u_wd <= u_rda;
                            u_we <= '1';
                        else
                            u_wa <= u_xa;
                            u_wd <= std_logic_vector(resize(u_xd, 32));
                            u_we <= '1';
                        end if;

                    -- EXECUTE
                    when others =>
                        v_op  := to_integer(unsigned(u_ir(39 downto 35)));
                        v_imm := signed(u_ir(13 downto 0));
                        v_a   := resize(signed(u_rda), 32);
                        v_b   := resize(signed(u_rdb), 32);
                        v_r   := (others => '0');
                        u_st  <= "00";
                        u_pc  <= u_pc + 1;

                        case v_op is
                            when 1  => v_r := v_a;
                            when 2  => v_r := v_a + v_b;
                            when 3  => v_r := v_a - v_b;
                            when 4  => v_r := shift_right(v_a, to_integer(unsigned(v_imm(3 downto 0))));
                            when 5  => v_r := shift_left(v_a, to_integer(unsigned(v_imm(3 downto 0))));
                            when 6  => v_r := -v_a;
                            when 7  => if v_a < 0 then v_r := -v_a; else v_r := v_a; end if;
                            when 8  => if v_a < v_b then v_r := v_a; else v_r := v_b; end if;
                            when 9  => if v_a > v_b then v_r := v_a; else v_r := v_b; end if;
                            when 10 =>                     -- MUL: start
                                fm_a <= resize(v_a, 18);
                                fm_b <= resize(v_b, 14);
                                mu_go_f <= '1';
                            when 11 =>                     -- MRD: stall, then take
                                if mu_idle = '0' then
                                    u_st <= "10";
                                    u_pc <= u_pc;
                                else
                                    v_r := resize(shift_right(mu_p,
                                              to_integer(unsigned(v_imm(4 downto 0)))), 32);
                                end if;
                            when 12 =>                     -- DIV: start
                                frd_ns <= shift_left(resize(unsigned(abs(v_a)), 32),
                                                     to_integer(unsigned(v_imm(4 downto 0))));
                                if v_b = 0 then frd_ds <= to_unsigned(1, 21);
                                else frd_ds <= resize(unsigned(abs(v_b)), 21); end if;
                                fd_sgn <= (v_a(31) xor v_b(31));
                                frd_start <= '1';
                            when 13 =>                     -- DRD: stall, then take
                                -- the quotient is a MAGNITUDE up to 65535, so it
                                -- has to be zero-extended before the sign goes on
                                if dv_bsy = '1' then
                                    u_st <= "10";
                                    u_pc <= u_pc;
                                else
                                    if dv_q > 65535 then
                                        v_q := to_unsigned(65535, 17);
                                    else
                                        v_q := resize(dv_q(15 downto 0), 17);
                                    end if;
                                    if fd_sgn = '1' then
                                        v_r := -signed(resize(v_q, 32));
                                    else
                                        v_r := signed(resize(v_q, 32));
                                    end if;
                                end if;
                            when 14 => v_r := resize(C_SIN(to_integer(
                                              unsigned(v_a(7 downto 0)))), 32);
                            when 15 => v_r := resize(v_imm, 32);
                            when 16 =>                     -- GWR
                                g_wa <= resize(unsigned(v_imm(7 downto 0))
                                               + unsigned(v_a(7 downto 0)), 8);
                                g_wd <= std_logic_vector(resize(v_b, 16));
                                g_we <= '1';
                            when 17 =>                     -- SLW
                                u_slp <= unsigned(v_imm(4 downto 0));
                                u_slv <= resize(v_a, 32);
                                u_sls <= unsigned(v_b(1 downto 0));
                                u_slw <= '1';
                            when 18 => u_pc <= unsigned(v_imm(9 downto 0));
                            when 19 => if v_a /= 0 then
                                           u_pc <= unsigned(v_imm(9 downto 0));
                                       end if;
                            when 20 => if v_a < v_b then
                                           u_pc <= unsigned(v_imm(9 downto 0));
                                       end if;
                            when 21 => if v_a >= v_b then
                                           u_pc <= unsigned(v_imm(9 downto 0));
                                       end if;
                            when 22 =>
                                if v_a < 0 then v_r := (others => '0');
                                elsif v_a > v_b then v_r := v_b;
                                else v_r := v_a; end if;
                            when 23 =>
                                -- GAM reads the SAME gamma ROM the pixel path
                                -- uses, so it costs no second table: drive the
                                -- address, stall one cycle, then take gm_d.
                                -- address registers, then the ROM registers:
                                -- two stall cycles, not one.
                                if u_gwt /= 2 then
                                    u_gama <= unsigned(v_a(7 downto 0));
                                    u_gwt  <= u_gwt + 1;
                                    u_st   <= "10";
                                    u_pc   <= u_pc;
                                else
                                    u_gwt <= (others => '0');
                                    v_r := signed(resize(gm_d, 32));
                                end if;
                            when 25 => v_r := u_ctlv;      -- knobs and raster
                            when 27 => v_r := v_a and signed(resize(unsigned(v_imm), 32));
                            when 28 =>
                                v_sh := shift_right(v_a,
                                            to_integer(unsigned(v_imm(3 downto 0))));
                                if v_sh(0) = '1' then v_r := to_signed(1, 32);
                                else                  v_r := (others => '0');
                                end if;
                            when 29 => v_r := v_a or shift_left(to_signed(1, 32),
                                              to_integer(unsigned(v_imm(3 downto 0))));
                            when 30 | 31 =>                -- LDX / STX
                                u_xa <= resize(unsigned(v_a(6 downto 0))
                                               + unsigned(v_imm(6 downto 0)), 7);
                                u_xd <= resize(v_b, 32);
                                u_ra <= resize(unsigned(v_a(6 downto 0))
                                               + unsigned(v_imm(6 downto 0)), 7);
                                u_st <= "11";
                                u_pc <= u_pc;
                            when 24 => u_done <= '1';
                            when others => null;
                        end case;

                        -- one write port, one write per instruction: every op
                        -- that produced a value in v_r above, and only those
                        if (v_op >= 1 and v_op <= 9) or v_op = 11 or v_op = 13
                           or v_op = 14 or v_op = 15 or v_op = 22 or v_op = 23
                           or v_op = 25 or (v_op >= 27 and v_op <= 29) then
                          -- ...but not on the cycle a read op is still stalling
                          if not ((v_op = 11 and mu_idle = '0')
                                  or (v_op = 13 and dv_bsy = '1')
                                  or (v_op = 23 and u_gwt /= 2)) then
                            u_wa <= unsigned(u_ir(34 downto 28));
                            u_wd <= std_logic_vector(resize(v_r, 32));
                            u_we <= '1';
                          end if;
                        end if;
                end case;
            end if;
        end if;
    end process p_ueng;

    ------------------------------------------------------------------------
    -- CTL reads the outside world.  u_ir is latched at DECODE, so selecting
    -- on its immediate field is stable through EXECUTE.
    ------------------------------------------------------------------------
    with to_integer(unsigned(u_ir(3 downto 0))) select u_ctlv <=
        signed(resize(s_k1, 32)) when 0,
        signed(resize(s_k2, 32)) when 1,
        signed(resize(s_k3, 32)) when 2,
        signed(resize(s_W, 32)) when 3,
        signed(resize(s_H, 32)) when 4,
        signed(resize(s_hf, 32)) when 5,
        signed(resize(s_cx, 32)) when 6,
        signed(resize(s_cy, 32)) when 7,
        (0 => s_ilace, others => '0')    when 8,
        (0 => s_zfirst, others => '0')   when 10,
        (others => '0')                  when others;

    ------------------------------------------------------------------------
    -- microcode ROM and the two register-file copies (dual read)
    ------------------------------------------------------------------------
    p_urom : process(clk)
    begin
        if rising_edge(clk) then
            u_rom <= C_UCODE(to_integer(u_pc));
        end if;
    end process p_urom;

    p_rfa : process(clk)
    begin
        if rising_edge(clk) then
            if u_we = '1' then rf_a(to_integer(u_wa)) <= u_wd; end if;
            u_rda <= rf_a(to_integer(u_ra));
        end if;
    end process p_rfa;

    p_rfb : process(clk)
    begin
        if rising_edge(clk) then
            if u_we = '1' then rf_b(to_integer(u_wa)) <= u_wd; end if;
            u_rdb <= rf_b(to_integer(u_rb));
        end if;
    end process p_rfb;


    ------------------------------------------------------------------------
    -- geometry EBR (write: frame FSM, read: line FSM)
    ------------------------------------------------------------------------
    p_gram_w : process(clk)
    begin
        if rising_edge(clk) then
            if g_we = '1' then
                gram(to_integer(g_wa)) <= g_wd;
            end if;
        end if;
    end process p_gram_w;

    ------------------------------------------------------------------------
    -- MATERIAL ROM ports.  Address combinational, data registered -- one
    -- cycle, the same shape as every other memory here.
    ------------------------------------------------------------------------
    -- Squared-distance tables.  The zone mux lives HERE rather than in the
    -- pixel process: reading the stage-C outputs directly keeps the data
    -- one cycle behind them, exactly where the old multipliers put it.
    p_sqromu : process(clk)
        variable v : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            if e_stz = '1' then v := e_qsu;
            elsif e_silz = '1' then v := e_mu;
            else v := e_gu; end if;
            sq_du <= C_SQA(to_integer(v(8 downto 1)));
        end if;
    end process p_sqromu;

    p_sqromv : process(clk)
        variable v : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            if e_stz = '1' then v := e_qsv;
            elsif e_silz = '1' then v := e_mv;
            else v := e_gv; end if;
            sq_dv <= C_SQB(to_integer(v(8 downto 1)));
        end if;
    end process p_sqromv;

    p_tromu : process(clk)
    begin
        if rising_edge(clk) then
            tr_du <= C_TILTA(to_integer(e_tu));
        end if;
    end process p_tromu;

    p_tromv : process(clk)
    begin
        if rising_edge(clk) then
            tr_dv <= C_TILTB(to_integer(e_tv));
        end if;
    end process p_tromv;

    p_srom2 : process(clk)
    begin
        if rising_edge(clk) then
            sp_d <= C_SPEC(to_integer(sp_a));
        end if;
    end process p_srom2;

    -- one gamma port, shared: the frame FSM only runs with fr_done = '0' and
    -- the pixel path only matters with it '1', exactly like the multiplier.
    p_gamrom : process(clk)
    begin
        if rising_edge(clk) then
            gm_d <= C_GAM(to_integer(gm_a));
        end if;
    end process p_gamrom;

    gm_a <= t_lf when fr_done = '0' else gm_ap;

    mu_idle <= '1' when (mu_bsy = '0' and mu_go_f = '0' and mu_go_l = '0')
              else '0';

    -- the ONE quarter-sine ROM read port
    p_srom : process(clk)
    begin
        if rising_edge(clk) then
            t_srd <= resize(C_SIN(to_integer(t_sk)), 14);
        end if;
    end process p_srom;

    p_gram_r : process(clk)
    begin
        if rising_edge(clk) then
            g_rd <= gram(to_integer(g_ra));
        end if;
    end process p_gram_r;

    ------------------------------------------------------------------------
    -- display list EBR (write: line FSM, read: pixel prefetcher)
    ------------------------------------------------------------------------
    p_dram_w : process(clk)
    begin
        if rising_edge(clk) then
            if d_we = '1' then
                dram(to_integer(d_wa)) <= d_wd;
            end if;
        end if;
    end process p_dram_w;

    p_dram_r : process(clk)
    begin
        if rising_edge(clk) then
            d_rd <= dram(to_integer(d_ra));
        end if;
    end process p_dram_r;



    ------------------------------------------------------------------------
    -- LINE FSM.  With an affine (orthographic) UV map there are no segments
    -- and no per-span division: each visible face contributes ONE record --
    -- span extent plus the UV value at the span's first pixel, obtained from
    -- the per-face screen gradients.  The pixel path then just accumulates.
    ------------------------------------------------------------------------
    p_line : process(clk)
        variable v_tgt  : unsigned(10 downto 0);
        variable v_yf   : unsigned(11 downto 0);
        variable v_e1   : integer range 0 to 3;
        variable v_x    : signed(15 downto 0);
        variable v_q    : signed(15 downto 0);
        variable v_sel  : unsigned(1 downto 0);
        variable v_best : signed(13 downto 0);
        variable v_go   : std_logic;
        variable v_uv   : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            d_we <= '0';
            mu_go_l <= '0';

            v_go := '0';
            if fr_done = '1' then
                if s_avid_r = '1' then
                    v_go := '1';
                    v_tgt := s_ycnt + 1;
                elsif s_hs_r = '1' and s_ycnt = 0 then
                    v_go := '1';
                    v_tgt := (others => '0');
                end if;
            end if;

            if v_go = '1' then
                if s_ilace = '1' then
                    if s_field = '0' then v_yf := (v_tgt(10 downto 0) & '0') + 1;
                    else                  v_yf := v_tgt(10 downto 0) & '0'; end if;
                else
                    v_yf := '0' & v_tgt(10 downto 0);
                end if;
                ln_ys   <= signed(resize(v_yf, 16));
                ln_yc2  <= signed(resize(v_yf & "10", 16));
                d_wp    <= v_tgt(0) & "0000000";
                sp_ok   <= (others => '0');
                ln_i    <= (others => '0');
                if v_tgt >= s_H then ln_st <= to_unsigned(30, 6);
                else                 ln_st <= to_unsigned(1, 6); end if;
            else
                case to_integer(ln_st) is
                    when 0 => null;
                    when 1 =>
                        g_ra <= to_unsigned(127, 8);
                        ln_st <= to_unsigned(2, 6);
                    when 2 => ln_st <= to_unsigned(3, 6);
                    when 3 =>
                        nslot_l <= unsigned(g_rd(1 downto 0));
                        ln_st <= to_unsigned(4, 6);

                    ------------------------------------------------------
                    -- span scan (ln_i = slot): 4 edge intersections
                    ------------------------------------------------------
                    when 4 =>
                        if ln_i > 2 or resize(ln_i, 2) >= nslot_l then
                            ln_st <= to_unsigned(12, 6);
                        else
                            g_ra <= shift_left(resize(ln_i, 8), 5);
                            ln_st <= to_unsigned(5, 6);
                        end if;
                    when 5 =>
                        g_ra <= shift_left(resize(ln_i, 8), 5) + 1;
                        ln_st <= to_unsigned(6, 6);
                    when 6 =>
                        ln_t0 <= signed(g_rd);
                        ln_st <= to_unsigned(7, 6);
                    when 7 =>
                        if ln_ys < ln_t0 or ln_ys >= signed(g_rd) then
                            sp_ok(to_integer(ln_i)) <= '0';
                            ln_i <= ln_i + 1;
                            ln_st <= to_unsigned(4, 6);
                        else
                            ln_e <= (others => '0');
                            ln_xmin <= to_signed(32767, 16);
                            ln_xmax <= to_signed(-32768, 16);
                            ln_st <= to_unsigned(8, 6);
                        end if;
                    when 8 =>
                        g_ra <= shift_left(resize(ln_i, 8), 5) + 2
                                + resize(ln_e(1 downto 0) & '0', 8)
                                + resize(ln_e(1 downto 0), 8);
                        ln_st <= to_unsigned(9, 6);
                    when 9 => ln_st <= to_unsigned(10, 6);
                    when 10 =>
                        cy4(to_integer(ln_e(1 downto 0))) <= signed(g_rd);
                        if ln_e = 3 then
                            ln_e <= (others => '0');
                            ln_st <= to_unsigned(11, 6);
                        else
                            ln_e <= ln_e + 1;
                            ln_st <= to_unsigned(8, 6);
                        end if;
                    when 11 =>
                        v_e1 := (to_integer(ln_e(1 downto 0)) + 1) mod 4;
                        if (ln_yc2 >= cy4(to_integer(ln_e(1 downto 0))))
                           /= (ln_yc2 >= cy4(v_e1)) then
                            g_ra <= shift_left(resize(ln_i, 8), 5) + 3
                                    + resize(ln_e(1 downto 0) & '0', 8)
                                    + resize(ln_e(1 downto 0), 8);
                            ln_st <= to_unsigned(13, 6);
                        elsif ln_e = 3 then
                            ln_st <= to_unsigned(18, 6);
                        else
                            ln_e <= ln_e + 1;
                        end if;
                    when 13 =>
                        g_ra <= shift_left(resize(ln_i, 8), 5) + 4
                                + resize(ln_e(1 downto 0) & '0', 8)
                                + resize(ln_e(1 downto 0), 8);
                        ln_st <= to_unsigned(14, 6);
                    when 14 =>
                        ln_edge_x0 <= signed(g_rd);
                        ln_st <= to_unsigned(15, 6);
                    when 15 =>
                        lm_a <= resize(signed(g_rd), 18);
                        lm_b <= resize(ln_yc2 - cy4(to_integer(ln_e(1 downto 0))), 14);
                        mu_go_l <= '1';
                        ln_st <= to_unsigned(16, 6);
                    when 16 =>
                        if mu_idle = '1' then ln_st <= to_unsigned(17, 6); end if;
                    when 17 =>
                        v_x := ln_edge_x0 + resize(shift_right(mu_p, 6), 16);
                        if v_x < ln_xmin then ln_xmin <= v_x; end if;
                        if v_x > ln_xmax then ln_xmax <= v_x; end if;
                        if ln_e = 3 then
                            ln_st <= to_unsigned(18, 6);
                        else
                            ln_e <= ln_e + 1;
                            ln_st <= to_unsigned(11, 6);
                        end if;
                    when 18 =>
                        v_x := shift_right(ln_xmin, 2);
                        if v_x < 0 then v_x := (others => '0'); end if;
                        sp_xl(to_integer(ln_i)) <= resize(v_x, 14);
                        v_q := shift_right(ln_xmax, 2) + 1;
                        if v_q > signed(resize(s_W, 16)) then
                            v_q := signed(resize(s_W, 16));
                        end if;
                        sp_xr(to_integer(ln_i)) <= resize(v_q, 14);
                        if ln_xmin <= ln_xmax and v_q > v_x then
                            sp_ok(to_integer(ln_i)) <= '1';
                        else
                            sp_ok(to_integer(ln_i)) <= '0';
                        end if;
                        ln_i <= ln_i + 1;
                        ln_st <= to_unsigned(4, 6);

                    ------------------------------------------------------
                    -- x-sort the up-to-3 spans
                    ------------------------------------------------------
                    when 12 =>
                        sp_msk <= sp_ok;
                        ocnt <= (others => '0');
                        ln_st <= to_unsigned(19, 6);
                    when 19 | 20 | 21 =>
                        v_best := to_signed(8191, 14);
                        v_sel := "11";
                        for i in 0 to 2 loop
                            if sp_msk(i) = '1' and sp_xl(i) < v_best then
                                v_best := sp_xl(i);
                                v_sel := to_unsigned(i, 2);
                            end if;
                        end loop;
                        if v_sel /= "11" then
                            sp_msk(to_integer(v_sel)) <= '0';
                            case to_integer(ln_st) is
                                when 19 => ord0 <= v_sel;
                                when 20 => ord1 <= v_sel;
                                when others => ord2 <= v_sel;
                            end case;
                            ocnt <= ocnt + 1;
                        end if;
                        if ln_st = 21 then
                            ln_k <= (others => '0');
                            ln_st <= to_unsigned(22, 6);
                        else
                            ln_st <= ln_st + 1;
                        end if;

                    ------------------------------------------------------
                    -- one record per span: u,v at the first pixel come from
                    -- the face gradients (2 multiplies each), then the pixel
                    -- path accumulates gu_x / gv_x across the span
                    ------------------------------------------------------
                    when 22 =>
                        if resize(ln_k, 2) >= ocnt then
                            ln_st <= to_unsigned(30, 6);
                        else
                            case to_integer(ln_k) is
                                when 0 => v_sel := ord0;
                                when 1 => v_sel := ord1;
                                when others => v_sel := ord2;
                            end case;
                            ln_slot <= v_sel;
                            ln_x0 <= sp_xl(to_integer(v_sel));
                            ln_x1 <= sp_xr(to_integer(v_sel));
                            ln_dxp <= resize(sp_xl(to_integer(v_sel))
                                             - resize(sl_px0(to_integer(v_sel)), 14), 14);
                            ln_dyp <= resize(resize(ln_ys, 14)
                                             - resize(sl_py0(to_integer(v_sel)), 14), 14);
                            ln_st <= to_unsigned(23, 6);
                        end if;
                    -- Four products for the span's UV seed, one at a time on
                    -- the serial multiplier.  The line engine runs a whole
                    -- line ahead of the pixel it feeds, so ~2200 clocks are
                    -- available and 4 x 16 costs nothing.
                    when 23 =>
                        lm_a <= resize(sl_gux(to_integer(ln_slot)), 18);
                        lm_b <= ln_dxp;
                        mu_go_l <= '1';
                        ln_st <= to_unsigned(24, 6);
                    when 24 =>
                        if mu_idle = '1' then
                            ln_acc <= resize(mu_p, 32);
                            lm_a <= resize(sl_guy(to_integer(ln_slot)), 18);
                            lm_b <= ln_dyp;
                            mu_go_l <= '1';
                            ln_st <= to_unsigned(25, 6);
                        end if;
                    when 25 =>
                        if mu_idle = '1' then
                            ln_acc <= ln_acc + resize(mu_p, 32);
                            lm_a <= resize(sl_gvx(to_integer(ln_slot)), 18);
                            lm_b <= ln_dxp;
                            mu_go_l <= '1';
                            ln_st <= to_unsigned(26, 6);
                        end if;
                    when 26 =>
                        if mu_idle = '1' then
                            ln_acc2 <= resize(mu_p, 32);
                            v_uv := resize(shift_right(ln_acc, 8), 20);
                            if v_uv > 16383 then v_uv := to_signed(16383, 20); end if;
                            if v_uv < -16384 then v_uv := to_signed(-16384, 20); end if;
                            ln_useed <= resize(v_uv, 16);
                            lm_a <= resize(sl_gvy(to_integer(ln_slot)), 18);
                            lm_b <= ln_dyp;
                            mu_go_l <= '1';
                            ln_st <= to_unsigned(27, 6);
                        end if;
                    when 27 =>
                        if mu_idle = '1' then
                            ln_acc2 <= ln_acc2 + resize(mu_p, 32);
                            ln_st <= to_unsigned(29, 6);
                        end if;
                    when 28 => null;
                    when 29 =>
                        v_uv := resize(shift_right(ln_acc2, 8), 20);
                        if v_uv > 16383 then v_uv := to_signed(16383, 20); end if;
                        if v_uv < -16384 then v_uv := to_signed(-16384, 20); end if;
                        ln_vseed <= resize(v_uv, 16);
                        ln_st <= to_unsigned(31, 6);
                    when 31 =>
                        d_wa <= d_wp;  d_wp <= d_wp + 1;  d_we <= '1';
                        d_wd <= "00" & std_logic_vector(ln_slot)
                                & std_logic_vector(resize(unsigned(resize(ln_x0, 14)), 12));
                        ln_st <= to_unsigned(32, 6);
                    when 32 =>
                        d_wa <= d_wp;  d_wp <= d_wp + 1;  d_we <= '1';
                        d_wd <= "0000" & std_logic_vector(resize(unsigned(resize(ln_x1, 14)), 12));
                        ln_st <= to_unsigned(33, 6);
                    when 33 =>
                        d_wa <= d_wp;  d_wp <= d_wp + 1;  d_we <= '1';
                        d_wd <= std_logic_vector(ln_useed);
                        ln_st <= to_unsigned(34, 6);
                    when 34 =>
                        d_wa <= d_wp;  d_wp <= d_wp + 1;  d_we <= '1';
                        d_wd <= std_logic_vector(ln_vseed);
                        ln_k <= ln_k + 1;
                        ln_st <= to_unsigned(22, 6);

                    ------------------------------------------------------
                    -- end-of-line marker
                    ------------------------------------------------------
                    when 30 =>
                        d_wa <= d_wp;  d_wp <= d_wp + 1;  d_we <= '1';
                        d_wd <= "1100" & x"FFF";
                        ln_e <= (others => '0');
                        ln_st <= to_unsigned(35, 6);
                    when 35 =>
                        d_wa <= d_wp;  d_wp <= d_wp + 1;  d_we <= '1';
                        d_wd <= (others => '0');
                        if ln_e = 2 then ln_st <= (others => '0');
                        else ln_e <= ln_e + 1; end if;

                    when others =>
                        ln_st <= (others => '0');
                end case;
            end if;
        end if;
    end process p_line;

    ------------------------------------------------------------------------
    -- record prefetcher: keeps the NEXT display-list record in nx_* shadow
    -- regs (record spacing >= ~28 px guarantees the 7-cycle fetch fits)
    ------------------------------------------------------------------------
    p_pref : process(clk)
    begin
        if rising_edge(clk) then
            if s_hs_r = '1' then
                pf_rdp <= s_ycnt(0) & "0000000";
                nx_end <= '1';
                pf_st  <= (others => '0');
            else
                case to_integer(pf_st) is
                    when 0 =>
                        d_ra <= pf_rdp;
                        pf_st <= to_unsigned(1, 3);
                    when 1 =>
                        d_ra <= pf_rdp + 1;
                        pf_st <= to_unsigned(2, 3);
                    when 2 =>
                        nx_x   <= unsigned(d_rd(11 downto 0));
                        nx_sl  <= unsigned(d_rd(13 downto 12));
                        nx_end <= d_rd(15);
                        d_ra <= pf_rdp + 2;
                        pf_st <= to_unsigned(3, 3);
                    when 3 =>
                        nx_xe <= unsigned(d_rd(11 downto 0));
                        d_ra <= pf_rdp + 3;
                        pf_st <= to_unsigned(4, 3);
                    when 4 =>
                        nx_u <= signed(d_rd);
                        pf_st <= to_unsigned(5, 3);
                    when 5 =>
                        nx_v <= signed(d_rd);
                        pf_rdp <= pf_rdp + 4;
                        pf_st <= to_unsigned(6, 3);
                    when others =>
                        if pf_consume = '1' then
                            pf_st <= (others => '0');
                        end if;
                end case;
            end if;
        end if;
    end process p_pref;

    ------------------------------------------------------------------------
    -- background: gradient + horizontal vignette, all accumulators
    ------------------------------------------------------------------------
    p_bg : process(clk)
        variable v_vig : unsigned(17 downto 0);
        variable v_y   : unsigned(17 downto 0);
        variable v_dx  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_fstart = '1' then
                bg_ly <= to_unsigned(644 * 256, 18);
                bg_uq <= to_unsigned(506 * 256, 18);
                bg_vq <= to_unsigned(516 * 256, 18);
            elsif s_avid_f = '1' then
                if s_H > 700 then
                    bg_ly <= bg_ly - 56;
                    bg_uq <= bg_uq + 3;
                    bg_vq <= bg_vq - 2;
                else
                    bg_ly <= bg_ly - 126;
                    bg_uq <= bg_uq + 6;
                    bg_vq <= bg_vq - 4;
                end if;
            end if;

            -- horizontal falloff straight off the pixel counter: a triangle
            -- reads the same as the parabola once it is this soft, and costs
            -- one subtract instead of a pair of wide accumulators
            if s_xcnt > s_cx then v_dx := resize(s_xcnt - s_cx, 12);
            else                  v_dx := resize(s_cx - s_xcnt, 12); end if;
            if s_W > 1200 then v_dx := shift_right(v_dx, 1); end if;
            v_vig := resize(shift_left(resize(v_dx, 18), 4)
                            + shift_left(resize(v_dx, 18), 3), 18);
            if bg_ly > v_vig then v_y := bg_ly - v_vig;
            else v_y := (others => '0'); end if;
            bg_vy <= v_y(17 downto 8);

            bgy_sr(0) <= bg_vy;
            for i in 1 to 13 loop
                bgy_sr(i) <= bgy_sr(i - 1);
            end loop;
            bg_u <= bg_uq(17 downto 8);
            bg_v <= bg_vq(17 downto 8);
        end if;
    end process p_bg;

    ------------------------------------------------------------------------
    -- PIXEL PIPE (stages A..L + output reg = C_LATENCY 13)
    ------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_off  : signed(15 downto 0);
        variable v_apu, v_apv : signed(14 downto 0);
        variable v_mu, v_mv : unsigned(8 downto 0);
        variable v_du, v_dv : signed(12 downto 0);
        variable v_outu, v_outv, v_su, v_sv : std_logic;
        variable v_qsu, v_qsv : signed(13 downto 0);
        variable v_gqu, v_gqv : unsigned(8 downto 0);
        variable v_ad   : signed(13 downto 0);
        variable v_am   : signed(16 downto 0);
        variable v_str  : signed(12 downto 0);
        variable v_qcu, v_qcv : unsigned(8 downto 0);
        variable v_w    : signed(27 downto 0);
        variable v_w8   : unsigned(25 downto 0);
        variable v_a    : signed(15 downto 0);
        variable v_a5   : unsigned(4 downto 0);
        variable v_dy   : signed(11 downto 0);
        variable v_c    : signed(16 downto 0);
        variable v_c19  : signed(19 downto 0);
        variable v_cm1  : unsigned(9 downto 0);
        variable v_hh   : signed(9 downto 0);
        variable v_del  : signed(11 downto 0);
        variable v_nd   : signed(12 downto 0);
        variable v_sed  : unsigned(7 downto 0);
        variable v_dsx  : signed(10 downto 0);
        variable v_ramp : signed(10 downto 0);
        variable v_li   : signed(13 downto 0);
        variable v_dir  : signed(13 downto 0);
        variable v_occ  : signed(13 downto 0);
        variable v_t8   : signed(8 downto 0);
        variable v_sil4 : std_logic_vector(3 downto 0);
        variable v_face : integer range 0 to 5;
    begin
        if rising_edge(clk) then
            pf_consume <= '0';

            ------------------------------------------------------ stage A
            -- Affine UV: one accumulator pair, re-seeded at each span start
            -- and stepped by that face's screen gradient.  No division and no
            -- segment records anywhere in the pixel path.
            if data_in.avid = '1' then
                if nx_end = '0' and s_xcnt = nx_x then
                    uacc <= shift_left(resize(nx_u, 26), 8);
                    vacc <= shift_left(resize(nx_v, 26), 8);
                    cu_sl <= nx_sl;
                    cu_xe <= nx_xe;
                    cu_du <= resize(sl_gux(to_integer(nx_sl)), 26);
                    cu_dv <= resize(sl_gvx(to_integer(nx_sl)), 26);
                    cu_on <= '1';
                    pf_consume <= '1';
                    b_u <= resize(shift_right(shift_left(resize(nx_u, 26), 8), 8), 16);
                    b_v <= resize(shift_right(shift_left(resize(nx_v, 26), 8), 8), 16);
                    b_sl <= nx_sl;
                    b_on <= '1';
                else
                    if cu_on = '1' and s_xcnt >= cu_xe then
                        cu_on <= '0';
                    end if;
                    uacc <= uacc + cu_du;
                    vacc <= vacc + cu_dv;
                    b_u <= resize(shift_right(uacc, 8), 16);
                    b_v <= resize(shift_right(vacc, 8), 16);
                    b_sl <= cu_sl;
                    b_on <= cu_on;
                end if;
            else
                b_on <= '0';
                cu_on <= '0';
            end if;

            ------------------------------------------------------ stage B
            -- Cell split only.  Every boundary distance the shader needs --
            -- cubie gap AND face bevel -- is 2048 - |p| off the in-cubie
            -- coordinate, because the face edge is simply the outermost grid
            -- line.  That collapses eight wide subtract/compare chains into
            -- one per axis.
            if b_u < 4096 then v_off := to_signed(0, 16); c_cu <= "00";
            elsif b_u < 8192 then v_off := to_signed(4096, 16); c_cu <= "01";
            else v_off := to_signed(8192, 16); c_cu <= "10"; end if;
            c_pu <= resize(b_u - v_off - 2048, 14);
            if b_v < 4096 then v_off := to_signed(0, 16); c_cv <= "00";
            elsif b_v < 8192 then v_off := to_signed(4096, 16); c_cv <= "01";
            else v_off := to_signed(8192, 16); c_cv <= "10"; end if;
            c_pv <= resize(b_v - v_off - 2048, 14);
            c_sl <= b_sl;
            c_on <= b_on;

            ------------------------------------------------------ stage C
            v_apu := resize(abs(c_pu), 15);
            v_apv := resize(abs(c_pv), 15);
            v_sil4 := sl_sil(to_integer(c_sl));

            -- distance to the nearest grid line on each axis, saturated to
            -- 12 bits: nothing downstream cares past the bevel radius
            v_du := to_signed(2048, 13) - resize(v_apu, 13);
            if v_du < 0 then v_du := (others => '0'); end if;
            v_dv := to_signed(2048, 13) - resize(v_apv, 13);
            if v_dv < 0 then v_dv := (others => '0'); end if;

            -- is that line the face outline (bevel) or an interior seam (gap)?
            v_outu := '0';  v_su := '0';
            if c_cu = "00" and c_pu < 0 then
                v_outu := '1';  v_su := v_sil4(0);
            elsif c_cu = "10" and c_pu > 0 then
                v_outu := '1';  v_su := v_sil4(1);
            end if;
            v_outv := '0';  v_sv := '0';
            if c_cv = "00" and c_pv < 0 then
                v_outv := '1';  v_sv := v_sil4(2);
            elsif c_cv = "10" and c_pv > 0 then
                v_outv := '1';  v_sv := v_sil4(3);
            end if;

            if v_outu = '1' and v_du < C_RB then
                v_mu := resize(unsigned(resize(to_signed(C_RB, 13) - v_du, 9)), 9);
            else v_mu := (others => '0'); end if;
            if v_outv = '1' and v_dv < C_RB then
                v_mv := resize(unsigned(resize(to_signed(C_RB, 13) - v_dv, 9)), 9);
            else v_mv := (others => '0'); end if;
            if v_outu = '0' and v_du < C_GB then
                v_gqu := resize(unsigned(resize(to_signed(C_GB, 13) - v_du, 9)), 9);
            else v_gqu := (others => '0'); end if;
            if v_outv = '0' and v_dv < C_GB then
                v_gqv := resize(unsigned(resize(to_signed(C_GB, 13) - v_dv, 9)), 9);
            else v_gqv := (others => '0'); end if;
            e_mu <= v_mu;  e_mv <= v_mv;
            e_gu <= v_gqu; e_gv <= v_gqv;

            -- Bevel tilt index.  Both rounded zones -- the face boundary
            -- (radius C_RB) and the cubie rounding at an interior gap (radius
            -- CBW) -- get rescaled onto ONE profile table, so the tilt costs a
            -- shift pair instead of a second ROM: >>1 for the face bevel,
            -- *0.75 for the gap, which lands both at index 164 = 80 degrees.
            -- Outward direction is just the sign of the in-cubie coordinate,
            -- identical in both zones, so it is a single bit.
            if v_outu = '1' then
                e_tu <= resize(shift_right(v_mu, 1), 8);
            else
                e_tu <= resize(shift_right(v_gqu, 1)
                               + shift_right(v_gqu, 2), 8);
            end if;
            if v_outv = '1' then
                e_tv <= resize(shift_right(v_mv, 1), 8);
            else
                e_tv <= resize(shift_right(v_gqv, 1)
                               + shift_right(v_gqv, 2), 8);
            end if;
            e_su <= c_pu(13);
            e_sv <= c_pv(13);
            if v_du < v_dv then e_tmax <= resize(shift_right(v_gqu, 1)
                                                 + shift_right(v_gqu, 2), 8);
            else                e_tmax <= resize(shift_right(v_gqv, 1)
                                                 + shift_right(v_gqv, 2), 8);
            end if;

            -- straight silhouette distance = distance to the nearest outline
            -- edge that actually is a silhouette (a far one can never cut here)
            v_str := to_signed(4095, 13);
            if v_su = '1' then v_str := resize(v_du, 13); end if;
            if v_sv = '1' and resize(v_dv, 13) < v_str then
                v_str := resize(v_dv, 13);
            end if;
            e_dstr <= v_str;
            e_cact <= v_su or v_sv;

            -- Sticker rounded rect.  Take the distance from the ALREADY
            -- CLAMPED corner coordinates: past 511 the edge is far enough
            -- that coverage saturates either way, so carrying the true
            -- (wide) value downstream buys nothing and costs a 15-bit chain.
            v_qsu := resize(v_apu, 14) - C_HSRS;
            v_qsv := resize(v_apv, 14) - C_HSRS;
            if v_qsu < 0 then v_qcu := (others => '0');
            elsif v_qsu > 511 then v_qcu := to_unsigned(511, 9);
            else v_qcu := resize(unsigned(v_qsu(8 downto 0)), 9); end if;
            if v_qsv < 0 then v_qcv := (others => '0');
            elsif v_qsv > 511 then v_qcv := to_unsigned(511, 9);
            else v_qcv := resize(unsigned(v_qsv(8 downto 0)), 9); end if;
            e_qsu <= v_qcu;
            e_qsv <= v_qcv;
            if v_qcu > v_qcv then
                e_dst <= to_signed(C_RS, 11) - signed(resize(v_qcu, 11));
            else
                e_dst <= to_signed(C_RS, 11) - signed(resize(v_qcv, 11));
            end if;

            -- zone select (bevel / gap / sticker are mutually exclusive)
            if v_mu > 0 or v_mv > 0 then
                e_silz <= '1'; e_stz <= '0';
            elsif v_gqu > 0 or v_gqv > 0 then
                e_silz <= '0'; e_stz <= '0';
            else
                e_silz <= '0'; e_stz <= '1';
            end if;
            e_stc <= '0';
            if v_qsu > 0 and v_qsv > 0 then e_stc <= '1'; end if;
            e_sl <= c_sl;
            e_on <= c_on;

            ------------------------------------------------------ stage D
            -- Squared distance by table, muxed by zone (see stage C).  The
            -- index drops the low bit of the 9-bit coordinate; that shifts
            -- the squared distance by under 0.4% and only where it is large,
            -- which is a small fraction of the anti-aliasing feather.

            f_dst <= e_dst;
            f_dstr <= e_dstr;
            f_cact <= e_cact;
            if e_mu > 0 and e_mv > 0 then f_2m <= '1'; else f_2m <= '0'; end if;
            f_silz <= e_silz;
            f_stz <= e_stz;
            f_stc <= e_stc;
            f_sl <= e_sl;
            f_on <= e_on;
            f_su <= e_su;
            f_sv <= e_sv;
            f_tmax <= e_tmax;

            ------------------------------------------------------ stage E
            -- SPECULAR / TILT DOT PRODUCT.  N = cos(phi)*n + sin(phi)*dir, and
            -- the camera is orthographic, so one half-vector serves both the
            -- diffuse tilt and the highlight:
            --   N.H = H.n + [ -(1-cos)*H.n + sin_u*H.U + sin_v*H.V ] / 256
            -- Three 9x10 multiplies -- the whole material rig.  Corners sum
            -- the two axes' (1-cos) terms, which is exact on a straight bevel
            -- (the other axis is zero) and close enough on the arcs.
            v_cm1 := resize(tr_du(7 downto 0), 10) + resize(tr_dv(7 downto 0), 10);
            if v_cm1 > 255 then v_cm1 := to_unsigned(255, 10); end if;
            v_hh := sl_hn(to_integer(f_sl));
            g_p0 <= resize(signed('0' & v_cm1(8 downto 0)) * v_hh, 19);
            v_hh := sl_hu(to_integer(f_sl));
            if f_su = '1' then v_hh := -v_hh; end if;
            g_p1 <= resize(signed('0' & tr_du(15 downto 8)) * v_hh, 19);
            v_hh := sl_hv(to_integer(f_sl));
            if f_sv = '1' then v_hh := -v_hh; end if;
            g_p2 <= resize(signed('0' & tr_dv(15 downto 8)) * v_hh, 19);

            g_sx <= resize(sq_du, 19) + resize(sq_dv, 19);
            g_tmax <= f_tmax;
            g_dst <= f_dst;
            g_dstr <= f_dstr;
            g_cact <= f_cact;
            g_2m <= f_2m;
            g_silz <= f_silz;
            g_stz <= f_stz;
            g_stc <= f_stc;
            g_sl2 <= f_sl;
            g_on <= f_on;

            ------------------------------------------------------ stage F
            -- Linearize the squared distance near its zero crossing.  The
            -- 1/(2r) scale factors (25 and 24) are shift-adds, not multiplies:
            -- x*25>>14 = x>>9 + x>>11 + x>>14,  x*24>>14 = x>>10 + x>>11.
            if g_stz = '1' then
                v_c19 := to_signed(C_RS2, 20) - signed(resize(g_sx, 20));
                h_dlin <= resize(shift_right(v_c19, 7)
                                 + shift_right(v_c19, 8), 12);
            else
                v_c19 := to_signed(C_RB2, 20) - signed(resize(g_sx, 20));
                h_dlin <= resize(shift_right(v_c19, 6)
                                 + shift_right(v_c19, 8)
                                 + shift_right(v_c19, 11), 12);
            end if;
            -- N.H, and the tilt's signed departure from the flat face value
            v_del := resize(shift_right(-g_p0 + g_p1 + g_p2, 8), 12);
            v_nd := resize(sl_hn(to_integer(g_sl2)), 13) + resize(v_del, 13);
            if v_nd < 0 then h_ndh <= (others => '0');
            elsif v_nd > 255 then h_ndh <= to_unsigned(255, 8);
            else h_ndh <= resize(unsigned(v_nd(7 downto 0)), 8); end if;
            h_dif <= resize(v_del, 11);

            -- crevice AO (the old blanket bevel darkening is gone: the tilt
            -- shades the rounding properly now)
            if g_silz = '1' or g_stz = '1' or g_sx <= C_CBW2 then
                h_ao <= (others => '0');
            else
                v_w8 := resize(shift_right(g_sx - to_unsigned(C_CBW2, 19), 5), 26);
                if v_w8 > 255 then h_ao <= to_unsigned(255, 8);
                else h_ao <= resize(v_w8, 8); end if;
            end if;
            h_tmax <= g_tmax;
            h_dst <= g_dst;
            h_dstr <= g_dstr;
            h_cact <= g_cact;
            h_2m <= g_2m;
            h_silz <= g_silz;
            h_stz <= g_stz;
            h_stc <= g_stc;
            h_sl <= g_sl2;
            h_on <= g_on;

            ------------------------------------------------------ stage G
            v_str := h_dstr;
            if h_cact = '1' and h_2m = '1' and h_stz = '0'
               and resize(h_dlin, 14) < resize(v_str, 14) then
                v_str := resize(h_dlin, 13);
            end if;
            i_dsil <= resize(v_str, 14);
            if h_stz = '0' then
                i_ds <= to_signed(-1024, 11);       -- bevel/gap: plastic only
            elsif h_stc = '1' then
                i_ds <= resize(h_dlin, 11);          -- sticker corner arc
            else
                i_ds <= h_dst;                       -- sticker straight edge
            end if;
            i_sl <= h_sl;
            i_on <= h_on;

            -- STICKER EDGE BEVEL.  The inset sticker's own rim turns away from
            -- the light over the last SEW of its edge.  Rather than scale the
            -- albedo (a multiply), subtract the deficit from the light index:
            -- the sticker alpha is already ~1 everywhere this is non-zero.
            v_sed := (others => '0');
            if h_stz = '1' then
                if h_stc = '1' then v_dsx := resize(h_dlin, 11);
                else                v_dsx := h_dst; end if;
                if v_dsx < C_SEW then
                    v_ramp := shift_right(v_dsx, 1) - shift_right(v_dsx, 4);
                    if v_dsx <= 0 then
                        v_sed := to_unsigned(C_SEDEF, 8);
                    elsif v_ramp >= C_SEDEF then
                        v_sed := (others => '0');
                    else
                        v_sed := to_unsigned(C_SEDEF, 8)
                                 - resize(unsigned(v_ramp(6 downto 0)), 8);
                    end if;
                end if;
            end if;

            -- LIGHT INDEX, linear Q8.  Only the DIRECTIONAL part of the face
            -- light may be turned away by the bevel: ambient does not depend
            -- on the normal, so it is held out of the tilt and floors the
            -- shading.  Without this every surface tilting away from the key
            -- crushes to black instead of going dark grey.
            v_dir := resize(signed(resize(sl_lf(to_integer(h_sl)), 12)), 14)
                     - to_signed(C_AMB, 14) + resize(h_dif, 14);
            if v_dir < 0 then v_dir := (others => '0'); end if;
            -- crevice occlusion, scaled to darken the ambient floor by the
            -- model's 3/4 at full gap depth, and the contact shadow where a
            -- cubie's rounding meets the seam
            v_occ := resize(signed('0' & (shift_right(h_ao, 3)
                                          + shift_right(h_ao, 5)
                                          + shift_right(h_ao, 6))), 14);
            -- Contact shadow on the cubie's rounding.  It has to CROSS-FADE
            -- out as the crevice term takes over -- gating it on a threshold
            -- instead leaves a visible step right where the two ramps meet.
            if h_silz = '0' and h_stz = '0' then
                v_t8 := signed('0' & shift_right(h_tmax, 3))
                        - signed('0' & shift_right(h_ao, 3));
                if v_t8 > 0 then v_occ := v_occ + resize(v_t8, 14); end if;
            end if;
            v_li := to_signed(C_AMB, 14) + v_dir - v_occ
                    - resize(signed('0' & v_sed), 14);
            if v_li < 0 then gm_ap <= (others => '0');
            elsif v_li > 255 then gm_ap <= to_unsigned(255, 8);
            else gm_ap <= resize(unsigned(v_li(7 downto 0)), 8); end if;

            -- the highlight is occluded in the crevices too
            v_li := resize(signed('0' & h_ndh), 14)
                    - resize(signed('0' & shift_right(h_ao, 1)), 14);
            if v_li < 0 then sp_a <= (others => '0');
            else sp_a <= resize(unsigned(v_li(7 downto 0)), 8); end if;

            ------------------------------------------------------ stage H
            -- Coverage from distance.  s_pxs is carried as a 3-bit mantissa
            -- and a shift (frame-computed), so the AA slope costs a small
            -- shift-add tree instead of two 16x9 multiplies.  Quantizing the
            -- slope to ~6% only dithers the feather width sub-pixel.
            v_ad := resize(i_dsil - to_signed(C_INSET, 14), 14);
            case to_integer(s_pxm) is
                when 0 => v_am := shift_left(resize(v_ad, 17), 2);
                when 1 => v_am := shift_left(resize(v_ad, 17), 2) + resize(v_ad, 17);
                when 2 => v_am := shift_left(resize(v_ad, 17), 2)
                                  + shift_left(resize(v_ad, 17), 1);
                when others => v_am := shift_left(resize(v_ad, 17), 3)
                                       - resize(v_ad, 17);
            end case;
            v_a := resize(shift_right(shift_right(v_am, 3), s_pxe), 16) + 8;
            if i_on = '0' or v_a < 0 then v_a5 := (others => '0');
            elsif v_a > 16 then v_a5 := to_unsigned(16, 5);
            else v_a5 := resize(unsigned(v_a(4 downto 0)), 5); end if;
            j_asil <= v_a5;
            v_ad := resize(i_ds, 14);
            case to_integer(s_pxm) is
                when 0 => v_am := shift_left(resize(v_ad, 17), 2);
                when 1 => v_am := shift_left(resize(v_ad, 17), 2) + resize(v_ad, 17);
                when 2 => v_am := shift_left(resize(v_ad, 17), 2)
                                  + shift_left(resize(v_ad, 17), 1);
                when others => v_am := shift_left(resize(v_ad, 17), 3)
                                       - resize(v_ad, 17);
            end case;
            v_a := resize(shift_right(shift_right(v_am, 3), s_pxe), 16) + 8;
            if v_a < 0 then v_a5 := (others => '0');
            elsif v_a > 16 then v_a5 := to_unsigned(16, 5);
            else v_a5 := resize(unsigned(v_a(4 downto 0)), 5); end if;
            j_ast <= v_a5;
            j_sl <= i_sl;
            j_on <= i_on;

            ------------------------------------------------------ stage I
            -- LUMA carries all the shading.  CHROMA is a per-face constant
            -- (lit at frame time) hard-switched at the sticker edge: the
            -- encoder subsamples chroma to 4:2:2 on the way out, so per-pixel
            -- chroma blending, lighting and AA would be thrown away downstream.
            -- UNLIT albedo: satin-black plastic blended to the face's sticker
            -- across the sticker's anti-aliased edge.
            v_c := resize((signed(resize(sl_ly(to_integer(j_sl)), 11))
                           - to_signed(C_PLY_I, 11))
                          * signed(resize(j_ast, 6)), 17);
            k_y <= f_clamp10(to_signed(C_PLY_I, 12)
                             + resize(shift_right(v_c, 4), 12));
            k_gam <= gm_d;
            -- specular: plastic sheen blended to the (much glossier) sticker
            -- sheen over the same edge, stored as a video-luma delta
            v_c := resize((signed(resize(sp_d(15 downto 8), 11))
                           - signed(resize(sp_d(7 downto 0), 11)))
                          * signed(resize(j_ast, 6)), 17);
            k_spec <= resize(shift_left(unsigned(resize(
                          signed(resize(sp_d(7 downto 0), 12))
                          + resize(shift_right(v_c, 4), 12), 10)), 2), 12);
            if j_ast >= 8 then k_ston <= '1'; else k_ston <= '0'; end if;
            k_sl2 <= j_sl;
            k_asil <= j_asil;

            ------------------------------------------------------ stage J
            -- albedo * light, then add the highlight (already in video units)
            m_y <= f_clamp10(resize(signed('0' & shift_right(k_y * k_gam, 8)), 14)
                             + resize(signed('0' & k_spec), 14));
            m_ston <= k_ston;
            m_sl2 <= k_sl2;
            m_asil <= k_asil;

            ------------------------------------------------------ stage L
            v_c := resize((signed(resize(m_y, 11)) - signed(resize(bgy_sr(9), 11)))
                          * signed(resize(m_asil, 6)), 17);
            n_y <= f_clamp10(signed(resize(bgy_sr(9), 12))
                             + resize(shift_right(v_c, 4), 12));
            if m_asil < 8 then
                n_u <= bg_u;
                n_v <= bg_v;
            elsif m_ston = '1' then
                n_u <= sl_cu(to_integer(m_sl2));
                n_v <= sl_cv(to_integer(m_sl2));
            else
                n_u <= to_unsigned(512, 10);       -- black plastic: neutral
                n_v <= to_unsigned(512, 10);
            end if;

            ------------------------------------------------------ output
            if s_avid_sr(C_LATENCY - 2) = '1' then
                o_y <= n_y;
                o_u <= n_v;      -- U/V swapped for hardware
                o_v <= n_u;
            else
                o_y <= to_unsigned(64, 10);
                o_u <= to_unsigned(512, 10);
                o_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

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

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(o_u);
    data_out.v       <= std_logic_vector(o_v);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture cubist;
