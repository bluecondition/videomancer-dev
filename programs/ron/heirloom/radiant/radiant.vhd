-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Radiant v0.5.0 -- Heirloom #52 (2026-10-27), rebuilt from duci.jpg (one of Ron's portrait paper collages).
--
-- The collage: on a dark maroon ground of paint and torn mauve tissue, four pink-and-white paper rays (white
-- rims, pink, a dark centre line) burst from behind a dark-red halo disc.  In front: a bust in a blue suit,
-- yellow shirt and tie, black glasses with one lens replaced by the Dominica flag and a white cup ring of
-- coffee sitting on his head; a woman in a cream gown and headscarf (left); a soldier in camouflage and a
-- green beret (right); at the top two hands holding black coal with a cream-on-navy DUCI label.
--
-- The program: the measured cut-outs (64-row slot tables, six interpolated edges and five zone labels a
-- row) print the camera picture through each figure's own colour ramps (16 zone ramps measured on the
-- collage: suit blues, shirt yellows, skin, camo, gown cream, coal ...).  Video = the live camera luma,
-- tapped from the input at the pixel's own position, picks the step of the zone's ramp; Source = Osc
-- uses a generated shade instead (two crossed triangle waves, drifting slowly).  Rays, halo disc, cup,
-- glasses and flag, DUCI label and the tissue ground are drawn: rays as per-line crossings with
-- half-width tests (white / pink / dark centre line), the disc and cup ellipses from a squared-distance
-- DDA against per-line thresholds, the ground from two integer-angle lattices of hashed shards with
-- ragged (noise-jittered) edges.
--
--   K1  Halo     halo disc radius (50 .. 150 %, 100 % = the collage)
--   K2  Lift     the cup rises off the head (0 = sitting on it, as in the collage)
--   K3  Stripe   the rays' stripe mix: white strips -> the collage's white / pink / line -> pink, wide line
--   K4  Tissue   torn tissue on the ground (bare paint .. dense)
--   K5  Flank    the woman and the soldier slide out to the frame edges
--   K6  Pulse    the rays breathe in width (depth and rate; 0 = still)
--   S7  Lens     Flag / Clear (the left lens shows the eye)
--   S8  Cup      On / Off (the bare head)
--   S9  Halo     Dark / Gold
--   S10 Label    DUCI / Coal (no label)
--   S11 Source   Video / Osc
--   P12 RADIANT  ray width: 0 = no rays (bare halo), 50 % = the collage's four strips, 100 % = 2.2x
--
-- Geometry in virtual 1080-line units (timing-ID steps; h = half virtual px).  Engines: the per-field
-- register machine (triplicate's: register file in one EBR, microcode in block RAM, one serial shift-add
-- multiplier (signed A x unsigned B), add and max-0 post-ops, two loops) computes the page origin,
-- figure positions, halo / cup thresholds and per-line steps, the ray half widths and crossings, the pulse
-- and the glides; a per-line engine in hblank steps the crossings / ellipse offsets / rows, reads and
-- interpolates the slot tables and squares the line offsets.  Generated avid; output conditioning
-- (luma [1 2 1] and to ~777, chroma [1 2 2 2 2 2 2 2 1]/16) before the blanking gate.  Latency 16, all
-- modes.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.radiant_pkg.all;
use work.all;

architecture radiant of program_top is
    constant C_LAT : integer := 16;

    ---------------------------------------------------------------------------
    -- mode tables (timing ID)
    ---------------------------------------------------------------------------
    function f_xs(t : std_logic_vector(3 downto 0)) return unsigned is      -- half virtual px per pixel
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    function f_ys(t : std_logic_vector(3 downto 0)) return unsigned is      -- virtual lines per line, Q.8
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 11);
            when 4          => return to_unsigned(576, 11);
            when 8          => return to_unsigned(960, 11);
            when 12         => return to_unsigned(480, 11);
            when 5 | 6 | 14 => return to_unsigned(384, 11);
            when 1 | 2 | 10 => return to_unsigned(512, 11);
            when others     => return to_unsigned(256, 11);
        end case;
    end function;

    ---------------------------------------------------------------------------
    -- register machine.  res = post((+/-)((A * B) >>> sh) + K) ; load live(a).
    -- Address modifiers OR the loop index (bits 2:0) or the Osc bit (bit 2).
    ---------------------------------------------------------------------------
    type t_uc is record
        op : integer range 0 to 2;
        a, b, k, d : integer range 0 to 255;
        am, bm, km, dm, asrc, bsrc : std_logic;
        sh : integer range 0 to 15;
        ng, bs, wide, wr, dn : std_logic;
        post : integer range 0 to 5;
        o : integer range 0 to 31;
    end record;
    function uc(op, a, b, sh, k, d : integer; wr, ng, bs, wide : std_logic; post : integer; o : integer;
                am, bm, km, dm, asrc, bsrc, dn : std_logic) return t_uc is
        variable r : t_uc;
    begin
        r.op := op; r.a := a; r.b := b; r.sh := sh; r.k := k; r.d := d; r.wr := wr; r.ng := ng; r.bs := bs;
        r.wide := wide; r.post := post; r.o := o; r.am := am; r.bm := bm; r.km := km; r.dm := dm;
        r.asrc := asrc; r.bsrc := bsrc; r.dn := dn;
        return r;
    end function;
    -- live sources
    constant L_T12 : integer := 0;
    constant L_K1 : integer := 1;
    constant L_K2 : integer := 2;
    constant L_K3 : integer := 3;
    constant L_K4 : integer := 4;
    constant L_K5 : integer := 5;
    constant L_K6 : integer := 6;
    constant L_W : integer := 7;
    constant L_XS : integer := 8;
    constant L_YS : integer := 9;
    constant L_XSQ : integer := 10;
    constant L_XS2 : integer := 11;
    -- outputs
    constant O_NONE : integer := 0;
    constant O_X0 : integer := 1;
    constant O_CX2 : integer := 2;
    constant O_XF : integer := 3;
    constant O_SU0 : integer := 4;
    constant O_KSU : integer := 5;
    constant O_KV : integer := 6;
    constant O_TR : integer := 7;
    constant O_TRG : integer := 8;
    constant O_A0 : integer := 9;
    constant O_D0 : integer := 10;
    constant O_HWW : integer := 11;
    constant O_HWP : integer := 12;
    constant O_HWD : integer := 13;
    constant O_LRS : integer := 14;
    constant O_LNS : integer := 15;
    constant O_ARS : integer := 16;
    constant O_KLU : integer := 17;
    constant O_LX0 : integer := 18;
    constant O_GTH : integer := 19;
    constant ST_FIG : integer := 25; constant ST_FIGEND : integer := 26;
    constant ST_RAY : integer := 59; constant ST_RAYEND : integer := 64;
    constant RF_XRAL : integer := 40;
    constant RF_ES0L : integer := 56;
    constant RF_DXL : integer := 16;
    constant RF_EST : integer := 92;
    constant RF_KV : integer := 80;
    constant RF_LRS : integer := 84;
    constant RF_LNS : integer := 85;
    constant RV0_0 : integer := 0;
    constant RV0_1 : integer := -791031;
    constant RV0_2 : integer := -596249;
    constant RV0_3 : integer := -1218615;
    type t_mc is array (0 to 255) of std_logic_vector(63 downto 0);
    constant C_MC : t_mc := (
        0 => x"00001001857C0002",
        1 => x"00001001897C0006",
        2 => x"000010018D7C000A",
        3 => x"00001001917C000E",
        4 => x"00001001957C0012",
        5 => x"00001001997C0016",
        6 => x"000010019D7C001A",
        7 => x"00001001A17C001E",
        8 => x"00001001A57C0022",
        9 => x"00001001A97C0026",
        10 => x"00001001AD7C002A",
        11 => x"00001001B17C002E",
        12 => x"00001401BD8581B4",
        13 => x"000010C1B5B581BC",
        14 => x"00011041C20DA5A0",
        15 => x"00021001C61181C0",
        16 => x"00120000021581C0",
        17 => x"00005001CA1981C0",
        18 => x"00001281CD7D99C8",
        19 => x"00001001321D81C0",
        20 => x"00001001362181C0",
        21 => x"00001001D22581C0",
        22 => x"0000140139D181CC",
        23 => x"00001001D22981C0",
        24 => x"000010013DD181CC",
        25 => x"00030004017D8130",
        26 => x"00001225417DA920",
        27 => x"00001281D6322D88",
        28 => x"00070380017DD5D4",
        29 => x"00001001DA3581D4",
        30 => x"00080380017DD9D8",
        31 => x"00001281DD7E398C",
        32 => x"00001401D1DD8164",
        33 => x"00001800E17D59D0",
        34 => x"00001201717DA958",
        35 => x"00001401D1DD8168",
        36 => x"00001800E57D5DD0",
        37 => x"00001201757DA95C",
        38 => x"00001401D1DD816C",
        39 => x"00001800E97D61D0",
        40 => x"00001201797DA960",
        41 => x"00090000017DC5C4",
        42 => x"000A040001ADB1C4",
        43 => x"00001281517E3DA8",
        44 => x"00001281557E41A8",
        45 => x"00110000017E45A4",
        46 => x"00130280017E4994",
        47 => x"00001001E17E4DB4",
        48 => x"00005001E65181B4",
        49 => x"00001301E9E255E4",
        50 => x"00001281F25E599C",
        51 => x"00001001B9B981F0",
        52 => x"00005401E17D81B8",
        53 => x"00001001F5BA4DE0",
        54 => x"00001001F66181F4",
        55 => x"00001341FA659DF4",
        56 => x"00001301ED7DF9E8",
        57 => x"00001281FE6E6990",
        58 => x"0000128202767190",
        59 => x"00001005D1C67880",
        60 => x"00001830A06279D0",
        61 => x"00001324417DA820",
        62 => x"000B1386057DEC00",
        63 => x"000C0280017DFE04",
        64 => x"000D0280017E0204",
        others => (others => '0')
    );
    type t_rf is array (0 to 255) of std_logic_vector(15 downto 0);
    function f_rf_init return t_rf is
        variable r : t_rf := (others => (others => '0'));
    begin
        r(0) := std_logic_vector(to_signed(245, 16));
        r(1) := std_logic_vector(to_signed(241, 16));
        r(2) := std_logic_vector(to_signed(334, 16));
        r(3) := std_logic_vector(to_signed(301, 16));
        r(8) := std_logic_vector(to_signed(3334, 16));
        r(9) := std_logic_vector(to_signed(-2938, 16));
        r(10) := std_logic_vector(to_signed(8872, 16));
        r(11) := std_logic_vector(to_signed(-7155, 16));
        r(24) := std_logic_vector(to_signed(20445, 16));
        r(25) := std_logic_vector(to_signed(-14121, 16));
        r(26) := std_logic_vector(to_signed(7067, 16));
        r(27) := std_logic_vector(to_signed(-19923, 16));
        r(32) := std_logic_vector(to_signed(-2, 16));
        r(33) := std_logic_vector(to_signed(2, 16));
        r(34) := std_logic_vector(to_signed(-5, 16));
        r(35) := std_logic_vector(to_signed(5, 16));
        r(72) := std_logic_vector(to_signed(3107, 16));
        r(73) := std_logic_vector(to_signed(1703, 16));
        r(74) := std_logic_vector(to_signed(1523, 16));
        r(75) := std_logic_vector(to_signed(2099, 16));
        r(86) := std_logic_vector(to_signed(472, 16));
        r(87) := std_logic_vector(to_signed(638, 16));
        r(88) := std_logic_vector(to_signed(568, 16));
        r(89) := std_logic_vector(to_signed(472, 16));
        r(90) := std_logic_vector(to_signed(513, 16));
        r(91) := std_logic_vector(to_signed(473, 16));
        r(96) := std_logic_vector(to_signed(1, 16));
        r(109) := std_logic_vector(to_signed(2048, 16));
        r(131) := std_logic_vector(to_signed(-827, 16));
        r(132) := std_logic_vector(to_signed(859, 16));
        r(133) := std_logic_vector(to_signed(719, 16));
        r(134) := std_logic_vector(to_signed(-40, 16));
        r(135) := std_logic_vector(to_signed(429, 16));
        r(136) := std_logic_vector(to_signed(383, 16));
        r(137) := std_logic_vector(to_signed(-86, 16));
        r(138) := std_logic_vector(to_signed(1354, 16));
        r(139) := std_logic_vector(to_signed(4112, 16));
        r(140) := std_logic_vector(to_signed(2056, 16));
        r(141) := std_logic_vector(to_signed(-48, 16));
        r(142) := std_logic_vector(to_signed(200, 16));
        r(143) := std_logic_vector(to_signed(287, 16));
        r(144) := std_logic_vector(to_signed(411, 16));
        r(145) := std_logic_vector(to_signed(209, 16));
        r(146) := std_logic_vector(to_signed(77, 16));
        r(147) := std_logic_vector(to_signed(2, 16));
        r(148) := std_logic_vector(to_signed(-2048, 16));
        r(149) := std_logic_vector(to_signed(1638, 16));
        r(150) := std_logic_vector(to_signed(375, 16));
        r(151) := std_logic_vector(to_signed(24, 16));
        r(152) := std_logic_vector(to_signed(-16384, 16));
        r(153) := std_logic_vector(to_signed(4096, 16));
        r(154) := std_logic_vector(to_signed(696, 16));
        r(155) := std_logic_vector(to_signed(100, 16));
        r(156) := std_logic_vector(to_signed(174, 16));
        r(157) := std_logic_vector(to_signed(20, 16));
        r(158) := std_logic_vector(to_signed(256, 16));
        return r;
    end function;
    function f_am(base : integer; m, s : std_logic; ci : unsigned(2 downto 0); osc : std_logic) return unsigned is
        variable v : unsigned(7 downto 0);
    begin
        v := to_unsigned(base, 8);
        if m = '1' then v(2 downto 0) := v(2 downto 0) or ci; end if;
        if s = '1' and osc = '1' then v(2) := '1'; end if;
        return v;
    end function;

    ---------------------------------------------------------------------------
    -- small helpers
    ---------------------------------------------------------------------------
    function f_tri6(x : unsigned(5 downto 0)) return unsigned is            -- 0..31..0
    begin
        if x(5) = '1' then return not x(4 downto 0); else return x(4 downto 0); end if;
    end function;
    -- cheaper cell hash for the ground (two add / xor rounds)
    function f_h4(a, b : unsigned(7 downto 0); s : unsigned(7 downto 0)) return unsigned is
        variable x : unsigned(7 downto 0);
    begin
        x := (a xor rotate_left(b, 4)) + (b(6 downto 0) & '0') + s;
        x := (x xor rotate_left(x, 4)) + (x(6 downto 0) & '0');
        return x;
    end function;

    ---------------------------------------------------------------------------
    -- colours (hardware convention: u = Cr, v = Cb)
    ---------------------------------------------------------------------------
    type t_c3 is array (0 to 2) of integer;
    constant K_WHITE  : t_c3 := (886, 527, 514);  constant K_PINK   : t_c3 := (606, 733, 520);
    constant K_RLINE  : t_c3 := (270, 602, 508);  constant K_HALO   : t_c3 := (192, 572, 482);
    constant K_RING   : t_c3 := (137, 534, 501);  constant K_GOLD   : t_c3 := (519, 621, 329);
    constant K_RIM    : t_c3 := (843, 510, 523);  constant K_WALL   : t_c3 := (740, 510, 526);
    constant K_FRAME  : t_c3 := (155, 516, 511);  constant K_EYE    : t_c3 := (185, 524, 501);
    constant K_GREEN  : t_c3 := (334, 365, 475);  constant K_YELLOW : t_c3 := (719, 630, 213);
    constant K_BLACK  : t_c3 := (133, 512, 512);  constant K_FWHITE : t_c3 := (888, 512, 512);
    constant K_RED    : t_c3 := (421, 818, 429);  constant K_PURPLE : t_c3 := (425, 524, 641);
    constant K_NAVY   : t_c3 := (318, 440, 568);  constant K_CREAM  : t_c3 := (757, 545, 451);
    type t_i12 is array (0 to 11) of integer range 0 to 1023;
    constant K_CFY : t_i12 := (681, 645, 608, 570, 530, 483, 437, 385, 342, 305, 279, 257);
    constant K_CFU : t_i12 := (576, 579, 581, 584, 583, 582, 581, 579, 575, 572, 566, 561);
    constant K_CFV : t_i12 := (394, 395, 394, 395, 402, 409, 417, 430, 441, 450, 457, 464);
    type t_i32 is array (0 to 31) of integer range 0 to 1023;
    -- overlay colours: 0..3 ground, 4 white, 5 pink, 6 line, 7 halo, 8 ring, 9 gold, 10 rim, 11 wall,
    -- 12 coffee (ramp), 13 frame, 14 eye, 16 flag green, 17 yellow, 18 black, 19 white, 20 red
    constant K_OY : t_i32 := (207, 229, 360, 518, 886, 606, 270, 192, 137, 519, 843, 740, 0, 155, 185, 0,
                              334, 719, 133, 888, 421, others => 64);
    constant K_OU : t_i32 := (548, 572, 576, 582, 527, 733, 602, 572, 534, 621, 510, 510, 512, 516, 524, 512,
                              365, 630, 512, 512, 818, others => 512);
    constant K_OV : t_i32 := (497, 488, 504, 492, 514, 520, 508, 482, 501, 329, 523, 526, 512, 511, 501, 512,
                              475, 213, 512, 512, 429, others => 512);
    type t_i4 is array (0 to 3) of integer range 0 to 1023;
    type t_ch is array (0 to 31) of integer range 0 to 127;
    -- red-disc half chords (h) by |dy| in steps of 1.25 v px
    constant K_CHORD : t_ch := (50, 49, 49, 49, 48, 48, 47, 46, 45, 44, 42, 41, 40, 38, 34, 32, 30, 27, 19, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    constant K_GNY : t_i4 := (207, 229, 360, 518);
    constant K_GNU : t_i4 := (548, 572, 576, 582);
    constant K_GNV : t_i4 := (497, 488, 504, 492);

    ---------------------------------------------------------------------------
    -- memories
    ---------------------------------------------------------------------------
    attribute no_rw_check : boolean;
    signal rf : t_rf := f_rf_init;
    attribute no_rw_check of rf : signal is true;
    signal t_ra : unsigned(9 downto 0) := (others => '0');
    signal t_q, t_qq : std_logic_vector(15 downto 0) := (others => '0');
    signal pa : unsigned(7 downto 0) := (others => '0');
    signal py_q, pc_q, py_qq, pc_qq : std_logic_vector(15 downto 0) := (others => '0');
    signal rf_ra, q_ra, l_ra, rf_wa : unsigned(7 downto 0) := (others => '0');
    signal rf_we : std_logic := '0';
    signal rf_wd, rf_q, rf_qq : std_logic_vector(15 downto 0) := (others => '0');
    type t_dly is array (0 to 255) of std_logic_vector(3 downto 0);
    signal s_dly : t_dly := (others => "1110");
    attribute no_rw_check of s_dly : signal is true;
    signal s_dwp, s_drp : unsigned(7 downto 0) := (others => '0');
    type t_cring is array (0 to 255) of std_logic_vector(15 downto 0);
    signal c_ring : t_cring := (others => x"8080");
    attribute no_rw_check of c_ring : signal is true;
    signal c_ra : unsigned(7 downto 0) := (others => '0');
    signal c_q : std_logic_vector(15 downto 0) := x"8080";
    signal s_dq, s_dg, s_do : std_logic_vector(3 downto 0) := "1110";

    ---------------------------------------------------------------------------
    -- timing, controls
    ---------------------------------------------------------------------------
    signal s_tid : std_logic_vector(3 downto 0) := (others => '0');
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal s_dd : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_idle : unsigned(10 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w : unsigned(10 downto 0) := to_unsigned(1920, 11);
    signal s_flag, s_cup, s_gold, s_lab, s_osc : std_logic := '0';
    signal s_fc : unsigned(7 downto 0) := (others => '0');                    -- field counter

    ---------------------------------------------------------------------------
    -- register machine
    ---------------------------------------------------------------------------
    signal q_run, q_done : std_logic := '0';
    signal q_st : unsigned(7 downto 0) := (others => '0');
    signal q_ci : unsigned(2 downto 0) := (others => '0');
    signal q_cnt : unsigned(5 downto 0) := (others => '0');
    signal q_uc : t_uc := uc(2, 0, 0, 0, 0, 0, '0', '0', '0', '0', 0, 0, '0', '0', '0', '0', '0', '0', '0');
    signal mc_q : std_logic_vector(63 downto 0) := (others => '0');
    signal m_a, m_k : signed(15 downto 0) := (others => '0');
    signal m_p : unsigned(36 downto 0) := (others => '0');
    signal m_res : signed(27 downto 0) := (others => '0');
    signal m_ld : std_logic_vector(15 downto 0) := (others => '0');
    signal s_xsq : unsigned(5 downto 0) := to_unsigned(4, 6);
    signal m_wr, m_wr2 : std_logic := '0';
    -- per-field outputs
    signal r_x0, r_cx2, r_lx0 : signed(13 downto 0) := (others => '0');
    type t_s14x4 is array (0 to 3) of signed(13 downto 0);
    signal r_xf : t_s14x4 := (others => (others => '0'));
    signal r_tr, r_trg : unsigned(11 downto 0) := (others => '0');
    signal r_a0 : unsigned(23 downto 0) := (others => '0');
    signal r_d0 : signed(17 downto 0) := (others => '0');
    type t_u11x4 is array (0 to 3) of unsigned(10 downto 0);
    type t_u10x4 is array (0 to 3) of unsigned(9 downto 0);
    type t_u8x4 is array (0 to 3) of unsigned(7 downto 0);
    signal r_hww : t_u11x4 := (others => (others => '0'));
    signal r_hwp : t_u10x4 := (others => (others => '0'));
    signal r_hwd : t_u8x4 := (others => (others => '0'));
    signal r_ars : unsigned(14 downto 0) := (others => '0');
    signal r_klu : unsigned(10 downto 0) := (others => '0');
    signal r_gth : unsigned(7 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- line engine
    ---------------------------------------------------------------------------
    signal l_run, l_init, l_fkick, l_kickp : std_logic := '0';
    signal l_dly : unsigned(3 downto 0) := (others => '0');
    signal l_t : unsigned(6 downto 0) := (others => '0');
    signal l_vy : unsigned(18 downto 0) := (others => '0');
    signal l_lr, l_ln : signed(19 downto 0) := (others => '0');
    signal l_ar : signed(23 downto 0) := (others => '0');
    type t_s24x4 is array (0 to 3) of signed(23 downto 0);
    signal l_xr : t_s24x4 := (others => (others => '0'));
    type t_s24x3 is array (0 to 2) of signed(23 downto 0);
    signal l_ey : t_s24x3 := (others => (others => '0'));
    type t_s22x4 is array (0 to 3) of signed(21 downto 0);
    signal l_rv : t_s22x4 := (others => (others => '0'));
    signal l_lo : std_logic_vector(15 downto 0) := (others => '0');
    -- figure part
    type t_u13x18 is array (0 to 17) of unsigned(11 downto 0);
    signal l_e : t_u13x18 := (others => (others => '1'));
    type t_l15x3 is array (0 to 2) of std_logic_vector(14 downto 0);
    signal l_lab : t_l15x3 := (others => (others => '1'));
    signal l_val : std_logic_vector(2 downto 0) := (others => '0');
    signal l_cfig : std_logic := '0';                                   -- centre slot: 0 hands, 1 bust
    signal l_fs : unsigned(1 downto 0) := (others => '0');
    signal l_tj : unsigned(2 downto 0) := (others => '0');
    signal l_tv : std_logic := '0';
    type t_tag is record
        v : std_logic;
        s, f : unsigned(1 downto 0);
        j : unsigned(2 downto 0);
    end record;
    constant C_TAG0 : t_tag := ('0', "00", "00", "000");
    signal l_tgC, l_tgD, l_tgE : t_tag := C_TAG0;
    signal l_w0 : std_logic_vector(15 downto 0) := (others => '0');
    signal l_lp : std_logic := '0';
    signal l_wfr : unsigned(3 downto 0) := (others => '0');
    signal l_d0, l_d1 : signed(8 downto 0) := (others => '0');
    signal l_b0, l_b1 : unsigned(7 downto 0) := (others => '0');
    signal l_wv, l_wv2 : std_logic := '0';
    signal l_d1q : signed(8 downto 0) := (others => '0');
    signal l_xw : signed(13 downto 0) := (others => '0');
    signal l_fw : unsigned(3 downto 0) := (others => '0');
    signal l_b1q : unsigned(7 downto 0) := (others => '0');
    signal l_wp : unsigned(3 downto 0) := (others => '0');
    signal l_xfq : signed(13 downto 0) := (others => '0');
    -- derived per-line values
    signal l_up : std_logic := '0';
    signal l_m1, l_m2, l_m3 : signed(13 downto 0) := (others => '0');          -- absolute midpoints (h)
    signal l_hin : std_logic := '0';                                            -- hands slot active (row in range)
    signal l_hdy : signed(19 downto 0) := (others => '0');
    signal l_sxa : unsigned(15 downto 0) := (others => '0');
    signal l_sqb : unsigned(10 downto 0) := (others => '0');
    signal l_fd1 : signed(19 downto 0) := (others => '0');
    signal l_pv1, l_pv2 : std_logic := '0';
    signal l_pk1, l_pk2 : integer range 0 to 17 := 0;
    signal l_pbs1, l_pbs2 : signed(14 downto 0) := (others => '0');
    signal l_ppa1, l_ppb1 : signed(10 downto 0) := (others => '0');
    signal l_pp2 : signed(13 downto 0) := (others => '0');
    signal l_sqi : unsigned(10 downto 0) := (others => '0');
    signal l_sqo : unsigned(15 downto 0) := (others => '0');
    signal l_sqa : unsigned(21 downto 0) := (others => '0');
    signal l_sk : unsigned(3 downto 0) := (others => '0');
    signal l_sq, l_sqn : unsigned(1 downto 0) := (others => '0');
    signal l_sqv : std_logic := '0';
    signal l_tr, l_trg, l_te1, l_te2, l_te3 : signed(12 downto 0) := (others => '0');
    signal l_wall, l_lens, l_brg, l_end, l_arm, l_labr, l_frow : std_logic := '0';
    signal l_hlo, l_hli, l_hro, l_hri : unsigned(6 downto 0) := (others => '0');
    signal ln_ra : unsigned(7 downto 0) := (others => '0');
    signal ln_q : std_logic_vector(15 downto 0) := (others => '0');
    signal ova : unsigned(7 downto 0) := (others => '0');
    signal ov_qh, ov_ql, ov_hh, ov_ll : std_logic_vector(15 downto 0) := (others => '0');
    signal l_axa, l_axb : unsigned(9 downto 0) := (others => '0');
    signal l_lrow : unsigned(4 downto 0) := (others => '0');
    signal l_fdy : unsigned(4 downto 0) := (others => '0');                 -- |dy| from the flag disc centre, 1.25 v px
    signal l_fey : std_logic := '0';                                         -- |dy| < 5 v px (eye rows)
    signal l_fch : unsigned(6 downto 0) := (others => '0');
    signal l_fhs : unsigned(1 downto 0) := (others => '0');
    signal l_cy : signed(5 downto 0) := (others => '0');
    signal l_vy2 : unsigned(11 downto 0) := (others => '0');
    signal l_line : unsigned(9 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- generated avid
    ---------------------------------------------------------------------------
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_avd : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    ---------------------------------------------------------------------------
    -- pixel path
    ---------------------------------------------------------------------------
    signal p_vx : unsigned(12 downto 0) := (others => '0');
    signal p_av : unsigned(23 downto 0) := (others => '0');
    signal p_dv : signed(17 downto 0) := (others => '0');
    signal p_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal p_nm : unsigned(11 downto 0) := to_unsigned(2048, 12);
    signal p_lu : unsigned(9 downto 0) := (others => '0');
    signal p_lb : unsigned(2 downto 0) := (others => '0');                   -- label bit 0..5
    signal p_lg : unsigned(3 downto 0) := (others => '0');                   -- label group 0..9
    signal p_lin : std_logic := '0';
    -- S2
    signal p_vxc2, p_vr2 : signed(13 downto 0) := (others => '0');
    signal p_ge2 : std_logic_vector(17 downto 0) := (others => '0');
    signal p_aq2 : unsigned(11 downto 0) := (others => '0');
    signal p_wob2 : signed(5 downto 0) := (others => '0');
    signal p_vx2 : unsigned(12 downto 0) := (others => '0');
    signal p_lab2 : std_logic_vector(7 downto 0) := (others => '0');          -- {in box, in bitmap, bit(3), group?}
    signal p_lg2 : unsigned(3 downto 0) := (others => '0');
    signal p_lb2 : unsigned(2 downto 0) := (others => '0');
    -- S3
    signal p_ri2 : unsigned(1 downto 0) := (others => '0');
    signal p_ra3 : unsigned(12 downto 0) := (others => '0');
    signal p_twd3 : unsigned(7 downto 0) := (others => '0');
    signal p_twp3 : unsigned(9 downto 0) := (others => '0');
    signal p_tww3 : unsigned(10 downto 0) := (others => '0');
    signal p_gfx3 : std_logic := '0';
    signal p_gst3 : unsigned(1 downto 0) := (others => '0');
    signal p_gc : unsigned(7 downto 0) := (others => '0');
    signal p_gx3, p_gx4 : unsigned(7 downto 0) := (others => '0');
    signal p_ma4, p_mb4 : unsigned(4 downto 0) := (others => '0');
    signal p_disc3, p_rin3, p_e13, p_e23, p_e33 : std_logic := '0';
    type t_u3x3 is array (0 to 2) of unsigned(2 downto 0);
    signal p_sl3 : t_u3x3 := (others => (others => '1'));       -- slot labels (7 = outside)
    signal p_u13, p_v13, p_u23, p_v23 : unsigned(7 downto 0) := (others => '0');
    signal p_cx3 : signed(7 downto 0) := (others => '0');
    signal p_wsh3 : unsigned(3 downto 0) := (others => '0');
    signal p_nm3 : unsigned(7 downto 0) := (others => '0');
    signal p_lab3 : std_logic_vector(1 downto 0) := (others => '0');
    signal p_ldu3 : unsigned(7 downto 0) := (others => '0');
    signal p_lb3 : unsigned(2 downto 0) := (others => '0');
    -- S4
    signal p_ray4 : unsigned(1 downto 0) := (others => '0');
    signal p_halo4 : unsigned(1 downto 0) := (others => '0');
    signal p_bowl4 : unsigned(1 downto 0) := (others => '0');
    signal p_ci4 : unsigned(3 downto 0) := (others => '0');
    signal p_wsh4 : unsigned(3 downto 0) := (others => '0');
    signal p_gl4 : unsigned(3 downto 0) := (others => '0');                  -- 0 none, 1 frame, 2 eye, 8+ flag colour
    signal p_win4 : unsigned(1 downto 0) := (others => '0');                 -- 0 none, 1 side, 2 centre
    signal p_zone4 : unsigned(3 downto 0) := (others => '0');
    signal p_ha4, p_hb4 : unsigned(7 downto 0) := (others => '0');
    signal p_lab4 : std_logic_vector(1 downto 0) := (others => '0');
    signal p_ldu4 : unsigned(7 downto 0) := (others => '0');
    signal p_lb4 : unsigned(2 downto 0) := (others => '0');
    signal g_st1, g_st2 : std_logic := '0';
    signal p_gv3, p_gw3 : std_logic := '0';
    signal p_dl3, p_dr3 : unsigned(9 downto 0) := (others => '0');
    type t_ba is record
        win : unsigned(1 downto 0);
        zone : unsigned(3 downto 0);
        lab : std_logic_vector(1 downto 0);
        ldu : unsigned(7 downto 0);
        lb : unsigned(2 downto 0);
        oc : unsigned(4 downto 0);
        op : unsigned(6 downto 0);
    end record;

    signal p_zone5 : unsigned(3 downto 0) := (others => '0');
    signal p_ldu5 : unsigned(7 downto 0) := (others => '0');
    signal p_l5 : unsigned(7 downto 0) := (others => '0');
    signal p_gr5 : signed(7 downto 0) := (others => '0');
    signal p_y9, p_u9, p_v9 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p_pa3, p_pb3 : unsigned(5 downto 0) := (others => '0');
    signal p_pl4 : unsigned(7 downto 0) := (others => '0');
    type t_bb is record
        win : unsigned(1 downto 0);
        lab : std_logic_vector(1 downto 0);
        lb : unsigned(2 downto 0);
        hi : std_logic;
    end record;
    type t_bbv is array (5 to 8) of t_bb;
    signal p_b : t_bbv := (others => ("00", "00", "000", '0'));
    signal p_oa5 : unsigned(7 downto 0) := (others => '0');
    signal p_y14, p_u14, p_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- conditioning

    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_y15, s_y16, s_y17, s_y18, s_y19, s_y20x, s_yf20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u20x, s_v20x : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_tab : process(clk)
    begin
        if rising_edge(clk) then
            t_q <= C_TAB(to_integer(t_ra));
            t_qq <= t_q;
        end if;
    end process;
    p_lens : process(clk)
    begin
        if rising_edge(clk) then
            ln_q <= C_LENS(to_integer(ln_ra));
        end if;
    end process p_lens;
    p_ovr : process(clk)
    begin
        if rising_edge(clk) then
            ov_qh <= C_OVH(to_integer(ova)); ov_ql <= C_OVL(to_integer(ova));
            ov_hh <= ov_qh; ov_ll <= ov_ql;
        end if;
    end process p_ovr;
    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            py_q <= C_PALY(to_integer(pa));
            pc_q <= C_PALC(to_integer(pa));
            py_qq <= py_q; pc_qq <= pc_q;
        end if;
    end process;
    p_cring : process(clk)
    begin
        if rising_edge(clk) then
            c_q <= c_ring(to_integer(c_ra));
            c_ring(to_integer(s_dwp)) <= std_logic_vector(p_u14(9 downto 2)) & std_logic_vector(p_v14(9 downto 2));
            c_ra <= s_dwp - 6;
        end if;
    end process p_cring;
    p_sdly : process(clk)
    begin
        if rising_edge(clk) then
            s_dq <= s_dly(to_integer(s_drp));
            s_dly(to_integer(s_dwp)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n & g_avid;
        end if;
    end process;
    -- register file: the machine owns it in vblank, the line engine in hblank
    rf_ra <= q_ra when q_run = '1' else l_ra;
    p_rf : process(clk)
    begin
        if rising_edge(clk) then
            rf_q <= rf(to_integer(rf_ra));
            if rf_we = '1' then rf(to_integer(rf_wa)) <= rf_wd; end if;
            rf_qq <= rf_q;
        end if;
    end process p_rf;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, width, sync delays
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_xs <= f_xs(s_tid);
            s_xsq <= f_xs(s_tid) * f_xs(s_tid);
            s_ys <= f_ys(s_tid);
            case to_integer(s_xs) is                                          -- dx^2 DDA step (xs^2 h^2)
                when 4 =>      s_dd <= to_unsigned(32, 6);
                when 3 =>      s_dd <= to_unsigned(18, 6);
                when others => s_dd <= to_unsigned(8, 6);
            end case;

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle(10) = '0' then
                s_idle <= s_idle + 1;
            end if;
            s_zone <= s_idle(10);
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            if s_zgo = '1' then
                s_flag <= not registers_in(6)(0);
                s_cup  <= not registers_in(6)(1);
                s_gold <= registers_in(6)(2);
                s_lab  <= not registers_in(6)(3);
                s_osc  <= registers_in(6)(4);
                if s_wmax /= 0 then s_w <= s_wmax(10 downto 0); end if;
                s_wmax <= (others => '0');
                s_fc <= s_fc + 1;
            end if;
            g_wm1 <= resize(s_w, 12) - 1;

            -- delay line: written each clock, read 12 entries back (s_dq = input delayed 14),
            -- then two registers (delay 15, 16 = C_LAT)
            s_dwp <= s_dwp + 1;
            s_drp <= s_dwp - 12;
            s_dg <= s_dq; s_do <= s_dg;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Register machine (vblank).  Load steps: 7 clocks.  Arithmetic steps: 48 clocks:
    -- cnt 3 microcode word, 4..6 operand addresses (A, B, K; data 3 clocks later), 9..24 multiply,
    -- 25..39 arithmetic shifts, 41 sign + add (K + 1 at 10 when negating), 42 post, 43 write (44 high word).
    ---------------------------------------------------------------------------
    p_mc : process(clk)
    begin
        if rising_edge(clk) then
            mc_q <= C_MC(to_integer(q_st));
        end if;
    end process p_mc;
    p_seq : process(clk)
        variable v_l : signed(15 downto 0);
        variable v_hi : signed(20 downto 0);
        variable v_end : std_logic;
        variable v_d : unsigned(7 downto 0);
        variable v_ci : integer range 0 to 7;
        variable w : std_logic_vector(63 downto 0);
    begin
        if rising_edge(clk) then
            q_done <= '0'; rf_we <= '0'; m_wr <= '0'; m_wr2 <= m_wr and q_uc.wide;
            if s_zgo = '1' then
                q_run <= '1'; q_st <= (others => '0'); q_ci <= (others => '0'); q_cnt <= (others => '0');
            elsif q_run = '1' then
                q_cnt <= q_cnt + 1;
                v_end := '0';
                if q_cnt = 3 then
                    -- microcode word (ROM in block RAM, read 2 clocks after the state changes)
                    w := mc_q;
                    q_uc.op <= to_integer(unsigned(w(1 downto 0)));
                    q_uc.a <= to_integer(unsigned(w(9 downto 2))); q_uc.b <= to_integer(unsigned(w(17 downto 10)));
                    q_uc.k <= to_integer(unsigned(w(25 downto 18))); q_uc.d <= to_integer(unsigned(w(33 downto 26)));
                    q_uc.am <= w(34); q_uc.bm <= w(35); q_uc.km <= w(36); q_uc.dm <= w(37);
                    q_uc.sh <= to_integer(unsigned(w(41 downto 38)));
                    q_uc.ng <= w(42); q_uc.wide <= w(43); q_uc.wr <= w(44);
                    q_uc.post <= to_integer(unsigned(w(47 downto 45)));
                    q_uc.o <= to_integer(unsigned(w(52 downto 48)));
                elsif q_cnt < 3 then
                    null;
                elsif q_uc.op = 2 then
                    if q_cnt = 4 then
                        case q_uc.a is
                            when L_T12 => v_l := signed(resize(unsigned(registers_in(7)) & "00", 16));
                            when L_K1  => v_l := signed(resize(unsigned(registers_in(0)), 16));
                            when L_K2  => v_l := signed(resize(unsigned(registers_in(1)), 16));
                            when L_K3  => v_l := signed(resize(unsigned(registers_in(2)), 16));
                            when L_K4  => v_l := signed(resize(unsigned(registers_in(3)), 16));
                            when L_K5  => v_l := signed(resize(unsigned(registers_in(4)), 16));
                            when L_K6  => v_l := signed(resize(unsigned(registers_in(5)), 16));
                            when L_W   => v_l := signed(resize(s_w, 16));
                            when L_XS  => v_l := signed(resize(s_xs, 16));
                            when L_YS  => v_l := signed(resize(s_ys, 16));
                            when L_XSQ => v_l := signed(resize(s_xsq, 16));
                            when others => v_l := signed(resize(s_xs & '0', 16));
                        end case;
                        m_ld <= std_logic_vector(v_l);
                    elsif q_cnt = 5 then
                        rf_we <= '1'; rf_wa <= to_unsigned(q_uc.d, 8); rf_wd <= m_ld;
                    elsif q_cnt = 6 then
                        v_end := '1';
                    end if;
                else
                    -- operand addresses at 4, 5, 6; data 3 clocks later (A at 7, B at 8, K at 9)
                    if q_cnt = 4 then
                        q_ra <= f_am(q_uc.a, q_uc.am, '0', q_ci, '0');
                    elsif q_cnt = 5 then
                        q_ra <= f_am(q_uc.b, q_uc.bm, '0', q_ci, '0');
                    elsif q_cnt = 6 then
                        q_ra <= f_am(q_uc.k, q_uc.km, '0', q_ci, '0');
                    end if;
                    if q_cnt = 7 then
                        m_a <= signed(rf_qq);
                    elsif q_cnt = 8 then
                        m_p <= resize(unsigned(rf_qq), 37);
                    elsif q_cnt = 9 then
                        m_k <= signed(rf_qq);
                    end if;
                    -- multiply 9..24, arithmetic shifts 25..39
                    if q_cnt >= 9 and q_cnt <= 24 then
                        if m_p(0) = '1' then
                            v_hi := signed(m_p(36 downto 16)) + resize(m_a, 21);
                        else
                            v_hi := signed(m_p(36 downto 16));
                        end if;
                        m_p(36 downto 15) <= unsigned(resize(v_hi, 22));
                        m_p(14 downto 0) <= m_p(15 downto 1);
                    elsif q_cnt >= 25 and q_cnt <= 39 then
                        if q_cnt - 25 < q_uc.sh then
                            m_p <= unsigned(shift_right(signed(m_p), 1));
                        end if;
                    end if;
                    if q_cnt = 10 then
                        if q_uc.ng = '1' then m_k <= m_k + 1; end if;                   -- -P = not P + 1
                    elsif q_cnt = 41 then
                        if q_uc.ng = '1' then
                            m_res <= signed(not m_p(27 downto 0)) + resize(m_k, 28);
                        else
                            m_res <= signed(m_p(27 downto 0)) + resize(m_k, 28);
                        end if;
                    elsif q_cnt = 42 then
                        if q_uc.post = 2 and m_res < 0 then m_res <= (others => '0'); end if;
                    elsif q_cnt = 43 then
                        m_wr <= '1';
                    elsif q_cnt = 47 then
                        v_end := '1';
                    end if;
                end if;
                -- write-back (the high word of a wide result one clock later, address + 8) and outputs
                v_d := f_am(q_uc.d, q_uc.dm, '0', q_ci, '0');
                v_ci := to_integer(q_ci);
                if m_wr = '1' then
                    if q_uc.wr = '1' then
                        rf_we <= '1'; rf_wa <= v_d; rf_wd <= std_logic_vector(m_res(15 downto 0));
                    end if;
                    case q_uc.o is
                        when O_X0  => r_x0 <= m_res(13 downto 0);
                        when O_CX2 => r_cx2 <= m_res(13 downto 0);
                        when O_LX0 => r_lx0 <= m_res(13 downto 0);
                        when O_XF  => r_xf(v_ci mod 4) <= m_res(13 downto 0);
                        when O_TR  => r_tr <= unsigned(m_res(11 downto 0));
                        when O_TRG => r_trg <= unsigned(m_res(11 downto 0));
                        when O_A0  => r_a0 <= unsigned(m_res(23 downto 0));
                        when O_D0  => r_d0 <= m_res(17 downto 0);
                        when O_HWW => r_hww(v_ci mod 4) <= unsigned(m_res(10 downto 0));
                        when O_HWP => r_hwp(v_ci mod 4) <= unsigned(m_res(9 downto 0));
                        when O_HWD => r_hwd(v_ci mod 4) <= unsigned(m_res(7 downto 0));
                        when O_ARS => r_ars <= unsigned(m_res(14 downto 0));
                        when O_KLU => r_klu <= unsigned(m_res(10 downto 0));
                        when O_GTH => r_gth <= unsigned(m_res(7 downto 0));
                        when others => null;
                    end case;
                elsif m_wr2 = '1' then
                    rf_we <= '1'; rf_wa <= v_d + 8; rf_wd <= std_logic_vector(resize(m_res(27 downto 16), 16));
                end if;
                if v_end = '1' then
                    q_cnt <= (others => '0');
                    if q_st = ST_FIGEND and q_ci /= 3 then
                        q_ci <= q_ci + 1; q_st <= to_unsigned(ST_FIG, 8);
                    elsif q_st = ST_RAYEND and q_ci /= 3 then
                        q_ci <= q_ci + 1; q_st <= to_unsigned(ST_RAY, 8);
                    elsif q_st = ST_RAYEND then
                        q_run <= '0'; q_done <= '1';
                    else
                        if q_st = ST_FIGEND then q_ci <= (others => '0'); end if;
                        q_st <= q_st + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_seq;

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 2/3 clocks after the source avid rise)
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
            g_avd <= g_avid;
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- Line engine: runs in hblank (8 clocks after the generated avid falls, or once after the register
    -- machine at the field start) and prepares the NEXT line.
    --   t 0       per-line DDAs (vy, label / lens / arm), figure rows (4-step rotation t 0..3)
    --   t 0..17   RF reads: ray steps (6) and ellipse steps (3); at the field start the ray crossings
    --             and ellipse offsets (lo/hi pairs); data 3 clocks later, rotating chains
    --   t 5..     slot tables (3 slots x 7 words: w3 of row r, then w0..w2 of rows r and r+1), lerps
    --   t 22..29  squares of the line offsets -> thresholds; flags
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_t : integer range 0 to 127;
        variable v_r : signed(21 downto 0);
        variable v_x : signed(23 downto 0);
        variable v_d : signed(19 downto 0);
        variable v_e : signed(14 downto 0);
        variable v_a : signed(14 downto 0);
        variable v_dy : signed(19 downto 0);
        variable v_cy : signed(19 downto 0);
        variable v_tg : t_tag;
        variable v_row : unsigned(5 downto 0);
        variable v_dd : signed(8 downto 0);
        variable v_bb : unsigned(7 downto 0);
        variable v_k : integer range 0 to 17;
        variable v_pa, v_pb : signed(10 downto 0);
        variable v_sa : unsigned(15 downto 0);
        variable v_q2 : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            l_kickp <= '0';
            if q_done = '1' then l_fkick <= '1'; end if;
            if g_avd = '1' and g_avid = '0' then
                l_dly <= to_unsigned(1, 4);
            elsif l_dly /= 0 then
                l_dly <= l_dly + 1;
                if l_dly = 8 then l_kickp <= '1'; l_dly <= (others => '0'); end if;
            end if;
            if (l_kickp = '1' or (l_fkick = '1' and g_avid = '0' and q_run = '0')) and l_run = '0' then
                l_run <= '1'; l_t <= (others => '0');
                l_init <= l_fkick and not l_kickp;
                if l_kickp = '0' then l_fkick <= '0'; end if;
            elsif l_run = '1' then
                l_t <= l_t + 1;
                if l_t = 75 then l_run <= '0'; end if;
            end if;
            v_t := to_integer(l_t);

            if l_run = '1' then
                -- per-line DDAs (next line)
                if v_t = 0 then
                    if l_init = '1' then
                        l_vy <= (others => '0'); l_line <= (others => '0');
                        l_lr <= to_signed(-12437, 20); l_ln <= to_signed(-63326, 20); l_ar <= to_signed(-2658816, 24);
                    else
                        l_vy <= l_vy + resize(s_ys, 19); l_line <= l_line + 1;
                        l_ar <= l_ar + signed(resize(r_ars, 24));
                    end if;
                end if;
                -- figure rows: 4-step rotation at t 10..13 (rv(0) <- rv(1) <- .. <- rv(0) + kv, kv from the RF);
                -- label / lens rows at t 14 / 15
                if v_t >= 10 and v_t <= 13 then
                    if l_init = '1' then
                        case v_t is
                            when 10 => v_r := to_signed(RV0_0, 22);
                            when 11 => v_r := to_signed(RV0_1, 22);
                            when 12 => v_r := to_signed(RV0_2, 22);
                            when others => v_r := to_signed(RV0_3, 22);
                        end case;
                    else
                        v_r := l_rv(0) + resize(signed(rf_qq), 22);
                    end if;
                    l_rv(0) <= l_rv(1); l_rv(1) <= l_rv(2); l_rv(2) <= l_rv(3); l_rv(3) <= v_r;
                end if;
                if l_init = '0' and v_t = 14 then l_lr <= l_lr + resize(signed(rf_qq), 20); end if;
                if v_t = 14 then
                    if l_rv(0) >= 0 and l_rv(0) < 2 ** 20 then l_hin <= '1'; else l_hin <= '0'; end if;
                end if;
                if v_t = 20 then l_hdy <= signed(resize(l_vy, 20)) - 152858; end if;     -- halo centre 597.1 v
                if l_init = '0' and v_t = 15 then l_ln <= l_ln + resize(signed(rf_qq), 20); end if;
                -- RF reads: normal t 0..3 ray steps, 4..6 ellipse steps (data 3 clocks later);
                -- field start t 0..7 ray crossings (lo, hi), 8..13 ellipse offsets (lo, hi)
                if l_init = '1' then
                    if v_t <= 7 then
                        l_ra <= to_unsigned(RF_XRAL + v_t / 2 + 8 * (v_t mod 2), 8);
                    elsif v_t <= 13 then
                        l_ra <= to_unsigned(RF_ES0L + (v_t - 8) / 2 + 8 * ((v_t - 8) mod 2), 8);
                    end if;
                else
                    if v_t <= 3 then
                        l_ra <= to_unsigned(RF_DXL + v_t, 8);
                    elsif v_t <= 6 then
                        l_ra <= to_unsigned(RF_EST + v_t - 4, 8);
                    elsif v_t <= 10 then
                        l_ra <= to_unsigned(RF_KV + v_t - 7, 8);
                    elsif v_t = 11 then
                        l_ra <= to_unsigned(RF_LRS, 8);
                    elsif v_t = 12 then
                        l_ra <= to_unsigned(RF_LNS, 8);
                    end if;
                end if;
                -- ray / ellipse chains (rotating)
                if l_init = '1' then
                    if v_t >= 3 and v_t <= 10 then
                        if v_t mod 2 = 1 then
                            l_lo <= rf_qq;
                        else
                            l_xr(0) <= l_xr(1); l_xr(1) <= l_xr(2); l_xr(2) <= l_xr(3);
                            l_xr(3) <= signed(std_logic_vector'(rf_qq(7 downto 0) & l_lo));
                        end if;
                    elsif v_t >= 11 and v_t <= 16 then
                        if v_t mod 2 = 1 then
                            l_lo <= rf_qq;
                        else
                            l_ey(0) <= l_ey(1); l_ey(1) <= l_ey(2);
                            l_ey(2) <= signed(std_logic_vector'(rf_qq(7 downto 0) & l_lo));
                        end if;
                    end if;
                else
                    if v_t >= 3 and v_t <= 6 then
                        l_xr(0) <= l_xr(1); l_xr(1) <= l_xr(2); l_xr(2) <= l_xr(3);
                        l_xr(3) <= l_xr(0) + resize(signed(rf_qq), 24);
                    elsif v_t >= 7 and v_t <= 9 then
                        l_ey(0) <= l_ey(1); l_ey(1) <= l_ey(2);
                        l_ey(2) <= l_ey(0) + resize(signed(rf_qq), 24);
                    end if;
                end if;
            end if;

            -- slot tables (slot s = 0 woman, 1 centre (hands or bust), 2 soldier).  Stage A counts
            -- (from t 5) slot s and word j = 0 (w3 of row r), 1 w0 r, 2 w0 r+1, 3 w1 r, 4 w1 r+1, 5 w2 r,
            -- 6 w2 r+1; stage B picks the slot's figure and row; stage C addresses the ROM; the tag
            -- travels to t_qq (stage E); lerps of the two edges of a word pair at j = 2 / 4 / 6.
            if l_run = '1' and v_t = 15 then
                l_tj <= (others => '0'); l_fs <= (others => '0'); l_tv <= '1';
            elsif l_tv = '1' then
                if l_tj = 6 then
                    l_tj <= (others => '0');
                    if l_fs = 2 then l_tv <= '0'; else l_fs <= l_fs + 1; end if;
                else
                    l_tj <= l_tj + 1;
                end if;
            end if;
            -- C: the slot's figure and row (the row DDAs are stable while the tables are read) -> ROM address
            v_tg.v := l_tv; v_tg.s := l_fs; v_tg.j := l_tj;
            case l_fs is
                when "00" => v_tg.f := "10";
                when "10" => v_tg.f := "11";
                when others =>
                    if l_hin = '1' then v_tg.f := "00"; else v_tg.f := "01"; end if;
            end case;
            v_r := l_rv(to_integer(v_tg.f));
            v_row := unsigned(v_r(19 downto 14));
            if l_tj = 0 then
                t_ra <= v_tg.f & v_row & "11";
            elsif l_tj(0) = '1' then
                t_ra <= v_tg.f & v_row & resize(shift_right(l_tj - 1, 1), 2);
            elsif v_row = 63 then
                t_ra <= v_tg.f & v_row & resize(shift_right(l_tj - 2, 1), 2);
            else
                t_ra <= v_tg.f & (v_row + 1) & resize(shift_right(l_tj - 2, 1), 2);
            end if;
            l_tgC <= v_tg; l_tgD <= l_tgC; l_tgE <= l_tgD;
            -- E: t_qq holds the word addressed in C
            l_wv <= '0';
            v_r := l_rv(to_integer(l_tgE.f));
            if l_tgE.v = '1' then
                if l_tgE.j = 0 then
                    l_lab(to_integer(l_tgE.s)) <= t_qq(14 downto 0);
                    l_lp <= t_qq(15);
                    if v_r >= 0 and v_r < 2 ** 20 then l_val(to_integer(l_tgE.s)) <= '1'; else l_val(to_integer(l_tgE.s)) <= '0'; end if;
                    if l_tgE.s = 1 then l_cfig <= l_tgE.f(0); end if;
                    l_wfr <= unsigned(v_r(13 downto 10));
                    l_xfq <= r_xf(to_integer(l_tgE.f));
                elsif l_tgE.j(0) = '1' then
                    l_w0 <= t_qq;
                else
                    if l_lp = '1' then
                        l_d0 <= signed(resize(unsigned(t_qq(7 downto 0)), 9)) - signed(resize(unsigned(l_w0(7 downto 0)), 9));
                        l_d1 <= signed(resize(unsigned(t_qq(15 downto 8)), 9)) - signed(resize(unsigned(l_w0(15 downto 8)), 9));
                    else
                        l_d0 <= (others => '0'); l_d1 <= (others => '0');
                    end if;
                    l_b0 <= unsigned(l_w0(7 downto 0)); l_b1 <= unsigned(l_w0(15 downto 8));
                    l_wp <= resize(l_tgE.s * 3, 4) + resize(shift_right(l_tgE.j - 2, 1), 4);
                    l_xw <= l_xfq; l_fw <= l_wfr;                       -- this slot's (the next slot's j 0 may follow)
                    l_wv <= '1';
                end if;
            end if;
            -- the lerp and the write, one edge per clock (the pair's second edge a clock later):
            -- E = Xf + 4 e + (d f) / 4 in three steps (P1 base and the two half products, P2 the product,
            -- P3 the sum, saturated and written)
            l_wv2 <= l_wv; l_d1q <= l_d1; l_b1q <= l_b1;
            if l_wv = '1' then
                v_dd := l_d0; v_bb := l_b0; v_k := to_integer(l_wp) * 2;
            else
                v_dd := l_d1q; v_bb := l_b1q; v_k := to_integer(l_wp) * 2 + 1;
            end if;
            l_pv1 <= l_wv or l_wv2; l_pk1 <= v_k;
            l_pbs1 <= resize(l_xw, 15) + signed(resize(shift_left(resize(v_bb, 10), 2), 15));
            v_pa := (others => '0'); v_pb := (others => '0');
            if l_fw(0) = '1' then v_pa := resize(v_dd, 11); end if;
            if l_fw(1) = '1' then v_pa := v_pa + shift_left(resize(v_dd, 11), 1); end if;
            if l_fw(2) = '1' then v_pb := resize(v_dd, 11); end if;
            if l_fw(3) = '1' then v_pb := v_pb + shift_left(resize(v_dd, 11), 1); end if;
            l_ppa1 <= v_pa; l_ppb1 <= v_pb;
            l_pv2 <= l_pv1; l_pk2 <= l_pk1; l_pbs2 <= l_pbs1;
            l_pp2 <= resize(l_ppa1, 14) + shift_left(resize(l_ppb1, 14), 2);
            if l_pv2 = '1' then
                v_e := l_pbs2 + resize(shift_right(l_pp2, 2), 15);
                for k in 0 to 17 loop
                    if k = l_pk2 then
                        if v_e < 0 then l_e(k) <= (others => '0');
                        elsif v_e > 4095 then l_e(k) <= (others => '1');
                        else l_e(k) <= unsigned(v_e(11 downto 0)); end if;
                    end if;
                end loop;
            end if;

            -- derived per-line values: four squares |dy|^2 / 64 on a serial shift-add squarer (one every 13
            -- clocks from t 22: halo, rim, base, coffee), then the thresholds; flags at t 72 .. 75
            if l_run = '1' then
                if v_t >= 22 and v_t < 74 then
                    if l_sk = 0 then
                        case l_sq is
                            when "00" => v_x := resize(l_hdy, 24);
                            when "01" => v_x := l_ey(0);
                            when "10" => v_x := l_ey(1);
                            when others => v_x := l_ey(2);
                        end case;
                        l_sxa <= unsigned(v_x(23 downto 8));                     -- signed, integer v px
                    elsif l_sk = 1 then
                        if l_sxa(15) = '1' then v_sa := not l_sxa; else v_sa := l_sxa; end if;   -- |x| (one's complement)
                        if v_sa(15 downto 11) /= 0 then l_sqi <= (others => '1'); l_sqb <= (others => '1');
                        else l_sqi <= v_sa(10 downto 0); l_sqb <= v_sa(10 downto 0); end if;
                        l_sqa <= (others => '0');
                    else
                        -- acc = acc / 2 + (bit ? sqi << 10 : 0), multiplier bits LSB first (sk 2 .. 12)
                        if l_sqb(0) = '1' then
                            l_sqa <= shift_right(l_sqa, 1) + shift_left(resize(l_sqi, 22), 10);
                        else
                            l_sqa <= shift_right(l_sqa, 1);
                        end if;
                        l_sqb <= shift_right(l_sqb, 1);
                    end if;
                    if l_sk = 12 then l_sk <= (others => '0'); l_sq <= l_sq + 1; else l_sk <= l_sk + 1; end if;
                else
                    l_sk <= (others => '0'); l_sq <= (others => '0');
                end if;
                -- square >> 6 is ready at sk = 0 of the next slot (acc holds sqi * sqi)
                l_sqo <= l_sqa(21 downto 6);
                l_sqv <= '0';
                if v_t >= 22 and l_sk = 0 and v_t > 22 then l_sqv <= '1'; l_sqn <= l_sq - 1; end if;
                if l_sqv = '1' then
                    case l_sqn is
                        when "00" =>
                            if l_sqo < 4096 then
                                l_tr <= signed(resize(r_tr, 13)) - signed(resize(l_sqo, 13));
                                l_trg <= signed(resize(r_trg, 13)) - signed(resize(l_sqo, 13));
                            else
                                l_tr <= to_signed(-1, 13); l_trg <= to_signed(-1, 13);
                            end if;
                        when "01" => if l_sqo < 4096 then l_te1 <= to_signed(441, 13) - signed(resize(l_sqo, 13)); else l_te1 <= to_signed(-1, 13); end if;
                        when "10" => if l_sqo < 4096 then l_te2 <= to_signed(441, 13) - signed(resize(l_sqo, 13)); else l_te2 <= to_signed(-1, 13); end if;
                        when others => if l_sqo < 4096 then l_te3 <= to_signed(357, 13) - signed(resize(l_sqo, 13)); else l_te3 <= to_signed(-1, 13); end if;
                    end case;
                end if;
                if v_t = 70 then ln_ra <= "00" & unsigned(l_ln(12 downto 8)) & '0'; end if;
                if v_t = 71 then ln_ra <= "00" & unsigned(l_ln(12 downto 8)) & '1'; end if;
                if v_t = 72 then l_hlo <= unsigned(ln_q(14 downto 8)); l_hli <= unsigned(ln_q(6 downto 0)); end if;
                if v_t = 73 then l_hro <= unsigned(ln_q(14 downto 8)); l_hri <= unsigned(ln_q(6 downto 0)); end if;
                if v_t = 11 then
                    -- crossings are final (t 3..6 rotation); midpoints between neighbours, left to right L TL TR R
                    l_m1 <= resize(shift_right(l_xr(2)(21 downto 8) + l_xr(0)(21 downto 8), 1), 14);
                    l_m2 <= resize(shift_right(l_xr(0)(21 downto 8) + l_xr(1)(21 downto 8), 1), 14);
                    l_m3 <= resize(shift_right(l_xr(1)(21 downto 8) + l_xr(3)(21 downto 8), 1), 14);
                end if;
                case v_t is
                    when 72 =>
                        -- flags from vy (virtual lines Q.8)
                        if l_vy < 152858 then l_up <= '1'; else l_up <= '0'; end if;
                        if l_ey(0) >= 0 and l_ey(1) < 0 then l_wall <= '1'; else l_wall <= '0'; end if;        -- rim .. base centres
                        v_q2 := l_ln(16 downto 6);                                      -- lens rows, Q.2
                        if v_q2 >= 0 and v_q2 < 128 then l_lens <= '1'; else l_lens <= '0'; end if;
                        if v_q2 >= 11 and v_q2 < 26 then l_brg <= '1'; else l_brg <= '0'; end if;
                        if v_q2 >= 4 and v_q2 < 30 then l_end <= '1'; else l_end <= '0'; end if;
                        if v_q2 >= -14 and v_q2 < 24 then l_arm <= '1'; else l_arm <= '0'; end if;
                        if v_q2 >= 42 and v_q2 < 53 then l_fhs <= "01";
                        elsif v_q2 >= 53 and v_q2 < 64 then l_fhs <= "10";
                        elsif v_q2 >= 64 and v_q2 < 74 then l_fhs <= "11";
                        else l_fhs <= "00"; end if;
                        if l_lr >= 0 and l_lr < 24 * 256 then l_labr <= '1'; else l_labr <= '0'; end if;
                        l_fd1 <= resize(l_ln, 20) - 3814;                                -- flag disc centre, lens rows Q.8

                        -- arm x range (h from X0)
                        if v_q2 < 4 then l_axa <= to_unsigned(560, 10); l_axb <= to_unsigned(670, 10);
                        elsif v_q2 < 14 then l_axa <= to_unsigned(620, 10); l_axb <= to_unsigned(760, 10);
                        else l_axa <= to_unsigned(710, 10); l_axb <= to_unsigned(801, 10); end if;
                        -- label row 0..23
                        if l_lr < 0 then l_lrow <= (others => '0');
                        elsif l_lr(19 downto 8) > 23 then l_lrow <= to_unsigned(23, 5);
                        else l_lrow <= unsigned(l_lr(12 downto 8)); end if;
                        -- coffee shading: (vy - 473.2) * 41 / 1024 (levels)
                        v_cy := shift_right(l_ey(2)(19 downto 0), 8);                    -- dy' = 2.217 dy
                        v_cy := shift_right(shift_left(v_cy, 4) + shift_left(v_cy, 1), 10);
                        if v_cy < -16 then l_cy <= to_signed(-16, 6); elsif v_cy > 15 then l_cy <= to_signed(15, 6); else l_cy <= v_cy(5 downto 0); end if;
                        l_vy2 <= l_vy(18 downto 7);
                    when 73 =>
                        if l_fd1 < 0 then l_fd1 <= not l_fd1; end if;                    -- |dy| (one's complement)
                    when 74 =>
                        -- half lens rows (Q.8 >> 7) = 1.25 v px
                        if l_fd1(19 downto 12) /= 0 then l_fdy <= (others => '1'); l_frow <= '0';
                        else l_fdy <= unsigned(l_fd1(11 downto 7)); l_frow <= '1'; end if;
                        if l_fd1(19 downto 9) = 0 then l_fey <= '1'; else l_fey <= '0'; end if;
                    when 75 =>
                        l_fch <= to_unsigned(K_CHORD(to_integer(l_fdy)), 7);
                        if l_frow = '0' then l_fch <= (others => '0'); end if;

                    when others => null;
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Pixel path (S1 = the register loaded for pixel 0 at the g_start edge; output s_y21 = latency 16)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_av : signed(24 downto 0);
        variable v_lu : unsigned(10 downto 0);
        variable v_d : signed(13 downto 0);
        variable v_a : unsigned(12 downto 0);
        variable v_g : signed(10 downto 0);
        variable v_ga : unsigned(9 downto 0);
        variable v_x14 : signed(14 downto 0);
        variable v_u : signed(15 downto 0);
        variable v_ta, v_tb, v_lt : std_logic;
        variable v_dd : signed(18 downto 0);
        variable v_l : signed(11 downto 0);
        variable v_gr : signed(11 downto 0);
        variable v_lvl : unsigned(3 downto 0);
        variable v_y, v_uu, v_vv : unsigned(9 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_ci : signed(7 downto 0);
        variable v_t14 : signed(13 downto 0);
        variable v_oa : unsigned(7 downto 0);
        variable v_xr, v_dp, v_dn : signed(13 downto 0);
        variable v_sl : unsigned(2 downto 0);
        variable v_rc : unsigned(1 downto 0);
        variable v_h8 : unsigned(7 downto 0);
        variable v_z0, v_z1, v_z2 : unsigned(3 downto 0);
        variable v_hi : std_logic;
        variable v_n13 : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- S1: beam, squared-distance DDA, label column, noise
            if p_lfsr(0) = '1' then p_lfsr <= shift_right(p_lfsr, 1) xor x"B400"; else p_lfsr <= shift_right(p_lfsr, 1); end if;
            v_n13 := signed(resize(p_lfsr(15 downto 8) & "0000", 13)) - signed(resize(p_nm, 13));
            v_n13 := signed(resize(p_nm, 13)) + shift_right(v_n13, 4);
            p_nm <= unsigned(v_n13(11 downto 0));
            g_st1 <= g_start; g_st2 <= g_st1;
            p_gc <= rotate_left(l_line(8 downto 1), 3) + resize(s_fc(7 downto 3), 8);
            if g_start = '1' then
                p_vx <= (others => '0');
                p_av <= r_a0; p_dv <= r_d0;
                p_lu <= (others => '0'); p_lb <= (others => '0'); p_lg <= (others => '0'); p_lin <= '0';
            else
                p_vx <= p_vx + resize(s_xs, 13);
                v_av := signed('0' & p_av) + resize(p_dv, 25);
                if v_av < 0 then p_av <= (others => '0'); else p_av <= unsigned(v_av(23 downto 0)); end if;
                p_dv <= p_dv + signed(resize(s_dd, 18));
                -- label: 60 columns (6 bits in each of 10 groups) from the inner start
                if signed('0' & p_vx) >= r_lx0 then p_lin <= '1'; end if;
                if p_lin = '1' then
                    v_lu := resize(p_lu, 11) + resize(r_klu, 11);
                    p_lu <= v_lu(9 downto 0);
                    if v_lu >= 1024 then
                        if p_lb = 5 then p_lb <= (others => '0'); p_lg <= p_lg + 1; else p_lb <= p_lb + 1; end if;
                    end if;
                end if;
            end if;
            -- S2: offsets from the halo centre and the page, edge compares, distance^2 (64 v-px^2, sat 4095)
            p_vxc2 <= signed('0' & p_vx) - r_cx2;
            if signed('0' & p_vx) < l_m1 then p_ri2 <= "10"; elsif signed('0' & p_vx) < l_m2 then p_ri2 <= "00";
            elsif signed('0' & p_vx) < l_m3 then p_ri2 <= "01"; else p_ri2 <= "11"; end if;
            p_vr2 <= signed('0' & p_vx) - r_x0;
            for k in 0 to 17 loop
                if p_vx(12) = '1' or p_vx(11 downto 0) >= l_e(k) then p_ge2(k) <= '1'; else p_ge2(k) <= '0'; end if;
            end loop;
            if p_av(23 downto 20) /= 0 then p_aq2 <= (others => '1'); else p_aq2 <= p_av(19 downto 8); end if;
            p_wob2 <= signed(resize(p_lfsr(4 downto 0), 6)) - 16;
            p_vx2 <= p_vx;
            p_lab2(0) <= p_lin; p_lg2 <= p_lg; p_lb2 <= p_lb;

            -- S3: rays, discs, zone counts, glasses, ground lattices, cup shading, label address
            -- rays: S2 picked the nearest crossing (absolute midpoints, L TL TR R); S3 takes d = x - crossing
            -- both ways (|d| = whichever is not negative) and that ray's three half widths; S4 compares
            v_xr := l_xr(to_integer(p_ri2))(21 downto 8);
            v_dp := signed('0' & p_vx2) - v_xr;
            v_dn := v_xr - signed('0' & p_vx2);
            if v_dp(13) = '0' then p_ra3 <= unsigned(v_dp(12 downto 0)); else p_ra3 <= unsigned(v_dn(12 downto 0)); end if;
            p_twd3 <= r_hwd(to_integer(p_ri2)); p_twp3 <= r_hwp(to_integer(p_ri2)); p_tww3 <= r_hww(to_integer(p_ri2));
            if signed('0' & p_aq2) < l_tr then p_disc3 <= '1'; else p_disc3 <= '0'; end if;
            if signed('0' & p_aq2) < l_trg then p_rin3 <= '1'; else p_rin3 <= '0'; end if;
            if signed('0' & p_aq2) < l_te1 then p_e13 <= '1'; else p_e13 <= '0'; end if;
            if signed('0' & p_aq2) < l_te2 or (l_wall = '1' and p_aq2 < 441) then p_e23 <= '1'; else p_e23 <= '0'; end if;
            if signed('0' & p_aq2) < l_te3 then p_e33 <= '1'; else p_e33 <= '0'; end if;
            for s in 0 to 2 loop
                if p_ge2(6 * s + 5) = '1' then v_sl := "111";
                elsif p_ge2(6 * s + 4) = '1' then v_sl := unsigned(l_lab(s)(14 downto 12));
                elsif p_ge2(6 * s + 3) = '1' then v_sl := unsigned(l_lab(s)(11 downto 9));
                elsif p_ge2(6 * s + 2) = '1' then v_sl := unsigned(l_lab(s)(8 downto 6));
                elsif p_ge2(6 * s + 1) = '1' then v_sl := unsigned(l_lab(s)(5 downto 3));
                elsif p_ge2(6 * s) = '1' then v_sl := unsigned(l_lab(s)(2 downto 0));
                else v_sl := "111"; end if;
                p_sl3(s) <= v_sl;
            end loop;
            -- glasses, relative to page h 560 (10 bits): lens centres 353 / 617, flag cross 357
            v_g := resize(p_vr2 - 560, 11);
            if p_vr2 >= 560 and p_vr2 < 1584 then p_gw3 <= '1'; else p_gw3 <= '0'; end if;
            if p_vr2 >= 560 and p_vr2 < 1584 and l_lens = '1' then p_gv3 <= '1'; else p_gv3 <= '0'; end if;
            v_ga := unsigned(v_g(9 downto 0));
            if v_ga > 353 then v_a := resize(v_ga - 353, 13); else v_a := resize(353 - v_ga, 13); end if;
            p_dl3 <= v_a(9 downto 0);
            if v_ga > 617 then v_a := resize(v_ga - 617, 13); else v_a := resize(617 - v_ga, 13); end if;
            p_dr3 <= v_a(9 downto 0);
            if p_vr2 >= 560 and p_vr2 < 1584 and ((l_brg = '1' and p_vr2 >= 1027 and p_vr2 < 1080)
               or (l_end = '1' and p_vr2 >= 1273 and p_vr2 < 1301)
               or (l_arm = '1' and p_vr2 >= signed(resize(l_axa, 14)) and p_vr2 < signed(resize(l_axb, 14)))) then
                p_gfx3 <= '1';
            else
                p_gfx3 <= '0';
            end if;
            if p_vr2 >= 897 and p_vr2 < 910 then p_gst3 <= "01";
            elsif p_vr2 >= 910 and p_vr2 < 924 then p_gst3 <= "10";
            elsif p_vr2 >= 924 and p_vr2 < 936 then p_gst3 <= "11";
            else p_gst3 <= "00"; end if;
            -- ground lattices: (3x + y, 3y - x) and (2x - y, x + 2y) in h units, with ragged jitter
            v_x14 := signed(resize(p_vx2 & '0', 15)) + signed(resize(p_vx2, 15)) + signed(resize(l_vy2, 15)) + resize(p_wob2, 15);
            p_u13 <= unsigned(resize(v_x14(14 downto 10), 8));
            v_u := signed(resize(l_vy2 & '0', 16)) + signed(resize(l_vy2, 16)) - signed(resize(p_vx2, 16)) + resize(p_wob2, 16) + 8192;
            p_v13 <= unsigned(resize(v_u(15 downto 9), 8));
            v_u := signed(resize(p_vx2 & '0', 16)) - signed(resize(l_vy2, 16)) + resize(p_wob2, 16) + 8192;
            p_u23 <= unsigned(resize(v_u(15 downto 9), 8));
            v_u := signed(resize(p_vx2, 16)) + signed(resize(l_vy2 & '0', 16)) + resize(p_wob2, 16);
            p_v23 <= unsigned(resize(v_u(15 downto 10), 8));
            p_nm3 <= p_nm(11 downto 4);
            v_h8 := p_vx2(8 downto 1) + p_gc;
            p_gx3 <= v_h8 + (v_h8(6 downto 0) & '0');
            -- Osc shade: crossed triangle waves (blob scale ~ 60 v px), drifting slowly with the field count
            v_u := signed(resize(p_vx2(12 downto 2), 16)) + signed(resize(l_vy2(11 downto 1), 16)) + signed(resize(s_fc, 16));
            v_x14 := signed(resize(l_vy2(11 downto 2) & '0', 15)) + signed(resize(l_vy2(11 downto 2), 15)) - signed(resize(p_vx2(12 downto 3), 15));
            p_pa3 <= f_tri6(unsigned(v_u(6 downto 1))) & "0";
            p_pb3 <= f_tri6(unsigned(v_x14(5 downto 0))) & "0";
            -- cup: coffee x term 5 + 17 dx / 1024 + shadow 26 (dx - 136)+ / 1024; wall shade 15 - dx / 32
            v_d := p_vxc2 - 6;
            v_t14 := shift_right(v_d, 5);                                           -- darker to the right
            if v_t14 > 15 then p_cx3 <= to_signed(15, 8); elsif v_t14 < -12 then p_cx3 <= to_signed(-12, 8);
            else p_cx3 <= v_t14(7 downto 0) + 5; end if;
            v_d := shift_right(p_vxc2, 5);
            if v_d < 0 then p_wsh3 <= to_unsigned(15, 4);
            elsif v_d > 7 then p_wsh3 <= to_unsigned(8, 4);
            else p_wsh3 <= to_unsigned(15, 4) - unsigned(v_d(3 downto 0)); end if;
            -- label (S10 = DUCI, rows from the engine): box 709..1023, bitmap 719..1013 (h from X0)
            p_lab3 <= "00";
            if s_lab = '1' and l_labr = '1' and l_cfig = '0' and p_vr2 >= 709 and p_vr2 < 1023 then
                p_lab3(1) <= '1';
                if p_lab2(0) = '1' and p_vr2 < 1013 then p_lab3(0) <= '1'; end if;
            end if;
            p_ldu3 <= resize(shift_left(resize(l_lrow, 8), 3) + shift_left(resize(l_lrow, 8), 1) + resize(p_lg2, 8), 8);
            p_lb3 <= p_lb2;

            -- S4: classes, figure winner and zone, ground hashes
            v_rc := "00";
            if p_ra3 < resize(p_tww3, 13) then v_rc := "01"; end if;
            if p_ra3 < resize(p_twp3, 13) then v_rc := "10"; end if;
            if p_ra3 < resize(p_twd3, 13) then v_rc := "11"; end if;
            if l_up = '1' then p_ray4 <= v_rc; else p_ray4 <= "00"; end if;
            if p_disc3 = '1' and p_rin3 = '1' then p_halo4 <= "01";
            elsif p_disc3 = '1' then p_halo4 <= "10";
            else p_halo4 <= "00"; end if;
            p_bowl4 <= "00";
            if s_cup = '1' then
                if p_e33 = '1' then p_bowl4 <= "11";
                elsif p_e13 = '1' then p_bowl4 <= "01";
                elsif p_e23 = '1' then p_bowl4 <= "10"; end if;
            end if;
            v_ci := p_cx3 + resize(l_cy, 8) + signed(resize(p_lfsr(9 downto 9), 8));
            if v_ci < 0 then p_ci4 <= (others => '0'); elsif v_ci > 11 then p_ci4 <= to_unsigned(11, 4); else p_ci4 <= unsigned(v_ci(3 downto 0)); end if;
            p_wsh4 <= p_wsh3;
            -- glasses and flag: 1 frame, 2 eye, 8 green, 9 yellow, 10 black, 11 white, 12 red
            p_gl4 <= "0000";
            if p_gv3 = '1' then
                if (p_dl3 < l_hlo and not (p_dl3 < l_hli)) or (p_dr3 < l_hro and not (p_dr3 < l_hri)) then
                    p_gl4 <= "0001";
                elsif p_dr3 < 12 and l_fey = '1' then
                    p_gl4 <= "0010";
                elsif p_dl3 < l_hli then
                    if s_flag = '0' then
                        if p_dl3 < 12 and l_fey = '1' then p_gl4 <= "0010"; end if;
                    elsif l_frow = '1' and p_dl3 < l_fch then
                        p_gl4 <= "1100";
                    elsif l_fhs /= "00" then
                        p_gl4 <= "10" & l_fhs;
                    else
                        p_gl4 <= "10" & p_gst3;
                    end if;
                end if;
            end if;
            if p_gfx3 = '1' then p_gl4 <= "0001"; end if;
            -- figure winner: side slots (woman 0, soldier 2) in front of the centre slot (1)
            -- zones: woman 8..11, soldier 12..15, hands 0.., bust 3..
            case to_integer(p_sl3(0)) is
                when 0 | 2 => v_z0 := to_unsigned(8, 4); when 1 => v_z0 := to_unsigned(9, 4);
                when 3 => v_z0 := to_unsigned(10, 4); when others => v_z0 := to_unsigned(11, 4);
            end case;
            case to_integer(p_sl3(2)) is
                when 0 => v_z2 := to_unsigned(12, 4); when 1 | 3 => v_z2 := to_unsigned(13, 4);
                when 2 => v_z2 := to_unsigned(14, 4); when others => v_z2 := to_unsigned(15, 4);
            end case;
            if l_cfig = '0' then
                v_z1 := resize(p_sl3(1), 4);
            else
                case to_integer(p_sl3(1)) is
                    when 0 => v_z1 := to_unsigned(3, 4); when 1 => v_z1 := to_unsigned(4, 4);
                    when 2 => v_z1 := to_unsigned(5, 4); when 3 => v_z1 := to_unsigned(6, 4);
                    when 4 => v_z1 := to_unsigned(7, 4); when 5 => v_z1 := to_unsigned(8, 4);
                    when 6 => v_z1 := to_unsigned(9, 4); when others => v_z1 := to_unsigned(10, 4);
                end case;
            end if;
            if l_val(0) = '1' and p_sl3(0) /= 7 then p_win4 <= "01"; p_zone4 <= v_z0;
            elsif l_val(2) = '1' and p_sl3(2) /= 7 then p_win4 <= "01"; p_zone4 <= v_z2;
            elsif l_val(1) = '1' and p_sl3(1) /= 7 then p_win4 <= "10"; p_zone4 <= v_z1;
            else p_win4 <= "00"; p_zone4 <= v_z1; end if;
            p_ha4 <= f_h4(p_u13, p_v13, x"35");
            p_hb4 <= f_h4(p_u23, p_v23, x"9B");
            v_h8 := p_gx3 xor shift_right(p_gx3, 3);
            v_h8 := v_h8 + rotate_left(v_h8, 5);
            p_gx4 <= v_h8 xor shift_right(v_h8, 4);
            p_ma4 <= to_unsigned(12, 5) + resize(p_nm3(7 downto 6), 5) + resize(p_lfsr(5 downto 5), 5);
            if p_nm3 < 118 then
                p_mb4 <= to_unsigned(7, 5) + resize(p_nm3(7 downto 6), 5) + resize(p_lfsr(5 downto 5), 5);
            else
                p_mb4 <= to_unsigned(12, 5) + resize(p_nm3(7 downto 6), 5) + resize(p_lfsr(5 downto 5), 5);
            end if;
            p_pl4 <= resize(p_pa3 & '0', 8) + resize(p_pb3, 8) + 16;
            p_lab4 <= p_lab3; p_ldu4 <= p_ldu3; p_lb4 <= p_lb3;

            -- S5: the overlay code (what shows where no figure wins) and its parameter; the shade source
            -- (Video = the camera luma tapped straight from the input, a few px of horizontal offset; Osc =
            -- the blob field) and the grain
            if p_ha4 < r_gth then v_ta := '1'; else v_ta := '0'; end if;
            if p_hb4 < r_gth then v_tb := '1'; else v_tb := '0'; end if;
            v_lt := '0';
            if (v_ta = '1' and p_ha4 < shift_right(r_gth, 2)) or (v_tb = '1' and p_hb4 < shift_right(r_gth, 2)) then v_lt := '1'; end if;
            -- overlay ROM address: 0..127 ground (level & luma x/16), 128 + coffee level, 144 + wall shade,
            -- 160 + k flat colours (4 white, 5 pink, 6 line, 7 halo, 8 ring, 9 gold, 10 rim, 13 frame, 14 eye,
            -- 16..20 flag green / yellow / black / white / red); hi = drawn over the centre figure (cup, glasses)
            v_hi := '0';
            if p_bowl4 = "11" then v_oa := "1000" & p_ci4; v_hi := '1';
            elsif p_bowl4 = "01" then v_oa := to_unsigned(170, 8); v_hi := '1';
            elsif p_bowl4 = "10" then v_oa := "1001" & p_wsh4; v_hi := '1';
            elsif p_gl4 = "0001" then v_oa := to_unsigned(173, 8); v_hi := '1';
            elsif p_gl4 = "0010" then v_oa := to_unsigned(174, 8); v_hi := '1';
            elsif p_gl4(3) = '1' then v_oa := "10110" & p_gl4(2 downto 0); v_hi := '1';
            elsif p_halo4 = "10" then v_oa := to_unsigned(168, 8);
            elsif p_halo4 = "01" then if s_gold = '1' then v_oa := to_unsigned(169, 8); else v_oa := to_unsigned(167, 8); end if;
            elsif p_ray4 = "11" then v_oa := to_unsigned(166, 8);
            elsif p_ray4 = "10" then v_oa := to_unsigned(165, 8);
            elsif p_ray4 = "01" then v_oa := to_unsigned(164, 8);
            else
                if (v_ta = '1' and v_tb = '1') or v_lt = '1' then v_oa := "011" & p_ma4;
                elsif v_ta = '1' or v_tb = '1' then v_oa := "010" & p_mb4;
                elsif p_ha4 > 160 then v_oa := "001" & p_ma4;
                else v_oa := "000" & p_ma4; end if;
            end if;
            p_b(5) <= (win => p_win4, lab => p_lab4, lb => p_lb4, hi => v_hi);
            for k in 6 to 8 loop p_b(k) <= p_b(k - 1); end loop;
            p_oa5 <= v_oa;
            p_zone5 <= p_zone4; p_ldu5 <= p_ldu4;
            if s_osc = '0' then
                v_l := signed(resize(unsigned(data_in.y(9 downto 2)), 12)) - 16;
                v_l := v_l + shift_right(v_l, 4);
            else
                v_l := signed(resize(p_pl4, 12));
            end if;
            if v_l < 0 then p_l5 <= (others => '0'); elsif v_l > 255 then p_l5 <= (others => '1'); else p_l5 <= unsigned(v_l(7 downto 0)); end if;
            if p_zone4 = 1 then
                v_gr := resize(signed((not p_gx4(7)) & p_gx4(6 downto 0)), 12);
                p_gr5 <= resize(shift_right(shift_left(v_gr, 3) + v_gr, 4), 8);
            else
                v_gr := resize(signed((not p_lfsr(11)) & p_lfsr(10 downto 4)), 12);
                p_gr5 <= resize(shift_right(v_gr, 4), 8);
            end if;
            -- S6: level = shade + grain + dither -> palette address (zone & level), or the DUCI word
            v_l := signed(resize(p_l5, 12)) + resize(p_gr5, 12) + signed(resize(p_lfsr(3 downto 0), 12));
            if v_l < 0 then v_lvl := (others => '0'); elsif v_l > 255 then v_lvl := (others => '1'); else v_lvl := unsigned(v_l(7 downto 4)); end if;
            if p_b(5).lab(1) = '1' then pa <= p_ldu5; else pa <= p_zone5 & v_lvl; end if;
            ova <= p_oa5;
            -- S7: palette ROM.  S8: fabric register (py_qq, pc_qq).  S9: compose.  S10: pad (latency 16)
            v_y := unsigned(ov_hh(15 downto 6)); v_uu := unsigned(ov_hh(5 downto 0)) & unsigned(ov_ll(15 downto 12));
            v_vv := unsigned(ov_ll(11 downto 2));
            if p_b(8).win = "01" or (p_b(8).win = "10" and p_b(8).hi = '0' and p_b(8).lab(1) = '0') then
                v_y := unsigned(py_qq(9 downto 0)); v_uu := unsigned(pc_qq(15 downto 8)) & "00"; v_vv := unsigned(pc_qq(7 downto 0)) & "00";
            elsif p_b(8).lab(1) = '1' and p_b(8).hi = '0' then
                if p_b(8).lab(0) = '1' and py_qq(10 + to_integer(p_b(8).lb)) = '1' then
                    v_y := to_unsigned(K_CREAM(0), 10); v_uu := to_unsigned(K_CREAM(1), 10); v_vv := to_unsigned(K_CREAM(2), 10);
                else
                    v_y := to_unsigned(K_NAVY(0), 10); v_uu := to_unsigned(K_NAVY(1), 10); v_vv := to_unsigned(K_NAVY(2), 10);
                end if;
            end if;
            p_y9 <= v_y; p_u9 <= v_uu; p_v9 <= v_vv;
            p_y14 <= p_y9; p_u14 <= p_u9; p_v14 <= p_v9;

            -- 15..20: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent 8-sums on 8-bit
            -- chroma, luma [1 2 1]); 21: luma toward black (white 940 -> 777), the gate
            -- the samples 8 back come from a block-RAM ring (written at s_dwp, read 6 back, 2-clock read)
            s_boxu <= s_boxu + resize(p_u14(9 downto 2), 11) - resize(unsigned(c_q(15 downto 8)), 11);
            s_boxv <= s_boxv + resize(p_v14(9 downto 2), 11) - resize(unsigned(c_q(7 downto 0)), 11);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 12) + s_boxud; s_u20x <= v_sc(11 downto 2);
            v_sc := resize(s_boxv, 12) + s_boxvd; s_v20x <= v_sc(11 downto 2);
            s_y15 <= p_y14; s_y16 <= s_y15; s_y17 <= s_y16; s_y18 <= s_y17; s_y19 <= s_y18;
            s_y20x <= s_y19;
            v_sy := resize(s_y18, 12) + shift_left(resize(s_y19, 12), 1) + resize(s_y20x, 12);
            s_yf20 <= v_sy(11 downto 2);
            if s_yf20 > 64 then v_dl := s_yf20 - 64; else v_dl := (others => '0'); end if;
            if s_dg(0) = '1' then
                s_y21 <= s_yf20 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u21 <= s_u20x; s_v21 <= s_v20x;
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y21);
    data_out.u       <= std_logic_vector(s_u21);
    data_out.v       <= std_logic_vector(s_v21);
    data_out.hsync_n <= s_do(3);
    data_out.vsync_n <= s_do(2);
    data_out.field_n <= s_do(1);
    data_out.avid    <= s_do(0);

end architecture radiant;
