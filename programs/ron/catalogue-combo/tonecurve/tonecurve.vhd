-- Tonecurve: catalogue combo #34 (seed 20260918) -- colour bleeding / NTSC dot
-- crawl + picture-in-picture with warping + gamma curves / S-curve contrast.
-- Two knobs each; the switches and the slider decide how the three combine.
--
--   K1 Curve    gamma 0.5, 0.7, 1.5, 2.2, soft S, hard S, inverse S, crush
--   K2 Contrast linear gain about mid grey before the curve
--   K3 Size     inset 1/2, 1/4, 1/8
--   K4 Warp     inset wobble (sine per line, drifting)
--   K5 Bleed    chroma IIR smear length
--   K6 Lag      chroma delayed 0..31 px
--
--   S7  Curve   main picture / inset only
--   S8  Place   inset in the corner / centred
--   S9  Bleed   main / inset only
--   S10 Inset   picture / negative
--   S11 Lag     main / inset only
--   P12 Amount  how far the curved luma replaces the original
--
-- One line buffer read at the main or inset address; curve = 8 x 128 entry
-- EBR ROM on contrast-stretched luma; bleed = IIR on the fetched chroma;
-- lag = EBR ring on chroma.  Latency 18 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture tonecurve of program_top is

    constant C_LAT : integer := 18;

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return unsigned(v(9 downto 0));
        end if;
    end function;

    type t_sine_rom is array (0 to 255) of integer range -256 to 255;
    constant C_SINE : t_sine_rom := (
        -255, -255, -255, -254, -254, -253, -252, -251,
        -250, -249, -247, -246, -244, -242, -240, -238,
        -236, -233, -231, -228, -225, -222, -219, -215,
        -212, -208, -205, -201, -197, -193, -189, -185,
        -180, -176, -171, -167, -162, -157, -152, -147,
        -142, -136, -131, -126, -120, -115, -109, -103,
         -98,  -92,  -86,  -80,  -74,  -68,  -62,  -56,
         -50,  -44,  -37,  -31,  -25,  -19,  -13,   -6,
           0,    6,   13,   19,   25,   31,   37,   44,
          50,   56,   62,   68,   74,   80,   86,   92,
          98,  103,  109,  115,  120,  126,  131,  136,
         142,  147,  152,  157,  162,  167,  171,  176,
         180,  185,  189,  193,  197,  201,  205,  208,
         212,  215,  219,  222,  225,  228,  231,  233,
         236,  238,  240,  242,  244,  246,  247,  249,
         250,  251,  252,  253,  254,  254,  255,  255,
         255,  255,  255,  254,  254,  253,  252,  251,
         250,  249,  247,  246,  244,  242,  240,  238,
         236,  233,  231,  228,  225,  222,  219,  215,
         212,  208,  205,  201,  197,  193,  189,  185,
         180,  176,  171,  167,  162,  157,  152,  147,
         142,  136,  131,  126,  120,  115,  109,  103,
          98,   92,   86,   80,   74,   68,   62,   56,
          50,   44,   37,   31,   25,   19,   13,    6,
           0,   -6,  -13,  -19,  -25,  -31,  -37,  -44,
         -50,  -56,  -62,  -68,  -74,  -80,  -86,  -92,
         -98, -103, -109, -115, -120, -126, -131, -136,
        -142, -147, -152, -157, -162, -167, -171, -176,
        -180, -185, -189, -193, -197, -201, -205, -208,
        -212, -215, -219, -222, -225, -228, -231, -233,
        -236, -238, -240, -242, -244, -246, -247, -249,
        -250, -251, -252, -253, -254, -254, -255, -255
    );

    type t_curve is array (0 to 1023) of integer range 0 to 1023;
    constant C_CURVE : t_curve := (
        64, 64, 64, 64, 64, 64, 64, 64, 123, 167, 196, 221, 242, 260, 277, 293,
        308, 322, 335, 348, 360, 372, 383, 394, 404, 414, 424, 434, 443, 452, 461, 470,
        478, 487, 495, 503, 511, 519, 526, 534, 541, 549, 556, 563, 570, 577, 583, 590,
        597, 603, 610, 616, 622, 629, 635, 641, 647, 653, 659, 665, 671, 676, 682, 688,
        693, 699, 704, 710, 715, 720, 726, 731, 736, 742, 747, 752, 757, 762, 767, 772,
        777, 782, 787, 791, 796, 801, 806, 810, 815, 820, 824, 829, 834, 838, 843, 847,
        852, 856, 860, 865, 869, 873, 878, 882, 886, 891, 895, 899, 903, 907, 912, 916,
        920, 924, 928, 932, 936, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940,
        64, 64, 64, 64, 64, 64, 64, 64, 84, 107, 126, 143, 158, 172, 185, 198,
        210, 222, 234, 245, 256, 266, 277, 287, 297, 307, 316, 326, 335, 344, 353, 362,
        371, 380, 388, 397, 405, 414, 422, 430, 438, 446, 454, 462, 470, 478, 485, 493,
        501, 508, 516, 523, 530, 538, 545, 552, 559, 567, 574, 581, 588, 595, 602, 608,
        615, 622, 629, 636, 642, 649, 656, 662, 669, 675, 682, 688, 695, 701, 708, 714,
        720, 727, 733, 739, 746, 752, 758, 764, 770, 776, 783, 789, 795, 801, 807, 813,
        819, 825, 831, 837, 842, 848, 854, 860, 866, 872, 877, 883, 889, 895, 900, 906,
        912, 917, 923, 929, 934, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940,
        64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 67, 69, 71, 74, 77, 80,
        83, 86, 90, 94, 98, 102, 106, 111, 115, 120, 125, 130, 135, 140, 146, 151,
        157, 162, 168, 174, 180, 186, 193, 199, 206, 212, 219, 226, 233, 240, 247, 254,
        261, 268, 276, 283, 291, 299, 306, 314, 322, 330, 338, 347, 355, 363, 372, 380,
        389, 397, 406, 415, 424, 433, 442, 451, 460, 469, 479, 488, 497, 507, 517, 526,
        536, 546, 556, 566, 576, 586, 596, 606, 616, 626, 637, 647, 658, 668, 679, 690,
        700, 711, 722, 733, 744, 755, 766, 777, 789, 800, 811, 823, 834, 846, 857, 869,
        881, 892, 904, 916, 928, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940,
        64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 65, 66, 66,
        67, 68, 69, 70, 71, 73, 74, 76, 78, 80, 82, 84, 86, 88, 91, 94,
        97, 99, 103, 106, 109, 113, 117, 121, 125, 129, 133, 138, 142, 147, 152, 157,
        162, 168, 173, 179, 185, 191, 197, 203, 210, 217, 224, 231, 238, 245, 253, 260,
        268, 276, 285, 293, 301, 310, 319, 328, 337, 347, 356, 366, 376, 386, 397, 407,
        418, 428, 439, 451, 462, 474, 485, 497, 509, 521, 534, 547, 559, 572, 585, 599,
        612, 626, 640, 654, 668, 683, 697, 712, 727, 743, 758, 774, 789, 805, 822, 838,
        854, 871, 888, 905, 922, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940,
        64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 65, 66, 68, 69, 72, 74,
        77, 80, 84, 88, 92, 96, 101, 107, 112, 118, 124, 131, 138, 145, 152, 160,
        168, 176, 185, 193, 202, 212, 221, 231, 241, 251, 262, 272, 283, 294, 305, 316,
        328, 339, 351, 363, 375, 387, 399, 412, 424, 436, 449, 461, 474, 486, 499, 511,
        524, 537, 549, 562, 574, 586, 599, 611, 623, 635, 647, 659, 670, 682, 693, 704,
        716, 726, 737, 748, 758, 768, 778, 788, 797, 806, 815, 824, 832, 840, 848, 856,
        863, 870, 877, 883, 889, 895, 900, 905, 910, 914, 918, 922, 925, 929, 931, 934,
        936, 937, 938, 939, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940,
        70, 70, 70, 70, 70, 70, 70, 70, 70, 71, 71, 72, 73, 74, 75, 76,
        77, 78, 79, 81, 82, 84, 86, 88, 90, 92, 95, 98, 101, 104, 108, 112,
        116, 121, 126, 131, 137, 143, 150, 158, 166, 174, 183, 193, 203, 214, 226, 238,
        251, 265, 280, 295, 311, 327, 344, 362, 380, 399, 418, 437, 457, 477, 497, 517,
        537, 557, 576, 596, 614, 633, 651, 668, 685, 701, 717, 732, 746, 759, 772, 784,
        795, 806, 816, 826, 834, 843, 850, 857, 864, 870, 876, 881, 886, 890, 894, 898,
        902, 905, 908, 910, 913, 915, 917, 919, 921, 923, 924, 926, 927, 928, 929, 930,
        931, 932, 932, 933, 934, 934, 934, 934, 934, 934, 934, 934, 934, 934, 934, 934,
        64, 64, 64, 64, 64, 64, 64, 64, 102, 129, 149, 164, 178, 190, 201, 212,
        221, 231, 240, 248, 256, 264, 272, 279, 286, 293, 300, 307, 314, 320, 326, 333,
        339, 345, 351, 357, 363, 368, 374, 380, 385, 391, 396, 402, 407, 413, 418, 423,
        429, 434, 439, 444, 449, 455, 460, 465, 470, 475, 480, 485, 491, 496, 501, 506,
        511, 516, 521, 526, 531, 536, 542, 547, 552, 557, 562, 568, 573, 578, 583, 589,
        594, 599, 605, 610, 616, 621, 627, 633, 639, 644, 650, 656, 662, 668, 674, 681,
        687, 694, 700, 707, 714, 721, 729, 736, 744, 752, 760, 769, 778, 787, 797, 808,
        820, 833, 847, 864, 887, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940,
        64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
        64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64,
        64, 64, 64, 66, 82, 98, 114, 130, 146, 162, 178, 194, 210, 226, 242, 258,
        274, 290, 306, 322, 338, 354, 370, 386, 402, 418, 434, 450, 466, 482, 498, 514,
        530, 546, 562, 578, 594, 610, 626, 642, 658, 674, 690, 706, 722, 738, 754, 770,
        786, 802, 818, 834, 850, 866, 882, 898, 914, 930, 940, 940, 940, 940, 940, 940,
        940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940,
        940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940, 940
    );

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_lring is array (0 to 31) of std_logic_vector(15 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal lring : t_lring := (others => x"8080");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_lring_q : std_logic_vector(15 downto 0) := x"8080";
    signal s_lring_w : unsigned(4 downto 0) := (others => '0');
    signal s_cv_addr : unsigned(9 downto 0) := (others => '0');
    signal s_cv_q, s_cv_q2 : unsigned(9 downto 0) := (others => '0');

    -- controls / per-frame
    signal s_k_curve, s_k_contrast, s_k_size, s_k_warp, s_k_bleed, s_k_lag : unsigned(9 downto 0);
    signal s_sw_curveinset, s_sw_centre, s_sw_bleedinset, s_sw_neg, s_sw_laginset : std_logic;
    signal s_p_amount : unsigned(9 downto 0);
    signal s_curve  : unsigned(2 downto 0) := (others => '0');
    signal s_con    : unsigned(7 downto 0) := to_unsigned(64, 8);     -- Q6 gain 0.25 .. 4
    signal s_ssh    : unsigned(1 downto 0) := "01";
    signal s_warp   : unsigned(7 downto 0) := (others => '0');
    signal s_bn     : unsigned(2 downto 0) := (others => '0');
    signal s_lag    : unsigned(4 downto 0) := (others => '0');
    signal s_amount : unsigned(7 downto 0) := (others => '0');
    signal s_iw, s_ih, s_px, s_py, s_cx, s_cy : unsigned(10 downto 0) := (others => '0');
    signal s_pxe, s_pye : unsigned(11 downto 0) := (others => '0');
    signal s_wphase, s_lphase : unsigned(15 downto 0) := (others => '0');
    signal s_vstep  : unsigned(2 downto 0) := "111";

    -- tracking / per-line
    signal s_rx, s_line, s_height, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_wpar, s_rbank, s_we : std_logic := '0';
    signal s_wr_x   : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_lstep  : unsigned(2 downto 0) := "111";
    signal s_sin_addr : unsigned(7 downto 0) := (others => '0');
    signal s_sin_q, s_sin_q2 : signed(9 downto 0) := (others => '0');
    signal s_wp     : signed(18 downto 0) := (others => '0');
    signal s_off_warp : signed(10 downto 0) := (others => '0');
    signal s_row_in, s_row_edge : std_logic := '0';

    -- address chain
    signal s_x1     : unsigned(10 downto 0) := (others => '0');
    signal s_di2    : signed(11 downto 0) := (others => '0');
    signal s_in2, s_edge2 : std_logic := '0';
    signal s_di3    : signed(12 downto 0) := (others => '0');
    signal s_x3     : unsigned(10 downto 0) := (others => '0');
    signal s_in3, s_edge3 : std_logic := '0';
    signal s_sel4   : signed(12 downto 0) := (others => '0');
    signal s_in4, s_edge4 : std_logic := '0';
    signal s_addr5  : unsigned(10 downto 0) := (others => '0');
    signal s_bg5, s_in5, s_edge5 : std_logic := '0';
    signal s_ind    : std_logic_vector(6 to 16) := (others => '0');
    signal s_edged  : std_logic_vector(6 to 16) := (others => '0');

    -- pixel path
    signal s_a_bg, s_a_rbank : std_logic := '0';
    signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := (others => '0');
    signal s_yc8    : signed(11 downto 0) := (others => '0');
    signal s_y8     : unsigned(9 downto 0) := (others => '0');
    signal s_qu, s_qv : unsigned(15 downto 0) := to_unsigned(512 * 64, 16);
    signal s_pc9    : signed(20 downto 0) := (others => '0');
    signal s_y9     : unsigned(9 downto 0) := (others => '0');
    signal s_u9, s_v9 : unsigned(9 downto 0) := (others => '0');
    signal s_ys10   : unsigned(6 downto 0) := (others => '0');
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := (others => '0');
    signal s_y11, s_y12, s_y13 : unsigned(9 downto 0) := (others => '0');
    signal s_u11, s_v11 : unsigned(9 downto 0) := (others => '0');
    signal s_d14    : signed(10 downto 0) := (others => '0');
    signal s_y14    : unsigned(9 downto 0) := (others => '0');
    signal s_pa15   : signed(19 downto 0) := (others => '0');
    signal s_y15    : unsigned(9 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := (others => '0');
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := (others => '0');
    type t_c is array (12 to 15) of unsigned(9 downto 0);
    signal s_ud, s_vd : t_c := (others => (others => '0'));

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_curve    <= unsigned(registers_in(0));
    s_k_contrast <= unsigned(registers_in(1));
    s_k_size     <= unsigned(registers_in(2));
    s_k_warp     <= unsigned(registers_in(3));
    s_k_bleed    <= unsigned(registers_in(4));
    s_k_lag      <= unsigned(registers_in(5));
    s_sw_curveinset <= registers_in(6)(0);
    s_sw_centre     <= registers_in(6)(1);
    s_sw_bleedinset <= registers_in(6)(2);
    s_sw_neg        <= registers_in(6)(3);
    s_sw_laginset   <= registers_in(6)(4);
    s_p_amount   <= unsigned(registers_in(7));

    p_roms : process(clk)
    begin
        if rising_edge(clk) then
            s_sin_q  <= to_signed(C_SINE(to_integer(s_sin_addr)), 10);
            s_sin_q2 <= s_sin_q;
            s_cv_q   <= to_unsigned(C_CURVE(to_integer(s_cv_addr)), 10);
            s_cv_q2  <= s_cv_q;
        end if;
    end process;

    -- chroma lag ring: written at stage 10, read 1+lag back
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_lring_w <= s_lring_w + 1;
            s_lring_q <= lring(to_integer(s_lring_w - 1 - s_lag));
            lring(to_integer(s_lring_w)) <= std_logic_vector(s_u10(9 downto 2)) & std_logic_vector(s_v10(9 downto 2));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;

            -- per-line: inset rows, warp
            case to_integer(s_lstep) is
                when 0 =>
                    s_sin_addr <= s_lphase(15 downto 8);
                    if s_line >= s_py and resize(s_line, 12) < s_pye then s_row_in <= '1'; else s_row_in <= '0'; end if;
                    if s_line = s_py or resize(s_line, 12) = s_pye - 1 then s_row_edge <= '1'; else s_row_edge <= '0'; end if;
                    s_lstep <= s_lstep + 1;
                when 1 | 2 => s_lstep <= s_lstep + 1;
                when 3 =>
                    s_wp <= s_sin_q2 * signed('0' & s_warp);
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    s_off_warp <= resize(shift_right(s_wp, 8), 11);
                    s_lstep <= "111";
                when others => null;
            end case;

            -- per-frame decode
            case to_integer(s_vstep) is
                when 0 =>
                    s_curve <= s_k_curve(9 downto 7);
                    s_con   <= resize(s_k_contrast(9 downto 2), 8) + 16;        -- Q6: 0.25 .. 4.2
                    if s_k_size(9 downto 8) = 0 then s_ssh <= "01"; elsif s_k_size(9 downto 8) = 3 then s_ssh <= "11"; else s_ssh <= "10"; end if;
                    s_warp  <= s_k_warp(9 downto 2);
                    s_bn    <= s_k_bleed(9 downto 7);
                    s_lag   <= s_k_lag(9 downto 5);
                    s_amount <= s_p_amount(9 downto 2);
                    s_cx <= '0' & s_line_width(10 downto 1);
                    s_cy <= '0' & s_height(10 downto 1);
                    s_wphase <= s_wphase + 700;
                    s_lphase <= s_wphase;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_iw <= shift_right(s_line_width, to_integer(s_ssh));
                    s_ih <= shift_right(s_height, to_integer(s_ssh));
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    if s_sw_centre = '1' then
                        s_px <= s_cx - ('0' & s_iw(10 downto 1));
                        s_py <= s_cy - ('0' & s_ih(10 downto 1));
                    else
                        s_px <= s_line_width - s_iw - 8;
                        s_py <= s_height - s_ih - 8;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_pxe <= resize(s_px, 12) + resize(s_iw, 12);
                    s_pye <= resize(s_py, 12) + resize(s_ih, 12);
                    s_vstep <= "111";
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1; s_height <= s_line + 1;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
                s_lphase <= s_lphase + 160;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0';
                if s_saw_active = '1' then s_vstep <= (others => '0'); end if;
                s_saw_active <= '0';
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Address chain (stages 1..5)
    --------------------------------------------------------------------------
    p_addr : process(clk)
    begin
        if rising_edge(clk) then
            s_x1 <= s_rx;
            -- stage 2
            s_di2 <= signed(resize(s_x1, 12)) - signed(resize(s_px, 12));
            if s_row_in = '1' and s_x1 >= s_px and resize(s_x1, 12) < s_pxe then s_in2 <= '1'; else s_in2 <= '0'; end if;
            if s_row_edge = '1' or s_x1 = s_px or resize(s_x1, 12) = s_pxe - 1 then s_edge2 <= '1'; else s_edge2 <= '0'; end if;
            -- stage 3: inset scale
            s_di3 <= shift_left(resize(s_di2, 13), to_integer(s_ssh)) + resize(s_off_warp, 13);
            s_x3 <= s_x1; s_in3 <= s_in2; s_edge3 <= s_edge2;
            -- (s_x3 lags one stage behind s_di3: re-align)
            -- stage 4: select
            if s_in3 = '1' then s_sel4 <= s_di3; else s_sel4 <= signed(resize(s_x3, 13)); end if;
            s_in4 <= s_in3; s_edge4 <= s_edge3;
            -- stage 5: range
            if s_sel4 < 0 or s_sel4 >= signed(resize(s_line_width, 13)) then s_bg5 <= '1'; else s_bg5 <= '0'; end if;
            s_addr5 <= unsigned(s_sel4(10 downto 0));
            s_in5 <= s_in4; s_edge5 <= s_edge4 and s_in4;
            s_ind(6) <= s_in5; s_edged(6) <= s_edge5;
            for i in 7 to 16 loop s_ind(i) <= s_ind(i - 1); s_edged(i) <= s_edged(i - 1); end loop;
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (written stage 1, read stage 5 -> data 6)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr5));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr5));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr5(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr5(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr5(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr5(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel path (stages 6..18)
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_n : integer range 0 to 7;
        variable v_t : signed(12 downto 0);
        variable v_b : std_logic;
    begin
        if rising_edge(clk) then
            -- stage 6 ctx / 7 fetch
            s_a_bg <= s_bg5; s_a_rbank <= s_rbank;
            if s_a_bg = '1' then
                s_y7 <= to_unsigned(64, 10); s_u7 <= to_unsigned(512, 10); s_v7 <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_y7 <= unsigned(s_rdY1); s_u7 <= unsigned(s_rdU1) & "00"; s_v7 <= unsigned(s_rdV1) & "00";
            else
                s_y7 <= unsigned(s_rdY0); s_u7 <= unsigned(s_rdU0) & "00"; s_v7 <= unsigned(s_rdV0) & "00";
            end if;
            -- stage 8: luma about mid (negative inset), bleed IIR on chroma
            if s_sw_neg = '1' and s_ind(7) = '1' then
                s_yc8 <= to_signed(502, 12) - signed(resize(s_y7, 12));
            else
                s_yc8 <= signed(resize(s_y7, 12)) - to_signed(502, 12);
            end if;
            s_y8 <= s_y7;
            v_n := to_integer(s_bn);
            if s_ind(7) = '1' then v_b := s_sw_bleedinset; else v_b := not s_sw_bleedinset; end if;
            if s_avid_sr(5) = '0' then
                s_qu <= s_u7 & "000000"; s_qv <= s_v7 & "000000";
            elsif v_b = '1' then
                s_qu <= s_qu - shift_right(s_qu, v_n) + shift_right(s_u7 & "000000", v_n);
                s_qv <= s_qv - shift_right(s_qv, v_n) + shift_right(s_v7 & "000000", v_n);
            else
                s_qu <= s_u7 & "000000"; s_qv <= s_v7 & "000000";
            end if;
            -- stage 9: contrast product
            s_pc9 <= s_yc8 * signed('0' & s_con);
            s_y9 <= s_y8; s_u9 <= s_qu(15 downto 6); s_v9 <= s_qv(15 downto 6);
            -- stage 10: curve index = contrast-stretched luma code / 8 (ROM spans codes 0..1023)
            v_t := resize(shift_right(s_pc9, 6), 13) + to_signed(502, 13);
            if v_t < 0 then s_ys10 <= (others => '0');
            elsif v_t > 1023 then s_ys10 <= to_unsigned(127, 7);
            else s_ys10 <= unsigned(v_t(9 downto 3)); end if;
            s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;
            -- stage 11: ROM address (q 12, q2 13), original rides; lag ring written at 10 -> q at 12 for lag 0
            s_cv_addr <= s_curve & s_ys10;
            s_y11 <= s_y10; s_y12 <= s_y11; s_y13 <= s_y12;
            s_u11 <= s_u10; s_v11 <= s_v10;
            if (s_ind(11) and s_sw_laginset) = '1' or (not s_ind(11) and not s_sw_laginset) = '1' then
                s_ud(12) <= unsigned(s_lring_q(15 downto 8)) & "00"; s_vd(12) <= unsigned(s_lring_q(7 downto 0)) & "00";
            else
                s_ud(12) <= s_u11; s_vd(12) <= s_v11;
            end if;
            for i in 13 to 15 loop s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            -- stage 14: curved - original (curve enabled by region)
            if (s_ind(13) and s_sw_curveinset) = '1' or (not s_ind(13) and not s_sw_curveinset) = '1' then
                s_d14 <= signed('0' & s_cv_q2) - signed('0' & s_y13);
            else
                s_d14 <= (others => '0');
            end if;
            s_y14 <= s_y13;
            -- stage 15: amount product
            s_pa15 <= s_d14 * signed('0' & s_amount);
            s_y15 <= s_y14;
            -- stage 16: apply, frame
            if s_edged(16) = '1' then
                s_y16 <= to_unsigned(768, 10); s_u16 <= to_unsigned(512, 10); s_v16 <= to_unsigned(512, 10);
            else
                s_y16 <= f_clamp10(signed(resize(s_y15, 13)) + resize(shift_right(s_pa15, 8), 13));
                s_u16 <= s_ud(15); s_v16 <= s_vd(15);
            end if;
            -- stage 17: gate
            if s_avid_sr(15) = '1' then
                s_y17 <= s_y16; s_u17 <= s_u16; s_v17 <= s_v16;
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            -- stage 18: output
            s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y18);
    data_out.u       <= std_logic_vector(s_u18);
    data_out.v       <= std_logic_vector(s_v18);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture tonecurve;
