-- Noisetone: catalogue combo #137 (seed 20260927) -- Perlin / value-noise
-- displacement + gamma curves / S-curve contrast.  Three knobs each; the
-- switches and the slider decide how they combine.
--
--   K1 Scale    noise cell size 4 .. 512 px
--   K2 Amount   displacement amplitude (up to +-255 px)
--   K3 Drift    noise field scroll speed, bipolar (centre = still)
--   K4 Curve    gamma 0.5, 0.7, 1.5, 2.2, soft S, hard S, inverse S, crush
--   K5 Contrast linear gain about the pivot before the curve (25 .. 420%)
--   K6 Pivot    grey level the contrast pivots about (0 .. 100%)
--
--   S7  Patchy  one exposure everywhere / the noise field shifts the pivot (patchy exposure)
--   S8  Displace all channels / luma only (colour stays put under the boiling luma)
--   S9  Stretch noise cells tall (vertical streaks) / wide (horizontal streaks)
--   S10 Field   smooth value noise / hard cells (chunked tears)
--   S11 Chroma  saturation flat / saturation follows the contrast gain
--   P12 Storm   amplitude and contrast climb together: 100% is a boiling high-contrast field
--
-- Edge clamp: reads that leave the line, or land in its C_EDGE blanking
-- columns, take the line's own edge pixel.  2-D value noise: per-line
-- hashed row keys, four add/xor-shift hashes per pixel, bilinear lerp with
-- Q7 fractions, driving a horizontal read of the dual-bank line buffer;
-- the fetched luma is contrast-stretched about a (noise-shifted) pivot
-- and mapped through an 8 x 128 entry EBR curve ROM.
-- Latency 28 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture noisetone of program_top is

    constant C_LAT  : integer := 28;
    constant C_SEED : unsigned(15 downto 0) := x"5A3C";
    constant C_SEED1 : unsigned(15 downto 0) := x"5A3D";

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

    function f_clamp511(v : signed) return signed is
    begin
        if v < -511 then
            return to_signed(-511, 11);
        elsif v > 511 then
            return to_signed(511, 11);
        else
            return resize(v, 11);
        end if;
    end function;

    function f_round1(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 5);
        return t xor shift_right(t, 6);
    end function;

    function f_round2(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 3);
        return t xor shift_right(t, 8);
    end function;

    function f_round3(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 7);
        t := t xor shift_right(t, 5);
        return t xor shift_left(t, 8);
    end function;

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
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_cv_addr : unsigned(9 downto 0) := (others => '0');
    signal s_cv_q, s_cv_q2 : unsigned(9 downto 0) := (others => '0');

    -- controls / per-frame
    signal s_k_scale, s_k_amount, s_k_drift, s_k_curve, s_k_con, s_k_pivot : unsigned(9 downto 0);
    signal s_sw_patchy, s_sw_lumaonly, s_sw_wide, s_sw_blocky, s_sw_satcon : std_logic;
    signal s_p_storm : unsigned(9 downto 0);
    signal s_k      : unsigned(3 downto 0) := to_unsigned(5, 4);      -- cell shift 2..9
    signal s_kx, s_ky : unsigned(3 downto 0) := to_unsigned(5, 4);    -- per axis 2..11
    signal s_amp0   : unsigned(8 downto 0) := (others => '0');
    signal s_amp    : unsigned(7 downto 0) := (others => '0');
    signal s_con0, s_con : unsigned(8 downto 0) := to_unsigned(80, 9);  -- Q6 gain
    signal s_con_a, s_con_b, s_con_c : unsigned(8 downto 0) := to_unsigned(80, 9);
    signal s_curve  : unsigned(2 downto 0) := (others => '0');
    signal s_pivot  : unsigned(9 downto 0) := to_unsigned(502, 10);
    signal s_d0, s_drift : signed(10 downto 0) := (others => '0');
    signal s_nphase : unsigned(15 downto 0) := (others => '0');
    signal s_vstep  : unsigned(2 downto 0) := "111";
    attribute keep : boolean;
    attribute keep of s_con_a : signal is true;
    attribute keep of s_con_b : signal is true;
    attribute keep of s_con_c : signal is true;

    -- tracking / per-line
    signal s_rx, s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_wpar, s_rbank, s_we : std_logic := '0';
    signal s_wr_x   : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_lstep  : unsigned(2 downto 0) := "111";
    signal s_ly     : unsigned(11 downto 0) := (others => '0');
    signal s_cy     : unsigned(9 downto 0) := (others => '0');
    signal s_fy     : unsigned(6 downto 0) := (others => '0');
    signal s_t0, s_t1 : unsigned(15 downto 0) := (others => '0');
    signal s_rk0, s_rk0p, s_rk1, s_rk1p : unsigned(15 downto 0) := (others => '0');

    -- noise pipeline
    signal s_x1     : unsigned(10 downto 0) := (others => '0');
    signal s_cx1    : unsigned(9 downto 0) := (others => '0');
    signal s_fx1    : unsigned(6 downto 0) := (others => '0');
    signal s_ka, s_kb, s_kc, s_kd : unsigned(15 downto 0) := (others => '0');
    signal s_ha3, s_hb3, s_hc3, s_hd3 : unsigned(15 downto 0) := (others => '0');
    signal s_ha4, s_hb4, s_hc4, s_hd4 : unsigned(15 downto 0) := (others => '0');
    signal s_ha5, s_hb5, s_hc5, s_hd5 : unsigned(15 downto 0) := (others => '0');
    signal s_a6, s_b6, s_c6, s_d6 : unsigned(7 downto 0) := (others => '0');
    signal s_a7, s_c7 : unsigned(7 downto 0) := (others => '0');
    signal s_dab7, s_dcd7 : signed(8 downto 0) := (others => '0');
    signal s_a8, s_c8 : unsigned(7 downto 0) := (others => '0');
    signal s_pab8, s_pcd8 : signed(16 downto 0) := (others => '0');
    signal s_top9, s_bot9 : unsigned(7 downto 0) := (others => '0');
    signal s_top10  : unsigned(7 downto 0) := (others => '0');
    signal s_dtb10  : signed(8 downto 0) := (others => '0');
    signal s_top11  : unsigned(7 downto 0) := (others => '0');
    signal s_ptb11  : signed(16 downto 0) := (others => '0');
    signal s_n12    : unsigned(7 downto 0) := (others => '0');
    signal s_nc13   : signed(8 downto 0) := (others => '0');
    signal s_d14    : signed(17 downto 0) := (others => '0');
    signal s_off15  : signed(9 downto 0) := (others => '0');
    type t_fx is array (2 to 7) of unsigned(6 downto 0);
    signal s_fxd    : t_fx := (others => (others => '0'));
    type t_xs is array (2 to 16) of unsigned(10 downto 0);
    signal s_xd     : t_xs := (others => (others => '0'));
    type t_n8 is array (13 to 19) of unsigned(7 downto 0);
    signal s_nd     : t_n8 := (others => (others => '0'));

    -- address / fetch / curve
    signal s_u16    : signed(12 downto 0) := (others => '0');
    signal s_addr17, s_caddr17 : unsigned(10 downto 0) := (others => '0');
    signal s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_piv20  : unsigned(9 downto 0) := (others => '0');
    signal s_y20    : unsigned(9 downto 0) := (others => '0');
    signal s_uc20, s_vc20 : signed(10 downto 0) := (others => '0');
    signal s_yc21   : signed(10 downto 0) := (others => '0');
    signal s_uc21, s_vc21 : signed(10 downto 0) := (others => '0');
    signal s_pu21, s_pv21 : signed(20 downto 0) := (others => '0');
    signal s_pc22   : signed(20 downto 0) := (others => '0');
    type t_c11 is array (22 to 26) of signed(10 downto 0);
    signal s_ucd, s_vcd : t_c11 := (others => (others => '0'));
    signal s_ys23   : unsigned(6 downto 0) := (others => '0');
    signal s_y27, s_u27, s_v27 : unsigned(9 downto 0) := (others => '0');
    signal s_y28, s_u28, s_v28 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');
    -- Edge clamp: reads that leave the line, or land within C_EDGE columns of
    -- either end (the capture's black blanking columns), are redirected to the
    -- first real column inside that inset -- the fill is the line's own edge
    -- pixel, so it continues the colour leading up to it with no black seam.
    constant C_EDGE : integer := 12;
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(200, 11);   -- line_width - 1 - C_EDGE
    function f_edge_addr(v : signed; we : unsigned(10 downto 0)) return unsigned is
    begin
        if v < C_EDGE then
            return to_unsigned(C_EDGE, 11);
        elsif v > signed(resize(we, v'length)) then
            return we;
        else
            return unsigned(v(10 downto 0));
        end if;
    end function;

begin

    s_k_scale  <= unsigned(registers_in(0));
    s_k_amount <= unsigned(registers_in(1));
    s_k_drift  <= unsigned(registers_in(2));
    s_k_curve  <= unsigned(registers_in(3));
    s_k_con    <= unsigned(registers_in(4));
    s_k_pivot  <= unsigned(registers_in(5));
    s_sw_patchy   <= registers_in(6)(0);
    s_sw_lumaonly <= registers_in(6)(1);
    s_sw_wide     <= registers_in(6)(2);
    s_sw_blocky   <= registers_in(6)(3);
    s_sw_satcon   <= registers_in(6)(4);
    s_p_storm  <= unsigned(registers_in(7));

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_cv_q  <= to_unsigned(C_CURVE(to_integer(s_cv_addr)), 10);
            s_cv_q2 <= s_cv_q;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control: tracking, per-line and per-frame sequencers
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_ky : integer range 2 to 11;
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;

            -- per-line sequencer: row cell index, fraction, hashed row keys
            v_ky := to_integer(s_ky);
            case to_integer(s_lstep) is
                when 0 =>
                    s_ly <= resize(s_line, 12) + resize(s_nphase(15 downto 4), 12);
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_cy <= resize(shift_right(s_ly, v_ky), 10);
                    case v_ky is
                        when 2 => s_fy <= s_ly(1 downto 0) & "00000";
                        when 3 => s_fy <= s_ly(2 downto 0) & "0000";
                        when 4 => s_fy <= s_ly(3 downto 0) & "000";
                        when 5 => s_fy <= s_ly(4 downto 0) & "00";
                        when 6 => s_fy <= s_ly(5 downto 0) & "0";
                        when 7 => s_fy <= s_ly(6 downto 0);
                        when 8 => s_fy <= s_ly(7 downto 1);
                        when 9 => s_fy <= s_ly(8 downto 2);
                        when 10 => s_fy <= s_ly(9 downto 3);
                        when others => s_fy <= s_ly(10 downto 4);
                    end case;
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    s_t0 <= C_SEED + resize(s_cy, 16);
                    s_t1 <= C_SEED1 + resize(s_cy, 16);
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    s_rk0 <= f_round1(s_t0);
                    s_rk1 <= f_round1(s_t1);
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    s_rk0p <= s_rk0 + 1;
                    s_rk1p <= s_rk1 + 1;
                    s_lstep <= "111";
                when others => null;
            end case;

            -- per-frame sequencer: decode, storm sums
            case to_integer(s_vstep) is
                when 0 =>
                    s_amp0 <= resize(s_k_amount(9 downto 2), 9) + resize(s_p_storm(9 downto 2), 9);
                    s_con0 <= to_unsigned(16, 9) + resize(s_k_con(9 downto 2), 9);       -- Q6: 0.25 .. 4.2
                    if s_k_scale(9 downto 7) = 7 then s_k <= to_unsigned(9, 4); else s_k <= resize(s_k_scale(9 downto 7), 4) + 2; end if;
                    s_curve <= s_k_curve(9 downto 7);
                    s_pivot <= s_k_pivot;
                    s_d0 <= signed(resize(s_k_drift, 11)) - 512;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    if s_amp0(8) = '1' then s_amp <= x"FF"; else s_amp <= s_amp0(7 downto 0); end if;
                    s_con <= s_con0 + resize(s_p_storm(9 downto 3), 9);                    -- up to 398 (6.2x)
                    if s_sw_wide = '1' then s_kx <= s_k + 2; s_ky <= s_k; else s_kx <= s_k; s_ky <= s_k + 2; end if;
                    if s_d0 > -8 and s_d0 < 8 then s_drift <= (others => '0'); else s_drift <= shift_right(s_d0, 1); end if;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_con_a <= s_con; s_con_b <= s_con; s_con_c <= s_con;
                    s_vstep <= "111";
                when others => null;
            end case;

            -- line / frame events
            s_w_edge <= s_line_width - (C_EDGE + 1);
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
                s_line <= s_line + 1;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0';
                if s_saw_active = '1' then
                    s_nphase <= s_nphase + unsigned(resize(s_drift, 16));
                    s_vstep <= (others => '0');
                end if;
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
    -- Noise + address pipeline (stages 1..17)
    --------------------------------------------------------------------------
    p_noise : process(clk)
        variable v_kx : integer range 2 to 11;
        variable v_t, v_b : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            v_kx := to_integer(s_kx);
            -- stage 1: cell index and Q7 fraction
            s_x1  <= s_rx;
            s_cx1 <= resize(shift_right(s_rx, v_kx), 10);
            case v_kx is
                when 2 => s_fx1 <= s_rx(1 downto 0) & "00000";
                when 3 => s_fx1 <= s_rx(2 downto 0) & "0000";
                when 4 => s_fx1 <= s_rx(3 downto 0) & "000";
                when 5 => s_fx1 <= s_rx(4 downto 0) & "00";
                when 6 => s_fx1 <= s_rx(5 downto 0) & "0";
                when 7 => s_fx1 <= s_rx(6 downto 0);
                when 8 => s_fx1 <= s_rx(7 downto 1);
                when 9 => s_fx1 <= s_rx(8 downto 2);
                when 10 => s_fx1 <= s_rx(9 downto 3);
                when others => s_fx1 <= s_rx(10 downto 4);
            end case;
            -- stage 2: lattice keys; screen x
            s_ka <= s_rk0  + resize(s_cx1, 16);
            s_kb <= s_rk0p + resize(s_cx1, 16);
            s_kc <= s_rk1  + resize(s_cx1, 16);
            s_kd <= s_rk1p + resize(s_cx1, 16);
            s_xd(2) <= s_x1;
            for i in 3 to 16 loop s_xd(i) <= s_xd(i - 1); end loop;
            if s_sw_blocky = '1' then s_fxd(2) <= (others => '0'); else s_fxd(2) <= s_fx1; end if;
            for i in 3 to 7 loop s_fxd(i) <= s_fxd(i - 1); end loop;
            -- stages 3..5: hash rounds
            s_ha3 <= f_round1(s_ka); s_hb3 <= f_round1(s_kb); s_hc3 <= f_round1(s_kc); s_hd3 <= f_round1(s_kd);
            s_ha4 <= f_round2(s_ha3); s_hb4 <= f_round2(s_hb3); s_hc4 <= f_round2(s_hc3); s_hd4 <= f_round2(s_hd3);
            s_ha5 <= f_round3(s_ha4); s_hb5 <= f_round3(s_hb4); s_hc5 <= f_round3(s_hc4); s_hd5 <= f_round3(s_hd4);
            -- stage 6: corner values
            s_a6 <= s_ha5(15 downto 8); s_b6 <= s_hb5(15 downto 8); s_c6 <= s_hc5(15 downto 8); s_d6 <= s_hd5(15 downto 8);
            -- stage 7: x differences
            s_dab7 <= signed('0' & s_b6) - signed('0' & s_a6);
            s_dcd7 <= signed('0' & s_d6) - signed('0' & s_c6);
            s_a7 <= s_a6; s_c7 <= s_c6;
            -- stage 8: x lerp products
            s_pab8 <= s_dab7 * signed('0' & s_fxd(7));
            s_pcd8 <= s_dcd7 * signed('0' & s_fxd(7));
            s_a8 <= s_a7; s_c8 <= s_c7;
            -- stage 9: top / bottom row values
            v_t := signed(resize(s_a8, 10) & '0') + resize(shift_right(s_pab8, 6), 11);
            v_b := signed(resize(s_c8, 10) & '0') + resize(shift_right(s_pcd8, 6), 11);
            s_top9 <= unsigned(v_t(8 downto 1));
            s_bot9 <= unsigned(v_b(8 downto 1));
            -- stage 10: y difference
            s_dtb10 <= signed('0' & s_bot9) - signed('0' & s_top9);
            s_top10 <= s_top9;
            -- stage 11: y lerp product
            if s_sw_blocky = '1' then
                s_ptb11 <= (others => '0');
            else
                s_ptb11 <= s_dtb10 * signed('0' & s_fy);
            end if;
            s_top11 <= s_top10;
            -- stage 12: noise value
            v_t := signed(resize(s_top11, 10) & '0') + resize(shift_right(s_ptb11, 6), 11);
            s_n12 <= unsigned(v_t(8 downto 1));
            -- stage 13: centred; noise rides along to the curve stage
            s_nc13 <= signed('0' & s_n12) - to_signed(128, 9);
            s_nd(13) <= s_n12;
            for i in 14 to 19 loop s_nd(i) <= s_nd(i - 1); end loop;
            -- stage 14: amplitude
            s_d14 <= s_nc13 * signed('0' & s_amp);
            -- stage 15: displacement
            s_off15 <= s_d14(16 downto 7);
            -- stage 16: source x
            s_u16 <= signed(resize(s_xd(15), 13)) + resize(s_off15, 13);
            -- stage 17: addresses (luma displaced; chroma displaced or not)
            s_addr17 <= f_edge_addr(s_u16, s_w_edge);
            if s_sw_lumaonly = '1' then s_caddr17 <= s_xd(16); else s_caddr17 <= f_edge_addr(s_u16, s_w_edge); end if;
        end if;
    end process p_noise;

    --------------------------------------------------------------------------
    -- Line buffers: written at stage 1 (s_in), read at stage 17 -> data 18
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr17));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr17));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_caddr17(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_caddr17(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_caddr17(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_caddr17(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Fetch + tone curve (stages 18..28)
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_p : signed(11 downto 0);
        variable v_n : signed(7 downto 0);
        variable v_i : signed(12 downto 0);
        variable v_u, v_v : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 18 ctx / 19 fetch
            s_a_rbank <= s_rbank;
            if s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- stage 20: pivot (noise-shifted when patchy), centred chroma
            v_n := signed(s_nd(19) xor x"80");                                  -- noise - 128
            if s_sw_patchy = '1' then
                v_p := signed(resize(s_pivot, 12)) + shift_left(resize(v_n, 12), 1);
            else
                v_p := signed(resize(s_pivot, 12));
            end if;
            s_piv20 <= f_clamp10(v_p);
            s_y20 <= s_wet_y;
            s_uc20 <= signed(resize(s_wet_u, 11)) - 512;
            s_vc20 <= signed(resize(s_wet_v, 11)) - 512;
            -- stage 21: luma about the pivot; chroma contrast products
            s_yc21 <= signed(resize(s_y20, 11)) - signed(resize(s_piv20, 11));
            s_pu21 <= s_uc20 * signed('0' & s_con_b);
            s_pv21 <= s_vc20 * signed('0' & s_con_c);
            s_uc21 <= s_uc20; s_vc21 <= s_vc20;
            -- stage 22: luma contrast product; chroma select
            s_pc22 <= s_yc21 * signed('0' & s_con_a);
            if s_sw_satcon = '1' then
                s_ucd(22) <= f_clamp511(shift_right(s_pu21, 6));
                s_vcd(22) <= f_clamp511(shift_right(s_pv21, 6));
            else
                s_ucd(22) <= s_uc21; s_vcd(22) <= s_vc21;
            end if;
            -- stage 23: curve index = stretched luma code / 8 (ROM spans codes 0..1023)
            v_i := resize(shift_right(s_pc22, 6), 13) + to_signed(502, 13);
            if v_i < 0 then s_ys23 <= (others => '0');
            elsif v_i > 1023 then s_ys23 <= to_unsigned(127, 7);
            else s_ys23 <= unsigned(v_i(9 downto 3)); end if;
            -- stage 24: ROM address (q 25, q2 26)
            s_cv_addr <= s_curve & s_ys23;
            -- stage 27: curved luma, recentred chroma
            s_y27 <= s_cv_q2;
            v_u := resize(s_ucd(26), 12) + 512;
            v_v := resize(s_vcd(26), 12) + 512;
            s_u27 <= unsigned(v_u(9 downto 0));
            s_v27 <= unsigned(v_v(9 downto 0));
            -- stage 28: gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y28 <= s_y27; s_u28 <= s_u27; s_v28 <= s_v27;
            else
                s_y28 <= to_unsigned(64, 10); s_u28 <= to_unsigned(512, 10); s_v28 <= to_unsigned(512, 10);
            end if;

            for i in 23 to 26 loop s_ucd(i) <= s_ucd(i - 1); s_vcd(i) <= s_vcd(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(s_u28);
    data_out.v       <= std_logic_vector(s_v28);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture noisetone;
