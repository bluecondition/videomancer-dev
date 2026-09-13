-- Outliner: catalogue combo #13 (seed 20260914) -- edge detect (Roberts /
-- wide gradient / difference-of-boxes) + split-screen + invert / negative /
-- bit-plane.  Two knobs each; the switches and the slider decide how they
-- combine.
--
--   K1 Threshold edge threshold
--   K2 Kernel    1-px gradient / 4-px gradient / difference of 5- and 17-px boxes
--   K3 Seam      split position
--   K4 Panes     1 / 2 / 4 / 8 vertical strips alternating processed / original
--   K5 Plane     negative / bit-planes 8..2
--   K6 Mix       negative or plane mixed over the picture
--
--   S7  Edges    white lines on black / lines drawn over the picture
--   S8  Right    the "original" side is inverted instead
--   S9  Sides    edges on the processed side only / both sides
--   S10 Invert   inversion applies to edges only
--   S11 Colour   edge lines take the source chroma / are white
--   P12 Sweep    the seam swings on a sine, speed = slider
--
-- Pure per-pixel (17-tap luma window).  Latency 12 clocks, all modes,
-- blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture outliner of program_top is

    constant C_LAT : integer := 12;

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
        -255, -255, -255, -254, -254, -253, -252, -251, -250, -249, -247, -246, -244, -242, -240, -238,
        -236, -233, -231, -228, -225, -222, -219, -215, -212, -208, -205, -201, -197, -193, -189, -185,
        -180, -176, -171, -167, -162, -157, -152, -147, -142, -136, -131, -126, -120, -115, -109, -103,
         -98,  -92,  -86,  -80,  -74,  -68,  -62,  -56,  -50,  -44,  -37,  -31,  -25,  -19,  -13,   -6,
           0,    6,   13,   19,   25,   31,   37,   44,   50,   56,   62,   68,   74,   80,   86,   92,
          98,  103,  109,  115,  120,  126,  131,  136,  142,  147,  152,  157,  162,  167,  171,  176,
         180,  185,  189,  193,  197,  201,  205,  208,  212,  215,  219,  222,  225,  228,  231,  233,
         236,  238,  240,  242,  244,  246,  247,  249,  250,  251,  252,  253,  254,  254,  255,  255,
         255,  255,  255,  254,  254,  253,  252,  251,  250,  249,  247,  246,  244,  242,  240,  238,
         236,  233,  231,  228,  225,  222,  219,  215,  212,  208,  205,  201,  197,  193,  189,  185,
         180,  176,  171,  167,  162,  157,  152,  147,  142,  136,  131,  126,  120,  115,  109,  103,
          98,   92,   86,   80,   74,   68,   62,   56,   50,   44,   37,   31,   25,   19,   13,    6,
           0,   -6,  -13,  -19,  -25,  -31,  -37,  -44,  -50,  -56,  -62,  -68,  -74,  -80,  -86,  -92,
         -98, -103, -109, -115, -120, -126, -131, -136, -142, -147, -152, -157, -162, -167, -171, -176,
        -180, -185, -189, -193, -197, -201, -205, -208, -212, -215, -219, -222, -225, -228, -231, -233,
        -236, -238, -240, -242, -244, -246, -247, -249, -250, -251, -252, -253, -254, -254, -255, -255
    );

    signal s_k_thr, s_k_kern, s_k_seam, s_k_panes, s_k_plane, s_k_mix : unsigned(9 downto 0);
    signal s_sw_over, s_sw_rightinv, s_sw_both, s_sw_invedges, s_sw_colour : std_logic;
    signal s_p_sweep : unsigned(9 downto 0);
    signal s_thr    : unsigned(9 downto 0) := (others => '0');
    signal s_kern   : unsigned(1 downto 0) := (others => '0');
    signal s_pshift : unsigned(1 downto 0) := (others => '0');
    signal s_plane  : unsigned(2 downto 0) := (others => '0');
    signal s_mix    : unsigned(7 downto 0) := (others => '0');
    signal s_seam_px : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_pane_w : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_rx     : unsigned(10 downto 0) := (others => '0');
    signal s_prev_avid, s_prev_vsync_n : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_vstep  : unsigned(2 downto 0) := "111";
    signal s_axp    : unsigned(20 downto 0) := (others => '0');
    signal s_swp    : signed(20 downto 0) := (others => '0');
    signal s_sweep_ph : unsigned(15 downto 0) := (others => '0');
    signal s_sin_addr : unsigned(7 downto 0) := (others => '0');
    signal s_sin_q  : signed(9 downto 0) := (others => '0');
    signal s_sin_q2 : signed(9 downto 0) := (others => '0');

    type t_taps is array (0 to 16) of unsigned(9 downto 0);
    signal s_t, s_cu, s_cv : t_taps := (others => (others => '0'));
    signal s_x1     : unsigned(10 downto 0) := (others => '0');
    signal s_acc5   : unsigned(12 downto 0) := (others => '0');
    signal s_acc17  : unsigned(14 downto 0) := (others => '0');
    -- stage 2
    signal s_g2     : signed(11 downto 0) := (others => '0');
    signal s_y2, s_u2, s_v2 : unsigned(9 downto 0) := (others => '0');
    signal s_x2     : unsigned(10 downto 0) := (others => '0');
    signal s_pc     : unsigned(10 downto 0) := (others => '0');
    signal s_pi     : unsigned(3 downto 0) := (others => '0');
    -- stage 3
    signal s_edge3  : std_logic := '0';
    signal s_proc3  : std_logic := '0';
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := (others => '0');
    -- stage 4
    signal s_pl4    : unsigned(9 downto 0) := (others => '0');
    signal s_y4, s_u4, s_v4 : unsigned(9 downto 0) := (others => '0');
    signal s_edge4, s_proc4 : std_logic := '0';
    -- stage 5
    signal s_e5     : signed(10 downto 0) := (others => '0');
    signal s_y5, s_u5, s_v5 : unsigned(9 downto 0) := (others => '0');
    signal s_edge5, s_proc5 : std_logic := '0';
    -- stage 6
    signal s_pe6    : signed(19 downto 0) := (others => '0');
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := (others => '0');
    signal s_edge6, s_proc6 : std_logic := '0';
    -- stage 7..12
    signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := (others => '0');
    signal s_edge7, s_proc7 : std_logic := '0';
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := (others => '0');
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := (others => '0');
    type t_y is array (8 to 11) of unsigned(9 downto 0);
    signal s_yd, s_ud, s_vd : t_y := (others => (others => '0'));

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_thr   <= unsigned(registers_in(0));
    s_k_kern  <= unsigned(registers_in(1));
    s_k_seam  <= unsigned(registers_in(2));
    s_k_panes <= unsigned(registers_in(3));
    s_k_plane <= unsigned(registers_in(4));
    s_k_mix   <= unsigned(registers_in(5));
    s_sw_over      <= registers_in(6)(0);
    s_sw_rightinv  <= registers_in(6)(1);
    s_sw_both      <= registers_in(6)(2);
    s_sw_invedges  <= registers_in(6)(3);
    s_sw_colour    <= registers_in(6)(4);
    s_p_sweep <= unsigned(registers_in(7));

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin_q <= to_signed(C_SINE(to_integer(s_sin_addr)), 10);
            s_sin_q2 <= s_sin_q;
        end if;
    end process;

    p_frame : process(clk)
        variable v_a : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;
            s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_prev_avid = '1' and data_in.avid = '0' then s_line_width <= s_rx; end if;
            s_thr    <= s_k_thr;
            s_kern   <= s_k_kern(9 downto 8);
            s_pshift <= s_k_panes(9 downto 8);
            s_plane  <= s_k_plane(9 downto 7);
            s_mix    <= s_k_mix(9 downto 2);
            case to_integer(s_vstep) is
                when 0 =>
                    s_axp <= s_k_seam * s_line_width;
                    s_sin_addr <= s_sweep_ph(15 downto 8);
                    s_vstep <= s_vstep + 1;
                when 1 | 2 => s_vstep <= s_vstep + 1;
                when 3 =>
                    s_swp <= s_sin_q2 * signed('0' & s_p_sweep);
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    v_a := signed(resize(s_axp(20 downto 10), 14)) + resize(shift_right(s_swp, 11), 14);
                    if v_a < 0 then v_a := (others => '0'); end if;
                    if v_a > signed(resize(s_line_width, 14)) then v_a := signed(resize(s_line_width, 14)); end if;
                    s_seam_px <= unsigned(v_a(10 downto 0));
                    case to_integer(s_pshift) is
                        when 0 => s_pane_w <= s_line_width;
                        when 1 => s_pane_w <= '0' & s_line_width(10 downto 1);
                        when 2 => s_pane_w <= "00" & s_line_width(10 downto 2);
                        when others => s_pane_w <= "000" & s_line_width(10 downto 3);
                    end case;
                    s_vstep <= "111";
                when others => null;
            end case;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                if s_saw_active = '1' then
                    s_sweep_ph <= s_sweep_ph + resize(s_p_sweep(9 downto 3), 16);
                    s_vstep <= (others => '0');
                end if;
                s_saw_active <= '0';
            end if;
        end if;
    end process;

    p_pix : process(clk)
        variable v_g : signed(11 downto 0);
        variable v_pl : unsigned(9 downto 0);
        variable v_proc : std_logic;
        variable v_rp : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 1: taps and running box sums (5 and 17 wide)
            s_t(0) <= unsigned(data_in.y); s_cu(0) <= unsigned(data_in.u); s_cv(0) <= unsigned(data_in.v);
            for i in 1 to 16 loop
                s_t(i) <= s_t(i - 1); s_cu(i) <= s_cu(i - 1); s_cv(i) <= s_cv(i - 1);
            end loop;
            s_x1 <= s_rx;
            if data_in.avid = '0' then
                s_acc5  <= to_unsigned(64 * 5, 13);
                s_acc17 <= to_unsigned(64 * 17, 15);
            else
                s_acc5  <= s_acc5  + resize(unsigned(data_in.y), 13) - resize(s_t(4), 13);
                s_acc17 <= s_acc17 + resize(unsigned(data_in.y), 15) - resize(s_t(16), 15);
            end if;
            -- stage 2: gradient by kernel, pane counters
            case to_integer(s_kern) is
                when 0 => v_g := signed(resize(s_t(7), 12)) - signed(resize(s_t(9), 12));
                when 1 => v_g := signed(resize(s_t(4), 12)) - signed(resize(s_t(12), 12));
                when others =>
                    -- difference of boxes: 17-box (/17 ~ /16) minus 5-box (/5 ~ /4)... keep it a ratio-free approx
                    v_g := signed(resize(s_acc17(14 downto 4), 12)) - signed(resize(s_acc5(12 downto 2), 12)) + 16;
            end case;
            s_g2 <= v_g;
            s_y2 <= s_t(8); s_u2 <= s_cu(8); s_v2 <= s_cv(8);
            s_x2 <= s_x1;
            if s_avid_sr(0) = '0' then
                s_pc <= (others => '0'); s_pi <= (others => '0');
            elsif s_pc >= s_pane_w - 1 then
                s_pc <= (others => '0'); s_pi <= s_pi + 1;
            else
                s_pc <= s_pc + 1;
            end if;
            -- stage 3: edge flag; processed-side flag (right of seam, even panes)
            if s_g2 > signed('0' & s_thr) or s_g2 < -signed('0' & s_thr) then s_edge3 <= '1'; else s_edge3 <= '0'; end if;
            if s_pshift = 0 then
                if s_x2 >= s_seam_px then v_proc := '1'; else v_proc := '0'; end if;
            else
                if s_x2 >= s_seam_px then v_proc := not s_pi(0); else v_proc := s_pi(0); end if;
            end if;
            s_proc3 <= v_proc;
            s_y3 <= s_y2; s_u3 <= s_u2; s_v3 <= s_v2;
            -- stage 4: plane / negative candidate
            case to_integer(s_plane) is
                when 0 => v_pl := not s_y3;
                when others =>
                    if s_y3(9 - to_integer(s_plane)) = '1' then v_pl := to_unsigned(940, 10); else v_pl := to_unsigned(64, 10); end if;
            end case;
            s_pl4 <= v_pl;
            s_y4 <= s_y3; s_u4 <= s_u3; s_v4 <= s_v3;
            s_edge4 <= s_edge3; s_proc4 <= s_proc3;
            -- stage 5: inversion difference, gated by side / edges-only rules
            if (s_sw_rightinv = '1' and s_proc4 = '0') or (s_sw_rightinv = '0' and s_proc4 = '1') then
                if s_sw_invedges = '0' or s_edge4 = '1' then
                    s_e5 <= signed('0' & s_pl4) - signed('0' & s_y4);
                else
                    s_e5 <= (others => '0');
                end if;
            else
                s_e5 <= (others => '0');
            end if;
            s_y5 <= s_y4; s_u5 <= s_u4; s_v5 <= s_v4;
            s_edge5 <= s_edge4; s_proc5 <= s_proc4;
            -- stage 6: mix product
            s_pe6 <= s_e5 * signed('0' & s_mix);
            s_y6 <= s_y5; s_u6 <= s_u5; s_v6 <= s_v5;
            s_edge6 <= s_edge5; s_proc6 <= s_proc5;
            -- stage 7: apply inversion mix
            s_y7 <= f_clamp10(signed(resize(s_y6, 13)) + resize(shift_right(s_pe6, 8), 13));
            s_u7 <= s_u6; s_v7 <= s_v6;
            s_edge7 <= s_edge6; s_proc7 <= s_proc6;
            -- stage 8: edges on the processed side (or both): lines on black, or over the picture
            if s_edge7 = '1' and (s_proc7 = '1' or s_sw_both = '1') then
                s_y8 <= to_unsigned(940, 10);
                if s_sw_colour = '1' then s_u8 <= s_u7; s_v8 <= s_v7; else s_u8 <= to_unsigned(512, 10); s_v8 <= to_unsigned(512, 10); end if;
            elsif s_proc7 = '1' and s_sw_over = '0' then
                s_y8 <= to_unsigned(64, 10); s_u8 <= to_unsigned(512, 10); s_v8 <= to_unsigned(512, 10);
            else
                s_y8 <= s_y7; s_u8 <= s_u7; s_v8 <= s_v7;
            end if;
            s_yd(8) <= s_y8; s_ud(8) <= s_u8; s_vd(8) <= s_v8;
            for i in 9 to 11 loop
                s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1);
            end loop;
            -- stage 12: gate
            if s_avid_sr(10) = '1' then
                s_y12 <= s_yd(11); s_u12 <= s_ud(11); s_v12 <= s_vd(11);
            else
                s_y12 <= to_unsigned(64, 10); s_u12 <= to_unsigned(512, 10); s_v12 <= to_unsigned(512, 10);
            end if;
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y12);
    data_out.u       <= std_logic_vector(s_u12);
    data_out.v       <= std_logic_vector(s_v12);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture outliner;
