-- Ridgeline: catalogue combo #98 (seed 20260924) -- Perlin / simplex
-- displacement + edge detect (Roberts / gradient).  Three knobs each; the
-- switches and the slider decide how they combine.
--
--   K1 Amount   noise displacement amplitude (+-255 px)
--   K2 Scale    noise frequency across x
--   K3 Speed    noise drift speed
--   K4 Gain     edge gain
--   K5 Edges    Roberts / horizontal / vertical / Sobel-lite
--   K6 Ink      edge ink luma (0 = black lines .. 100 = white lines)
--
--   S7  Edges   drawn over the displaced picture / edges only on black
--   S8  Order   edges of the displaced picture / displaced edges of the original
--   S9  Chroma  chroma follows luma / chroma on its own noise phase
--   S10 Edges   edges also push the noise (edge magnitude adds to the offset)
--   S11 Octaves one / two octaves
--   P12 Rows    noise frequency down the lines (0 = pure column waves)
--
-- Sine-ROM noise into the line-buffer read address; Roberts cross against
-- the input line.  Latency 19 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture ridgeline of program_top is

    constant C_LAT : integer := 19;

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

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    signal s_k_amp, s_k_scale, s_k_speed, s_k_gain, s_k_mode, s_k_ink : unsigned(9 downto 0);
    signal s_sw_onblack, s_sw_edgefirst, s_sw_csep, s_sw_push, s_sw_two : std_logic;
    signal s_p_rows : unsigned(9 downto 0);

    signal s_amp, s_f1, s_f2, s_gain, s_rowf : unsigned(7 downto 0) := (others => '0');
    signal s_mode : unsigned(1 downto 0) := "00";
    signal s_ink : unsigned(9 downto 0) := (others => '0');
    signal s_fph1, s_fph2, s_lph1, s_lph2 : unsigned(15 downto 0) := (others => '0');
    signal s_saw_active : std_logic := '0';

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_rbank, s_we : std_logic := '0';

    -- input-side gradient (for "edges first": edges are written to the buffer)
    signal s_ym2, s_gx2 : unsigned(9 downto 0) := (others => '0');
    signal s_eg3 : unsigned(17 downto 0) := (others => '0');
    signal s_ui3, s_vi3 : unsigned(9 downto 0) := (others => '0');
    signal s_wy4, s_wu4, s_wv4 : unsigned(9 downto 0) := (others => '0');
    signal s_wx2, s_wx3, s_wx4 : unsigned(10 downto 0) := (others => '0');
    signal s_we2, s_we3, s_we4, s_wp2, s_wp3, s_wp4 : std_logic := '0';
    type t_x is array (1 to 12) of unsigned(10 downto 0);
    signal s_x : t_x := (others => (others => '0'));
    type t_b is array (1 to 12) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_ph1, s_ph2 : unsigned(15 downto 0) := (others => '0');
    signal s_sa1, s_sa2, s_ca1, s_ca2 : unsigned(7 downto 0) := (others => '0');
    signal s_q1, s_q2, s_qc1, s_qc2, s_r1, s_r2, s_rc1, s_rc2 : signed(9 downto 0) := (others => '0');
    signal s_n6, s_nc6 : signed(10 downto 0) := (others => '0');
    signal s_pn7, s_pnc7 : signed(19 downto 0) := (others => '0');
    type t_g is array (2 to 7) of unsigned(9 downto 0);
    signal s_gxd : t_g := (others => (others => '0'));
    signal s_aY8, s_aC8 : signed(12 downto 0) := (others => '0');
    signal s_addrY, s_addrC : unsigned(10 downto 0) := (others => '0');
    signal s_bgY, s_bgC : std_logic := '0';
    signal s_a_bgY, s_a_bgC, s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_pv12 : unsigned(9 downto 0) := (others => '0');
    type t_yc is array (2 to 12) of unsigned(9 downto 0);
    signal s_yc : t_yc := (others => (others => '0'));
    signal s_r1_13, s_r2_13, s_gh13, s_gv13 : unsigned(9 downto 0) := (others => '0');
    signal s_y13, s_u13, s_v13 : unsigned(9 downto 0) := (others => '0');
    signal s_e14 : unsigned(10 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');
    signal s_eg15 : unsigned(18 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17, s_y18, s_u18, s_v18, s_y19, s_u19, s_v19 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_amp   <= unsigned(registers_in(0));
    s_k_scale <= unsigned(registers_in(1));
    s_k_speed <= unsigned(registers_in(2));
    s_k_gain  <= unsigned(registers_in(3));
    s_k_mode  <= unsigned(registers_in(4));
    s_k_ink   <= unsigned(registers_in(5));
    s_sw_onblack   <= registers_in(6)(0);
    s_sw_edgefirst <= registers_in(6)(1);
    s_sw_csep      <= registers_in(6)(2);
    s_sw_push      <= registers_in(6)(3);
    s_sw_two       <= registers_in(6)(4);
    s_p_rows <= unsigned(registers_in(7));

    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;

            s_amp <= s_k_amp(9 downto 2);
            s_f1 <= "00" & s_k_scale(9 downto 4); s_f2 <= '0' & s_k_scale(9 downto 3);
            s_gain <= s_k_gain(9 downto 2); s_mode <= s_k_mode(9 downto 8);
            s_ink <= resize(s_k_ink(9 downto 0), 10);
            s_rowf <= s_p_rows(9 downto 2);

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
                s_lph1 <= s_lph1 + resize(s_rowf, 16);
                s_lph2 <= s_lph2 - resize(s_rowf & '0', 16);
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then s_wpar <= not s_wpar; s_rbank <= s_wpar; end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0';
                if s_saw_active = '1' then
                    s_fph1 <= s_fph1 + resize(s_k_speed(9 downto 3) & "00", 16);
                    s_fph2 <= s_fph2 + resize(s_k_speed(9 downto 3) & "000", 16) + 37;
                end if;
                s_lph1 <= s_fph1; s_lph2 <= s_fph2;
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

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_q1 <= to_signed(C_SINE(to_integer(s_sa1)), 10);
            s_q2 <= to_signed(C_SINE(to_integer(s_sa2)), 10);
            s_qc1 <= to_signed(C_SINE(to_integer(s_ca1)), 10);
            s_qc2 <= to_signed(C_SINE(to_integer(s_ca2)), 10);
            s_r1 <= s_q1; s_r2 <= s_q2; s_rc1 <= s_qc1; s_rc2 <= s_qc2;
        end if;
    end process;

    -- input side: gradient of the incoming line, optional edge write (edges first)
    p_win : process(clk)
        variable v_e : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_ym2 <= s_in_y;
            if s_in_y >= s_ym2 then s_gx2 <= s_in_y - s_ym2; else s_gx2 <= s_ym2 - s_in_y; end if;
            s_wx2 <= s_wr_x; s_we2 <= s_in_avid; s_wp2 <= s_wpar;
            s_eg3 <= s_gx2 * s_gain;
            s_ui3 <= s_in_u; s_vi3 <= s_in_v;    -- (one stage behind: harmless for chroma)
            s_wx3 <= s_wx2; s_we3 <= s_we2; s_wp3 <= s_wp2;
            if s_eg3(17 downto 6) > 876 then v_e := to_unsigned(876, 10); else v_e := s_eg3(15 downto 6); end if;
            if s_sw_edgefirst = '1' then
                if s_sw_onblack = '1' then s_wy4 <= v_e + 64;
                else s_wy4 <= f_clamp10(signed(resize(s_ym2, 12)) + signed(resize(v_e, 12)) - signed(resize(s_ym2(9 downto 1), 12))); end if;
                s_wu4 <= s_ui3; s_wv4 <= s_vi3;
            else
                s_wy4 <= s_ym2; s_wu4 <= s_ui3; s_wv4 <= s_vi3;
            end if;
            s_wx4 <= s_wx3; s_we4 <= s_we3; s_wp4 <= s_wp3;
        end if;
    end process p_win;

    p_addr : process(clk)
        procedure edge(signal src : in signed(12 downto 0); signal bg : out std_logic; signal addr : out unsigned(10 downto 0)) is
        begin
            if s_avd(8) = '0' or src < 0 or src >= signed(resize(s_line_width, 13)) then bg <= '1'; addr <= (others => '0');
            else bg <= '0'; addr <= unsigned(src(10 downto 0)); end if;
        end procedure;
    begin
        if rising_edge(clk) then
            s_x(1) <= s_rx - 1; s_avd(1) <= s_in_avid;
            for i in 2 to 12 loop s_x(i) <= s_x(i - 1); s_avd(i) <= s_avd(i - 1); end loop;
            if s_in_avid = '0' then s_ph1 <= s_lph1; s_ph2 <= s_lph2;
            else s_ph1 <= s_ph1 + resize(s_f1, 16); s_ph2 <= s_ph2 + resize(s_f2, 16); end if;
            s_sa1 <= s_ph1(15 downto 8); s_sa2 <= s_ph2(15 downto 8);
            if s_sw_csep = '1' then s_ca1 <= s_ph1(15 downto 8) + 64; s_ca2 <= s_ph2(15 downto 8) + 96;
            else s_ca1 <= s_ph1(15 downto 8); s_ca2 <= s_ph2(15 downto 8); end if;
            if s_sw_two = '1' then
                s_n6 <= resize(s_r1, 11) + resize(shift_right(s_r2, 1), 11);
                s_nc6 <= resize(s_rc1, 11) + resize(shift_right(s_rc2, 1), 11);
            else
                s_n6 <= resize(s_r1, 11); s_nc6 <= resize(s_rc1, 11);
            end if;
            s_pn7 <= s_n6 * signed(resize(s_amp, 9));
            s_pnc7 <= s_nc6 * signed(resize(s_amp, 9));
            s_gxd(2) <= s_gx2;
            for i in 3 to 7 loop s_gxd(i) <= s_gxd(i - 1); end loop;
            if s_sw_push = '1' then
                s_aY8 <= signed(resize(s_x(7), 13)) + resize(shift_right(s_pn7, 8), 13) + signed(resize(s_gxd(7)(9 downto 2), 13));
                s_aC8 <= signed(resize(s_x(7), 13)) + resize(shift_right(s_pnc7, 8), 13) + signed(resize(s_gxd(7)(9 downto 2), 13));
            else
                s_aY8 <= signed(resize(s_x(7), 13)) + resize(shift_right(s_pn7, 8), 13);
                s_aC8 <= signed(resize(s_x(7), 13)) + resize(shift_right(s_pnc7, 8), 13);
            end if;
            edge(s_aY8, s_bgY, s_addrY);
            edge(s_aC8, s_bgC, s_addrC);
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addrY));
            if s_we4 = '1' and s_wp4 = '0' then lbY0(to_integer(s_wx4)) <= std_logic_vector(s_wy4); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addrY));
            if s_we4 = '1' and s_wp4 = '1' then lbY1(to_integer(s_wx4)) <= std_logic_vector(s_wy4); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addrC(10 downto 1)));
            if s_we4 = '1' and s_wp4 = '0' then lbU0(to_integer(s_wx4(10 downto 1))) <= std_logic_vector(s_wu4(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addrC(10 downto 1)));
            if s_we4 = '1' and s_wp4 = '1' then lbU1(to_integer(s_wx4(10 downto 1))) <= std_logic_vector(s_wu4(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addrC(10 downto 1)));
            if s_we4 = '1' and s_wp4 = '0' then lbV0(to_integer(s_wx4(10 downto 1))) <= std_logic_vector(s_wv4(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addrC(10 downto 1)));
            if s_we4 = '1' and s_wp4 = '1' then lbV1(to_integer(s_wx4(10 downto 1))) <= std_logic_vector(s_wv4(9 downto 2)); end if;
        end if;
    end process;

    p_pix : process(clk)
        variable v_e : unsigned(10 downto 0);
        variable v_ey : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_yc(2) <= s_in_y;
            for i in 3 to 12 loop s_yc(i) <= s_yc(i - 1); end loop;
            -- 10 (rd) / 11: fetch
            s_a_bgY <= s_bgY; s_a_bgC <= s_bgC; s_a_rbank <= s_rbank;
            if s_a_bgY = '1' then s_wet_y <= to_unsigned(64, 10);
            elsif s_a_rbank = '1' then s_wet_y <= unsigned(s_rdY1); else s_wet_y <= unsigned(s_rdY0); end if;
            if s_a_bgC = '1' then s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00"; end if;
            s_pv12 <= s_wet_y;
            -- 12: gradients of the displaced picture against the input line
            if s_yc(11) >= s_wet_y then s_r1_13 <= s_yc(11) - s_wet_y; else s_r1_13 <= s_wet_y - s_yc(11); end if;
            if s_yc(10) >= s_pv12 then s_r2_13 <= s_yc(10) - s_pv12; else s_r2_13 <= s_pv12 - s_yc(10); end if;
            if s_wet_y >= s_pv12 then s_gh13 <= s_wet_y - s_pv12; else s_gh13 <= s_pv12 - s_wet_y; end if;
            if s_yc(11) >= s_wet_y then s_gv13 <= s_yc(11) - s_wet_y; else s_gv13 <= s_wet_y - s_yc(11); end if;
            s_y13 <= s_wet_y; s_u13 <= s_wet_u; s_v13 <= s_wet_v;
            -- 13
            case to_integer(s_mode) is
                when 0 => v_e := resize(s_r1_13, 11) + resize(s_r2_13, 11);
                when 1 => v_e := resize(s_gh13, 11) + resize(s_gh13, 11);
                when 2 => v_e := resize(s_gv13, 11) + resize(s_gv13, 11);
                when others => v_e := resize(s_gh13, 11) + resize(s_gv13, 11);
            end case;
            s_e14 <= v_e;
            s_y14 <= s_y13; s_u14 <= s_u13; s_v14 <= s_v13;
            -- 14
            s_eg15 <= s_e14 * s_gain;
            s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14;
            -- 15: compose (edges of the displaced picture)
            if s_eg15(18 downto 6) > 876 then v_ey := to_unsigned(876, 10); else v_ey := s_eg15(15 downto 6); end if;
            if s_sw_edgefirst = '1' then
                s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;
            elsif s_sw_onblack = '1' then
                if s_ink(9) = '1' then s_y16 <= v_ey + 64; else s_y16 <= 940 - v_ey; end if;
                s_u16 <= s_u15; s_v16 <= s_v15;
            else
                if s_ink(9) = '1' then s_y16 <= f_clamp10(signed(resize(s_y15, 12)) + signed(resize(v_ey, 12)));
                else s_y16 <= f_clamp10(signed(resize(s_y15, 12)) - signed(resize(v_ey, 12))); end if;
                s_u16 <= s_u15; s_v16 <= s_v15;
            end if;
            s_y17 <= s_y16; s_u17 <= s_u16; s_v17 <= s_v16;
            s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y19);
    data_out.u       <= std_logic_vector(s_u19);
    data_out.v       <= std_logic_vector(s_v19);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture ridgeline;
