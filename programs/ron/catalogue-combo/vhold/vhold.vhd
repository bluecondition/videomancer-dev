-- Vhold: catalogue combo #73 (seed 20260922) -- rolling shutter / sync
-- roll + sync loss / horizontal collapse.  Three knobs each; the switches
-- and the slider decide how they combine.
--
--   K1 Roll     vertical-hold roll speed, bipolar (centre = locked)
--   K2 Bar      blanking bar height (0..255 lines)
--   K3 Jello    rows lean by a sine of row + time
--   K4 Collapse base squeeze of every row (0 = full width)
--   K5 Reach    how far below the seam the extra squeeze reaches
--   K6 Beam     collapsed rows brighten (beam concentrates)
--
--   S7  Bar     black / the squeezed picture shows through the bar
--   S8  Centre  rows collapse toward the screen centre / toward the jello offset
--   S9  Edges   outside the line: black / mirrored
--   S10 Tear    rows below the bar tear sideways (hashed per line)
--   S11 Pull    seam pull squeezes / stretches (negative pull)
--   P12 Pull    extra squeeze at the seam, decaying with Reach
--
-- Roll seam per frame (rollover), per-line squeeze factor from the seam
-- distance, DDA line-buffer read.  Latency 15 clocks, all modes,
-- blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture vhold of program_top is

    constant C_LAT : integer := 15;

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

    signal s_k_roll, s_k_bar, s_k_jello, s_k_col, s_k_reach, s_k_beam : unsigned(9 downto 0);
    signal s_sw_barshow, s_sw_cjello, s_sw_mirror, s_sw_tear, s_sw_stretch : std_logic;
    signal s_p_pull : unsigned(9 downto 0);

    signal s_barh : unsigned(7 downto 0) := (others => '0');
    signal s_col, s_pull, s_beam : unsigned(7 downto 0) := (others => '0');
    signal s_reach : unsigned(2 downto 0) := "011";
    signal s_seam : unsigned(10 downto 0) := (others => '0');
    signal s_seam_acc : unsigned(18 downto 0) := (others => '0');
    signal s_vstep : unsigned(2 downto 0) := "111";
    signal s_rd : signed(10 downto 0) := (others => '0');
    signal s_h16, s_sa, s_sap, s_sam : signed(19 downto 0) := (others => '0');
    signal s_neg, s_big : std_logic := '0';
    signal s_saw_active : std_logic := '0';
    signal s_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal s_jphase, s_line_phase : unsigned(15 downto 0) := (others => '0');
    signal s_cx : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_w2m1 : signed(12 downto 0) := (others => '0');

    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_height, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_rbank, s_we : std_logic := '0';

    -- per line
    signal s_lstep : unsigned(3 downto 0) := (others => '1');
    signal s_inbar, s_below : std_logic := '0';
    signal s_dist : unsigned(10 downto 0) := (others => '0');
    signal s_sin_addr : unsigned(7 downto 0) := (others => '0');
    signal s_sin_q, s_sin_q2 : signed(9 downto 0) := (others => '0');
    signal s_jp : signed(18 downto 0) := (others => '0');
    signal s_jel : signed(10 downto 0) := (others => '0');
    signal s_dsh : unsigned(10 downto 0) := (others => '0');
    signal s_pw : unsigned(7 downto 0) := (others => '0');
    signal s_pp : signed(16 downto 0) := (others => '0');
    signal s_q : signed(13 downto 0) := to_signed(256, 14);
    signal s_pc : signed(11 downto 0) := (others => '0');
    signal s_pq : signed(25 downto 0) := (others => '0');
    signal s_src0 : signed(21 downto 0) := (others => '0');
    signal s_gain_row : unsigned(9 downto 0) := (others => '0');
    signal s_vg : unsigned(9 downto 0) := (others => '0');
    signal s_gp : unsigned(17 downto 0) := (others => '0');
    signal s_bar_row : std_logic := '0';
    signal s_tear : signed(10 downto 0) := (others => '0');

    type t_b is array (1 to 6) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_acc : signed(21 downto 0) := (others => '0');
    signal s_src4 : signed(12 downto 0) := (others => '0');
    signal s_addr : unsigned(10 downto 0) := (others => '0');
    signal s_bg : std_logic := '0';
    signal s_a_bg, s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_yb8 : unsigned(10 downto 0) := (others => '0');
    signal s_u8, s_v8 : unsigned(9 downto 0) := (others => '0');
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := (others => '0');
    type t_c10 is array (10 to 14) of unsigned(9 downto 0);
    signal s_yd, s_ud, s_vd : t_c10 := (others => (others => '0'));
    type t_bd is array (2 to 14) of std_logic;
    signal s_bard : t_bd := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_roll  <= unsigned(registers_in(0));
    s_k_bar   <= unsigned(registers_in(1));
    s_k_jello <= unsigned(registers_in(2));
    s_k_col   <= unsigned(registers_in(3));
    s_k_reach <= unsigned(registers_in(4));
    s_k_beam  <= unsigned(registers_in(5));
    s_sw_barshow <= registers_in(6)(0);
    s_sw_cjello  <= registers_in(6)(1);
    s_sw_mirror  <= registers_in(6)(2);
    s_sw_tear    <= registers_in(6)(3);
    s_sw_stretch <= registers_in(6)(4);
    s_p_pull <= unsigned(registers_in(7));

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin_q <= to_signed(C_SINE(to_integer(s_sin_addr)), 10);
            s_sin_q2 <= s_sin_q;
        end if;
    end process;

    p_ctrl : process(clk)
        variable v_q : signed(13 downto 0);
        variable v_g : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;
            s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));

            s_barh <= s_k_bar(9 downto 2);
            s_col <= s_k_col(9 downto 2); s_pull <= s_p_pull(9 downto 2); s_beam <= s_k_beam(9 downto 2);
            s_reach <= s_k_reach(9 downto 7);
            s_cx <= '0' & s_line_width(10 downto 1);
            s_w2m1 <= signed(resize(s_line_width & '0', 13)) - 1;

            -- per-frame roll sequencer
            case to_integer(s_vstep) is
                when 0 =>
                    s_rd  <= signed(resize(s_k_roll, 11)) - to_signed(512, 11);
                    s_h16 <= signed(shift_left(resize(s_height, 20), 4));
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_sa <= signed(resize(s_seam_acc, 20)) + resize(s_rd, 20);
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_sap <= s_sa + s_h16; s_sam <= s_sa - s_h16;
                    if s_sa < 0 then s_neg <= '1'; else s_neg <= '0'; end if;
                    if s_sa >= s_h16 then s_big <= '1'; else s_big <= '0'; end if;
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    if s_neg = '1' then s_seam_acc <= unsigned(s_sap(18 downto 0));
                    elsif s_big = '1' then s_seam_acc <= unsigned(s_sam(18 downto 0));
                    else s_seam_acc <= unsigned(s_sa(18 downto 0)); end if;
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    s_seam <= s_seam_acc(14 downto 4);
                    s_vstep <= "111";
                when others => null;
            end case;

            -- per line
            case to_integer(s_lstep) is
                when 0 =>
                    if s_line >= s_seam and s_line < s_seam + resize(s_barh, 11) then s_inbar <= '1'; else s_inbar <= '0'; end if;
                    if s_line >= s_seam + resize(s_barh, 11) then s_below <= '1'; else s_below <= '0'; end if;
                    if s_line >= s_seam then s_dist <= s_line - s_seam; else s_dist <= s_line + s_height - s_seam; end if;
                    s_sin_addr <= s_line_phase(15 downto 8);
                    if s_sw_tear = '1' then s_tear <= signed(resize(s_lfsr(8 downto 0), 11)) - 256; else s_tear <= (others => '0'); end if;
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_dsh <= shift_right(s_dist, to_integer(s_reach));       -- 0.. : distance scaled by reach
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    if s_dsh > 255 then s_pw <= (others => '0'); else s_pw <= 255 - s_dsh(7 downto 0); end if;   -- pull weight 255 at the seam
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    s_pp <= signed(resize(s_pw * s_pull, 17));                -- pull * weight
                    s_jp <= s_sin_q2 * signed(resize(s_k_jello(9 downto 1), 9));
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    -- q = 256 + col*8 + pull-term (or minus when stretching), clamp >= 32
                    if s_sw_stretch = '1' then
                        v_q := to_signed(256, 14) + signed(resize(s_col & "000", 14)) - resize(shift_right(s_pp, 3), 14);
                    else
                        v_q := to_signed(256, 14) + signed(resize(s_col & "000", 14)) + resize(shift_right(s_pp, 3), 14);
                    end if;
                    if v_q < 32 then v_q := to_signed(32, 14); end if;
                    s_q <= v_q;
                    s_jel <= resize(shift_right(s_jp, 9), 11);
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    if s_sw_cjello = '1' then s_pc <= signed(resize(s_cx, 12)) + resize(s_jel, 12); else s_pc <= signed(resize(s_cx, 12)); end if;
                    if s_below = '1' then s_pc <= signed(resize(s_cx, 12)) + resize(s_tear, 12); end if;
                    v_g := unsigned(resize(shift_right(s_q - 256, 3), 14));
                    if v_g > 700 then v_g := to_unsigned(700, 14); end if;
                    s_vg <= v_g(9 downto 0);
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    s_pq <= s_pc * s_q;
                    s_gp <= s_vg * s_beam;
                    s_lstep <= s_lstep + 1;
                when 7 =>
                    s_src0 <= resize(shift_left(resize(signed(resize(s_cx, 12)), 22), 8) - resize(s_pq, 22) - resize(s_q, 22) + resize(shift_left(resize(s_jel, 22), 8), 22), 22);
                    s_gain_row <= s_gp(17 downto 8);
                    s_bar_row <= s_inbar;
                    s_lstep <= s_lstep + 1;
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1; s_height <= s_line + 1;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
                s_line_phase <= s_line_phase + 96;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0';
                if s_saw_active = '1' then s_jphase <= s_jphase + 700; s_line_phase <= s_jphase; s_vstep <= (others => '0'); end if;
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

    p_addr : process(clk)
        variable v_neg, v_ref : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_avd(1) <= s_in_avid;
            for i in 2 to 6 loop s_avd(i) <= s_avd(i - 1); end loop;
            if s_avd(1) = '0' then s_acc <= s_src0; else s_acc <= s_acc + resize(s_q, 22); end if;
            s_src4 <= s_acc(20 downto 8);
            v_neg := -s_src4; v_ref := s_w2m1 - s_src4;
            if s_avd(3) = '0' then
                s_bg <= '1'; s_addr <= (others => '0');
            elsif s_src4 < 0 then
                if s_sw_mirror = '1' and v_neg < signed(resize(s_line_width, 13)) then s_bg <= '0'; s_addr <= unsigned(v_neg(10 downto 0)); else s_bg <= '1'; s_addr <= (others => '0'); end if;
            elsif s_src4 >= signed(resize(s_line_width, 13)) then
                if s_sw_mirror = '1' and v_ref >= 0 then s_bg <= '0'; s_addr <= unsigned(v_ref(10 downto 0)); else s_bg <= '1'; s_addr <= (others => '0'); end if;
            else
                s_bg <= '0'; s_addr <= unsigned(s_src4(10 downto 0));
            end if;
            s_bard(2) <= s_bar_row;
            for i in 3 to 14 loop s_bard(i) <= s_bard(i - 1); end loop;
        end if;
    end process p_addr;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    p_pix : process(clk)
    begin
        if rising_edge(clk) then
            -- 6 (rd) / 7: fetch
            s_a_bg <= s_bg; s_a_rbank <= s_rbank;
            if s_a_bg = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- 8: beam
            s_yb8 <= resize(s_wet_y, 11) + resize(s_gain_row, 11);
            s_u8 <= s_wet_u; s_v8 <= s_wet_v;
            -- 9: clamp, bar
            if s_bard(8) = '1' and s_sw_barshow = '0' then
                s_y9 <= to_unsigned(64, 10); s_u9 <= to_unsigned(512, 10); s_v9 <= to_unsigned(512, 10);
            else
                if s_yb8 > 1023 then s_y9 <= (others => '1'); else s_y9 <= s_yb8(9 downto 0); end if;
                s_u9 <= s_u8; s_v9 <= s_v8;
            end if;
            s_yd(10) <= s_y9; s_ud(10) <= s_u9; s_vd(10) <= s_v9;
            for i in 11 to 14 loop s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1); end loop;
            -- 15: gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y15 <= s_yd(14); s_u15 <= s_ud(14); s_v15 <= s_vd(14);
            else
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y15);
    data_out.u       <= std_logic_vector(s_u15);
    data_out.v       <= std_logic_vector(s_v15);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture vhold;
