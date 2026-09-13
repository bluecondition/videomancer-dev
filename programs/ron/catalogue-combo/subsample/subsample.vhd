-- Subsample: catalogue combo #37 (seed 20260918) -- datamosh / block
-- displacement + chroma subsampling artifacts (4:1:1, composite).  Three
-- knobs each; the switches and the slider decide how the two combine.
--
--   K1 Block    block size 8 .. 64 px
--   K2 Shift    block displacement amount
--   K3 Rate     re-roll rate
--   K4 Hold     chroma sampled every 1 .. 32 px (4:1:1 and worse)
--   K5 Bleed    chroma IIR smear length
--   K6 Crawl    composite dot-crawl amplitude (luma dots at chroma edges)
--
--   S7  Blocks  shift / hold (block repeats its first column)
--   S8  Chroma  subsampled after the mosh / before it (blocks carry the artefacts)
--   S9  Crawl   checkerboard / bars
--   S10 Moshed  blocks keep chroma / lose chroma (grey blocks)
--   S11 Hold    boxes / interpolated (bled)
--   P12 Density fraction of blocks moshed
--
-- One line buffer; per-block hash decides displacement; chroma hold and
-- IIR bleed run pre-buffer (delayed write port) or post-fetch.
-- Latency 16 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture subsample of program_top is

    constant C_LAT : integer := 16;

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

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    -- controls / per-frame
    signal s_k_block, s_k_shift, s_k_rate, s_k_hold, s_k_bleed, s_k_crawl : unsigned(9 downto 0);
    signal s_sw_hold, s_sw_pre, s_sw_bars, s_sw_grey, s_sw_interp : std_logic;
    signal s_p_density : unsigned(9 downto 0);
    signal s_shift, s_dens, s_cg : unsigned(7 downto 0) := (others => '0');
    signal s_bs     : unsigned(2 downto 0) := to_unsigned(4, 3);
    signal s_bmask  : unsigned(10 downto 0) := (others => '0');
    signal s_hmask  : unsigned(4 downto 0) := (others => '0');       -- hold period - 1 (0,1,3,7,15,31)
    signal s_bn     : unsigned(2 downto 0) := (others => '0');
    signal s_roll   : unsigned(15 downto 0) := (others => '0');
    signal s_rate_cnt : unsigned(7 downto 0) := (others => '0');
    signal s_frame  : unsigned(1 downto 0) := (others => '0');
    signal s_vstep  : unsigned(1 downto 0) := "11";

    -- tracking
    signal s_rx, s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(720, 11);
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_wpar, s_rbank : std_logic := '0';
    signal s_rowkey : unsigned(15 downto 0) := (others => '0');

    -- input side (pre-buffer subsampling, stages 1..4)
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_hu2, s_hv2 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y2     : unsigned(9 downto 0) := (others => '0');
    signal s_pu, s_pv : unsigned(15 downto 0) := to_unsigned(512 * 64, 16);
    signal s_y3     : unsigned(9 downto 0) := (others => '0');
    signal s_y4, s_u4, s_v4 : unsigned(9 downto 0) := (others => '0');
    type t_wx is array (1 to 3) of unsigned(10 downto 0);
    signal s_wx : t_wx := (others => (others => '0'));
    signal s_wp, s_wc : std_logic_vector(1 to 3) := (others => '0');
    signal s_wx4 : unsigned(10 downto 0) := (others => '0');
    signal s_wp4, s_we4 : std_logic := '0';

    -- address / hash chain (stages 1..8)
    signal s_x1     : unsigned(10 downto 0) := (others => '0');
    signal s_key2   : unsigned(15 downto 0) := (others => '0');
    signal s_h3, s_h4, s_h5 : unsigned(15 downto 0) := (others => '0');
    signal s_mosh6  : std_logic := '0';
    signal s_po6    : signed(17 downto 0) := (others => '0');
    signal s_u7     : signed(12 downto 0) := (others => '0');
    signal s_addr8  : unsigned(10 downto 0) := (others => '0');
    signal s_bg8    : std_logic := '0';
    type t_xd is array (2 to 6) of unsigned(10 downto 0);
    signal s_xd     : t_xd := (others => (others => '0'));
    signal s_moshd  : std_logic_vector(7 to 14) := (others => '0');
    signal s_xlo    : std_logic_vector(2 to 10) := (others => '0');   -- hold-phase flag delayed

    -- pixel path (stages 9..16)
    signal s_a_bg, s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_hu11, s_hv11 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y11    : unsigned(9 downto 0) := (others => '0');
    signal s_qu, s_qv : unsigned(15 downto 0) := to_unsigned(512 * 64, 16);
    signal s_y12    : unsigned(9 downto 0) := (others => '0');
    signal s_y13, s_u13, s_v13 : unsigned(9 downto 0) := (others => '0');
    signal s_du13, s_dv13 : signed(10 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');
    signal s_sat14  : unsigned(10 downto 0) := (others => '0');
    signal s_pc15   : unsigned(16 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := (others => '0');
    signal s_cx, s_clp, s_prev_hs14 : std_logic := '0';

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_block <= unsigned(registers_in(0));
    s_k_shift <= unsigned(registers_in(1));
    s_k_rate  <= unsigned(registers_in(2));
    s_k_hold  <= unsigned(registers_in(3));
    s_k_bleed <= unsigned(registers_in(4));
    s_k_crawl <= unsigned(registers_in(5));
    s_sw_hold   <= registers_in(6)(0);
    s_sw_pre    <= registers_in(6)(1);
    s_sw_bars   <= registers_in(6)(2);
    s_sw_grey   <= registers_in(6)(3);
    s_sw_interp <= registers_in(6)(4);
    s_p_density <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- Control + input side
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_n : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            -- stage 1
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            -- stage 2: chroma hold (sample at the start of each hold period) when pre
            if s_sw_pre = '0' or (s_rx(4 downto 0) and s_hmask) = 0 then
                s_hu2 <= unsigned(data_in.u); s_hv2 <= unsigned(data_in.v);
            end if;
            s_y2 <= s_in_y;
            -- stage 3: bleed IIR (Q6) when pre and interpolating
            v_n := to_integer(s_bn);
            if s_avid_sr(0) = '0' then
                s_pu <= s_hu2 & "000000"; s_pv <= s_hv2 & "000000";
            elsif s_sw_pre = '1' and s_sw_interp = '1' then
                s_pu <= s_pu - shift_right(s_pu, v_n) + shift_right(s_hu2 & "000000", v_n);
                s_pv <= s_pv - shift_right(s_pv, v_n) + shift_right(s_hv2 & "000000", v_n);
            else
                s_pu <= s_hu2 & "000000"; s_pv <= s_hv2 & "000000";
            end if;
            s_y3 <= s_y2;
            -- stage 4: write data
            s_y4 <= s_y3; s_u4 <= s_pu(15 downto 6); s_v4 <= s_pv(15 downto 6);
            s_wx(1) <= s_rx; s_wp(1) <= s_wpar; s_wc(1) <= data_in.avid;
            for i in 2 to 3 loop s_wx(i) <= s_wx(i - 1); s_wp(i) <= s_wp(i - 1); s_wc(i) <= s_wc(i - 1); end loop;
            s_wx4 <= s_wx(3); s_wp4 <= s_wp(3); s_we4 <= s_wc(3);

            -- per-frame decode
            case to_integer(s_vstep) is
                when 0 =>
                    s_bs    <= resize(s_k_block(9 downto 8), 3) + 3;
                    s_shift <= s_k_shift(9 downto 2);
                    s_dens  <= s_p_density(9 downto 2);
                    s_bn    <= s_k_bleed(9 downto 7);
                    s_cg    <= s_k_crawl(9 downto 2);
                    case to_integer(s_k_hold(9 downto 7)) is
                        when 0 => s_hmask <= "00000";
                        when 1 => s_hmask <= "00001";
                        when 2 => s_hmask <= "00011";
                        when 3 | 4 => s_hmask <= "00111";
                        when 5 | 6 => s_hmask <= "01111";
                        when others => s_hmask <= "11111";
                    end case;
                    if s_rate_cnt >= resize(not s_k_rate(9 downto 3), 8) then
                        s_rate_cnt <= (others => '0');
                        s_roll <= s_roll + 1013;
                    else
                        s_rate_cnt <= s_rate_cnt + 1;
                    end if;
                    s_frame <= s_frame + 1;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    for i in 0 to 10 loop
                        if i < to_integer(s_bs) then s_bmask(i) <= '1'; else s_bmask(i) <= '0'; end if;
                    end loop;
                    s_vstep <= "11";
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar;
                s_rowkey <= x"2C9B" + s_roll + shift_left(resize(shift_right(s_line, to_integer(s_bs)), 16), 7);
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
    -- Block hash + display address (stages 1..8)
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_x : unsigned(10 downto 0);
        variable v_o : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_x1 <= s_rx;
            s_key2 <= s_rowkey + resize(shift_right(s_x1, to_integer(s_bs)), 16);
            s_xd(2) <= s_x1;
            for i in 3 to 6 loop s_xd(i) <= s_xd(i - 1); end loop;
            if (s_x1(4 downto 0) and s_hmask) = 0 then s_xlo(2) <= '1'; else s_xlo(2) <= '0'; end if;
            for i in 3 to 10 loop s_xlo(i) <= s_xlo(i - 1); end loop;
            s_h3 <= f_round1(s_key2);
            s_h4 <= f_round2(s_h3);
            s_h5 <= f_round3(s_h4);
            if s_h5(15 downto 8) < s_dens then s_mosh6 <= '1'; else s_mosh6 <= '0'; end if;
            s_po6 <= (signed('0' & s_h5(7 downto 0)) - to_signed(128, 9)) * signed('0' & s_shift);
            if s_mosh6 = '1' then
                v_o := resize(shift_right(s_po6, 5), 13);
                if v_o > 511 then v_o := to_signed(511, 13); elsif v_o < -511 then v_o := to_signed(-511, 13); end if;
                if s_sw_hold = '1' then v_x := s_xd(6) and not s_bmask; else v_x := s_xd(6); end if;
                s_u7 <= signed(resize(v_x, 13)) + v_o;
            else
                s_u7 <= signed(resize(s_xd(6), 13));
            end if;
            s_moshd(7) <= s_mosh6;
            for i in 8 to 14 loop s_moshd(i) <= s_moshd(i - 1); end loop;
            if s_u7 < 0 or s_u7 >= signed(resize(s_line_width, 13)) then s_bg8 <= '1'; else s_bg8 <= '0'; end if;
            s_addr8 <= unsigned(s_u7(10 downto 0));
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (written stage 4, read stage 8 -> data 9)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr8));
            if s_we4 = '1' and s_wp4 = '0' then lbY0(to_integer(s_wx4)) <= std_logic_vector(s_y4); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr8));
            if s_we4 = '1' and s_wp4 = '1' then lbY1(to_integer(s_wx4)) <= std_logic_vector(s_y4); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr8(10 downto 1)));
            if s_we4 = '1' and s_wp4 = '0' then lbU0(to_integer(s_wx4(10 downto 1))) <= std_logic_vector(s_u4(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr8(10 downto 1)));
            if s_we4 = '1' and s_wp4 = '1' then lbU1(to_integer(s_wx4(10 downto 1))) <= std_logic_vector(s_u4(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr8(10 downto 1)));
            if s_we4 = '1' and s_wp4 = '0' then lbV0(to_integer(s_wx4(10 downto 1))) <= std_logic_vector(s_v4(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr8(10 downto 1)));
            if s_we4 = '1' and s_wp4 = '1' then lbV1(to_integer(s_wx4(10 downto 1))) <= std_logic_vector(s_v4(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel path (stages 9..16): fetch, post hold, post bleed, crawl, gate
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_n : integer range 0 to 7;
        variable v_au, v_av : unsigned(9 downto 0);
        variable v_y : signed(12 downto 0);
        variable v_s : std_logic;
    begin
        if rising_edge(clk) then
            -- stage 9 ctx / 10 fetch
            s_a_bg <= s_bg8; s_a_rbank <= s_rbank;
            if s_a_bg = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- stage 11: post hold
            if s_sw_pre = '1' or s_xlo(10) = '1' then
                s_hu11 <= s_wet_u; s_hv11 <= s_wet_v;
            end if;
            s_y11 <= s_wet_y;
            -- stage 12: post bleed
            v_n := to_integer(s_bn);
            if s_avid_sr(9) = '0' then
                s_qu <= s_hu11 & "000000"; s_qv <= s_hv11 & "000000";
            elsif s_sw_pre = '0' and s_sw_interp = '1' then
                s_qu <= s_qu - shift_right(s_qu, v_n) + shift_right(s_hu11 & "000000", v_n);
                s_qv <= s_qv - shift_right(s_qv, v_n) + shift_right(s_hv11 & "000000", v_n);
            else
                s_qu <= s_hu11 & "000000"; s_qv <= s_hv11 & "000000";
            end if;
            s_y12 <= s_y11;
            -- stage 13: chroma out (grey in moshed blocks), centred copies
            if s_sw_grey = '1' and s_moshd(12) = '1' then
                s_u13 <= to_unsigned(512, 10); s_v13 <= to_unsigned(512, 10);
            else
                s_u13 <= s_qu(15 downto 6); s_v13 <= s_qv(15 downto 6);
            end if;
            s_du13 <= signed('0' & s_qu(15 downto 6)) - to_signed(512, 11);
            s_dv13 <= signed('0' & s_qv(15 downto 6)) - to_signed(512, 11);
            s_y13 <= s_y12;
            -- stage 14: saturation
            if s_du13 < 0 then v_au := unsigned(resize(-s_du13, 10)); else v_au := unsigned(resize(s_du13, 10)); end if;
            if s_dv13 < 0 then v_av := unsigned(resize(-s_dv13, 10)); else v_av := unsigned(resize(s_dv13, 10)); end if;
            s_sat14 <= resize(v_au, 11) + resize(v_av, 11);
            s_y14 <= s_y13; s_u14 <= s_u13; s_v14 <= s_v13;
            -- stage 15: crawl amplitude
            s_pc15 <= s_sat14(10 downto 2) * s_cg;
            s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14;
            -- stage 16: crawl + gate
            if s_avid_sr(13) = '0' then s_cx <= '0'; else s_cx <= not s_cx; end if;
            s_prev_hs14 <= s_hsync_sr(14);
            if s_vsync_sr(14) = '0' then s_clp <= '0';
            elsif s_hsync_sr(14) = '0' and s_prev_hs14 = '1' then s_clp <= not s_clp; end if;
            v_s := s_cx xor (s_clp and not s_sw_bars) xor s_frame(0);
            if v_s = '1' then
                v_y := signed(resize(s_y15, 13)) + signed(resize(s_pc15(16 downto 7), 13));
            else
                v_y := signed(resize(s_y15, 13)) - signed(resize(s_pc15(16 downto 7), 13));
            end if;
            if s_avid_sr(14) = '1' then
                s_y16 <= f_clamp10(v_y); s_u16 <= s_u15; s_v16 <= s_v15;
            else
                s_y16 <= to_unsigned(64, 10); s_u16 <= to_unsigned(512, 10); s_v16 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture subsample;
