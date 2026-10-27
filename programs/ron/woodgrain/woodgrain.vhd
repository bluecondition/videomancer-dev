-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- WOODGRAIN - an Atari 2600 inside the Videomancer.
--
-- A cycle-exact 6507 + TIA + RIOT (woodgrain_cpu.vhd, woodgrain_vcs.vhd) runs
-- the 4K cartridge in woodgrain_cart_pkg.vhd (cartridge.bin in this folder if
-- present, else the bundled CC0 room explorer; see woodgrain.py).  The
-- console renders one game frame per output field (1080p at 24-30 Hz: one
-- shown + one hidden frame, so games keep their 50-60 Hz speed) into an
-- 8-line ring buffer, paced just ahead of the display beam; this file scales
-- the 160 x 216 picture to the screen, colours it through the NTSC palette
-- and conditions it for the encoders.
--
-- GEOMETRY (from the timing ID; one TV-shaped picture on every raster)
--   1080p / 1080i  pixels 9 clocks x 5 rows (1080i: 2.5 field rows), 1440 x 1080
--   720p           5 x 3 rows, 800 x 648
--   480p / 576p    4 x 2 rows, 640 x 432;  480i / 576i 4 x 1 field row
-- The window shows game lines 25..240 after the VSYNC-off line (the usual
-- 192-line picture with a 12-line margin) centred on the raster.
--
-- CONTROLS
--   K1  Stick X     left third = left, right third = right (hysteresis)
--   K2  Stick Y     bottom third = down, top third = up
--   S7  Fire        the joystick button, held while on
--   S8  Reset       each flip presses Game Reset (8 fields)
--   S9  Select      each flip presses Game Select (8 fields)
--   S10 Difficulty  left difficulty switch B / A
--   S11 Auto        a wandering joystick drives the game while K1/K2 rest
--                   in their centre thirds
--
-- OUTPUT.  The HD HDMI bar fix is built in (crisp moving full-swing edges):
-- [1 2 1]/4 on Y, U and V and luma scaled to 13/16 above black (white ~777),
-- then the blanking gate.  The program makes its own avid (cubist recipe:
-- parity-locked start, field-latched width) since it draws its own colour.
-- Latency: 8 clocks in every mode.
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;
use work.woodgrain_consts_pkg.all;

architecture woodgrain of program_top is
    constant C_PROCESSING_DELAY_CLKS : natural := 8;
    constant C_LAT : integer := 8;
    constant C_W0  : natural := 25;
    constant C_WH  : natural := 216;

    signal s_timing : t_video_timing_port;

    -- mode (decoded every clock from the timing ID)
    signal m_xrep : unsigned(3 downto 0) := to_unsigned(8, 4);   -- clocks per pixel - 1
    signal m_left : unsigned(8 downto 0) := to_unsigned(240, 9); -- border clocks
    signal m_ra   : unsigned(2 downto 0) := to_unsigned(4, 3);   -- rows per even line - 1
    signal m_rb   : unsigned(2 downto 0) := to_unsigned(4, 3);   -- rows per odd line - 1
    signal m_top  : unsigned(6 downto 0) := (others => '0');     -- first window row
    signal m_hid  : std_logic := '1';

    -- field / rows
    signal s_saw, s_go : std_logic := '0';
    signal s_hcnt, s_meas_h : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal r_row   : unsigned(10 downto 0) := (others => '0');
    signal r_inwin : std_logic := '0';
    signal r_w     : unsigned(7 downto 0) := (others => '0');
    signal r_sub   : unsigned(2 downto 0) := (others => '0');
    signal r_dispw : unsigned(8 downto 0) := (others => '0');
    signal r_slot  : unsigned(2 downto 0) := (others => '0');

    -- generated avid (aurora / cubist)
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- column DDA
    signal c_lc  : unsigned(8 downto 0) := (others => '0');
    signal c_inx : std_logic := '0';
    signal c_ax  : unsigned(7 downto 0) := (others => '0');
    signal c_sub : unsigned(3 downto 0) := (others => '0');

    -- ring buffer (8 game lines x 256) and palette
    type t_ring is array(0 to 2047) of std_logic_vector(7 downto 0);
    signal ring : t_ring;
    attribute no_rw_check : boolean;
    attribute no_rw_check of ring : signal is true;
    signal ring_q  : std_logic_vector(7 downto 0) := (others => '0');
    signal rd_q    : std_logic_vector(7 downto 0) := (others => '0');
    signal pal_raw : std_logic_vector(29 downto 0) := (others => '0');
    signal pal_q   : std_logic_vector(29 downto 0) := (others => '0');
    signal f1, f2, f3, f4 : std_logic := '0';

    signal px_we   : std_logic;
    signal px_addr : unsigned(10 downto 0);
    signal px_col  : std_logic_vector(6 downto 0);

    -- output conditioning
    signal p0y, p0u, p0v, p1y, p1u, p1v, p2y, p2u, p2v : unsigned(9 downto 0) := (others => '0');
    signal smy, smu, smv : unsigned(9 downto 0) := (others => '0');

    -- controls
    signal j_l, j_r, j_u, j_d : std_logic := '0';
    signal a_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal a_dir  : std_logic_vector(3 downto 0) := (others => '0');   -- R L D U
    signal a_t    : unsigned(5 downto 0) := (others => '0');
    signal sw_q   : std_logic_vector(1 downto 0) := "00";
    signal sw_ok  : std_logic := '0';
    signal p_rst, p_sel : unsigned(3 downto 0) := (others => '0');
    signal swcha, swchb : std_logic_vector(7 downto 0) := x"FF";
    signal fire : std_logic := '0';

    -- sync delay
    type t_bsr is array(1 to C_LAT - 1) of std_logic;
    signal d_hs, d_vs, d_fl, d_gv : t_bsr := (others => '0');
    signal s_io : t_video_stream_yuv444_30b
        := ((others => '0'), (others => '0'), (others => '0'), '0', '1', '1', '1');
begin
    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    -- ====================================================================
    -- The console
    -- ====================================================================
    vcs_inst : entity work.woodgrain_vcs
        generic map (G_W0 => C_W0, G_WH => C_WH)
        port map (clk => clk, go => s_go, hidden => m_hid, disp_w => r_dispw,
                  swcha => swcha, swchb => swchb, fire => fire,
                  px_we => px_we, px_addr => px_addr, px_col => px_col);

    -- ====================================================================
    -- Mode table (timing ID)
    -- ====================================================================
    p_mode : process(clk)
    begin
        if rising_edge(clk) then
            m_hid <= '0';
            case registers_in(8)(3 downto 0) is
                when x"0" | x"8" =>                    -- 480i / 576i
                    m_xrep <= to_unsigned(3, 4); m_left <= to_unsigned(40, 9);
                    m_ra <= "000"; m_rb <= "000";
                    if registers_in(8)(3) = '1' then m_top <= to_unsigned(36, 7);
                    else m_top <= to_unsigned(13, 7); end if;
                when x"4" | x"C" =>                    -- 480p / 576p
                    m_xrep <= to_unsigned(3, 4); m_left <= to_unsigned(40, 9);
                    m_ra <= "001"; m_rb <= "001";
                    if registers_in(8)(3) = '1' then m_top <= to_unsigned(72, 7);
                    else m_top <= to_unsigned(24, 7); end if;
                when x"5" | x"6" | x"E" =>             -- 720p
                    m_xrep <= to_unsigned(4, 4); m_left <= to_unsigned(240, 9);
                    m_ra <= "010"; m_rb <= "010"; m_top <= to_unsigned(36, 7);
                when x"1" | x"2" | x"A" =>             -- 1080i: 2.5 field rows a line
                    m_xrep <= to_unsigned(8, 4); m_left <= to_unsigned(240, 9);
                    m_ra <= "010"; m_rb <= "001"; m_top <= (others => '0');
                when others =>                         -- 1080p 24-30 Hz
                    m_xrep <= to_unsigned(8, 4); m_left <= to_unsigned(240, 9);
                    m_ra <= "100"; m_rb <= "100"; m_top <= (others => '0');
                    m_hid <= '1';
            end case;
        end if;
    end process p_mode;

    -- ====================================================================
    -- Field start, line width, window rows
    -- ====================================================================
    p_rows : process(clk)
        variable v_rep : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            -- once per field: the first vsync edge after active video
            s_go <= '0';
            if s_timing.avid = '1' then
                s_saw <= '1';
            elsif s_timing.vsync_start = '1' and s_saw = '1' then
                s_saw <= '0';
                s_go  <= '1';
            end if;

            if s_timing.hsync_start = '1' then
                if s_hcnt > 0 then s_meas_h <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            elsif s_timing.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            end if;

            if s_go = '1' then
                g_wm1   <= s_meas_h - 1;
                r_row   <= (others => '0');
                r_inwin <= '0';
                r_dispw <= (others => '0');
            elsif s_timing.avid_start = '1' then
                if r_row = m_top then
                    r_inwin <= '1';
                    r_w     <= (others => '0');
                    r_sub   <= (others => '0');
                    r_dispw <= (others => '0');
                elsif r_inwin = '1' then
                    if r_w(0) = '0' then v_rep := m_ra; else v_rep := m_rb; end if;
                    if r_sub = v_rep then
                        r_sub <= (others => '0');
                        if r_w = C_WH - 1 then
                            r_inwin <= '0';
                            r_dispw <= to_unsigned(300, 9);
                        else
                            r_w     <= r_w + 1;
                            r_dispw <= '0' & (r_w + 1);
                        end if;
                    else
                        r_sub <= r_sub + 1;
                    end if;
                end if;
                if r_row /= 2047 then
                    r_row <= r_row + 1;
                end if;
            end if;
            r_slot <= r_w(2 downto 0) + to_unsigned(C_W0 mod 8, 3);
        end if;
    end process p_rows;

    -- ====================================================================
    -- Clean avid (see aurora): parity-locked start, field-latched width
    -- ====================================================================
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;    -- hsync_n falling
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;       -- avid rising

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

            -- column DDA: (c_inx, c_ax) describe the column g_avid is on now
            if g_hsr = '1' then
                c_lc  <= m_left;
                c_inx <= '0';
                c_ax  <= (others => '0');
                c_sub <= (others => '0');
            elsif g_avid = '1' then
                if c_inx = '0' then
                    if c_lc = 1 then
                        c_inx <= '1';
                        c_ax  <= (others => '0');
                        c_sub <= (others => '0');
                    end if;
                    if c_lc /= 0 then
                        c_lc <= c_lc - 1;
                    end if;
                elsif c_sub = m_xrep then
                    c_sub <= (others => '0');
                    if c_ax = 159 then
                        c_inx <= '0';
                    else
                        c_ax <= c_ax + 1;
                    end if;
                else
                    c_sub <= c_sub + 1;
                end if;
            end if;
        end if;
    end process p_gavid;

    -- ====================================================================
    -- Ring buffer (EBR, one write port from the console) and palette
    -- ====================================================================
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            if px_we = '1' then
                ring(to_integer(px_addr)) <= '0' & px_col;
            end if;
            ring_q <= ring(to_integer(r_slot & c_ax));
        end if;
    end process p_ring;

    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            pal_raw <= C_WG_PAL(to_integer(unsigned(rd_q(6 downto 0))));
        end if;
    end process p_pal;

    -- ====================================================================
    -- Pixel pipe: column -> ring -> palette -> [1 2 1] -> luma 13/16 -> gate
    -- ====================================================================
    p_pix : process(clk)
        variable v_d  : unsigned(9 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            f1 <= c_inx and r_inwin;                         -- stage 0 -> 1
            rd_q <= ring_q;                                  -- stage 1
            f2 <= f1;
            f3 <= f2;                                        -- stage 2 (palette address)
            pal_q <= pal_raw;                                -- stage 3
            f4 <= f3;
            if f4 = '1' then                                 -- stage 4
                p0y <= unsigned(pal_q(29 downto 20));
                p0u <= unsigned(pal_q(19 downto 10));
                p0v <= unsigned(pal_q(9 downto 0));
            else
                p0y <= to_unsigned(64, 10);
                p0u <= to_unsigned(512, 10);
                p0v <= to_unsigned(512, 10);
            end if;
            p1y <= p0y; p1u <= p0u; p1v <= p0v;
            p2y <= p1y; p2u <= p1u; p2v <= p1v;
            -- stage 5: [1 2 1] / 4 on all three channels (HD HDMI bar fix)
            v_sy := ("00" & p0y) + ("0" & p1y & "0") + ("00" & p2y);
            v_su := ("00" & p0u) + ("0" & p1u & "0") + ("00" & p2u);
            v_sv := ("00" & p0v) + ("0" & p1v & "0") + ("00" & p2v);
            smy <= v_sy(11 downto 2);
            smu <= v_su(11 downto 2);
            smv <= v_sv(11 downto 2);
            -- stage 6: luma to 13/16 above black, blanking gate on the generated avid
            if d_gv(C_LAT - 1) = '1' then
                v_d := smy - 64;
                s_io.y <= std_logic_vector(smy - shift_right(v_d, 3) - shift_right(v_d, 4));
                s_io.u <= std_logic_vector(smu);
                s_io.v <= std_logic_vector(smv);
            else
                s_io.y <= std_logic_vector(to_unsigned(64, 10));
                s_io.u <= std_logic_vector(to_unsigned(512, 10));
                s_io.v <= std_logic_vector(to_unsigned(512, 10));
            end if;
            s_io.hsync_n <= d_hs(C_LAT - 1);
            s_io.vsync_n <= d_vs(C_LAT - 1);
            s_io.field_n <= d_fl(C_LAT - 1);
            s_io.avid    <= d_gv(C_LAT - 1);
        end if;
    end process p_pix;

    -- ====================================================================
    -- Controls -> joystick / console switches (updated once per field)
    -- ====================================================================
    p_ctl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_c : std_logic;
        variable v_dir : std_logic_vector(3 downto 0);
        variable v_sw : std_logic_vector(1 downto 0);
    begin
        if rising_edge(clk) then
            if s_go = '1' then
                -- K1: stick X, thirds with hysteresis
                v_k := unsigned(registers_in(0));
                if v_k < 300 then j_l <= '1'; elsif v_k > 340 then j_l <= '0'; end if;
                if v_k > 723 then j_r <= '1'; elsif v_k < 683 then j_r <= '0'; end if;
                -- K2: stick Y (clockwise = up)
                v_k := unsigned(registers_in(1));
                if v_k < 300 then j_d <= '1'; elsif v_k > 340 then j_d <= '0'; end if;
                if v_k > 723 then j_u <= '1'; elsif v_k < 683 then j_u <= '0'; end if;

                -- wandering stick: a new heading every ~0.6-1 s
                a_lfsr <= a_lfsr(14 downto 0) & (a_lfsr(15) xor a_lfsr(13) xor a_lfsr(12) xor a_lfsr(10));
                if a_t = 0 then
                    a_t <= "10" & unsigned(a_lfsr(3 downto 0));
                    case a_lfsr(7 downto 5) is
                        when "000"  => a_dir <= "1000";        -- R
                        when "001"  => a_dir <= "0100";        -- L
                        when "010"  => a_dir <= "0010";        -- D
                        when "011"  => a_dir <= "0001";        -- U
                        when "100"  => a_dir <= "1001";        -- R U
                        when "101"  => a_dir <= "0110";        -- L D
                        when "110"  => a_dir <= "1010";        -- R D
                        when others => a_dir <= "0101";        -- L U
                    end case;
                else
                    a_t <= a_t - 1;
                end if;

                -- Reset / Select: each flip of S8 / S9 presses the button for 8 fields
                v_sw := registers_in(6)(2 downto 1);
                sw_q  <= v_sw;
                sw_ok <= '1';
                if sw_ok = '1' and v_sw(0) /= sw_q(0) then
                    p_rst <= to_unsigned(8, 4);
                elsif p_rst /= 0 then
                    p_rst <= p_rst - 1;
                end if;
                if sw_ok = '1' and v_sw(1) /= sw_q(1) then
                    p_sel <= to_unsigned(8, 4);
                elsif p_sel /= 0 then
                    p_sel <= p_sel - 1;
                end if;
            end if;

            v_dir := j_r & j_l & j_d & j_u;
            if v_dir = "0000" and registers_in(6)(4) = '1' then
                v_dir := a_dir;
            end if;
            swcha <= not v_dir & "1111";
            v_c := '0';
            if p_rst /= 0 then v_c := '1'; end if;
            swchb(0) <= not v_c;
            v_c := '0';
            if p_sel /= 0 then v_c := '1'; end if;
            swchb(1) <= not v_c;
            swchb(2) <= '1';
            swchb(3) <= '1';                                 -- colour
            swchb(5 downto 4) <= "11";
            swchb(6) <= registers_in(6)(3);                  -- left difficulty: 1 = A
            swchb(7) <= '0';
            fire <= registers_in(6)(0);
        end if;
    end process p_ctl;

    -- ====================================================================
    -- Sync delay
    -- ====================================================================
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            d_hs(1) <= data_in.hsync_n;
            d_vs(1) <= data_in.vsync_n;
            d_fl(1) <= data_in.field_n;
            d_gv(1) <= g_avid;
            for i in 2 to C_LAT - 1 loop
                d_hs(i) <= d_hs(i - 1);
                d_vs(i) <= d_vs(i - 1);
                d_fl(i) <= d_fl(i - 1);
                d_gv(i) <= d_gv(i - 1);
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

end architecture woodgrain;
