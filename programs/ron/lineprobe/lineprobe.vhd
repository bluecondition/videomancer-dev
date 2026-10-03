-- Videomancer SDK - Open source FPGA-based video effects development kit
-- Copyright (C) 2026 ron
-- File: lineprobe.vhd - input timing glitch detector + HDMI stress patterns
-- License: GNU General Public License v3.0
--
-- LINEPROBE v0.1 -- diagnostic for the cross-program "bars" glitch (a band
-- that starts partway along a line and runs to the right edge).
--
-- Watches the input timing every line and flags four kinds of glitch:
--   E1 RED    avid dropout: avid falls and rises again within one line
--   E2 GREEN  early hsync: an hsync edge during an active line, 8+ clocks
--             short of the previous line period (a doubled / glitched edge)
--   E3 BLUE   active-length change vs the previous active line
--   E4 YELLOW hsync -> avid start offset change vs the previous line
-- (E3/E4 tolerance: S8 Strict = any change, Loose = more than +-1 clock.)
--
-- Each event lights (a) a tick in its own 32-px column at the left edge for
-- the next 8 lines, and (b) its square in the top-right corner for ~1 s
-- (grey = idle).  K1 replaces the picture with stress patterns: flat black,
-- grey and orange (a U/V swap turns orange blue; HDMI link corruption shows
-- random colour even over black), 1-px stripes / checkerboard, noise, bars.
-- Output avid/syncs are the input's, delayed, exactly like a normal program.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture lineprobe of program_top is

    constant C_LAT : integer := 2;

    -- U/V-swapped hardware colours (scope-verified prism table)
    type t_c is array (0 to 7) of unsigned(9 downto 0);
    --                        W    R    O    Y    G    B    I    V
    constant C_BAR_Y : t_c := (to_unsigned(768,10), to_unsigned(328,10), to_unsigned(584,10), to_unsigned(768,10),
                               to_unsigned(580,10), to_unsigned(164,10), to_unsigned(192,10), to_unsigned(349,10));
    constant C_BAR_U : t_c := (to_unsigned(512,10), to_unsigned(960,10), to_unsigned(772,10), to_unsigned(584,10),
                               to_unsigned(136,10), to_unsigned(440,10), to_unsigned(608,10), to_unsigned(756,10));
    constant C_BAR_V : t_c := (to_unsigned(512,10), to_unsigned(360,10), to_unsigned(212,10), to_unsigned( 64,10),
                               to_unsigned(216,10), to_unsigned(960,10), to_unsigned(696,10), to_unsigned(852,10));

    -- marker colours: 1 R, 2 G, 3 B, 4 Y, 5 idle grey
    function f_my(i : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(i) is
            when 1 => return to_unsigned(328, 10);
            when 2 => return to_unsigned(580, 10);
            when 3 => return to_unsigned(164, 10);
            when 4 => return to_unsigned(768, 10);
            when others => return to_unsigned(300, 10);
        end case;
    end function;
    function f_mu(i : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(i) is
            when 1 => return to_unsigned(960, 10);
            when 2 => return to_unsigned(136, 10);
            when 3 => return to_unsigned(440, 10);
            when 4 => return to_unsigned(584, 10);
            when others => return to_unsigned(512, 10);
        end case;
    end function;
    function f_mv(i : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(i) is
            when 1 => return to_unsigned(360, 10);
            when 2 => return to_unsigned(216, 10);
            when 3 => return to_unsigned(960, 10);
            when 4 => return to_unsigned( 64, 10);
            when others => return to_unsigned(512, 10);
        end case;
    end function;

    -- |d| beyond tolerance (strict: any change; loose: more than +-1)
    function f_bad(d : signed(12 downto 0); loose : std_logic) return std_logic is
    begin
        if d = 0 then
            return '0';
        elsif loose = '1' and (d = 1 or d = -1) then
            return '0';
        else
            return '1';
        end if;
    end function;

    -- controls
    signal s_pat   : unsigned(2 downto 0) := (others => '0');
    signal s_mark  : std_logic := '1';
    signal s_loose : std_logic := '0';

    -- timing tracker
    signal s_hs_d, s_av_d : std_logic := '0';
    signal s_hcnt   : unsigned(11 downto 0) := (others => '0');  -- clocks since hsync fall
    signal s_hper   : unsigned(11 downto 0) := (others => '0');  -- last normal line period
    signal s_alen   : unsigned(11 downto 0) := (others => '0');  -- avid clocks this line (= x)
    signal s_ycnt   : unsigned(10 downto 0) := (others => '0');  -- active line (= y)
    signal s_seen, s_fell, s_started : std_logic := '0';
    signal s_sofs, s_sofs_prev : unsigned(11 downto 0) := (others => '0');
    signal s_sofs_chk, s_sofs_ok : std_logic := '0';
    signal s_alast, s_aprev : unsigned(11 downto 0) := (others => '0');
    signal s_alen_chk, s_aprev_ok : std_logic := '0';
    signal s_nav    : unsigned(11 downto 0) := (others => '0');
    signal s_nav11_d, s_fev : std_logic := '0';
    signal s_ev     : std_logic_vector(3 downto 0) := (others => '0');
    signal s_hsf    : std_logic := '0';   -- registered hsync-fall pulse

    type t_lh is array (0 to 3) of unsigned(2 downto 0);
    type t_fh is array (0 to 3) of unsigned(5 downto 0);
    signal s_lhold : t_lh := (others => (others => '0'));
    signal s_fhold : t_fh := (others => (others => '0'));

    -- measured geometry (latched per field)
    signal s_w    : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_xs   : unsigned(11 downto 0) := to_unsigned(592, 12);
    signal s_big  : std_logic := '0';     -- HD-class width: 64-px squares

    -- pixel stages
    signal s_lfsr : std_logic_vector(15 downto 0);
    signal a_y, a_u, a_v : unsigned(9 downto 0) := (others => '0');
    signal a_ov   : unsigned(2 downto 0) := (others => '0');
    signal a_av   : std_logic := '0';
    signal s_y_out, s_u_out, s_v_out : std_logic_vector(9 downto 0) := (others => '0');
    signal s_avid_sr    : std_logic_vector(C_LAT - 1 downto 0) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(C_LAT - 1 downto 0) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(C_LAT - 1 downto 0) := (others => '1');
    signal s_field_n_sr : std_logic_vector(C_LAT - 1 downto 0) := (others => '1');

begin

    p_ctl : process(clk)
    begin
        if rising_edge(clk) then
            s_pat   <= unsigned(registers_in(0)(9 downto 7));
            s_mark  <= registers_in(6)(0);
            s_loose <= registers_in(6)(1);
        end if;
    end process p_ctl;

    ----------------------------------------------------------------------------
    -- Timing tracker + event detection
    ----------------------------------------------------------------------------
    p_track : process(clk)
        variable v_hsf, v_rise, v_fall : boolean;
    begin
        if rising_edge(clk) then
            s_hs_d <= data_in.hsync_n;
            s_av_d <= data_in.avid;
            v_hsf  := data_in.hsync_n = '0' and s_hs_d = '1';
            v_rise := data_in.avid = '1' and s_av_d = '0';
            v_fall := data_in.avid = '0' and s_av_d = '1';
            s_ev   <= (others => '0');
            if v_hsf then s_hsf <= '1'; else s_hsf <= '0'; end if;

            if s_hcnt /= 4095 then
                s_hcnt <= s_hcnt + 1;
            end if;
            if data_in.avid = '1' then
                s_seen <= '1';
                if s_alen /= 4095 then
                    s_alen <= s_alen + 1;
                end if;
            end if;
            if v_fall then
                s_fell <= '1';
            end if;

            -- E1 avid re-rise within a line / first rise -> start offset
            if v_rise then
                if s_fell = '1' then
                    s_ev(0) <= '1';
                elsif s_started = '0' then
                    s_started  <= '1';
                    s_sofs     <= s_hcnt;
                    s_sofs_chk <= '1';
                end if;
            end if;
            -- E4 (one clock later)
            if s_sofs_chk = '1' then
                s_sofs_chk <= '0';
                if s_sofs_ok = '1' then
                    s_ev(3) <= f_bad(signed('0' & s_sofs) - signed('0' & s_sofs_prev), s_loose);
                end if;
                s_sofs_prev <= s_sofs;
                s_sofs_ok   <= '1';
            end if;

            if v_hsf then
                -- E2 early hsync during an active line
                if s_seen = '1' and s_hper > 256 and resize(s_hcnt, 13) + 8 < resize(s_hper, 13) then
                    s_ev(1) <= '1';
                elsif s_seen = '1' then
                    s_hper <= s_hcnt;
                end if;
                if s_seen = '1' then
                    s_alast    <= s_alen;
                    s_alen_chk <= '1';
                    if s_ycnt /= 2047 then
                        s_ycnt <= s_ycnt + 1;
                    end if;
                end if;
                s_hcnt    <= (others => '0');
                s_alen    <= (others => '0');
                s_seen    <= '0';
                s_fell    <= '0';
                s_started <= '0';
            end if;
            -- E3 (one clock later)
            if s_alen_chk = '1' then
                s_alen_chk <= '0';
                if s_aprev_ok = '1' then
                    s_ev(2) <= f_bad(signed('0' & s_alast) - signed('0' & s_aprev), s_loose);
                end if;
                s_aprev    <= s_alast;
                s_aprev_ok <= '1';
            end if;

            -- once-per-field event: avid absent for 2048 clocks
            if data_in.avid = '1' then
                s_nav <= (others => '0');
            elsif s_nav(11) = '0' then
                s_nav <= s_nav + 1;
            end if;
            s_nav11_d <= s_nav(11);
            s_fev     <= s_nav(11) and not s_nav11_d;
            if s_fev = '1' then
                s_ycnt     <= (others => '0');
                s_sofs_ok  <= '0';    -- don't compare across vertical blanking
                s_aprev_ok <= '0';
                s_w        <= s_aprev;
                if s_aprev > 1024 then
                    s_big <= '1';
                    s_xs  <= s_aprev - 256;
                else
                    s_big <= '0';
                    s_xs  <= s_aprev - 128;
                end if;
            end if;

            -- holds: 8 lines for the edge ticks, ~1 s for the corner squares
            for k in 0 to 3 loop
                if s_ev(k) = '1' then
                    s_lhold(k) <= to_unsigned(7, 3);
                elsif s_hsf = '1' and s_lhold(k) /= 0 then
                    s_lhold(k) <= s_lhold(k) - 1;
                end if;
                if s_ev(k) = '1' then
                    s_fhold(k) <= to_unsigned(60, 6);
                elsif s_fev = '1' and s_fhold(k) /= 0 then
                    s_fhold(k) <= s_fhold(k) - 1;
                end if;
            end loop;
        end if;
    end process p_track;

    ----------------------------------------------------------------------------
    -- Stage A: pattern / video + overlay decision.  x = s_alen, y = s_ycnt.
    ----------------------------------------------------------------------------
    p_a : process(clk)
        variable v_x  : unsigned(11 downto 0);
        variable v_dx : unsigned(11 downto 0);
        variable v_k  : unsigned(1 downto 0);
        variable v_b  : unsigned(2 downto 0);
        variable v_sq : boolean;
    begin
        if rising_edge(clk) then
            v_x := s_alen;
            case to_integer(s_pat) is
                when 0 =>                                   -- video
                    a_y <= unsigned(data_in.y);
                    a_u <= unsigned(data_in.u);
                    a_v <= unsigned(data_in.v);
                when 1 =>                                   -- black
                    a_y <= to_unsigned(64, 10);  a_u <= to_unsigned(512, 10); a_v <= to_unsigned(512, 10);
                when 2 =>                                   -- grey
                    a_y <= to_unsigned(512, 10); a_u <= to_unsigned(512, 10); a_v <= to_unsigned(512, 10);
                when 3 =>                                   -- orange
                    a_y <= C_BAR_Y(2); a_u <= C_BAR_U(2); a_v <= C_BAR_V(2);
                when 4 =>                                   -- 1-px B/W stripes
                    if v_x(0) = '1' then
                        a_y <= to_unsigned(768, 10);
                    else
                        a_y <= to_unsigned(64, 10);
                    end if;
                    a_u <= to_unsigned(512, 10); a_v <= to_unsigned(512, 10);
                when 5 =>                                   -- 1-px orange/blue checker
                    if (v_x(0) xor s_ycnt(0)) = '1' then
                        a_y <= C_BAR_Y(2); a_u <= C_BAR_U(2); a_v <= C_BAR_V(2);
                    else
                        a_y <= C_BAR_Y(5); a_u <= C_BAR_U(5); a_v <= C_BAR_V(5);
                    end if;
                when 6 =>                                   -- per-pixel noise
                    a_y <= resize(unsigned(s_lfsr(9 downto 1)), 10) + 64;
                    a_u <= resize(unsigned(s_lfsr(15 downto 8)), 10) + 384;
                    a_v <= resize(unsigned(s_lfsr(7 downto 0)), 10) + 384;
                when others =>                              -- 8 colour bars
                    if s_big = '1' then
                        v_b := v_x(10 downto 8);
                    else
                        v_b := v_x(9 downto 7);
                    end if;
                    a_y <= C_BAR_Y(to_integer(v_b));
                    a_u <= C_BAR_U(to_integer(v_b));
                    a_v <= C_BAR_V(to_integer(v_b));
            end case;

            -- overlay: edge ticks (x < 128, one 32-px column per event)
            a_ov <= (others => '0');
            v_k  := v_x(6 downto 5);
            if v_x < 128 and s_lhold(to_integer(v_k)) /= 0 then
                a_ov <= resize(v_k, 3) + 1;
            -- corner squares (top right, rows 8 .. 8+size)
            elsif v_x >= s_xs and v_x < s_w and s_ycnt >= 8 then
                v_dx := v_x - s_xs;
                if s_big = '1' then
                    v_k  := v_dx(7 downto 6);
                    v_sq := s_ycnt < 72 and v_dx(5 downto 2) /= 0;
                else
                    v_k  := v_dx(6 downto 5);
                    v_sq := s_ycnt < 40 and v_dx(4 downto 2) /= 0;
                end if;
                if v_sq then
                    if s_fhold(to_integer(v_k)) /= 0 then
                        a_ov <= resize(v_k, 3) + 1;
                    else
                        a_ov <= to_unsigned(5, 3);
                    end if;
                end if;
            end if;
            a_av <= data_in.avid;
        end if;
    end process p_a;

    ----------------------------------------------------------------------------
    -- Stage B: overlay + blanking gate -> output register
    ----------------------------------------------------------------------------
    p_b : process(clk)
    begin
        if rising_edge(clk) then
            if a_av = '0' then
                s_y_out <= std_logic_vector(to_unsigned(64, 10));
                s_u_out <= std_logic_vector(to_unsigned(512, 10));
                s_v_out <= std_logic_vector(to_unsigned(512, 10));
            elsif s_mark = '1' and a_ov /= 0 then
                s_y_out <= std_logic_vector(f_my(a_ov));
                s_u_out <= std_logic_vector(f_mu(a_ov));
                s_v_out <= std_logic_vector(f_mv(a_ov));
            else
                s_y_out <= std_logic_vector(a_y);
                s_u_out <= std_logic_vector(a_u);
                s_v_out <= std_logic_vector(a_v);
            end if;
        end if;
    end process p_b;

    p_sync_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= s_avid_sr   (C_LAT - 2 downto 0) & data_in.avid;
            s_hsync_n_sr <= s_hsync_n_sr(C_LAT - 2 downto 0) & data_in.hsync_n;
            s_vsync_n_sr <= s_vsync_n_sr(C_LAT - 2 downto 0) & data_in.vsync_n;
            s_field_n_sr <= s_field_n_sr(C_LAT - 2 downto 0) & data_in.field_n;
        end if;
    end process p_sync_delay;

    data_out.y       <= s_y_out;
    data_out.u       <= s_u_out;
    data_out.v       <= s_v_out;
    data_out.avid    <= s_avid_sr   (C_LAT - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LAT - 1);
    data_out.field_n <= s_field_n_sr(C_LAT - 1);

    lfsr_inst : entity work.lfsr16
        port map (
            clk    => clk,
            enable => '1',
            seed   => x"0000",
            load   => '0',
            q      => s_lfsr
        );

end architecture lineprobe;
