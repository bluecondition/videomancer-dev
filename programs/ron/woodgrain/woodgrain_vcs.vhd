-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- WOODGRAIN - the console: 6507 + TIA (video) + RIOT + cartridge ROM.
--
-- SLOTS.  Emulated time advances in 6-clock slots = one CPU cycle = three
-- TIA color clocks.  Inside a slot:
--   ph0  RIOT clock; the CPU's bus cycle is decoded: RAM write, TIA write
--        with delay 0 applied, delay-1/2 writes latched, HMOVE armed
--   ph1  color clock A        ROM/RAM data registered, TIA/RIOT reads sampled
--   ph2  delay-1 TIA writes   read data muxed into the CPU's din
--   ph3  color clock B        (CPU step)
--   ph4  delay-2 TIA writes   (CPU ALU 1)
--   ph5  color clock C        (CPU ALU 2) slot end: start the next slot or idle
-- The color clocks of slot n are those of CPU cycle n+1, so a read in cycle
-- n sees everything up to color clock 3n+2 and a write lands before color
-- clock 3n+3+d, d = 0 / 1 / 2 / 6 by register (Stella's delay table).
-- Writes only ever land on even phases and clocks only tick on odd ones.
--
-- TIA objects follow Stella's counter model: 0..159 counters ticking on
-- visible clocks and HMOVE pulses (every 4th clock, (HMxx xor 8) of them,
-- during HBLANK), draw-start decodes at 156 / 12 / 28 / 60, first pixel 5
-- clocks after a player's decode (4 for missiles / ball).  Not modelled:
-- starfield / late-HMOVE quirks, NUSIZ changes mid-draw, paddles, audio.
--
-- PACING.  The console free-runs about 1.9x real time (HD: ~9x).  It stops
--   - at the game's VSYNC until the next output field start (go): one game
--     frame per field, or per frame plus one hidden frame when 'hidden'
--   - before a window line that would overwrite a ring-buffer line the
--     display has not shown yet (8-line ring, see woodgrain.vhd)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.woodgrain_cart_pkg.all;

entity woodgrain_vcs is
    generic (
        G_W0 : natural := 25;       -- first displayed line, counted from the VSYNC-off line
        G_WH : natural := 216       -- displayed lines
    );
    port (
        clk     : in  std_logic;
        go      : in  std_logic;                     -- output field start (1-clock pulse)
        hidden  : in  std_logic;                     -- run a hidden frame after each shown one
        disp_w  : in  unsigned(8 downto 0);          -- window line on screen now
        swcha   : in  std_logic_vector(7 downto 0);  -- joysticks (0 = pressed)
        swchb   : in  std_logic_vector(7 downto 0);  -- console switches
        fire    : in  std_logic;                     -- P0 button, 1 = pressed
        px_we   : out std_logic;
        px_addr : out unsigned(10 downto 0);         -- ring slot (3) & x (8)
        px_col  : out std_logic_vector(6 downto 0)
    );
end entity woodgrain_vcs;

architecture rtl of woodgrain_vcs is
    -- ---------------- slot sequencer ----------------
    signal r_act   : std_logic := '0';
    signal r_ph    : unsigned(2 downto 0) := (others => '0');
    signal r_fwait : std_logic := '1';      -- waiting at a frame boundary for go
    signal r_gop   : std_logic := '0';      -- go pending
    signal r_first : std_logic := '1';      -- first go after configuration: reset the CPU
    signal r_shown : std_logic := '0';
    signal r_hleft : std_logic := '0';
    signal r_vsr   : std_logic := '0';      -- a frame boundary happened in this slot
    signal r_run   : std_logic := '0';      -- CPU runs in this slot
    signal r_wsync : std_logic := '0';
    signal r_wclr  : std_logic := '0';
    signal r_pace  : std_logic := '0';

    -- ---------------- CPU / bus ----------------
    signal c_addr : unsigned(15 downto 0);
    signal c_we   : std_logic;
    signal c_dout : std_logic_vector(7 downto 0);
    signal c_din  : std_logic_vector(7 downto 0) := (others => '0');
    signal c_rst  : std_logic := '0';
    signal c_sync : std_logic;

    signal rom_q  : std_logic_vector(7 downto 0) := (others => '0');
    signal rd_rom : std_logic_vector(7 downto 0) := (others => '0');
    type t_ram is array(0 to 127) of std_logic_vector(7 downto 0);
    signal ram    : t_ram;
    signal ram_q  : std_logic_vector(7 downto 0) := (others => '0');
    signal rd_ram : std_logic_vector(7 downto 0) := (others => '0');
    signal ram_we : std_logic;
    attribute ram_style : string;
    attribute ram_style of ram : signal is "block";
    attribute no_rw_check : boolean;
    attribute no_rw_check of ram : signal is true;
    signal rd_io  : std_logic_vector(7 downto 0) := (others => '0');   -- TIA / RIOT read

    -- ---------------- RIOT ----------------
    signal t_timer : unsigned(7 downto 0) := (others => '0');
    signal t_sub   : unsigned(9 downto 0) := (others => '0');
    signal t_div   : unsigned(1 downto 0) := "11";
    signal t_flag  : std_logic := '0';
    signal t_wpend : std_logic := '0';
    signal t_wval  : unsigned(7 downto 0) := (others => '0');
    signal t_wdiv  : unsigned(1 downto 0) := (others => '0');

    -- ---------------- TIA ----------------
    type t_obj is record
        cnt : unsigned(7 downto 0);
        ren : std_logic;
        rc  : signed(6 downto 0);
        smp : unsigned(3 downto 0);
        cp1 : std_logic;            -- drawing the main copy
        hm  : unsigned(3 downto 0); -- HMOVE pulses = HMxx(7:4) xor 8
        mov : std_logic;
    end record;
    constant C_OBJ0 : t_obj := ((others => '0'), '0', (others => '0'), (others => '0'), '0', x"8", '0');
    type t_objs is array(0 to 4) of t_obj;       -- P0 P1 M0 M1 BL
    signal o : t_objs := (others => C_OBJ0);

    type t_b8 is array(0 to 1) of std_logic_vector(7 downto 0);
    signal grpn, grpo : t_b8 := (others => (others => '0'));
    signal nus   : t_b8 := (others => (others => '0'));   -- NUSIZ (5:4 missile width, 2:0 copies)
    signal refp, vdelp, enam, resmp : std_logic_vector(1 downto 0) := "00";
    signal enbln, enblo, vdelbl : std_logic := '0';
    signal ctrlpf : std_logic_vector(5 downto 0) := (others => '0');
    signal pf     : std_logic_vector(19 downto 0) := (others => '0');
    signal pf_refl, pf_on : std_logic := '0';
    type t_col is array(0 to 3) of std_logic_vector(6 downto 0);   -- P0 P1 PF BK
    signal col    : t_col := (others => (others => '0'));
    signal cx     : std_logic_vector(14 downto 0) := (others => '0');
    signal vblank, vsync : std_logic := '0';
    signal hctr   : unsigned(7 downto 0) := (others => '0');
    signal fstate : std_logic := '0';               -- '1' = visible part of the line
    signal exthb  : std_logic := '0';
    signal mv_on  : std_logic := '0';
    signal mv_clk : unsigned(4 downto 0) := (others => '0');
    signal line_n : unsigned(8 downto 0) := (others => '0');

    -- delayed writes
    signal w1_v, w2_v : std_logic := '0';
    signal w1_a, w2_a : unsigned(5 downto 0) := (others => '0');
    signal w1_d, w2_d : std_logic_vector(7 downto 0) := (others => '0');
    signal hmv_pend   : std_logic := '0';
    signal hmv_cnt    : unsigned(1 downto 0) := (others => '0');

    -- pixel pipe: tick -> compose -> ring
    signal k_go, k_vis, k_blk, k_cxen, k_win : std_logic := '0';
    signal k_on  : std_logic_vector(5 downto 0) := (others => '0');   -- P0 P1 M0 M1 BL PF
    signal k_x   : unsigned(7 downto 0) := (others => '0');
    signal k_slot : unsigned(2 downto 0) := (others => '0');
    signal q_we  : std_logic := '0';
    signal q_addr : unsigned(10 downto 0) := (others => '0');
    signal q_col : std_logic_vector(6 downto 0) := (others => '0');

    -- ------------------------------------------------------------------
    function f_copy(cnt : unsigned(7 downto 0); n : std_logic_vector(2 downto 0)) return boolean is
    begin
        case n is
            when "001" => return cnt = 12;
            when "010" => return cnt = 28;
            when "011" => return cnt = 12 or cnt = 28;
            when "100" => return cnt = 60;
            when "110" => return cnt = 28 or cnt = 60;
            when others => return false;
        end case;
    end function;

    function f_cnt_next(c : unsigned(7 downto 0)) return unsigned is
    begin
        if c = 159 then return to_unsigned(0, 8); else return c + 1; end if;
    end function;

    -- one counter clock of a player (n = NUSIZ copies)
    function f_ptick(p : t_obj; n : std_logic_vector(2 downto 0)) return t_obj is
        variable v   : t_obj := p;
        variable rcn : signed(6 downto 0);
        variable adv : boolean;
    begin
        if p.cnt = 156 or f_copy(p.cnt, n) then
            v.ren := '1';
            v.smp := (others => '0');
            v.rc  := to_signed(-5, 7);
            if p.cnt = 156 then v.cp1 := '1'; else v.cp1 := '0'; end if;
        elsif p.ren = '1' then
            rcn  := p.rc + 1;
            v.rc := rcn;
            if n = "101" then                 -- double width
                adv := rcn > 1 and rcn(0) = '1';
            elsif n = "111" then              -- quad width
                adv := rcn > 1 and rcn(1 downto 0) = "01";
            else
                adv := rcn > 0;
            end if;
            if adv then
                v.smp := p.smp + 1;
                if p.smp = 7 then v.ren := '0'; end if;
            end if;
        end if;
        v.cnt := f_cnt_next(p.cnt);
        return v;
    end function;

    -- one counter clock of a missile / the ball (copies only for missiles)
    function f_mtick(m : t_obj; n : std_logic_vector(2 downto 0); w : std_logic_vector(1 downto 0);
                     hold : std_logic; ball : boolean) return t_obj is
        variable v   : t_obj := m;
        variable rcn : signed(6 downto 0);
        variable trig : boolean;
    begin
        if ball then
            trig := m.cnt = 156;
        else
            trig := (m.cnt = 156 or f_copy(m.cnt, n)) and hold = '0';
        end if;
        if trig then
            v.ren := '1';
            v.rc  := to_signed(-4, 7);
        elsif m.ren = '1' then
            rcn  := m.rc + 1;
            v.rc := rcn;
            case w is
                when "00"   => if rcn >= 1 then v.ren := '0'; end if;
                when "01"   => if rcn >= 2 then v.ren := '0'; end if;
                when "10"   => if rcn >= 4 then v.ren := '0'; end if;
                when others => if rcn >= 8 then v.ren := '0'; end if;
            end case;
        end if;
        v.cnt := f_cnt_next(m.cnt);
        return v;
    end function;

    function f_pon(p : t_obj; g : std_logic_vector(7 downto 0); refl : std_logic;
                   n : std_logic_vector(2 downto 0)) return std_logic is
        variable idx : integer range 0 to 7;
        variable trip : boolean;
    begin
        if refl = '1' then idx := to_integer(p.smp(2 downto 0));
        else               idx := 7 - to_integer(p.smp(2 downto 0)); end if;
        if n = "101" or n = "111" then trip := p.rc >= 1; else trip := p.rc >= 0; end if;
        if p.ren = '1' and trip and g(idx) = '1' then return '1'; else return '0'; end if;
    end function;
begin
    -- ==================================================================
    -- CPU
    -- ==================================================================
    cpu : entity work.woodgrain_cpu
        port map (clk => clk, ph => r_ph, act => r_act, run => r_run, rst => c_rst,
                  din => c_din, addr => c_addr, we => c_we, dout => c_dout,
                  sync => c_sync, dbg => open);

    -- cartridge ROM (EBR): A12 selects it; 4K window
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            rom_q <= C_WG_CART(to_integer(c_addr(11 downto 0)));
        end if;
    end process p_rom;

    -- RIOT RAM (EBR): one write port, registered read
    ram_we <= '1' when r_act = '1' and r_ph = 0 and r_run = '1' and c_we = '1'
                       and c_addr(12) = '0' and c_addr(7) = '1' and c_addr(9) = '0' else '0';
    p_ram : process(clk)
    begin
        if rising_edge(clk) then
            if ram_we = '1' then
                ram(to_integer(c_addr(6 downto 0))) <= c_dout;
            end if;
            ram_q <= ram(to_integer(c_addr(6 downto 0)));
        end if;
    end process p_ram;

    px_we   <= q_we;
    px_addr <= q_addr;
    px_col  <= q_col;

    -- ==================================================================
    -- Sequencer, bus, RIOT, TIA
    -- ==================================================================
    p_main : process(clk)
        variable v_wen  : std_logic;
        variable v_wa   : unsigned(5 downto 0);
        variable v_wd   : std_logic_vector(7 downto 0);
        variable v_tick : boolean;
        variable v_pulse : boolean;
        variable v_clk  : unsigned(3 downto 0);
        variable v_o    : t_objs;
        variable v_on   : std_logic_vector(5 downto 0);
        variable v_do   : std_logic_vector(4 downto 0);
        variable v_x    : unsigned(7 downto 0);
        variable v_col  : unsigned(5 downto 0);
        variable v_bit  : integer range 0 to 19;
        variable v_resx : unsigned(7 downto 0);
        variable v_cxn  : std_logic_vector(14 downto 0);
        variable v_c    : std_logic_vector(6 downto 0);
        variable v_rel  : signed(9 downto 0);
        variable v_ra   : unsigned(15 downto 0);
        variable v_t    : std_logic_vector(1 downto 0);
        variable v_div  : unsigned(9 downto 0);
        variable v_sub  : unsigned(9 downto 0);
        variable v_mv   : std_logic;
        variable v_back : unsigned(7 downto 0);
        variable v_fw, v_ws : std_logic;
    begin
        if rising_edge(clk) then
            c_rst <= '0';
            q_we  <= '0';
            if go = '1' then
                r_gop <= '1';
            end if;

            -- ---------------- slot sequencer ----------------
            -- end of a slot: WSYNC release and frame-boundary bookkeeping
            v_fw := r_fwait;
            v_ws := r_wsync;
            if r_act = '1' and r_ph = 5 then
                if r_wclr = '1' then
                    v_ws := '0';
                end if;
                r_wclr <= '0';
                if r_vsr = '1' then
                    r_vsr <= '0';
                    if r_hleft = '1' then
                        r_hleft <= '0';
                        r_shown <= '0';                      -- next frame runs hidden
                    else
                        v_fw := '1';                         -- wait for the next field
                    end if;
                end if;
                r_wsync <= v_ws;
            end if;
            if v_fw = '1' and r_gop = '1' and (r_act = '0' or r_ph = 5) then
                r_gop   <= '0';
                v_fw    := '0';
                r_shown <= '1';
                r_hleft <= hidden;
                if r_first = '1' then
                    r_first <= '0';
                    c_rst   <= '1';                          -- engine starts at a field start
                end if;
            end if;
            r_fwait <= v_fw;
            if r_act = '1' and r_ph /= 5 then
                r_ph <= r_ph + 1;
            elsif v_fw = '0' and r_pace = '0' then
                r_act <= '1';
                r_ph  <= (others => '0');
                r_run <= not v_ws;
            else
                r_act <= '0';
                r_ph  <= (others => '0');
            end if;

            -- pacing: do not let a window line run more than 6 lines ahead of the display
            v_rel := signed('0' & line_n) - to_signed(G_W0, 10) - signed('0' & disp_w);
            if r_shown = '1' and line_n >= G_W0 and line_n < G_W0 + G_WH and v_rel >= 7 then
                r_pace <= '1';
            else
                r_pace <= '0';
            end if;

            -- ---------------- RIOT clock (ph0) ----------------
            if r_act = '1' and r_ph = 0 then
                case t_div is
                    when "00"   => v_div := to_unsigned(0, 10);
                    when "01"   => v_div := to_unsigned(7, 10);
                    when "10"   => v_div := to_unsigned(63, 10);
                    when others => v_div := to_unsigned(1023, 10);
                end case;
                v_sub := (t_sub + 1) and v_div;
                t_sub <= v_sub;
                if t_flag = '1' then
                    t_timer <= t_timer - 1;
                elsif v_sub = 0 then
                    if t_timer = 0 then
                        t_timer <= x"FF";
                        t_flag  <= '1';
                    else
                        t_timer <= t_timer - 1;
                    end if;
                end if;
            end if;

            -- ---------------- bus cycle decode (ph0) ----------------
            if r_act = '1' and r_ph = 0 and r_run = '1' and c_we = '1' and c_addr(12) = '0' then
                if c_addr(7) = '0' then
                    -- TIA: latch delay-1 / delay-2 writes and HMOVE
                    case to_integer(c_addr(5 downto 0)) is
                        when 16#01# | 16#0B# | 16#0C# | 16#1B# | 16#1C# | 16#1D# | 16#1E# | 16#1F# =>
                            w1_v <= '1'; w1_a <= c_addr(5 downto 0); w1_d <= c_dout;
                        when 16#0D# | 16#0E# | 16#0F# | 16#20# | 16#21# | 16#22# | 16#23# | 16#24# | 16#2B# =>
                            w2_v <= '1'; w2_a <= c_addr(5 downto 0); w2_d <= c_dout;
                        when 16#2A# =>
                            hmv_pend <= '1'; hmv_cnt <= "10";
                        when others => null;
                    end case;
                elsif c_addr(9) = '1' and c_addr(4) = '1' and c_addr(2) = '1' then
                    t_wpend <= '1';                         -- RIOT timer write, lands at ph1
                    t_wval  <= unsigned(c_dout);
                    t_wdiv  <= c_addr(1 downto 0);
                end if;
            end if;

            -- ---------------- ph1: memory data, IO reads, timer write ----------------
            if r_act = '1' and r_ph = 1 then
                rd_rom <= rom_q;
                rd_ram <= ram_q;
                v_ra := c_addr;
                if v_ra(7) = '0' then
                    -- TIA read: D7/D6 driven, the rest is the last bus value
                    case to_integer(v_ra(3 downto 0)) is
                        when 0  => v_t := cx(0) & cx(1);
                        when 1  => v_t := cx(2) & cx(3);
                        when 2  => v_t := cx(4) & cx(5);
                        when 3  => v_t := cx(6) & cx(7);
                        when 4  => v_t := cx(8) & cx(9);
                        when 5  => v_t := cx(10) & cx(11);
                        when 6  => v_t := cx(12) & '0';
                        when 7  => v_t := cx(13) & cx(14);
                        when 12 => v_t := (not fire) & '0';
                        when others => v_t := "10";
                    end case;
                    rd_io <= v_t & c_din(5 downto 0);
                else
                    case to_integer(v_ra(2 downto 0)) is
                        when 0      => rd_io <= swcha;
                        when 2      => rd_io <= swchb;
                        when 4 | 6  => rd_io <= std_logic_vector(t_timer);
                        when 5 | 7  => rd_io <= t_flag & "0000000";
                        when others => rd_io <= x"00";
                    end case;
                    if r_run = '1' and c_we = '0' and v_ra(12) = '0' and v_ra(9) = '1'
                       and v_ra(0) = '0' and v_ra(2) = '1' then
                        t_flag <= '0';                       -- INTIM read clears the flag
                    end if;
                end if;
                if t_wpend = '1' then
                    t_wpend <= '0';
                    t_timer <= t_wval;
                    t_div   <= t_wdiv;
                    t_flag  <= '0';
                    case t_wdiv is
                        when "00"   => t_sub <= to_unsigned(0, 10);
                        when "01"   => t_sub <= to_unsigned(7, 10);
                        when "10"   => t_sub <= to_unsigned(63, 10);
                        when others => t_sub <= to_unsigned(1023, 10);
                    end case;
                end if;
            end if;

            -- ---------------- ph2: CPU read data ----------------
            if r_act = '1' and r_ph = 2 then
                if c_we = '1' then
                    c_din <= c_dout;                         -- the bus carries the written value
                elsif c_addr(12) = '1' then
                    c_din <= rd_rom;
                elsif c_addr(7) = '1' and c_addr(9) = '0' then
                    c_din <= rd_ram;
                else
                    c_din <= rd_io;
                end if;
            end if;

            -- ---------------- TIA register writes (even phases) ----------------
            v_wen := '0';
            v_wa  := (others => '0');
            v_wd  := (others => '0');
            if r_act = '1' and r_ph = 0 and r_run = '1' and c_we = '1'
               and c_addr(12) = '0' and c_addr(7) = '0' then
                v_wen := '1'; v_wa := c_addr(5 downto 0); v_wd := c_dout;
                case to_integer(v_wa) is
                    when 16#01# | 16#0B# | 16#0C# | 16#1B# | 16#1C# | 16#1D# | 16#1E# | 16#1F#
                       | 16#0D# | 16#0E# | 16#0F# | 16#20# | 16#21# | 16#22# | 16#23# | 16#24#
                       | 16#2B# | 16#2A# =>
                        v_wen := '0';                        -- delayed
                    when others => null;
                end case;
            elsif r_act = '1' and r_ph = 2 and w1_v = '1' then
                v_wen := '1'; v_wa := w1_a; v_wd := w1_d; w1_v <= '0';
            elsif r_act = '1' and r_ph = 4 and w2_v = '1' then
                v_wen := '1'; v_wa := w2_a; v_wd := w2_d; w2_v <= '0';
            end if;

            if fstate = '1' then
                v_resx := to_unsigned(157, 8);
            elsif hctr >= 73 then
                v_resx := to_unsigned(158, 8);
            else
                v_resx := to_unsigned(159, 8);
            end if;

            if v_wen = '1' then
                case to_integer(v_wa) is
                    when 16#00# =>
                        if v_wd(1) = '1' and vsync = '0' then
                            r_vsr <= '1';                    -- frame boundary
                        elsif v_wd(1) = '0' and vsync = '1' then
                            line_n <= (others => '0');       -- line 0 = the VSYNC-off line
                        end if;
                        vsync <= v_wd(1);
                    when 16#01# => vblank <= v_wd(1);
                    when 16#02# => r_wsync <= '1';
                    when 16#04# => nus(0) <= "000" & v_wd(5 downto 4) & v_wd(2 downto 0);
                    when 16#05# => nus(1) <= "000" & v_wd(5 downto 4) & v_wd(2 downto 0);
                    when 16#06# => col(0) <= v_wd(7 downto 1);
                    when 16#07# => col(1) <= v_wd(7 downto 1);
                    when 16#08# => col(2) <= v_wd(7 downto 1);
                    when 16#09# => col(3) <= v_wd(7 downto 1);
                    when 16#0A# => ctrlpf <= v_wd(5 downto 0);
                    when 16#0B# => refp(0) <= v_wd(3);
                    when 16#0C# => refp(1) <= v_wd(3);
                    when 16#0D# => pf(3 downto 0) <= v_wd(7 downto 4);
                    when 16#0E# =>
                        for b in 0 to 7 loop
                            pf(4 + b) <= v_wd(7 - b);
                        end loop;
                    when 16#0F# => pf(19 downto 12) <= v_wd;
                    when 16#10# => o(0).cnt <= v_resx;
                    when 16#11# => o(1).cnt <= v_resx;
                    when 16#12# => o(2).cnt <= v_resx;
                    when 16#13# => o(3).cnt <= v_resx;
                    when 16#14# => o(4).cnt <= v_resx;
                    when 16#1B# => grpn(0) <= v_wd; grpo(1) <= grpn(1);
                    when 16#1C# => grpn(1) <= v_wd; grpo(0) <= grpn(0); enblo <= enbln;
                    when 16#1D# => enam(0) <= v_wd(1);
                    when 16#1E# => enam(1) <= v_wd(1);
                    when 16#1F# => enbln <= v_wd(1);
                    when 16#20# => o(0).hm <= unsigned(v_wd(7 downto 4)) xor x"8";
                    when 16#21# => o(1).hm <= unsigned(v_wd(7 downto 4)) xor x"8";
                    when 16#22# => o(2).hm <= unsigned(v_wd(7 downto 4)) xor x"8";
                    when 16#23# => o(3).hm <= unsigned(v_wd(7 downto 4)) xor x"8";
                    when 16#24# => o(4).hm <= unsigned(v_wd(7 downto 4)) xor x"8";
                    when 16#25# => vdelp(0) <= v_wd(0);
                    when 16#26# => vdelp(1) <= v_wd(0);
                    when 16#27# => vdelbl <= v_wd(0);
                    when 16#28# => resmp(0) <= v_wd(1);
                    when 16#29# => resmp(1) <= v_wd(1);
                    when 16#2B# =>
                        for i in 0 to 4 loop
                            o(i).hm <= x"8";
                        end loop;
                    when others => null;
                end case;
            end if;

            -- HMOVE lands 6 color clocks after its write: ph0 two slots later
            if r_act = '1' and r_ph = 0 and hmv_pend = '1' and hmv_cnt = 0 then
                hmv_pend <= '0';
                mv_on  <= '1';
                mv_clk <= (others => '0');
                exthb  <= '1';
                for i in 0 to 4 loop
                    o(i).mov <= '1';
                end loop;
            end if;
            if r_act = '1' and r_ph = 5 and hmv_pend = '1' and hmv_cnt /= 0 then
                hmv_cnt <= hmv_cnt - 1;
            end if;

            -- ---------------- color clock (ph1 / ph3 / ph5) ----------------
            v_tick := r_act = '1' and r_ph(0) = '1';
            k_go <= '0';
            if v_tick then
                v_o := o;
                v_x := hctr - 68;
                v_on := (others => '0');
                -- HMOVE pulses: every 4th clock, during HBLANK they clock the counters
                v_pulse := mv_on = '1' and hctr(1 downto 0) = "00";
                if mv_clk > 15 then v_clk := (others => '0'); else v_clk := mv_clk(3 downto 0); end if;
                v_do := (others => '0');
                v_mv := '0';
                for i in 0 to 4 loop
                    if v_pulse and o(i).mov = '1' then
                        if v_clk = o(i).hm then
                            v_o(i).mov := '0';
                        elsif fstate = '0' then
                            v_do(i) := '1';
                        end if;
                    end if;
                    if fstate = '1' then
                        v_do(i) := '1';
                    end if;
                    v_mv := v_mv or v_o(i).mov;
                end loop;
                if v_pulse then
                    mv_on <= v_mv;
                    if mv_clk /= 31 then mv_clk <= mv_clk + 1; end if;
                end if;

                -- visibility from the state before this clock
                for i in 0 to 1 loop
                    if vdelp(i) = '1' then
                        v_on(i) := f_pon(o(i), grpo(i), refp(i), nus(i)(2 downto 0));
                    else
                        v_on(i) := f_pon(o(i), grpn(i), refp(i), nus(i)(2 downto 0));
                    end if;
                end loop;
                for i in 0 to 1 loop
                    if o(2 + i).ren = '1' and o(2 + i).rc >= 0 and enam(i) = '1' and resmp(i) = '0' then
                        v_on(2 + i) := '1';
                    end if;
                end loop;
                if o(4).ren = '1' and o(4).rc >= 0 and ((vdelbl = '1' and enblo = '1') or (vdelbl = '0' and enbln = '1')) then
                    v_on(4) := '1';
                end if;

                -- counter clocks
                for i in 0 to 1 loop
                    if v_do(i) = '1' then
                        v_o(i) := f_ptick(v_o(i), nus(i)(2 downto 0));
                    end if;
                    if v_do(2 + i) = '1' then
                        v_o(2 + i) := f_mtick(v_o(2 + i), nus(i)(2 downto 0), nus(i)(4 downto 3), resmp(i), false);
                    end if;
                end loop;
                if v_do(4) = '1' then
                    v_o(4) := f_mtick(v_o(4), "000", ctrlpf(5 downto 4), '0', true);
                end if;
                -- RESMP: lock the missile to the middle of its player
                for i in 0 to 1 loop
                    if fstate = '1' and resmp(i) = '1' and v_o(i).ren = '1' and v_o(i).smp = 4 and v_o(i).cp1 = '1' then
                        case nus(i)(2 downto 0) is
                            when "101"  => v_back := to_unsigned(8, 8);
                            when "111"  => v_back := to_unsigned(12, 8);
                            when others => v_back := to_unsigned(5, 8);
                        end case;
                        if v_o(i).cnt >= v_back then
                            v_o(2 + i).cnt := v_o(i).cnt - v_back;
                        else
                            v_o(2 + i).cnt := v_o(i).cnt + 160 - v_back;
                        end if;
                    end if;
                end loop;
                o <= v_o;

                -- playfield (latched every 4th visible clock)
                if fstate = '1' then
                    if v_x = 0 or v_x = 79 then
                        pf_refl <= ctrlpf(0);
                    end if;
                    if v_x(1 downto 0) = "00" then
                        v_col := v_x(7 downto 2);
                        if v_col < 20 then
                            v_bit := to_integer(v_col);
                        elsif pf_refl = '1' then
                            v_bit := to_integer(39 - v_col);
                        else
                            v_bit := to_integer(v_col - 20);
                        end if;
                        pf_on <= pf(v_bit);
                        v_on(5) := pf(v_bit);
                    else
                        v_on(5) := pf_on;
                    end if;
                end if;

                -- hand the pixel to the compose stage
                k_go   <= '1';
                k_on   <= v_on;
                k_x    <= v_x;
                k_slot <= line_n(2 downto 0);
                k_vis  <= fstate;
                k_blk  <= '0';
                if fstate = '0' and hctr >= 68 then
                    k_blk <= '1';                            -- HMOVE comb: black
                end if;
                k_cxen <= fstate and not vblank;
                if r_shown = '1' and line_n >= G_W0 and line_n < G_W0 + G_WH then
                    k_win <= '1';
                else
                    k_win <= '0';
                end if;

                -- horizontal state machine
                if fstate = '0' then
                    if hctr = 0 then
                        exthb <= '0';
                    elsif hctr = 67 and exthb = '0' then
                        fstate <= '1';
                    elsif hctr = 75 and exthb = '1' then
                        fstate <= '1';
                    end if;
                end if;
                if hctr = 0 and r_ph = 1 then
                    r_wclr <= '1';                           -- this slot starts a line: WSYNC ends
                end if;
                if hctr = 227 then
                    hctr   <= (others => '0');
                    fstate <= '0';
                    if line_n = 400 then
                        line_n <= (others => '0');           -- no VSYNC: free-run frames
                        r_vsr  <= '1';
                    else
                        line_n <= line_n + 1;
                    end if;
                else
                    hctr <= hctr + 1;
                end if;
            end if;

            -- ---------------- compose (the clock after a color clock) ----------------
            if k_go = '1' then
                v_c := col(3);
                if vblank = '1' or k_blk = '1' then
                    v_c := (others => '0');
                elsif ctrlpf(2) = '1' then                   -- playfield priority
                    if k_on(5) = '1' or k_on(4) = '1' then v_c := col(2);
                    elsif k_on(0) = '1' or k_on(2) = '1' then v_c := col(0);
                    elsif k_on(1) = '1' or k_on(3) = '1' then v_c := col(1);
                    end if;
                elsif ctrlpf(1) = '1' then                   -- score mode
                    if k_on(0) = '1' or k_on(2) = '1' then v_c := col(0);
                    elsif k_on(5) = '1' then
                        if k_x < 80 then v_c := col(0); else v_c := col(1); end if;
                    elsif k_on(1) = '1' or k_on(3) = '1' then v_c := col(1);
                    elsif k_on(4) = '1' then v_c := col(2);
                    end if;
                else
                    if k_on(0) = '1' or k_on(2) = '1' then v_c := col(0);
                    elsif k_on(1) = '1' or k_on(3) = '1' then v_c := col(1);
                    elsif k_on(5) = '1' or k_on(4) = '1' then v_c := col(2);
                    end if;
                end if;
                if (k_vis = '1' or k_blk = '1') and k_win = '1' then
                    q_we <= '1';
                end if;
                q_addr <= k_slot & k_x;
                q_col  <= v_c;

                -- collisions (M0P1 M0P0 M1P0 M1P1 P0PF P0BL P1PF P1BL M0PF M0BL M1PF M1BL BLPF P0P1 M0M1)
                v_cxn := (others => '0');
                if k_cxen = '1' then
                    v_cxn := (k_on(3) and k_on(2)) & (k_on(0) and k_on(1)) & (k_on(4) and k_on(5))
                           & (k_on(3) and k_on(4)) & (k_on(3) and k_on(5)) & (k_on(2) and k_on(4))
                           & (k_on(2) and k_on(5)) & (k_on(1) and k_on(4)) & (k_on(1) and k_on(5))
                           & (k_on(0) and k_on(4)) & (k_on(0) and k_on(5)) & (k_on(3) and k_on(1))
                           & (k_on(3) and k_on(0)) & (k_on(2) and k_on(0)) & (k_on(2) and k_on(1));
                end if;
                cx <= cx or v_cxn;
            end if;
            if v_wen = '1' and v_wa = 16#2C# then
                cx <= (others => '0');                       -- CXCLR (after this clock's collisions)
            end if;
        end if;
    end process p_main;
end architecture rtl;
