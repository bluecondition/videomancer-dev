-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Doppler v0.1.0 -- Patchbay: wavefronts from a moving source.
--
-- The patch: a circle generator (two ramps squared and summed) into a bank of sixteen comparators draws rings
-- that grow at the speed of sound; the generator's centre CV passes down an analog shift register clocked by
-- the ring oscillator, so each older ring is centred where the source WAS when it was emitted; the comparator
-- outputs sum into a colouriser; a slow LFO wobbles the heading, a second one breathes the Mach number.
--
--   ring m = n + f (n = 0..15):  F_m(p) = |q|^2 + 2 m (d.q) + m^2 (|d|^2 - L^2),  q = p - S,  |d| = Mach L
--   one DDA pair:  F_0 = P (second order),  J_0 = H (first order) + camera;  F_{n+1} = F_n + J_n, J_{n+1} = J_n + 2K
--   ring line  = soft tent of (T_n - |F_n|) (T_n = (n + f) 2 L w),  band = count of rings containing the pixel
--   Mach cone  = |cross| cos(a) - along sin(a) = 0 behind the source (sin a = 1 / Mach), brightened above Mach 1
--
--   K1  Wavelength  L, 40 .. 320 virtual px (exponential)
--   K2  Heading     direction of travel, 0 .. 360 deg (plus a slow sway)
--   K3  Rate        bipolar (+-8 deadband, square law): the rings expand, or contract back into the source
--   K4  Line        ring line weight, hairline .. fat
--   K5  Colour      colouriser rotation
--   K6  Camera      luma (box filtered) adds to the ring spacing: the fronts dent round the picture
--   S7  Look        Bands (ring count as colour bands, lines as fringes) / Rings (bright lines on black)
--   S8  Pulse       Continuous / Burst (three rings in four dimmed: wave packets)
--   S9  Fade        Off / On (older rings fade toward the edge)
--   S10 Colours     Shift (doppler hue: blue ahead, red behind) / Phosphor (one scope green)
--   S11 Source      Video / Osc
--   P12 MACH        0 = at rest (a concentric target), ~40 % = Mach 0.5, ~65 % = Mach 1 (the wall),
--                   100 % = Mach 2.2 (the cone, its envelope lines brightened)
--
-- Engine: a microcoded engine (doppler_engine below: one serial multiply-accumulate, 128 x 32 register file in
-- block RAM, 512-word program from doppler.py) runs the field program in the vblank zone and then one line ahead
-- of the beam, loading the pixel DDAs in hblank.  Pixel path: DDAs, a 2^-e barrel, the 16-stage ring chain,
-- the colouriser ROM, the band fade, then the shell's output conditioning (two [1 2 2 2 2 2 2 2 1]/16 boxes,
-- luma 13/16) and the blanking gate.  Generated avid, one latency (C_LAT 40) in every mode, virtual 1080-line
-- coordinates, colours in the hardware convention (U = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.doppler_pkg.all;

entity doppler_engine is
    port (
        clk    : in  std_logic;
        start  : in  std_logic;
        ev     : in  std_logic;
        i_k1, i_k2, i_k3, i_k4, i_k5, i_k6, i_p12 : in unsigned(9 downto 0);
        i_sw   : in  std_logic_vector(4 downto 0);
        i_sx   : in  unsigned(2 downto 0);
        i_ys   : in  unsigned(11 downto 0);
        i_x0   : in  unsigned(8 downto 0);
        o_we   : out std_logic;
        o_port : out unsigned(4 downto 0);
        o_data : out std_logic_vector(31 downto 0)
    );
end entity doppler_engine;

architecture rtl of doppler_engine is
    constant S_F0  : integer := 0;
    constant S_F1  : integer := 1;
    constant S_F2  : integer := 2;
    constant S_R1  : integer := 3;
    constant S_R2  : integer := 4;
    constant S_R3  : integer := 5;
    constant S_R4  : integer := 6;
    constant S_R5  : integer := 7;
    constant S_EX  : integer := 8;
    constant S_MUL : integer := 9;
    constant S_SH  : integer := 10;
    constant S_LDC : integer := 11;
    constant S_ADD : integer := 12;
    constant S_WR  : integer := 13;
    constant S_T1  : integer := 14;
    constant S_T2  : integer := 15;
    constant S_MM  : integer := 16;
    constant S_HALT : integer := 17;   -- after configuration: idle until the first field start

    signal pc    : unsigned(8 downto 0) := (others => '0');
    signal ir_q  : std_logic_vector(47 downto 0) := (others => '0');
    signal ir    : std_logic_vector(47 downto 0) := (others => '0');
    type t_rf is array (0 to 127) of std_logic_vector(31 downto 0);
    signal rf    : t_rf := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of rf : signal is true;
    signal rf_ra, rf_wa : unsigned(6 downto 0) := (others => '0');
    signal rf_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal rf_we : std_logic := '0';
    signal rf_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal tab_s : unsigned(1 downto 0) := (others => '0');
    signal tq_e, tq_s, tq_r : std_logic_vector(15 downto 0) := (others => '0');
    signal st    : integer range 0 to 31 := S_HALT;
    signal cnt   : unsigned(4 downto 0) := (others => '0');
    signal ra    : signed(43 downto 0) := (others => '0');
    signal acc   : signed(43 downto 0) := (others => '0');
    signal rb    : std_logic_vector(20 downto 0) := (others => '0');
    signal rc    : signed(31 downto 0) := (others => '0');
    signal e     : unsigned(4 downto 0) := (others => '0');
    signal evf   : std_logic := '0';
    signal op    : unsigned(3 downto 0) := (others => '0');
    signal f_lt  : std_logic := '0';
    signal msg   : std_logic := '0';
    signal s_we  : std_logic := '0';
    signal s_port : unsigned(4 downto 0) := (others => '0');
begin
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            ir_q  <= C_DP_IROM(to_integer(pc));
            tq_e  <= C_DP_EXP(to_integer(unsigned(ra(7 downto 0))));
            tq_s  <= C_DP_SIN(to_integer(unsigned(ra(7 downto 0))));
            tq_r  <= C_DP_SQRT(to_integer(unsigned(ra(7 downto 0))));
        end if;
    end process p_rom;

    p_rf : process(clk)
    begin
        if rising_edge(clk) then
            rf_q <= rf(to_integer(rf_ra));
            if rf_we = '1' then rf(to_integer(rf_wa)) <= rf_wd; end if;
        end if;
    end process p_rf;

    p_eng : process(clk)
        variable v_in  : unsigned(11 downto 0);
        variable v_nx  : std_logic;
        variable v_tq  : std_logic_vector(15 downto 0);
        variable v_df  : signed(32 downto 0);
    begin
        if rising_edge(clk) then
            rf_we <= '0';
            s_we  <= '0';
            if ev = '1' then evf <= '1'; end if;
            -- r[c] - r[a] (33 bits): SUB's result, and MIN / MAX's compare (used a clock later)
            v_df := resize(rc, 33) - resize(ra(31 downto 0), 33);
            if v_df(32) = '0' and v_df /= 0 then f_lt <= '1'; else f_lt <= '0'; end if;
            v_nx := '0';
            if start = '1' then
                pc <= (others => '0');
                st <= S_F0;
                evf <= '0';
            else
                case st is
                    when S_F0 => st <= S_F1;
                    when S_F1 => ir <= ir_q; st <= S_F2;
                    when S_F2 =>
                        -- decode; the operands are read in order a, b (or c), c; the next instruction is prefetched
                        op <= unsigned(ir(47 downto 44));
                        rf_ra <= unsigned(ir(36 downto 30));
                        pc <= pc + 1;
                        case to_integer(unsigned(ir(47 downto 44))) is
                            when 3 | 8 | 12 => st <= S_EX;          -- LDI, IN, WAIT: no operands
                            when others     => st <= S_R1;
                        end case;
                    when S_R1 =>
                        if op = 1 or op = 10 then rf_ra <= unsigned(ir(6 downto 0)); else rf_ra <= unsigned(ir(29 downto 23)); end if;
                        st <= S_R2;
                    when S_R2 =>
                        rf_ra <= unsigned(ir(29 downto 23));        -- (r[a] lands in ra, p_alu)
                        case to_integer(op) is
                            when 4 | 7 | 9 | 13 | 14 => st <= S_EX;     -- TAB, OUT, SETE, JMP, JNZ: A only
                            when others => st <= S_R3;
                        end case;
                    when S_R3 =>
                        if op = 1 or op = 10 then
                            st <= S_R4;                             -- r[b] (lands in rb, p_alu)
                        else
                            rc <= signed(rf_q); st <= S_EX;         -- r[c]
                        end if;
                    when S_R4 => rc <= signed(rf_q); st <= S_EX;    -- r[c]
                    when S_EX =>
                        case to_integer(op) is
                            when 1 | 2 | 10 | 11 =>                 -- MAC, MACI, MACE, MACEI (acc cleared in p_alu)
                                st  <= S_MUL;
                            when 3 =>                               -- LDI
                                rf_we <= '1'; rf_wa <= unsigned(ir(43 downto 37)); rf_wd <= ir(31 downto 0);
                                st <= S_F1;
                            when 4 =>                               -- TAB
                                tab_s <= unsigned(ir(1 downto 0));
                                st <= S_T1;
                            when 5 | 6 =>                           -- MIN, MAX (compare registered here)
                                st <= S_MM;
                            when 7 =>                               -- OUT (the port data is ra, held until the next R2)
                                s_we <= '1'; s_port <= unsigned(ir(41 downto 37));
                                v_nx := '1';
                            when 8 =>                               -- IN
                                case to_integer(unsigned(ir(33 downto 30))) is
                                    when 0      => v_in := resize(i_k1, 12);
                                    when 1      => v_in := resize(i_k2, 12);
                                    when 2      => v_in := resize(i_k3, 12);
                                    when 3      => v_in := resize(i_k4, 12);
                                    when 4      => v_in := resize(i_k5, 12);
                                    when 5      => v_in := resize(i_k6, 12);
                                    when 6      => v_in := resize(i_p12, 12);
                                    when 7      => v_in := resize(unsigned(i_sw), 12);
                                    when 8      => v_in := resize(i_sx, 12);
                                    when 9      => v_in := i_ys;
                                    when others => v_in := resize(i_x0, 12);
                                end case;
                                rf_we <= '1'; rf_wa <= unsigned(ir(43 downto 37)); rf_wd <= std_logic_vector(resize(v_in, 32));
                                st <= S_F1;
                            when 9 =>                               -- SETE
                                e <= unsigned(ra(4 downto 0));
                                v_nx := '1';
                            when 12 =>                              -- WAIT
                                if evf = '1' then evf <= '0'; st <= S_F1; else st <= S_EX; end if;
                            when 13 =>                              -- JMP
                                pc <= unsigned(ir(8 downto 0)); st <= S_F0;
                            when 14 =>                              -- JNZ
                                if ra /= 0 then pc <= unsigned(ir(8 downto 0)); st <= S_F0; else v_nx := '1'; end if;
                            when 15 =>                              -- ADD: r[c] + r[a]
                                rf_we <= '1'; rf_wa <= unsigned(ir(43 downto 37));
                                rf_wd <= std_logic_vector(rc + ra(31 downto 0));
                                v_nx := '1';
                            when 0 =>                               -- SUB: r[c] - r[a]
                                rf_we <= '1'; rf_wa <= unsigned(ir(43 downto 37));
                                rf_wd <= std_logic_vector(v_df(31 downto 0));
                                v_nx := '1';
                            when others => v_nx := '1';
                        end case;
                    when S_MUL =>
                        if rb(20 downto 1) = "00000000000000000000" then st <= S_SH; end if;
                    when S_SH =>
                        if cnt = 0 then st <= S_LDC; end if;
                    when S_LDC => st <= S_ADD;
                    when S_ADD => st <= S_WR;
                    when S_WR =>
                        rf_we <= '1'; rf_wa <= unsigned(ir(43 downto 37)); rf_wd <= std_logic_vector(acc(31 downto 0));
                        v_nx := '1';
                    when S_MM =>
                        rf_we <= '1'; rf_wa <= unsigned(ir(43 downto 37));
                        if (f_lt = '1') = (op = 5) then rf_wd <= std_logic_vector(ra(31 downto 0));
                        else rf_wd <= std_logic_vector(rc); end if;
                        v_nx := '1';
                    when S_T1 => st <= S_T2;
                    when S_T2 =>
                        case to_integer(tab_s) is
                            when 0      => v_tq := tq_e;
                            when 1      => v_tq := tq_s;
                            when others => v_tq := tq_r;
                        end case;
                        rf_we <= '1'; rf_wa <= unsigned(ir(43 downto 37));
                        rf_wd <= std_logic_vector(resize(signed(v_tq), 32));
                        v_nx := '1';
                    when S_HALT => null;
                    when others => st <= S_F0;
                end case;
                if v_nx = '1' then ir <= ir_q; st <= S_F2; end if;
            end if;
        end if;
    end process p_eng;

    -- datapath: the serial multiply-accumulate, its operand registers and the shift counter, each driven by a
    -- plain state compare (kept out of the FSM's next-state cone)
    p_alu : process(clk)
        variable v_b : signed(20 downto 0);
    begin
        if rising_edge(clk) then
            if st = S_R2 then
                ra <= resize(signed(rf_q), 44);
            elsif st = S_MUL then
                ra <= shift_left(ra, 1);
            elsif st = S_LDC then
                ra <= resize(rc, 44);
            end if;
            if st = S_R3 then
                if op = 1 or op = 10 then v_b := signed(rf_q(20 downto 0));
                else v_b := resize(signed(ir(17 downto 0)), 21); end if;
                if v_b(20) = '1' then rb <= std_logic_vector(-v_b); msg <= '1';
                else rb <= std_logic_vector(v_b); msg <= '0'; end if;
            elsif st = S_MUL then
                rb <= '0' & rb(20 downto 1);
            end if;
            -- sign-magnitude: acc +- A for every set bit of |B|; the FSM leaves S_MUL when no set bit is left
            if st = S_EX then
                acc <= (others => '0');
            elsif st = S_MUL then
                if rb(0) = '1' then
                    if msg = '1' then acc <= acc - ra; else acc <= acc + ra; end if;
                end if;
            elsif st = S_SH then
                if cnt /= 0 then acc <= shift_right(acc, 1); end if;
            elsif st = S_ADD then
                acc <= acc + ra;
            end if;
            if st = S_MUL then
                if op = 10 or op = 11 then cnt <= e; else cnt <= unsigned(ir(22 downto 18)); end if;
            elsif st = S_SH and cnt /= 0 then
                cnt <= cnt - 1;
            end if;
        end if;
    end process p_alu;

    o_we   <= s_we;
    o_port <= s_port;
    o_data <= std_logic_vector(ra(31 downto 0));
end architecture rtl;

-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.doppler_pkg.all;
use work.all;

architecture doppler of program_top is
    constant C_LAT : integer := 36;   -- = processing_delay_clks
    constant NR : integer := 12;      -- rings
    constant FW : integer := 25;      -- ring chain F width (units of 2^e h^2)
    constant JW : integer := 22;      -- ring chain J width

    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 12);
            when 4          => return to_unsigned(576, 12);
            when 8          => return to_unsigned(960, 12);
            when 12         => return to_unsigned(480, 12);
            when 5 | 6 | 14 => return to_unsigned(384, 12);
            when 1 | 2 | 10 => return to_unsigned(512, 12);
            when others     => return to_unsigned(256, 12);
        end case;
    end function;
    -- x step in h per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- x * m for a 2-bit m, unsigned (one partial-product pair of the brightness multiplier)
    function f_pu(x : unsigned(7 downto 0); m : unsigned(1 downto 0)) return unsigned is
        variable r : unsigned(12 downto 0);
    begin
        case m is
            when "00"   => r := (others => '0');
            when "01"   => r := resize(x, 13);
            when "10"   => r := shift_left(resize(x, 13), 1);
            when others => r := resize(x, 13) + shift_left(resize(x, 13), 1);
        end case;
        return r;
    end function;
    -- x * m for a 2-bit m (one partial-product pair of the brightness multiplier)
    function f_pp(x : signed(10 downto 0); m : unsigned(1 downto 0)) return signed is
        variable r : signed(15 downto 0);
    begin
        case m is
            when "00"   => r := (others => '0');
            when "01"   => r := resize(x, 16);
            when "10"   => r := shift_left(resize(x, 16), 1);
            when others => r := resize(x, 16) + shift_left(resize(x, 16), 1);
        end case;
        return r;
    end function;
    -- per-ring ramp shift (the line's soft edge in chain units)
    type t_sn is array (0 to NR - 1) of integer;
    constant C_SN : t_sn := (0, 0, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3);

    -- camera path: cell store (one 12-bit vertical-slew state per 8-pixel cell)
    type t_lb is array (0 to 511) of std_logic_vector(15 downto 0);
    signal lb : t_lb := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    signal s_lb_wa, s_lb_ra : unsigned(8 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(15 downto 0) := x"0000";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(15 downto 0) := x"0000";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');
    signal s_lin   : unsigned(7 downto 0) := (others => '0');
    signal s_lav   : std_logic := '0';
    signal s_lc    : unsigned(2 downto 0) := (others => '0');
    signal s_lcell, s_fcell, s_wcell : unsigned(8 downto 0) := (others => '0');
    signal s_acc   : unsigned(10 downto 0) := (others => '0');
    signal s_sum8  : unsigned(10 downto 0) := (others => '0');
    signal s_cur, s_old, s_new : unsigned(11 downto 0) := (others => '0');
    signal s_f1, s_f2 : std_logic := '0';
    signal s_lv    : unsigned(11 downto 0) := (others => '0');
    signal s_h1, s_h2 : unsigned(15 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_x0   : unsigned(8 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_c2   : unsigned(11 downto 0) := to_unsigned(512, 12);

    -- knobs: glided (Q3), raw rate, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(380 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(960 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(500 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(0, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(560 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_sw : std_logic_vector(4 downto 0) := (others => '0');
    signal s_rings, s_burst, s_fade, s_phos, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- engine ports
    signal e_start, e_ev : std_logic := '0';
    signal e_we : std_logic;
    signal e_port : unsigned(4 downto 0);
    signal e_data : std_logic_vector(31 downto 0);
    signal c_d0  : signed(23 downto 0) := (others => '0');
    signal c_hs  : signed(23 downto 0) := (others => '0');
    signal c_e   : unsigned(12 downto 0) := (others => '0');
    signal c_k2  : signed(JW - 1 downto 0) := (others => '0');
    signal c_ec  : unsigned(4 downto 0) := to_unsigned(12, 5);
    signal c_ecf : unsigned(4 downto 0) := to_unsigned(3, 5);
    signal c_hus, c_crs, c_als : signed(15 downto 0) := (others => '0');
    signal c_t0  : unsigned(8 downto 0) := (others => '0');
    signal c_t1s : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal c_capl : unsigned(3 downto 0) := (others => '1');
    signal c_wm  : unsigned(1 downto 0) := (others => '0');
    signal c_fe  : unsigned(4 downto 0) := (others => '0');
    signal c_we  : unsigned(1 downto 0) := (others => '0');
    signal c_cenv : unsigned(3 downto 0) := (others => '0');

    -- per-ring constants (shift-loaded after the field program): threshold, brightness cap, packet bit
    type t_t9 is array (0 to NR - 1) of unsigned(8 downto 0);
    type t_u4 is array (0 to NR - 1) of unsigned(3 downto 0);
    signal st_t : t_t9 := (others => (others => '0'));
    signal st_pk : std_logic_vector(NR - 1 downto 0) := (others => '1');
    signal ld_run : std_logic := '0';
    signal ld_n : unsigned(3 downto 0) := (others => '0');
    signal ld_t : unsigned(8 downto 0) := (others => '0');

    -- pixel DDAs
    signal d_p : signed(31 downto 0) := (others => '0');
    signal d_d : signed(23 downto 0) := (others => '0');
    signal d_h : signed(31 downto 0) := (others => '0');
    signal d_hu, d_cr, d_al : signed(23 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- pixel path: barrel, camera product
    signal p2_p, p2_h : signed(31 downto 0) := (others => '0');
    signal p3_p : signed(FW - 1 downto 0) := (others => '0');
    signal p3_h : signed(JW - 1 downto 0) := (others => '0');
    signal p2_lo, p2_hi : unsigned(16 downto 0) := (others => '0');
    signal p3_le : unsigned(12 downto 0) := (others => '0');

    -- ring chain
    type t_f is array (0 to NR - 1) of signed(FW - 1 downto 0);
    type t_j is array (0 to NR - 1) of signed(JW - 1 downto 0);
    signal c_f : t_f := (others => (others => '0'));
    signal c_j : t_j := (others => (others => '0'));
    signal r1_a : t_t9 := (others => (others => '0'));
    signal r1_z, r2_v : std_logic_vector(NR - 1 downto 0) := (others => '0');
    signal v_dl1, v_dl2, v_dl3 : unsigned(3 downto 0) := (others => '0');
    signal r2_x : t_t9 := (others => (others => '0'));
    signal r3_b : t_u4 := (others => (others => '0'));
    type t_v4 is array (1 to NR) of unsigned(3 downto 0);
    type t_b5 is array (1 to NR) of unsigned(4 downto 0);
    signal a_v : t_v4 := (others => (others => '0'));
    signal a_b : t_b5 := (others => (others => '0'));

    -- colour path
    signal q24_u7 : unsigned(6 downto 0) := (others => '0');
    signal q24_e  : signed(24 downto 0) := (others => '0');
    signal q24_alok : std_logic := '0';
    signal q24_band, q25_band, q26_band, q27_band, q28_band : unsigned(1 downto 0) := (others => '0');
    signal q24_d : signed(10 downto 0) := (others => '0');
    signal q24_b, q25_b, q26_b : unsigned(4 downto 0) := (others => '0');
    signal q25_hf : unsigned(7 downto 0) := (others => '0');
    signal q25_ae : unsigned(5 downto 0) := (others => '0');
    signal q25_near : std_logic := '0';
    signal q25_fi : unsigned(5 downto 0) := (others => '0');
    signal q26_h, q27_h, q28_h : unsigned(5 downto 0) := (others => '0');
    signal q26_be, q27_be : unsigned(3 downto 0) := (others => '0');
    signal q26_f4, q27_f4, q28_f4 : unsigned(3 downto 0) := (others => '0');
    signal q27_bm, q27_bt : unsigned(4 downto 0) := (others => '0');
    signal q28_ln : unsigned(1 downto 0) := (others => '0');
    signal q28_tot : unsigned(5 downto 0) := (others => '0');
    signal r_col_a : unsigned(9 downto 0) := (others => '0');
    signal r_col_q, q30_c : std_logic_vector(23 downto 0) := (others => '0');
    signal q29_fd, q30_fd, q31_fd, q32_fd : unsigned(4 downto 0) := (others => '0');
    signal q32_ya, q32_yb, q32_ua, q32_ub, q32_va, q32_vb : unsigned(12 downto 0) := (others => '0');
    signal q33_y, q33_u, q33_v : unsigned(12 downto 0) := (others => '0');
    signal q33_oy, q33_oc : unsigned(9 downto 0) := (others => '0');
    signal s_y34, s_u34, s_v34 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y35, s_u35, s_v35 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal x25 : unsigned(1 downto 0) := (others => '0');

    -- output conditioning (box delay lines are 16-deep block-RAM rings)
    type t_dl is array (0 to 15) of std_logic_vector(31 downto 0);
    signal da, db : t_dl := (others => (others => '0'));
    attribute no_rw_check of da : signal is true;
    attribute no_rw_check of db : signal is true;
    signal s_dp : unsigned(3 downto 0) := to_unsigned(0, 4);
    signal s_dr : unsigned(3 downto 0) := to_unsigned(10, 4);
    signal s_qa, s_qb, s_fa, s_fb : std_logic_vector(31 downto 0) := (others => '0');
    signal s_bay, s_bau, s_bav : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_day, s_dau, s_dav, s_dby, s_dbu, s_dbv : signed(10 downto 0) := (others => '0');
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y37, s_u37, s_v37 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y39, s_u39, s_v39 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y40, s_u40, s_v40 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y41 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u41, s_v41 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel
    -- hsync / vsync / field delayed by C_LAT in a block-RAM ring (read C_LAT - 2 behind the write, plus a fabric
    -- register); no read ever meets the write address
    type t_sd is array (0 to 255) of std_logic_vector(3 downto 0);
    signal sd : t_sd := (others => "0111");
    attribute no_rw_check of sd : signal is true;
    signal sd_wp : unsigned(7 downto 0) := (others => '0');
    signal sd_rp : unsigned(7 downto 0) := to_unsigned(256 - (C_LAT - 2), 8);
    signal sd_q, sd_f : std_logic_vector(3 downto 0) := "0111";

begin

    ---------------------------------------------------------------------------
    -- Microcoded engine (field program in the vblank zone, then one line ahead)
    ---------------------------------------------------------------------------
    u_eng : entity work.doppler_engine
        port map (
            clk => clk, start => e_start, ev => e_ev,
            i_k1 => s_g1(12 downto 3), i_k2 => s_g2(12 downto 3), i_k3 => unsigned(registers_in(2)), i_k4 => s_g4(12 downto 3),
            i_k5 => s_g5(12 downto 3), i_k6 => s_g6(12 downto 3), i_p12 => s_gp(12 downto 3),
            i_sw => s_sw, i_sx => s_xs, i_ys => s_ys, i_x0 => s_x0,
            o_we => e_we, o_port => e_port, o_data => e_data);

    ---------------------------------------------------------------------------
    -- Camera cell store and the colouriser ROM
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_col : process(clk)
    begin
        if rising_edge(clk) then
            r_col_q <= C_DP_COL(to_integer(r_col_a));
        end if;
    end process p_col;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glide, switches, engine start
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
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
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_sw    <= registers_in(6)(4 downto 0);
                s_rings <= registers_in(6)(0);
                s_burst <= registers_in(6)(1);
                s_fade  <= registers_in(6)(2);
                s_phos  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9); s_c2 <= to_unsigned(2048, 12);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9); s_c2 <= to_unsigned(1152, 12);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9); s_c2 <= to_unsigned(512, 12);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- engine: restarted at every field start (the zone start); line event after the last DDA user
            e_start <= s_zgo;
            e_ev <= s_gav_sr(20) and not s_gav_sr(19);

            -- shared glide (1/8 of the gap per field, +-2 deadband): K1, K2, K4, K5, K6, P12
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g4; s_gr <= unsigned(registers_in(3));
                            when 3      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 4      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g4 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Engine output ports, the per-ring constant loader, the pixel DDAs
    ---------------------------------------------------------------------------
    p_ports : process(clk)
        variable v_pk : std_logic;
    begin
        if rising_edge(clk) then
            if e_we = '1' then
                case to_integer(e_port) is
                    when 5  => c_d0  <= signed(e_data(23 downto 0));
                    when 6  => c_hs  <= signed(e_data(23 downto 0));
                    when 7  => c_e   <= unsigned(e_data(12 downto 0));
                    when 8  => c_k2  <= resize(signed(e_data(17 downto 0)), JW);
                    when 9  => c_ec  <= unsigned(e_data(4 downto 0));
                    when 10 => c_hus <= signed(e_data(15 downto 0));
                    when 11 => c_crs <= signed(e_data(15 downto 0));
                    when 12 => c_als <= signed(e_data(15 downto 0));
                    when 13 => c_t0  <= unsigned(e_data(8 downto 0));
                    when 14 => c_t1s <= unsigned(e_data(5 downto 0));
                    when 15 => c_capl <= unsigned(e_data(3 downto 0));
                    when 16 => c_wm  <= unsigned(e_data(1 downto 0));
                    when 17 => c_fe  <= unsigned(e_data(4 downto 0));
                    when 18 => c_we  <= unsigned(e_data(1 downto 0));
                    when 19 => c_cenv <= unsigned(e_data(3 downto 0));
                    when others => null;
                end case;
            end if;

            -- per-ring constants: n = NR - 1 first (it ends in st_x(NR - 1)): T_n = T0 + n T1s, the packet bit
            if e_we = '1' and e_port = 20 then
                ld_run <= '1'; ld_n <= to_unsigned(NR - 1, 4);
                ld_t <= c_t0 + shift_left(resize(c_t1s, 9), 3) + shift_left(resize(c_t1s, 9), 1) + resize(c_t1s, 9);  -- + 11 T1s
            elsif ld_run = '1' then
                if s_burst = '1' and (ld_n(1 downto 0) - c_wm) /= "00" then v_pk := '0'; else v_pk := '1'; end if;
                st_t(1 to NR - 1) <= st_t(0 to NR - 2); st_t(0) <= ld_t;
                st_pk(NR - 1 downto 1) <= st_pk(NR - 2 downto 0); st_pk(0) <= v_pk;
                ld_t <= ld_t - resize(c_t1s, 9);
                ld_n <= ld_n - 1;
                if ld_n = 0 then ld_run <= '0'; end if;
            end if;

            -- stage 1 DDAs: P (second order) and H, loaded by the engine in hblank, stepping with the generated avid
            if e_we = '1' and e_port = 0 then
                d_p <= signed(e_data);
            elsif g_avid = '1' then
                d_p <= d_p + resize(d_d, 32);
            end if;
            if g_start = '1' then
                d_d <= c_d0;
            elsif g_avid = '1' then
                d_d <= d_d + signed(resize(s_c2, 24));
            end if;
            if e_we = '1' and e_port = 1 then
                d_h <= signed(e_data);
            elsif g_avid = '1' then
                d_h <= d_h + resize(c_hs, 32);
            end if;
            -- stage 23 DDAs: hue and the Mach-cone pair
            if e_we = '1' and e_port = 2 then
                d_hu <= signed(e_data(23 downto 0));
            elsif s_gav_sr(17) = '1' then
                d_hu <= d_hu + resize(c_hus, 24);
            end if;
            if e_we = '1' and e_port = 3 then
                d_cr <= signed(e_data(23 downto 0));
            elsif s_gav_sr(17) = '1' then
                d_cr <= d_cr + resize(c_crs, 24);
            end if;
            if e_we = '1' and e_port = 4 then
                d_al <= signed(e_data(23 downto 0));
            elsif s_gav_sr(17) = '1' then
                d_al <= d_al + resize(c_als, 24);
            end if;
        end if;
    end process p_ports;

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 3/4 clocks after the source's avid
    -- rise, field-latched width)
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
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- Source side: oscillator (2-D triangle lattice, drifting) or camera luma
    -- ((Y >> 2) - 16), averaged over 8-pixel cells, each cell slewed down the
    -- screen (one pole, 1/32 per line; the state lives in the cell store), then
    -- slewed across by two one-pole filters (1/16 per pixel).  All three
    -- filters decay, so none needs a resync.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_e : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- 1: sample
            if s_src = '1' then
                s_lin <= s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lin <= v_l - 16; else s_lin <= (others => '0'); end if;
            end if;
            s_lav   <= data_in.avid;
            s_lc    <= s_wcol(2 downto 0);
            s_lcell <= resize(s_wcol(10 downto 3), 9);
            if data_in.avid = '1' then
                s_wcol  <= s_wcol + 1;
                s_lb_ra <= resize(s_wcol(10 downto 3), 9) + 1;
            else
                s_wcol  <= (others => '0');
                s_lb_ra <= (others => '0');
            end if;
            -- 2: sum eight; at the eighth, take the cell's state (read a cell ahead) for the update
            s_f1 <= '0';
            if s_lav = '0' then
                s_cur <= unsigned(s_lb_q(11 downto 0));
            else
                if s_lc = 0 then s_acc <= resize(s_lin, 11); else s_acc <= s_acc + s_lin; end if;
                if s_lc = 7 then
                    s_sum8  <= s_acc + s_lin;
                    s_old   <= s_cur;
                    s_cur   <= unsigned(s_lb_q(11 downto 0));
                    s_fcell <= s_lcell;
                    s_f1    <= '1';
                end if;
            end if;
            -- 3: state += (cell average * 16 - state) / 32
            v_d := signed(resize(s_sum8 & '0', 13)) - signed(resize(s_old, 13));
            v_d := signed(resize(s_old, 13)) + shift_right(v_d, 5);
            s_new   <= unsigned(v_d(11 downto 0));
            s_wcell <= s_fcell;
            s_f2    <= s_f1;
            -- 4: write
            s_lb_we <= s_f2;
            s_lb_wa <= s_wcell;
            s_lb_wd <= "0000" & std_logic_vector(s_new);

            -- pixel side: the cell stream (the line above's state), two slews across
            s_lv <= unsigned(s_lb_q(11 downto 0));
            v_e  := signed(resize(s_lv & "0000", 17)) - signed(resize(s_h1, 17));
            v_e  := signed(resize(s_h1, 17)) + shift_right(v_e, 4);
            s_h1 <= unsigned(v_e(15 downto 0));
            v_e  := signed(resize(s_h1, 17)) - signed(resize(s_h2, 17));
            v_e  := signed(resize(s_h2, 17)) + shift_right(v_e, 4);
            s_h2 <= unsigned(v_e(15 downto 0));
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Pixel path: barrel (2^-e), camera term, the ring chain, the colouriser
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_fi  : signed(FW - 1 downto 0);
        variable v_x   : signed(9 downto 0);
        variable v_b   : unsigned(3 downto 0);
        variable v_t   : unsigned(8 downto 0);
        variable v_s6  : unsigned(5 downto 0);
        variable v_e   : signed(24 downto 0);
        variable v_u   : signed(23 downto 0);
        variable v_d   : signed(10 downto 0);
        variable v_fd  : signed(7 downto 0);
        variable v_h   : unsigned(7 downto 0);
        variable v_ae  : signed(24 downto 0);
        variable v_be  : unsigned(5 downto 0);
        variable v_ln  : unsigned(8 downto 0);
        variable v_tot : unsigned(5 downto 0);
        variable v_rl  : unsigned(6 downto 0);
        variable v_f5  : unsigned(4 downto 0);
        variable v_s   : signed(15 downto 0);
        variable v_o   : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: coarse shift (0 / 8 / 16) of P and H; luma nibble products
            if c_ec >= 17 then p2_p <= shift_right(d_p, 17); p2_h <= shift_right(d_h, 17);
            else p2_p <= shift_right(d_p, 9); p2_h <= shift_right(d_h, 9); end if;
            c_ecf <= c_ec - 9;
            p2_lo <= resize(s_h2(11 downto 8) * c_e, 17);
            p2_hi <= resize(s_h2(15 downto 12) * c_e, 17);
            -- 3: fine shift (0 .. 7); camera term = (l E) >> 4
            p3_p <= resize(shift_right(p2_p, to_integer(c_ecf(2 downto 0))), FW);
            p3_h <= resize(shift_right(p2_h, to_integer(c_ecf(2 downto 0))), JW);
            p3_le <= resize(shift_right(resize(p2_lo, 21) + shift_left(resize(p2_hi, 21), 4), 8), 13);
            -- 4: F_0, J_0
            c_f(0) <= p3_p;
            c_j(0) <= p3_h + signed(resize(p3_le, JW));

            -- the ring chain: stage 4 + n holds F_n, J_n; ring n: the count bit (sign of F_n) is added at once,
            -- the line: R1 (stage 5 + n) |F_n|, R2 (6 + n) T_n - |F_n|, R3 (7 + n) brightness, summed at 8 + n
            for n in 0 to NR - 1 loop
                if n < NR - 1 then
                    c_f(n + 1) <= c_f(n) + resize(c_j(n), FW);
                    c_j(n + 1) <= c_j(n) + c_k2;
                end if;
                -- band count (stage 4 + n -> 5 + n)
                if n = 0 then
                    a_v(1) <= "000" & (c_f(0)(FW - 1) and st_pk(0));
                else
                    a_v(n + 1) <= a_v(n) + ("000" & (c_f(n)(FW - 1) and st_pk(n)));
                end if;
                -- R1
                v_fi := c_f(n);
                if v_fi(FW - 1) = '1' then v_fi := not v_fi; end if;
                if v_fi(FW - 1 downto 9) = 0 then r1_z(n) <= '1'; else r1_z(n) <= '0'; end if;
                r1_a(n) <= unsigned(v_fi(8 downto 0));
                -- R2
                v_x := signed(resize(st_t(n), 10)) - signed(resize(r1_a(n), 10));
                if r1_z(n) = '1' and v_x > 0 then r2_v(n) <= '1'; else r2_v(n) <= '0'; end if;
                r2_x(n) <= unsigned(v_x(8 downto 0));
                -- R3 (Burst: rings off the packet at a quarter; the oldest ring fades out over its last wrap)
                v_t := shift_right(r2_x(n), C_SN(n));
                if r2_v(n) = '0' then v_b := (others => '0');
                elsif v_t(8 downto 4) /= 0 then v_b := "1111";
                else v_b := v_t(3 downto 0); end if;
                if st_pk(n) = '0' then v_b := shift_right(v_b, 2); end if;
                if n = NR - 1 and v_b > c_capl then v_b := c_capl; end if;
                r3_b(n) <= v_b;
                -- R4 (the line sum saturates at 31)
                if n = 0 then
                    a_b(1) <= resize(r3_b(0), 5);
                else
                    v_s6 := resize(a_b(n), 6) + resize(r3_b(n), 6);
                    if v_s6(5) = '1' then a_b(n + 1) <= "11111"; else a_b(n + 1) <= v_s6(4 downto 0); end if;
                end if;
            end loop;
            -- the count is ready at stage 4 + NR, the line sum at 7 + NR: delay the count by 3
            v_dl1 <= a_v(NR); v_dl2 <= v_dl1; v_dl3 <= v_dl2;

            -- (stage numbers below are the 16-ring ones minus 4: the totals arrive at stage 19)
            -- 19 -> 20: count, line sum, doppler coordinate, the cone distance e' = |cross'| - along'
            v_u := shift_right(d_hu, 11);
            if v_u > 63 then q24_u7 <= to_unsigned(127, 7);
            elsif v_u < -64 then q24_u7 <= to_unsigned(0, 7);
            else q24_u7 <= unsigned(v_u(6 downto 0)) + 64; end if;
            if d_cr(23) = '1' then v_e := resize(not d_cr, 25); else v_e := resize(d_cr, 25); end if;
            q24_e <= v_e - resize(d_al, 25);
            if d_al > to_signed(-16384, 24) then q24_alok <= '1'; else q24_alok <= '0'; end if;
            if s_phos = '1' and (v_dl3(1 downto 0) + c_we) = "11" then q24_band <= "01";
            else q24_band <= v_dl3(1 downto 0) + c_we; end if;
            -- band age for the fade (Rings: the band outside the line, so a line is not cut in half)
            if s_rings = '1' then
                q24_d <= signed(shift_left(resize(v_dl3, 11) + 1, 5)) - signed(resize(c_fe, 11));
            else
                q24_d <= signed(shift_left(resize(v_dl3, 11), 5)) - signed(resize(c_fe, 11));
            end if;
            q24_b <= a_b(NR);
            -- 21: hue (8 bits), |e'| (64 = the cone line's half-width), fade index
            if s_phos = '1' then q25_hf <= to_unsigned(84, 8); else q25_hf <= C_DP_HTAB(to_integer(q24_u7)); end if;
            if q24_e(24) = '1' then v_ae := not q24_e; else v_ae := q24_e; end if;
            if v_ae(23 downto 14) = 0 then q25_ae <= unsigned(v_ae(13 downto 8)); q25_near <= q24_alok;
            else q25_ae <= (others => '1'); q25_near <= '0'; end if;
            v_d := shift_right(q24_d, 3) + 2;
            if v_d < 0 then q25_fi <= (others => '0');
            elsif v_d > 63 then q25_fi <= to_unsigned(63, 6);
            else q25_fi <= unsigned(v_d(5 downto 0)); end if;
            q25_band <= q24_band; q25_b <= q24_b;
            -- 22: hue index (rotation and a 4-phase horizontal dither); the cone line's brightness; band fade
            if s_gav_sr(19) = '1' then x25 <= x25 + 1; else x25 <= (others => '0'); end if;
            case to_integer(x25) is
                when 0      => v_h := q25_hf + s_g5(12 downto 5);
                when 1      => v_h := q25_hf + s_g5(12 downto 5) + 2;
                when 2      => v_h := q25_hf + s_g5(12 downto 5) + 1;
                when others => v_h := q25_hf + s_g5(12 downto 5) + 3;
            end case;
            q26_h <= v_h(7 downto 2);
            if q25_near = '1' then
                v_be := resize(shift_right(to_unsigned(64, 7) - resize(q25_ae, 7), 1), 6);
                if v_be > 15 then v_be := to_unsigned(15, 6); end if;
            else
                v_be := (others => '0');
            end if;
            if v_be(3 downto 0) > c_cenv then q26_be <= c_cenv; else q26_be <= v_be(3 downto 0); end if;
            q26_f4 <= C_DP_FADE(to_integer(s_fade & q25_fi));
            q26_band <= q25_band; q26_b <= q25_b;
            -- 23: line sum scaled (Bands) and clipped (Rings)
            q27_bm <= resize(shift_right(resize(q26_b, 9) * to_unsigned(13, 4), 4), 5);
            if q26_b > 24 then q27_bt <= to_unsigned(24, 5); else q27_bt <= q26_b; end if;
            q27_h <= q26_h; q27_f4 <= q26_f4; q27_band <= q26_band; q27_be <= q26_be;
            -- 24: line level (Bands) or ring total (Rings)
            v_ln := resize(q27_bm, 9) + resize(q27_be, 9);
            if v_ln > 15 then q28_ln <= "11"; else q28_ln <= v_ln(3 downto 2); end if;
            q28_tot <= resize(q27_bt, 6) + resize(q27_be, 6) + resize(shift_right(q27_be, 2), 6);
            q28_h <= q27_h; q28_f4 <= q27_f4; q28_band <= q27_band;
            -- 25: colouriser address and brightness multiplier
            if q28_f4 = 15 then v_f5 := to_unsigned(16, 5); else v_f5 := resize(q28_f4, 5); end if;
            if s_rings = '1' then
                v_rl := C_DP_RLEV(to_integer(q28_tot));
                r_col_a <= q28_h & "01" & v_rl(1 downto 0);
                if v_rl(6 downto 2) > v_f5 then q29_fd <= v_f5; else q29_fd <= v_rl(6 downto 2); end if;
            else
                r_col_a <= q28_h & q28_band & q28_ln;
                if q28_f4 = 15 then q29_fd <= to_unsigned(16, 5); else q29_fd <= resize(q28_f4, 5); end if;
            end if;
            -- 26: ROM data (EBR); 27: fabric copy; 28: offsets removed; 29: partial products; 30: product
            q30_fd <= q29_fd;
            q30_c  <= r_col_q;
            q31_fd <= q30_fd;
            q32_fd <= q31_fd;
            -- brightness f = 0 .. 16 on unsigned 8-bit colours (no sign extension, so no self-add for router1):
            -- out = 64 - 4 f + (Y8 f) >> 2 (luma), 512 - 32 f + (C8 f) >> 2 (chroma), exactly 64 + ((Y - 64) f) >> 4
            q32_ya <= f_pu(unsigned(q30_c(23 downto 16)), q31_fd(1 downto 0));
            q32_ua <= f_pu(unsigned(q30_c(15 downto 8)), q31_fd(1 downto 0));
            q32_va <= f_pu(unsigned(q30_c(7 downto 0)), q31_fd(1 downto 0));
            if q31_fd(4) = '1' then
                q32_yb <= shift_left(resize(unsigned(q30_c(23 downto 16)), 13), 4);
                q32_ub <= shift_left(resize(unsigned(q30_c(15 downto 8)), 13), 4);
                q32_vb <= shift_left(resize(unsigned(q30_c(7 downto 0)), 13), 4);
            else
                q32_yb <= shift_left(f_pu(unsigned(q30_c(23 downto 16)), q31_fd(3 downto 2)), 2);
                q32_ub <= shift_left(f_pu(unsigned(q30_c(15 downto 8)), q31_fd(3 downto 2)), 2);
                q32_vb <= shift_left(f_pu(unsigned(q30_c(7 downto 0)), q31_fd(3 downto 2)), 2);
            end if;
            q33_y <= q32_ya + q32_yb; q33_u <= q32_ua + q32_ub; q33_v <= q32_va + q32_vb;
            q33_oy <= to_unsigned(64, 10) - shift_left(resize(q32_fd, 10), 2);
            q33_oc <= to_unsigned(512, 10) - shift_left(resize(q32_fd, 10), 5);
            s_y34 <= q33_oy + resize(shift_right(q33_y, 2), 10);
            s_u34 <= q33_oc + resize(shift_right(q33_u, 2), 10);
            s_v34 <= q33_oc + resize(shift_right(q33_v, 2), 10);
            -- 31: compose: black outside the generated avid
            if s_gav_sr(28) = '1' then
                s_y35 <= s_y34; s_u35 <= s_u34; s_v35 <= s_v34;
            else
                s_y35 <= to_unsigned(64, 10); s_u35 <= to_unsigned(512, 10); s_v35 <= to_unsigned(512, 10);
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(29);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Sync delay ring
    ---------------------------------------------------------------------------
    p_sd : process(clk)
    begin
        if rising_edge(clk) then
            sd_wp <= sd_wp + 1; sd_rp <= sd_rp + 1;
            sd(to_integer(sd_wp)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            sd_q <= sd(to_integer(sd_rp));
            sd_f <= sd_q;
        end if;
    end process p_sd;

    ---------------------------------------------------------------------------
    -- Output filter stores
    ---------------------------------------------------------------------------
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            s_dp <= s_dp + 1; s_dr <= s_dr + 1;
            s_qa <= da(to_integer(s_dr));
            da(to_integer(s_dp)) <= "00" & std_logic_vector(s_y35) & std_logic_vector(s_u35) & std_logic_vector(s_v35);
            s_qb <= db(to_integer(s_dr));
            db(to_integer(s_dp)) <= "00" & std_logic_vector(s_y37) & std_logic_vector(s_u37) & std_logic_vector(s_v37);
            s_fa <= s_qa; s_fb <= s_qb;                 -- = the box inputs 8 pixels back
        end if;
    end process p_dl;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the HD HDMI bar fix), luma 13/16, the
    -- blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : signed(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- Each box: D = x - x(8 back), then S += D and out = (2 S + D) / 16 = (S_new + S_old) / 16, i.e.
            -- [1 2 2 2 2 2 2 2 1]/16.  The running 8-sums snap back to black (the off-avid input of both boxes)
            -- once 8 blank samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 32 differences, 33 sums and output
            s_day <= signed(resize(s_y35, 11)) - signed(resize(unsigned(s_fa(29 downto 20)), 11));
            s_dau <= signed(resize(s_u35, 11)) - signed(resize(unsigned(s_fa(19 downto 10)), 11));
            s_dav <= signed(resize(s_v35, 11)) - signed(resize(unsigned(s_fa(9 downto 0)), 11));
            if s_bah = "00000000" then
                s_bay <= to_unsigned(64 * 8, 13); s_bau <= to_unsigned(512 * 8, 13); s_bav <= to_unsigned(512 * 8, 13);
            else
                s_bay <= unsigned(signed(s_bay) + resize(s_day, 13));
                s_bau <= unsigned(signed(s_bau) + resize(s_dau, 13));
                s_bav <= unsigned(signed(s_bav) + resize(s_dav, 13));
            end if;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(30) = '1' then
                v_sc := signed(shift_left(resize(s_bay, 15), 1)) + resize(s_day, 15); s_y37 <= unsigned(v_sc(13 downto 4));
                v_sc := signed(shift_left(resize(s_bau, 15), 1)) + resize(s_dau, 15); s_u37 <= unsigned(v_sc(13 downto 4));
                v_sc := signed(shift_left(resize(s_bav, 15), 1)) + resize(s_dav, 15); s_v37 <= unsigned(v_sc(13 downto 4));
            else
                s_y37 <= to_unsigned(64, 10); s_u37 <= to_unsigned(512, 10); s_v37 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(31);
            -- box B: stage 34 differences, 35 sums and output
            s_dby <= signed(resize(s_y37, 11)) - signed(resize(unsigned(s_fb(29 downto 20)), 11));
            s_dbu <= signed(resize(s_u37, 11)) - signed(resize(unsigned(s_fb(19 downto 10)), 11));
            s_dbv <= signed(resize(s_v37, 11)) - signed(resize(unsigned(s_fb(9 downto 0)), 11));
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(64 * 8, 13); s_bbu <= to_unsigned(512 * 8, 13); s_bbv <= to_unsigned(512 * 8, 13);
            else
                s_bby <= unsigned(signed(s_bby) + resize(s_dby, 13));
                s_bbu <= unsigned(signed(s_bbu) + resize(s_dbu, 13));
                s_bbv <= unsigned(signed(s_bbv) + resize(s_dbv, 13));
            end if;
            v_sc := signed(shift_left(resize(s_bby, 15), 1)) + resize(s_dby, 15); s_y39 <= unsigned(v_sc(13 downto 4));
            v_sc := signed(shift_left(resize(s_bbu, 15), 1)) + resize(s_dbu, 15); s_u39 <= unsigned(v_sc(13 downto 4));
            v_sc := signed(shift_left(resize(s_bbv, 15), 1)) + resize(s_dbv, 15); s_v39 <= unsigned(v_sc(13 downto 4));
            -- 36: luma 13/16
            if s_y39 > 64 then v_dl := s_y39 - 64; else v_dl := (others => '0'); end if;
            s_y40 <= s_y39 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u40 <= s_u39; s_v40 <= s_v39;
            -- 37: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y41 <= s_y40; s_u41 <= s_u40; s_v41 <= s_v40;
            else
                s_y41 <= to_unsigned(64, 10); s_u41 <= to_unsigned(512, 10); s_v41 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y41);
    data_out.u       <= std_logic_vector(s_u41);
    data_out.v       <= std_logic_vector(s_v41);
    data_out.hsync_n <= sd_f(0);
    data_out.vsync_n <= sd_f(1);
    data_out.field_n <= sd_f(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture doppler;
