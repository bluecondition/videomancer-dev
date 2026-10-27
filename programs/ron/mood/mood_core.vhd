-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- mood_core.vhd -- MOOD: a circuit-level Atari Video Music (C240, 1976).
--
-- Shared by MOOD (G_CLEAN = false, VINTAGE render) and MOOD HD (G_CLEAN =
-- true, CLEAN render).  No frame buffer: everything is computed in raster
-- order from the beam position, per-line state and per-frame latches.
--
--  1. VIRTUAL NTSC TIMEBASE.  A 228-clock x 262-line virtual raster is mapped
--     onto every output format: VIS_CLK (208) virtual clocks across the
--     active width and 200 virtual lines (31..230) down the active height,
--     with steps from the timing ID, so the composition is identical at SD,
--     720p and 1080p (4:3 SD squeezes the tiles horizontally).
--  2. ENGINE.  DAMIER's microcoded engine (16-bit, Booth multiplier, ROM
--     unit, two-copy EBR register file, 1024-word program) runs the sample
--     program (mood_ucode.py) once per virtual line, 15.734 kHz in REAL time
--     (858 / 1716 / 4719 pixel clocks per tick): phantom stereo or Path B
--     source, detectors, integrators (CONTOUR), the alternating zero-crossing
--     counter, Table I, the colour modulator, SHAPE / AUTO.  Each tick
--     restarts it from PC 0 (a stuck program cannot survive a tick).
--  3. ENERGY RING.  Every sample writes {E1, E2} into a 1024-entry EBR ring;
--     the display reads, per output line, the virtual line it is drawing
--     from the latest complete virtual frame (base latched per field).  So
--     ENERGY changes line by line down the screen -- the CONTOUR fringe.
--  4. PIXEL PATH.  VINTAGE: the 4-bit V ramps snapped to the virtual clock
--     grid, comparators, SOLID / HOLE / one-shot RING, the RS colour
--     flip-flop, chroma smear.  CLEAN: continuous |x|+|y| tiles on the
--     visible window, ENERGY interpolated between virtual lines, coverage
--     anti-aliasing, uniform-width rings, [1 2 1] output conditioning.
--
-- Generated avid (parity-locked, cubist/damier), blanking gate, U/V swapped
-- at the output compose only (engine colours are unswapped BT.601).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.mood_pkg.all;

entity mood_core is
    generic (
        G_CLEAN : boolean := false
    );
    port (
        clk          : in  std_logic;
        registers_in : in  t_spi_ram;
        data_in      : in  t_video_stream_yuv444_30b;
        data_out     : out t_video_stream_yuv444_30b
    );
end entity mood_core;

architecture rtl of mood_core is

    function f_sel(c : boolean; a, b : natural) return natural is
    begin
        if c then return a; else return b; end if;
    end function;

    -- pixel pipeline depth (P0 state -> P(n) result) and sync latency
    constant C_PIPE : natural := f_sel(G_CLEAN, 13, 7);
    constant C_LAT  : natural := f_sel(G_CLEAN, 16, 12);
    constant C_RW   : natural := f_sel(G_CLEAN, 32, 16);
    constant C_YO   : natural := C_Y_OBJ - 64;

    ------------------------------------------------------------------------
    -- format class (free-running from the timing ID)
    ------------------------------------------------------------------------
    signal f_cls   : unsigned(2 downto 0) := (others => '0');
    signal f_xstep : unsigned(15 downto 0) := C_XSTEP(0);
    signal f_ystep : unsigned(15 downto 0) := C_YSTEP(0);
    signal f_hstc  : unsigned(15 downto 0) := C_HSTC(0);
    signal f_vstc  : unsigned(16 downto 0) := C_VSTC(0);
    signal f_tick  : unsigned(12 downto 0) := C_TICK(0);
    signal f_hd    : std_logic := '1';

    ------------------------------------------------------------------------
    -- raster: measured width, field anchor, generated avid
    ------------------------------------------------------------------------
    signal s_avq   : std_logic := '0';
    signal s_xcnt  : unsigned(11 downto 0) := (others => '0');
    signal s_W     : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_idle  : unsigned(12 downto 0) := (others => '0');
    signal s_earm, s_anc, s_anc_d, s_anc_d2, s_run : std_logic := '0';

    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_sel, g_par, g_go : std_logic := '0';
    signal g_rsr  : std_logic_vector(1 downto 0) := (others => '0');
    signal g_pc   : unsigned(3 downto 0) := "1000";
    signal g_avid, g_avid_q : std_logic := '0';
    signal g_cnt, g_wm1 : unsigned(11 downto 0) := to_unsigned(719, 12);

    ------------------------------------------------------------------------
    -- engine (DAMIER's design, 1024-word program)
    ------------------------------------------------------------------------
    signal rf_a, rf_b : t_rf := C_RF_INIT;
    signal u_pc, u_fa, u_jr : unsigned(9 downto 0) := (others => '0');
    signal u_st  : unsigned(1 downto 0) := (others => '0');
    signal u_gwt : unsigned(1 downto 0) := (others => '0');
    signal u_ir, u_rom : std_logic_vector(31 downto 0) := (others => '0');
    signal u_wlast, u_we : std_logic := '0';
    signal u_wa  : unsigned(6 downto 0) := (others => '0');
    signal u_rda, u_rdb, u_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal u_sh  : signed(15 downto 0) := (others => '0');
    signal u_shc : unsigned(3 downto 0) := (others => '0');
    signal u_cin, u_rwr, u_blt, u_bnz : std_logic := '0';
    signal u_rsel : unsigned(2 downto 0) := (others => '0');
    signal u_done : std_logic := '1';
    signal u_romr : signed(15 downto 0) := (others => '0');
    signal u_ctlv, u_ctlr : signed(15 downto 0) := (others => '0');
    signal u_prt : std_logic := '0';
    signal u_prp : unsigned(4 downto 0) := (others => '0');
    signal u_prv : std_logic_vector(15 downto 0) := (others => '0');
    signal u_go  : std_logic := '0';
    signal u_tk  : unsigned(12 downto 0) := (others => '0');

    signal mu_pa  : signed(15 downto 0) := (others => '0');
    signal mu_ph  : signed(17 downto 0) := (others => '0');
    signal mu_pl  : unsigned(15 downto 0) := (others => '0');
    signal mu_bp  : std_logic := '0';
    signal mu_cnt : unsigned(2 downto 0) := (others => '0');
    signal mu_bsy, mu_d1, mu_d2 : std_logic := '0';
    signal u_msh : unsigned(4 downto 0) := (others => '0');
    signal u_mra : signed(22 downto 0) := (others => '0');
    signal u_mrd : signed(15 downto 0) := (others => '0');
    signal u_mwt : std_logic := '0';

    signal sn_neg, sn_spec, sn_raw : std_logic := '0';
    signal eaddr : unsigned(9 downto 0) := (others => '0');
    signal erom_q, erom_q2 : std_logic_vector(15 downto 0) := (others => '0');

    signal s_lfsr : std_logic_vector(15 downto 0) := x"ACE1";
    signal s_swv  : std_logic_vector(15 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- engine ports: pending (engine) and active (latched at the anchor)
    ------------------------------------------------------------------------
    signal e1_hold : std_logic_vector(15 downto 0) := (others => '0');
    signal p_mode  : std_logic_vector(5 downto 0) := (others => '0');
    signal p_u1, p_v1, p_u2, p_v2 : signed(9 downto 0) := (others => '0');
    signal p_aas, p_aaw : unsigned(13 downto 0) := to_unsigned(256, 14);
    signal a_mode  : std_logic_vector(5 downto 0) := (others => '0');
    signal a_u1, a_v1, a_u2, a_v2 : signed(9 downto 0) := (others => '0');
    signal a_aas, a_aaw : unsigned(13 downto 0) := to_unsigned(256, 14);

    -- per-frame derived
    signal fr_hn32 : unsigned(7 downto 0) := to_unsigned(32, 8);   -- 32 N
    signal fr_vm32 : unsigned(8 downto 0) := to_unsigned(32, 9);   -- 32 M
    signal fr_hr0  : unsigned(7 downto 0) := to_unsigned(92, 8);
    signal fr_hs0  : unsigned(4 downto 0) := to_unsigned(1, 5);
    signal fr_vr0  : unsigned(8 downto 0) := to_unsigned(206, 9);
    signal fr_vs0  : unsigned(4 downto 0) := to_unsigned(3, 5);
    signal fr_hst  : unsigned(23 downto 0) := (others => '0');      -- CLEAN: N * hstc
    signal fr_vst  : unsigned(23 downto 0) := (others => '0');      -- CLEAN: M * vstc
    signal fr_shp  : unsigned(1 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- ENERGY ring (1024 virtual lines; VINTAGE Q5.3 pairs, CLEAN Q5.8 pairs)
    ------------------------------------------------------------------------
    type t_ring is array (0 to 1023) of std_logic_vector(C_RW - 1 downto 0);
    signal ring  : t_ring;
    signal r_we  : std_logic := '0';
    signal r_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_wp, r_ra, r_base : unsigned(9 downto 0) := (others => '0');
    signal r_rd  : std_logic_vector(C_RW - 1 downto 0) := (others => '0');
    signal r_rd2 : std_logic_vector(31 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- line setup (vertical): runs in hblank before each line
    ------------------------------------------------------------------------
    signal vacc   : unsigned(23 downto 0) := (others => '0');   -- Q8.16 virtual lines
    signal vph    : unsigned(23 downto 0) := (others => '0');   -- CLEAN tile phase
    signal ls_cnt : unsigned(4 downto 0) := (others => '0');
    signal ls_first : std_logic := '0';
    signal vl_cur : unsigned(8 downto 0) := to_unsigned(31, 9);
    signal vr_acc : unsigned(8 downto 0) := (others => '0');
    signal v_stp  : unsigned(4 downto 0) := (others => '0');
    signal ls_vc  : unsigned(15 downto 0) := (others => '0');
    signal ls_elo, ls_ehi : std_logic_vector(31 downto 0) := (others => '0');
    signal ls_f   : unsigned(7 downto 0) := (others => '0');
    signal ls_d1, ls_d2 : signed(14 downto 0) := (others => '0');
    signal ls_m1, ls_m2 : signed(23 downto 0) := (others => '0');
    -- line registers (consumed by the pixel path)
    signal ln_v   : unsigned(15 downto 0) := (others => '0');   -- VINTAGE: V ramp (4 bits); CLEAN: Q5.11
    signal ln_t1, ln_t2 : unsigned(5 downto 0) := (others => '0');
    signal ln_e1, ln_e2 : unsigned(15 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- pixel path results (before the blanking gate) and output
    ------------------------------------------------------------------------
    signal px_v : std_logic := '0';
    signal px_y : unsigned(9 downto 0) := (others => '0');       -- offset above black
    signal px_u, px_w : signed(10 downto 0) := (others => '0');  -- chroma offsets (u, v)
    type t_sd is array (0 to C_LAT - 2) of std_logic_vector(2 downto 0);
    signal s_sd : t_sd := (others => (others => '1'));
    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_out_s : std_logic_vector(3 downto 0) := "0111";

    attribute no_rw_check : boolean;
    attribute no_rw_check of rf_a, rf_b, ring : signal is true;

    function tri4(s : unsigned(4 downto 0)) return unsigned is
    begin
        if s(4) = '0' then return not s(3 downto 0);         -- 15 - s
        else return s(3 downto 0); end if;                    -- s - 16
    end function;

begin

    -- ====================================================================
    -- Format class and per-class constants (free-running)
    -- ====================================================================
    p_fmt : process(clk)
        variable v_c : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 1 | 2 | 10 => v_c := 1;     -- 1080i
                when 5 | 6 | 14 => v_c := 2;     -- 720p
                when 0          => v_c := 3;     -- 480i
                when 4          => v_c := 4;     -- 480p
                when 8          => v_c := 5;     -- 576i
                when 12         => v_c := 6;     -- 576p
                when others     => v_c := 0;     -- 1080p
            end case;
            f_cls   <= to_unsigned(v_c, 3);
            f_xstep <= C_XSTEP(v_c);
            f_ystep <= C_YSTEP(v_c);
            f_hstc  <= C_HSTC(v_c);
            f_vstc  <= C_VSTC(v_c);
            f_tick  <= C_TICK(v_c);
            if v_c <= 2 then f_hd <= '1'; else f_hd <= '0'; end if;
        end if;
    end process p_fmt;

    -- ====================================================================
    -- Raster: width, field anchor (no active video for two line widths:
    -- in vertical blanking, never on a vsync or hsync edge), sample ticks
    -- ====================================================================
    p_meas : process(clk)
    begin
        if rising_edge(clk) then
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avq = '1' then
                if s_xcnt > 64 then s_W <= s_xcnt; end if;
                s_xcnt <= (others => '0');
            end if;

            s_anc <= '0';
            if data_in.avid = '1' then
                s_idle <= (others => '0');
                s_earm <= '1';
            else
                if s_idle /= "1111111111111" then s_idle <= s_idle + 1; end if;
                if s_earm = '1' and s_idle = (s_W & '0') then
                    s_anc  <= '1';
                    s_earm <= '0';
                end if;
            end if;
            s_anc_d  <= s_anc;
            s_anc_d2 <= s_anc_d;
            if s_anc = '1' then s_run <= '1'; end if;

            -- 15.734 kHz sample ticks (real time in every format); the engine
            -- only starts after the first field anchor
            u_go <= '0';
            if s_run = '1' then
                if u_tk = 0 then
                    u_tk <= f_tick;
                    u_go <= '1';
                else
                    u_tk <= u_tk - 1;
                end if;
            end if;

            -- LFSR for the engine's noise (x^16 + x^14 + x^13 + x^11 + 1)
            if s_lfsr(0) = '1' then
                s_lfsr <= ('0' & s_lfsr(15 downto 1)) xor x"B400";
            else
                s_lfsr <= '0' & s_lfsr(15 downto 1);
            end if;

            -- switch word: Gray-coded pairs (off,off)=0 (on,off)=1 (on,on)=2
            -- (off,on)=3; SOURCE; format class
            s_swv <= "00000000" & std_logic_vector(f_cls) & registers_in(6)(4)
                     & registers_in(6)(3) & (registers_in(6)(2) xor registers_in(6)(3))
                     & registers_in(6)(1) & (registers_in(6)(0) xor registers_in(6)(1));
        end if;
    end process p_meas;

    -- ====================================================================
    -- Clean avid: starts 2 or 3 clocks after the source's first avid rise,
    -- whichever keeps the hsync->start parity at its voted usual value; ends
    -- after the field-latched width.  g_go is the cycle BEFORE it rises,
    -- when the pixel accumulators load their line seeds.
    -- ====================================================================
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
            g_rsr <= g_rsr(0) & '0';
            if (g_sel = '0' and g_rsr(0) = '1') or (g_sel = '1' and g_rsr(1) = '1') then
                g_go <= '1';
            else
                g_go <= '0';
            end if;
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

            g_avid_q <= g_avid;
            if g_go = '1' then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;
            if s_anc = '1' then
                g_wm1 <= s_W - 1;
            end if;
        end if;
    end process p_gavid;

    -- ====================================================================
    -- Microcoded engine (DAMIER's design, 1024 words).  Restarts at every
    -- sample tick; three clocks per straight-line instruction; MRD, SHR /
    -- SHL and SIN stall in EXEC.
    -- ====================================================================
    u_fa <= u_pc + 1 when u_st = "11" else u_pc;

    p_urom : process(clk)
    begin
        if rising_edge(clk) then
            u_rom <= C_UCODE(to_integer(u_fa));
        end if;
    end process p_urom;

    p_rfa : process(clk)
    begin
        if rising_edge(clk) then
            if u_we = '1' then rf_a(to_integer(u_wa)) <= u_wd; end if;
            u_rda <= rf_a(to_integer(unsigned(u_rom(19 downto 13))));
        end if;
    end process p_rfa;

    p_rfb : process(clk)
    begin
        if rising_edge(clk) then
            if u_we = '1' then rf_b(to_integer(u_wa)) <= u_wd; end if;
            u_rdb <= rf_b(to_integer(unsigned(u_rom(12 downto 6))));
        end if;
    end process p_rfb;

    p_erom : process(clk)
    begin
        if rising_edge(clk) then
            erom_q  <= C_ROM(to_integer(eaddr));
            erom_q2 <= erom_q;
        end if;
    end process p_erom;

    with to_integer(unsigned(u_ir(3 downto 0))) select u_ctlv <=
        signed(resize(unsigned(registers_in(0)), 16)) when 0,
        signed(resize(unsigned(registers_in(1)), 16)) when 1,
        signed(resize(unsigned(registers_in(2)), 16)) when 2,
        signed(resize(unsigned(registers_in(3)), 16)) when 3,
        signed(resize(unsigned(registers_in(4)), 16)) when 4,
        signed(resize(unsigned(registers_in(5)), 16)) when 5,
        signed(resize(unsigned(registers_in(7)), 16)) when 6,
        signed(s_swv)  when 7,
        signed(s_lfsr) when 8,
        (others => '0') when others;

    p_ueng : process(clk)
        variable v_r : signed(15 downto 0);
        variable v_a, v_b : signed(15 downto 0);
        variable v_op, v_op2 : integer range 0 to 31;
        variable v_stl, v_jmp : boolean;
        variable v_ms, v_md : signed(17 downto 0);
        variable v_pr, v_pr2 : signed(31 downto 0);
        variable v_ma : signed(22 downto 0);
        variable v_fa, v_fb : std_logic_vector(15 downto 0);
        variable v_ang : unsigned(15 downto 0);
        variable v_pm : unsigned(14 downto 0);
        variable v_sv : signed(16 downto 0);
        variable v_bank : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            u_wlast <= u_we;
            -- serial multiply, radix-4 Booth, 8 steps: {mu_ph, mu_pl} =
            -- mu_pa * B for a two's-complement B
            if mu_bsy = '1' then
                case std_logic_vector'(mu_pl(1) & mu_pl(0) & mu_bp) is
                    when "001" | "010" | "101" | "110" => v_md := resize(mu_pa, 18);
                    when "011" | "100"                 => v_md := shift_left(resize(mu_pa, 18), 1);
                    when others                        => v_md := (others => '0');
                end case;
                if mu_pl(1) = '1' then
                    v_ms := mu_ph + (not v_md) + 1;
                else
                    v_ms := mu_ph + v_md;
                end if;
                mu_ph <= shift_right(v_ms, 2);
                mu_pl <= unsigned(v_ms(1 downto 0)) & mu_pl(15 downto 2);
                mu_bp <= mu_pl(1);
                if mu_cnt = 0 then mu_bsy <= '0';
                else mu_cnt <= mu_cnt - 1; end if;
            end if;
            v_pr := signed(std_logic_vector'(std_logic_vector(mu_ph(15 downto 0)) & std_logic_vector(mu_pl)));
            mu_d1 <= mu_bsy;
            mu_d2 <= mu_d1;
            -- MRD shifter, free-running on the current product and shift
            v_pr2 := shift_right(v_pr, 8 * to_integer(u_msh(4 downto 3)));
            u_mra <= v_pr2(22 downto 0);
            v_ma  := shift_right(u_mra, to_integer(u_msh(2 downto 0)));
            u_mrd <= v_ma(15 downto 0);

            u_we  <= '0';
            u_prt <= '0';

            if u_go = '1' then
                u_pc   <= (others => '0');
                u_st   <= "00";
                u_gwt  <= (others => '0');
                u_done <= '0';
            elsif u_done = '0' then
                case u_st is

                    -- FETCH (after a jump only): the ROM answers next cycle
                    when "00" =>
                        u_st <= "01";

                    -- DECODE: latch the instruction; the register-file reads
                    -- are addressed straight off the ROM output this cycle
                    when "01" =>
                        u_ir  <= u_rom;
                        u_msh <= unsigned(u_rom(4 downto 0));
                        u_st <= "10";

                    -- OPERAND LATCH: block-RAM outputs into fabric; every unit
                    -- launches here from the register-file outputs
                    when "10" =>
                        v_op2 := to_integer(unsigned(u_ir(31 downto 27)));
                        -- forwarding: a just-written source comes from the
                        -- write-data register (EBR read-during-write is undefined)
                        if u_wlast = '1' and unsigned(u_ir(19 downto 13)) = u_wa then
                            v_fa := u_wd;
                        else
                            v_fa := u_rda;
                        end if;
                        if u_wlast = '1' and unsigned(u_ir(12 downto 6)) = u_wa then
                            v_fb := u_wd;
                        else
                            v_fb := u_rdb;
                        end if;
                        u_sh <= signed(v_fa);
                        -- SUB = A + not B + 1 on the one adder
                        if v_op2 = 3 then
                            u_wd <= not v_fb;  u_cin <= '1';
                        else
                            u_wd <= v_fb;      u_cin <= '0';
                        end if;
                        u_ctlr <= u_ctlv;
                        case v_op2 is
                            when 1 | 4 | 5 => u_rsel <= "000"; u_rwr <= '1';
                            when 2 | 3     => u_rsel <= "001"; u_rwr <= '1';
                            when 27        => u_rsel <= "010"; u_rwr <= '1';
                            when 15        => u_rsel <= "011"; u_rwr <= '1';
                            when 11        => u_rsel <= "100"; u_rwr <= '1';
                            when 14        => u_rsel <= "101"; u_rwr <= '1';
                            when 25        => u_rsel <= "110"; u_rwr <= '1';
                            when others    => u_rsel <= "000"; u_rwr <= '0';
                        end case;
                        u_shc <= unsigned(u_ir(3 downto 0));
                        u_mwt <= '1';
                        u_wa  <= unsigned(u_ir(26 downto 20));
                        if signed(v_fa) < signed(v_fb) then u_blt <= '1'; else u_blt <= '0'; end if;
                        if v_fa /= x"0000" then u_bnz <= '1'; else u_bnz <= '0'; end if;
                        u_jr <= unsigned(v_fa(9 downto 0));
                        case v_op2 is
                            when 10 =>                     -- MUL
                                mu_pa  <= signed(v_fa);
                                mu_pl  <= unsigned(v_fb);
                                mu_ph  <= (others => '0');
                                mu_bp  <= '0';
                                mu_cnt <= to_unsigned(7, 3);
                                mu_bsy <= '1';
                            when 14 =>                     -- SIN / raw table
                                if u_ir(1) = '1' then
                                    v_bank := shift_left(resize(unsigned(std_logic_vector'(u_ir(5 downto 2) & u_ir(0))), 9), 4);
                                    eaddr  <= '1' & (unsigned(v_fa(8 downto 0)) + v_bank);
                                    sn_raw <= '1';
                                else
                                    v_ang := unsigned(v_fa);
                                    if u_ir(0) = '1' then
                                        v_ang(15 downto 14) := v_ang(15 downto 14) + 1;
                                    end if;
                                    if v_ang(14) = '1' then
                                        v_pm := to_unsigned(16384, 15) - resize(v_ang(13 downto 0), 15);
                                    else
                                        v_pm := resize(v_ang(13 downto 0), 15);
                                    end if;
                                    sn_spec <= v_pm(14);
                                    sn_neg  <= v_ang(15);
                                    sn_raw  <= '0';
                                    eaddr   <= '0' & v_pm(13 downto 5);
                                end if;
                            when 17 =>                     -- PRT operands
                                u_prp <= unsigned(u_ir(4 downto 0));
                                u_prv <= v_fa;
                            when others => null;
                        end case;
                        u_st <= "11";

                    -- EXECUTE
                    when others =>
                        v_op  := to_integer(unsigned(u_ir(31 downto 27)));
                        v_a   := u_sh;
                        v_b   := signed(u_wd);
                        v_stl := false;
                        v_jmp := false;

                        case to_integer(u_rsel) is
                            when 0 => v_r := u_sh;                          -- MOV, SHR/SHL
                            when 1 => if u_cin = '1' then v_r := v_a + v_b + 1;
                                      else                v_r := v_a + v_b; end if;
                            when 2 => v_r := v_a and signed(resize(unsigned(u_ir(12 downto 0)), 16));
                            when 3 => v_r := resize(signed(u_ir(13 downto 0)), 16);
                            when 4 => v_r := u_mrd;                         -- MRD
                            when 5 => v_r := u_romr;                        -- SIN / ROM
                            when others => v_r := u_ctlr;                   -- CTL
                        end case;

                        case v_op is
                            when 4 | 5 =>                  -- SHR / SHL
                                if u_shc /= 0 then
                                    if v_op = 4 then u_sh <= shift_right(u_sh, 1);
                                    else             u_sh <= shift_left(u_sh, 1); end if;
                                    u_shc <= u_shc - 1;
                                    v_stl := true;
                                end if;
                            when 11 =>                     -- MRD: product final, shifted
                                if mu_bsy = '1' or mu_d1 = '1' or mu_d2 = '1' or u_mwt = '1' then
                                    u_mwt <= '0';
                                    v_stl := true;
                                end if;
                            when 14 =>                     -- SIN: the ROM answers in 2,
                                case to_integer(u_gwt) is  -- into fabric in 3
                                    when 0 | 1 =>
                                        u_gwt <= u_gwt + 1;
                                        v_stl := true;
                                    when 2 =>
                                        if sn_raw = '1' then
                                            u_romr <= signed(erom_q2);
                                        else
                                            if sn_spec = '1' then
                                                v_sv := to_signed(32767, 17);
                                            else
                                                v_sv := signed('0' & erom_q2);
                                            end if;
                                            if sn_neg = '1' then u_romr <= -v_sv(15 downto 0);
                                            else                 u_romr <= v_sv(15 downto 0); end if;
                                        end if;
                                        u_gwt <= "11";
                                        v_stl := true;
                                    when others =>
                                        u_gwt <= (others => '0');
                                end case;
                            when 17 => u_prt <= '1';
                            when 18 => v_jmp := true;
                                       u_pc <= unsigned(u_ir(9 downto 0));
                            when 19 => if u_bnz = '1' then
                                           v_jmp := true;
                                           u_pc <= unsigned(u_ir(9 downto 0));
                                       end if;
                            when 20 => if u_blt = '1' then
                                           v_jmp := true;
                                           u_pc <= unsigned(u_ir(23 downto 20)) & unsigned(u_ir(5 downto 0));
                                       end if;
                            when 21 => if u_blt = '0' then
                                           v_jmp := true;
                                           u_pc <= unsigned(u_ir(23 downto 20)) & unsigned(u_ir(5 downto 0));
                                       end if;
                            when 26 => v_jmp := true;
                                       u_pc <= u_jr;
                            when 24 => u_done <= '1';
                            when others => null;
                        end case;

                        if v_stl then
                            u_st <= "11";
                        elsif v_jmp then
                            u_st <= "00";
                        else
                            u_st <= "01";
                            u_pc <= u_pc + 1;
                        end if;

                        u_wd <= std_logic_vector(v_r);
                        if u_rwr = '1' and not v_stl then
                            u_we <= '1';
                        end if;
                end case;
            end if;
        end if;
    end process p_ueng;

    -- ====================================================================
    -- Engine ports.  E2 writes {E1, E2} into the ENERGY ring; display
    -- values are pending until the next field anchor.
    -- ====================================================================
    p_ports : process(clk)
    begin
        if rising_edge(clk) then
            r_we <= '0';
            if u_prt = '1' then
                case to_integer(u_prp) is
                    when 0 => e1_hold <= u_prv;
                    when 1 =>
                        r_we <= '1';
                        if G_CLEAN then
                            r_wd <= "000" & e1_hold(12 downto 0) & "000" & u_prv(12 downto 0);
                        else
                            r_wd <= x"0000" & e1_hold(12 downto 5) & u_prv(12 downto 5);
                        end if;
                    when 2 => p_mode <= u_prv(5 downto 0);
                    when 3 => p_u1 <= signed(u_prv(9 downto 0));
                    when 4 => p_v1 <= signed(u_prv(9 downto 0));
                    when 5 => p_u2 <= signed(u_prv(9 downto 0));
                    when 6 => p_v2 <= signed(u_prv(9 downto 0));
                    when 7 => p_aas <= unsigned(u_prv(13 downto 0));
                    when 8 => p_aaw <= unsigned(u_prv(13 downto 0));
                    when others => null;
                end case;
            end if;
            if r_we = '1' then r_wp <= r_wp + 1; end if;

            -- per-field latch: mode, colours, AA constants, ring base, width
            if s_anc = '1' then
                a_mode <= p_mode;
                a_u1 <= p_u1; a_v1 <= p_v1; a_u2 <= p_u2; a_v2 <= p_v2;
                a_aas <= p_aas; a_aaw <= p_aaw;
                r_base <= r_wp - 262;
            end if;
            -- per-field derived constants
            if s_anc_d = '1' then
                fr_shp <= unsigned(a_mode(5 downto 4));
                case a_mode(1 downto 0) is
                    when "00"   => fr_hn32 <= to_unsigned(32, 8);  fr_hr0 <= to_unsigned(92, 8);  fr_hs0 <= to_unsigned(1, 5);
                                   fr_hst <= resize(f_hstc, 24);
                    when "01"   => fr_hn32 <= to_unsigned(64, 8);  fr_hr0 <= to_unsigned(184, 8); fr_hs0 <= to_unsigned(2, 5);
                                   fr_hst <= shift_left(resize(f_hstc, 24), 1);
                    when "10"   => fr_hn32 <= to_unsigned(96, 8);  fr_hr0 <= to_unsigned(48, 8);  fr_hs0 <= to_unsigned(4, 5);
                                   fr_hst <= shift_left(resize(f_hstc, 24), 1) + resize(f_hstc, 24);
                    when others => fr_hn32 <= to_unsigned(160, 8); fr_hr0 <= to_unsigned(4, 8);   fr_hs0 <= to_unsigned(7, 5);
                                   fr_hst <= shift_left(resize(f_hstc, 24), 2) + resize(f_hstc, 24);
                end case;
                case a_mode(3 downto 2) is
                    when "00"   => fr_vm32 <= to_unsigned(32, 9);  fr_vr0 <= to_unsigned(206, 9); fr_vs0 <= to_unsigned(3, 5);
                                   fr_vst <= resize(f_vstc, 24);
                    when "01"   => fr_vm32 <= to_unsigned(64, 9);  fr_vr0 <= to_unsigned(150, 9); fr_vs0 <= to_unsigned(7, 5);
                                   fr_vst <= shift_left(resize(f_vstc, 24), 1);
                    when "10"   => fr_vm32 <= to_unsigned(128, 9); fr_vr0 <= to_unsigned(38, 9);  fr_vs0 <= to_unsigned(15, 5);
                                   fr_vst <= shift_left(resize(f_vstc, 24), 2);
                    when others => fr_vm32 <= to_unsigned(256, 9); fr_vr0 <= to_unsigned(76, 9);  fr_vs0 <= to_unsigned(30, 5);
                                   fr_vst <= shift_left(resize(f_vstc, 24), 3);
                end case;
            end if;
        end if;
    end process p_ports;

    -- the ring: one write port (engine), one registered read port (display)
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            if r_we = '1' then
                ring(to_integer(r_wp)) <= r_wd(C_RW - 1 downto 0);
            end if;
            r_rd <= ring(to_integer(r_ra));
        end if;
    end process p_ring;

    -- ====================================================================
    -- Line setup.  Started at the field anchor (first line) and at every
    -- end of active video; finishes ~30 clocks later, inside hblank, and
    -- only then commits the next line's registers.
    -- ====================================================================
    p_line : process(clk)
        variable v_vl : unsigned(8 downto 0);
        variable v_s  : unsigned(9 downto 0);
        variable v_h  : unsigned(15 downto 0);
        variable v_e  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            r_rd2 <= std_logic_vector(resize(unsigned(r_rd), 32));
            if s_anc_d2 = '1' then
                if G_CLEAN then
                    vacc <= resize(f_ystep(15 downto 1), 24);      -- line centres
                    vph  <= shift_right(fr_vst, 1);
                else
                    vacc <= (others => '0');
                end if;
                ls_cnt   <= to_unsigned(1, 5);
                ls_first <= '1';
            elsif g_avid_q = '1' and g_avid = '0' then
                vacc     <= vacc + f_ystep;
                vph      <= vph + fr_vst;
                ls_cnt   <= to_unsigned(1, 5);
                ls_first <= '0';
            elsif ls_cnt /= 0 then
                ls_cnt <= ls_cnt + 1;
            end if;

            case to_integer(ls_cnt) is
                when 8 =>
                    v_vl := resize(vacc(23 downto 16), 9) + 31;
                    if v_vl > 261 then v_vl := to_unsigned(261, 9); end if;
                    r_ra <= r_base + resize(v_vl, 10);
                    ls_f <= vacc(15 downto 8);
                    -- VINTAGE V ramp: floor(vl * 32M / 262) as a DDA
                    if ls_first = '1' then
                        vl_cur <= to_unsigned(31, 9);
                        vr_acc <= fr_vr0;
                        v_stp  <= fr_vs0;
                    elsif v_vl /= vl_cur then
                        vl_cur <= v_vl;
                        v_s := resize(vr_acc, 10) + resize(fr_vm32, 10);
                        if v_s >= 262 then
                            vr_acc <= resize(v_s - 262, 9);
                            v_stp  <= v_stp + 1;
                        else
                            vr_acc <= resize(v_s, 9);
                        end if;
                    end if;
                    -- CLEAN V ramp: |phase - 1/2| in Q5.11 reference units
                    v_h := vph(23 downto 8);
                    if v_h(15) = '1' then ls_vc <= v_h - 32768;
                    else                  ls_vc <= 32768 - v_h; end if;
                when 11 =>
                    ls_elo <= r_rd2;
                    r_ra   <= r_ra + 1;
                when 14 =>
                    ls_ehi <= r_rd2;
                    if G_CLEAN then
                        ls_d1 <= signed(resize(unsigned(r_rd2(28 downto 16)), 15)) - signed(resize(unsigned(ls_elo(28 downto 16)), 15));
                        ls_d2 <= signed(resize(unsigned(r_rd2(12 downto 0)), 15)) - signed(resize(unsigned(ls_elo(12 downto 0)), 15));
                    end if;
                    ls_m1 <= (others => '0');
                    ls_m2 <= (others => '0');
                when 15 | 16 | 17 | 18 | 19 | 20 | 21 | 22 =>
                    -- serial (E_hi - E_lo) * f, MSB first
                    if ls_f(7) = '1' then
                        ls_m1 <= shift_left(ls_m1, 1) + resize(ls_d1, 24);
                        ls_m2 <= shift_left(ls_m2, 1) + resize(ls_d2, 24);
                    else
                        ls_m1 <= shift_left(ls_m1, 1);
                        ls_m2 <= shift_left(ls_m2, 1);
                    end if;
                    ls_f <= ls_f(6 downto 0) & '0';
                when 28 =>
                    if G_CLEAN then
                        ln_v <= ls_vc;
                        v_e  := shift_left(resize(unsigned(ls_elo(28 downto 16)), 16), 3);
                        ln_e1 <= unsigned(signed(v_e) + resize(shift_right(ls_m1, 5), 16));
                        v_e  := shift_left(resize(unsigned(ls_elo(12 downto 0)), 16), 3);
                        ln_e2 <= unsigned(signed(v_e) + resize(shift_right(ls_m2, 5), 16));
                    else
                        ln_v  <= resize(tri4(v_stp), 16);
                        -- ref < T  <=>  ref < ceil(E): T = (E + 7) >> 3 (Q5.3)
                        ln_t1 <= resize(shift_right(resize(unsigned(ls_elo(15 downto 8)), 9) + 7, 3), 6);
                        ln_t2 <= resize(shift_right(resize(unsigned(ls_elo(7 downto 0)), 9) + 7, 3), 6);
                    end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    -- ====================================================================
    -- VINTAGE pixel path
    -- ====================================================================
    gen_vintage : if not G_CLEAN generate
        signal h_fr : unsigned(15 downto 0) := (others => '0');
        signal h_ra : unsigned(7 downto 0) := (others => '0');
        signal h_st : unsigned(4 downto 0) := (others => '0');
        signal h_tk : std_logic := '0';
        signal q1_v, q1_tk, q1_f : std_logic := '0';
        signal q1_hv : unsigned(3 downto 0) := (others => '0');
        signal q2_v, q2_tk, q2_f : std_logic := '0';
        signal q2_ref : unsigned(4 downto 0) := (others => '0');
        signal q3_v, q3_tk, q3_f, q3_on1, q3_on2 : std_logic := '0';
        signal q4_v, q4_lum, q4_sel : std_logic := '0';
        signal ff_s, ff_pb, rg_px : std_logic := '0';
        signal rg_c1, rg_c2 : unsigned(2 downto 0) := (others => '0');
        signal q5_v : std_logic := '0';
        signal q5_y : unsigned(9 downto 0) := (others => '0');
        signal q5_u, q5_w : signed(10 downto 0) := (others => '0');
        signal i_y : signed(11 downto 0) := (others => '0');
        signal i_u, i_w : signed(11 downto 0) := (others => '0');
        signal q6_v : std_logic := '0';
    begin
        p_vin : process(clk)
            variable v_fs : unsigned(16 downto 0);
            variable v_s  : unsigned(8 downto 0);
            variable v_a, v_b, v_s_ff, v_pb, v_px, v_xo, v_lum : std_logic;
            variable v_c1, v_c2 : unsigned(2 downto 0);
            variable v_ky, v_kc : natural range 0 to 7;
        begin
            if rising_edge(clk) then
                -- P0: virtual clock DDA (ticks) and the H ramp step DDA
                if g_go = '1' then
                    h_fr <= (others => '0');
                    h_ra <= fr_hr0;
                    h_st <= fr_hs0;
                    h_tk <= '0';
                elsif g_avid = '1' then
                    v_fs := resize(h_fr, 17) + resize(f_xstep, 17);
                    h_fr <= v_fs(15 downto 0);
                    h_tk <= v_fs(16);
                    if v_fs(16) = '1' then
                        v_s := resize(h_ra, 9) + resize(fr_hn32, 9);
                        if v_s >= 228 then
                            h_ra <= resize(v_s - 228, 8);
                            h_st <= h_st + 1;
                        else
                            h_ra <= v_s(7 downto 0);
                        end if;
                    end if;
                end if;
                -- P1: H ramp value
                q1_v  <= g_avid;
                q1_f  <= g_avid and not g_avid_q;
                q1_tk <= h_tk;
                q1_hv <= tri4(h_st);
                -- P2: reference = H + V (0..30)
                q2_v <= q1_v; q2_f <= q1_f; q2_tk <= q1_tk;
                q2_ref <= resize(q1_hv, 5) + resize(ln_v(3 downto 0), 5);
                -- P3: comparators
                q3_v <= q2_v; q3_f <= q2_f; q3_tk <= q2_tk;
                if resize(q2_ref, 6) < ln_t1 then q3_on1 <= '1'; else q3_on1 <= '0'; end if;
                if resize(q2_ref, 6) < ln_t2 then q3_on2 <= '1'; else q3_on2 <= '0'; end if;
                -- P4: object logic + colour-select flip-flop
                q4_v <= q3_v;
                if q3_v = '1' then
                    v_a := q3_on1; v_b := q3_on2;
                    v_xo := v_a xor v_b;
                    if q3_f = '1' then
                        v_s_ff := '0'; v_pb := '0'; v_px := v_xo;
                        v_c1 := (others => '0'); v_c2 := (others => '0');
                    else
                        v_s_ff := ff_s; v_pb := ff_pb; v_px := rg_px;
                        v_c1 := rg_c1; v_c2 := rg_c2;
                    end if;
                    -- RS flip-flop: set by ch1 alone, reset by ch2 alone,
                    -- toggled when both become on (last to turn on wins)
                    if v_a = '1' and v_b = '0' then v_s_ff := '0';
                    elsif v_a = '0' and v_b = '1' then v_s_ff := '1';
                    elsif v_a = '1' and v_b = '1' and v_pb = '0' then v_s_ff := not v_s_ff;
                    end if;
                    ff_s  <= v_s_ff;
                    ff_pb <= v_a and v_b;
                    -- RING: two 7-clock one-shots on the XOR's rising and
                    -- falling edges, counted in virtual clocks
                    if q3_tk = '1' then
                        if v_c1 /= 0 then v_c1 := v_c1 - 1; end if;
                        if v_c2 /= 0 then v_c2 := v_c2 - 1; end if;
                    end if;
                    if v_xo = '1' and v_px = '0' then v_c1 := to_unsigned(7, 3); end if;
                    if v_xo = '0' and v_px = '1' then v_c2 := to_unsigned(7, 3); end if;
                    rg_px <= v_xo;
                    rg_c1 <= v_c1;
                    rg_c2 <= v_c2;
                    case fr_shp is
                        when "00"   => v_lum := v_a or v_b;
                        when "01"   => v_lum := v_xo;
                        when others =>
                            if v_c1 /= 0 or v_c2 /= 0 then v_lum := '1'; else v_lum := '0'; end if;
                    end case;
                    q4_lum <= v_lum;
                    q4_sel <= v_s_ff;
                else
                    q4_lum <= '0';
                end if;
                -- P5: one luma level; the selected channel's colour, gated
                q5_v <= q4_v;
                if q4_lum = '1' then
                    q5_y <= to_unsigned(C_YO, 10);
                    if q4_sel = '1' then
                        q5_u <= resize(a_u2, 11); q5_w <= resize(a_v2, 11);
                    else
                        q5_u <= resize(a_u1, 11); q5_w <= resize(a_v1, 11);
                    end if;
                else
                    q5_y <= (others => '0');
                    q5_u <= (others => '0');
                    q5_w <= (others => '0');
                end if;
                -- P6: RF-path softening: luma ~0.5 virtual clock, chroma
                -- smear ~2 virtual clocks (one-pole, trailing right)
                if f_hd = '1' then v_ky := 2; v_kc := 4; else v_ky := 1; v_kc := 3; end if;
                q6_v <= q5_v;
                if q5_v = '0' then
                    i_y <= (others => '0'); i_u <= (others => '0'); i_w <= (others => '0');
                else
                    i_y <= i_y + shift_right(signed(resize(q5_y, 12)) - i_y, v_ky);
                    i_u <= i_u + shift_right(resize(q5_u, 12) - i_u, v_kc);
                    i_w <= i_w + shift_right(resize(q5_w, 12) - i_w, v_kc);
                end if;
            end if;
        end process p_vin;
        px_v <= q6_v;
        px_y <= unsigned(i_y(9 downto 0));
        px_u <= i_u(10 downto 0);
        px_w <= i_w(10 downto 0);
    end generate gen_vintage;

    -- ====================================================================
    -- CLEAN pixel path
    -- ====================================================================
    gen_clean : if G_CLEAN generate
        signal h_ph : unsigned(23 downto 0) := (others => '0');
        signal fr_ka, fr_kb : signed(14 downto 0) := to_signed(32, 15);
        signal q1_v, q2_v, q3_v, q4_v, q5_v, q6_v, q7_v, q8_v, q9_v, q10_v, q11_v : std_logic := '0';
        signal q1_f, q2_f, q3_f, q4_f, q5_f, q6_f : std_logic := '0';
        signal q1_h : unsigned(15 downto 0) := (others => '0');
        signal q2_ref : unsigned(16 downto 0) := (others => '0');
        signal q3_d1, q3_d2 : signed(12 downto 0) := (others => '0');
        signal q4_a1, q4_b1, q4_a2, q4_b2 : signed(20 downto 0) := (others => '0');
        signal q5_p1, q5_p2 : signed(13 downto 0) := (others => '0');
        signal q6_pmax, q6_pmin : signed(13 downto 0) := (others => '0');
        signal q6_c1, q6_c2 : unsigned(6 downto 0) := (others => '0');
        signal q6_on1, q6_on2 : std_logic := '0';
        signal ff_s, ff_pb : std_logic := '0';
        signal q7_ca, q7_cb, q7_hi, q7_hole : unsigned(6 downto 0) := (others => '0');
        signal q7_sel : std_logic := '0';
        signal q8_m : unsigned(6 downto 0) := (others => '0');
        signal q8_su, q8_sv : signed(9 downto 0) := (others => '0');
        signal q9_y : unsigned(15 downto 0) := (others => '0');
        signal q9_uh, q9_vh : signed(14 downto 0) := (others => '0');
        signal q9_ul, q9_vl : signed(13 downto 0) := (others => '0');
        signal q10_y : unsigned(9 downto 0) := (others => '0');
        signal q10_u, q10_w : signed(10 downto 0) := (others => '0');
        signal w_v0 : std_logic := '0';
        signal w_y0, w_y1 : unsigned(9 downto 0) := (others => '0');
        signal w_u0, w_u1, w_w0, w_w1 : signed(10 downto 0) := (others => '0');
        signal q11_y : unsigned(9 downto 0) := (others => '0');
        signal q11_u, q11_w : signed(10 downto 0) := (others => '0');

        -- coverage: clamp(p + k, 0, 64), p in pixels Q.6
        function covk(p : signed(13 downto 0); k : signed(14 downto 0)) return unsigned is
            variable v : signed(14 downto 0);
        begin
            v := resize(p, 15) + k;
            if v < 0 then return to_unsigned(0, 7);
            elsif v > 64 then return to_unsigned(64, 7);
            else return unsigned(v(6 downto 0)); end if;
        end function;

        function clampd(d : signed(17 downto 0)) return signed is
        begin
            if d > 4095 then return to_signed(4095, 13);
            elsif d < -4096 then return to_signed(-4096, 13);
            else return d(12 downto 0); end if;
        end function;
    begin
        -- per-field ring-core offsets: coverage at p - W and p + W
        p_cfr : process(clk)
        begin
            if rising_edge(clk) then
                if s_anc_d = '1' then
                    fr_ka <= 32 - signed(resize(a_aaw(13 downto 2), 15));
                    fr_kb <= 32 + signed(resize(a_aaw(13 downto 2), 15));
                end if;
            end if;
        end process p_cfr;

        p_cln : process(clk)
            variable v_h  : unsigned(15 downto 0);
            variable v_s1, v_s2 : signed(21 downto 0);
            variable v_a, v_b, v_s_ff, v_pb : std_logic;
            variable v_core, v_m : signed(7 downto 0);
        begin
            if rising_edge(clk) then
                -- P0: tile phase (Q24), N tiles across the visible window
                if g_go = '1' then
                    h_ph <= shift_right(fr_hst, 1);
                elsif g_avid = '1' then
                    h_ph <= h_ph + fr_hst;
                end if;
                -- P1: H = |phase - 1/2| in Q5.11 reference units (0..16)
                q1_v <= g_avid;
                q1_f <= g_avid and not g_avid_q;
                v_h := h_ph(23 downto 8);
                if v_h(15) = '1' then q1_h <= v_h - 32768;
                else                  q1_h <= 32768 - v_h; end if;
                -- P2: reference = H + V (0..32)
                q2_v <= q1_v; q2_f <= q1_f;
                q2_ref <= resize(q1_h, 17) + resize(ln_v, 17);
                -- P3: distance to each channel's threshold, +-2 units
                q3_v <= q2_v; q3_f <= q2_f;
                q3_d1 <= clampd(signed(resize(ln_e1, 18)) - signed(resize(q2_ref, 18)));
                q3_d2 <= clampd(signed(resize(ln_e2, 18)) - signed(resize(q2_ref, 18)));
                -- P4: pixels = distance x pixels-per-unit, split multiply
                q4_v <= q3_v; q4_f <= q3_f;
                q4_a1 <= q3_d1 * signed('0' & a_aas(13 downto 7));
                q4_b1 <= q3_d1 * signed('0' & a_aas(6 downto 0));
                q4_a2 <= q3_d2 * signed('0' & a_aas(13 downto 7));
                q4_b2 <= q3_d2 * signed('0' & a_aas(6 downto 0));
                -- P5: Q2.11 x Q8.8 >> 13 = pixels, Q.6
                q5_v <= q4_v; q5_f <= q4_f;
                v_s1 := shift_left(resize(q4_a1, 22), 7) + resize(q4_b1, 22);
                v_s2 := shift_left(resize(q4_a2, 22), 7) + resize(q4_b2, 22);
                q5_p1 <= resize(shift_right(v_s1, 13), 14);
                q5_p2 <= resize(shift_right(v_s2, 13), 14);
                -- P6: edge coverages, larger / smaller distance, inside flags
                q6_v <= q5_v; q6_f <= q5_f;
                q6_c1 <= covk(q5_p1, to_signed(32, 15));
                q6_c2 <= covk(q5_p2, to_signed(32, 15));
                if q5_p1 > q5_p2 then q6_pmax <= q5_p1; q6_pmin <= q5_p2;
                else                  q6_pmax <= q5_p2; q6_pmin <= q5_p1; end if;
                if q5_p1 > 0 then q6_on1 <= '1'; else q6_on1 <= '0'; end if;
                if q5_p2 > 0 then q6_on2 <= '1'; else q6_on2 <= '0'; end if;
                -- P7: ring-core coverages, union / band, colour flip-flop
                q7_v <= q6_v;
                q7_ca <= covk(q6_pmax, fr_ka);
                q7_cb <= covk(q6_pmin, fr_kb);
                if q6_c1 > q6_c2 then
                    q7_hi <= q6_c1; q7_hole <= q6_c1 - q6_c2;
                else
                    q7_hi <= q6_c2; q7_hole <= q6_c2 - q6_c1;
                end if;
                v_a := q6_on1; v_b := q6_on2;
                if q6_f = '1' then v_s_ff := '0'; v_pb := '0';
                else               v_s_ff := ff_s; v_pb := ff_pb; end if;
                -- RS flip-flop: set by ch1 alone, reset by ch2 alone,
                -- toggled when both become on (last to turn on wins)
                if v_a = '1' and v_b = '0' then v_s_ff := '0';
                elsif v_a = '0' and v_b = '1' then v_s_ff := '1';
                elsif v_a = '1' and v_b = '1' and v_pb = '0' then v_s_ff := not v_s_ff;
                end if;
                ff_s  <= v_s_ff;
                ff_pb <= v_a and v_b;
                q7_sel <= v_s_ff;
                -- P8: shape coverage; the selected channel's colour
                q8_v <= q7_v;
                v_core := signed(resize(q7_ca, 8)) - signed(resize(q7_cb, 8));
                if v_core < 0 then v_core := (others => '0'); end if;
                v_m := signed(resize(q7_hole, 8)) - v_core;
                if v_m < 0 then v_m := (others => '0'); end if;
                if q7_v = '0' then
                    q8_m <= (others => '0');
                else
                    case fr_shp is
                        when "00"   => q8_m <= q7_hi;                        -- SOLID: union
                        when "01"   => q8_m <= q7_hole;                      -- HOLE: band
                        when others => q8_m <= unsigned(v_m(6 downto 0));    -- RING: band edges
                    end case;
                end if;
                if q7_sel = '1' then q8_su <= a_u2; q8_sv <= a_v2;
                else                 q8_su <= a_u1; q8_sv <= a_v1; end if;
                -- P9: luma and chroma x coverage (chroma split by m's bits)
                q9_v <= q8_v;
                q9_y  <= resize(q8_m * to_unsigned(C_YO, 9), 16);
                q9_uh <= q8_su * signed('0' & q8_m(6 downto 3));
                q9_ul <= q8_su * signed('0' & q8_m(2 downto 0));
                q9_vh <= q8_sv * signed('0' & q8_m(6 downto 3));
                q9_vl <= q8_sv * signed('0' & q8_m(2 downto 0));
                -- P10: / 64
                q10_v <= q9_v;
                q10_y <= resize(shift_right(q9_y, 6), 10);
                q10_u <= resize(shift_right(shift_left(resize(q9_uh, 18), 3) + resize(q9_ul, 18), 6), 11);
                q10_w <= resize(shift_right(shift_left(resize(q9_vh, 18), 3) + resize(q9_vl, 18), 6), 11);
                -- P11: [1 2 1] on Y, U and V (HD HDMI bar conditioning);
                -- the window's centre is one pixel behind q10
                w_v0 <= q10_v;
                q11_v <= w_v0;
                w_y0 <= q10_y; w_y1 <= w_y0;
                w_u0 <= q10_u; w_u1 <= w_u0;
                w_w0 <= q10_w; w_w1 <= w_w0;
                q11_y <= resize(shift_right(resize(q10_y, 12) + shift_left(resize(w_y0, 12), 1) + resize(w_y1, 12), 2), 10);
                q11_u <= resize(shift_right(resize(q10_u, 13) + shift_left(resize(w_u0, 13), 1) + resize(w_u1, 13), 2), 11);
                q11_w <= resize(shift_right(resize(q10_w, 13) + shift_left(resize(w_w0, 13), 1) + resize(w_w1, 13), 2), 11);
            end if;
        end process p_cln;
        px_v <= q11_v;
        px_y <= q11_y;
        px_u <= q11_u;
        px_w <= q11_w;
    end generate gen_clean;

    -- ====================================================================
    -- Output: sync delay, blanking gate, U/V swapped for this hardware
    -- ====================================================================
    p_out : process(clk)
        variable v_y : unsigned(10 downto 0);
        variable v_cu, v_cv : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s_sd(0) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            for i in 1 to C_LAT - 2 loop
                s_sd(i) <= s_sd(i - 1);
            end loop;
            if px_v = '1' then
                v_y := resize(px_y, 11) + 64;
                if v_y > 940 then v_y := to_unsigned(940, 11); end if;
                s_out_y <= v_y(9 downto 0);
                v_cu := resize(px_w, 12) + 512;                -- hardware U <- V
                v_cv := resize(px_u, 12) + 512;                -- hardware V <- U
                s_out_u <= unsigned(v_cu(9 downto 0));
                s_out_v <= unsigned(v_cv(9 downto 0));
            else
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= to_unsigned(512, 10);
                s_out_v <= to_unsigned(512, 10);
            end if;
            s_out_s <= px_v & s_sd(C_LAT - 2);
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= s_out_s(3);
    data_out.hsync_n <= s_out_s(2);
    data_out.vsync_n <= s_out_s(1);
    data_out.field_n <= s_out_s(0);

end architecture rtl;
