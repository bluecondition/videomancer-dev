-- wake.vhd - WAKE: motion-keyed effects
-- Copyright (C) 2026 bluecondition
-- License: GNU General Public License v3.0
--------------------------------------------------------------------------------
-- Program Name:
--   WAKE
-- Overview:
--   Effects land where the picture MOVES, not where it is bright. The
--   motion_key engine (motion_key.vhd) keeps a 32x18 (SD 24x18) grid of cell
--   averages, compares it with the previous update, and holds the per-cell
--   difference with a WAKE release. It renders the grid back to a smooth
--   matte and shapes it into a key. This file is one host of that engine:
--   it picks an effect, mixes dry/wet by the key, and gates the output.
--
-- Pipeline (cycle = clocks after data_in):
--   1   s1: register input; U/V swapped to BT.601 order (the hardware swaps U/V)
--   1   motion_key analysis tap + render sync/luma (L = 7) -> key/matte at 8
--   2-8 video delay (vp); 7 -> 8 wet signal of the selected effect + Heat dim
--   8   per-effect a/b/t select -> mixer stage A
--   8-11 3-stage mixer a + ((b-a)*t7 + 32) >> 6 (t7 = (t+8) >> 4) -> 11
--        (replaces the 3 SDK interpolators: ~700 LUT, needed to fit v3 + Phase 3)
--   11  blanking gate on the generated avid, U/V swapped back -> output 12
--   C_PROCESSING_DELAY_CLKS = 12; hsync/vsync/field delayed 12; avid is
--   regenerated (aurora/chantilly pattern, hsync parity locked) because
--   Heat and Matte draw their own colour.
--
-- Register map:
--   K1  registers_in(0)    WAKE      release time (1 update .. ~1 s tau)
--   K2  registers_in(1)    FLOOR     noise coring
--   K3  registers_in(2)    CHROMA    chroma weight (centre = equal)
--   K4  registers_in(3)    SOFTNESS  <12% blocky nearest cell, then softer
--   K5  registers_in(4)    DETAIL    edge gate: effect clings to moving contours
--   K6  registers_in(5)    EFFECT    Matte, Heat, Mono, Quarter Turn, Negative,
--                                     Solarize, Crush, Smear, Cloak (-> Matte, Phase 4)
--   S7  registers_in(6)(0) MODE      (Phase 4)
--   S8  registers_in(6)(1) PLATE     (Phase 4)
--   S9  registers_in(6)(2) STEADY    pan/fade compensation + scene-cut guard
--   S10 registers_in(6)(3) TRACK     (Phase 4)
--   S11 registers_in(6)(4) KEY       Normal / Invert
--   P12 registers_in(7)    STIR      sensitivity; bottom ~8% = nothing keyed
--   registers_in(8)(3:0)   timing ID (grid columns, update rate, interlace)
--
-- Views:
--   Matte  key as grey: interp(black, white, key)
--   Heat   interp(dimmed mono, HEAT[matte>>4], min(1023, raw_key<<2));
--          palette by motion strength, ignores KEY Invert
--   others interp(dry, wet, key)
--
-- Resources: engine 9 EBR; see PROGRESS.md for measured LC/Fmax.
--------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;
use work.motion_key_pkg.all;

architecture wake of program_top is

    constant C_PROCESSING_DELAY_CLKS : integer := 12;
    constant C_LATENCY     : integer := C_PROCESSING_DELAY_CLKS;
    constant C_TEST_OBJECT : boolean := false;   -- true: moving square in the analysis

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Heat palette: black, blue, magenta, red, yellow, white (BT.601 10-bit)
    type t_pal is array (0 to 63) of std_logic_vector(29 downto 0);
    function f_yuv(y, u, v : integer) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(y, 10) & to_unsigned(u, 10) & to_unsigned(v, 10));
    end function;
    constant C_HEAT : t_pal := (
        f_yuv(64,512,512),  f_yuv(70,540,507),  f_yuv(76,568,503),  f_yuv(83,596,498),
        f_yuv(89,624,494),  f_yuv(95,652,489),  f_yuv(101,680,485), f_yuv(108,708,480),
        f_yuv(114,736,476), f_yuv(120,764,471), f_yuv(126,792,466), f_yuv(133,820,462),
        f_yuv(139,848,457), f_yuv(145,876,453), f_yuv(151,904,448), f_yuv(158,932,444),
        f_yuv(164,960,439), f_yuv(186,947,476), f_yuv(208,935,514), f_yuv(229,922,551),
        f_yuv(251,910,588), f_yuv(273,897,626), f_yuv(295,884,663), f_yuv(317,872,700),
        f_yuv(338,859,738), f_yuv(360,847,775), f_yuv(382,834,812), f_yuv(404,821,850),
        f_yuv(426,809,887), f_yuv(417,771,893), f_yuv(409,734,899), f_yuv(401,697,905),
        f_yuv(392,659,911), f_yuv(384,622,918), f_yuv(376,585,924), f_yuv(368,547,930),
        f_yuv(359,510,936), f_yuv(351,473,942), f_yuv(343,435,948), f_yuv(334,398,954),
        f_yuv(326,361,960), f_yuv(369,336,929), f_yuv(412,311,897), f_yuv(454,287,866),
        f_yuv(497,262,835), f_yuv(540,237,804), f_yuv(583,212,772), f_yuv(626,188,741),
        f_yuv(669,163,710), f_yuv(712,138,679), f_yuv(754,113,647), f_yuv(797,89,616),
        f_yuv(840,64,585),  f_yuv(849,105,578), f_yuv(858,145,572), f_yuv(867,186,565),
        f_yuv(876,227,558), f_yuv(886,268,552), f_yuv(895,308,545), f_yuv(904,349,538),
        f_yuv(913,390,532), f_yuv(922,431,525), f_yuv(931,471,519), f_yuv(940,512,512));

    -- effect labels (K6)
    constant E_MATTE : integer := 0;
    constant E_HEAT  : integer := 1;
    constant E_MONO  : integer := 2;
    constant E_QT    : integer := 3;
    constant E_NEG   : integer := 4;
    constant E_SOL   : integer := 5;
    constant E_CRUSH : integer := 6;
    constant E_SMEAR : integer := 7;

    signal s1         : t_video_stream_yuv444_30b;
    signal ana_params : t_mk_analysis_params;
    signal rnd_params : t_mk_render_params;
    signal mk_key     : unsigned(9 downto 0);
    signal mk_matte   : unsigned(9 downto 0);
    signal mk_stats   : t_mk_stats;

    signal eff        : integer range 0 to 8 := E_HEAT;
    signal key_inv    : std_logic := '0';

    type t_vid is record
        y, u, v : unsigned(9 downto 0);
    end record;
    constant C_BLACK : t_vid := (to_unsigned(64, 10), C_MID, C_MID);
    type t_vpipe is array (2 to 8) of t_vid;
    signal vp     : t_vpipe := (others => C_BLACK);
    signal av     : std_logic_vector(2 to 7) := (others => '0');   -- avid along vp
    signal wet8   : t_vid := C_BLACK;
    signal dim8   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal sm_cnt : unsigned(4 downto 0) := (others => '0');       -- Smear block counter
    signal sm_hold : t_vid := C_BLACK;

    signal ia, ib : t_vid := C_BLACK;
    signal it     : unsigned(9 downto 0) := (others => '0');
    signal ry, ru, rv : unsigned(9 downto 0) := (others => '0');
    -- 3-stage mixer: a + ((b - a) * t7 + 32) >> 6, t7 = (t + 8) >> 4 (0..64)
    type t_dv is array (0 to 2) of signed(10 downto 0);
    type t_pv is array (0 to 2) of signed(18 downto 0);
    type t_uv is array (0 to 2) of unsigned(9 downto 0);
    signal mx_d   : t_dv := (others => (others => '0'));
    signal mx_p   : t_pv := (others => (others => '0'));
    signal mx_a1, mx_a2 : t_uv := (others => (others => '0'));
    signal mx_t7  : unsigned(6 downto 0) := (others => '0');

    signal s_out_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_out_u : unsigned(9 downto 0) := C_MID;
    signal s_out_v : unsigned(9 downto 0) := C_MID;

    -- generated avid (hsync-parity locked; see chantilly/aurora)
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_sel, g_par  : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := (others => '0');
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_avid : std_logic := '0';
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := (others => '0');
    signal gav_sr       : std_logic_vector(0 to C_LATENCY - 5) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');

begin

    --------------------------------------------------------------------------
    -- controls
    --------------------------------------------------------------------------
    p_ctl : process(clk)
        variable v_k6 : unsigned(9 downto 0);
        variable v_z  : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            ana_params.wake        <= unsigned(registers_in(0));
            ana_params.floor       <= unsigned(registers_in(1));
            ana_params.chroma      <= unsigned(registers_in(2));
            ana_params.mode        <= registers_in(6)(0);
            ana_params.plate       <= registers_in(6)(1);
            ana_params.steady      <= registers_in(6)(2);
            ana_params.both_fields <= '0';
            rnd_params.sens        <= unsigned(registers_in(7));
            rnd_params.softness    <= unsigned(registers_in(3));
            rnd_params.detail      <= unsigned(registers_in(4));
            rnd_params.invert      <= '0';            -- applied here (Heat needs the raw key)
            rnd_params.follow      <= registers_in(6)(3);
            key_inv                <= registers_in(6)(4);
            -- labelled K6: firmware loads the label INDEX, a turned knob gives 0..1023
            v_k6 := unsigned(registers_in(5));
            if v_k6 < 9 then
                eff <= to_integer(v_k6);
            else
                v_z := shift_left(resize(v_k6, 14), 3) + resize(v_k6, 14);
                eff <= to_integer(v_z(13 downto 10));
            end if;
        end if;
    end process p_ctl;

    --------------------------------------------------------------------------
    -- 1: input register (U/V swapped into BT.601 order)
    --------------------------------------------------------------------------
    p_in : process(clk)
    begin
        if rising_edge(clk) then
            s1.y       <= data_in.y;
            s1.u       <= data_in.v;
            s1.v       <= data_in.u;
            s1.avid    <= data_in.avid;
            s1.hsync_n <= data_in.hsync_n;
            s1.vsync_n <= data_in.vsync_n;
            s1.field_n <= data_in.field_n;
        end if;
    end process p_in;

    u_mk : entity work.motion_key
        generic map (G_TEST_OBJECT => C_TEST_OBJECT)
        port map (
            clk         => clk,
            ana_in      => s1,
            timing_id   => registers_in(8)(3 downto 0),
            ana_params  => ana_params,
            rnd_avid    => s1.avid,
            rnd_vsync_n => s1.vsync_n,
            rnd_y       => s1.y,
            rnd_params  => rnd_params,
            key         => mk_key,
            matte       => mk_matte,
            stats       => mk_stats,
            dbg_m_we    => open,
            dbg_m_addr  => open,
            dbg_m_data  => open,
            dbg_r_we    => open,
            dbg_r_addr  => open,
            dbg_r_data  => open,
            dbg_upd     => open);

    --------------------------------------------------------------------------
    -- 2..8: video delay; wet signal and Heat dim from 7 -> 8 (key arrives at 8)
    --------------------------------------------------------------------------
    p_vid : process(clk)
        variable v_d  : signed(10 downto 0);
        variable v_f  : unsigned(9 downto 0);
        variable v_x  : t_vid;
        variable v_c  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            vp(2) <= (unsigned(s1.y), unsigned(s1.u), unsigned(s1.v));
            av(2) <= s1.avid;
            for i in 3 to 8 loop
                vp(i) <= vp(i - 1);
            end loop;
            for i in 3 to 7 loop
                av(i) <= av(i - 1);
            end loop;

            -- Smear: horizontal sample-and-hold in blocks of W >> 7 pixels
            if av(7) = '0' then
                sm_cnt <= (others => '0');
            elsif sm_cnt = 0 then
                sm_hold <= vp(7);
                sm_cnt  <= resize(shift_right(mk_stats.width, 7), 5) - 1;
            else
                sm_cnt <= sm_cnt - 1;
            end if;

            -- wet signal of the selected effect
            v_x := vp(7);
            case eff is
                when E_QT =>      -- 90 degree hue turn: U' = 1024 - V, V' = U
                    wet8.y <= v_x.y;
                    if v_x.v = 0 then wet8.u <= to_unsigned(1023, 10);
                    else wet8.u <= 0 - v_x.v; end if;
                    wet8.v <= v_x.u;
                when E_NEG =>
                    if v_x.y > 1004 then wet8.y <= (others => '0');
                    else wet8.y <= to_unsigned(1004, 10) - v_x.y; end if;
                    if v_x.u = 0 then wet8.u <= to_unsigned(1023, 10);
                    else wet8.u <= 0 - v_x.u; end if;
                    if v_x.v = 0 then wet8.v <= to_unsigned(1023, 10);
                    else wet8.v <= 0 - v_x.v; end if;
                when E_SOL =>     -- luma folded at mid-grey: black->black, mid->white, white->black
                    if v_x.y < 64 then v_f := to_unsigned(64, 10); else v_f := v_x.y; end if;
                    if v_x.y > 1004 then
                        v_f := (others => '0');
                    elsif to_unsigned(1004, 10) - v_x.y < v_f then
                        v_f := to_unsigned(1004, 10) - v_x.y;
                    end if;
                    if v_f < 64 then
                        wet8.y <= to_unsigned(64, 10);
                    else
                        wet8.y <= shift_left(v_f - 64, 1) + 64;
                    end if;
                    wet8.u <= v_x.u;
                    wet8.v <= v_x.v;
                when E_CRUSH =>   -- ~3 bits of luma, 2 bits of chroma (neutral kept)
                    if v_x.y < 64 then v_f := (others => '0'); else v_f := v_x.y - 64; end if;
                    v_f := (v_f(9 downto 7) & "0000000") + 128;
                    if v_f > 940 then wet8.y <= to_unsigned(940, 10); else wet8.y <= v_f; end if;
                    v_c := resize(v_x.u, 11) + 128;
                    if v_c(10) = '1' then wet8.u <= to_unsigned(1023, 10);
                    else wet8.u <= v_c(9 downto 8) & "00000000"; end if;
                    v_c := resize(v_x.v, 11) + 128;
                    if v_c(10) = '1' then wet8.v <= to_unsigned(1023, 10);
                    else wet8.v <= v_c(9 downto 8) & "00000000"; end if;
                when E_SMEAR =>
                    if sm_cnt = 0 then wet8 <= v_x; else wet8 <= sm_hold; end if;
                when others =>    -- Mono
                    wet8 <= (v_x.y, C_MID, C_MID);
            end case;
            -- Heat background: mono dimmed 50% toward black
            v_d  := signed(resize(vp(7).y, 11)) - 64;
            v_d  := shift_right(v_d, 1) + 64;
            dim8 <= unsigned(v_d(9 downto 0));
        end if;
    end process p_vid;

    --------------------------------------------------------------------------
    -- 8: per-effect mixer inputs (registered by mixer stage A)
    --------------------------------------------------------------------------
    p_sel : process(all)
        variable v_pal : std_logic_vector(29 downto 0);
        variable v_key : unsigned(9 downto 0);
    begin
        v_key := mk_key;
        if key_inv = '1' then
            v_key := not mk_key;
        end if;
        v_pal := C_HEAT(to_integer(mk_matte(9 downto 4)));
        case eff is
            when E_HEAT =>
                ia <= (dim8, C_MID, C_MID);
                ib <= (unsigned(v_pal(29 downto 20)), unsigned(v_pal(19 downto 10)),
                       unsigned(v_pal(9 downto 0)));
                if mk_key(9 downto 8) /= "00" then
                    it <= (others => '1');
                else
                    it <= shift_left(resize(mk_key(7 downto 0), 10), 2);
                end if;
            when E_MONO | E_QT | E_NEG | E_SOL | E_CRUSH | E_SMEAR =>
                ia <= vp(8);
                ib <= wet8;
                it <= v_key;
            when others =>    -- Matte (and Cloak until Phase 4)
                ia <= C_BLACK;
                ib <= (to_unsigned(940, 10), C_MID, C_MID);
                it <= v_key;
        end case;
    end process p_sel;

    --------------------------------------------------------------------------
    -- 8..11: mixer (replaces 3 SDK interpolators: 11x7 vs 11x11 multipliers,
    -- ~700 LUT saved; 65 levels, exactly b at t = 1023). Model: mix7().
    --------------------------------------------------------------------------
    p_mix : process(clk)
        type t_in is array (0 to 2) of unsigned(9 downto 0);
        variable v_a, v_b : t_in;
        variable v_r : signed(12 downto 0);
        variable v_t : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            v_a := (ia.y, ia.u, ia.v);
            v_b := (ib.y, ib.u, ib.v);
            v_t := resize(it, 11) + 8;
            mx_t7 <= v_t(10 downto 4);
            for c in 0 to 2 loop
                -- A: difference
                mx_d(c)  <= signed(resize(v_b(c), 11)) - signed(resize(v_a(c), 11));
                mx_a1(c) <= v_a(c);
                -- B: product
                mx_p(c)  <= mx_d(c) * signed(resize(mx_t7, 8));
                mx_a2(c) <= mx_a1(c);
            end loop;
            -- C: round, add, clamp
            for c in 0 to 2 loop
                v_r := signed(resize(mx_a2(c), 13)) + resize(shift_right(mx_p(c) + 32, 6), 13);
                if v_r < 0 then
                    v_a(c) := (others => '0');
                elsif v_r > 1023 then
                    v_a(c) := (others => '1');
                else
                    v_a(c) := unsigned(v_r(9 downto 0));
                end if;
            end loop;
            ry <= v_a(0);
            ru <= v_a(1);
            rv <= v_a(2);
        end if;
    end process p_mix;

    --------------------------------------------------------------------------
    -- generated avid: starts 4 clocks after the source avid (5 when its
    -- hsync->avid parity is off the voted usual), ends after W pixels
    --------------------------------------------------------------------------
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_wm1 <= mk_stats.width - 1;
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

            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
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

    --------------------------------------------------------------------------
    -- 11 -> 12: blanking gate, U/V back to hardware order
    --------------------------------------------------------------------------
    p_out : process(clk)
    begin
        if rising_edge(clk) then
            if gav_sr(C_LATENCY - 6) = '0' then
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            else
                s_out_y <= ry;
                s_out_u <= rv;
                s_out_v <= ru;
            end if;
        end if;
    end process p_out;

    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            gav_sr       <= g_avid          & gav_sr(0 to C_LATENCY - 6);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= gav_sr(C_LATENCY - 5);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture wake;
