-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- starwipe.vhd  (v0.2.0 -- scanline engine)
--
-- STARWIPE: the wipe / pattern section of a production switcher.  The input
-- video (A) wipes to a generated B source; the P12 slider is the T-bar.
--
-- Every wipe is an analytic distance field.  A pattern is a FRAME TABLE of
-- rays from an anchor; the piece between two rays is a field that is linear
-- (polygon edges, bars) or circular (q = g (|p - c|^2 - R^2)); along a
-- scanline every piece is a quadratic and along a ray the crossing x and the
-- field value are polynomials in the line number.  So:
--
--  * the microcoded ENGINE (tools/sw_ucode2.py) runs once per field in
--    vertical blanking: controls, T-bar mapping with an analytic bound so
--    both ends of travel are exactly A / exactly B, colours, the pattern's
--    sector spec, and a RECORD per ray (x, dx, D, dD, ddD, s, ds) in the
--    DATA memory, in two lists per accumulator (lines above the anchor /
--    at and below);
--  * the LINE BUILDER steps each record once per line (adds only) and writes
--    a LINE TABLE of breakpoints (x, D, s) for the next line, ping-pong;
--  * the PIXEL PATH runs two quadratic accumulators (D += s; s += q),
--    reloaded at each breakpoint, combines them (A / min / max), keys with
--    key = 128 - D (key units: 256 per edge width, the edge gain is folded
--    into the table) and the border key = key + DK, then composes A ->
--    border -> B with one lerp per channel, chroma co-sited per 4:2:2 pair.
--
-- Hardware contracts: program-generated window at a fixed offset from
-- hsync; A's chroma pair swapped on lines whose source hsync->avid parity
-- differs (drape v1.3); outputs gated to neutral outside the window; one
-- latency (20) in every mode; A rides an EBR ring; the frame anchor is the
-- first vsync edge after active video.
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.starwipe_pkg.all;

architecture starwipe of program_top is
    -- data_in -> data_out latency, a multiple of 4; must match
    -- processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 20;

    -- The window starts G_K + 2 clocks after the A sample that is its pixel
    -- 0.  Stage k holds pixel p during clock c0 + p + k + 1 where c0 is the
    -- window's first clock; the output register is stage 8, so the latency
    -- is (G_K + 2) + 9 = 19 by that count, and the RTL sim (sw_simcmp
    -- against the emulator render) puts the key 2 px left of A at G_K = 6:
    -- G_K = 8 aligns them.  G_K moves the window by an EVEN count only, so
    -- the hsync -> avid parity (U/V pairing at the encoder) is unchanged.
    -- The line-table fetch for a line is primed C_PREF clocks before its
    -- window so entry 0 is loaded AT pixel 0 (a fetch takes ~4 clocks).
    constant G_K    : natural := 8;
    constant C_PREF  : natural := 6;
    constant RA_OFS : natural := 13;     -- A ring read offset: aligned with stage 4
    constant REC0   : natural := 256;    -- record set 0 in the DATA memory; set 1 at REC0 + 320

    ------------------------------------------------------------------------
    -- helpers
    ------------------------------------------------------------------------
    -- Exact signed product a * m, radix-4 Booth on m (hypnos).
    function bmul(a : signed; m : signed) return signed is
        constant NA  : natural := a'length;
        constant NML : natural := m'length;
        constant NM  : natural := NML + (NML mod 2);
        constant NP  : natural := NA + NML;
        constant NPP : natural := NM / 2;
        type t_pp is array (0 to 7) of signed(NP - 1 downto 0);
        variable va   : signed(NA - 1 downto 0);
        variable vm   : signed(NM - 1 downto 0);
        variable me   : std_logic_vector(NM downto 0);
        variable a1, a2, mag, pp, corr : signed(NP - 1 downto 0);
        variable d    : std_logic_vector(2 downto 0);
        variable p    : t_pp;
    begin
        va := a;
        vm := resize(m, NM);
        me := std_logic_vector(vm) & '0';
        a1 := resize(va, NP);
        a2 := shift_left(a1, 1);
        corr := (others => '0');
        for i in 0 to 7 loop
            p(i) := (others => '0');
        end loop;
        for i in 0 to NPP - 1 loop
            d := me(2 * i + 2 downto 2 * i);
            case d is
                when "001" | "010" | "101" | "110" => mag := a1;
                when "011" | "100"                 => mag := a2;
                when others                        => mag := (others => '0');
            end case;
            if d(2) = '1' and d /= "111" then
                pp := not mag;
                corr(2 * i) := '1';
            else
                pp := mag;
            end if;
            p(i) := shift_left(pp, 2 * i);
        end loop;
        p(NPP) := corr;
        return ((p(0) + p(1)) + (p(2) + p(3))) + ((p(4) + p(5)) + (p(6) + p(7)));
    end function;

    -- Saturate to w bits: x fits when the bits above bit w-2 all equal the
    -- sign (a reduce, not two wide comparators).
    function sat_s(x : signed; w : natural) return signed is
        variable hi, lo : signed(w - 1 downto 0);
        variable top    : std_logic_vector(x'high downto x'low + w - 1);
    begin
        hi := (others => '0'); hi(w - 2 downto 0) := (others => '1');     -- 2^(w-1) - 1
        lo := (others => '1'); lo(w - 2 downto 0) := (others => '0');     -- -2^(w-1)
        top := std_logic_vector(x(x'high downto x'low + w - 1));
        if top = (top'range => '0') or top = (top'range => '1') then
            return x(x'low + w - 1 downto x'low);
        elsif x(x'high) = '1' then
            return lo;
        else
            return hi;
        end if;
    end function;

    -- register window: r >= 96 -> 128 + 32 bank + (r - 96)
    function ra_of(r : unsigned(6 downto 0); bank : unsigned(1 downto 0)) return unsigned is
    begin
        if r(6 downto 5) = "11" then
            return "1" & bank & r(4 downto 0);
        end if;
        return '0' & r;
    end function;

    ------------------------------------------------------------------------
    -- raster measurement, anchor, generated window
    ------------------------------------------------------------------------
    signal m_hsq, m_avq, m_vsq : std_logic := '1';
    signal m_hc   : unsigned(11 downto 0) := (others => '0');
    signal m_la   : std_logic := '0';
    signal m_offl : unsigned(11 downto 0) := (others => '0');
    signal m_off  : unsigned(11 downto 0) := to_unsigned(200, 12);
    signal m_offv : std_logic := '0';                             -- m_off / m_w latched at least once
    signal m_offk : unsigned(11 downto 0) := to_unsigned(206, 12);
    signal m_offp : unsigned(11 downto 0) := to_unsigned(200, 12);   -- m_offk - C_PREF: fetch priming
    signal m_avc  : unsigned(11 downto 0) := (others => '0');
    signal m_wl   : unsigned(11 downto 0) := (others => '0');
    signal m_w    : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal m_lc   : unsigned(10 downto 0) := (others => '0');
    signal m_h    : unsigned(10 downto 0) := to_unsigned(243, 11);
    signal m_saw  : std_logic := '0';
    signal m_fld  : std_logic := '1';
    signal s_vs_go : std_logic := '0';

    signal g_go   : std_logic := '0';
    signal g_pre  : std_logic := '0';                             -- C_PREF clocks before g_go: prime the fetchers
    signal g_av   : std_logic := '0';
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_swp  : std_logic := '0';
    signal g_x    : unsigned(11 downto 0) := (others => '0');
    signal g_line : unsigned(10 downto 0) := (others => '0');
    signal r_vline : unsigned(10 downto 0) := (others => '0');
    signal r_vok  : std_logic := '0';

    ------------------------------------------------------------------------
    -- engine ports (per field)
    ------------------------------------------------------------------------
    signal p_dk    : unsigned(17 downto 0) := (others => '0');
    signal p_tk    : unsigned(8 downto 0) := (others => '0');
    signal p_mode  : std_logic_vector(3 downto 0) := (others => '0');   -- [1:0] op, [2] mix, [3] reverse
    signal p_qa, p_qb : signed(23 downto 0) := (others => '0');
    signal p_bcy, p_bcu, p_bcv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p_rcy, p_rcu, p_rcv : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_desc is array (0 to 3) of unsigned(17 downto 0);          -- record address << 8 | count
    signal p_lst   : t_desc := (others => (others => '0'));
    signal p_aya, p_ayb : unsigned(10 downto 0) := (others => '0');
    signal p_ready : std_logic := '0';                             -- a set is live for the builder
    signal p_set   : std_logic := '0';                             -- the live record set
    -- the engine stores the next field's ports in the DATA port block
    -- (896 + 16 set + index); the builder loads them at the anchor
    signal lb_set  : std_logic := '0';
    signal u_set   : std_logic := '1';                             -- the set the engine writes
    signal u_ran   : std_logic := '0';                             -- the engine has finished once

    ------------------------------------------------------------------------
    -- DATA memory (1024 x 32: spec, header, two record sets, scratch) and
    -- the line table (256 x 64)
    ------------------------------------------------------------------------
    type t_dmem is array (0 to 1023) of std_logic_vector(31 downto 0);
    signal dmem : t_dmem := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of dmem : signal is true;
    signal dm_a  : unsigned(9 downto 0) := (others => '0');
    signal dm_we : std_logic := '0';
    signal dm_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal dm_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal dmb_a  : unsigned(9 downto 0) := (others => '0');
    signal dmb_we : std_logic := '0';
    signal dmb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal dme_a  : unsigned(9 downto 0) := (others => '0');
    signal dme_we : std_logic := '0';
    signal dme_wd : std_logic_vector(31 downto 0) := (others => '0');

    type t_ltab is array (0 to 255) of std_logic_vector(63 downto 0);
    signal ltab  : t_ltab := (others => (others => '0'));
    attribute no_rw_check of ltab : signal is true;
    signal lt_wa : unsigned(7 downto 0) := (others => '0');
    signal lt_we : std_logic := '0';
    signal lt_wd : std_logic_vector(63 downto 0) := (others => '0');
    signal lt_ra : unsigned(7 downto 0) := (others => '0');
    signal lt_q  : std_logic_vector(63 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- line builder
    ------------------------------------------------------------------------
    type t_lb is (LBS_IDLE, LBS_PORTS, LBS_ACC, LBS_RD, LBS_STEP, LBS_WR, LBS_CEIL, LBS_ENTRY, LBS_SHIFT, LBS_MULQ, LBS_MULW, LBS_EMIT, LBS_NEXT, LBS_END);
    signal lb_st   : t_lb := LBS_IDLE;
    signal lb_acc  : unsigned(1 downto 0) := (others => '0');
    signal lb_idx  : unsigned(7 downto 0) := (others => '0');
    signal lb_cnt  : unsigned(7 downto 0) := (others => '0');
    signal lb_step : std_logic := '0';
    signal lb_last : std_logic_vector(1 downto 0) := "00";
    signal lb_lastv : std_logic_vector(1 downto 0) := "00";       -- '1' = the list of the previous line is valid
    signal lb_w    : unsigned(4 downto 0) := (others => '0');
    signal lb_rec  : unsigned(9 downto 0) := (others => '0');
    signal lb_x, lb_dx, lb_D, lb_dD, lb_ddD, lb_s : signed(31 downto 0) := (others => '0');
    signal lb_rng  : std_logic_vector(31 downto 0) := (others => '0');
    signal lb_out  : unsigned(6 downto 0) := (others => '0');
    signal lb_xi   : unsigned(11 downto 0) := (others => '0');
    signal lb_q    : signed(23 downto 0) := (others => '0');
    signal lb_qx   : signed(35 downto 0) := (others => '0');     -- q * xi, Q.16
    signal lb_wv_s : signed(36 downto 0) := (others => '0');     -- s0 + q (xi - 1) / 2, Q.16
    signal lb_p    : signed(48 downto 0) := (others => '0');     -- xi * w
    signal lb_mc   : unsigned(3 downto 0) := (others => '0');
    signal lb_mw   : unsigned(1 downto 0) := (others => '0');    -- MULW phase: 0 qx-q, 1 w, 2 steps
    signal lb_xc   : signed(32 downto 0) := (others => '0');     -- x + 65535 (ceil = bits 32..16)
    signal lb_inr, lb_skp : std_logic := '0';                    -- line inside the ray's range / skip it
    signal lb_wq   : signed(36 downto 0) := (others => '0');     -- (qx - q) / 2
    signal lb_mb   : unsigned(11 downto 0) := (others => '0');   -- multiplier bits, MSB first
    signal lb_bank : std_logic := '0';
    -- entry spacing: the fetcher needs ~5 clocks per load, so an entry
    -- < 5 px right of the previous one is dropped (first-eligible rule) and
    -- one at the SAME x overwrites it (coincident crossings at the anchor
    -- row / clamped at x = 0: the last piece applies); lb_px holds the x of
    -- the entry in slot lb_out
    signal lb_pv   : std_logic := '0';
    signal lb_px   : unsigned(11 downto 0) := (others => '0');      -- x of the entry in slot lb_out
    signal lb_fnew : std_logic := '0';                            -- xi >= px + 5 (or no pending), registered every clock
    signal lb_wx   : unsigned(11 downto 0) := (others => '0');      -- x of the last COMMITTED entry (slot lb_out - 1)
    signal lb_wv   : std_logic := '0';
    signal lb_wx5  : signed(16 downto 0) := (others => '0');       -- lb_wx + 5, registered
    -- list selection (line above the anchor -> up list) pre-registered in
    -- p_gen for the next line (r_sel*) and for line 0 (r_sel0*); lb_l0 says
    -- which pair LBS_ACC reads (the compare was a 13 ns path inside ACC)
    signal r_sela, r_selb, r_sel0a, r_sel0b : std_logic := '0';
    signal lb_l0   : std_logic := '0';
    signal lb_built : unsigned(11 downto 0) := (others => '1');
    signal lb_busy : std_logic := '0';
    signal lb_line : unsigned(10 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- pixel path
    ------------------------------------------------------------------------
    signal fa_ptr, fb_ptr : unsigned(5 downto 0) := (others => '0');
    signal fa_x, fb_x : unsigned(11 downto 0) := (others => '1');
    signal fa_D, fb_D : signed(31 downto 0) := (others => '0');
    signal fa_s, fb_s : signed(19 downto 0) := (others => '0');
    signal f_sel  : std_logic := '0';
    signal f_pend : std_logic_vector(1 downto 0) := "00";
    signal f_pend2 : std_logic_vector(1 downto 0) := "00";
    signal f_need : std_logic_vector(1 downto 0) := "00";
    signal acc_DA, acc_DB : signed(31 downto 0) := (others => '0');
    signal acc_sA, acc_sB : signed(27 downto 0) := (others => '0');   -- Q.16
    signal s1_D   : signed(31 downto 0) := (others => '0');
    signal s2_kr  : signed(24 downto 0) := (others => '0');
    signal s3_ki, s3_ko : unsigned(8 downto 0) := (others => '0');
    type t_c3 is array (0 to 2) of unsigned(9 downto 0);
    type t_d3 is array (0 to 2) of signed(10 downto 0);
    signal s4_x : t_c3 := (others => (others => '0'));
    signal s4_d : t_d3 := (others => (others => '0'));
    signal s4_k, s4_kc : unsigned(8 downto 0) := (others => '0');
    signal s5_d : t_d3 := (others => (others => '0'));
    signal s5_x : t_c3 := (others => (others => '0'));
    signal s5_k, s5_kc : unsigned(8 downto 0) := (others => '0');
    type t_p3 is array (0 to 2) of signed(19 downto 0);
    signal s6_p : t_p3 := (others => (others => '0'));
    signal s6_x : t_c3 := (others => (others => '0'));
    signal s7_o : t_c3 := (others => (others => '0'));
    signal hold_x : t_c3 := (others => (others => '0'));
    signal hold_d : t_d3 := (others => (others => '0'));
    signal hold_k : unsigned(8 downto 0) := (others => '0');
    signal fl_gv : std_logic_vector(0 to 8) := (others => '0');
    signal fl_sw : std_logic_vector(0 to 4) := (others => '0');
    signal fl_od : std_logic_vector(0 to 4) := (others => '0');
    type t_r32 is array (0 to 31) of std_logic_vector(31 downto 0);
    signal ra_mem : t_r32;
    attribute no_rw_check of ra_mem : signal is true;
    signal r_wa : unsigned(4 downto 0) := (others => '0');
    signal ra_q, ra_q2 : std_logic_vector(31 downto 0) := (others => '0');
    signal fn_d : std_logic_vector(0 to RA_OFS + 1) := (others => '1');   -- field_n, same delay as the ring
    type t_sy is array (0 to 2) of std_logic_vector(2 downto 0);
    signal sy_d : t_sy := (others => (others => '1'));
    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');
    signal s_out_s : std_logic_vector(3 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- microcoded engine v2 (32-bit)
    ------------------------------------------------------------------------
    constant ST_FETCH : unsigned(2 downto 0) := "000";
    constant ST_DEC   : unsigned(2 downto 0) := "001";
    constant ST_RDA   : unsigned(2 downto 0) := "010";
    constant ST_RDB   : unsigned(2 downto 0) := "011";
    constant ST_EX    : unsigned(2 downto 0) := "100";
    signal u_st   : unsigned(2 downto 0) := ST_FETCH;
    signal u_pc   : unsigned(10 downto 0) := (others => '0');
    signal u_fa   : unsigned(10 downto 0);
    signal u_rom  : std_logic_vector(31 downto 0) := (others => '0');
    signal u_ir   : std_logic_vector(31 downto 0) := (others => '0');
    signal u_done : std_logic := '1';
    signal u_pause : std_logic := '0';                           -- builder owned the port last cycle
    signal u_cyc  : unsigned(5 downto 0) := (others => '0');
    signal u_bank : unsigned(1 downto 0) := (others => '0');
    type t_rf is array (0 to 255) of std_logic_vector(31 downto 0);
    signal rf     : t_rf := (others => (others => '0'));
    attribute no_rw_check of rf : signal is true;
    signal rf_ra  : unsigned(7 downto 0);
    signal rf_q   : std_logic_vector(31 downto 0) := (others => '0');
    signal u_we, u_wlast : std_logic := '0';
    signal u_wa   : unsigned(7 downto 0) := (others => '0');
    signal u_wd   : std_logic_vector(31 downto 0) := (others => '0');
    signal u_a, u_b : signed(31 downto 0) := (others => '0');
    signal u_blt, u_bnz : std_logic := '0';
    signal u_ctlr : signed(31 downto 0) := (others => '0');
    signal u_shc  : unsigned(5 downto 0) := (others => '0');
    -- one register set for the serial units (shift, MUL, DIV, SQRT); every
    -- result is delivered in mu_pl
    signal mu_a   : signed(31 downto 0) := (others => '0');      -- multiplicand / divisor / root
    signal mu_ph  : signed(33 downto 0) := (others => '0');      -- product high / remainder
    signal mu_pl  : unsigned(31 downto 0) := (others => '0');    -- shift reg / product low / quotient / radicand
    signal mu_bp  : std_logic := '0';
    signal mu_shf : std_logic := '0';                            -- MUL result-shift phase (addend 0)
    signal u_f_sh, u_f_shr, u_f_shv, u_f_mul, u_f_div, u_f_sq, u_f_ser : std_logic := '0';   -- op class, latched at DEC
    signal f_c0, f_le15, f_le16, f_le24, f_le48, f_eq25 : std_logic := '0';         -- u_cyc flags (next cycle)
    signal sn_ea  : unsigned(7 downto 0) := (others => '0');
    signal sn_spec, sn_neg : std_logic := '0';
    signal sn_q, sn_q2 : std_logic_vector(15 downto 0) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- Raster measurement and the frame anchor
    ------------------------------------------------------------------------
    p_meas : process(clk)
        variable v_d : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            m_hsq <= data_in.hsync_n;
            m_avq <= data_in.avid;
            m_vsq <= data_in.vsync_n;
            s_vs_go <= '0';

            if m_hsq = '1' and data_in.hsync_n = '0' then
                m_hc <= (others => '0');
                m_la <= '0';
            elsif m_hc /= 4095 then
                m_hc <= m_hc + 1;
            end if;
            if data_in.avid = '1' and m_avq = '0' then
                m_la   <= '1';
                m_offl <= m_hc;
                m_avc  <= to_unsigned(1, 12);
                m_fld  <= data_in.field_n;
                if m_lc /= 2047 then m_lc <= m_lc + 1; end if;
            elsif data_in.avid = '1' then
                if m_avc /= 4095 then m_avc <= m_avc + 1; end if;
            end if;
            if data_in.avid = '0' and m_avq = '1' then
                m_wl <= m_avc;
            end if;

            if data_in.avid = '1' then
                m_saw <= '1';
            elsif m_vsq = '1' and data_in.vsync_n = '0' and m_saw = '1' then
                m_saw   <= '0';
                s_vs_go <= '1';
                -- the first field latches unconditionally: a source whose
                -- offset sits within 1 of the initial value would otherwise
                -- never be latched, leaving every line's parity "different"
                -- and the per-line U/V swap on for the whole picture
                if m_offl > m_off then v_d := m_offl - m_off; else v_d := m_off - m_offl; end if;
                if v_d >= 2 or m_offv = '0' then m_off <= m_offl; end if;
                if m_wl > m_w then v_d := m_wl - m_w; else v_d := m_w - m_wl; end if;
                if (v_d >= 2 or m_offv = '0') and m_wl > 15 then m_w <= m_wl; end if;
                m_offv <= '1';
                if m_lc > 15 then m_h <= m_lc; end if;
                m_lc <= (others => '0');
            end if;
            m_offk <= m_off + G_K;
            m_offp <= m_off + (G_K - C_PREF);
        end if;
    end process p_meas;

    ------------------------------------------------------------------------
    -- Generated window; g_line = field line shown (windows since the anchor)
    ------------------------------------------------------------------------
    p_gen : process(clk)
    begin
        if rising_edge(clk) then
            g_go  <= '0';
            g_pre <= '0';
            if m_hc = m_offk and m_la = '1' then
                g_go  <= '1';
                g_swp <= m_offl(0) xor m_off(0);
            end if;
            if m_hc = m_offp and m_la = '1' then
                g_pre <= '1';
            end if;
            if g_go = '1' then
                g_av  <= '1';
                g_cnt <= m_w - 1;
                g_x   <= (others => '0');
            elsif g_av = '1' then
                g_x <= g_x + 1;
                if g_cnt = 0 or (m_hsq = '1' and data_in.hsync_n = '0') then
                    g_av   <= '0';
                    g_line <= g_line + 1;
                else
                    g_cnt <= g_cnt - 1;
                end if;
            end if;
            if s_vs_go = '1' then
                g_line <= (others => '0');
            end if;
            -- the builder's next line and whether it is wanted (quasi-static:
            -- g_line changes at the window end, g_go comes a blanking later)
            r_vline <= g_line + 1;
            if resize(r_vline, 12) /= lb_built and r_vline < m_h then
                r_vok <= '1';
            else
                r_vok <= '0';
            end if;
            if r_vline < p_aya then r_sela <= '1'; else r_sela <= '0'; end if;
            if r_vline < p_ayb then r_selb <= '1'; else r_selb <= '0'; end if;
            if p_aya /= 0 then r_sel0a <= '1'; else r_sel0a <= '0'; end if;
            if p_ayb /= 0 then r_sel0b <= '1'; else r_sel0b <= '0'; end if;
        end if;
    end process p_gen;

    ------------------------------------------------------------------------
    -- DATA memory: one port; the builder owns it while busy
    ------------------------------------------------------------------------
    -- The engine's registered write pulse may land in the builder's FIRST busy
    -- cycle (the builder never accesses the port in that cycle), so it has
    -- priority; the engine resumes one cycle after the builder releases the
    -- port so a pending LDD sees its own word in dm_q.
    dm_a  <= dme_a when dme_we = '1' else dmb_a when lb_busy = '1' else dme_a;
    dm_we <= dme_we or (dmb_we and lb_busy);
    dm_wd <= dme_wd when dme_we = '1' else dmb_wd;

    p_dmem : process(clk)
    begin
        if rising_edge(clk) then
            if dm_we = '1' then
                dmem(to_integer(dm_a)) <= dm_wd;
            end if;
            dm_q <= dmem(to_integer(dm_a));
        end if;
    end process p_dmem;

    p_ltab : process(clk)
    begin
        if rising_edge(clk) then
            if lt_we = '1' then
                ltab(to_integer(lt_wa)) <= lt_wd;
            end if;
            lt_q <= ltab(to_integer(lt_ra));
        end if;
    end process p_ltab;

    ------------------------------------------------------------------------
    -- Line builder.  Builds line g_line + 1 at each window start (g_go)
    -- once the engine has published the field (p_ready); line 0 as soon as
    -- p_ready rises.  Per accumulator: pick the list (up if line < ay else
    -- dn); if the same list served the previous line, step every record;
    -- emit entries (x, D at pixel x, s) in list order, then a sentinel.
    ------------------------------------------------------------------------
    lb_busy <= '0' when lb_st = LBS_IDLE else '1';

    p_lb : process(clk)
        variable v_line : unsigned(10 downto 0);
        variable v_desc : unsigned(17 downto 0);
        variable v_sel  : std_logic;
        variable v_xi   : signed(16 downto 0);
        variable v_Dq   : signed(33 downto 0);
        variable v_sq   : signed(36 downto 0);
        variable v_same : std_logic;
        variable v_skip : std_logic;
        variable v_word : std_logic_vector(63 downto 0);
        variable v_slot : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            lt_we  <= '0';
            dmb_we <= '0';
            -- entry-spacing flags for EMIT (lb_xi settles at ENTRY, lb_px /
            -- lb_pv at the previous EMIT: both >= 12 clocks before use)
            if lb_pv = '0' or resize(lb_xi, 13) >= resize(lb_px, 13) + 5 then lb_fnew <= '1'; else lb_fnew <= '0'; end if;
            lb_wx5 <= signed(resize(lb_wx, 17)) + 5;
            case lb_st is
                when LBS_IDLE =>
                    if s_vs_go = '1' or p_ready = '0' then
                        lb_lastv <= "00";
                        lb_built <= (others => '1');
                        if s_vs_go = '1' and u_done = '1' and u_ran = '1' then
                            -- the anchor: the finished set goes live; its
                            -- ports are loaded from the DATA port block
                            lb_set <= u_set;
                            lb_w   <= (others => '0');
                            lb_st  <= LBS_PORTS;
                        end if;
                    elsif lb_built = x"FFF" then
                        -- line 0 right after the ports load
                        lb_built <= (others => '0');
                        lb_line  <= (others => '0');
                        lb_bank  <= '0';
                        lb_acc   <= "00";
                        lb_l0    <= '1';
                        lb_st    <= LBS_ACC;
                    elsif g_go = '1' and r_vok = '1' then
                        -- the next line (decision pre-registered in p_gen)
                        lb_built <= resize(r_vline, 12);
                        lb_line  <= r_vline;
                        lb_bank  <= r_vline(0);
                        lb_acc   <= "00";
                        lb_l0    <= '0';
                        lb_st    <= LBS_ACC;
                    end if;

                when LBS_PORTS =>
                    -- 15 words; the data arrives two clocks after the address
                    if lb_w <= 14 then
                        dmb_a <= "11100" & lb_set & lb_w(3 downto 0);
                    end if;
                    case to_integer(lb_w) is
                        when 2  => p_dk   <= unsigned(dm_q(17 downto 0));
                        when 3  => p_bcy  <= unsigned(dm_q(9 downto 0));
                        when 4  => p_bcu  <= unsigned(dm_q(19 downto 10)); p_bcv <= unsigned(dm_q(9 downto 0));
                        when 5  => p_rcy  <= unsigned(dm_q(9 downto 0));
                        when 6  => p_rcu  <= unsigned(dm_q(19 downto 10)); p_rcv <= unsigned(dm_q(9 downto 0));
                        when 7  => p_tk   <= unsigned(dm_q(8 downto 0));
                        when 8  => p_mode <= dm_q(3 downto 0);
                        when 9  => p_qa   <= signed(dm_q(23 downto 0));
                        when 10 => p_qb   <= signed(dm_q(23 downto 0));
                        when 11 => p_lst(0) <= unsigned(dm_q(17 downto 0));
                        when 12 => p_lst(1) <= unsigned(dm_q(17 downto 0));
                        when 13 => p_aya  <= unsigned(dm_q(10 downto 0));
                        when 14 => p_lst(2) <= unsigned(dm_q(17 downto 0));
                        when 15 => p_lst(3) <= unsigned(dm_q(17 downto 0));
                        when 16 => p_ayb  <= unsigned(dm_q(10 downto 0));
                                   p_set   <= lb_set;
                                   p_ready <= '1';
                                   lb_st   <= LBS_IDLE;
                        when others => null;
                    end case;
                    lb_w <= lb_w + 1;

                when LBS_ACC =>
                    -- v_sel = '0' (up list) when the line is above the anchor;
                    -- the compares are pre-registered (r_sel*, r_sel0*)
                    if lb_l0 = '1' then
                        if lb_acc = "00" then v_sel := not r_sel0a; else v_sel := not r_sel0b; end if;
                    else
                        if lb_acc = "00" then v_sel := not r_sela; else v_sel := not r_selb; end if;
                    end if;
                    if lb_acc = "00" then
                        if v_sel = '0' then v_desc := p_lst(0); else v_desc := p_lst(1); end if;
                        if lb_lastv(0) = '1' and lb_last(0) = v_sel then v_same := '1'; else v_same := '0'; end if;
                        lb_last(0) <= v_sel; lb_lastv(0) <= '1';
                    else
                        if v_sel = '0' then v_desc := p_lst(2); else v_desc := p_lst(3); end if;
                        if lb_lastv(1) = '1' and lb_last(1) = v_sel then v_same := '1'; else v_same := '0'; end if;
                        lb_last(1) <= v_sel; lb_lastv(1) <= '1';
                    end if;
                    lb_step <= v_same;
                    if lb_acc = "00" then lb_q <= p_qa; else lb_q <= p_qb; end if;
                    lb_cnt  <= v_desc(7 downto 0);
                    lb_idx  <= (others => '0');
                    lb_out  <= (others => '0');
                    lb_pv   <= '0';
                    lb_wv   <= '0';
                    lb_rec <= v_desc(17 downto 8);                  -- the engine stores the address
                    lb_w    <= (others => '0');
                    if v_desc(7 downto 0) = 0 then
                        lb_st <= LBS_END;
                    else
                        lb_st <= LBS_RD;
                    end if;

                when LBS_RD =>
                    -- issue 7 reads; the data arrives two clocks after the
                    -- address (address register + RAM output register)
                    if lb_w <= 6 then
                        dmb_a <= lb_rec + lb_w;
                    end if;
                    case to_integer(lb_w) is
                        when 2 => lb_x   <= signed(dm_q);
                        when 3 => lb_dx  <= signed(dm_q);
                        when 4 => lb_D   <= signed(dm_q);
                        when 5 => lb_dD  <= signed(dm_q);
                        when 6 => lb_ddD <= signed(dm_q);
                        when 7 => lb_s   <= signed(dm_q);
                        when 8 => lb_rng <= dm_q;
                        when others => null;
                    end case;
                    if lb_w = 8 then
                        lb_w  <= (others => '0');
                        lb_st <= LBS_STEP;
                    else
                        lb_w <= lb_w + 1;
                    end if;

                when LBS_STEP =>
                    if lb_step = '1' then
                        lb_x  <= lb_x + lb_dx;
                        lb_D  <= lb_D + shift_right(lb_dD, 8);
                        lb_dD <= lb_dD + shift_right(lb_ddD, 4);
                        lb_st <= LBS_WR;
                    else
                        lb_st <= LBS_CEIL;
                    end if;
                    lb_w <= (others => '0');

                when LBS_WR =>
                    dmb_we <= '1';
                    case to_integer(lb_w) is
                        when 0 => dmb_a <= lb_rec + 0; dmb_wd <= std_logic_vector(lb_x);
                        when 1 => dmb_a <= lb_rec + 2; dmb_wd <= std_logic_vector(lb_D);
                        when others => dmb_a <= lb_rec + 3; dmb_wd <= std_logic_vector(lb_dD);
                    end case;
                    if lb_w = 2 then
                        lb_st <= LBS_CEIL;
                    end if;
                    lb_w <= lb_w + 1;

                when LBS_CEIL =>
                    -- ceil(x) = (x + 65535) >> 16, and the line-range decision,
                    -- registered for ENTRY (timing)
                    lb_xc <= resize(lb_x, 33) + 65535;
                    lb_inr <= '1'; lb_skp <= '0';
                    if lb_idx /= 0 then
                        if lb_line < unsigned(lb_rng(10 downto 0)) then
                            lb_inr <= '0'; lb_skp <= not lb_rng(22);
                        elsif lb_line > unsigned(lb_rng(21 downto 11)) then
                            lb_inr <= '0'; lb_skp <= not lb_rng(23);
                        end if;
                    else
                        lb_inr <= '0';                                   -- START record: x = 0
                    end if;
                    lb_st <= LBS_ENTRY;

                when LBS_ENTRY =>
                    -- START record: x = 0.  Ray: inside its active line range
                    -- xi = ceil(x) (clamped at 0, skipped past the right edge);
                    -- outside it, loaded at 0 when the crossing is far left,
                    -- skipped when far right.
                    v_xi := (others => '0');
                    v_skip := lb_skp;
                    if lb_inr = '1' then
                        v_xi := lb_xc(32 downto 16);
                        if v_xi >= 4095 then
                            v_skip := '1';
                        elsif v_xi < 0 then
                            v_xi := (others => '0');
                        end if;
                    end if;
                    if v_skip = '1' then
                        lb_st <= LBS_NEXT;
                    else
                        lb_xi <= unsigned(v_xi(11 downto 0));
                        lb_st <= LBS_SHIFT;
                    end if;

                when LBS_SHIFT =>
                    -- entry spacing: raise x to the last committed x + 5 (the
                    -- fetcher needs ~5 clocks per load); D/s are evaluated at
                    -- the raised x by the multiplies that follow
                    -- (before any commit the pending entry is the x = 0 START /
                    -- clamped ray: a newcomer at 1..4 is raised to 5 so slot 0
                    -- always holds x = 0 and pixels 0..4 have an entry)
                    if lb_wv = '1' and signed(resize(lb_xi, 17)) < lb_wx5 then
                        lb_xi <= unsigned(lb_wx5(11 downto 0));
                        lb_mb <= unsigned(lb_wx5(11 downto 0));
                    elsif lb_wv = '0' and lb_pv = '1' and lb_xi /= 0 and lb_xi < 5 then
                        lb_xi <= to_unsigned(5, 12);
                        lb_mb <= to_unsigned(5, 12);
                    else
                        lb_mb <= lb_xi;
                    end if;
                    lb_qx <= (others => '0');
                    lb_p  <= (others => '0');
                    lb_mc <= (others => '0');
                    if lb_q = 0 then
                        lb_wv_s <= resize(lb_s, 37);
                        lb_mw <= "10";                               -- straight to the steps
                        lb_st <= LBS_MULW;
                    else
                        lb_st <= LBS_MULQ;
                    end if;

                when LBS_MULQ =>
                    -- qx = q * xi (12 shift-add steps, xi's bits MSB first)
                    if lb_mb(11) = '1' then
                        lb_qx <= shift_left(lb_qx, 1) + resize(lb_q, 36);
                    else
                        lb_qx <= shift_left(lb_qx, 1);
                    end if;
                    lb_mb <= lb_mb(10 downto 0) & '0';
                    if lb_mc = 11 then
                        lb_st <= LBS_MULW;
                        lb_mw <= "00";
                    end if;
                    lb_mc <= lb_mc + 1;

                when LBS_MULW =>
                    -- w = s0 + (qx - q) / 2 in two clocks (one adder each), then
                    -- p = xi * w (12 shift-add steps, xi's bits MSB first)
                    case lb_mw is
                        when "00" =>
                            lb_wq <= shift_right(resize(lb_qx, 37) - resize(lb_q, 37), 1);
                            lb_mw <= "01";
                        when "01" =>
                            lb_wv_s <= resize(lb_s, 37) + lb_wq;
                            lb_mb <= lb_xi;
                            lb_mc <= (others => '0');
                            lb_mw <= "10";
                        when others =>
                            if lb_mb(11) = '1' then
                                lb_p <= shift_left(lb_p, 1) + resize(lb_wv_s, 49);
                            else
                                lb_p <= shift_left(lb_p, 1);
                            end if;
                            lb_mb <= lb_mb(10 downto 0) & '0';
                            if lb_mc = 11 then
                                lb_st <= LBS_EMIT;
                            end if;
                            lb_mc <= lb_mc + 1;
                    end case;

                when LBS_EMIT =>
                    -- D' = D0 + p >> 8 ; s' = s0 + q xi
                    v_Dq := resize(lb_D, 34) + resize(shift_right(lb_p, 8), 34);
                    v_sq := resize(lb_s, 37) + lb_qx;
                    v_word := std_logic_vector(lb_xi) &
                              std_logic_vector(sat_s(shift_right(v_sq, 8), 20)) &
                              std_logic_vector(sat_s(v_Dq, 32));
                    -- the newcomer always takes over: into the next slot when it
                    -- lies >= 5 px right of the pending entry (which is thereby
                    -- committed), else over the pending slot (pending dropped)
                    if lb_fnew = '1' then
                        if lb_pv = '1' then
                            v_slot := lb_out + 1;
                            lb_wx  <= lb_px;
                            lb_wv  <= '1';
                        else
                            v_slot := lb_out;
                        end if;
                        lb_out <= v_slot;
                    else
                        v_slot := lb_out;
                    end if;
                    lt_we <= '1';
                    lt_wa <= lb_bank & lb_acc(0) & v_slot(5 downto 0);
                    lt_wd <= v_word;
                    lb_px <= lb_xi;
                    lb_pv <= '1';
                    lb_st <= LBS_NEXT;

                when LBS_NEXT =>
                    lb_idx <= lb_idx + 1;
                    lb_rec <= lb_rec + 7;
                    lb_w   <= (others => '0');
                    if lb_idx + 1 >= lb_cnt or lb_out >= 62 then
                        lb_st <= LBS_END;
                    else
                        lb_st <= LBS_RD;
                    end if;

                when LBS_END =>
                    -- terminator after the last used slot
                    if lb_pv = '1' then v_slot := lb_out + 1; else v_slot := lb_out; end if;
                    lt_we <= '1';
                    lt_wa <= lb_bank & lb_acc(0) & v_slot(5 downto 0);
                    lt_wd <= x"FFF" & x"00000" & x"00000000";
                    if lb_acc = "00" then
                        lb_acc <= "01";
                        lb_st  <= LBS_ACC;
                    else
                        lb_st <= LBS_IDLE;
                    end if;
            end case;
        end if;
    end process p_lb;

    ------------------------------------------------------------------------
    -- Pixel path.  Two fetchers share the table read port (A on even
    -- clocks, B on odd); each holds its NEXT entry.  When the pixel counter
    -- reaches the entry's x the accumulator loads and the fetcher advances.
    ------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ldA, v_ldB : std_logic;
        variable v_kr, v_kx : signed(24 downto 0);
        variable v_k  : signed(24 downto 0);
        variable v_lt : boolean;
    begin
        if rising_edge(clk) then
            f_sel  <= not f_sel;
            f_pend <= "00";
            if f_sel = '0' and f_need(0) = '1' then
                lt_ra <= g_line(0) & '0' & fa_ptr;
                f_pend(0) <= '1';
                f_need(0) <= '0';
            elsif f_sel = '1' and f_need(1) = '1' then
                lt_ra <= g_line(0) & '1' & fb_ptr;
                f_pend(1) <= '1';
                f_need(1) <= '0';
            end if;
            f_pend2 <= f_pend;
            if f_pend2(0) = '1' then
                fa_x <= unsigned(lt_q(63 downto 52));
                fa_s <= signed(lt_q(51 downto 32));
                fa_D <= signed(lt_q(31 downto 0));
            end if;
            if f_pend2(1) = '1' then
                fb_x <= unsigned(lt_q(63 downto 52));
                fb_s <= signed(lt_q(51 downto 32));
                fb_D <= signed(lt_q(31 downto 0));
            end if;

            v_ldA := '0'; v_ldB := '0';
            if g_av = '1' then
                if fa_x <= g_x and f_pend2(0) = '0' and f_need(0) = '0' and f_pend(0) = '0' then
                    v_ldA := '1';
                end if;
                if fb_x <= g_x and f_pend2(1) = '0' and f_need(1) = '0' and f_pend(1) = '0' then
                    v_ldB := '1';
                end if;
            end if;
            if g_pre = '1' then
                -- prime: entry 0 of this line's bank is in fa_x/fb_x before
                -- the window starts (g_line already counts this line)
                fa_ptr <= (others => '0');
                fb_ptr <= (others => '0');
                fa_x <= (others => '1');
                fb_x <= (others => '1');
                f_need <= "11";
            else
                if v_ldA = '1' then
                    acc_DA <= fa_D;
                    acc_sA <= shift_left(resize(fa_s, 28), 8);
                    fa_ptr <= fa_ptr + 1;
                    f_need(0) <= '1';
                else
                    acc_DA <= acc_DA + resize(shift_right(acc_sA, 8), 32);
                    acc_sA <= acc_sA + resize(p_qa, 28);
                end if;
                if v_ldB = '1' then
                    acc_DB <= fb_D;
                    acc_sB <= shift_left(resize(fb_s, 28), 8);
                    fb_ptr <= fb_ptr + 1;
                    f_need(1) <= '1';
                else
                    acc_DB <= acc_DB + resize(shift_right(acc_sB, 8), 32);
                    acc_sB <= acc_sB + resize(p_qb, 28);
                end if;
            end if;
            fl_gv(0) <= g_av;
            fl_sw(0) <= g_swp;
            fl_od(0) <= g_x(0);
            for i in 1 to 8 loop fl_gv(i) <= fl_gv(i - 1); end loop;
            for i in 1 to 4 loop fl_sw(i) <= fl_sw(i - 1); end loop;
            for i in 1 to 4 loop fl_od(i) <= fl_od(i - 1); end loop;

            -- stage 1: combine (one comparator: "01" = min, "10" = max)
            v_lt := acc_DA < acc_DB;
            if p_mode(1 downto 0) = "00" or v_lt = (p_mode(0) = '1') then
                s1_D <= acc_DA;
            else
                s1_D <= acc_DB;
            end if;
            -- stage 2: raw key = 128 - D (key units; the rounding half is in the
            -- field constant); REVERSE = 128 + D
            v_kx := sat_s(shift_right(s1_D, 8), 25);
            if p_mode(3) = '1' then
                v_kr := to_signed(128, 25) + v_kx;
            else
                v_kr := to_signed(128, 25) - v_kx;
            end if;
            s2_kr <= v_kr;
            -- stage 3: inner / outer keys (MIX: both = TK)
            if p_mode(2) = '1' then
                s3_ki <= p_tk;
                s3_ko <= p_tk;
            else
                if s2_kr < 0 then s3_ki <= (others => '0');
                elsif s2_kr > 256 then s3_ki <= to_unsigned(256, 9);
                else s3_ki <= unsigned(s2_kr(8 downto 0)); end if;
                v_k := s2_kr + signed(resize(p_dk, 25));
                if v_k < 0 then s3_ko <= (others => '0');
                elsif v_k > 256 then s3_ko <= to_unsigned(256, 9);
                else s3_ko <= unsigned(v_k(8 downto 0)); end if;
            end if;
        end if;
    end process p_pix;

    ------------------------------------------------------------------------
    -- A ring (written raw, read aligned with stage 4) and compose
    ------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            r_wa <= r_wa + 1;
            ra_mem(to_integer(r_wa)) <= data_in.y & data_in.u & data_in.v & data_in.hsync_n & data_in.vsync_n;
            ra_q  <= ra_mem(to_integer(r_wa - RA_OFS));
            ra_q2 <= ra_q;
            fn_d(0) <= data_in.field_n;
            for i in 1 to RA_OFS + 1 loop fn_d(i) <= fn_d(i - 1); end loop;
        end if;
    end process p_ring;

    p_comp : process(clk)
        variable va, vb, vr : t_c3;
        variable vx, vy : t_c3;
        variable vd : t_d3;
        variable vk : unsigned(8 downto 0);
        variable vp : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 4: start point, difference and weight (0..256); chroma
            -- values are held from the even pixel of each 4:2:2 pair
            -- (co-sited chroma).  Until the engine has published a field
            -- the output is A.
            va(0) := unsigned(ra_q2(31 downto 22));
            if fl_sw(3) = '1' then
                va(1) := unsigned(ra_q2(11 downto 2));
                va(2) := unsigned(ra_q2(21 downto 12));
            else
                va(1) := unsigned(ra_q2(21 downto 12));
                va(2) := unsigned(ra_q2(11 downto 2));
            end if;
            vb := (p_bcy, p_bcu, p_bcv);
            vr := (p_rcy, p_rcu, p_rcv);
            if s3_ko < 256 then
                vx := va; vy := vr; vk := s3_ko;
            else
                vx := vr; vy := vb; vk := s3_ki;
            end if;
            if p_ready = '0' then
                vx := va; vk := (others => '0');
            end if;
            for c in 0 to 2 loop
                vd(c) := signed(resize(vy(c), 11)) - signed(resize(vx(c), 11));
            end loop;
            s4_x(0) <= vx(0); s4_d(0) <= vd(0); s4_k <= vk;
            if fl_od(3) = '0' then
                s4_x(1) <= vx(1); s4_x(2) <= vx(2);
                s4_d(1) <= vd(1); s4_d(2) <= vd(2);
                s4_kc <= vk;
                hold_x <= vx; hold_d <= vd; hold_k <= vk;
            else
                s4_x(1) <= hold_x(1); s4_x(2) <= hold_x(2);
                s4_d(1) <= hold_d(1); s4_d(2) <= hold_d(2);
                s4_kc <= hold_k;
            end if;
            -- stage 5
            s5_d <= s4_d; s5_x <= s4_x; s5_k <= s4_k; s5_kc <= s4_kc;
            -- stage 6: products with the 9-bit weight (256 = the far end exactly);
            -- luma with its own weight, chroma with the pair's
            s6_p(0) <= resize(bmul(s5_d(0), signed('0' & s5_k)), 20);
            s6_p(1) <= resize(bmul(s5_d(1), signed('0' & s5_kc)), 20);
            s6_p(2) <= resize(bmul(s5_d(2), signed('0' & s5_kc)), 20);
            s6_x <= s5_x;
            -- stage 7
            for c in 0 to 2 loop
                vp := signed(resize(s6_x(c), 20)) + shift_right(s6_p(c) + 128, 8);
                s7_o(c) <= unsigned(vp(9 downto 0));
            end loop;
            sy_d(0) <= ra_q2(1 downto 0) & fn_d(RA_OFS + 1);
            for i in 1 to 2 loop sy_d(i) <= sy_d(i - 1); end loop;
            -- output register (stage 8): gated outside the window
            if fl_gv(7) = '1' then
                s_out_y <= s7_o(0); s_out_u <= s7_o(1); s_out_v <= s7_o(2);
            else
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= to_unsigned(512, 10);
                s_out_v <= to_unsigned(512, 10);
            end if;
            s_out_s <= fl_gv(7) & sy_d(2);
        end if;
    end process p_comp;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= s_out_s(3);
    data_out.hsync_n <= s_out_s(2);
    data_out.vsync_n <= s_out_s(1);
    data_out.field_n <= s_out_s(0);

    ------------------------------------------------------------------------
    -- Microcoded engine v2 (32-bit).  States: FETCH (after a jump) -> DEC
    -- (latch the instruction; read srcA) -> RDA (latch A with forwarding;
    -- read srcB) -> RDB (latch B and the CTL input) -> EX (one or more
    -- clocks).  Serial units: shifter, Booth multiplier (16 steps + the
    -- result shift), restoring divider (32 steps), square root (24 steps),
    -- quarter-sine ROM.  The engine pauses while the line builder owns the
    -- DATA memory.
    ------------------------------------------------------------------------
    u_fa <= unsigned(u_a(10 downto 0)) when u_st = ST_EX and u_ir(31 downto 27) = "11010" and u_cyc = 0
            else u_pc + 1 when u_st = ST_EX
            else u_pc;
    -- srcA is read during DEC and latched in RDA, srcB read during RDA for
    -- RDB.  While the builder holds the engine (lb_busy / u_pause) in RDA
    -- the address must STAY on srcA: rf_q is refreshed every clock, so a
    -- pause there would hand RDA srcB's value (frame-4 record corruption).
    rf_ra <= ra_of(unsigned(u_rom(19 downto 13)), u_bank) when u_st = ST_DEC
             else ra_of(unsigned(u_ir(19 downto 13)), u_bank) when u_st = ST_RDA and (lb_busy = '1' or u_pause = '1')
             else ra_of(unsigned(u_ir(12 downto 6)), u_bank);

    p_urom : process(clk)
    begin
        if rising_edge(clk) then
            u_rom <= C_UCODE(to_integer(u_fa));
        end if;
    end process p_urom;

    p_rf : process(clk)
    begin
        if rising_edge(clk) then
            if u_we = '1' then rf(to_integer(u_wa)) <= u_wd; end if;
            rf_q <= rf(to_integer(rf_ra));
        end if;
    end process p_rf;

    p_sin : process(clk)
    begin
        if rising_edge(clk) then
            sn_q  <= C_SIN(to_integer(sn_ea));
            sn_q2 <= sn_q;
        end if;
    end process p_sin;

    p_ueng : process(clk)
        variable v_op, v_rop : integer range 0 to 31;
        variable v_r   : signed(31 downto 0);
        variable v_b   : signed(31 downto 0);
        variable v_wr, v_adv, v_jmp, v_lt : boolean;
        variable v_tgt : unsigned(10 downto 0);
        variable v_md, v_ms : signed(33 downto 0);
        variable v_t, v_td, v_b34 : signed(33 downto 0);
        variable v_ang : unsigned(15 downto 0);
        variable v_pm  : unsigned(14 downto 0);
        variable v_sv  : signed(16 downto 0);
        variable v_i6  : integer range 0 to 63;
        variable v_sq, v_sqt, v_sd : signed(28 downto 0);
        variable v_cin : signed(31 downto 0);
        variable v_psel, v_hsel, v_asel : integer range 0 to 7;
        variable v_top, v_lsb, v_qb, v_mneg : std_logic;
    begin
        if rising_edge(clk) then
            u_we    <= '0';
            u_wlast <= u_we;
            dme_we  <= '0';
            u_pause <= lb_busy;
            if s_vs_go = '1' and u_done = '1' then
                -- start the next field's program (the finished set goes live
                -- in p_ports); an unfinished one just keeps running
                u_pc    <= (others => '0');
                u_st    <= ST_FETCH;
                u_done  <= '0';
                u_bank  <= (others => '0');
                u_set   <= not u_set;
            elsif u_done = '0' and lb_busy = '0' and u_pause = '0' then
                v_i6 := to_integer(unsigned(u_ir(5 downto 0)));
                case u_st is
                    when ST_FETCH =>
                        u_st <= ST_DEC;

                    when ST_DEC =>
                        u_ir <= u_rom;
                        v_rop := to_integer(unsigned(u_rom(31 downto 27)));
                        u_f_sh  <= '0'; u_f_shr <= '0'; u_f_shv <= '0';
                        u_f_mul <= '0'; u_f_div <= '0'; u_f_sq <= '0';
                        if v_rop = 4 or v_rop = 5 or v_rop = 22 or v_rop = 23 then u_f_sh <= '1'; end if;
                        if v_rop = 4 or v_rop = 23 then u_f_shr <= '1'; end if;
                        if v_rop = 22 or v_rop = 23 then u_f_shv <= '1'; end if;
                        if v_rop = 8 then u_f_mul <= '1'; end if;
                        if v_rop = 10 then u_f_div <= '1'; end if;
                        if v_rop = 30 then u_f_sq <= '1'; end if;
                        u_f_ser <= '0';
                        if v_rop = 4 or v_rop = 5 or v_rop = 22 or v_rop = 23 or v_rop = 8 or v_rop = 10 or v_rop = 30 then
                            u_f_ser <= '1';
                        end if;
                        u_st <= ST_RDA;

                    when ST_RDA =>
                        if u_wlast = '1' and ra_of(unsigned(u_ir(19 downto 13)), u_bank) = u_wa then
                            u_a <= signed(u_wd);
                        else
                            u_a <= signed(rf_q);
                        end if;
                        u_st <= ST_RDB;

                    when ST_RDB =>
                        if u_wlast = '1' and ra_of(unsigned(u_ir(12 downto 6)), u_bank) = u_wa then
                            u_b <= signed(u_wd);
                        else
                            u_b <= signed(rf_q);
                        end if;
                        case v_i6 is
                            when 0 => u_ctlr <= signed(resize(unsigned(registers_in(0)), 32));
                            when 1 => u_ctlr <= signed(resize(unsigned(registers_in(1)), 32));
                            when 2 => u_ctlr <= signed(resize(unsigned(registers_in(2)), 32));
                            when 3 => u_ctlr <= signed(resize(unsigned(registers_in(3)), 32));
                            when 4 => u_ctlr <= signed(resize(unsigned(registers_in(4)), 32));
                            when 5 => u_ctlr <= signed(resize(unsigned(registers_in(5)), 32));
                            when 6 => u_ctlr <= signed(resize(unsigned(registers_in(6)), 32));
                            when 7 => u_ctlr <= signed(resize(unsigned(registers_in(7)), 32));
                            when 8 => u_ctlr <= signed(resize(unsigned(registers_in(8)(3 downto 0)), 32));
                            when 9 => u_ctlr <= signed(resize(m_w, 32));
                            when 10 => u_ctlr <= signed(resize(m_h, 32));
                            when 11 =>
                                -- the field the program is preparing: the one AFTER the field just started
                                if m_fld = '0' then u_ctlr <= to_signed(1, 32);
                                else u_ctlr <= to_signed(0, 32); end if;
                            when 12 =>
                                if u_set = '1' then u_ctlr <= to_signed(1, 32);
                                else u_ctlr <= to_signed(0, 32); end if;
                            when others => u_ctlr <= (others => '0');
                        end case;
                        u_cyc <= (others => '0');
                        f_c0 <= '1'; f_le15 <= '1'; f_le16 <= '1'; f_le24 <= '1'; f_le48 <= '1'; f_eq25 <= '0';
                        u_st  <= ST_EX;

                    when others =>                          -- EX
                        -- cycle flags for the NEXT cycle (u_cyc + 1); unused when the op advances
                        f_c0 <= '0';
                        if u_cyc < 15 then f_le15 <= '1'; else f_le15 <= '0'; end if;
                        if u_cyc < 16 then f_le16 <= '1'; else f_le16 <= '0'; end if;
                        if u_cyc < 24 then f_le24 <= '1'; else f_le24 <= '0'; end if;
                        if u_cyc < 48 then f_le48 <= '1'; else f_le48 <= '0'; end if;
                        if u_cyc = 24 then f_eq25 <= '1'; else f_eq25 <= '0'; end if;
                        v_op  := to_integer(unsigned(u_ir(31 downto 27)));
                        v_wr  := false;
                        v_adv := true;
                        v_jmp := false;
                        v_r   := (others => '0');
                        v_tgt := unsigned(u_ir(10 downto 0));
                        v_lt  := u_a < u_b;                         -- one comparator for MIN MAX JLT JGE
                        -- serial-unit datapath, one instance of each (selected below)
                        v_psel := 0; v_hsel := 0; v_asel := 0;
                        v_top := '0'; v_lsb := '0'; v_qb := '0';
                        -- Booth addend from the registers alone (no opcode decode in
                        -- front of the adder); zero during the result-shift phase
                        case std_logic_vector'(mu_pl(1) & mu_pl(0) & mu_bp) is
                            when "001" | "010" | "101" | "110" => v_md := resize(mu_a, 34);
                            when "011" | "100"                 => v_md := shift_left(resize(mu_a, 34), 1);
                            when others                        => v_md := (others => '0');
                        end case;
                        v_mneg := mu_pl(1);
                        if mu_shf = '1' then
                            v_md := (others => '0'); v_mneg := '0';
                        end if;
                        if v_mneg = '1' then v_ms := mu_ph - v_md; else v_ms := mu_ph + v_md; end if;
                        -- DIV / SQRT are NON-restoring: the add/sub choice is the REGISTERED
                        -- remainder sign (in front of the adder); the new sign only feeds a
                        -- 1-bit shift-in, so no wide mux follows the carry chain
                        v_t := shift_left(resize(mu_ph(32 downto 0), 34), 1); v_t(0) := mu_pl(31);     -- 2r + next bit
                        v_b34 := signed(resize(unsigned(mu_a), 34));
                        if mu_ph(33) = '0' then v_td := v_t - v_b34;
                        else                    v_td := v_t + v_b34; end if;
                        v_sq := shift_left(resize(mu_ph(26 downto 0), 29), 2);              -- 4r + next pair (29 bits)
                        v_sq(1 downto 0) := signed(mu_pl(31 downto 30));
                        v_sqt := shift_left(signed(resize(unsigned(mu_a(23 downto 0)), 29)), 2);
                        v_sqt(0) := '1';
                        if mu_ph(28) = '0' then v_sd := v_sq - v_sqt;
                        else                    v_sqt(1) := '1'; v_sd := v_sq + v_sqt; end if;
                        case v_op is
                            when 1 =>                                   -- MOV
                                v_r := u_a; v_wr := true;
                            when 2 | 3 | 9 =>                           -- ADD SUB ADDI
                                v_cin := (others => '0');
                                if v_op = 9 then v_b := resize(signed(u_ir(12 downto 0)), 32);
                                elsif v_op = 3 then v_b := not u_b; v_cin(0) := '1';
                                else v_b := u_b; end if;
                                v_r := u_a + v_b + v_cin;
                                v_wr := true;
                            when 6 =>                                   -- AND
                                v_r := u_a and u_b; v_wr := true;
                            when 11 =>                                  -- ANDI
                                v_r := u_a and resize(signed(u_ir(12 downto 0)), 32); v_wr := true;
                            when 7 =>                                   -- LDI
                                v_r := resize(signed(u_ir(19 downto 0)), 32); v_wr := true;
                            when 13 =>                                  -- CTL
                                v_r := u_ctlr; v_wr := true;
                            when 12 =>                                  -- SIN (grid sample)
                                if sn_spec = '1' then v_sv := to_signed(32767, 17);
                                else v_sv := signed('0' & sn_q2); end if;
                                if sn_neg = '1' then v_sv := -v_sv; end if;
                                v_r := resize(v_sv, 32);
                                if u_cyc = 0 then
                                    v_ang := unsigned(u_a(15 downto 6)) & "000000";
                                    if v_ang(14) = '1' then
                                        v_pm := to_unsigned(16384, 15) - resize(v_ang(13 downto 0), 15);
                                    else
                                        v_pm := resize(v_ang(13 downto 0), 15);
                                    end if;
                                    sn_spec <= v_pm(14);
                                    sn_neg  <= v_ang(15);
                                    sn_ea   <= v_pm(13 downto 6);
                                    v_adv := false;
                                elsif u_cyc < 3 then
                                    v_adv := false;
                                else
                                    v_wr := true;
                                end if;
                            when 26 =>                                  -- LDM
                                v_r := signed(u_rom);
                                if u_cyc = 0 then
                                    v_adv := false;
                                else
                                    v_wr := true;
                                end if;
                            when 28 =>                                  -- LDD dst = DATA[A + i6]
                                v_r := signed(dm_q);
                                if u_cyc = 0 then
                                    dme_a <= resize(unsigned(u_a(9 downto 0)) + v_i6, 10);
                                    v_adv := false;
                                elsif u_cyc = 1 then
                                    v_adv := false;
                                else
                                    v_wr := true;
                                end if;
                            when 29 =>                                  -- STD DATA[A + i6] = B
                                dme_a  <= resize(unsigned(u_a(9 downto 0)) + v_i6, 10);
                                dme_wd <= std_logic_vector(u_b);
                                dme_we <= '1';
                            when 24 | 25 =>                             -- MIN MAX
                                if (u_blt = '1') = (v_op = 24) then v_r := u_a; else v_r := u_b; end if;
                                if u_cyc = 0 then
                                    if v_lt then u_blt <= '1'; else u_blt <= '0'; end if;
                                    v_adv := false;
                                else
                                    v_wr := true;
                                end if;
                            when 14 =>                                  -- PRT (WIN only; other ports are DATA words)
                                if v_i6 = 31 then
                                    u_bank <= unsigned(u_a(1 downto 0));
                                end if;
                            when 15 =>                                  -- JMP
                                v_jmp := true;
                            when 16 | 21 =>                             -- JNZ JZ
                                if u_cyc = 0 then
                                    if u_a /= 0 then u_bnz <= '1'; else u_bnz <= '0'; end if;
                                    v_adv := false;
                                elsif (u_bnz = '1') = (v_op = 16) then
                                    v_jmp := true;
                                end if;
                            when 17 | 18 =>                             -- JLT JGE
                                v_tgt := unsigned(u_ir(24 downto 20)) & unsigned(u_ir(5 downto 0));
                                if u_cyc = 0 then
                                    if v_lt then u_blt <= '1'; else u_blt <= '0'; end if;
                                    v_adv := false;
                                elsif (u_blt = '1') = (v_op = 17) then
                                    v_jmp := true;
                                end if;
                            when 19 =>                                  -- JR
                                v_tgt := unsigned(u_a(10 downto 0));
                                v_jmp := true;
                            when 27 =>                                  -- CALL
                                v_r := signed(resize(u_pc + 1, 32));
                                v_wr := true;
                                v_jmp := true;
                            when 20 =>                                  -- END
                                u_done  <= '1';
                                u_ran   <= '1';
                            when others => null;
                        end case;

                        -- serial units: registered op-class and cycle flags keep this cone shallow;
                        -- the result VALUE depends on the class only (cycles gate the write)
                        if u_f_ser = '1' then v_r := signed(mu_pl); end if;
                            if u_f_sh = '1' then                        -- SHR SHL SHRV SHLV (1 bit per clock)
                                if f_c0 = '1' then
                                    v_psel := 1;
                                    if u_f_shv = '1' then u_shc <= '0' & unsigned(u_b(4 downto 0));
                                    else u_shc <= '0' & unsigned(u_ir(4 downto 0)); end if;
                                    v_adv := false;
                                elsif u_shc /= 0 then
                                    if u_f_shr = '1' then v_psel := 2; v_top := mu_pl(31);
                                    else v_psel := 3; end if;
                                    u_shc <= u_shc - 1;
                                    v_adv := false;
                                else
                                    v_wr := true;
                                end if;
                            elsif u_f_mul = '1' then                    -- MUL: 16 Booth steps, then the result
                                                                        -- shift as empty Booth steps (2 bits each)
                                if f_c0 = '1' then
                                    v_psel := 1; v_hsel := 1; v_asel := 1;
                                    mu_bp  <= '0';
                                    mu_shf <= '0';
                                    u_shc <= unsigned(u_ir(5 downto 0));
                                    v_adv := false;
                                elsif f_le16 = '1' then
                                    v_psel := 4; v_hsel := 2;
                                    mu_bp <= mu_pl(1);
                                    if f_le15 = '0' then mu_shf <= '1'; end if;
                                    v_adv := false;
                                elsif u_shc >= 2 then
                                    v_psel := 4; v_hsel := 2;                -- v_md = 0: a pair shift by 2
                                    u_shc <= u_shc - 2;
                                    v_adv := false;
                                elsif u_shc = 1 then
                                    v_psel := 2; v_top := mu_ph(0);          -- odd amount: one more bit
                                    u_shc <= (others => '0');
                                    v_adv := false;
                                else
                                    v_wr := true;
                                end if;
                            elsif u_f_div = '1' then                    -- DIV (A << 16) / B: 48 restoring steps on
                                                                        -- 0:A (the top 16 quotient bits are 0 by contract)
                                if f_c0 = '1' then
                                    v_psel := 1; v_hsel := 1; v_asel := 1;
                                    v_adv := false;
                                elsif f_le48 = '1' then
                                    v_hsel := 3; v_psel := 3; v_lsb := not v_td(33);   -- the new sign IS the quotient bit
                                    v_adv := false;
                                else
                                    v_wr := true;
                                end if;
                            elsif u_f_sq = '1' then                     -- SQRT: floor(sqrt(A << 16)), 24 steps
                                if f_c0 = '1' then
                                    v_psel := 1; v_hsel := 1; v_asel := 2;
                                    v_adv := false;
                                elsif f_le24 = '1' then
                                    v_hsel := 5; v_qb := not v_sd(28);
                                    v_psel := 5; v_asel := 3;
                                    v_adv := false;
                                elsif f_eq25 = '1' then
                                    v_psel := 6;                              -- deliver the root in mu_pl
                                    v_adv := false;
                                else
                                    v_wr := true;
                                end if;
                            end if;

                        case v_psel is
                            when 1 => mu_pl <= unsigned(u_a);
                            when 2 => mu_pl <= v_top & mu_pl(31 downto 1);
                            when 3 => mu_pl <= mu_pl(30 downto 0) & v_lsb;
                            when 4 => mu_pl <= unsigned(v_ms(1 downto 0)) & mu_pl(31 downto 2);
                            when 5 => mu_pl <= mu_pl(29 downto 0) & "00";
                            when 6 => mu_pl <= unsigned(mu_a);
                            when others => null;
                        end case;
                        case v_hsel is
                            when 1 => mu_ph <= (others => '0');
                            when 2 => mu_ph <= shift_right(v_ms, 2);
                            when 3 => mu_ph <= v_td;
                            when 5 => mu_ph <= resize(v_sd, 34);
                            when others => null;
                        end case;
                        case v_asel is
                            when 1 => mu_a <= u_b;
                            when 2 => mu_a <= (others => '0');
                            when 3 => mu_a <= mu_a(30 downto 0) & v_qb;
                            when others => null;
                        end case;

                        -- the write data / address are formed every EX cycle (shallow enables);
                        -- only u_we carries the deep "result ready" decision
                        if v_wr then u_we <= '1'; end if;
                        u_wd <= std_logic_vector(v_r);
                        if v_op = 27 then u_wa <= to_unsigned(95, 8);
                        else u_wa <= ra_of(unsigned(u_ir(26 downto 20)), u_bank); end if;
                        if v_jmp then
                            u_pc <= v_tgt;
                            u_st <= ST_FETCH;
                        elsif v_adv then
                            u_pc <= u_pc + 1;
                            u_st <= ST_DEC;
                        else
                            u_cyc <= u_cyc + 1;
                        end if;
                end case;
            end if;
        end if;
    end process p_ueng;

end architecture starwipe;
