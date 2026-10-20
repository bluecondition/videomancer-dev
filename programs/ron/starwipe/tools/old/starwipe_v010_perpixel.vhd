-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- starwipe.vhd  (v0.1.0 -- core: field engine, T-bar mapping, EDGE, B = black /
-- matte / white; MIX, BAR, BOX, CIRCLE, STAR)
--
-- STARWIPE: the wipe / pattern section of a production switcher.  The input
-- video (A) wipes to a generated B source; the P12 slider is the T-bar.
--
-- Every wipe is an ANALYTIC distance field evaluated per pixel -- no bitmaps,
-- no masks -- so edges are equally crisp at 480i and 1080p:
--
--  1. RASTER DDA.  u, v = the pixel's rotated, anchored position in u-units
--     (1/16 of a horizontal pixel; SD lines are 11/10 or 11/12 of that, from
--     the video timing id, so circles are round on every monitor).  Two
--     accumulator pairs (pixel, line), increments from the engine.
--  2. FIELD.  linear (bars u; box max(|u|-a, |v|-b)) or polar from a LOG-DOMAIN
--     engine (hypnos lineage, no CORDIC): log2 max/min |u|,|v| -> log2 r and
--     theta from interpolated tables; stars and polygons get EXACT straight
--     edges as r cos(fold_n(theta) - alpha) (-log2 cos from a table), back to
--     linear distance through an interpolated exp2.
--  3. KEY.  sd = D - T (REVERSE: T - D); key = 1/2 - sd / w with w >= one
--     pixel (the filter gain is a mantissa x shift); the border is the same
--     ramp offset by its width.  Both keys 0..256.
--  4. COMPOSE.  A -> border -> B with one lerp per channel.
--
-- The per-frame work runs on a small microcoded engine (cubist / damier
-- lineage, 24-bit; program tools/sw_ucode.py) from the frame anchor.  To map
-- the T-bar so both ends of travel are EXACTLY A / EXACTLY B under any
-- pattern, origin, spin or edge setting, it WALKS the frame perimeter through
-- this very pixel pipeline in vertical blanking and reads back the field's
-- min / max (the trackers sit at the merge stage).
--
-- Hardware contracts: program-generated avid at a fixed offset from hsync
-- (parity-stable for the encoders; A's chroma pair is swapped on lines where
-- the source's hsync->avid parity differs, drape v1.3); outputs gated to
-- neutral outside it; one latency (52) in every mode; A rides an EBR ring,
-- never a deep FF pipe; frame anchor = first vsync edge after active video.
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
    constant C_PROCESSING_DELAY_CLKS : natural := 56;

    -- Stage k (sk_*) holds pixel p during clock c0 + p + k + 1, where c0 is
    -- the window's first clock.  The window starts G_K + 2 = 8 clocks after
    -- the A sample that is its pixel 0 (a source avid up to G_K - 1 clocks
    -- late still counts), and the output register is stage 47, so the latency
    -- is 8 + 48 = 56.
    constant G_K      : natural := 6;
    -- ring read offsets (write address minus; each read = EBR register +
    -- fabric register): A + syncs line up with S42, the linear field + flags
    -- (written from S3) with S35
    constant RA_OFS   : natural := 49;
    constant RD_OFS   : natural := 30;
    constant WK_FLUSH : natural := 40;        -- walk point -> stats tracker depth

    constant D_MAX  : integer := 2**19 - 1;
    constant SD_MAX : integer := 2**20 - 1;
    constant KR_MAX : integer := 2**17 - 1;
    constant LC_CAP : integer := 2 * 2**17 - 1;

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

    -- Exact unsigned product (width len(a) + len(m)).
    function bmul_u(a : unsigned; m : unsigned) return unsigned is
        constant NP : natural := a'length + m'length;
        variable vp : signed(NP + 1 downto 0);
    begin
        vp := bmul(signed('0' & a), signed('0' & m));
        return unsigned(vp(NP - 1 downto 0));
    end function;

    -- index of the highest set bit of a 17-bit value (x >= 1)
    function msb17(x : unsigned(16 downto 0)) return unsigned is
        variable m : integer range 0 to 16;
    begin
        m := 0;
        for i in 0 to 16 loop
            if x(i) = '1' then m := i; end if;
        end loop;
        return to_unsigned(m, 5);
    end function;

    function sat_s(x : signed; w : natural) return signed is
    begin
        if x > to_signed(2**(w - 1) - 1, x'length) then
            return to_signed(2**(w - 1) - 1, w);
        elsif x < to_signed(-2**(w - 1), x'length) then
            return to_signed(-2**(w - 1), w);
        end if;
        return resize(x, w);
    end function;

    ------------------------------------------------------------------------
    -- raster measurement, anchor, generated window
    ------------------------------------------------------------------------
    signal m_hsq, m_avq, m_vsq : std_logic := '1';
    signal m_hc   : unsigned(11 downto 0) := (others => '0');   -- clocks since hsync
    signal m_la   : std_logic := '0';                            -- line has source video
    signal m_offl : unsigned(11 downto 0) := (others => '0');   -- this line's avid offset
    signal m_off  : unsigned(11 downto 0) := to_unsigned(200, 12);
    signal m_offk : unsigned(11 downto 0) := to_unsigned(205, 12);
    signal m_avc  : unsigned(11 downto 0) := (others => '0');
    signal m_wl   : unsigned(11 downto 0) := (others => '0');
    signal m_w    : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal m_lc   : unsigned(10 downto 0) := (others => '0');
    signal m_h    : unsigned(10 downto 0) := to_unsigned(243, 11);
    signal m_saw  : std_logic := '0';
    signal m_fld  : std_logic := '1';
    signal s_vs_go : std_logic := '0';

    signal g_go   : std_logic := '0';
    signal g_av   : std_logic := '0';
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_swp  : std_logic := '0';    -- this line: source chroma pairing differs

    ------------------------------------------------------------------------
    -- engine ports (per frame)
    ------------------------------------------------------------------------
    signal p_dux, p_dvx, p_duy, p_dvy : signed(23 downto 0) := (others => '0');
    signal p_flags : std_logic_vector(4 downto 0) := (others => '0');
    signal p_t     : signed(20 downto 0) := (others => '0');
    signal p_gm    : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal p_gs    : unsigned(4 downto 0) := to_unsigned(4, 5);
    signal p_dk    : unsigned(17 downto 0) := (others => '0');
    signal p_tk    : unsigned(8 downto 0) := (others => '0');
    signal p_ba, p_bb : signed(19 downto 0) := (others => '0');
    signal p_sn    : unsigned(3 downto 0) := to_unsigned(5, 4);
    signal p_ss    : unsigned(19 downto 0) := (others => '0');
    signal p_sal   : unsigned(17 downto 0) := (others => '0');
    signal p_bcy, p_bcu, p_bcv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p_rcy, p_rcu, p_rcv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p_psel  : unsigned(3 downto 0);
    signal p_rev   : std_logic;
    signal p_pol   : std_logic;      -- polar field (circle / star)
    signal p_star  : std_logic;
    signal p_mix   : std_logic;

    ------------------------------------------------------------------------
    -- raster DDA + perimeter walker
    ------------------------------------------------------------------------
    signal d_ul, d_vl, d_u, d_v : signed(31 downto 0) := (others => '0');
    type t_wk is (WK_IDLE, WK_LOAD, WK_RUN);
    signal wk_st   : t_wk := WK_IDLE;
    signal wk_cnt  : unsigned(11 downto 0) := (others => '0');
    signal wk_fl   : unsigned(5 downto 0) := (others => '0');
    signal wk_step : std_logic;
    signal wk_busy : std_logic;
    signal st_min, st_max : signed(19 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- delay rings (EBR): A video + syncs, and the linear field
    ------------------------------------------------------------------------
    type t_r32 is array (0 to 255) of std_logic_vector(31 downto 0);
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal ra_mem : t_r32;
    signal rb_mem : t_r16;
    signal rd_mem : t_r32;
    attribute no_rw_check : boolean;
    attribute no_rw_check of ra_mem : signal is true;
    attribute no_rw_check of rb_mem : signal is true;
    attribute no_rw_check of rd_mem : signal is true;
    signal r_wa : unsigned(7 downto 0) := (others => '0');
    signal ra_q, ra_q2 : std_logic_vector(31 downto 0) := (others => '0');
    signal rb_q, rb_q2 : std_logic_vector(15 downto 0) := (others => '0');
    signal rd_q, rd_q2 : std_logic_vector(31 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- table ROM outputs
    ------------------------------------------------------------------------
    signal lq_h, lq_l, ug_q, ua_q, lc_q, ex_q : std_logic_vector(31 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- pixel pipeline (stage N = N registers after the DDA)
    ------------------------------------------------------------------------
    signal s0_u, s0_v : signed(17 downto 0) := (others => '0');
    signal s0_gv, s0_wv, s0_sw : std_logic := '0';
    signal s1_au, s1_av : unsigned(16 downto 0) := (others => '0');
    signal s1_u : signed(17 downto 0) := (others => '0');
    signal s1_su, s1_sv, s1_gv, s1_wv, s1_sw : std_logic := '0';
    signal s2_hi, s2_lo, s2_au, s2_av : unsigned(16 downto 0) := (others => '0');
    signal s2_bu, s2_bv : signed(20 downto 0) := (others => '0');
    signal s2_u : signed(17 downto 0) := (others => '0');
    signal s2_q : std_logic_vector(3 downto 0) := (others => '0');   -- lz sw su sv
    signal s2_gv, s2_wv, s2_sw : std_logic := '0';
    signal s3_hi, s3_lo : unsigned(16 downto 0) := (others => '0');
    signal s3_mh, s3_ml : unsigned(4 downto 0) := (others => '0');
    signal s3_dl : signed(19 downto 0) := (others => '0');
    signal s3_gv, s3_wv, s3_sw : std_logic := '0';
    signal s4_ih, s4_fh, s4_il, s4_fl : unsigned(7 downto 0) := (others => '0');
    signal s4_mh, s4_ml, s5_mh, s5_ml, s6_mh, s6_ml, s7_mh, s7_ml : unsigned(4 downto 0) := (others => '0');
    signal s5_fh, s5_fl, s6_fh, s6_fl : unsigned(7 downto 0) := (others => '0');
    signal s6_lh, s6_ll : std_logic_vector(31 downto 0) := (others => '0');
    signal s7_ph, s7_pl : unsigned(18 downto 0) := (others => '0');
    signal s7_vh, s7_vl : unsigned(17 downto 0) := (others => '0');
    signal s8_lh, s8_ll : unsigned(22 downto 0) := (others => '0');
    signal s9_du : unsigned(21 downto 0) := (others => '0');
    signal s10_ia : unsigned(7 downto 0) := (others => '0');
    signal s10_fa, s11_fa, s12_fa : unsigned(11 downto 0) := (others => '0');
    signal s12_ug, s12_ua : std_logic_vector(31 downto 0) := (others => '0');
    signal s13_pgh, s13_pgl : unsigned(19 downto 0) := (others => '0');
    signal s13_pah, s13_pal : unsigned(17 downto 0) := (others => '0');
    signal s13_g, s13_a, s14_g, s14_a : unsigned(17 downto 0) := (others => '0');
    signal s14_pg : unsigned(25 downto 0) := (others => '0');
    signal s14_pa : unsigned(23 downto 0) := (others => '0');
    signal s15_g, s15_a : unsigned(17 downto 0) := (others => '0');
    type t_lh is array (9 to 15) of unsigned(22 downto 0);
    signal lh_d : t_lh := (others => (others => '0'));
    type t_q is array (3 to 15) of std_logic_vector(3 downto 0);
    signal q_d : t_q := (others => (others => '0'));
    signal s16_lr : unsigned(22 downto 0) := (others => '0');
    signal s16_th : unsigned(19 downto 0) := (others => '0');
    type t_lr is array (17 to 27) of unsigned(22 downto 0);
    signal lr_d : t_lr := (others => (others => '0'));
    signal s17_k : unsigned(3 downto 0) := (others => '0');
    signal s17_th, s18_th : unsigned(19 downto 0) := (others => '0');
    signal s18_ks : unsigned(23 downto 0) := (others => '0');
    signal s19_ph, s20_ph : unsigned(19 downto 0) := (others => '0');
    signal s21_ix : unsigned(7 downto 0) := (others => '0');
    signal s21_fx, s22_fx, s23_fx : unsigned(9 downto 0) := (others => '0');
    signal s23_lc : std_logic_vector(31 downto 0) := (others => '0');
    signal s24_ph, s24_pl : unsigned(17 downto 0) := (others => '0');
    signal s24_v, s25_v : unsigned(17 downto 0) := (others => '0');
    signal s25_p : unsigned(22 downto 0) := (others => '0');
    signal s26_lc : unsigned(17 downto 0) := (others => '0');
    signal s27_l : unsigned(23 downto 0) := (others => '0');
    signal s28_ie, s29_ie, s30_ie, s31_ie, s32_ie, s33_ie : unsigned(5 downto 0) := (others => '0');
    signal s28_f, s29_f, s30_f : unsigned(9 downto 0) := (others => '0');
    signal s28_i : unsigned(7 downto 0) := (others => '0');
    signal s30_ex : std_logic_vector(31 downto 0) := (others => '0');
    signal s31_ph, s31_pl : unsigned(15 downto 0) := (others => '0');
    signal s31_v, s32_v : unsigned(17 downto 0) := (others => '0');
    signal s32_p : unsigned(20 downto 0) := (others => '0');
    signal s33_m : unsigned(18 downto 0) := (others => '0');
    signal s34_mr : unsigned(19 downto 0) := (others => '0');
    signal s34_sh : unsigned(4 downto 0) := (others => '0');
    signal s34_ov : std_logic := '0';
    signal s35_dp : unsigned(19 downto 0) := (others => '0');
    -- linear field + flags from the D ring land at S35's input
    signal s36_d : signed(19 downto 0) := (others => '0');
    signal s37_sd : signed(20 downto 0) := (others => '0');
    signal s38_ph : signed(26 downto 0) := (others => '0');
    signal s38_pl : signed(25 downto 0) := (others => '0');
    signal s39_pr : signed(29 downto 0) := (others => '0');
    signal s40_sc : signed(29 downto 0) := (others => '0');
    signal s41_kr : signed(18 downto 0) := (others => '0');
    signal s42_ki, s42_ko : unsigned(8 downto 0) := (others => '0');
    type t_c3 is array (0 to 2) of unsigned(9 downto 0);
    signal s43_x, s43_y : t_c3 := (others => (others => '0'));
    signal s43_k : unsigned(8 downto 0) := (others => '0');
    type t_d3 is array (0 to 2) of signed(10 downto 0);
    signal s44_d : t_d3 := (others => (others => '0'));
    signal s44_x, s44_y, s45_y : t_c3 := (others => (others => '0'));
    signal s44_k : unsigned(8 downto 0) := (others => '0');
    type t_p3 is array (0 to 2) of signed(19 downto 0);
    signal s45_p : t_p3 := (others => (others => '0'));
    signal s45_x : t_c3 := (others => (others => '0'));
    signal s45_full : std_logic := '0';
    type t_o3 is array (0 to 2) of unsigned(9 downto 0);
    signal s46_o : t_o3 := (others => (others => '0'));
    -- window / walk / swap flags from the D ring (index = aligned stage)
    signal fl_gv : std_logic_vector(36 to 46) := (others => '0');
    signal fl_wv : std_logic := '0';                        -- aligned with S36
    signal fl_sw : std_logic_vector(36 to 42) := (others => '0');
    -- syncs from the A ring, aligned with S43..S46
    type t_sy is array (0 to 3) of std_logic_vector(2 downto 0);
    signal sy_d : t_sy := (others => (others => '1'));
    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');
    signal s_out_s : std_logic_vector(3 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- microcoded engine
    ------------------------------------------------------------------------
    constant ST_FETCH : unsigned(2 downto 0) := "000";
    constant ST_DEC   : unsigned(2 downto 0) := "001";
    constant ST_RDA   : unsigned(2 downto 0) := "010";
    constant ST_RDB   : unsigned(2 downto 0) := "011";
    constant ST_EX    : unsigned(2 downto 0) := "100";
    signal u_st   : unsigned(2 downto 0) := ST_FETCH;
    signal u_pc   : unsigned(8 downto 0) := (others => '0');
    signal u_fa   : unsigned(8 downto 0);
    signal u_rom  : std_logic_vector(31 downto 0) := (others => '0');
    signal u_ir   : std_logic_vector(31 downto 0) := (others => '0');
    signal u_done : std_logic := '1';
    signal u_cyc  : unsigned(5 downto 0) := (others => '0');
    type t_rf is array (0 to 127) of std_logic_vector(23 downto 0);
    signal rf     : t_rf := (others => (others => '0'));
    attribute no_rw_check of rf : signal is true;
    signal rf_ra  : unsigned(6 downto 0);
    signal rf_q   : std_logic_vector(23 downto 0) := (others => '0');
    signal u_we, u_wlast : std_logic := '0';
    signal u_wa   : unsigned(6 downto 0) := (others => '0');
    signal u_wd   : std_logic_vector(23 downto 0) := (others => '0');
    signal u_a, u_b : signed(23 downto 0) := (others => '0');
    signal u_blt, u_bnz : std_logic := '0';
    signal u_ctlr : signed(23 downto 0) := (others => '0');
    signal u_sh   : signed(23 downto 0) := (others => '0');
    signal u_shc  : unsigned(5 downto 0) := (others => '0');
    signal mu_a   : signed(23 downto 0) := (others => '0');
    signal mu_ph  : signed(25 downto 0) := (others => '0');
    signal mu_pl  : unsigned(23 downto 0) := (others => '0');
    signal mu_bp  : std_logic := '0';
    signal dv_r   : unsigned(24 downto 0) := (others => '0');
    signal dv_q   : unsigned(23 downto 0) := (others => '0');
    signal dv_b   : unsigned(23 downto 0) := (others => '0');
    signal sn_ea  : unsigned(7 downto 0) := (others => '0');
    signal sn_spec, sn_neg : std_logic := '0';
    signal sn_q, sn_q2 : std_logic_vector(15 downto 0) := (others => '0');
    signal u_prt  : std_logic := '0';
    signal u_prp  : unsigned(5 downto 0) := (others => '0');
    signal u_prv  : std_logic_vector(23 downto 0) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- Raster measurement and the frame anchor.  hc counts clocks since the
    -- hsync fall; a line is active once the source avid rises on it.  The
    -- source's avid offset and width are latched per field with 2-clock
    -- hysteresis (analog lines wander a clock), so the generated window sits
    -- at ONE hsync offset: geometry stays straight and the 4:2:2 parity is
    -- constant.  Frame anchor = first vsync fall after active video.
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
                if m_offl > m_off then v_d := m_offl - m_off; else v_d := m_off - m_offl; end if;
                if v_d >= 2 then m_off <= m_offl; end if;
                if m_wl > m_w then v_d := m_wl - m_w; else v_d := m_w - m_wl; end if;
                if v_d >= 2 and m_wl > 15 then m_w <= m_wl; end if;
                if m_lc > 15 then m_h <= m_lc; end if;
                m_lc <= (others => '0');
            end if;
            m_offk <= m_off + G_K;
        end if;
    end process p_meas;

    ------------------------------------------------------------------------
    -- Generated window: g_go one clock before pixel 0 (the DDA loads), then
    -- exactly m_w pixels.  Pixel 0 is the A sample at hc = m_off.  g_swp: the
    -- source paired this line's chroma from a different parity than ours.
    ------------------------------------------------------------------------
    p_gen : process(clk)
    begin
        if rising_edge(clk) then
            g_go <= '0';
            if m_hc = m_offk and m_la = '1' then
                g_go  <= '1';
                g_swp <= m_offl(0) xor m_off(0);
            end if;
            if g_go = '1' then
                g_av  <= '1';
                g_cnt <= m_w - 1;
            elsif g_av = '1' then
                if g_cnt = 0 or (m_hsq = '1' and data_in.hsync_n = '0') then
                    g_av <= '0';
                else
                    g_cnt <= g_cnt - 1;
                end if;
            end if;
        end if;
    end process p_gen;

    ------------------------------------------------------------------------
    -- Raster DDA (accumulators = u-units << 14).  The engine loads the field's
    -- first-line seed straight into the line accumulators; each active line
    -- start copies them into the pixel accumulators and steps them.  The
    -- walker reuses the pixel accumulators in vertical blanking: LOAD copies
    -- the seed, then n steps of the (temporarily walk-sized) DUX/DVX.
    ------------------------------------------------------------------------
    wk_step <= '1' when wk_st = WK_RUN and wk_cnt /= 0 else '0';
    wk_busy <= '1' when wk_st /= WK_IDLE or wk_fl /= 0 else '0';

    p_dda : process(clk)
    begin
        if rising_edge(clk) then
            if u_prt = '1' and u_prp = 4 then
                d_ul <= shift_left(resize(signed(u_prv), 32), 8);
            elsif g_go = '1' then
                d_ul <= d_ul + resize(p_duy, 32);
            end if;
            if u_prt = '1' and u_prp = 5 then
                d_vl <= shift_left(resize(signed(u_prv), 32), 8);
            elsif g_go = '1' then
                d_vl <= d_vl + resize(p_dvy, 32);
            end if;
            if wk_st = WK_LOAD or g_go = '1' then
                d_u <= d_ul;
                d_v <= d_vl;
            elsif g_av = '1' or wk_step = '1' then
                d_u <= d_u + resize(p_dux, 32);
                d_v <= d_v + resize(p_dvx, 32);
            end if;

            -- walker
            case wk_st is
                when WK_IDLE => null;
                when WK_LOAD => wk_st <= WK_RUN;
                when WK_RUN =>
                    if wk_cnt = 0 then
                        wk_st <= WK_IDLE;
                        wk_fl <= to_unsigned(WK_FLUSH, 6);
                    else
                        wk_cnt <= wk_cnt - 1;
                    end if;
            end case;
            if wk_fl /= 0 and wk_st = WK_IDLE then
                wk_fl <= wk_fl - 1;
            end if;
            if u_prt = '1' and u_prp = 23 then
                wk_cnt <= unsigned(u_prv(11 downto 0));
                if u_prv(23) = '1' then wk_st <= WK_LOAD; else wk_st <= WK_RUN; end if;
            end if;
        end if;
    end process p_dda;

    ------------------------------------------------------------------------
    -- Delay rings.  A (+ syncs) is written raw from data_in and read so it
    -- is aligned with S42 (S43 composes); the linear field (+ window / walk /
    -- swap flags) is written from S3 and aligned with S35.  One free-running
    -- write address.
    ------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            r_wa <= r_wa + 1;
            ra_mem(to_integer(r_wa)) <= data_in.y & data_in.u & data_in.v & data_in.hsync_n & data_in.vsync_n;
            ra_q  <= ra_mem(to_integer(r_wa - RA_OFS));
            ra_q2 <= ra_q;
            rb_mem(to_integer(r_wa)) <= x"000" & "000" & data_in.field_n;
            rb_q  <= rb_mem(to_integer(r_wa - RA_OFS));
            rb_q2 <= rb_q;
            rd_mem(to_integer(r_wa)) <= "000000000" & s3_sw & s3_wv & s3_gv & std_logic_vector(s3_dl);
            rd_q  <= rd_mem(to_integer(r_wa - RD_OFS));
            rd_q2 <= rd_q;
        end if;
    end process p_ring;

    -- table ROMs: every address is a register, every read is registered
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            lq_h <= C_LOG(to_integer(s4_ih));
            lq_l <= C_LOG(to_integer(s4_il));
            ug_q <= C_UG(to_integer(s10_ia));
            ua_q <= C_UA(to_integer(s10_ia));
            lc_q <= C_LCOS(to_integer(s21_ix));
            ex_q <= C_EXP(to_integer(s28_i));
        end if;
    end process p_rom;

    p_psel <= unsigned(p_flags(3 downto 0));
    p_rev  <= p_flags(4);
    p_pol  <= p_flags(3);
    p_star <= '1' when p_flags(3 downto 0) = "1001" else '0';
    p_mix  <= '1' when p_flags(3 downto 0) = "0000" else '0';

    ------------------------------------------------------------------------
    -- S0..S8: coordinates, |u| |v|, linear fields, log2 max / min  (Q18)
    ------------------------------------------------------------------------
    p_s0 : process(clk)
        variable vu, vv : signed(18 downto 0);
        variable vh, vl : unsigned(16 downto 0);
        variable vn : unsigned(16 downto 0);
        variable vd : signed(20 downto 0);
    begin
        if rising_edge(clk) then
            -- S0: integer u-units (floor)
            s0_u  <= d_u(31 downto 14);
            s0_v  <= d_v(31 downto 14);
            s0_gv <= g_av;
            s0_wv <= wk_step;
            s0_sw <= g_swp;
            -- S1: magnitudes (saturated to 17 bits)
            vu := abs(resize(s0_u, 19));
            vv := abs(resize(s0_v, 19));
            if vu > 131071 then s1_au <= (others => '1'); else s1_au <= unsigned(vu(16 downto 0)); end if;
            if vv > 131071 then s1_av <= (others => '1'); else s1_av <= unsigned(vv(16 downto 0)); end if;
            s1_su <= s0_u(17);
            s1_sv <= s0_v(17);
            s1_u  <= s0_u;
            s1_gv <= s0_gv; s1_wv <= s0_wv; s1_sw <= s0_sw;
            -- S2: hi / lo, box edge distances
            if s1_av > s1_au then
                vh := s1_av; vl := s1_au; s2_q(2) <= '1';
            else
                vh := s1_au; vl := s1_av; s2_q(2) <= '0';
            end if;
            if vh = 0 then vh := to_unsigned(1, 17); end if;
            s2_hi <= vh;
            s2_lo <= vl;
            if vl = 0 then s2_q(3) <= '1'; else s2_q(3) <= '0'; end if;
            s2_q(1) <= s1_su;
            s2_q(0) <= s1_sv;
            s2_bu <= resize(signed('0' & s1_au), 21) - resize(p_ba, 21);
            s2_bv <= resize(signed('0' & s1_av), 21) - resize(p_bb, 21);
            s2_u  <= s1_u;
            s2_au <= s1_au; s2_av <= s1_av;
            s2_gv <= s1_gv; s2_wv <= s1_wv; s2_sw <= s1_sw;
            -- S3: msb, linear field
            vl := s2_lo;
            if vl = 0 then vl := to_unsigned(1, 17); end if;
            s3_hi <= s2_hi;
            s3_lo <= vl;
            s3_mh <= msb17(s2_hi);
            s3_ml <= msb17(vl);
            case to_integer(p_psel) is
                when 1 => s3_dl <= resize(s2_u, 20);
                when 2 =>
                    if s2_bu > s2_bv then vd := s2_bu; else vd := s2_bv; end if;
                    s3_dl <= sat_s(vd, 20);
                when 3 => s3_dl <= signed(resize(s2_au, 20));
                when 4 => s3_dl <= signed(resize(s2_av, 20));
                when others => s3_dl <= (others => '0');
            end case;
            s3_gv <= s2_gv; s3_wv <= s2_wv; s3_sw <= s2_sw;
            -- S4: normalise -> 8-bit segment index + 8-bit fraction
            vn := shift_left(s3_hi, 16 - to_integer(s3_mh));
            s4_ih <= vn(15 downto 8); s4_fh <= vn(7 downto 0);
            vn := shift_left(s3_lo, 16 - to_integer(s3_ml));
            s4_il <= vn(15 downto 8); s4_fl <= vn(7 downto 0);
            s4_mh <= s3_mh; s4_ml <= s3_ml;
            -- S5 (EBR), S6 (fabric)
            s5_fh <= s4_fh; s5_fl <= s4_fl; s5_mh <= s4_mh; s5_ml <= s4_ml;
            s6_fh <= s5_fh; s6_fl <= s5_fl; s6_mh <= s5_mh; s6_ml <= s5_ml;
            s6_lh <= lq_h; s6_ll <= lq_l;
            -- S7: slope x fraction
            s7_ph <= bmul_u(unsigned(s6_lh(10 downto 0)), s6_fh);
            s7_pl <= bmul_u(unsigned(s6_ll(10 downto 0)), s6_fl);
            s7_vh <= unsigned(s6_lh(28 downto 11));
            s7_vl <= unsigned(s6_ll(28 downto 11));
            s7_mh <= s6_mh; s7_ml <= s6_ml;
            -- S8: log2 = msb << 18 + PWL (Q18)
            s8_lh <= shift_left(resize(s7_mh, 23), 18) + resize(s7_vh, 23) + resize(shift_right(s7_ph + 128, 8), 23);
            s8_ll <= shift_left(resize(s7_ml, 23), 18) + resize(s7_vl, 23) + resize(shift_right(s7_pl + 128, 8), 23);
        end if;
    end process p_s0;

    ------------------------------------------------------------------------
    -- S9..S16: u = log2(hi/lo) -> log2 r = log2 hi + 0.5 log2(1+4^-u) and the
    -- octant angle atan(2^-u) (1/16-octave tables, 12-bit interpolation),
    -- then the full angle (2^20 per turn, clockwise on screen).
    ------------------------------------------------------------------------
    p_s9 : process(clk)
        variable vdu : unsigned(22 downto 0);
        variable vph : unsigned(19 downto 0);
        variable vq  : std_logic_vector(3 downto 0);
    begin
        if rising_edge(clk) then
            -- quadrant flags and log2 hi ride along
            q_d(3) <= s2_q;
            for i in 4 to 15 loop q_d(i) <= q_d(i - 1); end loop;
            lh_d(9) <= s8_lh;
            for i in 10 to 15 loop lh_d(i) <= lh_d(i - 1); end loop;
            -- S9
            vdu := s8_lh - s8_ll;
            if q_d(8)(3) = '1' or vdu > 4194303 then
                s9_du <= to_unsigned(4194303, 22);
            else
                s9_du <= vdu(21 downto 0);
            end if;
            -- S10
            s10_ia <= s9_du(21 downto 14);
            s10_fa <= s9_du(13 downto 2);
            -- S11 (EBR), S12 (fabric)
            s11_fa <= s10_fa;
            s12_fa <= s11_fa;
            s12_ug <= ug_q; s12_ua <= ua_q;
            -- S13: slope x fraction, split on the fraction
            s13_pgh <= bmul_u(unsigned(s12_ug(13 downto 0)), s12_fa(11 downto 6));
            s13_pgl <= bmul_u(unsigned(s12_ug(13 downto 0)), s12_fa(5 downto 0));
            s13_pah <= bmul_u(unsigned(s12_ua(11 downto 0)), s12_fa(11 downto 6));
            s13_pal <= bmul_u(unsigned(s12_ua(11 downto 0)), s12_fa(5 downto 0));
            s13_g <= unsigned(s12_ug(31 downto 14));
            s13_a <= unsigned(s12_ua(29 downto 12));
            -- S14
            s14_pg <= shift_left(resize(s13_pgh, 26), 6) + resize(s13_pgl, 26);
            s14_pa <= shift_left(resize(s13_pah, 24), 6) + resize(s13_pal, 24);
            s14_g <= s13_g; s14_a <= s13_a;
            -- S15
            s15_g <= s14_g - resize(shift_right(s14_pg + 2048, 12), 18);
            s15_a <= s14_a - resize(shift_right(s14_pa + 2048, 12), 18);
            -- S16: log2 r ; full angle
            s16_lr <= lh_d(15) + resize(s15_g, 23);
            vq := q_d(15);
            if vq(2) = '1' then vph := to_unsigned(262144, 20) - resize(s15_a, 20);
            else                vph := resize(s15_a, 20); end if;
            if    vq(1) = '0' and vq(0) = '0' then s16_th <= vph;
            elsif vq(1) = '1' and vq(0) = '0' then s16_th <= to_unsigned(524288, 20) - vph;
            elsif vq(1) = '1' and vq(0) = '1' then s16_th <= to_unsigned(524288, 20) + vph;
            else                                   s16_th <= to_unsigned(0, 20) - vph; end if;
        end if;
    end process p_s9;

    ------------------------------------------------------------------------
    -- S17..S26: star / polygon fold.  k = sector of n; ph = theta - k S,
    -- mirrored about the sector bisector; x = |ph - alpha|; -log2 cos x from
    -- the table (Q17, capped at 2 octaves).
    ------------------------------------------------------------------------
    p_s17 : process(clk)
        variable vt  : unsigned(23 downto 0);
        variable vp  : signed(21 downto 0);
        variable vx  : signed(20 downto 0);
        variable vxa : unsigned(20 downto 0);
        variable vlc : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            lr_d(17) <= s16_lr;
            for i in 18 to 27 loop lr_d(i) <= lr_d(i - 1); end loop;
            -- S17
            vt := bmul_u(s16_th, p_sn);
            s17_k  <= vt(23 downto 20);
            s17_th <= s16_th;
            -- S18
            s18_ks <= bmul_u(p_ss, s17_k);
            s18_th <= s17_th;
            -- S19: clamp to [0, S]
            vp := signed(resize(s18_th, 22)) - signed(resize(s18_ks, 22));
            if vp < 0 then
                s19_ph <= (others => '0');
            elsif vp > signed(resize(p_ss, 22)) then
                s19_ph <= p_ss;
            else
                s19_ph <= unsigned(vp(19 downto 0));
            end if;
            -- S20: mirror
            if s19_ph > p_ss - s19_ph then s20_ph <= p_ss - s19_ph;
            else                           s20_ph <= s19_ph; end if;
            -- S21
            vx := signed(resize(s20_ph, 21)) - signed(resize(p_sal, 21));
            if vx < 0 then vxa := unsigned(-vx); else vxa := unsigned(vx); end if;
            if vxa > 262143 then vxa := to_unsigned(262143, 21); end if;
            s21_ix <= vxa(17 downto 10);
            s21_fx <= vxa(9 downto 0);
            -- S22 (EBR), S23 (fabric)
            s22_fx <= s21_fx;
            s23_fx <= s22_fx;
            s23_lc <= lc_q;
            -- S24
            s24_ph <= bmul_u(unsigned(s23_lc(12 downto 0)), s23_fx(9 downto 5));
            s24_pl <= bmul_u(unsigned(s23_lc(12 downto 0)), s23_fx(4 downto 0));
            s24_v  <= unsigned(s23_lc(30 downto 13));
            -- S25
            s25_p <= shift_left(resize(s24_ph, 23), 5) + resize(s24_pl, 23);
            s25_v <= s24_v;
            -- S26
            vlc := resize(s25_v, 19) + resize(shift_right(s25_p + 512, 10), 19);
            if vlc > LC_CAP then s26_lc <= to_unsigned(LC_CAP, 18);
            else                 s26_lc <= vlc(17 downto 0); end if;
        end if;
    end process p_s17;

    ------------------------------------------------------------------------
    -- S27..S35: L = log2 D (log2 r, minus the star term) -> D = 2^L u-units:
    -- exp2 mantissa (interpolated), rounded shift, saturated.
    ------------------------------------------------------------------------
    p_s27 : process(clk)
        variable vl : signed(24 downto 0);
        variable vm : unsigned(18 downto 0);
        variable vs : integer range 0 to 18;
        variable vd : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            -- S27
            if p_star = '1' then
                vl := signed(resize(lr_d(26), 25)) - signed(resize(shift_left(resize(s26_lc, 19), 1), 25));
            else
                vl := signed(resize(lr_d(26), 25));
            end if;
            if vl < 0 then s27_l <= (others => '0'); else s27_l <= unsigned(vl(23 downto 0)); end if;
            -- S28
            s28_ie <= s27_l(23 downto 18);
            s28_i  <= s27_l(17 downto 10);
            s28_f  <= s27_l(9 downto 0);
            -- S29 (EBR), S30 (fabric)
            s29_ie <= s28_ie; s29_f <= s28_f;
            s30_ie <= s29_ie; s30_f <= s29_f;
            s30_ex <= ex_q;
            -- S31
            s31_ph <= bmul_u(unsigned(s30_ex(10 downto 0)), s30_f(9 downto 5));
            s31_pl <= bmul_u(unsigned(s30_ex(10 downto 0)), s30_f(4 downto 0));
            s31_v  <= unsigned(s30_ex(28 downto 11));
            s31_ie <= s30_ie;
            -- S32
            s32_p  <= shift_left(resize(s31_ph, 21), 5) + resize(s31_pl, 21);
            s32_v  <= s31_v;
            s32_ie <= s31_ie;
            -- S33: mantissa (19 bits)
            s33_m  <= to_unsigned(262144, 19) + resize(s32_v, 19) + resize(shift_right(s32_p + 512, 10), 19);
            s33_ie <= s32_ie;
            -- S34: rounding half for the shift
            if s33_ie > 18 then
                s34_ov <= '1'; vs := 0;
            else
                s34_ov <= '0'; vs := 18 - to_integer(s33_ie);
            end if;
            s34_sh <= to_unsigned(vs, 5);
            if vs = 0 then
                s34_mr <= resize(s33_m, 20);
            else
                s34_mr <= resize(s33_m, 20) + shift_left(to_unsigned(1, 20), vs - 1);
            end if;
            -- S35: D (polar)
            vd := shift_right(s34_mr, to_integer(s34_sh));
            if s34_ov = '1' or vd > D_MAX then
                s35_dp <= to_unsigned(D_MAX, 20);
            else
                s35_dp <= vd;
            end if;
        end if;
    end process p_s27;

    ------------------------------------------------------------------------
    -- S36..S42: merge, walk trackers, sd, filter gain, keys
    ------------------------------------------------------------------------
    p_s36 : process(clk)
        variable vd  : signed(19 downto 0);
        variable vsd : signed(21 downto 0);
        variable vsc : signed(29 downto 0);
        variable vk  : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            -- flags out of the D ring (aligned with S35, like the linear field)
            fl_gv(36) <= rd_q2(20);
            fl_wv     <= rd_q2(21);
            fl_sw(36) <= rd_q2(22);
            for i in 37 to 46 loop fl_gv(i) <= fl_gv(i - 1); end loop;
            for i in 37 to 42 loop fl_sw(i) <= fl_sw(i - 1); end loop;
            -- S36: the field
            if p_pol = '1' then
                s36_d <= signed(s35_dp);
            else
                s36_d <= signed(rd_q2(19 downto 0));
            end if;
            -- walk trackers (reset when a walk loads its start point)
            if wk_st = WK_LOAD then
                st_min <= to_signed(D_MAX, 20);
                st_max <= to_signed(-D_MAX - 1, 20);
            elsif fl_wv = '1' then
                if s36_d < st_min then st_min <= s36_d; end if;
                if s36_d > st_max then st_max <= s36_d; end if;
            end if;
            -- S37: sd
            if p_rev = '1' then
                vsd := resize(p_t, 22) - resize(s36_d, 22);
            else
                vsd := resize(s36_d, 22) - resize(p_t, 22);
            end if;
            s37_sd <= sat_s(vsd, 21);
            -- S38: sd x GM, split on GM
            s38_ph <= bmul(s37_sd, signed('0' & p_gm(8 downto 4)));
            s38_pl <= bmul(s37_sd, signed('0' & p_gm(3 downto 0)));
            -- S39
            s39_pr <= shift_left(resize(s38_ph, 30), 4) + resize(s38_pl, 30);
            -- S40: >> GS by multiples of 8
            s40_sc <= shift_right(s39_pr, 8 * to_integer(p_gs(4 downto 3)));
            -- S41: >> the rest, saturate, raw key
            vsc := shift_right(s40_sc, to_integer(p_gs(2 downto 0)));
            vk  := resize(sat_s(vsc, 18), 20);
            s41_kr <= resize(128 - vk, 19);
            -- S42: inner / outer keys (MIX: both = TK)
            if p_mix = '1' then
                s42_ki <= p_tk;
                s42_ko <= p_tk;
            else
                if s41_kr < 0 then s42_ki <= (others => '0');
                elsif s41_kr > 256 then s42_ki <= to_unsigned(256, 9);
                else s42_ki <= unsigned(s41_kr(8 downto 0)); end if;
                vk := resize(s41_kr, 20) + signed(resize(p_dk, 20));
                if vk < 0 then s42_ko <= (others => '0');
                elsif vk > 256 then s42_ko <= to_unsigned(256, 9);
                else s42_ko <= unsigned(vk(8 downto 0)); end if;
            end if;
        end if;
    end process p_s36;

    ------------------------------------------------------------------------
    -- S43..S46: compose A -> border -> B (one lerp per channel), gate
    ------------------------------------------------------------------------
    p_s43 : process(clk)
        variable va : t_c3;
        variable vb, vr : t_c3;
        variable vp : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            -- A from the ring; U/V swapped on lines the source paired the
            -- other way round
            va(0) := unsigned(ra_q2(31 downto 22));
            if fl_sw(42) = '1' then
                va(1) := unsigned(ra_q2(11 downto 2));
                va(2) := unsigned(ra_q2(21 downto 12));
            else
                va(1) := unsigned(ra_q2(21 downto 12));
                va(2) := unsigned(ra_q2(11 downto 2));
            end if;
            vb := (p_bcy, p_bcu, p_bcv);
            vr := (p_rcy, p_rcu, p_rcv);
            -- S43: endpoints and weight
            if s42_ko < 256 then
                s43_x <= va; s43_y <= vr; s43_k <= s42_ko;
            else
                s43_x <= vr; s43_y <= vb; s43_k <= s42_ki;
            end if;
            -- S44: differences
            for c in 0 to 2 loop
                s44_d(c) <= signed(resize(s43_y(c), 11)) - signed(resize(s43_x(c), 11));
            end loop;
            s44_x <= s43_x; s44_y <= s43_y; s44_k <= s43_k;
            -- S45: products (k = 256 is exactly Y)
            for c in 0 to 2 loop
                s45_p(c) <= resize(bmul(s44_d(c), signed('0' & s44_k(7 downto 0))), 20);
            end loop;
            s45_x <= s44_x; s45_y <= s44_y;
            s45_full <= s44_k(8);
            -- S46
            for c in 0 to 2 loop
                if s45_full = '1' then
                    s46_o(c) <= s45_y(c);
                else
                    vp := signed(resize(s45_x(c), 20)) + shift_right(s45_p(c) + 128, 8);
                    s46_o(c) <= unsigned(vp(9 downto 0));
                end if;
            end loop;
            -- syncs from the A ring (aligned with S42) -> aligned with S46
            sy_d(0) <= ra_q2(1 downto 0) & rb_q2(0);
            for i in 1 to 3 loop sy_d(i) <= sy_d(i - 1); end loop;
            -- output register: gated to neutral outside the generated window
            if fl_gv(46) = '1' then
                s_out_y <= s46_o(0); s_out_u <= s46_o(1); s_out_v <= s46_o(2);
            else
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= to_unsigned(512, 10);
                s_out_v <= to_unsigned(512, 10);
            end if;
            s_out_s <= fl_gv(46) & sy_d(3);
        end if;
    end process p_s43;

    ------------------------------------------------------------------------
    -- Microcoded engine (cubist / damier lineage, 24-bit).  Restarts at
    -- every frame anchor and runs tools/sw_ucode.py to END.  States: FETCH
    -- (after a jump) -> DEC (latch the instruction; read srcA) -> RDA (latch
    -- A, forwarding a register the previous instruction just wrote; read
    -- srcB) -> RDB (latch B and the CTL input) -> EX (one or more clocks).
    -- Every unit is serial: shifter, radix-4 Booth multiplier (+ a serial
    -- post-shift), restoring divider, quarter-sine ROM.
    ------------------------------------------------------------------------
    u_fa <= unsigned(u_a(8 downto 0)) when u_st = ST_EX and u_ir(31 downto 27) = "11010" and u_cyc = 0
            else u_pc + 1 when u_st = ST_EX
            else u_pc;
    rf_ra <= unsigned(u_rom(19 downto 13)) when u_st = ST_DEC else unsigned(u_ir(12 downto 6));

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
        variable v_op  : integer range 0 to 31;
        variable v_r   : signed(23 downto 0);
        variable v_b   : signed(23 downto 0);
        variable v_wr, v_adv, v_jmp : boolean;
        variable v_tgt : unsigned(8 downto 0);
        variable v_md, v_ms : signed(25 downto 0);
        variable v_t   : unsigned(24 downto 0);
        variable v_ang : unsigned(15 downto 0);
        variable v_pm  : unsigned(14 downto 0);
        variable v_sv  : signed(16 downto 0);
        variable v_i6  : integer range 0 to 63;
    begin
        if rising_edge(clk) then
            u_we    <= '0';
            u_wlast <= u_we;
            u_prt   <= '0';
            if s_vs_go = '1' then
                u_pc   <= (others => '0');
                u_st   <= ST_FETCH;
                u_done <= '0';
            elsif u_done = '0' then
                v_i6 := to_integer(unsigned(u_ir(5 downto 0)));
                case u_st is
                    when ST_FETCH =>
                        u_st <= ST_DEC;

                    when ST_DEC =>
                        u_ir <= u_rom;
                        u_st <= ST_RDA;

                    when ST_RDA =>
                        if u_wlast = '1' and unsigned(u_ir(19 downto 13)) = u_wa then
                            u_a <= signed(u_wd);
                        else
                            u_a <= signed(rf_q);
                        end if;
                        u_st <= ST_RDB;

                    when ST_RDB =>
                        u_b <= signed(rf_q);
                        case v_i6 is
                            when 0 => u_ctlr <= signed(resize(unsigned(registers_in(0)), 24));
                            when 1 => u_ctlr <= signed(resize(unsigned(registers_in(1)), 24));
                            when 2 => u_ctlr <= signed(resize(unsigned(registers_in(2)), 24));
                            when 3 => u_ctlr <= signed(resize(unsigned(registers_in(3)), 24));
                            when 4 => u_ctlr <= signed(resize(unsigned(registers_in(4)), 24));
                            when 5 => u_ctlr <= signed(resize(unsigned(registers_in(5)), 24));
                            when 6 => u_ctlr <= signed(resize(unsigned(registers_in(6)), 24));
                            when 7 => u_ctlr <= signed(resize(unsigned(registers_in(7)), 24));
                            when 8 => u_ctlr <= signed(resize(unsigned(registers_in(8)(3 downto 0)), 24));
                            when 9 => u_ctlr <= signed(resize(m_w, 24));
                            when 10 => u_ctlr <= signed(resize(m_h, 24));
                            when 11 =>
                                if m_fld = '1' then u_ctlr <= to_signed(1, 24);
                                else u_ctlr <= to_signed(0, 24); end if;
                            when others => u_ctlr <= (others => '0');
                        end case;
                        u_cyc <= (others => '0');
                        u_st  <= ST_EX;

                    when others =>                          -- EX
                        v_op  := to_integer(unsigned(u_ir(31 downto 27)));
                        v_wr  := false;
                        v_adv := true;
                        v_jmp := false;
                        v_r   := (others => '0');
                        v_tgt := unsigned(u_ir(8 downto 0));
                        case v_op is
                            when 1 =>                                   -- MOV
                                v_r := u_a; v_wr := true;
                            when 2 | 3 | 9 =>                           -- ADD SUB ADDI
                                if v_op = 9 then v_b := resize(signed(u_ir(12 downto 0)), 24);
                                elsif v_op = 3 then v_b := not u_b;
                                else v_b := u_b; end if;
                                if v_op = 3 then v_r := u_a + v_b + 1;
                                else v_r := u_a + v_b; end if;
                                v_wr := true;
                            when 6 =>                                   -- AND
                                v_r := u_a and u_b; v_wr := true;
                            when 11 =>                                  -- ANDI
                                v_r := u_a and resize(signed(u_ir(12 downto 0)), 24); v_wr := true;
                            when 7 =>                                   -- LDI
                                v_r := resize(signed(u_ir(19 downto 0)), 24); v_wr := true;
                            when 13 =>                                  -- CTL
                                if v_i6 = 13 or v_i6 = 14 then
                                    if wk_busy = '1' then
                                        v_adv := false;                -- hold until the walk flushed
                                    elsif v_i6 = 13 then
                                        v_r := resize(st_min, 24); v_wr := true;
                                    else
                                        v_r := resize(st_max, 24); v_wr := true;
                                    end if;
                                else
                                    v_r := u_ctlr; v_wr := true;
                                end if;
                            when 4 | 5 | 22 | 23 =>                     -- SHR SHL SHRV SHLV
                                if u_cyc = 0 then
                                    u_sh <= u_a;
                                    if v_op >= 22 then u_shc <= '0' & unsigned(u_b(4 downto 0));
                                    else u_shc <= '0' & unsigned(u_ir(4 downto 0)); end if;
                                    v_adv := false;
                                elsif u_shc /= 0 then
                                    if v_op = 4 or v_op = 23 then u_sh <= shift_right(u_sh, 1);
                                    else u_sh <= shift_left(u_sh, 1); end if;
                                    u_shc <= u_shc - 1;
                                    v_adv := false;
                                else
                                    v_r := u_sh; v_wr := true;
                                end if;
                            when 8 =>                                   -- MUL
                                if u_cyc = 0 then
                                    mu_a  <= u_a;
                                    mu_pl <= unsigned(u_b);
                                    mu_ph <= (others => '0');
                                    mu_bp <= '0';
                                    u_shc <= unsigned(u_ir(5 downto 0));
                                    v_adv := false;
                                elsif u_cyc <= 12 then
                                    case std_logic_vector'(mu_pl(1) & mu_pl(0) & mu_bp) is
                                        when "001" | "010" | "101" | "110" => v_md := resize(mu_a, 26);
                                        when "011" | "100"                 => v_md := shift_left(resize(mu_a, 26), 1);
                                        when others                        => v_md := (others => '0');
                                    end case;
                                    if mu_pl(1) = '1' then v_ms := mu_ph + (not v_md) + 1;
                                    else                   v_ms := mu_ph + v_md; end if;
                                    mu_ph <= shift_right(v_ms, 2);
                                    mu_pl <= unsigned(v_ms(1 downto 0)) & mu_pl(23 downto 2);
                                    mu_bp <= mu_pl(1);
                                    v_adv := false;
                                elsif u_shc /= 0 then                  -- the result shift, serially
                                    mu_ph <= shift_right(mu_ph, 1);
                                    mu_pl <= unsigned(mu_ph(0 downto 0)) & mu_pl(23 downto 1);
                                    u_shc <= u_shc - 1;
                                    v_adv := false;
                                else
                                    v_r := signed(mu_pl); v_wr := true;
                                end if;
                            when 10 =>                                  -- DIV (A << 16) / B
                                if u_cyc = 0 then
                                    dv_r <= resize(unsigned(u_a(23 downto 8)), 25);
                                    dv_q <= unsigned(u_a(7 downto 0)) & x"0000";
                                    dv_b <= unsigned(u_b);
                                    v_adv := false;
                                elsif u_cyc <= 24 then
                                    v_t := dv_r(23 downto 0) & dv_q(23);
                                    if v_t >= resize(dv_b, 25) then
                                        dv_r <= v_t - resize(dv_b, 25);
                                        dv_q <= dv_q(22 downto 0) & '1';
                                    else
                                        dv_r <= v_t;
                                        dv_q <= dv_q(22 downto 0) & '0';
                                    end if;
                                    v_adv := false;
                                else
                                    v_r := signed(dv_q); v_wr := true;
                                end if;
                            when 12 =>                                  -- SIN (grid sample)
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
                                    if sn_spec = '1' then v_sv := to_signed(32767, 17);
                                    else v_sv := signed('0' & sn_q2); end if;
                                    if sn_neg = '1' then v_sv := -v_sv; end if;
                                    v_r := resize(v_sv, 24); v_wr := true;
                                end if;
                            when 26 =>                                  -- LDM
                                if u_cyc = 0 then
                                    v_adv := false;                    -- u_fa = A this clock
                                else
                                    v_r := signed(u_rom(23 downto 0)); v_wr := true;
                                end if;
                            when 24 | 25 =>                             -- MIN MAX
                                if u_cyc = 0 then
                                    if u_a < u_b then u_blt <= '1'; else u_blt <= '0'; end if;
                                    v_adv := false;
                                else
                                    if (u_blt = '1') = (v_op = 24) then v_r := u_a; else v_r := u_b; end if;
                                    v_wr := true;
                                end if;
                            when 14 =>                                  -- PRT
                                if u_cyc = 0 then
                                    u_prt <= '1';
                                    u_prp <= unsigned(u_ir(5 downto 0));
                                    u_prv <= std_logic_vector(u_a);
                                    if v_i6 = 23 then v_adv := false; end if;
                                elsif u_cyc < 2 or wk_st /= WK_IDLE then
                                    v_adv := false;                    -- WALK: hold until stepped
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
                                v_tgt := unsigned(u_ir(22 downto 20)) & unsigned(u_ir(5 downto 0));
                                if u_cyc = 0 then
                                    if u_a < u_b then u_blt <= '1'; else u_blt <= '0'; end if;
                                    v_adv := false;
                                elsif (u_blt = '1') = (v_op = 17) then
                                    v_jmp := true;
                                end if;
                            when 19 =>                                  -- JR
                                v_tgt := unsigned(u_a(8 downto 0));
                                v_jmp := true;
                            when 27 =>                                  -- CALL
                                v_r := signed(resize(u_pc + 1, 24));
                                v_wr := true;
                                v_jmp := true;
                            when 20 =>                                  -- END
                                u_done <= '1';
                            when others => null;                        -- NOP / data
                        end case;

                        if v_wr then
                            u_we <= '1';
                            u_wd <= std_logic_vector(v_r);
                            if v_op = 27 then u_wa <= to_unsigned(127, 7);
                            else u_wa <= unsigned(u_ir(26 downto 20)); end if;
                        end if;
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

    -- engine ports (seeds SU / SV go straight into the line accumulators,
    -- WALK into the walker -- both in p_dda)
    p_ports : process(clk)
    begin
        if rising_edge(clk) then
            if u_prt = '1' then
                case to_integer(u_prp) is
                    when 0  => p_dux <= signed(u_prv);
                    when 1  => p_dvx <= signed(u_prv);
                    when 2  => p_duy <= signed(u_prv);
                    when 3  => p_dvy <= signed(u_prv);
                    when 8  => p_flags <= u_prv(4 downto 0);
                    when 9  => p_t   <= signed(u_prv(20 downto 0));
                    when 10 => p_gm  <= unsigned(u_prv(8 downto 0));
                    when 11 => p_gs  <= unsigned(u_prv(4 downto 0));
                    when 12 => p_dk  <= unsigned(u_prv(17 downto 0));
                    when 13 => p_tk  <= unsigned(u_prv(8 downto 0));
                    when 14 => p_ba  <= signed(u_prv(19 downto 0));
                    when 15 => p_bb  <= signed(u_prv(19 downto 0));
                    when 16 => p_sn  <= unsigned(u_prv(3 downto 0));
                    when 17 => p_ss  <= unsigned(u_prv(19 downto 0));
                    when 18 => p_sal <= unsigned(u_prv(17 downto 0));
                    when 19 => p_bcy <= unsigned(u_prv(9 downto 0));
                    when 20 => p_bcu <= unsigned(u_prv(19 downto 10)); p_bcv <= unsigned(u_prv(9 downto 0));
                    when 21 => p_rcy <= unsigned(u_prv(9 downto 0));
                    when 22 => p_rcu <= unsigned(u_prv(19 downto 10)); p_rcv <= unsigned(u_prv(9 downto 0));
                    when others => null;
                end case;
            end if;
        end if;
    end process p_ports;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= s_out_s(3);
    data_out.hsync_n <= s_out_s(2);
    data_out.vsync_n <= s_out_s(1);
    data_out.field_n <= s_out_s(0);

end architecture starwipe;
