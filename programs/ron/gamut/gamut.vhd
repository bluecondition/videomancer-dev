-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- gamut.vhd  (v0.2.0 -- Phase 2: library + DIVE/LIFT)
--
-- A palette explorer.  A curated library of colour palettes (ROM from
-- palettes.txt via gamut.py) is drawn as rounded cards with soft shadows on a
-- tinted backdrop, scrolling like a wheel picker with a spring.  P12 DIVE
-- lifts the cursor card out of the list until it fills the screen (LIFT).
--
-- Structure:
--   frame engine (gamut_engine, PROG_FRAME, runs at vsync): controls, springs,
--     density layout, hero rectangle lerp, squared radii / AA shift amounts,
--     shadow spreads, dominant-colour tint, carousel, row-walker start.
--   line engine  (gamut_engine, PROG_LINE, runs at each hsync): row walker,
--     per-row recede inset / dim, card y edges, EY^2 terms (square ROM),
--     palette lookups per column.
--   pixel path (14 stages): two rounded-rect evaluators (list card from
--     per-frame column constants + per-row inset; hero from per-frame
--     constants) using D2 = (2cx-1)^2 + (2cy-1)^2 against (2r)^2, shift-based
--     saturating AA / shadow ramps, ring, one shared colour-ROM port (hero
--     owns it where it covers), ONE blend: (over, cov) = hero | ring | list
--     over (list-if-fully-covered | shadowed backdrop).  Receding/fading
--     cards blend toward the backdrop (dim scales the list coverage).
--
-- Colours are authored in BT.601 (ROM unswapped); U/V are swapped once at the
-- output compose (the hardware convention, per mondrian/CGA).
--
-- Controls (Phase 2): K1 Browse, K3 Shelf, K4 View (density), K6 Shape,
-- S7 Glide, S10 Theme, P12 Dive.
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
use work.gamut_palettes_pkg.all;
use work.gamut_prog_pkg.all;

architecture gamut of program_top is
    constant C_PROCESSING_DELAY_CLKS : natural := 16;
    constant LATENCY : natural := 14;            -- p13 <-> gv_pipe(13); s_io is LATENCY+1 after data_in; pad 1 -> 16
    constant PAD     : natural := 1;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to PAD - 1) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLK : unsigned(9 downto 0) := to_unsigned(64, 10);

    subtype u12 is unsigned(11 downto 0);
    subtype s13 is signed(12 downto 0);
    subtype s20 is signed(19 downto 0);

    ---------------------------------------------------------------------------
    -- helpers
    ---------------------------------------------------------------------------
    -- clamp(8 + (v >> sh), 0, 16) for sh 0..7 : the AA ramp
    function sat_cov(v : s20; sh : unsigned(2 downto 0)) return unsigned is
        variable t : signed(19 downto 0);
        variable r : integer range 0 to 16;
    begin
        t := shift_right(v, to_integer(sh));
        if t < -8 then r := 0;
        elsif t > 8 then r := 16;
        else r := 8 + to_integer(t); end if;
        return to_unsigned(r, 5);
    end function;

    -- clamp(v >> sh, 0, 64) for sh 0..15 : the shadow falloff
    function sat_shadow(v : s20; sh : unsigned(3 downto 0)) return unsigned is
        variable t : signed(19 downto 0);
        variable r : integer range 0 to 64;
    begin
        t := shift_right(v, to_integer(sh));
        if t < 0 then r := 0;
        elsif t > 64 then r := 64;
        else r := to_integer(t); end if;
        return to_unsigned(r, 7);
    end function;

    function clamp255(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else return resize(unsigned(v), 8); end if;
    end function;

    function sat13(v : signed) return s13 is
    begin
        if v > 4095 then return to_signed(4095, 13);
        elsif v < -4095 then return to_signed(-4095, 13);
        else return resize(v, 13); end if;
    end function;

    function umin(a, b : unsigned) return unsigned is
    begin
        if a < b then return a; else return b; end if;
    end function;

    function s20u(v : unsigned) return s20 is
    begin
        return signed(resize(v, 20));
    end function;
    function s20s(v : signed) return s20 is
    begin
        return resize(v, 20);
    end function;

    -- square table for the line engine / list evaluator: 512 x 18
    type t_sq_rom is array (0 to 511) of std_logic_vector(17 downto 0);
    function make_sq return t_sq_rom is
        variable r : t_sq_rom;
    begin
        for i in 0 to 511 loop
            r(i) := std_logic_vector(to_unsigned(i * i, 18));
        end loop;
        return r;
    end function;

    ---------------------------------------------------------------------------
    -- Timing / counters
    ---------------------------------------------------------------------------
    signal pixel_x, pixel_y : u12 := (others => '0');
    signal prev_hsync_n, prev_vsync_n : std_logic := '1';
    signal s_xcnt, s_wlin : u12 := to_unsigned(1920, 12);
    signal s_yfst, s_ylast : u12 := (others => '0');
    signal s_yfirst_f : u12 := (others => '0');
    signal s_seenav : std_logic := '0';
    signal s_sawav  : std_logic := '0';
    signal s_wid : u12 := to_unsigned(1920, 12);
    signal s_hgt : u12 := to_unsigned(1080, 12);
    signal s_hedge_q, s_lineav : std_logic := '0';

    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : u12 := (others => '0');
    signal g_wm1 : u12 := to_unsigned(1919, 12);

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);
    signal gv_pipe : std_logic_vector(0 to LATENCY - 1) := (others => '0');

    ---------------------------------------------------------------------------
    -- ROMs
    ---------------------------------------------------------------------------
    signal colour_rom : t_colour_rom := C_COLOUR_ROM;
    signal pal_rom    : t_palette_rom := C_PALETTE_ROM;
    signal sq_rom     : t_sq_rom := make_sq;
    signal sqh_rom    : t_sq_rom := make_sq;
    signal rom_q, rom_q2 : std_logic_vector(31 downto 0) := (others => '0');
    signal pal_q      : std_logic_vector(15 downto 0) := (others => '0');
    signal sq_q, sqh_q : std_logic_vector(17 downto 0) := (others => '0');
    signal rom_addr_f : unsigned(9 downto 0) := (others => '0');
    signal pal_addr_f, pal_addr_l : unsigned(7 downto 0) := (others => '0');
    signal sq_addr_l  : unsigned(8 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Controls (free-running) and theme constants
    ---------------------------------------------------------------------------
    signal c_k1, c_k3, c_k4, c_k6, c_p12 : unsigned(9 downto 0) := (others => '0');
    signal c_glide, c_light : std_logic := '0';
    signal f_light : std_logic := '0';
    signal lfsr    : unsigned(15 downto 0) := x"ACE1";

    ---------------------------------------------------------------------------
    -- Engines, register file, copier
    ---------------------------------------------------------------------------
    signal fe_start, fe_busy, le_start, le_busy, e_busy, e_isframe : std_logic := '0';
    signal fe_start_p, le_start_p : std_logic := '0';
    signal e_rfa_e, e_rfa, cp_rfa : unsigned(6 downto 0) := (others => '0');
    signal e_esel : unsigned(7 downto 0) := (others => '0');
    signal e_dsel, cp_dsel, n_dsel : unsigned(6 downto 0) := (others => '0');
    signal e_dval, cp_dval, n_dval, e_sval, e_ext, e_ext_q : s20 := (others => '0');
    signal e_isext_q : std_logic := '0';
    signal e_mres : signed(32 downto 0) := (others => '0');
    signal e_we, n_we, cp_we, cp_busy, cp_go : std_logic := '0';
    signal cp_st : integer range 0 to 3 := 0;
    signal cp_i  : unsigned(4 downto 0) := (others => '0');
    signal rfile : t_rfile := C_RF_INIT;
    signal rf_q  : std_logic_vector(19 downto 0) := (others => '0');
    attribute no_rw_check : boolean;
    attribute no_rw_check of rfile : signal is true;

    ---------------------------------------------------------------------------
    -- Named frame constants (twins of file entries, or named-only)
    ---------------------------------------------------------------------------
    signal f_dens    : unsigned(1 downto 0) := "01";
    signal f_shelf   : unsigned(3 downto 0) := to_unsigned(9, 4);
    signal f_card_h  : u12 := to_unsigned(100, 12);
    type t_col3 is array (0 to 2) of s13;
    signal f_colx, f_x1m1 : t_col3 := (others => (others => '0'));
    signal f_b1, f_b2 : u12 := (others => '1');
    signal f_r, f_rh : unsigned(7 downto 0) := (others => '0');
    signal f_curcol  : unsigned(1 downto 0) := (others => '0');
    signal f_ex0, f_ex1, f_ey0, f_ey1, f_ex1m1 : s13 := (others => '0');
    signal f_shsl, f_shsh : unsigned(3 downto 0) := (others => '0');
    signal f_shl, f_shh, f_shg : unsigned(2 downto 0) := (others => '0');
    signal f_rlt2, f_rhlt2 : std_logic := '0';
    signal f_ringa   : unsigned(4 downto 0) := (others => '0');
    signal f_hbase   : unsigned(9 downto 0) := (others => '0');
    signal f_hn      : unsigned(4 downto 0) := to_unsigned(1, 5);
    signal f_tintu, f_tintv : signed(10 downto 0) := to_signed(512, 11);
    signal f_hw      : u12 := (others => '0');
    signal f_cx2, f_cy2 : unsigned(21 downto 0) := (others => '0');
    -- derived (combinational)
    signal f_cx, f_cy : u12;
    signal f_shrad   : unsigned(1 downto 0);
    signal f_ybase, f_ringy : unsigned(9 downto 0);
    signal f_inset_px, f_lift, f_Sl, f_oyl, f_oyh : unsigned(7 downto 0);
    signal f_kh, f_kw, f_kg : unsigned(7 downto 0);
    signal f_cols    : unsigned(1 downto 0);
    signal f_colxcur : s13;
    signal f_pow2l, f_pow2h : s20;

    ---------------------------------------------------------------------------
    -- Named per-line constants (copied from the file at each hsync)
    ---------------------------------------------------------------------------
    signal l_y       : s13 := (others => '0');
    signal l_inset, l_cri : u12 := (others => '0');
    signal l_dim     : unsigned(6 downto 0) := (others => '0');
    signal l_y0, l_y1 : s13 := (others => '0');
    signal l_y0f     : unsigned(3 downto 0) := (others => '0');
    signal l_w       : u12 := (others => '0');
    signal l_KL, l_KSL, l_KH, l_KG, l_KGT, l_KSH : s20 := (others => '0');
    signal l_ex      : std_logic_vector(2 downto 0) := (others => '0');
    type t_b3 is array (0 to 2) of unsigned(9 downto 0);
    signal l_base    : t_b3 := (others => (others => '0'));
    type t_n3 is array (0 to 2) of unsigned(4 downto 0);
    signal l_n       : t_n3 := (others => to_unsigned(1, 5));
    -- registered per-line derived flags
    signal lc_covy   : unsigned(4 downto 0) := (others => '0');
    signal lc_iny, lc_inyh : std_logic := '0';
    -- radial backdrop per-line term
    signal r_dy      : s13 := (others => '0');
    signal r_sqy     : unsigned(21 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- Pixel pipeline
    ---------------------------------------------------------------------------
    signal p0_x    : u12 := (others => '0');
    signal p0_sel  : integer range 0 to 2 := 0;
    signal p1_x    : u12;
    signal p1_colx, p1_x1m1 : s13;
    signal p1_base : unsigned(9 downto 0);
    signal p1_n    : unsigned(4 downto 0);
    signal p1_ex   : std_logic;
    signal p2_dl, p2_dr, p2_dlh, p2_drh : s13;
    signal p2_base : unsigned(9 downto 0);
    signal p2_n    : unsigned(4 downto 0);
    signal p2_ex   : std_logic;
    signal p3_m, p3_mh : s13;
    signal p3_ex, p3_pri : std_logic;
    signal p3_acc, p3_acch : u12 := (others => '0');
    signal p3_bar, p3_barh : unsigned(3 downto 0) := (others => '0');
    signal p3_base : unsigned(9 downto 0);
    signal p4_cx, p4_cxh : unsigned(7 downto 0);
    signal p4_in, p4_inh, p4_pri : std_logic;
    signal p4_addr : unsigned(9 downto 0);
    signal p5_EX, p5_EXh : unsigned(8 downto 0);
    signal p5_in, p5_inh, p5_pri : std_logic;
    signal p5_addr : unsigned(9 downto 0);
    signal p6_in, p6_inh, p6_pri : std_logic;
    signal p6_addr : unsigned(9 downto 0);
    signal p7_EX2, p7_EX2h : unsigned(17 downto 0);
    signal p7_in, p7_inh, p7_pri : std_logic;
    signal p7_addr : unsigned(9 downto 0);
    signal p8_dl, p8_dsl, p8_dh, p8_dsh, p8_dg, p8_dgt : s20;
    signal p8_in, p8_inh, p8_pri : std_logic;
    signal p8_addr : unsigned(9 downto 0);
    signal p9_covc, p9_covch, p9_ra, p9_rb : unsigned(4 downto 0);
    signal p9_shl, p9_shh : unsigned(6 downto 0);
    signal p9_in, p9_inh, p9_pri : std_logic;
    signal p10_covl, p10_covh, p10_ring : unsigned(4 downto 0);
    signal p10_shbg : unsigned(6 downto 0);
    signal p10_pri : std_logic;
    signal hold_l, hold_h : std_logic_vector(31 downto 0) := (others => '0');
    signal p11_covl, p11_covh, p11_ring : unsigned(4 downto 0);
    signal p11_bgy : unsigned(9 downto 0);
    signal p12_dy, p12_du, p12_dv : signed(11 downto 0);
    signal p12_uy, p12_uu, p12_uv : unsigned(9 downto 0);
    signal p12_cov : unsigned(4 downto 0);
    signal p13_y, p13_u, p13_v : signed(11 downto 0);

    -- backdrop radial pipeline
    signal r_dx    : s13 := (others => '0');
    signal r_sqx   : unsigned(21 downto 0) := (others => '0');
    signal r_rad   : unsigned(22 downto 0) := (others => '0');
    signal r_lift  : unsigned(7 downto 0) := (others => '0');
    signal r_y     : unsigned(9 downto 0) := (others => '0');

    signal s_io : t_video_stream_yuv444_30b;

begin
    ---------------------------------------------------------------------------
    -- Free-running control conditioning
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            c_k1  <= unsigned(registers_in(0));
            c_k3  <= unsigned(registers_in(2));
            c_k4  <= unsigned(registers_in(3));
            c_k6  <= unsigned(registers_in(5));
            c_p12 <= unsigned(registers_in(7));
            c_glide <= registers_in(6)(0);
            c_light <= registers_in(6)(3);
            lfsr <= lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Position counters, width/height measurement, engine triggers
    ---------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_y : s13;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;
            fe_start <= '0';
            le_start <= '0';

            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif g_avq = '1' then
                if s_xcnt > to_unsigned(16, 12) then s_wlin <= s_xcnt; end if;
                s_xcnt <= (others => '0');
            end if;

            if v_h_edge = '1' then
                pixel_x <= (others => '0');
            elsif g_avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if g_avid = '1' then
                s_sawav <= '1';
                s_ylast <= pixel_y;
                if s_seenav = '0' then
                    s_seenav <= '1';
                    s_yfst <= pixel_y;
                end if;
            end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');
                s_seenav <= '0';
                if s_sawav = '1' then
                    s_yfirst_f <= s_yfst;
                    s_hgt <= s_ylast - s_yfst + 1;
                    s_wid <= s_wlin;
                    g_wm1 <= s_wlin - 1;
                    f_light <= c_light;
                    fe_start_p <= '1';
                    s_sawav <= '0';
                end if;
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
                -- radial dy^2 walker for the upcoming line
                v_y := resize(signed('0' & pixel_y), 13) + 1 - resize(signed('0' & s_yfirst_f), 13);
                if v_y <= 0 then
                    r_dy  <= -signed('0' & f_cy);
                    r_sqy <= f_cy2;
                else
                    r_dy  <= r_dy + 1;
                    r_sqy <= unsigned(signed(r_sqy) + resize(r_dy & '0', 22) + 1);
                end if;
            end if;
            -- copier at every hsync; frame engine once the copier is idle; line engine (for the
            -- NEXT line) at the generated avid rise, or at hsync during vblank lines
            s_hedge_q <= v_h_edge;
            cp_go <= s_hedge_q;
            if s_hedge_q = '1' and s_lineav = '0' then le_start_p <= '1'; end if;
            if g_avid = '1' then s_lineav <= '1'; elsif v_h_edge = '1' then s_lineav <= '0'; end if;
            if g_avr = '1' then le_start_p <= '1'; end if;
            if fe_start_p = '1' and cp_busy = '0' and cp_go = '0' and e_busy = '0' and le_start = '0' then
                fe_start <= '1'; fe_start_p <= '0'; e_isframe <= '1';
            elsif le_start_p = '1' and cp_busy = '0' and cp_go = '0' and e_busy = '0' and fe_start_p = '0' and fe_start = '0' then
                le_start <= '1'; le_start_p <= '0'; e_isframe <= '0';
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Generated avid
    ---------------------------------------------------------------------------
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;
            if g_hsr = '1' then g_ph <= '0'; g_arm <= '1';
            else g_ph <= not g_ph; end if;
            g_rsr <= g_rsr(1 downto 0) & '0';
            if g_avr = '1' and g_arm = '1' then
                g_arm <= '0'; g_rsr(0) <= '1';
                g_sel <= g_ph xor g_par;
                if g_ph = '1' then
                    if g_pc /= 15 then g_pc <= g_pc + 1; end if;
                else
                    if g_pc /= 0 then g_pc <= g_pc - 1; end if;
                end if;
            end if;
            if g_pc = 15 then g_par <= '1'; elsif g_pc = 0 then g_par <= '0'; end if;
            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1'; g_cnt <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then g_avid <= '0'; end if;
            end if;
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Derived frame constants (combinational)
    ---------------------------------------------------------------------------
    f_cx <= '0' & s_wid(11 downto 1);
    f_cy <= '0' & s_hgt(11 downto 1);
    f_shrad <= "10" when s_wid >= 1536 else "01" when s_wid >= 1024 else "00";
    f_ybase <= to_unsigned(600, 10) when f_light = '1' else to_unsigned(110, 10);
    f_ringy <= to_unsigned(110, 10) when f_light = '1' else to_unsigned(880, 10);
    f_inset_px <= f_card_h(11 downto 4);
    f_lift <= resize(f_card_h(11 downto 5), 8);
    f_Sl   <= resize(f_card_h(11 downto 3), 8);
    f_oyl  <= resize(f_card_h(11 downto 5), 8);
    f_oyh  <= resize(f_card_h(11 downto 5), 8) + resize(f_card_h(11 downto 4), 8);
    with f_dens select f_kh <= to_unsigned(64, 8) when "00", to_unsigned(37, 8) when "01", to_unsigned(32, 8) when "10", to_unsigned(26, 8) when others;
    with f_dens select f_kw <= to_unsigned(128, 8) when "00", to_unsigned(115, 8) when "01", to_unsigned(85, 8) when "10", to_unsigned(64, 8) when others;
    with f_dens select f_kg <= to_unsigned(10, 8) when "00", to_unsigned(6, 8) when "01", to_unsigned(6, 8) when "10", to_unsigned(5, 8) when others;
    with f_dens select f_cols <= "01" when "00", "01" when "01", "10" when "10", "11" when others;
    f_colxcur <= f_colx(0) when f_curcol = 0 else f_colx(1) when f_curcol = 1 else f_colx(2);
    f_pow2l <= shift_left(to_signed(64, 20), to_integer(f_shsl));
    f_pow2h <= shift_left(to_signed(64, 20), to_integer(f_shsh));

    ---------------------------------------------------------------------------
    -- Engines, register file, external-source mux, copy sequencer
    ---------------------------------------------------------------------------
    u_eng : entity work.gamut_engine
        port map (clk => clk, start => fe_start, start_l => le_start, busy => e_busy,
                  rf_addr => e_rfa_e, ext_sel => e_esel, src_val => e_sval,
                  dst_sel => e_dsel, dst_we => e_we, dst_val => e_dval, m_res => e_mres);
    fe_busy <= e_busy and e_isframe;
    le_busy <= e_busy and not e_isframe;
    e_rfa  <= cp_rfa when cp_busy = '1' else e_rfa_e;
    -- the external mux is registered (e_ext_q) alongside the file read; the engine
    -- consumes src_val one stage later
    e_sval <= e_ext_q when e_isext_q = '1' else signed(rf_q);
    p_extq : process(clk)
    begin
        if rising_edge(clk) then
            e_ext_q   <= e_ext;
            e_isext_q <= e_esel(7);
        end if;
    end process;

    p_ext : process(all)
        variable v : s20;
    begin
        v := (others => '0');
        case to_integer(e_esel(6 downto 0)) is
            when S_ACC - 128    => v := e_dval;
            when S_MRES0 - 128  => v := e_mres(19 downto 0);
            when S_MRES4 - 128  => v := e_mres(23 downto 4);
            when S_MRES8 - 128  => v := e_mres(27 downto 8);
            when S_MRES10 - 128 => v := e_mres(29 downto 10);
            when S_MRESFRAC - 128 => v := s20u(unsigned(e_mres(9 downto 0)));
            when S_W - 128      => v := s20u(s_wid);
            when S_H - 128      => v := s20u(s_hgt);
            when S_K1 - 128     => v := s20u(c_k1);
            when S_K3 - 128     => v := s20u(c_k3);
            when S_K4 - 128     => v := s20u(c_k4);
            when S_K6 - 128     => v := s20u(c_k6);
            when S_P12 - 128    => v := s20u(c_p12);
            when S_GLIDE - 128  => v := (0 => c_glide, others => '0');
            when S_LIGHT - 128  => v := (0 => f_light, others => '0');
            when S_KH - 128     => v := s20u(f_kh);
            when S_KW - 128     => v := s20u(f_kw);
            when S_KG - 128     => v := s20u(f_kg);
            when S_COLS - 128   => v := s20u(f_cols);
            when S_COLXCUR - 128 => v := s20s(f_colxcur);
            when S_SHFIRST - 128 => v := s20u(unsigned(C_SHELF_ROM(to_integer(f_shelf))(15 downto 8)));
            when S_SHCOUNT - 128 => v := s20u(unsigned(C_SHELF_ROM(to_integer(f_shelf))(7 downto 0)));
            when S_POW2SHSL - 128 => v := f_pow2l;
            when S_POW2SHSH - 128 => v := f_pow2h;
            when S_ROMROLE - 128 => v := s20u(unsigned(rom_q(1 downto 0)));
            when S_ROMU - 128   => v := s20u(unsigned(rom_q(21 downto 12)));
            when S_ROMV - 128   => v := s20u(unsigned(rom_q(11 downto 2)));
            when S_PALBASE - 128 => v := s20u(unsigned(pal_q(13 downto 4)));
            when S_PALN - 128   => v := s20u(unsigned(pal_q(3 downto 0)));
            when S_SQ - 128     => v := s20u(unsigned(sq_q));
            when S_PIXELY - 128 => v := s20u(pixel_y);
            when S_YFIRST - 128 => v := s20u(s_yfirst_f);
            when S_CX - 128     => v := s20u(f_cx);
            when S_CY - 128     => v := s20u(f_cy);
            when S_INSETPX - 128 => v := s20u(f_inset_px);
            when S_LIFT - 128   => v := s20u(f_lift);
            when S_SL - 128     => v := s20u(f_Sl);
            when S_OYL - 128    => v := s20u(f_oyl);
            when S_OYH - 128    => v := s20u(f_oyh);
            when others         => v := (others => '0');
        end case;
        e_ext <= v;
    end process;

    -- register file (EBR): one read port (engine A stage / copier), one write (engine ST)
    p_rf : process(clk)
    begin
        if rising_edge(clk) then
            if e_we = '1' then rfile(to_integer(e_dsel)) <= std_logic_vector(e_dval); end if;
            rf_q <= rfile(to_integer(e_rfa));
        end if;
    end process;

    -- copy sequencer: at each hsync move the per-line file entries into the named FFs
    p_copy : process(clk)
    begin
        if rising_edge(clk) then
            cp_we <= '0';
            case cp_st is
                when 0 =>
                    -- skip the copy while the engine owns the file port (the frame program
                    -- outlasts an SD line; vblank lines have nothing to show anyway)
                    if cp_go = '1' and e_busy = '0' then cp_st <= 1; cp_i <= (others => '0'); cp_busy <= '1'; end if;
                when 1 => cp_rfa <= C_COPY_LIST(to_integer(cp_i)); cp_st <= 2;
                when 2 => cp_st <= 3;
                when 3 =>
                    cp_we <= '1'; cp_dsel <= C_COPY_LIST(to_integer(cp_i)); cp_dval <= signed(rf_q);
                    if cp_i = C_COPY_LEN - 1 then cp_st <= 0; cp_busy <= '0';
                    else cp_i <= cp_i + 1; cp_st <= 1; end if;
                when others => cp_st <= 0;
            end case;
        end if;
    end process;

    -- named-FF decode: frame-engine stores, copier writes, and named-only dests from either engine
    n_we   <= '1' when cp_we = '1' or (e_we = '1' and (e_isframe = '1' or e_dsel >= 96)) else '0';
    n_dsel <= cp_dsel when cp_we = '1' else e_dsel;
    n_dval <= cp_dval when cp_we = '1' else e_dval;

    p_named : process(clk)
        variable d : s20;
    begin
        if rising_edge(clk) then
            if n_we = '1' then
                d := n_dval;
                case to_integer(n_dsel) is
                    when D_DENS => f_dens <= unsigned(d(1 downto 0));
                    when D_SHELF => f_shelf <= unsigned(d(3 downto 0));
                    when D_CARDH => f_card_h <= unsigned(d(11 downto 0));
                    when D_COLX0 => f_colx(0) <= sat13(d);
                    when D_COLX1 => f_colx(1) <= sat13(d);
                    when D_COLX2 => f_colx(2) <= sat13(d);
                    when D_B1 => f_b1 <= unsigned(d(11 downto 0));
                    when D_B2 => f_b2 <= unsigned(d(11 downto 0));
                    when D_R => f_r <= unsigned(d(7 downto 0));
                    when D_RH => f_rh <= unsigned(d(7 downto 0));
                    when D_CURCOL => f_curcol <= unsigned(d(1 downto 0));
                    when D_EX0 => f_ex0 <= sat13(d);
                    when D_EX1 => f_ex1 <= sat13(d); f_ex1m1 <= sat13(d) - 1;
                    when D_EY0 => f_ey0 <= sat13(d);
                    when D_EY1 => f_ey1 <= sat13(d);
                    when D_SHSH => f_shsh <= unsigned(d(3 downto 0));
                    when D_SHSL => f_shsl <= unsigned(d(3 downto 0));
                    when D_RLT2 => f_rlt2 <= d(0);
                    when D_RHLT2 => f_rhlt2 <= d(0);
                    when D_SHL => f_shl <= unsigned(d(2 downto 0));
                    when D_SHH => f_shh <= unsigned(d(2 downto 0));
                    when D_SHG => f_shg <= unsigned(d(2 downto 0));
                    when D_RINGA => f_ringa <= unsigned(d(4 downto 0));
                    when D_HBASE => f_hbase <= unsigned(d(9 downto 0));
                    when D_HN => f_hn <= unsigned(d(4 downto 0));
                    when D_TINTU => f_tintu <= d(10 downto 0);
                    when D_TINTV => f_tintv <= d(10 downto 0);
                    when D_HW => f_hw <= unsigned(d(11 downto 0));
                    when D_CX2 => f_cx2 <= unsigned(d) & "00";
                    when D_CY2 => f_cy2 <= unsigned(d) & "00";
                    when D_ROMADDR => rom_addr_f <= unsigned(d(9 downto 0));
                    when D_PALADDRF => pal_addr_f <= unsigned(d(7 downto 0));
                    when D_PALADDR => pal_addr_l <= unsigned(d(7 downto 0));
                    when D_SQADDR => sq_addr_l <= unsigned(d(8 downto 0));
                    when D_X1M10 => f_x1m1(0) <= sat13(d);
                    when D_X1M11 => f_x1m1(1) <= sat13(d);
                    when D_X1M12 => f_x1m1(2) <= sat13(d);
                    -- per-line (via the copier)
                    when D_LY => l_y <= sat13(d);
                    when D_LINSET => l_inset <= unsigned(d(11 downto 0));
                    when D_LCRI => l_cri <= unsigned(d(11 downto 0));
                    when D_LDIM => l_dim <= unsigned(d(6 downto 0));
                    when D_LY0 => l_y0 <= sat13(d);
                    when D_LY1 => l_y1 <= sat13(d);
                    when D_LY0F => l_y0f <= unsigned(d(3 downto 0));
                    when D_LW => l_w <= unsigned(d(11 downto 0));
                    when D_KL => l_KL <= d;
                    when D_KSL => l_KSL <= d;
                    when D_KH => l_KH <= d;
                    when D_KG => l_KG <= d;
                    when D_KGT => l_KGT <= d;
                    when D_KSH => l_KSH <= d;
                    when D_LEX0 => l_ex(0) <= d(0);
                    when D_LEX1 => l_ex(1) <= d(0);
                    when D_LEX2 => l_ex(2) <= d(0);
                    when D_LBASE0 => l_base(0) <= unsigned(d(9 downto 0));
                    when D_LBASE1 => l_base(1) <= unsigned(d(9 downto 0));
                    when D_LBASE2 => l_base(2) <= unsigned(d(9 downto 0));
                    when D_LN0 => l_n(0) <= unsigned(d(4 downto 0));
                    when D_LN1 => l_n(1) <= unsigned(d(4 downto 0));
                    when D_LN2 => l_n(2) <= unsigned(d(4 downto 0));
                    when others => null;
                end case;
            end if;
            -- per-line derived flags (static during active video)
            if l_y = l_y0 then lc_covy <= 16 - resize(l_y0f, 5);
            elsif l_y = l_y1 then lc_covy <= resize(l_y0f, 5);
            else lc_covy <= to_unsigned(16, 5); end if;
            if l_y >= l_y0 and l_y <= l_y1 then lc_iny <= '1'; else lc_iny <= '0'; end if;
            if l_y >= f_ey0 and l_y < f_ey1 then lc_inyh <= '1'; else lc_inyh <= '0'; end if;
        end if;
    end process;
    ---------------------------------------------------------------------------
    -- ROM ports
    ---------------------------------------------------------------------------
    p_rom : process(clk)
        variable v_addr : unsigned(9 downto 0);
        variable v_sq   : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            if fe_busy = '1' then v_addr := rom_addr_f; else v_addr := p8_addr; end if;
            rom_q  <= colour_rom(to_integer(v_addr));
            rom_q2 <= rom_q;
            if fe_busy = '1' then pal_q <= pal_rom(to_integer(pal_addr_f));
            else pal_q <= pal_rom(to_integer(pal_addr_l)); end if;
            if le_busy = '1' then v_sq := sq_addr_l; else v_sq := p5_EX; end if;
            sq_q  <= sq_rom(to_integer(v_sq));
            sqh_q <= sqh_rom(to_integer(p5_EXh));
        end if;
    end process;

    ---------------------------------------------------------------------------
    -- Pixel pipeline
    ---------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_t   : u12;
        variable v_q   : unsigned(11 downto 0);
        variable v_p   : signed(17 downto 0);
        variable v_rad : unsigned(22 downto 0);
        variable v_oy, v_ou, v_ov : unsigned(9 downto 0);
        variable v_uy, v_uu, v_uv : unsigned(9 downto 0);
        variable v_cov : unsigned(4 downto 0);
        variable v_listfull : std_logic;
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;
            gv_pipe(0) <= g_avid;
            for i in 1 to LATENCY - 1 loop gv_pipe(i) <= gv_pipe(i - 1); end loop;

            -- P0: column select
            p0_x <= pixel_x;
            if pixel_x >= f_b2 then p0_sel <= 2;
            elsif pixel_x >= f_b1 then p0_sel <= 1;
            else p0_sel <= 0; end if;

            -- radial backdrop: the accumulator runs 6 stages behind the counter so r_y lands at stage 10
            if gv_pipe(6) = '0' then
                r_dx  <= -signed('0' & f_cx);
                r_sqx <= f_cx2;
            else
                r_dx  <= r_dx + 1;
                r_sqx <= unsigned(signed(r_sqx) + resize(r_dx & '0', 22) + 1);
            end if;
            v_rad := resize(r_sqx, 23) + resize(r_sqy, 23);
            case to_integer(f_shrad) is
                when 2 => r_rad <= shift_right(v_rad, 12);
                when 1 => r_rad <= shift_right(v_rad, 11);
                when others => r_rad <= shift_right(v_rad, 10);
            end case;
            if r_rad >= 160 then r_lift <= (others => '0'); else r_lift <= 160 - resize(r_rad, 8); end if;
            if f_light = '1' then
                r_y <= f_ybase + resize(r_lift, 10) + resize(lfsr(1 downto 0), 10);
            else
                r_y <= f_ybase + resize(r_lift(7 downto 2), 10) + resize(lfsr(1 downto 0), 10);
            end if;

            -- P1: column constants
            p1_x    <= p0_x;
            p1_colx <= f_colx(p0_sel);
            p1_x1m1 <= f_x1m1(p0_sel);
            p1_base <= l_base(p0_sel);
            p1_n    <= l_n(p0_sel);
            p1_ex   <= l_ex(p0_sel);

            -- P2: distances from the card's left/right edges (frame constants)
            p2_dl  <= signed('0' & p1_x) - p1_colx;
            p2_dr  <= p1_x1m1 - signed('0' & p1_x);
            p2_dlh <= signed('0' & p1_x) - f_ex0;
            p2_drh <= f_ex1m1 - signed('0' & p1_x);
            p2_base <= p1_base; p2_n <= p1_n; p2_ex <= p1_ex;

            -- P3: nearest-edge distance, bar DDAs, port priority
            if p2_dl < p2_dr then p3_m <= p2_dl; else p3_m <= p2_dr; end if;
            if p2_dlh < p2_drh then p3_mh <= p2_dlh; else p3_mh <= p2_drh; end if;
            if p2_dl = signed('0' & l_inset) then
                p3_acc <= (others => '0'); p3_bar <= (others => '0');
            else
                v_t := p3_acc + resize(p2_n, 12);
                if v_t >= l_w then p3_acc <= v_t - l_w; p3_bar <= p3_bar + 1;
                else p3_acc <= v_t; end if;
            end if;
            if p2_dlh = 0 then
                p3_acch <= (others => '0'); p3_barh <= (others => '0');
            else
                v_t := p3_acch + resize(f_hn, 12);
                if v_t >= f_hw then p3_acch <= v_t - f_hw; p3_barh <= p3_barh + 1;
                else p3_acch <= v_t; end if;
            end if;
            if p2_dlh >= -1 and p2_drh >= -1 and lc_inyh = '1' then p3_pri <= '1'; else p3_pri <= '0'; end if;
            p3_base <= p2_base; p3_ex <= p2_ex;

            -- P4: corner distance cx = (r + inset) - m, in-range, colour address
            p4_cx  <= clamp255(signed('0' & l_cri) - p3_m);
            p4_cxh <= clamp255(signed(resize(f_rh, 13)) - p3_mh);
            if p3_m >= signed('0' & l_inset) and p3_ex = '1' and lc_iny = '1' then p4_in <= '1'; else p4_in <= '0'; end if;
            if p3_mh >= 0 and lc_inyh = '1' then p4_inh <= '1'; else p4_inh <= '0'; end if;
            p4_pri <= p3_pri;
            if p3_pri = '1' then p4_addr <= f_hbase + resize(p3_barh, 10);
            else p4_addr <= p3_base + resize(p3_bar, 10); end if;

            -- P5: EX = max(2cx-1, 0)   (square ROMs read from these at the next edge)
            if p4_cx = 0 then p5_EX <= (others => '0'); else p5_EX <= (p4_cx & '0') - 1; end if;
            if p4_cxh = 0 then p5_EXh <= (others => '0'); else p5_EXh <= (p4_cxh & '0') - 1; end if;
            p5_in <= p4_in; p5_inh <= p4_inh; p5_pri <= p4_pri; p5_addr <= p4_addr;

            -- P6: squares arrive in sq_q / sqh_q (EBR output registers)
            p6_in <= p5_in; p6_inh <= p5_inh; p6_pri <= p5_pri; p6_addr <= p5_addr;

            -- P7: fabric re-register before arithmetic
            p7_EX2  <= unsigned(sq_q);
            p7_EX2h <= unsigned(sqh_q);
            p7_in <= p6_in; p7_inh <= p6_inh; p7_pri <= p6_pri; p7_addr <= p6_addr;

            -- P8: differences against the per-line (radius^2 - EY^2) terms
            p8_dl  <= l_KL  - signed('0' & p7_EX2);
            p8_dsl <= l_KSL - signed('0' & p7_EX2);
            p8_dh  <= l_KH  - signed('0' & p7_EX2h);
            p8_dsh <= l_KSH - signed('0' & p7_EX2h);
            p8_dg  <= signed('0' & p7_EX2h) - l_KG;
            p8_dgt <= l_KGT - signed('0' & p7_EX2h);
            p8_in <= p7_in; p8_inh <= p7_inh; p8_pri <= p7_pri; p8_addr <= p7_addr;

            -- P9: AA ramps / shadow falloff   (colour ROM read issued from p8_addr; rom_q = P9)
            if f_rlt2 = '1' then p9_covc <= to_unsigned(16, 5); else p9_covc <= sat_cov(p8_dl, f_shl); end if;
            if f_rhlt2 = '1' then p9_covch <= to_unsigned(16, 5); else p9_covch <= sat_cov(p8_dh, f_shh); end if;
            p9_ra  <= sat_cov(p8_dg, f_shg);
            p9_rb  <= sat_cov(p8_dgt, f_shg);
            p9_shl <= sat_shadow(p8_dsl, f_shsl);
            p9_shh <= sat_shadow(p8_dsh, f_shsh);
            p9_in <= p8_in; p9_inh <= p8_inh; p9_pri <= p8_pri;

            -- P10: coverage combine, shadow total   (rom_q2 = P10)
            if p9_in = '1' then p10_covl <= umin(p9_covc, lc_covy); else p10_covl <= (others => '0'); end if;
            if p9_inh = '1' then p10_covh <= p9_covch; else p10_covh <= (others => '0'); end if;
            if p9_inh = '1' and p9_covch /= 0 then p10_ring <= (others => '0');
            else p10_ring <= umin(p9_ra, p9_rb); end if;
            if resize(p9_shl, 8) + resize(p9_shh, 8) > 64 then p10_shbg <= to_unsigned(64, 7);
            else p10_shbg <= p9_shl + p9_shh; end if;
            p10_pri <= p9_pri;
            -- colour holds: the port served the hero or the list for this pixel
            if p9_pri = '1' then hold_h <= rom_q2; else hold_l <= rom_q2; end if;

            -- P11: list coverage x dim, ring x alpha, backdrop shadow darken
            v_q := resize(p10_covl * l_dim, 12); p11_covl <= v_q(10 downto 6);
            v_q := resize(p10_ring * f_ringa, 12); p11_ring <= v_q(8 downto 4);
            p11_covh <= p10_covh;
            v_p := resize(signed('0' & (r_y - C_BLK)) * signed(resize(p10_shbg, 9)), 18);
            p11_bgy <= r_y - unsigned(v_p(15 downto 6));

            -- P12: layer select -> (over - under), under, cov
            if p11_covl = 16 then v_listfull := '1'; else v_listfull := '0'; end if;
            if p11_covh /= 0 then
                v_oy := unsigned(hold_h(31 downto 22)); v_ou := unsigned(hold_h(21 downto 12)); v_ov := unsigned(hold_h(11 downto 2));
                v_cov := p11_covh;
            elsif p11_ring /= 0 then
                v_oy := f_ringy; v_ou := C_MID; v_ov := C_MID;
                v_cov := p11_ring;
            else
                v_oy := unsigned(hold_l(31 downto 22)); v_ou := unsigned(hold_l(21 downto 12)); v_ov := unsigned(hold_l(11 downto 2));
                v_cov := p11_covl;
                v_listfull := '0';
            end if;
            if v_listfull = '1' then
                v_uy := unsigned(hold_l(31 downto 22)); v_uu := unsigned(hold_l(21 downto 12)); v_uv := unsigned(hold_l(11 downto 2));
            else
                v_uy := p11_bgy; v_uu := unsigned(f_tintu(9 downto 0)); v_uv := unsigned(f_tintv(9 downto 0));
            end if;
            p12_dy <= signed(resize(v_oy, 12)) - signed(resize(v_uy, 12));
            p12_du <= signed(resize(v_ou, 12)) - signed(resize(v_uu, 12));
            p12_dv <= signed(resize(v_ov, 12)) - signed(resize(v_uv, 12));
            p12_uy <= v_uy; p12_uu <= v_uu; p12_uv <= v_uv;
            p12_cov <= v_cov;

            -- P13: blend
            v_p := resize(p12_dy * signed('0' & p12_cov), 18); p13_y <= signed(resize(p12_uy, 12)) + resize(shift_right(v_p, 4), 12);
            v_p := resize(p12_du * signed('0' & p12_cov), 18); p13_u <= signed(resize(p12_uu, 12)) + resize(shift_right(v_p, 4), 12);
            v_p := resize(p12_dv * signed('0' & p12_cov), 18); p13_v <= signed(resize(p12_uv, 12)) + resize(shift_right(v_p, 4), 12);

            -- s_io: blanking gate, clamp, U/V swap for the hardware
            if gv_pipe(LATENCY - 1) = '0' then
                s_io.y <= std_logic_vector(C_BLK);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            else
                if p13_y < 64 then s_io.y <= std_logic_vector(C_BLK);
                elsif p13_y > 940 then s_io.y <= std_logic_vector(to_unsigned(940, 10));
                else s_io.y <= std_logic_vector(p13_y(9 downto 0)); end if;
                if p13_v < 64 then s_io.u <= std_logic_vector(C_BLK);
                elsif p13_v > 960 then s_io.u <= std_logic_vector(to_unsigned(960, 10));
                else s_io.u <= std_logic_vector(p13_v(9 downto 0)); end if;
                if p13_u < 64 then s_io.v <= std_logic_vector(C_BLK);
                elsif p13_u > 960 then s_io.v <= std_logic_vector(to_unsigned(960, 10));
                else s_io.v <= std_logic_vector(p13_u(9 downto 0)); end if;
            end if;
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= gv_pipe(LATENCY - 1);
            s_io.field_n <= pipe(LATENCY - 1).field_n;
        end if;
    end process p_pipe;

    s_io_align_in <= s_io;
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
            for i in 1 to PAD - 1 loop s_io_align_pad(i) <= s_io_align_pad(i - 1); end loop;
        end if;
    end process;
    data_out <= s_io_align_pad(PAD - 1);

end architecture gamut;
