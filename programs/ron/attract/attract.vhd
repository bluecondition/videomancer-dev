-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- ATTRACT (v0.1.0) -- an arcade that plays itself.
--
-- The "DODECA board": a 1981-style arcade board in one program.
--   * 256x240 canvas of square virtual pixels, scaled by a whole number in
--     every mode (4x4 at 1080, 3x3 at 720p, 2 wide x 1 field line at SD) and
--     centred.  Both interlaced fields draw the same line (no twitter).
--   * Tile layer: 32x32 tilemap (tile 8 bits + palette 2 + H/V flip),
--     per-row X scroll, playfield Y scroll.
--   * Sprite layer: 64-entry double-buffered sprite table, up to 16 sprites
--     per line rendered one virtual line ahead into a line buffer (first
--     writer wins = lower table index on top; overflow drops the rest).
--   * One shared pattern ROM (8x8, 2bpp) feeds both layers.
--   * Colour: 2bpp + palette -> 32-entry palette RAM of 3-3-2 colours ->
--     YUV (BT.601, studio range, white 768), U/V swapped for the hardware.
--   * Game CPU: 16-bit accumulator machine (tools/dasm.py is the ISA
--     reference), program in EBR, ticks once per field (twice per frame on
--     1080p's 24-30 Hz, so the games keep arcade speed).
--   * Output conditioning for HD: [1 2 1] smoothing (the HD HDMI bar fix);
--     bypassed at SD at the same latency.
--
-- Clocking: every stage registers every clock; a virtual pixel's values
-- simply hold for N clocks.  Shared EBR read ports (tilemap: render/CPU,
-- pattern ROM: tile layer/sprite engine, palette RAM: render/row scroll)
-- give the second user any clock except the first clock of a new render
-- address, and the render capture holds through a borrowed clock.
--
-- Latency: g_avid -> data_out = data_in syncs -> data_out = C_PIPE (20).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.attract_pkg.all;

architecture attract of program_top is
    constant C_PIPE : integer := 20;
    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLK  : unsigned(9 downto 0) := to_unsigned(64, 10);

    attribute ram_style   : string;
    attribute no_rw_check : boolean;

    -- ------------------------------------------------------------------ timing
    signal s_timing        : t_video_timing_port;
    signal s_h_pixel_counter : unsigned(11 downto 0) := (others => '0');
    signal s_v_line_counter  : unsigned(11 downto 0) := (others => '0');
    signal s_measured_h    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_measured_v    : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal s_firstline     : std_logic := '1';
    signal s_saw, s_ftick  : std_logic := '0';

    -- generated avid (oubliette / aurora / cubist recipe)
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_avd : std_logic := '0';
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_pc  : unsigned(3 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal s_gav_start : std_logic;

    -- mode: N = clocks per virtual pixel, L = output lines per virtual line
    signal s_nm1   : unsigned(1 downto 0) := "11";
    signal s_lm1   : unsigned(1 downto 0) := "11";
    signal s_x0    : unsigned(11 downto 0) := to_unsigned(448, 12);
    signal s_vc    : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_ydf   : signed(12 downto 0) := (others => '0');
    signal s_y0    : unsigned(11 downto 0) := to_unsigned(60, 12);
    signal s_two   : std_logic := '1';
    signal s_hd    : std_logic := '1';
    signal s_half  : unsigned(11 downto 0) := to_unsigned(540, 12);

    -- line-level
    signal s_aline : unsigned(11 downto 0) := (others => '0');
    signal s_vy    : unsigned(7 downto 0) := to_unsigned(239, 8);
    signal s_vsub  : unsigned(1 downto 0) := (others => '0');
    signal s_inv   : std_logic := '0';
    signal s_vlast : std_logic := '0';
    signal s_front : std_logic := '1';
    signal s_gs1, s_gs2, s_gs3, s_gs4 : std_logic := '0';
    signal s_ty    : unsigned(7 downto 0) := (others => '0');
    signal s_sy    : unsigned(7 downto 0) := (others => '0');
    signal s_rs    : unsigned(7 downto 0) := (others => '0');
    signal s_mtick : std_logic := '0';

    -- --------------------------------------------------------- pixel pipeline
    signal s_hc    : unsigned(11 downto 0) := (others => '0');
    signal s0_vx   : unsigned(7 downto 0) := (others => '0');
    signal s0_ph   : unsigned(1 downto 0) := (others => '0');
    signal s0_inh, s0_av : std_logic := '0';
    signal s1_vx, s1_tx : unsigned(7 downto 0) := (others => '0');
    signal s1_ph   : unsigned(1 downto 0) := (others => '0');
    signal s1_inh, s1_av : std_logic := '0';
    signal s1_h    : unsigned(15 downto 0) := (others => '0');
    signal s2_ta   : unsigned(9 downto 0) := (others => '0');
    signal s2_tx3  : unsigned(2 downto 0) := (others => '0');
    signal s2_ph   : unsigned(1 downto 0) := (others => '0');
    signal s2_inh, s2_av : std_logic := '0';
    signal s2_h    : unsigned(15 downto 0) := (others => '0');
    signal s3_tx3  : unsigned(2 downto 0) := (others => '0');
    signal s3_ph   : unsigned(1 downto 0) := (others => '0');
    signal s3_inh, s3_av : std_logic := '0';
    signal s3_h    : unsigned(15 downto 0) := (others => '0');
    signal s3_lb   : std_logic_vector(4 downto 0) := (others => '0');
    signal s4_tile : std_logic_vector(7 downto 0) := (others => '0');
    signal s4_attr : std_logic_vector(3 downto 0) := (others => '0');
    signal s4_tx3  : unsigned(2 downto 0) := (others => '0');
    signal s4_ph   : unsigned(1 downto 0) := (others => '0');
    signal s4_inh, s4_av : std_logic := '0';
    signal s4_h    : unsigned(15 downto 0) := (others => '0');
    signal s4_lb   : std_logic_vector(4 downto 0) := (others => '0');
    signal s5_pa   : unsigned(11 downto 0) := (others => '0');
    signal s5_ph   : unsigned(1 downto 0) := (others => '0');
    signal s5_inh, s5_av, s5_hf : std_logic := '0';
    signal s5_pal  : std_logic_vector(1 downto 0) := (others => '0');
    signal s5_tx3  : unsigned(2 downto 0) := (others => '0');
    signal s5_star : std_logic_vector(1 downto 0) := (others => '0');
    signal s5_lb   : std_logic_vector(4 downto 0) := (others => '0');
    type t_c6 is record
        inh, av, hf : std_logic;
        pal, star   : std_logic_vector(1 downto 0);
        tx3         : unsigned(2 downto 0);
        lb          : std_logic_vector(4 downto 0);
    end record;
    constant C_C6_0 : t_c6 := ('0', '0', '0', "00", "00", "000", "00000");
    signal s6, s7, s8 : t_c6 := C_C6_0;
    signal s8_word : std_logic_vector(15 downto 0) := (others => '0');
    signal s9_idx  : std_logic_vector(5 downto 0) := (others => '0');
    signal s9_bord, s9_av : std_logic := '0';
    signal s10_bord, s10_av, s11_bord, s11_av : std_logic := '0';
    signal s12_c   : std_logic_vector(7 downto 0) := (others => '0');
    signal s12_av  : std_logic := '0';
    signal s13_yr, s13_yg, s13_yb, s13_ur, s13_ug, s13_ub,
           s13_vr, s13_vg, s13_vb : integer range -1024 to 1023 := 0;
    signal s13_av  : std_logic := '0';
    signal s14_y, s14_u, s14_v : integer range -2048 to 2047 := 0;
    signal s14_yb, s14_ub, s14_vb : integer range -1024 to 1023 := 0;
    signal s14_av  : std_logic := '0';
    signal s15_y, s15_u, s15_v : unsigned(9 downto 0) := (others => '0');
    signal s15_av  : std_logic := '0';
    signal g1_y, g1_u, g1_v, g2_y, g2_u, g2_v : unsigned(9 downto 0) := (others => '0');
    signal g1_av   : std_logic := '0';
    signal s16_y, s16_u, s16_v : unsigned(9 downto 0) := (others => '0');
    signal s16_av  : std_logic := '0';
    signal s17_y, s17_u, s17_v : unsigned(9 downto 0) := (others => '0');
    signal s17_av  : std_logic := '0';
    signal s18_y, s18_u, s18_v : unsigned(9 downto 0) := (others => '0');
    signal s18_av  : std_logic := '0';
    type t_syn is array (0 to C_PIPE - 2) of std_logic_vector(2 downto 0);
    signal s_syn   : t_syn := (others => (others => '1'));

    -- --------------------------------------------------------------- memories
    -- tilemap: index (1024x8) + attributes (1024x4: vflip hflip pal1 pal0)
    type t_tidx is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_tatt is array (0 to 1023) of std_logic_vector(3 downto 0);
    signal r_tidx : t_tidx := (others => (others => '0'));
    signal r_tatt : t_tatt := (others => (others => '0'));
    attribute ram_style of r_tidx : signal is "block";
    attribute ram_style of r_tatt : signal is "block";
    attribute no_rw_check of r_tidx : signal is true;
    attribute no_rw_check of r_tatt : signal is true;
    signal s_tq_i  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_tq_a  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ta_rd : unsigned(9 downto 0);
    signal s_tsteal, s_tsteal_d : std_logic := '0';
    signal s_twe   : std_logic := '0';
    signal s_twa   : unsigned(9 downto 0) := (others => '0');
    signal s_twd   : std_logic_vector(11 downto 0) := (others => '0');

    -- control RAM: 0..15 tile palettes, 32..63 sprite palettes (3-3-2),
    -- 64..95 row X scroll
    type t_ctl is array (0 to 127) of std_logic_vector(7 downto 0);
    signal r_ctl : t_ctl := (others => (others => '0'));
    attribute ram_style of r_ctl : signal is "block";
    attribute no_rw_check of r_ctl : signal is true;
    signal s_cq    : std_logic_vector(7 downto 0) := (others => '0');
    signal s_ca_rd : unsigned(6 downto 0);
    signal s_cwe   : std_logic := '0';
    signal s_cwa   : unsigned(6 downto 0) := (others => '0');
    signal s_cwd   : std_logic_vector(7 downto 0) := (others => '0');
    signal s10_ca  : unsigned(6 downto 0) := (others => '0');
    signal s_rsrd, s_rsrd1 : std_logic := '0';

    -- sprite table: {bank, entry(5:0), word} x 16
    type t_obj is array (0 to 255) of std_logic_vector(15 downto 0);
    signal r_obj : t_obj := (others => x"F000");
    attribute ram_style of r_obj : signal is "block";
    attribute no_rw_check of r_obj : signal is true;
    signal s_oq    : std_logic_vector(15 downto 0) := (others => '0');
    signal s_ora   : unsigned(7 downto 0) := (others => '0');
    signal s_owe   : std_logic := '0';
    signal s_owa   : unsigned(7 downto 0) := (others => '0');
    signal s_owd   : std_logic_vector(15 downto 0) := (others => '0');
    signal s_ofront : std_logic := '0';

    -- sprite line buffers (one EBR per bank; display reads/clears the front,
    -- the engine read-modify-writes the back)
    type t_lb is array (0 to 255) of std_logic_vector(4 downto 0);
    signal r_lb0, r_lb1 : t_lb := (others => (others => '0'));
    attribute ram_style of r_lb0 : signal is "block";
    attribute ram_style of r_lb1 : signal is "block";
    attribute no_rw_check of r_lb0 : signal is true;
    attribute no_rw_check of r_lb1 : signal is true;
    signal s_lq0, s_lq1 : std_logic_vector(4 downto 0) := (others => '0');
    signal s_lra0, s_lra1, s_lwa0, s_lwa1 : unsigned(7 downto 0);
    signal s_lwe0, s_lwe1 : std_logic;
    signal s_lwd0, s_lwd1 : std_logic_vector(4 downto 0);
    signal s_clr_we : std_logic := '0';
    signal s_clr_wa : unsigned(7 downto 0) := (others => '0');

    -- pattern ROM (shared)
    signal s_pa    : unsigned(11 downto 0);
    signal s_pq    : std_logic_vector(15 downto 0);
    signal s_psteal, s_psteal_d1, s_psteal_d2 : std_logic := '0';

    -- ---------------------------------------------------------- sprite engine
    type t_est is (E_IDLE, E_SCAN, E_FETCH, E_FWAIT, E_DRAW, E_DRAIN);
    signal e_st    : t_est := E_IDLE;
    signal e_go    : std_logic := '0';
    signal e_line  : unsigned(7 downto 0) := (others => '0');
    signal e_si    : unsigned(5 downto 0) := (others => '0');
    signal e_sw    : std_logic := '0';
    signal e_tok, e_tok1, e_tok2 : std_logic := '0';
    signal e_tw, e_tw1, e_tw2 : std_logic := '0';
    signal e_ti, e_ti1, e_ti2 : unsigned(5 downto 0) := (others => '0');
    signal e_ocap  : std_logic_vector(15 downto 0) := (others => '0');
    signal e_w0    : std_logic_vector(15 downto 0) := (others => '0');
    signal e_ev    : std_logic := '0';
    signal e_evi   : unsigned(5 downto 0) := (others => '0');
    signal e_evw0, e_evw1 : std_logic_vector(15 downto 0) := (others => '0');
    signal e_dy    : unsigned(7 downto 0) := (others => '0');
    signal e_hits  : unsigned(4 downto 0) := (others => '0');
    signal e_x     : unsigned(8 downto 0) := (others => '0');
    signal e_attr  : std_logic_vector(15 downto 0) := (others => '0');
    signal e_rv    : unsigned(3 downto 0) := (others => '0');
    signal e_preq  : std_logic := '0';
    signal e_pa    : unsigned(11 downto 0) := (others => '0');
    signal e_pn    : unsigned(1 downto 0) := (others => '0');
    signal e_ptok1, e_ptok2 : std_logic := '0';
    signal e_pslot1, e_pslot2 : std_logic := '0';
    signal e_got   : unsigned(1 downto 0) := (others => '0');
    signal e_wl, e_wr : std_logic_vector(15 downto 0) := (others => '0');
    signal e_row   : std_logic_vector(31 downto 0) := (others => '0');
    signal e_cnt   : unsigned(4 downto 0) := (others => '0');
    signal e_col   : unsigned(8 downto 0) := (others => '0');
    signal e_dw    : unsigned(2 downto 0) := (others => '0');
    signal e_iss, e_last : std_logic := '0';
    -- RMW pipe
    signal e_ra    : unsigned(7 downto 0) := (others => '0');
    signal m1_v, m2_v, m3_v : std_logic := '0';
    signal m1_a, m2_a, m3_a : unsigned(7 downto 0) := (others => '0');
    signal m1_p, m2_p, m3_p : std_logic_vector(4 downto 0) := (others => '0');
    signal m3_old  : std_logic_vector(4 downto 0) := (others => '0');
    signal e_we    : std_logic := '0';
    signal e_wa    : unsigned(7 downto 0) := (others => '0');
    signal e_wd    : std_logic_vector(4 downto 0) := (others => '0');
    signal e_l0go  : std_logic := '0';
    signal e_l0cnt : unsigned(8 downto 0) := (others => '0');

    -- ------------------------------------------------------------------- CPU
    type t_cst is (CS_FETCH, CS_IR, CS_DEC, CS_MEM, CS_MD, CS_LDC, CS_LDC2, CS_IN,
                   CS_EXEC, CS_WAIT);
    signal c_st    : t_cst := CS_FETCH;
    signal c_pc    : unsigned(11 downto 0) := (others => '0');
    signal c_bt    : unsigned(11 downto 0) := (others => '0');
    signal c_ir    : std_logic_vector(15 downto 0) := (others => '0');
    signal c_a, c_x, c_b, c_r : unsigned(15 downto 0) := (others => '0');
    signal c_isel  : unsigned(3 downto 0) := (others => '0');
    signal c_rwa   : unsigned(7 downto 0) := (others => '0');
    signal c_rwd   : unsigned(15 downto 0) := (others => '0');
    signal c_z, c_n, c_c, c_zn : std_logic := '0';
    signal c_ea    : unsigned(11 downto 0) := (others => '0');
    signal c_code_a : unsigned(11 downto 0);
    signal c_code_q : std_logic_vector(15 downto 0);
    type t_stk is array (0 to 7) of unsigned(11 downto 0);
    signal c_stk   : t_stk := (others => (others => '0'));
    signal c_sp    : unsigned(2 downto 0) := (others => '0');
    type t_ram is array (0 to 255) of std_logic_vector(15 downto 0);
    signal r_ram   : t_ram := (others => (others => '0'));
    attribute ram_style of r_ram : signal is "block";
    attribute no_rw_check of r_ram : signal is true;
    signal c_ramq  : std_logic_vector(15 downto 0) := (others => '0');
    signal c_rwe   : std_logic := '0';
    -- I/O
    signal o_stb   : std_logic := '0';
    signal o_port  : unsigned(3 downto 0) := (others => '0');
    signal o_dat   : unsigned(15 downto 0) := (others => '0');
    signal t_addr  : unsigned(9 downto 0) := (others => '0');
    signal t_raddr : unsigned(9 downto 0) := (others => '0');
    signal t_req, t_rdy : std_logic := '0';
    signal t_rdat  : std_logic_vector(11 downto 0) := (others => '0');
    signal c_caddr : unsigned(6 downto 0) := (others => '0');
    signal c_sidx  : unsigned(5 downto 0) := (others => '0');
    signal c_yscr  : unsigned(7 downto 0) := (others => '0');
    signal c_star  : unsigned(9 downto 0) := (others => '0');
    signal c_rng   : unsigned(15 downto 0) := x"0001";
    signal c_frame : unsigned(15 downto 0) := (others => '0');
    signal c_swreq : std_logic := '0';
    signal c_wait, c_go : std_logic := '0';
    signal k_clr   : std_logic := '0';
    signal k_ci    : unsigned(5 downto 0) := (others => '0');
    signal k_tick  : std_logic := '0';

    -- star hash helpers
    -- xorshift16 (7, 9, 8): tools/dasm.py lfsr_step
    function lfsr_next(s : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := s xor shift_left(s, 7);
        v := v xor shift_right(v, 9);
        v := v xor shift_left(v, 8);
        return v;
    end function;

    function pix8(w : std_logic_vector(15 downto 0); c : unsigned(2 downto 0))
        return std_logic_vector is
    begin
        case c is
            when "000" => return w(15 downto 14);
            when "001" => return w(13 downto 12);
            when "010" => return w(11 downto 10);
            when "011" => return w(9 downto 8);
            when "100" => return w(7 downto 6);
            when "101" => return w(5 downto 4);
            when "110" => return w(3 downto 2);
            when others => return w(1 downto 0);
        end case;
    end function;

    function rev8(w : std_logic_vector(15 downto 0)) return std_logic_vector is
        variable r : std_logic_vector(15 downto 0);
    begin
        for i in 0 to 7 loop
            r(2 * i + 1 downto 2 * i) := w(15 - 2 * i downto 14 - 2 * i);
        end loop;
        return r;
    end function;

begin
    --------------------------------------------------------------------------
    -- Timing
    --------------------------------------------------------------------------
    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_h_pixel_counter > 0 then s_measured_h <= s_h_pixel_counter; end if;
                s_h_pixel_counter <= (others => '0');
            elsif s_timing.avid = '1' then
                s_h_pixel_counter <= s_h_pixel_counter + 1;
            end if;
            if s_timing.vsync_start = '1' then
                if s_v_line_counter > 0 then s_measured_v <= s_v_line_counter; end if;
                s_v_line_counter <= (others => '0');
            elsif s_timing.avid_start = '1' then
                s_v_line_counter <= s_v_line_counter + 1;
            end if;
        end if;
    end process;

    -- frame tick: first vsync edge after active video (saw-active guard)
    p_field : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_firstline <= '1';
            elsif s_gav_start = '1' then
                s_firstline <= '0';
            end if;
            s_ftick <= '0';
            if s_timing.avid = '1' then
                s_saw <= '1';
            elsif s_timing.vsync_start = '1' and s_saw = '1' then
                s_saw   <= '0';
                s_ftick <= '1';
            end if;
            if s_ftick = '1' then
                g_wm1 <= s_measured_h - 1;
            end if;
        end if;
    end process;

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
            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;
            g_avd <= g_avid;
        end if;
    end process;
    s_gav_start <= g_avid and not g_avd;

    -- mode from the measured raster (latched per field, then derived
    -- free-running one op per register)
    p_mode : process(clk)
    begin
        if rising_edge(clk) then
            if s_ftick = '1' then
                if s_measured_h >= 1600 then
                    s_nm1 <= "11"; s_x0 <= to_unsigned(448, 12); s_hd <= '1';
                elsif s_measured_h >= 1000 then
                    s_nm1 <= "10"; s_x0 <= to_unsigned(256, 12); s_hd <= '1';
                else
                    s_nm1 <= "01"; s_x0 <= to_unsigned(104, 12); s_hd <= '0';
                end if;
                if s_measured_v >= 1000 then
                    s_lm1 <= "11"; s_vc <= to_unsigned(960, 12); s_two <= '1';
                elsif s_measured_v >= 650 then
                    s_lm1 <= "10"; s_vc <= to_unsigned(720, 12); s_two <= '0';
                elsif s_measured_v >= 400 then
                    s_lm1 <= "01"; s_vc <= to_unsigned(480, 12); s_two <= '0';
                else
                    s_lm1 <= "00"; s_vc <= to_unsigned(240, 12); s_two <= '0';
                end if;
                s_half <= '0' & s_measured_v(11 downto 1);
            end if;
            s_ydf <= signed('0' & s_measured_v) - signed('0' & s_vc);
            if s_ydf(12) = '1' then
                s_y0 <= (others => '0');
            else
                s_y0 <= unsigned(s_ydf(12 downto 1));
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Line-level: virtual line counter, row scroll, sprite engine trigger
    --------------------------------------------------------------------------
    p_line : process(clk)
        variable v_al : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_gs1 <= s_gav_start;
            s_gs2 <= s_gs1;
            s_gs3 <= s_gs2;
            s_gs4 <= s_gs3;
            s_mtick <= '0';
            if s_gav_start = '1' then
                if s_firstline = '1' then
                    v_al := (others => '0');
                    s_inv <= '0';
                else
                    v_al := s_aline + 1;
                end if;
                s_aline <= v_al;
                if v_al = s_half and s_two = '1' then
                    s_mtick <= '1';
                end if;
                if v_al = s_y0 then
                    s_vy <= (others => '0');
                    s_vsub <= (others => '0');
                    s_inv <= '1';
                elsif s_inv = '1' then
                    if s_vsub = s_lm1 then
                        s_vsub <= (others => '0');
                        if s_vy = 239 then
                            s_inv <= '0';
                        else
                            s_vy <= s_vy + 1;
                        end if;
                    else
                        s_vsub <= s_vsub + 1;
                    end if;
                end if;
            end if;
            if s_gs1 = '1' then
                s_front <= s_vy(0);
                if s_vsub = s_lm1 then s_vlast <= '1'; else s_vlast <= '0'; end if;
                if s_vy >= 16 and s_vy < 232 then
                    s_ty <= s_vy + c_yscr;
                else
                    s_ty <= s_vy;
                end if;
                s_sy <= s_vy + c_star(7 downto 0);
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel pipeline
    --------------------------------------------------------------------------
    p_s0 : process(clk)
        variable v_hc : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s0_av <= g_avid;
            if s_gav_start = '1' then
                v_hc := (others => '0');
            else
                v_hc := s_hc + 1;
            end if;
            s_hc <= v_hc;
            if g_avid = '1' and v_hc = s_x0 then
                s0_vx  <= (others => '0');
                s0_ph  <= (others => '0');
                s0_inh <= '1';
            elsif s0_inh = '1' then
                if s0_ph = s_nm1 then
                    s0_ph <= (others => '0');
                    s0_vx <= s0_vx + 1;
                    if s0_vx = 255 then
                        s0_inh <= '0';
                    end if;
                else
                    s0_ph <= s0_ph + 1;
                end if;
            else
                s0_ph <= (others => '0');
            end if;
        end if;
    end process;

    p_s1 : process(clk)
        variable v_key : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            s1_vx  <= s0_vx;
            s1_tx  <= s0_vx + s_rs;
            s1_ph  <= s0_ph;
            s1_inh <= s0_inh;
            s1_av  <= s0_av;
            v_key := s_sy & s0_vx;
            s1_h  <= v_key xor shift_right(v_key, 7);
            -- clear-behind-beam: when a new pixel enters S1, zero the one
            -- the display just finished reading (last output line only)
            if s0_ph = 0 and s1_inh = '1' and s_vlast = '1' then
                s_clr_we <= '1';
                s_clr_wa <= s1_vx;
            else
                s_clr_we <= '0';
            end if;
        end if;
    end process;

    p_s2 : process(clk)
    begin
        if rising_edge(clk) then
            s2_ta  <= s_ty(7 downto 3) & s1_tx(7 downto 3);
            s2_tx3 <= s1_tx(2 downto 0);
            s2_ph  <= s1_ph;
            s2_inh <= s1_inh;
            s2_av  <= s1_av;
            s2_h   <= s1_h + shift_left(s1_h, 5);
        end if;
    end process;

    p_s3 : process(clk)
    begin
        if rising_edge(clk) then
            s3_tx3 <= s2_tx3;
            s3_ph  <= s2_ph;
            s3_inh <= s2_inh;
            s3_av  <= s2_av;
            s3_h   <= s2_h xor shift_right(s2_h, 9);
            if s_front = '0' then s3_lb <= s_lq0; else s3_lb <= s_lq1; end if;
        end if;
    end process;

    p_s4 : process(clk)
    begin
        if rising_edge(clk) then
            if s_tsteal_d = '0' then
                s4_tile <= s_tq_i;
                s4_attr <= s_tq_a;
            end if;
            s4_tx3 <= s3_tx3;
            s4_ph  <= s3_ph;
            s4_inh <= s3_inh;
            s4_av  <= s3_av;
            s4_h   <= s3_h + shift_left(s3_h, 3);
            s4_lb  <= s3_lb;
        end if;
    end process;

    p_s5 : process(clk)
        variable v_row : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            v_row := s_ty(2 downto 0);
            if s4_attr(3) = '1' then v_row := not v_row; end if;
            s5_pa  <= '0' & unsigned(s4_tile) & v_row;
            s5_hf  <= s4_attr(2);
            s5_pal <= s4_attr(1 downto 0);
            s5_tx3 <= s4_tx3;
            s5_ph  <= s4_ph;
            s5_inh <= s4_inh;
            s5_av  <= s4_av;
            s5_lb  <= s4_lb;
            s5_star <= "00";
            if c_star(8) = '1' and s4_h(15 downto 9) = "0000000" and s4_h(1 downto 0) /= "00" then
                if not (c_star(9) = '1' and s4_h(2) = '1' and c_frame(4) = '1') then
                    s5_star <= std_logic_vector(s4_h(1 downto 0));
                end if;
            end if;
        end if;
    end process;

    p_s678 : process(clk)
    begin
        if rising_edge(clk) then
            s6 <= (inh => s5_inh, av => s5_av, hf => s5_hf, pal => s5_pal,
                   star => s5_star, tx3 => s5_tx3, lb => s5_lb);
            s7 <= s6;
            s8 <= s7;
            if s_psteal_d2 = '0' then
                s8_word <= s_pq;
            end if;
        end if;
    end process;

    p_s9 : process(clk)
        variable v_c  : unsigned(2 downto 0);
        variable v_tp : std_logic_vector(1 downto 0);
    begin
        if rising_edge(clk) then
            v_c := s8.tx3;
            if s8.hf = '1' then v_c := not v_c; end if;
            v_tp := pix8(s8_word, v_c);
            if s8.lb(1 downto 0) /= "00" then
                s9_idx <= '1' & s8.lb;
            elsif v_tp /= "00" then
                s9_idx <= "00" & s8.pal & v_tp;
            elsif s8.star /= "00" then
                s9_idx <= "00" & s8.star & "00";
            else
                s9_idx <= (others => '0');
            end if;
            s9_bord <= not (s8.inh and s_inv);
            s9_av   <= s8.av;
        end if;
    end process;

    -- S10: palette address (the row-scroll read borrows this port at line
    -- start, outside the canvas); S11: EBR; S12: fabric copy
    p_s10 : process(clk)
    begin
        if rising_edge(clk) then
            s10_ca   <= '0' & unsigned(s9_idx);
            s10_bord <= s9_bord;
            s10_av   <= s9_av;
            s11_bord <= s10_bord;
            s11_av   <= s10_av;
            if s11_bord = '1' then
                s12_c <= (others => '0');
            else
                s12_c <= s_cq;
            end if;
            s12_av <= s11_av;
            -- row scroll: the read address is presented while s_rsrd is
            -- high; the EBR word is on s_cq the next clock (s_rsrd1)
            s_rsrd  <= s_gs2;
            s_rsrd1 <= s_rsrd;
            if s_rsrd1 = '1' then
                s_rs <= unsigned(s_cq);
            end if;
        end if;
    end process;
    s_ca_rd <= ("10" & s_vy(7 downto 3)) when s_rsrd = '1' else s10_ca;

    -- S13..S15: 3-3-2 -> YUV
    p_yuv : process(clk)
        variable r3, g3 : integer range 0 to 7;
        variable b2     : integer range 0 to 3;
        variable vy, vu, vv : integer range -4096 to 4095;
    begin
        if rising_edge(clk) then
            r3 := to_integer(unsigned(s12_c(7 downto 5)));
            g3 := to_integer(unsigned(s12_c(4 downto 2)));
            b2 := to_integer(unsigned(s12_c(1 downto 0)));
            s13_yr <= C_YR(r3); s13_yg <= C_YG(g3); s13_yb <= C_YB(b2);
            s13_ur <= C_UR(r3); s13_ug <= C_UG(g3); s13_ub <= C_UB(b2);
            s13_vr <= C_VR(r3); s13_vg <= C_VG(g3); s13_vb <= C_VB(b2);
            s13_av <= s12_av;
            s14_y <= s13_yr + s13_yg; s14_yb <= s13_yb;
            s14_u <= s13_ur + s13_ug; s14_ub <= s13_ub;
            s14_v <= s13_vr + s13_vg; s14_vb <= s13_vb;
            s14_av <= s13_av;
            vy := s14_y + s14_yb;
            vu := s14_u + s14_ub;
            vv := s14_v + s14_vb;
            s15_y <= to_unsigned(vy, 10);
            s15_u <= to_unsigned(vu, 10);
            s15_v <= to_unsigned(vv, 10);
            s15_av <= s14_av;
        end if;
    end process;

    -- S16: [1 2 1] horizontal smoothing at HD (bar fix), centre tap at SD;
    -- S17: blanking gate + U/V swap; S18: output register
    p_out : process(clk)
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            g1_y <= s15_y; g1_u <= s15_u; g1_v <= s15_v; g1_av <= s15_av;
            g2_y <= g1_y;  g2_u <= g1_u;  g2_v <= g1_v;
            v_sy := resize(g2_y, 12) + shift_left(resize(g1_y, 12), 1) + resize(s15_y, 12);
            v_su := resize(g2_u, 12) + shift_left(resize(g1_u, 12), 1) + resize(s15_u, 12);
            v_sv := resize(g2_v, 12) + shift_left(resize(g1_v, 12), 1) + resize(s15_v, 12);
            if s_hd = '1' then
                s16_y <= v_sy(11 downto 2);
                s16_u <= v_su(11 downto 2);
                s16_v <= v_sv(11 downto 2);
            else
                s16_y <= g1_y;
                s16_u <= g1_u;
                s16_v <= g1_v;
            end if;
            s16_av <= g1_av;
            if s16_av = '0' then
                s17_y <= C_BLK;
                s17_u <= C_MID;
                s17_v <= C_MID;
            else
                s17_y <= s16_y;
                s17_u <= s16_v;     -- hardware U/V swap
                s17_v <= s16_u;
            end if;
            s17_av <= s16_av;
            s18_y <= s17_y; s18_u <= s17_u; s18_v <= s17_v; s18_av <= s17_av;
        end if;
    end process;

    -- source syncs: data_in -> data_out in C_PIPE clocks (s_syn + output reg)
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_syn(0) <= data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            for i in 1 to C_PIPE - 2 loop
                s_syn(i) <= s_syn(i - 1);
            end loop;
            data_out.hsync_n <= s_syn(C_PIPE - 2)(0);
            data_out.vsync_n <= s_syn(C_PIPE - 2)(1);
            data_out.field_n <= s_syn(C_PIPE - 2)(2);
            data_out.avid    <= s18_av;
            data_out.y       <= std_logic_vector(s18_y);
            data_out.u       <= std_logic_vector(s18_u);
            data_out.v       <= std_logic_vector(s18_v);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Tilemap RAMs: render reads at S2 (address) .. S4 (capture); the CPU
    -- borrows the read port on any clock but a new render address's first.
    --------------------------------------------------------------------------
    s_tsteal <= t_req and not (s2_inh and (s2_ph ?= "00"));
    s_ta_rd  <= t_raddr when s_tsteal = '1' else s2_ta;

    p_tmap : process(clk)
    begin
        if rising_edge(clk) then
            s_tq_i <= r_tidx(to_integer(s_ta_rd));
            s_tq_a <= r_tatt(to_integer(s_ta_rd));
            if s_twe = '1' then
                r_tidx(to_integer(s_twa)) <= s_twd(7 downto 0);
                r_tatt(to_integer(s_twa)) <= s_twd(11 downto 8);
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control RAM (palette + row scroll)
    --------------------------------------------------------------------------
    p_ctl : process(clk)
    begin
        if rising_edge(clk) then
            s_cq <= r_ctl(to_integer(s_ca_rd));
            if s_cwe = '1' then
                r_ctl(to_integer(s_cwa)) <= s_cwd;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pattern ROM: tile layer at S5, sprite engine on borrowed clocks
    --------------------------------------------------------------------------
    s_psteal <= e_preq and not (s5_inh and (s5_ph ?= "00"));
    s_pa     <= e_pa when s_psteal = '1' else s5_pa;

    patrom_inst : entity work.attract_patrom
        port map (clk => clk, addr => s_pa, q => s_pq);

    p_psteal : process(clk)
    begin
        if rising_edge(clk) then
            s_psteal_d1 <= s_psteal;
            s_psteal_d2 <= s_psteal_d1;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Sprite table RAM: engine reads the front bank; CPU / tick clear write
    -- the back bank.
    --------------------------------------------------------------------------
    p_obj : process(clk)
    begin
        if rising_edge(clk) then
            s_oq <= r_obj(to_integer(s_ora));
            if s_owe = '1' then
                r_obj(to_integer(s_owa)) <= s_owd;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Line buffers
    --------------------------------------------------------------------------
    s_lra0 <= s1_vx when s_front = '0' else e_ra;
    s_lra1 <= s1_vx when s_front = '1' else e_ra;
    s_lwe0 <= s_clr_we when s_front = '0' else e_we;
    s_lwe1 <= s_clr_we when s_front = '1' else e_we;
    s_lwa0 <= s_clr_wa when s_front = '0' else e_wa;
    s_lwa1 <= s_clr_wa when s_front = '1' else e_wa;
    s_lwd0 <= "00000" when s_front = '0' else e_wd;
    s_lwd1 <= "00000" when s_front = '1' else e_wd;

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lq0 <= r_lb0(to_integer(s_lra0));
            if s_lwe0 = '1' then
                r_lb0(to_integer(s_lwa0)) <= s_lwd0;
            end if;
            s_lq1 <= r_lb1(to_integer(s_lra1));
            if s_lwe1 = '1' then
                r_lb1(to_integer(s_lwa1)) <= s_lwd1;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Sprite engine.  Renders virtual line e_line into the back line buffer:
    -- scans the 64 entries (word 0 / word 1 streamed one word per clock),
    -- draws each hit (one or two pattern words, then 8/16 pixels at one per
    -- clock, read-modify-write so the first writer = lowest entry wins), at
    -- most 16 sprites per line (the rest are dropped, as on real boards).
    --------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_h  : unsigned(4 downto 0);
        variable v_rv : unsigned(3 downto 0);
        variable v_pn : unsigned(8 downto 0);
        variable v_px : std_logic_vector(1 downto 0);
    begin
        if rising_edge(clk) then
            -- triggers: each virtual line's first output line renders the
            -- next one; line 0 renders in vblank, 256 clocks after the tick
            e_go   <= '0';
            e_l0go <= '0';
            if s_ftick = '1' then
                e_l0cnt <= (others => '1');
            elsif e_l0cnt /= 0 then
                e_l0cnt <= e_l0cnt - 1;
                if e_l0cnt = 1 then e_l0go <= '1'; end if;
            end if;
            if s_gs2 = '1' and s_inv = '1' and s_vsub = 0 and s_vy /= 239 then
                e_go   <= '1';
                e_line <= s_vy + 1;
            elsif e_l0go = '1' then
                e_go   <= '1';
                e_line <= (others => '0');
            end if;

            -- object read stream: address issued with e_tok, word in
            -- e_ocap two clocks later (EBR register + fabric copy)
            e_tok1 <= e_tok; e_tw1 <= e_tw; e_ti1 <= e_ti;
            e_tok2 <= e_tok1; e_tw2 <= e_tw1; e_ti2 <= e_ti1;
            e_ocap <= s_oq;
            e_tok  <= '0';
            e_ev   <= '0';
            if e_tok2 = '1' then
                if e_tw2 = '0' then
                    e_w0 <= e_ocap;
                else
                    e_ev   <= '1';
                    e_evi  <= e_ti2;
                    e_evw0 <= e_w0;
                    e_evw1 <= e_ocap;
                    e_dy   <= e_line - unsigned(e_w0(15 downto 8));
                end if;
            end if;

            -- pattern words come back 2 clocks after the grant
            e_ptok1  <= s_psteal;
            e_pslot1 <= e_pn(0);
            e_ptok2  <= e_ptok1;
            e_pslot2 <= e_pslot1;

            -- RMW pipe: m1 = read issued, m2 = EBR q, m3 = fabric copy
            m2_v <= m1_v; m2_a <= m1_a; m2_p <= m1_p;
            m3_v <= m2_v; m3_a <= m2_a; m3_p <= m2_p;
            if s_front = '0' then m3_old <= s_lq1; else m3_old <= s_lq0; end if;
            e_we <= '0';
            if m3_v = '1' and m3_old(1 downto 0) = "00" then
                e_we <= '1';
                e_wa <= m3_a;
                e_wd <= m3_p;
            end if;
            m1_v <= '0';

            case e_st is
                when E_IDLE =>
                    e_iss <= '0';

                when E_SCAN =>
                    if e_iss = '1' then
                        s_ora <= s_ofront & e_si & e_sw;
                        e_tok <= '1';
                        e_tw  <= e_sw;
                        e_ti  <= e_si;
                        if e_sw = '0' then
                            e_sw <= '1';
                        else
                            e_sw <= '0';
                            if e_si = 63 then
                                e_iss <= '0';
                            else
                                e_si <= e_si + 1;
                            end if;
                        end if;
                    end if;
                    if e_ev = '1' then
                        v_h := "10000";
                        if e_evw1(14) = '0' then v_h := "01000"; end if;
                        if e_dy < v_h then
                            e_hits <= e_hits + 1;
                            e_x    <= e_evw1(15) & unsigned(e_evw0(7 downto 0));
                            e_attr <= e_evw1;
                            if e_evw1(13) = '1' then
                                v_rv := v_h(3 downto 0) - 1 - e_dy(3 downto 0);
                            else
                                v_rv := e_dy(3 downto 0);
                            end if;
                            e_rv   <= v_rv;
                            e_iss  <= '0';
                            e_si   <= e_evi + 1;
                            e_sw   <= '0';
                            if e_evi = 63 then e_last <= '1'; else e_last <= '0'; end if;
                            e_preq <= '0';
                            e_pn   <= "00";
                            e_got  <= "00";
                            e_st   <= E_FETCH;
                        elsif e_evi = 63 then
                            e_st <= E_IDLE;
                        end if;
                    end if;

                when E_FETCH =>
                    if e_preq = '0' then
                        v_pn := unsigned(e_attr(8 downto 0));
                        if e_attr(14) = '1' then
                            v_pn(1 downto 0) := e_rv(3) & e_attr(12);
                        end if;
                        e_pa   <= v_pn & e_rv(2 downto 0);
                        e_preq <= '1';
                        e_pn   <= "00";
                    elsif s_psteal = '1' then
                        if e_attr(14) = '1' and e_pn = "00" then
                            v_pn := unsigned(e_attr(8 downto 0));
                            v_pn(1 downto 0) := e_rv(3) & (not e_attr(12));
                            e_pa <= v_pn & e_rv(2 downto 0);
                            e_pn <= "01";
                        else
                            e_preq <= '0';
                            e_st   <= E_FWAIT;
                        end if;
                    end if;

                when E_FWAIT =>
                    if e_ptok2 = '1' then
                        if e_pslot2 = '0' then
                            if e_attr(12) = '1' then e_wl <= rev8(s_pq); else e_wl <= s_pq; end if;
                            e_got(0) <= '1';
                        else
                            if e_attr(12) = '1' then e_wr <= rev8(s_pq); else e_wr <= s_pq; end if;
                            e_got(1) <= '1';
                        end if;
                    end if;
                    if (e_attr(14) = '1' and e_got = "11") or (e_attr(14) = '0' and e_got(0) = '1') then
                        e_row <= e_wl & e_wr;
                        e_col <= e_x;
                        if e_attr(14) = '1' then e_cnt <= to_unsigned(15, 5);
                        else e_cnt <= to_unsigned(7, 5); end if;
                        e_st <= E_DRAW;
                    end if;

                when E_DRAW =>
                    v_px := e_row(31 downto 30);
                    e_row <= e_row(29 downto 0) & "00";
                    e_ra  <= e_col(7 downto 0);
                    if v_px /= "00" and e_col(8) = '0' then
                        m1_v <= '1';
                    end if;
                    m1_a  <= e_col(7 downto 0);
                    m1_p  <= e_attr(9) & e_attr(11 downto 10) & v_px;
                    e_col <= e_col + 1;
                    e_cnt <= e_cnt - 1;
                    if e_cnt = 0 then
                        e_dw <= (others => '0');
                        e_st <= E_DRAIN;
                    end if;

                when E_DRAIN =>
                    -- the RMW pipe empties before another sprite touches the
                    -- same columns; then resume the scan after the hit
                    e_dw <= e_dw + 1;
                    if e_dw = 4 then
                        if e_hits = 16 or e_last = '1' then
                            e_st <= E_IDLE;
                        else
                            e_st  <= E_SCAN;
                            e_iss <= '1';
                        end if;
                    end if;
            end case;

            if e_go = '1' then
                e_st   <= E_SCAN;
                e_iss  <= '1';
                e_si   <= (others => '0');
                e_sw   <= '0';
                e_hits <= (others => '0');
                e_preq <= '0';
                e_tok  <= '0';
                e_last <= '0';
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Game CPU (tools/dasm.py is the reference).  FETCH -> IR -> DEC ->
    -- [MEM -> MD | LDC -> LDC2 | IN] -> EXEC.  The operand register c_b is
    -- loaded with the immediate (DEC), RAM word (MD), ROM word (LDC2) or
    -- port value (IN), so EXEC has one adder and one subtractor.
    --------------------------------------------------------------------------
    c_code_a <= c_ea when c_st = CS_LDC else c_pc;
    coderom_inst : entity work.attract_coderom
        port map (clk => clk, addr => c_code_a, q => c_code_q);

    p_ram : process(clk)
    begin
        if rising_edge(clk) then
            c_ramq <= r_ram(to_integer(c_ea(7 downto 0)));
            if c_rwe = '1' then
                r_ram(to_integer(c_rwa)) <= std_logic_vector(c_rwd);
            end if;
        end if;
    end process;

    p_cpu : process(clk)
        variable v_op   : integer range 0 to 31;
        variable v_k    : unsigned(10 downto 0);
        variable v_sk   : unsigned(15 downto 0);
        variable v_sum  : unsigned(16 downto 0);
        variable v_dif  : unsigned(16 downto 0);
        variable v_npc  : unsigned(11 downto 0);
        variable v_x1   : unsigned(15 downto 0);
        variable v_mr   : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            c_rwe <= '0';
            o_stb <= '0';
            if c_zn = '1' then
                if c_r = 0 then c_z <= '1'; else c_z <= '0'; end if;
                c_n  <= c_r(15);
                c_zn <= '0';
            end if;
            v_op := to_integer(unsigned(c_ir(15 downto 11)));
            v_k  := unsigned(c_ir(10 downto 0));
            v_sk := unsigned(resize(signed(c_ir(10 downto 0)), 16));

            case c_st is
                when CS_FETCH =>
                    c_st <= CS_IR;

                when CS_IR =>
                    c_ir <= c_code_q;
                    c_st <= CS_DEC;

                when CS_DEC =>
                    -- effective address, immediate operand, branch target
                    if v_op = 15 or v_op = 16 or v_op = 19 then     -- LDX STX LDC
                        c_ea <= c_x(11 downto 0) + resize(v_k, 12);
                    elsif v_op = 0 then                             -- memory-direct
                        c_ea <= resize(v_k(7 downto 0), 12);
                    else
                        c_ea <= resize(v_k, 12);
                    end if;
                    c_bt <= c_pc + 1 + unsigned(resize(signed(c_ir(10 downto 0)), 12));
                    if v_op = 12 or v_op = 13 or v_op = 20 then     -- ANDI ORI XORI
                        c_b <= resize(v_k, 16);
                    else
                        c_b <= v_sk;
                    end if;
                    c_isel <= v_k(3 downto 0);
                    case v_op is
                        when 0 =>
                            if v_k(10 downto 8) = "001" or v_k(10 downto 8) = "010"
                               or v_k(10 downto 8) = "100" then
                                c_st <= CS_MEM;                     -- INC DEC LDXM
                            else
                                c_st <= CS_EXEC;
                            end if;
                        when 3 | 5 | 6 | 7 | 8 | 9 | 10 | 15 => c_st <= CS_MEM;
                        when 19 => c_st <= CS_LDC;
                        when 29 => c_st <= CS_IN;
                        when others => c_st <= CS_EXEC;
                    end case;

                when CS_MEM =>
                    c_st <= CS_MD;

                when CS_MD =>
                    c_b  <= unsigned(c_ramq);
                    c_st <= CS_EXEC;

                when CS_LDC =>
                    c_st <= CS_LDC2;

                when CS_LDC2 =>
                    c_b  <= unsigned(c_code_q);
                    c_st <= CS_EXEC;

                when CS_IN =>
                    case to_integer(c_isel) is
                        when 0 =>
                            c_b <= resize(unsigned(t_rdat), 16);
                        when 1 to 6 =>
                            c_b <= resize(unsigned(registers_in(to_integer(c_isel) - 1)), 16);
                        when 7 =>
                            c_b <= resize(unsigned(registers_in(6)(4 downto 0)), 16);
                        when 8 =>
                            c_b <= resize(unsigned(registers_in(7)), 16);
                        when 9 =>
                            c_b <= lfsr_next(c_rng);
                        when 10 =>
                            c_b <= c_frame;
                        when others =>
                            c_b <= (others => '0');
                    end case;
                    if c_isel /= 0 or (t_rdy = '1' and t_req = '0') then
                        c_st <= CS_EXEC;
                    end if;

                when CS_EXEC =>
                    v_sum := ('0' & c_a) + ('0' & c_b);
                    v_dif := ('0' & c_a) - ('0' & c_b);
                    v_npc := c_pc + 1;
                    v_x1  := c_x - 1;
                    c_st  <= CS_FETCH;
                    case v_op is
                        when 0 =>                                   -- SYS
                          case to_integer(v_k(10 downto 8)) is
                            when 1 | 2 =>                           -- INC DEC m
                                if v_k(9) = '0' then v_mr := c_b + 1; else v_mr := c_b - 1; end if;
                                c_rwe <= '1';
                                c_rwa <= c_ea(7 downto 0);
                                c_rwd <= v_mr;
                                c_a <= v_mr; c_r <= v_mr; c_zn <= '1';
                            when 3 =>                               -- CLR m
                                c_rwe <= '1';
                                c_rwa <= c_ea(7 downto 0);
                                c_rwd <= (others => '0');
                            when 4 =>                               -- LDXM m
                                c_x <= c_b;
                            when 5 =>                               -- STXM m
                                c_rwe <= '1';
                                c_rwa <= c_ea(7 downto 0);
                                c_rwd <= c_x;
                            when others =>
                            case to_integer(v_k(3 downto 0)) is
                                when 1 =>                           -- RET
                                    v_npc := c_stk(to_integer(c_sp - 1));
                                    c_sp  <= c_sp - 1;
                                when 2 =>                           -- WAIT
                                    c_st <= CS_WAIT;
                                when 3 => c_x <= c_a;               -- TAX
                                when 4 => c_a <= c_x; c_r <= c_x; c_zn <= '1';
                                when 5 => c_x <= c_x + 1;
                                when 6 => c_x <= v_x1;
                                when 7 =>                           -- SHL
                                    c_c <= c_a(15);
                                    c_a <= c_a(14 downto 0) & '0';
                                    c_r <= c_a(14 downto 0) & '0'; c_zn <= '1';
                                when 8 =>                           -- SHR
                                    c_c <= c_a(0);
                                    c_a <= '0' & c_a(15 downto 1);
                                    c_r <= '0' & c_a(15 downto 1); c_zn <= '1';
                                when 9 =>                           -- ASR
                                    c_c <= c_a(0);
                                    c_a <= c_a(15) & c_a(15 downto 1);
                                    c_r <= c_a(15) & c_a(15 downto 1); c_zn <= '1';
                                when 10 =>                          -- NOT
                                    c_a <= not c_a; c_r <= not c_a; c_zn <= '1';
                                when 11 =>                          -- NEG
                                    c_a <= (not c_a) + 1; c_r <= (not c_a) + 1; c_zn <= '1';
                                when 12 =>                          -- SWAPB
                                    c_a <= c_a(7 downto 0) & c_a(15 downto 8);
                                    c_r <= c_a(7 downto 0) & c_a(15 downto 8); c_zn <= '1';
                                when 13 =>                          -- SHL4
                                    c_a <= c_a(11 downto 0) & "0000";
                                    c_r <= c_a(11 downto 0) & "0000"; c_zn <= '1';
                                when 14 =>                          -- SHR4
                                    c_a <= "0000" & c_a(15 downto 4);
                                    c_r <= "0000" & c_a(15 downto 4); c_zn <= '1';
                                when 15 =>                          -- CLA
                                    c_a <= (others => '0'); c_r <= (others => '0'); c_zn <= '1';
                                when others => null;
                            end case;
                          end case;
                        when 1 | 3 | 15 | 19 | 29 =>                -- LDI LD LDX LDC IN
                            c_a <= c_b; c_r <= c_b; c_zn <= '1';
                            if v_op = 29 and c_isel = 9 then
                                c_rng <= c_b;
                            end if;
                        when 2 =>                                   -- LDH
                            c_a <= v_k(7 downto 0) & c_a(7 downto 0);
                            c_r <= v_k(7 downto 0) & c_a(7 downto 0); c_zn <= '1';
                        when 4 | 16 =>                              -- ST STX
                            c_rwe <= '1';
                            c_rwa <= c_ea(7 downto 0);
                            c_rwd <= c_a;
                        when 5 | 11 =>                              -- ADD ADDI
                            c_a <= v_sum(15 downto 0); c_r <= v_sum(15 downto 0);
                            c_c <= v_sum(16); c_zn <= '1';
                        when 6 =>                                   -- SUB
                            c_a <= v_dif(15 downto 0); c_r <= v_dif(15 downto 0);
                            c_c <= v_dif(16); c_zn <= '1';
                        when 10 | 14 =>                             -- CMP CMPI
                            c_r <= v_dif(15 downto 0);
                            c_c <= v_dif(16); c_zn <= '1';
                        when 7 | 12 =>                              -- AND ANDI
                            c_a <= c_a and c_b; c_r <= c_a and c_b; c_zn <= '1';
                        when 8 | 13 =>                              -- OR ORI
                            c_a <= c_a or c_b; c_r <= c_a or c_b; c_zn <= '1';
                        when 9 | 20 =>                              -- XOR XORI
                            c_a <= c_a xor c_b; c_r <= c_a xor c_b; c_zn <= '1';
                        when 17 =>                                  -- LXI
                            c_x <= c_b;
                        when 18 =>                                  -- ADX
                            c_x <= c_x + c_b;
                        when 21 =>                                  -- JMP (even)
                            v_npc := v_k & '0';
                        when 22 => if c_z = '1' then v_npc := c_bt; end if;
                        when 23 => if c_z = '0' then v_npc := c_bt; end if;
                        when 24 => if c_c = '1' then v_npc := c_bt; end if;
                        when 25 => if c_c = '0' then v_npc := c_bt; end if;
                        when 26 => if c_n = '1' then v_npc := c_bt; end if;
                        when 27 => if c_n = '0' then v_npc := c_bt; end if;
                        when 28 =>                                  -- CALL (even)
                            c_stk(to_integer(c_sp)) <= v_npc;
                            c_sp  <= c_sp + 1;
                            v_npc := v_k & '0';
                        when 30 =>                                  -- OUT
                            o_stb  <= '1';
                            o_port <= v_k(3 downto 0);
                            o_dat  <= c_a;
                            if v_k(3 downto 0) = 10 then
                                if c_a = 0 then c_rng <= x"0001"; else c_rng <= c_a; end if;
                            end if;
                        when 31 =>                                  -- LOOP
                            c_x <= v_x1;
                            if v_x1 /= 0 then v_npc := c_bt; end if;
                        when others => null;
                    end case;
                    c_pc <= v_npc;

                when CS_WAIT =>
                    if c_go = '1' then
                        c_st <= CS_FETCH;
                    end if;
            end case;
        end if;
    end process;
    c_wait <= '1' when c_st = CS_WAIT else '0';

    --------------------------------------------------------------------------
    -- I/O ports, the tick sequencer and the tilemap read service
    --------------------------------------------------------------------------
    p_io : process(clk)
    begin
        if rising_edge(clk) then
            s_twe <= '0';
            s_cwe <= '0';
            s_owe <= '0';
            c_go  <= '0';
            s_tsteal_d <= s_tsteal;

            -- tilemap read for the CPU: granted on a free clock, data 2 later
            if s_tsteal = '1' then
                t_req <= '0';
            end if;
            if s_tsteal_d = '1' then
                t_rdat <= s_tq_a & s_tq_i;
                t_rdy  <= '1';
            end if;

            -- tick: swap the sprite table at the frame tick when the CPU
            -- finished one; wake a waiting CPU after clearing the back bank
            if s_ftick = '1' and c_swreq = '1' then
                s_ofront <= not s_ofront;
                c_swreq  <= '0';
            end if;
            if (s_ftick = '1' or s_mtick = '1') and c_wait = '1' and k_clr = '0' then
                k_clr <= '1';
                k_ci  <= (others => '0');
            end if;
            if k_clr = '1' then
                s_owe <= '1';
                s_owa <= (not s_ofront) & k_ci & '0';
                s_owd <= x"F000";
                k_ci  <= k_ci + 1;
                if k_ci = 63 then
                    k_clr   <= '0';
                    c_go    <= '1';
                    c_frame <= c_frame + 1;
                    c_swreq <= '0';
                end if;
            end if;

            if o_stb = '1' then
                case to_integer(o_port) is
                    when 0 =>                                       -- TADDR
                        t_addr  <= o_dat(9 downto 0);
                        t_raddr <= o_dat(9 downto 0);
                        t_req   <= '1';
                        t_rdy   <= '0';
                    when 1 =>                                       -- TDATA
                        s_twe  <= '1';
                        s_twa  <= t_addr;
                        s_twd  <= std_logic_vector(o_dat(11 downto 0));
                        t_addr <= t_addr + 1;
                    when 2 => c_caddr <= o_dat(6 downto 0);         -- CADDR
                    when 3 =>                                       -- CDATA
                        s_cwe   <= '1';
                        s_cwa   <= c_caddr;
                        s_cwd   <= std_logic_vector(o_dat(7 downto 0));
                        c_caddr <= c_caddr + 1;
                    when 4 => c_sidx <= o_dat(5 downto 0);          -- SIDX
                    when 5 =>                                       -- SPRY
                        s_owe <= '1';
                        s_owa <= (not s_ofront) & c_sidx & '0';
                        s_owd <= std_logic_vector(o_dat);
                    when 6 =>                                       -- SPRA
                        s_owe  <= '1';
                        s_owa  <= (not s_ofront) & c_sidx & '1';
                        s_owd  <= std_logic_vector(o_dat);
                        c_sidx <= c_sidx + 1;
                    when 7 => c_yscr  <= o_dat(7 downto 0);         -- YSCR
                    when 8 => c_star  <= o_dat(9 downto 0);         -- STAR
                    when 9 => c_swreq <= '1';                       -- SWAP
                    when others => null;
                end case;
            end if;
        end if;
    end process;

end architecture attract;
