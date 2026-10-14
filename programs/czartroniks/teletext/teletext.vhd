-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Teletext v0.2.0: catalogue combo #47 (seed 20260920) -- chromatic glitch /
-- datamosh bars + ASCII / dither-art + scanline shear.  Two knobs each; the
-- switches and the slider decide how they combine.
--
--   K1 Bars     fraction of glyph bands displaced (datamosh)
--   K2 Distance mosh displacement (up to +-1020 px, wraps inside the picture)
--   K3 Cell     glyph cell 8 / 16 / 32 / 64 px
--   K4 Contrast density -> glyph mapping (100% = unity: white = densest glyph)
--   K5 Shear    per-line shear amount (+-30 px)
--   K6 Rate     shear wave rate (Wave) / tear probability (Tear)
--
--   S7  Ink     white glyphs on black / black on white
--   S8  Colour  glyphs mono / glyphs take the cell's chroma
--   S9  Mosh    displaces the video / displaces whole glyph cells
--   S10 Shear   triangle wave / random per-line tear
--   S11 Glyphs  ASCII ramp / block-dither set
--   P12 Reveal  video (moshed, sheared) .. glyph rendering
--
-- Line buffer read a line late with the mosh offset.  Each glyph band is
-- sampled on the line directly above it: that line's cell luma means and
-- chroma go to one bank of a two-bank cell RAM, which the band then reads
-- (the grid starts two lines down, so the top band samples the first real
-- line).  Glyph = 8x8 ROM row indexed by 16 density levels.  Shear is
-- applied on the output side through a 64-deep ring, so the glyph raster
-- tears too; each ring word carries its line's glyph row / bank / valid
-- bits, so the glyph lookup always matches the pixel it is drawn on.
--
-- v0.2.0: glyph positions keep counting through blanking (v0.1 drew the
-- right ~33 px of every line as background); the top band no longer shows
-- the previous frame's bottom band; moshed reads wrap inside the C_EDGE
-- inset and sheared edges fill with the line's own edge pixel (v0.1 painted
-- black); Cells mosh wraps on the line's real cell count; the right-edge
-- partial cell is written; a new line's shear is applied when that line
-- starts leaving the ring; Contrast 100% maps white to the densest glyph;
-- 64 px cells; HD HDMI bar conditioning (damier v0.4.4): [1 2 1] on Y/U/V
-- before the gate, and the glyph levels scaled 64 + 13/16 (y - 64) (white
-- ink / paper 776, coloured dark ink 168) -- on the glyphs only, so the
-- video keeps its level at low Reveal; labelled K3 honours the firmware's
-- label-INDEX default until the knob first reads a position; no_rw_check on
-- every RAM (the cell RAM's discarded bank read is the only collision).
-- Latency 59 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture teletext of program_top is
    -- IO-align pad: 1 register stage after the program's own output so the
    -- data_in -> data_out latency (59 + 1 = 60 clocks) is a multiple of 4,
    -- as the SDK requires; must match processing_delay_clks in the TOML.
    constant C_PROCESSING_DELAY_CLKS : natural := 60;
    signal s_io_align_in : t_video_stream_yuv444_30b;
    type t_io_align_pad is array (0 to 0) of t_video_stream_yuv444_30b;
    signal s_io_align_pad : t_io_align_pad;

    constant C_D    : integer := 33;          -- ring delay at zero shear
    constant C_LAT  : integer := 26 + C_D;    -- output register O25 + 1
    constant C_EDGE : integer := 12;          -- capture's black side columns

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return unsigned(v(9 downto 0));
        end if;
    end function;

    type t_glyph is array (0 to 255) of std_logic_vector(7 downto 0);
    constant C_GLYPH : t_glyph := (
        x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00",
        x"00", x"00", x"00", x"00", x"00", x"18", x"18", x"00",
        x"00", x"00", x"00", x"00", x"00", x"18", x"18", x"30",
        x"00", x"18", x"18", x"00", x"00", x"18", x"18", x"00",
        x"00", x"18", x"18", x"00", x"00", x"18", x"18", x"30",
        x"00", x"00", x"00", x"7e", x"00", x"00", x"00", x"00",
        x"00", x"00", x"32", x"4c", x"00", x"00", x"00", x"00",
        x"00", x"00", x"7e", x"00", x"7e", x"00", x"00", x"00",
        x"00", x"18", x"18", x"7e", x"7e", x"18", x"18", x"00",
        x"00", x"66", x"3c", x"ff", x"3c", x"66", x"00", x"00",
        x"00", x"00", x"3c", x"66", x"66", x"66", x"3c", x"00",
        x"00", x"00", x"66", x"3c", x"18", x"3c", x"66", x"00",
        x"24", x"24", x"ff", x"24", x"24", x"ff", x"24", x"24",
        x"62", x"66", x"0c", x"18", x"30", x"66", x"46", x"00",
        x"3c", x"66", x"66", x"3c", x"66", x"66", x"3c", x"00",
        x"3c", x"66", x"6e", x"6e", x"6e", x"60", x"3c", x"00",
        x"00", x"00", x"00", x"00", x"00", x"00", x"00", x"00",
        x"88", x"00", x"00", x"00", x"88", x"00", x"00", x"00",
        x"88", x"00", x"22", x"00", x"88", x"00", x"22", x"00",
        x"aa", x"00", x"22", x"00", x"aa", x"00", x"22", x"00",
        x"aa", x"00", x"aa", x"00", x"aa", x"00", x"aa", x"00",
        x"aa", x"44", x"aa", x"00", x"aa", x"44", x"aa", x"00",
        x"aa", x"44", x"aa", x"11", x"aa", x"44", x"aa", x"11",
        x"aa", x"55", x"aa", x"11", x"aa", x"55", x"aa", x"11",
        x"aa", x"55", x"aa", x"55", x"aa", x"55", x"aa", x"55",
        x"ee", x"55", x"aa", x"55", x"ee", x"55", x"aa", x"55",
        x"ee", x"55", x"bb", x"55", x"ee", x"55", x"bb", x"55",
        x"ff", x"55", x"bb", x"55", x"ff", x"55", x"bb", x"55",
        x"ff", x"55", x"ff", x"55", x"ff", x"55", x"ff", x"55",
        x"ff", x"dd", x"ff", x"55", x"ff", x"dd", x"ff", x"55",
        x"ff", x"dd", x"ff", x"77", x"ff", x"dd", x"ff", x"77",
        x"ff", x"ff", x"ff", x"ff", x"ff", x"ff", x"ff", x"ff"
    );

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    -- ring word: Y10 & U8 & V8 & glyph row 3 & bank & valid & '0'
    type t_ring is array (0 to 63) of std_logic_vector(31 downto 0);
    signal ring : t_ring := (others => x"10202000");
    signal s_rrd : std_logic_vector(31 downto 0) := x"10202000";
    -- cell word: luma mean 10 & U8 & V8 & "000000"
    type t_dens is array (0 to 255) of std_logic_vector(31 downto 0);
    signal dens0, dens1 : t_dens := (others => x"00202000");
    signal s_drd0, s_drd1 : std_logic_vector(31 downto 0) := x"00202000";

    attribute no_rw_check : boolean;
    attribute no_rw_check of lbY0, lbY1, lbU0, lbU1, lbV0, lbV1, ring, dens0, dens1 : signal is true;

    signal s_k_bars, s_k_dist, s_k_cell, s_k_con, s_k_shear, s_k_rate : unsigned(9 downto 0);
    signal s_sw_black, s_sw_colour, s_sw_cells, s_sw_tear, s_sw_blocks : std_logic;
    signal s_p_reveal : unsigned(9 downto 0);

    signal s_bars, s_dist, s_con, s_rate : unsigned(7 downto 0) := (others => '0');
    signal s_amt    : unsigned(4 downto 0) := (others => '0');
    signal s_live3  : std_logic := '0';
    signal s_ssk    : unsigned(1 downto 0) := "00";       -- K3 decode (live)
    signal s_ss     : unsigned(1 downto 0) := "00";       -- per frame: 8/16/32/64 px
    signal s_cmask  : unsigned(5 downto 0) := "000111";   -- cell px - 1
    signal s_mix    : unsigned(7 downto 0) := (others => '0');
    signal s_lfsr   : unsigned(15 downto 0) := x"ACE1";
    signal s_saw_active : std_logic := '0';
    signal s_wp     : unsigned(5 downto 0) := (others => '0');
    signal s_tph, s_fph : unsigned(7 downto 0) := (others => '0');

    -- positions: s_rx counts from each avid rise and keeps counting through
    -- blanking (saturates), so glyph positions past the line end stay true
    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '1');
    signal s_tail   : unsigned(6 downto 0) := (others => '1');   -- clocks since avid fell
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid, s_we : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n : std_logic := '1';
    signal s_prev_avid : std_logic := '0';
    signal s_gl : signed(11 downto 0) := to_signed(-2, 12);      -- glyph line = line - 2
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_wpar, s_rbank : std_logic := '0';

    -- per-line sequencer
    signal s_lstep  : unsigned(3 downto 0) := x"8";
    signal s_lw     : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_hi1    : unsigned(10 downto 0) := to_unsigned(467, 11);   -- lw - 1 - C_EDGE
    signal s_hi0    : signed(12 downto 0) := to_signed(468, 13);       -- lw - C_EDGE
    signal s_span   : signed(11 downto 0) := to_signed(456, 12);       -- lw - 2 C_EDGE
    signal s_wcm    : unsigned(11 downto 0) := (others => '0');
    signal s_ncells : unsigned(8 downto 0) := to_unsigned(60, 9);
    signal s_samp, s_newband, s_first, s_wbank : std_logic := '0';
    signal s_row_l  : unsigned(2 downto 0) := "000";
    signal s_dbank_l, s_valid_l : std_logic := '0';
    signal s_tri, s_tear : signed(7 downto 0) := (others => '0');
    signal s_shp   : signed(13 downto 0) := (others => '0');
    signal s_off_nx, s_off_row : unsigned(5 downto 0) := to_unsigned(C_D, 6);
    signal s_mp    : signed(18 downto 0) := (others => '0');
    signal s_mraw  : signed(11 downto 0) := (others => '0');   -- band displacement, px
    signal s_mhi, s_mlo : std_logic := '0';
    signal s_dn    : signed(11 downto 0) := (others => '0');   -- in (-span, span)
    signal s_dv    : signed(11 downto 0) := (others => '0');   -- video displacement
    signal s_dpos  : unsigned(10 downto 0) := (others => '0'); -- dn mod lw
    signal s_dvp, s_dvm, s_elo, s_ehi : signed(12 downto 0) := (others => '0');
    signal s_dc    : unsigned(8 downto 0) := (others => '0');  -- cell displacement
    signal s_dcm   : signed(9 downto 0) := (others => '0');    -- dc - ncells

    -- input side (S = clocks after the pixel was registered at S1)
    type t_x is array (2 to 9) of unsigned(10 downto 0);
    signal s_x : t_x := (others => (others => '1'));
    type t_b is array (2 to 9) of std_logic;
    signal s_avd : t_b := (others => '0');
    signal s_xa2   : unsigned(10 downto 0) := (others => '0');    -- read column (held/filled in blanking)
    signal s_xc3   : unsigned(10 downto 0) := (others => '0');
    signal s_u4, s_up4, s_um4 : signed(12 downto 0) := (others => '0');
    signal s_lt4, s_ge4 : std_logic := '0';
    signal s_addr5 : unsigned(10 downto 0) := (others => '0');
    signal s_a_rbank, s_a_first : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_csum  : unsigned(15 downto 0) := (others => '0');
    signal s_u8, s_v8 : unsigned(9 downto 0) := (others => '0');
    signal s_dwe9  : std_logic := '0';
    signal s_da9   : unsigned(7 downto 0) := (others => '0');
    signal s_dd9   : std_logic_vector(31 downto 0) := (others => '0');

    -- output side (O = clocks after the pixel's ring read was issued + ...)
    signal s_xo7   : signed(11 downto 0) := (others => '0');
    signal s_ra7   : unsigned(5 downto 0) := (others => '0');
    signal s_cia8  : unsigned(8 downto 0) := (others => '0');
    signal s_cib8  : signed(9 downto 0) := (others => '0');
    signal s_ci9   : unsigned(7 downto 0) := (others => '0');
    signal s_dens11 : unsigned(9 downto 0) := (others => '0');
    signal s_cu11, s_cv11 : unsigned(7 downto 0) := x"80";
    signal s_d12   : unsigned(9 downto 0) := (others => '0');
    signal s_dk13  : unsigned(10 downto 0) := (others => '0');
    signal s_p14   : unsigned(18 downto 0) := (others => '0');
    signal s_romaddr15 : unsigned(7 downto 0) := (others => '0');
    signal s_pat16, s_pat17 : std_logic_vector(7 downto 0) := (others => '0');
    signal s_inky18, s_inku18, s_inkv18 : unsigned(9 downto 0) := (others => '0');
    signal s_dy19, s_du19, s_dv19 : signed(10 downto 0) := (others => '0');
    signal s_py20, s_pu20, s_pv20 : signed(19 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := (others => '0');
    signal s_y22 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning: [1 2 1] window, g1 the centre
    signal s_g1y, s_g2y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_g1u, s_g2u, s_g1v, s_g2v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u24, s_v24, s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_c10 is array (9 to 20) of unsigned(9 downto 0);
    signal s_wy, s_wu, s_wv : t_c10 := (others => (others => '0'));
    type t_c8 is array (12 to 17) of unsigned(7 downto 0);
    signal s_cu, s_cv : t_c8 := (others => x"80");
    type t_g3 is array (8 to 17) of unsigned(2 downto 0);
    signal s_gcol : t_g3 := (others => (others => '0'));
    type t_bo is array (8 to 17) of std_logic;
    signal s_bgo : t_bo := (others => '1');
    type t_vo is array (9 to 17) of std_logic;
    signal s_val : t_vo := (others => '0');
    type t_r3 is array (9 to 14) of unsigned(2 downto 0);
    signal s_row : t_r3 := (others => (others => '0'));
    signal s_dbank9, s_dbank10 : std_logic := '0';

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_bars  <= unsigned(registers_in(0));
    s_k_dist  <= unsigned(registers_in(1));
    s_k_cell  <= unsigned(registers_in(2));
    s_k_con   <= unsigned(registers_in(3));
    s_k_shear <= unsigned(registers_in(4));
    s_k_rate  <= unsigned(registers_in(5));
    s_sw_black  <= registers_in(6)(0);
    s_sw_colour <= registers_in(6)(1);
    s_sw_cells  <= registers_in(6)(2);
    s_sw_tear   <= registers_in(6)(3);
    s_sw_blocks <= registers_in(6)(4);
    s_p_reveal <= unsigned(registers_in(7));

    p_ctrl : process(clk)
        variable v_t : unsigned(6 downto 0);
        variable v_s, v_o : signed(7 downto 0);
        variable v_w : signed(11 downto 0);
        variable v_lo : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_saw_active <= '1'; end if;
            -- position of the S1 pixel + 1; keeps counting past the line end
            if data_in.avid = '1' and s_prev_avid = '0' then
                s_rx <= to_unsigned(1, 11);
            elsif s_rx /= 2047 then
                s_rx <= s_rx + 1;
            end if;
            if data_in.avid = '1' then
                s_tail <= (others => '0');
            elsif s_tail /= 127 then
                s_tail <= s_tail + 1;
            end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;
            s_we <= data_in.avid;
            s_wp <= s_wp + 1;
            s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
            s_x(2) <= s_rx - 1; s_avd(2) <= s_in_avid;
            for i in 3 to 9 loop s_x(i) <= s_x(i - 1); s_avd(i) <= s_avd(i - 1); end loop;
            -- read column: the pixel itself; in blanking the last column is
            -- held (right fill) until 64 clocks after the line, then the first
            -- (left fill for the next line); the inset clamp follows
            if s_in_avid = '1' then
                s_xa2 <= s_rx - 1;
            elsif s_tail(6) = '1' then
                s_xa2 <= (others => '0');
            end if;

            s_bars <= s_k_bars(9 downto 2);
            s_dist <= s_k_dist(9 downto 2);
            -- the firmware loads a labelled default as the label INDEX;
            -- honour 0..3 as an index only until the knob first reads a position
            if s_k_cell >= 4 then s_live3 <= '1'; end if;
            if s_live3 = '0' and s_k_cell < 4 then s_ssk <= s_k_cell(1 downto 0); else s_ssk <= s_k_cell(9 downto 8); end if;
            s_con <= s_k_con(9 downto 2);
            s_amt <= s_k_shear(9 downto 5);
            s_rate <= s_k_rate(9 downto 2);
            s_mix <= s_p_reveal(9 downto 2);

            -- a new line's shear takes effect as its first pixel enters the
            -- ring (the previous line has left it by then: blank > 63)
            if s_avd(7) = '1' and s_avd(8) = '0' then
                s_off_row <= s_off_nx;
            end if;

            -- per-line sequencer (one small op per state), starts at hsync
            case to_integer(s_lstep) is
                when 0 =>
                    s_lw <= s_line_width;
                    v_lo := unsigned(s_gl(5 downto 0)) and s_cmask;
                    if v_lo = s_cmask then s_samp <= '1'; else s_samp <= '0'; end if;
                    if v_lo = 0 then s_newband <= '1'; else s_newband <= '0'; end if;
                    if s_gl = -2 then s_first <= '1'; else s_first <= '0'; end if;
                    s_valid_l <= not s_gl(11);
                    case to_integer(s_ss) is
                        when 0 => s_row_l <= unsigned(s_gl(2 downto 0)); s_dbank_l <= s_gl(3); s_wbank <= not s_gl(3);
                        when 1 => s_row_l <= unsigned(s_gl(3 downto 1)); s_dbank_l <= s_gl(4); s_wbank <= not s_gl(4);
                        when 2 => s_row_l <= unsigned(s_gl(4 downto 2)); s_dbank_l <= s_gl(5); s_wbank <= not s_gl(5);
                        when others => s_row_l <= unsigned(s_gl(5 downto 3)); s_dbank_l <= s_gl(6); s_wbank <= not s_gl(6);
                    end case;
                    if s_tph(7) = '1' then v_t := not s_tph(6 downto 0); else v_t := s_tph(6 downto 0); end if;
                    s_tri <= signed(resize(v_t, 8)) - 64;
                    if s_lfsr(15 downto 8) < s_rate then s_tear <= signed(resize(s_lfsr(6 downto 0), 8)) - 64; else s_tear <= (others => '0'); end if;
                    s_tph <= s_tph + resize(s_rate(7 downto 3), 8);
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    if s_sw_tear = '1' then s_shp <= s_tear * signed(resize(s_amt, 6)); else s_shp <= s_tri * signed(resize(s_amt, 6)); end if;
                    s_mp <= (signed(resize(s_lfsr(8 downto 0), 10)) - 256) * signed(resize(s_dist, 9));
                    s_hi1  <= s_lw - (C_EDGE + 1);
                    s_hi0  <= signed(resize(s_lw, 13)) - C_EDGE;
                    s_span <= signed(resize(s_lw, 12)) - 2 * C_EDGE;
                    s_wcm  <= resize(s_lw, 12) + resize(s_cmask, 12);
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    v_s := resize(shift_right(s_shp, 6), 8);
                    if v_s > 30 then v_s := to_signed(30, 8); elsif v_s < -30 then v_s := to_signed(-30, 8); end if;
                    -- slice, not resize: synthesis truncates either way, but a
                    -- narrowing signed resize folds 32..63 in GHDL simulation
                    v_o := to_signed(C_D, 8) + v_s;
                    s_off_nx <= unsigned(v_o(5 downto 0));
                    if s_newband = '1' then
                        if s_lfsr(7 downto 0) < s_bars then s_mraw <= resize(shift_right(s_mp, 6), 12); else s_mraw <= (others => '0'); end if;
                    end if;
                    case to_integer(s_ss) is
                        when 0 => s_ncells <= resize(s_wcm(11 downto 3), 9);
                        when 1 => s_ncells <= resize(s_wcm(11 downto 4), 9);
                        when 2 => s_ncells <= resize(s_wcm(11 downto 5), 9);
                        when others => s_ncells <= resize(s_wcm(11 downto 6), 9);
                    end case;
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    if s_mraw >= s_span then s_mhi <= '1'; else s_mhi <= '0'; end if;
                    if s_mraw <= -s_span then s_mlo <= '1'; else s_mlo <= '0'; end if;
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    if s_mhi = '1' then s_dn <= s_mraw - s_span;
                    elsif s_mlo = '1' then s_dn <= s_mraw + s_span;
                    else s_dn <= s_mraw; end if;
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    if s_sw_cells = '1' then s_dv <= (others => '0'); else s_dv <= s_dn; end if;
                    if s_dn < 0 then
                        v_w := s_dn + signed(resize(s_lw, 12));
                        s_dpos <= unsigned(v_w(10 downto 0));
                    else
                        s_dpos <= unsigned(s_dn(10 downto 0));
                    end if;
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    s_dvp <= resize(s_dv, 13) + resize(s_span, 13);
                    s_dvm <= resize(s_dv, 13) - resize(s_span, 13);
                    s_elo <= to_signed(C_EDGE, 13) - resize(s_dv, 13);
                    s_ehi <= s_hi0 - resize(s_dv, 13);
                    case to_integer(s_ss) is
                        when 0 => s_dc <= resize(s_dpos(10 downto 3), 9);
                        when 1 => s_dc <= resize(s_dpos(10 downto 4), 9);
                        when 2 => s_dc <= resize(s_dpos(10 downto 5), 9);
                        when others => s_dc <= resize(s_dpos(10 downto 6), 9);
                    end case;
                    s_lstep <= s_lstep + 1;
                when 7 =>
                    s_dcm <= signed(resize(s_dc, 10)) - signed(resize(s_ncells, 10));
                    s_lstep <= s_lstep + 1;
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_gl <= s_gl + 1;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
            end if;
            -- frame events: resets are idempotent; toggles need the saw-active guard
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_gl <= to_signed(-2, 12); s_wpar <= '0';
                s_mraw <= (others => '0');
                s_ss <= s_ssk;
                case to_integer(s_ssk) is
                    when 0 => s_cmask <= "000111";
                    when 1 => s_cmask <= "001111";
                    when 2 => s_cmask <= "011111";
                    when others => s_cmask <= "111111";
                end case;
                if s_saw_active = '1' then s_fph <= s_fph + 3; end if;
                s_tph <= s_fph;
                s_saw_active <= '0';
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr5));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr5));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr5(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr5(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr5(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr5(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_rrd <= ring(to_integer(s_ra7));
            ring(to_integer(s_wp)) <= std_logic_vector(s_wet_y) & std_logic_vector(s_wet_u(9 downto 2)) & std_logic_vector(s_wet_v(9 downto 2))
                                      & std_logic_vector(s_row_l) & s_dbank_l & s_valid_l & '0';
        end if;
    end process;
    p_dens0 : process(clk)
    begin
        if rising_edge(clk) then
            s_drd0 <= dens0(to_integer(s_ci9));
            if s_dwe9 = '1' and s_wbank = '0' then dens0(to_integer(s_da9)) <= s_dd9; end if;
        end if;
    end process;
    p_dens1 : process(clk)
    begin
        if rising_edge(clk) then
            s_drd1 <= dens1(to_integer(s_ci9));
            if s_dwe9 = '1' and s_wbank = '1' then dens1(to_integer(s_da9)) <= s_dd9; end if;
        end if;
    end process;
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_pat16 <= C_GLYPH(to_integer(s_romaddr15));
            s_pat17 <= s_pat16;
        end if;
    end process;

    p_pix : process(clk)
        variable v_lo : unsigned(5 downto 0);
        variable v_ink : std_logic;
        variable v_g : unsigned(6 downto 0);
        variable v_ci : unsigned(7 downto 0);
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            ------------------------------------------------------------------
            -- input side: moshed, wrapped, inset-clamped line-buffer read
            ------------------------------------------------------------------
            -- S3: clamp into the inset (the capture's side columns are black)
            if s_xa2 < C_EDGE then
                s_xc3 <= to_unsigned(C_EDGE, 11);
            elsif s_xa2 > s_hi1 then
                s_xc3 <= s_hi1;
            else
                s_xc3 <= s_xa2;
            end if;
            -- S4: displaced column and its two wrap candidates, range tests
            s_u4  <= signed(resize(s_xc3, 13)) + resize(s_dv, 13);
            s_up4 <= signed(resize(s_xc3, 13)) + s_dvp;
            s_um4 <= signed(resize(s_xc3, 13)) + s_dvm;
            if signed(resize(s_xc3, 13)) < s_elo then s_lt4 <= '1'; else s_lt4 <= '0'; end if;
            if signed(resize(s_xc3, 13)) >= s_ehi then s_ge4 <= '1'; else s_ge4 <= '0'; end if;
            -- S5: read address, wrapped inside [C_EDGE, lw - C_EDGE)
            if s_lt4 = '1' then s_addr5 <= unsigned(s_up4(10 downto 0));
            elsif s_ge4 = '1' then s_addr5 <= unsigned(s_um4(10 downto 0));
            else s_addr5 <= unsigned(s_u4(10 downto 0)); end if;
            -- S7: fetch (S6 = RAM out); the frame's first line has no line above
            s_a_rbank <= s_rbank; s_a_first <= s_first;
            if s_a_first = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- S8: cell sum of the S7 pixel, sampled chroma
            v_lo := s_x(7)(5 downto 0) and s_cmask;
            if v_lo = 0 then s_csum <= resize(s_wet_y, 16); else s_csum <= s_csum + resize(s_wet_y, 16); end if;
            s_u8 <= s_wet_u; s_v8 <= s_wet_v;
            -- S9: cell write on sample lines (the S8 pixel ends its cell, or the line)
            v_lo := s_x(8)(5 downto 0) and s_cmask;
            if s_samp = '1' and s_avd(8) = '1' and (v_lo = s_cmask or s_avd(7) = '0') then s_dwe9 <= '1'; else s_dwe9 <= '0'; end if;
            case to_integer(s_ss) is
                when 0 => s_da9 <= s_x(8)(10 downto 3);
                          s_dd9 <= std_logic_vector(s_csum(12 downto 3)) & std_logic_vector(s_u8(9 downto 2)) & std_logic_vector(s_v8(9 downto 2)) & "000000";
                when 1 => s_da9 <= resize(s_x(8)(10 downto 4), 8);
                          s_dd9 <= std_logic_vector(s_csum(13 downto 4)) & std_logic_vector(s_u8(9 downto 2)) & std_logic_vector(s_v8(9 downto 2)) & "000000";
                when 2 => s_da9 <= resize(s_x(8)(10 downto 5), 8);
                          s_dd9 <= std_logic_vector(s_csum(14 downto 5)) & std_logic_vector(s_u8(9 downto 2)) & std_logic_vector(s_v8(9 downto 2)) & "000000";
                when others => s_da9 <= resize(s_x(8)(10 downto 6), 8);
                          s_dd9 <= std_logic_vector(s_csum(15 downto 6)) & std_logic_vector(s_u8(9 downto 2)) & std_logic_vector(s_v8(9 downto 2)) & "000000";
            end case;

            ------------------------------------------------------------------
            -- output side: the pixel leaving the ring and its glyph
            ------------------------------------------------------------------
            -- O7: its position (x keeps counting through blanking) and ring address
            s_xo7 <= signed(resize(s_x(7), 12)) - signed(resize(s_off_row, 12));
            s_ra7 <= s_wp - s_off_row;
            -- O8: cell index (+ Cells displacement, two wrap candidates), glyph column
            case to_integer(s_ss) is
                when 0 => v_ci := unsigned(s_xo7(10 downto 3)); s_gcol(8) <= unsigned(s_xo7(2 downto 0));
                when 1 => v_ci := resize(unsigned(s_xo7(10 downto 4)), 8); s_gcol(8) <= unsigned(s_xo7(3 downto 1));
                when 2 => v_ci := resize(unsigned(s_xo7(10 downto 5)), 8); s_gcol(8) <= unsigned(s_xo7(4 downto 2));
                when others => v_ci := resize(unsigned(s_xo7(10 downto 6)), 8); s_gcol(8) <= unsigned(s_xo7(5 downto 3));
            end case;
            s_cia8 <= resize(v_ci, 9) + s_dc;
            s_cib8 <= signed(resize(v_ci, 10)) + s_dcm;
            if s_xo7 < 0 or s_xo7 >= signed(resize(s_lw, 12)) then s_bgo(8) <= '1'; else s_bgo(8) <= '0'; end if;
            -- O9: ring out -> fabric; wrapped cell address
            s_wy(9) <= unsigned(s_rrd(31 downto 22)); s_wu(9) <= unsigned(s_rrd(21 downto 14)) & "00"; s_wv(9) <= unsigned(s_rrd(13 downto 6)) & "00";
            s_row(9) <= unsigned(s_rrd(5 downto 3)); s_dbank9 <= s_rrd(2); s_val(9) <= s_rrd(1);
            if s_cib8 < 0 then s_ci9 <= s_cia8(7 downto 0); else s_ci9 <= unsigned(s_cib8(7 downto 0)); end if;
            -- O11: cell RAM out (O10) -> fabric, the band's bank
            s_dbank10 <= s_dbank9;
            if s_dbank10 = '1' then
                s_dens11 <= unsigned(s_drd1(31 downto 22)); s_cu11 <= unsigned(s_drd1(21 downto 14)); s_cv11 <= unsigned(s_drd1(13 downto 6));
            else
                s_dens11 <= unsigned(s_drd0(31 downto 22)); s_cu11 <= unsigned(s_drd0(21 downto 14)); s_cv11 <= unsigned(s_drd0(13 downto 6));
            end if;
            -- O12
            if s_dens11 < 64 then s_d12 <= (others => '0'); else s_d12 <= s_dens11 - 64; end if;
            -- O13: x 1.156 so that white (876) reaches the densest glyph at 100%
            s_dk13 <= resize(s_d12, 11) + resize(s_d12(9 downto 3), 11) + resize(s_d12(9 downto 5), 11);
            -- O14
            s_p14 <= s_dk13 * s_con;
            -- O15: glyph index and row
            v_g := s_p14(18 downto 12);
            if v_g > 15 then v_g := to_unsigned(15, 7); end if;
            s_romaddr15 <= s_sw_blocks & v_g(3 downto 0) & s_row(14);
            -- O18: ink (O16 ROM, O17 fabric)
            v_ink := s_pat17(7 - to_integer(s_gcol(17)));
            if s_bgo(17) = '1' or s_val(17) = '0' then v_ink := '0'; end if;
            -- glyph levels pre-scaled 64 + 13/16 (y - 64): white 776 (the bar
            -- fix's luma cut, on the glyphs only, so the video keeps its level)
            if v_ink = '1' and s_sw_colour = '1' then
                if s_sw_black = '1' then s_inky18 <= to_unsigned(168, 10); else s_inky18 <= to_unsigned(776, 10); end if;
                s_inku18 <= s_cu(17) & "00"; s_inkv18 <= s_cv(17) & "00";
            else
                if (v_ink xor s_sw_black) = '1' then s_inky18 <= to_unsigned(776, 10); else s_inky18 <= to_unsigned(64, 10); end if;
                s_inku18 <= to_unsigned(512, 10); s_inkv18 <= to_unsigned(512, 10);
            end if;
            -- O19
            s_dy19 <= signed(resize(s_inky18, 11)) - signed(resize(s_wy(18), 11));
            s_du19 <= signed(resize(s_inku18, 11)) - signed(resize(s_wu(18), 11));
            s_dv19 <= signed(resize(s_inkv18, 11)) - signed(resize(s_wv(18), 11));
            -- O20
            s_py20 <= s_dy19 * signed(resize(s_mix, 9));
            s_pu20 <= s_du19 * signed(resize(s_mix, 9));
            s_pv20 <= s_dv19 * signed(resize(s_mix, 9));
            -- O21
            s_y21 <= f_clamp10(signed(resize(s_wy(20), 13)) + resize(shift_right(s_py20, 8), 13));
            s_u21 <= f_clamp10(signed(resize(s_wu(20), 13)) + resize(shift_right(s_pu20, 8), 13));
            s_v21 <= f_clamp10(signed(resize(s_wv(20), 13)) + resize(shift_right(s_pv20, 8), 13));
            -- O22: neutral outside avid (so the smoothing's edge taps read black)
            if s_avid_sr(C_D + 21) = '1' then
                s_y22 <= s_y21; s_u22 <= s_u21; s_v22 <= s_v21;
            else
                s_y22 <= to_unsigned(64, 10); s_u22 <= to_unsigned(512, 10); s_v22 <= to_unsigned(512, 10);
            end if;
            -- O23..O24: horizontal [1 2 1]/4 on Y/U/V (g1 the centre)
            s_g1y <= s_y22; s_g2y <= s_g1y;
            s_g1u <= s_u22; s_g2u <= s_g1u;
            s_g1v <= s_v22; s_g2v <= s_g1v;
            v_sy := resize(s_g2y, 12) + shift_left(resize(s_g1y, 12), 1) + resize(s_y22, 12);
            v_su := resize(s_g2u, 12) + shift_left(resize(s_g1u, 12), 1) + resize(s_u22, 12);
            v_sv := resize(s_g2v, 12) + shift_left(resize(s_g1v, 12), 1) + resize(s_v22, 12);
            s_y24 <= v_sy(11 downto 2); s_u24 <= v_su(11 downto 2); s_v24 <= v_sv(11 downto 2);
            -- O25: gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y25 <= s_y24; s_u25 <= s_u24; s_v25 <= s_v24;
            else
                s_y25 <= to_unsigned(64, 10); s_u25 <= to_unsigned(512, 10); s_v25 <= to_unsigned(512, 10);
            end if;

            -- carry chains
            for i in 10 to 20 loop s_wy(i) <= s_wy(i - 1); s_wu(i) <= s_wu(i - 1); s_wv(i) <= s_wv(i - 1); end loop;
            s_cu(12) <= s_cu11; s_cv(12) <= s_cv11;
            for i in 13 to 17 loop s_cu(i) <= s_cu(i - 1); s_cv(i) <= s_cv(i - 1); end loop;
            for i in 9 to 17 loop s_gcol(i) <= s_gcol(i - 1); s_bgo(i) <= s_bgo(i - 1); end loop;
            for i in 10 to 17 loop s_val(i) <= s_val(i - 1); end loop;
            for i in 10 to 14 loop s_row(i) <= s_row(i - 1); end loop;
        end if;
    end process p_pix;

    s_io_align_in.y       <= std_logic_vector(s_y25);
    s_io_align_in.u       <= std_logic_vector(s_u25);
    s_io_align_in.v       <= std_logic_vector(s_v25);
    s_io_align_in.hsync_n <= s_hsync_sr(C_LAT - 1);
    s_io_align_in.vsync_n <= s_vsync_sr(C_LAT - 1);
    s_io_align_in.field_n <= s_field_sr(C_LAT - 1);
    s_io_align_in.avid    <= s_avid_sr(C_LAT - 1);

    -- IO-align pad: video and syncs delayed together, so only the total
    -- latency changes (59 -> 60).
    p_io_align_pad : process(clk)
    begin
        if rising_edge(clk) then
            s_io_align_pad(0) <= s_io_align_in;
        end if;
    end process p_io_align_pad;

    data_out <= s_io_align_pad(0);

end architecture teletext;
