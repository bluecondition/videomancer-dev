-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Morphogen v0.1.0: catalogue combo (2026-10-21) from two rarely used
-- catalogue entries -- "reaction-diffusion driven by input" (never used by a
-- combo) and "cartoon / cel" (used once).  A Gray-Scott reaction-diffusion
-- grows down the screen out of the picture's edges, fed by its light, and
-- redraws it as beads, strands, arches and coats; the ground is the picture
-- cel-flattened into luma bands with ink outlines.
--
--   K1 Species  8 Gray-Scott regimes (labelled): Beads, Seeds, Pearls,
--               Streaks, Strands, Arches, Fronds, Waves
--   K2 Grain    reaction cell 4 .. 16 px on the 1080-line canvas (pattern size)
--   K3 Drift    the pattern leans left .. straight .. right (advection)
--   K4 Bands    cel luma bands 2 .. 9 in the ground
--   K5 Ink      outline weight: cel band edges and bead rims (0 = none)
--   K6 Skin     Pearl (glossy beads in the picture's colours) / Toon (flat
--               beads, ink rims) / Coat (dark spots on a tawny cel coat) /
--               Ink (black blots on paper)
--   S7  Drive   the light (Light) or the shadows (Dark) feed the reaction
--   S8  Rain    seeds also fall at random over the whole frame
--   S9  Flip    beads <-> ground: the pattern becomes a net round holes
--   S10 Shine   specular highlights on the beads
--   S11 Ground  the cel picture / black (the pattern alone)
--   P12 Bloom   sparse beads on the strongest edges .. the whole picture
--               encrusted (edge-seed threshold, feed boost, rain density).
--               Glided.
--
-- Engine.  The reaction runs in 1-D across each line, one line = two time
-- steps, on cells whose width is K2 virtual pixels (a DDA on the virtual
-- 1080-line canvas).  State {u, v} Q0.12 per cell in a 1024 x 32 RAM, ONE
-- bank: the old state of cell k+2 is read at the start of cell k, and the new
-- state of a cell is written ~9 cells later, so every read precedes its own
-- overwrite.  One 7-stage update pipeline is shared by the two sub-steps:
-- slot A (cell start + 1) steps cell k-2 from the old state (plus seeds),
-- writing a 256-entry hand-off RAM; slot B (cell start + 2) steps cell k-9
-- from those results.  Cells are >= 2 clocks wide, so the slots never
-- collide.  A second 1024 x 16 RAM keeps each cell's luma and the previous
-- line's v (vertical edges for the seeds, vertical slope for the shading).
-- Seeds: cell luma steps above the Bloom threshold (and Rain).  The display
-- shows the previous line's state, interpolated between cells.  The first
-- line of a field starts empty, so nothing carries between frames.
-- dt per line is 1 at 1080p, 1.5 at 720p and 2 elsewhere (capped), so the
-- pattern keeps its shape across modes (interlaced SD evolves 2x slower).
--
-- Hardware contracts: vblank zone (no avid for 1024 clocks) for every
-- per-field event; the picture is carried in a 256-word EBR ring; generated
-- colours swap U/V on lines off the voted hsync->avid parity; output
-- conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma x13/16);
-- blanking gate; no_rw_check on every RAM.  Latency 24 clocks, all modes.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture morphogen of program_top is
    -- data_in -> data_out latency (multiple of 4); matches processing_delay_clks.
    constant C_PROCESSING_DELAY_CLKS : natural := 24;
    constant C_LAT  : integer := 24;
    constant C_TIN  : integer := 820;    -- v above this = bead (0.2)

    -- K1 species: Gray-Scott feed / kill (x 4096)
    type t_i8 is array (0 to 7) of integer range 0 to 1023;
    constant C_F : t_i8 := (143, 160, 123, 164, 205, 102, 74, 90);
    constant C_K : t_i8 := (254, 266, 246, 246, 250, 225, 209, 209);
    -- K2 grain: 256 / (2 x cell) for cell 4 .. 16
    type t_r13 is array (0 to 12) of integer range 0 to 63;
    constant C_RECIP : t_r13 := (32, 26, 21, 18, 16, 14, 13, 12, 11, 10, 9, 9, 8);
    -- K4 bands 2 .. 9: luma step 768 / (B - 1)
    type t_s8 is array (0 to 7) of integer range 0 to 1023;
    constant C_STEP : t_s8 := (768, 384, 256, 192, 154, 128, 110, 96);

    -- x step per pixel (Q.1 virtual units: 2 / 3 / 4 = 1080 / 720 / SD)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- y step in Q.8 virtual units per FIELD line
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
    -- time step per field line: 0 = x1 (1080p), 1 = x1.5 (720p), 2 = x2 (rest, capped)
    function f_dt(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 3 | 7 | 9 | 11 | 13 | 15 => return "00";
            when 5 | 6 | 14              => return "01";
            when others                  => return "10";
        end case;
    end function;
    -- v x dt
    function f_xdt(v : unsigned; d : unsigned(1 downto 0)) return unsigned is
        variable a : unsigned(v'length downto 0);
    begin
        a := resize(v, v'length + 1);
        if d = "00" then
            return a;
        elsif d = "01" then
            return a + shift_right(a, 1);
        else
            return shift_left(a, 1);
        end if;
    end function;

    -- cell state word {u 12, v 12, spare 8}; dead = u 4095, v 0
    constant C_DEADW : std_logic_vector(31 downto 0) := x"FFF000" & x"00";
    constant C_BLACK : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    -- RAMs
    type t_m32 is array (0 to 1023) of std_logic_vector(31 downto 0);   -- state
    type t_o16 is array (0 to 1023) of std_logic_vector(15 downto 0);   -- {v of the line above >> 4, cell luma}
    type t_s32 is array (0 to 255) of std_logic_vector(31 downto 0);    -- sub-step-1 hand-off {u, v, luma}
    type t_ring is array (0 to 255) of std_logic_vector(31 downto 0);   -- picture {y, u, v, 00}
    type t_c8 is array (0 to 2047) of std_logic_vector(7 downto 0);     -- cel column {band 4, ink count 2, 00}
    signal mram : t_m32 := (others => C_DEADW);
    signal oram : t_o16 := (others => (others => '0'));
    signal sram : t_s32 := (others => C_DEADW);
    signal ring : t_ring := (others => C_BLACK);
    signal cram : t_c8 := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of mram, oram, sram, ring, cram : signal is true;

    signal s_mra, s_mwa, s_ora, s_owa : unsigned(9 downto 0) := (others => '0');
    signal s_mq, s_mwd : std_logic_vector(31 downto 0) := C_DEADW;
    signal s_mwe, s_owe : std_logic := '0';
    signal s_oq, s_owd : std_logic_vector(15 downto 0) := (others => '0');
    signal s_sra, s_swa : unsigned(7 downto 0) := (others => '0');
    signal s_sq, s_swd : std_logic_vector(31 downto 0) := C_DEADW;
    signal s_swe : std_logic := '0';
    signal s_vw, s_rw_a, s_rr_a : unsigned(7 downto 0) := (others => '0');
    signal s_rw_d, s_rr_q : std_logic_vector(31 downto 0) := C_BLACK;
    signal s_cra, s_cwa : unsigned(10 downto 0) := (others => '0');
    signal s_cq, s_cwd : std_logic_vector(7 downto 0) := (others => '0');
    signal s_cwe : std_logic := '0';

    -- controls
    signal s_tid : std_logic_vector(3 downto 0) := (others => '0');
    signal s_xs  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_spk : unsigned(2 downto 0) := (others => '0');
    signal s_live1, s_live6 : std_logic := '0';
    signal s_j2, s_j4, s_j5 : unsigned(3 downto 0) := (others => '0');
    signal s_j3 : unsigned(5 downto 0) := to_unsigned(32, 6);
    signal s_skk : unsigned(1 downto 0) := "00";
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');      -- glided P12, Q.2
    -- field-latched
    signal s_sp : unsigned(2 downto 0) := (others => '0');
    signal s_l2, s_l4, s_l5 : unsigned(3 downto 0) := (others => '0');
    signal s_l3 : unsigned(5 downto 0) := to_unsigned(32, 6);
    signal s_skin : unsigned(1 downto 0) := "00";
    signal s_dark, s_rain, s_flip, s_shine, s_black : std_logic := '0';
    signal s_dtq : unsigned(1 downto 0) := "00";
    -- derived
    signal s_bq : unsigned(5 downto 0) := (others => '0');
    signal s_boost : unsigned(8 downto 0) := to_unsigned(218, 9);
    signal s_fb : unsigned(16 downto 0) := (others => '0');
    signal s_f0, s_k0 : unsigned(10 downto 0) := (others => '0');
    signal s_a0 : unsigned(9 downto 0) := (others => '0');
    signal s_a1 : unsigned(9 downto 0) := (others => '0');
    signal s_dneg : std_logic := '0';                       -- drift direction
    signal s_dlev : unsigned(1 downto 0) := "00";           -- drift strength 0 .. 3
    signal s_thr : unsigned(7 downto 0) := to_unsigned(96, 8);
    signal s_bsq : unsigned(11 downto 0) := (others => '0');
    signal s_raind : unsigned(11 downto 0) := (others => '0');
    signal s_cell2 : unsigned(5 downto 0) := to_unsigned(8, 6);   -- 2 x cell, virtual Q.1
    signal s_recip : unsigned(5 downto 0) := to_unsigned(32, 6);
    signal s_gys : unsigned(1 downto 0) := "01";
    signal s_bands : unsigned(3 downto 0) := to_unsigned(4, 4);
    signal s_bstep : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_inkw : unsigned(1 downto 0) := "00";
    signal s_trim : unsigned(11 downto 0) := to_unsigned(700, 12);

    -- line
    signal s_hsq, s_ph, s_upar, s_swl : std_logic := '0';
    signal s_upc : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal s_vl : unsigned(19 downto 0) := (others => '0');
    signal s_first, s_farm : std_logic := '0';

    -- pixel input / delays
    type t_u8a  is array (natural range <>) of unsigned(7 downto 0);
    type t_u11a is array (natural range <>) of unsigned(10 downto 0);
    signal s_y1 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u1, s_v1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_rx : t_u11a(1 to 12) := (others => (others => '0'));
    signal s_y8 : t_u8a(2 to 9) := (others => (others => '0'));

    -- cell DDA (stage 2) and windows
    signal s_cx : unsigned(5 downto 0) := (others => '0');
    signal s_ck : unsigned(9 downto 0) := (others => '0');
    signal s_cs : std_logic := '0';
    signal s_ly : unsigned(7 downto 0) := (others => '0');
    type t_w32 is array (0 to 4) of std_logic_vector(23 downto 0);
    type t_w16 is array (0 to 3) of std_logic_vector(15 downto 0);
    signal w_old : t_w32 := (others => x"FFF000");          -- {u, v}: (0) = old(k+1) .. (4) = old(k-3)
    signal w_or  : t_w16 := (others => (others => '0'));    -- {older v8, luma above}: (0) = k+1 .. (3) = k-2
    signal w_lum : t_u8a(0 to 2) := (others => (others => '0'));   -- (0) = lum(k-1) ..
    signal w_sd  : std_logic_vector(0 to 1) := "00";        -- seeds: (0) = seed(k-1), (1) = seed(k-2)
    signal w_act : std_logic_vector(0 to 3) := (others => '0');    -- (0) = act(k) .. (3) = act(k-3)
    type t_ws is array (0 to 2) of std_logic_vector(31 downto 0);
    signal w_s1 : t_ws := (others => C_DEADW);              -- (0) = s1(k-8), (1) = s1(k-9), (2) = s1(k-10)
    signal s_vla, s_vlb, s_vlh, s_ck37, s_hk, s_hr1, s_hr : unsigned(15 downto 0) := (others => '0');
    signal s_slA, s_slB : std_logic := '0';
    signal s_e1d, s_e2d : signed(8 downto 0) := (others => '0');   -- seed: luma steps of the ended cell
    signal s_erain, s_sgo : std_logic := '0';
    signal s_kA : unsigned(9 downto 0) := (others => '0');
    signal s_kB : unsigned(9 downto 0) := (others => '0');

    -- update pipeline
    type t_op is record
        v, kb : std_logic;
        idx   : unsigned(9 downto 0);
        uc, vc : unsigned(11 downto 0);
        lum   : unsigned(7 downto 0);
    end record;
    constant C_OP0 : t_op := ('0', '0', (others => '0'), (others => '1'), (others => '0'), (others => '0'));
    signal o0, o1, o2, o3, o4, o5, o6 : t_op := C_OP0;
    signal o0_ul, o0_ur, o0_vl, o0_vr : unsigned(11 downto 0) := (others => '0');
    signal x1_lu, x1_lv, x1_du, x1_dv : signed(13 downto 0) := (others => '0');
    signal x1_nu : unsigned(7 downto 0) := (others => '0');
    signal x2_vv : unsigned(15 downto 0) := (others => '0');
    signal x2_pu, x2_pv, x2_au, x2_av : signed(13 downto 0) := (others => '0');
    signal x2_ml : unsigned(17 downto 0) := (others => '0');
    signal x2_nu : unsigned(7 downto 0) := (others => '0');
    signal x3_r : unsigned(15 downto 0) := (others => '0');
    signal x3_fc : unsigned(10 downto 0) := (others => '0');
    signal x3_dfu, x3_dfv : signed(14 downto 0) := (others => '0');
    signal x3_nu : unsigned(7 downto 0) := (others => '0');
    signal x4_rd : unsigned(12 downto 0) := (others => '0');
    signal x4_fu : unsigned(18 downto 0) := (others => '0');
    signal x4_g  : unsigned(11 downto 0) := (others => '0');
    signal x4_dfu, x4_dfv : signed(14 downto 0) := (others => '0');
    signal x5_gv : unsigned(19 downto 0) := (others => '0');
    signal x5_du : signed(15 downto 0) := (others => '0');
    signal x5_rd : unsigned(12 downto 0) := (others => '0');
    signal x5_dfv : signed(14 downto 0) := (others => '0');
    signal x6_dv : signed(15 downto 0) := (others => '0');
    signal x6_u : unsigned(11 downto 0) := (others => '1');

    -- display / compose (suffix = stage)
    signal s_cx5, s_cx6 : unsigned(5 downto 0) := (others => '0');
    signal s_vc6, s_vn6 : unsigned(11 downto 0) := (others => '0');
    signal s_vo6 : unsigned(7 downto 0) := (others => '0');
    signal s_fr7 : unsigned(11 downto 0) := (others => '0');
    signal s_vc7, s_vc8 : unsigned(11 downto 0) := (others => '0');
    signal s_dd7 : signed(12 downto 0) := (others => '0');
    signal s_gs7, s_gs8, s_gs9 : signed(11 downto 0) := (others => '0');
    signal s_pr8 : signed(21 downto 0) := (others => '0');
    signal s_f9 : unsigned(11 downto 0) := (others => '0');
    signal s_q7 : unsigned(11 downto 0) := (others => '0');
    signal s_q8, s_q9, s_q10 : unsigned(3 downto 0) := (others => '0');
    signal s_cq9 : std_logic_vector(7 downto 0) := (others => '0');
    signal s_yc9, s_yc10, s_yc11, s_yc12, s_yc13 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_hrun : unsigned(1 downto 0) := (others => '0');
    signal s_bead10, s_rim10, s_ink10 : std_logic := '0';
    signal s_lt10 : unsigned(7 downto 0) := (others => '0');
    type t_flags is array (11 to 13) of std_logic_vector(3 downto 0);   -- {spec, bead, rim, ink}
    signal s_fl : t_flags := (others => (others => '0'));
    type t_lt is array (11 to 13) of unsigned(7 downto 0);
    signal s_lt : t_lt := (others => (others => '0'));
    signal s_pic13 : std_logic_vector(31 downto 0) := C_BLACK;
    signal s_py14 : unsigned(17 downto 0) := (others => '0');
    signal s_gy14, s_gu14, s_gv14, s_bu14, s_bv14, s_yc14 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k14 : unsigned(2 downto 0) := (others => '0');   -- 0 ground, 1 bead, 2 rim, 3 ink line, 4 spec
    signal s_y15 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y16, s_y17, s_y18, s_y19, s_y20, s_y21x, s_yf21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_y10 is array (22 to 24) of unsigned(9 downto 0);
    signal s_yo : t_y10 := (others => to_unsigned(64, 10));
    signal s_uo, s_vo : t_y10 := (others => to_unsigned(512, 10));

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');   -- s_avid_sr(k) = avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- RAMs: one write statement, registered read, no conditional read
    ---------------------------------------------------------------------------
    p_mram : process(clk)
    begin
        if rising_edge(clk) then
            s_mq <= mram(to_integer(s_mra));
            if s_mwe = '1' then mram(to_integer(s_mwa)) <= s_mwd; end if;
        end if;
    end process p_mram;

    p_oram : process(clk)
    begin
        if rising_edge(clk) then
            s_oq <= oram(to_integer(s_ora));
            if s_owe = '1' then oram(to_integer(s_owa)) <= s_owd; end if;
        end if;
    end process p_oram;

    p_sram : process(clk)
    begin
        if rising_edge(clk) then
            s_sq <= sram(to_integer(s_sra));
            if s_swe = '1' then sram(to_integer(s_swa)) <= s_swd; end if;
        end if;
    end process p_sram;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            s_rr_q <= ring(to_integer(s_rr_a));
            ring(to_integer(s_rw_a)) <= s_rw_d;
        end if;
    end process p_ring;

    p_cram : process(clk)
    begin
        if rising_edge(clk) then
            s_cq <= cram(to_integer(s_cra));
            if s_cwe = '1' then cram(to_integer(s_cwa)) <= s_cwd; end if;
        end if;
    end process p_cram;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches and derived constants
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_d : signed(6 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_xs <= f_vstep_x(s_tid);
            s_ys <= f_vstep_y(s_tid);

            -- vblank zone: no avid for 1024 clocks (hblank is at most 370)
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            -- K1 Species (8 labels): the firmware loads the default as the label
            -- INDEX; honour it until the knob first reads a real position
            v_k := unsigned(registers_in(0));
            if v_k >= 8 then s_live1 <= '1'; end if;
            if s_live1 = '0' and v_k < 8 then
                s_spk <= v_k(2 downto 0);
            elsif s_warm /= 15 or ((v_k(6 downto 0) >= 12 or v_k(9 downto 7) = 0) and
                                   (v_k(6 downto 0) <= 115 or v_k(9 downto 7) = 7)) then
                s_spk <= v_k(9 downto 7);
            end if;
            -- K2 Grain, K4 Bands, K5 Ink: 16 zones, deadband
            v_k := unsigned(registers_in(1));
            if s_warm /= 15 or ((v_k(5 downto 0) >= 6 or v_k(9 downto 6) = 0) and
                                (v_k(5 downto 0) <= 57 or v_k(9 downto 6) = 15)) then
                s_j2 <= v_k(9 downto 6);
            end if;
            v_k := unsigned(registers_in(3));
            if s_warm /= 15 or ((v_k(5 downto 0) >= 6 or v_k(9 downto 6) = 0) and
                                (v_k(5 downto 0) <= 57 or v_k(9 downto 6) = 15)) then
                s_j4 <= v_k(9 downto 6);
            end if;
            v_k := unsigned(registers_in(4));
            if s_warm /= 15 or ((v_k(5 downto 0) >= 6 or v_k(9 downto 6) = 0) and
                                (v_k(5 downto 0) <= 57 or v_k(9 downto 6) = 15)) then
                s_j5 <= v_k(9 downto 6);
            end if;
            -- K3 Drift: 64 zones, deadband
            v_k := unsigned(registers_in(2));
            if s_warm /= 15 or ((v_k(3 downto 0) >= 3 or v_k(9 downto 4) = 0) and
                                (v_k(3 downto 0) <= 12 or v_k(9 downto 4) = 63)) then
                s_j3 <= v_k(9 downto 4);
            end if;
            -- K6 Skin (4 labels), same index rule
            v_k := unsigned(registers_in(5));
            if v_k >= 4 then s_live6 <= '1'; end if;
            if s_live6 = '0' and v_k < 4 then
                s_skk <= v_k(1 downto 0);
            elsif s_warm /= 15 or ((v_k(7 downto 0) >= 12 or v_k(9 downto 8) = 0) and
                                   (v_k(7 downto 0) <= 243 or v_k(9 downto 8) = 3)) then
                s_skk <= v_k(9 downto 8);
            end if;
            -- P12 Bloom glide (1/8 of the gap a field)
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            -- per field (zone entry: exactly once per vertical blank)
            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_sp <= s_spk; s_l2 <= s_j2; s_l3 <= s_j3; s_l4 <= s_j4; s_l5 <= s_j5;
                s_skin  <= s_skk;
                s_dark  <= registers_in(6)(0);
                s_rain  <= registers_in(6)(1);
                s_flip  <= registers_in(6)(2);
                s_shine <= registers_in(6)(3);
                s_black <= registers_in(6)(4);
                s_dtq   <= f_dt(s_tid);
            end if;

            -- derived (one op per register; they settle within a few clocks)
            s_bq <= s_g(11 downto 6);
            s_boost <= to_unsigned(218, 9) + resize(s_bq, 9) + resize(s_bq(5 downto 2), 9);  -- 0.85 .. 1.16
            s_fb <= to_unsigned(C_F(to_integer(s_sp)), 8) * s_boost;                         -- F x boost (x 256)
            s_f0 <= resize(f_xdt(s_fb(16 downto 8), s_dtq), 11);                               -- F dt, Q0.12
            s_k0 <= f_xdt(to_unsigned(C_K(to_integer(s_sp)), 10), s_dtq);
            -- per-cell feed Fc = F dt (0.6 + 0.8 L / 256): A0 = 0.625 F dt, A1 = 0.8125 F dt
            s_a0 <= resize(shift_right(s_f0, 1) + shift_right(s_f0, 3), 10);
            s_a1 <= resize(s_f0 - shift_right(s_f0, 2) + shift_right(s_f0, 4), 10);
            -- drift: 3 levels each side of a centre deadband
            v_d := signed(resize(s_l3, 7)) - 32;
            if v_d < 0 then s_dneg <= '1'; v_d := -v_d; else s_dneg <= '0'; end if;
            if v_d < 4 then s_dlev <= "00"; elsif v_d < 14 then s_dlev <= "01"; elsif v_d < 24 then s_dlev <= "10"; else s_dlev <= "11"; end if;
            -- seed threshold 96 .. 17 and rain density (of 4096) 0 .. 62, x dt
            s_thr <= to_unsigned(96, 8) - resize(s_bq, 8) - resize(s_bq(5 downto 2), 8);
            s_bsq <= s_bq * s_bq;
            s_raind <= resize(f_xdt(resize(s_bsq(11 downto 6), 10), s_dtq), 12);
            -- grain: cell = 4 + j2 (max 16)
            if s_l2 > 12 then
                s_cell2 <= to_unsigned(32, 6); s_recip <= to_unsigned(C_RECIP(12), 6);
            else
                s_cell2 <= shift_left(to_unsigned(4, 6) + resize(s_l2, 6), 1); s_recip <= to_unsigned(C_RECIP(to_integer(s_l2)), 6);
            end if;
            if s_l2 < 2 then s_gys <= "01"; elsif s_l2 < 8 then s_gys <= "10"; else s_gys <= "11"; end if;
            -- bands 2 .. 9
            s_bands <= to_unsigned(2, 4) + resize(s_l4(3 downto 1), 4);
            s_bstep <= to_unsigned(C_STEP(to_integer(s_l4(3 downto 1))), 10);
            s_inkw <= s_l5(3 downto 2);
            -- bead rim band: 160 .. 640 below the bead threshold
            s_trim <= to_unsigned(C_TIN - 160, 12) - shift_left(resize(s_l5(3 downto 2), 12), 7) - shift_left(resize(s_l5(3 downto 2), 12), 5);

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Line events: hsync->avid parity vote, virtual line, first-line flag
    ---------------------------------------------------------------------------
    p_line : process(clk)
    begin
        if rising_edge(clk) then
            s_hsq <= data_in.hsync_n;
            if s_hsq = '1' and data_in.hsync_n = '0' then s_ph <= '0'; else s_ph <= not s_ph; end if;
            if data_in.avid = '1' and s_avid_sr(0) = '0' then
                s_swl <= s_ph xor s_upar;
                if s_ph = '1' then
                    if s_upc /= 15 then s_upc <= s_upc + 1; end if;
                else
                    if s_upc /= 0 then s_upc <= s_upc - 1; end if;
                end if;
            end if;
            if s_upc = 15 then s_upar <= '1'; elsif s_upc = 0 then s_upar <= '0'; end if;

            if s_zone = '1' then
                s_vl <= (others => '0');
            elsif data_in.avid = '0' and s_avid_sr(0) = '1' then
                s_vl <= s_vl + s_ys;
            end if;

            -- first line of a field: everything above reads as empty.  Cleared
            -- at the input avid rise of the second line (the first line's last
            -- cells are stepped in its blanking).
            if s_zgo = '1' then
                s_first <= '1';
                s_farm <= '0';
            elsif data_in.avid = '1' and s_avid_sr(0) = '0' then
                if s_farm = '1' then s_first <= '0'; end if;
                s_farm <= '1';
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Cells (stage 2): DDA on the virtual canvas, state / luma windows, seeds,
    -- the two update slots and the state RAM reads / hand-off writes
    ---------------------------------------------------------------------------
    p_cell : process(clk)
        variable v_n   : unsigned(6 downto 0);
        variable v_d1, v_d2 : signed(8 downto 0);
        variable v_e   : unsigned(9 downto 0);
        variable v_h   : unsigned(15 downto 0);
        variable v_l, v_c, v_r : std_logic_vector(23 downto 0);
        variable v_pre : std_logic;
    begin
        if rising_edge(clk) then
            -- a new line is entering the pipe (stages 0..3): blank cells of the
            -- old line stop, and cells 0 / 1 are primed
            v_pre := (data_in.avid or s_avid_sr(0) or s_avid_sr(1) or s_avid_sr(2)) and not s_avid_sr(3);

            -- DDA (stage 4): s_ck = cell of the stage-4 pixel, s_cs = it starts that
            -- cell.  Runs through blanking (blank cells flush the update lag).
            s_cs <= '0';
            if s_avid_sr(2) = '1' and s_avid_sr(3) = '0' then
                s_cx <= (others => '0');
                s_ck <= (others => '0');
                s_cs <= '1';
            elsif v_pre = '0' then
                v_n := resize(s_cx, 7) + s_xs;
                if v_n >= s_cell2 then
                    s_cx <= resize(v_n - s_cell2, 6);
                    if s_ck /= 1023 then
                        s_ck <= s_ck + 1;
                        s_cs <= '1';
                    end if;
                else
                    s_cx <= v_n(5 downto 0);
                end if;
            end if;
            -- cell luma: fast IIR over the pixels entering stage 4
            s_ly <= s_ly - shift_right(s_ly, 1) + shift_right(s_y8(3), 1);

            -- rain hash of (cell, virtual line), free-running, sampled at each
            -- cell start (a fixed offset per mode, so it stays put on a still
            -- picture).  Key = 37 cell + 155 line; two add / xor-shift rounds;
            -- the top 12 bits are compared (spot-checked: no lattice).
            v_h := resize(s_vl(18 downto 8), 16);
            s_vla <= shift_left(v_h, 7) + shift_left(v_h, 4);
            s_vlb <= shift_left(v_h, 3) + shift_left(v_h, 1) + v_h;
            s_vlh <= s_vla + s_vlb;
            s_ck37 <= shift_left(resize(s_ck, 16), 5) + shift_left(resize(s_ck, 16), 2) + resize(s_ck, 16);
            s_hk <= (s_ck37 + s_vlh) xor x"3C6E";
            v_h := s_hk + shift_left(s_hk, 7);
            s_hr1 <= v_h xor shift_right(v_h, 3);
            v_h := s_hr1 + shift_left(s_hr1, 9);
            s_hr <= v_h xor shift_right(v_h, 5);

            s_slA <= '0';
            s_slB <= s_slA;
            s_sgo <= '0';
            s_owe <= '0';
            if data_in.avid = '1' and s_avid_sr(0) = '0' then
                -- prime: old state of cell 0
                s_mra <= (others => '0'); s_ora <= (others => '0');
            elsif s_avid_sr(0) = '1' and s_avid_sr(1) = '0' then
                -- prime: old state of cell 1
                s_mra <= to_unsigned(1, 10); s_ora <= to_unsigned(1, 10);
            elsif s_avid_sr(1) = '1' and s_avid_sr(2) = '0' then
                -- cell 0's old state has arrived
                w_old(0) <= s_mq(31 downto 8);
                for i in 1 to 4 loop w_old(i) <= w_old(i - 1); end loop;
                w_or(0) <= s_oq;
                for i in 1 to 3 loop w_or(i) <= w_or(i - 1); end loop;
            elsif s_cs = '1' then
                -- cell k = s_ck starts.  Reads: old state + {older v, luma} of k+2,
                -- hand-off of k-7.  Windows shift (q holds k+1 / k-8).
                s_mra <= s_ck + 2;
                s_ora <= s_ck + 2;
                s_sra <= resize(s_ck - 7, 8);
                w_old(0) <= s_mq(31 downto 8);
                for i in 1 to 4 loop w_old(i) <= w_old(i - 1); end loop;
                w_or(0) <= s_oq;
                for i in 1 to 3 loop w_or(i) <= w_or(i - 1); end loop;
                w_act(0) <= s_avid_sr(3);
                for i in 1 to 3 loop w_act(i) <= w_act(i - 1); end loop;
                w_s1(0) <= s_sq;
                w_s1(1) <= w_s1(0);
                w_s1(2) <= w_s1(1);
                -- cell k-1 just ended: its luma and seed (step to the left
                -- neighbour and to the cell above)
                w_lum(0) <= s_ly;
                w_lum(1) <= w_lum(0);
                w_lum(2) <= w_lum(1);
                if s_first = '1' then
                    s_e1d <= (others => '0');
                else
                    s_e1d <= signed(resize(s_ly, 9)) - signed(resize(unsigned(w_or(1)(7 downto 0)), 9));
                end if;
                s_e2d <= signed(resize(s_ly, 9)) - signed(resize(w_lum(0), 9));
                if s_rain = '1' and s_hr(15 downto 4) < s_raind then s_erain <= '1'; else s_erain <= '0'; end if;
                s_sgo <= '1';
                if s_ck >= 2 then s_slA <= '1'; end if;
                s_kA <= s_ck - 2;
                if s_ck >= 9 then s_kB <= s_ck - 9; else s_kB <= (others => '1'); end if;
            end if;

            -- seed of the cell that just ended (cell start + 1): |left step| +
            -- |vertical step| over the Bloom threshold, or Rain
            if s_sgo = '1' then
                v_d1 := s_e1d; v_d2 := s_e2d;
                if v_d1 < 0 then v_d1 := -v_d1; end if;
                if v_d2 < 0 then v_d2 := -v_d2; end if;
                v_e := unsigned(resize(v_d1, 10)) + unsigned(resize(v_d2, 10));
                w_sd(1) <= w_sd(0);
                if v_e > s_thr or s_erain = '1' then w_sd(0) <= '1'; else w_sd(0) <= '0'; end if;
            end if;

            -- slot A (cell start + 1): sub-step 1 of cell c = k-2 from the old
            -- state (left w_old(4), centre w_old(3), right w_old(2)); seed it;
            -- record its v and luma for the line below
            o0 <= C_OP0;
            if s_slA = '1' then
                v_l := w_old(4); v_c := w_old(3); v_r := w_old(2);
                if s_first = '1' or w_act(2) = '0' then v_c := x"FFF000"; end if;
                if s_first = '1' or w_act(3) = '0' or s_kA = 0 then v_l := x"FFF000"; end if;
                if s_first = '1' or w_act(1) = '0' then v_r := x"FFF000"; end if;
                if w_sd(0) = '1' and w_act(2) = '1' then
                    if unsigned(v_c(11 downto 0)) < 2048 then v_c(11 downto 0) := x"800"; end if;
                    if unsigned(v_c(23 downto 12)) > 2048 then v_c(23 downto 12) := x"800"; end if;
                end if;
                o0.v <= '1'; o0.kb <= '0'; o0.idx <= s_kA;
                o0.uc <= unsigned(v_c(23 downto 12)); o0.vc <= unsigned(v_c(11 downto 0));
                o0_ul <= unsigned(v_l(23 downto 12)); o0_vl <= unsigned(v_l(11 downto 0));
                o0_ur <= unsigned(v_r(23 downto 12)); o0_vr <= unsigned(v_r(11 downto 0));
                if w_act(2) = '0' then
                    o0.lum <= (others => '0');
                elsif s_dark = '1' then
                    o0.lum <= not w_lum(1);
                else
                    o0.lum <= w_lum(1);
                end if;
                s_owe <= '1';
                s_owa <= s_kA;
                s_owd <= v_c(11 downto 4) & std_logic_vector(w_lum(1));
            -- slot B (cell start + 2): sub-step 2 of cell m = k-9 from the
            -- hand-off (left w_s1(2), centre w_s1(1), right w_s1(0))
            elsif s_slB = '1' and s_kB /= 1023 then
                v_l := w_s1(2)(31 downto 8); v_c := w_s1(1)(31 downto 8); v_r := w_s1(0)(31 downto 8);
                if s_kB = 0 then v_l := x"FFF000"; end if;
                o0.v <= '1'; o0.kb <= '1'; o0.idx <= s_kB;
                o0.uc <= unsigned(v_c(23 downto 12)); o0.vc <= unsigned(v_c(11 downto 0));
                o0_ul <= unsigned(v_l(23 downto 12)); o0_vl <= unsigned(v_l(11 downto 0));
                o0_ur <= unsigned(v_r(23 downto 12)); o0_vr <= unsigned(v_r(11 downto 0));
                o0.lum <= unsigned(w_s1(1)(7 downto 0));
            end if;
        end if;
    end process p_cell;

    ---------------------------------------------------------------------------
    -- Gray-Scott update pipeline (7 stages, one op per clock):
    --   u' = u + Du lap u - u v^2 + F (1 - u)
    --   v' = v + Dv lap v + u v^2 - (F + k) v       (dt folded into Du, Dv, F, k)
    -- F per cell from its luma; drift adds an asymmetric term to both laplacians.
    ---------------------------------------------------------------------------
    p_op : process(clk)
        variable v_t  : signed(15 downto 0);
        variable v_a  : signed(16 downto 0);
        variable v_p1, v_p2, v_q1, v_q2 : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- X1: laplacians, neighbour differences (drift sign applied), 1 - u
            o1 <= o0;
            x1_lu <= signed(resize(o0_ul, 14)) + signed(resize(o0_ur, 14)) - signed(resize(o0.uc & '0', 14));
            x1_lv <= signed(resize(o0_vl, 14)) + signed(resize(o0_vr, 14)) - signed(resize(o0.vc & '0', 14));
            if s_dneg = '1' then
                x1_du <= signed(resize(o0_ur, 14)) - signed(resize(o0_ul, 14));
                x1_dv <= signed(resize(o0_vr, 14)) - signed(resize(o0_vl, 14));
            else
                x1_du <= signed(resize(o0_ul, 14)) - signed(resize(o0_ur, 14));
                x1_dv <= signed(resize(o0_vl, 14)) - signed(resize(o0_vr, 14));
            end if;
            x1_nu <= 255 - o0.uc(11 downto 4);
            -- X2: v^2 (8-bit); diffusion Du = 0.19 dt, Dv = Du / 2 and drift as
            -- shift pairs (one add of two selected shifts); feed x luma
            o2 <= o1;
            x2_vv <= o1.vc(11 downto 4) * o1.vc(11 downto 4);
            -- Du x dt: 0.19 / 0.28 / 0.375 as (a >> s1) + (a >> s2); one add each
            case s_dtq is
                when "00"   => v_p1 := shift_right(x1_lu, 3); v_p2 := shift_right(x1_lu, 4);
                               v_q1 := shift_right(x1_lv, 4); v_q2 := shift_right(x1_lv, 5);
                when "01"   => v_p1 := shift_right(x1_lu, 2); v_p2 := shift_right(x1_lu, 5);
                               v_q1 := shift_right(x1_lv, 3); v_q2 := shift_right(x1_lv, 6);
                when others => v_p1 := shift_right(x1_lu, 2); v_p2 := shift_right(x1_lu, 3);
                               v_q1 := shift_right(x1_lv, 3); v_q2 := shift_right(x1_lv, 4);
            end case;
            x2_pu <= v_p1 + v_p2;
            x2_pv <= v_q1 + v_q2;
            case s_dlev is
                when "00"   => x2_au <= (others => '0'); x2_av <= (others => '0');
                when "01"   => x2_au <= shift_right(x1_du, 5); x2_av <= shift_right(x1_dv, 5);
                when "10"   => x2_au <= shift_right(x1_du, 4); x2_av <= shift_right(x1_dv, 4);
                when others => x2_au <= shift_right(x1_du, 3); x2_av <= shift_right(x1_dv, 3);
            end case;
            x2_ml <= s_a1 * o1.lum;
            x2_nu <= x1_nu;
            -- X3: u v^2 (8-bit, Q0.16); F; diffusion + drift
            o3 <= o2;
            x3_r <= o2.uc(11 downto 4) * x2_vv(15 downto 8);
            x3_fc <= resize(s_a0, 11) + resize(x2_ml(17 downto 8), 11);
            x3_dfu <= resize(x2_pu, 15) + resize(x2_au, 15);
            x3_dfv <= resize(x2_pv, 15) + resize(x2_av, 15);
            x3_nu <= x2_nu;
            -- X4: reaction x dt (Q0.12); feed term; F + k
            o4 <= o3;
            x4_rd <= f_xdt(x3_r(15 downto 4), s_dtq);
            x4_fu <= x3_fc * x3_nu;
            x4_g <= resize(x3_fc, 12) + resize(s_k0, 12);
            x4_dfu <= x3_dfu; x4_dfv <= x3_dfv;
            -- X5: kill term; du
            o5 <= o4;
            x5_gv <= x4_g * o4.vc(11 downto 4);
            v_t := resize(x4_dfu, 16) - signed(resize(x4_rd, 16)) + signed(resize(x4_fu(18 downto 8), 16));
            x5_du <= v_t;
            x5_rd <= x4_rd; x5_dfv <= x4_dfv;
            -- X6: dv; u'
            o6 <= o5;
            x6_dv <= resize(x5_dfv, 16) + signed(resize(x5_rd, 16)) - signed(resize(x5_gv(19 downto 8), 16));
            v_a := signed(resize(o5.uc, 17)) + resize(x5_du, 17);
            if v_a < 0 then x6_u <= (others => '0');
            elsif v_a > 4095 then x6_u <= (others => '1');
            else x6_u <= unsigned(v_a(11 downto 0)); end if;
            -- X7: v'; write sub-step 1 to the hand-off RAM, sub-step 2 to the state RAM
            v_a := signed(resize(o6.vc, 17)) + resize(x6_dv, 17);
            if v_a < 0 then v_a := (others => '0'); elsif v_a > 4095 then v_a := to_signed(4095, 17); end if;
            s_swe <= o6.v and not o6.kb;
            s_swa <= o6.idx(7 downto 0);
            s_swd <= std_logic_vector(x6_u) & std_logic_vector(v_a(11 downto 0)) & std_logic_vector(o6.lum);
            s_mwe <= o6.v and o6.kb;
            s_mwa <= o6.idx;
            s_mwd <= std_logic_vector(x6_u) & std_logic_vector(v_a(11 downto 0)) & x"00";
        end if;
    end process p_op;

    ---------------------------------------------------------------------------
    -- Pixel path: display of the previous line's field, cel ground, compose
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_f   : signed(13 downto 0);
        variable v_lt  : signed(12 downto 0);
        variable v_fl  : unsigned(11 downto 0);
        variable v_ch  : std_logic;
        variable v_cn  : unsigned(1 downto 0);
        variable v_y   : unsigned(10 downto 0);
        variable v_c   : signed(11 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_dl  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: input; column
            s_y1 <= unsigned(data_in.y);
            s_u1 <= unsigned(data_in.u);
            s_v1 <= unsigned(data_in.v);
            if data_in.avid = '1' and s_avid_sr(0) = '0' then
                s_rx(1) <= (others => '0');
            elsif s_rx(1) /= 2047 then
                s_rx(1) <= s_rx(1) + 1;
            end if;
            for i in 2 to 12 loop s_rx(i) <= s_rx(i - 1); end loop;
            -- 2: picture into the ring; luma
            s_rw_a <= s_vw;
            s_vw <= s_vw + 1;
            s_rw_d <= std_logic_vector(s_y1) & std_logic_vector(s_u1) & std_logic_vector(s_v1) & "00";
            s_y8(2) <= s_y1(9 downto 2);
            for i in 3 to 9 loop s_y8(i) <= s_y8(i - 1); end loop;

            -- 5..6: this pixel's cell k (the windows shift as a cell's first pixel
            -- leaves stage 4): old v of k and k+1, the line above's v of k
            s_cx5 <= s_cx;
            s_cx6 <= s_cx5;
            if s_first = '1' then
                s_vc6 <= (others => '0'); s_vn6 <= (others => '0'); s_vo6 <= (others => '0');
            else
                s_vc6 <= unsigned(w_old(1)(11 downto 0));
                s_vn6 <= unsigned(w_old(0)(11 downto 0));
                s_vo6 <= unsigned(w_or(1)(15 downto 8));
            end if;
            -- 7: fraction across the cell; slopes; cel band; cel column read
            s_fr7 <= s_cx6 * s_recip;
            s_dd7 <= signed(resize(s_vn6, 13)) - signed(resize(s_vc6, 13));
            s_gs7 <= signed(resize(s_vc6(11 downto 4), 12)) - signed(resize(s_vo6, 12));   -- vertical (v >> 4 units)
            s_vc7 <= s_vc6;
            s_q7 <= s_y8(6) * s_bands;
            s_cra <= s_rx(6);
            -- 8: interpolation product; shading slope
            s_pr8 <= resize(s_dd7(12 downto 2) * signed('0' & s_fr7(7 downto 2)), 22);   -- (dd fr) >> 4
            s_gs8 <= shift_left(s_gs7, to_integer(s_gys)) + resize(shift_right(s_dd7, 4), 12);
            s_vc8 <= s_vc7;
            s_q8 <= s_q7(11 downto 8);
            -- 9: field value; cel luma; cel column word
            v_f := signed(resize(s_vc8, 14)) + resize(shift_right(s_pr8, 4), 14);
            if v_f < 0 then s_f9 <= (others => '0'); elsif v_f > 4095 then s_f9 <= (others => '1'); else s_f9 <= unsigned(v_f(11 downto 0)); end if;
            s_gs9 <= s_gs8;
            s_yc9 <= resize(to_unsigned(64, 11) + resize(s_q8 * s_bstep, 11), 10);
            s_q9 <= s_q8;
            s_cq9 <= s_cq;
            -- 10: bead / rim (Flip inverts the field); light; cel outlines (a run
            -- of Ink px across, a countdown of Ink lines down in the column RAM)
            v_fl := s_f9;
            if s_flip = '1' then v_fl := to_unsigned(2048, 12) - resize(s_f9(11 downto 1), 12); end if;
            if v_fl > C_TIN then s_bead10 <= '1'; else s_bead10 <= '0'; end if;
            if v_fl > s_trim and v_fl <= C_TIN then s_rim10 <= '1'; else s_rim10 <= '0'; end if;
            v_lt := to_signed(128, 13) + resize(shift_right(s_gs9, 1), 13);
            if v_lt < 0 then s_lt10 <= (others => '0'); elsif v_lt > 255 then s_lt10 <= (others => '1'); else s_lt10 <= unsigned(v_lt(7 downto 0)); end if;
            v_ch := '0';
            if s_q9 /= s_q10 then v_ch := '1'; end if;
            if s_first = '0' and s_q9 /= unsigned(s_cq9(7 downto 4)) then v_ch := '1'; end if;
            if v_ch = '1' and s_inkw /= 0 then
                s_hrun <= s_inkw;
                v_cn := s_inkw;
                s_ink10 <= '1';
            else
                if s_hrun /= 0 then s_hrun <= s_hrun - 1; end if;
                v_cn := unsigned(s_cq9(3 downto 2));
                if v_cn /= 0 then v_cn := v_cn - 1; end if;
                if s_hrun /= 0 or unsigned(s_cq9(3 downto 2)) /= 0 then s_ink10 <= '1'; else s_ink10 <= '0'; end if;
            end if;
            s_q10 <= s_q9;
            s_yc10 <= s_yc9;
            s_cwa <= s_rx(9);
            s_cwd <= std_logic_vector(s_q9) & std_logic_vector(v_cn) & "00";
            s_cwe <= s_avid_sr(8);
            -- 11: highlight; picture ring read (written at stage 2, 9 clocks back)
            if s_shine = '1' and s_bead10 = '1' and s_lt10 > 200 then s_fl(11)(3) <= '1'; else s_fl(11)(3) <= '0'; end if;
            s_fl(11)(2 downto 0) <= s_bead10 & s_rim10 & s_ink10;
            s_lt(11) <= s_lt10;
            s_yc11 <= s_yc10;
            s_rr_a <= s_vw - 9;
            -- 12, 13: ring output -> fabric
            s_fl(12) <= s_fl(11); s_fl(13) <= s_fl(12);
            s_lt(12) <= s_lt(11); s_lt(13) <= s_lt(12);
            s_yc12 <= s_yc11; s_yc13 <= s_yc12;
            s_pic13 <= s_rr_q;
            -- 14: ground and bead colours for the skin (generated chroma swaps
            -- U/V on lines off the usual hsync parity)
            s_py14 <= resize(unsigned(s_pic13(31 downto 22)) * s_lt(13)(7 downto 3), 18);
            case s_skin is
                when "00" =>      -- Pearl: dim cel ground, beads in the picture's colours
                    s_gy14 <= to_unsigned(64, 10) + shift_right(s_yc13 - 64, 2) + shift_right(s_yc13 - 64, 3);
                    s_gu14 <= to_unsigned(256, 10) + shift_right(unsigned(s_pic13(21 downto 12)), 1);
                    s_gv14 <= to_unsigned(256, 10) + shift_right(unsigned(s_pic13(11 downto 2)), 1);
                    v_c := signed(resize(unsigned(s_pic13(21 downto 12)), 12)) - 512;
                    v_c := 512 + v_c + shift_right(v_c, 2);
                    if v_c < 16 then v_c := to_signed(16, 12); elsif v_c > 1008 then v_c := to_signed(1008, 12); end if;
                    s_bu14 <= unsigned(v_c(9 downto 0));
                    v_c := signed(resize(unsigned(s_pic13(11 downto 2)), 12)) - 512;
                    v_c := 512 + v_c + shift_right(v_c, 2);
                    if v_c < 16 then v_c := to_signed(16, 12); elsif v_c > 1008 then v_c := to_signed(1008, 12); end if;
                    s_bv14 <= unsigned(v_c(9 downto 0));
                when "01" =>      -- Toon: cel ground (flattened chroma), flat lifted beads
                    s_gy14 <= s_yc13;
                    s_gu14 <= (unsigned(s_pic13(21 downto 12)) + 32) and "1111000000";
                    s_gv14 <= (unsigned(s_pic13(11 downto 2)) + 32) and "1111000000";
                    s_bu14 <= unsigned(s_pic13(21 downto 12));
                    s_bv14 <= unsigned(s_pic13(11 downto 2));
                when "10" =>      -- Coat: tawny cel coat, dark brown spots
                    s_gy14 <= s_yc13;
                    if s_swl = '1' then s_gu14 <= to_unsigned(330, 10); s_gv14 <= to_unsigned(600, 10);
                    else s_gu14 <= to_unsigned(600, 10); s_gv14 <= to_unsigned(330, 10); end if;
                    if s_swl = '1' then s_bu14 <= to_unsigned(470, 10); s_bv14 <= to_unsigned(560, 10);
                    else s_bu14 <= to_unsigned(560, 10); s_bv14 <= to_unsigned(470, 10); end if;
                when others =>    -- Ink: warm paper, black blots
                    s_gy14 <= to_unsigned(600, 10) + shift_right(s_yc13 - 64, 3);
                    if s_swl = '1' then s_gu14 <= to_unsigned(496, 10); s_gv14 <= to_unsigned(528, 10);
                    else s_gu14 <= to_unsigned(528, 10); s_gv14 <= to_unsigned(496, 10); end if;
                    s_bu14 <= to_unsigned(512, 10); s_bv14 <= to_unsigned(512, 10);
            end case;
            if s_fl(13)(3) = '1' then s_k14 <= "100";
            elsif s_fl(13)(2) = '1' then s_k14 <= "001";
            elsif s_fl(13)(1) = '1' then s_k14 <= "010";
            elsif s_fl(13)(0) = '1' then s_k14 <= "011";
            else s_k14 <= "000"; end if;
            s_yc14 <= s_yc13;
            -- 15: compose
            case s_k14 is
                when "100" =>     -- highlight
                    s_y15 <= to_unsigned(780, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
                when "001" =>     -- bead
                    case s_skin is
                        when "00" =>
                            v_y := resize(shift_right(s_py14, 4), 11) + 90;
                            if v_y > 800 then v_y := to_unsigned(800, 11); end if;
                            s_y15 <= v_y(9 downto 0);
                        when "01" =>
                            v_y := resize(s_yc14, 11) + 160;
                            if v_y > 800 then v_y := to_unsigned(800, 11); end if;
                            s_y15 <= v_y(9 downto 0);
                        when "10" =>
                            s_y15 <= to_unsigned(64, 10) + shift_right(s_yc14 - 64, 2);
                        when others =>
                            if s_black = '1' then s_y15 <= to_unsigned(760, 10); else s_y15 <= to_unsigned(64, 10); end if;
                    end case;
                    s_u15 <= s_bu14; s_v15 <= s_bv14;
                when "010" =>     -- rim
                    if s_skin = "11" and s_black = '1' then s_y15 <= to_unsigned(760, 10); else s_y15 <= to_unsigned(64, 10); end if;
                    s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
                when "011" =>     -- cel outline
                    if s_black = '1' then s_y15 <= to_unsigned(64, 10);
                    else s_y15 <= to_unsigned(64, 10) + shift_right(s_gy14 - 64, 3); end if;
                    s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
                when others =>    -- ground
                    if s_black = '1' then
                        s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
                    else
                        s_y15 <= s_gy14; s_u15 <= s_gu14; s_v15 <= s_gv14;
                    end if;
            end case;

            -- 16..21: output conditioning.  Chroma [1 2 2 2 2 2 2 2 1]/16 as
            -- two adjacent 8-sums (centre at stage 20 -> s_u21); luma [1 2 1]
            -- centred at stage 20 -> s_yf21.
            s_csu(0) <= s_u15; s_csv(0) <= s_v15;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u15 - s_csu(7);
            s_boxv <= s_boxv + s_v15 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u21 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v21 <= v_sc(13 downto 4);
            s_y16 <= s_y15; s_y17 <= s_y16; s_y18 <= s_y17; s_y19 <= s_y18;
            s_y20 <= s_y19; s_y21x <= s_y20;
            v_sy := resize(s_y19, 12) + shift_left(resize(s_y20, 12), 1) + resize(s_y21x, 12);
            s_yf21 <= v_sy(11 downto 2);
            -- 22: luma toward black, 64 + 13/16 (y - 64); blanking gate
            if s_yf21 > 64 then v_dl := s_yf21 - 64; else v_dl := (others => '0'); end if;
            if s_avid_sr(20) = '1' then
                s_yo(22) <= s_yf21 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_uo(22) <= s_u21; s_vo(22) <= s_v21;
            else
                s_yo(22) <= to_unsigned(64, 10); s_uo(22) <= to_unsigned(512, 10); s_vo(22) <= to_unsigned(512, 10);
            end if;
            -- 23..24: pad to the 24-clock latency
            for i in 23 to 24 loop
                s_yo(i) <= s_yo(i - 1); s_uo(i) <= s_uo(i - 1); s_vo(i) <= s_vo(i - 1);
            end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_yo(24));
    data_out.u       <= std_logic_vector(s_uo(24));
    data_out.v       <= std_logic_vector(s_vo(24));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture morphogen;
