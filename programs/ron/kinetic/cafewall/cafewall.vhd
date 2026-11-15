-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Cafewall v0.1.1 -- Kinetic.
--
-- The cafe wall illusion (Gregory and Heard, 1979): rows of alternating dark and light tiles, each row shifted
-- against the next, with thin mid-tone mortar lines between the rows.  The mortar lines are parallel and
-- straight but look like long wedges.
--
--   element  a row of tiles (height H, tile width W = H * aspect); state = its horizontal shift
--   theta    = Spread * index - phase          index: Linear = row number; Radial = |row number|
--   shift    = SHIFT * wave(theta) tiles (0 .. 2), negated on odd rows with Rows = Alternate; odd rows start
--              one tile over, so shift 0 is a checkerboard
--   Radial   also mirrors the row about the screen centre (tile coordinate = |x - centre| + shift)
--   mortar   the first M virtual lines of each row (at least 2 field lines), tone between the two tile tones,
--              blended by line coverage in eighths; the row's first line is always all mortar
--   tiles    a counter in 1/16 virtual px runs along the row; the pixel holding a tile edge blends the two
--              tiles by the edge's position in it (16 steps)
--   source   luma (or the Osc lattice) filtered 1/16 along x, sampled at each tile's last pixel on the row's
--              first line (all mortar) into a RAM indexed by tile number and held for the row: 16 tone steps
--              per tile from a per-field table (ground tiles toward A, A tiles toward B); the step is the
--              sample through the Camera law (see K6)
--
--   K1  Scale    row height 24 .. 288 virtual lines (exponential, glided)
--   K2  Shape    tile aspect 1 .. 4 (squares .. long bricks)
--   K3  Mortar   thickness 2.5 % .. 37 % of the row (square law) and tone 1/4 .. 3/4 between the tile tones
--   K4  Rate     bipolar, centre = still; 60 s per cycle .. 2 cycles per second
--   K5  Colour   the 16 shared palettes (ground and A = the tiles, B = the picture's tint, accent = Reveal's mortar)
--                [K6 before v0.1.1]
--   K6  Camera   the picture's tone strength (glided), g = k / 256: up to 25 % the step is g * (v0.1.0's step)
--                (0 = none, 25 % = v0.1.0's); above, the step's contrast is stretched g times about the middle
--                of the 16 steps (50 % = 2 x, 100 % = 4 x), clamped to the v0.1.0 range so the two tile sets
--                never meet (2 x in amplitude washed the wall to one grey)
--       (Spread, K5 before v0.1.1, is fixed at 420: phase step per row, one slow S down the wall)
--   S7  Wave     Smooth / Snap
--   S8  Field    Linear / Radial
--   S9  Rows     Same / Alternate
--   S10 Reveal   Off / On (tiles drop to two close tones, the mortar takes the accent colour)
--   S11 Source   Video / Osc
--   P12 SHIFT    0 .. 2 tiles (square law, glided): 50 % = half a tile
--
-- Sections (the Kinetic chassis, lifted from louver):
--   [A] timing ID -> virtual-1080 steps, vblank zone, width, knob glides, per-field sequencer (serial multiplier)
--   [B] generated avid
--   [C] source field: Osc lattice, luma filter, per-tile hold
--   [D] wave engine: row counter per line, per-line engine (wave ROM, shift, mortar coverage)
--   [E] the look: tile counter, edge blend, mortar blend
--   [F] output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1] with resync, luma 13/16, blanking gate
-- One latency (C_LAT = 22) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K5 = Colour (was K6); K5 Spread fixed at 420 (C_KFIX_5).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.cafewall_pkg.all;
use work.all;

architecture cafewall of program_top is
    constant C_KFIX_5 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(420, 10));   -- K6 camera pass: fixed knob
    constant C_LAT      : integer := 22;      -- = processing_delay_clks
    constant C_SIM_SNAP : boolean := false;   -- sim-only copies set this: knob glides jump straight to the knob
    constant C_DEF_K6   : integer := 512;     -- K6 Camera (TOML default; the glide starts there)

    -- [A] virtual-1080 steps from the timing ID: y step per field line (Q8), x step in half virtual px
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
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- 8 / (y step) in Q11: mortar coverage in eighths of a field line
    function f_rec(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(14, 7);
            when 4          => return to_unsigned(28, 7);
            when 8          => return to_unsigned(17, 7);
            when 12         => return to_unsigned(34, 7);
            when 5 | 6 | 14 => return to_unsigned(43, 7);
            when 1 | 2 | 10 => return to_unsigned(32, 7);
            when others     => return to_unsigned(64, 7);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    function f_gs(x : signed; b : std_logic; w : natural) return signed is
        variable r : signed(w - 1 downto 0) := (others => '0');
    begin
        if b = '1' then r := resize(x, w); end if;
        return r;
    end function;

    attribute no_rw_check : boolean;

    -- ROMs (one read port each, registered address and data, then a fabric register)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_w_a  : unsigned(9 downto 0) := (others => '0');
    signal r_w_q  : unsigned(15 downto 0) := (others => '0');
    signal r_pl_a : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    -- [C] per-tile luma hold: address = bank (2) & tile number (7)
    type t_lh is array (0 to 511) of std_logic_vector(7 downto 0);
    signal lh : t_lh := (others => x"00");
    attribute no_rw_check of lh : signal is true;
    signal lh_ra, lh_wa : unsigned(8 downto 0) := (others => '0');
    signal lh_q, lh_wd  : std_logic_vector(7 downto 0) := x"00";
    signal lh_we : std_logic := '0';
    -- [E] tile table, refilled every field: address = parity & luma step; {Y 10, U 8, V 8}
    type t_tt is array (0 to 255) of std_logic_vector(31 downto 0);
    signal tt : t_tt := (others => (others => '0'));
    attribute no_rw_check of tt : signal is true;
    signal tt_ra, tt_wa : unsigned(7 downto 0) := (others => '0');
    signal tt_q, tt_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal tt_we : std_logic := '0';
    -- delay rings (write at rc + d, read at rc, RAM register, fabric register: d + 2 stages)
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal cr : t_r16 := (others => (others => '0'));          -- chroma 8 back for the running 8-sums
    attribute no_rw_check of cr : signal is true;
    signal cr_q, cr_f : std_logic_vector(15 downto 0) := (others => '0');
    type t_r4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal sr : t_r4 := (others => (others => '1'));           -- syncs: C_LAT
    attribute no_rw_check of sr : signal is true;
    signal sr_q, sr_f : std_logic_vector(3 downto 0) := (others => '1');
    signal rc   : unsigned(7 downto 0) := (others => '0');

    type t_c3  is array (0 to 2) of unsigned(9 downto 0);      -- a colour: Y 10 bits, U, V 8 bits
    type t_d3  is array (0 to 2) of signed(10 downto 0);
    type t_p3  is array (0 to 2) of signed(12 downto 0);
    type t_a3  is array (0 to 2) of signed(17 downto 0);
    type t_e3  is array (0 to 2) of signed(12 downto 0);
    type t_mm3 is array (0 to 2) of unsigned(11 downto 0);
    type t_g3  is array (0 to 2) of unsigned(10 downto 0);
    constant C_NEUT : t_c3 := (to_unsigned(64, 10), to_unsigned(128, 10), to_unsigned(128, 10));

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_rec  : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_sd   : std_logic := '0';
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw rate, switches, palette
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(560 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(C_DEF_K6 * 8, 13);   -- K6 Camera
    signal c_kg : unsigned(9 downto 0) := to_unsigned(C_DEF_K6, 10);        -- its per-field value
    signal c_ko : unsigned(16 downto 0) := to_unsigned(120 * (C_DEF_K6 - 256), 17);   -- the stretch offset
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_alt, s_rev, s_src, s_fpar : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(0, 4);
    signal s_asp : unsigned(9 downto 0) := to_unsigned(406, 10);
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- field serial multiplier (24 iterations), serial shifter, sequencer scratch
    signal f_a, f_acc, z_fa : unsigned(47 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(23 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_e   : unsigned(10 downto 0) := (others => '0');
    signal z_sh  : unsigned(2 downto 0) := (others => '0');
    signal z_shn : unsigned(2 downto 0) := (others => '0');
    signal z_sht : std_logic := '0';
    signal z_mt  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_q   : unsigned(7 downto 0) := (others => '0');
    signal z_mf  : unsigned(8 downto 0) := to_unsigned(60, 9);
    signal z_m   : unsigned(16 downto 0) := (others => '0');
    signal z_loc : signed(18 downto 0) := (others => '0');
    signal z_k   : signed(7 downto 0) := (others => '0');
    signal z_nrun, z_first, z_xrun, z_xp : std_logic := '0';
    signal z_x   : signed(16 downto 0) := (others => '0');

    -- per-field constants
    signal c_pw   : unsigned(19 downto 0) := to_unsigned(23040 * 8, 20);   -- row height, Q11 (c_p = Q8 virtual lines)
    signal c_p    : unsigned(16 downto 0);
    signal c_w    : unsigned(14 downto 0) := to_unsigned(2400, 15);    -- tile width, Q4 virtual px
    signal c_m    : unsigned(16 downto 0) := to_unsigned(2048, 17);    -- mortar, Q8 virtual lines
    signal c_d    : unsigned(9 downto 0) := (others => '0');           -- shift amplitude, Q8 tiles
    signal c_s    : unsigned(22 downto 0) := (others => '0');          -- spread per row, Q24 cycles
    signal c_yst  : unsigned(11 downto 0) := to_unsigned(256, 12);     -- row step per line, Q8
    signal c_ymp  : signed(18 downto 0) := (others => '0');
    signal c_pxq, c_nxq, c_wmx, c_xmw : signed(16 downto 0) := (others => '0');   -- tile counter steps, Q4
    signal c_hm1  : unsigned(11 downto 0) := to_unsigned(959, 12);     -- the pixel before the screen centre
    signal c_base : unsigned(14 downto 0) := (others => '0');          -- tile counter at pixel 0 with no shift
    signal c_bpar : std_logic := '0';
    signal c_ph   : unsigned(25 downto 0) := (others => '0');          -- wave phase, Q26 cycles
    signal c_dph  : unsigned(21 downto 0) := (others => '0');
    signal c_g, c_a, c_b, c_c : t_c3 := (others => (others => '0'));   -- palette: ground, A, B, accent
    signal c_mo   : t_c3 := C_NEUT;                                    -- the mortar colour

    -- tile table fill
    signal t_st  : unsigned(2 downto 0) := (others => '0');
    signal t_l   : unsigned(3 downto 0) := (others => '0');
    signal t_mc  : unsigned(4 downto 0) := (others => '0');
    signal t_da, t_d1 : t_d3 := (others => (others => '0'));
    signal t_ga  : t_g3 := (others => (others => '0'));
    signal t_e0  : t_e3 := (others => (others => '0'));
    signal t_m3  : t_mm3 := (others => (others => '0'));
    signal t_a0, t_a1, t_ma : t_a3 := (others => (others => '0'));
    signal t_w0, t_w1 : t_c3 := (others => (others => '0'));

    -- [B] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- [C] source
    signal s_ox, s_oy : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_ov, s_sv : unsigned(7 downto 0) := (others => '0');
    signal s_dd  : unsigned(9 downto 0) := (others => '0');
    signal s_av0, s_sav, s_savd : std_logic := '0';
    signal s_y1  : unsigned(13 downto 0) := (others => '0');
    signal s_ymv : unsigned(7 downto 0) := (others => '0');     -- the filtered sample (Osc: x 2, clipped)
    signal s_ym  : unsigned(17 downto 0) := (others => '0');    -- ... x Camera (/ 4096 = the v0.1.0 step x g)
    signal s_yl  : unsigned(3 downto 0) := (others => '0');     -- the tone step after the stretch and clamp

    -- [D] row registers (the NEXT line's values) and the per-line engine
    signal ls_sk  : signed(27 downto 0) := (others => '0');
    signal ls_loc : unsigned(16 downto 0) := (others => '0');
    signal ls_k   : signed(7 downto 0) := (others => '0');
    signal ls_n1  : unsigned(16 downto 0) := (others => '0');
    signal ls_n2  : signed(18 downto 0) := (others => '0');
    signal ls_new : std_logic := '0';
    signal le_kick, le_run, le_neg, le_par, le_par0 : std_logic := '0';
    signal le_t   : unsigned(5 downto 0) := (others => '0');
    signal le_x, le_th : unsigned(13 downto 0) := (others => '0');
    signal le_d1  : signed(17 downto 0) := (others => '0');
    signal le_part : unsigned(10 downto 0) := (others => '0');
    signal le_w, le_frac : unsigned(9 downto 0) := (others => '0');
    signal le_c   : unsigned(15 downto 0) := (others => '0');
    signal le_cmw : signed(16 downto 0) := (others => '0');
    signal le_c0  : unsigned(14 downto 0) := (others => '0');
    signal le_cov : unsigned(3 downto 0) := (others => '0');
    signal m_a, m_acc : unsigned(24 downto 0) := (others => '0');
    signal m_b    : unsigned(9 downto 0) := (others => '0');

    -- [E] the look
    signal a_c    : unsigned(14 downto 0) := (others => '0');
    signal a_col  : unsigned(6 downto 0) := (others => '0');
    signal a_cov  : unsigned(3 downto 0) := (others => '0');
    signal a_par, a_dec, a_samp, a_top : std_logic := '0';
    signal q2_col : unsigned(6 downto 0) := (others => '0');
    signal q2_fn  : unsigned(5 downto 0) := (others => '0');
    signal q2_par, q2_cross, q3_par, q4_par, q5_par : std_logic := '0';
    signal q3_f, q4_f, q5_f, q6_f, q7_f, q8_f, q9_f, q10_f : unsigned(3 downto 0) := (others => '0');
    signal q5_l   : unsigned(3 downto 0) := (others => '0');
    signal q8_c, q9_c, q10_c, q11_c, q12_c, q13_c, q14_c : t_c3 := C_NEUT;
    signal q10_d, q13_d : t_d3 := (others => (others => '0'));
    signal q11_p01, q11_p23, q14_p01, q14_p23 : t_p3 := (others => (others => '0'));
    signal q15_c  : t_c3 := C_NEUT;

    -- [F] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_ysr  : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv, s_bud, s_bvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y31, s_y32, s_y33 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u31, s_v31, s_u32, s_v32, s_u33, s_v33 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    c_p <= c_pw(19 downto 3);

    ---------------------------------------------------------------------------
    -- Block RAM: ROMs, hold, table, delay rings (one write statement each)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_CW_EXP2(to_integer(r_ex_a));
            r_w_q  <= C_CW_SIN(to_integer(r_w_a));
            r_pl_q <= C_CW_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    p_lh : process(clk)
    begin
        if rising_edge(clk) then
            lh_q <= lh(to_integer(lh_ra));
            if lh_we = '1' then lh(to_integer(lh_wa)) <= lh_wd; end if;
        end if;
    end process p_lh;

    p_tt : process(clk)
    begin
        if rising_edge(clk) then
            tt_q <= tt(to_integer(tt_ra));
            if tt_we = '1' then tt(to_integer(tt_wa)) <= tt_wd; end if;
        end if;
    end process p_tt;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            cr(to_integer(rc + 6)) <= std_logic_vector(q15_c(1)(7 downto 0)) & std_logic_vector(q15_c(2)(7 downto 0));
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            sr(to_integer(rc + (C_LAT - 2))) <= '1' & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
            rc <= rc + 1;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- [A] Controls, vblank zone, glides, per-field sequencer (one small
    -- operation per zone clock; quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_b  : unsigned(25 downto 0);
        variable v_a27 : unsigned(26 downto 0);
        variable v_k3 : unsigned(11 downto 0);
        variable v_xq : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_rec <= f_rec(s_tid);
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
            if s_zone = '0' then
                s_zc <= (others => '0');
            elsif s_zc /= 2047 then
                s_zc <= s_zc + 1;
            end if;

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- Rate: bipolar with a +-8 deadband
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
            s_vneg <= v_s(10);
            if v_s > -9 and v_s < 9 then
                v_m := (others => '0');
            elsif v_s(10) = '1' then
                v_m := unsigned(not v_s(9 downto 0)) - 7;     -- |v| - 8
            else
                v_m := unsigned(v_s(9 downto 0)) - 8;
            end if;
            s_vm <= v_m(8 downto 0);

            -- Shape: aspect 1 + 3 k2 / 1024, Q8
            v_k3 := resize(unsigned(registers_in(1)) & '0', 12) + resize(unsigned(registers_in(1)), 12);
            s_asp <= to_unsigned(256, 10) + v_k3(11 downto 2);

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_snap <= registers_in(6)(0);
                s_rad  <= registers_in(6)(1);
                s_alt  <= registers_in(6)(2);
                s_rev  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_fpar <= not s_fpar;
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); s_sd <= '1'; c_hm1 <= to_unsigned(359, 12);
                    when 3      => s_oxs <= to_unsigned(153, 9); s_sd <= '0'; c_hm1 <= to_unsigned(639, 12);
                    when others => s_oxs <= to_unsigned(102, 9); s_sd <= '0'; c_hm1 <= to_unsigned(959, 12);
                end case;
                -- Colour: 16 palettes with hysteresis
                v_d := signed(resize(unsigned(registers_in(4)), 11)) - signed(resize(s_pal & "100000", 11));
                if v_d > 40 or v_d < -40 then s_pal <= unsigned(registers_in(4)(9 downto 6)); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide for Scale, SHIFT and Camera (1/8 of the gap per field, +-2 deadband, Camera +-4):
            -- four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        if z_gc(1) = '1' then s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                        elsif z_gc(0) = '0' then s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                        else s_gm <= s_gp; s_gr <= unsigned(registers_in(7)); end if;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if C_SIM_SNAP then
                            null;
                        elsif (s_gd > 16 or s_gd < -16) and (z_gc(1) = '0' or s_gd > 32 or s_gd < -32) then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        if z_gc(1) = '1' then s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        elsif z_gc(0) = '0' then s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        else s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0)); end if;
                        z_gp <= "00";
                        if z_gc = 2 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- K6 Camera: the glided value, latched once the glides are done (zone clock 24), and the stretch
            -- offset 7.5 (g - 1) steps = 120 (k - 256) in product units (k >= 256; 0 below)
            if s_zone = '1' and s_zc = 24 then c_kg <= s_g6(12 downto 3); end if;
            if s_zone = '1' and s_zc = 25 then
                if c_kg(9 downto 8) = "00" then c_ko <= (others => '0');
                else c_ko <= shift_left(resize(c_kg - 256, 17), 7) - shift_left(resize(c_kg - 256, 17), 3); end if;
            end if;

            -- serial products: slot k loads at zc = 32 + 32 k, 24 iterations, stored at 60 + 32 k.  The slot
            -- number is decoded at 30 + 32 k and the operands pre-muxed at 31 + 32 k.
            z_pre <= '0';
            if s_zone = '1' and s_zc(4 downto 0) = 30 and s_zc(10 downto 9) = "00" then
                z_sel <= s_zc(8 downto 5); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                case to_integer(z_sel) is
                    when 0  => z_fa(12 downto 0) <= s_g1;                 z_fb <= to_unsigned(7349, 24);      -- row height exponent
                    when 1  =>                                                                                -- mortar^2
                        z_fa(9 downto 0) <= unsigned(registers_in(2)); z_fb(9 downto 0) <= unsigned(registers_in(2));
                    when 2  => z_fa(16 downto 0) <= c_p;                  z_fb <= resize(s_asp, 24);          -- tile width
                    when 3  => z_fa(16 downto 0) <= c_p;                  z_fb <= resize(z_mf, 24);           -- mortar
                    when 4  => z_fa(12 downto 0) <= s_gp;                 z_fb <= resize(s_gp, 24);           -- shift^2
                    when 7  =>                                                                                -- spread^2
                        z_fa(12 downto 3) <= unsigned(C_KFIX_5); z_fb(12 downto 3) <= unsigned(C_KFIX_5);
                    when 12 => z_fa(8 downto 0)  <= s_vm;                 z_fb <= to_unsigned(900, 24);       -- rate exponent
                    when others => z_fa(15 downto 0) <= z_mt;             z_fb <= to_unsigned(1193046, 24);   -- rate (14)
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(46 downto 0) & '0';
                f_b <= '0' & f_b(23 downto 1);
            end if;

            -- serial shifter for the 2^n scalings (no barrels)
            if z_shn /= 0 then
                z_shn <= z_shn - 1;
                if z_sht = '0' then c_pw <= c_pw(18 downto 0) & '0';
                else c_dph <= c_dph(20 downto 0) & '0'; end if;
            end if;

            -- origins: row 0 is centred on the screen centre (step back to line 0); the tile counter at pixel 0
            if z_nrun = '1' then
                if z_loc(18) = '1' then
                    z_loc <= z_loc + signed(resize(c_p, 19));
                    z_k   <= z_k - 1;
                else
                    z_nrun <= '0';
                end if;
            end if;
            if z_xrun = '1' then
                if z_x(16) = '1' then
                    z_x  <= z_x + signed(resize(c_w, 17));
                    z_xp <= not z_xp;
                else
                    z_xrun <= '0';
                end if;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- wave phase (last field's step): one adder, add or subtract
                    when 20 =>
                        v_b := resize(c_dph, 26);
                        if s_vneg = '1' then v_b := not v_b; end if;
                        v_a27 := (c_ph & '1') + (v_b & s_vneg);
                        c_ph <= v_a27(26 downto 1);
                    -- palette: ground, A, B, accent (chroma kept to 8 bits)
                    when 22 => r_pl_a <= "00" & s_pal & "00";
                    when 25 =>
                        c_g(0) <= unsigned(r_pl_q(31 downto 22));
                        c_g(1) <= "00" & unsigned(r_pl_q(21 downto 14)); c_g(2) <= "00" & unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "01";
                    when 28 =>
                        c_a(0) <= unsigned(r_pl_q(31 downto 22));
                        c_a(1) <= "00" & unsigned(r_pl_q(21 downto 14)); c_a(2) <= "00" & unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "10";
                    when 31 =>
                        c_b(0) <= unsigned(r_pl_q(31 downto 22));
                        c_b(1) <= "00" & unsigned(r_pl_q(21 downto 14)); c_b(2) <= "00" & unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "11";
                    when 34 =>
                        c_c(0) <= unsigned(r_pl_q(31 downto 22));
                        c_c(1) <= "00" & unsigned(r_pl_q(21 downto 14)); c_c(2) <= "00" & unsigned(r_pl_q(11 downto 4));

                    -- row height: e = 0.585 + k1 * 3.585 octaves (Q8), H = 16 * 2^e
                    when 60 => z_e <= to_unsigned(150, 11) + f_acc(26 downto 16);
                    when 61 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 63 => z_mt <= r_ex_q;
                    when 64 => c_pw <= "0000" & z_mt; z_shn <= z_sh; z_sht <= '0';
                    when 80 => c_yst <= s_ys;
                    when 81 => c_ymp <= signed(resize(c_yst, 19)) - signed(resize(c_p, 19));
                    -- mortar fraction (Q10) = 26 + q + q/4 + q/8, q = k3^2 / 4096
                    when 92 => z_q <= f_acc(19 downto 12);
                    when 93 => z_mf <= to_unsigned(26, 9) + resize(z_q, 9) + resize(z_q(7 downto 2), 9) + resize(z_q(7 downto 3), 9);
                    when 124 => c_w <= f_acc(26 downto 12);
                    when 130 =>
                        v_xq := signed(resize(s_xs & "000", 17));
                        c_pxq <= v_xq; c_nxq <= 0 - v_xq;
                        c_wmx <= signed(resize(c_w, 17)) - v_xq;
                        c_xmw <= v_xq - signed(resize(c_w, 17));
                    when 156 => z_m <= f_acc(26 downto 10);
                    when 157 =>
                        if z_m < resize(s_ys & '0', 17) then c_m <= resize(s_ys & '0', 17); else c_m <= z_m; end if;
                    when 188 => c_d <= f_acc(26 downto 17);
                    when 284 => c_s <= f_acc(25 downto 3);
                    -- rate: 2^(m * 6.9 / 503) / 3600 cycles per field
                    when 444 => z_e <= f_acc(18 downto 8);
                    when 445 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 447 => z_mt <= r_ex_q;
                    when 508 =>
                        if s_vm = 0 then c_dph <= (others => '0'); else c_dph <= resize(f_acc(36 downto 21), 22); end if;
                        z_shn <= z_sh; z_sht <= '1';
                    -- origins
                    when 550 =>
                        z_loc <= signed(resize(c_p(16 downto 1), 19)) + signed(resize(c_m(16 downto 1), 19)) - 138240;   -- 540 lines
                        z_k <= (others => '0');
                        z_nrun <= '1';
                        if s_sd = '1' then z_x <= to_signed(-11520, 17);    -- 720 px
                        else z_x <= to_signed(-15360, 17); end if;          -- 960 px
                        z_xp <= '0';
                        z_xrun <= '1';
                    when 620 =>
                        z_first <= '1';
                        if s_rad = '1' and z_x /= 0 then
                            c_base <= c_w - unsigned(z_x(14 downto 0)); c_bpar <= not z_xp;
                        else
                            c_base <= unsigned(z_x(14 downto 0)); c_bpar <= z_xp;
                        end if;
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            if g_start = '1' then z_first <= '0'; end if;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Tile table fill (zone, from zc 40; adders only).  For luma step l = 0 .. 15:
    --   ground tile = G + 3 l (A - G) / 64      A tile = A + l (B - A) / 64
    --   Reveal: both pulled 3/4 of the way to the mid tone (G + A) / 2
    -- and the mortar colour = G + (8 + k3 / 64) (A - G) / 32, or the accent (Reveal).
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_w  : unsigned(9 downto 0);
        variable v_s  : unsigned(12 downto 0);
        variable v_g3 : unsigned(12 downto 0);
        variable v_wd : t_c3;
    begin
        if rising_edge(clk) then
            tt_we <= '0';
            if t_mc /= 0 then
                for i in 0 to 2 loop t_ma(i) <= t_ma(i) + resize(t_da(i), 18); end loop;
                t_mc <= t_mc - 1;
            end if;
            for i in 0 to 2 loop
                if t_st(0) = '1' then v_w := t_w0(i); else v_w := t_w1(i); end if;
                if s_rev = '1' then
                    v_s := resize(v_w, 13) + resize(t_m3(i), 13);
                    v_wd(i) := v_s(11 downto 2);
                else
                    v_wd(i) := v_w;
                end if;
            end loop;
            if s_zone = '1' and s_zc = 40 then
                t_st <= "001";
                for i in 0 to 2 loop
                    t_da(i) <= signed(resize(c_a(i), 11)) - signed(resize(c_g(i), 11));
                    t_d1(i) <= signed(resize(c_b(i), 11)) - signed(resize(c_a(i), 11));
                    t_ga(i) <= resize(c_g(i), 11) + resize(c_a(i), 11);
                end loop;
            else
                case t_st is
                    when "001" =>
                        for i in 0 to 2 loop
                            t_e0(i) <= resize(t_da(i), 13) + shift_left(resize(t_da(i), 13), 1);
                            v_g3 := resize(t_ga(i), 13) + shift_left(resize(t_ga(i), 13), 1);
                            t_m3(i) <= v_g3(12 downto 1);
                            t_a0(i) <= signed(resize(c_g(i) & "100000", 18));
                            t_a1(i) <= signed(resize(c_a(i) & "100000", 18));
                            t_ma(i) <= signed(resize(c_g(i) & "10000", 18));
                        end loop;
                        t_l <= (others => '0');
                        t_mc <= to_unsigned(8, 5) + unsigned(registers_in(2)(9 downto 6));
                        t_st <= "010";
                    when "010" =>
                        for i in 0 to 2 loop
                            t_w0(i) <= unsigned(t_a0(i)(15 downto 6)); t_w1(i) <= unsigned(t_a1(i)(15 downto 6));
                            t_a0(i) <= t_a0(i) + resize(t_e0(i), 18);
                            t_a1(i) <= t_a1(i) + resize(t_d1(i), 18);
                        end loop;
                        t_st <= "011";
                    when "011" | "100" =>
                        tt_we <= '1'; tt_wa <= "000" & (not t_st(0)) & t_l;
                        tt_wd <= "000000" & std_logic_vector(v_wd(0)) & std_logic_vector(v_wd(1)(7 downto 0)) & std_logic_vector(v_wd(2)(7 downto 0));
                        if t_st(0) = '1' then
                            t_st <= "100";
                        else
                            t_l <= t_l + 1;
                            if t_l = 15 then t_st <= "000"; else t_st <= "010"; end if;
                        end if;
                    when others => null;
                end case;
            end if;
            if s_zone = '1' and s_zc = 200 then
                for i in 0 to 2 loop
                    if s_rev = '1' then c_mo(i) <= c_c(i); else c_mo(i) <= unsigned(t_ma(i)(14 downto 5)); end if;
                end loop;
            end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- [B] Generated avid (parity-locked start 3/4 clocks after the source's
    -- avid rise, field-latched width)
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
    -- [C] Source field.  Osc: a two-axis triangle lattice (640 x 540 virtual
    -- px) drifting diagonally.  The luma x 1.5 (or the lattice) is filtered
    -- along x (1/16 one-pole, restarted every line, held through blanking).
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_tx, v_ty : unsigned(6 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_s9 : unsigned(8 downto 0);
        variable v_x  : unsigned(13 downto 0);
        variable v_d1 : signed(14 downto 0);
        variable v_yo : signed(18 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;  -- ot * 96
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_tx := f_tri7(s_ox(15 downto 8)); v_ty := f_tri7(s_oy(15 downto 8));
            v_s := resize(v_tx, 8) + resize(v_ty, 8);

            -- input sample (two registers), held through blanking
            s_av0 <= data_in.avid; s_sav <= s_av0; s_savd <= s_sav;
            if data_in.avid = '1' then
                s_ov <= v_s - shift_right(v_s, 3);
                if unsigned(data_in.y) > 64 then s_dd <= unsigned(data_in.y) - 64; else s_dd <= (others => '0'); end if;
            end if;
            if s_av0 = '1' then
                v_s9 := resize(s_dd(9 downto 2), 9) + resize(s_dd(9 downto 3), 9);
                if s_src = '1' then s_sv <= s_ov;
                elsif v_s9(8) = '1' then s_sv <= (others => '1');
                else s_sv <= v_s9(7 downto 0); end if;
            end if;

            v_x := s_sv & "000000";
            if s_sav = '1' and s_savd = '0' then
                s_y1 <= v_x;
            elsif s_sav = '1' then
                v_d1 := signed(resize(v_x, 15)) - signed(resize(s_y1, 15));
                s_y1 <= unsigned(signed(s_y1) + resize(shift_right(v_d1, 4), 14));
            end if;
            -- K6 Camera: the filtered sample (Osc twice the camera's, clipped), x k in its own stage, less the
            -- stretch offset, clamped to 0 .. 15 steps (k = 256: exactly the v0.1.0 step, y1 bits 13 .. 10).
            -- Three clocks behind s_y1, so the sample sits three pixels further along (inside the 16-pixel
            -- filter).
            if s_src = '0' then s_ymv <= s_y1(13 downto 6);
            elsif s_y1(13) = '1' then s_ymv <= (others => '1');
            else s_ymv <= s_y1(12 downto 5); end if;
            s_ym <= s_ymv * c_kg;
            v_yo := signed(resize(s_ym, 19)) - signed(resize(c_ko, 19));
            if v_yo < 0 then s_yl <= (others => '0');
            elsif v_yo(18 downto 16) /= "000" then s_yl <= (others => '1');
            else s_yl <= unsigned(v_yo(15 downto 12)); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [D] Wave engine.  The row counter steps once per line (ls_* hold the
    -- NEXT line's values: position in the row, row number, Spread * number).
    -- After every line start the per-line engine works out the next line's
    -- tile-counter start, tile parity and mortar coverage on one small serial
    -- multiplier (45 clocks).
    ---------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_b   : signed(27 downto 0);
        variable v_a31 : signed(28 downto 0);
        variable v_sub, v_en : std_logic;
    begin
        if rising_edge(clk) then
            -- wrap candidates are pre-registered (ls_loc is stable between line starts)
            ls_n1 <= resize(ls_loc + c_yst, 17);
            ls_n2 <= signed(resize(ls_loc, 19)) + c_ymp;
            -- Spread * row number: - S per step back in the zone's origin search, + S at each row wrap (one adder)
            v_sub := z_nrun;
            v_en := (z_nrun and z_loc(18)) or (g_start and not ls_n2(18));
            v_b := signed(resize(c_s(22 downto 2), 28));
            if v_sub = '1' then v_b := not v_b; end if;
            v_a31 := (ls_sk & '1') + (v_b & v_sub);
            if s_zone = '1' and s_zc = 550 then
                ls_sk <= (others => '0');
            elsif v_en = '1' then
                ls_sk <= v_a31(28 downto 1);
            end if;
            if s_zone = '1' and s_zc = 620 then
                ls_loc <= unsigned(z_loc(16 downto 0)); ls_k <= z_k; ls_new <= '0';
            elsif g_start = '1' then
                if ls_n2(18) = '0' then
                    ls_loc <= unsigned(ls_n2(16 downto 0)); ls_k <= ls_k + 1; ls_new <= '1';
                else
                    ls_loc <= ls_n1; ls_new <= '0';
                end if;
            end if;
        end if;
    end process p_acc;

    p_line : process(clk)
        variable v_a   : signed(19 downto 0);
        variable v_pos : unsigned(11 downto 0);
        variable v_m   : unsigned(10 downto 0);
        variable v_c   : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            le_kick <= g_start;
            -- serial multiplier (free-running; loaded by the engine)
            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(23 downto 0) & '0';
            m_b <= '0' & m_b(9 downto 1);

            if le_kick = '1' or (s_zone = '1' and s_zc = 624) then
                le_t <= (others => '0'); le_run <= '1';
            elsif le_run = '1' then
                le_t <= le_t + 1;
                case to_integer(le_t) is
                    when 0 =>            -- the row's index term; mortar left below this line's top
                        v_a := abs(ls_sk(27 downto 8));
                        if s_rad = '1' then le_x <= unsigned(v_a(13 downto 0)); else le_x <= unsigned(ls_sk(21 downto 8)); end if;
                        le_d1 <= signed(resize(c_m, 18)) - signed(resize(ls_loc, 18));
                    when 1 =>            -- theta (14 bits per cycle); the part of this line that is mortar (Q8 lines)
                        le_th <= le_x - c_ph(25 downto 12);
                        if le_d1 > 0 then
                            if le_d1 > signed(resize(s_ys, 18)) then le_part <= s_ys(10 downto 0);
                            else le_part <= unsigned(le_d1(10 downto 0)); end if;
                        elsif ls_n2 > 0 then
                            le_part <= unsigned(ls_n2(10 downto 0));
                        else
                            le_part <= (others => '0');
                        end if;
                    when 2 =>            -- wave ROM address: quarter-wave fold; Snap = the first sixteenth stretched
                        v_pos := le_th(11 downto 0);
                        if le_th(12) = '1' then v_pos := not v_pos; end if;
                        if s_snap = '0' then r_w_a <= v_pos(11 downto 2);
                        elsif v_pos(11 downto 10) /= "00" then r_w_a <= (others => '1');
                        else r_w_a <= v_pos(9 downto 0); end if;
                        le_neg <= le_th(13) xor (s_alt and ls_k(0));
                    when 4 => le_w <= r_w_q(9 downto 0);
                    when 5 =>            -- SHIFT * wave
                        m_a <= resize(le_w, 25); m_b <= c_d; m_acc <= (others => '0');
                    when 17 =>           -- the shift mod 2 tiles (Q10): whole tile = parity, fraction -> px
                        v_m := m_acc(18 downto 8);
                        if le_neg = '1' then v_m := 0 - v_m; end if;
                        le_frac <= v_m(9 downto 0);
                        le_par <= v_m(10) xor ls_k(0) xor c_bpar;
                    when 18 =>
                        m_a <= resize(c_w, 25); m_b <= le_frac; m_acc <= (others => '0');
                    when 30 => le_c <= resize(c_base, 16) + resize(m_acc(24 downto 10), 16);
                    when 31 => le_cmw <= signed(resize(le_c, 17)) - signed(resize(c_w, 17));
                    when 32 =>
                        if le_cmw(16) = '0' then le_c0 <= unsigned(le_cmw(14 downto 0)); le_par0 <= not le_par;
                        else le_c0 <= le_c(14 downto 0); le_par0 <= le_par; end if;
                        m_a <= resize(le_part, 25); m_b <= resize(s_rec, 10); m_acc <= (others => '0');
                    when 44 =>           -- mortar coverage in eighths
                        v_c := m_acc(17 downto 0) + 1024;
                        le_cov <= v_c(14 downto 11);
                        le_run <= '0';
                    when others => null;
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [E] The look.  Stage 1: the tile counter (Q4 virtual px at the pixel's
    -- left edge; Radial counts down to the screen centre, then up).  The pixel
    -- holding a tile edge gets the next tile's share of it in sixteenths.
    -- 3 .. 5: the tile's held luma step; 6 .. 8: its colour from the table;
    -- 9 .. 12: edge blend with the next pixel's colour; 13 .. 15: mortar blend.
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_s1, v_s2, v_d, v_alt : signed(16 downto 0);
        variable v_cross : std_logic;
        variable v_fn  : unsigned(5 downto 0);
        variable v_f9  : unsigned(8 downto 0);
        variable v_f5  : unsigned(4 downto 0);
        variable v_bk  : unsigned(1 downto 0);
        variable v_s   : signed(15 downto 0);
        variable v_o   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 1
            if a_dec = '1' then v_s1 := c_nxq; v_s2 := c_wmx; else v_s1 := c_pxq; v_s2 := c_xmw; end if;
            v_d   := signed(resize(a_c, 17)) + v_s1;
            v_alt := signed(resize(a_c, 17)) + v_s2;
            if a_dec = '1' then
                v_cross := v_d(16);
                v_fn := unsigned(0 - v_d(5 downto 0));
            else
                v_cross := not v_alt(16);
                v_fn := unsigned(v_alt(5 downto 0));
            end if;
            if g_start = '1' then
                a_c <= le_c0; a_par <= le_par0; a_col <= (others => '0'); a_cov <= le_cov;
                a_samp <= z_first or ls_new;
                if z_first = '1' then a_top <= '1'; elsif ls_new = '1' then a_top <= '0'; end if;
                a_dec <= s_rad;
            else
                if v_cross = '1' then
                    a_c <= unsigned(v_alt(14 downto 0)); a_par <= not a_par; a_col <= a_col + 1;
                else
                    a_c <= unsigned(v_d(14 downto 0));
                end if;
                if g_cnt = c_hm1 then a_dec <= '0'; end if;
            end if;
            -- 2
            q2_col <= a_col; q2_par <= a_par; q2_cross <= v_cross; q2_fn <= v_fn;
            -- 3: the edge's share in sixteenths; hold RAM address; on the row's first line, the hold write at
            -- each tile's last pixel (the top row of the field uses its own bank, alternating by field)
            case to_integer(s_xs) is
                when 2      => v_f5 := q2_fn(4 downto 0);
                when 3      =>
                    v_f9 := resize(q2_fn & "000", 9) + resize(q2_fn & '0', 9) + resize(q2_fn, 9) + 8;
                    v_f5 := v_f9(8 downto 4);
                when others => v_f5 := q2_fn(5 downto 1);
            end case;
            if q2_cross = '0' then q3_f <= (others => '0');
            elsif v_f5(4) = '1' then q3_f <= (others => '1');
            else q3_f <= v_f5(3 downto 0); end if;
            q3_par <= q2_par;
            if a_top = '1' then v_bk := '1' & (not s_fpar); else v_bk := "00"; end if;
            lh_ra <= v_bk & q2_col;
            if a_top = '1' then v_bk := '1' & s_fpar; else v_bk := "00"; end if;
            lh_wa <= v_bk & q2_col;
            lh_wd <= "0000" & std_logic_vector(s_yl);
            lh_we <= a_samp and s_gav_sr(0) and (q2_cross or not g_avid);
            -- 4: RAM; 5: fabric copy
            q4_par <= q3_par; q4_f <= q3_f;
            q5_l <= unsigned(lh_q(3 downto 0)); q5_par <= q4_par; q5_f <= q4_f;
            -- 6: table address; 7: RAM; 8: fabric copy
            tt_ra <= "000" & q5_par & q5_l;
            q6_f <= q5_f; q7_f <= q6_f; q8_f <= q7_f;
            q8_c(0) <= unsigned(tt_q(25 downto 16));
            q8_c(1) <= "00" & unsigned(tt_q(15 downto 8));
            q8_c(2) <= "00" & unsigned(tt_q(7 downto 0));
            -- 9: this pixel beside the next one
            q9_c <= q8_c; q9_f <= q8_f;
            -- 10 .. 12: tile colour = this + (next - this) * share / 16
            q10_c <= q9_c; q10_f <= q9_f;
            q11_c <= q10_c;
            q13_c <= q12_c; q14_c <= q13_c;
            for i in 0 to 2 loop
                q10_d(i) <= signed(resize(q8_c(i), 11)) - signed(resize(q9_c(i), 11));
                q11_p01(i) <= f_gs(q10_d(i), q10_f(0), 13) + shift_left(f_gs(q10_d(i), q10_f(1), 13), 1);
                q11_p23(i) <= f_gs(q10_d(i), q10_f(2), 13) + shift_left(f_gs(q10_d(i), q10_f(3), 13), 1);
                v_s := resize(q11_p01(i), 16) + shift_left(resize(q11_p23(i), 16), 2) + 8;
                v_o := signed(resize(q11_c(i), 12)) + resize(shift_right(v_s, 4), 12);
                q12_c(i) <= unsigned(v_o(9 downto 0));
                -- 13 .. 15: mortar over it by line coverage / 8; the mortar colour outside the generated avid
                q13_d(i) <= signed(resize(c_mo(i), 11)) - signed(resize(q12_c(i), 11));
                q14_p01(i) <= f_gs(q13_d(i), a_cov(0), 13) + shift_left(f_gs(q13_d(i), a_cov(1), 13), 1);
                q14_p23(i) <= f_gs(q13_d(i), a_cov(2), 13) + shift_left(f_gs(q13_d(i), a_cov(3), 13), 1);
                v_s := resize(q14_p01(i), 16) + shift_left(resize(q14_p23(i), 16), 2) + 4;
                v_o := signed(resize(q14_c(i), 12)) + resize(shift_right(v_s, 3), 12);
                if s_gav_sr(12) = '0' then q15_c(i) <= c_mo(i);
                else q15_c(i) <= unsigned(v_o(9 downto 0)); end if;
            end loop;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [F] Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent
    -- 8-sums of the 8-bit chroma, resynced every line: 10 bits out), luma [1 2 1]/4 delayed to the chroma's
    -- group delay, luma 13/16, blanking gate.  The filters add 4 pixels of
    -- delay: the output register is effective stage 23 = C_LAT + 1.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 16: running 8-sums (the sample of 8 clocks before comes back from its delay ring); they snap
            -- back to 8 x the mortar colour once 8 blank samples are in, so a glitched sample can't offset
            -- the colours for good
            if s_bah = "00000000" then
                s_bu <= c_mo(1)(7 downto 0) & "000"; s_bv <= c_mo(2)(7 downto 0) & "000";
            else
                s_bu <= s_bu + q15_c(1)(7 downto 0) - unsigned(cr_f(15 downto 8));
                s_bv <= s_bv + q15_c(2)(7 downto 0) - unsigned(cr_f(7 downto 0));
            end if;
            s_bud <= s_bu; s_bvd <= s_bv;
            s_ysr(0) <= q15_c(0);
            for i in 1 to 5 loop s_ysr(i) <= s_ysr(i - 1); end loop;
            -- 17: filtered
            v_sc := resize(s_bu, 12) + s_bud; s_u31 <= v_sc(11 downto 2);
            v_sc := resize(s_bv, 12) + s_bvd; s_v31 <= v_sc(11 downto 2);
            v_y := resize(s_ysr(3), 12) + resize(s_ysr(4) & '0', 12) + resize(s_ysr(5), 12);
            s_y31 <= v_y(11 downto 2);
            -- 18: luma 13/16 toward black
            if s_y31 > 64 then v_dl := s_y31 - 64; else v_dl := (others => '0'); end if;
            s_y32 <= s_y31 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u32 <= s_u31; s_v32 <= s_v31;
            -- 19: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y33 <= s_y32; s_u33 <= s_u32; s_v33 <= s_v32;
            else
                s_y33 <= to_unsigned(64, 10); s_u33 <= to_unsigned(512, 10); s_v33 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y33);
    data_out.u       <= std_logic_vector(s_u33);
    data_out.v       <= std_logic_vector(s_v33);
    data_out.hsync_n <= sr_f(2);
    data_out.vsync_n <= sr_f(1);
    data_out.field_n <= sr_f(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture cafewall;
