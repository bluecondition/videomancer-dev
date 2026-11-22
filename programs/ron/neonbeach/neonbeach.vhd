-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- NEON BEACH v0.5.1 -- Pixel Beach's side-on shorebreak drawn by Neonscape's wire engine.
--
-- A cross-section of a beach: sky with a sun or moon, clouds, gulls and stars; under the horizon a strip of
-- distant sea that mirrors them; in front of it the water cut through like a glass tank, its layers following
-- the surface; under everything the sand, a mesh of quads.  Every pixel gets a REGION ID (object + facet), a
-- wire is drawn wherever the ID differs from a neighbour (left, or the line above: one line store), and a
-- region is filled from its own colour ramp when the fill is on (Neonscape's engine).
--
--   sky half (rows above the horizon, d = rows above it)
--     sky            the colour follows the sun: two per-field keys (zenith, horizon) lerped from eight
--                    keyframes by the sun's elevation (computed by running the colour path in vblank)
--     sun / moon     half-chord table, radius 128, raised and set by the slider; slits roll down the sun,
--                    a second disc bites the moon into a crescent
--     clouds, gulls  span-coded sprites in drifting lattice cells (slot A); gulls roost when it is dark
--     stars          hashed cells, twinkling; more of them as the sun goes down, all of them at night
--   sea strip       every line under the horizon redraws the sky objects at the same distance (a mirror),
--                    dimmed and broken by drifting ripple gaps
--   water           between the surface and the sea bed, both read per pixel from a knot table (block RAM,
--                    16 virtual px a knot, lerped, double-buffered by field): eight layers at k / 8 of the local
--                    depth, each its own colour (Pixel Beach's bands: deep, mid, aqua, surf, a neon top), columns
--                    every 64 px that sway with the passing wave; foam = 16 px squares under the surface that turn
--                    white one by one; spray over it.  The water's colours follow the sun's elevation gradually
--                    (per-field palette entries, written in vblank from keyframes)
--   waves           a small microcoded engine (nbengine.py) fills the knot table once a field: each wave's phase
--                    (Rhythm: how often they come; Speed: how fast they run in, by stretching them), its size (waves come in sets), shoaling, the depth cap (breaking: the excess is
--                    foam), the profile by class, two chop waves (Chop), the surge, foam (Foam), the swash sheet
--                    running up the beach; it also runs the crab
--   sand            under the bed line: cells 32 px wide and 32 rows deep, the rows parallel to the sand's
--                    surface.  Wet sand (a 4-bit counter per cell, swept in vblank): the wash covers cells a
--                    triangle at a time; as it drains the cells keep their diagonals and lose them as they dry,
--                    and their colour fades back smoothly (a per-field fraction between the counter's steps)
--   crab            span-coded, folded about its centre line (slot B); walks, pauses, runs from the wash, burrows
--                    when switched off and comes up somewhere new
--
--   K1 Rhythm  K2 Tide  K3 Foam  K4 Chop  K5 Swell  K6 Speed
--   S7 Fill (Wires / Fill)  S8 Colour (Neon / Ice)  S9 Crab (Off / On)  S10 Sky (Day / Night)
--   (no input video: a synthesis program since v0.5.0)
--   P12 Sun    elevation of the sun or moon; the sky, the palette and the stars follow it
--
-- Stage numbering: a register "at stage S" holds pixel n at T + S + n (T = the g_start clock).  One latency
-- (33) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).  Chassis ([A] timing, virtual
-- 1080-line coordinates, generated avid, output conditioning) from neonscape / riley / outrun.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.neonbeach_pkg.all;
use work.all;

architecture neonbeach of program_top is
    constant C_LAT : integer := 33;   -- = processing_delay_clks

    constant C_HZ    : integer := 356;      -- the horizon (virtual row)
    constant C_SL0   : integer := 626;      -- still sea level at Tide 50 %
    constant C_SUNX  : integer := 240;      -- sun / moon centre, virtual px right of the screen centre
    constant C_DEF_P12 : integer := 614;

    constant C_CM1 : std_logic_vector(15 downto 0) := x"5A93";   -- clouds
    constant C_BM  : std_logic_vector(15 downto 0) := x"1242";   -- gulls
    -- spray: a hand-picked aperiodic mask (a coordinate-bit hash tiles a visible motif)
    constant C_DOT   : std_logic_vector(0 to 63) := "1011000000101000000000001000000110000000000100000000000110000000";
    -- foam squares: each square's threshold jitter (0 .. 15 rows), by knot (4 bits, reshuffled in time) & square row
    type t_fj is array (0 to 63) of unsigned(3 downto 0);
    constant C_FJ : t_fj := (
        x"A", x"4", x"C", x"F", x"2", x"7", x"3", x"8", x"B", x"D", x"0", x"5", x"1", x"0", x"8", x"6",
        x"B", x"6", x"5", x"A", x"8", x"D", x"2", x"1", x"E", x"C", x"1", x"D", x"C", x"8", x"1", x"0",
        x"6", x"B", x"E", x"7", x"E", x"7", x"4", x"3", x"E", x"3", x"F", x"F", x"9", x"A", x"B", x"5",
        x"2", x"D", x"0", x"A", x"5", x"7", x"6", x"2", x"4", x"4", x"3", x"F", x"9", x"9", x"C", x"9");
    -- sheen: which pulses show, by (pulse number, object): about three in five.  Never on the sand (object 8, the
    -- first bit of each byte): v0.3's bands running down the sand read as a constant pulsing.
    constant C_SHG : std_logic_vector(0 to 63) := x"3655_631A_6F4B_526D";

    ---------------------------------------------------------------------------
    -- [A] timing-ID steps
    ---------------------------------------------------------------------------
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
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    function f_max4(a, b : unsigned(3 downto 0)) return unsigned is
    begin
        if a > b then return a; else return b; end if;
    end function;
    -- signed 8-bit delta times one bit of t, shifted: a gated operand for the colour lerp
    function f_gate(d : signed(7 downto 0); b : std_logic; sh : natural) return signed is
        variable v : signed(13 downto 0);
    begin
        if b = '1' then v := shift_left(resize(d, 14), sh); else v := (others => '0'); end if;
        return v;
    end function;
    -- a signed 11-bit knot difference times one bit of the fraction, shifted (the surface / bed lerp)
    function f_gk(d : signed(10 downto 0); b : std_logic; sh : natural) return signed is
        variable v : signed(14 downto 0);
    begin
        if b = '1' then v := shift_left(resize(d, 15), sh); else v := (others => '0'); end if;
        return v;
    end function;

    -- gated operands for the filler's small products (size x shoal; the foam gains)
    function f_gw(d : unsigned(4 downto 0); b : std_logic; sh : natural) return unsigned is
        variable v : unsigned(8 downto 0);
    begin
        if b = '1' then v := shift_left(resize(d, 9), sh); else v := (others => '0'); end if;
        return v;
    end function;
    function f_gf(d : unsigned(9 downto 0); b : std_logic; sh : natural) return unsigned is
        variable v : unsigned(11 downto 0);
    begin
        if b = '1' then v := shift_left(resize(d, 12), sh); else v := (others => '0'); end if;
        return v;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(12 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_th2, s_th3, s_tv2 : std_logic := '0';     -- wire thickness by mode
    signal w_x0   : signed(13 downto 0);                -- the left edge, half units from the centre

    -- [A] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k): pixels 0 .. W - 1 at stage k + 2
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(1 to 16) := (others => '0');   -- s_st(k) = g_start delayed k

    -- [B] controls
    signal s_fsw, s_theme, s_croff, s_night : std_logic := '0';
    signal s_k2 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_gs : unsigned(12 downto 0) := to_unsigned(C_DEF_P12 * 8, 13);    -- sun glide (Q3)
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);          -- Speed glide
    signal s_gp : unsigned(12 downto 0) := (others => '0');                   -- fill glide
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(1 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- [C] per-field values
    signal c_t    : unsigned(15 downto 0) := (others => '0');     -- field counter
    signal c_e    : unsigned(9 downto 0) := to_unsigned(C_DEF_P12, 10);   -- sun / moon elevation
    signal c_sunc : signed(11 downto 0) := to_signed(70, 12);     -- its centre, rows above the horizon
    signal c_sl   : unsigned(10 downto 0) := to_unsigned(C_SL0, 11);      -- sea level row
    signal c_fp   : unsigned(13 downto 0) := (others => '0');     -- fill position (Q3)
    signal c_sa1, c_sab : unsigned(13 downto 0) := (others => '0');   -- cloud and gull drift
    signal c_tod  : unsigned(1 downto 0) := "00";                 -- 0 day, 1 low sun, 2 night
    signal c_pal  : unsigned(2 downto 0) := "000";                -- theme & time of day
    signal c_sden : unsigned(1 downto 0) := "00";                 -- star density
    signal c_hi, c_mid, c_lo : std_logic := '0';
    signal c_ebird, c_esun, c_emoon, c_ecrab : std_logic := '0';
    -- the sky keys: the colour path is run on two made-up pixels in vblank (zenith, horizon keyframes lerped
    -- by the elevation) and the results are kept as the sky ramp's c0 and delta
    -- (and the same for the water: its five keys and its wire, so it changes colour gradually with the sun).  Each
    -- result is either kept (the previous key) or written into the live palette with the previous key as a ramp
    -- segment: entry = prev & (this - prev) / 2.
    signal c_inj  : std_logic := '0';
    signal c_injk : std_logic_vector(1 downto 0) := "00";        -- 01 keep, 10 write and keep, 11 write
    signal c_injent : unsigned(4 downto 0) := (others => '0');
    signal c_injt : unsigned(5 downto 0) := (others => '0');
    signal c_injw : unsigned(4 downto 0) := (others => '0');      -- the entry it writes
    signal c_wiv  : unsigned(1 downto 0) := "11";                 -- the water's keyframe interval
    signal c_wt   : unsigned(5 downto 0) := (others => '1');      -- and the position in it
    signal c_kp   : std_logic_vector(23 downto 0) := (others => '0');

    -- [C] line ROM / colour ROM / line store
    signal lr_a  : unsigned(9 downto 0) := (others => '0');
    signal lr_q, lr_qf : unsigned(15 downto 0) := (others => '0');
    signal pal_a : unsigned(7 downto 0) := (others => '0');
    signal pal_q, pal_qf : unsigned(47 downto 0) := (others => '0');
    signal pal_ram : t_c_nb_pal := C_NB_PAL;
    attribute no_rw_check of pal_ram : signal is true;
    signal pal_we : std_logic := '0';
    signal pal_wa : unsigned(7 downto 0) := (others => '0');
    signal pal_wd : unsigned(47 downto 0) := (others => '0');
    type t_lb is array (0 to 2047) of std_logic_vector(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q, lb_qf : std_logic_vector(15 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal lb_wa : unsigned(10 downto 0) := (others => '0');
    signal lb_wd : std_logic_vector(15 downto 0) := (others => '0');

    -- [C] knot table: per knot (16 virtual px), the surface row, the step to the next knot's, the bed row,
    -- its step, the foam.  The filler writes it in vblank; the pixel path reads it in active video.
    type t_kr is array (0 to 255) of std_logic_vector(47 downto 0);
    signal kr_ram : t_kr := (others => (others => '0'));
    attribute no_rw_check of kr_ram : signal is true;
    signal kr_a  : unsigned(7 downto 0) := (others => '0');
    signal kr_q, kr_qf : std_logic_vector(47 downto 0) := (others => '0');
    signal kr_we : std_logic := '0';
    signal kr_wa : unsigned(7 downto 0) := (others => '0');
    signal kr_wd : std_logic_vector(47 downto 0) := (others => '0');

    -- [C] the engine (nbengine.py): once a field, from the field start on, the waves into the knot table bank
    -- that is not on screen.  Registers r0 .. r15 live in the scratch RAM's top (496 .. 511).
    signal u_run : std_logic := '0';
    signal u_wd  : unsigned(1 downto 0) := (others => '0');          -- watchdog: field starts since it began
    signal u_pc, u_npc : unsigned(8 downto 0) := (others => '0');
    signal u_ph  : unsigned(3 downto 0) := (others => '0');
    signal u_iq, u_ir : std_logic_vector(23 downto 0) := (others => '0');
    signal u_a, u_b, u_res : unsigned(15 downto 0) := (others => '0');
    signal u_sum : unsigned(16 downto 0) := (others => '0');
    signal d_sub, d_xz, d_yi, d_ya : std_logic := '0';                -- pre-decoded controls
    signal d_mw, d_mul, d_frd, d_fwr, d_kwr, d_end, d_wr, p6, p9 : std_logic := '0';
    signal d_rs : std_logic_vector(7 downto 0) := (others => '0');
    signal d_fs : std_logic_vector(6 downto 0) := (others => '0');
    signal d_kf, d_of : std_logic_vector(7 downto 0) := (others => '0');
    signal d_jc : std_logic_vector(2 downto 0) := "000";
    signal d_mrd : unsigned(1 downto 0) := "00";
    signal d_dst : unsigned(3 downto 0) := (others => '0');
    signal d_tgt : unsigned(8 downto 0) := (others => '0');
    signal d_imm, d_in, d_tab, sc_qf : unsigned(15 downto 0) := (others => '0');
    signal d_i11 : unsigned(10 downto 0) := (others => '0');
    signal d_sh  : natural range 0 to 3 := 0;
    signal sh_qf : unsigned(7 downto 0) := (others => '0');
    signal sc_ram : t_c_nb_scinit := C_NB_SCINIT;
    attribute no_rw_check of sc_ram : signal is true;
    signal sc_ra, sc_wa : unsigned(8 downto 0) := (others => '0');
    signal sc_q, sc_wd : unsigned(15 downto 0) := (others => '0');
    signal sc_we : std_logic := '0';
    signal sh_q  : unsigned(7 downto 0) := (others => '0');
    signal m_x, m_p : unsigned(23 downto 0) := (others => '0');
    signal m_y   : unsigned(7 downto 0) := (others => '0');
    signal m_c   : unsigned(3 downto 0) := (others => '0');
    signal k_srf, k_bed : unsigned(9 downto 0) := (others => '0');   -- the staged knot word
    signal k_ds  : unsigned(10 downto 0) := (others => '0');
    signal k_db, k_fm : unsigned(7 downto 0) := (others => '0');
    signal k_w   : std_logic := '0';
    signal o_e, o_ks : unsigned(7 downto 0) := (others => '0');      -- swash envelope, shoreline knot
    signal o_cx, o_cy : unsigned(9 downto 0) := to_unsigned(540, 10);  -- the crab: centre x, top row
    signal o_cs : unsigned(7 downto 0) := (others => '0');           -- its sink (0 .. 96) | frame << 7
    signal o_ch : unsigned(10 downto 0) := (others => '0');          -- its hole's wetness cell
    signal c_cry, c_cyh : unsigned(10 downto 0) := to_unsigned(700, 11);   -- per field: its drawn top, its hole line
    signal c_cx2 : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal c_dbank : std_logic := '0';                              -- the knot table bank on screen
    signal u_lfsr : unsigned(15 downto 0) := x"ACE1";
    signal s_k1, s_k3, s_k4, s_k5 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- [C] wet sand: Pixel Beach's per-cell counter.  2048 cells (32 rows of depth x 64 columns of 32 px), 4 bits:
    -- 15 = under the wash, 14 = only the upper triangle, 13 = damp and held (the sea bed's top row, the crab's
    -- hole), 12 .. 1 drying (one step every 12 fields; drawn with the fields since the step as a fraction, so
    -- the colour fades back smoothly: v0.4 stepped every drying cell at once, 5 times a second, a pulse along
    -- the whole wet edge), 0 dry.  A sweep rewrites every cell in vblank; the pixel path reads it in active video.
    type t_wt is array (0 to 2047) of std_logic_vector(3 downto 0);
    signal wt_ram : t_wt := (others => (others => '0'));
    attribute no_rw_check of wt_ram : signal is true;
    signal wt_a, wt_wa, wt_pa : unsigned(10 downto 0) := (others => '0');
    signal wt_q, wt_qf, wt_wd : std_logic_vector(3 downto 0) := (others => '0');
    signal wt_we : std_logic := '0';
    signal ws_run, ws_rows, ws_tick, ws_uw : std_logic := '0';
    signal ws_col, ws_c1, ws_c2 : unsigned(5 downto 0) := (others => '0');
    signal ws_r0 : unsigned(5 downto 0) := (others => '0');
    signal ws_r1, ws_r2 : unsigned(4 downto 0) := (others => '0');
    signal ws_v1, ws_v2 : std_logic := '0';
    signal ws_p  : unsigned(2 downto 0) := (others => '0');
    signal ws_dp : unsigned(4 downto 0) := (others => '0');
    signal ws_fc, ws_dist : signed(8 downto 0) := (others => '0');
    signal ws_reach, ws_rc2 : signed(9 downto 0) := (others => '0');
    signal ws_div : unsigned(3 downto 0) := (others => '0');
    signal ws_fr  : unsigned(2 downto 0) := (others => '0');       -- 8 x (fields since the step) / 12

    -- [C] line engine
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_lc   : unsigned(6 downto 0) := (others => '1');
    signal e_ln   : unsigned(1 downto 0) := (others => '0');
    signal e_s    : signed(11 downto 0) := (others => '0');
    signal e_d    : unsigned(9 downto 0) := (others => '0');
    signal e_ab, e_mir, e_gq, e_kick : std_logic := '0';
    signal e_sa, e_sb : unsigned(7 downto 0) := (others => '0');
    signal e_tsun : unsigned(6 downto 0) := (others => '0');
    signal e_tb   : unsigned(5 downto 0) := (others => '0');
    signal e_asel : std_logic := '0';
    signal e_dyu  : signed(11 downto 0) := (others => '0');
    signal e_c1, e_bd : signed(10 downto 0) := (others => '0');
    signal e_cr   : signed(11 downto 0) := (others => '0');
    signal e_cv   : std_logic := '0';
    signal e_rm   : unsigned(8 downto 0) := (others => '0');
    signal e_aa, e_ba : unsigned(9 downto 0) := (others => '0');
    signal e_aen, e_ben, e_son, e_bon : std_logic := '0';
    signal e_asc  : unsigned(13 downto 0) := (others => '0');
    signal e_x0   : signed(13 downto 0) := (others => '0');
    signal e_fc   : unsigned(13 downto 0) := (others => '0');

    -- this line's constants (written by the line engine in blanking) and the next line's start values
    signal l_d   : unsigned(9 downto 0) := (others => '0');
    signal l_row : unsigned(10 downto 0) := (others => '0');
    signal l_slr : unsigned(7 downto 0) := (others => '0');                   -- (sea level - row), low bits
    signal l_mir, l_ab : std_logic := '0';
    signal l_sw, l_bw : unsigned(8 downto 0) := (others => '0');
    signal l_son, l_bon : std_logic := '0';
    signal l_tsun : unsigned(6 downto 0) := (others => '0');
    signal l_tsky : unsigned(5 downto 0) := (others => '0');
    type t_row is array (0 to 3) of unsigned(15 downto 0);
    signal l_arow, l_brow : t_row := (others => (others => '0'));
    signal l_asel : std_logic := '0';
    signal l_ta, l_tb : unsigned(6 downto 0) := (others => '0');
    signal l_fill : std_logic_vector(7 downto 0) := (others => '0');
    signal l_rip, l_v1, l_v2 : std_logic := '0';
    signal l_od, l_pc : unsigned(13 downto 0) := (others => '0');             -- sheen: per-line phase terms
    signal n_xa0, n_xw0, n_pa0 : unsigned(13 downto 0) := (others => '0');
    signal n_xb0 : signed(13 downto 0) := (others => '0');
    signal n_x0  : signed(12 downto 0) := (others => '0');

    -- [C] beam counters
    signal b_xw, b_xa, b_pa : unsigned(13 downto 0) := (others => '0');
    signal b_xb : signed(13 downto 0) := (others => '0');
    signal b_x  : signed(12 downto 0) := (others => '0');
    signal b_px : unsigned(10 downto 0) := (others => '0');
    signal s18_np : unsigned(8 downto 0) := (others => '0');
    signal s18_ix, s18_wo : unsigned(2 downto 0) := (others => '0');
    signal s18_ng : std_logic := '0';
    signal s19_n6 : unsigned(5 downto 0) := (others => '0');

    -- water and sand (stages 2 .. 14)
    type t_f4 is array (2 to 5) of unsigned(3 downto 0);
    signal w_f   : t_f4 := (others => (others => '0'));
    type t_x7 is array (2 to 10) of unsigned(10 downto 0);
    signal w_xb  : t_x7 := (others => (others => '0'));     -- column parities (64 px, 32 px) & the sand x (5)
    signal w5_ps1, w5_ps2 : signed(14 downto 0) := (others => '0');
    signal w5_s0, w5_b0 : unsigned(9 downto 0) := (others => '0');
    signal w5_db : signed(7 downto 0) := (others => '0');
    signal w6_ps, w6_pb1, w6_pb2, w7_pb : signed(14 downto 0) := (others => '0');
    signal w6_s0, w6_b0, w7_b0 : signed(11 downto 0) := (others => '0');
    signal w7_srf, w8_srf, w8_bed, w8_n : signed(11 downto 0) := (others => '0');
    signal w9_n, w9_dd, w9_sd : unsigned(9 downto 0) := (others => '0');
    signal w9_inw, w9_ins, w9_nb : std_logic := '0';
    signal w9_sw : unsigned(6 downto 0) := (others => '0');      -- the columns' sway: (sea level - surface) / 2
    signal w9_fv : signed(6 downto 0) := (others => '0');        -- foam square: foam rows past its top + jitter
    type t_r10 is array (10 to 12) of unsigned(9 downto 0);
    signal wd_r, wd_d : t_r10 := (others => (others => '0'));
    type t_q3 is array (10 to 12) of unsigned(2 downto 0);
    signal wd_q : t_q3 := (others => (others => '0'));
    type t_t7 is array (10 to 13) of unsigned(6 downto 0);
    signal wd_st : t_t7 := (others => (others => '0'));
    type t_s3 is array (10 to 13) of std_logic_vector(2 downto 0);
    signal wd_ss : t_s3 := (others => (others => '0'));       -- sand facets: column, row, diagonal
    signal wd_inw, wd_ins, wd_cp : std_logic_vector(10 to 13) := (others => '0');
    signal w13_q : unsigned(2 downto 0) := (others => '0');
    signal w13_stp : unsigned(7 downto 0) := (others => '0');      -- the sand's position + the drying fraction
    signal w5_fm, w6_fm, w7_fm, w8_fm : unsigned(7 downto 0) := (others => '0');
    signal w8_sph, w9_sph : unsigned(6 downto 0) := (others => '0');
    signal w9_up : std_logic := '0';
    signal w9_nn : signed(11 downto 0) := (others => '0');
    signal wd_inf, wd_sp : std_logic_vector(10 to 13) := (others => '0');
    type t_s2 is array (10 to 13) of std_logic_vector(1 downto 0);
    signal wd_ff : t_s2 := (others => "00");                               -- foam square: knot, row parities
    signal wd_ft : t_t7 := (others => (others => '0'));
    signal f14_in, sp14 : std_logic := '0';
    signal wd_up : std_logic_vector(10 to 13) := (others => '0');          -- the upper triangle of a sand cell
    type t_h2 is array (10 to 13) of unsigned(1 downto 0);
    signal wd_h : t_h2 := (others => "00");                                -- a 2-bit cell hash
    signal k14_wash : std_logic := '0';
    signal f14_sub : std_logic_vector(1 downto 0) := "00";
    signal f14_t : unsigned(6 downto 0) := (others => '0');
    signal w14_in, s14_in : std_logic := '0';
    signal w14_sub : std_logic_vector(1 downto 0) := "00";
    signal s14_sub : std_logic_vector(2 downto 0) := "000";
    signal s14_t : unsigned(6 downto 0) := (others => '0');
    signal w14_q : unsigned(2 downto 0) := (others => '0');

    -- sun, stars
    signal s12_xr, s12_xr2 : signed(12 downto 0) := (others => '0');
    signal s12_ax, s12_ax2 : unsigned(11 downto 0) := (others => '0');
    signal u14_in : std_logic := '0';
    signal t12_h, t13_h : unsigned(11 downto 0) := (others => '0');
    signal t12_x, t13_x : unsigned(2 downto 0) := (others => '0');
    signal t14_on : std_logic := '0';
    signal t14_tw : unsigned(1 downto 0) := (others => '0');
    -- the sky's ramp positions, registered ahead of the merge
    signal k14_skt : unsigned(5 downto 0) := (others => '0');

    -- sprite slots
    signal a12_xs, b12_xs : unsigned(7 downto 0) := (others => '0');
    signal a12_pr, b12_pr : std_logic := '0';
    signal a13_g, b13_g : std_logic_vector(5 downto 0) := (others => '0');
    signal a13_pr, b13_pr : std_logic := '0';
    signal a14_z, b14_z : unsigned(1 downto 0) := (others => '0');

    -- merge, edges, colour
    signal s15_id : std_logic_vector(7 downto 0) := (others => '0');
    signal s15_ent : unsigned(4 downto 0) := (others => '0');     -- the fill's colour table entry
    signal s15_t6 : unsigned(5 downto 0) := (others => '0');      -- and its lerp position
    signal s15_fon : std_logic := '0';
    signal s15_ovl : unsigned(1 downto 0) := (others => '0');
    signal s15_tw : unsigned(1 downto 0) := (others => '0');
    signal i_d1, i_d2, i_d3 : std_logic_vector(7 downto 0) := (others => '0');
    signal s_g15 : std_logic_vector(3 downto 0) := (others => '0');
    signal s16_e : std_logic := '0';
    signal s16_m1, s16_m2, s16_m3 : unsigned(3 downto 0) := (others => '0');
    signal s16_ent : unsigned(4 downto 0) := (others => '0');
    signal s16_t6 : unsigned(5 downto 0) := (others => '0');
    signal s16_fon : std_logic := '0';
    signal s16_ovl, s16_tw : unsigned(1 downto 0) := (others => '0');
    signal s17_ent : unsigned(4 downto 0) := (others => '0');
    signal s17_pn  : unsigned(2 downto 0) := (others => '0');
    signal s17_t6, s18_t6, s19_t6, s20_t6 : unsigned(5 downto 0) := (others => '0');
    signal s17_blk, s18_blk, s19_blk, s20_blk, s21_blk, s22_blk, s23_blk : std_logic := '1';
    signal s17_w, s18_w, s19_w, s20_w, s21_w, s22_w, s23_w : std_logic := '0';
    signal s17_dim, s18_dim, s19_dim, s20_dim, s21_dim, s22_dim : std_logic := '0';
    type t_ik is array (17 to 23) of std_logic_vector(1 downto 0);
    signal s_ik : t_ik := (others => "00");
    signal s24_y, s24_u, s24_v, h_u, h_v : unsigned(9 downto 0) := (others => '0');
    signal h_c : unsigned(2 downto 0) := (others => '0');
    type t_s3p is array (0 to 2) of signed(13 downto 0);
    signal s21_pa, s21_pb, s21_pc : t_s3p := (others => (others => '0'));
    type t_c3 is array (0 to 2) of unsigned(7 downto 0);
    signal s21_c0 : t_c3 := (others => (others => '0'));
    type t_r3 is array (0 to 2) of signed(10 downto 0);
    signal s22_s : t_r3 := (others => (others => '0'));
    signal s23_y, s23_u, s23_v : unsigned(9 downto 0) := (others => '0');

    -- [F] output conditioning
    -- the chroma filter's 8-pixel delay: a circular buffer in one block RAM (U and V are 8-bit here)
    type t_cr is array (0 to 255) of std_logic_vector(15 downto 0);
    signal cr_ram : t_cr := (others => (others => '0'));
    attribute no_rw_check of cr_ram : signal is true;
    signal cr_q, cr_qf : std_logic_vector(15 downto 0) := x"8080";
    signal cr_wp : unsigned(7 downto 0) := to_unsigned(6, 8);
    signal cr_rp : unsigned(7 downto 0) := to_unsigned(0, 8);
    -- and the luma's first five pixels of delay, in a second one
    signal yr_ram : t_cr := (others => (others => '0'));
    attribute no_rw_check of yr_ram : signal is true;
    signal yr_q : std_logic_vector(15 downto 0) := x"0040";
    signal yr_rp : unsigned(7 downto 0) := to_unsigned(3, 8);
    signal s_bu, s_bv2, s_bud, s_bvd : unsigned(12 downto 0) := to_unsigned(4100, 13);
    type t_y5 is array (5 to 7) of unsigned(9 downto 0);
    signal s_yd : t_y5 := (others => to_unsigned(64, 10));
    signal s_hist : std_logic_vector(7 downto 0) := (others => '0');
    signal o1_ya : unsigned(11 downto 0) := (others => '0');
    signal o1_yb, o2_y, o2_u, o2_v, o3_y, o3_u, o3_v, o4_y, o4_u, o4_v : unsigned(9 downto 0) := (others => '0');
begin

    -- NOTE: the order of the processes below is deliberate.  yosys abc9 (with ABC's &mfs) hits an assert
    -- ("Boxes are not in a topological order", then count_id) depending only on the order cells reach
    -- ABC; this order passes all six configs.  After any edit run .neonbeach_work/check6.sh, and if a
    -- config fails, try another order with .neonbeach_work/abc9_order.py.

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + the per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_e  : unsigned(9 downto 0);
        variable v_s  : signed(11 downto 0);
        variable v_c  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- [A] timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);

            -- [A] vblank zone: no avid for 1024 clocks
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
            elsif s_zc /= 8191 then
                s_zc <= s_zc + 1;
            end if;

            -- [A] width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_fsw   <= registers_in(6)(0);
                s_theme <= registers_in(6)(1);
                s_croff <= not registers_in(6)(2);       -- S9: 1 = On
                s_night <= registers_in(6)(3);
                s_k2    <= unsigned(registers_in(1));
                s_k1    <= unsigned(registers_in(0));
                s_k3    <= unsigned(registers_in(2));
                s_k4    <= unsigned(registers_in(3));
                s_k5    <= unsigned(registers_in(4));
                s_xs    <= s_xsr;
                -- wire thickness: 3 x 2 at 1080p, 2 x 2 at 720p, 3 x 1 at 1080i, 1 x 1 at SD
                if s_xsr = 2 then s_th2 <= '1'; s_th3 <= '1';
                elsif s_xsr = 3 then s_th2 <= '1'; s_th3 <= '0';
                else s_th2 <= '0'; s_th3 <= '0'; end if;
                if s_ys <= 384 then s_tv2 <= '1'; else s_tv2 <= '0'; end if;
            end if;
            g_wm1 <= s_w - 1;

            -- [B] shared glide (1/8 of the gap per field, +-2 deadband): fill (to the Fill switch), Speed, sun
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "00"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_gp; s_gr <= (others => s_fsw);
                            when 1      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_gs; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        -- outside the deadband (|gap| >= 32, as bit tests: a signed compare's carry fans out)
                        if (s_gd(14) = '0' and s_gd(13 downto 5) /= "000000000") or (s_gd(14) = '1' and s_gd(13 downto 5) /= "111111111") then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gs <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 2 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] per-field constants, one small step per state
            c_inj <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 40 =>
                        c_t <= c_t + 1;
                        c_e <= s_gs(12 downto 3);
                    when 41 =>                              -- the body's centre: -150 .. +217 rows
                        v_e := c_e;
                        c_sunc <= signed(resize(v_e(9 downto 2), 12)) + signed(resize(v_e(9 downto 3), 12))
                                  - signed(resize(v_e(9 downto 6), 12)) - 150;
                    -- time of day thresholds (with hysteresis), one compare a state
                    when 47 => if c_e > 616 then c_hi <= '1'; else c_hi <= '0'; end if;
                    when 48 => if c_e < 584 and c_e > 176 then c_mid <= '1'; else c_mid <= '0'; end if;
                    when 49 => if c_e < 144 then c_lo <= '1'; else c_lo <= '0'; end if;
                    when 42 =>                              -- sea level: 626 -+ 96 rows
                        -- 626 - 3 (K2 - 512) / 16 = 722 - K2 / 8 - K2 / 16 (exactly: 512 divides by both).  On
                        -- the unsigned knob: the old signed form added two sign-extended shifts of one value,
                        -- so its sign net fed both inputs of a LUT, and router1 stalled on HDMI seed 1 (v0.5.1)
                        c_sl <= to_unsigned(722, 11) - (resize(s_k2(9 downto 3), 11) + resize(s_k2(9 downto 4), 11));
                    when 43 =>
                        c_fp <= resize(s_gp, 14);
                    when 52 =>                              -- the crab, from the engine's last pass: half
                        c_cry <= resize(o_cy, 11) + 48;     -- size, its feet where v0.3's were, sinking at half
                    when 54 =>                              -- rate (hidden under its hole line at 96)
                        c_cry <= c_cry + resize(o_cs(6 downto 1), 11);
                    when 55 =>                              -- the water's keyframe interval and position: night,
                        if s_night = '1' then               -- dusk, golden over the slider's lower three
                            c_wiv <= "00"; c_wt <= (others => '0');            -- quarters, day above
                        elsif c_e(9 downto 8) = "11" then
                            c_wiv <= "10"; c_wt <= (others => '1');
                        else
                            c_wiv <= c_e(9 downto 8); c_wt <= c_e(7 downto 2);
                        end if;
                    when 53 =>
                        c_cyh <= resize(o_cy, 11) + 96;
                        c_cx2 <= o_cx & '0';
                    when 44 =>                              -- clouds and gulls drift on their own
                        c_sa1 <= c_t(13 downto 0);
                        c_sab <= c_t(11 downto 0) & "00";
                    when 45 =>                              -- time of day (with hysteresis): day, low sun, night
                        if s_night = '1' then c_tod <= "10";
                        elsif c_hi = '1' then c_tod <= "00";
                        elsif c_mid = '1' then c_tod <= "01";
                        elsif c_lo = '1' then c_tod <= "10";
                        end if;
                        c_esun <= not s_night; c_emoon <= s_night; c_ecrab <= '1';   -- (the engine burrows it)
                        if s_night = '0' and c_e(9 downto 6) /= "0000" then c_ebird <= '1'; else c_ebird <= '0'; end if;
                        if s_night = '1' or c_e(9 downto 6) = "0000" then c_sden <= "11";
                        elsif c_e(9 downto 6) = "0001" then c_sden <= "10";
                        elsif c_e(9 downto 6) = "0010" then c_sden <= "01";
                        else c_sden <= "00"; end if;
                    when 46 =>
                        c_pal <= s_theme & c_tod;
                    when others => null;
                end case;
            end if;

            -- [C] the per-field palette entries: nine made-up pixels, 8 clocks apart (s_zc 128 .. 192), run through
            -- the colour path from the keyframe slot; each comes out 7 clocks later and is kept or written (p_mix)
            --   0 .. 4  the water's keys K0 .. K4 (keyframe interval c_wiv, position c_wt): entries 4 .. 7
            --   5, 6    white: the water's and the wash's wires (K4 -> white), entries 21, 23
            --   7, 8    the sky's zenith and horizon keyframes, lerped by the elevation (day: all eight keyframes
            --           over the slider; night: the first two): entry 19
            if s_zone = '1' and s_zc(12 downto 7) = "000001" and s_zc(2 downto 0) = "000" and s_zc(6 downto 3) <= 8 then
                c_inj <= '1';
                c_injt <= c_wt;
                case to_integer(s_zc(6 downto 3)) is
                    when 0 => c_injent <= "100" & c_wiv; c_injk <= "01";
                    when 1 => c_injent <= "101" & c_wiv; c_injk <= "10"; c_injw <= to_unsigned(4, 5);
                    when 2 => c_injent <= "110" & c_wiv; c_injk <= "10"; c_injw <= to_unsigned(5, 5);
                    when 3 => c_injent <= "111" & c_wiv; c_injk <= "10"; c_injw <= to_unsigned(6, 5);
                    when 4 => c_injent <= '1' & c_wiv & "11"; c_injk <= "10"; c_injw <= to_unsigned(7, 5);
                    when 5 => c_injent <= "11111"; c_injk <= "11"; c_injw <= to_unsigned(21, 5);
                    when 6 => c_injent <= "11111"; c_injk <= "11"; c_injw <= to_unsigned(23, 5);
                    when 7 | 8 =>
                        -- (7: s_zc(3) = 1, the zenith, kept; 8: the horizon, written)
                        if s_night = '1' then
                            c_injent <= '0' & (not s_zc(3)) & "000"; c_injt <= c_e(9 downto 4);
                        else
                            c_injent <= '0' & (not s_zc(3)) & c_e(9 downto 7); c_injt <= c_e(6 downto 1);
                        end if;
                        if s_zc(3) = '1' then c_injk <= "01"; else c_injk <= "10"; end if;
                        c_injw <= to_unsigned(19, 5);
                    when others => null;
                end case;
            end if;

            -- [A] sync delay, generated-avid and start-pulse shift registers
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(1 to 15);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [C] BEAM COUNTERS: each layer's x (half units), loaded so that its results reach stage 14
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1'  then b_xw <= n_xw0; else b_xw <= b_xw + s_xs; end if;      -- stage 1
            if s_st(10) = '1' then                                                       -- stage 11
                b_xa <= n_xa0; b_xb <= n_xb0; b_x <= n_x0;
            else
                b_xa <= b_xa + s_xs;
                b_xb <= b_xb + signed(resize(s_xs, 14));
                b_x <= b_x + signed(resize(s_xs, 13));
            end if;
            if s_st(12) = '1' then b_px <= (others => '0'); else b_px <= b_px + 1; end if;   -- stage 13
            if s_st(16) = '1' then b_pa <= n_pa0; else b_pa <= b_pa + s_xs; end if;      -- stage 17
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- [C] LINE ENGINE: kicked when a line's last pixel has left stage 1 (and once in the zone for line 0),
    -- computes the NEXT line's constants in horizontal blanking.  The l_* constants are written at state 32
    -- or later, after the previous line's last pixel has left the stages that read them.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y   : unsigned(10 downto 0);
        variable v_d   : unsigned(11 downto 0);
        variable v_a   : unsigned(11 downto 0);
        variable v_b   : signed(11 downto 0);
        variable v_row : unsigned(7 downto 0);
        variable v_t   : unsigned(9 downto 0);
        variable v_g3, v_g5, v_g32 : unsigned(4 downto 0);
        variable v_th  : unsigned(4 downto 0);
        variable v_a4  : unsigned(11 downto 0);
        variable v_rm  : signed(11 downto 0);
        variable v_b4  : signed(11 downto 0);
        variable v_w   : signed(4 downto 0);
        variable v_by  : unsigned(7 downto 0);
        variable v_fo  : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;     -- after the field constants
            e_gq <= g_avid;
            e_kick <= e_gq and not g_avid;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_ln <= (others => '0');
            elsif e_kick = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                if e_ln /= 3 then e_ln <= e_ln + 1; end if;
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            -- fill flags (states 40 .. 47, ramp = the state's low bits): ramp i is filled out to
            -- (fill - order x 1024) rows from the horizon; the sun goes first, then the sky, then ramps 2 .. 7
            if e_lc(6 downto 3) = "0101" then
                if resize(e_d, 14) + e_fc < c_fp then l_fill(to_integer(e_lc(2 downto 0))) <= '1';
                else l_fill(to_integer(e_lc(2 downto 0))) <= '0'; end if;
            end if;
            v_fo := e_lc(2 downto 0) + 1;
            case to_integer(v_fo) is
                when 1      => e_fc <= to_unsigned(0, 14);        -- sun
                when 0      => e_fc <= to_unsigned(1024, 14);     -- sky
                when others => e_fc <= resize(v_fo & "0000000000", 14);
            end case;

            v_y := e_vy(18 downto 8);
            case to_integer(e_lc) is
                when 0 =>
                    e_s <= signed(resize(v_y, 12)) - C_HZ;
                when 1 =>
                    if e_s(11) = '1' then v_d := unsigned(-e_s); else v_d := unsigned(e_s); end if;
                    if v_d(11 downto 10) /= "00" then e_d <= (others => '1'); else e_d <= v_d(9 downto 0); end if;
                    e_ab <= e_s(11);
                when 2 =>
                    -- under the horizon every line is a mirror line: the sky objects at the same distance
                    e_mir <= not e_ab;
                    e_dyu <= signed(resize(e_d, 12)) - c_sunc;
                    e_c1  <= signed(resize(e_d, 11)) - 200;
                    e_bd  <= signed(resize(e_d, 11)) - 60;
                    e_cr  <= signed(resize(v_y, 12)) - signed(resize(c_cry, 12));
                    if v_y < c_cyh then e_cv <= '1'; else e_cv <= '0'; end if;    -- above its hole line
                    v_rm := to_signed(C_HZ, 12) - signed(resize(e_d, 12));
                    if v_rm(11) = '1' then e_rm <= (others => '0'); else e_rm <= unsigned(v_rm(8 downto 0)); end if;
                when 3 =>
                    -- sun / moon (radius 128): half-chord address, the sun's slits, ramp position
                    if e_dyu(11) = '1' then v_a := unsigned(-e_dyu); else v_a := unsigned(e_dyu); end if;
                    e_son <= '0';
                    e_sa <= v_a(6 downto 0) & '0';
                    if v_a < 128 then
                        if c_emoon = '1' then
                            e_son <= '1';
                        elsif c_esun = '1' then
                            -- slits below the centre: period 32 rows, opening with depth, rolling
                            v_g3 := v_a(4 downto 0) + c_t(5 downto 1);
                            v_th := v_a(6 downto 2) + resize(v_a(6 downto 3), 5);
                            if e_dyu(11) = '0' or v_g3 >= v_th then e_son <= '1'; end if;
                        end if;
                    end if;
                    -- across the disc, 0 .. 126.  (An elevation term added here tripped yosys abc9's box-order
                    -- assert in every config: the netlist is shape-sensitive, see SUMMARY.md.)
                    v_b := e_dyu + 128;
                    e_tb <= unsigned(v_b(7 downto 2));
                    -- slot A: the cloud or gull band this line crosses
                    e_aen <= '0'; e_asel <= '0'; e_asc <= c_sa1;
                    e_aa <= (others => '0');
                    if e_c1(10 downto 7) = "0000" and e_c1(6 downto 5) /= "11" then          -- 0 <= e_c1 < 96 as bit tests
                        v_row := to_unsigned(47, 8) - unsigned(e_c1(8 downto 1));
                        e_aa <= to_unsigned(128, 10) + (v_row(7 downto 0) & "00");
                        e_aen <= '1';
                    elsif c_ebird = '1' and e_bd(10 downto 6) = "00000" then                  -- 0 <= e_bd < 64
                        v_row := to_unsigned(31, 8) - unsigned(e_bd(8 downto 1));
                        if c_t(4) = '1' then e_aa <= to_unsigned(448, 10) + (v_row(7 downto 0) & "00");
                        else e_aa <= to_unsigned(320, 10) + (v_row(7 downto 0) & "00"); end if;
                        e_asel <= '1'; e_asc <= c_sab;
                        e_aen <= '1';
                    end if;
                    -- slot B: the crab (48 rows, one ROM row a line)
                    e_ben <= '0';
                    e_ba <= (others => '0');
                    if c_ecrab = '1' and e_cv = '1' and e_cr(11 downto 6) = "000000" and e_cr(5 downto 4) /= "11" then   -- 0 <= e_cr < 48
                        if o_cs(7) = '1' then e_ba <= to_unsigned(768, 10) + (unsigned(e_cr(5 downto 0)) & "00");
                        else e_ba <= to_unsigned(576, 10) + (unsigned(e_cr(5 downto 0)) & "00"); end if;
                        e_ben <= '1';
                    end if;
                when 4 =>                                    -- the moon's bite: a disc 24 rows up
                    e_tsun <= e_tb & '0';                 -- (a low sun is redder through the low-sun palette)
                    v_b4 := e_dyu - 24;
                    if v_b4(11) = '1' then v_a4 := unsigned(-v_b4); else v_a4 := unsigned(v_b4); end if;
                    e_sb <= v_a4(6 downto 0) & '0';
                    if c_emoon = '1' and v_a4 < 128 then e_bon <= '1'; else e_bon <= '0'; end if;
                when 5 =>                                    -- the line's left edge (the sea strip wobbles it)
                    v_w := (others => '0');
                    if e_mir = '1' then
                        v_g5 := resize(e_d(4 downto 1) + c_t(5 downto 2), 5);
                        if v_g5(3) = '1' then v_w := signed('0' & (not v_g5(2 downto 0)) & '0') - 7;
                        else v_w := signed('0' & v_g5(2 downto 0) & '0') - 7; end if;
                    end if;
                    e_x0 <= w_x0 + resize(v_w, 14);
                when 6 =>
                    n_xa0 <= unsigned(e_x0) + e_asc;
                    n_xw0 <= unsigned(w_x0 + 1920);          -- water and sand: half units from the canvas's left
                when 7 =>
                    n_x0 <= e_x0(12 downto 0);
                    n_xb0 <= signed(n_xw0) - signed(resize(c_cx2, 14));   -- the crab: half units from its centre line
                when 12 =>                                   -- the sheen's diagonal phase: x + y + 4 t
                    n_pa0 <= unsigned(e_x0) + resize(v_y & '0', 14) + (c_t(10 downto 0) & "000");
                when 32 =>
                    -- the line's constants (the previous line has left the pixel path by now)
                    l_d <= e_d; l_ab <= e_ab; l_mir <= e_mir; l_row <= v_y;
                    l_slr <= c_sl(7 downto 0) - v_y(7 downto 0);
                    l_son <= e_son; l_bon <= e_bon; l_tsun <= e_tsun;
                    l_asel <= e_asel;
                    -- slot A's ramp positions: zone 1, zone 2 (gulls: wings, body; clouds: top, shelf)
                    if e_asel = '1' then l_ta <= to_unsigned(70, 7); l_tb <= to_unsigned(93, 7);
                    else l_ta <= to_unsigned(104, 7); l_tb <= to_unsigned(127, 7); end if;
                    -- the sky ramp's position: 0 at the top, 63 at the horizon (by the mirrored row below it)
                    v_t := resize(e_rm(8 downto 3), 10) + resize(e_rm(8 downto 4), 10) - resize(e_rm(8 downto 7), 10);
                    if v_t > 63 then l_tsky <= (others => '1'); else l_tsky <= v_t(5 downto 0); end if;
                    -- sea strip ripples: gaps of a 16-row period that open with depth, drifting
                    v_g32 := resize(e_d(3 downto 0) + c_t(4 downto 1), 5);
                    if e_mir = '1' and v_g32(3 downto 0) <= e_d(9 downto 6) then l_rip <= '1'; else l_rip <= '0'; end if;
                    if e_ln /= 0 then l_v1 <= '1'; else l_v1 <= '0'; end if;
                    if e_ln(1) = '1' and s_tv2 = '1' then l_v2 <= '1'; else l_v2 <= '0'; end if;
                    lr_a <= "000" & e_sa(7 downto 1);
                when 33 => lr_a <= "000" & e_sb(7 downto 1);
                when 34 => lr_a <= e_aa;
                when 35 => lr_a <= e_aa(9 downto 2) & "01";
                    if e_sa(0) = '1' then v_by := lr_qf(15 downto 8); else v_by := lr_qf(7 downto 0); end if;
                    l_sw <= "00" & v_by(7 downto 1);
                when 36 => lr_a <= e_aa(9 downto 2) & "10";
                    if e_sb(0) = '1' then v_by := lr_qf(15 downto 8); else v_by := lr_qf(7 downto 0); end if;
                    l_bw <= "00" & v_by(7 downto 1);
                when 37 => lr_a <= e_aa(9 downto 2) & "11";
                    l_arow(0) <= lr_qf;
                when 38 => lr_a <= e_ba;
                    l_arow(1) <= lr_qf;
                when 39 => lr_a <= e_ba(9 downto 2) & "01";
                    l_arow(2) <= lr_qf;
                when 40 => lr_a <= e_ba(9 downto 2) & "10";
                    if e_aen = '1' then l_arow(3) <= lr_qf; else l_arow(3) <= (others => '0'); end if;
                when 41 => lr_a <= e_ba(9 downto 2) & "11";
                    l_brow(0) <= lr_qf;
                when 42 => l_brow(1) <= lr_qf;
                when 43 => l_brow(2) <= lr_qf;
                when 44 =>
                    if e_ben = '1' then l_brow(3) <= lr_qf; else l_brow(3) <= (others => '0'); end if;
                -- sheen: the other directions.  x + 4 t (the diagonal phase less y) and y - 2 t
                when 45 => l_od <= to_unsigned(0, 14) - resize(v_y & '0', 14);
                when 46 => l_pc <= resize(v_y & '0', 14) - (c_t(11 downto 0) & "00");
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent running 8-sums (resynced to
    -- neutral in blanking), luma [1 2 1]/4 at the chroma's group delay (+ 2 px for the glow), luma 13/16
    -- toward black, blanking gate.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_iu, v_iv, v_iy : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- the avid of the stage-24 pixel; neutral outside it
            if s_gav_sr(22) = '1' then
                v_iy := s24_y; v_iu := s24_u; v_iv := s24_v;
            else
                v_iy := to_unsigned(64, 10); v_iu := to_unsigned(512, 10); v_iv := to_unsigned(512, 10);
            end if;
            s_hist <= s_hist(6 downto 0) & s_gav_sr(22);
            -- cr_qf = the input 8 pixels back (written at t, read at t + 6 into cr_q, fabric register at t + 7)
            cr_ram(to_integer(cr_wp)) <= std_logic_vector(v_iu(9 downto 2)) & std_logic_vector(v_iv(9 downto 2));
            cr_q  <= cr_ram(to_integer(cr_rp));
            cr_qf <= cr_q;
            cr_wp <= cr_wp + 1; cr_rp <= cr_rp + 1;
            if s_hist = "00000000" and s_gav_sr(22) = '0' then
                s_bu <= to_unsigned(4100, 13); s_bv2 <= to_unsigned(4100, 13);
            else
                s_bu  <= s_bu + v_iu - (unsigned(cr_qf(15 downto 8)) & "00");
                s_bv2 <= s_bv2 + v_iv - (unsigned(cr_qf(7 downto 0)) & "00");
            end if;
            s_bud <= s_bu; s_bvd <= s_bv2;
            -- luma taps y(n - 5), y(n - 6), y(n - 7): the chroma's group delay plus the glow's 2 px.
            -- s_yd(5) = the input 5 pixels back (written at t, read at t + 3 into yr_q, fabric register at t + 4)
            yr_ram(to_integer(cr_wp)) <= "000000" & std_logic_vector(v_iy);
            yr_q  <= yr_ram(to_integer(yr_rp));
            yr_rp <= yr_rp + 1;
            s_yd(5) <= unsigned(yr_q(9 downto 0));
            for i in 6 to 7 loop s_yd(i) <= s_yd(i - 1); end loop;
            o1_ya <= resize(s_yd(5), 12) + resize(s_yd(7), 12) + 2;
            o1_yb <= s_yd(6);
            v_sc := resize(s_bu, 14) + s_bud;  o2_u <= v_sc(13 downto 4);
            v_sc := resize(s_bv2, 14) + s_bvd; o2_v <= v_sc(13 downto 4);
            v_y := o1_ya + shift_left(resize(o1_yb, 12), 1);
            o2_y <= v_y(11 downto 2);
            -- luma 13/16 toward black
            if o2_y > 64 then v_dl := o2_y - 64; else v_dl := (others => '0'); end if;
            o3_y <= o2_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            o3_u <= o2_u; o3_v <= o2_v;
            -- blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                o4_y <= o3_y; o4_u <= o3_u; o4_v <= o3_v;
            else
                o4_y <= to_unsigned(64, 10); o4_u <= to_unsigned(512, 10); o4_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;


    ---------------------------------------------------------------------------
    -- [A] Generated avid (parity-locked start 3/4 clocks after the source's avid rise, field-latched width)
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
    -- [C] PIXEL PATH, stages 2 .. 14: one block per object, each ending in a few bits at stage 14
    ---------------------------------------------------------------------------
    p_obj : process(clk)
        variable v_xn  : signed(12 downto 0);
        variable v_xr  : signed(12 downto 0);
        variable v_a12 : unsigned(11 downto 0);
        variable v_hh  : unsigned(11 downto 0);
        variable v_cell : unsigned(3 downto 0);
        variable v_ax  : unsigned(12 downto 0);
        variable v_bx  : signed(12 downto 0);
        variable v_ds  : signed(10 downto 0);
        variable v_db  : signed(10 downto 0);
        variable v_n, v_sd, v_dd : signed(11 downto 0);
        variable v_r2  : unsigned(10 downto 0);
        variable v_wv  : unsigned(3 downto 0);
        variable v_st8, v_d8 : unsigned(7 downto 0);
        variable v_j   : unsigned(3 downto 0);
        variable v_cx  : unsigned(6 downto 0);
        variable v_fa, v_fb : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            ----------------------------------------------------------------
            -- water and sand: the knot table read (2 .. 4), the surface and bed lerps (5 .. 7), the camera
            -- lift (8), the inside tests (9), the layer divide (10 .. 12), level of detail (13)
            ----------------------------------------------------------------
            -- 2
            kr_a <= c_dbank & b_xw(11 downto 5);
            w_f(2) <= b_xw(4 downto 1);
            w_xb(2) <= b_xw(11 downto 1);
            for i in 3 to 5 loop w_f(i) <= w_f(i - 1); end loop;
            for i in 3 to 10 loop w_xb(i) <= w_xb(i - 1); end loop;
            -- 5 .. 8: the steps times the fraction, as gated operands.  The bed's lerp runs one stage behind
            -- the surface's: with both sums gated by the same fraction bits in one stage, yosys abc9 hit its
            -- box-order assert in five of the six configs.
            v_ds := signed(kr_qf(37 downto 27));
            w5_ps1 <= f_gk(v_ds, w_f(4)(0), 0) + f_gk(v_ds, w_f(4)(1), 1);
            w5_ps2 <= f_gk(v_ds, w_f(4)(2), 2) + f_gk(v_ds, w_f(4)(3), 3);
            w5_s0 <= unsigned(kr_qf(47 downto 38));
            w5_b0 <= unsigned(kr_qf(26 downto 17));
            w5_db <= signed(kr_qf(16 downto 9));
            w5_fm <= unsigned(kr_qf(8 downto 1));
            -- 6
            v_db := resize(w5_db, 11);
            w6_ps <= w5_ps1 + w5_ps2;
            w6_fm <= w5_fm;
            w6_pb1 <= f_gk(v_db, w_f(5)(0), 0) + f_gk(v_db, w_f(5)(1), 1);
            w6_pb2 <= f_gk(v_db, w_f(5)(2), 2) + f_gk(v_db, w_f(5)(3), 3);
            w6_s0 <= signed(resize(w5_s0, 12));
            w6_b0 <= signed(resize(w5_b0, 12));
            -- 7
            w7_srf <= w6_s0 + resize(shift_right(w6_ps, 4), 12);
            w7_pb <= w6_pb1 + w6_pb2;
            w7_b0 <= w6_b0;
            w7_fm <= w6_fm;
            -- 8: rows under the surface; the bed.  (n, the rows under the sand and the depth are linearly
            -- dependent: computed in one stage they gave ABC's &mfs a redundancy to exploit, which broke
            -- abc9's carry-box order.  n goes one stage early.)
            w8_n   <= signed(resize(l_row, 12)) - w7_srf;
            w8_fm  <= w7_fm;
            -- the spray band over the surface: (foam - 64) / 2 rows where the foam is over 64
            if w7_fm(7 downto 6) /= "00" then w8_sph <= w7_fm(7 downto 1) - 32; else w8_sph <= (others => '0'); end if;
            w8_srf <= w7_srf;
            w8_bed <= w7_b0 + resize(shift_right(w7_pb, 4), 12);
            -- 9: the local depth, rows under the sand line
            v_n  := w8_n;
            v_sd := signed(resize(l_row, 12)) - w8_bed;
            v_dd := w8_bed - w8_srf;
            w9_inw <= (not v_n(11)) and v_sd(11);
            -- foam squares: 16 px (one knot) x 16 rows, riding the surface, the top four rows of squares.  A square's
            -- foam = the knot's foam / 4 (rows) less its top row and its own jitter (reshuffled 4 times a second):
            -- positive = foam, from turning white to white (stage 10)
            if v_n(10 downto 6) = "00000" then w9_nb <= '1'; else w9_nb <= '0'; end if;
            v_j := C_FJ(to_integer((w_xb(8)(7 downto 4) xor c_t(7 downto 4)) & unsigned(v_n(5 downto 4))));
            v_fa := '0' & w8_fm(7 downto 2);
            v_fb := '0' & unsigned(v_n(5 downto 4)) & v_j;
            w9_fv <= signed(v_fa) - signed(v_fb);
            -- the columns sway with the passing wave: x + (sea level - surface) / 2 (low bits only)
            w9_sw <= resize(shift_right(l_slr + unsigned(v_n(7 downto 0)), 1), 7);
            w9_up <= v_n(11) and v_sd(11);
            w9_nn <= -v_n;
            w9_sph <= w8_sph;
            w9_ins <= not v_sd(11);
            if v_n(11) = '1' then w9_n <= (others => '0');
            elsif v_n(10) = '1' then w9_n <= (others => '1');
            else w9_n <= unsigned(v_n(9 downto 0)); end if;
            if v_dd(11) = '1' then w9_dd <= (others => '0');
            elsif v_dd(10) = '1' then w9_dd <= (others => '1');
            else w9_dd <= unsigned(v_dd(9 downto 0)); end if;
            if v_sd(11) = '1' then w9_sd <= (others => '0');
            elsif v_sd(10) = '1' then w9_sd <= (others => '1');
            else w9_sd <= unsigned(v_sd(9 downto 0)); end if;
            -- 10 .. 12: q = floor(8 n / depth), one bit a stage (n < depth inside the water)
            v_r2 := w9_n & '0';
            if v_r2 >= resize(w9_dd, 11) then
                wd_r(10) <= resize(v_r2 - resize(w9_dd, 11), 10); wd_q(10) <= "001";
            else
                wd_r(10) <= v_r2(9 downto 0); wd_q(10) <= "000";
            end if;
            wd_d(10) <= w9_dd;
            for i in 11 to 12 loop
                v_r2 := wd_r(i - 1) & '0';
                if v_r2 >= resize(wd_d(i - 1), 11) then
                    wd_r(i) <= resize(v_r2 - resize(wd_d(i - 1), 11), 10); wd_q(i) <= wd_q(i - 1)(1 downto 0) & '1';
                else
                    wd_r(i) <= v_r2(9 downto 0); wd_q(i) <= wd_q(i - 1)(1 downto 0) & '0';
                end if;
                wd_d(i) <= wd_d(i - 1);
            end loop;
            -- 10: the sand's ramp position (by depth) and its facets (the water's colour is its layer: stage 15)
            wd_st(10) <= to_unsigned(127, 7) - w9_sd(9 downto 3);
            wd_ss(10) <= w_xb(9)(5) & w9_sd(5) & '0';
            -- the wetness cell under this pixel (row of depth, column), its upper triangle, its hash
            wt_pa <= w9_sd(9 downto 5) & w_xb(9)(10 downto 5);
            if w9_sd(4 downto 0) < w_xb(9)(4 downto 0) then wd_up(10) <= '1'; else wd_up(10) <= '0'; end if;
            wd_h(10) <= (w_xb(9)(5) xor w9_sd(6)) & (w_xb(9)(6) xor w9_sd(5));
            wd_inw(10) <= w9_inw; wd_ins(10) <= w9_ins;
            -- the water's columns: every 64 px of the swayed x
            v_cx := w_xb(9)(6 downto 0) + w9_sw;
            wd_cp(10) <= v_cx(6);
            -- foam squares: in the water, in the top four rows of squares, foam past the square's threshold.  Its
            -- whiteness: 8 rows past the threshold is white; its facets: the knot's parity, the square row's
            if w9_inw = '1' and w9_nb = '1' and w9_fv(6) = '0' and w9_fv /= 0 then wd_inf(10) <= '1'; else wd_inf(10) <= '0'; end if;
            if w9_fv(5 downto 3) /= "000" then wd_ft(10) <= (others => '1');
            else wd_ft(10) <= unsigned(w9_fv(2 downto 0)) & "1111"; end if;
            wd_ff(10) <= w_xb(9)(4) & w9_n(4);
            -- spray: dots in 8 px cells over the band, reshuffled a few times a second
            if w9_up = '1' and w9_nn(11 downto 7) = "00000" and unsigned(w9_nn(6 downto 0)) < w9_sph
               and C_DOT(to_integer((w_xb(9)(5 downto 3) xor c_t(5 downto 3)) & l_row(5 downto 3))) = '1' then
                wd_sp(10) <= '1';
            else
                wd_sp(10) <= '0';
            end if;
            for i in 11 to 13 loop
                wd_st(i) <= wd_st(i - 1); wd_ss(i) <= wd_ss(i - 1);
                wd_inw(i) <= wd_inw(i - 1); wd_ins(i) <= wd_ins(i - 1); wd_cp(i) <= wd_cp(i - 1);
                wd_inf(i) <= wd_inf(i - 1); wd_sp(i) <= wd_sp(i - 1); wd_ff(i) <= wd_ff(i - 1); wd_ft(i) <= wd_ft(i - 1);
                wd_up(i) <= wd_up(i - 1); wd_h(i) <= wd_h(i - 1);
            end loop;
            -- 13: always eight layers (switching the layer bit by depth drew stair-step verticals where the
            -- count changed); where they crowd near the shore they merge into a solid bright edge
            w13_q <= wd_q(12);
            w13_stp <= resize(wd_st(12), 8) + resize(ws_fr, 8);
            -- 14
            w14_in <= wd_inw(13); w14_sub <= w13_q(0) & wd_cp(13); w14_q <= w13_q;
            s14_in <= wd_ins(13);
            -- the wash: 15 = the whole cell, 14 = its upper triangle.  Drying sand keeps its diagonals for a while
            -- and loses them a triangle at a time (the upper ones last), by the cell's hash
            v_wv := unsigned(wt_qf);
            if v_wv = 15 or (v_wv = 14 and wd_up(13) = '1') then
                k14_wash <= '1'; s14_sub <= wd_ss(13)(2 downto 1) & wd_up(13);
            else
                k14_wash <= '0';
                if v_wv >= 14 or (wd_up(13) = '1' and v_wv >= 3 + resize(wd_h(13), 4))
                   or (wd_up(13) = '0' and v_wv >= 8 + resize(wd_h(13), 4)) then
                    s14_sub <= wd_ss(13)(2 downto 1) & wd_up(13);
                else
                    s14_sub <= wd_ss(13)(2 downto 1) & '0';
                end if;
            end if;
            -- wet sand: 8 steps of the ramp a wetness step (v0.3: 4), so a fresh wash leaves the sand dark brown;
            -- drying cells (12 .. 1) give back the fraction since their last step (no step in the colour)
            if v_wv /= 0 and v_wv < 13 then v_st8 := w13_stp; else v_st8 := resize(wd_st(13), 8); end if;
            if v_st8 > resize(v_wv & "000", 8) then
                v_d8 := v_st8 - resize(v_wv & "000", 8); s14_t <= v_d8(6 downto 0);
            else
                s14_t <= (others => '0');
            end if;
            f14_in <= wd_inf(13); f14_sub <= wd_ff(13); f14_t <= wd_ft(13); sp14 <= wd_sp(13);

            ----------------------------------------------------------------
            -- sun / moon
            ----------------------------------------------------------------
            -- (one carry chain a stage: the old chained subtract / abs / subtract / abs in one stage, on two
            -- related values, broke abc9's carry-box order under ABC's &mfs)
            -- 12: x from the disc's centre and from the bite's, both straight from the counter
            s12_xr  <= resize(b_x(12 downto 1), 13) - C_SUNX;
            s12_xr2 <= resize(b_x(12 downto 1), 13) - (C_SUNX + 50);
            -- 13: their magnitudes
            v_xn := -s12_xr;
            if s12_xr(12) = '1' then s12_ax <= unsigned(v_xn(11 downto 0)); else s12_ax <= unsigned(s12_xr(11 downto 0)); end if;
            v_xn := -s12_xr2;
            if s12_xr2(12) = '1' then s12_ax2 <= unsigned(v_xn(11 downto 0)); else s12_ax2 <= unsigned(s12_xr2(11 downto 0)); end if;
            -- 14
            u14_in <= '0';
            if l_son = '1' and s12_ax < resize(l_sw, 12) then
                if l_bon = '0' or s12_ax2 >= resize(l_bw, 12) then u14_in <= '1'; end if;
            end if;

            ----------------------------------------------------------------
            -- stars: 32 x 32 cells, up to three in thirty-two hold a 4 x 4 star; one phase in four it is dimmed
            ----------------------------------------------------------------
            -- 12
            v_a12 := unsigned(b_x(12 downto 6)) & l_d(9 downto 5);
            t12_h <= v_a12 + shift_left(v_a12, 3) + 693;
            t12_x <= unsigned(b_x(5 downto 3));
            -- 13
            v_hh := t12_h xor ("0000" & t12_h(11 downto 4)) xor ("0000000" & t12_h(11 downto 7));
            t13_h <= v_hh + shift_left(v_hh, 5) + 1447;
            t13_x <= t12_x;
            -- 14
            v_hh := t13_h xor ("000000" & t13_h(11 downto 6));
            if v_hh(11 downto 7) < resize(c_sden, 5) and v_hh(2 downto 0) = t13_x and v_hh(5 downto 3) = l_d(4 downto 2) then
                t14_on <= '1';
            else
                t14_on <= '0';
            end if;
            if v_hh(7 downto 6) = c_t(6 downto 5) then t14_tw <= "00"; else t14_tw <= "11"; end if;

            ----------------------------------------------------------------
            -- the sky's ramp position (filled)
            ----------------------------------------------------------------
            k14_skt <= l_tsky;

            ----------------------------------------------------------------
            -- sprite slots: span-coded rows.  A = clouds / gulls (2 virtual px a unit), B = the crab (folded)
            ----------------------------------------------------------------
            -- 12: lattice cell, presence, x inside the sprite
            v_cell := b_xa(13 downto 10);
            a12_xs <= b_xa(9 downto 2);
            if l_asel = '0' then a12_pr <= C_CM1(to_integer(v_cell)); else a12_pr <= C_BM(to_integer(v_cell)); end if;
            v_bx := b_xb(13 downto 1);
            if v_bx(12) = '1' then v_ax := unsigned(-v_bx); else v_ax := unsigned(v_bx); end if;
            b12_xs <= v_ax(6 downto 0) & '0';                      -- half size: two ROM units a virtual px
            if v_ax(12 downto 7) = "000000" then b12_pr <= '1'; else b12_pr <= '0'; end if;
            -- 13: thermometer against the six boundaries
            for i in 0 to 2 loop
                if a12_xs >= l_arow(i)(7 downto 0)  then a13_g(2 * i) <= '1';     else a13_g(2 * i) <= '0'; end if;
                if a12_xs >= l_arow(i)(15 downto 8) then a13_g(2 * i + 1) <= '1'; else a13_g(2 * i + 1) <= '0'; end if;
                if b12_xs >= l_brow(i)(7 downto 0)  then b13_g(2 * i) <= '1';     else b13_g(2 * i) <= '0'; end if;
                if b12_xs >= l_brow(i)(15 downto 8) then b13_g(2 * i + 1) <= '1'; else b13_g(2 * i + 1) <= '0'; end if;
            end loop;
            a13_pr <= a12_pr; b13_pr <= b12_pr;
            -- 14: the zone of the interval
            if a13_pr = '0' then a14_z <= "00";
            elsif a13_g(5) = '1' then a14_z <= l_arow(3)(13 downto 12);
            elsif a13_g(4) = '1' then a14_z <= l_arow(3)(11 downto 10);
            elsif a13_g(3) = '1' then a14_z <= l_arow(3)(9 downto 8);
            elsif a13_g(2) = '1' then a14_z <= l_arow(3)(7 downto 6);
            elsif a13_g(1) = '1' then a14_z <= l_arow(3)(5 downto 4);
            elsif a13_g(0) = '1' then a14_z <= l_arow(3)(3 downto 2);
            else a14_z <= l_arow(3)(1 downto 0); end if;
            if b13_pr = '0' then b14_z <= "00";
            elsif b13_g(5) = '1' then b14_z <= l_brow(3)(13 downto 12);
            elsif b13_g(4) = '1' then b14_z <= l_brow(3)(11 downto 10);
            elsif b13_g(3) = '1' then b14_z <= l_brow(3)(9 downto 8);
            elsif b13_g(2) = '1' then b14_z <= l_brow(3)(7 downto 6);
            elsif b13_g(1) = '1' then b14_z <= l_brow(3)(5 downto 4);
            elsif b13_g(0) = '1' then b14_z <= l_brow(3)(3 downto 2);
            else b14_z <= l_brow(3)(1 downto 0); end if;
        end if;
    end process p_obj;


    ---------------------------------------------------------------------------
    -- [C] WET SAND: the RAM (one write statement; the pixel path reads it in active video) and the sweep (in
    -- vblank, after the sky keys): per column a reach in quarter cells from the swash sheet (the engine's
    -- envelope and shoreline, from its last pass), then its 32 cells read, updated and written back one a clock.
    -- The wash wets only what the sheet touches: the top row of cells, whole under the sheet, the upper triangle
    -- at its front column.  (v0.5.0 and earlier soaked e rows deep, about 112 at the defaults, under a sheet at
    -- most 14 rows thick, with drifting lobes past its front: the top of the beach went water-coloured and then
    -- dark where no water was, row after row as the swash drained.)
    ---------------------------------------------------------------------------
    p_wet : process(clk)
        variable v_l9  : signed(8 downto 0);
        variable v_rch : signed(9 downto 0);
        variable v_top : signed(9 downto 0);
        variable v_o, v_n : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            if s_zone = '1' and ws_rows = '1' then wt_a <= ws_r0(4 downto 0) & ws_col; else wt_a <= wt_pa; end if;
            wt_q  <= wt_ram(to_integer(wt_a));
            wt_qf <= wt_q;
            if wt_we = '1' then wt_ram(to_integer(wt_wa)) <= wt_wd; end if;
            -- the drying fraction: round(8 x ws_div / 12), quasi-static through the field
            case to_integer(ws_div) is
                when 0      => ws_fr <= "000";
                when 1 | 2  => ws_fr <= "001";
                when 3      => ws_fr <= "010";
                when 4 | 5  => ws_fr <= "011";
                when 6      => ws_fr <= "100";
                when 7 | 8  => ws_fr <= "101";
                when 9      => ws_fr <= "110";
                when others => ws_fr <= "111";
            end case;

            -- the row pipeline: the cell issued two clocks ago is in wt_qf now
            ws_r1 <= ws_r0(4 downto 0); ws_r2 <= ws_r1;
            ws_c1 <= ws_col; ws_c2 <= ws_c1;
            ws_v1 <= ws_rows and not ws_r0(5); ws_v2 <= ws_v1;
            ws_rc2 <= ws_reach;
            wt_we <= '0';
            if ws_v2 = '1' then
                v_o := unsigned(wt_qf);
                v_top := signed(resize(ws_r2 & "00", 10));
                if ws_rc2 >= v_top + 4 then v_n := to_unsigned(15, 4);
                elsif ws_rc2 > v_top then v_n := to_unsigned(14, 4);
                elsif v_o >= 14 then v_n := to_unsigned(12, 4);
                elsif ws_tick = '1' and v_o /= 0 then v_n := v_o - 1;
                else v_n := v_o; end if;
                if ws_uw = '1' and ws_r2 = 0 then v_n := to_unsigned(13, 4); end if;    -- the sea bed's top row: damp
                if o_cs(6 downto 0) >= 90 and (ws_r2 & ws_c2) = o_ch then v_n := to_unsigned(13, 4); end if;   -- the crab's hole
                wt_we <= '1'; wt_wa <= ws_r2 & ws_c2; wt_wd <= std_logic_vector(v_n);
            end if;

            if s_zone = '1' and s_zc = 120 then
                ws_run <= '1'; ws_rows <= '0'; ws_col <= (others => '0'); ws_p <= "000";
                ws_dp <= o_e(7 downto 3);
                v_l9 := signed(resize(o_ks, 9)) - signed(resize(o_e(7 downto 3), 9));
                ws_fc <= shift_right(v_l9, 1);                                     -- the run-up's front column
                if ws_div = 11 then ws_div <= (others => '0'); ws_tick <= '1'; else ws_div <= ws_div + 1; ws_tick <= '0'; end if;
            elsif ws_run = '1' then
                if ws_rows = '0' then
                    case ws_p is
                        when "000" =>
                            if (ws_col & '1') >= o_ks(6 downto 0) and o_ks(7) = '0' then ws_uw <= '1'; else ws_uw <= '0'; end if;
                            ws_dist <= ws_fc - signed(resize(ws_col, 9));
                            ws_p <= "001";
                        when "001" =>
                            -- under the sheet: the whole top cell (4 quarter cells); its front column: the upper
                            -- triangle (2); past the front, under the sea or no swash: nothing
                            if ws_uw = '1' or ws_dp = 0 or ws_dist(8) = '0' then
                                if ws_uw = '0' and ws_dp /= 0 and ws_dist = 0 then ws_reach <= to_signed(2, 10);
                                else ws_reach <= (others => '0'); end if;
                            else
                                ws_reach <= to_signed(4, 10);
                            end if;
                            ws_p <= "100";
                        when others =>
                            ws_rows <= '1'; ws_r0 <= (others => '0');
                    end case;
                else
                    if ws_r0(5) = '0' then
                        ws_r0 <= ws_r0 + 1;
                    else
                        ws_rows <= '0'; ws_p <= "000";
                        if ws_col = 63 then ws_run <= '0'; else ws_col <= ws_col + 1; end if;
                    end if;
                end if;
            end if;
        end if;
    end process p_wet;

    p_eng : process(clk)
        variable v_op  : integer range 0 to 31;
        variable v_d, v_ra, v_rb : unsigned(3 downto 0);
        variable v_i11 : unsigned(10 downto 0);
        variable v_n   : natural range 0 to 3;
        variable v_x, v_y : unsigned(16 downto 0);
        variable v_res : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            -- the program, the scratch RAM (one write statement), the wave shapes: registered reads
            u_iq <= std_logic_vector(C_NB_UCODE(to_integer(u_pc)));
            sc_q <= sc_ram(to_integer(sc_ra));
            if sc_we = '1' then sc_ram(to_integer(sc_wa)) <= sc_wd; end if;
            sh_q <= C_NB_SHAPE(to_integer(u_a(8 downto 0)));
            sc_we <= '0';
            kr_we <= '0';
            u_lfsr <= u_lfsr(14 downto 0) & (u_lfsr(15) xor u_lfsr(13) xor u_lfsr(12) xor u_lfsr(10));
            -- the serial multiplier: one partial product a clock
            if m_c /= 0 then
                if m_y(0) = '1' then m_p <= m_p + m_x; end if;
                m_x <= m_x(22 downto 0) & '0';
                m_y <= '0' & m_y(7 downto 1);
                m_c <= m_c - 1;
            end if;

            v_op  := to_integer(unsigned(u_ir(23 downto 19)));
            v_d   := unsigned(u_ir(18 downto 15));
            v_ra  := unsigned(u_ir(14 downto 11));
            v_rb  := unsigned(u_ir(10 downto 7));
            v_i11 := unsigned(u_ir(10 downto 0));
            v_n   := to_integer(unsigned(u_ir(1 downto 0)));

            -- (every enable and select below comes from a register decoded two or more clocks earlier: the
            -- execute paths start at flip-flops, not at the instruction decode; 10 clocks an instruction)
            p6 <= '0'; p9 <= '0';

            -- started at every field start once it has finished (the bank it wrote goes on screen); a
            -- watchdog restarts it if four field starts pass without it finishing
            if s_zgo = '1' and (u_run = '0' or u_wd = 3) then
                if u_run = '0' then c_dbank <= not c_dbank; end if;
                u_run <= '1'; u_pc <= (others => '0'); u_ph <= (others => '0'); u_wd <= (others => '0');
            elsif s_zgo = '1' then
                u_wd <= u_wd + 1;
            elsif u_run = '1' then
                case to_integer(u_ph) is
                    when 0 => u_ph <= to_unsigned(1, 4);                 -- (program word read)
                    when 1 =>
                        u_ir <= u_iq;
                        u_ph <= to_unsigned(2, 4);
                    when 2 =>
                        sc_ra <= "11111" & v_ra;                         -- operand a
                        -- decode, off the execute paths
                        if v_op = 3 or v_op = 25 or v_op = 26 or v_op = 27 then d_sub <= '1'; else d_sub <= '0'; end if;
                        if v_op = 27 then d_xz <= '1'; d_ya <= '1'; else d_xz <= '0'; d_ya <= '0'; end if;
                        if v_op = 4 then d_yi <= '1'; else d_yi <= '0'; end if;
                        d_rs <= (others => '0');                         -- phase 6 result: one-hot
                        case v_op is
                            when 1      => d_rs(0) <= '1';               -- LDI
                            when 5      => d_rs(1) <= '1';               -- SHR
                            when 6      => d_rs(2) <= '1';               -- SHL
                            when 7      => d_rs(3) <= '1';               -- AND
                            when 13     => d_rs(4) <= '1';               -- IN
                            when 16     => d_rs(5) <= '1';               -- TAB
                            when 23     => d_rs(6) <= '1';               -- TRI
                            when 27     => d_rs(7) <= '1';               -- ABS (a, negated in 8)
                            when others => null;
                        end case;
                        d_fs <= (others => '0');                         -- phase 8 result: one-hot
                        case v_op is
                            when 2 | 3 | 4 => d_fs(0) <= '1';            -- the sum
                            when 27     => d_fs(1) <= '1';               -- ABS
                            when 25     => d_fs(2) <= '1';               -- MIN
                            when 26     => d_fs(3) <= '1';               -- MAX
                            when 17     => d_fs(4) <= '1';               -- FRD
                            when 15     => d_fs(5) <= '1';               -- SHP
                            when 9      => d_fs(6) <= '1';               -- MRD
                            when others => null;
                        end case;
                        d_mrd <= v_i11(1 downto 0);
                        if v_op = 9 then d_mw <= '1'; else d_mw <= '0'; end if;
                        if v_op = 8 then d_mul <= '1'; else d_mul <= '0'; end if;
                        if v_op = 17 then d_frd <= '1'; else d_frd <= '0'; end if;
                        if v_op = 18 then d_fwr <= '1'; else d_fwr <= '0'; end if;
                        if v_op = 20 then d_kwr <= '1'; else d_kwr <= '0'; end if;
                        if v_op = 22 then d_end <= '1'; else d_end <= '0'; end if;
                        d_kf <= (others => '0'); d_of <= (others => '0');
                        if v_op = 19 then d_kf(to_integer(v_i11(2 downto 0))) <= '1'; end if;   -- KST field
                        if v_op = 21 then d_of(to_integer(v_i11(2 downto 0))) <= '1'; end if;   -- OUT port
                        d_jc <= "000";
                        case v_op is
                            when 10     => d_jc <= "100";                -- JMP
                            when 11     => d_jc <= "101";                -- JN
                            when 12     => d_jc <= "110";                -- JZ
                            when 24     => d_jc <= "111";                -- JP
                            when others => null;
                        end case;
                        case v_op is
                            when 1 | 2 | 3 | 4 | 5 | 6 | 7 | 9 | 13 | 15 | 16 | 17 | 23 | 25 | 26 | 27 =>
                                if v_d /= 0 then d_wr <= '1'; else d_wr <= '0'; end if;
                            when others => d_wr <= '0';
                        end case;
                        d_dst <= v_d;
                        d_tgt <= unsigned(u_ir(8 downto 0));
                        d_imm <= unsigned(resize(signed(u_ir(14 downto 0)), 16));
                        d_i11 <= v_i11;
                        d_sh  <= v_n;
                        -- the input port, fetched now
                        case to_integer(v_i11(2 downto 0)) is
                            when 0      => d_in <= resize(s_k1, 16);
                            when 1      => d_in <= resize(s_k3, 16);
                            when 2      => d_in <= resize(s_k4, 16);
                            when 3      => d_in <= resize(s_k5, 16);
                            when 4      => d_in <= resize(c_sl, 16);
                            when 5      => d_in <= c_t;
                            when 6      => d_in <= u_lfsr;
                            when others => d_in <= resize(s_g6(12 downto 3) & s_croff, 16);   -- Speed, crab off
                        end case;
                        u_ph <= to_unsigned(3, 4);
                    when 3 =>
                        sc_ra <= "11111" & v_rb;                         -- operand b
                        u_ph <= to_unsigned(4, 4);
                    when 4 =>
                        if v_ra = 0 then u_a <= (others => '0'); else u_a <= sc_q; end if;
                        u_ph <= to_unsigned(5, 4);
                    when 5 =>
                        if v_rb = 0 then u_b <= (others => '0'); else u_b <= sc_q; end if;
                        if d_frd = '1' then sc_ra <= u_a(8 downto 0); end if;                           -- FRD
                        -- the table entry, fetched now (from a)
                        if d_i11(0) = '0' then d_tab <= resize(C_NB_DT2(to_integer(u_a(3 downto 0))), 16);
                        else d_tab <= resize(C_NB_SIZE(to_integer(u_a(3 downto 0))), 16); end if;
                        p6 <= '1';
                        u_ph <= to_unsigned(6, 4);
                    when 6 =>
                        -- execute: the adder into u_sum; the simple results into u_res; the units (p6-gated)
                        if d_xz = '1' then v_x := (others => '0'); else v_x := unsigned(resize(signed(u_a), 17)); end if;
                        if d_yi = '1' then v_y := unsigned(resize(signed(d_i11), 17));
                        elsif d_ya = '1' then v_y := unsigned(resize(signed(u_a), 17));
                        else v_y := unsigned(resize(signed(u_b), 17)); end if;
                        if d_sub = '1' then v_y := not v_y; end if;
                        u_sum <= v_x + v_y + ("" & d_sub);
                        v_res := (others => '0');
                        if d_rs(0) = '1' then v_res := v_res or d_imm; end if;
                        if d_rs(1) = '1' then v_res := v_res or shift_right(u_a, d_sh + 1); end if;
                        if d_rs(2) = '1' then v_res := v_res or shift_left(u_a, d_sh + 1); end if;
                        if d_rs(3) = '1' then v_res := v_res or (u_a and resize(d_i11, 16)); end if;
                        if d_rs(4) = '1' then v_res := v_res or d_in; end if;
                        if d_rs(5) = '1' then v_res := v_res or d_tab; end if;
                        if d_rs(6) = '1' then v_res := v_res or resize(f_tri7(u_a(15 downto 8)), 16); end if;
                        if d_rs(7) = '1' then v_res := v_res or u_a; end if;
                        u_res <= v_res;
                        -- branches
                        u_npc <= u_pc + 1;
                        if d_jc(2) = '1' then
                            case d_jc(1 downto 0) is
                                when "00"   => u_npc <= d_tgt;
                                when "01"   => if u_a(15) = '1' then u_npc <= d_tgt; end if;
                                when "10"   => if u_a = 0 then u_npc <= d_tgt; end if;
                                when others => if u_a(15) = '0' and u_a /= 0 then u_npc <= d_tgt; end if;
                            end case;
                        end if;
                        u_ph <= to_unsigned(7, 4);
                    when 7 =>
                        sc_qf <= sc_q; sh_qf <= sh_q;                    -- block RAM data into fabric first
                        if d_mw = '0' or m_c = 0 then u_ph <= to_unsigned(8, 4); end if;   -- MRD waits
                    when 8 =>
                        if d_fs(0) = '1' then u_res <= u_sum(15 downto 0); end if;
                        if d_fs(1) = '1' and u_a(15) = '1' then u_res <= u_sum(15 downto 0); end if;
                        if d_fs(2) = '1' then if u_sum(16) = '1' then u_res <= u_a; else u_res <= u_b; end if; end if;
                        if d_fs(3) = '1' then if u_sum(16) = '1' then u_res <= u_b; else u_res <= u_a; end if; end if;
                        if d_fs(4) = '1' then u_res <= sc_qf; end if;
                        if d_fs(5) = '1' then u_res <= unsigned(resize(signed(sh_qf), 16)); end if;
                        if d_fs(6) = '1' then
                            case to_integer(d_mrd) is
                                when 0      => u_res <= m_p(15 downto 0);
                                when 1      => u_res <= m_p(21 downto 6);
                                when 2      => u_res <= m_p(22 downto 7);
                                when others => u_res <= m_p(18 downto 3);
                            end case;
                        end if;
                        p9 <= '1';
                        u_ph <= to_unsigned(9, 4);
                    when others =>
                        u_pc <= u_npc;
                        u_ph <= (others => '0');
                end case;
            end if;

            -- the units, gated by the registered phase strobes and the pre-decoded flags
            if p6 = '1' then
                if d_mul = '1' then
                    m_p <= (others => '0'); m_x <= resize(u_a, 24); m_y <= u_b(7 downto 0);
                    m_c <= to_unsigned(8, 4);
                end if;
                if d_fwr = '1' then sc_we <= '1'; sc_wa <= u_a(8 downto 0); sc_wd <= u_b; end if;
                if d_kf(0) = '1' then k_srf <= u_a(9 downto 0); end if;
                if d_kf(1) = '1' then k_ds  <= u_a(10 downto 0); end if;
                if d_kf(2) = '1' then k_bed <= u_a(9 downto 0); end if;
                if d_kf(3) = '1' then k_db  <= u_a(7 downto 0); end if;
                if d_kf(4) = '1' then k_fm  <= u_a(7 downto 0); end if;
                if d_kf(5) = '1' then k_w   <= u_a(0); end if;
                if d_kwr = '1' then
                    kr_we <= '1';
                    kr_wa <= (not c_dbank) & u_a(6 downto 0);
                    kr_wd <= std_logic_vector(k_srf) & std_logic_vector(k_ds) & std_logic_vector(k_bed)
                             & std_logic_vector(k_db) & std_logic_vector(k_fm) & k_w;
                end if;
                if d_of(0) = '1' then o_e  <= u_a(7 downto 0); end if;
                if d_of(1) = '1' then o_ks <= u_a(7 downto 0); end if;
                if d_of(2) = '1' then o_cx <= u_a(9 downto 0); end if;
                if d_of(3) = '1' then o_cy <= u_a(9 downto 0); end if;
                if d_of(4) = '1' then o_cs <= u_a(7 downto 0); end if;
                if d_of(5) = '1' then o_ch <= u_a(10 downto 0); end if;
                if d_end = '1' then u_run <= '0'; end if;
            end if;
            if p9 = '1' then                                             -- write back
                sc_we <= d_wr; sc_wa <= "11111" & d_dst; sc_wd <= u_res;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- [E] MERGE, EDGES, COLOUR (stages 15 .. 24)
    ---------------------------------------------------------------------------
    p_mix : process(clk)
        variable v_obj  : unsigned(3 downto 0);
        variable v_sub  : std_logic_vector(3 downto 0);
        variable v_ent  : unsigned(4 downto 0);
        variable v_t6   : unsigned(5 downto 0);
        variable v_fon  : std_logic;
        variable v_e    : std_logic;
        variable v_w    : unsigned(3 downto 0);
        variable v_tri  : unsigned(4 downto 0);
        variable v_src  : std_logic_vector(1 downto 0);
        variable v_sc   : std_logic;
        variable v_ph   : unsigned(13 downto 0);
        variable v_gi   : unsigned(5 downto 0);
        variable v_sum  : signed(13 downto 0);
        variable v_c    : signed(10 downto 0);
        variable v_d8   : signed(7 downto 0);
        type t_o3 is array (0 to 2) of unsigned(7 downto 0);
        variable v_out  : t_o3;
        variable v_y10  : unsigned(9 downto 0);
        variable v_k9   : signed(8 downto 0);
    begin
        if rising_edge(clk) then
            ----------------------------------------------------------------
            -- 15: who owns the pixel, nearest first: crab, wash, sand, foam, water, clouds / gulls, sun / moon, sky.
            -- Each fill names its colour table entry and lerp position directly (ramp r: entry 2 r + t(6)).
            ----------------------------------------------------------------
            -- (no adds in this stage: every position arrives registered.  An add, a clamp and per-arm offsets
            -- here tripped yosys abc9's box-order assert in the full build.)
            v_obj := "0000"; v_sub := "0000";
            if l_fill(0) = '1' then                      -- the sky's per-field colours: 19
                v_fon := '1';
                v_ent := to_unsigned(19, 5); v_t6 := k14_skt;
            else
                v_fon := '0';
                v_ent := (others => '0'); v_t6 := (others => '0');
            end if;
            if b14_z /= 0 then
                v_obj := "1001"; v_sub := "00" & std_logic_vector(b14_z);
                if b14_z = "01" then v_sub(2) := l_brow(3)(14); end if;       -- creases across the shell
                case b14_z is                                                 -- (the facet bands alternate)
                    when "01"   => if l_brow(3)(14) = '1' then v_ent := "01110"; v_t6 := to_unsigned(44, 6);
                                   else v_ent := "01111"; v_t6 := to_unsigned(0, 6); end if;
                    when "10"   => v_ent := "01110"; v_t6 := to_unsigned(20, 6);
                    when others => v_ent := "01111"; v_t6 := to_unsigned(36, 6);
                end case;
                v_fon := l_fill(7);
            elsif s14_in = '1' and k14_wash = '1' then   -- the wash: the water's top colour (3/4 of K3 -> K4;
                v_obj := "0111"; v_sub := '0' & s14_sub; -- halfway went grey at sunset)
                v_ent := to_unsigned(7, 5); v_t6 := to_unsigned(48, 6); v_fon := l_fill(4);
            elsif s14_in = '1' then
                v_obj := "1000"; v_sub := '0' & s14_sub;
                v_ent := "0101" & s14_t(6); v_t6 := s14_t(5 downto 0); v_fon := l_fill(5);
            elsif f14_in = '1' then                      -- foam squares: 28 / 29, turning white
                v_obj := "0110"; v_sub := "00" & f14_sub;
                v_ent := "1110" & f14_t(6); v_t6 := f14_t(5 downto 0); v_fon := l_fill(3);
            elsif w14_in = '1' then                      -- water: layer q at t = 127 - 16 q of entries 4 .. 7
                v_obj := "0101"; v_sub := "00" & w14_sub;
                v_ent := "001" & (not w14_q(2 downto 1)); v_t6 := (not w14_q(0)) & "11110"; v_fon := l_fill(2);
            elsif a14_z /= 0 then
                v_obj := "0010"; v_sub := "00" & std_logic_vector(a14_z);
                if a14_z = "10" then v_ent := "0110" & l_tb(6); v_t6 := l_tb(5 downto 0);
                else v_ent := "0110" & l_ta(6); v_t6 := l_ta(5 downto 0); end if;
                v_fon := l_fill(6);
            elsif u14_in = '1' then
                v_obj := "0001"; v_ent := "0001" & l_tsun(6); v_t6 := l_tsun(5 downto 0); v_fon := l_fill(1);
            end if;
            s15_id <= std_logic_vector(v_obj) & v_sub;
            s15_ent <= v_ent; s15_t6 <= v_t6; s15_fon <= v_fon;
            if sp14 = '1' and v_obj < 5 then s15_ovl <= "10";
            elsif t14_on = '1' and v_obj = 0 then s15_ovl <= "01";
            else s15_ovl <= "00"; end if;
            s15_tw <= t14_tw;

            -- the ID history along the line, and the line store (this line's ID above last line's)
            i_d1 <= s15_id; i_d2 <= i_d1; i_d3 <= i_d2;
            s_g15 <= s_g15(2 downto 0) & s_gav_sr(13);
            lb_we <= s_gav_sr(13);
            if s_st(15) = '1' then lb_wa <= (others => '0'); else lb_wa <= lb_wa + 1; end if;     -- pixel index at stage 16
            lb_wd <= lb_qf(7 downto 0) & s15_id;

            ----------------------------------------------------------------
            -- 16: a wire wherever the ID changed within the last pixels or lines; its colour = the nearest
            -- object among the neighbours
            ----------------------------------------------------------------
            v_e := '0';
            if s_g15(0) = '1' and s15_id /= i_d1 then v_e := '1'; end if;
            if s_g15(1) = '1' and s_th2 = '1' and s15_id /= i_d2 then v_e := '1'; end if;
            if s_g15(2) = '1' and s_th3 = '1' and s15_id /= i_d3 then v_e := '1'; end if;
            if l_v1 = '1' and s15_id /= lb_qf(7 downto 0) then v_e := '1'; end if;
            if l_v2 = '1' and s15_id /= lb_qf(15 downto 8) then v_e := '1'; end if;
            s16_e  <= v_e;
            s16_m1 <= f_max4(unsigned(s15_id(7 downto 4)), unsigned(i_d1(7 downto 4)));
            s16_m2 <= f_max4(unsigned(i_d2(7 downto 4)), unsigned(i_d3(7 downto 4)));
            s16_m3 <= f_max4(unsigned(lb_qf(7 downto 4)), unsigned(lb_qf(15 downto 12)));
            s16_ent <= s15_ent; s16_t6 <= s15_t6; s16_fon <= s15_fon;
            s16_ovl <= s15_ovl; s16_tw <= s15_tw;

            ----------------------------------------------------------------
            -- 17: colour table entry and lerp position; the sea strip's ripple gaps; the made-up sky pixels
            ----------------------------------------------------------------
            v_w := f_max4(f_max4(s16_m1, s16_m2), s16_m3);
            s17_blk <= '0'; s17_w <= '0'; s17_pn <= c_pal; s_ik(17) <= "00";
            if s16_ovl = "10" then                      -- spray
                s17_ent <= to_unsigned(16, 5); s17_t6 <= (others => '0');
            elsif s16_e = '1' then
                s17_ent <= '1' & v_w; s17_w <= '1';
                s17_t6 <= (others => '0');
            elsif s16_ovl = "01" then
                s17_ent <= to_unsigned(27, 5); s17_t6 <= s16_tw & "0000";
            elsif s16_fon = '1' then
                s17_ent <= s16_ent; s17_t6 <= s16_t6;
            else
                s17_ent <= (others => '0'); s17_t6 <= (others => '0');
                s17_blk <= '1';
            end if;
            -- the mirror (objects below the water's number): ripple gaps and dimming
            if l_rip = '1' and v_w < 5 then s17_blk <= '1'; s17_w <= '0'; end if;
            if l_mir = '1' and v_w < 5 then s17_dim <= '1'; else s17_dim <= '0'; end if;
            if c_inj = '1' then
                s17_ent <= c_injent; s17_t6 <= c_injt; s17_pn <= s_theme & "11";
                s17_blk <= '0'; s17_w <= '0'; s17_dim <= '0';
                s_ik(17) <= c_injk;
            end if;

            -- 18 / 19: the colour table (address register, EBR, fabric register)
            pal_a <= s17_pn & s17_ent;
            s18_t6 <= s17_t6; s18_blk <= s17_blk;
            s19_t6 <= s18_t6; s19_blk <= s18_blk;
            s20_blk <= s19_blk;                          -- pal_q at 19, pal_qf at 20
            s18_w <= s17_w; s19_w <= s18_w; s20_w <= s19_w; s21_w <= s20_w; s22_w <= s21_w; s23_w <= s22_w;
            s18_dim <= s17_dim; s19_dim <= s18_dim; s20_dim <= s19_dim; s21_dim <= s20_dim; s22_dim <= s21_dim;
            for i in 18 to 23 loop s_ik(i) <= s_ik(i - 1); end loop;

            ----------------------------------------------------------------
            -- 18 .. 20: SHEEN, the light that runs along the wires (a wire's lerp position toward the
            -- palette's hot colour): a 64-px triangle pulse in every 256 of a phase.  Sun and clouds / gulls
            -- read the diagonal phase with every pulse shown; water, foam, wash, sand and crab each read
            -- their own phase and scale, and a table keyed by the pulse's number and the object lets about
            -- three pulses in five through.
            --   A  x + y + 4 t   "/" fronts      C  y - 2 t   bands moving down      D  x + 4 t   bands moving left
            ----------------------------------------------------------------
            case s17_ent(3 downto 0) is
                when "0101" => v_src := "11"; v_sc := '0';       -- water          D, 256
                when "0110" => v_src := "11"; v_sc := '0';       -- foam           D, 256
                when "0111" => v_src := "11"; v_sc := '1';       -- wash           D, 512
                when "1000" => v_src := "10"; v_sc := '1';       -- sand           C, 512
                when "1001" => v_src := "11"; v_sc := '0';       -- crab           D, 256
                when others => v_src := "00"; v_sc := '0';
            end case;
            if unsigned(s17_ent(3 downto 0)) < 5 then
                v_src := "00"; v_sc := '0'; s18_ng <= '1';
            else
                s18_ng <= '0';
            end if;
            if v_src = "11" then v_ph := b_pa + l_od; else v_ph := b_pa; end if;
            if v_src = "10" then v_ph := l_pc; end if;
            if v_sc = '1' then s18_np <= v_ph(9 downto 1); s18_ix <= v_ph(12 downto 10);
            else s18_np <= v_ph(8 downto 0); s18_ix <= v_ph(11 downto 9); end if;
            s18_wo <= s17_ent(2 downto 0);
            -- 19: the pulse (a triangle, 0 .. 31)
            v_gi := s18_ix & s18_wo;
            if s18_np(6) = '1' then v_tri := not s18_np(5 downto 1); else v_tri := s18_np(5 downto 1); end if;
            if s18_np(8 downto 7) /= "00" or (s18_ng = '0' and C_SHG(to_integer(v_gi)) = '0') then
                v_tri := (others => '0');
            end if;
            s19_n6 <= v_tri & '0';
            -- 20
            if s19_w = '1' then s20_t6 <= s19_n6; else s20_t6 <= s19_t6; end if;

            ----------------------------------------------------------------
            -- 21 / 22: c0 + (delta x t) >> 5, the product as three pairs of gated operands
            ----------------------------------------------------------------
            for k in 0 to 2 loop
                v_d8 := signed(pal_qf(23 - 8 * k downto 16 - 8 * k));
                s21_pa(k) <= f_gate(v_d8, s20_t6(0), 0) + f_gate(v_d8, s20_t6(1), 1);
                s21_pb(k) <= f_gate(v_d8, s20_t6(2), 2) + f_gate(v_d8, s20_t6(3), 3);
                s21_pc(k) <= f_gate(v_d8, s20_t6(4), 4) + f_gate(v_d8, s20_t6(5), 5);
                s21_c0(k) <= pal_qf(47 - 8 * k downto 40 - 8 * k);
            end loop;
            s21_blk <= s20_blk;
            for k in 0 to 2 loop
                v_sum := s21_pa(k) + s21_pb(k) + s21_pc(k);
                s22_s(k) <= signed(resize(s21_c0(k), 11)) + resize(shift_right(v_sum, 5), 11);
            end loop;
            s22_blk <= s21_blk;

            -- 23: clamp, black, the mirror's dimming
            for k in 0 to 2 loop
                v_c := s22_s(k);
                if v_c(10) = '1' then v_out(k) := (others => '0');
                elsif v_c(9 downto 8) /= "00" then v_out(k) := (others => '1');
                else v_out(k) := unsigned(v_c(7 downto 0)); end if;
            end loop;
            v_y10 := v_out(0) & "00";
            if s22_dim = '1' and v_y10 > 64 then
                v_y10 := v_y10 - shift_right(v_y10 - 64, 2);
            end if;
            if s22_blk = '1' then
                s23_y <= to_unsigned(64, 10); s23_u <= to_unsigned(512, 10); s23_v <= to_unsigned(512, 10);
            else
                if v_y10 < 64 then s23_y <= to_unsigned(64, 10); else s23_y <= v_y10; end if;
                s23_u <= v_out(1) & "00"; s23_v <= v_out(2) & "00";
            end if;
            s23_blk <= s22_blk;

            -- the per-field palette entries from the made-up pixels: keep the key (01, 10), and write a ramp
            -- segment into the live palette (10, 11): c0 = the previous key, delta = half the step to this one
            pal_we <= '0';
            if s_ik(23)(1) = '1' then
                pal_we <= '1';
                pal_wa <= c_pal & c_injw;
                pal_wd(47 downto 24) <= unsigned(c_kp);
                v_k9 := signed(resize(s23_y(9 downto 2), 9)) - signed(resize(unsigned(c_kp(23 downto 16)), 9)); pal_wd(23 downto 16) <= unsigned(v_k9(8 downto 1));
                v_k9 := signed(resize(s23_u(9 downto 2), 9)) - signed(resize(unsigned(c_kp(15 downto 8)), 9));  pal_wd(15 downto 8)  <= unsigned(v_k9(8 downto 1));
                v_k9 := signed(resize(s23_v(9 downto 2), 9)) - signed(resize(unsigned(c_kp(7 downto 0)), 9));   pal_wd(7 downto 0)   <= unsigned(v_k9(8 downto 1));
            end if;
            if s_ik(23) = "01" or s_ik(23) = "10" then
                c_kp <= std_logic_vector(s23_y(9 downto 2)) & std_logic_vector(s23_u(9 downto 2)) & std_logic_vector(s23_v(9 downto 2));
            end if;

            -- 24: neon glow.  A wire's chroma is held over the black pixels that follow it (4 px), so the
            -- wide chroma filter below returns a wire at close to full saturation instead of a pastel, with
            -- a faint coloured halo on black; the luma is delayed 2 px to sit in the middle of it.
            s24_y <= s23_y; s24_u <= s23_u; s24_v <= s23_v;
            if s23_w = '1' then
                h_u <= s23_u; h_v <= s23_v; h_c <= to_unsigned(4, 3);
            elsif s23_blk = '1' and h_c /= 0 then
                s24_u <= h_u; s24_v <= h_v; h_c <= h_c - 1;
            else
                h_c <= (others => '0');
            end if;
        end if;
    end process p_mix;

    ---------------------------------------------------------------------------
    -- [C] FILLER (vblank): writes the knot table from the right edge to the left.  v0.1.0: the surface is
    -- the sea level wherever the bed is below it, and the bed itself on the beach.
    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- ROMs, the colour table (a RAM: vblank rewrites the per-field entries), the ID line store and the knot table
    -- (one write statement each; registered reads, re-registered in fabric).
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            lr_q   <= C_NB_LROM(to_integer(lr_a));
            lr_qf  <= lr_q;
            pal_q  <= pal_ram(to_integer(pal_a));
            pal_qf <= pal_q;
            if pal_we = '1' then pal_ram(to_integer(pal_wa)) <= pal_wd; end if;
        end if;
    end process p_rom;

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            lb_q  <= lb_ram(to_integer(b_px));
            lb_qf <= lb_q;
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lb;

    p_kr : process(clk)
    begin
        if rising_edge(clk) then
            kr_q  <= kr_ram(to_integer(kr_a));
            kr_qf <= kr_q;
            if kr_we = '1' then kr_ram(to_integer(kr_wa)) <= kr_wd; end if;
        end if;
    end process p_kr;

    w_x0 <= to_signed(-1440, 14) when s_xs = 4 else to_signed(-1920, 14);
    data_out.y       <= std_logic_vector(o4_y);
    data_out.u       <= std_logic_vector(o4_u);
    data_out.v       <= std_logic_vector(o4_v);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture neonbeach;
