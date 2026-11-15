-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Euclid v0.1.1 -- Patchbay: a Euclidean step sequencer drawn as a piano roll, through a colouriser.
--
-- The patch:
--   clock (Tempo) -> one step counter per track (length n, or n, n-1, n-2, n-3 in Poly)
--   step counter  -> Euclidean pattern generator (k hits over n steps, rotated per track) -> gate
--   gate          -> envelope generator (a pure function of the steps since the strike; Tails) -> VCA level
--   H ramp x Steps  -> step phase -> pad window (Gate) -> triangle shaper
--   V ramp x Tracks -> row phase  -> triangle shaper
--   (H triangle + V triangle, clipped) x (rest level + envelope + envelope-gated camera S&H)
--                 -> slew limiter along the scan (the streak behind every pad)
--                 -> + accent bus (the row glows after a strike) + playhead pulse
--                 -> colouriser (16 hues x 64 levels; the hue steps from track to track by Colour)
--   breath LFO -> rest level of each track (phase-shifted per track); hum bar -> level
--   camera -> 8-px box -> sample-and-hold per pad (clocked by the step grid, held down the row) -> VCA
--   (Osc: a drifting triangle lattice, not held, at twice the camera's level)
--
--   K1  Steps    n = 4 .. 32
--   K2  Gate     pad width within its step (30 % .. 100 %)
--   K3  Tracks   2 .. 12 rows
--   K4  Tempo    bipolar clock rate (square law, +-8 deadband): centre = stopped, envelopes still breathe
--   K5  Colour   hue step from track to track (0 = one hue .. 2.75 hue steps per track)
--   K6  Camera   source level x K6 / 256 (0 = none, 25 % = v0.1.0, 50 % = twice, 100 % = four times)
--       (Rotate, the old K5, is fixed at 1 step per track: C_KFIX_5)
--   S7  Layout   Sweep (the playhead crosses a fixed grid) / Roll (the grid, at twice the size, scrolls
--                under a playhead fixed at the centre)
--   S8  Tails    Long / Short (envelope decay and the streak behind each pad; Short strikes harder)
--   S9  Accent   Off / On: a strike lights its whole row, decaying on the long envelope
--   S10 Metre    Same / Poly (n, n-1, n-2, n-3: each row is one loop, so the playheads drift apart)
--   S11 Source   Video / Osc
--   P12 FILL     0 % four-on-the-floor on every track .. 100 % every step a hit
--
-- Engine: vblank-zone sequencer (shared glide, one serial multiplier, a per-track loop that builds each
-- track's hit mask with accumulators and writes its record into a 128 x 16 track RAM), a per-line engine
-- kicked at each line start (computes the NEXT line: row, vertical triangle, the row's record), a beam
-- phase accumulator, a 15-stage pixel path.  Rows fill the 1080 virtual lines, steps fill the width
-- (a count, so pads are narrower on 4:3 SD).  Generated avid; output conditioning (Y, U and V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes with a per-line resync, luma 13/16) before the blanking gate.  Colours in
-- the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K5 = Colour (was K6); K5 Rotate fixed at 256 (C_KFIX_5).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.euclid_pkg.all;
use work.all;

architecture euclid of program_top is
    constant C_KFIX_5 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(256, 10));   -- K6 camera pass: fixed knob
    constant C_LAT  : integer := 20;   -- = processing_delay_clks
    constant C_NOGLIDE : boolean := false;   -- true only in a sim-only copy (knobs take effect at once)

    constant C_K  : std_logic_vector(31 downto 0) := C_EU_PAL(0);         -- the colouriser's black
    constant c_ky : unsigned(9 downto 0) := unsigned(C_K(31 downto 22));
    constant c_ku : unsigned(9 downto 0) := unsigned(C_K(21 downto 12));
    constant c_kv : unsigned(9 downto 0) := unsigned(C_K(11 downto 2));

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
    -- row phase per field line, per track: (ystep * 3884) >> 8  (3884 / 2^22 = 1 / (1080 * 4) rows per virtual line)
    function f_kys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(17265, 16);
            when 4          => return to_unsigned(8739, 16);
            when 8          => return to_unsigned(14565, 16);
            when 12         => return to_unsigned(7282, 16);
            when 5 | 6 | 14 => return to_unsigned(5826, 16);
            when 1 | 2 | 10 => return to_unsigned(7768, 16);
            when others     => return to_unsigned(3884, 16);
        end case;
    end function;
    -- x step in h per pixel: 1080 = 2, 720p = 3, SD = 4
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
    -- per-track fill offsets (Q8)
    function f_ofs(t : unsigned(3 downto 0)) return unsigned is
    begin
        case to_integer(t) is
            when 0      => return to_unsigned(0, 9);
            when 1      => return to_unsigned(159, 9);
            when 2      => return to_unsigned(61, 9);
            when 3      => return to_unsigned(220, 9);
            when 4      => return to_unsigned(123, 9);
            when 5      => return to_unsigned(26, 9);
            when 6      => return to_unsigned(184, 9);
            when 7      => return to_unsigned(87, 9);
            when 8      => return to_unsigned(245, 9);
            when 9      => return to_unsigned(148, 9);
            when 10     => return to_unsigned(51, 9);
            when others => return to_unsigned(210, 9);
        end case;
    end function;

    -- line buffer: the 8-bit source level x K6 gain (camera: boxed luma; Osc: the lattice).  Written 3 clocks
    -- after the source pixel, read 5..6 clocks after it (the generated avid), so no address collides.
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    -- K6 camera gain on the source level, two stages before the write (multiply, clip)
    signal s_sl  : unsigned(6 downto 0) := (others => '0');
    signal s_slp : unsigned(16 downto 0) := (others => '0');
    signal s_lb_we1, s_lb_we2 : std_logic := '0';
    signal s_lb_wa1, s_lb_wa2 : unsigned(10 downto 0) := (others => '0');
    signal c_cg  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ev_a : unsigned(9 downto 0) := (others => '0');
    signal r_ev_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_pl_a : unsigned(9 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    -- track RAM: 16 tracks x 8 words x 16 bits; written by the sequencer in the vblank zone only, read by
    -- the sequencer (zone) and the line engine (active lines)
    type t_tk is array (0 to 255) of std_logic_vector(15 downto 0);
    signal tk : t_tk := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of tk : signal is true;
    signal tk_we : std_logic := '0';
    signal tk_wa, tk_ra : unsigned(6 downto 0) := (others => '0');
    signal tk_wd, tk_q, tk_qf : std_logic_vector(15 downto 0) := (others => '0');
    signal tk_sa, tk_la : unsigned(6 downto 0) := (others => '0');

    -- sample-and-hold of the source per pad: written on the first line of each row, read on the others
    type t_hr is array (0 to 255) of std_logic_vector(7 downto 0);
    signal hr : t_hr := (others => x"00");
    attribute no_rw_check of hr : signal is true;
    signal h_we : std_logic := '0';
    signal h_wa, h_ra : unsigned(4 downto 0) := (others => '0');
    signal h_wd, h_q : std_logic_vector(7 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    -- camera luma box (8 px, resynced every line)
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    signal s_bx   : t_u8(0 to 7) := (others => (others => '0'));
    signal s_bsum : unsigned(10 downto 0) := (others => '0');
    signal s_bxh  : std_logic_vector(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_kys  : unsigned(15 downto 0) := to_unsigned(3884, 16);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw tempo, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(441 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(500 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(605 * 8, 13);
    signal s_gc : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);   -- K6 Camera (the freed Rotate channel)
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(200 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(370 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(784, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_roll, s_short, s_acc, s_poly, s_src : std_logic := '0';

    -- per-field constants
    signal c_n    : unsigned(5 downto 0) := to_unsigned(16, 6);       -- steps
    signal c_t    : unsigned(3 downto 0) := to_unsigned(8, 4);        -- tracks
    signal c_g9   : unsigned(8 downto 0) := to_unsigned(164, 9);      -- gate, 1/256 step
    signal c_vsq  : unsigned(17 downto 0) := (others => '0');
    signal c_d    : unsigned(8 downto 0) := (others => '0');          -- |clock step| per field, Q11 steps
    signal c_spr  : unsigned(9 downto 0) := to_unsigned(137, 10);     -- hue step per track, Q8
    signal c_rinc : unsigned(14 downto 0) := to_unsigned(1942, 15);   -- row phase per line, Q18
    signal c_hinc : unsigned(7 downto 0) := to_unsigned(48, 8);       -- hum phase per line
    signal c_c25  : unsigned(15 downto 0) := to_unsigned(17476, 16);  -- 2^25 / width
    signal c_lfo  : unsigned(15 downto 0) := (others => '0');         -- breath LFO
    signal c_humb : unsigned(15 downto 0) := (others => '0');         -- hum bar phase
    signal c_dec  : unsigned(4 downto 0) := to_unsigned(2, 5);        -- slew fall per pixel, 1/8 levels

    -- field serial multiplier
    signal f_a   : unsigned(23 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(23 downto 0) := (others => '0');
    signal z_fa  : unsigned(23 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(2 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- track sequencer (one pass per field, started in the zone)
    signal q_st   : unsigned(5 downto 0) := (others => '0');
    attribute fsm_encoding : string;
    attribute fsm_encoding of q_st : signal is "none";
    signal q_busy, q_done, q_go : std_logic := '0';
    signal q_t    : unsigned(3 downto 0) := (others => '0');
    signal q_nt   : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal q_phi  : unsigned(8 downto 0) := (others => '0');
    signal q_dens : unsigned(8 downto 0) := (others => '0');
    signal q_kt   : unsigned(5 downto 0) := to_unsigned(4, 6);
    signal q_s1   : signed(18 downto 0) := (others => '0');
    signal q_s0   : unsigned(15 downto 0) := (others => '0');
    signal q_lim  : unsigned(16 downto 0) := (others => '0');
    signal q_inc  : unsigned(20 downto 0) := (others => '0');
    signal q_l    : signed(18 downto 0) := (others => '0');
    signal q_a    : unsigned(5 downto 0) := (others => '0');
    signal q_rc   : unsigned(5 downto 0) := (others => '0');
    signal q_j    : unsigned(5 downto 0) := (others => '0');
    signal q_msk  : std_logic_vector(31 downto 0) := (others => '0');
    signal q_le, q_la : unsigned(5 downto 0) := (others => '0');
    signal q_have : std_logic := '0';
    signal q_ra   : unsigned(5 downto 0) := (others => '0');
    signal q_ac   : unsigned(3 downto 0) := (others => '0');
    signal q_rest : unsigned(3 downto 0) := to_unsigned(5, 4);
    signal q_racc : unsigned(13 downto 0) := (others => '0');
    signal q_hacc : unsigned(13 downto 0) := (others => '0');
    signal q_bph  : unsigned(7 downto 0) := (others => '0');
    signal q_wc   : unsigned(4 downto 0) := (others => '0');
    signal q_ld   : std_logic := '0';
    signal q_eva  : unsigned(9 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_lc   : unsigned(4 downto 0) := to_unsigned(31, 5);
    signal e_zk   : std_logic := '0';
    signal e_r    : unsigned(21 downto 0) := (others => '0');
    signal e_hacc : unsigned(15 downto 0) := (others => '0');
    signal e_prow : unsigned(3 downto 0) := (others => '1');
    signal e_dm   : signed(8 downto 0) := (others => '0');
    signal e_hum  : unsigned(2 downto 0) := (others => '0');
    signal e_ac   : unsigned(3 downto 0) := (others => '0');
    signal e_rest : unsigned(3 downto 0) := (others => '0');
    signal n_row  : unsigned(3 downto 0) := (others => '0');
    signal n_blank, n_cap : std_logic := '0';
    signal n_ry   : unsigned(5 downto 0) := (others => '0');
    signal n_mask : std_logic_vector(31 downto 0) := (others => '0');
    signal n_inc  : unsigned(20 downto 0) := (others => '0');
    signal n_nt, n_ntm1 : unsigned(5 downto 0) := (others => '0');
    signal n_s, n_l : unsigned(15 downto 0) := (others => '0');
    signal n_hue  : unsigned(3 downto 0) := (others => '0');
    signal n_resth : unsigned(5 downto 0) := (others => '0');
    signal n_glow : unsigned(3 downto 0) := (others => '0');

    -- line registers (loaded at the line start)
    signal l_blank, l_cap : std_logic := '0';
    signal l_ry   : unsigned(5 downto 0) := (others => '0');
    signal l_mask : std_logic_vector(31 downto 0) := (others => '0');
    signal l_inc  : unsigned(20 downto 0) := (others => '0');
    signal l_nt, l_ntm1 : unsigned(5 downto 0) := (others => '0');
    signal l_si   : unsigned(4 downto 0) := (others => '0');
    signal l_sf8  : unsigned(7 downto 0) := (others => '0');
    signal l_hue  : unsigned(3 downto 0) := (others => '0');
    signal l_resth : unsigned(5 downto 0) := (others => '0');
    signal l_glow : unsigned(3 downto 0) := (others => '0');

    -- beam
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_fr  : unsigned(24 downto 0) := (others => '0');
    signal b_c   : unsigned(4 downto 0) := (others => '0');

    -- pixel path
    signal p2_hit, p3_hit, p4_hit, p5_hit, p6_hit, p7_hit, p8_hit : std_logic := '0';
    signal p2_e   : signed(5 downto 0) := (others => '0');
    signal p2_fr8, p3_fr8 : unsigned(7 downto 0) := (others => '0');
    signal p2_c, p3_c : unsigned(4 downto 0) := (others => '0');
    signal p3_e   : unsigned(4 downto 0) := (others => '0');
    signal p3_d2  : signed(9 downto 0) := (others => '0');
    signal p3_ge  : std_logic := '0';
    signal p3_lm, p4_lm, p5_lm, p6_lm, p6_lh, p7_lum : unsigned(7 downto 0) := (others => '0');
    signal p2_f8, p3_f8 : std_logic := '0';
    signal p4_age : unsigned(4 downto 0) := (others => '0');
    signal p4_past, p5_pastq : std_logic := '1';
    signal p4_rx  : unsigned(5 downto 0) := (others => '0');
    signal p5_roof, p6_roof, p7_roof, p8_roof, p9_roof : unsigned(5 downto 0) := (others => '0');
    signal p5_ph, p6_ph, p7_ph, p8_ph, p9_ph, p10_ph, p11_ph : std_logic := '0';
    signal p5_phc : unsigned(4 downto 0) := (others => '0');
    signal p7_env : unsigned(6 downto 0) := (others => '0');
    signal p7_cw  : unsigned(4 downto 0) := (others => '0');
    signal p8_cam : unsigned(12 downto 0) := (others => '0');
    signal p8_base : unsigned(7 downto 0) := (others => '0');
    signal p9_v   : unsigned(6 downto 0) := (others => '0');
    signal p10_f  : unsigned(5 downto 0) := (others => '0');
    signal p11_l  : unsigned(8 downto 0) := (others => '0');
    signal p14_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 inputs of boxes A, B
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), ROMs, track
    -- RAM and the per-pad hold RAM
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ev_q <= C_EU_ENV(to_integer(r_ev_a));
            r_pl_q <= C_EU_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    p_tk : process(clk)
    begin
        if rising_edge(clk) then
            tk_q <= tk(to_integer('0' & tk_ra));
            if tk_we = '1' then tk(to_integer('0' & tk_wa)) <= tk_wd; end if;
            tk_qf <= tk_q;
            if q_busy = '1' then tk_ra <= tk_sa; else tk_ra <= tk_la; end if;
        end if;
    end process p_tk;

    p_hr : process(clk)
    begin
        if rising_edge(clk) then
            h_q <= hr(to_integer("000" & h_ra));
            if h_we = '1' then hr(to_integer("000" & h_wa)) <= h_wd; end if;
        end if;
    end process p_hr;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_t  : unsigned(19 downto 0);
        variable v_x  : signed(10 downto 0);
        variable v_n  : unsigned(5 downto 0);
        variable v_a  : unsigned(6 downto 0);
        variable v_b  : std_logic;
        variable v_p  : unsigned(10 downto 0);
        variable v_k  : unsigned(15 downto 0);
        variable v_r  : unsigned(6 downto 0);
        variable v_h  : unsigned(7 downto 0);
        variable v_w  : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_kys <= f_kys(s_tid);
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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_roll  <= registers_in(6)(0);
                s_short <= registers_in(6)(1);
                s_acc   <= registers_in(6)(2);
                s_poly  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); c_c25 <= to_unsigned(46603, 16);
                    when 3      => s_oxs <= to_unsigned(153, 9); c_c25 <= to_unsigned(26214, 16);
                    when others => s_oxs <= to_unsigned(102, 9); c_c25 <= to_unsigned(17476, 16);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            c_cg  <= s_gc(12 downto 3);   -- K6 camera gain, x / 256 (changes in the zone only)
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);
            c_hinc <= resize(shift_right(s_ys, 3), 8) + resize(shift_right(s_ys, 4), 8);
            -- slew fall per pixel in 1/8 levels: Long = x step, Short = 4 x step
            if s_short = '1' then c_dec <= s_xs & "00"; else c_dec <= resize(s_xs, 5); end if;

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters (no zc decode on these enables)
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_gc; s_gr <= unsigned(registers_in(5));
                            when 4      => s_gm <= s_g6; s_gr <= unsigned(registers_in(4));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if C_NOGLIDE then
                            null;
                        elsif (z_gc /= 3 and (s_gd > 16 or s_gd < -16)) or s_gd > 32 or s_gd < -32 then   -- Camera: +-4
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_gc <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products.  Global slots: slot k loads at zc = 32 + 24 k, 16 iterations, result
            -- stable from load + 17.  The slot number is decoded at 30 + 24 k and the operands are
            -- pre-muxed into z_fa / z_fb at 31 + 24 k.  The track sequencer loads its own operands
            -- through q_ld afterwards.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 3); z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 3); z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 3); z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 3); z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 3); z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 3); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre or q_ld;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0      => z_fa <= resize(s_g1(12 downto 3), 24); z_fb <= to_unsigned(29, 16);     -- steps
                    when 1      => z_fa <= resize(s_g3(12 downto 3), 24); z_fb <= to_unsigned(11, 16);     -- tracks
                    when 2      => z_fa <= resize(s_g2(12 downto 3), 24); z_fb <= to_unsigned(179, 16);    -- gate
                    when 3      => z_fa <= resize(s_vm, 24);              z_fb <= resize(s_vm, 16);        -- tempo^2
                    when 4      => z_fa <= resize(s_g6(12 downto 3), 24); z_fb <= to_unsigned(11, 16);     -- hue step
                    when others => z_fa <= resize(c_t, 24);               z_fb <= s_kys;                   -- row phase step
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(22 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            q_go <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 20  =>                       -- the free-running modulators
                        c_lfo  <= c_lfo + 218;
                        c_humb <= c_humb + 150;
                    when 50  => c_n   <= resize(f_acc(14 downto 10), 6) + 4;
                    when 74  => c_t   <= f_acc(13 downto 10) + 2;
                    when 98  => c_g9  <= resize(f_acc(17 downto 10), 9) + 77;
                    when 122 => c_vsq <= f_acc(17 downto 0);
                    when 123 =>                       -- |d| = 3 vm^2 >> 11
                        v_t := resize(c_vsq, 20) + shift_left(resize(c_vsq, 20), 1);
                        c_d <= v_t(19 downto 11);
                    when 146 => c_spr <= f_acc(13 downto 4);
                    when 170 => c_rinc <= f_acc(18 downto 4);
                    when 200 => q_go <= '1';
                    when others => null;
                end case;
            end if;

            -------------------------------------------------------------------
            -- Track sequencer: one pass over the 12 tracks per field.  Per
            -- track: its loop length, hit count, clock position (read-modify-
            -- write of the track RAM), phase step, Roll offset, the Euclidean
            -- hit mask by accumulators, the steps since its last strike, and
            -- the record the line engine reads.
            -------------------------------------------------------------------
            q_ld <= '0';
            q_done <= '0';
            tk_we <= '0';
            if q_go = '1' then
                q_st <= to_unsigned(1, 6); q_busy <= '1';
                q_t <= (others => '0'); q_racc <= (others => '0'); q_hacc <= (others => '0');
                q_bph <= c_lfo(15 downto 8);
            else
                case to_integer(q_st) is
                    when 0 => null;
                    when 1 =>                         -- loop length; fill fraction; read the clock word
                        v_n := c_n;
                        if s_poly = '1' then
                            v_n := c_n - resize(q_t(1 downto 0), 6);
                            if v_n < 2 then v_n := to_unsigned(2, 6); end if;
                        end if;
                        q_nt <= v_n;
                        v_x := signed(resize(s_gp(12 downto 4), 11)) - signed(resize(f_ofs(q_t), 11));
                        if v_x < 0 then q_phi <= (others => '0');
                        elsif v_x > 256 then q_phi <= to_unsigned(256, 9);
                        else q_phi <= unsigned(v_x(8 downto 0)); end if;
                        tk_sa <= q_t & "100";
                        q_st <= q_st + 1;
                    when 2 =>                         -- density 64 .. 256 = 1/4 .. 1
                        v_p := resize(q_phi, 11) + shift_left(resize(q_phi, 11), 1);
                        q_dens <= resize(v_p(10 downto 2), 9) + 64;
                        q_lim <= q_nt & "00000000000";
                        q_st <= q_st + 1;
                    when 3 =>                         -- hits = loop x density
                        z_fa <= resize(q_nt, 24); z_fb <= resize(q_dens, 16); q_ld <= '1';
                        q_wc <= (others => '0');
                        q_st <= q_st + 1;
                    when 4 =>
                        q_wc <= q_wc + 1;
                        if q_wc = 20 then q_st <= q_st + 1; end if;
                    when 5 =>
                        v_k := resize(f_acc(15 downto 0) + 128, 16);
                        if v_k(13 downto 8) = 0 then q_kt <= to_unsigned(1, 6); else q_kt <= v_k(13 downto 8); end if;
                        -- clock: position + step (the RAM word has been out since state 4)
                        if s_vneg = '1' then q_s1 <= signed(resize(unsigned(tk_qf), 19)) - signed(resize(c_d, 19));
                        else q_s1 <= signed(resize(unsigned(tk_qf), 19)) + signed(resize(c_d, 19)); end if;
                        q_st <= q_st + 1;
                    when 6 =>                         -- wrap once
                        if q_s1 < 0 then q_s1 <= q_s1 + signed(resize(q_lim, 19));
                        elsif q_s1 >= signed(resize(q_lim, 19)) then q_s1 <= q_s1 - signed(resize(q_lim, 19)); end if;
                        q_st <= q_st + 1;
                    when 7 =>                         -- still out of the loop (Steps just shrank): restart; Same: all follow track 0
                        if q_s1 < 0 or q_s1 >= signed(resize(q_lim, 19)) then q_s1 <= (others => '0'); end if;
                        if s_poly = '0' and q_t /= 0 then q_s1 <= signed(resize(q_s0, 19)); end if;
                        q_st <= q_st + 1;
                    when 8 =>                         -- phase step = loop x 2^25 / width
                        if q_t = 0 then q_s0 <= unsigned(q_s1(15 downto 0)); end if;
                        z_fa <= resize(q_nt, 24); z_fb <= c_c25; q_ld <= '1';
                        q_wc <= (others => '0');
                        q_st <= q_st + 1;
                    when 9 =>
                        q_wc <= q_wc + 1;
                        if q_wc = 20 then q_st <= q_st + 1; end if;
                    when 10 =>                        -- Roll: half the step, the left edge is a quarter loop behind the clock
                        if s_roll = '1' then
                            q_inc <= f_acc(21 downto 1);
                            q_l <= q_s1 - signed(resize(q_nt & "000000000", 19));
                        else
                            q_inc <= f_acc(20 downto 0);
                            q_l <= (others => '0');
                        end if;
                        q_a <= (others => '0');
                        q_rc <= q_racc(13 downto 8);
                        q_j <= (others => '0');
                        q_msk <= (others => '0');
                        q_have <= '0';
                        q_st <= q_st + 1;
                    when 11 =>                        -- rotate: run the pattern accumulator ahead
                        if q_l < 0 then q_l <= q_l + signed(resize(q_lim, 19)); end if;
                        if q_rc = 0 then
                            q_st <= q_st + 1;
                        else
                            v_a := resize(q_a, 7) + resize(q_kt, 7);
                            if v_a >= resize(q_nt, 7) then v_a := v_a - resize(q_nt, 7); end if;
                            q_a <= v_a(5 downto 0);
                            q_rc <= q_rc - 1;
                        end if;
                    when 12 =>                        -- the Euclidean pattern: step j is a hit when acc < k
                        if q_j = q_nt then
                            q_st <= q_st + 1;
                        else
                            if q_a < q_kt then v_b := '1'; else v_b := '0'; end if;
                            q_msk(to_integer(q_j(4 downto 0))) <= v_b;
                            if v_b = '1' then
                                q_la <= q_j;
                                if q_j <= resize(unsigned(q_s1(15 downto 11)), 6) then q_le <= q_j; q_have <= '1'; end if;
                            end if;
                            v_a := resize(q_a, 7) + resize(q_kt, 7);
                            if v_a >= resize(q_nt, 7) then v_a := v_a - resize(q_nt, 7); end if;
                            q_a <= v_a(5 downto 0);
                            q_j <= q_j + 1;
                        end if;
                    when 13 =>                        -- steps since the last strike
                        if q_have = '1' then v_r := resize(unsigned(q_s1(15 downto 11)), 7) - resize(q_le, 7);
                        else v_r := resize(unsigned(q_s1(15 downto 11)), 7) + resize(q_nt, 7) - resize(q_la, 7); end if;
                        q_ra <= v_r(5 downto 0);
                        q_st <= q_st + 1;
                    when 14 =>                        -- accent level from the envelope ROM (long curve)
                        q_eva <= '0' & q_ra(4 downto 0) & unsigned(q_s1(10 downto 7));
                        -- rest level 5 .. 10 from the breath LFO
                        v_h := resize(f_tri7(q_bph), 8);
                        v_w := resize(v_h, 9) + shift_left(resize(v_h, 9), 1);
                        q_rest <= resize(v_w(8 downto 6), 4) + 5;
                        q_wc <= (others => '0');
                        q_st <= q_st + 1;
                    when 15 =>
                        q_wc <= q_wc + 1;
                        if q_wc = 4 then q_st <= q_st + 1; end if;
                    when 16 =>
                        if s_acc = '1' then q_ac <= unsigned(r_ev_q(15 downto 12)); else q_ac <= (others => '0'); end if;
                        q_st <= q_st + 1;
                    when 17 => tk_we <= '1'; tk_wa <= q_t & "000"; tk_wd <= q_msk(15 downto 0);  q_st <= q_st + 1;
                    when 18 => tk_we <= '1'; tk_wa <= q_t & "001"; tk_wd <= q_msk(31 downto 16); q_st <= q_st + 1;
                    when 19 => tk_we <= '1'; tk_wa <= q_t & "010"; tk_wd <= std_logic_vector(q_inc(15 downto 0)); q_st <= q_st + 1;
                    when 20 => tk_we <= '1'; tk_wa <= q_t & "011";
                               tk_wd <= '0' & std_logic_vector(q_rest) & std_logic_vector(q_nt) & std_logic_vector(q_inc(20 downto 16));
                               q_st <= q_st + 1;
                    when 21 => tk_we <= '1'; tk_wa <= q_t & "100"; tk_wd <= std_logic_vector(q_s1(15 downto 0)); q_st <= q_st + 1;
                    when 22 => tk_we <= '1'; tk_wa <= q_t & "101"; tk_wd <= std_logic_vector(q_l(15 downto 0));  q_st <= q_st + 1;
                    when 23 => tk_we <= '1'; tk_wa <= q_t & "110";
                               tk_wd <= x"00" & std_logic_vector(q_hacc(11 downto 8)) & std_logic_vector(q_ac);
                               q_st <= q_st + 1;
                    when 24 =>                        -- next track
                        q_racc <= q_racc + resize(unsigned(C_KFIX_5), 14);
                        q_hacc <= q_hacc + resize(c_spr, 14);
                        q_bph  <= q_bph + 40;
                        q_t <= q_t + 1;
                        if q_t = 11 then q_st <= to_unsigned(25, 6); else q_st <= to_unsigned(1, 6); end if;
                    when 25 => q_busy <= '0'; q_st <= q_st + 1;
                    when 26 => q_st <= q_st + 1;
                    when others => q_done <= '1'; q_st <= (others => '0');
                end case;
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Generated avid (parity-locked start 3/4 clocks after the source's avid
    -- rise, field-latched width)
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
    -- Source side: oscillator (2-D triangle lattice, drifting), camera luma
    -- through an 8-px box (resynced every line), line-buffer write of the
    -- 7-bit source level
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_i : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;  -- ot * 96
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- camera: (luma >> 2) - 16, zero off the source avid, 8-px running sum; the sum snaps to
            -- zero once the input and the 8 samples before it are all blank
            v_l := unsigned(data_in.y(9 downto 2));
            if data_in.avid = '1' and v_l > 16 then v_i := v_l - 16; else v_i := (others => '0'); end if;
            s_bx(0) <= v_i;
            for i in 1 to 7 loop s_bx(i) <= s_bx(i - 1); end loop;
            s_bxh <= s_bxh(6 downto 0) & data_in.avid;
            if s_bxh = "00000000" and data_in.avid = '0' then
                s_bsum <= (others => '0');
            else
                s_bsum <= s_bsum + resize(v_i, 11) - resize(s_bx(7), 11);
            end if;

            -- source level (camera 0 .. 59, Osc 0 .. 111), x K6 / 256 in its own stage, clipped at 255
            s_lb_we1 <= data_in.avid;
            s_lb_wa1 <= s_wcol;
            if s_src = '1' then s_sl <= s_osum(7 downto 1);
            else s_sl <= resize(s_bsum(10 downto 5), 7); end if;
            s_lb_we2 <= s_lb_we1; s_lb_wa2 <= s_lb_wa1;
            s_slp <= s_sl * c_cg;
            s_lb_we <= s_lb_we2; s_lb_wa <= s_lb_wa2;
            if s_slp(16) = '1' then s_lb_wd <= x"FF"; else s_lb_wd <= std_logic_vector(s_slp(15 downto 8)); end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone, after the
    -- track sequencer, for line 0); computes the NEXT line: its row, the
    -- vertical triangle, the hum bar, and the row's record from the track RAM
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_yf : unsigned(7 downto 0);
        variable v_d1, v_d2 : signed(8 downto 0);
        variable v_g  : unsigned(9 downto 0);
        variable v_t7 : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= q_done;

            if e_zk = '1' then
                e_r <= (others => '0');
                e_hacc <= c_humb;
                e_prow <= (others => '1');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_r <= e_r + resize(c_rinc, 22);
                e_hacc <= e_hacc + resize(c_hinc, 16);
                e_lc <= (others => '0');
            elsif e_lc /= 31 then
                e_lc <= e_lc + 1;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    n_row <= e_r(21 downto 18);
                    if e_r(21 downto 18) >= c_t then n_blank <= '1'; else n_blank <= '0'; end if;
                    if e_r(21 downto 18) /= e_prow then n_cap <= '1'; else n_cap <= '0'; end if;
                    e_prow <= e_r(21 downto 18);
                    v_yf := e_r(17 downto 10);
                    v_d1 := signed(resize(v_yf, 9)) - 12;
                    v_d2 := to_signed(244, 9) - signed(resize(v_yf, 9));
                    if v_d1 < v_d2 then e_dm <= v_d1; else e_dm <= v_d2; end if;
                    v_t7 := f_tri7(e_hacc(15 downto 8));
                    e_hum <= v_t7(6 downto 4);
                when 2 =>
                    if e_dm < 0 then n_ry <= (others => '0');
                    elsif e_dm > 64 then n_ry <= to_unsigned(32, 6);
                    else n_ry <= unsigned(e_dm(6 downto 1)); end if;
                    tk_la <= n_row & "000";
                when 3  => tk_la <= n_row & "001";
                when 4  => tk_la <= n_row & "010";
                when 5  => tk_la <= n_row & "011";
                when 6  => tk_la <= n_row & "100"; n_mask(15 downto 0) <= tk_qf;
                when 7  => tk_la <= n_row & "101"; n_mask(31 downto 16) <= tk_qf;
                when 8  => tk_la <= n_row & "110"; n_inc(15 downto 0) <= unsigned(tk_qf);
                when 9  =>
                    n_inc(20 downto 16) <= unsigned(tk_qf(4 downto 0));
                    n_nt <= unsigned(tk_qf(10 downto 5));
                    e_rest <= unsigned(tk_qf(14 downto 11));
                when 10 => n_s <= unsigned(tk_qf); n_ntm1 <= n_nt - 1;
                when 11 => n_l <= unsigned(tk_qf);
                when 12 =>
                    n_hue <= unsigned(tk_qf(7 downto 4));
                    e_ac  <= unsigned(tk_qf(3 downto 0));
                when 13 =>                    -- rest level in half levels: 2 x rest + hum - 4; accent bus x vertical triangle
                    n_resth <= shift_left(resize(e_rest, 6), 1) + resize(e_hum, 6) - 4;
                    v_g := e_ac * n_ry;
                    n_glow <= v_g(8 downto 5);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the step phase (25 fraction bits) and the step counter, wrapping
    -- at the row's loop length
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_s : unsigned(25 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_px <= (others => '0');
                b_fr <= n_l(10 downto 0) & "00000000000000";
                b_c  <= n_l(15 downto 11);
                l_blank <= n_blank; l_cap <= n_cap; l_ry <= n_ry; l_mask <= n_mask; l_inc <= n_inc;
                l_nt <= n_nt; l_ntm1 <= n_ntm1; l_si <= n_s(15 downto 11); l_sf8 <= n_s(10 downto 3);
                l_hue <= n_hue; l_resth <= n_resth; l_glow <= n_glow;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                v_s := resize(b_fr, 26) + resize(l_inc, 26);
                b_fr <= v_s(24 downto 0);
                if v_s(25) = '1' then
                    if b_c = l_ntm1(4 downto 0) then b_c <= (others => '0'); else b_c <= b_c + 1; end if;
                end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_e  : signed(6 downto 0);
        variable v_m  : unsigned(7 downto 0);
        variable v_s  : signed(7 downto 0);
        variable v_c  : unsigned(5 downto 0);
        variable v_v  : unsigned(8 downto 0);
        variable v_p  : unsigned(12 downto 0);
        variable v_d  : signed(9 downto 0);
        variable v_f  : unsigned(8 downto 0);
        variable v_g  : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: hit bit of the step, step distance to the clock
            p2_hit <= l_mask(to_integer(b_c));
            p2_e   <= signed(resize(b_c, 6)) - signed(resize(l_si, 6));
            p2_fr8 <= b_fr(24 downto 17);
            p2_c   <= b_c;
            if b_px < 8 then p2_f8 <= '1'; else p2_f8 <= '0'; end if;
            -- 3: wrap the distance; pad window; source level from the line buffer
            if p2_e < 0 then v_e := resize(p2_e, 7) + signed(resize(l_nt, 7)); else v_e := resize(p2_e, 7); end if;
            p3_e   <= unsigned(v_e(4 downto 0));
            p3_d2  <= signed(resize(c_g9, 10)) - signed(resize(p2_fr8, 10));
            if p2_fr8 >= l_sf8 then p3_ge <= '1'; else p3_ge <= '0'; end if;
            p3_fr8 <= p2_fr8; p3_hit <= p2_hit; p3_c <= p2_c;
            p3_lm  <= unsigned(s_lb_q);
            p3_f8  <= p2_f8;
            -- 4: age of the step (steps since the clock entered it); past the playhead; horizontal triangle
            if p3_e = 0 then p4_age <= (others => '0'); else p4_age <= resize(l_nt - resize(p3_e, 6), 5); end if;
            if (p3_e = 0 and p3_ge = '1') or p3_e = 1 then p4_past <= '1'; else p4_past <= '0'; end if;
            if p3_d2 < 0 then v_m := (others => '0');
            elsif p3_d2 < signed(resize(p3_fr8, 10)) then v_m := unsigned(p3_d2(7 downto 0));
            else v_m := p3_fr8; end if;
            if v_m(7 downto 1) > 32 then p4_rx <= to_unsigned(32, 6); else p4_rx <= v_m(6 downto 1); end if;
            p4_hit <= p3_hit; p4_lm <= p3_lm;
            -- sample-and-hold: on the first line of a row write the source level per step (the second
            -- quarter of each step, and the line's first pixels for a step cut by the left edge)
            if l_cap = '1' and s_gav_sr(1) = '1' and (p3_fr8(7 downto 6) = "01" or p3_f8 = '1') then h_we <= '1'; else h_we <= '0'; end if;
            h_wa <= p3_c; h_wd <= std_logic_vector(p3_lm);
            h_ra <= p3_c;
            -- 5: envelope ROM address; pad shape = H triangle + V triangle, clipped; playhead one-shot
            if q_busy = '1' then r_ev_a <= q_eva;
            else r_ev_a <= s_short & p4_age & l_sf8(7 downto 4); end if;
            v_s := signed(resize(p4_rx, 8)) + signed(resize(l_ry, 8)) - 24;
            if v_s < 0 then p5_roof <= (others => '0');
            elsif v_s > 32 then p5_roof <= to_unsigned(32, 6);
            else p5_roof <= unsigned(v_s(5 downto 0)); end if;
            if s_gav_sr(2) = '0' then
                p5_pastq <= '1'; p5_ph <= '0';
            else
                p5_pastq <= p4_past;
                if p4_past = '1' and p5_pastq = '0' then
                    p5_ph <= '1'; p5_phc <= (others => '0');
                elsif p5_ph = '1' then
                    v_c := resize(p5_phc, 6) + resize(s_xs, 6);
                    if v_c >= 16 then p5_ph <= '0'; else p5_phc <= v_c(4 downto 0); end if;
                end if;
            end if;
            p5_hit <= p4_hit; p5_lm <= p4_lm;
            -- 6: envelope ROM data (EBR); held source level, fabric copy
            p6_roof <= p5_roof; p6_hit <= p5_hit; p6_ph <= p5_ph; p6_lm <= p5_lm;
            p6_lh <= unsigned(h_q);
            -- 7: fabric copy of the envelope; the source level: Osc tracks, Video holds per pad
            p7_env <= unsigned(r_ev_q(6 downto 0));
            p7_cw  <= unsigned(r_ev_q(11 downto 7));
            if s_src = '1' then p7_lum <= p6_lm;
            elsif l_cap = '1' then p7_lum <= (others => '0');
            else p7_lum <= p6_lh; end if;
            p7_roof <= p6_roof; p7_hit <= p6_hit; p7_ph <= p6_ph;
            -- 8: camera window gain x source level; rest + envelope
            p8_cam  <= p7_cw * p7_lum;
            p8_base <= resize(l_resth, 8) + resize(p7_env, 8);
            p8_roof <= p7_roof; p8_hit <= p7_hit; p8_ph <= p7_ph;
            -- 9: level in half levels (an empty step is a faint ghost)
            v_v := resize(p8_base, 9) + resize(p8_cam(12 downto 5), 9);
            if p8_hit = '0' then p9_v <= to_unsigned(3, 7);
            elsif v_v > 127 then p9_v <= to_unsigned(127, 7);
            else p9_v <= v_v(6 downto 0); end if;
            p9_roof <= p8_roof; p9_ph <= p8_ph;
            -- 10: level x shape
            v_p := p9_v * p9_roof;
            p10_f <= v_p(11 downto 6);
            p10_ph <= p9_ph;
            -- 11: slew limiter along the scan (1/8 levels)
            v_d := signed(resize(p11_l, 10)) - signed(resize(c_dec, 10));
            if v_d < 0 then v_d := (others => '0'); end if;
            v_f := p10_f & "000";
            if s_gav_sr(8) = '0' then p11_l <= (others => '0');
            elsif v_f > unsigned(v_d(8 downto 0)) then p11_l <= v_f;
            else p11_l <= unsigned(v_d(8 downto 0)); end if;
            p11_ph <= p10_ph;
            -- 12: + accent bus; playhead; colouriser address
            v_g := resize(p11_l(8 downto 3), 7) + resize(l_glow, 7);
            if l_blank = '1' then r_pl_a <= l_hue & "000000";
            elsif p11_ph = '1' then r_pl_a <= l_hue & to_unsigned(37, 6);
            elsif v_g > 63 then r_pl_a <= l_hue & to_unsigned(63, 6);
            else r_pl_a <= l_hue & v_g(5 downto 0); end if;
            -- 13: colouriser ROM (EBR); 14: fabric copy
            p14_q <= r_pl_q;
            -- 15: compose; black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= c_ky; s_u15 <= c_ku; s_v15 <= c_kv;
            else
                s_y15 <= unsigned(p14_q(31 downto 22)); s_u15 <= unsigned(p14_q(21 downto 12)); s_v15 <= unsigned(p14_q(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the analog softness, and the HD HDMI bar
    -- fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to the palette black (the off-avid input of both boxes) once
            -- 8 blank samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 16 sums, stage 17 output
            s_cay(0) <= s_y15; s_cau(0) <= s_u15; s_cav(0) <= s_v15;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= c_ky & "000"; s_bau <= c_ku & "000"; s_bav <= c_kv & "000";
            else
                s_bay <= s_bay + s_y15 - s_cay(7);
                s_bau <= s_bau + s_u15 - s_cau(7);
                s_bav <= s_bav + s_v15 - s_cav(7);
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: palette black off the generated avid (what box A gives there anyway)
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= c_ky; s_u17 <= c_ku; s_v17 <= c_kv;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            -- box B: stage 18 sums, stage 19 output
            s_cby(0) <= s_y17; s_cbu(0) <= s_u17; s_cbv(0) <= s_v17;
            for i in 1 to 7 loop s_cby(i) <= s_cby(i - 1); s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bby <= c_ky & "000"; s_bbu <= c_ku & "000"; s_bbv <= c_kv & "000";
            else
                s_bby <= s_bby + s_y17 - s_cby(7);
                s_bbu <= s_bbu + s_u17 - s_cbu(7);
                s_bbv <= s_bbv + s_v17 - s_cbv(7);
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y19 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u19 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v19 <= v_sc(13 downto 4);
            -- 20: luma 13/16
            if s_y19 > 64 then v_dl := s_y19 - 64; else v_dl := (others => '0'); end if;
            s_y20 <= s_y19 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u20 <= s_u19; s_v20 <= s_v19;
            -- 21: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y21 <= s_y20; s_u21 <= s_u20; s_v21 <= s_v20;
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y21);
    data_out.u       <= std_logic_vector(s_u21);
    data_out.v       <= std_logic_vector(s_v21);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture euclid;
