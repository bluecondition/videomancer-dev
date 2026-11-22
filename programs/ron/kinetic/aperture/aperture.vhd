-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Aperture v0.1.0 -- Kinetic (wave 2).
--
-- A wall of 8-blade iris diaphragms (the camera iris; Jean Nouvel's mechanical-iris facade of the Institut du
-- Monde Arabe).  Each cell is an iris: a regular octagonal hole and, around it, eight flat blades that
-- partition the rest of the housing like a pinwheel, each blade one lit metal tone, with dark seams between
-- the blades.  A travelling wave opens and closes the irises in turn.
--
--   element  the iris of one cell (pitch P); state = its aperture inradius a
--   theta    = Spread * index - phase;  index: Linear = i + j, Radial = octagonal distance (|i|, |j|)
--   f        = Open + Depth * wave(theta) + source   (clamped 0 .. 1; a = f * amax from a per-field table)
--   blades   cell-centred (p, q); four linear forms L0 = p, L1 = (p + q) 0.7071, L2 = q, L3 = (q - p) 0.7071;
--            edge k: e_k = n_k . (p, q) - a (n_k at k * 45 degrees);  hole = every e_k < 0;  blade k owns the
--            pixel where e_k >= 0 and e_(k+1) < 0 (cyclic): bounded by its own edge line and the next one's
--   edges    seam = distance to the blade-blade boundaries (min(e_k, -e_(k+1)) or -e_(k+1)), hole edge =
--            max |L| - a; both anti-aliased over one (1080p) or two virtual px in eighths
--   housing  rounded square (corner radius from Shape) with a rim ring: r^2 = max(|p| - c, 0)^2 +
--            max(|q| - c, 0)^2 against rho^2 and (rho - rim)^2; the corners and the frame in the ground colour
--   colour   I = T1[blade, seam coverage] (per-field table: the blade's tone toward the seam colour);
--            C2 = I (1 - hole cov) + X * shade * hole cov  (X = glass colour B, or the camera; shade = the
--            recessed inner shadow); out = C2 + (T2[ground cov] - C2) * rim cov  (T2: accent toward ground)
--
--   K1  Scale    cell pitch 150 .. 1120 virtual lines (exponential, glided)
--   K2  Shape    housing: square shutters filling the cell .. round portholes with a rim and ground between
--   K3  Depth    wave depth 0 .. a full aperture (square law; Snap x1.5)
--   K4  Rate     bipolar, centre = still; 60 s per cycle .. 2 cycles per second
--   K5  Colour   the 16 shared palettes (ground, A = blade metal, B = glass, accent = rim)
--   K6  Camera   camera (and Osc) strength: the cell's held luma opens its iris (bright = open), default 100 %
--   S7  Wave     Smooth / Snap
--   S8  Field    Linear / Radial
--   S9  Hole     Glass (palette B) / Video (the camera through the holes)
--   S10 Pack     Square / Stagger (alternate rows offset half a cell)
--   S11 Source   Video / Osc
--   P12 OPEN     the mean aperture: 0 = shut (star seams) .. 100 % = wide open (glided)
--
-- The file is in lifted sections (the Kinetic chassis, from louver.vhd via pinwheel.vhd):
--   [A] timing ID -> virtual-1080 steps, vblank zone, width, knob glides, per-field sequencer (serial multiplier)
--   [B] generated avid
--   [C] source field: Osc lattice, luma smear, per-cell hold, video delay ring
--   [D] wave engine: a serial per-cell engine computes the next cell's aperture ahead of the beam
--       (and the next line's first two cells in blanking): theta -> wave ROM -> Depth x wave -> state -> aperture
--   [E] the look: iris geometry, housing, coverage, colour
--   [F] output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1] with resync, luma 13/16, blanking gate
-- One latency (C_LAT = 27) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.aperture_pkg.all;
use work.all;

architecture aperture of program_top is
    constant C_LAT      : integer := 27;      -- = processing_delay_clks
    constant C_SIM_SNAP : boolean := false;   -- sim-only copies set this: knob glides jump straight to the knob
    -- serial multiplier slots in use: 0 pitch, 2 shape x pitch, 3 depth^2, 4 frame, 5 rim, 6 shape x half-size,
    -- 7 camera gain, 8 rho^2, 9 (rho - rim)^2, 10 Osc centre, 12 rate exponent, 14 rate
    constant C_SLOTS : std_logic_vector(15 downto 0) := "0101011111111101";

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
    -- the Osc lattice's y step: y step x 121 / 256
    function f_oys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(537, 10);
            when 4          => return to_unsigned(272, 10);
            when 8          => return to_unsigned(453, 10);
            when 12         => return to_unsigned(226, 10);
            when 5 | 6 | 14 => return to_unsigned(181, 10);
            when 1 | 2 | 10 => return to_unsigned(242, 10);
            when others     => return to_unsigned(121, 10);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- gated operands: every multiply in this file is a sum of these (one adder per two operand bits per stage)
    function f_gu(x : unsigned; b : std_logic; w : natural) return unsigned is
        variable r : unsigned(w - 1 downto 0) := (others => '0');
    begin
        if b = '1' then r := resize(x, w); end if;
        return r;
    end function;
    function f_gs(x : signed; b : std_logic; w : natural) return signed is
        variable r : signed(w - 1 downto 0) := (others => '0');
    begin
        if b = '1' then r := resize(x, w); end if;
        return r;
    end function;
    -- coverage in eighths from a pre-biased distance and a 0 / 1 shift: clamp(t >> s, 0, 8)
    function f_cov(t : signed; s : std_logic) return unsigned is
        variable v : signed(t'length - 1 downto 0);
    begin
        if s = '1' then v := shift_right(t, 1); else v := t; end if;
        if v < 0 then return to_unsigned(0, 4);
        elsif v > 8 then return to_unsigned(8, 4);
        else return unsigned(v(3 downto 0)); end if;
    end function;
    -- the housing coverage: v < 0 -> 0, v >= top (= 8 << r) -> 8, else bits r + 2 .. r of v
    function f_hc(v : signed(13 downto 0); r : unsigned(3 downto 0); top : unsigned(12 downto 0)) return unsigned is
        variable q : unsigned(3 downto 0) := (others => '0');
        variable u : unsigned(12 downto 0);
    begin
        u := unsigned(v(12 downto 0));
        if v(13) = '1' then
            q := (others => '0');
        elsif u >= top then
            q := to_unsigned(8, 4);
        else
            case to_integer(r) is
                when 0      => q := '0' & u(2 downto 0);
                when 1      => q := '0' & u(3 downto 1);
                when 2      => q := '0' & u(4 downto 2);
                when 3      => q := '0' & u(5 downto 3);
                when 4      => q := '0' & u(6 downto 4);
                when 5      => q := '0' & u(7 downto 5);
                when 6      => q := '0' & u(8 downto 6);
                when 7      => q := '0' & u(9 downto 7);
                when 8      => q := '0' & u(10 downto 8);
                when others => q := '0' & u(11 downto 9);
            end case;
        end if;
        return q;
    end function;
    function f_max4(a, b : unsigned(3 downto 0)) return unsigned is
    begin
        if a > b then return a; else return b; end if;
    end function;

    attribute no_rw_check : boolean;

    -- ROMs (one read port each, registered address and data, then a fabric register)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_w_a  : unsigned(7 downto 0) := (others => '0');
    signal r_w_q  : unsigned(15 downto 0) := (others => '0');
    signal r_t0_a : unsigned(7 downto 0) := (others => '0');
    signal r_t0_q : unsigned(15 downto 0) := (others => '0');
    signal r_pl_a : unsigned(8 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    -- [C] source hold: one value per cell column, two banks by cell-row parity (read the row's bank, write the
    -- next row's), written on the last line of each cell row
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal sl : t_r16 := (others => (others => '0'));
    attribute no_rw_check of sl : signal is true;
    signal sl_ra, sl_wa : unsigned(7 downto 0) := (others => '0');
    signal sl_q, sl_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal sl_we : std_logic := '0';
    -- [C] camera gain by smeared sample, refilled every field: (s kq) >> 7 (Osc: 2 (s kq >> 7) - centre)
    signal gt : t_r16 := (others => (others => '0'));
    attribute no_rw_check of gt : signal is true;
    signal gt_ra, gt_wa : unsigned(7 downto 0) := (others => '0');
    signal gt_q, gt_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal gt_we, gt_run : std_logic := '0';
    signal gt_acc : unsigned(16 downto 0) := (others => '0');
    signal gt_k   : unsigned(7 downto 0) := (others => '0');
    -- [D] aperture by state: a = f * amax / 256 (Q3 virtual px), refilled every field
    signal at : t_r16 := (others => (others => '0'));
    attribute no_rw_check of at : signal is true;
    signal at_ra, at_wa : unsigned(7 downto 0) := (others => '0');
    signal at_q, at_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal at_we, at_run : std_logic := '0';
    signal at_acc : unsigned(20 downto 0) := (others => '0');
    signal at_k   : unsigned(7 downto 0) := (others => '0');
    -- delay rings (write at rc + d, read at rc, RAM register, fabric register: d + 2 stages)
    type t_r32 is array (0 to 255) of std_logic_vector(31 downto 0);
    signal vr : t_r32 := (others => (others => '0'));          -- the camera (or Osc colour): input -> stage 15
    attribute no_rw_check of vr : signal is true;
    signal vr_d, vr_q, vr_f : std_logic_vector(31 downto 0) := (others => '0');
    signal cr : t_r16 := (others => (others => '0'));          -- chroma 8 back for the running 8-sums
    attribute no_rw_check of cr : signal is true;
    signal cr_q, cr_f : std_logic_vector(15 downto 0) := (others => '0');
    -- [E] T1: blade tone k toward the seam colour by the seam coverage cs (0 .. 8): address k & cs;
    -- {Y - 64 (10 bits), U, V centred 8-bit}
    type t_t1 is array (0 to 127) of std_logic_vector(31 downto 0);
    signal tt : t_t1 := (others => (others => '0'));
    attribute no_rw_check of tt : signal is true;
    signal tt_ra, tt_wa : unsigned(6 downto 0) := (others => '0');
    signal tt_q : std_logic_vector(31 downto 0) := (others => '0');
    signal tt_we : std_logic := '0';
    -- T2: 0 .. 8 the rim colour (accent) toward the ground by the outer coverage; 16 .. 29 the glass shaded
    type t_t2 is array (0 to 255) of std_logic_vector(31 downto 0);
    signal o2 : t_t2 := (others => (others => '0'));
    attribute no_rw_check of o2 : signal is true;
    signal o2_ra, o2_wa : unsigned(7 downto 0) := (others => '0');
    signal o2_q : std_logic_vector(31 downto 0) := (others => '0');
    signal o2_we : std_logic := '0';
    type t_r4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal sr : t_r4 := (others => (others => '1'));           -- syncs: C_LAT
    attribute no_rw_check of sr : signal is true;
    signal sr_q, sr_f : std_logic_vector(3 downto 0) := (others => '1');
    signal rc   : unsigned(7 downto 0) := (others => '0');

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
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
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(240 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(460 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_k2 : unsigned(9 downto 0) := to_unsigned(260, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_vh, s_stag, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(7, 4);
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    -- K6 Camera: knob (deadband +-4), glided gain in Q3
    signal s_kr  : unsigned(9 downto 0) := to_unsigned(1023, 10);
    signal s_kt  : unsigned(9 downto 0) := to_unsigned(1023, 10);
    signal s_kgd : signed(13 downto 0) := (others => '0');
    signal s_kg  : unsigned(12 downto 0) := to_unsigned(1023 * 8, 13);
    signal c_kq  : unsigned(7 downto 0) := to_unsigned(170, 8);       -- camera gain: 171 K6 / 1024 (/ 128)
    signal c_off : unsigned(8 downto 0) := to_unsigned(294, 9);       -- Osc centre: 222 kq / 128

    -- field serial multiplier (24 iterations), serial shifter, sequencer scratch
    signal f_a, f_acc, z_fa : unsigned(31 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_e   : unsigned(10 downto 0) := (others => '0');
    signal z_sh  : unsigned(2 downto 0) := (others => '0');
    signal z_shn : unsigned(2 downto 0) := (others => '0');
    signal z_sht : std_logic := '0';
    signal z_mt  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_loc : signed(12 downto 0) := (others => '0');
    signal z_kp  : std_logic := '0';
    signal z_k   : signed(4 downto 0) := (others => '0');
    signal z_nrun, z_first : std_logic := '0';
    signal z_mp, z_fr, z_tr, z_hh, z_hm, z_th : unsigned(11 downto 0) := (others => '0');

    -- per-field constants
    signal c_pw   : unsigned(19 downto 0) := to_unsigned(481 * 128, 20);    -- pitch in half virtual px, Q7
    signal c_p2   : unsigned(11 downto 0);                               -- pitch, half virtual px
    signal c_p2h  : unsigned(10 downto 0);                               -- to the cell centre
    signal c_sh   : unsigned(1 downto 0) := "00";                        -- the pitch's octave
    signal c_d    : unsigned(8 downto 0) := (others => '0');             -- depth, Q8 of a full aperture
    signal c_d0   : unsigned(7 downto 0) := (others => '0');
    signal c_ymp  : signed(20 downto 0) := (others => '0');              -- y step - pitch (Q7 half px)
    signal c_x0, c_x1 : unsigned(11 downto 0) := (others => '0');        -- pixel 0: offset in its cell (even / odd rows)
    signal c_ci0, c_ci1 : signed(4 downto 0) := (others => '0');         --          its cell index
    signal c_ph   : unsigned(25 downto 0) := (others => '0');            -- wave phase, Q26 cycles
    signal c_dph  : unsigned(21 downto 0) := (others => '0');
    signal c_open : unsigned(7 downto 0) := to_unsigned(115, 8);         -- Open, Q8
    signal c_hs4  : signed(9 downto 0) := to_signed(20, 10);             -- seam: (4 << rs) + half-width (Q3)
    signal c_c4   : signed(14 downto 0) := to_signed(8, 15);             -- hole: 4 << rs
    signal c_rs   : std_logic := '0';                                    -- AA ramp 1 (1080p) / 2 virtual px
    signal c_sw   : unsigned(1 downto 0) := "00";                        -- inner-shadow step 2^(3 + sw) Q3
    signal c_rho, c_rhoi, c_c2 : unsigned(11 downto 0) := (others => '0');   -- housing, half px
    signal c_rsq, c_risq : unsigned(23 downto 0) := (others => '0');
    signal c_amax : unsigned(12 downto 0) := (others => '0');            -- largest aperture inradius, Q3
    signal c_ho4, c_hi4 : signed(14 downto 0) := (others => '0');         -- (c + rho) and (c + rho - rim) in Q3, less 4 << rs
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u8  is array (1 to 2) of unsigned(7 downto 0);
    type t_s8  is array (1 to 2) of signed(7 downto 0);
    -- palette: the ground (raw) for the blanking and the chroma filter's resync
    signal c_gy   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal c_gc   : t_u8 := (others => to_unsigned(128, 8));

    -- table fill (palette reads, T2, T1): one accumulator and one step register
    signal f_st  : unsigned(3 downto 0) := (others => '0');
    signal f_cnt : unsigned(3 downto 0) := (others => '0');
    signal f_k   : unsigned(2 downto 0) := (others => '0');
    signal f_wt  : unsigned(1 downto 0) := (others => '0');
    signal f_ay, f_dy : signed(16 downto 0) := (others => '0');
    type t_s13 is array (1 to 2) of signed(12 downto 0);
    signal f_ac, f_dc : t_s13 := (others => (others => '0'));
    signal f_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal f_d1, f_d2, f_d3 : unsigned(4 downto 0) := (others => '0');
    signal f_v1, f_v2, f_v3 : std_logic := '0';

    -- [B] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';
    signal g_st1, g_st2, g_st3 : std_logic := '0';

    -- [C] source
    signal s_ox, s_oy : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_sv  : unsigned(7 downto 0) := (others => '0');
    signal s_sav, s_savd, s_srow, s_shx, s_wbk : std_logic := '0';
    signal s_y1 : unsigned(13 downto 0) := (others => '0');
    signal v_iy, v_iu, v_iv : unsigned(9 downto 0) := (others => '0');
    -- the held value: the camera gain table's output
    signal s_hv : signed(11 downto 0) := (others => '0');

    -- [D] wave engine: line registers (the NEXT line's start values), pixel accumulators (stage 1), the per-cell engine
    signal ls_cj  : signed(4 downto 0) := (others => '0');
    signal ls_y   : unsigned(18 downto 0) := (others => '0');
    signal ls_n1  : unsigned(18 downto 0) := (others => '0');
    signal ls_n2  : signed(20 downto 0) := (others => '0');
    signal ls_q8  : signed(14 downto 0) := (others => '0');
    signal ls_aq  : unsigned(13 downto 0) := (others => '0');
    signal ls_jp  : std_logic := '0';
    -- per-line housing (computed for the next line): W = c + isqrt(max(rho^2 - u^2, 0)) (Q3), u = max(|q| - c, 0),
    -- for the outer and the rim edge; the vertical coverages; the ramp shift from |W(next) - W(this)|
    signal h_st   : unsigned(5 downto 0) := (others => '0');
    signal h_z    : std_logic := '0';
    signal h_a, h_sq : unsigned(25 downto 0) := (others => '0');
    signal h_b    : unsigned(12 downto 0) := (others => '0');
    signal h_x, h_r, h_o : unsigned(24 downto 0) := (others => '0');
    signal h_d    : signed(14 downto 0) := (others => '0');
    signal ls_wo, ls_wi, l_wo : unsigned(13 downto 0) := (others => '0');
    signal ls_wbo, ls_wbi, l_wbo, l_wbi : signed(13 downto 0) := (others => '0');
    signal ls_top, l_top : unsigned(12 downto 0) := (others => '1');
    signal ls_rr, l_rr : unsigned(3 downto 0) := (others => '0');
    signal ls_cvo, ls_cvi, l_cvo, l_cvi : unsigned(3 downto 0) := (others => '0');
    signal l_cj   : signed(4 downto 0) := (others => '0');
    signal l_odd  : std_logic := '0';
    signal l_q8   : signed(14 downto 0) := (others => '0');
    signal l_aq   : unsigned(13 downto 0) := (others => '0');
    signal l_jp, l_fw : std_logic := '0';
    signal a_ci   : signed(4 downto 0) := (others => '0');
    signal a_x    : unsigned(11 downto 0) := (others => '0');
    signal a_nw, a_hx : std_logic := '0';
    signal a_cur, a_prev : unsigned(12 downto 0) := (others => '0');
    signal kc     : unsigned(6 downto 0) := (others => '0');
    signal s_nw, s_hx : std_logic_vector(2 to 9) := (others => '0');
    signal kw8, kw9, kw10 : unsigned(7 downto 0) := (others => '0');
    -- engine: one job = one cell's aperture in 21 clocks (a next cell at each wrap; the next line's cells 0, 1
    -- in blanking); one small operation per clock
    signal e_gonx, e_gol : std_logic := '0';
    signal e_nci  : signed(4 downto 0) := (others => '0');
    signal e_ncol : unsigned(6 downto 0) := (others => '0');
    signal e_st   : unsigned(4 downto 0) := (others => '0');
    signal e_job  : std_logic_vector(1 downto 0) := "00";
    signal e_ci, e_cj : signed(4 downto 0) := (others => '0');
    signal e_od   : std_logic := '0';
    signal e_col  : unsigned(6 downto 0) := (others => '0');
    signal e_bk, e_neg : std_logic := '0';
    signal e_th   : unsigned(13 downto 0) := (others => '0');
    signal e_ma, e_acc : unsigned(18 downto 0) := (others => '0');
    signal e_mb   : unsigned(8 downto 0) := (others => '0');
    signal e_src  : signed(11 downto 0) := (others => '0');
    signal e_f    : signed(12 downto 0) := (others => '0');
    signal a_nx, a_l0, a_l1 : unsigned(12 downto 0) := (others => '0');

    -- [E] the look
    signal q2_p, q3_p : signed(13 downto 0) := (others => '0');
    signal q3_sp, q3_sm : signed(14 downto 0) := (others => '0');
    signal q4_asp, q4_asm : unsigned(13 downto 0) := (others => '0');
    signal q4_ap, q5_ap, q6_ap : unsigned(12 downto 0) := (others => '0');
    signal q4_ng, q5_ng, q6_ng : std_logic_vector(2 downto 0) := (others => '0');   -- sign of L3, L1, L0 (p)
    signal q5_p1, q5_m1 : unsigned(13 downto 0) := (others => '0');
    signal q5_p2, q5_m2 : unsigned(9 downto 0) := (others => '0');
    signal q6_l1m, q6_l3m : unsigned(12 downto 0) := (others => '0');
    type t_m4 is array (0 to 3) of unsigned(12 downto 0);
    signal q7_m   : t_m4 := (others => (others => '0'));
    signal q7_s, q7_b : std_logic_vector(7 downto 0) := (others => '0');
    signal q7_ng  : std_logic_vector(3 downto 0) := (others => '0');
    signal q7_m01, q7_m23 : unsigned(12 downto 0) := (others => '0');
    signal q7_sel01, q7_sel23 : std_logic := '0';
    signal q8_sa, q8_sb, q8_r8 : unsigned(12 downto 0) := (others => '0');
    signal q8_bn, q8_sp, q8_hole, q9_sp, q9_hole : std_logic := '0';
    type t_k3 is array (natural range <>) of unsigned(2 downto 0);
    signal q_k : t_k3(8 to 13) := (others => (others => '0'));
    signal q9_d1, q9_d2 : unsigned(7 downto 0) := (others => '0');
    signal q9_nd  : signed(14 downto 0) := (others => '0');
    signal q10_ch, q11_ch, q12_ch : unsigned(3 downto 0) := (others => '0');
    signal q10_ds : unsigned(7 downto 0) := (others => '0');
    signal q11_t  : signed(9 downto 0) := (others => '0');
    signal q12_cs, q13_cs : unsigned(3 downto 0) := (others => '0');
    signal q13_a2 : unsigned(3 downto 0) := (others => '0');
    type t_c4 is array (natural range <>) of unsigned(3 downto 0);
    signal q_c   : t_c4(13 to 17) := (others => (others => '0'));
    signal q_rim : std_logic_vector(13 to 15) := (others => '0');
    -- housing
    signal q8_vo, q8_vi : signed(13 downto 0) := (others => '0');
    signal q_co, q_cr : t_c4(9 to 12) := (others => (others => '0'));
    -- colour
    signal q16_iy, q17_iy : unsigned(9 downto 0) := (others => '0');
    signal q16_ic, q17_ic : t_s8 := (others => (others => '0'));
    signal q16_zy : unsigned(9 downto 0) := (others => '0');
    signal q16_zc : t_s8 := (others => (others => '0'));
    signal q17_dy : signed(10 downto 0) := (others => '0');
    type t_s9 is array (1 to 2) of signed(8 downto 0);
    signal q17_dc : t_s9 := (others => (others => '0'));
    signal q18_ty0, q18_ty1 : signed(13 downto 0) := (others => '0');
    type t_s12 is array (1 to 2) of signed(11 downto 0);
    signal q18_tc0, q18_tc1 : t_s12 := (others => (others => '0'));
    signal q19_y  : signed(13 downto 0) := (others => '0');
    signal q19_c  : t_s12 := (others => (others => '0'));
    signal q20_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q20_c  : t_u8 := (others => to_unsigned(128, 8));

    -- [F] output conditioning
    signal s_ysr  : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv, s_bud, s_bvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y31, s_y32, s_y33 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u31, s_v31, s_u32, s_v32, s_u33, s_v33 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    c_p2  <= c_pw(18 downto 7);
    c_p2h <= c_pw(18 downto 8);

    ---------------------------------------------------------------------------
    -- Block RAM: ROMs, hold, tables, delay rings (one write statement each;
    -- none ever reads the address it is writing while the picture is drawn)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_AP_EXP2(to_integer(r_ex_a));
            r_w_q  <= C_AP_SIN(to_integer(r_w_a));
            r_pl_q <= C_AP_PAL(to_integer(r_pl_a));
            r_t0_q <= C_AP_TH0(to_integer(r_t0_a));
        end if;
    end process p_rom;

    p_sl : process(clk)
    begin
        if rising_edge(clk) then
            sl_q <= sl(to_integer(sl_ra));
            if sl_we = '1' then sl(to_integer(sl_wa)) <= sl_wd; end if;
        end if;
    end process p_sl;

    p_at : process(clk)
    begin
        if rising_edge(clk) then
            at_q <= at(to_integer(at_ra));
            if at_we = '1' then at(to_integer(at_wa)) <= at_wd; end if;
            -- fill (zone, zc 400 .. 656): one accumulator, entry f = f * amax / 256
            at_we <= '0';
            if s_zone = '1' and s_zc = 400 then
                at_run <= '1'; at_k <= (others => '0'); at_acc <= (others => '0');
            elsif at_run = '1' then
                at_we <= '1'; at_wa <= at_k; at_wd <= "000" & std_logic_vector(at_acc(20 downto 8));
                at_acc <= at_acc + c_amax;
                at_k <= at_k + 1;
                if at_k = 255 then at_run <= '0'; end if;
            end if;
        end if;
    end process p_at;

    p_tt : process(clk)
    begin
        if rising_edge(clk) then
            tt_q <= tt(to_integer(tt_ra));
            if tt_we = '1' then tt(to_integer(tt_wa)) <= f_wd; end if;
        end if;
    end process p_tt;

    p_o2 : process(clk)
    begin
        if rising_edge(clk) then
            o2_q <= o2(to_integer(o2_ra));
            if o2_we = '1' then o2(to_integer(o2_wa)) <= f_wd; end if;
        end if;
    end process p_o2;

    p_gt : process(clk)
    begin
        if rising_edge(clk) then
            gt_q <= gt(to_integer(gt_ra));
            if gt_we = '1' then gt(to_integer(gt_wa)) <= gt_wd; end if;
            -- fill (zone, zc 400 .. 656): entry s = s kq / 128 (camera) or 2 (s kq / 128) - 222 kq / 128 (Osc)
            gt_we <= '0';
            if s_zone = '1' and s_zc = 400 then
                gt_run <= '1'; gt_k <= (others => '0'); gt_acc <= (others => '0');
            elsif gt_run = '1' then
                gt_we <= '1'; gt_wa <= gt_k;
                if s_src = '1' then
                    gt_wd <= "0000" & std_logic_vector(signed(resize(gt_acc(16 downto 7) & '0', 12)) - signed(resize(c_off, 12)));
                else
                    gt_wd <= "0000" & std_logic_vector(resize(gt_acc(16 downto 7), 12));
                end if;
                gt_acc <= gt_acc + c_kq;
                gt_k <= gt_k + 1;
                if gt_k = 255 then gt_run <= '0'; end if;
            end if;
        end if;
    end process p_gt;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            vr(to_integer(rc + 15)) <= vr_d;
            vr_q <= vr(to_integer(rc));
            vr_f <= vr_q;
            cr(to_integer(rc + 6)) <= std_logic_vector(q20_c(1)) & std_logic_vector(q20_c(2));
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            sr(to_integer(rc + (C_LAT - 2))) <= '1' & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
            rc <= rc + 1;
        end if;
    end process p_ring;
    vr_d <= "000000" & std_logic_vector(v_iy) & std_logic_vector(v_iu(9 downto 2)) & std_logic_vector(v_iv(9 downto 2));

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
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_snap <= registers_in(6)(0);
                s_rad  <= registers_in(6)(1);
                s_vh   <= registers_in(6)(2);
                s_stag <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_k2   <= unsigned(registers_in(1));
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); s_sd <= '1';
                    when 3      => s_oxs <= to_unsigned(153, 9); s_sd <= '0';
                    when others => s_oxs <= to_unsigned(102, 9); s_sd <= '0';
                end case;
                -- Colour: 16 palettes with hysteresis
                v_d := signed(resize(unsigned(registers_in(4)), 11)) - signed(resize(s_pal & "100000", 11));
                if v_d > 40 or v_d < -40 then s_pal <= unsigned(registers_in(4)(9 downto 6)); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= f_oys(s_tid);

            -- shared glide for Scale and Open (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        if z_gc(0) = '0' then s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                        else s_gm <= s_gp; s_gr <= unsigned(registers_in(7)); end if;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if C_SIM_SNAP then
                            null;
                        elsif s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        if z_gc(0) = '0' then s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        else s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0)); end if;
                        z_gp <= "00";
                        if z_gc = 1 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 32 k, 24 iterations, stored at 60 + 32 k.  The slot
            -- number is decoded at 30 + 32 k and the operands pre-muxed at 31 + 32 k.
            z_pre <= '0';
            if s_zone = '1' and s_zc(4 downto 0) = 30 and s_zc(10 downto 9) = "00"
               and C_SLOTS(to_integer(s_zc(8 downto 5))) = '1' then
                z_sel <= s_zc(8 downto 5); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                case to_integer(z_sel) is
                    when 0  => z_fa(12 downto 0) <= s_g1;                 z_fb <= to_unsigned(5940, 16);      -- pitch exponent
                    when 3  =>                                                                                -- depth^2
                        z_fa(12 downto 3) <= unsigned(registers_in(2)); z_fb(12 downto 3) <= unsigned(registers_in(2));
                    when 2  => z_fa(9 downto 0)  <= s_k2;                 z_fb(11 downto 0) <= c_p2;          -- shape x pitch
                    when 4  => z_fa(11 downto 0) <= z_mp;                 z_fb <= to_unsigned(72, 16);        -- frame
                    when 5  => z_fa(11 downto 0) <= z_mp;                 z_fb <= to_unsigned(56, 16);        -- rim
                    when 7  => z_fa(9 downto 0)  <= s_kg(12 downto 3);    z_fb <= to_unsigned(171, 16);       -- camera gain
                    when 10 => z_fa(7 downto 0)  <= c_kq;                 z_fb <= to_unsigned(222, 16);       -- Osc centre
                    when 6  => z_fa(9 downto 0)  <= s_k2;                 z_fb(11 downto 0) <= z_hm;          -- shape x half-size
                    when 8  => z_fa(11 downto 0) <= c_rho;                z_fb(11 downto 0) <= c_rho;         -- rho^2
                    when 9  => z_fa(11 downto 0) <= c_rhoi;               z_fb(11 downto 0) <= c_rhoi;        -- (rho - rim)^2
                    when 12 => z_fa(8 downto 0)  <= s_vm;                 z_fb <= to_unsigned(900, 16);       -- rate exponent
                    when others => z_fa(15 downto 0) <= z_mt;             z_fb <= to_unsigned(37282, 16);     -- rate (14)
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- serial shifter for the 2^n scalings (no barrels)
            if z_shn /= 0 then
                z_shn <= z_shn - 1;
                if z_sht = '0' then c_pw <= c_pw(18 downto 0) & '0';
                else c_dph <= c_dph(20 downto 0) & '0'; end if;
            end if;

            -- the cell origin: cell (0, 0) is centred on the screen centre; step back to the first pixel / line
            if s_zone = '1' and (s_zc = 550 or s_zc = 630 or s_zc = 710) then z_k <= (others => '0'); end if;
            if z_nrun = '1' then
                if z_loc(12) = '1' then
                    z_loc <= z_loc + signed(resize(c_p2, 13));
                    z_kp  <= not z_kp; z_k <= z_k - 1;
                else
                    z_nrun <= '0';
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
                    when 21 =>
                        c_open <= s_gp(12 downto 5);

                    -- pitch: e = 59 + k1 * 2.9 octaves (Q8), P2 = 256 * 2^(e / 256) half px
                    when 60 => z_e <= f_acc(26 downto 16) + 59;
                    when 61 =>
                        r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 63 => z_mt <= r_ex_q;
                    when 64 =>
                        c_pw <= "0000" & z_mt; z_shn <= z_sh; z_sht <= '0';
                        c_sh <= z_sh(1 downto 0);
                    when 72 =>
                        c_ymp <= signed(resize(s_ys, 21)) - signed(resize(c_p2 & "0000000", 21));
                        if s_xs = 2 then
                            c_rs <= '0'; c_c4 <= to_signed(4, 15);
                            c_hs4 <= signed(resize(c_p2(11 downto 6), 10)) + to_signed(13, 10);   -- 4 + 9 + P2 / 64
                        elsif s_sd = '0' then
                            c_rs <= '1'; c_c4 <= to_signed(8, 15);
                            c_hs4 <= signed(resize(c_p2(11 downto 6), 10)) + to_signed(17, 10);   -- 8 + 9 + P2 / 64
                        else                         -- SD: seams 2.25 px fatter, so no seam is a single SD row
                            c_rs <= '1'; c_c4 <= to_signed(8, 15);
                            c_hs4 <= signed(resize(c_p2(11 downto 6), 10)) + to_signed(26, 10);   -- 8 + 18 + P2 / 64
                        end if;
                        c_sw <= c_sh;
                    -- K6 Camera: knob with a +-4 deadband (ends snap to 0 / 1023), then ease 1/8 of the gap
                    when 96 => s_kr <= unsigned(registers_in(5));
                    when 97 =>
                        v_d := signed(resize(s_kr, 11)) - signed(resize(s_kt, 11));
                        if s_kr < 4 then s_kt <= (others => '0');
                        elsif s_kr > 1019 then s_kt <= (others => '1');
                        elsif v_d > 4 or v_d < -4 then s_kt <= s_kr; end if;
                    when 98 => s_kgd <= signed(resize(s_kt & "000", 14)) - signed(resize(s_kg, 14));
                    when 99 =>
                        if C_SIM_SNAP or (s_kgd < 8 and s_kgd > -8) then s_kg <= s_kt & "000";
                        else s_kg <= unsigned(signed(s_kg) + resize(shift_right(s_kgd, 3), 13)); end if;
                    -- depth = (k3 / 1023)^2 (Q8 of a full aperture); Snap x 1.5
                    when 156 => c_d0 <= f_acc(25 downto 18);
                    when 157 =>
                        if s_snap = '1' then c_d <= resize(c_d0, 9) + resize(c_d0(7 downto 1), 9);
                        else c_d <= resize(c_d0, 9); end if;
                    -- housing (half px): mP = k2 P2 / 1024; frame = 4 + 72 mP / 1024; rim = 5 + 56 mP / 1024;
                    -- Hh = P2 / 2 - frame
                    when 124 => z_mp <= f_acc(21 downto 10);
                    when 188 => z_fr <= resize(f_acc(17 downto 10), 12) + 4;
                    when 189 => z_hh <= resize(c_p2h, 12) - z_fr;
                    when 190 => z_hm <= z_hh - 12;
                    when 220 => z_tr <= resize(f_acc(17 downto 10), 12) + 5;
                    -- corner radius rho = 12 + k2 (Hh - 12) / 1024
                    when 252 => c_rho <= f_acc(21 downto 10) + 12;
                    when 253 =>
                        c_c2 <= z_hh - c_rho; c_rhoi <= c_rho - z_tr; z_th <= z_hh - z_tr;
                    when 254 =>                      -- amax = 4 (Hh - rim) * 15/16 (Q3): the slivers stay >= 2 px
                        c_amax <= resize(shift_left(resize(z_th, 14), 2) - resize(shift_right(z_th, 2), 14), 13);
                        c_ho4 <= signed(resize(z_hh & "00", 15)) - c_c4;      -- the vertical edges' biases (Q3)
                    when 255 =>
                        c_hi4 <= signed(resize(z_th & "00", 15)) - c_c4;
                    -- camera gain kq = 171 k / 1024 (k = the glided K6): white (239) at 100 % adds 1.25 apertures;
                    -- the Osc centre 222 kq / 128
                    when 284 => c_kq <= f_acc(17 downto 10);
                    when 380 => c_off <= f_acc(15 downto 7);
                    when 316 => c_rsq <= f_acc(23 downto 0);
                    when 348 => c_risq <= f_acc(23 downto 0);
                    -- rate: 2^(m * 6.9 / 503) / 3600 cycles per field (EXP2 x 37282 / 2^16 Q26)
                    when 444 => z_e <= f_acc(18 downto 8);
                    when 445 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 447 => z_mt <= r_ex_q;
                    when 508 =>
                        if s_vm = 0 then c_dph <= (others => '0'); else c_dph <= resize(f_acc(31 downto 16), 22); end if;
                        z_shn <= z_sh; z_sht <= '1';
                    -- cell origin along x (960 or 720 virtual px to the centre) for even rows, then for odd rows
                    -- (Stagger: the grid half a cell to the right, Spread * (i + 1/2)), then along y (540)
                    when 550 =>
                        if s_sd = '1' then z_loc <= signed(resize(c_p2h, 13)) - 1440;
                        else z_loc <= signed(resize(c_p2h, 13)) - 1920; end if;
                        z_kp <= '0'; z_nrun <= '1';
                    when 620 =>
                        c_x0 <= unsigned(z_loc(11 downto 0)); c_ci0 <= z_k;
                    when 630 =>
                        if s_sd = '1' then z_loc <= to_signed(-1440, 13); else z_loc <= to_signed(-1920, 13); end if;
                        z_kp <= '0'; z_nrun <= '1';
                    when 700 =>
                        c_x1 <= unsigned(z_loc(11 downto 0)); c_ci1 <= z_k;
                    when 710 =>
                        z_loc <= signed(resize(c_p2h, 13)) - 1080;
                        z_kp <= '0'; z_nrun <= '1';
                    when 780 => z_first <= '1';
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
    -- Palette reads and table fills (zone, from zc 22; adders only).
    --   T2(co) = accent + (ground - accent) co / 8          co = 0 .. 8
    --   T1(k, cs) = tone_k + (seam - tone_k) cs / 8         k = 0 .. 7, cs = 0 .. 8
    -- (centred: luma - 64, chroma 8 bits - 128)
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_qy : signed(16 downto 0);
        variable v_qc : t_s13;
    begin
        if rising_edge(clk) then
            tt_we <= '0'; o2_we <= '0';
            v_qy := signed(resize(unsigned(r_pl_q(25 downto 16)), 17));
            v_qc(1) := resize(signed(r_pl_q(15 downto 8)), 13);
            v_qc(2) := resize(signed(r_pl_q(7 downto 0)), 13);
            -- the copy's pipeline: word address (d1) -> address register (d2) -> ROM output (d3) -> T2 (word 8 is
            -- the ground: the blanking and resync colour)
            f_v2 <= f_v1; f_d2 <= f_d1; f_v3 <= f_v2; f_d3 <= f_d2;
            if f_v3 = '1' then
                o2_we <= '1';
                o2_wa <= "000" & f_d3;
                f_wd <= r_pl_q;
                if f_d3 = 8 then
                    c_gy <= unsigned(r_pl_q(25 downto 16)) + 64;
                    c_gc(1) <= unsigned(r_pl_q(15 downto 8)) xor "10000000"; c_gc(2) <= unsigned(r_pl_q(7 downto 0)) xor "10000000";
                end if;
            else
                f_wd <= "000000" & std_logic_vector(f_ay(12 downto 3)) & std_logic_vector(f_ac(1)(10 downto 3))
                        & std_logic_vector(f_ac(2)(10 downto 3));
            end if;
            if s_zone = '1' and s_zc = 22 then
                f_st <= to_unsigned(1, 4); f_d1 <= (others => '0'); f_v1 <= '0';
            else
                case to_integer(f_st) is
                    when 1 =>                                    -- copy words 0 .. 9 into T2
                        r_pl_a <= s_pal & f_d1; f_v1 <= '1';
                        if f_v1 = '1' then f_d1 <= f_d1 + 1; end if;
                        if f_d1 = 9 and f_v1 = '1' then f_v1 <= '0'; f_k <= (others => '0'); f_wt <= "00"; f_st <= to_unsigned(2, 4); end if;
                    when 2 | 3 =>                                -- T1 row k: tone k (2), then the seam (3)
                        if f_wt = 0 then
                            if f_st = 2 then r_pl_a <= s_pal & ("10111" + resize(f_k, 5)); else r_pl_a <= s_pal & "11111"; end if;
                        end if;
                        f_wt <= f_wt + 1;
                        if f_wt = 3 then
                            f_wt <= "00";
                            if f_st = 2 then
                                f_ay <= shift_left(v_qy, 3);
                                for i in 1 to 2 loop f_ac(i) <= shift_left(v_qc(i), 3); end loop;
                                f_st <= to_unsigned(3, 4);
                            else
                                f_dy <= v_qy - shift_right(f_ay, 3);
                                for i in 1 to 2 loop f_dc(i) <= v_qc(i) - shift_right(f_ac(i), 3); end loop;
                                f_cnt <= (others => '0');
                                f_st <= to_unsigned(4, 4);
                            end if;
                        end if;
                    when 4 =>                                    -- 9 writes: tone + (seam - tone) cs / 8
                        tt_we <= '1'; tt_wa <= f_k & f_cnt;
                        f_ay <= f_ay + f_dy;
                        for i in 1 to 2 loop f_ac(i) <= f_ac(i) + f_dc(i); end loop;
                        f_cnt <= f_cnt + 1;
                        if f_cnt = 8 then
                            if f_k = 7 then f_st <= (others => '0');
                            else f_k <= f_k + 1; f_st <= to_unsigned(2, 4); end if;
                        end if;
                    when others => null;
                end case;
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
            g_st1 <= g_start; g_st2 <= g_st1; g_st3 <= g_st2;
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- [C] Source field.  Osc: a two-axis triangle lattice (640 x 540 virtual
    -- px) drifting diagonally, with its own colours.  The luma (or the lattice)
    -- is smeared along x by a 1/64 filter, scaled by the camera gain (a table), and held
    -- per cell (sampled on the last line of each cell row at each cell column's
    -- last pixel of the NEXT row's grid), so every iris is rigid.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_tx, v_ty : unsigned(6 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_o  : unsigned(7 downto 0);
        variable v_l  : unsigned(7 downto 0);
        variable v_x  : unsigned(13 downto 0);
        variable v_d1 : signed(14 downto 0);
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
            v_o := v_s - shift_right(v_s, 3);

            -- input sample: source value (8 bits) and the hole colour {y - 64, u, v}; held through blanking
            s_sav <= data_in.avid; s_savd <= s_sav;
            if data_in.avid = '1' then
                if s_src = '1' then
                    s_sv <= v_o;
                    v_iy <= shift_left(resize(v_o, 10), 1) + resize(v_o, 10);                        -- 3 o
                    v_iu <= shift_left(resize(v_tx, 10), 2) + shift_left(resize(v_tx, 10), 1) + 128; -- 6 tx + 128
                    v_iv <= shift_left(resize(v_ty, 10), 2) + shift_left(resize(v_ty, 10), 1) + 128;
                else
                    v_l := unsigned(data_in.y(9 downto 2));
                    if v_l > 16 then s_sv <= v_l - 16; else s_sv <= (others => '0'); end if;
                    if unsigned(data_in.y) > 64 then v_iy <= unsigned(data_in.y) - 64; else v_iy <= (others => '0'); end if;
                    v_iu <= unsigned(data_in.u); v_iv <= unsigned(data_in.v);
                end if;
            end if;

            -- smear along the line
            v_x := s_sv & "000000";
            if s_sav = '1' and s_savd = '0' then
                s_y1 <= v_x;
                s_srow <= z_first or not ls_n2(20);    -- line 0, or the last row before a new cell row
                s_shx  <= s_stag and not z_first;      -- Stagger: the next row's grid is half a cell over
                s_wbk  <= ls_jp xor not z_first;       -- the bank the next row reads
            elsif s_sav = '1' then
                v_d1 := signed(resize(v_x, 15)) - signed(resize(s_y1, 15));
                s_y1 <= unsigned(signed(s_y1) + resize(shift_right(v_d1, 6), 14));
            end if;

            -- K6 Camera: the smeared sample through the gain table (address 1, RAM 2, held value 3 clocks on)
            gt_ra <= s_y1(13 downto 6);
            s_hv <= signed(gt_q(11 downto 0));
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [D] Cell accumulators: the offset inside the cell (x per pixel, y per
    -- line; half virtual px), Spread * i and Spread * j stepped at each cell
    -- wrap, the cell column, the cell's aperture (from the engine).  ls_* hold
    -- the NEXT line's values.  Per line, a serial squarer forms
    -- max(|q| - c, 0)^2 for the housing.
    ---------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_n   : unsigned(12 downto 0);
        variable v_w   : signed(13 downto 0);
        variable v_c   : signed(13 downto 0);
        variable v_aq  : signed(14 downto 0);
        variable v_r   : signed(27 downto 0);
        variable v_rq  : unsigned(23 downto 0);
        variable v_q   : unsigned(25 downto 0);
        variable v_m   : unsigned(14 downto 0);
        variable v_rr  : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- line registers: wrap candidates are pre-registered (ls_y is stable between line starts)
            ls_n1 <= ls_y + resize(s_ys, 19);
            ls_n2 <= signed(resize(ls_y, 21)) + c_ymp;
            ls_q8 <= signed(resize(ls_y(18 downto 5), 15)) - signed(resize(c_p2 & '0', 15));
            v_aq := abs(ls_q8); ls_aq <= unsigned(v_aq(13 downto 0));
            if s_zone = '1' and s_zc = 780 then
                ls_y <= unsigned(z_loc(11 downto 0)) & "0000000"; ls_jp <= z_kp; ls_cj <= z_k;
            elsif g_start = '1' then
                if ls_n2(20) = '0' then
                    ls_y <= unsigned(ls_n2(18 downto 0)); ls_jp <= not ls_jp; ls_cj <= ls_cj + 1;
                else
                    ls_y <= ls_n1;
                end if;
            end if;
            -- per-line housing for the next line: u^2 (13 steps), isqrt of max(rho^2 - u^2, 0) for the outer and
            -- the rim edge (13 steps each), W = c + root, the ramp shift, the biased half-widths; done ~45 clocks in
            if (s_zone = '1' and s_zc = 783) or g_st3 = '1' then
                h_z <= s_zone;
                v_c := signed(resize(ls_aq, 14)) - signed(resize(c_c2 & "00", 14));       -- u = |q| - c (Q3)
                if v_c(13) = '1' then h_b <= (others => '0'); h_a <= (others => '0');
                else h_b <= unsigned(v_c(12 downto 0)); h_a <= resize(unsigned(v_c(12 downto 0)), 26); end if;
                h_sq <= (others => '0');
                ls_cvo <= f_cov(signed(resize(ls_aq, 15)) - c_ho4, c_rs);
                ls_cvi <= f_cov(signed(resize(ls_aq, 15)) - c_hi4, c_rs);
                h_st <= to_unsigned(1, 6);
            elsif h_st /= 0 then
                h_st <= h_st + 1;
                if h_st < 14 then                                -- 1 .. 13: u^2
                    if h_b(0) = '1' then h_sq <= h_sq + h_a; end if;
                    h_a <= h_a(24 downto 0) & '0'; h_b <= '0' & h_b(12 downto 1);
                end if;
                if h_st = 14 or h_st = 28 then                   -- the radicand (14 outer, 28 rim)
                    if h_st(4) = '1' then v_rq := c_risq; else v_rq := c_rsq; end if;
                    v_r := signed(resize(v_rq & "0000", 28)) - signed(resize(h_sq, 28));
                    if v_r(27) = '1' then h_x <= (others => '0'); else h_x <= unsigned(v_r(24 downto 0)); end if;
                    h_r <= (others => '0'); h_o <= (24 => '1', others => '0');
                else                                             -- isqrt, one bit per clock
                    v_q := ('0' & h_x) - ('0' & (h_r or h_o));
                    if v_q(25) = '0' then h_x <= v_q(24 downto 0); h_r <= ('0' & h_r(24 downto 1)) or h_o;
                    else h_r <= '0' & h_r(24 downto 1); end if;
                    h_o <= "00" & h_o(24 downto 2);
                end if;
                if h_st = 28 then ls_wo <= resize(c_c2 & "00", 14) + resize(h_r(12 downto 0), 14); end if;
                if h_st = 42 then
                    ls_wi <= resize(c_c2 & "00", 14) + resize(h_r(12 downto 0), 14);
                    if h_z = '1' then h_d <= (others => '0');
                    else h_d <= signed(resize(ls_wo, 15)) - signed(resize(l_wo, 15)); end if;
                end if;
                if h_st = 43 then                                -- shift = clamp(bit length of |dW| - 3, rs, 9)
                    v_m := unsigned(h_d xor (h_d'range => h_d(14)));
                    v_rr := "000" & c_rs;
                    for i in 3 to 14 loop
                        if v_m(i) = '1' then
                            if i > 11 then v_rr := to_unsigned(9, 4); else v_rr := to_unsigned(i - 2, 4); end if;
                        end if;
                    end loop;
                    if v_rr = 0 and c_rs = '1' then v_rr := to_unsigned(1, 4); end if;
                    ls_rr <= v_rr;
                end if;
                if h_st = 44 then
                    ls_top <= shift_left(to_unsigned(8, 13), to_integer(ls_rr));
                    ls_wbo <= signed(ls_wo) - signed(resize(shift_left(to_unsigned(4, 13), to_integer(ls_rr)), 14));
                    ls_wbi <= signed(ls_wi) - signed(resize(shift_left(to_unsigned(4, 13), to_integer(ls_rr)), 14));
                    h_st <= (others => '0');
                end if;
            end if;

            -- the engine's line job (the next line's cells 0, 1): at the end of each line, and in the zone
            if (s_zone = '1' and s_zc = 785) or (s_gav_sr(0) = '1' and g_avid = '0') then e_gol <= '1'; else e_gol <= '0'; end if;

            -- pixel accumulators (stage 1); at each wrap the next cell's aperture comes in and the engine starts
            -- on the cell after it
            e_gonx <= '0';
            if g_start = '1' then
                if s_stag = '1' and ls_jp = '1' then a_x <= c_x1; a_ci <= c_ci1;
                else a_x <= c_x0; a_ci <= c_ci0; end if;
                l_odd <= s_stag and ls_jp;
                a_nw <= '0'; a_hx <= '0';
                a_cur <= a_l0; l_fw <= '1'; kc <= (others => '0');
                l_cj <= ls_cj; l_q8 <= ls_q8; l_aq <= ls_aq; l_jp <= ls_jp;
                l_wo <= ls_wo; l_wbo <= ls_wbo; l_wbi <= ls_wbi; l_top <= ls_top; l_rr <= ls_rr; l_cvo <= ls_cvo; l_cvi <= ls_cvi;
            elsif g_avid = '1' then
                v_n := resize(a_x, 13) + s_xs;
                v_w := signed(resize(v_n, 14)) - signed(resize(c_p2, 14));
                if v_w(13) = '0' then
                    a_x <= unsigned(v_w(11 downto 0));
                    a_ci <= a_ci + 1;
                    a_nw <= '1'; a_hx <= '0';
                    if l_fw = '1' then a_cur <= a_l1; else a_cur <= a_nx; end if;
                    a_prev <= a_cur;
                    l_fw <= '0';
                    kc <= kc + 1;
                    e_gonx <= '1'; e_nci <= a_ci + 2; e_ncol <= kc + 2;
                else
                    a_x <= v_n(11 downto 0);
                    a_nw <= '0';
                    if a_x < resize(c_p2h, 12) and v_n >= resize(c_p2h, 13) then a_hx <= '1'; else a_hx <= '0'; end if;
                end if;
            else
                a_nw <= '0'; a_hx <= '0';
            end if;
        end if;
    end process p_acc;

    ---------------------------------------------------------------------------
    -- [D] The per-cell engine.  A job computes one cell's aperture in 21
    -- clocks, one small operation per clock (one serial Depth x wave):
    --   theta = Spread * index - phase, the first term from a ROM by cell index (Linear: i + j;
    --   Radial: max + min / 2 of |i|, |j|; staggered rows count i + 1/2)
    --   f = Open + source +- Depth * wave(theta), clamped 0 .. 1;  a = table(f)
    -- Jobs: at each wrap the cell after next (a_nx); at the end of each line
    -- the next line's cells 0 and 1 (a_l0, a_l1), which abort a next-cell job.
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_h   : signed(5 downto 0);
        variable v_v   : signed(4 downto 0);
        variable v_pos : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if e_gol = '1' then
                e_job <= "10"; e_st <= to_unsigned(1, 5);
                if s_stag = '1' and ls_jp = '1' then e_ci <= c_ci1; else e_ci <= c_ci0; end if;
                e_cj <= ls_cj; e_od <= s_stag and ls_jp; e_col <= (others => '0'); e_bk <= ls_jp;
            elsif e_gonx = '1' then
                e_job <= "00"; e_st <= to_unsigned(1, 5);
                e_ci <= e_nci; e_cj <= l_cj; e_od <= l_odd; e_col <= e_ncol; e_bk <= l_jp;
            elsif e_st /= 0 then
                e_st <= e_st + 1;
                case to_integer(e_st) is
                    when 1 =>                                    -- the base-phase ROM's address; the hold's
                        if s_rad = '1' then
                            v_h := (e_ci & '0') + ("00000" & e_od);       -- 2 i (+1 in a staggered row)
                            if v_h(5) = '1' then v_h := -v_h; end if;
                            v_v := e_cj;
                            if v_v(4) = '1' then v_v := -v_v; end if;
                            r_t0_a <= '1' & unsigned(v_h(3 downto 0)) & unsigned(v_v(2 downto 0));
                        else
                            r_t0_a <= "00" & unsigned(e_ci + e_cj) & e_od;
                        end if;
                        sl_ra <= e_bk & e_col;
                    when 2 => null;                              -- ROMs
                    when 3 =>                                    -- theta = Spread * index - phase; the held source
                        e_th <= r_t0_q(13 downto 0) - c_ph(25 downto 12);
                        e_src <= signed(sl_q(11 downto 0));
                    when 4 => null;
                    when 5 =>                                    -- wave ROM address: quarter fold; Snap
                        v_pos := e_th(11 downto 0);
                        if e_th(12) = '1' then v_pos := not v_pos; end if;
                        if s_snap = '0' then r_w_a <= v_pos(11 downto 4);
                        elsif v_pos(11 downto 10) /= "00" then r_w_a <= (others => '1');
                        else r_w_a <= v_pos(9 downto 2); end if;
                        e_neg <= e_th(13);
                    when 6 => null;                              -- ROM
                    when 7 =>                                    -- Depth * wave: 9 shift-add steps
                        e_ma <= resize(r_w_q(9 downto 0), 19); e_mb <= c_d; e_acc <= (others => '0');
                    when 8 to 16 =>
                        if e_mb(0) = '1' then e_acc <= e_acc + e_ma; end if;
                        e_ma <= e_ma(17 downto 0) & '0'; e_mb <= '0' & e_mb(8 downto 1);
                    when 17 =>                                   -- f = Open + source +- Depth * wave (Q8)
                        e_f <= signed(resize(c_open, 13)) + resize(e_src, 13);
                    when 18 =>
                        if e_neg = '1' then e_f <= e_f - signed(resize(e_acc(18 downto 10), 13));
                        else e_f <= e_f + signed(resize(e_acc(18 downto 10), 13)); end if;
                    when 19 =>                                   -- the aperture table (clamped 0 .. 1)
                        if e_f(12) = '1' then at_ra <= (others => '0');
                        elsif e_f > 255 then at_ra <= (others => '1');
                        else at_ra <= unsigned(e_f(7 downto 0)); end if;
                    when 20 => null;                             -- RAM
                    when others =>                               -- 21: the result
                        if e_job = "00" then a_nx <= unsigned(at_q(12 downto 0));
                        elsif e_job = "10" then a_l0 <= unsigned(at_q(12 downto 0));
                        else a_l1 <= unsigned(at_q(12 downto 0)); end if;
                        if e_job = "10" then                     -- then the line's second cell
                            e_job <= "11"; e_st <= to_unsigned(1, 5);
                            e_ci <= e_ci + 1; e_col <= to_unsigned(1, 7);
                        else
                            e_st <= (others => '0');
                        end if;
                end case;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- [C] Source hold, write side: on the sample row, at each of the next
    -- row's cell ends (wraps, or half-cell crossings in Stagger), store the
    -- held value in the next row's bank
    ---------------------------------------------------------------------------
    p_hold : process(clk)
        variable v_kw : unsigned(7 downto 0);
        variable v_st7, v_st9 : std_logic;
    begin
        if rising_edge(clk) then
            s_nw(2) <= a_nw; s_hx(2) <= a_hx;
            for i in 3 to 9 loop s_nw(i) <= s_nw(i - 1); s_hx(i) <= s_hx(i - 1); end loop;
            if s_shx = '1' then v_st7 := s_hx(7); v_st9 := s_hx(9); else v_st7 := s_nw(7); v_st9 := s_nw(9); end if;
            v_kw := kw8;
            if s_gav_sr(5) = '1' and s_gav_sr(6) = '0' then v_kw := (others => '0');
            elsif v_st7 = '1' then v_kw := kw8 + 1; end if;
            kw8 <= v_kw; kw9 <= kw8; kw10 <= kw9;
            sl_wd <= "0000" & std_logic_vector(s_hv);
            sl_wa <= s_wbk & kw10(6 downto 0);
            if s_srow = '1' and s_gav_sr(8) = '1' and (v_st9 = '1' or s_gav_sr(7) = '0') then sl_we <= '1'; else sl_we <= '0'; end if;
        end if;
    end process p_hold;

    ---------------------------------------------------------------------------
    -- [E] The look: iris geometry (2 .. 14), housing (8 .. 12), colour (15 .. 20)
    ---------------------------------------------------------------------------
    p_geo : process(clk)
        variable v_p8  : signed(14 downto 0);
        variable v_ab  : signed(14 downto 0);
        variable v_ap  : signed(13 downto 0);
        variable v_t   : unsigned(13 downto 0);
        variable v_s   : std_logic_vector(7 downto 0);
        variable v_m   : t_m4;
        variable v_ng  : std_logic_vector(3 downto 0);
        variable v_ge, v_nz : std_logic;
        variable v_ka  : unsigned(2 downto 0);
        variable v_sa, v_sb : unsigned(12 downto 0);
        variable v_bn, v_sp, v_any : std_logic;
        variable v_kb  : unsigned(2 downto 0);
        variable v_d   : signed(13 downto 0);
        variable v_n   : signed(14 downto 0);
        variable v_j   : integer range 0 to 3;
        variable v_a6, v_a8 : unsigned(12 downto 0);
        variable v_o5, v_o7 : std_logic;
    begin
        if rising_edge(clk) then
            -- the aperture of the pixel at stage k: the current cell's, or the previous cell's while a wrap is
            -- younger than it (cells are far wider than the pipe)
            v_o5 := a_nw or s_nw(2) or s_nw(3) or s_nw(4) or s_nw(5);
            v_o7 := v_o5 or s_nw(6) or s_nw(7);
            if v_o5 = '1' then v_a6 := a_prev; else v_a6 := a_cur; end if;
            if v_o7 = '1' then v_a8 := a_prev; else v_a8 := a_cur; end if;
            -- 2: p (Q3)
            v_p8 := signed(resize(a_x & "00", 15)) - signed(resize(c_p2 & '0', 15));
            q2_p <= v_p8(13 downto 0);
            -- 3: the sums for the diagonal forms
            q3_sp <= resize(q2_p, 15) + resize(l_q8, 15);
            q3_sm <= resize(l_q8, 15) - resize(q2_p, 15);
            q3_p  <= q2_p;
            -- 4: magnitudes and signs
            v_ab := abs(q3_sp); q4_asp <= unsigned(v_ab(13 downto 0));
            v_ab := abs(q3_sm); q4_asm <= unsigned(v_ab(13 downto 0));
            v_ap := abs(q3_p);  q4_ap  <= unsigned(v_ap(12 downto 0));
            q4_ng <= q3_sm(14) & q3_sp(14) & q3_p(13);
            -- 5, 6: x 181 / 256 = x - x/4 - (x/32 + x/64 - x/256) on the magnitudes
            q5_p1 <= q4_asp - shift_right(q4_asp, 2);
            q5_p2 <= resize(shift_right(q4_asp, 5), 10) + resize(shift_right(q4_asp, 6), 10) - resize(shift_right(q4_asp, 8), 10);
            q5_m1 <= q4_asm - shift_right(q4_asm, 2);
            q5_m2 <= resize(shift_right(q4_asm, 5), 10) + resize(shift_right(q4_asm, 6), 10) - resize(shift_right(q4_asm, 8), 10);
            q5_ap <= q4_ap; q5_ng <= q4_ng;
            v_t := q5_p1 - resize(q5_p2, 14); q6_l1m <= v_t(12 downto 0);
            v_t := q5_m1 - resize(q5_m2, 14); q6_l3m <= v_t(12 downto 0);
            q6_ap <= q5_ap; q6_ng <= q5_ng;
            -- 7: |L_j| >= a: edge j (L_j >= 0) or edge j + 4 (L_j <= 0) lies beyond it; e_k >= 0 for the eight
            -- edges; blade k owns the pixel where e_k >= 0 and e_(k+1) < 0 (one-hot); pair maxima of |L|
            v_m(0) := q6_ap; v_m(1) := q6_l1m; v_m(2) := l_aq(12 downto 0); v_m(3) := q6_l3m;
            v_ng := q6_ng(2) & l_q8(14) & q6_ng(1) & q6_ng(0);
            for j in 0 to 3 loop
                if v_m(j) >= v_a6 then v_ge := '1'; else v_ge := '0'; end if;
                if v_m(j) /= 0 then v_nz := '1'; else v_nz := '0'; end if;
                v_s(j)     := v_ge and not (v_ng(j) and v_nz);
                v_s(j + 4) := v_ge and (v_ng(j) or not v_nz);
            end loop;
            q7_s <= v_s;
            for k in 0 to 7 loop q7_b(k) <= v_s(k) and not v_s((k + 1) mod 8); end loop;
            q7_m(0) <= v_m(0); q7_m(1) <= v_m(1); q7_m(2) <= v_m(2); q7_m(3) <= v_m(3);
            q7_ng <= v_ng;
            if q6_l1m > q6_ap then q7_m01 <= q6_l1m; q7_sel01 <= '1'; else q7_m01 <= q6_ap; q7_sel01 <= '0'; end if;
            if q6_l3m > l_aq(12 downto 0) then q7_m23 <= q6_l3m; q7_sel23 <= '1'; else q7_m23 <= l_aq(12 downto 0); q7_sel23 <= '0'; end if;
            -- 8: the blade's two edges by and-or select: |L_k|, and edge k+1's form with its sign as seen from the
            -- blade (d2 = a - sigma L); the previous edge's state; no blade = the hole (k = its nearest edge)
            v_sa := (others => '0'); v_sb := (others => '0'); v_bn := '0'; v_sp := '0'; v_kb := (others => '0'); v_any := '0';
            for k in 0 to 7 loop
                if q7_b(k) = '1' then
                    v_sa := v_sa or q7_m(k mod 4);
                    v_sb := v_sb or q7_m((k + 1) mod 4);
                    if (k + 1) mod 8 < 4 then v_bn := v_bn or q7_ng((k + 1) mod 4);
                    else v_bn := v_bn or not q7_ng((k + 1) mod 4); end if;
                    v_sp := v_sp or q7_s((k + 7) mod 8);
                    v_kb := v_kb or to_unsigned(k, 3);
                    v_any := '1';
                end if;
            end loop;
            q8_sa <= v_sa; q8_sb <= v_sb; q8_bn <= v_bn; q8_sp <= v_sp;
            if q7_m23 > q7_m01 then
                q8_r8 <= q7_m23;
                if q7_sel23 = '1' then v_ka := q7_ng(3) & "11"; else v_ka := q7_ng(2) & "10"; end if;
            else
                q8_r8 <= q7_m01;
                if q7_sel01 = '1' then v_ka := q7_ng(1) & "01"; else v_ka := q7_ng(0) & "00"; end if;
            end if;
            if v_any = '0' then q8_hole <= '1'; q_k(8) <= v_ka; else q8_hole <= '0'; q_k(8) <= v_kb; end if;
            for i in 9 to 13 loop q_k(i) <= q_k(i - 1); end loop;
            -- 9: d1 = |L_k| - a, d2 = a -+ |L_(k+1)| (0 .. 255); -dh = a - (max |L|)
            v_d := signed(resize(q8_sa, 14)) - signed(resize(v_a8, 14));
            if v_d > 255 then q9_d1 <= (others => '1'); else q9_d1 <= unsigned(v_d(7 downto 0)); end if;
            if q8_bn = '1' then v_d := signed(resize(v_a8, 14)) + signed(resize(q8_sb, 14));
            else v_d := signed(resize(v_a8, 14)) - signed(resize(q8_sb, 14)); end if;
            if v_d > 255 then q9_d2 <= (others => '1'); else q9_d2 <= unsigned(v_d(7 downto 0)); end if;
            q9_sp <= q8_sp; q9_hole <= q8_hole;
            q9_nd <= signed(resize(v_a8, 15)) - signed(resize(q8_r8, 15));
            -- 10: hole coverage = clamp(4 + (-dh >> rs)); seam distance
            q10_ch <= f_cov(q9_nd + c_c4, c_rs);
            if q9_hole = '1' then q10_ds <= (others => '1');
            elsif q9_sp = '1' and q9_d1 < q9_d2 then q10_ds <= q9_d1;
            else q10_ds <= q9_d2; end if;
            -- 11, 12: seam coverage = clamp(((4 << rs) + hs - d) >> rs, 0, 8)
            q11_t <= c_hs4 - signed(resize(q10_ds, 10));
            q11_ch <= q10_ch;
            q12_cs <= f_cov(q11_t, c_rs);
            q12_ch <= q11_ch;
            -- 13: the second colour Z and its coverage: the rim / ground (T2 0 .. 8) where the rim covers, else
            -- the hole (T2 9: the glass; or the camera)
            if q_cr(12) /= 0 then
                q13_a2 <= q_co(12); q_c(13) <= q_cr(12); q_rim(13) <= '1';
            else
                q13_a2 <= to_unsigned(9, 4); q_c(13) <= q12_ch; q_rim(13) <= '0';
            end if;
            q13_cs <= q12_cs;
            for i in 14 to 17 loop q_c(i) <= q_c(i - 1); end loop;
            for i in 14 to 15 loop q_rim(i) <= q_rim(i - 1); end loop;
            -- 14: table addresses (T1: blade tone toward the seam; T2: Z)
            tt_ra <= q_k(13) & q13_cs;
            o2_ra <= "0000" & q13_a2;

            -- housing ------------------------------------------------------------------------------------
            -- 8: |p| against the line's biased half-widths; 9: coverages (horizontal, then max with the line's
            -- vertical coverage for the straight top / bottom edges)
            q8_vo <= signed(resize(q7_m(0), 14)) - l_wbo;
            q8_vi <= signed(resize(q7_m(0), 14)) - l_wbi;
            q_co(9) <= f_max4(f_hc(q8_vo, l_rr, l_top), l_cvo);
            q_cr(9) <= f_max4(f_hc(q8_vi, l_rr, l_top), l_cvi);
            for i in 10 to 12 loop q_co(i) <= q_co(i - 1); q_cr(i) <= q_cr(i - 1); end loop;
        end if;
    end process p_geo;

    p_col : process(clk)
    begin
        if rising_edge(clk) then
            -- 16: T1 (fabric, RAM at 15); Z: T2, or the camera (from its ring) through the hole in Video mode
            q16_iy <= unsigned(tt_q(25 downto 16));
            q16_ic(1) <= signed(tt_q(15 downto 8)); q16_ic(2) <= signed(tt_q(7 downto 0));
            if s_vh = '1' and q_rim(15) = '0' then
                q16_zy <= unsigned(vr_f(25 downto 16));
                q16_zc(1) <= signed(vr_f(15 downto 8) xor "10000000"); q16_zc(2) <= signed(vr_f(7 downto 0) xor "10000000");
            else
                q16_zy <= unsigned(o2_q(25 downto 16));
                q16_zc(1) <= signed(o2_q(15 downto 8)); q16_zc(2) <= signed(o2_q(7 downto 0));
            end if;
            -- 17: Z less the blade colour
            q17_dy <= signed(resize(q16_zy, 11)) - signed(resize(q16_iy, 11));
            for i in 1 to 2 loop q17_dc(i) <= resize(q16_zc(i), 9) - resize(q16_ic(i), 9); end loop;
            q17_iy <= q16_iy; q17_ic <= q16_ic;
            -- 18: 8 I + (Z - I) c as gated sums (c = 0 .. 8), in two parts
            if q_c(17)(3) = '1' then
                q18_ty0 <= shift_left(resize(q17_dy, 14), 3);
                q18_ty1 <= shift_left(signed(resize(q17_iy, 14)), 3);
                for i in 1 to 2 loop
                    q18_tc0(i) <= shift_left(resize(q17_dc(i), 12), 3);
                    q18_tc1(i) <= shift_left(resize(q17_ic(i), 12), 3);
                end loop;
            else
                q18_ty0 <= f_gs(q17_dy, q_c(17)(0), 14) + shift_left(f_gs(q17_dy, q_c(17)(1), 14), 1);
                q18_ty1 <= shift_left(f_gs(q17_dy, q_c(17)(2), 14), 2) + shift_left(signed(resize(q17_iy, 14)), 3);
                for i in 1 to 2 loop
                    q18_tc0(i) <= f_gs(q17_dc(i), q_c(17)(0), 12) + shift_left(f_gs(q17_dc(i), q_c(17)(1), 12), 1);
                    q18_tc1(i) <= shift_left(f_gs(q17_dc(i), q_c(17)(2), 12), 2) + shift_left(resize(q17_ic(i), 12), 3);
                end loop;
            end if;
            -- 19
            q19_y <= q18_ty0 + q18_ty1;
            for i in 1 to 2 loop q19_c(i) <= q18_tc0(i) + q18_tc1(i); end loop;
            -- 20: / 8, offsets (a convex blend: always in range); the ground colour outside the generated avid
            if s_gav_sr(17) = '0' then
                q20_y <= c_gy; q20_c <= c_gc;
            else
                q20_y <= unsigned(q19_y(12 downto 3)) + 64;
                for i in 1 to 2 loop q20_c(i) <= unsigned(q19_c(i)(10 downto 3)) xor "10000000"; end loop;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(17);
        end if;
    end process p_col;

    ---------------------------------------------------------------------------
    -- [F] Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent
    -- 8-sums of the 8-bit chroma, resynced every line: 10 bits out), luma [1 2 1]/4 delayed to the chroma's
    -- group delay, luma 13/16, blanking gate.  The filters add 4 pixels of
    -- delay: the output register is effective stage 28 = C_LAT + 1.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 22: running 8-sums (the sample of 8 clocks before comes back from its delay ring); they snap
            -- back to 8 x ground once 8 blank samples are in, so a glitched sample can't offset the colours for good
            if s_bah = "00000000" then
                s_bu <= c_gc(1) & "000"; s_bv <= c_gc(2) & "000";
            else
                s_bu <= s_bu + q20_c(1) - unsigned(cr_f(15 downto 8));
                s_bv <= s_bv + q20_c(2) - unsigned(cr_f(7 downto 0));
            end if;
            s_bud <= s_bu; s_bvd <= s_bv;
            s_ysr(0) <= q20_y;
            for i in 1 to 5 loop s_ysr(i) <= s_ysr(i - 1); end loop;
            -- 23: filtered
            v_sc := resize(s_bu, 12) + s_bud; s_u31 <= v_sc(11 downto 2);
            v_sc := resize(s_bv, 12) + s_bvd; s_v31 <= v_sc(11 downto 2);
            v_y := resize(s_ysr(3), 12) + resize(s_ysr(4) & '0', 12) + resize(s_ysr(5), 12);
            s_y31 <= v_y(11 downto 2);
            -- 24: luma 13/16 toward black
            if s_y31 > 64 then v_dl := s_y31 - 64; else v_dl := (others => '0'); end if;
            s_y32 <= s_y31 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u32 <= s_u31; s_v32 <= s_v31;
            -- 25: blanking gate
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

end architecture aperture;
