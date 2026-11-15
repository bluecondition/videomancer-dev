-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Agate v0.1.0 -- Patchbay: marbling by cascaded cross-modulation of the H and V ramps.
--
-- The patch (agate.py has the palettes; .patchbay_work/rb_agate/imodel.py is the bit-accurate model):
--   X = H ramp * Scale, Y = V ramp * Scale                     (phases, one unit = one base cycle)
--   stage 0:  X += ax0 * w(Y + p0)      (per line),   Y += ay0 * w(X + p1)
--   stage 1:  X += ax1 * w(2 Y + p2),   X += luma (the camera, or the Osc lattice at four times the level),
--             Y += ay1 * w(2 X + p3)
--   stage 2:  X += ax2 * w(4 Y + p4)
--             w = sine (Round) or triangle (Angular); every phase p drifts at its own rate
--   U = X * Bands -> wavefolder (triangle) -> + Colour -> banded colouriser (256-entry palette ROM)
--   footprint |dU/dx| + |dU/dy| fades the colouriser to a blurred palette, then to the window's mean
--
-- The stage multiplies a * w() are done once per field: four 256-entry quarter-wave tables hold
-- amplitude * waveform (one serial multiplier fills them in the vblank zone), so the pixel path is
-- lookups and adds.  The two largest (stage 0's Y term, stage 1's X term) also hold the forward
-- difference and are read with linear interpolation (3 fraction bits): at 1024 phase steps per cycle
-- their steps showed as row-to-row jitter of the band edges.  Stage 0's X term depends on the line
-- only: the line engine computes it (interpolated, 4 fraction bits).
--
--   K1  Scale     base frequency of the ramps (3 octaves)
--   K2  Lean      balance of the V-into-H (left) and H-into-V (right) modulation depths
--   K3  Detail    amplitude ratio from each stage to the next finer one (0.25 .. 0.75)
--   K4  Speed     drift of the five stage phases (bipolar, square law, +-8 deadband)
--   K5  Bands     gain into the wavefolder: colour bands per base cycle (4 octaves)
--   K6  Colour    rotates the colouriser: which half of the palette the fold sweeps
--   S7  Shape     Round (sine) / Angular (triangle)
--   S8  Bookmatch Off / On: the H ramp mirrored about the screen centre
--   S9  Veins     Off / On: pale or dark lines at the band edges
--   S10 Colours   Stone / Neon
--   S11 Source    Video / Osc
--   P12 STIR      modulation depth: straight ruled bands .. deep folded marbling (square law)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, table fill), a line engine kicked
-- at each line start (computes the NEXT line), a 41-stage pixel path whose long waits run through
-- block-RAM delay rings (FF chains cost a logic cell per bit).  Virtual 1080-line coordinates (SD shows
-- the centre 4:3 of the picture).  Generated avid; output conditioning (Y, U and V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16) before the blanking gate.  Colours in the hardware
-- convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K5 = Colour (was K6); K5 Bands fixed at 512 (C_KFIX_5).
-- 2026-11-15: K6 camera top +25 % (Ron): gain s_cgq = 65 k >> 10 (1.25 x 13 k >> 7, one bit coarser), slices one bit lower.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.agate_pkg.all;
use work.all;

architecture agate of program_top is
    constant C_KFIX_5 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(512, 10));   -- K6 camera pass: fixed knob
    constant C_LAT : integer := 40;   -- = processing_delay_clks

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
    -- x step in half virtual px per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- the oscillator's y step = f_vstep_y * 121 / 256
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
    -- a + b (sub = '0') or a - b (sub = '1') on one carry chain
    function f_addsub(a : unsigned; b : unsigned; sub : std_logic) return unsigned is
        variable va, vb, vs : unsigned(a'length downto 0);
        variable vx : unsigned(a'length - 1 downto 0);
    begin
        vx := resize(b, a'length);
        if sub = '1' then vx := not vx; end if;
        va := a & '1';
        vb := vx & sub;
        vs := va + vb;
        return vs(a'length downto 1);
    end function;
    -- quarter-wave table address of phase (src << s) + p: bit 11 = sign, bit 10 = mirror
    function f_ph(src : unsigned(13 downto 0); s : integer; p : unsigned(19 downto 0)) return unsigned is
        variable v : unsigned(11 downto 0);
    begin
        case s is
            when 0      => v := src(13 downto 2);
            when 1      => v := src(12 downto 1);
            when 2      => v := src(11 downto 0);
            when others => v := src(10 downto 0) & '0';
        end case;
        return v + p(19 downto 8);
    end function;
    -- 13-bit phase of an interpolated table (s = 0 or 1): bit 12 = sign, 11 = mirror, 10..3 = entry, 2..0 = fraction
    function f_phi(src : unsigned(13 downto 0); s : integer; p : unsigned(19 downto 0)) return unsigned is
        variable v : unsigned(12 downto 0);
    begin
        if s = 0 then v := src(13 downto 1); else v := src(12 downto 0); end if;
        return v + p(19 downto 7);
    end function;
    function f_mir(a : unsigned(11 downto 0)) return unsigned is
    begin
        if a(10) = '1' then return not a(9 downto 2); else return a(9 downto 2); end if;
    end function;

    attribute no_rw_check : boolean;

    -- stage tables: amplitude * waveform, quarter wave at 256 integer phase steps; written in the vblank
    -- zone only, read by the pixel path.  Tables 1, 2: {forward difference (8 bits, 6.2), value (16 bits,
    -- 14.2)}, interpolated.  Tables 3, 4: the value (14 bits).
    type t_tab is array (0 to 255) of unsigned(15 downto 0);
    type t_tab32 is array (0 to 255) of std_logic_vector(31 downto 0);
    signal t1, t2 : t_tab32 := (others => x"00000000");
    signal t3, t4 : t_tab := (others => x"0000");
    attribute no_rw_check of t1 : signal is true;
    attribute no_rw_check of t2 : signal is true;
    attribute no_rw_check of t3 : signal is true;
    attribute no_rw_check of t4 : signal is true;
    type t_a8 is array (3 to 4) of unsigned(7 downto 0);
    type t_a16 is array (3 to 4) of unsigned(15 downto 0);
    type t_a14 is array (3 to 4) of unsigned(13 downto 0);
    signal t_ra : t_a8 := (others => (others => '0'));
    signal t_q  : t_a16 := (others => (others => '0'));
    signal t_f  : t_a14 := (others => (others => '0'));
    signal t_sa, t_sb, t_sc : std_logic_vector(3 to 4) := (others => '0');
    signal t_we : std_logic_vector(3 to 4) := (others => '0');
    signal t_wa : unsigned(7 downto 0) := (others => '0');
    signal t_wd : unsigned(15 downto 0) := (others => '0');
    -- interpolated tables 1, 2
    type t_i8 is array (1 to 2) of unsigned(7 downto 0);
    type t_i4 is array (1 to 2) of unsigned(3 downto 0);
    type t_i32 is array (1 to 2) of std_logic_vector(31 downto 0);
    type t_i16 is array (1 to 2) of unsigned(15 downto 0);
    type t_i12 is array (1 to 2) of unsigned(11 downto 0);
    type t_i10 is array (1 to 2) of unsigned(9 downto 0);
    type t_i14 is array (1 to 2) of unsigned(13 downto 0);
    signal i_ra : t_i8 := (others => (others => '0'));
    signal i_m1, i_m2, i_m3 : t_i4 := (others => (others => '0'));
    signal i_s1, i_s2, i_s3, i_s4, i_s5 : std_logic_vector(1 to 2) := (others => '0');
    signal i_q  : t_i32 := (others => (others => '0'));
    signal i_v, i_v4 : t_i16 := (others => (others => '0'));
    signal i_d  : t_i8 := (others => (others => '0'));
    signal i_ph : t_i12 := (others => (others => '0'));
    signal i_pl : t_i10 := (others => (others => '0'));
    signal i_val : t_i14 := (others => (others => '0'));
    signal i_we : std_logic_vector(1 to 2) := (others => '0');
    signal i_wa : unsigned(7 downto 0) := (others => '0');
    signal i_wd : std_logic_vector(31 downto 0) := (others => '0');

    -- luma store: 12 bits (8.4), an eighth of the horizontal rate, vertical one-pole; each entry is
    -- read for its eight pixels and rewritten by the last of them
    type t_lb is array (0 to 255) of unsigned(15 downto 0);
    signal lb : t_lb := (others => x"0000");
    attribute no_rw_check of lb : signal is true;
    signal l_ra, l_ra1, l_ra2, l_wa : unsigned(7 downto 0) := (others => '0');
    signal l_q  : unsigned(15 downto 0) := (others => '0');
    signal l_f  : unsigned(11 downto 0) := (others => '0');
    signal l_wd : unsigned(15 downto 0) := (others => '0');
    signal l_we : std_logic := '0';
    signal l_p10, l_p11, l_p12 : std_logic := '0';
    signal c9   : unsigned(10 downto 0) := (others => '0');
    signal s13_l : unsigned(15 downto 0) := (others => '0');

    -- colouriser phase store (the line above, a quarter of the horizontal rate) for the vertical
    -- footprint; each entry is read for its four pixels and rewritten by the last of them
    type t_ub is array (0 to 511) of unsigned(15 downto 0);
    signal ub : t_ub := (others => x"0000");
    attribute no_rw_check of ub : signal is true;
    signal u_ra, u_ra1, u_ra2, u_wa : unsigned(8 downto 0) := (others => '0');
    signal u_q  : unsigned(15 downto 0) := (others => '0');
    signal u_f  : unsigned(11 downto 0) := (others => '0');
    signal u_wd : unsigned(15 downto 0) := (others => '0');
    signal u_we : std_logic := '0';
    signal u_e23, u_e24, u_e25 : std_logic := '0';
    signal c22  : unsigned(10 downto 0) := (others => '0');

    -- block-RAM delay rings on one free-running write pointer: written every clock, read N back, RAM
    -- register, fabric register = N + 2 clocks (never the write address)
    type t_ring4 is array (0 to 255) of std_logic_vector(3 downto 0);
    type t_ring16 is array (0 to 255) of std_logic_vector(15 downto 0);
    type t_ring32 is array (0 to 255) of std_logic_vector(31 downto 0);
    signal r_sync : t_ring4 := (others => (others => '1'));       -- hsync, vsync, field: C_LAT
    signal r_yr   : t_ring16 := (others => (others => '0'));      -- Y of stage 7 -> 17
    signal r_xr   : t_ring16 := (others => (others => '0'));      -- X (low 16 bits) of stage 14 -> 21
    signal r_ca   : t_ring32 := (others => (others => '0'));      -- box A input, 8 back
    signal r_cb   : t_ring32 := (others => (others => '0'));      -- box B input, 8 back
    attribute no_rw_check of r_sync : signal is true;
    attribute no_rw_check of r_yr : signal is true;
    attribute no_rw_check of r_xr : signal is true;
    attribute no_rw_check of r_ca : signal is true;
    attribute no_rw_check of r_cb : signal is true;
    signal r_wp   : unsigned(7 downto 0) := (others => '0');
    signal r_syq, r_syq2 : std_logic_vector(3 downto 0) := (others => '1');
    signal r_yq, r_yq2, r_xq, r_xq2 : std_logic_vector(15 downto 0) := (others => '0');
    signal r_caq, r_caq2, r_cbq, r_cbq2 : std_logic_vector(31 downto 0) := (others => '0');
    type t_xhi is array (15 to 21) of unsigned(1 downto 0);
    signal x_hi : t_xhi := (others => (others => '0'));          -- X's top two bits of stage 15 .. 21

    -- ROMs (registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_sq_a : unsigned(7 downto 0) := (others => '0');
    signal r_sq_q : unsigned(15 downto 0) := (others => '0');
    signal r_sh_a : unsigned(8 downto 0) := (others => '0');
    signal r_sh_q, r_sh_f : std_logic_vector(31 downto 0) := (others => '0');
    signal r_sf_a : unsigned(7 downto 0) := (others => '0');
    signal r_sf_q, r_sf_f : std_logic_vector(31 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_l8  : unsigned(7 downto 0) := (others => '0');
    signal s_avqq : std_logic := '0';
    signal s_h1, s_h2 : unsigned(13 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(11 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);   -- K6 camera pass: glided K6 Camera
    signal s_cgq : unsigned(6 downto 0) := to_unsigned(32, 7);         -- camera gain, 65 k / 1024 (K6 top +25 %; was 13 k / 128)
    signal l_v13 : unsigned(11 downto 0) := (others => '0');
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(0, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(800 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_tri, s_bm, s_vein, s_pal, s_src : std_logic := '0';

    -- per-field constants
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_a    : unsigned(19 downto 0) := (others => '0');
    signal c_fh   : unsigned(16 downto 0) := to_unsigned(26214, 17);   -- cycles per half virtual px, Q26
    signal c_dx   : unsigned(18 downto 0) := to_unsigned(52428, 19);   -- cycles per pixel, Q26
    signal c_xst  : unsigned(29 downto 0) := (others => '0');          -- X of pixel 0 before the line term
    signal c_half : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal c_dy   : unsigned(25 downto 0) := (others => '0');          -- cycles per field line, Q26
    signal c_y0   : unsigned(25 downto 0) := (others => '0');
    signal c_a0, c_a1, c_a2 : unsigned(15 downto 0) := (others => '0');
    signal c_r    : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal c_lx   : unsigned(15 downto 0) := to_unsigned(39423, 16);
    signal c_ly   : unsigned(15 downto 0) := to_unsigned(39219, 16);
    type t_amp is array (0 to 4) of unsigned(11 downto 0);
    signal c_amp  : t_amp := (others => (others => '0'));
    constant c_g  : unsigned(11 downto 0) := to_unsigned(512, 12);     -- Bands gain, Q8 (K5 Bands fixed at 512)
    signal c_rot  : unsigned(7 downto 0) := (others => '0');
    signal c_my, c_mu, c_mv : unsigned(9 downto 0) := to_unsigned(512, 10);   -- the window's mean colour
    signal c_d    : unsigned(13 downto 0) := (others => '0');          -- drift per field (speed^2 / 16)
    signal c_d1, c_d2, c_d3, c_d4 : unsigned(14 downto 0) := (others => '0');
    signal p0 : unsigned(19 downto 0) := to_unsigned(136315, 20);
    signal p1 : unsigned(19 downto 0) := to_unsigned(744489, 20);
    signal p2 : unsigned(19 downto 0) := to_unsigned(440402, 20);
    signal p3 : unsigned(19 downto 0) := to_unsigned(52428, 20);
    signal p4 : unsigned(19 downto 0) := to_unsigned(922747, 20);

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- table fill (vblank zone): four tables x 257 entries, one entry per three clocks (the amplitude a
    -- nibble at a time through one 16 x 4 multiplier)
    signal f_run  : std_logic := '0';
    signal f_cy   : unsigned(1 downto 0) := (others => '0');
    signal f_t, f_t1, f_t2 : unsigned(1 downto 0) := (others => '0');
    signal f_i, f_i1, f_i2 : unsigned(8 downto 0) := (others => '0');
    signal f_v1, f_v2 : std_logic := '0';
    signal f_wq   : unsigned(15 downto 0) := (others => '0');
    signal f_am   : unsigned(11 downto 0) := (others => '0');
    signal f_pp   : unsigned(19 downto 0) := (others => '0');
    signal f_ac   : unsigned(20 downto 0) := (others => '0');
    signal f_vx, f_prev, f_df : unsigned(15 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine (stage 0's X term for the next line, on its own serial multiplier)
    signal e_lc   : unsigned(5 downto 0) := to_unsigned(63, 6);
    signal e_zk   : std_logic := '0';
    signal e_y    : unsigned(25 downto 0) := (others => '0');
    signal e_ia, e_j : unsigned(7 downto 0) := (others => '0');
    signal e_m    : unsigned(4 downto 0) := (others => '0');
    signal e_sg, e_top : std_logic := '0';
    signal e_s0, e_s1 : unsigned(15 downto 0) := (others => '0');
    signal m_a    : unsigned(27 downto 0) := (others => '0');
    signal m_b    : unsigned(11 downto 0) := (others => '0');
    signal m_acc  : unsigned(27 downto 0) := (others => '0');
    signal n_m    : unsigned(13 downto 0) := (others => '0');
    signal n_x0   : unsigned(29 downto 0) := (others => '0');
    signal n_y    : unsigned(13 downto 0) := (others => '0');
    signal n_first, l_first : std_logic := '0';

    -- beam (stage 1) and the same ramp again at stage 12
    signal b_x, b2_x : unsigned(29 downto 0) := (others => '0');
    signal b_hc, b2_hc : unsigned(10 downto 0) := (others => '0');
    signal b_ph, b2_ph : unsigned(1 downto 0) := (others => '0');
    signal l_y   : unsigned(13 downto 0) := (others => '0');

    -- pixel path
    signal py7, py18 : unsigned(13 downto 0) := (others => '0');
    signal px13, px14, px22 : unsigned(17 downto 0) := (others => '0');
    signal q23_a, q23_b, q23_c : unsigned(23 downto 0) := (others => '0');
    signal q24_s, q24_c : unsigned(23 downto 0) := (others => '0');
    signal q25_p, q26_p : unsigned(11 downto 0) := (others => '0');
    signal q26_dh, q26_dv : unsigned(11 downto 0) := (others => '0');
    signal q26_f  : std_logic := '0';
    signal q26_i, q27_i : unsigned(7 downto 0) := (others => '0');
    signal q28_i : unsigned(5 downto 0) := (others => '0');
    signal q27_dh, q27_dv : unsigned(11 downto 0) := (others => '0');
    signal q28_d  : unsigned(12 downto 0) := (others => '0');
    signal q29_w, q30_w, q31_w, q32_w : unsigned(3 downto 0) := (others => '0');
    signal q29_far, q30_far, q31_far : std_logic := '0';
    type t_s12 is array (0 to 2) of signed(11 downto 0);
    type t_u10c is array (0 to 2) of unsigned(9 downto 0);
    signal q31_s, q32_a : t_u10c := (others => (others => '0'));
    signal q32_d  : t_s12 := (others => (others => '0'));
    signal q33_s1, q33_t2, q34_c : t_s12 := (others => (others => '0'));
    signal s_y35, s_u35, s_v35 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 inputs of boxes A, B
    signal s_y37, s_u37, s_v37 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y39, s_u39, s_v39 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y40, s_u40, s_v40 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y41 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u41, s_v41 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories: stage tables, luma store, colouriser phase store, ROMs, delay rings
    ---------------------------------------------------------------------------
    p_tab : process(clk)
    begin
        if rising_edge(clk) then
            i_q(1) <= t1(to_integer(i_ra(1)));
            if i_we(1) = '1' then t1(to_integer(i_wa)) <= i_wd; end if;
            i_q(2) <= t2(to_integer(i_ra(2)));
            if i_we(2) = '1' then t2(to_integer(i_wa)) <= i_wd; end if;
            t_q(3) <= t3(to_integer(t_ra(3)));
            if t_we(3) = '1' then t3(to_integer(t_wa)) <= t_wd; end if;
            t_q(4) <= t4(to_integer(t_ra(4)));
            if t_we(4) = '1' then t4(to_integer(t_wa)) <= t_wd; end if;
        end if;
    end process p_tab;

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            l_q <= lb(to_integer(l_ra));
            if l_we = '1' then lb(to_integer(l_wa)) <= l_wd; end if;
            u_q <= ub(to_integer(u_ra));
            if u_we = '1' then ub(to_integer(u_wa)) <= u_wd; end if;
        end if;
    end process p_lb;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_AG_EXP2(to_integer(r_ex_a));
            r_sq_q <= C_AG_SINI(to_integer(r_sq_a));
            r_sh_q <= C_AG_SHARP(to_integer(r_sh_a));
            r_sf_q <= C_AG_SOFT(to_integer(r_sf_a));
        end if;
    end process p_rom;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            r_wp <= r_wp + 1;
            -- syncs: C_LAT = 38 back + RAM register + fabric register
            r_sync(to_integer(r_wp)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            r_syq <= r_sync(to_integer(r_wp - (C_LAT - 2)));
            r_syq2 <= r_syq;
            -- Y of stage 7 -> stage 17 (10 clocks)
            r_yr(to_integer(r_wp)) <= "00" & std_logic_vector(py7);
            r_yq <= r_yr(to_integer(r_wp - 8));
            r_yq2 <= r_yq;
            -- X of stage 14 -> stage 21 (7 clocks)
            r_xr(to_integer(r_wp)) <= std_logic_vector(px14(15 downto 0));
            r_xq <= r_xr(to_integer(r_wp - 5));
            r_xq2 <= r_xq;
            -- the two box filters' inputs of 8 clocks before
            r_ca(to_integer(r_wp)) <= "00" & std_logic_vector(s_y35) & std_logic_vector(s_u35) & std_logic_vector(s_v35);
            r_caq <= r_ca(to_integer(r_wp - 6));
            r_caq2 <= r_caq;
            r_cb(to_integer(r_wp)) <= "00" & std_logic_vector(s_y37) & std_logic_vector(s_u37) & std_logic_vector(s_v37);
            r_cbq <= r_cb(to_integer(r_wp - 6));
            r_cbq2 <= r_cbq;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(10 downto 0);
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
            elsif s_zc /= 4095 then
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
                s_tri  <= registers_in(6)(0);
                s_bm   <= registers_in(6)(1);
                s_vein <= registers_in(6)(2);
                s_pal  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            c_half <= s_w(11 downto 1);
            s_oys <= f_oys(s_tid);

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
                            when 3      => s_gm <= s_g5; s_gr <= unsigned(registers_in(5));   -- K6 Camera
                            when 4      => s_gm <= s_g6; s_gr <= unsigned(registers_in(4));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                            if z_gc = 3 and s_gd > -33 and s_gd < 33 then s_gd <= (others => '0'); end if;   -- camera: +-4
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from
            -- load + 17 (stores at load + 18).  The slot number is decoded at 30 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 31 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 390 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 414 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 438 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when 462 => z_sel <= to_unsigned(18, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(26214, 36);              z_fb <= z_mt;                        -- fh
                    when 1  => z_fa <= resize(s_gp(12 downto 3), 36);       z_fb <= resize(s_gp(12 downto 3), 16); -- stir^2
                    when 2  => z_fa <= resize(z_a, 36);                     z_fb <= to_unsigned(40718, 16);      -- A
                    when 3  => z_fa <= resize(s_g3(12 downto 3), 36);       z_fb <= to_unsigned(32, 16);         -- detail ratio
                    when 4  => z_fa <= resize(c_a0, 36);                    z_fb <= c_r;
                    when 5  => z_fa <= resize(c_a1, 36);                    z_fb <= c_r;
                    when 7  => z_fa <= resize(s_g2(12 downto 3), 36);       z_fb <= to_unsigned(51, 16);         -- lean
                    when 8  => z_fa <= resize(c_a0, 36);                    z_fb <= c_lx;
                    when 9  => z_fa <= resize(c_a0, 36);                    z_fb <= c_ly;
                    when 10 => z_fa <= resize(c_a1, 36);                    z_fb <= c_lx;
                    when 11 => z_fa <= resize(c_a1, 36);                    z_fb <= c_ly;
                    when 12 => z_fa <= resize(c_a2, 36);                    z_fb <= c_lx;
                    when 13 => z_fa <= resize(c_a2, 36);                    z_fb <= c_ly;
                    when 15 => z_fa <= resize(c_dx, 36);                    z_fb <= resize(g_wm1, 16);           -- (W - 1) dx
                    when 16 => z_fa <= resize(c_fh, 36);                    z_fb <= resize(s_ys, 16);            -- dy
                    when 17 => z_fa <= resize(c_fh, 36);                    z_fb <= to_unsigned(1080, 16);       -- centre line
                    when others => z_fa <= resize(s_vm, 36);                z_fb <= resize(s_vm, 16);            -- speed^2
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion: every stage phase drifts at its own rate (last field's steps)
                    when 20 =>
                        p0 <= f_addsub(p0, resize(c_d, 20), s_vneg);
                        p1 <= f_addsub(p1, resize(c_d1, 20), not s_vneg);
                        p2 <= f_addsub(p2, resize(c_d2, 20), s_vneg);
                        p3 <= f_addsub(p3, resize(c_d3, 20), not s_vneg);
                        p4 <= f_addsub(p4, resize(c_d4, 20), s_vneg);
                    -- Scale exponent = 0.75 (k1 - 512): +-1.5 octaves
                    when 26 =>
                        v_e := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        v_e := v_e - shift_right(v_e, 2);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                        c_rot <= s_g6(12 downto 5);
                    when 28 => z_mt <= r_ex_q;
                    -- (K6 camera pass: Bands is fixed at its default, gain 2.0 = the constant c_g)
                    when 31 =>
                        -- the Scale shift again, for slot 0's store
                        v_e := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        v_e := v_e - shift_right(v_e, 2);
                        z_sh <= v_e(10 downto 8);
                    when 40 =>
                        c_my <= unsigned(r_sf_f(31 downto 22)); c_mu <= unsigned(r_sf_f(21 downto 12)); c_mv <= unsigned(r_sf_f(11 downto 2));
                    when 50 =>                        -- fh = 26214 * mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh) is
                            when -2     => c_fh <= resize(f_acc(30 downto 17), 17);
                            when -1     => c_fh <= resize(f_acc(30 downto 16), 17);
                            when 0      => c_fh <= resize(f_acc(30 downto 15), 17);
                            when others => c_fh <= f_acc(30 downto 14);
                        end case;
                    when 51 =>
                        case to_integer(s_xs) is
                            when 4      => c_dx <= c_fh & "00";
                            when 3      => c_dx <= resize(c_fh & '0', 19) + resize(c_fh, 19);
                            when others => c_dx <= resize(c_fh & '0', 19);
                        end case;
                    when 74  => z_a <= f_acc(19 downto 0);
                    when 98  => c_a0 <= f_acc(35 downto 20);
                    when 122 => c_r <= to_unsigned(16384, 16) + f_acc(15 downto 0);
                    when 146 => c_a1 <= f_acc(31 downto 16);
                    when 170 => c_a2 <= f_acc(31 downto 16);
                    when 218 =>
                        c_lx <= to_unsigned(65535, 16) - f_acc(15 downto 0);
                        c_ly <= to_unsigned(13107, 16) + f_acc(15 downto 0);
                    when 242 => c_amp(0) <= f_acc(31 downto 20);
                    when 266 => c_amp(1) <= f_acc(31 downto 20);
                    when 290 => c_amp(2) <= f_acc(31 downto 20);
                    when 314 => c_amp(3) <= f_acc(31 downto 20);
                    when 338 => c_amp(4) <= f_acc(31 downto 20);
                    when 410 =>                       -- X of pixel 0: -(W - 1) dx / 2, or + for the bookmatch
                        if s_bm = '1' then c_xst <= f_acc(30 downto 1); else c_xst <= (not f_acc(30 downto 1)) + 1; end if;
                    when 434 => c_dy <= resize(f_acc(28 downto 7), 26);
                    when 458 => c_y0 <= resize(c_dy(25 downto 1), 26) - f_acc(25 downto 0);
                    when 482 => c_d <= f_acc(17 downto 4);
                    when 483 =>
                        c_d1 <= resize(c_d, 15) - resize(c_d(13 downto 2), 15);
                        c_d2 <= resize(c_d, 15) + resize(c_d(13 downto 2), 15);
                        c_d3 <= resize(c_d(13 downto 1), 15) + resize(c_d(13 downto 3), 15);
                        c_d4 <= resize(c_d, 15) - resize(c_d(13 downto 3), 15);
                    when others => null;
                end case;
            end if;

            -- K6 camera gain from the glided knob (changes only in the vblank glide): 65 k >> 10 (0 .. 64) =
            -- 1.25 x the v0.1.1 13 k >> 7 at half the scale (K6 top +25 %; the slices below drop a bit)
            s_cgq <= resize(shift_right(shift_left(resize(s_g5(12 downto 3), 17), 6) + resize(s_g5(12 downto 3), 17), 10), 7);
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Table fill (vblank zone, zc 520 .. 3620): amplitude * waveform for the four pixel-path tables,
    -- entries 0 .. 256 of each (256 = the peak), one entry per three clocks: the 12-bit amplitude goes
    -- through one 16 x 4 multiplier a nibble at a time, low nibble first, the sum shifting right by four
    -- (= the product >> 8 exactly).  Waveform = the quarter sine ROM, or a linear quarter ramp (Angular).
    -- Tables 1, 2 get {entry i + 1 - entry i, entry i} (14.2); tables 3, 4 the value (14 bits).
    -- Entry i: ROM address in its own period, multiply in the next, writes in the one after.
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_n  : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 520 then
                f_run <= '1'; f_cy <= "00"; f_t <= (others => '0'); f_i <= (others => '0');
            elsif f_cy = 2 then
                f_cy <= "00";
                f_i1 <= f_i; f_t1 <= f_t; f_v1 <= f_run;
                f_i2 <= f_i1; f_t2 <= f_t1; f_v2 <= f_v1;
                if f_run = '1' then
                    if f_i = 256 then
                        f_i <= (others => '0'); f_t <= f_t + 1;
                        if f_t = 3 then f_run <= '0'; end if;
                    else
                        f_i <= f_i + 1;
                    end if;
                end if;
            else
                f_cy <= f_cy + 1;
            end if;
            -- ROM address (the line engine owns the port outside the fill); data two clocks later
            if f_run = '1' then r_sq_a <= f_i(7 downto 0); else r_sq_a <= e_ia; end if;
            -- end of the entry's own period: fabric copy / ramp / peak, amplitude of its table
            if f_cy = 2 then
                if f_i(8) = '1' then f_wq <= (others => '1');
                elsif s_tri = '1' then f_wq <= f_i(7 downto 0) & x"00";
                else f_wq <= r_sq_q; end if;
                case to_integer(f_t) is
                    when 0      => f_am <= c_amp(1);
                    when 1      => f_am <= c_amp(2);
                    when 2      => f_am <= c_amp(3);
                    when others => f_am <= c_amp(4);
                end case;
            end if;
            -- next period: one nibble per clock, registered, then accumulated
            case f_cy is
                when "00"   => v_n := f_am(3 downto 0);
                when "01"   => v_n := f_am(7 downto 4);
                when others => v_n := f_am(11 downto 8);
            end case;
            f_pp <= f_wq * v_n;
            if f_cy = 1 then f_ac <= resize(f_pp, 21);
            else f_ac <= resize(f_ac(20 downto 4), 21) + resize(f_pp, 21); end if;
            -- the period after: f_ac = product >> 8 during phase 1 (a plain table is written from it then);
            -- this entry and the one before it are in f_vx / f_prev during phase 2 (an interpolated table
            -- is written then)
            if f_cy = 1 then
                f_vx <= f_ac(19 downto 4);
                f_prev <= f_vx;
            end if;
        end if;
    end process p_fill;
    t_wa <= f_i2(7 downto 0);
    t_wd <= "00" & f_ac(19 downto 6);
    t_we(3) <= '1' when f_cy = 1 and f_v2 = '1' and f_i2(8) = '0' and f_t2 = 2 else '0';
    t_we(4) <= '1' when f_cy = 1 and f_v2 = '1' and f_i2(8) = '0' and f_t2 = 3 else '0';
    i_wa <= f_i2(7 downto 0) - 1;
    f_df <= f_vx - f_prev;
    i_wd <= x"00" & std_logic_vector(f_df(7 downto 0)) & std_logic_vector(f_prev);
    i_we(1) <= '1' when f_cy = 2 and f_v2 = '1' and f_i2 /= 0 and f_t2 = 0 else '0';
    i_we(2) <= '1' when f_cy = 2 and f_v2 = '1' and f_i2 /= 0 and f_t2 = 1 else '0';

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
    -- Source side: oscillator (2-D triangle lattice, drifting), luma tap
    -- (luma >> 2) - 16, two horizontal one-poles (1/16 per pixel, 8.6)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d, v_n : signed(14 downto 0);
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
            if s_src = '1' then
                s_l8 <= s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_l8 <= v_l - 16; else s_l8 <= (others => '0'); end if;
            end if;
            s_avqq <= s_avq;
            -- s_l8 is the luma of the source pixel flagged by s_avq
            if s_avq = '1' then
                if s_avqq = '0' then
                    s_h1 <= s_l8 & "000000"; s_h2 <= s_l8 & "000000";
                else
                    v_d := signed(resize(s_l8 & "000000", 15)) - signed(resize(s_h1, 15));
                    v_n := signed(resize(s_h1, 15)) + shift_right(v_d, 4);
                    s_h1 <= unsigned(v_n(13 downto 0));
                    v_d := signed(resize(s_h1, 15)) - signed(resize(s_h2, 15));
                    v_n := signed(resize(s_h2, 15)) + shift_right(v_d, 4);
                    s_h2 <= unsigned(v_n(13 downto 0));
                end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone, after the
    -- table fill, for line 0); computes the NEXT line's Y phase and stage 0's
    -- X term  ax_0 * w(Y + p0): two neighbouring quarter-wave entries, linear
    -- interpolation (4 fraction bits) and the amplitude, on one serial multiplier
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_a : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 3700 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_y <= c_y0;
                e_lc <= (others => '0');
                n_first <= '1';
            elsif g_start = '1' then
                e_y <= e_y + c_dy;
                e_lc <= (others => '0');
                n_first <= '0';
            elsif e_lc /= 63 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(26 downto 0) & '0';
            m_b <= '0' & m_b(11 downto 1);

            case to_integer(e_lc) is
                when 1 =>                     -- phase: sign, mirror, entry (8), fraction (4)
                    v_a := e_y(25 downto 10) + p0(19 downto 4);
                    e_sg <= v_a(15);
                    if v_a(14) = '1' then
                        e_ia <= not v_a(13 downto 6); e_j <= not v_a(13 downto 6);
                        e_m <= to_unsigned(16, 5) - resize(v_a(5 downto 2), 5);
                        if v_a(13 downto 6) = 0 then e_top <= '1'; else e_top <= '0'; end if;
                    else
                        e_ia <= v_a(13 downto 6); e_j <= v_a(13 downto 6);
                        e_m <= resize(v_a(5 downto 2), 5);
                        if v_a(13 downto 6) = 255 then e_top <= '1'; else e_top <= '0'; end if;
                    end if;
                when 2 => e_ia <= e_ia + 1;   -- entry j is on the ROM address now, j + 1 next
                when 4 =>
                    if s_tri = '1' then e_s0 <= e_j & x"00"; else e_s0 <= r_sq_q; end if;
                when 5 =>
                    if e_top = '1' then e_s1 <= (others => '1');
                    elsif s_tri = '1' then e_s1 <= e_ia & x"00";
                    else e_s1 <= r_sq_q; end if;
                when 7 =>                     -- (entry j + 1 - entry j) * fraction: 5 iterations
                    m_a <= resize(e_s1 - e_s0, 28); m_b <= resize(e_m, 12); m_acc <= (others => '0');
                when 13 =>                    -- interpolated waveform * amplitude: 12 iterations
                    m_a <= resize(e_s0 + m_acc(19 downto 4), 28); m_b <= c_amp(0); m_acc <= (others => '0');
                when 27 =>
                    n_m <= m_acc(27 downto 14);
                when 28 =>
                    n_x0 <= f_addsub(c_xst, resize(n_m & x"000", 30), e_sg);
                    n_y <= e_y(25 downto 12);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1): X = the H ramp plus the line's stage-0 term.  Bookmatch: the ramp runs
    -- down to the centre, holds one pixel and runs up again (|x - centre|).  A second copy of the
    -- ramp runs 11 clocks behind (X of the stage-12 pixel), started and stepped by the generated avid
    -- of that stage: cheaper than carrying X through eleven registers.  (n_x0 is rewritten for the
    -- next line 29 clocks after the line start, so both copies load the same value.)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= n_x0;
                b_hc <= c_half - 1;
                if s_bm = '1' then b_ph <= "00"; else b_ph <= "10"; end if;
                l_y  <= n_y;
                l_first <= n_first;
            elsif g_avid = '1' then
                if b_ph /= "01" then b_x <= f_addsub(b_x, resize(c_dx, 30), not b_ph(1)); end if;
                if b_ph = "00" then
                    b_hc <= b_hc - 1;
                    if b_hc = 1 then b_ph <= "01"; end if;
                elsif b_ph = "01" then
                    b_ph <= "10";
                end if;
            end if;
            if s_gav_sr(9) = '1' and s_gav_sr(10) = '0' then
                b2_x  <= n_x0;
                b2_hc <= c_half - 1;
                if s_bm = '1' then b2_ph <= "00"; else b2_ph <= "10"; end if;
            elsif s_gav_sr(10) = '1' then
                if b2_ph /= "01" then b2_x <= f_addsub(b2_x, resize(c_dx, 30), not b2_ph(1)); end if;
                if b2_ph = "00" then
                    b2_hc <= b2_hc - 1;
                    if b2_hc = 1 then b2_ph <= "01"; end if;
                elsif b2_ph = "01" then
                    b2_ph <= "10";
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path.  Stage 1 = the beam values (b_x, l_y).  A plain cross-modulation step takes four
    -- stages (table address, block RAM, fabric copy, add); an interpolated one six (address, block
    -- RAM, fabric copy, fraction * difference, value, add).  X and Y wait for each other in the delay
    -- rings (p_ring), not in registers.
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a  : unsigned(11 downto 0);
        variable v_b  : unsigned(12 downto 0);
        variable v_x  : unsigned(23 downto 0);
        variable v_x21 : unsigned(17 downto 0);
        variable v_l  : unsigned(11 downto 0);
        variable v_mp : unsigned(18 downto 0);
        variable v_d  : signed(12 downto 0);
        variable v_w2 : unsigned(12 downto 0);
        variable v_g1, v_g2 : unsigned(11 downto 0);
        variable v_s19 : unsigned(18 downto 0);
        variable v_sh, v_vc : t_u10c;
        variable v_ca, v_cb, v_sf : t_u10c;
        variable v_dv : signed(11 downto 0);
        variable v_t1, v_t2 : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            for k in 3 to 4 loop
                t_f(k) <= t_q(k)(13 downto 0);
                t_sb(k) <= t_sa(k); t_sc(k) <= t_sb(k);
            end loop;
            for k in 1 to 2 loop
                -- fabric copy; fraction (0 .. 8) * difference as two partial sums; value = (8 v + m d) / 32
                i_v(k) <= unsigned(i_q(k)(15 downto 0)); i_d(k) <= unsigned(i_q(k)(23 downto 16));
                i_m2(k) <= i_m1(k); i_m3(k) <= i_m2(k);
                i_s2(k) <= i_s1(k); i_s3(k) <= i_s2(k); i_s4(k) <= i_s3(k); i_s5(k) <= i_s4(k);
                v_g1 := (others => '0'); v_g2 := (others => '0');
                if i_m3(k)(3) = '1' then v_g1 := resize(i_d(k) & "000", 12); end if;
                if i_m3(k)(2) = '1' then v_g2 := resize(i_d(k) & "00", 12); end if;
                i_ph(k) <= v_g1 + v_g2;
                v_g1 := (others => '0'); v_g2 := (others => '0');
                if i_m3(k)(1) = '1' then v_g1 := resize(i_d(k) & '0', 12); end if;
                if i_m3(k)(0) = '1' then v_g2 := resize(i_d(k), 12); end if;
                i_pl(k) <= resize(v_g1 + v_g2, 10);
                i_v4(k) <= i_v(k);
                v_s19 := (i_v4(k) & "000") + resize(i_ph(k), 19) + resize(i_pl(k), 19);
                i_val(k) <= v_s19(18 downto 5);
            end loop;

            -- Y stage 0: table 1 by X, interpolated (stage 1 -> 7); Y before it is the line's constant
            v_b := f_phi(b_x(25 downto 12), 0, p1);
            i_s1(1) <= v_b(12);
            if v_b(11) = '1' then i_ra(1) <= not v_b(10 downto 3); i_m1(1) <= "1000" - ('0' & v_b(2 downto 0));
            else i_ra(1) <= v_b(10 downto 3); i_m1(1) <= '0' & v_b(2 downto 0); end if;
            py7 <= f_addsub(l_y, i_val(1), i_s5(1));
            -- X stage 1: table 2 by Y, interpolated (7 -> 13); X of stage 12 is the second ramp
            v_b := f_phi(py7, 1, p2);
            i_s1(2) <= v_b(12);
            if v_b(11) = '1' then i_ra(2) <= not v_b(10 downto 3); i_m1(2) <= "1000" - ('0' & v_b(2 downto 0));
            else i_ra(2) <= v_b(10 downto 3); i_m1(2) <= '0' & v_b(2 downto 0); end if;
            px13 <= f_addsub(b2_x(29 downto 12), resize(i_val(2), 18), i_s5(2));
            -- the picture: luma (12 bits, 8.4) * 13 / 16, four times that for the oscillator (13 -> 14)
            px14 <= px13 + s13_l;
            x_hi(15) <= px14(17 downto 16);
            for i in 16 to 21 loop x_hi(i) <= x_hi(i - 1); end loop;
            -- Y stage 1: table 3 by X (14 -> 18); Y of stage 17 comes out of its ring
            v_a := f_ph(px14(13 downto 0), 1, p3);
            t_ra(3) <= f_mir(v_a); t_sa(3) <= v_a(11);
            py18 <= f_addsub(unsigned(r_yq2(13 downto 0)), t_f(3), t_sc(3));
            -- X stage 2: table 4 by Y (18 -> 22); X of stage 21 comes out of its ring
            v_a := f_ph(py18, 2, p4);
            t_ra(4) <= f_mir(v_a); t_sa(4) <= v_a(11);
            v_x21(17 downto 16) := x_hi(21); v_x21(15 downto 0) := unsigned(r_xq2);
            px22 <= f_addsub(v_x21, resize(t_f(4), 18), t_sc(4));

            -- luma store: address at 10 (column / 8 of the stage-9 pixel), data 11, fabric copy 12; at 13
            -- the value used (the lines above; the current line on the first line of a field) times 13/16,
            -- and the vertical one-pole's new value (1/16 of the gap to the current line), written by the
            -- last pixel of each eight
            if s_gav_sr(7) = '1' then c9 <= c9 + 1; else c9 <= (others => '0'); end if;
            l_ra <= c9(10 downto 3);
            if c9(2 downto 0) = "111" then l_p10 <= '1'; else l_p10 <= '0'; end if;
            l_ra1 <= l_ra; l_p11 <= l_p10;
            l_ra2 <= l_ra1; l_p12 <= l_p11;
            l_f <= l_q(11 downto 0);
            if l_first = '1' then
                v_l := s_h2(13 downto 2);
                l_wd <= "0000" & s_h2(13 downto 2);
            else
                v_l := l_f;
                v_d := signed(resize(s_h2(13 downto 2), 13)) - signed(resize(l_f, 13));
                v_d := signed(resize(l_f, 13)) + shift_right(v_d, 4);
                l_wd <= "0000" & unsigned(v_d(11 downto 0));
            end if;
            l_wa <= l_ra2;
            l_we <= s_gav_sr(10) and l_p12;
            -- K6 camera pass: luma x 65 k / 16384 in its own stage (one pixel later): 25 % = x1 (x13/16 before
            -- the +25 %), 50 % = x2.5, 100 % = x5; four times that for the oscillator.  v_mp <= 4080 x 64 < 2^18
            l_v13 <= v_l;
            v_mp := l_v13 * s_cgq;
            if s_src = '1' then s13_l <= v_mp(17 downto 2); else s13_l <= resize(v_mp(17 downto 4), 16); end if;

            -- 23..25: U = X * Bands (mod 4 colouriser cycles), 1024 per cycle
            v_x := unsigned(resize(signed(px22), 24));
            q23_a <= resize(v_x * c_g(3 downto 0), 24);
            q23_b <= resize(v_x(19 downto 0) * c_g(7 downto 4), 20) & "0000";
            q23_c <= resize(v_x(15 downto 0) * c_g(11 downto 8), 16) & x"00";
            q24_s <= q23_a + q23_b; q24_c <= q23_c;
            v_x := q24_s + q24_c;
            q25_p <= v_x(23 downto 12);

            -- colouriser phase store: address at 23 (column / 4), data 24, fabric copy 25; rewritten at 26
            -- by the last pixel of each four
            if s_gav_sr(20) = '1' then c22 <= c22 + 1; else c22 <= (others => '0'); end if;
            u_ra <= c22(10 downto 2);
            if c22(1 downto 0) = "11" then u_e23 <= '1'; else u_e23 <= '0'; end if;
            u_ra1 <= u_ra; u_e24 <= u_e23;
            u_ra2 <= u_ra1; u_e25 <= u_e24;
            u_f <= u_q(11 downto 0);
            u_wa <= u_ra2;
            u_wd <= "0000" & q25_p;
            u_we <= s_gav_sr(23) and u_e25;

            -- 26: steps to the pixel on the left and to the line above; wavefolder + Colour
            q26_p  <= q25_p;
            q26_dh <= q25_p - q26_p;
            q26_dv <= q25_p - u_f;
            q26_f  <= not s_gav_sr(24);
            if q25_p(9) = '1' then q26_i <= ('0' & not q25_p(8 downto 2)) + c_rot;
            else q26_i <= ('0' & q25_p(8 downto 2)) + c_rot; end if;
            -- 27: magnitudes (no left step on the first pixel, no up step on the first line)
            if q26_f = '1' then q27_dh <= (others => '0');
            elsif q26_dh(11) = '1' then q27_dh <= (not q26_dh) + 1;
            else q27_dh <= q26_dh; end if;
            if l_first = '1' then q27_dv <= (others => '0');
            elsif q26_dv(11) = '1' then q27_dv <= (not q26_dv) + 1;
            else q27_dv <= q26_dv; end if;
            q27_i <= q26_i;
            -- 28: footprint; palette addresses (the soft ROM's mean entry is read in the zone)
            q28_d <= resize(q27_dh, 13) + resize(q27_dv, 13);
            r_sh_a(8) <= s_pal; r_sh_a(7 downto 0) <= q27_i;
            q28_i <= q27_i(7 downto 2);
            -- (the soft ROM runs one stage behind the sharp one: its colour is first needed at 32)
            r_sf_a(7) <= s_pal; r_sf_a(6) <= s_zone;
            if s_zone = '1' then r_sf_a(5 downto 0) <= c_rot(7 downto 2) + 16;
            else r_sf_a(5 downto 0) <= q28_i; end if;
            -- 29: ROM data; blend weight: sharp -> soft over 0 .. 4 entries per pixel, soft -> mean over 4 .. 36
            if q28_d(12 downto 5) /= 0 then
                q29_far <= '1';
                v_w2 := q28_d - 16;
                if v_w2(12 downto 7) /= 0 then q29_w <= "1000"; else q29_w <= '0' & v_w2(6 downto 4); end if;
            else
                q29_far <= '0';
                if q28_d(4) = '1' then q29_w <= "1000"; else q29_w <= q28_d(4 downto 1); end if;
            end if;
            -- 30: fabric copies
            r_sh_f <= r_sh_q; r_sf_f <= r_sf_q;     -- (soft: stage 31)
            q30_w <= q29_w; q30_far <= q29_far;
            -- 31: veins: the sharp colour blended to the pale / dark line by the entry's coverage
            v_sh(0) := unsigned(r_sh_f(31 downto 22));
            v_sh(1) := unsigned(r_sh_f(21 downto 13)) & '0';
            v_sh(2) := unsigned(r_sh_f(12 downto 4)) & '0';
            if r_sh_f(3) = '1' then
                v_vc := (to_unsigned(C_AG_VDY, 10), to_unsigned(C_AG_VDU, 10), to_unsigned(C_AG_VDV, 10));
            else
                v_vc := (to_unsigned(C_AG_VPY, 10), to_unsigned(C_AG_VPU, 10), to_unsigned(C_AG_VPV, 10));
            end if;
            for c in 0 to 2 loop
                v_dv := signed(resize(v_vc(c), 12)) - signed(resize(v_sh(c), 12));
                if s_vein = '0' or r_sh_f(2 downto 1) = "00" then v_dv := (others => '0');
                elsif r_sh_f(2 downto 1) = "01" then v_dv := shift_right(v_dv, 2);
                elsif r_sh_f(2 downto 1) = "10" then v_dv := shift_right(v_dv, 1);
                end if;
                v_dv := signed(resize(v_sh(c), 12)) + v_dv;
                q31_s(c) <= unsigned(v_dv(9 downto 0));
            end loop;
            q31_w <= q30_w; q31_far <= q30_far;
            -- 32: operands and their difference
            v_sf := (unsigned(r_sf_f(31 downto 22)), unsigned(r_sf_f(21 downto 12)), unsigned(r_sf_f(11 downto 2)));
            if q31_far = '1' then
                v_ca := v_sf; v_cb := (c_my, c_mu, c_mv);
            else
                v_ca := q31_s; v_cb := v_sf;
            end if;
            q32_w <= q31_w;
            for c in 0 to 2 loop
                q32_a(c) <= v_ca(c);
                q32_d(c) <= signed(resize(v_cb(c), 12)) - signed(resize(v_ca(c), 12));
                -- 33: weight / 8 as gated shifts: the operand plus the two large terms, and the two small ones
                v_t1 := (others => '0'); v_t2 := (others => '0');
                if q32_w(3) = '1' then v_t1 := q32_d(c); end if;
                if q32_w(2) = '1' then v_t2 := shift_right(q32_d(c), 1); end if;
                q33_s1(c) <= signed(resize(q32_a(c), 12)) + v_t1 + v_t2;
                v_t1 := (others => '0'); v_t2 := (others => '0');
                if q32_w(1) = '1' then v_t1 := shift_right(q32_d(c), 2); end if;
                if q32_w(0) = '1' then v_t2 := shift_right(q32_d(c), 3); end if;
                q33_t2(c) <= v_t1 + v_t2;
                -- 34: sum
                q34_c(c) <= q33_s1(c) + q33_t2(c);
            end loop;
            -- 35: the window's mean colour outside the generated avid
            if s_gav_sr(32) = '0' then
                s_y35 <= c_my; s_u35 <= c_mu; s_v35 <= c_mv;
            else
                s_y35 <= unsigned(q34_c(0)(9 downto 0)); s_u35 <= unsigned(q34_c(1)(9 downto 0)); s_v35 <= unsigned(q34_c(2)(9 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(32);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the rack's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16 and the blanking gate.  The 8-clock delays of
    -- both running sums are block-RAM rings (p_ring).
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to the mean colour (the off-avid input of both boxes) once
            -- 8 blank samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 36 sums, stage 37 output
            if s_bah = "00000000" then
                s_bay <= c_my & "000"; s_bau <= c_mu & "000"; s_bav <= c_mv & "000";
            else
                s_bay <= s_bay + s_y35 - unsigned(r_caq2(29 downto 20));
                s_bau <= s_bau + s_u35 - unsigned(r_caq2(19 downto 10));
                s_bav <= s_bav + s_v35 - unsigned(r_caq2(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: the mean colour off the generated avid (what box A gives there anyway)
            if s_gav_sr(34) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y37 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u37 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v37 <= v_sc(13 downto 4);
            else
                s_y37 <= c_my; s_u37 <= c_mu; s_v37 <= c_mv;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(34);
            -- box B: stage 38 sums, stage 39 output
            if s_bbh = "00000000" then
                s_bby <= c_my & "000"; s_bbu <= c_mu & "000"; s_bbv <= c_mv & "000";
            else
                s_bby <= s_bby + s_y37 - unsigned(r_cbq2(29 downto 20));
                s_bbu <= s_bbu + s_u37 - unsigned(r_cbq2(19 downto 10));
                s_bbv <= s_bbv + s_v37 - unsigned(r_cbq2(9 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y39 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u39 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v39 <= v_sc(13 downto 4);
            -- 40: luma 13/16
            if s_y39 > 64 then v_dl := s_y39 - 64; else v_dl := (others => '0'); end if;
            s_y40 <= s_y39 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u40 <= s_u39; s_v40 <= s_v39;
            -- 41: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y41 <= s_y40; s_u41 <= s_u40; s_v41 <= s_v40;
            else
                s_y41 <= to_unsigned(64, 10); s_u41 <= to_unsigned(512, 10); s_v41 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y41);
    data_out.u       <= std_logic_vector(s_u41);
    data_out.v       <= std_logic_vector(s_v41);
    data_out.hsync_n <= r_syq2(0);
    data_out.vsync_n <= r_syq2(1);
    data_out.field_n <= r_syq2(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture agate;
