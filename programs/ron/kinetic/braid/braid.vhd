-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Braid v0.1.0 -- Kinetic.  Plaited cord: vertical three-strand plaits standing side by side, each strand a
-- round lit tube (bright centre line, dark edges, a contact shadow where it dives under another), the three
-- strands in three palette colours with true over-and-under.  Art reference: three-strand braids, sennit,
-- cable-knit and Celtic plaitwork.
--
-- Element: a plait (column i, pitch P = 2H).  Strand s of plait i, on line y:
--   t_s   = psi_i(y) + s / 3                  (2 strands: s / 2)
--   x_s   = W sin(2 pi t_s)                   the centre, from the plait centre
--   z_s   = sin(4 pi t_s)                     the height (the figure-eight);  2 strands: cos(2 pi t_s)
--   rh_s  = R g(|cos 2 pi t_s|) + c_i         the horizontal half-width of the tilted tube, g = sqrt(1 + (A cos)^2),
--                                             plus the camera swell of the plait (and the Swell wave)
--   psi_i = K(P12) y / H + phase + Depth wave(phase + Spread index_i)      (state = the plait's phase: Wave = Slip)
-- At a pixel the visible strand is the highest (by z) of those whose |x - x_s| < rh_s; its shade comes from the
-- tube profile at e / rh_s (e = rh_s - |x - x_s|, the distance in from its edge), dimmed by the contact shadow
-- of the strand directly above it, and its edge is anti-aliased against what lies beneath (black where another
-- strand passes under it, else the ground).
--
--   K1  Scale    plait pitch 160 .. 640 virtual px (a dozen cords .. three fat ropes), exponential, glided
--   K2  Shape    strand thickness: thin cords with ground between .. fat strands that fill the plait
--   K3  Depth    wave depth, square law (Slip: 0 .. 1/2 braid cycle;  Swell: 0 .. +-2 R)
--   K4  Rate     wave travel and braid flow, bipolar, centre = still (60 s per cycle .. 2 cycles per second)
--   K5  Colour   the 16 shared Kinetic palettes (strands A, B, accent on the ground), hysteresis
--   K6  Camera   camera (and Osc) strength: the luma at each plait swells its strands; 0 = none
--   S7  Wave     Slip (the wave slides the plaits along their length) / Swell (the wave fattens and thins them)
--   S8  Field    Linear / Radial (mirror-symmetric about the centre line, the wave runs out both ways)
--   S9  Strands  3 (plait) / 2 (twisted rope)
--   S10 Lay      Parallel / Mirrored (alternate plaits mirrored: the chevron of a cable-knit)
--   S11 Source   Video / Osc
--   P12 PLAIT    braid amplitude and pitch together: nearly straight cords .. classic braid .. woven fabric
--
-- Sections (the integer model .kinetic_work/rb_braid/imodel.py mirrors each one):
--   [A] TIMING            timing-ID steps, vblank zone, width, generated avid, sync delay (riley's chassis)
--   [B] CONTROLS          glides, switches, rate deadband, palette hysteresis
--   [C] FIELD SEQUENCER   the vblank zone (no avid for 1024 clocks): every per-field constant on the shared
--                         pipelined 15 x 12 multiplier (polynomials, no divisions), the ground colour, the
--                         colour tables CTY / CTU / CTV from the palette's ramp words
--   [D] SOURCE            Osc lattice, luma -> horizontal 1/32 IIR -> sampled as the beam crosses each plait
--                         centre -> a vertical 1/16 IIR per plait (ping-pong state RAM)
--   [E] FILL + FETCH      per field, on the same multiplier: RT (R g and the AA ramp over |cos|, 256), WT (the
--                         camera swell by level, 256), D (the per-plait wave, 16); per line: slots 0 and 1 are
--                         fetched in the blanking after the line before, every further slot just in time as the
--                         beam enters the one before it (16 clocks: the three phases, SIN and RT reads, W |sin|
--                         on the multiplier) straight into the staging words
--   [F] BEAM + PIXEL      plait coordinate, slot words read ahead, edge distances, visibility, shadow, divider,
--                         profile, colour tables, AA blend
--   [G] OUTPUT            chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1]/4 at the same group delay, luma 13/16,
--                         blanking gate (riley's)
--
-- Stage numbering as riley: a register "at stage S" holds pixel n at T + S + n (T = the g_start clock); pixels
-- 0 .. W + 7 are rendered and output column j is the 9-tap filter over pixels j .. j + 8; the generated avid
-- lands 4 clocks after the delayed source avid.  One latency (40) in every mode.  Colours in the hardware
-- convention (u = Cr, v = Cb).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.braid_pkg.all;
use work.all;

architecture braid of program_top is
    constant C_LAT  : integer := 40;      -- = processing_delay_clks
    constant C_SNAP : boolean := false;   -- sim-only copies set this true: the glides snap to the knobs

    -- TOML defaults: the glides and the palette start there
    constant C_DEF_K1  : integer := 460;
    constant C_DEF_K2  : integer := 665;
    constant C_DEF_K3  : integer := 563;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 512;
    constant C_DEF_PAL : integer := 10;

    ---------------------------------------------------------------------------
    -- [A] timing-ID steps (riley)
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
    function f_addsub(a, b : unsigned; sub : std_logic) return unsigned is
        variable v_a, v_b, v_r : unsigned(a'length + 1 downto 0);
    begin
        v_a := '0' & a & '1';
        if sub = '1' then v_b := '0' & (not b) & '1'; else v_b := '0' & b & '0'; end if;
        v_r := v_a + v_b;
        return v_r(a'length downto 1);
    end function;
    -- quarter-wave index of t (16-bit cycles)
    function f_fold(t : unsigned(15 downto 0)) return unsigned is
    begin
        if t(14) = '1' then return not t(13 downto 4); else return t(13 downto 4); end if;
    end function;
    -- quarter-wave index of t + 1/4 (|cos t| = SIN[that])
    function f_cfold(t : unsigned(15 downto 0)) return unsigned is
        variable v_u : unsigned(15 downto 0);
    begin
        v_u := t + to_unsigned(16384, 16);
        if v_u(14) = '1' then return not v_u(13 downto 4); else return v_u(13 downto 4); end if;
    end function;
    -- the slope-table index: the top 8 bits of f_cfold
    function f_tgi(t : unsigned(15 downto 0)) return unsigned is
        variable v_i : unsigned(9 downto 0);
    begin
        v_i := f_cfold(t);
        return v_i(9 downto 2);
    end function;


    attribute no_rw_check : boolean;
    type t_r256x16 is array (0 to 255) of unsigned(15 downto 0);
    type t_r512x16 is array (0 to 511) of unsigned(15 downto 0);
    type t_r256x8 is array (0 to 255) of unsigned(7 downto 0);

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys, s_ysl : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(12 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- [A] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    type t_bit_shift is array (0 to 25) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');
    -- block-RAM delay lines (write and read counters a fixed distance apart; never the same address)
    signal dl_wa : unsigned(7 downto 0) := to_unsigned(37, 8);
    signal dl_ra : unsigned(7 downto 0) := (others => '0');
    signal dl_rc : unsigned(7 downto 0) := to_unsigned(30, 8);
    signal dls_ram, dlc_ram, dlp_ram : t_r256x16 := (others => (others => '0'));
    attribute no_rw_check of dls_ram : signal is true;
    attribute no_rw_check of dlc_ram : signal is true;
    attribute no_rw_check of dlp_ram : signal is true;
    signal dls_wd, dlc_wd, dlp_wd : unsigned(15 downto 0) := (others => '0');
    signal dls_q, dlc_q, dlp_q : unsigned(15 downto 0) := (others => '0');
    signal dls_q2 : unsigned(2 downto 0) := "110";
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 6) := (others => '0');   -- g_start delayed 1 .. 7

    -- [B] knobs: glided (Q3), rate, switches, palette
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(C_DEF_K1 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(C_DEF_K6 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(C_DEF_K2 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(C_DEF_K3 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(C_DEF_P12 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_swell, s_rad, s_two, s_lay, s_src : std_logic := '0';
    signal s_half : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(C_DEF_PAL, 4);

    -- [C] ROM C_BR_TAB (exp2, palette ramp words, bcap, the profile)
    signal r_ta : unsigned(9 downto 0) := (others => '0');
    signal r_tq, r_tf : unsigned(15 downto 0) := (others => '0');
    signal w_ta  : unsigned(9 downto 0);
    signal w_pqa : unsigned(6 downto 0);

    -- [C] per-field serial multiplier (riley's, 34 bits)
    signal z_op, fe_op, m_op : unsigned(4 downto 0) := to_unsigned(31, 5);   -- multiplier op codes (31 = none)
    signal f_idxm : unsigned(4 downto 0) := (others => '0');
    signal z_sh4 : unsigned(2 downto 0) := (others => '0');

    -- [C] per-field constants
    signal c_ph    : unsigned(31 downto 0) := (others => '0');     -- wave phase, Q32 cycles
    signal c_rstep : unsigned(27 downto 0) := (others => '0');
    signal c_p2    : unsigned(12 downto 0) := (others => '0');     -- P12^2, Q13
    signal c_K     : unsigned(15 downto 0) := (others => '0');     -- braid cycles per half pitch, Q16
    signal c_aW    : unsigned(15 downto 0) := (others => '0');     -- swing / half pitch, Q16
    signal c_Ap    : unsigned(15 downto 0) := (others => '0');     -- A / 2 pi, Q16
    signal c_A12   : unsigned(13 downto 0) := (others => '0');     -- A, Q12
    signal c_bfrac, c_bcap, c_b : unsigned(15 downto 0) := (others => '0');
    signal c_oct   : unsigned(8 downto 0) := (others => '0');
    signal c_P8    : unsigned(17 downto 0) := to_unsigned(40960, 18);   -- plait pitch, px Q8
    signal c_W4    : unsigned(12 downto 0) := (others => '0');     -- swing, px Q4
    signal c_R4    : unsigned(12 downto 0) := (others => '0');
    signal c_Rmx, c_R2, c_span : unsigned(10 downto 0) := (others => '0');
    signal c_room  : signed(11 downto 0) := (others => '0');
    signal c_g6q   : unsigned(9 downto 0) := (others => '0');
    signal c_dep   : unsigned(14 downto 0) := (others => '0');     -- depth, cycles Q16
    signal c_t1    : unsigned(16 downto 0) := (others => '0');
    signal c_ky    : unsigned(27 downto 0) := (others => '0');     -- braid cycles per field line, Q32
    signal c_esh   : unsigned(1 downto 0) := (others => '0');
    signal c_nsh   : unsigned(3 downto 0) := (others => '0');
    signal c_bacc  : signed(19 downto 0) := (others => '0');
    signal c_i0    : signed(5 downto 0) := (others => '0');
    signal c_v0    : unsigned(17 downto 0) := (others => '0');
    signal c_brun  : std_logic := '0';
    signal c_gy, c_gu, c_gv : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- colour fill
    signal cf_lo, cf_s1, cf_s2 : signed(11 downto 0) := (others => '0');
    signal cf_acc : signed(17 downto 0) := (others => '0');
    signal cf_s, cf_ch : unsigned(1 downto 0) := (others => '0');
    signal cf_k : unsigned(7 downto 0) := (others => '0');
    signal cf_run : std_logic := '0';
    signal cf_ph : unsigned(2 downto 0) := (others => '0');
    signal cf_u8 : unsigned(7 downto 0) := (others => '0');
    signal cf_ra : unsigned(9 downto 0) := (others => '0');
    -- WT fill (the camera swell by level)
    signal fe_go  : std_logic := '0';

    -- RAMs written by the sequencer: WT, CTY (Y by strand, k), CTU / CTV (by strand, k / 2)
    signal wt_ram : t_r256x16 := (others => (others => '0'));
    attribute no_rw_check of wt_ram : signal is true;
    signal wt_we : std_logic := '0';
    signal wt_wa, wt_a : unsigned(7 downto 0) := (others => '0');
    signal wt_wd, wt_q : unsigned(15 downto 0) := (others => '0');
    signal cty_ram : t_r512x16 := (others => (others => '0'));
    attribute no_rw_check of cty_ram : signal is true;
    signal cty_we : std_logic := '0';
    signal cty_wa, cty_a : unsigned(8 downto 0) := (others => '0');
    signal cty_wd, cty_q : unsigned(15 downto 0) := (others => '0');
    signal ctu_ram, ctv_ram : t_r256x8 := (others => (others => '0'));
    attribute no_rw_check of ctu_ram : signal is true;
    attribute no_rw_check of ctv_ram : signal is true;
    signal ctu_we, ctv_we : std_logic := '0';
    signal ctu_wa, ctu_a : unsigned(7 downto 0) := (others => '0');
    signal ctu_q, ctv_q : unsigned(7 downto 0) := (others => '0');

    -- [D] source: Osc, luma IIR, per-plait sampling, the ping-pong state RAM (2 x 16 plaits)
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl  : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_svd : std_logic := '0';
    signal s_h   : unsigned(11 downto 0) := (others => '0');
    signal s_tk1, s_tk2, s_tk3, s_hv, s_tb1 : std_logic := '0';
    signal s_hs1 : unsigned(11 downto 0) := (others => '0');
    signal s_tj1 : unsigned(3 downto 0) := (others => '0');
    signal s_old : unsigned(11 downto 0) := (others => '0');
    signal s_wb  : std_logic := '0';
    signal cm_ram, cmf_ram : t_r256x16 := (others => (others => '0'));
    attribute no_rw_check of cm_ram : signal is true;
    attribute no_rw_check of cmf_ram : signal is true;
    signal cm_we : std_logic := '0';
    signal cm_wa, cm_ra, cm_a2 : unsigned(7 downto 0) := (others => '0');
    signal cm_wd, cm_q, cmf_q : unsigned(15 downto 0) := (others => '0');

    -- [E] the pipelined multiplier (product 4 clocks after the operands are written)
    signal mu_a  : unsigned(14 downto 0) := (others => '0');
    signal mu_b  : unsigned(11 downto 0) := (others => '0');
    signal mu_p0, mu_p1, mu_p2, mu_p3 : unsigned(17 downto 0) := (others => '0');
    signal mu_s01, mu_s23 : unsigned(20 downto 0) := (others => '0');
    signal mu_p  : unsigned(26 downto 0) := (others => '0');

    -- [E] fill + fetch engine
    signal fe_mode : unsigned(2 downto 0) := (others => '0');      -- 0 idle, 1 RT, 2 WT, 3 D, 4 slot fetch
    signal fe_c : unsigned(11 downto 0) := (others => '0');
    signal fe_j : unsigned(4 downto 0) := (others => '0');          -- the slot being fetched
    signal fe_seq : unsigned(1 downto 0) := (others => '0');        -- line start: 1 slot 0, 2 slot 1; 0 a slot change
    signal fe_xf, fe_xf_req, fe_lsp, fe_scp : std_logic := '0';     -- the line-start transfer; requests
    signal f_next : unsigned(4 downto 0) := (others => '0');
    signal f_w0, f_w1, f_w2 : unsigned(31 downto 0) := (others => '0');
    signal f_l2 : unsigned(15 downto 0) := (others => '0');         -- l^2 (WT fill)
    signal f_u  : unsigned(11 downto 0) := (others => '0');
    signal f_ka, f_kb : unsigned(1 downto 0) := (others => '0');
    signal sn_a : unsigned(9 downto 0) := (others => '0');
    signal sn_q : unsigned(11 downto 0) := (others => '0');
    signal sq_a : unsigned(7 downto 0) := (others => '0');
    signal sq_q : unsigned(9 downto 0) := (others => '0');
    signal rt_ram : t_r256x16 := (others => (others => '0'));
    attribute no_rw_check of rt_ram : signal is true;
    signal rt_we : std_logic := '0';
    signal rt_wa, rt_a : unsigned(7 downto 0) := (others => '0');
    signal rt_wd, rt_q : unsigned(15 downto 0) := (others => '0');
    signal d_ram : t_r256x16 := (others => (others => '0'));
    attribute no_rw_check of d_ram : signal is true;
    signal d_we : std_logic := '0';
    signal d_wa, d_a : unsigned(7 downto 0) := (others => '0');
    signal d_wd, d_q : unsigned(15 downto 0) := (others => '0');
    signal f_yacc : unsigned(31 downto 0) := (others => '0');
    signal f_yh, f_psi : unsigned(15 downto 0) := (others => '0');
    signal f_sgn : std_logic_vector(0 to 2) := (others => '0');     -- the three half-cycle signs t_s(15)
    signal f_rq  : unsigned(12 downto 0) := (others => '0');        -- RT read, fabric copy
    signal f_sw  : signed(15 downto 0) := (others => '0');
    signal f_mir : std_logic := '0';
    signal f_d   : unsigned(15 downto 0) := (others => '0');
    type t_s13x3 is array (0 to 2) of signed(12 downto 0);
    type t_u2x3 is array (0 to 2) of unsigned(1 downto 0);
    signal f_cw  : unsigned(10 downto 0) := (others => '0');
    signal f_ct  : signed(12 downto 0) := (others => '0');
    signal f_idx : signed(5 downto 0) := (others => '0');
    signal f_th  : unsigned(15 downto 0) := (others => '0');
    signal f_V   : unsigned(14 downto 0) := (others => '0');
    signal f_vsg : std_logic := '0';



    -- [F] beam (stage 8), reader, current / staged slot words
    signal b_v  : unsigned(17 downto 0) := (others => '0');
    signal b_rnd, b_rnd_d, b_xf, b_end, b_smp : std_logic := '0';
    signal b_slot : unsigned(3 downto 0) := (others => '0');
    signal c_cm1 : unsigned(9 downto 0) := (others => '0');
    signal c_s0 : std_logic := '0';
    type t_w32x3 is array (0 to 2) of unsigned(31 downto 0);
    signal r_cur : t_w32x3 := (others => (others => '0'));

    -- [F] pixel path
    type t_u12x3b is array (0 to 2) of unsigned(11 downto 0);
    signal p9_d : t_u12x3b := (others => (others => '0'));
    type t_u11x3 is array (0 to 2) of unsigned(10 downto 0);
    signal p9_rh, p10_rh : t_u11x3 := (others => (others => '0'));
    signal p9_kaa, p10_kaa : t_u2x3 := (others => (others => '0'));
    signal p9_rk, p10_rk : t_u2x3 := (others => (others => '0'));
    signal p9_en, p10_en, p10_in : std_logic_vector(0 to 2) := (others => '0');
    signal p10_e : t_s13x3 := (others => (others => '0'));
    signal p11_rh : unsigned(10 downto 0) := (others => '0');
    signal p11_e, p11_esh : signed(12 downto 0) := (others => '0');
    signal p11_kaa : unsigned(1 downto 0) := (others => '0');
    signal p11_hab, p12_hab : std_logic := '0';
    signal p12_r5 : unsigned(4 downto 0) := (others => '0');
    signal p12_big : std_logic := '0';
    type t_dvr is array (12 to 18) of unsigned(9 downto 0);   -- remainder < divisor: 10 bits
    signal pv_r : t_dvr := (others => (others => '0'));
    type t_dvd is array (12 to 18) of unsigned(9 downto 0);
    signal pv_d : t_dvd := (others => (others => '0'));
    type t_dvq is array (12 to 20) of unsigned(6 downto 0);
    signal pv_q : t_dvq := (others => (others => '0'));
    signal pv_eq : std_logic_vector(12 to 20) := (others => '0');
    type t_c2 is array (11 to 28) of unsigned(1 downto 0);
    signal pc_vs : t_c2 := (others => (others => '0'));
    signal pc_bv, pc_any : std_logic_vector(11 to 28) := (others => '0');
    type t_c4 is array (12 to 28) of unsigned(3 downto 0);
    signal pc_cov : t_c4 := (others => (others => '0'));
    signal p13_kzs, p22_kzs : unsigned(5 downto 0) := (others => '0');
    type t_kzs is array (0 to 15) of unsigned(5 downto 0);
    constant C_KZS : t_kzs := (to_unsigned(44, 6), to_unsigned(42, 6), to_unsigned(39, 6), to_unsigned(36, 6),
                               to_unsigned(33, 6), to_unsigned(31, 6), to_unsigned(28, 6), to_unsigned(25, 6), to_unsigned(22, 6), to_unsigned(20, 6), to_unsigned(17, 6), to_unsigned(14, 6), to_unsigned(11, 6), to_unsigned(9, 6), to_unsigned(6, 6), to_unsigned(3, 6));
    signal p22_vs : unsigned(1 downto 0) := (others => '0');
    signal p22_bv, p22_any, p22_win : std_logic := '0';
    signal p22_cov : unsigned(3 downto 0) := (others => '0');
    signal p28_c : unsigned(15 downto 0) := (others => '0');
    signal p23_ca, p24_ca, p25_ca : unsigned(7 downto 0) := (others => '0');
    signal p22_kq : unsigned(7 downto 0) := (others => '0');
    signal p25_ty, p25_by : unsigned(9 downto 0) := (others => '0');
    signal p26_dy : signed(10 downto 0) := (others => '0');
    signal p27_ay, p27_ey : signed(14 downto 0) := (others => '0');
    signal p28_y : signed(14 downto 0) := (others => '0');
    signal s29_y, s29_u, s29_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- [G] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);

    signal s_yd : t_u10(1 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv2, s_bud, s_bvd : unsigned(12 downto 0) := to_unsigned(4100, 13);
    signal s_hist : std_logic_vector(7 downto 0) := (others => '0');
    signal s30_ya : unsigned(11 downto 0) := (others => '0');
    signal s30_yb : unsigned(9 downto 0) := (others => '0');
    signal s31_y, s32_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s31_u, s31_v, s32_u, s32_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s33_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s33_u, s33_v : unsigned(9 downto 0) := to_unsigned(512, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    -- C_BR_TAB: the sequencer's in the zone, the pixel path's profile outside it (512 + 384 + q)
    w_pqa <= (others => '1') when pv_eq(20) = '1' else pv_q(20);
    w_ta  <= r_ta when s_zone_d = '1' else "111" & w_pqa;     -- (its own copy of the zone flag)
    p_tab : process(clk)
    begin
        if rising_edge(clk) then
            r_tq <= C_BR_TAB(to_integer(w_ta));
            r_tf <= r_tq;
        end if;
    end process p_tab;

    p_sin : process(clk)
    begin
        if rising_edge(clk) then
            sn_q <= C_BR_SIN(to_integer(sn_a));
            sq_q <= C_BR_SQ(to_integer(sq_a));
        end if;
    end process p_sin;

    p_wtram : process(clk)
    begin
        if rising_edge(clk) then
            wt_q <= wt_ram(to_integer(wt_a));
            if wt_we = '1' then wt_ram(to_integer(wt_wa)) <= wt_wd; end if;
        end if;
    end process p_wtram;

    p_ctyram : process(clk)
    begin
        if rising_edge(clk) then
            cty_q <= cty_ram(to_integer(cty_a));
            if cty_we = '1' then cty_ram(to_integer(cty_wa)) <= cty_wd; end if;
        end if;
    end process p_ctyram;
    p_cturam : process(clk)
    begin
        if rising_edge(clk) then
            ctu_q <= ctu_ram(to_integer(ctu_a));
            if ctu_we = '1' then ctu_ram(to_integer(ctu_wa)) <= cf_u8; end if;
        end if;
    end process p_cturam;
    p_ctvram : process(clk)
    begin
        if rising_edge(clk) then
            ctv_q <= ctv_ram(to_integer(ctu_a));
            if ctv_we = '1' then ctv_ram(to_integer(ctu_wa)) <= cf_u8; end if;
        end if;
    end process p_ctvram;

    p_rtram : process(clk)
    begin
        if rising_edge(clk) then
            rt_q <= rt_ram(to_integer(rt_a));
            if rt_we = '1' then rt_ram(to_integer(rt_wa)) <= rt_wd; end if;
        end if;
    end process p_rtram;

    p_dram : process(clk)
    begin
        if rising_edge(clk) then
            d_q <= d_ram(to_integer(d_a));
            if d_we = '1' then d_ram(to_integer(d_wa)) <= d_wd; end if;
        end if;
    end process p_dram;

    -- delay lines: sync (hsync, vsync, avid: 38 clocks, then a register), chroma (the 8-sums' trailing taps,
    -- 8 clocks), pixel state (stage 13 -> 21)
    p_dlram : process(clk)
    begin
        if rising_edge(clk) then
            dls_q <= dls_ram(to_integer(dl_ra));
            dls_ram(to_integer(dl_wa)) <= dls_wd;
        end if;
    end process p_dlram;
    p_dlcram : process(clk)
    begin
        if rising_edge(clk) then
            dlc_q <= dlc_ram(to_integer(dl_rc));
            dlc_ram(to_integer(dl_wa)) <= dlc_wd;
        end if;
    end process p_dlcram;
    p_dlpram : process(clk)
    begin
        if rising_edge(clk) then
            dlp_q <= dlp_ram(to_integer(dl_rc));
            dlp_ram(to_integer(dl_wa)) <= dlp_wd;
        end if;
    end process p_dlpram;

    -- camera state: two copies with one write (the source path's vertical IIR reads one, the fetch the other);
    -- both read the bank (line) not being written
    p_cmram : process(clk)
    begin
        if rising_edge(clk) then
            cm_q <= cm_ram(to_integer(cm_ra));
            if cm_we = '1' then cm_ram(to_integer(cm_wa)) <= cm_wd; end if;
        end if;
    end process p_cmram;
    p_cmfram : process(clk)
    begin
        if rising_edge(clk) then
            cmf_q <= cmf_ram(to_integer(cm_a2));
            if cm_we = '1' then cmf_ram(to_integer(cm_wa)) <= cm_wd; end if;
        end if;
    end process p_cmfram;


    ---------------------------------------------------------------------------
    -- [A] TIMING + [B] CONTROLS (riley's chassis)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_c  : signed(10 downto 0);
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
            elsif s_zc /= 8191 then
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

            -- rate: bipolar with a +-8 deadband
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - to_signed(512, 11);
            s_vneg <= v_s(10);
            if v_s > -9 and v_s < 9 then
                v_m := (others => '0');
            elsif v_s(10) = '1' then
                v_m := unsigned(not v_s(9 downto 0)) - 7;
            else
                v_m := unsigned(v_s(9 downto 0)) - 8;
            end if;
            s_vm <= v_m(8 downto 0);

            -- per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_swell <= registers_in(6)(0);
                s_rad   <= registers_in(6)(1);
                s_two   <= registers_in(6)(2);
                s_lay   <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_half  <= registers_in(6)(1) or registers_in(6)(3);
                s_xs    <= s_xsr;
                s_ysl   <= s_ys;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                -- palette: 16 steps with hysteresis (leave a step only 8 past its edge; both ends reachable)
                v_k := unsigned(registers_in(4));
                v_c := signed(resize(v_k, 11)) - signed(resize(s_pal & "100000", 11));
                if v_c > 40 or v_c < -40 then s_pal <= v_k(9 downto 6); end if;
            end if;
            g_wm1 <= s_w - 1;
            case to_integer(s_ys) is                         -- (ys x 121) / 256, by mode
                when 1138   => s_oys <= to_unsigned(537, 10);
                when 576    => s_oys <= to_unsigned(272, 10);
                when 960    => s_oys <= to_unsigned(453, 10);
                when 480    => s_oys <= to_unsigned(226, 10);
                when 384    => s_oys <= to_unsigned(181, 10);
                when 512    => s_oys <= to_unsigned(242, 10);
                when others => s_oys <= to_unsigned(121, 10);
            end case;

            -- shared glide (1/8 of the gap per field, +-2 deadband, the camera +-4): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when 2      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 3      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if C_SNAP then
                            null;
                        elsif (s_gd > 16 or s_gd < -16) and (z_gc /= 1 or s_gd > 32 or s_gd < -32) then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 4 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- sync delay, generated-avid shift register (field_n is tied high by the core: one register)
            s_gav_sr(0) <= g_avid;
            for i in 1 to 25 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
            -- the sync delay line (tap 0 here, tap 38 out of the RAM, tap 39 one register later)
            dls_wd <= resize(unsigned'(data_in.hsync_n & data_in.vsync_n & g_avid), 16);
            dl_wa <= dl_wa + 1; dl_ra <= dl_ra + 1; dl_rc <= dl_rc + 1;
            dls_q2 <= dls_q(2 downto 0);
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 5);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [A] Generated avid (parity-locked start 3/4 clocks after the source's
    -- avid rise, field-latched width) -- riley
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
    -- [C] FIELD SEQUENCER (vblank zone; one small operation per zone clock).
    -- Serial products: slot k's op code is issued at zc = 29 + 24k, registered
    -- (m_op) at 30 + 24k, the operands load at 31 + 24k; stored at 50 + 24k.
    ---------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_b  : signed(19 downto 0);
        variable v_rm : signed(11 downto 0);
        variable v_x  : signed(17 downto 0);
        variable v_sg : unsigned(10 downto 0);
        variable v_no : unsigned(8 downto 0);
        variable v_rc : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            fe_go <= '0';
            -- (no zone tests here: s_zc is 0 outside the zone and saturates at 8191 long before it ends)
            if s_zc = 440 then fe_go <= '1'; end if;

            -- slot decode and operand pre-mux (no zc decode on the multiplier's path)
            -- per-field products: slot k's op code at zc 29 + 24k (operands into the multiplier at 31 + 24k,
            -- the product stable from 36 + 24k until the next load; stored at 50 + 24k)
            z_op <= to_unsigned(31, 5);
            if s_zc(12 downto 9) = "0000" and s_zc >= 29 and s_zc <= 413 then
                case to_integer(s_zc) is
                    when 29  => z_op <= to_unsigned(0, 5);
                    when 53  => z_op <= to_unsigned(1, 5);
                    when 77  => z_op <= to_unsigned(2, 5);
                    when 101 => z_op <= to_unsigned(3, 5);
                    when 125 => z_op <= to_unsigned(4, 5);
                    when 149 => z_op <= to_unsigned(5, 5);
                    when 173 => z_op <= to_unsigned(6, 5);
                    when 197 => z_op <= to_unsigned(7, 5);
                    when 221 => z_op <= to_unsigned(8, 5);
                    when 245 => z_op <= to_unsigned(9, 5);
                    when 269 => z_op <= to_unsigned(10, 5);
                    when 317 => z_op <= to_unsigned(12, 5);
                    when 341 => z_op <= to_unsigned(13, 5);
                    when 365 => z_op <= to_unsigned(14, 5);
                    when 389 => z_op <= to_unsigned(15, 5);
                    when 413 => z_op <= to_unsigned(16, 5);
                    when others => null;
                end case;
            end if;

            case to_integer(s_zc) is
                when 20 =>
                    c_ph <= f_addsub(c_ph, resize(c_rstep, 32), s_vneg);
                when 21 =>
                    c_oct <= s_g1(12 downto 4);
                    c_g6q <= s_g6(12 downto 3);                  -- camera gain 4 K6

                when 22 =>
                    if c_oct = 0 then c_esh <= "00";
                    elsif c_oct(8) = '0' or c_oct(7 downto 0) = 0 then c_esh <= "01";
                    else c_esh <= "10"; end if;
                when 50  => c_p2 <= mu_p(24 downto 12);
                when 74  => c_K <= to_unsigned(3604, 16) + mu_p(24 downto 9);
                when 98  => c_K <= c_K + resize(mu_p(22 downto 9), 16);
                when 122 => c_aW <= to_unsigned(39322, 16) + resize(mu_p(23 downto 10), 16);
                when 123 => if s_two = '1' then c_aW <= c_aW - resize(c_aW(15 downto 3), 16); end if;
                when 146 => c_Ap <= resize(mu_p(26 downto 12), 16);
                when 168 =>                                      -- (c_t1 is free until zc 410)
                    c_t1 <= to_unsigned(19661, 17) + resize(s_g2 & "00", 17) + resize(c_p2 & '0', 17);
                when 170 =>
                    c_A12 <= mu_p(25 downto 12);
                    if c_t1 > 62259 then c_bfrac <= to_unsigned(62259, 16); else c_bfrac <= c_t1(15 downto 0); end if;
                when 171 => r_ta <= "110" & c_A12(13 downto 7);
                when 172 => r_ta <= "00" & c_oct(7 downto 0);
                when 174 =>
                    if s_two = '1' then c_bcap <= to_unsigned(62259, 16); else c_bcap <= r_tf; end if;
                when 194 =>
                    if c_oct(8) = '1' then c_P8 <= mu_p(26 downto 10) & '0';
                    else c_P8 <= resize(mu_p(26 downto 10), 18); end if;
                -- (H = P / 2 and its other scalings are slices of c_P8; the camera sample sits at H + 22 px)
                when 196 => c_cm1 <= resize(c_P8(17 downto 9), 10) + 21;
                when 218 => c_b <= mu_p(25 downto 10);
                when 242 => c_W4 <= mu_p(26 downto 14);
                when 266 =>
                    c_R4 <= mu_p(24 downto 12);
                    c_room <= signed(resize(c_P8(17 downto 7), 12)) - signed(resize(c_W4(12 downto 2), 12));
                when 267 =>
                    -- tube shadow width sigma = R / 4: 2^nsh px, or 1.45 x that (shh)
                    v_sg := c_R4(12 downto 2);
                    if v_sg(10 downto 7) /= "0000" then c_nsh <= to_unsigned(3, 4);
                    elsif v_sg(6) = '1' then c_nsh <= to_unsigned(2, 4);
                    elsif v_sg(5) = '1' then c_nsh <= to_unsigned(1, 4);
                    else c_nsh <= to_unsigned(0, 4); end if;
                when 288 => c_room <= c_room - shift_right(c_room, 4) - 4;
                when 290 =>
                    -- the radius ceiling: the no-overlap cap, and the room inside the plait (15/16, less 1 px)
                    v_rm := c_room;
                    v_rc := mu_p(24 downto 14);
                    if v_rm < 0 then c_Rmx <= (others => '0');
                    elsif signed(resize(v_rc, 12)) < v_rm then c_Rmx <= v_rc;
                    else c_Rmx <= unsigned(v_rm(10 downto 0)); end if;
                when 291 =>
                    if c_R4(12 downto 2) < c_Rmx then c_R2 <= c_R4(12 downto 2); else c_R2 <= c_Rmx; end if;
                when 292 => c_span <= c_Rmx - c_R2;
                when 338 => c_dep <= mu_p(19 downto 5);
                when 362 => r_ta <= "00" & mu_p(15 downto 8); z_sh4 <= mu_p(18 downto 16);
                when 386 =>
                    if s_vm = 0 then c_rstep <= (others => '0');
                    else c_rstep <= resize(mu_p(26 downto 5), 28); end if;
                    v_no := (not c_oct) + 1;
                    r_ta <= "00" & v_no(7 downto 0);
                when 410 => c_t1 <= resize(mu_p(26 downto 11), 17);
                    if c_v0(17 downto 8) > c_cm1 then c_s0 <= '1'; else c_s0 <= '0'; end if;
                when 434 => c_ky <= mu_p & '0';
                when 435 => if c_esh /= 0 then c_ky <= '0' & c_ky(27 downto 1); end if;
                when 436 => if c_esh(1) = '1' then c_ky <= '0' & c_ky(27 downto 1); end if;
                when others => null;
            end case;

            -- rate step: shift left by z_sh4, one bit a clock (zc 387 .. 393)
            if s_zc >= 387 and s_zc < 394 and resize(s_zc - 386, 4) <= resize(z_sh4, 4) then
                c_rstep <= c_rstep(26 downto 0) & '0';
            end if;

            -- beam start (zc 300 ..): pixel 0's offset from its plait slot's left boundary
            if s_zc = 300 then
                case to_integer(s_xs) is                     -- the screen centre, px Q8
                    when 4      => v_b := to_signed(-92160, 20);
                    when 3      => v_b := to_signed(-163840, 20);
                    when others => v_b := to_signed(-245760, 20);
                end case;
                if s_half = '0' then v_b := v_b + signed(resize(c_P8(17 downto 1), 20)); end if;
                c_bacc <= v_b; c_i0 <= (others => '0'); c_brun <= '1';
            elsif c_brun = '1' then
                if c_bacc(19) = '1' then
                    c_bacc <= c_bacc + signed(resize(c_P8, 20));
                    c_i0 <= c_i0 - 1;
                else
                    c_v0 <= unsigned(c_bacc(17 downto 0));
                    c_brun <= '0';
                end if;
            end if;

            -- ground colour (zc 768 .. 780): 3 words, one every 4 clocks (read i, store i - 1)
            if s_zc(12 downto 4) = "000110000" and s_zc(1 downto 0) = "00" then
                r_ta <= to_unsigned(256, 10) + resize(s_pal & "00000", 10) + resize(s_zc(3 downto 2), 10);
                case to_integer(s_zc(3 downto 2)) is
                    when 1 => c_gy <= r_tf(9 downto 0);
                    when 2 => c_gu <= r_tf(9 downto 0);
                    when 3 => c_gv <= r_tf(9 downto 0);
                    when others => null;
                end case;
            end if;

            -- colour tables (zc 832 ..): nine passes (strand colour s, channel ch): three ramp words (lo, s1,
            -- s2), then 128 entries: acc = 64 lo + k s1 (k < 64), + (k - 64) s2 after;  Y every k, U and V
            -- (8 bit, (v + 2) / 4) every even k
            cty_we <= '0'; ctu_we <= '0'; ctv_we <= '0';
            -- the tables' write address and data every clock (the write enables pick the clocks)
            v_x := shift_right(cf_acc, 6);
            cty_wa <= cf_s & cf_k(6 downto 0);
            cty_wd <= resize(unsigned(v_x(9 downto 0)), 16);
            ctu_wa <= cf_s & cf_k(6 downto 1);
            v_x := v_x + 2;
            cf_u8 <= unsigned(v_x(9 downto 2));
            if s_zc = 832 then
                cf_run <= '1'; cf_s <= (others => '0'); cf_ch <= (others => '0'); cf_ph <= (others => '0');
                cf_ra <= to_unsigned(259, 10) + resize(s_pal & "00000", 10);
            elsif cf_run = '1' then
                case to_integer(cf_ph) is
                    when 0 | 1 | 2 =>
                        r_ta <= cf_ra; cf_ra <= cf_ra + 1;
                        cf_ph <= cf_ph + 1;
                    when 3 => cf_lo <= signed(r_tf(11 downto 0)); cf_ph <= cf_ph + 1;
                    when 4 => cf_s1 <= signed(r_tf(11 downto 0)); cf_ph <= cf_ph + 1;
                    when 5 =>
                        cf_s2 <= signed(r_tf(11 downto 0));
                        cf_acc <= shift_left(resize(cf_lo, 18), 6);
                        cf_k <= (others => '0');
                        cf_ph <= cf_ph + 1;
                    when others =>
                        if cf_ch = 0 then cty_we <= '1'; end if;
                        if cf_ch = 1 and cf_k(0) = '0' then ctu_we <= '1'; end if;
                        if cf_ch = 2 and cf_k(0) = '0' then ctv_we <= '1'; end if;
                        if cf_k(6) = '0' then cf_acc <= cf_acc + resize(cf_s1, 18);
                        else cf_acc <= cf_acc + resize(cf_s2, 18); end if;
                        cf_k <= cf_k + 1;
                        if cf_k = 127 then
                            cf_ph <= (others => '0');
                            if cf_ch = 2 then
                                cf_ch <= (others => '0');
                                if cf_s = 2 then cf_run <= '0'; end if;
                                cf_s <= cf_s + 1;
                            else
                                cf_ch <= cf_ch + 1;
                            end if;
                        end if;
                end case;
            end if;
        end if;
    end process p_seq;

    ---------------------------------------------------------------------------
    -- [D] SOURCE: luma (>> 2, less 16) or the Osc lattice -> a horizontal 1/32
    -- IIR -> sampled at each plait centre (+ 32 px for the IIR's lag; a centre
    -- past the line's end takes its last value) -> a vertical 1/16 IIR per
    -- plait in the ping-pong state RAM (bank wb written, bank not-wb read)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_d : signed(13 downto 0);
        variable v_h : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting (riley)
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 2);

            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid; s_svd <= s_sv;
            s_hv <= s_sv;

            -- horizontal IIR (the line's first pixel starts it at its own value)
            if s_sv = '1' then
                if s_svd = '0' then
                    s_h <= s_sl & "0000";
                    s_wb <= not s_wb;
                else
                    v_d := signed(resize(s_sl & "0000", 14)) - signed(resize(s_h, 14));
                    v_h := signed(resize(s_h, 14)) + shift_right(v_d, 5);
                    s_h <= unsigned(v_h(11 downto 0));
                end if;
            end if;
            -- a sample when the beam crosses a plait's centre (b_smp, slot b_slot)
            s_tk1 <= b_smp;
            cm_ra <= unsigned'(0 => not s_wb) & "000" & b_slot;
            if b_smp = '1' then s_tj1 <= b_slot; s_hs1 <= s_h; s_tb1 <= s_wb; end if;   -- held until written
            -- the vertical IIR, pipelined: read (1), RAM (2), fabric (3), write (4)
            s_tk2 <= s_tk1; s_tk3 <= s_tk2;
            s_old <= cm_q(11 downto 0);
            cm_we <= '0';
            if s_tk3 = '1' then
                v_d := signed(resize(s_hs1, 14)) - signed(resize(s_old, 14));
                v_h := signed(resize(s_old, 14)) + shift_right(v_d, 4);
                cm_wd <= resize(unsigned(v_h(11 downto 0)), 16);
                cm_wa <= unsigned'(0 => s_tb1) & "000" & s_tj1;
                cm_we <= '1';
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] THE MULTIPLIER'S OPERANDS: one op code a clock (the zone's, else the
    -- fill / fetch engine's); held while the op code is 31
    ---------------------------------------------------------------------------
    p_op : process(clk)
    begin
        if rising_edge(clk) then
            -- the op code is registered once (m_op) so the operand mux decodes a register
            if z_op /= 31 then m_op <= z_op; else m_op <= fe_op; end if;
            case to_integer(m_op) is
                when 0  => mu_a <= resize(s_gp, 15);              mu_b <= s_gp(12 downto 1);          -- P12^2
                when 1  => mu_a <= resize(s_gp, 15);              mu_b <= to_unsigned(1233, 12);      -- K, linear
                when 2  => mu_a <= resize(c_p2, 15);              mu_b <= to_unsigned(418, 12);       -- K, quadratic
                when 3  => mu_a <= resize(s_gp, 15);              mu_b <= to_unsigned(983, 12);       -- swing / half pitch
                when 4  => mu_a <= c_K(14 downto 0);              mu_b <= c_aW(15 downto 4);          -- A / 2 pi
                when 5  => mu_a <= c_Ap(14 downto 0);             mu_b <= to_unsigned(1609, 12);      -- A
                when 6  =>                                                                         -- pitch
                    mu_a <= r_tf(15 downto 1);
                    case to_integer(s_xs) is
                        when 4      => mu_b <= to_unsigned(1280, 12);
                        when 3      => mu_b <= to_unsigned(1707, 12);
                        when others => mu_b <= to_unsigned(2560, 12);
                    end case;
                when 7  => mu_a <= resize(c_bfrac(15 downto 2), 15); mu_b <= c_bcap(15 downto 4);     -- R / W at black
                when 8  => mu_a <= c_P8(17 downto 3);             mu_b <= c_aW(15 downto 4);          -- swing (H x)
                when 9  => mu_a <= resize(c_W4, 15);              mu_b <= c_b(15 downto 4);           -- R
                when 10 => mu_a <= resize(c_W4, 15);              mu_b <= c_bcap(15 downto 4);        -- R cap
                when 12 => mu_a <= resize(s_g3(12 downto 3), 15); mu_b <= resize(s_g3(12 downto 3), 12);   -- depth
                when 13 => mu_a <= resize(s_vm, 15);              mu_b <= to_unsigned(897, 12);       -- rate exponent
                when 14 => mu_a <= r_tf(15 downto 1);             mu_b <= to_unsigned(2330, 12);      -- rate step
                when 15 => mu_a <= c_K(14 downto 0);              mu_b <= r_tf(15 downto 4);          -- K x 2^(-oct)
                when 16 =>                                                                         -- cycles per line
                    mu_a <= c_t1(15 downto 1);
                    case to_integer(s_ysl) is
                        when 384    => mu_b <= to_unsigned(1228, 12);
                        when 512    => mu_b <= to_unsigned(1638, 12);
                        when 1138   => mu_b <= to_unsigned(3641, 12);
                        when 576    => mu_b <= to_unsigned(1843, 12);
                        when 960    => mu_b <= to_unsigned(3072, 12);
                        when 480    => mu_b <= to_unsigned(1536, 12);
                        when others => mu_b <= to_unsigned(819, 12);
                    end case;
                when 17 => mu_a <= resize(c_A12, 15);             mu_b <= sn_q;                       -- RT: A sin
                when 18 => mu_a <= resize(c_R2, 15);              mu_b <= "00" & sq_q;                -- RT: R g
                when 19 => mu_a <= resize(c_g6q, 15);             mu_b <= f_l2(15 downto 4);          -- WT: gain l^2
                when 20 => mu_a <= resize(c_span, 15);            mu_b <= f_u;                        -- WT: span u
                when 21 => mu_a <= resize(f_idxm, 15);            mu_b <= to_unsigned(2949, 12);      -- D: Spread |index| / 2
                when 22 => mu_a <= c_dep;                         mu_b <= sn_q;                       -- D: Depth sin
                when 23 => mu_a <= f_V;                           mu_b <= resize(c_R2, 12);           -- D: Swell x R
                when 24 => mu_a <= "00" & c_W4;                   mu_b <= sn_q;                       -- slot: W |sin t_s|
                when others => null;
            end case;
        end if;
    end process p_op;

    ---------------------------------------------------------------------------
    -- [E] THE MULTIPLIER: 15 x 12 bits, pipelined (radix-8 partial products,
    -- two adds); the product is in mu_p 4 clocks after mu_a / mu_b are written
    ---------------------------------------------------------------------------
    p_mul : process(clk)
        function f_pp(a : unsigned(14 downto 0); d : unsigned(2 downto 0)) return unsigned is
            variable v : unsigned(17 downto 0);
        begin
            v := (others => '0');
            if d(0) = '1' then v := v + resize(a, 18); end if;
            if d(1) = '1' then v := v + resize(a & '0', 18); end if;
            if d(2) = '1' then v := v + resize(a & "00", 18); end if;
            return v;
        end function;
    begin
        if rising_edge(clk) then
            mu_p0 <= f_pp(mu_a, mu_b(2 downto 0));
            mu_p1 <= f_pp(mu_a, mu_b(5 downto 3));
            mu_p2 <= f_pp(mu_a, mu_b(8 downto 6));
            mu_p3 <= f_pp(mu_a, mu_b(11 downto 9));
            mu_s01 <= resize(mu_p0, 21) + shift_left(resize(mu_p1, 21), 3);
            mu_s23 <= resize(mu_p2, 21) + shift_left(resize(mu_p3, 21), 3);
            mu_p <= resize(mu_s01, 27) + shift_left(resize(mu_s23, 27), 6);
        end if;
    end process p_mul;

    ---------------------------------------------------------------------------
    -- [E] FILL + FETCH ENGINE.  Per field (kicked in the zone), on the multiplier:
    -- RT (256 entries, one every 4 clocks: A sin -> SQ -> R g, the AA ramp
    -- shift), WT (256, one every 4 clocks: the camera swell by level), D (16 x
    -- 20 clocks: the per-plait wave), then line 0.  Slots (16 clocks each): two
    -- at the end of each line's rendered window (the next line's slots 0 and 1,
    -- with the line-start transfer between them), then one more each time the
    -- beam steps into a new slot.
    ---------------------------------------------------------------------------
    p_fe : process(clk)
        variable v_i   : signed(5 downto 0);
        variable v_ix  : signed(5 downto 0);
        variable v_u   : unsigned(15 downto 0);
        variable v_ct  : signed(12 downto 0);
        variable v_rh  : signed(13 downto 0);
        variable v_rhu : unsigned(10 downto 0);
        variable v_tp, v_tq, v_tb : unsigned(15 downto 0);
        variable v_sg  : std_logic;
        variable v_xc  : unsigned(11 downto 0);
        variable v_aq  : unsigned(8 downto 0);
        variable v_y32 : unsigned(31 downto 0);
    begin
        if rising_edge(clk) then
            rt_we <= '0'; d_we <= '0'; wt_we <= '0'; fe_op <= to_unsigned(31, 5);
            -- the line-start transfer, one clock after slot 0's words land in ST
            fe_xf <= fe_xf_req; fe_xf_req <= '0';
            if b_end = '1' then fe_lsp <= '1'; end if;
            if b_xf = '1' then fe_scp <= '1'; end if;

            if fe_go = '1' then
                fe_mode <= "001"; fe_c <= (others => '0');
                f_yacc <= (others => '0'); fe_lsp <= '0'; fe_scp <= '0';
            elsif fe_mode = "000" then
                if fe_lsp = '1' then
                    -- the next line's slots 0 and 1
                    fe_lsp <= '0'; fe_scp <= '0';
                    fe_mode <= "100"; fe_c <= (others => '0'); fe_j <= (others => '0'); fe_seq <= "01";
                    v_y32 := f_yacc + resize(c_ky, 32);
                    f_yacc <= v_y32; f_yh <= v_y32(31 downto 16);
                elsif fe_scp = '1' then
                    fe_scp <= '0';
                    fe_mode <= "100"; fe_c <= (others => '0'); fe_j <= f_next; fe_seq <= "00";
                    if f_next /= 15 then f_next <= f_next + 1; end if;
                end if;
            elsif fe_mode = "001" then
                ----------------------------------------------------------------
                -- RT: entry j: SIN(4j + 2) at 4j; A x it at 4j + 2; SQ(.. >> 18) and the AA shift at 4j + 6;
                -- R x g at 4j + 8; written at 4j + 12
                ----------------------------------------------------------------
                -- (op codes are issued two clocks before the multiplier takes the operands: m_op, then p_op)
                if fe_c(1 downto 0) = "00" and fe_c < 1024 then fe_op <= to_unsigned(17, 5); end if;   -- A sin at 4j + 2
                if fe_c(1 downto 0) = "10" and fe_c >= 6 and fe_c <= 1026 then fe_op <= to_unsigned(18, 5); end if;   -- R g at 4j + 8
                if fe_c(1 downto 0) = "00" then
                    if fe_c < 1024 then sn_a <= fe_c(9 downto 2) & "10"; end if;
                    if fe_c >= 8 and fe_c <= 1028 then f_kb <= f_ka; end if;
                    if fe_c >= 12 then
                        rt_we <= '1'; rt_wa <= resize(shift_right(fe_c - 12, 2), 8);
                        rt_wd <= "000" & f_kb & mu_p(18 downto 8);
                    end if;
                elsif fe_c(1 downto 0) = "10" then
                    if fe_c >= 6 then
                        v_aq := mu_p(26 downto 18);
                        sq_a <= v_aq(7 downto 0);
                        -- the AA ramp: 2^kaa px wide where the edge moves more than 1.5 / 3 / 6 px a field line
                        case to_integer(s_ysl) is
                            when 512 =>
                                if v_aq >= 192 then f_ka <= "11"; elsif v_aq >= 96 then f_ka <= "10";
                                elsif v_aq >= 48 then f_ka <= "01"; else f_ka <= "00"; end if;
                            when 1138 | 960 =>
                                if v_aq >= 173 then f_ka <= "11"; elsif v_aq >= 86 then f_ka <= "10";
                                elsif v_aq >= 43 then f_ka <= "01"; else f_ka <= "00"; end if;
                            when others =>
                                if v_aq >= 192 then f_ka <= "10";
                                elsif v_aq >= 96 then f_ka <= "01"; else f_ka <= "00"; end if;
                        end case;
                    end if;
                end if;
                if fe_c = 1031 then fe_op <= to_unsigned(19, 5); end if;                   -- WT level 0
                if fe_c = 1032 then
                    fe_mode <= "010"; fe_c <= (others => '0'); f_l2 <= (others => '0');
                else
                    fe_c <= fe_c + 1;
                end if;
            elsif fe_mode = "010" then
                ----------------------------------------------------------------
                -- WT: level l: g6q x (l^2 >> 4) at 4l; u = min(4095, .. >> 8) at 4l + 4; span x u at 4l + 6;
                -- WT(l) = .. >> 12 at 4l + 10
                ----------------------------------------------------------------
                if fe_c(1 downto 0) = "10" and fe_c <= 1018 then fe_op <= to_unsigned(19, 5); end if;   -- gain l^2 at 4l
                if fe_c(1 downto 0) = "00" and fe_c >= 4 and fe_c <= 1024 then fe_op <= to_unsigned(20, 5); end if;   -- span u at 4l + 6
                if fe_c(1 downto 0) = "00" then
                    if fe_c < 1024 then
                        f_l2 <= f_l2 + resize(fe_c(9 downto 2) & '1', 16);    -- + 2l + 1
                    end if;
                    if mu_p(26 downto 20) /= 0 then f_u <= (others => '1'); else f_u <= mu_p(19 downto 8); end if;
                elsif fe_c(1 downto 0) = "10" then
                    if fe_c >= 10 then
                        wt_we <= '1'; wt_wa <= resize(shift_right(fe_c - 10, 2), 8);
                        wt_wd <= resize(mu_p(22 downto 12), 16);
                    end if;
                end if;
                if fe_c = 1030 then fe_op <= to_unsigned(21, 5); end if;                   -- D slot 0's index product
                if fe_c = 1030 then fe_mode <= "011"; fe_c <= (others => '0'); fe_j <= (others => '0');
                else fe_c <= fe_c + 1; end if;
            elsif fe_mode = "011" then
                ----------------------------------------------------------------
                -- D: slot j: theta = phase + Spread index; V = Depth sin(theta) (Slip, cycles Q16) or
                -- V x R / 2^14 (Swell, px Q2); sign-magnitude
                ----------------------------------------------------------------
                case to_integer(fe_c(4 downto 0)) is
                    when 0 =>
                        v_i := c_i0 + signed(resize(fe_j(3 downto 0), 6));
                        if s_lay = '1' then
                            v_ix := shift_right(v_i + 1, 1);
                            if s_rad = '1' and v_ix(5) = '1' then v_ix := -v_ix; end if;
                        elsif s_rad = '1' then
                            if v_i(5) = '1' then v_ix := not v_i; else v_ix := v_i; end if;
                        else
                            v_ix := v_i;
                        end if;
                        f_idx <= v_ix;
                        if v_ix(5) = '1' then f_idxm <= unsigned(-v_ix(4 downto 0)); else f_idxm <= unsigned(v_ix(4 downto 0)); end if;
                    when 5 => f_th <= f_addsub(c_ph(31 downto 16), mu_p(14 downto 0) & '0', f_idx(5));
                    when 6 => sn_a <= f_fold(f_th); f_vsg <= f_th(15); fe_op <= to_unsigned(22, 5);
                    when 11 => fe_op <= to_unsigned(23, 5);
                    when 12 => f_V <= mu_p(26 downto 12);
                    when 17 => if s_swell = '1' then f_V <= resize(mu_p(26 downto 14), 15); end if;
                    when 18 =>
                        d_we <= '1'; d_wa <= resize(fe_j(3 downto 0), 8);
                        d_wd <= f_vsg & f_V;
                    when others => null;
                end case;
                if fe_c(4 downto 0) = 19 then
                    fe_op <= to_unsigned(21, 5);                                          -- the next slot's index product
                    fe_c <= (others => '0');
                    fe_j <= fe_j + 1;
                    if fe_j(3 downto 0) = 15 then
                        -- then line 0's slots 0 and 1 (the braid phase at line 0 is 0)
                        fe_mode <= "100"; fe_j <= (others => '0'); fe_seq <= "01";
                        f_yh <= (others => '0');
                    end if;
                else
                    fe_c <= fe_c + 1;
                end if;
            elsif fe_mode = "100" then
                ----------------------------------------------------------------
                -- slot fetch: slot j (plait i0 + j), strands 0 .. 2, 16 clocks.  The words go field by field
                -- straight into ST (f_w0 .. 2; CUR took the previous slot's words before this fetch began):
                --   xs' (12) rh (11) kaa (2) 0000 rank (2) en (1)
                ----------------------------------------------------------------
                -- one adder for the three radii (fc 8 .. 10, from f_rq: the RT word a clock after its read) and one
                -- for the three centres xs' = H +- W |sin t| >> 14 (fc 11 .. 13; the sign is the half-cycle's,
                -- flipped for a mirrored plait); one adder makes the three phases t_s = psi + s/3 (or s/2) at fc 5 .. 7
                f_rq <= rt_q(12 downto 0);
                v_rh := signed(resize(f_rq(10 downto 0), 14)) + resize(f_ct, 14);
                if v_rh < 6 then v_rhu := to_unsigned(6, 11);
                else v_rhu := unsigned(v_rh(10 downto 0)); end if;
                case to_integer(fe_c(1 downto 0)) is
                    when 3      => v_sg := f_sgn(0);
                    when 0      => v_sg := f_sgn(1);
                    when others => v_sg := f_sgn(2);
                end case;
                v_xc := f_addsub(resize(c_P8(17 downto 7), 12), mu_p(25 downto 14), v_sg xor f_mir);
                if fe_c(1 downto 0) = "01" then v_tq := to_unsigned(0, 16);
                elsif fe_c(1 downto 0) = "10" then
                    if s_two = '1' then v_tq := to_unsigned(32768, 16); else v_tq := to_unsigned(21845, 16); end if;
                else
                    if s_two = '1' then v_tq := to_unsigned(0, 16); else v_tq := to_unsigned(43691, 16); end if;
                end if;
                v_tp := f_psi + v_tq;
                case to_integer(fe_c(3 downto 0)) is
                    when 0 =>
                        d_a <= resize(fe_j(3 downto 0), 8);
                        cm_a2 <= unsigned'(0 => not s_wb) & "000" & fe_j(3 downto 0);
                        v_i := c_i0 + signed(resize(fe_j(3 downto 0), 6));
                        f_mir <= (s_lay and v_i(0)) or (s_rad and not s_lay and v_i(5));
                    when 2 =>
                        f_d <= d_q; wt_a <= cmf_q(11 downto 4);
                    when 3 =>
                        v_tb := f_yh + c_ph(31 downto 16);
                        if s_swell = '1' then
                            f_psi <= v_tb; f_sw <= signed(f_d);
                        else
                            f_psi <= f_addsub(v_tb, '0' & f_d(14 downto 0), f_d(15)); f_sw <= (others => '0');
                        end if;
                    when 4 =>
                        f_cw <= wt_q(10 downto 0);
                        -- depth order: 3 strands by the sector of 2 psi (the heights sin 4 pi t_s cross at its
                        -- edges); 2 strands: strand 0 in front while cos 2 pi psi > 0
                        -- (ranks and enables straight into the words)
                        f_w0(0) <= '1'; f_w1(0) <= '1'; f_w2(0) <= not s_two;
                        if s_two = '1' then
                            if f_psi < 16384 or f_psi > 49152 then f_w0(2 downto 1) <= "01"; f_w1(2 downto 1) <= "00";
                            else f_w0(2 downto 1) <= "00"; f_w1(2 downto 1) <= "01"; end if;
                            f_w2(2 downto 1) <= "00";
                        else
                            v_u := f_psi(14 downto 0) & '0';
                            if v_u < 5462 or v_u >= 60075 then
                                f_w0(2 downto 1) <= "01"; f_w1(2 downto 1) <= "00"; f_w2(2 downto 1) <= "10";
                            elsif v_u < 16384 then
                                f_w0(2 downto 1) <= "10"; f_w1(2 downto 1) <= "00"; f_w2(2 downto 1) <= "01";
                            elsif v_u < 27307 then
                                f_w0(2 downto 1) <= "10"; f_w1(2 downto 1) <= "01"; f_w2(2 downto 1) <= "00";
                            elsif v_u < 38230 then
                                f_w0(2 downto 1) <= "01"; f_w1(2 downto 1) <= "10"; f_w2(2 downto 1) <= "00";
                            elsif v_u < 49152 then
                                f_w0(2 downto 1) <= "00"; f_w1(2 downto 1) <= "10"; f_w2(2 downto 1) <= "01";
                            else
                                f_w0(2 downto 1) <= "00"; f_w1(2 downto 1) <= "01"; f_w2(2 downto 1) <= "10";
                            end if;
                        end if;
                    when 5 =>
                        if f_sw(15) = '1' then v_ct := signed(resize(f_cw, 13)) - signed(resize(unsigned(f_sw(14 downto 0)), 13));
                        else v_ct := signed(resize(f_cw, 13)) + signed(resize(unsigned(f_sw(14 downto 0)), 13)); end if;
                        if v_ct > signed(resize(c_span, 13)) then f_ct <= signed(resize(c_span, 13)); else f_ct <= v_ct; end if;
                    when others => null;
                end case;
                -- fc 5 .. 7: the strands' SIN and RT addresses; SIN(t_s) is in sn_q two clocks later, when the
                -- multiplier takes it (op 24 issued at fc 5, 6, 7, two clocks ahead; products at fc 11, 12, 13)
                if fe_c(3 downto 0) = 5 or fe_c(3 downto 0) = 6 or fe_c(3 downto 0) = 7 then
                    sn_a <= f_fold(v_tp); rt_a <= f_tgi(v_tp); fe_op <= to_unsigned(24, 5);
                    f_sgn <= f_sgn(1 to 2) & v_tp(15);
                end if;
                case to_integer(fe_c(3 downto 0)) is
                    when 8 =>
                        f_w0(19 downto 7) <= v_rhu & f_rq(12 downto 11);
                    when 9 =>
                        f_w1(19 downto 7) <= v_rhu & f_rq(12 downto 11);
                    when 10 =>
                        f_w2(19 downto 7) <= v_rhu & f_rq(12 downto 11);
                    when 11 =>
                        f_w0(31 downto 20) <= v_xc;
                    when 12 =>
                        f_w1(31 downto 20) <= v_xc;
                    when 13 =>
                        f_w2(31 downto 20) <= v_xc;
                    when others => null;
                end case;
                if fe_c(3 downto 0) = 15 then
                    fe_c <= (others => '0');
                    if fe_seq = "01" then
                        fe_seq <= "10"; fe_j <= to_unsigned(1, 5); f_next <= to_unsigned(2, 5);
                        fe_xf_req <= '1';
                    else
                        fe_mode <= "000";
                    end if;
                else
                    fe_c <= fe_c + 1;
                end if;
            end if;
        end if;
    end process p_fe;

    ---------------------------------------------------------------------------
    -- [F] BEAM + SLOT WORDS.  The beam (stage 8, loaded 7 clocks after the line
    -- start): pixel offset from the slot's left boundary, px Q8.  ST takes the
    -- fetch's words; CUR takes ST at the line start and as the beam steps into
    -- each new slot.
    ---------------------------------------------------------------------------
    p_rd : process(clk)
        variable v_n : unsigned(18 downto 0);
        variable v_w : unsigned(18 downto 0);
        variable v_wrap : boolean;
    begin
        if rising_edge(clk) then
            b_xf <= '0';
            b_rnd_d <= b_rnd;
            v_wrap := false;
            b_smp <= '0';
            if s_st(6) = '1' then
                b_v <= c_v0;
                b_slot <= (others => '0');
                b_smp <= c_s0;
            else
                v_n := resize(b_v, 19) + 256;
                v_w := v_n - resize(c_P8, 19);
                if v_w(18) = '0' then
                    b_v <= v_w(17 downto 0); v_wrap := true;
                    if b_slot /= 15 then b_slot <= b_slot + 1; end if;
                else
                    b_v <= v_n(17 downto 0);
                    -- the camera sample: the pixel whose integer beam position is the centre's
                    if b_v(17 downto 8) = c_cm1 and b_slot /= 15 then b_smp <= '1'; end if;
                end if;

            end if;
            if fe_xf = '1' or (v_wrap and b_rnd = '1') then
                r_cur(0) <= f_w0; r_cur(1) <= f_w1; r_cur(2) <= f_w2;
            end if;
            if v_wrap and b_rnd = '1' then b_xf <= '1'; end if;
        end if;
    end process p_rd;

    ---------------------------------------------------------------------------
    -- [F] PIXEL PATH (stages 9 .. 29)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a  : signed(12 downto 0);
        variable v_e10 : signed(12 downto 0);
        variable v_in, v_ab : std_logic_vector(0 to 2);
        variable v_rk : unsigned(1 downto 0);
        variable v_vs, v_bs, v_hs : integer range 0 to 2;
        variable v_vf, v_bf, v_hf : boolean;
        variable v_dd : signed(13 downto 0);
        variable v_c  : signed(13 downto 0);
        variable v_t  : unsigned(10 downto 0);
        variable v_k  : signed(9 downto 0);
        variable v_g  : signed(14 downto 0);
        variable v_by : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 9: distances to the three strand centres (ones' complement |a|); the slot's fields
            for s in 0 to 2 loop
                v_a := signed(resize(b_v(17 downto 6), 13)) - signed(resize(r_cur(s)(31 downto 20), 13));
                if v_a(12) = '0' then p9_d(s) <= unsigned(v_a(11 downto 0)); else p9_d(s) <= unsigned(not v_a(11 downto 0)); end if;
                p9_rh(s) <= r_cur(s)(19 downto 9);
                p9_kaa(s) <= r_cur(s)(8 downto 7);
                p9_rk(s) <= r_cur(s)(2 downto 1);
                p9_en(s) <= r_cur(s)(0);
            end loop;
            -- 10: edge distances e = rh - d (px Q2)
            for s in 0 to 2 loop
                v_e10 := signed(resize(p9_rh(s), 13)) - signed(resize(p9_d(s), 13));
                p10_e(s) <= v_e10;
                -- the inside test here (e >= -1), so stage 11 starts from registers
                if p9_en(s) = '1' and v_e10 >= -1 then p10_in(s) <= '1'; else p10_in(s) <= '0'; end if;
                p10_rh(s) <= p9_rh(s); p10_kaa(s) <= p9_kaa(s);
                p10_rk(s) <= p9_rk(s); p10_en(s) <= p9_en(s);
            end loop;
            -- 11: inside tests; the visible strand (highest rank inside) and its values; the one beneath it;
            -- of the strands above it, the one whose edge is nearest (largest e) casts the shadow
            for s in 0 to 2 loop
                v_in(s) := p10_in(s);
            end loop;
            v_vf := false; v_vs := 0; v_rk := "00";
            for s in 0 to 2 loop
                if v_in(s) = '1' and (not v_vf or p10_rk(s) > v_rk) then v_vf := true; v_vs := s; v_rk := p10_rk(s); end if;
            end loop;
            v_bf := false; v_bs := 0;
            for s in 0 to 2 loop
                if v_in(s) = '1' and p10_rk(s) < v_rk and (not v_bf or p10_rk(s) > p10_rk(v_bs)) then v_bf := true; v_bs := s; end if;
            end loop;
            for s in 0 to 2 loop
                if p10_en(s) = '1' and p10_rk(s) > v_rk and v_vf then v_ab(s) := '1'; else v_ab(s) := '0'; end if;
            end loop;
            v_hf := false; v_hs := 0;
            for s in 0 to 2 loop
                if p10_en(s) = '1' and p10_rk(s) = v_rk + 1 and v_vf then v_hf := true; v_hs := s; end if;
            end loop;
            p11_rh <= p10_rh(v_vs); p11_e <= p10_e(v_vs);
            p11_kaa <= p10_kaa(v_vs);
            p11_esh <= p10_e(v_hs);
            if v_hf then p11_hab <= '1'; else p11_hab <= '0'; end if;
            pc_vs(11) <= to_unsigned(v_vs, 2);
            if v_vf then pc_any(11) <= '1'; else pc_any(11) <= '0'; end if;
            if v_bf then pc_bv(11) <= '1'; else pc_bv(11) <= '0'; end if;
            -- 12: divider set-up (q = 128 max(e, 0) / rh); coverage; the shadow distance scaled by its width
            if p11_e(12) = '0' and p11_e(11 downto 1) >= signed(resize(p11_rh(10 downto 1), 11)) then
                pv_eq(12) <= '1'; pv_r(12) <= (others => '0');
            else
                pv_eq(12) <= '0';
                if p11_e(12) = '1' then pv_r(12) <= (others => '0'); else pv_r(12) <= unsigned(p11_e(10 downto 1)); end if;
            end if;
            pv_d(12) <= p11_rh(10 downto 1); pv_q(12) <= (others => '0');
            v_c := shift_right(shift_left(resize(p11_e, 14), 1), to_integer(p11_kaa)) + 4;
            if v_c < 0 then pc_cov(12) <= (others => '0');
            elsif v_c > 8 then pc_cov(12) <= to_unsigned(8, 4);
            else pc_cov(12) <= unsigned(v_c(3 downto 0)); end if;
            -- r = 4 dist >> nsh as its low 5 bits and an r >= 16 flag (dist >= 2^(nsh + 2))
            v_dd := -resize(p11_esh, 14) - 2;
            if v_dd(13) = '1' then v_dd := (others => '0'); end if;
            case to_integer(c_nsh(1 downto 0)) is
                when 0      => p12_r5 <= unsigned(v_dd(2 downto 0)) & "00";
                when 1      => p12_r5 <= unsigned(v_dd(3 downto 0)) & '0';
                when 2      => p12_r5 <= unsigned(v_dd(4 downto 0));
                when others => p12_r5 <= unsigned(v_dd(5 downto 1));
            end case;
            if v_dd(12 downto 5) /= 0 or (v_dd(4) = '1' and c_nsh(1 downto 0) /= 3)
               or (v_dd(3) = '1' and c_nsh(1) = '0') or (v_dd(2) = '1' and c_nsh(1 downto 0) = 0) then
                p12_big <= '1';
            else
                p12_big <= '0';
            end if;
            p12_hab <= p11_hab;
            -- 13: the shadow in shade steps: ramp sh = 16 - r (16 at the edge of the strand above, 0 one shadow
            -- width away), x 2.75 (2 sh + sh - sh / 4), as a table of r; 13 -> 21 through the pixel-state delay
            -- line, then a fabric register (22)
            if p12_hab = '0' or p12_big = '1' then p13_kzs <= (others => '0');
            else p13_kzs <= C_KZS(to_integer(p12_r5(3 downto 0))); end if;
            for i in 12 to 13 loop
                pc_vs(i) <= pc_vs(i - 1); pc_any(i) <= pc_any(i - 1); pc_bv(i) <= pc_bv(i - 1);
            end loop;
            pc_cov(13) <= pc_cov(12);
            p22_vs <= dlp_q(15 downto 14); p22_win <= dlp_q(12); p22_bv <= dlp_q(11);
            p22_any <= dlp_q(10); p22_cov <= dlp_q(9 downto 6); p22_kzs <= dlp_q(5 downto 0);
            pc_any(23) <= p22_any; pc_bv(23) <= p22_bv; pc_cov(23) <= p22_cov;
            for i in 24 to 28 loop
                pc_any(i) <= pc_any(i - 1); pc_bv(i) <= pc_bv(i - 1); pc_cov(i) <= pc_cov(i - 1);
            end loop;
            -- 13 .. 19: the quotient (7 bits), restoring; 20: a copy
            for i in 13 to 18 loop
                v_t := pv_r(i - 1) & '0';
                if v_t >= resize(pv_d(i - 1), 11) then
                    pv_r(i) <= resize(v_t - resize(pv_d(i - 1), 11), 10);
                    pv_q(i) <= pv_q(i - 1)(5 downto 0) & '1';
                else
                    pv_r(i) <= v_t(9 downto 0);
                    pv_q(i) <= pv_q(i - 1)(5 downto 0) & '0';
                end if;
                pv_d(i) <= pv_d(i - 1);
                pv_eq(i) <= pv_eq(i - 1);
            end loop;
            v_t := pv_r(18) & '0';
            if v_t >= resize(pv_d(18), 11) then pv_q(19) <= pv_q(18)(5 downto 0) & '1';
            else pv_q(19) <= pv_q(18)(5 downto 0) & '0'; end if;
            pv_eq(19) <= pv_eq(18);
            pv_q(20) <= pv_q(19); pv_eq(20) <= pv_eq(19);
            -- 21: the profile (C_BR_TAB 896 + q, read outside the zone; 127 when e >= rh)
            -- 22: fabric copy
            p22_kq <= r_tq(7 downto 0);
            -- 23: the shade step k = profile - depth - shadow -> colour-table addresses
            v_k := signed(resize(p22_kq, 10)) - signed(resize(p22_kzs, 10));
            if v_k < 0 then v_k := (others => '0'); elsif v_k > 127 then v_k := to_signed(127, 10); end if;
            cty_a <= p22_vs & unsigned(v_k(6 downto 0));
            p23_ca <= p22_vs & unsigned(v_k(6 downto 1));
            -- the chroma tables are read three stages later (address at 26), so only their 8-bit address is
            -- carried, not the 16-bit colour
            p24_ca <= p23_ca; p25_ca <= p24_ca; ctu_a <= p25_ca;
            -- 24: the luma table read; 25: fabric copy, the luma beneath
            p25_ty <= cty_q(9 downto 0);
            if pc_bv(24) = '1' then p25_by <= to_unsigned(64, 10); else p25_by <= c_gy; end if;
            -- 26: top - beneath (luma)
            p26_dy <= signed(resize(p25_ty, 11)) - signed(resize(p25_by, 11));
            -- 27: difference x coverage (0 .. 8) as two gated sums
            if pc_cov(26)(3) = '1' then
                p27_ay <= shift_left(resize(p26_dy, 15), 3); p27_ey <= (others => '0');
            else
                v_g := (others => '0');
                if pc_cov(26)(2) = '1' then v_g := shift_left(resize(p26_dy, 15), 2); end if;
                if pc_cov(26)(1) = '1' then p27_ay <= v_g + shift_left(resize(p26_dy, 15), 1); else p27_ay <= v_g; end if;
                if pc_cov(26)(0) = '1' then p27_ey <= resize(p26_dy, 15); else p27_ey <= (others => '0'); end if;
            end if;
            -- 28
            p28_y <= p27_ay + p27_ey;
            -- 27: the chroma table reads; 28: chroma = the top colour where the coverage is at least half, else
            -- the colour beneath (the output's 9-tap chroma filter is the AA)
            if pc_cov(27)(3 downto 2) /= "00" then
                p28_c <= ctu_q & ctv_q;
            elsif pc_bv(27) = '1' then
                p28_c <= x"8080";
            else
                p28_c <= c_gu(9 downto 2) & c_gv(9 downto 2);
            end if;
            -- 29: beneath (selected again) + (difference x coverage + 4) / 8; the ground with no strand, or
            -- outside pixels 0 .. W + 7
            if pc_bv(28) = '1' then v_by := to_unsigned(64, 10); else v_by := c_gy; end if;
            if pc_any(28) = '1' and p22_win = '1' then
                v_g := signed(resize(v_by, 15)) + shift_right(p28_y + 4, 3); s29_y <= unsigned(v_g(9 downto 0));
                s29_u <= p28_c(15 downto 8) & "00"; s29_v <= p28_c(7 downto 0) & "00";
            else
                s29_y <= c_gy; s29_u <= c_gu; s29_v <= c_gv;
            end if;
            s_hist <= s_hist(6 downto 0) & p22_win;
        end if;
    end process p_pix;
    -- the pixel-state delay line: in at stage 13, out at stage 21; bit 12 carries the stage-29 window
    -- (generated avid 26 or 34 clocks late = 17 or 25, 8 clocks later, then the p22 register)
    dlp_wd <= pc_vs(13) & '0' & (s_gav_sr(17) or s_gav_sr(25)) & pc_bv(13) & pc_any(13) & pc_cov(13) & p13_kzs;
    -- the beam's window: pixels 0 .. W + 3 at stage 8 (the generated avid 7 and 11 clocks late), its end
    b_rnd <= s_gav_sr(6) or s_gav_sr(10);
    b_end <= b_rnd_d and not b_rnd;

    ---------------------------------------------------------------------------
    -- [G] OUTPUT CONDITIONING (stages 30 .. 33, riley's p_out shifted by 4; the trailing chroma taps and
    -- the sync from block-RAM delay lines)
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- running 8-sums; the trailing taps (8 clocks back) come out of the chroma delay line
            if s_hist = "00000000" then
                s_bu <= (c_gu & "000") + 4; s_bv2 <= (c_gv & "000") + 4;
            else
                s_bu  <= s_bu + s29_u - (dlc_q(15 downto 8) & "00");
                s_bv2 <= s_bv2 + s29_v - (dlc_q(7 downto 0) & "00");
            end if;
            s_bud <= s_bu; s_bvd <= s_bv2;
            s_yd(1) <= s29_y;
            for i in 2 to 5 loop s_yd(i) <= s_yd(i - 1); end loop;
            s30_ya <= resize(s_yd(3), 12) + resize(s_yd(5), 12) + 2;
            s30_yb <= s_yd(4);
            v_sc := resize(s_bu, 14) + s_bud;  s31_u <= v_sc(13 downto 4);
            v_sc := resize(s_bv2, 14) + s_bvd; s31_v <= v_sc(13 downto 4);
            v_y := s30_ya + shift_left(resize(s30_yb, 12), 1);
            s31_y <= v_y(11 downto 2);
            if s31_y > 64 then v_dl := s31_y - 64; else v_dl := (others => '0'); end if;
            s32_y <= s31_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s32_u <= s31_u; s32_v <= s31_v;
            if dls_q(0) = '1' then
                s33_y <= s32_y; s33_u <= s32_u; s33_v <= s32_v;
            else
                s33_y <= to_unsigned(64, 10); s33_u <= to_unsigned(512, 10); s33_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    dlc_wd <= s29_u(9 downto 2) & s29_v(9 downto 2);

    data_out.y       <= std_logic_vector(s33_y);
    data_out.u       <= std_logic_vector(s33_u);
    data_out.v       <= std_logic_vector(s33_v);
    data_out.hsync_n <= dls_q2(2);
    data_out.vsync_n <= dls_q2(1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= dls_q2(0);

end architecture braid;
