-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Meander v0.1.0 -- Straightedge.  Friezes of the Greek key (the meander borders of Attic pottery and mosaic
-- floors, Frank Stella's pinstripes) stacked up the screen, alternate rows mirrored and running the opposite
-- way.  One continuous line: a square wave whose every box holds a square spiral that grows out of the wave
-- and winds to the eye of the box.  Luma only: U = V = 512 on every pixel.  Every edge is a straight line.
--
--   unit       16 x 8 nodes; a ROM gives each node its arms (N / E / S / W), its order along the spiral (0 .. 49)
--              and the headings in and out.  The line is the set of points within h (Chebyshev) of the path
--              up to arc length L (the Wind), so it grows along itself and ends in a square cap.
--   row r      u = (x - centre) / pitch -+ (X + drift_r) +- slant (v - 4)      nodes, modulo 16
--              v = (y - centre) / pitch + pan                                   nodes, modulo the row period
--              odd rows are mirrored (a mirrored copy of the ROM) and lean the other way
--   coverage   every edge is u = const or v = const in its row: a wrapping DDA runs across each node in
--              anti-aliasing units (one unit = the pixel's footprint across the edge), so every coverage is
--              clamp(distance + 1/2) by adds only (17 steps); bars combine by min, arms by max
--
--   K1  X        slides the alternate rows against each other (+-3.5 nodes each)
--   K2  Y        pans the stack (+-1.5 rows)
--   K3  Scale    node pitch 10.8 .. 108 virtual px (exponential): about ten rows .. one giant frieze
--   K4  Slant    bipolar shear of each row, centre = the right-angle key, +-45 degrees at the ends
--   K5  Drift    bipolar speed (Hypnos law, 2^-11 .. 2^-5 unit per field); rows run at +1, -3/4, +2/3, -1/2
--   K6  Camera   the line's weight per node, from the block-held luma (a mosaic; never a bend)
--   S7  Glide    Off / On      S8 Form  Frieze (plain bands between rows) / Carpet (rows abut)
--   S9  Tone     Grey (alternate rows white on black / light on dark grey) / Ink (white on black)
--   S10 Weight   Bold (line = gap) / Fine (1/6 duty, at least 2 virtual px)      S11 Source  Video / Osc
--   P12 WIND     how far the spiral is wound: 0 = the bare square wave, 50 % = a single hook, 100 % = the full key
--
-- The camera: the source field (luma or the Osc lattice -> box -> vertical IIR) is written, 8 bits, into a
-- 60 x 34 grid of 32 x 32 virtual-px blocks in block RAM (two banks ping-ponged per field).  The next field
-- reads the block under each NODE's centre (constant across the node on every line of it) and a per-field
-- table turns that level into the node's half line width.  All edges stay straight; the only new edges are
-- the steps in line weight at node borders.
--
-- The chassis is the Kinetic riley shell (via Vellum rosette):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         +-1-count holds, glides, switches, the Drift speed (Hypnos)
--   [C] ENGINE                           per-field sequencer (one serial multiplier, one serial divider), table
--                                        fill, line engine (one serial multiplier, computes the NEXT line)
--   [D] SOURCE FIELD                     luma (or the Osc lattice) -> box -> vertical IIR -> the block grid
--   [E] KEY                              node DDA, grid and weight tables, node ROM, coverage, tone
--   [F] OUTPUT CONDITIONING              luma [1 2 1]/4, 13/16 toward black, blanking gate
--
-- Stage numbering: stage 1 is the generated avid's own clock; a register "at stage S" holds pixel n at
-- T + S + n (T = the g_start clock).  Pixel n sits at x = x0 + n xs half virtual px.  The output register is
-- at stage C_LAT - 3 and the generated avid lands 4 clocks after the delayed source avid (the shell's
-- parity-locked start).  s_gav_sr(S - 2) flags pixels 0 .. W - 1 at stage S.  One latency (24) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.meander_pkg.all;
use work.all;

architecture meander of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

    -- TOML defaults: the glides and holds start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 407;
    constant C_DEF_K4  : integer := 512;
    constant C_DEF_K5  : integer := 660;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 1023;

    constant C_NSLOT : integer := 14;      -- serial-product slots 0 .. 14
    constant C_TF_ZC : integer := 410;     -- the weight-table fills start here (after slot 14)
    constant C_LK_ZC : integer := 960;     -- the line engine's kick for line 0 (after the fills)

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
    -- 2^23 / (the line step): anti-aliasing units per virtual line
    function f_vrecip(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(7371, 16);
            when 4          => return to_unsigned(14564, 16);
            when 8          => return to_unsigned(8738, 16);
            when 12         => return to_unsigned(17476, 16);
            when 5 | 6 | 14 => return to_unsigned(21845, 16);
            when 1 | 2 | 10 => return to_unsigned(16384, 16);
            when others     => return to_unsigned(32768, 16);
        end case;
    end function;
    -- x step in h (half virtual px) per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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
    -- a + b or a - b on ONE carry chain (the low column makes the carry-in)
    function f_addsub(a, b : unsigned; sub : std_logic) return unsigned is
        variable v_a, v_b, v_r : unsigned(a'length + 1 downto 0);
    begin
        v_a := '0' & a & '1';
        if sub = '1' then v_b := '0' & (not b) & '1'; else v_b := '0' & b & '0'; end if;
        v_r := v_a + v_b;
        return v_r(a'length downto 1);
    end function;
    -- coverage clamp(x, 0, 16)
    function f_sat(x : signed(14 downto 0)) return unsigned is
    begin
        if x(14) = '1' then return to_unsigned(0, 5); end if;
        if x(13 downto 4) /= "0000000000" then return to_unsigned(16, 5); end if;
        return unsigned(x(4 downto 0));
    end function;
    -- a strip between two edges: clamp(a + b - 16, 0, 16)
    function f_strip(a, b : unsigned(4 downto 0)) return unsigned is
        variable v : unsigned(5 downto 0);
    begin
        v := resize(a, 6) + resize(b, 6);
        if v(5) = '1' then return to_unsigned(16, 5); end if;
        if v(4) = '0' then return to_unsigned(0, 5); end if;
        return '0' & v(3 downto 0);
    end function;
    function f_min(a, b : unsigned(4 downto 0)) return unsigned is
    begin
        if a < b then return a; else return b; end if;
    end function;
    function f_max(a, b : unsigned(4 downto 0)) return unsigned is
    begin
        if a > b then return a; else return b; end if;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_rvr  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal w_dxm  : unsigned(10 downto 0);     -- the beam's pre-pixel: its distance left of the centre (h)
    signal w_xp   : signed(17 downto 0);       -- the pre-pixel's x in h, Q4
    signal w_xq   : unsigned(11 downto 0);     -- pixel 0's x in h

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

    -- [B] knobs: +-1-count holds, glide accumulators (Q10.6), switches, the Drift speed glide (Q6)
    signal s_h1 : unsigned(9 downto 0) := to_unsigned(C_DEF_K1, 10);
    signal s_h2 : unsigned(9 downto 0) := to_unsigned(C_DEF_K2, 10);
    signal s_h3 : unsigned(9 downto 0) := to_unsigned(C_DEF_K3, 10);
    signal s_h4 : unsigned(9 downto 0) := to_unsigned(C_DEF_K4, 10);
    signal s_h6 : unsigned(9 downto 0) := to_unsigned(C_DEF_K6, 10);
    signal s_hp : unsigned(9 downto 0) := to_unsigned(C_DEF_P12, 10);
    signal s_a1 : unsigned(15 downto 0) := to_unsigned(C_DEF_K1 * 64, 16);
    signal s_a2 : unsigned(15 downto 0) := to_unsigned(C_DEF_K2 * 64, 16);
    signal s_a3 : unsigned(15 downto 0) := to_unsigned(C_DEF_K3 * 64, 16);
    signal s_a4 : unsigned(15 downto 0) := to_unsigned(C_DEF_K4 * 64, 16);
    signal s_a6 : unsigned(15 downto 0) := to_unsigned(C_DEF_K6 * 64, 16);
    signal s_ap : unsigned(15 downto 0) := to_unsigned(C_DEF_P12 * 64, 16);
    signal s_gm : unsigned(15 downto 0) := (others => '0');
    signal s_gr, s_gh, s_gt : unsigned(9 downto 0) := (others => '0');
    signal s_hd : signed(10 downto 0) := (others => '0');
    signal s_gd : signed(16 downto 0) := (others => '0');
    signal z_gp : unsigned(2 downto 0) := (others => '0');
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_glide : std_logic := '1';
    signal s_carpet, s_ink, s_fine, s_src : std_logic := '0';
    signal s_bank : std_logic := '0';                               -- the grid bank being written this field
    signal s_k5  : unsigned(9 downto 0) := to_unsigned(C_DEF_K5, 10);
    signal s_vtg : signed(16 downto 0) := (others => '0');
    signal s_vgs : signed(16 downto 0) := to_signed((C_DEF_K5 - 512) * 64, 17);
    signal s_vneg, s_vz : std_logic := '0';
    signal s_av  : unsigned(9 downto 0) := (others => '0');
    signal s_u   : unsigned(9 downto 0) := (others => '0');
    signal s_ls  : unsigned(9 downto 0) := (others => '0');
    signal s_scnt : unsigned(2 downto 0) := (others => '0');

    -- [C] ROMs: C_MD_TAB (2^(i/256), then the tone table), C_MD_KEY (the node records)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal kr_a : unsigned(7 downto 0) := (others => '0');
    signal kr_q, kr_f : unsigned(15 downto 0) := (others => '0');
    signal tn_a : unsigned(5 downto 0) := (others => '0');
    signal w_ra : unsigned(8 downto 0);

    -- [C] per-field serial multiplier, slot counter and scratch
    signal f_a, f_acc : unsigned(33 downto 0) := (others => '0');
    signal z_fa : unsigned(23 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_ph   : unsigned(4 downto 0) := (others => '0');
    signal z_slot : unsigned(3 downto 0) := (others => '0');
    signal z_run, z_pre, z_ld : std_logic := '0';
    signal z_oct  : unsigned(10 downto 0) := (others => '0');
    signal z_shi  : unsigned(2 downto 0) := (others => '0');
    signal z_mag  : unsigned(15 downto 0) := (others => '0');
    signal z_m    : unsigned(14 downto 0) := (others => '0');
    signal z_t, z_h1, z_hm, z_hb : unsigned(15 downto 0) := (others => '0');
    signal z_fl   : unsigned(8 downto 0) := (others => '0');
    signal z_xs   : unsigned(19 downto 0) := (others => '0');      -- the X slide, nodes Q16 (two's complement)
    signal z_yo   : signed(21 downto 0) := (others => '0');        -- the Y pan, nodes Q16
    signal z_vt   : unsigned(26 downto 0) := (others => '0');      -- line 0's v + 8 row periods, nodes Q20
    signal z_g    : unsigned(14 downto 0) := (others => '0');      -- camera gain
    signal z_L    : unsigned(15 downto 0) := (others => '0');      -- the wind, nodes Q10
    signal z_Lf   : unsigned(9 downto 0) := (others => '0');
    signal z_spu, z_spv : unsigned(13 downto 0) := (others => '0');

    -- [C] serial divider: R = 2^24 / F (the reciprocal of the pixel's footprint across a leaning edge)
    signal dv_R   : unsigned(12 downto 0) := (others => '0');
    signal dv_Q   : unsigned(15 downto 0) := (others => '0');
    signal dv_n   : unsigned(4 downto 0) := (others => '0');
    signal dv_st  : std_logic := '0';

    -- [C] per-field constants
    signal c_inv  : unsigned(21 downto 0) := (others => '0');      -- nodes per h, Q24
    signal c_P    : unsigned(15 downto 0) := to_unsigned(13824, 16);   -- pitch in h, Q8
    signal c_Sq   : unsigned(14 downto 0) := (others => '0');      -- |slant|, Q15
    signal c_sng  : std_logic := '0';
    signal c_F    : unsigned(12 downto 0) := to_unsigned(512, 13); -- footprint in h, Q8
    signal c_R    : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal c_Ku   : unsigned(23 downto 0) := to_unsigned(1769472, 24);   -- AA units per node across u, Q16
    signal c_Kv   : unsigned(19 downto 0) := to_unsigned(110592, 20);    -- AA units per node down v, Q12
    signal c_dT   : unsigned(17 downto 0) := to_unsigned(65536, 18);     -- AA units per pixel, Q16
    signal c_dTmK : signed(24 downto 0) := (others => '0');
    signal c_dv   : unsigned(19 downto 0) := (others => '0');      -- nodes per line, Q20
    signal c_Ab   : unsigned(19 downto 0) := (others => '0');      -- u at the pre-pixel, nodes Q16 mod 16
    signal c_Li, c_Li1 : unsigned(5 downto 0) := (others => '0');
    signal c_tu1, c_tv1 : signed(13 downto 0) := (others => '0');   -- the entering tip's reach, 1/16 AA
    signal c_LfKu, c_LfKv : unsigned(11 downto 0) := (others => '0');
    signal c_hbu, c_hbv : unsigned(15 downto 0) := (others => '0');     -- the base half width, AA Q8
    signal c_hmu, c_hmv : unsigned(11 downto 0) := (others => '0');     -- its cap, 1/16 AA
    signal c_slu, c_slv : unsigned(16 downto 0) := (others => '0');     -- per camera level, AA Q16
    signal c_spd  : unsigned(16 downto 0) := (others => '0');      -- drift step per field, Q20 units
    signal c_s23  : unsigned(16 downto 0) := (others => '0');
    signal c_sneg : std_logic := '0';
    type t_u20x4 is array (0 to 3) of unsigned(19 downto 0);
    signal c_ph   : t_u20x4 := (others => (others => '0'));        -- drift phases, units of 16 nodes, Q20

    -- [C] weight tables (half line width by camera level, 1/16 AA unit): fill
    signal tf_run, tf_pass, tf_pd : std_logic := '0';
    signal tf_i   : unsigned(7 downto 0) := (others => '0');
    signal tf_acc : unsigned(23 downto 0) := (others => '0');
    signal hk_we  : std_logic := '0';
    signal hk_wa  : unsigned(7 downto 0) := (others => '0');
    signal hk_wd  : unsigned(9 downto 0) := (others => '0');
    type t_hk is array (0 to 255) of unsigned(9 downto 0);
    signal hku_ram, hkv_ram : t_hk := (others => (others => '0'));
    attribute no_rw_check of hku_ram : signal is true;
    attribute no_rw_check of hkv_ram : signal is true;
    signal hu_q, hv_q, hu_f, hv_f : unsigned(9 downto 0) := (others => '0');

    -- [C] line engine
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal l_v    : unsigned(26 downto 0) := (others => '0');      -- the next line's v in its row, nodes Q20
    signal l_rc   : unsigned(3 downto 0) := (others => '0');
    signal l_row  : unsigned(1 downto 0) := (others => '0');
    signal l_y    : unsigned(18 downto 0) := (others => '0');      -- the next line's y, virtual lines Q8
    signal l_pj   : unsigned(3 downto 0) := (others => '0');
    signal l_pjv  : std_logic := '0';
    signal e_mneg, e_dneg, e_xneg : std_logic := '0';
    signal e_mag  : unsigned(11 downto 0) := (others => '0');
    signal e_dmag : unsigned(15 downto 0) := (others => '0');
    signal e_xmag : unsigned(15 downto 0) := (others => '0');
    signal e_A, e_u0 : unsigned(19 downto 0) := (others => '0');
    signal e_shc  : unsigned(17 downto 0) := (others => '0');
    signal e_dl   : unsigned(3 downto 0) := (others => '0');
    signal e_xc   : signed(17 downto 0) := (others => '0');
    signal e_yc   : signed(20 downto 0) := (others => '0');
    signal e_yb, e_ybw : unsigned(5 downto 0) := (others => '0');
    signal m_a, m_acc : unsigned(39 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal n_Tv   : signed(10 downto 0) := (others => '0');
    signal n_i0   : unsigned(3 downto 0) := (others => '0');
    signal n_U0   : unsigned(23 downto 0) := (others => '0');
    signal n_xc0  : signed(17 downto 0) := (others => '0');
    signal n_j    : unsigned(2 downto 0) := (others => '0');
    signal n_band, n_par : std_logic := '0';
    signal n_b60, n_w60 : unsigned(10 downto 0) := (others => '0');

    -- [E] pixel path
    signal b_U    : unsigned(23 downto 0) := (others => '0');
    signal b_i    : unsigned(3 downto 0) := (others => '0');
    signal b_xc   : signed(17 downto 0) := (others => '0');
    signal p_Tv   : signed(10 downto 0) := (others => '0');
    signal p_j    : unsigned(2 downto 0) := (others => '0');
    signal p_band, p_par : std_logic := '0';
    signal p_b60, p_w60 : unsigned(10 downto 0) := (others => '0');
    type t_s13 is array (3 to 8) of signed(10 downto 0);
    signal s_T    : t_s13 := (others => (others => '0'));
    signal s3_i, s4_i, s5_i : unsigned(3 downto 0) := (others => '0');
    signal s3_xb  : unsigned(5 downto 0) := (others => '0');
    signal s4_ga  : unsigned(11 downto 0) := (others => '0');
    signal gr_q, gr_f : unsigned(7 downto 0) := (others => '0');
    signal s9_culo, s9_cuhi, s9_cvlo, s9_cvhi : unsigned(4 downto 0) := (others => '0');
    signal s9_mm  : std_logic_vector(3 downto 0) := (others => '0');
    signal s9_tip, s9_tout : std_logic := '0';
    signal s9_hd  : unsigned(1 downto 0) := (others => '0');
    signal s9_tq  : signed(14 downto 0) := (others => '0');
    signal s9_s   : signed(10 downto 0) := (others => '0');
    signal s10_su, s10_sv, s10_eu, s10_ev, s10_ct, s10_bk : unsigned(4 downto 0) := (others => '0');
    signal s10_he, s10_ve, s10_hz, s10_tip : std_logic := '0';
    signal s11_hb, s11_vb, s11_tb, s11_st : unsigned(4 downto 0) := (others => '0');
    signal s11_tip : std_logic := '0';
    signal s12_m, s12_t : unsigned(4 downto 0) := (others => '0');

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_p, s_pfirst : unsigned(7 downto 0) := (others => '0');
    signal s_avd : std_logic_vector(1 to 5) := (others => '0');    -- data_in.avid delayed
    signal s_col, s_pcol, s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal c_b32 : std_logic := '1';                               -- box length 32 (HD) / 16 pixels (SD)
    signal c_brp : unsigned(7 downto 0) := to_unsigned(225, 8);
    signal c_iir : unsigned(2 downto 0) := to_unsigned(5, 3);
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(5 downto 0) := (others => '0');
    signal s_bsum : unsigned(12 downto 0) := (others => '0');
    signal s_ev, s_fl : std_logic := '0';
    signal s_eb, s_eq : unsigned(11 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(11 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_wd : unsigned(11 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    -- the block grid: 2 banks x 34 rows x 60 blocks, 8 bits
    type t_gr is array (0 to 4095) of unsigned(7 downto 0);
    signal gr_ram : t_gr := (others => (others => '0'));
    attribute no_rw_check of gr_ram : signal is true;
    signal gw_xb : unsigned(5 downto 0) := (others => '0');
    signal gw_a  : unsigned(11 downto 0) := (others => '0');
    signal gw_d  : unsigned(7 downto 0) := (others => '0');
    signal gw_we : std_logic := '0';

    -- [F] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_yd : t_u10(1 to 3) := (others => to_unsigned(64, 10));
    signal s18_ya : unsigned(11 downto 0) := (others => '0');
    signal s18_yb : unsigned(9 downto 0) := (others => '0');
    signal s19_y, s20_y, s21_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_MD_TAB(to_integer(w_ra));
            r_exf  <= r_ex_q;
            kr_q   <= C_MD_KEY(to_integer(kr_a));
        end if;
    end process p_rom;

    -- box delay: read (box length) samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_avd(2) = '1' then bx_ram(to_integer(bx_wp)) <= s_p; end if;
        end if;
    end process p_bxram;

    -- source-field line store (half horizontal resolution): read 3 pixels ahead of the write
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_pcol(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    -- the block grid: this field writes bank s_bank, the pixel path reads the other one
    p_grram : process(clk)
    begin
        if rising_edge(clk) then
            gr_q <= gr_ram(to_integer(s4_ga));
            if gw_we = '1' then gr_ram(to_integer(gw_a)) <= gw_d; end if;
        end if;
    end process p_grram;

    -- weight tables: written in the vblank zone only, read by the pixel path
    p_hkuram : process(clk)
    begin
        if rising_edge(clk) then
            hu_q <= hku_ram(to_integer(gr_f));
            if hk_we = '1' and tf_pd = '0' then hku_ram(to_integer(hk_wa)) <= hk_wd; end if;
        end if;
    end process p_hkuram;
    p_hkvram : process(clk)
    begin
        if rising_edge(clk) then
            hv_q <= hkv_ram(to_integer(gr_f));
            if hk_we = '1' and tf_pd = '1' then hkv_ram(to_integer(hk_wa)) <= hk_wd; end if;
        end if;
    end process p_hkvram;

    -- one ROM, two users: the sequencer's 2^(i/256) in the vblank zone, the pixel path's tone table outside it
    w_ra <= '0' & r_ex_a when s_zone = '1' else "100" & tn_a;

    -- [A] the beam's pre-pixel by video mode
    w_dxm <= to_unsigned(1444, 11) when s_xs = 4 else to_unsigned(1923, 11) when s_xs = 3 else to_unsigned(1922, 11);
    w_xp  <= to_signed(7616, 18) when s_xs = 4 else to_signed(-48, 18) when s_xs = 3 else to_signed(-32, 18);
    w_xq  <= to_unsigned(480, 12) when s_xs = 4 else to_unsigned(0, 12);

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + [C] the per-field
    -- sequencer (one small operation per zone clock; the zone is quasi-static
    -- but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_17  : signed(16 downto 0);
        variable v_st  : signed(16 downto 0);
        variable v_v   : signed(10 downto 0);
        variable v_20  : signed(19 downto 0);
        variable v_s17 : unsigned(16 downto 0);
        variable v_22  : signed(21 downto 0);
    begin
        if rising_edge(clk) then
            -- [A] timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_rvr <= f_vrecip(s_tid);
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
            elsif s_zc /= 2047 then
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

            -- [A][B] per-field latches (the zone start is the once-per-field event: the grid bank toggles here)
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide  <= registers_in(6)(0);
                s_carpet <= registers_in(6)(1);
                s_ink    <= registers_in(6)(2);
                s_fine   <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_k5     <= unsigned(registers_in(4));
                s_bank   <= not s_bank;
                s_xs     <= s_xsr;
                case to_integer(s_xsr) is                  -- Osc step, box length (32 virtual px at 1080 and SD), its read offset
                    when 4      => s_oxs <= to_unsigned(204, 9); c_b32 <= '0'; c_brp <= to_unsigned(241, 8);
                    when 3      => s_oxs <= to_unsigned(153, 9); c_b32 <= '1'; c_brp <= to_unsigned(225, 8);
                    when others => s_oxs <= to_unsigned(102, 9); c_b32 <= '1'; c_brp <= to_unsigned(225, 8);
                end case;
                -- the vertical IIR's shift: about 32 .. 48 virtual lines in every mode
                if s_ys >= 960 then c_iir <= to_unsigned(3, 3);
                elsif s_ys >= 480 then c_iir <= to_unsigned(4, 3);
                else c_iir <= to_unsigned(5, 3); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- [B] shared hold + glide, six clocks per channel (X, Y, Scale, Slant, Camera, Wind):
            --   hold: the knob is re-read only when it has moved more than one count (pot jitter)
            --   glide On: 1/16 of the gap per field, never slower than one count, landing exactly
            --   glide Off: the accumulator takes the held value of THIS field start (no easing, no lag)
            if s_zgo = '1' then
                z_gp <= "000"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                z_gp <= z_gp + 1;
                case to_integer(z_gp) is
                    when 0 =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_a1; s_gh <= s_h1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_a2; s_gh <= s_h2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_a3; s_gh <= s_h3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_a4; s_gh <= s_h4; s_gr <= unsigned(registers_in(3));
                            when 4      => s_gm <= s_a6; s_gh <= s_h6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_ap; s_gh <= s_hp; s_gr <= unsigned(registers_in(7));
                        end case;
                    when 1 =>
                        s_hd <= signed(resize(s_gr, 11)) - signed(resize(s_gh, 11));
                    when 2 =>
                        if s_hd > 1 or s_hd < -1 then s_gt <= s_gr; else s_gt <= s_gh; end if;
                    when 3 =>
                        s_gd <= signed(resize(s_gt & "000000", 17)) - signed(resize(s_gm, 17));
                        case to_integer(z_gc) is
                            when 0      => s_h1 <= s_gt;
                            when 1      => s_h2 <= s_gt;
                            when 2      => s_h3 <= s_gt;
                            when 3      => s_h4 <= s_gt;
                            when 4      => s_h6 <= s_gt;
                            when others => s_hp <= s_gt;
                        end case;
                    when 4 =>
                        if s_glide = '0' or (s_gd <= 64 and s_gd >= -64) then
                            null;                                          -- the whole gap
                        elsif s_gd < 1024 and s_gd > -1024 then
                            if s_gd(16) = '1' then s_gd <= to_signed(-64, 17); else s_gd <= to_signed(64, 17); end if;
                        else
                            s_gd <= shift_right(s_gd, 4);
                        end if;
                    when others =>
                        v_st := signed(resize(s_gm, 17)) + s_gd;
                        case to_integer(z_gc) is
                            when 0      => s_a1 <= unsigned(v_st(15 downto 0));
                            when 1      => s_a2 <= unsigned(v_st(15 downto 0));
                            when 2      => s_a3 <= unsigned(v_st(15 downto 0));
                            when 3      => s_a4 <= unsigned(v_st(15 downto 0));
                            when 4      => s_a6 <= unsigned(v_st(15 downto 0));
                            when others => s_ap <= unsigned(v_st(15 downto 0));
                        end case;
                        z_gp <= "000";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [B] Drift (Hypnos v1.1.0 SPEED): glide 1/8 (Off: this field's knob), |v| < 24 still, else
            -- 2^-11 .. 2^-5 unit (16 nodes) per field (Q20), exponential (mantissa from the 2^(i/256) table, then
            -- a serial shift by the octave); the four row phases advance once per field at 1, 3/4, 85/128, 1/2
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 8  => s_vtg <= shift_left(resize(signed(resize(s_k5, 11)) - 512, 17), 6);
                    when 9  =>
                        if s_glide = '1' then s_vgs <= s_vgs + shift_right(s_vtg - s_vgs, 3);
                        else s_vgs <= s_vtg; end if;
                    when 10 =>
                        v_v := resize(shift_right(s_vgs, 6), 11);
                        s_vneg <= v_v(10);
                        if v_v(10) = '1' then s_av <= unsigned(resize(-v_v, 10)); else s_av <= unsigned(v_v(9 downto 0)); end if;
                    when 11 =>
                        if s_av < 24 then s_vz <= '1'; s_u <= (others => '0');
                        else s_vz <= '0'; s_u <= s_av - 24; end if;
                    when 12 =>
                        s_ls <= shift_right(s_u, 1) + shift_right(s_u, 2) + shift_right(s_u, 5) + shift_right(s_u, 8);
                    when 13 =>
                        r_ex_a <= s_ls(5 downto 0) & "00";
                    when 29 => c_s23 <= shift_right(c_spd, 1) + shift_right(c_spd, 3);
                    when 30 =>
                        c_ph(0) <= f_addsub(c_ph(0), resize(c_spd, 20), c_sneg);
                        v_s17 := c_spd - shift_right(c_spd, 2);
                        c_ph(1) <= f_addsub(c_ph(1), resize(v_s17, 20), c_sneg);
                        v_s17 := c_s23 + shift_right(c_s23, 4);
                        c_ph(2) <= f_addsub(c_ph(2), resize(v_s17, 20), c_sneg);
                        c_ph(3) <= f_addsub(c_ph(3), resize(shift_right(c_spd, 1), 20), c_sneg);
                    when others => null;
                end case;
            end if;
            if s_zone = '1' and s_zc = 16 then
                if s_vz = '1' then c_spd <= (others => '0');
                else c_spd <= resize(r_exf(15 downto 6), 17); end if;
                c_sneg <= s_vneg;
                s_scnt <= s_ls(8 downto 6);
            elsif s_zone = '1' and s_zc > 16 and s_zc < 26 and s_scnt /= 0 then
                c_spd <= shift_left(c_spd, 1);
                s_scnt <= s_scnt - 1;
            end if;

            -- [C] before the slots (zc 40 .. 43): the slant's sign and magnitude past a +-16-count dead centre,
            -- the X slide (+-3.5 nodes), the Y pan (+-1.5 row periods)
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 40 =>
                        v_17 := signed(resize(s_a4, 17)) - 32768;
                        c_sng <= v_17(16);
                        v_st := -v_17;
                        if v_17(16) = '1' then z_mag <= unsigned(v_st(15 downto 0)); else z_mag <= unsigned(v_17(15 downto 0)); end if;
                        v_20 := resize(signed(resize(s_a1, 17)) - 32768, 20);
                        z_xs <= unsigned(shift_left(v_20, 3) - v_20);
                    when 41 =>
                        if z_mag > 1024 then z_m <= resize(z_mag - 1024, 15); else z_m <= (others => '0'); end if;
                        v_22 := resize(signed(resize(s_a2, 17)) - 32768, 22);
                        if s_carpet = '1' then z_yo <= shift_left(v_22, 5) - shift_left(v_22, 3);
                        else z_yo <= shift_left(v_22, 5) - shift_left(v_22, 1); end if;
                    when 42 =>
                        z_t <= resize(z_m, 16) + resize(shift_right(z_m, 5), 16);
                    when 43 =>
                        c_Sq <= resize(z_t + resize(shift_right(z_m, 10), 16), 15);
                    when others => null;
                end case;
            end if;

            -- [C] serial products, one slot every 24 zone clocks from zc 46: the operands are muxed at phase
            -- 1, loaded at phase 2, 16 iterations, the product is stable from phase 19 and stored at phase 20.
            if s_zone = '0' then
                z_run <= '0';
            elsif s_zc = 46 then
                z_run <= '1'; z_ph <= (others => '0'); z_slot <= (others => '0');
            elsif z_run = '1' then
                if z_ph = 23 then
                    z_ph <= (others => '0');
                    if z_slot = C_NSLOT then z_run <= '0'; else z_slot <= z_slot + 1; end if;
                else
                    z_ph <= z_ph + 1;
                end if;
            end if;
            if z_run = '1' and z_ph = 0 then z_pre <= '1'; else z_pre <= '0'; end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_slot) is
                    when 0  => z_fa <= resize(s_a3, 24);  z_fb <= to_unsigned(852, 16);              -- octaves above 10.8 v-px, Q8
                    when 1  => z_fa <= resize(r_exf, 24); z_fb <= to_unsigned(48546, 16);            -- nodes per h
                               r_ex_a <= z_oct(7 downto 0);
                    when 2  => z_fa <= resize(r_exf, 24); z_fb <= to_unsigned(44237, 16);            -- pitch in h
                    when 3  => z_fa <= resize(c_Sq, 24);  z_fb <= resize(s_ys, 16);                  -- the lean per line
                    when 4  => z_fa <= resize(c_P, 24);   z_fb <= s_rvr;                             -- AA units per node, v
                    when 5  => z_fa <= resize(c_inv, 24); z_fb <= resize(s_ys, 16);                  -- nodes per line
                    when 6  => z_fa <= resize(c_inv, 24); z_fb <= resize(w_dxm, 16);                 -- u at the pre-pixel
                    when 7  => z_fa <= resize(c_inv, 24); z_fb <= to_unsigned(1080, 16);             -- v at line 0
                    when 8  => z_fa <= resize(c_P, 24);   z_fb <= c_R;                               -- AA units per node, u
                    when 9  => z_fa <= resize(s_a6, 24);  z_fb <= to_unsigned(23967, 16);            -- camera gain
                    when 10 => z_fa <= resize(s_ap, 24);  z_fb <= to_unsigned(50, 16);               -- the wind
                    when 11 => z_fa <= c_Ku;              z_fb <= resize(z_Lf, 16);                  -- the tip's reach
                    when 12 => z_fa <= resize(c_Kv, 24);  z_fb <= resize(z_Lf, 16);
                    when 13 => z_fa <= resize(z_spu, 24); z_fb <= resize(z_g, 16);                   -- weight per camera level
                    when others => z_fa <= resize(z_spv, 24); z_fb <= resize(z_g, 16);
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 34); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(32 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if z_run = '1' then
                -- slots 11 (u) and 12 (v), phases 3 .. 7: the base half width, its cap and the camera's span
                -- (AA units Q8).  Bold 1/4 node; Fine 5/64 node, at least one virtual px; cap 7/16 node.
                if z_slot = 11 or z_slot = 12 then
                    case to_integer(z_ph) is
                        when 3 =>
                            if z_slot = 11 then z_t <= c_Ku(23 downto 8); z_fl <= c_R(15 downto 7);
                            else z_t <= c_Kv(19 downto 4); z_fl <= s_rvr(15 downto 7); end if;
                        when 4 => z_h1 <= shift_right(z_t, 4) + shift_right(z_t, 6);
                        when 5 => z_hm <= shift_right(z_t, 1) - shift_right(z_t, 4);
                        when 6 =>
                            if s_fine = '0' then z_hb <= shift_right(z_t, 2);
                            elsif z_h1 < resize(z_fl, 16) then z_hb <= resize(z_fl, 16);
                            else z_hb <= z_h1; end if;
                        when 7 =>
                            if z_slot = 11 then
                                c_hbu <= z_hb; c_hmu <= z_hm(15 downto 4); z_spu <= resize(z_hm - z_hb, 14);
                            else
                                c_hbv <= z_hb; c_hmv <= z_hm(15 downto 4); z_spv <= resize(z_hm - z_hb, 14);
                            end if;
                        when others => null;
                    end case;
                end if;
                if z_ph = 20 then
                    case to_integer(z_slot) is
                        when 0  => z_oct <= f_acc(26 downto 16);
                        when 1  =>
                            case to_integer(z_shi) is
                                when 0      => c_inv <= f_acc(32 downto 11);
                                when 1      => c_inv <= f_acc(33 downto 12);
                                when 2      => c_inv <= resize(f_acc(33 downto 13), 22);
                                when 3      => c_inv <= resize(f_acc(33 downto 14), 22);
                                when others => c_inv <= resize(f_acc(33 downto 15), 22);
                            end case;
                        when 2  =>
                            case to_integer(z_oct(9 downto 8)) is
                                when 0      => c_P <= f_acc(33 downto 18);
                                when 1      => c_P <= f_acc(32 downto 17);
                                when 2      => c_P <= f_acc(31 downto 16);
                                when others => c_P <= f_acc(30 downto 15);
                            end case;
                        when 3  => c_F <= resize(s_xs & x"00", 13) + resize(f_acc(25 downto 14), 13);
                        when 4  => c_Kv <= f_acc(31 downto 12);
                        when 5  => c_dv <= f_acc(30 downto 11);
                        when 6  => c_Ab <= to_unsigned(0, 20) - f_acc(27 downto 8);
                        when 7  =>                         -- + 8 row periods (the row class is unchanged), so it is never negative
                            if s_carpet = '1' then z_vt <= to_unsigned(64 * 1048576, 27) - resize(f_acc(29 downto 4), 27);
                            else z_vt <= to_unsigned(89 * 1048576, 27) - resize(f_acc(29 downto 4), 27); end if;
                        when 8  => c_Ku <= f_acc(31 downto 8);
                        when 9  => z_g <= f_acc(30 downto 16);
                        when 10 => z_L <= f_acc(21 downto 6);
                        when 11 => c_LfKu <= f_acc(33 downto 22);
                        when 12 => c_LfKv <= f_acc(29 downto 18);
                        when 13 =>
                            if s_src = '1' then c_slu <= f_acc(29 downto 14) & '0'; else c_slu <= '0' & f_acc(29 downto 14); end if;
                        when others =>
                            if s_src = '1' then c_slv <= f_acc(29 downto 14) & '0'; else c_slv <= '0' & f_acc(29 downto 14); end if;
                    end case;
                end if;
                if z_ph = 21 then
                    case to_integer(z_slot) is
                        when 0  =>                         -- 2^-oct = table(-oct mod 256) * 2^-shi
                            r_ex_a <= not z_oct(7 downto 0) + 1;
                            if z_oct(7 downto 0) = 0 then z_shi <= resize(z_oct(9 downto 8), 3);
                            else z_shi <= resize(z_oct(9 downto 8), 3) + 1; end if;
                        when 7  => z_vt <= unsigned(signed(z_vt) + shift_left(resize(z_yo, 27), 4));
                        when 8  =>                         -- the pixel step in AA units
                            case to_integer(s_xs) is
                                when 4      => c_dT <= c_R & "00";
                                when 3      => c_dT <= resize(c_R & '0', 18) + resize(c_R, 18);
                                when others => c_dT <= resize(c_R & '0', 18);
                            end case;
                        when 10 =>                         -- the wind ends at the eye (node 49)
                            if z_L >= 50176 then c_Li <= to_unsigned(49, 6); z_Lf <= (others => '0');
                            else c_Li <= z_L(15 downto 10); z_Lf <= z_L(9 downto 0); end if;
                        when others => null;
                    end case;
                end if;
                if z_ph = 22 then
                    case to_integer(z_slot) is
                        when 8  => c_dTmK <= signed(resize(c_dT, 25)) - signed(resize(c_Ku, 25));
                        when 10 => c_Li1 <= c_Li + 1;
                        when 12 =>
                            c_tu1 <= signed(resize(c_LfKu, 14)) - signed(resize(c_Ku(23 downto 12), 14));
                            c_tv1 <= signed(resize(c_LfKv, 14)) - signed(resize(c_Kv(19 downto 8), 14));
                        when others => null;
                    end case;
                end if;
            end if;

            -- [C] serial divider (slot 3, phase 21): R = 2^24 / F, bit-serial long division: the remainder
            -- starts at the numerator's leading one, 24 zero bits follow; the first 8 quotient bits are zero
            if z_run = '1' and z_slot = 3 and z_ph = 21 then
                dv_R <= to_unsigned(1, 13);
                dv_Q <= (others => '0'); dv_n <= to_unsigned(24, 5); dv_st <= '0';
            elsif dv_n /= 0 then
                if (dv_R(11 downto 0) & '0') >= c_F then
                    dv_R <= (dv_R(11 downto 0) & '0') - c_F; dv_Q <= dv_Q(14 downto 0) & '1';
                else
                    dv_R <= dv_R(11 downto 0) & '0'; dv_Q <= dv_Q(14 downto 0) & '0';
                end if;
                dv_n <= dv_n - 1;
                if dv_n = 1 then dv_st <= '1'; end if;
            elsif dv_st = '1' then
                dv_st <= '0';
                c_R <= dv_Q;
            end if;

            -- [A] sync delay and generated-avid shift registers (field_n is tied high by the core on every
            -- input, so it only gets a register)
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [C] WEIGHT TABLE FILLS (vblank zone, after the slots): the node's half
    -- line width by camera level, in 1/16 AA unit, across u then down v:
    --   T(L) = min(hb + L * slope, cap)
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_t : unsigned(11 downto 0);
        variable v_c : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            hk_we <= '0';
            if s_zone = '1' and s_zc = C_TF_ZC then
                tf_run <= '1'; tf_pass <= '0'; tf_i <= (others => '0');
                tf_acc <= c_hbu & x"00";
            elsif tf_run = '1' then
                v_t := tf_acc(23 downto 12);
                if tf_pass = '0' then v_c := c_hmu; else v_c := c_hmv; end if;
                hk_we <= '1'; hk_wa <= tf_i; tf_pd <= tf_pass;
                if v_t > v_c then hk_wd <= v_c(9 downto 0); else hk_wd <= v_t(9 downto 0); end if;
                tf_i <= tf_i + 1;
                if tf_i = 255 then
                    if tf_pass = '0' then tf_pass <= '1'; tf_acc <= c_hbv & x"00";
                    else tf_run <= '0'; end if;
                elsif tf_pass = '0' then
                    tf_acc <= tf_acc + resize(c_slu, 24);
                else
                    tf_acc <= tf_acc + resize(c_slv, 24);
                end if;
            end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- [A] Generated avid (parity-locked start 3/4 clocks after the source's
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
    -- [C] LINE ENGINE: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line's values on one serial multiplier (16
    -- iterations, the product is read 17 clocks after the load):
    --    1  v wraps at the row period (the row class steps)
    --    2  the node row j, the fraction's distance from the node's centre line, v - 4, the row's slide
    --    3  A = the row's u at the pre-pixel, unsheared              load  |fv - 1/2| * Kv
    --   20  Tv (1/16 AA unit, signed)                                load  |v - 4| * slant
    --   37  u0 = A +- the lean;  the lean of the node row's centre line (slant * |2j - 7| / 2, shift-adds)
    --   38  the node index i0; uc = the centre line's u               load  frac(u0) * Ku
    --   55  U0 = the DDA's start;  the first node's centre: 1/2 - frac(uc)      56  load  * pitch
    --   73  xc = the pre-pixel's x +- that                           load  |fv - 1/2| * pitch
    --   74  the first node ON THIS LINE is that one or its neighbour (+- one pitch)
    --   90  (a new node row only) the node row's centre line yc
    --   91, 92  the grid rows: the node row's (read) and this line's (write), * 60
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_per : unsigned(26 downto 0);
        variable v_d   : signed(16 downto 0);
        variable v_dn  : signed(16 downto 0);
        variable v_m   : unsigned(10 downto 0);
        variable v_sg  : std_logic;
        variable v_x   : signed(16 downto 0);
        variable v_xn  : signed(16 downto 0);
        variable v_yb  : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = C_LK_ZC then e_zk <= '1'; end if;

            if s_carpet = '1' then v_per := to_unsigned(8 * 1048576, 27); else v_per := to_unsigned(10 * 1048576, 27); end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(38 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            if e_zk = '1' then                      -- line 0: v + 8 periods, brought into its row in at most 15 steps
                l_v <= z_vt; l_row <= (others => '0');
                l_y <= (others => '0'); l_pjv <= '0';
                l_rc <= to_unsigned(15, 4);
            elsif l_rc /= 0 then
                if l_v >= v_per then l_v <= l_v - v_per; l_row <= l_row + 1; end if;
                l_rc <= l_rc - 1;
                if l_rc = 1 then e_lc <= (others => '0'); end if;
            elsif g_start = '1' then
                l_v <= l_v + resize(c_dv, 27);
                l_y <= l_y + resize(s_ys, 19);
                e_lc <= (others => '0');
            else
                if e_lc /= 255 then e_lc <= e_lc + 1; end if;
                case to_integer(e_lc) is
                    when 1 =>
                        if l_v >= v_per then l_v <= l_v - v_per; l_row <= l_row + 1; end if;
                    when 2 =>
                        e_mneg <= not l_v(19);
                        if l_v(19) = '1' then e_mag <= '0' & l_v(18 downto 8);
                        else e_mag <= to_unsigned(2048, 12) - resize(l_v(18 downto 8), 12); end if;
                        v_d := signed(resize(l_v(23 downto 8), 17)) - 16384;
                        v_dn := -v_d;
                        e_dneg <= v_d(16);
                        if v_d(16) = '1' then e_dmag <= unsigned(v_dn(15 downto 0)); else e_dmag <= unsigned(v_d(15 downto 0)); end if;
                        e_A <= c_ph(to_integer(l_row)) + z_xs;
                    when 3 =>
                        e_A <= f_addsub(c_Ab, e_A, not l_row(0));
                        m_a <= resize(c_Kv, 40); m_b <= resize(e_mag, 16); m_acc <= (others => '0');
                    when 20 =>
                        v_m := '0' & m_acc(29 downto 20);
                        if e_mneg = '1' then n_Tv <= -signed(v_m); else n_Tv <= signed(v_m); end if;
                        m_a <= resize(c_Sq, 40); m_b <= e_dmag; m_acc <= (others => '0');
                    when 37 =>
                        v_sg := e_dneg xor c_sng xor l_row(0);
                        e_u0 <= f_addsub(e_A, resize(m_acc(29 downto 11), 20), v_sg);
                        case to_integer(l_v(22 downto 20)) is
                            when 0 | 7  => e_shc <= resize(c_Sq & "000", 18) - resize(c_Sq, 18);
                            when 1 | 6  => e_shc <= resize(c_Sq & "00", 18) + resize(c_Sq, 18);
                            when 2 | 5  => e_shc <= resize(c_Sq & '0', 18) + resize(c_Sq, 18);
                            when others => e_shc <= resize(c_Sq, 18);
                        end case;
                    when 38 =>
                        n_i0 <= e_u0(19 downto 16);
                        v_sg := (not l_v(22)) xor c_sng xor l_row(0);
                        e_u0 <= f_addsub(e_A, resize(e_shc, 20), v_sg);          -- (from here e_u0 holds uc)
                        m_a <= resize(c_Ku, 40); m_b <= e_u0(15 downto 0); m_acc <= (others => '0');
                    when 55 =>
                        n_U0 <= m_acc(39 downto 16);
                        v_x := to_signed(32768, 17) - signed(resize(e_u0(15 downto 0), 17));
                        v_xn := -v_x;
                        e_xneg <= v_x(16);
                        if v_x(16) = '1' then e_xmag <= unsigned(v_xn(15 downto 0)); else e_xmag <= unsigned(v_x(15 downto 0)); end if;
                        e_dl <= n_i0 - e_u0(19 downto 16);
                    when 56 =>
                        m_a <= resize(c_P, 40); m_b <= e_xmag; m_acc <= (others => '0');
                    when 73 =>
                        if e_xneg = '1' then e_xc <= w_xp - signed(resize(m_acc(31 downto 20), 18));
                        else e_xc <= w_xp + signed(resize(m_acc(31 downto 20), 18)); end if;
                        m_a <= resize(c_P, 40); m_b <= resize(e_mag, 16); m_acc <= (others => '0');
                    when 74 =>
                        if e_dl = 1 then n_xc0 <= e_xc + signed(resize(c_P(15 downto 4), 18));
                        elsif e_dl = 15 then n_xc0 <= e_xc - signed(resize(c_P(15 downto 4), 18));
                        else n_xc0 <= e_xc; end if;
                    when 90 =>
                        if l_v(23 downto 20) /= l_pj or l_pjv = '0' then
                            if e_mneg = '1' then e_yc <= signed(resize(l_y, 21)) + signed(resize(m_acc(26 downto 13), 21));
                            else e_yc <= signed(resize(l_y, 21)) - signed(resize(m_acc(26 downto 13), 21)); end if;
                        end if;
                    when 91 =>
                        if e_yc(20) = '1' then e_yb <= (others => '0');
                        elsif e_yc(19 downto 13) > 33 then e_yb <= to_unsigned(33, 6);
                        else e_yb <= unsigned(e_yc(18 downto 13)); end if;
                        if l_y(18 downto 13) > 33 then e_ybw <= to_unsigned(33, 6); else e_ybw <= l_y(18 downto 13); end if;
                        l_pj <= l_v(23 downto 20); l_pjv <= '1';
                        n_j <= l_v(22 downto 20);
                        if l_v(23 downto 20) >= 8 then n_band <= '1'; else n_band <= '0'; end if;
                        n_par <= l_row(0);
                    when 92 =>
                        n_b60 <= resize(e_yb & "000000", 11) - resize(e_yb & "00", 11);
                        n_w60 <= resize(e_ybw & "000000", 11) - resize(e_ybw & "00", 11);
                    when others => null;
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [E] KEY, pixel path (stages 2 .. 15); coverages in 1/16 (0 .. 16)
    --    2  the node DDA: U = the distance into the node in AA units (Q16), wrapping at the node width K;
    --       the node index i and the node centre's x step with it
    --    3  T = U - K/2 (1/16 AA, signed); the node's grid column          4  the grid address
    --    5  grid RAM         6  fabric; the node ROM's address             7  weight tables, node ROM
    --    8  fabric
    --    9  the four edge coverages of the node's centre square; the node's state (full / tip leaving /
    --       tip entering / not reached); the tip's reach along its heading
    --   10  strips, bar extents, the tip's coverage           11  bars          12, 13  union -> tone address
    --   14  tone ROM         15  fabric
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_b   : signed(24 downto 0);
        variable v_t   : signed(24 downto 0);
        variable v_tu  : signed(14 downto 0);
        variable v_hu, v_hv, v_tv : signed(14 downto 0);
        variable v_full, v_tout, v_tin, v_hz : std_logic;
        variable v_m   : std_logic_vector(3 downto 0);
        variable v_hd  : unsigned(1 downto 0);
        variable v_ord : unsigned(5 downto 0);
        variable v_tq  : signed(14 downto 0);
        variable v_ink : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            -- 2
            if g_start = '1' then
                b_U <= n_U0; b_i <= n_i0; b_xc <= n_xc0;
                p_Tv <= n_Tv; p_j <= n_j; p_band <= n_band; p_par <= n_par; p_b60 <= n_b60; p_w60 <= n_w60;
            else
                v_b := signed(resize(b_U, 25)) + c_dTmK;
                if v_b(24) = '0' then
                    b_U <= unsigned(v_b(23 downto 0)); b_i <= b_i + 1;
                    b_xc <= b_xc + signed(resize(c_P(15 downto 4), 18));
                else
                    b_U <= b_U + resize(c_dT, 24);
                end if;
            end if;
            -- 3
            v_t := signed(resize(b_U, 25)) - signed(resize(c_Ku(23 downto 1), 25));
            s_T(3) <= v_t(22 downto 12);
            if b_xc(17) = '1' then s3_xb <= (others => '0');
            elsif b_xc(16 downto 10) > 59 then s3_xb <= to_unsigned(59, 6);
            else s3_xb <= unsigned(b_xc(15 downto 10)); end if;
            s3_i <= b_i;
            -- 4
            s4_ga <= (not s_bank) & (p_b60 + resize(s3_xb, 11));
            s4_i <= s3_i;
            -- 5 (grid RAM), 6
            gr_f <= gr_q;
            s5_i <= s4_i;
            kr_a <= p_par & p_j & s5_i;
            -- 7 (tables, node ROM), 8
            hu_f <= hu_q; hv_f <= hv_q; kr_f <= kr_q;
            for k in 4 to 8 loop s_T(k) <= s_T(k - 1); end loop;
            -- 9
            v_tu := resize(s_T(8), 15);
            v_tv := resize(p_Tv, 15);
            v_hu := signed(resize(hu_f, 15)); v_hv := signed(resize(hv_f, 15));
            s9_culo <= f_sat(v_tu + v_hu + 8);  s9_cuhi <= f_sat(v_hu - v_tu + 8);
            s9_cvlo <= f_sat(v_tv + v_hv + 8); s9_cvhi <= f_sat(v_hv - v_tv + 8);
            v_ord := kr_f(13 downto 8);
            v_full := '0'; v_tout := '0'; v_tin := '0';
            if kr_f(14) = '1' or v_ord < c_Li then v_full := '1';
            elsif v_ord = c_Li then v_tout := '1';
            elsif v_ord = c_Li1 then v_tin := '1'; end if;
            v_m := std_logic_vector(kr_f(3 downto 0));
            if v_tout = '1' then
                v_hd := kr_f(7 downto 6);
                v_m(to_integer(kr_f(7 downto 6))) := '0';
            else
                v_hd := kr_f(5 downto 4);
                if v_full = '0' then v_m := "0000"; end if;
            end if;
            s9_mm <= v_m; s9_tip <= v_tout or v_tin; s9_tout <= v_tout; s9_hd <= v_hd;
            if v_hd(0) = '1' then            -- E, W: along u
                if v_tin = '1' then v_tq := resize(c_tu1, 15) + v_hu; else v_tq := signed(resize(c_LfKu, 15)) + v_hu; end if;
                if v_hd(1) = '0' then s9_s <= s_T(8); else s9_s <= -s_T(8); end if;
            else                             -- N, S: along v
                if v_tin = '1' then v_tq := resize(c_tv1, 15) + v_hv; else v_tq := signed(resize(c_LfKv, 15)) + v_hv; end if;
                if v_hd(1) = '1' then s9_s <= p_Tv; else s9_s <= -p_Tv; end if;
            end if;
            s9_tq <= v_tq;
            -- 10
            s10_su <= f_strip(s9_culo, s9_cuhi); s10_sv <= f_strip(s9_cvlo, s9_cvhi);
            if s9_mm(3) = '1' and s9_mm(1) = '1' then s10_eu <= to_unsigned(16, 5);
            elsif s9_mm(3) = '1' then s10_eu <= s9_cuhi;
            elsif s9_mm(1) = '1' then s10_eu <= s9_culo;
            else s10_eu <= f_strip(s9_culo, s9_cuhi); end if;
            if s9_mm(0) = '1' and s9_mm(2) = '1' then s10_ev <= to_unsigned(16, 5);
            elsif s9_mm(0) = '1' then s10_ev <= s9_cvhi;
            elsif s9_mm(2) = '1' then s10_ev <= s9_cvlo;
            else s10_ev <= f_strip(s9_cvlo, s9_cvhi); end if;
            s10_he <= s9_mm(3) or s9_mm(1); s10_ve <= s9_mm(0) or s9_mm(2);
            s10_ct <= f_sat(s9_tq - resize(s9_s, 15) + 8);
            if s9_tout = '0' then s10_bk <= to_unsigned(16, 5);
            else
                case to_integer(s9_hd) is
                    when 1      => s10_bk <= s9_culo;
                    when 3      => s10_bk <= s9_cuhi;
                    when 2      => s10_bk <= s9_cvlo;
                    when others => s10_bk <= s9_cvhi;
                end case;
            end if;
            s10_hz <= s9_hd(0); s10_tip <= s9_tip;
            -- 11
            if s10_he = '1' then s11_hb <= f_min(s10_sv, s10_eu); else s11_hb <= (others => '0'); end if;
            if s10_ve = '1' then s11_vb <= f_min(s10_su, s10_ev); else s11_vb <= (others => '0'); end if;
            s11_tb <= f_min(s10_bk, s10_ct);
            if s10_hz = '1' then s11_st <= s10_sv; else s11_st <= s10_su; end if;
            s11_tip <= s10_tip;
            -- 12
            s12_m <= f_max(s11_hb, s11_vb);
            if s11_tip = '1' then s12_t <= f_min(s11_st, s11_tb); else s12_t <= (others => '0'); end if;
            -- 13: the tone ROM's address (the plain band between friezes is black)
            v_ink := f_max(s12_m, s12_t);
            if p_band = '1' then tn_a <= (others => '0');
            else tn_a <= (p_par and not s_ink) & v_ink; end if;
            -- 14 (the shared ROM), 15 (its fabric register r_exf)
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> box over 32 virtual px
    -- (48 at 720p), edge-replicated and reloaded at every line start (the box's
    -- resync) -> vertical one-pole IIR in the line store (half horizontal
    -- resolution; the first line of a field loads it) -> the block grid: every
    -- write of the store also goes to the 32 x 32 block it lies in, so each
    -- block ends the field holding its last line's box mean at its right end.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l   : unsigned(7 downto 0);
        variable v_s   : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_d   : signed(12 downto 0);
        variable v_dd  : signed(12 downto 0);
        variable v_n   : signed(12 downto 0);
        variable v_full : std_logic;
        variable v_xh  : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting; both phases restart in the
            -- zone (Hardline dazzle's fix of riley's line-0 phase)
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);   -- ot * 96
            end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- 1: the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_avd <= data_in.avid & s_avd(1 to 4);
            if s_avd(1) = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- 2: a plain register (keeps the shell's timing)
            s_p <= s_sl;
            s_pcol <= s_col;

            -- 3: box: sum of the last N samples; the samples before the line start count as the first one
            if s_avd(2) = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= c_brp; end if;
            if (c_b32 = '1' and s_bcnt = 32) or (c_b32 = '0' and s_bcnt = 16) then v_full := '1'; else v_full := '0'; end if;
            if v_full = '0' then v_tap := s_pfirst; else v_tap := bx_q; end if;
            if s_avd(2) = '1' and s_avd(3) = '0' then
                s_pfirst <= s_p;
                if c_b32 = '1' then s_bsum <= shift_left(resize(s_p, 13), 5);
                else s_bsum <= shift_left(resize(s_p, 13), 4); end if;
                s_bcnt <= to_unsigned(1, 6);
            elsif s_avd(2) = '1' then
                s_bsum <= s_bsum + s_p - v_tap;
                if v_full = '0' then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bcol <= s_pcol;

            -- 4: fabric copies (the box mean x 16); 5: vertical IIR on the odd columns, q += (box - q) >> k
            -- (the first line: q = box)
            if c_b32 = '1' then s_eb <= s_bsum(12 downto 1); else s_eb <= s_bsum(11 downto 0); end if;
            s_eq <= lb_q1;
            s_ev <= s_avd(3) and s_bcol(0); s_ecol <= s_bcol;
            if s_zone = '1' then s_fl <= '1';
            elsif s_avd(5) = '1' and s_avd(4) = '0' then s_fl <= '0'; end if;
            v_d := signed(resize(s_eb, 13)) - signed(resize(s_eq, 13));
            case to_integer(c_iir) is
                when 3      => v_dd := shift_right(v_d, 3);
                when 4      => v_dd := shift_right(v_d, 4);
                when others => v_dd := shift_right(v_d, 5);
            end case;
            v_n := signed(resize(s_eq, 13)) + v_dd;
            if s_fl = '1' then lb_wd <= s_eb; else lb_wd <= unsigned(v_n(11 downto 0)); end if;
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            -- the block grid: the column's block (x = pixel 0's x + column * xs, in h; 64 h per block), then
            -- the address in this field's bank
            case to_integer(s_xs) is
                when 4      => v_xh := resize(s_ecol & "00", 13);
                when 3      => v_xh := resize(s_ecol & '0', 13) + resize(s_ecol, 13);
                when others => v_xh := resize(s_ecol & '0', 13);
            end case;
            v_xh := v_xh + resize(w_xq, 13);
            gw_xb <= v_xh(11 downto 6);
            gw_a  <= s_bank & (p_w60 + resize(gw_xb, 11));
            gw_d  <= lb_wd(11 downto 4);
            gw_we <= lb_we;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING (stages 16 .. 21): luma [1 2 1]/4, luma 13/16
    -- toward black (white 776), blanking gate.  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 16 .. 18: taps y(n + 1), y(n), y(n - 1)
            s_yd(1) <= r_exf(9 downto 0);
            for i in 2 to 3 loop s_yd(i) <= s_yd(i - 1); end loop;
            s18_ya <= resize(s_yd(1), 12) + resize(s_yd(3), 12) + 2;
            s18_yb <= s_yd(2);
            -- 19
            v_y := s18_ya + shift_left(resize(s18_yb, 12), 1);
            s19_y <= v_y(11 downto 2);
            -- 20: luma 13/16 toward black
            if s19_y > 64 then v_dl := s19_y - 64; else v_dl := (others => '0'); end if;
            s20_y <= s19_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            -- 21: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s21_y <= s20_y;
            else
                s21_y <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s21_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture meander;
