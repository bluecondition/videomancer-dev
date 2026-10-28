-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Abacus v0.5.0 -- Heirloom #42, rebuilt from the photo WIN_20230605_01_03_50.
--
-- The photo: rows of glossy red beads between wavy blue/green bands, one bead
-- every 188 px (10.2 across), the rows mirrored about the bead-row centres
-- (period 495 lines, bead rows on lines 59 / 554 / 1049).  The beads change
-- across the frame: dark maroon with green lenses and dark 'eyes' on the left,
-- bright red ovals in the middle, pink-cored hourglass beads on the right; the
-- band carries green diamonds (left) / blue ellipses (right) on the bead
-- columns, a red dot at x ~ 1000 and a red band-edge line with a navy fringe.
--
-- The patch, as measured (abacus.py holds the tables):
--   ONE scalar field s(x, d) through ONE palette (a colourizer).  s is a
--   measured knot grid -- 32 rows (d = distance from the bead row, every 8
--   lines) x 128 columns (every 15 virtual px), bilinear -- and the palette is
--   the 512-entry colour sequence along s (bright red .. maroon .. pink/cream
--   glint .. olive .. navy .. red line .. violet .. green .. blue).
--   Vertical: d = |triangle| of a 495-line oscillator (Stack: a saw).
--   Horizontal: the knot column comes from a triangle of the horizontal
--   oscillator: the photo, then its mirror image, repeating.
--   Per pixel: s' = 512 + Rims (s - 512) + Swell + Etch (luma - 110); index s'/2.
--   Gloss: the photo's specular glints (cream / white arcs on the bead tops
--   and bottoms, orange and yellow sparks, grey band-edge glints) and the left
--   third's green lenses, navy eyes and amber eye arcs are thin runs measured
--   column by column (abacus.py C_AB_GL: 2 runs per 7.5 px column, centre /
--   half-width / amplitude / colour), drawn as soft-edged lines over the
--   field's colour.  They ride the same coordinates (K1/K2 zoom, Slide, Roll,
--   Mirror, Stack); glints are drawn only on beads and rims (s' < 768).
--
--   K1  Beads    beads across: 0.5 .. 2.5x (centre = the photo's 10.2)
--   K2  Rows     bead rows per frame height: 0.5 .. 2.5x (centre = the photo's 2.18)
--   K3  Rims     field gain 0.5 .. 2.5x: thick soft dark rims .. thin hard rims
--   K4  Slide    bipolar: the beads slide along the rows (centre +-8 still)
--   K5  Roll     bipolar: the rows roll up / down (centre +-8 still)
--   K6  Etch     the picture's luma pushed into the field
--   S7  Mirror   left/right mirror about the centre
--   S8  Palette  Photo / Swap (U and V exchanged: blue beads, red bands)
--   S9  Stack    Mirror (the photo) / Saw (rows repeat, cut flat)
--   S10 Flip     beads and bands exchange places (s -> 1 - s)
--   S11 Source   Video / Osc
--   P12 SWELL    bead size: 0 = small pips in a blue/green sea, 100 % = beads merge into red bars
--
-- Engine: a vblank-zone sequencer (one serial multiplier, fixed slots, a
-- shared glide) sets the per-field constants; a per-line engine (kicked at
-- each line start, computes the NEXT line into the other bank of a 2 x 128
-- row RAM) blends two knot rows by the line's fraction; the beam runs the
-- horizontal phase.  Generated avid (parity-locked); output conditioning
-- (chroma 9-tap, luma [1 2 1], luma 13/16) before the blanking gate.  Latency
-- 24 clocks in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.abacus_pkg.all;
use work.all;

architecture abacus of program_top is
    constant C_LAT  : integer := 24;          -- = processing_delay_clks
    constant C_VPER : integer := 126720;      -- vertical period, 495 lines x 256
    constant C_VMID : integer := 63360;       -- band centre (d = 247.5 lines)

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
    -- horizontal units per pixel x 256 at zoom 1 (2048 units = 1920 virtual px):
    -- 1920 px -> 273, 1280 px -> 410, 720 px (4:3, 1440 virtual px) -> 546
    function f_hstep(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(546, 10);
            when 5 | 6 | 14     => return to_unsigned(410, 10);
            when others         => return to_unsigned(273, 10);
        end case;
    end function;
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- video line buffer: (luma >> 2) - 16, clipped at 0, of the line being drawn
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- row RAM: 2 banks x 128 entries {R[j] (25..16), R[j+1] (9..0)}
    type t_row is array (0 to 255) of std_logic_vector(31 downto 0);
    signal r_ram : t_row := (others => (others => '0'));
    signal r_q   : std_logic_vector(31 downto 0) := (others => '0');
    attribute no_rw_check : boolean;
    attribute no_rw_check of r_ram : signal is true;

    -- oscillator source (Source = Osc)
    signal s_ox : unsigned(9 downto 0) := (others => '0');
    signal s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_hb   : unsigned(9 downto 0) := to_unsigned(273, 10);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(8 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_half : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_hm1  : unsigned(10 downto 0) := to_unsigned(959, 11);

    -- knobs: glided (Q3), raw speeds, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(160 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');                 -- shared glide: selected knob
    signal s_gd : signed(14 downto 0) := (others => '0');                   -- and its step
    signal s_k4, s_k5 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_m4, s_m5 : unsigned(9 downto 0) := (others => '0');            -- |speed| - 8
    signal s_n4, s_n5, s_st4, s_st5 : std_logic := '1';
    signal s_mirror, s_swap, s_saw, s_flip, s_src : std_logic := '0';

    -- per-field constants
    signal s_z1, s_z2 : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_st   : unsigned(10 downto 0) := to_unsigned(273, 11);           -- units x 256 per pixel
    signal s_xt   : unsigned(19 downto 0) := (others => '0');
    signal s_x0   : unsigned(19 downto 0) := to_unsigned(64, 20);            -- line-start phase
    signal s_pan  : unsigned(19 downto 0) := (others => '0');
    signal s_pst  : signed(12 downto 0) := (others => '0');
    signal s_vst  : unsigned(11 downto 0) := to_unsigned(256, 12);           -- vertical phase per line
    signal s_rst  : signed(14 downto 0) := (others => '0');
    signal s_rla  : signed(18 downto 0) := (others => '0');
    signal s_roll : unsigned(16 downto 0) := (others => '0');
    signal s_v0a  : unsigned(18 downto 0) := (others => '0');
    signal s_vp0  : unsigned(16 downto 0) := to_unsigned(48256, 17);
    signal s_g    : unsigned(7 downto 0) := to_unsigned(64, 8);              -- Rims gain (Q6)
    signal s_c0   : unsigned(10 downto 0) := to_unsigned(512, 11);           -- 512 + Swell
    signal s_e    : unsigned(7 downto 0) := to_unsigned(40, 8);              -- Etch gain (Q6)

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_zk   : std_logic := '0';
    signal e_lc   : unsigned(2 downto 0) := (others => '0');
    signal e_vpa  : unsigned(17 downto 0) := (others => '0');
    signal e_vp   : unsigned(16 downto 0) := (others => '0');
    signal e_dd   : signed(17 downto 0) := (others => '0');
    signal e_d    : unsigned(15 downto 0) := (others => '0');
    signal e_i    : unsigned(4 downto 0) := (others => '0');
    signal e_fy   : unsigned(4 downto 0) := (others => '0');
    signal e_run  : std_logic := '0';
    signal e_cnt  : unsigned(7 downto 0) := (others => '0');
    signal e_ka   : unsigned(11 downto 0) := (others => '0');
    signal e_kq   : std_logic_vector(7 downto 0) := (others => '0');
    signal e_kr, e_kh : unsigned(7 downto 0) := (others => '0');
    signal e_v0, e_v1, e_v2, e_v3, e_v4, e_v5 : std_logic := '0';           -- sample valid
    signal e_s0, e_s1, e_s2, e_s3, e_s4, e_s5 : std_logic := '0';           -- sample = row i+1
    type t_j is array (0 to 5) of unsigned(6 downto 0);
    signal e_j    : t_j := (others => (others => '0'));
    signal e_df   : signed(8 downto 0) := (others => '0');
    signal e_a5, e_a5d : unsigned(12 downto 0) := (others => '0');
    signal e_pr   : signed(14 downto 0) := (others => '0');
    signal e_R, e_Rp : unsigned(9 downto 0) := (others => '0');
    signal e_last : std_logic := '0';
    signal e_we   : std_logic := '0';
    signal e_wa   : unsigned(7 downto 0) := (others => '0');
    signal e_wd   : std_logic_vector(31 downto 0) := (others => '0');
    signal e_wb, s_rb : std_logic := '0';

    -- beam
    signal b_hp  : unsigned(19 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_dir : std_logic := '0';

    -- pixel path (pN = stage N)
    signal p1_ra  : unsigned(7 downto 0) := (others => '0');
    signal p1_fx, p2_fx, p3_fx, p4_fx : unsigned(5 downto 0) := (others => '0');
    signal p3_q   : std_logic_vector(31 downto 0) := (others => '0');
    signal p3_y8  : unsigned(7 downto 0) := (others => '0');
    signal p4_r0, p5_r0 : unsigned(9 downto 0) := (others => '0');
    signal p4_df  : signed(10 downto 0) := (others => '0');
    signal p4_ld  : signed(8 downto 0) := (others => '0');
    signal p5_pr  : signed(17 downto 0) := (others => '0');
    signal p5_lp  : signed(17 downto 0) := (others => '0');
    signal p6_sc  : signed(9 downto 0) := (others => '0');
    signal p6_lt, p7_lt, p8_lt : signed(11 downto 0) := (others => '0');
    signal p7_tg  : signed(18 downto 0) := (others => '0');
    signal p8_a   : signed(13 downto 0) := (others => '0');
    signal p9_ix, p10_ad : unsigned(8 downto 0) := (others => '0');
    signal p11_q, p12_q : std_logic_vector(31 downto 0) := (others => '0');
    signal p9_gg, p10_gg, p11_gg : std_logic := '0';                     -- gloss gate: glint classes
    signal p9_gf, p10_gf, p11_gf : std_logic := '0';                     -- gloss gate: fill classes
    signal b13_y, b14_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal b13_u, b14_u, b13_v, b14_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_yq, s_yr : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_uq, s_ur, s_vq, s_vr : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- gloss layer (measured glint / lens / eye runs, 2 slots per gloss column)
    signal l_d8   : unsigned(10 downto 0) := (others => '0');               -- line d in 1/8 lines
    signal p1_ga  : unsigned(7 downto 0) := (others => '0');
    signal p1_gf, p2_gf, p3_gf : unsigned(2 downto 0) := (others => '0');
    signal g2_q, g3_q : std_logic_vector(63 downto 0) := (others => '0');
    type t_u3x  is array (0 to 1) of unsigned(2 downto 0);
    type t_u4x  is array (0 to 1) of unsigned(3 downto 0);
    type t_u5x  is array (0 to 1) of unsigned(4 downto 0);
    type t_u7x  is array (0 to 1) of unsigned(6 downto 0);
    type t_u8x  is array (0 to 1) of unsigned(7 downto 0);
    type t_u9x  is array (0 to 1) of unsigned(8 downto 0);
    type t_u12x is array (0 to 1) of unsigned(11 downto 0);
    type t_s6x  is array (0 to 1) of signed(5 downto 0);
    type t_s8x  is array (0 to 1) of signed(7 downto 0);
    type t_s9x  is array (0 to 1) of signed(8 downto 0);
    type t_s12x is array (0 to 1) of signed(11 downto 0);
    type t_s13x is array (0 to 1) of signed(12 downto 0);
    type t_s14x is array (0 to 1) of signed(13 downto 0);
    type t_slx  is array (0 to 1) of std_logic;
    signal q4_da, q5_da : t_u7x := (others => (others => '0'));
    signal q4_dd : t_s8x := (others => (others => '0'));
    signal q4_vf : t_u4x := (others => (others => '0'));
    signal q4_w, q5_w, q6_w, q7_w : t_u9x := (others => (others => '0'));    -- half-width, 1/8 lines
    signal q4_aa, q5_aa : t_u5x := (others => (others => '0'));
    signal q4_ad : t_s6x := (others => (others => '0'));
    signal q4_c, q5_c, q6_c, q7_c, q8_c, q9_c, q10_c : t_u3x := (others => (others => '0'));
    signal q4_wd, q5_wd, q6_wd, q7_wd, q8_wd, q9_wd, q10_wd : t_slx := (others => '0');  -- wide run
    signal q4_f : unsigned(2 downto 0) := (others => '0');
    signal q5_pd : t_s12x := (others => (others => '0'));
    signal q5_pl : t_s9x := (others => (others => '0'));
    signal q6_dc : t_u12x := (others => (others => '0'));
    signal q6_an, q7_an, q8_an, q9_an : t_u8x := (others => (others => '0'));   -- amplitude 0..128
    signal q7_e  : t_s13x := (others => (others => '0'));
    signal q8_r1, q8_r2 : t_s14x := (others => (others => '0'));
    signal q9_r  : t_u5x := (others => (others => '0'));
    signal q10_al : t_u5x := (others => (others => '0'));
    signal p11_al, p12_al, p13_al : unsigned(4 downto 0) := (others => '0');
    signal p11_c, p12_c : unsigned(2 downto 0) := (others => '0');
    signal p13_dy, p13_du, p13_dv : signed(10 downto 0) := (others => '0');
    signal p14_py, p14_pu, p14_pv : signed(16 downto 0) := (others => '0');
    signal s_yo : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_uo, s_vo : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y14, s_y15, s_y16, s_y17, s_y18, s_y19x, s_yf19 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Video line buffer (collisions possible: read-before-write kept)
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    ---------------------------------------------------------------------------
    -- Row RAM (the engine writes one bank while the beam reads the other)
    ---------------------------------------------------------------------------
    p_row : process(clk)
    begin
        if rising_edge(clk) then
            r_q <= r_ram(to_integer(p1_ra));
            if e_we = '1' then r_ram(to_integer(e_wa)) <= e_wd; end if;
        end if;
    end process p_row;

    ---------------------------------------------------------------------------
    -- ROMs: knot grid (line engine), palette (pixel path)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            e_kq  <= C_AB_KNOT(to_integer(e_ka));
            p11_q <= C_AB_PAL(to_integer(p10_ad));
            g2_q  <= C_AB_GL(to_integer(p1_ga));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer.  Every zone-clock step is one
    -- small operation (the zone is quasi-static but timed at the pixel clock).
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_k  : integer range 0 to 63;
        variable v_r  : signed(18 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_hb  <= f_hstep(s_tid);

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
            elsif s_zc /= 511 then
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

            -- speeds: |v| - 8 and the still zone, registered free-running
            s_k4 <= unsigned(registers_in(3));
            s_k5 <= unsigned(registers_in(4));
            v_s := signed(resize(s_k4, 11)) - 512;
            if v_s > -9 and v_s < 9 then s_st4 <= '1'; else s_st4 <= '0'; end if;
            s_n4 <= v_s(10);
            if v_s(10) = '1' then v_m := unsigned(not v_s(9 downto 0)) - 7; else v_m := unsigned(v_s(9 downto 0)) - 8; end if;
            s_m4 <= v_m;
            v_s := signed(resize(s_k5, 11)) - 512;
            if v_s > -9 and v_s < 9 then s_st5 <= '1'; else s_st5 <= '0'; end if;
            s_n5 <= v_s(10);
            if v_s(10) = '1' then v_m := unsigned(not v_s(9 downto 0)) - 7; else v_m := unsigned(v_s(9 downto 0)) - 8; end if;
            s_m5 <= v_m;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_mirror <= registers_in(6)(0);
                s_swap   <= registers_in(6)(1);
                s_saw    <= registers_in(6)(2);
                s_flip   <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
            end if;
            s_half <= s_w(11 downto 1);
            s_hm1  <= s_half - 1;
            g_wm1  <= s_w - 1;

            -- shared glide (1/8 of the gap per field, +-2 deadband): knob k on
            -- zone clocks 3k+1 (select, distance), 3k+2 (step), 3k+3 (update)
            if s_zone = '1' and s_zc >= 1 and s_zc <= 15 then
                case to_integer(s_zc) is
                    when 1  => s_gm <= s_g1; s_gd <= signed(resize(unsigned(registers_in(0)) & "000", 15)) - signed(resize(s_g1, 15));
                    when 4  => s_gm <= s_g2; s_gd <= signed(resize(unsigned(registers_in(1)) & "000", 15)) - signed(resize(s_g2, 15));
                    when 7  => s_gm <= s_g3; s_gd <= signed(resize(unsigned(registers_in(2)) & "000", 15)) - signed(resize(s_g3, 15));
                    when 10 => s_gm <= s_g6; s_gd <= signed(resize(unsigned(registers_in(5)) & "000", 15)) - signed(resize(s_g6, 15));
                    when 13 => s_gm <= s_gp; s_gd <= signed(resize(unsigned(registers_in(7)) & "000", 15)) - signed(resize(s_gp, 15));
                    when 2 | 5 | 8 | 11 | 14 =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                    when 3  => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 6  => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 9  => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 12 => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                end case;
            end if;

            -- serial products: one slot every 24 zone clocks (k = 1, 4, 7 ..
            -- 25): load at 8k, 16 iterations, result stable from 8k + 17.
            v_k := to_integer(s_zc(8 downto 3));
            if s_zc(2 downto 0) = "000" and (v_k = 1 or v_k = 4 or v_k = 7 or v_k = 10 or v_k = 13 or
               v_k = 16 or v_k = 19 or v_k = 22 or v_k = 25) then
                f_acc <= (others => '0');
                case v_k is
                    when 1  => f_a <= resize(s_g1(12 downto 3), 36); f_b <= resize(s_g1(12 downto 3), 16);
                    when 4  => f_a <= resize(s_z1, 36);              f_b <= resize(s_hb, 16);
                    when 7  => f_a <= resize(s_st, 36);              f_b <= resize(s_half, 16);
                    when 10 => f_a <= resize(s_g2(12 downto 3), 36); f_b <= resize(s_g2(12 downto 3), 16);
                    when 13 => f_a <= resize(s_ys, 36);              f_b <= resize(s_z2, 16);
                    when 16 => f_a <= resize(s_m4, 36);              f_b <= resize(s_m4, 16);
                    when 19 => f_a <= resize(s_m5, 36);              f_b <= resize(s_m5, 16);
                    when 22 => f_a <= resize(s_g3(12 downto 3), 36); f_b <= resize(s_g3(12 downto 3), 16);
                    when others => f_a <= resize(s_z2, 36);          f_b <= to_unsigned(540, 16);
                end case;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;
            -- stores and shift-adds (one operation per zone clock)
            case to_integer(s_zc) is
                when 26  => s_z1 <= to_unsigned(128, 10) + resize(f_acc(19 downto 11), 10);
                when 50  => s_st <= f_acc(18 downto 8);
                when 74  => s_xt <= to_unsigned(262144, 20) - f_acc(19 downto 0);
                when 75  => s_x0 <= s_xt + s_pan;
                when 98  => s_z2 <= to_unsigned(128, 10) + resize(f_acc(19 downto 11), 10);
                when 122 => s_vst <= f_acc(19 downto 8);
                when 146 =>
                    if s_st4 = '1' then
                        s_pst <= (others => '0');
                    elsif s_n4 = '1' then
                        s_pst <= -signed(resize(f_acc(17 downto 6), 13));
                    else
                        s_pst <= signed(resize(f_acc(17 downto 6), 13));
                    end if;
                when 147 => s_pan <= unsigned(signed(s_pan) + resize(s_pst, 20));
                when 170 =>
                    if s_st5 = '1' then
                        s_rst <= (others => '0');
                    elsif s_n5 = '1' then
                        s_rst <= -signed(resize(f_acc(17 downto 4), 15));
                    else
                        s_rst <= signed(resize(f_acc(17 downto 4), 15));
                    end if;
                when 171 => s_rla <= signed(resize(s_roll, 19)) + resize(s_rst, 19);
                when 172 =>
                    if s_rla < 0 then
                        v_r := s_rla + C_VPER;
                    elsif s_rla >= C_VPER then
                        v_r := s_rla - C_VPER;
                    else
                        v_r := s_rla;
                    end if;
                    s_roll <= unsigned(v_r(16 downto 0));
                when 194 => s_g <= to_unsigned(32, 8) + resize(f_acc(19 downto 13), 8);
                -- vp0 = (59776 - 540 z2 + roll) mod 126720 (59776 + 3 x 126720 = 439936)
                when 218 => s_v0a <= to_unsigned(439936, 19) - f_acc(18 downto 0);
                when 219 => s_v0a <= s_v0a + resize(s_roll, 19);
                when 220 | 221 | 222 | 223 =>
                    if s_v0a >= C_VPER then s_v0a <= s_v0a - C_VPER; end if;
                when 224 => s_vp0 <= s_v0a(16 downto 0);
                -- Swell: 512 + (512 - p12) / 2
                when 225 => s_c0 <= to_unsigned(768, 11) - resize(s_gp(12 downto 4), 11);
                when 226 => s_e <= s_g6(12 downto 5);
                when others => null;
            end case;

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
    -- Source side: oscillator, line-buffer write of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_t : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then s_ox <= s_ox + 3; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 5; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zgo = '1' then s_ot <= s_ot + 3; end if;
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then
                v_t := resize(f_tri8(s_ox(9 downto 2) + s_ot), 8) + resize(f_tri8(s_oy(9 downto 2) - s_ot), 8);
                s_lb_wd <= std_logic_vector(to_unsigned(46, 8) + resize(v_t(7 downto 1), 8));
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line's row R[j] = K[i][j] + (K[i+1][j] - K[i][j]) fy
    -- (Q2, 0..1020) for the 128 knot columns into the bank the beam is not reading.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_nd  : signed(17 downto 0);
        variable v_sel : unsigned(4 downto 0);
        variable v_rs  : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 300 then e_zk <= '1'; end if;

            if g_start = '1' then
                s_rb <= e_wb;
                e_wb <= not e_wb;
                l_d8 <= e_d(15 downto 5);              -- this line's d (computed during the last one)
            end if;

            -- setup
            if e_zk = '1' then
                e_vpa <= resize(s_vp0, 18);
                e_lc  <= to_unsigned(1, 3);
            elsif g_start = '1' then
                e_vpa <= resize(e_vp, 18) + resize(s_vst, 18);
                e_lc  <= to_unsigned(1, 3);
            else
                case to_integer(e_lc) is
                    when 1 =>
                        if e_vpa >= C_VPER then e_vp <= resize(e_vpa - C_VPER, 17); else e_vp <= e_vpa(16 downto 0); end if;
                        e_lc <= to_unsigned(2, 3);
                    when 2 =>
                        e_dd <= signed(resize(e_vp, 18)) - C_VMID;
                        e_lc <= to_unsigned(3, 3);
                    when 3 =>
                        if s_saw = '1' then
                            e_d <= e_vp(16 downto 1);
                        elsif e_dd(17) = '1' then
                            v_nd := -e_dd;
                            e_d <= unsigned(v_nd(15 downto 0));
                        else
                            e_d <= unsigned(e_dd(15 downto 0));
                        end if;
                        e_lc <= to_unsigned(4, 3);
                    when 4 =>
                        e_i  <= e_d(15 downto 11);
                        e_fy <= e_d(10 downto 6);
                        e_lc <= to_unsigned(5, 3);
                    when 5 =>
                        e_run <= '1';
                        e_cnt <= (others => '0');
                        e_lc  <= to_unsigned(0, 3);
                    when others => null;
                end case;
            end if;

            -- column loop: 256 knot reads (row i, row i+1) per line
            e_v0 <= '0';
            if e_run = '1' then
                v_sel := (others => '0'); v_sel(0) := e_cnt(0);
                e_ka <= (e_i + v_sel) & e_cnt(7 downto 1);
                e_v0 <= '1'; e_s0 <= e_cnt(0); e_j(0) <= e_cnt(7 downto 1);
                if e_cnt = 255 then e_run <= '0'; end if;
                e_cnt <= e_cnt + 1;
            end if;
            -- 1: ROM (p_rom); 2: fabric copy
            e_v1 <= e_v0; e_s1 <= e_s0; e_j(1) <= e_j(0);
            e_kr <= unsigned(e_kq);
            e_v2 <= e_v1; e_s2 <= e_s1; e_j(2) <= e_j(1);
            -- 3: hold row i's knot; difference
            if e_v2 = '1' and e_s2 = '0' then e_kh <= e_kr; end if;
            e_df <= signed(resize(e_kr, 9)) - signed(resize(e_kh, 9));
            e_a5 <= e_kh & "00000";
            e_v3 <= e_v2; e_s3 <= e_s2; e_j(3) <= e_j(2);
            -- 4: (b - a) fy
            e_pr  <= e_df * signed(resize(e_fy, 6));
            e_a5d <= e_a5;
            e_v4 <= e_v3; e_s4 <= e_s3; e_j(4) <= e_j(3);
            -- 5: R
            v_rs := signed(resize(e_a5d, 15)) + e_pr;
            e_R <= unsigned(v_rs(12 downto 3));
            e_v5 <= e_v4; e_s5 <= e_s4; e_j(5) <= e_j(4);
            -- 6: write the pair (R[j-1], R[j]); after column 127 also (R[127], R[127])
            e_we <= '0';
            if e_v5 = '1' and e_s5 = '1' then
                e_Rp <= e_R;
                if e_j(5) /= 0 then
                    e_we <= '1';
                    e_wa <= e_wb & (e_j(5) - 1);
                    e_wd <= "000000" & std_logic_vector(e_Rp) & "000000" & std_logic_vector(e_R);
                end if;
                if e_j(5) = 127 then e_last <= '1'; end if;
            elsif e_last = '1' then
                e_last <= '0';
                e_we <= '1';
                e_wa <= e_wb & "1111111";
                e_wd <= "000000" & std_logic_vector(e_Rp) & "000000" & std_logic_vector(e_Rp);
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: horizontal phase (Mirror: reversed after the centre pixel)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_hp  <= s_x0;
                b_px  <= (others => '0');
                b_dir <= '0';
            elsif g_avid = '1' then
                if b_dir = '1' then
                    b_hp <= b_hp - resize(s_st, 20);
                else
                    b_hp <= b_hp + resize(s_st, 20);
                end if;
                b_px <= b_px + 1;
                if s_mirror = '1' and b_px = s_hm1 then b_dir <= '1'; end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_mu : unsigned(10 downto 0);
        variable v_mf : unsigned(1 downto 0);
        variable v_s  : unsigned(9 downto 0);
        variable v_ss : signed(11 downto 0);
        variable v_t  : signed(13 downto 0);
        variable v_y  : unsigned(9 downto 0);
        variable v_bl : signed(11 downto 0);
        variable v_bu, v_bv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: triangle fold of the horizontal phase -> knot column j, fraction fx
            if b_hp(19) = '1' then
                v_mu := not b_hp(18 downto 8); v_mf := not b_hp(7 downto 6);
            else
                v_mu := b_hp(18 downto 8); v_mf := b_hp(7 downto 6);
            end if;
            p1_ra <= s_rb & v_mu(10 downto 4);
            p1_fx <= v_mu(3 downto 0) & v_mf;
            p1_ga <= v_mu(10 downto 3);                -- gloss column (8 units = 7.5 virtual px)
            p1_gf <= v_mu(2 downto 0);

            -- 2: row RAM (p_row)
            p2_fx <= p1_fx;

            -- 3: fabric copy; luma
            p3_q  <= r_q;
            p3_fx <= p2_fx;
            p3_y8 <= unsigned(s_lb_q);

            -- 4: R1 - R0; luma - 110
            p4_r0 <= unsigned(p3_q(25 downto 16));
            p4_df <= signed(resize(unsigned(p3_q(9 downto 0)), 11)) - signed(resize(unsigned(p3_q(25 downto 16)), 11));
            p4_fx <= p3_fx;
            p4_ld <= signed(resize(p3_y8, 9)) - 110;

            -- 5: products
            p5_pr <= p4_df * signed(resize(p4_fx, 7));
            p5_r0 <= p4_r0;
            p5_lp <= p4_ld * signed(resize(s_e, 9));

            -- 6: s = R0 + df fx / 64 (0..1020); s - 512; luma term
            v_ss := signed(resize(p5_r0, 12)) + p5_pr(17 downto 6);
            v_s := unsigned(v_ss(9 downto 0));
            p6_sc <= signed((not v_s(9)) & v_s(8 downto 0));
            p6_lt <= p5_lp(17 downto 6);

            -- 7: Rims gain
            p7_tg <= p6_sc * signed(resize(s_g, 9));
            p7_lt <= p6_lt;

            -- 8: 512 + Swell + gain term
            p8_a  <= signed(resize(s_c0, 14)) + resize(p7_tg(18 downto 6), 14);
            p8_lt <= p7_lt;

            -- 9: + Etch, clip, Flip -> palette index
            v_t := p8_a + resize(p8_lt, 14);
            if v_t < 0 then v_y := (others => '0');
            elsif v_t > 1023 then v_y := (others => '1');
            else v_y := unsigned(v_t(9 downto 0)); end if;
            if v_y < 768 then p9_gg <= '1'; else p9_gg <= '0'; end if;
            if v_y < 960 then p9_gf <= '1'; else p9_gf <= '0'; end if;
            if s_flip = '1' then v_y := not v_y; end if;
            p9_ix <= v_y(9 downto 1);

            -- 10: palette address; 11: ROM (p_rom); 12: fabric copy
            p10_ad <= p9_ix;
            p12_q <= p11_q;

            p10_gg <= p9_gg; p11_gg <= p10_gg;
            p10_gf <= p9_gf; p11_gf <= p10_gf;

            -- 13, 14: the base colour waits for the gloss layer
            b13_y <= unsigned(p12_q(31 downto 22)); b13_u <= unsigned(p12_q(21 downto 12)); b13_v <= unsigned(p12_q(11 downto 2));
            b14_y <= b13_y; b14_u <= b13_u; b14_v <= b13_v;

            -- 15: blend the gloss colour in; Palette Swap exchanges U and V
            v_bl := signed(resize(b14_y, 12)) + resize(shift_right(p14_py, 4), 12);
            s_yq <= unsigned(v_bl(9 downto 0));
            v_bl := signed(resize(b14_u, 12)) + resize(shift_right(p14_pu, 4), 12);
            v_bu := unsigned(v_bl(9 downto 0));
            v_bl := signed(resize(b14_v, 12)) + resize(shift_right(p14_pv, 4), 12);
            v_bv := unsigned(v_bl(9 downto 0));
            if s_swap = '1' then
                s_uq <= v_bv; s_vq <= v_bu;
            else
                s_uq <= v_bu; s_vq <= v_bv;
            end if;
            -- 16, 17: latency padding (C_LAT stays a multiple of 4)
            s_yr <= s_yq; s_ur <= s_uq; s_vr <= s_vq;
            s_yo <= s_yr; s_uo <= s_ur; s_vo <= s_vr;
        end if;
    end process p_pix;


    ---------------------------------------------------------------------------
    -- Gloss layer.  Per slot, the run's centre and amplitude are lerped between
    -- gloss columns c and c + 1 (the centre is held at the stronger end where
    -- the table marks a jump), the half-width is the stronger end's;
    -- alpha = amplitude x clamp((w - |d - centre|) / 2 lines, 0, 1).  The
    -- stronger slot wins (ties: the narrow run), gated by the base index
    -- (glints only on the beads and rims, fills not on the band centre), and
    -- its colour is blended over the base colour.
    ---------------------------------------------------------------------------
    p_gloss : process(clk)
        -- half-width classes x 8 (1.5, 2.5, 3.5, 5, 8, 13, 22, 40 lines); amplitudes x 16
        function f_wq8(c : std_logic_vector(2 downto 0)) return unsigned is
        begin
            case c is
                when "000" => return to_unsigned(12, 9);
                when "001" => return to_unsigned(20, 9);
                when "010" => return to_unsigned(28, 9);
                when "011" => return to_unsigned(40, 9);
                when "100" => return to_unsigned(64, 9);
                when "101" => return to_unsigned(104, 9);
                when "110" => return to_unsigned(176, 9);
                when others => return to_unsigned(320, 9);
            end case;
        end function;
        function f_am(a : std_logic_vector(1 downto 0)) return unsigned is
        begin
            case a is
                when "00" => return to_unsigned(0, 5);
                when "01" => return to_unsigned(5, 5);
                when "10" => return to_unsigned(11, 5);
                when others => return to_unsigned(16, 5);
            end case;
        end function;
        variable v_ea, v_eb : std_logic_vector(15 downto 0);
        variable v_es : std_logic_vector(15 downto 0);
        variable v_a16a, v_a16b : unsigned(4 downto 0);
        variable v_s1 : std_logic;
        variable v_dc : signed(12 downto 0);
        variable v_an : signed(9 downto 0);
        variable v_r  : signed(13 downto 0);
        variable v_p  : unsigned(12 downto 0);
        variable v_gc : std_logic_vector(29 downto 0);
        variable v_al : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: ROM (p_rom); 3: fabric copy
            p2_gf <= p1_gf;
            g3_q <= g2_q; p3_gf <= p2_gf;

            -- 4: decode both columns' entries of each slot
            q4_f <= p3_gf;
            for n in 0 to 1 loop
                v_ea := g3_q(16 * n + 15 downto 16 * n);
                v_eb := g3_q(16 * (n + 2) + 15 downto 16 * (n + 2));
                if unsigned(v_eb(4 downto 3)) > unsigned(v_ea(4 downto 3)) or
                   (v_eb(4 downto 3) = v_ea(4 downto 3) and p3_gf(2) = '1') then v_s1 := '1'; else v_s1 := '0'; end if;
                if v_s1 = '1' then v_es := v_eb; else v_es := v_ea; end if;
                v_a16a := f_am(v_ea(4 downto 3));
                v_a16b := f_am(v_eb(4 downto 3));
                q4_da(n) <= unsigned(v_ea(15 downto 9));
                q4_dd(n) <= signed(resize(unsigned(v_eb(15 downto 9)), 8)) - signed(resize(unsigned(v_ea(15 downto 9)), 8));
                if v_ea(8) = '1' then                       -- jump: hold the stronger end's centre
                    if v_s1 = '1' then q4_vf(n) <= to_unsigned(8, 4); else q4_vf(n) <= (others => '0'); end if;
                else
                    q4_vf(n) <= '0' & p3_gf;
                end if;
                q4_w(n)  <= f_wq8(v_es(7 downto 5));
                q4_wd(n) <= v_es(7) and (v_es(6) or v_es(5));
                q4_aa(n) <= v_a16a;
                q4_ad(n) <= signed(resize(v_a16b, 6)) - signed(resize(v_a16a, 6));
                q4_c(n)  <= unsigned(v_es(2 downto 0));
            end loop;

            for n in 0 to 1 loop
                -- 5: fraction products
                q5_pd(n) <= resize(q4_dd(n) * signed('0' & q4_vf(n)), 12);
                q5_pl(n) <= resize(q4_ad(n) * signed('0' & q4_f), 9);
                q5_da(n) <= q4_da(n); q5_w(n) <= q4_w(n); q5_aa(n) <= q4_aa(n);
                q5_c(n) <= q4_c(n); q5_wd(n) <= q4_wd(n);

                -- 6: centre (1/8 lines) and amplitude (0..128)
                v_dc := signed(resize(q5_da(n) & "0000", 13)) + shift_left(resize(q5_pd(n), 13), 1);
                q6_dc(n) <= unsigned(v_dc(11 downto 0));
                v_an := signed(resize(q5_aa(n) & "000", 10)) + resize(q5_pl(n), 10);
                q6_an(n) <= unsigned(v_an(7 downto 0));
                q6_w(n) <= q5_w(n); q6_c(n) <= q5_c(n); q6_wd(n) <= q5_wd(n);

                -- 7: d - centre
                q7_e(n) <= signed(resize(l_d8, 13)) - signed(resize(q6_dc(n), 13));
                q7_w(n) <= q6_w(n); q7_an(n) <= q6_an(n); q7_c(n) <= q6_c(n); q7_wd(n) <= q6_wd(n);

                -- 8: w - (d - centre), w + (d - centre)
                q8_r1(n) <= signed(resize(q7_w(n), 14)) - resize(q7_e(n), 14);
                q8_r2(n) <= signed(resize(q7_w(n), 14)) + resize(q7_e(n), 14);
                q8_an(n) <= q7_an(n); q8_c(n) <= q7_c(n); q8_wd(n) <= q7_wd(n);

                -- 9: w - |d - centre|, clamped to 0..16 (a 2-line soft edge)
                if q8_r1(n) < q8_r2(n) then v_r := q8_r1(n); else v_r := q8_r2(n); end if;
                if v_r < 0 then q9_r(n) <= (others => '0');
                elsif v_r > 16 then q9_r(n) <= to_unsigned(16, 5);
                else q9_r(n) <= unsigned(v_r(4 downto 0)); end if;
                q9_an(n) <= q8_an(n); q9_c(n) <= q8_c(n); q9_wd(n) <= q8_wd(n);

                -- 10: alpha (0..16) = amplitude x edge
                v_p := q9_an(n) * q9_r(n);
                q10_al(n) <= v_p(11 downto 7);
                q10_c(n) <= q9_c(n); q10_wd(n) <= q9_wd(n);
            end loop;

            -- 11: the stronger slot (ties: the narrow run, then slot 0)
            if q10_al(0) > q10_al(1) or (q10_al(0) = q10_al(1) and (q10_wd(0) = '0' or q10_wd(1) = '1')) then
                p11_al <= q10_al(0); p11_c <= q10_c(0);
            else
                p11_al <= q10_al(1); p11_c <= q10_c(1);
            end if;

            -- 12: gate by the base index (fills: classes 5 and 6)
            if p11_c = 5 or p11_c = 6 then
                if p11_gf = '1' then p12_al <= p11_al; else p12_al <= (others => '0'); end if;
            else
                if p11_gg = '1' then p12_al <= p11_al; else p12_al <= (others => '0'); end if;
            end if;
            p12_c <= p11_c;

            -- 13: gloss colour - base; 14: x alpha / 16 (stage 15 adds it, p_pix)
            v_gc := f_ab_gc(p12_c);
            p13_dy <= signed(resize(unsigned(v_gc(29 downto 20)), 11)) - signed(resize(unsigned(p12_q(31 downto 22)), 11));
            p13_du <= signed(resize(unsigned(v_gc(19 downto 10)), 11)) - signed(resize(unsigned(p12_q(21 downto 12)), 11));
            p13_dv <= signed(resize(unsigned(v_gc(9 downto 0)), 11)) - signed(resize(unsigned(p12_q(11 downto 2)), 11));
            p13_al <= p12_al;
            p14_py <= resize(p13_dy * signed('0' & p13_al), 17);
            p14_pu <= resize(p13_du * signed('0' & p13_al), 17);
            p14_pv <= resize(p13_dv * signed('0' & p13_al), 17);
        end if;
    end process p_gloss;

    ---------------------------------------------------------------------------
    -- Output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent
    -- 8-sums, luma [1 2 1]; centre at stage 22 -> 23), luma 13/16 and the
    -- blanking gate at stage 24
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_csu(0) <= s_uo; s_csv(0) <= s_vo;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_uo - s_csu(7);
            s_boxv <= s_boxv + s_vo - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u19 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v19 <= v_sc(13 downto 4);
            s_y14 <= s_yo; s_y15 <= s_y14; s_y16 <= s_y15; s_y17 <= s_y16;
            s_y18 <= s_y17; s_y19x <= s_y18;
            v_sy := resize(s_y17, 12) + shift_left(resize(s_y18, 12), 1) + resize(s_y19x, 12);
            s_yf19 <= v_sy(11 downto 2);
            if s_yf19 > 64 then v_dl := s_yf19 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y20 <= s_yf19 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u20 <= s_u19; s_v20 <= s_v19;
            else
                s_y20 <= to_unsigned(64, 10); s_u20 <= to_unsigned(512, 10); s_v20 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture abacus;
