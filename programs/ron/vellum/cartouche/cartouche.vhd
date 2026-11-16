-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Cartouche v0.1.0 -- Vellum.  One nested figure filling the screen: bands of constant distance from a core
-- rectangle.  Inside the core they are sharp nested rectangles; outside, rounded rectangles whose corners
-- round off band by band until the far bands are near circles.  After Egyptian cartouches, Frank Stella's
-- concentric stripe paintings, the rounded-rectangle TV screen and Josef Albers' nested squares, softened.
--
--   D     = sdBox(p, b') + e,   b' = max(b + e, 0) per axis          (the band phase is D / P)
--           e > 0: square-cornered bands out to distance e (Round < 50 %, 0 % = Albers all the way out)
--           e = 0: the true signed distance of the core box (50 %)
--           e < 0: a rounded core (corner radius -e), then a stadium, then an oval core (100 %)
--   phase = D / P - drift + camera;  Grey: bands step black, dark, light, white;  Ink: black / white
--   Pair  = the same figure turned 90 degrees, a translucent film streaming the other way at 3/4 speed
--
--   K1  X       the figure's centre, +-1920 virtual px      K2  Y   +-1080 virtual px
--   K3  Scale   band period 12 .. 200 virtual px (exponential, default 49)
--   K4  Core    the core box: horizontal line .. wide 2.8:1 (default) .. square .. vertical line
--   K5  Drift   the bands stream outward (up) / inward (down); centre +-24 = still; 2^-12 .. 2^-4.1 of
--               4 bands per field (one Grey tone cycle), exponential
--   K6  Camera  the luma added to the band phase, 0 .. 3.4 bands at white (default 100 %)
--   S7  Glide   Off = the knobs act this field;  On = 1/16 of the gap per field (Drift speed 1/8)
--   S8  Pair    Single / Pair (Grey: the two films averaged;  Ink: XOR moire)
--   S9  Tone    Grey / Ink
--   S10 Weight  Bold (equal bands) / Fine (a line of 1/6 band on every band edge)
--   S11 Source  Video / Osc (the shell's drifting triangle lattice at twice the camera level)
--   P12 Round   how the corners round: 0 % square-cornered, 50 % the true distance, 100 % round fast
--
-- The distance runs in "cordic units" (2^I / (K8 P) per virtual px, K8 = 1.64674 the gain of 8 vectoring
-- iterations, I = the period's octave, so every period uses the same word length): the figure feeds
-- (max(qx, 0), max(qy, 0)), or (not max(qx, qy), 0) inside the core, to an 8-stage CORDIC whose x output,
-- shifted down by I, is D / P in bands, with no multiplier anywhere in the pixel path.  With Pair the one
-- CORDIC takes the figure on the even pixels and the turned figure on the odd ones; each film's phase on the
-- other pixels is the mean of its two neighbours (the phase is smooth: exact to a small fraction of a pixel).
-- The figure's position, size, Round offset and the band period are folded into per-field constants on one
-- serial multiplier; x is a beam DDA, y a per-line DDA.
--
-- Sections (the integer model .vellum_work/rb_cartouche/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         +-1 hold, shared glide (S7), switches, Drift speed glide
--   [C] FIELD SEQUENCER                  one serial multiplier: the period, core, Round offset, position
--   [L] LINE ENGINE                      the y DDA and both figures' |y| - b'
--   [P] PIXEL PATH                       x DDA -> |x| - b' -> CORDIC -> phase (Pair: films interleaved)
--                                        -> coverage (a 4-step divider per film) -> tone -> fade
--   [D] SOURCE FIELD                     luma (or Osc) * K6 -> 128/64-px box -> vertical one-pole -> phase
--   [F] OUTPUT CONDITIONING              luma [1 2 1] / 4, 13/16 toward black, blanking gate; U = V = 512
--
-- Stage numbering: a register "at stage S" holds pixel n at T + S + n (T = the g_start clock).  Pixels
-- 0 .. W + 1 are rendered; output column j is the [1 2 1] filter over pixels j .. j + 2 and lands on screen
-- column j + 4 (the generated avid starts 4 clocks after the delayed source avid), so pixel m sits at screen
-- column m + 3.  s_gav_sr(S - 3) flags pixels 0 .. W - 1 on their way into a stage-S register.  One latency
-- (40) in every mode.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.cartouche_pkg.all;
use work.all;

architecture cartouche of program_top is
    constant C_LAT : integer := 40;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 512;
    constant C_DEF_K4  : integer := 266;
    constant C_DEF_K5  : integer := 680;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 512;

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
    function f_as(a, b : signed; sub : std_logic) return signed is
        variable v_a, v_b, v_r : signed(a'length downto 0);
        variable v_o : signed(a'length - 1 downto 0);
    begin
        v_a := a & '1';
        if sub = '1' then v_b := (not b) & '1'; else v_b := b & '0'; end if;
        v_r := v_a + v_b;
        v_o := v_r(a'length downto 1);
        return v_o;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
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

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k): pixels 0 .. W - 1 at stage k + 2
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 15) := (others => '0');  -- s_st(k) = g_start delayed k + 1

    -- [B] knobs: +-1 hold (10 bits), glide (Q3)
    signal h_1 : unsigned(9 downto 0) := to_unsigned(C_DEF_K1, 10);
    signal h_2 : unsigned(9 downto 0) := to_unsigned(C_DEF_K2, 10);
    signal h_3 : unsigned(9 downto 0) := to_unsigned(C_DEF_K3, 10);
    signal h_4 : unsigned(9 downto 0) := to_unsigned(C_DEF_K4, 10);
    signal h_5 : unsigned(9 downto 0) := to_unsigned(C_DEF_K5, 10);
    signal h_6 : unsigned(9 downto 0) := to_unsigned(C_DEF_K6, 10);
    signal h_p : unsigned(9 downto 0) := to_unsigned(C_DEF_P12, 10);
    signal g_1 : unsigned(12 downto 0) := to_unsigned(C_DEF_K1 * 8, 13);
    signal g_2 : unsigned(12 downto 0) := to_unsigned(C_DEF_K2 * 8, 13);
    signal g_3 : unsigned(12 downto 0) := to_unsigned(C_DEF_K3 * 8, 13);
    signal g_4 : unsigned(12 downto 0) := to_unsigned(C_DEF_K4 * 8, 13);
    signal g_6 : unsigned(12 downto 0) := to_unsigned(C_DEF_K6 * 8, 13);
    signal g_p : unsigned(12 downto 0) := to_unsigned(C_DEF_P12 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(13 downto 0) := (others => '0');
    signal s_gr, s_gh : unsigned(9 downto 0) := (others => '0');
    signal z_gp, z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_glide : std_logic := '1';
    signal s_pair, s_ink, s_fine, s_src : std_logic := '0';
    signal s_lg7 : std_logic := '1';                               -- box: 128 samples (1080) / 64
    signal s_ish : unsigned(2 downto 0) := to_unsigned(5, 3);      -- the vertical one-pole's shift
    signal s_vtg, s_vdd : signed(16 downto 0) := (others => '0');
    signal s_vgs : signed(16 downto 0) := to_signed((C_DEF_K5 - 512) * 64, 17);   -- Drift speed glide, Q6

    -- ROM: 2^(i/256)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');

    -- [C] the serial multiplier and the sequencer's scratch
    signal f_a, f_acc : unsigned(43 downto 0) := (others => '0');
    signal f_b : unsigned(20 downto 0) := (others => '0');
    signal z_fa : unsigned(23 downto 0) := (others => '0');
    signal z_fb : unsigned(20 downto 0) := (others => '0');
    signal z_sx : std_logic := '0';
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_oct : unsigned(10 downto 0) := (others => '0');
    signal z_sh  : unsigned(2 downto 0) := (others => '0');
    signal z_H   : unsigned(20 downto 0) := to_unsigned(342039, 21);
    signal z_G   : unsigned(19 downto 0) := to_unsigned(207708, 20);
    signal z_lo  : std_logic := '0';
    signal z_u12, z_w12 : unsigned(12 downto 0) := (others => '0');
    signal z_ax, z_ay, z_E : unsigned(15 downto 0) := (others => '0');
    signal z_elo : unsigned(14 downto 0) := (others => '0');
    signal z_d1, z_d2 : signed(13 downto 0) := (others => '0');
    signal z_xo, z_yo : signed(17 downto 0) := (others => '0');
    signal z_u0, z_v0 : signed(17 downto 0) := (others => '0');
    signal z_bx, z_by, z_bm : unsigned(12 downto 0) := (others => '0');
    signal z_r3  : unsigned(13 downto 0) := (others => '0');
    signal z_e4  : signed(17 downto 0) := (others => '0');
    signal z_bpx, z_bpy : signed(18 downto 0) := (others => '0');
    signal z_av  : unsigned(9 downto 0) := (others => '0');
    signal z_vz, z_vn : std_logic := '1';
    signal z_ls  : unsigned(9 downto 0) := (others => '0');
    signal z_spd : unsigned(13 downto 0) := (others => '0');
    signal z_ssh : unsigned(2 downto 0) := (others => '0');
    signal z_sp2 : unsigned(13 downto 0) := (others => '0');
    signal z_dl1, z_dl2 : unsigned(17 downto 0) := (others => '0');   -- Drift accumulators, Q18 of 4 bands

    -- [C] per-field constants
    signal c_kx  : unsigned(17 downto 0) := to_unsigned(19472, 18);    -- cordic units per pixel, Q20
    signal c_ky  : unsigned(19 downto 0) := to_unsigned(19472, 20);    -- cordic units per field line, Q20
    signal c_w0  : unsigned(12 downto 0) := to_unsigned(125, 13);      -- bands per pixel, Q12
    signal c_b1x : unsigned(20 downto 0) := to_unsigned(10944, 21);    -- b'x, cordic units Q12
    signal c_b1y : unsigned(20 downto 0) := to_unsigned(3844, 21);     -- b'y
    signal c_I   : unsigned(2 downto 0) := to_unsigned(2, 3);          -- the cordic units' octave
    signal z_t   : unsigned(19 downto 0) := (others => '0');
    signal c_e   : unsigned(13 downto 0) := (others => '0');           -- e / P, Q12 bands mod 4
    signal c_u0  : signed(29 downto 0) := (others => '0');             -- pixel 0's x, Q20
    signal c_v0  : signed(29 downto 0) := (others => '0');             -- line 0's y, Q20
    signal c_k1, c_k2 : unsigned(13 downto 0) := (others => '0');      -- e / P - drift, per film

    -- [L] line engine
    signal e_st  : unsigned(2 downto 0) := (others => '1');
    signal n_vb  : signed(29 downto 0) := (others => '0');
    signal n_avb : unsigned(20 downto 0) := (others => '0');
    signal n_q1y, n_q2y : signed(21 downto 0) := (others => '0');
    signal l_q1y, l_q2y : signed(21 downto 0) := (others => '0');

    -- [P] pixel path
    signal b_u    : signed(29 downto 0) := (others => '0');
    signal b_par, s2_f, s3_f : std_logic := '0';
    signal s2_au  : unsigned(20 downto 0) := (others => '0');
    signal s3_qx, s3_qy : signed(21 downto 0) := (others => '0');
    type t_cx is array (0 to 8) of unsigned(20 downto 0);
    type t_cy is array (0 to 8) of signed(21 downto 0);
    signal cx : t_cx := (others => (others => '0'));
    signal cy : t_cy := (others => (others => '0'));
    signal cn, cf : std_logic_vector(0 to 8) := (others => '0');
    signal s13_xs : unsigned(13 downto 0) := (others => '0');
    signal s13_n, s13_f, f14, f15, f16 : std_logic := '0';
    signal z14, z15, z16, s16_a, s16_df : unsigned(13 downto 0) := (others => '0');
    signal s17_p1, s17_p2 : unsigned(13 downto 0) := (others => '0');
    signal s18_ph1, s18_ph2 : unsigned(13 downto 0) := (others => '0');
    type t_w is array (18 to 28) of unsigned(12 downto 0);
    signal s_wp   : t_w := (others => (others => '0'));
    signal s19_wh : unsigned(12 downto 0) := (others => '0');
    type t_e is array (1 to 2) of signed(12 downto 0);
    signal s19_e  : t_e := (others => (others => '0'));
    type t_e15 is array (1 to 2) of signed(14 downto 0);
    signal s20_e  : t_e15 := (others => (others => '0'));
    -- per film: the tones either side (stages 19 .. 26), the divider (5 stages)
    type t_t2 is array (19 to 26) of unsigned(1 downto 0);
    signal s_tn1, s_to1, s_tn2, s_to2 : t_t2 := (others => "00");
    type t_dv13 is array (0 to 4) of unsigned(12 downto 0);
    type t_dv4 is array (0 to 4) of unsigned(3 downto 0);
    signal d1r, d2r, dvw : t_dv13 := (others => (others => '0'));
    signal d1q, d2q : t_dv4 := (others => (others => '0'));
    signal d1z, d1f, d2z, d2f : std_logic_vector(0 to 4) := (others => '0');
    signal s26_cv1, s26_cv2 : unsigned(4 downto 0) := (others => '0');
    signal s27_v1, s27_v2 : unsigned(5 downto 0) := (others => '0');
    signal s28_s : unsigned(6 downto 0) := (others => '0');
    signal s28_v1 : unsigned(5 downto 0) := (others => '0');
    signal s29_t, s30_t, s31_t : unsigned(6 downto 0) := (others => '0');
    signal s29_g, s30_g, s31_g : unsigned(4 downto 0) := (others => '0');
    signal s30_dm : signed(7 downto 0) := (others => '0');
    signal s31_pr : signed(11 downto 0) := (others => '0');
    signal c_mean : unsigned(6 downto 0) := to_unsigned(48, 7);
    signal s32_t  : unsigned(6 downto 0) := (others => '0');
    signal s33_la : unsigned(9 downto 0) := (others => '0');
    signal s33_lb : unsigned(6 downto 0) := (others => '0');
    signal s34_y  : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_l9  : unsigned(8 downto 0) := (others => '0');
    signal s_g8  : unsigned(7 downto 0) := (others => '1');
    signal s_p, s_sfirst : unsigned(12 downto 0) := (others => '0');
    signal s_sv, s_sv1, s_svd, s_bv, s_ev : std_logic := '0';
    signal s_col, s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    type t_bx is array (0 to 255) of unsigned(15 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(15 downto 0) := (others => '0');
    signal s_bcnt : unsigned(7 downto 0) := (others => '0');
    signal s_bsum : unsigned(19 downto 0) := (others => '0');
    signal s_eb  : unsigned(12 downto 0) := (others => '0');
    signal s_eq  : unsigned(15 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal s_upd, c_a, c_b, c_dy, s_Ls, s_wcam : unsigned(12 downto 0) := (others => '0');
    signal c_new : std_logic := '0';

    -- [F] output conditioning
    signal s35_y, s36_y, s37_y, s38_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s38_ya : unsigned(11 downto 0) := (others => '0');
    signal s39_y, s40_y, s41_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROM and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_CT_EXP(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
        end if;
    end process p_rom;

    -- box delay: read N samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv1 = '1' then bx_ram(to_integer(bx_wp)) <= resize(s_p, 16); end if;
        end if;
    end process p_bxram;

    -- luma line store (half horizontal resolution): the one-pole's own read, 2 addresses ahead of its write
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q <= lb_ram(to_integer(s_col(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + [C] the per-field
    -- sequencer (one small operation per zone clock; the zone is quasi-static
    -- but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_d  : signed(10 downto 0);
        variable v_x  : unsigned(43 downto 0);
        variable v_v  : signed(10 downto 0);
        variable v_n11 : signed(10 downto 0);
        variable v_ne : signed(11 downto 0);
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

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_pair  <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                if s_xsr = 2 then s_lg7 <= '1'; else s_lg7 <= '0'; end if;
                if s_ys <= 384 then s_ish <= to_unsigned(5, 3);
                elsif s_ys >= 960 then s_ish <= to_unsigned(3, 3);
                else s_ish <= to_unsigned(4, 3); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- [B] shared hold + glide, five clocks per channel.  Hold: the knob moves the held value only by
            -- 2 counts or more (or to either end).  Glide On: 1/16 of the gap per field, never slower than
            -- one step (1/8 count; X and Y two steps), landing exactly.  Glide Off: the held value, this field.
            if s_zgo = '1' then
                z_gp <= "000"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case to_integer(z_gp) is
                    when 0 =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= g_1; s_gh <= h_1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= g_2; s_gh <= h_2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= g_3; s_gh <= h_3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= g_4; s_gh <= h_4; s_gr <= unsigned(registers_in(3));
                            when 4      => s_gm <= g_6; s_gh <= h_6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= g_p; s_gh <= h_p; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "001";
                    when 1 =>
                        v_d := signed(resize(s_gr, 11)) - signed(resize(s_gh, 11));
                        if s_gr = 0 or s_gr = 1023 or v_d > 1 or v_d < -1 then s_gh <= s_gr; end if;
                        z_gp <= "010";
                    when 2 =>
                        s_gd <= signed(resize(s_gh & "000", 14)) - signed(resize(s_gm, 14));
                        z_gp <= "011";
                    when 3 =>
                        if s_glide = '1' then
                            if z_gc(2 downto 1) = "00" then                -- X, Y: at least 2 steps
                                if s_gd > 32 or s_gd < -32 then s_gd <= shift_right(s_gd, 4);
                                elsif s_gd > 2 then s_gd <= to_signed(2, 14);
                                elsif s_gd < -2 then s_gd <= to_signed(-2, 14);
                                end if;
                            else
                                if s_gd > 16 or s_gd < -16 then s_gd <= shift_right(s_gd, 4);
                                elsif s_gd > 1 then s_gd <= to_signed(1, 14);
                                elsif s_gd < -1 then s_gd <= to_signed(-1, 14);
                                end if;
                            end if;
                        end if;
                        z_gp <= "100";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => g_1 <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_1 <= s_gh;
                            when 1      => g_2 <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_2 <= s_gh;
                            when 2      => g_3 <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_3 <= s_gh;
                            when 3      => g_4 <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_4 <= s_gh;
                            when 4      => g_6 <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_6 <= s_gh;
                            when others => g_p <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_p <= s_gh;
                        end case;
                        z_gp <= "000";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] serial products: a slot is decoded at zc = P (below), its operands are pre-muxed one
            -- clock later, it loads the clock after that, runs 20 iterations and is stored at P + 24.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 72  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 100 => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 128 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 156 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 240 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 296 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 328 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 356 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when 384 => z_sel <= to_unsigned(12, 4); z_pre <= '1';
                    when 412 => z_sel <= to_unsigned(13, 4); z_pre <= '1';
                    when 440 => z_sel <= to_unsigned(14, 4); z_pre <= '1';
                    when 468 => z_sel <= to_unsigned(15, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_sx <= '0';
                case to_integer(z_sel) is
                    when 1  => z_fa <= resize(r_exf, 24); z_fb <= to_unsigned(43691, 21);     -- H = 2^24 / P
                    when 2  => z_fa <= resize(r_exf, 24); z_fb <= to_unsigned(26532, 21);     -- G = H / K8
                    when 3  => z_fa <= resize(z_G, 24);  z_fb <= resize(s_ys, 21);            -- per field line
                    when 4  =>                                                                -- the pixel's footprint
                        z_fa <= resize(z_H, 24);
                        if resize(s_xs & "0000000", 12) > s_ys then z_fb <= resize(s_xs & "0000000", 21);
                        else z_fb <= resize(s_ys, 21); end if;
                    when 7  => z_fa <= resize(z_E, 24); z_fb <= resize(z_u12, 21);            -- Round: E u
                    when 9  => z_fa <= resize(z_bm, 24); z_fb <= resize(z_w12, 21);           -- max(b) w
                    when 10 => z_fa <= resize(unsigned(z_bpx(16 downto 0)), 24); z_fb <= resize(z_G, 21);   -- b' in cordic units
                    when 11 => z_fa <= resize(unsigned(z_bpy(16 downto 0)), 24); z_fb <= resize(z_G, 21);
                    -- signed operands are sign-extended into the 44-bit shifter: the product is exact mod 2^44
                    when 12 => z_fa <= unsigned(resize(z_e4, 24)); z_fb <= z_H; z_sx <= '1';  -- e / P
                    when 13 => z_fa <= unsigned(resize(z_u0, 24)); z_fb <= resize(z_G, 21); z_sx <= '1';   -- pixel 0's x
                    when 14 => z_fa <= unsigned(resize(z_v0, 24)); z_fb <= resize(z_G, 21); z_sx <= '1';   -- line 0's y
                    when others => z_fa <= resize(z_G, 24); z_fb <= resize(s_xs, 21);        -- per pixel
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                if z_sx = '1' then f_a <= unsigned(resize(signed(z_fa), 44)); else f_a <= resize(z_fa, 44); end if;
                f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(42 downto 0) & '0';
                f_b <= '0' & f_b(20 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- [B] Drift: hold, then the speed glide (1/8 of the gap per field), Q6
                    when 33 =>
                        v_d := signed(resize(unsigned(registers_in(4)), 11)) - signed(resize(h_5, 11));
                        if unsigned(registers_in(4)) = 0 or unsigned(registers_in(4)) = 1023 or v_d > 1 or v_d < -1 then
                            h_5 <= unsigned(registers_in(4));
                        end if;
                    when 34 => s_vtg <= shift_left(resize(signed('0' & h_5) - 512, 17), 6);
                    when 35 => s_vdd <= s_vtg - s_vgs;
                    when 36 =>
                        if s_glide = '1' then s_vgs <= s_vgs + shift_right(s_vdd, 3);
                        else s_vgs <= s_vtg; end if;
                        s_g8 <= g_6(12 downto 5);
                        z_lo <= not g_p(12);
                        z_u12 <= to_unsigned(4096, 13) - g_p;
                        z_w12 <= g_p - to_unsigned(4096, 13);
                    -- position: X +-1920 virtual px (Q4: (g1 - 4096) * 7.5), Y +-1088 ((g2 - 4096) * 4.25)
                    when 37 =>
                        z_d1 <= signed(resize(g_1, 14)) - 4096;
                        z_d2 <= signed(resize(g_2, 14)) - 4096;
                    when 38 =>
                        z_xo <= shift_left(resize(z_d1, 18), 3) - shift_right(resize(z_d1, 18), 1);   -- 7.5
                        z_yo <= shift_left(resize(z_d2, 18), 2) + shift_right(resize(z_d2, 18), 2);   -- 4.25
                    when 39 =>
                        -- line 0's y: (ys / 32) - 8640, Q4 virtual px from the screen centre
                        z_v0 <= signed(resize(s_ys(11 downto 5), 18)) - 8640;
                        -- the farthest corner the screen can show: (960 + |X|, 540 + |Y|) from the figure's centre
                        if z_xo(17) = '1' then z_ax <= to_unsigned(15360, 16) + resize(unsigned(not z_xo(16 downto 0)), 16);
                        else z_ax <= to_unsigned(15360, 16) + resize(unsigned(z_xo(16 downto 0)), 16); end if;
                        if z_yo(17) = '1' then z_ay <= to_unsigned(8640, 16) + resize(unsigned(not z_yo(16 downto 0)), 16);
                        else z_ay <= to_unsigned(8640, 16) + resize(unsigned(z_yo(16 downto 0)), 16); end if;
                    when 40 =>
                        case to_integer(s_xs) is                  -- pixel 0 sits at screen column 3
                            when 4      => z_u0 <= to_signed(-11408, 18) - z_xo;
                            when 3      => z_u0 <= to_signed(-15276, 18) - z_xo;
                            when others => z_u0 <= to_signed(-15304, 18) - z_xo;
                        end case;
                        z_v0 <= z_v0 - z_yo;
                        if z_ax < z_ay then z_E <= z_ax; else z_E <= z_ay; end if;
                    -- core half sizes, Q4 virtual px (sum 291.7): (t * 73) >> 7 as shift-adds
                    when 44 => z_t <= shift_left(resize(to_unsigned(8184, 13) - g_4, 20), 6) + shift_left(resize(to_unsigned(8184, 13) - g_4, 20), 3);
                    when 45 => z_bx <= resize(shift_right(z_t + resize(to_unsigned(8184, 13) - g_4, 20), 7), 13);
                    when 46 => z_t <= shift_left(resize(g_4, 20), 6) + shift_left(resize(g_4, 20), 3);
                    when 47 => z_by <= resize(shift_right(z_t + resize(g_4, 20), 7), 13);
                    when 48 => if z_bx > z_by then z_bm <= z_bx; else z_bm <= z_by; end if;

                    -- the band period P = 12 * 2^(oct / 256): 2^-oct = 2^(-oct mod 256 / 256) >> sh
                    when 42 => z_oct <= resize(shift_right(shift_left(resize(g_3, 20), 6) + resize(g_3, 20), 9), 11);
                    when 43 =>
                        v_ne := -signed(resize(z_oct, 12));
                        r_ex_a <= unsigned(v_ne(7 downto 0));
                        if z_oct(7 downto 0) = 0 then z_sh <= resize(z_oct(10 downto 8), 3);
                        else z_sh <= resize(z_oct(10 downto 8), 3) + 1; end if;
                    when 96 =>
                        case to_integer(z_sh) is
                            when 0      => v_x := shift_right(f_acc, 10);
                            when 1      => v_x := shift_right(f_acc, 11);
                            when 2      => v_x := shift_right(f_acc, 12);
                            when 3      => v_x := shift_right(f_acc, 13);
                            when 4      => v_x := shift_right(f_acc, 14);
                            when others => v_x := shift_right(f_acc, 15);
                        end case;
                        z_H <= v_x(20 downto 0);
                    when 124 =>                       -- G = 2^I / (K8 P): the units float by octave
                        if z_oct(7 downto 0) = 0 then v_x := shift_right(f_acc, 10); else v_x := shift_right(f_acc, 11); end if;
                        z_G <= v_x(19 downto 0);
                        c_I <= z_oct(10 downto 8);
                    when 152 => c_ky <= f_acc(31 downto 12);
                    when 180 => c_w0 <= f_acc(32 downto 20);
                    -- core half sizes, Q4 virtual px (sum 291.7)
                    -- Round: e = E u below 50 % (E = the farthest visible corner: square bands out to the screen's
                    -- corner at 0 %), -3/4 max(b) w above
                    when 264 => z_elo <= f_acc(26 downto 12);
                    when 320 => z_r3 <= resize(f_acc(25 downto 13), 14) + resize(f_acc(25 downto 14), 14);   -- 3/4 max(b) w
                    when 321 =>
                        if z_lo = '1' then z_e4 <= signed(resize(z_elo, 18));
                        else z_e4 <= -signed(resize(z_r3, 18)); end if;
                    when 322 =>
                        z_bpx <= signed(resize(z_bx, 19)) + resize(z_e4, 19);
                        z_bpy <= signed(resize(z_by, 19)) + resize(z_e4, 19);
                    when 323 =>
                        if z_bpx(18) = '1' then z_bpx <= (others => '0'); end if;
                        if z_bpy(18) = '1' then z_bpy <= (others => '0'); end if;
                    when 352 => c_b1x <= f_acc(36 downto 16);
                    when 380 => c_b1y <= f_acc(36 downto 16);
                    when 408 => c_e <= f_acc(29 downto 16);
                    when 436 => c_u0 <= signed(f_acc(37 downto 8));
                    when 464 => c_v0 <= signed(f_acc(37 downto 8));
                    when 492 => c_kx <= f_acc(22 downto 5);
                    when 466 =>
                        -- the fade's mean tone (0 .. 96)
                        if s_fine = '0' then c_mean <= to_unsigned(48, 7);
                        elsif s_ink = '0' then c_mean <= to_unsigned(11, 7);
                        elsif s_pair = '1' then c_mean <= to_unsigned(27, 7);
                        else c_mean <= to_unsigned(16, 7); end if;
                    -- [B] Drift speed: |v| < 24 still; else 2^-12 .. 2^-4.1 of 4 bands per field
                    when 470 =>
                        v_v := resize(shift_right(s_vgs, 6), 11);
                        z_vn <= v_v(10);
                        v_n11 := -v_v;
                        if v_v(10) = '1' then z_av <= unsigned(v_n11(9 downto 0)); else z_av <= unsigned(v_v(9 downto 0)); end if;
                    when 471 =>
                        if z_av < 24 then z_vz <= '1'; z_av <= (others => '0');
                        else z_vz <= '0'; z_av <= z_av - 24; end if;
                    when 472 => z_ls <= z_av + shift_right(z_av, 5);
                    when 473 => r_ex_a <= z_ls(5 downto 0) & "00";
                    when 476 =>
                        if z_vz = '1' then z_spd <= (others => '0');
                        else z_spd <= resize(r_exf(15 downto 9), 14); end if;
                        z_ssh <= z_ls(8 downto 6);
                    when 477 =>
                        case to_integer(z_ssh) is
                            when 0      => null;
                            when 1      => z_spd <= shift_left(z_spd, 1);
                            when 2      => z_spd <= shift_left(z_spd, 2);
                            when 3      => z_spd <= shift_left(z_spd, 3);
                            when 4      => z_spd <= shift_left(z_spd, 4);
                            when 5      => z_spd <= shift_left(z_spd, 5);
                            when 6      => z_spd <= shift_left(z_spd, 6);
                            when others => z_spd <= shift_left(z_spd, 7);
                        end case;
                    when 478 =>
                        z_dl1 <= f_addsub(z_dl1, resize(z_spd, 18), z_vn);
                        z_sp2 <= z_spd - shift_right(z_spd, 2);
                    when 479 => z_dl2 <= f_addsub(z_dl2, resize(z_sp2, 18), not z_vn);
                    when 480 => c_k1 <= c_e - z_dl1(17 downto 4);
                    when 481 => c_k2 <= c_e - z_dl2(17 downto 4);
                    when others => null;
                end case;
            end if;

            -- [A] sync delay, generated-avid and start-pulse shift registers (field_n is tied high by the
            -- core on every input, so it only gets a register)
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 14);
        end if;
    end process p_ctrl;

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
    -- [L] LINE ENGINE: the y DDA (cordic units, Q20) and both figures' |y| - b' for the NEXT line, computed
    -- just after this line's values were taken (and once in the zone for line 0).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y : signed(21 downto 0);
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 500 then
                n_vb <= c_v0; e_st <= to_unsigned(1, 3);
            elsif s_st(0) = '1' then
                l_q1y <= n_q1y; l_q2y <= n_q2y;
                e_st <= (others => '0');
            elsif e_st /= 7 then
                e_st <= e_st + 1;
            end if;
            case to_integer(e_st) is
                when 0 => n_vb <= n_vb + signed(resize(c_ky, 30));
                when 1 =>                                     -- |y| as y xor sign (Q12)
                    v_y := n_vb(29 downto 8);
                    if v_y(21) = '1' then n_avb <= unsigned(not v_y(20 downto 0)); else n_avb <= unsigned(v_y(20 downto 0)); end if;
                when 2 =>
                    n_q1y <= signed('0' & n_avb) - signed('0' & c_b1y);
                    n_q2y <= signed('0' & n_avb) - signed('0' & c_b1x);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [P] PIXEL PATH.  One CORDIC; with Pair it computes the figure on the even pixels and the turned
    -- figure on the odd ones, and each film's phase on the other pixels is the mean of its two neighbours
    -- (the phase is smooth, so this is exact to a small fraction of a pixel).
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_x  : signed(21 downto 0);
        variable v_xs, v_xn, v_yn : signed(21 downto 0);
        variable v_ck : unsigned(13 downto 0);
        variable v_df14 : unsigned(13 downto 0);
        variable v_av : unsigned(13 downto 0);
        variable v_ph : unsigned(13 downto 0);
        variable v_f  : unsigned(11 downto 0);
        variable v_n  : unsigned(1 downto 0);
        variable v_dist : unsigned(12 downto 0);
        variable v_e  : signed(12 downto 0);
        variable v_m, v_o : unsigned(1 downto 0);
        variable v_t  : unsigned(13 downto 0);
        variable v_df : signed(2 downto 0);
        variable v_mg : unsigned(5 downto 0);
        variable v_v  : signed(7 downto 0);
        variable v_wt : signed(13 downto 0);
        variable v_s7 : unsigned(7 downto 0);
        variable v_g0, v_g1, v_g2, v_g3 : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: the x DDA (cordic units, Q20) and the pixel's parity
            if g_start = '1' then b_u <= c_u0; b_par <= '0';
            else b_u <= b_u + signed(resize(c_kx, 30)); b_par <= not b_par; end if;
            -- 2: |x| as x xor sign (Q12); Pair: the turned figure on the odd pixels
            v_x := b_u(29 downto 8);
            if v_x(21) = '1' then s2_au <= unsigned(not v_x(20 downto 0)); else s2_au <= unsigned(v_x(20 downto 0)); end if;
            s2_f <= s_pair and b_par;
            -- 3: qx = |x| - b'x, qy = |y| - b'y of this pixel's figure (the turned one swaps b'x and b'y)
            if s2_f = '1' then
                s3_qx <= signed('0' & s2_au) - signed('0' & c_b1y); s3_qy <= l_q2y;
            else
                s3_qx <= signed('0' & s2_au) - signed('0' & c_b1x); s3_qy <= l_q1y;
            end if;
            s3_f <= s2_f;
            -- 4: the CORDIC's input: (max(qx, 0), max(qy, 0)), or inside the core (not max(qx, qy), 0), negated
            if s3_qx(21) = '1' and s3_qy(21) = '1' then
                if s3_qx > s3_qy then cx(0) <= unsigned(not s3_qx(20 downto 0)); else cx(0) <= unsigned(not s3_qy(20 downto 0)); end if;
                cy(0) <= (others => '0'); cn(0) <= '1';
            else
                if s3_qx(21) = '1' then cx(0) <= (others => '0'); else cx(0) <= unsigned(s3_qx(20 downto 0)); end if;
                if s3_qy(21) = '1' then cy(0) <= (others => '0'); else cy(0) <= s3_qy; end if;
                cn(0) <= '0';
            end if;
            cf(0) <= s3_f;
            -- 5: the first iteration always adds (y >= 0); 6 .. 12: seven more (y driven to 0, x -> K8 * length)
            cx(1) <= cx(0) + unsigned(cy(0)(20 downto 0));
            cy(1) <= cy(0) - signed('0' & cx(0));
            for i in 1 to 7 loop
                v_xs := signed('0' & cx(i));
                v_xn := f_as(v_xs, shift_right(cy(i), i), cy(i)(21));
                cx(i + 1) <= unsigned(v_xn(20 downto 0));
                -- |y| shrinks with every iteration: keep the bits it can reach (measured over all inputs)
                v_yn := f_as(cy(i), shift_right(v_xs, i), not cy(i)(21));
                case i is
                    when 2      => cy(i + 1) <= resize(v_yn(20 downto 0), 22);
                    when 3      => cy(i + 1) <= resize(v_yn(19 downto 0), 22);
                    when 4      => cy(i + 1) <= resize(v_yn(18 downto 0), 22);
                    when 5 | 6  => cy(i + 1) <= resize(v_yn(17 downto 0), 22);
                    when others => cy(i + 1) <= v_yn;
                end case;
            end loop;
            for i in 0 to 7 loop cn(i + 1) <= cn(i); cf(i + 1) <= cf(i); end loop;
            -- 13: D / P in Q12 bands mod 4 (the units float by octave: shift back down by I)
            case to_integer(c_I) is
                when 0      => s13_xs <= cx(8)(13 downto 0);
                when 1      => s13_xs <= cx(8)(14 downto 1);
                when 2      => s13_xs <= cx(8)(15 downto 2);
                when 3      => s13_xs <= cx(8)(16 downto 3);
                when others => s13_xs <= cx(8)(17 downto 4);
            end case;
            s13_n <= cn(8); s13_f <= cf(8);
            -- 14: the film's phase before the camera, (e / P - drift) +- D / P
            if s13_f = '1' then v_ck := c_k2; else v_ck := c_k1; end if;
            z14 <= f_addsub(v_ck, s13_xs, s13_n); f14 <= s13_f;
            z15 <= z14; f15 <= f14;
            -- 16: the neighbours' difference (mod 4 bands) for the other film's mean
            z16 <= z15; f16 <= f15;
            s16_df <= z14 - z16; s16_a <= z16;
            -- 17: the other film = the neighbours' mean (pixel 0: the next pixel's value)
            if s_st(15) = '1' then v_av := s16_a + s16_df;
            else
                v_df14 := s16_df(13) & s16_df(13 downto 1);
                v_av := s16_a + v_df14;
            end if;
            if f16 = '1' then s17_p1 <= v_av; s17_p2 <= z16; else s17_p1 <= z16; s17_p2 <= v_av; end if;
            -- 18: + the camera; bands per pixel = the footprint + the camera's gradient
            s18_ph1 <= s17_p1 + shift_left(resize(s_Ls(11 downto 0), 14), 2);
            s18_ph2 <= s17_p2 + shift_left(resize(s_Ls(11 downto 0), 14), 2);
            v_t := resize(c_w0, 14) + resize(s_wcam, 14);
            if v_t(13) = '1' then s_wp(18) <= (others => '1'); else s_wp(18) <= v_t(12 downto 0); end if;
            for i in 19 to 28 loop s_wp(i) <= s_wp(i - 1); end loop;
            -- 19: the nearest band edge: distance, the tones either side (Fine: the line's tone and black)
            s19_wh <= shift_right(s_wp(18), 1) + shift_right(s_wp(18), 5);
            for k in 1 to 2 loop
                if k = 1 then v_ph := s18_ph1; else v_ph := s18_ph2; end if;
                v_f := v_ph(11 downto 0); v_n := v_ph(13 downto 12);
                if v_f(11) = '1' then
                    v_dist := resize(not v_f, 13); v_o := v_n + 1; v_m := v_n + 1;
                else
                    v_dist := resize(v_f, 13); v_o := v_n - 1; v_m := v_n;
                end if;
                if s_fine = '1' then v_e := to_signed(341, 13) - signed(v_dist); else v_e := signed(v_dist); end if;
                s19_e(k) <= v_e;
                if s_fine = '1' then
                    if s_ink = '1' then v_m := "11";
                    else
                        case to_integer(v_m) is
                            when 0      => v_m := "11";
                            when 2      => v_m := "01";
                            when others => v_m := "10";
                        end case;
                    end if;
                    v_o := "00";
                elsif s_ink = '1' then
                    if v_n(0) = '1' then v_m := "11"; else v_m := "00"; end if;
                    if v_o(0) = '1' then v_o := "11"; else v_o := "00"; end if;
                else
                    v_m := v_n;
                end if;
                if k = 1 then s_tn1(19) <= v_m; s_to1(19) <= v_o; else s_tn2(19) <= v_m; s_to2(19) <= v_o; end if;
            end loop;
            for i in 20 to 26 loop
                s_tn1(i) <= s_tn1(i - 1); s_to1(i) <= s_to1(i - 1); s_tn2(i) <= s_tn2(i - 1); s_to2(i) <= s_to2(i - 1);
            end loop;
            -- 20: e + w/2 + w/32
            s20_e(1) <= resize(s19_e(1), 15) + signed(resize(s19_wh, 15));
            s20_e(2) <= resize(s19_e(2), 15) + signed(resize(s19_wh, 15));
            -- 21: range tests; 22 .. 25: four quotient bits (riley's divider, one per film)
            d1z(0) <= s20_e(1)(14); d2z(0) <= s20_e(2)(14);
            if s20_e(1) >= signed(resize(s_wp(20), 15)) then d1f(0) <= '1'; else d1f(0) <= '0'; end if;
            if s20_e(2) >= signed(resize(s_wp(20), 15)) then d2f(0) <= '1'; else d2f(0) <= '0'; end if;
            d1r(0) <= unsigned(s20_e(1)(12 downto 0)); d2r(0) <= unsigned(s20_e(2)(12 downto 0));
            dvw(0) <= s_wp(20);
            for i in 1 to 4 loop
                v_t := d1r(i - 1) & '0';
                if v_t >= resize(dvw(i - 1), 14) then
                    d1r(i) <= resize(v_t - resize(dvw(i - 1), 14), 13); d1q(i) <= d1q(i - 1)(2 downto 0) & '1';
                else
                    d1r(i) <= v_t(12 downto 0); d1q(i) <= d1q(i - 1)(2 downto 0) & '0';
                end if;
                v_t := d2r(i - 1) & '0';
                if v_t >= resize(dvw(i - 1), 14) then
                    d2r(i) <= resize(v_t - resize(dvw(i - 1), 14), 13); d2q(i) <= d2q(i - 1)(2 downto 0) & '1';
                else
                    d2r(i) <= v_t(12 downto 0); d2q(i) <= d2q(i - 1)(2 downto 0) & '0';
                end if;
                dvw(i) <= dvw(i - 1);
                d1z(i) <= d1z(i - 1); d1f(i) <= d1f(i - 1); d2z(i) <= d2z(i - 1); d2f(i) <= d2f(i - 1);
            end loop;
            -- 26: coverage of the pixel's own side, 0 .. 16
            if d1z(4) = '1' then s26_cv1 <= (others => '0');
            elsif d1f(4) = '1' then s26_cv1 <= to_unsigned(16, 5);
            else s26_cv1 <= '0' & d1q(4); end if;
            if d2z(4) = '1' then s26_cv2 <= (others => '0');
            elsif d2f(4) = '1' then s26_cv2 <= to_unsigned(16, 5);
            else s26_cv2 <= '0' & d2q(4); end if;
            -- 27: V = 16 to + (tn - to) cov  (0 .. 48)
            for k in 1 to 2 loop
                if k = 1 then
                    v_df := signed('0' & s_tn1(26)) - signed('0' & s_to1(26)); v_m := s_to1(26);
                else
                    v_df := signed('0' & s_tn2(26)) - signed('0' & s_to2(26)); v_m := s_to2(26);
                end if;
                v_mg := (others => '0');
                if v_df(2) = '1' then v_df := -v_df; end if;
                if k = 1 then
                    if v_df(0) = '1' then v_mg := resize(s26_cv1, 6); end if;
                    if v_df(1) = '1' then v_mg := v_mg + (s26_cv1 & '0'); end if;
                else
                    if v_df(0) = '1' then v_mg := resize(s26_cv2, 6); end if;
                    if v_df(1) = '1' then v_mg := v_mg + (s26_cv2 & '0'); end if;
                end if;
                v_v := signed(resize(v_m & "0000", 8));
                if (k = 1 and s_tn1(26) < s_to1(26)) or (k = 2 and s_tn2(26) < s_to2(26)) then
                    v_v := v_v - signed(resize(v_mg, 8));
                else
                    v_v := v_v + signed(resize(v_mg, 8));
                end if;
                if k = 1 then s27_v1 <= unsigned(v_v(5 downto 0)); else s27_v2 <= unsigned(v_v(5 downto 0)); end if;
            end loop;
            -- 28: the two films' sum
            s28_s <= resize(s27_v1, 7) + resize(s27_v2, 7);
            s28_v1 <= s27_v1;
            -- 29: tone 0 .. 96 (single: 2 V;  Pair Grey: the films averaged;  Pair Ink: XOR as a triangle of the
            -- summed coverage); the fade weight (bands under ~2 px go to their mean)
            if s_pair = '0' then
                s29_t <= s28_v1 & '0';
            elsif s_ink = '0' then
                s29_t <= s28_s;
            elsif s28_s <= 48 then
                s29_t <= s28_s(5 downto 0) & '0';
            else
                v_s7 := to_unsigned(192, 8) - (s28_s & '0');
                s29_t <= v_s7(6 downto 0);
            end if;
            if s_fine = '1' then v_wt := signed(resize(s_wp(28), 14)) - 512; v_wt := shift_right(v_wt, 5);
            else v_wt := signed(resize(s_wp(28), 14)) - 1024; v_wt := shift_right(v_wt, 6); end if;
            if v_wt < 0 then s29_g <= (others => '0');
            elsif v_wt > 16 then s29_g <= to_unsigned(16, 5);
            else s29_g <= unsigned(v_wt(4 downto 0)); end if;
            -- 30: mean - tone
            s30_dm <= signed(resize(c_mean, 8)) - signed(resize(s29_t, 8));
            s30_t <= s29_t; s30_g <= s29_g;
            -- 31: (mean - tone) * g as gated shifts
            v_g0 := (others => '0'); v_g1 := (others => '0'); v_g2 := (others => '0'); v_g3 := (others => '0');
            if s30_g(0) = '1' then v_g0 := resize(s30_dm, 12); end if;
            if s30_g(1) = '1' then v_g1 := shift_left(resize(s30_dm, 12), 1); end if;
            if s30_g(2) = '1' then v_g2 := shift_left(resize(s30_dm, 12), 2); end if;
            if s30_g(3) = '1' then v_g3 := shift_left(resize(s30_dm, 12), 3); end if;
            s31_pr <= (v_g0 + v_g1) + (v_g2 + v_g3);
            s31_t <= s30_t; s31_g <= s30_g;
            -- 32: the faded tone
            if s31_g(4) = '1' then s32_t <= c_mean;
            else
                v_v := signed(resize(s31_t, 8)) + resize(shift_right(s31_pr, 4), 8);
                s32_t <= unsigned(v_v(6 downto 0));
            end if;
            -- 33, 34: luma = 64 + 9 t + t / 8  (white 940)
            s33_la <= shift_left(resize(s32_t, 10), 3) + resize(s32_t, 10);
            s33_lb <= to_unsigned(64, 7) + resize(s32_t(6 downto 3), 7);
            s34_y <= s33_la + resize(s33_lb, 10);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice, doubled) * K6 -> box of 128
    -- (1080) or 64 samples, edge-replicated and reloaded at every line start ->
    -- vertical one-pole in the line store (half horizontal resolution, Q13.3).
    -- The pixel path takes the one-pole's stream as it is written (the camera
    -- is never shown, so it needs no alignment): L = phase in 2^-10 band,
    -- wcam = |its x step| + |its y step| in bands per pixel.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(12 downto 0);
        variable v_pr : unsigned(16 downto 0);
        variable v_d : signed(16 downto 0);
        variable v_u : signed(16 downto 0);
        variable v_d14 : signed(13 downto 0);
        variable v_df : unsigned(12 downto 0);
        variable v_wc : unsigned(14 downto 0);
        variable v_mn : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting; both phases restart from this
            -- field's drift in the zone (riley's lattice carried the last field's x drift on line 0)
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

            -- the source sample: (luma >> 2) - 16, or the lattice at twice the level
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_l9 <= s_osum & '0';
            elsif v_l > 16 then s_l9 <= resize(v_l - 16, 9);
            else s_l9 <= (others => '0'); end if;
            s_sv <= data_in.avid;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;
            -- * K6 (a stage to itself)
            v_pr := s_l9 * s_g8;
            s_p <= v_pr(16 downto 4);
            s_sv1 <= s_sv; s_svd <= s_sv1;

            -- box: sum of the last N samples; the samples before the line start count as the first one
            if s_sv1 = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else
                bx_wp <= (others => '0');
                if s_lg7 = '1' then bx_rp <= to_unsigned(129, 8); else bx_rp <= to_unsigned(193, 8); end if;
            end if;
            if (s_lg7 = '1' and s_bcnt(7) = '0') or (s_lg7 = '0' and s_bcnt(7 downto 6) = "00") then v_tap := s_sfirst;
            else v_tap := bx_q(12 downto 0); end if;
            if s_sv1 = '1' and s_svd = '0' then
                s_sfirst <= s_p;
                if s_lg7 = '1' then s_bsum <= resize(s_p & "0000000", 20); else s_bsum <= resize(s_p & "000000", 20); end if;
                s_bcnt <= to_unsigned(1, 8);
            elsif s_sv1 = '1' then
                s_bsum <= s_bsum + s_p - v_tap;
                if (s_lg7 = '1' and s_bcnt(7) = '0') or (s_lg7 = '0' and s_bcnt(7 downto 6) = "00") then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bv <= s_sv1; s_bcol <= s_col;

            -- vertical one-pole on the odd columns: q += (box - q) >> sh
            if s_lg7 = '1' then s_eb <= s_bsum(19 downto 7); else s_eb <= s_bsum(18 downto 6); end if;
            s_eq <= lb_q;
            s_ev <= s_bv and not s_bcol(0); s_ecol <= s_bcol;
            v_d := signed(resize(s_eb & "000", 17)) - signed(resize(s_eq, 17));
            case to_integer(s_ish) is
                when 3      => v_u := shift_right(v_d, 3);
                when 4      => v_u := shift_right(v_d, 4);
                when others => v_u := shift_right(v_d, 5);
            end case;
            lb_wd <= unsigned(signed(s_eq) + v_u(15 downto 0));
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);
            if v_u(16) = '1' then s_upd <= unsigned(not v_u(12 downto 0)); else s_upd <= unsigned(v_u(12 downto 0)); end if;

            -- the stream to the pixel path: the latest and the previous column's values, their mean on the
            -- clock in between
            c_new <= lb_we;
            if lb_we = '1' then
                c_b <= c_a; c_a <= lb_wd(15 downto 3); c_dy <= s_upd;
            end if;
            v_mn := resize(c_a, 14) + resize(c_b, 14);
            if c_new = '1' then s_Ls <= v_mn(13 downto 1); else s_Ls <= c_a; end if;
            v_d14 := signed('0' & c_a) - signed('0' & c_b);
            if v_d14(13) = '1' then v_df := unsigned(not v_d14(12 downto 0)); else v_df := unsigned(v_d14(12 downto 0)); end if;
            v_wc := resize(v_df & '0', 15) + resize(c_dy(12 downto 1), 15);
            if v_wc(14 downto 13) /= "00" then s_wcam <= (others => '1'); else s_wcam <= v_wc(12 downto 0); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: black outside pixels 0 .. W + 1 (35), luma
    -- [1 2 1] / 4 (38, 39), 13/16 toward black (40), blanking gate (41).
    -- Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_gav_sr(32) = '1' or s_gav_sr(34) = '1' then s35_y <= s34_y; else s35_y <= to_unsigned(64, 10); end if;
            s36_y <= s35_y; s37_y <= s36_y;
            s38_ya <= resize(s35_y, 12) + resize(s37_y, 12) + 2;
            s38_yb <= s36_y;
            v_y := s38_ya + shift_left(resize(s38_yb, 12), 1);
            s39_y <= v_y(11 downto 2);
            if s39_y > 64 then v_dl := s39_y - 64; else v_dl := (others => '0'); end if;
            s40_y <= s39_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if s_gav_sr(C_LAT - 2) = '1' then s41_y <= s40_y; else s41_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s41_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture cartouche;
