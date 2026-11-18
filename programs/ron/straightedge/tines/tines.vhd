-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Tines v0.1.1 -- Straightedge.  A row of long needle triangles hanging from the top edge and a row standing on
-- the bottom edge, half a pitch apart, interleaving: the points of a backgammon board, saw blades, the teeth of
-- a zip, Bridget Riley's black and white triangles.  Every edge on the screen is a straight line.
--
--   cell coordinate  phi = x / P - offset - drift          (one row; a wrapping DDA, Q28 cycles)
--   tooth            eL < frac(phi) < 1 - eR,   eL = a t,  eR = (1 - a) t,   t = depth / reach
--                    (a = where the apex sits on the base: Skew; t >= 1: past the tip, nothing)
--   coverage         each edge: the signed distance in cycles -> pixels of distance (a per-field shift and a
--                    3-bit mantissa) -> a soft step F;  tooth = clamp(FL + FR + S - 1), S counting which
--                    neighbouring teeth's edges the pixel sees (sub-pixel exact tips and notches)
--   Grey             the rows are additive films (top 3/8, bottom 5/8 of white): ground, each row and the
--                    overlap are four flat tones.   Ink: the rows XOR.   Fine: each tooth as an outline drawn
--                    inside its edges (tooth - the tooth shrunk by the line width).
--   Cross            a row from the left edge and one from the right (the same teeth turned 90 degrees, reach
--                    x 16/9), crossing the vertical ones.
--   camera           the luma (x K6) is box-filtered and sampled into a grid of 32 x 32 virtual-px blocks, 2
--                    bits per block (above the light film's threshold, above the dark film's, each with a
--                    hysteresis of +-2 codes against the block's flag of the field before), written in one
--                    field and read in the next.  Each tooth edge, per block row (side rows: per block column),
--                    looks its own quarter point up: where the bit is set the edge steps out, keeping half of
--                    its inset beyond 1/16 cell (e = 1/32 + e / 2): the tooth is fatter and longer there, keeps
--                    half its taper, and a clear line of 1/8 pitch at least stays between it and its
--                    neighbour.  Black = the clean figure; mid tones = the light rows fat; highlights = both.
--
--   K1  X       slides the top (and left) row against the others, +-3/8 pitch
--   K2  Y       the balance: one pair of rows lengthens as the other shortens (x 1/8 .. 15/8)
--   K3  Scale   the pitch, 32 .. 479 virtual px (60 .. 4 teeth across), exponential
--   K4  Skew    where the apex sits on the base: 0 .. 1/2 (centre) .. 1 (right-angle saw teeth); the rows lean
--               opposite ways
--   K5  Drift   the rows slide at 1, -3/4, 21/32, -1/2 of the speed: +-24 counts still, 2^-12 .. 2^-4 pitch per
--               field, exponential
--   K6  Camera  the gain in front of the films' thresholds (default 100 %)
--   S7  Glide   Off = the knobs act this field;  On = 1/16 of the gap per field (Drift speed 1/8)
--   S8  Rows    Jaws (top and bottom) / Cross (+ left and right)
--   S9  Tone    Grey / Ink
--   S10 Weight  Bold / Fine
--   S11 Source  Video / Osc (the shell's drifting triangle lattice)
--   P12 Reach   0 % short teeth at the edges .. 50 % the tips meet along the middle .. 100 % every needle
--               reaches the opposite edge
--
-- Sections (the integer model .straightedge_work/rb_tines/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [C] FIELD ENGINE                     a microcoded accumulator machine (tines.py holds its program): the
--                                        knob hold and glide, the Drift phases, every per-field constant
--   [L] LINE ENGINE                      the rows' depth accumulators, the side rows' strip phase, the camera
--                                        anchors and their time-shared grid read, the stage-1 DDAs
--   [P] PIXEL PATH                       edges -> coverage -> tone (stages 2 .. 10)
--   [D] SOURCE FIELD                     luma (or Osc) -> 32-sample box -> vertical one-pole -> the grid
--   [F] OUTPUT CONDITIONING              luma [1 2 1] / 4, 13/16 toward black, blanking gate; U = V = 512
--
-- Stage numbering: a register "at stage S" holds pixel n at T + S + n (T = the g_start clock).  Pixels
-- 0 .. W + 1 are rendered; output column j is the [1 2 1] filter over pixels j .. j + 2 and lands on screen
-- column j + 4 (the generated avid starts 4 clocks after the delayed source avid), so pixel m sits at screen
-- column m + 3.  One latency (16) in every mode.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.tines_pkg.all;
use work.all;

architecture tines of program_top is
    constant C_LAT : integer := 16;   -- = processing_delay_clks
    constant C_CT0 : integer := 264;  -- the data ROM's first constant

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
    -- an edge's inset as a cell fraction (0 .. 65536, Q16) from its accumulator (Q5.20, signed); f = its block
    -- is over its film's threshold (the camera's step-out): an inset of 1/16 cell or more then becomes 1/32 +
    -- half of it
    function f_ins(a : signed(25 downto 0); f : std_logic) return unsigned is
        variable v : unsigned(16 downto 0);
    begin
        if a(25) = '1' then return to_unsigned(0, 17); end if;
        if f = '1' and a(24 downto 16) /= "000000000" then
            if a(24 downto 21) /= "0000" then return to_unsigned(65536, 17); end if;
            v := ('0' & unsigned(a(20 downto 5))) + 2048;
            if v(16) = '1' then return to_unsigned(65536, 17); else return v; end if;
        end if;
        if a(24 downto 20) /= "00000" then return to_unsigned(65536, 17);
        else return '0' & unsigned(a(19 downto 4)); end if;
    end function;
    -- a virtual line -> its block row: (y - 28) / 32, 0 .. 31
    function f_by(y : signed(12 downto 0)) return unsigned is
        variable v : signed(13 downto 0);
    begin
        v := resize(y, 14) - 28;
        if v(13) = '1' then return to_unsigned(0, 5);
        elsif v(12 downto 10) /= "000" then return to_unsigned(31, 5);
        else return unsigned(v(9 downto 5)); end if;
    end function;
    -- a strip anchor (Q5 virtual lines, less 28 lines: the engine takes them off) -> its block row
    function f_byq(y : signed(17 downto 0)) return unsigned is
    begin
        if y(17) = '1' then return to_unsigned(0, 5);
        elsif y(16 downto 15) /= "00" then return to_unsigned(31, 5);
        else return unsigned(y(14 downto 10)); end if;
    end function;
    -- a virtual x (Q5) -> its block column, clamped to the columns the picture has (SD 7 .. 52, else 0 .. 59)
    function f_bx(x : signed(17 downto 0); sd : std_logic) return unsigned is
        variable v : signed(7 downto 0);
    begin
        v := x(17 downto 10);
        if sd = '1' then
            if v < 7 then return to_unsigned(7, 6);
            elsif v > 52 then return to_unsigned(52, 6);
            else return unsigned(v(5 downto 0)); end if;
        else
            if v(7) = '1' then return to_unsigned(0, 6);
            elsif v > 59 then return to_unsigned(59, 6);
            else return unsigned(v(5 downto 0)); end if;
        end if;
    end function;
    -- a film's flag with hysteresis: d = luma - threshold; set at +32, cleared under -32 (2 codes of 8-bit luma)
    function f_hys(d : signed(13 downto 0); old : std_logic) return std_logic is
    begin
        if old = '1' then
            if d(13) = '0' or d(12 downto 5) = "11111111" then return '1'; else return '0'; end if;
        else
            if d(13) = '0' and d(12 downto 5) /= "00000000" then return '1'; else return '0'; end if;
        end if;
    end function;
    -- the edge distance (Q16 cycles) shifted down to its pixel window, saturated to 9 bits
    function f_win(d : signed(15 downto 0); c : unsigned(2 downto 0)) return signed is
        variable v : signed(15 downto 0);
    begin
        if c(2) = '1' then v := shift_right(d, 7);
        elsif c(1) = '1' then v := shift_right(d, 5);
        else v := shift_right(d, 3); end if;
        if c(0) = '1' then v := shift_right(v, 1); end if;
        if v(15) = '0' and v(14 downto 8) /= "0000000" then return to_signed(255, 9);
        elsif v(15) = '1' and v(14 downto 8) /= "1111111" then return to_signed(-256, 9);
        else return v(8 downto 0); end if;
    end function;
    -- (y * (4 + k)) >> 3, k = 0 .. 4: sixteenths of a pixel
    function f_mul(y : signed(8 downto 0); k : unsigned(2 downto 0)) return signed is
        variable v_a, v_b, v_c, v_p : signed(11 downto 0);
    begin
        if k(2) = '1' then v_a := shift_left(resize(y, 12), 3); else v_a := shift_left(resize(y, 12), 2); end if;
        if k(1) = '1' then v_b := shift_left(resize(y, 12), 1); else v_b := (others => '0'); end if;
        if k(0) = '1' then v_c := resize(y, 12); else v_c := (others => '0'); end if;
        v_p := v_a + v_b + v_c;
        return v_p(11 downto 3);
    end function;
    -- the soft step: clamp(z + o, 0, 16)  (o = 8: half covered on the edge)
    function f_step(z : signed(8 downto 0); o : signed(8 downto 0)) return unsigned is
        variable v : signed(9 downto 0);
    begin
        v := resize(z, 10) + resize(o, 10);
        if v(9) = '1' then return to_unsigned(0, 5);
        elsif v > 16 then return to_unsigned(16, 5);
        else return unsigned(v(4 downto 0)); end if;
    end function;
    -- the right edge runs on the NEGATED distance: clamp(o - z, 0, 16) (so a floor on one edge is a ceiling on
    -- the other: two rows that tile the screen add up to exactly 16 on their shared edge)
    function f_stepn(z : signed(8 downto 0); o : signed(8 downto 0)) return unsigned is
        variable v : signed(9 downto 0);
    begin
        v := resize(o, 10) - resize(z, 10);
        if v(9) = '1' then return to_unsigned(0, 5);
        elsif v > 16 then return to_unsigned(16, 5);
        else return unsigned(v(4 downto 0)); end if;
    end function;
    -- a tooth from its two edges' steps and the neighbour count
    function f_tooth(a, b : unsigned(4 downto 0); s : signed(2 downto 0)) return unsigned is
        variable v : unsigned(5 downto 0);
    begin
        v := resize(a, 6) + resize(b, 6);
        if s(2) = '1' then return to_unsigned(0, 5);
        elsif s = 0 then
            if v(5) = '1' or v(4) = '1' then return resize(v - 16, 5); else return to_unsigned(0, 5); end if;
        elsif s = 1 then
            if v > 16 then return to_unsigned(16, 5); else return v(4 downto 0); end if;
        else return to_unsigned(16, 5); end if;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ysr  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_mi   : unsigned(1 downto 0) := (others => '0');             -- 0 1080-line, 1 720p, 2 SD
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo, q_go : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_x0h  : unsigned(12 downto 0);                                -- pixel 0's x (h)
    signal s_b0   : unsigned(5 downto 0);                                 -- its block column
    signal s_xi   : unsigned(12 downto 0);                                -- the source stream's first x (h)
    signal s_glide : std_logic := '1';
    signal s_cross, s_ink, s_fine, s_src : std_logic := '0';
    signal s_bank : std_logic := '0';
    signal s_ish  : unsigned(2 downto 0) := to_unsigned(4, 3);

    -- [A] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_avd : std_logic := '0';
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

    -- [C] field engine
    type t_eng is (E_IDLE, E_W1, E_DEC, E_M1, E_M2, E_EX, E_EX2, E_SH, E_MUL, E_NORM, E_DIV);
    signal e_s    : t_eng := E_IDLE;
    signal e_pc   : unsigned(9 downto 0) := (others => '0');
    signal pg_a   : unsigned(9 downto 0) := (others => '0');
    signal pg_q   : unsigned(15 downto 0) := (others => '0');
    signal e_op   : unsigned(4 downto 0) := (others => '0');
    signal e_a    : unsigned(7 downto 0) := (others => '0');
    signal e_x    : unsigned(2 downto 0) := (others => '0');
    signal e_acc  : signed(31 downto 0) := (others => '0');
    signal e_cnt  : unsigned(5 downto 0) := (others => '0');
    signal e_tk   : unsigned(2 downto 0) := (others => '0');
    signal e_f    : std_logic := '0';
    signal ex_x, ex_y : signed(31 downto 0) := (others => '0');
    signal ex_c, ex_s : std_logic := '0';
    signal e_sl   : std_logic := '0';
    signal rf_ram : t_tn_rf := C_TN_RFINIT;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_ra, rf_wa : unsigned(7 downto 0) := (others => '0');
    signal rf_q, rf_d : signed(31 downto 0) := (others => '0');
    signal rf_we  : std_logic := '0';
    signal dr_a   : unsigned(8 downto 0) := (others => '0');
    signal dr_q, dr_d : unsigned(15 downto 0) := (others => '0');
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_p    : unsigned(48 downto 0) := (others => '0');
    signal m_sgn  : std_logic := '0';
    signal d_r    : unsigned(26 downto 0) := (others => '0');
    signal e_oe, e_done : std_logic := '0';
    signal e_oa   : unsigned(5 downto 0) := (others => '0');

    -- [C] per-field constants (the engine's outputs; the line engine's own constants stay in the register
    -- file, words 33 .. 45, and are read after every line)
    type t_u28x2 is array (0 to 1) of unsigned(27 downto 0);
    type t_u18x2 is array (0 to 1) of unsigned(17 downto 0);
    type t_s26x2 is array (0 to 1) of signed(25 downto 0);
    type t_s18x2 is array (0 to 1) of signed(17 downto 0);
    type t_u12x4 is array (0 to 3) of unsigned(11 downto 0);
    signal c_dphi : unsigned(27 downto 0) := to_unsigned(2316576, 28);
    signal c_dl, c_dr : t_u18x2 := (others => (others => '0'));            -- left / right: insets per pixel
    signal c_pq   : unsigned(13 downto 0) := to_unsigned(5562, 14);        -- the pitch, Q5 virtual px
    signal c_ec   : t_u12x4 := (others => (others => '0'));                -- edge normalisers (T, B, L, R)
    signal c_t16  : signed(8 downto 0) := (others => '0');                 -- 8 - the Fine line width
    signal c_thb, c_tht : unsigned(12 downto 0) := (others => '1');       -- the films' thresholds (Q13 luma)

    -- [L] line engine
    signal e_lc   : unsigned(5 downto 0) := (others => '1');
    signal h_run  : std_logic := '0';
    signal rf_rx  : unsigned(7 downto 0);
    signal l_al, l_ar : t_s26x2 := (others => (others => '0'));            -- top / bottom insets (Q5.20)
    signal l_ps   : t_u28x2 := (others => (others => '0'));                -- left / right strip phase
    signal l_wr   : std_logic_vector(0 to 1) := "00";
    signal l_yc   : t_s18x2 := (others => (others => '0'));                -- left / right strip anchor (Q5) - 28 lines
    signal l_yq   : unsigned(19 downto 0) := (others => '0');              -- the line's y, Q8
    type t_u17x2 is array (0 to 1) of unsigned(16 downto 0);
    type t_u5x2  is array (0 to 1) of unsigned(4 downto 0);
    type t_u6x2  is array (0 to 1) of unsigned(5 downto 0);
    signal l_by   : unsigned(4 downto 0) := (others => '0');
    signal l_byL, l_byR : t_u5x2 := (others => (others => '0'));
    -- camera anchors, the time-shared grid read, the element flags
    signal a_xc   : t_s18x2 := (others => (others => '0'));
    signal a_bxL, a_bxR : t_u6x2 := (others => (others => '0'));
    signal a_bs   : unsigned(6 downto 0) := (others => '0');
    signal a_bsc  : unsigned(5 downto 0) := (others => '0');
    signal t_q, t_q1, t_q2 : unsigned(2 downto 0) := (others => '0');
    signal p_f, c_f : std_logic_vector(0 to 7) := (others => '0');         -- pending / current: T L R, B L R, L, R
    type t_grid is array (0 to 4095) of std_logic_vector(1 downto 0);
    signal g_ram  : t_grid := (others => "00");
    attribute no_rw_check of g_ram : signal is true;
    signal g_ra   : unsigned(11 downto 0) := (others => '0');
    signal g_rq, g_wd : std_logic_vector(1 downto 0) := "00";
    signal g_we   : std_logic := '0';
    -- a second copy of the grid, read at the block being written: its flags of the field before (hysteresis)
    signal h_ram  : t_grid := (others => "00");
    attribute no_rw_check of h_ram : signal is true;
    signal h_ra   : unsigned(11 downto 0);
    signal h_rq, h_rf : std_logic_vector(1 downto 0) := "00";
    signal k_v    : unsigned(12 downto 0) := (others => '0');
    signal k_wa   : unsigned(11 downto 0) := (others => '0');
    signal k_h    : std_logic_vector(2 downto 0) := "000";
    -- stage 1
    signal b_ph   : t_u28x2 := (others => (others => '0'));
    signal h_aL, h_aR : t_s26x2 := (others => (others => '0'));
    signal b_xh   : unsigned(12 downto 0) := (others => '0');

    -- [P] pixel path
    type t_s18x4 is array (0 to 3) of signed(17 downto 0);
    type t_s9x4  is array (0 to 3) of signed(8 downto 0);
    type t_s3x4  is array (0 to 3) of signed(2 downto 0);
    type t_u5x4  is array (0 to 3) of unsigned(4 downto 0);
    signal q_dL, q_dR : t_s18x4 := (others => (others => '0'));
    signal q_gone : std_logic_vector(0 to 3) := (others => '0');
    signal p_yL, p_yR, z_L, z_R : t_s9x4 := (others => (others => '0'));
    signal p_S, z_S, f_S : t_s3x4 := (others => (others => '0'));
    signal f_1L, f_1R, f_2L, f_2R, s6_cv : t_u5x4 := (others => (others => '0'));
    signal s7_sa, s7_sb : unsigned(8 downto 0) := (others => '0');
    signal s7_qa, s7_qb : unsigned(5 downto 0) := (others => '0');
    signal s8_s   : unsigned(8 downto 0) := (others => '0');
    signal s9_a   : unsigned(9 downto 0) := (others => '0');
    signal s9_b   : unsigned(14 downto 0) := (others => '0');
    signal s10_y  : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_l9  : unsigned(8 downto 0) := (others => '0');
    signal s_p, s_sfirst : unsigned(8 downto 0) := (others => '0');
    signal s_sv, s_sv1, s_svd, s_bv, s_ev : std_logic := '0';
    signal s_col, s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    type t_bx is array (0 to 255) of unsigned(15 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(15 downto 0) := (others => '0');
    signal s_bcnt : unsigned(5 downto 0) := (others => '0');
    signal s_bsum : unsigned(13 downto 0) := (others => '0');
    signal s_eb  : unsigned(12 downto 0) := (others => '0');
    signal s_eq  : unsigned(15 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal s_sy  : unsigned(19 downto 0) := (others => '0');               -- the source line's y, Q8
    signal g_sx  : unsigned(12 downto 0) := (others => '0');               -- the stream sample's x (h)
    signal g_cl  : std_logic := '0';
    signal g_row : unsigned(4 downto 0) := (others => '0');

    -- [F] output conditioning
    signal s11_y, s12_y, s13_y, s14_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s14_ya : unsigned(11 downto 0) := (others => '0');
    signal s15_y, s16_y, s17_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            pg_q <= C_TN_PROG(to_integer(pg_a));
            dr_q <= C_TN_ROM(to_integer(dr_a));
        end if;
    end process p_rom;

    p_rf : process(clk)
    begin
        if rising_edge(clk) then
            rf_q <= rf_ram(to_integer(rf_rx));
            if rf_we = '1' then rf_ram(to_integer(rf_wa)) <= e_acc; end if;   -- (ST's EX is followed by DEC: acc holds)
        end if;
    end process p_rf;

    -- by video mode: pixel 0's x and block column, the source stream's first x (SD: the centre 1440 virtual px)
    s_x0h <= to_unsigned(494, 13) when s_mi = "10" else to_unsigned(10, 13) when s_mi = "01" else to_unsigned(7, 13);
    s_xi  <= to_unsigned(486, 13) when s_mi = "10" else to_unsigned(4, 13) when s_mi = "01" else to_unsigned(3, 13);
    s_b0  <= to_unsigned(7, 6) when s_mi = "10" else to_unsigned(0, 6);

    -- the register file's read port: the field engine, or the line engine while the engine is idle
    rf_rx <= ("001" & e_lc(4 downto 0)) when e_s = E_IDLE else rf_ra;

    -- the camera grid: two banks ping-ponged per field (the bank being read is never the one being written)
    p_grid : process(clk)
    begin
        if rising_edge(clk) then
            g_rq <= g_ram(to_integer(g_ra));
            if g_we = '1' then g_ram(to_integer(k_wa)) <= g_wd; end if;
        end if;
    end process p_grid;

    h_ra <= (not k_wa(11)) & k_wa(10 downto 0);
    p_hgrid : process(clk)
    begin
        if rising_edge(clk) then
            h_rq <= h_ram(to_integer(h_ra));
            if g_we = '1' then h_ram(to_integer(k_wa)) <= g_wd; end if;
        end if;
    end process p_hgrid;

    -- box delay: read 32 samples behind the write
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
    -- [A] TIMING AND VIRTUAL COORDINATES
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
    begin
        if rising_edge(clk) then
            -- timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ysr <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);

            -- vblank zone: no avid for 1024 clocks (hblank is 138 .. 830: 1080p24)
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;
            q_go <= s_zgo;                      -- the field engine starts one clock after the latches

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- per-field latches; the grid's banks swap
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_glide <= registers_in(6)(0);
                s_cross <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_ys    <= s_ysr;
                s_bank  <= not s_bank;
                case to_integer(s_xsr) is
                    when 4 =>
                        s_oxs <= to_unsigned(204, 9); s_mi <= "10";
                    when 3 =>
                        s_oxs <= to_unsigned(153, 9); s_mi <= "01";
                    when others =>
                        s_oxs <= to_unsigned(102, 9); s_mi <= "00";
                end case;
                if s_ysr <= 256 then s_ish <= to_unsigned(4, 3);
                elsif s_ysr <= 512 then s_ish <= to_unsigned(3, 3);
                else s_ish <= to_unsigned(2, 3); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- sync delay and generated-avid shift registers (field_n is tied high by the core on every input,
            -- so it only gets a register)
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
    -- [C] FIELD ENGINE.  One instruction = DEC (the word is in pg_q: latch it,
    -- issue the register-file / ROM read, prefetch the next word) [, M1 (block
    -- RAM read), M2 (fabric copy)], EX (one adder; shifts 4 bits a clock,
    -- multiplies 16 steps, divides 45).  Ops and program: tines.py.  It starts
    -- at every field start from word 0, whatever state it was in.
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_op  : unsigned(4 downto 0);
        variable v_a   : unsigned(7 downto 0);
        variable v_sel : unsigned(1 downto 0);
        variable v_b   : signed(31 downto 0);
        variable v_x, v_y : signed(31 downto 0);
        variable v_s33 : signed(32 downto 0);
        variable v_sum : signed(31 downto 0);
        variable v_mem : boolean;
        variable v_nx  : unsigned(9 downto 0);
        variable v_t   : unsigned(26 downto 0);
    begin
        if rising_edge(clk) then
            rf_we <= '0';
            e_oe <= '0';
            e_done <= '0';
            if e_op = 1 or e_op = 7 or e_op = 17 or e_op = 27 then v_b := resize(signed(dr_d), 32);   -- CT: signed
            elsif e_op = 16 then v_b := signed(resize(dr_d, 32));                          -- EXR: unsigned
            else v_b := rf_d; end if;
            v_s33 := (ex_x & '1') + (ex_y & ex_c);       -- one carry chain; the low column makes the carry-in
            v_sum := v_s33(32 downto 1);

            case e_s is
                when E_IDLE => rf_d <= rf_q;          -- the line engine's reads (a fabric copy)
                when E_W1 =>
                    e_s <= E_DEC;
                when E_DEC =>
                    v_op := pg_q(15 downto 11); v_a := pg_q(7 downto 0);
                    case to_integer(pg_q(10 downto 8)) is
                        when 1      => v_sel := s_mi;
                        when 2      => v_sel := '0' & s_glide;
                        when others => v_sel := "00";
                    end case;
                    e_op <= v_op; e_a <= v_a; e_x <= pg_q(10 downto 8);
                    -- operands of the ops that go straight to EX: MRD (+- the product, shifted), ABS (- acc)
                    ex_y <= (others => '0');
                    case to_integer(pg_q(9 downto 8)) is
                        when 1      => v_x := signed(m_p(35 downto 4));
                        when 2      => v_x := signed(m_p(43 downto 12));
                        when others => v_x := signed(m_p(31 downto 0));
                    end case;
                    if v_op = 20 then
                        ex_x <= not e_acc; ex_c <= '1';
                    elsif m_sgn = '1' then
                        ex_x <= not v_x; ex_c <= '1';
                    else
                        ex_x <= v_x; ex_c <= '0';
                    end if;
                    if (v_op = 2 or v_op = 3) and pg_q(8) = '1' then     -- LD / ST, x = 1: the knob bank
                        rf_ra <= '1' & e_tk & v_a(3 downto 0);
                    else
                        rf_ra <= v_a;
                    end if;
                    if v_op = 16 then dr_a <= unsigned(e_acc(8 downto 0));
                    else dr_a <= to_unsigned(C_CT0, 9) + resize(v_a, 9) + resize(v_sel, 9); end if;
                    -- next word
                    v_nx := e_pc + 1;
                    if v_op = 23 and e_tk /= 0 then v_nx := pg_q(9 downto 8) & v_a; e_tk <= e_tk - 1; end if;
                    if v_op = 28 then e_tk <= v_a(2 downto 0); end if;                       -- SETK
                    e_pc <= v_nx; pg_a <= v_nx;
                    case to_integer(v_op) is
                        when 1 | 2 | 4 | 5 | 6 | 7 | 11 | 12 | 13 | 14 | 16 | 17 | 18 | 19 | 24 | 26 | 27 => v_mem := true;
                        when others => v_mem := false;
                    end case;
                    if v_op = 0 then e_s <= E_IDLE; e_done <= '1';
                    elsif v_op = 23 then e_s <= E_W1;
                    elsif v_mem then e_s <= E_M1;
                    else e_s <= E_EX; end if;
                when E_M1 =>
                    e_s <= E_M2;
                when E_M2 =>
                    rf_d <= rf_q; dr_d <= dr_q;
                    if e_op = 1 or e_op = 7 or e_op = 17 or e_op = 27 then v_y := resize(signed(dr_q), 32);
                    else v_y := rf_q; end if;
                    case to_integer(e_op) is
                        when 5 | 17 | 18 | 19 | 27 => ex_x <= e_acc; ex_y <= not v_y; ex_c <= '1';   -- SUB MAX* MIN*
                        when 6                     => ex_x <= not e_acc; ex_y <= v_y; ex_c <= '1';   -- RSUB
                        when 13 | 14               => ex_x <= not e_acc; ex_y <= (others => '0'); ex_c <= '1';   -- |acc|
                        when others                => ex_x <= e_acc; ex_y <= v_y; ex_c <= '0';       -- ADD ADDC
                    end case;
                    e_s <= E_EX;
                when E_EX =>
                    e_s <= E_DEC;
                    case to_integer(e_op) is
                        when 1 | 2 | 16 => e_acc <= v_b;                                          -- LDC LD EXR
                        when 4 | 5 | 6 | 7 | 15 => e_acc <= v_sum;                                -- ADD SUB RSUB ADDC MRD
                        when 17 | 18 | 19 | 27 =>                                                  -- MAXC MAXR MINR MINC:
                            ex_s <= v_sum(31); e_s <= E_EX2;                                           -- select next clock
                        when 26 => if e_f = '1' then e_acc <= v_b; end if;                        -- LDF
                        when 25 =>                                                                 -- TST
                            case to_integer(e_x) is
                                when 0      => e_f <= not e_acc(31);
                                when 1      => e_f <= e_f or e_acc(31);
                                when others => e_f <= e_f and not e_acc(31);
                            end case;
                        when 20 => if e_acc(31) = '1' then e_acc <= v_sum; end if;                -- ABS
                        when 3 =>                                                                  -- ST
                            rf_we <= '1';
                            if e_x(0) = '1' then rf_wa <= '1' & e_tk & e_a(3 downto 0);
                            else rf_wa <= e_a; end if;
                        when 8 =>                                                                  -- KNB
                            if e_a(3) = '0' then e_acc <= signed(resize(s_ys, 32));
                            else
                                case to_integer(e_tk) is                                           -- loop channel's knob
                                    when 6      => e_acc <= signed(resize(unsigned(registers_in(4)), 32));
                                    when 5      => e_acc <= signed(resize(unsigned(registers_in(0)), 32));
                                    when 4      => e_acc <= signed(resize(unsigned(registers_in(1)), 32));
                                    when 3      => e_acc <= signed(resize(unsigned(registers_in(2)), 32));
                                    when 2      => e_acc <= signed(resize(unsigned(registers_in(3)), 32));
                                    when 1      => e_acc <= signed(resize(unsigned(registers_in(5)), 32));
                                    when others => e_acc <= signed(resize(unsigned(registers_in(7)), 32));
                                end case;
                            end if;
                        when 9 | 10 =>                                                             -- SRA SHL
                            e_cnt <= '0' & e_a(4 downto 0);
                            if e_op = 10 then e_sl <= '1'; else e_sl <= '0'; end if;
                            if e_a(4 downto 0) /= 0 then e_s <= E_SH; end if;
                        when 11 | 12 =>                                                            -- SRAV SHLV
                            e_cnt <= '0' & unsigned(rf_d(4 downto 0));
                            if e_op = 12 then e_sl <= '1'; else e_sl <= '0'; end if;
                            if rf_d(4 downto 0) /= "00000" then e_s <= E_SH; end if;
                        when 13 | 14 =>                                                            -- MULR MULC
                            if e_acc(31) = '1' then m_a <= unsigned(v_sum); else m_a <= unsigned(e_acc); end if;
                            if e_op = 13 then m_p <= resize(unsigned(rf_d(15 downto 0)), 49);
                            else m_p <= resize(dr_d, 49); end if;
                            m_sgn <= e_acc(31);
                            e_cnt <= to_unsigned(16, 6);
                            e_s <= E_MUL;
                        when 21 =>                                                                 -- NORM
                            e_cnt <= to_unsigned(30, 6);
                            e_s <= E_NORM;
                        when 24 =>                                                                 -- DIV
                            m_a <= unsigned(rf_d);
                            d_r <= (others => '0');
                            e_cnt <= to_unsigned(45, 6);
                            e_s <= E_DIV;
                        when 22 =>                                                                 -- OUT
                            e_oe <= '1'; e_oa <= e_a(5 downto 0);
                        when others => null;
                    end case;
                when E_EX2 =>                                                              -- the conditional select
                    e_s <= E_DEC;
                    if e_op = 17 or e_op = 18 then
                        if ex_s = '1' then e_acc <= v_b; end if;                                  -- MAXC MAXR
                    else
                        if ex_s = '0' then e_acc <= v_b; end if;                                  -- MINC MINR
                    end if;
                when E_SH =>
                    if e_cnt >= 4 then
                        if e_sl = '1' then e_acc <= shift_left(e_acc, 4); else e_acc <= shift_right(e_acc, 4); end if;
                        e_cnt <= e_cnt - 4;
                        if e_cnt = 4 then e_s <= E_DEC; end if;
                    else
                        if e_sl = '1' then e_acc <= shift_left(e_acc, 1); else e_acc <= shift_right(e_acc, 1); end if;
                        e_cnt <= e_cnt - 1;
                        if e_cnt = 1 then e_s <= E_DEC; end if;
                    end if;
                when E_MUL =>
                    e_cnt <= e_cnt - 1;
                    if e_cnt = 1 then e_s <= E_DEC; end if;
                when E_NORM =>                    -- q = the top bit's index, f = the next 3 bits: acc = 8 q + f
                    if e_acc(30) = '1' then
                        e_acc <= signed(resize(e_cnt & unsigned(e_acc(29 downto 27)), 32));
                        e_s <= E_DEC;
                    elsif e_cnt = 0 then
                        e_acc <= (others => '0');
                        e_s <= E_DEC;
                    else
                        e_acc <= shift_left(e_acc, 1); e_cnt <= e_cnt - 1;
                    end if;
                when E_DIV =>                     -- restoring: [d_r | acc] shifts left, the quotient enters acc
                    v_t := d_r(25 downto 0) & e_acc(31);
                    if v_t >= m_a(26 downto 0) then
                        d_r <= v_t - m_a(26 downto 0);
                        e_acc <= e_acc(30 downto 0) & '1';
                    else
                        d_r <= v_t;
                        e_acc <= e_acc(30 downto 0) & '0';
                    end if;
                    e_cnt <= e_cnt - 1;
                    if e_cnt = 1 then e_s <= E_DEC; end if;
            end case;

            -- the serial multiplier: m_p = [partial product | multiplier], add at bit 16, shift right (16 steps)
            if e_s = E_MUL then
                if m_p(0) = '1' then
                    m_p <= '0' & (m_p(48 downto 16) + resize(m_a, 33)) & m_p(15 downto 1);
                else
                    m_p <= '0' & m_p(48 downto 1);
                end if;
            end if;

            -- the field start (also the watchdog: a stranded engine restarts here)
            if q_go = '1' then
                e_pc <= (others => '0'); pg_a <= (others => '0'); e_tk <= (others => '0'); e_s <= E_W1;
                rf_we <= '0'; e_oe <= '0'; e_done <= '0';
            end if;
        end if;
    end process p_eng;

    -- OUT: the per-field constants (the line engine takes its own start values in p_line)
    p_outs : process(clk)
    begin
        if rising_edge(clk) then
            if e_oe = '1' then
                case to_integer(e_oa) is
                    when C_TN_O_DPHI => c_dphi <= unsigned(e_acc(27 downto 0));
                    when C_TN_O_DLL  => c_dl(0) <= unsigned(e_acc(17 downto 0));
                    when C_TN_O_DRL  => c_dr(0) <= unsigned(e_acc(17 downto 0));
                    when C_TN_O_DLR  => c_dl(1) <= unsigned(e_acc(17 downto 0));
                    when C_TN_O_DRR  => c_dr(1) <= unsigned(e_acc(17 downto 0));
                    when C_TN_O_PQ5  => c_pq <= unsigned(e_acc(13 downto 0));
                    when C_TN_O_ECT  => c_ec(0) <= unsigned(e_acc(11 downto 0));
                    when C_TN_O_ECB  => c_ec(1) <= unsigned(e_acc(11 downto 0));
                    when C_TN_O_ECL  => c_ec(2) <= unsigned(e_acc(11 downto 0));
                    when C_TN_O_ECR  => c_ec(3) <= unsigned(e_acc(11 downto 0));
                    when C_TN_O_THP  => c_t16 <= to_signed(8, 9) - signed(resize(unsigned(e_acc(7 downto 0)), 9));
                    when C_TN_O_THB  => c_thb <= unsigned(e_acc(12 downto 0));
                    when C_TN_O_THT  => c_tht <= unsigned(e_acc(12 downto 0));
                    when others => null;
                end case;
            end if;
        end if;
    end process p_outs;

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
    -- [L] LINE ENGINE.  After each line (and after the engine's END, for line
    -- 0) a 31-clock pass reads its 13 constants from the register file (word
    -- 32 + step, used two clocks later):
    --    3 .. 6   the top / bottom rows' insets step one line
    --    7        the side rows' strip phase steps (a wrap moves the strip anchor one pitch: 8); the line's y
    --    8, 9     the top / bottom cell phases of pixel 0; the block rows
    --   10 .. 13  the side rows' insets at pixel 0
    --   14, 15    the top / bottom anchors go back to the first cell, the side anchor to the first block
    --   30        the element flags of the first cell / block are taken, the anchors move to the next one
    -- The grid is read on one port, one anchor a clock (eight slots): an anchor's flag is pending within 12
    -- clocks of its move, and the shortest tooth or block is 16 pixels.
    -- Stage 1 (running from g_start): the cell-phase DDAs (top / bottom), the inset DDAs (left / right), the
    -- pixel's x; a cell wrap or a block border takes the pending flags and moves the anchor on.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_ph : unsigned(28 downto 0);
        variable v_xh : unsigned(12 downto 0);
        variable v_yc : signed(17 downto 0);
        variable v_xc : signed(17 downto 0);
        variable v_pq : signed(17 downto 0);
        variable v_ph2 : signed(17 downto 0);
    begin
        if rising_edge(clk) then
            v_pq := signed(resize(c_pq, 18));
            v_ph2 := signed(resize(c_pq(13 downto 1), 18));
            g_avd <= g_avid;
            if e_done = '1' or (g_avd = '1' and g_avid = '0') then e_lc <= (others => '0');
            elsif e_lc /= 63 then e_lc <= e_lc + 1; end if;
            if g_start = '1' then h_run <= '1'; elsif e_lc = 0 then h_run <= '0'; end if;

            -- stage 1
            if h_run = '1' then
                for v in 0 to 1 loop
                    v_ph := ('0' & b_ph(v)) + ('0' & c_dphi);
                    b_ph(v) <= v_ph(27 downto 0);
                    if v_ph(28) = '1' then
                        c_f(2 * v) <= p_f(2 * v); c_f(2 * v + 1) <= p_f(2 * v + 1);
                        a_xc(v) <= a_xc(v) + v_pq;
                    end if;
                end loop;
                h_aL(0) <= h_aL(0) + signed(resize(c_dl(0), 26)); h_aR(0) <= h_aR(0) + signed(resize(c_dr(0), 26));
                h_aL(1) <= h_aL(1) - signed(resize(c_dl(1), 26)); h_aR(1) <= h_aR(1) - signed(resize(c_dr(1), 26));
                v_xh := b_xh + resize(s_xs, 13);
                b_xh <= v_xh;
                if v_xh(6) /= b_xh(6) then
                    c_f(4 to 7) <= p_f(4 to 7);
                    a_bs <= a_bs + 1;
                end if;
            end if;

            -- the line pass
            case to_integer(e_lc) is
                when 3 => l_al(0) <= l_al(0) + rf_d(25 downto 0);
                when 4 => l_ar(0) <= l_ar(0) + rf_d(25 downto 0);
                when 5 => l_al(1) <= l_al(1) - rf_d(25 downto 0);
                when 6 => l_ar(1) <= l_ar(1) - rf_d(25 downto 0);
                when 7 =>
                    for s in 0 to 1 loop
                        v_ph := ('0' & l_ps(s)) + ('0' & unsigned(rf_d(27 downto 0)));
                        l_ps(s) <= v_ph(27 downto 0); l_wr(s) <= v_ph(28);
                    end loop;
                    l_yq <= l_yq + resize(s_ys, 20);
                when 8 =>
                    b_ph(0) <= unsigned(rf_d(27 downto 0));
                    for s in 0 to 1 loop
                        if l_wr(s) = '1' then l_yc(s) <= l_yc(s) + v_pq; end if;
                    end loop;
                when 9 =>
                    b_ph(1) <= unsigned(rf_d(27 downto 0));
                    l_by <= f_by(signed(resize(l_yq(19 downto 8), 13)));
                    for s in 0 to 1 loop
                        l_byL(s) <= f_byq(l_yc(s));
                        v_yc := l_yc(s) + v_ph2;
                        l_byR(s) <= f_byq(v_yc);
                    end loop;
                when 10 =>
                    h_aL(0) <= rf_d(25 downto 0);
                when 11 => h_aR(0) <= rf_d(25 downto 0);
                when 12 => h_aL(1) <= rf_d(25 downto 0);
                when 13 => h_aR(1) <= rf_d(25 downto 0);
                when 14 =>
                    a_xc(0) <= rf_d(17 downto 0);
                    a_bs <= '0' & s_b0;
                    b_xh <= s_x0h;
                when 15 => a_xc(1) <= rf_d(17 downto 0);
                when 30 =>
                    c_f <= p_f;
                    a_xc(0) <= a_xc(0) + v_pq; a_xc(1) <= a_xc(1) + v_pq;
                    a_bs <= ('0' & s_b0) + 1;
                when others => null;
            end case;

            -- the engine's start values: the accumulators one line before line 0
            if e_oe = '1' then
                case to_integer(e_oa) is
                    when C_TN_O_ALT0P => l_al(0) <= e_acc(25 downto 0);
                    when C_TN_O_ART0P => l_ar(0) <= e_acc(25 downto 0);
                    when C_TN_O_ALB0P => l_al(1) <= e_acc(25 downto 0);
                    when C_TN_O_ARB0P => l_ar(1) <= e_acc(25 downto 0);
                    when C_TN_O_PSL0P => l_ps(0) <= unsigned(e_acc(27 downto 0));
                    when C_TN_O_PSR0P => l_ps(1) <= unsigned(e_acc(27 downto 0));
                    when C_TN_O_YCL0P => l_yc(0) <= e_acc(17 downto 0);
                    when C_TN_O_YCR0P => l_yc(1) <= e_acc(17 downto 0);
                    when others => null;
                end case;
            end if;
            if s_zgo = '1' then l_yq <= resize(s_ysr(11 downto 1), 20) - resize(s_ysr, 20); end if;

            -- the anchors' block columns (registered), the grid read in eight slots
            for v in 0 to 1 loop
                a_bxL(v) <= f_bx(a_xc(v), s_mi(1));
                v_xc := a_xc(v) + v_ph2;
                a_bxR(v) <= f_bx(v_xc, s_mi(1));
            end loop;
            if s_mi(1) = '1' then
                if a_bs > 52 then a_bsc <= to_unsigned(52, 6); else a_bsc <= a_bs(5 downto 0); end if;
            else
                if a_bs > 59 then a_bsc <= to_unsigned(59, 6); else a_bsc <= a_bs(5 downto 0); end if;
            end if;
            t_q <= t_q + 1; t_q1 <= t_q; t_q2 <= t_q1;
            case to_integer(t_q) is
                when 0      => g_ra <= (not s_bank) & l_by & a_bxL(0);
                when 1      => g_ra <= (not s_bank) & l_by & a_bxR(0);
                when 2      => g_ra <= (not s_bank) & l_by & a_bxL(1);
                when 3      => g_ra <= (not s_bank) & l_by & a_bxR(1);
                when 4      => g_ra <= (not s_bank) & l_byL(0) & a_bsc;
                when 5      => g_ra <= (not s_bank) & l_byR(0) & a_bsc;
                when 6      => g_ra <= (not s_bank) & l_byL(1) & a_bsc;
                when others => g_ra <= (not s_bank) & l_byR(1) & a_bsc;
            end case;
            -- the top and left rows are the dark film (bit 1), the bottom and right rows the light one (bit 0)
            if t_q2(1) = '0' then p_f(to_integer(t_q2)) <= g_rq(1);
            else p_f(to_integer(t_q2)) <= g_rq(0); end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- [P] PIXEL PATH (stages 2 .. 10).  Rows 0 top, 1 bottom, 2 left, 3 right.
    --   2      the edge distances (Q16 cycles; the right edge's negated) from the cell phase and the element's
    --          insets; past the tip?
    --   3      the neighbour counts, the distances in their pixel windows
    --   4      x the mantissa: sixteenths of a pixel
    --   5      the soft steps (and the Fine line's inner steps)
    --   6      tooth (Fine: - the inner tooth) = the row's coverage 0 .. 16
    --   7, 8   Grey: the films' sum;  Ink: the XOR as a triangle of the summed coverage  (0 .. 256)
    --   9, 10  luma = 64 + 876 s / 256
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_p16 : unsigned(15 downto 0);
        variable v_eL, v_eR : unsigned(16 downto 0);
        variable v_s  : unsigned(17 downto 0);
        variable v_nL, v_nR : signed(1 downto 0);
        variable v_to, v_in : unsigned(4 downto 0);
        variable v_t  : t_u5x4;
        variable v_sa, v_sb : unsigned(8 downto 0);
        variable v_sum : unsigned(9 downto 0);
        variable v_q  : unsigned(6 downto 0);
        variable v_v  : unsigned(5 downto 0);
        variable v_d  : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: the insets (the camera's step-out where the element is flagged), the distances, past the tip?
            for r in 0 to 3 loop
                if r < 2 then
                    v_p16 := b_ph(r)(27 downto 12);
                    v_eL := f_ins(l_al(r), c_f(2 * r)); v_eR := f_ins(l_ar(r), c_f(2 * r + 1));
                else
                    v_p16 := l_ps(r - 2)(27 downto 12);
                    v_eL := f_ins(h_aL(r - 2), c_f(2 * r)); v_eR := f_ins(h_aR(r - 2), c_f(2 * r + 1));
                end if;
                q_dL(r) <= signed(resize(v_p16, 18)) - signed(resize(v_eL, 18));
                v_s := resize(v_p16, 18) + resize(v_eR, 18);
                q_dR(r) <= signed(v_s) - 65536;                                   -- (negated)
                v_s := resize(v_eL, 18) + resize(v_eR, 18);
                if r < 2 then q_gone(r) <= v_s(17) or v_s(16);
                else q_gone(r) <= v_s(17) or v_s(16) or not s_cross; end if;
            end loop;
            -- 3 .. 6, per row
            for r in 0 to 3 loop
                -- 3
                v_nL := signed(q_dL(r)(17 downto 16)) + signed'('0' & q_dL(r)(15));
                v_nR := signed(q_dR(r)(17 downto 16)) + signed'('0' & q_dR(r)(15));
                if q_gone(r) = '1' then p_S(r) <= "100";          -- past the tip: no tooth
                else p_S(r) <= resize(v_nL, 3) - resize(v_nR, 3); end if;
                p_yL(r) <= f_win(q_dL(r)(15 downto 0), c_ec(r)(11 downto 9));
                p_yR(r) <= f_win(q_dR(r)(15 downto 0), c_ec(r)(5 downto 3));
                -- 4
                z_L(r) <= f_mul(p_yL(r), c_ec(r)(8 downto 6));
                z_R(r) <= f_mul(p_yR(r), c_ec(r)(2 downto 0));
                z_S(r) <= p_S(r);
                -- 5
                f_1L(r) <= f_step(z_L(r), to_signed(8, 9)); f_1R(r) <= f_stepn(z_R(r), to_signed(8, 9));
                f_2L(r) <= f_step(z_L(r), c_t16); f_2R(r) <= f_stepn(z_R(r), c_t16);
                f_S(r) <= z_S(r);
                -- 6
                v_to := f_tooth(f_1L(r), f_1R(r), f_S(r));
                v_in := f_tooth(f_2L(r), f_2R(r), f_S(r));
                if s_fine = '1' then s6_cv(r) <= v_to - v_in;
                else s6_cv(r) <= v_to; end if;
            end loop;
            -- 7: the films (sixteenths of white per row): Jaws 6, 10 (Fine 10, 16); Cross 4, 6, 2, 4 (Fine 8, 12, 5, 8)
            v_t := s6_cv;
            if s_cross = '0' then
                if s_fine = '0' then
                    v_sa := resize(v_t(0) & "00", 9) + resize(v_t(0) & '0', 9);            -- 6
                    v_sb := resize(v_t(1) & "000", 9) + resize(v_t(1) & '0', 9);           -- 10
                else
                    v_sa := resize(v_t(0) & "000", 9) + resize(v_t(0) & '0', 9);           -- 10
                    v_sb := resize(v_t(1) & "0000", 9);                                    -- 16
                end if;
            else
                if s_fine = '0' then
                    v_sa := resize(v_t(0) & "00", 9) + resize(v_t(1) & "00", 9) + resize(v_t(1) & '0', 9);      -- 4, 6
                    v_sb := resize(v_t(2) & '0', 9) + resize(v_t(3) & "00", 9);                                 -- 2, 4
                else
                    v_sa := resize(v_t(0) & "000", 9) + resize(v_t(1) & "000", 9) + resize(v_t(1) & "00", 9);   -- 8, 12
                    v_sb := resize(v_t(2) & "00", 9) + resize(v_t(2), 9) + resize(v_t(3) & "000", 9);           -- 5, 8
                end if;
            end if;
            s7_sa <= v_sa; s7_sb <= v_sb;
            s7_qa <= resize(v_t(0), 6) + resize(v_t(1), 6);
            s7_qb <= resize(v_t(2), 6) + resize(v_t(3), 6);
            -- 8
            if s_ink = '1' then
                v_q := resize(s7_qa, 7) + resize(s7_qb, 7);
                v_v := '0' & v_q(4 downto 0);
                if v_v > 16 then v_v := to_unsigned(32, 6) - v_v; end if;
                s8_s <= resize(v_v & "0000", 9);
            else
                v_sum := resize(s7_sa, 10) + resize(s7_sb, 10);
                if v_sum > 256 then s8_s <= to_unsigned(256, 9); else s8_s <= v_sum(8 downto 0); end if;
            end if;
            -- 9, 10: 64 + (219 s) >> 6 = (256 (s + 16) - 37 s) >> 6
            s9_a <= resize(s8_s, 10) + 16;
            s9_b <= resize(s8_s & "00000", 15) + resize(s8_s & "00", 15) + resize(s8_s, 15);
            v_d := (s9_a & "00000000") - resize(s9_b, 18);
            s10_y <= v_d(15 downto 6);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> box of 32 samples,
    -- edge-replicated and reloaded at every line start -> vertical one-pole in
    -- the line store (half horizontal resolution, Q13.3).  At the centre of
    -- every 32 x 32 virtual-px block the stream is compared with the two films'
    -- thresholds and the 2 bits go into the grid's write bank.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(8 downto 0);
        variable v_d : signed(16 downto 0);
        variable v_u : signed(16 downto 0);
        variable v_v : signed(20 downto 0);
        variable v_2x : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting; both phases restart from this
            -- field's drift in the zone
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

            -- the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_l9 <= resize(s_osum, 9);
            elsif v_l > 16 then s_l9 <= resize(v_l - 16, 9);
            else s_l9 <= (others => '0'); end if;
            s_sv <= data_in.avid;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;
            -- (the camera gain is in the grid's thresholds)
            s_p <= s_l9;
            s_sv1 <= s_sv; s_svd <= s_sv1;

            -- box: sum of the last 32 samples; the samples before the line start count as the first one
            if s_sv1 = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else
                bx_wp <= (others => '0');
                bx_rp <= to_unsigned(225, 8);
            end if;
            if s_bcnt(5) = '0' then v_tap := s_sfirst; else v_tap := bx_q(8 downto 0); end if;
            if s_sv1 = '1' and s_svd = '0' then
                s_sfirst <= s_p;
                s_bsum <= s_p & "00000";
                s_bcnt <= to_unsigned(1, 6);
            elsif s_sv1 = '1' then
                s_bsum <= s_bsum + s_p - v_tap;
                if s_bcnt(5) = '0' then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bv <= s_sv1; s_bcol <= s_col;

            -- vertical one-pole on the odd columns: q += (box - q) >> sh
            s_eb <= s_bsum(13 downto 1);          -- the mean x 16: Q13 luma
            s_eq <= lb_q;
            s_ev <= s_bv and not s_bcol(0); s_ecol <= s_bcol;
            v_d := signed(resize(s_eb & "000", 17)) - signed(resize(s_eq, 17));
            case to_integer(s_ish) is
                when 2      => v_u := shift_right(v_d, 2);
                when 3      => v_u := shift_right(v_d, 3);
                when others => v_u := shift_right(v_d, 4);
            end case;
            lb_wd <= unsigned(signed(s_eq) + v_u(15 downto 0));
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            -- the source line's y (Q8); at each line start: is this a block row's centre line, and which row
            if s_zone = '1' and data_in.avid = '0' then
                s_sy <= resize(s_ys(11 downto 1), 20);
            elsif s_avq = '1' and data_in.avid = '0' then
                s_sy <= s_sy + resize(s_ys, 20);
            end if;
            v_2x := resize(s_xs & '0', 5);
            if s_sv = '1' and s_sv1 = '0' then
                v_v := signed(resize(s_sy, 21)) - 7168;
                if v_v(20 downto 18) = "000" and v_v(12) = '1' and unsigned(v_v(11 downto 0)) < s_ys then
                    g_cl <= '1';
                else
                    g_cl <= '0';
                end if;
                g_row <= unsigned(v_v(17 downto 13));
                g_sx <= s_xi;
            elsif lb_we = '1' then
                g_sx <= g_sx + resize(v_2x, 13);
            end if;
            -- the grid write: the stream sample at the block's centre column is held (k_v, k_wa), the block's
            -- flags of the field before are read from the second copy (two clocks), then each film's flag is
            -- set or cleared with hysteresis about its threshold and written to both copies
            if lb_we = '1' and g_cl = '1' and g_sx(5) = '1' and g_sx(4 downto 0) < v_2x then
                k_v <= lb_wd(15 downto 3);
                k_wa <= s_bank & g_row & g_sx(11 downto 6);
                k_h <= k_h(1 downto 0) & '1';
            else
                k_h <= k_h(1 downto 0) & '0';
            end if;
            h_rf <= h_rq;
            g_we <= k_h(2);
            g_wd(0) <= f_hys(signed('0' & k_v) - signed('0' & c_thb), h_rf(0));
            g_wd(1) <= f_hys(signed('0' & k_v) - signed('0' & c_tht), h_rf(1));
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: black outside pixels 0 .. W + 1 (11), luma
    -- [1 2 1] / 4 (14, 15), 13/16 toward black (16), blanking gate (17).
    -- Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_gav_sr(8) = '1' or s_gav_sr(10) = '1' then s11_y <= s10_y; else s11_y <= to_unsigned(64, 10); end if;
            s12_y <= s11_y; s13_y <= s12_y;
            s14_ya <= resize(s11_y, 12) + resize(s13_y, 12) + 2;
            s14_yb <= s12_y;
            v_y := s14_ya + shift_left(resize(s14_yb, 12), 1);
            s15_y <= v_y(11 downto 2);
            if s15_y > 64 then v_dl := s15_y - 64; else v_dl := (others => '0'); end if;
            s16_y <= s15_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if s_gav_sr(C_LAT - 2) = '1' then s17_y <= s16_y; else s17_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s17_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture tines;
