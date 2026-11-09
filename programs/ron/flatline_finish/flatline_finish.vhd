-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- FLATLINE_FINISH -- STAMPED COPY of programs/ron/flatline/flatline.vhd (variants.py);
-- edit the master, not this file.
-- FLATLINE -- image flattening lab: edge-preserving smoothing -> luma/chroma
-- quantization -> label cleanup -> ink lines -> flat-matte composite, with the
-- raw input on a P12 wipe for side-by-side comparison.  Proof of concept: each
-- stage has selectable variants; stages are meant to be lifted into finished
-- programs.  Read DESIGN.md first (budget, stage table, deviations).
--
-- Controls (see CONTROLS.md):
--   K1 Smoother   Off / Median / Bilateral / Kuwahara (labels per build)
--   K2 Strength   bilateral range sigma; median/Kuwahara blend toward centre
--   K3 Levels     luma bins 2..8, then two coarse-chroma steps (9 labels)
--   K4 Lines      None / Edge / Edge White / XDoG|Canny / XDoG|Canny White
--   K5 Ink        line weight: boundary footprint + alpha, XDoG epsilon, Canny gain
--   K6 Shadow     0 off; offset grows to the centre; past the centre it softens
--   S7  Quant     Hard / Soft (count-weighted vertical blend + horizontal half-step)
--   S8  Levels    Fixed / Adaptive (1D k-means on last frame's histogram)
--   S9  Cleanup   Off / On (separable 5x5 mode filter on the label field)
--   S10 Temporal  Off / On (last frame's cell label votes near bin boundaries)
--                 build C (Canny): Chroma gradient Off / On
--   S11 View      Final / Lines (ink only, on mid grey)
--   P12 Wipe      processed (left) <-> raw (right); 100% = fully processed
--
-- Architecture: an F domain (every clock) and an H domain (one step per pixel
-- pair, clock enable `ce`).  The H pipeline runs at half horizontal resolution
-- so every line buffer is 1024 deep (chroma 512); see DESIGN.md section 1.
-- Stage offsets in comments are H steps after the capture of column m
-- ("stage k holds column m during step m+k").  Latency 84 clocks, all modes.
--
-- BRAM (30 EBR): 4 luma + 2 chroma + 4 label + 1 cleaned-label + 1 ink
-- history line buffers, shadow ring, temporal store, histogram, dry/sync ring.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture flatline of program_top is

    ------------------------------------------------------------------------
    -- BUILD CONFIGURATION (stamped per variant by variants.py; do not edit the
    -- stamped copies by hand -- edit programs/ron/flatline/ and restamp)
    ------------------------------------------------------------------------
    constant C_VARIANT      : string  := "E";
    constant C_HAS_MEDIAN   : boolean := true;
    constant C_HAS_BILAT    : boolean := false;
    constant C_HAS_KUWA     : boolean := false;
    constant C_HAS_XDOG     : boolean := false;
    constant C_HAS_CANNY    : boolean := false;
    constant C_HAS_TEMPORAL : boolean := true;
    constant C_HAS_SHADOW   : boolean := true;
    constant C_HAS_SOFT     : boolean := true;

    ------------------------------------------------------------------------
    -- latency: 2 * 37 H steps + 3 (front end) + 7 F stages = 84
    ------------------------------------------------------------------------
    constant C_PROCESSING_DELAY_CLKS : natural := 84;
    constant C_RING_RD : integer := 79;      -- dry ring read offset (see DESIGN)
    constant C_NH      : integer := 37;      -- last H stage (hand-off "current")

    -- tuning constants (8-bit luma units unless noted)
    constant C_HYST   : integer := 3;        -- spatial hysteresis band
    constant C_SAT1   : integer := 10;       -- grey below (octagonal |c|)
    constant C_SAT2   : integer := 36;       -- mid below, high above
    constant C_SATC   : integer := 16;       -- coarse 2-level sat threshold
    constant C_TOPG   : integer := 6;        -- lines blanked at the top of frame
    constant C_BLKY   : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID    : unsigned(9 downto 0) := to_unsigned(512, 10);

    ------------------------------------------------------------------------
    -- helpers
    ------------------------------------------------------------------------
    function f_c8(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else return resize(unsigned(v(7 downto 0)), 8); end if;
    end function;

    function f_b2sl(b : boolean) return std_logic is
    begin
        if b then return '1'; else return '0'; end if;
    end function;

    function f_umax(a, b : unsigned) return unsigned is
    begin
        if a >= b then return a; else return b; end if;
    end function;

    function f_umin(a, b : unsigned) return unsigned is
    begin
        if a <= b then return a; else return b; end if;
    end function;

    -- |v| of a 9-bit signed in the range -255..255 -> 8 bits
    function f_abs8(v : signed(8 downto 0)) return unsigned is
        variable t : signed(8 downto 0);
    begin
        if v < 0 then t := -v; else t := v; end if;
        return unsigned(t(7 downto 0));
    end function;

    function f_pop7(v : std_logic_vector(6 downto 0)) return unsigned is
        variable n : unsigned(2 downto 0) := (others => '0');
    begin
        for i in 0 to 6 loop
            if v(i) = '1' then n := n + 1; end if;
        end loop;
        return n;
    end function;

    -- 5-sample mode of 3-bit symbols: partner counts, then pick (ties -> centre)
    type t_sym5 is array (0 to 4) of unsigned(2 downto 0);
    type t_cnt5 is array (0 to 4) of unsigned(2 downto 0);
    -- popcount of 4 bits as a lookup (an incrementer chain costs carries)
    function f_pop4(v : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case v is
            when "0000" => return "000";
            when "0001" | "0010" | "0100" | "1000" => return "001";
            when "0111" | "1011" | "1101" | "1110" => return "011";
            when "1111" => return "100";
            when others => return "010";
        end case;
    end function;
    function f_cnt2(a, b : std_logic) return unsigned is
    begin
        return unsigned'((a and b) & (a xor b));
    end function;
    function f_cnt4(v : std_logic_vector(3 downto 0)) return unsigned is
        variable c : unsigned(2 downto 0);
    begin
        c := (others => '0');
        for i in 0 to 3 loop if v(i) = '1' then c := c + 1; end if; end loop;
        return c;
    end function;
    function f_cnt5(s : t_sym5) return t_cnt5 is
        variable c : t_cnt5;
        variable e : std_logic_vector(3 downto 0);
        variable k : integer;
    begin
        for i in 0 to 4 loop
            k := 0;
            for j in 0 to 4 loop
                if i /= j then
                    if s(i) = s(j) then e(k) := '1'; else e(k) := '0'; end if;
                    k := k + 1;
                end if;
            end loop;
            c(i) := f_pop4(e);
        end loop;
        return c;
    end function;
    -- argmax of five 3-bit counts (0..4) as parallel threshold logic: no
    -- compare chains (the sequential max/compare form routed at 54 MHz).
    -- mask(i) = '0' excludes a sample.  Returns index(2:0) & level(2:0), level
    -- = the winning count + 1 (0 when nothing is left in the mask).
    function f_argmax5(c : t_cnt5; mask : std_logic_vector(0 to 4)) return unsigned is
        variable s4, s3, s2, s1, s0, sel : std_logic_vector(0 to 4);
        variable lvl : unsigned(2 downto 0);
        variable idx : unsigned(2 downto 0);
    begin
        for i in 0 to 4 loop
            s4(i) := mask(i) and c(i)(2);
            s3(i) := mask(i) and (c(i)(2) or (c(i)(1) and c(i)(0)));
            s2(i) := mask(i) and (c(i)(2) or c(i)(1));
            s1(i) := mask(i) and (c(i)(2) or c(i)(1) or c(i)(0));
            s0(i) := mask(i);
        end loop;
        if s4 /= "00000" then sel := s4; lvl := "101";
        elsif s3 /= "00000" then sel := s3; lvl := "100";
        elsif s2 /= "00000" then sel := s2; lvl := "011";
        elsif s1 /= "00000" then sel := s1; lvl := "010";
        elsif s0 /= "00000" then sel := s0; lvl := "001";
        else sel := "00100"; lvl := "000"; end if;
        -- ties: centre first (2, 1, 3, 0, 4)
        if sel(2) = '1' then idx := "010";
        elsif sel(1) = '1' then idx := "001";
        elsif sel(3) = '1' then idx := "011";
        elsif sel(0) = '1' then idx := "000";
        else idx := "100"; end if;
        return idx & lvl;
    end function;
    function f_pick5(c : t_cnt5) return integer is
        variable r : unsigned(5 downto 0);
    begin
        r := f_argmax5(c, "11111");
        return to_integer(r(5 downto 3));
    end function;

    -- the one smoother of this build: 1 median, 2 bilateral, 3 kuwahara (K1 is
    -- Off / <smoother>, so its zone is the knob's top bit)
    function f_smcode return integer is
    begin
        if C_HAS_BILAT then return 2; elsif C_HAS_KUWA then return 3; elsif C_HAS_MEDIAN then return 1; else return 0; end if;
    end function;
    constant C_SMCODE : integer := f_smcode;

    -- representative chroma (internal BT.601: U = Cb, V = Cr), 10-bit.
    -- Sector k is centred at 22.5 + 45k degrees (coarse: 45 + 90k); the
    -- magnitudes are precomputed (R x cos/sin: 160 -> 148/61, 320 -> 296/123,
    -- coarse 240 -> 170) so no runtime multiply or divide is synthesized.
    function f_rep_uv(hue : unsigned(2 downto 0); sat : unsigned(1 downto 0);
                      coarse : unsigned(1 downto 0)) return std_logic_vector is
        variable big, small : unsigned(8 downto 0);
        variable mu, mv : unsigned(8 downto 0);     -- |dU|, |dV|
        variable cu, cv : unsigned(9 downto 0);
        variable nu, nv : std_logic;                -- negative U / V
    begin
        if coarse = "10" then
            if sat = "00" then big := to_unsigned(0, 9); else big := to_unsigned(170, 9); end if;
            small := big;
        elsif coarse = "01" then
            case sat is
                when "00" => big := to_unsigned(0, 9);
                when "01" => big := to_unsigned(113, 9);
                when others => big := to_unsigned(226, 9);
            end case;
            small := big;
        else
            case sat is
                when "00" => big := to_unsigned(0, 9); small := to_unsigned(0, 9);
                when "01" => big := to_unsigned(148, 9); small := to_unsigned(61, 9);
                when others => big := to_unsigned(296, 9); small := to_unsigned(123, 9);
            end case;
        end if;
        -- odd sectors (1,3,5,7 at 67.5/112.5/...) have the small cosine
        if hue(0) = '1' then mu := small; mv := big; else mu := big; mv := small; end if;
        -- signs: U negative in sectors 2..5, V negative in sectors 4..7
        if hue = "010" or hue = "011" or hue = "100" or hue = "101" then nu := '1'; else nu := '0'; end if;
        nv := hue(2);
        if nu = '1' then cu := to_unsigned(512, 10) - mu; else cu := to_unsigned(512, 10) + mu; end if;
        if nv = '1' then cv := to_unsigned(512, 10) - mv; else cv := to_unsigned(512, 10) + mv; end if;
        return std_logic_vector(cu) & std_logic_vector(cv);
    end function;

    ------------------------------------------------------------------------
    -- memories (all 1W1R; read every clock, written on ce_d: never a collision)
    ------------------------------------------------------------------------
    type t_lb8  is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_lb16 is array (0 to 511)  of std_logic_vector(15 downto 0);
    type t_sh   is array (0 to 8191) of std_logic_vector(0 downto 0);
    type t_tmp  is array (0 to 511)  of std_logic_vector(7 downto 0);
    type t_hist is array (0 to 255)  of unsigned(15 downto 0);
    type t_ring is array (0 to 127)  of std_logic_vector(31 downto 0);

    signal lby1, lby2, lby3, lby4 : t_lb8 := (others => x"10");   -- raw luma rows
    signal lbc1, lbc2 : t_lb16 := (others => x"8080");           -- chroma rows
    signal lbl1, lbl2, lbl3, lbl4 : t_lb8 := (others => (others => '0'));  -- row-modes
    signal lbcl  : t_lb8 := (others => (others => '0'));          -- cleaned labels
    signal lbink : t_lb8 := (others => (others => '0'));          -- ink history
    signal shr   : t_sh  := (others => (others => '0'));          -- shadow ring
    signal tmpr  : t_tmp := (others => (others => '0'));          -- temporal cells
    signal hist  : t_hist := (others => (others => '0'));         -- luma histogram
    signal ring  : t_ring := (others => (others => '0'));         -- dry video + syncs
    attribute no_rw_check : boolean;
    attribute no_rw_check of lby1  : signal is true;
    attribute no_rw_check of lby2  : signal is true;
    attribute no_rw_check of lby3  : signal is true;
    attribute no_rw_check of lby4  : signal is true;
    attribute no_rw_check of lbc1  : signal is true;
    attribute no_rw_check of lbc2  : signal is true;
    attribute no_rw_check of lbl1  : signal is true;
    attribute no_rw_check of lbl2  : signal is true;
    attribute no_rw_check of lbl3  : signal is true;
    attribute no_rw_check of lbl4  : signal is true;
    attribute no_rw_check of lbcl  : signal is true;
    attribute no_rw_check of lbink : signal is true;
    attribute no_rw_check of shr   : signal is true;
    attribute no_rw_check of tmpr  : signal is true;
    attribute no_rw_check of hist  : signal is true;
    attribute no_rw_check of ring  : signal is true;

    signal lby_q1, lby_q2, lby_q3, lby_q4 : std_logic_vector(7 downto 0) := (others => '0');
    signal lbc_q1, lbc_q2 : std_logic_vector(15 downto 0) := (others => '0');
    signal lbl_q1, lbl_q2, lbl_q3, lbl_q4 : std_logic_vector(7 downto 0) := (others => '0');
    signal lbcl_q, lbink_q : std_logic_vector(7 downto 0) := (others => '0');
    signal shr_q  : std_logic_vector(0 downto 0) := (others => '0');
    signal tmp_q  : std_logic_vector(7 downto 0) := (others => '0');
    signal hist_q : unsigned(15 downto 0) := (others => '0');
    signal ring_q : std_logic_vector(31 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- controls (free-running decode, one op per cycle)
    ------------------------------------------------------------------------
    signal k1, k2, k3, k4, k5, k6, p12 : unsigned(9 downto 0) := (others => '0');
    signal s_soft, s_adapt, s_clean, s_temporal, s_lines : std_logic := '0';
    signal k1p, k1m, k3p, k3m, k4p, k4m : unsigned(9 downto 0) := (others => '0');
    signal z3p, z3m : unsigned(2 downto 0) := (others => '0');
    signal z4p, z4m : unsigned(1 downto 0) := (others => '0');
    signal c_sm    : integer range 0 to 3 := 0;        -- smoother code
    signal c_z3    : integer range 0 to 7 := 2;        -- K3 zone
    signal c_z4    : integer range 0 to 3 := 1;        -- K4 zone
    signal c_nlev  : integer range 2 to 8 := 4;
    signal c_coarse : unsigned(1 downto 0) := "00";
    signal c_lmode : unsigned(1 downto 0) := "01";     -- 0 none, 1 edge, 2 alt (xdog/canny), 3 bold edge
    signal c_white : std_logic := '0';
    signal c_sigsh : unsigned(2 downto 0) := "011";    -- bilateral sigma = 2^(sigsh+1): 4..128
    signal c_kb2   : unsigned(1 downto 0) := "11";     -- blend 1/4 .. 1 (median / kuwahara)
    signal c_width : unsigned(1 downto 0) := "00";     -- 0 thin 1 medium 2 thick
    signal c_inkw  : unsigned(1 downto 0) := "11";     -- boundary ink alpha (half / full)
    signal c_eps8  : unsigned(4 downto 0) := (others => '0');  -- XDoG epsilon
    signal c_cgain : unsigned(3 downto 0) := x"4";     -- Canny pre-gain
    signal c_shofs : unsigned(3 downto 0) := (others => '0');  -- shadow offset, 1080-line units (0..15)
    signal c_shsoft : unsigned(1 downto 0) := "00";    -- soften 1/2/4/8
    signal c_sd, c_720, c_il : std_logic := '0';
    signal c_my, c_mx : unsigned(7 downto 0) := x"FF"; -- shadow scale per mode (x1/256)

    ------------------------------------------------------------------------
    -- per-frame values (sequencer outputs)
    ------------------------------------------------------------------------
    -- thr(0) = black (16), never written; thr(k >= nlev) = 511 (never passed)
    type t_thr is array (0 to 7) of unsigned(8 downto 0);
    type t_cen is array (0 to 7) of unsigned(7 downto 0);
    signal thr : t_thr := (to_unsigned(16, 9), to_unsigned(71, 9), to_unsigned(126, 9), to_unsigned(181, 9),
                           to_unsigned(511, 9), to_unsigned(511, 9), to_unsigned(511, 9), to_unsigned(511, 9));
    signal cen : t_cen := (to_unsigned(43, 8), to_unsigned(98, 8), to_unsigned(153, 8), to_unsigned(208, 8),
                           to_unsigned(235, 8), to_unsigned(235, 8), to_unsigned(235, 8), to_unsigned(235, 8));
    -- threshold / centroid register files are written ONLY through these: one
    -- shared value bus each and a registered one-hot enable (scattered
    -- register-file writes decoded from the state were a 31 ns routing path)
    signal thr_wd  : unsigned(8 downto 0) := (others => '0');
    signal cen_wd  : unsigned(7 downto 0) := (others => '0');
    signal thr_we  : std_logic_vector(1 to 7) := (others => '0');
    signal cen_we  : std_logic_vector(0 to 7) := (others => '0');
    signal f_nlev  : integer range 2 to 8 := 4;        -- levels in force this frame
    signal f_coarse : unsigned(1 downto 0) := "00";
    signal f_dy    : unsigned(3 downto 0) := (others => '0');   -- shadow rows
    signal f_dxq   : unsigned(4 downto 0) := (others => '0');   -- shadow quarter columns
    signal f_wx    : unsigned(10 downto 0) := (others => '0');  -- wipe column (half-res)
    signal f_kb2   : unsigned(1 downto 0) := "11";     -- effective blend code (bilateral: full)
    signal f_temp  : std_logic := '0';

    -- sequencer
    signal sq      : integer range 0 to 63 := 0;
    signal sq_k    : integer range 0 to 8 := 0;
    signal sq_i    : unsigned(5 downto 0) := (others => '0');
    signal sq_acc  : unsigned(8 downto 0) := (others => '0');
    signal sq_fdel : unsigned(6 downto 0) := to_unsigned(55, 7);
    signal sq_reload : std_logic := '1';
    signal sq_nprev : integer range 2 to 8 := 2;
    signal sq_hval : unsigned(15 downto 0) := (others => '0');
    signal sq_j, sq_jm : integer range 0 to 8 := 0;      -- thresholds / medians placed
    signal sq_cum, sq_tt, sq_mt, sq_step : unsigned(17 downto 0) := (others => '0');
    -- percentile placement, one op per state
    signal pl_thr, pl_med : std_logic := '0';               -- which placement fired
    signal pl_old, pl_prev2, pl_new : unsigned(8 downto 0) := (others => '0');
    signal pl_dif : signed(9 downto 0) := (others => '0');
    signal pl_lp : signed(10 downto 0) := (others => '0');
    signal hist_sa : unsigned(7 downto 0) := (others => '0');
    signal hist_swe, seq_hist : std_logic := '0';
    -- serial multiplier 21 x 10 (10 steps) and a tiny divider by N (3 bits)
    signal mul_a   : unsigned(20 downto 0) := (others => '0');
    signal mul_b   : unsigned(9 downto 0) := (others => '0');
    signal mul_p   : unsigned(20 downto 0) := (others => '0');
    signal mul_n   : unsigned(3 downto 0) := (others => '0');
    signal mul_busy : std_logic := '0';
    -- states only post an op code + go; the multiplier fetches its operands on
    -- the next clock (the state decode fanning into 31 operand muxes was the
    -- HD Analog critical path at 57 MHz)
    signal mul_op  : unsigned(1 downto 0) := (others => '0');   -- 0 total, 1 shadow dy, 2 shadow dx, 3 wipe
    signal mul_go  : std_logic := '0';
    signal dv_q    : unsigned(17 downto 0) := (others => '0');
    signal dv_rem  : unsigned(4 downto 0) := (others => '0');
    signal dv_n    : unsigned(4 downto 0) := (others => '0');
    signal div_busy : std_logic := '0';

    ------------------------------------------------------------------------
    -- F domain front end
    ------------------------------------------------------------------------
    signal f0_y, f0_u, f0_v : unsigned(9 downto 0) := (others => '0');
    signal f0_avid, f0_hs, f0_vs, f0_fld : std_logic := '0';
    signal f0_x  : unsigned(11 downto 0) := (others => '0');
    signal f0p_y8, f0p_u8, f0p_v8 : unsigned(7 downto 0) := (others => '0');
    signal f0p_avid : std_logic := '0';
    signal ce, ce_d : std_logic := '0';
    attribute keep : boolean;
    attribute keep of ce : signal is true;
    signal meas_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal meas_h : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal aline  : unsigned(10 downto 0) := (others => '0');
    signal vs_prev, saw_act, vs_edge : std_logic := '0';
    signal hs_prev : std_logic := '1';
    -- hsync -> avid parity vote (the 4:2:2 packer counts from hsync)
    signal par_ph, par_usual, line_off : std_logic := '0';
    signal par_cnt : unsigned(3 downto 0) := (others => '0');
    signal ring_w  : unsigned(6 downto 0) := (others => '0');

    ------------------------------------------------------------------------
    -- H domain
    ------------------------------------------------------------------------
    signal hv_c, hf_c : std_logic_vector(0 to C_NH) := (others => '0');
    signal hx : unsigned(9 downto 0) := (others => '0');         -- column at H0
    signal h0_y, h0_u, h0_v : unsigned(7 downto 0) := (others => '0');
    -- H1: newest luma column rows r-4 .. r (ny0 oldest line)
    type t_col5 is array (0 to 4) of unsigned(7 downto 0);
    signal ny : t_col5 := (others => (others => '0'));
    signal hx_d1 : unsigned(9 downto 0) := (others => '0');
    signal h1_u, h1_v : unsigned(7 downto 0) := (others => '0');
    signal cq_u, cq_v : unsigned(7 downto 0) := (others => '0');  -- quarter-res chroma
    signal cq_v1 : std_logic := '0';
    signal c1_r1, c1_r2 : std_logic_vector(15 downto 0) := (others => '0');
    -- identity centre (row r-2 of column m at stage 1), delayed 12 steps through
    -- the delay ring together with the chroma class
    signal cen1, cen13 : unsigned(7 downto 0) := (others => '0');
    type t_dring is array (0 to 31) of std_logic_vector(15 downto 0);
    signal dring : t_dring := (others => (others => '0'));
    attribute no_rw_check of dring : signal is true;
    signal dr_w : unsigned(4 downto 0) := (others => '0');
    signal dr_q : std_logic_vector(15 downto 0) := (others => '0');
    signal dr_c : std_logic_vector(15 downto 0) := (others => '0');
    -- smoother outputs aligned at stage 14
    signal med14, bil14, bil_vc14, kw14, cen14 : unsigned(7 downto 0) := (others => '0');
    -- select / blend
    signal s15_sm, s15_cen, s16_cen : unsigned(7 downto 0) := (others => '0');
    signal s16_d : signed(8 downto 0) := (others => '0');
    signal ys17, ys18 : unsigned(7 downto 0) := (others => '0');  -- smoothed luma
    -- chroma smoothing / quantization (quarter res: a sample every other step)
    signal c_fill : std_logic := '0';
    signal cvs_u, cvs_v, cvp_u, cvp_v : unsigned(8 downto 0) := (others => '0');   -- 2-row sums, this / previous quarter column
    signal us8, vs8 : unsigned(7 downto 0) := x"80";
    signal c4_du, c4_dv : signed(8 downto 0) := (others => '0');
    signal c4_au, c4_av : unsigned(7 downto 0) := (others => '0');
    signal c5_sat : unsigned(8 downto 0) := (others => '0');
    signal c5_sdu, c5_sdv, c5_gt : std_logic := '0';
    signal c6_hue : unsigned(2 downto 0) := (others => '0');
    signal c6_sat : unsigned(1 downto 0) := (others => '0');
    signal cc0 : unsigned(4 downto 0) := (others => '0');          -- chroma class, delayed 12 steps in the ring
    -- quantizer
    signal q19_h, q19_lo3, q19_hi3 : std_logic_vector(6 downto 0) := (others => '0');
    signal q20_bh, q20_blo3, q20_bhi3 : unsigned(2 downto 0) := (others => '0');
    signal tmp_c : std_logic_vector(7 downto 0) := (others => '0');
    signal lab21 : unsigned(7 downto 0) := (others => '0');       -- {lum, hue, sat}
    -- histogram sampling
    signal cnt18 : unsigned(9 downto 0) := (others => '0');
    signal hist_ra : unsigned(5 downto 0) := (others => '0');
    signal hs_c : std_logic_vector(0 to 2) := (others => '0');
    signal hq20 : unsigned(15 downto 0) := (others => '0');
    signal tmp_ra : unsigned(8 downto 0) := (others => '0');
    -- mode filter, horizontal (always on = the minimum-run-length filter)
    type t_lab6 is array (0 to 5) of unsigned(7 downto 0);
    signal mw : t_lab6 := (others => (others => '0'));
    signal mh_cl, mh_ch, mh_cs : t_cnt5 := (others => (others => '0'));
    signal rm26 : unsigned(7 downto 0) := (others => '0');
    signal cnt26, cnt26_d : unsigned(9 downto 0) := (others => '0');
    -- mode filter, vertical
    type t_lab5 is array (0 to 4) of unsigned(7 downto 0);
    signal mv : t_lab5 := (others => (others => '0'));            -- 0 = row r .. 4 = row r-4
    signal mv_cl, mv_ch, mv_cs : t_cnt5 := (others => (others => '0'));
    signal mvc : t_lab5 := (others => (others => '0'));
    signal cl29 : unsigned(7 downto 0) := (others => '0');
    signal cl29_pick : integer range 0 to 4 := 2;
    signal cl29_cl : t_cnt5 := (others => (others => '0'));
    signal cl29_l : t_sym5 := (others => (others => '0'));
    -- soft blend
    signal sf30_lum2 : unsigned(2 downto 0) := (others => '0');
    signal sf30_n2 : unsigned(2 downto 0) := (others => '0');
    signal sf30_nm : unsigned(2 downto 0) := (others => '0');
    signal sf30_cl : unsigned(7 downto 0) := (others => '0');
    signal sf31_w : unsigned(3 downto 0) := (others => '0');
    signal sf31_d : signed(8 downto 0) := (others => '0');
    signal sf31_c, sf32_c : unsigned(7 downto 0) := (others => '0');
    signal sf31_hs, sf32_hs, sf33_hs, sf34_hs, sf35_hs : unsigned(4 downto 0) := (others => '0');
    signal sf31_lum, sf32_lum, sf33_lum, sf34_lum, sf35_lum : unsigned(2 downto 0) := (others => '0');
    signal sf32_p : signed(13 downto 0) := (others => '0');
    signal ym33, ym34, ym35 : unsigned(7 downto 0) := (others => '0');
    type t_ld is array (30 to 34) of unsigned(7 downto 0);
    signal ld : t_ld := (others => (others => '0'));
    -- boundary
    type t_lab3 is array (0 to 2) of unsigned(7 downto 0);
    signal cw3 : t_lab3 := (others => (others => '0'));
    signal cnt30, cnt30_d, cnt30_dd : unsigned(9 downto 0) := (others => '0');
    signal cla31 : unsigned(7 downto 0) := (others => '0');
    signal inkq32 : std_logic_vector(7 downto 0) := (others => '0');
    signal t32, dr32 : std_logic := '0';
    type t_bit3 is array (0 to 2) of std_logic;
    signal srt, srdr, srt1, srt2 : t_bit3 := (others => '0');
    type t_a3 is array (0 to 2) of unsigned(1 downto 0);
    signal srxa : t_a3 := (others => (others => '0'));
    signal ia35 : unsigned(1 downto 0) := (others => '0');        -- ink alpha 0/4/8/16 sixteenths
    -- line alpha from XDoG / Canny, delayed to the ink RAM write (stage 32)
    signal xa10 : unsigned(1 downto 0) := (others => '0');
    type t_xa_d is array (11 to 32) of unsigned(1 downto 0);
    signal xa_d : t_xa_d := (others => (others => '0'));
    signal ca13 : unsigned(1 downto 0) := (others => '0');
    type t_ca_d is array (15 to 32) of unsigned(1 downto 0);
    signal ca_d : t_ca_d := (others => (others => '0'));
    -- shadow
    signal srow_w, srow_r : unsigned(3 downto 0) := (others => '0');
    signal sline_r : unsigned(4 downto 0) := (others => '0');
    signal sil_prev : std_logic := '0';
    signal cnt33s : unsigned(9 downto 0) := (others => '0');
    signal sh_ra : unsigned(12 downto 0) := (others => '0');
    signal sh_oob : std_logic := '0';
    signal sw : std_logic_vector(0 to 16) := (others => '0');
    -- pipelined shadow popcount (3 stages of small adds; a serial 16-bit count in one stage
    -- was 7 ns of logic and the critical path of build E at 52 MHz)
    signal shc_a, shc_b : unsigned(1 downto 0) := (others => '0');
    signal shc_c, shc_d, shc_e : unsigned(2 downto 0) := (others => '0');
    signal shs_2, sht_2 : unsigned(1 downto 0) := (others => '0');
    signal shs_4, sht_4, shs_c : unsigned(2 downto 0) := (others => '0');
    signal shs_de, sht_8, sht_de : unsigned(3 downto 0) := (others => '0');
    signal sha36, sha37 : unsigned(4 downto 0) := (others => '0');
    -- hand-off
    signal cnt37h : unsigned(10 downto 0) := (others => '0');
    signal ho_n_y, ho_c_y, ho_p_y : unsigned(7 downto 0) := (others => '0');
    signal ho_n_hs, ho_c_hs : unsigned(4 downto 0) := (others => '0');
    signal ho_n_lum, ho_c_lum : unsigned(2 downto 0) := (others => '0');
    signal ho_n_a, ho_c_a : unsigned(1 downto 0) := (others => '0');
    signal ho_n_v, ho_c_v : std_logic := '0';

    ------------------------------------------------------------------------
    -- F domain output
    ------------------------------------------------------------------------
    signal o1_y, o1_u, o1_v : unsigned(9 downto 0) := (others => '0');
    signal o1_a : unsigned(1 downto 0) := (others => '0');
    signal o1_s : unsigned(4 downto 0) := (others => '0');
    signal o1_bg, o1_noink : std_logic := '0';
    signal o3_y, o3_u, o3_v : unsigned(9 downto 0) := (others => '0');
    signal o3_py, o3_pu, o3_pv : signed(10 downto 0) := (others => '0');
    signal o4_y, o4_u, o4_v : unsigned(9 downto 0) := (others => '0');
    signal g1_y, g1_u, g1_v, g2_y, g2_u, g2_v : unsigned(9 downto 0) := (others => '0');
    signal o6_y, o6_u, o6_v : unsigned(9 downto 0) := (others => '0');
    signal o7_y, o7_u, o7_v : unsigned(9 downto 0) := (others => '0');
    signal wipe_c : std_logic_vector(1 to 6) := (others => '0');
    signal rq_avid_d, out_sel, out_avid : std_logic := '0';
    signal out_y, out_u, out_v : unsigned(9 downto 0) := (others => '0');
    signal out_hs, out_vs, out_fld : std_logic := '1';

begin

    ------------------------------------------------------------------------
    -- controls: free-running decode (hysteresis on stepped knobs)
    ------------------------------------------------------------------------
    p_ctl : process(clk)
        variable v_z : integer range 0 to 8;
    begin
        if rising_edge(clk) then
            k1 <= unsigned(registers_in(0)); k2 <= unsigned(registers_in(1));
            k3 <= unsigned(registers_in(2)); k4 <= unsigned(registers_in(3));
            k5 <= unsigned(registers_in(4)); k6 <= unsigned(registers_in(5));
            p12 <= unsigned(registers_in(7));
            s_soft <= registers_in(6)(0); s_adapt <= registers_in(6)(1);
            s_clean <= registers_in(6)(2); s_temporal <= registers_in(6)(3);
            s_lines <= registers_in(6)(4);

            -- timing class
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 4 | 8 | 12 => c_sd <= '1'; c_720 <= '0';
                when 5 | 6 | 14     => c_sd <= '0'; c_720 <= '1';
                when others         => c_sd <= '0'; c_720 <= '0';
            end case;
            case to_integer(unsigned(registers_in(8)(3 downto 0))) is
                when 0 | 8 | 1 | 2 | 10 => c_il <= '1';
                when others             => c_il <= '0';
            end case;
            -- shadow scale: rows per 1080 unit (x256), pixels per unit x64 (quarter px)
            if c_sd = '1' then
                if c_il = '1' then c_my <= to_unsigned(58, 8); else c_my <= to_unsigned(114, 8); end if;
                c_mx <= to_unsigned(32, 8);
            elsif c_720 = '1' then
                c_my <= to_unsigned(171, 8); c_mx <= to_unsigned(43, 8);
            else
                if c_il = '1' then c_my <= to_unsigned(128, 8); else c_my <= to_unsigned(255, 8); end if;
                c_mx <= to_unsigned(64, 8);
            end if;

            -- stepped knobs: zone of k+8 and k-8 must agree; labelled default
            -- loads the label INDEX (< n) at power-up
            if k1 > 1015 then k1p <= (others => '1'); else k1p <= k1 + 8; end if;
            if k1 < 8 then k1m <= (others => '0'); else k1m <= k1 - 8; end if;
            if k1 < 2 then
                if k1(0) = '1' then c_sm <= C_SMCODE; else c_sm <= 0; end if;
            elsif k1p(9) = k1m(9) then
                if k1p(9) = '1' then c_sm <= C_SMCODE; else c_sm <= 0; end if;
            end if;

            if k3 > 1015 then k3p <= (others => '1'); else k3p <= k3 + 8; end if;
            if k3 < 8 then k3m <= (others => '0'); else k3m <= k3 - 8; end if;
            z3p <= k3p(9 downto 7); z3m <= k3m(9 downto 7);
            if k3 < 8 then c_z3 <= to_integer(k3);
            elsif z3p = z3m then c_z3 <= to_integer(z3p); end if;
            if c_z3 <= 6 then c_nlev <= c_z3 + 2; else c_nlev <= 8; end if;
            if c_z3 = 7 then c_coarse <= "10"; else c_coarse <= "00"; end if;

            if k4 > 1015 then k4p <= (others => '1'); else k4p <= k4 + 8; end if;
            if k4 < 8 then k4m <= (others => '0'); else k4m <= k4 - 8; end if;
            z4p <= k4p(9 downto 8); z4m <= k4m(9 downto 8);
            if k4 < 4 then c_z4 <= to_integer(k4);
            elsif z4p = z4m then c_z4 <= to_integer(z4p); end if;
            -- 4 labels: None / Edge / Edge White / XDoG|Canny (or Edge Bold)
            case c_z4 is
                when 0 => c_lmode <= "00"; c_white <= '0';
                when 1 => c_lmode <= "01"; c_white <= '0';
                when 2 => c_lmode <= "01"; c_white <= '1';
                when others =>
                    if C_HAS_XDOG or C_HAS_CANNY then c_lmode <= "10"; else c_lmode <= "11"; end if;
                    c_white <= '0';
            end case;

            -- K2: bilateral sigma in octaves 4..128 (knob top 3 bits, 6 steps), blend quarters
            if k2(9 downto 7) > 5 then c_sigsh <= "101"; else c_sigsh <= k2(9 downto 7); end if;
            c_kb2 <= k2(9 downto 8);
            -- K5: footprint + alpha (boundary), epsilon (XDoG), gain (Canny)
            if k5 < 342 then c_width <= "00"; elsif k5 < 683 then c_width <= "01"; else c_width <= "10"; end if;
            if k5(9 downto 8) = "00" then c_inkw <= "10"; else c_inkw <= "11"; end if;   -- half ink in the bottom quarter
            c_eps8 <= k5(9 downto 5);
            c_cgain <= k5(9 downto 6) + 1;
            -- K6: offset over the lower half, soften over the upper half
            if k6(9) = '0' then
                c_shofs <= k6(8 downto 5); c_shsoft <= "00";
            else
                c_shofs <= (others => '1'); c_shsoft <= k6(8 downto 7);
            end if;
        end if;
    end process p_ctl;

    ------------------------------------------------------------------------
    -- F front end: input latch (U/V swapped to BT.601 internally), pair
    -- average, counters, half-rate enable, parity vote, dry ring write
    ------------------------------------------------------------------------
    p_front : process(clk)
        variable v_x : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- F0
            f0_y <= unsigned(data_in.y); f0_u <= unsigned(data_in.v); f0_v <= unsigned(data_in.u);
            f0_avid <= data_in.avid; f0_hs <= data_in.hsync_n; f0_vs <= data_in.vsync_n; f0_fld <= data_in.field_n;
            if data_in.avid = '1' and f0_avid = '0' then v_x := (others => '0'); else v_x := f0_x + 1; end if;
            f0_x <= v_x;
            -- previous pixel (8-bit)
            f0p_y8 <= f0_y(9 downto 2); f0p_u8 <= f0_u(9 downto 2); f0p_v8 <= f0_v(9 downto 2);
            f0p_avid <= f0_avid;
            -- half-rate enable: '1' while F0 holds the second pixel of a pair (H0
            -- then captures the pair average), toggling through blanking
            -- (re-aligned at every active rise)
            if data_in.avid = '1' then ce <= v_x(0); else ce <= not ce; end if;
            ce_d <= ce;

            -- measured width / height, active line counter
            if f0_avid = '0' and f0p_avid = '1' then
                meas_w <= f0_x;                       -- f0_x = last index + 1 here
                if aline /= 2047 then aline <= aline + 1; end if;
            end if;
            -- frame anchor: first vsync falling edge after active video
            vs_prev <= f0_vs;
            vs_edge <= '0';
            if f0_avid = '1' then saw_act <= '1'; end if;
            if f0_vs = '0' and vs_prev = '1' and saw_act = '1' then
                saw_act <= '0'; vs_edge <= '1';
                meas_h <= aline; aline <= (others => '0');
            end if;

            -- hsync -> avid phase (the encoder pairs Cb/Cr counting from hsync)
            hs_prev <= f0_hs;
            if f0_hs = '0' and hs_prev = '1' then par_ph <= '0'; else par_ph <= not par_ph; end if;
            if f0_avid = '1' and f0p_avid = '0' then
                line_off <= par_ph xor par_usual;
                if par_ph = '1' then
                    if par_cnt /= 15 then par_cnt <= par_cnt + 1; end if;
                else
                    if par_cnt /= 0 then par_cnt <= par_cnt - 1; end if;
                end if;
            end if;
            if par_cnt = 15 then par_usual <= '1'; elsif par_cnt = 0 then par_usual <= '0'; end if;

            ring_w <= ring_w + 1;
        end if;
    end process p_front;

    -- dry ring: hardware-order chroma (8 bit), syncs, parity flag
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            ring_q <= ring(to_integer(ring_w - C_RING_RD));
            ring(to_integer(ring_w)) <= '0' & line_off & f0_fld & f0_vs & f0_hs & f0_avid
                & std_logic_vector(f0_u(9 downto 2)) & std_logic_vector(f0_v(9 downto 2))
                & std_logic_vector(f0_y);
        end if;
    end process p_ring;
    -- (ring bits: 9:0 y, 17:10 raw u (hardware), 25:18 raw v (hardware), 26 avid, 27 hs, 28 vs, 29 fld, 30 line_off)
    -- note f0_u holds data_in.v (internal order), so the "raw u" field = data_in.v ... restored at output

    ------------------------------------------------------------------------
    -- line buffer memories (read every clock, write on ce_d)
    ------------------------------------------------------------------------
    p_lby : process(clk)
    begin
        if rising_edge(clk) then
            lby_q1 <= lby1(to_integer(hx)); lby_q2 <= lby2(to_integer(hx));
            lby_q3 <= lby3(to_integer(hx)); lby_q4 <= lby4(to_integer(hx));
            if ce_d = '1' and hv_c(1) = '1' then
                lby1(to_integer(hx_d1)) <= std_logic_vector(ny(4));
                lby2(to_integer(hx_d1)) <= std_logic_vector(ny(3));
                lby3(to_integer(hx_d1)) <= std_logic_vector(ny(2));
                lby4(to_integer(hx_d1)) <= std_logic_vector(ny(1));
            end if;
        end if;
    end process p_lby;

    p_lbc : process(clk)
    begin
        if rising_edge(clk) then
            lbc_q1 <= lbc1(to_integer(hx(9 downto 1)));
            lbc_q2 <= lbc2(to_integer(hx(9 downto 1)));
            if ce_d = '1' and cq_v1 = '1' then
                lbc1(to_integer(hx_d1(9 downto 1))) <= std_logic_vector(cq_u) & std_logic_vector(cq_v);
                lbc2(to_integer(hx_d1(9 downto 1))) <= c1_r1;
            end if;
        end if;
    end process p_lbc;

    p_lbl : process(clk)
    begin
        if rising_edge(clk) then
            lbl_q1 <= lbl1(to_integer(cnt26)); lbl_q2 <= lbl2(to_integer(cnt26));
            lbl_q3 <= lbl3(to_integer(cnt26)); lbl_q4 <= lbl4(to_integer(cnt26));
            if ce_d = '1' and hv_c(27) = '1' then
                lbl1(to_integer(cnt26_d)) <= std_logic_vector(mv(0));
                lbl2(to_integer(cnt26_d)) <= std_logic_vector(mv(1));
                lbl3(to_integer(cnt26_d)) <= std_logic_vector(mv(2));
                lbl4(to_integer(cnt26_d)) <= std_logic_vector(mv(3));
            end if;
        end if;
    end process p_lbl;

    p_lbcl : process(clk)
    begin
        if rising_edge(clk) then
            lbcl_q <= lbcl(to_integer(cnt30));
            if ce_d = '1' and hv_c(31) = '1' then
                lbcl(to_integer(cnt30_d)) <= std_logic_vector(cw3(1));
            end if;
        end if;
    end process p_lbcl;

    p_lbink : process(clk)
    begin
        if rising_edge(clk) then
            lbink_q <= lbink(to_integer(cnt30_d));
            if ce_d = '1' and hv_c(32) = '1' then
                if C_HAS_CANNY then
                    lbink(to_integer(cnt30_dd)) <= std_logic_vector(ca_d(32)) & "0000" & t32 & inkq32(1);
                elsif C_HAS_XDOG then
                    lbink(to_integer(cnt30_dd)) <= std_logic_vector(xa_d(32)) & inkq32(7 downto 6) & "00" & t32 & inkq32(1);
                else
                    lbink(to_integer(cnt30_dd)) <= "000000" & t32 & inkq32(1);
                end if;
            end if;
        end if;
    end process p_lbink;

    g_shadow_ram : if C_HAS_SHADOW generate
        p_shr : process(clk)
        begin
            if rising_edge(clk) then
                shr_q <= shr(to_integer(sh_ra));
                if ce_d = '1' and hv_c(30) = '1' and cnt30(0) = '1' then
                    shr(to_integer(srow_w & cnt30(9 downto 1))) <= (0 => (sil_prev or (cw3(0)(7) or cw3(0)(6) or cw3(0)(5))));
                end if;
            end if;
        end process p_shr;
    end generate;

    g_tmp_ram : if C_HAS_TEMPORAL generate
        p_tmp : process(clk)
        begin
            if rising_edge(clk) then
                tmp_q <= tmpr(to_integer(tmp_ra));
                if ce_d = '1' and hv_c(30) = '1' and cnt30(5 downto 0) = 32 and aline(5 downto 0) = 32 then
                    tmpr(to_integer(aline(10 downto 6) & cnt30(9 downto 6))) <= std_logic_vector(cw3(0));
                end if;
            end if;
        end process p_tmp;
    end generate;

    -- histogram: pixel path samples during active video, sequencer owns it in
    -- vblank.  ONE read and ONE write statement (addresses/data muxed first),
    -- or yosys builds the table out of flip-flops.
    p_hist : process(clk)
        variable v_a  : unsigned(7 downto 0);
        variable v_we : std_logic;
        variable v_wd : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            if seq_hist = '1' then
                v_a := hist_sa; v_we := hist_swe; v_wd := (others => '0');
            else
                v_a := "00" & hist_ra; v_we := ce_d and hs_c(2); v_wd := hq20 + 1;
            end if;
            hist_q <= hist(to_integer(v_a));
            if v_we = '1' then hist(to_integer(v_a)) <= v_wd; end if;
        end if;
    end process p_hist;

    -- delay ring: {centre luma, chroma class} come back 12 H steps later
    -- (write and read pointers are 10 apart, never equal)
    p_dring : process(clk)
    begin
        if rising_edge(clk) then
            dr_q <= dring(to_integer(dr_w - 10));
            if ce = '1' then
                dring(to_integer(dr_w)) <= std_logic_vector(cen1) & "000" & std_logic_vector(cc0);
            end if;
        end if;
    end process p_dring;

    ------------------------------------------------------------------------
    -- H domain: main pipeline (every register advances on ce)
    ------------------------------------------------------------------------
    p_h : process(clk)
        variable v_cnt5 : t_cnt5;
        variable v_s5 : t_sym5;
        variable v_pick : integer range 0 to 4;
        variable v_lum : unsigned(2 downto 0);
        variable v_hue : unsigned(2 downto 0);
        variable v_t, v_tb : std_logic;
        variable v_msk : std_logic_vector(0 to 4);
        variable v_am : unsigned(5 downto 0);
        variable v_vs : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            if ce = '1' then
                ----------------------------------------------------------
                -- valid / first chains
                ----------------------------------------------------------
                hv_c(0) <= f0_avid and f0p_avid and f0_x(0);
                if f0_x = 1 then hf_c(0) <= '1'; else hf_c(0) <= '0'; end if;
                for i in 1 to C_NH loop
                    hv_c(i) <= hv_c(i - 1); hf_c(i) <= hf_c(i - 1);
                end loop;

                ----------------------------------------------------------
                -- H0: pair average of F0 (this pixel) and the previous one;
                -- column counter = LB read address
                ----------------------------------------------------------
                h0_y <= resize(shift_right(resize(f0_y(9 downto 2), 9) + resize(f0p_y8, 9) + 1, 1), 8);
                h0_u <= resize(shift_right(resize(f0_u(9 downto 2), 9) + resize(f0p_u8, 9) + 1, 1), 8);
                h0_v <= resize(shift_right(resize(f0_v(9 downto 2), 9) + resize(f0p_v8, 9) + 1, 1), 8);
                if (f0_avid and f0p_avid and f0_x(0)) = '1' then
                    if f0_x = 1 then hx <= (others => '0'); else hx <= hx + 1; end if;
                end if;

                ----------------------------------------------------------
                -- H1: newest luma column (rows r-4..r), chroma quarter sample
                ----------------------------------------------------------
                hx_d1 <= hx;
                ny(0) <= unsigned(lby_q4); ny(1) <= unsigned(lby_q3); ny(2) <= unsigned(lby_q2);
                ny(3) <= unsigned(lby_q1); ny(4) <= h0_y;
                h1_u <= h0_u; h1_v <= h0_v;
                cq_u <= resize(shift_right(resize(h0_u, 9) + resize(h1_u, 9) + 1, 1), 8);
                cq_v <= resize(shift_right(resize(h0_v, 9) + resize(h1_v, 9) + 1, 1), 8);
                cq_v1 <= hv_c(0) and hx(0);
                c1_r1 <= lbc_q1; c1_r2 <= lbc_q2;

                -- identity centre: stage 1, then the delay ring -> stage 13 -> cen13
                cen1 <= unsigned(lby_q2);
                dr_w <= dr_w + 1;
                dr_c <= dr_q;
                cen13 <= unsigned(dr_c(15 downto 8));

                ----------------------------------------------------------
                -- stage 15..18: smoother select and K2 blend -> ys18
                ----------------------------------------------------------
                case c_sm is
                    when 1 => s15_sm <= med14;
                    when 2 => s15_sm <= bil14;
                    when 3 => s15_sm <= kw14;
                    when others => s15_sm <= cen14;
                end case;
                s15_cen <= cen14;
                s16_d <= signed(resize(s15_sm, 9)) - signed(resize(s15_cen, 9));
                s16_cen <= s15_cen;
                -- K2 blend in quarters: centre + delta x {1/4, 1/2, 3/4, 1}
                case f_kb2 is
                    when "00"   => ys17 <= f_c8(signed(resize(s16_cen, 10)) + resize(shift_right(s16_d, 2), 10));
                    when "01"   => ys17 <= f_c8(signed(resize(s16_cen, 10)) + resize(shift_right(s16_d, 1), 10));
                    when "10"   => ys17 <= f_c8(signed(resize(s16_cen, 10)) + resize(s16_d, 10) - resize(shift_right(s16_d, 2), 10));
                    when others => ys17 <= f_c8(signed(resize(s16_cen, 10)) + resize(s16_d, 10));
                end case;
                ys18 <= ys17;

                ----------------------------------------------------------
                -- chroma: quarter-res 2(v) x 2(h) box (8 px x 2 lines at HD),
                -- hue sector (8, or 4 when coarse) and saturation level (3 or 2).
                -- No chroma hysteresis: the run-length and mode filters clean
                -- chroma specks (the near-boundary logic cost ~400 LC).
                ----------------------------------------------------------
                if hf_c(1) = '1' then c_fill <= '1'; end if;
                if hx_d1(0) = '1' then
                    v_vs := resize(unsigned(c1_r1(15 downto 8)), 9) + resize(unsigned(c1_r2(15 downto 8)), 9);
                    cvs_u <= v_vs;
                    cvs_v <= resize(unsigned(c1_r1(7 downto 0)), 9) + resize(unsigned(c1_r2(7 downto 0)), 9);
                    if c_fill = '1' then
                        cvp_u <= v_vs;
                        cvp_v <= resize(unsigned(c1_r1(7 downto 0)), 9) + resize(unsigned(c1_r2(7 downto 0)), 9);
                        c_fill <= '0';
                    else
                        cvp_u <= cvs_u; cvp_v <= cvs_v;
                    end if;
                end if;
                if c_sm = 0 then
                    us8 <= cvs_u(8 downto 1); vs8 <= cvs_v(8 downto 1);        -- Off: the 2-line average only
                else
                    us8 <= resize(shift_right(resize(cvs_u, 10) + resize(cvp_u, 10), 2), 8);
                    vs8 <= resize(shift_right(resize(cvs_v, 10) + resize(cvp_v, 10), 2), 8);
                end if;
                c4_du <= signed(resize(us8, 9)) - to_signed(128, 9);
                c4_dv <= signed(resize(vs8, 9)) - to_signed(128, 9);
                c4_au <= f_abs8(signed(resize(us8, 9)) - to_signed(128, 9));
                c4_av <= f_abs8(signed(resize(vs8, 9)) - to_signed(128, 9));
                -- octagonal saturation and octant bits
                c5_sat <= resize(f_umax(c4_au, c4_av), 9) + resize(shift_right(f_umin(c4_au, c4_av), 1), 9);
                c5_sdu <= c4_du(8); c5_sdv <= c4_dv(8);
                if c4_au > c4_av then c5_gt <= '1'; else c5_gt <= '0'; end if;
                -- sector: 0..7 counter-clockwise from +U (Cb)
                if c5_sdv = '0' and c5_sdu = '0' then
                    if c5_gt = '1' then v_hue := "000"; else v_hue := "001"; end if;
                elsif c5_sdv = '0' and c5_sdu = '1' then
                    if c5_gt = '0' then v_hue := "010"; else v_hue := "011"; end if;
                elsif c5_sdv = '1' and c5_sdu = '1' then
                    if c5_gt = '1' then v_hue := "100"; else v_hue := "101"; end if;
                else
                    if c5_gt = '0' then v_hue := "110"; else v_hue := "111"; end if;
                end if;
                if f_coarse /= "00" then v_hue(0) := '0'; end if;
                c6_hue <= v_hue;
                if f_coarse = "10" then
                    if c5_sat < C_SATC then c6_sat <= "00"; else c6_sat <= "10"; end if;
                else
                    if c5_sat < C_SAT1 then c6_sat <= "00"; elsif c5_sat < C_SAT2 then c6_sat <= "01"; else c6_sat <= "10"; end if;
                end if;
                -- class (hue, sat) of quarter column q lands at step 2q+8; delayed
                -- 12 more steps (delay ring) it meets the luma of half-res column m
                -- (q = m/2) at stage 20
                cc0 <= c6_hue & c6_sat;

                ----------------------------------------------------------
                -- stage 18: histogram sample + temporal read address
                ----------------------------------------------------------
                if hv_c(17) = '1' then
                    if hf_c(17) = '1' then cnt18 <= (others => '0'); else cnt18 <= cnt18 + 1; end if;
                end if;
                hs_c(0) <= '0';
                if hv_c(18) = '1' and cnt18(2 downto 0) = "000" then
                    hist_ra <= ys18(7 downto 2); hs_c(0) <= '1';
                end if;
                hs_c(1) <= hs_c(0); hs_c(2) <= hs_c(1);
                hq20 <= hist_q;
                tmp_ra <= aline(10 downto 6) & cnt18(9 downto 6);
                tmp_c <= tmp_q;

                ----------------------------------------------------------
                -- stage 19..21: luma bins with hysteresis -> label
                ----------------------------------------------------------
                -- bin = count of thresholds passed; the +-C_HYST band gives the
                -- candidate bins for the temporal and the left-neighbour vote
                for k in 1 to 7 loop
                    if signed(resize(ys18, 10)) > signed(resize(thr(k), 10)) then q19_h(k - 1) <= '1'; else q19_h(k - 1) <= '0'; end if;
                    if signed(resize(ys18, 10)) - C_HYST > signed(resize(thr(k), 10)) then q19_lo3(k - 1) <= '1'; else q19_lo3(k - 1) <= '0'; end if;
                    if signed(resize(ys18, 10)) + C_HYST > signed(resize(thr(k), 10)) then q19_hi3(k - 1) <= '1'; else q19_hi3(k - 1) <= '0'; end if;
                end loop;
                q20_bh <= f_pop7(q19_h);
                q20_blo3 <= f_pop7(q19_lo3); q20_bhi3 <= f_pop7(q19_hi3);
                v_lum := q20_bh;
                if f_temp = '1' and q20_blo3 /= q20_bhi3
                   and unsigned(tmp_c(7 downto 5)) >= q20_blo3 and unsigned(tmp_c(7 downto 5)) <= q20_bhi3 then
                    v_lum := unsigned(tmp_c(7 downto 5));
                elsif q20_blo3 /= q20_bhi3 and lab21(7 downto 5) >= q20_blo3 and lab21(7 downto 5) <= q20_bhi3 then
                    v_lum := lab21(7 downto 5);
                end if;
                lab21 <= v_lum & unsigned(dr_c(4 downto 0));

                ----------------------------------------------------------
                -- stage 22..26: horizontal 5-sample mode per component.  Always
                -- on: it is the minimum-run-length filter (a label survives only
                -- with 3 of the 5 samples around it).
                ----------------------------------------------------------
                if hf_c(21) = '1' then
                    for i in 0 to 5 loop mw(i) <= lab21; end loop;
                else
                    mw(0) <= lab21;
                    for i in 1 to 5 loop mw(i) <= mw(i - 1); end loop;
                end if;
                for i in 0 to 4 loop v_s5(i) := mw(i)(7 downto 5); end loop;
                mh_cl <= f_cnt5(v_s5);
                for i in 0 to 4 loop v_s5(i) := mw(i)(4 downto 2); end loop;
                mh_ch <= f_cnt5(v_s5);
                for i in 0 to 4 loop v_s5(i) := '0' & mw(i)(1 downto 0); end loop;
                mh_cs <= f_cnt5(v_s5);
                -- the counts were taken one step ago, so the sample picked at
                -- index p now sits at mw(p + 1)
                rm26 <= mw(f_pick5(mh_cl) + 1)(7 downto 5) & mw(f_pick5(mh_ch) + 1)(4 downto 2) & mw(f_pick5(mh_cs) + 1)(1 downto 0);
                if hv_c(25) = '1' then
                    if hf_c(25) = '1' then cnt26 <= (others => '0'); else cnt26 <= cnt26 + 1; end if;
                end if;
                cnt26_d <= cnt26;

                ----------------------------------------------------------
                -- stage 27..29: vertical 5-row mode of the row-modes (S9) -> cl29
                ----------------------------------------------------------
                mv(0) <= rm26; mv(1) <= unsigned(lbl_q1); mv(2) <= unsigned(lbl_q2);
                mv(3) <= unsigned(lbl_q3); mv(4) <= unsigned(lbl_q4);
                for i in 0 to 4 loop v_s5(i) := mv(i)(7 downto 5); end loop;
                mv_cl <= f_cnt5(v_s5);
                for i in 0 to 4 loop v_s5(i) := mv(i)(4 downto 2); end loop;
                mv_ch <= f_cnt5(v_s5);
                for i in 0 to 4 loop v_s5(i) := '0' & mv(i)(1 downto 0); end loop;
                mv_cs <= f_cnt5(v_s5);
                mvc <= mv;
                v_pick := f_pick5(mv_cl);
                if s_clean = '1' then
                    cl29 <= mvc(v_pick)(7 downto 5) & mvc(f_pick5(mv_ch))(4 downto 2) & mvc(f_pick5(mv_cs))(1 downto 0);
                    cl29_pick <= v_pick;
                else
                    cl29 <= mvc(2);
                    cl29_pick <= 2;
                end if;
                cl29_cl <= mv_cl;
                for i in 0 to 4 loop cl29_l(i) <= mvc(i)(7 downto 5); end loop;

                ----------------------------------------------------------
                -- stage 30..35: matte luma. With soft quantization built: blend
                -- toward the runner-up bin (luma). Otherwise: the bin centroid.
                ----------------------------------------------------------
                if C_HAS_SOFT then
                    sf30_cl <= cl29;
                    sf30_nm <= cl29_cl(cl29_pick) + 1;
                    -- runner-up: the best count among samples whose bin differs from the mode
                    for i in 0 to 4 loop
                        if cl29_l(i) /= cl29(7 downto 5) then v_msk(i) := '1'; else v_msk(i) := '0'; end if;
                    end loop;
                    v_am := f_argmax5(cl29_cl, v_msk);
                    sf30_n2 <= v_am(2 downto 0);
                    sf30_lum2 <= cl29_l(to_integer(v_am(5 downto 3)));
                    -- weight = n2 / (nm + n2), in sixteenths
                    if s_soft = '1' then
                        case to_integer(sf30_nm & sf30_n2) is
                            when 4*8 + 1 => sf31_w <= to_unsigned(3, 4);   -- 4,1 -> 0.2
                            when 3*8 + 2 => sf31_w <= to_unsigned(6, 4);   -- 3,2 -> 0.4
                            when 3*8 + 1 => sf31_w <= to_unsigned(4, 4);   -- 3,1 -> 0.25
                            when 2*8 + 2 => sf31_w <= to_unsigned(8, 4);   -- 2,2 -> 0.5
                            when 2*8 + 1 => sf31_w <= to_unsigned(5, 4);   -- 2,1 -> 0.33
                            when 1*8 + 1 => sf31_w <= to_unsigned(8, 4);
                            when others  => sf31_w <= (others => '0');
                        end case;
                    else
                        sf31_w <= (others => '0');
                    end if;
                    sf31_d <= signed(resize(cen(to_integer(sf30_lum2)), 9)) - signed(resize(cen(to_integer(sf30_cl(7 downto 5))), 9));
                    sf31_c <= cen(to_integer(sf30_cl(7 downto 5)));
                    sf31_hs <= sf30_cl(4 downto 0); sf31_lum <= sf30_cl(7 downto 5);
                    sf32_p <= sf31_d * signed('0' & sf31_w);
                    sf32_c <= sf31_c; sf32_hs <= sf31_hs; sf32_lum <= sf31_lum;
                    ym33 <= f_c8(signed(resize(sf32_c, 10)) + resize(shift_right(sf32_p, 4), 10));
                    sf33_hs <= sf32_hs; sf33_lum <= sf32_lum;
                    ym34 <= ym33; sf34_hs <= sf33_hs; sf34_lum <= sf33_lum;
                    ym35 <= ym34; sf35_hs <= sf34_hs; sf35_lum <= sf34_lum;
                else
                    -- plain delay of the cleaned label 29 -> 34, centroid lookup at 35
                    ld(30) <= cl29;
                    for i in 31 to 34 loop ld(i) <= ld(i - 1); end loop;
                    ym35 <= cen(to_integer(ld(34)(7 downto 5)));
                    sf35_hs <= ld(34)(4 downto 0); sf35_lum <= ld(34)(7 downto 5);
                end if;

                ----------------------------------------------------------
                -- stage 30..32: region boundary; ink history RAM
                ----------------------------------------------------------
                cw3(0) <= cl29; cw3(1) <= cw3(0); cw3(2) <= cw3(1);
                if hf_c(29) = '1' then cw3(1) <= cl29; cw3(2) <= cl29; end if;
                if hv_c(29) = '1' then
                    if hf_c(29) = '1' then cnt30 <= (others => '0'); else cnt30 <= cnt30 + 1; end if;
                end if;
                cnt30_d <= cnt30; cnt30_dd <= cnt30_d;
                cla31 <= unsigned(lbcl_q);
                -- (cw3 at step v+31 holds (v+1, v, v-1); cla31 = above(v))
                if cw3(1) /= cw3(2) or cw3(1) /= cla31 then t32 <= '1'; else t32 <= '0'; end if;
                if cw3(1) /= cw3(0) then dr32 <= '1'; else dr32 <= '0'; end if;
                inkq32 <= lbink_q;
                -- 3-wide windows for the footprint (centre = sr(1))
                srt(0) <= t32; srt(1) <= srt(0); srt(2) <= srt(1);
                srdr(0) <= dr32; srdr(1) <= srdr(0); srdr(2) <= srdr(1);
                srt1(0) <= inkq32(1); srt1(1) <= srt1(0); srt1(2) <= srt1(1);
                srt2(0) <= inkq32(0); srt2(1) <= srt2(0); srt2(2) <= srt2(1);
                -- XDoG: the field two lines old (5:4); Canny: one line old (7:6)
                if C_HAS_CANNY then srxa(0) <= unsigned(inkq32(7 downto 6)); else srxa(0) <= unsigned(inkq32(5 downto 4)); end if;
                srxa(1) <= srxa(0); srxa(2) <= srxa(1);
                if hf_c(32) = '1' then
                    srt(1) <= t32; srt(2) <= t32; srdr(1) <= dr32; srdr(2) <= dr32;
                    srt1(1) <= inkq32(1); srt1(2) <= inkq32(1); srt2(1) <= inkq32(0); srt2(2) <= inkq32(0);
                end if;
                -- stage 35: line bit by footprint -> ink alpha
                case c_width is
                    when "00"   => v_t := srt(1);
                    when "01"   => v_t := srt(1) or srt(2) or srt1(1) or srdr(1);
                    when others => v_t := srt(0) or srt(1) or srt(2) or srdr(0) or srdr(1) or srdr(2)
                                          or srt1(0) or srt1(1) or srt1(2) or srt2(0) or srt2(1) or srt2(2);
                end case;
                v_tb := srt(0) or srt(1) or srt(2) or srdr(0) or srdr(1) or srdr(2)
                        or srt1(0) or srt1(1) or srt1(2) or srt2(0) or srt2(1) or srt2(2);
                case c_lmode is
                    when "01" =>
                        if v_t = '1' then ia35 <= c_inkw; else ia35 <= (others => '0'); end if;
                    when "10" =>
                        ia35 <= srxa(1);
                    when "11" =>                                       -- bold edge: 3x3, full ink
                        if v_tb = '1' then ia35 <= "11"; else ia35 <= (others => '0'); end if;
                    when others =>
                        ia35 <= (others => '0');
                end case;
                if hv_c(36) = '1' then
                    if hf_c(36) = '1' then cnt37h <= (others => '0'); else cnt37h <= cnt37h + 1; end if;
                end if;

                ----------------------------------------------------------
                -- shadow: silhouette written at stage 30 (quarter res, OR of
                -- the pair), read at (row - dy, col - dx + 4), 16-bit soften
                ----------------------------------------------------------
                sil_prev <= cw3(0)(7) or cw3(0)(6) or cw3(0)(5);
                if hv_c(30) = '1' and hv_c(29) = '0' then srow_w <= srow_w + 1; end if;   -- line end at stage 30
                if hv_c(33) = '1' and hv_c(32) = '0' then
                    srow_r <= srow_r + 1;
                    if sline_r /= 31 then sline_r <= sline_r + 1; end if;
                end if;
                if hv_c(32) = '1' then
                    if hf_c(32) = '1' then cnt33s <= (others => '0'); else cnt33s <= cnt33s + 1; end if;
                end if;
                sh_ra <= (srow_r - f_dy) & (resize(cnt33s(9 downto 1), 9) + 6 - resize(f_dxq, 9));
                if resize(cnt33s(9 downto 1), 9) + 6 < resize(f_dxq, 9) or sline_r < resize(f_dy, 5) or f_dy = 0 then
                    sh_oob <= '1';
                else sh_oob <= '0'; end if;
                if sh_oob = '1' then sw(0) <= '0'; else sw(0) <= shr_q(0); end if;
                for i in 1 to 16 loop sw(i) <= sw(i - 1); end loop;
                -- soften: popcount over 2 / 4 / 8 / 16 samples centred on sw(9), scaled to
                -- 0..16. Pipelined as group counts -> pair sums -> window sums -> select.
                shc_a <= f_cnt2(sw(8), sw(9));                            -- window 2
                shc_b <= f_cnt2(sw(7), sw(10));                           -- + -> window 4
                shc_c <= f_cnt4(sw(5) & sw(6) & sw(11) & sw(12));         -- + -> window 8
                shc_d <= f_cnt4(sw(1) & sw(2) & sw(3) & sw(4));           -- + -> window 16
                shc_e <= f_cnt4(sw(13) & sw(14) & sw(15) & sw(16));
                shs_2 <= shc_a;
                shs_4 <= resize(shc_a, 3) + resize(shc_b, 3);
                shs_c <= shc_c;
                shs_de <= resize(shc_d, 4) + resize(shc_e, 4);
                sht_2 <= shs_2;
                sht_4 <= shs_4;
                sht_8 <= resize(shs_4, 4) + resize(shs_c, 4);
                sht_de <= shs_de;
                case c_shsoft is
                    when "00"   => sha36 <= shift_left(resize(sht_2, 5), 3);
                    when "01"   => sha36 <= shift_left(resize(sht_4, 5), 2);
                    when "10"   => sha36 <= shift_left(resize(sht_8, 5), 1);
                    when others => sha36 <= resize(sht_8, 5) + resize(sht_de, 5);
                end case;
                sha37 <= sha36;

                ----------------------------------------------------------
                -- hand-off: stage 36 (next) and 37 (current), 38 (previous)
                ----------------------------------------------------------
                ho_n_y <= ym35; ho_n_hs <= sf35_hs; ho_n_lum <= sf35_lum; ho_n_a <= ia35; ho_n_v <= hv_c(35);
                ho_c_y <= ho_n_y; ho_c_hs <= ho_n_hs; ho_c_lum <= ho_n_lum; ho_c_a <= ho_n_a; ho_c_v <= ho_n_v;
                ho_p_y <= ho_c_y;
            end if;

            -- frame-start housekeeping (vblank: streams are idle)
            if vs_edge = '1' then
                srow_w <= (others => '0'); srow_r <= (others => '0'); sline_r <= (others => '0');
            end if;
        end if;
    end process p_h;

    ------------------------------------------------------------------------
    -- median 3x3: sorted-column network (19 compare-exchanges)
    ------------------------------------------------------------------------
    g_median : if C_HAS_MEDIAN generate
        signal m2_a, m2_b, m2_c : unsigned(7 downto 0) := (others => '0');
        signal m3_a, m3_b, m3_c : unsigned(7 downto 0) := (others => '0');
        signal m4_lo, m4_mid, m4_hi : unsigned(7 downto 0) := (others => '0');
        type t_tri is array (0 to 2) of unsigned(7 downto 0);
        signal sc_lo, sc_mid, sc_hi : t_tri := (others => (others => '0'));
        signal m7_mxlo, m7_mnhi, m7_plo, m7_phi, m7_mid2 : unsigned(7 downto 0) := (others => '0');
        signal m8_mxlo, m8_mnhi, m8_plo, m8_t : unsigned(7 downto 0) := (others => '0');
        signal m9_mxlo, m9_mnhi, m9_mm : unsigned(7 downto 0) := (others => '0');
        signal m10_v1, m10_v2, m10_mnhi : unsigned(7 downto 0) := (others => '0');
        signal m11_med : unsigned(7 downto 0) := (others => '0');
        type t_md is array (12 to 14) of unsigned(7 downto 0);
        signal md : t_md := (others => (others => '0'));
    begin
        p_med : process(clk)
        begin
            if rising_edge(clk) then
                if ce = '1' then
                    -- stage 2..4: sort the newest column rows r-3, r-2, r-1
                    m2_a <= f_umin(ny(1), ny(2)); m2_b <= f_umax(ny(1), ny(2)); m2_c <= ny(3);
                    m3_a <= m2_a; m3_b <= f_umin(m2_b, m2_c); m3_c <= f_umax(m2_b, m2_c);
                    m4_lo <= f_umin(m3_a, m3_b); m4_mid <= f_umax(m3_a, m3_b); m4_hi <= m3_c;
                    -- stage 5..: sorted columns (0 newest); centre column = sc(1)
                    sc_lo(0) <= m4_lo; sc_mid(0) <= m4_mid; sc_hi(0) <= m4_hi;
                    for i in 1 to 2 loop
                        sc_lo(i) <= sc_lo(i - 1); sc_mid(i) <= sc_mid(i - 1); sc_hi(i) <= sc_hi(i - 1);
                    end loop;
                    if hf_c(4) = '1' then
                        for i in 1 to 2 loop sc_lo(i) <= m4_lo; sc_mid(i) <= m4_mid; sc_hi(i) <= m4_hi; end loop;
                    end if;
                    -- window complete for centre u at step u+6 -> stage 7
                    m7_mxlo <= f_umax(f_umax(sc_lo(0), sc_lo(1)), sc_lo(2));
                    m7_mnhi <= f_umin(f_umin(sc_hi(0), sc_hi(1)), sc_hi(2));
                    m7_plo <= f_umin(sc_mid(0), sc_mid(1)); m7_phi <= f_umax(sc_mid(0), sc_mid(1)); m7_mid2 <= sc_mid(2);
                    m8_mxlo <= m7_mxlo; m8_mnhi <= m7_mnhi; m8_plo <= m7_plo; m8_t <= f_umin(m7_phi, m7_mid2);
                    m9_mxlo <= m8_mxlo; m9_mnhi <= m8_mnhi; m9_mm <= f_umax(m8_plo, m8_t);
                    m10_v1 <= f_umin(m9_mxlo, m9_mm); m10_v2 <= f_umax(m9_mxlo, m9_mm); m10_mnhi <= m9_mnhi;
                    m11_med <= f_umax(m10_v1, f_umin(m10_v2, m10_mnhi));
                    md(12) <= m11_med; md(13) <= md(12); md(14) <= md(13);
                end if;
            end if;
        end process p_med;
        med14 <= md(14);
    end generate;
    g_nomedian : if not C_HAS_MEDIAN generate
        med14 <= cen14;
    end generate;

    ------------------------------------------------------------------------
    -- bilateral 5x5, separable: vertical on the newest column, then
    -- horizontal on the vertical results. Range weights {8,4,2,0} at
    -- |d| <= s1, s2, s3 (distance-2 taps halved); out = c + sum/32.
    ------------------------------------------------------------------------
    g_bilat : if C_HAS_BILAT generate
        -- Stage plan (H steps; "stage k holds column m during step m+k"):
        --   vertical   : 2 difference, 3 range level, 4 weighted sum, 5 output, 6..7 delay
        --   horizontal : 8 window in, 10 window centre, 11 difference, 12 range level,
        --                13 weighted sum, 14 output (bil14)
        -- The difference and the |d| -> level test are separate stages on purpose: merged,
        -- they were the critical path of every HD config (sub + abs + barrel = 4.7 ns of
        -- logic plus 9 ns of routing at 86% utilisation; 70-72 MHz on router1, all seeds).
        type t_d4 is array (0 to 3) of signed(8 downto 0);
        type t_l4 is array (0 to 3) of unsigned(1 downto 0);
        signal v2_d, v3_d : t_d4 := (others => (others => '0'));
        signal v3_l : t_l4 := (others => (others => '0'));
        signal v2_c, v3_c, v4_c : unsigned(7 downto 0) := (others => '0');
        signal v4_s : signed(13 downto 0) := (others => '0');
        signal v5_o, v6_o, v7_o : unsigned(7 downto 0) := (others => '0');
        type t_w5 is array (0 to 4) of unsigned(7 downto 0);
        signal hw : t_w5 := (others => (others => '0'));
        signal h11_d, h12_d : t_d4 := (others => (others => '0'));
        signal h12_l : t_l4 := (others => (others => '0'));
        signal h11_c, h12_c, h13_c : unsigned(7 downto 0) := (others => '0');
        signal h13_s : signed(13 downto 0) := (others => '0');
        -- private copies of the sigma shift, one per pass (each drives 4 barrel shifters);
        -- loaded under different enables so yosys cannot merge them back into one FF
        signal sig_v, sig_h : unsigned(2 downto 0) := "011";
        attribute keep of sig_v : signal is true;
        attribute keep of sig_h : signal is true;
        -- range level from |d| against sigma = 2^(sh+1): 3 = within sigma, 2 = within
        -- 2 sigma, 1 = within 4 sigma, 0 = beyond (bit tests, no compare chains)
        function f_lvl(d : signed(8 downto 0); sh : unsigned(2 downto 0)) return unsigned is
            variable a : unsigned(8 downto 0);
            variable t : unsigned(8 downto 0);
        begin
            a := resize(f_abs8(d), 9);
            t := shift_right(a, to_integer(sh) + 1);      -- |d| / sigma
            if t = 0 then return "11";
            elsif t = 1 then return "10";
            elsif t(8 downto 2) = 0 then return "01";
            else return "00"; end if;
        end function;
        -- inner taps: weight 2^lvl (lvl 1..3), outer: 2^(lvl-1); zero at lvl 0
        function f_term(d : signed(8 downto 0); l : unsigned(1 downto 0); inner : boolean) return signed is
            variable r : signed(12 downto 0);
        begin
            r := resize(d, 13);
            case l is
                when "00" => return to_signed(0, 13);
                when "01" => if inner then return shift_left(r, 1); else return r; end if;
                when "10" => if inner then return shift_left(r, 2); else return shift_left(r, 1); end if;
                when others => if inner then return shift_left(r, 3); else return shift_left(r, 2); end if;
            end case;
        end function;
    begin
        p_bil : process(clk)
        begin
            if rising_edge(clk) then
                if ce = '1' then sig_v <= c_sigsh; else sig_h <= c_sigsh; end if;
                if ce = '1' then
                    -- vertical pass over the 5-row window ny(0..4), centre ny(2)
                    v2_d(0) <= signed(resize(ny(0), 9)) - signed(resize(ny(2), 9));
                    v2_d(1) <= signed(resize(ny(1), 9)) - signed(resize(ny(2), 9));
                    v2_d(2) <= signed(resize(ny(3), 9)) - signed(resize(ny(2), 9));
                    v2_d(3) <= signed(resize(ny(4), 9)) - signed(resize(ny(2), 9));
                    v2_c <= ny(2);
                    for i in 0 to 3 loop
                        v3_d(i) <= v2_d(i);
                        v3_l(i) <= f_lvl(v2_d(i), sig_v);
                    end loop;
                    v3_c <= v2_c;
                    v4_s <= resize(f_term(v3_d(0), v3_l(0), false) + f_term(v3_d(3), v3_l(3), false), 14)
                            + resize(f_term(v3_d(1), v3_l(1), true) + f_term(v3_d(2), v3_l(2), true), 14);
                    v4_c <= v3_c;
                    v5_o <= f_c8(signed(resize(v4_c, 11)) + resize(shift_right(v4_s + 16, 5), 11));
                    v6_o <= v5_o;
                    v7_o <= v6_o;
                    -- horizontal pass: 5-wide window, left edge replicated on the first column
                    hw(0) <= v7_o;
                    for i in 1 to 4 loop hw(i) <= hw(i - 1); end loop;
                    if hf_c(7) = '1' then
                        for i in 1 to 4 loop hw(i) <= v7_o; end loop;
                    end if;
                    h11_d(0) <= signed(resize(hw(0), 9)) - signed(resize(hw(2), 9));
                    h11_d(1) <= signed(resize(hw(1), 9)) - signed(resize(hw(2), 9));
                    h11_d(2) <= signed(resize(hw(3), 9)) - signed(resize(hw(2), 9));
                    h11_d(3) <= signed(resize(hw(4), 9)) - signed(resize(hw(2), 9));
                    h11_c <= hw(2);
                    for i in 0 to 3 loop
                        h12_d(i) <= h11_d(i);
                        h12_l(i) <= f_lvl(h11_d(i), sig_h);
                    end loop;
                    h12_c <= h11_c;
                    h13_s <= resize(f_term(h12_d(0), h12_l(0), false) + f_term(h12_d(3), h12_l(3), false), 14)
                             + resize(f_term(h12_d(1), h12_l(1), true) + f_term(h12_d(2), h12_l(2), true), 14);
                    h13_c <= h12_c;
                    bil14 <= f_c8(signed(resize(h13_c, 11)) + resize(shift_right(h13_s + 16, 5), 11));
                    bil_vc14 <= h13_c;
                end if;
            end if;
        end process p_bil;
        cen14 <= bil_vc14;
    end generate;
    g_nobilat : if not C_HAS_BILAT generate
        bil14 <= cen14;
        cen14 <= cen13;
    end generate;

    ------------------------------------------------------------------------
    -- Kuwahara 5x5 (range criterion): four 3x3 quadrants from column stats
    ------------------------------------------------------------------------
    g_kuwa : if C_HAS_KUWA generate
        type t_cs is record
            ts, bs : unsigned(9 downto 0);
            tmx, tmn, bmx, bmn : unsigned(7 downto 0);
        end record;
        constant C_CS0 : t_cs := ((others => '0'), (others => '0'), (others => '0'), (others => '0'), (others => '0'), (others => '0'));
        type t_cs5 is array (0 to 4) of t_cs;
        signal k2_ts, k2_bs : unsigned(9 downto 0) := (others => '0');
        signal k2_tm, k2_tn, k2_bm, k2_bn, k2_y2 : unsigned(7 downto 0) := (others => '0');
        signal kst3 : t_cs := C_CS0;
        signal cs : t_cs5 := (others => C_CS0);
        signal k8_stl, k8_str, k8_sbl, k8_sbr : unsigned(11 downto 0) := (others => '0');
        signal k8_mtl, k8_ntl, k8_mtr, k8_ntr, k8_mbl, k8_nbl, k8_mbr, k8_nbr : unsigned(7 downto 0) := (others => '0');
        signal k9_rtl, k9_rtr, k9_rbl, k9_rbr : unsigned(7 downto 0) := (others => '0');
        signal k9_utl, k9_utr, k9_ubl, k9_ubr : unsigned(7 downto 0) := (others => '0');
        signal k10 : unsigned(7 downto 0) := (others => '0');
        type t_kd is array (10 to 14) of unsigned(7 downto 0);
        signal kd : t_kd := (others => (others => '0'));
        -- mean of 9 as (7 s + 32) / 64 = s / 9.14 (1.6% low, one subtractor);
        -- x57 >> 9 was two 19-bit adders per quadrant
        function f_mean9(s : unsigned(11 downto 0)) return unsigned is
            variable p : unsigned(15 downto 0);
        begin
            p := shift_left(resize(s, 16), 3) - resize(s, 16) + 32;
            return p(13 downto 6);
        end function;
    begin
        p_kuwa : process(clk)
            variable v_a, v_b, v_c, v_d : boolean;
        begin
            if rising_edge(clk) then
                if ce = '1' then
                    -- stage 2..3: column statistics (top rows 0..2, bottom rows 2..4)
                    k2_ts <= resize(ny(0), 10) + resize(ny(1), 10) + resize(ny(2), 10);
                    k2_bs <= resize(ny(2), 10) + resize(ny(3), 10) + resize(ny(4), 10);
                    k2_tm <= f_umax(ny(0), ny(1)); k2_tn <= f_umin(ny(0), ny(1));
                    k2_bm <= f_umax(ny(3), ny(4)); k2_bn <= f_umin(ny(3), ny(4));
                    k2_y2 <= ny(2);
                    kst3.ts <= k2_ts; kst3.bs <= k2_bs;
                    kst3.tmx <= f_umax(k2_tm, k2_y2); kst3.tmn <= f_umin(k2_tn, k2_y2);
                    kst3.bmx <= f_umax(k2_bm, k2_y2); kst3.bmn <= f_umin(k2_bn, k2_y2);
                    -- stage 4..: 5 columns of stats (0 newest); centre = cs(2)
                    cs(0) <= kst3;
                    for i in 1 to 4 loop cs(i) <= cs(i - 1); end loop;
                    if hf_c(3) = '1' then
                        for i in 1 to 4 loop cs(i) <= kst3; end loop;
                    end if;
                    -- stage 7: quadrant sums and max/min of three columns (two compare levels)
                    k8_stl <= resize(cs(2).ts, 12) + resize(cs(3).ts, 12) + resize(cs(4).ts, 12);
                    k8_str <= resize(cs(0).ts, 12) + resize(cs(1).ts, 12) + resize(cs(2).ts, 12);
                    k8_sbl <= resize(cs(2).bs, 12) + resize(cs(3).bs, 12) + resize(cs(4).bs, 12);
                    k8_sbr <= resize(cs(0).bs, 12) + resize(cs(1).bs, 12) + resize(cs(2).bs, 12);
                    k8_mtl <= f_umax(f_umax(cs(3).tmx, cs(4).tmx), cs(2).tmx); k8_ntl <= f_umin(f_umin(cs(3).tmn, cs(4).tmn), cs(2).tmn);
                    k8_mtr <= f_umax(f_umax(cs(0).tmx, cs(1).tmx), cs(2).tmx); k8_ntr <= f_umin(f_umin(cs(0).tmn, cs(1).tmn), cs(2).tmn);
                    k8_mbl <= f_umax(f_umax(cs(3).bmx, cs(4).bmx), cs(2).bmx); k8_nbl <= f_umin(f_umin(cs(3).bmn, cs(4).bmn), cs(2).bmn);
                    k8_mbr <= f_umax(f_umax(cs(0).bmx, cs(1).bmx), cs(2).bmx); k8_nbr <= f_umin(f_umin(cs(0).bmn, cs(1).bmn), cs(2).bmn);
                    -- stage 8: ranges and means
                    k9_rtl <= k8_mtl - k8_ntl; k9_rtr <= k8_mtr - k8_ntr; k9_rbl <= k8_mbl - k8_nbl; k9_rbr <= k8_mbr - k8_nbr;
                    k9_utl <= f_mean9(k8_stl); k9_utr <= f_mean9(k8_str); k9_ubl <= f_mean9(k8_sbl); k9_ubr <= f_mean9(k8_sbr);
                    -- stage 9: lowest range wins (ties: TL, TR, BL, BR)
                    v_a := k9_rtr < k9_rtl; v_b := k9_rbl < k9_rtl; v_c := k9_rbr < k9_rtl;
                    if not (v_a or v_b or v_c) then
                        k10 <= k9_utl;
                    elsif v_a and not (k9_rbl < k9_rtr) and not (k9_rbr < k9_rtr) then
                        k10 <= k9_utr;
                    elsif v_b and not (k9_rbr < k9_rbl) and not (k9_rtr < k9_rbl) then
                        k10 <= k9_ubl;
                    else
                        k10 <= k9_ubr;
                    end if;
                    kd(10) <= k10;
                    for i in 11 to 14 loop kd(i) <= kd(i - 1); end loop;
                end if;
            end if;
        end process p_kuwa;
        kw14 <= kd(14);
    end generate;
    g_nokuwa : if not C_HAS_KUWA generate
        kw14 <= cen14;
    end generate;

    ------------------------------------------------------------------------
    -- XDoG on the raw window: 3x3 vs 5x5 binomial, tau = 63/64,
    -- alpha = clamp((eps - D) / 2 codes), 3 bits, at stage 10
    ------------------------------------------------------------------------
    g_xdog : if C_HAS_XDOG generate
        signal x2_a, x2_b : unsigned(8 downto 0) := (others => '0');
        signal x2_c3 : unsigned(9 downto 0) := (others => '0');
        signal x2_y2 : unsigned(7 downto 0) := (others => '0');
        signal x3_c5 : unsigned(11 downto 0) := (others => '0');
        type t_c3s is array (0 to 2) of unsigned(9 downto 0);
        type t_c5s is array (0 to 4) of unsigned(11 downto 0);
        signal c3s : t_c3s := (others => (others => '0'));
        signal c5s : t_c5s := (others => (others => '0'));
        signal x5_g1, x6_g1, x7_g1, x8_g1 : unsigned(11 downto 0) := (others => '0');
        signal x7_a, x7_b : unsigned(12 downto 0) := (others => '0');
        signal x7_c : unsigned(11 downto 0) := (others => '0');
        signal x8_g2 : unsigned(15 downto 0) := (others => '0');
        signal x9_d : signed(17 downto 0) := (others => '0');
    begin
        p_xdog : process(clk)
            variable v_t : signed(17 downto 0);
        begin
            if rising_edge(clk) then
                if ce = '1' then
                    x2_a <= resize(ny(0), 9) + resize(ny(4), 9);
                    x2_b <= resize(ny(1), 9) + resize(ny(3), 9);
                    x2_c3 <= resize(ny(1), 10) + resize(ny(3), 10) + shift_left(resize(ny(2), 10), 1);
                    x2_y2 <= ny(2);
                    x3_c5 <= resize(x2_a, 12) + shift_left(resize(x2_b, 12), 2) + shift_left(resize(x2_y2, 12), 2) + shift_left(resize(x2_y2, 12), 1);
                    c3s(0) <= x2_c3; c3s(1) <= c3s(0); c3s(2) <= c3s(1);
                    if hf_c(2) = '1' then c3s(1) <= x2_c3; c3s(2) <= x2_c3; end if;
                    c5s(0) <= x3_c5;
                    for i in 1 to 4 loop c5s(i) <= c5s(i - 1); end loop;
                    if hf_c(3) = '1' then for i in 1 to 4 loop c5s(i) <= x3_c5; end loop; end if;
                    -- c3 window centre u at step u+4 -> g1 at 5; c5 centre at u+6 -> g2 at 8
                    x5_g1 <= resize(c3s(0), 12) + shift_left(resize(c3s(1), 12), 1) + resize(c3s(2), 12);
                    x6_g1 <= x5_g1; x7_g1 <= x6_g1; x8_g1 <= x7_g1;
                    x7_a <= resize(c5s(0), 13) + resize(c5s(4), 13);
                    x7_b <= resize(c5s(1), 13) + resize(c5s(3), 13);
                    x7_c <= c5s(2);
                    x8_g2 <= resize(x7_a, 16) + shift_left(resize(x7_b, 16), 2) + shift_left(resize(x7_c, 16), 2) + shift_left(resize(x7_c, 16), 1);
                    -- D in 1/256 codes: 16 g1 - g2 + g2/64
                    x9_d <= shift_left(signed(resize(x8_g1, 18)), 4) - signed(resize(x8_g2, 18)) + signed(resize(shift_right(x8_g2, 6), 18));
                    v_t := shift_left(signed(resize(c_eps8, 18)), 8) - x9_d;
                    v_t := shift_right(v_t, 10);                 -- 4 codes per alpha step
                    if v_t <= 0 then xa10 <= (others => '0');
                    elsif v_t >= 3 then xa10 <= "11";
                    else xa10 <= unsigned(v_t(1 downto 0)); end if;
                    xa_d(11) <= xa10;
                    for i in 12 to 32 loop xa_d(i) <= xa_d(i - 1); end loop;
                end if;
            end if;
        end process p_xdog;
    end generate;

    ------------------------------------------------------------------------
    -- Canny-lite (build C): Sobel (luma, + chroma difference when S10), 4-bit
    -- magnitude rows in one RAM, NMS along the quantized direction, hysteresis
    -- from the left and the row above. Alpha (3 bits) at stage 13, delayed one
    -- line through the ink RAM to land on the matte row.
    ------------------------------------------------------------------------
    g_canny : if C_HAS_CANNY generate
        type t_lbm is array (0 to 1023) of std_logic_vector(7 downto 0);
        type t_lbd is array (0 to 1023) of std_logic_vector(3 downto 0);
        signal lbmag : t_lbm := (others => (others => '0'));
        signal lbdir : t_lbd := (others => (others => '0'));
        attribute no_rw_check of lbmag : signal is true;
        attribute no_rw_check of lbdir : signal is true;
        signal lbmag_q : std_logic_vector(7 downto 0) := (others => '0');
        signal lbdir_q : std_logic_vector(3 downto 0) := (others => '0');
        signal n2_c3 : unsigned(9 downto 0) := (others => '0');
        signal n2_gy : signed(8 downto 0) := (others => '0');
        type t_c3s is array (0 to 2) of unsigned(9 downto 0);
        type t_g3s is array (0 to 2) of signed(8 downto 0);
        signal c3s : t_c3s := (others => (others => '0'));
        signal gys : t_g3s := (others => (others => '0'));
        signal n5_gx, n5_gy : signed(11 downto 0) := (others => '0');
        signal n6_ax, n6_ay : unsigned(10 downto 0) := (others => '0');
        signal n6_sx, n6_sy : std_logic := '0';
        signal n7_mag : unsigned(7 downto 0) := (others => '0');
        signal n7_dir : unsigned(1 downto 0) := (others => '0');
        signal n7_cg : unsigned(10 downto 0) := (others => '0');
        signal n8_mp : unsigned(11 downto 0) := (others => '0');
        signal n8_dir, n9_dir, n10_dir, n11_dir, n12_dir, n13_dir, n14_dir : unsigned(1 downto 0) := (others => '0');
        signal n9_m4, n10_m0, n11_m0, n11_m1 : unsigned(3 downto 0) := (others => '0');
        signal cnt9, cnt9_d, cnt9_dd, cnt14 : unsigned(9 downto 0) := (others => '0');
        signal n14_acc : std_logic := '0';
        signal n10_mq : std_logic_vector(7 downto 0) := (others => '0');
        signal n10_dq : std_logic_vector(3 downto 0) := (others => '0');
        type t_m3 is array (0 to 2) of unsigned(3 downto 0);
        type t_d3 is array (0 to 2) of unsigned(1 downto 0);
        type t_b3 is array (0 to 2) of std_logic;
        signal w0, w1, w2 : t_m3 := (others => (others => '0'));
        signal wd : t_d3 := (others => (others => '0'));
        signal wa : t_b3 := (others => '0');
        signal n12_nms, n12_strong, n12_weak, n12_accA : std_logic := '0';
        signal n12_mc : unsigned(3 downto 0) := (others => '0');
        signal n13_acc : std_logic := '0';
    begin
        p_canny_ram : process(clk)
        begin
            if rising_edge(clk) then
                lbmag_q <= lbmag(to_integer(cnt9));
                lbdir_q <= lbdir(to_integer(cnt9));
                -- magnitude rows {rho, rho-1} written at stage 11; direction of the
                -- Sobel row + accept of the NMS row written at stage 14 (read next line)
                if ce_d = '1' and hv_c(11) = '1' then
                    lbmag(to_integer(cnt9_dd)) <= std_logic_vector(n11_m0) & std_logic_vector(n11_m1);
                end if;
                if ce_d = '1' and hv_c(14) = '1' then
                    lbdir(to_integer(cnt14)) <= std_logic_vector(n14_dir) & n14_acc & '0';
                end if;
            end if;
        end process p_canny_ram;

        p_canny : process(clk)
            variable v_na, v_nb : unsigned(3 downto 0);
            variable v_m : unsigned(11 downto 0);
            variable v_cg : unsigned(9 downto 0);
        begin
            if rising_edge(clk) then
                if ce = '1' then
                    -- stage 2: column sums for the Sobel (rows r-3..r-1)
                    n2_c3 <= resize(ny(1), 10) + resize(ny(3), 10) + shift_left(resize(ny(2), 10), 1);
                    n2_gy <= signed(resize(ny(3), 9)) - signed(resize(ny(1), 9));
                    c3s(0) <= n2_c3; c3s(1) <= c3s(0); c3s(2) <= c3s(1);
                    gys(0) <= n2_gy; gys(1) <= gys(0); gys(2) <= gys(1);
                    if hf_c(2) = '1' then c3s(1) <= n2_c3; c3s(2) <= n2_c3; gys(1) <= n2_gy; gys(2) <= n2_gy; end if;
                    -- stage 5: gradients for centre column (window centre at step u+4)
                    n5_gx <= signed(resize(c3s(0), 12)) - signed(resize(c3s(2), 12));
                    n5_gy <= resize(gys(0), 12) + shift_left(resize(gys(1), 12), 1) + resize(gys(2), 12);
                    -- stage 6: magnitudes and signs
                    if n5_gx < 0 then n6_ax <= unsigned(resize(-n5_gx, 11)); else n6_ax <= unsigned(resize(n5_gx, 11)); end if;
                    if n5_gy < 0 then n6_ay <= unsigned(resize(-n5_gy, 11)); else n6_ay <= unsigned(resize(n5_gy, 11)); end if;
                    n6_sx <= n5_gx(11); n6_sy <= n5_gy(11);
                    -- stage 7: |gx|+|gy| -> 8 bits, 4 directions (sign rule: same signs -> down-right)
                    v_m := resize(n6_ax, 12) + resize(n6_ay, 12);
                    if s_temporal = '1' and n7_cg > v_m(10 downto 0) then v_m := '0' & n7_cg; end if;
                    if v_m(11) = '1' then n7_mag <= (others => '1'); else n7_mag <= v_m(10 downto 3); end if;
                    if shift_left(resize(n6_ay, 13), 1) + resize(shift_right(n6_ay, 1), 13) < resize(n6_ax, 13) then
                        n7_dir <= "00";                                   -- gradient horizontal: vertical edge
                    elsif shift_left(resize(n6_ax, 13), 1) + resize(shift_right(n6_ax, 1), 13) < resize(n6_ay, 13) then
                        n7_dir <= "10";                                   -- gradient vertical
                    elsif n6_sx = n6_sy then
                        n7_dir <= "01";                                   -- down-right
                    else
                        n7_dir <= "11";                                   -- up-right
                    end if;
                    -- chroma gradient: |dU| + |dV| across the chroma window (vertical sums)
                    if cvs_u >= cvp_u then v_cg := resize(cvs_u - cvp_u, 10); else v_cg := resize(cvp_u - cvs_u, 10); end if;
                    if cvs_v >= cvp_v then v_cg := v_cg + resize(cvs_v - cvp_v, 10); else v_cg := v_cg + resize(cvp_v - cvs_v, 10); end if;
                    n7_cg <= v_cg & '0';
                    -- stage 8..9: pre-gain -> 4-bit magnitude
                    n8_mp <= n7_mag * c_cgain;
                    n8_dir <= n7_dir;
                    if n8_mp(11 downto 10) /= "00" then n9_m4 <= (others => '1'); else n9_m4 <= n8_mp(9 downto 6); end if;
                    n9_dir <= n8_dir;
                    if hv_c(8) = '1' then
                        if hf_c(8) = '1' then cnt9 <= (others => '0'); else cnt9 <= cnt9 + 1; end if;
                    end if;
                    cnt9_d <= cnt9; cnt9_dd <= cnt9_d;
                    -- stage 10: current row + RAM rows (rho-1, rho-2) and dir(rho-1)
                    n10_m0 <= n9_m4; n10_mq <= lbmag_q; n10_dq <= lbdir_q; n10_dir <= n9_dir;
                    n11_m0 <= n10_m0; n11_m1 <= unsigned(n10_mq(7 downto 4));
                    if hv_c(13) = '1' then
                        if hf_c(13) = '1' then cnt14 <= (others => '0'); else cnt14 <= cnt14 + 1; end if;
                    end if;
                    -- 3-wide windows (centre index 1)
                    w0(0) <= n10_m0; w0(1) <= w0(0); w0(2) <= w0(1);
                    w1(0) <= unsigned(n10_mq(7 downto 4)); w1(1) <= w1(0); w1(2) <= w1(1);
                    w2(0) <= unsigned(n10_mq(3 downto 0)); w2(1) <= w2(0); w2(2) <= w2(1);
                    wd(0) <= unsigned(n10_dq(3 downto 2)); wd(1) <= wd(0); wd(2) <= wd(1);
                    wa(0) <= n10_dq(1); wa(1) <= wa(0); wa(2) <= wa(1);
                    n11_dir <= n10_dir;
                    -- stage 12: non-maximum suppression on the centre row
                    case wd(1) is
                        when "00"   => v_na := w1(0); v_nb := w1(2);
                        when "10"   => v_na := w0(1); v_nb := w2(1);
                        when "01"   => v_na := w0(0); v_nb := w2(2);     -- down-right / up-left
                        when others => v_na := w2(0); v_nb := w0(2);     -- up-right / down-left
                    end case;
                    if w1(1) >= v_na and w1(1) > v_nb then n12_nms <= '1'; else n12_nms <= '0'; end if;
                    if w1(1) >= 8 then n12_strong <= '1'; else n12_strong <= '0'; end if;
                    if w1(1) >= 4 then n12_weak <= '1'; else n12_weak <= '0'; end if;
                    n12_accA <= wa(0) or wa(1) or wa(2);
                    n12_mc <= w1(1);
                    n12_dir <= n11_dir;
                    -- stage 14 (window centre is one step behind the newest sample):
                    -- hysteresis from the left neighbour and the row above, alpha 3 bits
                    if n12_nms = '1' and (n12_strong = '1' or (n12_weak = '1' and (n13_acc = '1' or n12_accA = '1'))) then
                        n13_acc <= '1';
                        if n12_mc >= 8 then ca13 <= "11"; elsif n12_mc >= 6 then ca13 <= "10"; else ca13 <= "01"; end if;
                    else
                        n13_acc <= '0';
                        ca13 <= (others => '0');
                    end if;
                    n14_acc <= n13_acc;
                    n13_dir <= n12_dir; n14_dir <= n13_dir;
                    ca_d(15) <= ca13;
                    for i in 16 to 32 loop ca_d(i) <= ca_d(i - 1); end loop;
                end if;
            end if;
        end process p_canny;
    end generate;

    ------------------------------------------------------------------------
    -- per-frame sequencer: fixed thresholds, k-means on last frame's
    -- histogram, shadow/wipe products (one shared serial multiplier)
    ------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_rem : unsigned(4 downto 0);
        variable v_t : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            hist_swe <= '0';
            -- register files: plain flop <- bus, enabled by last clock's one-hot
            for k in 1 to 7 loop
                if thr_we(k) = '1' then thr(k) <= thr_wd; end if;
            end loop;
            for k in 0 to 7 loop
                if cen_we(k) = '1' then cen(k) <= cen_wd; end if;
            end loop;
            thr_we <= (others => '0'); cen_we <= (others => '0');
            -- serial multiplier: operand fetch by op code, then 10 shift-add steps
            mul_go <= '0';
            if mul_go = '1' then
                case mul_op is
                    when "00"   => mul_a <= resize(meas_h, 21);  mul_b <= resize(meas_w(11 downto 4), 10);
                    when "01"   => mul_a <= resize(c_shofs, 21); mul_b <= resize(c_my, 10);
                    when "10"   => mul_a <= resize(c_shofs, 21); mul_b <= resize(c_mx, 10);
                    when others => mul_a <= resize(p12, 21);     mul_b <= resize(meas_w(11 downto 1), 10);
                end case;
                mul_p <= (others => '0'); mul_n <= to_unsigned(9, 4); mul_busy <= '1';
            elsif mul_busy = '1' then
                if mul_b(0) = '1' then mul_p <= mul_p + mul_a; end if;
                mul_a <= shift_left(mul_a, 1); mul_b <= shift_right(mul_b, 1);
                if mul_n = 0 then mul_busy <= '0'; else mul_n <= mul_n - 1; end if;
            end if;
            -- divider by the level count: restoring, 18 steps, 5-bit remainder
            if div_busy = '1' then
                v_rem := dv_rem(3 downto 0) & dv_q(17);
                if v_rem >= to_unsigned(f_nlev, 5) then
                    dv_rem <= v_rem - to_unsigned(f_nlev, 5);
                    dv_q <= dv_q(16 downto 0) & '1';
                else
                    dv_rem <= v_rem;
                    dv_q <= dv_q(16 downto 0) & '0';
                end if;
                if dv_n = 0 then div_busy <= '0'; else dv_n <= dv_n - 1; end if;
            end if;

            if vs_edge = '1' then
                sq <= 1;
            else
                case sq is
                    when 0 => null;
                    when 1 =>
                        -- latch this frame's level count; reload the table when it changes or fixed
                        f_nlev <= c_nlev; f_coarse <= c_coarse;
                        f_temp <= s_temporal and f_b2sl(C_HAS_TEMPORAL);
                        if c_sm = 2 then f_kb2 <= "11"; else f_kb2 <= c_kb2; end if;
                        if c_nlev /= sq_nprev or s_adapt = '0' then sq_reload <= '1'; else sq_reload <= '0'; end if;
                        sq_nprev <= c_nlev;
                        case c_nlev is
                            when 2 => sq_fdel <= to_unsigned(110, 7);
                            when 3 => sq_fdel <= to_unsigned(73, 7);
                            when 4 => sq_fdel <= to_unsigned(55, 7);
                            when 5 => sq_fdel <= to_unsigned(44, 7);
                            when 6 => sq_fdel <= to_unsigned(37, 7);
                            when 7 => sq_fdel <= to_unsigned(31, 7);
                            when others => sq_fdel <= to_unsigned(27, 7);
                        end case;
                        sq_acc <= to_unsigned(16, 9); sq_k <= 1;
                        sq <= 2;
                    when 2 =>
                        -- fixed thresholds / centres, one bin per pass (k = 1..8):
                        -- values on the buses, one-hot enables, landed in state 3
                        cen_wd <= resize(sq_acc + resize(shift_right(sq_fdel, 1), 9), 8);
                        if sq_k < f_nlev then thr_wd <= sq_acc + resize(sq_fdel, 9);
                        else thr_wd <= to_unsigned(511, 9); end if;
                        if sq_reload = '1' then
                            if sq_k <= f_nlev then cen_we(sq_k - 1) <= '1'; end if;
                            if sq_k <= 7 then thr_we(sq_k) <= '1'; end if;
                        end if;
                        sq_acc <= sq_acc + resize(sq_fdel, 9);
                        sq <= 3;
                    when 3 =>
                        if sq_k = 8 then
                            if s_adapt = '1' and sq_reload = '0' then sq <= 10; else sq <= 40; end if;
                        else
                            sq_k <= sq_k + 1; sq <= 2;
                        end if;
                    -- adaptive thresholds: equal-population (percentile) placement
                    -- in ONE pass over last frame's 64-bin histogram; the
                    -- representative luma of a bin is its median sample.  Both are
                    -- low-passed 1/4 per frame and kept monotonic.  (A k-means
                    -- engine was ~1000 LC; this is ~350.)
                    when 10 =>
                        seq_hist <= '1';
                        -- sample total = (Wh / 8) x H  (a sample every 8th half-res column)
                        mul_op <= "00"; mul_go <= '1';
                        sq <= 11;
                    when 11 =>
                        if mul_busy = '0' and mul_go = '0' then
                            dv_q <= mul_p(17 downto 0); dv_rem <= (others => '0');
                            dv_n <= to_unsigned(17, 5); div_busy <= '1';
                            sq <= 12;
                        end if;
                    when 12 =>
                        if div_busy = '0' then
                            sq_step <= dv_q;
                            sq_tt <= dv_q; sq_mt <= '0' & dv_q(17 downto 1);
                            sq_cum <= (others => '0'); sq_i <= (others => '0'); sq_j <= 0; sq_jm <= 0;
                            sq <= 14;
                        end if;
                    when 14 =>
                        hist_sa <= resize(sq_i, 8); sq <= 15;
                    when 15 => sq <= 16;
                    when 16 =>
                        sq_hval <= hist_q; hist_swe <= '1';                 -- read, then clear
                        sq <= 17;
                    when 17 =>
                        sq_cum <= sq_cum + resize(sq_hval, 18);
                        sq <= 18;
                    when 18 =>
                        -- which placement fires for this bin (one compare each)
                        if sq_j < f_nlev - 1 and sq_cum >= sq_tt then pl_thr <= '1'; else pl_thr <= '0'; end if;
                        if sq_jm < f_nlev and sq_cum >= sq_mt then pl_med <= '1'; else pl_med <= '0'; end if;
                        sq <= 22;
                    when 22 =>
                        -- fetch the register being updated (and the one below it)
                        if pl_thr = '1' then
                            pl_old <= thr(sq_j + 1); pl_prev2 <= thr(sq_j) + 2;
                            pl_new <= shift_left(resize(sq_i, 9), 2) + 4;      -- upper edge of this bin
                            sq <= 23;
                        elsif pl_med = '1' then
                            pl_old <= '0' & cen(sq_jm); pl_prev2 <= (others => '0');
                            pl_new <= shift_left(resize(sq_i, 9), 2) + 2;      -- centre of this bin
                            sq <= 23;
                        elsif sq_i = 63 then
                            sq <= 19;
                        else
                            sq_i <= sq_i + 1; sq <= 14;
                        end if;
                    when 23 =>
                        if pl_new > 235 then pl_new <= to_unsigned(235, 9); end if;
                        sq <= 24;
                    when 24 =>
                        pl_dif <= signed(resize(pl_new, 10)) - signed(resize(pl_old, 10));
                        sq <= 25;
                    when 25 =>
                        -- low-pass 1/4 of the way
                        pl_lp <= signed(resize(pl_old, 11)) + resize(shift_right(pl_dif, 2), 11);
                        sq <= 26;
                    when 26 =>
                        -- monotonic (thresholds only), then onto the bus
                        v_t := pl_lp;
                        if pl_thr = '1' and v_t < signed(resize(pl_prev2, 11)) then v_t := signed(resize(pl_prev2, 11)); end if;
                        if v_t > 235 then v_t := to_signed(235, 11); end if;
                        if pl_thr = '1' then
                            thr_wd <= unsigned(v_t(8 downto 0)); thr_we(sq_j + 1) <= '1';
                            sq_tt <= sq_tt + sq_step; sq_j <= sq_j + 1;
                        else
                            cen_wd <= unsigned(v_t(7 downto 0)); cen_we(sq_jm) <= '1';
                            sq_mt <= sq_mt + sq_step; sq_jm <= sq_jm + 1;
                        end if;
                        sq <= 20;                                              -- land the write
                    when 20 => sq <= 18;                                   -- write landed; re-check
                    when 19 =>
                        -- anything not reached (total over-estimated): white
                        seq_hist <= '0';
                        if sq_j < f_nlev - 1 then
                            thr_wd <= to_unsigned(235, 9); thr_we(sq_j + 1) <= '1'; sq_j <= sq_j + 1; sq <= 21;
                        elsif sq_jm < f_nlev then
                            cen_wd <= to_unsigned(235, 8); cen_we(sq_jm) <= '1'; sq_jm <= sq_jm + 1; sq <= 21;
                        else
                            sq <= 40;
                        end if;
                    when 21 => sq <= 19;
                    -- shadow offsets and wipe column
                    when 40 =>
                        mul_op <= "01"; mul_go <= '1'; sq <= 41;
                    when 41 =>
                        if mul_busy = '0' and mul_go = '0' then
                            if mul_p(13 downto 12) /= "00" then f_dy <= (others => '1');
                            else f_dy <= mul_p(11 downto 8); end if;
                            if C_HAS_SHADOW = false or c_shofs = 0 then f_dy <= (others => '0'); end if;
                            mul_op <= "10"; mul_go <= '1'; sq <= 42;
                        end if;
                    when 42 =>
                        if mul_busy = '0' and mul_go = '0' then
                            if mul_p(13) = '1' then f_dxq <= (others => '1');
                            else f_dxq <= mul_p(12 downto 8); end if;
                            mul_op <= "11"; mul_go <= '1'; sq <= 43;
                        end if;
                    when 43 =>
                        if mul_busy = '0' and mul_go = '0' then
                            f_wx <= mul_p(20 downto 10) + 1;
                            sq <= 0;
                        end if;
                    when others => sq <= 0;
                end case;
            end if;
        end if;
    end process p_seq;

    ------------------------------------------------------------------------
    -- F composite: colour table, ink/shadow blend, lines view, conditioning,
    -- wipe, gate.  O1 holds pixel x during cycle t_in + x + 79.
    ------------------------------------------------------------------------
    p_out : process(clk)
        variable v_uv : std_logic_vector(19 downto 0);
        variable v_a3 : unsigned(2 downto 0);
        variable v_a2 : unsigned(1 downto 0);
        variable v_cy, v_cu, v_cv : unsigned(9 downto 0);
        variable v_dy, v_du, v_dv : signed(10 downto 0);
        variable v_sum : signed(11 downto 0);
        variable v_y : unsigned(9 downto 0);
        variable v_wipe : std_logic;
        variable v_sy, v_su, v_sv : unsigned(11 downto 0);
        variable v_d : unsigned(9 downto 0);
        variable v_av : std_logic;
    begin
        if rising_edge(clk) then
            ----------------------------------------------------------
            -- O1: per-pixel values from the hand-off
            ----------------------------------------------------------
            if cnt37h < f_wx then v_wipe := '1'; else v_wipe := '0'; end if;
            wipe_c(1) <= v_wipe;
            for i in 2 to 6 loop wipe_c(i) <= wipe_c(i - 1); end loop;

            v_uv := f_rep_uv(ho_c_hs(4 downto 2), ho_c_hs(1 downto 0), f_coarse);
            v_y := ho_c_y & "00";
            -- soft: horizontal half-step across label changes
            if C_HAS_SOFT and s_soft = '1' then
                if ce = '1' and ho_n_y /= ho_c_y then
                    v_y := resize(shift_right(resize(v_y, 12) + shift_left(resize(v_y, 12), 1) + resize(ho_n_y & "00", 12), 2), 10);
                elsif ce = '0' and ho_p_y /= ho_c_y then
                    v_y := resize(shift_right(resize(v_y, 12) + shift_left(resize(v_y, 12), 1) + resize(ho_p_y & "00", 12), 2), 10);
                end if;
            end if;
            o1_bg <= '0'; o1_noink <= '0';
            if s_lines = '1' then
                -- lines only, on mid grey
                o1_y <= C_MID; o1_u <= C_MID; o1_v <= C_MID;
            else
                o1_y <= v_y; o1_u <= unsigned(v_uv(19 downto 10)); o1_v <= unsigned(v_uv(9 downto 0));
                if ho_c_lum = "000" then o1_bg <= '1'; end if;
            end if;
            -- ink alpha code 0..3 (0 / quarter / half / full); the odd pixel takes
            -- the rounded mean of this and the next sample (horizontal AA)
            v_a3 := resize(ho_c_a, 3) + resize(ho_n_a, 3) + 1;
            if ce = '1' then o1_a <= v_a3(2 downto 1); else o1_a <= ho_c_a; end if;
            o1_s <= sha37;
            if aline < C_TOPG then o1_a <= (others => '0'); o1_s <= (others => '0'); o1_bg <= '1'; o1_noink <= '1'; end if;

            ----------------------------------------------------------
            -- O2: background + shadow colour, ink delta scaled by the alpha code
            -- (0 / quarter / half / full); O3: compose
            ----------------------------------------------------------
            if o1_bg = '1' then
                -- shadow grey 64 + 10 * s (s = 0..16 -> up to 224)
                v_cy := C_BLKY + resize(shift_left(resize(o1_s, 10), 3) + shift_left(resize(o1_s, 10), 1), 10);
                v_cu := C_MID; v_cv := C_MID;
            else
                v_cy := o1_y; v_cu := o1_u; v_cv := o1_v;
            end if;
            if c_white = '1' then v_dy := to_signed(775, 11) - signed(resize(v_cy, 11));
            else v_dy := to_signed(64, 11) - signed(resize(v_cy, 11)); end if;
            v_du := to_signed(512, 11) - signed(resize(v_cu, 11));
            v_dv := to_signed(512, 11) - signed(resize(v_cv, 11));
            if o1_noink = '1' then v_a2 := "00"; else v_a2 := o1_a; end if;
            case v_a2 is
                when "00"   => o3_py <= (others => '0'); o3_pu <= (others => '0'); o3_pv <= (others => '0');
                when "01"   => o3_py <= shift_right(v_dy, 2); o3_pu <= shift_right(v_du, 2); o3_pv <= shift_right(v_dv, 2);
                when "10"   => o3_py <= shift_right(v_dy, 1); o3_pu <= shift_right(v_du, 1); o3_pv <= shift_right(v_dv, 1);
                when others => o3_py <= v_dy; o3_pu <= v_du; o3_pv <= v_dv;
            end case;
            o3_y <= v_cy; o3_u <= v_cu; o3_v <= v_cv;
            -- the blend is convex (0..1023): SLICE the 12-bit sum, never resize a
            -- signed value to 10 bits (that keeps the sign and drops bit 9)
            v_sum := signed(resize(o3_y, 12)) + resize(o3_py, 12); o4_y <= unsigned(v_sum(9 downto 0));
            v_sum := signed(resize(o3_u, 12)) + resize(o3_pu, 12); o4_u <= unsigned(v_sum(9 downto 0));
            v_sum := signed(resize(o3_v, 12)) + resize(o3_pv, 12); o4_v <= unsigned(v_sum(9 downto 0));

            ----------------------------------------------------------
            -- O5..O7: [1 2 1] on Y/U/V, luma toward black (HD HDMI bar fix)
            ----------------------------------------------------------
            g1_y <= o4_y; g1_u <= o4_u; g1_v <= o4_v;
            g2_y <= g1_y; g2_u <= g1_u; g2_v <= g1_v;
            v_sy := resize(g2_y, 12) + shift_left(resize(g1_y, 12), 1) + resize(o4_y, 12);
            v_su := resize(g2_u, 12) + shift_left(resize(g1_u, 12), 1) + resize(o4_u, 12);
            v_sv := resize(g2_v, 12) + shift_left(resize(g1_v, 12), 1) + resize(o4_v, 12);
            o6_y <= v_sy(11 downto 2); o6_u <= v_su(11 downto 2); o6_v <= v_sv(11 downto 2);
            if o6_y > 64 then v_d := o6_y - 64; else v_d := (others => '0'); end if;
            o7_y <= o6_y - shift_right(v_d, 3) - shift_right(v_d, 4);
            o7_u <= o6_u; o7_v <= o6_v;

            ----------------------------------------------------------
            -- O8: wipe against the dry ring, blanking gate, parity-corrected avid
            ----------------------------------------------------------
            rq_avid_d <= ring_q(26);
            if ring_q(26) = '1' and rq_avid_d = '0' then out_sel <= ring_q(30); end if;
            if ring_q(26) = '1' and rq_avid_d = '0' then
                if ring_q(30) = '1' then v_av := '0'; else v_av := '1'; end if;
            elsif out_sel = '1' then v_av := rq_avid_d;
            else v_av := ring_q(26); end if;
            out_avid <= v_av;
            if ring_q(26) = '0' then
                out_y <= C_BLKY; out_u <= C_MID; out_v <= C_MID;
            elsif wipe_c(6) = '1' then
                out_y <= o7_y; out_u <= o7_v; out_v <= o7_u;         -- back to hardware order
            else
                out_y <= unsigned(ring_q(9 downto 0));
                out_u <= unsigned(ring_q(25 downto 18)) & "00";      -- ring: 17:10 = data_in.v, 25:18 = data_in.u
                out_v <= unsigned(ring_q(17 downto 10)) & "00";
            end if;
            out_hs <= ring_q(27); out_vs <= ring_q(28); out_fld <= ring_q(29);
        end if;
    end process p_out;

    data_out.y <= std_logic_vector(out_y);
    data_out.u <= std_logic_vector(out_u);
    data_out.v <= std_logic_vector(out_v);
    data_out.avid <= out_avid;
    data_out.hsync_n <= out_hs;
    data_out.vsync_n <= out_vs;
    data_out.field_n <= out_fld;

end architecture flatline;
