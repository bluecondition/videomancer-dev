-- SPILLWAY: dam-overflow luma cascade.
--
-- Luma is water behind a dam. K1 CREST sets a luma threshold: content below
-- it is the calm reservoir (passthrough); content above it spills downward
-- over the crest, accelerating into vertical streaks, aerating into
-- whitewater, throwing mist where it lands, optionally pooling in a rippling
-- reflective basin. P12 is the master FLOODGATE: a 2nd-order spring with
-- asymmetric stiffness gives the release a weighty settle with one or two
-- damped backwash lobes (~2 s) instead of a snap.
--
-- The spill direction is the raster scan direction, so the engine is
-- per-column state carried down the frame in scan order -- no frame buffer.
--
-- "Modules" (this platform is one-architecture-per-program; each module is a
-- clearly named process):
--   p_in      input regs, counters, per-line sequencer (wobble, basin rows)
--   p_ppipe   SPILL ENGINE: pair-cadence column-state pipeline + noise shaper
--             (R1..R6: BRAM fetch -> filament/foam hashes -> render terms)
--             + the state-update ALU fed by the live even pixel
--   p_epipe   streaming compositor E2..E7 (drain, water crossfade, foam,
--             glint, mist, undertow, basin reflection, roar)
--   p_frame   DYNAMICS: vblank sequencer (floodgate spring, surge, freeze,
--             basin level, palette via one shared multiplier), 16-gate FSM,
--             and the 1024-column sweep (state reset, mist decay+smooth,
--             undertow height chase)
--   p_ports + BRAM processes: canonical 1W1R column-state / ring memories
--
-- Column state lives at HALF horizontal resolution (state column c = x>>1,
-- 1024 entries): only the even pixel of a pair updates state, so every
-- per-column operation gets 2 clocks. All per-pixel math is shift/add/mux
-- against pre-registered per-frame constants; every multiply lives in the
-- vblank sequencer on the shared registered multiplier.
--
-- Fixed point: video 10 b; w (water flux), v (fall velocity), foam 8 b
-- saturating; spring pos/vel Q10.6; fall-phase scroll 16 b wrapping.
-- Height class (0/1/2 at >=540/>=864 active lines) rescales gravity, streak
-- persistence and the mist band so all modes match in character.
--
-- BRAM budget (4 kbit EBRs): 4 state arrays 1024x16 = 16, undertow heights
-- 1024x8 = 2, basin ring 4 x 1024x8 = 8 -> 26 EBR.
--
-- Column state semantics, updated once per line as the beam passes:
--   w     water flux falling through this column   (streak body)
--   v     fall velocity: filament cells stretch 4->32 lines and the texture
--         scrolls faster as v grows; coverage thins with v (continuity:
--         sheet necks down into fewer, brighter filaments)
--   foam  aeration, grows with v at the CHURN rate -> clumpy whitecaps
--   cu/cv carried source chroma (the water remembers what it swallowed)
--   im_s/im_y  impact record (basin landing or bright->dark ledge lip),
--         rendered NEXT frame as rising mist above the impact (upward mist
--         cannot be drawn in scan order same-frame; one frame late is
--         invisible); smoothed across columns in the sweep -> soft clouds
--   dp    "was discharging last line" bit (ledge lip detector)
--   dk    dark-line count below the waterline (UNDERTOW accumulator); the
--         sweep chases per-column tendril heights toward it -- the coarse,
--         frame-late approximation for motion that fights the scan order
--
-- FREEZE does NOT touch the state carry (it is rewritten every line; with a
-- static source it re-derives identically each frame). Crystallization =
-- stop the scroll phase + noise seed + ripple wobble + gate motion, latch
-- the effective slider position in the ice (thaw crossing releases it = the
-- lurch), and paint rare stable glint threads.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture spillway of program_top is

    constant C_DW  : integer := C_VIDEO_DATA_WIDTH;  -- 10
    constant C_LAT : integer := 8;
    constant C_PF  : integer := 6;                   -- prefetch, pairs

    ----------------------------------------------------------------------
    -- Column state / ring BRAMs (power-of-2 padded; inferno lesson)
    ----------------------------------------------------------------------
    type t_st16 is array (0 to 1023) of std_logic_vector(15 downto 0);
    type t_st8  is array (0 to 1023) of std_logic_vector(7 downto 0);

    signal st_wv : t_st16 := (others => (others => '0'));  -- {w, v}
    signal st_uv : t_st16 := (others => (others => '0'));  -- {cu, cv}
    signal st_fi : t_st16 := (others => (others => '0'));  -- {foam, im_s}
    signal st_iy : t_st16 := (others => (others => '0'));  -- {im_y, dp, dk}
    signal ut_mem : t_st8 := (others => (others => '0'));  -- tendril heights
    -- 2-line ring: the depth fade to deep water hides the short repeat
    signal ring0, ring1 : t_st8 := (others => (others => '0'));

    -- shared port registers (p_ports muxes scan / sweep). Two read
    -- addresses: early (st_wv, feeds the hash pipe) and late (uv/fi/iy/ut,
    -- lands directly at the R5 latch -- saves 4 stages of pipeline copies).
    signal s_st_ra, s_st_rb, s_st_wa : unsigned(9 downto 0) := (others => '0');
    signal s_wv_wd, s_uv_wd, s_fi_wd, s_iy_wd
        : std_logic_vector(15 downto 0) := (others => '0');
    signal s_st_we, s_uv_we : std_logic := '0';
    signal s_ut_wd : std_logic_vector(7 downto 0) := (others => '0');
    signal s_ut_we : std_logic := '0';
    signal s_wv_rd, s_uv_rd, s_fi_rd, s_iy_rd
        : std_logic_vector(15 downto 0) := (others => '0');
    signal s_ut_rd : std_logic_vector(7 downto 0) := (others => '0');

    signal s_rg_ra : unsigned(9 downto 0) := (others => '0');
    signal s_rg_wa : unsigned(9 downto 0) := (others => '0');
    signal s_rg_wd : std_logic_vector(7 downto 0) := (others => '0');
    signal s_rg_we : std_logic_vector(1 downto 0) := "00";
    signal s_rg_rd0, s_rg_rd1
        : std_logic_vector(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Controls
    ----------------------------------------------------------------------
    signal k_crest, k_flow, k_plunge, k_churn, k_mist, k_depths, k_slider
        : unsigned(C_DW - 1 downto 0);
    signal sw_gates, sw_ut, sw_basin, sw_surge, sw_freeze : std_logic;

    ----------------------------------------------------------------------
    -- p_in: input regs, counters, line sequencer
    ----------------------------------------------------------------------
    signal s_in_y, s_in_u, s_in_v : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_x       : unsigned(10 downto 0) := (others => '0');  -- E1 pixel col
    signal s_prev_hsync_n, s_prev_vsync_n : std_logic := '1';
    signal s_had_avid : std_logic := '0';
    signal s_line_y   : unsigned(10 downto 0) := (others => '0');
    signal s_active_h : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_new_frame : std_logic := '0';   -- 1-clk pulse at vsync fall

    -- per-line values (line sequencer)
    signal s_lstep     : unsigned(2 downto 0) := "111";
    signal s_y8        : unsigned(7 downto 0) := (others => '0');
    signal s_yb1       : unsigned(9 downto 0) := (others => '0'); -- (H-y)>>1
    signal s_in_basin  : std_logic := '0';
    signal s_at_surf   : std_logic := '0';
    signal s_depth     : unsigned(9 downto 0) := (others => '0');
    signal s_wobx      : unsigned(15 downto 0) := (others => '0');
    signal s_wob       : unsigned(7 downto 0) := (others => '0');
    signal s_mi        : std_logic := '0';
    signal s_hoff      : signed(3 downto 0) := (others => '0');
    signal s_bs_dsh    : unsigned(2 downto 0) := "001";
    signal s_bs_q      : unsigned(1 downto 0) := "00";
    signal s_wline     : std_logic := '0';
    signal s_crn_line  : std_logic := '0';
    signal s_ymid      : std_logic := '0';   -- y > active_h/2

    -- sync/dry delay shift registers (video taps trimmed to what the
    -- compositor actually reads: y tap 2, u/v taps 1 and 3)
    type t_dsr3 is array (0 to 2) of std_logic_vector(C_DW - 1 downto 0);
    type t_dsr4 is array (0 to 4) of std_logic_vector(C_DW - 1 downto 0);
    type t_bsr is array (0 to C_LAT - 1) of std_logic;
    signal s_y_sr : t_dsr3 := (others => (others => '0'));
    signal s_u_sr, s_v_sr : t_dsr4 := (others => (others => '0'));
    signal s_hs_sr, s_vs_sr, s_fd_sr : t_bsr := (others => '1');
    signal s_av_sr : t_bsr := (others => '0');

    ----------------------------------------------------------------------
    -- p_ppipe: pair pipeline (R-stages) + state ALU
    ----------------------------------------------------------------------
    signal en_pair : std_logic := '0';

    -- pair-address / valid threading (bulletproof read/write pairing)
    signal a_r1, a_r2, a_r3, a_r4, a_r5, a_r6, a_alu
        : unsigned(9 downto 0) := (others => '0');
    signal v_r1, v_r2, v_r3, v_r4, v_r5, v_r6, v_alu : std_logic := '0';

    -- R1: latched early (st_wv) read
    signal r_w, r_v : unsigned(7 downto 0) := (others => '0');

    -- R2: scroll / cells (w and v ride the hash pipe; everything else is
    -- fetched on the late port and lands at R5)
    signal r2_w, r2_v : unsigned(7 downto 0) := (others => '0');
    signal r2_cell, r2_y4 : unsigned(10 downto 0) := (others => '0');

    -- R3/R4/R4X: staged add-shift hash rounds
    signal h_fil_a, h_pf_a, h_n_a : unsigned(15 downto 0) := (others => '0');
    signal h_fil_b, h_pf_b, h_n_b : unsigned(15 downto 0) := (others => '0');
    signal r3_w, r3_v : unsigned(7 downto 0) := (others => '0');
    signal r4_w, r4_v : unsigned(7 downto 0) := (others => '0');
    signal r4x_w, r4x_v : unsigned(7 downto 0) := (others => '0');
    signal a_r4x : unsigned(9 downto 0) := (others => '0');
    signal v_r4x : std_logic := '0';
    signal r4_fil, r4_pf, r4_n, r4_f2 : unsigned(7 downto 0) := (others => '0');

    -- R5: render terms 1 (opacity legs, combined at R6)
    signal r5_glint, r5_glassy : std_logic := '0';
    signal r5_opc, r5_cap, r5_fm : unsigned(7 downto 0) := (others => '0');
    signal r5_wl    : unsigned(9 downto 0) := (others => '0');
    signal r5_n, r5_fil : unsigned(7 downto 0) := (others => '0');
    signal r5_fogsh : unsigned(2 downto 0) := (others => '0');
    signal r5_fogen : std_logic := '0';
    signal r5_ims   : unsigned(7 downto 0) := (others => '0');
    signal r5_utm   : std_logic := '0';
    signal r5_w, r5_v, r5_foam, r5_cu, r5_cv : unsigned(7 downto 0) := (others => '0');
    signal r5_dp : std_logic := '0';
    signal r5_dk : unsigned(6 downto 0) := (others => '0');
    signal r5_imy : unsigned(7 downto 0) := (others => '0');

    -- R6: render terms 2 (held for the E-pipe). g_op is 8-bit water
    -- opacity: quarter-alpha base + 6-bit dither fraction (E5).
    signal g_op : unsigned(7 downto 0) := (others => '0');
    signal g_wl, g_wcu, g_wcv : unsigned(9 downto 0) := (others => '0');
    signal g_foamy, g_glint, g_utm, g_crownn, g_spark, g_margin : std_logic := '0';
    signal g_fogl : unsigned(8 downto 0) := (others => '0');
    signal g_n : unsigned(7 downto 0) := (others => '0');
    -- ALU-side view of the pair state (pre-update)
    signal g_w, g_v, g_foam, g_cu, g_cv, g_ims, g_imy : unsigned(7 downto 0) := (others => '0');
    signal g_dp : std_logic := '0';
    signal g_dk : unsigned(6 downto 0) := (others => '0');
    -- R6-precomputed ALU terms: the barrel shifts and long subtract chains
    -- run at pair rate here so the ALU clock is only mux + add + saturate
    -- (this path was the routed-timing killer: 65 MHz as one cone)
    signal g_wdec, g_wdro, g_vcs, g_fdec, g_vnx
        : unsigned(7 downto 0) := (others => '0');
    -- pair-rate decision flags (the rock/impact cone was the next
    -- routed-timing failure after the flux ALU)
    signal g_wet, g_wgims, g_rockp, g_dkok : std_logic := '0';
    signal g_dk1 : unsigned(6 downto 0) := (others => '0');

    -- live even-pixel capture for the ALU
    signal ev_dq  : unsigned(7 downto 0) := (others => '0');
    signal ev_u8, ev_v8 : unsigned(7 downto 0) := (others => '0');
    signal ev_d220, ev_d200, ev_h0 : std_logic := '0';
    signal ev_go  : std_logic := '0';    -- ALU fire (1 clk after even E3)

    -- scan-side write requests
    signal p_wr_a  : unsigned(9 downto 0) := (others => '0');
    signal p_wv_wd, p_uv_wd, p_fi_wd, p_iy_wd
        : std_logic_vector(15 downto 0) := (others => '0');
    signal p_we    : std_logic := '0';

    -- spill volume accumulator (basin level + crown), dq/16 units
    signal s_vol       : unsigned(15 downto 0) := (others => '0');
    signal s_vol_latch : unsigned(15 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- p_epipe: pixel compositor
    ----------------------------------------------------------------------
    signal e2_h    : unsigned(9 downto 0) := (others => '0');
    signal e2_shim : std_logic := '0';
    signal e2_ph   : std_logic := '0';
    signal e3_ph   : std_logic := '0';
    signal av_e2, av_e3 : std_logic := '0';
    signal e2_d220, e2_d200 : std_logic := '0';
    signal e3_h    : unsigned(9 downto 0) := (others => '0');
    signal e3_dq8  : unsigned(7 downto 0) := (others => '0');
    signal e3_drain : unsigned(9 downto 0) := (others => '0');
    signal e3_roll : std_logic := '0';
    signal e3_shim : std_logic := '0';
    signal e3_u8, e3_v8 : unsigned(7 downto 0) := (others => '0');
    signal e3_d220, e3_d200 : std_logic := '0';
    signal e4_yb   : unsigned(9 downto 0) := (others => '0');
    signal e4_roll : std_logic := '0';
    -- E5a: registered alpha select + carried operands
    signal s_dith6 : unsigned(5 downto 0) := (others => '0');
    signal e5_a : unsigned(2 downto 0) := (others => '0');
    signal e5_wls : unsigned(9 downto 0) := (others => '0');
    signal e5a_yb : unsigned(9 downto 0) := (others => '0');
    signal e5a_glint, e5a_spark, e5a_utm, e5a_crownn : std_logic := '0';
    signal e5a_fogl : unsigned(8 downto 0) := (others => '0');
    signal e5a_wcu, e5a_wcv : unsigned(9 downto 0) := (others => '0');
    -- E5b: partial products
    signal e5_py : unsigned(11 downto 0) := (others => '0');
    signal e5_pu, e5_pv : unsigned(9 downto 0) := (others => '0');
    signal e5_glint, e5_spark, e5_utm : std_logic := '0';
    signal e5_fogl : unsigned(8 downto 0) := (others => '0');
    signal e5_crownn : std_logic := '0';
    signal e6_y, e6_u, e6_v : unsigned(9 downto 0) := (others => '0');
    signal e6_utm, e6_crownn : std_logic := '0';
    signal s_out_y, s_out_u, s_out_v : unsigned(9 downto 0) := (others => '0');

    -- per-line flags delayed alongside the pixel path
    signal bsn_d : std_logic_vector(7 downto 0) := (others => '0');
    signal crn_d : std_logic_vector(7 downto 0) := (others => '0');

    -- ring read data pipeline (E2..E7)
    signal ry2, ry3, ry4, ry5, ry6, ry7 : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- p_frame: per-frame constants and slow dynamics
    ----------------------------------------------------------------------
    signal s_frame   : unsigned(15 downto 0) := (others => '0');
    signal s_lfsr    : unsigned(31 downto 0) := x"CAFEB0BA";
    signal s_fe      : unsigned(7 downto 0) := (others => '0');
    signal s_nph     : unsigned(7 downto 0) := (others => '0');
    signal s_fall_ph : unsigned(15 downto 0) := (others => '0');

    signal fg_pos : signed(17 downto 0) := (others => '0');
    signal fg_vel : signed(19 downto 0) := (others => '0');
    signal s_pos_eff : unsigned(9 downto 0) := (others => '0');
    signal s_pos_lat : unsigned(9 downto 0) := (others => '0');
    signal s_lat_on  : std_logic := '0';
    signal s_armed   : std_logic := '0';

    signal s_surge_ph : unsigned(19 downto 0) := (others => '0');
    signal s_surge    : unsigned(7 downto 0) := (others => '0');

    -- per-frame render constants
    signal s_crest_base : unsigned(9 downto 0) := to_unsigned(586, 10);
    signal s_crest_lo, s_crest_hi : unsigned(9 downto 0) := (others => '0');
    signal s_fs0, s_fs1 : unsigned(3 downto 0) := "1111";
    signal s_g    : unsigned(4 downto 0) := to_unsigned(8, 5);
    signal s_dsh  : unsigned(3 downto 0) := to_unsigned(6, 4);
    signal s_leak : unsigned(0 downto 0) := "1";
    signal s_cs   : unsigned(3 downto 0) := to_unsigned(6, 4);
    signal s_k4   : unsigned(7 downto 0) := (others => '0');
    signal s_mist_on : std_logic := '0';
    signal s_mist_sh : unsigned(1 downto 0) := "00";
    signal s_mband   : unsigned(7 downto 0) := to_unsigned(12, 8);
    signal s_tintY, s_tintU, s_tintV : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_tintY4 : unsigned(7 downto 0) := (others => '0');
    signal s_deep8  : unsigned(7 downto 0) := (others => '0');
    signal s_tint34u, s_tint34v : unsigned(9 downto 0) := to_unsigned(384, 10);
    signal s_bu, s_bv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_cw   : unsigned(2 downto 0) := to_unsigned(4, 3);
    signal s_roar : unsigned(7 downto 0) := (others => '0');
    signal s_level : unsigned(18 downto 0) := (others => '0');
    signal s_surface : unsigned(10 downto 0) := to_unsigned(2047, 11);
    signal s_crown : unsigned(7 downto 0) := (others => '0');
    signal s_crown_h : unsigned(5 downto 0) := (others => '0');
    signal s_hclass : unsigned(1 downto 0) := "10";
    signal s_fe_sh : unsigned(2 downto 0) := "000";  -- ripple atten (7 = off)
    signal s_hmid  : unsigned(10 downto 0) := (others => '0');

    -- gates
    -- 8 gates (in the 8-16 spec): halves the rotate-ring FF count and its
    -- routing tension -- the ring's wires were the routed critical path
    type t_g8 is array (0 to 7) of unsigned(7 downto 0);
    signal g_env : t_g8 := (others => to_unsigned(255, 8));
    signal g_tmr : t_g8 := (others => to_unsigned(60, 8));
    signal g_tgt : t_g8 := (others => to_unsigned(255, 8));
    -- lift stored in 8-bit (255-env); the <<2 to luma happens at use site
    signal s_gate_lift : t_g8 := (others => (others => '0'));
    signal s_glift_r : unsigned(7 downto 0) := (others => '0');
    -- gate step pipeline regs (LFSR->decision cone split over 2 substeps)
    signal s_gr8   : unsigned(7 downto 0) := (others => '0');
    signal s_gfire : std_logic := '0';

    -- frame sequencer / shared multiplier (internally 2-stage: partial
    -- products then sum -- every consumer reads s_mp two steps after
    -- loading ma/mb, so the extra register is timing-free)
    signal s_vstep : unsigned(5 downto 0) := (others => '1');
    signal s_ma : signed(12 downto 0) := (others => '0');
    signal s_mb : signed(10 downto 0) := (others => '0');
    signal s_mp_a, s_mp_b : signed(18 downto 0) := (others => '0');
    signal s_mp : signed(23 downto 0) := (others => '0');
    -- spring pipeline registers (the one-clock 4-term accumulate was the
    -- routed-timing critical path across the whole design)
    signal s_sperr : signed(18 downto 0) := (others => '0');
    signal s_sdamp : signed(19 downto 0) := (others => '0');
    signal s_gain : unsigned(9 downto 0) := (others => '0');
    signal s_open_k : unsigned(7 downto 0) := (others => '0');
    signal s_flow_k : unsigned(7 downto 0) := (others => '0');
    signal s_crest_drop : unsigned(9 downto 0) := (others => '0');
    signal s_p2 : unsigned(9 downto 0) := (others => '0');
    signal s_seg : unsigned(1 downto 0) := "00";
    signal s_segf : unsigned(7 downto 0) := (others => '0');
    signal s_k63 : unsigned(11 downto 0) := (others => '0');
    signal s_pal_ay, s_pal_by, s_pal_au, s_pal_bu, s_pal_av, s_pal_bv
        : unsigned(9 downto 0) := (others => '0');

    -- gate FSM / sweep FSM
    signal s_gstep : unsigned(4 downto 0) := (others => '1');
    signal s_sw_run : std_logic := '0';
    signal s_sw_c   : unsigned(10 downto 0) := (others => '0');
    signal s_sw_ph  : std_logic := '0';
    signal sw_ra    : unsigned(9 downto 0) := (others => '0');
    signal sw_wa    : unsigned(9 downto 0) := (others => '0');
    signal sw_we    : std_logic := '0';
    signal sw_fi_wd, sw_iy_wd : std_logic_vector(15 downto 0) := (others => '0');
    signal sw_ut_wd : std_logic_vector(7 downto 0) := (others => '0');
    signal sw_ims_p, sw_ims_c : unsigned(7 downto 0) := (others => '0');
    signal sw_imy_p, sw_imy_c : unsigned(7 downto 0) := (others => '0');
    signal sw_dk_p, sw_dk_c   : unsigned(6 downto 0) := (others => '0');
    signal sw_ut_p, sw_ut_c   : unsigned(7 downto 0) := (others => '0');

    -- palette anchors: clear -> glacial cyan-white -> teal -> flood brown
    type t_pal is array (0 to 3) of unsigned(9 downto 0);
    constant C_PAL_Y : t_pal := (to_unsigned(0, 10),   to_unsigned(940, 10),
                                 to_unsigned(560, 10), to_unsigned(420, 10));
    constant C_PAL_U : t_pal := (to_unsigned(512, 10), to_unsigned(588, 10),
                                 to_unsigned(560, 10), to_unsigned(470, 10));
    constant C_PAL_V : t_pal := (to_unsigned(512, 10), to_unsigned(452, 10),
                                 to_unsigned(420, 10), to_unsigned(600, 10));

    -- barrel shift with off code (fs = 15)
    function f_bsh(h : unsigned(9 downto 0); fs : unsigned(3 downto 0))
        return unsigned is
        variable r : unsigned(9 downto 0);
    begin
        if fs = "1111" then
            r := (others => '0');
        else
            r := shift_right(h, to_integer(fs(3 downto 0)));
        end if;
        return r;
    end function;

    -- 16-bit add-shift mixers. Adds propagate carries, so three staged
    -- rounds give near-ideal avalanche on counter-like seeds (measured:
    -- threshold adjacency 0.41/0.49 vs ideal 0.5); pure xorshift on such
    -- seeds produced period-2 column combs and row banding.
    function f_mxa(x : unsigned(15 downto 0)) return unsigned is
        variable a : unsigned(15 downto 0);
    begin
        a := x + shift_left(x, 5);          -- *33
        return a xor shift_right(a, 7);
    end function;

    function f_mxb(x : unsigned(15 downto 0)) return unsigned is
        variable a : unsigned(15 downto 0);
    begin
        a := x + shift_left(x, 3);          -- *9
        return a xor shift_right(a, 5);
    end function;

    function f_mxc(x : unsigned(15 downto 0)) return unsigned is
        variable a : unsigned(15 downto 0);
    begin
        a := x + shift_left(x, 7);          -- *129
        return a xor shift_right(a, 9);
    end function;

    function f_sat8(x : signed(11 downto 0)) return unsigned is
        variable r : unsigned(7 downto 0);
    begin
        if x < 0 then
            r := (others => '0');
        elsif x > to_signed(255, 12) then
            r := (others => '1');
        else
            r := unsigned(x(7 downto 0));
        end if;
        return r;
    end function;

    -- alpha "multiplier": y * a for a in 0..4 (dithered quarter crossfade)
    function f_am(y : unsigned(9 downto 0); a : unsigned(2 downto 0))
        return unsigned is
        variable r : unsigned(11 downto 0);
    begin
        case a is
            when "000"  => r := (others => '0');
            when "001"  => r := resize(y, 12);
            when "010"  => r := resize(y & '0', 12);
            when "011"  => r := resize(y, 12) + resize(y & '0', 12);
            when others => r := y & "00";
        end case;
        return r;
    end function;

begin

    ----------------------------------------------------------------------
    -- Control decode (used only inside sequencers / registered stages)
    ----------------------------------------------------------------------
    k_crest  <= unsigned(registers_in(0));
    k_flow   <= unsigned(registers_in(1));
    k_plunge <= unsigned(registers_in(2));
    k_churn  <= unsigned(registers_in(3));
    k_mist   <= unsigned(registers_in(4));
    k_depths <= unsigned(registers_in(5));
    sw_gates  <= registers_in(6)(0);
    sw_ut     <= registers_in(6)(1);
    sw_basin  <= registers_in(6)(2);
    sw_surge  <= registers_in(6)(3);
    sw_freeze <= registers_in(6)(4);
    k_slider <= unsigned(registers_in(7));

    ----------------------------------------------------------------------
    -- p_in: E1 input registers, counters, line sequencer, delay SRs
    ----------------------------------------------------------------------
    p_in : process(clk)
        variable v_wt : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y    <= unsigned(data_in.y);
            s_in_u    <= unsigned(data_in.u);
            s_in_v    <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_new_frame <= '0';

            if data_in.avid = '1' then
                s_had_avid <= '1';
                s_x <= s_x + 1;
            end if;

            --------------------------------------------------------------
            -- line sequencer (kicked at hsync fall, done in hblank)
            --------------------------------------------------------------
            case to_integer(s_lstep) is
                when 0 =>
                    s_y8  <= s_line_y(10 downto 3);
                    s_yb1 <= resize(shift_right(s_active_h - s_line_y, 1), 10);
                    s_ymid <= '0';
                    if s_line_y > s_hmid then
                        s_ymid <= '1';
                    end if;
                    s_at_surf <= '0';
                    if sw_basin = '1' and s_line_y = s_surface then
                        s_at_surf <= '1';
                    end if;
                    if sw_basin = '1' and s_line_y >= s_surface then
                        s_in_basin <= '1';
                        s_depth    <= s_depth + 1;
                    else
                        s_in_basin <= '0';
                        s_depth    <= (others => '0');
                    end if;
                    s_wobx <= (resize(s_line_y(7 downto 0), 16) xor
                               shift_left(resize(s_frame(8 downto 1), 16), 8))
                              xor x"5A17";
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_wobx  <= f_mxa(s_wobx);
                    s_lstep <= s_lstep + 1;
                when 2 =>
                    s_wobx  <= f_mxb(s_wobx);
                    s_lstep <= s_lstep + 1;
                when 3 =>
                    v_wt   := f_mxc(s_wobx);
                    if s_fe_sh = "111" then
                        s_wob <= (others => '0');
                    else
                        s_wob <= shift_right(v_wt(15 downto 8),
                                             to_integer(s_fe_sh));
                    end if;
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    s_mi   <= s_depth(0) xor s_wob(5);
                    s_hoff <= signed('0' & s_wob(4 downto 2)) - to_signed(4, 4);
                    -- reflection darkens with depth: dsh = min(4, 1+d/32)
                    if s_depth < 32 then
                        s_bs_dsh <= to_unsigned(1, 3);
                    elsif s_depth < 64 then
                        s_bs_dsh <= to_unsigned(2, 3);
                    elsif s_depth < 96 then
                        s_bs_dsh <= to_unsigned(3, 3);
                    else
                        s_bs_dsh <= to_unsigned(4, 3);
                    end if;
                    s_crn_line <= '0';
                    if s_depth < resize(s_crown_h, 10) then
                        s_crn_line <= '1';
                    end if;
                    -- deep-water fade quarters + surface highlight line
                    if s_depth >= 48 then
                        s_bs_q <= "11";
                    else
                        s_bs_q <= s_depth(5 downto 4);
                    end if;
                    s_wline <= '0';
                    if s_in_basin = '1' and s_depth = 1 then
                        s_wline <= '1';
                    end if;
                    s_lstep <= s_lstep + 1;
                when others =>
                    null;
            end case;

            -- hsync fall: next line
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_x <= (others => '0');
                if s_had_avid = '1' then
                    s_line_y <= s_line_y + 1;
                end if;
                s_had_avid <= '0';
                s_lstep <= (others => '0');
            end if;

            -- vsync fall: new frame/field (active height = counted lines;
            -- never latch height AT vsync from a zeroed counter)
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                if s_line_y /= 0 then
                    s_active_h <= s_line_y;
                end if;
                s_line_y <= (others => '0');
                s_depth  <= (others => '0');
                s_in_basin <= '0';
                s_new_frame <= '1';
            end if;

            -- sync/dry shift registers
            s_y_sr(0)  <= data_in.y;
            s_u_sr(0)  <= data_in.u;
            s_v_sr(0)  <= data_in.v;
            s_hs_sr(0) <= data_in.hsync_n;
            s_vs_sr(0) <= data_in.vsync_n;
            s_fd_sr(0) <= data_in.field_n;
            s_av_sr(0) <= data_in.avid;
            for i in 1 to 2 loop
                s_y_sr(i) <= s_y_sr(i - 1);
            end loop;
            for i in 1 to 4 loop
                s_u_sr(i) <= s_u_sr(i - 1);
                s_v_sr(i) <= s_v_sr(i - 1);
            end loop;
            for i in 1 to C_LAT - 1 loop
                s_hs_sr(i) <= s_hs_sr(i - 1);
                s_vs_sr(i) <= s_vs_sr(i - 1);
                s_fd_sr(i) <= s_fd_sr(i - 1);
                s_av_sr(i) <= s_av_sr(i - 1);
            end loop;

            -- basin / crown line flags ride beside the pixel pipe
            bsn_d(0) <= s_in_basin;
            crn_d(0) <= s_crn_line;
            for i in 1 to 7 loop
                bsn_d(i) <= bsn_d(i - 1);
                crn_d(i) <= crn_d(i - 1);
            end loop;
        end if;
    end process p_in;

    ----------------------------------------------------------------------
    -- p_ppipe: SPILL ENGINE -- pair-cadence R-pipeline (noise shaper +
    -- render terms) and the live state-update ALU.
    ----------------------------------------------------------------------
    p_ppipe : process(clk)
        variable v_sc   : unsigned(3 downto 0);
        variable v_yy   : unsigned(15 downto 0);
        variable v_csh  : unsigned(2 downto 0);
        variable v_seed : unsigned(15 downto 0);
        variable v_cth  : signed(11 downto 0);
        variable v_opc, v_cap, v_op, v_fm, v_w7 : unsigned(7 downto 0);
        variable v_foamy : std_logic;
        variable v_wl   : signed(11 downto 0);
        variable v_d    : signed(8 downto 0);
        variable v_sh   : unsigned(3 downto 0);
        variable v_t12  : signed(11 downto 0);
        variable v_w2   : unsigned(7 downto 0);
        variable v_wet  : std_logic;
        variable v_rock : std_logic;
        variable v_land : std_logic;
        variable v_cd   : signed(8 downto 0);
        variable v_u10  : unsigned(9 downto 0);
        variable v_fog  : unsigned(7 downto 0);
        variable v_lfoam, v_limy, v_lut : unsigned(7 downto 0);
        variable v_t9   : unsigned(8 downto 0);
        variable v_du   : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- pair pipe advances only during active video: free-running
            -- through hblank it re-labels stale data and (with the ALU
            -- firing on a frozen phase) repeatedly clobbers the tail
            -- columns -- the right-edge garbage band
            en_pair <= s_x(0) and s_in_avid;

            ----------------------------------------------------------------
            -- R stages advance at pair cadence
            ----------------------------------------------------------------
            if en_pair = '1' then
                -- R1: capture the EARLY (st_wv) read. The dout present now
                -- belongs to the address registered 2 clks ago =
                -- s_x/2 + C_PF - 1; the label MUST match the data's column
                -- or state migrates sideways one column per line.
                -- (st_uv/st_fi/st_iy/ut are read on the LATE port and land
                -- directly at R5 -- no pipeline copies.)
                a_r1 <= s_x(10 downto 1) + to_unsigned(C_PF - 1, 10);
                v_r1 <= s_in_avid;
                r_w    <= unsigned(s_wv_rd(15 downto 8));
                r_v    <= unsigned(s_wv_rd(7 downto 0));

                -- R2: velocity-scaled scroll + stretch cells.
                -- pcell = cell>>2 and y8 = y4>>1, derived downstream: no
                -- extra barrel shifters, no extra pipeline registers.
                a_r2 <= a_r1; v_r2 <= v_r1;
                r2_w <= r_w; r2_v <= r_v;
                v_sc := "0011" - ("00" & r_v(7 downto 6));  -- scroll shift 3..0
                v_yy := resize(s_line_y, 16)
                      + shift_right(s_fall_ph, to_integer(v_sc));
                v_csh := "010" + ("0" & r_v(7 downto 6));   -- cell shift 2..5
                r2_cell  <= resize(shift_right(v_yy, to_integer(v_csh)), 11);
                r2_y4    <= resize(shift_right(v_yy, 2), 11);

                -- R3: hash round A (seed mixes)
                a_r3 <= a_r2; v_r3 <= v_r2;
                r3_w <= r2_w; r3_v <= r2_v;
                v_seed := (resize(a_r2, 16)
                           xor shift_left(resize(r2_cell, 16), 5)) xor x"9E37";
                h_fil_a <= f_mxa(v_seed);
                v_seed := (resize(a_r2, 16)
                           xor shift_left(resize(r2_cell(10 downto 2), 16), 5))
                          xor x"A1A1";
                h_pf_a <= f_mxa(v_seed);
                v_seed := (resize(a_r2, 16)
                           xor shift_left(resize(r2_y4, 16), 5))
                          xor shift_left(resize(s_nph, 16), 10) xor x"5AC3";
                h_n_a <= f_mxa(v_seed);

                -- R4: hash round B
                a_r4 <= a_r3; v_r4 <= v_r3;
                r4_w <= r3_w; r4_v <= r3_v;
                h_fil_b <= f_mxb(h_fil_a);
                h_pf_b  <= f_mxb(h_pf_a);
                h_n_b   <= f_mxb(h_n_a);

                -- R4X: hash round C for the threshold-compare noises (pf, n);
                -- fil is a gradient use and 2 rounds suffice. Foam clusters
                -- (f2) ride the filament hash's mid-bits: clumps stretch and
                -- scroll with the filament cells, which is what real foam
                -- does, and it saves a whole hash chain.
                a_r4x <= a_r4; v_r4x <= v_r4;
                r4x_w <= r4_w; r4x_v <= r4_v;
                r4_fil <= h_fil_b(15 downto 8);
                r4_pf  <= f_mxc(h_pf_b)(15 downto 8);
                r4_n   <= f_mxc(h_n_b)(15 downto 8);
                r4_f2  <= h_fil_b(11 downto 4);

                -- R5: coverage / alpha / foam / glint / mist / undertow.
                -- The late-port douts (uv/fi/iy/ut) land at this latch.
                v_lfoam := unsigned(s_fi_rd(15 downto 8));
                v_limy  := unsigned(s_iy_rd(15 downto 8));
                v_lut   := unsigned(s_ut_rd);
                a_r5 <= a_r4x; v_r5 <= v_r4x;
                r5_w <= r4x_w; r5_v <= r4x_v; r5_foam <= v_lfoam;
                r5_cu <= unsigned(s_uv_rd(15 downto 8));
                r5_cv <= unsigned(s_uv_rd(7 downto 0));
                r5_dp <= s_iy_rd(7);
                r5_dk <= unsigned(s_iy_rd(6 downto 0));
                r5_imy <= v_limy;
                r5_ims <= unsigned(s_fi_rd(7 downto 0));
                r5_n <= r4_n; r5_fil <= r4_fil;

                -- continuity: coverage ~ flux/velocity. GRADED opacity from
                -- the threshold MARGIN (soft filament edges, not the on/off
                -- barcode that read as a glitch). R5 computes the three
                -- clipped legs in PARALLEL; R6 does the mins/overrides --
                -- one serial chain here was itself a critical cone.
                if r4x_w > to_unsigned(240, 8) then
                    v_cth := to_signed(240, 12);
                else
                    v_cth := signed(resize(r4x_w, 12));
                end if;
                v_cth := v_cth - signed(resize(r4x_v(7 downto 1), 12))
                       - signed(resize(r4_pf, 12));
                if v_cth <= 0 then
                    r5_opc <= (others => '0');
                elsif v_cth >= to_signed(64, 12) then
                    r5_opc <= (others => '1');
                else
                    r5_opc <= unsigned(v_cth(5 downto 0)) & "00";
                end if;
                if r4x_w <= 8 then
                    r5_cap <= (others => '0');
                elsif r4x_w >= to_unsigned(136, 8) then
                    r5_cap <= (others => '1');
                else
                    v_w7 := r4x_w - 8;
                    r5_cap <= v_w7(6 downto 0) & '0';
                end if;
                -- foam leg: whiteness graded by the cluster margin
                r5_fm <= (others => '0');
                if v_lfoam > r4_f2 then
                    v_w7 := v_lfoam - r4_f2;
                    if v_w7 >= 128 then
                        r5_fm <= (others => '1');
                    else
                        r5_fm <= v_w7(6 downto 0) & '0';
                    end if;
                end if;
                r5_glassy <= '0';
                if r4x_v < 64 and r4_f2 >= v_lfoam then
                    r5_glassy <= '1';
                end if;
                v_wl := to_signed(220, 12)
                      + signed(resize(r4x_w & "00", 12))
                      - signed(resize(r4_fil(5 downto 0) & '0', 12));
                if v_wl < 0 then
                    r5_wl <= (others => '0');
                elsif v_wl > to_signed(1023, 12) then
                    r5_wl <= (others => '1');
                else
                    r5_wl <= unsigned(v_wl(9 downto 0));
                end if;

                -- freeze glint threads
                r5_glint <= '0';
                if s_fe > 128 and v_cth > 0 and r4x_w > 24 and
                   r4_fil(4 downto 0) = "00000" and r4_n < 128 then
                    r5_glint <= '1';
                end if;

                -- mist band above last frame's impact
                v_d := signed('0' & v_limy) - signed('0' & s_y8);
                r5_fogen <= '0';
                r5_fogsh <= (others => '0');
                if s_mist_on = '1' and v_d >= 0 and
                   v_d < signed(resize(s_mband, 9)) then
                    -- band-edge dither (+0/+1 on d) kills the halo terraces
                    v_du := unsigned(v_d(7 downto 0))
                          + ("0000000" & r4_n(0 downto 0));
                    v_sh := ("0" & v_du(3 downto 1))
                          + ("00" & s_mist_sh) + 1;
                    if v_sh > to_unsigned(7, 4) then
                        v_sh := to_unsigned(7, 4);
                    end if;
                    r5_fogen <= '1';
                    r5_fogsh <= v_sh(2 downto 0);
                end if;

                -- undertow tendril membership (2-line units + ragged top)
                r5_utm <= '0';
                if sw_ut = '1' and
                   s_yb1 < (resize(v_lut, 10)
                            + resize(r4_n(7 downto 4), 10)) then
                    r5_utm <= '1';
                end if;

                -- R6: hold bank for the E-pipe; combine the opacity legs
                v_r6 <= v_r5;
                a_r6 <= a_r5;
                v_op := r5_opc;
                if r5_cap < v_op then
                    v_op := r5_cap;
                end if;
                if r5_glassy = '1' and v_op > to_unsigned(160, 8) then
                    v_op := to_unsigned(160, 8);
                end if;
                v_fm := r5_fm;
                if r5_cap < v_fm then
                    v_fm := r5_cap;
                end if;
                v_foamy := '0';
                if v_fm /= 0 and r5_n < 176 and r5_w > 8 and
                   v_op > 32 then
                    v_foamy := '1';
                    if v_fm > v_op then
                        v_op := v_fm;
                    end if;
                end if;
                g_op <= v_op;
                g_margin <= '0';
                if a_r5 < 7 then
                    g_margin <= '1';   -- prefetch-less left edge: dry
                end if;
                if v_foamy = '1' then
                    g_wl <= to_unsigned(800, 10)
                          + resize(r5_n(4 downto 0) & "00", 10);
                else
                    g_wl <= r5_wl;
                end if;
                -- water chroma: carried colour vs DEPTHS tint (cw quarters)
                case s_cw is
                    when "100" =>
                        v_u10 := r5_cu & "00";
                    when "010" =>
                        v_u10 := resize(r5_cu & '0', 10)
                               + resize(s_tintU(9 downto 1), 10);
                    when others =>
                        v_u10 := resize(r5_cu, 10) + s_tint34u;
                end case;
                if v_foamy = '1' then
                    v_u10 := to_unsigned(512, 10)
                           + unsigned(resize(shift_right(signed('0' & v_u10)
                                     - to_signed(512, 11), 2), 10));
                end if;
                g_wcu <= v_u10;
                case s_cw is
                    when "100" =>
                        v_u10 := r5_cv & "00";
                    when "010" =>
                        v_u10 := resize(r5_cv & '0', 10)
                               + resize(s_tintV(9 downto 1), 10);
                    when others =>
                        v_u10 := resize(r5_cv, 10) + s_tint34v;
                end case;
                if v_foamy = '1' then
                    v_u10 := to_unsigned(512, 10)
                           + unsigned(resize(shift_right(signed('0' & v_u10)
                                     - to_signed(512, 11), 2), 10));
                end if;
                g_wcv <= v_u10;
                g_foamy <= v_foamy;
                g_glint <= r5_glint;
                g_utm   <= r5_utm;
                g_n     <= r5_n;
                -- fog luma lift (min(384, fog<<3)) + sparkle + crown noise
                if r5_fogen = '1' then
                    v_fog := shift_right(r5_ims, to_integer(r5_fogsh));
                else
                    v_fog := (others => '0');
                end if;
                if v_fog > to_unsigned(48, 8) then
                    g_fogl <= to_unsigned(384, 9);
                else
                    g_fogl <= resize(v_fog & "000", 9);
                end if;
                g_spark <= '0';
                if v_fog > 8 and
                   (r5_n xor r5_fil) < resize(v_fog(7 downto 1), 8) and
                   (r5_n xor r5_fil) < to_unsigned(24, 8) then
                    g_spark <= '1';
                end if;
                g_crownn <= '0';
                if r5_n < s_crown then
                    g_crownn <= '1';
                end if;
                -- pre-update state view for the ALU
                g_w <= r5_w; g_v <= r5_v; g_foam <= r5_foam;
                g_cu <= r5_cu; g_cv <= r5_cv; g_ims <= r5_ims;
                g_imy <= r5_imy; g_dp <= r5_dp; g_dk <= r5_dk;
                -- decayed flux (and its rock-attenuated variant), aeration
                -- gain, decayed foam and bumped velocity -- all pair-rate
                v_t12 := signed(resize(r5_w, 12))
                       - signed(resize(shift_right(r5_w,
                                                   to_integer(s_dsh)), 12))
                       - signed(resize(s_leak, 12));
                if v_t12 < 0 then
                    v_t12 := (others => '0');
                end if;
                g_wdec <= unsigned(v_t12(7 downto 0));
                g_wdro <= unsigned(v_t12(7 downto 0))
                        - unsigned("00" & v_t12(7 downto 2));
                g_vcs  <= shift_right(r5_v, to_integer(s_cs));
                g_fdec <= r5_foam - ("0000" & r5_foam(7 downto 4));
                if r5_v > to_unsigned(255, 8) - resize(s_g, 8) then
                    g_vnx <= (others => '1');
                else
                    g_vnx <= r5_v + resize(s_g, 8);
                end if;
                -- decision flags: everything that doesn't need the live
                -- pixel collapses to single registered bits here
                g_wet <= '0';
                if r5_w > 8 then
                    g_wet <= '1';
                end if;
                g_wgims <= '0';
                if r5_w > r5_ims then
                    g_wgims <= '1';
                end if;
                g_rockp <= '0';
                if r5_dp = '1' and r5_w > 96 and s_in_basin = '0' then
                    g_rockp <= '1';
                end if;
                g_dkok <= '0';
                if sw_ut = '1' and s_ymid = '1' and r5_dk /= "1111111" then
                    g_dkok <= '1';
                end if;
                g_dk1 <= r5_dk + 1;
                a_alu <= a_r6;
                v_alu <= v_r6;
            end if;

            ----------------------------------------------------------------
            -- live even-pixel capture (E2/E3 phases) for the state ALU
            ----------------------------------------------------------------
            ev_go <= '0';
            if e3_ph = '0' and av_e3 = '1' then  -- even ACTIVE pixel at E3
                ev_h0  <= '0';
                if e3_h = 0 then
                    ev_h0 <= '1';
                end if;
                ev_dq  <= e3_dq8;
                ev_d220 <= e3_d220;
                ev_d200 <= e3_d200;
                ev_u8  <= e3_u8;
                ev_v8  <= e3_v8;
                ev_go  <= '1';
            end if;

            ----------------------------------------------------------------
            -- state-update ALU: fires once per pair, writes back
            ----------------------------------------------------------------
            p_we <= '0';
            if ev_go = '1' and v_alu = '1' then
                v_wet := g_wet;
                -- rock: ledge lip (one-shot) attenuates + records impact
                v_rock := g_rockp and ev_h0 and ev_d220;
                v_land := s_at_surf;
                -- w' = precomputed decayed flux (+rock variant) + dq
                if v_rock = '1' then
                    v_t9 := resize(g_wdro, 9) + resize(ev_dq, 9);
                else
                    v_t9 := resize(g_wdec, 9) + resize(ev_dq, 9);
                end if;
                if v_t9(8) = '1' then
                    v_w2 := (others => '1');
                else
                    v_w2 := v_t9(7 downto 0);
                end if;
                -- velocity / foam from precomputed pair-rate terms
                if v_wet = '1' then
                    p_wv_wd(7 downto 0) <= std_logic_vector(g_vnx);
                else
                    p_wv_wd(7 downto 0) <= std_logic_vector(to_unsigned(16, 8));
                end if;
                p_wv_wd(15 downto 8) <= std_logic_vector(v_w2);
                if v_wet = '1' then
                    v_t9 := resize(g_fdec, 9) + resize(g_vcs, 9);
                else
                    v_t9 := resize(g_fdec, 9);
                end if;
                if v_t9(8) = '1' then
                    p_fi_wd(15 downto 8) <= (others => '1');
                else
                    p_fi_wd(15 downto 8) <= std_logic_vector(v_t9(7 downto 0));
                end if;
                -- impact record: strongest wins
                if (v_rock = '1' or v_land = '1') and g_wgims = '1' then
                    p_fi_wd(7 downto 0) <= std_logic_vector(g_w);
                    p_iy_wd(15 downto 8) <= std_logic_vector(s_y8);
                else
                    p_fi_wd(7 downto 0) <= std_logic_vector(g_ims);
                    p_iy_wd(15 downto 8) <= std_logic_vector(g_imy);
                end if;
                -- ledge-lip bit + undertow dark accumulator
                if ev_dq > 2 then
                    p_iy_wd(7) <= '1';
                else
                    p_iy_wd(7) <= '0';
                end if;
                if g_dkok = '1' and ev_d200 = '1' then
                    p_iy_wd(6 downto 0) <= std_logic_vector(g_dk1);
                else
                    p_iy_wd(6 downto 0) <= std_logic_vector(g_dk);
                end if;
                -- carried colour chases the source while discharging
                if ev_dq > 2 then
                    v_cd := signed('0' & ev_u8) - signed('0' & g_cu);
                    p_uv_wd(15 downto 8) <= std_logic_vector(
                        g_cu + unsigned(resize(shift_right(v_cd, 2), 8)));
                    v_cd := signed('0' & ev_v8) - signed('0' & g_cv);
                    p_uv_wd(7 downto 0) <= std_logic_vector(
                        g_cv + unsigned(resize(shift_right(v_cd, 2), 8)));
                else
                    p_uv_wd(15 downto 8) <= std_logic_vector(g_cu);
                    p_uv_wd(7 downto 0)  <= std_logic_vector(g_cv);
                end if;
                p_wr_a <= a_alu;
                p_we   <= '1';
                -- spill volume (basin level + crown)
                if s_vol /= x"FFFF" then
                    s_vol <= s_vol + resize(ev_dq(7 downto 4), 16);
                end if;
            end if;

            if s_new_frame = '1' then
                s_vol_latch <= s_vol;
                s_vol <= (others => '0');
            end if;
        end if;
    end process p_ppipe;

    ----------------------------------------------------------------------
    -- p_epipe: streaming compositor E2..E7
    ----------------------------------------------------------------------
    p_epipe : process(clk)
        variable v_h11  : signed(11 downto 0);
        variable v_dqw  : unsigned(10 downto 0);
        variable v_t12  : signed(12 downto 0);
        variable v_y12  : unsigned(11 downto 0);
        variable v_a    : unsigned(2 downto 0);
        variable v_wl   : unsigned(9 downto 0);
        variable v_ry11 : unsigned(10 downto 0);
        variable v_u12  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            --------------------------------------------------------------
            -- E1 -> E2: head above the effective crest (weir + gate lift)
            --------------------------------------------------------------
            s_glift_r <= s_gate_lift(to_integer(s_x(10 downto 8)));
            e2_ph <= s_x(0);
            av_e2 <= s_in_avid;
            av_e3 <= av_e2;
            v_h11 := signed(resize(s_in_y, 12))
                   - signed(resize(s_crest_base, 12))
                   - signed(resize(s_glift_r & "00", 12));
            if v_h11 < 0 then
                e2_h <= (others => '0');
            elsif v_h11 > to_signed(1023, 12) then
                e2_h <= (others => '1');
            else
                e2_h <= unsigned(v_h11(9 downto 0));
            end if;
            e2_shim <= '0';
            if s_in_y > s_crest_lo and s_in_y < s_crest_hi then
                e2_shim <= '1';
            end if;
            e2_d220 <= '0';
            if s_in_y < to_unsigned(220, 10) then
                e2_d220 <= '1';
            end if;
            e2_d200 <= '0';
            if s_in_y < to_unsigned(200, 10) then
                e2_d200 <= '1';
            end if;

            --------------------------------------------------------------
            -- E2 -> E3: discharge (two-term shift pair) + drain + roll
            --------------------------------------------------------------
            e3_ph <= e2_ph;
            v_dqw := resize(f_bsh(e2_h, s_fs0), 11)
                   + resize(f_bsh(e2_h, s_fs1), 11);
            if v_dqw > to_unsigned(255, 11) then
                e3_dq8 <= (others => '1');
            else
                e3_dq8 <= v_dqw(7 downto 0);
            end if;
            -- drain at 3/4 strength: keeps structure alive in spilling
            -- regions instead of posterized flats (the curtain covers it)
            if resize(v_dqw - shift_right(v_dqw, 2), 11)
               > resize(e2_h, 11) then
                e3_drain <= e2_h;
            else
                e3_drain <= resize(v_dqw - shift_right(v_dqw, 2), 10);
            end if;
            e3_h <= e2_h;
            e3_shim <= e2_shim;
            e3_d220 <= e2_d220;
            e3_d200 <= e2_d200;
            e3_u8 <= unsigned(s_u_sr(1)(9 downto 2));
            e3_v8 <= unsigned(s_v_sr(1)(9 downto 2));
            e3_roll <= '0';
            if e2_h > 0 and v_dqw /= 0 and
               e2_h < (to_unsigned(8, 10) + resize(g_n(3 downto 0), 10)) then
                e3_roll <= '1';
            end if;

            --------------------------------------------------------------
            -- E3 -> E4: base luma (drain toward crest / armed shimmer)
            --------------------------------------------------------------
            e4_roll <= e3_roll;
            v_t12 := signed(resize(unsigned(s_y_sr(2)), 13));
            if s_armed = '1' then
                if e3_shim = '1' then
                    v_t12 := v_t12 + signed(resize(g_n(2 downto 0) & "00", 13))
                           - to_signed(16, 13);
                end if;
            else
                v_t12 := v_t12 - signed(resize(e3_drain, 13));
            end if;
            if v_t12 < 0 then
                e4_yb <= (others => '0');
            elsif v_t12 > to_signed(1023, 13) then
                e4_yb <= (others => '1');
            else
                e4_yb <= unsigned(v_t12(9 downto 0));
            end if;

            --------------------------------------------------------------
            -- E4 -> E5a: ALPHA SELECT only. Quarter base + pixel-rate noise
            -- dither of the 6-bit fraction (at 60 fps it reads as a smooth
            -- 8-bit blend; the grain reads as aeration). The dither bits are
            -- locally re-registered (s_dith6) so the LFSR's cross-die net
            -- stays out of this cone, and the f_am products moved to their
            -- own stage -- select+mux+sum in one clock was the routed
            -- critical path.
            s_dith6 <= s_lfsr(5 downto 0);
            v_a := resize(g_op(7 downto 6), 3);
            if g_op(5 downto 0) > s_dith6 then
                v_a := v_a + 1;
            end if;
            v_wl := g_wl;
            if e4_roll = '1' then
                v_a  := "100";
                v_wl := to_unsigned(1000, 10);
            end if;
            if g_margin = '1' then
                v_a := "000";
            end if;
            e5_a   <= v_a;
            e5_wls <= v_wl;
            e5a_yb <= e4_yb;
            e5a_glint <= g_glint;
            e5a_spark <= g_spark;
            e5a_utm   <= g_utm;
            e5a_fogl  <= g_fogl;
            e5a_crownn <= g_crownn;
            e5a_wcu <= g_wcu;
            e5a_wcv <= g_wcv;

            -- E5a -> E5b: crossfade partial products from the REGISTERED
            -- alpha select
            e5_py <= resize(e5a_yb & "00", 12) - f_am(e5a_yb, e5_a)
                   + f_am(e5_wls, e5_a);
            -- chroma crossfade at 3 levels (src / average / water)
            case e5_a is
                when "000" =>
                    e5_pu <= unsigned(s_u_sr(4));
                    e5_pv <= unsigned(s_v_sr(4));
                when "011" | "100" =>
                    e5_pu <= e5a_wcu;
                    e5_pv <= e5a_wcv;
                when others =>
                    e5_pu <= resize((resize(unsigned(s_u_sr(4)), 11)
                                     + resize(e5a_wcu, 11)) srl 1, 10);
                    e5_pv <= resize((resize(unsigned(s_v_sr(4)), 11)
                                     + resize(e5a_wcv, 11)) srl 1, 10);
            end case;
            e5_glint <= e5a_glint;
            e5_spark <= e5a_spark;
            e5_utm   <= e5a_utm;
            e5_fogl  <= e5a_fogl;
            e5_crownn <= e5a_crownn;

            --------------------------------------------------------------
            -- E5 -> E6: blend, glint, mist
            --------------------------------------------------------------
            v_y12 := resize(e5_py(11 downto 2), 12) + resize(e5_fogl, 12);
            if e5_glint = '1' or e5_spark = '1' then
                e6_y <= (others => '1');
            elsif v_y12 > to_unsigned(1023, 12) then
                e6_y <= (others => '1');
            else
                e6_y <= v_y12(9 downto 0);
            end if;
            -- chroma: heavy mist pulls toward grey (glint is luma-only)
            v_u12 := signed(resize(e5_pu, 13));
            if e5_fogl > to_unsigned(128, 9) then
                v_u12 := to_signed(512, 13)
                       + shift_right(v_u12 - to_signed(512, 13), 1);
            end if;
            e6_u <= unsigned(v_u12(9 downto 0));
            v_u12 := signed(resize(e5_pv, 13));
            if e5_fogl > to_unsigned(128, 9) then
                v_u12 := to_signed(512, 13)
                       + shift_right(v_u12 - to_signed(512, 13), 1);
            end if;
            e6_v <= unsigned(v_u12(9 downto 0));
            e6_utm <= e5_utm;
            e6_crownn <= e5_crownn;

            --------------------------------------------------------------
            -- E6 -> E7: undertow, basin override, roar, output regs
            --------------------------------------------------------------
            v_y12 := resize(e6_y, 12);
            v_u12 := signed(resize(e6_u, 13));
            if e6_utm = '1' and bsn_d(6) = '0' then
                v_y12 := "00" & v_y12(11 downto 2);
                v_u12 := to_signed(512, 13)
                       + shift_right(v_u12 - to_signed(512, 13), 2);
            end if;
            if bsn_d(6) = '1' then
                -- wobble-mirrored reflection (already depth-darkened at ry4)
                v_ry11 := resize(ry7 & "00", 11);
                v_y12 := resize(v_ry11, 12) + resize(s_tintY4, 12);
                if crn_d(6) = '1' and e6_crownn = '1' then
                    v_y12 := to_unsigned(1023, 12);
                    v_u12 := to_signed(512, 13)
                           + shift_right(signed(resize(s_bu, 13))
                                         - to_signed(512, 13), 2);
                else
                    v_u12 := signed(resize(s_bu, 13));
                end if;
            end if;
            v_y12 := v_y12 + resize(s_roar, 12);
            if v_y12 > to_unsigned(1023, 12) then
                s_out_y <= (others => '1');
            else
                s_out_y <= v_y12(9 downto 0);
            end if;
            s_out_u <= unsigned(v_u12(9 downto 0));
            -- V channel mirrors U handling
            v_u12 := signed(resize(e6_v, 13));
            if e6_utm = '1' and bsn_d(6) = '0' then
                v_u12 := to_signed(512, 13)
                       + shift_right(v_u12 - to_signed(512, 13), 2);
            end if;
            if bsn_d(6) = '1' then
                if crn_d(6) = '1' and e6_crownn = '1' then
                    v_u12 := to_signed(512, 13)
                           + shift_right(signed(resize(s_bv, 13))
                                         - to_signed(512, 13), 2);
                else
                    v_u12 := signed(resize(s_bv, 13));
                end if;
            end if;
            s_out_v <= unsigned(v_u12(9 downto 0));

            -- ring read data pipeline (bank select at E3 per line)
            if s_mi = '0' then
                ry3 <= unsigned(s_rg_rd0);
            else
                ry3 <= unsigned(s_rg_rd1);
            end if;
            -- depth darkening folded into an idle delay slot: (ry-ry>>d)<<2
            -- == (ry<<2)-(ry<<2)>>d, and doing it here takes the variable
            -- barrel out of the E7 basin cone (was the hd_hdmi critical path)
            ry4 <= ry3 - shift_right(ry3, to_integer(s_bs_dsh));
            -- deep-water fade in the next idle slot: the 4-line ring
            -- repetition disappears into the body of the pool
            case s_bs_q is
                when "00" =>
                    ry5 <= ry4;
                when "01" =>
                    ry5 <= resize((resize(ry4, 10) + resize(ry4 & '0', 10)
                                   + resize(s_deep8, 10)) srl 2, 8);
                when "10" =>
                    ry5 <= resize((resize(ry4, 9)
                                   + resize(s_deep8, 9)) srl 1, 8);
                when others =>
                    ry5 <= resize((resize(ry4, 10) + resize(s_deep8, 10)
                                   + resize(s_deep8 & '0', 10)) srl 2, 8);
            end case;
            -- waterline: one bright specular line at the surface
            if s_wline = '1' and ry5 < to_unsigned(190, 8) then
                ry6 <= to_unsigned(190, 8);
            else
                ry6 <= ry5;
            end if;
            ry7 <= ry6;
        end if;
    end process p_epipe;

    ----------------------------------------------------------------------
    -- p_frame: vblank sequencer, gate FSM, sweep FSM, LFSR
    ----------------------------------------------------------------------
    p_frame : process(clk)
        variable v_err  : signed(18 downto 0);
        variable v_t    : unsigned(7 downto 0);
        variable v_g18  : signed(20 downto 0);
        variable v_gain : unsigned(9 downto 0);
        variable v_fs   : unsigned(3 downto 0);
        variable v_rem  : unsigned(9 downto 0);
        variable v_pos  : unsigned(9 downto 0);
        variable v_i3   : unsigned(2 downto 0);
        variable v_lv   : unsigned(18 downto 0);
        variable v_r8   : unsigned(7 downto 0);
        variable v_env8, v_tgt8, v_tmr8 : unsigned(7 downto 0);
        variable v_d9   : signed(8 downto 0);
        variable v_sm   : unsigned(9 downto 0);
        variable v_adv  : unsigned(7 downto 0);
        variable v_x32  : unsigned(31 downto 0);
    begin
        if rising_edge(clk) then
            -- free-running xorshift32 (freezes only when fully iced)
            if s_fe /= x"FF" then
                v_x32 := s_lfsr xor shift_left(s_lfsr, 13);
                v_x32 := v_x32 xor shift_right(v_x32, 17);
                s_lfsr <= v_x32 xor shift_left(v_x32, 5);
            end if;

            -- shared multiplier: registered 13x6 partials; the final sum is
            -- COMBINATIONAL (concurrent assign below) so consumers keep the
            -- original load->read latency of two sequencer steps. A
            -- registered sum here silently fed every consumer the PREVIOUS
            -- product (the crest read a stale ~0 drop: wedge bug).
            s_mp_a <= s_ma * signed('0' & s_mb(4 downto 0));
            s_mp_b <= s_ma * resize(s_mb(10 downto 5), 6);

            if s_new_frame = '1' then
                s_vstep <= (others => '0');
                s_frame <= s_frame + 1;
            end if;

            --------------------------------------------------------------
            -- frame sequencer: one registered step per clock
            --------------------------------------------------------------
            case to_integer(s_vstep) is
                when 0 =>
                    -- FREEZE envelope (state with hysteresis)
                    if sw_freeze = '1' then
                        if s_fe > x"FD" then
                            s_fe <= x"FF";
                        else
                            s_fe <= s_fe + 2;
                        end if;
                    else
                        if s_fe < 4 then
                            s_fe <= (others => '0');
                        else
                            s_fe <= s_fe - 4;
                        end if;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    if s_fe < x"FF" then
                        s_nph <= s_frame(7 downto 0);
                    end if;
                    -- ripple attenuation shift from freeze env
                    if s_fe > 240 then
                        s_fe_sh <= "111";
                    else
                        s_fe_sh <= '0' & s_fe(7 downto 6);
                    end if;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    -- FLOODGATE spring, staged: error + damping this step
                    s_sperr <= shift_left(resize(signed('0' & k_slider), 19), 6)
                             - resize(fg_pos, 19);
                    s_sdamp <= resize(shift_right(fg_vel, 5), 20)
                             + resize(shift_right(fg_vel, 6), 20);
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    -- asymmetric stiffness (shift mux, no carry), one add
                    if s_sperr >= 0 then
                        fg_vel <= fg_vel
                                + resize(shift_right(s_sperr, 2), 20)
                                - s_sdamp;
                    else
                        fg_vel <= fg_vel
                                + resize(shift_right(s_sperr, 5), 20)
                                - s_sdamp;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    -- anti-windup: hitting a clamp zeroes the velocity, or
                    -- a fast pull pins the gate at the stop for a second
                    v_g18 := resize(fg_pos, 21)
                           + resize(shift_right(fg_vel, 3), 21);
                    if v_g18 < to_signed(-131072, 21) then
                        fg_pos <= to_signed(-131072, 18);
                        fg_vel <= (others => '0');
                    elsif v_g18 > to_signed(65472, 21) then
                        fg_pos <= to_signed(65472, 18);
                        fg_vel <= (others => '0');
                    else
                        fg_pos <= resize(v_g18, 21)(17 downto 0);
                    end if;
                    s_vstep <= s_vstep + 1;
                when 5 =>
                    -- pos_eff latch (uses this frame's pos); SURGE below
                    -- reads the previous frame's pos_eff -- a one-frame lag
                    -- on a slow envelope, invisible
                    if fg_pos < 0 then
                        v_pos := (others => '0');
                    else
                        v_pos := unsigned(fg_pos(15 downto 6));
                    end if;
                    -- slider captured in the ice; thaw crossing = the lurch
                    if s_fe > 128 then
                        if s_lat_on = '0' then
                            s_pos_lat <= v_pos;
                            s_lat_on  <= '1';
                        end if;
                        s_pos_eff <= s_pos_lat;
                    else
                        s_lat_on  <= '0';
                        s_pos_eff <= v_pos;
                    end if;
                    -- SURGE oscillator (~0.16 Hz triangle)
                    s_surge <= (others => '0');
                    if sw_surge = '1' then
                        if s_fe <= 200 then
                            s_surge_ph <= s_surge_ph + to_unsigned(700, 20);
                        end if;
                        v_t := s_surge_ph(19 downto 12);
                        if v_t < 128 then
                            s_surge <= v_t;
                        else
                            s_surge <= 255 - v_t;
                        end if;
                    end if;
                    s_armed <= '0';
                    if s_pos_eff < 40 then
                        s_armed <= '1';
                    end if;
                    s_ma <= signed(resize(s_pos_eff, 13));
                    s_mb <= signed(resize(s_pos_eff, 11));
                    s_vstep <= s_vstep + 1;
                when 6 =>
                    -- open curve while pos^2 multiplies
                    if s_pos_eff < 40 then
                        s_open_k <= (others => '0');
                    else
                        v_sm := resize(s_pos_eff - 40, 10);
                        v_sm := resize(v_sm, 10)
                              + resize(v_sm(9 downto 1), 10);   -- *1.5
                        if v_sm(9 downto 1) >= 255 then
                            s_open_k <= (others => '1');
                        else
                            s_open_k <= v_sm(8 downto 1);       -- *0.75
                        end if;
                    end if;
                    s_flow_k <= k_flow(9 downto 2);
                    s_vstep <= s_vstep + 1;
                when 7 =>
                    s_p2 <= unsigned(s_mp(19 downto 10));
                    s_vstep <= s_vstep + 1;
                when 8 =>
                    s_crest_drop <= s_p2 - resize(s_p2(9 downto 2), 10);
                    s_ma <= signed(resize(s_flow_k, 13));
                    s_mb <= signed(resize(s_open_k, 11));
                    s_vstep <= s_vstep + 1;
                when 9 =>
                    s_ma <= signed(resize(s_surge, 13));
                    s_mb <= signed(resize(s_open_k, 11));
                    s_vstep <= s_vstep + 1;
                when 10 =>
                    v_gain := resize(unsigned(s_mp(16 downto 7)), 10);
                    if v_gain > 511 then
                        s_gain <= to_unsigned(511, 10);
                    else
                        s_gain <= v_gain;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 11 =>
                    v_gain := s_gain
                            + resize(unsigned(s_mp(14 downto 7)), 10);
                    if v_gain > 1023 then
                        s_gain <= (others => '1');
                    else
                        s_gain <= v_gain;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 12 =>
                    -- two-term shift pair: 2^-fs0 + 2^-fs1 ~= gain/1024
                    -- (MSB gives fs0 = 10 - msb_index; remainder gives fs1)
                    v_fs := "1111";
                    for i in 9 downto 0 loop
                        if v_fs = "1111" and s_gain(i) = '1' then
                            v_fs := to_unsigned(10 - i, 4);
                        end if;
                    end loop;
                    if v_fs = "1111" then
                        v_rem := (others => '0');
                    else
                        v_rem := s_gain
                               - shift_right(to_unsigned(512, 10),
                                             to_integer(v_fs) - 1);
                    end if;
                    s_fs0 <= v_fs;
                    s_vstep <= s_vstep + 1;
                    -- stash remainder in s_gain (consumed next step)
                    s_gain <= v_rem;
                when 13 =>
                    v_fs := "1111";
                    for i in 9 downto 0 loop
                        if v_fs = "1111" and s_gain(i) = '1' then
                            v_fs := to_unsigned(10 - i, 4);
                        end if;
                    end loop;
                    s_fs1 <= v_fs;
                    s_vstep <= s_vstep + 1;
                when 14 =>
                    -- CREST: base threshold minus the floodgate's drop
                    v_sm := to_unsigned(96, 10)
                          + resize(k_crest(9 downto 0), 10)
                          - resize(k_crest(9 downto 3), 10);
                    if v_sm > s_crest_drop then
                        s_crest_base <= v_sm - s_crest_drop;
                    else
                        s_crest_base <= (others => '0');
                    end if;
                    s_vstep <= s_vstep + 1;
                when 15 =>
                    if s_crest_base > 24 then
                        s_crest_lo <= s_crest_base - 24;
                    else
                        s_crest_lo <= (others => '0');
                    end if;
                    if s_crest_base < 999 then
                        s_crest_hi <= s_crest_base + 24;
                    else
                        s_crest_hi <= (others => '1');
                    end if;
                    -- height class
                    if s_active_h >= 864 then
                        s_hclass <= "10";
                    elsif s_active_h >= 540 then
                        s_hclass <= "01";
                    else
                        s_hclass <= "00";
                    end if;
                    s_hmid <= '0' & s_active_h(10 downto 1);
                    s_vstep <= s_vstep + 1;
                when 16 =>
                    -- PLUNGE: gravity + streak persistence
                    s_g <= resize(k_plunge(9 downto 5), 5);
                    if k_plunge(9 downto 5) = "00000" then
                        s_g <= to_unsigned(1, 5);
                    end if;
                    v_i3 := k_plunge(9 downto 7);
                    case v_i3 is
                        when "000"  => s_dsh <= to_unsigned(4, 4);
                        when "001"  => s_dsh <= to_unsigned(5, 4);
                        when "010"  => s_dsh <= to_unsigned(6, 4);
                        when "011"  => s_dsh <= to_unsigned(7, 4);
                        when others => s_dsh <= to_unsigned(8, 4);
                    end case;
                    if s_hclass = "10" then
                        if v_i3 = "000" then
                            s_dsh <= to_unsigned(5, 4);
                        elsif v_i3 = "001" then
                            s_dsh <= to_unsigned(6, 4);
                        elsif v_i3 = "010" then
                            s_dsh <= to_unsigned(7, 4);
                        else
                            s_dsh <= to_unsigned(8, 4);
                        end if;
                    end if;
                    if v_i3 >= "010" then
                        s_leak <= "0";
                    else
                        s_leak <= "1";
                    end if;
                    s_vstep <= s_vstep + 1;
                when 17 =>
                    -- CHURN (+ surge whitening)
                    v_t := k_churn(9 downto 2);
                    if v_t > 255 - s_surge(7 downto 1) then
                        s_k4 <= (others => '1');
                    else
                        s_k4 <= v_t + s_surge(7 downto 1);
                    end if;
                    s_vstep <= s_vstep + 1;
                when 18 =>
                    if to_unsigned(8, 4) - resize(s_k4(7 downto 6), 4) < 3 then
                        s_cs <= to_unsigned(3, 4);
                    else
                        s_cs <= to_unsigned(8, 4) - resize(s_k4(7 downto 6), 4);
                    end if;
                    -- MIST
                    s_mist_on <= '0';
                    if k_mist > 8 then
                        s_mist_on <= '1';
                    end if;
                    if k_mist(9 downto 8) = "11" then
                        s_mist_sh <= "00";
                    else
                        s_mist_sh <= to_unsigned(3, 2) - k_mist(9 downto 8);
                    end if;
                    case s_hclass is
                        when "00"   => s_mband <= to_unsigned(6, 8);
                        when "01"   => s_mband <= to_unsigned(9, 8);
                        when others => s_mband <= to_unsigned(12, 8);
                    end case;
                    -- DEPTHS segment: seg = min(2, k6*3 >> 10)
                    s_k63 <= resize(k_depths, 12)
                           + resize(k_depths & '0', 12);
                    s_vstep <= s_vstep + 1;
                when 19 =>
                    if s_k63(11 downto 10) >= 2 then
                        s_seg <= "10";
                    else
                        s_seg <= s_k63(11 downto 10);
                    end if;
                    s_vstep <= s_vstep + 1;
                when 20 =>
                    s_segf <= s_k63(9 downto 2);
                    s_pal_ay <= C_PAL_Y(to_integer(s_seg));
                    s_pal_by <= C_PAL_Y(to_integer(s_seg) + 1);
                    s_pal_au <= C_PAL_U(to_integer(s_seg));
                    s_pal_bu <= C_PAL_U(to_integer(s_seg) + 1);
                    s_pal_av <= C_PAL_V(to_integer(s_seg));
                    s_pal_bv <= C_PAL_V(to_integer(s_seg) + 1);
                    s_vstep <= s_vstep + 1;
                when 21 =>
                    -- carried weight quarters: (4,2,1,1) anchors
                    if s_seg = "00" then
                        if s_segf < 128 then
                            s_cw <= "100";
                        else
                            s_cw <= "010";
                        end if;
                    elsif s_seg = "01" then
                        if s_segf < 128 then
                            s_cw <= "010";
                        else
                            s_cw <= "001";
                        end if;
                    else
                        s_cw <= "001";
                    end if;
                    s_ma <= signed(resize(s_pal_by, 13))
                          - signed(resize(s_pal_ay, 13));
                    s_mb <= signed(resize(s_segf, 11));
                    s_vstep <= s_vstep + 1;
                when 22 =>
                    s_ma <= signed(resize(s_pal_bu, 13))
                          - signed(resize(s_pal_au, 13));
                    s_mb <= signed(resize(s_segf, 11));
                    s_vstep <= s_vstep + 1;
                when 23 =>
                    s_tintY <= s_pal_ay
                             + unsigned(resize(shift_right(s_mp, 8), 10));
                    s_ma <= signed(resize(s_pal_bv, 13))
                          - signed(resize(s_pal_av, 13));
                    s_mb <= signed(resize(s_segf, 11));
                    s_vstep <= s_vstep + 1;
                when 24 =>
                    s_tintU <= s_pal_au
                             + unsigned(resize(shift_right(s_mp, 8), 10));
                    s_vstep <= s_vstep + 1;
                when 25 =>
                    s_tintV <= s_pal_av
                             + unsigned(resize(shift_right(s_mp, 8), 10));
                    s_tintY4 <= s_tintY(9 downto 2);
                    -- deep-water luma in ring (>>2) domain: (170+tintY/4)/4
                    s_deep8 <= to_unsigned(42, 8)
                             + resize(s_tintY(9 downto 4), 8);
                    -- 3/4-tint terms for the carried-colour blend
                    s_tint34u <= s_tintU - resize(s_tintU(9 downto 2), 10);
                    s_tint34v <= s_tintV - resize(s_tintV(9 downto 2), 10);
                    s_vstep <= s_vstep + 1;
                when 26 =>
                    -- basin hue: never colourless (clear keeps a cool cast)
                    case s_cw is
                        when "100" =>
                            s_bu <= to_unsigned(528, 10);
                            s_bv <= to_unsigned(476, 10);
                        when "010" =>
                            s_bu <= resize(s_tintU(9 downto 1), 10)
                                  + to_unsigned(264, 10);
                            s_bv <= resize(s_tintV(9 downto 1), 10)
                                  + to_unsigned(238, 10);
                        when others =>
                            s_bu <= s_tintU;
                            s_bv <= s_tintV;
                    end case;
                    -- roar at the very top of the slider
                    if s_pos_eff > 960 then
                        s_roar <= resize((s_pos_eff - 960) & '0', 8);
                    else
                        s_roar <= (others => '0');
                    end if;
                    s_vstep <= s_vstep + 1;
                when 27 =>
                    -- BASIN level dynamics
                    if sw_basin = '1' then
                        v_lv := s_level
                              + resize(s_vol_latch(15 downto 11), 19)
                              - resize(s_level(18 downto 6), 19);
                        if v_lv > (resize(s_active_h(10 downto 2), 11)
                                   & "00000000") then
                            s_level <= resize(s_active_h(10 downto 2), 11)
                                     & "00000000";
                        else
                            s_level <= v_lv;
                        end if;
                    else
                        if s_level > 320 then
                            s_level <= s_level - resize(s_level(18 downto 4), 19)
                                     - 64;
                        else
                            s_level <= (others => '0');
                        end if;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 28 =>
                    s_surface <= s_active_h - resize(s_level(18 downto 8), 11);
                    s_crown <= s_vol_latch(15 downto 8);
                    s_vstep <= s_vstep + 1;
                when 29 =>
                    s_crown_h <= resize(s_crown(7 downto 6), 6)
                               + to_unsigned(2, 6);
                    -- fall-phase scroll advance (freeze ramps it to zero)
                    v_adv := to_unsigned(8, 8)
                           + resize(s_pos_eff(9 downto 5), 8);
                    s_ma <= signed(resize(v_adv, 13));
                    s_mb <= signed(resize(x"FF" - s_fe, 11));
                    s_vstep <= s_vstep + 1;
                when 30 =>
                    s_vstep <= s_vstep + 1;
                when 31 =>
                    s_fall_ph <= s_fall_ph
                               + resize(unsigned(s_mp(15 downto 8)), 16);
                    s_gstep <= (others => '0');
                    s_vstep <= s_vstep + 1;
                when others =>
                    null;
            end case;

            --------------------------------------------------------------
            -- gate FSM: rotate-ring, head-only access (no index muxes or
            -- write decoders). One gate per step; 16 rotations restore
            -- positional order before active video resumes -- the pixel
            -- path only reads s_gate_lift during active lines. A gate's
            -- identity is its step number (stable across rotations).
            --------------------------------------------------------------
            if s_gstep(4) = '0' then
                if s_gstep(0) = '0' then
                    -- substep A: sample randomness + fire decision only
                    -- (splitting the LFSR->decision->chase->lift cone)
                    s_gr8 <= s_lfsr(7 downto 0)
                           xor (s_gstep(3 downto 1)
                                & s_gstep(3 downto 1) & "01");
                    s_gfire <= '0';
                    if g_tmr(0) = 0 then
                        s_gfire <= '1';
                    end if;
                else
                    -- substep B: schedule, chase, lift, rotate
                    v_env8 := g_env(0);
                    v_tgt8 := g_tgt(0);
                    v_tmr8 := g_tmr(0);
                    if sw_gates = '0' then
                        v_env8 := (others => '1');
                        v_tgt8 := (others => '1');
                    elsif s_fe <= 200 then
                        if s_gfire = '1' then
                            if s_gr8(1 downto 0) = "00" then
                                v_tgt8 := (others => '0');
                            else
                                v_tgt8 := (others => '1');
                            end if;
                            v_tmr8 := to_unsigned(90, 8)
                                    + resize(s_gr8(7 downto 1), 8);
                        else
                            v_tmr8 := v_tmr8 - 1;
                        end if;
                        -- damped mechanical travel (~1 s full swing)
                        if v_env8 < v_tgt8 then
                            if v_tgt8 - v_env8 < 4 then
                                v_env8 := v_tgt8;
                            else
                                v_env8 := v_env8 + 4;
                            end if;
                        elsif v_env8 > v_tgt8 then
                            if v_env8 - v_tgt8 < 4 then
                                v_env8 := v_tgt8;
                            else
                                v_env8 := v_env8 - 4;
                            end if;
                        end if;
                    end if;
                    for i in 0 to 6 loop
                        g_env(i) <= g_env(i + 1);
                        g_tgt(i) <= g_tgt(i + 1);
                        g_tmr(i) <= g_tmr(i + 1);
                        s_gate_lift(i) <= s_gate_lift(i + 1);
                    end loop;
                    g_env(7) <= v_env8;
                    g_tgt(7) <= v_tgt8;
                    g_tmr(7) <= v_tmr8;
                    -- closed gate raises the weir out of reach
                    s_gate_lift(7) <= x"FF" - v_env8;
                end if;
                s_gstep <= s_gstep + 1;
                if s_gstep = "01111" then
                    s_sw_run <= '1';
                    s_sw_c   <= (others => '0');
                    s_sw_ph  <= '0';
                    sw_ims_p <= (others => '0');
                    sw_ims_c <= (others => '0');
                end if;
            end if;

            --------------------------------------------------------------
            -- column sweep: state reset + mist decay/smooth + undertow
            -- chase. 2 clks per column; runs entirely inside vblank.
            --------------------------------------------------------------
            sw_we <= '0';
            if s_sw_run = '1' then
                s_sw_ph <= not s_sw_ph;
                if s_sw_ph = '0' then
                    -- issue read for column s_sw_c
                    sw_ra <= s_sw_c(9 downto 0);
                else
                    -- BRAM latency: the dout landing now is column
                    -- (s_sw_c - 1). Shift a 3-column window; the write
                    -- targets its centre, column (s_sw_c - 2).
                    sw_ims_p <= sw_ims_c;
                    sw_ims_c <= unsigned(s_fi_rd(7 downto 0));
                    sw_imy_p <= sw_imy_c;
                    sw_imy_c <= unsigned(s_iy_rd(15 downto 8));
                    sw_dk_p  <= sw_dk_c;
                    sw_dk_c  <= unsigned(s_iy_rd(6 downto 0));
                    sw_ut_p  <= sw_ut_c;
                    sw_ut_c  <= unsigned(s_ut_rd);
                    if s_sw_c >= 2 then
                        -- mist: 1/8 decay (band dither hides columniness)
                        v_r8 := sw_ims_p - ("000" & sw_ims_p(7 downto 3));
                        sw_fi_wd <= x"00" & std_logic_vector(v_r8);
                        sw_iy_wd <= std_logic_vector(sw_imy_p) & x"00";
                        -- undertow height chases the dark accumulator
                        v_d9 := signed(resize(sw_dk_p, 9))
                              - signed(resize(sw_ut_p, 9));
                        sw_ut_wd <= std_logic_vector(
                            sw_ut_p + unsigned(resize(shift_right(v_d9, 3), 8)));
                        sw_wa <= s_sw_c(9 downto 0) - 2;
                        sw_we <= '1';
                    end if;
                    if s_sw_c = to_unsigned(1025, 11) then
                        s_sw_run <= '0';
                    else
                        s_sw_c <= s_sw_c + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_frame;

    ----------------------------------------------------------------------
    -- p_ports: BRAM port register muxing (scan vs sweep), pre-muxed
    ----------------------------------------------------------------------
    -- shared-multiplier sum: registered partials -> one 24-bit add -> the
    -- consuming sequencer registers (short STA path)
    s_mp <= resize(s_mp_a, 24) + shift_left(resize(s_mp_b, 24), 5);

    p_ports : process(clk)
    begin
        if rising_edge(clk) then
            if s_sw_run = '1' then
                s_st_ra <= sw_ra;
                s_st_rb <= sw_ra;
                s_st_wa <= sw_wa;
                s_fi_wd <= sw_fi_wd;
                s_iy_wd <= sw_iy_wd;
                s_wv_wd <= x"0010";          -- w=0, v=16: frame-start reset
                s_uv_wd <= (others => '0');
                s_st_we <= sw_we;
                s_uv_we <= '0';              -- carried colour persists
                s_ut_wd <= sw_ut_wd;
                s_ut_we <= sw_we;
            else
                s_st_ra <= s_x(10 downto 1) + to_unsigned(C_PF, 10);
                -- late port: dout must land at the R5 latch labelled with
                -- the same column (derived: R5 latches column P when the
                -- pair counter is P; dout then = mem[P - 1 + offset])
                s_st_rb <= s_x(10 downto 1) + to_unsigned(1, 10);
                s_st_wa <= p_wr_a;
                s_wv_wd <= p_wv_wd;
                s_uv_wd <= p_uv_wd;
                s_fi_wd <= p_fi_wd;
                s_iy_wd <= p_iy_wd;
                s_st_we <= p_we;
                s_uv_we <= p_we;
                s_ut_we <= '0';
            end if;

            -- ring: read for the current pixel (+wobble), write the output
            s_rg_ra <= unsigned(signed(resize(s_x(10 downto 1), 10))
                                + resize(s_hoff, 10));
            s_rg_wa <= resize(shift_right(s_x - to_unsigned(C_LAT, 11), 1), 10);
            s_rg_wd <= std_logic_vector(s_out_y(9 downto 2));
            s_rg_we <= "00";
            if s_av_sr(C_LAT - 1) = '1' and bsn_d(7) = '0'
               and s_x(0) = '1' then
                if s_line_y(0) = '0' then
                    s_rg_we(0) <= '1';
                else
                    s_rg_we(1) <= '1';
                end if;
            end if;
        end if;
    end process p_ports;

    ----------------------------------------------------------------------
    -- BRAMs: canonical 1W1R
    ----------------------------------------------------------------------
    p_m_wv : process(clk)
    begin
        if rising_edge(clk) then
            s_wv_rd <= st_wv(to_integer(s_st_ra));
            if s_st_we = '1' then
                st_wv(to_integer(s_st_wa)) <= s_wv_wd;
            end if;
        end if;
    end process p_m_wv;

    p_m_uv : process(clk)
    begin
        if rising_edge(clk) then
            s_uv_rd <= st_uv(to_integer(s_st_rb));
            if s_uv_we = '1' then
                st_uv(to_integer(s_st_wa)) <= s_uv_wd;
            end if;
        end if;
    end process p_m_uv;

    p_m_fi : process(clk)
    begin
        if rising_edge(clk) then
            s_fi_rd <= st_fi(to_integer(s_st_rb));
            if s_st_we = '1' then
                st_fi(to_integer(s_st_wa)) <= s_fi_wd;
            end if;
        end if;
    end process p_m_fi;

    p_m_iy : process(clk)
    begin
        if rising_edge(clk) then
            s_iy_rd <= st_iy(to_integer(s_st_rb));
            if s_st_we = '1' then
                st_iy(to_integer(s_st_wa)) <= s_iy_wd;
            end if;
        end if;
    end process p_m_iy;

    p_m_ut : process(clk)
    begin
        if rising_edge(clk) then
            s_ut_rd <= ut_mem(to_integer(s_st_rb));
            if s_ut_we = '1' then
                ut_mem(to_integer(s_st_wa)) <= s_ut_wd;
            end if;
        end if;
    end process p_m_ut;

    p_m_rg0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rg_rd0 <= ring0(to_integer(s_rg_ra));
            if s_rg_we(0) = '1' then
                ring0(to_integer(s_rg_wa)) <= s_rg_wd;
            end if;
        end if;
    end process p_m_rg0;

    p_m_rg1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rg_rd1 <= ring1(to_integer(s_rg_ra));
            if s_rg_we(1) = '1' then
                ring1(to_integer(s_rg_wa)) <= s_rg_wd;
            end if;
        end if;
    end process p_m_rg1;

    ----------------------------------------------------------------------
    -- Output
    ----------------------------------------------------------------------
    data_out.hsync_n <= s_hs_sr(C_LAT - 1);
    data_out.vsync_n <= s_vs_sr(C_LAT - 1);
    data_out.field_n <= s_fd_sr(C_LAT - 1);
    data_out.avid    <= s_av_sr(C_LAT - 1);
    data_out.y <= std_logic_vector(s_out_y);
    data_out.u <= std_logic_vector(s_out_u);
    data_out.v <= std_logic_vector(s_out_v);

end architecture spillway;
