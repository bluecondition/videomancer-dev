-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Kagome v0.1.0 -- Hardline.  The Japanese triaxial bamboo basket weave (kagome) and chair caning: white
-- strips in three directions 60 degrees apart, woven over and under on black, each strip edged with a black
-- line so the crossings read, leaving hexagonal eyes ringed by small triangles.  Black and white only
-- (U = V = 512).  The whole basket turns slowly and glides along one strip direction.
--
--   c_k   = (p - pivot) . n_k / Period + o_k   (cycles)     n_k = unit normals at th, th + 120, th + 240 degrees
--           c_0, c_1 are affine DDAs; c_2 = (o_0 + o_1 + o_2) - c_0 - c_1 (the normals sum to zero)
--   a_k   = distance to the nearest strip centre line (0 .. 1/2 cycle); e_k = hw - a_k (positive inside)
--   weave = family 0 over 1, 1 over 2, 2 over 0; in a triple overlap the least-covered strip drops out and
--           the cyclic rule picks between the other two (a pinwheel, continuous across every boundary):
--           white_k = min(hw - L - a_k [, a_k - L/2 Double], a_{k-1} - min(hw, max(a_{k+1}, a_k + L)))
--                   (= min(e_k - L, max(-e_{k-1}, min(a_{k-1} - a_{k+1}, a_{k-1} - a_k - L))))
--           W = max_k white_k   (Fine: max(W, min_k a_k - hw), shown inverted: only the outlines)
--   cover = clamp(W x pixels per cycle + 1/2): one scale at the end, analytic 1-px anti-aliasing
--   Shade = a ramp of W itself (rounded canes that darken where they dive under the next strip)
--
--   K1  X       pans the weave, +-1024 virtual px (the turn pivot)                        (glided)
--   K2  Y       +-1024 virtual px                                                         (glided)
--   K3  Period  strip pitch 60 .. 480 virtual px, exponential                             (glided)
--   K4  Shift   slides the third family: true kagome -> lopsided eyes -> six-way knots    (glided)
--   K5  Drift   bipolar, centre = still: the weave turns (2^-16 .. 2^-8 turn per field) and glides along
--               strip 0 (8 x the turn, in cycles); the speed is glided 1/8 per field
--   K6  Camera  box-filtered luma (a square law above a knee) widens the strips           (glided)
--   S7  Glide   Off = the knob value of this field, On = 1/16 of the gap per field
--   S8  Strand  Single / Double (a black centre line: two canes per strip, chair caning)
--   S9  Shade   Flat / Shade (a triangle ramp across each cane)
--   S10 Weight  Bold (white strips) / Fine (only the outlines: a wire weave); with Shade: 4 steps / smooth
--   S11 Source  Video / Osc
--   P12 WEAVE   strip width / pitch: 0.05 (hairline star lattice) .. 0.25 (50 %) .. 0.75 (a tight surface,
--               pin-hole eyes)                                                            (glided)
--
-- Sections (the integer model .hardline_work/rb_kagome/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         +-1 hold, shared glide (7 channels), switches
--   [C] FIELD SEQUENCER                  one serial multiplier, 10 global + 2 x 8 family slots of 24 clocks
--   [D] SOURCE FIELD                     Osc lattice, gain table, 32-px box, vertical one-pole in a line store
--   [E] WEAVE                            2 DDAs (cycles, Q26), the third family, distances, the weave SDF
--   [F] COVERAGE + SHADE                 scale to pixels, coverage, the cane ramp, luma
--   [G] OUTPUT CONDITIONING              luma [1 2 1]/4, luma 13/16 toward black, blanking gate
--
-- Timing: T = the g_start clock.  The DDAs hold DDA pixel i at T + 11 + i (stage 5); a register "at stage S"
-- holds pixel i at T + 6 + i + S; the camera reaches stage 6, the luma stage 21, the output register stage 26.
-- Output column j is the [1 2 1] filter centred on DDA pixel j + 1.  One latency (32) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.kagome_pkg.all;
use work.all;

architecture kagome of program_top is
    constant C_LAT : integer := 32;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 440;
    constant C_DEF_K4  : integer := 0;
    constant C_DEF_K5  : integer := 660;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 460;

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
    -- lines per virtual px, x 32768: 256 * 32768 / ys
    function f_scy(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(7372, 16);
            when 4          => return to_unsigned(14564, 16);
            when 8          => return to_unsigned(8738, 16);
            when 12         => return to_unsigned(17476, 16);
            when 5 | 6 | 14 => return to_unsigned(21845, 16);
            when 1 | 2 | 10 => return to_unsigned(16384, 16);
            when others     => return to_unsigned(32768, 16);
        end case;
    end function;
    -- the Osc lattice's line step: (ys * 121) >> 8
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
    -- interlaced modes (field lines are two pixels tall): 480i, 576i, 1080i
    function f_il(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8 | 1 | 2 | 10 => return '1';
            when others             => return '0';
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
    function f_min(a, b : signed) return signed is
    begin
        if b < a then return b; else return a; end if;
    end function;
    function f_max(a, b : signed) return signed is
    begin
        if b > a then return b; else return a; end if;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ilr, s_il : std_logic := '0';
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
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
    signal s_gav_sr   : t_bit_shift := (others => '0');
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 9) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] knobs: +-1 hold (10 bits), glide (Q3)
    signal h_x : unsigned(9 downto 0) := to_unsigned(C_DEF_K1, 10);
    signal h_y : unsigned(9 downto 0) := to_unsigned(C_DEF_K2, 10);
    signal h_p : unsigned(9 downto 0) := to_unsigned(C_DEF_K3, 10);
    signal h_f : unsigned(9 downto 0) := to_unsigned(C_DEF_K4, 10);
    signal h_c : unsigned(9 downto 0) := to_unsigned(C_DEF_K6, 10);
    signal h_w : unsigned(9 downto 0) := to_unsigned(C_DEF_P12, 10);
    signal h_v : unsigned(9 downto 0) := to_unsigned(C_DEF_K5, 10);
    signal g_x : unsigned(12 downto 0) := to_unsigned(C_DEF_K1 * 8, 13);
    signal g_y : unsigned(12 downto 0) := to_unsigned(C_DEF_K2 * 8, 13);
    signal g_p : unsigned(12 downto 0) := to_unsigned(C_DEF_K3 * 8, 13);
    signal g_f : unsigned(12 downto 0) := to_unsigned(C_DEF_K4 * 8, 13);
    signal g_c : unsigned(12 downto 0) := to_unsigned(C_DEF_K6 * 8, 13);
    signal g_w : unsigned(12 downto 0) := to_unsigned(C_DEF_P12 * 8, 13);
    signal g_v : unsigned(12 downto 0) := to_unsigned(C_DEF_K5 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(13 downto 0) := (others => '0');
    signal s_gr, s_gh : unsigned(9 downto 0) := (others => '0');
    signal z_gp, z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_glide : std_logic := '1';
    signal s_dbl, s_shade, s_fine, s_src, s_ff : std_logic := '0';

    -- [C] ROMs
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal sn_a : unsigned(9 downto 0) := (others => '0');
    signal sn_q, sn_f : unsigned(15 downto 0) := (others => '0');

    -- [C] sequencer and its serial multiplier
    signal q_run, q_go, q_fam, q_kf : std_logic := '0';
    signal q_j  : unsigned(3 downto 0) := (others => '0');
    signal q_ph : unsigned(4 downto 0) := (others => '0');
    signal f_a, f_acc : signed(41 downto 0) := (others => '0');
    signal z_fa : signed(26 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    -- global slots
    signal z_oct  : unsigned(9 downto 0) := (others => '0');
    signal z_I    : unsigned(4 downto 0) := (others => '0');
    signal z_Lp   : unsigned(11 downto 0) := (others => '0');
    signal c_invp : unsigned(17 downto 0) := (others => '0');      -- cycles per h, Q24
    signal c_iu   : unsigned(21 downto 0) := (others => '0');      -- cycles per pixel, Q26
    signal c_iv   : unsigned(23 downto 0) := (others => '0');      -- cycles per line, Q26
    signal c_m4   : unsigned(3 downto 0) := to_unsigned(8, 4);     -- pixels per cycle = m4 / 8 * 2^E
    signal c_shaa : unsigned(3 downto 0) := to_unsigned(7, 4);     -- 13 - E (+1 interlaced)
    signal z_sc   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_scy  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_m    : unsigned(11 downto 0) := to_unsigned(1928, 12);
    signal z_w48  : unsigned(13 downto 0) := to_unsigned(7688, 14);
    signal z_pv   : signed(16 downto 0) := (others => '0');
    signal z_apx, z_apy : unsigned(15 downto 0) := (others => '0'); -- |pivot| column / row (Q3 pixels / lines)
    signal z_pxn, z_pyn : std_logic := '0';
    signal c_hw0  : unsigned(12 downto 0) := to_unsigned(1882, 13); -- strip half-width, Q14 cycles
    signal c_hw0L : unsigned(12 downto 0) := to_unsigned(1472, 13); -- hw0 - L
    signal z_iv1  : unsigned(8 downto 0) := (others => '0');
    signal c_L    : unsigned(11 downto 0) := to_unsigned(410, 12);  -- edge line width, Q14 cycles
    signal c_Lh   : unsigned(11 downto 0) := to_unsigned(205, 12);  -- L / 2 (a temporary in slot 8)
    signal r_sh_a : unsigned(5 downto 0) := (others => '0');
    signal r_sh_q, r_shf : unsigned(7 downto 0) := (others => '0');
    signal c_mq   : unsigned(3 downto 0) := to_unsigned(8, 4);     -- the cane ramp: ((W + L) * mq) >> shs
    signal c_shs  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal z_u, z_ls : unsigned(9 downto 0) := (others => '0');
    signal z_vn, z_vz : std_logic := '0';
    signal z_step : unsigned(16 downto 0) := (others => '0');
    signal c_th   : unsigned(23 downto 0) := (others => '0');      -- the turn, 2^24 per turn
    signal c_de   : unsigned(25 downto 0) := (others => '0');      -- the glide along strip 0, Q26 cycles
    signal c_S    : unsigned(13 downto 0) := to_unsigned(8192, 14); -- o_0 + o_1 + o_2, Q14
    -- family slots
    signal z_th   : unsigned(15 downto 0) := (others => '0');
    signal z_p    : unsigned(14 downto 0) := (others => '0');
    signal z_ov, z_ov1, z_sneg, z_cneg : std_logic := '0';
    signal z_fs, z_fc : unsigned(3 downto 0) := (others => '0');
    signal z_sm, z_cm : unsigned(14 downto 0) := (others => '0');
    signal z_sdf, z_cdf : unsigned(5 downto 0) := (others => '0');
    signal z_dxm  : unsigned(21 downto 0) := (others => '0');
    signal z_dy, z_seed : unsigned(25 downto 0) := (others => '0');

    -- [E] the two family DDAs (cycles, Q26): pixel step, line wrap, field seed
    type t_u26x2 is array (0 to 1) of unsigned(25 downto 0);
    signal d_inc, d_wrap, d_acc : t_u26x2 := (others => (others => '0'));
    signal l_ld : std_logic_vector(1 downto 0) := (others => '0');   -- load the field seed (z_seed) into a DDA
    signal b_run, b_end : std_logic := '0';
    signal b_cnt, b_mm1 : unsigned(11 downto 0) := (others => '0');

    -- [E] the weave (Q14 cycles; the distance terms are 15-bit signed)
    type t_u14x3 is array (0 to 2) of unsigned(13 downto 0);
    type t_u13x3 is array (0 to 2) of unsigned(12 downto 0);
    type t_s15x3 is array (0 to 2) of signed(14 downto 0);
    signal x6_c : t_u14x3 := (others => (others => '0'));
    signal x7_a, x8_a, x9_a : t_u13x3 := (others => (others => '0'));
    signal x7_aL, x8_u, x9_v : t_u14x3 := (others => (others => '0'));
    signal x7_hw, x7_hwL, x8_hw, x9_hw : unsigned(13 downto 0) := (others => '0');
    signal x8_b, x8_bd, x9_body, x10_body, x10_vis, x11_w : t_s15x3 := (others => (others => '0'));
    signal x8_m01, x9_mina : unsigned(12 downto 0) := (others => '0');
    signal x10_nem, x11_nem : signed(14 downto 0) := (others => '0');
    signal x12_w01, x12_w2, x13_W : signed(14 downto 0) := (others => '0');
    signal x14_Zc : signed(11 downto 0) := (others => '0');
    signal x14_Ws : signed(15 downto 0) := (others => '0');
    signal x15_P  : signed(15 downto 0) := (others => '0');
    signal x15_Wc : unsigned(12 downto 0) := (others => '0');
    signal x16_cov, x17_cov : unsigned(4 downto 0) := (others => '0');
    signal x16_dm : unsigned(16 downto 0) := (others => '0');
    signal x17_lv : unsigned(5 downto 0) := (others => '0');
    signal x18_prod : unsigned(10 downto 0) := (others => '0');
    signal x19_p14 : unsigned(9 downto 0) := (others => '0');
    signal x20_y : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl  : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_v1, s_v2, s_v2d, s_v3, s_v4, s_ld, s_fl, s_seen, s_efl, s_ffl : std_logic := '0';
    type t_gt is array (0 to 255) of unsigned(15 downto 0);
    signal gt_ram : t_gt := (others => (others => '0'));
    attribute no_rw_check of gt_ram : signal is true;
    signal gt_q, s_tv, s_tvd, s_sfirst, gt_wd : unsigned(15 downto 0) := (others => '0');
    signal gt_wa, gt_i : unsigned(7 downto 0) := (others => '0');
    signal gt_we, gt_run : std_logic := '0';
    signal gt_A  : unsigned(29 downto 0) := (others => '0');
    signal gt_D  : unsigned(23 downto 0) := (others => '0');
    signal s_col, s_c2, s_bcol, s_ecol, s_fcol : unsigned(10 downto 0) := (others => '0');
    signal bx_ram : t_gt := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q, bx_f : unsigned(15 downto 0) := (others => '0');
    signal s_bcnt : unsigned(6 downto 0) := (others => '0');
    signal s_b64 : std_logic := '0';                                  -- 64-px box on the 1920-wide rasters
    signal s_df   : signed(16 downto 0) := (others => '0');
    signal s_bsum : unsigned(21 downto 0) := (others => '0');
    signal s_ev, s_fv : std_logic := '0';
    signal s_eb, s_eq, s_fq : unsigned(15 downto 0) := (others => '0');
    signal s_dd : signed(16 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_q2, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal lb_rc : unsigned(10 downto 0) := (others => '0');
    signal x_q0, x_q1 : unsigned(15 downto 0) := (others => '0');
    signal x6_cs : unsigned(16 downto 0) := (others => '0');

    -- [G] output conditioning
    signal s_y23, s_yd1, s_yd2, s_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ya : unsigned(11 downto 0) := (others => '0');
    signal s_y26, s_y27, s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_KG_EXP(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            sn_q   <= C_KG_SIN(to_integer(sn_a));
            sn_f   <= sn_q;
            r_sh_q <= C_KG_SHD(to_integer(r_sh_a));
            r_shf  <= r_sh_q;
        end if;
    end process p_rom;

    -- gain table: written in the zone, read by the source path
    p_gtram : process(clk)
    begin
        if rising_edge(clk) then
            gt_q <= gt_ram(to_integer(s_sl));
            if gt_we = '1' then gt_ram(to_integer(gt_wa)) <= gt_wd; end if;
        end if;
    end process p_gtram;

    -- box delay: read 30 (or 62) samples behind the write (+ two registers = 32 or 64)
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_v2 = '1' then bx_ram(to_integer(bx_wp)) <= s_tv; end if;
        end if;
    end process p_bxram;

    -- source line store (half horizontal resolution): port 1 = the one-pole's own read (ahead of its
    -- write), port 2 = the pixel path (behind it)
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_c2(10 downto 1)));
            lb_q2 <= lb_ram(to_integer(lb_rc(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + [C] the field sequencer
    -- (one small operation per zone clock; the zone is quasi-static but timed at
    -- the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_d   : signed(10 downto 0);
        variable v_e   : unsigned(12 downto 0);
        variable v_s14 : signed(13 downto 0);
        variable v_r   : unsigned(15 downto 0);
        variable v_i   : unsigned(17 downto 0);
        variable v_t   : unsigned(25 downto 0);
        variable v_vk  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- [A] timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            s_ilr <= f_il(s_tid);

            -- [A] vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

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
                s_dbl   <= registers_in(6)(1);
                s_shade <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_ff    <= registers_in(6)(3) and not registers_in(6)(2);
                s_xs    <= s_xsr;
                s_il    <= s_ilr;
                if s_xsr = 2 then s_b64 <= '1'; else s_b64 <= '0'; end if;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); z_sc <= to_unsigned(16384, 16);
                    when 3      => s_oxs <= to_unsigned(153, 9); z_sc <= to_unsigned(21845, 16);
                    when others => s_oxs <= to_unsigned(102, 9); z_sc <= to_unsigned(32768, 16);
                end case;
                z_scy <= f_scy(s_tid);
            end if;
            g_wm1 <= s_w - 1;
            b_mm1 <= s_w + 7;
            z_m   <= s_w + 8;
            z_w48 <= (s_w & "00") + 8;
            s_oys <= f_oys(s_tid);

            -- [B] shared hold + glide, five clocks per channel.  Hold: the knob moves the held value only by
            -- 2 counts or more (or to either end).  Glide On: 1/16 of the gap per field (the Drift speed 1/8),
            -- never slower than one step (1/8 count), landing exactly.  Glide Off: the held value, this field.
            q_go <= '0';
            if s_zgo = '1' then
                z_gp <= "000"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case to_integer(z_gp) is
                    when 0 =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= g_x; s_gh <= h_x; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= g_y; s_gh <= h_y; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= g_p; s_gh <= h_p; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= g_f; s_gh <= h_f; s_gr <= unsigned(registers_in(3));
                            when 4      => s_gm <= g_c; s_gh <= h_c; s_gr <= unsigned(registers_in(5));
                            when 5      => s_gm <= g_w; s_gh <= h_w; s_gr <= unsigned(registers_in(7));
                            when others => s_gm <= g_v; s_gh <= h_v; s_gr <= unsigned(registers_in(4));
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
                            if z_gc = 6 then
                                if s_gd > 8 or s_gd < -8 then s_gd <= shift_right(s_gd, 3);
                                elsif s_gd > 0 then s_gd <= to_signed(1, 14);
                                elsif s_gd < 0 then s_gd <= to_signed(-1, 14);
                                end if;
                            else
                                if s_gd > 16 or s_gd < -16 then s_gd <= shift_right(s_gd, 4);
                                elsif s_gd > 0 then s_gd <= to_signed(1, 14);
                                elsif s_gd < 0 then s_gd <= to_signed(-1, 14);
                                end if;
                            end if;
                        end if;
                        z_gp <= "100";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => g_x <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_x <= s_gh;
                            when 1      => g_y <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_y <= s_gh;
                            when 2      => g_p <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_p <= s_gh;
                            when 3      => g_f <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_f <= s_gh;
                            when 4      => g_c <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_c <= s_gh;
                            when 5      => g_w <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_w <= s_gh;
                            when others => g_v <= unsigned(signed(s_gm) + s_gd(12 downto 0)); h_v <= s_gh;
                        end case;
                        z_gp <= "000";
                        if z_gc = 6 then z_grun <= '0'; q_go <= '1'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] the sequencer: slots of 24 clocks; q_fam = 0 holds the 10 global slots (q_j), q_fam = 1 the
            -- 8 slots (q_j) of family q_kf.  Phase 0 pre-muxes the operands, 1 loads the multiplier, 2 .. 17
            -- iterate, the product is read at 19.
            l_ld <= "00";
            if q_go = '1' then
                q_run <= '1'; q_fam <= '0'; q_kf <= '0';
                q_j <= (others => '0'); q_ph <= (others => '0');
            elsif q_run = '1' then
                if q_ph = 23 then
                    q_ph <= (others => '0');
                    if q_fam = '0' then
                        if q_j = 9 then q_fam <= '1'; q_j <= (others => '0'); else q_j <= q_j + 1; end if;
                    else
                        if q_j = 7 then
                            q_j <= (others => '0');
                            if q_kf = '1' then q_run <= '0'; end if;
                            q_kf <= '1';
                        else
                            q_j <= q_j + 1;
                        end if;
                    end if;
                else
                    q_ph <= q_ph + 1;
                end if;
            end if;

            -- operand pre-mux (phase 0)
            if q_run = '1' and q_ph = 0 then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                if q_fam = '0' then
                    case to_integer(q_j) is
                        when 1      => z_fa <= signed(resize(c_invp, 27)); z_fb <= resize(s_ys, 16);       -- cycles per line
                        when 2      =>                                                                     -- pivot x: 2 virtual px per count
                            v_s14 := signed(resize(g_x, 14)) - 4096;
                            z_fa <= resize(shift_left(resize(v_s14, 16), 1), 27); z_fb <= z_sc;
                        when 3      =>                                                                     -- pivot y, in lines
                            v_s14 := signed(resize(g_y, 14)) - 4096;
                            z_fa <= resize(shift_left(resize(v_s14, 16), 1) + 4320, 27); z_fb <= z_scy;
                        when 4      => z_fa <= to_signed(13107, 27); z_fb <= resize(g_w, 16);              -- half-width
                        when 5      =>
                            z_fa <= to_signed(19661, 27);
                            if g_w > 4092 then z_fb <= resize(g_w - 4092, 16); end if;
                        when others => null;
                    end case;
                else
                    case to_integer(q_j) is
                        when 1      => z_fa <= signed(resize(z_sdf, 27));  z_fb <= resize(z_fs, 16);       -- sine interpolation
                        when 2      => z_fa <= signed(resize(z_cdf, 27));  z_fb <= resize(z_fc, 16);       -- cosine
                        when 3      => z_fa <= signed(resize(c_iu, 27));   z_fb <= resize(z_cm, 16);       -- pixel step
                        when 4      => z_fa <= signed(resize(c_iv, 27));   z_fb <= resize(z_sm, 16);       -- line step
                        when 5      => z_fa <= signed(resize(z_dxm, 27));  z_fb <= resize(z_m, 16);        -- steps per line
                        when 6      =>                                                                     -- seed: pixel step x |pivot column|
                            if q_kf = '1' then z_fa <= resize(signed(d_inc(1)), 27); else z_fa <= resize(signed(d_inc(0)), 27); end if;
                            z_fb <= z_apx;
                        when 7      => z_fa <= resize(signed(z_dy), 27);   z_fb <= z_apy;                 -- line step x |pivot row|
                        when others => null;
                    end case;
                end if;
            end if;
            if q_ph = 1 then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 42); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(40 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            -- results and the small steps between the products
            if q_run = '1' and q_fam = '0' then
                case to_integer(q_j) is
                    when 0 =>                      -- the pitch: octaves, 1 / Period, pixels per cycle
                        case to_integer(q_ph) is
                            when 1 => z_oct <= resize(g_p(12 downto 4), 10) + resize(g_p(12 downto 5), 10);
                            when 2 =>
                                v_e := to_unsigned(4376, 13) - resize(z_oct, 13);
                                r_ex_a <= v_e(7 downto 0);
                                z_I <= v_e(12 downto 8);
                                case to_integer(s_xs) is
                                    when 4      => z_Lp <= to_unsigned(1256, 12) + resize(z_oct, 12);
                                    when 3      => z_Lp <= to_unsigned(1362, 12) + resize(z_oct, 12);
                                    when others => z_Lp <= to_unsigned(1512, 12) + resize(z_oct, 12);
                                end case;
                            when 5 =>
                                v_i := resize(r_exf, 18);
                                case to_integer(z_I) is
                                    when 14     => c_invp <= shift_right(v_i, 1);
                                    when 15     => c_invp <= v_i;
                                    when 16     => c_invp <= shift_left(v_i, 1);
                                    when others => c_invp <= shift_left(v_i, 2);
                                end case;
                                r_ex_a <= z_Lp(7 downto 0);
                            when 8 =>
                                c_m4 <= r_exf(15 downto 12);
                                if s_il = '1' then
                                    v_e := to_unsigned(14, 13) - resize(z_Lp(11 downto 8), 13);
                                else
                                    v_e := to_unsigned(13, 13) - resize(z_Lp(11 downto 8), 13);
                                end if;
                                c_shaa <= v_e(3 downto 0);
                            when others => null;
                        end case;
                    when 1 =>
                        if q_ph = 19 then
                            c_iv <= unsigned(f_acc(28 downto 5));
                            case to_integer(s_xs) is
                                when 4      => c_iu <= shift_left(resize(c_invp, 22), 4);
                                when 3      => c_iu <= shift_left(resize(c_invp, 22), 3) + shift_left(resize(c_invp, 22), 2);
                                when others => c_iu <= shift_left(resize(c_invp, 22), 3);
                            end case;
                        end if;
                    when 2 =>                      -- the pivot's DDA column (Q3 pixels)
                        if q_ph = 19 then
                            z_pv <= signed(resize(z_w48, 17)) + f_acc(31 downto 15);
                        elsif q_ph = 20 then
                            z_pxn <= z_pv(16);
                            if z_pv(16) = '1' then z_apx <= unsigned(-z_pv(15 downto 0)); else z_apx <= unsigned(z_pv(15 downto 0)); end if;
                        end if;
                    when 3 =>                      -- the pivot's row (Q3 lines)
                        if q_ph = 19 then
                            z_pv <= f_acc(31 downto 15);
                        elsif q_ph = 20 then
                            z_pyn <= z_pv(16);
                            if z_pv(16) = '1' then z_apy <= unsigned(-z_pv(15 downto 0)); else z_apy <= unsigned(z_pv(15 downto 0)); end if;
                        end if;
                    when 4 =>                      -- half-width hw0 = 410 + gW * 0.4 + max(0, gW - 4092) * 0.6 (Q14)
                        if q_ph = 19 then c_hw0 <= unsigned(f_acc(27 downto 15));
                        elsif q_ph = 20 then c_hw0 <= c_hw0 + 410; end if;
                    when 5 =>
                        if q_ph = 19 then c_hw0 <= c_hw0 + unsigned(f_acc(27 downto 15)); end if;
                    when 6 =>                      -- half-width >= 3 virtual px; the edge line: 3 .. 8 virtual px, at least
                        case to_integer(q_ph) is    -- 1/40 cycle, >= 2 px, and last of all <= 0.39 hw0 (so hw0 - L > 0)
                            when 1 => z_iv1 <= c_invp(17 downto 9);
                            when 2 => c_L <= shift_left(resize(z_iv1, 12), 1) + resize(z_iv1, 12);
                            when 3 =>
                                if c_hw0 < resize(c_L, 13) then c_hw0 <= resize(c_L, 13); end if;
                                if c_L < 410 then c_L <= to_unsigned(410, 12); end if;
                            when 4 => if c_L > shift_left(resize(z_iv1, 12), 3) then c_L <= shift_left(resize(z_iv1, 12), 3); end if;
                            when 5 => c_Lh <= resize(shift_right(c_hw0, 2), 12) + resize(shift_right(c_hw0, 3), 12);
                            when 6 => c_Lh <= c_Lh + resize(shift_right(c_hw0, 6), 12);
                            when 7 => if c_L < shift_left(resize(z_iv1, 12), 1) then c_L <= shift_left(resize(z_iv1, 12), 1); end if;
                            when 8 => if c_L > c_Lh then c_L <= c_Lh; end if;
                            when 9 =>
                                c_Lh <= '0' & c_L(11 downto 1);
                                c_hw0L <= c_hw0 - resize(c_L, 13);
                            when others => null;
                        end case;
                    when 7 =>                      -- the cane ramp gain (a table of 2^20 / hw0 by Weave)
                        if q_ph = 1 then r_sh_a <= g_w(12 downto 7);
                        elsif q_ph = 4 then
                            c_mq <= r_shf(3 downto 0);
                            v_e := to_unsigned(14, 13) - resize(r_shf(7 downto 4), 13);
                            if s_dbl = '1' then v_e := v_e - 1; end if;
                            c_shs <= v_e(3 downto 0);
                        end if;
                    when 8 =>                      -- Drift: |v - 512| (deadband 24), log speed, the two accumulators
                        case to_integer(q_ph) is
                            when 1 =>
                                v_vk := g_v(12 downto 3);
                                z_vn <= not v_vk(9);
                                if v_vk(9) = '1' then z_u <= '0' & v_vk(8 downto 0);
                                else z_u <= '0' & (not v_vk(8 downto 0)); end if;
                            when 2 =>
                                if z_u < 24 then z_vz <= '1'; else z_vz <= '0'; end if;
                                z_u <= z_u - 24;
                            when 3 => z_ls <= z_u + shift_right(z_u, 4);
                            when 4 => r_ex_a <= z_ls(5 downto 0) & "00";
                            when 7 =>
                                if z_vz = '1' then z_step <= (others => '0');
                                else z_step <= resize(r_exf(15 downto 7), 17); end if;
                            when 8 | 9 | 10 | 11 | 12 | 13 | 14 | 15 =>
                                if z_ls(9 downto 6) > resize(q_ph(2 downto 0), 4) then z_step <= shift_left(z_step, 1); end if;
                            when 16 =>
                                c_th <= f_addsub(c_th, resize(z_step, 24), z_vn);
                                c_de <= f_addsub(c_de, shift_left(resize(z_step, 26), 5), z_vn);
                            when others => null;
                        end case;
                    when 9 =>                      -- o_0 + o_1 + o_2 = 3/2 + Shift / 2 (Q14)
                        if q_ph = 1 then c_S <= resize(g_f, 14) + resize(g_f(12 downto 10), 14);
                        elsif q_ph = 2 then c_S <= '1' & c_S(12 downto 0); end if;
                    when others => null;
                end case;
            elsif q_run = '1' then
                case to_integer(q_j) is
                    when 0 =>                      -- the family's angle, then sine and cosine from the quarter-wave ROM
                        case to_integer(q_ph) is
                            when 1 =>
                                if q_kf = '1' then z_th <= c_th(23 downto 8) + to_unsigned(24576, 16);
                                else z_th <= c_th(23 downto 8) + to_unsigned(2731, 16); end if;
                            when 6 =>              -- sine: quadrant fold
                                z_sneg <= z_th(15);
                                if z_th(14) = '1' then z_p <= to_unsigned(16384, 15) - resize(z_th(13 downto 0), 15);
                                else z_p <= resize(z_th(13 downto 0), 15); end if;
                            when 7 =>
                                sn_a <= z_p(13 downto 4); z_ov <= z_p(14); z_fs <= z_p(3 downto 0);
                                if z_p(14) = '1' or z_p(13 downto 4) = 1023 then z_ov1 <= '1'; else z_ov1 <= '0'; end if;
                            when 8 =>
                                sn_a <= sn_a + 1;
                            when 10 =>
                                if z_ov = '1' then z_sm <= to_unsigned(16384, 15); else z_sm <= sn_f(14 downto 0); end if;
                            when 11 =>
                                if z_ov = '1' then z_sdf <= (others => '0');
                                elsif z_ov1 = '1' then z_sdf <= resize(to_unsigned(16384, 15) - z_sm, 6);
                                else z_sdf <= resize(sn_f(14 downto 0) - z_sm, 6); end if;
                            when 12 =>             -- cosine = sine a quarter turn on
                                v_r := z_th + 16384;
                                z_cneg <= v_r(15);
                                if v_r(14) = '1' then z_p <= to_unsigned(16384, 15) - resize(v_r(13 downto 0), 15);
                                else z_p <= resize(v_r(13 downto 0), 15); end if;
                            when 13 =>
                                sn_a <= z_p(13 downto 4); z_ov <= z_p(14); z_fc <= z_p(3 downto 0);
                                if z_p(14) = '1' or z_p(13 downto 4) = 1023 then z_ov1 <= '1'; else z_ov1 <= '0'; end if;
                            when 14 =>
                                sn_a <= sn_a + 1;
                            when 16 =>
                                if z_ov = '1' then z_cm <= to_unsigned(16384, 15); else z_cm <= sn_f(14 downto 0); end if;
                            when 17 =>
                                if z_ov = '1' then z_cdf <= (others => '0');
                                elsif z_ov1 = '1' then z_cdf <= resize(to_unsigned(16384, 15) - z_cm, 6);
                                else z_cdf <= resize(sn_f(14 downto 0) - z_cm, 6); end if;
                            when others => null;
                        end case;
                    when 1 =>
                        if q_ph = 19 then z_sm <= z_sm + resize(unsigned(f_acc(10 downto 4)) + ("000000" & f_acc(3)), 15); end if;
                    when 2 =>
                        if q_ph = 19 then z_cm <= z_cm + resize(unsigned(f_acc(10 downto 4)) + ("000000" & f_acc(3)), 15); end if;
                    when 3 =>
                        if q_ph = 19 then z_dxm <= unsigned(f_acc(35 downto 14)); end if;
                    when 4 =>
                        if q_ph = 19 then
                            z_dy <= f_addsub(to_unsigned(0, 26), unsigned(f_acc(39 downto 14)), z_sneg);
                        end if;
                    when 5 =>
                        if q_ph = 19 then          -- the pixel step +-dxm; the wrap = line step - steps * pixel step
                            if q_kf = '1' then
                                d_inc(1)  <= f_addsub(to_unsigned(0, 26), resize(z_dxm, 26), z_cneg);
                                d_wrap(1) <= f_addsub(z_dy, unsigned(f_acc(25 downto 0)), not z_cneg);
                            else
                                d_inc(0)  <= f_addsub(to_unsigned(0, 26), resize(z_dxm, 26), z_cneg);
                                d_wrap(0) <= f_addsub(z_dy, unsigned(f_acc(25 downto 0)), not z_cneg);
                            end if;
                        end if;
                    when 6 =>                      -- the phase at pixel 0, line 0 = o_k - (pivot column x pixel step
                        if q_ph = 19 then          --   + pivot row x line step)
                            z_seed <= f_addsub(to_unsigned(0, 26), unsigned(f_acc(28 downto 3)), not z_pxn);
                        end if;
                    when others =>
                        if q_ph = 19 then
                            z_seed <= f_addsub(z_seed, unsigned(f_acc(28 downto 3)), not z_pyn);
                        elsif q_ph = 20 then       -- + o_k: 1/2, and family 1 carries the glide along strip 0
                            if q_kf = '1' then v_t := c_de + to_unsigned(33554432, 26);
                            else v_t := to_unsigned(33554432, 26); end if;
                            z_seed <= z_seed + v_t;
                        elsif q_ph = 21 then
                            if q_kf = '1' then l_ld <= "10"; else l_ld <= "01"; end if;
                        end if;
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
            s_st <= g_start & s_st(0 to 8);
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
    -- [E] The two family DDAs (cycles, Q26): each adds its pixel step on every
    -- clock of the line window (W + 8 steps), then the line wrap once (wrap =
    -- line step - steps * pixel step).  The sequencer loads each field seed
    -- (the phase at DDA pixel 0 of line 0) in the zone.
    ---------------------------------------------------------------------------
    p_dda : process(clk)
        variable v_inc : unsigned(25 downto 0);
    begin
        if rising_edge(clk) then
            b_end <= '0';
            if s_st(9) = '1' then
                b_run <= '1'; b_cnt <= (others => '0');
            elsif b_run = '1' then
                b_cnt <= b_cnt + 1;
                if b_cnt = b_mm1 then b_run <= '0'; b_end <= '1'; end if;
            end if;
            for k in 0 to 1 loop
                if b_run = '1' then v_inc := d_inc(k); else v_inc := d_wrap(k); end if;
                if l_ld(k) = '1' then
                    d_acc(k) <= z_seed;
                elsif b_run = '1' or b_end = '1' then
                    d_acc(k) <= d_acc(k) + v_inc;
                end if;
            end loop;
        end if;
    end process p_dda;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> gain table (K6: Q16 cycles
    -- of half-width, a square law above a knee) -> box of 32 px (64 px on the
    -- 1920-wide rasters), edge-replicated and reloaded at every line start ->
    -- vertical one-pole (1/32 per line; 1/64 at 1080p, 1/16 at 480i / 576i) in
    -- the line store at half horizontal resolution.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(15 downto 0);
        variable v_dl : signed(16 downto 0);
        variable v_n : signed(17 downto 0);
        variable v_g : unsigned(29 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then   -- both phases restart from this field's drift
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);   -- ot * 96
            end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid;
            -- gain table (p_gtram), then its fabric copy: s_tv is the sample of column s_col
            s_v1 <= s_sv; s_v2 <= s_v1; s_v2d <= s_v2;
            s_tv <= gt_q;
            if s_v2 = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- box: sum of the last 32 (64) samples; the samples before the line start count as the first one
            if s_v2 = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            elsif s_b64 = '1' then bx_wp <= (others => '0'); bx_rp <= to_unsigned(194, 8);
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(226, 8); end if;
            bx_f <= bx_q;
            if s_bcnt(6) = '0' and (s_b64 = '1' or s_bcnt(5) = '0') then v_tap := s_sfirst; else v_tap := bx_f; end if;
            s_df <= signed(resize(s_tv, 17)) - signed(resize(v_tap, 17));
            s_tvd <= s_tv; s_v3 <= s_v2; s_c2 <= s_col;
            s_ld <= s_v2 and not s_v2d;
            if s_v2 = '1' and s_v2d = '0' then
                s_sfirst <= s_tv;
                s_bcnt <= to_unsigned(1, 7);
            elsif s_v2 = '1' then
                if s_bcnt(6) = '0' then s_bcnt <= s_bcnt + 1; end if;
            end if;
            if s_ld = '1' then
                if s_b64 = '1' then s_bsum <= shift_left(resize(s_tvd, 22), 6);
                else s_bsum <= shift_left(resize(s_tvd, 22), 5); end if;
            elsif s_v3 = '1' then
                s_bsum <= unsigned(signed(s_bsum) + resize(s_df, 22));
            end if;
            s_v4 <= s_v3; s_bcol <= s_c2;

            -- the first line of a field loads the one-pole instead of stepping it
            if s_zone = '1' then
                s_fl <= '1'; s_seen <= '0';
            elsif s_ld = '1' then
                if s_seen = '1' then s_fl <= '0'; end if;
                s_seen <= '1';
            end if;

            -- vertical one-pole on the odd columns: q += (box - q) >> 5 (6 at 1080p, 4 at 480i / 576i)
            if s_b64 = '1' then s_eb <= s_bsum(21 downto 6); else s_eb <= s_bsum(20 downto 5); end if;
            s_eq <= lb_q1;
            s_ev <= s_v4 and s_bcol(0); s_ecol <= s_bcol; s_efl <= s_fl;
            s_dd <= signed(resize(s_eb, 17)) - signed(resize(s_eq, 17));
            if s_efl = '1' then s_fq <= s_eb; else s_fq <= s_eq; end if;
            s_fv <= s_ev; s_fcol <= s_ecol; s_ffl <= s_efl;
            if s_ffl = '1' then v_dl := (others => '0');
            elsif s_b64 = '1' and s_il = '0' then v_dl := shift_right(s_dd, 6);
            elsif s_b64 = s_il then v_dl := shift_right(s_dd, 5);
            else v_dl := shift_right(s_dd, 4); end if;
            v_n := signed(resize(s_fq, 18)) + resize(v_dl, 18);
            lb_wd <= unsigned(v_n(15 downto 0));
            lb_we <= s_fv; lb_wa <= s_fcol(10 downto 1);

            -- gain table fill (zone, after the glides): T(i) = (max(0, i - 72)^2 * K6) >> 13 (Osc: >> 12), by
            -- second differences, saturated at 40960 (so hw0 + the camera stays under one cycle: <= 16378 in Q14)
            gt_we <= '0';
            if q_go = '1' then
                gt_run <= '1'; gt_i <= (others => '0'); gt_A <= (others => '0'); gt_D <= resize(g_c, 24);
            elsif gt_run = '1' then
                gt_we <= '1'; gt_wa <= gt_i;
                if s_src = '1' then v_g := shift_right(gt_A, 12); else v_g := shift_right(gt_A, 13); end if;
                if v_g > 40960 then gt_wd <= to_unsigned(40960, 16); else gt_wd <= v_g(15 downto 0); end if;
                if gt_i >= 72 then
                    gt_A <= gt_A + resize(gt_D, 30);
                    gt_D <= gt_D + shift_left(resize(g_c, 24), 1);
                end if;
                gt_i <= gt_i + 1;
                if gt_i = 255 then gt_run <= '0'; end if;
            end if;

            -- pixel path: the store's column counter, two fabric copies of the read, and at stage 6 the sum
            -- of two neighbouring samples (the camera half-width x 2)
            if b_run = '1' then lb_rc <= lb_rc + 1; else lb_rc <= (others => '0'); end if;
            x_q0 <= lb_q2; x_q1 <= x_q0;
            x6_cs <= resize(x_q0, 17) + resize(x_q1, 17);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] WEAVE (stages 6 .. 13) and [F] COVERAGE + SHADE (stages 14 .. 21)
    ---------------------------------------------------------------------------
    p_weave : process(clk)
        variable v_z : signed(15 downto 0);
        variable v_c : signed(5 downto 0);
        variable v_m : unsigned(16 downto 0);
        variable v_l : unsigned(5 downto 0);
        variable v_p6 : unsigned(5 downto 0);
        variable v_cam : unsigned(13 downto 0);
        variable km, kp : integer;
    begin
        if rising_edge(clk) then
            -- 6: the family phases (Q14); the third is the sum's complement
            x6_c(0) <= d_acc(0)(25 downto 12);
            x6_c(1) <= d_acc(1)(25 downto 12);
            x6_c(2) <= c_S - d_acc(0)(25 downto 12) - d_acc(1)(25 downto 12);
            -- 7: a_k = distance to the nearest centre line (one's complement fold), a_k + L; the half-width with
            -- the camera, and less the edge line
            for k in 0 to 2 loop
                if x6_c(k)(13) = '1' then x7_a(k) <= not x6_c(k)(12 downto 0);
                    x7_aL(k) <= resize(not x6_c(k)(12 downto 0), 14) + resize(c_L, 14);
                else x7_a(k) <= x6_c(k)(12 downto 0);
                    x7_aL(k) <= resize(x6_c(k)(12 downto 0), 14) + resize(c_L, 14);
                end if;
            end loop;
            v_cam := x6_cs(16 downto 3);
            x7_hw  <= resize(c_hw0, 14) + v_cam;
            x7_hwL <= resize(c_hw0L, 14) + v_cam;
            -- 8: body = hw - L - a_k, the Double centre line a_k - L/2; u_k = max(a_(k+1), a_k + L)
            for k in 0 to 2 loop
                kp := (k + 1) mod 3;
                x8_b(k)  <= signed(resize(x7_hwL, 15)) - signed(resize(x7_a(k), 15));
                x8_bd(k) <= signed(resize(x7_a(k), 15)) - signed(resize(c_Lh, 15));
                if resize(x7_a(kp), 14) > x7_aL(k) then x8_u(k) <= resize(x7_a(kp), 14); else x8_u(k) <= x7_aL(k); end if;
            end loop;
            x8_a <= x7_a; x8_hw <= x7_hw;
            if x7_a(1) < x7_a(0) then x8_m01 <= x7_a(1); else x8_m01 <= x7_a(0); end if;
            -- 9: v_k = min(hw, u_k)
            for k in 0 to 2 loop
                if s_dbl = '1' then x9_body(k) <= f_min(x8_b(k), x8_bd(k)); else x9_body(k) <= x8_b(k); end if;
                if x8_u(k) < x8_hw then x9_v(k) <= x8_u(k); else x9_v(k) <= x8_hw; end if;
            end loop;
            x9_a <= x8_a; x9_hw <= x8_hw;
            if x8_a(2) < x8_m01 then x9_mina <= x8_a(2); else x9_mina <= x8_m01; end if;
            -- 10: visible = a_(k-1) - v_k: positive unless the strip above covers (and, in a triple overlap, the
            -- pinwheel rule)
            for k in 0 to 2 loop
                km := (k + 2) mod 3;
                x10_vis(k) <= signed(resize(x9_a(km), 15)) - signed(resize(x9_v(k), 15));
            end loop;
            x10_body <= x9_body;
            x10_nem <= signed(resize(x9_mina, 15)) - signed(resize(x9_hw, 15));
            -- 11
            for k in 0 to 2 loop x11_w(k) <= f_min(x10_body(k), x10_vis(k)); end loop;
            x11_nem <= x10_nem;
            -- 12, 13: the maximum over the three strips (Fine folds in min_k a_k - hw)
            x12_w01 <= f_max(x11_w(0), x11_w(1));
            if s_ff = '1' then x12_w2 <= f_max(x11_w(2), x11_nem); else x12_w2 <= x11_w(2); end if;
            x13_W <= f_max(x12_w01, x12_w2);
            -- 14: clamp for the pixel scale; W + L for the cane ramp
            if x13_W < -2048 then x14_Zc <= to_signed(-2048, 12);
            elsif x13_W > 2047 then x14_Zc <= to_signed(2047, 12);
            else x14_Zc <= x13_W(11 downto 0); end if;
            x14_Ws <= resize(x13_W, 16) + signed(resize(c_L, 16));
            -- 15: x pixels per cycle (mantissa)
            x15_P <= resize(x14_Zc * signed('0' & c_m4), 16);
            if x14_Ws(15) = '1' then x15_Wc <= (others => '0');
            elsif x14_Ws(14 downto 13) /= "00" then x15_Wc <= (others => '1');
            else x15_Wc <= unsigned(x14_Ws(12 downto 0)); end if;
            -- 16: coverage 0 .. 16 (1/16 px per step, 1/8 interlaced); the ramp product
            case to_integer(c_shaa) is
                when 5      => v_z := shift_right(x15_P, 5);
                when 6      => v_z := shift_right(x15_P, 6);
                when 7      => v_z := shift_right(x15_P, 7);
                when 8      => v_z := shift_right(x15_P, 8);
                when 9      => v_z := shift_right(x15_P, 9);
                when others => v_z := shift_right(x15_P, 10);
            end case;
            if v_z < -8 then v_c := to_signed(0, 6);
            elsif v_z > 8 then v_c := to_signed(16, 6);
            else v_c := v_z(5 downto 0) + 8; end if;
            if s_ff = '1' then v_c := to_signed(16, 6) - v_c; end if;
            x16_cov <= unsigned(v_c(4 downto 0));
            x16_dm <= resize(x15_Wc * c_mq, 17);
            -- 17: the luma level: Flat 63; Shade + Bold 4 steps; Shade + Fine a smooth ramp
            case to_integer(c_shs) is
                when 5      => v_m := shift_right(x16_dm, 5);
                when 6      => v_m := shift_right(x16_dm, 6);
                when 7      => v_m := shift_right(x16_dm, 7);
                when 8      => v_m := shift_right(x16_dm, 8);
                when 9      => v_m := shift_right(x16_dm, 9);
                when others => v_m := shift_right(x16_dm, 10);
            end case;
            if v_m(16 downto 6) /= 0 then v_l := (others => '1'); else v_l := v_m(5 downto 0); end if;
            if s_shade = '0' then x17_lv <= to_unsigned(63, 6);
            elsif s_fine = '0' then
                case to_integer(v_l(5 downto 4)) is
                    when 0      => x17_lv <= to_unsigned(0, 6);
                    when 1      => x17_lv <= to_unsigned(21, 6);
                    when 2      => x17_lv <= to_unsigned(42, 6);
                    when others => x17_lv <= to_unsigned(63, 6);
                end case;
            else x17_lv <= v_l; end if;
            x17_cov <= x16_cov;
            -- 18 .. 21: luma = 64 + ((cov * level) >> 4) * 14
            x18_prod <= resize(x17_cov * x17_lv, 11);
            v_p6 := x18_prod(9 downto 4);
            x19_p14 <= shift_left(resize(v_p6, 10), 4) - shift_left(resize(v_p6, 10), 1);
            x20_y <= x19_p14 + 64;
            s_y23 <= x20_y;
        end if;
    end process p_weave;

    ---------------------------------------------------------------------------
    -- [G] OUTPUT CONDITIONING: luma [1 2 1]/4, luma 13/16 toward black, blanking
    -- gate.  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_yd1 <= s_y23; s_yd2 <= s_yd1;
            s_ya <= resize(s_y23, 12) + resize(s_yd2, 12) + 2;
            s_yb <= s_yd1;
            v_y := s_ya + shift_left(resize(s_yb, 12), 1);
            s_y26 <= v_y(11 downto 2);
            if s_y26 > 64 then v_dl := s_y26 - 64; else v_dl := (others => '0'); end if;
            s_y27 <= s_y26 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y28 <= s_y27;
            else
                s_y28 <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture kagome;
