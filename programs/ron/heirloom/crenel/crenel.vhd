-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Crenel v0.5.0 -- Heirloom #10, rebuilt from the photo WIN_20230223_01_46_41.
--
-- The photo: one source picture tiled ~2x across the line by a horizontal oscillator whose
-- phase resets at every line start and whose period is bent down the frame (1064 px at the top,
-- 962 at line 205, slipping to other values in horizontal bands).  Down the frame, in bands:
-- a row of hot-pink "+" crenels (3 per tile; a bar through vertical teeth, a hot-pink ridge
-- on every left edge with two feedback echoes, pink rims, a blue glow); a bright-blue camera
-- dome; a slipped row with the top of a pink V; the pink V's (tile phase 0.56); rows of blue
-- camera blocks, "(" arcs and staggered dashes; a navy-blue crenel bar carrying a soft pink
-- arch ring; pink blocks on teeth 1 and 3; a dash floor.  The blue areas are camera content
-- through a rippled colouriser (soft horizontal key x source -> navy / blue folds / green and
-- red fringes) and are torn sideways in runs of lines.
--
--   K1  Crenels  crosses per tile 1 .. 6 (3 = the photo); every crenel-locked row follows
--   K2  Tiles    tiles across the 1920 line, 1 .. 4 (2 = the photo)
--   K3  Echo     feedback echoes left of every tooth: none .. strong and wide (square law)
--   K4  Drift    tile phase drift, bipolar (still in the centre): the whole tiled picture slides
--   K5  Ripples  soft-key width of the camera blobs: hard flat blobs .. wide rippled flanks
--   K6  Palette  Photo / Ember / Jade / Rust
--   S7  Tear     sideways tearing of the camera rows (On)
--   S8  Outline  pink shapes reduced to their ridges and rims
--   S9  Mirror   every other tile mirrored
--   S10 Roll     the band structure rolls up the screen (V-hold lost)
--   S11 Source   Video / Osc
--   P12 BEND     oscillator FM depth: 0 = upright battlements, the photo at 40 %,
--                100 % = crenels leaning into long slashes, slipped rows torn apart; the
--                tear amplitude follows
--
-- Geometry: virtual 1080-line rows, tile phase with a fixed virtual tile width (round arch at
-- every resolution; SD 4:3 shows 1.5 tiles).  Generated avid (parity-locked start, field-latched
-- width); output conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma to ~777)
-- before the blanking gate.  Latency 24 clocks in every mode.  Colours in the hardware
-- convention (u = Cr, v = Cb), see crenel.py.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.crenel_pkg.all;
use work.all;

architecture crenel of program_top is
    constant C_LAT : integer := 24;   -- matches processing_delay_clks

    constant C_INCC : integer := 17119;          -- 2^24 / 980, the FM centre
    type t_by is array(0 to 12) of integer;
    constant C_BY : t_by := (0, 38, 276, 300, 412, 478, 574, 645, 700, 790, 852, 948, 1018);

    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
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
    function f_interl(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8 | 1 | 2 | 10 => return '1';
            when others => return '0';
        end case;
    end function;
    -- line-buffer write step, Q16 per source pixel: 1024 entries per line (HD, 720p), 512 at SD so
    -- that every entry is written (a step above 1 would leave stale entries)
    function f_wstep(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(46603, 17);
            when 5 | 6 | 14     => return to_unsigned(52429, 17);
            when others         => return to_unsigned(34953, 17);
        end case;
    end function;
    function f_tri8(p : unsigned(7 downto 0)) return unsigned is
    begin
        if p(7) = '0' then return p(6 downto 0) & '0'; else return (not p(6 downto 0)) & '1'; end if;
    end function;
    -- signed triangle of a 16-bit phase: 32767 at 0, -32767 at the half
    function f_tri16s(p : unsigned(15 downto 0)) return signed is
        variable a : unsigned(14 downto 0);
    begin
        if p(15) = '0' then a := p(14 downto 0); else a := not p(14 downto 0); end if;
        return to_signed(32767, 17) - signed(resize(a & '0', 17));
    end function;
    -- min(63, q >> s)
    function f_win6(q : unsigned(15 downto 0); s : unsigned(3 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := shift_right(q, to_integer(s));
        if v(15 downto 6) /= 0 then return to_unsigned(63, 6); else return v(5 downto 0); end if;
    end function;
    function f_xs(s : unsigned(31 downto 0)) return unsigned is
        variable v : unsigned(31 downto 0);
    begin
        v := s xor shift_left(s, 13);
        v := v xor shift_right(v, 17);
        v := v xor shift_left(v, 5);
        return v;
    end function;
    function f_mix(v0 : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := v0 xor shift_right(v0, 7);
        v := v xor shift_left(v, 3);
        v := v + shift_right(v, 5);
        return v;
    end function;

    -- timing
    signal t_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal t_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal t_il : std_logic := '0';
    signal t_ws : unsigned(16 downto 0) := to_unsigned(34953, 17);
    signal t_sd : std_logic := '0';

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- controls (glided K1 K2 K3 K5 P12, Q.3)
    type t_g is array(0 to 4) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(410 * 8, 13), to_unsigned(341 * 8, 13), to_unsigned(460 * 8, 13),
                         to_unsigned(512 * 8, 13), to_unsigned(410 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');

    -- Osc source per-line phase steps (345 and 164 per virtual line) by timing ID
    function f_oya(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1534, 12);
            when 4          => return to_unsigned(776, 12);
            when 8          => return to_unsigned(1294, 12);
            when 12         => return to_unsigned(647, 12);
            when 5 | 6 | 14 => return to_unsigned(518, 12);
            when 1 | 2 | 10 => return to_unsigned(690, 12);
            when others     => return to_unsigned(345, 12);
        end case;
    end function;
    function f_oyb(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(729, 12);
            when 4          => return to_unsigned(369, 12);
            when 8          => return to_unsigned(615, 12);
            when 12         => return to_unsigned(308, 12);
            when 5 | 6 | 14 => return to_unsigned(246, 12);
            when 1 | 2 | 10 => return to_unsigned(328, 12);
            when others     => return to_unsigned(164, 12);
        end case;
    end function;
    function f_bystart(b : unsigned(3 downto 0)) return unsigned is
    begin
        case to_integer(b) is
            when 1 => return to_unsigned(38, 11);   when 2 => return to_unsigned(276, 11);
            when 3 => return to_unsigned(300, 11);  when 4 => return to_unsigned(412, 11);
            when 5 => return to_unsigned(478, 11);  when 6 => return to_unsigned(574, 11);
            when 7 => return to_unsigned(645, 11);  when 8 => return to_unsigned(700, 11);
            when 9 => return to_unsigned(790, 11);  when 10 => return to_unsigned(852, 11);
            when 11 => return to_unsigned(948, 11); when 12 => return to_unsigned(1018, 11);
            when 13 => return to_unsigned(2047, 11);
            when others => return to_unsigned(0, 11);
        end case;
    end function;
    -- min(255, q >> s), s = 6 or 8
    function f_d8(q : unsigned(15 downto 0); t : std_logic) return unsigned is
    begin
        if t = '1' then
            if q(15 downto 14) /= 0 then return to_unsigned(255, 8); else return q(13 downto 6); end if;
        else
            return q(15 downto 8);
        end if;
    end function;

    signal t_oya, t_oyb : unsigned(11 downto 0) := (others => '0');

    -- per-field values
    signal f_k3    : unsigned(2 downto 0) := to_unsigned(3, 3);       -- crenels per tile 1 .. 6
    signal f_ca    : unsigned(1 downto 0) := to_unsigned(1, 2);       -- c = (t << ca) + (t << cb) [cben]
    signal f_cb, f_cben : std_logic := '1';
    signal f_tiles : unsigned(10 downto 0) := to_unsigned(511, 11);
    signal f_rip   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal f_bend  : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal f_bendl : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal f_eg    : unsigned(6 downto 0) := to_unsigned(16, 7);
    signal f_tk    : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal f_drift : signed(9 downto 0) := (others => '0');
    signal f_phi   : unsigned(23 downto 0) := (others => '0');
    signal f_roll  : unsigned(10 downto 0) := (others => '0');
    signal f_fld   : unsigned(15 downto 0) := (others => '0');
    signal f_ot    : unsigned(15 downto 0) := (others => '0');
    signal f_lfo   : unsigned(7 downto 0) := (others => '0');
    signal f_pal   : unsigned(1 downto 0) := (others => '0');
    signal f_tear, f_outl, f_mir, f_rollon, f_osc, f_odd : std_logic := '0';

    -- serial multiplier (shared by the field sequencer and the line engine)
    signal q_job : unsigned(4 downto 0) := (others => '0');
    signal q_ph  : unsigned(4 downto 0) := (others => '0');
    signal q_run : std_logic := '0';
    signal m_a : unsigned(17 downto 0) := (others => '0');
    signal m_b : unsigned(10 downto 0) := (others => '0');
    signal m_h : unsigned(17 downto 0) := (others => '0');
    signal m_l : unsigned(10 downto 0) := (others => '0');
    signal m_neg : std_logic := '0';

    -- line ROM
    signal r_la : unsigned(9 downto 0) := (others => '0');
    signal r_lq, r_lq2 : std_logic_vector(15 downto 0) := (others => '0');

    -- per-line engine
    signal e_vy   : unsigned(19 downto 0) := (others => '0');
    signal e_slot : unsigned(3 downto 0) := (others => '1');
    signal e_ph   : unsigned(4 downto 0) := (others => '0');
    signal e_yi, e_y, e_yp : unsigned(10 downto 0) := (others => '0');
    signal e_band : unsigned(3 downto 0) := (others => '0');
    signal e_bnext : unsigned(10 downto 0) := to_unsigned(38, 11);
    signal e_d    : unsigned(6 downto 0) := (others => '0');
    signal e_dy   : unsigned(7 downto 0) := (others => '0');
    signal e_inca, e_incb2, e_lr, e_gn, e_hw, e_koa, e_fil, e_ha, e_hb : unsigned(15 downto 0) := (others => '0');
    signal e_inck : unsigned(16 downto 0) := (others => '0');
    signal e_dmag : unsigned(11 downto 0) := (others => '0');
    signal e_dneg : std_logic := '0';
    signal e_incb : unsigned(16 downto 0) := (others => '0');
    signal e_incv : unsigned(17 downto 0) := (others => '0');
    signal e_dt, e_dc : unsigned(15 downto 0) := (others => '0');
    signal e_lin0, e_lin1 : unsigned(15 downto 0) := (others => '0');
    signal e_row : unsigned(6 downto 0) := (others => '0');
    signal e_dy2 : unsigned(15 downto 0) := (others => '0');
    signal e_done, e_zk : std_logic := '0';
    signal s_fc : unsigned(5 downto 0) := (others => '1');
    -- tears
    signal t_s : unsigned(31 downto 0) := x"9E3779B9";
    signal t_run : unsigned(5 downto 0) := (others => '0');
    signal t_mag : unsigned(4 downto 0) := (others => '0');
    signal t_sgn, t_hit : std_logic := '0';
    signal e_jit : unsigned(5 downto 0) := (others => '0');   -- |jit'| px
    signal e_jneg : std_logic := '0';

    -- the live line values (assembled in the horizontal blanking from the engine's results)
    type t_lv is record
        incr : unsigned(19 downto 0);
        dt, dc, koff4 : unsigned(15 downto 0);
        pon, psel, pbar, prim, primb, punder, pgate, echo, glow : std_logic;
        pa, pw : unsigned(15 downto 0);
        pamp : unsigned(4 downto 0);
        pcls : unsigned(1 downto 0);
        plo : unsigned(2 downto 0);
        nb : unsigned(1 downto 0);
        aon, agate : std_logic;
        asel : unsigned(1 downto 0);
        aa, aw : unsigned(15 downto 0);
        ashl, ashr : unsigned(3 downto 0);
        ag : unsigned(4 downto 0);
        mode : unsigned(1 downto 0);
        envon, envs, envc, wob, aron : std_logic;
        envk : unsigned(6 downto 0);
        ak : unsigned(16 downto 0);
        gidx : unsigned(5 downto 0);
    end record;
    constant C_LV0 : t_lv := (incr => (others => '0'), dt => (others => '0'), dc => (others => '0'),
        koff4 => (others => '0'),
        pon => '0', psel => '0', pbar => '0', prim => '0', primb => '0', punder => '0', pgate => '0',
        echo => '0', glow => '0', pa => (others => '0'), pw => (others => '0'),
        pamp => (others => '0'), pcls => (others => '0'), plo => (others => '0'),
        nb => (others => '0'), aon => '0', agate => '0',
        asel => (others => '0'), aa => (others => '0'), aw => (others => '0'),
        ashl => (others => '0'), ashr => (others => '0'), ag => (others => '0'),
        mode => (others => '0'), envon => '0', envs => '0', envc => '0', wob => '0', aron => '0',
        envk => (others => '0'), ak => (others => '0'), gidx => to_unsigned(9, 6));
    signal l_lv : t_lv := C_LV0;

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- source side: line buffer (2 banks x 1024 x 8) and the Osc source
    type t_lb is array(0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    signal lb_q : std_logic_vector(7 downto 0) := (others => '0');
    signal lb_ra, lb_wa : unsigned(10 downto 0) := (others => '0');
    signal lb_wd : std_logic_vector(7 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal s_wacc : unsigned(26 downto 0) := (others => '0');
    signal s_wbank, s_rbank : std_logic := '0';
    signal s_ya, s_aoff : unsigned(15 downto 0) := (others => '0');
    signal w1_a, w2_a, w3_a, w1_x : unsigned(9 downto 0) := (others => '0');
    signal w1_we, w2_we, w3_we, w1_b, w2_b, w3_b, w2_osc : std_logic := '0';
    signal w2_pa : unsigned(15 downto 0) := (others => '0');
    signal w1_l, w2_l, w3_d : unsigned(7 downto 0) := (others => '0');

    -- echo profile ROM
    signal r_eca : unsigned(9 downto 0) := (others => '0');
    signal r_ec : std_logic_vector(7 downto 0) := (others => '0');
    -- square ROM (arch radius)
    signal r_sqa : unsigned(7 downto 0) := (others => '0');
    signal r_sq : std_logic_vector(15 downto 0) := (others => '0');

    -- beam
    signal b_p : unsigned(24 downto 0) := (others => '0');

    -- pixel path
    signal p1_t : unsigned(15 downto 0) := (others => '0');
    signal p2_t, p2_tc : unsigned(15 downto 0) := (others => '0');
    signal p2_ct : unsigned(18 downto 0) := (others => '0');
    signal p3_t, p3_tc, p3_c, p3_cc, p3_kc : unsigned(15 downto 0) := (others => '0');
    signal p3_tooth : unsigned(2 downto 0) := (others => '0');
    signal p4_q, p4_qa : unsigned(15 downto 0) := (others => '0');
    signal p4_t10 : unsigned(9 downto 0) := (others => '0');
    signal p4_dx, p4_de : unsigned(9 downto 0) := (others => '0');
    signal p4_tooth, p5_tooth : unsigned(2 downto 0) := (others => '0');
    signal p5_wq, p5_wqa : unsigned(16 downto 0) := (others => '0');
    signal p5_dli : unsigned(5 downto 0) := (others => '0');
    signal p5_dn, p6_dn, p6_drn : unsigned(7 downto 0) := (others => '0');
    signal p5_gate, p5_cut, p6_cut, p7_cut, p8_cut, p9_cut, p10_cut : std_logic := '0';
    signal p5_src : unsigned(7 downto 0) := (others => '0');
    signal p5_px : unsigned(7 downto 0) := (others => '0');
    signal p5_ae : unsigned(9 downto 0) := (others => '0');
    signal p5_kl, p6_key, p6_rid, p6_env, p7_env : unsigned(5 downto 0) := (others => '0');
    signal p5_gq : std_logic := '0';
    signal p6_under, p6_rim, p6_inside, p6_win : std_logic := '0';
    signal p7_r : unsigned(10 downto 0) := (others => '0');
    signal p7_gi : unsigned(4 downto 0) := (others => '0');
    signal p7_kg : unsigned(10 downto 0) := (others => '0');
    signal p7_under, p7_rim, p7_inside : std_logic := '0';
    signal p8_lv, p8_glow, p8_key : unsigned(5 downto 0) := (others => '0');
    signal p8_ec : unsigned(7 downto 0) := (others => '0');
    signal p8_sq : unsigned(15 downto 0) := (others => '0');
    signal p8_inside, p8_full, p8_camm : std_logic := '0';
    signal p9_lv, p9_glev : unsigned(5 downto 0) := (others => '0');
    signal p9_ev : unsigned(14 downto 0) := (others => '0');
    signal p9_cam : unsigned(13 downto 0) := (others => '0');
    signal p9_full, p9_inside, p9_camm, p9_echoon : std_logic := '0';
    signal p9_d2m : signed(18 downto 0) := (others => '0');
    signal p10_lv, p10_echo, p10_glev, p10_lec : unsigned(5 downto 0) := (others => '0');
    signal p10_inside, p10_camm : std_logic := '0';
    signal p10_dr : signed(9 downto 0) := (others => '0');
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    type t_u9a is array (natural range <>) of unsigned(8 downto 0);
    signal p_sr : t_u8a(6 to 8) := (others => (others => '0'));
    signal p_code : t_u9a(11 to 14) := (others => (others => '0'));
    -- colour ROM
    signal r_ca : unsigned(10 downto 0) := (others => '0');
    signal r_cy, r_cu, r_cv : std_logic_vector(7 downto 0) := (others => '0');
    signal p17_y, p17_u, p17_v : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_ya1, s_yb, s_yc, s_yd, s_ye, s_yfx, s_yf : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_uf, s_vf : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_yg : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ug, s_vg : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs and the line buffer
    ---------------------------------------------------------------------------
    p_crom : process(clk)
    begin
        if rising_edge(clk) then
            r_cy <= C_CY(to_integer(r_ca)); r_cu <= C_CU(to_integer(r_ca)); r_cv <= C_CV(to_integer(r_ca));
        end if;
    end process p_crom;

    p_lrom : process(clk)
    begin
        if rising_edge(clk) then
            r_lq <= C_LR(to_integer(r_la));
            r_lq2 <= r_lq;
        end if;
    end process p_lrom;

    p_sqrom : process(clk)
    begin
        if rising_edge(clk) then
            r_sq <= C_SQ(to_integer(r_sqa));
            r_ec <= C_ECP(to_integer(r_eca));
        end if;
    end process p_sqrom;

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            lb_q <= lb(to_integer(lb_ra));
            if lb_we = '1' then lb(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lb;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field values, shared serial multiplier
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_x  : signed(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_s  : unsigned(18 downto 0);
        variable v_pr : unsigned(28 downto 0);
        variable v_t8 : unsigned(7 downto 0);
        variable v_k6 : unsigned(9 downto 0);
        variable v_rl : unsigned(11 downto 0);
        variable v_sg : signed(16 downto 0);
        variable v_k3 : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            t_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            t_ys <= f_vstep_y(registers_in(8)(3 downto 0));
            t_il <= f_interl(registers_in(8)(3 downto 0));
            t_ws <= f_wstep(registers_in(8)(3 downto 0));
            if f_vstep_x(registers_in(8)(3 downto 0)) = 4 then t_sd <= '1'; else t_sd <= '0'; end if;
            t_oya <= f_oya(registers_in(8)(3 downto 0));
            t_oyb <= f_oyb(registers_in(8)(3 downto 0));

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
            elsif s_zc /= 1023 then
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
            g_wm1 <= s_w - 1;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                -- K4 Drift (bipolar, deadband 8), K6 Palette (labelled: index 0..3 or a knob value)
                v_x := signed(resize(unsigned(registers_in(3)), 11)) - 512;
                if v_x > 7 then f_drift <= resize(v_x - 8, 10);
                elsif v_x < -7 then f_drift <= resize(v_x + 8, 10);
                else f_drift <= (others => '0'); end if;
                v_k6 := unsigned(registers_in(5));
                if v_k6 < 4 then f_pal <= v_k6(1 downto 0); else f_pal <= v_k6(9 downto 8); end if;
                f_tear   <= registers_in(6)(0);
                f_outl   <= registers_in(6)(1);
                f_mir    <= registers_in(6)(2);
                f_rollon <= registers_in(6)(3);
                f_osc    <= registers_in(6)(4);
                -- per-field phases
                f_phi <= f_phi + unsigned(resize(f_drift * to_signed(96, 8), 24));
                if f_rollon = '1' then
                    v_rl := resize(f_roll, 12) + 2;
                    if v_rl >= 1080 then f_roll <= resize(v_rl - 1080, 11); else f_roll <= v_rl(10 downto 0); end if;
                end if;
                f_fld <= f_fld + 1;
                f_ot <= f_ot + 5;
                f_odd <= not f_odd;
            end if;

            -- glides: K1, K2, K3, K5, P12 ease 1/8 of the gap per field (deadband 4); one channel per
            -- four zone clocks (4..23): select, difference, ease, write
            if s_zone = '1' and s_zc >= 4 and s_zc < 24 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(2)) & "000"; gl_o <= s_g(2);
                            when 3 => gl_k <= unsigned(registers_in(4)) & "000"; gl_o <= s_g(3);
                            when others => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(4);
                        end case;
                    when 1 =>
                        gl_d <= signed(resize(gl_k, 14)) - signed(resize(gl_o, 14));
                    when 2 =>
                        v_ng := signed(resize(gl_o, 14)) + shift_right(gl_d, 3);
                        gl_n <= unsigned(v_ng(12 downto 0));
                        if gl_d > 31 or gl_d < -31 then gl_w <= '1'; else gl_w <= '0'; end if;
                    when others =>
                        if gl_w = '1' then
                            case to_integer(q_gi) is
                                when 0 => s_g(0) <= gl_n;
                                when 1 => s_g(1) <= gl_n;
                                when 2 => s_g(2) <= gl_n;
                                when 3 => s_g(3) <= gl_n;
                                when others => s_g(4) <= gl_n;
                            end case;
                        end if;
                end case;
            end if;

            -- per-field decode (zone clock 32), from the glided values
            if s_zone = '1' and s_zc = 32 then
                v_k3 := resize(shift_right(resize(s_g(0)(12 downto 3), 13) * 6, 10), 3);   -- 0 .. 5
                f_k3 <= v_k3 + 1;
                case to_integer(v_k3) is
                    when 0 => f_ca <= "00"; f_cben <= '0'; f_cb <= '0';
                    when 1 => f_ca <= "01"; f_cben <= '0'; f_cb <= '0';
                    when 2 => f_ca <= "01"; f_cben <= '1'; f_cb <= '0';
                    when 3 => f_ca <= "10"; f_cben <= '0'; f_cb <= '0';
                    when 4 => f_ca <= "10"; f_cben <= '1'; f_cb <= '0';
                    when others => f_ca <= "10"; f_cben <= '1'; f_cb <= '1';
                end case;
                f_tiles <= to_unsigned(256, 11) + resize(shift_right(resize(s_g(1)(12 downto 3), 12) * 3, 2), 11);
                f_eg    <= unsigned(C_EGL(to_integer(s_g(2)(12 downto 8))));
                f_rip   <= resize(shift_right(resize(s_g(3)(12 downto 3), 13) * 5, 10), 3);
                f_bend  <= resize(shift_right(resize(s_g(4)(12 downto 3), 13) * 5, 3), 10);
                v_t8 := f_fld(7 downto 0) + (f_fld(6 downto 0) & '0');
                f_lfo <= f_tri8(v_t8);
            end if;
            if s_zone = '1' and s_zc = 224 then
                f_tk <= to_unsigned(4, 6) + resize(shift_right(resize(f_bendl, 13) * 3, 6), 6);
            end if;

            -- shared serial multiplier: job q_job, phases 0 load, 1..11 step, 13 store
            if q_run = '1' then
                if q_ph = 15 then q_run <= '0'; end if;
                q_ph <= q_ph + 1;
            end if;
            if s_zone = '1' and s_zc = 64 then
                q_run <= '1'; q_ph <= (others => '0'); q_job <= to_unsigned(3, 5);
            elsif e_ph = 0 and e_slot >= 1 and e_slot <= 11 and e_slot /= 8 and e_slot /= 9 and e_slot /= 10 then
                q_run <= '1'; q_ph <= (others => '0');
                q_job <= resize(e_slot, 5) + 7;                        -- 8 .. 18
            end if;
            if q_run = '1' and q_ph = 0 then
                m_h <= (others => '0'); m_l <= (others => '0');
                m_neg <= '0';
                case to_integer(q_job) is
                    when 3  => m_a <= resize(f_bend, 18);
                               if f_lfo >= 128 then m_b <= resize(f_lfo - 128, 11);
                               else m_b <= resize(128 - f_lfo, 11); m_neg <= '1'; end if;
                    when 8  => m_a <= resize(e_dmag, 18); m_b <= f_bendl; m_neg <= e_dneg;
                    when 9  => m_a <= resize(e_incb, 18); m_b <= f_tiles;
                    when 10 => m_a <= e_incv; m_b <= resize(e_jit, 11); m_neg <= e_jneg;
                    when 11 => if e_dt(15) = '1' then m_a <= resize(0 - e_dt, 18); m_neg <= '1';
                               else m_a <= resize(e_dt, 18); end if;
                               m_b <= resize(f_k3, 11);
                    when 12 | 13 =>
                        m_a <= resize(e_dy, 18);
                        if e_band = 4 then
                            if q_job = 12 then m_b <= to_unsigned(839, 11); else m_b <= to_unsigned(629, 11); end if;
                            m_neg <= '1';
                        elsif e_band = 5 then
                            if q_job = 12 then m_b <= to_unsigned(1028, 11); else m_b <= to_unsigned(1395, 11); m_neg <= '1'; end if;
                        else
                            m_b <= (others => '0');
                        end if;
                    when 14 => m_a <= resize(e_dy(6 downto 0), 18); m_b <= to_unsigned(1171, 11);   -- row = dy * 1171 / 8192 = floor(dy / 7)
                    when others =>
                        if e_y >= 1064 then m_a <= resize(e_y - 1064, 18); m_b <= resize(e_y - 1064, 11);
                        elsif e_y < 1064 - 255 then m_a <= to_unsigned(255, 18); m_b <= to_unsigned(255, 11);
                        else m_a <= resize(1064 - e_y, 18); m_b <= resize(1064 - e_y, 11); end if;
                end case;
            elsif q_run = '1' and q_ph >= 1 and q_ph <= 11 then
                if m_b(0) = '1' then v_s := resize(m_h, 19) + resize(m_a, 19); else v_s := resize(m_h, 19); end if;
                m_h <= v_s(18 downto 1);
                m_l <= v_s(0) & m_l(10 downto 1);
                m_b <= '0' & m_b(10 downto 1);
            elsif q_run = '1' and q_ph = 13 then
                v_pr := m_h & m_l;
                case to_integer(q_job) is
                    when 3 =>
                        if m_neg = '1' then f_bendl <= resize(f_bend, 11) - resize(v_pr(19 downto 10), 11);
                        else f_bendl <= resize(f_bend, 11) + resize(v_pr(19 downto 10), 11); end if;
                    when 8 =>
                        if m_neg = '1' then e_incb <= to_unsigned(C_INCC, 17) - resize(v_pr(24 downto 8), 17);
                        else e_incb <= to_unsigned(C_INCC, 17) + resize(v_pr(24 downto 8), 17); end if;
                    when 9 => e_incv <= v_pr(26 downto 9);
                    when 10 =>
                        if m_neg = '1' then e_dt <= 0 - v_pr(23 downto 8); else e_dt <= v_pr(23 downto 8); end if;
                    when 11 =>
                        if m_neg = '1' then e_dc <= 0 - v_pr(15 downto 0); else e_dc <= v_pr(15 downto 0); end if;
                    when 12 | 13 =>
                        if m_neg = '1' then v_sg := 0 - signed(resize(v_pr(19 downto 4), 17));
                        else v_sg := signed(resize(v_pr(19 downto 4), 17)); end if;
                        if m_neg = '1' and v_pr(3 downto 0) /= 0 then v_sg := v_sg - 1; end if;
                        if q_job = 12 then e_lin0 <= unsigned(v_sg(15 downto 0)); else e_lin1 <= unsigned(v_sg(15 downto 0)); end if;
                    when 14 =>
                        if e_band = 12 then e_row <= resize(shift_right(e_dy, 3), 7); else e_row <= v_pr(19 downto 13); end if;
                    when others => e_dy2 <= v_pr(15 downto 0);
                end case;
            end if;

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
    -- Per-line engine: kicked at the source avid rise (and late in the vblank
    -- zone for line 0); computes the NEXT line during this line, and writes
    -- the live line values in the horizontal blanking (24 clocks after the
    -- source avid fall, when this line's last pixel has left the pixel path)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y  : unsigned(11 downto 0);
        variable v_d  : signed(11 downto 0);
        variable v_i  : unsigned(9 downto 0);
        variable v_s  : unsigned(31 downto 0);
        variable v_m  : unsigned(10 downto 0);
        variable v_n  : t_lv;
        variable v_r  : unsigned(3 downto 0);
        variable v_ad : unsigned(10 downto 0);
        variable v_t  : unsigned(15 downto 0);
        variable v_w  : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- tear seed: redrawn every 8 fields
            if s_zone = '1' and s_zc = 224 then
                v_t := "000" & f_fld(15 downto 3);
                t_s <= (x"9E3779B9" xor (v_t & v_t)) or x"00000001";
                t_run <= (others => '0');
            end if;
            -- assembly trigger: clocks since the source avid fall
            if data_in.avid = '0' and s_avq = '1' then s_fc <= (others => '0');
            elsif s_fc /= 63 then s_fc <= s_fc + 1; end if;
            -- kick
            if s_zone = '1' and s_zc = 512 then
                if t_il = '1' and f_odd = '1' then e_vy <= resize(t_ys(11 downto 1), 20); else e_vy <= (others => '0'); end if;
                e_slot <= (others => '0'); e_ph <= (others => '0'); e_zk <= '1'; e_done <= '0';
            elsif data_in.avid = '1' and s_avq = '0' then
                e_vy <= e_vy + t_ys;
                e_slot <= (others => '0'); e_ph <= (others => '0'); e_zk <= '0'; e_done <= '0';
            elsif e_slot /= 15 then
                if e_ph = 19 then
                    e_ph <= (others => '0');
                    if e_slot = 11 then e_slot <= (others => '1'); e_done <= '1'; else e_slot <= e_slot + 1; end if;
                else
                    e_ph <= e_ph + 1;
                end if;
            end if;

            if e_slot = 0 then
                case to_integer(e_ph) is
                    when 0 =>
                        if e_vy(19 downto 8) > 1079 then e_yi <= to_unsigned(1079, 11); else e_yi <= e_vy(18 downto 8); end if;
                    when 1 =>
                        v_y := resize(e_yi, 12) + resize(f_roll, 12);
                        if v_y >= 1080 then e_y <= resize(v_y - 1080, 11); else e_y <= v_y(10 downto 0); end if;
                    when 2 =>
                        v_d := signed(resize(e_y, 12)) - 151;
                        if v_d < 0 then v_d := -v_d; end if;
                        if v_d > 127 then e_d <= to_unsigned(127, 7); else e_d <= unsigned(v_d(6 downto 0)); end if;
                        r_la <= resize(e_y(10 downto 1), 10);
                        -- band search restarts after a wrap (roll) or at the field's first line
                        if e_zk = '1' or e_y < e_yp then e_band <= (others => '0'); e_bnext <= to_unsigned(38, 11); end if;
                        e_yp <= e_y;
                        -- tear run generator (one draw per run)
                        if t_run = 0 then
                            v_s := f_xs(t_s);
                            t_s <= v_s;
                            t_run <= resize(v_s(4 downto 0), 6) + 2;
                            if v_s(7 downto 5) < 2 then t_hit <= '1'; else t_hit <= '0'; end if;
                            t_mag <= to_unsigned(4, 5) + resize(v_s(11 downto 8), 5) + resize(v_s(15 downto 13), 5);
                            t_sgn <= v_s(16);
                        else
                            t_run <= t_run - 1;
                        end if;
                    when 3 =>
                        if e_y(10 downto 1) >= 539 then r_la <= to_unsigned(539, 10); else r_la <= resize(e_y(10 downto 1), 10) + 1; end if;
                        v_m := resize(t_mag * f_tk, 11);
                        if t_hit = '1' and f_tear = '1' then e_jit <= resize(shift_right(v_m, 4), 6); else e_jit <= (others => '0'); end if;
                        e_jneg <= not t_sgn;
                    when 4 =>
                        r_la <= to_unsigned(544, 10) + resize(e_d, 10);
                    when 5 =>
                        e_inca <= unsigned(r_lq2);
                        r_la <= to_unsigned(672, 10) + resize(e_d, 10);
                    when 6 =>
                        e_incb2 <= unsigned(r_lq2);
                        if e_y >= 300 and e_y < 412 then v_i := resize(e_y - 300, 10); else v_i := (others => '0'); end if;
                        r_la <= to_unsigned(800, 10) + v_i;
                    when 7 =>
                        e_lr <= unsigned(r_lq2);
                        if e_y >= 645 and e_y < 700 then v_i := resize(e_y - 645, 10); else v_i := (others => '0'); end if;
                        r_la <= to_unsigned(928, 10) + v_i;
                    when 8 =>
                        e_gn <= unsigned(r_lq2);
                        if e_y >= 31 and e_y <= 53 then
                            if e_y >= 42 then v_i := resize(e_y - 42, 10); else v_i := resize(42 - e_y, 10); end if;
                            r_la <= to_unsigned(992, 10) + v_i;
                        elsif e_y >= 258 and e_y <= 280 then
                            if e_y >= 269 then v_i := resize(e_y - 269, 10); else v_i := resize(269 - e_y, 10); end if;
                            r_la <= to_unsigned(992, 10) + v_i;
                        elsif e_y >= 93 and e_y <= 105 then
                            r_la <= to_unsigned(1008, 10) + resize(106 - e_y, 10);
                        elsif e_y >= 198 and e_y <= 211 then
                            r_la <= to_unsigned(1008, 10) + resize(e_y - 198, 10);
                        else
                            r_la <= to_unsigned(1023, 10);                  -- (0)
                        end if;
                    when 9 =>
                        e_hw <= unsigned(r_lq2);
                    when 10 =>
                        e_koa <= unsigned(r_lq2);
                    when 11 =>
                        e_fil <= unsigned(r_lq2);
                        if e_y(0) = '1' then
                            e_inck <= shift_right(resize(e_inca, 17) + resize(e_incb2, 17), 1);
                        else
                            e_inck <= resize(e_inca, 17);
                        end if;
                    when 12 =>
                        if e_inck >= C_INCC then
                            e_dneg <= '0'; e_dmag <= resize(e_inck - C_INCC, 12);
                        else
                            e_dneg <= '1'; e_dmag <= resize(to_unsigned(C_INCC, 17) - e_inck, 12);
                        end if;
                    when 16 =>
                        v_ad := e_y - f_bystart(e_band);
                        if v_ad > 255 then e_dy <= (others => '1'); else e_dy <= v_ad(7 downto 0); end if;
                    when others => null;
                end case;
                -- band search: one step per clock (phases 3 .. 15)
                if e_ph >= 3 and e_ph <= 15 then
                    if e_y >= e_bnext and e_band < 12 then
                        e_band <= e_band + 1;
                        e_bnext <= f_bystart(e_band + 2);
                    end if;
                end if;
            end if;
            -- slot 8: the dash-row hash from the line ROM
            if e_slot = 8 then
                case to_integer(e_ph) is
                    when 0 => r_la <= to_unsigned(912, 10) + resize(e_row(3 downto 0), 10);
                    when 3 => e_ha <= unsigned(r_lq2);
                    when 4 => e_hb <= f_mix(e_ha);
                    when others => null;
                end case;
            end if;

            -- assemble the live line values in the horizontal blanking
            if e_done = '1' and (e_zk = '1' or s_fc = 24) then
                e_done <= '0';
                v_n := C_LV0;
                case to_integer(t_xs) is
                    when 4 => v_n.incr := resize(e_incv & '0', 20);
                    when 3 => v_n.incr := resize(e_incv, 20) + resize(e_incv(17 downto 1), 20);
                    when others => v_n.incr := resize(e_incv, 20);
                end case;
                v_n.dt := e_dt; v_n.dc := e_dc;
                v_r := resize(f_rip, 4);
                case to_integer(e_band) is
                    when 0 => v_n.gidx := to_unsigned(9, 6);
                    when 1 =>
                        v_n.gidx := to_unsigned(12, 6) + resize(e_dy(7 downto 6), 6);
                        if e_y >= 42 and e_y < 270 then
                            v_n.pon := '1'; v_n.psel := '0';
                            v_n.pa := e_lr;
                            if e_y >= 106 and e_y < 198 then
                                v_n.pbar := '1'; v_n.punder := '1';
                                v_n.pw := (to_unsigned(62390, 16) - e_fil) - e_lr;
                            elsif (e_y >= 93 and e_y <= 105) or (e_y >= 198 and e_y <= 211) then
                                v_n.pw := (to_unsigned(62390, 16) + e_fil) - e_lr;
                            else
                                v_n.pw := (to_unsigned(62390, 16) - e_fil) - e_lr;
                            end if;
                            v_n.pamp := unsigned(e_gn(14 downto 10));
                            if e_y < 149 then v_n.pcls := "01"; elsif e_y < 236 then v_n.pcls := "10"; else v_n.pcls := "11"; end if;
                            if (e_y >= 43 and e_y <= 45) or (e_y >= 267 and e_y <= 269) then v_n.prim := '1'; end if;
                            if (e_y >= 106 and e_y <= 109) or (e_y >= 195 and e_y <= 197) then v_n.primb := '1'; end if;
                            if e_y >= 200 then v_n.plo := resize(shift_right(e_y - 200, 4), 3); end if;
                            if not (e_y >= 106 and e_y < 198) then
                                v_n.echo := '1'; v_n.glow := '1';
                                v_n.nb := unsigned(e_gn(1 downto 0));
                            end if;
                        end if;
                    when 2 =>
                        if e_dy(7 downto 3) >= 2 then v_n.gidx := to_unsigned(9, 6);
                        else v_n.gidx := to_unsigned(11, 6) - resize(e_dy(7 downto 3), 6); end if;
                    when 3 =>
                        v_n.gidx := to_unsigned(9, 6);
                        v_n.aon := '1'; v_n.asel := "00";
                        v_n.aa := to_unsigned(1966, 16) - e_hw; v_n.aw := e_hw(14 downto 0) & '0';
                        v_n.ashl := to_unsigned(5, 4) + v_r; v_n.ashr := to_unsigned(6, 4) + v_r; v_n.ag := to_unsigned(16, 5);
                    when 4 =>
                        v_n.gidx := to_unsigned(8, 6);
                        v_n.aon := '1'; v_n.asel := "00";
                        v_n.aa := to_unsigned(43254, 16); v_n.aw := to_unsigned(11141 + 65536 - 43254, 16);
                        v_n.ashl := to_unsigned(4, 4) + v_r; v_n.ashr := to_unsigned(5, 4) + v_r; v_n.ag := to_unsigned(16, 5);
                        if e_y >= 420 then
                            v_n.pon := '1'; v_n.psel := '1';
                            v_n.pa := to_unsigned(35429, 16) + e_lin0;
                            v_n.pw := (to_unsigned(42481, 16) + e_lin1) - (to_unsigned(35429, 16) + e_lin0);
                            v_n.pcls := "01"; v_n.pamp := to_unsigned(16, 5);
                            if e_y <= 422 then v_n.prim := '1'; end if;
                        end if;
                    when 5 =>
                        v_n.gidx := to_unsigned(3, 6);
                        if e_y < 558 then
                            v_n.pon := '1'; v_n.psel := '1'; v_n.glow := '1';
                            v_n.pa := to_unsigned(30774, 16) + e_lin0;
                            v_n.pw := (to_unsigned(43258, 16) + e_lin1) - (to_unsigned(30774, 16) + e_lin0);
                            if e_y < 520 then v_n.pcls := "10"; else v_n.pcls := "11"; end if;
                            v_n.pamp := to_unsigned(16, 5);
                        end if;
                    when 6 =>
                        v_n.gidx := to_unsigned(4, 6);
                        v_n.aon := '1'; v_n.asel := "01";
                        v_n.aa := to_unsigned(63570, 16); v_n.aw := to_unsigned(20316 + 65536 - 63570, 16);
                        v_n.ashl := to_unsigned(4, 4) + v_r; v_n.ashr := to_unsigned(5, 4) + v_r; v_n.ag := to_unsigned(16, 5);
                    when 7 =>
                        v_n.gidx := to_unsigned(5, 6);
                        v_n.mode := "01"; v_n.wob := '1'; v_n.envon := '1'; v_n.envk := to_unsigned(63, 7);
                        v_n.aon := '1'; v_n.asel := "10";
                        v_n.aa := (others => '0'); v_n.aw := to_unsigned(43254, 16);
                        v_n.ashl := to_unsigned(7, 4) + v_r; v_n.ashr := to_unsigned(5, 4) + v_r; v_n.ag := to_unsigned(16, 5);
                    when 8 =>
                        v_n.gidx := to_unsigned(5, 6);
                        v_n.mode := "10";
                        if e_y < 770 then
                            v_n.envon := '1';
                            if e_y + 64 <= 770 then v_w := to_unsigned(64, 7); else v_w := resize(to_unsigned(770, 11) - e_y, 7); end if;
                            v_n.envk := v_w - 1;
                        end if;
                        v_n.aon := '1'; v_n.asel := "10";
                        v_n.aa := (others => '0'); v_n.aw := to_unsigned(40632, 16) + resize(e_hb(7 downto 0) * to_unsigned(51, 6), 16);
                        v_n.ashl := to_unsigned(8, 4); v_n.ashr := to_unsigned(7, 4);
                    when 9 =>
                        if e_y < 792 then v_n.gidx := to_unsigned(6, 6); else v_n.gidx := to_unsigned(35, 6); end if;
                        v_n.aon := '1'; v_n.asel := "01";
                        v_n.aa := (others => '0'); v_n.aw := to_unsigned(22282, 16);
                        v_n.ashl := to_unsigned(5, 4) + v_r; v_n.ashr := to_unsigned(6, 4) + v_r; v_n.ag := to_unsigned(10, 5);
                    when 10 =>
                        v_n.gidx := to_unsigned(36, 6);
                        if e_y >= 860 and e_y < 946 then v_n.aron := '1'; v_n.ak := unsigned(signed(resize(e_dy2, 17)) - 29584); end if;
                    when 11 =>
                        v_n.gidx := to_unsigned(4, 6);
                        if e_y < 1016 then
                            v_n.aon := '1'; v_n.asel := "01"; v_n.agate := '1';
                            v_n.aa := to_unsigned(5243, 16); v_n.aw := to_unsigned(27525 - 5243, 16);
                            v_n.ashl := to_unsigned(2, 4) + v_r; v_n.ashr := to_unsigned(4, 4) + v_r; v_n.ag := to_unsigned(5, 5);
                            v_n.pon := '1'; v_n.psel := '0'; v_n.pgate := '1';
                            v_n.pa := to_unsigned(44564, 16);
                            v_n.pw := to_unsigned(60293 - 44564, 16);
                            v_n.pcls := "11"; v_n.pamp := to_unsigned(14, 5);
                        end if;
                    when others =>
                        v_n.gidx := to_unsigned(3, 6);
                        if e_y >= 1022 then
                            v_n.mode := "11"; v_n.envon := '1'; v_n.envs := '1'; v_n.envc := '1';
                            v_n.envk := to_unsigned(88, 7);
                            v_n.aon := '1'; v_n.asel := "10";
                            v_n.aa := (others => '0'); v_n.aw := to_unsigned(36045, 16);
                            v_n.ashl := to_unsigned(7, 4); v_n.ashr := to_unsigned(6, 4);
                        end if;
                end case;
                -- arcs / dash / floor phase offset (4 x the camera crenel phase + koff)
                if e_band = 7 then
                    v_n.koff4 := e_koa + (e_dc(13 downto 0) & "00");
                elsif e_band = 8 then
                    v_n.koff4 := (e_ha(15 downto 8) & x"00") + (e_dc(13 downto 0) & "00");
                else
                    v_n.koff4 := (e_ha(7 downto 0) & x"00") + (e_dc(13 downto 0) & "00");
                end if;
                -- dash gains (row tent x row brightness)
                if e_band = 8 then
                    v_ad := e_y - 700;
                    v_i := resize(v_ad - shift_left(resize(e_row, 11), 3) + resize(e_row, 11), 10);   -- (dy - 7 row)
                    case to_integer(v_i(2 downto 0)) is
                        when 0 | 6 => v_m := to_unsigned(6, 11);
                        when 1 | 5 => v_m := to_unsigned(13, 11);
                        when others => v_m := to_unsigned(16, 11);
                    end case;
                    v_m := resize(v_m(4 downto 0) * (to_unsigned(9, 5) + resize(e_hb(15 downto 13), 5)), 11);   -- 9 .. 16 needs 5 bits
                    v_n.ag := resize(shift_right(v_m, 4), 5);
                elsif e_band = 12 and e_y >= 1022 then
                    case to_integer(e_dy(2 downto 0)) is
                        when 0 | 7 => v_n.ag := to_unsigned(5, 5);
                        when 1 | 6 => v_n.ag := to_unsigned(11, 5);
                        when others => v_n.ag := to_unsigned(16, 5);
                    end case;
                end if;
                l_lv <= v_n;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Source side: camera luma or the Osc source into the line buffer
    -- (1024 entries per line, two banks by line parity)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(9 downto 0);
        variable v_s : unsigned(11 downto 0);
        variable v_a, v_b : unsigned(14 downto 0);
        variable v_da, v_db : unsigned(14 downto 0);
        variable v_o : unsigned(8 downto 0);
        variable v_ta : unsigned(14 downto 0);
        variable v_h : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            -- per source line: Osc phases (advanced at each source avid fall)
            if s_zone = '1' then
                s_ya <= resize(f_ot, 16) + resize(f_ot & '0', 16);
            elsif data_in.avid = '0' and s_avq = '1' then
                s_ya <= s_ya + t_oya;
                s_wbank <= not s_wbank;
            end if;
            -- aoff = 0.625 x the signed triangle of ya (one line late: harmless)
            if s_ya(15) = '0' then v_ta := s_ya(14 downto 0); else v_ta := not s_ya(14 downto 0); end if;
            v_h := to_signed(16383, 17) - signed(resize(v_ta, 17));
            s_aoff <= unsigned(resize(v_h + shift_right(v_h, 2), 16));
            -- write address DDA
            if data_in.avid = '1' then s_wacc <= s_wacc + t_ws; else s_wacc <= (others => '0'); end if;
            -- 1: luma and the address
            w1_we <= data_in.avid; w1_b <= s_wbank;
            w1_a <= s_wacc(25 downto 16);
            if t_sd = '1' then w1_x <= s_wacc(24 downto 16) & '0'; else w1_x <= s_wacc(25 downto 16); end if;
            if unsigned(data_in.y) > 64 then v_l := unsigned(data_in.y) - 64; else v_l := (others => '0'); end if;
            v_s := resize(v_l(9 downto 2), 12) + resize(v_l(9 downto 5), 12) + resize(v_l(9 downto 6), 12);
            if v_s > 255 then w1_l <= to_unsigned(255, 8); else w1_l <= v_s(7 downto 0); end if;
            -- 2: Osc phases
            w2_we <= w1_we; w2_b <= w1_b; w2_a <= w1_a; w2_l <= w1_l; w2_osc <= f_osc;
            w2_pa <= (resize(w1_x, 16) sll 8) + (resize(w1_x, 16) sll 6) + s_aoff;
            -- 3: Osc level (bright, dipped near the two triangles' peaks) or the luma
            if w2_pa(15) = '0' then v_a := w2_pa(14 downto 0); else v_a := not w2_pa(14 downto 0); end if;
            if v_a < 9830 then v_da := to_unsigned(9830, 15) - v_a; else v_da := (others => '0'); end if;
            v_o := to_unsigned(255, 9) - resize(v_da(14 downto 8), 9);
            w3_we <= w2_we; w3_b <= w2_b; w3_a <= w2_a;
            if w2_osc = '1' then w3_d <= v_o(7 downto 0); else w3_d <= w2_l; end if;
            -- write
            lb_we <= w3_we;
            lb_wa <= w3_b & w3_a;
            lb_wd <= std_logic_vector(w3_d);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam: tile phase (25 bits: tile parity on top)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_p <= '0' & f_phi;
                s_rbank <= not s_wbank;
            elsif g_avid = '1' then
                b_p <= b_p + resize(l_lv.incr, 25);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_p : unsigned(15 downto 0);
        variable v_k1, v_k2, v_le : unsigned(5 downto 0);
        variable v_a10 : unsigned(9 downto 0);
        variable v_lv : signed(8 downto 0);
        variable v_e : unsigned(12 downto 0);
        variable v_dr : signed(18 downto 0);
        variable v_g : signed(8 downto 0);
        variable v_c : unsigned(5 downto 0);
        variable v_cl : unsigned(2 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_yl : unsigned(9 downto 0);
        variable v_d8 : unsigned(7 downto 0);
        variable v_ev : signed(7 downto 0);
        variable v_c19, v_b19 : unsigned(18 downto 0);
        variable v_ar : signed(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: tile phase (mirrored on odd tiles)
            if f_mir = '1' and b_p(24) = '1' then p1_t <= not b_p(23 downto 8); else p1_t <= b_p(23 downto 8); end if;
            -- 2: crenel phase product (shift-add: k3 = 1 .. 6), torn tile phase
            p2_t <= p1_t;
            p2_tc <= p1_t + l_lv.dt;
            case f_ca is
                when "00" => v_c19 := resize(p1_t, 19);
                when "01" => v_c19 := resize(p1_t & '0', 19);
                when others => v_c19 := resize(p1_t & "00", 19);
            end case;
            if f_cben = '1' then
                if f_cb = '1' then v_b19 := resize(p1_t & '0', 19); else v_b19 := resize(p1_t, 19); end if;
            else
                v_b19 := (others => '0');
            end if;
            p2_ct <= v_c19 + v_b19;
            -- 3: crenel phase / tooth, camera crenel phase, arcs phase (4 x crenel + offset + wobble);
            -- line-buffer read
            p3_t <= p2_t; p3_tc <= p2_tc;
            p3_c <= p2_ct(15 downto 0);
            p3_tooth <= p2_ct(18 downto 16);
            p3_cc <= p2_ct(15 downto 0) + l_lv.dc;
            if l_lv.wob = '1' then
                p3_kc <= (p2_ct(13 downto 0) & "00") + l_lv.koff4 + unsigned(C_WOB(to_integer(p2_tc(15 downto 10)))(15 downto 0));
            else
                p3_kc <= (p2_ct(13 downto 0) & "00") + l_lv.koff4;
            end if;
            if t_sd = '1' then lb_ra <= s_rbank & '0' & p2_tc(15 downto 7); else lb_ra <= s_rbank & p2_tc(15 downto 6); end if;
            -- 4: pink window offset; camera offset; arch / env offsets (10-bit tile phase)
            if l_lv.psel = '1' then v_p := p3_t; else v_p := p3_c; end if;
            p4_q <= v_p - l_lv.pa;
            case l_lv.asel is
                when "00" => v_p := p3_tc;
                when "01" => v_p := p3_cc;
                when others => v_p := p3_kc;
            end case;
            p4_qa <= v_p - l_lv.aa;
            p4_t10 <= p3_t(15 downto 6);
            p4_dx <= p3_t(15 downto 6) - 41;
            if l_lv.envc = '1' then p4_de <= p3_tc(15 downto 6) - 584; else p4_de <= p3_tc(15 downto 6) - 614; end if;
            p4_tooth <= p3_tooth;
            -- 5: pink window, ridge index, left distance; camera window / left key; arch px; source
            p5_src <= unsigned(lb_q);
            p5_wq <= resize(l_lv.pw, 17) - resize(p4_q, 17);
            if p4_q(15) = '1' then p5_dli <= to_unsigned(63, 6);
            else v_d8 := f_d8(p4_q, l_lv.psel); if v_d8 > 63 then p5_dli <= to_unsigned(63, 6); else p5_dli <= v_d8(5 downto 0); end if; end if;
            p5_dn <= f_d8(not p4_q, l_lv.psel);
            if l_lv.pgate = '1' and p4_tooth = 1 then p5_gate <= '1'; else p5_gate <= '0'; end if;
            p5_wqa <= resize(l_lv.aw, 17) - resize(p4_qa, 17);
            p5_kl <= f_win6(p4_qa, l_lv.ashl);
            if l_lv.agate = '1' and p4_tooth = 0 then p5_gq <= '1'; else p5_gq <= '0'; end if;
            if p4_dx(9) = '1' then v_a10 := not p4_dx; else v_a10 := p4_dx; end if;
            v_a10 := v_a10 - shift_right(v_a10, 4);
            if v_a10 > 255 then p5_px <= to_unsigned(255, 8); else p5_px <= v_a10(7 downto 0); end if;
            if p4_t10 > 158 and p4_t10 < 512 then p5_cut <= '1'; else p5_cut <= '0'; end if;
            if p4_de(9) = '1' then p5_ae <= not p4_de; else p5_ae <= p4_de; end if;
            -- 6: right distance, rims, ridge profile, echo profile address; camera key; arch square; env
            if p5_wq(16) = '1' then p6_drn <= f_d8(not p5_wq(15 downto 0), l_lv.psel); else p6_drn <= (others => '1'); end if;
            p6_win <= not p5_wq(16) and not p5_gate;
            p6_inside <= (not p5_wq(16) and not p5_gate) or l_lv.pbar;
            if (l_lv.prim = '1' and (p5_wq(16) = '0' or l_lv.pbar = '1'))
               or (l_lv.primb = '1' and p5_wq(16) = '1' and l_lv.pbar = '1')
               or (l_lv.pgate = '0' and p5_wq(16) = '0' and f_d8(p5_wq(15 downto 0), l_lv.psel) < 4) then
                p6_rim <= '1';
            else
                p6_rim <= '0';
            end if;
            p6_rid <= unsigned(C_RID(to_integer(p5_dli)));
            r_eca <= l_lv.nb & p5_dn;
            p6_dn <= p5_dn;
            if l_lv.punder = '1' and p5_dn >= 1 and p5_dn < 6 then p6_under <= '1'; else p6_under <= '0'; end if;
            v_k2 := f_win6(p5_wqa(15 downto 0), l_lv.ashr);
            if v_k2 < p5_kl then v_k1 := v_k2; else v_k1 := p5_kl; end if;
            if p5_wqa(16) = '1' or p5_gq = '1' or l_lv.aon = '0' then v_k1 := (others => '0'); end if;
            p6_key <= v_k1;
            r_sqa <= p5_px;
            if l_lv.envs = '1' then v_a10 := shift_right(p5_ae, 2); else v_a10 := shift_right(p5_ae, 1); end if;
            if resize(v_a10, 11) >= resize(l_lv.envk, 11) then p6_env <= (others => '0');
            elsif l_lv.envk - resize(v_a10, 7) > 63 then p6_env <= to_unsigned(63, 6);
            else p6_env <= resize(l_lv.envk - v_a10(6 downto 0), 6); end if;
            p6_cut <= p5_cut;
            p_sr(6) <= p5_src;
            -- 7: ridge amplitude, glow index; camera key gain
            p7_r <= p6_rid * l_lv.pamp;
            if p6_win = '0' then
                if p6_dn < p6_drn then v_d8 := p6_dn; else v_d8 := p6_drn; end if;
            else
                v_d8 := (others => '1');
            end if;
            if v_d8(7 downto 6) /= 0 then p7_gi <= to_unsigned(31, 5); else p7_gi <= v_d8(5 downto 1); end if;
            p7_kg <= p6_key * l_lv.ag;
            p7_env <= p6_env;
            p7_under <= p6_under; p7_rim <= p6_rim; p7_inside <= p6_inside;
            p7_cut <= p6_cut;
            p_sr(7) <= p_sr(6);
            -- 8: fill level, glow, echo profile (EBR out), camera key vs env; square (fabric copy)
            v_lv := signed(resize(p7_r(10 downto 4), 9));
            if v_lv > 61 then v_lv := to_signed(61, 9); end if;
            v_ev := resize(shift_right(signed('0' & p_sr(7)) - 128, 5), 8);
            v_lv := v_lv + 1 + resize(v_ev, 9) + signed(resize(l_lv.plo, 9));
            if v_lv < 1 then v_lv := to_signed(1, 9); elsif v_lv > 63 then v_lv := to_signed(63, 9); end if;
            if p7_under = '1' then p8_lv <= (others => '0');
            elsif p7_rim = '1' then p8_lv <= to_unsigned(32, 6);
            else p8_lv <= unsigned(v_lv(5 downto 0)); end if;
            if f_outl = '1' then
                if p7_rim = '1' or p7_r(10 downto 4) >= 10 then p8_inside <= p7_inside; else p8_inside <= '0'; end if;
            else
                p8_inside <= p7_inside;
            end if;
            p8_glow <= unsigned(C_GLO(to_integer(p7_gi)));
            p8_ec <= unsigned(r_ec);
            v_k1 := p7_kg(9 downto 4);
            p8_full <= '1';
            if l_lv.mode = "01" and l_lv.envon = '1' then
                if p7_env > v_k1 then v_k1 := p7_env; end if;
            elsif l_lv.mode = "10" then
                p8_full <= '0';
                if l_lv.envon = '1' and (p7_env & "0000") > (resize(v_k1 & "0000", 10) - resize(v_k1 & '0', 10)) then
                    v_k1 := p7_env; p8_full <= '1';
                end if;
            elsif l_lv.mode = "11" then
                p8_full <= '0';
                if p7_env < v_k1 then v_k1 := p7_env; end if;
            end if;
            p8_key <= v_k1;
            if l_lv.mode = "01" or l_lv.mode = "10" or v_k1 /= 0 then p8_camm <= '1'; else p8_camm <= '0'; end if;
            p8_sq <= unsigned(r_sq);
            p8_cut <= p7_cut;
            p_sr(8) <= p_sr(7);
            -- 9: echo product, ground level, camera level, ring d2
            p9_lv <= p8_lv; p9_inside <= p8_inside and l_lv.pon;
            p9_ev <= p8_ec * f_eg;
            p9_echoon <= l_lv.echo and not p8_inside;
            if l_lv.glow = '1' and p8_inside = '0' then v_g := signed(resize(l_lv.gidx, 9)) + signed(resize(p8_glow, 9));
            else v_g := signed(resize(l_lv.gidx, 9)); end if;
            if v_g > 63 then p9_glev <= to_unsigned(63, 6); else p9_glev <= unsigned(v_g(5 downto 0)); end if;
            p9_cam <= p8_key * p_sr(8);
            p9_full <= p8_full; p9_camm <= p8_camm;
            p9_d2m <= signed(resize(p8_sq, 19)) + resize(signed(l_lv.ak), 19);    -- ak is negative: sign-extend
            p9_cut <= p8_cut;
            -- 10: echo level, camera colouriser level, ring radius
            p10_lv <= p9_lv; p10_inside <= p9_inside; p10_glev <= p9_glev; p10_camm <= p9_camm;
            if p9_echoon = '0' then p10_echo <= (others => '0');
            elsif p9_ev(14 downto 4) > 63 then p10_echo <= to_unsigned(63, 6);
            else p10_echo <= p9_ev(9 downto 4); end if;
            v_c := p9_cam(13 downto 8);
            if p9_full = '1' then
                p10_lec <= resize(to_unsigned(2, 7) + v_c - shift_right(resize(v_c, 8) * 3, 6), 6);
            else
                p10_lec <= resize(shift_right(resize(v_c, 10) * 13, 6), 6);
            end if;
            v_dr := shift_right(p9_d2m, 9) + shift_right(p9_d2m, 10);
            if v_dr < -64 then p10_dr <= to_signed(-65, 10);
            elsif v_dr > 63 then p10_dr <= to_signed(64, 10);
            else p10_dr <= resize(v_dr, 10); end if;
            p10_cut <= p9_cut;
            -- 11: priority -> class / level
            if p10_inside = '1' then
                p_code(11) <= "0" & l_lv.pcls & p10_lv;
            elsif l_lv.aron = '1' and p10_dr >= -64 and p10_dr <= 63 and p10_cut = '0' then
                v_ar := shift_right(p10_dr + 64, 1);                       -- 0 .. 63: slice, not resize
                p_code(11) <= "101" & unsigned(v_ar(5 downto 0));
            elsif p10_echo > 1 then
                p_code(11) <= "100" & p10_echo;
            elsif p10_camm = '1' then
                p_code(11) <= "110" & p10_lec;
            else
                p_code(11) <= "000" & p10_glev;
            end if;
            -- 12..14: pad (the 24-clock latency)
            for i in 12 to 14 loop p_code(i) <= p_code(i - 1); end loop;
            -- 15: colour ROM address; 16: ROM out; 17: fabric copy (to 10 bits)
            r_ca <= f_pal & p_code(14);
            p17_y <= unsigned(r_cy) & "00"; p17_u <= unsigned(r_cu) & "00"; p17_v <= unsigned(r_cv) & "00";
            -- 18..23: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent 8-sums,
            -- luma [1 2 1]; centre at stage 22 -> 23)
            s_csu(0) <= p17_u; s_csv(0) <= p17_v;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + p17_u - s_csu(7);
            s_boxv <= s_boxv + p17_v - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_uf <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_vf <= v_sc(13 downto 4);
            s_ya1 <= p17_y; s_yb <= s_ya1; s_yc <= s_yb; s_yd <= s_yc;
            s_ye <= s_yd; s_yfx <= s_ye;
            v_sy := resize(s_yd, 12) + shift_left(resize(s_ye, 12), 1) + resize(s_yfx, 12);
            s_yf <= v_sy(11 downto 2);
            -- 24: luma toward black (white 940 -> 777); blanking gate
            if s_yf > 64 then v_yl := s_yf - 64; else v_yl := (others => '0'); end if;
            if s_gav_sr(22) = '1' then
                s_yg <= s_yf - shift_right(v_yl, 3) - shift_right(v_yl, 4);
                s_ug <= s_uf; s_vg <= s_vf;
            else
                s_yg <= to_unsigned(64, 10); s_ug <= to_unsigned(512, 10); s_vg <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_yg);
    data_out.u       <= std_logic_vector(s_ug);
    data_out.v       <= std_logic_vector(s_vg);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture crenel;
