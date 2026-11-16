-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Tweed v0.1.0 -- Hardline.  Colour-and-weave effects from a weaving draft: houndstooth, shepherd's check,
-- glen plaid, log cabin, pinwheels, hairline stripes, herringbone.  The real textile rule draws all of them:
--
--   pixel = weave(i, j) ? colour(i) : colour(j)
--
-- i, j are the warp and weft thread indices in cloth coordinates (u, v), weave is one of eight 8 x 8 drafts
-- (1 = the warp thread is on top) and colour is one of eight dark / light thread orders of period 16, used
-- for warp and weft alike.  Black and white only: U = V = 512 on every pixel.
--
--   (u, v) = R(turn) (x - pivot) / thread size  +-  camera          (threads, Q4.12 in the pixel path)
--
--   K1  X        the cloth's pivot, -512 .. +511 virtual px from the screen centre (it pans the cloth)
--   K2  Y        the same, vertically
--   K3  Period   thread size, 3 .. 64 virtual px (exponential)
--   K4  Weave    Plain / Basket / Twill (2/2) / Warp (3/1) / Broken / Wide (4/4) / Weft (1/3) / Satin
--   K5  Order    1:1 / 2:2 / 3:1 / 4:4 / Glen / Cabin / 1:3 / 8:8
--   K6  Camera   how far the picture's luma pushes the cloth along its diagonal (0 = none)
--   S7  Glide    Off = the knob values of this field start are used as they are / On = eased
--   S8  Draft    Straight / Point (the thread index of the DRAFT reflects every 8 threads)
--   S9  Threads  Flat / Threads (a hairline along one side of every float, in the opposite tone)
--   S10 Weight   Bold (filled) / Fine (outlines of the dark / light figure, white on black)
--   S11 Source   Video / Osc
--   P12 TURN     rotates the cloth about its pivot, bipolar: centre = square, +-90 degrees at the ends
--
-- The cloth coordinates are an accumulator pair (per-pixel and per-line steps from the vblank sequencer, no
-- pixel-path multiplier in the geometry).  Each axis gives a thread index, a sub-thread fraction and a
-- footprint w = |d/dx| + |d/dy| (the per-field steps plus the camera field's own differences), and the
-- picture is the bilinear blend of the pixel's own zone and the nearest zone across each axis, weighted by
-- the 1-px box coverage (17 steps per axis).  Threads finer than about a pixel fade to flat mid-grey.
--
-- The chassis is the Kinetic riley shell:
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         +-1-count holds, glides, stepped knobs, switches
--   [C] ENGINE                           per-field sequencer (one serial multiplier), line DDAs, beam DDAs
--   [D] SOURCE FIELD                     luma (or the Osc lattice) x camera gain -> box -> vertical IIR
--   [E] CLOTH                            footprint, zones, coverage dividers, draft lookup, blend
--   [F] OUTPUT CONDITIONING              luma [1 2 1]/4, 13/16 toward black, blanking gate
--
-- Stage numbering: stage 1 is the generated avid's own clock; a register "at stage S" holds pixel n at
-- T + S + n (T = the g_start clock).  Pixel n sits at x = x0 + n xs half virtual px.  The output register is
-- at stage C_LAT - 3 and the generated avid lands 4 clocks after the delayed source avid (the shell's
-- parity-locked start), so output column j shows pixel j + 4 on screen column j + 4.  s_gav_sr(S - 2) flags
-- pixels 0 .. W - 1 at stage S.  One latency (40) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.tweed_pkg.all;
use work.all;

architecture tweed of program_top is
    constant C_LAT : integer := 40;   -- = processing_delay_clks

    -- TOML defaults: the glides and holds start there
    constant C_DEF_K1  : integer := 512;
    constant C_DEF_K2  : integer := 512;
    constant C_DEF_K3  : integer := 512;
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
    -- |v| held to 13 bits
    function f_abs13(v : signed(16 downto 0)) return unsigned is
        variable v_a : unsigned(16 downto 0);
    begin
        if v(16) = '1' then v_a := unsigned(-v); else v_a := unsigned(v); end if;
        if v_a(16 downto 13) /= "0000" then return to_unsigned(8191, 13); end if;
        return v_a(12 downto 0);
    end function;
    -- the draft's thread index: straight, or reflected every 8 threads
    function f_refl(x : unsigned(3 downto 0); point : std_logic) return unsigned is
    begin
        if point = '1' and x(3) = '1' then return not x(2 downto 0); end if;
        return x(2 downto 0);
    end function;
    -- camera field (threads << sh, Q8.8) -> threads Q4.12, modulo 16 threads
    function f_shf(q : unsigned(15 downto 0); sh : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(sh) is
            when 0      => return q(11 downto 0) & "0000";
            when 1      => return q(12 downto 0) & "000";
            when 2      => return q(13 downto 0) & "00";
            when 3      => return q(14 downto 0) & "0";
            when 4      => return q;
            when others => return '0' & q(15 downto 1);
        end case;
    end function;
    -- its line-to-line difference, the same scaling, held to +-8191
    function f_shd(d : signed(13 downto 0); sh : unsigned(2 downto 0)) return signed is
        variable v : signed(17 downto 0);
    begin
        case to_integer(sh) is
            when 0      => v := d & "0000";
            when 1      => v := resize(d & "000", 18);
            when 2      => v := resize(d & "00", 18);
            when 3      => v := resize(d & "0", 18);
            when 4      => v := resize(d, 18);
            when others => v := resize(d(13 downto 1), 18);
        end case;
        if v > 8191 then return to_signed(8191, 14); end if;
        if v < -8191 then return to_signed(-8191, 14); end if;
        return v(13 downto 0);
    end function;
    -- one corner of the blend: the tone of zone (zu, zv) of the cell at offset (a, b) from the pixel's own
    -- (codes 0 / 1 / 2 = -1 / 0 / +1; bit b * 3 + a of c9 / w9).  A gap zone only occurs at offsets 0 and +1.
    function f_pick(c9 : std_logic_vector(8 downto 0); a, b : unsigned(1 downto 0)) return std_logic is
        variable v_r : std_logic_vector(2 downto 0);
    begin
        case to_integer(b) is
            when 0      => v_r := c9(2 downto 0);
            when 1      => v_r := c9(5 downto 3);
            when others => v_r := c9(8 downto 6);
        end case;
        case to_integer(a) is
            when 0      => return v_r(0);
            when 1      => return v_r(1);
            when others => return v_r(2);
        end case;
    end function;
    function f_dec(a : unsigned(1 downto 0)) return unsigned is
    begin
        if a = 2 then return to_unsigned(1, 2); end if;
        return to_unsigned(0, 2);
    end function;
    -- own-zone weight cu (8 .. 16) of the own tone fp against the neighbour's fn
    function f_mix(fp, fn : std_logic; cu : unsigned(4 downto 0)) return unsigned is
    begin
        if fp = '1' and fn = '1' then return to_unsigned(16, 5); end if;
        if fp = '1' then return cu; end if;
        if fn = '1' then return to_unsigned(16, 5) - cu; end if;
        return to_unsigned(0, 5);
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
    signal w_dxm  : unsigned(10 downto 0);     -- the beam's pre-pixel: its distance left of the centre (h)

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
    signal s_st       : std_logic_vector(0 to 9) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] knobs: +-1-count holds, glide accumulators (Q10.6), stepped knobs, switches
    signal s_hx : unsigned(9 downto 0) := to_unsigned(C_DEF_K1, 10);
    signal s_hy : unsigned(9 downto 0) := to_unsigned(C_DEF_K2, 10);
    signal s_hp : unsigned(9 downto 0) := to_unsigned(C_DEF_K3, 10);
    signal s_hc : unsigned(9 downto 0) := to_unsigned(C_DEF_K6, 10);
    signal s_ht : unsigned(9 downto 0) := to_unsigned(C_DEF_P12, 10);
    signal s_ax : unsigned(15 downto 0) := to_unsigned(C_DEF_K1 * 64, 16);
    signal s_ay : unsigned(15 downto 0) := to_unsigned(C_DEF_K2 * 64, 16);
    signal s_ap : unsigned(15 downto 0) := to_unsigned(C_DEF_K3 * 64, 16);
    signal s_ac : unsigned(15 downto 0) := to_unsigned(C_DEF_K6 * 64, 16);
    signal s_at : unsigned(15 downto 0) := to_unsigned(C_DEF_P12 * 64, 16);
    signal s_gm : unsigned(15 downto 0) := (others => '0');
    signal s_gr, s_gh, s_gt : unsigned(9 downto 0) := (others => '0');
    signal s_hd : signed(10 downto 0) := (others => '0');
    signal s_gd : signed(16 downto 0) := (others => '0');
    signal z_gp : unsigned(2 downto 0) := (others => '0');
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_z4 : unsigned(2 downto 0) := to_unsigned(2, 3);     -- Weave: Twill
    signal s_z5 : unsigned(2 downto 0) := to_unsigned(3, 3);     -- Order: 4:4
    signal s_glide : std_logic := '1';
    signal s_point, s_thr, s_fine, s_src : std_logic := '0';

    -- [C] ROM: C_TW_TAB (2^(i/256), then a quarter sine)
    signal r_ex_a : unsigned(8 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');

    -- [C] per-field serial multiplier, slot counter and scratch
    signal f_a, f_acc, z_fa : unsigned(39 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(15 downto 0) := (others => '0');
    signal z_ph   : unsigned(4 downto 0) := (others => '0');
    signal z_slot : unsigned(4 downto 0) := (others => '0');
    signal z_run, z_pre, z_ld : std_logic := '0';
    signal z_oct  : unsigned(10 downto 0) := (others => '0');
    signal z_tneg : std_logic := '0';
    signal z_tmag : unsigned(15 downto 0) := (others => '0');
    signal z_a    : unsigned(15 downto 0) := (others => '0');
    signal z_ang  : unsigned(15 downto 0) := (others => '0');
    signal z_i8, z_i8n : std_logic := '0';
    signal z_s0, z_s1, z_dS : unsigned(15 downto 0) := (others => '0');
    signal z_mx, z_my, z_t1 : unsigned(15 downto 0) := (others => '0');
    signal z_g6 : unsigned(12 downto 0) := (others => '0');

    -- [C] per-field constants
    signal c_sh   : unsigned(2 downto 0) := to_unsigned(3, 3);     -- thread size = 3 * 2^oct; 2^-oct = M * 2^-sh
    signal c_M    : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal c_inv  : unsigned(21 downto 0) := (others => '0');      -- threads per h, Q24
    signal c_sn, c_cs : unsigned(15 downto 0) := (others => '0');  -- |sin|, cos of the turn, Q15
    signal c_cu, c_su : unsigned(21 downto 0) := (others => '0');  -- c_inv * cos, c_inv * |sin|
    signal c_UX, c_VX, c_UY, c_VY : unsigned(27 downto 0) := (others => '0');   -- DDA steps, Q4.24 modulo 16 threads
    signal c_U0, c_V0 : unsigned(27 downto 0) := (others => '0');  -- line 0, pre-pixel
    signal c_uxc  : unsigned(11 downto 0) := to_unsigned(256, 12); -- the steps in Q12 for the footprint
    signal c_vyc  : unsigned(12 downto 0) := to_unsigned(256, 13);
    signal c_vxm  : unsigned(11 downto 0) := (others => '0');
    signal c_uym  : unsigned(12 downto 0) := (others => '0');
    signal c_vxs, c_uys : signed(14 downto 0) := (others => '0');
    signal c_g    : unsigned(11 downto 0) := (others => '0');      -- hairline width, threads Q12
    signal c_lines : std_logic := '0';
    signal c_gain    : unsigned(9 downto 0) := (others => '0');       -- camera gain
    signal c_seq  : std_logic_vector(15 downto 0) := x"F0F0";

    -- [C] line and beam DDAs
    signal e_zk : std_logic := '0';
    signal l_U, l_V : unsigned(27 downto 0) := (others => '0');
    signal b_U, b_V : unsigned(27 downto 0) := (others => '0');

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl  : unsigned(7 downto 0) := (others => '0');
    signal s_avd : std_logic_vector(1 to 5) := (others => '0');    -- data_in.avid delayed
    signal s_col, s_pcol, s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal s_p, s_pfirst : unsigned(12 downto 0) := (others => '0');
    signal c_bn  : unsigned(6 downto 0) := to_unsigned(96, 7);     -- box length: 96 / 64 / 48 pixels = 96 virtual px
    signal c_brp : unsigned(7 downto 0) := to_unsigned(161, 8);
    signal c_iir : unsigned(2 downto 0) := to_unsigned(5, 3);
    type t_bx is array (0 to 255) of unsigned(15 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(15 downto 0) := (others => '0');
    signal s_bcnt : unsigned(6 downto 0) := (others => '0');
    signal s_bsum : unsigned(18 downto 0) := (others => '0');
    signal s_ev, s_fl : std_logic := '0';
    signal s_eb, s_eq : unsigned(15 downto 0) := (others => '0');
    type t_lb is array (0 to 1023) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_wd : unsigned(15 downto 0) := (others => '0');
    signal lb_dd : signed(13 downto 0) := (others => '0');
    signal lb_wa : unsigned(9 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal h_cur, h_prev, s8_q : unsigned(15 downto 0) := (others => '0');
    signal h_dy, s8_dy : signed(13 downto 0) := (others => '0');
    signal h_ph : std_logic := '0';
    signal s9_D : unsigned(15 downto 0) := (others => '0');
    signal s9_Dy : signed(13 downto 0) := (others => '0');

    -- [E] cloth: coordinates and footprint (stages 10 .. 14)
    signal s10_U, s10_V, s10_Dp, s11_U, s11_V, s12_U, s12_V, s13_U, s13_V : unsigned(15 downto 0) := (others => '0');
    signal s14_fu, s14_fv, s15_fu, s15_fv : unsigned(11 downto 0) := (others => '0');
    signal s10_Dx : signed(15 downto 0) := (others => '0');
    signal s10_Dy : signed(13 downto 0) := (others => '0');
    signal s11_gux, s11_guy, s11_gvx, s11_gvy : signed(16 downto 0) := (others => '0');
    signal s12_au, s12_bu, s12_av, s12_bv : unsigned(12 downto 0) := (others => '0');
    signal s13_wu, s13_wv : unsigned(12 downto 0) := (others => '0');
    signal s14_weu, s14_wev, s14_wm : unsigned(12 downto 0) := (others => '0');
    signal s14_geu, s14_gev, s15_geu, s15_gev : unsigned(11 downto 0) := (others => '0');
    signal s15_weu, s15_wev, s16_weu, s16_wev, s17_weu, s17_wev : unsigned(12 downto 0) := (others => '0');
    -- zones (15, 16), coverage dividers (17 .. 22)
    signal s15_igu, s15_igv, s15_nlu, s15_nlv : std_logic := '0';
    signal s16_du, s16_dv : unsigned(11 downto 0) := (others => '0');
    signal s16_zpu, s16_zpv : std_logic := '0';
    signal s16_nou, s16_nov : unsigned(1 downto 0) := (others => '0');
    signal s17_tu, s17_tv : unsigned(12 downto 0) := (others => '0');
    type t_dv14 is array (0 to 3) of unsigned(13 downto 0);
    type t_dv13 is array (0 to 3) of unsigned(12 downto 0);
    type t_dv3 is array (0 to 3) of unsigned(2 downto 0);
    signal du_r, dv_r : t_dv14 := (others => (others => '0'));
    signal du_w, dv_w : t_dv13 := (others => (others => '0'));
    signal du_q, dv_q : t_dv3 := (others => (others => '0'));
    signal du_s, dv_s : std_logic_vector(0 to 3) := (others => '0');
    signal s22_cu, s22_cv, s23_cv, s24_cv : unsigned(4 downto 0) := (others => '0');
    -- fade to mid-grey (threads under about a pixel)
    type t_tf is array (15 to 27) of unsigned(3 downto 0);
    signal s_tf : t_tf := (others => (others => '0'));

    -- [E] the draft: three rows of the weave matrix (one ROM each), three columns picked from each
    signal wr_a0, wr_a1, wr_a2 : unsigned(7 downto 0) := (others => '0');
    signal wr_q0, wr_q1, wr_q2, wr_f0, wr_f1, wr_f2 : std_logic_vector(7 downto 0) := (others => '0');
    type t_ca is array (0 to 2) of unsigned(2 downto 0);
    signal s11_ca, s12_ca, s13_ca : t_ca := (others => (others => '0'));
    signal s11_cw, s11_cf, s12_cw, s12_cf, s13_cw, s13_cf, s14_cw, s14_cf : std_logic_vector(2 downto 0) := (others => '0');
    signal s14_W9, s15_W9, s15_C9, s16_W9, s16_C9 : std_logic_vector(8 downto 0) := (others => '0');
    -- the four corners: tone, warp-on-top, the tones left / above / diagonal, the two gap flags
    type t_cn is array (0 to 3) of std_logic_vector(6 downto 0);
    signal s17_cn : t_cn := (others => (others => '0'));
    signal s18_F, s19_F, s20_F, s21_F, s22_F : std_logic_vector(3 downto 0) := (others => '0');

    -- [E] blend (23 .. 31)
    signal s23_rt, s23_rb, s24_rb, s25_rb : unsigned(4 downto 0) := (others => '0');
    signal s24_df : signed(5 downto 0) := (others => '0');
    signal s25_pr : signed(11 downto 0) := (others => '0');
    signal s26_o, s27_o, s28_o, s29_o : unsigned(8 downto 0) := (others => '0');
    signal s27_e : signed(9 downto 0) := (others => '0');
    signal s28_m : signed(14 downto 0) := (others => '0');
    signal s30_a : unsigned(18 downto 0) := (others => '0');
    signal s30_b : unsigned(13 downto 0) := (others => '0');
    signal s31_y : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- [F] output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_yd : t_u10(1 to 3) := (others => to_unsigned(64, 10));
    signal s34_ya : unsigned(11 downto 0) := (others => '0');
    signal s34_yb : unsigned(9 downto 0) := (others => '0');
    signal s35_y, s36_y, s37_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_TW_TAB(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            wr_q0  <= C_TW_WV(to_integer(wr_a0));
            wr_q1  <= C_TW_WV(to_integer(wr_a1));
            wr_q2  <= C_TW_WV(to_integer(wr_a2));
        end if;
    end process p_rom;

    -- box delay: read (box length) samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_avd(2) = '1' then bx_ram(to_integer(bx_wp)) <= resize(s_p, 16); end if;
        end if;
    end process p_bxram;

    -- camera-field line store (half horizontal resolution): read 3 pixels ahead of the write
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_pcol(10 downto 1)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    -- [A] the beam's pre-pixel by video mode
    w_dxm <= to_unsigned(1444, 11) when s_xs = 4 else to_unsigned(1923, 11) when s_xs = 3 else to_unsigned(1922, 11);

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS + [C] the per-field
    -- sequencer (one small operation per zone clock; the zone is quasi-static
    -- but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k   : unsigned(9 downto 0);
        variable v_e   : signed(11 downto 0);
        variable v_17  : signed(16 downto 0);
        variable v_st  : signed(16 downto 0);
        variable v_i9  : unsigned(8 downto 0);
        variable v_g4  : unsigned(12 downto 0);
        variable v_c14 : unsigned(13 downto 0);
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
                s_point <= registers_in(6)(1);
                s_thr   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is                  -- Osc step, box length (96 virtual px), its read offset
                    when 4      => s_oxs <= to_unsigned(204, 9); c_bn <= to_unsigned(48, 7); c_brp <= to_unsigned(209, 8);
                    when 3      => s_oxs <= to_unsigned(153, 9); c_bn <= to_unsigned(64, 7); c_brp <= to_unsigned(193, 8);
                    when others => s_oxs <= to_unsigned(102, 9); c_bn <= to_unsigned(96, 7); c_brp <= to_unsigned(161, 8);
                end case;
                -- the vertical IIR's shift: about 32 .. 48 virtual lines in every mode
                if s_ys >= 960 then c_iir <= to_unsigned(3, 3);
                elsif s_ys >= 480 then c_iir <= to_unsigned(4, 3);
                else c_iir <= to_unsigned(5, 3); end if;
                -- stepped knobs: a new step is taken only 8 counts inside it (and at either end of the travel)
                v_k := unsigned(registers_in(3));
                if v_k(9 downto 7) /= s_z4 and ((v_k(6 downto 3) /= "0000" and v_k(6 downto 3) /= "1111")
                        or v_k(9 downto 3) = "0000000" or v_k(9 downto 3) = "1111111") then
                    s_z4 <= v_k(9 downto 7);
                end if;
                v_k := unsigned(registers_in(4));
                if v_k(9 downto 7) /= s_z5 and ((v_k(6 downto 3) /= "0000" and v_k(6 downto 3) /= "1111")
                        or v_k(9 downto 3) = "0000000" or v_k(9 downto 3) = "1111111") then
                    s_z5 <= v_k(9 downto 7);
                end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- [B] shared hold + glide, six clocks per channel (X, Y, Period, Camera, Turn):
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
                            when 0      => s_gm <= s_ax; s_gh <= s_hx; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_ay; s_gh <= s_hy; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_ap; s_gh <= s_hp; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_ac; s_gh <= s_hc; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_at; s_gh <= s_ht; s_gr <= unsigned(registers_in(7));
                        end case;
                    when 1 =>
                        s_hd <= signed(resize(s_gr, 11)) - signed(resize(s_gh, 11));
                    when 2 =>
                        if s_hd > 1 or s_hd < -1 then s_gt <= s_gr; else s_gt <= s_gh; end if;
                    when 3 =>
                        s_gd <= signed(resize(s_gt & "000000", 17)) - signed(resize(s_gm, 17));
                        case to_integer(z_gc) is
                            when 0      => s_hx <= s_gt;
                            when 1      => s_hy <= s_gt;
                            when 2      => s_hp <= s_gt;
                            when 3      => s_hc <= s_gt;
                            when others => s_ht <= s_gt;
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
                            when 0      => s_ax <= unsigned(v_st(15 downto 0));
                            when 1      => s_ay <= unsigned(v_st(15 downto 0));
                            when 2      => s_ap <= unsigned(v_st(15 downto 0));
                            when 3      => s_ac <= unsigned(v_st(15 downto 0));
                            when others => s_at <= unsigned(v_st(15 downto 0));
                        end case;
                        z_gp <= "000";
                        if z_gc = 4 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- [C] before the slots (zc 40 .. 45): the turn's sign and magnitude past a +-6-count dead centre,
            -- the pre-pixel's distance from the pivot (h, 4 fraction bits), the colour order
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 40 =>
                        v_17 := signed(resize(s_at, 17)) - 32768;
                        z_tneg <= v_17(16);
                        v_st := -v_17;
                        if v_17(16) = '1' then z_tmag <= unsigned(v_st(15 downto 0)); else z_tmag <= unsigned(v_17(15 downto 0)); end if;
                    when 41 =>
                        if z_tmag > 384 then z_a <= z_tmag - 384; else z_a <= (others => '0'); end if;
                    when 42 =>
                        v_17 := shift_right(signed(resize(s_ax, 17)) - 32768, 1) + signed(resize(w_dxm & "0000", 17));
                        z_mx <= unsigned(v_17(15 downto 0));
                    when 43 =>
                        v_17 := shift_right(signed(resize(s_ay, 17)) - 32768, 1) + to_signed(17280, 17);
                        z_my <= unsigned(v_17(15 downto 0));
                    when 44 =>
                        c_seq <= C_TW_SEQ(to_integer(s_z5));
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
                    if z_slot = 18 then z_run <= '0'; else z_slot <= z_slot + 1; end if;
                else
                    z_ph <= z_ph + 1;
                end if;
            end if;
            if z_run = '1' and z_ph = 0 then z_pre <= '1'; else z_pre <= '0'; end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_slot) is
                    when 0  => z_fa <= resize(s_ap, 40);  z_fb <= to_unsigned(1131, 16);             -- octaves above 3 px, Q8
                    when 1  => z_fa <= resize(r_exf, 40); z_fb <= to_unsigned(21845, 16);            -- threads per h << sh
                               c_M <= r_exf;
                    when 2  => z_fa <= resize(z_a, 40);   z_fb <= to_unsigned(33222, 16);            -- the angle, 32768 = 90 deg
                    when 4 | 6 => z_fa <= resize(z_dS, 40); z_fb <= resize(z_ang(6 downto 0), 16);   -- sine interpolation
                    when 7  => z_fa <= resize(c_inv, 40); z_fb <= c_cs;
                    when 8  => z_fa <= resize(c_inv, 40); z_fb <= c_sn;
                    when 9  => z_fa <= resize(c_cu, 40);  z_fb <= resize(s_xs, 16);                  -- du / pixel
                    when 10 => z_fa <= resize(c_su, 40);  z_fb <= resize(s_xs, 16);                  -- dv / pixel
                    when 11 => z_fa <= resize(c_su, 40);  z_fb <= resize(s_ys, 16);                  -- du / line
                    when 12 => z_fa <= resize(c_cu, 40);  z_fb <= resize(s_ys, 16);                  -- dv / line
                    when 13 => z_fa <= resize(c_cu, 40);  z_fb <= z_mx;                              -- line 0, pre-pixel
                    when 14 => z_fa <= resize(c_su, 40);  z_fb <= z_my;
                    when 15 => z_fa <= resize(c_su, 40);  z_fb <= z_mx;
                    when 16 => z_fa <= resize(c_cu, 40);  z_fb <= z_my;
                    when 17 => z_fa <= resize(s_ac, 40);  z_fb <= c_M;                               -- camera gain
                    when 18 =>
                        z_fa <= resize(z_t1, 40);
                        case to_integer(s_xs) is
                            when 4      => z_fb <= to_unsigned(932, 16);
                            when 3      => z_fb <= to_unsigned(699, 16);
                            when others => z_fb <= to_unsigned(466, 16);
                        end case;
                    when others => null;                                                             -- 3, 5: table reads
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(38 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if z_run = '1' then
                -- slots 3 and 5: S(idx), S(idx + 1) of the quarter sine (S(256) = 32768 is not in the table)
                if z_slot = 3 or z_slot = 5 then
                    v_i9 := resize(z_ang(14 downto 7), 9) + 1;
                    case to_integer(z_ph) is
                        when 1 => r_ex_a <= '1' & z_ang(14 downto 7); z_i8 <= z_ang(15);
                        when 2 => r_ex_a <= '1' & v_i9(7 downto 0);   z_i8n <= z_ang(15) or v_i9(8);
                        when 4 => if z_i8 = '1' then z_s0 <= to_unsigned(32768, 16); else z_s0 <= r_exf; end if;
                        when 5 => if z_i8n = '1' then z_s1 <= to_unsigned(32768, 16); else z_s1 <= r_exf; end if;
                        when 6 => z_dS <= z_s1 - z_s0;
                        when others => null;
                    end case;
                end if;
                if z_ph = 20 then
                    case to_integer(z_slot) is
                        when 0  => z_oct <= f_acc(26 downto 16);
                        when 1  =>
                            case to_integer(c_sh) is
                                when 0      => c_inv <= f_acc(29 downto 8);
                                when 1      => c_inv <= f_acc(30 downto 9);
                                when 2      => c_inv <= resize(f_acc(30 downto 10), 22);
                                when 3      => c_inv <= resize(f_acc(30 downto 11), 22);
                                when 4      => c_inv <= resize(f_acc(30 downto 12), 22);
                                when others => c_inv <= resize(f_acc(30 downto 13), 22);
                            end case;
                        when 2  =>
                            if f_acc(31 downto 30) /= "00" then z_ang <= to_unsigned(32768, 16);
                            else z_ang <= f_acc(30 downto 15); end if;
                        when 4  => c_sn <= z_s0 + f_acc(22 downto 7);
                        when 6  => c_cs <= z_s0 + f_acc(22 downto 7);
                        when 7  => c_cu <= f_acc(36 downto 15);
                        when 8  => c_su <= f_acc(36 downto 15);
                        when 9  => c_UX <= resize(f_acc(23 downto 0), 28); c_uxc <= f_acc(23 downto 12);
                        when 10 =>
                            c_VX <= f_addsub(to_unsigned(0, 28), resize(f_acc(23 downto 0), 28), not z_tneg);
                            c_vxm <= f_acc(23 downto 12);
                        when 11 =>
                            c_UY <= f_addsub(to_unsigned(0, 28), resize(f_acc(32 downto 7), 28), z_tneg);
                            if f_acc(32) = '1' then c_uym <= (others => '1'); else c_uym <= f_acc(31 downto 19); end if;
                        when 12 =>
                            c_VY <= resize(f_acc(32 downto 7), 28);
                            if f_acc(32) = '1' then c_vyc <= (others => '1'); else c_vyc <= f_acc(31 downto 19); end if;
                        when 13 => c_U0 <= f_addsub(to_unsigned(0, 28), f_acc(31 downto 4), '1');
                        when 14 => c_U0 <= f_addsub(c_U0, f_acc(31 downto 4), not z_tneg);
                        when 15 => c_V0 <= f_addsub(to_unsigned(0, 28), f_acc(31 downto 4), z_tneg);
                        when 16 => c_V0 <= f_addsub(c_V0, f_acc(31 downto 4), '1');
                        when 17 => z_t1 <= f_acc(31 downto 16);
                        when 18 => c_gain <= f_acc(25 downto 16);
                        when others => null;
                    end case;
                end if;
                if z_ph = 21 then
                    case to_integer(z_slot) is
                        when 0  =>                         -- 2^-oct = table(-oct mod 256) * 2^-sh
                            v_e := -signed(resize(z_oct, 12));
                            r_ex_a <= '0' & unsigned(v_e(7 downto 0));
                            v_e := -shift_right(v_e, 8);                       -- 0 .. 5
                            c_sh <= unsigned(v_e(2 downto 0));
                        when 4  => z_ang <= to_unsigned(32768, 16) - z_ang;                -- the cosine's angle
                        when 10 =>
                            if z_tneg = '1' then c_vxs <= signed(resize(c_vxm, 15)); else c_vxs <= -signed(resize(c_vxm, 15)); end if;
                        when 11 =>
                            if z_tneg = '1' then c_uys <= -signed(resize(c_uym, 15)); else c_uys <= signed(resize(c_uym, 15)); end if;
                        when 17 =>                         -- 3 virtual px in threads, Q12
                            z_g6 <= resize(c_inv(21 downto 10), 13) + resize(c_inv(21 downto 11), 13);
                        when 18 =>                         -- hairline width: Fine 1/4 thread (3 px at least), Threads
                            v_g4 := resize(c_inv(21 downto 10), 13);       -- 1/6 thread (2 px at least), half a thread at most
                            if s_fine = '1' then
                                if z_g6 > 2048 then c_g <= to_unsigned(2048, 12);
                                elsif z_g6 > 1024 then c_g <= z_g6(11 downto 0);
                                else c_g <= to_unsigned(1024, 12); end if;
                            elsif s_thr = '1' then
                                if v_g4 > 2048 then c_g <= to_unsigned(2048, 12);
                                elsif v_g4 > 683 then c_g <= v_g4(11 downto 0);
                                else c_g <= to_unsigned(683, 12); end if;
                            else
                                c_g <= (others => '0');
                            end if;
                            c_lines <= s_fine or s_thr;
                        when others => null;
                    end case;
                end if;
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
    -- [C] ENGINE, line and beam DDAs.  l_U / l_V hold this line's cloth
    -- coordinate at the pre-pixel: set to line 0 in the zone, stepped once per
    -- ACTIVE line after the beam has taken them.  The beam loads them so that
    -- pixel 0 is at stage 9 and steps once per pixel.  Q4.24, modulo 16 threads.
    ---------------------------------------------------------------------------
    p_dda : process(clk)
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;     -- after the field constants

            if e_zk = '1' then
                l_U <= c_U0; l_V <= c_V0;
            elsif s_st(8) = '1' then
                l_U <= l_U + c_UY; l_V <= l_V + c_VY;
            end if;

            if s_st(6) = '1' then
                b_U <= l_U; b_V <= l_V;
            else
                b_U <= b_U + c_UX; b_V <= b_V + c_VX;
            end if;
        end if;
    end process p_dda;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) x camera gain -> box over 96
    -- virtual px, edge-replicated and reloaded at every line start -> vertical
    -- one-pole IIR in the line store (half horizontal resolution; the first
    -- line of a field loads it).  The field is the cloth's displacement in
    -- threads << sh (Q8.8); the pixel path taps the store's write data (the
    -- camera is never shown, so it need not be aligned to the pixel).
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l   : unsigned(7 downto 0);
        variable v_s   : unsigned(7 downto 0);
        variable v_pr  : unsigned(17 downto 0);
        variable v_tap : unsigned(12 downto 0);
        variable v_nf  : unsigned(18 downto 0);
        variable v_d   : signed(16 downto 0);
        variable v_dd  : signed(16 downto 0);
        variable v_n   : signed(16 downto 0);
        variable v_m   : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting
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

            -- 1: the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_avd <= data_in.avid & s_avd(1 to 4);
            if s_avd(1) = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;

            -- 2: x camera gain (the one multiplier of the source path); the lattice goes in at twice the level
            v_pr := s_sl * c_gain;
            if s_src = '1' then s_p <= v_pr(17 downto 5); else s_p <= '0' & v_pr(17 downto 6); end if;
            s_pcol <= s_col;

            -- 3: box: sum of the last N samples; the samples before the line start count as the first one
            if s_avd(2) = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= c_brp; end if;
            if s_bcnt /= c_bn then v_tap := s_pfirst; else v_tap := bx_q(12 downto 0); end if;
            case to_integer(s_xs) is
                when 4      => v_nf := shift_left(resize(s_p, 19), 5) + shift_left(resize(s_p, 19), 4);   -- 48
                when 3      => v_nf := shift_left(resize(s_p, 19), 6);                                    -- 64
                when others => v_nf := shift_left(resize(s_p, 19), 6) + shift_left(resize(s_p, 19), 5);   -- 96
            end case;
            if s_avd(2) = '1' and s_avd(3) = '0' then
                s_pfirst <= s_p;
                s_bsum <= v_nf;
                s_bcnt <= to_unsigned(1, 7);
            elsif s_avd(2) = '1' then
                s_bsum <= s_bsum + s_p - v_tap;
                if s_bcnt /= c_bn then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bcol <= s_pcol;

            -- 4: fabric copies; 5: vertical IIR on the odd columns, q += (box - q) >> k (the first line: q = box)
            s_eb <= s_bsum(18 downto 3); s_eq <= lb_q1;
            s_ev <= s_avd(3) and s_bcol(0); s_ecol <= s_bcol;
            if s_zone = '1' then s_fl <= '1';
            elsif s_avd(5) = '1' and s_avd(4) = '0' then s_fl <= '0'; end if;
            v_d := signed(resize(s_eb, 17)) - signed(resize(s_eq, 17));
            case to_integer(c_iir) is
                when 3      => v_dd := shift_right(v_d, 3);
                when 4      => v_dd := shift_right(v_d, 4);
                when others => v_dd := shift_right(v_d, 5);
            end case;
            v_n := signed(resize(s_eq, 17)) + v_dd;
            if s_fl = '1' then
                lb_wd <= s_eb; lb_dd <= (others => '0');
            else
                lb_wd <= unsigned(v_n(15 downto 0)); lb_dd <= v_dd(13 downto 0);
            end if;
            lb_we <= s_ev; lb_wa <= s_ecol(10 downto 1);

            -- the pixel path's tap: the last two columns written; stage 8 interpolates between them on the
            -- clock after a write, stage 9 scales to threads Q4.12
            if lb_we = '1' then
                h_cur <= lb_wd; h_prev <= h_cur; h_dy <= lb_dd; h_ph <= '1';
            else
                h_ph <= '0';
            end if;
            v_m := resize(h_cur, 17) + resize(h_prev, 17);
            if h_ph = '1' then s8_q <= v_m(16 downto 1); else s8_q <= h_cur; end if;
            s8_dy <= h_dy;
            s9_D  <= f_shf(s8_q, c_sh);
            s9_Dy <= f_shd(s8_dy, c_sh);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] CLOTH, per axis (stages 10 .. 22):
    --   10  U = beam + D, V = beam - D (the camera pushes along the cloth's diagonal), D's pixel difference
    --   11  the four partial derivatives (per-field step + the camera field's own difference)
    --   12  magnitudes        13  footprint w = |d/dx| + |d/dy| (threads per pixel, Q12, held under 2)
    --   14  weff = w, widened past 1/2 thread per pixel; ge = the hairline zone, at least one footprint wide
    --   15  which zone the pixel is in and which breakpoint is nearer
    --   16  d = the distance to that breakpoint; the neighbouring zone across it
    --   17  t = d + weff / 32           18  saturation test, 19 .. 21 three quotient bits
    --   22  cov = the own zone's weight, 8 .. 16 sixteenths
    ---------------------------------------------------------------------------
    p_axis : process(clk)
        variable v_w   : unsigned(13 downto 0);
        variable v_w5  : unsigned(15 downto 0);
        variable v_ge  : unsigned(12 downto 0);
        variable v_a, v_b, v_d : unsigned(12 downto 0);
        variable v_nl  : std_logic;
        variable v_t   : unsigned(14 downto 0);
    begin
        if rising_edge(clk) then
            -- 10
            s10_U <= b_U(27 downto 12) + s9_D;
            s10_V <= b_V(27 downto 12) - s9_D;
            s10_Dx <= signed(s9_D - s10_Dp); s10_Dp <= s9_D;
            s10_Dy <= s9_Dy;
            -- 11
            s11_gux <= signed(resize(c_uxc, 17)) + resize(s10_Dx, 17);
            s11_guy <= resize(c_uys, 17) + resize(s10_Dy, 17);
            s11_gvx <= resize(c_vxs, 17) - resize(s10_Dx, 17);
            s11_gvy <= signed(resize(c_vyc, 17)) - resize(s10_Dy, 17);
            s11_U <= s10_U; s11_V <= s10_V;
            -- 12
            s12_au <= f_abs13(s11_gux); s12_bu <= f_abs13(s11_guy);
            s12_av <= f_abs13(s11_gvx); s12_bv <= f_abs13(s11_gvy);
            s12_U <= s11_U; s12_V <= s11_V;
            -- 13
            v_w := resize(s12_au, 14) + resize(s12_bu, 14);
            if v_w(13) = '1' then s13_wu <= (others => '1'); else s13_wu <= v_w(12 downto 0); end if;
            v_w := resize(s12_av, 14) + resize(s12_bv, 14);
            if v_w(13) = '1' then s13_wv <= (others => '1'); else s13_wv <= v_w(12 downto 0); end if;
            s13_U <= s12_U; s13_V <= s12_V;
            -- 14
            if s13_wu(12 downto 11) /= "00" then
                v_w5 := shift_left(resize(s13_wu, 16), 2) + resize(s13_wu, 16) - 8192;
                if v_w5(15 downto 13) /= "000" then s14_weu <= (others => '1'); else s14_weu <= v_w5(12 downto 0); end if;
            else
                s14_weu <= s13_wu;
            end if;
            if s13_wv(12 downto 11) /= "00" then
                v_w5 := shift_left(resize(s13_wv, 16), 2) + resize(s13_wv, 16) - 8192;
                if v_w5(15 downto 13) /= "000" then s14_wev <= (others => '1'); else s14_wev <= v_w5(12 downto 0); end if;
            else
                s14_wev <= s13_wv;
            end if;
            if c_lines = '0' then s14_geu <= (others => '0');
            elsif s13_wu > 2048 then s14_geu <= to_unsigned(2048, 12);
            elsif s13_wu > resize(c_g, 13) then s14_geu <= s13_wu(11 downto 0);
            else s14_geu <= c_g; end if;
            if c_lines = '0' then s14_gev <= (others => '0');
            elsif s13_wv > 2048 then s14_gev <= to_unsigned(2048, 12);
            elsif s13_wv > resize(c_g, 13) then s14_gev <= s13_wv(11 downto 0);
            else s14_gev <= c_g; end if;
            if s13_wu > s13_wv then s14_wm <= s13_wu; else s14_wm <= s13_wv; end if;
            s14_fu <= s13_U(11 downto 0); s14_fv <= s13_V(11 downto 0);
            -- 15: in the hairline zone?  nearer its low breakpoint?
            if s14_fu < s14_geu then
                s15_igu <= '1';
                if (s14_fu & '0') < resize(s14_geu, 13) then s15_nlu <= '1'; else s15_nlu <= '0'; end if;
            else
                s15_igu <= '0';
                if resize(s14_fu & '0', 14) < to_unsigned(4096, 14) + resize(s14_geu, 14) then s15_nlu <= '1'; else s15_nlu <= '0'; end if;
            end if;
            if s14_fv < s14_gev then
                s15_igv <= '1';
                if (s14_fv & '0') < resize(s14_gev, 13) then s15_nlv <= '1'; else s15_nlv <= '0'; end if;
            else
                s15_igv <= '0';
                if resize(s14_fv & '0', 14) < to_unsigned(4096, 14) + resize(s14_gev, 14) then s15_nlv <= '1'; else s15_nlv <= '0'; end if;
            end if;
            s15_fu <= s14_fu; s15_fv <= s14_fv; s15_geu <= s14_geu; s15_gev <= s14_gev;
            s15_weu <= s14_weu; s15_wev <= s14_wev;
            -- fade: 0 below half a thread per pixel, 8 (flat grey) from a whole thread per pixel
            if s14_wm(12) = '1' then s_tf(15) <= to_unsigned(8, 4);
            elsif s14_wm(11) = '1' then s_tf(15) <= '0' & s14_wm(10 downto 8);
            else s_tf(15) <= (others => '0'); end if;
            for i in 16 to 27 loop s_tf(i) <= s_tf(i - 1); end loop;
            -- 16: one subtraction with muxed operands
            if s15_igu = '1' then
                if s15_nlu = '1' then v_a := resize(s15_fu, 13); v_b := (others => '0'); s16_nou <= "00";
                else v_a := resize(s15_geu, 13); v_b := resize(s15_fu, 13); s16_nou <= "01"; end if;
            else
                if s15_nlu = '1' then v_a := resize(s15_fu, 13); v_b := resize(s15_geu, 13);
                    if c_lines = '1' then s16_nou <= "01"; else s16_nou <= "00"; end if;
                else v_a := to_unsigned(4096, 13); v_b := resize(s15_fu, 13); s16_nou <= "10"; end if;
            end if;
            v_d := v_a - v_b; s16_du <= v_d(11 downto 0); s16_zpu <= s15_igu;
            if s15_igv = '1' then
                if s15_nlv = '1' then v_a := resize(s15_fv, 13); v_b := (others => '0'); s16_nov <= "00";
                else v_a := resize(s15_gev, 13); v_b := resize(s15_fv, 13); s16_nov <= "01"; end if;
            else
                if s15_nlv = '1' then v_a := resize(s15_fv, 13); v_b := resize(s15_gev, 13);
                    if c_lines = '1' then s16_nov <= "01"; else s16_nov <= "00"; end if;
                else v_a := to_unsigned(4096, 13); v_b := resize(s15_fv, 13); s16_nov <= "10"; end if;
            end if;
            v_d := v_a - v_b; s16_dv <= v_d(11 downto 0); s16_zpv <= s15_igv;
            s16_weu <= s15_weu; s16_wev <= s15_wev;
            -- 17
            s17_tu <= resize(s16_du, 13) + resize(s16_weu(12 downto 5), 13);
            s17_tv <= resize(s16_dv, 13) + resize(s16_wev(12 downto 5), 13);
            s17_weu <= s16_weu; s17_wev <= s16_wev;
            -- 18: 2 t >= weff: the own zone covers the whole pixel
            du_r(0) <= s17_tu & '0'; du_w(0) <= s17_weu; du_q(0) <= "000";
            if (s17_tu & '0') >= resize(s17_weu, 14) then du_s(0) <= '1'; else du_s(0) <= '0'; end if;
            dv_r(0) <= s17_tv & '0'; dv_w(0) <= s17_wev; dv_q(0) <= "000";
            if (s17_tv & '0') >= resize(s17_wev, 14) then dv_s(0) <= '1'; else dv_s(0) <= '0'; end if;
            -- 19 .. 21
            for i in 1 to 3 loop
                v_t := du_r(i - 1) & '0';
                if v_t >= resize(du_w(i - 1), 15) then
                    du_r(i) <= resize(v_t - resize(du_w(i - 1), 15), 14);
                    du_q(i) <= du_q(i - 1)(1 downto 0) & '1';
                else
                    du_r(i) <= v_t(13 downto 0);
                    du_q(i) <= du_q(i - 1)(1 downto 0) & '0';
                end if;
                du_w(i) <= du_w(i - 1); du_s(i) <= du_s(i - 1);
                v_t := dv_r(i - 1) & '0';
                if v_t >= resize(dv_w(i - 1), 15) then
                    dv_r(i) <= resize(v_t - resize(dv_w(i - 1), 15), 14);
                    dv_q(i) <= dv_q(i - 1)(1 downto 0) & '1';
                else
                    dv_r(i) <= v_t(13 downto 0);
                    dv_q(i) <= dv_q(i - 1)(1 downto 0) & '0';
                end if;
                dv_w(i) <= dv_w(i - 1); dv_s(i) <= dv_s(i - 1);
            end loop;
            -- 22
            if du_s(3) = '1' then s22_cu <= to_unsigned(16, 5); else s22_cu <= "01" & du_q(3); end if;
            if dv_s(3) = '1' then s22_cv <= to_unsigned(16, 5); else s22_cv <= "01" & dv_q(3); end if;
        end if;
    end process p_axis;

    ---------------------------------------------------------------------------
    -- [E] CLOTH, the draft (stages 11 .. 22): the 3 x 3 cells around the pixel
    --   11  row addresses of the weave ROM, the three draft columns, the six thread colours
    --   12  ROM        13  fabric copies        14  W9 = warp on top, per cell (bit b * 3 + a)
    --   15  C9 = the cell's tone: the warp thread's colour where it is on top, else the weft's
    --   17  per blend corner (own / u-neighbour x own / v-neighbour): the cell's tone and the tones to
    --       its left, above and diagonal      18  the corner's tone in its zone:
    --         Flat Bold      the cell's tone
    --         Threads Bold   the cell's tone, inverted in the top thread's own hairline zone
    --         Fine           white where a hairline zone borders a cell of the other tone (the outline);
    --                        with Threads also the hairlines of the light floats
    ---------------------------------------------------------------------------
    p_cell : process(clk)
        variable v_iu, v_iv : unsigned(3 downto 0);
        variable v_x  : unsigned(3 downto 0);
        variable v_a, v_b : unsigned(1 downto 0);
        variable v_zu, v_zv : std_logic;
        variable v_cn : std_logic_vector(6 downto 0);
        variable v_tl, v_ol : std_logic;
    begin
        if rising_edge(clk) then
            -- 11
            v_iu := s10_U(15 downto 12); v_iv := s10_V(15 downto 12);
            for k in 0 to 2 loop
                v_x := v_iu + to_unsigned((k + 15) mod 16, 4);            -- i - 1, i, i + 1
                s11_ca(k) <= f_refl(v_x, s_point);
                s11_cw(k) <= c_seq(to_integer(v_x));
                v_x := v_iv + to_unsigned((k + 15) mod 16, 4);
                s11_cf(k) <= c_seq(to_integer(v_x));
                case k is
                    when 0      => wr_a0 <= "00" & s_z4 & f_refl(v_x, s_point);
                    when 1      => wr_a1 <= "00" & s_z4 & f_refl(v_x, s_point);
                    when others => wr_a2 <= "00" & s_z4 & f_refl(v_x, s_point);
                end case;
            end loop;
            -- 12 (ROM), 13
            s12_ca <= s11_ca; s12_cw <= s11_cw; s12_cf <= s11_cf;
            wr_f0 <= wr_q0; wr_f1 <= wr_q1; wr_f2 <= wr_q2;
            s13_ca <= s12_ca; s13_cw <= s12_cw; s13_cf <= s12_cf;
            -- 14
            for a in 0 to 2 loop
                s14_W9(a)     <= wr_f0(to_integer(s13_ca(a)));
                s14_W9(3 + a) <= wr_f1(to_integer(s13_ca(a)));
                s14_W9(6 + a) <= wr_f2(to_integer(s13_ca(a)));
            end loop;
            s14_cw <= s13_cw; s14_cf <= s13_cf;
            -- 15
            for b in 0 to 2 loop
                for a in 0 to 2 loop
                    if s14_W9(b * 3 + a) = '1' then s15_C9(b * 3 + a) <= s14_cw(a); else s15_C9(b * 3 + a) <= s14_cf(b); end if;
                end loop;
            end loop;
            s15_W9 <= s14_W9;
            -- 16
            s16_C9 <= s15_C9; s16_W9 <= s15_W9;
            -- 17: corners 0 = own / own, 1 = u-neighbour / own, 2 = own / v-neighbour, 3 = both neighbours
            for n in 0 to 3 loop
                if n = 1 or n = 3 then v_a := s16_nou; v_zu := not s16_zpu; else v_a := "01"; v_zu := s16_zpu; end if;
                if n = 2 or n = 3 then v_b := s16_nov; v_zv := not s16_zpv; else v_b := "01"; v_zv := s16_zpv; end if;
                s17_cn(n) <= v_zv & v_zu
                           & f_pick(s16_C9, f_dec(v_a), f_dec(v_b))      -- diagonal
                           & f_pick(s16_C9, v_a, f_dec(v_b))             -- above
                           & f_pick(s16_C9, f_dec(v_a), v_b)             -- left
                           & f_pick(s16_W9, v_a, v_b)
                           & f_pick(s16_C9, v_a, v_b);
            end loop;
            -- 18
            for n in 0 to 3 loop
                v_cn := s17_cn(n);
                if v_cn(1) = '1' then v_tl := v_cn(5); else v_tl := v_cn(6); end if;
                v_ol := (v_cn(5) and (v_cn(0) xor v_cn(2))) or (v_cn(6) and (v_cn(0) xor v_cn(3)))
                        or (v_cn(5) and v_cn(6) and (v_cn(0) xor v_cn(4)));
                if c_lines = '0' then s18_F(n) <= v_cn(0);
                elsif s_fine = '0' then s18_F(n) <= v_cn(0) xor v_tl;
                else s18_F(n) <= v_ol or (s_thr and v_tl and v_cn(0)); end if;
            end loop;
            s19_F <= s18_F; s20_F <= s19_F; s21_F <= s20_F; s22_F <= s21_F;
        end if;
    end process p_cell;

    ---------------------------------------------------------------------------
    -- [E] BLEND (stages 23 .. 31): bilinear over the four corners (one small
    -- multiplier with a stage to itself), fade to mid-grey (another), luma
    --   o = 16 rb + cv (rt - rb)   0 .. 256      y = 64 + o * 876 / 256   (white 940)
    ---------------------------------------------------------------------------
    p_blend : process(clk)
        variable v_o : signed(12 downto 0);
        variable v_f : signed(10 downto 0);
        variable v_y : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            -- 23
            s23_rt <= f_mix(s22_F(0), s22_F(1), s22_cu);
            s23_rb <= f_mix(s22_F(2), s22_F(3), s22_cu);
            s23_cv <= s22_cv;
            -- 24
            s24_df <= signed(resize(s23_rt, 6)) - signed(resize(s23_rb, 6));
            s24_rb <= s23_rb; s24_cv <= s23_cv;
            -- 25
            s25_pr <= s24_df * signed(resize(s24_cv, 6));
            s25_rb <= s24_rb;
            -- 26
            v_o := signed(resize(s25_rb & "0000", 13)) + resize(s25_pr, 13);
            s26_o <= unsigned(v_o(8 downto 0));
            -- 27
            s27_e <= to_signed(128, 10) - signed(resize(s26_o, 10));
            s27_o <= s26_o;
            -- 28
            s28_m <= s27_e * signed(resize(s_tf(27), 5));
            s28_o <= s27_o;
            -- 29
            v_f := signed(resize(s28_o, 11)) + resize(shift_right(s28_m, 3), 11);
            s29_o <= unsigned(v_f(8 downto 0));
            -- 30: o * 876 = (o << 10) - (o << 7) - (o << 4) - (o << 2)
            s30_a <= shift_left(resize(s29_o, 19), 10) - shift_left(resize(s29_o, 19), 7);
            s30_b <= shift_left(resize(s29_o, 14), 4) + shift_left(resize(s29_o, 14), 2);
            -- 31
            v_y := s30_a - resize(s30_b, 19);
            s31_y <= to_unsigned(64, 10) + resize(v_y(18 downto 8), 10);
        end if;
    end process p_blend;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING (stages 32 .. 37): luma [1 2 1]/4, luma 13/16
    -- toward black (white 776), blanking gate.  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 32 .. 34: taps y(n + 1), y(n), y(n - 1)
            s_yd(1) <= s31_y;
            for i in 2 to 3 loop s_yd(i) <= s_yd(i - 1); end loop;
            s34_ya <= resize(s_yd(1), 12) + resize(s_yd(3), 12) + 2;
            s34_yb <= s_yd(2);
            -- 35
            v_y := s34_ya + shift_left(resize(s34_yb, 12), 1);
            s35_y <= v_y(11 downto 2);
            -- 36: luma 13/16 toward black
            if s35_y > 64 then v_dl := s35_y - 64; else v_dl := (others => '0'); end if;
            s36_y <= s35_y - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            -- 37: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s37_y <= s36_y;
            else
                s37_y <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s37_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture tweed;
