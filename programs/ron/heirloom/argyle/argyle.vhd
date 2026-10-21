-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Argyle v0.5.0 -- Heirloom #8, rebuilt from the photo WIN_20230207_23_50_54.
--
-- The photo: a lattice of nested diamonds, 7.32 across and 4.14 down (2-cell
-- period 524.8 px, row period 261.0 lines), every other diamond a row lower.
-- Magenta cores (lens-shaped: pointed at the sides, round on top), about eight
-- fine rings out to a dark diagonal boundary: pink / white near the core,
-- lavender, blue, cyan and green outside.  Where four diamonds meet, the
-- junctions alternate: a dark-centred green X between vertical neighbours and
-- a cyan X between horizontal ones.  Every other core in a row is big, the
-- next one small.
--
-- The patch: a horizontal oscillator G (a rounded triangle with a weak half-
-- frequency component -> the big / small cores) plus a vertical triangle H,
-- summed (s = G + H) into a colourizer: green carries the rings (a fold of s),
-- red and blue are smooth functions of the oscillators -- taken 15 px further
-- right, the capture's chroma offset.  All of it fitted to the photo
-- (.heirloom_work/rb_argyle; tables in argyle.py).
--
--   K1  Columns  diamonds across 16:9, 2 .. 16 (7.32 = the photo)
--   K2  Rows     diamond rows down the frame, 1 .. 10 (4.14)
--   K3  Pairs    the big / small core alternation, 0 (all equal) .. 4x (photo 1x)
--   K4  Drift    the lattice slides along its diagonal, bipolar, still in the centre
--   K5  Bend     how far the source (camera luma / Osc) bends the field
--   K6  Fringe   the colour offset against the rings, 0 .. 64 px (photo 15)
--   S7  Saw      vertical oscillator as a sawtooth: diamonds -> stacked chevrons
--   S8  Hard     hard-clipped rings
--   S9  Shape    Lens (the photo) / Diamond: triangle horizontal oscillator, straight-sided cores
--   S10 Palette  Neon (the photo) / Warm: red and blue swapped, an amber / orange lattice
--   S11 Source   Video / Osc
--   P12 CORES    oscillator gain: 0 = no cores, soft rings; 50 % = the photo;
--                100 % = cores and junctions clip flat, a solid argyle
--
-- Virtual 1080-line coordinates (diamonds keep their shape at SD / 720p /
-- 1080; 4:3 SD shows 5.5 across).  Generated avid; output conditioning (luma
-- [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma to ~777) before the blanking
-- gate.  Latency 24 clocks in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.argyle_pkg.all;
use work.all;

architecture argyle of program_top is
    constant C_LAT : integer := 24;   -- matches processing_delay_clks

    constant C_P0 : unsigned(23 downto 0) := to_unsigned(6665510, 24);   -- phase at the screen centre (0.3973)
    constant C_Q0 : unsigned(23 downto 0) := to_unsigned(8504327, 24);   -- row phase at the centre line (0.5069)

    -- virtual px per pixel (Q.8 phase scale): 4369 = 1080 lines, 6554 = 720p, 8738 = SD (4:3)
    function f_kx(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(8738, 14);
            when 5 | 6 | 14     => return to_unsigned(6554, 14);
            when others         => return to_unsigned(4369, 14);
        end case;
    end function;
    -- row-phase step per line per Q.8 row: virtual lines per line (Q.8) * 243/256 (Q.2)
    function f_ys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1080, 11);   -- 480i  (4.45 virtual lines)
            when 4          => return to_unsigned(547, 11);    -- 480p  (2.25)
            when 8          => return to_unsigned(911, 11);    -- 576i  (3.75)
            when 12         => return to_unsigned(456, 11);    -- 576p  (1.875)
            when 5 | 6 | 14 => return to_unsigned(365, 11);    -- 720p  (1.5)
            when 1 | 2 | 10 => return to_unsigned(486, 11);    -- 1080i (2)
            when others     => return to_unsigned(243, 11);    -- 1080p (1)
        end case;
    end function;
    function f_tri8(p : unsigned(15 downto 0)) return signed is
        variable v : unsigned(15 downto 0);
    begin
        if p(15) = '1' then v := p - 32768; else v := 32768 - p; end if;
        return to_signed(127, 9) - signed(resize(v(15 downto 7), 9));
    end function;

    -- timing
    signal t_kx : unsigned(13 downto 0) := to_unsigned(4369, 14);
    signal t_ys : unsigned(10 downto 0) := to_unsigned(243, 11);

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- controls: glides K1 K2 K3 K5 P12 (Q.3)
    type t_g is array(0 to 4) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(389 * 8, 13), to_unsigned(357 * 8, 13), to_unsigned(256 * 8, 13),
                         to_unsigned(394 * 8, 13), to_unsigned(512 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');
    signal q_p : unsigned(4 downto 0) := (others => '0');
    signal q_ld, q_st : std_logic := '0';
    signal q_wr : std_logic_vector(1 to 10) := (others => '0');
    signal s_m4 : unsigned(8 downto 0) := (others => '0');
    signal s_s4 : std_logic := '0';
    signal s_k6 : unsigned(9 downto 0) := to_unsigned(240, 10);

    -- knob curves
    signal k_cols : unsigned(11 downto 0) := to_unsigned(1872, 12);   -- Q.8 diamonds across 1920 virtual
    signal k_rows : unsigned(11 downto 0) := to_unsigned(1059, 12);   -- Q.8 rows per 1080
    signal k_eps  : unsigned(9 downto 0) := to_unsigned(256, 10);     -- Q.8 pairs
    signal k_bend : unsigned(8 downto 0) := to_unsigned(154, 9);
    signal k_a    : unsigned(9 downto 0) := to_unsigned(256, 10);     -- Q.8 gain
    signal k_ag   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal k_c1, k_c2, k_c, k_hi, k_hi2, k_pr : unsigned(19 downto 0) := (others => '0');
    signal k_al, k_al2, k_al3, k_al4 : std_logic := '1';

    -- serial multiplier (per field)
    signal m_a  : unsigned(19 downto 0) := (others => '0');
    signal m_b  : unsigned(17 downto 0) := (others => '0');
    signal m_h  : unsigned(19 downto 0) := (others => '0');
    signal m_l  : unsigned(17 downto 0) := (others => '0');

    -- per-field values
    signal f_inc  : unsigned(19 downto 0) := to_unsigned(31947, 20);
    signal f_ck   : unsigned(12 downto 0) := to_unsigned(1996, 13);
    signal f_p0   : unsigned(23 downto 0) := (others => '0');
    signal f_dp   : unsigned(23 downto 0) := to_unsigned(479040, 24);
    signal f_yinc : unsigned(19 downto 0) := to_unsigned(64336, 20);
    signal f_msq  : unsigned(17 downto 0) := (others => '0');
    signal f_dpx  : unsigned(19 downto 0) := (others => '0');
    signal f_rq   : unsigned(12 downto 0) := (others => '0');
    signal f_dqy  : unsigned(19 downto 0) := (others => '0');
    signal f_ae   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal f_px, f_qy : unsigned(23 downto 0) := (others => '0');
    signal f_neg  : std_logic := '0';
    signal f_ps, f_pds, f_qtop : unsigned(23 downto 0) := (others => '0');
    signal f_saw, f_hard, f_dia, f_warm, f_osc : std_logic := '0';
    signal f_ot : unsigned(15 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- per-line engine (computes the NEXT line) and committed line values
    signal le_st  : unsigned(4 downto 0) := (others => '0');
    signal le_k   : unsigned(4 downto 0) := (others => '0');
    signal le_j   : unsigned(1 downto 0) := (others => '0');
    signal le_i   : unsigned(3 downto 0) := (others => '0');
    signal n_q    : unsigned(23 downto 0) := (others => '0');
    signal n_oy   : unsigned(15 downto 0) := (others => '0');
    signal n_fr   : unsigned(7 downto 0) := (others => '0');
    signal n_saw  : signed(12 downto 0) := (others => '0');
    signal n_ha, n_hb, n_h : signed(12 downto 0) := (others => '0');
    signal n_hp   : signed(13 downto 0) := (others => '0');
    signal n_r    : unsigned(4 downto 0) := (others => '0');
    signal n_fh   : unsigned(3 downto 0) := (others => '0');
    signal n_bank : std_logic := '0';
    signal lm_a   : signed(23 downto 0) := (others => '0');
    signal lm_b   : unsigned(9 downto 0) := (others => '0');
    signal lm_acc : signed(23 downto 0) := (others => '0');
    type t_s9a is array(0 to 3) of signed(9 downto 0);
    type t_s15a is array(0 to 3) of signed(14 downto 0);
    signal n_wa, n_wb : std_logic_vector(31 downto 0) := (others => '0');
    signal n_d, n_v, n_vp : t_s9a := (others => (others => '0'));
    signal n_p : t_s15a := (others => (others => '0'));
    signal r_ha : unsigned(7 downto 0) := (others => '0');
    signal r_hq, r_hq2 : std_logic_vector(11 downto 0) := (others => '0');
    signal r_ca : unsigned(9 downto 0) := (others => '0');
    signal r_cq, r_cq2 : std_logic_vector(31 downto 0) := (others => '0');
    signal l_hp : signed(13 downto 0) := (others => '0');
    signal l_bank : std_logic := '0';
    signal l_oy : unsigned(15 downto 0) := (others => '0');

    -- line RAM: bank(1) & entry(6) (bit 6 unused) -> R & B & BASE & AMP
    type t_lr is array(0 to 255) of std_logic_vector(31 downto 0);
    signal lr : t_lr := (others => (others => '0'));
    signal lr_wa : unsigned(7 downto 0) := (others => '0');
    signal lr_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal lr_we : std_logic := '0';
    signal lr_ra : unsigned(7 downto 0) := (others => '0');
    signal lr_q  : std_logic_vector(31 downto 0) := (others => '0');
    attribute no_rw_check : boolean;
    attribute no_rw_check of lr : signal is true;

    -- beam
    signal b_p, b_pd : unsigned(23 downto 0) := (others => '0');
    signal b_ox, b_oy : unsigned(15 downto 0) := (others => '0');

    -- oscillator RAMs: G'(phase) = (gain*GE + gain*pairs*GO) >> 8, 1024 phases, rebuilt every field
    type t_gr is array(0 to 1023) of std_logic_vector(12 downto 0);
    signal gr0, grd : t_gr := (others => (others => '0'));
    attribute no_rw_check of gr0 : signal is true;
    attribute no_rw_check of grd : signal is true;
    signal gr_wa : unsigned(9 downto 0) := (others => '0');
    signal gr_wd : std_logic_vector(12 downto 0) := (others => '0');
    signal gr_we : std_logic := '0';
    signal gr0_ra, grd_ra : unsigned(9 downto 0) := (others => '0');
    signal gr0_q, grd_q : std_logic_vector(12 downto 0) := (others => '0');
    -- G' builder
    signal e_run : std_logic := '0';
    signal e_cnt : unsigned(10 downto 0) := (others => '0');
    signal e_ga : unsigned(8 downto 0) := (others => '0');
    signal e_oa : unsigned(7 downto 0) := (others => '0');
    signal r_eg, r_eo : std_logic_vector(11 downto 0) := (others => '0');
    type t_u11a is array (natural range <>) of unsigned(10 downto 0);
    signal e_c : t_u11a(1 to 6) := (others => (others => '0'));
    signal e_v : std_logic_vector(1 to 6) := (others => '0');
    signal e_n : std_logic_vector(3 to 5) := (others => '0');
    signal e_u : unsigned(8 downto 0) := (others => '0');
    signal e_x : signed(12 downto 0) := (others => '0');
    signal e_y : unsigned(11 downto 0) := (others => '0');
    signal e_pl, e_ph : signed(19 downto 0) := (others => '0');
    signal e_p, e_p0 : signed(24 downto 0) := (others => '0');

    -- pixel path
    signal p1_tx, p1_ty : signed(8 downto 0) := (others => '0');
    signal v_y1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p2_yc : signed(10 downto 0) := (others => '0');
    signal p3_g0, p3_gd : signed(12 downto 0) := (others => '0');
    signal p3_yc : signed(10 downto 0) := (others => '0');
    signal p4_g0, p4_gd : signed(12 downto 0) := (others => '0');
    signal p4_vb : signed(15 downto 0) := (others => '0');
    signal p5_g0, p5_gd : signed(13 downto 0) := (others => '0');
    signal p6_s0 : signed(15 downto 0) := (others => '0');
    signal p6_li : unsigned(5 downto 0) := (others => '0');
    signal r_ra : unsigned(8 downto 0) := (others => '0');
    signal r_rv, r_rv2 : std_logic_vector(7 downto 0) := (others => '0');
    signal p9_c : std_logic_vector(31 downto 0) := (others => '0');
    signal p10_pm : signed(15 downto 0) := (others => '0');
    signal p10_r, p10_b, p10_ba : unsigned(7 downto 0) := (others => '0');
    signal p11_r, p11_g, p11_b : unsigned(7 downto 0) := (others => '0');
    signal p12_yr, p12_yg, p12_yb, p12_ur, p12_ug, p12_ub, p12_vr, p12_vg, p12_vb : signed(18 downto 0) := (others => '0');
    signal p13_y, p13_u, p13_v : signed(19 downto 0) := (others => '0');
    signal p14_y, p14_u, p14_v : signed(12 downto 0) := (others => '0');
    signal p15_y, p15_u, p15_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p16_y, p16_u, p16_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_sl is array (natural range <>) of std_logic;
    signal p_hard : t_sl(8 to 9) := (others => '0');
    signal p_warm : t_sl(8 to 10) := (others => '0');

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y17, s_y18, s_y19, s_y20, s_y21, s_y22x, s_yf22 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs (registered address and output) and the line RAM
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_eg  <= C_GE(to_integer(e_ga));
            r_eo  <= C_GO(to_integer(e_oa));
            r_rv  <= C_RING(to_integer(r_ra));
            r_hq  <= C_HH(to_integer(r_ha));
            r_cq  <= C_CT(to_integer(r_ca));
        end if;
    end process p_rom;

    p_grw : process(clk)
    begin
        if rising_edge(clk) then
            if gr_we = '1' then
                gr0(to_integer(gr_wa)) <= gr_wd;
                grd(to_integer(gr_wa)) <= gr_wd;
            end if;
        end if;
    end process p_grw;
    p_grr : process(clk)
    begin
        if rising_edge(clk) then
            gr0_q <= gr0(to_integer(gr0_ra));
            grd_q <= grd(to_integer(grd_ra));
        end if;
    end process p_grr;

    ---------------------------------------------------------------------------
    -- G' builder (vblank zone, after the per-field products): for each of 1024
    -- phases, G' = (gain*GE + gain*pairs*GO) >> 8 (Diamond: a triangle for GE).
    -- Two clocks per phase on one split multiplier.
    ---------------------------------------------------------------------------
    p_gbuild : process(clk)
        variable v_t : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            gr_we <= '0';
            if s_zone = '1' and s_zc = 400 then
                e_run <= '1'; e_cnt <= (others => '0');
            elsif e_run = '1' then
                if e_cnt = 2047 then e_run <= '0'; end if;
                e_cnt <= e_cnt + 1;
            end if;
            -- E1: ROM addresses (entry i = cnt(10:1); GE index i(8:0), GO index i(8:1))
            e_ga <= e_cnt(9 downto 1); e_oa <= e_cnt(9 downto 2);
            e_c(1) <= e_cnt; e_v(1) <= e_run;
            -- E2: ROM out; distance to the cell centre for the Diamond triangle
            e_c(2) <= e_c(1); e_v(2) <= e_v(1);
            if e_c(1)(9 downto 1) >= 256 then e_u <= 511 - e_c(1)(9 downto 1); else e_u <= e_c(1)(9 downto 1); end if;
            -- E3: operands: phase 0 = GE (or the triangle) x gain, phase 1 = +-GO x gain*pairs
            e_c(3) <= e_c(2); e_v(3) <= e_v(2);
            e_n(3) <= '0';
            if e_c(2)(0) = '0' then
                if f_dia = '1' then
                    v_t := signed(resize(e_u, 13)); v_t := shift_left(v_t, 2) + shift_left(v_t, 1) + shift_right(v_t, 1);
                    e_x <= to_signed(836, 13) - v_t;
                else
                    e_x <= resize(signed(r_eg), 13);
                end if;
                e_y <= resize(k_a, 12);
            else
                e_x <= resize(signed(r_eo), 13); e_n(3) <= e_c(2)(10);
                e_y <= f_ae;
            end if;
            -- E4: partial products (operand split 6/6)
            e_c(4) <= e_c(3); e_v(4) <= e_v(3); e_n(4) <= e_n(3);
            e_pl <= resize(e_x * signed(resize(e_y(5 downto 0), 7)), 20);
            e_ph <= resize(e_x * signed(resize(e_y(11 downto 6), 7)), 20);
            -- E5: sum
            e_c(5) <= e_c(4); e_v(5) <= e_v(4);
            e_p <= resize(e_pl, 25) + shift_left(resize(e_ph, 25), 6); e_n(5) <= e_n(4);
            -- E6: hold the GE product, write on the GO product
            e_c(6) <= e_c(5); e_v(6) <= e_v(5);
            if e_c(5)(0) = '0' then
                e_p0 <= e_p;
            elsif e_v(5) = '1' then
                gr_wa <= e_c(5)(10 downto 1);
                if e_n(5) = '1' then gr_wd <= std_logic_vector(resize(shift_right(e_p0 - e_p, 8), 13));
                else gr_wd <= std_logic_vector(resize(shift_right(e_p0 + e_p, 8), 13)); end if;
                gr_we <= '1';
            end if;
        end if;
    end process p_gbuild;

    p_lrw : process(clk)
    begin
        if rising_edge(clk) then
            if lr_we = '1' then
                lr(to_integer(lr_wa)) <= lr_wd;
            end if;
        end if;
    end process p_lrw;
    p_lrr : process(clk)
    begin
        if rising_edge(clk) then
            lr_q <= lr(to_integer(lr_ra));
        end if;
    end process p_lrr;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one serial multiplier)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_x  : signed(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_p  : integer range 0 to 31;
        variable v_ph : integer range 0 to 31;
        variable v_s  : unsigned(20 downto 0);
        variable v_pr : unsigned(37 downto 0);
        variable v_g  : unsigned(9 downto 0);
        variable v_t  : unsigned(14 downto 0);
        variable v_t2 : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            t_kx <= f_kx(registers_in(8)(3 downto 0));
            t_ys <= f_ys(registers_in(8)(3 downto 0));

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
                -- K4 Drift: bipolar, deadband 8
                v_x := signed(resize(unsigned(registers_in(3)), 11)) - 512;
                if v_x > 8 then s_s4 <= '0'; v_u := unsigned(v_x) - 8; s_m4 <= v_u(8 downto 0);
                elsif v_x < -8 then s_s4 <= '1'; v_u := unsigned(-v_x) - 8; s_m4 <= v_u(8 downto 0);
                else s_s4 <= '0'; s_m4 <= (others => '0'); end if;
                s_k6 <= unsigned(registers_in(5));
                f_saw  <= registers_in(6)(0);
                f_hard <= registers_in(6)(1);
                f_dia  <= registers_in(6)(2);
                f_warm <= registers_in(6)(3);
                f_osc  <= registers_in(6)(4);
                -- drift accumulators (last field's products)
                if f_neg = '1' then
                    f_px <= f_px - resize(f_dpx, 24); f_qy <= f_qy - resize(f_dqy, 24);
                else
                    f_px <= f_px + resize(f_dpx, 24); f_qy <= f_qy + resize(f_dqy, 24);
                end if;
                f_neg <= s_s4;
                f_ot <= f_ot + 197;
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

            -- knob curves (settle long before the sequencer reads them)
            v_g := s_g(0)(12 downto 3);                          -- cols = 2 + 14 g/1023 (Q.8: 512 + g*7/2)
            v_t := resize(v_g & "00", 15) + resize(v_g & '0', 15) + resize(v_g, 15);
            k_cols <= to_unsigned(512, 12) + resize(v_t(14 downto 1), 12);
            v_g := s_g(1)(12 downto 3);                          -- rows = 1 + 9 g/1023 (Q.8: 256 + g*9/4)
            v_t := resize(v_g & "000", 15) + resize(v_g, 15);
            k_rows <= to_unsigned(256, 12) + resize(v_t(14 downto 2), 12);
            k_eps <= s_g(2)(12 downto 3);
            v_g := s_g(3)(12 downto 3);                          -- bend = g*25/64
            v_t := resize(v_g & "0000", 15) + resize(v_g & "000", 15) + resize(v_g, 15);
            k_bend <= resize(v_t(14 downto 6), 9);
            -- gain a: 0.55 .. 1.0 .. 2.2 = 141 + g*115/512 (g <= 512), 256 + (g-512)*307/512; one add per clock
            v_g := s_g(4)(12 downto 3);
            if v_g <= 512 then k_al <= '1'; k_ag <= v_g; else k_al <= '0'; k_ag <= v_g - 512; end if;
            k_c1 <= resize(k_ag & "00000", 20) + resize(k_ag & "0000", 20);      -- 48 g
            k_c2 <= resize(k_ag & '0', 20) + resize(k_ag, 20);                   -- 3 g
            if k_al = '1' then k_hi <= resize(k_ag & "000000", 20); else k_hi <= resize(k_ag & "00000000", 20); end if;
            k_al2 <= k_al;
            k_c <= k_c1 + k_c2; k_hi2 <= k_hi; k_al3 <= k_al2;                   -- 51 g
            k_pr <= k_c + k_hi2; k_al4 <= k_al3;                                 -- 115 g / 307 g
            if k_al4 = '1' then k_a <= to_unsigned(141, 10) + resize(k_pr(19 downto 9), 10);
            else k_a <= to_unsigned(256, 10) + resize(k_pr(19 downto 9), 10); end if;

            -- serial products: slot p = zc/32 (1..10), phase 0 load, 1..18 step, 20 store
            v_p  := to_integer(s_zc(9 downto 5));
            v_ph := to_integer(s_zc(4 downto 0));
            q_p <= s_zc(9 downto 5);
            if s_zone = '1' and v_p >= 1 and v_p <= 10 and v_ph = 0 then q_ld <= '1'; else q_ld <= '0'; end if;
            if s_zone = '1' and v_p >= 1 and v_p <= 10 and v_ph >= 1 and v_ph <= 18 then q_st <= '1'; else q_st <= '0'; end if;
            for k in 1 to 10 loop
                if s_zone = '1' and v_p = k and v_ph = 20 then q_wr(k) <= '1'; else q_wr(k) <= '0'; end if;
            end loop;
            if q_ld = '1' then
                m_h <= (others => '0'); m_l <= (others => '0');
                case to_integer(q_p) is
                    when 1  => m_a <= resize(k_cols, 20);  m_b <= resize(t_kx, 18);
                    when 2  => m_a <= resize(k_cols, 20);  m_b <= to_unsigned(273, 18);
                    when 3  => m_a <= f_inc;               m_b <= resize(s_w(11 downto 1), 18);
                    when 4  => m_a <= resize(f_ck, 20);    m_b <= resize(s_k6, 18);
                    when 5  => m_a <= resize(k_rows, 20);  m_b <= resize(t_ys, 18);
                    when 6  => m_a <= resize(s_m4, 20);    m_b <= resize(s_m4, 18);
                    when 7  => m_a <= resize(f_ck, 20);    m_b <= resize(f_msq(17 downto 11), 18);
                    when 8  => m_a <= resize(k_rows, 20);  m_b <= to_unsigned(485, 18);
                    when 9  => m_a <= resize(f_rq, 20);    m_b <= resize(f_msq(17 downto 11), 18);
                    when others => m_a <= resize(k_a, 20); m_b <= resize(k_eps, 18);
                end case;
            end if;
            if q_st = '1' then
                if m_b(0) = '1' then v_s := resize(m_h, 21) + resize(m_a, 21); else v_s := resize(m_h, 21); end if;
                m_h <= v_s(20 downto 1);
                m_l <= v_s(0) & m_l(17 downto 1);
                m_b <= '0' & m_b(17 downto 1);
            end if;
            v_pr := m_h & m_l;
            if q_wr(1)  = '1' then f_inc  <= v_pr(27 downto 8); end if;
            if q_wr(2)  = '1' then f_ck   <= v_pr(20 downto 8); end if;
            if q_wr(3)  = '1' then f_p0   <= C_P0 - v_pr(23 downto 0); end if;
            if q_wr(4)  = '1' then f_dp   <= v_pr(23 downto 0); end if;
            if q_wr(5)  = '1' then f_yinc <= v_pr(21 downto 2); end if;
            if q_wr(6)  = '1' then f_msq  <= v_pr(17 downto 0); end if;
            if q_wr(7)  = '1' then f_dpx  <= v_pr(19 downto 0); end if;
            if q_wr(8)  = '1' then f_rq   <= v_pr(20 downto 8); end if;
            if q_wr(9)  = '1' then f_dqy  <= v_pr(19 downto 0); end if;
            if q_wr(10) = '1' then f_ae   <= v_pr(19 downto 8); end if;

            -- line-start and frame-top phases
            f_ps   <= f_p0 + f_px;
            f_pds  <= f_ps + f_dp;
            f_qtop <= C_Q0 - (k_rows(8 downto 0) & "000000000000000") + f_qy;

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
    -- Per-line engine.  Kicked at the source avid rise (and for line 0 late in
    -- the vblank zone); computes the NEXT line: H (interpolated), H' = gain*H,
    -- the colour-table row and fraction, then the 128-entry line table (rows
    -- interpolated in H, quarter steps in G) into the other RAM bank.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_hp : signed(14 downto 0);
        variable v_c  : signed(9 downto 0);
        variable v_w  : std_logic_vector(31 downto 0);
        variable v_q  : unsigned(23 downto 0);
        variable v_e  : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            r_hq2 <= r_hq;
            r_cq2 <= r_cq;
            lr_we <= '0';

            if s_zone = '1' and s_zc = 690 then
                n_q <= f_qtop; n_oy <= f_ot;
            end if;

            if s_zone = '1' and s_zc = 700 then
                le_st <= to_unsigned(2, 5);
            elsif data_in.avid = '1' and s_avq = '0' then
                -- commit this line's values, then compute the next line
                l_hp <= n_hp; l_bank <= n_bank; l_oy <= n_oy;
                n_bank <= not n_bank;
                le_st <= to_unsigned(1, 5);
            else
                case to_integer(le_st) is
                    when 1 =>
                        n_q <= n_q + resize(f_yinc, 24);
                        n_oy <= n_oy + resize(f_yinc(19 downto 10), 16);
                        le_st <= le_st + 1;
                    when 2 =>
                        r_ha <= n_q(23 downto 16); n_fr <= n_q(15 downto 8);
                        v_q := n_q + to_unsigned(4194304, 24);
                        n_saw <= signed(resize(v_q(23 downto 13), 13)) - 1024;
                        le_st <= le_st + 1;
                    when 3 =>
                        r_ha <= n_q(23 downto 16) + 1;
                        le_st <= le_st + 1;
                    when 4 =>
                        le_st <= le_st + 1;
                    when 5 =>
                        n_ha <= resize(signed(r_hq2), 13);
                        le_st <= le_st + 1;
                    when 6 =>
                        n_hb <= resize(signed(r_hq2), 13);
                        le_st <= le_st + 1;
                    when 7 =>
                        -- (B - A) * fr
                        lm_a <= resize(n_hb - n_ha, 24); lm_b <= resize(n_fr, 10);
                        lm_acc <= (others => '0'); le_i <= (others => '0');
                        le_st <= le_st + 1;
                    when 8 | 11 =>
                        if lm_b(0) = '1' then lm_acc <= lm_acc + lm_a; end if;
                        lm_a <= shift_left(lm_a, 1); lm_b <= '0' & lm_b(9 downto 1);
                        if le_i = 9 then le_st <= le_st + 1; else le_i <= le_i + 1; end if;
                    when 9 =>
                        if f_saw = '1' then n_h <= n_saw;
                        else n_h <= n_ha + resize(shift_right(lm_acc, 8), 13); end if;
                        le_st <= le_st + 1;
                    when 10 =>
                        -- gain * H
                        lm_a <= resize(n_h, 24); lm_b <= k_a;
                        lm_acc <= (others => '0'); le_i <= (others => '0');
                        le_st <= le_st + 1;
                    when 12 =>
                        n_hp <= resize(shift_right(lm_acc, 8), 14);
                        le_st <= le_st + 1;
                    when 13 =>
                        v_hp := resize(n_hp, 15) + 992;
                        if v_hp < 0 then v_hp := (others => '0');
                        elsif v_hp > 1983 then v_hp := to_signed(1983, 15); end if;
                        n_r <= unsigned(v_hp(10 downto 6)); n_fh <= unsigned(v_hp(5 downto 2));
                        le_k <= (others => '0');
                        le_st <= le_st + 1;
                    -- table build, per G node k
                    when 14 =>
                        r_ca <= n_r & le_k;
                        le_st <= le_st + 1;
                    when 15 =>
                        r_ca <= (n_r + 1) & le_k;
                        le_st <= le_st + 1;
                    when 16 =>
                        le_st <= le_st + 1;
                    when 17 =>
                        n_wa <= r_cq2;
                        le_st <= le_st + 1;
                    when 18 =>
                        n_wb <= r_cq2;
                        le_st <= le_st + 1;
                    when 19 =>
                        for j in 0 to 2 loop
                            n_d(j) <= signed(resize(unsigned(n_wb(31 - 8 * j downto 24 - 8 * j)), 10))
                                      - signed(resize(unsigned(n_wa(31 - 8 * j downto 24 - 8 * j)), 10));
                        end loop;
                        n_d(3) <= resize(signed(n_wb(7 downto 0)), 10) - resize(signed(n_wa(7 downto 0)), 10);
                        le_st <= le_st + 1;
                    when 20 =>
                        for j in 0 to 3 loop
                            n_p(j) <= n_d(j) * signed(resize(n_fh, 5));
                        end loop;
                        le_st <= le_st + 1;
                    when 21 =>
                        for j in 0 to 2 loop
                            n_v(j) <= signed(resize(unsigned(n_wa(31 - 8 * j downto 24 - 8 * j)), 10))
                                      + resize(shift_right(n_p(j), 4), 10);
                        end loop;
                        n_v(3) <= resize(signed(n_wa(7 downto 0)), 10) + resize(shift_right(n_p(3), 4), 10);
                        le_j <= (others => '0');
                        if le_k = 0 then le_st <= to_unsigned(23, 5); else le_st <= le_st + 1; end if;
                    when 22 =>
                        -- segment k-1: entry 2(k-1) = node k-1, entry 2(k-1)+1 = the midpoint
                        for j in 0 to 3 loop
                            if le_j(0) = '0' then v_e := resize(n_vp(j), 11);
                            else v_e := shift_right(resize(n_vp(j), 11) + resize(n_v(j), 11), 1); end if;
                            v_w(31 - 8 * j downto 24 - 8 * j) := std_logic_vector(v_e(7 downto 0));
                        end loop;
                        lr_wa <= n_bank & '0' & (le_k - 1) & le_j(0);
                        lr_wd <= v_w; lr_we <= '1';
                        if le_j(0) = '1' then le_st <= le_st + 1; else le_j <= le_j + 1; end if;
                    when 23 =>
                        n_vp <= n_v;
                        le_j <= (others => '0');
                        if le_k = 31 then
                            le_st <= to_unsigned(24, 5);
                        else
                            le_k <= le_k + 1;
                            le_st <= to_unsigned(14, 5);
                        end if;
                    when 24 =>
                        -- last segment: entries 62, 63 hold node 31
                        for j in 0 to 3 loop v_w(31 - 8 * j downto 24 - 8 * j) := std_logic_vector(n_vp(j)(7 downto 0)); end loop;
                        lr_wa <= n_bank & '0' & to_unsigned(31, 5) & le_j(0);
                        lr_wd <= v_w; lr_we <= '1';
                        if le_j(0) = '1' then le_st <= (others => '0'); else le_j <= le_j + 1; end if;
                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: oscillator phases (field-shifted chroma phase), Osc triangles
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_p <= f_ps; b_pd <= f_pds;
                b_ox <= f_ot; b_oy <= l_oy;
            elsif g_avid = '1' then
                b_p <= b_p + resize(f_inc, 24);
                b_pd <= b_pd + resize(f_inc, 24);
                b_ox <= b_ox + resize(f_inc(19 downto 10), 16);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a   : signed(10 downto 0);
        variable v_t   : signed(12 downto 0);
        variable v_g   : signed(14 downto 0);
        variable v_s   : signed(16 downto 0);
        variable v_li  : signed(15 downto 0);
        variable v_ri  : signed(16 downto 0);
        variable v_rv  : signed(7 downto 0);
        variable v_gg  : signed(11 downto 0);
        variable v_r, v_b : unsigned(7 downto 0);
        variable v_y   : signed(12 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_dl  : unsigned(9 downto 0);
        variable v_ge  : signed(11 downto 0);
        variable v_ox  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: oscillator RAM addresses (ring phase, colour phase); Osc triangles; video tap
            gr0_ra <= b_p(23 downto 14); grd_ra <= b_pd(23 downto 14);
            p1_tx <= f_tri8(b_ox); p1_ty <= f_tri8(b_oy);
            v_y1 <= unsigned(data_in.y);

            -- 2: RAM out; source value centred (Osc: 1.25 tx + 0.75 ty)
            if f_osc = '1' then
                v_ox := resize(p1_tx, 12) + resize(shift_right(p1_tx, 2), 12) + resize(shift_right(p1_ty, 1), 12) + resize(shift_right(p1_ty, 2), 12);
                p2_yc <= resize(v_ox, 11);
            else
                p2_yc <= signed(resize(v_y1, 11)) - 512;
            end if;

            -- 3: fabric copies
            p3_g0 <= signed(gr0_q); p3_gd <= signed(grd_q); p3_yc <= p2_yc;

            -- 4: bend = source * Bend (5-bit)
            p4_g0 <= p3_g0; p4_gd <= p3_gd;
            p4_vb <= resize(p3_yc * signed(resize(k_bend(8 downto 4), 6)), 16);

            -- 5: G (ring phase) and Gd (colour phase), bent by the source
            p5_g0 <= resize(p4_g0, 14) + resize(shift_right(p4_vb, 4), 14);
            p5_gd <= resize(p4_gd, 14) + resize(shift_right(p4_vb, 4), 14);

            -- 6: field s = G + H'; colour-table column (1/32 in G)
            p6_s0 <= resize(p5_g0, 16) + resize(l_hp, 16);
            v_li := resize(p5_gd, 16) + 1024;
            if v_li < 0 then p6_li <= (others => '0');
            elsif v_li(15 downto 11) /= 0 then p6_li <= (others => '1');
            else p6_li <= unsigned(v_li(10 downto 5)); end if;

            -- 7: ring and line-table addresses
            v_ri := resize(p6_s0, 17) + 2048;
            if v_ri < 0 then r_ra <= (others => '0');
            elsif v_ri(16 downto 12) /= 0 then r_ra <= (others => '1');
            else r_ra <= unsigned(v_ri(11 downto 3)); end if;
            lr_ra <= l_bank & '0' & p6_li;
            p_hard(8) <= f_hard; p_warm(8) <= f_warm;

            -- 8: ROM / RAM out (p_rom, p_lrr); 9: fabric copies
            r_rv2 <= r_rv;
            p9_c <= lr_q;

            -- 10: amp * ring
            v_rv := signed(r_rv2);
            if p_hard(9) = '1' then
                if v_rv >= 0 then v_rv := to_signed(127, 8); else v_rv := to_signed(-127, 8); end if;
            end if;
            p10_pm <= signed(p9_c(7 downto 0)) * v_rv;
            p10_r <= unsigned(p9_c(31 downto 24)); p10_b <= unsigned(p9_c(23 downto 16)); p10_ba <= unsigned(p9_c(15 downto 8));

            -- 11: green = base + amp*ring/64, clipped; Warm swaps red and blue
            v_gg := signed(resize(p10_ba, 12)) + resize(shift_right(p10_pm, 6), 12);
            if v_gg < 0 then p11_g <= (others => '0');
            elsif v_gg > 255 then p11_g <= (others => '1');
            else p11_g <= unsigned(v_gg(7 downto 0)); end if;
            if p_warm(10) = '1' then p11_r <= p10_b; p11_b <= p10_r; else p11_r <= p10_r; p11_b <= p10_b; end if;

            -- 12: RGB -> YUV products, 2-term constants (Y pre-boosted 16/13: 320 640 128;
            --     U = Cr: 448 -384 -64; V = Cb: -160 -288 448; all /256)
            p12_yr <= signed(resize(p11_r & x"00", 19)) + signed(resize(p11_r & "000000", 19));
            p12_yg <= signed(resize(p11_g & "000000000", 19)) + signed(resize(p11_g & "0000000", 19));
            p12_yb <= signed(resize(p11_b & "0000000", 19));
            p12_ur <= signed(resize(p11_r & "000000000", 19)) - signed(resize(p11_r & "000000", 19));
            p12_ug <= signed(resize(p11_g & x"00", 19)) + signed(resize(p11_g & "0000000", 19));
            p12_ub <= signed(resize(p11_b & "000000", 19));
            p12_vr <= signed(resize(p11_r & "0000000", 19)) + signed(resize(p11_r & "00000", 19));
            p12_vg <= signed(resize(p11_g & x"00", 19)) + signed(resize(p11_g & "00000", 19));
            p12_vb <= signed(resize(p11_b & "000000000", 19)) - signed(resize(p11_b & "000000", 19));

            -- 13: sums
            p13_y <= resize(p12_yr, 20) + resize(p12_yg, 20) + resize(p12_yb, 20);
            p13_u <= resize(p12_ur, 20) - resize(p12_ug, 20) - resize(p12_ub, 20);
            p13_v <= resize(p12_vb, 20) - resize(p12_vr, 20) - resize(p12_vg, 20);

            -- 14: scale and offset
            p14_y <= resize(shift_right(p13_y, 8), 13) + 64;
            p14_u <= resize(shift_right(p13_u, 8), 13) + 512;
            p14_v <= resize(shift_right(p13_v, 8), 13) + 512;

            -- 15: clamp (luma to 940: the conditioning brings it to ~777); 16: pad
            if p14_y > 940 then p15_y <= to_unsigned(940, 10); else p15_y <= unsigned(p14_y(9 downto 0)); end if;
            if p14_u < 0 then p15_u <= (others => '0'); elsif p14_u > 1023 then p15_u <= (others => '1'); else p15_u <= unsigned(p14_u(9 downto 0)); end if;
            if p14_v < 0 then p15_v <= (others => '0'); elsif p14_v > 1023 then p15_v <= (others => '1'); else p15_v <= unsigned(p14_v(9 downto 0)); end if;
            p16_y <= p15_y; p16_u <= p15_u; p16_v <= p15_v;

            -- 17..22: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two
            -- adjacent 8-sums, luma [1 2 1]; centre at stage 21 -> 22)
            s_csu(0) <= p16_u; s_csv(0) <= p16_v;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + p16_u - s_csu(7);
            s_boxv <= s_boxv + p16_v - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u22 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v22 <= v_sc(13 downto 4);
            s_y17 <= p16_y; s_y18 <= s_y17; s_y19 <= s_y18; s_y20 <= s_y19;
            s_y21 <= s_y20; s_y22x <= s_y21;
            v_sy := resize(s_y20, 12) + shift_left(resize(s_y21, 12), 1) + resize(s_y22x, 12);
            s_yf22 <= v_sy(11 downto 2);
            -- 23: luma toward black (white 940 -> 777); blanking gate
            if s_yf22 > 64 then v_dl := s_yf22 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(21) = '1' then
                s_y23 <= s_yf22 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u23 <= s_u22; s_v23 <= s_v22;
            else
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
            end if;
            -- 24: pad to the 24-clock latency
            s_y24 <= s_y23; s_u24 <= s_u23; s_v24 <= s_v23;

            -- delays
            p_hard(9) <= p_hard(8);
            p_warm(9) <= p_warm(8); p_warm(10) <= p_warm(9);
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture argyle;
