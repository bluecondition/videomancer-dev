-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Warp v0.5.0 -- Heirloom #48, rebuilt from the photo WIN_20230909_23_29_18.
--
-- The photo: fine orange / lavender / green stripes on a plane receding to a horizon seam at
-- y ~695: above it hyperbolic arcs (isolines of (x + 94)(690 - y), period 30-110 px), below it
-- broad red / violet '/' stripes ((x + 94)(y - 305)); the stripes are warped into a checker of
-- steep seam bands at the top left, moire lenses at the top right and a band of hooks and loops
-- with dark green cores either side of the seam.  A black / green margin with a staircase edge
-- on the left.
--
-- The patch: an hsync-reset VCO whose frequency is the vertical ramp (two laws either side of the
-- horizon), so its phase is a product field (x + 94) f(y); a warp phase D(x, y) added to it (a
-- 120 x 136 knot map fitted to the photo, interpolated); one colourizer over the phase; a green
-- haze fading down the frame; a keyed margin.  The colouriser runs over the FULL cycle of the phase
-- (green -> teal -> lavender -> orange -> olive -> lavender -> green, measured from the photo), with one
-- palette above the horizon and a redder / more violet one below it.
--
--   K1  Stripes  carrier density x0.5 .. x2 (centre = the photo)
--   K2  Horizon  the seam / horizon row, +-256 lines (centre = the photo, y 695)
--   K3  Flow     stripe drift, bipolar, still at the centre +-8 (square law)
--   K4  Bend     how far the camera luma (or the Osc) bends the stripe phase
--   K5  Haze     the green haze down the frame: none .. the photo (centre) .. double
--   K6  Palette  Warp (the photo) / Ember / Ice / Toxic / Candy / Mono / Neon / Rust
--   S7  Seam     On: the photo's horizon seam (two laws) / Off: one through-zero law
--   S8  Margin   the black / green left margin with its staircase edge
--   S9  Mirror   the right half mirrors the left
--   S10 Zigzag   a rolling per-line zig-zag on the stripe phase (24-line period)
--   S11 Source   Video / Osc
--   P12 WARP     0 = the clean perspective plane; the warp grows out from the horizon; 50 % = the
--                photo; 50 .. 100 % a second front doubles the warp
--
-- Virtual 1080-line coordinates (SD shows the central 4:3).  Generated avid (parity-locked start,
-- field-latched width); output conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16, luma to
-- ~777) before the blanking gate.  Latency 24 clocks in every mode.  Colours in the hardware
-- convention (u = Cr, v = Cb), see warp.py.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.warp_pkg.all;
use work.all;

architecture warp of program_top is
    constant C_LAT : integer := 24;   -- matches processing_delay_clks

    function f_tri10(p : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        if p(15) = '1' then v := not p; else v := p; end if;   -- 65535 - p for the upper half
        return v(14 downto 5);
    end function;

    -- timing
    signal t_ys  : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal t_sx  : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal t_x0  : unsigned(12 downto 0) := (others => '0');
    signal t_vp  : unsigned(8 downto 0) := to_unsigned(94, 9);
    signal t_oxi : unsigned(7 downto 0) := to_unsigned(102, 8);
    signal t_zi  : unsigned(13 downto 0) := to_unsigned(2731, 14);

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- controls (glided Q.3) and per-field values
    signal g_p12 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal g_k1  : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal g_k2  : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal f_ka, f_kb : unsigned(17 downto 0) := to_unsigned(71583, 18);
    signal f_yh  : unsigned(10 downto 0) := to_unsigned(695, 11);
    signal f_r1, f_r2 : unsigned(11 downto 0) := (others => '0');
    signal f_seam, f_marg, f_mir, f_zig, f_osc : std_logic := '0';
    signal f_pal : unsigned(2 downto 0) := (others => '0');
    signal f_k4h : unsigned(4 downto 0) := (others => '0');
    signal f_k5  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal f_m   : unsigned(8 downto 0) := (others => '0');
    signal f_neg3 : std_logic := '0';
    signal f_msq : unsigned(17 downto 0) := (others => '0');
    signal f_spd : signed(21 downto 0) := (others => '0');
    signal f_T   : unsigned(23 downto 0) := (others => '0');
    signal f_to, f_tz : unsigned(15 downto 0) := (others => '0');
    signal q_ma  : unsigned(17 downto 0) := (others => '0');
    signal q_mb  : unsigned(8 downto 0) := (others => '0');

    -- per-line engine
    signal e_st  : unsigned(5 downto 0) := (others => '0');
    signal e_vy  : unsigned(19 downto 0) := (others => '0');
    signal e_vyi : unsigned(10 downto 0) := (others => '0');
    signal e_zp  : unsigned(15 downto 0) := (others => '0');
    signal e_neg : std_logic := '0';
    signal e_hz  : unsigned(5 downto 0) := (others => '0');
    signal e_z   : signed(23 downto 0) := (others => '0');
    signal m_a, m_p : unsigned(31 downto 0) := (others => '0');
    signal m_b   : unsigned(10 downto 0) := (others => '0');
    signal n_F   : signed(22 downto 0) := (others => '0');
    signal n_inc : signed(23 downto 0) := (others => '0');
    signal n_S0  : unsigned(23 downto 0) := (others => '0');
    signal n_n   : unsigned(1 downto 0) := (others => '0');
    signal n_h   : unsigned(6 downto 0) := (others => '0');
    signal n_dy, n_du, n_dv : unsigned(7 downto 0) := (others => '0');
    signal n_edge : unsigned(10 downto 0) := (others => '0');
    signal n_oy  : unsigned(9 downto 0) := (others => '0');
    signal n_half, l_half : std_logic := '0';
    signal l_inc : signed(23 downto 0) := (others => '0');
    signal l_S0  : unsigned(23 downto 0) := (others => '0');
    signal l_n   : unsigned(1 downto 0) := (others => '0');
    signal l_dy, l_du, l_dv : unsigned(7 downto 0) := (others => '0');
    signal l_edge : unsigned(10 downto 0) := (others => '0');
    signal l_oy  : unsigned(9 downto 0) := (others => '0');
    signal l_bank : std_logic := '0';

    -- map fill (two neighbour knot rows -> one line of column values)
    signal mf_run : std_logic := '0';
    signal mf_c  : unsigned(6 downto 0) := (others => '0');
    signal mf_be, mf_bo : unsigned(12 downto 0) := (others => '0');
    signal mf_ae, mf_ao : unsigned(12 downto 0) := (others => '0');
    signal mf_odd : std_logic := '0';
    signal mf_fy : unsigned(3 downto 0) := (others => '0');
    signal mf_v1, mf_v2, mf_v3, mf_v4 : std_logic := '0';
    signal mf_c1, mf_c2, mf_c3, mf_c4 : unsigned(6 downto 0) := (others => '0');
    signal r_me, r_mo : std_logic_vector(3 downto 0) := (others => '0');
    signal q_me, q_mo : unsigned(3 downto 0) := (others => '0');
    signal mf_dv, mf_dvp : unsigned(7 downto 0) := (others => '0');
    signal mf_fin : std_logic := '0';
    signal b_we  : std_logic := '0';
    signal b_wa  : unsigned(7 downto 0) := (others => '0');
    signal b_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal b_ra  : unsigned(7 downto 0) := (others => '0');
    type t_buf is array (0 to 255) of std_logic_vector(15 downto 0);
    signal r_buf_ram : t_buf := (others => (others => '0'));
    signal r_bd : std_logic_vector(15 downto 0) := (others => '0');
    attribute no_rw_check : boolean;
    attribute no_rw_check of r_buf_ram : signal is true;

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- video tap
    signal v_y1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal v_src : signed(11 downto 0) := (others => '0');

    -- beam (stage 0)
    signal b_vx  : unsigned(14 downto 0) := (others => '0');
    signal b_acc : unsigned(23 downto 0) := (others => '0');
    signal b_ox  : unsigned(15 downto 0) := (others => '0');
    signal b_r   : std_logic := '0';

    -- pixel pipeline
    signal p1_vxm : unsigned(14 downto 0) := (others => '0');
    signal p1_acc : unsigned(9 downto 0) := (others => '0');
    signal p1_ox, p1_oy : unsigned(9 downto 0) := (others => '0');
    signal p2_ex : signed(12 downto 0) := (others => '0');
    signal p2_mi : unsigned(5 downto 0) := (others => '0');
    signal p2_fx : unsigned(3 downto 0) := (others => '0');
    signal p2_acc : unsigned(9 downto 0) := (others => '0');
    signal p2_osc : signed(11 downto 0) := (others => '0');
    signal p3_a, p3_b : unsigned(7 downto 0) := (others => '0');
    signal p3_fx : unsigned(3 downto 0) := (others => '0');
    signal p3_acc : unsigned(9 downto 0) := (others => '0');
    signal p3_src : signed(11 downto 0) := (others => '0');
    signal p3_a4 : unsigned(2 downto 0) := (others => '0');
    signal p3_mi : unsigned(5 downto 0) := (others => '0');
    signal p4_a : unsigned(7 downto 0) := (others => '0');
    signal p4_pr : signed(12 downto 0) := (others => '0');
    signal p4_bp : signed(17 downto 0) := (others => '0');
    signal p4_acc : unsigned(9 downto 0) := (others => '0');
    signal p5_d : unsigned(7 downto 0) := (others => '0');
    signal p5_bend : unsigned(9 downto 0) := (others => '0');
    signal p5_acc : unsigned(9 downto 0) := (others => '0');
    signal p6_p : unsigned(9 downto 0) := (others => '0');
    signal r_pa : unsigned(9 downto 0) := (others => '0');
    signal r_ma : unsigned(5 downto 0) := (others => '0');
    signal r_py, r_pu, r_pv : std_logic_vector(7 downto 0) := (others => '0');
    signal r_my, r_mu, r_mv : std_logic_vector(7 downto 0) := (others => '0');
    signal p9_y, p9_u, p9_v, p9_my, p9_mu, p9_mv : unsigned(7 downto 0) := (others => '0');
    signal p10_y, p10_u, p10_v, p10_my, p10_mu, p10_mv : unsigned(7 downto 0) := (others => '0');
    signal p11_y, p11_u, p11_v : unsigned(9 downto 0) := (others => '0');
    type t_u3 is array (natural range <>) of unsigned(2 downto 0);
    signal p_a4 : t_u3(4 to 10) := (others => (others => '0'));
    type t_u6 is array (natural range <>) of unsigned(5 downto 0);
    signal p_mi : t_u6(4 to 6) := (others => (others => '0'));
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal p12_y, p13_y, p14_y, p15_y, p16_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p12_u, p13_u, p14_u, p15_u, p16_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p12_v, p13_v, p14_v, p15_v, p16_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
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
    -- ROMs: warp map (even / odd knot rows), palette, margin ramp
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_me <= C_ME(to_integer(mf_ae));
            r_mo <= C_MO(to_integer(mf_ao));
            r_py <= C_PAL_Y(to_integer(r_pa)); r_pu <= C_PAL_U(to_integer(r_pa)); r_pv <= C_PAL_V(to_integer(r_pa));
            r_my <= C_MG_Y(to_integer(r_ma)); r_mu <= C_MG_U(to_integer(r_ma)); r_mv <= C_MG_V(to_integer(r_ma));
        end if;
    end process p_rom;

    -- column buffer: two banks of 128 column pairs {D[c], D[c+1]}; the engine writes the bank the
    -- pixel path is not reading (no read meets the write address)
    p_bufw : process(clk)
    begin
        if rising_edge(clk) then
            if b_we = '1' then
                r_buf_ram(to_integer(b_wa)) <= b_wd;
            end if;
        end if;
    end process p_bufw;
    p_bufr : process(clk)
    begin
        if rising_edge(clk) then
            r_bd <= r_buf_ram(to_integer(b_ra));
        end if;
    end process p_bufr;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field values
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_id : integer range 0 to 15;
        variable v_d  : signed(13 downto 0);
        variable v_x  : signed(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_p  : unsigned(9 downto 0);
        variable v_q  : unsigned(9 downto 0);
        variable v_s  : unsigned(21 downto 0);
        variable v_k6 : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_id := to_integer(unsigned(registers_in(8)(3 downto 0)));
            t_ys <= unsigned(C_T_YS(v_id)); t_sx <= unsigned(C_T_SX(v_id)); t_x0 <= unsigned(C_T_X0(v_id));
            t_vp <= unsigned(C_T_VP(v_id)); t_oxi <= unsigned(C_T_OXI(v_id)); t_zi <= unsigned(C_T_ZI(v_id));

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
                -- glides: ease 1/8 of the gap per field, deadband 4
                v_d := signed(shift_left(resize(unsigned(registers_in(7)), 14), 3)) - signed(resize(g_p12, 14));
                if v_d > 31 or v_d < -32 then g_p12 <= unsigned(signed(g_p12) + resize(shift_right(v_d, 3), 13)); end if;
                v_d := signed(shift_left(resize(unsigned(registers_in(0)), 14), 3)) - signed(resize(g_k1, 14));
                if v_d > 31 or v_d < -32 then g_k1 <= unsigned(signed(g_k1) + resize(shift_right(v_d, 3), 13)); end if;
                v_d := signed(shift_left(resize(unsigned(registers_in(1)), 14), 3)) - signed(resize(g_k2, 14));
                if v_d > 31 or v_d < -32 then g_k2 <= unsigned(signed(g_k2) + resize(shift_right(v_d, 3), 13)); end if;
                -- K3 Flow: bipolar, deadband 8
                v_x := signed(resize(unsigned(registers_in(2)), 11)) - 512;
                if v_x > 8 then f_neg3 <= '0'; v_u := unsigned(v_x) - 8; f_m <= v_u(8 downto 0);
                elsif v_x < -8 then f_neg3 <= '1'; v_u := unsigned(-v_x) - 8; f_m <= v_u(8 downto 0);
                else f_neg3 <= '0'; f_m <= (others => '0'); end if;
                f_k4h <= unsigned(registers_in(3)(9 downto 5));
                f_k5  <= unsigned(registers_in(4));
                v_k6 := unsigned(registers_in(5));
                if v_k6 < 8 then f_pal <= v_k6(2 downto 0); else f_pal <= v_k6(9 downto 7); end if;
                f_seam <= registers_in(6)(0);
                f_marg <= registers_in(6)(1);
                f_mir  <= registers_in(6)(2);
                f_zig  <= registers_in(6)(3);
                f_osc  <= registers_in(6)(4);
                -- drifts
                f_T  <= unsigned(signed(f_T) + resize(f_spd, 24));
                f_to <= f_to + 131;
                f_tz <= f_tz + 1638;
            end if;
            -- zone clocks: carrier gains, horizon, warp fronts, flow speed (serial m^2)
            if s_zone = '1' then
                if s_zc = 4 then
                    f_ka <= unsigned(C_KA(to_integer(g_k1(12 downto 7))));
                    f_kb <= unsigned(C_KB(to_integer(g_k1(12 downto 7))));
                    f_yh <= to_unsigned(439, 11) + resize(g_k2(12 downto 4), 11);
                    v_p := g_p12(12 downto 3);
                    f_r1 <= resize(v_p, 12) + resize(v_p(9 downto 2), 12) + resize(v_p(9 downto 3), 12) + resize(v_p(9 downto 5), 12);
                    if v_p > 512 then
                        v_q := v_p - 512;
                        f_r2 <= resize(v_q, 12) + resize(v_q(9 downto 2), 12) + resize(v_q(9 downto 3), 12) + resize(v_q(9 downto 5), 12);
                    else
                        f_r2 <= (others => '0');
                    end if;
                    q_ma <= resize(f_m, 18); q_mb <= f_m; f_msq <= (others => '0');
                elsif s_zc > 4 and s_zc < 14 then
                    if q_mb(0) = '1' then f_msq <= f_msq + q_ma; end if;
                    q_ma <= shift_left(q_ma, 1); q_mb <= shift_right(q_mb, 1);
                elsif s_zc = 14 then
                    -- spd = m^2 * 13 / 4
                    v_s := resize(shift_left(resize(f_msq, 22), 3), 22) + resize(shift_left(resize(f_msq, 22), 2), 22) + resize(f_msq, 22);
                    if f_neg3 = '1' then f_spd <= -signed(resize(v_s(21 downto 2), 22));
                    else f_spd <= signed(resize(v_s(21 downto 2), 22)); end if;
                end if;
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
    -- Per-line engine: computes the NEXT line (kicked at the commit, i.e. at
    -- each source avid rise, and for line 0 late in the vblank zone)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_dy  : signed(12 downto 0);
        variable v_dv  : signed(12 downto 0);
        variable v_row : unsigned(7 downto 0);
        variable v_t   : unsigned(15 downto 0);
        variable v_tr  : signed(11 downto 0);
        variable v_z   : signed(15 downto 0);
        variable v_s0  : signed(23 downto 0);
        variable v_h   : unsigned(15 downto 0);
        variable v_dvl : unsigned(7 downto 0);
        variable v_top, v_bot : unsigned(3 downto 0);
        variable v_wd  : signed(3 downto 0);
        variable v_pr  : signed(8 downto 0);
        variable v_ix  : unsigned(6 downto 0);
        variable v_r9  : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            b_we <= '0';
            -- commit / kick
            if s_zone = '1' and s_zc = 600 then
                e_vy <= (others => '0');
                e_zp <= 0 - f_tz;
                e_st <= to_unsigned(1, 6);
            elsif g_avr = '1' then
                l_inc <= n_inc; l_S0 <= n_S0; l_n <= n_n;
                l_dy <= n_dy; l_du <= n_du; l_dv <= n_dv;
                l_edge <= n_edge; l_oy <= n_oy; l_half <= n_half;
                l_bank <= not l_bank;
                e_vy <= e_vy + t_ys;
                e_zp <= e_zp + t_zi(13 downto 0);
                e_st <= to_unsigned(1, 6);
            elsif e_st /= 0 then
                if e_st = 40 then e_st <= (others => '0'); else e_st <= e_st + 1; end if;
            end if;

            case to_integer(e_st) is
                when 1 =>
                    e_vyi <= e_vy(18 downto 8);
                when 2 =>
                    -- carrier law, row, warp count, haze / edge / osc lookups
                    if f_seam = '1' and e_vyi < f_yh then
                        v_dy := signed(resize(f_yh, 13)) - 5 - signed(resize(e_vyi, 13)); m_a <= resize(f_ka, 32);
                    elsif f_seam = '1' then
                        v_dy := signed(resize(e_vyi, 13)) - signed(resize(f_yh, 13)) + 390; m_a <= resize(f_kb, 32);
                    else
                        v_dy := signed(resize(f_yh, 13)) - 5 - signed(resize(e_vyi, 13)); m_a <= resize(f_ka, 32);
                    end if;
                    if v_dy < 0 then e_neg <= '1'; m_b <= unsigned(resize(-v_dy, 11));
                    else e_neg <= '0'; m_b <= unsigned(resize(v_dy, 11)); end if;
                    m_p <= (others => '0');
                    v_dv := signed(resize(e_vyi, 13)) - signed(resize(f_yh, 13));
                    if v_dv < 0 then v_dv := -v_dv; n_half <= '0'; else n_half <= '1'; end if;
                    if v_dv < signed(resize(f_r1, 13)) then
                        if v_dv < signed(resize(f_r2, 13)) then n_n <= "10"; else n_n <= "01"; end if;
                    else
                        n_n <= "00";
                    end if;
                    if e_vyi(10 downto 5) > 33 then e_hz <= unsigned(C_HZ(33));
                    else e_hz <= unsigned(C_HZ(to_integer(e_vyi(10 downto 5)))); end if;
                    if e_vyi(10 downto 4) > 67 then n_edge <= resize(unsigned(C_ED(67)) & "00", 11);
                    else n_edge <= resize(unsigned(C_ED(to_integer(e_vyi(10 downto 4)))) & "00", 11); end if;
                    v_t := resize(shift_left(resize(e_vyi, 16), 7), 16) - resize(shift_left(resize(e_vyi, 16), 3), 16) + resize(e_vyi, 16);
                    n_oy <= f_tri10(v_t);
                    v_tr := signed(resize(f_tri10(e_zp), 12)) - 512;
                    v_z := resize(shift_left(resize(v_tr, 16), 2), 16) + resize(v_tr, 16);
                    if f_zig = '1' then e_z <= shift_left(resize(v_z, 24), 10); else e_z <= (others => '0'); end if;
                    -- map fill: rows r and r + 1 (r <= 134)
                    if e_vyi(10 downto 3) > 134 then v_row := to_unsigned(134, 8); else v_row := e_vyi(10 downto 3); end if;
                    mf_odd <= v_row(0);
                    v_ix := resize(v_row(7 downto 1), 7);
                    mf_bo <= resize(shift_left(resize(v_ix, 13), 7), 13) - resize(shift_left(resize(v_ix, 13), 3), 13);
                    v_r9 := resize(v_row, 9) + 1;
                    v_ix := v_r9(7 downto 1);
                    mf_be <= resize(shift_left(resize(v_ix, 13), 7), 13) - resize(shift_left(resize(v_ix, 13), 3), 13);
                    mf_fy <= e_vy(10 downto 7);
                    mf_c <= (others => '0');
                    mf_run <= '1';
                when 3 to 13 =>
                    if m_b(0) = '1' then m_p <= m_p + m_a; end if;
                    m_a <= shift_left(m_a, 1); m_b <= shift_right(m_b, 1);
                when 14 =>
                    if e_neg = '1' then n_F <= -signed(resize(m_p(28 downto 8), 23));
                    else n_F <= signed(resize(m_p(28 downto 8), 23)); end if;
                    m_a <= resize(m_p(28 downto 8), 32); m_b <= resize(t_vp, 11); m_p <= (others => '0');
                when 15 to 25 =>
                    if m_b(0) = '1' then m_p <= m_p + m_a; end if;
                    m_a <= shift_left(m_a, 1); m_b <= shift_right(m_b, 1);
                when 26 =>
                    if e_neg = '1' then v_s0 := -signed(m_p(23 downto 0)); else v_s0 := signed(m_p(23 downto 0)); end if;
                    n_S0 <= unsigned(v_s0 + signed(f_T) + e_z);
                    if t_sx = 24 then n_inc <= resize(n_F, 24) + resize(shift_right(n_F, 1), 24);
                    elsif t_sx = 32 then n_inc <= shift_left(resize(n_F, 24), 1);
                    else n_inc <= resize(n_F, 24); end if;
                    m_a <= resize(f_k5, 32); m_b <= resize(e_hz, 11); m_p <= (others => '0');
                when 27 to 37 =>
                    if m_b(0) = '1' then m_p <= m_p + m_a; end if;
                    m_a <= shift_left(m_a, 1); m_b <= shift_right(m_b, 1);
                when 38 =>
                    n_h <= m_p(15 downto 9);
                when 39 =>
                    -- haze offsets: h * {159, 94, 75} / 256
                    v_h := shift_left(resize(n_h, 16), 7) + shift_left(resize(n_h, 16), 5) - resize(n_h, 16);
                    n_dy <= v_h(15 downto 8);
                    v_h := shift_left(resize(n_h, 16), 6) + shift_left(resize(n_h, 16), 5) - shift_left(resize(n_h, 16), 1);
                    n_du <= v_h(15 downto 8);
                    v_h := shift_left(resize(n_h, 16), 6) + shift_left(resize(n_h, 16), 3) + shift_left(resize(n_h, 16), 1) + resize(n_h, 16);
                    n_dv <= v_h(15 downto 8);
                when others => null;
            end case;

            -- map fill pipeline: issue 120 address pairs, ROM (1), fabric copy (2), column value (3), write
            if mf_run = '1' then
                mf_ae <= mf_be + resize(mf_c, 13);
                mf_ao <= mf_bo + resize(mf_c, 13);
                if mf_c = 119 then mf_run <= '0'; else mf_c <= mf_c + 1; end if;
            end if;
            mf_v1 <= mf_run; mf_c1 <= mf_c;
            q_me <= unsigned(r_me); q_mo <= unsigned(r_mo);
            mf_v2 <= mf_v1; mf_c2 <= mf_c1;
            -- stage 3: D[c] = top * 16 + wrapdiff(bot - top) * fy  (1/256 cycle, mod 256)
            if mf_odd = '1' then v_top := q_mo; v_bot := q_me; else v_top := q_me; v_bot := q_mo; end if;
            v_wd := signed(v_bot - v_top);
            v_pr := resize(v_wd * signed(resize(mf_fy, 5)), 9);
            v_dvl := shift_left(resize(v_top, 8), 4) + unsigned(resize(v_pr, 8));
            mf_v3 <= mf_v2; mf_c3 <= mf_c2;
            if mf_v3 = '1' then mf_dv <= v_dvl; end if;
            mf_v4 <= mf_v3; mf_c4 <= mf_c3;
            -- stage 4: write pair {D[c-1], D[c]} at c-1; after c = 119, {D[119], D[119]} at 119
            mf_fin <= '0';
            if mf_v4 = '1' then
                mf_dvp <= mf_dv;
                if mf_c4 /= 0 then
                    b_we <= '1'; b_wa <= (not l_bank) & (mf_c4 - 1); b_wd <= std_logic_vector(mf_dvp) & std_logic_vector(mf_dv);
                end if;
                if mf_c4 = 119 then mf_fin <= '1'; end if;
            elsif mf_fin = '1' then
                b_we <= '1'; b_wa <= (not l_bank) & to_unsigned(119, 7); b_wd <= std_logic_vector(mf_dvp) & std_logic_vector(mf_dvp);
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Pixel pipeline
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_vn  : unsigned(14 downto 0);
        variable v_rn  : std_logic;
        variable v_vxm : unsigned(14 downto 0);
        variable v_ex  : signed(12 downto 0);
        variable v_d8  : signed(7 downto 0);
        variable v_nd  : unsigned(7 downto 0);
        variable v_s9  : signed(9 downto 0);
        variable v_s10 : unsigned(9 downto 0);
        variable v_c   : unsigned(9 downto 0);
        variable v_m   : unsigned(9 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_dl  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- video tap: camera luma -> signed source (2 (Y - 512))
            v_y1 <= unsigned(data_in.y);
            v_src <= shift_left(signed(resize(v_y1, 12)) - 512, 1);

            -- 0: beam
            if g_start = '1' then
                b_vx  <= resize(t_x0, 15);
                b_acc <= l_S0;
                b_ox  <= f_to;
                b_r   <= '0';
            elsif g_avid = '1' then
                v_vn := b_vx + resize(t_sx, 15);
                if f_mir = '1' and v_vn >= 15360 then v_rn := '1'; else v_rn := '0'; end if;
                if v_rn = '0' then b_acc <= b_acc + unsigned(l_inc);
                elsif b_r = '1' then b_acc <= b_acc - unsigned(l_inc); end if;
                b_r  <= v_rn;
                b_vx <= v_vn;
                b_ox <= b_ox + resize(t_oxi, 16);
            end if;

            -- 1: mirrored x, column-buffer address
            if b_r = '1' then v_vxm := to_unsigned(30719, 15) - b_vx; else v_vxm := b_vx; end if;
            p1_vxm <= v_vxm;
            b_ra <= l_bank & v_vxm(14 downto 8);
            p1_acc <= b_acc(23 downto 14);
            p1_ox <= f_tri10(b_ox);
            p1_oy <= l_oy;

            -- 2: column-buffer data out; margin key distance; osc
            v_ex := signed(resize(p1_vxm(14 downto 4), 13)) - signed(resize(l_edge, 13)) + 4;
            p2_ex <= v_ex;
            if p1_vxm(14 downto 12) /= "000" then p2_mi <= (others => '1'); else p2_mi <= p1_vxm(11 downto 6); end if;
            p2_fx <= p1_vxm(7 downto 4);
            p2_acc <= p1_acc;
            p2_osc <= signed(resize(p1_ox, 12)) + signed(resize(p1_oy, 12)) - 1024;

            -- 3: fabric copy of the column pair; margin alpha in quarters; source select
            p3_a <= unsigned(r_bd(15 downto 8)); p3_b <= unsigned(r_bd(7 downto 0));
            p3_fx <= p2_fx; p3_acc <= p2_acc;
            if f_marg = '0' or p2_ex > 15 then p3_a4 <= to_unsigned(4, 3);
            elsif p2_ex < 0 then p3_a4 <= (others => '0');
            else p3_a4 <= unsigned(p2_ex(4 downto 2)); end if;
            p3_mi <= p2_mi;
            if f_osc = '1' then p3_src <= p2_osc; else p3_src <= v_src; end if;

            -- 4: horizontal wrap difference x fx; bend product
            v_d8 := signed(p3_b - p3_a);
            p4_pr <= resize(v_d8 * signed(resize(p3_fx, 5)), 13);
            p4_bp <= resize(p3_src * signed(resize(f_k4h, 6)), 18);
            p4_a <= p3_a; p4_acc <= p3_acc;

            -- 5: warp phase D (1/256 cycle); bend (1/1024 cycle)
            p5_d <= p4_a + unsigned(p4_pr(11 downto 4));
            p5_bend <= unsigned(p4_bp(14 downto 5));
            p5_acc <= p4_acc;

            -- 6: P = carrier + n D + bend (1/1024 cycle)
            case to_integer(l_n) is
                when 0 => v_nd := (others => '0');
                when 1 => v_nd := p5_d;
                when others => v_nd := shift_left(p5_d, 1);
            end case;
            p6_p <= p5_acc + (v_nd & "00") + p5_bend;

            -- 7: palette / margin ROM addresses; 8: ROM out; 9: fabric copy
            r_pa <= f_pal & l_half & p6_p(9 downto 4);
            r_ma <= p_mi(6);
            p9_y <= unsigned(r_py); p9_u <= unsigned(r_pu); p9_v <= unsigned(r_pv);
            p9_my <= unsigned(r_my); p9_mu <= unsigned(r_mu); p9_mv <= unsigned(r_mv);

            -- 10: haze (the green fading down the frame)
            v_s9 := signed(resize(p9_y, 10)) - signed(resize(l_dy, 10));
            if v_s9 < 0 then p10_y <= (others => '0'); else p10_y <= unsigned(v_s9(7 downto 0)); end if;
            v_s10 := resize(p9_u, 10) + resize(l_du, 10);
            if v_s10 > 255 then p10_u <= (others => '1'); else p10_u <= v_s10(7 downto 0); end if;
            v_s10 := resize(p9_v, 10) + resize(l_dv, 10);
            if v_s10 > 255 then p10_v <= (others => '1'); else p10_v <= v_s10(7 downto 0); end if;
            p10_my <= p9_my; p10_mu <= p9_mu; p10_mv <= p9_mv;

            -- 11: margin key, in quarters
            case to_integer(p_a4(10)) is
                when 0 =>
                    p11_y <= p10_my & "00"; p11_u <= p10_mu & "00"; p11_v <= p10_mv & "00";
                when 1 =>
                    p11_y <= resize(p10_my, 10) + resize(p10_my & '0', 10) + resize(p10_y, 10);
                    p11_u <= resize(p10_mu, 10) + resize(p10_mu & '0', 10) + resize(p10_u, 10);
                    p11_v <= resize(p10_mv, 10) + resize(p10_mv & '0', 10) + resize(p10_v, 10);
                when 2 =>
                    p11_y <= shift_left(resize(p10_my, 10) + resize(p10_y, 10), 1);
                    p11_u <= shift_left(resize(p10_mu, 10) + resize(p10_u, 10), 1);
                    p11_v <= shift_left(resize(p10_mv, 10) + resize(p10_v, 10), 1);
                when 3 =>
                    p11_y <= resize(p10_y, 10) + resize(p10_y & '0', 10) + resize(p10_my, 10);
                    p11_u <= resize(p10_u, 10) + resize(p10_u & '0', 10) + resize(p10_mu, 10);
                    p11_v <= resize(p10_v, 10) + resize(p10_v & '0', 10) + resize(p10_mv, 10);
                when others =>
                    p11_y <= p10_y & "00"; p11_u <= p10_u & "00"; p11_v <= p10_v & "00";
            end case;

            -- 12..16: pad to stage 16
            p12_y <= p11_y; p12_u <= p11_u; p12_v <= p11_v;
            p13_y <= p12_y; p13_u <= p12_u; p13_v <= p12_v;
            p14_y <= p13_y; p14_u <= p13_u; p14_v <= p13_v;
            p15_y <= p14_y; p15_u <= p14_u; p15_v <= p14_v;
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
            -- 24: output register
            s_y24 <= s_y23; s_u24 <= s_u23; s_v24 <= s_v23;

            -- delays
            p_a4(4) <= p3_a4;
            for i in 5 to 10 loop p_a4(i) <= p_a4(i - 1); end loop;
            p_mi(4) <= p3_mi;
            for i in 5 to 6 loop p_mi(i) <= p_mi(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture warp;
