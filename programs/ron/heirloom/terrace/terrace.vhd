-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Terrace v0.5.0 -- Heirloom #11, rebuilt 2026-10-21 from WIN_20230305_23_38_39.
--
-- The photo: a strict grid of flat blocks, 25 rows (42.98 lines, the top row
-- stretched to 54) by 2.72 tiles of 6 columns (705.95 px a tile, the columns
-- narrow / wide / narrow / narrow / wide / narrow).  Down every column the
-- colours run yellow, magenta, cyan, white, light grey, grey, dark grey,
-- black; each column is the sequence dropped one row, so the colours stair-
-- step down to the right.  At every tile seam a ~25 px fringe flashes the
-- in-between colours (a dark notch, an orange sliver, a white sliver).
-- Vertical edges soft (sigma ~4.4 px), row edges sharp.
--
-- The patch (fitted, see SUMMARY.md): a frame ramp, clamped at the top, into
-- a quantizer = rows; a line-locked sawtooth with a bent rise and a 21.5 px
-- RC flyback into a second quantizer = columns (the flyback sweeps back
-- through the steps: the seam fringe); colour = palette[(row - column) mod 8].
--
--   K1  Tiles    sawtooth cycles across, 1 .. 8 linear (2.72 = the photo)
--   K2  Rows     rows per frame height, 4 .. 64 linear (25.1)
--   K3  Bend     the column bend 0 .. 5 x the photo's (1) + the camera's bend depth
--   K4  Speed    bipolar row drift (centre still); positive = rows rise, the stairs climb
--   K5  Seam     flyback width 0.4 % .. 50 % of a tile (3.05 %; 50 % = chevrons)
--   K6  Palette  Photo / Primary / Ember / Ocean / Neon / Pastel / Inverse / Mono
--   S7  Climb    the staircase climbs to the right
--   S8  Ramp     the treads become continuous slopes
--   S9  Lean     the columns lean half a tile over the frame
--   S10 Fill     the grey blocks take the source picture (50 %)
--   S11 Source   Video / Osc (Video: camera luma bends the column steps)
--   P12 STEPS    column steps per tile: 0 = flat colour bars, 50 % = the
--                photo's 6, 100 % = 24 (steep fine diagonals)
--
-- Engine: per-field sequencer on one serial shift-add multiplier; per-line
-- engine (row accumulator, Lean phase); per-pixel three DDAs (tile phase,
-- rise c = g u / (1 - f), fall (u - r) / f), corrected at every tile wrap, so
-- the pixel path has one 15 x 13 product.  Generated avid; output
-- conditioning ([1 2 .. 2 1]/32 on Y/U/V at 1080, /16 at 720p/SD, luma 13/16)
-- before the blanking gate.  Latency 28 clocks in every mode.  Colours in
-- the hardware convention (u = Cr, v = Cb), see terrace.py.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.terrace_pkg.all;
use work.all;

architecture terrace of program_top is
    constant C_LAT : integer := 28;   -- matches processing_delay_clks

    constant C_U0  : unsigned(23 downto 0) := to_unsigned(327016, 24);   -- tile phase at x = 0
    constant C_YSV : unsigned(15 downto 0) := to_unsigned(2911, 16);     -- V ramp start, 11.37 lines Q.8

    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- ROM data
    signal r_da, r_fa : unsigned(8 downto 0) := (others => '0');
    signal r_dq : std_logic_vector(8 downto 0) := (others => '0');
    signal r_fq : std_logic_vector(11 downto 0) := (others => '0');
    signal r_pa : unsigned(5 downto 0) := (others => '0');
    signal r_py, r_pu, r_pv : std_logic_vector(9 downto 0) := (others => '0');
    signal r_sa : unsigned(7 downto 0) := to_unsigned(96, 8);
    signal r_sr, r_skr, r_skf : std_logic_vector(15 downto 0) := (others => '0');

    -- timing
    signal t_rw   : unsigned(14 downto 0) := to_unsigned(8738, 15);
    signal t_ys   : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal t_lean : unsigned(15 downto 0) := to_unsigned(7767, 16);
    signal t_hd   : std_logic := '1';

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- controls (glided Q.3: K1, K2, K3, K5, P12)
    type t_g is array(0 to 4) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(251 * 8, 13), to_unsigned(360 * 8, 13), to_unsigned(458 * 8, 13),
                         to_unsigned(386 * 8, 13), to_unsigned(512 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');
    signal s_s4 : std_logic := '0';
    signal s_m4 : unsigned(8 downto 0) := (others => '0');
    signal s_pal, f_pal : unsigned(2 downto 0) := (others => '0');
    signal f_climb, f_ramp, f_lean, f_fill, f_osc : std_logic := '0';

    -- serial multiplier
    signal q_p : unsigned(4 downto 0) := (others => '0');
    signal q_ld, q_st : std_logic := '0';
    signal q_wr : std_logic_vector(1 to 20) := (others => '0');
    signal m_a : unsigned(23 downto 0) := (others => '0');
    signal m_b : unsigned(15 downto 0) := (others => '0');
    signal m_h : unsigned(23 downto 0) := (others => '0');
    signal m_l : unsigned(15 downto 0) := (others => '0');

    -- per-field products (power up near the defaults at 1080p)
    signal f_k1l : unsigned(20 downto 0) := (others => '0');
    signal f_k2l : unsigned(23 downto 0) := (others => '0');
    signal f_k3sq, f_psq : unsigned(19 downto 0) := (others => '0');
    signal f_s4sq  : unsigned(17 downto 0) := (others => '0');
    signal f_inc   : unsigned(17 downto 0) := to_unsigned(23790, 18);
    signal f_kv    : unsigned(15 downto 0) := to_unsigned(24423, 16);
    signal f_kvl   : unsigned(18 downto 0) := to_unsigned(24423, 19);
    signal f_yskv  : unsigned(19 downto 0) := to_unsigned(277715, 20);
    signal f_gr    : unsigned(13 downto 0) := to_unsigned(1584, 14);
    signal f_gb    : unsigned(14 downto 0) := to_unsigned(1536, 15);
    signal f_incc  : unsigned(17 downto 0) := (others => '0');
    signal f_incf  : unsigned(19 downto 0) := (others => '0');
    signal f_c0    : unsigned(17 downto 0) := (others => '0');
    signal f_f0m   : unsigned(25 downto 0) := (others => '0');
    signal f_leanc : unsigned(15 downto 0) := (others => '0');
    signal f_leanf : unsigned(17 downto 0) := (others => '0');
    signal f_am    : unsigned(10 downto 0) := (others => '0');
    signal f_gvv   : unsigned(13 downto 0) := (others => '0');
    signal f_spd   : unsigned(15 downto 0) := (others => '0');
    -- derived per-field values
    signal f_tiles : unsigned(10 downto 0) := to_unsigned(697, 11);
    signal f_rows  : unsigned(13 downto 0) := to_unsigned(6439, 14);
    signal f_beta  : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal f_g     : unsigned(12 downto 0) := to_unsigned(1536, 13);
    signal f_va    : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal f_m     : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal f_r16, f_kr, f_kf : unsigned(15 downto 0) := (others => '0');
    signal f_rmu   : unsigned(23 downto 0) := (others => '0');
    signal f_leanu : unsigned(15 downto 0) := (others => '0');
    signal f_g13, f_gb13 : unsigned(14 downto 0) := (others => '0');
    signal f_gvx   : unsigned(13 downto 0) := (others => '0');
    signal f_gr18  : unsigned(23 downto 0) := (others => '0');
    signal f_kf18  : unsigned(25 downto 0) := (others => '0');
    signal f_inccw : signed(24 downto 0) := (others => '0');
    signal f_incfw : signed(27 downto 0) := (others => '0');
    signal f_leancw : signed(24 downto 0) := (others => '0');
    signal f_leanfw : signed(27 downto 0) := (others => '0');
    signal f_f0    : signed(27 downto 0) := (others => '0');
    signal f_o     : unsigned(22 downto 0) := (others => '0');     -- row offset, Q.20 mod 8

    -- per-line engine (next line) and committed line values
    signal le_st : unsigned(2 downto 0) := (others => '0');
    signal n_vacc : signed(27 downto 0) := (others => '0');
    signal n_u0 : unsigned(23 downto 0) := C_U0;
    signal n_wr : std_logic := '0';
    signal n_c0 : signed(24 downto 0) := (others => '0');
    signal n_f0 : signed(27 downto 0) := (others => '0');
    signal n_lv : unsigned(22 downto 0) := (others => '0');
    signal n_uq0 : unsigned(24 downto 0) := (others => '0');
    signal l_v : unsigned(22 downto 0) := (others => '0');
    signal l_u0 : unsigned(23 downto 0) := C_U0;
    signal l_uq0 : unsigned(24 downto 0) := (others => '0');
    signal l_c0 : signed(24 downto 0) := (others => '0');
    signal l_f0 : signed(27 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- Osc source (for Fill)
    signal s_ox, s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_oty, s_otu, s_otv : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- beam DDAs
    signal b_u  : unsigned(23 downto 0) := (others => '0');
    signal b_uq : unsigned(24 downto 0) := (others => '0');
    signal b_c  : signed(24 downto 0) := (others => '0');
    signal b_f  : signed(27 downto 0) := (others => '0');

    -- pixel path
    type t_sl is array (natural range <>) of std_logic;
    type t_s25 is array (natural range <>) of signed(24 downto 0);
    signal p_rise : t_sl(1 to 8) := (others => '0');
    signal p_c : t_s25(1 to 8) := (others => (others => '0'));
    signal p3_d : signed(8 downto 0) := (others => '0');
    signal p3_f : unsigned(11 downto 0) := (others => '0');
    signal p4_a : unsigned(14 downto 0) := (others => '0');
    signal p4_b : signed(12 downto 0) := (others => '0');
    signal p5_q0, p5_q1, p5_q2 : signed(18 downto 0) := (others => '0');
    signal p6_s : signed(27 downto 0) := (others => '0');
    signal p6_q2 : signed(18 downto 0) := (others => '0');
    signal p7_prod : signed(27 downto 0) := (others => '0');
    signal p8_ch : signed(26 downto 0) := (others => '0');
    signal p9_ch : signed(26 downto 0) := (others => '0');
    -- video bend tap
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    signal v_y4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal v_y8 : t_u8(0 to 7) := (others => (others => '0'));
    signal v_box : unsigned(10 downto 0) := (others => '0');
    signal v_lb : unsigned(7 downto 0) := (others => '0');
    signal v_q0, v_q1 : unsigned(17 downto 0) := (others => '0');
    signal v_vt : unsigned(21 downto 0) := (others => '0');
    -- colour
    signal p_grey : t_sl(10 to 12) := (others => '0');
    signal p12_y, p12_u, p12_v : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p12_sy, p12_su, p12_sv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal x13_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal x13_u, x13_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cy : t_u10(0 to 15) := (others => to_unsigned(64, 10));
    signal s_cu, s_cv : t_u10(0 to 15) := (others => to_unsigned(512, 10));
    signal s_by16, s_by16d : unsigned(13 downto 0) := to_unsigned(1024, 14);
    signal s_bu16, s_bu16d, s_bv16, s_bv16d : unsigned(13 downto 0) := to_unsigned(8192, 14);
    signal s_by8, s_by8d : unsigned(12 downto 0) := to_unsigned(512, 13);
    signal s_bu8, s_bu8d, s_bv8, s_bv8d : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_oy16, s_ou16, s_ov16 : unsigned(14 downto 0) := (others => '0');
    signal s_oy8, s_ou8, s_ov8 : unsigned(13 downto 0) := (others => '0');
    signal s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25, s_y26, s_y27, s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25, s_u26, s_v26, s_u27, s_v27, s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs (registered address and output; a fabric copy follows each)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_dq <= C_DB(to_integer(r_da));
            r_fq <= C_FB(to_integer(r_fa));
            r_py <= C_PAL_Y(to_integer(r_pa)); r_pu <= C_PAL_U(to_integer(r_pa)); r_pv <= C_PAL_V(to_integer(r_pa));
            r_sr <= C_SR(to_integer(r_sa)); r_skr <= C_SKR(to_integer(r_sa)); r_skf <= C_SKF(to_integer(r_sa));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one serial multiplier)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_id : integer range 0 to 15;
        variable v_x  : signed(10 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_p  : integer range 0 to 31;
        variable v_ph : integer range 0 to 31;
        variable v_s  : unsigned(24 downto 0);
        variable v_pr : unsigned(39 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_t23 : unsigned(22 downto 0);
        variable v_t26 : unsigned(25 downto 0);
    begin
        if rising_edge(clk) then
            v_id := to_integer(unsigned(registers_in(8)(3 downto 0)));
            t_rw <= unsigned(C_T_RW(v_id)); t_ys <= unsigned(C_T_YS(v_id)); t_lean <= unsigned(C_T_LEAN(v_id));
            case v_id is
                when 0 | 4 | 5 | 6 | 8 | 12 | 14 => t_hd <= '0';
                when others => t_hd <= '1';
            end case;

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

            -- K6 palette (8 labels; a value < 8 is the firmware's label index)
            v_k := unsigned(registers_in(5));
            if v_k < 8 then s_pal <= v_k(2 downto 0); else s_pal <= v_k(9 downto 7); end if;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                -- K4 Speed, bipolar, deadband 8
                v_x := signed(resize(unsigned(registers_in(3)), 11)) - 512;
                if v_x > 7 then s_s4 <= '0'; v_u := unsigned(v_x) - 8; s_m4 <= v_u(8 downto 0);
                elsif v_x < -7 then s_s4 <= '1'; v_u := unsigned(-v_x) - 8; s_m4 <= v_u(8 downto 0);
                else s_s4 <= '0'; s_m4 <= (others => '0'); end if;
                f_pal <= s_pal;
                f_climb <= registers_in(6)(0);
                f_ramp  <= registers_in(6)(1);
                f_lean  <= registers_in(6)(2);
                f_fill  <= registers_in(6)(3);
                f_osc   <= registers_in(6)(4);
                -- row drift: positive = rows rise and the stairs climb (offset grows), mod 8 rows
                if s_s4 = '1' then f_o <= f_o - resize(f_spd, 23); else f_o <= f_o + resize(f_spd, 23); end if;
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
                        if gl_d > 31 or gl_d < -31 then gl_n <= unsigned(v_ng(12 downto 0)); else gl_n <= gl_k; end if;
                        if gl_d > 31 or gl_d < -31 or (gl_d /= 0 and (gl_k(12 downto 3) = 0 or gl_k(12 downto 3) = 1023)) then
                            gl_w <= '1';
                        else
                            gl_w <= '0';
                        end if;
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

            -- seam ROM address (glided K5), then fabric copies
            r_sa <= s_g(3)(12 downto 5);
            f_r16 <= unsigned(r_sr); f_kr <= unsigned(r_skr); f_kf <= unsigned(r_skf);

            -- knob curves and derived constants (settle long before the sequencer reads them)
            f_tiles <= to_unsigned(256, 11) + resize(f_k1l(20 downto 10), 11);               -- 1 + 7 k1 (Q.8)
            f_rows <= to_unsigned(1024, 14) + resize(f_k2l(23 downto 10), 14);                -- 4 + 60 k2 (Q.8)
            v_t23 := shift_left(resize(f_k3sq, 23), 2) + resize(f_k3sq, 23);              -- 5 k3^2
            f_beta <= resize(v_t23(22 downto 12), 11);
            v_t26 := shift_left(resize(f_psq, 26), 4) + shift_left(resize(f_psq, 26), 3);  -- 24 p^2
            f_g <= resize(v_t26(25 downto 12), 13);
            f_va <= to_unsigned(64, 10) + resize(shift_right(resize(f_beta, 13) + shift_left(resize(f_beta, 13), 1), 2), 10);
            -- the camera bend doubles toward Steps 0 so the picture still shows on flat bars:
            -- m = max(1, 2 - g / 6), Q.8
            v_t26 := shift_left(resize(f_g, 26), 5) + shift_left(resize(f_g, 26), 3) + shift_left(resize(f_g, 26), 1) + resize(f_g, 26);
            if v_t26(25 downto 8) >= 256 then f_m <= to_unsigned(256, 10);
            else f_m <= to_unsigned(512, 10) - resize(v_t26(15 downto 8), 10); end if;
            f_rmu <= (f_r16 & x"00") - C_U0;
            if f_lean = '1' then f_leanu <= t_lean; else f_leanu <= (others => '0'); end if;
            f_g13 <= resize(f_g, 15);
            f_gb13 <= f_gb;
            if f_osc = '1' then f_gvx <= (others => '0'); else f_gvx <= f_gvv; end if;
            f_gr18 <= f_gr & "0000000000";
            f_kf18 <= f_kf & "0000000000";
            f_inccw <= signed(resize(f_incc, 25)) - signed('0' & f_gr18);
            f_incfw <= signed(resize(f_incf, 28)) - signed(resize(f_kf18, 28));
            f_leancw <= signed(resize(f_leanc, 25)) - signed('0' & f_gr18);
            f_leanfw <= signed(resize(f_leanf, 28)) - signed(resize(f_kf18, 28));
            f_f0 <= -signed(resize(f_f0m, 28));

            -- serial products: slot p = zc/32 (1..20), phase 0 load, 1..16 step, 20 store; the strobes and
            -- the slot number are registered from zc first (one clock late, same order)
            v_p  := to_integer(s_zc(9 downto 5));
            v_ph := to_integer(s_zc(4 downto 0));
            q_p <= s_zc(9 downto 5);
            if s_zone = '1' and v_p >= 1 and v_p <= 20 and v_ph = 0 then q_ld <= '1'; else q_ld <= '0'; end if;
            if s_zone = '1' and v_p >= 1 and v_p <= 20 and v_ph >= 1 and v_ph <= 16 then q_st <= '1'; else q_st <= '0'; end if;
            for k in 1 to 20 loop
                if s_zone = '1' and v_p = k and v_ph = 20 then q_wr(k) <= '1'; else q_wr(k) <= '0'; end if;
            end loop;
            if q_ld = '1' then
                m_h <= (others => '0'); m_l <= (others => '0');
                case to_integer(q_p) is
                    when 1  => m_a <= resize(s_g(0)(12 downto 3), 24); m_b <= to_unsigned(1792, 16);
                    when 2  => m_a <= resize(s_g(1)(12 downto 3), 24); m_b <= to_unsigned(15375, 16);
                    when 3  => m_a <= resize(s_g(2)(12 downto 3), 24); m_b <= resize(s_g(2)(12 downto 3), 16);
                    when 4  => m_a <= resize(s_g(4)(12 downto 3), 24); m_b <= resize(s_g(4)(12 downto 3), 16);
                    when 5  => m_a <= resize(s_m4, 24);                m_b <= resize(s_m4, 16);
                    when 6  => m_a <= resize(f_tiles, 24);             m_b <= resize(t_rw, 16);
                    when 7  => m_a <= resize(f_rows, 24);              m_b <= to_unsigned(971, 16);
                    when 8  => m_a <= resize(f_kv, 24);                m_b <= resize(t_ys, 16);
                    when 9  => m_a <= resize(f_kv, 24);                m_b <= C_YSV;
                    when 10 => m_a <= resize(f_g, 24);                 m_b <= f_kr;
                    when 11 => m_a <= resize(f_g, 24);                 m_b <= resize(f_beta, 16);
                    when 12 => m_a <= resize(f_inc, 24);               m_b <= resize(f_gr, 16);
                    when 13 => m_a <= resize(f_inc, 24);               m_b <= f_kf;
                    when 14 => m_a <= C_U0;                            m_b <= resize(f_gr, 16);
                    when 15 => m_a <= f_rmu;                           m_b <= f_kf;
                    when 16 => m_a <= resize(f_leanu, 24);             m_b <= resize(f_gr, 16);
                    when 17 => m_a <= resize(f_leanu, 24);             m_b <= f_kf;
                    when 18 => m_a <= resize(f_va, 24);                m_b <= resize(f_m, 16);
                    when 19 => m_a <= resize(f_s4sq, 24);              m_b <= to_unsigned(34953, 16);
                    when others => m_a <= resize(f_am, 24);            m_b <= to_unsigned(1197, 16);
                end case;
            end if;
            if q_st = '1' then
                if m_b(0) = '1' then v_s := resize(m_h, 25) + resize(m_a, 25); else v_s := resize(m_h, 25); end if;
                m_h <= v_s(24 downto 1);
                m_l <= v_s(0) & m_l(15 downto 1);
                m_b <= '0' & m_b(15 downto 1);
            end if;
            v_pr := m_h & m_l;
            if q_wr(1)  = '1' then f_k1l   <= v_pr(20 downto 0); end if;
            if q_wr(2)  = '1' then f_k2l   <= v_pr(23 downto 0); end if;
            if q_wr(3)  = '1' then f_k3sq  <= v_pr(19 downto 0); end if;
            if q_wr(4)  = '1' then f_psq   <= v_pr(19 downto 0); end if;
            if q_wr(5)  = '1' then f_s4sq  <= v_pr(17 downto 0); end if;
            if q_wr(6)  = '1' then f_inc   <= v_pr(25 downto 8); end if;
            if q_wr(7)  = '1' then f_kv    <= v_pr(23 downto 8); end if;
            if q_wr(8)  = '1' then f_kvl   <= v_pr(26 downto 8); end if;
            if q_wr(9)  = '1' then f_yskv  <= v_pr(27 downto 8); end if;
            if q_wr(10) = '1' then f_gr    <= v_pr(27 downto 14); end if;
            if q_wr(11) = '1' then f_gb    <= v_pr(22 downto 8); end if;
            if q_wr(12) = '1' then f_incc  <= v_pr(31 downto 14); end if;
            if q_wr(13) = '1' then f_incf  <= v_pr(33 downto 14); end if;
            if q_wr(14) = '1' then f_c0    <= v_pr(31 downto 14); end if;
            if q_wr(15) = '1' then f_f0m   <= v_pr(39 downto 14); end if;
            if q_wr(16) = '1' then f_leanc <= v_pr(29 downto 14); end if;
            if q_wr(17) = '1' then f_leanf <= v_pr(31 downto 14); end if;
            if q_wr(18) = '1' then f_am    <= v_pr(18 downto 8); end if;
            if q_wr(19) = '1' then f_spd   <= v_pr(33 downto 18); end if;
            if q_wr(20) = '1' then f_gvv   <= v_pr(21 downto 8); end if;

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
    -- Per-line engine: the next line's row value and Lean phase, advanced at
    -- the source avid fall (line 0 late in the vblank zone); committed at the
    -- source avid rise
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_t : unsigned(24 downto 0);
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 900 then
                n_vacc <= signed(resize(f_o, 28)) - signed(resize(f_yskv, 28));
                n_u0 <= C_U0; n_c0 <= signed(resize(f_c0, 25)); n_f0 <= f_f0;
                le_st <= to_unsigned(3, 3);
            elsif data_in.avid = '0' and s_avq = '1' then
                le_st <= to_unsigned(1, 3);
            elsif le_st /= 0 then
                if le_st = 4 then le_st <= (others => '0'); else le_st <= le_st + 1; end if;
            end if;
            case to_integer(le_st) is
                when 1 =>
                    n_vacc <= n_vacc + signed(resize(f_kvl, 28));
                    v_t := resize(n_u0, 25) + resize(f_leanu, 25);
                    n_u0 <= v_t(23 downto 0); n_wr <= v_t(24);
                when 2 =>
                    if n_wr = '1' then n_c0 <= n_c0 + f_leancw; n_f0 <= n_f0 + f_leanfw;
                    else n_c0 <= n_c0 + signed(resize(f_leanc, 25)); n_f0 <= n_f0 + signed(resize(f_leanf, 28)); end if;
                when 3 =>
                    if n_vacc > signed(resize(f_o, 28)) then n_lv <= unsigned(n_vacc(22 downto 0)); else n_lv <= f_o; end if;
                    n_uq0 <= resize(n_u0, 25) + resize(f_inc, 25);
                when others => null;
            end case;
            if data_in.avid = '1' and s_avq = '0' then
                l_v <= n_lv; l_u0 <= n_u0; l_uq0 <= n_uq0; l_c0 <= n_c0; l_f0 <= n_f0;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Osc source for Fill (folded triangles of the beam position and time)
    ---------------------------------------------------------------------------
    p_src : process(clk)
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then s_ox <= s_ox + 3; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 5; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zgo = '1' then s_ot <= s_ot + 3; end if;
            s_oty <= to_unsigned(64, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot) & '0', 10)
                     + resize(f_tri8(s_ox(9 downto 2) + s_ot), 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot), 10);
            s_otu <= to_unsigned(320, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot + s_ot) & '0', 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot + s_ot), 10);
            s_otv <= to_unsigned(320, 10) + resize(f_tri8(s_oy(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) + s_ot), 10);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam: tile phase u, rise c = g u / (1 - f) and fall (u - r) / f, all
    -- corrected at every tile wrap (b_uq = u + inc, one step ahead)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_u <= l_u0; b_uq <= l_uq0; b_c <= l_c0; b_f <= l_f0;
            elsif g_avid = '1' then
                b_u <= b_uq(23 downto 0);
                b_uq <= resize(b_uq(23 downto 0), 25) + resize(f_inc, 25);
                if b_uq(24) = '1' then
                    b_c <= b_c + f_inccw; b_f <= b_f + f_incfw;
                else
                    b_c <= b_c + signed(resize(f_incc, 25)); b_f <= b_f + signed(resize(f_incf, 28));
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ch : signed(26 downto 0);
        variable v_v8, v_c8, v_d8 : unsigned(10 downto 0);
        variable v_i3 : unsigned(2 downto 0);
        variable v_yy : signed(10 downto 0);
        variable v_m  : unsigned(10 downto 0);
        variable v_y  : unsigned(9 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: ROM addresses (bend by tile phase, fall by fr), rise flag
            r_da <= b_u(23 downto 15);
            if b_f(27) = '0' and b_f(26 downto 18) /= 0 then r_fa <= (others => '1');
            else r_fa <= unsigned(b_f(17 downto 9)); end if;
            p_rise(1) <= b_f(27);
            p_c(1) <= b_c;
            -- 2: ROM read; 3: fabric copy
            p3_d <= signed(r_dq); p3_f <= unsigned(r_fq);
            -- 4: product operands: g.beta x D (rise) or g x F (fall)
            if p_rise(3) = '1' then
                p4_a <= f_gb13; p4_b <= resize(p3_d, 13);
            else
                p4_a <= f_g13; p4_b <= signed('0' & p3_f);
            end if;
            -- 5: three 5 x 13 partial products
            p5_q0 <= signed('0' & p4_a(4 downto 0)) * p4_b;
            p5_q1 <= signed('0' & p4_a(9 downto 5)) * p4_b;
            p5_q2 <= signed('0' & p4_a(14 downto 10)) * p4_b;
            -- 6, 7: sum (Q.20)
            p6_s <= resize(p5_q0, 28) + shift_left(resize(p5_q1, 28), 5);
            p6_q2 <= p5_q2;
            p7_prod <= p6_s + shift_left(resize(p6_q2, 28), 10);
            -- 8: rise adds c (Q.18)
            if p_rise(7) = '1' then
                p8_ch <= resize(p_c(7), 27) + resize(shift_right(p7_prod, 2), 27);
            else
                p8_ch <= resize(shift_right(p7_prod, 2), 27);
            end if;
            -- 9: + the source's bend
            p9_ch <= p8_ch + signed(resize(v_vt, 27));
            -- 10: colour index and palette address
            v_ch := p9_ch;
            v_v8 := l_v(22 downto 12);
            v_c8 := unsigned(v_ch(20 downto 10));
            if f_ramp = '1' then
                if f_climb = '1' then v_d8 := v_v8 + v_c8; else v_d8 := v_v8 - v_c8; end if;
                v_i3 := v_d8(10 downto 8);
            else
                if f_climb = '1' then v_i3 := v_v8(10 downto 8) + v_c8(10 downto 8);
                else v_i3 := v_v8(10 downto 8) - v_c8(10 downto 8); end if;
            end if;
            r_pa <= f_pal & v_i3;
            p_grey(10) <= v_i3(2);
            -- 11: palette ROM; 12: fabric copy, fill source
            p12_y <= unsigned(r_py); p12_u <= unsigned(r_pu); p12_v <= unsigned(r_pv);
            if f_osc = '1' then
                p12_sy <= s_oty; p12_su <= s_otu; p12_sv <= s_otv;
            else
                p12_sy <= unsigned(data_in.y); p12_su <= unsigned(data_in.u); p12_sv <= unsigned(data_in.v);
            end if;
            -- 13: fill (grey half of the palette takes the source 50 %); neutral outside the active line
            if s_gav_sr(11) = '0' then
                x13_y <= to_unsigned(64, 10); x13_u <= to_unsigned(512, 10); x13_v <= to_unsigned(512, 10);
            elsif f_fill = '1' and p_grey(12) = '1' then
                v_m := resize(p12_y, 11) + resize(p12_sy, 11); x13_y <= v_m(10 downto 1);
                v_m := resize(p12_u, 11) + resize(p12_su, 11); x13_u <= v_m(10 downto 1);
                v_m := resize(p12_v, 11) + resize(p12_sv, 11); x13_v <= v_m(10 downto 1);
            else
                x13_y <= p12_y; x13_u <= p12_u; x13_v <= p12_v;
            end if;

            -- 14..23: soft edges = output conditioning: two adjacent 16-sums (1080) or 8-sums
            -- (720p / SD, taps 4..11 so both are centred alike) on Y, U and V; centre at stage 23
            s_cy(0) <= x13_y; s_cu(0) <= x13_u; s_cv(0) <= x13_v;
            for i in 1 to 15 loop s_cy(i) <= s_cy(i - 1); s_cu(i) <= s_cu(i - 1); s_cv(i) <= s_cv(i - 1); end loop;
            s_by16 <= s_by16 + x13_y - s_cy(15); s_bu16 <= s_bu16 + x13_u - s_cu(15); s_bv16 <= s_bv16 + x13_v - s_cv(15);
            s_by8 <= s_by8 + s_cy(3) - s_cy(11); s_bu8 <= s_bu8 + s_cu(3) - s_cu(11); s_bv8 <= s_bv8 + s_cv(3) - s_cv(11);
            s_by16d <= s_by16; s_bu16d <= s_bu16; s_bv16d <= s_bv16;
            s_by8d <= s_by8; s_bu8d <= s_bu8; s_bv8d <= s_bv8;
            s_oy16 <= resize(s_by16, 15) + s_by16d; s_ou16 <= resize(s_bu16, 15) + s_bu16d; s_ov16 <= resize(s_bv16, 15) + s_bv16d;
            s_oy8 <= resize(s_by8, 14) + s_by8d; s_ou8 <= resize(s_bu8, 14) + s_bu8d; s_ov8 <= resize(s_bv8, 14) + s_bv8d;
            -- 24: normalise
            if t_hd = '1' then
                s_y24 <= s_oy16(14 downto 5); s_u24 <= s_ou16(14 downto 5); s_v24 <= s_ov16(14 downto 5);
            else
                s_y24 <= s_oy8(13 downto 4); s_u24 <= s_ou8(13 downto 4); s_v24 <= s_ov8(13 downto 4);
            end if;
            -- 25: luma toward black (white 940 -> 777); blanking gate
            if s_y24 > 64 then v_dl := s_y24 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(23) = '1' then
                s_y25 <= s_y24 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u25 <= s_u24; s_v25 <= s_v24;
            else
                s_y25 <= to_unsigned(64, 10); s_u25 <= to_unsigned(512, 10); s_v25 <= to_unsigned(512, 10);
            end if;
            -- 26..28: pad to the 28-clock latency
            s_y26 <= s_y25; s_u26 <= s_u25; s_v26 <= s_v25;
            s_y27 <= s_y26; s_u27 <= s_u26; s_v27 <= s_v26;
            s_y28 <= s_y27; s_u28 <= s_u27; s_v28 <= s_v27;

            -- video bend tap (a few px early: invisible in a bend): unipolar luma, 8-px box, x gvv
            v_y4 <= unsigned(data_in.y);
            if v_y4 <= 64 then v_y := (others => '0'); else v_y := v_y4 - 64; end if;
            if v_y(9 downto 2) > 219 then v_y8(0) <= to_unsigned(219, 8); else v_y8(0) <= v_y(9 downto 2); end if;
            for i in 1 to 7 loop v_y8(i) <= v_y8(i - 1); end loop;
            v_box <= v_box + v_y8(0) - v_y8(7);
            v_lb <= v_box(10 downto 3);
            v_q0 <= f_gvx * v_lb(3 downto 0);
            v_q1 <= f_gvx * v_lb(7 downto 4);
            v_vt <= resize(v_q0, 22) + shift_left(resize(v_q1, 22), 4);

            -- delays
            for i in 2 to 8 loop p_rise(i) <= p_rise(i - 1); p_c(i) <= p_c(i - 1); end loop;
            for i in 11 to 12 loop p_grey(i) <= p_grey(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(s_u28);
    data_out.v       <= std_logic_vector(s_v28);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture terrace;
