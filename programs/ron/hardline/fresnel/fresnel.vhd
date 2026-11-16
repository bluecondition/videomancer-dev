-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Fresnel v0.2.0 -- Hardline.  The Fresnel zone plate, and the moire of two zone plates: two displaced
-- plates beat into straight, evenly spaced stripes; two plates of different scale beat into a third,
-- phantom plate.  Black and white only (U = V = 512 on every pixel).
--
--   PHI_p(x, y) = s_p (a dx^2 + b dx dy + c dy^2) +- pump + camera       (dx, dy) from the plate's centre
--   a, c = k0 ((1 - fm) +- fm cos 2t),  b = k0 2 fm sin 2t               k0 = 1 / (2 rz^2), t = Turn, fm = Form
--   ink_p = |frac(PHI_p + off_p) - 1/2| < duty / 2                       Pair: ink_a xor ink_b
--
--   K1  X        plate centre, +-4096 virtual px (the second plate sits at the mirror point)
--   K2  Y        plate centre, +-2048 virtual px
--   K3  Period   zone radius rz 48 .. 400 virtual px, exponential
--   K4  Form     round (a 9:8 ellipse) .. ellipse .. parallel chirp bars .. saddle (hyperbolas)
--   K5  Turn     rotates the form's axes, a full half-turn
--   K6  Camera   the luma (64-px box, vertical 1/16 IIR) adds to PHI: 0 .. 3.4 cycles at full white
--   S7  Glide    Off = zero easing / On = 1/16 of the gap per field
--   S8  Pair     Single / Pair (two plates, XOR)
--   S9  Ghosts   Clean (anti-aliased; grey where the zones outrun the raster) / Raw (binary, sampled on a
--                coarse lattice of rz / 10, so the plate aliases into a constellation of ghost plates)
--   S10 Weight   Bold (equal zones) / Fine (thin rings, 1/6 duty, at least 2 pixels wide)
--   S11 Source   Video / Osc
--   P12 SPEED    bipolar, centre = still (+-24 counts), the Hypnos shape: the ring phase runs on by
--                2^-12 .. 2^-2 cycle per field, wrapping modulo one ring.  Up: plate a's rings stream
--                outward, plate b's inward (opposite phases, so the beat stripes march); Pair halves the
--                step so the beat moves at the decoded speed.  Glide eases the speed (1/8), never the phase.
--
-- v0.2.0 (2026-11-16, Ron): P12 Pump -> Speed (self-motion; the old pump's -+25 % plate-scale split is gone).
--
-- Each plate is an exact quadratic field run by second differences: per pixel  S += Dx; Dx += DDx  (and the
-- vertical gradient Gy += GXY), per line  S0 += E; E += EE; Dx0 += GXY; Gy0 += EE.  The seeds of both
-- plates, the per-line steps and the first values of every line are computed by a small microcoded
-- accumulator machine (register file in block RAM, one serial multiplier; microcode in fresnel.py).
-- The anti-aliasing is the analytic box filter of the band: its width |grad PHI| comes from the DDAs' own
-- first differences (plus the camera's finite differences), rounded up to a power of two so the divide is
-- a shift (edges 1 .. 2 pixels); past 1/4 cycle per pixel the band fades to flat grey by 1/2 cycle per
-- pixel, and where BOTH plates have faded the beat PHI_b - PHI_a is drawn in their place as crisp stripes.
--
-- Sections (the integer model .hardline_work/rb_fresnel/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing-ID steps, vblank zone, width, generated avid, sync delay
--   [B] CONTROLS                         +-1 hold, shared glide (7 channels), switches
--   [C] FIELD ENGINE                     microcoded machine (per field, per line), beam DDAs
--   [D] SOURCE FIELD                     luma or the Osc lattice -> 64-px box -> vertical IIR (quarter res)
--   [E] COVERAGE                         three box-filter units (plate a, plate b, beat), XOR blend
--   [F] OUTPUT CONDITIONING              luma [1 2 1]/4, 13/16 toward black, blanking gate
--
-- Stage numbering: a register "at stage S" holds pixel n at T + S + n (T = the g_start clock).  Pixel n sits
-- at x = x0 + n xs half virtual px; output column j is the [1 2 1] filter centred on pixel j + 4, and the
-- generated avid lands 4 clocks after the delayed source avid, so the picture centre is the screen centre.
-- s_gav_sr(S - 2) flags pixels 0 .. W - 1 at stage S.  One latency (32) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.fresnel_pkg.all;
use work.all;

architecture fresnel of program_top is
    constant C_LAT : integer := 32;   -- = processing_delay_clks

    -- TOML defaults: the glides start there
    constant C_DEF_K1  : integer := 526;
    constant C_DEF_K2  : integer := 505;
    constant C_DEF_K3  : integer := 600;
    constant C_DEF_K4  : integer := 0;
    constant C_DEF_K5  : integer := 0;
    constant C_DEF_K6  : integer := 1023;
    constant C_DEF_P12 : integer := 850;

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

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ysr, s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
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
    signal s_st       : std_logic_vector(0 to 11) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] knobs: held raw values and glide accumulators (Q6), K1 K2 K3 K4 K5 K6 P12; both arrays rotate
    -- through slot 0 while the glide runs and are back in place when it ends
    type t_g16 is array (0 to 6) of unsigned(15 downto 0);
    type t_h10 is array (0 to 6) of unsigned(9 downto 0);
    signal s_ga : t_g16 := (to_unsigned(C_DEF_K1 * 64, 16), to_unsigned(C_DEF_K2 * 64, 16),
                            to_unsigned(C_DEF_K3 * 64, 16), to_unsigned(C_DEF_K4 * 64, 16),
                            to_unsigned(C_DEF_K5 * 64, 16), to_unsigned(C_DEF_K6 * 64, 16),
                            to_unsigned(C_DEF_P12 * 64, 16));
    signal s_gh : t_h10 := (to_unsigned(C_DEF_K1, 10), to_unsigned(C_DEF_K2, 10), to_unsigned(C_DEF_K3, 10),
                            to_unsigned(C_DEF_K4, 10), to_unsigned(C_DEF_K5, 10), to_unsigned(C_DEF_K6, 10),
                            to_unsigned(C_DEF_P12, 10));
    signal s_gd, s_gdl : signed(16 downto 0) := (others => '0');
    signal s_gr   : unsigned(9 downto 0) := (others => '0');
    signal z_gp   : unsigned(2 downto 0) := "000";
    signal z_gc   : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_glide : std_logic := '1';
    signal s_pair, s_raw, s_fine, s_src : std_logic := '0';

    -- [C] ROMs
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q, r_exf : unsigned(15 downto 0) := (others => '0');
    signal sn_a : unsigned(9 downto 0) := (others => '0');
    signal sn_q, sn_qf : unsigned(15 downto 0) := (others => '0');

    -- [C] the field engine: microcode ROM, register file, A / B / ACC
    constant OP_LD  : integer := 1;
    constant OP_LDX : integer := 2;
    constant OP_ST  : integer := 3;
    constant OP_ADD : integer := 4;
    constant OP_MUL : integer := 5;
    constant OP_CLR : integer := 6;
    constant OP_SHR : integer := 7;
    constant OP_J   : integer := 8;
    constant OP_SPC : integer := 9;
    signal mc_q   : std_logic_vector(15 downto 0) := x"9000";
    signal c_pc   : unsigned(8 downto 0) := (others => '0');
    signal c_run, c_kill, c_pl : std_logic := '0';
    signal c_cnt  : unsigned(5 downto 0) := (others => '0');
    signal x_v    : std_logic := '0';
    signal x_op, x_x : unsigned(3 downto 0) := (others => '0');
    signal x_y    : unsigned(7 downto 0) := (others => '0');
    signal x_wa   : unsigned(6 downto 0) := (others => '0');
    signal m_run, h_run, m_last, x_neg : std_logic := '0';
    signal f_a, f_acc : signed(47 downto 0) := (others => '0');
    signal f_b    : signed(17 downto 0) := (others => '0');
    signal rf_ram : t_c_fr_rf := C_FR_RF;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q   : signed(47 downto 0) := (others => '0');
    signal rf_ra  : unsigned(6 downto 0);
    signal rf_we  : std_logic;
    signal t_field, t_lstep, t_lload : std_logic := '0';
    signal s_lo   : unsigned(3 downto 0) := (others => '0');
    signal s_sh   : unsigned(4 downto 0) := (others => '0');
    signal s_sn2  : std_logic := '0';
    signal l_rownext : std_logic := '1';
    signal w_ext  : signed(17 downto 0);

    -- [C] live per-field registers the beam needs (loaded by the engine)
    type t_u32x2 is array (0 to 1) of unsigned(31 downto 0);
    type t_s32x2 is array (0 to 1) of signed(31 downto 0);
    type t_s24x2 is array (0 to 1) of signed(23 downto 0);
    type t_s40x2 is array (0 to 1) of signed(39 downto 0);
    signal c_DDx  : t_s32x2 := (others => (others => '0'));
    signal c_GXY  : t_s24x2 := (others => (others => '0'));
    signal r_P8   : unsigned(8 downto 0) := to_unsigned(64, 9);     -- Raw lattice pitch, 1/8 virtual px
    signal r_bx0  : unsigned(8 downto 0) := (others => '0');

    -- [C] beam DDAs (stage 13)
    signal b_S    : t_u32x2 := (others => (others => '0'));
    signal b_Dx   : t_s40x2 := (others => (others => '0'));
    signal b_Gy   : t_s32x2 := (others => (others => '0'));
    signal b_bx   : unsigned(9 downto 0) := (others => '0');
    signal b_stp  : std_logic := '1';
    signal b_run  : std_logic := '0';

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl, s_sfirst : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_svd : std_logic := '0';
    signal s_col : unsigned(10 downto 0) := (others => '0');
    type t_bx is array (0 to 255) of unsigned(7 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q : unsigned(7 downto 0) := (others => '0');
    signal s_bcnt : unsigned(6 downto 0) := (others => '0');
    signal s_bsum : unsigned(13 downto 0) := (others => '0');
    signal s_bv, s_ev : std_logic := '0';
    signal s_bcol, s_ecol : unsigned(10 downto 0) := (others => '0');
    signal s_eb, s_eq : unsigned(10 downto 0) := (others => '0');
    signal s_rownow : std_logic := '1';
    type t_lb is array (0 to 511) of unsigned(15 downto 0);
    signal lb_ram : t_lb := (others => (others => '0'));
    attribute no_rw_check of lb_ram : signal is true;
    signal lb_q1, lb_q2, lb_wd : unsigned(15 downto 0) := (others => '0');
    type t_dl is array (0 to 511) of signed(7 downto 0);
    signal dl_ram : t_dl := (others => (others => '0'));
    attribute no_rw_check of dl_ram : signal is true;
    signal dl_q, dl_wd : signed(7 downto 0) := (others => '0');
    signal lb_wa : unsigned(8 downto 0) := (others => '0');
    signal lb_we : std_logic := '0';
    signal lb_rc : unsigned(10 downto 0) := (others => '0');
    signal lb_fr7, s8_fr, s9_fr : unsigned(1 downto 0) := "00";
    signal lb_z7  : std_logic := '0';
    signal s8_q1, s8_q0, s9_q0 : unsigned(10 downto 0) := (others => '0');
    signal s9_d   : signed(11 downto 0) := (others => '0');
    signal s10_v  : unsigned(10 downto 0) := (others => '0');
    signal s11_p  : unsigned(18 downto 0) := (others => '0');
    signal s12_cam, s13_cf, s13_cam : unsigned(18 downto 0) := (others => '0');
    signal s13_cdx : signed(15 downto 0) := (others => '0');
    signal s8_dl, s9_dl, s10_dl, s11_dl : signed(7 downto 0) := (others => '0');
    signal s12_dp : signed(16 downto 0) := (others => '0');
    signal s13_cdy : signed(12 downto 0) := (others => '0');

    -- [E] coverage units: 0 = plate a, 1 = plate b, 2 = the beat
    type t_u16x3 is array (0 to 2) of unsigned(15 downto 0);
    type t_s21x3 is array (0 to 2) of signed(20 downto 0);
    type t_u13x3 is array (0 to 2) of unsigned(12 downto 0);
    type t_u12x3 is array (0 to 2) of unsigned(11 downto 0);
    type t_s13x3 is array (0 to 2) of signed(12 downto 0);
    type t_u5x3  is array (0 to 2) of unsigned(4 downto 0);
    type t_u4x3  is array (0 to 2) of unsigned(3 downto 0);
    signal u14_f  : t_u16x3 := (others => (others => '0'));
    signal u14_gx, u14_gy : t_s21x3 := (others => (others => '0'));
    signal u15_ax, u15_ay, u16_mx, u16_mn, u15_t, u16_t, u17_t : t_u12x3 := (others => (others => '0'));
    signal u17_w  : t_u13x3 := (others => (others => '0'));
    signal u18_k, u18_al, u19_al, u20_kap : t_u4x3 := (others => (others => '0'));
    signal u18_d  : t_s13x3 := (others => (others => '0'));
    signal u19_c, u20_cov : t_u5x3 := (others => (others => '0'));
    -- blend
    signal s21_pab : unsigned(9 downto 0) := (others => '0');
    signal s21_kab : unsigned(7 downto 0) := (others => '0');
    signal s21_sab : unsigned(5 downto 0) := (others => '0');
    signal s21_cd8 : signed(5 downto 0) := (others => '0');
    signal s21_ca, s22_ca : unsigned(4 downto 0) := (others => '0');
    signal s22_x  : signed(11 downto 0) := (others => '0');
    signal s22_bt : signed(13 downto 0) := (others => '0');
    signal s23_o  : unsigned(8 downto 0) := (others => '0');
    signal s24_t1 : unsigned(16 downto 0) := (others => '0');
    signal s24_t2 : unsigned(10 downto 0) := (others => '0');

    -- [F] output conditioning
    signal s25_y, s26_y, s27_y, s27_f, s28_y, s29_y : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_FR_EXP(to_integer(r_ex_a));
            r_exf  <= r_ex_q;
            sn_q   <= C_FR_SIN(to_integer(sn_a));
            sn_qf  <= sn_q;
            mc_q   <= C_FR_MC(to_integer(c_pc));
        end if;
    end process p_rom;

    -- the engine's register file: 128 x 48, constants in its start values.  The microcode never reads a
    -- register in the clock it is written (fresnel.py inserts the NOP).
    p_rf : process(clk)
    begin
        if rising_edge(clk) then
            rf_q <= rf_ram(to_integer(rf_ra));
            if rf_we = '1' then rf_ram(to_integer(x_wa)) <= f_acc; end if;
        end if;
    end process p_rf;

    -- box delay: read 64 samples behind the write
    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_sv = '1' then bx_ram(to_integer(bx_wp)) <= s_sl; end if;
        end if;
    end process p_bxram;

    -- luma line store (quarter horizontal resolution): port 1 = the IIR's own read (one address ahead of
    -- its write), port 2 = the pixel path (one address behind it, two clocks after the write)
    p_lbram : process(clk)
    begin
        if rising_edge(clk) then
            lb_q1 <= lb_ram(to_integer(s_col(10 downto 2)));
            lb_q2 <= lb_ram(to_integer(lb_rc(10 downto 2)));
            if lb_we = '1' then lb_ram(to_integer(lb_wa)) <= lb_wd; end if;
        end if;
    end process p_lbram;

    -- the IIR's per-line increment (the camera's vertical gradient)
    p_dlram : process(clk)
    begin
        if rising_edge(clk) then
            dl_q <= dl_ram(to_integer(lb_rc(10 downto 2)));
            if lb_we = '1' then dl_ram(to_integer(lb_wa)) <= dl_wd; end if;
        end if;
    end process p_dlram;

    ---------------------------------------------------------------------------
    -- [A] TIMING AND VIRTUAL COORDINATES + [B] CONTROLS
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_d   : signed(10 downto 0);
        variable v_n   : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            -- [A] timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ysr <= f_vstep_y(s_tid);
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
                s_raw   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_ys    <= s_ysr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- [B] shared glide, five clocks per channel (K1 K2 K3 K4 K5 K6 P12), one small operation each:
            --   0  the knob
            --   1  +-1 hold: the held value follows the knob only when it moves by 2 counts or more
            --   2  gap = held << 6 - accumulator
            --   3  Off, or within 1/4 count: land exactly;  under 4 counts: 1/4 count per field;  else 1/16
            --   4  accumulator += step; both arrays rotate one place
            if s_zgo = '1' then
                z_gp <= "000"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case to_integer(z_gp) is
                    when 0 =>
                        case to_integer(z_gc) is
                            when 0      => s_gr <= unsigned(registers_in(0));
                            when 1      => s_gr <= unsigned(registers_in(1));
                            when 2      => s_gr <= unsigned(registers_in(2));
                            when 3      => s_gr <= unsigned(registers_in(3));
                            when 4      => s_gr <= unsigned(registers_in(4));
                            when 5      => s_gr <= unsigned(registers_in(5));
                            when others => s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "001";
                    when 1 =>
                        v_d := signed(resize(s_gr, 11)) - signed(resize(s_gh(0), 11));
                        if v_d > 1 or v_d < -1 then s_gh(0) <= s_gr; end if;
                        z_gp <= "010";
                    when 2 =>
                        s_gd <= signed(shift_left(resize(s_gh(0), 17), 6)) - signed(resize(s_ga(0), 17));
                        z_gp <= "011";
                    when 3 =>
                        if s_glide = '0' or (s_gd <= 16 and s_gd >= -16) then s_gdl <= s_gd;
                        elsif s_gd < 256 and s_gd > -256 then
                            if s_gd(16) = '1' then s_gdl <= to_signed(-16, 17); else s_gdl <= to_signed(16, 17); end if;
                        elsif z_gc = 6 then s_gdl <= shift_right(s_gd, 3);      -- Speed: 1/8 per field (Hypnos)
                        else s_gdl <= shift_right(s_gd, 4); end if;
                        z_gp <= "100";
                    when others =>
                        v_n := signed(resize(s_ga(0), 17)) + s_gdl;
                        for i in 0 to 5 loop
                            s_ga(i) <= s_ga(i + 1); s_gh(i) <= s_gh(i + 1);
                        end loop;
                        s_ga(6) <= unsigned(v_n(15 downto 0)); s_gh(6) <= s_gh(0);
                        z_gp <= "000";
                        if z_gc = 6 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
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
            s_st <= g_start & s_st(0 to 10);
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
    -- [C] FIELD ENGINE.  A two-stage accumulator machine (instruction set and
    -- microcode: fresnel.py).  D: the instruction at the ROM's output register;
    -- it issues the register-file read.  X: one clock later, its effect.
    --   MUL holds D for 17 clocks (18 shift-add iterations; B's top bit is its sign),
    --   SHR n for n - 1.  A taken jump kills the two instructions behind it.
    -- Three entry points, each restarts the machine whatever it was doing:
    --   FIELD  in the vblank zone, once the glide has run: both plates' seeds, then LLOAD
    --   LSTEP  at every line start (the beam is running): the line DDAs take one step
    --   LLOAD  after the line's last pixel: the next line's first values go to the beam
    ---------------------------------------------------------------------------
    rf_ra <= (c_pl & unsigned(mc_q(5 downto 0))) when mc_q(7 downto 6) = "01" else
             ('0' & unsigned(mc_q(5 downto 1)) & (mc_q(0) xor s_ga(4)(14))) when mc_q(7 downto 6) = "10" else
             ('0' & unsigned(mc_q(5 downto 0)));
    rf_we <= '1' when x_v = '1' and x_op = OP_ST else '0';

    -- outside values for B
    p_ext : process(all)
        variable v_e : unsigned(17 downto 0);
    begin
        v_e := (others => '0');
        case to_integer(x_x) is
            when 0  => v_e := resize(s_ga(0), 18);
            when 1  => v_e := resize(s_ga(1), 18);
            when 2  => v_e := resize(s_ga(2), 18);
            when 3  => v_e := resize(s_ga(3), 18);
            when 4  => v_e := resize(s_ga(4), 18);
            when 5  => v_e := resize(s_ga(5), 18);
            when 6  => v_e := resize(s_ga(6), 18);
            when 7  => v_e := resize(r_exf, 18);
            when 8  => v_e := to_unsigned(65536, 18) + resize(shift_left(resize(s_lo, 8), 3), 18)
                              + resize(shift_left(resize(s_lo, 8), 1), 18) + resize(s_lo, 18);   -- 65536 + 11 lo
            when 9  => v_e := shift_left(to_unsigned(1, 18), to_integer(s_sh));
            when 10 =>
                if s_sn2 = '1' and s_ga(4)(13 downto 4) = 0 then v_e := to_unsigned(32767, 18);   -- sin(pi / 2)
                else v_e := resize(sn_qf(14 downto 0), 18); end if;
            when 11 =>                                    -- first sample left of the centre, 1/16 vpx (Clean)
                case to_integer(s_xs) is
                    when 4      => v_e := to_unsigned(11552, 18);
                    when 3      => v_e := to_unsigned(15384, 18);
                    when others => v_e := to_unsigned(15376, 18);
                end case;
            when 12 => v_e := resize(s_xs & "000000", 18);            -- pixel step, 1/128 vpx
            when 13 => v_e := resize(s_ys(11 downto 1), 18);          -- line step, 1/128 vpx
            when others =>                                -- half the picture width, 1/8 vpx
                if s_xs = 4 then v_e := to_unsigned(5760, 18); else v_e := to_unsigned(7680, 18); end if;
        end case;
        w_ext <= signed(v_e);
    end process p_ext;

    p_eng : process(clk)
        variable v_dv, v_long, v_adv, v_tk, v_neg, v_sub, v_do : std_logic;
        variable v_op : integer range 0 to 15;
        variable v_ax : signed(47 downto 0);
        variable v_s  : signed(48 downto 0);
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 48 then t_field <= '1'; else t_field <= '0'; end if;   -- the glide is done by 37
            t_lstep <= s_st(10);
            t_lload <= s_gav_sr(18) and not s_gav_sr(17);

            -- D stage
            v_op := to_integer(unsigned(mc_q(15 downto 12)));
            if c_run = '1' and c_kill = '0' and c_cnt = 0 then v_dv := '1'; else v_dv := '0'; end if;
            v_long := '0';
            if v_op = OP_MUL then v_long := '1'; end if;
            if v_op = OP_SHR and unsigned(mc_q(5 downto 0)) /= 1 then v_long := '1'; end if;
            if (v_dv = '1' and v_long = '0') or c_cnt = 1 or (c_kill = '1' and c_run = '1') then v_adv := '1'; else v_adv := '0'; end if;
            if v_adv = '1' then c_pc <= c_pc + 1; end if;
            c_kill <= '0';
            if c_cnt /= 0 then c_cnt <= c_cnt - 1; end if;
            x_v <= v_dv;
            case mc_q(10 downto 8) is                     -- the sign of an ADD / MUL, decoded here
                when "000"  => v_neg := '0';
                when "001"  => v_neg := '1';
                when "010"  => v_neg := s_ga(4)(15) xor s_ga(4)(14);         -- cos < 0
                when "011"  => v_neg := not (s_ga(4)(15) xor s_ga(4)(14));
                when "100"  => v_neg := s_ga(4)(15);                         -- sin < 0
                when "101"  => v_neg := c_pl;
                when others => v_neg := not c_pl;
            end case;
            if v_dv = '1' then
                x_op <= unsigned(mc_q(15 downto 12)); x_x <= unsigned(mc_q(11 downto 8)); x_y <= unsigned(mc_q(7 downto 0));
                x_wa <= rf_ra; x_neg <= v_neg;
                if v_op = OP_MUL then c_cnt <= to_unsigned(17, 6); m_run <= '1'; end if;
                if v_op = OP_SHR then c_cnt <= unsigned(mc_q(5 downto 0)) - 1; h_run <= '1'; end if;
            end if;

            -- ACC: one adder for MUL and ADD (a - b as a + not b + 1 on one carry chain), a right shift, a clear.
            -- The subtract control comes straight from two registers: the instruction's sign, and "this is
            -- the multiply's last iteration" (B's top bit is its sign), flagged one clock ahead.
            if m_run = '1' and c_cnt = 1 then m_last <= '1'; else m_last <= '0'; end if;
            v_sub := x_neg xor m_last;
            if v_sub = '1' then v_ax := not f_a; else v_ax := f_a; end if;
            v_s := (f_acc & '1') + (v_ax & v_sub);
            v_do := '0';
            if m_run = '1' then v_do := f_b(0);
            elsif x_v = '1' and x_op = OP_ADD then v_do := '1'; end if;
            if v_do = '1' then f_acc <= v_s(48 downto 1);
            elsif h_run = '1' then f_acc <= f_acc(47) & f_acc(47 downto 1);
            elsif x_v = '1' and x_op = OP_CLR then f_acc <= (others => '0'); end if;
            if m_run = '1' then
                f_a <= f_a(46 downto 0) & '0';
                f_b <= '0' & f_b(17 downto 1);
                if c_cnt = 0 then m_run <= '0'; end if;
            end if;
            if h_run = '1' and c_cnt = 0 then h_run <= '0'; end if;

            -- X stage: the single-clock effects
            if x_v = '1' then
                case to_integer(x_op) is
                    when OP_LD =>
                        if x_x = 0 then f_a <= rf_q; end if;
                        if x_x = 1 then f_b <= rf_q(17 downto 0); end if;
                    when OP_LDX => f_b <= w_ext;
                    when OP_J =>
                        case to_integer(x_x(2 downto 0)) is
                            when 0      => v_tk := '1';
                            when 1      => v_tk := f_acc(47);
                            when 2      => v_tk := not f_acc(47);
                            when 3      => v_tk := s_raw;
                            when 5      => v_tk := s_pair;
                            when others => v_tk := not c_pl; c_pl <= not c_pl;
                        end case;
                        if v_tk = '1' then
                            c_pc <= x_y & '0'; c_kill <= '1'; x_v <= '0';
                            c_cnt <= (others => '0'); m_run <= '0'; h_run <= '0';
                        end if;
                    when OP_SPC =>
                        case to_integer(x_x(2 downto 0)) is
                            when 0 => c_run <= '0'; x_v <= '0'; c_cnt <= (others => '0'); m_run <= '0'; h_run <= '0';
                            when 1 => r_ex_a <= unsigned(f_acc(11 downto 4)); s_lo <= unsigned(f_acc(3 downto 0));
                            when 2 => s_sh <= unsigned(f_acc(4 downto 0));
                            when 3 => sn_a <= s_ga(4)(13 downto 4); s_sn2 <= '0';
                            when 4 => sn_a <= to_unsigned(0, 10) - s_ga(4)(13 downto 4); s_sn2 <= '1';
                            when 5 => l_rownext <= '0';
                            when 6 => l_rownext <= '1';
                            when others => null;
                        end case;
                    when others => null;
                end case;
            end if;

            -- entry points
            if t_field = '1' or t_lstep = '1' or t_lload = '1' then
                if t_field = '1' then c_pc <= to_unsigned(C_MC_FIELD, 9);
                elsif t_lstep = '1' then c_pc <= to_unsigned(C_MC_LSTEP, 9);
                else c_pc <= to_unsigned(C_MC_LLOAD, 9); end if;
                c_run <= '1'; c_kill <= '1'; x_v <= '0'; c_pl <= '0';
                c_cnt <= (others => '0'); m_run <= '0'; h_run <= '0';
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- [C] FIELD ENGINE, beam: second-order DDAs per plate.  The engine leaves
    -- the pre-sample's values in them before the line; they start stepping 10
    -- clocks after g_start, so pixel 0 arrives at stage 13.  Raw: they step only
    -- when the pixel crosses into the next lattice cell.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_bx : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if s_st(10) = '1' then
                b_run <= '1';
                b_bx <= resize(r_bx0, 10);
                b_stp <= '1';
            elsif t_lload = '1' then
                b_run <= '0';
            elsif b_run = '1' then
                if b_stp = '1' then
                    for p in 0 to 1 loop
                        b_S(p)  <= b_S(p) + unsigned(b_Dx(p)(31 downto 0));
                        b_Dx(p) <= b_Dx(p) + resize(c_DDx(p), 40);
                        b_Gy(p) <= b_Gy(p) + resize(c_GXY(p), 32);
                    end loop;
                end if;
                if s_raw = '1' then
                    v_bx := b_bx + resize(s_xs & "00", 10);
                    if v_bx >= resize(r_P8, 10) then
                        b_bx <= v_bx - resize(r_P8, 10); b_stp <= '1';
                    else
                        b_bx <= v_bx; b_stp <= '0';
                    end if;
                else
                    b_stp <= '1';
                end if;
            end if;
            -- loads from the register file (the engine's LD with a beam destination)
            if x_v = '1' and x_op = OP_LD then
                case to_integer(x_x) is
                    when 2 => if c_pl = '0' then b_S(0) <= unsigned(rf_q(31 downto 0)); else b_S(1) <= unsigned(rf_q(31 downto 0)); end if;
                    when 3 => if c_pl = '0' then b_Dx(0) <= rf_q(39 downto 0); else b_Dx(1) <= rf_q(39 downto 0); end if;
                    when 4 => if c_pl = '0' then b_Gy(0) <= rf_q(31 downto 0); else b_Gy(1) <= rf_q(31 downto 0); end if;
                    when 5 => if c_pl = '0' then c_DDx(0) <= rf_q(31 downto 0); else c_DDx(1) <= rf_q(31 downto 0); end if;
                    when 6 => if c_pl = '0' then c_GXY(0) <= rf_q(31 downto 8); else c_GXY(1) <= rf_q(31 downto 8); end if;
                    when 7 => r_P8 <= unsigned(rf_q(8 downto 0));
                    when 8 => r_bx0 <= unsigned(rf_q(8 downto 0));
                    when others => null;
                end case;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> 64-px box, edge-replicated
    -- and reloaded at every line start -> vertical 1/16 IIR in the line store
    -- (quarter horizontal resolution, 11 bits).  Raw: the store takes the box
    -- as it is on the first line of each lattice row and holds it.  The pixel
    -- path reads the store, interpolates and scales by K6: stages 7 .. 13.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_tap : unsigned(7 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_i : signed(12 downto 0);
        variable v_a : signed(12 downto 0);
        variable v_m : signed(13 downto 0);
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

            -- the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid; s_svd <= s_sv;
            if s_sv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;
            if s_sv = '1' and s_svd = '0' then s_rownow <= l_rownext; end if;

            -- box: sum of the last 64 samples; the samples before the line start count as the first one
            if s_sv = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(193, 8); end if;
            if s_bcnt < 64 then v_tap := s_sfirst; else v_tap := bx_q; end if;
            if s_sv = '1' and s_svd = '0' then
                s_sfirst <= s_sl;
                s_bsum <= shift_left(resize(s_sl, 14), 6);
                s_bcnt <= to_unsigned(1, 7);
            elsif s_sv = '1' then
                s_bsum <= s_bsum + s_sl - v_tap;
                if s_bcnt /= 64 then s_bcnt <= s_bcnt + 1; end if;
            end if;
            s_bv <= s_sv; s_bcol <= s_col;

            -- vertical IIR on every fourth column: q += (box - q + 8) >> 4
            s_eb <= s_bsum(13 downto 3); s_eq <= lb_q1(10 downto 0);
            if s_bv = '1' and s_bcol(1 downto 0) = "11" then s_ev <= '1'; else s_ev <= '0'; end if;
            s_ecol <= s_bcol;
            v_d := signed(resize(s_eb, 13)) - signed(resize(s_eq, 13)) + 8;
            v_i := shift_right(v_d, 4);
            v_a := signed(resize(s_eq, 13)) + v_i;
            if s_raw = '1' then
                lb_wd <= resize(s_eb, 16); dl_wd <= (others => '0');
                lb_we <= s_ev and s_rownow;
            else
                lb_wd <= resize(unsigned(v_a(10 downto 0)), 16); dl_wd <= v_i(7 downto 0);
                lb_we <= s_ev;
            end if;
            lb_wa <= s_ecol(10 downto 2);

            -- pixel path.  7: the store's own registers (lb_q2, dl_q), sub-sample position
            if s_st(4) = '1' then lb_rc <= (others => '0'); else lb_rc <= lb_rc + 1; end if;
            lb_fr7 <= lb_rc(1 downto 0);
            if lb_rc = 0 then lb_z7 <= '1'; else lb_z7 <= '0'; end if;
            -- 8: this sample and the one before it
            s8_q1 <= lb_q2(10 downto 0);
            if lb_fr7 = "00" then
                if lb_z7 = '1' then s8_q0 <= lb_q2(10 downto 0); else s8_q0 <= s8_q1; end if;
            end if;
            s8_fr <= lb_fr7;
            s8_dl <= dl_q;
            -- 9, 10: linear interpolation between them
            s9_d <= signed(resize(s8_q1, 12)) - signed(resize(s8_q0, 12));
            s9_q0 <= s8_q0; s9_fr <= s8_fr;
            v_m := (others => '0');
            if s9_fr(1) = '1' then v_m := shift_left(resize(s9_d, 14), 1); end if;
            if s9_fr(0) = '1' then v_m := v_m + resize(s9_d, 14); end if;
            v_a := signed(resize(s9_q0, 13)) + resize(shift_right(v_m, 2), 13);
            s10_v <= unsigned(v_a(10 downto 0));
            -- 11: x camera gain (the multiply has the stage to itself)
            s11_p <= s10_v * s_ga(5)(15 downto 8);
            -- 12: Osc is fed at twice the camera's level.  Past the last pixel of the line the value holds
            -- (the store has nothing there, and pixel W still feeds the last column's filter).
            if s_gav_sr(9) = '1' then
                if s_src = '1' then s12_cam <= s11_p; else s12_cam <= '0' & s11_p(18 downto 1); end if;
            end if;
            -- 13: the phase offset (held over a lattice cell in Raw) and its horizontal difference
            if b_stp = '1' then s13_cam <= s12_cam; end if;
            s13_cf <= s12_cam;
            s13_cdx <= resize(shift_right(signed(resize(s12_cam, 20)) - signed(resize(s13_cf, 20)), 4), 16);
            -- the vertical difference: the IIR's increment x gain
            s9_dl <= s8_dl; s10_dl <= s9_dl; s11_dl <= s10_dl;
            s12_dp <= s11_dl * signed(resize(s_ga(5)(15 downto 8), 9));
            if s_gav_sr(10) = '1' then
                if s_src = '1' then s13_cdy <= resize(shift_right(s12_dp, 4), 13);
                else s13_cdy <= resize(shift_right(s12_dp, 5), 13); end if;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] COVERAGE (stages 14 .. 20), three identical units: plate a, plate b,
    -- the beat.  f = the phase (Q16), (gx, gy) = its gradient per pixel (Q12).
    --   w   = |grad| ~ max + 3/8 min
    --   k   = ceil(log2 w): the box filter's window is 2^k / 4096 cycle (1 .. 2 pixels of gradient)
    --   dh  = half the band: 1/4 (Bold) or 1/12, at least one pixel of gradient (Fine)
    --   cov = clamp(8 + 16 (dh - t) / 2^k), t = |frac(f) - 1/2|     (exact for a window no wider than band and gap)
    --   past w = 1/4 the window stays at 1/2 cycle and the result fades to grey (al = 8 .. 0 by w = 1/2)
    ---------------------------------------------------------------------------
    p_cov : process(clk)
        variable v_g  : unsigned(20 downto 0);
        variable v_f  : unsigned(11 downto 0);
        variable v_w  : unsigned(12 downto 0);
        variable v_wm : unsigned(9 downto 0);
        variable v_k  : unsigned(3 downto 0);
        variable v_dh : unsigned(10 downto 0);
        variable v_big, v_bad : std_logic;
        variable v_sh : signed(16 downto 0);
        variable v_m  : signed(10 downto 0);
        variable v_c  : signed(6 downto 0);
    begin
        if rising_edge(clk) then
            -- 14: phases and gradients
            u14_f(0) <= b_S(0)(31 downto 16) + s13_cam(15 downto 0);
            u14_f(1) <= b_S(1)(31 downto 16) + s13_cam(15 downto 0);
            u14_f(2) <= b_S(1)(31 downto 16) - b_S(0)(31 downto 16);
            u14_gx(0) <= resize(b_Dx(0)(39 downto 20), 21) + resize(s13_cdx, 21);
            u14_gx(1) <= resize(b_Dx(1)(39 downto 20), 21) + resize(s13_cdx, 21);
            u14_gx(2) <= resize(b_Dx(1)(39 downto 20), 21) - resize(b_Dx(0)(39 downto 20), 21);
            u14_gy(0) <= resize(b_Gy(0)(31 downto 12), 21) + resize(s13_cdy, 21);
            u14_gy(1) <= resize(b_Gy(1)(31 downto 12), 21) + resize(s13_cdy, 21);
            u14_gy(2) <= resize(b_Gy(1)(31 downto 12), 21) - resize(b_Gy(0)(31 downto 12), 21);

            for u in 0 to 2 loop
                -- 15: |gx|, |gy| (clamped under 1 cycle per pixel), t
                if u14_gx(u)(20) = '1' then v_g := unsigned(-u14_gx(u)); else v_g := unsigned(u14_gx(u)); end if;
                if v_g(20 downto 12) /= 0 then u15_ax(u) <= (others => '1'); else u15_ax(u) <= v_g(11 downto 0); end if;
                if u14_gy(u)(20) = '1' then v_g := unsigned(-u14_gy(u)); else v_g := unsigned(u14_gy(u)); end if;
                if v_g(20 downto 12) /= 0 then u15_ay(u) <= (others => '1'); else u15_ay(u) <= v_g(11 downto 0); end if;
                v_f := u14_f(u)(15 downto 4);
                if v_f(11) = '1' then u15_t(u) <= '0' & v_f(10 downto 0);
                else u15_t(u) <= to_unsigned(2048, 12) - resize(v_f(10 downto 0), 12); end if;
                -- 16: max, min
                if u15_ax(u) >= u15_ay(u) then u16_mx(u) <= u15_ax(u); u16_mn(u) <= u15_ay(u);
                else u16_mx(u) <= u15_ay(u); u16_mn(u) <= u15_ax(u); end if;
                u16_t(u) <= u15_t(u);
                -- 17: w
                u17_w(u) <= resize(u16_mx(u), 13) + resize(shift_right(u16_mn(u), 2), 13) + resize(shift_right(u16_mn(u), 3), 13);
                u17_t(u) <= u16_t(u);
                -- 18: k, the fade, dh - t
                v_w := u17_w(u);
                if v_w(12 downto 11) /= "00" or (v_w(10) = '1' and v_w(9 downto 0) /= 0) then v_big := '1'; else v_big := '0'; end if;   -- w > 1/4
                if v_w(10 downto 0) = 0 then v_wm := (others => '0'); else v_wm := resize(v_w(10 downto 0) - 1, 10); end if;
                v_k := (others => '0');
                for b in 0 to 9 loop
                    if v_wm(b) = '1' then v_k := to_unsigned(b + 1, 4); end if;
                end loop;
                if v_big = '1' then u18_k(u) <= to_unsigned(11, 4); else u18_k(u) <= v_k; end if;
                if s_raw = '1' or v_big = '0' then u18_al(u) <= to_unsigned(8, 4);
                elsif v_w(12 downto 11) /= "00" then u18_al(u) <= (others => '0');
                else u18_al(u) <= to_unsigned(8, 4) - resize(v_w(9 downto 7), 4); end if;
                if s_fine = '0' or u = 2 or v_w(12 downto 10) /= "000" then v_dh := to_unsigned(1024, 11);
                elsif v_w < 341 then v_dh := to_unsigned(341, 11);
                else v_dh := v_w(10 downto 0); end if;
                u18_d(u) <= signed(resize(v_dh, 13)) - signed(resize(u17_t(u), 13));
                -- 19: clamp(16 (dh - t) / 2^k + 8) to 0 .. 16 (Raw: a plain threshold).  The value is inside the
                -- ramp exactly when dh - t fits k signed bits, so no comparator follows the shifter, and + 8 on
                -- a value in -8 .. 7 is its bit 3 inverted.
                v_sh := shift_right(shift_left(resize(u18_d(u), 17), 4), to_integer(u18_k(u)));
                v_bad := '0';
                for i in 0 to 12 loop
                    if i + 1 >= to_integer(u18_k(u)) and u18_d(u)(i) /= u18_d(u)(12) then v_bad := '1'; end if;
                end loop;
                if u18_k(u) = 0 and u18_d(u)(12) = '1' then v_bad := '1'; end if;
                if s_raw = '1' then
                    if u18_d(u)(12) = '0' and u18_d(u) /= 0 then u19_c(u) <= to_unsigned(16, 5); else u19_c(u) <= (others => '0'); end if;
                elsif v_bad = '1' then
                    if u18_d(u)(12) = '1' then u19_c(u) <= (others => '0'); else u19_c(u) <= to_unsigned(16, 5); end if;
                else
                    u19_c(u) <= '0' & (not v_sh(3)) & unsigned(v_sh(2 downto 0));
                end if;
                u19_al(u) <= u18_al(u);
                -- 20: the fade toward grey; how far the unit has faded (0 .. 8)
                v_m := (signed(resize(u19_c(u), 6)) - 8) * signed(resize(u19_al(u), 5));
                v_c := to_signed(8, 7) + resize(shift_right(v_m, 3), 7);
                u20_cov(u) <= unsigned(v_c(4 downto 0));
                u20_kap(u) <= to_unsigned(8, 4) - u19_al(u);
            end loop;
        end if;
    end process p_cov;

    ---------------------------------------------------------------------------
    -- [E] blend (stages 21 .. 25):  Pair = a xor b = a + b - 2ab, plus the beat
    -- where both plates have faded;  Single = a.  Then luma 64 .. 940.
    -- [F] OUTPUT CONDITIONING (stages 26 .. 29): luma [1 2 1]/4, 13/16 toward
    -- black, blanking gate.  U = V = 512 always.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_o  : signed(13 downto 0);
        variable v_y  : unsigned(16 downto 0);
        variable v_s  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_win : std_logic;
    begin
        if rising_edge(clk) then
            -- 21: the small products (each has the stage to itself)
            s21_pab <= u20_cov(0) * u20_cov(1);
            s21_kab <= u20_kap(0) * u20_kap(1);
            s21_sab <= resize(u20_cov(0), 6) + resize(u20_cov(1), 6);
            s21_cd8 <= signed(resize(u20_cov(2), 6)) - 8;
            s21_ca  <= u20_cov(0);
            -- 22
            s22_x  <= signed(resize(s21_sab & "0000", 12)) - signed(resize(s21_pab & '0', 12));
            s22_bt <= signed(resize(s21_kab, 9)) * s21_cd8(4 downto 0);
            s22_ca <= s21_ca;
            -- 23: coverage 0 .. 256
            v_o := resize(s22_x, 14) + resize(shift_right(s22_bt, 2), 14);
            if s_pair = '0' then s23_o <= s22_ca & "0000";
            elsif v_o(13) = '1' then s23_o <= (others => '0');
            elsif v_o > 256 then s23_o <= to_unsigned(256, 9);
            else s23_o <= unsigned(v_o(8 downto 0)); end if;
            -- 24, 25: y = 64 + coverage * 219 / 64; black outside pixels 0 .. W + 3
            s24_t1 <= (s23_o & "00000000") - resize(s23_o & "00000", 17);
            s24_t2 <= resize(s23_o & "00", 11) + resize(s23_o, 11);
            v_y := s24_t1 - resize(s24_t2, 17);
            v_win := s_gav_sr(22) or s_gav_sr(26);
            if v_win = '1' then s25_y <= to_unsigned(64, 10) + v_y(15 downto 6);
            else s25_y <= to_unsigned(64, 10); end if;
            -- 26, 27: [1 2 1] / 4
            s26_y <= s25_y; s27_y <= s26_y;
            v_s := resize(s25_y, 12) + resize(s26_y & '0', 12) + resize(s27_y, 12) + 2;
            s27_f <= v_s(11 downto 2);
            -- 28: 13/16 toward black
            if s27_f > 64 then v_dl := s27_f - 64; else v_dl := (others => '0'); end if;
            s28_y <= s27_f - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            -- 29: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then s29_y <= s28_y; else s29_y <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s29_y);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture fresnel;
