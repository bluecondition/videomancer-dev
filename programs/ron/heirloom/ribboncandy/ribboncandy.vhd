-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Ribboncandy v0.5.0 -- Heirloom #39 (2026-10-27 rebuild), from WIN_20230530_22_27_05.
--
-- The photo is two hsync-reset horizontal oscillators, both restarting 91 px before active
-- video, each frequency-modulated line by line by its own vertical waveform:
--   * the RIBBON oscillator u: its phase picks the thick ribbon (a G ramp, dark left -> bright
--     right), a thin ribbon at u = 0.45 and the ground between; its frequency is a slow vertical
--     cosine (740 lines) plus three raised-cosine "kinks" of vsync ringing (the top zigzags), so
--     every ribbon S-curves and the waves grow toward the right;
--   * the FACE oscillator: a square wave whose half-periods are the ribbons' two faces -- blue
--     (B on: blue -> cyan with the ramp) and green (B off) -- and the thin ribbon's magenta (B on)
--     or red (B off).  Its frequency is a clipped vertical cosine, so the faces are vertical at the
--     bottom and twist into "/" diagonals mid-frame.  8x its frequency clocks the ground: a
--     sample-and-hold mosaic of 38 px x 32.5-line cells (b/w checker, white / light-grey rows,
--     yellow / cyan / magenta cells, black in one half of every face period).
-- Osc = the photo's own cells (a 64 x 64 cell table, ribboncandy.py).  Video = the camera,
-- sample-and-held on the same cell grid and thresholded per RGB channel into the same colours.
--
--   K1  Ribbons  ribbon oscillator frequency x0.4 .. x2.6 (3.7 across = the photo)
--   K2  Bends    the slow wave's frequency x0.3 .. x3.5 (1.46 S-bends per frame = the photo)
--   K3  Twist    face oscillator frequency x0.45 .. x2.5 (faces and cells together)
--   K4  Speed    bipolar, still in the centre: the bends and twists roll down the frame
--   K5  Cells    cells per face period 2 .. 24 (8 = the photo), rows scale with them
--   K6  Patch    RGB cable patch: RGB / RBG / GRB / GBR / BRG / BGR
--   S7  Faces    Twist (photo) / Solid (every ribbon on its blue face)
--   S8  Ground   Cells (photo) / Black
--   S9  Kinks    On (photo) / Off (the top and bottom zigzags)
--   S10 Flip     Down (photo) / Up: the vertical modulators flipped top to bottom
--   S11 Source   Video / Osc
--   P12 WAVE     FM depth: 0 = straight ribbons and upright faces, 50 % = the photo,
--                100 % = deep sway, sharp kinks and dense twists
--
-- Virtual coordinates: x spans 1920 virtual px on every mode (timing-ID steps), y = 1080 virtual
-- lines.  Generated avid; output conditioning ([1 2 2 2 2 2 2 2 1]/16 on Y, U and V, luma 13/16)
-- and blanking gate.  Latency 16 clocks in every mode.  Colours: u = Cr, v = Cb.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.ribboncandy_pkg.all;
use work.all;

architecture ribboncandy of program_top is
    constant C_LAT : integer := 16;   -- matches processing_delay_clks

    function f_kx(t : std_logic_vector(3 downto 0)) return unsigned is    -- virtual px per px, Q12
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(10923, 14);
            when 5 | 6 | 14     => return to_unsigned(6144, 14);
            when others         => return to_unsigned(4096, 14);
        end case;
    end function;
    function f_xr(t : std_logic_vector(3 downto 0)) return unsigned is    -- 91 virtual px in px, Q4
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(546, 11);
            when 5 | 6 | 14     => return to_unsigned(971, 11);
            when others         => return to_unsigned(1456, 11);
        end case;
    end function;
    function f_ys(t : std_logic_vector(3 downto 0)) return unsigned is    -- virtual lines per line, Q8
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 11);
            when 4          => return to_unsigned(576, 11);
            when 8          => return to_unsigned(960, 11);
            when 12         => return to_unsigned(480, 11);
            when 5 | 6 | 14 => return to_unsigned(384, 11);
            when 1 | 2 | 10 => return to_unsigned(512, 11);
            when others     => return to_unsigned(256, 11);
        end case;
    end function;

    -- ROMs / RAM
    signal s_sina : unsigned(9 downto 0) := (others => '0');
    signal s_sinq : std_logic_vector(11 downto 0) := (others => '0');
    signal s_bmpa : unsigned(8 downto 0) := (others => '0');
    signal s_bmpq : std_logic_vector(7 downto 0) := (others => '0');
    signal s_cela : unsigned(11 downto 0) := (others => '0');
    signal s_celq : std_logic_vector(2 downto 0) := (others => '0');
    type t_cram is array (0 to 63) of std_logic_vector(3 downto 0);
    signal cram : t_cram := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of cram : signal is true;
    signal s_cwe : std_logic := '0';
    signal s_cwa : unsigned(5 downto 0) := (others => '0');
    signal s_cwd : std_logic_vector(3 downto 0) := (others => '0');
    signal s_cra : unsigned(5 downto 0) := (others => '0');
    signal s_crq : std_logic_vector(3 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid : std_logic_vector(3 downto 0) := (others => '1');
    signal s_kx : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_xr : unsigned(10 downto 0) := to_unsigned(1456, 11);
    signal s_ys : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq : std_logic := '0';
    signal s_kick_c, s_kick_e : std_logic := '0';     -- registered source-avid rise (commit / engine copies)
    attribute keep : boolean;
    attribute keep of s_kick_c : signal is true;
    attribute keep of s_kick_e : signal is true;
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- glided knobs (10.3), switches
    signal g1, g2, g3, g5, g12 : unsigned(12 downto 0) := (others => '0');
    signal s_sp : signed(10 downto 0) := (others => '0');
    signal s_asp : unsigned(9 downto 0) := (others => '0');
    signal s_patch : unsigned(2 downto 0) := (others => '0');
    signal s_solid, s_black, s_nokink, s_flip, s_osc : std_logic := '0';
    signal s_warm : std_logic := '0';
    -- glide pipeline (3 clocks after the zone start): d = k - g; step = g + d/8, big = |d| > 31; apply
    type t_gd is array (0 to 4) of signed(14 downto 0);
    type t_gn is array (0 to 4) of unsigned(12 downto 0);
    signal s_gd : t_gd := (others => (others => '0'));
    signal s_gn : t_gn := (others => (others => '0'));
    signal s_gb : std_logic_vector(4 downto 0) := (others => '0');
    signal s_zgo2, s_zgo3 : std_logic := '0';

    -- field constants
    signal c_f0s, c_ts, c_ffs : unsigned(9 downto 0) := to_unsigned(256, 10);     -- Q8
    signal c_mq : unsigned(8 downto 0) := to_unsigned(128, 9);                     -- Q4
    signal c_amp, c_fdep, c_bmp : unsigned(9 downto 0) := to_unsigned(256, 10);   -- Q8
    signal c_F0, c_CF, c_FM, c_AF : unsigned(22 downto 0) := (others => '0');      -- Q28 per px
    signal c_A : unsigned(17 downto 0) := (others => '0');
    signal c_B : unsigned(18 downto 0) := (others => '0');                         -- Q8 per bump unit
    signal c_rinc : unsigned(15 downto 0) := (others => '0');                      -- Q16 rows per line
    signal c_dta, c_dtf : unsigned(23 downto 0) := (others => '0');               -- Q24 per line
    signal c_ta0, c_tf0 : unsigned(23 downto 0) := (others => '0');
    signal s_rolla, s_rollf : unsigned(23 downto 0) := (others => '0');
    signal s_linit : std_logic := '0';
    -- field sequencer: one serial shift-add multiplier
    signal f_st : unsigned(5 downto 0) := (others => '0');
    signal f_ma : unsigned(37 downto 0) := (others => '0');
    signal f_mb : unsigned(13 downto 0) := (others => '0');
    signal f_acc : unsigned(37 downto 0) := (others => '0');
    signal f_mc : unsigned(3 downto 0) := (others => '0');
    signal f_ci : unsigned(1 downto 0) := (others => '0');
    signal f_hi : std_logic := '0';
    signal f_k : unsigned(9 downto 0) := (others => '0');
    signal f_t : unsigned(23 downto 0) := (others => '0');

    -- per-line engine (computes the NEXT line, kicked at the source avid rise)
    signal e_st : unsigned(4 downto 0) := (others => '0');
    signal e_ma : unsigned(39 downto 0) := (others => '0');
    signal e_mb : unsigned(10 downto 0) := (others => '0');
    signal e_acc : unsigned(39 downto 0) := (others => '0');
    signal e_mc : unsigned(3 downto 0) := (others => '0');
    signal e_vy8 : unsigned(19 downto 0) := (others => '0');
    signal e_ta, e_tf : unsigned(23 downto 0) := (others => '0');
    signal e_racc : unsigned(21 downto 0) := (others => '0');
    signal e_vy : unsigned(10 downto 0) := (others => '0');
    signal e_ca, e_cf : signed(11 downto 0) := (others => '0');
    signal e_b8 : unsigned(7 downto 0) := (others => '0');
    signal e_ta2 : unsigned(28 downto 0) := (others => '0');
    signal e_incA, e_incF : unsigned(22 downto 0) := (others => '0');
    signal e_tfs : signed(24 downto 0) := (others => '0');
    signal e_phF : signed(30 downto 0) := (others => '0');
    signal e_rprev : unsigned(5 downto 0) := (others => '1');
    -- next-line values
    signal n_incA, n_incF : unsigned(22 downto 0) := (others => '0');
    signal n_incC : unsigned(21 downto 0) := (others => '0');
    signal n_phA, n_phF, n_phC : unsigned(27 downto 0) := (others => '0');
    signal n_r : unsigned(5 downto 0) := (others => '0');
    signal n_first : std_logic := '1';
    -- committed line values
    signal l_incA, l_incF : unsigned(22 downto 0) := (others => '0');
    signal l_incC : unsigned(21 downto 0) := (others => '0');
    signal l_phA, l_phF, l_phC : unsigned(27 downto 0) := (others => '0');
    signal l_r : unsigned(5 downto 0) := (others => '0');
    signal l_first : std_logic := '1';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- beam (oscillator accumulators, Q28)
    signal b_aA, b_aF, b_aC : unsigned(27 downto 0) := (others => '0');

    -- video: camera -> per-channel code
    signal v_y1, v_u1, v_v1 : unsigned(9 downto 0) := (others => '0');
    signal v_r2, v_g2, v_b2 : signed(11 downto 0) := (others => '0');
    signal v_l2 : signed(10 downto 0) := (others => '0');
    signal v_cd3, v_ch3 : unsigned(3 downto 0) := (others => '0');    -- low / high threshold codes

    -- pixel pipeline
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal p_f1, p_f2 : unsigned(11 downto 0) := (others => '0');
    signal p_face1, p_face2, p_face3 : std_logic := '0';
    signal p_n1 : unsigned(5 downto 0) := (others => '0');
    signal p_cs1, p_cs2 : std_logic := '0';
    signal p_tf2, p_t02, p_hf2, p_h02, p_g92 : std_logic := '0';
    signal p_gm2 : unsigned(18 downto 0) := (others => '0');
    signal p_rm2 : signed(18 downto 0) := (others => '0');
    signal p_thk3, p_thn3 : std_logic := '0';
    signal p_g3, p_rg3 : unsigned(7 downto 0) := (others => '0');
    signal p_cel3 : unsigned(2 downto 0) := (others => '0');
    signal p_vc3, s_vhold : unsigned(3 downto 0) := (others => '0');
    signal p_rgb4, p_o5 : t_u8(0 to 2) := (others => (others => '0'));
    signal p_yr6, p_yg6, p_yb6 : unsigned(15 downto 0) := (others => '0');
    signal p_ur6, p_ug6, p_ub6, p_vr6, p_vg6, p_vb6 : signed(16 downto 0) := (others => '0');
    signal p_ys7 : unsigned(16 downto 0) := (others => '0');
    signal p_us7, p_vs7 : signed(16 downto 0) := (others => '0');
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
    signal s_csy, s_csu, s_csv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_boxy, s_boxu, s_boxv, s_boxyd, s_boxud, s_boxvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y12 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u12, s_v12 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_yf : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_uf, s_vf : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_sin : process(clk)
    begin
        if rising_edge(clk) then
            s_sinq <= C_RSIN(to_integer(s_sina));
        end if;
    end process p_sin;

    p_bmp : process(clk)
    begin
        if rising_edge(clk) then
            s_bmpq <= C_RBMP(to_integer(s_bmpa));
        end if;
    end process p_bmp;

    p_cel : process(clk)
    begin
        if rising_edge(clk) then
            s_celq <= C_RCEL(to_integer(s_cela));
        end if;
    end process p_cel;

    p_cram : process(clk)
    begin
        if rising_edge(clk) then
            s_crq <= cram(to_integer(s_cra));
            if s_cwe = '1' then cram(to_integer(s_cwa)) <= s_cwd; end if;
        end if;
    end process p_cram;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s : signed(10 downto 0);
        variable v_t : unsigned(12 downto 0);
        variable v_d : unsigned(9 downto 0);
        variable v_m : unsigned(23 downto 0);
        variable v_c3 : unsigned(2 downto 0);
        variable v_g : signed(14 downto 0);
        variable v_a : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_kx <= f_kx(s_tid);
            s_xr <= f_xr(s_tid);
            s_ys <= f_ys(s_tid);

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            g_wm1 <= s_w - 1;
            s_linit <= '0';

            -- glides on K1, K2, K3, K5, P12 (1/8 of the gap per field, +-4 deadband, snap on the first field)
            s_zgo2 <= s_zgo; s_zgo3 <= s_zgo2;
            if s_zgo = '1' then
                s_gd(0) <= signed(resize(unsigned(registers_in(0)) & "000", 15)) - signed(resize(g1, 15));
                s_gd(1) <= signed(resize(unsigned(registers_in(1)) & "000", 15)) - signed(resize(g2, 15));
                s_gd(2) <= signed(resize(unsigned(registers_in(2)) & "000", 15)) - signed(resize(g3, 15));
                s_gd(3) <= signed(resize(unsigned(registers_in(4)) & "000", 15)) - signed(resize(g5, 15));
                s_gd(4) <= signed(resize(unsigned(registers_in(7)) & "000", 15)) - signed(resize(g12, 15));
            end if;
            if s_zgo2 = '1' then
                v_g := signed(resize(g1, 15)) + shift_right(s_gd(0), 3); s_gn(0) <= unsigned(v_g(12 downto 0));
                v_g := signed(resize(g2, 15)) + shift_right(s_gd(1), 3); s_gn(1) <= unsigned(v_g(12 downto 0));
                v_g := signed(resize(g3, 15)) + shift_right(s_gd(2), 3); s_gn(2) <= unsigned(v_g(12 downto 0));
                v_g := signed(resize(g5, 15)) + shift_right(s_gd(3), 3); s_gn(3) <= unsigned(v_g(12 downto 0));
                v_g := signed(resize(g12, 15)) + shift_right(s_gd(4), 3); s_gn(4) <= unsigned(v_g(12 downto 0));
                for i in 0 to 4 loop
                    if s_gd(i) > 31 or s_gd(i) < -31 then s_gb(i) <= '1'; else s_gb(i) <= '0'; end if;
                end loop;
            end if;
            if s_zgo3 = '1' then
                s_warm <= '1';
                if s_warm = '0' then
                    g1 <= unsigned(registers_in(0)) & "000"; g2 <= unsigned(registers_in(1)) & "000";
                    g3 <= unsigned(registers_in(2)) & "000"; g5 <= unsigned(registers_in(4)) & "000";
                    g12 <= unsigned(registers_in(7)) & "000";
                else
                    if s_gb(0) = '1' then g1 <= s_gn(0); end if;
                    if s_gb(1) = '1' then g2 <= s_gn(1); end if;
                    if s_gb(2) = '1' then g3 <= s_gn(2); end if;
                    if s_gb(3) = '1' then g5 <= s_gn(3); end if;
                    if s_gb(4) = '1' then g12 <= s_gn(4); end if;
                end if;
                f_ci <= (others => '0');
                f_st <= to_unsigned(1, 6);
            end if;

            if s_zgo = '1' then
                v_s := signed(resize(unsigned(registers_in(3)), 11)) - 512;          -- K4 Speed
                if v_s > -8 and v_s < 8 then v_s := (others => '0'); end if;
                s_sp <= v_s;
                v_a := -v_s;
                if v_s < 0 then s_asp <= unsigned(v_a(9 downto 0)); else s_asp <= unsigned(v_s(9 downto 0)); end if;
                -- K6 patch: (k6 * 3) >> 9 -> 0..5; a labelled default arrives as its index
                v_t := resize(unsigned(registers_in(5)), 13) + resize(unsigned(registers_in(5)) & '0', 13);
                if unsigned(registers_in(5)) < 6 then
                    s_patch <= unsigned(registers_in(5)(2 downto 0));
                else
                    s_patch <= v_t(11 downto 9);
                end if;
                s_solid  <= registers_in(6)(0);
                s_black  <= registers_in(6)(1);
                s_nokink <= registers_in(6)(2);
                s_flip   <= registers_in(6)(3);
                s_osc    <= registers_in(6)(4);
                if s_wmax /= 0 then s_w <= s_wmax; end if;
                s_wmax <= (others => '0');
            end if;

            -- per-field sequencer: one op per state; the multiplier steps while f_mc /= 0.
            if f_mc /= 0 then
                if f_mb(0) = '1' then f_acc <= f_acc + f_ma; end if;
                f_ma <= f_ma(36 downto 0) & '0';
                f_mb <= '0' & f_mb(13 downto 1);
                f_mc <= f_mc - 1;
            else
                case to_integer(f_st) is
                    -- knob curves (f_ci = 0 K1, 1 K2, 2 K3, 3 K5): lo + .. mid .. + hi, square-law halves
                    when 1 =>
                        case to_integer(f_ci) is
                            when 0      => v_d := g1(12 downto 3);
                            when 1      => v_d := g2(12 downto 3);
                            when 2      => v_d := g3(12 downto 3);
                            when others => v_d := g5(12 downto 3);
                        end case;
                        if v_d(9) = '1' then
                            f_hi <= '1'; f_k <= v_d - 512;
                        else
                            f_hi <= '0'; f_k <= 512 - v_d;
                        end if;
                        f_st <= f_st + 1;
                    when 2 =>       -- d^2
                        f_ma <= resize(f_k, 38); f_mb <= resize(f_k, 14); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 3 =>       -- (d^2 >> 2) * coef
                        f_ma <= resize(f_acc(19 downto 2), 38);
                        v_c3 := f_ci & f_hi;
                        case to_integer(v_c3) is
                            when 0  => f_mb <= to_unsigned(154, 14);
                            when 1  => f_mb <= to_unsigned(410, 14);
                            when 2  => f_mb <= to_unsigned(179, 14);
                            when 3  => f_mb <= to_unsigned(640, 14);
                            when 4  => f_mb <= to_unsigned(141, 14);
                            when 5  => f_mb <= to_unsigned(384, 14);
                            when 6  => f_mb <= to_unsigned(96, 14);
                            when others => f_mb <= to_unsigned(256, 14);
                        end case;
                        f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 4 =>
                        if f_hi = '1' then
                            v_d := to_unsigned(256, 10) + resize(f_acc(25 downto 16), 10);
                        else
                            v_d := to_unsigned(256, 10) - resize(f_acc(25 downto 16), 10);
                        end if;
                        case to_integer(f_ci) is
                            when 0      => c_f0s <= v_d;
                            when 1      => c_ts <= v_d;
                            when 2      => c_ffs <= v_d;
                            when others =>       -- K5 is centred on 128 (Q4)
                                if f_hi = '1' then
                                    c_mq <= to_unsigned(128, 9) + resize(f_acc(24 downto 16), 9);
                                else
                                    c_mq <= to_unsigned(128, 9) - resize(f_acc(24 downto 16), 9);
                                end if;
                        end case;
                        f_ci <= f_ci + 1;
                        if f_ci = 3 then f_st <= f_st + 1; else f_st <= to_unsigned(1, 6); end if;
                    when 5 =>       -- P12 Wave: amp = w^2, fdep = bump = w below 50 %; linear above
                        if g12(12) = '0' then
                            f_ma <= resize(g12(12 downto 3), 38); f_mb <= resize(g12(12 downto 3), 14);
                        else
                            f_ma <= resize(g12(11 downto 3), 38); f_mb <= to_unsigned(205, 14);
                        end if;
                        f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    -- below 50 %: amp = 0.15 + 0.85 w^2, fdep = 0.2 + 0.8 w, bump = 0.25 + 0.75 w (floors keep
                    -- the bends, twists and kinks alive at 0 %); above: 1 + 1.6 / 0.9 / 1.4 (w - 1)
                    when 6 =>
                        if g12(12) = '0' then
                            f_ma <= resize(f_acc(19 downto 10), 38); f_mb <= to_unsigned(218, 14);
                        else
                            c_amp <= to_unsigned(256, 10) + resize(f_acc(16 downto 8), 10);
                            f_ma <= resize(g12(11 downto 3), 38); f_mb <= to_unsigned(115, 14);
                        end if;
                        f_acc <= (others => '0'); f_mc <= to_unsigned(9, 4);
                        f_st <= f_st + 1;
                    when 7 =>
                        if g12(12) = '0' then
                            c_amp <= to_unsigned(38, 10) + resize(f_acc(15 downto 8), 10);
                            f_ma <= resize(g12(12 downto 4), 38); f_mb <= to_unsigned(205, 14);
                        else
                            c_fdep <= to_unsigned(256, 10) + resize(f_acc(16 downto 8), 10);
                            f_ma <= resize(g12(11 downto 3), 38); f_mb <= to_unsigned(179, 14);
                        end if;
                        f_acc <= (others => '0'); f_mc <= to_unsigned(8, 4);
                        f_st <= f_st + 1;
                    when 8 =>
                        if g12(12) = '0' then
                            c_fdep <= to_unsigned(51, 10) + resize(f_acc(15 downto 8), 10);
                            f_ma <= resize(g12(12 downto 4), 38); f_mb <= to_unsigned(192, 14);
                            f_acc <= (others => '0'); f_mc <= to_unsigned(8, 4);
                        else
                            c_bmp <= to_unsigned(256, 10) + resize(f_acc(16 downto 8), 10);
                        end if;
                        f_st <= f_st + 1;
                    when 9 =>       -- F0 = (517005 f0s >> 8) kx >> 12
                        if s_nokink = '1' then
                            c_bmp <= (others => '0');
                        elsif g12(12) = '0' then
                            c_bmp <= to_unsigned(64, 10) + resize(f_acc(15 downto 8), 10);
                        end if;
                        f_ma <= to_unsigned(517005, 38); f_mb <= resize(c_f0s, 14); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 10 =>
                        f_ma <= resize(f_acc(31 downto 8), 38); f_mb <= s_kx; f_acc <= (others => '0'); f_mc <= to_unsigned(14, 4);
                        f_st <= f_st + 1;
                    when 11 =>      -- A = (29394 amp >> 8) kx >> 12
                        c_F0 <= f_acc(34 downto 12);
                        f_ma <= to_unsigned(29394, 38); f_mb <= resize(c_amp, 14); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 12 =>
                        f_ma <= resize(f_acc(31 downto 8), 38); f_mb <= s_kx; f_acc <= (others => '0'); f_mc <= to_unsigned(14, 4);
                        f_st <= f_st + 1;
                    when 13 =>      -- B = 213 bmp kx >> 12
                        c_A <= f_acc(29 downto 12);
                        f_ma <= to_unsigned(213, 38); f_mb <= resize(c_bmp, 14); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 14 =>
                        f_ma <= resize(f_acc(23 downto 0), 38); f_mb <= s_kx; f_acc <= (others => '0'); f_mc <= to_unsigned(14, 4);
                        f_st <= f_st + 1;
                    when 15 =>      -- CF = (760423 ffs >> 8) kx >> 12
                        c_B <= f_acc(30 downto 12);
                        f_ma <= to_unsigned(760423, 38); f_mb <= resize(c_ffs, 14); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 16 =>
                        f_ma <= resize(f_acc(31 downto 8), 38); f_mb <= s_kx; f_acc <= (others => '0'); f_mc <= to_unsigned(14, 4);
                        f_st <= f_st + 1;
                    when 17 =>      -- FM = (880036 ffs >> 8) kx >> 12
                        c_CF <= f_acc(34 downto 12);
                        f_ma <= to_unsigned(880036, 38); f_mb <= resize(c_ffs, 14); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 18 =>
                        f_ma <= resize(f_acc(31 downto 8), 38); f_mb <= s_kx; f_acc <= (others => '0'); f_mc <= to_unsigned(14, 4);
                        f_st <= f_st + 1;
                    when 19 =>      -- AF = ((193113 fdep >> 8) ffs >> 8) kx >> 12
                        c_FM <= f_acc(34 downto 12);
                        f_ma <= to_unsigned(193113, 38); f_mb <= resize(c_fdep, 14); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 20 =>
                        f_ma <= resize(f_acc(31 downto 8), 38); f_mb <= resize(c_ffs, 14); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 21 =>
                        f_ma <= resize(f_acc(31 downto 8), 38); f_mb <= s_kx; f_acc <= (others => '0'); f_mc <= to_unsigned(14, 4);
                        f_st <= f_st + 1;
                    when 22 =>      -- RINC = (ys mq) 2014 >> 15
                        c_AF <= f_acc(34 downto 12);
                        f_ma <= resize(s_ys, 38); f_mb <= resize(c_mq, 14); f_acc <= (others => '0'); f_mc <= to_unsigned(9, 4);
                        f_st <= f_st + 1;
                    when 23 =>
                        f_ma <= resize(f_acc(19 downto 0), 38); f_mb <= to_unsigned(2014, 14); f_acc <= (others => '0'); f_mc <= to_unsigned(11, 4);
                        f_st <= f_st + 1;
                    when 24 =>      -- DTA = ((22651 ts) >> 8) ys >> 8
                        c_rinc <= f_acc(30 downto 15);
                        f_ma <= to_unsigned(22651, 38); f_mb <= resize(c_ts, 14); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 25 =>
                        f_ma <= resize(f_acc(31 downto 8), 38); f_mb <= resize(s_ys, 14); f_acc <= (others => '0'); f_mc <= to_unsigned(11, 4);
                        f_st <= f_st + 1;
                    when 26 =>      -- DTF = 12062 ys >> 8
                        c_dta <= f_acc(31 downto 8);
                        f_ma <= to_unsigned(12062, 38); f_mb <= resize(s_ys, 14); f_acc <= (others => '0'); f_mc <= to_unsigned(11, 4);
                        f_st <= f_st + 1;
                    when 27 =>      -- roll A step: (|sp| ts) 443 >> 10
                        c_dtf <= f_acc(31 downto 8);
                        f_ma <= resize(s_asp, 38); f_mb <= resize(c_ts, 14); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 28 =>
                        f_ma <= resize(f_acc(19 downto 0), 38); f_mb <= to_unsigned(443, 14); f_acc <= (others => '0'); f_mc <= to_unsigned(9, 4);
                        f_st <= f_st + 1;
                    when 29 =>
                        v_m := resize(f_acc(29 downto 10), 24);
                        if s_sp < 0 then s_rolla <= s_rolla - v_m; else s_rolla <= s_rolla + v_m; end if;
                        f_ma <= resize(s_asp, 38); f_mb <= to_unsigned(38, 14); f_acc <= (others => '0'); f_mc <= to_unsigned(6, 4);
                        f_st <= f_st + 1;
                    when 30 =>      -- roll F step: |sp| 38
                        v_m := resize(f_acc(15 downto 0), 24);
                        if s_sp < 0 then s_rollf <= s_rollf - v_m; else s_rollf <= s_rollf + v_m; end if;
                        f_ma <= resize(c_ts, 38);
                        if s_flip = '1' then f_mb <= to_unsigned(13039, 14); else f_mb <= to_unsigned(0, 14); end if;
                        f_acc <= (others => '0'); f_mc <= to_unsigned(14, 4);
                        f_st <= f_st + 1;
                    when 31 =>      -- ta = ts * (flip ? 13039 : 0) - ts * 82431 - rollA
                        f_t <= f_acc(23 downto 0);
                        f_ma <= resize(c_ts, 38); f_mb <= to_unsigned(10304, 14);      -- 82431 = 8 * 10304 - 1
                        f_acc <= (others => '0'); f_mc <= to_unsigned(14, 4);
                        f_st <= f_st + 1;
                    when 32 =>
                        v_m := f_acc(20 downto 0) & "000";
                        c_ta0 <= f_t - v_m + resize(c_ts, 24) - s_rolla;
                        if s_flip = '1' then
                            c_tf0 <= to_unsigned(10356036, 24) - s_rollf;
                        else
                            c_tf0 <= to_unsigned(14118340, 24) - s_rollf;
                        end if;
                        f_st <= f_st + 1;
                    when 33 =>
                        s_linit <= '1';            -- start the line engine on line 0
                        f_st <= (others => '0');
                    when others =>
                        null;
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
    -- Generated avid (parity-locked start, field-latched width)
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
    -- Per-line engine: at the source avid rise it commits the values computed for this
    -- line and starts on the next one (~110 clocks, inside SD's 858-clock line).
    --   incA = F0 - A cos(ta) + B bump(yv) >> 8          ribbon oscillator, Q28 per px
    --   incF = min(FM, CF - AF cos(tf))                   face oscillator
    --   phA  = XR incA >> 4 - 0.0803                      phases at the first active pixel
    --   phF  = XR incF >> 4 - 0.5644 (signed)
    --   incC = MQ incF >> 10, phC = MQ phF >> 10          cell counter, 2^22 per cell
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_n : unsigned(22 downto 0);
        variable v_s : signed(24 downto 0);
        variable v_p : signed(30 downto 0);
        variable v_q : unsigned(27 downto 0);
        variable v_y : unsigned(10 downto 0);
        variable v_c : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s_kick_c <= data_in.avid and not s_avq;
            s_kick_e <= data_in.avid and not s_avq;
            if s_kick_c = '1' then
                -- commit this line (1 clock after the source avid rise; g_start comes 3+ clocks after it)
                l_incA <= n_incA; l_incF <= n_incF; l_incC <= n_incC;
                l_phA <= n_phA; l_phF <= n_phF; l_phC <= n_phC;
                l_r <= n_r; l_first <= n_first;
            end if;

            if s_linit = '1' then
                e_vy8 <= (others => '0'); e_ta <= c_ta0; e_tf <= c_tf0;
                e_racc <= to_unsigned(6685, 22); e_rprev <= (others => '1');
                e_st <= to_unsigned(2, 5);
            elsif s_kick_e = '1' then
                e_st <= to_unsigned(1, 5);
            elsif e_mc /= 0 then
                if e_mb(0) = '1' then e_acc <= e_acc + e_ma; end if;
                e_ma <= e_ma(38 downto 0) & '0';
                e_mb <= '0' & e_mb(10 downto 1);
                e_mc <= e_mc - 1;
            else
                case to_integer(e_st) is
                    when 1 =>       -- advance to the next line
                        e_vy8 <= e_vy8 + resize(s_ys, 20);
                        e_racc <= e_racc + resize(c_rinc, 22);
                        if s_flip = '1' then
                            e_ta <= e_ta - c_dta; e_tf <= e_tf - c_dtf;
                        else
                            e_ta <= e_ta + c_dta; e_tf <= e_tf + c_dtf;
                        end if;
                        e_st <= e_st + 1;
                    when 2 =>       -- cos(ta) address; virtual line
                        s_sina <= e_ta(23 downto 14) + 256;
                        if e_vy8(19 downto 8) > 1079 then v_y := to_unsigned(1079, 11); else v_y := e_vy8(18 downto 8); end if;
                        e_vy <= v_y;
                        e_st <= e_st + 1;
                    when 3 =>       -- bump address (flipped line); cos(tf) address
                        if s_flip = '1' then v_y := to_unsigned(1079, 11) - e_vy; else v_y := e_vy; end if;
                        s_bmpa <= v_y(10 downto 2);
                        s_sina <= e_tf(23 downto 14) + 256;
                        e_st <= e_st + 1;
                    when 4 =>
                        e_ca <= signed(s_sinq);
                        e_st <= e_st + 1;
                    when 5 =>       -- A |cos(ta)|
                        e_cf <= signed(s_sinq);
                        e_b8 <= unsigned(s_bmpq);
                        e_ma <= resize(c_A, 40);
                        v_c := -e_ca; if e_ca < 0 then e_mb <= unsigned(v_c(10 downto 0)); else e_mb <= unsigned(e_ca(10 downto 0)); end if;
                        e_acc <= (others => '0'); e_mc <= to_unsigned(11, 4);
                        e_st <= e_st + 1;
                    when 6 =>       -- B bump
                        e_ta2 <= resize(e_acc(28 downto 11), 29);
                        e_ma <= resize(c_B, 40); e_mb <= resize(e_b8, 11); e_acc <= (others => '0'); e_mc <= to_unsigned(8, 4);
                        e_st <= e_st + 1;
                    when 7 =>
                        if e_ca < 0 then
                            v_n := c_F0 + e_ta2(22 downto 0) + e_acc(30 downto 8);
                        else
                            v_n := c_F0 - e_ta2(22 downto 0) + e_acc(30 downto 8);
                        end if;
                        e_incA <= v_n;
                        e_st <= e_st + 1;
                    when 8 =>       -- AF |cos(tf)|
                        e_ma <= resize(c_AF, 40);
                        v_c := -e_cf; if e_cf < 0 then e_mb <= unsigned(v_c(10 downto 0)); else e_mb <= unsigned(e_cf(10 downto 0)); end if;
                        e_acc <= (others => '0'); e_mc <= to_unsigned(11, 4);
                        e_st <= e_st + 1;
                    when 9 =>
                        if e_cf < 0 then
                            v_s := signed(resize(c_CF, 25)) + signed(resize(e_acc(33 downto 11), 25));
                        else
                            v_s := signed(resize(c_CF, 25)) - signed(resize(e_acc(33 downto 11), 25));
                        end if;
                        e_tfs <= v_s;
                        e_st <= e_st + 1;
                    when 10 =>      -- clip at FM; XR incA
                        if e_tfs > signed(resize(c_FM, 25)) then
                            e_incF <= c_FM;
                        elsif e_tfs < 0 then
                            e_incF <= (others => '0');
                        else
                            e_incF <= unsigned(e_tfs(22 downto 0));
                        end if;
                        e_ma <= resize(e_incA, 40); e_mb <= s_xr; e_acc <= (others => '0'); e_mc <= to_unsigned(11, 4);
                        e_st <= e_st + 1;
                    when 11 =>      -- phA; XR incF
                        n_phA <= e_acc(31 downto 4) - to_unsigned(21564000, 28);
                        n_incA <= e_incA;
                        e_ma <= resize(e_incF, 40); e_mb <= s_xr; e_acc <= (others => '0'); e_mc <= to_unsigned(11, 4);
                        e_st <= e_st + 1;
                    when 12 =>      -- phF (signed); MQ incF
                        v_p := signed(resize(e_acc(33 downto 4), 31)) - to_signed(151498000, 31);
                        e_phF <= v_p;
                        n_phF <= unsigned(v_p(27 downto 0));
                        n_incF <= e_incF;
                        e_ma <= resize(e_incF, 40); e_mb <= resize(c_mq, 11); e_acc <= (others => '0'); e_mc <= to_unsigned(9, 4);
                        e_st <= e_st + 1;
                    when 13 =>      -- incC; MQ |phF|
                        n_incC <= e_acc(31 downto 10);
                        if e_phF < 0 then
                            e_ma <= unsigned(resize(-e_phF, 40));
                        else
                            e_ma <= unsigned(resize(e_phF, 40));
                        end if;
                        e_mb <= resize(c_mq, 11); e_acc <= (others => '0'); e_mc <= to_unsigned(9, 4);
                        e_st <= e_st + 1;
                    when 14 =>      -- phC; row
                        v_q := e_acc(37 downto 10);
                        if e_phF < 0 then n_phC <= 0 - v_q; else n_phC <= v_q; end if;
                        n_r <= e_racc(21 downto 16);
                        if e_racc(21 downto 16) /= e_rprev then n_first <= '1'; else n_first <= '0'; end if;
                        e_rprev <= e_racc(21 downto 16);
                        e_st <= (others => '0');
                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the three oscillator accumulators (value = the current pixel)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_aA <= l_phA; b_aF <= l_phF; b_aC <= l_phC;
            elsif g_avid = '1' then
                b_aA <= b_aA + resize(l_incA, 28);
                b_aF <= b_aF + resize(l_incF, 28);
                b_aC <= b_aC + resize(l_incC, 28);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Camera: R, G, B (0 .. 876 over black .. white) thresholded per channel.
    -- Low threshold 395 (bright half of the face period), high 653 (dark half);
    -- code 8 = light grey (all three on, luma below 687).
    ---------------------------------------------------------------------------
    p_vid : process(clk)
        variable v_cr, v_cb : signed(10 downto 0);
        variable v_l : signed(10 downto 0);
        variable v_c : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            v_y1 <= unsigned(data_in.y); v_u1 <= unsigned(data_in.u); v_v1 <= unsigned(data_in.v);
            v_cr := signed(resize(v_u1, 11)) - 512;
            v_cb := signed(resize(v_v1, 11)) - 512;
            v_l := signed(resize(v_y1, 11)) - 64;
            -- R = L + 1.406 Cr, G = L - 0.344 Cb - 0.719 Cr, B = L + 1.781 Cb
            v_r2 <= resize(v_l, 12) + resize(v_cr, 12) + resize(shift_right(v_cr, 2), 12)
                    + resize(shift_right(v_cr, 3), 12) + resize(shift_right(v_cr, 5), 12);
            v_g2 <= resize(v_l, 12) - resize(shift_right(v_cb, 2), 12) - resize(shift_right(v_cb, 4), 12)
                    - resize(shift_right(v_cb, 5), 12) - resize(shift_right(v_cr, 1), 12)
                    - resize(shift_right(v_cr, 3), 12) - resize(shift_right(v_cr, 4), 12) - resize(shift_right(v_cr, 5), 12);
            v_b2 <= resize(v_l, 12) + resize(v_cb, 12) + resize(v_cb, 12) - resize(shift_right(v_cb, 2), 12)
                    + resize(shift_right(v_cb, 5), 12);
            v_l2 <= v_l;
            v_c := (others => '0');
            if v_r2 > 395 then v_c(2) := '1'; end if;
            if v_g2 > 395 then v_c(1) := '1'; end if;
            if v_b2 > 395 then v_c(0) := '1'; end if;
            if v_c = "0111" and v_l2 < 687 then v_c := "1000"; end if;
            v_cd3 <= v_c;
            v_c := (others => '0');
            if v_r2 > 653 then v_c(2) := '1'; end if;
            if v_g2 > 653 then v_c(1) := '1'; end if;
            if v_b2 > 653 then v_c(0) := '1'; end if;
            if v_c = "0111" and v_l2 < 687 then v_c := "1000"; end if;
            v_ch3 <= v_c;
        end if;
    end process p_vid;

    ---------------------------------------------------------------------------
    -- Pixel pipeline
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_fl : unsigned(11 downto 0);
        variable v_g : unsigned(18 downto 0);
        variable v_rm : signed(18 downto 0);
        variable v_c : unsigned(3 downto 0);
        variable v_x : t_u8(0 to 2);
        variable v_rr, v_gg, v_bv : unsigned(7 downto 0);
        variable v_y : unsigned(16 downto 0);
        variable v_cc : signed(16 downto 0);
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: ribbon phase (12 bits), face, cell; ROM / RAM addresses
            p_f1 <= b_aA(27 downto 16);
            p_face1 <= (not b_aF(27)) or s_solid;
            p_n1 <= b_aC(27 downto 22);
            if b_aC(27 downto 22) /= p_n1 then p_cs1 <= '1'; else p_cs1 <= '0'; end if;
            s_cela <= l_r & b_aC(27 downto 22);
            s_cra <= b_aC(27 downto 22);

            -- 2: ribbon tests; ramp products; cell RAM write on a row's first line
            p_f2 <= p_f1; p_face2 <= p_face1; p_cs2 <= p_cs1;
            v_fl := p_f1 + 246;
            if v_fl < 1618 then p_tf2 <= '1'; else p_tf2 <= '0'; end if;
            if v_fl < 1905 then p_t02 <= '1'; else p_t02 <= '0'; end if;
            if p_f1 >= 1851 and p_f1 < 2138 then p_hf2 <= '1'; else p_hf2 <= '0'; end if;
            if p_f1 >= 1831 and p_f1 < 2449 then p_h02 <= '1'; else p_h02 <= '0'; end if;
            if p_f1 >= 3686 then p_g92 <= '1'; else p_g92 <= '0'; end if;
            v_g := shift_left(resize(p_f1, 19), 6) + shift_left(resize(p_f1, 19), 3)
                   + shift_left(resize(p_f1, 19), 1) + resize(p_f1, 19);                    -- f * 75
            p_gm2 <= v_g;
            v_rm := signed(resize(p_f1, 19)) - 1884;
            p_rm2 <= shift_left(v_rm, 4) + shift_left(v_rm, 3) + shift_left(v_rm, 1) + v_rm;  -- * 27
            s_cwe <= '0';
            if l_first = '1' and p_cs1 = '1' then
                if p_n1(2 downto 0) = "001" or p_n1(2 downto 0) = "010" or
                   p_n1(2 downto 0) = "011" or p_n1(2 downto 0) = "100" then
                    v_c := v_ch3;
                else
                    v_c := v_cd3;
                end if;
                s_cwe <= '1'; s_cwa <= p_n1; s_cwd <= std_logic_vector(v_c);
                s_vhold <= v_c;
            end if;

            -- 3: ribbon flags, ramps; ROM / RAM data to fabric
            p_face3 <= p_face2;
            if p_face2 = '1' then p_thk3 <= p_tf2; p_thn3 <= p_hf2;
            else p_thk3 <= p_t02; p_thn3 <= p_h02; end if;
            if p_g92 = '1' then
                p_g3 <= to_unsigned(10, 8);
            elsif p_gm2(18 downto 9) > 245 then
                p_g3 <= to_unsigned(255, 8);
            else
                p_g3 <= p_gm2(16 downto 9) + 10;
            end if;
            if p_rm2 < 0 then
                p_rg3 <= to_unsigned(26, 8);
            elsif p_rm2(18 downto 8) > 229 then
                p_rg3 <= to_unsigned(255, 8);
            else
                p_rg3 <= unsigned(p_rm2(15 downto 8)) + 26;
            end if;
            p_cel3 <= unsigned(s_celq);
            if l_first = '1' then p_vc3 <= s_vhold; else p_vc3 <= unsigned(s_crq); end if;

            -- 4: colour: thin over thick over the ground
            if p_thn3 = '1' then
                if p_face3 = '1' then
                    v_x(0) := to_unsigned(242, 8); v_x(1) := to_unsigned(22, 8); v_x(2) := to_unsigned(248, 8);
                else
                    v_x(0) := to_unsigned(250, 8); v_x(1) := p_rg3; v_x(2) := to_unsigned(14, 8);
                end if;
            elsif p_thk3 = '1' then
                v_x(1) := p_g3;
                if p_face3 = '1' then v_x(0) := to_unsigned(2, 8); v_x(2) := to_unsigned(252, 8);
                else v_x(0) := to_unsigned(4, 8); v_x(2) := to_unsigned(10, 8); end if;
            elsif s_black = '1' then
                v_x(0) := to_unsigned(19, 8); v_x(1) := to_unsigned(16, 8); v_x(2) := to_unsigned(7, 8);
            elsif s_osc = '1' then
                case to_integer(p_cel3) is
                    when 1 => v_x(0) := to_unsigned(235, 8); v_x(1) := to_unsigned(241, 8); v_x(2) := to_unsigned(235, 8);
                    when 2 => v_x(0) := to_unsigned(198, 8); v_x(1) := to_unsigned(221, 8); v_x(2) := to_unsigned(226, 8);
                    when 3 => v_x(0) := to_unsigned(239, 8); v_x(1) := to_unsigned(247, 8); v_x(2) := to_unsigned(3, 8);
                    when 4 => v_x(0) := to_unsigned(11, 8);  v_x(1) := to_unsigned(244, 8); v_x(2) := to_unsigned(242, 8);
                    when 5 => v_x(0) := to_unsigned(240, 8); v_x(1) := to_unsigned(24, 8);  v_x(2) := to_unsigned(248, 8);
                    when 6 => v_x(0) := to_unsigned(139, 8); v_x(1) := to_unsigned(154, 8); v_x(2) := to_unsigned(160, 8);
                    when 7 => v_x(0) := to_unsigned(243, 8); v_x(1) := to_unsigned(52, 8);  v_x(2) := to_unsigned(13, 8);
                    when others => v_x(0) := to_unsigned(19, 8); v_x(1) := to_unsigned(16, 8); v_x(2) := to_unsigned(7, 8);
                end case;
            else
                case to_integer(p_vc3) is
                    when 1 => v_x(0) := to_unsigned(7, 8);   v_x(1) := to_unsigned(27, 8);  v_x(2) := to_unsigned(248, 8);
                    when 2 => v_x(0) := to_unsigned(2, 8);   v_x(1) := to_unsigned(205, 8); v_x(2) := to_unsigned(6, 8);
                    when 3 => v_x(0) := to_unsigned(11, 8);  v_x(1) := to_unsigned(244, 8); v_x(2) := to_unsigned(242, 8);
                    when 4 => v_x(0) := to_unsigned(243, 8); v_x(1) := to_unsigned(52, 8);  v_x(2) := to_unsigned(13, 8);
                    when 5 => v_x(0) := to_unsigned(240, 8); v_x(1) := to_unsigned(24, 8);  v_x(2) := to_unsigned(248, 8);
                    when 6 => v_x(0) := to_unsigned(239, 8); v_x(1) := to_unsigned(247, 8); v_x(2) := to_unsigned(3, 8);
                    when 7 => v_x(0) := to_unsigned(235, 8); v_x(1) := to_unsigned(241, 8); v_x(2) := to_unsigned(235, 8);
                    when 8 => v_x(0) := to_unsigned(198, 8); v_x(1) := to_unsigned(221, 8); v_x(2) := to_unsigned(226, 8);
                    when others => v_x(0) := to_unsigned(19, 8); v_x(1) := to_unsigned(16, 8); v_x(2) := to_unsigned(7, 8);
                end case;
            end if;
            p_rgb4 <= v_x;

            -- 5: RGB cable patch
            case to_integer(s_patch) is
                when 1      => p_o5(0) <= p_rgb4(0); p_o5(1) <= p_rgb4(2); p_o5(2) <= p_rgb4(1);
                when 2      => p_o5(0) <= p_rgb4(1); p_o5(1) <= p_rgb4(0); p_o5(2) <= p_rgb4(2);
                when 3      => p_o5(0) <= p_rgb4(1); p_o5(1) <= p_rgb4(2); p_o5(2) <= p_rgb4(0);
                when 4      => p_o5(0) <= p_rgb4(2); p_o5(1) <= p_rgb4(0); p_o5(2) <= p_rgb4(1);
                when 5      => p_o5(0) <= p_rgb4(2); p_o5(1) <= p_rgb4(1); p_o5(2) <= p_rgb4(0);
                when others => p_o5 <= p_rgb4;
            end case;

            -- 6..8: RGB -> 10-bit YUV (u = Cr, v = Cb), Q6; luma x1.07 so photo white lands at 940:
            -- Y = 64 + (70 R + 138 G + 27 B) / 64, Cr = 512 + (113 R - 94 G - 18 B) / 64,
            -- Cb = 512 + (-38 R - 75 G + 113 B) / 64
            v_rr := p_o5(0); v_gg := p_o5(1); v_bv := p_o5(2);
            p_yr6 <= shift_left(resize(v_rr, 16), 6) + shift_left(resize(v_rr, 16), 2) + shift_left(resize(v_rr, 16), 1);
            p_yg6 <= shift_left(resize(v_gg, 16), 7) + shift_left(resize(v_gg, 16), 3) + shift_left(resize(v_gg, 16), 1);
            p_yb6 <= shift_left(resize(v_bv, 16), 5) - shift_left(resize(v_bv, 16), 2) - resize(v_bv, 16);
            p_ur6 <= signed(shift_left(resize(v_rr, 17), 7) - shift_left(resize(v_rr, 17), 4) + resize(v_rr, 17));
            p_ug6 <= -signed(shift_left(resize(v_gg, 17), 6) + shift_left(resize(v_gg, 17), 5) - shift_left(resize(v_gg, 17), 1));
            p_ub6 <= -signed(shift_left(resize(v_bv, 17), 4) + shift_left(resize(v_bv, 17), 1));
            p_vr6 <= -signed(shift_left(resize(v_rr, 17), 5) + shift_left(resize(v_rr, 17), 2) + shift_left(resize(v_rr, 17), 1));
            p_vg6 <= -signed(shift_left(resize(v_gg, 17), 6) + shift_left(resize(v_gg, 17), 3) + shift_left(resize(v_gg, 17), 1) + resize(v_gg, 17));
            p_vb6 <= signed(shift_left(resize(v_bv, 17), 7) - shift_left(resize(v_bv, 17), 4) + resize(v_bv, 17));
            p_ys7 <= resize(p_yr6, 17) + resize(p_yg6, 17) + resize(p_yb6, 17);
            p_us7 <= p_ur6 + p_ug6 + p_ub6;
            p_vs7 <= p_vr6 + p_vg6 + p_vb6;
            v_y := shift_right(p_ys7, 6) + 64;
            if v_y > 940 then s_y8 <= to_unsigned(940, 10); else s_y8 <= v_y(9 downto 0); end if;
            v_cc := shift_right(p_us7, 6) + 512;
            s_u8 <= unsigned(v_cc(9 downto 0));
            v_cc := shift_right(p_vs7, 6) + 512;
            s_v8 <= unsigned(v_cc(9 downto 0));

            -- 9..11: output conditioning: [1 2 2 2 2 2 2 2 1]/16 on Y, U and V as two adjacent 8-sums
            s_csy(0) <= s_y8; s_csu(0) <= s_u8; s_csv(0) <= s_v8;
            for i in 1 to 7 loop s_csy(i) <= s_csy(i - 1); s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxy <= s_boxy + s_y8 - s_csy(7);
            s_boxu <= s_boxu + s_u8 - s_csu(7);
            s_boxv <= s_boxv + s_v8 - s_csv(7);
            s_boxyd <= s_boxy; s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxy, 14) + s_boxyd; s_y11 <= v_sc(13 downto 4);
            v_sc := resize(s_boxu, 14) + s_boxud; s_u11 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 14) + s_boxvd; s_v11 <= v_sc(13 downto 4);
            -- 12: luma toward black (940 -> 777); blanking gate on the filter-centre (stage 14) pixel's avid
            if s_y11 > 64 then v_dl := s_y11 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(13) = '1' then
                s_y12 <= s_y11 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u12 <= s_u11; s_v12 <= s_v11;
            else
                s_y12 <= to_unsigned(64, 10); s_u12 <= to_unsigned(512, 10); s_v12 <= to_unsigned(512, 10);
            end if;
            -- 13: output register (content stage 16 = C_LAT)
            s_yf <= s_y12; s_uf <= s_u12; s_vf <= s_v12;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_yf);
    data_out.u       <= std_logic_vector(s_uf);
    data_out.v       <= std_logic_vector(s_vf);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture ribboncandy;
