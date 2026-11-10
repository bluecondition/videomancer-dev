-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Tiger v0.1.1 -- Heirloom (video phase), rebuilt from the clip WIN_20231208_23_03_08 at 0.5 s.
--
-- v0.1.1: the camera bend is smooth: the luma passes a 16 px horizontal box and a vertical IIR (1/4 new,
-- 3/4 the previous smoothed line, in the line buffer itself) before it enters the phase; the default Etch
-- is a quarter of v0.1.0's bend (K6 128) and the Osc runs at 24x the camera gain (1.5x v0.1.0's Osc).
--
-- The clip: orange / blue / pink tiger bands that sweep diagonally across the frame, near-vertical at the
-- top left and twice as dense and leaning at the top right, bending into a row of pink "X" saddles and
-- pointed leaf tips along a fold row near the bottom; dark green gaps, thin red glows along the blue
-- edges.  Across the clip the fold row rolls up the screen (~8 lines per field) while the bands creep left.
--
-- The patch, as fitted (tiger.py has every constant):
--   theta = (x + xb) * f(y) + c(y)     a line-locked H triangle oscillator; its line-start reset sits 166 px
--                                      left of the picture, so the fan pivots off screen left
--   f(y)  = (1 + aF |sin((V - d)/2)|^2.37) / 638.8      FM from a slow V sine: the fan
--   V     = 2 pi (y0 - y) / 2754       the V oscillator; the fold row is its zero crossing (y0 = 880.6)
--   w     = A sin V                    the split, ADDED to the red colouriser and SUBTRACTED from the blue
--   uR    = frac(T + w), uB = frac(T - w)   two wrapping colourisers; R, B = camera transfer; G = floor
--   On the fold row R = B = T (pink peaks, dark troughs); above it red wraps around the peaks and blue
--   around the troughs (blue wedges), below the reverse (orange wedges).
--
--   K1  Bands     the H oscillator's frequency (x1/4 .. x3.1)
--   K2  Fan       the FM depth (linear): parallel bands (0) .. the clip (512) .. twice the clip's fan (1023)
--   K3  Folds     the V oscillator's frequency: one long fold (0) .. the clip .. ~5 fold rows (1023)
--   K4  Roll      bipolar: the fold rows roll up (right) or down (left), the bands creep with them
--   K5  Lean      a V ramp into the H phase: bands lean left (0) .. the clip (512) .. lean right (1023)
--   K6  Etch      how far the smoothed camera (or the Osc) bends the bands
--   S7  Mirror    the fan mirrored about the centre (chevrons)
--   S8  Wave      a 540-line V sine patched into the H phase: the bands wriggle
--   S9  Cross     the blue colouriser reads a reversed H ramp: blue bands lean against the orange
--   S10 Colours   Tiger (the clip) / Jungle (the session's other look: green floor, yellow glows)
--   S11 Source    Video / Osc
--   P12 SPLIT     the split's depth: one pink fan (0, the clip a second earlier) .. the clip (467) ..
--                 colour bands cycling down the frame (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each
-- line start (computes the NEXT line: V sine and fan by interpolated ROMs, the split, the H frequency and
-- the two start phases), two phase accumulators, a 16-stage pixel path, two colouriser term ROMs summed.
-- Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (Y, U and V
-- through two [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16) with the running-sum resync before the blanking
-- gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.tiger_pkg.all;
use work.all;

architecture tiger of program_top is
    constant C_LAT : integer := 16;   -- = processing_delay_clks

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

    -- line buffer: the smoothed camera (or Osc) line, (luma >> 2) - 16 through a 16 px box and a vertical
    -- IIR; read and written in one pass by the source side (the read runs 2 columns ahead of the write, so
    -- the addresses never meet)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    attribute no_rw_check : boolean;
    attribute no_rw_check of lb : signal is true;
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    signal s_hsr  : t_u8(0 to 15) := (others => (others => '0'));
    signal s_hsum : unsigned(11 downto 0) := (others => '0');
    signal s_hh   : std_logic_vector(15 downto 0) := (others => '0');
    signal s_ls, s_hb, s_lmp : unsigned(7 downto 0) := (others => '0');
    signal s_ca, s_cb, s_cc : unsigned(10 downto 0) := (others => '0');
    signal s_va, s_vb, s_vc, s_first : std_logic := '0';
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_sn_a : unsigned(7 downto 0) := (others => '0');
    signal r_sn_q : signed(15 downto 0) := (others => '0');
    signal r_fn_a : unsigned(7 downto 0) := (others => '0');
    signal r_fn_q : unsigned(15 downto 0) := (others => '0');
    signal r_cr_a, r_cb_a : unsigned(8 downto 0) := (others => '0');
    signal r_cr_q, r_cb_q : std_logic_vector(31 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 640 x 540 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_x0   : unsigned(8 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw roll, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(C_K5D * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(C_K6D * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(C_P12D * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(C_K4D, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vup : std_logic := '1';
    signal s_mirror, s_wave, s_cross, s_pal, s_src : std_logic := '0';

    -- per-field constants
    signal z_e1, z_e3 : signed(10 downto 0) := (others => '0');
    signal z_sh1, z_sh3 : signed(2 downto 0) := (others => '0');
    signal z_mt1, z_mt3 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_v2, z_p2, z_sq6 : unsigned(19 downto 0) := (others => '0');
    signal z_d5   : unsigned(8 downto 0) := (others => '0');          -- |k5 - 512|
    signal z_n5   : std_logic := '0';
    signal z_l    : unsigned(15 downto 0) := (others => '0');
    signal c_lv   : signed(16 downto 0) := to_signed(C_LEAN0, 17);    -- lean, Q24 per virtual line
    signal c_cl0  : unsigned(23 downto 0) := (others => '0');          -- H phase of line 0 (no drift)
    signal c_hacc : unsigned(23 downto 0) := (others => '0');          -- accumulated H drift
    signal c_kb   : unsigned(15 downto 0) := to_unsigned(C_KB0, 16);   -- H step per h, Q24 cycles
    signal c_kv   : unsigned(15 downto 0) := to_unsigned(C_KV0, 16);   -- V phase per virtual line, Q24
    signal c_af   : unsigned(14 downto 0) := (others => '0');          -- fan depth, Q12
    signal c_dvl  : unsigned(23 downto 0) := (others => '0');          -- V phase per line
    signal c_droll : unsigned(23 downto 0) := (others => '0');         -- V phase per field (roll)
    signal c_hd   : unsigned(23 downto 0) := (others => '0');          -- H phase per field (drift)
    signal c_a    : unsigned(12 downto 0) := (others => '0');          -- split depth, Q12
    signal c_eg0  : unsigned(16 downto 0) := (others => '0');
    signal c_eg   : unsigned(21 downto 0) := (others => '0');          -- etch gain, Q24 per luma step
    signal c_lst : unsigned(23 downto 0) := (others => '0');          -- H phase per line (lean)
    signal c_wvl  : unsigned(23 downto 0) := (others => '0');          -- wave phase per line
    signal c_vs   : unsigned(23 downto 0) := to_unsigned(C_VS0, 24);   -- V phase of line 0 (rolls)
    signal c_c0   : unsigned(23 downto 0) := to_unsigned(C_CC, 24);    -- H phase of line 0 (drifts)
    signal c_ws   : unsigned(23 downto 0) := to_unsigned(C_WS0, 24);   -- wave phase of line 0
    signal c_egp  : unsigned(14 downto 0) := (others => '0');          -- etch gain >> 7 (pixel path)
    signal c_ma   : unsigned(11 downto 0) := to_unsigned(C_XBH, 12);   -- theta A multiplicand (h)
    signal c_mb   : unsigned(12 downto 0) := to_unsigned(3840 + C_XBH, 13);

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk   : std_logic := '0';
    signal e_vph  : unsigned(23 downto 0) := (others => '0');
    signal e_cl   : unsigned(23 downto 0) := (others => '0');
    signal e_wph  : unsigned(23 downto 0) := (others => '0');
    signal e_fph  : unsigned(23 downto 0) := (others => '0');
    signal e_s0   : signed(16 downto 0) := (others => '0');
    signal e_d    : signed(16 downto 0) := (others => '0');
    signal e_neg  : std_logic := '0';
    signal e_iv   : signed(16 downto 0) := (others => '0');     -- interpolated table value
    signal e_f0   : unsigned(15 downto 0) := (others => '0');
    signal e_g    : unsigned(15 downto 0) := (others => '0');
    signal e_fd   : signed(17 downto 0) := (others => '0');
    signal e_sg   : std_logic := '0';
    signal e_w    : signed(13 downto 0) := (others => '0');
    signal e_bf   : unsigned(15 downto 0) := to_unsigned(4096, 16);
    signal e_f    : unsigned(18 downto 0) := (others => '0');
    signal e_pa, e_pb : unsigned(23 downto 0) := (others => '0');
    signal e_wv   : unsigned(23 downto 0) := (others => '0');
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(31 downto 0) := (others => '0');
    signal n_w    : unsigned(11 downto 0) := (others => '0');   -- next line: split (mod one cycle, Q12)
    signal n_wb   : unsigned(11 downto 0) := (others => '0');   -- next line: DB - split
    signal n_st   : unsigned(23 downto 0) := (others => '0');   -- next line: H step per pixel
    signal n_ta, n_tb : unsigned(23 downto 0) := (others => '0');

    -- beam (stage 1)
    signal b_x   : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_ta, b_tb, b_st : unsigned(23 downto 0) := (others => '0');
    signal b_dn  : std_logic := '0';
    signal l_w, l_wb : unsigned(11 downto 0) := (others => '0');

    -- pixel path (phases carried as bits 23 .. 7)
    signal p2_ta, p2_tb, p3_ta, p3_tb, p4_ta, p4_tb : unsigned(16 downto 0) := (others => '0');
    signal p2_lm  : unsigned(7 downto 0) := (others => '0');
    signal p3_ea, p3_eb : unsigned(18 downto 0) := (others => '0');
    signal p4_e   : unsigned(16 downto 0) := (others => '0');
    signal p5_pa, p5_pb : unsigned(16 downto 0) := (others => '0');
    signal p6_ta, p6_tb : unsigned(11 downto 0) := (others => '0');
    signal p8_cr, p8_cb, p9_cr, p9_cb : std_logic_vector(31 downto 0) := (others => '0');
    signal p10_y  : signed(11 downto 0) := (others => '0');
    signal p10_u, p10_v : signed(12 downto 0) := (others => '0');
    signal c_by   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal c_bu, c_bv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y13, s_u13, s_v13 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y16, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y17 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (read 2 columns ahead of the write: no collisions) and ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_TG_EXP2(to_integer(r_ex_a));
            r_sn_q <= C_TG_SIN(to_integer(r_sn_a));
            r_fn_q <= C_TG_FAN(to_integer(r_fn_a));
        end if;
    end process p_rom;

    p_crom : process(clk)
    begin
        if rising_edge(clk) then
            r_cr_q <= C_TG_CR(to_integer(r_cr_a));
            r_cb_q <= C_TG_CB(to_integer(r_cb_a));
        end if;
    end process p_crom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_k  : std_logic_vector(31 downto 0);
        variable v_t  : unsigned(23 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);

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
            elsif s_zc /= 2047 then
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
            -- roll: bipolar, +-8 deadband, |v| - 8
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
            if v_s > -9 and v_s < 9 then
                v_m := (others => '0');
            elsif v_s(10) = '1' then
                v_m := unsigned(not v_s(9 downto 0)) - 7;     -- |v| - 8
            else
                v_m := unsigned(v_s(9 downto 0)) - 8;
            end if;
            s_vm <= v_m(8 downto 0);

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_vup     <= not v_s(10);
                s_mirror  <= registers_in(6)(0);
                s_wave    <= registers_in(6)(1);
                s_cross   <= registers_in(6)(2);
                s_pal     <= registers_in(6)(3);
                s_src     <= registers_in(6)(4);
                s_xs      <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters (no zc decode on these enables)
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 4      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
                            when others => s_gm <= s_gp; s_gr <= unsigned(registers_in(7));
                        end case;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        case to_integer(z_gc) is
                            when 0      => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 1      => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 2      => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 3      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 5 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 40 + 24 k, 16 iterations, result stable from
            -- load + 17 (stores at load + 18).  The slot number is decoded at 38 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 39 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 38  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 62  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 86  => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 134 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 158 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 182 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 206 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 230 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 254 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 278 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 302 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when 326 => z_sel <= to_unsigned(12, 4); z_pre <= '1';
                    when 350 => z_sel <= to_unsigned(13, 4); z_pre <= '1';
                    when 374 => z_sel <= to_unsigned(14, 4); z_pre <= '1';
                    when 398 => z_sel <= to_unsigned(15, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(C_KB0, 36);           z_fb <= z_mt1;                 -- H step
                    when 1  => z_fa <= to_unsigned(C_KV0, 36);           z_fb <= z_mt3;                 -- V step
                    when 2  => z_fa <= resize(s_g2(12 downto 3), 36);    z_fb <= to_unsigned(C_AFL, 16);  -- fan
                    when 4  => z_fa <= resize(c_kv, 36);                 z_fb <= resize(s_ys, 16);      -- V per line
                    when 5  => z_fa <= resize(s_vm, 36);                 z_fb <= resize(s_vm, 16);      -- roll^2
                    when 6  => z_fa <= resize(z_v2, 36);                 z_fb <= c_kv;                  -- roll
                    when 7  => z_fa <= resize(z_v2, 36);                 z_fb <= to_unsigned(C_HDK / 2, 16); -- drift
                    when 8  => z_fa <= resize(s_gp(12 downto 3), 36);    z_fb <= resize(s_gp(12 downto 3), 16);
                    when 9  => z_fa <= resize(z_p2, 36);                 z_fb <= to_unsigned(C_APK, 16);  -- split
                    when 10 => z_fa <= resize(s_g6(12 downto 3), 36);    z_fb <= resize(s_g6(12 downto 3), 16);
                    when 11 => z_fa <= resize(z_sq6, 36);                z_fb <= to_unsigned(C_EGK, 16);  -- etch
                    when 12 => z_fa <= resize(z_d5, 36);                 z_fb <= to_unsigned(C_LEANK, 16); -- lean knob
                    when 13 => z_fa <= resize(unsigned(abs(c_lv)), 36);  z_fb <= resize(s_ys, 16);      -- lean per line
                    when 14 => z_fa <= resize(unsigned(abs(c_lv)), 36);  z_fb <= to_unsigned(C_Y0L, 16); -- lean pivot
                    when others => z_fa <= to_unsigned(C_KW, 36);        z_fb <= resize(s_ys, 16);      -- wave
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion: last field's steps
                    when 20 =>
                        if s_vup = '1' then
                            c_vs <= c_vs - c_droll; c_hacc <= c_hacc + c_hd;
                        else
                            c_vs <= c_vs + c_droll; c_hacc <= c_hacc - c_hd;
                        end if;
                        c_ws <= c_ws + to_unsigned(C_WROLL, 24);
                    when 21 =>
                        if s_mirror = '1' then
                            c_ma <= to_unsigned(1920 + C_XBH, 12) - resize(s_x0, 12);
                        else
                            c_ma <= resize(s_x0, 12) + to_unsigned(C_XBH, 12);
                        end if;
                        c_mb <= to_unsigned(3840 + C_XBH, 13) - resize(s_x0, 13);
                        if s_pal = '1' then v_k := C_TG_BASE(1); else v_k := C_TG_BASE(0); end if;
                        c_by <= unsigned(v_k(31 downto 22)); c_bu <= unsigned(v_k(21 downto 12)); c_bv <= unsigned(v_k(11 downto 2));
                    -- exponents (glides have settled by zc 26): K1 bands, K3 folds, x1/4 .. x4
                    when 27 =>
                        z_e1 <= signed(resize(s_g1(12 downto 3), 11)) - 512;
                    when 28 =>
                        if z_e1 > 0 then z_e1 <= z_e1 - shift_right(z_e1, 3) - shift_right(z_e1, 4); end if;
                        z_e3 <= signed(resize(s_g3(12 downto 3), 11)) - 512;
                    when 29 => r_ex_a <= unsigned(z_e1(7 downto 0)); z_sh1 <= z_e1(10 downto 8);
                    when 30 => r_ex_a <= unsigned(z_e3(7 downto 0)); z_sh3 <= z_e3(10 downto 8);
                    when 31 => z_mt1 <= r_ex_q;
                    when 32 => z_mt3 <= r_ex_q;
                    -- lean knob: |k5 - 512|, sign
                    when 33 =>
                        if s_g5(12 downto 3) >= 512 then z_d5 <= resize(s_g5(12 downto 3) - 512, 9); z_n5 <= '0';
                        else z_d5 <= resize(512 - s_g5(12 downto 3), 9); z_n5 <= '1'; end if;
                    -- products (slot k stores at 58 + 24 k)
                    when 58 =>                        -- kb = KB0 * mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh1) is
                            when -2     => c_kb <= f_acc(32 downto 17);
                            when -1     => c_kb <= f_acc(31 downto 16);
                            when 0      => c_kb <= f_acc(30 downto 15);
                            when others => c_kb <= f_acc(29 downto 14);
                        end case;
                    when 82 =>
                        case to_integer(z_sh3) is
                            when -2     => c_kv <= f_acc(32 downto 17);
                            when -1     => c_kv <= f_acc(31 downto 16);
                            when 0      => c_kv <= f_acc(30 downto 15);
                            when others => c_kv <= f_acc(29 downto 14);
                        end case;
                    when 106 => c_af  <= f_acc(20 downto 6);
                    when 154 => c_dvl <= resize(f_acc(31 downto 8), 24);
                    when 178 => z_v2  <= f_acc(19 downto 0);
                    when 202 => c_droll <= f_acc(35 downto 12);
                    when 226 => c_hd  <= resize(f_acc(35 downto 15), 24);
                    when 250 => z_p2  <= f_acc(19 downto 0);
                    when 274 => c_a   <= f_acc(32 downto 20);
                    when 298 => z_sq6 <= f_acc(19 downto 0);
                    when 322 => c_eg0 <= f_acc(32 downto 16);
                    when 323 =>
                        if s_src = '1' then
                            c_eg <= shift_left(resize(c_eg0, 22), 4) + shift_left(resize(c_eg0, 22), 3);  -- Osc: 24x
                        else
                            c_eg <= resize(c_eg0, 22);
                        end if;
                    when 324 => c_egp <= c_eg(21 downto 7);
                    when 346 => z_l <= f_acc(22 downto 7);                                     -- |k5 - 512| * 36.4
                    when 347 =>                       -- lean = LEAN0 - (k5 - 512) * 36.4 (+-2 cycles over the frame at the ends)
                        if z_n5 = '1' then c_lv <= to_signed(C_LEAN0, 17) + signed(resize(z_l, 17));
                        else c_lv <= to_signed(C_LEAN0, 17) - signed(resize(z_l, 17)); end if;
                    when 370 =>                       -- per line: lean * ys >> 8 (two's complement)
                        v_t := f_acc(31 downto 8);
                        if c_lv < 0 then c_lst <= 0 - v_t; else c_lst <= v_t; end if;
                    when 394 =>                       -- line 0: CC - lean * 881 (the fold row keeps CC)
                        v_t := f_acc(23 downto 0);
                        if c_lv < 0 then c_cl0 <= to_unsigned(C_CC, 24) + v_t; else c_cl0 <= to_unsigned(C_CC, 24) - v_t; end if;
                    when 395 => c_c0 <= c_cl0 + c_hacc;
                    when 418 => c_wvl  <= resize(f_acc(31 downto 8), 24);
                    when others => null;
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
    -- Source side: oscillator (2-D triangle lattice, drifting); the camera (or
    -- Osc) line smoothed: (luma >> 2) - 16, a 16 px running box (resynced to 0
    -- after 16 blank samples), then a vertical IIR against the previous smoothed
    -- line read from the line buffer (the first line of a field loads directly)
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_i : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
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
            -- A: the sample (0 off the source avid)
            if data_in.avid = '0' then
                s_ls <= (others => '0');
            elsif s_src = '1' then
                s_ls <= s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_ls <= v_l - 16; else s_ls <= (others => '0'); end if;
            end if;
            s_ca <= s_wcol; s_va <= data_in.avid;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
            -- B: 16 px running box; read the previous smoothed line at this column
            s_hsr(0) <= s_ls;
            for i in 1 to 15 loop s_hsr(i) <= s_hsr(i - 1); end loop;
            s_hh <= s_hh(14 downto 0) & s_va;
            if s_hh = x"0000" then
                s_hsum <= (others => '0');
            else
                s_hsum <= s_hsum + s_ls - s_hsr(15);
            end if;
            s_lb_ra <= s_ca; s_cb <= s_ca; s_vb <= s_va;
            -- C: box mean
            s_hb <= s_hsum(11 downto 4); s_cc <= s_cb; s_vc <= s_vb;
            -- D: IIR (1/4 new + 3/4 previous, rounded), write back; the previous line goes to the pixel path
            if s_first = '1' then
                s_lb_wd <= std_logic_vector(s_hb);
            else
                v_i := resize(s_hb, 10) + shift_left(resize(unsigned(s_lb_q), 10), 1) + resize(unsigned(s_lb_q), 10) + 2;
                s_lb_wd <= std_logic_vector(v_i(9 downto 2));
            end if;
            s_lb_wa <= s_cc; s_lb_we <= s_vc;
            s_lmp <= unsigned(s_lb_q);
            if s_zone = '1' then s_first <= '1'; elsif s_vc = '1' and s_vb = '0' then s_first <= '0'; end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line on one serial multiplier (16 iterations, result
    -- at load + 17): sin V and the fan shape by interpolated ROMs, the split
    -- w = A sin V, F = 1 + aF g, f = kb F, the step, and the start phases
    -- (x0 + xb) f + c and (3840 - x0 + xb) f + c (+ the wave)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d  : signed(16 downto 0);
        variable v_i  : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 440 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vph <= c_vs; e_cl <= c_c0; e_wph <= c_ws;
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vph <= e_vph - c_dvl; e_cl <= e_cl + c_lst; e_wph <= e_wph + c_wvl;
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(30 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                -- sin V: table entries i, i + 1
                when 1 => r_sn_a <= e_vph(23 downto 16);
                          e_fph <= e_vph - to_unsigned(C_DPH, 24);
                when 2 => r_sn_a <= e_vph(23 downto 16) + 1;
                when 3 => e_s0 <= resize(r_sn_q, 17);
                when 4 =>
                    v_d := resize(r_sn_q, 17) - e_s0;
                    if v_d < 0 then e_neg <= '1'; v_d := -v_d; else e_neg <= '0'; end if;
                    m_a <= resize(unsigned(v_d), 32); m_b <= resize(e_vph(15 downto 8), 16); m_acc <= (others => '0');
                    -- fan shape: entries at the offset phase
                    r_fn_a <= e_fph(23 downto 16);
                when 5 => r_fn_a <= e_fph(23 downto 16) + 1;
                when 6 => e_f0 <= r_fn_q;
                when 7 => e_fd <= signed(resize(r_fn_q, 18)) - signed(resize(e_f0, 18));
                when 21 =>                    -- sin V = s0 +- |d| fr >> 8 (Q15)
                    v_i := signed(resize(m_acc(23 downto 8), 17));
                    if e_neg = '1' then e_iv <= e_s0 - v_i; else e_iv <= e_s0 + v_i; end if;
                when 22 =>                    -- split: |sin| * A
                    if e_iv < 0 then e_sg <= '1'; m_a <= resize(unsigned(-e_iv), 32);
                    else e_sg <= '0'; m_a <= resize(unsigned(e_iv), 32); end if;
                    m_b <= resize(c_a, 16); m_acc <= (others => '0');
                when 39 =>                    -- w = +- (|sin| A) >> 15, Q12
                    if e_sg = '1' then e_w <= -signed(resize(m_acc(28 downto 15), 14));
                    else e_w <= signed(resize(m_acc(28 downto 15), 14)); end if;
                    -- fan interpolation
                    if e_fd < 0 then e_neg <= '1'; m_a <= resize(unsigned(-e_fd), 32);
                    else e_neg <= '0'; m_a <= resize(unsigned(e_fd), 32); end if;
                    m_b <= resize(e_fph(15 downto 8), 16); m_acc <= (others => '0');
                when 40 =>
                    n_w  <= unsigned(e_w(11 downto 0));
                    n_wb <= to_unsigned(C_DB, 12) - unsigned(e_w(11 downto 0));
                when 56 =>                    -- g = f0 +- |fd| fr >> 8 (Q16)
                    if e_neg = '1' then e_g <= e_f0 - m_acc(23 downto 8);
                    else e_g <= e_f0 + m_acc(23 downto 8); end if;
                when 57 =>
                    m_a <= resize(e_g, 32); m_b <= resize(c_af, 16); m_acc <= (others => '0');
                when 74 =>                    -- F = 1 + aF g (Q12)
                    e_bf <= to_unsigned(4096, 16) + m_acc(31 downto 16);
                when 75 =>
                    m_a <= resize(c_kb, 32); m_b <= e_bf; m_acc <= (others => '0');
                when 92 =>                    -- f = kb F >> 12 (Q24 cycles per h)
                    e_f <= m_acc(30 downto 12);
                when 93 =>
                    case to_integer(s_xs) is
                        when 4      => n_st <= shift_left(resize(e_f, 24), 2);
                        when 3      => n_st <= shift_left(resize(e_f, 24), 1) + resize(e_f, 24);
                        when others => n_st <= shift_left(resize(e_f, 24), 1);
                    end case;
                    m_a <= resize(e_f, 32); m_b <= resize(c_ma, 16); m_acc <= (others => '0');
                    -- wave: table entries
                    r_sn_a <= e_wph(23 downto 16);
                when 94 => r_sn_a <= e_wph(23 downto 16) + 1;
                when 95 => e_s0 <= resize(r_sn_q, 17);
                when 96 => e_d <= resize(r_sn_q, 17) - e_s0;
                when 110 =>
                    e_pa <= m_acc(23 downto 0);
                    m_a <= resize(e_f, 32); m_b <= resize(c_mb, 16); m_acc <= (others => '0');
                when 127 =>
                    e_pb <= m_acc(23 downto 0);
                    if e_d < 0 then e_neg <= '1'; m_a <= resize(unsigned(-e_d), 32);
                    else e_neg <= '0'; m_a <= resize(unsigned(e_d), 32); end if;
                    m_b <= resize(e_wph(15 downto 8), 16); m_acc <= (others => '0');
                when 144 =>                   -- sin W
                    v_i := signed(resize(m_acc(23 downto 8), 17));
                    if e_neg = '1' then e_iv <= e_s0 - v_i; else e_iv <= e_s0 + v_i; end if;
                when 145 =>
                    if e_iv < 0 then e_sg <= '1'; m_a <= resize(unsigned(-e_iv), 32);
                    else e_sg <= '0'; m_a <= resize(unsigned(e_iv), 32); end if;
                    m_b <= to_unsigned(C_WD, 16); m_acc <= (others => '0');
                when 162 =>                   -- wave offset = +- (|sin| WD) >> 3 (Q24)
                    if s_wave = '0' then
                        e_wv <= (others => '0');
                    elsif e_sg = '1' then
                        e_wv <= 0 - m_acc(26 downto 3);
                    else
                        e_wv <= m_acc(26 downto 3);
                    end if;
                when 163 => e_wv <= e_wv + e_cl;               -- the line's phase offset
                when 164 =>
                    n_ta <= e_pa + e_wv;
                    n_tb <= e_pb + e_wv;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the two phase accumulators (stage 1).  A runs up (or, mirrored,
    -- down to the centre and up again); B is A, or with Cross a reversed ramp.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= resize(s_x0, 12);
                b_px <= (others => '0');
                b_ta <= n_ta; b_tb <= n_tb; b_st <= n_st;
                l_w  <= n_w; l_wb <= n_wb;
                b_dn <= s_mirror;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x + s_xs;
                if b_dn = '1' then b_ta <= b_ta - b_st; else b_ta <= b_ta + b_st; end if;
                b_tb <= b_tb - b_st;
                -- mirrored: the step after the first pixel at x >= 1920 h runs up
                if b_x + s_xs >= 1920 then b_dn <= '0'; end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage k = the pixel's value is valid in cycle k)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_fa, v_fb : unsigned(11 downto 0);
        variable v_y  : signed(11 downto 0);
        variable v_u, v_v : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: phases of the pixel, bits 23 .. 7 (B = A unless Cross); the camera luma (a pixel or two
            -- behind: invisible in a smooth displacement)
            p2_ta <= b_ta(23 downto 7);
            if s_cross = '1' then p2_tb <= b_tb(23 downto 7); else p2_tb <= b_ta(23 downto 7); end if;
            p2_lm <= s_lmp;
            -- 3: etch partials (luma nibbles x gain >> 7)
            p3_ea <= resize(p2_lm(3 downto 0) * c_egp, 19);
            p3_eb <= resize(p2_lm(7 downto 4) * c_egp, 19);
            p3_ta <= p2_ta; p3_tb <= p2_tb;
            -- 4: etch (mod one cycle)
            p4_e  <= resize(p3_ea, 17) + shift_left(resize(p3_eb(12 downto 0), 17), 4);
            p4_ta <= p3_ta; p4_tb <= p3_tb;
            -- 5: bent phases
            p5_pa <= p4_ta + p4_e;
            p5_pb <= p4_tb + p4_e;
            -- 6: triangle T = 2 f (f < 1/2) or 2 (1 - f) (12 bits, no arithmetic)
            v_fa := p5_pa(16 downto 5); v_fb := p5_pb(16 downto 5);
            if v_fa(11) = '0' then p6_ta <= v_fa(10 downto 0) & '0'; else p6_ta <= (not v_fa(10 downto 0)) & '1'; end if;
            if v_fb(11) = '0' then p6_tb <= v_fb(10 downto 0) & '0'; else p6_tb <= (not v_fb(10 downto 0)) & '1'; end if;
            -- 7: colouriser phases uR = T + w, uB = T - w + DB (mod one cycle) -> term ROM addresses
            v_fa := p6_ta + l_w;
            v_fb := p6_tb + l_wb;
            r_cr_a <= s_pal & v_fa(11 downto 4);
            r_cb_a <= s_pal & v_fb(11 downto 4);
            -- 8: ROM data (EBR); 9: fabric copy
            p8_cr <= r_cr_q; p8_cb <= r_cb_q;
            p9_cr <= p8_cr; p9_cb <= p8_cb;
            -- 10: base + red term + blue term (Y 10-bit, U / V 11-bit terms, two's complement)
            p10_y <= signed(resize(c_by, 12)) + resize(signed(p9_cr(31 downto 22)), 12) + resize(signed(p9_cb(31 downto 22)), 12);
            p10_u <= signed(resize(c_bu, 13)) + resize(signed(p9_cr(21 downto 11)), 13) + resize(signed(p9_cb(21 downto 11)), 13);
            p10_v <= signed(resize(c_bv, 13)) + resize(signed(p9_cr(10 downto 0)), 13) + resize(signed(p9_cb(10 downto 0)), 13);
            -- 11: clamp (luma 64 .. 940, chroma 64 .. 960) and compose; black outside the generated avid
            v_y := p10_y; v_u := p10_u; v_v := p10_v;
            if s_gav_sr(8) = '0' then
                s_y11 <= to_unsigned(64, 10); s_u11 <= to_unsigned(512, 10); s_v11 <= to_unsigned(512, 10);
            else
                if v_y < 64 then s_y11 <= to_unsigned(64, 10);
                elsif v_y > 940 then s_y11 <= to_unsigned(940, 10);
                else s_y11 <= unsigned(v_y(9 downto 0)); end if;
                if v_u < 64 then s_u11 <= to_unsigned(64, 10);
                elsif v_u > 960 then s_u11 <= to_unsigned(960, 10);
                else s_u11 <= unsigned(v_u(9 downto 0)); end if;
                if v_v < 64 then s_v11 <= to_unsigned(64, 10);
                elsif v_v > 960 then s_v11 <= to_unsigned(960, 10);
                else s_v11 <= unsigned(v_v(9 downto 0)); end if;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(8);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the clip's softness, and the HD HDMI bar fix),
    -- luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to black (the off-avid input of both boxes) once 8 blank
            -- samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 12 sums, stage 13 output
            s_cay(0) <= s_y11; s_cau(0) <= s_u11; s_cav(0) <= s_v11;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= to_unsigned(64 * 8, 13); s_bau <= to_unsigned(512 * 8, 13); s_bav <= to_unsigned(512 * 8, 13);
            else
                s_bay <= s_bay + s_y11 - s_cay(7);
                s_bau <= s_bau + s_u11 - s_cau(7);
                s_bav <= s_bav + s_v11 - s_cav(7);
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(10) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y13 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u13 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v13 <= v_sc(13 downto 4);
            else
                s_y13 <= to_unsigned(64, 10); s_u13 <= to_unsigned(512, 10); s_v13 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(10);
            -- box B: stage 14 sums, stage 15 output
            s_cby(0) <= s_y13; s_cbu(0) <= s_u13; s_cbv(0) <= s_v13;
            for i in 1 to 7 loop s_cby(i) <= s_cby(i - 1); s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(64 * 8, 13); s_bbu <= to_unsigned(512 * 8, 13); s_bbv <= to_unsigned(512 * 8, 13);
            else
                s_bby <= s_bby + s_y13 - s_cby(7);
                s_bbu <= s_bbu + s_u13 - s_cbu(7);
                s_bbv <= s_bbv + s_v13 - s_cbv(7);
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y15 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u15 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v15 <= v_sc(13 downto 4);
            -- 16: luma 13/16
            if s_y15 > 64 then v_dl := s_y15 - 64; else v_dl := (others => '0'); end if;
            s_y16 <= s_y15 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u16 <= s_u15; s_v16 <= s_v15;
            -- 17: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y17 <= s_y16; s_u17 <= s_u16; s_v17 <= s_v16;
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y17);
    data_out.u       <= std_logic_vector(s_u17);
    data_out.v       <= std_logic_vector(s_v17);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture tiger;
