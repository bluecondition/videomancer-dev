-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Lentil v0.1.0 -- Heirloom (phase 3, video clips), rebuilt from the clip WIN_20230501_01_34_42 at 0.5 s.
--
-- The still: a band of solid blue lenses (rounded, lumpy diamonds, 940 px apart, pinched to a point between
-- them) across the middle of the frame, the pink hourglass gaps between them filled with sawtooth stripes
-- (purple -> pink -> yellow) that leave each lens parallel to its edge and then fan out flat; above and below
-- the band the same stripes run free as ")" chevrons; thin shifted strips and a dashed line at the band edges.
-- In the clip the whole patch breathes at 1.6 Hz: the pinches close and open, the band edges and the chevron
-- tips move with them.
--
-- The patch, as fitted (lentil.py has every constant):
--   lens    = blue where |e| < W(row), e = position in the lens period, W from the inverse lens profile at
--             v = |y - 518| / sigma - c  (sigma = band height, c = pinch threshold + breathing)
--   stripes = a VCO at f(y) = fs (1 + 1.14e-3 hyp(| |y - 527| - 469 |)) (the chevrons), colourised by the
--             still's own saw palette; in the band the phase is re-seeded at every lens exit to
--             0.05 + Fan * 2 W and runs with a lag tail Gt e^(-d / 32) on top of f(y)
--   fringe  = a 26 px dark-purple ramp outside each lens
--   camera  = luma (or the Osc lattice) frequency-modulates the stripe VCO
--
--   K1  Stripes   stripe density x1/4 .. x4 (the still at 512)
--   K2  Fan       how hard the stripes are flung out of each lens (FM depth (k / 512)^2: 0 .. x4)
--   K3  Lenses    lens period: 0.6 .. 6.8 lenses across (2 at 512)
--   K4  Breath    breathing rate 0.1 .. 6.4 Hz (1.59 Hz = the clip at 512)
--   K5  Pinch     lens threshold: seeds (0) .. the still (512) .. a blue river (1023)
--   K6  Bend      how far the camera (or the Osc) bends the stripes
--   S7  Shape     Lumpy (the still's lens) / Round (a smooth sine lens)
--   S8  Fill      Solid blue / Ghost (the hidden fast stripes show through the lens)
--   S9  Chevrons  Bow (the still) / Wave (zigzag)
--   S10 Colours   Flamingo (the still) / Lagoon
--   S11 Source    Video / Osc
--   P12 BAND      band height: a seam of tiny lentils (0) .. the still (650) .. lenses filling the frame (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each
-- line start (computes the NEXT line: band flags, lens half-width, chevron frequency, exit phase, line-start
-- phase), a 15-stage pixel path (lens phase DDA, inside / exit events, lag tail, stripe phase accumulator,
-- palette ROM, fringe / fill blend).  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated
-- avid; output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the running-sum
-- resync, luma 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.lentil_pkg.all;
use work.all;

architecture lentil of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

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

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_pf_a, r_ch_a : unsigned(9 downto 0) := (others => '0');
    signal r_pf_q, r_ch_q : unsigned(15 downto 0) := (others => '0');
    signal r_pl_a : unsigned(8 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

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

    -- knobs: glided (Q3), raw breath rate, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(384 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(650 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal z_gp : std_logic_vector(1 downto 0) := "00";
    signal z_gc : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_shape, s_ghost, s_wave, s_pal, s_src : std_logic := '0';

    -- sequencer scratch
    signal z_e    : signed(10 downto 0) := (others => '0');
    signal z_d    : signed(10 downto 0) := (others => '0');
    signal z_sh1, z_sh3, z_sh3n, z_sh4 : signed(2 downto 0) := (others => '0');
    signal z_mt1, z_mt3, z_mt3n, z_mt4 : unsigned(15 downto 0) := to_unsigned(32768, 16);

    -- per-field constants
    signal c_fs    : unsigned(16 downto 0) := to_unsigned(27170, 17);     -- stripe base frequency, Q24 cycles per h
    signal c_dlh   : unsigned(14 downto 0) := to_unsigned(8924, 15);      -- lens phase step per h, Q24 periods
    signal c_pdh   : unsigned(12 downto 0) := to_unsigned(1880, 13);      -- lens period in h
    signal c_dl    : unsigned(16 downto 0) := to_unsigned(17848, 17);     -- lens phase step per pixel
    signal c_p0    : unsigned(23 downto 0) := (others => '0');            -- lens phase at the beam start
    signal c_ae0   : unsigned(16 downto 0) := (others => '0');            -- |e| at the beam start
    signal c_e3    : signed(10 downto 0) := (others => '0');
    signal c_fsh   : unsigned(3 downto 0) := to_unsigned(8, 4);          -- fringe: 7 << fsh lens units ~ 26 px
    signal c_g2sq  : unsigned(19 downto 0) := to_unsigned(262144, 20);
    signal c_kq    : unsigned(17 downto 0) := to_unsigned(37749, 18);
    signal c_kwpd  : unsigned(30 downto 0) := (others => '0');            -- Fan * period, Q24 cycles per unit W/32768
    signal c_gtpix : unsigned(21 downto 0) := (others => '0');            -- lag tail start, per pixel
    signal c_gttot : unsigned(25 downto 0) := (others => '0');            -- lag tail area
    signal c_klpix : unsigned(10 downto 0) := (others => '0');            -- luma FM per luma step per pixel
    signal c_ginpix : unsigned(18 downto 0) := to_unsigned(201326, 19);   -- ghost stripes inside the lens
    signal c_rate  : unsigned(15 downto 0) := (others => '0');            -- breathing, Q16 cycles per field
    signal c_phi   : unsigned(15 downto 0) := to_unsigned(62528, 16);     -- breathing phase (the still at start)
    signal c_par   : unsigned(10 downto 0) := (others => '0');
    signal c_cbq   : signed(9 downto 0) := (others => '0');               -- band offset, Q4 lines (-8 .. +16)
    signal c_cq    : signed(12 downto 0) := (others => '0');              -- lens threshold, Q4 lines
    signal c_sig, c_isig : unsigned(14 downto 0) := to_unsigned(4096, 15);  -- band height and 1/height, Q12
    signal c_et, c_eb : unsigned(14 downto 0) := to_unsigned(5728, 15);   -- band edges (|y - 518|), Q4 lines
    signal c_x0e   : unsigned(11 downto 0) := to_unsigned(16, 12);        -- beam start x (h)
    signal c_xr    : unsigned(11 downto 0) := to_unsigned(796, 12);       -- beam start + the 390 px pre-roll (h)
    signal c_dec4  : std_logic := '0';                                    -- SD: tail decays 1/16 per pixel

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';

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
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_ay   : unsigned(17 downto 0) := (others => '0');
    signal e_top  : std_logic := '0';
    signal e_yv   : unsigned(10 downto 0) := (others => '0');
    signal e_m    : unsigned(12 downto 0) := (others => '0');
    signal e_vq   : signed(18 downto 0) := (others => '0');
    signal e_full, e_zero, e_band, e_strip, e_eline, e_ins0 : std_logic := '0';
    signal e_W    : unsigned(15 downto 0) := (others => '0');
    signal e_fbh  : unsigned(17 downto 0) := (others => '0');
    signal e_fbstep : unsigned(19 downto 0) := (others => '0');
    signal e_phx  : unsigned(23 downto 0) := (others => '0');
    signal e_q    : unsigned(23 downto 0) := (others => '0');
    signal e_dist : unsigned(12 downto 0) := (others => '0');
    signal m_a    : unsigned(39 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(39 downto 0) := (others => '0');
    -- next line (loaded at g_start) and this line
    signal n_W, l_W : unsigned(15 downto 0) := (others => '0');
    signal n_full, l_full, n_band, l_band, n_eline, l_eline : std_logic := '0';
    signal n_fbstep, l_fbstep : unsigned(19 downto 0) := (others => '0');
    signal n_phx, l_phx, n_ph0, l_ph0 : unsigned(23 downto 0) := (others => '0');

    -- beam
    signal b_lp    : unsigned(23 downto 0) := (others => '0');
    signal b_px    : unsigned(10 downto 0) := (others => '0');
    signal b_first : std_logic := '0';

    -- pixel path (stage k registers hold pixel n while the beam holds pixel n + k - 1)
    signal p2_ae   : unsigned(16 downto 0) := (others => '0');
    signal p2_first, p3_first, p4_first, p5_first, p6_first, p7_first : std_logic := '0';
    signal p3_ins, p4_ins, p5_ins, p6_ins, p7_ins, p8_ins, p9_ins, p10_ins, p11_ins : std_logic := '0';
    signal p3_dout : signed(17 downto 0) := (others => '0');
    signal p4_exit, p5_exit, p6_exit, p7_exit : std_logic := '0';
    signal p4_entry, p5_entry, p6_entry, p7_entry : std_logic := '0';
    signal p4_w, p5_w, p6_w, p7_w, p8_w, p9_w, p10_w, p11_w : unsigned(2 downto 0) := (others => '0');
    signal p4_llo, p4_lhi : unsigned(14 downto 0) := (others => '0');
    signal p5_g    : unsigned(21 downto 0) := (others => '0');
    signal p5_l    : unsigned(18 downto 0) := (others => '0');
    signal p6_t    : unsigned(22 downto 0) := (others => '0');
    signal p7_st   : unsigned(23 downto 0) := (others => '0');
    signal p8_ph   : unsigned(23 downto 0) := (others => '0');
    signal p11_q   : std_logic_vector(31 downto 0) := (others => '0');
    signal p12_ay, p12_au, p12_av, p12_by, p12_bu, p12_bv : unsigned(9 downto 0) := (others => '0');
    signal p13_ay, p13_au, p13_av, p14_ay, p14_au, p14_av : unsigned(9 downto 0) := (others => '0');
    signal p12_w, p13_w : unsigned(2 downto 0) := (others => '0');
    signal p13_dy, p13_du, p13_dv : signed(10 downto 0) := (others => '0');
    signal p14_py, p14_pu, p14_pv : signed(13 downto 0) := (others => '0');
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 inputs of boxes A, B
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept) and ROMs
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
            r_ex_q <= C_LT_EXP2(to_integer(r_ex_a));
            r_pf_q <= C_LT_PROF(to_integer(r_pf_a));
            r_ch_q <= C_LT_CHEV(to_integer(r_ch_a));
            r_pl_q <= C_LT_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_e  : signed(10 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_s  : signed(14 downto 0);
        variable v_x  : signed(16 downto 0);
        variable v_f5 : signed(4 downto 0);
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
            s_k4 <= unsigned(registers_in(3));

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_shape <= registers_in(6)(0);
                s_ghost <= registers_in(6)(1);
                s_wave  <= registers_in(6)(2);
                s_pal   <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4 =>
                        s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                        c_ginpix <= to_unsigned(402652, 19); c_dec4 <= '1';
                    when 3 =>
                        s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                        c_ginpix <= to_unsigned(301989, 19); c_dec4 <= '0';
                    when others =>
                        s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
                        c_ginpix <= to_unsigned(201326, 19); c_dec4 <= '0';
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel
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

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result read at 50 + 24 k.  The
            -- slot number is decoded at 30 + 24 k and the operands pre-muxed into z_fa / z_fb at 31 + 24 k.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(12, 4); z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(13, 4); z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(14, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(27170, 36);           z_fb <= z_mt1;                  -- stripes
                    when 1  => z_fa <= to_unsigned(8924, 36);            z_fb <= z_mt3;                  -- lens step
                    when 2  => z_fa <= to_unsigned(1880, 36);            z_fb <= z_mt3n;                 -- lens period
                    when 3  => z_fa <= resize(c_dlh, 36);                z_fb <= resize(to_unsigned(2016, 12) - c_x0e, 16);
                    when 4  => z_fa <= resize(s_g2(12 downto 3), 36);    z_fb <= resize(s_g2(12 downto 3), 16);  -- fan^2
                    when 5  => z_fa <= to_unsigned(37749, 36);           z_fb <= c_g2sq(19 downto 4);
                    when 6  => z_fa <= resize(c_kq, 36);                 z_fb <= resize(c_pdh, 16);
                    when 7  => z_fa <= to_unsigned(209715, 36);          z_fb <= c_g2sq(19 downto 4);
                    when 8  => z_fa <= resize(s_g6(12 downto 3), 36);    z_fb <= resize(s_g6(12 downto 3), 16);  -- bend^2
                    when 9  => z_fa <= to_unsigned(639, 36);             z_fb <= c_g2sq(19 downto 4);   -- (g2sq holds bend^2 here)
                    when 10 => z_fa <= to_unsigned(1737, 36);            z_fb <= z_mt4;                  -- breath rate
                    when 11 => z_fa <= resize(c_phi(14 downto 0), 36);   z_fb <= resize(to_unsigned(32768, 16) - resize(c_phi(14 downto 0), 16), 16);
                    when 12 => z_fa <= resize(c_par, 36);                z_fb <= to_unsigned(192, 16);
                    when 13 => z_fa <= unsigned(resize(to_signed(5728, 14) + resize(c_cbq, 14), 36)); z_fb <= resize(c_sig, 16);
                    when others => z_fa <= unsigned(resize(to_signed(5552, 14) + resize(c_cbq, 14), 36)); z_fb <= resize(c_sig, 16);
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
                    -- breathing phase (once per field); exponents and exp2 lookups (address at k, data at k + 2)
                    when 20 =>
                        c_phi <= c_phi + c_rate;
                        z_e <= signed(resize(s_g1(12 downto 3), 11)) - 512;               -- stripes: +-2 octaves
                        z_d <= signed(resize(s_g3(12 downto 3), 11)) - 512;
                        c_x0e <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
                    when 21 =>
                        r_ex_a <= unsigned(z_e(7 downto 0)); z_sh1 <= z_e(10 downto 8);
                        z_e <= z_d - shift_right(z_d, 3);                                   -- lenses: 7/8 octave per 256
                    when 22 =>
                        c_e3 <= z_e;
                        c_xr <= c_x0e + to_unsigned(780, 12);
                    when 23 =>
                        z_mt1 <= r_ex_q;
                        r_ex_a <= unsigned(c_e3(7 downto 0)); z_sh3 <= c_e3(10 downto 8);
                        v_e := c_e3 + 128;
                        v_f5 := to_signed(8, 5) + resize(shift_right(v_e, 8), 5);
                        c_fsh <= unsigned(v_f5(3 downto 0));
                    when 24 =>
                        z_e <= -c_e3;
                    when 25 =>
                        z_mt3 <= r_ex_q;
                        r_ex_a <= unsigned(z_e(7 downto 0)); z_sh3n <= z_e(10 downto 8);
                        v_d := signed(resize(s_k4, 11)) - 512;                              -- breath rate
                        if v_d(10) = '1' then z_e <= shift_left(v_d, 1); else z_e <= v_d; end if;
                    when 27 =>
                        z_mt3n <= r_ex_q;
                        r_ex_a <= unsigned(z_e(7 downto 0)); z_sh4 <= z_e(10 downto 8);
                        z_d <= signed(resize(s_gp(12 downto 3), 11)) - 650;                 -- band
                    when 28 =>
                        z_e <= z_d + shift_right(z_d, 3);
                    when 29 =>
                        z_mt4 <= r_ex_q;
                        z_e <= z_e + shift_right(z_d, 5);
                    when 30 =>
                        r_ex_a <= unsigned(z_e(7 downto 0)); z_sh1 <= z_e(10 downto 8);     -- (z_sh1 reused after slot 0 pre)
                        z_e <= -z_e;
                    when 32 =>                    -- sigma = 4096 * 2^e (Q12)
                        case to_integer(z_sh1) is
                            when -4     => c_sig <= resize(shift_right(r_ex_q, 7), 15);
                            when -3     => c_sig <= resize(shift_right(r_ex_q, 6), 15);
                            when -2     => c_sig <= resize(shift_right(r_ex_q, 5), 15);
                            when -1     => c_sig <= resize(shift_right(r_ex_q, 4), 15);
                            when 0      => c_sig <= resize(shift_right(r_ex_q, 3), 15);
                            when 1      => c_sig <= resize(shift_right(r_ex_q, 2), 15);
                            when others => c_sig <= resize(shift_right(r_ex_q, 1), 15);
                        end case;
                        r_ex_a <= unsigned(z_e(7 downto 0)); z_sh1 <= z_e(10 downto 8);
                    when 34 =>                    -- 1/sigma
                        case to_integer(z_sh1) is
                            when -4     => c_isig <= resize(shift_right(r_ex_q, 7), 15);
                            when -3     => c_isig <= resize(shift_right(r_ex_q, 6), 15);
                            when -2     => c_isig <= resize(shift_right(r_ex_q, 5), 15);
                            when -1     => c_isig <= resize(shift_right(r_ex_q, 4), 15);
                            when 0      => c_isig <= resize(shift_right(r_ex_q, 3), 15);
                            when 1      => c_isig <= resize(shift_right(r_ex_q, 2), 15);
                            when 2      => c_isig <= resize(shift_right(r_ex_q, 1), 15);
                            when others => c_isig <= resize(r_ex_q, 15);
                        end case;
                        -- the stripes' shift (z_sh1 was overwritten: recompute from g1)
                        v_e := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        z_sh1 <= v_e(10 downto 8);

                    -- slot results
                    when 50 =>                    -- fs = 27170 * mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh1) is
                            when -2     => c_fs <= resize(f_acc(35 downto 17), 17);
                            when -1     => c_fs <= resize(f_acc(35 downto 16), 17);
                            when 0      => c_fs <= resize(f_acc(35 downto 15), 17);
                            when others => c_fs <= resize(f_acc(35 downto 14), 17);
                        end case;
                    when 74 =>                    -- dlh = 8924 * mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh3) is
                            when -2     => c_dlh <= resize(f_acc(35 downto 17), 15);
                            when -1     => c_dlh <= resize(f_acc(35 downto 16), 15);
                            when 0      => c_dlh <= resize(f_acc(35 downto 15), 15);
                            when others => c_dlh <= resize(f_acc(35 downto 14), 15);
                        end case;
                    when 75 =>
                        case to_integer(s_xs) is
                            when 4      => c_dl <= resize(shift_left(resize(c_dlh, 17), 2), 17);
                            when 3      => c_dl <= resize(shift_left(resize(c_dlh, 17), 1), 17) + resize(c_dlh, 17);
                            when others => c_dl <= resize(shift_left(resize(c_dlh, 17), 1), 17);
                        end case;
                    when 98 =>                    -- pdh = 1880 * mt >> (15 - sh)
                        case to_integer(z_sh3n) is
                            when -2     => c_pdh <= resize(f_acc(35 downto 17), 13);
                            when -1     => c_pdh <= resize(f_acc(35 downto 16), 13);
                            when 0      => c_pdh <= resize(f_acc(35 downto 15), 13);
                            when others => c_pdh <= resize(f_acc(35 downto 14), 13);
                        end case;
                    when 122 =>                   -- p0 = -(2016 - x0e) * dlh, mod one period
                        c_p0 <= (not f_acc(23 downto 0)) + 1;
                    when 123 =>
                        v_x := resize(signed(c_p0(23 downto 8)), 17);
                        c_ae0 <= unsigned(abs(v_x));
                    when 146 => c_g2sq <= f_acc(19 downto 0);
                    when 170 => c_kq   <= f_acc(31 downto 14);
                    when 194 => c_kwpd <= f_acc(30 downto 0);
                    when 218 =>                   -- tail: Gt per h = 209715 * fan^2; per pixel and the area (x 64 h)
                        case to_integer(s_xs) is
                            when 4      => c_gtpix <= resize(shift_left(resize(f_acc(33 downto 14), 22), 2), 22);
                            when 3      => c_gtpix <= resize(shift_left(resize(f_acc(33 downto 14), 22), 1), 22) + resize(f_acc(33 downto 14), 22);
                            when others => c_gtpix <= resize(shift_left(resize(f_acc(33 downto 14), 22), 1), 22);
                        end case;
                        c_gttot <= f_acc(33 downto 14) & "000000";
                    when 242 => c_g2sq <= f_acc(19 downto 0);         -- bend^2 (the fan^2 copy is no longer needed)
                    when 266 =>                   -- luma FM per pixel = kl * xs / 2
                        case to_integer(s_xs) is
                            when 4      => c_klpix <= resize(shift_left(resize(f_acc(25 downto 16), 11), 1), 11);
                            when 3      => c_klpix <= resize(f_acc(25 downto 16), 11) + resize(f_acc(25 downto 17), 11);
                            when others => c_klpix <= resize(f_acc(25 downto 16), 11);
                        end case;
                    when 290 =>                   -- rate = 1737 * mt >> (15 - sh), sh = -4 .. 1
                        case to_integer(z_sh4) is
                            when -4     => c_rate <= resize(f_acc(35 downto 19), 16);
                            when -3     => c_rate <= resize(f_acc(35 downto 18), 16);
                            when -2     => c_rate <= resize(f_acc(35 downto 17), 16);
                            when -1     => c_rate <= resize(f_acc(35 downto 16), 16);
                            when 0      => c_rate <= resize(f_acc(35 downto 15), 16);
                            when others => c_rate <= resize(f_acc(35 downto 14), 16);
                        end case;
                    when 314 => c_par <= f_acc(28 downto 18);
                    when 338 =>                   -- band offset: 4 lines + 12 sin (Q4)
                        if c_phi(15) = '0' then
                            c_cbq <= to_signed(64, 10) + signed(resize(f_acc(17 downto 10), 10));
                        else
                            c_cbq <= to_signed(64, 10) - signed(resize(f_acc(17 downto 10), 10));
                        end if;
                    when 339 =>                   -- lens threshold = band offset + 7.5 (k5 - 512) / 16 lines
                        v_s := signed(resize(s_g5(12 downto 3), 15)) - 512;
                        v_s := shift_right(shift_left(v_s, 4) - v_s, 1);
                        c_cq <= resize(c_cbq, 13) + v_s(12 downto 0);
                    when 362 => c_et <= f_acc(26 downto 12);
                    when 386 => c_eb <= f_acc(26 downto 12);
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
    -- Source side: oscillator (2-D triangle lattice, drifting), line-buffer
    -- write of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
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
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then
                s_lb_wd <= std_logic_vector(s_osum);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line on one serial multiplier (16 iterations, result at
    -- load + 17): band / strip / dashed-edge flags, the lens half-width W, the
    -- chevron frequency, the lens-exit phase and the line-start phase
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d : signed(19 downto 0);
        variable v_v : signed(11 downto 0);
        variable v_u : unsigned(10 downto 0);
        variable v_w : unsigned(5 downto 0);
        variable v_t : unsigned(5 downto 0);
        variable v_g : unsigned(14 downto 0);
        variable v_f : unsigned(17 downto 0);
        variable v_a : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(38 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    v_d := signed(resize(e_vy, 20)) - 132608;            -- 518 lines, Q8
                    if v_d < 0 then v_d := -v_d; e_top <= '1'; else e_top <= '0'; end if;
                    e_ay <= unsigned(v_d(17 downto 0));
                    e_yv <= e_vy(18 downto 8);
                when 2 =>
                    m_a <= resize(e_ay, 40); m_b <= resize(c_isig, 16); m_acc <= (others => '0');
                    -- chevron index: Bow = | |y - 527| - 469 |, Wave = 5 |((y - 527) mod 64) - 32|
                    v_v := signed(resize(e_yv, 12)) - 527;
                    if s_wave = '1' then
                        v_w := unsigned(v_v(5 downto 0));
                        if v_w >= 32 then v_t := v_w - 32; else v_t := 32 - v_w; end if;
                        v_u := shift_left(resize(v_t, 11), 2) + resize(v_t, 11);
                    else
                        v_a := abs(v_v);
                        v_u := unsigned(v_a(10 downto 0));
                        if v_u >= 469 then v_u := v_u - 469; else v_u := 469 - v_u; end if;
                    end if;
                    if v_u > 1023 then r_ch_a <= (others => '1'); else r_ch_a <= v_u(9 downto 0); end if;
                when 3 =>                         -- band, strip and dashed-edge flags (Q4 lines)
                    if e_top = '1' then v_g := c_et; else v_g := c_eb; end if;
                    if e_ay(17 downto 4) < resize(v_g, 14) then e_band <= '1'; else e_band <= '0'; end if;
                    if e_ay(17 downto 4) >= resize(v_g, 14) and
                       ((e_top = '1' and e_ay(17 downto 4) < resize(v_g, 14) + 224) or
                        (e_top = '0' and e_ay(17 downto 4) < resize(v_g, 14) + 128)) then
                        e_strip <= '1';
                    else
                        e_strip <= '0';
                    end if;
                    if e_ay(17 downto 4) < resize(v_g, 14) and e_ay(17 downto 4) + 48 >= resize(v_g, 14) then
                        e_eline <= '1';
                    else
                        e_eline <= '0';
                    end if;
                when 4 =>
                    e_m <= r_ch_q(12 downto 0);
                when 19 =>                        -- profile row v = |y| / sigma - c (Q4)
                    e_vq <= signed(resize(m_acc(32 downto 16), 19)) - resize(c_cq, 19);
                    m_a <= resize(c_fs, 40); m_b <= resize(e_m, 16); m_acc <= (others => '0');
                when 20 =>
                    if e_vq < 0 then e_full <= '1'; else e_full <= '0'; end if;
                    if e_vq >= 8192 then e_zero <= '1'; else e_zero <= '0'; end if;
                    r_pf_a <= s_shape & (not e_top) & unsigned(e_vq(12 downto 5));
                when 22 =>
                    if e_full = '1' then e_W <= to_unsigned(32768, 16);
                    elsif e_zero = '1' then e_W <= (others => '0');
                    else e_W <= r_pf_q; end if;
                when 36 =>                        -- chevron frequency per h (+ the strips' slant)
                    v_f := resize(c_fs, 18) + resize(m_acc(29 downto 12), 18);
                    if e_strip = '1' then v_f := v_f + 655; end if;
                    e_fbh <= v_f;
                    m_a <= resize(c_kwpd(30 downto 7), 40); m_b <= e_W; m_acc <= (others => '0');
                when 37 =>
                    case to_integer(s_xs) is
                        when 4      => e_fbstep <= resize(shift_left(resize(e_fbh, 20), 2), 20);
                        when 3      => e_fbstep <= resize(shift_left(resize(e_fbh, 20), 1), 20) + resize(e_fbh, 20);
                        when others => e_fbstep <= resize(shift_left(resize(e_fbh, 20), 1), 20);
                    end case;
                    if e_band = '1' and (e_full = '1' or c_ae0 < resize(e_W, 17)) then e_ins0 <= '1'; else e_ins0 <= '0'; end if;
                when 38 =>                        -- lens phase distance from the entry (inside) or the last exit
                    if e_ins0 = '1' then e_q <= c_p0 + shift_left(resize(e_W, 24), 8);
                    else e_q <= c_p0 - shift_left(resize(e_W, 24), 8); end if;
                when 53 =>                        -- exit phase 0.05 + Fan * 2W (+ half a cycle on the dashed edge)
                    if e_eline = '1' then e_phx <= to_unsigned(838861, 24) + m_acc(31 downto 8) + x"800000";
                    else e_phx <= to_unsigned(838861, 24) + m_acc(31 downto 8); end if;
                    m_a <= resize(e_q, 40); m_b <= resize(c_pdh, 16); m_acc <= (others => '0');
                when 70 =>
                    e_dist <= m_acc(36 downto 24);
                when 71 =>
                    if e_band = '1' then
                        if e_ins0 = '1' then m_a <= resize(e_fbh, 40) + 100663; else m_a <= resize(e_fbh, 40); end if;
                        m_b <= resize(e_dist, 16);
                    else
                        m_a <= resize(e_fbh, 40); m_b <= resize(c_xr, 16);
                    end if;
                    m_acc <= (others => '0');
                when 88 =>
                    if e_band = '1' then
                        if e_ins0 = '1' then n_ph0 <= m_acc(23 downto 0);
                        else n_ph0 <= m_acc(23 downto 0) + e_phx + c_gttot(23 downto 0); end if;
                    elsif e_strip = '1' then
                        n_ph0 <= m_acc(23 downto 0) + 1258291;
                    else
                        n_ph0 <= m_acc(23 downto 0);
                    end if;
                    n_W <= e_W; n_full <= e_full; n_band <= e_band; n_eline <= e_eline;
                    n_fbstep <= e_fbstep; n_phx <= e_phx;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the lens phase DDA, the pixel counter (line-buffer read) and the
    -- line registers
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_lp <= c_p0;
                b_px <= (others => '0');
                b_first <= '1';
                l_W <= n_W; l_full <= n_full; l_band <= n_band; l_eline <= n_eline;
                l_fbstep <= n_fbstep; l_phx <= n_phx; l_ph0 <= n_ph0;
            elsif g_avid = '1' then
                b_lp <= b_lp + c_dl;
                b_px <= b_px + 1;
                b_first <= '0';
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = the beam)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_e   : signed(16 downto 0);
        variable v_sat : unsigned(2 downto 0);
        variable v_big : std_logic;
        variable v_w   : unsigned(2 downto 0);
        variable v_ins : std_logic;
        variable v_fw, v_rw : std_logic_vector(31 downto 0);
        variable v_d   : signed(10 downto 0);
        variable v_p   : signed(13 downto 0);
        variable v_r   : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: |e|, distance from the lens centre (32768 = half a period)
            v_e := resize(signed(b_lp(23 downto 8)), 17);
            p2_ae <= unsigned(abs(v_e));
            p2_first <= b_first;
            -- 3: inside the lens; distance outside it
            if l_band = '1' and (l_full = '1' or p2_ae < resize(l_W, 17)) then p3_ins <= '1'; else p3_ins <= '0'; end if;
            p3_dout <= signed(resize(p2_ae, 18)) - signed(resize(l_W, 18));
            p3_first <= p2_first;
            -- 4: exit / entry events, fringe weight 7 .. 0 over 7 << fsh lens units, luma nibble products
            p4_ins <= p3_ins;
            if p3_first = '1' then
                p4_exit <= '0'; p4_entry <= '0';
            else
                p4_exit <= p4_ins and not p3_ins;
                p4_entry <= (not p4_ins) and p3_ins;
            end if;
            v_big := '0';
            case to_integer(c_fsh) is
                when 6 =>
                    v_sat := unsigned(p3_dout(8 downto 6));
                    if p3_dout(17 downto 9) /= 0 then v_big := '1'; end if;
                when 7 =>
                    v_sat := unsigned(p3_dout(9 downto 7));
                    if p3_dout(17 downto 10) /= 0 then v_big := '1'; end if;
                when 8 =>
                    v_sat := unsigned(p3_dout(10 downto 8));
                    if p3_dout(17 downto 11) /= 0 then v_big := '1'; end if;
                when 9 =>
                    v_sat := unsigned(p3_dout(11 downto 9));
                    if p3_dout(17 downto 12) /= 0 then v_big := '1'; end if;
                when others =>
                    v_sat := unsigned(p3_dout(12 downto 10));
                    if p3_dout(17 downto 13) /= 0 then v_big := '1'; end if;
            end case;
            if p3_ins = '1' or l_band = '0' or l_eline = '1' or v_big = '1' or v_sat = 7 then
                p4_w <= (others => '0');
            else
                p4_w <= 7 - v_sat;
            end if;
            p4_llo <= unsigned(s_lb_q(3 downto 0)) * c_klpix;
            p4_lhi <= unsigned(s_lb_q(7 downto 4)) * c_klpix;
            p4_first <= p3_first;
            -- 5: lag tail (reloaded at each exit, decays 1/32 per pixel, 1/16 at SD); luma FM
            if p4_first = '1' then
                p5_g <= (others => '0');
            elsif p4_exit = '1' then
                p5_g <= c_gtpix;
            elsif c_dec4 = '1' then
                p5_g <= p5_g - shift_right(p5_g, 4);
            else
                p5_g <= p5_g - shift_right(p5_g, 5);
            end if;
            p5_l <= resize(p4_llo, 19) + shift_left(resize(p4_lhi, 19), 4);
            p5_ins <= p4_ins; p5_exit <= p4_exit; p5_entry <= p4_entry; p5_w <= p4_w; p5_first <= p4_first;
            -- 6: step extras (inside: the ghost's fast stripes; outside: tail + luma)
            if p5_ins = '1' then p6_t <= resize(c_ginpix, 23);
            else p6_t <= resize(p5_g, 23) + resize(p5_l, 23); end if;
            p6_ins <= p5_ins; p6_exit <= p5_exit; p6_entry <= p5_entry; p6_w <= p5_w; p6_first <= p5_first;
            -- 7: step
            p7_st <= resize(l_fbstep, 24) + resize(p6_t, 24);
            p7_ins <= p6_ins; p7_exit <= p6_exit; p7_entry <= p6_entry; p7_w <= p6_w; p7_first <= p6_first;
            -- 8: stripe phase
            if p7_first = '1' then
                p8_ph <= l_ph0;
            elsif p7_exit = '1' then
                p8_ph <= l_phx;
            elsif p7_entry = '1' and s_ghost = '1' then
                p8_ph <= (others => '0');
            else
                p8_ph <= p8_ph + p7_st;
            end if;
            p8_ins <= p7_ins; p8_w <= p7_w;
            -- 9: palette address; 10: ROM data; 11: fabric copy
            r_pl_a <= s_pal & p8_ph(23 downto 16);
            p9_ins <= p8_ins; p9_w <= p8_w;
            p10_ins <= p9_ins; p10_w <= p9_w;
            p11_q <= r_pl_q; p11_ins <= p10_ins; p11_w <= p10_w;
            -- 12: blend operands: inside = fill -> (ghost) palette; outside = palette -> fringe
            if s_pal = '1' then v_fw := C_LT_FILL(1); v_rw := C_LT_RIM(1);
            else v_fw := C_LT_FILL(0); v_rw := C_LT_RIM(0); end if;
            v_ins := p11_ins and not l_eline;
            if v_ins = '1' then
                p12_ay <= unsigned(v_fw(31 downto 22)); p12_au <= unsigned(v_fw(21 downto 12)); p12_av <= unsigned(v_fw(11 downto 2));
                p12_by <= unsigned(p11_q(31 downto 22)); p12_bu <= unsigned(p11_q(21 downto 12)); p12_bv <= unsigned(p11_q(11 downto 2));
                if s_ghost = '1' then p12_w <= to_unsigned(3, 3); else p12_w <= (others => '0'); end if;
            else
                p12_ay <= unsigned(p11_q(31 downto 22)); p12_au <= unsigned(p11_q(21 downto 12)); p12_av <= unsigned(p11_q(11 downto 2));
                p12_by <= unsigned(v_rw(31 downto 22)); p12_bu <= unsigned(v_rw(21 downto 12)); p12_bv <= unsigned(v_rw(11 downto 2));
                p12_w <= p11_w;
            end if;
            -- 13: differences
            p13_dy <= signed(resize(p12_by, 11)) - signed(resize(p12_ay, 11));
            p13_du <= signed(resize(p12_bu, 11)) - signed(resize(p12_au, 11));
            p13_dv <= signed(resize(p12_bv, 11)) - signed(resize(p12_av, 11));
            p13_ay <= p12_ay; p13_au <= p12_au; p13_av <= p12_av; p13_w <= p12_w;
            -- 14: d * w (shift-adds)
            v_p := (others => '0');
            if p13_w(0) = '1' then v_p := v_p + resize(p13_dy, 14); end if;
            if p13_w(1) = '1' then v_p := v_p + shift_left(resize(p13_dy, 14), 1); end if;
            if p13_w(2) = '1' then v_p := v_p + shift_left(resize(p13_dy, 14), 2); end if;
            p14_py <= v_p;
            v_p := (others => '0');
            if p13_w(0) = '1' then v_p := v_p + resize(p13_du, 14); end if;
            if p13_w(1) = '1' then v_p := v_p + shift_left(resize(p13_du, 14), 1); end if;
            if p13_w(2) = '1' then v_p := v_p + shift_left(resize(p13_du, 14), 2); end if;
            p14_pu <= v_p;
            v_p := (others => '0');
            if p13_w(0) = '1' then v_p := v_p + resize(p13_dv, 14); end if;
            if p13_w(1) = '1' then v_p := v_p + shift_left(resize(p13_dv, 14), 1); end if;
            if p13_w(2) = '1' then v_p := v_p + shift_left(resize(p13_dv, 14), 2); end if;
            p14_pv <= v_p;
            p14_ay <= p13_ay; p14_au <= p13_au; p14_av <= p13_av;
            -- 15: compose; black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            else
                v_r := signed(resize(p14_ay, 14)) + shift_right(p14_py, 3); s_y15 <= unsigned(v_r(9 downto 0));
                v_r := signed(resize(p14_au, 14)) + shift_right(p14_pu, 3); s_u15 <= unsigned(v_r(9 downto 0));
                v_r := signed(resize(p14_av, 14)) + shift_right(p14_pv, 3); s_v15 <= unsigned(v_r(9 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the still's horizontal softness, and the HD
    -- HDMI bar fix), luma 13/16 and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to black (the off-avid input of both boxes) once 8 blank samples are
            -- in, so a glitched sample can't offset the colours for good.
            -- box A: stage 16 sums, stage 17 output
            s_cay(0) <= s_y15; s_cau(0) <= s_u15; s_cav(0) <= s_v15;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= to_unsigned(512, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y15 - s_cay(7);
                s_bau <= s_bau + s_u15 - s_cau(7);
                s_bav <= s_bav + s_v15 - s_cav(7);
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: black off the generated avid (what box A gives there anyway)
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            -- box B: stage 18 sums, stage 19 output
            s_cby(0) <= s_y17; s_cbu(0) <= s_u17; s_cbv(0) <= s_v17;
            for i in 1 to 7 loop s_cby(i) <= s_cby(i - 1); s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(512, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y17 - s_cby(7);
                s_bbu <= s_bbu + s_u17 - s_cbu(7);
                s_bbv <= s_bbv + s_v17 - s_cbv(7);
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y19 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u19 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v19 <= v_sc(13 downto 4);
            -- 20: luma 13/16
            if s_y19 > 64 then v_dl := s_y19 - 64; else v_dl := (others => '0'); end if;
            s_y20 <= s_y19 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u20 <= s_u19; s_v20 <= s_v19;
            -- 21: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y21 <= s_y20; s_u21 <= s_u20; s_v21 <= s_v20;
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y21);
    data_out.u       <= std_logic_vector(s_u21);
    data_out.v       <= std_logic_vector(s_v21);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture lentil;
