-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Colorfield v0.1.0 -- Patchbay #10: soft window keys as paint (Rothko's stacked soft rectangles; at
-- the other end of the slider, Ellsworth Kelly's hard-edged bars).
--
-- The patch:
--   V ramp, H ramp -> + slow noise (four ridge terms from 512-entry ROMs) + source luma + dither
--     -> the ADDRESS of two 512-entry profile tables (the window keys, drawn fresh every field):
--        long axis  : paint key K and paint colour (Y, U, V): up to four soft windows, screen-mixed,
--                     colours cross-faded at the gap midpoints (the colour mixer)
--        cross axis : one soft window Kc
--   out = ground + K * Kc * (paint - ground)          (the keyer; real multiplies, one per stage)
--   -> canvas grain (fixed noise + the edge noise as a faint mottle) -> the shell's output boxes.
--   Per field a microcoded engine (register file RAM, one serial multiplier; microcode from
--   colorfield.py) glides the knobs, runs nine LFOs (each window's size on its own, each colour's
--   place on the chord's palette path on its own, the noise drifts), looks up layout and colours,
--   and leaves 8 pairs of constants; the table fill turns those into the two profiles in the vblank.
--
--   K1  Fields    16 layouts: 1 .. 4 windows and their proportions (the windows slide to the new layout)
--   K2  Feather   edge softness (exponential)
--   K3  Breath    depth of each window's size LFO
--   K4  Speed     bipolar, slow: all LFOs and the noise drift
--   K5  Drift     how far apart the fields sit on the palette path, and how far each wanders
--   K6  Colour    16 chords (ground + a looped 16-point palette path)
--   S7  Layout    Stacked / Columns (the two tables swap axes)
--   S8  Grain     On (canvas: grain, mottle, scumbled feathers) / Off (clean)
--   S9  Ground    Dark / Light
--   S10 Glow      Glow (a halo skirt four feathers wide around each window) / Matte
--   S11 Source    Video (camera luma moves the edges, stains the paint) / Osc (a drifting triangle lattice)
--   P12 DISSOLVE  crisp straight bars .. the photo-like default .. one fog of colour
--
-- Shell (from heirloom lozenge): timing-ID virtual steps, vblank zone, generated avid, one latency
-- (C_LAT 20), output conditioning (two [1 2 2 2 2 2 2 2 1]/16 boxes on Y, U, V with the per-line
-- resync, luma 13/16), blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K3 = Colour (was K6); K3 Breath fixed at 512 (C_KFIX_3).
-- 2026-11-15: K6 camera top +25 % (Ron): gain s_cg = 1.25 k / 8 in Q4 (was k / 8 in Q5); slices one bit lower, v_vdg 13 bits.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.colorfield_pkg.all;
use work.all;

architecture colorfield of program_top is
    constant C_KFIX_3 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(512, 10));   -- K6 camera pass: fixed knob
    constant C_LAT  : integer := 20;   -- = processing_delay_clks
    constant C_SNAP : boolean := false; -- sim only: glides jump to their targets

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
    -- x step in half virtual px per pixel: 1080 = 2, 720p = 3, SD = 4
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
    function f_bayer(r : unsigned(1 downto 0); c : unsigned(1 downto 0)) return unsigned is
        variable v_i : unsigned(3 downto 0);
    begin
        v_i := r & c;
        case to_integer(v_i) is
            when 0  => return to_unsigned(0, 4);  when 1  => return to_unsigned(8, 4);
            when 2  => return to_unsigned(2, 4);  when 3  => return to_unsigned(10, 4);
            when 4  => return to_unsigned(12, 4); when 5  => return to_unsigned(4, 4);
            when 6  => return to_unsigned(14, 4); when 7  => return to_unsigned(6, 4);
            when 8  => return to_unsigned(3, 4);  when 9  => return to_unsigned(11, 4);
            when 10 => return to_unsigned(1, 4);  when 11 => return to_unsigned(9, 4);
            when 12 => return to_unsigned(15, 4); when 13 => return to_unsigned(7, 4);
            when 14 => return to_unsigned(13, 4); when others => return to_unsigned(5, 4);
        end case;
    end function;

    attribute no_rw_check : boolean;

    -- profile tables (written by the fill in the vblank zone, read by the pixel path)
    -- long axis word: E_Y(10) E_U(10) E_V(10) kq(2), E = K * (paint - ground) / 2; cross axis: Kc(8)
    signal pf : t_c_cf_pf := C_CF_PF;
    attribute no_rw_check of pf : signal is true;
    signal pf_ra, pf_wa : unsigned(8 downto 0) := (others => '0');
    signal pf_q, pf_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal pf_we : std_logic := '0';
    signal cr : t_c_cf_cr := C_CF_CR;
    attribute no_rw_check of cr : signal is true;
    signal cr_ra, cr_wa : unsigned(8 downto 0) := (others => '0');
    signal cr_q, cr_wd  : std_logic_vector(7 downto 0) := (others => '0');
    signal cr_we : std_logic := '0';

    -- fill constants (written by the engine during the field, read by the fill): 0..7 X, 8..15 Y
    signal fc : t_c_cf_fc := C_CF_FC;
    attribute no_rw_check of fc : signal is true;
    signal fc_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal fc_wa : unsigned(3 downto 0) := (others => '0');
    signal fc_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal fc_we : std_logic := '0';

    -- engine memories
    signal rf : t_c_cf_rf := C_CF_RF;
    attribute no_rw_check of rf : signal is true;
    signal rf_ra, rf_wa : unsigned(7 downto 0) := (others => '0');
    signal rf_q, rf_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal rf_we : std_logic := '0';
    signal uc_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal dr_q  : std_logic_vector(7 downto 0) := (others => '0');
    signal ex_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal sr_a  : unsigned(8 downto 0) := (others => '0');
    signal sr_q  : std_logic_vector(7 downto 0) := (others => '0');
    signal na_q, nb_q, nc_q, nd_q : std_logic_vector(7 downto 0) := (others => '0');

    -- video vertical lag (one state per 8 px)
    type t_m16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal vb : t_m16 := (others => x"0000");
    attribute no_rw_check of vb : signal is true;
    signal vb_ra, vb_wa : unsigned(7 downto 0) := (others => '0');
    signal vb_q, vb_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal vb_we : std_logic := '0';

    -- 8-clock delay ring: grain(10) | box A in: Y(10) U(8) V(8) | box B in: Y(10) U(9) V(9)
    type t_m64 is array (0 to 255) of std_logic_vector(63 downto 0);
    signal rg : t_m64 := (others => (others => '0'));
    attribute no_rw_check of rg : signal is true;
    signal rg_wa, rg_ra : unsigned(7 downto 0) := (others => '0');
    signal rg_q, rg_f : std_logic_vector(63 downto 0) := (others => '0');
    signal rg_wd : std_logic_vector(63 downto 0);
    signal g4 : signed(9 downto 0) := (others => '0');

    -- oscillator source (Source = Osc): 2-D triangle lattice, 320 x 270 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(204, 9);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal v_la, v_lb : unsigned(11 downto 0) := (others => '0');     -- two one-pole lags along the line (Q4)
    signal vb_f, vb_n, v_fs : unsigned(11 downto 0) := (others => '0');
    signal v_hl  : unsigned(7 downto 0) := (others => '0');
    signal v_st  : unsigned(7 downto 0) := (others => '0');            -- stain
    signal v_vd  : signed(11 downto 0) := (others => '0');             -- edge displacement, Q4 entries
    -- K6 camera pass: K6 Camera glided per field (Q3, +-4 deadband), gain Q5 (32 = the shipped x1)
    signal s_kc  : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_kcr : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_kcd : signed(14 downto 0) := (others => '0');
    signal s_kcp : std_logic_vector(3 downto 0) := (others => '0');
    signal s_cg  : unsigned(6 downto 0) := to_unsigned(40, 7);   -- K6 top +25 %: Q4 (16 = x1), 1.25 k / 8 / 2, max 79
    signal v_vdp : signed(19 downto 0) := (others => '0');
    signal v_stp : unsigned(14 downto 0) := (others => '0');
    signal v_vdg : signed(12 downto 0) := (others => '0');            -- K6 top +25 %: max |2360|
    signal v_stg : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_diy  : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_dix  : unsigned(7 downto 0) := to_unsigned(68, 8);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_cols, s_groff, s_matte, s_src : std_logic := '0';

    -- engine outputs: o_* for the fill (written by the main program during the field), c_* for the
    -- pixel path and the fill (written by the commit block at the zone start)
    signal o_g   : unsigned(15 downto 0) := C_CF_I_G;
    signal o_c0y : unsigned(9 downto 0) := C_CF_I_C0Y;
    signal o_c0u : unsigned(6 downto 0) := C_CF_I_C0U;
    signal o_c0v : unsigned(6 downto 0) := C_CF_I_C0V;
    signal c_gy  : unsigned(9 downto 0) := C_CF_I_GY;
    signal c_gu  : unsigned(9 downto 0) := C_CF_I_GU;
    signal c_gv  : unsigned(9 downto 0) := C_CF_I_GV;
    signal c_amp : unsigned(5 downto 0) := C_CF_I_AMP;
    signal c_mask : unsigned(9 downto 0) := to_unsigned(63, 10);
    signal c_half : unsigned(9 downto 0) := to_unsigned(32, 10);
    signal c_nz0 : unsigned(14 downto 0) := C_CF_I_NZ0;
    signal c_nz1 : unsigned(14 downto 0) := C_CF_I_NZ1;
    signal c_nz2 : unsigned(14 downto 0) := C_CF_I_NZ2;
    signal c_nz3 : unsigned(14 downto 0) := C_CF_I_NZ3;

    -- table fill
    signal fl_arm, fl_run, fl_done : std_logic := '0';
    signal fl_c  : unsigned(13 downto 0) := (others => '0');
    signal fl_q  : unsigned(25 downto 0) := (others => '0');
    signal fl_w, fl_a1, fl_a2, fl_am : signed(27 downto 0) := (others => '0');
    signal fl_kind : unsigned(1 downto 0) := (others => '0');          -- 0 window, 1 cross window, 2 crossfade
    signal fl_zero, fl_one, fl_adj : std_logic := '0';
    signal fl_zero6, fl_one6, fl_adj6 : std_logic := '0';
    signal fl_v  : unsigned(7 downto 0) := (others => '0');
    signal fl_t1, fl_t2, fl_t3 : unsigned(7 downto 0) := (others => '0');
    signal fl_sw : std_logic_vector(26 downto 0) := (others => '0');
    signal fl_u  : unsigned(8 downto 0) := (others => '0');
    signal fl_kh : unsigned(7 downto 0) := (others => '0');
    signal fl_ma : signed(9 downto 0) := (others => '0');
    signal fl_mb : signed(10 downto 0) := (others => '0');
    signal fl_pr : signed(20 downto 0) := (others => '0');
    signal fl_ay : signed(21 downto 0) := (others => '0');
    signal fl_au, fl_av : signed(18 downto 0) := (others => '0');
    signal fl_pd : signed(10 downto 0) := (others => '0');
    signal fl_pc : unsigned(9 downto 0) := (others => '0');
    signal x_a1, x_a2, x_sw, x_amt, x_amc, x_vu, x_vt1, x_vt2, x_vt3, x_vcr, x_pu : std_logic := '0';
    signal x_ayi, x_aya, x_aui, x_aua, x_avi, x_ava, x_kh, x_pcy, x_pcu, x_pcv : std_logic := '0';
    signal x_pdy, x_pdu, x_pdv, x_wy, x_wu, x_wv, x_wr, x_qup : std_logic := '0';
    signal x_ma, x_mb : std_logic_vector(2 downto 0) := "000";
    signal fl_wy, fl_wu, fl_wv : signed(9 downto 0) := (others => '0');

    -- engine
    type t_est is (E_IDLE, E_F0, E_F1, E_RA, E_RB, E_XA, E_XB, E_EX, E_ADD, E_GL2, E_SEL, E_MUL, E_SH, E_WR);
    signal e_st  : t_est := E_IDLE;
    attribute fsm_encoding : string;
    attribute fsm_encoding of e_st : signal is "none";
    signal e_pc  : unsigned(7 downto 0) := (others => '0');
    signal e_ir  : std_logic_vector(31 downto 0) := (others => '0');
    signal e_fi  : unsigned(1 downto 0) := (others => '0');
    signal e_x, e_h, e_m : signed(27 downto 0) := (others => '0');
    signal e_b   : unsigned(15 downto 0) := (others => '0');
    signal e_cnt : unsigned(4 downto 0) := (others => '0');
    signal e_ci, e_shl : std_logic := '0';
    signal e_nx  : unsigned(1 downto 0) := (others => '0');            -- after the add: 0 write, 1 glide, 2 max / min

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start, g_st1, g_st2 : std_logic := '0';

    -- line and beam accumulators (Q6 noise entries, Q8 table entries); the pixel path's stage L holds the
    -- pixel whose generated avid is s_gav_sr(L)
    signal l_na, l_nb, l_nc, l_nd : unsigned(14 downto 0) := (others => '0');
    signal l_iy  : unsigned(16 downto 0) := (others => '0');
    signal l_cnt : unsigned(1 downto 0) := (others => '0');
    signal b_na, b_nb, b_nc, b_nd : unsigned(14 downto 0) := (others => '0');
    signal b_iy  : unsigned(12 downto 0) := (others => '0');
    signal b_row : unsigned(1 downto 0) := (others => '0');
    signal c_dla, c_dlb, c_dlc : unsigned(14 downto 0) := (others => '0');
    signal c_dpa, c_dpc, c_dpd : unsigned(14 downto 0) := (others => '0');

    -- generators: fixed random numbers (stage 3), table x coordinate and dither column (stage 5)
    signal q_ix   : unsigned(16 downto 0) := (others => '0');
    signal q_lfsr : std_logic_vector(30 downto 0) := "0101010010110100101101001011011";
    signal q_w    : unsigned(15 downto 0) := (others => '0');
    signal q_col  : unsigned(1 downto 0) := (others => '0');
    signal q_in   : std_logic := '0';

    -- pixel path
    signal p3_na, p3_nb, p3_nc, p3_nd : signed(7 downto 0) := (others => '0');
    signal p4_nh, p4_nv : signed(8 downto 0) := (others => '0');
    signal p5_mh, p5_mv : signed(15 downto 0) := (others => '0');
    signal p6_th, p6_tv : signed(15 downto 0) := (others => '0');
    signal p6_e1, p6_e2 : signed(11 downto 0) := (others => '0');
    signal p7_dy, p7_dx : signed(15 downto 0) := (others => '0');
    signal p10_pf : std_logic_vector(31 downto 0) := (others => '0');
    signal p10_kc : unsigned(7 downto 0) := (others => '0');
    signal p11_my : signed(18 downto 0) := (others => '0');
    signal p11_mu, p11_mv : signed(17 downto 0) := (others => '0');
    signal p11_st : unsigned(7 downto 0) := (others => '0');
    signal p12_y  : signed(11 downto 0) := (others => '0');
    signal p12_u, p12_v : unsigned(9 downto 0) := (others => '0');
    signal s_y15 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_u15, s_v15 : unsigned(7 downto 0) := to_unsigned(128, 8);

    -- output conditioning (chroma: 8 bits into box A, 9 into box B, 10 out)
    signal s_bay, s_bayd, s_bby, s_bbyd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bbu, s_bbv, s_bbud, s_bbvd : unsigned(11 downto 0) := to_unsigned(2048, 12);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_u17, s_v17 : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid delayed k + 1 clocks

    -- fill: constant address per phase of the 16-clock entry cycle
    function f_fca(k : unsigned(3 downto 0)) return unsigned is
    begin
        case to_integer(k) is
            when 0  => return to_unsigned(14, 4);   -- colour step 2
            when 1  => return to_unsigned(1, 4);    -- window 1: X, Y
            when 2  => return to_unsigned(9, 4);
            when 3  => return to_unsigned(15, 4);   -- colour step 3
            when 4  => return to_unsigned(2, 4);    -- window 2
            when 5  => return to_unsigned(10, 4);
            when 6  => return to_unsigned(3, 4);    -- window 3
            when 7  => return to_unsigned(11, 4);
            when 8  => return to_unsigned(5, 4);    -- crossfades 1 .. 3
            when 9  => return to_unsigned(6, 4);
            when 10 => return to_unsigned(7, 4);
            when 11 => return to_unsigned(4, 4);    -- cross window
            when 12 => return to_unsigned(12, 4);
            when 13 => return to_unsigned(13, 4);   -- colour step 1
            when 14 => return to_unsigned(0, 4);    -- window 0 of the next entry
            when others => return to_unsigned(8, 4);
        end case;
    end function;

begin

    ---------------------------------------------------------------------------
    -- Memories (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_mem : process(clk)
    begin
        if rising_edge(clk) then
            pf_q <= pf(to_integer(pf_ra));
            if pf_we = '1' then pf(to_integer(pf_wa)) <= pf_wd; end if;
            cr_q <= cr(to_integer(cr_ra));
            if cr_we = '1' then cr(to_integer(cr_wa)) <= cr_wd; end if;
            fc_q <= fc(to_integer(f_fca(fl_c(3 downto 0))));
            if fc_we = '1' then fc(to_integer(fc_wa)) <= fc_wd; end if;
            rf_q <= rf(to_integer(rf_ra));
            if rf_we = '1' then rf(to_integer(rf_wa)) <= rf_wd; end if;
            vb_q <= vb(to_integer(vb_ra));
            if vb_we = '1' then vb(to_integer(vb_wa)) <= vb_wd; end if;
            rg_q <= rg(to_integer(rg_ra));
            rg(to_integer(rg_wa)) <= rg_wd;
            rg_f <= rg_q;
            rg_wa <= rg_wa + 1;
            rg_ra <= rg_wa - 5;
            uc_q <= C_CF_UC(to_integer(e_pc));
            dr_q <= C_CF_DROM(to_integer(unsigned(e_x(9 downto 0))));
            ex_q <= C_CF_EXP2(to_integer(unsigned(e_x(7 downto 0))));
            sr_q <= C_CF_SROM(to_integer(sr_a));
            na_q <= C_CF_NA(to_integer(b_na(14 downto 6)));
            nb_q <= C_CF_NB(to_integer(b_nb(14 downto 6)));
            nc_q <= C_CF_NC(to_integer(b_nc(14 downto 6)));
            nd_q <= C_CF_ND(to_integer(b_nd(14 downto 6)));
        end if;
    end process p_mem;
    rg_wd <= std_logic_vector(g4) & std_logic_vector(s_y15) & std_logic_vector(s_u15) & std_logic_vector(s_v15)
             & std_logic_vector(s_y17) & std_logic_vector(s_u17) & std_logic_vector(s_v17);

    ---------------------------------------------------------------------------
    -- Timing, vblank zone, width, per-field latches, sync delay
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_m : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            v_m := s_ys * to_unsigned(121, 7);
            s_diy <= v_m(17 downto 8);

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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_cols  <= registers_in(6)(0);
                s_groff <= registers_in(6)(1);
                s_matte <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_dix <= to_unsigned(182, 8); s_oxs <= to_unsigned(408, 9);
                    when 3      => s_dix <= to_unsigned(102, 8); s_oxs <= to_unsigned(306, 9);
                    when others => s_dix <= to_unsigned(68, 8);  s_oxs <= to_unsigned(204, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;

            -- noise steps: per line (Q6 entries, an entry = 4 virtual px) and per pixel
            c_dla <= resize(shift_right(s_ys, 5), 15);
            c_dlb <= resize(shift_right(s_ys, 6), 15) + resize(shift_right(s_ys, 7), 15);
            c_dlc <= resize(shift_right(s_ys, 4), 15);
            c_dpa <= shift_left(resize(s_xs, 15), 3);
            c_dpc <= shift_left(resize(s_xs, 15), 2);
            c_dpd <= shift_left(resize(s_xs, 15), 1) + resize(s_xs, 15);

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Table fill: starts when the commit block has run, 16 clocks per entry, one
    -- adder pair, one ROM port, one multiplier.  k = fl_c(3:0), cycle = fl_c(13:4).
    -- A constant read at phase k is in fl_w at k + 2.  Per entry e (cycle e + 1):
    --   windows 0..3: X in fl_w at 0, 3, 6, 8, Y one later; min of the two ramps
    --     captured at 2, 5, 8, 10; value in fl_v at 6, 9, 12, 14
    --   crossfades: X at 10, 11, 12; value at 15, 0, 1 (the last two in cycle e + 2)
    --   cross window: X, Y at 13, 14; value at 3 of cycle e + 2
    -- Multiplier (operands picked at k, product in fl_pr at k + 2):
    --   9, 12, 14: K = 1 - prod(1 - k);  0..8: the nine colour steps (entry e, in cycle e + 2);
    --   11, 13, 15: E = K * (paint - ground) / 2.  Entry e is written at phase 2 of cycle e + 3.
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_k   : unsigned(3 downto 0);
        variable v_kn  : integer range 0 to 15;
        variable v_cyc : unsigned(9 downto 0);
        variable v_q   : signed(27 downto 0);
        variable v_s   : unsigned(7 downto 0);
        variable v_a   : unsigned(8 downto 0);
        variable v_c   : unsigned(7 downto 0);
        variable v_cp  : unsigned(8 downto 0);
        variable v_tp  : unsigned(8 downto 0);
        variable v_k8  : unsigned(9 downto 0);
        variable v_y   : signed(13 downto 0);
        variable v_u   : signed(10 downto 0);
        variable v_e   : signed(11 downto 0);
        variable v_e10 : signed(9 downto 0);
        variable v_n   : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            fl_done <= '0'; pf_we <= '0'; cr_we <= '0';
            v_k := fl_c(3 downto 0);
            v_cyc := fl_c(13 downto 4);
            if s_zgo = '1' then
                fl_arm <= '1'; fl_run <= '0';
            elsif fl_arm = '1' and e_st = E_IDLE then
                fl_arm <= '0'; fl_run <= '1'; fl_c <= to_unsigned(14, 14); fl_q <= (others => '0');
            elsif fl_run = '1' then
                fl_c <= fl_c + 1;
                if fl_c = 8228 then fl_run <= '0'; fl_done <= '1'; end if;
                if x_qup = '1' and v_cyc /= 0 then fl_q <= fl_q + resize(o_g, 26); end if;
            end if;

            -- phase strobes for the next clock (registered, so no phase decode sits on a data path)
            v_kn := to_integer(v_k + 1);
            x_a1 <= '0'; x_a2 <= '0'; x_sw <= '0'; x_amt <= '0'; x_amc <= '0';
            x_vu <= '0'; x_vt1 <= '0'; x_vt2 <= '0'; x_vt3 <= '0'; x_vcr <= '0';
            x_pu <= '0'; x_ayi <= '0'; x_aya <= '0'; x_aui <= '0'; x_aua <= '0'; x_avi <= '0'; x_ava <= '0';
            x_kh <= '0'; x_pcy <= '0'; x_pcu <= '0'; x_pcv <= '0'; x_pdy <= '0'; x_pdu <= '0'; x_pdv <= '0';
            x_wy <= '0'; x_wu <= '0'; x_wv <= '0'; x_wr <= '0'; x_qup <= '0';
            case v_kn is
                when 0 | 3 | 6 | 8 | 10 | 11 | 12 | 13 => x_a1 <= '1';
                when 1 | 4 | 7 | 9 | 14                => x_a2 <= '1';
                when others => null;
            end case;
            case v_kn is when 15 | 2 | 5 => x_sw <= '1'; when others => null; end case;
            case v_kn is when 11 | 12 | 13 => x_amt <= '1'; when 15 => x_amc <= '1'; when others => null; end case;
            case v_kn is
                when 6 => x_vu <= '1'; when 15 => x_vt1 <= '1'; when 0 => x_vt2 <= '1'; when 1 => x_vt3 <= '1';
                when 3 => x_vcr <= '1'; when others => null;
            end case;
            case v_kn is
                when 0 | 1 | 2 => x_ma <= "000";      -- crossfade 1
                when 3 | 4 | 5 => x_ma <= "001";
                when 6 | 7 | 8 => x_ma <= "010";
                when 9 | 12    => x_ma <= "011";      -- the running product
                when 14        => x_ma <= "100";      -- the product register itself
                when others    => x_ma <= "101";      -- K
            end case;
            case v_kn is
                when 0 | 3 | 6    => x_mb <= "000";   -- colour steps Y, U, V
                when 1 | 4 | 7    => x_mb <= "001";
                when 2 | 5 | 8    => x_mb <= "010";
                when 9 | 12 | 14  => x_mb <= "011";   -- 1 - window
                when others       => x_mb <= "100";   -- paint - ground
            end case;
            case v_kn is when 11 | 14 | 0 => x_pu <= '1'; when others => null; end case;
            case v_kn is
                when 2 => x_ayi <= '1'; x_kh <= '1'; x_wr <= '1';
                when 5 | 8 => x_aya <= '1';
                when 3 => x_aui <= '1';
                when 6 | 9 => x_aua <= '1';
                when 4 => x_avi <= '1';
                when 7 | 10 => x_ava <= '1';
                when others => null;
            end case;
            case v_kn is
                when 9  => x_pcy <= '1'; when 11 => x_pcu <= '1'; when 13 => x_pcv <= '1' ; x_wy <= '1';
                when 10 => x_pdy <= '1'; when 12 => x_pdu <= '1'; when 14 => x_pdv <= '1';
                when 15 => x_wu <= '1'; x_qup <= '1';
                when 1  => x_wv <= '1';
                when others => null;
            end case;

            -- fabric copy of the constant; the two ramps (Q = entry * gain)
            fl_w <= signed(fc_q(27 downto 0));
            v_q := signed(resize(fl_q, 28));
            if x_a1 = '1' then fl_a1 <= fl_w + v_q; end if;
            if x_a2 = '1' then fl_a2 <= fl_w - v_q; end if;
            if x_sw = '1' then fl_sw <= std_logic_vector(fl_w(26 downto 0)); end if;
            -- the window = the lower ramp (a crossfade has one ramp, at twice the gain)
            if x_amt = '1' then
                fl_am <= fl_a1; fl_kind <= "10";
            else
                if fl_a2 < fl_a1 then fl_am <= fl_a2; else fl_am <= fl_a1; end if;
                fl_kind <= '0' & x_amc;
            end if;
            -- ROM address: key (0 .. 1 over the feather) or the halo skirt (-4 .. 0), and the flags
            fl_zero <= '0'; fl_one <= '0'; fl_adj <= '0';
            if fl_kind = "10" then
                sr_a <= '0' & unsigned(fl_am(14 downto 7));
                if fl_am(27) = '1' then fl_zero <= '1';
                elsif fl_am(26 downto 15) /= 0 then fl_one <= '1'; end if;
            elsif fl_am(27) = '1' then
                sr_a <= '1' & unsigned(fl_am(17 downto 10));
                if s_matte = '1' or fl_kind = "01" or fl_am(27 downto 18) /= "1111111111" then fl_zero <= '1'; end if;
            else
                sr_a <= '0' & unsigned(fl_am(15 downto 8));
                if fl_am(27 downto 16) /= 0 then fl_one <= '1'; end if;
                if s_matte = '0' and fl_kind = "00" then fl_adj <= '1'; end if;
            end if;
            -- ROM data (EBR), then the value
            fl_zero6 <= fl_zero; fl_one6 <= fl_one; fl_adj6 <= fl_adj;
            if fl_zero6 = '1' then v_s := (others => '0');
            elsif fl_one6 = '1' then v_s := (others => '1');
            else v_s := unsigned(sr_q); end if;
            if fl_adj6 = '1' then
                v_a := resize(v_s, 9) - resize(shift_right(v_s, 2), 9) + 64;
                if v_a(8) = '1' then fl_v <= (others => '1'); else fl_v <= v_a(7 downto 0); end if;
            else
                fl_v <= v_s;
            end if;

            -- values as they arrive
            v_c  := not fl_v;
            v_cp := resize(v_c, 9) + resize(v_c(7 downto 7), 9);
            if x_vt1 = '1' then fl_t1 <= fl_v; end if;
            if x_vt2 = '1' then fl_t2 <= fl_v; end if;
            if x_vt3 = '1' then fl_t3 <= fl_v; end if;
            v_n := v_cyc - 2;
            cr_wd <= std_logic_vector(fl_v); cr_wa <= v_n(8 downto 0);
            if x_vcr = '1' and fl_run = '1' and v_cyc >= 2 and v_cyc <= 513 then cr_we <= '1'; end if;

            -- multiplier operands
            case x_ma is
                when "000"  => v_tp := resize(fl_t1, 9) + resize(fl_t1(7 downto 7), 9);
                when "001"  => v_tp := resize(fl_t2, 9) + resize(fl_t2(7 downto 7), 9);
                when "010"  => v_tp := resize(fl_t3, 9) + resize(fl_t3(7 downto 7), 9);
                when "011"  => v_tp := fl_u;
                when "100"  => v_tp := unsigned(fl_pr(16 downto 8));
                when others => v_tp := resize(fl_kh, 9) + resize(fl_kh(7 downto 7), 9);
            end case;
            fl_ma <= signed('0' & v_tp);
            case x_mb is
                when "000"  => fl_mb <= signed(fl_sw(26 downto 16));
                when "001"  => fl_mb <= resize(signed(fl_sw(15 downto 8)), 11);
                when "010"  => fl_mb <= resize(signed(fl_sw(7 downto 0)), 11);
                when "011"  => fl_mb <= signed(resize(v_cp, 11));
                when others => fl_mb <= fl_pd;
            end case;
            fl_pr <= fl_ma * fl_mb;

            -- products as they arrive (picked two clocks earlier)
            if x_vu = '1' then fl_u <= v_cp; elsif x_pu = '1' then fl_u <= unsigned(fl_pr(16 downto 8)); end if;
            if x_ayi = '1' then fl_ay <= signed(resize(o_c0y & x"00", 22)) + resize(fl_pr, 22);
            elsif x_aya = '1' then fl_ay <= fl_ay + resize(fl_pr, 22); end if;
            if x_aui = '1' then fl_au <= signed(resize(o_c0u & x"00", 19)) + resize(fl_pr, 19);
            elsif x_aua = '1' then fl_au <= fl_au + resize(fl_pr, 19); end if;
            if x_avi = '1' then fl_av <= signed(resize(o_c0v & x"00", 19)) + resize(fl_pr, 19);
            elsif x_ava = '1' then fl_av <= fl_av + resize(fl_pr, 19); end if;
            if x_kh = '1' then
                v_k8 := to_unsigned(256, 10) - resize(fl_u, 10);
                if v_k8(8) = '1' then fl_kh <= (others => '1'); else fl_kh <= v_k8(7 downto 0); end if;
            end if;
            -- paint (clamped), then paint - ground, one channel at a time
            v_y := fl_ay(21 downto 8);
            if x_pcy = '1' then
                if v_y < 0 then fl_pc <= (others => '0'); elsif v_y > 1023 then fl_pc <= (others => '1'); else fl_pc <= unsigned(v_y(9 downto 0)); end if;
            end if;
            if x_pcu = '1' then v_u := fl_au(18 downto 8); else v_u := fl_av(18 downto 8); end if;
            if x_pcu = '1' or x_pcv = '1' then
                if v_u < 0 then fl_pc <= (others => '0'); elsif v_u > 127 then fl_pc <= "1111111000"; else fl_pc <= unsigned(v_u(6 downto 0)) & "000"; end if;
            end if;
            if x_pdy = '1' then fl_pd <= signed(resize(fl_pc, 11)) - signed(resize(c_gy, 11)); end if;
            if x_pdu = '1' then fl_pd <= signed(resize(fl_pc, 11)) - signed(resize(c_gu, 11)); end if;
            if x_pdv = '1' then fl_pd <= signed(resize(fl_pc, 11)) - signed(resize(c_gv, 11)); end if;
            -- E = K * (paint - ground) / 2, clamped to 10 bits
            v_e := fl_pr(20 downto 9);
            if v_e < -512 then v_e10 := to_signed(-512, 10); elsif v_e > 511 then v_e10 := to_signed(511, 10); else v_e10 := v_e(9 downto 0); end if;
            if x_wy = '1' then fl_wy <= v_e10; end if;
            if x_wu = '1' then fl_wu <= v_e10; end if;
            if x_wv = '1' then fl_wv <= v_e10; end if;
            v_n := v_cyc - 3;
            pf_wd <= std_logic_vector(fl_wy) & std_logic_vector(fl_wu) & std_logic_vector(fl_wv) & std_logic_vector(fl_kh(7 downto 6));
            pf_wa <= v_n(8 downto 0);
            if x_wr = '1' and fl_run = '1' and v_cyc >= 3 and v_cyc <= 514 then pf_we <= '1'; end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- Engine: microcode from colorfield.py.  The commit block (pc 0) runs at the
    -- zone start, the main program when the fill is done.
    -- op(31:28) n(27:23) a(22:16) b(15:9) d(8:2) ix(0); registers >= 112 are per
    -- field (+ 16 * field index).  One adder: e_h + e_m + carry.
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_op : unsigned(3 downto 0);
        variable v_n  : unsigned(4 downto 0);
        variable v_a, v_b, v_d : unsigned(7 downto 0);
        variable v_bq : signed(27 downto 0);
        variable v_sum : signed(27 downto 0);
        variable v_pn : unsigned(4 downto 0);
        variable v_in : unsigned(9 downto 0);
        variable v_tr : unsigned(15 downto 0);
        variable v_s29 : unsigned(28 downto 0);
    begin
        if rising_edge(clk) then
            rf_we <= '0'; fc_we <= '0';
            v_op := unsigned(e_ir(31 downto 28));
            v_n  := unsigned(e_ir(27 downto 23));
            v_a  := '0' & unsigned(e_ir(22 downto 16));
            v_b  := '0' & unsigned(e_ir(15 downto 9));
            v_d  := '0' & unsigned(e_ir(8 downto 2));
            if v_a >= 112 then v_a := v_a + (e_fi & "0000"); end if;
            if v_b >= 112 then v_b := v_b + (e_fi & "0000"); end if;
            if v_d >= 112 then v_d := v_d + (e_fi & "0000"); end if;
            v_bq := signed(rf_q(27 downto 0));
            v_s29 := (unsigned(e_h) & '1') + (unsigned(e_m) & e_ci);
            v_sum := signed(v_s29(28 downto 1));

            case e_st is
                when E_IDLE => null;
                when E_F0   => e_st <= E_F1;
                when E_F1   => e_ir <= uc_q; e_st <= E_RA;
                when E_RA   => rf_ra <= v_a; e_st <= E_RB;
                when E_RB   => rf_ra <= v_b; e_st <= E_XA;
                when E_XA   => e_x <= v_bq; e_st <= E_XB;
                when E_XB   =>                                   -- rf_q holds operand b
                    e_st <= E_WR; e_ci <= '0'; e_nx <= "00";
                    case to_integer(v_op) is
                        when 0 => e_st <= E_IDLE;                                    -- HALT
                        when 1 => e_h <= e_x; e_m <= v_bq; e_st <= E_ADD;           -- ADD
                        when 2 => e_h <= e_x; e_m <= not v_bq; e_ci <= '1'; e_st <= E_ADD;   -- SUB
                        when 3 =>                                                     -- MUL
                            e_h <= (others => '0'); e_m <= e_x; e_b <= unsigned(v_bq(15 downto 0));
                            e_cnt <= (others => '0'); e_st <= E_MUL;
                        when 4 =>                                                     -- GLD: b - a first
                            e_h <= v_bq; e_m <= not e_x; e_ci <= '1'; e_nx <= "01"; e_st <= E_ADD;
                        when 5 | 6 => e_st <= E_EX;                                   -- ROM / EXP
                        when 7 =>                                                     -- SHR
                            e_h <= e_x; e_shl <= '0'; e_st <= E_SH;
                            if v_n = 31 then e_cnt <= unsigned(v_bq(4 downto 0)); else e_cnt <= v_n; end if;
                        when 8 => e_h <= e_x; e_shl <= '1'; e_cnt <= v_n; e_st <= E_SH;   -- SHL
                        when 9 =>                                                     -- IN
                            case to_integer(v_n(2 downto 0)) is
                                when 0 => v_in := unsigned(registers_in(0));
                                when 1 => v_in := unsigned(registers_in(1));
                                when 2 => v_in := unsigned(C_KFIX_3);
                                when 3 => v_in := unsigned(registers_in(3));
                                when 4 => v_in := unsigned(registers_in(4));
                                when 5 => v_in := unsigned(registers_in(2));
                                when 6 => v_in := unsigned(registers_in(6));
                                when others => v_in := unsigned(registers_in(7));
                            end case;
                            e_h <= signed(resize(v_in, 28));
                        when 10 =>                                                    -- OUT
                            v_pn := v_n;
                            if e_ir(0) = '1' then v_pn := v_n + e_fi; end if;
                            fc_wa <= v_pn(3 downto 0); fc_wd <= std_logic_vector(resize(e_x, 32));
                            if v_pn(4) = '0' then fc_we <= '1'; end if;
                            case to_integer(v_pn) is
                                when 16 => o_g   <= unsigned(e_x(15 downto 0));
                                when 17 => o_c0y <= unsigned(e_x(9 downto 0));
                                when 18 => o_c0u <= unsigned(e_x(6 downto 0));
                                when 19 => o_c0v <= unsigned(e_x(6 downto 0));
                                when 20 => c_gy  <= unsigned(e_x(9 downto 0));
                                when 21 => c_gu  <= unsigned(e_x(9 downto 0));
                                when 22 => c_gv  <= unsigned(e_x(9 downto 0));
                                when 23 => c_amp <= unsigned(e_x(5 downto 0));
                                when 24 =>
                                    case to_integer(unsigned(e_x(3 downto 0))) is
                                        when 0      => c_mask <= to_unsigned(3, 10);   c_half <= to_unsigned(2, 10);
                                        when 1      => c_mask <= to_unsigned(7, 10);   c_half <= to_unsigned(4, 10);
                                        when 2      => c_mask <= to_unsigned(15, 10);  c_half <= to_unsigned(8, 10);
                                        when 3      => c_mask <= to_unsigned(31, 10);  c_half <= to_unsigned(16, 10);
                                        when 4      => c_mask <= to_unsigned(63, 10);  c_half <= to_unsigned(32, 10);
                                        when 5      => c_mask <= to_unsigned(127, 10); c_half <= to_unsigned(64, 10);
                                        when 6      => c_mask <= to_unsigned(255, 10); c_half <= to_unsigned(128, 10);
                                        when 7      => c_mask <= to_unsigned(511, 10); c_half <= to_unsigned(256, 10);
                                        when others => c_mask <= to_unsigned(1023, 10); c_half <= to_unsigned(512, 10);
                                    end case;
                                when 25 => c_nz0 <= unsigned(e_x(14 downto 0));
                                when 26 => c_nz1 <= unsigned(e_x(14 downto 0));
                                when 27 => c_nz2 <= unsigned(e_x(14 downto 0));
                                when 28 => c_nz3 <= unsigned(e_x(14 downto 0));
                                when others => null;
                            end case;
                            e_pc <= e_pc + 1; e_st <= E_F0;
                        when 12 =>                                                    -- MAX / MIN: a - b first
                            e_h <= e_x; e_m <= not v_bq; e_ci <= '1'; e_nx <= "10"; e_st <= E_ADD;
                        when 13 => e_h <= e_x and v_bq;                               -- AND
                        when 14 =>                                                    -- TRI
                            v_tr := unsigned(e_x(23 downto 8));
                            if v_tr(15) = '1' then e_h <= signed(resize(not v_tr(14 downto 0), 28));
                            else e_h <= signed(resize(v_tr(14 downto 0), 28)); end if;
                        when 15 =>                                                    -- LOOP
                            e_fi <= e_fi + 1; e_st <= E_F0;
                            if e_fi = 3 then e_pc <= e_pc + 1; else e_pc <= unsigned(e_ir(23 downto 16)); end if;
                        when others => null;
                    end case;
                when E_EX   =>
                    if v_op = 5 then e_h <= signed(resize(unsigned(dr_q), 28));
                    else e_h <= signed(resize(unsigned(ex_q), 28)); end if;
                    e_st <= E_WR;
                when E_ADD  =>
                    e_h <= v_sum; e_ci <= '0';
                    case e_nx is
                        when "01"   => e_st <= E_GL2;
                        when "10"   => e_st <= E_SEL;
                        when others => e_st <= E_WR;
                    end case;
                when E_GL2  =>                                   -- e_h = target - state
                    if C_SNAP or (e_h > -8 and e_h < 8) then e_m <= e_h;
                    else e_m <= shift_right(e_h, 3); end if;
                    e_h <= e_x; e_nx <= "00"; e_st <= E_ADD;
                when E_SEL  =>                                   -- e_h = a - b, e_m = not b
                    if (e_h(27) = '1') = (v_n(0) = '1') then e_h <= e_x; else e_h <= not e_m; end if;
                    e_st <= E_WR;
                when E_MUL  =>
                    if e_b(0) = '1' then v_bq := v_sum; else v_bq := e_h; end if;
                    if e_cnt < v_n then
                        e_h <= shift_right(v_bq, 1);
                    else
                        e_h <= v_bq; e_m <= shift_left(e_m, 1);
                    end if;
                    e_b <= '0' & e_b(15 downto 1);
                    e_cnt <= e_cnt + 1;
                    if e_cnt = 15 then e_st <= E_WR; end if;
                when E_SH   =>
                    if e_cnt = 0 then
                        e_st <= E_WR;
                    else
                        if e_shl = '1' then e_h <= shift_left(e_h, 1); else e_h <= shift_right(e_h, 1); end if;
                        e_cnt <= e_cnt - 1;
                    end if;
                when E_WR   =>
                    rf_we <= '1'; rf_wa <= v_d; rf_wd <= std_logic_vector(resize(e_h, 32));
                    e_pc <= e_pc + 1; e_st <= E_F0;
            end case;

            if s_zgo = '1' then
                e_pc <= (others => '0'); e_fi <= (others => '0'); e_st <= E_F0;
            elsif fl_done = '1' then
                e_pc <= C_CF_MAIN; e_fi <= (others => '0'); e_st <= E_F0;
            end if;
        end if;
    end process p_eng;

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
    -- Source: camera luma or the oscillator lattice -> two one-pole lags along
    -- the line -> a vertical lag (one state per 8 px, read-modify-write once per
    -- group) -> a last lag; self-healing filters, zero with a black input
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l  : unsigned(7 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_in : unsigned(7 downto 0);
        variable v_d, v_e : signed(12 downto 0);
        variable v_ra : unsigned(10 downto 0);
        variable v_o  : signed(9 downto 0);
        variable v_cgs : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + shift_left(resize(s_diy, 16), 1);
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);
            if s_src = '1' then
                v_in := s_osum;
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then v_in := v_l - 16; else v_in := (others => '0'); end if;
            end if;
            if data_in.avid = '1' then
                s_wcol <= s_wcol + 1;
                v_d := signed(resize(v_in & "0000", 13)) - signed(resize(v_la, 13));
                v_e := signed(resize(v_la, 13)) + shift_right(v_d, 3); v_la <= unsigned(v_e(11 downto 0));
                v_d := signed(resize(v_la, 13)) - signed(resize(v_lb, 13));
                v_e := signed(resize(v_lb, 13)) + shift_right(v_d, 3); v_lb <= unsigned(v_e(11 downto 0));
            else
                s_wcol <= (others => '0'); v_la <= (others => '0'); v_lb <= (others => '0');
            end if;
            -- vertical lag: read the next group's state during the second half of a group
            v_ra := s_wcol + 4;
            vb_ra <= v_ra(10 downto 3);
            vb_f  <= unsigned(vb_q(11 downto 0));
            vb_we <= '0';
            if s_wcol(2 downto 0) = "011" then
                v_d := signed(resize(v_lb, 13)) - signed(resize(vb_f, 13));
                v_e := signed(resize(vb_f, 13)) + shift_right(v_d, 2); vb_n <= unsigned(v_e(11 downto 0));
            end if;
            if s_wcol(2 downto 0) = "100" and data_in.avid = '1' then vb_we <= '1'; end if;
            vb_wa <= s_wcol(10 downto 3);
            vb_wd <= "0000" & std_logic_vector(vb_n);
            v_d := signed(resize(vb_n, 13)) - signed(resize(v_fs, 13));
            v_e := signed(resize(v_fs, 13)) + shift_right(v_d, 2); v_fs <= unsigned(v_e(11 downto 0));
            -- the two uses: edge displacement (Q4 entries) and the stain
            v_hl <= v_lb(11 downto 4);
            if s_src = '1' then
                v_o  := signed(resize(v_fs(11 downto 4), 10)) - 111;
                v_vd <= shift_left(resize(v_o, 12), 2);
                v_st <= resize(shift_right(v_hl, 1), 8) - resize(shift_right(v_hl, 4), 8);
            else
                v_vd <= signed(resize(v_fs(11 downto 4) & '0', 12));
                v_st <= v_hl - resize(shift_right(v_hl, 2), 8);
            end if;
            -- K6 camera pass: the camera gain, one multiply per use in its own stage, then the slice
            v_vdp <= v_vd * signed('0' & s_cg);
            v_stp <= v_st * s_cg;
            -- K6 top +25 %: gain in Q4, so the slices drop a bit (displacement max 478 x 79 / 16 = 2360)
            v_vdg <= v_vdp(16 downto 4);
            if v_stp(14 downto 12) /= "000" then v_stg <= (others => '1'); else v_stg <= v_stp(11 downto 4); end if;
            -- K6 glide, once per field from the zone start: 1/8 of the gap, +-4 deadband; gain = 1.25 k / 8
            -- (Q4: (k / 8 + k / 32) / 2): 25 % = x1.25, 50 % = x2.5, 100 % = x4.94 (was x1 / x2 / x4); k < 8 = 0
            s_kcp <= s_kcp(2 downto 0) & s_zgo;
            if s_zgo = '1' then s_kcr <= unsigned(registers_in(5)); end if;
            if s_kcp(0) = '1' then s_kcd <= signed(resize(s_kcr & "000", 15)) - signed(resize(s_kc, 15)); end if;
            if s_kcp(1) = '1' then
                if s_kcd > 32 or s_kcd < -32 then s_kcd <= shift_right(s_kcd, 3); else s_kcd <= (others => '0'); end if;
            end if;
            if s_kcp(2) = '1' then s_kc <= unsigned(signed(s_kc) + s_kcd(12 downto 0)); end if;
            if s_kcp(3) = '1' then
                v_cgs := resize(s_kc(12 downto 6), 8) + resize(s_kc(12 downto 8), 8);
                s_cg <= v_cgs(7 downto 1);
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line and beam accumulators (stage 1, started two clocks after the
    -- generated avid) and the generators
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            g_st1 <= g_start; g_st2 <= g_st1;
            if g_st2 = '1' then
                b_na <= l_na; b_nb <= l_nb; b_nc <= l_nc; b_nd <= l_nd;
                b_iy <= l_iy(16 downto 4); b_row <= l_cnt;
                l_na <= l_na + c_dla; l_nb <= l_nb - c_dlb; l_nc <= l_nc + c_dlc; l_nd <= l_nd + c_dlc;
                l_iy <= l_iy + resize(s_diy, 17);
                l_cnt <= l_cnt + 1;
            elsif s_zone = '1' then
                l_na <= c_nz0; l_nb <= c_nz1; l_nc <= c_nz2; l_nd <= c_nz3;
                l_iy <= (others => '0'); l_cnt <= (others => '0');
            elsif s_gav_sr(1) = '1' then
                b_na <= b_na + c_dpa; b_nb <= b_nb + c_dpa; b_nc <= b_nc + c_dpc; b_nd <= b_nd - c_dpd;
            end if;

            -- stage 3: fixed random numbers (the same every field)
            if s_gav_sr(2) = '1' then
                q_lfsr <= q_lfsr(29 downto 0) & (q_lfsr(30) xor q_lfsr(27));
                q_w <= q_w + x"9E37";
            elsif s_zone = '1' then
                q_lfsr <= "0101010010110100101101001011011";
                q_w <= (others => '0');
            end if;
            -- stage 5: table x coordinate, dither column
            q_in <= s_gav_sr(4);
            if s_gav_sr(4) = '1' then
                if q_in = '1' then
                    q_ix <= q_ix + resize(s_dix, 17); q_col <= q_col + 1;
                else
                    q_ix <= (others => '0'); q_col <= (others => '0');
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_r1, v_r2 : unsigned(9 downto 0);
        variable v_r3 : unsigned(6 downto 0);
        variable v_s  : signed(9 downto 0);
        variable v_by : unsigned(3 downto 0);
        variable v_lo, v_ca : signed(15 downto 0);
        variable v_kp : unsigned(8 downto 0);
        variable v_ql, v_qc : unsigned(1 downto 0);
        variable v_sh : unsigned(2 downto 0);
        variable v_g  : signed(11 downto 0);
        variable v_am : signed(6 downto 0);
    begin
        if rising_edge(clk) then
            -- 3: fabric copies of the noise ROM data
            p3_na <= signed(na_q); p3_nb <= signed(nb_q); p3_nc <= signed(nc_q); p3_nd <= signed(nd_q);
            -- 4: the two noise fields (H displaces the V lookup, V the H lookup); canvas grain = fixed
            --    noise + the mottle (both fields at 1/4), into the delay ring for stage 12
            p4_nh <= resize(p3_na, 9) + resize(p3_nb, 9);
            p4_nv <= resize(p3_nc, 9) + resize(p3_nd, 9);
            v_r3 := unsigned(q_lfsr(26 downto 20)) xor q_w(13 downto 7);
            v_s := resize(p3_na, 10) + resize(p3_nb, 10) + resize(p3_nc, 10) + resize(p3_nd, 10);
            g4 <= signed(resize(v_r3, 10)) - 64 + shift_right(v_s, 2);
            -- 5: x amplitude
            v_am := signed('0' & c_amp);
            p5_mh <= p4_nh * v_am;
            p5_mv <= p4_nv * v_am;
            -- 6: table coordinates (Q4 entries) + noise + source; dither + scumble
            v_r1 := unsigned(q_lfsr(9 downto 0)) xor q_w(15 downto 6);
            v_r2 := unsigned(q_lfsr(19 downto 10)) xor q_w(10 downto 1);
            v_by := f_bayer(b_row, q_col);
            if s_cols = '0' then
                p6_th <= signed(resize(b_iy, 16)) + resize(shift_right(p5_mh, 4), 16) + resize(v_vdg, 16);
                p6_tv <= signed(resize(q_ix(16 downto 4), 16)) + resize(shift_right(p5_mv, 4), 16);
            else
                p6_th <= signed(resize(b_iy, 16)) + resize(shift_right(p5_mh, 4), 16);
                p6_tv <= signed(resize(q_ix(16 downto 4), 16)) + resize(shift_right(p5_mv, 4), 16) + resize(v_vdg, 16);
            end if;
            if s_groff = '0' then
                p6_e1 <= signed(resize(v_by, 12)) + signed(resize(v_r1 and c_mask, 12)) - signed(resize(c_half, 12));
                p6_e2 <= signed(resize(v_by, 12)) + signed(resize(v_r2 and c_mask, 12)) - signed(resize(c_half, 12));
            else
                p6_e1 <= signed(resize(v_by, 12)); p6_e2 <= signed(resize(v_by, 12));
            end if;
            -- 7: the displaced coordinates
            p7_dy <= p6_th + resize(p6_e1, 16);
            p7_dx <= p6_tv + resize(p6_e2, 16);
            -- 8: table addresses (Stacked: long = y, cross = x; Columns: swapped), clamped
            if s_cols = '1' then v_lo := p7_dx; v_ca := p7_dy; else v_lo := p7_dy; v_ca := p7_dx; end if;
            if v_lo(15) = '1' then pf_ra <= (others => '0');
            elsif v_lo(14 downto 13) /= "00" then pf_ra <= (others => '1');
            else pf_ra <= unsigned(v_lo(12 downto 4)); end if;
            if v_ca(15) = '1' then cr_ra <= (others => '0');
            elsif v_ca(14 downto 13) /= "00" then cr_ra <= (others => '1');
            else cr_ra <= unsigned(v_ca(12 downto 4)); end if;
            -- 9: table data (EBR); 10: fabric copies
            p10_pf <= pf_q; p10_kc <= unsigned(cr_q);
            -- 11: the keyer: E x cross key (one multiplier per channel); the stain, keyed by both keys' top bits
            v_kp := resize(p10_kc, 9) + resize(p10_kc(7 downto 7), 9);
            p11_my <= signed(p10_pf(31 downto 22)) * signed('0' & v_kp(8 downto 1));
            p11_mu <= signed(p10_pf(21 downto 12)) * signed('0' & v_kp(8 downto 2));
            p11_mv <= signed(p10_pf(11 downto 2)) * signed('0' & v_kp(8 downto 2));
            v_ql := unsigned(p10_pf(1 downto 0)); v_qc := p10_kc(7 downto 6);
            v_sh := to_unsigned(6, 3) - resize(v_ql, 3) - resize(v_qc, 3);
            if v_ql = 0 or v_qc = 0 then p11_st <= (others => '0');
            else
                case to_integer(v_sh) is
                    when 0      => p11_st <= v_stg;
                    when 1      => p11_st <= shift_right(v_stg, 1);
                    when 2      => p11_st <= shift_right(v_stg, 2);
                    when 3      => p11_st <= shift_right(v_stg, 3);
                    when others => p11_st <= shift_right(v_stg, 4);
                end case;
            end if;
            -- 12: + ground
            p12_y <= signed(resize(c_gy, 12)) + resize(shift_right(p11_my, 6), 12) + signed(resize(p11_st, 12));
            v_g := signed(resize(c_gu, 12)) + resize(shift_right(p11_mu, 5), 12); p12_u <= unsigned(v_g(9 downto 0));
            v_g := signed(resize(c_gv, 12)) + resize(shift_right(p11_mv, 5), 12); p12_v <= unsigned(v_g(9 downto 0));
            -- 13 (the shell's stage 15): canvas grain, clamp; ground outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= c_gy; s_u15 <= c_gu(9 downto 2); s_v15 <= c_gv(9 downto 2);
            else
                if s_groff = '0' then v_g := p12_y + resize(signed(rg_f(63 downto 54)), 12); else v_g := p12_y; end if;
                if v_g < 0 then s_y15 <= (others => '0');
                elsif v_g > 1023 then s_y15 <= (others => '1');
                else s_y15 <= unsigned(v_g(9 downto 0)); end if;
                s_u15 <= p12_u(9 downto 2); s_v15 <= p12_v(9 downto 2);
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the HD HDMI bar fix), luma 13/16 and the
    -- blanking gate.  Off the generated avid both boxes are fed the ground.  The
    -- 8-back taps come from the delay ring; chroma is 8 bits into box A, 9 bits
    -- into box B and 10 bits out.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_cc : unsigned(12 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- The running 8-sums snap back to the ground (the off-avid input of both boxes) once
            -- 8 blank samples are in, so a glitched sample can't offset the colours for good.
            if s_bah = "00000000" then
                s_bay <= c_gy & "000"; s_bau <= c_gu(9 downto 2) & "000"; s_bav <= c_gv(9 downto 2) & "000";
            else
                s_bay <= s_bay + s_y15 - unsigned(rg_f(53 downto 44));
                s_bau <= s_bau + s_u15 - unsigned(rg_f(43 downto 36));
                s_bav <= s_bav + s_v15 - unsigned(rg_f(35 downto 28));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_cc := resize(resize(s_bau, 12) + s_baud, 13); s_u17 <= v_cc(11 downto 3);
                v_cc := resize(resize(s_bav, 12) + s_bavd, 13); s_v17 <= v_cc(11 downto 3);
            else
                s_y17 <= c_gy; s_u17 <= c_gu(9 downto 2) & '0'; s_v17 <= c_gv(9 downto 2) & '0';
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            if s_bbh = "00000000" then
                s_bby <= c_gy & "000"; s_bbu <= c_gu(9 downto 2) & "0000"; s_bbv <= c_gv(9 downto 2) & "0000";
            else
                s_bby <= s_bby + s_y17 - unsigned(rg_f(27 downto 18));
                s_bbu <= s_bbu + s_u17 - unsigned(rg_f(17 downto 9));
                s_bbv <= s_bbv + s_v17 - unsigned(rg_f(8 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y19 <= v_sc(13 downto 4);
            v_cc := resize(s_bbu, 13) + s_bbud; s_u19 <= v_cc(12 downto 3);
            v_cc := resize(s_bbv, 13) + s_bbvd; s_v19 <= v_cc(12 downto 3);
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

end architecture colorfield;
