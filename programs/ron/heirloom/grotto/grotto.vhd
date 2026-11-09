-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Grotto v0.1.0 -- Heirloom (phase 3, video clips), from the still WIN_20240104_00_30_04 at 3.0 s.
--
-- The still: a blue / white / black cavern "face": a hard-keyed black eye with a pale rim in the middle, a
-- black slit keyed above it to the right, soft black lenses and basins, and white bands with green edges
-- folding around everything (X crossings and chevrons across the top).  Mechanism: ONE smooth field S
-- through a folding colouriser (black -> navy -> blue -> green -> light blue -> pale, folding back to navy
-- and up again), with a hard key where S passes a level (the eye and the slit).  Across the clip an LFO
-- (period ~10 s) grows the eye and sinks it while the slit closes, then reverses.
--
-- The patch, as built (grotto.py has every table):
--   S      = measured field (64 x 64 knot table, bilinear; a per-line slice of it is built in block RAM)
--            + eye bump (separable profile x amplitude E) + slit bump + Etch * luma (camera or Osc)
--   key    = S >= 470 (w units): the eye and the slit (black, or Iris rings)
--   w2     = S + Tide, folded faster above the first crest by Rings
--   p      = black ramp below the first crest, then a triangle fold between navy and pale
--   colour = palette(p) with a green band where p is near the green level (width = Green)
--
--   K1 Rings  fold frequency above the first crest (x0.33 .. x3; 512 = the still)
--   K2 Tide   level shift: black floods in (left) / drains away (right); centre = the still
--   K3 Green  width of the green bands (0 = the clip's 0.5 s look; 512 = the still)
--   K4 Speed  breathing LFO rate: 0 = held, 512 = 10 s (the clip), 1023 = ~1.2 s
--   K5 Slit   the slit's level: closed (left) .. the still (centre) .. a black wing (right)
--   K6 Etch   how far the camera (or the Osc) bends the field
--   S7 Twin   the field mirrored about the centre: two eyes
--   S8 Iris   Hole (the still's black eye) / Rings (the fold carries on inside the eye)
--   S9 Palette Blue (the still) / Gold (the clip WIN_20240104_00_15_02)
--   S10 Wave  the field's x lookup swims in a sideways wave
--   S11 Source Video / Osc
--   P12 EYE   eye amplitude: shut (0) .. the still (512) .. wide open (1023)
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine kicked at each
-- line start (computes the NEXT line: table row and fraction, the two bump line amplitudes, the wave
-- shift), a slice builder (the next line's 64 knot values, bilinear in y, into a ping-pong block RAM), a
-- pixel path (x interpolation, bumps, etch, key, fold, palette).  Virtual 1080-line coordinates (SD shows
-- the centre 4:3).  Generated avid; output conditioning (Y, U, V through two [1 2 2 2 2 2 2 2 1]/16 boxes
-- with the running-sum resync, luma 13/16) before the blanking gate.  Colours in the hardware convention
-- (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.grotto_pkg.all;
use work.all;

architecture grotto of program_top is
    constant C_LAT : integer := 28;   -- = processing_delay_clks

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
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return (not v(6 downto 0)) & '0'; else return v(6 downto 0) & '0'; end if;
    end function;
    -- +-8 deadband of a 10-bit knob about 512: signed |k - 512| - 8
    function f_db(k : unsigned(9 downto 0)) return signed is
        variable v : signed(10 downto 0);
    begin
        v := signed(resize(k, 11)) - 512;
        if v > -9 and v < 9 then return to_signed(0, 11);
        elsif v > 0 then return v - 8;
        else return v + 8; end if;
    end function;

    -- ROMs (registered address and data; a fabric copy after each)
    signal r_tb_a  : unsigned(11 downto 0) := (others => '0');
    signal r_tb_q  : unsigned(9 downto 0) := (others => '0');
    signal r_pl_a  : unsigned(8 downto 0) := (others => '0');
    signal r_pl_q  : std_logic_vector(23 downto 0) := (others => '0');
    signal r_be_a, r_bs_a : unsigned(7 downto 0) := (others => '0');
    signal r_be_q, r_bs_q : unsigned(7 downto 0) := (others => '0');
    signal r_sn_a  : unsigned(7 downto 0) := (others => '0');
    signal r_sn_q  : signed(7 downto 0) := (others => '0');
    signal r_ex_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q  : unsigned(15 downto 0) := (others => '0');

    -- slice RAM: 2 banks x 64 entries of {s(k), s(k+1)}; the builder writes the bank the pixel path is
    -- not reading
    type t_sl is array (0 to 127) of std_logic_vector(19 downto 0);
    signal sl : t_sl := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of sl : signal is true;
    signal sl_we  : std_logic := '0';
    signal sl_wa, sl_ra : unsigned(6 downto 0) := (others => '0');
    signal sl_wd  : std_logic_vector(19 downto 0) := (others => '0');
    signal sl_q   : std_logic_vector(19 downto 0) := (others => '0');
    signal s_bank : std_logic := '0';

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

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(448 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_twin, s_iris, s_pal, s_wave, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field sequencer
    signal c_phi  : unsigned(15 downto 0) := (others => '0');      -- breathing LFO phase
    signal c_dphi : unsigned(15 downto 0) := to_unsigned(109, 16);
    signal c_phw  : unsigned(11 downto 0) := (others => '0');      -- wave drift
    signal c_phox, c_phoy : unsigned(15 downto 0) := (others => '0');
    signal z_m    : signed(7 downto 0) := (others => '0');
    signal z_mabs : unsigned(6 downto 0) := (others => '0');
    signal z_mneg : std_logic := '0';
    signal z_e12  : signed(10 downto 0) := (others => '0');
    signal z_eabs : unsigned(9 downto 0) := (others => '0');
    signal z_eneg : std_logic := '0';
    signal z_k5d, z_d2, z_k1, z_e4 : signed(10 downto 0) := (others => '0');
    signal z_e1, z_e2, z_y1, z_y2, z_s1, z_s2, z_s3 : signed(10 downto 0) := (others => '0');
    signal z_e8, z_es : signed(10 downto 0) := (others => '0');
    signal z_sq   : unsigned(7 downto 0) := (others => '0');
    signal z_seam : signed(7 downto 0) := (others => '0');
    signal c_ew   : signed(10 downto 0) := (others => '0');        -- eye bump amplitude (w)
    signal c_ye   : signed(11 downto 0) := to_signed(600, 12);     -- eye bump centre (virtual line)
    signal c_slw  : signed(11 downto 0) := (others => '0');        -- slit bump amplitude (w)
    signal c_tide : signed(9 downto 0) := (others => '0');
    signal c_tm   : signed(10 downto 0) := to_signed(-256, 11);    -- tide - 256
    signal c_fr7  : unsigned(8 downto 0) := to_unsigned(128, 9);   -- fold frequency, Q7
    signal c_et8  : unsigned(7 downto 0) := to_unsigned(44, 8);    -- etch gain, w per luma step, Q8
    signal c_wg, c_wg3 : unsigned(4 downto 0) := to_unsigned(11, 5);
    signal c_xsh  : unsigned(11 downto 0) := to_unsigned(730, 12);  -- seam x (h)
    signal c_sl0, c_sl1 : unsigned(11 downto 0) := to_unsigned(730, 12);
    signal c_oxs  : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal c_oys  : unsigned(9 downto 0) := to_unsigned(121, 10);

    -- field serial multiplier (unsigned magnitudes)
    signal f_a   : unsigned(31 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(31 downto 0) := (others => '0');
    signal z_fa  : unsigned(31 downto 0) := (others => '0');
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
    signal e_px   : unsigned(10 downto 0) := (others => '0');
    signal e_ad   : unsigned(9 downto 0) := (others => '0');
    signal e_r    : unsigned(8 downto 0) := (others => '0');
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(31 downto 0) := (others => '0');
    signal e_kick : std_logic := '0';
    signal n_ce   : signed(11 downto 0) := (others => '0');
    signal n_cs   : signed(12 downto 0) := (others => '0');
    signal n_ws   : signed(9 downto 0) := (others => '0');
    signal e_sneg : std_logic := '0';
    signal s_oy   : unsigned(15 downto 0) := (others => '0');
    signal l_ce   : signed(11 downto 0) := (others => '0');
    signal l_cs   : signed(12 downto 0) := (others => '0');
    signal l_ws   : signed(9 downto 0) := (others => '0');
    signal l_ty   : unsigned(7 downto 0) := (others => '0');

    -- slice builder
    signal b_run  : std_logic := '0';
    signal b_i    : unsigned(7 downto 0) := (others => '0');
    signal b_j    : unsigned(5 downto 0) := (others => '0');
    signal b_fy   : unsigned(7 downto 0) := (others => '0');
    signal b_v    : std_logic_vector(3 downto 0) := (others => '0');   -- valid pipe of the reads
    signal b_odd  : std_logic_vector(3 downto 0) := (others => '0');
    signal b_k0, b_k1, b_k2, b_k3 : unsigned(5 downto 0) := (others => '0');
    signal b_r    : unsigned(9 downto 0) := (others => '0');
    signal b_a    : unsigned(9 downto 0) := (others => '0');
    signal b_d    : signed(10 downto 0) := (others => '0');
    signal b_a2   : unsigned(9 downto 0) := (others => '0');
    signal b_pr   : signed(19 downto 0) := (others => '0');
    signal b_a3   : unsigned(9 downto 0) := (others => '0');
    signal b_go4, b_go5, b_go6 : std_logic := '0';
    signal b_pl, b_ph : signed(15 downto 0) := (others => '0');
    signal b_a4   : unsigned(9 downto 0) := (others => '0');
    signal b_kk5  : unsigned(5 downto 0) := (others => '0');
    signal b_kk3, b_kk4 : unsigned(5 downto 0) := (others => '0');
    signal b_s    : unsigned(9 downto 0) := (others => '0');
    signal b_wb   : std_logic := '0';

    -- beam
    signal b_vxh  : unsigned(11 downto 0) := (others => '0');
    signal b_ox   : unsigned(15 downto 0) := (others => '0');

    -- pixel path
    signal q2_xa  : signed(13 downto 0) := (others => '0');
    signal q2_sm, q2_sl : std_logic := '0';
    signal q2_tx  : unsigned(7 downto 0) := (others => '0');
    signal q3_xt  : signed(13 downto 0) := (others => '0');
    signal q3_sl : std_logic := '0';
    signal q3_tx  : unsigned(7 downto 0) := (others => '0');
    signal q4_x   : unsigned(11 downto 0) := (others => '0');
    signal q4_sl  : std_logic := '0';
    signal q4_lum : signed(9 downto 0) := (others => '0');
    signal q5_fx  : unsigned(4 downto 0) := (others => '0');
    signal q5_de, q5_ds : signed(11 downto 0) := (others => '0');
    signal q5_lum : signed(9 downto 0) := (others => '0');
    signal q6_fx  : unsigned(4 downto 0) := (others => '0');
    signal q6_lum : signed(9 downto 0) := (others => '0');
    signal q7_a, q7_b : unsigned(9 downto 0) := (others => '0');
    signal q7_fx  : unsigned(4 downto 0) := (others => '0');
    signal q7_lum : signed(9 downto 0) := (others => '0');
    signal q8_d   : signed(10 downto 0) := (others => '0');
    signal q8_a   : unsigned(9 downto 0) := (others => '0');
    signal q8_fx  : unsigned(4 downto 0) := (others => '0');
    signal q8_be, q8_bs : unsigned(7 downto 0) := (others => '0');
    signal q8_lum : signed(9 downto 0) := (others => '0');
    signal q9_pr  : signed(16 downto 0) := (others => '0');
    signal q9_a   : unsigned(9 downto 0) := (others => '0');
    signal q9_pel, q9_peh : signed(16 downto 0) := (others => '0');
    signal q9_psl, q9_psh : signed(17 downto 0) := (others => '0');
    signal q9_etl, q9_eth : signed(14 downto 0) := (others => '0');
    signal q10_s0 : signed(11 downto 0) := (others => '0');
    signal q10_be, q10_bs : signed(11 downto 0) := (others => '0');
    signal q10_et : signed(10 downto 0) := (others => '0');
    signal q11_u, q11_v : signed(12 downto 0) := (others => '0');
    signal q12_sk : signed(12 downto 0) := (others => '0');
    signal q13_sa : signed(12 downto 0) := (others => '0');
    signal q13_ki : unsigned(7 downto 0) := (others => '0');
    signal q13_key : std_logic := '0';
    signal q13_v  : unsigned(9 downto 0) := (others => '0');
    signal q13_hi : std_logic := '0';
    signal q14_pl, q14_ph : unsigned(13 downto 0) := (others => '0');
    signal q14_hi : std_logic := '0';
    signal q14_sa : signed(12 downto 0) := (others => '0');
    signal q14_ti : unsigned(7 downto 0) := (others => '0');
    signal q15_pr : unsigned(18 downto 0) := (others => '0');
    signal q15_hi : std_logic := '0';
    signal q15_sa : signed(12 downto 0) := (others => '0');
    signal q15_ti : unsigned(7 downto 0) := (others => '0');
    signal q16_w2 : signed(13 downto 0) := (others => '0');
    signal q16_ti : unsigned(7 downto 0) := (others => '0');
    signal q17_lo, q17_mid : std_logic := '0';
    signal q17_r  : unsigned(7 downto 0) := (others => '0');
    signal q17_tr : unsigned(7 downto 0) := (others => '0');
    signal q17_ti : unsigned(7 downto 0) := (others => '0');
    signal q18_p, q18_pi : unsigned(7 downto 0) := (others => '0');
    signal q19_dg : unsigned(7 downto 0) := (others => '0');
    signal q20_full, q20_half : std_logic := '0';
    signal q21_full, q21_half : std_logic := '0';
    signal q21_pl : std_logic_vector(23 downto 0) := (others => '0');
    signal s_key_sr : std_logic_vector(9 downto 0) := (others => '0');   -- key from stage 13 on
    signal s_sl_sr  : std_logic_vector(17 downto 0) := (others => '0');  -- seam line from stage 4 on
    signal q22_y, q22_u, q22_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- video tap (luma, low-passed)
    signal v_y8   : unsigned(7 downto 0) := (others => '0');
    signal v_lp   : unsigned(10 downto 0) := (others => '0');
    signal v_lum  : unsigned(7 downto 0) := (others => '0');

    -- output conditioning
    -- the boxes' 8-back samples come from two 256 x 32 block-RAM rings (read pointer = write - 6, then a
    -- fabric register: an exact 8-clock delay), not from FF delay lines
    type t_ob is array (0 to 255) of std_logic_vector(31 downto 0);
    signal ob_a, ob_b : t_ob := (others => (others => '0'));
    attribute no_rw_check of ob_a : signal is true;
    attribute no_rw_check of ob_b : signal is true;
    signal o_wp : unsigned(7 downto 0) := (others => '0');
    signal oa_q, ob_q, oa_f, ob_f : std_logic_vector(31 downto 0) := (others => '0');
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y25, s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y27, s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y28, s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y29 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u29, s_v29 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- ROMs and the slice RAM
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_tb_q <= C_GR_TAB(to_integer(r_tb_a));
            r_pl_q <= C_GR_PAL(to_integer(r_pl_a));
            r_be_q <= C_GR_BXE(to_integer(r_be_a));
            r_bs_q <= C_GR_BXS(to_integer(r_bs_a));
            r_sn_q <= C_GR_SIN(to_integer(r_sn_a));
            r_ex_q <= C_GR_EXP2(to_integer(r_ex_a));
        end if;
    end process p_rom;

    p_sl : process(clk)
    begin
        if rising_edge(clk) then
            sl_q <= sl(to_integer(sl_ra));
            if sl_we = '1' then sl(to_integer(sl_wa)) <= sl_wd; end if;
        end if;
    end process p_sl;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_e : signed(10 downto 0);
        variable v_t : signed(11 downto 0);
        variable v_a9 : signed(8 downto 0);
        variable v_a11 : signed(10 downto 0);
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
                s_twin <= registers_in(6)(0);
                s_iris <= registers_in(6)(1);
                s_pal  <= registers_in(6)(2);
                s_wave <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); c_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     c_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     c_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            c_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

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

            -- serial products: slot k loads at zc = 64 + 24 k, 16 iterations, result stable from load + 17
            -- (stored at load + 18).  The slot is decoded at 62 + 24 k and the operands pre-muxed at 63 + 24 k.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 62  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 86  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 110 => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 134 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 158 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 182 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 206 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 230 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 254 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 278 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 302 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 326 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when 350 => z_sel <= to_unsigned(12, 4); z_pre <= '1';
                    when 374 => z_sel <= to_unsigned(13, 4); z_pre <= '1';
                    when 398 => z_sel <= to_unsigned(14, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(z_eabs, 32);                                        -- E1
                               if z_eneg = '1' then z_fb <= to_unsigned(233, 16); else z_fb <= to_unsigned(210, 16); end if;
                    when 1  => z_fa <= resize(z_mabs, 32);                                        -- E2
                               if z_mneg = '1' then z_fb <= to_unsigned(140, 16); else z_fb <= to_unsigned(65, 16); end if;
                    when 2  => z_fa <= resize(z_mabs, 32); z_fb <= to_unsigned(140, 16);           -- Y1
                    when 3  => z_fa <= resize(z_eabs, 32); z_fb <= to_unsigned(150, 16);           -- Y2
                    when 4  => z_fa <= resize(unsigned(abs(z_k5d)), 32); z_fb <= to_unsigned(303, 16);  -- S1
                    when 5  => z_fa <= resize(z_mabs, 32);                                        -- S2
                               if z_mneg = '1' then z_fb <= to_unsigned(302, 16); else z_fb <= to_unsigned(128, 16); end if;
                    when 6  => z_fa <= resize(z_eabs, 32); z_fb <= to_unsigned(140, 16);           -- S3
                    when 7  => z_fa <= resize(unsigned(abs(z_d2)), 32); z_fb <= to_unsigned(189, 16);   -- tide
                    when 8  => z_fa <= resize(unsigned(abs(z_k1)), 32); z_fb <= to_unsigned(205, 16);   -- rings octaves
                    when 9  => z_fa <= resize(unsigned(abs(z_e4)), 32); z_fb <= to_unsigned(386, 16);   -- speed octaves
                    when 10 => z_fa <= resize(s_g6(12 downto 3), 32); z_fb <= resize(s_g6(12 downto 3), 16);  -- etch^2
                    when 11 => z_fa <= resize(z_sq, 32); z_fb <= to_unsigned(233, 16);             -- etch gain
                    when 12 => z_fa <= resize(s_g3(12 downto 3), 32); z_fb <= to_unsigned(11, 16); -- green width
                    when 13 => z_fa <= resize(z_mabs, 32); z_fb <= to_unsigned(60, 16);            -- seam drift
                    when others => z_fa <= resize(r_ex_q, 32); z_fb <= to_unsigned(109, 16);       -- speed
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(30 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 20 =>
                        c_phi  <= c_phi + c_dphi;
                        c_phw  <= c_phw + 9;
                        c_phox <= c_phox + 65;
                        c_phoy <= c_phoy + 44;
                    when 24 =>
                        z_m <= r_sn_q;
                        z_mneg <= r_sn_q(7);
                        v_a9 := abs(resize(r_sn_q, 9));
                        z_mabs <= unsigned(v_a9(6 downto 0));
                        v_e := signed(resize(s_gp(12 downto 3), 11)) - 512;
                        z_e12 <= v_e;
                        z_eneg <= v_e(10);
                        v_a11 := abs(v_e);
                        z_eabs <= unsigned(v_a11(9 downto 0));
                        z_k5d <= f_db(s_g5(12 downto 3));
                        z_d2  <= f_db(s_g2(12 downto 3));
                        z_k1  <= signed(resize(s_g1(12 downto 3), 11)) - 512;
                        z_e4  <= signed(resize(s_k4, 11)) - 512;
                    -- results: load + 18 (loads at 64 + 24 k)
                    when 82  => if z_eneg = '1' then z_e1 <= -signed(resize(f_acc(19 downto 9), 11));
                                else z_e1 <= signed(resize(f_acc(19 downto 9), 11)); end if;
                    when 106 => if z_mneg = '1' then z_e2 <= -signed(resize(f_acc(17 downto 7), 11));
                                else z_e2 <= signed(resize(f_acc(17 downto 7), 11)); end if;
                    when 130 => if z_mneg = '1' then z_y1 <= -signed(resize(f_acc(17 downto 7), 11));
                                else z_y1 <= signed(resize(f_acc(17 downto 7), 11)); end if;
                    when 154 => if z_eneg = '1' then z_y2 <= -signed(resize(f_acc(19 downto 9), 11));
                                else z_y2 <= signed(resize(f_acc(19 downto 9), 11)); end if;
                    when 178 => if z_k5d < 0 then z_s1 <= -signed(resize(f_acc(19 downto 9), 11));
                                else z_s1 <= signed(resize(f_acc(19 downto 9), 11)); end if;
                    when 202 => if z_mneg = '1' then z_s2 <= -signed(resize(f_acc(17 downto 7), 11));
                                else z_s2 <= signed(resize(f_acc(17 downto 7), 11)); end if;
                    when 226 => if z_eneg = '1' then z_s3 <= -signed(resize(f_acc(19 downto 9), 11));
                                else z_s3 <= signed(resize(f_acc(19 downto 9), 11)); end if;
                    when 250 => if z_d2 < 0 then c_tide <= -signed(resize(f_acc(17 downto 9), 10));
                                else c_tide <= signed(resize(f_acc(17 downto 9), 10)); end if;
                    when 274 => if z_k1 < 0 then z_e8 <= -signed(resize(f_acc(18 downto 8), 11));
                                else z_e8 <= signed(resize(f_acc(18 downto 8), 11)); end if;
                    when 275 => r_ex_a <= unsigned(z_e8(7 downto 0));
                    when 278 =>                        -- fr7 = exp2(lo) >> (8 - hi), hi = -2 .. 1
                        case to_integer(z_e8(10 downto 8)) is
                            when -3     => c_fr7 <= resize(shift_right(r_ex_q, 11), 9);
                            when -2     => c_fr7 <= resize(shift_right(r_ex_q, 10), 9);
                            when -1     => c_fr7 <= resize(shift_right(r_ex_q, 9), 9);
                            when 0      => c_fr7 <= resize(shift_right(r_ex_q, 8), 9);
                            when others => c_fr7 <= resize(shift_right(r_ex_q, 7), 9);
                        end case;
                    when 298 => if z_e4 < 0 then z_es <= -signed(resize(f_acc(18 downto 8), 11));
                                else z_es <= signed(resize(f_acc(18 downto 8), 11)); end if;
                    when 299 => r_ex_a <= unsigned(z_es(7 downto 0));
                    when 322 => z_sq <= f_acc(19 downto 12);
                    when 346 => c_et8 <= f_acc(15 downto 8);
                    when 370 => c_wg <= f_acc(13 downto 9);
                    when 371 => c_wg3 <= c_wg + 3;
                    when 394 => z_seam <= signed(resize(f_acc(13 downto 7), 8));
                    when 418 =>                        -- dphi = 109 exp2(lo) >> (15 - hi), hi = -4 .. 3; held below 8
                        if s_k4 < 8 then
                            c_dphi <= (others => '0');
                        else
                            case to_integer(z_es(10 downto 8)) is
                                when -4     => c_dphi <= resize(shift_right(f_acc, 19), 16);
                                when -3     => c_dphi <= resize(shift_right(f_acc, 18), 16);
                                when -2     => c_dphi <= resize(shift_right(f_acc, 17), 16);
                                when -1     => c_dphi <= resize(shift_right(f_acc, 16), 16);
                                when 0      => c_dphi <= resize(shift_right(f_acc, 15), 16);
                                when 1      => c_dphi <= resize(shift_right(f_acc, 14), 16);
                                when 2      => c_dphi <= resize(shift_right(f_acc, 13), 16);
                                when others => c_dphi <= resize(shift_right(f_acc, 12), 16);
                            end case;
                        end if;
                    -- sums
                    when 420 => c_ew <= z_e1 + z_e2;
                    when 421 => c_ye <= to_signed(600, 12) + resize(z_y1, 12) + resize(z_y2, 12);
                    when 422 => v_t := resize(z_s1, 12) - resize(z_s2, 12); c_slw <= v_t;
                    when 423 => c_slw <= c_slw - resize(z_s3, 12);
                    when 424 =>
                        if z_mneg = '1' then
                            c_xsh <= to_unsigned(730, 12) + shift_left(resize(unsigned(abs(resize(z_seam, 9))), 12), 1);
                        else
                            c_xsh <= to_unsigned(730, 12) - shift_left(resize(unsigned(z_seam), 12), 1);
                        end if;
                    when 425 => c_sl0 <= c_xsh - 6; c_sl1 <= c_xsh + 6;
                                c_tm <= resize(c_tide, 11) - 256;
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
    -- Video tap: luma (Y - 64) * 0.28 -> 0..246, a 4-pixel IIR low-pass
    ---------------------------------------------------------------------------
    p_vid : process(clk)
        variable v_d : unsigned(9 downto 0);
        variable v_s : unsigned(9 downto 0);
        variable v_df, v_nl : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            if unsigned(data_in.y) > 64 then v_d := unsigned(data_in.y) - 64; else v_d := (others => '0'); end if;
            v_s := shift_right(v_d, 2) + shift_right(v_d, 5);
            if v_s > 255 then v_y8 <= (others => '1'); else v_y8 <= v_s(7 downto 0); end if;
            v_df := signed(resize(v_y8 & "000", 12)) - signed(resize(v_lp, 12));
            v_nl := signed(resize(v_lp, 12)) + shift_right(v_df, 3);
            v_lp <= unsigned(v_nl(10 downto 0));
            if v_lp(10 downto 3) > 32 then v_lum <= v_lp(10 downto 3) - 32; else v_lum <= (others => '0'); end if;
        end if;
    end process p_vid;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line on one serial multiplier (16 iterations,
    -- result at load + 17): the table row / fraction (kicks the slice builder),
    -- the eye and slit line amplitudes (profile (1 - (dy / b)^2)^2 times the
    -- bump amplitude), the wave shift.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d : signed(12 downto 0);
        variable v_da : signed(12 downto 0);
        variable v_q : unsigned(11 downto 0);
        variable v_w : unsigned(7 downto 0);
        variable v_a9 : signed(8 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;
            if s_zone = '1' and s_zc = 1290 then s_oy <= c_phoy; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                s_oy <= s_oy + resize(c_oys, 16);
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;
            if e_lc = 1 then l_ty <= f_tri8(s_oy(15 downto 8)); end if;
            if s_zone = '1' and s_zc = 21 then r_sn_a <= c_phi(15 downto 8); end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(30 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            e_kick <= '0';
            case to_integer(e_lc) is
                when 1 =>
                    m_a <= resize(e_vy, 32); m_b <= to_unsigned(3823, 16); m_acc <= (others => '0');
                    e_px <= e_vy(18 downto 8);
                when 2 =>
                    v_d := signed(resize(e_px, 13)) - resize(c_ye, 13);
                    v_da := abs(v_d);
                    if v_da > 1023 then e_ad <= (others => '1'); else e_ad <= unsigned(v_da(9 downto 0)); end if;
                when 18 =>                     -- row j and fraction fy of the next line: kick the slice builder
                    if m_acc(31 downto 24) > 62 then b_j <= to_unsigned(62, 6); else b_j <= m_acc(29 downto 24); end if;
                    b_fy <= m_acc(23 downto 16);
                    e_kick <= '1';
                    m_a <= resize(e_ad, 32); m_b <= resize(e_ad, 16); m_acc <= (others => '0');
                when 35 =>
                    m_a <= resize(m_acc(19 downto 0), 32); m_b <= to_unsigned(73, 16); m_acc <= (others => '0');
                when 52 =>
                    v_q := m_acc(27 downto 16);
                    if v_q >= 256 then e_r <= (others => '0'); else e_r <= 256 - resize(v_q, 9); end if;
                when 53 =>
                    m_a <= resize(e_r, 32); m_b <= resize(e_r, 16); m_acc <= (others => '0');
                when 70 =>
                    if m_acc(17 downto 8) > 255 then v_w := (others => '1'); else v_w := m_acc(15 downto 8); end if;
                    m_a <= resize(v_w, 32); m_b <= resize(unsigned(abs(c_ew)), 16); m_acc <= (others => '0');
                    v_d := signed(resize(e_px, 13)) - 270;
                    v_da := abs(v_d);
                    if v_da > 1023 then e_ad <= (others => '1'); else e_ad <= unsigned(v_da(9 downto 0)); end if;
                when 87 =>
                    if c_ew < 0 then n_ce <= -signed(resize(m_acc(16 downto 6), 12));
                    else n_ce <= signed(resize(m_acc(16 downto 6), 12)); end if;
                    m_a <= resize(e_ad, 32); m_b <= resize(e_ad, 16); m_acc <= (others => '0');
                when 104 =>
                    m_a <= resize(m_acc(19 downto 0), 32); m_b <= to_unsigned(154, 16); m_acc <= (others => '0');
                when 121 =>
                    v_q := m_acc(27 downto 16);
                    if v_q >= 256 then e_r <= (others => '0'); else e_r <= 256 - resize(v_q, 9); end if;
                when 122 =>
                    m_a <= resize(e_r, 32); m_b <= resize(e_r, 16); m_acc <= (others => '0');
                when 139 =>
                    if m_acc(17 downto 8) > 255 then v_w := (others => '1'); else v_w := m_acc(15 downto 8); end if;
                    m_a <= resize(v_w, 32); m_b <= resize(unsigned(abs(c_slw)), 16); m_acc <= (others => '0');
                when 156 =>
                    if c_slw < 0 then n_cs <= -signed(resize(m_acc(17 downto 6), 13));
                    else n_cs <= signed(resize(m_acc(17 downto 6), 13)); end if;
                    m_a <= resize(e_px, 32); m_b <= to_unsigned(156, 16); m_acc <= (others => '0');
                when 173 =>
                    r_sn_a <= m_acc(15 downto 8) + c_phw(11 downto 4);
                when 176 =>
                    e_sneg <= r_sn_q(7);
                    v_a9 := abs(resize(r_sn_q, 9));
                    m_a <= resize(unsigned(v_a9), 32); m_b <= to_unsigned(300, 16); m_acc <= (others => '0');
                when 193 =>
                    if s_wave = '0' then n_ws <= (others => '0');
                    elsif e_sneg = '1' then n_ws <= -signed(resize(m_acc(16 downto 7), 10));
                    else n_ws <= signed(resize(m_acc(16 downto 7), 10)); end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Slice builder: for k = 0 .. 63 reads the table at (j, k) and (j + 1, k),
    -- s(k) = a + (b - a) fy / 256, and writes {s(k - 1), s(k)} to entry k - 1 of
    -- the bank the pixel path is not reading.  ~135 clocks after the kick.
    ---------------------------------------------------------------------------
    p_build : process(clk)
        variable v_s : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- read issue: i = 0 .. 127, row j + i(0), column i(7 downto 1)
            if e_kick = '1' then
                b_run <= '1'; b_i <= (others => '0'); b_wb <= not s_bank;
            elsif b_run = '1' then
                b_i <= b_i + 1;
                if b_i = 127 then b_run <= '0'; end if;
            end if;
            if b_i(0) = '1' then
                r_tb_a <= (b_j + 1) & b_i(6 downto 1);
            else
                r_tb_a <= b_j & b_i(6 downto 1);
            end if;
            b_v   <= b_v(2 downto 0) & b_run;
            b_odd <= b_odd(2 downto 0) & b_i(0);
            b_k0 <= b_i(6 downto 1); b_k1 <= b_k0; b_k2 <= b_k1; b_k3 <= b_k2;
            -- r_tb_q is valid two clocks after the issue (b_v(1)); fabric copy at b_v(2)
            b_r <= r_tb_q;
            if b_v(2) = '1' and b_odd(2) = '0' then b_a <= b_r; end if;
            if b_v(2) = '1' and b_odd(2) = '1' then
                b_d   <= signed(resize(b_r, 11)) - signed(resize(b_a, 11));
                b_a2  <= b_a;
                b_go4 <= '1';
                b_kk3 <= b_k2;
            else
                b_go4 <= '0';
            end if;
            b_pl <= b_d * signed(resize(b_fy(3 downto 0), 5));
            b_ph <= b_d * signed(resize(b_fy(7 downto 4), 5));
            b_a3 <= b_a2;
            b_kk4 <= b_kk3;
            b_go5 <= b_go4;
            b_pr <= resize(b_pl, 20) + shift_left(resize(b_ph, 20), 4);
            b_a4 <= b_a3;
            b_kk5 <= b_kk4;
            b_go6 <= b_go5;
            sl_we <= '0';
            if b_go6 = '1' then
                v_s := signed(resize(b_a4, 12)) + resize(shift_right(b_pr, 8), 12);
                b_s <= unsigned(v_s(9 downto 0));
                if b_kk5 /= 0 then
                    sl_we <= '1';
                    sl_wa <= b_wb & (b_kk5 - 1);
                    sl_wd <= std_logic_vector(b_s) & std_logic_vector(unsigned(v_s(9 downto 0)));
                end if;
            end if;
        end if;
    end process p_build;

    ---------------------------------------------------------------------------
    -- Beam: virtual x in h units, the Osc x phase; the line registers are
    -- loaded at the line start
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_vxh <= resize(s_x0, 12);
                b_ox  <= c_phox;
                s_bank <= not s_bank;
                l_ce <= n_ce; l_cs <= n_cs; l_ws <= n_ws;
            elsif g_avid = '1' then
                b_vxh <= b_vxh + s_xs;
                b_ox  <= b_ox + c_oxs;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_x   : signed(13 downto 0);
        variable v_j   : signed(13 downto 0);
        variable v_vr  : signed(13 downto 0);
        variable v_ad  : unsigned(11 downto 0);
        variable v_t   : unsigned(8 downto 0);
        variable v_r   : unsigned(10 downto 0);
        variable v_w   : signed(13 downto 0);
        variable v_ph  : unsigned(7 downto 0);
        variable v_dg  : signed(9 downto 0);
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_gy, v_gu, v_gv : unsigned(9 downto 0);
        variable v_kc  : std_logic_vector(23 downto 0);
        variable v_sy  : unsigned(10 downto 0);
        variable v_c   : signed(11 downto 0);
        variable v_s12 : unsigned(11 downto 0);
        variable v_p21 : signed(20 downto 0);
        variable v_p22 : signed(21 downto 0);
        variable v_p19 : signed(18 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: wave shift; seam flags; Osc x triangle
            q2_xa <= signed(resize(b_vxh, 14)) + resize(l_ws, 14);
            if b_vxh < c_xsh then q2_sm <= '1'; else q2_sm <= '0'; end if;
            if b_vxh > c_sl0 and b_vxh < c_sl1 then q2_sl <= '1'; else q2_sl <= '0'; end if;
            q2_tx <= f_tri8(b_ox(15 downto 8));
            -- 3: Twin mirror about the centre (1920 h) and the seam jog (+10 px left of the seam)
            if q2_sm = '1' then v_j := to_signed(20, 14); else v_j := to_signed(0, 14); end if;
            if s_twin = '1' and q2_xa < 1920 then q3_xt <= 3840 + v_j - q2_xa; else q3_xt <= q2_xa + v_j; end if;
            q3_sl <= q2_sl; q3_tx <= q2_tx;
            -- 4: reflect at the table edges (the wave can push the lookup past them)
            v_x := resize(q3_xt, 14);
            if v_x < 0 then v_x := -v_x;
            elsif v_x > 3838 then v_x := 7676 - v_x; end if;
            q4_x <= unsigned(v_x(11 downto 0));
            q4_sl <= q3_sl;
            if s_src = '1' then
                q4_lum <= shift_left(signed(resize(q3_tx, 10)) + signed(resize(l_ty, 10)) - 256, 1);
            else
                q4_lum <= signed(resize(v_lum, 10));
            end if;
            -- 5: slice address; x fraction; distances to the bump centres
            sl_ra <= s_bank & q4_x(11 downto 6);
            q5_fx <= q4_x(5 downto 1);
            q5_de <= signed(resize(q4_x(11 downto 1), 12)) - 1160;
            q5_ds <= signed(resize(q4_x(11 downto 1), 12)) - 1560;
            q5_lum <= q4_lum;
            -- 6: slice RAM data (EBR); bump ROM addresses
            v_ad := unsigned(abs(q5_de));
            if v_ad(11 downto 10) /= "00" then r_be_a <= (others => '1'); else r_be_a <= v_ad(9 downto 2); end if;
            v_ad := unsigned(abs(q5_ds));
            if v_ad(11 downto 10) /= "00" then r_bs_a <= (others => '1'); else r_bs_a <= v_ad(9 downto 2); end if;
            q6_fx <= q5_fx; q6_lum <= q5_lum;
            -- 7: fabric copy of the slice pair; bump ROM data (EBR)
            q7_a <= unsigned(sl_q(19 downto 10)); q7_b <= unsigned(sl_q(9 downto 0));
            q7_fx <= q6_fx; q7_lum <= q6_lum;
            -- 8: difference; bump fabric copies
            q8_d <= signed(resize(q7_b, 11)) - signed(resize(q7_a, 11));
            q8_a <= q7_a; q8_fx <= q7_fx;
            q8_be <= r_be_q; q8_bs <= r_bs_q;
            q8_lum <= q7_lum;
            -- 9: products (x interpolation; eye, slit and etch as nibble partial products)
            q9_pr  <= resize(q8_d * signed(resize(q8_fx, 6)), 17);
            q9_a   <= q8_a;
            q9_pel <= signed(resize(q8_be(3 downto 0), 5)) * l_ce;
            q9_peh <= signed(resize(q8_be(7 downto 4), 5)) * l_ce;
            q9_psl <= signed(resize(q8_bs(3 downto 0), 5)) * l_cs;
            q9_psh <= signed(resize(q8_bs(7 downto 4), 5)) * l_cs;
            q9_etl <= q8_lum * signed(resize(c_et8(3 downto 0), 5));
            q9_eth <= q8_lum * signed(resize(c_et8(7 downto 4), 5));
            -- 10: combine and scale
            q10_s0 <= signed(resize(q9_a, 12)) + resize(shift_right(q9_pr, 5), 12) - 64;
            v_p21 := resize(q9_pel, 21) + shift_left(resize(q9_peh, 21), 4);
            q10_be <= resize(shift_right(v_p21, 10), 12);
            v_p22 := resize(q9_psl, 22) + shift_left(resize(q9_psh, 22), 4);
            q10_bs <= resize(shift_right(v_p22, 10), 12);
            v_p19 := resize(q9_etl, 19) + shift_left(resize(q9_eth, 19), 4);
            q10_et <= resize(shift_right(v_p19, 8), 11);
            -- 11, 12: S = S0 + eye + slit + etch
            q11_u <= resize(q10_s0, 13) + resize(q10_be, 13);
            q11_v <= resize(q10_bs, 13) + resize(q10_et, 13);
            q12_sk <= q11_u + q11_v;
            -- 13: key; tide; iris phase
            if q12_sk >= 470 then q13_key <= '1'; else q13_key <= '0'; end if;
            q13_sa <= q12_sk + resize(c_tide, 13);
            v_vr := resize(q12_sk, 14) + resize(c_tm, 14);         -- S + tide - 256
            if v_vr < 0 then q13_v <= (others => '0'); q13_hi <= '0';
            elsif v_vr > 1023 then q13_v <= (others => '1'); q13_hi <= '1';
            else q13_v <= unsigned(v_vr(9 downto 0)); q13_hi <= '1'; end if;
            v_x := resize(q12_sk, 14) - 470;
            q13_ki <= unsigned(v_x(5 downto 0)) & "00";
            -- 14: Rings product (fold frequency) as two partial products; iris triangle
            q14_pl <= q13_v(4 downto 0) * c_fr7;
            q14_ph <= q13_v(9 downto 5) * c_fr7;
            q14_hi <= q13_hi;
            q14_sa <= q13_sa;
            if q13_ki(7) = '1' then q14_ti <= q13_ki - 128; else q14_ti <= 128 - q13_ki; end if;
            -- 15: combine
            q15_pr <= resize(q14_pl, 19) + shift_left(resize(q14_ph, 19), 5);
            q15_hi <= q14_hi; q15_sa <= q14_sa; q15_ti <= q14_ti;
            -- 16: w2
            if q15_hi = '1' then q16_w2 <= 256 + signed(resize(shift_right(q15_pr, 7), 14));
            else q16_w2 <= resize(q15_sa, 14); end if;
            q16_ti <= q15_ti;
            -- 17: fold parts
            if q16_w2 < 110 then q17_lo <= '1'; else q17_lo <= '0'; end if;
            if q16_w2 < 256 then q17_mid <= '1'; else q17_mid <= '0'; end if;
            v_w := q16_w2 - 110;
            q17_r <= unsigned(v_w(7 downto 0));
            v_w := q16_w2 - 256;
            v_ph := unsigned(v_w(7 downto 0));
            if v_ph(7) = '1' then q17_tr <= v_ph - 128; else q17_tr <= 128 - v_ph; end if;
            q17_ti <= q16_ti;
            -- 18: palette position p (iris: pi)
            if q17_lo = '1' then
                q18_p <= (others => '0');
            elsif q17_mid = '1' then
                v_r := shift_right(shift_left(resize(q17_r, 11), 3) - resize(q17_r, 11), 2);
                if v_r > 255 then q18_p <= (others => '1'); else q18_p <= v_r(7 downto 0); end if;
            else
                v_r := shift_right(shift_left(resize(q17_tr, 11), 3) - resize(q17_tr, 11), 2) + 31;
                if v_r > 255 then q18_p <= (others => '1'); else q18_p <= v_r(7 downto 0); end if;
            end if;
            v_r := shift_right(shift_left(resize(q17_ti, 11), 3) - resize(q17_ti, 11), 2) + 31;
            if v_r > 255 then q18_pi <= (others => '1'); else q18_pi <= v_r(7 downto 0); end if;
            -- 19: palette address; green distance
            if s_iris = '1' and s_key_sr(4) = '1' then r_pl_a <= s_pal & q18_pi;
            else r_pl_a <= s_pal & q18_p; end if;
            v_dg := abs(signed(resize(q18_p, 10)) - 111);
            q19_dg <= unsigned(v_dg(7 downto 0));
            -- 20: palette data (EBR); green band tests
            if q19_dg < c_wg then q20_full <= '1'; else q20_full <= '0'; end if;
            if q19_dg < c_wg3 then q20_half <= '1'; else q20_half <= '0'; end if;
            -- 21: palette fabric copy (in the compose); delay flags
            q21_full <= q20_full; q21_half <= q20_half;
            q21_pl <= r_pl_q;
            -- 22: compose: palette, green band, key
            v_y := unsigned(q21_pl(23 downto 16)) & "00";
            v_u := unsigned(q21_pl(15 downto 8)) & "00";
            v_v := unsigned(q21_pl(7 downto 0)) & "00";
            if s_pal = '1' then v_kc := C_GR_SPEC(1); else v_kc := C_GR_SPEC(0); end if;
            v_gy := unsigned(v_kc(23 downto 16)) & "00"; v_gu := unsigned(v_kc(15 downto 8)) & "00"; v_gv := unsigned(v_kc(7 downto 0)) & "00";
            if s_key_sr(7) = '1' then
                if s_iris = '0' then
                    if s_pal = '1' then v_kc := C_GR_SPEC(3); else v_kc := C_GR_SPEC(2); end if;
                    q22_y <= unsigned(v_kc(23 downto 16)) & "00"; q22_u <= unsigned(v_kc(15 downto 8)) & "00"; q22_v <= unsigned(v_kc(7 downto 0)) & "00";
                else
                    q22_y <= v_y; q22_u <= v_u; q22_v <= v_v;
                end if;
            elsif q21_full = '1' then
                q22_y <= v_gy; q22_u <= v_gu; q22_v <= v_gv;
            elsif q21_half = '1' then
                v_sy := resize(v_y, 11) + v_gy; q22_y <= v_sy(10 downto 1);
                v_sy := resize(v_u, 11) + v_gu; q22_u <= v_sy(10 downto 1);
                v_sy := resize(v_v, 11) + v_gv; q22_v <= v_sy(10 downto 1);
            else
                q22_y <= v_y; q22_u <= v_u; q22_v <= v_v;
            end if;
            -- 23: seam line (3/4 toward black / neutral); black outside the generated avid
            if s_gav_sr(20) = '0' then
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
            elsif s_sl_sr(17) = '1' then
                v_s12 := resize(q22_y - 64, 12);
                s_y23 <= 64 + resize(shift_right(shift_left(v_s12, 1) + v_s12, 2), 10);
                v_c := signed(resize(q22_u, 12)) - 512;
                v_c := shift_right(shift_left(v_c, 1) + v_c, 2) + 512;
                s_u23 <= unsigned(v_c(9 downto 0));
                v_c := signed(resize(q22_v, 12)) - 512;
                v_c := shift_right(shift_left(v_c, 1) + v_c, 2) + 512;
                s_v23 <= unsigned(v_c(9 downto 0));
            else
                s_y23 <= q22_y; s_u23 <= q22_u; s_v23 <= q22_v;
            end if;
            -- flag delays: key from stage 13 (s_key_sr(0) = stage 14), seam line from stage 4 (s_sl_sr(0) = stage 5)
            s_key_sr <= s_key_sr(8 downto 0) & q13_key;
            s_sl_sr  <= s_sl_sr(16 downto 0) & q4_sl;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(20);
        end if;
    end process p_pix;

    p_obox : process(clk)
    begin
        if rising_edge(clk) then
            o_wp <= o_wp + 1;
            ob_a(to_integer(o_wp)) <= "00" & std_logic_vector(s_y23) & std_logic_vector(s_u23) & std_logic_vector(s_v23);
            ob_b(to_integer(o_wp)) <= "00" & std_logic_vector(s_y25) & std_logic_vector(s_u25) & std_logic_vector(s_v25);
            oa_q <= ob_a(to_integer(o_wp - 6));
            ob_q <= ob_b(to_integer(o_wp - 6));
            oa_f <= oa_q;
            ob_f <= ob_q;
        end if;
    end process p_obox;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the HD HDMI bar fix), luma 13/16 and the
    -- blanking gate.  The running sums snap back to black once 8 blank samples
    -- are in (the family-wide resync).
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- box A: stage 24 sums, stage 25 output
            if s_bah = "00000000" then
                s_bay <= to_unsigned(512, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y23 - unsigned(oa_f(29 downto 20));
                s_bau <= s_bau + s_u23 - unsigned(oa_f(19 downto 10));
                s_bav <= s_bav + s_v23 - unsigned(oa_f(9 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(22) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y25 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u25 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v25 <= v_sc(13 downto 4);
            else
                s_y25 <= to_unsigned(64, 10); s_u25 <= to_unsigned(512, 10); s_v25 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(22);
            -- box B: stage 26 sums, stage 27 output
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(512, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y25 - unsigned(ob_f(29 downto 20));
                s_bbu <= s_bbu + s_u25 - unsigned(ob_f(19 downto 10));
                s_bbv <= s_bbv + s_v25 - unsigned(ob_f(9 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y27 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u27 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v27 <= v_sc(13 downto 4);
            -- 28: luma 13/16
            if s_y27 > 64 then v_dl := s_y27 - 64; else v_dl := (others => '0'); end if;
            s_y28 <= s_y27 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u28 <= s_u27; s_v28 <= s_v27;
            -- 29: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y29 <= s_y28; s_u29 <= s_u28; s_v29 <= s_v28;
            else
                s_y29 <= to_unsigned(64, 10); s_u29 <= to_unsigned(512, 10); s_v29 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y29);
    data_out.u       <= std_logic_vector(s_u29);
    data_out.v       <= std_logic_vector(s_v29);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture grotto;
