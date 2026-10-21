-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Ricochet v0.5.0 -- Heirloom #30, rebuilt from the photo WIN_20230515_23_08_59.
--
-- The photo: two systems of nested chevrons pointing right (cores on lines
-- 256 / 779 of 1080), red teardrop cores at the left, bands running orange ->
-- salmon -> pink -> lavender -> periwinkle -> cyan, growing cyan toward the
-- vertex at x ~ 1600, then a flat orange ground; around the seams the chevron
-- arms kink into fine near-horizontal stripe bands that run to the right edge;
-- a red flyback strip at the bottom right.
--
-- The patch, as fitted (ricochet.py has every constant):
--   phiA = knee(F0 + F1 x - F2 x^2) + Y (gA0 - gA1 x)   horizontal ramp + vertical triangle
--   phiB = gB0 - gB1 x + c (AMP - Y)                    the triangle inverted, 4x steeper: the seams
--   phi  = min(phiA, phiB) + Etch * luma
--   colour = palette(clip(max(min(band, wall, cut), floor)))  with
--     band  = base(phi) - a0 cos 2pi(Nb phi / Ddef + chirp - t)   (8.3 bands, flowing)
--     wall  = the red core (bands emerge from under it), floor = its red -> orange shoulder,
--     cut   = the reach: past it the flat orange ground.
--
--   K1  Chevrons  chevron systems per frame height (1 .. 6, 2.07 = the photo)
--   K2  Core      size of the red teardrop cores
--   K3  Bands     bands per chevron (2 .. 24, 8.3 = the photo)
--   K4  Seams     seam stripe bands: thin and dense .. wide
--   K5  Speed     bipolar band flow (centre +-8 still; right = rings stream out of the cores)
--   K6  Etch      how far the picture pushes the bands
--   S7  Point     Right / Left
--   S8  Twin      chevrons fired from both sides, meeting in a bowtie of ground
--   S9  Ground    Flat / Echo (the chevron block repeats past the reach)
--   S10 Palette   Photo / Azure (chroma mirrored)
--   S11 Source    Video / Osc
--   P12 REACH     where the chevrons stop and the flat ground begins (0 = cores alone,
--                 100 % = no ground, chevrons and seams over the whole frame)
--
-- Engine: a vblank-zone sequencer (one serial multiplier, one serial divider,
-- fixed slots, a shared glide; one small operation per zone clock) sets the
-- per-field constants; a per-line engine (one serial multiplier, kicked at each
-- line start, computes the NEXT line) sets the DDA starts; the beam runs the F
-- ramp (2nd order), the Y gain and the seam field (the last two 2 / 3 pixels
-- behind, so they meet the pipe at the stage that uses them); Twin holds and
-- reverses them at the centre.  Multiplies are split into 3-bit row partials.
-- The side values {base, limit, shoulder} ride a block-RAM ring.  Generated
-- avid (parity-locked); output conditioning (chroma 9-tap, luma [1 2 1], luma
-- 13/16) before the blanking gate.  Latency 28 clocks in every mode.  Colours
-- in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.ricochet_pkg.all;
use work.all;

architecture ricochet of program_top is
    constant C_LAT : integer := 28;   -- = processing_delay_clks

    -- fitted constants (hw units; see ricochet.py)
    constant C_FF    : integer := -549;
    constant C_PHTOP : integer := 11027;
    constant C_YFB   : integer := 264192;    -- virtual line 1032, Q8

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
    -- vertical phase constant: ys * 2^20 / (1080 * 256)
    function f_vk(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(4316, 13);
            when 4          => return to_unsigned(2184, 13);
            when 8          => return to_unsigned(3641, 13);
            when 12         => return to_unsigned(1820, 13);
            when 5 | 6 | 14 => return to_unsigned(1456, 13);
            when 1 | 2 | 10 => return to_unsigned(1942, 13);
            when others     => return to_unsigned(971, 13);
        end case;
    end function;
    -- width class: 0 = 1920, 1 = 1280, 2 = 720
    function f_cls(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(2, 2);
            when 5 | 6 | 14     => return to_unsigned(1, 2);
            when others         => return to_unsigned(0, 2);
        end case;
    end function;
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0, of the line being drawn
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- oscillator source (Source = Osc)
    signal s_ox : unsigned(9 downto 0) := (others => '0');
    signal s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_oty : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_vk   : unsigned(12 downto 0) := to_unsigned(971, 13);
    signal s_clsr : unsigned(1 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(8 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_half : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_half2, s_half3 : unsigned(10 downto 0) := to_unsigned(962, 11);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(473 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(353 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(549 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(200 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(490 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');                 -- shared glide: selected knob
    signal s_gd : signed(14 downto 0) := (others => '0');                   -- and its step
    signal s_k5 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(9 downto 0) := (others => '0');                  -- |speed| - 8
    signal s_vneg, s_vstill : std_logic := '1';
    signal s_left, s_twin, s_echo, s_azure, s_src : std_logic := '0';
    signal s_cls  : unsigned(1 downto 0) := (others => '0');

    -- per-field constants
    signal s_tmp   : unsigned(26 downto 0) := (others => '0');              -- scratch for split shift-adds
    signal s_vstep : unsigned(19 downto 0) := to_unsigned(2006, 20);
    signal s_dyfb  : unsigned(15 downto 0) := to_unsigned(301, 16);       -- Q4 hw per line
    signal s_p0    : unsigned(12 downto 0) := to_unsigned(1608, 13);
    signal s_d     : unsigned(14 downto 0) := to_unsigned(6228, 15);
    signal s_pe    : unsigned(15 downto 0) := to_unsigned(7877, 16);
    signal s_rq    : unsigned(10 downto 0) := to_unsigned(336, 11);        -- 2^21 / D
    signal s_wq    : unsigned(11 downto 0) := to_unsigned(1402, 12);
    signal s_nbi   : unsigned(4 downto 0) := to_unsigned(8, 5);            -- bands, integer part
    signal s_cq8   : unsigned(11 downto 0) := to_unsigned(1016, 12);
    signal s_eq    : unsigned(9 downto 0) := to_unsigned(26, 10);
    signal s_spd   : signed(11 downto 0) := (others => '0');
    signal s_t     : unsigned(15 downto 0) := (others => '0');               -- band phase, Q14 cycles
    signal s_tq    : unsigned(9 downto 0) := (others => '0');
    -- DDA constants for this field (class, mode)
    signal c_fst  : signed(29 downto 0) := (others => '0');
    signal c_fd0  : signed(27 downto 0) := (others => '0');
    signal c_dd   : signed(16 downto 0) := (others => '0');
    signal c_gast : unsigned(12 downto 0) := (others => '0');
    signal c_gain : signed(12 downto 0) := (others => '0');
    signal c_gbst : unsigned(12 downto 0) := (others => '0');
    signal c_gbin : signed(16 downto 0) := (others => '0');

    -- field serial multiplier / divider
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal d_rem : unsigned(15 downto 0) := (others => '0');
    signal d_q   : unsigned(21 downto 0) := (others => '0');
    signal d_num : unsigned(21 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_lc   : unsigned(5 downto 0) := to_unsigned(63, 6);
    signal e_zk   : std_logic := '0';
    signal e_ph   : unsigned(19 downto 0) := to_unsigned(C_PHTOP, 20);
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_ylin : unsigned(12 downto 0) := (others => '0');
    signal e_fbt, e_fb, e_dir : std_logic := '0';
    signal e_yfb  : signed(17 downto 0) := (others => '0');                  -- Q4
    signal e_y    : signed(13 downto 0) := (others => '0');
    signal e_tt   : signed(14 downto 0) := (others => '0');
    signal e_ct   : signed(20 downto 0) := (others => '0');
    signal m_a    : unsigned(26 downto 0) := (others => '0');
    signal m_b    : unsigned(12 downto 0) := (others => '0');
    signal m_acc  : unsigned(26 downto 0) := (others => '0');
    signal m_neg  : std_logic := '0';
    signal n_yast : signed(29 downto 0) := (others => '0');
    signal n_yain : signed(17 downto 0) := (others => '0');
    signal n_bst  : signed(29 downto 0) := (others => '0');

    -- beam
    signal b_fl  : signed(29 downto 0) := (others => '0');
    signal b_fd  : signed(27 downto 0) := (others => '0');
    signal b_ya  : signed(29 downto 0) := (others => '0');
    signal b_yin : signed(17 downto 0) := (others => '0');
    signal b_bb  : signed(29 downto 0) := (others => '0');
    signal b_bin : signed(16 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal s_gs1, s_gs2, s_gs3 : std_logic := '0';                          -- g_start delayed

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal p1_fl : signed(17 downto 0) := (others => '0');
    signal p1_dk : signed(18 downto 0) := (others => '0');
    signal p2_f  : signed(17 downto 0) := (others => '0');
    signal p3_pa : signed(18 downto 0) := (others => '0');
    signal p3_lm : unsigned(7 downto 0) := (others => '0');
    signal p4_ph : signed(18 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(13 downto 0) := (others => '0');
    signal p5_ph : signed(18 downto 0) := (others => '0');
    signal p6_d, p6_g : signed(18 downto 0) := (others => '0');
    -- stage 7: 3-bit row partial products and clipped side terms
    signal p7_t0 : unsigned(19 downto 0) := (others => '0');
    signal p7_t1 : unsigned(16 downto 0) := (others => '0');
    signal p7_t2 : unsigned(13 downto 0) := (others => '0');
    signal p7_t3 : unsigned(10 downto 0) := (others => '0');
    signal p7_e0, p7_e1, p7_e2 : unsigned(16 downto 0) := (others => '0');
    signal p7_e3 : unsigned(15 downto 0) := (others => '0');
    signal p7_dpos : std_logic := '0';
    signal p7_wqs  : signed(18 downto 0) := (others => '0');
    signal p7_dc   : signed(6 downto 0) := (others => '0');
    signal p7_ng   : unsigned(8 downto 0) := (others => '0');
    signal p7_gc   : unsigned(11 downto 0) := (others => '0');
    signal p7_gneg : std_logic := '0';
    signal p7_fx   : unsigned(9 downto 0) := (others => '0');
    signal p7_flon : std_logic := '0';
    -- stage 8
    signal p8_ta   : unsigned(19 downto 0) := (others => '0');
    signal p8_tb   : unsigned(13 downto 0) := (others => '0');
    signal p8_ea   : unsigned(19 downto 0) := (others => '0');
    signal p8_eb   : unsigned(18 downto 0) := (others => '0');
    signal p8_dpos : std_logic := '0';
    signal p8_wqn  : unsigned(11 downto 0) := (others => '0');
    signal p8_wl   : signed(11 downto 0) := (others => '0');
    signal p8_cn   : unsigned(10 downto 0) := (others => '0');
    signal p8_cp   : unsigned(7 downto 0) := (others => '0');
    signal p8_gneg : std_logic := '0';
    signal p8_fr   : unsigned(8 downto 0) := (others => '0');
    -- stage 9 ..
    signal p9_thm  : unsigned(9 downto 0) := (others => '0');
    signal p9_wq   : unsigned(11 downto 0) := (others => '0');
    signal p9_l    : unsigned(9 downto 0) := (others => '0');
    signal p9_f    : unsigned(8 downto 0) := (others => '0');
    signal p10_c   : signed(11 downto 0) := (others => '0');
    signal p10_ba, p10_bb : unsigned(9 downto 0) := (others => '0');
    signal p10_l, p11_l : unsigned(9 downto 0) := (others => '0');
    signal p10_f, p11_f : unsigned(8 downto 0) := (others => '0');
    signal p10_thm, p11_thm : unsigned(9 downto 0) := (others => '0');
    signal p11_c   : signed(11 downto 0) := (others => '0');
    signal p11_base : unsigned(9 downto 0) := (others => '0');
    signal p12_tt  : unsigned(9 downto 0) := (others => '0');
    signal p12_ca  : signed(14 downto 0) := (others => '0');
    signal p12_cb  : signed(13 downto 0) := (others => '0');
    signal p13_th  : unsigned(9 downto 0) := (others => '0');
    signal p14_cv  : signed(8 downto 0) := (others => '0');
    signal p15_cv  : signed(8 downto 0) := (others => '0');
    signal p16_sb, p17_s : signed(11 downto 0) := (others => '0');
    signal p17_f   : unsigned(8 downto 0) := (others => '0');
    signal p18_idx : unsigned(7 downto 0) := (others => '0');
    signal p19_q   : std_logic_vector(31 downto 0) := (others => '0');
    signal p20_q   : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- side values {base, limit, shoulder} ride a block-RAM ring: written at
    -- stage 12, read 1 address back, out of the RAM at 15, fabric copy 16
    type t_ring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal ring : t_ring := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of ring : signal is true;
    signal r_wa, r_ra : unsigned(7 downto 0) := (others => '0');
    signal r_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal r_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_q16 : std_logic_vector(31 downto 0) := (others => '0');

    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y22, s_y23, s_y24, s_y25, s_y26, s_y27x, s_yf27 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u27, s_v27 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u28, s_v28 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept)
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer.  Every zone-clock step is one
    -- small operation (the zone is quasi-static but timed at the pixel clock).
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_r  : unsigned(16 downto 0);
        variable v_dr : unsigned(16 downto 0);
        variable v_i  : integer range 0 to 8;
        variable v_m  : unsigned(9 downto 0);
        variable v_st : signed(12 downto 0);
        variable v_k  : integer range 0 to 63;
    begin
        if rising_edge(clk) then
            s_tid  <= registers_in(8)(3 downto 0);
            s_ys   <= f_vstep_y(s_tid);
            s_vk   <= f_vk(s_tid);
            s_clsr <= f_cls(s_tid);

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
            elsif s_zc /= 511 then
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
            s_k5 <= unsigned(registers_in(4));
            -- speed: |v| - 8 and the still zone, registered free-running
            v_s := signed(resize(s_k5, 11)) - 512;
            if v_s > -9 and v_s < 9 then s_vstill <= '1'; else s_vstill <= '0'; end if;
            s_vneg <= v_s(10);
            if v_s(10) = '1' then v_m := unsigned(not v_s(9 downto 0)) - 7; else v_m := unsigned(v_s(9 downto 0)) - 8; end if;
            s_vm <= v_m;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_left  <= registers_in(6)(0);
                s_twin  <= registers_in(6)(1);
                s_echo  <= registers_in(6)(2);
                s_azure <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_cls   <= s_clsr;
                s_t <= unsigned(signed(s_t) + resize(s_spd, 16));
            end if;
            s_tq <= s_t(13 downto 4);
            s_half <= s_w(11 downto 1);
            s_half2 <= s_half + 2;
            s_half3 <= s_half + 3;
            g_wm1 <= s_w - 1;

            -- shared glide (1/8 of the gap per field, +-2 deadband): knob k on
            -- zone clocks 3k+1 (select, distance), 3k+2 (step), 3k+3 (update)
            if s_zone = '1' and s_zc >= 1 and s_zc <= 18 then
                case to_integer(s_zc) is
                    when 1  => s_gm <= s_g1; s_gd <= signed(resize(unsigned(registers_in(0)) & "000", 15)) - signed(resize(s_g1, 15));
                    when 4  => s_gm <= s_g2; s_gd <= signed(resize(unsigned(registers_in(1)) & "000", 15)) - signed(resize(s_g2, 15));
                    when 7  => s_gm <= s_g3; s_gd <= signed(resize(unsigned(registers_in(2)) & "000", 15)) - signed(resize(s_g3, 15));
                    when 10 => s_gm <= s_g4; s_gd <= signed(resize(unsigned(registers_in(3)) & "000", 15)) - signed(resize(s_g4, 15));
                    when 13 => s_gm <= s_g6; s_gd <= signed(resize(unsigned(registers_in(5)) & "000", 15)) - signed(resize(s_g6, 15));
                    when 16 => s_gm <= s_gp; s_gd <= signed(resize(unsigned(registers_in(7)) & "000", 15)) - signed(resize(s_gp, 15));
                    when 2 | 5 | 8 | 11 | 14 | 17 =>
                        if s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                    when 3  => s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 6  => s_g2 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 9  => s_g3 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 12 => s_g4 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when 15 => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                    when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                end case;
            end if;

            -- DDA constants for (class, mode)
            if s_zc = 2 then
                if s_twin = '1' then v_i := 2; elsif s_left = '1' then v_i := 1; else v_i := 0; end if;
                v_i := v_i + 3 * to_integer(s_cls);
                c_fst  <= to_signed(C_DDA_FST(v_i), 30);
                c_fd0  <= to_signed(C_DDA_FD0(v_i), 28);
                c_dd   <= to_signed(C_DDA_DD(v_i), 17);
                c_gast <= to_unsigned(C_DDA_GAST(v_i), 13);
                c_gain <= to_signed(C_DDA_GAIN(v_i), 13);
                c_gbst <= to_unsigned(C_DDA_GBST(v_i), 13);
                c_gbin <= to_signed(C_DDA_GBIN(v_i), 17);
            end if;

            -- serial products: one slot every 24 zone clocks (k = 1, 4, 7 ..
            -- 34): load at 8k, 16 iterations, result stable from 8k + 17.  The
            -- scale slots take the square left in f_acc (or s_tmp).
            v_k := to_integer(s_zc(8 downto 3));
            if s_zc(2 downto 0) = "000" and (v_k = 1 or v_k = 4 or v_k = 7 or v_k = 10 or v_k = 13 or v_k = 16 or
               v_k = 19 or v_k = 22 or v_k = 25 or v_k = 28 or v_k = 31 or v_k = 34) then
                f_acc <= (others => '0');
                case v_k is
                    when 1  => f_a <= resize(s_g1(12 downto 3), 36); f_b <= resize(s_g1(12 downto 3), 16);
                    when 4  => f_a <= resize(s_tmp(14 downto 0), 36); f_b <= resize(s_vk, 16);
                    when 7  => f_a <= to_unsigned(3759, 36);         f_b <= resize(s_g2(12 downto 3), 16);
                    when 10 => f_a <= resize(s_gp(12 downto 3), 36); f_b <= resize(s_gp(12 downto 3), 16);
                    when 16 => f_a <= resize(s_g3(12 downto 3), 36); f_b <= resize(s_g3(12 downto 3), 16);
                    when 22 => f_a <= resize(1023 - s_g4(12 downto 3), 36); f_b <= resize(1023 - s_g4(12 downto 3), 16);
                    when 28 => f_a <= resize(s_g6(12 downto 3), 36); f_b <= resize(s_g6(12 downto 3), 16);
                    when 34 => f_a <= resize(s_vm, 36);              f_b <= resize(s_vm, 16);
                    when 13 => f_a <= resize(f_acc(19 downto 0), 36); f_b <= to_unsigned(20363, 16);
                    when 19 => f_a <= resize(f_acc(19 downto 0), 36); f_b <= to_unsigned(3707, 16);
                    when 25 => f_a <= resize(f_acc(19 downto 0), 36); f_b <= to_unsigned(3311, 16);
                    when others => f_a <= resize(f_acc(19 downto 0), 36); f_b <= to_unsigned(687, 16);
                end case;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;
            -- stores and split shift-adds (one add per zone clock)
            case to_integer(s_zc) is
                -- cycles = 4096 + 5 k1^2 / 256 (Q12), for slot 4
                when 26  => s_tmp <= resize(f_acc(19 downto 0), 27) + shift_left(resize(f_acc(19 downto 0), 27), 2);
                when 27  => s_tmp <= resize(to_unsigned(4096, 15) + s_tmp(22 downto 8), 27);
                when 50  => s_vstep <= f_acc(31 downto 12);
                -- flyback step = vstep * 77 >> 9 (Q4)
                when 51  => s_tmp <= shift_left(resize(s_vstep, 27), 6) + shift_left(resize(s_vstep, 27), 3);
                when 52  => s_tmp <= s_tmp + shift_left(resize(s_vstep, 27), 2);
                when 53  => s_tmp <= s_tmp + resize(s_vstep, 27);
                when 54  => s_dyfb <= resize(s_tmp(26 downto 9), 16);
                when 74  => s_p0 <= to_unsigned(313, 13) + resize(f_acc(21 downto 10), 13);
                when 122 => s_d <= to_unsigned(1566, 15) + resize(f_acc(35 downto 20), 15);
                when 123 => s_pe <= resize(s_p0, 16) + resize(s_d, 16);
                when 124 => s_pe <= s_pe + 41;
                -- bands, integer part = (32 + 11 k3^2 >> 15) >> 4
                when 146 => s_tmp <= shift_left(resize(f_acc(19 downto 0), 27), 3) + shift_left(resize(f_acc(19 downto 0), 27), 1);
                when 147 => s_tmp <= s_tmp + resize(f_acc(19 downto 0), 27);
                when 148 => s_tmp <= resize(to_unsigned(32, 9) + s_tmp(23 downto 15), 27);
                when 149 => s_nbi <= s_tmp(8 downto 4);
                when 170 => s_wq <= to_unsigned(337, 12) + resize(f_acc(31 downto 20), 12);
                when 218 => s_cq8 <= to_unsigned(192, 12) + resize(f_acc(31 downto 20), 12);
                when 266 => s_eq <= resize(f_acc(29 downto 20), 10);
                -- speed = sign (|v| - 8)^2 >> 7, or still
                when 290 =>
                    if s_vstill = '1' then
                        s_spd <= (others => '0');
                    else
                        v_st := signed(resize(f_acc(17 downto 7), 13));
                        if s_vneg = '1' then s_spd <= resize(-v_st, 12); else s_spd <= v_st(11 downto 0); end if;
                    end if;
                when others => null;
            end case;

            -- divider: Rq = 2^21 / D (restoring, zc 130 .. 152, one subtract
            -- per step)
            if s_zc = 130 then
                d_num <= '1' & to_unsigned(0, 21);
                d_rem <= (others => '0');
                d_q   <= (others => '0');
            elsif s_zc > 130 and s_zc <= 152 then
                v_r := d_rem & d_num(21);
                v_dr := v_r - resize(s_d, 17);
                if v_dr(16) = '0' then
                    d_rem <= v_dr(15 downto 0);
                    d_q <= d_q(20 downto 0) & '1';
                else
                    d_rem <= v_r(15 downto 0);
                    d_q <= d_q(20 downto 0) & '0';
                end if;
                d_num <= d_num(20 downto 0) & '0';
            elsif s_zc = 153 then
                s_rq <= d_q(10 downto 0);
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
    -- Source side: oscillator, line-buffer write of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then s_ox <= s_ox + 3; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 5; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zgo = '1' then s_ot <= s_ot + 3; end if;
            s_oty <= to_unsigned(64, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot) & '0', 10)
                     + resize(f_tri8(s_ox(9 downto 2) + s_ot), 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot), 10);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then v_l := s_oty(9 downto 2); else v_l := unsigned(data_in.y(9 downto 2)); end if;
            if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line's vertical triangle Y (with the bottom
    -- flyback) and the DDA starts on one signed serial multiplier
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_p  : signed(27 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 320 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_ph <= to_unsigned(C_PHTOP, 20);
                e_vy <= (others => '0');
                e_fb <= '0';
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_ph <= e_ph + s_vstep;
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
            elsif e_lc /= 63 then
                e_lc <= e_lc + 1;
            end if;

            -- multiplier (iterates every clock it is not loaded)
            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(25 downto 0) & '0';
            m_b <= '0' & m_b(12 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    -- |ph - 2^19| >> 7
                    if e_ph(19) = '1' then
                        e_ylin <= resize(e_ph(18 downto 7), 13);
                    else
                        e_ylin <= resize(shift_right(to_unsigned(524288, 20) - e_ph, 7), 13);
                    end if;
                    if e_vy >= C_YFB then e_fbt <= '1'; else e_fbt <= '0'; end if;
                when 2 =>
                    if e_fbt = '1' then
                        if e_fb = '0' then
                            e_fb  <= '1';
                            e_dir <= e_ph(19);
                            e_yfb <= signed(resize(e_ylin & "0000", 18));
                        elsif e_dir = '1' then
                            e_yfb <= e_yfb + signed(resize(s_dyfb, 18));
                        else
                            e_yfb <= e_yfb - signed(resize(s_dyfb, 18));
                        end if;
                    end if;
                when 3 =>
                    if e_fbt = '1' then e_y <= e_yfb(17 downto 4); else e_y <= signed(resize(e_ylin, 14)); end if;
                when 4 =>
                    e_tt <= to_signed(4096, 15) - resize(e_y, 15);
                    if e_y < 0 then m_a <= unsigned(resize(-e_y, 27)); else m_a <= unsigned(resize(e_y, 27)); end if;
                    m_b <= c_gast; m_acc <= (others => '0');
                    m_neg <= e_y(13);
                when 19 =>
                    v_p := signed(resize(m_acc, 28));
                    if m_neg = '1' then v_p := -v_p; end if;
                    n_yast <= resize(v_p, 30);
                    if e_y < 0 then m_a <= unsigned(resize(-e_y, 27)); else m_a <= unsigned(resize(e_y, 27)); end if;
                    if c_gain < 0 then m_b <= unsigned(resize(-c_gain, 13)); else m_b <= unsigned(c_gain); end if;
                    m_acc <= (others => '0');
                    m_neg <= e_y(13) xor c_gain(12);
                when 34 =>
                    v_p := signed(resize(m_acc, 28));
                    if m_neg = '1' then v_p := -v_p; end if;
                    n_yain <= v_p(25 downto 8);
                    if e_tt < 0 then m_a <= unsigned(resize(-e_tt, 27)); else m_a <= unsigned(resize(e_tt, 27)); end if;
                    m_b <= '0' & s_cq8;
                    m_acc <= (others => '0');
                    m_neg <= e_tt(14);
                when 49 =>
                    v_p := signed(resize(m_acc, 28));
                    if m_neg = '1' then v_p := -v_p; end if;
                    e_ct <= resize(v_p(27 downto 8), 21);
                when 50 =>
                    n_bst <= shift_left(resize(signed(resize(c_gbst, 21)) + e_ct, 30), 12);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: F ramp (2nd-order DDA), Y-gain and seam-field DDAs.  Twin holds the
    -- centre value for one pixel and reverses the differences.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            s_gs1 <= g_start; s_gs2 <= s_gs1; s_gs3 <= s_gs2;
            -- F ramp: in step with the pixel entering stage 1
            if g_start = '1' then
                b_fl  <= c_fst;
                b_fd  <= c_fd0;
                b_px  <= (others => '0');
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                if s_twin = '1' and b_px = s_half then
                    b_fd <= resize(c_dd, 28) - b_fd;
                else
                    b_fl <= b_fl + resize(shift_right(b_fd, 8), 30);
                    b_fd <= b_fd + resize(c_dd, 28);
                end if;
            end if;
            -- Y gain: two pixels behind (meets the knee output at stage 3)
            if s_gs2 = '1' then
                b_ya  <= n_yast;
                b_yin <= n_yain;
            elsif s_gav_sr(1) = '1' then
                if s_twin = '1' and b_px = s_half2 then
                    b_yin <= -b_yin;
                else
                    b_ya <= b_ya + resize(b_yin, 30);
                end if;
            end if;
            -- seam field: three pixels behind (meets phiA at stage 4)
            if s_gs3 = '1' then
                b_bb  <= n_bst;
                b_bin <= c_gbin;
            elsif s_gav_sr(2) = '1' then
                if s_twin = '1' and b_px = s_half3 then
                    b_bin <= -b_bin;
                else
                    b_bb <= b_bb + resize(b_bin, 30);
                end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_k   : signed(18 downto 0);
        variable v_dq  : unsigned(13 downto 0);
        variable v_x   : signed(18 downto 0);
        variable v_n10 : unsigned(9 downto 0);
        variable v_p23 : signed(22 downto 0);
        variable v_t20 : unsigned(19 downto 0);
        variable v_w   : unsigned(11 downto 0);
        variable v_wl  : unsigned(9 downto 0);
        variable v_ct  : unsigned(9 downto 0);
        variable v_cs  : signed(17 downto 0);
        variable v_s   : signed(11 downto 0);
        variable v_e   : unsigned(17 downto 0);
        variable v_es  : unsigned(24 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: integer part of the ramp; knee distance
            p1_fl <= b_fl(29 downto 12);
            p1_dk <= resize(b_fl(29 downto 12), 19) - C_FF;

            -- 2: knee (quadratic smooth max, half-width 256; square from a table)
            if p1_dk >= 256 then
                p2_f <= p1_fl;
            elsif p1_dk < -256 then
                p2_f <= to_signed(C_FF, 18);
            else
                v_k := p1_dk + 256;
                p2_f <= to_signed(C_FF, 18) + signed(resize(C_RIC_KSQ(to_integer(unsigned(v_k(8 downto 2)))), 18));
            end if;

            -- 3: phiA (Y-gain DDA two pixels behind); luma
            p3_pa <= resize(p2_f, 19) + resize(b_ya(29 downto 12), 19);
            p3_lm <= unsigned(s_lb_q);

            -- 4: min with the seam field (three pixels behind); etch partials
            if p3_pa < resize(b_bb(29 downto 12), 19) then p4_ph <= p3_pa; else p4_ph <= resize(b_bb(29 downto 12), 19); end if;
            p4_ea <= p3_lm(3 downto 0) * s_eq;
            p4_eb <= p3_lm(7 downto 4) * s_eq;

            -- 5: etched field
            v_e := resize(p4_ea, 18) + shift_left(resize(p4_eb, 18), 4);
            p5_ph <= p4_ph + signed(resize(v_e(17 downto 4), 19));

            -- 6: distances from the core and from the reach
            p6_d <= p5_ph - signed(resize(s_p0, 19));
            p6_g <= p5_ph - signed(resize(s_pe, 19));

            -- 7: band phase and echo partial products (3-bit rows; only the
            --    product bits that land below 2^20 / 2^18 are kept), base
            --    position sum, clipped wall / cut / shoulder terms
            v_p23 := p6_d * signed(resize(s_wq(2 downto 0), 4));
            p7_t0 <= unsigned(v_p23(19 downto 0));
            v_p23 := p6_d * signed(resize(s_wq(5 downto 3), 4));
            p7_t1 <= unsigned(v_p23(16 downto 0));
            v_p23 := p6_d * signed(resize(s_wq(8 downto 6), 4));
            p7_t2 <= unsigned(v_p23(13 downto 0));
            v_p23 := p6_d * signed(resize(s_wq(11 downto 9), 4));
            p7_t3 <= unsigned(v_p23(10 downto 0));
            if p6_d > 0 then v_dq := unsigned(p6_d(16 downto 3)); p7_dpos <= '1'; else v_dq := (others => '0'); p7_dpos <= '0'; end if;
            p7_e0 <= v_dq * s_rq(2 downto 0);
            p7_e1 <= v_dq * s_rq(5 downto 3);
            p7_e2 <= v_dq * s_rq(8 downto 6);
            p7_e3 <= v_dq * s_rq(10 downto 9);
            p7_wqs <= shift_right(p6_d, 1) + shift_right(p6_d, 3) + shift_right(p6_d, 5) + shift_right(p6_d, 9);
            if p6_d < -16 then p7_dc <= to_signed(-16, 7);
            elsif p6_d > 63 then p7_dc <= to_signed(63, 7);
            else p7_dc <= p6_d(6 downto 0); end if;
            if p6_g < 0 then
                v_n10 := unsigned(-p6_g(9 downto 0));
                if p6_g < -511 then p7_ng <= (others => '1'); else p7_ng <= v_n10(8 downto 0); end if;
                p7_gneg <= '1';
            else
                if p6_g > 4095 then p7_gc <= (others => '1'); else p7_gc <= unsigned(p6_g(11 downto 0)); end if;
                p7_gneg <= '0';
            end if;
            v_x := p6_d + 423;
            if v_x < 0 then p7_fx <= (others => '0'); elsif v_x > 1023 then p7_fx <= (others => '1'); else p7_fx <= unsigned(v_x(9 downto 0)); end if;
            if p6_d < 187 then p7_flon <= '1'; else p7_flon <= '0'; end if;

            -- 8: partial sums; normal base position; raw wall / cut / shoulder
            p8_ta <= p7_t0 + shift_left(resize(p7_t1, 20), 3);
            p8_tb <= p7_t2 + shift_left(resize(p7_t3, 14), 3);
            p8_ea <= resize(p7_e0, 20) + shift_left(resize(p7_e1, 20), 3);
            p8_eb <= resize(p7_e2, 19) + shift_left(resize(p7_e3, 19), 3);
            p8_dpos <= p7_dpos;
            if p7_wqs < 0 then p8_wqn <= (others => '0');
            elsif p7_wqs > 4095 then p8_wqn <= (others => '1');
            else p8_wqn <= unsigned(p7_wqs(11 downto 0)); end if;
            p8_wl <= to_signed(172, 12) + shift_left(resize(p7_dc, 12), 4) + resize(p7_dc, 12) + resize(shift_right(p7_dc, 4), 12);
            p8_cn <= to_unsigned(210, 11) + shift_left(resize(p7_ng, 11), 1) - resize(shift_right(p7_ng, 3), 11) + resize(shift_right(p7_ng, 7), 11);
            p8_cp <= to_unsigned(210, 8) - resize(shift_right(p7_gc, 5), 8);
            p8_gneg <= p7_gneg;
            if p7_flon = '1' then
                p8_fr <= to_unsigned(61, 9) + resize(shift_right(p7_fx, 2), 9) + resize(shift_right(p7_fx, 4), 9) + resize(shift_right(p7_fx, 6), 9);
            else
                p8_fr <= (others => '0');
            end if;

            -- 9: band phase; base position (echo or normal); limit; shoulder
            v_t20 := p8_ta + shift_left(resize(p8_tb, 20), 6);
            p9_thm <= v_t20(19 downto 10);
            if s_echo = '1' then
                v_es := resize(p8_ea, 25) + shift_left(resize(p8_eb, 25), 6);
                if p8_dpos = '1' then v_w := v_es(17 downto 6); else v_w := (others => '0'); end if;
            else
                v_w := p8_wqn;
            end if;
            p9_wq <= v_w;
            if p8_wl < 0 then v_wl := (others => '0');
            elsif p8_wl > 1023 then v_wl := (others => '1');
            else v_wl := unsigned(p8_wl(9 downto 0)); end if;
            if p8_gneg = '1' then
                if p8_cn > 1023 then v_ct := (others => '1'); else v_ct := p8_cn(9 downto 0); end if;
            else
                if p8_cp < 143 then v_ct := to_unsigned(143, 10); else v_ct := resize(p8_cp, 10); end if;
            end if;
            if s_echo = '0' and v_ct < v_wl then p9_l <= v_ct; else p9_l <= v_wl; end if;
            p9_f <= p8_fr;

            -- 10: chirp ROM; base partial sums
            p10_c <= C_RIC_CORR(to_integer(p9_wq(11 downto 3)));
            p10_ba <= resize(p9_wq(11 downto 3), 10) + resize(p9_wq(11 downto 6), 10);
            p10_bb <= resize(p9_wq(11 downto 7), 10) + resize(p9_wq(11 downto 10), 10);
            p10_l <= p9_l; p10_f <= p9_f; p10_thm <= p9_thm;
            -- 11: fabric copy; base
            p11_c <= p10_c;
            p11_base <= to_unsigned(326, 10) + p10_ba + p10_bb;
            p11_l <= p10_l; p11_f <= p10_f; p11_thm <= p10_thm;
            -- 12: band phase minus time; chirp x bands (integer part) partials;
            --     side values into the ring
            p12_tt <= p11_thm - s_tq;
            p12_ca <= resize(signed('0' & s_nbi(2 downto 0)) * p11_c, 15);
            p12_cb <= resize(signed('0' & s_nbi(4 downto 3)) * p11_c, 14);
            r_wd <= "000" & std_logic_vector(p11_base) & std_logic_vector(p11_l) & std_logic_vector(p11_f);
            -- 13: band phase (the cosine ROM address)
            v_cs := resize(p12_ca, 18) + shift_left(resize(p12_cb, 18), 3);
            v_cs := shift_right(v_cs, 4);
            p13_th <= p12_tt + unsigned(v_cs(9 downto 0));
            -- 14: full-wave cosine ROM; 15: fabric copy; ring fabric copy at 16
            p14_cv <= C_RIC_COS(to_integer(p13_th));
            p15_cv <= p14_cv;
            r_q16 <= r_q;
            -- 16: band
            p16_sb <= signed(resize(unsigned(r_q(28 downto 19)), 12)) - resize(p15_cv, 12);
            -- 17: under the core wall / the reach cut
            if signed(resize(unsigned(r_q16(18 downto 9)), 12)) < p16_sb then p17_s <= signed(resize(unsigned(r_q16(18 downto 9)), 12)); else p17_s <= p16_sb; end if;
            p17_f <= unsigned(r_q16(8 downto 0));
            -- 18: over the red shoulder; clip; palette index
            v_s := p17_s;
            if signed(resize(p17_f, 12)) > v_s then v_s := signed(resize(p17_f, 12)); end if;
            if v_s < 61 then v_s := to_signed(61, 12); elsif v_s > 1023 then v_s := to_signed(1023, 12); end if;
            p18_idx <= unsigned(v_s(9 downto 2));
            -- 19: palette ROM; 20: fabric copy; 21: Azure chroma mirror
            p19_q <= C_RIC_PAL(to_integer(p18_idx));
            p20_q <= p19_q;
            s_y21 <= unsigned(p20_q(31 downto 22));
            if s_azure = '1' then
                s_u21 <= 1024 - unsigned(p20_q(21 downto 12));
                s_v21 <= 1024 - unsigned(p20_q(11 downto 2));
            else
                s_u21 <= unsigned(p20_q(21 downto 12));
                s_v21 <= unsigned(p20_q(11 downto 2));
            end if;
        end if;
    end process p_pix;

    -- side-value ring (one write, one registered read; the read address is 1
    -- behind the write address, so they never collide)
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            r_q <= ring(to_integer(r_ra));
            ring(to_integer(r_wa)) <= r_wd;
            r_wa <= r_wa + 1;
            r_ra <= r_wa - 1;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- Output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent
    -- 8-sums, luma [1 2 1]; centre at stage 26 -> 27), luma 13/16 and the
    -- blanking gate at stage 28
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_csu(0) <= s_u21; s_csv(0) <= s_v21;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u21 - s_csu(7);
            s_boxv <= s_boxv + s_v21 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u27 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v27 <= v_sc(13 downto 4);
            s_y22 <= s_y21; s_y23 <= s_y22; s_y24 <= s_y23; s_y25 <= s_y24;
            s_y26 <= s_y25; s_y27x <= s_y26;
            v_sy := resize(s_y25, 12) + shift_left(resize(s_y26, 12), 1) + resize(s_y27x, 12);
            s_yf27 <= v_sy(11 downto 2);
            if s_yf27 > 64 then v_dl := s_yf27 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(26) = '1' then
                s_y28 <= s_yf27 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u28 <= s_u27; s_v28 <= s_v27;
            else
                s_y28 <= to_unsigned(64, 10); s_u28 <= to_unsigned(512, 10); s_v28 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(s_u28);
    data_out.v       <= std_logic_vector(s_v28);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture ricochet;
