-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Tapers v0.1.0 -- Heirloom (video phase), from the clip WIN_20230203_00_19_56 at 6.0 s.
--
-- The clip: a row of candle-flame columns ("tapers") rising from the bottom, filled with horizontal
-- mosaic rows of hot pink / lavender / sky / teal / green, on a dark background of blue, maroon, black
-- and navy mosaic dashes shaped by the camera (a blue bar, a maroon bar, a plant, a pink/yellow strip,
-- a "lamp").  Above the tips the columns are flat rectangles; below, each 154-line band widens and steps
-- back.  The tips rise about 440 lines/s, fill the frame in ~2 s, and a new row appears at the bottom.
--
-- The patch, as fitted (tapers.py has the tables; .heirloom_work/rb_tapers/notes.md the measurements):
--   key     cos(2pi (x - 48) / 289.5) - (0.0017 |x - 1000| - 0.5)  >  T(y)         (virtual 1080-line px)
--   T(y)    0.75 sin(2pi (y - ys) / 1380) + Steps * (1/2 - frac((y - 185) / 154.3)), ys drifting up
--   inside  a vertical sine (period 206 lines) through the pink..green LUT, sampled per 22-line mosaic
--           row, scrolling up; the camera's level 2 shifts it a quarter cycle, 5 a half, 6-7 show through
--   outside the camera (44 x 22 mosaic, 8 luma levels): black, navy, dashed cycle, block cycle, maroon,
--           blue, pink, yellow; the cycle = two square waves of period 174 lines, scrolling up
--   edge    the key blends inside / outside over ~6 px (the analog softness)
--
--   K1  Tapers    count across the screen (2.3 .. 19; 6.63 = the clip)
--   K2  Bowl      how much the outer tapers are shortened (0 = all equal .. 3x the clip)
--   K3  Steps     the 7-per-frame sawtooth: 0 = smooth bells .. 358 = the clip .. 1023 = deep stair steps
--   K4  Rise      bipolar (deadband): the tips rise (right) or sink (left); the colour rows follow
--   K5  Hue cycle the interior colour cycle: long sweeps (0) .. 206 lines (512) .. fine stripes (1023)
--   K6  Mosaic    block size 9 x 5 .. 44 x 22 (512) .. 211 x 106 virtual px (camera and colour rows)
--   S7  Flip      tapers hang from the top and the motion mirrors
--   S8  Invert    swap the palettes: dark tapers on the striped field
--   S9  Palette   Neon (pink / green, the clip) / Amber (amber / cyan)
--   S10 Rows      colour rows sampled per mosaic row (the clip) / continuous gradients
--   S11 Source    Video / Osc
--   P12 HEIGHT    the threshold: 0 % = a row of small flames at the apex, 500 = the clip, 100 % = a
--                 solid striped wall with dark notches
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine (phase
-- accumulators for the threshold sine, the band sawtooth, the hue wave and the background cycle; the
-- sawtooth level on its own serial multiplier; the sine read from the shared ROM in hblank), per-pixel
-- accumulators for the carrier phase, the bowl and the mosaic columns, a 21-stage pixel path.  Virtual
-- 1080-line coordinates (SD shows the centre 4:3).  Generated avid; output conditioning (Y, U and V
-- through two [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16) before the blanking gate.  Colours in the
-- hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.tapers_pkg.all;
use work.all;

architecture tapers of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- fitted constants (phases: 2^24 per cycle)
    constant C_PH_ANCH : integer := 2516582;   -- carrier phase at virtual x = 960: 0.15 cycles (peaks at 48 + 289.5 k)
    constant C_PS_CLIP : integer := 16095827;     -- threshold sine phase at line 0 of the clip (-56 / 1380)
    constant C_PB_CLIP : integer := 13440318;     -- band sawtooth phase at line 0 (-185 / 154.3)
    constant C_PH_CLIP : integer := 15636169;     -- hue phase at line 0 (-220 / 206)
    constant C_PS_FLIP : integer := 13129000;     -- Flip offsets: 1080 / 1380, 1080 / 206, 1080 / 174 (frac)
    constant C_PH_FLIP : integer := 4076000;
    constant C_PG_FLIP : integer := 3472000;

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

    -- ROMs (one read port each, registered address and data); the sine ROM is shared by the pixel
    -- path (carrier, active video) and the line engine (threshold sine, one read in hblank)
    signal r_sn_a : unsigned(9 downto 0) := (others => '0');
    signal r_sn_q : signed(11 downto 0) := (others => '0');
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_pi_a, r_po_a : unsigned(7 downto 0) := (others => '0');
    signal r_pi_q, r_po_q : std_logic_vector(31 downto 0) := (others => '0');

    -- camera mosaic row memory: the level of each block column, written on the first line of a
    -- block row (block starts), read on the other lines.  Never read at the written address (the
    -- sampling line uses the x-hold instead).
    type t_cam is array (0 to 255) of std_logic_vector(3 downto 0);
    signal cam : t_cam := (others => x"0");
    attribute no_rw_check : boolean;
    attribute no_rw_check of cam : signal is true;
    signal cam_we : std_logic := '0';
    signal cam_wa, cam_ra : unsigned(7 downto 0) := (others => '0');
    signal cam_wd : std_logic_vector(3 downto 0) := x"0";
    signal cam_q  : std_logic_vector(3 downto 0) := x"0";

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs16 : unsigned(6 downto 0) := to_unsigned(32, 7);
    signal s_x0   : unsigned(9 downto 0) := to_unsigned(16, 10);
    signal s_wsp  : unsigned(11 downto 0) := to_unsigned(1920, 12);   -- 1920 - x0 (h)
    signal s_bsp  : unsigned(11 downto 0) := to_unsigned(2000, 12);   -- bowl centre - x0 (h)
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_hd1080 : std_logic := '1';

    -- knobs: glided (Q3), raw rise, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(341 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(358 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(500 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(776, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_flip, s_inv, s_pal, s_cont, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(10 downto 0) := (others => '0');
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_a    : unsigned(17 downto 0) := (others => '0');
    signal c_kx   : unsigned(16 downto 0) := to_unsigned(28967, 17);  -- carrier phase per h, Q4 of 2^20
    signal c_kxs  : unsigned(18 downto 0) := to_unsigned(57934, 19);  -- per pixel
    signal c_ph0  : unsigned(23 downto 0) := (others => '0');          -- carrier phase at the line start
    signal c_ds   : unsigned(11 downto 0) := to_unsigned(891, 12);    -- bowl slope per h, Q20
    signal c_dsx  : unsigned(13 downto 0) := to_unsigned(1782, 14);   -- per pixel
    signal c_d0   : signed(23 downto 0) := (others => '0');            -- bowl at the line start, Q20
    signal c_drift : unsigned(18 downto 0) := (others => '0');         -- sine drift per field (2^24 / cycle)
    signal c_khv  : unsigned(17 downto 0) := (others => '0');
    signal c_kh   : unsigned(19 downto 0) := to_unsigned(81429, 20);  -- hue phase per line
    signal c_kg   : unsigned(20 downto 0) := to_unsigned(96419, 21);  -- background phase per line
    signal c_ks   : unsigned(15 downto 0) := to_unsigned(12158, 16);  -- threshold sine phase per line
    signal c_kb   : unsigned(18 downto 0) := to_unsigned(108737, 19); -- band sawtooth phase per line
    signal c_bwh  : unsigned(13 downto 0) := to_unsigned(1421, 14);   -- block width, Q4 h
    signal c_bh   : unsigned(15 downto 0) := to_unsigned(5684, 16);   -- block height, Q8 virtual lines
    signal c_bh2  : unsigned(15 downto 0) := to_unsigned(2842, 16);   -- half
    signal c_asaw : unsigned(9 downto 0) := to_unsigned(358, 10);     -- Steps, Q10
    signal c_toff : signed(23 downto 0) := (others => '0');            -- Height offset, Q20
    signal c_ps0, c_phh0, c_pg0 : unsigned(23 downto 0) := (others => '0');   -- field-start phases
    signal c_ox0, c_ox20, c_oy0 : unsigned(15 downto 0) := (others => '0');   -- osc drifts
    signal c_oxs, c_ox2s : unsigned(8 downto 0) := to_unsigned(108, 9);
    signal c_oys  : unsigned(9 downto 0) := to_unsigned(73, 10);
    signal c_ky, c_ku, c_kv : unsigned(9 downto 0) := to_unsigned(512, 10);   -- palette black
    signal z_first : std_logic := '1';

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
    signal e_lc   : unsigned(5 downto 0) := to_unsigned(63, 6);
    signal e_zk   : std_logic := '0';
    signal e_ps, e_pb, e_ph, e_pg, e_pgq : unsigned(23 downto 0) := (others => '0');
    signal e_rc   : unsigned(15 downto 0) := (others => '0');
    signal e_oy   : unsigned(15 downto 0) := (others => '0');
    signal e_sin  : signed(11 downto 0) := (others => '0');
    signal m_a    : unsigned(39 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(39 downto 0) := (others => '0');
    signal n_tsaw, n_tsin, n_t : signed(23 downto 0) := (others => '0');
    signal n_hue  : unsigned(4 downto 0) := (others => '0');
    signal n_a, n_b, n_half, n_samp : std_logic := '0';
    signal l_t    : signed(23 downto 0) := (others => '0');
    signal l_hue  : unsigned(4 downto 0) := (others => '0');
    signal l_a, l_b, l_half, l_samp : std_logic := '0';
    signal y_cnt  : unsigned(5 downto 0) := (others => '0');
    signal y_sel  : std_logic := '0';

    -- beam (per-pixel accumulators)
    signal b_x    : unsigned(11 downto 0) := (others => '0');
    signal b_dir  : std_logic := '1';
    signal b_phi  : unsigned(23 downto 0) := (others => '0');
    signal b_d    : signed(23 downto 0) := (others => '0');
    signal b_bc   : unsigned(14 downto 0) := (others => '0');
    signal b_bi   : unsigned(7 downto 0) := (others => '0');
    signal b_bst  : std_logic := '0';
    signal b_hold : unsigned(2 downto 0) := (others => '0');
    signal b_ox, b_ox2 : unsigned(15 downto 0) := (others => '0');
    signal o_s    : unsigned(8 downto 0) := (others => '0');
    signal o_lv   : unsigned(2 downto 0) := (others => '0');
    signal s_lvin : unsigned(2 downto 0) := (others => '0');

    -- pixel path
    signal p2_d, p3_d : signed(23 downto 0) := (others => '0');
    signal p2_lv, p3_lv, p4_l, p5_l, p6_l : unsigned(2 downto 0) := (others => '0');
    signal p4_cos : signed(11 downto 0) := (others => '0');
    signal p4_d   : signed(23 downto 0) := (others => '0');
    signal p5_hd  : signed(24 downto 0) := (others => '0');
    signal p6_m   : signed(25 downto 0) := (others => '0');
    signal p7_w, p8_w, p9_w, p10_w, p11_w : unsigned(2 downto 0) := (others => '0');
    signal p7_over, p8_over, p9_over : std_logic := '0';
    signal p9_in, p9_out, p10_in, p10_out : std_logic_vector(31 downto 0) := (others => '0');
    signal p11_dy, p11_du, p11_dv : signed(10 downto 0) := (others => '0');
    signal p11_oy, p11_ou, p11_ov : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p12_py, p12_pu, p12_pv : signed(12 downto 0) := (others => '0');
    signal p12_oy, p12_ou, p12_ov : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y13, s_u13, s_v13 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y15, s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
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
    -- ROMs and the camera row memory
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_sn_q <= C_TP_SIN(to_integer(r_sn_a));
            r_ex_q <= C_TP_EXP2(to_integer(r_ex_a));
            r_pi_q <= C_TP_PIN(to_integer(r_pi_a));
            r_po_q <= C_TP_POUT(to_integer(r_po_a));
        end if;
    end process p_rom;

    p_cam : process(clk)
    begin
        if rising_edge(clk) then
            cam_q <= cam(to_integer(cam_ra));
            if cam_we = '1' then cam(to_integer(cam_wa)) <= cam_wd; end if;
        end if;
    end process p_cam;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(10 downto 0);
        variable v_k  : std_logic_vector(31 downto 0);
        variable v_d  : unsigned(23 downto 0);
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
            v_s := signed(resize(s_k4, 11)) - 512;
            s_vneg <= v_s(10);
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
                s_flip <= registers_in(6)(0);
                s_inv  <= registers_in(6)(1);
                s_pal  <= registers_in(6)(2);
                s_cont <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                s_xs16 <= s_xsr & "0000";
                -- the beam starts 8 px in (the two output boxes' group delay), as in lozenge
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(512, 10); s_wsp <= to_unsigned(1408, 12); s_bsp <= to_unsigned(1488, 12);
                    when 3      => s_x0 <= to_unsigned(24, 10);  s_wsp <= to_unsigned(1896, 12); s_bsp <= to_unsigned(1976, 12);
                    when others => s_x0 <= to_unsigned(16, 10);  s_wsp <= to_unsigned(1904, 12); s_bsp <= to_unsigned(1984, 12);
                end case;
                if s_ys = 256 then s_hd1080 <= '1'; else s_hd1080 <= '0'; end if;
            end if;
            g_wm1 <= s_w - 1;

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

            -- serial products: slot k loads at zc = 40 + 24 k, 16 iterations, result stable from
            -- load + 17 (stores at load + 18); the slot is decoded at load - 2, the operands pre-muxed at load - 1
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 38  => z_sel <= to_unsigned(0, 4);  z_pre <= '1';
                    when 62  => z_sel <= to_unsigned(1, 4);  z_pre <= '1';
                    when 86  => z_sel <= to_unsigned(2, 4);  z_pre <= '1';
                    when 110 => z_sel <= to_unsigned(3, 4);  z_pre <= '1';
                    when 134 => z_sel <= to_unsigned(4, 4);  z_pre <= '1';
                    when 158 => z_sel <= to_unsigned(5, 4);  z_pre <= '1';
                    when 182 => z_sel <= to_unsigned(6, 4);  z_pre <= '1';
                    when 206 => z_sel <= to_unsigned(7, 4);  z_pre <= '1';
                    when 230 => z_sel <= to_unsigned(8, 4);  z_pre <= '1';
                    when 254 => z_sel <= to_unsigned(9, 4);  z_pre <= '1';
                    when 278 => z_sel <= to_unsigned(10, 4); z_pre <= '1';
                    when 302 => z_sel <= to_unsigned(11, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(28967, 36);          z_fb <= z_mt;                      -- carrier step
                    when 1  => z_fa <= resize(c_kx, 36);                z_fb <= resize(s_wsp, 16);         -- phase at x0
                    when 2  => z_fa <= resize(s_g2(12 downto 3), 36);   z_fb <= to_unsigned(10702, 16);    -- bowl slope
                    when 3  => z_fa <= resize(c_ds, 36);                z_fb <= resize(s_bsp, 16);         -- bowl at x0
                    when 4  => z_fa <= resize(s_vm, 36);                z_fb <= resize(s_vm, 16);          -- rise^2
                    when 5  => z_fa <= resize(z_a, 36);                 z_fb <= to_unsigned(1225, 16);     -- drift
                    when 6  => z_fa <= to_unsigned(81429, 36);          z_fb <= z_mt;                      -- hue step / virtual line
                    when 7  => z_fa <= resize(c_khv, 36);               z_fb <= resize(s_ys, 16);          -- x line step
                    when 8  => z_fa <= resize(c_kh, 36);                z_fb <= to_unsigned(1212, 16);     -- background step
                    when 9  => z_fa <= to_unsigned(12158, 36);          z_fb <= resize(s_ys, 16);          -- sine step
                    when 10 => z_fa <= to_unsigned(108737, 36);         z_fb <= resize(s_ys, 16);          -- band step
                    when others => z_fa <= to_unsigned(1421, 36);       z_fb <= z_mt;                      -- block width
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
                    when 21 =>
                        if s_pal = '1' then v_k := C_TP_POUT(32); else v_k := C_TP_POUT(0); end if;
                        c_ky <= unsigned(v_k(31 downto 22)); c_ku <= unsigned(v_k(21 downto 12)); c_kv <= unsigned(v_k(11 downto 2));
                    -- tapers: e = 0.75 (k1 - 512), +-1.5 octaves
                    when 28 =>
                        v_e := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        v_e := v_e - shift_right(v_e, 2);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 30 => z_mt <= r_ex_q;
                    when 58 =>
                        case to_integer(z_sh) is
                            when -2     => c_kx <= f_acc(33 downto 17);
                            when -1     => c_kx <= f_acc(32 downto 16);
                            when 0      => c_kx <= f_acc(31 downto 15);
                            when others => c_kx <= f_acc(30 downto 14);
                        end case;
                    when 82  => c_ph0 <= to_unsigned(C_PH_ANCH, 24) - f_acc(23 downto 0);
                    when 106 => c_ds  <= f_acc(23 downto 12);
                    when 130 => c_d0  <= signed(resize(f_acc(23 downto 0), 24)) - 524288;
                    when 154 => z_a   <= f_acc(17 downto 0);
                    when 178 => c_drift <= f_acc(28 downto 10);
                    -- hue cycle: e = 0.75 (512 - k5)
                    when 180 =>
                        v_e := signed(resize(s_g5(12 downto 3), 11)) - 512;
                        v_e := v_e - shift_right(v_e, 2);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 182 => z_mt <= r_ex_q;
                    when 202 =>
                        case to_integer(z_sh) is
                            when -2     => c_khv <= f_acc(34 downto 17);
                            when -1     => c_khv <= f_acc(33 downto 16);
                            when 0      => c_khv <= f_acc(32 downto 15);
                            when others => c_khv <= f_acc(31 downto 14);
                        end case;
                    when 226 => c_kh <= f_acc(27 downto 8);
                    when 250 => c_kg <= f_acc(30 downto 10);
                    when 274 => c_ks <= f_acc(23 downto 8);
                    when 298 => c_kb <= f_acc(26 downto 8);
                    -- mosaic: e = 1.125 (k6 - 512), +-2.25 octaves
                    when 300 =>
                        v_e := signed(resize(s_g6(12 downto 3), 11)) - 512;
                        v_e := v_e + shift_right(v_e, 3);
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                    when 302 => z_mt <= r_ex_q;
                    when 322 =>
                        case to_integer(z_sh) is
                            when -3     => c_bwh <= f_acc(31 downto 18);
                            when -2     => c_bwh <= f_acc(30 downto 17);
                            when -1     => c_bwh <= f_acc(29 downto 16);
                            when 0      => c_bwh <= f_acc(28 downto 15);
                            when 1      => c_bwh <= f_acc(27 downto 14);
                            when others => c_bwh <= f_acc(26 downto 13);
                        end case;
                    when 330 =>
                        case to_integer(s_xs) is
                            when 4      => c_kxs <= shift_left(resize(c_kx, 19), 2); c_dsx <= shift_left(resize(c_ds, 14), 2);
                                           c_oxs <= to_unsigned(216, 9); c_ox2s <= to_unsigned(100, 9);
                            when 3      => c_kxs <= shift_left(resize(c_kx, 19), 1) + resize(c_kx, 19);
                                           c_dsx <= shift_left(resize(c_ds, 14), 1) + resize(c_ds, 14);
                                           c_oxs <= to_unsigned(162, 9); c_ox2s <= to_unsigned(75, 9);
                            when others => c_kxs <= shift_left(resize(c_kx, 19), 1); c_dsx <= shift_left(resize(c_ds, 14), 1);
                                           c_oxs <= to_unsigned(108, 9); c_ox2s <= to_unsigned(50, 9);
                        end case;
                        c_bh  <= c_bwh & "00";
                        c_bh2 <= '0' & c_bwh & '0';
                        c_asaw <= s_g3(12 downto 3);
                        c_toff <= shift_left(resize(to_signed(512, 12) - signed(resize(s_gp(12 downto 3), 12)), 24), 11);
                        c_oys <= resize(shift_right(resize(s_ys, 20) * to_unsigned(73, 8), 8), 10);
                    when 332 =>
                        -- the phases of the first line of this field: the clip's phases, drifting
                        if s_hd1080 = '1' then v_d := resize(c_drift, 23) & '0'; else v_d := resize(c_drift, 24); end if;
                        if z_first = '1' then
                            z_first <= '0';
                            c_ps0 <= to_unsigned(C_PS_CLIP, 24); c_phh0 <= to_unsigned(C_PH_CLIP, 24); c_pg0 <= (others => '0');
                        elsif s_vneg = '1' then
                            c_ps0  <= c_ps0 - v_d;
                            c_phh0 <= c_phh0 - (v_d(22 downto 0) & '0');
                            c_pg0  <= c_pg0 - (v_d(22 downto 0) & '0') - v_d;
                        else
                            c_ps0  <= c_ps0 + v_d;
                            c_phh0 <= c_phh0 + (v_d(22 downto 0) & '0');
                            c_pg0  <= c_pg0 + (v_d(22 downto 0) & '0') + v_d;
                        end if;
                        c_ox0 <= c_ox0 + 100; c_ox20 <= c_ox20 - 60; c_oy0 <= c_oy0 + 40;
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
    -- Line engine.  At the zone kick: the field's line-0 phases.  At each line
    -- start: the current line's values are taken from n_*, then the phases
    -- advance to the next line, the sawtooth level is multiplied, and the
    -- threshold sine is read from the shared ROM 5 clocks after the generated
    -- avid falls (or in the zone for line 0).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_rc : unsigned(16 downto 0);
        variable v_pb : unsigned(24 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 400 then e_zk <= '1'; end if;

            if e_zk = '1' or g_start = '1' then
                e_lc <= (others => '0');
            elsif e_lc /= 63 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(38 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            if e_zk = '1' then
                -- line 0 of the field
                if s_flip = '1' then
                    e_ps <= c_ps0 + to_unsigned(C_PS_FLIP, 24);
                    e_ph <= c_phh0 + to_unsigned(C_PH_FLIP, 24);
                    e_pg <= c_pg0 + to_unsigned(C_PG_FLIP, 24);
                else
                    e_ps <= c_ps0; e_ph <= c_phh0; e_pg <= c_pg0;
                end if;
                e_pb <= to_unsigned(C_PB_CLIP, 24);
                e_rc <= (others => '0');
                n_samp <= '1';
                e_oy <= c_oy0;
            elsif g_start = '1' then
                l_t <= n_t; l_hue <= n_hue; l_a <= n_a; l_b <= n_b; l_half <= n_half; l_samp <= n_samp;
                -- advance to the next line
                if s_flip = '1' then
                    e_ps <= e_ps - c_ks; e_pb <= e_pb - c_kb; e_ph <= e_ph - c_kh; e_pg <= e_pg - c_kg;
                else
                    e_ps <= e_ps + c_ks; e_pb <= e_pb + c_kb; e_ph <= e_ph + c_kh; e_pg <= e_pg + c_kg;
                end if;
                v_rc := resize(e_rc, 17) + resize(s_ys, 17);
                if v_rc >= resize(c_bh, 17) then
                    e_rc <= resize(v_rc - resize(c_bh, 17), 16); n_samp <= '1';
                else
                    e_rc <= v_rc(15 downto 0); n_samp <= '0';
                end if;
                e_oy <= e_oy + c_oys;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    if e_rc >= c_bh2 then n_half <= '1'; else n_half <= '0'; end if;
                    if n_samp = '1' or s_cont = '1' then n_hue <= e_ph(23 downto 19); end if;
                    e_pgq <= e_pg - 4194304;
                when 2 =>
                    if n_samp = '1' then n_a <= not e_pg(23); n_b <= not e_pgq(23); end if;
                    v_pb := to_unsigned(16777216, 25) - resize(e_pb, 25);
                    m_a <= resize(v_pb, 40); m_b <= resize(c_asaw, 16); m_acc <= (others => '0');
                when 20 =>
                    n_tsaw <= signed(resize(m_acc(37 downto 14), 24)) - signed(resize(c_asaw & "000000000", 24));
                when others => null;
            end case;

            -- threshold sine for the next line: one read of the shared ROM in hblank
            if (s_gav_sr(0) = '1' and g_avid = '0') or (s_zone = '1' and s_zc = 430) then   -- line 0: after its saw product (zc 422)
                y_cnt <= to_unsigned(1, 6);
            elsif y_cnt /= 0 and y_cnt /= 40 then
                y_cnt <= y_cnt + 1;
            elsif y_cnt = 40 then
                y_cnt <= (others => '0');
            end if;
            if y_cnt = 5 then y_sel <= '1'; elsif y_cnt = 6 then y_sel <= '0'; end if;
            if y_cnt = 8 then e_sin <= r_sn_q; end if;
            if y_cnt = 9 then n_tsin <= shift_left(resize(e_sin, 24), 9) + shift_left(resize(e_sin, 24), 8); end if;
            if y_cnt = 10 then n_t <= n_tsin + n_tsaw + c_toff; end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: per-pixel accumulators (carrier phase, bowl, mosaic column, the
    -- Osc generator) and the camera level sampled at every block start
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_bc : unsigned(14 downto 0);
        variable v_y  : unsigned(9 downto 0);
        variable v_y11 : unsigned(10 downto 0);
        variable v_s  : unsigned(8 downto 0);
        variable v_l  : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- the level of the input pixel: camera luma in 8 steps, or the generator
            -- camera: (Y - 64) * 9/8 in 8 steps of 114, so the eighth step is reached at Y = 860
            v_y := unsigned(data_in.y);
            if v_y < 64 then v_y := (others => '0'); else v_y := v_y - 64; end if;
            v_y11 := resize(v_y, 11) + resize(v_y srl 3, 11);
            if v_y11(10) = '1' then v_y11 := to_unsigned(1023, 11); end if;
            v_s := resize(f_tri7(b_ox(15 downto 8)), 9) + resize(f_tri7(b_ox2(15 downto 8)) srl 1, 9) + resize(f_tri7(e_oy(15 downto 8)) srl 3, 9);
            o_s <= v_s;
            if o_s < 64 then o_lv <= (others => '0');
            else
                v_l := o_s - 64;
                if v_l(8 downto 7) /= "00" then o_lv <= "111"; else o_lv <= v_l(6 downto 4); end if;
            end if;
            if s_src = '1' then s_lvin <= o_lv; else s_lvin <= v_y11(9 downto 7); end if;

            if g_start = '1' then
                b_x   <= resize(s_x0, 12);
                b_dir <= '1';
                b_phi <= c_ph0;
                b_d   <= c_d0;
                b_bc  <= (others => '0');
                b_bi  <= (others => '0');
                b_bst <= '1';
                b_hold <= s_lvin;
                b_ox  <= c_ox0; b_ox2 <= c_ox20;
            elsif g_avid = '1' then
                b_x <= b_x + s_xs;
                if b_x + s_xs < 2000 then b_dir <= '1'; else b_dir <= '0'; end if;
                b_phi <= b_phi + c_kxs;
                if b_dir = '1' then b_d <= b_d - signed(resize(c_dsx, 24)); else b_d <= b_d + signed(resize(c_dsx, 24)); end if;
                v_bc := b_bc + s_xs16;
                if v_bc >= c_bwh then
                    b_bc <= v_bc - c_bwh; b_bi <= b_bi + 1; b_bst <= '1'; b_hold <= s_lvin;
                else
                    b_bc <= v_bc; b_bst <= '0';
                end if;
                b_ox <= b_ox + c_oxs; b_ox2 <= b_ox2 + c_ox2s;
            else
                b_bst <= '0';
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_m  : signed(25 downto 0);
        variable v_in, v_out : std_logic_vector(31 downto 0);
        variable v_dy, v_du, v_dv : signed(10 downto 0);
        variable v_py, v_pu, v_pv : signed(12 downto 0);
        variable v_sy, v_su, v_sv : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: ROM / RAM addresses
            if y_sel = '1' then r_sn_a <= e_ps(23 downto 14); else r_sn_a <= b_phi(23 downto 14); end if;
            cam_ra <= b_bi;
            cam_we <= b_bst and l_samp and g_avid;
            cam_wa <= b_bi; cam_wd <= '0' & std_logic_vector(b_hold);
            p2_d <= b_d; p2_lv <= b_hold;
            -- 3: ROM / RAM data
            p3_d <= p2_d; p3_lv <= p2_lv;
            -- 4: fabric copies; the block's level (x-hold on the sampling line, the row memory otherwise)
            p4_cos <= r_sn_q; p4_d <= p3_d;
            if l_samp = '1' then p4_l <= p3_lv; else p4_l <= unsigned(cam_q(2 downto 0)); end if;
            -- 5: carrier - bowl (Q20)
            p5_hd <= shift_left(resize(p4_cos, 25), 10) - resize(p4_d, 25);
            p5_l <= p4_l;
            -- 6: minus the threshold
            p6_m <= resize(p5_hd, 26) - resize(l_t, 26);
            p6_l <= p5_l;
            -- 7: soft key weight (0 .. 4 over +-1/16), palette addresses
            v_m := p6_m;
            if v_m >= 65536 then p7_w <= "100";
            elsif v_m < -65536 then p7_w <= "000";
            else p7_w <= '0' & unsigned(v_m(16 downto 15)) xor "010"; end if;
            -- the camera inside the tapers: level 2 shifts the hue a quarter cycle, 5 a half, 6 and 7 show through
            if p6_l = 2 then r_pi_a <= "00" & s_pal & (l_hue + 8);
            elsif p6_l = 5 then r_pi_a <= "00" & s_pal & (l_hue + 16);
            else r_pi_a <= "00" & s_pal & l_hue; end if;
            if p6_l = 2 then r_po_a <= "00" & s_pal & p6_l & l_half & (l_half xor l_a);
            else r_po_a <= "00" & s_pal & p6_l & l_a & l_b; end if;
            if p6_l(2 downto 1) = "11" then p7_over <= '1'; else p7_over <= '0'; end if;
            -- 8: palette ROMs (EBR)
            p8_w <= p7_w; p8_over <= p7_over;
            -- 9: fabric copies
            p9_in <= r_pi_q; p9_out <= r_po_q; p9_w <= p8_w; p9_over <= p8_over;
            -- 10: camera override, Invert
            if p9_over = '1' then v_in := p9_out; else v_in := p9_in; end if;
            v_out := p9_out;
            if s_inv = '1' then p10_in <= v_out; p10_out <= v_in; else p10_in <= v_in; p10_out <= v_out; end if;
            p10_w <= p9_w;
            -- 11: inside - outside
            p11_dy <= signed(resize(unsigned(p10_in(31 downto 22)), 11)) - signed(resize(unsigned(p10_out(31 downto 22)), 11));
            p11_du <= signed(resize(unsigned(p10_in(21 downto 12)), 11)) - signed(resize(unsigned(p10_out(21 downto 12)), 11));
            p11_dv <= signed(resize(unsigned(p10_in(11 downto 2)), 11)) - signed(resize(unsigned(p10_out(11 downto 2)), 11));
            p11_oy <= unsigned(p10_out(31 downto 22)); p11_ou <= unsigned(p10_out(21 downto 12)); p11_ov <= unsigned(p10_out(11 downto 2));
            p11_w <= p10_w;
            -- 12: times the weight (0, 1, 2, 3, 4)
            case to_integer(p11_w) is
                when 0 => v_py := (others => '0'); v_pu := (others => '0'); v_pv := (others => '0');
                when 1 => v_py := resize(p11_dy, 13); v_pu := resize(p11_du, 13); v_pv := resize(p11_dv, 13);
                when 2 => v_py := shift_left(resize(p11_dy, 13), 1); v_pu := shift_left(resize(p11_du, 13), 1); v_pv := shift_left(resize(p11_dv, 13), 1);
                when 3 => v_py := shift_left(resize(p11_dy, 13), 1) + resize(p11_dy, 13);
                          v_pu := shift_left(resize(p11_du, 13), 1) + resize(p11_du, 13);
                          v_pv := shift_left(resize(p11_dv, 13), 1) + resize(p11_dv, 13);
                when others => v_py := shift_left(resize(p11_dy, 13), 2); v_pu := shift_left(resize(p11_du, 13), 2); v_pv := shift_left(resize(p11_dv, 13), 2);
            end case;
            p12_py <= v_py; p12_pu <= v_pu; p12_pv <= v_pv;
            p12_oy <= p11_oy; p12_ou <= p11_ou; p12_ov <= p11_ov;
            -- 13: blend
            v_sy := signed(resize(p12_oy, 14)) + resize(shift_right(p12_py, 2), 14);
            v_su := signed(resize(p12_ou, 14)) + resize(shift_right(p12_pu, 2), 14);
            v_sv := signed(resize(p12_ov, 14)) + resize(shift_right(p12_pv, 2), 14);
            s_y13 <= unsigned(v_sy(9 downto 0)); s_u13 <= unsigned(v_su(9 downto 0)); s_v13 <= unsigned(v_sv(9 downto 0));
            -- 14: compose; palette black outside the generated avid (this beam registers a pixel one clock
            -- after its g_avid clock, so s_gav_sr(k) is the stage-(k+1) pixel here: the box / gate / output
            -- indices then match lozenge's with one stage fewer in front)
            if s_gav_sr(12) = '0' then
                s_y15 <= c_ky; s_u15 <= c_ku; s_v15 <= c_kv;
            else
                s_y15 <= s_y13; s_u15 <= s_u13; s_v15 <= s_v13;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (the HD HDMI bar fix and the clip's softness), luma 13/16 and the
    -- blanking gate.  The running 8-sums snap back to the palette black once
    -- 8 blank samples are in (resync).
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_cay(0) <= s_y15; s_cau(0) <= s_u15; s_cav(0) <= s_v15;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= c_ky & "000"; s_bau <= c_ku & "000"; s_bav <= c_kv & "000";
            else
                s_bay <= s_bay + s_y15 - s_cay(7);
                s_bau <= s_bau + s_u15 - s_cau(7);
                s_bav <= s_bav + s_v15 - s_cav(7);
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(14) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y17 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u17 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v17 <= v_sc(13 downto 4);
            else
                s_y17 <= c_ky; s_u17 <= c_ku; s_v17 <= c_kv;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            s_cby(0) <= s_y17; s_cbu(0) <= s_u17; s_cbv(0) <= s_v17;
            for i in 1 to 7 loop s_cby(i) <= s_cby(i - 1); s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bby <= c_ky & "000"; s_bbu <= c_ku & "000"; s_bbv <= c_kv & "000";
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

end architecture tapers;
