-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Bunting v0.6.0 -- Heirloom #35, rebuilt from the photo WIN_20230521_23_37_25; phase 2 (Ron 10-30).
--
-- The photo: a white ground cut into yellow and cyan pennants by two bow-tie "X"s (centres x 527
-- and 1471, y 516: one H triangle oscillator, period 945 px), a middle band that flares open to
-- the right (half-height 178 + 6.9e-5 (x - 42)^2 lines, mirrored about y 519) and, over it, a
-- 4.3 x 4.6 grid of identical 8 x 3-block glyph marks (pitch 450 x 233) keyed into the ground:
-- red on yellow, blue on cyan, and on white a black one-shot (33 px from each run start) that
-- charges up to grey.  Everything is softened horizontally (~16 px); the first 65 px of each line
-- hold the X oscillator.
--
-- The patch, as fitted (.heirloom_work/rb_bunting/fit2.py, gedge3.py, oneshot.py), with the
-- phase-2 scalings (sh, sv = P12; T = K2; C, xv = K3):
--   Tx  = H triangle (0 at the X centres, 1 midway), Ty = |y - 516.2|
--   K   = Tx > 1 - 0.894 sh + sh Ty / (309 sv)  or  Tx < sh sv (Ty - 70.1) / 876   (the bow-tie wedges)
--   mid = |y - 519.1| < 178 + T (x - 960) + C (x - xv)^2                         (the band)
--   ground = mid ? (K ? cyan : white) : (K ? white : yellow)
--   glyph ink: yellow -> red (mask dilated 3.8 px), cyan -> blue (eroded 3.8 px),
--              white -> black for 33 px of each run, then an RC slew from dark grey to the source
--
--   K1  Flags    X oscillator frequency: 0.375 .. 12 bow-ties across (2.03 = the photo)
--   K2  Band     bipolar taper: the band closes into a pennant facing left (0 %) or right (100 %)
--   K3  Flare    bipolar: the band flares open to the left (0 %) or right (100 %), straight at 50 %
--   K4  Speed    bipolar (centre +-8 still): the X's drift sideways; the glyph grid glides to a
--                position 2 px per knob step from home at the default (-1200 .. +846 px)
--   K5  Glyphs   glyph grid size: 1.5 .. 9 columns (4.26 = the photo); away from the photo's size
--                every glyph's blocks re-roll, a different glyph in each cell
--   K6  Bend     how far the picture (or the Osc) bends the X oscillator's phase
--   S7  Shot     On (the photo's black one-shot in the grey glyphs) / Off (plain grey)
--   S8  Swap     yellow and cyan exchanged (inks follow: red on yellow, blue on cyan)
--   S9  Flip     the glyph marks mirrored
--   S10 Edges    Soft (the photo's horizontal softness) / Hard
--   S11 Source   Video / Osc
--   P12 SPREAD   the wedges: 0 .. 50 % widen from slivers to the photo, 50 .. 100 % grow tall
--                into wide fans with a yellow zig-zag string
--
-- Engine: vblank-zone sequencer (one serial multiplier, one serial divider, shared glide), a
-- per-line engine kicked at each line start (computes the NEXT line's wedge thresholds, band
-- offset and glyph row, and writes the next line's 16 glyph masks into a line-parity EBR bank
-- from a serial hash), beam DDAs (X phase, band quadratic F(x), glyph phase with a cell index),
-- a 20-stage pixel path.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated
-- avid; soft [1 2 .. 2 1] box on Y/U/V; output conditioning (chroma [1 2 .. 2 1]/16, luma [1 2 1],
-- luma 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture bunting of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- fitted constants (h = half virtual px; Q8 = virtual lines * 256; Q12 = fraction of the half period)
    constant C_CY8  : integer := 132140;    -- X centre y 516.17
    constant C_CYZ8 : integer := 132882;    -- band axis y 519.07
    constant C_Z08  : integer := 45619;     -- band half-height 178.2 lines
    constant C_V08  : integer := 17946;     -- V wedge apex 70.1 lines from the centre
    constant C_XH   : integer := 130;       -- line-start hold (65 virtual px)
    constant C_UANC   : integer := 14011240;  -- glyph x phase at the anchor (+64/4096 offset), Q24
    constant C_VANC   : integer := 11540376;  -- glyph y phase at the anchor, Q24
    constant C_DIL  : integer := 35;        -- glyph ink dilation (3.8 px), Q12 of the pitch

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
    -- x step in h per pixel: 1080 = 2, 720p = 3, SD = 4 (SD = the centre 1440 virtual px)
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

    -- rings (block RAM): soft box {Y, U, V}, shell chroma box {U, V}
    type t_sring is array (0 to 255) of std_logic_vector(31 downto 0);
    signal sr_ram : t_sring := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    type t_cring is array (0 to 255) of std_logic_vector(19 downto 0);
    signal cr_ram : t_cring := (others => "1000000000" & "1000000000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of sr_ram : signal is true;
    attribute no_rw_check of cr_ram : signal is true;
    signal s_wp, sr_ra, cr_ra : unsigned(7 downto 0) := (others => '0');
    signal sr_wd : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";
    signal sr_q, sr_q2 : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";
    signal cr_wd : std_logic_vector(19 downto 0) := "1000000000" & "1000000000";
    signal cr_q, cr_q2 : std_logic_vector(19 downto 0) := "1000000000" & "1000000000";
    -- glyph masks (block RAM): one 8-block mask per (line-parity bank, cell column mod 16)
    type t_mram is array (0 to 255) of std_logic_vector(7 downto 0);
    signal mk_ram : t_mram := (others => (others => '0'));
    attribute no_rw_check of mk_ram : signal is true;
    signal mk_wa  : unsigned(7 downto 0) := (others => '0');
    signal mk_wd  : std_logic_vector(7 downto 0) := (others => '0');
    signal mk_we  : std_logic := '0';
    signal mk_ra  : unsigned(7 downto 0) := (others => '0');
    signal mk_q, mk_q2 : std_logic_vector(7 downto 0) := (others => '0');
    signal mk_rb  : std_logic := '0';                                 -- bank the beam reads
    signal mk_wb  : std_logic := '1';                                 -- bank the engine writes

    -- oscillator source (Source = Osc): 2-D triangle lattice, ~2520 x 2160 virtual px, drifting
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(8 downto 0) := (others => '0');
    signal s_oxs : unsigned(5 downto 0) := to_unsigned(26, 6);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(61, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');

    -- timing / zone / width / mode
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xh0  : unsigned(8 downto 0) := (others => '0');
    signal s_xd   : unsigned(10 downto 0) := to_unsigned(1868, 11);   -- 1998 - start of the X phase
    signal s_xg   : unsigned(10 downto 0) := to_unsigned(1898, 11);   -- 1898 - line start (glyph anchor)
    signal s_hd   : std_logic := '1';                                 -- 1080 class (soft box 16, slew >> 5)
    signal s_short : std_logic := '0';                                -- 720p / SD: the short chroma box
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(9 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(506 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(768 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(621 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(340 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(600, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_shot, s_swap, s_flip, s_hard, s_src : std_logic := '0';
    signal s_g4   : unsigned(15 downto 0) := to_unsigned(600 * 64, 16);   -- K4 slide glide (Q6)
    signal s_g4d  : signed(16 downto 0) := (others => '0');
    signal s_off  : signed(12 downto 0) := (others => '0');          -- glyph slide, h (signed)
    signal s_m3, s_n2 : unsigned(8 downto 0) := (others => '0');      -- |K3|, |K2| (0 within +-8 of the centre)
    signal s_face : std_logic := '1';                                 -- K3: flare opens right
    signal s_tsg  : std_logic := '1';                                 -- K2: band widens right
    signal s_dz   : unsigned(12 downto 0) := (others => '0');         -- K5 distance from the photo (Q3)
    signal s_zside : std_logic := '0';

    -- per-field constants
    signal c_k1sq, c_k1q, c_k5sq : unsigned(19 downto 0) := (others => '0');
    signal c_k3sq  : unsigned(17 downto 0) := (others => '0');
    signal c_inch  : unsigned(15 downto 0) := to_unsigned(8879, 16);    -- X phase per h (2^24 / period)
    signal c_incp  : unsigned(17 downto 0) := to_unsigned(17758, 18);   -- X phase per pixel
    signal c_po    : unsigned(23 downto 0) := (others => '0');          -- (1998 - xs) * inc_h
    signal s_up    : std_logic := '0';                                  -- P12 in its vertical half
    signal c_s     : unsigned(12 downto 0) := to_unsigned(4096, 13);    -- sv, then sh sv (Q12)
    signal c_sh    : unsigned(12 downto 0) := to_unsigned(4096, 13);    -- horizontal spread sh, Q12
    signal c_is    : unsigned(17 downto 0) := to_unsigned(65536, 18);   -- 1 / sv, then sh / sv (Q16)
    signal c_iar, c_ial : unsigned(17 downto 0) := (others => '0');
    signal c_sb    : unsigned(15 downto 0) := (others => '0');
    signal c_h0    : unsigned(11 downto 0) := to_unsigned(433, 12);     -- H wedge apex, Q12 of the half period
    signal c_duh   : unsigned(15 downto 0) := to_unsigned(18628, 16);
    signal c_dup   : unsigned(17 downto 0) := (others => '0');
    signal c_dvv   : unsigned(17 downto 0) := (others => '0');
    signal c_dvl   : unsigned(19 downto 0) := (others => '0');
    signal c_drs   : unsigned(16 downto 0) := (others => '0');
    signal c_dr    : unsigned(23 downto 0) := (others => '0');          -- X drift, Q24
    signal c_phi0  : unsigned(23 downto 0) := (others => '0');
    signal c_u0    : unsigned(27 downto 0) := (others => '0');          -- glyph x phase: cell (27..24) . Q24
    signal c_v0    : unsigned(27 downto 0) := (others => '0');          -- glyph y phase: cell (27..24) . Q24
    -- band F(x) = T (x - 960) + C (x - xv)^2 in Q22 lines, stepped per pixel (F += G; G += H)
    signal c_C     : unsigned(16 downto 0) := (others => '0');          -- flare, Q30 lines per h^2
    signal c_T     : signed(19 downto 0) := (others => '0');            -- taper, Q20 lines per h
    signal c_H     : unsigned(13 downto 0) := (others => '0');
    signal c_F0    : signed(33 downto 0) := (others => '0');
    signal c_G0    : signed(25 downto 0) := (others => '0');
    signal c_g6    : unsigned(7 downto 0) := to_unsigned(85, 8);

    -- field serial multiplier (a 20 x b 18 bits, 18 iterations) and divider (35 / 20 bits)
    signal f_a   : unsigned(37 downto 0) := (others => '0');
    signal f_b   : unsigned(17 downto 0) := (others => '0');
    signal f_acc : unsigned(37 downto 0) := (others => '0');
    signal z_fa  : unsigned(19 downto 0) := (others => '0');
    signal z_fb  : unsigned(17 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';
    signal z_neg : std_logic := '0';
    signal z_sx, z_ng, z_ac, z_in : std_logic := '0';               -- slot: signed a, negate a, accumulate, inc_h base
    signal d_num : unsigned(34 downto 0) := (others => '0');
    signal d_den : unsigned(19 downto 0) := (others => '1');
    signal d_rem : unsigned(20 downto 0) := (others => '0');
    signal d_q   : unsigned(24 downto 0) := (others => '0');
    signal d_i   : unsigned(5 downto 0) := (others => '0');
    signal d_run, d_ld : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- line engine
    signal e_lc   : unsigned(6 downto 0) := to_unsigned(127, 7);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(19 downto 0) := (others => '0');
    signal e_vg   : unsigned(27 downto 0) := (others => '0');
    signal e_ty, e_tz : unsigned(17 downto 0) := (others => '0');
    signal e_dz   : signed(18 downto 0) := (others => '0');
    signal e_tvz  : std_logic := '0';
    signal e_row  : unsigned(1 downto 0) := "11";
    signal e_rowc : unsigned(3 downto 0) := (others => '0');
    signal m_a    : unsigned(40 downto 0) := (others => '0');
    signal m_b    : unsigned(17 downto 0) := (others => '0');
    signal m_acc  : unsigned(40 downto 0) := (others => '0');
    signal n_thr, n_thl, n_tv : unsigned(11 downto 0) := (others => '0');
    signal n_d    : signed(18 downto 0) := (others => '0');

    -- glyph mask engine: 16 columns x 8 blocks per line, one block per clock through a 7-stage
    -- hash pipeline (block bit = photo bit xor a re-roll that starts as K5 leaves the photo)
    signal k_run  : std_logic := '0';
    signal k_j    : unsigned(6 downto 0) := (others => '0');
    signal k_row  : unsigned(1 downto 0) := "11";
    signal k_rowc : unsigned(3 downto 0) := (others => '0');
    signal k_wj   : unsigned(6 downto 0) := (others => '0');          -- write side: column . block
    signal k_vp   : std_logic_vector(1 to 6) := (others => '0');
    signal k_a, k_b, k_h, k_x, k_y, k_z : unsigned(15 downto 0) := (others => '0');
    signal k_g    : unsigned(5 downto 0) := (others => '0');
    signal k_gz5, k_gz6 : std_logic := '0';
    signal k_m    : std_logic_vector(6 downto 0) := (others => '0');

    -- beam
    signal b_xh, b_x1 : unsigned(11 downto 0) := (others => '0');
    signal b_phi : unsigned(23 downto 0) := (others => '0');
    signal b_u   : unsigned(27 downto 0) := (others => '0');
    signal b_F   : signed(33 downto 0) := (others => '0');
    signal b_G   : signed(25 downto 0) := (others => '0');
    signal l_thr, l_thl, l_tv : unsigned(11 downto 0) := (others => '0');
    signal l_d   : signed(18 downto 0) := (others => '0');

    -- pixel path
    signal p1_l8  : unsigned(7 downto 0) := (others => '0');
    signal p1_os  : signed(8 downto 0) := (others => '0');
    signal p1_phi, p2_phi, p3_phi, p4_phi : unsigned(23 downto 0) := (others => '0');
    signal p1_zs  : std_logic := '0';
    signal p1_v0  : signed(12 downto 0) := (others => '0');                -- glyph x - 64
    signal p2_bin : signed(9 downto 0) := (others => '0');
    signal p2_src : unsigned(7 downto 0) := (others => '0');
    signal p2_mid : std_logic := '0';
    signal p2_i0, p2_ip, p2_im : unsigned(2 downto 0) := (others => '0');  -- block indices
    signal p2_k0, p2_kp, p2_km : std_logic := '0';                         -- inside the 8 blocks
    signal p3_gw, p3_gy, p3_gc : std_logic := '0';
    signal p3_pl, p3_ph : signed(14 downto 0) := (others => '0');
    signal p4_bp  : signed(18 downto 0) := (others => '0');
    signal p5_phib : unsigned(23 downto 0) := (others => '0');
    signal p6_txn : unsigned(11 downto 0) := (others => '0');
    signal p6_rt  : std_logic := '0';
    signal p7_k   : std_logic := '0';
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    signal p_sv   : t_u8a(3 to 8) := (others => (others => '0'));
    type t_sla is array (natural range <>) of std_logic;
    signal p_gw, p_gy, p_gc : t_sla(4 to 7) := (others => '0');
    signal p_mid : t_sla(3 to 7) := (others => '0');
    signal p8_cls : unsigned(1 downto 0) := "00";
    signal p8_wg, p8_red, p8_blu : std_logic := '0';
    signal p8_sa  : unsigned(11 downto 0) := (others => '0');
    signal p9_cls : unsigned(1 downto 0) := "00";
    signal p9_wg, p9_red, p9_blu, p9_blk : std_logic := '0';
    signal p9_g   : unsigned(7 downto 0) := (others => '0');
    signal x_cnt  : unsigned(6 downto 0) := (others => '0');
    signal x_g    : unsigned(11 downto 0) := to_unsigned(1520, 12);
    signal x_seg  : unsigned(6 downto 0) := (others => '0');
    signal x_lf   : std_logic_vector(15 downto 0) := x"ACE1";
    signal x_nz   : signed(3 downto 0) := (others => '0');
    signal p10_code : unsigned(2 downto 0) := (others => '0');
    signal p10_g  : unsigned(7 downto 0) := (others => '0');
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_hw1  : std_logic_vector(29 downto 0) := (others => '0');
    signal s_sy, s_su, s_sv, s_syd, s_sud, s_svd : unsigned(14 downto 0) := (others => '0');
    signal s_sbh  : std_logic_vector(15 downto 0) := (others => '0');   -- input gate of the last 16 soft-ring samples
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10a is array (natural range <>) of unsigned(9 downto 0);
    signal s_ly : t_u10a(0 to 6) := (others => to_unsigned(64, 10));
    signal s_usum, s_vsum, s_ud, s_vd : unsigned(13 downto 0) := (others => '0');
    signal s_cbh : std_logic_vector(7 downto 0) := (others => '0');   -- input gate of the last 8 chroma-ring samples
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Rings (block RAM; the read never meets the write address)
    ---------------------------------------------------------------------------
    p_sring : process(clk)
    begin
        if rising_edge(clk) then
            sr_ram(to_integer(s_wp)) <= sr_wd;
            sr_q <= sr_ram(to_integer(sr_ra));
            sr_q2 <= sr_q;
        end if;
    end process p_sring;

    p_cring : process(clk)
    begin
        if rising_edge(clk) then
            cr_ram(to_integer(s_wp)) <= cr_wd;
            cr_q <= cr_ram(to_integer(cr_ra));
            cr_q2 <= cr_q;
        end if;
    end process p_cring;

    -- glyph masks: the engine writes the next line's bank while the beam reads this line's
    mk_ra(7 downto 5) <= "000";
    mk_ra(4)          <= mk_rb;
    mk_ra(3 downto 0) <= b_u(27 downto 24);
    p_mring : process(clk)
    begin
        if rising_edge(clk) then
            if mk_we = '1' then mk_ram(to_integer(mk_wa)) <= mk_wd; end if;
            mk_q <= mk_ram(to_integer(mk_ra));
            mk_q2 <= mk_q;
        end if;
    end process p_mring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_r  : unsigned(20 downto 0);
        variable v_g  : signed(16 downto 0);
        variable v_a  : signed(20 downto 0);
        variable v_z  : signed(13 downto 0);
        variable v_zm : unsigned(12 downto 0);
        variable v_h  : unsigned(12 downto 0);
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
            -- speed: |k4 - 512| - 8 (deadband +-8), sign
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
                s_shot <= not registers_in(6)(0);
                s_swap <= registers_in(6)(1);
                s_flip <= registers_in(6)(2);
                s_hard <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4 =>                                         -- SD: the centre 1440 virtual px
                        s_xh0 <= to_unsigned(480, 9); s_xd <= to_unsigned(1518, 11); s_xg <= to_unsigned(1418, 11);
                        s_hd <= '0'; s_short <= '1'; s_oxs <= to_unsigned(52, 6);
                    when 3 =>                                         -- 720p
                        s_xh0 <= (others => '0'); s_xd <= to_unsigned(1868, 11); s_xg <= to_unsigned(1898, 11);
                        s_hd <= '0'; s_short <= '1'; s_oxs <= to_unsigned(39, 6);
                    when others =>                                    -- 1080
                        s_xh0 <= (others => '0'); s_xd <= to_unsigned(1868, 11); s_xg <= to_unsigned(1898, 11);
                        s_hd <= '1'; s_short <= '0'; s_oxs <= to_unsigned(26, 6);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 18) * to_unsigned(61, 6), 8), 10);

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

            -- serial products: slot k pre-selected at zc = 40 + 32 k, loaded at +2, 18 iterations,
            -- result stored at +24.  The multiplicand may be signed (z_sx) and negated (z_ng); an
            -- accumulating slot (z_ac) adds onto the previous slot's product (mod 2^38).
            z_pre <= '0';
            if s_zone = '1' and s_zc(4 downto 0) = "01000" and s_zc >= 40 and s_zc <= 840 then
                z_sel <= resize(shift_right(s_zc - 40, 5), 5);
                z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_sx <= '0'; z_ng <= '0'; z_ac <= '0'; z_in <= '0';
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(s_g1(12 downto 3), 20); z_fb <= resize(s_g1(12 downto 3), 18);   -- k1^2
                    when 1  => z_fa <= c_k1sq;                         z_fb <= c_k1sq(19 downto 2);           -- k1^4
                    when 2  => z_fa <= c_k1sq;                         z_fb <= to_unsigned(22741, 18); z_in <= '1';   -- inc_h = 1638 +
                    when 3  => z_fa <= c_k1q;                          z_fb <= to_unsigned(28049, 18); z_ac <= '1';   --   t^2 + t^4 terms
                    when 4  => z_fa <= resize(c_inch, 20);             z_fb <= resize(s_xd, 18);              -- X phase origin
                    when 5  =>                                                                               -- spread: sh, then sv
                        z_fa <= resize(s_gp(11 downto 3), 20); z_neg <= s_gp(12);
                        if s_gp(12) = '0' then z_fb <= to_unsigned(6144, 18); else z_fb <= to_unsigned(6560, 18); end if;
                    when 6  => z_fa <= resize(s_m3, 20);               z_fb <= resize(s_m3, 18);              -- |k3|^2
                    when 7  => z_fa <= resize(s_g5(12 downto 3), 20); z_fb <= resize(s_g5(12 downto 3), 18);   -- k5^2
                    when 8  => z_fa <= c_k5sq;                         z_fb <= to_unsigned(32833, 18);         -- du_h
                    when 9  => z_fa <= resize(c_duh, 20);              z_fb <= resize(s_xg, 18);              -- glyph x origin
                    when 10 => z_fa <= unsigned(resize(s_off, 20));    z_fb <= resize(c_duh, 18); z_sx <= '1'; z_ac <= '1';   -- + slide
                    when 11 => z_fa <= resize(c_duh, 20);              z_fb <= to_unsigned(63339, 18);         -- dv per virtual line
                    when 12 => z_fa <= resize(c_dvv, 20);              z_fb <= resize(s_ys, 18);              -- dv per line
                    when 13 => z_fa <= resize(c_dvv, 20);              z_fb <= to_unsigned(136115, 18) - resize(s_ys(11 downto 1), 18);  -- glyph y origin
                    when 14 => z_fa <= resize(s_vm, 20);               z_fb <= to_unsigned(141, 18);           -- drift step
                    when 15 => z_fa <= resize(c_is, 20);               z_fb <= to_unsigned(54917, 18);         -- sh / (A_R sv)
                    when 16 => z_fa <= resize(c_is, 20);               z_fb <= to_unsigned(53722, 18);         -- sh / (A_L sv)
                    when 17 => z_fa <= resize(c_s, 20);                z_fb <= to_unsigned(38295, 18);         -- sh sv / BN
                    when 18 => z_fa <= resize(c_sh, 20);               z_fb <= to_unsigned(3663, 18);          -- 0.894 sh
                    when 19 => z_fa <= resize(c_k3sq, 20);             z_fb <= to_unsigned(18568, 18);         -- flare C
                    when 20 => z_fa <= resize(s_n2, 20);               z_fb <= to_unsigned(564, 18); z_ng <= not s_tsg;   -- taper T
                    -- band DDA start (Q24 lines): F0 = C u0^2 - T |x0 - 960| ...
                    when 21 =>
                        z_fa <= resize(c_C, 20);
                        if s_xs = 4 then
                            if s_face = '1' then z_fb <= to_unsigned(2437, 18); else z_fb <= to_unsigned(167587, 18); end if;
                        else
                            if s_face = '1' then z_fb <= to_unsigned(112, 18); else z_fb <= to_unsigned(220312, 18); end if;
                        end if;
                    when 22 =>
                        z_fa <= unsigned(resize(c_T, 20)); z_sx <= '1'; z_ng <= '1'; z_ac <= '1';
                        if s_xs = 4 then z_fb <= to_unsigned(23040, 18); else z_fb <= to_unsigned(30720, 18); end if;
                    -- ... G0 = +-C |2 u0 s + s^2| + 4 T s (Q30) ...
                    when 23 =>
                        z_fa <= resize(c_C, 20);
                        case to_integer(s_xs) is
                            when 4 =>
                                if s_face = '1' then z_fb <= to_unsigned(3176, 18); else z_fb <= to_unsigned(26184, 18); z_ng <= '1'; end if;
                            when 3 =>
                                z_ng <= '1';
                                if s_face = '1' then z_fb <= to_unsigned(501, 18); else z_fb <= to_unsigned(22521, 18); end if;
                            when others =>
                                z_ng <= '1';
                                if s_face = '1' then z_fb <= to_unsigned(336, 18); else z_fb <= to_unsigned(15016, 18); end if;
                        end case;
                    when 24 =>
                        z_fa <= unsigned(resize(c_T, 20)); z_sx <= '1'; z_ac <= '1';
                        case to_integer(s_xs) is
                            when 4      => z_fb <= to_unsigned(4096, 18);
                            when 3      => z_fb <= to_unsigned(3072, 18);
                            when others => z_fb <= to_unsigned(2048, 18);
                        end case;
                    -- ... H = 2 C s^2 (Q30)
                    when others =>
                        z_fa <= resize(c_C, 20);
                        case to_integer(s_xs) is
                            when 4      => z_fb <= to_unsigned(32, 18);
                            when 3      => z_fb <= to_unsigned(18, 18);
                            when others => z_fb <= to_unsigned(8, 18);
                        end case;
                end case;
            end if;
            if z_ld = '1' then
                if z_ac = '0' then
                    if z_in = '1' then f_acc <= shift_left(to_unsigned(1638, 38), 20); else f_acc <= (others => '0'); end if;
                end if;
                v_a := signed(resize(z_fa, 21));
                if z_sx = '1' then v_a := resize(signed(z_fa), 21); end if;
                if z_ng = '1' then v_a := -v_a; end if;
                f_a <= unsigned(resize(v_a, 38)); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(36 downto 0) & '0';
                f_b <= '0' & f_b(17 downto 1);
            end if;

            -- serial divider (restoring, one dividend bit per clock, 35 clocks)
            d_ld <= '0';
            if d_ld = '1' then
                d_rem <= (others => '0'); d_q <= (others => '0'); d_i <= (others => '0'); d_run <= '1';
            elsif d_run = '1' then
                v_r := d_rem(19 downto 0) & d_num(34);
                if v_r >= ('0' & d_den) then
                    d_rem <= v_r - ('0' & d_den); d_q <= d_q(23 downto 0) & '1';
                else
                    d_rem <= v_r; d_q <= d_q(23 downto 0) & '0';
                end if;
                d_num <= d_num(33 downto 0) & '0';
                if d_i = 34 then d_run <= '0'; end if;
                d_i <= d_i + 1;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's step)
                    when 20 =>
                        if s_vneg = '1' then c_dr <= c_dr - resize(c_drs, 24); else c_dr <= c_dr + resize(c_drs, 24); end if;
                        c_g6 <= s_g6(12 downto 5);
                    -- K4 glyph slide: glide 1/32 of the gap per field (+-2 deadband), then |offset| and sign
                    when 21 =>
                        s_g4d <= signed(shift_left(resize(s_k4, 17), 6)) - signed(resize(s_g4, 17));
                    when 22 =>
                        if s_g4d > 128 or s_g4d < -128 then s_g4d <= shift_right(s_g4d, 5); else s_g4d <= (others => '0'); end if;
                    when 23 =>
                        v_g := signed(resize(s_g4, 17)) + s_g4d;
                        s_g4 <= unsigned(v_g(15 downto 0));
                    when 24 =>                                    -- home (the photo) at the default 600
                        s_off <= signed(resize(s_g4(15 downto 4), 13)) - 2400;
                    -- K3, K2: side and magnitude |k - 511.5| - 0.5 (0 within +-8); K5: distance from the
                    -- photo's size (Q3, 0 within +-4 knob steps)
                    when 30 =>
                        s_face <= s_g3(12);
                        if s_g3(12) = '1' then v_m := resize(s_g3(11 downto 3), 10); else v_m := resize(not s_g3(11 downto 3), 10); end if;
                        if v_m(8 downto 3) = 0 then s_m3 <= (others => '0'); else s_m3 <= v_m(8 downto 0); end if;
                    when 31 =>
                        s_tsg <= s_g2(12);
                        if s_g2(12) = '1' then v_m := resize(s_g2(11 downto 3), 10); else v_m := resize(not s_g2(11 downto 3), 10); end if;
                        if v_m(8 downto 3) = 0 then s_n2 <= (others => '0'); else s_n2 <= v_m(8 downto 0); end if;
                    when 32 =>
                        v_z := signed(resize(s_g5, 14)) - 4968;
                        s_zside <= not v_z(13);
                        if v_z(13) = '1' then v_z := not v_z; end if;
                        v_zm := unsigned(v_z(12 downto 0));
                        if v_zm(12 downto 5) = 0 then s_dz <= (others => '0'); else s_dz <= v_zm; end if;
                    -- products
                    when 64  => c_k1sq <= f_acc(19 downto 0);
                    when 96  => c_k1q  <= f_acc(37 downto 18);
                    when 160 => c_inch <= f_acc(35 downto 20);
                    when 161 =>
                        case to_integer(s_xs) is
                            when 4      => c_incp <= shift_left(resize(c_inch, 18), 2);
                            when 3      => c_incp <= shift_left(resize(c_inch, 18), 1) + resize(c_inch, 18);
                            when others => c_incp <= shift_left(resize(c_inch, 18), 1);
                        end case;
                    when 192 => c_po <= f_acc(23 downto 0);
                    when 224 =>
                        s_up <= z_neg;
                        if z_neg = '1' then
                            c_s <= to_unsigned(4096, 13) + f_acc(22 downto 10); c_sh <= to_unsigned(4096, 13);
                        else
                            c_s <= to_unsigned(4096, 13); c_sh <= to_unsigned(1024, 13) + f_acc(22 downto 10);
                        end if;
                    -- divider: 1 / sv (sv ready at 224); below 50 % sv = 1, so sh / sv = sh and sh sv = sh
                    when 232 =>
                        d_num <= to_unsigned(2 ** 28, 35); d_den <= resize(c_s, 20); d_ld <= '1';
                    when 256 => c_k3sq <= f_acc(17 downto 0);
                    when 280 =>
                        if s_up = '1' then c_is <= d_q(17 downto 0);
                        else c_is <= shift_left(resize(c_sh, 18), 4); c_s <= c_sh; end if;
                    when 288 => c_k5sq <= f_acc(19 downto 0);
                    when 320 => c_duh <= to_unsigned(6554, 16) + f_acc(35 downto 20);
                    when 321 =>
                        case to_integer(s_xs) is
                            when 4      => c_dup <= shift_left(resize(c_duh, 18), 2);
                            when 3      => c_dup <= shift_left(resize(c_duh, 18), 1) + resize(c_duh, 18);
                            when others => c_dup <= shift_left(resize(c_duh, 18), 1);
                        end case;
                    when 384 => c_u0  <= to_unsigned(C_UANC, 28) - f_acc(27 downto 0);
                    when 416 => c_dvv <= f_acc(31 downto 14);
                    when 448 => c_dvl <= f_acc(27 downto 8);
                    when 480 => c_v0  <= to_unsigned(C_VANC, 28) - f_acc(35 downto 8);
                    when 512 => c_drs <= f_acc(16 downto 0);
                    when 544 => c_iar <= f_acc(33 downto 16);
                    when 576 => c_ial <= f_acc(33 downto 16);
                    when 608 => c_sb  <= f_acc(28 downto 13);
                    when 640 =>
                        v_h := to_unsigned(4096, 13) - resize(f_acc(23 downto 12), 13);
                        c_h0 <= v_h(11 downto 0);
                    when 672 => c_C <= f_acc(32 downto 16);
                    when 704 => c_T <= signed(f_acc(19 downto 0));
                    when 768 => c_F0 <= signed(f_acc(35 downto 2));
                    when 832 => c_G0 <= signed(f_acc(33 downto 8));
                    when 864 => c_H  <= f_acc(21 downto 8);
                    -- X phase origin
                    when 880 =>
                        c_phi0 <= to_unsigned(2 ** 23, 24) - c_dr - c_po;
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
    -- Source side: oscillator lattice (two triangles, drifting)
    ---------------------------------------------------------------------------
    p_src : process(clk)
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + resize(s_oxs, 16);
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                    -- ot * 128
                s_oy <= s_oy + resize(s_oys, 16);
            end if;
            if s_zone = '1' then s_oy <= to_unsigned(0, 16) - shift_left(resize(s_ot, 16), 6); end if;   -- -ot * 64
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            s_osum <= resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line's wedge thresholds, band offset and glyph row on one
    -- serial multiplier (18 iterations, result at load + 19); the glyph mask
    -- engine then writes that line's 16 column masks into the free bank
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d  : signed(20 downto 0);
        variable v_t  : unsigned(20 downto 0);
        variable v_k  : unsigned(15 downto 0);
        variable v_x  : unsigned(15 downto 0);
        variable v_gs : unsigned(13 downto 0);
        variable v_pb : std_logic_vector(7 downto 0);
        variable v_bt : std_logic;
        variable v_mk : std_logic_vector(7 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 900 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= resize(s_ys(11 downto 1), 20);
                e_vg <= c_v0;
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 20);
                e_vg <= e_vg + resize(c_dvl, 28);
                e_lc <= (others => '0');
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(39 downto 0) & '0';
            m_b <= '0' & m_b(17 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    v_d := signed(resize(e_vy, 21)) - C_CY8;
                    if v_d < 0 then v_d := -v_d; end if;
                    e_ty <= unsigned(v_d(17 downto 0));
                    v_d := signed(resize(e_vy, 21)) - C_CYZ8;
                    if v_d < 0 then v_d := -v_d; end if;
                    e_tz <= unsigned(v_d(17 downto 0));
                    if e_vg(23 downto 12) < 510 then e_row <= "00";
                    elsif e_vg(23 downto 12) < 1021 then e_row <= "01";
                    elsif e_vg(23 downto 12) < 1540 then e_row <= "10";
                    else e_row <= "11"; end if;
                    e_rowc <= e_vg(27 downto 24);
                when 2 =>
                    e_dz <= signed(resize(e_tz, 19)) - C_Z08;
                    if e_ty < C_V08 then e_tvz <= '1'; else e_tvz <= '0'; end if;
                    m_a <= resize(c_iar, 41); m_b <= e_ty; m_acc <= (others => '0');
                when 3 =>
                    n_d <= e_dz;
                when 21 =>
                    v_t := m_acc(40 downto 20) + c_h0;
                    if v_t(20 downto 12) /= 0 then n_thr <= to_unsigned(4095, 12); else n_thr <= v_t(11 downto 0); end if;
                    m_a <= resize(c_ial, 41); m_b <= e_ty; m_acc <= (others => '0');
                when 40 =>
                    v_t := m_acc(40 downto 20) + c_h0;
                    if v_t(20 downto 12) /= 0 then n_thl <= to_unsigned(4095, 12); else n_thl <= v_t(11 downto 0); end if;
                    m_a <= resize(c_sb, 41); m_acc <= (others => '0');
                    if e_tvz = '1' then m_b <= (others => '0'); else m_b <= e_ty - C_V08; end if;
                when 59 =>
                    v_t := m_acc(40 downto 20);
                    if e_tvz = '1' then n_tv <= (others => '0');
                    elsif v_t > 4095 then n_tv <= to_unsigned(4095, 12);
                    else n_tv <= v_t(11 downto 0); end if;
                when others => null;
            end case;

            -- glyph mask engine.  Key = column, cell row, sub-row, block, zoom side.  Stage 1-3: hash
            -- h; stage 4: generation g = (dz + h(7..0)) / 256 (0 at the photo's size, a fresh roll
            -- every 32 knob units, staggered per block); stage 5-7: a second hash of h and g, whose
            -- bit 13 re-rolls the block once g > 0.  Bit = photo bit xor re-roll; the gap row is empty.
            if e_lc = 2 then
                k_run <= '1'; k_j <= (others => '0'); k_row <= e_row; k_rowc <= e_rowc;
            elsif k_run = '1' then
                if k_j = 127 then k_run <= '0'; end if;
                k_j <= k_j + 1;
            end if;
            k_vp(1) <= k_run;
            for i in 2 to 6 loop k_vp(i) <= k_vp(i - 1); end loop;
            -- 1
            v_k := "00" & s_zside & k_j(2 downto 0) & k_row & k_rowc & k_j(6 downto 3);
            k_a <= (v_k xor x"5A3C") + rotate_left(v_k, 7);
            -- 2
            k_b <= (k_a xor shift_right(k_a, 5)) + x"3B9D";
            -- 3
            v_x := k_b + rotate_left(k_b, 11);
            k_h <= v_x xor shift_right(v_x, 7);
            -- 4
            v_gs := resize(s_dz, 14) + resize(k_h(7 downto 0), 14);
            k_g <= v_gs(13 downto 8);
            k_x <= k_h xor shift_left(resize(v_gs(13 downto 8), 16), 10) xor shift_left(resize(v_gs(13 downto 8), 16), 3);
            -- 5
            k_y <= k_x + rotate_left(k_x, 7);
            if k_g /= 0 then k_gz5 <= '1'; else k_gz5 <= '0'; end if;
            -- 6
            k_z <= (k_y xor shift_right(k_y, 6)) + x"6D2B";
            k_gz6 <= k_gz5;
            -- 7: bit, shift into the mask, write after block 7
            v_x := k_z + rotate_left(k_z, 3);
            case k_row is
                when "00"   => v_pb := x"E7";
                when "01"   => v_pb := x"F8";
                when others => v_pb := x"FF";
            end case;
            v_bt := (v_pb(to_integer(k_wj(2 downto 0))) xor (k_gz6 and v_x(13))) and not (k_row(1) and k_row(0));
            mk_we <= '0';
            if e_lc = 2 then k_wj <= (others => '0'); end if;
            if k_vp(6) = '1' then
                k_wj <= k_wj + 1;
                v_mk := v_bt & k_m;
                k_m <= v_mk(7 downto 1);
                if k_wj(2 downto 0) = "111" then
                    mk_we <= '1';
                    mk_wa <= "000" & mk_wb & k_wj(6 downto 3);
                    if s_flip = '1' then
                        for i in 0 to 7 loop mk_wd(i) <= v_mk(7 - i); end loop;
                    else
                        mk_wd <= v_mk;
                    end if;
                end if;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: X phase (held left of the line-start hold point), band quadratic
    -- F(x) (F += G; G += H), glyph phase (cell index . Q24); bank swap
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_xh  <= resize(s_xh0, 12);
                b_x1  <= resize(s_xh0, 12) + s_xs;
                b_phi <= c_phi0;
                b_u   <= c_u0;
                b_F   <= c_F0;
                b_G   <= c_G0;
                l_thr <= n_thr; l_thl <= n_thl; l_tv <= n_tv; l_d <= n_d;
                mk_rb <= mk_wb; mk_wb <= not mk_wb;
            elsif g_avid = '1' then
                b_xh <= b_x1;
                b_x1 <= b_x1 + s_xs;
                if b_x1 > C_XH then b_phi <= b_phi + c_incp; end if;
                b_F <= b_F + resize(b_G, 34);
                b_G <= b_G + signed(resize(c_H, 26));
                b_u <= b_u + resize(c_dup, 28);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage k register = the beam pixel of k clocks before)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_d   : unsigned(9 downto 0);
        variable v_l   : unsigned(9 downto 0);
        variable v_os  : signed(9 downto 0);
        variable v_ur  : signed(12 downto 0);
        variable v_25  : signed(17 downto 0);
        variable v_bi  : unsigned(2 downto 0);
        variable v_nr, v_nl, v_ok : std_logic;
        variable v_th  : unsigned(11 downto 0);
        variable v_k   : std_logic;
        variable v_c   : unsigned(1 downto 0);
        variable v_g   : unsigned(11 downto 0);
        variable v_n   : unsigned(7 downto 0);
        variable v_gs  : signed(9 downto 0);
        variable v_cnt : unsigned(7 downto 0);
        variable v_sg  : unsigned(6 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: video tap (luma -> 0..255, a few px early: invisible in a bend); band compare
            --    F(x) > |y - axis| - 178; glyph x within the cell - 64 (+- dilation)
            if unsigned(data_in.y) > 64 then v_d := unsigned(data_in.y) - 64; else v_d := (others => '0'); end if;
            v_l := shift_right(v_d, 2) + shift_right(v_d, 4);
            if v_l > 255 then p1_l8 <= (others => '1'); else p1_l8 <= v_l(7 downto 0); end if;
            p1_os <= signed(resize(s_osum, 9)) - 127;
            p1_phi <= b_phi;
            if b_F(33 downto 14) > resize(l_d, 20) then p1_zs <= '1'; else p1_zs <= '0'; end if;
            v_ur := signed(resize(b_u(23 downto 12), 13));
            p1_v0 <= v_ur - 64;

            -- 2: source (bend input, grey target); glyph block indices (x 25 / 8192: 8 blocks of
            --    327.7 / 4096 of the pitch); band
            if s_src = '0' then
                p2_bin <= signed(resize(p1_l8, 10));
                p2_src <= to_unsigned(40, 8) + resize(p1_l8(7 downto 1), 8) + resize(p1_l8(7 downto 3), 8) + resize(p1_l8(7 downto 4), 8);
            else
                v_os := resize(p1_os, 10);
                p2_bin <= shift_right(v_os, 2);
                v_gs := to_signed(158, 10) + shift_right(v_os, 2);
                p2_src <= unsigned(v_gs(7 downto 0));
            end if;
            -- block n = (x - 64) 25 (8192 per block); the +- dilation (35 / 4096 = 875 / 8192 of a block)
            -- lands in the next / previous block within 875 of a block edge
            v_25 := shift_left(resize(p1_v0, 18), 4) + shift_left(resize(p1_v0, 18), 3) + resize(p1_v0, 18);
            v_bi := unsigned(v_25(15 downto 13));
            if unsigned(v_25(12 downto 0)) >= 8192 - C_DIL * 25 then v_nr := '1'; else v_nr := '0'; end if;
            if unsigned(v_25(12 downto 0)) < C_DIL * 25 then v_nl := '1'; else v_nl := '0'; end if;
            v_ok := not (v_25(17) or v_25(16));
            p2_i0 <= v_bi; p2_k0 <= v_ok;
            if v_nr = '1' then p2_ip <= v_bi + 1; else p2_ip <= v_bi; end if;
            if v_nl = '1' then p2_im <= v_bi - 1; else p2_im <= v_bi; end if;
            if (v_ok = '1' and not (v_nr = '1' and v_bi = 7)) or (v_25(17 downto 13) = "11111" and v_nr = '1') then
                p2_kp <= '1'; else p2_kp <= '0'; end if;
            if (v_ok = '1' and not (v_nl = '1' and v_bi = 0)) or (v_25(17 downto 13) = "01000" and v_nl = '1') then
                p2_km <= '1'; else p2_km <= '0'; end if;
            p2_mid <= p1_zs;
            p2_phi <= p1_phi;

            -- 3: glyph masks (this cell's 8-block mask from the bank): plain, dilated, eroded
            p3_gw <= p2_k0 and mk_q2(to_integer(p2_i0));
            p3_gy <= (p2_kp and mk_q2(to_integer(p2_ip))) or (p2_km and mk_q2(to_integer(p2_im)));
            p3_gc <= (p2_kp and mk_q2(to_integer(p2_ip))) and (p2_km and mk_q2(to_integer(p2_im)));
            -- 3: bend partial products (bend input x gain nibbles)
            p3_pl <= resize(p2_bin * signed('0' & c_g6(3 downto 0)), 15);
            p3_ph <= resize(p2_bin * signed('0' & c_g6(7 downto 4)), 15);
            p3_phi <= p2_phi;
            -- 4: bend product
            p4_bp <= resize(p3_pl, 19) + shift_left(resize(p3_ph, 19), 4);
            p4_phi <= p3_phi;
            -- 5: bent X phase
            p5_phib <= p4_phi + unsigned(shift_left(resize(p4_bp, 24), 6));
            -- 6: fold: Txn = 0 at the X centres, 1 midway; right of a centre = first half
            if p5_phib(23) = '1' then p6_txn <= not p5_phib(22 downto 11); p6_rt <= '0';
            else p6_txn <= p5_phib(22 downto 11); p6_rt <= '1'; end if;
            -- 7: wedge key
            if p6_rt = '1' then v_th := l_thr; else v_th := l_thl; end if;
            if p6_txn > v_th or p6_txn < l_tv then p7_k <= '1'; else p7_k <= '0'; end if;

            -- delay lines for source and masks
            p_sv(3) <= p2_src;
            for i in 4 to 8 loop p_sv(i) <= p_sv(i - 1); end loop;
            p_mid(3) <= p2_mid;
            p_gw(4) <= p3_gw; p_gy(4) <= p3_gy; p_gc(4) <= p3_gc;
            for i in 4 to 7 loop p_mid(i) <= p_mid(i - 1); end loop;
            for i in 5 to 7 loop
                p_gw(i) <= p_gw(i - 1); p_gy(i) <= p_gy(i - 1); p_gc(i) <= p_gc(i - 1);
            end loop;

            -- 8: ground class (0 white, 1 yellow, 2 cyan; swap), ink flags; slew input
            if p_mid(7) = '1' then
                if p7_k = '1' then v_c := "10"; else v_c := "00"; end if;
            else
                if p7_k = '1' then v_c := "00"; else v_c := "01"; end if;
            end if;
            if s_swap = '1' and v_c /= "00" then v_c := not v_c; end if;
            p8_cls <= v_c;
            if v_c = "00" and p_gw(7) = '1' and s_gav_sr(6) = '1' then p8_wg <= '1'; else p8_wg <= '0'; end if;
            if v_c = "01" and p_gy(7) = '1' then p8_red <= '1'; else p8_red <= '0'; end if;
            if v_c = "10" and p_gc(7) = '1' then p8_blu <= '1'; else p8_blu <= '0'; end if;
            if s_hd = '1' then p8_sa <= resize(p_sv(7)(7 downto 1), 12);  -- src * 16 >> 5
            else p8_sa <= resize(p_sv(7), 12); end if;                   -- src * 16 >> 4

            -- 9: one-shot (black for 66 h from each white-glyph run start) and the RC slew
            p9_cls <= p8_cls; p9_red <= p8_red; p9_blu <= p8_blu; p9_wg <= p8_wg;
            p9_g <= x_g(11 downto 4);
            if p8_wg = '1' and s_shot = '1' and x_cnt < 66 then p9_blk <= '1'; else p9_blk <= '0'; end if;
            if p8_wg = '0' then
                x_cnt <= (others => '0');
                if s_shot = '1' then x_g <= to_unsigned(1520, 12); else x_g <= shift_left(resize(p_sv(8), 12), 4); end if;
            else
                v_cnt := resize(x_cnt, 8) + s_xs;
                if v_cnt > 127 then x_cnt <= (others => '1'); else x_cnt <= v_cnt(6 downto 0); end if;
                if s_shot = '1' and x_cnt < 66 then
                    x_g <= to_unsigned(1520, 12);
                else
                    if s_hd = '1' then v_g := x_g - shift_right(x_g, 5); else v_g := x_g - shift_right(x_g, 4); end if;
                    x_g <= v_g + p8_sa;
                end if;
            end if;
            -- streak noise: a new value every 64 h (32 virtual px)
            v_sg := x_seg + s_xs;
            x_seg <= '0' & v_sg(5 downto 0);
            if v_sg(6) = '1' then
                x_lf <= x_lf(14 downto 0) & (x_lf(15) xor x_lf(13) xor x_lf(12) xor x_lf(10));
                x_nz <= signed('0' & x_lf(2 downto 0)) - 4;
            end if;

            -- 10: colour code (0 W, 1 Y, 2 C, 3 R, 4 B, 5 K, 6 grey); grey + noise
            if p9_blk = '1' then p10_code <= "101";
            elsif p9_wg = '1' then p10_code <= "110";
            elsif p9_red = '1' then p10_code <= "011";
            elsif p9_blu = '1' then p10_code <= "100";
            else p10_code <= resize(p9_cls, 3); end if;
            v_gs := signed(resize(p9_g, 10)) + resize(x_nz, 10);
            if v_gs < 0 then p10_g <= (others => '0');
            elsif v_gs > 205 then p10_g <= to_unsigned(205, 8);
            else p10_g <= unsigned(v_gs(7 downto 0)); end if;

            -- 11: colour {Y, U = Cr, V = Cb}; luma pre-scaled so the shell's 13/16 restores the photo
            case to_integer(p10_code) is
                when 0 => s_y11 <= to_unsigned(940, 10); s_u11 <= to_unsigned(511, 10); s_v11 <= to_unsigned(499, 10);
                when 1 => s_y11 <= to_unsigned(865, 10); s_u11 <= to_unsigned(644, 10); s_v11 <= to_unsigned(121, 10);
                when 2 => s_y11 <= to_unsigned(785, 10); s_u11 <= to_unsigned(76, 10);  s_v11 <= to_unsigned(649, 10);
                when 3 => s_y11 <= to_unsigned(397, 10); s_u11 <= to_unsigned(939, 10); s_v11 <= to_unsigned(358, 10);
                when 4 => s_y11 <= to_unsigned(405, 10); s_u11 <= to_unsigned(325, 10); s_v11 <= to_unsigned(844, 10);
                when 5 => s_y11 <= to_unsigned(170, 10); s_u11 <= to_unsigned(525, 10); s_v11 <= to_unsigned(516, 10);
                when others =>
                    s_y11 <= to_unsigned(64, 10) + shift_left(resize(p10_g, 10), 2) + resize(p10_g(7 downto 2), 10);
                    s_u11 <= to_unsigned(512, 10); s_v11 <= to_unsigned(512, 10);
            end case;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Soft box (the photo's horizontal softness): [1 2 .. 2 1] / 2N on Y, U, V,
    -- N = 16 at 1080, 8 at 720p / SD (S10 Hard bypasses it); then the output
    -- conditioning: chroma [1 2 2 2 2 2 2 2 1] / 16 (4-tap pair at 720p / SD),
    -- luma [1 2 1], luma 13/16 and the blanking gate.  Ring inputs are forced
    -- neutral outside the generated avid and the sums re-seeded in the zone.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(15 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_iy, v_iu, v_iv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_wp <= s_wp + 1;
            if s_hd = '1' then sr_ra <= s_wp - 13; else sr_ra <= s_wp - 5; end if;
            if s_short = '1' then cr_ra <= s_wp - 1; else cr_ra <= s_wp - 5; end if;

            -- 12: soft ring input
            if s_zone = '1' or s_gav_sr(10) = '0' then
                v_iy := to_unsigned(64, 10); v_iu := to_unsigned(512, 10); v_iv := to_unsigned(512, 10);
            else
                v_iy := s_y11; v_iu := s_u11; v_iv := s_v11;
            end if;
            sr_wd <= std_logic_vector(v_iy) & std_logic_vector(v_iu) & std_logic_vector(v_iv) & "00";
            s_sbh <= s_sbh(14 downto 0) & (s_gav_sr(10) and not s_zone);
            -- 13: box sums (also snapped back to neutral once 16 blank samples are in, so they resync in
            -- every blanking interval, not only once a field)
            if (s_zone = '1' and s_zc = 40) or s_sbh = x"0000" then
                if s_hd = '1' then
                    s_sy <= to_unsigned(1024, 15); s_su <= to_unsigned(8192, 15); s_sv <= to_unsigned(8192, 15);
                else
                    s_sy <= to_unsigned(512, 15); s_su <= to_unsigned(4096, 15); s_sv <= to_unsigned(4096, 15);
                end if;
            else
                s_sy <= s_sy + unsigned(sr_wd(31 downto 22)) - unsigned(sr_q2(31 downto 22));
                s_su <= s_su + unsigned(sr_wd(21 downto 12)) - unsigned(sr_q2(21 downto 12));
                s_sv <= s_sv + unsigned(sr_wd(11 downto 2)) - unsigned(sr_q2(11 downto 2));
            end if;
            s_hw1 <= sr_wd(31 downto 2);
            -- 14: box pair (or the hard bypass)
            s_syd <= s_sy; s_sud <= s_su; s_svd <= s_sv;
            if s_hard = '1' then
                s_y14 <= unsigned(s_hw1(29 downto 20)); s_u14 <= unsigned(s_hw1(19 downto 10)); s_v14 <= unsigned(s_hw1(9 downto 0));
            elsif s_hd = '1' then
                v_sc := resize(s_sy, 16) + s_syd; s_y14 <= v_sc(14 downto 5);
                v_sc := resize(s_su, 16) + s_sud; s_u14 <= v_sc(14 downto 5);
                v_sc := resize(s_sv, 16) + s_svd; s_v14 <= v_sc(14 downto 5);
            else
                v_sc := resize(s_sy, 16) + s_syd; s_y14 <= v_sc(13 downto 4);
                v_sc := resize(s_su, 16) + s_sud; s_u14 <= v_sc(13 downto 4);
                v_sc := resize(s_sv, 16) + s_svd; s_v14 <= v_sc(13 downto 4);
            end if;

            -- 15: chroma ring input; luma chain
            if s_zone = '1' or s_gav_sr(13) = '0' then
                cr_wd <= std_logic_vector(to_unsigned(512, 10)) & std_logic_vector(to_unsigned(512, 10));
                s_ly(0) <= to_unsigned(64, 10);
            else
                cr_wd <= std_logic_vector(s_u14) & std_logic_vector(s_v14);
                s_ly(0) <= s_y14;
            end if;
            s_cbh <= s_cbh(6 downto 0) & (s_gav_sr(13) and not s_zone);
            for k in 1 to 6 loop s_ly(k) <= s_ly(k - 1); end loop;
            -- 16: chroma box sums (likewise snapped back once 8 blank samples are in)
            if (s_zone = '1' and s_zc = 40) or s_cbh = "00000000" then
                if s_short = '1' then s_usum <= to_unsigned(2048, 14); s_vsum <= to_unsigned(2048, 14);
                else s_usum <= to_unsigned(4096, 14); s_vsum <= to_unsigned(4096, 14); end if;
            else
                s_usum <= s_usum + unsigned(cr_wd(19 downto 10)) - unsigned(cr_q2(19 downto 10));
                s_vsum <= s_vsum + unsigned(cr_wd(9 downto 0)) - unsigned(cr_q2(9 downto 0));
            end if;
            -- 17: chroma box pair; luma [1 2 1] centred on the same pixel
            s_ud <= s_usum; s_vd <= s_vsum;
            v_sc := resize(s_usum, 16) + s_ud;
            if s_short = '1' then s_u17 <= v_sc(12 downto 3); else s_u17 <= v_sc(13 downto 4); end if;
            v_sc := resize(s_vsum, 16) + s_vd;
            if s_short = '1' then s_v17 <= v_sc(12 downto 3); else s_v17 <= v_sc(13 downto 4); end if;
            if s_short = '1' then
                v_sy := resize(s_ly(2), 12) + shift_left(resize(s_ly(3), 12), 1) + resize(s_ly(4), 12);
            else
                v_sy := resize(s_ly(4), 12) + shift_left(resize(s_ly(5), 12), 1) + resize(s_ly(6), 12);
            end if;
            s_y17 <= v_sy(11 downto 2);
            -- 18: luma toward black (white 940 -> 777); blanking gate
            if s_y17 > 64 then v_dl := s_y17 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(17) = '1' then
                s_y18 <= s_y17 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u18 <= s_u17; s_v18 <= s_v17;
            else
                s_y18 <= to_unsigned(64, 10); s_u18 <= to_unsigned(512, 10); s_v18 <= to_unsigned(512, 10);
            end if;
            -- 19, 20
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture bunting;
