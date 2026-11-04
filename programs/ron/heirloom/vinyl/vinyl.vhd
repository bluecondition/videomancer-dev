-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Vinyl v0.1.0 -- Heirloom (phase 3, video clips), from the clip WIN_20230428_23_30_39 at 0.5 s.
--
-- The still: nested L-shaped bands -- near-vertical at the top-left, bending through a corner and
-- running out nearly flat to the right -- green and blue plateaus with a thin red fringe at every
-- crossing (a triangle colouriser: G / R / B / R), dense at the top-right, wide at the bottom.  A
-- fine hatch cuts sawtooth teeth into the blue side of every red fringe and twists the blue bands
-- into rope; in the wide blue band at the bottom fine curved grooves run like the grooves of a
-- record; a sync strip at the left edge (green, stepping to red at the bottom-left).
--
-- The patch, as fitted (vinyl.py has every constant), in virtual 1080-line coordinates:
--   F_A = G (A 2^(-f y/LA) R(x - xa + 0.379 y) - C 2^(-f y/LC)) + c0 + flow + etch * luma
--         R = softplus^0.191, a log-like horizontal shaper (ROM); A, C decay by per-line DDAs
--   F_G = gG(y) x + flowG, gG = gG0 2^(-y/LG)        the grooves (a phase accumulator per line)
--   F_H = x/62 + y/75 + flowH                        the hatch
--   u   = F_A + aG(y) saw(F_G) + aH(y) w(u0) saw(F_H) (w: the hatch acts on the blue half only)
--   colour = palette(u) x envelope (blue brighter at the top, green at the top-left)
--
--   K1  Rings    band density: G = x0.33 .. x3 (512 = the photo)
--   K2  Corner   where the bands turn: the vertical asymptote slides -517 .. +915 virtual px
--   K3  Fall     how fast the bands open going down: x0.44 .. x2.3 (512 = the photo)
--   K4  Speed    bipolar (+-8 deadband): the grooves run (the record turns), the teeth crawl, the
--                bands drift; 640 = the clip's motion
--   K5  Teeth    the hatch: 0 = clean fringes .. 380 = the photo .. 1023 = deep saw teeth
--   K6  Etch     how far the camera (or the Osc) bends the bands
--   S7  Wave     Tri (G R B R, the photo) / Saw (G R B, hard reset)
--   S8  Mirror   Off / On (the picture folded about the centre: two corners)
--   S9  Palette  Primaries (the photo) / Sepia (the clip's other look: pale green, olive, teal)
--   S10 Grooves  Low (the photo) / High (the groove family flipped: dense at the bottom, fading in
--                from the top)
--   S11 Source   Video / Osc
--   P12 MOIRE    the groove oscillator: 0 = no grooves .. 30 % = the photo's fine grooves at the
--                bottom .. 100 % = a coarse second family crossing every band in a lattice
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide); a per-line engine kicked at
-- each line start that computes the NEXT line (multiplicative decays of A, C and gG, the groove and
-- hatch amplitude ramps); a pixel path of 22 stages (R ROM, a 16 x 16 product split in two, the
-- hatch and groove products, the palette ROM, the envelope products) + output conditioning.
-- Generated avid; output conditioning (Y, U, V through two [1 2 2 2 2 2 2 2 1]/16 boxes, luma 13/16)
-- before the blanking gate; one latency (24) in every mode.  Colours in the hardware convention.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.vinyl_pkg.all;
use work.all;

architecture vinyl of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

    -- fitted constants
    constant C_A_Q16  : integer := 283912;   -- A = 4.332147, Q16
    constant C_C_Q16  : integer := 665316;   -- C = 10.151921, Q16
    constant C_KLA    : integer := 45214;    -- ln2 / LA * 2^24
    constant C_KLC    : integer := 38168;    -- ln2 / LC * 2^24
    constant C_C0     : integer := 816;      -- c0 = 0.199122, Q12
    constant C_IR0    : integer := 854462;   -- (512 - xa) / 1.5 * 4096: R index at x = 0, y = 0
    constant C_EB0    : integer := 47514;    -- blue envelope 1.45 at the top (Q1.15)
    constant C_EG0    : integer := 52429;    -- green envelope 1.6 at the top-left
    constant C_EBMIN  : integer := 18022;    -- floors 0.55 / 0.45
    constant C_EGMIN  : integer := 14746;

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
    -- x step in h (half virtual px) per pixel: 1080 = 2, 720p = 3, SD = 4 (SD shows virtual x 0 .. 1439)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- x (a + b): a = 1 or 1/2, b = 0, 1/8, 1/4, 1/2
    function f_eab(x : unsigned(9 downto 0); a : std_logic; b : unsigned(1 downto 0)) return unsigned is
        variable va, vb : unsigned(10 downto 0);
    begin
        if a = '1' then va := resize(x, 11); else va := resize(x(9 downto 1), 11); end if;
        case b is
            when "01"   => vb := resize(x(9 downto 3), 11);
            when "10"   => vb := resize(x(9 downto 2), 11);
            when "11"   => vb := resize(x(9 downto 1), 11);
            when others => vb := (others => '0');
        end case;
        return va + vb;
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
    signal r_r_a  : unsigned(10 downto 0) := (others => '0');
    signal r_r_q  : unsigned(15 downto 0) := (others => '0');
    signal r_p_a  : unsigned(8 downto 0) := (others => '0');
    signal r_p_q  : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_half : unsigned(10 downto 0) := to_unsigned(959, 11);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(380 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(200 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(307 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_wave, s_mirror, s_pal, s_gflip, s_src : std_logic := '0';

    -- per-field sequencer scratch and results
    signal z_e    : signed(12 downto 0) := (others => '0');
    signal z_neg  : std_logic := '0';
    signal z_sh   : signed(4 downto 0) := (others => '0');
    signal z_m1   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_sh1  : signed(4 downto 0) := (others => '0');
    signal z_m3   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_sh3  : signed(4 downto 0) := (others => '0');
    signal z_mlg  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_shlg : signed(4 downto 0) := (others => '0');
    signal z_tf   : unsigned(17 downto 0) := (others => '0');
    signal z_tg   : unsigned(18 downto 0) := (others => '0');
    signal z_elg  : unsigned(11 downto 0) := to_unsigned(2142, 12);
    signal z_e900 : unsigned(11 downto 0) := (others => '0');
    signal z_e1079 : unsigned(11 downto 0) := (others => '0');
    signal z_eper : unsigned(11 downto 0) := (others => '0');
    signal z_sq   : unsigned(19 downto 0) := (others => '0');
    signal z_hi   : std_logic := '0';
    signal z_dp   : unsigned(9 downto 0) := (others => '0');
    signal z_kn, z_kc : unsigned(9 downto 0) := (others => '0');
    signal z_md   : std_logic_vector(1 downto 0) := "00";
    signal z_kd   : std_logic := '0';
    signal c_a0   : unsigned(27 downto 0) := (others => '0');   -- A G, Q4.24
    signal c_cc0   : unsigned(29 downto 0) := (others => '0');   -- C G, Q6.24
    signal c_ka   : unsigned(19 downto 0) := (others => '0');   -- per-line decay of A, Q0.24
    signal c_kc   : unsigned(19 downto 0) := (others => '0');
    signal c_kg   : unsigned(19 downto 0) := (others => '0');   -- per-line change of gG, Q0.24
    signal c_gg0  : unsigned(23 downto 0) := (others => '0');   -- gG at line 0, Q0.24 cycles per virtual px
    signal c_ag   : unsigned(9 downto 0) := (others => '0');    -- groove amplitude, Q0.10
    signal c_gy0  : signed(11 downto 0) := to_signed(600, 12);  -- groove fade-in start line
    signal c_ah   : unsigned(9 downto 0) := (others => '0');    -- hatch amplitude, Q0.10
    signal c_eg   : unsigned(7 downto 0) := (others => '0');    -- etch gain (luma * eg >> 2 = 12-bit phase)
    signal c_vsq  : unsigned(17 downto 0) := (others => '0');
    signal c_dfa  : unsigned(19 downto 0) := (others => '0');   -- flow steps per field
    signal c_dfg  : unsigned(23 downto 0) := (others => '0');
    signal c_dfh  : unsigned(15 downto 0) := (others => '0');
    signal c_fa   : unsigned(23 downto 0) := (others => '0');   -- flows: band phase Q0.24, groove Q0.24, hatch Q0.16
    signal c_fg   : unsigned(23 downto 0) := (others => '0');
    signal c_fh   : unsigned(15 downto 0) := (others => '0');
    signal c_irs  : signed(24 downto 0) := to_signed(C_IR0, 25);
    signal c_dir  : unsigned(15 downto 0) := (others => '0');   -- R index per line, Q12.12
    signal c_dh   : unsigned(15 downto 0) := (others => '0');   -- hatch phase per line, Q0.16
    signal c_deb, c_deg : unsigned(9 downto 0) := (others => '0');   -- envelope per line, Q1.15
    signal c_sr   : signed(14 downto 0) := (others => '0');     -- R index per pixel, Q12.12
    signal c_shx  : unsigned(11 downto 0) := (others => '0');   -- hatch per pixel, Q0.16
    signal c_ebx, c_egx : unsigned(5 downto 0) := (others => '0');   -- envelope per pixel

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(4 downto 0) := (others => '0');
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

    -- line engine (state for the NEXT line)
    signal e_lc   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal e_zk, e_first : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_a    : unsigned(27 downto 0) := (others => '0');
    signal e_c    : unsigned(29 downto 0) := (others => '0');
    signal e_g    : unsigned(23 downto 0) := (others => '0');
    signal e_ir   : signed(24 downto 0) := (others => '0');
    signal e_hp   : unsigned(15 downto 0) := (others => '0');
    signal e_eb, e_eg : unsigned(15 downto 0) := (others => '0');
    signal e_rg, e_rh : unsigned(7 downto 0) := (others => '0');
    signal e_yv   : signed(12 downto 0) := (others => '0');
    signal m_a    : unsigned(39 downto 0) := (others => '0');
    signal m_b    : unsigned(19 downto 0) := (others => '0');
    signal m_acc  : unsigned(39 downto 0) := (others => '0');
    signal n_a16  : unsigned(15 downto 0) := (others => '0');
    signal n_coff : unsigned(11 downto 0) := (others => '0');
    signal n_gs   : unsigned(23 downto 0) := (others => '0');
    signal n_ag, n_ah : unsigned(9 downto 0) := (others => '0');
    signal n_sp   : unsigned(11 downto 0) := (others => '0');
    signal n_xsl  : unsigned(11 downto 0) := (others => '0');

    -- beam (stage 1)
    signal l_a16  : unsigned(15 downto 0) := (others => '0');
    signal l_coff : unsigned(11 downto 0) := (others => '0');
    signal l_ag, l_ah : unsigned(5 downto 0) := (others => '0');
    signal l_sp   : unsigned(11 downto 0) := (others => '0');
    signal l_xsl  : unsigned(11 downto 0) := (others => '0');
    signal l_gs   : unsigned(23 downto 0) := (others => '0');
    signal b_px   : unsigned(10 downto 0) := (others => '0');
    signal b_xv   : unsigned(11 downto 0) := (others => '0');
    signal b_rev  : std_logic := '0';
    signal b_ir   : signed(24 downto 0) := (others => '0');
    signal b_fg   : unsigned(23 downto 0) := (others => '0');
    signal b_fh   : unsigned(15 downto 0) := (others => '0');
    signal b_eg   : unsigned(15 downto 0) := (others => '0');
    signal l_eb   : unsigned(15 downto 0) := (others => '0');

    -- pixel path
    signal p2_st, p3_st, p4_st, p5_st, p6_st, p7_st : std_logic := '0';
    signal p4_r   : unsigned(15 downto 0) := (others => '0');
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p4_et  : unsigned(15 downto 0) := (others => '0');
    signal p5_lo, p5_hi : unsigned(23 downto 0) := (others => '0');
    signal p5_et  : unsigned(15 downto 0) := (others => '0');
    signal p6_pf  : unsigned(11 downto 0) := (others => '0');
    signal p6_et  : unsigned(11 downto 0) := (others => '0');
    signal p7_fa  : unsigned(11 downto 0) := (others => '0');
    signal p8_u0, p9_u0, p10_u0, p11_u0 : unsigned(11 downto 0) := (others => '0');
    signal p8_tri : unsigned(10 downto 0) := (others => '0');
    signal p9_w   : unsigned(4 downto 0) := (others => '0');
    signal p10_ahw : unsigned(5 downto 0) := (others => '0');
    signal p10_gm : unsigned(10 downto 0) := (others => '0');
    signal p10_gn, p11_gn, p11_hn : std_logic := '0';
    signal p11_gm, p11_hm : unsigned(10 downto 0) := (others => '0');
    signal p12_u  : unsigned(11 downto 0) := (others => '0');
    signal p15_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal p16_yd, p16_um, p16_vm : unsigned(9 downto 0) := (others => '0');
    signal p16_un, p16_vn, p17_un, p17_vn : std_logic := '0';
    signal p16_ea : std_logic := '1';
    signal p16_eb : unsigned(1 downto 0) := (others => '0');
    signal p17_y, p17_u, p17_v : unsigned(10 downto 0) := (others => '0');
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (an 8-sum read back from a block-RAM ring), luma [1 2 1]
    attribute no_rw_check : boolean;
    type t_cring is array (0 to 255) of std_logic_vector(15 downto 0);
    signal r_cring : t_cring := (others => x"8080");
    attribute no_rw_check of r_cring : signal is true;
    signal r_cq, r_cq2 : std_logic_vector(15 downto 0) := x"8080";
    type t_sring is array (0 to 255) of std_logic_vector(3 downto 0);
    signal r_sring : t_sring := (others => (others => '1'));
    attribute no_rw_check of r_sring : signal is true;
    signal r_sq, r_sq2 : std_logic_vector(3 downto 0) := (others => '1');
    signal r_wp : unsigned(7 downto 0) := (others => '0');
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');
    signal s_bu, s_bv, s_bud, s_bvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y20, s_y21, s_y22, s_yf22, s_y23, s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21, s_u22, s_v22, s_u23, s_v23, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
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
            r_ex_q <= C_VN_EXP2(to_integer(r_ex_a));
            r_r_q  <= C_VN_R(to_integer(r_r_a));
            r_p_q  <= C_VN_PAL(to_integer(r_p_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(12 downto 0);
        variable v_t  : unsigned(35 downto 0);
        variable v_k  : unsigned(9 downto 0);
        variable v_dk : signed(10 downto 0);
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
                s_wave   <= registers_in(6)(0);
                s_mirror <= registers_in(6)(1);
                s_pal    <= registers_in(6)(2);
                s_gflip  <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_xs     <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_half <= s_w(11 downto 1) - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters
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

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations, result stable from
            -- load + 17 (stores at load + 18 = 50 + 24 k).  The slot number is decoded at 30 + 24 k and
            -- the operands are pre-muxed into z_fa / z_fb at 31 + 24 k.
            z_pre <= '0';
            if s_zone = '1' and s_zc(2 downto 0) = "110" then
                case to_integer(s_zc(10 downto 3)) is
                    when 3  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';    -- zc 30
                    when 6  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';    -- 54
                    when 9  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';    -- 78
                    when 12 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';    -- 102
                    when 15 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';    -- 126
                    when 18 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';    -- 150
                    when 21 => z_sel <= to_unsigned(6, 5);  z_pre <= '1';    -- 174
                    when 24 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';    -- 198
                    when 27 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';    -- 222
                    when 30 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';    -- 246
                    when 33 => z_sel <= to_unsigned(10, 5); z_pre <= '1';    -- 270
                    when 36 => z_sel <= to_unsigned(11, 5); z_pre <= '1';    -- 294
                    when 39 => z_sel <= to_unsigned(12, 5); z_pre <= '1';    -- 318
                    when 42 => z_sel <= to_unsigned(13, 5); z_pre <= '1';    -- 342
                    when 45 => z_sel <= to_unsigned(14, 5); z_pre <= '1';    -- 366
                    when 48 => z_sel <= to_unsigned(15, 5); z_pre <= '1';    -- 390
                    when 51 => z_sel <= to_unsigned(16, 5); z_pre <= '1';    -- 414
                    when 54 => z_sel <= to_unsigned(17, 5); z_pre <= '1';    -- 438
                    when 57 => z_sel <= to_unsigned(18, 5); z_pre <= '1';    -- 462
                    when 60 => z_sel <= to_unsigned(19, 5); z_pre <= '1';    -- 486
                    when 63 => z_sel <= to_unsigned(20, 5); z_pre <= '1';    -- 510
                    when 66 => z_sel <= to_unsigned(21, 5); z_pre <= '1';    -- 534
                    when 69 => z_sel <= to_unsigned(22, 5); z_pre <= '1';    -- 558
                    when 72 => z_sel <= to_unsigned(23, 5); z_pre <= '1';    -- 582
                    when 75 => z_sel <= to_unsigned(24, 5); z_pre <= '1';    -- 606
                    when 78 => z_sel <= to_unsigned(25, 5); z_pre <= '1';    -- 630
                    when 81 => z_sel <= to_unsigned(26, 5); z_pre <= '1';    -- 654
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= resize(z_dp, 36);               z_fb <= to_unsigned(205, 16);     -- Rings exponent
                    when 1  => z_fa <= to_unsigned(C_A_Q16, 36);       z_fb <= z_m1;                     -- A G
                    when 2  => z_fa <= to_unsigned(C_C_Q16, 36);       z_fb <= z_m1;                     -- C G
                    when 3  => z_fa <= resize(z_dp, 36);               z_fb <= to_unsigned(154, 16);     -- Fall exponent
                    when 4  => z_fa <= resize(s_ys, 36);               z_fb <= z_m3;                     -- ys f
                    when 5  => z_fa <= resize(z_tf, 36);               z_fb <= to_unsigned(C_KLA, 16);   -- kA
                    when 6  => z_fa <= resize(z_tf, 36);               z_fb <= to_unsigned(C_KLC, 16);   -- kC
                    when 7  => z_fa <= resize(z_dp, 36);               z_fb <= to_unsigned(146, 16);     -- log2 LG
                    when 8  => z_fa <= resize(s_ys, 36);               z_fb <= z_mlg;                    -- ys / LG
                    when 9  => z_fa <= resize(z_tg, 36);               z_fb <= to_unsigned(45426, 16);   -- x ln2
                    when 10 => z_fa <= to_unsigned(230400, 36);        z_fb <= z_mlg;                    -- 900 / LG
                    when 11 => z_fa <= to_unsigned(276224, 36);        z_fb <= z_mlg;                    -- 1079 / LG
                    when 12 =>                                                                           -- log2 per900
                        z_fa <= resize(z_dp, 36);
                        if z_hi = '1' then z_fb <= to_unsigned(251, 16); else z_fb <= to_unsigned(125, 16); end if;
                    when 13 =>                                                                           -- groove amplitude
                        z_fa <= resize(z_dp, 36);
                        if z_hi = '1' then z_fb <= to_unsigned(121, 16); else z_fb <= to_unsigned(102, 16); end if;
                    when 14 => z_fa <= resize(z_dp, 36);               z_fb <= to_unsigned(322, 16);     -- groove fade start
                    when 15 =>                                                                           -- hatch amplitude
                        z_fa <= resize(z_dp, 36);
                        if z_hi = '1' then z_fb <= to_unsigned(77, 16); else z_fb <= to_unsigned(76, 16); end if;
                    when 16 =>                                                                           -- etch
                        z_fa <= resize(z_dp, 36);
                        if z_hi = '1' then z_fb <= to_unsigned(40, 16); else z_fb <= resize(z_dp, 16); end if;
                    when 17 => z_fa <= resize(z_sq, 36);               z_fb <= to_unsigned(17, 16);      -- etch k^2 scale
                    when 18 => z_fa <= resize(s_vm, 36);               z_fb <= resize(s_vm, 16);         -- speed^2
                    when 19 => z_fa <= resize(c_vsq, 36);              z_fb <= to_unsigned(18035, 16);   -- band flow
                    when 20 => z_fa <= resize(s_vm, 36);               z_fb <= to_unsigned(3329, 16);    -- groove flow
                    when 21 => z_fa <= resize(s_vm, 36);               z_fb <= to_unsigned(888, 16);     -- hatch flow
                    when 22 => z_fa <= resize(z_dp, 36);               z_fb <= to_unsigned(3823, 16);    -- corner
                    when 23 => z_fa <= resize(s_ys, 36);               z_fb <= to_unsigned(1036, 16);    -- R index / line
                    when 24 => z_fa <= resize(s_ys, 36);               z_fb <= to_unsigned(874, 16);     -- hatch / line
                    when 25 => z_fa <= resize(s_ys, 36);               z_fb <= to_unsigned(7550, 16);    -- blue envelope / line
                    when others => z_fa <= resize(s_ys, 36);           z_fb <= to_unsigned(6711, 16);    -- green envelope / line
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

            -- knob deviation: md 00 = |k - c| (sign in z_neg), 01 = k > c ? k - c : 0, 10 = k > c ? k - c : k
            v_dk := signed(resize(z_kn, 11)) - signed(resize(z_kc, 11));
            if z_kd = '1' then
                if v_dk > 0 then
                    z_dp <= unsigned(v_dk(9 downto 0)); z_neg <= '0'; z_hi <= '1';
                else
                    z_hi <= '0';
                    if v_dk < 0 then z_neg <= '1'; else z_neg <= '0'; end if;
                    case z_md is
                        when "00"   => z_dp <= unsigned(-v_dk(9 downto 0));
                        when "01"   => z_dp <= (others => '0');
                        when others => z_dp <= z_kn;
                    end case;
                end if;
            end if;
            z_kd <= '0';
            if s_zone = '1' and (s_zc = 23 or s_zc = 75 or s_zc = 146 or s_zc = 291 or s_zc = 386 or s_zc = 410 or s_zc = 531) then
                z_kd <= '1';
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's steps)
                    when 20 =>
                        if s_vneg = '1' then
                            c_fa <= c_fa - resize(c_dfa, 24); c_fg <= c_fg + c_dfg; c_fh <= c_fh - c_dfh;
                        else
                            c_fa <= c_fa + resize(c_dfa, 24); c_fg <= c_fg - c_dfg; c_fh <= c_fh + c_dfh;
                        end if;
                    when 21 =>                      -- per-pixel steps by the x step (h per pixel 2 / 3 / 4)
                        case to_integer(s_xs) is
                            when 4      => c_sr <= to_signed(5461, 15); c_shx <= to_unsigned(2114, 12); c_ebx <= to_unsigned(13, 6); c_egx <= to_unsigned(39, 6);
                            when 3      => c_sr <= to_signed(4096, 15); c_shx <= to_unsigned(1586, 12); c_ebx <= to_unsigned(10, 6); c_egx <= to_unsigned(29, 6);
                            when others => c_sr <= to_signed(2731, 15); c_shx <= to_unsigned(1057, 12); c_ebx <= to_unsigned(7, 6);  c_egx <= to_unsigned(20, 6);
                        end case;
                    -- Rings: e1 = (k1 - 512) * 0.8 (Q8 octaves)
                    when 23 => z_kn <= s_g1(12 downto 3); z_kc <= to_unsigned(512, 10); z_md <= "00";
                    when 50 =>
                        if z_neg = '1' then z_e <= -signed(resize(f_acc(17 downto 8), 13));
                        else z_e <= signed(resize(f_acc(17 downto 8), 13)); end if;
                    when 51 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh1 <= resize(z_e(12 downto 8), 5);
                    when 53 => z_m1 <= r_ex_q;
                    -- a0 = A m1 >> 7, shifted by sh1 (-2 .. +1): Q4.24
                    when 74 =>
                        v_t := f_acc;
                        case to_integer(z_sh1) is
                            when -2     => c_a0 <= resize(v_t(35 downto 9), 28);
                            when -1     => c_a0 <= resize(v_t(35 downto 8), 28);
                            when 0      => c_a0 <= v_t(34 downto 7);
                            when others => c_a0 <= v_t(33 downto 6);
                        end case;
                    -- Fall: e3 = (k3 - 512) * 0.6
                    when 75 => z_kn <= s_g3(12 downto 3); z_kc <= to_unsigned(512, 10); z_md <= "00";
                    when 98 =>
                        v_t := f_acc;
                        case to_integer(z_sh1) is
                            when -2     => c_cc0 <= resize(v_t(35 downto 9), 30);
                            when -1     => c_cc0 <= resize(v_t(35 downto 8), 30);
                            when 0      => c_cc0 <= resize(v_t(35 downto 7), 30);
                            when others => c_cc0 <= v_t(35 downto 6);
                        end case;
                    when 122 =>
                        if z_neg = '1' then z_e <= -signed(resize(f_acc(17 downto 8), 13));
                        else z_e <= signed(resize(f_acc(17 downto 8), 13)); end if;
                    when 123 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh3 <= resize(z_e(12 downto 8), 5);
                    when 125 => z_m3 <= r_ex_q;
                    -- tf = ys f in Q12 lines: ys(Q8) m3(Q15) >> 11, shifted by sh3 (-2 .. +1)
                    when 146 =>
                        v_t := f_acc;
                        case to_integer(z_sh3) is
                            when -2     => z_tf <= resize(v_t(35 downto 13), 18);
                            when -1     => z_tf <= resize(v_t(35 downto 12), 18);
                            when 0      => z_tf <= resize(v_t(35 downto 11), 18);
                            when others => z_tf <= resize(v_t(35 downto 10), 18);
                        end case;
                        -- P12 above / below the photo's 307
                        z_kn <= s_gp(12 downto 3); z_kc <= to_unsigned(307, 10); z_md <= "01";
                    when 170 => c_ka <= resize(f_acc(35 downto 12), 20);
                    when 194 => c_kc <= resize(f_acc(35 downto 12), 20);
                    -- log2 LG = 2142 + (p - 307) * 0.57 (Q8), then 2^-eLG
                    when 218 => z_elg <= to_unsigned(2142, 12) + f_acc(19 downto 8);
                    when 219 =>
                        v_e := -signed(resize(z_elg, 13));
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_shlg <= resize(v_e(12 downto 8), 5);
                    when 221 => z_mlg <= r_ex_q;
                    -- tg = ys mLG >> 8; kG = tg * ln2 >> (7 - int): int = -9 -> >> 16, -10 -> >> 17
                    when 242 => z_tg <= resize(f_acc(35 downto 8), 19);
                    when 266 =>
                        if z_shlg = -9 then c_kg <= resize(f_acc(35 downto 16), 20);
                        else c_kg <= resize(f_acc(35 downto 17), 20); end if;
                    -- 900 / LG and 1079 / LG in Q8 octaves: >> (15 - int)
                    when 290 =>
                        if z_shlg = -9 then z_e900 <= resize(f_acc(35 downto 24), 12);
                        else z_e900 <= resize(f_acc(35 downto 25), 12); end if;
                    when 291 => z_kn <= s_gp(12 downto 3); z_kc <= to_unsigned(307, 10); z_md <= "10";
                    when 314 =>
                        if z_shlg = -9 then z_e1079 <= resize(f_acc(35 downto 24), 12);
                        else z_e1079 <= resize(f_acc(35 downto 25), 12); end if;
                    when 338 =>                     -- log2 per900: 1256 + p 0.488 / 1406 + (p - 307) 0.979
                        if z_hi = '1' then z_eper <= to_unsigned(1406, 12) + f_acc(19 downto 8);
                        else z_eper <= to_unsigned(1256, 12) + f_acc(19 downto 8); end if;
                    when 339 =>                     -- log2 gG0 = 900/LG - log2 per900 (- 1079/LG flipped)
                        v_e := signed(resize(z_e900, 13)) - signed(resize(z_eper, 13));
                        if s_gflip = '1' then v_e := v_e - signed(resize(z_e1079, 13)); end if;
                        z_e <= v_e;
                    when 340 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh <= resize(z_e(12 downto 8), 5);
                    when 342 =>                     -- gG0 = m << (9 + int), int -9 .. -1 (Q0.24)
                        case to_integer(z_sh) is
                            when -1     => c_gg0 <= resize(r_ex_q & "00000000", 24);
                            when -2     => c_gg0 <= resize(r_ex_q & "0000000", 24);
                            when -3     => c_gg0 <= resize(r_ex_q & "000000", 24);
                            when -4     => c_gg0 <= resize(r_ex_q & "00000", 24);
                            when -5     => c_gg0 <= resize(r_ex_q & "0000", 24);
                            when -6     => c_gg0 <= resize(r_ex_q & "000", 24);
                            when -7     => c_gg0 <= resize(r_ex_q & "00", 24);
                            when -8     => c_gg0 <= resize(r_ex_q & "0", 24);
                            when others => c_gg0 <= resize(r_ex_q, 24);
                        end case;
                    when 362 =>                     -- groove amplitude Q0.10
                        if z_hi = '1' then c_ag <= to_unsigned(123, 10) + f_acc(17 downto 8);
                        else c_ag <= f_acc(17 downto 8); end if;
                    when 386 =>                     -- groove fade start: 600 - (p - 307) 1.257
                        if z_hi = '1' then c_gy0 <= to_signed(600, 12) - signed(resize(f_acc(19 downto 8), 12));
                        else c_gy0 <= to_signed(600, 12); end if;
                        z_kn <= s_g5(12 downto 3); z_kc <= to_unsigned(380, 10); z_md <= "10";
                    when 410 =>                     -- hatch amplitude Q0.10: 0.11 at 380, 0.30 at 1023
                        if z_hi = '1' then c_ah <= to_unsigned(113, 10) + f_acc(17 downto 8);
                        else c_ah <= f_acc(17 downto 8); end if;
                        z_kn <= s_g6(12 downto 3); z_kc <= to_unsigned(300, 10); z_md <= "10";
                    when 434 => z_sq <= f_acc(19 downto 0);
                    when 458 =>                     -- etch gain: 24 (k/300)^2 (0.35 cycles at 300), then 24 + (k - 300) 0.156
                        if z_hi = '1' then c_eg <= to_unsigned(24, 8) + z_sq(15 downto 8);
                        else c_eg <= f_acc(23 downto 16); end if;
                    when 482 => c_vsq <= f_acc(17 downto 0);
                    when 506 => c_dfa <= f_acc(35 downto 16);
                    when 530 => c_dfg <= f_acc(23 downto 0);
                    when 531 => z_kn <= s_g2(12 downto 3); z_kc <= to_unsigned(512, 10); z_md <= "00";
                    when 554 => c_dfh <= f_acc(23 downto 8);
                    when 578 =>                     -- R index at x = 0, y = 0: C_IR0 -/+ 3823 (k2 - 512)
                        if z_neg = '1' then c_irs <= to_signed(C_IR0, 25) + signed(resize(f_acc(21 downto 0), 25));
                        else c_irs <= to_signed(C_IR0, 25) - signed(resize(f_acc(21 downto 0), 25)); end if;
                    when 602 => c_dir <= f_acc(23 downto 8);
                    when 626 => c_dh  <= f_acc(23 downto 8);
                    when 650 => c_deb <= f_acc(25 downto 16);
                    when 674 => c_deg <= f_acc(25 downto 16);
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
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
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then v_l := s_osum; s_lb_wd <= std_logic_vector(v_l);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked in the zone (line 0) and at each line start, computes
    -- the NEXT line on one serial multiplier (20 iterations, result at load + 21):
    -- A <- A - A kA, C <- C - C kC, gG <- gG -/+ gG kG (multiplicative decays: exact
    -- exponentials, no per-line exp2), the groove and hatch amplitude ramps.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d  : signed(12 downto 0);
        variable v_x  : unsigned(12 downto 0);
        variable v_e  : unsigned(15 downto 0);
        variable v_c  : unsigned(11 downto 0);
        variable v_g  : unsigned(25 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 700 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_first <= '1';
                e_a <= c_a0; e_c <= c_cc0; e_g <= c_gg0;
                e_ir <= c_irs; e_hp <= c_fh;
                e_eb <= to_unsigned(C_EB0, 16); e_eg <= to_unsigned(C_EG0, 16);
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                e_first <= '0';
                e_ir <= e_ir + signed(resize(c_dir, 25));
                e_hp <= e_hp + c_dh;
                v_e := e_eb - resize(c_deb, 16);
                if v_e < C_EBMIN or v_e > e_eb then e_eb <= to_unsigned(C_EBMIN, 16); else e_eb <= v_e; end if;
                v_e := e_eg - resize(c_deg, 16);
                if v_e < C_EGMIN or v_e > e_eg then e_eg <= to_unsigned(C_EGMIN, 16); else e_eg <= v_e; end if;
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;   -- (a >> 8) kA >> 16 = a kA >> 24
            m_a <= m_a(38 downto 0) & '0';
            m_b <= '0' & m_b(19 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    m_a <= resize(e_a(27 downto 8), 40); m_b <= c_ka; m_acc <= (others => '0');
                    -- ramps: groove fades in from gy0 (down; up when flipped), hatch fades out 650 .. 850
                    if s_gflip = '1' then v_d := to_signed(1079, 13) - signed(resize(e_vy(18 downto 8), 13));
                    else v_d := signed(resize(e_vy(18 downto 8), 13)); end if;
                    e_yv <= v_d - resize(c_gy0, 13);
                    v_d := to_signed(850, 13) - signed(resize(e_vy(18 downto 8), 13));
                    if v_d < 0 then e_rh <= (others => '0');
                    elsif v_d > 255 then e_rh <= (others => '1');
                    else e_rh <= unsigned(v_d(7 downto 0)); end if;
                when 2 =>
                    if e_yv < 0 then e_rg <= (others => '0');
                    elsif e_yv > 255 then e_rg <= (others => '1');
                    else e_rg <= unsigned(e_yv(7 downto 0)); end if;
                when 22 =>
                    if e_first = '0' then e_a <= e_a - resize(m_acc(39 downto 16), 28); end if;
                    m_a <= resize(e_c(29 downto 10), 40); m_b <= c_kc; m_acc <= (others => '0');
                when 43 =>
                    if e_first = '0' then e_c <= e_c - resize(m_acc(39 downto 14), 30); end if;
                    m_a <= resize(e_g(23 downto 4), 40); m_b <= c_kg; m_acc <= (others => '0');
                when 64 =>
                    if e_first = '0' then
                        if s_gflip = '1' then e_g <= e_g + resize(m_acc(39 downto 20), 24);
                        else e_g <= e_g - resize(m_acc(39 downto 20), 24); end if;
                    end if;
                    m_a <= resize(c_ag, 40); m_b <= resize(e_rg, 20); m_acc <= (others => '0');
                when 85 =>
                    n_ag <= m_acc(17 downto 8);
                    m_a <= resize(c_ah, 40); m_b <= resize(e_rh, 20); m_acc <= (others => '0');
                when 106 =>
                    n_ah <= m_acc(17 downto 8);
                    -- the line's values
                    n_a16 <= e_a(27 downto 12);
                    v_c := to_unsigned(C_C0, 12) + c_fa(23 downto 12);
                    n_coff <= v_c - e_c(23 downto 12);
                    -- groove step per pixel = gG * xs / 2
                    v_g := resize(e_g, 26);
                    case to_integer(s_xs) is
                        when 4      => n_gs <= e_g(22 downto 0) & '0';
                        when 3      => v_g := v_g + shift_right(v_g, 1); n_gs <= v_g(23 downto 0);
                        when others => n_gs <= e_g;
                    end case;
                    n_sp <= C_VN_STRIP(to_integer(e_vy(18 downto 13)));
                    -- strip: x < 52 + max(0, y - 760) / 4  (h units: 104 + max(0, y - 760) / 2)
                    if e_vy(18 downto 8) > 760 then
                        v_x := to_unsigned(104, 13) + resize(shift_right(e_vy(18 downto 8) - 760, 1), 13);
                    else
                        v_x := to_unsigned(104, 13);
                    end if;
                    n_xsl <= v_x(11 downto 0);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: virtual x, the R-table index, the groove and hatch phases and the
    -- envelope x terms, one step per pixel; the Mirror switch reverses every step
    -- at the centre pixel (the picture folds about the screen centre)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_e : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                l_a16 <= n_a16; l_coff <= n_coff; l_ag <= n_ag(9 downto 4); l_ah <= n_ah(9 downto 4);
                l_sp <= n_sp; l_xsl <= n_xsl; l_gs <= n_gs;
                b_px <= (others => '0');
                b_xv <= (others => '0');
                b_rev <= '0';
                b_ir <= e_ir;
                b_fg <= c_fg;
                b_fh <= e_hp;
                l_eb <= e_eb; b_eg <= e_eg;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                if s_mirror = '1' and b_px = s_half then b_rev <= '1'; end if;
                -- the fold: only the bands (R index) and the strip (virtual x); the grooves, the hatch and the
                -- envelope run on
                if b_rev = '1' then
                    b_xv <= b_xv - s_xs;
                    b_ir <= b_ir - resize(c_sr, 25);
                else
                    b_xv <= b_xv + s_xs;
                    b_ir <= b_ir + resize(c_sr, 25);
                end if;
                b_fg <= b_fg + l_gs;
                b_fh <= b_fh + resize(c_shx, 16);
                if b_eg > C_EGMIN + 64 then b_eg <= b_eg - c_egx; end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_p   : unsigned(24 downto 0);
        variable v_u   : unsigned(11 downto 0);
        variable v_t   : signed(12 downto 0);
        variable v_s16 : signed(16 downto 0);
        variable v_w   : unsigned(15 downto 0);
        variable v_m5  : unsigned(4 downto 0);
        variable v_ek  : unsigned(16 downto 0);
        variable v_y   : unsigned(10 downto 0);
        variable v_k4  : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: R index (clamped), strip test, copies
            if b_ir(24) = '1' then r_r_a <= (others => '0');
            elsif b_ir(23) = '1' then r_r_a <= (others => '1');
            else r_r_a <= unsigned(b_ir(22 downto 12)); end if;
            if b_xv < l_xsl then p2_st <= '1'; else p2_st <= '0'; end if;
           
            -- 3: ROM data; luma
            p3_st <= p2_st;
            p3_lm <= unsigned(s_lb_q);
            -- 4: fabric copy of R; etch nibble products
            p4_r <= r_r_q;
            p4_st <= p3_st;
            p4_et <= p3_lm * c_eg;
            -- 5: a * R in two halves; etch sum
            p5_lo <= resize(l_a16(15 downto 2) * p4_r(8 downto 2), 24);
            p5_hi <= resize(l_a16(15 downto 2) * p4_r(15 downto 9), 24);
            p5_et <= p4_et;
            p5_st <= p4_st;
            -- 6: product mod one cycle (Q7.25 -> 12 fraction bits)
            v_p := resize(p5_lo, 25) + shift_left(resize(p5_hi, 25), 7);
            p6_pf <= v_p(20 downto 9);
            p6_et <= p5_et(13 downto 2);
            p6_st <= p5_st;
            -- 7: F_A = a R - C + c0 + flow + etch
            p7_fa <= p6_pf + l_coff + p6_et;
            p7_st <= p6_st;
            -- 8: u0 (the strip holds its own phase); triangle
            if p7_st = '1' then v_u := l_sp; else v_u := p7_fa; end if;
            p8_u0 <= v_u;
            if v_u(11) = '0' then p8_tri <= not v_u(10 downto 0); else p8_tri <= v_u(10 downto 0); end if;
            -- 9: hatch weight w = clamp((1270 - tri) / 32, 0, 16): the blue half only
            v_t := to_signed(1270, 13) - signed(resize(p8_tri, 13));
            if v_t < 0 then p9_w <= (others => '0');
            elsif v_t >= 512 then p9_w <= to_unsigned(16, 5);
            else p9_w <= resize(unsigned(v_t(8 downto 5)), 5); end if;
            p9_u0 <= p8_u0;
            -- 10: aH w (8 x 4); groove term |saw(F_G)| aG (5 x 8, sign-magnitude)
            v_w := resize(l_ah * p9_w(4 downto 1), 16);
            if p9_w(4) = '1' then p10_ahw <= l_ah; else p10_ahw <= v_w(8 downto 3); end if;
            if b_fg(23) = '1' then v_m5 := b_fg(22 downto 18); p10_gn <= '0';
            else v_m5 := not b_fg(22 downto 18); p10_gn <= '1'; end if;
            p10_gm <= resize(v_m5 * l_ag, 11);
            p10_u0 <= p9_u0;
            -- 11: hatch term |saw(F_H)| aH w
            if b_fh(15) = '1' then v_m5 := b_fh(14 downto 10); p11_hn <= '0';
            else v_m5 := not b_fh(14 downto 10); p11_hn <= '1'; end if;
            p11_hm <= resize(v_m5 * p10_ahw, 11);
            p11_gm <= p10_gm; p11_gn <= p10_gn; p11_u0 <= p10_u0;
            -- 12: u = u0 +/- hatch +/- groove (each m/64 * a/64 cycles = m a in 12-bit units)
            if p11_hn = '1' then v_s16 := -signed(resize(p11_hm, 17)); else v_s16 := signed(resize(p11_hm, 17)); end if;
            if p11_gn = '1' then v_s16 := v_s16 - signed(resize(p11_gm, 17)); else v_s16 := v_s16 + signed(resize(p11_gm, 17)); end if;
            p12_u <= p11_u0 + unsigned(v_s16(11 downto 0));
            -- 13: palette address: Tri = u / 16, Saw = u / 32 (G .. R .. B, then a hard reset)
            if s_wave = '1' then r_p_a <= s_pal & '0' & p12_u(11 downto 5);
            else r_p_a <= s_pal & p12_u(11 downto 4); end if;
            -- 14: ROM data; 15: fabric copy
            p15_q <= r_p_q;
            -- 16: offsets from black (chroma as sign and magnitude), envelope k = 8 e by channel tag
            -- (blue / green; none for Sepia) with a 2 x 2 ordered dither between the 1/8 steps
            p16_yd <= unsigned(p15_q(31 downto 22)) - 64;
            if p15_q(21) = '1' then p16_um <= '0' & unsigned(p15_q(20 downto 12)); p16_un <= '0';
            else p16_um <= '0' & (not unsigned(p15_q(20 downto 12))); p16_un <= '1'; end if;
            if p15_q(11) = '1' then p16_vm <= '0' & unsigned(p15_q(10 downto 2)); p16_vn <= '0';
            else p16_vm <= '0' & (not unsigned(p15_q(10 downto 2))); p16_vn <= '1'; end if;
            case p15_q(1 downto 0) is
                when "10"   => v_ek := resize(b_eg, 17);
                when "11"   => v_ek := resize(l_eb, 17);
                when others => v_ek := to_unsigned(32768, 17);
            end case;
            if s_pal = '1' then v_ek := to_unsigned(32768, 17); end if;
            v_ek := v_ek + shift_left(resize(unsigned'(b_px(0) & (b_px(0) xor e_vy(8))), 17), 10);
            -- k = 8 e (dithered) -> e = a + b: 4 -> 1/2, 5 -> 5/8, 6 -> 3/4, 7 -> 7/8 ~ 3/4 + 1/8 not available: 1,
            -- 8 -> 1, 9 -> 9/8, 10 -> 5/4, 11 -> 5/4, 12 and up -> 3/2
            if v_ek(16) = '1' then v_k4 := (others => '1'); else v_k4 := v_ek(15 downto 12); end if;
            case to_integer(v_k4) is
                when 0 to 4 => p16_ea <= '0'; p16_eb <= "00";
                when 5      => p16_ea <= '0'; p16_eb <= "01";
                when 6      => p16_ea <= '0'; p16_eb <= "10";
                when 7 | 8  => p16_ea <= '1'; p16_eb <= "00";
                when 9      => p16_ea <= '1'; p16_eb <= "01";
                when 10 | 11 => p16_ea <= '1'; p16_eb <= "10";
                when others => p16_ea <= '1'; p16_eb <= "11";
            end case;
            -- 17: envelope x e = x a + x b (one adder per channel)
            p17_y <= f_eab(p16_yd, p16_ea, p16_eb);
            p17_u <= f_eab(p16_um, p16_ea, p16_eb); p17_un <= p16_un;
            p17_v <= f_eab(p16_vm, p16_ea, p16_eb); p17_vn <= p16_vn;
            -- 18: compose (clamped); black outside the generated avid
            if s_gav_sr(15) = '0' then
                s_y18 <= to_unsigned(64, 10); s_u18 <= to_unsigned(512, 10); s_v18 <= to_unsigned(512, 10);
            else
                if p17_y > 955 then s_y18 <= to_unsigned(1019, 10);
                else s_y18 <= resize(p17_y(9 downto 0), 10) + 64; end if;
                v_y := p17_u;
                if v_y > 507 then v_y := to_unsigned(507, 11); end if;
                if p17_un = '1' then s_u18 <= 512 - v_y(9 downto 0); else s_u18 <= 512 + v_y(9 downto 0); end if;
                v_y := p17_v;
                if v_y > 507 then v_y := to_unsigned(507, 11); end if;
                if p17_vn = '1' then s_v18 <= 512 - v_y(9 downto 0); else s_v18 <= 512 + v_y(9 downto 0); end if;
            end if;
            -- 19: pad (one latency, a multiple of 4)
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(16);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Block-RAM rings, written every clock: the syncs (read C_LAT - 2 back + two
    -- registers = C_LAT) and the 8-bit chroma (read 6 back + two registers = the
    -- 8-clock box delay).  The read never meets the write address.
    ---------------------------------------------------------------------------
    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            r_sring(to_integer(r_wp)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
            r_sq <= r_sring(to_integer(r_wp - (C_LAT - 2)));
            r_sq2 <= r_sq;
            r_cring(to_integer(r_wp)) <= std_logic_vector(s_u19(9 downto 2)) & std_logic_vector(s_v19(9 downto 2));
            r_cq <= r_cring(to_integer(r_wp - 6));
            r_cq2 <= r_cq;
            r_wp <= r_wp + 1;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent 8-sums,
    -- luma [1 2 1] (the HD HDMI bar fix), luma 13/16 and the blanking gate.  The
    -- running sums snap back to neutral once 8 blank samples are in (the resync).
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(14 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 20: chroma 8-sums (neutral off the generated avid: the compose stage writes 512 there)
            if s_bah = "00000000" then
                s_bu <= to_unsigned(4096, 14); s_bv <= to_unsigned(4096, 14);
            else
                s_bu <= s_bu + (s_u19(9 downto 2) & "00") - (unsigned(r_cq2(15 downto 8)) & "00");
                s_bv <= s_bv + (s_v19(9 downto 2) & "00") - (unsigned(r_cq2(7 downto 0)) & "00");
            end if;
            s_y20 <= s_y19;
            -- 21
            s_bud <= s_bu; s_bvd <= s_bv;
            v_sc := resize(s_bu, 15) + s_bud; s_u21 <= v_sc(13 downto 4);
            v_sc := resize(s_bv, 15) + s_bvd; s_v21 <= v_sc(13 downto 4);
            s_y21 <= s_y20;
            -- 22: luma [1 2 1]
            s_y22 <= s_y21;
            v_sy := resize(s_y20, 12) + shift_left(resize(s_y21, 12), 1) + resize(s_y22, 12);
            s_yf22 <= v_sy(11 downto 2);
            s_u22 <= s_u21; s_v22 <= s_v21;
            -- 23: luma 13/16
            if s_yf22 > 64 then v_dl := s_yf22 - 64; else v_dl := (others => '0'); end if;
            s_y23 <= s_yf22 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u23 <= s_u22; s_v23 <= s_v22;
            -- 24: pad (one latency, a multiple of 4)
            s_y24 <= s_y23; s_u24 <= s_u23; s_v24 <= s_v23;
            -- 25: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y25 <= s_y24; s_u25 <= s_u24; s_v25 <= s_v24;
            else
                s_y25 <= to_unsigned(64, 10); s_u25 <= to_unsigned(512, 10); s_v25 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y25);
    data_out.u       <= std_logic_vector(s_u25);
    data_out.v       <= std_logic_vector(s_v25);
    data_out.hsync_n <= r_sq2(0);
    data_out.vsync_n <= r_sq2(1);
    data_out.field_n <= r_sq2(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture vinyl;
