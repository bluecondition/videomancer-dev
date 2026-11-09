-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Arcade v0.1.0 -- Heirloom (video phase), from the clip WIN_20230825_00_00_20 at 6.5 s.
--
-- The still: a row of textured "globes" (circles on one H fold, 948 px apart) with orange rings, blue
-- ring stripes arching between them (a cyan / blue weave crossing the rings), orange chevrons rising
-- between the globes from below, and two tear lines.  Ron's synth: an H triangle and a V ramp into a
-- circle (distance) generator and a colourizer with level bands; the chevrons are the H triangle summed
-- with a second V ramp.  In the clip the globe row and the chevron row rise at different rates (64 and
-- 90 px/s), each with its own tear, and the globes grow (Ron's size knob).
--
-- The patch (arcade.py has every constant; h = half virtual px):
--   u        position in the fold period P (centre circle at x = 996 px), tx = min(u, P - u)
--   dA       distance to the nearest circle centre (square by DDA, sqrt by normalize + ROM)
--   globe    dA < rd: texture tile (stripes x bands of dA)
--   ring     rd .. rd + 128 px: yellow -> red -> yellow ring, white rim, cyan halo (palette by dA)
--   arches   to rd + 417 px: dark rings every lambda, a cyan / blue weave from the far circle's
--            squared distance (palette 2-D)
--   chevrons hB = (y - yB) -+ kch |x - midline|: double orange band, navy line, then texture of hB
--   tears    each row wraps (period 1900 lines) with a bright line and a few rows of jitter
--
--   K1  Globes    fold period: 1 .. 4 globes across (512 = the still's 2.03)
--   K2  Arches    arch ring spacing 140 .. 22 px (56 = the still)
--   K3  Chevrons  chevron slope: wide flat zig-zag .. steep spires (1.2 = the still)
--   K4  Speed     bipolar: both rows rise (right) / sink (left); 700 = the clip's 64 / 90 px/s
--   K5  Texture   texture scale x2.5 .. x0.4 (fine)
--   K6  Etch      how far the camera (or the Osc) bends the texture bands and arch rings
--   S7  Globe     Texture (the still) / Rings (the arches run on into the globe)
--   S8  Chevrons  Peak (the still) / Valley (inverted: V's hanging between the globes)
--   S9  Weave     On (the still) / Off (plain deep-blue arches)
--   S10 Palette   Photo / Neon (hue turned 90 degrees: magenta rings, green arches)
--   S11 Source    Video / Osc
--   P12 GLOBE SIZE globe radius 0.35 .. 1 (50 %, the still) .. 2.0: beads in a sea of arches .. giants
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, exp2-coded gains), a per-line
-- engine kicked at each line start (computes the NEXT line: both rows' vertical terms, the tear jitter,
-- the beam start values), a beam of square DDAs, a 24-stage pixel path.  Generated avid; output
-- conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes whose 8-back taps ride block-RAM
-- rings, luma 13/16) before the blanking gate; the syncs ride a block-RAM ring too.  Colours in the
-- hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.arcade_pkg.all;
use work.all;

architecture arcade of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

    constant C_XC   : integer := 1992;        -- centre circle x (h)
    constant C_STX  : integer := 130;         -- stripe origin x (h)
    constant C_Q    : integer := 486400;      -- row period, Q8 virtual lines (1900)
    constant C_TA   : integer := 281088;      -- row A wraps 1098 lines above / 802 below the centre (Q8)
    constant C_TB   : integer := 39936;       -- row B wraps 156 lines above the chevron edge (Q8)

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

    -- line buffer: (luma >> 2) - 16, clipped at 0
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- delay rings (256 deep, read address = write pointer - (delay - 3), then a fabric register);
    -- the read never meets the write address
    type t_r32 is array (0 to 255) of std_logic_vector(31 downto 0);
    type t_r4  is array (0 to 255) of std_logic_vector(3 downto 0);
    signal rg_a, rg_b : t_r32 := (others => (others => '0'));
    signal rg_s : t_r4 := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of rg_a : signal is true;
    attribute no_rw_check of rg_b : signal is true;
    attribute no_rw_check of rg_s : signal is true;
    signal rg_wp, rg_aa, rg_ab, rg_as : unsigned(7 downto 0) := (others => '0');
    signal rg_qa, rg_qb, rg_fa, rg_fb : std_logic_vector(31 downto 0) := (others => '0');
    signal rg_qs, rg_fs : std_logic_vector(3 downto 0) := (others => '1');
    signal rg_wa, rg_wb : std_logic_vector(31 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_sqa_a : unsigned(9 downto 0) := (others => '0');
    signal r_sqa_q : std_logic_vector(10 downto 0) := (others => '0');
    signal r_ti_a  : unsigned(10 downto 0) := (others => '0');
    signal r_ti_q  : std_logic_vector(2 downto 0) := (others => '0');
    signal r_ex_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q  : std_logic_vector(15 downto 0) := (others => '0');
    signal r_sz_a  : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal r_sz_q  : std_logic_vector(11 downto 0) := (others => '0');
    signal r_pl_a  : unsigned(8 downto 0) := (others => '0');
    signal r_pl_q  : std_logic_vector(23 downto 0) := (others => '0');

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

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(520 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_rings, s_valley, s_wvoff, s_neon, s_src : std_logic := '0';

    -- per-field scratch and constants
    signal z_d2, z_d3, z_d5 : signed(10 downto 0) := (others => '0');
    signal z_m2, z_m3, z_m5 : signed(19 downto 0) := (others => '0');
    signal z_e1, z_e2, z_e3, z_e5, z_l : signed(10 downto 0) := (others => '0');
    signal z_el, z_eta, z_etb : unsigned(11 downto 0) := (others => '0');
    signal z_mt1, z_mt3, z_mt5 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_r    : signed(13 downto 0) := (others => '0');
    signal z_v2   : unsigned(19 downto 0) := (others => '0');
    signal c_n    : unsigned(7 downto 0) := to_unsigned(79, 8);
    signal c_P    : unsigned(11 downto 0) := to_unsigned(1896, 12);
    signal c_P2   : unsigned(10 downto 0) := to_unsigned(948, 11);
    signal c_PP   : unsigned(23 downto 0) := to_unsigned(1896 * 1896, 24);
    signal c_k2P  : unsigned(14 downto 0) := to_unsigned(7584, 15);
    signal c_kk   : unsigned(4 downto 0) := to_unsigned(4, 5);
    signal c_kk2  : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal c_u0   : unsigned(11 downto 0) := to_unsigned(1800, 12);
    signal c_kch  : unsigned(9 downto 0) := to_unsigned(307, 10);
    signal c_kstep : unsigned(11 downto 0) := to_unsigned(614, 12);
    signal c_KwP2 : unsigned(20 downto 0) := (others => '0');
    signal c_ml, c_mta, c_mtb : unsigned(4 downto 0) := to_unsigned(16, 5);   -- gains: mantissa << shift
    signal c_sl, c_sta, c_stb : unsigned(2 downto 0) := to_unsigned(4, 3);
    signal c_ws   : unsigned(1 downto 0) := "01";                    -- weave bits: sE(16 + ws + 2 .. 16 + ws)
    signal c_ks   : unsigned(9 downto 0) := to_unsigned(231, 10);
    signal c_kks  : unsigned(11 downto 0) := to_unsigned(462, 12);
    signal c_sx0  : unsigned(15 downto 0) := (others => '0');     -- (x0 - 130) * ks mod 2^16
    signal c_ge6  : unsigned(5 downto 0) := (others => '0');
    signal c_rd   : unsigned(11 downto 0) := to_unsigned(736, 12);
    signal c_rre, c_rbl, c_pb : unsigned(12 downto 0) := to_unsigned(992, 13);
    signal c_vA, c_vB : unsigned(11 downto 0) := (others => '0');
    signal c_yA   : unsigned(18 downto 0) := to_unsigned(64 * 256, 19);
    signal c_yB   : unsigned(18 downto 0) := to_unsigned(700 * 256, 19);
    signal z_ya, z_yb : signed(19 downto 0) := (others => '0');

    -- field serial multiplier
    signal f_a   : unsigned(27 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(27 downto 0) := (others => '0');
    signal z_fa  : unsigned(27 downto 0) := (others => '0');
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
    signal g_sd  : std_logic_vector(9 downto 0) := (others => '0');   -- g_start delayed 1..10

    -- line engine
    signal e_lc   : unsigned(6 downto 0) := to_unsigned(127, 7);
    signal e_zk   : std_logic := '0';
    signal e_yq   : unsigned(19 downto 0) := (others => '0');
    signal e_ta, e_tb : signed(20 downto 0) := (others => '0');
    signal e_pa, e_pb : signed(20 downto 0) := (others => '0');
    signal e_tc   : unsigned(3 downto 0) := (others => '1');
    signal e_tya  : signed(12 downto 0) := (others => '0');
    signal e_u    : signed(13 downto 0) := (others => '0');
    signal e_side : std_logic := '1';
    signal e_w0   : unsigned(10 downto 0) := (others => '0');
    signal e_pu   : unsigned(11 downto 0) := (others => '0');
    signal m_a    : unsigned(23 downto 0) := (others => '0');
    signal m_b    : unsigned(11 downto 0) := (others => '0');
    signal m_acc  : unsigned(23 downto 0) := (others => '0');
    signal n_tyA2 : unsigned(22 downto 0) := (others => '0');
    signal n_tyB  : signed(12 downto 0) := (others => '0');
    signal n_tear : std_logic := '0';
    signal n_u    : unsigned(11 downto 0) := (others => '0');
    signal n_v1, n_v2 : unsigned(23 downto 0) := (others => '0');
    signal n_D1   : unsigned(15 downto 0) := (others => '0');
    signal n_D2   : unsigned(15 downto 0) := (others => '0');      -- two's complement
    signal n_Kw   : unsigned(20 downto 0) := (others => '0');
    signal n_side, n_fw, n_fm : std_logic := '0';

    -- beam (stage 0)
    signal b_u    : unsigned(11 downto 0) := (others => '0');
    signal b_v1, b_v2 : unsigned(23 downto 0) := (others => '0');
    signal b_D1, b_D2 : unsigned(15 downto 0) := (others => '0');
    signal b_Kw   : unsigned(20 downto 0) := (others => '0');
    signal b_side, b_fw, b_fm : std_logic := '0';
    signal b_px   : unsigned(10 downto 0) := (others => '0');
    signal l_tyA2 : unsigned(22 downto 0) := (others => '0');
    signal l_tyB  : signed(12 downto 0) := (others => '0');
    signal l_tear : std_logic := '0';
    signal t_sgx  : unsigned(15 downto 0) := (others => '0');      -- stripe phase (stage 10)

    -- pixel path
    signal p1_sa, p1_se, p2_sa : unsigned(24 downto 0) := (others => '0');
    signal p1_hb, p2_hb, p3_hb, p4_hb, p5_hb, p6_hb : signed(14 downto 0) := (others => '0');
    signal p2_ea, p3_ea, p4_ea, p5_ea : unsigned(3 downto 0) := (others => '0');
    type t_ps is array (2 to 9) of unsigned(2 downto 0);
    signal p_ps : t_ps := (others => (others => '0'));
    signal p5_qa : unsigned(10 downto 0) := (others => '0');
    signal p6_da : unsigned(12 downto 0) := (others => '0');
    signal p7_zd, p7_zr, p7_zb, p7_zband, p7_zin : std_logic := '0';
    signal p7_ri  : unsigned(6 downto 0) := (others => '0');
    signal p7_dp  : unsigned(15 downto 0) := (others => '0');
    signal p7_f   : unsigned(12 downto 0) := (others => '0');
    signal p7_l   : unsigned(7 downto 0) := (others => '0');
    signal p7_hq  : unsigned(6 downto 0) := (others => '0');
    signal p8_ph  : unsigned(15 downto 0) := (others => '0');
    signal p8_bt  : unsigned(16 downto 0) := (others => '0');
    signal p8_ep  : unsigned(13 downto 0) := (others => '0');
    signal p8_zin : std_logic := '0';
    signal p8_zd, p8_zr, p8_zb, p8_zband, p9_zd, p9_zr, p9_zb, p9_zband : std_logic := '0';
    signal p8_hq, p9_hq, p8_ri, p9_ri : unsigned(6 downto 0) := (others => '0');
    signal p9_ph  : unsigned(15 downto 0) := (others => '0');
    signal p9_bt  : unsigned(16 downto 0) := (others => '0');
    signal p9_ep  : unsigned(13 downto 0) := (others => '0');
    signal p10_bt : unsigned(5 downto 0) := (others => '0');
    type t_pa is array (10 to 13) of unsigned(9 downto 0);   -- {tex, address}
    signal p_pa : t_pa := (others => (others => '0'));
    signal p13_cls : unsigned(2 downto 0) := (others => '0');
    signal s_y16, s_u16, s_v16 : unsigned(7 downto 0) := to_unsigned(128, 8);

    -- output conditioning
    signal s_bay : unsigned(10 downto 0) := to_unsigned(128, 11);
    signal s_bau, s_bav : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bayd, s_baud, s_bavd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y20, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y22, s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), delay rings, ROMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_rga : process(clk)
    begin
        if rising_edge(clk) then
            rg_qa <= rg_a(to_integer(rg_aa));
            rg_a(to_integer(rg_wp)) <= rg_wa;
        end if;
    end process p_rga;

    p_rgb : process(clk)
    begin
        if rising_edge(clk) then
            rg_qb <= rg_b(to_integer(rg_ab));
            rg_b(to_integer(rg_wp)) <= rg_wb;
        end if;
    end process p_rgb;

    p_rgs : process(clk)
    begin
        if rising_edge(clk) then
            rg_qs <= rg_s(to_integer(rg_as));
            rg_s(to_integer(rg_wp)) <= data_in.hsync_n & data_in.vsync_n & data_in.field_n & '1';
        end if;
    end process p_rgs;

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_sqa_q <= C_AR_SQA(to_integer(r_sqa_a));
            r_ti_q  <= C_AR_TILE(to_integer(r_ti_a));
            r_ex_q  <= C_AR_EXP2(to_integer(r_ex_a));
            r_sz_q  <= C_AR_SZ(to_integer(r_sz_a));
            r_pl_q  <= C_AR_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_t  : unsigned(27 downto 0);
        variable v_x  : unsigned(15 downto 0);
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
                s_rings  <= registers_in(6)(0);
                s_valley <= registers_in(6)(1);
                s_wvoff  <= registers_in(6)(2);
                s_neon   <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_xs     <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9);
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

            -- serial products: slot k loads at zc = 40 + 24 k, 16 iterations, result stable from
            -- load + 17 (stored at load + 18).  The slot number is decoded at 38 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 39 + 24 k.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 38  => z_sel <= to_unsigned(0, 4); z_pre <= '1';
                    when 62  => z_sel <= to_unsigned(1, 4); z_pre <= '1';
                    when 86  => z_sel <= to_unsigned(2, 4); z_pre <= '1';
                    when 110 => z_sel <= to_unsigned(3, 4); z_pre <= '1';
                    when 134 => z_sel <= to_unsigned(4, 4); z_pre <= '1';
                    when 158 => z_sel <= to_unsigned(5, 4); z_pre <= '1';
                    when 182 => z_sel <= to_unsigned(6, 4); z_pre <= '1';
                    when 206 => z_sel <= to_unsigned(7, 4); z_pre <= '1';
                    when 230 => z_sel <= to_unsigned(8, 4); z_pre <= '1';
                    when 254 => z_sel <= to_unsigned(9, 4); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(79, 28);          z_fb <= z_mt1;                       -- n
                    when 1  => z_fa <= resize(c_P, 28);               z_fb <= resize(c_P, 16);             -- P^2
                    when 2  => z_fa <= to_unsigned(307, 28);          z_fb <= z_mt3;                       -- kch
                    when 3  => z_fa <= to_unsigned(231, 28);          z_fb <= z_mt5;                       -- ks
                    when 4  => z_fa <= resize(c_kch, 28);             z_fb <= resize(c_P2, 16);            -- Kw at P/2
                    when 5  =>                                                                             -- stripe origin
                        z_fa <= resize(c_ks, 28);
                        if s_x0 = 0 then z_fb <= to_unsigned(C_STX, 16);
                        else z_fb <= resize(s_x0 - C_STX, 16); end if;
                    when 6  => z_fa <= resize(s_vm, 28);              z_fb <= resize(s_vm, 16);            -- speed^2
                    when 7  => z_fa <= resize(z_v2, 28);              z_fb <= to_unsigned(276, 16);        -- vA
                    when 8  => z_fa <= resize(s_g6(12 downto 3), 28); z_fb <= resize(s_g6(12 downto 3), 16); -- etch^2
                    when others => z_fa <= resize(z_v2(19 downto 10), 28); z_fb <= to_unsigned(1796, 16); -- etch gain
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(26 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- motion (last field's steps)
                    when 18 =>
                        if s_vneg = '1' then
                            z_ya <= signed(resize(c_yA, 20)) + signed(resize(c_vA, 20));
                            z_yb <= signed(resize(c_yB, 20)) + signed(resize(c_vB, 20));
                        else
                            z_ya <= signed(resize(c_yA, 20)) - signed(resize(c_vA, 20));
                            z_yb <= signed(resize(c_yB, 20)) - signed(resize(c_vB, 20));
                        end if;
                    when 19 =>
                        if z_ya < 0 then c_yA <= unsigned(resize(z_ya + C_Q, 19));
                        elsif z_ya >= C_Q then c_yA <= unsigned(resize(z_ya - C_Q, 19));
                        else c_yA <= unsigned(z_ya(18 downto 0)); end if;
                        if z_yb < 0 then c_yB <= unsigned(resize(z_yb + C_Q, 19));
                        elsif z_yb >= C_Q then c_yB <= unsigned(resize(z_yb - C_Q, 19));
                        else c_yB <= unsigned(z_yb(18 downto 0)); end if;

                    -- exponents (Q8 octaves), exp2 lookups, the size ROM
                    when 28 =>
                        z_d2 <= 512 - signed(resize(s_g2(12 downto 3), 11));
                        z_d3 <= signed(resize(s_g3(12 downto 3), 11)) - 512;
                        z_d5 <= signed(resize(s_g5(12 downto 3), 11)) - 512;
                        z_e1 <= shift_right(512 - signed(resize(s_g1(12 downto 3), 11)), 1);
                        r_sz_a <= s_gp(12 downto 5);
                    when 29 =>
                        z_m2 <= shift_left(resize(z_d2, 20), 7) + shift_left(resize(z_d2, 20), 5) + shift_left(resize(z_d2, 20), 3);
                        z_m3 <= shift_left(resize(z_d3, 20), 7) + shift_left(resize(z_d3, 20), 4) + shift_left(resize(z_d3, 20), 3) + shift_left(resize(z_d3, 20), 2);
                        z_m5 <= shift_left(resize(z_d5, 20), 7) + shift_left(resize(z_d5, 20), 5) + shift_left(resize(z_d5, 20), 3) + resize(z_d5, 20);
                        r_ex_a <= unsigned(z_e1(7 downto 0));
                    when 30 =>
                        z_l  <= resize(shift_right(z_m2, 8), 11);        -- lambda exponent
                        z_e2 <= -resize(shift_right(z_m2, 8), 11);
                        z_e3 <= resize(shift_right(z_m3, 8), 11);
                        z_e5 <= -resize(shift_right(z_m5, 8), 11);
                        c_rd <= unsigned(r_sz_q);
                    when 31 =>
                        z_mt1 <= unsigned(r_ex_q);
                        r_ex_a <= unsigned(z_e3(7 downto 0));
                        z_el  <= unsigned(resize(z_e2 + 2353, 12));
                        z_eta <= unsigned(resize(z_e5 + 2057, 12));
                        z_etb <= unsigned(resize(z_e5 + 2360, 12));
                        if z_l < -128 then c_ws <= "00"; elsif z_l >= 128 then c_ws <= "10"; else c_ws <= "01"; end if;
                        c_rre <= resize(c_rd, 13) + 256;
                    when 32 =>
                        r_ex_a <= unsigned(z_e5(7 downto 0));
                        c_rbl <= c_rre + 578;
                        c_pb  <= c_rre + 20;
                    when 33 =>
                        z_mt3 <= unsigned(r_ex_q);
                        r_ex_a <= z_el(7 downto 0);
                    when 34 =>
                        z_mt5 <= unsigned(r_ex_q);
                        r_ex_a <= z_eta(7 downto 0);
                    when 35 =>
                        c_ml <= unsigned(r_ex_q(15 downto 11)); c_sl <= resize(z_el(11 downto 8) - 4, 3);
                        r_ex_a <= z_etb(7 downto 0);
                    when 36 => c_mta <= unsigned(r_ex_q(15 downto 11)); c_sta <= resize(z_eta(11 downto 8) - 4, 3);
                    when 37 => c_mtb <= unsigned(r_ex_q(15 downto 11)); c_stb <= resize(z_etb(11 downto 8) - 4, 3);

                    -- n = 79 * 2^(e1/256), clamped 40 .. 158; P = 24 n
                    when 58 =>
                        case to_integer(z_e1(10 downto 8)) is
                            when 1      => v_t := shift_right(f_acc, 14);
                            when 0      => v_t := shift_right(f_acc, 15);
                            when others => v_t := shift_right(f_acc, 16);
                        end case;
                        if v_t < 40 then c_n <= to_unsigned(40, 8);
                        elsif v_t > 158 then c_n <= to_unsigned(158, 8);
                        else c_n <= v_t(7 downto 0); end if;
                    when 59 =>
                        c_P  <= shift_left(resize(c_n, 12), 4) + shift_left(resize(c_n, 12), 3);
                        c_P2 <= shift_left(resize(c_n, 11), 3) + shift_left(resize(c_n, 11), 2);
                        z_r  <= signed(resize(s_x0, 14)) - C_XC;
                    when 60 | 61 | 62 | 63 =>
                        if z_r < 0 then z_r <= z_r + signed(resize(c_P, 14)); end if;
                    when 64 =>
                        c_u0 <= unsigned(z_r(11 downto 0));
                        case to_integer(s_xs) is
                            when 4      => c_k2P <= shift_left(resize(c_P, 15), 3); c_kk <= to_unsigned(16, 5); c_kk2 <= to_unsigned(32, 6);
                            when 3      => c_k2P <= shift_left(resize(c_P, 15), 2) + shift_left(resize(c_P, 15), 1);
                                           c_kk <= to_unsigned(9, 5); c_kk2 <= to_unsigned(18, 6);
                            when others => c_k2P <= shift_left(resize(c_P, 15), 2); c_kk <= to_unsigned(4, 5); c_kk2 <= to_unsigned(8, 6);
                        end case;
                    when 82  => c_PP <= f_acc(23 downto 0);
                    when 106 | 130 =>
                        if s_zc = 106 then
                            case to_integer(z_e3(10 downto 8)) is
                                when 1      => v_t := shift_right(f_acc, 14);
                                when 0      => v_t := shift_right(f_acc, 15);
                                when -1     => v_t := shift_right(f_acc, 16);
                                when others => v_t := shift_right(f_acc, 17);
                            end case;
                            c_kch <= v_t(9 downto 0);
                        else
                            case to_integer(z_e5(10 downto 8)) is
                                when 1      => v_t := shift_right(f_acc, 14);
                                when 0      => v_t := shift_right(f_acc, 15);
                                when -1     => v_t := shift_right(f_acc, 16);
                                when others => v_t := shift_right(f_acc, 17);
                            end case;
                            c_ks <= v_t(9 downto 0);
                        end if;
                    when 107 =>
                        case to_integer(s_xs) is
                            when 4      => c_kstep <= shift_left(resize(c_kch, 12), 2);
                            when 3      => c_kstep <= shift_left(resize(c_kch, 12), 1) + resize(c_kch, 12);
                            when others => c_kstep <= shift_left(resize(c_kch, 12), 1);
                        end case;
                    when 131 =>
                        case to_integer(s_xs) is
                            when 4      => c_kks <= shift_left(resize(c_ks, 12), 2);
                            when 3      => c_kks <= shift_left(resize(c_ks, 12), 1) + resize(c_ks, 12);
                            when others => c_kks <= shift_left(resize(c_ks, 12), 1);
                        end case;
                    when 154 => c_KwP2 <= f_acc(20 downto 0);
                    when 178 =>
                        if s_x0 = 0 then c_sx0 <= (not f_acc(15 downto 0)) + 1;
                        else c_sx0 <= f_acc(15 downto 0); end if;
                    when 202 => z_v2 <= f_acc(19 downto 0);
                    when 226 =>
                        c_vA <= f_acc(26 downto 15);
                        v_x := resize(f_acc(26 downto 15), 16);
                        v_x := shift_left(v_x, 3) + shift_left(v_x, 2) + v_x;    -- * 13
                        c_vB <= f_acc(26 downto 15) + v_x(15 downto 5);
                    when 250 => z_v2 <= f_acc(19 downto 0);
                    when 274 => c_ge6 <= f_acc(20 downto 15);
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop s_gav_sr(i) <= s_gav_sr(i - 1); end loop;
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
            g_sd <= g_sd(8 downto 0) & g_start;
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
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line: both rows' wrapped vertical terms, the tear
    -- count and jitter, |tyA|^2, and the beam's start values for the jittered
    -- start u (u^2, (P - u)^2, kch w) on one serial multiplier (12 iterations,
    -- result at load + 13)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_u  : signed(13 downto 0);
        variable v_ty : signed(12 downto 0);
        variable v_k  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 400 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_yq <= (others => '0');
                e_lc <= (others => '0');
                e_pa <= to_signed(-1048575, 21); e_pb <= to_signed(-1048575, 21);
                e_tc <= (others => '1');
            elsif g_start = '1' then
                e_yq <= e_yq + resize(s_ys, 20);
                e_lc <= (others => '0');
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(22 downto 0) & '0';
            m_b <= '0' & m_b(11 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    e_ta <= signed(resize(e_yq, 21)) - signed(resize(c_yA, 21));
                    e_tb <= signed(resize(e_yq, 21)) - signed(resize(c_yB, 21));
                when 2 =>
                    if e_ta < -C_TA then e_ta <= e_ta + C_Q;
                    elsif e_ta >= C_Q - C_TA then e_ta <= e_ta - C_Q; end if;
                    if e_tb < -C_TB then e_tb <= e_tb + C_Q;
                    elsif e_tb >= C_Q - C_TB then e_tb <= e_tb - C_Q; end if;
                when 3 =>
                    -- tear: the wrapped term jumped down since the last line
                    if e_ta < e_pa or e_tb < e_pb then
                        e_tc <= (others => '0');
                    elsif e_tc /= 15 then
                        e_tc <= e_tc + 1;
                    end if;
                    e_pa <= e_ta; e_pb <= e_tb;
                    e_tya <= e_ta(19 downto 7);
                    n_tyB <= e_tb(19 downto 7);
                when 4 =>
                    n_tear <= '0';
                    v_u := signed(resize(c_u0, 14));
                    case to_integer(e_tc) is
                        when 0      => n_tear <= '1';
                        when 1 | 3  => v_u := v_u + 24;
                        when 2 | 4  => v_u := v_u - 24;
                        when others => null;
                    end case;
                    e_u <= v_u;
                    v_ty := abs(e_tya);
                    m_a <= resize(unsigned(v_ty), 24); m_b <= unsigned(v_ty(11 downto 0)); m_acc <= (others => '0');
                when 5 =>
                    if e_u < 0 then e_u <= e_u + signed(resize(c_P, 14));
                    elsif e_u >= signed(resize(c_P, 14)) then e_u <= e_u - signed(resize(c_P, 14)); end if;
                when 6 =>
                    n_u <= unsigned(e_u(11 downto 0));
                    if e_u < signed(resize(c_P2, 14)) then
                        e_side <= '1';
                        e_w0 <= resize(c_P2 - unsigned(e_u(10 downto 0)), 11);
                    else
                        e_side <= '0';
                        e_w0 <= resize(unsigned(e_u(11 downto 0)) - c_P2, 11);
                    end if;
                    e_pu <= c_P - unsigned(e_u(11 downto 0));
                    if unsigned(e_u(11 downto 0)) + s_xs = c_P then n_fw <= '1'; else n_fw <= '0'; end if;
                    if unsigned(e_u(11 downto 0)) + s_xs = c_P2 then n_fm <= '1'; else n_fm <= '0'; end if;
                when 7 =>
                    n_side <= e_side;
                    -- D1 = 2k u + k^2, D2 = k^2 - 2k (P - u)
                    case to_integer(s_xs) is
                        when 4 =>
                            n_D1 <= shift_left(resize(n_u, 16), 3) + c_kk;
                            v_k := shift_left(resize(e_pu, 16), 3);
                        when 3 =>
                            n_D1 <= shift_left(resize(n_u, 16), 2) + shift_left(resize(n_u, 16), 1) + c_kk;
                            v_k := shift_left(resize(e_pu, 16), 2) + shift_left(resize(e_pu, 16), 1);
                        when others =>
                            n_D1 <= shift_left(resize(n_u, 16), 2) + c_kk;
                            v_k := shift_left(resize(e_pu, 16), 2);
                    end case;
                    n_D2 <= resize(c_kk, 16) - v_k;
                when 17 =>
                    n_tyA2 <= m_acc(22 downto 0);
                    m_a <= resize(n_u, 24); m_b <= n_u; m_acc <= (others => '0');
                when 30 =>
                    n_v1 <= m_acc;
                    m_a <= resize(e_pu, 24); m_b <= e_pu; m_acc <= (others => '0');
                when 43 =>
                    n_v2 <= m_acc;
                    m_a <= resize(c_kch, 24); m_b <= resize(e_w0, 12); m_acc <= (others => '0');
                when 56 =>
                    n_Kw <= m_acc(20 downto 0);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 0): u, u^2 and (P - u)^2 by second-order DDAs, kch w, the
    -- fold events registered a clock ahead.  Stripe phase DDA at stage 10.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_un : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_u <= n_u; b_v1 <= n_v1; b_D1 <= n_D1; b_v2 <= n_v2; b_D2 <= n_D2;
                b_Kw <= n_Kw; b_side <= n_side; b_fw <= n_fw; b_fm <= n_fm;
                b_px <= (others => '0');
                l_tyA2 <= n_tyA2; l_tyB <= n_tyB; l_tear <= n_tear;
            else
                b_px <= b_px + 1;
                if b_fw = '1' then
                    v_un := (others => '0');
                    b_v1 <= (others => '0'); b_D1 <= resize(c_kk, 16);
                    b_v2 <= c_PP; b_D2 <= resize(c_kk, 16) - resize(c_k2P, 16);
                    b_Kw <= c_KwP2; b_side <= '1';
                else
                    v_un := b_u + s_xs;
                    b_v1 <= b_v1 + b_D1; b_D1 <= b_D1 + c_kk2;
                    b_v2 <= unsigned(signed(b_v2) + resize(signed(b_D2), 24)); b_D2 <= b_D2 + c_kk2;
                    if b_side = '1' then b_Kw <= b_Kw - c_kstep; else b_Kw <= b_Kw + c_kstep; end if;
                    if b_fm = '1' then b_side <= '0'; end if;
                end if;
                b_u <= v_un;
                if v_un + s_xs = c_P then b_fw <= '1'; else b_fw <= '0'; end if;
                if v_un + s_xs = c_P2 then b_fm <= '1'; else b_fm <= '0'; end if;
            end if;
            s_lb_ra <= b_px - 4;
            -- stripe phase: loaded at g_start + 10, so it holds pixel n at stage 10
            if g_sd(9) = '1' then t_sgx <= c_sx0; else t_sgx <= t_sgx + c_kks; end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ea  : unsigned(3 downto 0);
        variable v_q   : unsigned(18 downto 0);
        variable v_bt  : unsigned(16 downto 0);
        variable v_ph  : unsigned(15 downto 0);
        variable v_ar  : unsigned(8 downto 0);
        variable v_ri  : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: squares of both distances (near: the circle; far: the weave); the chevron field
            if b_side = '1' then
                p1_sa <= resize(b_v1, 25) + l_tyA2; p1_se <= resize(b_v2, 25) + l_tyA2;
            else
                p1_sa <= resize(b_v2, 25) + l_tyA2; p1_se <= resize(b_v1, 25) + l_tyA2;
            end if;
            if s_valley = '1' then
                p1_hb <= resize(l_tyB, 15) + signed(resize(b_Kw(20 downto 8), 15));
            else
                p1_hb <= resize(l_tyB, 15) - signed(resize(b_Kw(20 downto 8), 15));
            end if;
            -- 2: normalize (even shift so that the mantissa fits 10 bits); weave = 3 bits of eA^2
            v_ea := (others => '0');
            for i in 0 to 7 loop
                if p1_sa(24 downto 10 + 2 * i) /= 0 then v_ea := to_unsigned(i + 1, 4); end if;
            end loop;
            p2_ea <= v_ea; p2_sa <= p1_sa; p2_hb <= p1_hb;
            if s_wvoff = '1' then
                p_ps(2) <= "100";
            else
                case to_integer(c_ws) is
                    when 0      => p_ps(2) <= p1_se(18 downto 16);
                    when 2      => p_ps(2) <= p1_se(20 downto 18);
                    when others => p_ps(2) <= p1_se(19 downto 17);
                end case;
            end if;
            for i in 3 to 9 loop p_ps(i) <= p_ps(i - 1); end loop;
            -- 3: mantissa -> sqrt ROM address
            r_sqa_a <= resize(shift_right(p2_sa, 2 * to_integer(p2_ea)), 10);
            p3_ea <= p2_ea; p3_hb <= p2_hb;
            -- 4: ROM data;  5: fabric copy
            p4_ea <= p3_ea; p4_hb <= p3_hb;
            p5_qa <= unsigned(r_sqa_q); p5_ea <= p4_ea; p5_hb <= p4_hb;
            -- 6: denormalize: dA = (q << e) >> 6
            v_q := shift_left(resize(p5_qa, 19), to_integer(p5_ea));
            p6_da <= v_q(18 downto 6);
            p6_hb <= p5_hb;
            -- 7: zones and multiplier operands
            if p6_da < c_rd then p7_zd <= '1'; else p7_zd <= '0'; end if;
            if p6_da < c_rre then p7_zr <= '1'; else p7_zr <= '0'; end if;
            if p6_da < c_rbl then p7_zb <= '1'; else p7_zb <= '0'; end if;
            if p6_hb >= 0 and p6_hb < 332 then p7_zband <= '1'; else p7_zband <= '0'; end if;
            v_ri := p6_da - resize(c_rd, 13);
            p7_ri <= v_ri(7 downto 1);
            p7_dp <= resize(p6_da, 16) - resize(c_pb, 16);
            if p6_hb >= 332 and p6_hb < 1200 then
                p7_zin <= '1'; p7_f <= unsigned(p6_hb(12 downto 0));
            else
                p7_zin <= '0'; p7_f <= p6_da;
            end if;
            p7_hq <= unsigned(p6_hb(8 downto 2));
            p7_l  <= unsigned(s_lb_q);
            -- 8: products (5-bit mantissas)
            p8_ph <= resize(p7_dp * c_ml, 16);
            if p7_zin = '1' then p8_bt <= resize(p7_f * c_mtb, 17); else p8_bt <= resize(p7_f * c_mta, 17); end if;
            p8_ep <= p7_l * c_ge6;
            p8_zin <= p7_zin;
            p8_zd <= p7_zd; p8_zr <= p7_zr; p8_zb <= p7_zb; p8_zband <= p7_zband; p8_hq <= p7_hq; p8_ri <= p7_ri;
            -- 9: gain shifts
            p9_ph <= shift_left(p8_ph, to_integer(c_sl));
            if p8_zin = '1' then p9_bt <= shift_left(p8_bt, to_integer(c_stb)); else p9_bt <= shift_left(p8_bt, to_integer(c_sta)); end if;
            p9_ep <= p8_ep;
            p9_zd <= p8_zd; p9_zr <= p8_zr; p9_zb <= p8_zb; p9_zband <= p8_zband; p9_hq <= p8_hq; p9_ri <= p8_ri;
            -- 10: etch (texture phase + eps << 5, arch phase + eps << 3); the non-texture palette address
            v_bt := p9_bt + shift_left(resize(p9_ep, 17), 5);
            p10_bt <= v_bt(16) & v_bt(15 downto 11);
            v_ph := p9_ph + shift_left(resize(p9_ep, 16), 3);
            v_ar := '1' & p_ps(9) & v_ph(15 downto 11);
            if p9_zd = '1' then
                if s_rings = '1' then p_pa(10) <= '0' & v_ar; else p_pa(10) <= '1' & "000000000"; end if;
            elsif p9_zr = '1' then
                p_pa(10) <= "000" & p9_ri;
            elsif p9_zband = '1' then
                p_pa(10) <= "001" & p9_hq;
            elsif p9_zb = '1' then
                p_pa(10) <= '0' & v_ar;
            else
                p_pa(10) <= '1' & "000000000";
            end if;
            for i in 11 to 13 loop p_pa(i) <= p_pa(i - 1); end loop;
            -- 11: tile ROM address {parity, sigma5, beta5}
            r_ti_a <= p10_bt(5) & t_sgx(15 downto 11) & p10_bt(4 downto 0);
            -- 12: tile ROM data;  13: fabric copy -> class
            p13_cls <= unsigned(r_ti_q);
            -- 14: palette address (tear line > texture class > zone address)
            if l_tear = '1' then
                r_pl_a <= to_unsigned(232, 9);
            elsif p_pa(13)(9) = '1' then
                r_pl_a <= "011100" & p13_cls;
            else
                r_pl_a <= p_pa(13)(8 downto 0);
            end if;
            -- 15: palette ROM data;  16: compose (8-bit; black outside the generated avid; Neon turns the
            -- hue 90 degrees: Cr' = -Cb, Cb' = Cr)
            if s_gav_sr(14) = '0' then
                s_y16 <= to_unsigned(16, 8); s_u16 <= to_unsigned(128, 8); s_v16 <= to_unsigned(128, 8);
            elsif s_neon = '1' then
                s_y16 <= unsigned(r_pl_q(23 downto 16)); s_u16 <= unsigned(not r_pl_q(7 downto 0)); s_v16 <= unsigned(r_pl_q(15 downto 8));
            else
                s_y16 <= unsigned(r_pl_q(23 downto 16)); s_u16 <= unsigned(r_pl_q(15 downto 8)); s_v16 <= unsigned(r_pl_q(7 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(14);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums, the 8-back samples from block-RAM rings; the
    -- HD HDMI bar fix), luma 13/16, two pad stages, and the blanking gate
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sa : unsigned(11 downto 0);
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            rg_wp <= rg_wp + 1;
            rg_aa <= rg_wp - 4; rg_ab <= rg_wp - 4; rg_as <= rg_wp - (C_LAT - 3);   -- box taps 8 back (write data registered), syncs C_LAT back
            rg_fa <= rg_qa; rg_fb <= rg_qb; rg_fs <= rg_qs;
            -- The running 8-sums snap back to black once 8 blank samples are in, so a glitched sample
            -- can't offset the colours for good.
            -- box A (8-bit samples): stage 17 sums, stage 18 output
            rg_wa <= x"00" & std_logic_vector(s_y16) & std_logic_vector(s_u16) & std_logic_vector(s_v16);
            if s_bah = "00000000" then
                s_bay <= to_unsigned(128, 11); s_bau <= to_unsigned(1024, 11); s_bav <= to_unsigned(1024, 11);
            else
                s_bay <= s_bay + s_y16 - unsigned(rg_fa(23 downto 16));
                s_bau <= s_bau + s_u16 - unsigned(rg_fa(15 downto 8));
                s_bav <= s_bav + s_v16 - unsigned(rg_fa(7 downto 0));
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            if s_gav_sr(16) = '1' then
                v_sa := resize(s_bay, 12) + s_bayd; s_y18 <= v_sa(11 downto 2);
                v_sa := resize(s_bau, 12) + s_baud; s_u18 <= v_sa(11 downto 2);
                v_sa := resize(s_bav, 12) + s_bavd; s_v18 <= v_sa(11 downto 2);
            else
                s_y18 <= to_unsigned(64, 10); s_u18 <= to_unsigned(512, 10); s_v18 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(16);
            -- box B (10-bit samples): stage 19 sums, stage 20 output
            rg_wb <= "00" & std_logic_vector(s_y18) & std_logic_vector(s_u18) & std_logic_vector(s_v18);
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(512, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y18 - unsigned(rg_fb(29 downto 20));
                s_bbu <= s_bbu + s_u18 - unsigned(rg_fb(19 downto 10));
                s_bbv <= s_bbv + s_v18 - unsigned(rg_fb(9 downto 0));
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y20 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u20 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v20 <= v_sc(13 downto 4);
            -- 21: luma 13/16
            if s_y20 > 64 then v_dl := s_y20 - 64; else v_dl := (others => '0'); end if;
            s_y21 <= s_y20 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u21 <= s_u20; s_v21 <= s_v20;
            -- 22, 23: pad (one latency, a multiple of 4)
            s_y22 <= s_y21; s_u22 <= s_u21; s_v22 <= s_v21;
            s_y23 <= s_y22; s_u23 <= s_u22; s_v23 <= s_v22;
            -- 24: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y24 <= s_y23; s_u24 <= s_u23; s_v24 <= s_v23;
            else
                s_y24 <= to_unsigned(64, 10); s_u24 <= to_unsigned(512, 10); s_v24 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= rg_fs(3);
    data_out.vsync_n <= rg_fs(2);
    data_out.field_n <= rg_fs(1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture arcade;
