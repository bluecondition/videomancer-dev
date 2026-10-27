-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Argus v0.5.0 -- Heirloom #21, rebuilt from the photo WIN_20230325_11_04_55.
--
-- The photo: camera silhouettes (a dark left block, arms, blobs and a ">" band) folded top to bottom
-- about row 522, keyed into olive / brown / green with dark contour lines, and around them a sawtooth
-- stripe field: a horizontal ramp (158 px) that the silhouettes reset and whose frequency their
-- low-passed darkness boosts, so every silhouette's trailing edge is echoed by compressed stripes
-- (the "ribs" of the central band).  An RGB colourizer: G = |H-V + V-triangle| (cyan centre, magenta
-- ends), R = |x| V mixed with a vertical ramp, B = the saw; inside the silhouettes B is cut and an
-- olive fill follows the background hue; short B dropouts speckle the pastels.
--
-- The patch: darkness store (Osc: fitted to the photo; Video: the camera's top half, written live)
-- -> fold -> bilinear darkness D -> key (D > 0) -> RC charge E -> ramp f += inc (1 + m E), reset in
-- the key -> RGB laws -> RGB -> YUV.
--
--   K1  Echo    FM depth: how hard the stripes bunch up behind each silhouette (0 = even ribs)
--   K2  Shadow  silhouette level: cores only .. flooded
--   K3  Tint    offset inside the G fold: magenta / violet .. cyan / white
--   K4  Speed   stripes stream out of the silhouettes, bipolar, still in the centre +-8
--   K5  Spread  width of the cyan centre field
--   K6  Grain   silhouette texture: flat olive .. dark contour lines, dashes, mottling
--   S7  Mirror  Fold (the photo) / Repeat
--   S8  Specks  On (the photo) / Off
--   S9  Swap    R <-> B laws
--   S10 Kaleido left/right mirror of the silhouettes and colour field
--   S11 Source  Video / Osc
--   P12 Ribs    stripe period 640 .. 160 (the photo) .. 40 virtual px
--
-- Virtual geometry from the measured width and active lines (1920 x 1080 virtual), generated avid,
-- output conditioning (luma [1 2 1]/4, chroma [1 2 2 2 2 2 2 2 1]/16, luma 13/16) before the
-- blanking gate.  Latency 28 in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.argus_pkg.all;
use work.all;

architecture argus of program_top is
    constant C_LAT : integer := 28;   -- matches processing_delay_clks

    constant C_AX8  : integer := 133632;    -- fold line, Q.8 virtual lines (522)
    constant C_XC8  : integer := 244160;    -- hue-V centre, Q.8 virtual px (953.75)
    constant C_C0   : integer := 182031;    -- line-start phase, Q.18
    constant C_B0Q  : integer := 36044;     -- charge floor 0.55, Q.16

    function f_clip8(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 8);
        elsif v > 1020 then return to_unsigned(255, 8);
        else return unsigned(v(9 downto 2)); end if;
    end function;

    -- stores: Osc ROM banks (argus_pkg), Video RAM banks, capture RAM, EXP ROM
    type t_ram8 is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal vr0, vr1, vr2, vr3 : t_ram8 := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of vr0 : signal is true;
    attribute no_rw_check of vr1 : signal is true;
    attribute no_rw_check of vr2 : signal is true;
    attribute no_rw_check of vr3 : signal is true;
    signal a_b0, a_b1, a_b2, a_b3 : unsigned(9 downto 0) := (others => '0');
    signal r_o0, r_o1, r_o2, r_o3 : std_logic_vector(7 downto 0) := (others => '0');
    signal r_v0, r_v1, r_v2, r_v3 : std_logic_vector(7 downto 0) := (others => '0');
    signal w_we : std_logic_vector(3 downto 0) := (others => '0');
    signal w_a  : unsigned(9 downto 0) := (others => '0');
    signal w_d  : std_logic_vector(7 downto 0) := (others => '0');
    signal x_a  : unsigned(8 downto 0) := (others => '0');
    signal r_x  : std_logic_vector(15 downto 0) := (others => '0');

    -- zone / width / lines
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wd : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_lcnt, s_hl, s_hd : unsigned(10 downto 0) := to_unsigned(1080, 11);

    -- controls (glided Q.3: K1, K2, K5, P12)
    type t_g is array(0 to 3) of unsigned(12 downto 0);
    signal s_g : t_g := (others => to_unsigned(512 * 8, 13));
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w, q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');

    -- serial divider
    signal d_n : unsigned(18 downto 0) := (others => '0');
    signal d_r : unsigned(12 downto 0) := (others => '0');
    signal d_v : unsigned(11 downto 0) := (others => '0');

    -- shared serial multiplier (a 18 b x b 13 b, 13 steps)
    signal mu_ldf, mu_ldl : std_logic := '0';
    signal mu_af, mu_al : unsigned(17 downto 0) := (others => '0');
    signal mu_bf, mu_bl : unsigned(12 downto 0) := (others => '0');
    signal m_a : unsigned(17 downto 0) := (others => '0');
    signal m_b, m_l : unsigned(12 downto 0) := (others => '0');
    signal m_h : unsigned(17 downto 0) := (others => '0');
    signal m_n : unsigned(3 downto 0) := (others => '0');
    signal m_p : unsigned(30 downto 0);

    -- per-field values
    signal f_dxq : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal f_dyq : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal f_set : unsigned(1 downto 0) := (others => '0');
    signal f_ip  : unsigned(12 downto 0) := to_unsigned(1639, 13);
    signal f_kh  : unsigned(10 downto 0) := to_unsigned(429, 11);
    signal f_inc0 : unsigned(14 downto 0) := to_unsigned(1639, 15);
    signal f_x   : unsigned(10 downto 0) := (others => '0');
    signal f_me  : unsigned(11 downto 0) := to_unsigned(1177, 12);
    signal f_tmp : unsigned(14 downto 0) := (others => '0');
    signal f_m   : unsigned(13 downto 0) := (others => '0');
    signal f_kvl : unsigned(14 downto 0) := (others => '0');
    signal f_kq  : unsigned(8 downto 0) := to_unsigned(68, 9);
    signal f_gs  : unsigned(12 downto 0) := (others => '0');
    signal f_gs0 : unsigned(20 downto 0) := (others => '0');
    signal f_sh  : signed(8 downto 0) := (others => '0');
    signal f_tint : signed(10 downto 0) := (others => '0');
    signal f_flow : unsigned(17 downto 0) := (others => '0');
    signal f_spa : unsigned(8 downto 0) := (others => '0');
    signal f_sps : std_logic := '0';
    signal f_sp  : unsigned(14 downto 0) := (others => '0');
    signal f_off  : unsigned(5 downto 0) := to_unsigned(19, 6);
    signal f_flat : std_logic := '0';
    signal f_mot  : unsigned(1 downto 0) := "10";
    signal f_rep, f_nosp, f_swap, f_kal, f_osc : std_logic := '0';
    signal f_k4 : unsigned(9 downto 0) := (others => '0');
    signal f_k6 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal f_k3 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal f_g6lo : std_logic := '0';
    signal f_th2, f_th1, f_tb2, f_tb1 : unsigned(8 downto 0) := (others => '0');
    signal f_g6d : unsigned(9 downto 0) := (others => '0');

    -- line engine (computes the next line during the current one) and committed line values
    signal le_on, le_first, le_mul : std_logic := '0';
    signal le_st : unsigned(2 downto 0) := (others => '0');
    signal le_k  : unsigned(3 downto 0) := (others => '0');
    signal le_ph : unsigned(3 downto 0) := (others => '0');
    signal le_kick, le_kf, le_ld, le_sv : std_logic := '0';
    signal le_sg : std_logic := '0';
    signal n_vy : unsigned(18 downto 0) := (others => '0');
    signal n_t  : signed(19 downto 0) := (others => '0');
    signal n_vyf : unsigned(17 downto 0) := (others => '0');
    signal n_yr, n_ty : unsigned(12 downto 0) := (others => '0');
    signal n_r, n_fy : unsigned(5 downto 0) := (others => '0');
    signal n_f0 : unsigned(17 downto 0) := (others => '0');
    signal n_ar, n_br, n_cr, n_gk : signed(12 downto 0) := (others => '0');
    signal n_sr : signed(16 downto 0) := (others => '0');
    signal n_st : signed(1 downto 0) := (others => '0');
    signal n_top : std_logic := '1';
    signal n_s1 : std_logic := '0';
    signal n_x0, n_x1 : unsigned(11 downto 0) := (others => '0');
    signal sp_r1 : unsigned(1 downto 0) := (others => '0');
    signal lx : unsigned(31 downto 0) := x"2545F491";
    signal l_par : std_logic := '0';
    signal l_f0 : unsigned(17 downto 0) := (others => '0');
    signal l_gk : signed(12 downto 0) := (others => '0');
    signal l_sr : signed(16 downto 0) := (others => '0');
    signal l_vr0 : signed(23 downto 0) := (others => '0');

    -- row writer (horizontal blank): the next line's darkness row, 61 columns, into the row buffer;
    -- R saw table (next line's bank)
    type t_rb is array (0 to 127) of std_logic_vector(15 downto 0);
    signal rbe, rbo, ttr : t_rb := (others => (others => '0'));
    attribute no_rw_check of rbe : signal is true;
    attribute no_rw_check of rbo : signal is true;
    attribute no_rw_check of ttr : signal is true;
    signal rw_on : std_logic := '0';
    signal rw_c : unsigned(5 downto 0) := (others => '0');
    signal rw_v1, rw_v2, rw_v3, rw_v4, rw_v5 : std_logic := '0';
    signal rw_c1, rw_c2, rw_c3, rw_c4, rw_c5 : unsigned(5 downto 0) := (others => '0');
    signal rw_rp, rw_bk : std_logic := '0';
    signal rw_qa, rw_qb : std_logic_vector(7 downto 0) := (others => '0');
    signal rw_a, rw_d, rw_a5 : signed(8 downto 0) := (others => '0');
    signal rw_p : signed(14 downto 0) := (others => '0');
    signal rb_we, rw_v6 : std_logic := '0';
    signal rw_c6 : unsigned(5 downto 0) := (others => '0');
    signal rw_ds, rw_dp : signed(10 downto 0) := (others => '0');
    signal rb_wa : unsigned(6 downto 0) := (others => '0');
    signal rb_wd, rb_wdd : std_logic_vector(15 downto 0) := (others => '0');
    signal rb_a : unsigned(6 downto 0) := (others => '0');
    signal r_rbe, r_rbo : std_logic_vector(15 downto 0) := (others => '0');
    signal tw_on, tw_we : std_logic := '0';
    signal tw_s : unsigned(6 downto 0) := (others => '0');
    signal tw_a : unsigned(6 downto 0) := (others => '0');
    signal tw_cr : signed(18 downto 0) := (others => '0');
    signal tw_qr, tw_dq : signed(23 downto 0) := (others => '0');
    signal tw_dr : std_logic_vector(15 downto 0) := (others => '0');
    signal ta : unsigned(6 downto 0) := (others => '0');
    signal tga : unsigned(5 downto 0) := (others => '0');
    signal r_tr, r_tgb : std_logic_vector(15 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- video write (straight into the store banks during active video)
    signal w_yf : unsigned(11 downto 0) := (others => '0');
    signal w_vx : unsigned(18 downto 0) := (others => '0');
    signal w_y8 : unsigned(7 downto 0) := (others => '0');
    signal w_yq : unsigned(9 downto 0) := (others => '0');
    signal w_a1, w_a2, w_a3 : std_logic := '0';
    signal w_capl, w_ptop : std_logic := '0';
    signal w_pr : unsigned(5 downto 0) := (others => '1');
    signal w_row : unsigned(5 downto 0) := (others => '0');
    signal w_ev : std_logic := '0';
    signal w_col : unsigned(5 downto 0) := (others => '0');
    signal w_val : unsigned(7 downto 0) := (others => '0');
    signal w_sh4 : std_logic := '1';

    -- beam (Kaleido: it turns at the centre and runs back)
    signal b_vx : unsigned(18 downto 0) := (others => '0');
    signal b_stp : signed(10 downto 0) := (others => '0');
    signal b_dn, b_mv, b_first : std_logic := '0';
    signal b_sp : std_logic := '0';

    -- pixel pipeline
    type t_sl is array (natural range <>) of std_logic;
    type t_u2 is array (natural range <>) of unsigned(1 downto 0);
    type t_s4 is array (natural range <>) of signed(3 downto 0);
    type t_s11 is array (natural range <>) of signed(10 downto 0);
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal p_first : t_sl(1 to 10) := (others => '0');
    signal p_sp : t_sl(1 to 18) := (others => '0');
    signal p_dir   : t_sl(1 to 10) := (others => '0');
    signal p_key   : t_sl(6 to 18) := (others => '0');
    signal p_dl    : t_u2(6 to 17) := (others => (others => '0'));
    signal p_rl    : t_u2(7 to 18) := (others => (others => '0'));
    signal p_mot   : t_s4(6 to 16) := (others => (others => '0'));
    signal p1_fx, p_fx, p_fxb : unsigned(5 downto 0) := (others => '0');
    signal p3_l, p4_l : signed(10 downto 0) := (others => '0');
    signal p3_d : signed(11 downto 0) := (others => '0');
    signal p4_p : signed(18 downto 0) := (others => '0');
    signal p5_dp : signed(12 downto 0) := (others => '0');
    signal p6_t : unsigned(15 downto 0) := (others => '0');
    signal p6_q9 : unsigned(8 downto 0) := (others => '0');
    signal p7_ca, p7_cb : std_logic_vector(1 downto 0) := (others => '0');
    signal p7_sp : std_logic := '0';
    signal p7_tk : unsigned(17 downto 0) := (others => '0');
    signal p8_e : unsigned(17 downto 0) := (others => '0');
    signal p9_pl, p9_ph : unsigned(13 downto 0) := (others => '0');
    signal p10_inc : unsigned(18 downto 0) := (others => '0');
    signal p11_f : unsigned(17 downto 0) := (others => '0');
    signal p11_ar : signed(23 downto 0) := (others => '0');
    signal p11_ag : unsigned(20 downto 0) := (others => '0');
    signal p_vbr : t_s11(12 to 14) := (others => (others => '0'));
    signal p12_gin, p13_gin : signed(14 downto 0) := (others => '0');
    signal p14_tr : signed(11 downto 0) := (others => '0');
    signal p14_tg : unsigned(6 downto 0) := (others => '0');
    signal p14_tb : unsigned(7 downto 0) := (others => '0');
    signal p14_gab : unsigned(14 downto 0) := (others => '0');
    signal p15_r, p15_g, p15_b : unsigned(7 downto 0) := (others => '0');
    signal p16_d : signed(8 downto 0) := (others => '0');
    signal p16_r, p16_g, p16_b : unsigned(7 downto 0) := (others => '0');
    signal p17_u3 : unsigned(2 downto 0) := (others => '0');
    signal p16_m : signed(11 downto 0) := (others => '0');
    signal p17_r, p17_g, p17_b : unsigned(7 downto 0) := (others => '0');
    signal p18_r, p18_g, p18_b, p18_rb, p18_gb : unsigned(7 downto 0) := (others => '0');
    signal p19_r, p19_g, p19_b : unsigned(7 downto 0) := (others => '0');
    signal p20_a, p20_b : unsigned(9 downto 0) := (others => '0');
    signal p20_u1, p20_u2 : unsigned(15 downto 0) := (others => '0');
    signal p20_u3 : unsigned(7 downto 0) := (others => '0');
    signal p21_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p21_u, p21_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    -- chroma delay ring (8-bit U & V, 8 pixels back) for the box sums
    type t_cd is array (0 to 255) of std_logic_vector(15 downto 0);
    signal cdl : t_cd := (others => x"8080");
    attribute no_rw_check of cdl : signal is true;
    signal cd_wa : unsigned(7 downto 0) := (others => '0');
    signal cd_ra : unsigned(7 downto 0) := to_unsigned(250, 8);
    signal cd_wd : std_logic_vector(15 downto 0) := x"8080";
    signal r_cd, cd_q : std_logic_vector(15 downto 0) := x"8080";
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_y1, s_y2, s_y3, s_y4, s_y5, s_y6, s_yf : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_uc, s_vc : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_yo, s_uo, s_vo : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- Stores: Osc ROM banks, Video RAM banks (write: video side during
    -- active video; read: row writer in blanking), EXP ROM, G/B saw ROM
    ---------------------------------------------------------------------------
    p_srom : process(clk)
    begin
        if rising_edge(clk) then
            r_o0 <= C_ST0(to_integer(a_b0)); r_o1 <= C_ST1(to_integer(a_b1));
            r_o2 <= C_ST2(to_integer(a_b2)); r_o3 <= C_ST3(to_integer(a_b3));
            r_x <= C_EXP(to_integer(x_a));
        end if;
    end process p_srom;
    p_vr0 : process(clk)
    begin
        if rising_edge(clk) then
            if w_we(0) = '1' then vr0(to_integer(w_a)) <= w_d; end if;
            r_v0 <= vr0(to_integer(a_b0));
        end if;
    end process p_vr0;
    p_vr1 : process(clk)
    begin
        if rising_edge(clk) then
            if w_we(1) = '1' then vr1(to_integer(w_a)) <= w_d; end if;
            r_v1 <= vr1(to_integer(a_b1));
        end if;
    end process p_vr1;
    p_vr2 : process(clk)
    begin
        if rising_edge(clk) then
            if w_we(2) = '1' then vr2(to_integer(w_a)) <= w_d; end if;
            r_v2 <= vr2(to_integer(a_b2));
        end if;
    end process p_vr2;
    p_vr3 : process(clk)
    begin
        if rising_edge(clk) then
            if w_we(3) = '1' then vr3(to_integer(w_a)) <= w_d; end if;
            r_v3 <= vr3(to_integer(a_b3));
        end if;
    end process p_vr3;
    p_tgb : process(clk)
    begin
        if rising_edge(clk) then
            r_tgb <= C_TGB(to_integer(tga));
        end if;
    end process p_tgb;

    ---------------------------------------------------------------------------
    -- Shared serial multiplier: load (field sequencer or line engine), 13
    -- shift-add steps; the product m_p is valid 15 clocks after the request
    ---------------------------------------------------------------------------
    p_mul : process(clk)
        variable v_s : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            if mu_ldl = '1' or mu_ldf = '1' then
                if mu_ldl = '1' then m_a <= mu_al; m_b <= mu_bl; else m_a <= mu_af; m_b <= mu_bf; end if;
                m_h <= (others => '0'); m_l <= (others => '0'); m_n <= to_unsigned(13, 4);
            elsif m_n /= 0 then
                if m_b(0) = '1' then v_s := resize(m_h, 19) + resize(m_a, 19); else v_s := resize(m_h, 19); end if;
                m_h <= v_s(18 downto 1);
                m_l <= v_s(0) & m_l(12 downto 1);
                m_b <= '0' & m_b(12 downto 1);
                m_n <= m_n - 1;
            end if;
        end if;
    end process p_mul;
    m_p <= m_h & m_l;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_x  : signed(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_rm : unsigned(12 downto 0);
        variable v_zc : integer range 0 to 1023;
        variable v_a  : unsigned(9 downto 0);
        variable v_th : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
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
            v_zc := to_integer(s_zc);

            -- width (max source avid pixels per line) and active lines per field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
                if s_lcnt /= 2047 then s_lcnt <= s_lcnt + 1; end if;
            end if;
            g_wm1 <= s_w - 1;
            if s_w < 512 then s_wd <= to_unsigned(1920, 12); else s_wd <= s_w; end if;
            if s_hl < 200 then s_hd <= to_unsigned(1080, 11); else s_hd <= s_hl; end if;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_hl <= s_lcnt; s_lcnt <= (others => '0');
                f_k3 <= unsigned(registers_in(2));
                f_k4 <= unsigned(registers_in(3));
                f_k6 <= unsigned(registers_in(5));
                f_rep  <= registers_in(6)(0);
                f_nosp <= registers_in(6)(1);
                f_swap <= registers_in(6)(2);
                f_kal  <= registers_in(6)(3);
                f_osc  <= registers_in(6)(4);
                -- stripes stream out of the silhouettes (phase falls -> crests move away from the reset)
                if f_sps = '1' then f_flow <= f_flow + resize(f_sp, 18);
                else f_flow <= f_flow - resize(f_sp, 18); end if;
            end if;

            -- glides: K1, K2, K5, P12 ease 1/8 of the gap per field (deadband 4)
            if s_zone = '1' and s_zc >= 4 and s_zc < 20 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(4)) & "000"; gl_o <= s_g(2);
                            when others => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(3);
                        end case;
                    when 1 =>
                        gl_d <= signed(resize(gl_k, 14)) - signed(resize(gl_o, 14));
                    when 2 =>
                        v_ng := signed(resize(gl_o, 14)) + shift_right(gl_d, 3);
                        gl_n <= unsigned(v_ng(12 downto 0));
                        if gl_d > 31 or gl_d < -31 then gl_w <= '1'; else gl_w <= '0'; end if;
                    when others =>
                        if gl_w = '1' then
                            case to_integer(q_gi) is
                                when 0 => s_g(0) <= gl_n;
                                when 1 => s_g(1) <= gl_n;
                                when 2 => s_g(2) <= gl_n;
                                when others => s_g(3) <= gl_n;
                            end case;
                        end if;
                end case;
            end if;

            -- serial divider: zc 32..59 dxq = 491520 / W, zc 64..91 dyq = 276480 / H
            if s_zone = '1' and (v_zc >= 32 and v_zc < 60) then
                if v_zc = 32 then
                    d_r <= (others => '0'); d_n <= to_unsigned(491520, 19); d_v <= s_wd;
                elsif v_zc <= 51 then
                    v_rm := d_r(11 downto 0) & d_n(18);
                    if v_rm >= resize(d_v, 13) then d_r <= v_rm - resize(d_v, 13); d_n <= d_n(17 downto 0) & '1';
                    else d_r <= v_rm; d_n <= d_n(17 downto 0) & '0'; end if;
                else
                    f_dxq <= d_n(9 downto 0);
                end if;
            elsif s_zone = '1' and (v_zc >= 64 and v_zc < 92) then
                if v_zc = 64 then
                    d_r <= (others => '0'); d_n <= to_unsigned(276480, 19); d_v <= resize(s_hd, 12);
                elsif v_zc <= 83 then
                    v_rm := d_r(11 downto 0) & d_n(18);
                    if v_rm >= resize(d_v, 13) then d_r <= v_rm - resize(d_v, 13); d_n <= d_n(17 downto 0) & '1';
                    else d_r <= v_rm; d_n <= d_n(17 downto 0) & '0'; end if;
                else
                    f_dyq <= d_n(10 downto 0);
                end if;
            end if;

            -- EXP ROM reads (stripe rate for P12, G slope for K5)
            if v_zc = 96 then x_a <= '0' & s_g(3)(12 downto 5);
            elsif v_zc = 99 then x_a <= '1' & s_g(2)(12 downto 5); end if;
            if v_zc = 98 then f_ip <= unsigned(r_x(12 downto 0)); end if;
            if v_zc = 101 then f_kh <= unsigned(r_x(10 downto 0)); end if;

            -- scale set for the charge IIR, decoded knobs
            if f_dxq < 320 then f_set <= "00"; elsif f_dxq < 448 then f_set <= "01";
            elsif f_dxq < 576 then f_set <= "10"; else f_set <= "11"; end if;
            f_x <= s_g(0)(12 downto 3) & '0';
            f_sh <= resize(shift_right(signed(resize(s_g(1)(12 downto 3), 11)) - 512, 2), 9);
            v_x := signed(resize(f_k3, 11)) - 512;
            f_tint <= v_x + shift_right(v_x, 2);
            v_x := signed(resize(f_k4, 11)) - 512;
            if v_x > 8 then f_sps <= '0'; v_a := unsigned(v_x(9 downto 0)) - 8; f_spa <= v_a(8 downto 0);
            elsif v_x < -8 then f_sps <= '1'; v_x := -v_x; v_a := unsigned(v_x(9 downto 0)) - 8; f_spa <= v_a(8 downto 0);
            else f_sps <= '0'; f_spa <= (others => '0'); end if;
            -- K6 Grain: ring line width (offset), mottle amount, flat below 12 %
            if f_k6 < 512 then f_g6lo <= '1'; f_g6d <= 512 - f_k6; else f_g6lo <= '0'; f_g6d <= f_k6 - 512; end if;
            if f_k6 < 128 then f_flat <= '1'; else f_flat <= '0'; end if;
            -- ring thresholds on the contour distance: (48 + off) * 8/7, (16 + off) * 8/7, and their mirrors
            v_th := resize(f_off, 9) + 48;
            f_th2 <= v_th + shift_right(v_th, 3) + shift_right(v_th, 6);
            v_th := resize(f_off, 9) + 16;
            f_th1 <= v_th + shift_right(v_th, 3) + shift_right(v_th, 6);
            f_tb2 <= resize(to_unsigned(512, 10) - resize(f_th2, 10), 9);
            f_tb1 <= resize(to_unsigned(512, 10) - resize(f_th1, 10), 9);
            if f_k6 < 128 then f_mot <= "00"; elsif f_k6 < 384 then f_mot <= "01";
            elsif f_k6 < 768 then f_mot <= "10"; else f_mot <= "11"; end if;

            -- per-field products on the serial multiplier: request at zc = 128 + 16 k, product at +15
            mu_ldf <= '0';
            case v_zc is
                when 128 => mu_af <= resize(f_ip, 18);   mu_bf <= resize(f_dxq, 13); mu_ldf <= '1';     -- inc0
                when 144 => mu_af <= resize(f_x, 18);    mu_bf <= resize(f_x, 13); mu_ldf <= '1';       -- x^2
                when 160 => mu_af <= resize(f_x, 18);    mu_bf <= to_unsigned(353, 13); mu_ldf <= '1';  -- 353 x
                when 176 => mu_af <= resize(f_tmp, 18);   mu_bf <= to_unsigned(824, 13); mu_ldf <= '1';  -- 824 x^2
                when 192 =>
                    mu_af <= resize(f_inc0, 18); mu_ldf <= '1';
                    case f_set is
                        when "00" => mu_bf <= to_unsigned(236, 13);
                        when "01" => mu_bf <= to_unsigned(264, 13);
                        when "10" => mu_bf <= to_unsigned(239, 13);
                        when others => mu_bf <= to_unsigned(248, 13);
                    end case;
                when 208 => mu_af <= resize(f_tmp, 18);   mu_bf <= resize(f_me, 13); mu_ldf <= '1';      -- M
                when 224 => mu_af <= resize(f_ip, 18);   mu_bf <= to_unsigned(880, 13); mu_ldf <= '1';  -- kvl
                when 240 => mu_af <= resize(f_spa, 18);  mu_bf <= resize(f_spa, 13); mu_ldf <= '1';     -- a^2
                when 256 => mu_af <= resize(f_tmp, 18);  mu_bf <= to_unsigned(169, 13); mu_ldf <= '1';  -- speed
                when 272 => mu_af <= resize(f_dxq, 18);  mu_bf <= to_unsigned(275, 13); mu_ldf <= '1';  -- kq
                when 288 => mu_af <= resize(f_kh, 18);   mu_bf <= resize(f_dxq, 13); mu_ldf <= '1';     -- G slope
                when 304 => mu_af <= resize(f_kh, 18);   mu_bf <= to_unsigned(953, 13); mu_ldf <= '1';  -- G start
                when 320 =>
                    mu_af <= resize(f_g6d, 18); mu_ldf <= '1';                                           -- K6 rings
                    if f_g6lo = '1' then mu_bf <= to_unsigned(19, 13); else mu_bf <= to_unsigned(9, 13); end if;
                when others => null;
            end case;
            case v_zc is
                when 143 => f_inc0 <= m_p(22 downto 8);
                when 159 => f_tmp <= resize(m_p(21 downto 10), 15);
                when 175 => f_me <= resize(m_p(20 downto 10), 12);
                when 191 => f_me <= f_me + resize(m_p(29 downto 10), 12);
                when 207 => f_tmp <= m_p(22 downto 8);
                when 223 => f_m <= m_p(26 downto 13);
                when 239 => f_kvl <= m_p(22 downto 8);
                when 255 => f_tmp <= resize(m_p(17 downto 8), 15);
                when 271 => f_sp <= m_p(17 downto 3);
                when 287 => f_kq <= m_p(18 downto 10);
                when 303 => f_gs <= m_p(20 downto 8);
                when 319 => f_gs0 <= m_p(20 downto 0);
                when 335 =>
                    if f_g6lo = '1' then f_off <= to_unsigned(19, 6) + resize(m_p(13 downto 8), 6);
                    else f_off <= to_unsigned(19, 6) - resize(m_p(13 downto 9), 6); end if;
                when others => null;
            end case;

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
    -- Row buffers (even / odd store columns) and the R saw table: written for
    -- the next line (bank = its parity), read by the pixel path (this line)
    ---------------------------------------------------------------------------
    p_rbe : process(clk)
    begin
        if rising_edge(clk) then
            if rb_we = '1' then rbe(to_integer(rb_wa)) <= rb_wd; end if;
            r_rbe <= rbe(to_integer(rb_a));
        end if;
    end process p_rbe;
    p_rbo : process(clk)
    begin
        if rising_edge(clk) then
            if rb_we = '1' then rbo(to_integer(rb_wa)) <= rb_wdd; end if;
            r_rbo <= rbo(to_integer(rb_a));
        end if;
    end process p_rbo;
    p_ttr : process(clk)
    begin
        if rising_edge(clk) then
            if tw_we = '1' then ttr(to_integer(tw_a)) <= tw_dr; end if;
            r_tr <= ttr(to_integer(ta));
        end if;
    end process p_ttr;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at the source avid rise (right after the commit) it
    -- computes the NEXT line's values during the current one: setup, then 12
    -- serial products of 16 clocks; it starts the R saw-table writer after
    -- product 9.  Line 0 runs late in the vblank zone.  Committed at the next
    -- source avid rise.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_t  : signed(19 downto 0);
        variable v_ps : signed(31 downto 0);
        variable v_cu : signed(12 downto 0);
        variable v_nu : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- commit at the source avid rise
            if data_in.avid = '1' and s_avq = '0' then
                l_par <= not l_par;
                l_f0 <= n_f0; l_gk <= n_gk; l_sr <= n_sr;
                -- DDA start value at x = 0 (1019/1024 of the V term, Q.10)
                l_vr0 <= shift_left(resize(n_br, 24), 10) - shift_left(resize(n_br, 24), 2) - resize(n_br, 24);
            end if;
            -- kick (registered): one clock after the commit, or late in the vblank zone for line 0
            le_kick <= '0';
            if s_zone = '1' and s_zc = 900 then
                le_kick <= '1'; le_kf <= '1';
            elsif data_in.avid = '1' and s_avq = '0' then
                le_kick <= '1'; le_kf <= '0';
            end if;

            le_ld <= '0'; le_sv <= '0';
            if le_kick = '1' then
                le_on <= '1'; le_first <= le_kf; le_st <= to_unsigned(1, 3); le_mul <= '0';
            elsif le_on = '1' then
                if le_mul = '0' then
                    le_st <= le_st + 1;
                    case to_integer(le_st) is
                        when 1 =>
                            if le_first = '1' then n_vy <= (others => '0');
                            else n_vy <= n_vy + resize(f_dyq, 19); end if;
                        when 2 =>
                            -- Fold: mirror about the fold line; Repeat: wrap
                            if n_vy >= C_AX8 then
                                n_top <= '0';
                                if f_rep = '1' then n_t <= signed(resize(n_vy, 20)) - C_AX8;
                                else n_t <= to_signed(2 * C_AX8, 20) - signed(resize(n_vy, 20)); end if;
                            else
                                n_top <= '1';
                                n_t <= signed(resize(n_vy, 20));
                            end if;
                        when 3 =>
                            if n_t < 0 then v_t := -n_t; else v_t := n_t; end if;
                            if v_t >= C_AX8 then
                                if f_rep = '1' then v_t := v_t - C_AX8; else v_t := to_signed(C_AX8 - 1, 20); end if;
                            end if;
                            n_vyf <= unsigned(v_t(17 downto 0));
                        when 4 =>
                            v_t := to_signed(66816, 20) - signed(resize(n_vyf, 20));
                            if v_t < 0 then v_t := -v_t; end if;
                            n_t <= v_t;                -- |the vertical-triangle distance|
                        when others =>
                            le_mul <= '1'; le_k <= (others => '0'); le_ph <= (others => '0'); le_ld <= '1';
                    end case;
                else
                    -- product k: requested at phase 0 (le_ld), stored at phase 15 (le_sv)
                    le_ph <= le_ph + 1;
                    if le_ph = 14 then le_sv <= '1'; end if;
                    if le_ph = 15 then
                        if le_k = 11 then le_on <= '0'; le_mul <= '0';
                        else le_k <= le_k + 1; le_ld <= '1'; end if;
                    end if;
                end if;
            end if;

            -- product request
            mu_ldl <= '0';
            if le_ld = '1' then
                mu_ldl <= '1';
                case to_integer(le_k) is
                    when 0  => mu_al <= n_vyf;                       mu_bl <= to_unsigned(7909, 13); le_sg <= '0';
                    when 1  => mu_al <= n_vyf;                       mu_bl <= to_unsigned(2009, 13); le_sg <= '0';
                    when 2  => mu_al <= unsigned(n_t(17 downto 0));  mu_bl <= to_unsigned(4017, 13); le_sg <= '0';
                    when 3  => mu_al <= resize(f_kvl, 18);           mu_bl <= resize(n_vyf(17 downto 6), 13); le_sg <= '0';
                    when 4  => mu_al <= resize(n_yr, 18);            mu_bl <= to_unsigned(878, 13);  le_sg <= '0';
                    when 5  => mu_al <= resize(n_yr, 18);            mu_bl <= to_unsigned(1117, 13); le_sg <= '1';
                    when 6  => mu_al <= resize(n_yr, 18);            mu_bl <= to_unsigned(235, 13);  le_sg <= '0';
                    when 7  => mu_al <= resize(n_ty, 18);            mu_bl <= to_unsigned(23, 13);   le_sg <= '1';
                    when 8  => mu_al <= resize(n_ty, 18);            mu_bl <= to_unsigned(75, 13);   le_sg <= '1';
                    when 9  => mu_al <= resize(n_ty, 18);            mu_bl <= to_unsigned(68, 13);   le_sg <= '1';
                    when 10 => mu_al <= resize(n_ty, 18);            mu_bl <= to_unsigned(988, 13);  le_sg <= '0';
                    when others =>
                        mu_al <= resize(f_kq, 18);
                        if n_br < 0 then mu_bl <= unsigned(-n_br); le_sg <= '1'; else mu_bl <= unsigned(n_br); le_sg <= '0'; end if;
                end case;
            end if;

            -- product store (coefficients start from their constants at the first request)
            if le_ld = '1' and le_k = 0 then
                n_ar <= to_signed(-84, 13); n_br <= to_signed(700, 13); n_cr <= to_signed(280, 13);
                n_gk <= to_signed(640, 13) + resize(f_tint, 13);
            elsif le_sv = '1' then
                if le_sg = '1' then v_ps := -signed(resize(m_p, 32)); else v_ps := signed(resize(m_p, 32)); end if;
                case to_integer(le_k) is
                    when 4 | 7 => v_cu := n_ar;
                    when 5 | 8 => v_cu := n_br;
                    when 6 | 9 => v_cu := n_cr;
                    when others => v_cu := n_gk;
                end case;
                v_nu := v_cu + resize(shift_right(v_ps, 12), 13);
                case to_integer(le_k) is
                    when 0 =>
                        if m_p(30 downto 24) > 62 then n_r <= to_unsigned(62, 6); n_fy <= to_unsigned(63, 6);
                        else n_r <= m_p(29 downto 24); n_fy <= m_p(23 downto 18); end if;
                    when 1 => n_yr <= m_p(28 downto 16);
                    when 2 => n_ty <= to_unsigned(4096, 13) - m_p(28 downto 16);
                    when 3 => n_f0 <= m_p(19 downto 2) + to_unsigned(C_C0, 18) + f_flow;
                    when 4 | 7 => n_ar <= v_nu;
                    when 5 | 8 => n_br <= v_nu;
                    when 6 | 9 => n_cr <= v_nu;
                    when 10 => n_gk <= v_nu;
                    when others => n_sr <= resize(shift_right(v_ps, 6), 17);
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Specks, dashes and the fill's line streak for the next line, decided in
    -- the horizontal blank (row-writer clocks 40..43, after the pipeline has
    -- drained) from a xorshift LFSR
    ---------------------------------------------------------------------------
    p_spk : process(clk)
    begin
        if rising_edge(clk) then
            if rw_on = '1' then
                case to_integer(rw_c) is
                    when 40 => lx <= lx xor shift_left(lx, 13);
                    when 41 => lx <= lx xor shift_right(lx, 17);
                    when 42 => lx <= lx xor shift_left(lx, 5);
                    when 43 =>
                        -- specks (p 0.19 / line, 1..3 lines, 8..39 virtual px): B dropouts over the colour field
                        -- (S8), dark dashes inside the silhouettes (off when K6 is flat)
                        if sp_r1 /= 0 then
                            sp_r1 <= sp_r1 - 1;
                        elsif lx(7 downto 0) < 48 then
                            n_s1 <= '1';
                            n_x0 <= '0' & lx(18 downto 8);
                            n_x1 <= ('0' & lx(18 downto 8)) + 8 + resize(lx(23 downto 19), 12);
                            if lx(26 downto 25) = "11" then sp_r1 <= "10"; else sp_r1 <= lx(26 downto 25); end if;
                        else
                            n_s1 <= '0';
                        end if;
                        -- line streak of the olive fill: +1/8, 0, -1/8
                        if lx(29 downto 28) = "11" then n_st <= to_signed(1, 2);
                        elsif lx(29 downto 28) = "00" then n_st <= to_signed(-1, 2);
                        else n_st <= to_signed(0, 2); end if;
                    when others => null;
                end case;
            end if;
        end if;
    end process p_spk;

    ---------------------------------------------------------------------------
    -- Row writer (horizontal blank, kicked at the source avid fall; line 0 in
    -- the vblank zone): y-lerps the store rows r, r+1 (5-bit fraction) for the
    -- next line, one column per clock, into the row buffer (Q.2 darkness)
    ---------------------------------------------------------------------------
    p_rows : process(clk)
        variable v_r1 : unsigned(5 downto 0);
        variable v_s : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            if (data_in.avid = '0' and s_avq = '1') or (s_zone = '1' and s_zc = 1000) then
                rw_on <= '1'; rw_c <= (others => '0'); rw_bk <= not l_par;
            elsif rw_on = '1' then
                if rw_c = 60 then rw_on <= '0'; else rw_c <= rw_c + 1; end if;
            end if;
            -- 1: store addresses for (r, c) and (r+1, c)
            v_r1 := n_r + 1;
            if n_r(0) = '0' then
                if rw_c(0) = '0' then a_b0 <= n_r(5 downto 1) & rw_c(5 downto 1); a_b2 <= v_r1(5 downto 1) & rw_c(5 downto 1);
                else a_b1 <= n_r(5 downto 1) & rw_c(5 downto 1); a_b3 <= v_r1(5 downto 1) & rw_c(5 downto 1); end if;
            else
                if rw_c(0) = '0' then a_b2 <= n_r(5 downto 1) & rw_c(5 downto 1); a_b0 <= v_r1(5 downto 1) & rw_c(5 downto 1);
                else a_b3 <= n_r(5 downto 1) & rw_c(5 downto 1); a_b1 <= v_r1(5 downto 1) & rw_c(5 downto 1); end if;
            end if;
            rw_v1 <= rw_on; rw_c1 <= rw_c; rw_rp <= n_r(0);
            -- 2: EBR output (store processes)
            rw_v2 <= rw_v1; rw_c2 <= rw_c1;
            -- 3: fabric copy of rows r / r+1
            rw_v3 <= rw_v2; rw_c3 <= rw_c2;
            if f_osc = '1' then
                case std_logic_vector'(rw_rp & rw_c2(0)) is
                    when "00" => rw_qa <= r_o0; rw_qb <= r_o2;
                    when "01" => rw_qa <= r_o1; rw_qb <= r_o3;
                    when "10" => rw_qa <= r_o2; rw_qb <= r_o0;
                    when others => rw_qa <= r_o3; rw_qb <= r_o1;
                end case;
            else
                case std_logic_vector'(rw_rp & rw_c2(0)) is
                    when "00" => rw_qa <= r_v0; rw_qb <= r_v2;
                    when "01" => rw_qa <= r_v1; rw_qb <= r_v3;
                    when "10" => rw_qa <= r_v2; rw_qb <= r_v0;
                    when others => rw_qa <= r_v3; rw_qb <= r_v1;
                end case;
            end if;
            -- 4: a, b - a
            rw_v4 <= rw_v3; rw_c4 <= rw_c3;
            rw_a <= signed(resize(unsigned(rw_qa), 9)) - 128;
            rw_d <= signed(resize(unsigned(rw_qb), 9)) - signed(resize(unsigned(rw_qa), 9));
            -- 5: product (b - a) * fy5
            rw_v5 <= rw_v4; rw_c5 <= rw_c4; rw_a5 <= rw_a;
            rw_p <= rw_d * signed(resize(n_fy(5 downto 1), 6));
            -- 6: Q.2 darkness of column c
            rw_v6 <= rw_v5; rw_c6 <= rw_c5;
            v_s := shift_left(resize(rw_a5, 12), 2) + resize(shift_right(rw_p, 3), 12);
            rw_ds <= v_s(10 downto 0);
            -- 7: row buffer entry c - 1: darkness and the step to column c (bank = next line's parity)
            rb_we <= '0';
            if rw_v6 = '1' then
                rw_dp <= rw_ds;
                if rw_c6 /= 0 then
                    rb_wd <= std_logic_vector(resize(rw_dp, 16));
                    rb_wdd <= std_logic_vector(resize(resize(rw_ds, 12) - resize(rw_dp, 12), 16));
                    rb_wa <= rw_bk & (rw_c6 - 1);
                    rb_we <= '1';
                end if;
            end if;
        end if;
    end process p_rows;

    ---------------------------------------------------------------------------
    -- R saw-table writer: T_R[s] = A_R + (s C_R) >> 6 - (239 s^2) >> 12,
    -- s = 0..63, by running sums (no multiplier)
    ---------------------------------------------------------------------------
    p_tab : process(clk)
        variable v_r : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            tw_we <= '0';
            if le_ld = '1' and le_k = 10 then
                tw_on <= '1'; tw_s <= (others => '0');
                tw_cr <= (others => '0'); tw_qr <= (others => '0'); tw_dq <= to_signed(-239, 24);
            elsif tw_on = '1' then
                v_r := resize(n_ar, 20) + resize(shift_right(tw_cr, 6), 20) + resize(shift_right(tw_qr, 12), 20);
                tw_dr <= std_logic_vector(resize(v_r, 16));
                tw_a <= (not l_par) & tw_s(5 downto 0); tw_we <= '1';
                -- next s: C s += C; S2 s^2 += S2 (2 s + 1)
                tw_cr <= tw_cr + resize(n_cr, 19);
                tw_qr <= tw_qr + tw_dq;                  -- -239 s^2: increments -239 (2 s + 1)
                tw_dq <= tw_dq - 478;
                if tw_s = 63 then tw_on <= '0'; else tw_s <= tw_s + 1; end if;
            end if;
        end if;
    end process p_tab;

    ---------------------------------------------------------------------------
    -- Video write: on node lines the IIR-smoothed luma is sampled at the node
    -- columns and written straight into the store banks (active video only;
    -- the row writer reads in blanking)
    ---------------------------------------------------------------------------
    p_vw : process(clk)
        variable v_w  : unsigned(10 downto 0);
        variable v_f  : signed(13 downto 0);
        variable v_dk : signed(9 downto 0);
        variable v_x  : unsigned(18 downto 0);
        variable v_cap : std_logic;
    begin
        if rising_edge(clk) then
            if f_dxq < 320 then w_sh4 <= '1'; else w_sh4 <= '0'; end if;
            -- luma 0..255: (y - 64) * 0.291 (shift-add), two stages; avid delayed to match
            if unsigned(data_in.y) > 64 then w_yq <= unsigned(data_in.y) - 64; else w_yq <= (others => '0'); end if;
            v_w := resize(shift_right(w_yq, 2), 11) + resize(shift_right(w_yq, 5), 11) + resize(shift_right(w_yq, 7), 11)
                   + resize(shift_right(w_yq, 9), 11);
            if v_w > 255 then w_y8 <= to_unsigned(255, 8); else w_y8 <= v_w(7 downto 0); end if;
            w_a1 <= data_in.avid; w_a2 <= w_a1; w_a3 <= w_a2;

            -- node rows: in the top half, the first line of each store row (the line engine's row of this
            -- line, n_r); row 63 on the first line past the fold
            v_cap := w_capl;
            if s_zgo = '1' then
                w_pr <= (others => '1'); w_ptop <= '1';
            elsif w_a2 = '1' and w_a3 = '0' then
                if n_top = '1' then
                    if n_r /= w_pr then v_cap := '1'; w_row <= n_r; else v_cap := '0'; end if;
                    w_pr <= n_r;
                elsif w_ptop = '1' then
                    v_cap := '1'; w_row <= (others => '1');
                else
                    v_cap := '0';
                end if;
                w_ptop <= n_top;
            end if;
            w_capl <= v_cap;

            -- node columns (w_y8 is the pixel of this w_a2 clock)
            w_ev <= '0';
            if w_a2 = '1' then
                if w_a3 = '0' then
                    w_yf <= shift_left(resize(w_y8, 12), 4); w_vx <= (others => '0');
                    w_ev <= v_cap; w_col <= (others => '0'); w_val <= w_y8;
                else
                    v_f := signed(shift_left(resize(w_y8, 14), 4)) - signed(resize(w_yf, 14));
                    if w_sh4 = '1' then v_f := signed(resize(w_yf, 14)) + shift_right(v_f, 4);
                    else v_f := signed(resize(w_yf, 14)) + shift_right(v_f, 3); end if;
                    w_yf <= unsigned(v_f(11 downto 0));
                    v_x := w_vx + f_dxq;
                    w_vx <= v_x;
                    if v_x(18 downto 13) /= w_vx(18 downto 13) then
                        w_ev <= v_cap; w_col <= v_x(18 downto 13); w_val <= w_yf(11 downto 4);
                    end if;
                end if;
            elsif w_a3 = '1' then
                -- avid fall: the last column (60)
                w_ev <= v_cap; w_col <= to_unsigned(60, 6); w_val <= w_yf(11 downto 4);
            end if;

            -- darkness = clip(128 + 1.25 (89 - Y8)) into bank (row parity, column parity)
            w_we <= (others => '0');
            if w_ev = '1' and w_col <= 60 then
                v_dk := 89 - signed(resize(w_val, 10));
                v_dk := 128 + v_dk + shift_right(v_dk, 2);
                if v_dk < 0 then w_d <= (others => '0');
                elsif v_dk > 255 then w_d <= (others => '1');
                else w_d <= std_logic_vector(v_dk(7 downto 0)); end if;
                w_a <= w_row(5 downto 1) & w_col(5 downto 1);
                case std_logic_vector'(w_row(0) & w_col(0)) is
                    when "00" => w_we <= "0001";
                    when "01" => w_we <= "0010";
                    when "10" => w_we <= "0100";
                    when others => w_we <= "1000";
                end case;
            end if;
        end if;
    end process p_vw;

    ---------------------------------------------------------------------------
    -- Beam (stage 0): virtual x (Kaleido: turns at the centre and runs back),
    -- speck and dash windows
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_s : signed(19 downto 0);
        variable v_x : unsigned(18 downto 0);
    begin
        if rising_edge(clk) then
            b_mv <= b_dn;
            if g_start = '1' then
                v_x := (others => '0'); b_first <= '1'; b_dn <= '0'; b_mv <= '0';
                b_stp <= signed(resize(f_dxq, 11));
            else
                if g_avid = '1' then
                    v_s := signed(resize(b_vx, 20)) + resize(b_stp, 20);
                    if v_s < 0 then v_x := (others => '0'); else v_x := unsigned(v_s(18 downto 0)); end if;
                    b_first <= '0';
                else
                    v_x := b_vx;
                end if;
                if f_kal = '1' and b_dn = '0' and b_vx(18 downto 14) >= 15 then
                    b_dn <= '1'; b_stp <= -signed(resize(f_dxq, 11));
                end if;
            end if;
            b_vx <= v_x;
            if n_s1 = '1' and b_vx(18 downto 8) >= n_x0 and b_vx(18 downto 8) < n_x1 then b_sp <= '1'; else b_sp <= '0'; end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_t20 : signed(19 downto 0);
        variable v_t3  : signed(12 downto 0);
        variable v_n9, v_m9 : signed(8 downto 0);
        variable v_ca, v_cb : std_logic_vector(1 downto 0);
        variable v_sp  : std_logic;
        variable v_ta, v_tb : unsigned(17 downto 0);
        variable v_x2, v_x3, v_s1, v_k1, v_s2, v_k2 : unsigned(17 downto 0);
        variable v_f   : unsigned(17 downto 0);
        variable v_g   : unsigned(15 downto 0);
        variable v_u   : signed(11 downto 0);
        variable v_ro, v_go : unsigned(7 downto 0);
        variable v_rb, v_gb : unsigned(7 downto 0);
        variable v_rr, v_gg, v_bb : unsigned(7 downto 0);
        variable v_rl  : unsigned(1 downto 0);
        variable v_ys  : signed(19 downto 0);
        variable v_sc  : unsigned(11 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_dl  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: row-buffer address (column c: darkness and step to c+1), x fraction (6 bits), hue-V direction
            rb_a <= l_par & b_vx(18 downto 13);
            p1_fx <= b_vx(12 downto 7);
            if b_vx >= C_XC8 then p_dir(1) <= not b_mv; else p_dir(1) <= b_mv; end if;
            p_first(1) <= b_first; p_sp(1) <= b_sp;

            -- 2: row-buffer outputs (RAM processes)
            p_fx <= p1_fx;

            -- 3: fabric copy: column value and step (Q.2 darkness)
            p_fxb <= p_fx;
            p3_l <= signed(r_rbe(10 downto 0)); p3_d <= signed(r_rbo(11 downto 0));

            -- 4: x-lerp product
            p4_l <= p3_l;
            p4_p <= p3_d * signed(resize(p_fxb, 7));

            -- 5: D' = D + shadow
            p5_dp <= resize(p4_l, 13) + resize(shift_right(p4_p, 6), 13) + resize(f_sh, 13);

            -- 6: key, charge target, depth level, ring phase (3 D' + 1/2), mottle (contours of 3 D' / 4)
            if p5_dp > 0 then
                p_key(6) <= '1';
                v_t20 := to_signed(C_B0Q, 20) + shift_left(resize(p5_dp, 20), 9);
                if v_t20 > 65280 then p6_t <= to_unsigned(65280, 16); else p6_t <= unsigned(v_t20(15 downto 0)); end if;
            else
                p_key(6) <= '0'; p6_t <= (others => '0');
            end if;
            if p5_dp < 43 then p_dl(6) <= "00"; elsif p5_dp < 86 then p_dl(6) <= "01"; else p_dl(6) <= "10"; end if;
            v_t3 := p5_dp + shift_left(p5_dp, 1);
            p6_q9 <= unsigned(v_t3(8 downto 0)) xor "100000000";
            v_n9 := resize(signed((not v_t3(9)) & v_t3(8 downto 2)), 9);
            case f_mot is
                when "00" => p_mot(6) <= (others => '0');
                when "01" => p_mot(6) <= resize(shift_right(v_n9, 6), 4);
                when "10" => p_mot(6) <= resize(shift_right(v_n9, 5), 4);
                when others => v_m9 := v_n9 + shift_right(v_n9, 1); p_mot(6) <= resize(shift_right(v_m9, 5), 4);
            end case;

            -- 7: charge IIR coefficients for this pixel (a shift pair by key and scale set) and the target term
            --    tk = (t >> a) + s (t >> b), with the +1 corrections of the inverted operands folded in; ring level
            if p_key(6) = '1' then
                case f_set is
                    when "00"   => v_ca := "10"; v_cb := "01"; v_sp := '1';   -- 2^-5 + 2^-7
                    when "01"   => v_ca := "01"; v_cb := "10"; v_sp := '0';   -- 2^-4 - 2^-8
                    when "10"   => v_ca := "01"; v_cb := "00"; v_sp := '1';   -- 2^-4 + 2^-6
                    when others => v_ca := "00"; v_cb := "00"; v_sp := '0';   -- 2^-3 - 2^-6
                end case;
            else
                case f_set is
                    when "00"   => v_ca := "11"; v_cb := "10"; v_sp := '0';   -- 2^-6 - 2^-8
                    when "01"   => v_ca := "11"; v_cb := "10"; v_sp := '1';   -- 2^-6 + 2^-8
                    when "10"   => v_ca := "11"; v_cb := "01"; v_sp := '1';   -- 2^-6 + 2^-7
                    when others => v_ca := "10"; v_cb := "11"; v_sp := '1';   -- 2^-5 + 2^-10
                end case;
            end if;
            p7_ca <= v_ca; p7_cb <= v_cb; p7_sp <= v_sp;
            case v_ca is
                when "00" => v_ta := shift_right(resize(p6_t, 18), 3);
                when "01" => v_ta := shift_right(resize(p6_t, 18), 4);
                when "10" => v_ta := shift_right(resize(p6_t, 18), 5);
                when others => v_ta := shift_right(resize(p6_t, 18), 6);
            end case;
            case v_cb is
                when "00" => v_tb := shift_right(resize(p6_t, 18), 6);
                when "01" => v_tb := shift_right(resize(p6_t, 18), 7);
                when "10" => v_tb := shift_right(resize(p6_t, 18), 8);
                when others => v_tb := shift_right(resize(p6_t, 18), 10);
            end case;
            if v_sp = '1' then p7_tk <= v_ta + v_tb + 2; else p7_tk <= v_ta - v_tb + 1; end if;
            -- ring level: distance of q9 to the nearest contour (triangle) against the field's thresholds
            if f_flat = '1' or (p6_q9 >= f_th2 and p6_q9 <= f_tb2) then p_rl(7) <= "10";
            elsif p6_q9 >= f_th1 and p6_q9 <= f_tb1 then p_rl(7) <= "01"; else p_rl(7) <= "00"; end if;

            -- 8: charge IIR (one-clock loop): E += tk - (E >> a) - s (E >> b) as a 4-operand carry-save sum
            --    and one carry chain; E restarts at 0 on the first pixel of the line
            case p7_ca is
                when "00" => v_x2 := not shift_right(p8_e, 3);
                when "01" => v_x2 := not shift_right(p8_e, 4);
                when "10" => v_x2 := not shift_right(p8_e, 5);
                when others => v_x2 := not shift_right(p8_e, 6);
            end case;
            case p7_cb is
                when "00" => v_x3 := shift_right(p8_e, 6);
                when "01" => v_x3 := shift_right(p8_e, 7);
                when "10" => v_x3 := shift_right(p8_e, 8);
                when others => v_x3 := shift_right(p8_e, 10);
            end case;
            if p7_sp = '1' then v_x3 := not v_x3; end if;
            v_s1 := p8_e xor v_x2 xor v_x3;
            v_k1 := shift_left((p8_e and v_x2) or (p8_e and v_x3) or (v_x2 and v_x3), 1);
            v_s2 := v_s1 xor v_k1 xor p7_tk;
            v_k2 := shift_left((v_s1 and v_k1) or (v_s1 and p7_tk) or (v_k1 and p7_tk), 1);
            if p_first(7) = '1' then p8_e <= (others => '0'); else p8_e <= v_s2 + v_k2; end if;

            -- 9: FM products (E 8 b x M 12 b)
            p9_pl <= p8_e(15 downto 8) * f_m(7 downto 2);
            p9_ph <= p8_e(15 downto 8) * f_m(13 downto 8);

            -- 10: increment
            p10_inc <= resize(f_inc0, 19) + resize(shift_right(resize(p9_pl, 20) + shift_left(resize(p9_ph, 20), 6), 1), 19);

            -- 11: phase accumulator (one-clock loop), reset in the key; hue-V DDAs (Q.10 / Q.8)
            if p_first(10) = '1' then v_f := l_f0; else v_f := p11_f; end if;
            if p_key(10) = '1' then p11_f <= f_flow; else p11_f <= v_f + p10_inc(17 downto 0); end if;
            if p_first(10) = '1' then
                p11_ar <= l_vr0; p11_ag <= f_gs0;
            elsif p_dir(10) = '1' then
                p11_ar <= p11_ar + resize(l_sr, 24); p11_ag <= p11_ag + resize(f_gs, 21);
            else
                p11_ar <= p11_ar - resize(l_sr, 24); p11_ag <= p11_ag - resize(f_gs, 21);
            end if;

            -- 12: saw-table addresses; V term; G fold input
            ta <= l_par & p11_f(17 downto 12);
            tga <= p11_f(17 downto 12);
            p_vbr(12) <= resize(shift_right(p11_ar, 10), 11);
            p12_gin <= resize(l_gk, 15) - signed(resize(p11_ag(20 downto 8), 15));

            -- 13: table outputs (RAM / ROM processes)
            p13_gin <= p12_gin;

            -- 14: fabric copy of the tables; |gin|
            p14_tr <= signed(r_tr(11 downto 0));
            p14_tg <= unsigned(r_tgb(15 downto 9));
            p14_tb <= unsigned(r_tgb(8 downto 1));
            if p13_gin < 0 then p14_gab <= unsigned(-p13_gin); else p14_gab <= unsigned(p13_gin); end if;

            -- 15: laws, clipped to 8 bits
            p15_r <= f_clip8(resize(p14_tr, 14) + resize(p_vbr(14), 14));
            v_g := resize(p14_gab, 16) + resize(p14_tg, 16);
            if v_g > 1020 then p15_g <= (others => '1'); else p15_g <= v_g(9 downto 2); end if;
            p15_b <= p14_tb;

            -- 16: swap; fill hue difference
            if f_swap = '1' then p16_r <= p15_b; p16_b <= p15_r; p16_d <= signed(resize(p15_g, 9)) - signed(resize(p15_b, 9));
            else p16_r <= p15_r; p16_b <= p15_b; p16_d <= signed(resize(p15_g, 9)) - signed(resize(p15_r, 9)); end if;
            p16_g <= p15_g;
            p16_m <= to_signed(153, 12) + shift_left(resize(p_mot(15), 12), 5);

            -- 17: u (eighths): 0.60 + 0.28 (G - R) / 255 + mottle
            v_u := p16_m + resize(shift_right(shift_left(resize(p16_d, 12), 3) + resize(p16_d, 12), 5), 12);
            if v_u < 0 then p17_u3 <= (others => '0'); elsif v_u > 255 then p17_u3 <= (others => '1');
            else p17_u3 <= unsigned(v_u(7 downto 5)); end if;
            p17_r <= p16_r; p17_g <= p16_g; p17_b <= p16_b;

            -- 18: olive palette, depth blend (law colour 7/8 -> mix -> olive)
            case to_integer(p17_u3) is
                when 0 => v_ro := to_unsigned(106, 8); v_go := to_unsigned(25, 8);
                when 1 => v_ro := to_unsigned(98, 8);  v_go := to_unsigned(40, 8);
                when 2 => v_ro := to_unsigned(90, 8);  v_go := to_unsigned(55, 8);
                when 3 => v_ro := to_unsigned(82, 8);  v_go := to_unsigned(69, 8);
                when 4 => v_ro := to_unsigned(74, 8);  v_go := to_unsigned(84, 8);
                when 5 => v_ro := to_unsigned(66, 8);  v_go := to_unsigned(98, 8);
                when 6 => v_ro := to_unsigned(57, 8);  v_go := to_unsigned(113, 8);
                when others => v_ro := to_unsigned(49, 8); v_go := to_unsigned(128, 8);
            end case;
            v_rb := p17_r - shift_right(p17_r, 3); v_gb := p17_g - shift_right(p17_g, 3);
            case p_dl(17) is
                when "00" => null;
                when "01" => v_rb := resize(shift_right(resize(v_rb, 9) + resize(v_ro, 9), 1), 8);
                             v_gb := resize(shift_right(resize(v_gb, 9) + resize(v_go, 9), 1), 8);
                when others => v_rb := v_ro; v_gb := v_go;
            end case;
            p18_rb <= v_rb; p18_gb <= v_gb;
            p18_r <= p17_r; p18_g <= p17_g; p18_b <= p17_b;

            -- 19: key fill (ring level, dashes, line streak), specks
            v_rl := p_rl(18);
            if p_sp(18) = '1' and f_flat = '0' then v_rl := "00"; end if;
            if p_key(18) = '1' then
                case v_rl is
                    when "10" =>
                        if n_st = 1 then v_rr := p18_rb + shift_right(p18_rb, 3); v_gg := p18_gb + shift_right(p18_gb, 3);
                        elsif n_st = -1 then v_rr := p18_rb - shift_right(p18_rb, 3); v_gg := p18_gb - shift_right(p18_gb, 3);
                        else v_rr := p18_rb; v_gg := p18_gb; end if;
                        v_bb := to_unsigned(5, 8);
                    when "01" => v_rr := shift_right(p18_rb, 1); v_gg := shift_right(p18_gb, 1); v_bb := to_unsigned(2, 8);
                    when others => v_rr := shift_right(p18_rb, 3); v_gg := shift_right(p18_gb, 3); v_bb := to_unsigned(1, 8);
                end case;
            else
                v_rr := p18_r; v_gg := p18_g;
                if p_sp(18) = '1' and f_nosp = '0' then v_bb := resize(shift_right(shift_left(resize(p18_b, 12), 3) + shift_left(resize(p18_b, 12), 1), 6), 8);
                else v_bb := p18_b; end if;
            end if;
            p19_r <= v_rr; p19_g <= v_gg; p19_b <= v_bb;

            -- 20: RGB (8 b) -> YUV, coefficients rounded to few shift-add terms (neutral stays neutral):
            --     Y = R + 2 G + 3 B / 8; U = Cr = (184 R - 152 G - 32 B) / 128; V = Cb = (3 B - (R + 2 G)) / 2
            p20_a <= resize(p19_r, 10) + shift_left(resize(p19_g, 10), 1);
            p20_b <= resize(p19_b, 10) + shift_left(resize(p19_b, 10), 1);
            p20_u1 <= shift_left(resize(p19_r, 16), 7) + shift_left(resize(p19_r, 16), 6) - shift_left(resize(p19_r, 16), 3);
            p20_u2 <= shift_left(resize(p19_g, 16), 7) + shift_left(resize(p19_g, 16), 4) + shift_left(resize(p19_g, 16), 3);
            p20_u3 <= p19_b;

            -- 21: 10-bit Y / U / V
            p21_y <= to_unsigned(64, 10) + p20_a + shift_right(p20_b, 3);
            v_ys := resize(shift_right(signed(resize(p20_u1, 20)) - signed(resize(p20_u2, 20)) - signed(resize(shift_left(resize(p20_u3, 16), 5), 20)), 7), 20) + 512;
            p21_u <= unsigned(v_ys(9 downto 0));
            v_ys := resize(shift_right(signed(resize(p20_b, 20)) - signed(resize(p20_a, 20)), 1), 20) + 512;
            p21_v <= unsigned(v_ys(9 downto 0));

            -- 22..28: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two adjacent 8-sums of 8-bit
            -- chroma, the 8-back value from the ring; luma [1 2 1]); the sums restart in every blank
            if s_gav_sr(20) = '0' and s_gav_sr(27) = '0' then
                s_boxu <= to_unsigned(1024, 11); s_boxv <= to_unsigned(1024, 11);
            else
                s_boxu <= s_boxu + unsigned(cd_wd(15 downto 8)) - unsigned(cd_q(15 downto 8));
                s_boxv <= s_boxv + unsigned(cd_wd(7 downto 0)) - unsigned(cd_q(7 downto 0));
            end if;
            cd_q <= r_cd;
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 12) + s_boxud; s_uc <= v_sc(11 downto 2);
            v_sc := resize(s_boxv, 12) + s_boxvd; s_vc <= v_sc(11 downto 2);
            s_y1 <= p21_y; s_y2 <= s_y1; s_y3 <= s_y2; s_y4 <= s_y3; s_y5 <= s_y4; s_y6 <= s_y5;
            v_sy := resize(s_y4, 12) + shift_left(resize(s_y5, 12), 1) + resize(s_y6, 12);
            s_yf <= v_sy(11 downto 2);
            if s_yf > 64 then v_dl := s_yf - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(C_LAT - 2) = '1' then
                s_yo <= s_yf - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_uo <= s_uc; s_vo <= s_vc;
            else
                s_yo <= to_unsigned(64, 10); s_uo <= to_unsigned(512, 10); s_vo <= to_unsigned(512, 10);
            end if;

            -- delays
            for i in 2 to 10 loop p_first(i) <= p_first(i - 1); p_dir(i) <= p_dir(i - 1); end loop;
            for i in 2 to 18 loop p_sp(i) <= p_sp(i - 1); end loop;
            for i in 7 to 18 loop p_key(i) <= p_key(i - 1); end loop;
            for i in 7 to 17 loop p_dl(i) <= p_dl(i - 1); end loop;
            for i in 8 to 18 loop p_rl(i) <= p_rl(i - 1); end loop;
            for i in 7 to 16 loop p_mot(i) <= p_mot(i - 1); end loop;
            for i in 13 to 14 loop p_vbr(i) <= p_vbr(i - 1); end loop;
        end if;
    end process p_pix;

    cd_wd <= std_logic_vector(p21_u(9 downto 2)) & std_logic_vector(p21_v(9 downto 2)) when s_gav_sr(20) = '1' else x"8080";
    p_cdl : process(clk)
    begin
        if rising_edge(clk) then
            cdl(to_integer(cd_wa)) <= cd_wd;
            r_cd <= cdl(to_integer(cd_ra));
            cd_wa <= cd_wa + 1; cd_ra <= cd_ra + 1;
        end if;
    end process p_cdl;

    data_out.y       <= std_logic_vector(s_yo);
    data_out.u       <= std_logic_vector(s_uo);
    data_out.v       <= std_logic_vector(s_vo);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture argus;
