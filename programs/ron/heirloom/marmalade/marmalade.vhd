-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Marmalade v0.6.0 (phase 2 diag) -- Heirloom #14 (2026-10-21 rebuild), from WIN_20230307_23_36_05.
--
-- The photo: a sample-and-hold mosaic run through a nine-zone colourizer, crossed
-- by a quantized vertical triangle.
--   * Horizontal S&H locked to H: cells 40.24 px (47.7 across), the grid centred.
--   * Line hold on a row clock locked to a slow vertical master (1180 lines,
--     ~1.05 frames): 16 rows per master cycle (73.6 lines).  The master's square
--     shifts half the rows 90 px sideways (the stepped top/bottom bands).
--   * The held level picks one of nine colourizer zones, in level order
--     magenta / pink / red / orange key / dark orange / brown / dark / amber key /
--     yellow key: two stage palettes (magenta and dark) at three orange-mix levels,
--     and three flat warm keys.  The level slews between cells, so a big step
--     passes through thin slivers of the zones in between (the dark seam where
--     magenta meets amber).
--   * Stage stripes: a triangle at 6x the master, FM'd by the master's sine,
--     quantized to 8 steps with the top half clipped; every stripe band runs
--     k4 -> k3 -> k2 -> k1 -> k0 -> back (magenta / cyan / white / grey / dark in
--     the magenta palette).
--   * Soft analogue edges, chroma 3 px behind luma.
--
-- The program: the photo's measured 16-row x 64-cell zone pattern (one master cycle)
-- is the held source, in a ROM.  Video adds the camera's luma to the stage
-- triangle before it is quantized, so the stripe contours bend around the picture
-- inside every block (the block layout and colours keep the photo's composition);
-- Osc feeds two slowly drifting triangles into the same input.
--
--   K1  Cells    bipolar, per row: every block row's S&H clock detunes by its own factor
--                (8 + perm(row)) / 16 of +-1.5 octaves: right = each row finer (x1.7 .. x4.4),
--                left = each row coarser (x0.6 .. x0.22); centre = the photo's 47.7 cells
--   K2  Rows     bipolar row cascade: the row clock chirps down the frame (top row = the
--                photo's 73.6 lines); right = rows shrink to ~8 lines at the bottom, left =
--                rows swell into slabs; centre = the photo's 16 equal rows
--   K3  Picture  how hard the camera (Video) or two drifting triangles (Osc) bend the stage
--                stripes, 0 .. +-1.2 stripe cycle (25 % = the default weave); above 25 % the
--                source also bends the cell edges, up to +-4.5 cells
--   K4  Speed    bipolar (still in the centre): the master rolls, stripes crawl (top x3.2)
--   K5  Slew     bipolar about the photo (50 %): ringing rainbow fringes .. hard cells ..
--                the photo .. long rainbow smears
--   K6  Orange   orange mix: none (magenta/cyan/yellow) .. the photo .. burnt
--   S7  Shift    Stepped (the photo) / Drift (each row also slides at its own rate, with K4)
--   S8  Keys     Warm (the photo) / Cool
--   S9  Tilt     Level (the photo) / Slant (45-degree stripes)
--   S10 Mirror   Off / On (the mosaic mirrored about the centre)
--   S11 Source   Video / Osc
--   P12 STRIPES  stripe bands per master cycle 1 .. 6 (50 %, the photo) .. 36, and the band
--                width: thin bands at 0 % .. the stripes take over the blocks at 100 %
--
-- Virtual 1080-line coordinates (timing ID steps; centred grid).  Per-field
-- constants on one serial multiplier in the vblank zone; a palette engine
-- interpolates the 45 colours between the measured orange-mix anchors each field;
-- a per-line engine computes the next line's stage, rows and shift.  Generated
-- avid, output conditioning ([1 2 1] luma, [1 2 2 2 2 2 2 2 1]/16 chroma, luma
-- 13/16), blanking gate.  Latency 28 clocks in every mode.  Colours: u = Cr.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.marmalade_pkg.all;
use work.all;

architecture marmalade of program_top is
    constant C_LAT : integer := 28;   -- matches processing_delay_clks

    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
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
    -- glide: ease 1/8 of the gap, +-3 hold, single steps when close (d = target - g)
    function f_glide(g : unsigned(9 downto 0); d : signed(10 downto 0)) return unsigned is
        variable r : signed(10 downto 0);
    begin
        if d >= 8 or d <= -8 then
            r := signed('0' & g) + shift_right(d, 3);
        elsif d >= 4 then
            r := signed('0' & g) + 1;
        elsif d <= -4 then
            r := signed('0' & g) - 1;
        else
            r := signed('0' & g);
        end if;
        return unsigned(r(9 downto 0));
    end function;
    -- K1 Cells: each row's share of the detune, (8 + perm) / 16
    function f_perm(r : unsigned(3 downto 0)) return unsigned is
        type t_p is array (0 to 15) of integer range 0 to 15;
        constant C_P : t_p := (5, 12, 1, 9, 14, 3, 10, 7, 0, 13, 6, 2, 11, 15, 4, 8);
    begin
        return to_unsigned(8 + C_P(to_integer(r)), 12);
    end function;
    -- S7 Drift: each row's drift rate and direction, -3 .. +3 (never 0)
    function f_drate(r : unsigned(3 downto 0)) return integer is
        type t_d is array (0 to 15) of integer range -3 to 3;
        constant C_D : t_d := (1, -2, 3, -1, 2, -3, 2, -1, 3, -2, 1, -3, 2, -2, 1, -1);
    begin
        return C_D(to_integer(r));
    end function;
    -- zone -> palette mode / orange-mix level (zones in level order)
    type t_z4 is array (0 to 8) of unsigned(1 downto 0);
    constant C_ZMODE : t_z4 := ("00", "00", "00", "10", "01", "01", "01", "10", "10");
    constant C_ZTINT : t_z4 := ("00", "01", "10", "10", "10", "01", "00", "01", "00");

    ---------------------------------------------------------------------------
    -- ROMs / RAMs
    ---------------------------------------------------------------------------
    signal s_zt_a : unsigned(9 downto 0) := (others => '0');
    signal s_zt_q : std_logic_vector(3 downto 0) := (others => '0');
    signal s_sw_a : unsigned(9 downto 0) := (others => '0');
    signal s_sw_q : std_logic_vector(8 downto 0) := (others => '0');
    signal s_ex_a : unsigned(7 downto 0) := (others => '0');
    signal s_ex_q : std_logic_vector(9 downto 0) := (others => '0');
    signal s_sn_a : unsigned(7 downto 0) := (others => '0');
    signal s_sn_q : std_logic_vector(8 downto 0) := (others => '0');
    signal s_an_a : unsigned(6 downto 0) := (others => '0');
    signal s_an_q : std_logic_vector(29 downto 0) := (others => '0');
    -- palette RAM {zone(4), stage(3)} -> Y & U & V (written in vblank, read in active video)
    type t_pal is array (0 to 127) of std_logic_vector(29 downto 0);
    signal ram_pal : t_pal := (others => "000100000010000000001000000000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of ram_pal : signal is true;
    signal s_pw_a, s_pr_a : unsigned(6 downto 0) := (others => '0');
    signal s_pw_d : std_logic_vector(29 downto 0) := (others => '0');
    signal s_pr_q : std_logic_vector(29 downto 0) := "000100000010000000001000000000";
    signal s_pw_e : std_logic := '0';

    ---------------------------------------------------------------------------
    -- timing, controls, per-field constants
    ---------------------------------------------------------------------------
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_g1, s_g2, s_g3, s_g5, s_g6, s_gp, s_gl_g : unsigned(9 downto 0) := (others => '0');
    signal s_gl_t : std_logic_vector(9 downto 0) := (others => '0');
    signal s_gl_r : unsigned(9 downto 0) := (others => '0');
    signal s_gl_d : signed(10 downto 0) := (others => '0');
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_drift, s_cool, s_tilt, s_mir, s_osc : std_logic := '0';
    signal s_spd : signed(10 downto 0) := (others => '0');
    signal s_spv : signed(11 downto 0) := (others => '0');           -- K4 curve (x3.2 at the top)
    signal s_dt, s_dt3 : unsigned(21 downto 0) := (others => '0');   -- S7 drift phase T, 3T (Q16 cells)
    signal s_roll : unsigned(23 downto 0) := (others => '0');
    signal s_crawl : unsigned(15 downto 0) := (others => '0');
    signal s_u0 : unsigned(23 downto 0) := to_unsigned(44876, 24);
    signal s_t1, s_tp : unsigned(19 downto 0) := (others => '0');
    signal s_inc : unsigned(13 downto 0) := to_unsigned(1628, 14);
    signal s_du : unsigned(16 downto 0) := to_unsigned(14244, 17);
    signal s_rpi : unsigned(27 downto 0) := (others => '0');
    signal s_rp0 : unsigned(27 downto 0) := (others => '0');
    signal s_nq8 : unsigned(13 downto 0) := to_unsigned(1536, 14);
    signal s_nqh : unsigned(13 downto 0) := (others => '0');
    signal s_phi_i : unsigned(15 downto 0) := (others => '0');
    signal s_phi0 : unsigned(15 downto 0) := (others => '0');
    signal s_b : unsigned(17 downto 0) := (others => '0');
    signal s_pq, s_pql, s_pqh, s_gc : unsigned(11 downto 0) := (others => '0');
    -- v0.6 per-field terms
    signal s_gph : unsigned(8 downto 0) := (others => '0');          -- (P12 - 512)+
    signal s_g2m : unsigned(9 downto 0) := (others => '0');          -- |K2 - 512|
    signal s_g2n : std_logic := '0';
    signal s_g3l : unsigned(8 downto 0) := (others => '0');          -- min(K3, 256)
    signal s_g3h : unsigned(9 downto 0) := (others => '0');          -- (K3 - 256)+
    signal s_kq : unsigned(20 downto 0) := (others => '0');
    signal s_kqb, s_drpm : unsigned(15 downto 0) := (others => '0');
    signal s_drp : unsigned(27 downto 0) := (others => '0');         -- K2 row-clock chirp per line (signed)
    signal s_e1 : signed(15 downto 0) := (others => '0');            -- K1 detune, Q12 octaves
    signal s_mb0, s_mb1, s_hm1 : unsigned(11 downto 0) := (others => '0');
    signal s_bq : unsigned(13 downto 0) := to_unsigned(4096, 14);    -- P12 band width
    signal s_th1, s_th2, s_th3, s_th4 : unsigned(15 downto 0) := (others => '0');
    signal s_slope : unsigned(15 downto 0) := (others => '0');       -- S9 Slant, per pixel
    signal s_yoi : unsigned(15 downto 0) := to_unsigned(111, 16);
    signal s_xoi : unsigned(15 downto 0) := to_unsigned(111, 16);
    signal s_odr : unsigned(15 downto 0) := (others => '0');
    signal s_ss : unsigned(3 downto 0) := to_unsigned(8, 4);
    -- field serial multiplier
    signal m_a, m_acc : unsigned(31 downto 0) := (others => '0');
    signal m_b : unsigned(15 downto 0) := (others => '0');
    signal m_ld, m_stp, m_st : std_logic := '0';
    signal m_slot : unsigned(4 downto 0) := (others => '0');

    ---------------------------------------------------------------------------
    -- generated avid
    ---------------------------------------------------------------------------
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    ---------------------------------------------------------------------------
    -- palette engine (vblank)
    ---------------------------------------------------------------------------
    signal pe_st : unsigned(3 downto 0) := (others => '0');
    signal pe_z : unsigned(3 downto 0) := (others => '0');
    signal pe_k : unsigned(2 downto 0) := (others => '0');
    signal pe_m, pe_t : unsigned(1 downto 0) := (others => '0');
    signal pe_i : unsigned(1 downto 0) := (others => '0');
    signal pe_f : unsigned(7 downto 0) := (others => '0');
    signal pe_tau : unsigned(9 downto 0) := (others => '0');
    signal pe_ma : unsigned(16 downto 0) := (others => '0');
    signal pe_mb : unsigned(7 downto 0) := (others => '0');
    signal pe_macc : unsigned(16 downto 0) := (others => '0');
    signal pe_n : unsigned(3 downto 0) := (others => '0');
    signal pe_ay, pe_au, pe_av : unsigned(9 downto 0) := (others => '0');
    type t_s11 is array (0 to 2) of signed(10 downto 0);
    signal pe_d : t_s11 := (others => (others => '0'));
    type t_u18 is array (0 to 2) of unsigned(17 downto 0);
    signal pe_ca, pe_cacc : t_u18 := (others => (others => '0'));

    ---------------------------------------------------------------------------
    -- line engine
    ---------------------------------------------------------------------------
    signal e_kick, e_first : std_logic := '0';
    signal e_st, e_ret : unsigned(3 downto 0) := (others => '0');
    signal e_u : unsigned(23 downto 0) := (others => '0');
    signal e_rp, e_rpi : unsigned(27 downto 0) := (others => '0');
    signal e_phl : unsigned(15 downto 0) := (others => '0');
    signal e_n1p, e_row : unsigned(3 downto 0) := (others => '0');
    signal e_sh, e_neg : std_logic := '0';
    signal e_yo : unsigned(15 downto 0) := (others => '0');
    signal e_ma : unsigned(25 downto 0) := (others => '0');
    signal e_mb : unsigned(11 downto 0) := (others => '0');
    signal e_macc : unsigned(25 downto 0) := (others => '0');
    signal e_mi, e_n : unsigned(3 downto 0) := (others => '0');
    signal e_ei : signed(3 downto 0) := (others => '0');
    signal e_inc : unsigned(13 downto 0) := (others => '0');
    signal e_ph, e_dsel : unsigned(21 downto 0) := (others => '0');
    signal n_phi, n_yo : unsigned(15 downto 0) := (others => '0');
    signal n_ra : unsigned(3 downto 0) := (others => '0');
    signal n_inc : unsigned(13 downto 0) := to_unsigned(1628, 14);
    signal n_ph0 : unsigned(21 downto 0) := (others => '0');
    signal l_phi, l_yo : unsigned(15 downto 0) := (others => '0');
    signal l_ra : unsigned(3 downto 0) := (others => '0');
    signal l_inc : unsigned(13 downto 0) := to_unsigned(1628, 14);

    ---------------------------------------------------------------------------
    -- pixel path
    ---------------------------------------------------------------------------
    signal s_ph : unsigned(21 downto 0) := (others => '0');
    signal s_mf : std_logic := '0';                                  -- S10 Mirror: past the centre
    signal s_sx : unsigned(15 downto 0) := (others => '0');          -- S9 Slant x ramp
    type t_u6a is array (natural range <>) of unsigned(5 downto 0);
    signal p_c : t_u6a(1 to 4) := (others => (others => '0'));
    signal p_v : std_logic_vector(1 to 16) := (others => '0');
    signal p3_zt : unsigned(3 downto 0) := (others => '0');
    signal p4_zo : unsigned(3 downto 0) := (others => '0');
    signal p5_z : unsigned(3 downto 0) := (others => '0');
    signal p6_t : unsigned(7 downto 0) := (others => '0');
    signal sl_cur, sl_prev : unsigned(7 downto 0) := (others => '0');
    signal sl_d : unsigned(5 downto 0) := (others => '0');
    signal p8_cur, p9_cur, p10_cur, p11_cur, p12_cur : unsigned(7 downto 0) := (others => '0');
    signal p8_dl, p9_dl, p10_dl : signed(8 downto 0) := (others => '0');
    signal p10_e : signed(8 downto 0) := (others => '0');
    signal p11_lo, p11_hi : signed(13 downto 0) := (others => '0');
    signal p12_pr : signed(17 downto 0) := (others => '0');
    signal p13_zs : unsigned(3 downto 0) := (others => '0');
    signal p16_y, p16_u, p16_v : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_ydl : t_u10(0 to 7) := (others => to_unsigned(64, 10));
    signal s_udl, s_vdl : t_u10(0 to 10) := (others => to_unsigned(512, 10));
    signal s_by : unsigned(12 downto 0) := to_unsigned(512, 13);
    signal s_bu, s_bv : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_b8h : std_logic_vector(10 downto 0) := (others => '0');   -- generated avid of the last 11 p16 samples
    signal p18_y, p19_y, s_y12 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal p18_u, p18_v, p19_u, p19_v, s_u12, s_v12 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- picture: camera luma (Video) or the drifting triangles (Osc) into the stage phase
    signal s_xo : unsigned(15 downto 0) := (others => '0');
    signal p8_src : unsigned(7 downto 0) := x"80";
    signal p9_d : signed(8 downto 0) := (others => '0');
    signal p10_lo : signed(15 downto 0) := (others => '0');
    signal p10_hi : signed(15 downto 0) := (others => '0');
    signal p11_o : signed(20 downto 0) := (others => '0');
    signal p11_q : unsigned(15 downto 0) := (others => '0');
    signal p12_phi, p12_t : unsigned(15 downto 0) := (others => '0');
    -- K3 above 25 %: the source bends the cell edges
    signal p10c_lo, p10c_hi : signed(15 downto 0) := (others => '0');
    signal p11c : signed(21 downto 0) := (others => '0');
    signal p13_k : unsigned(2 downto 0) := (others => '0');
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_bxh : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 box inputs
    signal s_y13, s_y14, s_y15, s_y16, s_y17, s_y18x, s_yf18 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u18, s_v18 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- Memories
    ---------------------------------------------------------------------------
    p_roms : process(clk)
    begin
        if rising_edge(clk) then
            s_zt_q <= C_ZT(to_integer(s_zt_a));
            s_sw_q <= C_SLW(to_integer(s_sw_a));
            s_ex_q <= C_EXP(to_integer(s_ex_a));
            s_sn_q <= C_SIN(to_integer(s_sn_a));
            s_an_q <= C_ANC(to_integer(s_an_a));
        end if;
    end process p_roms;

    p_palram : process(clk)
    begin
        if rising_edge(clk) then
            s_pr_q <= ram_pal(to_integer(s_pr_a));
            if s_pw_e = '1' then ram_pal(to_integer(s_pw_a)) <= s_pw_d; end if;
        end if;
    end process p_palram;


    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one serial multiplier)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s : signed(10 downto 0);
        variable v_slot : integer range 0 to 31;
        variable v_r : unsigned(31 downto 0);
        variable v_k : signed(10 downto 0);
        variable v_o : unsigned(11 downto 0);
        variable v_d : signed(21 downto 0);
    begin
        if rising_edge(clk) then
            s_xs <= f_vstep_x(registers_in(8)(3 downto 0));
            s_ys <= f_vstep_y(registers_in(8)(3 downto 0));

            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            if s_zone = '0' then
                s_zc <= (others => '0');
            elsif s_zc /= 1023 then
                s_zc <= s_zc + 1;
            end if;

            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- zone clock 1: switches, speed, motion phases
            if s_zone = '1' and s_zc = 1 then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_drift <= registers_in(6)(0);   -- S7 Shift: Stepped / Drift
                s_cool  <= registers_in(6)(1);   -- S8 Keys
                s_tilt  <= registers_in(6)(2);   -- S9 Tilt: Level / Slant
                s_mir   <= registers_in(6)(3);   -- S10 Mirror
                s_osc   <= registers_in(6)(4);   -- S11 Source
                v_s := signed(resize(unsigned(registers_in(3)), 11)) - 512;      -- K4 Speed
                if v_s > -8 and v_s < 8 then v_s := (others => '0'); end if;
                s_spd <= v_s;
                -- master roll v x 2^-17 cycles, stripe crawl v x 2^-14 cycles per field
                s_roll  <= s_roll + unsigned(resize(shift_left(resize(s_spv, 24), 7), 24));
                s_crawl <= s_crawl + unsigned(resize(shift_left(resize(s_spv, 16), 2), 16));
                -- S7 Drift phase T (Q16 cells): 16 v plus a slow free run, so Drift moves at K4 0
                v_d := shift_left(resize(s_spv, 22), 4);
                if s_spv < 0 then v_d := v_d - 512; else v_d := v_d + 512; end if;
                if s_drift = '0' then s_dt <= (others => '0');
                else s_dt <= s_dt + unsigned(v_d); end if;
                -- the Osc picture source drifts on its own (about 15 s per cycle)
                s_odr <= s_odr + 73;
                case to_integer(s_xs) is
                    when 4      => s_xoi <= to_unsigned(111, 16);
                    when 3      => s_xoi <= to_unsigned(83, 16);
                    when others => s_xoi <= to_unsigned(56, 16);
                end case;
            end if;
            if s_zone = '1' and s_zc = 2 then
                s_u0 <= to_unsigned(44876, 24) - s_roll;
                -- K4 curve: v0.5's speed within +-128 (the default +128), steeper beyond (x3.2 at the top)
                if s_spd > 128 then s_spv <= shift_left(resize(s_spd, 12), 2) - 384;
                elsif s_spd < -128 then s_spv <= shift_left(resize(s_spd, 12), 2) + 384;
                else s_spv <= resize(s_spd, 12); end if;
            end if;
            if s_zone = '1' and s_zc = 3 then
                s_dt3 <= s_dt + shift_left(s_dt, 1);
                -- cell centring: (W - centre offset - 2 x the filter delay); shifted rows + 2 K
                case to_integer(s_xs) is
                    when 4      => v_o := s_w - 31; s_mb1 <= s_w - 31 + 90;
                    when 3      => v_o := s_w - 34; s_mb1 <= s_w - 34 + 120;
                    when others => v_o := s_w - 40; s_mb1 <= s_w - 40 + 180;
                end case;
                s_mb0 <= v_o;
                s_hm1 <= shift_right(v_o, 1) - 1;      -- S10 Mirror turns at the centre cell boundary
            end if;
            -- zone clock 28 (after the glides): knob terms for the multiplier
            if s_zone = '1' and s_zc = 28 then
                if s_gp > 512 then s_gph <= resize(s_gp - 512, 9); else s_gph <= (others => '0'); end if;
                if s_g2 >= 512 then s_g2m <= s_g2 - 512; s_g2n <= '0';
                else s_g2m <= 512 - s_g2; s_g2n <= '1'; end if;
                if s_g3 > 256 then s_g3l <= to_unsigned(256, 9); s_g3h <= s_g3 - 256;
                else s_g3l <= resize(s_g3, 9); s_g3h <= (others => '0'); end if;
            end if;
            -- K2 chirp gain: right = rows shrink (c up to 8.4 by the bottom), left = rows swell (c to -0.92)
            if s_zone = '1' and s_zc = 300 then
                if s_g2n = '0' then s_kqb <= resize(s_kq(20 downto 8), 16);
                else s_kqb <= resize(s_kq(20 downto 11), 16) - resize(s_kq(20 downto 14), 16); end if;
            end if;
            -- zone clocks 4 .. 27: glides, four clocks per channel on one shared datapath
            if s_zone = '1' and s_zc(9 downto 5) = 0 and s_zc(4 downto 2) /= 0 and s_zc(4 downto 2) /= 7 then
                case to_integer(s_zc(1 downto 0)) is
                    when 0 =>
                        case to_integer(s_zc(4 downto 2)) is
                            when 1      => s_gl_g <= s_g1; s_gl_t <= registers_in(0);
                            when 2      => s_gl_g <= s_g2; s_gl_t <= registers_in(1);
                            when 3      => s_gl_g <= s_g3; s_gl_t <= registers_in(2);
                            when 4      => s_gl_g <= s_g5; s_gl_t <= registers_in(4);
                            when 5      => s_gl_g <= s_g6; s_gl_t <= registers_in(5);
                            when others => s_gl_g <= s_gp; s_gl_t <= registers_in(7);
                        end case;
                    when 1 =>
                        s_gl_d <= signed('0' & unsigned(s_gl_t)) - signed('0' & s_gl_g);
                    when 2 =>
                        if s_warm = 1 then s_gl_r <= unsigned(s_gl_t); else s_gl_r <= f_glide(s_gl_g, s_gl_d); end if;
                    when others =>
                        case to_integer(s_zc(4 downto 2)) is
                            when 1      => s_g1 <= s_gl_r;
                            when 2      => s_g2 <= s_gl_r;
                            when 3      => s_g3 <= s_gl_r;
                            when 4      => s_g5 <= s_gl_r;
                            when 5      => s_g6 <= s_gl_r;
                            when others => s_gp <= s_gl_r;
                        end case;
                end case;
            end if;

            -- serial multiplier slots: slot = zc / 32 (1 .. 23), operands at sub 0,
            -- 16 shift-add steps, result at sub 20 (decode registered a clock ahead)
            m_slot <= s_zc(9 downto 5);
            if s_zone = '1' and s_zc(4 downto 0) = 0 and s_zc(9 downto 5) /= 0 and s_zc(9 downto 5) <= 17 then
                m_ld <= '1'; else m_ld <= '0'; end if;
            if s_zone = '1' and s_zc(4 downto 0) >= 1 and s_zc(4 downto 0) <= 16 then
                m_stp <= '1'; else m_stp <= '0'; end if;
            if s_zone = '1' and s_zc(4 downto 0) = 20 and s_zc(9 downto 5) /= 0 and s_zc(9 downto 5) <= 17 then
                m_st <= '1'; else m_st <= '0'; end if;
            v_slot := to_integer(m_slot);
            if m_ld = '1' then
                m_acc <= (others => '0');
                case v_slot is
                    when 1  => m_a <= resize(s_gph, 32); m_b <= resize(s_gph, 16);
                    when 2  => m_a <= resize(s_t1, 32); m_b <= to_unsigned(15, 16);
                    when 3  =>
                        m_a <= to_unsigned(763, 32);     -- cells x 16 (47.7, the photo; K1 detunes per row)
                        case to_integer(s_xs) is
                            when 4      => m_b <= to_unsigned(4368, 16);
                            when 3      => m_b <= to_unsigned(3276, 16);
                            when others => m_b <= to_unsigned(2184, 16);
                        end case;
                    when 4  => m_a <= resize(s_g2m, 32); m_b <= resize(s_ys, 16);
                    when 5  => m_a <= resize(s_g3l, 32); m_b <= to_unsigned(77, 16);
                    when 6  => m_a <= resize(s_ys, 32); m_b <= to_unsigned(14244, 16);
                    when 7  => m_a <= resize(s_g3h, 32); m_b <= to_unsigned(180, 16);
                    when 8  => m_a <= resize(s_du, 32); m_b <= to_unsigned(256, 16);     -- 16 rows (the photo)
                    when 9  => m_a <= resize(s_u0, 32); m_b <= to_unsigned(256, 16);
                    when 10 => m_a <= resize(s_gp, 32); m_b <= resize(s_gp, 16);
                    when 11 => m_a <= resize(s_tp, 32); m_b <= to_unsigned(5, 16);
                    when 12 => m_a <= resize(s_du, 32); m_b <= resize(s_nq8, 16);
                    when 13 => m_a <= resize(s_u0 - 969742, 32); m_b <= resize(s_nq8, 16);
                    when 14 => m_a <= to_unsigned(4765, 32); m_b <= resize(s_nq8, 16);
                    when 15 => m_a <= resize(s_ys, 32); m_b <= to_unsigned(111, 16);
                    when 16 => m_a <= resize(s_rpi, 32); m_b <= s_kqb;
                    when others =>
                        m_a <= resize(s_nq8, 32);
                        case to_integer(s_xs) is
                            when 4      => m_b <= to_unsigned(28488, 16);
                            when 3      => m_b <= to_unsigned(21366, 16);
                            when others => m_b <= to_unsigned(14244, 16);
                        end case;
                end case;
            elsif m_stp = '1' then
                if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
                m_a <= m_a(30 downto 0) & '0';
                m_b <= '0' & m_b(15 downto 1);
            elsif m_st = '1' then
                v_r := m_acc;
                case v_slot is
                    when 1  => s_t1 <= v_r(19 downto 0);
                    when 2  => s_nqh <= v_r(23 downto 10);
                    when 3  => s_inc <= v_r(23 downto 10);
                    when 4  => s_kq <= v_r(20 downto 0);
                    when 5  => s_pql <= v_r(17 downto 6);
                    when 6  => s_du <= v_r(24 downto 8);
                    when 7  => s_pqh <= v_r(17 downto 6);
                    when 8  => s_rpi <= v_r(31 downto 4);
                    when 9  => s_rp0 <= v_r(31 downto 4);
                    when 10 => s_tp <= v_r(19 downto 0);
                    when 11 => s_nq8 <= to_unsigned(256, 14) + resize(v_r(22 downto 10), 14) + s_nqh;
                    when 12 => s_phi_i <= v_r(31 downto 16);
                    when 13 => s_phi0 <= v_r(31 downto 16) - s_crawl;
                    when 14 => s_b <= v_r(25 downto 8);
                    when 15 => s_yoi <= v_r(23 downto 8);
                    when 16 => s_drpm <= v_r(31 downto 16);
                    when others => s_slope <= v_r(31 downto 16);
                end case;
            end if;
            -- zone clocks 780 / 781: direct terms
            if s_zone = '1' and s_zc = 780 then
                s_ss <= s_g5(9 downto 6);
                -- K1: detune = (K1 - 512) x 12 (Q12 octaves; +-1.5 octaves at the ends)
                v_k := signed('0' & s_g1) - 512;
                s_e1 <= shift_left(resize(v_k, 16), 3) + shift_left(resize(v_k, 16), 2);
                -- K2: the chirp, signed
                if s_g2n = '1' then s_drp <= to_unsigned(0, 28) - resize(s_drpm, 28);
                else s_drp <= resize(s_drpm, 28); end if;
                -- K3: stripe bend (v0.5's slope to the default, steeper beyond) and the cell bend
                s_pq <= s_pql + s_pqh;
                s_gc <= resize(shift_left(resize(s_g3h, 12), 1) + resize(s_g3h, 12), 12);
                -- P12: stripe band width (quantizer step), 2048 .. 4096 (the photo) .. 7162
                if s_gp > 512 then
                    s_bq <= to_unsigned(4096, 14) + resize(shift_left(resize(s_gph, 14), 2) + shift_left(resize(s_gph, 14), 1), 14);
                else
                    s_bq <= to_unsigned(2048, 14) + shift_left(resize(s_gp, 14), 2);
                end if;
            end if;
            if s_zone = '1' and s_zc = 781 then
                s_th1 <= resize(s_bq, 16);
                s_th2 <= shift_left(resize(s_bq, 16), 1);
                s_th3 <= resize(s_bq, 16) + shift_left(resize(s_bq, 16), 1);
                s_th4 <= shift_left(resize(s_bq, 16), 2);
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            g_wm1 <= s_w - 1;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Palette engine: 9 zones x 5 stages, interpolated between the anchors by the
    -- orange mix (K6), written into the palette RAM in vblank
    ---------------------------------------------------------------------------
    p_paleng : process(clk)
        variable v_t : unsigned(1 downto 0);
        variable v_tau : unsigned(10 downto 0);
        variable v_m : unsigned(1 downto 0);
        variable v_p : unsigned(9 downto 0);
        variable v_q : signed(11 downto 0);
        variable v_o : unsigned(9 downto 0);
        variable v_ch : std_logic_vector(29 downto 0);
    begin
        if rising_edge(clk) then
            s_pw_e <= '0';
            case to_integer(pe_st) is
                when 0 =>
                    if s_zone = '1' and s_zc = 800 then
                        pe_z <= (others => '0');
                        pe_st <= to_unsigned(1, 4);
                    end if;
                when 1 =>
                    -- the zone's orange-mix position tau (Q8): t * g6 / 2 below the centre,
                    -- t + (g6 - 512)(154 - 51 t) / 256 above it
                    v_t := C_ZTINT(to_integer(pe_z));
                    v_m := C_ZMODE(to_integer(pe_z));
                    if v_m = 2 and s_cool = '1' then v_m := "11"; end if;
                    pe_m <= v_m; pe_t <= v_t;
                    if s_g6(9) = '0' then
                        case to_integer(v_t) is
                            when 0      => pe_tau <= (others => '0');
                            when 1      => pe_tau <= resize(s_g6(8 downto 1), 10);
                            when others => pe_tau <= resize(s_g6(8 downto 0), 10);
                        end case;
                        pe_st <= to_unsigned(3, 4);
                    else
                        pe_ma <= resize(s_g6(8 downto 0), 17);
                        case to_integer(v_t) is
                            when 0      => pe_mb <= to_unsigned(154, 8);
                            when 1      => pe_mb <= to_unsigned(103, 8);
                            when others => pe_mb <= to_unsigned(52, 8);
                        end case;
                        pe_macc <= (others => '0'); pe_n <= (others => '0');
                        pe_st <= to_unsigned(2, 4);
                    end if;
                when 2 =>
                    if pe_mb(0) = '1' then pe_macc <= pe_macc + pe_ma; end if;
                    pe_ma <= pe_ma(15 downto 0) & '0';
                    pe_mb <= '0' & pe_mb(7 downto 1);
                    if pe_n = 7 then
                        pe_st <= to_unsigned(12, 4);
                    end if;
                    pe_n <= pe_n + 1;
                when 12 =>
                    pe_tau <= shift_left(resize(pe_t, 10), 8) + resize(pe_macc(16 downto 8), 10);
                    pe_st <= to_unsigned(3, 4);
                when 3 =>
                    v_tau := resize(pe_tau, 11);
                    if v_tau(10 downto 8) >= 2 then
                        pe_i <= "10";
                        if v_tau - 512 > 255 then pe_f <= (others => '1');
                        else pe_f <= resize(v_tau - 512, 8); end if;
                    else
                        pe_i <= v_tau(9 downto 8);
                        pe_f <= v_tau(7 downto 0);
                    end if;
                    pe_k <= (others => '0');
                    pe_st <= to_unsigned(4, 4);
                when 4 =>
                    -- anchor A address
                    s_an_a <= pe_m & pe_i & pe_k;
                    pe_st <= to_unsigned(5, 4);
                when 5 =>
                    -- anchor B address (tint i + 1)
                    s_an_a <= pe_m & (pe_i + 1) & pe_k;
                    pe_st <= to_unsigned(6, 4);
                when 6 =>
                    -- A arrives
                    pe_ay <= unsigned(s_an_q(29 downto 20));
                    pe_au <= unsigned(s_an_q(19 downto 10));
                    pe_av <= unsigned(s_an_q(9 downto 0));
                    pe_st <= to_unsigned(7, 4);
                when 7 =>
                    -- B arrives: differences, then |d| x f on three serial multipliers
                    pe_d(0) <= signed('0' & unsigned(s_an_q(29 downto 20))) - signed('0' & pe_ay);
                    pe_d(1) <= signed('0' & unsigned(s_an_q(19 downto 10))) - signed('0' & pe_au);
                    pe_d(2) <= signed('0' & unsigned(s_an_q(9 downto 0))) - signed('0' & pe_av);
                    pe_st <= to_unsigned(8, 4);
                when 8 =>
                    for j in 0 to 2 loop
                        if pe_d(j) < 0 then pe_ca(j) <= resize(unsigned(-pe_d(j)), 18);
                        else pe_ca(j) <= resize(unsigned(pe_d(j)), 18); end if;
                        pe_cacc(j) <= (others => '0');
                    end loop;
                    pe_mb <= pe_f; pe_n <= (others => '0');
                    pe_st <= to_unsigned(9, 4);
                when 9 =>
                    for j in 0 to 2 loop
                        if pe_mb(0) = '1' then pe_cacc(j) <= pe_cacc(j) + pe_ca(j); end if;
                        pe_ca(j) <= pe_ca(j)(16 downto 0) & '0';
                    end loop;
                    pe_mb <= '0' & pe_mb(7 downto 1);
                    if pe_n = 7 then pe_st <= to_unsigned(10, 4); end if;
                    pe_n <= pe_n + 1;
                when 10 =>
                    for j in 0 to 2 loop
                        if j = 0 then v_o := pe_ay; elsif j = 1 then v_o := pe_au; else v_o := pe_av; end if;
                        v_p := pe_cacc(j)(17 downto 8);
                        if pe_d(j) < 0 then v_q := signed(resize(v_o, 12)) - signed(resize(v_p, 12));
                        else v_q := signed(resize(v_o, 12)) + signed(resize(v_p, 12)); end if;
                        v_ch((29 - 10 * j) downto (20 - 10 * j)) := std_logic_vector(unsigned(v_q(9 downto 0)));
                    end loop;
                    s_pw_d <= v_ch;
                    s_pw_a <= pe_z & pe_k;
                    s_pw_e <= '1';
                    if pe_k = 4 then
                        if pe_z = 8 then
                            pe_st <= to_unsigned(0, 4);
                        else
                            pe_z <= pe_z + 1;
                            pe_st <= to_unsigned(1, 4);
                        end if;
                    else
                        pe_k <= pe_k + 1;
                        pe_st <= to_unsigned(4, 4);
                    end if;
                when others =>
                    pe_st <= to_unsigned(0, 4);
            end case;
        end if;
    end process p_paleng;

    ---------------------------------------------------------------------------
    -- Generated avid
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
    -- Line engine: the next line's stage phase, table row, shift, Osc row phase,
    -- and (v0.6) its cell clock (K1, per row), cell start phase and drift (S7);
    -- the row clock chirps by K2.  One serial shift-add multiplier (state 15).
    ---------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_fm : unsigned(15 downto 0);
        variable v_phi : unsigned(15 downto 0);
        variable v_n1 : unsigned(3 downto 0);
        variable v_sh : std_logic;
        variable v_us : unsigned(23 downto 0);
        variable v_sn : signed(8 downto 0);
        variable v_e : signed(15 downto 0);
        variable v_q : unsigned(16 downto 0);
        variable v_dr : integer range -3 to 3;
    begin
        if rising_edge(clk) then
            e_kick <= '0';
            if s_zone = '1' and s_zc = 1000 then
                e_u <= s_u0; e_rp <= s_rp0; e_rpi <= s_rpi; e_phl <= s_phi0;
                e_yo <= to_unsigned(0, 16) - s_odr;
                e_first <= '1';
                e_kick <= '1';
            elsif g_start = '1' then
                e_kick <= '1';
            end if;
            if g_start = '1' then
                l_phi <= n_phi; l_ra <= n_ra; l_yo <= n_yo; l_inc <= n_inc;
            end if;

            case to_integer(e_st) is
                when 0 =>
                    if e_kick = '1' then
                        v_us := e_u + 5295000;
                        s_sn_a <= v_us(23 downto 16);
                        e_st <= to_unsigned(1, 4);
                    end if;
                when 1 =>
                    e_st <= to_unsigned(2, 4);
                when 2 =>
                    -- |sin| x B, 8 steps
                    e_neg <= s_sn_q(8);
                    v_sn := signed(s_sn_q);
                    if v_sn < 0 then v_sn := -v_sn; end if;
                    e_mb <= resize(unsigned(v_sn(7 downto 0)), 12);
                    e_ma <= resize(s_b, 26);
                    e_macc <= (others => '0'); e_mi <= (others => '0');
                    e_n <= to_unsigned(7, 4); e_ret <= to_unsigned(3, 4);
                    e_st <= to_unsigned(15, 4);
                when 3 =>
                    -- stage phase = linear phase + FM (the pixel path adds the picture)
                    v_fm := e_macc(23 downto 8);
                    if e_neg = '1' then v_phi := e_phl - v_fm; else v_phi := e_phl + v_fm; end if;
                    n_phi <= v_phi;
                    v_n1 := e_rp(27 downto 24);
                    n_ra <= v_n1; e_row <= v_n1;
                    -- the shift is latched at each row start from the master's square
                    v_sh := e_sh;
                    if e_first = '1' or v_n1 /= e_n1p then
                        v_us := e_u + 6815744;
                        v_sh := not v_us(23);
                    end if;
                    e_sh <= v_sh;
                    n_yo <= e_yo;
                    e_n1p <= v_n1; e_first <= '0';
                    -- K2: the row clock chirps by s_drp per line (0 = the photo's 16 equal rows)
                    e_u <= e_u + s_du; e_rp <= e_rp + e_rpi; e_rpi <= e_rpi + s_drp;
                    e_phl <= e_phl + s_phi_i; e_yo <= e_yo + s_yoi;
                    -- K1: E1 = detune x (8 + perm(row)) / 16, 5 steps (signed, modular)
                    e_ma <= unsigned(resize(s_e1, 26));
                    e_mb <= f_perm(v_n1);
                    e_macc <= (others => '0'); e_mi <= (others => '0');
                    e_n <= to_unsigned(4, 4); e_ret <= to_unsigned(4, 4);
                    e_st <= to_unsigned(15, 4);
                when 4 =>
                    v_e := signed(e_macc(19 downto 4));
                    s_ex_a <= unsigned(v_e(11 downto 4));
                    e_ei <= v_e(15 downto 12);
                    e_st <= to_unsigned(5, 4);
                when 5 =>
                    e_st <= to_unsigned(6, 4);
                when 6 =>
                    -- this row's cell clock = inc x 2^E1: mantissa (512 .. 1021) x inc, 10 steps
                    e_ma <= resize(s_inc, 26);
                    e_mb <= resize(unsigned(s_ex_q), 12);
                    e_macc <= (others => '0'); e_mi <= (others => '0');
                    e_n <= to_unsigned(9, 4); e_ret <= to_unsigned(7, 4);
                    e_st <= to_unsigned(15, 4);
                when 7 =>
                    case to_integer(e_ei) is
                        when 2 | 3 | 4 | 5 | 6 | 7 => v_q := resize(e_macc(23 downto 7), 17);
                        when 1      => v_q := resize(e_macc(23 downto 8), 17);
                        when 0      => v_q := resize(e_macc(23 downto 9), 17);
                        when -1     => v_q := resize(e_macc(23 downto 10), 17);
                        when -2     => v_q := resize(e_macc(23 downto 11), 17);
                        when others => v_q := resize(e_macc(23 downto 12), 17);
                    end case;
                    if v_q > 16383 then e_inc <= to_unsigned(16383, 14);
                    else e_inc <= v_q(13 downto 0); end if;
                    e_st <= to_unsigned(8, 4);
                when 8 =>
                    -- centred start phase: inc x (W - offsets [+ 2 x the 90 px shift]) / 2
                    e_ma <= resize(e_inc, 26);
                    if e_sh = '1' then e_mb <= s_mb1; else e_mb <= s_mb0; end if;
                    e_macc <= (others => '0'); e_mi <= (others => '0');
                    e_n <= to_unsigned(11, 4); e_ret <= to_unsigned(9, 4);
                    e_st <= to_unsigned(15, 4);
                when 9 =>
                    e_ph <= to_unsigned(26 * 65536, 22) - e_macc(22 downto 1);
                    -- S7 Drift: T x this row's rate (T = 0 when Stepped)
                    v_dr := f_drate(e_row);
                    if v_dr = 1 or v_dr = -1 then e_dsel <= s_dt;
                    elsif v_dr = 2 or v_dr = -2 then e_dsel <= shift_left(s_dt, 1);
                    else e_dsel <= s_dt3; end if;
                    e_st <= to_unsigned(10, 4);
                when 10 =>
                    if f_drate(e_row) < 0 then n_ph0 <= e_ph - e_dsel; else n_ph0 <= e_ph + e_dsel; end if;
                    n_inc <= e_inc;
                    e_st <= to_unsigned(0, 4);
                when 15 =>
                    -- serial shift-add multiplier: e_macc += e_ma x e_mb, e_n + 1 steps
                    if e_mb(0) = '1' then e_macc <= e_macc + e_ma; end if;
                    e_ma <= e_ma(24 downto 0) & '0';
                    e_mb <= '0' & e_mb(11 downto 1);
                    if e_mi = e_n then e_st <= e_ret; end if;
                    e_mi <= e_mi + 1;
                when others =>
                    e_st <= to_unsigned(0, 4);
            end case;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_pa : unsigned(13 downto 0);
        variable v_kn : unsigned(2 downto 0);
        variable v_v : signed(18 downto 0);
        variable v_z : signed(18 downto 0);
        variable v_tx, v_ty : unsigned(14 downto 0);
    begin
        if rising_edge(clk) then
            -- 0: cell phase (this line's start phase and cell clock from the line engine;
            -- S10 Mirror runs it backwards past the centre), the Osc x triangle phase and
            -- the S9 Slant ramp (aligned with g_avid)
            if g_start = '1' then
                s_ph <= n_ph0;
                s_mf <= '0';
                s_xo <= s_odr;
                s_sx <= (others => '0');
            elsif g_avid = '1' then
                if s_mf = '1' then s_ph <= s_ph - resize(l_inc, 22);
                else s_ph <= s_ph + resize(l_inc, 22); end if;
                if g_cnt = s_hm1 and s_mir = '1' then s_mf <= '1'; end if;
                s_xo <= s_xo + s_xoi;
                if s_tilt = '1' then s_sx <= s_sx + s_slope; end if;
            end if;
            -- 1: zone ROM address; K3 above 25 % bends the cell phase by the source
            v_pa := s_ph(21 downto 8) + unsigned(p11c(21 downto 8));
            s_zt_a <= l_ra & v_pa(13 downto 8);
            p_v(1) <= g_avid;
            for i in 2 to 16 loop p_v(i) <= p_v(i - 1); end loop;
            -- 2: ROM out; 3: fabric; 4: pad
            p3_zt <= unsigned(s_zt_q);
            p4_zo <= p3_zt;
            -- 5: zone (16ths, clamped to the nine zones)
            if p4_zo > 8 then p5_z <= to_unsigned(8, 4); else p5_z <= p4_zo; end if;
            -- 6: target level
            p6_t <= shift_left(resize(p5_z, 8), 4);
            -- 7: slew tracking (fresh at each line start)
            if p_v(6) = '0' then
                sl_cur <= p6_t; sl_prev <= p6_t; sl_d <= (others => '1');
            elsif p6_t /= sl_cur then
                sl_prev <= sl_cur; sl_cur <= p6_t; sl_d <= (others => '0');
            elsif sl_d /= 63 then
                sl_d <= sl_d + 1;
            end if;
            -- 8: slew ROM address (signed kernel: K5 left of the photo rings)
            s_sw_a <= s_ss & sl_d;
            p8_cur <= sl_cur;
            p8_dl <= signed('0' & sl_cur) - signed('0' & sl_prev);
            -- 9: ROM out; 10: fabric
            p9_cur <= p8_cur; p9_dl <= p8_dl;
            p10_cur <= p9_cur; p10_dl <= p9_dl;
            p10_e <= signed(s_sw_q);
            -- 11: delta x e, split on e (low nibble unsigned, high part signed)
            p11_cur <= p10_cur;
            p11_lo <= p10_dl * signed('0' & std_logic_vector(p10_e(3 downto 0)));
            p11_hi <= p10_dl * p10_e(8 downto 4);
            -- 12: sum
            p12_cur <= p11_cur;
            p12_pr <= shift_left(resize(p11_hi, 18), 4) + resize(p11_lo, 18);
            -- 13: v = cur - delta e / 256 -> zone (rounded, clamped 0 .. 8)
            v_v := signed(resize(shift_left(resize(p12_cur, 16), 8), 19)) - resize(p12_pr, 19);
            v_z := shift_right(v_v + 2048, 12);
            if v_z < 0 then p13_zs <= (others => '0');
            elsif v_z > 8 then p13_zs <= to_unsigned(8, 4);
            else p13_zs <= unsigned(v_z(3 downto 0)); end if;

            -- picture into the stage phase (parallel to stages 7 .. 13); the camera is
            -- tapped straight off the input (its few-pixel lead is invisible)
            -- 8: source
            if s_osc = '1' then
                if s_xo(15) = '1' then v_tx := not s_xo(14 downto 0); else v_tx := s_xo(14 downto 0); end if;
                if l_yo(15) = '1' then v_ty := not l_yo(14 downto 0); else v_ty := l_yo(14 downto 0); end if;
                p8_src <= to_unsigned(64, 8) + shift_right(resize(v_tx(14 downto 8), 8) + resize(v_ty(14 downto 8), 8), 1);
            else
                p8_src <= unsigned(data_in.y(9 downto 2));
            end if;
            -- 9: centred
            p9_d <= signed('0' & p8_src) - 128;
            -- 10: x Picture, split on the bits of pq; x the cell bend (K3 above 25 %)
            p10_lo <= p9_d * signed('0' & s_pq(5 downto 0));
            p10_hi <= p9_d * signed('0' & s_pq(11 downto 6));
            p10c_lo <= p9_d * signed('0' & s_gc(5 downto 0));
            p10c_hi <= p9_d * signed('0' & s_gc(11 downto 6));
            -- 11: offset = d pq / 4 (Q16 stripe cycles); line phase + Slant ramp; cell bend (Q16 cells)
            p11_o <= shift_left(resize(p10_hi, 21), 6) + resize(p10_lo, 21);
            p11_q <= l_phi + s_sx;
            p11c <= shift_left(resize(p10c_hi, 22), 6) + resize(p10c_lo, 22);
            -- 12: phase
            p12_phi <= p11_q + unsigned(resize(shift_right(p11_o, 2), 16));
            -- 12b: t = |frac - 1/2|
            if p12_phi(15) = '1' then p12_t <= p12_phi - 32768; else p12_t <= 32768 - p12_phi; end if;
            -- 13: stage k = min(floor(t / bq), 4) (bq 4096 = the photo's floor(16 t))
            v_kn := "000";
            if p12_t >= s_th1 then v_kn := "001"; end if;
            if p12_t >= s_th2 then v_kn := "010"; end if;
            if p12_t >= s_th3 then v_kn := "011"; end if;
            if p12_t >= s_th4 then v_kn := "100"; end if;
            p13_k <= v_kn;

            -- 14: palette RAM address; 15: RAM out; 16: fabric
            -- black / neutral off the generated avid, so the box sums below resync in every blanking interval
            s_pr_a <= p13_zs & p13_k;
            if s_gav_sr(14) = '1' then
                p16_y <= unsigned(s_pr_q(29 downto 20));
                p16_u <= unsigned(s_pr_q(19 downto 10));
                p16_v <= unsigned(s_pr_q(9 downto 0));
            else
                p16_y <= to_unsigned(64, 10); p16_u <= to_unsigned(512, 10); p16_v <= to_unsigned(512, 10);
            end if;
            s_b8h <= s_b8h(9 downto 0) & s_gav_sr(14);
            -- 17: luma box 8; chroma 3 px behind luma, box 8 (both snap back to black / neutral once
            -- 11 blank samples are in, so a glitched sample can't offset them for good)
            s_ydl(0) <= p16_y;
            for i in 1 to 7 loop s_ydl(i) <= s_ydl(i - 1); end loop;
            s_udl(0) <= p16_u; s_vdl(0) <= p16_v;
            for i in 1 to 10 loop s_udl(i) <= s_udl(i - 1); s_vdl(i) <= s_vdl(i - 1); end loop;
            if s_b8h = "00000000000" then
                s_by <= to_unsigned(512, 13); s_bu <= to_unsigned(4096, 13); s_bv <= to_unsigned(4096, 13);
            else
                s_by <= s_by + resize(p16_y, 13) - resize(s_ydl(7), 13);
                s_bu <= s_bu + resize(s_udl(2), 13) - resize(s_udl(10), 13);
                s_bv <= s_bv + resize(s_vdl(2), 13) - resize(s_vdl(10), 13);
            end if;
            -- 18, 19: pads; 20: compose (chroma neutral off the generated avid, for the shell box sums)
            p18_y <= s_by(12 downto 3); p18_u <= s_bu(12 downto 3); p18_v <= s_bv(12 downto 3);
            p19_y <= p18_y; p19_u <= p18_u; p19_v <= p18_v;
            s_y12 <= p19_y;
            if s_gav_sr(18) = '1' then
                s_u12 <= p19_u; s_v12 <= p19_v;
            else
                s_u12 <= to_unsigned(512, 10); s_v12 <= to_unsigned(512, 10);
            end if;
            s_bxh <= s_bxh(6 downto 0) & s_gav_sr(18);

            -- output conditioning (shell): chroma [1 2 2 2 2 2 2 2 1]/16, luma [1 2 1], 13/16;
            -- the running 8-sums snap back to neutral once 8 blank samples are in
            s_csu(0) <= s_u12; s_csv(0) <= s_v12;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            if s_bxh = "00000000" then
                s_boxu <= to_unsigned(4096, 14); s_boxv <= to_unsigned(4096, 14);
            else
                s_boxu <= s_boxu + s_u12 - s_csu(7);
                s_boxv <= s_boxv + s_v12 - s_csv(7);
            end if;
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            s_u18 <= resize(shift_right(resize(s_boxu, 15) + s_boxud, 4), 10);
            s_v18 <= resize(shift_right(resize(s_boxv, 15) + s_boxvd, 4), 10);
            s_y13 <= s_y12; s_y14 <= s_y13; s_y15 <= s_y14; s_y16 <= s_y15;
            s_y17 <= s_y16; s_y18x <= s_y17;
            s_yf18 <= resize(shift_right(resize(s_y16, 12) + shift_left(resize(s_y17, 12), 1) + resize(s_y18x, 12), 2), 10);
            if s_gav_sr(C_LAT - 3) = '1' then
                if s_yf18 > 64 then
                    s_y19 <= s_yf18 - shift_right(s_yf18 - 64, 3) - shift_right(s_yf18 - 64, 4);
                else
                    s_y19 <= s_yf18;
                end if;
                s_u19 <= s_u18; s_v19 <= s_v18;
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture marmalade;
