-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Pods v0.1.1 -- Heirloom (video phase), from the clip WIN_20230328_00_11_14 at 0.5 s.
--
-- The picture: a lattice of cyan "eyes" and maroon cells in olive, yellow-green and red rings, cut into pods by
-- mint gaps.  Rows of cells (a V oscillator, 187.5 lines) cross the rings of a zone plate whose centre sits just
-- off the top-left corner (mirrored at the seam x = 1004.5): the cells are small at the top centre and stretch
-- into long bars at the edges.  A slow waveform of the same r^2 ramp opens the mint ground: the top corners, two
-- chains of lens gaps along arcs round the corner (open on the maroon rows, bridged on the cyan rows), and the
-- big bottom-centre triangle.  The rows scroll up at 141.6 px/s; the rings and arcs stand still.
--
-- The patch (pods.py has the tables, .heirloom_work/rb_pods/notes.md the measurements):
--   r^2  = (X + 50)^2 + y^2, X = 1004.5 - |x - 1004.5|              (virtual 1080-line px)
--   F    = V(y) + Ax cos(2 pi (kL r^2 + 0.567)) + 0.19 - Dv val(kE r^2) + etch * luma
--   PAL[(F + 2.75) * 128]: mint | dark brown | maroon | red | olive | yellow-green | dark teal | cyan
--
--   K1  Rows    the rows' density: x1/2.83 .. x2.83 (512 = the still's 187.5-line period)
--   K2  Rings   the zone-plate rings' density (the cells' spacing): x1/2.83 .. x2.83
--   K3  Arcs    the gap waveform's rate: x1/4 (one huge corner gap) .. x4 (many nested pod arcs)
--   K4  Speed   bipolar (+-8 deadband): the rows scroll up (+) or down (-); 700 = the clip's 141.6 px/s
--   K5  Cells   the ring level: bars only (0) .. the still (512, 0.58) .. ring stripes (1023, 1.98)
--   K6  Etch    how far the camera (or the Osc) lifts the field and bends the r^2 ramp: bright areas close the
--               gaps and grow the eyes; the Osc (2x on the level, 32x on the bend) makes the arcs and gaps wander
--   S7  Quad    Off / On: the zone-plate origin moves to the middle line: mint half-discs at the side middles,
--               arcs bowing round them, mirrored top and bottom (four-way)
--   S8  Palette Pods (the still) / Chevron (the clip at 00_17_52: grey and black with rainbow fringes)
--   S9  Mosaic  Off / On: a 38 px horizontal sample-and-hold (the 00_17_52 clip's blocks)
--   S10 Fold    Mirror (the still) / Sweep: no fold at the seam, the rings and arcs sweep on across the right
--   S11 Source  Video / Osc
--   P12 PODS    the gaps' depth: 0 = the bare lattice (the arcs only darken it) .. the still (487) .. 100 % =
--               wide mint arcs pinching the lattice into thin pods
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a per-line engine (the row cosine and
-- the two zone plates' y^2 parts by second-order DDAs), a beam with two second-order phase DDAs folded at the
-- seam, a 15-stage pixel path to the compose, then the shell's output conditioning (two [1 2 2 2 2 2 2 2 1]/16
-- boxes with the per-line resync, luma 13/16) and the blanking gate.  Virtual 1080-line coordinates (SD shows
-- the centre 4:3).  Generated avid; one latency (C_LAT 20) in every mode.  Colours in the hardware convention
-- (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.pods_pkg.all;
use work.all;

architecture pods of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    constant C_KY0  : integer := 89478;       -- row phase step per virtual line, Q24 turns (2^24 / 187.5)
    constant C_KL0  : integer := 1768;        -- zone-plate rate, 2^-28 turns per px^2 (6.587e-6)
    constant C_PL   : integer := 152306890;   -- ring phase offset, 2^-28 turns (0.567)
    constant C_LV0  : integer := 753;         -- envelope plateau (49) + palette offset (704), F/256

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
    -- '1' for 50 / 60 field-per-second modes (the scroll step halves), '0' for 1080p
    function f_half(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 3 | 7 | 9 | 11 | 13 | 15 => return '0';
            when others                   => return '1';
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- off-avid colour: black
    constant C_BKY : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_BKU : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BKV : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc lattice)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a  : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q  : unsigned(15 downto 0) := (others => '0');
    signal r_vc_a  : unsigned(7 downto 0) := (others => '0');
    signal r_vc_q  : unsigned(11 downto 0) := (others => '0');
    signal r_cos_a, r_val_a : unsigned(9 downto 0) := (others => '0');
    signal r_cos_q : unsigned(9 downto 0) := (others => '0');
    signal r_val_q : unsigned(7 downto 0) := (others => '0');
    signal r_pal_a : unsigned(9 downto 0) := (others => '0');
    signal r_pal_q : std_logic_vector(31 downto 0) := (others => '0');

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
    signal s_half : std_logic := '0';
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
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(540 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(487 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(700, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_quad, s_pal, s_mos, s_swp, s_src : std_logic := '0';

    -- per-field constants
    signal z_sh1, z_sh2, z_sh3 : signed(2 downto 0) := (others => '0');
    signal z_mt1, z_mt2, z_mt3 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_kv, z_kve : unsigned(23 downto 0) := (others => '0');      -- k vs (Q8), L / E
    signal z_vsq  : unsigned(17 downto 0) := (others => '0');
    signal z_k5   : unsigned(9 downto 0) := (others => '0');
    signal z_k5hi : std_logic := '0';
    signal c_ky   : unsigned(17 downto 0) := to_unsigned(C_KY0, 18);
    signal c_kyl  : unsigned(23 downto 0) := to_unsigned(C_KY0, 24);    -- row step per line, Q24
    signal c_th0  : unsigned(23 downto 0) := to_unsigned(1073742, 24);  -- row phase at line 0 (scrolls); power-up: the still
    signal c_dth  : unsigned(23 downto 0) := (others => '0');           -- scroll step per field
    signal c_kl, c_ke : unsigned(12 downto 0) := to_unsigned(C_KL0, 13);
    signal c_d0l, c_d0e : signed(27 downto 0) := (others => '0');        -- phase step at the first pixel
    signal c_ddl, c_dde : unsigned(15 downto 0) := (others => '0');      -- step of the step (2 k hp^2)
    signal c_pcl, c_pce : unsigned(31 downto 0) := (others => '0');      -- k u0^2 (+ ring offset)
    signal c_dy0l, c_dy0e : unsigned(17 downto 0) := (others => '0');    -- k vs^2
    signal c_ddyl, c_ddye : unsigned(18 downto 0) := (others => '0');    -- 2 k vs^2
    signal c_pyql, c_pyqe : unsigned(31 downto 0) := (others => '0');    -- Quad: k 540^2
    signal c_dyql, c_dyqe : unsigned(31 downto 0) := (others => '0');    -- Quad: k (vs^2 - 1080 vs)
    signal c_ax   : unsigned(7 downto 0) := to_unsigned(74, 8);          -- ring level, Q7
    signal c_dv   : unsigned(8 downto 0) := to_unsigned(144, 9);         -- gap depth, Q5
    signal c_eg   : unsigned(7 downto 0) := (others => '0');             -- etch gain
    signal c_eg112 : unsigned(14 downto 0) := (others => '0');           -- Osc mean * etch gain
    signal c_cd0  : unsigned(12 downto 0) := to_unsigned(404, 13);       -- 4 (2 u0 hp + hp^2)
    signal c_cpc  : unsigned(16 downto 0) := to_unsigned(2500, 17);      -- u0^2
    signal c_pfm1 : unsigned(10 downto 0) := to_unsigned(1003, 11);      -- seam pixel - 1
    signal c_mbm1 : unsigned(5 downto 0) := to_unsigned(37, 6);          -- mosaic block - 1
    signal c_mvm1 : unsigned(1 downto 0) := to_unsigned(3, 2);           -- mosaic band (lines) - 1

    -- field serial multiplier
    signal f_a   : unsigned(39 downto 0) := (others => '0');
    signal f_b   : unsigned(17 downto 0) := (others => '0');
    signal f_acc : unsigned(39 downto 0) := (others => '0');
    signal z_fa  : unsigned(39 downto 0) := (others => '0');
    signal z_fb  : unsigned(17 downto 0) := (others => '0');
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

    -- line engine
    signal e_lc   : unsigned(3 downto 0) := to_unsigned(15, 4);
    signal e_zk   : std_logic := '0';
    signal e_th   : unsigned(23 downto 0) := (others => '0');
    signal e_q    : unsigned(1 downto 0) := (others => '0');
    signal e_v    : signed(9 downto 0) := (others => '0');
    signal e_pyl, e_pye : unsigned(31 downto 0) := (others => '0');
    signal e_dyl, e_dye : unsigned(31 downto 0) := (others => '0');
    signal n_lv   : signed(10 downto 0) := (others => '0');
    signal n_phl, n_phe : unsigned(31 downto 0) := (others => '0');
    signal l_v    : signed(10 downto 0) := (others => '0');

    -- beam
    signal b_phl, b_phe : unsigned(31 downto 0) := (others => '0');
    signal b_dl, b_de   : signed(27 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_fp  : std_logic := '0';
    signal b_mc  : unsigned(5 downto 0) := (others => '0');
    signal b_ld  : std_logic := '0';
    signal b_bc  : unsigned(1 downto 0) := (others => '0');
    signal h_phl, h_phe : unsigned(31 downto 0) := (others => '0');

    -- pixel path
    signal ld_sr : std_logic_vector(9 downto 3) := (others => '0');
    signal p4_c  : signed(9 downto 0) := (others => '0');
    signal p4_v  : unsigned(7 downto 0) := (others => '0');
    -- luma chain (read 9 px ahead): box, x etch gain, the geometry bend
    signal g_l1, g_l2, g_l3, g_box : unsigned(7 downto 0) := (others => '0');
    signal g_ta, g_tb : unsigned(11 downto 0) := (others => '0');
    signal g_t   : unsigned(15 downto 0) := (others => '0');
    signal g_d   : unsigned(9 downto 0) := (others => '0');
    signal p5_ha, p5_hb : signed(14 downto 0) := (others => '0');
    signal p5_ea : unsigned(12 downto 0) := (others => '0');
    signal p5_eb : unsigned(11 downto 0) := (others => '0');
    signal p6_h  : signed(9 downto 0) := (others => '0');
    signal p6_e  : unsigned(11 downto 0) := (others => '0');
    signal p7_s1 : signed(13 downto 0) := (others => '0');
    signal p7_e  : unsigned(11 downto 0) := (others => '0');
    signal p7_et : signed(13 downto 0) := (others => '0');
    signal p8_a, p8_b : signed(13 downto 0) := (others => '0');
    signal p9_i, p10_i : unsigned(8 downto 0) := (others => '0');
    signal p13_q : std_logic_vector(31 downto 0) := (others => '0');
    signal p14_y, p14_u, p14_v : unsigned(9 downto 0) := to_unsigned(512, 10);
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
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), ROMs
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
            r_ex_q  <= C_PD_EXP2(to_integer(r_ex_a));
            r_vc_q  <= C_PD_VCOS(to_integer(r_vc_a));
            r_cos_q <= C_PD_COS(to_integer(r_cos_a));
            r_val_q <= C_PD_VAL(to_integer(r_val_a));
            r_pal_q <= C_PD_PAL(to_integer(r_pal_a));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(10 downto 0);
        variable v_t  : unsigned(39 downto 0);
        variable v_u  : unsigned(11 downto 0);
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
                s_quad <= registers_in(6)(0);
                s_pal  <= registers_in(6)(1);
                s_mos  <= registers_in(6)(2);
                s_swp  <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                s_half <= f_half(s_tid);
                case to_integer(s_xsr) is
                    when 4 =>
                        s_oxs <= to_unsigned(204, 9);
                        c_cd0 <= to_unsigned(4656, 13); c_cpc <= to_unsigned(84100, 17);
                        c_pfm1 <= to_unsigned(381, 11); c_mbm1 <= to_unsigned(18, 6); c_mvm1 <= to_unsigned(1, 2);
                    when 3 =>
                        s_oxs <= to_unsigned(153, 9);
                        c_cd0 <= to_unsigned(609, 13);  c_cpc <= to_unsigned(2500, 17);
                        c_pfm1 <= to_unsigned(668, 11); c_mbm1 <= to_unsigned(24, 6); c_mvm1 <= to_unsigned(2, 2);
                    when others =>
                        s_oxs <= to_unsigned(102, 9);
                        c_cd0 <= to_unsigned(404, 13);  c_cpc <= to_unsigned(2500, 17);
                        c_pfm1 <= to_unsigned(1003, 11); c_mbm1 <= to_unsigned(37, 6); c_mvm1 <= to_unsigned(3, 2);
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

            -- serial products: slot k is decoded at zc = 30 + 24 k (operands pre-muxed), loads one clock later,
            -- 18 iterations, the result is read at zc = 52 + 24 k
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(6, 5);  z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(7, 5);  z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(8, 5);  z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(9, 5);  z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(11, 5); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 342 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 366 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 390 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 414 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 438 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when 462 => z_sel <= to_unsigned(18, 5); z_pre <= '1';
                    when 486 => z_sel <= to_unsigned(19, 5); z_pre <= '1';
                    when 510 => z_sel <= to_unsigned(20, 5); z_pre <= '1';
                    when 534 => z_sel <= to_unsigned(21, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(C_KY0, 40);           z_fb <= resize(z_mt1, 18);       -- ky
                    when 1  => z_fa <= to_unsigned(C_KL0, 40);           z_fb <= resize(z_mt2, 18);       -- kL
                    when 2  => z_fa <= to_unsigned(C_KL0, 40);           z_fb <= resize(z_mt3, 18);       -- kE
                    when 3  => z_fa <= resize(c_ky, 40);                 z_fb <= resize(s_ys, 18);        -- ky * line step
                    when 4  => z_fa <= resize(c_kl, 40);                 z_fb <= resize(c_cd0, 18);       -- d0 L
                    when 5  => z_fa <= resize(c_kl, 40);                 z_fb <= resize(c_cpc, 18);       -- k u0^2 L
                    when 6  => z_fa <= resize(c_kl, 40);                 z_fb <= resize(s_ys, 18);        -- k vs L
                    when 7  => z_fa <= resize(z_kv, 40);                 z_fb <= resize(s_ys, 18);        -- k vs^2 L
                    when 8  => z_fa <= resize(c_ke, 40);                 z_fb <= resize(c_cd0, 18);       -- d0 E
                    when 9  => z_fa <= resize(c_ke, 40);                 z_fb <= resize(c_cpc, 18);       -- k u0^2 E
                    when 10 => z_fa <= resize(c_ke, 40);                 z_fb <= resize(s_ys, 18);        -- k vs E
                    when 11 => z_fa <= resize(z_kve, 40);                z_fb <= resize(s_ys, 18);        -- k vs^2 E
                    when 12 => z_fa <= resize(s_vm, 40);                 z_fb <= resize(s_vm, 18);        -- speed^2
                    when 13 => z_fa <= resize(z_vsq, 40);                z_fb <= to_unsigned(13, 18);     -- scroll step
                    when 14 =>                                                                              -- ring level
                        z_fa <= resize(z_k5, 40);
                        if z_k5hi = '1' then z_fb <= to_unsigned(181, 18); else z_fb <= to_unsigned(58, 18); end if;
                    when 15 => z_fa <= resize(s_gp(12 downto 3), 40);    z_fb <= to_unsigned(59, 18);     -- gap depth
                    when 16 => z_fa <= resize(s_g6(12 downto 3), 40);    z_fb <= resize(s_g6(12 downto 3), 18);  -- etch^2
                    when 17 => z_fa <= resize(c_eg, 40);                 z_fb <= to_unsigned(112, 18);    -- Osc mean * eg
                    when 18 => z_fa <= resize(c_kl, 40);                 z_fb <= to_unsigned(145800, 18); -- Quad: k 540^2 / 2, L
                    when 19 => z_fa <= resize(c_ke, 40);                 z_fb <= to_unsigned(145800, 18); -- Quad: k 540^2 / 2, E
                    when 20 => z_fa <= resize(z_kv, 40);                 z_fb <= to_unsigned(1080, 18);   -- Quad: k vs 1080, L
                    when others => z_fa <= resize(z_kve, 40);            z_fb <= to_unsigned(1080, 18);   -- Quad: k vs 1080, E
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(38 downto 0) & '0';
                f_b <= '0' & f_b(17 downto 1);
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 18 =>                        -- rows exponent: 0.75 (k1 - 512) = (3 k1 >> 2) - 384, Q8 octaves
                        v_u := resize(s_g1(12 downto 3), 12) + shift_left(resize(s_g1(12 downto 3), 12), 1);
                        v_e := signed(resize(v_u(11 downto 2), 11)) - 384;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh1 <= v_e(10 downto 8);
                    when 20 =>
                        z_mt1 <= r_ex_q;
                        -- scroll (last field's step): + speed advances the phase, the rows move UP (the clip)
                        if s_vneg = '1' then c_th0 <= c_th0 - c_dth; else c_th0 <= c_th0 + c_dth; end if;
                    when 22 =>                        -- rings exponent: 0.75 (k2 - 512)
                        v_u := resize(s_g2(12 downto 3), 12) + shift_left(resize(s_g2(12 downto 3), 12), 1);
                        v_e := signed(resize(v_u(11 downto 2), 11)) - 384;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh2 <= v_e(10 downto 8);
                    when 24 => z_mt2 <= r_ex_q;
                    when 26 =>                        -- arcs exponent: k3 - 512
                        v_e := signed(resize(s_g3(12 downto 3), 11)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh3 <= v_e(10 downto 8);
                    when 28 =>
                        z_mt3 <= r_ex_q;
                        -- ring level operand: k5 (<= 512) or k5 - 512
                        if s_g5(12) = '1' and s_g5(11 downto 3) /= 0 then
                            z_k5hi <= '1'; z_k5 <= resize(s_g5(11 downto 3), 10);
                        else
                            z_k5hi <= '0'; z_k5 <= s_g5(12 downto 3);
                        end if;
                    when 52 =>                        -- ky = KY0 mt >> (15 - sh), sh = -2 .. 1
                        case to_integer(z_sh1) is
                            when -2     => c_ky <= resize(f_acc(39 downto 17), 18);
                            when -1     => c_ky <= resize(f_acc(39 downto 16), 18);
                            when 0      => c_ky <= resize(f_acc(39 downto 15), 18);
                            when others => c_ky <= resize(f_acc(39 downto 14), 18);
                        end case;
                    when 76 =>
                        case to_integer(z_sh2) is
                            when -2     => c_kl <= resize(f_acc(39 downto 17), 13);
                            when -1     => c_kl <= resize(f_acc(39 downto 16), 13);
                            when 0      => c_kl <= resize(f_acc(39 downto 15), 13);
                            when others => c_kl <= resize(f_acc(39 downto 14), 13);
                        end case;
                    when 100 =>
                        case to_integer(z_sh3) is
                            when -2     => c_ke <= resize(f_acc(39 downto 17), 13);
                            when -1     => c_ke <= resize(f_acc(39 downto 16), 13);
                            when 0      => c_ke <= resize(f_acc(39 downto 15), 13);
                            when others => c_ke <= resize(f_acc(39 downto 14), 13);
                        end case;
                    when 102 =>                       -- 2 k hp^2 (1080: 2k; 720p: 4.5k; SD: 8k)
                        case to_integer(s_xs) is
                            when 4 =>
                                c_ddl <= resize(shift_left(resize(c_kl, 16), 3), 16);
                                c_dde <= resize(shift_left(resize(c_ke, 16), 3), 16);
                            when 3 =>
                                c_ddl <= resize(shift_right(shift_left(resize(c_kl, 17), 3) + resize(c_kl, 17), 1), 16);
                                c_dde <= resize(shift_right(shift_left(resize(c_ke, 17), 3) + resize(c_ke, 17), 1), 16);
                            when others =>
                                c_ddl <= resize(shift_left(resize(c_kl, 16), 1), 16);
                                c_dde <= resize(shift_left(resize(c_ke, 16), 1), 16);
                        end case;
                    when 124 => c_kyl <= f_acc(31 downto 8);
                    when 148 => c_d0l <= signed(f_acc(29 downto 2));
                    when 172 =>
                        v_t := f_acc + to_unsigned(C_PL, 40);
                        c_pcl <= v_t(31 downto 0);
                    when 196 => z_kv <= f_acc(23 downto 0);
                    when 220 =>
                        c_dy0l <= f_acc(33 downto 16);
                        c_ddyl <= f_acc(33 downto 16) & '0';
                    when 244 => c_d0e <= signed(f_acc(29 downto 2));
                    when 268 => c_pce <= f_acc(31 downto 0);
                    when 292 => z_kve <= f_acc(23 downto 0);
                    when 316 =>
                        c_dy0e <= f_acc(33 downto 16);
                        c_ddye <= f_acc(33 downto 16) & '0';
                    when 340 => z_vsq <= f_acc(17 downto 0);
                    when 364 =>
                        if s_half = '1' then c_dth <= f_acc(24 downto 1); else c_dth <= f_acc(23 downto 0); end if;
                    when 388 =>                       -- Ax = 16 + k5 * 58 / 512, or 74 + (k5 - 512) * 181 / 512
                        if z_k5hi = '1' then c_ax <= resize(f_acc(16 downto 9), 8) + 74;
                        else c_ax <= resize(f_acc(16 downto 9), 8) + 16; end if;
                    when 412 => c_dv <= resize(f_acc(17 downto 8), 9) + 32;   -- Dv = 32 + p12 * 59 / 256
                    when 436 => c_eg <= f_acc(20 downto 13);                  -- eg = k6^2 / 8192
                    when 460 => c_eg112 <= f_acc(14 downto 0);
                    when 484 => c_pyql <= f_acc(30 downto 0) & '0';
                    when 508 => c_pyqe <= f_acc(30 downto 0) & '0';
                    when 532 => c_dyql <= resize(c_dy0l, 32) - f_acc(39 downto 8);
                    when 556 => c_dyqe <= resize(c_dy0e, 32) - f_acc(39 downto 8);
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
    -- Generated avid (parity-locked start 3/4 clocks after the source's avid rise, field-latched width)
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
    -- Source side: oscillator (2-D triangle lattice, drifting), line-buffer write of (luma >> 2) - 16
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
    -- Line engine: kicked at each line start (and once in the zone for line 0), computes the NEXT line:
    -- the row cosine, and the zone plates' y^2 parts by second-order DDAs (Quad: (y - 540)^2)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_c : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1500 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_th  <= c_th0;
                if s_quad = '1' then                              -- Quad: the origin on the middle line
                    e_pyl <= c_pyql; e_dyl <= c_dyql;
                    e_pye <= c_pyqe; e_dye <= c_dyqe;
                else
                    e_pyl <= (others => '0'); e_dyl <= resize(c_dy0l, 32);
                    e_pye <= (others => '0'); e_dye <= resize(c_dy0e, 32);
                end if;
                e_lc  <= (others => '0');
            elsif g_start = '1' then
                e_th <= e_th + c_kyl;
                e_pyl <= e_pyl + e_dyl; e_dyl <= e_dyl + resize(c_ddyl, 32);
                e_pye <= e_pye + e_dye; e_dye <= e_dye + resize(c_ddye, 32);
                e_lc <= (others => '0');
            elsif e_lc /= 15 then
                e_lc <= e_lc + 1;
            end if;

            case to_integer(e_lc) is
                when 1 =>                      -- row cosine: quadrant fold
                    e_q <= e_th(23 downto 22);
                    if e_th(22) = '1' then r_vc_a <= not e_th(21 downto 14); else r_vc_a <= e_th(21 downto 14); end if;
                when 3 =>                      -- V = +-cos >> 3 (rows = +-256)
                    v_c := signed(resize(r_vc_q, 13));
                    if e_q = "01" or e_q = "10" then v_c := -v_c; end if;
                    e_v <= resize(shift_right(v_c, 3), 10);
                when 4 =>                      -- the line's values
                    n_lv  <= resize(e_v, 11) + C_LV0;
                    n_phl <= c_pcl + e_pyl;
                    n_phe <= c_pce + e_pye;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the two zone-plate phases by second-order DDAs, folded at the seam (hold one pixel, reflect the
    -- step); the mosaic block and band counters
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if s_zone = '1' then b_bc <= (others => '0'); end if;
            if g_start = '1' then
                -- Mosaic: the line values are held over bands of 4 / 3 / 2 lines (1080 / 720 / SD)
                if s_mos = '0' or b_bc = 0 then
                    b_phl <= n_phl; b_phe <= n_phe; l_v <= n_lv;
                    h_phl <= n_phl; h_phe <= n_phe;
                else
                    b_phl <= h_phl; b_phe <= h_phe;
                end if;
                if b_bc = c_mvm1 then b_bc <= (others => '0'); else b_bc <= b_bc + 1; end if;
                b_dl  <= c_d0l;
                b_de  <= c_d0e;
                b_px  <= (others => '0');
                b_fp  <= '0';
                b_mc  <= (others => '0');
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                if b_px = c_pfm1 then b_fp <= not s_swp; else b_fp <= '0'; end if;
                if b_fp = '1' then
                    b_dl <= signed(resize(c_ddl, 28)) - b_dl;
                    b_de <= signed(resize(c_dde, 28)) - b_de;
                else
                    b_phl <= unsigned(signed(b_phl) + resize(b_dl, 32));
                    b_dl  <= b_dl + signed(resize(c_ddl, 28));
                    b_phe <= unsigned(signed(b_phe) + resize(b_de, 32));
                    b_de  <= b_de + signed(resize(c_dde, 28));
                end if;
                if b_mc = c_mbm1 then b_mc <= (others => '0'); else b_mc <= b_mc + 1; end if;
            end if;
            s_lb_ra <= b_px + 9;                       -- the luma chain reads 9 px ahead
            if b_mc = 0 then b_ld <= '1'; else b_ld <= '0'; end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_f  : signed(13 downto 0);
        variable v_t  : signed(18 downto 0);
        variable v_l  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: ROM addresses: ring cosine (the turn fraction), gap waveform (16 turns)
            -- (+ the luma bend of the r^2 ramp: g_d in 1/64 turn of the gap phase, a quarter of it on the rings)
            r_cos_a <= b_phl(27 downto 18) + (g_d(7 downto 0) & "00");
            r_val_a <= b_phe(31 downto 22) + g_d;
            -- luma chain: box over 4 px, x etch gain (two partial products), the bend (video: t >> 10; Osc,
            -- centred on its mean: (t - 112 eg) >> 5, 32x)
            g_l1 <= unsigned(s_lb_q); g_l2 <= g_l1; g_l3 <= g_l2;
            v_l := resize(unsigned(s_lb_q), 10) + resize(g_l1, 10) + resize(g_l2, 10) + resize(g_l3, 10);
            g_box <= v_l(9 downto 2);
            g_ta <= g_box(3 downto 0) * c_eg;
            g_tb <= g_box(7 downto 4) * c_eg;
            g_t  <= resize(g_ta, 16) + shift_left(resize(g_tb, 16), 4);
            if s_src = '1' then
                v_t := signed(resize(g_t, 19)) - signed(resize(c_eg112, 19));
                g_d <= unsigned(v_t(14 downto 5));
            else
                g_d <= resize(g_t(15 downto 10), 10);
            end if;
            ld_sr(3) <= b_ld;
            for i in 4 to 9 loop ld_sr(i) <= ld_sr(i - 1); end loop;
            -- 3: ROM data (EBR); line buffer data
            -- 4: fabric copies
            p4_c <= signed(r_cos_q); p4_v <= r_val_q;
            -- 5: partial products
            p5_ha <= p4_c * signed('0' & c_ax(3 downto 0));
            p5_hb <= p4_c * signed('0' & c_ax(7 downto 4));
            p5_ea <= p4_v * c_dv(4 downto 0);
            p5_eb <= p4_v * c_dv(8 downto 5);
            -- 6: H = cos Ax >> 8; Ev = v Dv >> 5
            v_t := resize(p5_ha, 19) + shift_left(resize(p5_hb, 19), 4);
            p6_h <= resize(shift_right(v_t, 8), 10);
            p6_e <= resize(shift_right(resize(p5_ea, 18) + shift_left(resize(p5_eb, 18), 5), 5), 12);
            -- 7: line value + H; the etch level (video: luma eg >> 6; Osc: (luma - 112) eg >> 5, 2x; this
            --    luma sample is 6 px ahead of the bend's, an invisible offset)
            p7_s1 <= resize(l_v, 14) + resize(p6_h, 14);
            p7_e  <= p6_e;
            if s_src = '1' then
                v_t := signed(resize(g_t, 19)) - signed(resize(c_eg112, 19));
                p7_et <= resize(shift_right(v_t, 5), 14);
            else
                p7_et <= signed(resize(shift_right(g_t, 6), 14));
            end if;
            -- 8
            p8_a <= p7_s1 - signed(resize(p7_e, 14));
            p8_b <= p7_et;
            -- 9: F + 704, palette index = clamp(>> 1)
            v_f := p8_a + p8_b;
            if v_f < 0 then
                p9_i <= (others => '0');
            elsif v_f(13 downto 10) /= "0000" then
                p9_i <= (others => '1');
            else
                p9_i <= unsigned(v_f(9 downto 1));
            end if;
            -- 10: mosaic sample-and-hold (38 virtual px; the lines are held in the beam)
            if s_mos = '0' or ld_sr(9) = '1' then p10_i <= p9_i; end if;
            -- 11: palette address (bit 9 = Chevron)
            r_pal_a <= s_pal & p10_i;
            -- 12: ROM data (r_pal_q)
            -- 13: fabric copy
            p13_q <= r_pal_q;
            -- 14: colours
            p14_y <= unsigned(p13_q(31 downto 22));
            p14_u <= unsigned(p13_q(21 downto 12));
            p14_v <= unsigned(p13_q(11 downto 2));
            -- 15: compose; black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= C_BKY; s_u15 <= C_BKU; s_v15 <= C_BKV;
            else
                s_y15 <= p14_y; s_u15 <= p14_u; s_v15 <= p14_v;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes (the HD HDMI bar fix and the
    -- capture's horizontal softness), luma 13/16 and the blanking gate.  The running 8-sums snap back to
    -- black once 8 blank samples are in, so a glitched sample can't offset the colours for good.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- box A: stage 16 sums, stage 17 output
            s_cay(0) <= s_y15; s_cau(0) <= s_u15; s_cav(0) <= s_v15;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= C_BKY & "000"; s_bau <= C_BKU & "000"; s_bav <= C_BKV & "000";
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
                s_y17 <= C_BKY; s_u17 <= C_BKU; s_v17 <= C_BKV;
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            -- box B: stage 18 sums, stage 19 output
            s_cby(0) <= s_y17; s_cbu(0) <= s_u17; s_cbv(0) <= s_v17;
            for i in 1 to 7 loop s_cby(i) <= s_cby(i - 1); s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bby <= C_BKY & "000"; s_bbu <= C_BKU & "000"; s_bbv <= C_BKV & "000";
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

end architecture pods;
