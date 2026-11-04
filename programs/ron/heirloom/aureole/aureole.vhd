-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Aureole v0.1.1 -- Heirloom from the video clips, from WIN_20230314_23_42_54 at 7.5 s.
-- v0.1.1: S9 Gold is a full-frame gold halo on every line (was only the afterglow below the warm cap,
-- off screen at the default phase); S9 Blue (the default) is bit-identical with v0.1.0.
--
-- The clip: a soft cyan bloom figure in nested vertical bands (blue, dark, a thin teal line, olive, brown,
-- yellow-green, the cyan plateau), a dark-red floor at the bottom with a thin bright line and a horned wave
-- along it, and a warm cap (white core, orange, red, pink) that scrolls down through the picture every
-- ~14 s with sharp horns on its lower edge.
--
-- The patch, as fitted (aureole.py has every table, .heirloom_work/rb_aureole/notes.md the measurements):
--   F     = 511 - k (x - xc)^2 + v(line) - drop(pos') + bend(video)   a parabola in x through the cool
--           palette PC (xc = 1000 virtual px; k = 4.9e-4 F/px^2 left, 5.6e-4 right)
--   pos   = (y - floorA) mod 2200          a vertical waveform scrolling down 165 px/s: the cool band
--           (1300 px) and the warm band (900 px) meet at two "floors"
--   pos'  = pos + horns(x) + etch(video)   the horn wave: a skewed parabolic sawtooth, ~5.5 per width
--   near a floor: the field drops (every band walks down the palette, through the bright teal entry = the
--           thin line), the cool layer gates off (B first), the warm layer PW rises (R first, then G and B);
--           a blue-white glow below the warm->cool floor
--   RGB   = PC(F) gc(Z) + PW(F) gw(Z) + glow,  then RGB -> YUV (U register = Cr), luma pre-scaled 16/13
--
--   K1  Width    the bands' spread: x0.25 .. x4 of the clip's parabola
--   K2  Horns    the horn wave's height, 0 .. +-400 px (80 = the clip)
--   K3  Skew     the horn wave's shape: slow rise / sharp fall .. triangle .. sharp rise / slow fall
--   K4  Speed    bipolar: the warm cap scrolls down (+) or up (-), still in the centre
--   K5  Teeth    horns per screen width, 2 .. 12
--   K6  Etch     how far the picture (or the Osc) pushes the floor and bends the bands
--   S7  Palette  Cyan (the clip) / Gold (chroma rotated 240 degrees)
--   S8  Twin     one arch / two arches side by side
--   S9  Glow     Blue (the clip's blue-white trail below the warm cap) / Gold (a gold halo over the whole frame)
--   S10 Shape    Arch (the clip) / Well (the parabola inverted: a dark slot between cyan walls)
--   S11 Source   Video / Osc
--   P12 BLOOM    the warm cap's share of the cycle: all cool with a thin seam .. the clip (489) .. the bloom
--                fills the screen with a cyan slot
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide), a vertical table (1024 x 16, 4 px per
-- entry: Z and the field drop by position) refilled every field, a per-line engine (the line's position,
-- the field bulge and the glow), a beam with the exact parabola DDA (reloading at the centre; Twin mirrors D
-- at the mid point) and the horn wave DDAs, a 17-stage RGB pixel path with an RGB->YUV matrix, then the output
-- conditioning (chroma through one [1 2 2 2 2 2 2 2 1]/16 box on a block-RAM ring, luma [1 2 1] and 13/16)
-- before the blanking gate.
-- Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated avid; one latency (C_LAT 24) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.aureole_pkg.all;
use work.all;

architecture aureole of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

    constant C_P    : integer := 2200;                -- vertical period, virtual px
    constant C_P8   : integer := 563200;              -- Q8
    constant C_XC   : integer := 2004;                -- arch centre (h): every mode's beam lands on it exactly
    constant C_XC1  : integer := 960;                 -- twin centres (h)
    constant C_XC2  : integer := 2880;
    constant C_XMID : integer := 1920;
    constant C_KBL  : integer := 128;                 -- parabola k, Q20 F per h^2 (left / right)
    constant C_KBR  : integer := 146;

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
    function f_sat8(v : unsigned) return unsigned is
    begin
        if v > 255 then return to_unsigned(255, 8); else return resize(v, 8); end if;
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
    signal r_sh_a : unsigned(9 downto 0) := (others => '0');
    signal r_sh_q : unsigned(7 downto 0) := (others => '0');
    signal r_pc_a : unsigned(9 downto 0) := (others => '0');
    signal r_pc_q : std_logic_vector(23 downto 0) := (others => '0');
    signal r_pw_a : unsigned(8 downto 0) := (others => '0');
    signal r_pw_q : std_logic_vector(23 downto 0) := (others => '0');

    -- vertical table: {Z (signed 8), drop (8)} by pos' / 4, written by the fill in the vblank zone only
    type t_vt is array (0 to 1023) of std_logic_vector(15 downto 0);
    signal vt : t_vt := (others => x"0000");
    attribute no_rw_check : boolean;
    attribute no_rw_check of vt : signal is true;
    signal t_we : std_logic := '0';
    signal t_wa, t_ra : unsigned(9 downto 0) := (others => '0');
    signal t_wd, t_rq : std_logic_vector(15 downto 0) := (others => '0');

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
    signal s_zc   : unsigned(11 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(205 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(820 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(357 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(400 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(489 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(761, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_pal, s_twin, s_gold, s_well, s_src : std_logic := '0';

    -- per-field constants
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_sh   : signed(2 downto 0) := (others => '0');
    signal z_mt   : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_sh2  : unsigned(2 downto 0) := (others => '0');
    signal z_a    : unsigned(23 downto 0) := (others => '0');
    signal c_kl, c_kr : unsigned(12 downto 0) := to_unsigned(128, 13);   -- Q20 F/h^2 (twin: x4)
    signal c_ddl, c_ddr : unsigned(17 downto 0) := (others => '0');      -- 2 k xs^2
    signal c_am   : unsigned(31 downto 0) := (others => '0');            -- A2 at the line start
    signal c_dm   : signed(27 downto 0) := (others => '0');              -- D at the line start
    signal c_dr0  : signed(27 downto 0) := (others => '0');              -- D at a centre crossing: kR xs^2
    signal c_ep0, c_ep1, c_ep2 : unsigned(11 downto 0) := to_unsigned(4095, 12);
    signal c_d0a  : unsigned(11 downto 0) := (others => '0');            -- |d0| (h)
    signal c_xb0  : unsigned(11 downto 0) := to_unsigned(16, 12);
    signal c_xs2  : unsigned(4 downto 0) := to_unsigned(4, 5);           -- xs^2
    signal c_inc  : unsigned(10 downto 0) := to_unsigned(186, 11);       -- wave phase step per pixel, Q16
    signal c_incp : unsigned(9 downto 0) := to_unsigned(186, 10);        -- per virtual px
    signal c_a    : unsigned(8 downto 0) := to_unsigned(80, 9);          -- horns, px
    signal c_s    : unsigned(13 downto 0) := to_unsigned(3074, 14);      -- rise slope, Q8
    signal c_sneg : std_logic := '0';
    signal c_as   : unsigned(22 downto 0) := (others => '0');            -- A S
    signal c_ai   : unsigned(19 downto 0) := (others => '0');            -- A inc
    signal c_dw0, c_dw2 : signed(21 downto 0) := (others => '0');
    signal c_incr : unsigned(16 downto 0) := (others => '0');
    signal c_wrst : signed(23 downto 0) := (others => '0');
    signal c_lw, c_lc : unsigned(11 downto 0) := to_unsigned(899, 12);
    signal c_e    : unsigned(7 downto 0) := to_unsigned(100, 8);
    signal c_step : unsigned(11 downto 0) := (others => '0');            -- scroll per field, Q8 px
    signal c_off8 : unsigned(19 downto 0) := to_unsigned(51200, 20);     -- line-0 position, Q8, mod P8

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

    -- table fill (zone)
    signal q_run : std_logic := '0';
    signal q_i   : unsigned(10 downto 0) := (others => '0');
    signal q1_v, q2_v, q3_v : std_logic := '0';
    signal q1_i, q2_i, q3_i : unsigned(9 downto 0) := (others => '0');
    signal q1_cool : std_logic := '0';
    signal q1_da, q1_db : unsigned(11 downto 0) := (others => '0');
    signal q2_z, q3_z : signed(7 downto 0) := (others => '0');

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
    signal e_pos8 : unsigned(19 downto 0) := to_unsigned(51200, 20);
    signal e_pos  : unsigned(11 downto 0) := (others => '0');
    signal e_cool : std_logic := '1';
    signal e_da   : unsigned(11 downto 0) := (others => '0');
    signal e_b, e_g : unsigned(7 downto 0) := (others => '0');
    signal e_gq   : unsigned(7 downto 0) := to_unsigned(160, 8);    -- Gold halo factor of the next line
    signal e_sel  : std_logic := '0';
    signal n_pos  : unsigned(11 downto 0) := (others => '0');
    signal n_v    : signed(6 downto 0) := (others => '0');
    signal n_gr, n_gg, n_gb : unsigned(6 downto 0) := (others => '0');
    signal l_pos  : unsigned(11 downto 0) := (others => '0');
    signal l_v    : signed(6 downto 0) := (others => '0');
    signal l_gr, l_gg, l_gb : unsigned(6 downto 0) := (others => '0');

    -- beam
    signal b_x, b_x1, b_x2 : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_a2  : unsigned(31 downto 0) := (others => '0');
    signal b_d   : signed(27 downto 0) := (others => '0');
    signal b_dd  : unsigned(17 downto 0) := (others => '0');
    signal b_fc  : std_logic := '0';
    signal b_fm  : std_logic := '0';                           -- the pending event is the twin mid point (D mirrored)
    signal b_ev  : unsigned(1 downto 0) := (others => '0');
    signal b_pos : unsigned(11 downto 0) := to_unsigned(4095, 12);
    signal b_u   : unsigned(15 downto 0) := (others => '0');
    signal b_ws  : signed(26 downto 0) := (others => '0');
    signal b_wd  : signed(21 downto 0) := (others => '0');
    signal b_wr  : signed(23 downto 0) := (others => '0');

    -- pixel path
    signal p1_fq  : unsigned(11 downto 0) := (others => '0');
    signal p1_w   : signed(23 downto 0) := (others => '0');
    signal p2_fh  : unsigned(8 downto 0) := (others => '0');
    signal p2_dw  : signed(9 downto 0) := (others => '0');
    signal p3_fh  : unsigned(8 downto 0) := (others => '0');
    signal p3_dw  : signed(9 downto 0) := (others => '0');
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p4_fh  : unsigned(8 downto 0) := (others => '0');
    signal p4_dw  : signed(9 downto 0) := (others => '0');
    signal p4_pe  : unsigned(15 downto 0) := (others => '0');
    signal p5_pos : signed(13 downto 0) := (others => '0');
    signal p5_fb, p6_fb, p7_fb, p8_fb : unsigned(9 downto 0) := (others => '0');
    signal p8_z   : signed(7 downto 0) := (others => '0');
    signal p8_dr  : unsigned(7 downto 0) := (others => '0');
    signal p9_z, p10_z, p11_z : signed(7 downto 0) := (others => '0');
    signal p12_rc, p12_rw : std_logic_vector(23 downto 0) := (others => '0');
    signal p12_gcr, p12_gcg, p12_gcb, p12_gwr, p12_gwg, p12_gwb : unsigned(7 downto 0) := (others => '0');
    signal p13_rc, p13_rw, p13_gc, p13_gw, p13_bc, p13_bw : unsigned(15 downto 0) := (others => '0');
    signal p14_r, p14_g, p14_b : unsigned(7 downto 0) := (others => '0');
    signal p15_r, p15_b : unsigned(7 downto 0) := (others => '0');
    signal p15_yr, p15_yg, p15_yb : unsigned(15 downto 0) := (others => '0');
    signal p16_r, p16_b, p16_y8 : unsigned(7 downto 0) := (others => '0');
    signal p17_cr, p17_cb : signed(18 downto 0) := (others => '0');
    signal p17_y10 : unsigned(10 downto 0) := (others => '0');
    signal s_y18 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u18, s_v18 : unsigned(7 downto 0) := to_unsigned(128, 8);

    -- output conditioning: chroma (8 bits) through one [1 2 2 2 2 2 2 2 1]/16 box (two adjacent 8-sums, the
    -- 8-back tap from one block-RAM ring), luma [1 2 1]
    type t_ring is array (0 to 255) of std_logic_vector(15 downto 0);
    signal ra : t_ring := (others => x"8080");
    attribute no_rw_check of ra : signal is true;
    signal ra_wa, ra_ra : unsigned(7 downto 0) := (others => '0');
    signal ra_q, ra_t : std_logic_vector(15 downto 0) := x"8080";
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah : std_logic_vector(7 downto 0) := (others => '0');   -- generated avid of the last 8 inputs of the box
    signal s_y19, s_y20, s_y21, s_y22, s_y23 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u20, s_v20, s_u22, s_v22 : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Line buffer (collisions possible: read-before-write kept), ROMs, table
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
            r_ex_q <= C_AU_EXP2(to_integer(r_ex_a));
            r_sh_q <= C_AU_SHAPE(to_integer(r_sh_a));
            r_pc_q <= C_AU_PC(to_integer(r_pc_a));
            r_pw_q <= C_AU_PW(to_integer(r_pw_a));
        end if;
    end process p_rom;

    p_vt : process(clk)
    begin
        if rising_edge(clk) then
            t_rq <= vt(to_integer(t_ra));
            if t_we = '1' then vt(to_integer(t_wa)) <= t_wd; end if;
        end if;
    end process p_vt;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            ra_q <= ra(to_integer(ra_ra));
            ra(to_integer(ra_wa)) <= std_logic_vector(s_u18) & std_logic_vector(s_v18);
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_e  : signed(10 downto 0);
        variable v_u  : unsigned(31 downto 0);
        variable v_d  : signed(27 downto 0);
        variable v_t  : unsigned(15 downto 0);
        variable v_k  : unsigned(12 downto 0);
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
            elsif s_zc /= 4095 then
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
                s_pal  <= registers_in(6)(0);
                s_twin <= registers_in(6)(1);
                s_gold <= registers_in(6)(2);
                s_well <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9); c_xs2 <= to_unsigned(16, 5);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9); c_xs2 <= to_unsigned(9, 5);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9); c_xs2 <= to_unsigned(4, 5);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, run
            -- from the zone start by its own phase / channel counters (no zc decode on these enables)
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
            -- load + 17 (stores at load + 18).  The slot number is decoded at 30 + 24 k and the operands
            -- are pre-muxed into z_fa / z_fb at 31 + 24 k, so no zc decode sits on the multiplier's path.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30  => z_sel <= to_unsigned(0, 5);  z_pre <= '1';
                    when 54  => z_sel <= to_unsigned(1, 5);  z_pre <= '1';
                    when 78  => z_sel <= to_unsigned(2, 5);  z_pre <= '1';
                    when 102 => z_sel <= to_unsigned(3, 5);  z_pre <= '1';
                    when 126 => z_sel <= to_unsigned(4, 5);  z_pre <= '1';
                    when 150 => z_sel <= to_unsigned(5, 5);  z_pre <= '1';
                    when 174 => z_sel <= to_unsigned(10, 5); z_pre <= '1';
                    when 198 => z_sel <= to_unsigned(12, 5); z_pre <= '1';
                    when 222 => z_sel <= to_unsigned(13, 5); z_pre <= '1';
                    when 246 => z_sel <= to_unsigned(14, 5); z_pre <= '1';
                    when 270 => z_sel <= to_unsigned(15, 5); z_pre <= '1';
                    when 294 => z_sel <= to_unsigned(16, 5); z_pre <= '1';
                    when 318 => z_sel <= to_unsigned(17, 5); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0  => z_fa <= to_unsigned(C_KBL, 36);          z_fb <= z_mt;                        -- kL
                    when 1  => z_fa <= to_unsigned(C_KBR, 36);          z_fb <= z_mt;                        -- kR
                    when 2  => z_fa <= resize(s_g5(12 downto 3), 36);   z_fb <= to_unsigned(341, 16);        -- teeth
                    when 3  => z_fa <= resize(s_vm, 36);                z_fb <= resize(s_vm, 16);            -- speed^2
                    when 4  => z_fa <= resize(z_a(17 downto 0), 36);    z_fb <= to_unsigned(3105, 16);       -- step
                    when 5  => z_fa <= resize(s_gp(12 downto 3), 36);   z_fb <= to_unsigned(450, 16);        -- bloom
                    when 10 => z_fa <= resize(c_a, 36);                 z_fb <= resize(c_s, 16);             -- A S
                    when 12 => z_fa <= resize(c_as(22 downto 7), 36);   z_fb <= resize(c_inc, 16);           -- (AS >> 7) inc
                    when 13 => z_fa <= resize(c_a, 36);                 z_fb <= resize(c_inc, 16);           -- A inc
                    when 14 => z_fa <= resize(c_ai, 36);                z_fb <= resize(c_inc, 16);           -- A inc^2
                    when 15 => z_fa <= resize(c_d0a, 36);               z_fb <= resize(c_d0a, 16);           -- d0^2
                    when 16 => z_fa <= resize(z_a, 36);                 z_fb <= resize(c_kl, 16);            -- d0^2 kL
                    when 17 =>                                                                               -- |D0| = (2 |d0| xs - xs^2) kL
                        case to_integer(s_xs) is
                            when 4      => z_fa <= resize(shift_left(resize(c_d0a, 16), 3) - 16, 36);
                            when 3      => z_fa <= resize(shift_left(resize(c_d0a, 16), 2) + shift_left(resize(c_d0a, 16), 1) - 9, 36);
                            when others => z_fa <= resize(shift_left(resize(c_d0a, 16), 2) - 4, 36);
                        end case;
                        z_fb <= resize(c_kl, 16);
                    when others => z_fa <= (others => '0');             z_fb <= (others => '0');
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
                    -- motion (last field's step); beam origin; constants from the switches
                    when 20 =>
                        if s_vneg = '1' then
                            if c_off8 + c_step >= C_P8 then c_off8 <= c_off8 + c_step - C_P8; else c_off8 <= c_off8 + c_step; end if;
                        else
                            if c_off8 < c_step then c_off8 <= c_off8 + C_P8 - c_step; else c_off8 <= c_off8 - c_step; end if;
                        end if;
                        c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);
                        c_e <= s_g6(12 downto 5);
                    when 21 =>
                        -- horns A = (k2 * 100) >> 8 ; width exponent ; skew exponent
                        v_u := shift_left(resize(s_g2(12 downto 3), 32), 6) + shift_left(resize(s_g2(12 downto 3), 32), 5)
                               + shift_left(resize(s_g2(12 downto 3), 32), 2);
                        c_a <= v_u(16 downto 8);
                        v_e := signed(resize(s_g1(12 downto 3), 11)) - 512;
                        r_ex_a <= unsigned(v_e(7 downto 0)); z_sh <= v_e(10 downto 8);
                        if s_twin = '1' then
                            c_ep0 <= to_unsigned(C_XC1, 12); c_ep1 <= to_unsigned(C_XMID, 12); c_ep2 <= to_unsigned(C_XC2, 12);
                            if c_xb0 < C_XC1 then c_d0a <= to_unsigned(C_XC1, 12) - c_xb0; else c_d0a <= c_xb0 - C_XC1; end if;
                        else
                            c_ep0 <= to_unsigned(C_XC, 12); c_ep1 <= to_unsigned(4095, 12); c_ep2 <= to_unsigned(4095, 12);
                            c_d0a <= to_unsigned(C_XC, 12) - c_xb0;
                        end if;
                    when 22 =>
                        -- skew: e8 = |k3 - 512| * 2202 >> 10
                        v_e := signed(resize(s_g3(12 downto 3), 11)) - 512;
                        c_sneg <= v_e(10);
                        if v_e(10) = '1' then v_e := -v_e; end if;
                        v_m := unsigned(v_e(9 downto 0));
                        z_a <= shift_left(resize(v_m, 24), 1) + resize(v_m(9 downto 3), 24);
                    when 23 => z_mt <= r_ex_q;                                      -- width mantissa
                    when 24 => r_ex_a <= z_a(7 downto 0); z_sh2 <= z_a(10 downto 8);
                    when 27 =>                                                      -- S = exp2 >> (6 - sh2)
                        case to_integer(z_sh2) is
                            when 0      => c_s <= resize(r_ex_q(15 downto 6), 14);
                            when 1      => c_s <= resize(r_ex_q(15 downto 5), 14);
                            when 2      => c_s <= resize(r_ex_q(15 downto 4), 14);
                            when 3      => c_s <= resize(r_ex_q(15 downto 3), 14);
                            when others => c_s <= r_ex_q(15 downto 2);
                        end case;
                    -- slot 0 (kL) at 50, slot 1 (kR) at 74: k = kb * mt >> (15 - sh), twin x4
                    when 50 =>
                        case to_integer(z_sh) is
                            when -2     => v_k := resize(f_acc(29 downto 17), 13);
                            when -1     => v_k := resize(f_acc(28 downto 16), 13);
                            when 0      => v_k := resize(f_acc(27 downto 15), 13);
                            when others => v_k := resize(f_acc(26 downto 14), 13);
                        end case;
                        if s_twin = '1' then c_kl <= v_k(10 downto 0) & "00"; else c_kl <= v_k; end if;
                    when 74 =>
                        case to_integer(z_sh) is
                            when -2     => v_k := resize(f_acc(29 downto 17), 13);
                            when -1     => v_k := resize(f_acc(28 downto 16), 13);
                            when 0      => v_k := resize(f_acc(27 downto 15), 13);
                            when others => v_k := resize(f_acc(26 downto 14), 13);
                        end case;
                        if s_twin = '1' then c_kr <= c_kl; else c_kr <= v_k; end if;   -- Twin: one k (the mid-point mirror needs it)
                    when 75 =>
                        case to_integer(s_xs) is
                            when 4      => c_ddl <= shift_left(resize(c_kl, 18), 5); c_ddr <= shift_left(resize(c_kr, 18), 5);
                            when 3      => c_ddl <= shift_left(resize(c_kl, 18), 4) + shift_left(resize(c_kl, 18), 1);
                                           c_ddr <= shift_left(resize(c_kr, 18), 4) + shift_left(resize(c_kr, 18), 1);
                            when others => c_ddl <= shift_left(resize(c_kl, 18), 3); c_ddr <= shift_left(resize(c_kr, 18), 3);
                        end case;
                    when 98 =>                                                      -- teeth: inc_px = 68 + (k5 341 >> 10)
                        c_incp <= to_unsigned(68, 10) + f_acc(19 downto 10);
                    when 99 =>
                        case to_integer(s_xs) is
                            when 4      => c_inc <= c_incp & '0';
                            when 3      => c_inc <= resize(c_incp, 11) + resize(c_incp(9 downto 1), 11);
                            when others => c_inc <= resize(c_incp, 11);
                        end case;
                    when 122 => z_a <= resize(f_acc(17 downto 0), 24);              -- speed m^2
                    when 146 => c_step <= f_acc(29 downto 18);                      -- step8 = m^2 3105 >> 18
                    when 170 =>                                                     -- bloom: Lw = 40 + (p12 450 >> 8)
                        c_lw <= to_unsigned(40, 12) + f_acc(19 downto 8);
                    when 171 => c_lc <= to_unsigned(C_P, 12) - c_lw;
                    when 194 => c_as <= f_acc(22 downto 0);                         -- A S
                    when 195 => c_wrst <= signed(shift_left(resize(c_a, 24), 8)) - signed(resize(c_as, 24));
                    when 218 => c_incr <= f_acc(25 downto 9);                       -- incR
                    when 242 => c_ai <= f_acc(19 downto 0);                         -- A inc
                    when 266 =>                                                     -- A inc^2 -> Dw0, Dw2
                        c_dw0 <= -signed(resize(c_ai, 22) + resize(f_acc(29 downto 16), 22));
                        c_dw2 <= -signed(resize(f_acc(29 downto 15), 22));
                    when 290 => z_a <= f_acc(23 downto 0);                          -- d0^2
                    when 314 => c_am <= f_acc(31 downto 0);                         -- A2 at the line start
                    when 338 => c_dm <= -signed(resize(f_acc(27 downto 0), 28));    -- D at the line start (descending)
                    when 339 =>
                        -- D at a centre crossing (the beam lands on it exactly): kR xs^2
                        case to_integer(s_xs) is
                            when 4      => c_dr0 <= signed(shift_left(resize(c_kr, 28), 4));
                            when 3      => c_dr0 <= signed(shift_left(resize(c_kr, 28), 3) + resize(c_kr, 28));
                            when others => c_dr0 <= signed(shift_left(resize(c_kr, 28), 2));
                        end case;
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
    -- Vertical table fill (vblank zone, zc 600 .. ~1630): entry i = position
    -- 4 i; cool band below Lc (dA from the warm->cool floor, dB to the
    -- cool->warm floor), warm band above.  Z = -dist/2 (cool) or +dist/4
    -- (warm) from the nearest floor, the drop from the shape ROM by
    -- min(1.5 dA, dB) (cool) or min(dA, dB) (warm).  Four pipelined stages.
    ---------------------------------------------------------------------------
    p_fill : process(clk)
        variable v_pos : unsigned(11 downto 0);
        variable v_dd  : unsigned(12 downto 0);
        variable v_dz  : unsigned(11 downto 0);
        variable v_da3 : unsigned(12 downto 0);
    begin
        if rising_edge(clk) then
            if s_zone = '1' and s_zc = 600 then
                q_run <= '1'; q_i <= (others => '0');
            elsif q_run = '1' then
                q_i <= q_i + 1;
                if q_i = 1023 then q_run <= '0'; end if;
            end if;
            -- stage 1: band and distances
            q1_v <= q_run; q1_i <= q_i(9 downto 0);
            v_pos := q_i(9 downto 0) & "00";
            if v_pos < c_lc then
                q1_cool <= '1'; q1_da <= v_pos; q1_db <= c_lc - v_pos;
            else
                q1_cool <= '0'; q1_da <= v_pos - c_lc; q1_db <= to_unsigned(C_P, 12) - v_pos;
            end if;
            -- stage 2: drop distance -> ROM address, Z
            q2_v <= q1_v; q2_i <= q1_i;
            v_da3 := resize(q1_da, 13) + resize(q1_da(11 downto 1), 13);
            if q1_cool = '1' then
                if v_da3 < resize(q1_db, 13) then v_dd := v_da3; else v_dd := resize(q1_db, 13); end if;
            else
                if q1_da < q1_db then v_dd := resize(q1_da, 13); else v_dd := resize(q1_db, 13); end if;
            end if;
            if v_dd > 1023 then v_dd := to_unsigned(1023, 13); end if;
            if q1_da < q1_db then v_dz := q1_da; else v_dz := q1_db; end if;
            if q1_cool = '1' then
                if v_dz(11 downto 1) > 127 then q2_z <= to_signed(-127, 8); else q2_z <= -signed(resize(v_dz(7 downto 1), 8)); end if;
            else
                if v_dz(11 downto 2) > 127 then q2_z <= to_signed(127, 8); else q2_z <= signed(resize(v_dz(8 downto 2), 8)); end if;
            end if;
            if q1_v = '1' then r_sh_a <= "10" & v_dd(9 downto 2); else r_sh_a <= '0' & e_sel & e_da(9 downto 2); end if;
            -- stage 3: ROM read in flight
            q3_v <= q2_v; q3_i <= q2_i; q3_z <= q2_z;
            -- stage 4: write (the ROM data of stage 3's entry is in r_sh_q now)
            t_we <= q3_v; t_wa <= q3_i;
            if (q3_i & "00") < C_P then
                t_wd <= std_logic_vector(q3_z) & std_logic_vector(r_sh_q);
            else
                t_wd <= (others => '0');
            end if;
        end if;
    end process p_fill;

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
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;  -- ot * 96
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
    -- 0, after the table fill), computes the NEXT line: its position in the
    -- vertical cycle, the field bulge v and the glow (shape ROM reads in
    -- hblank, while the fill is idle)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_b25 : unsigned(12 downto 0);
        variable v_g   : unsigned(14 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1700 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_pos8 <= c_off8;
                e_lc <= (others => '0');
            elsif g_start = '1' then
                if e_pos8 + resize(s_ys, 20) >= C_P8 then
                    e_pos8 <= e_pos8 + resize(s_ys, 20) - C_P8;
                else
                    e_pos8 <= e_pos8 + resize(s_ys, 20);
                end if;
                e_lc <= (others => '0');
                l_pos <= n_pos; l_v <= n_v; l_gr <= n_gr; l_gg <= n_gg; l_gb <= n_gb;
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            case to_integer(e_lc) is
                when 1 =>
                    e_pos <= e_pos8(19 downto 8);
                when 2 =>
                    e_sel <= '0';
                    if e_pos < c_lc then
                        e_cool <= '1';
                        if e_pos > 1023 then e_da <= to_unsigned(1023, 12); else e_da <= e_pos; end if;
                    else
                        e_cool <= '0';
                        if e_pos - c_lc > 1023 then e_da <= to_unsigned(1023, 12); else e_da <= e_pos - c_lc; end if;
                    end if;
                when 5 => e_b <= r_sh_q;                                             -- bump (address set at 3 by p_fill's mux)
                when 6 => e_sel <= '1';
                when 9 => e_g <= r_sh_q;                                             -- glow
                when 10 =>
                    n_pos <= e_pos;
                    v_b25 := resize(e_b, 13) + shift_left(resize(e_b, 13), 3) + shift_left(resize(e_b, 13), 4);
                    if e_cool = '1' then
                        n_v <= signed(resize(v_b25(12 downto 8), 7)) - 25;
                    else
                        n_v <= to_signed(-40, 7);
                    end if;
                    -- Gold halo factor: 160 on every line (cool band and warm cap), plus 5/16 of the glow shape
                    -- below the warm cap (max 238)
                    if e_cool = '1' then
                        e_gq <= to_unsigned(160, 8) + shift_right(e_g, 2) + shift_right(e_g, 4);
                    else
                        e_gq <= to_unsigned(160, 8);
                    end if;
                when 11 =>
                    if s_gold = '1' then
                        -- Gold: a full-frame gold lift (+70, +45, +5 .. +104, +67, +7), whatever the scroll phase
                        v_g := shift_left(resize(e_gq, 15), 6) + shift_left(resize(e_gq, 15), 5) + shift_left(resize(e_gq, 15), 4);   -- 112 g
                        n_gr <= v_g(14 downto 8);
                        v_g := shift_left(resize(e_gq, 15), 6) + shift_left(resize(e_gq, 15), 3);                                        -- 72 g
                        n_gg <= v_g(14 downto 8);
                        v_g := shift_left(resize(e_gq, 15), 3);                                                                          -- 8 g
                        n_gb <= v_g(14 downto 8);
                    elsif e_cool = '0' then
                        n_gr <= (others => '0'); n_gg <= (others => '0'); n_gb <= (others => '0');
                    else
                        v_g := shift_left(resize(e_g, 15), 4) + shift_left(resize(e_g, 15), 3) + shift_left(resize(e_g, 15), 2);   -- 28 g
                        n_gr <= v_g(14 downto 8);
                        v_g := shift_left(resize(e_g, 15), 5) + shift_left(resize(e_g, 15), 4);                                       -- 48 g
                        n_gg <= v_g(14 downto 8);
                        v_g := shift_left(resize(e_g, 15), 6);                                                                         -- 64 g
                        n_gb <= v_g(14 downto 8);
                    end if;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: the parabola A2 = k d^2 as an exact second-order DDA in h
    -- (reloaded at the centre crossing and, for Twin, at the mid point),
    -- and the horn wave: phase u (Q16 per period), the slow side as a
    -- second-order DDA (Q16 px), the rise as a ramp (Q8 px), both reset at
    -- the wrap.  The event flag and residual are registered a clock ahead.
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_u  : unsigned(16 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x  <= c_xb0;
                b_x1 <= c_xb0 + s_xs;
                b_x2 <= c_xb0 + shift_left(resize(s_xs, 12), 1);
                b_px <= (others => '0');
                b_a2 <= c_am;
                b_d  <= c_dm;
                b_dd <= c_ddl;
                b_fc <= '0';
                b_fm <= '0';
                b_ev <= "00";
                b_pos <= c_ep0;
                b_u  <= (others => '0');
                b_ws <= signed(shift_left(resize(c_a, 27), 16));
                b_wd <= c_dw0;
                b_wr <= c_wrst;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x1;
                b_x1 <= b_x2;
                b_x2 <= b_x2 + s_xs;
                -- parabola for the next pixel
                -- centre crossing: reload A2 = 0, D = k xs^2.  Twin mid point (k equal on both sides in
                -- Twin): the normal step lands exactly on k 960^2, and the next step descends, so D turns
                -- into -D (the step that arrived, mirrored).
                if b_fc = '1' and b_fm = '0' then
                    b_a2 <= (others => '0'); b_d <= c_dr0; b_dd <= c_ddr;
                else
                    b_a2 <= unsigned(signed(b_a2) + resize(b_d, 32));
                    if b_fc = '1' then
                        b_d <= -b_d;
                    else
                        b_d <= b_d + signed(resize(b_dd, 28));
                    end if;
                end if;
                -- the event for the step after next: x(n+2) = b_x2 is the first x >= pos
                if b_x2 >= b_pos and b_x1 < b_pos then
                    b_fc <= '1';
                    b_fm <= '0';
                    if b_ev = 1 and s_twin = '1' then b_fm <= '1'; end if;
                    b_ev <= b_ev + 1;
                    case to_integer(b_ev) is
                        when 0      => b_pos <= c_ep1;
                        when 1      => b_pos <= c_ep2;
                        when others => b_pos <= to_unsigned(4095, 12);
                    end case;
                else
                    b_fc <= '0';
                end if;
                -- horn wave for the next pixel
                v_u := resize(b_u, 17) + resize(c_inc, 17);
                b_u <= v_u(15 downto 0);
                if v_u(16) = '1' then
                    b_ws <= signed(shift_left(resize(c_a, 27), 16));
                    b_wd <= c_dw0;
                    b_wr <= c_wrst;
                else
                    b_ws <= b_ws + resize(b_wd, 27);
                    b_wd <= b_wd + c_dw2;
                    b_wr <= b_wr + signed(resize(c_incr, 24));
                end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_w   : signed(23 downto 0);
        variable v_a8  : signed(23 downto 0);
        variable v_ws8 : signed(23 downto 0);
        variable v_s   : signed(13 downto 0);
        variable v_f   : signed(11 downto 0);
        variable v_fi  : unsigned(8 downto 0);
        variable v_pm  : signed(11 downto 0);
        variable v_d9  : signed(8 downto 0);
        variable v_nz, v_pz, v_t : unsigned(8 downto 0);
        variable v_g   : unsigned(11 downto 0);
        variable v_m   : unsigned(16 downto 0);
        variable v_y   : unsigned(18 downto 0);
        variable v_c   : signed(18 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: field quotient and the larger of the two wave parts (Q8 px)
            p1_fq <= b_a2(31 downto 20);
            v_ws8 := resize(b_ws(26 downto 8), 24);
            if v_ws8 > b_wr then p1_w <= v_ws8; else p1_w <= b_wr; end if;
            -- 2: field height (arch or well), the clipped displacement in px
            if s_well = '1' then
                if p1_fq > 511 then p2_fh <= to_unsigned(511, 9); else p2_fh <= p1_fq(8 downto 0); end if;
            else
                if p1_fq >= 511 then p2_fh <= (others => '0'); else p2_fh <= to_unsigned(511, 9) - p1_fq(8 downto 0); end if;
            end if;
            v_w := p1_w;
            v_a8 := signed(shift_left(resize(c_a, 24), 8));
            if v_w > v_a8 then v_w := v_a8; end if;
            if v_w < -v_a8 then v_w := -v_a8; end if;
            p2_dw <= v_w(17 downto 8);
            -- 3: luma of this pixel; skew sign
            p3_fh <= p2_fh;
            if c_sneg = '1' then p3_dw <= -p2_dw; else p3_dw <= p2_dw; end if;
            p3_lm <= unsigned(s_lb_q);
            -- 4: etch product
            p4_pe <= p3_lm * c_e;
            p4_fh <= p3_fh; p4_dw <= p3_dw;
            -- 5: displaced position and the bent field
            v_s := signed(resize(l_pos, 14)) + resize(p4_dw, 14) + signed(resize(p4_pe(15 downto 7), 14));
            p5_pos <= v_s;
            p5_fb <= resize(p4_fh, 10) + resize(p4_pe(15 downto 9), 10);
            -- 6: mod P straight into the table's address register
            if p5_pos < 0 then
                v_pm := resize(p5_pos + C_P, 12);
            elsif p5_pos >= C_P then
                v_pm := resize(p5_pos - C_P, 12);
            else
                v_pm := p5_pos(11 downto 0);
            end if;
            t_ra <= unsigned(v_pm(11 downto 2));
            p6_fb <= p5_fb;
            -- 7: table data register; 8: fabric copy
            p7_fb <= p6_fb;
            p8_z <= signed(t_rq(15 downto 8)); p8_dr <= unsigned(t_rq(7 downto 0)); p8_fb <= p7_fb;
            -- 9: field index straight into the palette address registers
            v_f := signed(resize(p8_fb, 12)) + resize(l_v, 12) - signed(resize(p8_dr, 12));
            if v_f < 0 then v_fi := (others => '0');
            elsif v_f > 511 then v_fi := to_unsigned(511, 9);
            else v_fi := unsigned(v_f(8 downto 0)); end if;
            r_pc_a <= s_pal & v_fi; r_pw_a <= v_fi;
            p9_z <= p8_z;
            -- 10: ROM data registers; 11: fabric copies and the gates
            p10_z <= p9_z;
            p12_rc <= r_pc_q; p12_rw <= r_pw_q;
            if p10_z < 0 then v_nz := unsigned(-resize(p10_z, 9)); else v_nz := (others => '0'); end if;
            if p10_z > 0 then v_pz := unsigned(resize(p10_z, 9)); else v_pz := (others => '0'); end if;
            v_g := shift_left(resize(v_nz, 12), 2) + shift_left(resize(v_nz, 12), 1) + resize(v_nz(8 downto 1), 12);           -- 6.5 (-Z)
            p12_gcr <= f_sat8(v_g);
            v_g := shift_left(resize(v_nz, 12), 2) + resize(v_nz(8 downto 1), 12) + resize(v_nz(8 downto 3), 12);              -- 4.625 (-Z)
            p12_gcg <= f_sat8(v_g);
            v_g := shift_left(resize(v_nz, 12), 1) + resize(v_nz, 12) + resize(v_nz(8 downto 2), 12) + resize(v_nz(8 downto 3), 12);   -- 3.375 (-Z)
            p12_gcb <= f_sat8(v_g);
            if p10_z > -100 then v_t := unsigned(resize(p10_z, 9) + 100); else v_t := (others => '0'); end if;
            v_g := shift_left(resize(v_t, 12), 1) - resize(v_t(8 downto 2), 12) - resize(v_t(8 downto 5), 12);                 -- 1.72 (Z + 100)
            p12_gwr <= f_sat8(v_g);
            v_g := shift_left(resize(v_pz, 12), 2) + resize(v_pz, 12) + resize(v_pz(8 downto 3), 12);                           -- 5.125 Z
            p12_gwg <= f_sat8(v_g);
            if p10_z > 2 then v_t := unsigned(resize(p10_z, 9) - 2); else v_t := (others => '0'); end if;
            v_g := shift_left(resize(v_t, 12), 2) + resize(v_t, 12) + resize(v_t(8 downto 1), 12) + resize(v_t(8 downto 4), 12);   -- 5.56 (Z - 2)
            p12_gwb <= f_sat8(v_g);
            -- 12: the six products
            p13_rc <= unsigned(p12_rc(23 downto 16)) * p12_gcr;
            p13_rw <= unsigned(p12_rw(23 downto 16)) * p12_gwr;
            p13_gc <= unsigned(p12_rc(15 downto 8)) * p12_gcg;
            p13_gw <= unsigned(p12_rw(15 downto 8)) * p12_gwg;
            p13_bc <= unsigned(p12_rc(7 downto 0)) * p12_gcb;
            p13_bw <= unsigned(p12_rw(7 downto 0)) * p12_gwb;
            -- 13: mix + glow, clipped
            v_m := resize(p13_rc, 17) + resize(p13_rw, 17);
            p14_r <= f_sat8(resize(v_m(16 downto 8), 10) + resize(l_gr, 10));
            v_m := resize(p13_gc, 17) + resize(p13_gw, 17);
            p14_g <= f_sat8(resize(v_m(16 downto 8), 10) + resize(l_gg, 10));
            v_m := resize(p13_bc, 17) + resize(p13_bw, 17);
            p14_b <= f_sat8(resize(v_m(16 downto 8), 10) + resize(l_gb, 10));
            -- 14: luma partials 77 R, 150 G, 29 B
            p15_yr <= shift_left(resize(p14_r, 16), 6) + shift_left(resize(p14_r, 16), 4) - shift_left(resize(p14_r, 16), 1) - resize(p14_r, 16);
            p15_yg <= shift_left(resize(p14_g, 16), 7) + shift_left(resize(p14_g, 16), 4) + shift_left(resize(p14_g, 16), 2) + shift_left(resize(p14_g, 16), 1);
            p15_yb <= shift_left(resize(p14_b, 16), 5) - shift_left(resize(p14_b, 16), 1) - resize(p14_b, 16);
            p15_r <= p14_r; p15_b <= p14_b;
            -- 15: Y8
            v_y := resize(p15_yr, 19) + resize(p15_yg, 19) + resize(p15_yb, 19);
            p16_y8 <= v_y(15 downto 8);
            p16_r <= p15_r; p16_b <= p15_b;
            -- 16: 641 (R - Y8), 508 (B - Y8), luma 64 + Y8 * 1086 / 256 (pre-scaled 16/13)
            v_d9 := signed(resize(p16_r, 9)) - signed(resize(p16_y8, 9));
            p17_cr <= shift_left(resize(v_d9, 19), 9) + shift_left(resize(v_d9, 19), 7) + resize(v_d9, 19);
            v_d9 := signed(resize(p16_b, 9)) - signed(resize(p16_y8, 9));
            p17_cb <= shift_left(resize(v_d9, 19), 9) - shift_left(resize(v_d9, 19), 2);
            v_y := shift_left(resize(p16_y8, 19), 10) + shift_left(resize(p16_y8, 19), 6) - shift_left(resize(p16_y8, 19), 1);
            p17_y10 <= to_unsigned(64, 11) + resize(v_y(18 downto 8), 11);
            -- 17: compose (luma 10 bits capped at 940, chroma 8 bits); black outside the generated avid
            if s_gav_sr(15) = '0' then
                s_y18 <= to_unsigned(64, 10); s_u18 <= to_unsigned(128, 8); s_v18 <= to_unsigned(128, 8);
            else
                if p17_y10 > 940 then s_y18 <= to_unsigned(940, 10); else s_y18 <= p17_y10(9 downto 0); end if;
                v_c := shift_right(p17_cr, 8) + 512;
                if v_c < 24 then s_u18 <= to_unsigned(6, 8); elsif v_c > 1000 then s_u18 <= to_unsigned(250, 8); else s_u18 <= unsigned(v_c(9 downto 2)); end if;
                v_c := shift_right(p17_cb, 8) + 512;
                if v_c < 24 then s_v18 <= to_unsigned(6, 8); elsif v_c > 1000 then s_v18 <= to_unsigned(250, 8); else s_v18 <= unsigned(v_c(9 downto 2)); end if;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(15);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: chroma through one [1 2 2 2 2 2 2 2 1]/16 box (two
    -- adjacent 8-sums; the 8-back tap from a block-RAM ring read 6 behind the
    -- write plus one fabric register), luma [1 2 1] and 13/16, then the
    -- blanking gate.  The running sums snap back to neutral once 8 blank
    -- samples are in, so a glitched sample can't offset the colours for good.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_l  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- ring pointers (free running); the tap = the sample 8 back
            ra_wa <= ra_wa + 1;
            ra_ra <= ra_wa - 5;
            ra_t  <= ra_q;
            -- 19: sums
            if s_bah = "00000000" then
                s_bau <= to_unsigned(1024, 11); s_bav <= to_unsigned(1024, 11);
            else
                s_bau <= s_bau + s_u18 - unsigned(ra_t(15 downto 8));
                s_bav <= s_bav + s_v18 - unsigned(ra_t(7 downto 0));
            end if;
            s_baud <= s_bau; s_bavd <= s_bav;
            s_y19 <= s_y18;
            s_y20 <= s_y19;
            -- 20: box out (8 bits), luma [1 2 1]
            v_sc := resize(s_bau, 12) + s_baud; s_u20 <= v_sc(11 downto 4);
            v_sc := resize(s_bav, 12) + s_bavd; s_v20 <= v_sc(11 downto 4);
            v_l := resize(s_y18, 12) + shift_left(resize(s_y19, 12), 1) + resize(s_y20, 12);
            s_y21 <= v_l(11 downto 2);
            -- 21: luma 13/16, chroma expanded
            if s_y21 > 64 then v_dl := s_y21 - 64; else v_dl := (others => '0'); end if;
            s_y22 <= s_y21 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u22 <= s_u20; s_v22 <= s_v20;
            s_u23 <= s_u22 & "00"; s_v23 <= s_v22 & "00"; s_y23 <= s_y22;
            -- 22 .. 24: padding to the latency
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
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture aureole;
