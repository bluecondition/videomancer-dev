-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Bunsen v0.1.0 -- Heirloom from the video clips, built from the still WIN_20231205_23_09_56 t0.5
-- (clip frame 15 at 0.5 s of a 6 s clip; Ron raises the key's fade through the clip).
--
-- The still: an orange "bat / crown" key (two peaks with a notch, two wings, an X crossing, two side
-- triangles, two bottom lobes round an arch) over a magenta contour field with dark contour lines, blue pools
-- in three corners; the key fades out toward the bottom (a V-shaped ramp in y), so the lobes read hot pink.
--
-- The patch, as rebuilt (bunsen.py has the tables; .heirloom_work/rb_bunsen/notes.md the measurements), in
-- virtual 1080-line coordinates:
--   F   = H(xc + (x - xc + D(y) w + oh) sl) + V(sgn(Y) (|Y| + E(x) w) sl + ov)   Y = y - 520, xc = 1006
--         two oscillator waveforms (H across, V down, folded at y 520), each phase-modulated by the other
--         axis (D, E: the warp), fitted to the still's contour orientations; oh / ov: a 3.9 s LFO sway
--       + camera luma x K6 (or the Osc lattice)
--   u   = frac(F x rings)  ->  background palette (magenta / violet / dark lines)  and key palette (orange / red)
--   bg  = mix(palette, blue, beta)      beta: the three blue pools (per-row edges, 64-px soft ramps)
--   key = the still's shape: a per-row table (4-line rows) of runs |x - 1006| in [a0, a1), [b0, b1),
--         [c0, edge), grown by K5;  S8 inverts it;  S9 adds a field-level key (flame blobs)
--   out = mix(bg, key colour, alpha)    alpha: a V in y about the notch (P12), with the field's contours in it;
--         below the notch the key's red falls (olive / dark red), the trailing edges get a dark red band
--
--   K1  Rings     contour density (x0.4 .. x3)
--   K2  Lattice   the oscillators' spatial scale: big cells .. a fine lattice (x0.5 .. x2)
--   K3  Warp      the cross-modulation depth: a straight lattice .. swirling drapes (x0 .. x1 .. x4)
--   K4  Speed     the LFO sway: still .. 0.5 s cycles (square law); 377 = the clip's 3.9 s
--   K5  Bat       the key shape's weight: thin slivers .. swollen (+-256 virtual px)
--   K6  Camera    how far the picture (or the Osc) bends the contours and pushes the bat's edges
--   S7  Gas       Ember (magenta field, orange bat) / Blue (blue-black field, yellow bat)
--   S8  Key       Bat / Cutout (inverted)
--   S9  Flames    Off / On: flame blobs keyed from the field's level
--   S10 Flip      the bat upside down
--   S11 Source    Video / Osc
--   P12 FADE      the key fade's notch: 0 = embers (the bat clear at the top, dark red / olive below);
--                 643 = the still; 100 % = the whole bat lit solid orange
--
-- Engine: vblank-zone sequencer (shared glide, one serial multiplier), a per-line engine kicked at each line start
-- computing the NEXT line (row table read, D interpolated, grow, D x warp, the H start, |Y| x lattice, the fade
-- alpha, the red fall-off), a 19-stage pixel path, the shell's output conditioning (Y, U, V through two
-- [1 2 2 2 2 2 2 2 1]/16 boxes with block-RAM taps and the resync, luma 13/16) and the blanking gate.
-- Generated avid; one latency (24) in every mode; colours in the hardware convention (u = Cr).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.bunsen_pkg.all;
use work.all;

architecture bunsen of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks
    constant C_FL  : integer := C_BU_FLAME;   -- flame level (Q8 bands), from bunsen.py

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
    -- sin(2 pi p / 64) x 127
    function f_sin6(p : unsigned(5 downto 0)) return signed is
        type t_q is array (0 to 16) of integer;
        constant Q : t_q := (0, 12, 25, 37, 49, 60, 71, 81, 90, 98, 106, 112, 117, 122, 125, 126, 127);
        variable i : integer;
        variable s : integer;
    begin
        i := to_integer(p(3 downto 0));
        if p(4) = '1' then i := 16 - i; end if;
        s := Q(i);
        if p(5) = '1' then s := -s; end if;
        return to_signed(s, 8);
    end function;

    -- line buffer: (luma >> 2) - 16, clipped at 0 (or the Osc lattice)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_pb_a, r_pf_a : unsigned(7 downto 0) := (others => '0');
    signal r_pb_q, r_pf_q : std_logic_vector(31 downto 0) := (others => '0');
    signal r_h_a, r_v_a, r_e_a : unsigned(8 downto 0) := (others => '0');
    signal r_h_q, r_v_q : std_logic_vector(15 downto 0) := (others => '0');
    signal r_e_q  : std_logic_vector(7 downto 0) := (others => '0');
    signal r_rt_a : unsigned(7 downto 0) := (others => '0');
    signal r_rt_q : std_logic_vector(79 downto 0) := (others => '0');

    attribute no_rw_check : boolean;

    -- output box rings (the 8-back taps of the two running sums) and the sync delay ring
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal ba1, ba2, bb1, bb2 : t_r16 := (others => x"0000");
    attribute no_rw_check of ba1 : signal is true;
    attribute no_rw_check of ba2 : signal is true;
    attribute no_rw_check of bb1 : signal is true;
    attribute no_rw_check of bb2 : signal is true;
    signal rg_wa, rg_ra, sy_ra : unsigned(7 downto 0) := (others => '0');
    signal ba1_q, ba2_q, bb1_q, bb2_q : std_logic_vector(15 downto 0) := x"0000";
    type t_r4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal syr : t_r4 := (others => "1111");
    attribute no_rw_check of syr : signal is true;
    signal syr_q, syr_c : std_logic_vector(3 downto 0) := "1111";

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
    signal s_otap : unsigned(2 downto 0) := to_unsigned(5, 3);

    -- knobs: glided (Q3), switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(377 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(320 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(643 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_pal, s_inv, s_flm, s_flip, s_src : std_logic := '0';
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- per-field values
    signal f_a    : unsigned(23 downto 0) := (others => '0');
    signal f_b    : unsigned(11 downto 0) := (others => '0');
    signal f_acc  : unsigned(23 downto 0) := (others => '0');
    signal z_base : unsigned(9 downto 0) := (others => '0');
    signal c_kr   : unsigned(9 downto 0) := to_unsigned(256, 10);      -- rings, Q8 (256 = x1)
    signal c_sl   : unsigned(9 downto 0) := to_unsigned(256, 10);      -- lattice scale, Q8
    signal c_w    : unsigned(9 downto 0) := to_unsigned(256, 10);      -- warp, Q8 (x0 .. x1 at 512 .. x4)
    signal c_ws   : unsigned(10 downto 0) := to_unsigned(256, 11);     -- warp x lattice, Q8
    signal c_hst  : unsigned(10 downto 0) := to_unsigned(256, 11);     -- H index step per pixel, Q8 virtual px
    signal c_spd  : unsigned(11 downto 0) := to_unsigned(277, 12);     -- LFO phase step per field
    signal c_ph   : unsigned(15 downto 0) := to_unsigned(0, 16);       -- LFO phase
    signal c_oh   : signed(7 downto 0) := (others => '0');             -- H sway, virtual px
    signal c_ov   : signed(8 downto 0) := (others => '0');             -- V sway, V-table entries (4 lines)
    signal c_g    : signed(9 downto 0) := (others => '0');             -- key grow, virtual px
    signal c_nt   : signed(11 downto 0) := to_signed(975, 12);         -- fade notch, virtual line
    signal c_cg   : unsigned(5 downto 0) := to_unsigned(20, 6);        -- camera gain, Q4 per luma step

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
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_ym, e_yk : unsigned(10 downto 0) := (others => '0');
    signal e_ay   : unsigned(10 downto 0) := (others => '0');
    signal e_sgy  : std_logic := '0';
    signal e_ra, e_rb : unsigned(7 downto 0) := (others => '0');
    signal e_fr   : unsigned(1 downto 0) := (others => '0');
    signal e_r0   : std_logic_vector(79 downto 0) := (others => '0');
    signal e_d1   : signed(7 downto 0) := (others => '0');
    signal e_dd, e_dp, e_dv : signed(9 downto 0) := (others => '0');
    signal e_dw   : signed(12 downto 0) := (others => '0');
    signal e_hs   : signed(12 downto 0) := (others => '0');
    signal e_e    : unsigned(11 downto 0) := (others => '0');
    signal e_ab   : unsigned(11 downto 0) := (others => '0');
    signal e_jc   : unsigned(2 downto 0) := (others => '0');
    signal e_dn   : std_logic := '0';
    signal m_a    : unsigned(23 downto 0) := (others => '0');
    signal m_b    : unsigned(11 downto 0) := (others => '0');
    signal m_acc  : unsigned(23 downto 0) := (others => '0');
    signal n_a0, n_a1, n_b0, n_b1, n_c0 : unsigned(11 downto 0) := (others => '0');
    signal n_dv   : signed(11 downto 0) := (others => '0');
    signal n_tb, n_bl, n_br : unsigned(7 downto 0) := (others => '0');
    signal n_h0   : unsigned(19 downto 0) := (others => '0');
    signal n_vb   : unsigned(19 downto 0) := (others => '0');
    signal n_sgy  : std_logic := '0';
    signal n_al   : unsigned(9 downto 0) := (others => '0');
    signal n_dy, n_dcr, n_dcb : signed(10 downto 0) := (others => '0');
    signal n_jit  : unsigned(3 downto 0) := (others => '0');
    signal n_ws   : unsigned(10 downto 0) := (others => '0');
    signal e_tp   : unsigned(6 downto 0) := (others => '0');

    -- beam
    signal b_x   : unsigned(11 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal b_hp  : unsigned(19 downto 0) := (others => '0');
    signal c_xb0 : unsigned(11 downto 0) := to_unsigned(16, 12);
    signal l_a0, l_a1, l_b0, l_b1, l_c0 : unsigned(11 downto 0) := (others => '0');
    signal l_tb, l_bl, l_br : unsigned(7 downto 0) := (others => '0');
    signal l_vb  : unsigned(19 downto 0) := (others => '0');
    signal l_sgy : std_logic := '0';
    signal l_al  : unsigned(9 downto 0) := (others => '0');
    signal l_dy, l_dcr, l_dcb : signed(10 downto 0) := (others => '0');
    signal l_jit : unsigned(3 downto 0) := (others => '0');
    signal l_ws  : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    signal p2_vx, p3_vx : unsigned(10 downto 0) := (others => '0');
    signal p2_kx  : unsigned(11 downto 0) := (others => '0');
    signal p3_x   : unsigned(11 downto 0) := (others => '0');
    signal p3_dt  : unsigned(10 downto 0) := (others => '0');
    signal p4_e   : signed(7 downto 0) := (others => '0');
    signal p4_h   : signed(11 downto 0) := (others => '0');
    signal p4_x, p5_x, p6_x : unsigned(11 downto 0) := (others => '0');
    signal p7_xk  : signed(13 downto 0) := (others => '0');
    signal p4_dt, p4_dl, p4_dr : signed(12 downto 0) := (others => '0');
    signal p4_lm  : unsigned(7 downto 0) := (others => '0');
    signal p5_ew  : signed(19 downto 0) := (others => '0');
    signal p5_cam : signed(15 downto 0) := (others => '0');
    signal p5_bt, p5_bl, p5_br : unsigned(3 downto 0) := (others => '0');
    signal p6_vi  : signed(20 downto 0) := (others => '0');
    signal p6_cam : signed(11 downto 0) := (others => '0');
    signal p6_be  : unsigned(3 downto 0) := (others => '0');
    signal p7_cam, p8_cam, p9_cam : signed(11 downto 0) := (others => '0');
    type t_h is array (5 to 9) of signed(11 downto 0);
    signal q_h    : t_h := (others => (others => '0'));
    signal p9_v   : signed(11 downto 0) := (others => '0');
    signal p10_f  : signed(13 downto 0) := (others => '0');
    signal p11_g1, p11_g0 : signed(19 downto 0) := (others => '0');
    signal p11_fl : std_logic := '0';
    signal p12_u  : unsigned(6 downto 0) := (others => '0');
    signal p12_t, p13_t : unsigned(5 downto 0) := (others => '0');
    signal p14_al : unsigned(3 downto 0) := (others => '0');
    signal p15_al, p16_al, p17_al : unsigned(3 downto 0) := (others => '0');
    signal p14_by, p14_bu, p14_bv, p14_fy, p14_fu, p14_fv : signed(11 downto 0) := (others => '0');
    signal p15_by, p15_bu, p15_bv : signed(11 downto 0) := (others => '0');
    signal p15_dy, p15_du, p15_dv : signed(11 downto 0) := (others => '0');
    signal p15_fy, p15_fu, p15_fv, p16_fy, p16_fu, p16_fv : signed(11 downto 0) := (others => '0');
    signal p16_y, p16_u, p16_v : signed(11 downto 0) := (others => '0');
    signal p17_y, p17_u, p17_v : signed(11 downto 0) := (others => '0');
    signal p17_dy, p17_du, p17_dv : signed(11 downto 0) := (others => '0');
    signal p18_y, p18_u, p18_v : signed(11 downto 0) := (others => '0');
    signal p18_ky, p18_ku, p18_kv : signed(11 downto 0) := (others => '0');
    -- beta (blue) and key bits ride per stage
    type t_b4 is array (7 to 15) of unsigned(3 downto 0);
    signal q_be   : t_b4 := (others => (others => '0'));
    type t_bit is array (8 to 18) of std_logic;
    signal q_sh, q_fl : t_bit := (others => '0');
    signal p17_k, p18_ol : std_logic := '0';
    signal q_kd   : std_logic_vector(5 downto 0) := (others => '0');
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    signal s_bay, s_bau, s_bav, s_bayd, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bby, s_bbu, s_bbv, s_bbyd, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_cay, s_cau, s_cav, s_cby, s_cbu, s_cbv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y21, s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y25 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u25, s_v25 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories: line buffer (collisions possible: read-before-write kept),
    -- ROMs, the box rings, the sync ring
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
            r_pb_q <= C_BU_PB(to_integer(r_pb_a));
            r_pf_q <= C_BU_PF(to_integer(r_pf_a));
            r_h_q  <= C_BU_H(to_integer(r_h_a));
            r_v_q  <= C_BU_V(to_integer(r_v_a));
            r_e_q  <= C_BU_E(to_integer(r_e_a));
            r_rt_q <= C_BU_RT(to_integer(r_rt_a));
        end if;
    end process p_rom;

    p_rings : process(clk)
    begin
        if rising_edge(clk) then
            ba1_q <= ba1(to_integer(rg_ra));
            ba1(to_integer(rg_wa)) <= std_logic_vector(s_y19) & std_logic_vector(s_u19(9 downto 4));
            ba2_q <= ba2(to_integer(rg_ra));
            ba2(to_integer(rg_wa)) <= std_logic_vector(s_u19(3 downto 0)) & std_logic_vector(s_v19) & "00";
            bb1_q <= bb1(to_integer(rg_ra));
            bb1(to_integer(rg_wa)) <= std_logic_vector(s_y21) & std_logic_vector(s_u21(9 downto 4));
            bb2_q <= bb2(to_integer(rg_ra));
            bb2(to_integer(rg_wa)) <= std_logic_vector(s_u21(3 downto 0)) & std_logic_vector(s_v21) & "00";
            syr_q <= syr(to_integer(sy_ra));
            syr(to_integer(rg_wa)) <= '0' & data_in.field_n & data_in.vsync_n & data_in.hsync_n;
        end if;
    end process p_rings;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_d  : signed(11 downto 0);
        variable v_p  : unsigned(11 downto 0);
        variable v_s  : signed(7 downto 0);
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

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_pal  <= registers_in(6)(0);
                s_inv  <= registers_in(6)(1);
                s_flm  <= registers_in(6)(2);
                s_flip <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_oxs <= to_unsigned(204, 9); s_otap <= to_unsigned(2, 3);
                    when 3      => s_x0 <= (others => '0');     s_oxs <= to_unsigned(153, 9); s_otap <= to_unsigned(3, 3);
                    when others => s_x0 <= (others => '0');     s_oxs <= to_unsigned(102, 9); s_otap <= to_unsigned(5, 3);
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);
            c_xb0 <= resize(s_x0, 12) + shift_left(resize(s_xs, 12), 3);

            -- shared glide (1/8 of the gap per field, +-2 deadband): four clocks per channel, seven channels
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        case to_integer(z_gc) is
                            when 0      => s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                            when 1      => s_gm <= s_g2; s_gr <= unsigned(registers_in(1));
                            when 2      => s_gm <= s_g3; s_gr <= unsigned(registers_in(2));
                            when 3      => s_gm <= s_g4; s_gr <= unsigned(registers_in(3));
                            when 4      => s_gm <= s_g5; s_gr <= unsigned(registers_in(4));
                            when 5      => s_gm <= s_g6; s_gr <= unsigned(registers_in(5));
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
                            when 3      => s_g4 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 4      => s_g5 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when 5      => s_g6 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                            when others => s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        end case;
                        z_gp <= "00";
                        if z_gc = 6 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial multiplier (16 iterations; the product is complete 17 clocks after the load)
            if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
            f_a <= f_a(22 downto 0) & '0';
            f_b <= '0' & f_b(11 downto 1);

            -- per-field values: zc 30 .. 110
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 30 =>                                        -- rings: x0.4 .. x1 .. x3 (two slopes)
                        if s_g1(12 downto 3) < 512 then
                            f_a <= to_unsigned(154, 24); f_b <= resize(s_g1(12 downto 3), 12); z_base <= to_unsigned(102, 10);
                        else
                            f_a <= to_unsigned(511, 24); f_b <= resize(s_g1(12 downto 3) - 512, 12); z_base <= to_unsigned(256, 10);
                        end if;
                        f_acc <= (others => '0');
                    when 48 =>
                        c_kr <= z_base + resize(f_acc(18 downto 9), 10);
                        if s_g2(12 downto 3) < 512 then              -- lattice: x0.5 .. x1 .. x2
                            f_a <= to_unsigned(128, 24); f_b <= resize(s_g2(12 downto 3), 12); z_base <= to_unsigned(128, 10);
                        else
                            f_a <= to_unsigned(256, 24); f_b <= resize(s_g2(12 downto 3) - 512, 12); z_base <= to_unsigned(256, 10);
                        end if;
                        f_acc <= (others => '0');
                    when 66 =>
                        c_sl <= z_base + resize(f_acc(18 downto 9), 10);
                        if s_g3(12 downto 3) < 512 then               -- warp: x0 .. x1 (512) .. x4
                            c_w <= resize(s_g3(12 downto 4), 10);
                        else
                            c_w <= to_unsigned(256, 10) + resize(s_g3(12 downto 3) - 512, 10) + resize(shift_right(s_g3(12 downto 3) - 512, 1), 10);
                        end if;
                    when 67 =>
                        f_a <= resize(c_w, 24); f_b <= resize(c_sl, 12); f_acc <= (others => '0');
                    when 85 =>
                        c_ws <= f_acc(18 downto 8);
                        case to_integer(s_xs) is                      -- H step per output pixel, Q8 virtual px x lattice
                            when 4      => c_hst <= shift_left(resize(c_sl, 11), 1);
                            when 3      => c_hst <= resize(c_sl, 11) + resize(shift_right(c_sl, 1), 11);
                            when others => c_hst <= resize(c_sl, 11);
                        end case;
                        f_a <= resize(s_g4(12 downto 3), 24); f_b <= resize(s_g4(12 downto 3), 12); f_acc <= (others => '0');
                    when 103 =>                                       -- speed: phase step k4^2 / 512
                        c_spd <= f_acc(20 downto 9);
                    when 104 =>
                        c_ph <= c_ph + resize(c_spd, 16);
                    when 105 =>                                       -- the sway: oh = sin / 2 px, ov = cos / 8 entries
                        v_s := f_sin6(c_ph(15 downto 10));
                        c_oh <= shift_right(v_s, 1);
                        v_s := f_sin6(c_ph(15 downto 10) + 16);
                        c_ov <= resize(shift_right(v_s, 3), 9);
                        -- bat grow: (k5 - 512) / 2
                        v_d := signed(resize(s_g5(12 downto 3), 12)) - 512;
                        c_g <= v_d(10 downto 1);
                        -- fade notch: -150 + 1.75 p12
                        v_p := resize(s_gp(12 downto 3), 12);
                        c_nt <= signed(v_p + shift_right(v_p, 1) + shift_right(v_p, 2)) - 150;
                        c_cg <= s_g6(12 downto 7);
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
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line: the key / blue row (4-line rows; the warp
    -- D interpolated), the grow, D x warp and the H start, |Y| x lattice, the
    -- fade alpha and the red fall-off below the notch.  One serial product at a
    -- time (16 iterations, complete at load + 17).
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_y  : signed(11 downto 0);
        variable v_s  : signed(12 downto 0);
        variable v_t  : signed(13 downto 0);
        variable v_u  : unsigned(12 downto 0);
        variable v_r  : unsigned(7 downto 0);
        variable v_d  : signed(11 downto 0);
        variable v_e0, v_e1 : boolean;
        variable v_a0, v_a1, v_b0, v_b1, v_c0 : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 1300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
                e_jc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
                if e_jc = 4 then e_jc <= (others => '0'); else e_jc <= e_jc + 1; end if;
            elsif e_lc /= 127 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(22 downto 0) & '0';
            m_b <= '0' & m_b(11 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    if e_vy(18 downto 8) > 1079 then e_ym <= to_unsigned(1079, 11); else e_ym <= e_vy(18 downto 8); end if;
                when 2 =>
                    if s_flip = '1' then e_yk <= 1079 - e_ym; else e_yk <= e_ym; end if;
                    v_y := signed('0' & e_ym) - 520;
                    v_d := -v_y;
                    if v_y < 0 then e_ay <= unsigned(v_d(10 downto 0)); e_sgy <= '1';
                    else e_ay <= unsigned(v_y(10 downto 0)); e_sgy <= '0'; end if;
                    n_jit <= resize(e_jc & '0', 4);
                when 3 =>                                             -- the warp tapers to 0 at the mirror: min(|Y|, 64) / 64
                    if e_ay >= 64 then e_tp <= to_unsigned(64, 7); else e_tp <= e_ay(6 downto 0); end if;
                when 4 =>
                    if e_yk >= 1024 then
                        e_ra <= to_unsigned(255, 8); e_rb <= to_unsigned(255, 8); e_fr <= "00";
                    else
                        e_ra <= e_yk(9 downto 2); e_fr <= e_yk(1 downto 0);
                        if e_yk(9 downto 2) = 255 then e_rb <= to_unsigned(255, 8); else e_rb <= e_yk(9 downto 2) + 1; end if;
                    end if;
                when 5 =>
                    r_rt_a <= e_rb;
                when 6 =>
                    r_rt_a <= e_ra;
                when 7 =>
                    e_d1 <= signed(r_rt_q(31 downto 24));
                when 8 =>
                    e_r0 <= r_rt_q;
                when 9 =>                                             -- the warp D interpolated between the rows: d = D1 - D0
                    e_dd <= resize(e_d1, 10) - resize(signed(e_r0(31 downto 24)), 10);
                    n_tb <= unsigned(e_r0(23 downto 16));
                    n_bl <= unsigned(e_r0(15 downto 8));
                    n_br <= unsigned(e_r0(7 downto 0));
                when 10 =>                                            -- p = d x fr (0 .. 3)
                    case e_fr is
                        when "00"   => e_dp <= (others => '0');
                        when "01"   => e_dp <= e_dd;
                        when "10"   => e_dp <= shift_left(e_dd, 1);
                        when others => e_dp <= shift_left(e_dd, 1) + e_dd;
                    end case;
                when 11 =>                                            -- D = D0 + p / 4
                    e_dv <= resize(signed(e_r0(31 downto 24)), 10) + shift_right(e_dp, 2);
                when 12 =>                                            -- grow (empty runs stay empty), D x warp
                    v_a0 := signed(resize(unsigned(e_r0(79 downto 72)), 12));
                    v_a1 := signed(resize(unsigned(e_r0(71 downto 62)), 12));
                    v_b0 := signed(resize(unsigned(e_r0(61 downto 52)), 12));
                    v_b1 := signed(resize(unsigned(e_r0(51 downto 42)), 12));
                    v_c0 := signed(resize(unsigned(e_r0(41 downto 32)), 12));
                    v_e0 := (v_a1 - v_a0) <= 1;
                    if v_e0 then
                        n_a0 <= to_unsigned(4095, 12); n_a1 <= (others => '0');
                    else
                        v_s := resize(v_a0, 13) - resize(c_g, 13);
                        if v_a0 = 0 or v_s < 0 then n_a0 <= (others => '0'); else n_a0 <= unsigned(v_s(11 downto 0)); end if;
                        v_s := resize(v_a1, 13) + resize(c_g, 13);
                        if v_s < 0 then n_a1 <= (others => '0'); else n_a1 <= unsigned(v_s(11 downto 0)); end if;
                    end if;
                    v_e1 := (v_b1 - v_b0) <= 1;
                    if v_e1 then
                        n_b0 <= to_unsigned(4095, 12); n_b1 <= (others => '0');
                    else
                        v_s := resize(v_b0, 13) - resize(c_g, 13);
                        if v_s < 0 then n_b0 <= (others => '0'); else n_b0 <= unsigned(v_s(11 downto 0)); end if;
                        v_s := resize(v_b1, 13) + resize(c_g, 13);
                        if v_s < 0 then n_b1 <= (others => '0'); else n_b1 <= unsigned(v_s(11 downto 0)); end if;
                    end if;
                    if v_c0 >= 1000 then
                        n_c0 <= to_unsigned(4095, 12);
                    else
                        v_s := resize(v_c0, 13) - resize(c_g, 13);
                        if v_s < 0 then n_c0 <= (others => '0'); else n_c0 <= unsigned(v_s(11 downto 0)); end if;
                    end if;
                    n_dv <= resize(e_dv, 12);
                    if e_dv < 0 then m_a <= resize(unsigned(-e_dv), 24); else m_a <= resize(unsigned(e_dv), 24); end if;
                    m_b <= resize(c_w, 12); m_acc <= (others => '0');
                when 29 =>                                            -- hs = x0 - 1006 + D w + oh
                    if n_dv < 0 then v_s := -signed(resize(m_acc(19 downto 8), 13)); else v_s := signed(resize(m_acc(19 downto 8), 13)); end if;
                    e_dw <= v_s;
                when 30 =>
                    e_hs <= signed(resize(shift_right(c_xb0, 1), 13)) - 1006 + e_dw + resize(c_oh, 13);
                when 31 =>
                    if e_hs < 0 then m_a <= resize(unsigned(-e_hs), 24); else m_a <= resize(unsigned(e_hs), 24); end if;
                    m_b <= resize(c_sl, 12); m_acc <= (others => '0');
                when 48 =>                                            -- H start = 1006 + hs sl, Q8 px (mod 2^20)
                    if e_hs < 0 then
                        n_h0 <= to_unsigned(1006 * 256, 20) - m_acc(19 downto 0);
                    else
                        n_h0 <= to_unsigned(1006 * 256, 20) + m_acc(19 downto 0);
                    end if;
                    m_a <= resize(e_ay, 24); m_b <= resize(c_sl, 12); m_acc <= (others => '0');
                when 65 =>                                            -- |Y| sl, Q8 lines; fade distance
                    n_vb <= m_acc(19 downto 0);
                    n_sgy <= e_sgy;
                    v_y := signed('0' & e_ym) - c_nt;
                    if v_y < 0 then e_e <= unsigned(-v_y); e_ab <= (others => '0');
                    else e_e <= unsigned(v_y); e_ab <= unsigned(v_y); end if;
                when 66 =>
                    m_a <= resize(e_e, 24); m_b <= to_unsigned(139, 12); m_acc <= (others => '0');
                when 83 =>                                            -- alpha8 = 26 + e / 470 + max(0, e - 200) x 3.25
                    v_u := to_unsigned(26, 13) + resize(m_acc(19 downto 8), 13);
                    if e_e > 200 then
                        v_u := v_u + shift_left(resize(e_e - 200, 13), 1) + resize(e_e - 200, 13) + resize(shift_right(e_e - 200, 2), 13);
                    end if;
                    if v_u > 1023 then n_al <= (others => '1'); else n_al <= v_u(9 downto 0); end if;
                    m_a <= resize(e_ab, 24); m_b <= to_unsigned(72, 12); m_acc <= (others => '0');
                when 100 =>                                            -- red fall-off below the notch: rr = 0.28 per line
                    if m_acc(19 downto 8) > 255 then v_r := (others => '1'); else v_r := m_acc(15 downto 8); end if;
                    -- key colour offsets for R = 255 - rr: dY = -1.2645 rr (luma pre-scaled), dCr = -1.757 rr, dCb = +0.593 rr
                    n_dy  <= -signed(resize(v_r, 11) + resize(shift_right(v_r, 2), 11) + resize(shift_right(v_r, 6), 11));
                    n_dcr <= -signed(shift_left(resize(v_r, 11), 1) - resize(shift_right(v_r, 2), 11) + resize(shift_right(v_r, 7), 11));
                    n_dcb <= signed(resize(shift_right(v_r, 1), 11) + resize(shift_right(v_r, 4), 11) + resize(shift_right(v_r, 5), 11));
                    m_a <= resize(c_ws, 24); m_b <= resize(e_tp, 12); m_acc <= (others => '0');
                when 117 =>                                           -- the line's warp gain: ws x min(|Y|, 64) / 64
                    n_ws <= m_acc(16 downto 6);
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1): x in h (half virtual px), the H coordinate (Q8 px), the
    -- line's values latched at the line start; the camera read issued from
    -- the pixel index
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x   <= c_xb0;
                b_px  <= (others => '0');
                b_hp  <= n_h0;
                l_a0  <= n_a0; l_a1 <= n_a1; l_b0 <= n_b0; l_b1 <= n_b1; l_c0 <= n_c0;
                l_tb  <= n_tb; l_bl <= n_bl; l_br <= n_br;
                l_vb  <= n_vb; l_sgy <= n_sgy;
                l_al  <= n_al;
                l_dy  <= n_dy; l_dcr <= n_dcr; l_dcb <= n_dcb;
                l_jit <= n_jit;
                l_ws  <= n_ws;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x  <= b_x + s_xs;
                b_hp <= b_hp + resize(c_hst, 20);
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_x  : signed(12 downto 0);
        variable v_n  : signed(12 downto 0);
        variable v_k  : signed(13 downto 0);
        variable v_l  : signed(9 downto 0);
        variable v_g  : signed(24 downto 0);
        variable v_vi : signed(20 downto 0);
        variable v_ix : signed(9 downto 0);
        variable v_a  : signed(11 downto 0);
        variable v_b  : unsigned(3 downto 0);
        variable v_y, v_cu, v_cv : signed(11 downto 0);

        function f_beta(d : signed(12 downto 0)) return unsigned is
            variable v : signed(12 downto 0);
        begin
            v := d + 32;
            if v < 0 then return to_unsigned(0, 4);
            elsif v >= 64 then return to_unsigned(8, 4);
            else return unsigned(v(6 downto 3));
            end if;
        end function;
        function f_mul3(d : signed(11 downto 0); b : unsigned(3 downto 0)) return signed is
            variable a : signed(15 downto 0);
        begin
            -- (d x b) / 8, b = 0 .. 8
            if b(3) = '1' then return d; end if;
            a := (others => '0');
            if b(2) = '1' then a := a + shift_left(resize(d, 16), 2); end if;
            if b(1) = '1' then a := a + shift_left(resize(d, 16), 1); end if;
            if b(0) = '1' then a := a + resize(d, 16); end if;
            return resize(shift_right(a, 3), 12);
        end function;
    begin
        if rising_edge(clk) then
            -- 2: virtual px; E and H table reads; the key's x with the serration jitter
            p2_vx <= b_x(11 downto 1);
            r_e_a <= b_x(11 downto 3);
            r_h_a <= b_hp(18 downto 10);
            p2_kx <= resize(b_x(11 downto 1), 12) + resize(l_jit, 12);
            -- 3: X = |x - 1006|; the blue bowl's |x - 1545|
            v_x := signed(resize(p2_kx, 13)) - 1006;
            v_n := -v_x;
            if v_x < 0 then p3_x <= unsigned(v_n(11 downto 0)); else p3_x <= unsigned(v_x(11 downto 0)); end if;
            v_x := signed(resize(p2_vx, 13)) - 1545;
            v_n := -v_x;
            if v_x < 0 then p3_dt <= unsigned(v_n(10 downto 0)); else p3_dt <= unsigned(v_x(10 downto 0)); end if;
            p3_vx <= p2_vx;
            -- 4: tables (fabric copy); the key runs; the blue distances; the camera luma
            p4_e <= signed(r_e_q);
            p4_h <= signed(r_h_q(11 downto 0));
            p4_x <= p3_x;
            if l_tb = 0 then p4_dt <= to_signed(-4000, 13); else p4_dt <= signed(resize(l_tb, 13)) - signed(resize(p3_dt, 13)); end if;
            if l_bl = 0 then p4_dl <= to_signed(-4000, 13); else p4_dl <= signed(resize(l_bl, 13)) - signed(resize(p3_vx, 13)); end if;
            if l_br = 255 then p4_dr <= to_signed(-4000, 13); else p4_dr <= signed(resize(p3_vx, 13)) - 1792 - signed(resize(l_br, 13)); end if;
            p4_lm <= unsigned(s_lb_q);
            -- 5: E x warp x lattice; the key bit; the blue ramps; the camera term
            p5_ew <= resize(p4_e * signed('0' & l_ws), 20);
            p5_x <= p4_x;
            p5_bt <= f_beta(p4_dt); p5_bl <= f_beta(p4_dl); p5_br <= f_beta(p4_dr);
            if s_src = '1' then v_l := shift_left(signed(resize(p4_lm, 10)) - 120, 1); else v_l := signed(resize(p4_lm, 10)); end if;
            p5_cam <= resize(v_l * signed('0' & c_cg), 16);
            q_h(5) <= p4_h;
            -- 6: V index (Q8 lines); beta = the strongest pool; camera / 16
            p6_vi <= signed(resize(l_vb, 21)) + resize(p5_ew, 21);
            v_b := p5_bt;
            if p5_bl > v_b then v_b := p5_bl; end if;
            if p5_br > v_b then v_b := p5_br; end if;
            p6_be <= v_b;
            p6_cam <= resize(shift_right(p5_cam, 4), 12);
            p6_x <= p5_x;
            -- 7: V table address = sgn(Y) vi / 1024 + ov (4-line entries, wrapping)
            if l_sgy = '1' then v_vi := -p6_vi; else v_vi := p6_vi; end if;
            v_ix := v_vi(19 downto 10) + resize(c_ov, 10);
            r_v_a <= unsigned(v_ix(8 downto 0));
            p7_cam <= p6_cam;
            q_be(7) <= p6_be;
            -- 7: the key's x, pushed by the camera (bright = the bat grows): max(0, X - cam / 4)
            v_k := signed(resize(p6_x, 14)) - resize(shift_right(p6_cam, 2), 14);
            if v_k < 0 then p7_xk <= (others => '0'); else p7_xk <= v_k; end if;
            -- 8: (V EBR); the key runs
            p8_cam <= p7_cam;
            if (p7_xk >= signed(resize(l_a0, 14)) and p7_xk < signed(resize(l_a1, 14))) or
               (p7_xk >= signed(resize(l_b0, 14)) and p7_xk < signed(resize(l_b1, 14))) or
               p7_xk >= signed(resize(l_c0, 14)) then
                q_sh(8) <= '1';
            else
                q_sh(8) <= '0';
            end if;
            -- 9: V (fabric copy)
            p9_v <= signed(r_v_q(11 downto 0));
            p9_cam <= p8_cam;
            for i in 6 to 9 loop q_h(i) <= q_h(i - 1); end loop;
            for i in 8 to 15 loop q_be(i) <= q_be(i - 1); end loop;
            for i in 9 to 18 loop q_sh(i) <= q_sh(i - 1); end loop;
            for i in 12 to 18 loop q_fl(i) <= q_fl(i - 1); end loop;
            -- 10: F = H + V + camera (Q8 bands)
            p10_f <= resize(q_h(9), 14) + resize(p9_v, 14) + resize(p9_cam, 14);
            -- 11: F x rings (two partial products); the flame level
            p11_g1 <= resize(p10_f * signed('0' & c_kr(9 downto 5)), 20);
            p11_g0 <= resize(p10_f * signed('0' & c_kr(4 downto 0)), 20);
            if p10_f > C_FL then q_fl(11) <= s_flm; else q_fl(11) <= '0'; end if;
            -- 12: u = frac(F rings) (7 bits) -> palette addresses; the contour triangle for the fade
            v_g := shift_left(resize(p11_g1, 25), 5) + resize(p11_g0, 25);
            p12_u <= unsigned(v_g(15 downto 9));
            r_pb_a <= s_pal & unsigned(v_g(15 downto 9));
            r_pf_a <= (s_pal xor (q_fl(11) and not (q_sh(11) xor s_inv))) & unsigned(v_g(15 downto 9));
            -- 13: (palette EBRs); contour triangle
            if p12_u(6) = '1' then p13_t <= not p12_u(5 downto 0); else p13_t <= p12_u(5 downto 0); end if;
            -- 14: palettes (fabric copy); alpha with the contours in it: alpha8 + 4 t - 128, clamped, -> 0 .. 8
            p14_by <= signed(resize(unsigned(r_pb_q(29 downto 20)), 12));
            p14_bu <= signed(resize(unsigned(r_pb_q(19 downto 10)), 12));
            p14_bv <= signed(resize(unsigned(r_pb_q(9 downto 0)), 12));
            p14_fy <= signed(resize(unsigned(r_pf_q(29 downto 20)), 12));
            p14_fu <= signed(resize(unsigned(r_pf_q(19 downto 10)), 12));
            p14_fv <= signed(resize(unsigned(r_pf_q(9 downto 0)), 12));
            v_a := signed(resize(l_al, 12)) + signed(resize(p13_t & "00", 12)) - 128;
            if v_a < 0 then p14_al <= (others => '0');
            elsif v_a > 239 then p14_al <= to_unsigned(8, 4);
            else p14_al <= resize(shift_right(unsigned(v_a(7 downto 0)) + 16, 5), 4); end if;
            -- 15: blue - bg; the key colour's red fall-off
            p15_by <= p14_by; p15_bu <= p14_bu; p15_bv <= p14_bv;
            if s_pal = '1' then                               -- the pools: blue (Ember) / magenta (Blue)
                p15_dy <= to_signed(C_BU_POOL2_Y, 12) - p14_by;
                p15_du <= to_signed(C_BU_POOL2_U, 12) - p14_bu;
                p15_dv <= to_signed(C_BU_POOL2_V, 12) - p14_bv;
            else
                p15_dy <= to_signed(C_BU_BLUE_Y, 12) - p14_by;
                p15_du <= to_signed(C_BU_BLUE_U, 12) - p14_bu;
                p15_dv <= to_signed(C_BU_BLUE_V, 12) - p14_bv;
            end if;
            p15_fy <= p14_fy + resize(l_dy, 12);
            p15_fu <= p14_fu + resize(l_dcr, 12);
            p15_fv <= p14_fv + resize(l_dcb, 12);
            p15_al <= p14_al;
            -- 16: bg = palette + (blue - palette) beta / 8
            p16_y <= p15_by + f_mul3(p15_dy, q_be(15));
            p16_u <= p15_bu + f_mul3(p15_du, q_be(15));
            p16_v <= p15_bv + f_mul3(p15_dv, q_be(15));
            p16_fy <= p15_fy; p16_fu <= p15_fu; p16_fv <= p15_fv;
            p16_al <= p15_al;
            -- 17: key - bg; the final key bit (Cutout inverts the shape; flames add to it)
            p17_y <= p16_y; p17_u <= p16_u; p17_v <= p16_v;
            p17_dy <= p16_fy - p16_y; p17_du <= p16_fu - p16_u; p17_dv <= p16_fv - p16_v;
            if (q_sh(16) xor s_inv) = '1' then
                p17_k <= '1'; p17_al <= p16_al;
            elsif q_fl(16) = '1' then
                p17_k <= '1'; p17_al <= to_unsigned(8, 4);
            else
                p17_k <= '0'; p17_al <= (others => '0');
            end if;
            -- 18: (key - bg) x alpha / 8; the trailing-edge outline (the key ended a few px ago)
            q_kd <= q_kd(4 downto 0) & p17_k;
            p18_y <= p17_y; p18_u <= p17_u; p18_v <= p17_v;
            p18_ky <= f_mul3(p17_dy, p17_al);
            p18_ku <= f_mul3(p17_du, p17_al);
            p18_kv <= f_mul3(p17_dv, p17_al);
            case to_integer(s_otap) is
                when 2      => p18_ol <= q_kd(1) and not p17_k;
                when 3      => p18_ol <= q_kd(2) and not p17_k;
                when others => p18_ol <= q_kd(4) and not p17_k;
            end case;
            -- 19: out = bg + (key - bg) alpha / 8; the outline colour; clamps; black outside the generated avid
            v_y  := p18_y + p18_ky;
            v_cu := p18_u + p18_ku;
            v_cv := p18_v + p18_kv;
            if p18_ol = '1' then
                v_y := to_signed(C_BU_OL_Y, 12); v_cu := to_signed(C_BU_OL_U, 12); v_cv := to_signed(C_BU_OL_V, 12);
            end if;
            if s_gav_sr(16) = '0' then
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            else
                if v_y < 64 then s_y19 <= to_unsigned(64, 10); elsif v_y > 940 then s_y19 <= to_unsigned(940, 10); else s_y19 <= unsigned(v_y(9 downto 0)); end if;
                if v_cu < 0 then s_u19 <= (others => '0'); elsif v_cu > 1023 then s_u19 <= (others => '1'); else s_u19 <= unsigned(v_cu(9 downto 0)); end if;
                if v_cv < 0 then s_v19 <= (others => '0'); elsif v_cv > 1023 then s_v19 <= (others => '1'); else s_v19 <= unsigned(v_cv(9 downto 0)); end if;
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(16);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (each two adjacent 8-sums; the 8-back taps come from block-RAM rings:
    -- the still's analog softness, and the HD HDMI bar fix), luma 13/16 and
    -- the blanking gate; the syncs ride a block-RAM ring of the same latency
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- ring addresses: one write per clock; the read, issued two clocks ahead and copied once,
            -- is the sample written 8 clocks before the one being added
            rg_wa <= rg_wa + 1;
            rg_ra <= rg_wa - 5;
            sy_ra <= rg_wa - 21;
            s_cay <= unsigned(ba1_q(15 downto 6)); s_cau <= unsigned(ba1_q(5 downto 0)) & unsigned(ba2_q(15 downto 12)); s_cav <= unsigned(ba2_q(11 downto 2));
            s_cby <= unsigned(bb1_q(15 downto 6)); s_cbu <= unsigned(bb1_q(5 downto 0)) & unsigned(bb2_q(15 downto 12)); s_cbv <= unsigned(bb2_q(11 downto 2));
            syr_c <= syr_q;
            -- The running 8-sums snap back to neutral (the off-avid input of both boxes) once 8 blank
            -- samples are in, so a glitched sample can't offset the colours for good.
            -- box A: stage 20 sums, stage 21 output
            if s_bah = "00000000" then
                s_bay <= to_unsigned(64 * 8, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
            else
                s_bay <= s_bay + s_y19 - s_cay;
                s_bau <= s_bau + s_u19 - s_cau;
                s_bav <= s_bav + s_v19 - s_cav;
            end if;
            s_bayd <= s_bay; s_baud <= s_bau; s_bavd <= s_bav;
            -- box B input: neutral off the generated avid (what box A gives there anyway)
            if s_gav_sr(18) = '1' then
                v_sc := resize(s_bay, 14) + s_bayd; s_y21 <= v_sc(13 downto 4);
                v_sc := resize(s_bau, 14) + s_baud; s_u21 <= v_sc(13 downto 4);
                v_sc := resize(s_bav, 14) + s_bavd; s_v21 <= v_sc(13 downto 4);
            else
                s_y21 <= to_unsigned(64, 10); s_u21 <= to_unsigned(512, 10); s_v21 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(18);
            -- box B: stage 22 sums, stage 23 output
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(64 * 8, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y21 - s_cby;
                s_bbu <= s_bbu + s_u21 - s_cbu;
                s_bbv <= s_bbv + s_v21 - s_cbv;
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y23 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v23 <= v_sc(13 downto 4);
            -- 24: luma 13/16
            if s_y23 > 64 then v_dl := s_y23 - 64; else v_dl := (others => '0'); end if;
            s_y24 <= s_y23 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u24 <= s_u23; s_v24 <= s_v23;
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
    data_out.hsync_n <= syr_c(0);
    data_out.vsync_n <= syr_c(1);
    data_out.field_n <= syr_c(2);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture bunsen;
