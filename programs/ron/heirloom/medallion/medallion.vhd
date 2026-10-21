-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Medallion v0.5.0 -- Heirloom #12, rebuilt from the photo WIN_20230305_23_51_38.
--
-- The photo: two diamond systems of nested fire rings (centres x 527 / 1466, one
-- vertical centre 522), horizontal stripes (18 boundaries, ~45 lines in the middle,
-- ~70 near the top / bottom, symmetric about line 553) that shift every other stripe
-- by half a colour cycle, and an oval medallion (980, 533; 459 x 431) keying in a
-- CMY / grey plaid (304 x 207.5 px tiles).
--
-- The patch, as fitted (medallion.py has every constant):
--   w      = T(x) + V(y) + Etch * luma     horizontal triangle (peak on the central fold, rounded,
--                                          held at the line start) + curved vertical ramp
--   level  = min(RING(p), E(w))            ring waveform (two periods, measured) / outer envelope
--   colour = PAL8[floor(level) XOR 4*stripe]  a 3-bit colourizer; the stripes flip its MSB
--   medallion: chord test per line, plaid tile ROM inside.
--
--   K1  Diamonds  diamond systems across (1 .. 6, 2.04 = the photo)
--   K2  Rings     ring density (0.3 .. 3x)
--   K3  Reach     where the rings stop and the flat stripes begin
--   K4  Stripes   stripe density (0.25 .. 3x)
--   K5  Speed     bipolar: rings stream out of the diamond centres, stripes roll, the plaid drifts
--   K6  Etch      how far the picture (or the Osc) bends the rings
--   S7  Flip      the stripes' half-cycle colour flip (Off = plain nested diamonds)
--   S8  Steps     Posterize (the photo's 8 levels) / Smooth ramps
--   S9  Swap      plaid outside, fire rings inside the medallion
--   S10 Wave      Tri (diamonds) / Saw (slanted ramps)
--   S11 Source    Video / Osc
--   P12 MEDALLION size of the medallion: 0 = the open diamond field, 100 % = a plaid test card
--
-- Engine: vblank-zone sequencer (one serial multiplier, one serial divider, a shared
-- glide), per-line engine kicked at each line start (computes the NEXT line: V, stripe
-- phase, chord half-width by a serial square root, plaid row), beam DDAs (triangle
-- distance with reflections, plaid column), 24-stage pixel path.  Virtual 1080-line
-- coordinates (SD shows the centre 4:3 of the picture).  Generated avid; output
-- conditioning (chroma and luma [1 2 2 2 2 2 2 2 1]/16, luma 13/16) before the blanking
-- gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.medallion_pkg.all;
use work.all;

architecture medallion of program_top is
    constant C_LAT : integer := 24;   -- = processing_delay_clks

    -- fitted constants (h = half virtual px; see medallion.py)
    constant C_XF2  : integer := 1993;     -- central fold (triangle peak)
    constant C_XH2  : integer := 114;      -- line-start hold point
    constant C_YC8  : integer := 133734;   -- vertical centre, Q8
    constant C_CX2  : integer := 1961;     -- medallion centre x
    constant C_CY8  : integer := 136497;   -- medallion centre y, Q8
    constant C_PX02 : integer := 1380;     -- plaid origin x
    constant C_PY08 : integer := 34432;    -- plaid origin y, Q8
    constant C_PDXW : integer := 77862;    -- plaid tile width, Q8 (drift wrap)
    constant C_PDYW : integer := 53120;    -- plaid tile height, Q8

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

    -- line buffer: (luma >> 2) - 16, clipped at 0, of the line being drawn
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (one read port each, registered address and data)
    signal r_ring_a : unsigned(9 downto 0) := (others => '0');
    signal r_ring_q : unsigned(7 downto 0) := (others => '0');
    signal r_pal_a  : unsigned(8 downto 0) := (others => '0');
    signal r_pal_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal r_tile_a : unsigned(10 downto 0) := (others => '0');
    signal r_tile_q : unsigned(3 downto 0) := (others => '0');
    signal r_psi_a  : unsigned(8 downto 0) := (others => '0');
    signal r_psi_q  : signed(11 downto 0) := (others => '0');
    signal r_psi_q2 : signed(11 downto 0) := (others => '0');

    -- oscillator source (Source = Osc), virtual units
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxi : unsigned(15 downto 0) := (others => '0');
    signal s_oxs : unsigned(9 downto 0) := to_unsigned(192, 10);
    signal s_oys : unsigned(11 downto 0) := to_unsigned(320, 12);
    signal s_oty : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_x0   : unsigned(8 downto 0) := (others => '0');
    signal s_tk   : unsigned(16 downto 0) := to_unsigned(55160, 17);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(9 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw speed, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(468 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(521 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(420 * 8, 13);
    signal s_g4 : unsigned(12 downto 0) := to_unsigned(534 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(396 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(487 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_k5 : unsigned(9 downto 0) := to_unsigned(680, 10);
    signal s_vm : unsigned(9 downto 0) := (others => '0');
    signal s_vneg, s_vstill : std_logic := '0';
    signal s_flip, s_smooth, s_swap, s_saw, s_src : std_logic := '0';

    -- per-field constants
    signal c_sq   : unsigned(19 downto 0) := (others => '0');
    signal c_n8   : unsigned(10 downto 0) := to_unsigned(523, 11);
    signal c_h2   : unsigned(10 downto 0) := to_unsigned(939, 11);
    signal c_p2   : unsigned(11 downto 0) := to_unsigned(1878, 12);
    signal c_q    : signed(13 downto 0) := (others => '0');
    signal c_zi   : unsigned(10 downto 0) := (others => '0');
    signal c_di   : std_logic := '0';
    signal c_si   : unsigned(10 downto 0) := (others => '0');
    signal c_h2mx : unsigned(10 downto 0) := to_unsigned(937, 11);       -- H2 - xs
    signal c_p2mx : unsigned(11 downto 0) := to_unsigned(1876, 12);      -- P2 - xs
    signal c_hold : unsigned(11 downto 0) := to_unsigned(112, 12);       -- 114 - xs
    signal c_kg   : unsigned(9 downto 0) := to_unsigned(250, 10);
    signal c_we2  : unsigned(11 downto 0) := to_unsigned(1291, 12);
    signal c_gs   : unsigned(9 downto 0) := to_unsigned(255, 10);
    signal c_eq   : unsigned(8 downto 0) := to_unsigned(64, 9);
    signal c_rh   : unsigned(10 downto 0) := to_unsigned(861, 11);
    signal c_rh2  : unsigned(21 downto 0) := (others => '0');
    signal c_v2   : unsigned(17 downto 0) := (others => '0');
    signal c_dth, c_dpx, c_dpy : unsigned(9 downto 0) := (others => '0');
    signal c_dths : unsigned(10 downto 0) := (others => '0');
    signal c_pa0  : unsigned(18 downto 0) := (others => '0');
    signal c_ta0  : unsigned(23 downto 0) := (others => '0');
    signal s_th   : unsigned(15 downto 0) := (others => '0');               -- ring flow, Q6 ring index
    signal s_ths  : unsigned(15 downto 0) := (others => '0');               -- stripe roll, Q15 half-cycles
    signal s_pdx  : signed(18 downto 0) := (others => '0');                 -- plaid drift, Q8 px
    signal s_pdy  : signed(18 downto 0) := (others => '0');

    -- field serial multiplier / divider
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal d_rem : unsigned(11 downto 0) := (others => '0');
    signal d_q   : unsigned(18 downto 0) := (others => '0');
    signal d_num : unsigned(18 downto 0) := (others => '0');

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
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_dq2  : unsigned(11 downto 0) := (others => '0');
    signal e_d104 : unsigned(17 downto 0) := (others => '0');
    signal e_f    : unsigned(7 downto 0) := (others => '0');
    signal e_pa, e_pb, e_ps : signed(11 downto 0) := (others => '0');
    signal e_v2a  : unsigned(11 downto 0) := (others => '0');
    signal e_v2   : unsigned(12 downto 0) := (others => '0');
    signal e_dd   : signed(12 downto 0) := (others => '0');
    signal e_psg  : signed(12 downto 0) := (others => '0');
    signal e_ph   : signed(12 downto 0) := (others => '0');
    signal e_typ  : std_logic := '0';
    signal e_dyh  : signed(11 downto 0) := (others => '0');
    signal e_sr   : signed(22 downto 0) := (others => '0');
    signal e_out  : std_logic := '0';
    signal e_sqn  : unsigned(21 downto 0) := (others => '0');
    signal e_rem  : unsigned(12 downto 0) := (others => '0');
    signal e_root : unsigned(10 downto 0) := (others => '0');
    signal e_hw2  : unsigned(11 downto 0) := (others => '0');
    signal e_dy3  : signed(16 downto 0) := (others => '0');
    signal e_row  : unsigned(2 downto 0) := (others => '0');
    signal m_a    : unsigned(29 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(31 downto 0) := (others => '0');
    signal m_neg  : std_logic := '0';
    signal n_v2   : unsigned(12 downto 0) := (others => '0');
    signal n_typ  : std_logic := '0';
    signal n_hw2  : unsigned(11 downto 0) := (others => '0');
    signal n_row  : unsigned(2 downto 0) := (others => '0');

    -- beam
    signal b_x2  : unsigned(11 downto 0) := (others => '0');
    signal b_z   : unsigned(10 downto 0) := (others => '0');
    signal b_zd  : std_logic := '0';
    signal b_s   : unsigned(10 downto 0) := (others => '0');
    signal b_ta  : unsigned(23 downto 0) := (others => '0');
    signal b_px  : unsigned(10 downto 0) := (others => '0');
    signal l_v2  : unsigned(12 downto 0) := (others => '0');
    signal l_flip : std_logic := '0';
    signal l_hw2 : unsigned(11 downto 0) := (others => '0');
    signal l_row : unsigned(2 downto 0) := (others => '0');

    -- pixel path
    signal p2_z, p2_d, p2_s : unsigned(10 downto 0) := (others => '0');
    signal p2_lt60, p2_lt48 : std_logic := '0';
    signal p2_dx  : unsigned(11 downto 0) := (others => '0');
    signal p3_t   : unsigned(11 downto 0) := (others => '0');
    signal p3_lm  : unsigned(7 downto 0) := (others => '0');
    signal p3_in  : std_logic := '0';
    signal p4_w0  : unsigned(12 downto 0) := (others => '0');
    signal p4_ea, p4_eb : unsigned(12 downto 0) := (others => '0');
    signal p5_w0  : unsigned(12 downto 0) := (others => '0');
    signal p5_e   : unsigned(9 downto 0) := (others => '0');
    signal p6_w   : unsigned(13 downto 0) := (others => '0');
    signal p7_pm, p7_d : signed(14 downto 0) := (others => '0');
    signal p8_p0, p8_p1 : signed(18 downto 0) := (others => '0');
    signal p8_p2 : signed(19 downto 0) := (others => '0');
    signal p8_da, p8_d5 : signed(14 downto 0) := (others => '0');
    signal p9_a, p9_b : signed(19 downto 0) := (others => '0');
    signal p9_eb  : signed(14 downto 0) := (others => '0');
    signal p10_e, p11_e, p12_e : unsigned(7 downto 0) := (others => '0');
    signal p12_r  : unsigned(7 downto 0) := (others => '0');
    signal p13_l  : unsigned(7 downto 0) := (others => '0');
    signal s_tc4  : unsigned(2 downto 0) := (others => '0');
    type t_c3 is array (natural range <>) of unsigned(2 downto 0);
    signal s_tcp  : t_c3(5 to 16) := (others => (others => '0'));
    signal s_inp  : std_logic_vector(4 to 16) := (others => '0');
    signal p16_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y17, s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_csy, s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxy, s_boxu, s_boxv, s_boxyd, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y23, s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

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
            r_ring_q <= C_MD_RING(to_integer(r_ring_a));
            r_pal_q  <= C_MD_PAL(to_integer(r_pal_a));
            r_tile_q <= C_MD_TILE(to_integer(r_tile_a));
            r_psi_q  <= C_MD_PSI(to_integer(r_psi_a));
            r_psi_q2 <= r_psi_q;
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_r  : unsigned(12 downto 0);
        variable v_dr : unsigned(12 downto 0);
        variable v_x  : signed(13 downto 0);
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
            s_k5 <= unsigned(registers_in(4));
            v_s := signed(resize(s_k5, 11)) - 512;
            if v_s > -9 and v_s < 9 then s_vstill <= '1'; else s_vstill <= '0'; end if;
            s_vneg <= v_s(10);
            if v_s(10) = '1' then v_m := unsigned(not v_s(9 downto 0)) - 7; else v_m := unsigned(v_s(9 downto 0)) - 8; end if;
            s_vm <= v_m;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_flip   <= registers_in(6)(0);
                s_smooth <= registers_in(6)(1);
                s_swap   <= registers_in(6)(2);
                s_saw    <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_xs     <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_x0 <= to_unsigned(480, 9); s_tk <= to_unsigned(110320, 17); s_oxs <= to_unsigned(384, 10); s_oxi <= to_unsigned(46080, 16);
                    when 3      => s_x0 <= (others => '0');     s_tk <= to_unsigned(82740, 17);  s_oxs <= to_unsigned(288, 10); s_oxi <= (others => '0');
                    when others => s_x0 <= (others => '0');     s_tk <= to_unsigned(55160, 17);  s_oxs <= to_unsigned(192, 10); s_oxi <= (others => '0');
                end case;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 14) + shift_left(resize(s_ys, 14), 2), 2), 12);

            -- shared glide (1/8 of the gap per field, +-2 deadband)
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

            -- motion (last field's steps), then the plaid drift wraps
            case to_integer(s_zc) is
                when 20 =>
                    if s_vneg = '1' then
                        s_th  <= s_th - c_dth;  s_ths <= s_ths - c_dths;
                        s_pdx <= s_pdx - signed(resize(c_dpx, 19)); s_pdy <= s_pdy - signed(resize(c_dpy, 19));
                    else
                        s_th  <= s_th + c_dth;  s_ths <= s_ths + c_dths;
                        s_pdx <= s_pdx + signed(resize(c_dpx, 19)); s_pdy <= s_pdy + signed(resize(c_dpy, 19));
                    end if;
                when 21 =>
                    if s_pdx >= C_PDXW then s_pdx <= s_pdx - C_PDXW; elsif s_pdx < 0 then s_pdx <= s_pdx + C_PDXW; end if;
                    if s_pdy >= C_PDYW then s_pdy <= s_pdy - C_PDYW; elsif s_pdy < 0 then s_pdy <= s_pdy + C_PDYW; end if;
                when 22 =>
                    -- |pa0| = 1380*256 - x0*256 + 2*pdx  (pa0 = (x0 - 1380)*256 - 2*pdx < 0 always)
                    c_pa0 <= to_unsigned(C_PX02 * 256, 19) - shift_left(resize(s_x0, 19), 8) + unsigned(shift_left(resize(s_pdx, 19), 1));
                when others => null;
            end case;

            -- serial products: slot k loads at zc = 32 + 24 k, 16 iterations,
            -- result stable from load + 17 (stores at load + 18)
            if s_zone = '1' and (s_zc = 32 or s_zc = 56 or s_zc = 80 or s_zc = 104 or s_zc = 128 or s_zc = 152 or
               s_zc = 176 or s_zc = 200 or s_zc = 224 or s_zc = 248 or s_zc = 272 or s_zc = 296 or s_zc = 320 or
               s_zc = 344 or s_zc = 368 or s_zc = 392 or s_zc = 416) then
                f_acc <= (others => '0');
                case to_integer(s_zc) is
                    when 32  => f_a <= resize(s_g1(12 downto 3), 36); f_b <= resize(s_g1(12 downto 3), 16);
                    when 56  => f_a <= resize(c_sq, 36);              f_b <= to_unsigned(1280, 16);
                    when 80  => f_a <= resize(s_g2(12 downto 3), 36); f_b <= resize(s_g2(12 downto 3), 16);
                    when 104 => f_a <= resize(c_sq, 36);              f_b <= to_unsigned(678, 16);
                    when 128 => f_a <= resize(s_g3(12 downto 3), 36); f_b <= to_unsigned(2563, 16);
                    when 152 => f_a <= resize(s_g4(12 downto 3), 36); f_b <= resize(s_g4(12 downto 3), 16);
                    when 176 => f_a <= resize(c_sq, 36);              f_b <= to_unsigned(704, 16);
                    when 200 => f_a <= resize(s_g6(12 downto 3), 36); f_b <= resize(s_g6(12 downto 3), 16);
                    when 224 => f_a <= resize(c_sq, 36);              f_b <= to_unsigned(430, 16);
                    when 248 => f_a <= resize(s_gp(12 downto 3), 36); f_b <= to_unsigned(1812, 16);
                    when 272 => f_a <= resize(c_rh, 36);              f_b <= resize(c_rh, 16);
                    when 296 => f_a <= resize(s_vm, 36);              f_b <= resize(s_vm, 16);
                    when 320 => f_a <= resize(c_v2, 36);              f_b <= to_unsigned(455, 16);
                    when 344 => f_a <= resize(c_v2, 36);              f_b <= to_unsigned(1365, 16);
                    when 368 => f_a <= resize(c_v2, 36);              f_b <= to_unsigned(340, 16);
                    when 392 => f_a <= resize(c_v2, 36);              f_b <= to_unsigned(204, 16);
                    when others => f_a <= resize(c_pa0, 36);          f_b <= to_unsigned(27580, 16);
                end case;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(34 downto 0) & '0';
                f_b <= '0' & f_b(15 downto 1);
            end if;
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 50  => c_sq  <= f_acc(19 downto 0);
                    when 74  => c_n8  <= to_unsigned(256, 11) + resize(f_acc(31 downto 20), 11);
                    when 98  => c_sq  <= f_acc(19 downto 0);
                    when 122 => c_kg  <= to_unsigned(75, 10) + resize(f_acc(29 downto 20), 10);
                    when 146 => c_we2 <= to_unsigned(240, 12) + resize(f_acc(21 downto 10), 12);
                    when 170 => c_sq  <= f_acc(19 downto 0);
                    when 194 => c_gs  <= to_unsigned(64, 10) + resize(f_acc(29 downto 20), 10);
                    when 218 => c_sq  <= f_acc(19 downto 0);
                    when 242 => c_eq  <= resize(f_acc(28 downto 20), 9);
                    when 266 => c_rh  <= resize(f_acc(20 downto 10), 11);
                    when 290 => c_rh2 <= f_acc(21 downto 0);
                    when 314 => c_v2  <= f_acc(17 downto 0);
                    when 338 => if s_vstill = '1' then c_dth  <= (others => '0'); else c_dth  <= resize(f_acc(27 downto 18), 10); end if;
                    when 362 => if s_vstill = '1' then c_dths <= (others => '0'); else c_dths <= f_acc(28 downto 18); end if;
                    when 386 => if s_vstill = '1' then c_dpx  <= (others => '0'); else c_dpx  <= resize(f_acc(27 downto 18), 10); end if;
                    when 410 => if s_vstill = '1' then c_dpy  <= (others => '0'); else c_dpy  <= resize(f_acc(27 downto 18), 10); end if;
                    when 434 => c_ta0 <= (not f_acc(31 downto 8)) + 1;     -- -(|pa0| * TK >> 8) mod 2^24
                    when others => null;
                end case;
            end if;

            -- divider: H2 = 491520 / n8 (restoring, zc 80 .. 99, one subtract per step)
            if s_zc = 80 then
                d_num <= to_unsigned(491520, 19);
                d_rem <= (others => '0');
                d_q   <= (others => '0');
            elsif s_zc > 80 and s_zc <= 99 then
                v_r := d_rem & d_num(18);
                v_dr := v_r - resize(c_n8, 13);
                if v_dr(12) = '0' then
                    d_rem <= v_dr(11 downto 0);
                    d_q <= d_q(17 downto 0) & '1';
                else
                    d_rem <= v_r(11 downto 0);
                    d_q <= d_q(17 downto 0) & '0';
                end if;
                d_num <= d_num(17 downto 0) & '0';
            elsif s_zc = 100 then
                c_h2 <= d_q(10 downto 0);
            end if;

            -- triangle state at the line start: q = (max(x0, 114) - 1993) mod 2 H2
            case to_integer(s_zc) is
                when 102 =>
                    c_p2 <= c_h2 & '0';
                    if s_x0 > C_XH2 then v_x := signed(resize(s_x0, 14)) - C_XF2; else v_x := to_signed(C_XH2 - C_XF2, 14); end if;
                    c_q <= v_x;
                when 103 | 104 | 105 | 106 =>
                    if c_q < 0 then c_q <= c_q + signed(resize(c_p2, 14)); end if;
                when 108 =>
                    c_h2mx <= c_h2 - s_xs;
                    c_p2mx <= c_p2 - s_xs;
                    c_hold <= to_unsigned(C_XH2, 12) - s_xs;
                when 107 =>
                    if c_q <= signed(resize(c_h2, 14)) then
                        c_zi <= unsigned(c_q(10 downto 0)); c_di <= '1';
                    else
                        c_zi <= resize(unsigned(signed(resize(c_p2, 14)) - c_q), 11); c_di <= '0';
                    end if;
                    if c_q >= signed(resize(c_h2, 14)) then
                        c_si <= resize(unsigned(c_q - signed(resize(c_h2, 14))), 11);
                    else
                        c_si <= unsigned(c_q(10 downto 0));
                    end if;
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
    -- Source side: oscillator (2-D triangle lattice, 341 x 205 virtual px,
    -- drifting), line-buffer write of (luma >> 2) - 16
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= s_oxi + shift_left(resize(s_ot, 16), 8) + shift_left(resize(s_ot, 16), 9);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= to_unsigned(0, 16) - shift_left(resize(s_ot, 16), 8) - shift_left(resize(s_ot, 16), 9); end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            s_oty <= to_unsigned(64, 10) + resize(f_tri7(s_ox(15 downto 8)), 10) + shift_left(resize(f_tri7(s_ox(15 downto 8)), 10), 1)
                     + resize(f_tri7(s_oy(15 downto 8)), 10) + shift_left(resize(f_tri7(s_oy(15 downto 8)), 10), 1);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then v_l := s_oty(9 downto 2); else v_l := unsigned(data_in.y(9 downto 2)); end if;
            if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line
    -- 0), computes the NEXT line: V, the stripe flip, the medallion chord
    -- half-width (serial square root) and the plaid row, on one serial
    -- multiplier (16 iterations, result at load + 17)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d  : signed(19 downto 0);
        variable v_p  : signed(32 downto 0);
        variable v_rm : unsigned(12 downto 0);
        variable v_tr : unsigned(12 downto 0);
        variable v_dy : signed(19 downto 0);
        variable v_t  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 600 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy <= (others => '0');
                e_lc <= (others => '0');
            elsif g_start = '1' then
                e_vy <= e_vy + resize(s_ys, 19);
                e_lc <= (others => '0');
            elsif e_lc /= 255 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + resize(m_a, 32); end if;
            m_a <= m_a(28 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    v_d := signed(resize(e_vy, 20)) - C_YC8;
                    if v_d < 0 then v_d := -v_d; end if;
                    e_dq2 <= unsigned(v_d(17 downto 6));
                    e_f   <= e_vy(9 downto 2);
                    r_psi_a <= e_vy(18 downto 10);
                when 2 =>
                    m_a <= resize(e_dq2, 30); m_b <= to_unsigned(499, 16); m_acc <= (others => '0');
                    e_d104 <= shift_left(resize(e_dq2, 18), 6) + shift_left(resize(e_dq2, 18), 5) + shift_left(resize(e_dq2, 18), 3);
                    r_psi_a <= e_vy(18 downto 10) + 1;
                when 4 =>
                    e_pa <= r_psi_q2;
                when 5 =>
                    e_pb <= r_psi_q2;
                when 19 =>
                    e_v2a <= m_acc(21 downto 10);
                    m_a <= resize(e_d104, 30); m_b <= resize(e_dq2, 16); m_acc <= (others => '0');
                when 36 =>
                    e_v2 <= resize(e_v2a, 13) + resize(m_acc(31 downto 20), 13);
                    if e_pb < e_pa then
                        m_a <= resize(unsigned(e_pa - e_pb), 30); m_neg <= '1';
                    else
                        m_a <= resize(unsigned(e_pb - e_pa), 30); m_neg <= '0';
                    end if;
                    m_b <= resize(e_f, 16); m_acc <= (others => '0');
                when 53 =>
                    v_p := signed(resize(m_acc, 33));
                    if m_neg = '1' then v_p := -v_p; end if;
                    e_dd <= resize(shift_right(v_p, 8), 13);
                when 54 =>
                    e_ps <= resize(resize(e_pa, 13) + e_dd, 12);
                when 55 =>
                    if e_ps < 0 then m_a <= resize(unsigned(-e_ps), 30); m_neg <= '1'; else m_a <= resize(unsigned(e_ps), 30); m_neg <= '0'; end if;
                    m_b <= resize(c_gs, 16); m_acc <= (others => '0');
                when 72 =>
                    v_p := signed(resize(m_acc, 33));
                    if m_neg = '1' then v_p := -v_p; end if;
                    e_psg <= resize(shift_right(v_p, 8), 13);
                when 73 =>
                    e_ph <= e_psg - signed(resize(s_ths(15 downto 8), 13));
                    v_dy := signed(resize(e_vy, 20)) - C_CY8;
                    e_dyh <= resize(shift_right(v_dy, 7), 12);
                when 74 =>
                    v_t := e_ph + 64;
                    e_typ <= not v_t(7);
                    if e_dyh < 0 then m_a <= resize(unsigned(-e_dyh), 30); m_b <= resize(unsigned(-e_dyh), 16);
                    else m_a <= resize(unsigned(e_dyh), 30); m_b <= resize(unsigned(e_dyh), 16); end if;
                    m_acc <= (others => '0');
                when 91 =>
                    e_sr <= signed(resize(c_rh2, 23)) - signed(resize(m_acc(21 downto 0), 23));
                when 92 =>
                    if e_sr <= 0 then e_out <= '1'; else e_out <= '0'; end if;
                    e_sqn <= unsigned(e_sr(21 downto 0));
                    e_rem <= (others => '0');
                    e_root <= (others => '0');
                when 93 | 94 | 95 | 96 | 97 | 98 | 99 | 100 | 101 | 102 | 103 =>
                    -- digit-by-digit square root, one bit per clock
                    v_rm := e_rem(10 downto 0) & e_sqn(21 downto 20);
                    v_tr := e_root(10 downto 0) & "01";
                    if v_rm >= v_tr then
                        e_rem <= v_rm - v_tr;
                        e_root <= e_root(9 downto 0) & '1';
                    else
                        e_rem <= v_rm;
                        e_root <= e_root(9 downto 0) & '0';
                    end if;
                    e_sqn <= e_sqn(19 downto 0) & "00";
                when 104 =>
                    m_a <= resize(e_root, 30); m_b <= to_unsigned(1091, 16); m_acc <= (others => '0');
                when 121 =>
                    if e_out = '1' then e_hw2 <= (others => '0'); else e_hw2 <= m_acc(21 downto 10); end if;
                    v_dy := signed(resize(e_vy, 20)) - C_PY08 - resize(s_pdy, 20);
                    e_dy3 <= resize(shift_right(v_dy, 3), 17);
                when 122 =>
                    if e_dy3 < 0 then m_a <= resize(unsigned(-e_dy3), 30); m_neg <= '1'; else m_a <= resize(unsigned(e_dy3), 30); m_neg <= '0'; end if;
                    m_b <= to_unsigned(2527, 16); m_acc <= (others => '0');
                when 139 =>
                    v_p := signed(resize(m_acc, 33));
                    if m_neg = '1' then v_p := -v_p; end if;
                    e_row <= unsigned(v_p(23 downto 21));
                when 140 =>
                    n_v2 <= e_v2; n_typ <= e_typ; n_hw2 <= e_hw2; n_row <= e_row;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: triangle distance-to-peak DDA (reflects at the peak and the
    -- minimum; held left of the line-start hold point), saw ramp, plaid column
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_x2 <= resize(s_x0, 12);
                b_z  <= c_zi; b_zd <= c_di; b_s <= c_si;
                b_ta <= c_ta0;
                b_px <= (others => '0');
                l_v2 <= n_v2; l_flip <= n_typ and s_flip; l_hw2 <= n_hw2; l_row <= n_row;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                b_x2 <= b_x2 + s_xs;
                b_ta <= b_ta + s_tk;
                if b_x2 > c_hold then
                    if b_zd = '1' then
                        if b_z > c_h2mx then
                            b_z <= resize(c_p2mx - b_z, 11); b_zd <= '0';
                        else
                            b_z <= b_z + s_xs;
                        end if;
                    else
                        if b_z < s_xs then
                            b_z <= resize(s_xs - b_z, 11); b_zd <= '1';
                        else
                            b_z <= b_z - s_xs;
                        end if;
                    end if;
                    if b_s >= c_h2mx then b_s <= b_s - c_h2mx; else b_s <= b_s + s_xs; end if;
                end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage 1 = beam values of the pixel)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_t  : unsigned(11 downto 0);
        variable v_dx : signed(12 downto 0);
        variable v_e  : unsigned(16 downto 0);
        variable v_w  : unsigned(13 downto 0);
        variable v_v  : signed(19 downto 0);
        variable v_eb : signed(15 downto 0);
        variable v_l  : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: distances, medallion |dx|, plaid tile address
            p2_z <= b_z; p2_s <= b_s;
            p2_d <= c_h2 - b_z;
            if b_z < 60 then p2_lt60 <= '1'; else p2_lt60 <= '0'; end if;
            if c_h2 - b_z < 48 then p2_lt48 <= '1'; else p2_lt48 <= '0'; end if;
            v_dx := signed(resize(b_x2, 13)) - C_CX2;
            if v_dx < 0 then p2_dx <= resize(unsigned(-v_dx), 12); else p2_dx <= unsigned(v_dx(11 downto 0)); end if;
            r_tile_a <= l_row & b_ta(23 downto 16);

            -- 3: T (rounded peak, filleted minimum / saw); luma; inside test
            if s_saw = '1' then
                p3_t <= resize(p2_s, 12) + 30;
            elsif p2_lt60 = '1' then
                p3_t <= resize(c_h2, 12) - resize(C_MD_RQM(to_integer(p2_z(5 downto 0))), 12);
            elsif p2_lt48 = '1' then
                p3_t <= to_unsigned(54, 12) + resize(C_MD_RQN(to_integer(p2_d(5 downto 0))), 12);
            else
                p3_t <= resize(c_h2, 12) + 30 - resize(p2_z, 12);
            end if;
            p3_lm <= unsigned(s_lb_q);
            if p2_dx < l_hw2 then p3_in <= not s_swap; else p3_in <= s_swap; end if;

            -- 4: w0 = T + V; etch partials
            p4_w0 <= resize(p3_t, 13) + l_v2;
            p4_ea <= resize(p3_lm(3 downto 0) * c_eq, 13);
            p4_eb <= resize(p3_lm(7 downto 4) * c_eq, 13);
            s_inp(4) <= p3_in;
            s_tc4 <= r_tile_q(2 downto 0);           -- tile ROM (address at 2, EBR out at 3, fabric 4)

            -- 5: etch
            v_e := resize(p4_ea, 17) + shift_left(resize(p4_eb, 17), 4);
            p5_e <= v_e(16 downto 7);
            p5_w0 <= p4_w0;
            s_tcp(5) <= s_tc4;

            -- 6: w
            p6_w <= resize(p5_w0, 14) + resize(p5_e, 14);

            -- 7: ring phase base, envelope distance
            p7_pm <= signed(resize(p6_w, 15)) - 167;
            p7_d  <= signed(resize(c_we2, 15)) - signed(resize(p6_w, 15));

            -- 8: ring product row partials (kg = 3 + 3 + 4 bits); envelope D - D/16, D/32
            p8_p0 <= resize(p7_pm * signed('0' & c_kg(2 downto 0)), 19);
            p8_p1 <= resize(p7_pm * signed('0' & c_kg(5 downto 3)), 19);
            p8_p2 <= resize(p7_pm * signed('0' & c_kg(9 downto 6)), 20);
            p8_da <= p7_d - shift_right(p7_d, 4);
            p8_d5 <= shift_right(p7_d, 5);

            -- 9: product halves, the flow folded into the high one (all mod 2^18, Q8 index units)
            p9_a <= resize(p8_p0, 20) + shift_left(resize(p8_p1, 20), 3);
            p9_b <= shift_left(p8_p2, 6) - signed(resize(s_th & "00", 20));
            p9_eb <= p8_da - p8_d5;

            -- 10: ring ROM address = (product - flow) >> 8, mod 1024; envelope clamp
            v_v := resize(p9_a + p9_b, 20);
            r_ring_a <= unsigned(v_v(17 downto 8));
            v_eb := resize(p9_eb, 16) + 224;
            if v_eb < 0 then p10_e <= (others => '0');
            elsif v_eb > 255 then p10_e <= (others => '1');
            else p10_e <= unsigned(v_eb(7 downto 0)); end if;

            -- 11: ring EBR out; 12: fabric copy
            p11_e <= p10_e;
            p12_r <= r_ring_q;
            p12_e <= p11_e;

            -- 13: level = min(ring, envelope)
            if p12_r < p12_e then p13_l <= p12_r; else p13_l <= p12_e; end if;

            -- 14: palette address (stripes flip the MSB; Steps selects the bank)
            v_l := p13_l;
            if l_flip = '1' then v_l(7) := not v_l(7); end if;
            r_pal_a <= s_smooth & v_l;

            -- 15: palette EBR out; 16: fabric copy
            p16_q <= r_pal_q;

            -- 17: compose: plaid inside the medallion
            if s_inp(16) = '1' then
                s_y17 <= unsigned(C_MD_TCOL(to_integer(s_tcp(16)))(31 downto 22));
                s_u17 <= unsigned(C_MD_TCOL(to_integer(s_tcp(16)))(21 downto 12));
                s_v17 <= unsigned(C_MD_TCOL(to_integer(s_tcp(16)))(11 downto 2));
            else
                s_y17 <= unsigned(p16_q(31 downto 22));
                s_u17 <= unsigned(p16_q(21 downto 12));
                s_v17 <= unsigned(p16_q(11 downto 2));
            end if;

            for i in 5 to 16 loop s_inp(i) <= s_inp(i - 1); end loop;
            for i in 6 to 16 loop s_tcp(i) <= s_tcp(i - 1); end loop;
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning (chroma and luma [1 2 2 2 2 2 2 2 1]/16 as two
    -- adjacent 8-sums, centre at stage 22 -> 23), luma 13/16 and the blanking
    -- gate at stage 24
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_csy(0) <= s_y17; s_csu(0) <= s_u17; s_csv(0) <= s_v17;
            for i in 1 to 8 loop s_csy(i) <= s_csy(i - 1); s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxy <= s_boxy + s_y17 - s_csy(7);
            s_boxu <= s_boxu + s_u17 - s_csu(7);
            s_boxv <= s_boxv + s_v17 - s_csv(7);
            s_boxyd <= s_boxy; s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxy, 15) + s_boxyd; s_y23 <= v_sc(13 downto 4);
            v_sc := resize(s_boxu, 15) + s_boxud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v23 <= v_sc(13 downto 4);
            if s_y23 > 64 then v_dl := s_y23 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(22) = '1' then
                s_y24 <= s_y23 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u24 <= s_u23; s_v24 <= s_v23;
            else
                s_y24 <= to_unsigned(64, 10); s_u24 <= to_unsigned(512, 10); s_v24 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture medallion;
