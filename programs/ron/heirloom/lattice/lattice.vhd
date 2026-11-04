-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Lattice v0.1.0 -- Heirloom phase 3 (video clips), from the still WIN_20230401_00_20_03 at 0.5 s.
--
-- The still: a lattice of blue diamonds on lime green, big ones (x 1008, y 257 + 525 k) and small ones half a cell
-- away, each ringed yellow / orange / dark; a faint relief trellis (a bright edge and a dark crease) running
-- diagonally between them; and magenta blades on the trellis: X crosses at the saddles between the diamonds, their
-- arms tapering to points, coloured from scan entry (an orange / dark fringe) through magenta and violet to a
-- white-pink exit.  In the clip the knob Ron turned was the blades' own vertical oscillator: as its rate rises the
-- blade windows crowd toward the top and the X crosses break into spindles sliding along the trellis.
--
-- The patch, as fitted (lattice.py has every constant):
--   X(x)  = shaped triangle (period 944 virtual px, max at x 1008), held at its x = 66 value left of x = 66
--   Y(y)  = 1.0095 (tri + 0.4645 ramp): the V triangle (period 524.8 lines) mixed with its own ramp; Z sag at the bottom
--   s     = X + Y; a = |s| + size offset of its side; fringe index jj = ((tf - 256 - a) * gw >> 8) + 64:
--           jj < 0 fill (blue, or the camera / Osc), 0..127 fringe, then rings, else the ground
--   q     = (Tx + Ty) * sign(dX/dx): the pure-triangle trellis; ground shade by q; blade = |q + 42| < hq(line),
--           hq = a parabola over the phase of the blades' vertical oscillator, colour index (q + 42 + hq) * 32 / hq
--
--   K1  Big        big diamonds' size (none .. the photo .. merged)
--   K2  Small      small diamonds' size
--   K3  Columns    diamond columns across: x1/3 .. x3 (the H oscillator's rate, about the screen centre)
--   K4  Drift      bipolar: the blades climb (right) or fall (left) along the trellis; 620 = the default drift
--   K5  Spindles   the blade oscillator's rate against the rows: x1/3 .. x3 (512 = locked, the photo)
--   K6  Rings      0 = hard edges .. 512 = the photo's fringe .. 1023 = four rainbow rings round every diamond
--   S7  Shape      Diamond / Bulb (both waveforms rounded: lenses)
--   S8  Cross      Single / Double (a second blade window half a cycle away)
--   S9  Fill       Flat (the photo's blue, lit by the camera) / Gem (graded toward a pale core)
--   S10 Palette    Lime (the photo) / Dusk (amber ground, navy diamonds, cyan blades)
--   S11 Source     Video / Osc (the camera, or a drifting triangle lattice, lights the diamonds' fill)
--   P12 BLADES     blade width: 0 = the bare lattice .. 380 = the photo .. 1023 = a woven magenta trellis
--
-- Engine: vblank-zone sequencer (one serial multiplier, shared glide, exp2 ROM), a per-line engine kicked at each
-- line start that computes the NEXT line (Y, the trellis Ty, the blade half-width hq and its reciprocal), a 15-stage
-- pixel path ending in one palette ROM lookup.  Virtual 1080-line coordinates (SD shows the centre 4:3).  Generated
-- avid; output conditioning (Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes with the per-line resync, luma
-- 13/16) before the blanking gate.  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.lattice_pkg.all;
use work.all;

architecture lattice of program_top is
    constant C_LAT : integer := 20;   -- = processing_delay_clks

    -- fitted constants (lattice.py CONSTS)
    constant C_A24   : integer := 15923230;   -- H phase at the screen centre (Q24)
    constant C_STEPV : integer := 17768;      -- H phase per virtual px at x1 (Q24)
    constant C_KY    : integer := 31969;      -- V phase per virtual line (Q24)
    constant C_PHY0  : integer := 8545555;    -- V phase at line 0
    constant C_DR    : integer := 4875539;    -- ramp reset offset (Q24)
    constant C_SOFT0 : integer := 950;        -- ramp softening starts here (of 1024)
    constant C_PHI0  : integer := 2007639;    -- blade oscillator phase at line 0 (power-up)
    constant C_TFB   : integer := 1403;       -- big-side fill edge - 256 (1/1024)
    constant C_TFS   : integer := 1649;       -- small-side fill edge - 256

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

    -- line buffer: (luma >> 2) - 16 clipped at 0, or the Osc (0..63)
    type t_lb is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal lb : t_lb := (others => x"00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(7 downto 0) := x"00";
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(7 downto 0) := x"00";
    signal s_wcol  : unsigned(10 downto 0) := (others => '0');

    -- ROMs (registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_xa   : unsigned(9 downto 0) := (others => '0');
    signal r_xq   : signed(11 downto 0) := (others => '0');
    signal r_pa   : unsigned(9 downto 0) := (others => '0');
    signal r_pq   : std_logic_vector(31 downto 0) := (others => '0');

    -- oscillator source
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

    -- knobs: glided (Q3), raw drift, switches
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g2 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g3 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g5 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_g6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(380 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(620, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_shape, s_cross, s_gem, s_pal, s_src : std_logic := '0';

    -- per-field constants
    signal z_k    : unsigned(9 downto 0) := (others => '0');
    signal z_t1, z_t2 : unsigned(17 downto 0) := (others => '0');
    signal z_e    : signed(11 downto 0) := (others => '0');
    signal z_sh6  : unsigned(2 downto 0) := (others => '0');
    signal z_m3, z_m5 : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_sh3, z_sh5 : signed(2 downto 0) := (others => '0');
    signal z_v2   : unsigned(17 downto 0) := (others => '0');
    signal c_hstepv : unsigned(17 downto 0) := to_unsigned(C_STEPV, 18);
    signal c_step  : unsigned(18 downto 0) := to_unsigned(C_STEPV, 19);
    signal c_phs   : unsigned(23 downto 0) := (others => '0');
    signal c_nhold : unsigned(6 downto 0) := to_unsigned(66, 7);
    signal c_dmul  : unsigned(9 downto 0) := to_unsigned(894, 10);
    signal c_k2v   : unsigned(17 downto 0) := to_unsigned(C_KY, 18);
    signal c_k2l   : unsigned(19 downto 0) := to_unsigned(C_KY, 20);
    signal c_kyl   : unsigned(19 downto 0) := to_unsigned(C_KY, 20);
    signal c_dph   : unsigned(23 downto 0) := to_unsigned(C_PHI0, 24);
    signal c_dstep : unsigned(23 downto 0) := (others => '0');
    signal c_offb, c_offs : signed(11 downto 0) := (others => '0');
    signal c_gw    : unsigned(10 downto 0) := to_unsigned(64, 11);
    signal c_lr128 : unsigned(9 downto 0) := to_unsigned(128, 10);
    signal c_gh    : unsigned(9 downto 0) := to_unsigned(276, 10);
    signal c_w2    : unsigned(11 downto 0) := to_unsigned(3044, 12);

    -- field serial multiplier
    signal f_a   : unsigned(35 downto 0) := (others => '0');
    signal f_b   : unsigned(15 downto 0) := (others => '0');
    signal f_acc : unsigned(35 downto 0) := (others => '0');
    signal z_fa  : unsigned(35 downto 0) := (others => '0');
    signal z_fb  : unsigned(15 downto 0) := (others => '0');
    signal z_sel : unsigned(2 downto 0) := (others => '0');
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

    -- line engine (computes the NEXT line)
    signal e_lc   : unsigned(8 downto 0) := to_unsigned(511, 9);
    signal e_zk   : std_logic := '0';
    signal e_vy   : unsigned(18 downto 0) := (others => '0');
    signal e_phy, e_phi : unsigned(23 downto 0) := (others => '0');
    signal e_p10, e_rp, e_f10 : unsigned(9 downto 0) := (others => '0');
    signal e_ph2  : unsigned(23 downto 0) := (others => '0');
    signal e_vyi  : unsigned(10 downto 0) := (others => '0');
    signal e_m    : unsigned(9 downto 0) := (others => '0');
    signal e_dd1, e_dd2 : unsigned(9 downto 0) := (others => '0');
    signal e_d1, e_d2 : unsigned(9 downto 0) := (others => '0');
    signal e_dz   : unsigned(8 downto 0) := (others => '0');
    signal e_tv, e_saw : signed(11 downto 0) := (others => '0');
    signal e_ya, e_yb, e_yd, e_yb2, e_ybl : signed(12 downto 0) := (others => '0');
    signal e_c    : signed(12 downto 0) := (others => '0');
    signal e_m2, e_m3 : unsigned(9 downto 0) := (others => '0');
    signal e_z    : unsigned(7 downto 0) := (others => '0');
    signal e_h    : unsigned(10 downto 0) := (others => '0');
    signal e_hq1, e_hq2, e_hq : unsigned(9 downto 0) := (others => '0');
    signal e_rem  : unsigned(10 downto 0) := (others => '0');
    signal e_quo  : unsigned(20 downto 0) := (others => '0');
    signal e_sh   : unsigned(3 downto 0) := (others => '0');
    signal m_a    : unsigned(35 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(35 downto 0) := (others => '0');
    signal n_y    : signed(12 downto 0) := (others => '0');
    signal n_ty   : signed(11 downto 0) := (others => '0');
    signal n_hq   : unsigned(9 downto 0) := (others => '0');
    signal n_r8   : unsigned(7 downto 0) := (others => '0');
    signal n_sh   : unsigned(3 downto 0) := (others => '0');
    signal l_y    : signed(12 downto 0) := (others => '0');
    signal l_ty   : signed(11 downto 0) := (others => '0');
    signal l_hq   : unsigned(9 downto 0) := (others => '0');
    signal l_h2   : unsigned(10 downto 0) := (others => '0');
    signal l_r8   : unsigned(7 downto 0) := (others => '0');
    signal l_sh   : unsigned(3 downto 0) := (others => '0');

    -- beam (stage 1)
    signal b_ph   : unsigned(23 downto 0) := (others => '0');
    signal b_hold : unsigned(6 downto 0) := (others => '0');
    signal b_px   : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    signal q2_u9  : unsigned(8 downto 0) := (others => '0');
    signal q2_sgn, q3_sgn : std_logic := '0';
    signal q2_tx  : signed(11 downto 0) := (others => '0');
    signal q3_sig : signed(12 downto 0) := (others => '0');
    signal q4_x   : signed(11 downto 0) := (others => '0');
    signal q4_q   : signed(12 downto 0) := (others => '0');
    signal q4_lc  : unsigned(5 downto 0) := (others => '0');
    signal q5_s   : signed(12 downto 0) := (others => '0');
    signal q5_e   : signed(13 downto 0) := (others => '0');
    signal q5_ri  : unsigned(5 downto 0) := (others => '0');
    signal q5_lc  : unsigned(5 downto 0) := (others => '0');
    signal q6_side : std_logic := '0';
    signal q6_abs : unsigned(11 downto 0) := (others => '0');
    signal q6_eh  : signed(13 downto 0) := (others => '0');
    signal q6_ri, q6_lc : unsigned(5 downto 0) := (others => '0');
    signal q7_a   : signed(13 downto 0) := (others => '0');
    signal q7_side, q7_bl : std_logic := '0';
    signal q7_pa, q7_pb : unsigned(14 downto 0) := (others => '0');
    signal q7_ri, q7_lc : unsigned(5 downto 0) := (others => '0');
    signal q8_dm  : signed(13 downto 0) := (others => '0');
    signal q8_bs  : unsigned(18 downto 0) := (others => '0');
    signal q8_side, q8_bl : std_logic := '0';
    signal q8_ri, q8_lc : unsigned(5 downto 0) := (others => '0');
    signal q9_ma  : signed(19 downto 0) := (others => '0');
    signal q9_mb  : signed(20 downto 0) := (others => '0');
    signal q9_bi  : unsigned(5 downto 0) := (others => '0');
    signal q9_dm  : signed(13 downto 0) := (others => '0');
    signal q9_side, q9_bl : std_logic := '0';
    signal q9_ri, q9_lc : unsigned(5 downto 0) := (others => '0');
    signal q10_jj : signed(17 downto 0) := (others => '0');
    signal q10_gi : unsigned(5 downto 0) := (others => '0');
    signal q10_bi, q10_ri, q10_lc : unsigned(5 downto 0) := (others => '0');
    signal q10_side, q10_bl : std_logic := '0';
    signal p14_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal s_y15 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_cay, s_cby : t_u10(0 to 7) := (others => to_unsigned(64, 10));
    signal s_cau, s_cav, s_cbu, s_cbv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_bay, s_bayd, s_bby, s_bbyd : unsigned(12 downto 0) := to_unsigned(512, 13);
    signal s_bau, s_bav, s_baud, s_bavd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bbu, s_bbv, s_bbud, s_bbvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_bah, s_bbh : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y17 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y21 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u21, s_v21 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
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
            r_ex_q <= C_LT_EXP2(to_integer(r_ex_a));
            r_xq   <= C_LT_XROM(to_integer(r_xa));
            r_pq   <= C_LT_PAL(to_integer(r_pa));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field sequencer (one small operation per zone
    -- clock; the zone is quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_p  : unsigned(9 downto 0);
        variable v_t  : unsigned(17 downto 0);
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
            -- drift: bipolar, +-8 deadband, |v| - 8
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
                s_shape <= registers_in(6)(0);
                s_cross <= registers_in(6)(1);
                s_gem   <= registers_in(6)(2);
                s_pal   <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); c_nhold <= to_unsigned(0, 7);  c_dmul <= to_unsigned(720, 10);
                    when 3      => s_oxs <= to_unsigned(153, 9); c_nhold <= to_unsigned(44, 7); c_dmul <= to_unsigned(894, 10);
                    when others => s_oxs <= to_unsigned(102, 9); c_nhold <= to_unsigned(66, 7); c_dmul <= to_unsigned(894, 10);
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

            -- serial products: slot k loads at zc = 64 + 24 k, 16 iterations, result stable from load + 17
            -- (stores at load + 18).  The slot is decoded at 62 + 24 k and the operands pre-muxed at 63 + 24 k.
            z_pre <= '0';
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 62  => z_sel <= to_unsigned(0, 3); z_pre <= '1';
                    when 86  => z_sel <= to_unsigned(1, 3); z_pre <= '1';
                    when 110 => z_sel <= to_unsigned(2, 3); z_pre <= '1';
                    when 134 => z_sel <= to_unsigned(3, 3); z_pre <= '1';
                    when 158 => z_sel <= to_unsigned(4, 3); z_pre <= '1';
                    when 182 => z_sel <= to_unsigned(5, 3); z_pre <= '1';
                    when 206 => z_sel <= to_unsigned(6, 3); z_pre <= '1';
                    when 230 => z_sel <= to_unsigned(7, 3); z_pre <= '1';
                    when others => null;
                end case;
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                case to_integer(z_sel) is
                    when 0      => z_fa <= to_unsigned(C_STEPV, 36);         z_fb <= z_m3;                     -- H step
                    when 1      => z_fa <= resize(c_hstepv, 36);              z_fb <= resize(c_dmul, 16);       -- start phase
                    when 2      => z_fa <= to_unsigned(C_KY, 36);            z_fb <= z_m5;                     -- blade rate
                    when 3      => z_fa <= resize(c_k2v, 36);                z_fb <= resize(s_ys, 16);         -- per line
                    when 4      => z_fa <= to_unsigned(C_KY, 36);            z_fb <= resize(s_ys, 16);         -- rows per line
                    when 5      => z_fa <= resize(s_vm, 36);                 z_fb <= resize(s_vm, 16);         -- drift^2
                    when 6      => z_fa <= resize(z_v2, 36);                 z_fb <= to_unsigned(566, 16);
                    when others => z_fa <= resize(s_gp(12 downto 3), 36);    z_fb <= to_unsigned(745, 16);     -- blade gain
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
                    -- exponents: e = (k - 512) * 202 >> 8 (Q8 octaves, x1/3 .. x3), two clocks of shift-adds each
                    when 26 => z_k <= s_g3(12 downto 3);
                    when 27 => z_t1 <= shift_left(resize(z_k, 18), 7) + shift_left(resize(z_k, 18), 6);
                               z_t2 <= shift_left(resize(z_k, 18), 3) + shift_left(resize(z_k, 18), 1);
                    when 28 => z_e <= resize(shift_right(signed(resize(z_t1 + z_t2, 20)) - 103424, 8), 12);
                    when 29 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh3 <= z_e(10 downto 8);
                    when 31 => z_m3 <= r_ex_q;
                    when 32 => z_k <= s_g5(12 downto 3);
                    when 33 => z_t1 <= shift_left(resize(z_k, 18), 7) + shift_left(resize(z_k, 18), 6);
                               z_t2 <= shift_left(resize(z_k, 18), 3) + shift_left(resize(z_k, 18), 1);
                    when 34 => z_e <= resize(shift_right(signed(resize(z_t1 + z_t2, 20)) - 103424, 8), 12);
                    when 35 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh5 <= z_e(10 downto 8);
                    when 37 => z_m5 <= r_ex_q;
                    -- Rings: below 512, gw = 64 * 2^((512 - k) / 128) (hard edges at 0); above, the ring count
                    when 38 => v_p := s_g6(12 downto 3);
                               if v_p < 512 then
                                   z_e <= signed(resize(shift_left(resize(512 - v_p, 11), 1), 12));
                                   c_lr128 <= to_unsigned(128, 10);
                               else
                                   z_e <= (others => '0');
                                   c_lr128 <= resize(v_p - 384, 10);   -- 128 + (k - 512)
                               end if;
                    when 39 => r_ex_a <= unsigned(z_e(7 downto 0)); z_sh6 <= unsigned(z_e(10 downto 8));
                    when 41 =>
                        case to_integer(z_sh6) is
                            when 0      => c_gw <= resize(r_ex_q(15 downto 9), 11);
                            when 1      => c_gw <= resize(r_ex_q(15 downto 8), 11);
                            when 2      => c_gw <= resize(r_ex_q(15 downto 7), 11);
                            when 3      => c_gw <= resize(r_ex_q(15 downto 6), 11);
                            when others => c_gw <= to_unsigned(1024, 11);
                        end case;
                    -- sizes, blade window width
                    when 42 =>
                        c_offb <= shift_left(signed(resize(s_g1(12 downto 3), 12)) - 512, 1);
                        c_offs <= shift_left(signed(resize(s_g2(12 downto 3), 12)) - 512, 1);
                        v_p := s_gp(12 downto 3);
                        if v_p <= 380 then
                            c_w2 <= to_unsigned(3044, 12);
                        else
                            c_w2 <= to_unsigned(3044, 12) - shift_left(resize(v_p - 380, 12), 2) - resize(shift_right(v_p - 380, 5), 12);
                        end if;
                    -- products
                    when 82  =>                       -- stepv = 17768 * m3 >> (15 - sh3), sh3 = -2 .. 1
                        case to_integer(z_sh3) is
                            when -2     => c_hstepv <= resize(f_acc(35 downto 17), 18);
                            when -1     => c_hstepv <= resize(f_acc(35 downto 16), 18);
                            when 0      => c_hstepv <= resize(f_acc(35 downto 15), 18);
                            when others => c_hstepv <= resize(f_acc(35 downto 14), 18);
                        end case;
                    when 83  =>
                        case to_integer(s_xs) is
                            when 4      => c_step <= shift_left(resize(c_hstepv, 19), 1);
                            when 3      => c_step <= resize(c_hstepv, 19) + resize(shift_right(c_hstepv, 1), 19);
                            when others => c_step <= resize(c_hstepv, 19);
                        end case;
                    when 106 => c_phs <= to_unsigned(C_A24, 24) - f_acc(23 downto 0);
                    when 130 =>
                        case to_integer(z_sh5) is
                            when -2     => c_k2v <= resize(f_acc(35 downto 17), 18);
                            when -1     => c_k2v <= resize(f_acc(35 downto 16), 18);
                            when 0      => c_k2v <= resize(f_acc(35 downto 15), 18);
                            when others => c_k2v <= resize(f_acc(35 downto 14), 18);
                        end case;
                    when 154 => c_k2l <= f_acc(27 downto 8);
                    when 178 => c_kyl <= f_acc(27 downto 8);
                    when 202 => z_v2 <= f_acc(17 downto 0);
                    when 226 => c_dstep <= resize(f_acc(35 downto 8), 24);
                    when 227 =>                       -- the blades' drift (climb for Drift > 512)
                        if s_vneg = '1' then c_dph <= c_dph - c_dstep; else c_dph <= c_dph + c_dstep; end if;
                    when 250 => c_gh <= f_acc(19 downto 10);
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
    -- Source side: oscillator (2-D triangle lattice, drifting), line-buffer
    -- write of (luma >> 2) - 16, or the oscillator (0..63)
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
            s_osum <= shift_right(v_s, 2);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then s_lb_wd <= std_logic_vector(s_osum);
            else
                v_l := unsigned(data_in.y(9 downto 2));
                if v_l > 16 then s_lb_wd <= std_logic_vector(v_l - 16); else s_lb_wd <= (others => '0'); end if;
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Line engine: kicked at each line start (and once in the zone for line 0),
    -- computes the NEXT line: Y (triangle + ramp, or the bulb), the bottom sag,
    -- the trellis Ty, the blade half-width hq (one or two windows) and its
    -- reciprocal (8-bit mantissa r8 and shift sh: 32 / hq = r8 / 2^sh).  One
    -- serial multiplier (16 iterations, result at load + 17), one restoring divider.
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_d : unsigned(9 downto 0);
        variable v_r : unsigned(10 downto 0);
        variable v_t : signed(12 downto 0);
        variable v_s : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            e_zk <= '0';
            if s_zone = '1' and s_zc = 300 then e_zk <= '1'; end if;

            if e_zk = '1' then
                e_vy  <= (others => '0');
                e_phy <= to_unsigned(C_PHY0, 24);
                e_phi <= c_dph;
                e_lc  <= (others => '0');
            elsif g_start = '1' then
                e_vy  <= e_vy + resize(s_ys, 19);
                e_phy <= e_phy + resize(c_kyl, 24);
                e_phi <= e_phi + resize(c_k2l, 24);
                e_lc  <= (others => '0');
            elsif e_lc /= 511 then
                e_lc <= e_lc + 1;
            end if;

            if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
            m_a <= m_a(34 downto 0) & '0';
            m_b <= '0' & m_b(15 downto 1);

            case to_integer(e_lc) is
                when 1 =>
                    e_p10 <= e_phy(23 downto 14);
                    e_ph2 <= e_phy - to_unsigned(C_DR, 24);
                    e_f10 <= e_phi(23 downto 14);
                    e_vyi <= e_vy(18 downto 8);
                when 2 =>
                    e_rp <= e_ph2(23 downto 14);
                    if e_p10(9) = '1' then e_m <= resize(1024 - resize(e_p10, 11), 10); else e_m <= e_p10; end if;
                    e_dd1 <= e_f10 - 271;
                    e_dd2 <= e_f10 + 241;                     -- + 512 - 271
                    if e_vyi > 600 then e_dz <= resize(e_vyi - 600, 9); else e_dz <= (others => '0'); end if;
                when 3 =>
                    e_tv <= resize(to_signed(1024, 13) - signed(resize(shift_left(resize(e_m, 13), 2), 13)), 12);
                    if e_rp >= C_SOFT0 then
                        e_saw <= resize(to_signed(1024, 13) - signed(resize(shift_left(resize(e_rp - C_SOFT0, 13), 5) - shift_left(resize(e_rp - C_SOFT0, 13), 2), 13)), 12);
                    else
                        e_saw <= resize(signed(resize(shift_left(resize(e_rp, 13), 1), 13)) - 1024, 12);
                    end if;
                    if e_dd1(9) = '1' then e_d1 <= resize(1024 - resize(e_dd1, 11), 10); else e_d1 <= e_dd1; end if;
                    if e_dd2(9) = '1' then e_d2 <= resize(1024 - resize(e_dd2, 11), 10); else e_d2 <= e_dd2; end if;
                when 4 =>
                    e_ya <= resize(e_tv, 13) + resize(shift_right(e_tv, 7), 13) + resize(shift_right(e_tv, 9), 13);
                    e_yb <= resize(shift_right(e_saw, 1), 13) - resize(shift_right(e_saw, 5), 13);
                when 5 =>
                    e_yd <= e_ya + e_yb;
                -- serial products
                when 8   => m_a <= resize(e_m, 36);  m_b <= resize(e_m, 16);  m_acc <= (others => '0');
                when 26  => e_m2 <= resize(m_acc(19 downto 9), 10);
                when 28  => m_a <= resize(e_m2, 36); m_b <= resize(e_m, 16);  m_acc <= (others => '0');
                when 46  => e_m3 <= resize(m_acc(19 downto 9), 10);
                when 48  => m_a <= resize(e_dz, 36); m_b <= resize(e_dz, 16); m_acc <= (others => '0');
                when 66  => m_a <= m_acc;            m_b <= to_unsigned(47, 16); m_acc <= (others => '0');
                when 85  => e_z <= m_acc(23 downto 16);
                when 88  => m_a <= resize(e_d1, 36); m_b <= resize(e_d1, 16); m_acc <= (others => '0');
                when 106 => m_a <= m_acc;            m_b <= resize(c_w2, 16); m_acc <= (others => '0');
                when 125 =>
                    if m_acc(35 downto 18) >= 1024 then e_h <= (others => '0');
                    else e_h <= resize(1024 - resize(m_acc(27 downto 18), 11), 11); end if;
                when 128 => m_a <= resize(e_h, 36);  m_b <= resize(c_gh, 16); m_acc <= (others => '0');
                when 146 => e_hq1 <= m_acc(19 downto 10);
                when 148 => m_a <= resize(e_d2, 36); m_b <= resize(e_d2, 16); m_acc <= (others => '0');
                when 166 => m_a <= m_acc;            m_b <= resize(c_w2, 16); m_acc <= (others => '0');
                when 185 =>
                    if m_acc(35 downto 18) >= 1024 then e_h <= (others => '0');
                    else e_h <= resize(1024 - resize(m_acc(27 downto 18), 11), 11); end if;
                when 188 => m_a <= resize(e_h, 36);  m_b <= resize(c_gh, 16); m_acc <= (others => '0');
                when 206 => e_hq2 <= m_acc(19 downto 10);
                -- bulb: c = 1024 - 4 (3 m2 - 2 m3), x 1.1602; choose Y, less the sag
                when 207 =>
                    e_c <= resize(to_signed(1024, 15) - signed(resize(shift_left(resize(e_m2, 15), 3) + shift_left(resize(e_m2, 15), 2), 15))
                                                      + signed(resize(shift_left(resize(e_m3, 15), 3), 15)), 13);
                    if s_cross = '1' and e_hq2 > e_hq1 then e_hq <= e_hq2; else e_hq <= e_hq1; end if;
                when 208 =>
                    e_ybl <= e_c + shift_right(e_c, 3) + shift_right(e_c, 5) + shift_right(e_c, 8);
                    -- divider start: 2^20 / hq (21 quotient bits)
                    e_rem <= (others => '0'); e_quo <= (others => '0');
                when 209 =>
                    if s_shape = '1' then e_yb2 <= e_ybl - signed(resize(e_z, 13)); else e_yb2 <= e_yd - signed(resize(e_z, 13)); end if;
                when 210 to 230 =>               -- restoring division, dividend bit 20 - (lc - 210)
                    if e_lc = 210 then v_r := to_unsigned(1, 11); else v_r := e_rem(9 downto 0) & '0'; end if;
                    if v_r >= resize(e_hq, 11) and e_hq /= 0 then
                        e_rem <= v_r - resize(e_hq, 11); e_quo <= e_quo(19 downto 0) & '1';
                    else
                        e_rem <= v_r; e_quo <= e_quo(19 downto 0) & '0';
                    end if;
                    e_sh <= to_unsigned(15, 4);
                when 231 to 244 =>               -- normalise: quotient to [128, 255], sh = 15 - shifts
                    if e_quo(20 downto 8) /= 0 then
                        e_quo <= '0' & e_quo(20 downto 1); e_sh <= e_sh - 1;
                    end if;
                when 250 =>
                    n_y  <= e_yb2;
                    n_ty <= e_tv;
                    if e_hq < 4 then n_hq <= (others => '0'); n_r8 <= (others => '0');
                    else n_hq <= e_hq; n_r8 <= e_quo(7 downto 0); end if;
                    n_sh <= e_sh;
                when others => null;
            end case;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam (stage 1): H phase, held for the first c_nhold pixels (x < 66 virtual px)
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_ph   <= c_phs;
                b_hold <= c_nhold;
                b_px   <= (others => '0');
                l_y <= n_y; l_ty <= n_ty; l_hq <= n_hq; l_h2 <= n_hq & '0'; l_r8 <= n_r8; l_sh <= n_sh;
            elsif g_avid = '1' then
                b_px <= b_px + 1;
                if b_hold /= 0 then b_hold <= b_hold - 1; else b_ph <= b_ph + resize(c_step, 24); end if;
            end if;
            s_lb_ra <= b_px;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path (stage n registers from stage n - 1 values)
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_p10 : unsigned(9 downto 0);
        variable v_lb  : unsigned(7 downto 0);
        variable v_lc  : unsigned(6 downto 0);
        variable v_r   : signed(12 downto 0);
        variable v_jj  : signed(24 downto 0);
        variable v_g   : unsigned(13 downto 0);
        variable v_reg : unsigned(2 downto 0);
        variable v_idx : unsigned(5 downto 0);
        variable v_ab  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: phase -> u9 (0 at the big column), slope sign, X ROM address, Tx
            v_p10 := b_ph(23 downto 14);
            if v_p10(9) = '1' then q2_u9 <= not v_p10(8 downto 0); else q2_u9 <= v_p10(8 downto 0); end if;
            q2_sgn <= v_p10(9);
            if v_p10(9) = '1' then
                r_xa <= s_shape & not v_p10(8 downto 0);
                q2_tx <= to_signed(1024, 12) - signed(resize(shift_left(resize(not v_p10(8 downto 0), 12), 2), 12));
            else
                r_xa <= s_shape & v_p10(8 downto 0);
                q2_tx <= to_signed(1024, 12) - signed(resize(shift_left(resize(v_p10(8 downto 0), 12), 2), 12));
            end if;
            -- 3: trellis sum (X ROM reads)
            q3_sig <= resize(q2_tx, 13) + resize(l_ty, 13);
            q3_sgn <= q2_sgn;
            -- 4: X (fabric copy), signed trellis q, camera luma -> fill index
            q4_x <= r_xq;
            if q3_sgn = '1' then q4_q <= q3_sig; else q4_q <= -q3_sig; end if;
            v_lb := unsigned(s_lb_q);
            v_lc := resize(shift_right(v_lb, 2), 7) + resize(shift_right(v_lb, 5), 7);
            if v_lc > 63 then q4_lc <= (others => '1'); else q4_lc <= v_lc(5 downto 0); end if;
            -- 5: s = X + Y, blade offset, ground index
            q5_s <= resize(q4_x, 13) + l_y;
            q5_e <= resize(q4_q, 14) + 42;
            v_r := shift_right(q4_q, 4) + 32;
            if v_r < 0 then q5_ri <= (others => '0');
            elsif v_r > 63 then q5_ri <= (others => '1');
            else q5_ri <= unsigned(v_r(5 downto 0)); end if;
            q5_lc <= q4_lc;
            -- 6: side, |s|, blade position e + hq
            q6_side <= not q5_s(12);
            v_ab := abs(q5_s);
            q6_abs <= unsigned(v_ab(11 downto 0));
            q6_eh <= q5_e + signed(resize(l_hq, 14));
            q6_ri <= q5_ri; q6_lc <= q5_lc;
            -- 7: a = |s| + size offset of the side; in-blade test; blade index partials
            if q6_side = '1' then q7_a <= signed(resize(q6_abs, 14)) + resize(c_offb, 14);
            else q7_a <= signed(resize(q6_abs, 14)) + resize(c_offs, 14); end if;
            if q6_eh >= 0 and q6_eh < signed(resize(l_h2, 14)) then q7_bl <= '1'; else q7_bl <= '0'; end if;
            q7_pa <= resize(unsigned(q6_eh(10 downto 0)) * l_r8(3 downto 0), 15);
            q7_pb <= resize(unsigned(q6_eh(10 downto 0)) * l_r8(7 downto 4), 15);
            q7_side <= q6_side; q7_ri <= q6_ri; q7_lc <= q6_lc;
            -- 8: distance to the fill edge (pre-biased by 256); blade product
            if q7_side = '1' then q8_dm <= to_signed(C_TFB, 14) - q7_a; else q8_dm <= to_signed(C_TFS, 14) - q7_a; end if;
            q8_bs <= resize(q7_pa, 19) + shift_left(resize(q7_pb, 19), 4);
            q8_side <= q7_side; q8_bl <= q7_bl; q8_ri <= q7_ri; q8_lc <= q7_lc;
            -- 9: fringe gain partials; blade index = bs >> sh (saturating)
            q9_ma <= resize(q8_dm * signed('0' & c_gw(4 downto 0)), 20);
            q9_mb <= resize(q8_dm * signed('0' & c_gw(10 downto 5)), 21);
            case to_integer(l_sh) is
                when 4      => if q8_bs(18 downto 10) /= 0 then q9_bi <= (others => '1'); else q9_bi <= q8_bs(9 downto 4); end if;
                when 5      => if q8_bs(18 downto 11) /= 0 then q9_bi <= (others => '1'); else q9_bi <= q8_bs(10 downto 5); end if;
                when 6      => if q8_bs(18 downto 12) /= 0 then q9_bi <= (others => '1'); else q9_bi <= q8_bs(11 downto 6); end if;
                when 7      => if q8_bs(18 downto 13) /= 0 then q9_bi <= (others => '1'); else q9_bi <= q8_bs(12 downto 7); end if;
                when 8      => if q8_bs(18 downto 14) /= 0 then q9_bi <= (others => '1'); else q9_bi <= q8_bs(13 downto 8); end if;
                when 9      => if q8_bs(18 downto 15) /= 0 then q9_bi <= (others => '1'); else q9_bi <= q8_bs(14 downto 9); end if;
                when 10     => if q8_bs(18 downto 16) /= 0 then q9_bi <= (others => '1'); else q9_bi <= q8_bs(15 downto 10); end if;
                when 11     => if q8_bs(18 downto 17) /= 0 then q9_bi <= (others => '1'); else q9_bi <= q8_bs(16 downto 11); end if;
                when others => if q8_bs(18) /= '0' then q9_bi <= (others => '1'); else q9_bi <= q8_bs(17 downto 12); end if;
            end case;
            q9_dm <= q8_dm; q9_side <= q8_side; q9_bl <= q8_bl; q9_ri <= q8_ri; q9_lc <= q8_lc;
            -- 10: fringe index jj = (dm * gw >> 8) + 64; gem index
            v_jj := resize(q9_ma, 25) + shift_left(resize(q9_mb, 25), 5);
            q10_jj <= resize(shift_right(v_jj, 8), 18) + 64;
            if q9_dm < -256 then
                v_g := resize(shift_right(unsigned(-(q9_dm + 256)), 3), 14) + resize(shift_right(q9_lc, 2), 14) + resize(shift_right(q9_lc, 3), 14);
            else
                v_g := resize(shift_right(q9_lc, 2), 14) + resize(shift_right(q9_lc, 3), 14);
            end if;
            if v_g > 63 then q10_gi <= (others => '1'); else q10_gi <= v_g(5 downto 0); end if;
            q10_bi <= q9_bi; q10_side <= q9_side; q10_bl <= q9_bl; q10_ri <= q9_ri; q10_lc <= q9_lc;
            -- 11: zone -> palette address: fill > fringe > blade > rings > ground
            if q10_jj < 0 then
                v_reg := "111";
                if s_gem = '1' then v_idx := q10_gi; else v_idx := q10_lc; end if;
            elsif q10_jj < 128 then
                v_reg(2) := '0'; v_reg(1) := not q10_side; v_reg(0) := q10_jj(6);
                v_idx := unsigned(q10_jj(5 downto 0));
            elsif q10_bl = '1' then
                v_reg := "101"; v_idx := q10_bi;
            elsif q10_jj < signed(resize(c_lr128, 18)) then
                v_reg := "100"; v_idx := unsigned(q10_jj(6 downto 1));
            else
                v_reg := "110"; v_idx := q10_ri;
            end if;
            r_pa <= s_pal & v_reg & v_idx;
            -- 12: address (EBR), 13: ROM data, 14: fabric copy
            p14_q <= r_pq;
            -- 15: compose; black outside the generated avid
            if s_gav_sr(12) = '0' then
                s_y15 <= to_unsigned(64, 10); s_u15 <= to_unsigned(512, 10); s_v15 <= to_unsigned(512, 10);
            else
                s_y15 <= unsigned(p14_q(31 downto 22)); s_u15 <= unsigned(p14_q(21 downto 12)); s_v15 <= unsigned(p14_q(11 downto 2));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(12);
        end if;
    end process p_pix;

    ---------------------------------------------------------------------------
    -- Output conditioning: Y, U and V through two [1 2 2 2 2 2 2 2 1]/16 boxes
    -- (the HD HDMI bar fix and the still's horizontal softness), luma 13/16 and
    -- the blanking gate.  The running 8-sums snap back to black once 8 blank
    -- samples are in (per-line resync).
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            s_cay(0) <= s_y15; s_cau(0) <= s_u15; s_cav(0) <= s_v15;
            for i in 1 to 7 loop s_cay(i) <= s_cay(i - 1); s_cau(i) <= s_cau(i - 1); s_cav(i) <= s_cav(i - 1); end loop;
            if s_bah = "00000000" then
                s_bay <= to_unsigned(512, 13); s_bau <= to_unsigned(4096, 13); s_bav <= to_unsigned(4096, 13);
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
                s_y17 <= to_unsigned(64, 10); s_u17 <= to_unsigned(512, 10); s_v17 <= to_unsigned(512, 10);
            end if;
            s_bbh <= s_bbh(6 downto 0) & s_gav_sr(14);
            s_cby(0) <= s_y17; s_cbu(0) <= s_u17; s_cbv(0) <= s_v17;
            for i in 1 to 7 loop s_cby(i) <= s_cby(i - 1); s_cbu(i) <= s_cbu(i - 1); s_cbv(i) <= s_cbv(i - 1); end loop;
            if s_bbh = "00000000" then
                s_bby <= to_unsigned(512, 13); s_bbu <= to_unsigned(4096, 13); s_bbv <= to_unsigned(4096, 13);
            else
                s_bby <= s_bby + s_y17 - s_cby(7);
                s_bbu <= s_bbu + s_u17 - s_cbu(7);
                s_bbv <= s_bbv + s_v17 - s_cbv(7);
            end if;
            s_bbyd <= s_bby; s_bbud <= s_bbu; s_bbvd <= s_bbv;
            v_sc := resize(s_bby, 14) + s_bbyd; s_y19 <= v_sc(13 downto 4);
            v_sc := resize(s_bbu, 14) + s_bbud; s_u19 <= v_sc(13 downto 4);
            v_sc := resize(s_bbv, 14) + s_bbvd; s_v19 <= v_sc(13 downto 4);
            if s_y19 > 64 then v_dl := s_y19 - 64; else v_dl := (others => '0'); end if;
            s_y20 <= s_y19 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u20 <= s_u19; s_v20 <= s_v19;
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

end architecture lattice;
