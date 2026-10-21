-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Radiant v0.1.0 -- Heirloom #52 (2026-10-20), from duci.jpg (a paper collage).
--
-- The collage: a maroon, scratched ground; pink / white / black stripe rays
-- fanning out from a point near the centre; a dark circular halo round a
-- cut-out (a cup of coffee for a head); figures either side.
--
-- The program: a sunburst of N rays through a centre point, the rays
-- alternately pink and white (ray boundaries are straight lines through the
-- centre, kept as per-line DDAs, so no angles are computed per pixel); a
-- dark halo ring round the centre; inside the ring the video (or the
-- oscillator) is cut in; the ground is a slowly moving maroon texture.
--
--   K1  Rays    number of rays, 2 .. 8
--   K2  Halo    halo radius
--   K3  Ring    halo ring width
--   K4  Ground  ground hue (maroon, navy, forest, ochre, plum, teal, rust, slate)
--   K5  Spin    ray rotation, bipolar (centre = still)
--   K6  Centre  vertical position of the centre, 25 .. 75 % of the height
--   S7  Rays    on / off
--   S8  Halo    Dark ring / Glowing ring
--   S9  Cutout  video inside the halo / outside it
--   S10 Palette Pink-white rays / Cyan-yellow rays
--   S11 Source  Video / Osc
--   P12 RAYS    ray width: hairlines .. the rays merge into a full disc
--
-- Virtual 1080-line geometry; the ray boundaries x_i = cx - tan(a_i)*(y - cy)
-- advance per line (16 DDAs, increments and bases from a tan ROM on the
-- serial multiplier in vblank).  dx^2 by second differences, dy^2 per
-- line.  Generated avid; output conditioning before the gate.  Latency 20.
-- EBR: 16 line buffer + 1 cot ROM.  Colours in the hardware convention
-- (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture radiant of program_top is
    constant C_LAT : integer := 20;
    constant C_NB : integer := 16;    -- ray boundaries (2 per ray, 8 rays)

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
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- tan ROM: angle a (0..255 = 0..360 deg from straight up, clockwise).
    -- A boundary line through the centre at angle a is x = cx - tan(a) (y - cy);
    -- the ROM holds |tan a| in Q.8 clamped to 16.0, the sign comes from the
    -- quadrant (tan > 0 in quadrants 1 and 3 -> coefficient negative).
    type t_cot is array (0 to 255) of unsigned(12 downto 0);
    function f_cot return t_cot is
        variable r : t_cot;
        variable s, c, t : real;
        variable v : integer;
    begin
        for i in 0 to 255 loop
            s := sin(real(i) * 6.283185307 / 256.0);
            c := cos(real(i) * 6.283185307 / 256.0);
            if abs(c) < 0.0625 then
                v := 4096;
            else
                v := integer(abs(s / c) * 256.0);
                if v > 4096 then v := 4096; end if;
            end if;
            r(i) := to_unsigned(v, 13);
        end loop;
        return r;
    end function;
    signal cotrom : t_cot := f_cot;
    signal s_cot_ra : unsigned(7 downto 0) := (others => '0');
    signal s_cot_q : unsigned(12 downto 0) := (others => '0');
    -- coefficient -tan(a) is negative in quadrants 1 and 3 (a(6) = '0')
    function f_cotneg(a : unsigned(7 downto 0)) return std_logic is
    begin
        return not a(6);
    end function;

    -- rays per step of angle for N = 2 .. 8 (256 / N)
    type t_astep is array (0 to 7) of integer;
    constant C_ASTEP : t_astep := (128, 128, 85, 64, 51, 43, 37, 32);

    -- ground hues {y, u, v}
    type t_col is array (0 to 2) of integer;
    type t_hue is array (0 to 7) of t_col;
    constant C_HUE : t_hue := ((224, 647, 500), (180, 470, 700), (210, 400, 420), (420, 620, 330),
                               (240, 640, 640), (300, 380, 560), (330, 690, 380), (330, 480, 560));

    -- line buffer {y, u, v, 00}
    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    -- oscillator source
    signal s_src : std_logic := '0';
    signal s_ox : unsigned(9 downto 0) := (others => '0');
    signal s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_oty, s_otu, s_otv : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- timing / controls
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(10 downto 0) := (others => '0');
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_j1 : unsigned(2 downto 0) := to_unsigned(5, 3);
    signal s_k2 : unsigned(9 downto 0) := (others => '0');
    signal s_k3 : unsigned(9 downto 0) := (others => '0');
    signal s_j4 : unsigned(2 downto 0) := (others => '0');
    signal s_live4 : std_logic := '0';
    signal s_k5r : signed(9 downto 0) := (others => '0');
    signal s_k6 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_n : unsigned(2 downto 0) := to_unsigned(5, 3);          -- rays - 1
    signal s_nb : unsigned(4 downto 0) := to_unsigned(12, 5);        -- live boundaries = 2 N
    signal s_aw : unsigned(5 downto 0) := to_unsigned(8, 6);         -- ray half width, angle units
    signal s_rot : unsigned(7 downto 0) := (others => '0');          -- rotation phase
    signal s_cy : unsigned(10 downto 0) := to_unsigned(540, 11);     -- centre y (virtual)
    signal s_cx : unsigned(11 downto 0) := to_unsigned(960, 12);     -- centre x (virtual)
    signal s_cx2sq : unsigned(23 downto 0) := (others => '0');
    signal s_inc0 : signed(14 downto 0) := (others => '0');
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_rr : unsigned(10 downto 0) := to_unsigned(300, 11);     -- halo radius
    signal s_rw : unsigned(10 downto 0) := to_unsigned(40, 11);      -- ring half width
    signal s_rin, s_rout : unsigned(10 downto 0) := (others => '0');
    signal s_rin2, s_rout2 : unsigned(21 downto 0) := (others => '0');
    signal s_hy, s_hu, s_hv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_rayon, s_glow, s_vin, s_cyn : std_logic := '0';
    -- ray DDAs
    type t_acc is array (0 to C_NB - 1) of signed(19 downto 0);
    type t_inc is array (0 to C_NB - 1) of signed(11 downto 0);
    signal s_bx : t_acc := (others => (others => '0'));              -- boundary x, Q.4 virtual (set to the
                                                                     -- top-line base in vblank, stepped per line)
    signal s_binc : t_inc := (others => (others => '0'));            -- per line increment, Q.4
    -- serial unit
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_mjob : unsigned(2 downto 0) := (others => '0');
    signal s_mk : unsigned(4 downto 0) := (others => '0');           -- boundary index of the job
    -- angle sequencer (zone)
    signal s_sq : unsigned(2 downto 0) := (others => '0');
    signal s_sk : unsigned(4 downto 0) := (others => '0');
    signal s_sa, s_sa0 : unsigned(7 downto 0) := (others => '0');
    signal s_sneg : std_logic := '0';
    signal s_scot : unsigned(12 downto 0) := (others => '0');
    signal s_sphase : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- source side / per line
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_fall1, s_fall2 : std_logic := '0';
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_ady_n : unsigned(10 downto 0) := to_unsigned(540, 11);
    signal s_dy2_n, s_dy2 : unsigned(19 downto 0) := to_unsigned(291600, 20);
    signal s_gty : unsigned(7 downto 0) := (others => '0');

    -- beam state
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_vxa : unsigned(12 downto 0) := (others => '0');
    signal s_dxh : signed(12 downto 0) := (others => '0');
    signal s_q : unsigned(23 downto 0) := (others => '0');
    signal s_inc : signed(14 downto 0) := (others => '0');
    signal s_gx : unsigned(9 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_q1 : unsigned(23 downto 0) := (others => '0');
    signal s_vx1 : signed(15 downto 0) := (others => '0');
    signal s_gt1 : unsigned(7 downto 0) := (others => '0');
    signal s_cmp2 : std_logic_vector(C_NB - 1 downto 0) := (others => '0');
    signal s_qi2 : unsigned(21 downto 0) := (others => '0');
    signal s_gt2 : unsigned(6 downto 0) := (others => '0');
    signal s_pink3, s_white3 : std_logic := '0';
    signal s_r2_3 : unsigned(22 downto 0) := (others => '0');
    signal s_pink, s_white : t_sl(4 to 8) := (others => '0');
    signal s_gt : t_u10(3 to 8) := (others => (others => '0'));
    signal s_in4, s_ring4 : std_logic := '0';
    signal s_in, s_ring : t_sl(5 to 8) := (others => '0');
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    signal s_rdq : t_u11(1 to 4) := (others => (others => '0'));
    signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y9, s_y10, s_y11, s_y12, s_y13x, s_yf13 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u13, s_v13 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y14, s_y15, s_y16, s_y17, s_y18, s_y19, s_y20 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u14, s_v14, s_u15, s_v15, s_u16, s_v16, s_u17, s_v17, s_u18, s_v18, s_u19, s_v19, s_u20, s_v20 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    ---------------------------------------------------------------------------
    -- RAMs
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_cot : process(clk)
    begin
        if rising_edge(clk) then
            s_cot_q <= cotrom(to_integer(s_cot_ra));
        end if;
    end process p_cot;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, angle sequencer + serial unit
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_i : signed(15 downto 0);
        variable v_p : unsigned(12 downto 0);
        variable v_r : unsigned(11 downto 0);
        variable v_c : unsigned(12 downto 0);
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
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;
            if s_zone = '0' then
                s_zc <= (others => '0');
            elsif s_zc /= 2047 then
                s_zc <= s_zc + 1;
            end if;

            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- K1 Rays 2 .. 8 (7 zones with a deadband)
            v_k := unsigned(registers_in(0));
            if s_warm /= 15 or ((v_k(6 downto 0) >= 6 or v_k(9 downto 7) = 0) and
                                (v_k(6 downto 0) <= 121 or v_k(9 downto 7) = 7)) then
                s_j1 <= v_k(9 downto 7);
            end if;
            s_k2 <= unsigned(registers_in(1));                                 -- K2 Halo
            s_k3 <= unsigned(registers_in(2));                                 -- K3 Ring
            -- K4 Ground (8 labels; the firmware loads the default as the index)
            v_k := unsigned(registers_in(3));
            if v_k >= 8 then s_live4 <= '1'; end if;
            if s_live4 = '0' and v_k < 8 then
                s_j4 <= v_k(2 downto 0);
            elsif s_warm /= 15 or ((v_k(6 downto 0) >= 6 or v_k(9 downto 7) = 0) and
                                   (v_k(6 downto 0) <= 121 or v_k(9 downto 7) = 7)) then
                s_j4 <= v_k(9 downto 7);
            end if;
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;        -- K5 Spin, deadband
            v_s := shift_right(v_s, 5);
            if v_s > -1 and v_s < 1 then s_k5r <= (others => '0'); else s_k5r <= v_s(9 downto 0); end if;
            s_k6 <= unsigned(registers_in(5));                                 -- K6 Centre
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                if s_j1 = 0 then s_n <= to_unsigned(1, 3); else s_n <= s_j1; end if;
                s_rot <= s_rot + unsigned(resize(s_k5r, 8));
                s_rayon <= not registers_in(6)(0);
                s_glow  <= registers_in(6)(1);
                s_vin   <= not registers_in(6)(2);
                s_cyn   <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_hy <= to_unsigned(C_HUE(to_integer(s_j4))(0), 10);
                s_hu <= to_unsigned(C_HUE(to_integer(s_j4))(1), 10);
                s_hv <= to_unsigned(C_HUE(to_integer(s_j4))(2), 10);
            end if;

            -- derived
            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            s_cx <= '0' & s_vw(11 downto 1);
            -- live boundaries 2 (N + 1)
            s_nb <= resize(s_n & '0', 5) + 2;
            -- ray half width: 1 .. 40 angle units (the full ray at 40 = 56 deg)
            v_p := resize(s_g(11 downto 2), 13);
            s_aw <= resize(v_p(12 downto 5), 6) + 1;
            -- centre y: 270 + k6 * 540 / 1024
            v_c := to_unsigned(270, 13) + resize(s_k6(9 downto 1), 13) + resize(s_k6(9 downto 5), 13) - resize(s_k6(9 downto 7), 13);
            s_cy <= v_c(10 downto 0);
            -- halo radius 60 .. 700, ring half width 4 .. 130
            v_r := to_unsigned(60, 12) + resize(s_k2(9 downto 1), 12) + resize(s_k2(9 downto 3), 12);
            s_rr <= v_r(10 downto 0);
            s_rw <= to_unsigned(4, 11) + resize(s_k3(9 downto 3), 11);
            if s_rr > s_rw then s_rin <= s_rr - s_rw; else s_rin <= (others => '0'); end if;
            s_rout <= s_rr + s_rw;
            case to_integer(s_xs) is
                when 4      => v_i := to_signed(16, 16) - signed(shift_left(resize(s_vw, 16), 3));
                               s_inc2 <= to_unsigned(32, 6);
                when 3      => v_i := to_signed(9, 16) - signed(shift_left(resize(s_vw, 16), 2) + shift_left(resize(s_vw, 16), 1));
                               s_inc2 <= to_unsigned(18, 6);
                when others => v_i := to_signed(4, 16) - signed(shift_left(resize(s_vw, 16), 2));
                               s_inc2 <= to_unsigned(8, 6);
            end case;
            s_inc0 <= v_i(14 downto 0);

            -- angle sequencer: for k in 0 .. 15 (ray k/2, boundary a - w or
            -- a + w) read the tan ROM, then run two serial products |tan| x ys
            -- (increment) and |tan| x cy (base).  Runs from zone clock 64, 32
            -- clocks per boundary.
            if s_zc = 60 then
                s_sk <= (others => '0');
                s_sa0 <= s_rot;
                s_sphase <= '0';
            end if;
            if s_zc >= 64 and s_zc < 64 + 32 * C_NB then
                case to_integer(s_zc(4 downto 0)) is
                    when 0 =>
                        if s_sphase = '0' then s_sa <= s_sa0 - s_aw; else s_sa <= s_sa0 + s_aw; end if;
                    when 1 => s_cot_ra <= s_sa;
                    when 3 => s_scot <= s_cot_q; s_sneg <= f_cotneg(s_sa);
                    when 4 =>   -- job: |cot| x ys
                        s_ma <= resize(s_scot, 24); s_mb <= s_ys; s_macc <= (others => '0');
                        s_mjob <= "001"; s_mrun <= '1'; s_mi <= (others => '0'); s_mk <= s_sk;
                    when 18 =>  -- job: |cot| x cy
                        s_ma <= resize(s_scot, 24); s_mb <= resize(s_cy, 12); s_macc <= (others => '0');
                        s_mjob <= "010"; s_mrun <= '1'; s_mi <= (others => '0'); s_mk <= s_sk;
                    when 31 =>
                        s_sk <= s_sk + 1;
                        if s_sphase = '1' then
                            s_sphase <= '0';
                            s_sa0 <= s_sa0 + to_unsigned(C_ASTEP(to_integer(s_n)), 8);
                        else
                            s_sphase <= '1';
                        end if;
                    when others => null;
                end case;
            end if;
            -- the squares: (2cx)^2 at zone clock 4, dy^2 for the first line at 20
            if s_zc = 4 then
                s_ma <= resize(s_vw, 24); s_mb <= s_vw; s_macc <= (others => '0');
                s_mjob <= "000"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 20 or s_fall2 = '1' then
                s_ma <= resize(s_ady_n, 24); s_mb <= resize(s_ady_n, 12); s_macc <= (others => '0');
                s_mjob <= "011"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 36 then
                s_ma <= resize(s_rin, 24); s_mb <= resize(s_rin, 12); s_macc <= (others => '0');
                s_mjob <= "100"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 48 then
                s_ma <= resize(s_rout, 24); s_mb <= resize(s_rout, 12); s_macc <= (others => '0');
                s_mjob <= "101"; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_fall1 = '1' then
                -- line end: step every boundary (the beam's last few pixels of
                -- the line see the next line's value, a 1-line slip, invisible)
                for i in 0 to C_NB - 1 loop s_bx(i) <= s_bx(i) + resize(s_binc(i), 20); end loop;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "000"  => s_cx2sq <= s_macc;
                    when "001"  =>   -- increment per field line, Q.4: (|tan| Q.8 x ys Q.8) >> 12
                        if s_sneg = '1' then
                            s_binc(to_integer(s_mk)) <= -signed(resize(s_macc(22 downto 12), 12));
                        else
                            s_binc(to_integer(s_mk)) <= signed(resize(s_macc(22 downto 12), 12));
                        end if;
                    when "010"  =>   -- top-line x = cx - coefficient * cy (Q.4), straight into the DDA
                        if s_sneg = '1' then
                            s_bx(to_integer(s_mk)) <= signed(resize(s_cx & x"0", 20)) + signed(resize(s_macc(23 downto 4), 20));
                        else
                            s_bx(to_integer(s_mk)) <= signed(resize(s_cx & x"0", 20)) - signed(resize(s_macc(23 downto 4), 20));
                        end if;
                    when "011"  => s_dy2_n <= s_macc(19 downto 0);
                    when "100"  => s_rin2 <= s_macc(21 downto 0);
                    when others => s_rout2 <= s_macc(21 downto 0);
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
    -- Source side: line-buffer write, oscillator, per-line values
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_dy : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            if data_in.avid = '1' then s_ox <= s_ox + 3; elsif s_avq = '1' then s_ox <= (others => '0'); s_oy <= s_oy + 5; end if;
            if s_zone = '1' then s_oy <= (others => '0'); end if;
            if s_zgo = '1' then s_ot <= s_ot + 3; end if;
            s_oty <= to_unsigned(64, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot) & '0', 10)
                     + resize(f_tri8(s_ox(9 downto 2) + s_ot), 10) + resize(f_tri8(s_oy(9 downto 2) - s_ot), 10);
            s_otu <= to_unsigned(320, 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot + s_ot) & '0', 10) + resize(f_tri8(s_ox(9 downto 2) + s_ot + s_ot), 10);
            s_otv <= to_unsigned(320, 10) + resize(f_tri8(s_oy(9 downto 2) + s_ot) & '0', 10) + resize(f_tri8(s_oy(9 downto 2) + s_ot), 10);
            s_lb_we <= data_in.avid;
            s_lb_wa <= s_wcol;
            if s_src = '1' then
                s_lb_wd <= std_logic_vector(s_oty) & std_logic_vector(s_otu) & std_logic_vector(s_otv) & "00";
            else
                s_lb_wd <= data_in.y & data_in.u & data_in.v & "00";
            end if;
            if data_in.avid = '1' then s_wcol <= s_wcol + 1; else s_wcol <= (others => '0'); end if;

            s_fall1 <= '0';
            if data_in.avid = '0' and s_avq = '1' then s_fall1 <= '1'; end if;
            s_fall2 <= s_fall1;

            if s_zone = '1' then
                s_vy_n <= (others => '0');
                s_ady_n <= s_cy;
                s_gty <= (others => '0');
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
                s_gty <= s_gty + 1;
            elsif s_fall2 = '1' then
                v_dy := signed(resize(s_vy_n(19 downto 8), 13)) - signed(resize(s_cy, 13));
                if v_dy < 0 then s_ady_n <= resize(unsigned(-v_dy), 11);
                else s_ady_n <= unsigned(v_dy(10 downto 0)); end if;
            end if;

            if data_in.avid = '1' and s_avq = '0' then
                s_dy2 <= s_dy2_n;
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state
    ---------------------------------------------------------------------------
    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= (others => '0');
                s_vxa <= (others => '0');
                s_dxh <= -signed(resize(s_vw, 13));
                s_q <= s_cx2sq;
                s_inc <= s_inc0;
                s_gx <= (others => '0');
            elsif g_avid = '1' then
                s_rdp <= s_rdp + 1;
                s_vxa <= s_vxa + s_xs;
                s_dxh <= s_dxh + signed(resize(s_xs, 13));
                s_q <= unsigned(signed(s_q) + resize(s_inc, 24));
                s_inc <= s_inc + signed(resize(s_inc2, 15));
                s_gx <= s_gx + 5;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_p, v_w : std_logic;
        variable v_hx : unsigned(21 downto 0);
        variable v_ty : unsigned(9 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_t : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 1
            s_q1 <= s_q;
            s_vx1 <= signed(resize(s_vxa(12 downto 1), 16));
            s_gt1 <= f_tri8(s_gx(9 downto 2) + s_gty) & '0';
            s_rdq(1) <= s_rdp;

            -- 2: boundary compares, dx^2
            for i in 0 to C_NB - 1 loop
                if i < to_integer(s_nb) and s_vx1 > s_bx(i)(19 downto 4) then s_cmp2(i) <= '1'; else s_cmp2(i) <= '0'; end if;
            end loop;
            s_qi2 <= s_q1(23 downto 2);
            s_gt2 <= s_gt1(7 downto 1);

            -- 3: ray stripes (the double cone between a-w and a+w), pink for
            --    even rays, white for odd ones; r^2
            v_p := '0'; v_w := '0';
            for k in 0 to 3 loop
                v_p := v_p or (s_cmp2(4 * k) xor s_cmp2(4 * k + 1));
                v_w := v_w or (s_cmp2(4 * k + 2) xor s_cmp2(4 * k + 3));
            end loop;
            s_pink3 <= v_p and s_rayon;
            s_white3 <= v_w and s_rayon;
            s_r2_3 <= resize(s_qi2, 23) + resize(s_dy2, 23);
            s_gt(3) <= resize(s_gt2, 10);

            -- 4: halo tests; video read
            if s_r2_3 < resize(s_rin2, 23) then s_in4 <= '1'; else s_in4 <= '0'; end if;
            if s_r2_3 >= resize(s_rin2, 23) and s_r2_3 <= resize(s_rout2, 23) then s_ring4 <= '1'; else s_ring4 <= '0'; end if;
            s_pink(4) <= s_pink3; s_white(4) <= s_white3;
            s_lb_ra <= s_rdq(3);

            -- 5: buffer output register (EBR); 6: fabric copy
            s_y7 <= unsigned(s_lb_q(31 downto 22));
            s_u7 <= unsigned(s_lb_q(21 downto 12));
            s_v7 <= unsigned(s_lb_q(11 downto 2));

            -- 7: compose (stage-6 flags, video fabric copy)
            v_ty := s_hy + resize(s_gt(6)(6 downto 1), 10);
            if s_ring(6) = '1' then
                if s_glow = '1' then
                    s_y8 <= to_unsigned(900, 10); s_u8 <= to_unsigned(512, 10); s_v8 <= to_unsigned(512, 10);
                else
                    s_y8 <= to_unsigned(80, 10); s_u8 <= to_unsigned(540, 10); s_v8 <= to_unsigned(500, 10);
                end if;
            elsif s_in(6) = s_vin then
                s_y8 <= s_y7; s_u8 <= s_u7; s_v8 <= s_v7;
            elsif s_white(6) = '1' then
                if s_cyn = '1' then
                    s_y8 <= to_unsigned(840, 10); s_u8 <= to_unsigned(584, 10); s_v8 <= to_unsigned(64, 10);
                else
                    s_y8 <= to_unsigned(940, 10); s_u8 <= to_unsigned(512, 10); s_v8 <= to_unsigned(512, 10);
                end if;
            elsif s_pink(6) = '1' then
                if s_cyn = '1' then
                    s_y8 <= to_unsigned(678, 10); s_u8 <= to_unsigned(64, 10); s_v8 <= to_unsigned(663, 10);
                else
                    s_y8 <= to_unsigned(752, 10); s_u8 <= to_unsigned(612, 10); s_v8 <= to_unsigned(512, 10);
                end if;
            else
                s_y8 <= v_ty; s_u8 <= s_hu; s_v8 <= s_hv;
            end if;
            -- 8..13: output conditioning (centre at stage 12 -> 13)
            s_csu(0) <= s_u8; s_csv(0) <= s_v8;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u8 - s_csu(7);
            s_boxv <= s_boxv + s_v8 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u13 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v13 <= v_sc(13 downto 4);
            s_y9 <= s_y8; s_y10 <= s_y9; s_y11 <= s_y10; s_y12 <= s_y11; s_y13x <= s_y12;
            v_sy := resize(s_y11, 12) + shift_left(resize(s_y12, 12), 1) + resize(s_y13x, 12);
            s_yf13 <= v_sy(11 downto 2);
            -- 14: luma toward black; blanking gate
            if s_yf13 > 64 then v_dl := s_yf13 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(12) = '1' then
                s_y14 <= s_yf13 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u14 <= s_u13; s_v14 <= s_v13;
            else
                s_y14 <= to_unsigned(64, 10); s_u14 <= to_unsigned(512, 10); s_v14 <= to_unsigned(512, 10);
            end if;
            -- 15..20: pad
            s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14;
            s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;
            s_y17 <= s_y16; s_u17 <= s_u16; s_v17 <= s_v16;
            s_y18 <= s_y17; s_u18 <= s_u17; s_v18 <= s_v17;
            s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            s_y20 <= s_y19; s_u20 <= s_u19; s_v20 <= s_v19;

            -- delays
            for i in 2 to 4 loop s_rdq(i) <= s_rdq(i - 1); end loop;
            for i in 4 to 8 loop s_gt(i) <= s_gt(i - 1); end loop;
            for i in 5 to 8 loop s_pink(i) <= s_pink(i - 1); s_white(i) <= s_white(i - 1); end loop;
            s_in(5) <= s_in4; s_ring(5) <= s_ring4;
            for i in 6 to 8 loop s_in(i) <= s_in(i - 1); s_ring(i) <= s_ring(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture radiant;
