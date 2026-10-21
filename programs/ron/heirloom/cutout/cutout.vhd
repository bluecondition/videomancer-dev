-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Cutout v0.1.0 -- Heirloom #55 (2026-10-20), from tezcatlipoca.jpg (a paper
-- collage).
--
-- The collage: a flat yellow ground; a head in profile cut out and filled
-- with torn blue, teal and black magazine pieces.
--
-- The program: a head silhouette (a 34-row half-width / centre-offset
-- profile, interpolated and scaled per line) on a flat yellow ground, filled
-- with the video posterized into collage pieces: a cell grid of torn paper
-- (hashed luma offsets per piece) and horizontal tears in bands.  P12 scales
-- the head.
--
--   K1  Tear     sideways tearing of the pieces in 16-line bands
--   K2  Levels   posterize threshold spread (2 .. 3 tones)
--   K3  Centre   head position, left .. right
--   K4  Cream    ground: yellow .. cream
--   K5  Pieces   piece size, 32 .. 256 px
--   K6  Sway     the head sways, bipolar rate (centre = still)
--   S7  Profile  Head / Oval
--   S8  Pieces   Torn (posterized pieces) / Plain (the video itself)
--   S9  Palette  Blue collage / Warm collage
--   S10 Outline  black outline on / off
--   S11 Source   Video / Osc
--   P12 HEAD     size, 25 .. 125 % of the screen height
--
-- Virtual 1080-line geometry; per-line profile interpolation and scaling on
-- the serial unit (4 products per line, a divide per field).  Generated
-- avid; output conditioning before the gate.  Latency 20.  EBR 16.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture cutout of program_top is
    constant C_LAT : integer := 20;

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

    -- profiles: 36 rows (32 virtual lines each at full scale), half width and
    -- centre offset in virtual px.  Head: a cranium leaning right, nose, jaw.
    type t_prof is array (0 to 35) of integer;
    constant C_HW_HEAD : t_prof := (0, 120, 230, 300, 345, 370, 385, 395, 400, 400, 398, 395, 390, 385, 380, 375, 370, 365,
                                    360, 355, 350, 345, 335, 320, 300, 275, 245, 210, 170, 120, 60, 0, 0, 0, 0, 0);
    constant C_CO_HEAD : t_prof := (0, 10, 20, 25, 30, 35, 38, 40, 40, 40, 38, 35, 30, 25, 20, 15, 10, 5,
                                    0, -5, -10, -15, -20, -25, -30, -35, -40, -45, -50, -55, -60, 0, 0, 0, 0, 0);
    constant C_HW_OVAL : t_prof := (0, 150, 230, 285, 325, 355, 375, 390, 398, 400, 400, 400, 400, 400, 400, 400, 400, 400,
                                    400, 400, 400, 400, 400, 400, 398, 390, 375, 355, 325, 285, 230, 150, 0, 0, 0, 0);

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
    signal s_zc : unsigned(6 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_tk : unsigned(7 downto 0) := (others => '0');
    signal s_k2 : unsigned(7 downto 0) := (others => '0');
    signal s_k3 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k4 : unsigned(7 downto 0) := (others => '0');
    signal s_k5 : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_k6r : signed(9 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_sc : unsigned(8 downto 0) := to_unsigned(256, 9);       -- head scale Q.8 (64 .. 320)
    signal s_y0 : unsigned(10 downto 0) := (others => '0');          -- head top (virtual line)
    signal s_inc : unsigned(11 downto 0) := to_unsigned(2048, 12);   -- profile rows per field line, Q.8
    signal s_t1, s_t2 : unsigned(9 downto 0) := to_unsigned(300, 10);
    signal s_cxb : signed(12 downto 0) := to_signed(960, 13);        -- head centre x
    signal s_gy, s_gu, s_gv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_psh : unsigned(2 downto 0) := to_unsigned(6, 3);        -- piece shift 5 .. 8
    signal s_seed : unsigned(7 downto 0) := (others => '0');
    signal s_swph : unsigned(9 downto 0) := (others => '0');
    signal s_oval, s_plain, s_warm, s_outl : std_logic := '0';
    -- serial unit
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun, s_mdiv : std_logic := '0';
    signal s_mjob : unsigned(2 downto 0) := (others => '0');
    signal s_dnum : unsigned(13 downto 0) := (others => '0');
    signal s_drem : unsigned(14 downto 0) := (others => '0');
    signal s_dquo : unsigned(13 downto 0) := (others => '0');
    signal s_lfsr : unsigned(15 downto 0) := x"C0DE";

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
    signal s_lseq : unsigned(6 downto 0) := (others => '0');          -- per-line job sequencer
    signal s_vy_n : unsigned(19 downto 0) := (others => '0');
    signal s_p : unsigned(15 downto 0) := (others => '0');             -- profile position, Q.8 rows
    signal s_hw0, s_hw1, s_hwi : unsigned(9 downto 0) := (others => '0');
    signal s_co0, s_co1, s_coi : signed(7 downto 0) := (others => '0');
    signal s_dhw : signed(10 downto 0) := (others => '0');
    signal s_dco : signed(8 downto 0) := (others => '0');
    signal s_frac : unsigned(7 downto 0) := (others => '0');
    signal s_hws_n, s_hws, s_hwe : unsigned(10 downto 0) := (others => '0');
    signal s_cos_n, s_cos : signed(8 downto 0) := (others => '0');
    signal s_cxo : signed(12 downto 0) := to_signed(960, 13);
    signal s_tear_n : unsigned(10 downto 0) := (others => '0');
    signal s_rst : unsigned(10 downto 0) := (others => '0');
    signal s_l16 : unsigned(3 downto 0) := (others => '0');
    signal s_cyid : unsigned(5 downto 0) := (others => '0');

    -- beam state
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_vxa : unsigned(12 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u11 is array (natural range <>) of unsigned(10 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_vx1 : unsigned(11 downto 0) := (others => '0');
    signal s_cxid1 : unsigned(6 downto 0) := (others => '0');
    signal s_rdq : t_u11(1 to 4) := (others => (others => '0'));
    signal s_dx2 : signed(12 downto 0) := (others => '0');
    signal s_h2 : unsigned(7 downto 0) := (others => '0');
    signal s_in3, s_edge3 : std_logic := '0';
    signal s_h3 : unsigned(3 downto 0) := (others => '0');
    signal s_in, s_edge : t_sl(4 to 7) := (others => '0');
    type t_u4 is array (natural range <>) of unsigned(3 downto 0);
    signal s_h : t_u4(4 to 7) := (others => (others => '0'));
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_py7, s_pu7, s_pv7 : unsigned(9 downto 0) := to_unsigned(64, 10);
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
    -- RAM
    ---------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, serial unit (per-field divide
    -- and product, per-line profile products)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_c : signed(13 downto 0);
        variable v_r : unsigned(14 downto 0);
        variable v_t : unsigned(17 downto 0);
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
            elsif s_zc /= 127 then
                s_zc <= s_zc + 1;
            end if;

            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            s_tk <= unsigned(registers_in(0)(9 downto 2));                     -- K1 Tear
            s_k2 <= unsigned(registers_in(1)(9 downto 2));                     -- K2 Levels
            s_k3 <= unsigned(registers_in(2));                                 -- K3 Centre
            s_k4 <= unsigned(registers_in(3)(9 downto 2));                     -- K4 Cream
            s_k5 <= unsigned(registers_in(4)(9 downto 7));                     -- K5 Pieces (8 zones)
            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;        -- K6 Sway, deadband
            v_s := shift_right(v_s, 4);
            if v_s > -1 and v_s < 1 then s_k6r <= (others => '0'); else s_k6r <= v_s(9 downto 0); end if;
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                -- scale 64 + p12/4 (0.25 .. 1.25)
                s_sc <= to_unsigned(64, 9) + resize(s_g(11 downto 4), 9);
                -- posterize thresholds: 160 + k2/2 and 480 + k2
                s_t1 <= to_unsigned(160, 10) + resize(s_k2(7 downto 1), 10);
                s_t2 <= to_unsigned(480, 10) + resize(s_k2, 10);
                s_psh <= to_unsigned(5, 3) + resize(s_k5(2 downto 1), 3);     -- 5 .. 8
                s_seed <= s_seed + 1;
                s_swph <= s_swph + unsigned(resize(s_k6r, 10));
                s_oval  <= registers_in(6)(0);
                s_plain <= registers_in(6)(1);
                s_warm  <= registers_in(6)(2);
                s_outl  <= not registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                -- ground: yellow (840, 584, 64) toward cream (900, 530, 420)
                s_gy <= to_unsigned(840, 10) + resize(s_k4(7 downto 2), 10);
                s_gu <= to_unsigned(584, 10) - resize(s_k4(7 downto 2), 10);
                v_t := s_k4 * to_unsigned(356, 10);
                s_gv <= to_unsigned(64, 10) + resize(v_t(17 downto 8), 10);
            end if;

            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                when others => v_vw := resize(s_w, 14);
            end case;
            s_vw <= v_vw(11 downto 0);
            -- head centre: vw/2 + (k3 - 512) * 3/4 + sway (+-64)
            v_c := signed(resize(s_vw(11 downto 1), 14)) + (signed(resize(s_k3, 14)) - 512) - shift_right(signed(resize(s_k3, 14)) - 512, 2)
                   + signed(resize(f_tri8(s_swph(9 downto 2)), 14)) - 64;
            s_cxb <= v_c(12 downto 0);

            -- serial unit.  Per field: job 0 = 540 x sc (head top), job 1 =
            -- (ys << 3) / sc (rows per line).  Per line (sequenced in p_src):
            -- job 2 |dhw| x frac, job 3 hwi x sc, job 4 |dco| x frac, job 5 |coi| x sc.
            if s_zc = 4 then
                s_ma <= to_unsigned(540, 24); s_mb <= resize(s_sc, 12); s_macc <= (others => '0');
                s_mjob <= "000"; s_mdiv <= '0'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_zc = 20 then
                s_dnum <= shift_left(resize(s_ys, 14), 3); s_drem <= (others => '0'); s_mb <= resize(s_sc, 12);
                s_mjob <= "001"; s_mdiv <= '1'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_lseq = 2 then
                if s_dhw < 0 then s_ma <= resize(unsigned(-s_dhw), 24); else s_ma <= resize(unsigned(s_dhw), 24); end if;
                s_mb <= resize(s_frac, 12); s_macc <= (others => '0');
                s_mjob <= "010"; s_mdiv <= '0'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_lseq = 18 then
                s_ma <= resize(s_hwi, 24); s_mb <= resize(s_sc, 12); s_macc <= (others => '0');
                s_mjob <= "011"; s_mdiv <= '0'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_lseq = 34 then
                if s_dco < 0 then s_ma <= resize(unsigned(-s_dco), 24); else s_ma <= resize(unsigned(s_dco), 24); end if;
                s_mb <= resize(s_frac, 12); s_macc <= (others => '0');
                s_mjob <= "100"; s_mdiv <= '0'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_lseq = 50 then
                if s_coi < 0 then s_ma <= resize(unsigned(-s_coi), 24); else s_ma <= resize(unsigned(s_coi), 24); end if;
                s_mb <= resize(s_sc, 12); s_macc <= (others => '0');
                s_mjob <= "101"; s_mdiv <= '0'; s_mrun <= '1'; s_mi <= (others => '0');
            elsif s_mrun = '1' then
                if s_mdiv = '1' then
                    v_r := s_drem(13 downto 0) & s_dnum(13);
                    s_dnum <= s_dnum(12 downto 0) & '0';
                    if v_r >= resize(s_mb, 15) then
                        s_drem <= v_r - resize(s_mb, 15);
                        s_dquo <= s_dquo(12 downto 0) & '1';
                    else
                        s_drem <= v_r;
                        s_dquo <= s_dquo(12 downto 0) & '0';
                    end if;
                    if s_mi = 13 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
                else
                    if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                    s_ma <= s_ma(22 downto 0) & '0';
                    s_mb <= '0' & s_mb(11 downto 1);
                    if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
                end if;
            elsif s_mi /= 0 then
                s_mi <= (others => '0');
                case s_mjob is
                    when "000"  => s_y0 <= to_unsigned(540, 11) - resize(s_macc(18 downto 8), 11);
                    when "001"  => s_inc <= s_dquo(11 downto 0);
                    when "010"  =>
                        if s_dhw < 0 then s_hwi <= s_hw0 - resize(s_macc(17 downto 8), 10);
                        else s_hwi <= s_hw0 + resize(s_macc(17 downto 8), 10); end if;
                    when "011"  => s_hws_n <= resize(s_macc(18 downto 8), 11);
                    when "100"  =>
                        if s_dco < 0 then s_coi <= s_co0 - signed(resize(s_macc(15 downto 8), 8));
                        else s_coi <= s_co0 + signed(resize(s_macc(15 downto 8), 8)); end if;
                    when others =>
                        if s_coi < 0 then s_cos_n <= -signed(resize(s_macc(16 downto 8), 9));
                        else s_cos_n <= signed(resize(s_macc(16 downto 8), 9)); end if;
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
    -- Source side: line-buffer write; per-line profile row + job sequence
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_t : unsigned(15 downto 0);
        variable v_tr : unsigned(10 downto 0);
        variable v_i : integer range 0 to 35;
        variable v_p : unsigned(16 downto 0);
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
            -- per-line job sequencer: starts at the fall, also once in the zone
            if s_fall1 = '1' or (s_zone = '1' and s_zc = 60) then
                s_lseq <= to_unsigned(1, 7);
            elsif s_lseq /= 0 and s_lseq /= 127 then
                s_lseq <= s_lseq + 1;
            end if;

            if s_zone = '1' then
                s_vy_n <= (others => '0');
                s_p <= (others => '0');
                s_l16 <= (others => '0');
                s_cyid <= (others => '0');
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                s_tear_n <= (others => '0');
            elsif s_fall1 = '1' then
                s_vy_n <= s_vy_n + s_ys;
                s_l16 <= s_l16 + 1;
                if s_vy_n(19 downto 8) >= s_y0 then
                    v_p := resize(s_p, 17) + resize(s_inc, 17);
                    if v_p(16) = '1' then s_p <= (others => '1'); else s_p <= v_p(15 downto 0); end if;
                else
                    s_p <= (others => '0');
                end if;
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                if s_l16 = 15 then
                    v_t := s_lfsr(7 downto 0) * s_tk;
                    v_tr := resize(v_t(15 downto 7), 11);
                    if v_tr >= s_wm1(10 downto 0) then s_tear_n <= s_wm1(10 downto 0);
                    else s_tear_n <= v_tr; end if;
                end if;
            end if;
            -- profile row lookup (s_lseq = 1): rows i and i + 1
            if s_lseq = 1 then
                if s_p(15 downto 8) >= 34 then v_i := 34; else v_i := to_integer(s_p(13 downto 8)); end if;
                if s_oval = '1' then
                    s_hw0 <= to_unsigned(C_HW_OVAL(v_i), 10);
                    s_hw1 <= to_unsigned(C_HW_OVAL(v_i + 1), 10);
                    s_co0 <= (others => '0');
                    s_co1 <= (others => '0');
                else
                    s_hw0 <= to_unsigned(C_HW_HEAD(v_i), 10);
                    s_hw1 <= to_unsigned(C_HW_HEAD(v_i + 1), 10);
                    s_co0 <= to_signed(C_CO_HEAD(v_i), 8);
                    s_co1 <= to_signed(C_CO_HEAD(v_i + 1), 8);
                end if;
                s_frac <= s_p(7 downto 0);
                -- piece row id from the virtual line
                case to_integer(s_psh) is
                    when 5      => s_cyid <= s_vy_n(18 downto 13);
                    when 6      => s_cyid <= s_vy_n(19 downto 14);
                    when 7      => s_cyid <= '0' & s_vy_n(19 downto 15);
                    when others => s_cyid <= "00" & s_vy_n(19 downto 16);
                end case;
            end if;
            s_dhw <= signed(resize(s_hw1, 11)) - signed(resize(s_hw0, 11));
            s_dco <= resize(s_co1, 9) - resize(s_co0, 9);

            if data_in.avid = '1' and s_avq = '0' then
                s_hws <= s_hws_n;
                if s_hws_n > 6 then s_hwe <= s_hws_n - 6; else s_hwe <= (others => '0'); end if;
                s_cos <= s_cos_n;
                s_cxo <= s_cxb + resize(s_cos_n, 13);
                if s_tear_n = 0 then s_rst <= (others => '0'); else s_rst <= s_wm1(10 downto 0) + 1 - s_tear_n; end if;
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
                s_rdp <= s_rst;
                s_vxa <= (others => '0');
            elsif g_avid = '1' then
                if s_rdp = s_wm1(10 downto 0) then s_rdp <= (others => '0'); else s_rdp <= s_rdp + 1; end if;
                s_vxa <= s_vxa + s_xs;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_ad : signed(12 downto 0);
        variable v_h : unsigned(7 downto 0);
        variable v_b : unsigned(1 downto 0);
        variable v_y : signed(11 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: virtual x, piece column id
            s_vx1 <= s_vxa(12 downto 1);
            case to_integer(s_psh) is
                when 5      => s_cxid1 <= s_vxa(12 downto 6);
                when 6      => s_cxid1 <= '0' & s_vxa(12 downto 7);
                when 7      => s_cxid1 <= "00" & s_vxa(12 downto 8);
                when others => s_cxid1 <= "000" & s_vxa(12 downto 9);
            end case;
            s_rdq(1) <= s_rdp;

            -- 2: dx from the head centre; piece hash
            s_dx2 <= signed(resize(s_vx1, 13)) - s_cxo;
            v_h := (s_cxid1 & '0') xor (s_cyid & "00") xor s_seed;
            s_h2 <= v_h + (s_cxid1 & '0') + (s_cyid(2 downto 0) & "00000") + 37;

            -- 3: inside / edge tests; hash bits
            if s_dx2 < 0 then v_ad := -s_dx2; else v_ad := s_dx2; end if;
            if unsigned(v_ad(10 downto 0)) <= s_hws and v_ad(12 downto 11) = "00" then s_in3 <= '1'; else s_in3 <= '0'; end if;
            if unsigned(v_ad(10 downto 0)) > s_hwe and v_ad(12 downto 11) = "00" then s_edge3 <= '1'; else s_edge3 <= '0'; end if;
            s_h3 <= s_h2(6 downto 3) xor s_h2(3 downto 0);

            -- 4: video read
            s_lb_ra <= s_rdq(3);
            s_in(4) <= s_in3; s_edge(4) <= s_edge3; s_h(4) <= s_h3;

            -- 5: EBR output register; 6: fabric copy
            s_y6 <= unsigned(s_lb_q(31 downto 22)); s_u6 <= unsigned(s_lb_q(21 downto 12)); s_v6 <= unsigned(s_lb_q(11 downto 2));

            -- 7: posterize into collage pieces
            v_b := "00";
            if s_y6 >= s_t1 then v_b := "01"; end if;
            if s_y6 >= s_t2 then v_b := "10"; end if;
            if s_h(6)(3) = '1' and v_b = "01" then v_b := "11"; end if;     -- some mid pieces take the fourth tone
            if s_warm = '1' then
                case v_b is
                    when "00"   => s_py7 <= to_unsigned(140, 10); s_pu7 <= to_unsigned(560, 10); s_pv7 <= to_unsigned(470, 10);   -- dark brown
                    when "01"   => s_py7 <= to_unsigned(584, 10); s_pu7 <= to_unsigned(772, 10); s_pv7 <= to_unsigned(212, 10);   -- orange
                    when "10"   => s_py7 <= to_unsigned(860, 10); s_pu7 <= to_unsigned(540, 10); s_pv7 <= to_unsigned(470, 10);   -- cream
                    when others => s_py7 <= to_unsigned(328, 10); s_pu7 <= to_unsigned(960, 10); s_pv7 <= to_unsigned(360, 10);   -- red
                end case;
            else
                case v_b is
                    when "00"   => s_py7 <= to_unsigned(90, 10);  s_pu7 <= to_unsigned(490, 10); s_pv7 <= to_unsigned(560, 10);   -- near-black navy
                    when "01"   => s_py7 <= to_unsigned(372, 10); s_pu7 <= to_unsigned(288, 10); s_pv7 <= to_unsigned(588, 10);   -- teal
                    when "10"   => s_py7 <= to_unsigned(678, 10); s_pu7 <= to_unsigned(64, 10);  s_pv7 <= to_unsigned(663, 10);   -- cyan-blue
                    when others => s_py7 <= to_unsigned(164, 10); s_pu7 <= to_unsigned(440, 10); s_pv7 <= to_unsigned(960, 10);   -- blue
                end case;
            end if;
            s_y7 <= s_y6; s_u7 <= s_u6; s_v7 <= s_v6;

            -- 8: compose (piece luma offset from the hash, +-56)
            v_y := signed(resize(s_py7, 12)) + signed(resize(s_h(7)(2 downto 0) & "000", 12)) - 28;
            if s_in(7) = '0' then
                s_y8 <= s_gy; s_u8 <= s_gu; s_v8 <= s_gv;
            elsif s_edge(7) = '1' and s_outl = '1' then
                s_y8 <= to_unsigned(64, 10); s_u8 <= to_unsigned(512, 10); s_v8 <= to_unsigned(512, 10);
            elsif s_plain = '1' then
                s_y8 <= s_y7; s_u8 <= s_u7; s_v8 <= s_v7;
            else
                if v_y < 64 then s_y8 <= to_unsigned(64, 10); else s_y8 <= unsigned(v_y(9 downto 0)); end if;
                s_u8 <= s_pu7; s_v8 <= s_pv7;
            end if;

            -- 9..13: output conditioning (centre at stage 12 -> 13)
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
            for i in 5 to 7 loop s_in(i) <= s_in(i - 1); s_edge(i) <= s_edge(i - 1); s_h(i) <= s_h(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture cutout;
