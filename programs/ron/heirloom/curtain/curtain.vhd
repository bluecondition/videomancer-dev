-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Curtain v0.1.0 -- Heirloom #34 (2026-10-20), from WIN_20230521_01_34_12.
--
-- The photo: a red field hung with magenta rippled drapes (vertical folds
-- wrung into S-curves), blue pools where the folds part, lime-green piping
-- along the fold crests, and rows torn sideways.
--
-- The program: a vertical fold texture tri(x + A tri(y)) shaded from red to
-- magenta (the fold's own luma), with the video (or the oscillator) added to
-- the fold phase so the drapes follow the picture; a slow 2-D oscillator
-- opens blue pools; lime piping where the folds crest; 8-line tear bands.
--
--   K1  Pitch    fold pitch
--   K2  Pools    pool size (none .. the drapes mostly parted)
--   K3  Piping   lime crest width
--   K4  Hue      Red/Magenta, Blue/Cyan, Green/Yellow, Violet/Pink
--   K5  Drift    fold phase drift, bipolar
--   K6  Tear     sideways tearing in 8-line bands
--   S7  Pools    on / off
--   S8  Piping   on / off
--   S9  Mirror   folds mirrored about the centre
--   S10 Shade    Dark / Light fold shading
--   S11 Source   Video / Osc
--   P12 DRAPE    the wring: straight folds .. deep S-curves
--
-- Virtual 1080-line geometry.  Generated avid; output conditioning before
-- the blanking gate.  Latency 20 clocks.  16 EBR (the buffer carries only
-- the luma the folds use).  Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture curtain of program_top is
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

    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    signal s_src : std_logic := '0';
    signal s_ox : unsigned(9 downto 0) := (others => '0');
    signal s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_oty, s_otu, s_otv : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_fx : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_pk : unsigned(7 downto 0) := (others => '0');
    signal s_pw : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_j4 : unsigned(1 downto 0) := (others => '0');
    signal s_live4 : std_logic := '0';
    signal s_k5r : signed(9 downto 0) := (others => '0');
    signal s_tk : unsigned(7 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- latched / derived
    signal s_hue : unsigned(1 downto 0) := (others => '0');
    signal s_pools, s_piping, s_mirror, s_light : std_logic := '0';
    signal s_amp : unsigned(7 downto 0) := (others => '0');
    signal s_fxs : unsigned(9 downto 0) := to_unsigned(16, 10);
    signal s_fxs2 : unsigned(9 downto 0) := to_unsigned(4, 10);
    signal s_ph : unsigned(7 downto 0) := (others => '0');
    signal s_pwt : unsigned(6 downto 0) := to_unsigned(119, 7);       -- crest threshold 127 - pw
    signal s_lfsr : unsigned(15 downto 0) := x"C0DE";

    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- source / per line
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_fall1 : std_logic := '0';
    signal s_py, s_py2 : unsigned(11 downto 0) := (others => '0');
    signal s_l8 : unsigned(2 downto 0) := (others => '0');
    signal s_tr_n : unsigned(7 downto 0) := (others => '0');
    signal s_cl_p : unsigned(14 downto 0) := (others => '0');          -- A x tri(py)
    signal s_cl, s_cl_n : unsigned(7 downto 0) := (others => '0');     -- line fold phase
    signal s_tpy2 : unsigned(6 downto 0) := (others => '0');

    -- beam
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_vxa : unsigned(12 downto 0) := (others => '0');
    signal s_px, s_px2 : unsigned(11 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    signal s_pxd : t_u8(1 to 3) := (others => (others => '0'));
    signal s_g1, s_g2, s_g3 : unsigned(7 downto 0) := (others => '0');
    signal s_y3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_t4 : unsigned(6 downto 0) := (others => '0');
    signal s_pool4 : std_logic := '0';
    signal s_y5, s_u5, s_v5 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_pool5, s_crest5 : std_logic := '0';
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y7, s_y8, s_y9, s_y10, s_y11, s_y12x, s_yf12 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u12, s_v12 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_yp : t_u10(13 to 20) := (others => to_unsigned(64, 10));
    signal s_up, s_vp : t_u10(13 to 20) := (others => to_unsigned(512, 10));

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');

begin

    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            s_lb_q <= lb(to_integer(s_lb_ra));
            if s_lb_we = '1' then lb(to_integer(s_lb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_lb;

    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
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

            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            s_fx <= resize(unsigned(registers_in(0)(9 downto 4)), 7) + 2;       -- K1 Pitch 2 .. 65
            s_pk <= unsigned(registers_in(1)(9 downto 2));                      -- K2 Pools
            s_pw <= resize(unsigned(registers_in(2)(9 downto 4)), 7) + 2;       -- K3 Piping 2 .. 65
            v_k := unsigned(registers_in(3));                                   -- K4 Hue (4 labels)
            if v_k >= 4 then s_live4 <= '1'; end if;
            if s_live4 = '0' and v_k < 4 then s_j4 <= v_k(1 downto 0); else s_j4 <= v_k(9 downto 8); end if;
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;         -- K5 Drift
            if v_s > -8 and v_s < 8 then s_k5r <= (others => '0'); else s_k5r <= v_s(9 downto 0); end if;
            s_tk <= unsigned(registers_in(5)(9 downto 2));                      -- K6 Tear
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_hue <= s_j4;
                s_pools  <= not registers_in(6)(0);
                s_piping <= not registers_in(6)(1);
                s_mirror <= registers_in(6)(2);
                s_light  <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                s_ph <= s_ph + unsigned(resize(shift_right(s_k5r, 4), 8));
            end if;

            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1);
                               s_fxs <= shift_left(resize(s_fx, 10), 2);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14);
                               s_fxs <= shift_left(resize(s_fx, 10), 1) + resize(s_fx, 10);
                when others => v_vw := resize(s_w, 14);
                               s_fxs <= shift_left(resize(s_fx, 10), 1);
            end case;
            s_vw <= v_vw(11 downto 0);
            s_fxs2 <= "00" & s_fxs(9 downto 2);
            s_amp <= s_g(11 downto 4);
            s_pwt <= to_unsigned(127, 7) - s_pw;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

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
    -- Source side: oscillator, line-buffer write, per-line fold phase
    -- (A tri(y) + drift + tear band) and the pool row term
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_t : unsigned(15 downto 0);
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

            if s_zone = '1' then
                s_py <= (others => '0');
                s_py2 <= (others => '0');
                s_l8 <= (others => '0');
                s_tr_n <= (others => '0');
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
            elsif s_fall1 = '1' then
                s_py <= s_py + resize(s_ys(11 downto 4), 12);
                s_py2 <= s_py2 + resize(s_ys(11 downto 6), 12);
                s_l8 <= s_l8 + 1;
                s_lfsr <= s_lfsr(14 downto 0) & (s_lfsr(15) xor s_lfsr(13) xor s_lfsr(12) xor s_lfsr(10));
                if s_l8 = 7 then
                    v_t := s_lfsr(7 downto 0) * s_tk;
                    s_tr_n <= v_t(15 downto 8);
                end if;
            end if;
            -- fold phase of the coming line (settles during hblank)
            s_cl_p <= s_amp * f_tri8(s_py(11 downto 4) + s_ph);
            s_cl_n <= s_cl_p(14 downto 7) + s_tr_n;
            s_tpy2 <= f_tri8(s_py2(9 downto 2));

            if data_in.avid = '1' and s_avq = '0' then
                s_cl <= s_cl_n;
            end if;
        end if;
    end process p_src;

    p_beam : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_rdp <= (others => '0');
                s_vxa <= (others => '0');
                s_px <= (others => '0');
                s_px2 <= (others => '0');
            elsif g_avid = '1' then
                s_rdp <= s_rdp + 1;
                s_vxa <= s_vxa + s_xs;
                if s_mirror = '1' and s_vxa >= resize(s_vw, 13) then s_px <= s_px - s_fxs; else s_px <= s_px + s_fxs; end if;
                s_px2 <= s_px2 + s_fxs2;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a : unsigned(7 downto 0);
        variable v_g : unsigned(8 downto 0);
        variable v_t3 : unsigned(9 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: read issued; fold x phase + line phase; pool field
            s_lb_ra <= s_rdp;
            s_pxd(1) <= s_px(11 downto 4) + s_cl;
            v_g := resize(f_tri8(s_px2(11 downto 4) + s_ph), 9) + resize(s_tpy2, 9);
            s_g1 <= v_g(8 downto 1);
            -- 2: EBR output; 3: fabric copy
            s_pxd(2) <= s_pxd(1); s_g2 <= s_g1;
            s_pxd(3) <= s_pxd(2); s_g3 <= s_g2;
            s_y3 <= unsigned(s_lb_q(31 downto 22));

            -- 4: fold value with the picture added to its phase; pool flag
            v_a := s_pxd(3) + s_y3(9 downto 2);
            s_t4 <= f_tri8(v_a);
            if s_g3 < s_pk then s_pool4 <= '1'; else s_pool4 <= '0'; end if;

            -- 5: fold colour by hue set (shading 3t), crest flag
            v_t3 := resize(s_t4, 10) + resize(s_t4 & '0', 10);         -- 3 t, 0 .. 381
            if s_light = '1' then s_y5 <= to_unsigned(680, 10) - v_t3; else s_y5 <= to_unsigned(280, 10) + v_t3; end if;
            case s_hue is
                when "00"   => s_u5 <= to_unsigned(960, 10) - resize(s_t4(6 downto 1), 10);                      -- red -> magenta
                               s_v5 <= to_unsigned(360, 10) + v_t3 + resize(s_t4(6 downto 1), 10);
                when "01"   => s_u5 <= to_unsigned(440, 10) - v_t3;                                              -- blue -> cyan
                               s_v5 <= to_unsigned(960, 10) - resize(s_t4 & '0', 10) - resize(s_t4(6 downto 2), 10);
                when "10"   => s_u5 <= to_unsigned(136, 10) + v_t3 + resize(s_t4(6 downto 1), 10);               -- green -> yellow
                               s_v5 <= to_unsigned(216, 10) - resize(s_t4, 10) - resize(s_t4(6 downto 2), 10);
                when others => s_u5 <= to_unsigned(756, 10) - resize(s_t4, 10);                                  -- violet -> pink
                               s_v5 <= to_unsigned(852, 10) - resize(s_t4 & '0', 10) - resize(s_t4(6 downto 1), 10);
            end case;
            if s_t4 >= s_pwt then s_crest5 <= s_piping; else s_crest5 <= '0'; end if;
            s_pool5 <= s_pool4 and s_pools;

            -- 6: compose: pool (blue) > crest (lime) > fold
            if s_pool5 = '1' then
                s_y6 <= to_unsigned(164, 10); s_u6 <= to_unsigned(440, 10); s_v6 <= to_unsigned(960, 10);
            elsif s_crest5 = '1' then
                s_y6 <= to_unsigned(804, 10); s_u6 <= to_unsigned(474, 10); s_v6 <= to_unsigned(184, 10);
            else
                s_y6 <= s_y5; s_u6 <= s_u5; s_v6 <= s_v5;
            end if;

            -- 7..12: output conditioning (centre at stage 11 -> 12)
            s_csu(0) <= s_u6; s_csv(0) <= s_v6;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u6 - s_csu(7);
            s_boxv <= s_boxv + s_v6 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u12 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v12 <= v_sc(13 downto 4);
            s_y7 <= s_y6; s_y8 <= s_y7; s_y9 <= s_y8; s_y10 <= s_y9;
            s_y11 <= s_y10; s_y12x <= s_y11;
            v_sy := resize(s_y10, 12) + shift_left(resize(s_y11, 12), 1) + resize(s_y12x, 12);
            s_yf12 <= v_sy(11 downto 2);
            -- 13: luma toward black; blanking gate
            if s_yf12 > 64 then v_dl := s_yf12 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(11) = '1' then
                s_yp(13) <= s_yf12 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_up(13) <= s_u12; s_vp(13) <= s_v12;
            else
                s_yp(13) <= to_unsigned(64, 10); s_up(13) <= to_unsigned(512, 10); s_vp(13) <= to_unsigned(512, 10);
            end if;
            for i in 14 to 20 loop s_yp(i) <= s_yp(i - 1); s_up(i) <= s_up(i - 1); s_vp(i) <= s_vp(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_yp(20));
    data_out.u       <= std_logic_vector(s_up(20));
    data_out.v       <= std_logic_vector(s_vp(20));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture curtain;
