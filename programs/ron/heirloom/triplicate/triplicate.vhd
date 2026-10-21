-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Triplicate v0.1.0 -- Heirloom #53 (2026-10-20), from selfie.jpg (a paper
-- collage).
--
-- The collage: a bear with a skull for a face and a home computer on its lap,
-- cut out three times in red, green and blue and cascaded diagonally over a
-- gray tiled wall of skulls, the full-colour original in front.
--
-- The program: the picture keyed by luma into a silhouette; three tinted
-- copies of the silhouette (red / green / blue, carrying the picture's luma
-- coarsely) sit behind the full-colour original, each offset further to the
-- right (P12), over a backdrop of the picture tiled small and turned gray.
--
--   K1  Key      luma threshold of the silhouette
--   K2  Tiles    backdrop tiles across, 2 .. 8
--   K3  Tint     copy saturation
--   K4  Wall     backdrop contrast (flat gray .. full)
--   K5  Detail   copy luma depth (flat silhouettes .. the picture's shading)
--   K6  Wobble   the offset breathes, bipolar rate (centre = still)
--   S7  Order    RGB (red furthest back) / BGR
--   S8  Backdrop Tiled / Plain (the picture itself, gray)
--   S9  Tints    RGB / CMY
--   S10 Key      Bright parts are the figure / Dark parts
--   S11 Source   Video / Osc
--   P12 SPREAD   copy offset, 1 .. 32 px
--
-- Two line buffers: the full line (2048 x 32) and a decimated copy for the
-- tiles (1024 x 32, escalator's write DDA).  The copies' key bit and 4-bit
-- luma run through a 96-deep shift register tapped at d, 2d, 3d.  Generated
-- avid; output conditioning before the gate.  Latency 20.  EBR 24.
-- Colours in the hardware convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture triplicate of program_top is
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
    function f_clamp(v : signed(12 downto 0)) return unsigned is
    begin
        if v < 64 then return to_unsigned(64, 10);
        elsif v > 940 then return to_unsigned(940, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;

    -- decimation fraction (Q.10) for 2 .. 8 tiles
    type t_frac is array (0 to 7) of integer;
    constant C_FRAC : t_frac := (512, 512, 341, 256, 205, 171, 146, 128);

    -- line buffers {y, u, v, 00}
    type t_lb is array (0 to 2047) of std_logic_vector(31 downto 0);
    type t_tb is array (0 to 1023) of std_logic_vector(31 downto 0);
    signal lb : t_lb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal tb : t_tb := (others => "0001000000" & "1000000000" & "1000000000" & "00");
    signal s_lb_wa, s_lb_ra : unsigned(10 downto 0) := (others => '0');
    signal s_lb_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s_lb_we : std_logic := '0';
    signal s_lb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";
    signal s_tb_wa, s_tb_ra : unsigned(9 downto 0) := (others => '0');
    signal s_tb_we : std_logic := '0';
    signal s_tb_q  : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    -- oscillator source
    signal s_src : std_logic := '0';
    signal s_ox : unsigned(9 downto 0) := (others => '0');
    signal s_oy : unsigned(9 downto 0) := (others => '0');
    signal s_ot : unsigned(7 downto 0) := (others => '0');
    signal s_oty, s_otu, s_otv : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_srcw : std_logic_vector(31 downto 0) := (others => '0');

    -- timing / controls
    signal s_xs : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_thr : unsigned(9 downto 0) := to_unsigned(400, 10);
    signal s_j2 : unsigned(2 downto 0) := to_unsigned(3, 3);
    signal s_k3, s_k4, s_k5 : unsigned(7 downto 0) := (others => '0');
    signal s_k6r : signed(9 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    -- field-latched / derived
    signal s_f : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal s_wt, s_wtm1 : unsigned(9 downto 0) := to_unsigned(480, 10);
    signal s_wtl : unsigned(9 downto 0) := to_unsigned(480, 10);
    signal s_d : unsigned(4 downto 0) := to_unsigned(8, 5);          -- offset - 1 (0..31)
    signal s_wph : unsigned(9 downto 0) := (others => '0');          -- wobble phase
    signal s_tint, s_wall, s_detail : unsigned(7 downto 0) := (others => '0');
    signal s_bgr, s_plain, s_cmy, s_dark : std_logic := '0';

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);

    -- source side
    signal s_wcol : unsigned(10 downto 0) := (others => '0');
    signal s_wacc : unsigned(10 downto 0) := (others => '0');
    signal s_wcnt : unsigned(9 downto 0) := (others => '0');

    -- beam state
    signal s_rdp : unsigned(10 downto 0) := (others => '0');
    signal s_r : unsigned(9 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u4 is array (natural range <>) of unsigned(3 downto 0);
    type t_sl is array (natural range <>) of std_logic;
    signal s_yf3, s_uf3, s_vf3, s_yt3, s_ut3, s_vt3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_key4 : std_logic := '0';
    signal s_l4 : unsigned(3 downto 0) := (others => '0');
    signal s_yf4, s_uf4, s_vf4, s_yb4, s_ub4, s_vb4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ksr : std_logic_vector(96 downto 1) := (others => '0');
    signal s_lsr : t_u4(1 to 96) := (others => (others => '0'));
    signal s_k0, s_k1, s_k2, s_k3b : std_logic := '0';
    signal s_l0, s_l1, s_l2, s_l3 : unsigned(3 downto 0) := (others => '0');
    signal s_yf5, s_uf5, s_vf5, s_yb5, s_ub5, s_vb5 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_orig6, s_copy6 : std_logic := '0';
    signal s_lc6 : unsigned(3 downto 0) := (others => '0');
    signal s_tu6, s_tv6 : signed(10 downto 0) := (others => '0');
    signal s_yf6, s_uf6, s_vf6, s_yb6, s_ub6, s_vb6 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_orig7, s_copy7 : std_logic := '0';
    signal s_pyb7 : unsigned(17 downto 0) := (others => '0');
    signal s_plc7 : unsigned(11 downto 0) := (others => '0');
    signal s_ptu7, s_ptv7 : signed(19 downto 0) := (others => '0');
    signal s_yf7, s_uf7, s_vf7 : unsigned(9 downto 0) := to_unsigned(64, 10);
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

    p_tb : process(clk)
    begin
        if rising_edge(clk) then
            s_tb_q <= tb(to_integer(s_tb_ra));
            if s_tb_we = '1' then tb(to_integer(s_tb_wa)) <= s_lb_wd; end if;
        end if;
    end process p_tb;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_d : unsigned(10 downto 0);
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

            v_k := unsigned(registers_in(0));                                  -- K1 Key: 96 .. 991
            s_thr <= to_unsigned(96, 10) + v_k - resize(v_k(9 downto 3), 10);
            v_k := unsigned(registers_in(1));                                  -- K2 Tiles 2 .. 8
            if s_warm /= 15 or ((v_k(6 downto 0) >= 6 or v_k(9 downto 7) = 0) and
                                (v_k(6 downto 0) <= 121 or v_k(9 downto 7) = 7)) then
                s_j2 <= v_k(9 downto 7);
            end if;
            s_k3 <= unsigned(registers_in(2)(9 downto 2));                     -- K3 Tint
            s_k4 <= unsigned(registers_in(3)(9 downto 2));                     -- K4 Wall
            s_k5 <= unsigned(registers_in(4)(9 downto 2));                     -- K5 Detail
            v_s := signed(resize(unsigned(registers_in(5)), 11)) - 512;        -- K6 Wobble, deadband
            v_s := shift_right(v_s, 3);
            if v_s > -1 and v_s < 1 then s_k6r <= (others => '0'); else s_k6r <= v_s(9 downto 0); end if;
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                if s_wtl /= 0 then s_wt <= s_wtl; end if;
                s_f <= to_unsigned(C_FRAC(to_integer(s_j2)), 11);
                s_tint <= s_k3; s_wall <= s_k4; s_detail <= s_k5;
                s_wph <= s_wph + unsigned(resize(s_k6r, 10));
                s_bgr   <= registers_in(6)(0);
                s_plain <= registers_in(6)(1);
                s_cmy   <= registers_in(6)(2);
                s_dark  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                -- offset: p12 (1..32) breathing by the wobble triangle (+-8)
                v_d := resize(s_g(11 downto 7), 11) + resize(f_tri8(s_wph(9 downto 2))(6 downto 3), 11);
                if v_d > 31 then s_d <= (others => '1'); else s_d <= v_d(4 downto 0); end if;
            end if;

            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            s_wtm1 <= s_wt - 1;

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
    -- Source side: full-line write every pixel, decimated tile write
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_a : unsigned(11 downto 0);
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

            -- decimated write (same data word, one in every 1024/f pixels)
            s_tb_we <= '0';
            if data_in.avid = '1' then
                v_a := resize(s_wacc, 12) + resize(s_f, 12);
                if v_a >= 1024 then
                    s_wacc <= resize(v_a - 1024, 11);
                    s_tb_we <= '1';
                    s_tb_wa <= s_wcnt;
                    s_wcnt <= s_wcnt + 1;
                else
                    s_wacc <= v_a(10 downto 0);
                end if;
            else
                if s_avq = '1' then s_wtl <= s_wcnt; end if;
                s_wacc <= (others => '0');
                s_wcnt <= (others => '0');
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
                s_r <= (others => '0');
            elsif g_avid = '1' then
                s_rdp <= s_rdp + 1;
                if s_r = s_wtm1 then s_r <= (others => '0'); else s_r <= s_r + 1; end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_y : signed(12 downto 0);
        variable v_u, v_v : signed(12 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
        variable v_k1, v_k2, v_k3 : std_logic;
        variable v_l1, v_l2, v_l3 : unsigned(3 downto 0);
        variable v_tu, v_tv : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: read addresses
            s_lb_ra <= s_rdp;
            s_tb_ra <= s_r;

            -- 2: EBR output registers; 3: fabric copies
            s_yf3 <= unsigned(s_lb_q(31 downto 22)); s_uf3 <= unsigned(s_lb_q(21 downto 12)); s_vf3 <= unsigned(s_lb_q(11 downto 2));
            s_yt3 <= unsigned(s_tb_q(31 downto 22)); s_ut3 <= unsigned(s_tb_q(21 downto 12)); s_vt3 <= unsigned(s_tb_q(11 downto 2));

            -- 4: key + coarse luma; backdrop pick; push the shift register
            if s_yf3 >= s_thr then s_key4 <= not s_dark; else s_key4 <= s_dark; end if;
            s_l4 <= s_yf3(9 downto 6);
            s_yf4 <= s_yf3; s_uf4 <= s_uf3; s_vf4 <= s_vf3;
            if s_plain = '1' then
                s_yb4 <= s_yf3; s_ub4 <= s_uf3; s_vb4 <= s_vf3;
            else
                s_yb4 <= s_yt3; s_ub4 <= s_ut3; s_vb4 <= s_vt3;
            end if;
            s_ksr <= s_ksr(95 downto 1) & s_key4;
            s_lsr(1) <= s_l4;
            for i in 2 to 96 loop s_lsr(i) <= s_lsr(i - 1); end loop;

            -- 5: taps at d, 2d, 3d (d = s_d + 1)
            s_k0 <= s_key4; s_l0 <= s_l4;
            v_k1 := s_ksr(to_integer(s_d) + 1);
            v_l1 := s_lsr(to_integer(s_d) + 1);
            v_k2 := s_ksr(2 * to_integer(s_d) + 2);
            v_l2 := s_lsr(2 * to_integer(s_d) + 2);
            v_k3 := s_ksr(3 * to_integer(s_d) + 3);
            v_l3 := s_lsr(3 * to_integer(s_d) + 3);
            s_k1 <= v_k1; s_l1 <= v_l1;
            s_k2 <= v_k2; s_l2 <= v_l2;
            s_k3b <= v_k3; s_l3 <= v_l3;
            s_yf5 <= s_yf4; s_uf5 <= s_uf4; s_vf5 <= s_vf4;
            s_yb5 <= s_yb4; s_ub5 <= s_ub4; s_vb5 <= s_vb4;

            -- 6: layer select (front to back: original, copy 1, 2, 3)
            s_orig6 <= s_k0;
            s_copy6 <= s_k1 or s_k2 or s_k3b;
            if s_k1 = '1' then
                s_lc6 <= s_l1;
                if s_bgr = '1' then v_tu := to_signed(960 - 512, 11); v_tv := to_signed(360 - 512, 11);      -- red
                else v_tu := to_signed(440 - 512, 11); v_tv := to_signed(960 - 512, 11); end if;             -- blue
            elsif s_k2 = '1' then
                s_lc6 <= s_l2;
                v_tu := to_signed(136 - 512, 11); v_tv := to_signed(216 - 512, 11);                           -- green
            else
                s_lc6 <= s_l3;
                if s_bgr = '1' then v_tu := to_signed(440 - 512, 11); v_tv := to_signed(960 - 512, 11);
                else v_tu := to_signed(960 - 512, 11); v_tv := to_signed(360 - 512, 11); end if;
            end if;
            if s_cmy = '1' then v_tu := -v_tu; v_tv := -v_tv; end if;      -- complement hues
            s_tu6 <= v_tu; s_tv6 <= v_tv;
            s_yf6 <= s_yf5; s_uf6 <= s_uf5; s_vf6 <= s_vf5;
            s_yb6 <= s_yb5; s_ub6 <= s_ub5; s_vb6 <= s_vb5;

            -- 7: products
            s_orig7 <= s_orig6; s_copy7 <= s_copy6;
            if s_yb6 > 64 then s_pyb7 <= (s_yb6 - 64) * s_wall; else s_pyb7 <= (others => '0'); end if;
            s_plc7 <= s_lc6 * s_detail;
            s_ptu7 <= s_tu6 * signed(resize(s_tint, 9));
            s_ptv7 <= s_tv6 * signed(resize(s_tint, 9));
            s_yf7 <= s_yf6; s_uf7 <= s_uf6; s_vf7 <= s_vf6;

            -- 8: compose
            if s_orig7 = '1' then
                s_y8 <= s_yf7; s_u8 <= s_uf7; s_v8 <= s_vf7;
            elsif s_copy7 = '1' then
                v_y := to_signed(128, 13) + signed(resize(s_plc7(11 downto 2), 13));
                v_u := to_signed(512, 13) + resize(shift_right(s_ptu7, 8), 13);
                v_v := to_signed(512, 13) + resize(shift_right(s_ptv7, 8), 13);
                s_y8 <= f_clamp(v_y); s_u8 <= f_clamp(v_u); s_v8 <= f_clamp(v_v);
            else
                v_y := to_signed(96, 13) + signed(resize(s_pyb7(17 downto 8), 13));
                s_y8 <= f_clamp(v_y); s_u8 <= to_unsigned(512, 10); s_v8 <= to_unsigned(512, 10);
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
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y20);
    data_out.u       <= std_logic_vector(s_u20);
    data_out.v       <= std_logic_vector(s_v20);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture triplicate;
