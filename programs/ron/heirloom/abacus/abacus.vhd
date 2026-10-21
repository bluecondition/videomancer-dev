-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Abacus v0.1.0 -- Heirloom #42 (2026-10-20), from WIN_20230605_01_03_50.
--
-- The photo: rows of glossy red ovals (beads) on a dark ground, rows of blue
-- and green blobs between them, every blob lit from above with a chrome rim
-- -- an oscillator product keyed and relief-shaded.
--
-- The program: a grid of cells, N across and M rows; in each cell a bead
-- (r^2 from the cell centre under a radius set by P12) coloured by its row
-- from an 8-colour palette, lit from the top (luma falls with y inside the
-- bead) with a white rim at the edge.  The bead's luma is the picture's.
--
--   K1  Cells    beads across, 1 .. 16
--   K2  Rows     bead rows, 1 .. 8
--   K3  Shade    lighting gradient, top bright .. bottom dark
--   K4  Hue      rotates the row colours
--   K5  Drift    beads slide sideways, bipolar
--   K6  Rim      rim width
--   S7  Mirror   row colours mirror about the middle row
--   S8  Palette  RGB cycle / Warm cycle
--   S9  Rim      Off / On
--   S10 Shape    Beads / Tubes (rows of horizontal tubes)
--   S11 Source   Video / Osc
--   P12 BEADS    bead size: dots .. beads that merge into tubes
--
-- Geometry in virtual 1080-line units.  Cell width from one serial divide
-- per frame; dx^2 by second differences, dy^2 per line on the serial
-- multiplier.  Generated avid; output conditioning before the blanking gate.
-- Latency 16 clocks.  EBR: 16 line buffer.  Colours in the hardware
-- convention (u = Cr, v = Cb).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture abacus of program_top is
    constant C_LAT : integer := 16;

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

    type t_ch is array (0 to 7) of integer;
    constant C_CH : t_ch := (1080, 540, 360, 270, 216, 180, 154, 135);

    -- row colours {y, u = Cr, v = Cb}: 0..7 RGB cycle, 8..15 warm cycle
    type t_col is array (0 to 2) of integer;
    type t_pdef is array (0 to 15) of t_col;
    constant C_PDEF : t_pdef := (
        (328, 960, 360), (164, 440, 960), (580, 136, 216), (426, 887, 808), (678, 64, 663), (840, 584, 64), (349, 756, 852), (584, 772, 212),
        (328, 960, 360), (584, 772, 212), (840, 584, 64), (328, 960, 360), (426, 887, 808), (584, 772, 212), (752, 612, 512), (328, 960, 360));

    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

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
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w, s_wm1 : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vw : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_j1 : unsigned(3 downto 0) := to_unsigned(7, 4);
    signal s_j2 : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_shk : unsigned(7 downto 0) := (others => '0');
    signal s_hue : unsigned(2 downto 0) := (others => '0');
    signal s_spd : signed(5 downto 0) := (others => '0');
    signal s_rimk : unsigned(7 downto 0) := (others => '0');
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');
    signal s_mirror, s_palb, s_rim, s_tubes : std_logic := '0';
    -- field-latched / derived
    signal s_n : unsigned(3 downto 0) := to_unsigned(7, 4);
    signal s_m : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_cwv : unsigned(11 downto 0) := to_unsigned(240, 12);
    signal s_cwv2 : unsigned(12 downto 0) := to_unsigned(480, 13);
    signal s_ch : unsigned(10 downto 0) := to_unsigned(360, 11);
    signal s_chq : unsigned(19 downto 0) := (others => '0');
    signal s_chh : unsigned(10 downto 0) := to_unsigned(180, 11);
    signal s_hm : unsigned(10 downto 0) := to_unsigned(120, 11);      -- half of min(cw, ch)
    signal s_r : unsigned(10 downto 0) := to_unsigned(60, 11);
    signal s_r2 : unsigned(21 downto 0) := (others => '0');
    signal s_rimq : unsigned(21 downto 0) := (others => '0');
    signal s_rin : unsigned(21 downto 0) := (others => '0');          -- R^2 - rim
    signal s_ph : unsigned(12 downto 0) := (others => '0');           -- drift phase (Q.1)
    signal s_dxh0 : signed(13 downto 0) := (others => '0');           -- first cell start offset
    signal s_q0, s_q0s : unsigned(23 downto 0) := (others => '0');
    signal s_inc0, s_inc0s : signed(15 downto 0) := (others => '0');
    signal s_inc2 : unsigned(5 downto 0) := to_unsigned(8, 6);
    -- serial units
    signal s_dnum : unsigned(11 downto 0) := (others => '0');
    signal s_drem : unsigned(12 downto 0) := (others => '0');
    signal s_dquo : unsigned(11 downto 0) := (others => '0');
    signal s_dden : unsigned(4 downto 0) := to_unsigned(8, 5);
    signal s_di : unsigned(3 downto 0) := (others => '0');
    signal s_drun : std_logic := '0';
    signal s_ma : unsigned(23 downto 0) := (others => '0');
    signal s_mb : unsigned(11 downto 0) := (others => '0');
    signal s_macc : unsigned(23 downto 0) := (others => '0');
    signal s_mi : unsigned(3 downto 0) := (others => '0');
    signal s_mrun : std_logic := '0';
    signal s_mjob : unsigned(2 downto 0) := (others => '0');

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
    signal s_lc : unsigned(6 downto 0) := (others => '1');
    signal s_cv_n : unsigned(19 downto 0) := (others => '0');
    signal s_row_n : unsigned(2 downto 0) := (others => '0');
    signal s_ady : unsigned(10 downto 0) := (others => '0');
    signal s_dyn : signed(11 downto 0) := (others => '0');
    signal s_dy2_n, s_dy2 : unsigned(21 downto 0) := (others => '0');
    signal s_term_n, s_term : signed(20 downto 0) := (others => '0');
    signal s_ry, s_ru, s_rv : unsigned(9 downto 0) := to_unsigned(328, 10);

    -- beam state
    signal s_col : unsigned(10 downto 0) := (others => '0');
    signal s_cu : unsigned(12 downto 0) := (others => '0');
    signal s_q : unsigned(23 downto 0) := (others => '0');
    signal s_inc : signed(15 downto 0) := (others => '0');

    -- pixel path
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_qa1 : unsigned(21 downto 0) := (others => '0');
    signal s_rr2 : unsigned(22 downto 0) := (others => '0');
    signal s_in3, s_rim3 : std_logic := '0';
    signal s_y3 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_y4, s_u4, s_v4 : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y5, s_y6, s_y7, s_y8, s_y9, s_y10x, s_yf10 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u10, s_v10 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y11, s_y12, s_y13, s_y14, s_y15, s_y16 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u11, s_v11, s_u12, s_v12, s_u13, s_v13, s_u14, s_v14, s_u15, s_v15, s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);

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

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches, serial divide / products
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_s : signed(10 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_vw : unsigned(13 downto 0);
        variable v_r : unsigned(12 downto 0);
        variable v_p : signed(14 downto 0);
        variable v_i : signed(16 downto 0);
        variable v_st : std_logic;
        variable v_jb : unsigned(2 downto 0);
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

            -- K1 Cells (16 zones), K2 Rows (8 zones), deadbands after warm-up
            v_k := unsigned(registers_in(0));
            if s_warm /= 15 or ((v_k(5 downto 0) >= 3 or v_k(9 downto 6) = 0) and
                                (v_k(5 downto 0) <= 60 or v_k(9 downto 6) = 15)) then
                s_j1 <= v_k(9 downto 6);
            end if;
            v_k := unsigned(registers_in(1));
            if s_warm /= 15 or ((v_k(6 downto 0) >= 6 or v_k(9 downto 7) = 0) and
                                (v_k(6 downto 0) <= 121 or v_k(9 downto 7) = 7)) then
                s_j2 <= v_k(9 downto 7);
            end if;
            s_shk <= unsigned(registers_in(2)(9 downto 2));                    -- K3 Shade
            s_hue <= unsigned(registers_in(3)(9 downto 7));                    -- K4 Hue (8 steps)
            v_s := signed(resize(unsigned(registers_in(4)), 11)) - 512;        -- K5 Drift, deadband
            v_s := shift_right(v_s, 4);
            if v_s > -1 and v_s < 1 then s_spd <= (others => '0'); else s_spd <= v_s(5 downto 0); end if;
            s_rimk <= unsigned(registers_in(5)(9 downto 2));                   -- K6 Rim
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_n <= s_j1;
                s_m <= s_j2;
                s_mirror <= registers_in(6)(0);
                s_palb   <= registers_in(6)(1);
                s_rim    <= registers_in(6)(2);
                s_tubes  <= registers_in(6)(3);
                s_src    <= registers_in(6)(4);
                -- drift phase in Q.1 cell units, wrapped to the cell
                v_p := signed(resize(s_ph, 15)) + resize(s_spd, 15);
                if v_p < 0 then v_p := v_p + signed(resize(s_cwv2, 15));
                elsif v_p >= signed(resize(s_cwv2, 15)) then v_p := v_p - signed(resize(s_cwv2, 15)); end if;
                s_ph <= unsigned(v_p(12 downto 0));
            end if;

            -- derived
            s_wm1 <= s_w - 1;
            g_wm1 <= s_w - 1;
            case to_integer(s_xs) is
                when 4      => v_vw := shift_left(resize(s_w, 14), 1); s_inc2 <= to_unsigned(32, 6);
                when 3      => v_vw := resize(s_w, 14) + resize(s_w(11 downto 1), 14); s_inc2 <= to_unsigned(18, 6);
                when others => v_vw := resize(s_w, 14); s_inc2 <= to_unsigned(8, 6);
            end case;
            s_vw <= v_vw(11 downto 0);
            s_cwv2 <= s_cwv & '0';
            s_ch <= to_unsigned(C_CH(to_integer(s_m)), 11);
            s_chq <= resize(s_ch, 12) & x"00";
            s_chh <= '0' & s_ch(10 downto 1);
            if s_cwv < resize(s_ch, 12) then s_hm <= s_cwv(11 downto 1); else s_hm <= '0' & s_ch(10 downto 1); end if;
            s_rin <= s_r2 - s_rimq;
            -- cell-start constants: a cell boundary (dxh = -cw) and the first
            -- partial cell of a line (dxh0 = ph - cw)
            s_dxh0 <= signed(resize(s_ph, 14)) - signed(resize(s_cwv, 14));
            case to_integer(s_xs) is
                when 4      => v_i := to_signed(16, 17) - signed(shift_left(resize(s_cwv, 17), 3));
                               s_inc0s <= resize(to_signed(16, 17) + shift_left(resize(s_dxh0, 17), 3), 16);
                when 3      => v_i := to_signed(9, 17) - signed(shift_left(resize(s_cwv, 17), 2) + shift_left(resize(s_cwv, 17), 1));
                               s_inc0s <= resize(to_signed(9, 17) + shift_left(resize(s_dxh0, 17), 2) + shift_left(resize(s_dxh0, 17), 1), 16);
                when others => v_i := to_signed(4, 17) - signed(shift_left(resize(s_cwv, 17), 2));
                               s_inc0s <= resize(to_signed(4, 17) + shift_left(resize(s_dxh0, 17), 2), 16);
            end case;
            s_inc0 <= v_i(15 downto 0);

            -- serial divide: cwv = vw / N (zone clocks 4 .. 16)
            if s_zc = 4 then
                s_dnum <= s_vw; s_drem <= (others => '0'); s_dden <= resize(s_n, 5) + 1;
                s_drun <= '1'; s_di <= (others => '0');
            elsif s_drun = '1' then
                v_r := s_drem(11 downto 0) & s_dnum(11);
                s_dnum <= s_dnum(10 downto 0) & '0';
                if v_r >= resize(s_dden, 13) then
                    s_drem <= v_r - resize(s_dden, 13);
                    s_dquo <= s_dquo(10 downto 0) & '1';
                else
                    s_drem <= v_r;
                    s_dquo <= s_dquo(10 downto 0) & '0';
                end if;
                if s_di = 11 then s_drun <= '0'; else s_di <= s_di + 1; end if;
            elsif s_di = 11 then
                s_di <= (others => '0');
                s_cwv <= s_dquo;
            end if;

            -- serial products: zone 0 R = hm x p12 / 1024, 1 R^2, 2 rim = R^2/256 x k6,
            -- 3 cw^2, 4 |dxh0|^2; per line 5 dy^2
            v_st := '0'; v_jb := (others => '0');
            if s_zone = '1' then
                case to_integer(s_zc) is
                    when 24  => v_st := '1'; v_jb := "000";
                    when 40  => v_st := '1'; v_jb := "001";
                    when 56  => v_st := '1'; v_jb := "010";
                    when 72  => v_st := '1'; v_jb := "011";
                    when 88  => v_st := '1'; v_jb := "100";
                    when 104 => v_st := '1'; v_jb := "101";
                    when others => null;
                end case;
            elsif s_lc = 2 then
                v_st := '1'; v_jb := "101";
            end if;
            if v_st = '1' then
                s_mjob <= v_jb; s_mrun <= '1'; s_mi <= (others => '0'); s_macc <= (others => '0');
                case to_integer(v_jb) is
                    when 0 => s_ma <= resize(s_hm, 24); s_mb <= resize(s_g(11 downto 2), 12);
                    when 1 => s_ma <= resize(s_r, 24); s_mb <= resize(s_r, 12);
                    when 2 => s_ma <= resize(s_r2(21 downto 8), 24); s_mb <= resize(s_rimk, 12);
                    when 3 => s_ma <= resize(s_cwv, 24); s_mb <= s_cwv;
                    when 4 => if s_dxh0 < 0 then s_ma <= resize(unsigned(-s_dxh0), 24); s_mb <= resize(unsigned(-s_dxh0), 12);
                              else s_ma <= resize(unsigned(s_dxh0), 24); s_mb <= resize(unsigned(s_dxh0), 12); end if;
                    when others => s_ma <= resize(s_ady, 24); s_mb <= resize(s_ady, 12);
                end case;
            elsif s_mrun = '1' then
                if s_mb(0) = '1' then s_macc <= s_macc + s_ma; end if;
                s_ma <= s_ma(22 downto 0) & '0';
                s_mb <= '0' & s_mb(11 downto 1);
                if s_mi = 11 then s_mrun <= '0'; else s_mi <= s_mi + 1; end if;
            elsif s_mi = 11 then
                s_mi <= (others => '0');
                case to_integer(s_mjob) is
                    when 0 => s_r <= resize(s_macc(20 downto 10), 11) + 4;
                    when 1 => s_r2 <= s_macc(21 downto 0);
                    when 2 => s_rimq <= s_macc(21 downto 0);
                    when 3 => s_q0 <= s_macc;
                    when 4 => s_q0s <= s_macc;
                    when others => s_dy2_n <= s_macc(21 downto 0);
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
    -- Source side: line-buffer write; row DDA, |dy|, shading term and row
    -- colour per line
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_c : unsigned(20 downto 0);
        variable v_d : signed(12 downto 0);
        variable v_rw : unsigned(2 downto 0);
        variable v_mh : unsigned(2 downto 0);
        variable v_ci : unsigned(3 downto 0);
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
            if s_fall1 = '1' then s_lc <= (others => '0'); elsif s_lc /= 127 then s_lc <= s_lc + 1; end if;

            if s_zone = '1' then
                s_cv_n <= (others => '0');
                s_row_n <= (others => '0');
            elsif s_fall1 = '1' then
                v_c := resize(s_cv_n, 21) + s_ys;
                if v_c >= s_chq then
                    s_cv_n <= resize(v_c - s_chq, 20);
                    s_row_n <= s_row_n + 1;
                else
                    s_cv_n <= v_c(19 downto 0);
                end if;
            end if;
            -- dy, |dy|, shading term (free-running from the next line's row position)
            v_d := signed(resize(s_cv_n(19 downto 8), 13)) - signed(resize(s_chh, 13));
            s_dyn <= v_d(11 downto 0);
            if v_d < 0 then s_ady <= resize(unsigned(-v_d), 11); else s_ady <= unsigned(v_d(10 downto 0)); end if;
            s_term_n <= s_dyn * signed(resize(s_shk, 9));
            -- row colour: mirrored row index, hue rotation, palette half
            v_rw := s_row_n;
            v_mh := s_m;
            if s_mirror = '1' and (resize(v_rw, 4) & '0') > resize(v_mh, 5) then v_rw := v_mh - v_rw; end if;
            v_ci := s_palb & (v_rw + s_hue);

            if data_in.avid = '1' and s_avq = '0' then
                s_dy2 <= s_dy2_n;
                s_term <= s_term_n;
                s_ry <= to_unsigned(C_PDEF(to_integer(v_ci))(0), 10);
                s_ru <= to_unsigned(C_PDEF(to_integer(v_ci))(1), 10);
                s_rv <= to_unsigned(C_PDEF(to_integer(v_ci))(2), 10);
            end if;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Beam state: column, cell x (Q.1), dx^2 by second differences
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_n : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                s_col <= (others => '0');
                s_cu <= s_ph;
                s_q <= s_q0s;
                s_inc <= s_inc0s;
            elsif g_avid = '1' then
                s_col <= s_col + 1;
                v_n := resize(s_cu, 14) + s_xs;
                if v_n >= s_cwv2 then
                    s_cu <= resize(v_n - s_cwv2, 13);
                    s_q <= s_q0;
                    s_inc <= s_inc0;
                else
                    s_cu <= v_n(12 downto 0);
                    s_q <= unsigned(signed(s_q) + resize(s_inc, 24));
                    s_inc <= s_inc + signed(resize(s_inc2, 16));
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_y : signed(12 downto 0);
        variable v_sy : unsigned(11 downto 0);
        variable v_sc : unsigned(14 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: dx^2 (virtual); video read
            s_qa1 <= s_q(23 downto 2);
            s_lb_ra <= s_col;

            -- 2: r^2 (tubes: |dy| only)
            if s_tubes = '1' then s_rr2 <= resize(s_dy2, 23);
            else s_rr2 <= resize(s_qa1, 23) + resize(s_dy2, 23); end if;

            -- 3: inside / rim tests; video (fabric copy)
            if s_rr2 < resize(s_r2, 23) then s_in3 <= '1'; else s_in3 <= '0'; end if;
            if s_rr2 >= resize(s_rin, 23) then s_rim3 <= '1'; else s_rim3 <= '0'; end if;
            s_y3 <= unsigned(s_lb_q(31 downto 22));

            -- 4: compose: rim white, bead = picture luma lit from above in the
            --    row colour, ground dark
            if s_in3 = '1' then
                if s_rim = '1' and s_rim3 = '1' then
                    s_y4 <= to_unsigned(900, 10); s_u4 <= to_unsigned(512, 10); s_v4 <= to_unsigned(512, 10);
                else
                    v_y := signed(resize(s_y3, 13)) + 192 - resize(shift_right(s_term, 8), 13);
                    if v_y < 64 then v_y := to_signed(64, 13); elsif v_y > 940 then v_y := to_signed(940, 13); end if;
                    s_y4 <= unsigned(v_y(9 downto 0));
                    s_u4 <= s_ru; s_v4 <= s_rv;
                end if;
            else
                s_y4 <= to_unsigned(84, 10); s_u4 <= to_unsigned(512, 10); s_v4 <= to_unsigned(512, 10);
            end if;

            -- 5..10: output conditioning (centre at stage 9 -> 10)
            s_csu(0) <= s_u4; s_csv(0) <= s_v4;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u4 - s_csu(7);
            s_boxv <= s_boxv + s_v4 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u10 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v10 <= v_sc(13 downto 4);
            s_y5 <= s_y4; s_y6 <= s_y5; s_y7 <= s_y6; s_y8 <= s_y7;
            s_y9 <= s_y8; s_y10x <= s_y9;
            v_sy := resize(s_y8, 12) + shift_left(resize(s_y9, 12), 1) + resize(s_y10x, 12);
            s_yf10 <= v_sy(11 downto 2);
            -- 11: luma toward black; blanking gate
            if s_yf10 > 64 then v_dl := s_yf10 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(9) = '1' then
                s_y11 <= s_yf10 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u11 <= s_u10; s_v11 <= s_v10;
            else
                s_y11 <= to_unsigned(64, 10); s_u11 <= to_unsigned(512, 10); s_v11 <= to_unsigned(512, 10);
            end if;
            -- 12..16: pad
            s_y12 <= s_y11; s_u12 <= s_u11; s_v12 <= s_v11;
            s_y13 <= s_y12; s_u13 <= s_u12; s_v13 <= s_v12;
            s_y14 <= s_y13; s_u14 <= s_u13; s_v14 <= s_v13;
            s_y15 <= s_y14; s_u15 <= s_u14; s_v15 <= s_v14;
            s_y16 <= s_y15; s_u16 <= s_u15; s_v16 <= s_v15;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y16);
    data_out.u       <= std_logic_vector(s_u16);
    data_out.v       <= std_logic_vector(s_v16);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture abacus;
