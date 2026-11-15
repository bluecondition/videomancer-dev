-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- Lenticular v0.1.0: catalogue combo (2026-10-27) from lesser-used catalogue
-- entries -- "depth-from-luma -> parallax" (two combos) and "masking"
-- (every masking combo was removed), seen through a lenticular lens array
-- ("lens distortion", one combo).  The picture becomes N views, each shifted
-- sideways by its depth (taken from its smoothed luma), interleaved behind
-- vertical ridges like a 3-D postcard.  Tilting the card (P12) changes which
-- view each ridge shows: near things slide against the background, a Flip
-- card winks between the picture and its negative, a Holo card shimmers.
--
--   K1 Pitch   lenticule width 4 .. 48 px on the 1080-line canvas (log)
--   K2 Views   2 .. 8 interleaved views
--   K3 Sheen   ridge shading: flat .. glossy cylinders with dark grooves
--   K4 Depth   parallax: flat .. near things travel 48 px across the views
--   K5 Spread  lens crosstalk: one view per ridge .. the raw interleave
--   K6 Card    3D (parallax views) / Flip (odd views negative) / Holo
--              (holographic rainbow sticker) / Pop (odd views posterized)
--   S7  Near   Light (bright things pop forward) / Dark
--   S8  Wobble the card rocks by itself (a wigglegram)
--   S9  Slant  Vertical / Diagonal lenticules
--   S10 Bands  the viewing angle changes across the card: view zones
--   S11 Ghost  neighbouring views bleed through at the transitions
--   P12 Tilt   sweeps the viewing angle across all the views.  Glided.
--
-- Pixel path.  Ridge phase = a Q12 DDA per pixel (plus a per-line offset
-- for Slant).  View coordinate (Q8 views) = tilt + wobble + band (a DDA
-- across the virtual width) + spread x ridge phase; view = its integer part
-- mod N (two compare-subtract stages).  Depth = 16-px luma box, smoothed
-- down each column by an IIR in a column RAM.  Shift = view x step x depth
-- (one-sided: views only read the past, so the latency stays short); the
-- picture is read from a 256-word EBR ring at that offset (a second ring for
-- the neighbouring view when Ghost is on).  Card transform, ghost blend,
-- ridge sheen (cosine ROM x Sheen), compose.
--
-- Hardware contracts: vblank zone (no avid for 1024 clocks) for per-field
-- events; generated chroma (Holo) swaps U/V on lines off the voted
-- hsync->avid parity; output conditioning (luma [1 2 1], chroma
-- [1 2 2 2 2 2 2 2 1]/16, luma x13/16); blanking gate; no_rw_check on every
-- RAM.  Latency 24 clocks, all modes.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture lenticular of program_top is
    -- data_in -> data_out latency (multiple of 4); matches processing_delay_clks.
    constant C_PROCESSING_DELAY_CLKS : natural := 24;
    constant C_LAT : integer := 24;

    -- K1: 4096 / pitch for pitch 4 .. 48 virtual px (log)
    type t_i16 is array (0 to 15) of integer range 0 to 4095;
    constant C_INC : t_i16 := (1024, 819, 683, 585, 512, 455, 372, 315, 273, 228, 195, 164, 141, 120, 100, 85);
    -- K2: views 2 .. 8 over 16 zones; 256 / (N - 1)
    type t_n16 is array (0 to 15) of integer range 0 to 15;
    constant C_NV : t_n16 := (2, 2, 3, 3, 3, 4, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8);
    type t_r8 is array (0 to 8) of integer range 0 to 255;
    constant C_RCP : t_r8 := (0, 0, 255, 128, 85, 64, 51, 43, 37);
    -- one ridge of cosine, signed: 127 cos(2 pi (i / 64 - 1/2)) (dark groove at the edges)
    type t_c64 is array (0 to 63) of integer range -128 to 127;
    constant C_COS : t_c64 := (
        -127, -126, -125, -122, -117, -112, -106, -98, -90, -81, -71, -60, -49, -37, -25, -12,
        0, 12, 25, 37, 49, 60, 71, 81, 90, 98, 106, 112, 117, 122, 125, 126,
        127, 126, 125, 122, 117, 112, 106, 98, 90, 81, 71, 60, 49, 37, 25, 12,
        0, -12, -25, -37, -49, -60, -71, -81, -90, -98, -106, -112, -117, -122, -125, -126);
    -- Holo hues {U, V}, U/V SWAPPED for the hardware (Fissure's prism table)
    function f_h(u, v : integer) return std_logic_vector is
    begin
        return std_logic_vector(to_unsigned(u, 10)) & std_logic_vector(to_unsigned(v, 10));
    end function;
    type t_prism is array (0 to 15) of std_logic_vector(19 downto 0);
    constant C_PRISM : t_prism := (
        f_h(841, 401), f_h(767, 289), f_h(654, 210), f_h(519, 178), f_h(384, 196), f_h(268, 263), f_h(189, 367), f_h(159, 494),
        f_h(183, 623), f_h(257, 735), f_h(370, 814), f_h(505, 846), f_h(640, 828), f_h(756, 761), f_h(835, 657), f_h(865, 530));

    -- x step per pixel (Q.1 virtual units: 2 / 3 / 4 = 1080 / 720 / SD)
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- y step in Q.8 virtual units per FIELD line
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

    constant C_BLACK : std_logic_vector(31 downto 0) := "0001000000" & "1000000000" & "1000000000" & "00";

    -- RAMs: two picture rings (main view, neighbouring view) and the depth column
    type t_ring is array (0 to 255) of std_logic_vector(31 downto 0);
    type t_col  is array (0 to 2047) of std_logic_vector(7 downto 0);
    signal ringa, ringb : t_ring := (others => C_BLACK);
    signal dcol : t_col := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of ringa, ringb, dcol : signal is true;

    signal s_vw, s_rw_a, s_raa, s_rab : unsigned(7 downto 0) := (others => '0');
    signal s_rw_d, s_rqa, s_rqb : std_logic_vector(31 downto 0) := C_BLACK;
    signal s_dra, s_dwa : unsigned(10 downto 0) := (others => '0');
    signal s_dq, s_dwd : std_logic_vector(7 downto 0) := (others => '0');
    signal s_dwe : std_logic := '0';

    -- controls
    signal s_tid : std_logic_vector(3 downto 0) := (others => '0');
    signal s_xs  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ys  : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_warm : unsigned(3 downto 0) := (others => '0');
    signal s_j1, s_j2, s_j3, s_j4 : unsigned(3 downto 0) := (others => '0');
    signal s_j5 : unsigned(5 downto 0) := (others => '0');
    signal s_cdk : unsigned(1 downto 0) := "00";
    signal s_live6 : std_logic := '0';
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_gdiff : signed(12 downto 0) := (others => '0');
    signal s_g : unsigned(11 downto 0) := (others => '0');      -- glided P12, Q.2
    -- field-latched
    signal s_l1, s_l2, s_l3, s_l4 : unsigned(3 downto 0) := (others => '0');
    signal s_l5 : unsigned(5 downto 0) := (others => '0');
    signal s_card : unsigned(1 downto 0) := "00";
    signal s_dark, s_wob, s_slant, s_band, s_ghost : std_logic := '0';
    signal s_wph : unsigned(7 downto 0) := (others => '0');
    -- derived
    signal s_n : unsigned(3 downto 0) := to_unsigned(4, 4);
    signal s_n2 : unsigned(4 downto 0) := to_unsigned(8, 5);
    signal s_incv : unsigned(11 downto 0) := to_unsigned(512, 12);
    signal s_inc  : unsigned(13 downto 0) := to_unsigned(512, 14);   -- ridge phase per pixel, Q12
    signal s_sa, s_sb : unsigned(11 downto 0) := (others => '0');
    signal s_incy : unsigned(11 downto 0) := (others => '0');        -- ridge phase per line (Slant)
    signal s_tq : unsigned(15 downto 0) := (others => '0');
    signal s_wc : signed(7 downto 0) := (others => '0');
    signal s_wa : signed(12 downto 0) := (others => '0');
    signal s_tilt : unsigned(12 downto 0) := (others => '0');        -- Q8 views, offset by N/2
    signal s_binc : unsigned(9 downto 0) := (others => '0');         -- band per pixel, Q16 views
    signal s_spn : unsigned(11 downto 0) := (others => '0');         -- spread x N, Q8
    signal s_dreal : unsigned(6 downto 0) := (others => '0');
    signal s_dt : unsigned(7 downto 0) := (others => '0');
    signal s_stepp : unsigned(14 downto 0) := (others => '0');
    signal s_step : unsigned(9 downto 0) := (others => '0');         -- shift per view at full depth, Q4 px
    signal s_sheen : unsigned(4 downto 0) := (others => '0');
    signal s_soff : unsigned(8 downto 0) := (others => '0');

    -- line
    signal s_hsq, s_ph, s_upar, s_swl : std_logic := '0';
    signal s_upc : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal s_vl : unsigned(19 downto 0) := (others => '0');
    signal s_lph : unsigned(11 downto 0) := (others => '0');

    -- pixel path (suffix = stage)
    type t_u6a  is array (natural range <>) of unsigned(5 downto 0);
    type t_u11a is array (natural range <>) of unsigned(10 downto 0);
    signal s_y1 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u1, s_v1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_rx : t_u11a(1 to 9) := (others => (others => '0'));
    signal s_ph1 : unsigned(11 downto 0) := (others => '0');
    signal s_bd1 : unsigned(19 downto 0) := (others => '0');
    signal s_yd : t_u6a(2 to 17) := (others => (others => '0'));
    signal s_box : unsigned(9 downto 0) := (others => '0');
    type t_u12a is array (natural range <>) of unsigned(11 downto 0);
    signal s_phd : t_u12a(2 to 13) := (others => (others => '0'));
    signal s_bd2 : unsigned(11 downto 0) := (others => '0');
    signal s_sp3 : unsigned(19 downto 0) := (others => '0');
    signal s_bd3 : unsigned(11 downto 0) := (others => '0');
    signal s_a4, s_a5 : unsigned(12 downto 0) := (others => '0');
    signal s_box3, s_box4, s_box5 : unsigned(9 downto 0) := (others => '0');
    signal s_dq5 : unsigned(7 downto 0) := (others => '0');
    signal s_d6, s_d7 : unsigned(7 downto 0) := (others => '0');
    signal s_v6 : unsigned(4 downto 0) := (others => '0');
    signal s_s6 : unsigned(7 downto 0) := (others => '0');
    signal s_v7 : unsigned(3 downto 0) := (others => '0');
    type t_u4a is array (natural range <>) of unsigned(3 downto 0);
    signal s_vd : t_u4a(8 to 12) := (others => (others => '0'));
    signal s_vbd : t_u4a(8 to 12) := (others => (others => '0'));
    type t_u8a is array (natural range <>) of unsigned(7 downto 0);
    signal s_sd : t_u8a(7 to 13) := (others => (others => '0'));
    signal s_vb7 : unsigned(3 downto 0) := (others => '0');
    signal s_dv8, s_dvb8 : unsigned(12 downto 0) := (others => '0');
    signal s_dd8 : unsigned(7 downto 0) := (others => '0');
    signal s_sha9, s_shb9 : unsigned(18 downto 0) := (others => '0');
    signal s_pa12, s_pb12 : std_logic_vector(31 downto 0) := C_BLACK;
    signal s_hix12 : unsigned(3 downto 0) := (others => '0');
    signal s_ya13, s_yb13 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ua13, s_va13, s_ub13, s_vb13 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y14 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u14, s_v14 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_cs14 : signed(13 downto 0) := (others => '0');
    signal s_f15 : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_y15 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u15, s_v15 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_yp16 : unsigned(19 downto 0) := (others => '0');
    signal s_u16, s_v16 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y17 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u17, s_v17 : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y18, s_y19, s_y20, s_y21, s_y22, s_y23x, s_yf23 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');   -- s_avid_sr(k) = avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- RAMs: one write statement, registered read, no conditional read
    ---------------------------------------------------------------------------
    p_ringa : process(clk)
    begin
        if rising_edge(clk) then
            s_rqa <= ringa(to_integer(s_raa));
            ringa(to_integer(s_rw_a)) <= s_rw_d;
        end if;
    end process p_ringa;

    p_ringb : process(clk)
    begin
        if rising_edge(clk) then
            s_rqb <= ringb(to_integer(s_rab));
            ringb(to_integer(s_rw_a)) <= s_rw_d;
        end if;
    end process p_ringb;

    p_dcol : process(clk)
    begin
        if rising_edge(clk) then
            s_dq <= dcol(to_integer(s_dra));
            if s_dwe = '1' then dcol(to_integer(s_dwa)) <= s_dwd; end if;
        end if;
    end process p_dcol;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-field latches and derived constants
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_k : unsigned(9 downto 0);
        variable v_g : signed(12 downto 0);
        variable v_t : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_xs <= f_vstep_x(s_tid);
            s_ys <= f_vstep_y(s_tid);

            -- vblank zone: no avid for 1024 clocks (hblank is at most 370)
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            -- K1..K4: 16 zones, deadband after warm-up
            v_k := unsigned(registers_in(0));
            if s_warm /= 15 or ((v_k(5 downto 0) >= 6 or v_k(9 downto 6) = 0) and
                                (v_k(5 downto 0) <= 57 or v_k(9 downto 6) = 15)) then
                s_j1 <= v_k(9 downto 6);
            end if;
            v_k := unsigned(registers_in(1));
            if s_warm /= 15 or ((v_k(5 downto 0) >= 6 or v_k(9 downto 6) = 0) and
                                (v_k(5 downto 0) <= 57 or v_k(9 downto 6) = 15)) then
                s_j2 <= v_k(9 downto 6);
            end if;
            v_k := unsigned(registers_in(2));
            if s_warm /= 15 or ((v_k(5 downto 0) >= 6 or v_k(9 downto 6) = 0) and
                                (v_k(5 downto 0) <= 57 or v_k(9 downto 6) = 15)) then
                s_j3 <= v_k(9 downto 6);
            end if;
            v_k := unsigned(registers_in(3));
            if s_warm /= 15 or ((v_k(5 downto 0) >= 6 or v_k(9 downto 6) = 0) and
                                (v_k(5 downto 0) <= 57 or v_k(9 downto 6) = 15)) then
                s_j4 <= v_k(9 downto 6);
            end if;
            -- K5 Spread: 64 zones
            v_k := unsigned(registers_in(4));
            if s_warm /= 15 or ((v_k(3 downto 0) >= 3 or v_k(9 downto 4) = 0) and
                                (v_k(3 downto 0) <= 12 or v_k(9 downto 4) = 63)) then
                s_j5 <= v_k(9 downto 4);
            end if;
            -- K6 Card (4 labels): the firmware loads the default as the label
            -- INDEX; honour it until the knob first reads a real position
            v_k := unsigned(registers_in(5));
            if v_k >= 4 then s_live6 <= '1'; end if;
            if s_live6 = '0' and v_k < 4 then
                s_cdk <= v_k(1 downto 0);
            elsif s_warm /= 15 or ((v_k(7 downto 0) >= 12 or v_k(9 downto 8) = 0) and
                                   (v_k(7 downto 0) <= 243 or v_k(9 downto 8) = 3)) then
                s_cdk <= v_k(9 downto 8);
            end if;
            -- P12 Tilt glide (1/8 of the gap a field)
            s_p12 <= unsigned(registers_in(7));
            s_gdiff <= signed(resize(s_p12 & "00", 13)) - signed(resize(s_g, 13));

            -- per field (zone entry: exactly once per vertical blank)
            if s_zgo = '1' then
                if s_warm /= 15 then s_warm <= s_warm + 1; end if;
                v_g := signed(resize(s_g, 13)) + shift_right(s_gdiff, 3);
                s_g <= unsigned(v_g(11 downto 0));
                s_l1 <= s_j1; s_l2 <= s_j2; s_l3 <= s_j3; s_l4 <= s_j4; s_l5 <= s_j5;
                s_card  <= s_cdk;
                s_dark  <= registers_in(6)(0);
                s_wob   <= registers_in(6)(1);
                s_slant <= registers_in(6)(2);
                s_band  <= registers_in(6)(3);
                s_ghost <= registers_in(6)(4);
                s_wph   <= s_wph + 1;          -- wobble: one rock every 256 fields (~4 s)
            end if;

            -- derived (one op per register; they settle within a few clocks)
            s_n  <= to_unsigned(C_NV(to_integer(s_l2)), 4);
            s_n2 <= shift_left(resize(s_n, 5), 1);
            -- ridge phase per pixel (Q12) = 4096 / pitch x (virtual px per pixel)
            s_incv <= to_unsigned(C_INC(to_integer(s_l1)), 12);
            s_inc  <= resize(shift_right(resize(s_incv, 14) * s_xs, 1), 14);
            -- Slant: half a pitch of phase per virtual line = incv x (ystep / 512),
            -- one add of two selected shifts per mode
            case to_integer(unsigned(s_tid)) is
                when 0          => s_sa <= shift_left(s_incv, 1);  s_sb <= shift_right(s_incv, 2);   -- x2.25
                when 4          => s_sa <= s_incv;                 s_sb <= shift_right(s_incv, 3);   -- x1.125
                when 8          => s_sa <= shift_left(s_incv, 1);  s_sb <= (others => '0');         -- x2
                when 12         => s_sa <= s_incv;                 s_sb <= (others => '0');         -- x1
                when 5 | 6 | 14 => s_sa <= shift_right(s_incv, 1); s_sb <= shift_right(s_incv, 2);   -- x0.75
                when 1 | 2 | 10 => s_sa <= s_incv;                 s_sb <= (others => '0');         -- x1
                when others     => s_sa <= shift_right(s_incv, 1); s_sb <= (others => '0');         -- x0.5
            end case;
            if s_slant = '1' then s_incy <= s_sa + s_sb; else s_incy <= (others => '0'); end if;
            -- tilt: P12 sweeps the N views (Q8 views) + wobble (+-N/2), offset N/2
            s_tq <= resize(s_g, 12) * s_n;                                   -- Q12 x N
            s_wc <= to_signed(C_COS(to_integer(s_wph(7 downto 2))), 8);
            s_wa <= resize(s_wc * signed('0' & s_n), 13);                     -- +-127 N
            if s_wob = '1' then
                v_t := signed(resize(s_tq(15 downto 4), 14)) + shift_left(resize(signed('0' & s_n), 14), 7) + resize(s_wa, 14);
            else
                v_t := signed(resize(s_tq(15 downto 4), 14)) + shift_left(resize(signed('0' & s_n), 14), 7);
            end if;
            s_tilt <= unsigned(v_t(12 downto 0));
            -- bands: (N - 1/4) views across the virtual width, Q16 per pixel
            if s_band = '1' then s_binc <= resize(s_n * s_xs * to_unsigned(16, 5), 10); else s_binc <= (others => '0'); end if;
            -- spread: up to N - 1/4 views across one ridge
            s_spn <= resize(s_l5 & "00" , 8) * s_n;
            -- parallax: Depth (0 .. 48 virtual px) in real px, split over N - 1 view steps
            s_dt <= resize(s_l4 * to_unsigned(13, 4), 8);                       -- 4 x virtual px (0 .. 195)
            if s_xs = 4 then
                s_dreal <= resize(shift_right(s_dt, 3), 7);
            elsif s_xs = 3 then
                s_dreal <= resize(shift_right(s_dt, 3) + shift_right(s_dt, 5), 7);
            else
                s_dreal <= resize(shift_right(s_dt, 2), 7);
            end if;
            s_stepp <= s_dreal * to_unsigned(C_RCP(to_integer(s_n)), 8);         -- px / (N - 1), Q8
            s_step <= s_stepp(13 downto 4);                                      -- Q4
            -- sheen 0 .. 15; offset keeps the mean brightness (sheen x 0.15)
            s_sheen <= resize(s_l3, 5);
            s_soff  <= resize(s_l3 * to_unsigned(5, 3), 9);

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Line events: hsync->avid parity vote, virtual line, ridge phase per line
    ---------------------------------------------------------------------------
    p_line : process(clk)
    begin
        if rising_edge(clk) then
            s_hsq <= data_in.hsync_n;
            if s_hsq = '1' and data_in.hsync_n = '0' then s_ph <= '0'; else s_ph <= not s_ph; end if;
            if data_in.avid = '1' and s_avid_sr(0) = '0' then
                s_swl <= s_ph xor s_upar;
                if s_ph = '1' then
                    if s_upc /= 15 then s_upc <= s_upc + 1; end if;
                else
                    if s_upc /= 0 then s_upc <= s_upc - 1; end if;
                end if;
            end if;
            if s_upc = 15 then s_upar <= '1'; elsif s_upc = 0 then s_upar <= '0'; end if;

            if s_zone = '1' then
                s_vl <= (others => '0');
                s_lph <= (others => '0');
            elsif data_in.avid = '0' and s_avid_sr(0) = '1' then
                s_vl <= s_vl + s_ys;
                s_lph <= s_lph + s_incy;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a   : unsigned(12 downto 0);
        variable v_v   : unsigned(4 downto 0);
        variable v_h   : unsigned(7 downto 0);
        variable v_dd, v_dn : signed(9 downto 0);
        variable v_off : unsigned(10 downto 0);
        variable v_y   : signed(11 downto 0);
        variable v_c   : signed(11 downto 0);
        variable v_ya, v_yb : unsigned(9 downto 0);
        variable v_ua, v_va, v_ub, v_vb : unsigned(9 downto 0);
        variable v_hue : std_logic_vector(19 downto 0);
        variable v_f   : signed(13 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_dl  : unsigned(9 downto 0);
        variable v_yy  : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: input; column; ridge phase and band DDAs (restart each line)
            s_y1 <= unsigned(data_in.y);
            s_u1 <= unsigned(data_in.u);
            s_v1 <= unsigned(data_in.v);
            if data_in.avid = '1' and s_avid_sr(0) = '0' then
                s_rx(1) <= (others => '0');
                s_ph1 <= s_lph;
                s_bd1 <= (others => '0');
            else
                if s_rx(1) /= 2047 then s_rx(1) <= s_rx(1) + 1; end if;
                s_ph1 <= s_ph1 + s_inc(11 downto 0);
                if s_bd1(19 downto 18) /= "11" then s_bd1 <= s_bd1 + s_binc; end if;
            end if;
            for i in 2 to 9 loop s_rx(i) <= s_rx(i - 1); end loop;

            -- 2: picture into both rings; luma box (16 px of 6-bit luma)
            s_rw_a <= s_vw;
            s_vw <= s_vw + 1;
            s_rw_d <= std_logic_vector(s_y1) & std_logic_vector(s_u1) & std_logic_vector(s_v1) & "00";
            s_yd(2) <= s_y1(9 downto 4);
            for i in 3 to 17 loop s_yd(i) <= s_yd(i - 1); end loop;
            s_box <= s_box + resize(s_y1(9 downto 4), 10) - resize(s_yd(17), 10);
            s_phd(2) <= s_ph1;
            for i in 3 to 13 loop s_phd(i) <= s_phd(i - 1); end loop;
            s_bd2 <= s_bd1(19 downto 8);

            -- 3: spread x ridge phase; depth column read
            s_sp3 <= s_phd(2)(11 downto 4) * s_spn;
            s_bd3 <= s_bd2;
            s_box3 <= s_box;
            s_dra <= s_rx(2);
            -- 4: view coordinate (Q8 views): tilt + band + spread
            s_a4 <= s_tilt + resize(s_bd3, 13) + resize(s_sp3(19 downto 8), 13);
            s_box4 <= s_box3;
            -- 5: first reduction (mod 2N); depth: column IIR of the box
            v_v := resize(s_a4(12 downto 8), 5);
            if v_v >= s_n2 then
                s_a5 <= s_a4 - shift_left(resize(s_n2, 13), 8);
            else
                s_a5 <= s_a4;
            end if;
            s_box5 <= s_box4;
            s_dq5 <= unsigned(s_dq);
            -- 6: view = int part mod N; sub-view fraction; depth
            v_v := resize(s_a5(12 downto 8), 5);
            if v_v >= s_n then s_v6 <= v_v - s_n; else s_v6 <= v_v; end if;
            s_s6 <= s_a5(7 downto 0);
            v_h := s_box5(9 downto 2);
            v_dd := signed(resize(v_h, 10)) - signed(resize(s_dq5, 10));
            v_dn := signed(resize(s_dq5, 10)) + shift_right(v_dd, 2);
            s_d6 <= unsigned(v_dn(7 downto 0));
            -- (depth column write-back: same column, 3 clocks after its read)
            s_dwa <= s_rx(5);
            s_dwd <= std_logic_vector(v_dn(7 downto 0));
            s_dwe <= s_avid_sr(4);
            -- 7: neighbouring view; near = light or dark
            s_v7 <= s_v6(3 downto 0);
            if s_v6(3 downto 0) + 1 >= s_n then s_vb7 <= (others => '0'); else s_vb7 <= s_v6(3 downto 0) + 1; end if;
            s_sd(7) <= s_s6;
            if s_dark = '1' then s_d7 <= not s_d6; else s_d7 <= s_d6; end if;
            -- 8: shift per view (view x step, Q4 px)
            s_dv8  <= resize(s_v7 * s_step, 13);
            s_dvb8 <= resize(s_vb7 * s_step, 13);
            s_dd8 <= s_d7;
            s_vd(8) <= s_v7; s_vbd(8) <= s_vb7;
            -- 9: shift = view step x depth (6-bit depth; Q10 px)
            s_sha9 <= s_dv8 * s_dd8(7 downto 2);
            s_shb9 <= s_dvb8 * s_dd8(7 downto 2);
            -- 10: ring reads at the shifted column (one-sided: past only; clamped
            --     to the line's first column).  This pixel went in at stage 2.
            v_off := resize(s_sha9(18 downto 10), 11);
            if v_off > s_rx(9) then v_off := s_rx(9); end if;
            s_raa <= s_vw - 8 - v_off(7 downto 0);
            v_off := resize(s_shb9(18 downto 10), 11);
            if v_off > s_rx(9) then v_off := s_rx(9); end if;
            s_rab <= s_vw - 8 - v_off(7 downto 0);
            -- 11: ring outputs (EBR registers).  12: fabric; Holo hue index
            s_pa12 <= s_rqa;
            s_pb12 <= s_rqb;
            v_h := resize(s_vd(11)(1 downto 0), 8);
            s_hix12 <= shift_left(v_h(3 downto 0), 2) + s_tilt(9 downto 6) + s_vl(16 downto 13);

            -- 13: card transform of both views (odd views of Flip / Pop)
            v_ya := unsigned(s_pa12(31 downto 22)); v_ua := unsigned(s_pa12(21 downto 12)); v_va := unsigned(s_pa12(11 downto 2));
            v_yb := unsigned(s_pb12(31 downto 22)); v_ub := unsigned(s_pb12(21 downto 12)); v_vb := unsigned(s_pb12(11 downto 2));
            case s_card is
                when "01" =>      -- Flip: odd views negative
                    if s_vd(12)(0) = '1' then
                        if v_ya > 940 then v_ya := to_unsigned(64, 10); else v_ya := to_unsigned(1004, 10) - v_ya; end if;
                        v_ua := to_unsigned(1023, 10) - v_ua; v_va := to_unsigned(1023, 10) - v_va;
                    end if;
                    if s_vbd(12)(0) = '1' then
                        if v_yb > 940 then v_yb := to_unsigned(64, 10); else v_yb := to_unsigned(1004, 10) - v_yb; end if;
                        v_ub := to_unsigned(1023, 10) - v_ub; v_vb := to_unsigned(1023, 10) - v_vb;
                    end if;
                when "10" =>      -- Holo: the picture's light in a rainbow that moves with view and tilt
                    v_hue := C_PRISM(to_integer(s_hix12));
                    if s_swl = '1' then
                        v_ua := unsigned(v_hue(9 downto 0)); v_va := unsigned(v_hue(19 downto 10));
                    else
                        v_ua := unsigned(v_hue(19 downto 10)); v_va := unsigned(v_hue(9 downto 0));
                    end if;
                    v_ub := v_ua; v_vb := v_va;
                when "11" =>      -- Pop: odd views posterized, chroma pushed
                    if s_vd(12)(0) = '1' then
                        if v_ya < 300 then v_ya := to_unsigned(110, 10); elsif v_ya < 600 then v_ya := to_unsigned(440, 10); else v_ya := to_unsigned(780, 10); end if;
                        v_c := signed(resize(v_ua, 12)) - 512; v_c := 512 + v_c + shift_right(v_c, 1);
                        if v_c < 16 then v_c := to_signed(16, 12); elsif v_c > 1008 then v_c := to_signed(1008, 12); end if;
                        v_ua := unsigned(v_c(9 downto 0));
                        v_c := signed(resize(v_va, 12)) - 512; v_c := 512 + v_c + shift_right(v_c, 1);
                        if v_c < 16 then v_c := to_signed(16, 12); elsif v_c > 1008 then v_c := to_signed(1008, 12); end if;
                        v_va := unsigned(v_c(9 downto 0));
                    end if;
                    if s_vbd(12)(0) = '1' then
                        if v_yb < 300 then v_yb := to_unsigned(110, 10); elsif v_yb < 600 then v_yb := to_unsigned(440, 10); else v_yb := to_unsigned(780, 10); end if;
                    end if;
                when others => null;   -- 3D: parallax views of the picture
            end case;
            s_ya13 <= v_ya; s_ua13 <= v_ua; s_va13 <= v_va;
            s_yb13 <= v_yb; s_ub13 <= v_ub; s_vb13 <= v_vb;

            -- 14: Ghost: near each view transition the next view bleeds in half;
            --     ridge cosine x Sheen
            if s_ghost = '1' and s_sd(13) >= 160 then
                s_y14 <= resize(shift_right(resize(s_ya13, 11) + resize(s_yb13, 11), 1), 10);
                s_u14 <= resize(shift_right(resize(s_ua13, 11) + resize(s_ub13, 11), 1), 10);
                s_v14 <= resize(shift_right(resize(s_va13, 11) + resize(s_vb13, 11), 1), 10);
            else
                s_y14 <= s_ya13; s_u14 <= s_ua13; s_v14 <= s_va13;
            end if;
            s_cs14 <= resize(to_signed(C_COS(to_integer(s_phd(13)(11 downto 6))), 8) * signed('0' & s_sheen), 14);
            -- 15: sheen factor (Q8): 256 + 0.09 sheen cos - offset
            v_f := to_signed(256, 14) + shift_right(s_cs14, 4) + shift_right(s_cs14, 5) - signed(resize(s_soff, 14));
            if v_f < 64 then s_f15 <= to_unsigned(64, 10); elsif v_f > 511 then s_f15 <= to_unsigned(511, 10); else s_f15 <= unsigned(v_f(9 downto 0)); end if;
            if s_y14 > 64 then s_y15 <= s_y14 - 64; else s_y15 <= (others => '0'); end if;   -- luma above black
            s_u15 <= s_u14; s_v15 <= s_v14;
            -- 16: luma x sheen (about black)
            s_yp16 <= s_y15 * s_f15;
            s_u16 <= s_u15; s_v16 <= s_v15;
            -- 17: compose (cap white)
            v_yy := shift_right(s_yp16, 8) + 64;
            if v_yy > 900 then s_y17 <= to_unsigned(900, 10); else s_y17 <= v_yy(9 downto 0); end if;
            s_u17 <= s_u16; s_v17 <= s_v16;

            -- 18..23: output conditioning.  Chroma [1 2 2 2 2 2 2 2 1]/16 as
            -- two adjacent 8-sums (centre at stage 22 -> s_u23); luma [1 2 1]
            -- centred at stage 22 -> s_yf23.
            s_csu(0) <= s_u17; s_csv(0) <= s_v17;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + s_u17 - s_csu(7);
            s_boxv <= s_boxv + s_v17 - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u23 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v23 <= v_sc(13 downto 4);
            s_y18 <= s_y17; s_y19 <= s_y18; s_y20 <= s_y19; s_y21 <= s_y20;
            s_y22 <= s_y21; s_y23x <= s_y22;
            v_sy := resize(s_y21, 12) + shift_left(resize(s_y22, 12), 1) + resize(s_y23x, 12);
            s_yf23 <= v_sy(11 downto 2);
            -- 24: luma toward black, 64 + 13/16 (y - 64); blanking gate
            if s_yf23 > 64 then v_dl := s_yf23 - 64; else v_dl := (others => '0'); end if;
            if s_avid_sr(22) = '1' then
                s_y24 <= s_yf23 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u24 <= s_u23; s_v24 <= s_v23;
            else
                s_y24 <= to_unsigned(64, 10); s_u24 <= to_unsigned(512, 10); s_v24 <= to_unsigned(512, 10);
            end if;

            -- delays
            s_vd(9) <= s_vd(8); s_vd(10) <= s_vd(9); s_vd(11) <= s_vd(10); s_vd(12) <= s_vd(11);
            s_vbd(9) <= s_vbd(8); s_vbd(10) <= s_vbd(9); s_vbd(11) <= s_vbd(10); s_vbd(12) <= s_vbd(11);
            for i in 8 to 13 loop s_sd(i) <= s_sd(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture lenticular;
