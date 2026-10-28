-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Goblet v0.5.0 -- Heirloom #49 (2026-10-27 rebuild), from WIN_20230916_01_08_00.
--
-- The photo is an additive RGB patch: three stripe oscillators (R ~124 px, G ~327 px, B ~182 px
-- columns, line-locked) plus a "glass" layer added on top of them channel by channel.  The glass
-- is a polar colourizer driven by two smooth control fields: its hue (angle) walks a colour wheel
-- neutral -> olive -> orange -> pink -> blue -> cyan -> blue-grey, its magnitude keys the dim cells
-- (the goblet's bowl, the tilted lenses, the cells under the rims) against the saturated bands.
--   * the two fields are a 120 x 67 knot grid (one byte per knot: 5-bit angle, 3-bit magnitude),
--     fitted to the photo with the stripes and the 64 x 8 colour LUT (goblet.py); a line engine
--     interpolates the line's knot row vertically (angle wrap-aware) into a ping-pong row RAM and
--     the pixel path interpolates along it;
--   * the stripes are the photo's own column profile every 4 virtual px, linearly interpolated;
--   * the line-start block: a colourized line-start ramp keyed into a round-ended block at the
--     bottom left and two short rows at the left edge.
--
--   K1  Stripes  stripe frequency x0.5 .. x2 (the photo at the centre)
--   K2  Width    the goblet's width (field x-scale about its axis), x0.6 .. x1.6
--   K3  Hue      the glass colours turned round the wheel (a full turn)
--   K4  Speed    bipolar, still in the centre: the glass colours flow, the stripes drift
--   K5  Melt     how far the source (video / oscillator) bends the glass colours and the stripes
--   K6  Patch    RGB cable patch: RGB / RBG / GRB / GBR / BRG / BGR
--   S7  Mirror   the left half folded onto the right: a symmetric goblet
--   S8  Stripes  On (photo) / Off: the glass on grey
--   S9  Block    On (photo) / Off: the line-start block and stubs
--   S10 Flip     the field upside down (bowl at the bottom)
--   S11 Source   Video / Osc
--   P12 GLASS    0 = a faint ghost of the glass over the stripes, 50 % = the photo,
--                100 % = the glass swells and its hue rings multiply (magnitude -> hue)
--
-- Virtual coordinates: x spans 1920 virtual px at 1080 / 720 (SD shows the central 1440), y =
-- 1080 virtual lines.  Generated avid; output conditioning ([1 2 2 2 2 2 2 2 1]/16 on Y, U, V,
-- luma 13/16) and blanking gate.  Latency 32 clocks in every mode.  Colours: u = Cr, v = Cb.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.goblet_pkg.all;
use work.all;

architecture goblet of program_top is
    constant C_LAT : integer := 32;   -- matches processing_delay_clks
    constant C_CX  : integer := 4224341;            -- goblet axis, Q16 knots (1040 virtual px)
    constant C_SM  : integer := 31457280;           -- stripe period, Q16 samples (480)
    constant C_CX5 : integer := 2063;               -- the axis in Q5 knots

    function f_kx(t : std_logic_vector(3 downto 0)) return unsigned is    -- virtual px per px, Q8
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(512, 10);
            when 5 | 6 | 14     => return to_unsigned(384, 10);
            when others         => return to_unsigned(256, 10);
        end case;
    end function;
    function f_sd(t : std_logic_vector(3 downto 0)) return std_logic is     -- SD: central 1440
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return '1';
            when others         => return '0';
        end case;
    end function;
    function f_ys(t : std_logic_vector(3 downto 0)) return unsigned is    -- virtual lines per line, Q8
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 11);
            when 4          => return to_unsigned(576, 11);
            when 8          => return to_unsigned(960, 11);
            when 12         => return to_unsigned(480, 11);
            when 5 | 6 | 14 => return to_unsigned(384, 11);
            when 1 | 2 | 10 => return to_unsigned(512, 11);
            when others     => return to_unsigned(256, 11);
        end case;
    end function;
    function f_tri8(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    -- ROMs
    signal s_ka00, s_ka01, s_ka10, s_ka11 : unsigned(10 downto 0) := (others => '0');
    signal s_kq00, s_kq01, s_kq10, s_kq11 : std_logic_vector(7 downto 0) := (others => '0');
    signal s_la : unsigned(8 downto 0) := (others => '0');
    signal s_lq : std_logic_vector(23 downto 0) := (others => '0');
    signal s_sea, s_soa : unsigned(7 downto 0) := (others => '0');
    signal s_seq, s_soq : std_logic_vector(23 downto 0) := (others => '0');
    -- row RAMs (ping-pong by line: bank & column pair): even / odd knot columns, angle 10 | magnitude 6
    type t_row is array (0 to 127) of std_logic_vector(15 downto 0);
    signal rram_e, rram_o : t_row := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of rram_e : signal is true;
    attribute no_rw_check of rram_o : signal is true;
    signal r_we : std_logic := '0';
    signal r_wa : unsigned(6 downto 0) := (others => '0');
    signal r_wde, r_wdo : std_logic_vector(15 downto 0) := (others => '0');
    signal s_rae, s_rao : unsigned(6 downto 0) := (others => '0');
    signal s_rqe, s_rqo : std_logic_vector(15 downto 0) := (others => '0');

    -- timing / zone / width
    signal s_tid : std_logic_vector(3 downto 0) := (others => '1');
    signal s_kx : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal s_sdm : std_logic := '0';
    signal s_ys : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq : std_logic := '0';
    signal s_kick_c, s_kick_e : std_logic := '0';
    attribute keep : boolean;
    attribute keep of s_kick_c : signal is true;
    attribute keep of s_kick_e : signal is true;
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- glided knobs (10.3), switches
    signal g1, g2, g3, g5, g12 : unsigned(12 downto 0) := (others => '0');
    signal s_sp : signed(10 downto 0) := (others => '0');
    signal s_asp : unsigned(9 downto 0) := (others => '0');
    signal s_patch : unsigned(2 downto 0) := (others => '0');
    signal s_mirror, s_nostr, s_noblk, s_flip, s_osc : std_logic := '0';
    signal s_warm : std_logic := '0';
    type t_gd is array (0 to 4) of signed(14 downto 0);
    type t_gn is array (0 to 4) of unsigned(12 downto 0);
    signal s_gd : t_gd := (others => (others => '0'));
    signal s_gn : t_gn := (others => (others => '0'));
    signal s_gb : std_logic_vector(4 downto 0) := (others => '0');
    signal s_zgo2, s_zgo3 : std_logic := '0';

    -- field constants
    signal c_sf, c_iw : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal c_dxk : unsigned(15 downto 0) := (others => '0');
    signal c_xk0 : signed(25 downto 0) := (others => '0');
    signal c_ds : unsigned(16 downto 0) := (others => '0');
    signal c_s0 : unsigned(24 downto 0) := (others => '0');
    signal c_dyk : unsigned(15 downto 0) := (others => '0');
    signal c_g : unsigned(8 downto 0) := to_unsigned(256, 9);
    signal c_kr : unsigned(7 downto 0) := (others => '0');
    signal c_km : unsigned(7 downto 0) := (others => '0');
    signal c_ka, c_kv : unsigned(8 downto 0) := (others => '0');
    signal c_hue : unsigned(9 downto 0) := (others => '0');
    signal c_osx : unsigned(7 downto 0) := (others => '0');
    signal c_osy : unsigned(9 downto 0) := (others => '0');
    signal s_hdr : unsigned(17 downto 0) := (others => '0');       -- hue drift, Q8 of 1/1024 turn
    signal s_sdr : unsigned(24 downto 0) := (others => '0');       -- stripe drift, Q16 samples (mod 480)
    signal s_oxt, s_oyt : unsigned(15 downto 0) := (others => '0');
    signal s_linit : std_logic := '0';
    -- field sequencer: one serial shift-add multiplier
    signal f_st : unsigned(5 downto 0) := (others => '0');
    signal f_ma : unsigned(33 downto 0) := (others => '0');
    signal f_mb : unsigned(12 downto 0) := (others => '0');
    signal f_acc : unsigned(33 downto 0) := (others => '0');
    signal f_mc : unsigned(3 downto 0) := (others => '0');
    signal f_hi : std_logic := '0';
    signal f_d : unsigned(9 downto 0) := (others => '0');
    signal f_m : unsigned(17 downto 0) := (others => '0');
    signal f_t : signed(26 downto 0) := (others => '0');

    -- per-line engine (computes the NEXT line, kicked at the source avid rise)
    signal e_st : unsigned(2 downto 0) := (others => '0');
    signal e_yv8 : unsigned(19 downto 0) := (others => '0');
    signal e_yk : signed(23 downto 0) := (others => '0');
    signal e_oyl : unsigned(15 downto 0) := (others => '0');
    signal e_j : unsigned(6 downto 0) := (others => '0');
    signal e_yv : unsigned(10 downto 0) := (others => '0');
    signal e_dy : unsigned(10 downto 0) := (others => '0');
    signal e_stub, e_purp, e_blank : std_logic := '0';
    signal n_j : unsigned(6 downto 0) := (others => '0');
    signal n_ty : unsigned(4 downto 0) := (others => '0');
    signal n_rb0, n_rb1 : unsigned(10 downto 0) := (others => '0');
    signal n_xe : unsigned(7 downto 0) := (others => '0');
    signal n_purp, n_blank, n_bank : std_logic := '0';
    signal n_oy : unsigned(7 downto 0) := (others => '0');
    signal l_xe : unsigned(7 downto 0) := (others => '0');
    signal l_purp, l_blank, l_bank : std_logic := '0';
    signal l_oy : unsigned(7 downto 0) := (others => '0');
    -- row engine (the next line's knot row, interpolated vertically, into the row RAMs)
    signal r_go, r_run : std_logic := '0';
    signal r_p : unsigned(5 downto 0) := (others => '0');
    signal r_v : std_logic_vector(4 downto 1) := (others => '0');
    type t_p6 is array (1 to 4) of unsigned(5 downto 0);
    signal r_pp : t_p6 := (others => (others => '0'));
    signal r_jp, r_bk : std_logic := '0';
    signal r_ty : unsigned(4 downto 0) := (others => '0');
    signal r_te, r_be, r_to, r_bo : unsigned(7 downto 0) := (others => '0');   -- top / bottom knots, even / odd column
    signal r_ate, r_ato : unsigned(4 downto 0) := (others => '0');
    signal r_mte, r_mto : unsigned(2 downto 0) := (others => '0');
    signal r_pae, r_pao : signed(10 downto 0) := (others => '0');
    signal r_pme, r_pmo : signed(9 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_start : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_dst, g_dav : std_logic_vector(13 downto 0) := (others => '0');   -- delayed g_start / g_avid

    -- beams
    signal b_xk : signed(25 downto 0) := (others => '0');        -- glass field x, Q16 knots (stage 0)
    signal b_ox : unsigned(15 downto 0) := (others => '0');      -- oscillator x phase (stage 7)
    signal b_s : unsigned(24 downto 0) := (others => '0');       -- stripe position, Q16 samples (stage 10)
    signal b_xv : unsigned(19 downto 0) := (others => '0');      -- virtual x, Q8 (stage 10)

    -- source
    signal v_y1 : unsigned(9 downto 0) := (others => '0');
    signal v_sd2 : signed(8 downto 0) := (others => '0');
    signal o_t8 : unsigned(6 downto 0) := (others => '0');
    signal o_s9 : signed(8 downto 0) := (others => '0');
    signal p_sd10, p_sd11, p_sd12 : signed(8 downto 0) := (others => '0');

    -- glass path
    signal p_d1 : signed(14 downto 0) := (others => '0');
    signal p_xk1, p_xk2 : signed(14 downto 0) := (others => '0');
    signal p_ad2 : unsigned(13 downto 0) := (others => '0');
    signal p_xm3, p_xm4, p_hi4 : signed(14 downto 0) := (others => '0');
    signal p_ng4 : signed(14 downto 0) := (others => '0');
    signal p_lt4, p_gt4 : std_logic := '0';
    signal p_xr5 : unsigned(11 downto 0) := (others => '0');
    signal p_i6 : unsigned(6 downto 0) := (others => '0');
    signal p_tx6, p_tx7, p_tx8, p_tx9, p_tx10 : unsigned(4 downto 0) := (others => '0');
    signal p_pi7, p_pi8 : std_logic := '0';
    signal p_al9, p_ar9, p_al10, p_al11 : unsigned(9 downto 0) := (others => '0');
    signal p_ml9, p_mr9, p_ml10, p_ml11 : unsigned(5 downto 0) := (others => '0');
    signal p_da10 : signed(9 downto 0) := (others => '0');
    signal p_dm10 : signed(6 downto 0) := (others => '0');
    signal p_pa11 : signed(15 downto 0) := (others => '0');
    signal p_pm11 : signed(12 downto 0) := (others => '0');
    signal p_a12 : unsigned(9 downto 0) := (others => '0');
    signal p_m12 : unsigned(8 downto 0) := (others => '0');
    signal p_mg13 : unsigned(15 downto 0) := (others => '0');
    signal p_ms13 : signed(15 downto 0) := (others => '0');
    signal p_aa13 : unsigned(9 downto 0) := (others => '0');
    signal p_as13 : signed(13 downto 0) := (others => '0');
    signal p_mg14 : signed(10 downto 0) := (others => '0');
    signal p_ab14, p_ab15, p_ab16 : unsigned(9 downto 0) := (others => '0');
    signal p_mc15 : unsigned(7 downto 0) := (others => '0');
    signal p_mi15, p_mi16 : unsigned(2 downto 0) := (others => '0');
    signal p_rp16 : unsigned(13 downto 0) := (others => '0');
    signal p_l19 : std_logic_vector(23 downto 0) := (others => '0');

    -- stripe path (stripe stage k = glass stage 10 + k)
    signal q_pos1 : unsigned(11 downto 0) := (others => '0');
    signal q_off1 : signed(13 downto 0) := (others => '0');
    signal q_p2 : unsigned(11 downto 0) := (others => '0');
    signal q_in1, q_in2, q_in3, q_in4, q_in5, q_in6, q_in7, q_in8, q_in9 : std_logic := '0';
    signal q_r1, q_r2 : unsigned(4 downto 0) := (others => '0');
    signal q_rf1, q_rf2 : unsigned(2 downto 0) := (others => '0');
    signal q_f3, q_f4, q_f5 : unsigned(2 downto 0) := (others => '0');
    signal q_par3, q_par4, q_par5 : std_logic := '0';
    signal q_e5, q_o5 : std_logic_vector(23 downto 0) := (others => '0');
    type t_u8 is array (natural range <>) of unsigned(7 downto 0);
    type t_s9 is array (natural range <>) of signed(8 downto 0);
    type t_s12 is array (natural range <>) of signed(11 downto 0);
    signal q_v06, q_v07 : t_u8(0 to 2) := (others => (others => '0'));
    signal q_dv6 : t_s9(0 to 2) := (others => (others => '0'));
    signal q_f6 : unsigned(2 downto 0) := (others => '0');
    signal q_pd7 : t_s12(0 to 2) := (others => (others => '0'));
    signal q_sv8 : t_u8(0 to 2) := (others => (others => '0'));
    signal q_out9 : t_u8(0 to 2) := (others => (others => '0'));

    -- composite / output
    signal p_rgb20, p_o21 : t_u8(0 to 2) := (others => (others => '0'));
    signal p_yr22, p_yg22, p_yb22 : unsigned(15 downto 0) := (others => '0');
    signal p_ur22, p_ug22, p_ub22, p_vr22, p_vg22, p_vb22 : signed(16 downto 0) := (others => '0');
    signal p_ys23 : unsigned(16 downto 0) := (others => '0');
    signal p_us23, p_vs23 : signed(16 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_y8, s_u8, s_v8 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_csy, s_csu, s_csv : t_u10(0 to 7) := (others => to_unsigned(512, 10));
    signal s_boxy, s_boxu, s_boxv, s_boxyd, s_boxud, s_boxvd : unsigned(12 downto 0) := to_unsigned(4096, 13);
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y12 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u12, s_v12 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_yf : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_uf, s_vf : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

begin

    ---------------------------------------------------------------------------
    -- Memories (registered reads; one read and at most one write per array)
    ---------------------------------------------------------------------------
    p_kb00 : process(clk) begin if rising_edge(clk) then s_kq00 <= C_KB00(to_integer(s_ka00)); end if; end process;
    p_kb01 : process(clk) begin if rising_edge(clk) then s_kq01 <= C_KB01(to_integer(s_ka01)); end if; end process;
    p_kb10 : process(clk) begin if rising_edge(clk) then s_kq10 <= C_KB10(to_integer(s_ka10)); end if; end process;
    p_kb11 : process(clk) begin if rising_edge(clk) then s_kq11 <= C_KB11(to_integer(s_ka11)); end if; end process;
    p_glut : process(clk) begin if rising_edge(clk) then s_lq <= C_GLUT(to_integer(s_la)); end if; end process;
    p_se   : process(clk) begin if rising_edge(clk) then s_seq <= C_SE(to_integer(s_sea)); end if; end process;
    p_so   : process(clk) begin if rising_edge(clk) then s_soq <= C_SO(to_integer(s_soa)); end if; end process;
    p_rre  : process(clk)
    begin
        if rising_edge(clk) then
            if r_we = '1' then rram_e(to_integer(r_wa)) <= r_wde; end if;
            s_rqe <= rram_e(to_integer(s_rae));
        end if;
    end process;
    p_rro  : process(clk)
    begin
        if rising_edge(clk) then
            if r_we = '1' then rram_o(to_integer(r_wa)) <= r_wdo; end if;
            s_rqo <= rram_o(to_integer(s_rao));
        end if;
    end process;



    ---------------------------------------------------------------------------
    -- Controls, vblank zone, glides, per-field sequencer
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s : signed(10 downto 0);
        variable v_t : unsigned(12 downto 0);
        variable v_k : unsigned(9 downto 0);
        variable v_g : signed(14 downto 0);
        variable v_a : signed(10 downto 0);
        variable v_q : signed(26 downto 0);
        variable v_u : unsigned(24 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_kx  <= f_kx(s_tid);
            s_sdm <= f_sd(s_tid);
            s_ys  <= f_ys(s_tid);

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            g_wm1 <= s_w - 1;
            s_linit <= '0';

            -- glides on K1, K2, K3, K5, P12 (1/8 of the gap per field, +-4 deadband, snap on the first field)
            s_zgo2 <= s_zgo; s_zgo3 <= s_zgo2;
            if s_zgo = '1' then
                s_gd(0) <= signed(resize(unsigned(registers_in(0)) & "000", 15)) - signed(resize(g1, 15));
                s_gd(1) <= signed(resize(unsigned(registers_in(1)) & "000", 15)) - signed(resize(g2, 15));
                s_gd(2) <= signed(resize(unsigned(registers_in(2)) & "000", 15)) - signed(resize(g3, 15));
                s_gd(3) <= signed(resize(unsigned(registers_in(4)) & "000", 15)) - signed(resize(g5, 15));
                s_gd(4) <= signed(resize(unsigned(registers_in(7)) & "000", 15)) - signed(resize(g12, 15));
            end if;
            if s_zgo2 = '1' then
                v_g := signed(resize(g1, 15)) + shift_right(s_gd(0), 3); s_gn(0) <= unsigned(v_g(12 downto 0));
                v_g := signed(resize(g2, 15)) + shift_right(s_gd(1), 3); s_gn(1) <= unsigned(v_g(12 downto 0));
                v_g := signed(resize(g3, 15)) + shift_right(s_gd(2), 3); s_gn(2) <= unsigned(v_g(12 downto 0));
                v_g := signed(resize(g5, 15)) + shift_right(s_gd(3), 3); s_gn(3) <= unsigned(v_g(12 downto 0));
                v_g := signed(resize(g12, 15)) + shift_right(s_gd(4), 3); s_gn(4) <= unsigned(v_g(12 downto 0));
                for i in 0 to 4 loop
                    if s_gd(i) > 31 or s_gd(i) < -31 then s_gb(i) <= '1'; else s_gb(i) <= '0'; end if;
                end loop;
            end if;
            if s_zgo3 = '1' then
                s_warm <= '1';
                if s_warm = '0' then
                    g1 <= unsigned(registers_in(0)) & "000"; g2 <= unsigned(registers_in(1)) & "000";
                    g3 <= unsigned(registers_in(2)) & "000"; g5 <= unsigned(registers_in(4)) & "000";
                    g12 <= unsigned(registers_in(7)) & "000";
                else
                    if s_gb(0) = '1' then g1 <= s_gn(0); end if;
                    if s_gb(1) = '1' then g2 <= s_gn(1); end if;
                    if s_gb(2) = '1' then g3 <= s_gn(2); end if;
                    if s_gb(3) = '1' then g5 <= s_gn(3); end if;
                    if s_gb(4) = '1' then g12 <= s_gn(4); end if;
                end if;
                f_st <= to_unsigned(1, 6);
            end if;

            if s_zgo = '1' then
                v_s := signed(resize(unsigned(registers_in(3)), 11)) - 512;          -- K4 Speed
                if v_s > -8 and v_s < 8 then v_s := (others => '0'); end if;
                s_sp <= v_s;
                v_a := -v_s;
                if v_s < 0 then s_asp <= unsigned(v_a(9 downto 0)); else s_asp <= unsigned(v_s(9 downto 0)); end if;
                -- K6 patch: (k6 * 3) >> 9 -> 0..5; a labelled default arrives as its index
                v_t := resize(unsigned(registers_in(5)), 13) + resize(unsigned(registers_in(5)) & '0', 13);
                if unsigned(registers_in(5)) < 6 then
                    s_patch <= unsigned(registers_in(5)(2 downto 0));
                else
                    s_patch <= v_t(11 downto 9);
                end if;
                s_mirror <= registers_in(6)(0);
                s_nostr  <= registers_in(6)(1);
                s_noblk  <= registers_in(6)(2);
                s_flip   <= registers_in(6)(3);
                s_osc    <= registers_in(6)(4);
                if s_wmax /= 0 then s_w <= s_wmax; end if;
                s_wmax <= (others => '0');
            end if;

            -- per-field sequencer: one op per state; the multiplier steps while f_mc /= 0
            if f_mc /= 0 then
                if f_mb(0) = '1' then f_acc <= f_acc + f_ma; end if;
                f_ma <= f_ma(32 downto 0) & '0';
                f_mb <= '0' & f_mb(12 downto 1);
                f_mc <= f_mc - 1;
            else
                case to_integer(f_st) is
                    -- K1 Stripes: sf = 256 -/+ (d^2 >> 9) * (128 | 256) >> 9
                    when 1 =>
                        v_k := g1(12 downto 3);
                        if v_k(9) = '1' then f_hi <= '1'; f_d <= v_k - 512; else f_hi <= '0'; f_d <= 512 - v_k; end if;
                        f_st <= f_st + 1;
                    when 2 =>
                        f_ma <= resize(f_d, 34); f_mb <= resize(f_d, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 3 =>
                        f_ma <= resize(f_acc(18 downto 9), 34);
                        if f_hi = '1' then f_mb <= to_unsigned(256, 13); else f_mb <= to_unsigned(128, 13); end if;
                        f_acc <= (others => '0'); f_mc <= to_unsigned(9, 4);
                        f_st <= f_st + 1;
                    when 4 =>
                        if f_hi = '1' then c_sf <= to_unsigned(256, 10) + resize(f_acc(17 downto 9), 10);
                        else c_sf <= to_unsigned(256, 10) - resize(f_acc(17 downto 9), 10); end if;
                        -- K2 Width (reversed): iw = 256 -/+ (d^2 >> 9) * (96 | 171) >> 9
                        v_k := not g2(12 downto 3);
                        if v_k(9) = '1' then f_hi <= '1'; f_d <= v_k - 512; else f_hi <= '0'; f_d <= 512 - v_k; end if;
                        f_st <= f_st + 1;
                    when 5 =>
                        f_ma <= resize(f_d, 34); f_mb <= resize(f_d, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 6 =>
                        f_ma <= resize(f_acc(18 downto 9), 34);
                        if f_hi = '1' then f_mb <= to_unsigned(171, 13); else f_mb <= to_unsigned(96, 13); end if;
                        f_acc <= (others => '0'); f_mc <= to_unsigned(8, 4);
                        f_st <= f_st + 1;
                    when 7 =>
                        if f_hi = '1' then c_iw <= to_unsigned(256, 10) + resize(f_acc(17 downto 9), 10);
                        else c_iw <= to_unsigned(256, 10) - resize(f_acc(17 downto 9), 10); end if;
                        f_st <= f_st + 1;
                    when 8 =>       -- kx * iw
                        f_ma <= resize(s_kx, 34); f_mb <= resize(c_iw, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 9 =>       -- dxk = (kx iw * 4062) >> 16
                        f_ma <= resize(f_acc(18 downto 0), 34); f_mb <= to_unsigned(4062, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(12, 4);
                        f_st <= f_st + 1;
                    when 10 =>      -- xk0 = CX - (Dk iw) >> 8
                        c_dxk <= f_acc(31 downto 16);
                        if s_sdm = '1' then f_ma <= to_unsigned(3249500, 34); else f_ma <= to_unsigned(4224341, 34); end if;
                        f_mb <= resize(c_iw, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 11 =>      -- ds = (kx sf) >> 2
                        c_xk0 <= to_signed(C_CX, 26) - signed(resize(f_acc(31 downto 8), 26));
                        f_ma <= resize(s_kx, 34); f_mb <= resize(c_sf, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 12 =>      -- s0 = 15728640 - sf * (61440 | 46080) + drift (mod 480 samples)
                        c_ds <= f_acc(18 downto 2);
                        if s_sdm = '1' then f_ma <= to_unsigned(46080, 34); else f_ma <= to_unsigned(61440, 34); end if;
                        f_mb <= resize(c_sf, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 13 =>
                        f_t <= to_signed(15728640, 27) - signed(resize(f_acc(24 downto 0), 27)) + signed(resize(s_sdr, 27));
                        f_st <= f_st + 1;
                    when 14 =>
                        if f_t < 0 then f_t <= f_t + C_SM;
                        elsif f_t >= C_SM then f_t <= f_t - C_SM; end if;
                        f_st <= f_st + 1;
                    when 15 =>      -- dyk = (ys * 4005) >> 8
                        c_s0 <= unsigned(f_t(24 downto 0));
                        f_ma <= resize(s_ys, 34); f_mb <= to_unsigned(4005, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(12, 4);
                        f_st <= f_st + 1;
                    when 16 =>      -- P12 Glass: gain g and ring kr
                        c_dyk <= f_acc(23 downto 8);
                        v_k := g12(12 downto 3);
                        if v_k(9) = '0' then
                            c_g <= to_unsigned(64, 9) + resize(shift_right(resize(v_k, 12) + resize(v_k & '0', 12), 3), 9);
                            f_ma <= (others => '0');
                        else
                            c_g <= to_unsigned(256, 9) + resize(v_k(8 downto 2), 9);
                            f_ma <= resize(v_k(8 downto 0), 34);
                        end if;
                        f_mb <= to_unsigned(219, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(8, 4);
                        f_st <= f_st + 1;
                    when 17 =>      -- km = ((k5 g) >> 6) * 7 >> 8
                        c_kr <= f_acc(16 downto 9);
                        f_ma <= resize(g5(12 downto 3), 34); f_mb <= resize(c_g, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(9, 4);
                        f_st <= f_st + 1;
                    when 18 =>
                        f_ma <= resize(f_acc(18 downto 6), 34); f_mb <= to_unsigned(7, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(3, 4);
                        f_st <= f_st + 1;
                    when 19 =>      -- ka = (k5 * 410) >> 10
                        c_km <= f_acc(15 downto 8);
                        f_ma <= resize(g5(12 downto 3), 34); f_mb <= to_unsigned(410, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(9, 4);
                        f_st <= f_st + 1;
                    when 20 =>      -- kv, hue; speed^2
                        c_ka <= f_acc(18 downto 10);
                        c_kv <= resize(g5(12 downto 4), 9);
                        c_hue <= g3(12 downto 3) + s_hdr(17 downto 8);
                        f_ma <= resize(s_asp, 34); f_mb <= resize(s_asp, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(10, 4);
                        f_st <= f_st + 1;
                    when 21 =>      -- hue drift step (m * 210) >> 14
                        f_m <= f_acc(17 downto 0);
                        f_ma <= resize(f_acc(17 downto 0), 34); f_mb <= to_unsigned(210, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(8, 4);
                        f_st <= f_st + 1;
                    when 22 =>      -- stripe drift m >> 1 (mod 480 samples); osc x step (m * 3277) >> 14
                        if s_sp < 0 then s_hdr <= s_hdr - resize(f_acc(25 downto 14), 18);
                        else s_hdr <= s_hdr + resize(f_acc(25 downto 14), 18); end if;
                        if s_sp < 0 then
                            v_q := signed(resize(s_sdr, 27)) - signed(resize(f_m(17 downto 1), 27));
                        else
                            v_q := signed(resize(s_sdr, 27)) + signed(resize(f_m(17 downto 1), 27));
                        end if;
                        f_t <= v_q;
                        f_ma <= resize(f_m, 34); f_mb <= to_unsigned(3277, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(12, 4);
                        f_st <= f_st + 1;
                    when 23 =>      -- osc y step (m * 2621) >> 14
                        if f_t < 0 then v_u := unsigned(resize(f_t + C_SM, 25));
                        elsif f_t >= C_SM then v_u := unsigned(resize(f_t - C_SM, 25));
                        else v_u := unsigned(f_t(24 downto 0)); end if;
                        s_sdr <= v_u;
                        if s_sp < 0 then s_oxt <= s_oxt - f_acc(29 downto 14); else s_oxt <= s_oxt + f_acc(29 downto 14); end if;
                        f_ma <= resize(f_m, 34); f_mb <= to_unsigned(2621, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(12, 4);
                        f_st <= f_st + 1;
                    when 24 =>      -- osc steps: x (93 kx) >> 8 per px
                        if s_sp < 0 then s_oyt <= s_oyt + f_acc(29 downto 14); else s_oyt <= s_oyt - f_acc(29 downto 14); end if;
                        f_ma <= resize(s_kx, 34); f_mb <= to_unsigned(93, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(7, 4);
                        f_st <= f_st + 1;
                    when 25 =>      -- y (131 ys) >> 8 per line
                        c_osx <= f_acc(15 downto 8);
                        f_ma <= resize(s_ys, 34); f_mb <= to_unsigned(131, 13); f_acc <= (others => '0'); f_mc <= to_unsigned(8, 4);
                        f_st <= f_st + 1;
                    when 26 =>
                        c_osy <= f_acc(17 downto 8);
                        f_st <= f_st + 1;
                    when 27 =>
                        s_linit <= '1';            -- start the line engine on line 0
                        f_st <= (others => '0');
                    when others =>
                        null;
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
    -- Generated avid (parity-locked start, field-latched width)
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
            g_dst <= g_dst(12 downto 0) & g_start;
            g_dav <= g_dav(12 downto 0) & g_avid;
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- Per-line engine: at the source avid rise it commits the values computed for this line and
    -- starts on the next one (a few clocks), then kicks the row engine.
    --   j, ty     the knot row and its 5-bit fraction (yk = line * dyk, or flipped from the bottom)
    --   rb0, rb1  the even / odd row bases in the knot banks, (row >> 1) * 60
    --   xe        the line-start block's right edge (virtual px): the round-ended block around
    --             line 984, the stubs at 176..215 and 330..371
    --   bank      the row RAM half this line reads (the engine fills the other one)
    ---------------------------------------------------------------------------
    p_line : process(clk)
        variable v_r : unsigned(6 downto 0);
        variable v_yk : signed(23 downto 0);
    begin
        if rising_edge(clk) then
            s_kick_c <= data_in.avid and not s_avq;
            s_kick_e <= data_in.avid and not s_avq;
            if s_kick_c = '1' then
                l_xe <= n_xe; l_purp <= n_purp; l_blank <= n_blank; l_oy <= n_oy; l_bank <= n_bank;
            end if;
            r_go <= '0';

            if s_linit = '1' then
                e_yv8 <= (others => '0');
                if s_flip = '1' then e_yk <= to_signed(4321443, 24); else e_yk <= (others => '0'); end if;
                e_oyl <= (others => '0');
                n_bank <= '0';
                e_st <= to_unsigned(2, 3);
            elsif s_kick_e = '1' then
                e_st <= to_unsigned(1, 3);
            else
                case to_integer(e_st) is
                    when 1 =>       -- advance to the next line
                        e_yv8 <= e_yv8 + resize(s_ys, 20);
                        if s_flip = '1' then e_yk <= e_yk - signed(resize(c_dyk, 24));
                        else e_yk <= e_yk + signed(resize(c_dyk, 24)); end if;
                        e_oyl <= e_oyl + resize(c_osy, 16);
                        n_bank <= not n_bank;
                        e_st <= e_st + 1;
                    when 2 =>       -- row, fraction; virtual line
                        if e_yk < 0 then v_yk := (others => '0'); else v_yk := e_yk; end if;
                        if v_yk(23 downto 16) > 65 then
                            e_j <= to_unsigned(65, 7); n_ty <= to_unsigned(31, 5);
                        else
                            e_j <= unsigned(v_yk(22 downto 16)); n_ty <= unsigned(v_yk(15 downto 11));
                        end if;
                        if e_yv8(19 downto 8) > 1079 then e_yv <= to_unsigned(1079, 11); else e_yv <= e_yv8(18 downto 8); end if;
                        n_oy <= e_oyl(15 downto 8) + s_oyt(15 downto 8);
                        e_st <= e_st + 1;
                    when 3 =>       -- row bases; |y - 984|; stubs
                        n_j <= e_j;
                        v_r := (e_j + 1) srl 1;
                        n_rb0 <= shift_left(resize(v_r, 11), 6) - shift_left(resize(v_r, 11), 2);
                        v_r := e_j srl 1;
                        n_rb1 <= shift_left(resize(v_r, 11), 6) - shift_left(resize(v_r, 11), 2);
                        if e_yv >= 984 then e_dy <= e_yv - 984; else e_dy <= 984 - e_yv; end if;
                        if (e_yv >= 176 and e_yv < 216) or (e_yv >= 330 and e_yv < 372) then e_stub <= '1'; else e_stub <= '0'; end if;
                        if (e_yv >= 188 and e_yv < 205) or (e_yv >= 344 and e_yv < 350) then e_purp <= '1'; else e_purp <= '0'; end if;
                        if e_yv >= 1073 then e_blank <= '1'; else e_blank <= '0'; end if;
                        e_st <= e_st + 1;
                    when 4 =>       -- block edge; start the row engine
                        if e_stub = '1' then
                            n_xe <= to_unsigned(52, 8);
                        elsif e_dy <= 73 then
                            n_xe <= unsigned(C_DE(to_integer(e_dy(6 downto 0))));
                        else
                            n_xe <= (others => '0');
                        end if;
                        n_purp <= e_purp; n_blank <= e_blank;
                        r_go <= '1';
                        e_st <= (others => '0');
                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Row engine: the next line's 120 knots interpolated vertically (60 column pairs, one per
    -- clock from the four banks), angle 10 bits (wrap-aware) | magnitude 6 bits, into the free
    -- half of the row RAMs
    ---------------------------------------------------------------------------
    p_row : process(clk)
        variable v_a : unsigned(4 downto 0);
        variable v_x : unsigned(9 downto 0);
        variable v_m : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: knot addresses (both row parities, both column parities)
            if r_go = '1' then
                r_run <= '1'; r_p <= (others => '0');
                r_jp <= n_j(0); r_ty <= n_ty; r_bk <= n_bank;
            elsif r_run = '1' then
                if r_p = 59 then r_run <= '0'; end if;
                r_p <= r_p + 1;
            end if;
            s_ka00 <= n_rb0 + resize(r_p, 11); s_ka01 <= n_rb0 + resize(r_p, 11);
            s_ka10 <= n_rb1 + resize(r_p, 11); s_ka11 <= n_rb1 + resize(r_p, 11);
            r_v(1) <= r_run and not r_go; r_pp(1) <= r_p;
            -- 2: ROM data
            r_v(2) <= r_v(1); r_pp(2) <= r_pp(1);
            -- 3: top / bottom by row parity (fabric registers)
            if r_jp = '0' then
                r_te <= unsigned(s_kq00); r_be <= unsigned(s_kq10); r_to <= unsigned(s_kq01); r_bo <= unsigned(s_kq11);
            else
                r_te <= unsigned(s_kq10); r_be <= unsigned(s_kq00); r_to <= unsigned(s_kq11); r_bo <= unsigned(s_kq01);
            end if;
            r_v(3) <= r_v(2); r_pp(3) <= r_pp(2);
            -- 4: (bottom - top) * ty
            v_a := r_be(7 downto 3) - r_te(7 downto 3);
            r_pae <= resize(signed(v_a) * signed(resize(r_ty, 6)), 11);
            v_a := r_bo(7 downto 3) - r_to(7 downto 3);
            r_pao <= resize(signed(v_a) * signed(resize(r_ty, 6)), 11);
            r_pme <= resize((signed(resize(r_be(2 downto 0), 4)) - signed(resize(r_te(2 downto 0), 4))) * signed(resize(r_ty, 6)), 10);
            r_pmo <= resize((signed(resize(r_bo(2 downto 0), 4)) - signed(resize(r_to(2 downto 0), 4))) * signed(resize(r_ty, 6)), 10);
            r_ate <= r_te(7 downto 3); r_ato <= r_to(7 downto 3);
            r_mte <= r_te(2 downto 0); r_mto <= r_to(2 downto 0);
            r_v(4) <= r_v(3); r_pp(4) <= r_pp(3);
            -- 5: sums -> the row RAMs
            v_x := shift_left(resize(r_ate, 10), 5) + unsigned(r_pae(9 downto 0));
            v_m := signed(resize(shift_left(resize(r_mte, 8), 5), 11)) + resize(r_pme, 11);
            r_wde <= std_logic_vector(v_x) & std_logic_vector(v_m(7 downto 2));
            v_x := shift_left(resize(r_ato, 10), 5) + unsigned(r_pao(9 downto 0));
            v_m := signed(resize(shift_left(resize(r_mto, 8), 5), 11)) + resize(r_pmo, 11);
            r_wdo <= std_logic_vector(v_x) & std_logic_vector(v_m(7 downto 2));
            r_we <= r_v(4);
            r_wa <= r_bk & r_pp(4);
        end if;
    end process p_row;

    ---------------------------------------------------------------------------
    -- Beams: the glass field x (stage 0), the oscillator x (stage 7), the stripe position and
    -- the virtual x (stage 10), each loaded at its own delayed g_start
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_s : unsigned(25 downto 0);
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_xk <= c_xk0;
            elsif g_avid = '1' then
                b_xk <= b_xk + signed(resize(c_dxk, 26));
            end if;
            if g_dst(6) = '1' then
                b_ox <= s_oxt;
            elsif g_dav(6) = '1' then
                b_ox <= b_ox + resize(c_osx, 16);
            end if;
            if g_dst(9) = '1' then
                b_s <= c_s0;
                if s_sdm = '1' then b_xv <= to_unsigned(240 * 256, 20); else b_xv <= (others => '0'); end if;
            elsif g_dav(9) = '1' then
                v_s := resize(b_s, 26) + resize(c_ds, 26);
                if v_s >= C_SM then b_s <= resize(v_s - C_SM, 25); else b_s <= v_s(24 downto 0); end if;
                b_xv <= b_xv + resize(s_kx, 20);
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Source: camera luma (or the oscillator) as a signed deviation sd, -128 .. 127
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_d : signed(10 downto 0);
        variable v_e : signed(10 downto 0);
        variable v_t : signed(8 downto 0);
    begin
        if rising_edge(clk) then
            v_y1 <= unsigned(data_in.y);
            v_d := signed(resize(v_y1, 11)) - 502;
            v_e := shift_right(v_d, 2) + shift_right(v_d, 5);
            if v_e > 127 then v_sd2 <= to_signed(127, 9);
            elsif v_e < -128 then v_sd2 <= to_signed(-128, 9);
            else v_sd2 <= v_e(8 downto 0); end if;
            -- oscillator: (tri(x) - 64 + tri(y) - 64) >> 3   (b_ox is at stage 7)
            o_t8 <= f_tri8(b_ox(15 downto 8));
            v_t := signed(resize(o_t8, 9)) + signed(resize(f_tri8(l_oy), 9)) - 128;
            o_s9 <= shift_right(v_t, 3);
            if s_osc = '1' then p_sd10 <= o_s9; else p_sd10 <= v_sd2; end if;
            p_sd11 <= p_sd10; p_sd12 <= p_sd11;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- Glass path (stage numbers = clocks after the beam register)
    ---------------------------------------------------------------------------
    p_glass : process(clk)
        variable v_x : signed(14 downto 0);
        variable v_c : unsigned(6 downto 0);
        variable v_d : unsigned(9 downto 0);
        variable v_a : signed(15 downto 0);
        variable v_m : signed(9 downto 0);
        variable v_s : signed(16 downto 0);
        variable v_q : signed(11 downto 0);
        variable v_w : signed(13 downto 0);
        variable v_ang : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 1..3: Q5 knots; mirror about the axis (S7 folds the right half onto the left)
            p_d1 <= b_xk(25 downto 11) - to_signed(C_CX5, 15); p_xk1 <= b_xk(25 downto 11);
            if p_d1 < 0 then p_ad2 <= unsigned(0 - p_d1(13 downto 0)); else p_ad2 <= unsigned(p_d1(13 downto 0)); end if;
            p_xk2 <= p_xk1;
            if s_mirror = '1' then p_xm3 <= to_signed(C_CX5, 15) - signed(resize(p_ad2, 15)); else p_xm3 <= p_xk2; end if;
            -- 4..5: reflect at the field's edges (0 and 119 knots)
            p_xm4 <= p_xm3;
            p_ng4 <= 0 - p_xm3;
            p_hi4 <= to_signed(238 * 32, 15) - p_xm3;
            if p_xm3 < 0 then p_lt4 <= '1'; else p_lt4 <= '0'; end if;
            if p_xm3 > 119 * 32 then p_gt4 <= '1'; else p_gt4 <= '0'; end if;
            if p_lt4 = '1' then v_x := p_ng4; elsif p_gt4 = '1' then v_x := p_hi4; else v_x := p_xm4; end if;
            p_xr5 <= unsigned(v_x(11 downto 0));
            -- 6: knot column and fraction
            if p_xr5(11 downto 5) > 118 then
                p_i6 <= to_unsigned(118, 7); p_tx6 <= to_unsigned(31, 5);
            else
                p_i6 <= p_xr5(11 downto 5); p_tx6 <= p_xr5(4 downto 0);
            end if;
            -- 7: row RAM addresses (this line's half)
            v_c := (p_i6 + 1) srl 1;
            s_rae <= l_bank & v_c(5 downto 0);
            v_c := p_i6 srl 1;
            s_rao <= l_bank & v_c(5 downto 0);
            p_pi7 <= p_i6(0); p_tx7 <= p_tx6;
            -- 8: RAM data
            p_pi8 <= p_pi7; p_tx8 <= p_tx7;
            -- 9: left (column i) / right (column i + 1) by column parity
            if p_pi8 = '0' then
                p_al9 <= unsigned(s_rqe(15 downto 6)); p_ml9 <= unsigned(s_rqe(5 downto 0));
                p_ar9 <= unsigned(s_rqo(15 downto 6)); p_mr9 <= unsigned(s_rqo(5 downto 0));
            else
                p_al9 <= unsigned(s_rqo(15 downto 6)); p_ml9 <= unsigned(s_rqo(5 downto 0));
                p_ar9 <= unsigned(s_rqe(15 downto 6)); p_mr9 <= unsigned(s_rqe(5 downto 0));
            end if;
            p_tx9 <= p_tx8;
            -- 10: wrap-aware angle difference, magnitude difference
            v_d := p_ar9 - p_al9; p_da10 <= signed(v_d);
            p_dm10 <= signed(resize(p_mr9, 7)) - signed(resize(p_ml9, 7));
            p_al10 <= p_al9; p_ml10 <= p_ml9; p_tx10 <= p_tx9;
            -- 11: products by tx
            p_pa11 <= resize(p_da10 * signed(resize(p_tx10, 6)), 16);
            p_pm11 <= resize(p_dm10 * signed(resize(p_tx10, 6)), 13);
            p_al11 <= p_al10; p_ml11 <= p_ml10;
            -- 12: A0 (1/1024 turn), M8 (0 .. 224)
            v_a := shift_right(p_pa11, 5);
            p_a12 <= p_al11 + unsigned(v_a(9 downto 0));
            v_w := signed(resize(shift_left(resize(p_ml11, 9), 2), 14)) + resize(shift_right(p_pm11, 3), 14);
            if v_w < 0 then p_m12 <= (others => '0'); else p_m12 <= unsigned(v_w(8 downto 0)); end if;
            -- 13: gain and source products; angle + hue
            p_mg13 <= p_m12 * c_g(8 downto 2);
            p_ms13 <= resize(p_sd12(8 downto 2) * signed(resize(c_km, 9)), 16);
            p_aa13 <= p_a12 + c_hue;
            p_as13 <= resize(p_sd12(8 downto 2) * signed(resize(c_ka(8 downto 3), 7)), 14);
            -- 14: Mg = (M8 g7 + s7 km) >> 6 ; A += (s7 ka6) >> 3
            v_s := signed(resize(p_mg13, 17)) + resize(p_ms13, 17);
            v_s := shift_right(v_s, 6);
            p_mg14 <= v_s(10 downto 0);
            v_w := shift_right(p_as13, 3);
            p_ab14 <= p_aa13 + unsigned(v_w(9 downto 0));
            -- 15: clamp; magnitude index
            if p_mg14 < 0 then p_mc15 <= (others => '0');
            elsif p_mg14 > 224 then p_mc15 <= to_unsigned(224, 8);
            else p_mc15 <= unsigned(p_mg14(7 downto 0)); end if;
            v_q := resize(p_mg14, 12) + 16;
            if v_q < 0 then p_mi15 <= (others => '0');
            elsif v_q > 255 then p_mi15 <= to_unsigned(7, 3);
            else p_mi15 <= unsigned(v_q(7 downto 5)); end if;
            p_ab15 <= p_ab14;
            -- 16: ring product (magnitude -> hue)
            p_rp16 <= p_mc15 * c_kr(7 downto 2);
            p_mi16 <= p_mi15; p_ab16 <= p_ab15;
            -- 17: LUT address
            v_ang := p_ab16 + p_rp16(12 downto 3);
            s_la <= v_ang(9 downto 4) & p_mi16;
            -- 18: LUT data; 19: fabric register
            p_l19 <= s_lq;
        end if;
    end process p_glass;

    ---------------------------------------------------------------------------
    -- Stripe path (stripe stage k = glass stage 10 + k): the stripe oscillators (bent by the
    -- source), or inside the line-start block the 8-px ramp from the spare entries of the banks
    ---------------------------------------------------------------------------
    p_stripe : process(clk)
        variable v_e, v_o : unsigned(8 downto 0);
        variable v_x : unsigned(11 downto 0);
        variable v_p : signed(12 downto 0);
        variable v_q : signed(11 downto 0);
        variable v_u : unsigned(15 downto 0);
        variable v_si : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: position (Q3 samples), the source's bend, block test
            q_pos1 <= b_s(24 downto 13);
            q_off1 <= resize(p_sd10(8 downto 2) * signed(resize(c_kv(8 downto 3), 7)), 14);
            v_x := b_xv(19 downto 8);
            if s_noblk = '0' and v_x < resize(l_xe, 12) then q_in1 <= '1'; else q_in1 <= '0'; end if;
            q_r1 <= v_x(7 downto 3); q_rf1 <= v_x(2 downto 0);
            -- 2: bent position, mod 480 samples
            v_p := signed(resize(q_pos1, 13)) + resize(shift_right(q_off1, 4), 13);
            if v_p < 0 then v_p := v_p + 3840; elsif v_p >= 3840 then v_p := v_p - 3840; end if;
            q_p2 <= unsigned(v_p(11 downto 0));
            q_in2 <= q_in1; q_r2 <= q_r1; q_rf2 <= q_rf1;
            -- 3: bank addresses
            if q_in2 = '1' then
                s_sea <= to_unsigned(241, 8) + resize((q_r2 + 1) srl 1, 8);
                s_soa <= to_unsigned(240, 8) + resize(q_r2 srl 1, 8);
                q_par3 <= q_r2(0); q_f3 <= q_rf2;
            else
                v_si := q_p2(11 downto 3);
                v_e := (v_si + 1) srl 1; v_o := v_si srl 1;
                s_sea <= v_e(7 downto 0); s_soa <= v_o(7 downto 0);
                q_par3 <= v_si(0); q_f3 <= q_p2(2 downto 0);
            end if;
            q_in3 <= q_in2;
            -- 4: ROM data; 5: fabric registers
            q_par4 <= q_par3; q_f4 <= q_f3; q_in4 <= q_in3;
            q_e5 <= s_seq; q_o5 <= s_soq;
            q_par5 <= q_par4; q_f5 <= q_f4; q_in5 <= q_in4;
            -- 6: v0 / v1 by parity
            for c in 0 to 2 loop
                if q_par5 = '0' then
                    q_v06(c) <= unsigned(q_e5(23 - 8 * c downto 16 - 8 * c));
                    q_dv6(c) <= signed(resize(unsigned(q_o5(23 - 8 * c downto 16 - 8 * c)), 9))
                                - signed(resize(unsigned(q_e5(23 - 8 * c downto 16 - 8 * c)), 9));
                else
                    q_v06(c) <= unsigned(q_o5(23 - 8 * c downto 16 - 8 * c));
                    q_dv6(c) <= signed(resize(unsigned(q_e5(23 - 8 * c downto 16 - 8 * c)), 9))
                                - signed(resize(unsigned(q_o5(23 - 8 * c downto 16 - 8 * c)), 9));
                end if;
            end loop;
            q_f6 <= q_f5; q_in6 <= q_in5;
            -- 7: (v1 - v0) f
            for c in 0 to 2 loop
                q_pd7(c) <= resize(q_dv6(c) * signed(resize(q_f6, 4)), 12);
                q_v07(c) <= q_v06(c);
            end loop;
            q_in7 <= q_in6;
            -- 8: S = v0 + (..) >> 3
            for c in 0 to 2 loop
                v_q := signed(resize(q_v07(c), 12)) + shift_right(q_pd7(c), 3);
                q_sv8(c) <= unsigned(v_q(7 downto 0));
            end loop;
            q_in8 <= q_in7;
            -- 9: stripes (or grey with S8 off) / the block's ramp (purple-tinted on the stub rows)
            if q_in8 = '1' then
                if l_purp = '1' then
                    v_u := resize(q_sv8(0), 16) + 120;
                    if v_u > 255 then q_out9(0) <= to_unsigned(255, 8); else q_out9(0) <= v_u(7 downto 0); end if;
                    v_u := q_sv8(1) * to_unsigned(38, 8);
                    q_out9(1) <= v_u(15 downto 8);
                else
                    q_out9(0) <= q_sv8(0); q_out9(1) <= q_sv8(1);
                end if;
                q_out9(2) <= q_sv8(2);
            elsif s_nostr = '1' then
                q_out9(0) <= to_unsigned(70, 8); q_out9(1) <= to_unsigned(70, 8); q_out9(2) <= to_unsigned(80, 8);
            else
                q_out9 <= q_sv8;
            end if;
            q_in9 <= q_in8;
        end if;
    end process p_stripe;

    ---------------------------------------------------------------------------
    -- Composite, patch, RGB -> YUV, output conditioning
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_l : signed(9 downto 0);
        variable v_t : signed(10 downto 0);
        variable v_rr, v_gg, v_bv : unsigned(7 downto 0);
        variable v_y : unsigned(16 downto 0);
        variable v_cc : signed(16 downto 0);
        variable v_sc : unsigned(13 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 20: stripes + 2 x glass (clamped), the block, the blanked bottom rows
            for c in 0 to 2 loop
                v_l := resize(signed(p_l19(23 - 8 * c downto 16 - 8 * c)), 10);
                v_t := signed(resize(q_out9(c), 11)) + shift_left(resize(v_l, 11), 1);
                if l_blank = '1' then
                    p_rgb20(c) <= to_unsigned(4, 8);
                elsif q_in9 = '1' then
                    p_rgb20(c) <= q_out9(c);
                elsif v_t < 0 then
                    p_rgb20(c) <= (others => '0');
                elsif v_t > 255 then
                    p_rgb20(c) <= to_unsigned(255, 8);
                else
                    p_rgb20(c) <= unsigned(v_t(7 downto 0));
                end if;
            end loop;

            -- 21: RGB cable patch
            case to_integer(s_patch) is
                when 1      => p_o21(0) <= p_rgb20(0); p_o21(1) <= p_rgb20(2); p_o21(2) <= p_rgb20(1);
                when 2      => p_o21(0) <= p_rgb20(1); p_o21(1) <= p_rgb20(0); p_o21(2) <= p_rgb20(2);
                when 3      => p_o21(0) <= p_rgb20(1); p_o21(1) <= p_rgb20(2); p_o21(2) <= p_rgb20(0);
                when 4      => p_o21(0) <= p_rgb20(2); p_o21(1) <= p_rgb20(0); p_o21(2) <= p_rgb20(1);
                when 5      => p_o21(0) <= p_rgb20(2); p_o21(1) <= p_rgb20(1); p_o21(2) <= p_rgb20(0);
                when others => p_o21 <= p_rgb20;
            end case;

            -- 22..24: RGB -> 10-bit YUV (u = Cr, v = Cb), Q6; luma x1.07:
            -- Y = 64 + (70 R + 138 G + 27 B) / 64, Cr = 512 + (113 R - 94 G - 18 B) / 64,
            -- Cb = 512 + (-38 R - 75 G + 113 B) / 64
            v_rr := p_o21(0); v_gg := p_o21(1); v_bv := p_o21(2);
            p_yr22 <= shift_left(resize(v_rr, 16), 6) + shift_left(resize(v_rr, 16), 2) + shift_left(resize(v_rr, 16), 1);
            p_yg22 <= shift_left(resize(v_gg, 16), 7) + shift_left(resize(v_gg, 16), 3) + shift_left(resize(v_gg, 16), 1);
            p_yb22 <= shift_left(resize(v_bv, 16), 5) - shift_left(resize(v_bv, 16), 2) - resize(v_bv, 16);
            p_ur22 <= signed(shift_left(resize(v_rr, 17), 7) - shift_left(resize(v_rr, 17), 4) + resize(v_rr, 17));
            p_ug22 <= -signed(shift_left(resize(v_gg, 17), 6) + shift_left(resize(v_gg, 17), 5) - shift_left(resize(v_gg, 17), 1));
            p_ub22 <= -signed(shift_left(resize(v_bv, 17), 4) + shift_left(resize(v_bv, 17), 1));
            p_vr22 <= -signed(shift_left(resize(v_rr, 17), 5) + shift_left(resize(v_rr, 17), 2) + shift_left(resize(v_rr, 17), 1));
            p_vg22 <= -signed(shift_left(resize(v_gg, 17), 6) + shift_left(resize(v_gg, 17), 3) + shift_left(resize(v_gg, 17), 1) + resize(v_gg, 17));
            p_vb22 <= signed(shift_left(resize(v_bv, 17), 7) - shift_left(resize(v_bv, 17), 4) + resize(v_bv, 17));
            p_ys23 <= resize(p_yr22, 17) + resize(p_yg22, 17) + resize(p_yb22, 17);
            p_us23 <= p_ur22 + p_ug22 + p_ub22;
            p_vs23 <= p_vr22 + p_vg22 + p_vb22;
            v_y := shift_right(p_ys23, 6) + 64;
            if v_y > 940 then s_y8 <= to_unsigned(940, 10); else s_y8 <= v_y(9 downto 0); end if;
            v_cc := shift_right(p_us23, 6) + 512;
            s_u8 <= unsigned(v_cc(9 downto 0));
            v_cc := shift_right(p_vs23, 6) + 512;
            s_v8 <= unsigned(v_cc(9 downto 0));

            -- 25..27: output conditioning: [1 2 2 2 2 2 2 2 1]/16 on Y, U and V as two adjacent 8-sums
            s_csy(0) <= s_y8; s_csu(0) <= s_u8; s_csv(0) <= s_v8;
            for i in 1 to 7 loop s_csy(i) <= s_csy(i - 1); s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxy <= s_boxy + s_y8 - s_csy(7);
            s_boxu <= s_boxu + s_u8 - s_csu(7);
            s_boxv <= s_boxv + s_v8 - s_csv(7);
            s_boxyd <= s_boxy; s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxy, 14) + s_boxyd; s_y11 <= v_sc(13 downto 4);
            v_sc := resize(s_boxu, 14) + s_boxud; s_u11 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 14) + s_boxvd; s_v11 <= v_sc(13 downto 4);
            -- 28: luma toward black (940 -> 777); blanking gate on the filter-centre (stage 30) pixel's avid
            if s_y11 > 64 then v_dl := s_y11 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(29) = '1' then
                s_y12 <= s_y11 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u12 <= s_u11; s_v12 <= s_v11;
            else
                s_y12 <= to_unsigned(64, 10); s_u12 <= to_unsigned(512, 10); s_v12 <= to_unsigned(512, 10);
            end if;
            -- 29: output register (content stage 32 = C_LAT)
            s_yf <= s_y12; s_uf <= s_u12; s_vf <= s_v12;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_yf);
    data_out.u       <= std_logic_vector(s_uf);
    data_out.v       <= std_logic_vector(s_vf);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture goblet;
