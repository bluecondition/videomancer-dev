-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Escalator v0.5.0 -- Heirloom #1, rebuilt from the photo WIN_20220629_23_49_20.
--
-- The photo: a pastel grid of ~7.1 tile columns (269 px) and ~4.15 band rows
-- (260 lines).  Every tile holds a lavender left half, a grey gap, and a
-- rounded violet blob that fades pale to the right; a pink band (27 % of the
-- row) crosses every row, where the blob turns into a yellow -> orange (top
-- band: -> magenta) gradient.  Over it run green parallelogram slabs: one per
-- column, each column 1/3 of a chain period (161 lines) lower than the last,
-- slanting down 0.59 line/px over the tile's left half in ~5 stair treads and
-- running flat over the blob, thickening across the frame (100 -> 220 lines).
-- Inside a slab the colourizer switches bank: green over the left half, teal
-- -> blue over the blob (blue on the band rows), with a blue glow at the
-- slab's bottom edge, cyan rims and a cyan lead strip into the next column.
-- The partial first tile is the "sync tile": a thin wedge slab, and magenta
-- blobs with chevron band blobs in the lower half.
--
-- The patch: a line-locked tile ramp and a frame-locked band ramp; keys on
-- the tile ramp (left half / blob, the blob edge bent by the source and
-- rounded at the band edges) into a colourizer; a window comparator on a
-- slow vertical ramp minus an H staircase (column steps + clipped tile ramp
-- + treads) switches the colourizer bank.  The window's phase rides per
-- field: the slabs ride down the stairs.
--
--   K1  Columns  tile columns across, 2 .. 14 (7.14 = the photo)
--   K2  Bands    band rows down the frame, 2 .. 10 (4.15)
--   K3  Blob     blob edge, bipolar (centre = the photo): wider / narrower blobs
--   K4  Flood    slab thickness, thin threads .. green flood (square law)
--   K5  Speed    ride speed of the slabs, bipolar, still at the centre
--   K6  Bend     how far the source (camera luma / Osc) bends the blob and slab edges
--   S7  Mirror   every other column mirrored: chevron stairs
--   S8  Climb    the staircase climbs to the right instead of descending
--   S9  Invert   key invert: green bank outside the slabs, the pastel grid inside
--   S10 Vivid    every blob in the band's yellow/orange/magenta palette
--   S11 Source   Video / Osc
--   P12 RISE     escalator step: 0 = level green bars, photo = 1/3 period, 100 % = 1/2 period cascade
--
-- Count-based tiles (K1 columns across any width), vertical geometry in
-- virtual 1080-line units.  Generated avid (parity-locked start, field-latched
-- width); output conditioning (luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1]/16,
-- luma to ~777) before the blanking gate.  Latency 24 clocks in every mode.
-- Generated colours in the hardware convention (u = Cr, v = Cb), see escalator.py.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.escalator_pkg.all;
use work.all;

architecture escalator of program_top is
    constant C_LAT : integer := 24;   -- matches processing_delay_clks

    -- geometry constants (16-bit tile phase / band phase / slab phase)
    constant C_LK    : unsigned(15 downto 0) := to_unsigned(31183, 16);   -- left key end (128/269)
    constant C_UE    : unsigned(15 downto 0) := to_unsigned(30209, 16);   -- slant end (124/269)
    constant C_NUE   : unsigned(15 downto 0) := to_unsigned(35326, 16);   -- 65535 - UE (mirrored)
    constant C_OVER  : unsigned(15 downto 0) := to_unsigned(32645, 16);   -- slab blue side (lk + 6 px)
    constant C_LEADT : unsigned(15 downto 0) := to_unsigned(62125, 16);   -- lead strip (last 14/269)
    constant C_BH    : unsigned(15 downto 0) := to_unsigned(18003, 16);   -- band height (0.2747)
    constant C_BOFF  : unsigned(12 downto 0) := to_unsigned(4141, 13);    -- band blob offset (17 px)
    constant C_TP0   : unsigned(23 downto 0) := to_unsigned(3367167, 24); -- line start: 0.2007 tile
    constant C_B0    : unsigned(15 downto 0) := to_unsigned(53537, 16);   -- slab bottom at col 0 (396 lines)

    -- 8-bit colours {Y, U(=Cr), V(=Cb)}, luma pre-boosted 16/13 (see escalator.py)
    type t_c3 is array(0 to 2) of unsigned(7 downto 0);
    function c3(y, u, v : integer) return t_c3 is
    begin
        return (to_unsigned(y, 8), to_unsigned(u, 8), to_unsigned(v, 8));
    end function;
    constant C_LAV  : t_c3 := c3(200, 124, 144);
    constant C_PNK  : t_c3 := c3(205, 146, 123);
    constant C_GRY  : t_c3 := c3(204, 127, 128);
    constant C_GRYB : t_c3 := c3(208, 127, 127);
    constant C_GL   : t_c3 := c3(125, 63, 77);
    constant C_GLB  : t_c3 := c3(123, 65, 78);
    constant C_GG   : t_c3 := c3(120, 66, 80);
    constant C_GGB  : t_c3 := c3(109, 73, 98);
    constant C_GLOW : t_c3 := c3(110, 72, 144);
    constant C_CYAN : t_c3 := c3(205, 61, 150);
    constant C_LGRN : t_c3 := c3(183, 73, 84);
    constant C_DGRN : t_c3 := c3(79, 90, 103);
    constant C_MAG  : t_c3 := c3(187, 184, 158);
    constant C_SALM : t_c3 := c3(201, 172, 126);
    constant C_PALE : t_c3 := c3(248, 134, 125);

    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;

    function f_tri8(p : unsigned(15 downto 0)) return signed is
        variable v : unsigned(15 downto 0);
    begin
        -- 127 - |p - 32768| / 128
        if p(15) = '1' then v := p - 32768; else v := 32768 - p; end if;
        return to_signed(127, 9) - signed(resize(v(15 downto 7), 9));
    end function;

    -- timing
    signal t_rw   : unsigned(14 downto 0) := to_unsigned(8738, 15);
    signal t_ysk  : unsigned(16 downto 0) := to_unsigned(15534, 17);
    signal t_kyl  : unsigned(17 downto 0) := to_unsigned(34592, 18);
    signal t_cdt  : unsigned(17 downto 0) := to_unsigned(52859, 18);
    signal t_cddt : unsigned(10 downto 0) := to_unsigned(208, 11);
    signal t_kax  : unsigned(7 downto 0) := to_unsigned(57, 8);
    signal t_kbx  : unsigned(8 downto 0) := to_unsigned(150, 9);
    signal t_kay  : unsigned(9 downto 0) := to_unsigned(210, 10);
    signal t_kby  : unsigned(9 downto 0) := to_unsigned(125, 10);
    signal t_rmp  : unsigned(10 downto 0) := to_unsigned(317, 11);
    signal t_ys   : unsigned(10 downto 0) := to_unsigned(256, 11);

    -- zone / width
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc : unsigned(9 downto 0) := (others => '0');
    signal s_avq : std_logic := '0';
    signal s_hcnt, s_wmax, s_w : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- controls (glided, Q.3) and decoded magnitudes
    type t_g is array(0 to 4) of unsigned(12 downto 0);
    signal s_g : t_g := (to_unsigned(438 * 8, 13), to_unsigned(275 * 8, 13), to_unsigned(512 * 8, 13),
                         to_unsigned(615 * 8, 13), to_unsigned(681 * 8, 13));   -- power up at the defaults
    signal s_m3, s_m5 : unsigned(8 downto 0) := (others => '0');
    signal s_s3, s_s5 : std_logic := '0';
    signal s_k6 : unsigned(9 downto 0) := (others => '0');
    signal gl_k, gl_o, gl_n : unsigned(12 downto 0) := (others => '0');
    signal gl_d : signed(13 downto 0) := (others => '0');
    signal gl_w : std_logic := '0';
    signal q_gon : std_logic := '0';
    signal q_gi : unsigned(2 downto 0) := (others => '0');
    signal q_gph : unsigned(1 downto 0) := (others => '0');
    signal q_p : unsigned(4 downto 0) := (others => '0');
    signal q_ld, q_st : std_logic := '0';
    signal q_wr : std_logic_vector(1 to 18) := (others => '0');

    -- serial multiplier
    signal m_a  : unsigned(19 downto 0) := (others => '0');
    signal m_b  : unsigned(17 downto 0) := (others => '0');
    signal m_h  : unsigned(19 downto 0) := (others => '0');
    signal m_l  : unsigned(17 downto 0) := (others => '0');

    -- per-frame values
    signal f_cols  : unsigned(11 downto 0) := to_unsigned(1827, 12);
    signal f_bands : unsigned(11 downto 0) := to_unsigned(1062, 12);
    signal f_inc   : unsigned(19 downto 0) := to_unsigned(62360, 20);
    signal f_kb    : unsigned(19 downto 0) := to_unsigned(64446, 20);
    signal f_r     : unsigned(14 downto 0) := to_unsigned(21792, 15);
    signal f_tt    : unsigned(17 downto 0) := (others => '0');
    signal f_sl    : unsigned(17 downto 0) := to_unsigned(20556, 18);
    signal f_st    : unsigned(14 downto 0) := to_unsigned(9953, 15);
    signal f_sl0   : unsigned(12 downto 0) := to_unsigned(4334, 13);
    signal f_dep   : unsigned(10 downto 0) := to_unsigned(1081, 11);
    signal f_tdi   : unsigned(15 downto 0) := to_unsigned(2557, 16);
    signal f_k4sq  : unsigned(19 downto 0) := (others => '0');
    signal f_fl    : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal f_t0    : unsigned(29 downto 0) := (others => '0');
    signal f_dt0   : signed(21 downto 0) := (others => '0');
    signal f_ddt   : unsigned(12 downto 0) := (others => '0');
    signal f_ts    : unsigned(13 downto 0) := (others => '0');
    signal f_s5sq  : unsigned(17 downto 0) := (others => '0');
    signal f_spd   : signed(19 downto 0) := (others => '0');
    signal f_bo    : signed(15 downto 0) := (others => '0');
    signal f_bend  : unsigned(13 downto 0) := (others => '0');
    signal f_ride  : unsigned(23 downto 0) := (others => '0');
    signal f_ta, f_tb : unsigned(15 downto 0) := (others => '0');
    signal f_mir, f_neg, f_inv, f_viv, f_osc : std_logic := '0';

    -- per-line engine (next line) and committed line values
    signal n_vy    : unsigned(19 downto 0) := (others => '0');   -- virtual line, Q.8
    signal n_vb    : unsigned(23 downto 0) := (others => '0');   -- band phase, Q.8
    signal n_s     : unsigned(23 downto 0) := (others => '0');   -- slab phase at the line, Q.8
    signal n_e0a   : unsigned(12 downto 0) := (others => '0');
    signal n_e0i   : unsigned(5 downto 0) := (others => '0');
    signal n_y8a   : unsigned(11 downto 0) := (others => '0');
    signal n_y8q   : signed(8 downto 0) := to_signed(-8, 9);
    signal n_pa, n_pb : unsigned(15 downto 0) := (others => '0');
    signal n_ramp  : unsigned(15 downto 0) := to_unsigned(36540, 16);
    signal n_band, n_bp1, n_bp2 : std_logic := '0';
    signal n_dtop, n_seg, n_dbot : unsigned(15 downto 0) := (others => '0');
    signal n_cta, n_cba : unsigned(4 downto 0) := (others => '0');
    signal r_la : unsigned(6 downto 0) := (others => '0');
    signal r_lq, r_lq2 : std_logic_vector(15 downto 0) := (others => '0');
    signal n_ct, n_cb : unsigned(12 downto 0) := (others => '0');
    signal n_e0 : unsigned(15 downto 0) := (others => '0');
    signal n_ea, n_eb2 : signed(17 downto 0) := (others => '0');
    signal n_e, n_es : signed(17 downto 0) := (others => '0');
    signal n_low, n_pale : std_logic := '0';
    signal n_l0 : unsigned(2 downto 0) := (others => '0');
    signal n_fr : unsigned(1 downto 0) := (others => '0');
    signal le_st : unsigned(3 downto 0) := (others => '0');
    signal le_adv : std_logic := '0';

    signal l_e, l_es : signed(17 downto 0) := (others => '0');
    signal l_s : unsigned(15 downto 0) := (others => '0');
    signal l_band, l_low, l_pale : std_logic := '0';
    signal l_l0 : unsigned(2 downto 0) := (others => '0');
    signal l_fr : unsigned(1 downto 0) := (others => '0');
    signal l_pa, l_pb : unsigned(15 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- video tap (source luma -> signed bend value, delayed to beam stage 3)
    type t_s8 is array (natural range <>) of signed(7 downto 0);
    signal v_y1 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_vd : t_s8(2 to 3) := (others => (others => '0'));

    -- beam (stage 0)
    signal b_tp   : unsigned(23 downto 0) := (others => '0');
    signal b_tq   : unsigned(24 downto 0) := (others => '0');   -- b_tp + inc, one step ahead (wrap = bit 24)
    signal b_cc   : unsigned(4 downto 0) := (others => '0');
    signal b_m, b_sync : std_logic := '0';
    signal b_ca   : unsigned(15 downto 0) := (others => '0');   -- column staircase (slab phase)
    signal b_sa   : unsigned(23 downto 0) := (others => '0');   -- slant, Q.8
    signal b_td   : unsigned(15 downto 0) := (others => '0');   -- tread phase
    signal b_t    : unsigned(30 downto 0) := (others => '0');   -- thickness DDA, Q.14
    signal b_dt   : signed(26 downto 0) := (others => '0');
    signal b_tl   : unsigned(15 downto 0) := (others => '0');   -- thickness latched at the column start
    signal b_pa, b_pb : unsigned(15 downto 0) := (others => '0');

    -- pixel pipeline
    signal f_dep8 : unsigned(7 downto 0) := (others => '0');
    signal f_rq   : unsigned(15 downto 0) := (others => '0');
    signal f_tsst : signed(17 downto 0) := (others => '0');
    signal f_bend8 : unsigned(7 downto 0) := (others => '0');
    signal p1_uf, p2_uf, p3_uf, p4_uf, p5_uf : unsigned(15 downto 0) := (others => '0');
    signal p1_tpf : unsigned(15 downto 0) := (others => '0');
    signal p1_sync : std_logic := '0';
    signal p1_ca : unsigned(15 downto 0) := (others => '0');
    signal p1_sa : unsigned(15 downto 0) := (others => '0');
    signal p1_td : unsigned(3 downto 0) := (others => '0');
    signal p1_tdk : std_logic := '0';
    signal p1_tl : unsigned(15 downto 0) := (others => '0');
    signal p1_ta, p1_tb : signed(8 downto 0) := (others => '0');
    signal p2_tr : unsigned(11 downto 0) := (others => '0');
    signal p2_ca, p2_sa : unsigned(15 downto 0) := (others => '0');
    signal p2_ta3, p2_tb2 : signed(10 downto 0) := (others => '0');
    signal p3_h, p3_hn : unsigned(15 downto 0) := (others => '0');
    signal p3_src : signed(7 downto 0) := (others => '0');
    signal p4_hs, p4_hns : unsigned(15 downto 0) := (others => '0');
    signal p4_pe : signed(16 downto 0) := (others => '0');
    signal p5_e : signed(17 downto 0) := (others => '0');
    signal p5_dd, p5_ddn : unsigned(15 downto 0) := (others => '0');
    signal p5_t405, p5_t946 : unsigned(15 downto 0) := (others => '0');
    type t_u16 is array (natural range <>) of unsigned(15 downto 0);
    signal p_teff : t_u16(2 to 5) := (others => (others => '0'));
    type t_sl is array (natural range <>) of std_logic;
    signal p_sync : t_sl(2 to 6) := (others => '0');
    signal p_lead0 : t_sl(2 to 5) := (others => '0');
    signal p_dk0 : t_sl(2 to 5) := (others => '0');
    -- stage 6 decisions
    signal p6_left, p6_blob, p6_slab, p6_over, p6_lead, p6_top3, p6_bot2, p6_dk : std_logic := '0';
    signal p6_dq : unsigned(4 downto 0) := (others => '0');
    signal p6_wb : signed(17 downto 0) := (others => '0');
    signal p6_tx : unsigned(15 downto 0) := (others => '0');
    -- glow factor chain
    signal p7_gb, p7_gt : unsigned(4 downto 0) := (others => '0');
    type t_u3 is array (natural range <>) of unsigned(2 downto 0);
    signal p_g4 : t_u3(8 to 12) := (others => (others => '0'));
    -- 7: ROM address, 8: ROM out, 9: fabric copy
    signal r_ae, r_ao : unsigned(8 downto 0) := (others => '0');
    signal r_ey, r_eu, r_ev, r_oy, r_ou, r_ov : std_logic_vector(7 downto 0) := (others => '0');
    signal p9_e, p9_o : t_c3 := (others => (others => '0'));
    type t_flags is record
        left, blob, slab, bank, over, lead, top3, bot2, dk, band, magz, pale : std_logic;
        dq : unsigned(4 downto 0);
    end record;
    constant C_FL0 : t_flags := ('0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', (others => '0'));
    type t_flp is array (natural range <>) of t_flags;
    signal p_fl : t_flp(7 to 15) := (others => C_FL0);
    signal p_odd : t_sl(7 to 9) := (others => '0');
    type t_u2 is array (natural range <>) of unsigned(1 downto 0);
    signal p_fr : t_u2(7 to 10) := (others => (others => '0'));
    -- colour stages
    type t_s10a is array(0 to 2) of signed(9 downto 0);
    type t_s11a is array(0 to 2) of signed(10 downto 0);
    signal p10_a, p10_k, p11_a, p11_k : t_c3 := (others => (others => '0'));
    signal p10_d : t_s10a := (others => (others => '0'));
    signal p11_p : t_s11a := (others => (others => '0'));
    signal p12_c, p13_c, p13_q, p13_k, p14_c, p15_c : t_c3 := (others => (others => '0'));
    signal p16_y, p16_u, p16_v : unsigned(9 downto 0) := to_unsigned(64, 10);
    -- output conditioning
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    signal s_csu, s_csv : t_u10(0 to 8) := (others => to_unsigned(512, 10));
    signal s_boxu, s_boxv, s_boxud, s_boxvd : unsigned(13 downto 0) := to_unsigned(4096, 14);
    signal s_y17, s_y18, s_y19, s_y20, s_y21, s_y22x, s_yf22 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u22, s_v22 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_y23, s_y24 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u23, s_v23, s_u24, s_v24 : unsigned(9 downto 0) := to_unsigned(512, 10);
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+1) pixel

    attribute ram_style : string;

begin

    ---------------------------------------------------------------------------
    -- Colourizer ROMs (even / odd levels), registered address and output
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ey <= C_PE_Y(to_integer(r_ae)); r_eu <= C_PE_U(to_integer(r_ae)); r_ev <= C_PE_V(to_integer(r_ae));
            r_oy <= C_PO_Y(to_integer(r_ao)); r_ou <= C_PO_U(to_integer(r_ao)); r_ov <= C_PO_V(to_integer(r_ao));
        end if;
    end process p_rom;

    ---------------------------------------------------------------------------
    -- Controls, vblank zone, per-frame sequencer (one serial multiplier)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_id : integer range 0 to 15;
        variable v_x  : signed(10 downto 0);
        variable v_k  : unsigned(12 downto 0);
        variable v_df : signed(13 downto 0);
        variable v_p  : integer range 0 to 31;
        variable v_ph : integer range 0 to 31;
        variable v_s  : unsigned(20 downto 0);
        variable v_pr : unsigned(37 downto 0);
        variable v_u  : unsigned(10 downto 0);
        variable v_ng : signed(13 downto 0);
        variable v_gi : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            v_id := to_integer(unsigned(registers_in(8)(3 downto 0)));
            t_rw <= unsigned(C_T_RW(v_id)); t_ysk <= unsigned(C_T_YSK(v_id)); t_kyl <= unsigned(C_T_KYL(v_id));
            t_cdt <= unsigned(C_T_CDT(v_id)); t_cddt <= unsigned(C_T_CDDT(v_id));
            t_kax <= unsigned(C_T_KAX(v_id)); t_kbx <= unsigned(C_T_KBX(v_id));
            t_kay <= unsigned(C_T_KAY(v_id)); t_kby <= unsigned(C_T_KBY(v_id));
            t_rmp <= unsigned(C_T_RMP(v_id)); t_ys <= unsigned(C_T_YS(v_id));

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
            g_wm1 <= s_w - 1;

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                -- bipolar knobs: K3 Blob, K5 Speed (deadband 8)
                v_x := signed(resize(unsigned(registers_in(4)), 11)) - 512;
                if v_x > 7 then s_s5 <= '0'; v_u := unsigned(v_x) - 8; s_m5 <= v_u(8 downto 0);
                elsif v_x < -7 then s_s5 <= '1'; v_u := unsigned(-v_x) - 8; s_m5 <= v_u(8 downto 0);
                else s_s5 <= '0'; s_m5 <= (others => '0'); end if;
                s_k6 <= unsigned(registers_in(5));
                f_mir <= registers_in(6)(0);
                f_neg <= registers_in(6)(1);
                f_inv <= registers_in(6)(2);
                f_viv <= registers_in(6)(3);
                f_osc <= registers_in(6)(4);
                -- ride phase and Osc drift
                f_ride <= f_ride + unsigned(resize(f_spd, 24));
                f_ta <= f_ta + 262; f_tb <= f_tb - 197;
            end if;
            -- glides: K1, K2, K3, K4, P12 ease 1/8 of the gap per field (deadband 4); one channel per
            -- four zone clocks (4..23): select, difference, ease, write
            if s_zone = '1' and s_zc >= 4 and s_zc < 24 then q_gon <= '1'; else q_gon <= '0'; end if;
            q_gi <= s_zc(4 downto 2) - 1; q_gph <= s_zc(1 downto 0);
            if q_gon = '1' then
                case to_integer(q_gph) is
                    when 0 =>
                        case to_integer(q_gi) is
                            when 0 => gl_k <= unsigned(registers_in(0)) & "000"; gl_o <= s_g(0);
                            when 1 => gl_k <= unsigned(registers_in(1)) & "000"; gl_o <= s_g(1);
                            when 2 => gl_k <= unsigned(registers_in(2)) & "000"; gl_o <= s_g(2);
                            when 3 => gl_k <= unsigned(registers_in(3)) & "000"; gl_o <= s_g(3);
                            when others => gl_k <= unsigned(registers_in(7)) & "000"; gl_o <= s_g(4);
                        end case;
                    when 1 =>
                        gl_d <= signed(resize(gl_k, 14)) - signed(resize(gl_o, 14));
                    when 2 =>
                        v_ng := signed(resize(gl_o, 14)) + shift_right(gl_d, 3);
                        gl_n <= unsigned(v_ng(12 downto 0));
                        if gl_d > 31 or gl_d < -31 then gl_w <= '1'; else gl_w <= '0'; end if;
                    when others =>
                        if gl_w = '1' then
                            case to_integer(q_gi) is
                                when 0 => s_g(0) <= gl_n;
                                when 1 => s_g(1) <= gl_n;
                                when 2 => s_g(2) <= gl_n;
                                when 3 => s_g(3) <= gl_n;
                                when others => s_g(4) <= gl_n;
                            end case;
                        end if;
                end case;
            end if;
            -- K3 (glided) bipolar decode
            v_x := signed(resize(s_g(2)(12 downto 3), 11)) - 512;
            if v_x > 7 then s_s3 <= '0'; v_u := unsigned(v_x) - 8; s_m3 <= v_u(8 downto 0);
            elsif v_x < -7 then s_s3 <= '1'; v_u := unsigned(-v_x) - 8; s_m3 <= v_u(8 downto 0);
            else s_s3 <= '0'; s_m3 <= (others => '0'); end if;
            -- knob curves (settle long before the sequencer reads them)
            f_cols  <= to_unsigned(512, 12) + resize(s_g(0)(12 downto 3), 12) + resize(s_g(0)(12 downto 3) & '0', 12);
            f_bands <= to_unsigned(512, 12) + resize(s_g(1)(12 downto 3) & '0', 12);
            f_r     <= s_g(4)(12 downto 3) & "00000";

            -- serial products: slot p = zc/32 (1..18), phase 0 load, 1..18 step, 20 store; the
            -- strobes and the slot number are registered from zc first (one clock late, same order)
            v_p  := to_integer(s_zc(9 downto 5));
            v_ph := to_integer(s_zc(4 downto 0));
            q_p <= s_zc(9 downto 5);
            if s_zone = '1' and v_p >= 1 and v_p <= 18 and v_ph = 0 then q_ld <= '1'; else q_ld <= '0'; end if;
            if s_zone = '1' and v_p >= 1 and v_p <= 18 and v_ph >= 1 and v_ph <= 18 then q_st <= '1'; else q_st <= '0'; end if;
            for k in 1 to 18 loop
                if s_zone = '1' and v_p = k and v_ph = 20 then q_wr(k) <= '1'; else q_wr(k) <= '0'; end if;
            end loop;
            if true then
                if q_ld = '1' then
                    m_h <= (others => '0'); m_l <= (others => '0');
                    case to_integer(q_p) is
                        when 1  => m_a <= resize(f_cols, 20);  m_b <= resize(t_rw, 18);
                        when 2  => m_a <= resize(f_bands, 20); m_b <= resize(t_ysk, 18);
                        when 3  => m_a <= f_inc;               m_b <= resize(f_r, 18);
                        when 4  => m_a <= resize(f_tt, 20);    m_b <= to_unsigned(1015, 18);
                        when 5  => m_a <= resize(f_r, 20);     m_b <= to_unsigned(29928, 18);
                        when 6  => m_a <= resize(f_r, 20);     m_b <= to_unsigned(13033, 18);
                        when 7  => m_a <= resize(f_r, 20);     m_b <= to_unsigned(3251, 18);
                        when 8  => m_a <= f_inc;               m_b <= to_unsigned(2688, 18);
                        when 9  => m_a <= resize(s_g(3)(12 downto 3), 20); m_b <= resize(s_g(3)(12 downto 3), 18);
                        when 10 => m_a <= f_k4sq;              m_b <= to_unsigned(609, 18);
                        when 11 => m_a <= resize(f_fl, 20);    m_b <= to_unsigned(13914, 18);
                        when 12 => m_a <= resize(f_fl, 20);    m_b <= t_cdt;
                        when 13 => m_a <= resize(f_fl, 20);    m_b <= resize(t_cddt, 18);
                        when 14 => m_a <= resize(f_fl, 20);    m_b <= to_unsigned(5405, 18);
                        when 15 => m_a <= resize(s_m5, 20);    m_b <= resize(s_m5, 18);
                        when 16 => m_a <= resize(f_s5sq, 20);  m_b <= to_unsigned(350, 18);
                        when 17 => m_a <= resize(s_m3, 20);    m_b <= to_unsigned(521, 18);
                        when others => m_a <= resize(s_k6, 20); m_b <= to_unsigned(975, 18);
                    end case;
                end if;
                if q_st = '1' then
                    if m_b(0) = '1' then v_s := resize(m_h, 21) + resize(m_a, 21); else v_s := resize(m_h, 21); end if;
                    m_h <= v_s(20 downto 1);
                    m_l <= v_s(0) & m_l(17 downto 1);
                    m_b <= '0' & m_b(17 downto 1);
                end if;
                v_pr := m_h & m_l;
                if q_wr(1)  = '1' then f_inc  <= v_pr(27 downto 8); end if;
                if q_wr(2)  = '1' then f_kb   <= v_pr(27 downto 8); end if;
                if q_wr(3)  = '1' then f_tt   <= v_pr(33 downto 16); end if;
                if q_wr(4)  = '1' then f_sl   <= v_pr(27 downto 10); end if;
                if q_wr(5)  = '1' then f_st   <= v_pr(30 downto 16); end if;
                if q_wr(6)  = '1' then f_sl0  <= v_pr(28 downto 16); end if;
                if q_wr(7)  = '1' then f_dep  <= v_pr(26 downto 16); end if;
                if q_wr(8)  = '1' then f_tdi  <= v_pr(31 downto 16); end if;
                if q_wr(9)  = '1' then f_k4sq <= v_pr(19 downto 0); end if;
                if q_wr(10) = '1' then f_fl   <= to_unsigned(31, 10) + resize(v_pr(29 downto 20), 10); end if;
                if q_wr(11) = '1' then f_t0   <= v_pr(23 downto 0) & "000000"; end if;
                if q_wr(12) = '1' then f_dt0  <= -signed(resize(v_pr(27 downto 8), 22)); end if;
                if q_wr(13) = '1' then f_ddt  <= v_pr(20 downto 8); end if;
                if q_wr(14) = '1' then f_ts   <= v_pr(21 downto 8); end if;
                if q_wr(15) = '1' then f_s5sq <= v_pr(17 downto 0); end if;
                if q_wr(16) = '1' then
                    if s_s5 = '1' then f_spd <= -signed(resize(v_pr(26 downto 8), 20));
                    else f_spd <= signed(resize(v_pr(26 downto 8), 20)); end if;
                end if;
                if q_wr(17) = '1' then
                    if s_s3 = '1' then f_bo <= -signed(resize(v_pr(17 downto 4), 16));
                    else f_bo <= signed(resize(v_pr(17 downto 4), 16)); end if;
                end if;
                if q_wr(18) = '1' then f_bend <= v_pr(19 downto 6); end if;
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
    -- Per-line engine: computes the next line at the source avid fall (and
    -- line 0 late in the vblank zone); committed at the source avid rise
    ---------------------------------------------------------------------------
    p_lt : process(clk)
    begin
        if rising_edge(clk) then
            r_lq <= C_LT(to_integer(r_la));
            r_lq2 <= r_lq;
        end if;
    end process p_lt;

    p_line : process(clk)
        variable v_d : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            if s_zgo = '1' then
                -- line 0 accumulators
                n_vy <= (others => '0');
                n_vb <= to_unsigned(36203, 16) & x"00";
                n_s  <= (C_B0 & x"00") + f_ride;
                n_e0a <= (others => '0'); n_e0i <= (others => '0');
                n_y8a <= (others => '0'); n_y8q <= to_signed(-8, 9);
                n_pa <= f_ta; n_pb <= f_tb;
                n_ramp <= to_unsigned(36540, 16);
                n_bp1 <= '0'; n_bp2 <= '0';
            end if;
            if s_zone = '1' and s_zc = 700 then
                le_st <= to_unsigned(2, 4); le_adv <= '0';
            elsif data_in.avid = '0' and s_avq = '1' then
                le_st <= to_unsigned(1, 4); le_adv <= '1';
            elsif le_st /= 0 then
                if le_st = 13 then le_st <= (others => '0'); else le_st <= le_st + 1; end if;
            end if;

            case to_integer(le_st) is
                when 1 =>
                    -- advance one line
                    n_vy <= n_vy + t_ys;
                    n_vb <= n_vb + f_kb;
                    n_s  <= n_s - t_kyl;
                    n_e0a <= n_e0a + t_ys;
                    n_y8a <= n_y8a + t_ys;
                    n_pa <= n_pa + t_kay; n_pb <= n_pb - t_kby;
                    if n_vy >= 530 * 256 then
                        if n_ramp + t_rmp >= 52130 then n_ramp <= to_unsigned(52130, 16);
                        else n_ramp <= n_ramp + t_rmp; end if;
                    end if;
                when 2 =>
                    if n_e0a >= 4320 then n_e0a <= n_e0a - 4320; n_e0i <= n_e0i + 1; end if;
                    if n_y8a >= 2160 then n_y8a <= n_y8a - 2160; n_y8q <= n_y8q + 1; end if;
                    if n_vb(23 downto 8) < C_BH then
                        n_band <= '1'; n_dtop <= n_vb(23 downto 8); n_seg <= C_BH;
                    else
                        n_band <= '0'; n_dtop <= n_vb(23 downto 8) - C_BH; n_seg <= 65535 - C_BH + 1;
                    end if;
                    if n_vy >= 530 * 256 then n_low <= '1'; else n_low <= '0'; end if;
                when 3 =>
                    r_la <= '0' & n_e0i;                                   -- drift
                    v_d := n_seg - n_dtop;
                    n_dbot <= v_d;
                    if n_dtop(15 downto 12) /= 0 then n_cta <= (others => '1'); else n_cta <= n_dtop(11 downto 7); end if;
                    if v_d(15 downto 13) /= 0 then n_cba <= (others => '1'); else n_cba <= v_d(12 downto 8); end if;
                    if n_y8q < 0 then n_l0 <= (others => '0'); n_fr <= (others => '0');
                    elsif n_y8q >= 112 then n_l0 <= to_unsigned(7, 3); n_fr <= (others => '0');
                    else n_l0 <= unsigned(n_y8q(6 downto 4)); n_fr <= unsigned(n_y8q(3 downto 2)); end if;
                when 4 =>
                    r_la <= "10" & n_cta;                                  -- top corner
                when 5 =>
                    r_la <= "11" & n_cba;                                  -- bottom corner
                when 6 =>
                    n_e0 <= unsigned(r_lq2);
                when 7 =>
                    n_ct <= unsigned(r_lq2(12 downto 0));
                when 8 =>
                    n_cb <= unsigned(r_lq2(12 downto 0));
                when 9 =>
                    n_ea <= signed(resize(n_e0, 18)) + signed(resize(n_ct, 18));
                    if n_band = '1' then
                        n_eb2 <= signed(resize(n_cb, 18)) + resize(f_bo, 18) + signed(resize(C_BOFF, 18));
                    else
                        n_eb2 <= signed(resize(n_cb, 18)) + resize(f_bo, 18);
                    end if;
                when 10 =>
                    n_e <= n_ea + n_eb2;
                    if n_vb(23 downto 8) >= C_BH(15 downto 1) then v_d := n_vb(23 downto 8) - C_BH(15 downto 1);
                    else v_d := C_BH(15 downto 1) - n_vb(23 downto 8); end if;
                    n_es <= to_signed(40925, 18) + signed(resize(v_d, 18));
                when 11 =>
                    if n_band = '0' then
                        if n_e > signed(resize(n_ramp, 18)) then n_es <= n_e; else n_es <= signed(resize(n_ramp, 18)); end if;
                    end if;
                    if (n_band /= n_bp1) or (n_bp1 /= n_bp2 and t_ys(10 downto 9) = "00") then n_pale <= '1'; else n_pale <= '0'; end if;
                when 12 =>
                    n_bp2 <= n_bp1; n_bp1 <= n_band;
                when others => null;
            end case;

            -- commit at the source avid rise (before the beam starts)
            if data_in.avid = '1' and s_avq = '0' then
                l_e <= n_e; l_es <= n_es; l_s <= n_s(23 downto 8);
                l_band <= n_band; l_low <= n_low; l_pale <= n_pale;
                l_l0 <= n_l0; l_fr <= n_fr; l_pa <= n_pa; l_pb <= n_pb;
            end if;
        end if;
    end process p_line;

    ---------------------------------------------------------------------------
    -- Beam: tile phase, columns, slab staircase DDAs, thickness DDA, Osc phases
    ---------------------------------------------------------------------------
    p_beam : process(clk)
        variable v_tp : unsigned(24 downto 0);
        variable v_tpf : unsigned(15 downto 0);
        variable v_in : std_logic;
        variable v_mn : std_logic;
    begin
        if rising_edge(clk) then
            if g_start = '1' then
                b_tp <= C_TP0; b_tq <= resize(C_TP0, 25) + resize(f_inc, 25);
                b_cc <= (others => '0'); b_m <= '0'; b_sync <= '1';
                b_ca <= unsigned(-signed(resize(f_r, 16)));
                b_sa <= resize(f_sl0 & x"00", 24);
                b_td <= to_unsigned(56219, 16);
                b_t  <= resize(f_t0, 31); b_dt <= resize(f_dt0, 27);
                b_tl <= (others => '0');
                b_pa <= l_pa; b_pb <= l_pb;
            elsif g_avid = '1' then
                v_tp := resize(b_tq(23 downto 0), 25) + resize(f_inc, 25);
                b_tp <= b_tq(23 downto 0);
                b_tq <= v_tp;
                v_tpf := b_tp(23 downto 8);
                if b_m = '1' then
                    if v_tpf > C_NUE then v_in := '1'; else v_in := '0'; end if;
                else
                    if v_tpf < C_UE then v_in := '1'; else v_in := '0'; end if;
                end if;
                b_t <= unsigned(signed(b_t) + resize(b_dt, 31));
                b_dt <= b_dt + signed(resize(f_ddt, 27));
                b_pa <= b_pa + t_kax; b_pb <= b_pb + t_kbx;
                if b_tq(24) = '1' then
                    -- tile wrap: next column
                    b_cc <= b_cc + 1; b_sync <= '0';
                    v_mn := f_mir and not b_cc(0);       -- the new column index is odd
                    b_m <= v_mn;
                    b_ca <= b_ca + resize(f_r, 16);
                    if v_mn = '1' then
                        b_sa <= resize(f_st & x"00", 24); b_td <= to_unsigned(38666, 16);
                    else
                        b_sa <= (others => '0'); b_td <= to_unsigned(49152, 16);
                    end if;
                    if b_t(30) = '1' then b_tl <= (others => '1'); else b_tl <= b_t(29 downto 14); end if;
                elsif v_in = '1' then
                    if b_m = '1' then
                        if b_sa > resize(f_sl, 24) then b_sa <= b_sa - resize(f_sl, 24); else b_sa <= (others => '0'); end if;
                        b_td <= b_td - f_tdi;
                    else
                        b_sa <= b_sa + resize(f_sl, 24);
                        b_td <= b_td + f_tdi;
                    end if;
                end if;
            end if;
        end if;
    end process p_beam;

    ---------------------------------------------------------------------------
    -- Pixel path
    ---------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_e   : signed(18 downto 0);
        variable v_d   : signed(18 downto 0);
        variable v_t   : signed(17 downto 0);
        variable v_x   : signed(10 downto 0);
        variable v_s8  : signed(7 downto 0);
        variable v_s12 : signed(11 downto 0);
        variable v_w   : signed(17 downto 0);
        variable v_gs  : unsigned(9 downto 0);
        variable v_g   : unsigned(4 downto 0);
        variable v_sy  : unsigned(11 downto 0);
        variable v_sc  : unsigned(14 downto 0);
        variable v_dl  : unsigned(9 downto 0);
        variable v_c   : t_c3;
        variable v_f   : t_flags;
        variable v_pe, v_po : unsigned(1 downto 0);
        variable v_bb  : std_logic;
        variable v_in  : std_logic;
        variable v_d10 : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- per-frame derived constants
            f_dep8 <= f_dep(10 downto 3);
            f_rq <= resize(f_r, 16) - resize(f_dep(10 downto 2) & '0', 16) - resize(f_dep(10 downto 2), 16);
            f_tsst <= signed(resize(f_ts, 18)) - signed(resize(f_st, 18));
            f_bend8 <= f_bend(13 downto 6);

            -- video tap: source luma -> signed bend value (a few px early: invisible in a bend field)
            v_y1 <= unsigned(data_in.y);
            v_x := signed(resize(v_y1, 11)) - 502;
            v_x := shift_right(v_x, 2);
            if v_x > 127 then v_s8 := to_signed(127, 8);
            elsif v_x < -128 then v_s8 := to_signed(-128, 8);
            else v_s8 := v_x(7 downto 0); end if;
            s_vd(2) <= v_s8;
            s_vd(3) <= s_vd(2);

            -- 1: tile phase (mirrored), slab parts, Osc triangles
            if b_m = '1' then p1_uf <= not b_tp(23 downto 8); else p1_uf <= b_tp(23 downto 8); end if;
            p1_tpf <= b_tp(23 downto 8);
            p1_sync <= b_sync;
            p1_ca <= b_ca; p1_sa <= b_sa(23 downto 8); p1_td <= b_td(15 downto 12);
            if b_td(15 downto 13) = "111" then p1_tdk <= '1'; else p1_tdk <= '0'; end if;
            p1_tl <= b_tl;
            p1_ta <= f_tri8(b_pa); p1_tb <= f_tri8(b_pb);

            -- 2: tread depth product, Osc partial sums, this pixel's thickness
            p2_uf <= p1_uf;
            p2_tr <= f_dep8 * p1_td;
            p2_ca <= p1_ca; p2_sa <= p1_sa;
            p2_ta3 <= resize(p1_ta, 11) + shift_left(resize(p1_ta, 11), 1);
            p2_tb2 <= shift_left(resize(p1_tb, 11), 1);
            if p1_sync = '1' then
                v_t := f_tsst + signed(resize(p1_sa, 18));
                if v_t < 0 then p_teff(2) <= (others => '0'); else p_teff(2) <= unsigned(v_t(15 downto 0)); end if;
            else
                p_teff(2) <= p1_tl;
            end if;
            p_sync(2) <= p1_sync;
            if p1_tpf >= C_LEADT and f_mir = '0' and f_inv = '0' then p_lead0(2) <= '1'; else p_lead0(2) <= '0'; end if;
            if p1_tdk = '1' and p1_uf < C_UE then p_dk0(2) <= '1'; else p_dk0(2) <= '0'; end if;

            -- 3: staircase h (and the next column's), source select
            p3_uf <= p2_uf;
            p3_h  <= p2_ca + p2_sa - resize(p2_tr(11 downto 1), 16);
            p3_hn <= p2_ca + f_rq;
            if f_osc = '1' then v_x := p2_ta3 + p2_tb2; p3_src <= v_x(10 downto 3); else p3_src <= s_vd(3); end if;

            -- 4: S +- h, bend product
            p4_uf <= p3_uf;
            if f_neg = '1' then p4_hs <= l_s - p3_h; p4_hns <= l_s - p3_hn;
            else p4_hs <= l_s + p3_h; p4_hns <= l_s + p3_hn; end if;
            p4_pe <= p3_src * signed(resize(f_bend8, 9));

            -- 5: blob edge, slab distance, rim thresholds
            p5_uf <= p4_uf;
            if p_sync(4) = '1' and l_low = '1' then v_e := resize(l_es, 19); else v_e := resize(l_e, 19); end if;
            p5_e <= resize(v_e + resize(shift_right(p4_pe, 1), 19), 18);
            p5_dd  <= p4_hs + unsigned(resize(shift_right(p4_pe, 4), 16));
            p5_ddn <= p4_hns + unsigned(resize(shift_right(p4_pe, 4), 16));
            if p_teff(4) > 404 then p5_t405 <= p_teff(4) - 404; else p5_t405 <= (others => '0'); end if;
            if p_teff(4) > 945 then p5_t946 <= p_teff(4) - 945; else p5_t946 <= (others => '0'); end if;

            -- 6: decisions
            if p5_uf < C_LK then p6_left <= '1'; else p6_left <= '0'; end if;
            if p5_uf >= C_OVER then p6_over <= '1'; else p6_over <= '0'; end if;
            v_d := signed(resize(p5_uf, 19)) - resize(p5_e, 19);
            if v_d >= 0 and p5_uf >= C_LK then p6_blob <= '1'; else p6_blob <= '0'; end if;
            if v_d < 0 then p6_dq <= (others => '0');
            elsif v_d(18 downto 15) /= 0 then p6_dq <= (others => '1');
            else p6_dq <= unsigned(v_d(14 downto 10)); end if;
            if p5_dd < p_teff(5) then v_in := '1'; else v_in := '0'; end if;
            p6_slab <= v_in;
            if p_lead0(5) = '1' and p5_ddn < p_teff(5) and v_in = '0' then p6_lead <= '1'; else p6_lead <= '0'; end if;
            if v_in = '1' and p5_dd >= p5_t405 then p6_top3 <= '1'; else p6_top3 <= '0'; end if;
            if p5_dd < 270 then p6_bot2 <= '1'; else p6_bot2 <= '0'; end if;
            if v_in = '1' and p5_dd >= p5_t946 and p5_dd < p5_t405 and p_dk0(5) = '1' then p6_dk <= '1'; else p6_dk <= '0'; end if;
            p6_wb <= to_signed(4594, 18) - signed(resize(p5_dd, 18));
            p6_tx <= p_teff(5) - p5_dd;
            p_sync(6) <= p_sync(5);

            -- 7: flags, ROM addresses; glow factors (bottom: (4594 - dd) / 205, top: (1622 - (T - dd)) / 64)
            v_f.left := p6_left; v_f.blob := p6_blob; v_f.slab := p6_slab; v_f.over := p6_over;
            v_f.bank := p6_slab xor f_inv;
            v_f.lead := p6_lead; v_f.top3 := p6_top3; v_f.bot2 := p6_bot2; v_f.dk := p6_dk;
            v_f.band := l_band; v_f.pale := l_pale;
            v_f.magz := p_sync(6) and l_low and p6_blob and not (p6_slab xor f_inv);
            v_f.dq := p6_dq;
            p_fl(7) <= v_f;
            if v_f.bank = '1' then v_bb := l_band; else v_bb := l_band or f_viv; end if;
            v_po := l_l0(2 downto 1);
            if l_l0(0) = '1' then
                if l_l0(2 downto 1) = "11" then v_pe := "11"; else v_pe := l_l0(2 downto 1) + 1; end if;
            else
                v_pe := l_l0(2 downto 1);
            end if;
            r_ae <= v_f.bank & v_bb & v_pe & p6_dq;
            r_ao <= v_f.bank & v_bb & v_po & p6_dq;
            p_odd(7) <= l_l0(0);
            p_fr(7) <= l_fr;
            if p6_wb <= 0 then p7_gb <= (others => '0');
            else
                v_gs := resize(unsigned(p6_wb(12 downto 8)), 10) + resize(unsigned(p6_wb(12 downto 10)), 10);
                if v_gs >= 16 then p7_gb <= to_unsigned(16, 5); else p7_gb <= v_gs(4 downto 0); end if;
            end if;
            v_w := to_signed(1622, 18) - signed(resize(p6_tx, 18));
            if p6_slab = '0' or v_w <= 0 then p7_gt <= (others => '0');
            elsif v_w(10 downto 6) >= 16 then p7_gt <= to_unsigned(16, 5);
            else p7_gt <= unsigned(v_w(10 downto 6)); end if;

            -- 8: ROM output (p_rom); glow factor in quarters
            if p_fl(7).over = '0' then v_g := (others => '0');
            elsif p7_gb > p7_gt then v_g := p7_gb; else v_g := p7_gt; end if;
            v_gs := resize(v_g, 10) + 2;
            p_g4(8) <= v_gs(4 downto 2);

            -- 9: fabric copy of the ROM outputs
            p9_e <= (unsigned(r_ey), unsigned(r_eu), unsigned(r_ev));
            p9_o <= (unsigned(r_oy), unsigned(r_ou), unsigned(r_ov));

            -- 10: A / B by level parity, B - A; region constant
            if p_odd(9) = '1' then
                p10_a <= p9_o;
                for k in 0 to 2 loop p10_d(k) <= signed(resize(p9_e(k), 10)) - signed(resize(p9_o(k), 10)); end loop;
            else
                p10_a <= p9_e;
                for k in 0 to 2 loop p10_d(k) <= signed(resize(p9_o(k), 10)) - signed(resize(p9_e(k), 10)); end loop;
            end if;
            v_f := p_fl(9);
            v_c := C_MAG;
            if v_f.left = '1' then
                if v_f.bank = '1' then
                    if v_f.band = '1' then v_c := C_GLB; else v_c := C_GL; end if;
                else
                    if v_f.band = '1' then v_c := C_PNK; else v_c := C_LAV; end if;
                end if;
            elsif v_f.blob = '0' then
                if v_f.bank = '1' then
                    if v_f.band = '1' then v_c := C_GGB; else v_c := C_GG; end if;
                else
                    if v_f.band = '1' then v_c := C_GRYB; else v_c := C_GRY; end if;
                end if;
            elsif v_f.band = '1' then
                case to_integer(v_f.dq(4 downto 2)) is
                    when 0 => v_c := C_SALM;
                    when 1 => v_c := c3(197, 175, 134);
                    when 2 => v_c := c3(194, 178, 142);
                    when 3 => v_c := c3(190, 181, 150);
                    when others => v_c := C_MAG;
                end case;
            end if;
            p10_k <= v_c;

            -- 11: (B - A) * fr, fr in quarters
            p11_a <= p10_a; p11_k <= p10_k;
            for k in 0 to 2 loop
                case to_integer(p_fr(10)) is
                    when 0 => p11_p(k) <= (others => '0');
                    when 1 => p11_p(k) <= resize(p10_d(k), 11);
                    when 2 => p11_p(k) <= shift_left(resize(p10_d(k), 11), 1);
                    when others => p11_p(k) <= resize(p10_d(k), 11) + shift_left(resize(p10_d(k), 11), 1);
                end case;
            end loop;

            -- 12: palette colour (interpolated), or the region constant
            v_f := p_fl(11);
            for k in 0 to 2 loop
                v_s12 := signed(resize(p11_a(k), 12)) + resize(shift_right(p11_p(k), 2), 12);
                if v_s12 < 0 then v_c(k) := (others => '0');
                elsif v_s12 > 255 then v_c(k) := (others => '1');
                else v_c(k) := unsigned(v_s12(7 downto 0)); end if;
            end loop;
            if v_f.left = '1' or v_f.blob = '0' or v_f.magz = '1' then p12_c <= p11_k; else p12_c <= v_c; end if;

            -- 13: glow mix in quarters: c * (4 - g) / 4 and GLOW * g / 4
            p13_c <= p12_c;
            for k in 0 to 2 loop
                case to_integer(p_g4(12)) is
                    when 0 => p13_q(k) <= p12_c(k);                          p13_k(k) <= (others => '0');
                    when 1 => p13_q(k) <= p12_c(k) - shift_right(p12_c(k), 2); p13_k(k) <= shift_right(C_GLOW(k), 2);
                    when 2 => p13_q(k) <= shift_right(p12_c(k), 1);          p13_k(k) <= shift_right(C_GLOW(k), 1);
                    when 3 => p13_q(k) <= shift_right(p12_c(k), 2);          p13_k(k) <= C_GLOW(k) - shift_right(C_GLOW(k), 2);
                    when others => p13_q(k) <= (others => '0');               p13_k(k) <= C_GLOW(k);
                end case;
            end loop;

            -- 14: glow sum (inside the slab only)
            v_f := p_fl(13);
            for k in 0 to 2 loop
                if v_f.slab = '1' and f_inv = '0' then p14_c(k) <= p13_q(k) + p13_k(k); else p14_c(k) <= p13_c(k); end if;
            end loop;

            -- 15: rims and treads
            v_f := p_fl(14);
            v_c := p14_c;
            if v_f.slab = '1' and f_inv = '0' then
                if v_f.over = '1' and v_f.top3 = '1' then v_c := C_CYAN;
                elsif v_f.over = '1' and v_f.bot2 = '1' then
                    for k in 0 to 2 loop
                        v_c(k) := resize((resize(p14_c(k), 11) + resize(p14_c(k) & '0', 11)
                                  + resize(C_CYAN(k), 11) + resize(C_CYAN(k) & "00", 11)) srl 3, 8);
                    end loop;
                elsif v_f.over = '0' and v_f.top3 = '1' then v_c := C_LGRN;
                elsif v_f.dk = '1' then v_c := C_DGRN;
                end if;
            end if;
            p15_c <= v_c;

            -- 16: cyan lead strip / pale band-edge lines; to 10 bits
            v_f := p_fl(15);
            v_c := p15_c;
            if v_f.lead = '1' then
                for k in 0 to 2 loop
                    v_c(k) := resize((resize(p15_c(k), 10) + resize(C_CYAN(k), 10) + resize(C_CYAN(k) & '0', 10)) srl 2, 8);
                end loop;
            elsif v_f.pale = '1' and v_f.slab = '0' then
                for k in 0 to 2 loop v_c(k) := resize((resize(p15_c(k), 9) + resize(C_PALE(k), 9)) srl 1, 8); end loop;
            end if;
            p16_y <= v_c(0) & "00"; p16_u <= v_c(1) & "00"; p16_v <= v_c(2) & "00";

            -- 17..22: output conditioning (chroma [1 2 2 2 2 2 2 2 1]/16 as two
            -- adjacent 8-sums, luma [1 2 1]; centre at stage 21 -> 22)
            s_csu(0) <= p16_u; s_csv(0) <= p16_v;
            for i in 1 to 8 loop s_csu(i) <= s_csu(i - 1); s_csv(i) <= s_csv(i - 1); end loop;
            s_boxu <= s_boxu + p16_u - s_csu(7);
            s_boxv <= s_boxv + p16_v - s_csv(7);
            s_boxud <= s_boxu; s_boxvd <= s_boxv;
            v_sc := resize(s_boxu, 15) + s_boxud; s_u22 <= v_sc(13 downto 4);
            v_sc := resize(s_boxv, 15) + s_boxvd; s_v22 <= v_sc(13 downto 4);
            s_y17 <= p16_y; s_y18 <= s_y17; s_y19 <= s_y18; s_y20 <= s_y19;
            s_y21 <= s_y20; s_y22x <= s_y21;
            v_sy := resize(s_y20, 12) + shift_left(resize(s_y21, 12), 1) + resize(s_y22x, 12);
            s_yf22 <= v_sy(11 downto 2);
            -- 23: luma toward black (white 940 -> 777); blanking gate
            if s_yf22 > 64 then v_dl := s_yf22 - 64; else v_dl := (others => '0'); end if;
            if s_gav_sr(21) = '1' then
                s_y23 <= s_yf22 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
                s_u23 <= s_u22; s_v23 <= s_v22;
            else
                s_y23 <= to_unsigned(64, 10); s_u23 <= to_unsigned(512, 10); s_v23 <= to_unsigned(512, 10);
            end if;
            -- 24: pad to the 24-clock latency
            s_y24 <= s_y23; s_u24 <= s_u23; s_v24 <= s_v23;

            -- delays
            for i in 3 to 5 loop p_teff(i) <= p_teff(i - 1); p_lead0(i) <= p_lead0(i - 1); p_dk0(i) <= p_dk0(i - 1); end loop;
            for i in 3 to 5 loop p_sync(i) <= p_sync(i - 1); end loop;
            for i in 8 to 15 loop p_fl(i) <= p_fl(i - 1); end loop;
            for i in 8 to 9 loop p_odd(i) <= p_odd(i - 1); end loop;
            for i in 8 to 10 loop p_fr(i) <= p_fr(i - 1); end loop;
            for i in 9 to 12 loop p_g4(i) <= p_g4(i - 1); end loop;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y24);
    data_out.u       <= std_logic_vector(s_u24);
    data_out.v       <= std_logic_vector(s_v24);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture escalator;
