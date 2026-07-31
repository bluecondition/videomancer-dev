-- turpentine.vhd
--
-- TURPENTINE v3 -- chroma as pigment: the picture rebuilt from paint dabs.
--
-- v1 (chroma-directed smear) and v2 (posterize + relief) both read as
-- filters -- the image stayed the image. v3 goes structural: the video is
-- RECONSTRUCTED from discrete dabs of paint on a canvas. A cell grid
-- samples the picture; each cell gets one glossy blob of pigment whose
-- colour is the cell's hue quantized to a limited palette and whose SIZE
-- is the cell's saturation (paint load). Gray cells get NO paint -- the
-- canvas shows through. Every control changes the structure of the
-- painting, not a gain:
--
--   K1 Pigments  2..32 hue sectors -- the palette collapses
--   K2 Dab       8/16/32/64 px cells -- pointillism -> chunky impasto
--   K3 Light     highlight position on every dab (S11 Orbit = auto)
--   K4 Scatter   grid -> hand-thrown chaos (position + size jitter)
--   K5 Load      how much saturation earns paint: everything painted ->
--                only vivid colours survive as sparse dots on bare canvas
--   K6 Smear     dabs stretch into long horizontal knife strokes
--   P12 Solvent  dry video -> dabs over the photo -> the photo strips
--                away to bare canvas -> glossy varnish climax
--   S7 Palette   Free (sampled saturation) / Vivid (full-load pigments)
--   S8 Canvas    Paper (warm white) / Velvet (near-black)
--   S9 Shape     Round dabs / Square tiles (mosaic)
--   S10 Rim      dark contour ring around every dab
--   S11 Orbit    light auto-rotation
--
-- Architecture: streaming, C_LATENCY = 9 (render E1..E6 + a 3-clock
-- inline dry/wet lerp). The v2 hue-sector quantizer (octant fold + parallel
-- tangent compares, no atan2) runs on every pixel but is USED once per
-- cell: on the first line of each cell row the cell-centre pixel's
-- {luma8, idx5, ring2, gray} is written to a 256x16 cell-sample BRAM
-- (1 EBR -- the whole program uses 1). Rendering never touches it
-- directly: each cell's paint is fully precomputed during the previous
-- cell's 8 pixel phases (BRAM land -> palette angle -> registered qsin ->
-- 2 multiplies -> colour/threshold registers), redshift prefetch style,
-- with an hblank sequence priming cell 0. Per PIXEL the dab test is just
-- |x-cx| -> squared-distance LUT -> compare against per-cell-per-line
-- thresholds (body / rim / highlight) -- shallow, fast logic.
--
-- Multiplies: 2 palette (u8 x s8, once per cell) + 3 mix (u8 x s11).
-- EBR: 1. Everything else is shift-add.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture turpentine of program_top is

    constant C_LATENCY : integer := 9;

    ----------------------------------------------------------------------
    -- helpers
    ----------------------------------------------------------------------
    function f_cu8(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 8);
        elsif v > 255 then
            return to_unsigned(255, 8);
        else
            return resize(unsigned(v), 8);
        end if;
    end function;

    function f_cu10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return resize(unsigned(v), 10);
        end if;
    end function;

    -- |s10| -> u9 (clamps -512 to 511)
    function f_abs9(v : signed(9 downto 0)) return unsigned is
        variable v11 : signed(10 downto 0);
    begin
        v11 := resize(v, 11);
        if v11 < 0 then
            v11 := -v11;
        end if;
        if v11 > 511 then
            v11 := to_signed(511, 11);
        end if;
        return resize(unsigned(v11(8 downto 0)), 9);
    end function;

    -- |s8| -> u6 clamped to 0..31 (squared-distance LUT domain)
    function f_abs31(v : signed(7 downto 0)) return unsigned is
        variable v9 : signed(8 downto 0);
    begin
        v9 := resize(v, 9);
        if v9 < 0 then
            v9 := -v9;
        end if;
        if v9 > 31 then
            v9 := to_signed(31, 9);
        end if;
        return resize(unsigned(v9(4 downto 0)), 5);
    end function;

    -- squared-distance LUT, 32 entries (logic ROM)
    type t_sq is array (0 to 31) of unsigned(9 downto 0);
    constant C_SQ : t_sq := (
        to_unsigned(  0, 10), to_unsigned(  1, 10), to_unsigned(  4, 10), to_unsigned(  9, 10),
        to_unsigned( 16, 10), to_unsigned( 25, 10), to_unsigned( 36, 10), to_unsigned( 49, 10),
        to_unsigned( 64, 10), to_unsigned( 81, 10), to_unsigned(100, 10), to_unsigned(121, 10),
        to_unsigned(144, 10), to_unsigned(169, 10), to_unsigned(196, 10), to_unsigned(225, 10),
        to_unsigned(256, 10), to_unsigned(289, 10), to_unsigned(324, 10), to_unsigned(361, 10),
        to_unsigned(400, 10), to_unsigned(441, 10), to_unsigned(484, 10), to_unsigned(529, 10),
        to_unsigned(576, 10), to_unsigned(625, 10), to_unsigned(676, 10), to_unsigned(729, 10),
        to_unsigned(784, 10), to_unsigned(841, 10), to_unsigned(900, 10), to_unsigned(961, 10));

    function f_sq(v : unsigned(4 downto 0)) return unsigned is
    begin
        return C_SQ(to_integer(v));
    end function;

    -- quarter-wave folded sine, 64-entry logic ROM (fireworks/mercurial)
    type t_qsin is array(0 to 63) of signed(9 downto 0);
    constant C_QSIN : t_qsin := (
        to_signed(  0, 10), to_signed( 13, 10), to_signed( 25, 10), to_signed( 38, 10), to_signed( 50, 10), to_signed( 63, 10), to_signed( 75, 10), to_signed( 87, 10),
        to_signed(100, 10), to_signed(112, 10), to_signed(124, 10), to_signed(136, 10), to_signed(148, 10), to_signed(160, 10), to_signed(172, 10), to_signed(184, 10),
        to_signed(196, 10), to_signed(207, 10), to_signed(218, 10), to_signed(230, 10), to_signed(241, 10), to_signed(252, 10), to_signed(263, 10), to_signed(273, 10),
        to_signed(284, 10), to_signed(294, 10), to_signed(304, 10), to_signed(314, 10), to_signed(324, 10), to_signed(334, 10), to_signed(343, 10), to_signed(352, 10),
        to_signed(361, 10), to_signed(370, 10), to_signed(379, 10), to_signed(387, 10), to_signed(395, 10), to_signed(403, 10), to_signed(410, 10), to_signed(418, 10),
        to_signed(425, 10), to_signed(432, 10), to_signed(438, 10), to_signed(445, 10), to_signed(451, 10), to_signed(456, 10), to_signed(462, 10), to_signed(467, 10),
        to_signed(472, 10), to_signed(477, 10), to_signed(481, 10), to_signed(485, 10), to_signed(489, 10), to_signed(492, 10), to_signed(496, 10), to_signed(499, 10),
        to_signed(501, 10), to_signed(503, 10), to_signed(505, 10), to_signed(507, 10), to_signed(509, 10), to_signed(510, 10), to_signed(510, 10), to_signed(511, 10));

    function f_qsin(a : unsigned(9 downto 0)) return signed is
        variable v_i : unsigned(5 downto 0);
        variable v_t : signed(9 downto 0);
    begin
        if a(8) = '0' then
            v_i := a(7 downto 2);
        else
            v_i := not a(7 downto 2);
        end if;
        v_t := C_QSIN(to_integer(v_i));
        if a(9) = '1' then
            return -v_t;
        else
            return v_t;
        end if;
    end function;

    -- value-noise hash, split across two stages (inferno lesson)
    function f_hash_a(cx, cy : unsigned(7 downto 0);
                      seed   : unsigned(15 downto 0)) return unsigned is
        variable h16 : unsigned(15 downto 0);
        variable h   : unsigned(11 downto 0);
    begin
        h16 := ((cx & cy)) xor seed;
        h16 := h16 xor (x"0" & h16(15 downto 4));
        h   := h16(11 downto 0);
        return h + rotate_left(h, 5);
    end function;

    function f_hash_b(hin : unsigned(11 downto 0)) return unsigned is
        variable h : unsigned(11 downto 0);
    begin
        h := hin xor rotate_left(hin, 9);
        h := h + rotate_left(h, 3);
        h := h xor ("00000" & h(11 downto 5));
        return h(9 downto 2);
    end function;

    ----------------------------------------------------------------------
    -- frame-latched controls + per-frame terms (vblank sequencer)
    ----------------------------------------------------------------------
    signal s_p12 : unsigned(9 downto 0) := (others => '0');
    signal s_k1  : unsigned(9 downto 0) := (others => '0');
    signal s_k2  : unsigned(9 downto 0) := (others => '0');
    signal s_k4  : unsigned(9 downto 0) := (others => '0');
    signal s_k5  : unsigned(9 downto 0) := (others => '0');
    signal s_k6  : unsigned(9 downto 0) := (others => '0');
    signal s_vivid  : std_logic := '0';                        -- S7
    signal s_velvet : std_logic := '0';                        -- S8
    signal s_square : std_logic := '0';                        -- S9
    signal s_rim_on : std_logic := '1';                        -- S10
    signal s_orbit  : std_logic := '0';                        -- S11

    signal s_lang      : unsigned(9 downto 0) := (others => '0');
    signal s_orbit_acc : unsigned(9 downto 0) := (others => '0');
    signal s_lx8, s_ly8 : signed(7 downto 0) := to_signed(90, 8);
    signal s_ox, s_oy   : signed(5 downto 0) := (others => '0');

    signal s_mix10  : unsigned(9 downto 0) := (others => '0');
    signal s_bgm    : unsigned(1 downto 0) := (others => '0'); -- bg morph
    signal s_varn   : std_logic := '0';
    signal s_mask5  : unsigned(4 downto 0) := (others => '1');
    signal s_half10 : unsigned(9 downto 0) := to_unsigned(16, 10);

    signal s_csh    : natural range 3 to 6 := 4;               -- cell size
    signal s_cmask  : unsigned(5 downto 0) := to_unsigned(15, 6);
    signal s_chalf  : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal s_r3, s_r2, s_r1 : unsigned(4 downto 0) := (others => '0');
    signal s_gate2  : unsigned(1 downto 0) := "01";            -- load gate
    signal s_sparse : std_logic := '0';
    signal s_smr    : natural range 0 to 3 := 0;               -- smear
    signal s_sct    : unsigned(1 downto 0) := "00";            -- scatter
    -- Scatter throw range, derived from the cell size so a dab can NEVER
    -- cross its own cell edge (a dab is only drawn within its own cell's
    -- pixels, so an overflowing dab renders as a clipped half-moon). The
    -- throw is chalf>>z and the radius is scaled to match, so Scatter
    -- reads as "hand-thrown = smaller, wandering dabs" instead of
    -- pac-men. jsl/jsr: one is always 0 (signed shift by csh-4-z).
    signal s_jz  : std_logic := '1';                           -- throw off
    signal s_jsl : natural range 0 to 2 := 0;
    signal s_jsr : natural range 0 to 4 := 0;
    signal s_glb    : unsigned(6 downto 0) := to_unsigned(96, 7); -- gloss

    -- per-frame pigment-load LUT (vivid folded in)
    signal s_sq1, s_sq2, s_sq3 : unsigned(7 downto 0) := (others => '0');

    -- canvas + rim colours (S8-dependent)
    signal s_cv_y  : unsigned(9 downto 0) := to_unsigned(816, 10);
    signal s_cv_u  : unsigned(9 downto 0) := to_unsigned(502, 10);
    signal s_cv_v  : unsigned(9 downto 0) := to_unsigned(520, 10);
    signal s_rim_y : unsigned(9 downto 0) := to_unsigned(96, 10);

    signal s_seq   : unsigned(3 downto 0) := (others => '0');
    signal s_qs_a  : unsigned(9 downto 0) := (others => '0');
    signal s_qs_ar : unsigned(9 downto 0) := (others => '0');
    signal s_qs_r  : signed(9 downto 0) := (others => '0');

    signal s_prev_vsync_n : std_logic := '1';
    signal s_prev_hsync_n : std_logic := '1';

    -- saturation ring thresholds (sat9 scale)
    constant C_GRAY : unsigned(9 downto 0) := to_unsigned(56, 10);
    constant C_RLO  : unsigned(9 downto 0) := to_unsigned(160, 10);
    constant C_RHI  : unsigned(9 downto 0) := to_unsigned(320, 10);

    ----------------------------------------------------------------------
    -- position / line bookkeeping
    ----------------------------------------------------------------------
    signal s_x_count : unsigned(10 downto 0) := (others => '0');
    signal s_aline   : unsigned(9 downto 0) := (others => '0');
    signal s_seen    : std_logic := '0';
    signal s_cyi     : unsigned(7 downto 0) := (others => '0');
    signal s_dyb     : signed(6 downto 0) := (others => '0');  -- y - centre
    signal s_capline : std_logic := '0';                       -- cell row ln0

    ----------------------------------------------------------------------
    -- hue-sector quantizer lanes E1..E5 (v2 machinery, feeds capture)
    ----------------------------------------------------------------------
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal px1 : unsigned(10 downto 0) := (others => '0');

    signal au2, av2 : unsigned(8 downto 0) := (others => '0');
    signal su2, sv2 : std_logic := '0';

    signal a3, b3 : unsigned(8 downto 0) := (others => '0');
    signal oct3   : unsigned(2 downto 0) := (others => '0');
    signal sat3   : unsigned(9 downto 0) := (others => '0');

    signal t22_4, t11_4, t33_4 : unsigned(8 downto 0) := (others => '0');
    signal b4   : unsigned(8 downto 0) := (others => '0');
    signal oct4 : unsigned(2 downto 0) := (others => '0');
    signal sat4 : unsigned(9 downto 0) := (others => '0');

    signal idx5  : unsigned(4 downto 0) := (others => '0');
    signal gray5 : std_logic := '0';
    signal ring5 : unsigned(1 downto 0) := (others => '0');

    -- capture pipe (flag + cell address)
    signal capf : std_logic_vector(1 to 5) := (others => '0');
    type t_cxa is array (1 to 5) of unsigned(7 downto 0);
    signal cxap : t_cxa := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- cell-sample BRAM {luma8, idx5, ring2, gray1}
    ----------------------------------------------------------------------
    type t_cc is array (0 to 255) of std_logic_vector(15 downto 0);
    signal cc_ram   : t_cc := (others => x"0001");             -- gray init
    signal cc_q     : std_logic_vector(15 downto 0) := (others => '0');
    signal cc_raddr : unsigned(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- cell engine: NEXT-cell precompute registers (loaded over the
    -- current cell's pixel phases / the hblank priming sequence)
    ----------------------------------------------------------------------
    signal p_ha  : unsigned(11 downto 0) := (others => '0');
    signal n_ys8  : unsigned(7 downto 0) := (others => '0');
    signal n_idx  : unsigned(4 downto 0) := (others => '0');
    signal n_ring : unsigned(1 downto 0) := (others => '0');
    signal n_gray : std_logic := '1';
    signal n_h8   : unsigned(7 downto 0) := (others => '0');
    signal n_ang  : unsigned(9 downto 0) := (others => '0');
    signal n_sq   : unsigned(7 downto 0) := (others => '0');
    signal n_jx   : signed(6 downto 0) := (others => '0');
    signal n_jy   : signed(6 downto 0) := (others => '0');
    signal n_cs, n_sn : signed(7 downto 0) := (others => '0');
    signal n_r    : unsigned(4 downto 0) := (others => '0');
    signal n_dy   : signed(6 downto 0) := (others => '0');
    signal n_pu, n_pv : signed(16 downto 0) := (others => '0');
    signal n_dy2  : unsigned(9 downto 0) := (others => '0');
    signal n_dyg2 : unsigned(9 downto 0) := (others => '0');
    signal n_rr   : unsigned(9 downto 0) := (others => '0');
    signal n_rrg  : unsigned(9 downto 0) := (others => '0');
    signal n_dyok : std_logic := '0';
    signal n_dyrim : std_logic := '0';

    -- CURRENT-cell render registers (shifted at each cell's phase 0)
    signal c_ucol, c_vcol : unsigned(9 downto 0) := (others => '0');
    signal c_ys8  : unsigned(7 downto 0) := (others => '0');
    signal c_en   : std_logic := '0';
    signal c_thr  : signed(11 downto 0) := (others => '0');
    signal c_thrr : signed(11 downto 0) := (others => '0');
    signal c_thrg : signed(11 downto 0) := (others => '0');
    signal c_r    : unsigned(4 downto 0) := (others => '0');
    signal c_dyok : std_logic := '0';
    signal c_dyrim : std_logic := '0';
    signal c_ccx  : unsigned(10 downto 0) := (others => '0');

    signal pf_seq : unsigned(2 downto 0) := (others => '0');

    -- per-pixel cell-context pipe (the cell registers advance while a
    -- pixel is in flight; the last pixels of a cell must be tested
    -- against THEIR cell, not the next one)
    type t_kthr is array (2 to 4) of signed(11 downto 0);
    type t_ku10 is array (2 to 4) of unsigned(9 downto 0);
    type t_ku8  is array (2 to 4) of unsigned(7 downto 0);
    type t_ku5  is array (2 to 4) of unsigned(4 downto 0);
    signal k_thr, k_thrr, k_thrg : t_kthr := (others => (others => '0'));
    signal k_ucol, k_vcol : t_ku10 := (others => (others => '0'));
    signal k_ys8 : t_ku8 := (others => (others => '0'));
    signal k_r   : t_ku5 := (others => (others => '0'));
    signal k_en, k_dyok, k_dyrim : std_logic_vector(2 to 4) := (others => '0');

    ----------------------------------------------------------------------
    -- render lanes E2..E6
    ----------------------------------------------------------------------
    signal dxs2   : signed(7 downto 0) := (others => '0');
    signal ch_a2  : unsigned(11 downto 0) := (others => '0');

    signal dxa3, dxg3 : unsigned(4 downto 0) := (others => '0');
    signal ch8_3  : unsigned(7 downto 0) := (others => '0');

    signal dx2_4, dxg2_4 : unsigned(9 downto 0) := (others => '0');
    signal dxa4   : unsigned(4 downto 0) := (others => '0');
    signal cnv4   : signed(4 downto 0) := (others => '0');

    signal sel5   : unsigned(1 downto 0) := (others => '0');   -- bg/dab/gloss/rim
    signal py5, pu5, pv5 : unsigned(9 downto 0) := (others => '0');
    signal cnv5   : signed(4 downto 0) := (others => '0');

    signal wy6, wu6, wv6 : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- dry / sync delay + inline mix
    ----------------------------------------------------------------------
    type t_data_sr is array (0 to 5) of std_logic_vector(9 downto 0);
    signal s_y_sr, s_u_sr, s_v_sr : t_data_sr := (others => (others => '0'));

    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

    -- inline dry/wet lerp (M1..M3). The SDK interpolator_u's single-stage
    -- 10x11 s_product multiply was the routed critical path on HD Dual
    -- (5 seeds missing, HD HDMI marginal at 74.98 = structural, not seed
    -- luck): three instances of it, each t x (b-a) in one clock. Inline
    -- version cuts t to 8 bits (mercurial squeeze -- worst-case error is
    -- (b-a)/256 <= 4 LSB at full solvent, invisible; t=0 is still EXACT
    -- dry video, which is the property that matters) and gives the
    -- multiply a stage of its own. 3 clocks instead of 4.
    signal m1_ay, m1_au, m1_av : unsigned(9 downto 0) := (others => '0');
    signal m1_dy, m1_du, m1_dv : signed(10 downto 0) := (others => '0');
    signal m2_ay, m2_au, m2_av : unsigned(9 downto 0) := (others => '0');
    signal m2_py, m2_pu, m2_pv : signed(19 downto 0) := (others => '0');
    signal m3_y, m3_u, m3_v    : unsigned(9 downto 0) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- shared qsin: sequencer-only, angle + output registered
    ------------------------------------------------------------------------
    s_qs_a <= s_lang                          when s_seq = 10 else
              s_lang + to_unsigned(256, 10);

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer (one step per line)
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_z : natural range 0 to 4;
        variable v_b : unsigned(7 downto 0);
        variable v_m : unsigned(11 downto 0);
        variable v_e : integer range -4 to 2;
    begin
        if rising_edge(clk) then
            s_prev_vsync_n <= data_in.vsync_n;
            s_qs_ar <= s_qs_a;
            s_qs_r  <= f_qsin(s_qs_ar);

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_p12 <= unsigned(registers_in(7));
                s_k1  <= unsigned(registers_in(0));
                s_k2  <= unsigned(registers_in(1));
                s_k4  <= unsigned(registers_in(3));
                s_k5  <= unsigned(registers_in(4));
                s_k6  <= unsigned(registers_in(5));
                s_vivid  <= registers_in(6)(0);    -- switch 7
                s_velvet <= registers_in(6)(1);    -- switch 8
                s_square <= registers_in(6)(2);    -- switch 9
                s_rim_on <= registers_in(6)(3);    -- switch 10
                s_orbit  <= registers_in(6)(4);    -- switch 11

                s_orbit_acc <= s_orbit_acc + 3;
                if registers_in(6)(4) = '1' then
                    s_lang <= unsigned(registers_in(2)) + s_orbit_acc;
                else
                    s_lang <= unsigned(registers_in(2));
                end if;

                s_seq <= to_unsigned(10, 4);
            elsif s_seq /= 0 and data_in.avid = '0' then
                case to_integer(s_seq) is
                    when 10 =>
                        -- Solvent: full painting by ~58%, then the photo
                        -- background strips to canvas, varnish past 80%
                        -- (qsin presented: light angle)
                        v_m := resize(s_p12, 12) + resize(s_p12(9 downto 1), 12)
                               + resize(s_p12(9 downto 2), 12);
                        if v_m > 1023 then
                            s_mix10 <= (others => '1');
                        else
                            s_mix10 <= v_m(9 downto 0);
                        end if;
                        if s_p12 > 800 then
                            s_bgm <= "11"; s_varn <= '1';
                        elsif s_p12 > 704 then
                            s_bgm <= "10"; s_varn <= '0';
                        elsif s_p12 > 608 then
                            s_bgm <= "01"; s_varn <= '0';
                        else
                            s_bgm <= "00"; s_varn <= '0';
                        end if;
                    when 9 =>
                        -- cell size (K2 zone) + pigment mask (K1 zone,
                        -- knob up = more pigments)
                        -- (qsin presented: light angle + 256)
                        s_csh <= 3 + to_integer(s_k2(9 downto 8));
                        case s_k2(9 downto 8) is
                            when "00" =>
                                s_cmask <= to_unsigned(7, 6);
                                s_chalf <= to_unsigned(4, 6);
                                s_r3 <= to_unsigned(4, 5);
                            when "01" =>
                                s_cmask <= to_unsigned(15, 6);
                                s_chalf <= to_unsigned(8, 6);
                                s_r3 <= to_unsigned(7, 5);
                            when "10" =>
                                s_cmask <= to_unsigned(31, 6);
                                s_chalf <= to_unsigned(16, 6);
                                s_r3 <= to_unsigned(14, 5);
                            when others =>
                                s_cmask <= to_unsigned(63, 6);
                                s_chalf <= to_unsigned(32, 6);
                                s_r3 <= to_unsigned(29, 5);
                        end case;
                        if s_k1(9 downto 7) > 4 then
                            v_z := 0;
                        else
                            v_z := 4 - to_integer(s_k1(9 downto 7));
                        end if;
                        case v_z is
                            when 0 => s_mask5 <= "11111";
                                      s_half10 <= to_unsigned(16, 10);
                            when 1 => s_mask5 <= "11110";
                                      s_half10 <= to_unsigned(32, 10);
                            when 2 => s_mask5 <= "11100";
                                      s_half10 <= to_unsigned(64, 10);
                            when 3 => s_mask5 <= "11000";
                                      s_half10 <= to_unsigned(128, 10);
                            when others => s_mask5 <= "10000";
                                      s_half10 <= to_unsigned(256, 10);
                        end case;
                    when 8 =>
                        -- capture two-behind: qsin(light angle) = ly;
                        -- highlight offset scales with cell size
                        s_ly8 <= resize(shift_right(s_qs_r, 2), 8);
                        s_oy  <= resize(shift_right(
                                     resize(shift_right(s_qs_r, 2), 8),
                                     10 - s_csh), 6);
                        -- dab radii per saturation ring
                        s_r2 <= s_r3 - resize(s_r3(4 downto 2), 5);
                        s_r1 <= '0' & s_r3(4 downto 1);
                    when 7 =>
                        -- capture: qsin(light angle + 256) = lx
                        s_lx8 <= resize(shift_right(s_qs_r, 2), 8);
                        s_ox  <= resize(shift_right(
                                     resize(shift_right(s_qs_r, 2), 8),
                                     10 - s_csh), 6);
                        s_smr <= to_integer(s_k6(9 downto 8));
                        s_sct <= s_k4(9 downto 8);
                        -- throw shift: |jx|max = chalf >> z, and the
                        -- 4-bit hash already carries magnitude 2^3, so
                        -- the net shift is (csh-1) - z - 3
                        if s_k4(9 downto 8) = "00" then
                            s_jz  <= '1';
                            s_jsl <= 0;
                            s_jsr <= 0;
                        else
                            s_jz <= '0';
                            v_e := s_csh - 4 - (4 - to_integer(s_k4(9 downto 8)));
                            if v_e >= 0 then
                                s_jsl <= v_e;
                                s_jsr <= 0;
                            else
                                s_jsl <= 0;
                                s_jsr <= -v_e;
                            end if;
                        end if;
                    when 6 =>
                        -- Load: which rings earn paint (top zone = sparse
                        -- shrunken dots)
                        case s_k5(9 downto 8) is
                            when "00"   => s_gate2 <= "01"; s_sparse <= '0';
                            when "01"   => s_gate2 <= "10"; s_sparse <= '0';
                            when "10"   => s_gate2 <= "11"; s_sparse <= '0';
                            when others => s_gate2 <= "11"; s_sparse <= '1';
                        end case;
                        if s_varn = '1' then
                            s_glb <= to_unsigned(127, 7);      -- varnish
                        else
                            s_glb <= to_unsigned(96, 7);
                        end if;
                    when 5 =>
                        if s_vivid = '1' then
                            v_b := to_unsigned(200, 8);
                        else
                            v_b := to_unsigned(216, 8);
                        end if;
                        s_sq3 <= v_b;
                    when 4 =>
                        if s_vivid = '1' then
                            s_sq2 <= to_unsigned(200, 8);
                        else
                            s_sq2 <= to_unsigned(152, 8);
                        end if;
                    when 3 =>
                        if s_vivid = '1' then
                            s_sq1 <= to_unsigned(200, 8);
                        else
                            s_sq1 <= to_unsigned(88, 8);
                        end if;
                    when 2 =>
                        -- canvas ground + rim colour (rim contrasts it)
                        if s_velvet = '1' then
                            s_cv_y <= to_unsigned(80, 10);
                            s_cv_u <= to_unsigned(512, 10);
                            s_cv_v <= to_unsigned(512, 10);
                            s_rim_y <= to_unsigned(760, 10);
                        else
                            s_cv_y <= to_unsigned(816, 10);
                            s_cv_u <= to_unsigned(502, 10);
                            s_cv_v <= to_unsigned(520, 10);
                            s_rim_y <= to_unsigned(96, 10);
                        end if;
                    when others =>
                        null;
                end case;
                s_seq <= s_seq - 1;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- main pixel pipeline
    ------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_cu, v_cv : signed(9 downto 0);
        variable v_swap : std_logic;
        variable v_a, v_b : unsigned(8 downto 0);
        variable v_oct  : unsigned(2 downto 0);
        variable v_c22, v_c11, v_c33 : std_logic;
        variable v_sub  : unsigned(1 downto 0);
        variable v_step : unsigned(2 downto 0);
        variable v_sv   : std_logic;
        variable v_cxn  : unsigned(7 downto 0);
        variable v_j    : signed(4 downto 0);
        variable v_r    : unsigned(5 downto 0);
        variable v_h    : unsigned(5 downto 0);
        variable v_dyg  : signed(7 downto 0);
        variable v_dxs  : signed(11 downto 0);
        variable v_in, v_rim, v_glo : std_logic;
        variable v_y    : unsigned(10 downto 0);
        variable v_bgy  : unsigned(9 downto 0);
        variable v_bgu, v_bgv : unsigned(9 downto 0);
        variable v_h8   : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;

            --------------------------------------------------------------
            -- E1: input registers + position
            --------------------------------------------------------------
            s_in_y <= unsigned(data_in.y);
            s_in_u <= unsigned(data_in.u);
            s_in_v <= unsigned(data_in.v);
            px1    <= s_x_count;
            if data_in.avid = '1' then
                s_x_count <= s_x_count + 1;
                s_seen    <= '1';
            end if;

            s_y_sr(0) <= data_in.y;
            s_u_sr(0) <= data_in.u;
            s_v_sr(0) <= data_in.v;
            for i in 1 to 5 loop
                s_y_sr(i) <= s_y_sr(i - 1);
                s_u_sr(i) <= s_u_sr(i - 1);
                s_v_sr(i) <= s_v_sr(i - 1);
            end loop;

            -- capture flag: cell-centre pixel of the first line of each
            -- cell row (rides the quantizer pipe, writes the BRAM at E6)
            if data_in.avid = '1' and s_capline = '1'
               and (s_x_count and resize(s_cmask, 11))
                   = resize(s_chalf, 11) then
                capf(1) <= '1';
            else
                capf(1) <= '0';
            end if;
            cxap(1) <= resize(shift_right(s_x_count, s_csh), 8);
            capf(2 to 5) <= capf(1 to 4);
            for i in 2 to 5 loop
                cxap(i) <= cxap(i - 1);
            end loop;

            --------------------------------------------------------------
            -- CELL ENGINE: the next cell's paint is fully precomputed
            -- across the current cell's pixel phases (or the hblank
            -- priming sequence pf_seq for cell 0 of each line)
            --------------------------------------------------------------
            v_step := "111";
            v_sv   := '0';
            if pf_seq /= 0 then
                v_step := pf_seq;
                v_sv   := '1';
                v_cxn  := (others => '0');
                pf_seq <= pf_seq + 1;
                if pf_seq = 7 then
                    pf_seq <= (others => '0');
                end if;
            elsif data_in.avid = '1'
                  and (s_x_count and resize(s_cmask, 11) and
                       not to_unsigned(7, 11)) = 0 then
                v_step := s_x_count(2 downto 0);
                v_sv   := '1';
                v_cxn  := resize(shift_right(s_x_count, s_csh), 8) + 1;
            end if;

            if v_sv = '1' then
                case v_step is
                    when "001" =>
                        cc_raddr <= v_cxn;
                    when "010" =>
                        p_ha <= f_hash_a(v_cxn, s_cyi, x"B0B5");
                    when "011" =>
                        n_ys8  <= unsigned(cc_q(15 downto 8));
                        n_idx  <= unsigned(cc_q(7 downto 3));
                        n_ring <= unsigned(cc_q(2 downto 1));
                        n_gray <= cc_q(0);
                        n_h8   <= f_hash_b(p_ha);
                    when "100" =>
                        n_ang <= shift_left(resize(n_idx and s_mask5, 10), 5)
                                 + s_half10;
                        case n_ring is
                            when "11"   => n_sq <= s_sq3;
                            when "10"   => n_sq <= s_sq2;
                            when others => n_sq <= s_sq1;
                        end case;
                        if n_gray = '1' then
                            n_sq <= (others => '0');
                        end if;
                        -- scatter throw from the cell hash, scaled to the
                        -- cell size (jsl/jsr are per-frame; one is 0)
                        if s_jz = '1' then
                            n_jx <= (others => '0');
                            n_jy <= (others => '0');
                        else
                            n_jx <= shift_left(shift_right(
                                signed(resize(n_h8(3 downto 0), 7)) - 8,
                                s_jsr), s_jsl);
                            n_jy <= shift_left(shift_right(
                                signed(resize(n_h8(7 downto 4), 7)) - 8,
                                s_jsr), s_jsl);
                        end if;
                    when "101" =>
                        n_cs <= resize(shift_right(f_qsin(n_ang + 256), 2), 8);
                        n_sn <= resize(shift_right(f_qsin(n_ang), 2), 8);
                        -- radius: saturation ring, size jitter at max
                        -- scatter, sparse Load shrink
                        case n_ring is
                            when "11"   => v_r := '0' & s_r3;
                            when "10"   => v_r := '0' & s_r2;
                            when others => v_r := '0' & s_r1;
                        end case;
                        -- Scatter shrinks the dab to make room for the
                        -- throw: r' + |j|max + 1 <= chalf holds in every
                        -- zone, so dabs are always whole (never clipped
                        -- by their cell edge). Size jitter is shrink-only
                        -- for the same reason.
                        case s_sct is
                            when "00"   => null;
                            when "01"   => v_r := v_r - shift_right(v_r, 3);
                            when "10"   => v_r := v_r - shift_right(v_r, 2);
                            when others =>
                                -- guarded: an unsigned underflow here
                                -- wraps to a giant radius that the
                                -- upper clamp happily accepts
                                v_r := shift_right(v_r, 1);
                                v_h := resize(unsigned(n_h8(6 downto 5)), 6);
                                if v_r > v_h then
                                    v_r := v_r - v_h;
                                else
                                    v_r := (others => '0');
                                end if;
                        end case;
                        if s_sparse = '1' then
                            v_r := '0' & v_r(5 downto 1);
                        end if;
                        if v_r > 29 then
                            v_r := to_unsigned(29, 6);
                        elsif v_r < 2 then
                            v_r := to_unsigned(2, 6);
                        end if;
                        n_r  <= v_r(4 downto 0);
                        n_dy <= s_dyb + resize(n_jy, 7);
                    when "110" =>
                        n_pu <= signed('0' & n_sq) * n_sn;
                        n_pv <= signed('0' & n_sq) * n_cs;
                        n_dy2 <= f_sq(f_abs31(resize(n_dy, 8)));
                        v_dyg := resize(n_dy, 8) - resize(s_oy, 8);
                        n_dyg2 <= f_sq(f_abs31(v_dyg));
                        n_rr  <= f_sq(n_r);
                        n_rrg <= f_sq('0' & n_r(4 downto 1));
                        if resize(f_abs31(resize(n_dy, 8)), 6)
                           <= ('0' & n_r) then
                            n_dyok <= '1';
                        else
                            n_dyok <= '0';
                        end if;
                        if resize(f_abs31(resize(n_dy, 8)), 6) + 1
                           >= ('0' & n_r) then
                            n_dyrim <= '1';
                        else
                            n_dyrim <= '0';
                        end if;
                    when others =>
                        null;
                end case;
            end if;

            -- phase 0 of every cell: commit the precomputed paint
            if data_in.avid = '1'
               and (s_x_count and resize(s_cmask, 11)) = 0 then
                c_ucol <= f_cu10(to_signed(512, 12)
                                 + resize(shift_right(n_pu, 7), 12));
                c_vcol <= f_cu10(to_signed(512, 12)
                                 + resize(shift_right(n_pv, 7), 12));
                c_ys8  <= n_ys8;
                c_r    <= n_r;
                c_dyok <= n_dyok;
                c_dyrim <= n_dyrim;
                c_thr  <= signed(resize(n_rr, 12))
                          - signed(resize(n_dy2, 12));
                c_thrr <= signed(resize(n_rr, 12))
                          - signed(resize(n_dy2, 12))
                          - signed(resize(n_r & '0', 12)) + 1;
                c_thrg <= signed(resize(n_rrg, 12))
                          - signed(resize(n_dyg2, 12));
                -- x & cmask = 0 here, so x IS the cell base
                c_ccx  <= s_x_count + resize(s_chalf, 11)
                          + unsigned(resize(n_jx, 11));
                if n_gray = '0' and n_ring >= s_gate2
                   and s_capline = '0' then
                    c_en <= '1';
                else
                    c_en <= '0';
                end if;
            end if;

            --------------------------------------------------------------
            -- E2..E5: hue-sector quantizer (capture path, v2 machinery)
            --------------------------------------------------------------
            v_cu := signed((not s_in_u(9)) & s_in_u(8 downto 0));
            v_cv := signed((not s_in_v(9)) & s_in_v(8 downto 0));
            au2 <= f_abs9(v_cu);
            av2 <= f_abs9(v_cv);
            su2 <= v_cu(9);
            sv2 <= v_cv(9);

            v_swap := '0';
            if au2 > av2 then
                v_swap := '1';
                v_a := au2;
                v_b := av2;
            else
                v_a := av2;
                v_b := au2;
            end if;
            a3 <= v_a;
            b3 <= v_b;
            sat3 <= resize(v_a, 10) + resize(v_b(8 downto 1), 10);
            if sv2 = '0' and su2 = '0' then
                if v_swap = '0' then v_oct := "000"; else v_oct := "001"; end if;
            elsif sv2 = '1' and su2 = '0' then
                if v_swap = '1' then v_oct := "010"; else v_oct := "011"; end if;
            elsif sv2 = '1' and su2 = '1' then
                if v_swap = '0' then v_oct := "100"; else v_oct := "101"; end if;
            else
                if v_swap = '1' then v_oct := "110"; else v_oct := "111"; end if;
            end if;
            oct3 <= v_oct;

            t22_4 <= resize(a3(8 downto 1), 9) - resize(a3(8 downto 4), 9)
                     - resize(a3(8 downto 5), 9) + resize(a3(8 downto 7), 9);
            t11_4 <= resize(a3(8 downto 2), 9) - resize(a3(8 downto 4), 9)
                     + resize(a3(8 downto 7), 9);
            t33_4 <= resize(a3(8 downto 1), 9) + resize(a3(8 downto 3), 9)
                     + resize(a3(8 downto 5), 9) + resize(a3(8 downto 7), 9);
            b4   <= b3;
            oct4 <= oct3;
            sat4 <= sat3;

            v_c22 := '0'; v_c11 := '0'; v_c33 := '0';
            if b4 > t22_4 then v_c22 := '1'; end if;
            if b4 > t11_4 then v_c11 := '1'; end if;
            if b4 > t33_4 then v_c33 := '1'; end if;
            if v_c22 = '1' then
                v_sub(1) := '1';
                v_sub(0) := v_c33;
            else
                v_sub(1) := '0';
                v_sub(0) := v_c11;
            end if;
            if oct4(0) = '1' then
                v_sub := not v_sub;
            end if;
            idx5 <= oct4 & v_sub;
            if sat4 < C_GRAY then
                gray5 <= '1';
            else
                gray5 <= '0';
            end if;
            if sat4 > C_RHI then
                ring5 <= "11";
            elsif sat4 > C_RLO then
                ring5 <= "10";
            else
                ring5 <= "01";
            end if;

            --------------------------------------------------------------
            -- E2..E5: dab render lanes (shallow: abs -> LUT -> compares)
            --------------------------------------------------------------
            v_dxs := signed(resize(px1, 12)) - signed(resize(c_ccx, 12));
            if v_dxs > 127 then
                dxs2 <= to_signed(127, 8);
            elsif v_dxs < -127 then
                dxs2 <= to_signed(-127, 8);
            else
                dxs2 <= resize(v_dxs, 8);
            end if;
            ch_a2 <= f_hash_a(px1(9 downto 2), s_aline(9 downto 2), x"CA9A");

            -- capture this pixel's cell context (the c_* registers move
            -- on to the next cell while the pixel is in flight)
            k_thr(2)  <= c_thr;
            k_thrr(2) <= c_thrr;
            k_thrg(2) <= c_thrg;
            k_ucol(2) <= c_ucol;
            k_vcol(2) <= c_vcol;
            k_ys8(2)  <= c_ys8;
            k_r(2)    <= c_r;
            k_en(2)   <= c_en;
            k_dyok(2) <= c_dyok;
            k_dyrim(2) <= c_dyrim;
            for i in 3 to 4 loop
                k_thr(i)  <= k_thr(i - 1);
                k_thrr(i) <= k_thrr(i - 1);
                k_thrg(i) <= k_thrg(i - 1);
                k_ucol(i) <= k_ucol(i - 1);
                k_vcol(i) <= k_vcol(i - 1);
                k_ys8(i)  <= k_ys8(i - 1);
                k_r(i)    <= k_r(i - 1);
                k_en(i)   <= k_en(i - 1);
                k_dyok(i) <= k_dyok(i - 1);
                k_dyrim(i) <= k_dyrim(i - 1);
            end loop;

            dxa3  <= f_abs31(shift_right(dxs2, s_smr));
            dxg3  <= f_abs31(shift_right(dxs2 - resize(s_ox, 8), s_smr));
            ch8_3 <= f_hash_b(ch_a2);

            dx2_4  <= f_sq(dxa3);
            dxg2_4 <= f_sq(dxg3);
            dxa4   <= dxa3;
            cnv4   <= signed(resize(ch8_3(7 downto 4), 5)) - 8;

            -- E5: shape tests + lane select + paint colours (all against
            -- the PIPED cell context)
            v_in  := '0';
            v_rim := '0';
            v_glo := '0';
            if s_square = '1' then
                if k_en(4) = '1' and k_dyok(4) = '1'
                   and resize(dxa4, 6) <= ('0' & k_r(4)) then
                    v_in := '1';
                    if resize(dxa4, 6) + 1 >= ('0' & k_r(4))
                       or k_dyrim(4) = '1' then
                        v_rim := '1';
                    end if;
                end if;
            else
                if k_en(4) = '1'
                   and signed(resize(dx2_4, 12)) <= k_thr(4) then
                    v_in := '1';
                    if signed(resize(dx2_4, 12)) > k_thrr(4) then
                        v_rim := '1';
                    end if;
                end if;
            end if;
            if signed(resize(dxg2_4, 12)) <= k_thrg(4) then
                v_glo := '1';
            end if;

            if v_in = '1' and v_rim = '1' and s_rim_on = '1' then
                sel5 <= "11";                                  -- rim
            elsif v_in = '1' and v_glo = '1' then
                sel5 <= "10";                                  -- highlight
            elsif v_in = '1' then
                sel5 <= "01";                                  -- dab body
            else
                sel5 <= "00";                                  -- background
            end if;
            py5 <= k_ys8(4) & "00";
            pu5 <= k_ucol(4);
            pv5 <= k_vcol(4);
            cnv5 <= cnv4;

            --------------------------------------------------------------
            -- E6: compose wet result (paint lanes over the background,
            -- background morphs photo -> canvas with the Solvent zone)
            --------------------------------------------------------------
            v_bgy := f_cu10(signed(resize(s_cv_y, 12))
                            + resize(cnv5, 12));
            v_bgu := s_cv_u;
            v_bgv := s_cv_v;
            case s_bgm is
                when "00" =>
                    v_bgy := unsigned(s_y_sr(4));
                    v_bgu := unsigned(s_u_sr(4));
                    v_bgv := unsigned(s_v_sr(4));
                when "01" =>
                    v_bgy := ('0' & unsigned(s_y_sr(4)(9 downto 1)))
                             + ('0' & v_bgy(9 downto 1));
                    v_bgu := ('0' & unsigned(s_u_sr(4)(9 downto 1)))
                             + ('0' & v_bgu(9 downto 1));
                    v_bgv := ('0' & unsigned(s_v_sr(4)(9 downto 1)))
                             + ('0' & v_bgv(9 downto 1));
                when others =>
                    null;
            end case;

            case sel5 is
                when "11" =>                                   -- rim
                    wy6 <= s_rim_y;
                    wu6 <= to_unsigned(512, 10);
                    wv6 <= to_unsigned(512, 10);
                when "10" =>                                   -- highlight
                    v_y := resize(py5, 11) + resize(s_glb, 11);
                    if v_y > 1023 then
                        wy6 <= (others => '1');
                    else
                        wy6 <= v_y(9 downto 0);
                    end if;
                    wu6 <= pu5;
                    wv6 <= pv5;
                when "01" =>                                   -- dab body
                    wy6 <= py5;
                    wu6 <= pu5;
                    wv6 <= pv5;
                when others =>                                 -- background
                    wy6 <= v_bgy;
                    wu6 <= v_bgu;
                    wv6 <= v_bgv;
            end case;

            --------------------------------------------------------------
            -- line / field bookkeeping
            --------------------------------------------------------------
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_x_count <= (others => '0');
                if s_seen = '1' then
                    s_seen <= '0';
                    if s_aline < 1000 then
                        s_aline <= s_aline + 1;
                    end if;
                    s_cyi <= resize(shift_right(s_aline + 1, s_csh), 8);
                    s_dyb <= signed(resize((s_aline + 1)
                                           and resize(s_cmask, 10), 7))
                             - signed(resize(s_chalf, 7));
                    if ((s_aline + 1) and resize(s_cmask, 10)) = 0 then
                        s_capline <= '1';
                    else
                        s_capline <= '0';
                    end if;
                else
                    s_aline <= (others => '0');
                    s_cyi <= (others => '0');
                    s_dyb <= -signed(resize(s_chalf, 7));
                    s_capline <= '1';
                end if;
                pf_seq <= "001";   -- prime cell 0 of the coming line
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_aline <= (others => '0');
            end if;
        end if;
    end process p_pix;

    ------------------------------------------------------------------------
    -- cell-sample BRAM: write from the quantizer pipe (E6, once per cell
    -- per cell-row), read by the cell engine prefetch -- canonical 1W1R
    ------------------------------------------------------------------------
    p_ccram : process(clk)
    begin
        if rising_edge(clk) then
            cc_q <= cc_ram(to_integer(cc_raddr));
            if capf(5) = '1' then
                cc_ram(to_integer(cxap(5))) <=
                    s_y_sr(4)(9 downto 2) & std_logic_vector(idx5)
                    & std_logic_vector(ring5) & gray5;
            end if;
        end if;
    end process p_ccram;

    ------------------------------------------------------------------------
    -- dry/wet mix, inline: result = a + (t8 * (b - a)) >> 8.
    -- M1 diff (wet is valid 6 clocks after data_in -> dry tap index 5),
    -- M2 the multiply alone in its stage, M3 sum + clamp.
    ------------------------------------------------------------------------
    p_mix : process(clk)
        variable v_s : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- M1: diff = wet - dry, dry carried alongside
            m1_ay <= unsigned(s_y_sr(5));
            m1_au <= unsigned(s_u_sr(5));
            m1_av <= unsigned(s_v_sr(5));
            m1_dy <= signed(resize(wy6, 11)) - signed(resize(unsigned(s_y_sr(5)), 11));
            m1_du <= signed(resize(wu6, 11)) - signed(resize(unsigned(s_u_sr(5)), 11));
            m1_dv <= signed(resize(wv6, 11)) - signed(resize(unsigned(s_v_sr(5)), 11));

            -- M2: the multiplies, alone, pure reg x reg
            m2_py <= signed('0' & s_mix10(9 downto 2)) * m1_dy;
            m2_pu <= signed('0' & s_mix10(9 downto 2)) * m1_du;
            m2_pv <= signed('0' & s_mix10(9 downto 2)) * m1_dv;
            m2_ay <= m1_ay;
            m2_au <= m1_au;
            m2_av <= m1_av;

            -- M3: a + prod>>8, clamped
            v_s := signed(resize(m2_ay, 13)) + resize(m2_py(19 downto 8), 13);
            m3_y <= f_cu10(v_s);
            v_s := signed(resize(m2_au, 13)) + resize(m2_pu(19 downto 8), 13);
            m3_u <= f_cu10(v_s);
            v_s := signed(resize(m2_av, 13)) + resize(m2_pv(19 downto 8), 13);
            m3_v <= f_cu10(v_s);
        end if;
    end process p_mix;

    ------------------------------------------------------------------------
    -- sync delay + output
    ------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= data_in.avid    & s_avid_sr   (0 to C_LATENCY - 2);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    data_out.y       <= std_logic_vector(m3_y);
    data_out.u       <= std_logic_vector(m3_u);
    data_out.v       <= std_logic_vector(m3_v);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture turpentine;
