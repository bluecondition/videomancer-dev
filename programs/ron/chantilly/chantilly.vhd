-- chantilly.vhd  (v0.1 -- black-lace halftone: video as cream circles on black)
--
-- Source image: black Chantilly-style lace -- bands of white/cream dots in
-- fans, rosettes and pebble rows on black ground, with a thick black wavy
-- cordonnet breaking over the dots.  This program re-renders incoming video
-- as that lace.
--
-- THE MAPPING (grid-free by construction):
--   The frame is cut into horizontal LACE BANDS (256 lace units tall).  Each
--   band owns a row of scallop CENTRES (spaced P apart, half-P staggered on
--   alternate bands); every pixel is taken to polar (r, theta) about its
--   nearest centre by a CORDIC vectoring pipeline.  Dots sit on CONCENTRIC
--   ARCS: ring = r/G, and the number of dots around a ring is 6*ring, so the
--   arc cells stay ~square at any radius yet never line up radially -- a
--   phyllotaxis-like packing with no Cartesian grid anywhere.  The dot test
--   is  frac_r^2 + frac_t^2 < radius^2  in NORMALIZED CELL UNITS (Q7,
--   128 = half a cell), so one compare renders every dot at every scale.
--
--   Four band styles cycle down the frame (the image's texture variety):
--     0 PEBBLE  centre 2 bands deep, G=32 -> gentle arcs of medium circles
--     1 FAN     centre at band base,  G=32 -> radiating spiderweb fans
--     2 SEED    centre 2 bands deep,  G=16 -> dense small-dot packing
--     3 BLOOM   centre mid-band, P=256, G=16 -> full rosette flowers
--
--   Dot radius comes from the DELAYED INPUT LUMA at the pixel (analog
--   halftone): bright video = fat merging cream dots, dark = pinpricks.
--
-- THE CORDONNET (black linework), two sources:
--   * r > R_edge of the band's fans is painted black -- the fan's own outer
--     radius IS a scalloped thick black border, cusping exactly at the fan
--     seams (no sine tables, no extra geometry).  A thin seam wisp at
--     |dx| ~ P/2 joins the cusps.  K5 sets the border weight.
--   * luma-gradient INK: strong vertical edges in the video become black
--     strokes that cut down through the dots (S9, weight on K4).
--
-- SCALE (P12) is a Q8 DDA: x,y are accumulated in LACE UNITS (u += s per
-- pixel), so every pitch stays a power of two in lace space while the screen
-- scale sweeps continuously -- the whole doily zooms without a single
-- variable modulo.  DRIFT (K3) scrolls lace space vertically; band structure
-- wraps mod 4096 lace units so the cycle is seamless.
--
-- Streaming, no BRAM, no line buffer.  Per-pixel multiplies: theta*ring,
-- ring*twist, three Q7 squares, luma*gain -- all narrow, each in its own
-- stage.  Blanking-gated output (Y=64, U=V=512 outside avid).
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture chantilly of program_top is

    constant C_LATENCY : integer := 19;
    constant C_MID     : unsigned(9 downto 0) := to_unsigned(512, 10);

    constant C_DB   : integer := 14;      -- knob detent deadband

    -- lace-space anchors (Q8): keep u,v positive; both wrap mod 2^20 = 4096
    -- lace units, which the band cycle (16 bands) divides exactly.
    constant C_UOFF : unsigned(19 downto 0) := to_unsigned(524288, 20);
    constant C_VOFF : unsigned(19 downto 0) := to_unsigned(524288, 20);

    -- band style constants (lace units, Q2 where noted)
    --   style 0 PEBBLE   1 FAN      2 SEED     3 BLOOM
    -- depth (Q2):  2048      1024       2048       512
    -- P shift:     9         9          9          8
    -- G shift:     5         5          4          4
    -- R_edge(Q2):  2240      1344       2240       704
    constant C_DEPTH0 : signed(12 downto 0) := to_signed(2048, 13);
    constant C_DEPTH1 : signed(12 downto 0) := to_signed(1024, 13);
    constant C_DEPTH3 : signed(12 downto 0) := to_signed(512, 13);
    constant C_REDGE0 : integer := 2240;
    constant C_REDGE1 : integer := 1344;
    constant C_REDGE3 : integer := 704;
    constant C_SEAM9  : integer := 1012;  -- |dx| beyond this (Q2) = seam wisp, P=512
    constant C_SEAM8  : integer := 500;   -- P=256

    ----------------------------------------------------------------------
    -- exp2 mantissa ROM: round(1024 * 2^(f/64)), f = 0..63  -> [1024,2048)
    ----------------------------------------------------------------------
    type t_mant is array(0 to 63) of unsigned(10 downto 0);
    constant C_MANT : t_mant := (
        to_unsigned(1024,11), to_unsigned(1035,11), to_unsigned(1046,11), to_unsigned(1058,11),
        to_unsigned(1069,11), to_unsigned(1081,11), to_unsigned(1093,11), to_unsigned(1105,11),
        to_unsigned(1117,11), to_unsigned(1129,11), to_unsigned(1141,11), to_unsigned(1154,11),
        to_unsigned(1166,11), to_unsigned(1179,11), to_unsigned(1192,11), to_unsigned(1205,11),
        to_unsigned(1218,11), to_unsigned(1231,11), to_unsigned(1244,11), to_unsigned(1258,11),
        to_unsigned(1272,11), to_unsigned(1286,11), to_unsigned(1300,11), to_unsigned(1314,11),
        to_unsigned(1328,11), to_unsigned(1342,11), to_unsigned(1357,11), to_unsigned(1372,11),
        to_unsigned(1387,11), to_unsigned(1402,11), to_unsigned(1417,11), to_unsigned(1433,11),
        to_unsigned(1448,11), to_unsigned(1464,11), to_unsigned(1480,11), to_unsigned(1496,11),
        to_unsigned(1512,11), to_unsigned(1529,11), to_unsigned(1545,11), to_unsigned(1562,11),
        to_unsigned(1579,11), to_unsigned(1596,11), to_unsigned(1614,11), to_unsigned(1631,11),
        to_unsigned(1649,11), to_unsigned(1667,11), to_unsigned(1685,11), to_unsigned(1704,11),
        to_unsigned(1722,11), to_unsigned(1741,11), to_unsigned(1760,11), to_unsigned(1779,11),
        to_unsigned(1798,11), to_unsigned(1818,11), to_unsigned(1838,11), to_unsigned(1858,11),
        to_unsigned(1878,11), to_unsigned(1898,11), to_unsigned(1919,11), to_unsigned(1940,11),
        to_unsigned(1961,11), to_unsigned(1983,11), to_unsigned(2004,11), to_unsigned(2026,11));

    ----------------------------------------------------------------------
    -- frame-latched controls
    ----------------------------------------------------------------------
    signal s_k_fill  : unsigned(9 downto 0) := to_unsigned(220, 10);
    signal s_k_gain  : unsigned(9 downto 0) := to_unsigned(240, 10);
    signal s_k_drift : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k_ink   : unsigned(9 downto 0) := to_unsigned(600, 10);
    signal s_k_bord  : unsigned(9 downto 0) := to_unsigned(300, 10);
    signal s_k_twist : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_zoom_k  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_ivory   : std_logic := '1';    -- S7 Ivory/Mono
    signal s_inv     : std_logic := '0';    -- S8 invert (white ground)
    signal s_inkon   : std_logic := '1';    -- S9 ink lines
    signal s_weave   : std_logic := '0';    -- S10 0=Doily(4 styles) 1=Pebble(calm)
    signal s_ghost   : std_logic := '0';    -- S11 dim video in the ground

    ----------------------------------------------------------------------
    -- runtime raster measurement
    ----------------------------------------------------------------------
    signal s_xcnt  : unsigned(11 downto 0) := (others => '0');
    signal s_lcnt  : unsigned(11 downto 0) := (others => '0');
    signal s_W     : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_H     : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal s_ilace : std_logic := '0';
    signal s_fpar  : std_logic := '0';
    signal s_avid_q : std_logic := '0';

    ----------------------------------------------------------------------
    -- per-frame resolved terms
    ----------------------------------------------------------------------
    signal s_scl   : unsigned(9 downto 0) := to_unsigned(256, 10);  -- scale s, Q8
    signal s_u0    : unsigned(19 downto 0) := (others => '0');      -- line-start u
    signal s_v0    : unsigned(19 downto 0) := (others => '0');      -- frame-start v
    signal s_scroll : unsigned(19 downto 0) := (others => '0');     -- drift accum, Q8
    signal s_gain  : unsigned(8 downto 0) := to_unsigned(120, 9);   -- luma slope
    signal s_bias  : unsigned(7 downto 0) := to_unsigned(55, 8);    -- dot radius floor
    signal s_twist : signed(7 downto 0) := (others => '0');
    signal s_inkth : unsigned(7 downto 0) := to_unsigned(60, 8);
    signal s_bw    : unsigned(8 downto 0) := to_unsigned(150, 9);   -- border weight Q2
    signal s_re0, s_re1, s_re2, s_re3 : unsigned(11 downto 0) := (others => '1');
    signal s_svi   : integer range -2 to 1 := 0;                    -- scale exp int part
    signal s_svf   : unsigned(5 downto 0) := (others => '0');       -- scale exp frac

    -- relay copies of pixel-rate multiply operands (placement decoupling)
    signal s_twist_d : signed(7 downto 0) := (others => '0');
    signal s_gain_d  : unsigned(8 downto 0) := to_unsigned(120, 9);

    -- vblank sequencer + one shared multiplier
    signal s_seq   : unsigned(4 downto 0) := (others => '0');
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse   : std_logic := '0';
    signal s_ma    : signed(12 downto 0) := (others => '0');
    signal s_mb    : signed(10 downto 0) := (others => '0');
    signal s_mp    : signed(23 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- lace-space DDA accumulators (Q8)
    ----------------------------------------------------------------------
    signal s_u     : unsigned(19 downto 0) := (others => '0');
    signal s_vb    : unsigned(19 downto 0) := (others => '0');     -- next line
    signal s_vc    : unsigned(19 downto 0) := (others => '0');     -- current line
    signal s_avid_p : std_logic := '0';
    signal s_newframe : std_logic := '1';

    ----------------------------------------------------------------------
    -- pixel pipeline
    ----------------------------------------------------------------------
    -- A1 (#2): band/style split, local centre coords (Q2)
    signal a1_dx   : signed(12 downto 0) := (others => '0');
    signal a1_dy   : signed(12 downto 0) := (others => '0');
    signal a1_sty  : unsigned(1 downto 0) := (others => '0');
    signal a1_seam : std_logic := '0';

    -- A2 (#3): abs + signs
    signal a2_a, a2_b : unsigned(11 downto 0) := (others => '0');
    signal a2_dxs, a2_dys : std_logic := '0';

    -- A3 (#4): max/min swap
    signal a3_hi, a3_lo : unsigned(11 downto 0) := (others => '0');
    signal a3_swap, a3_dxs, a3_dys : std_logic := '0';

    -- CORDIC (#5..#10): 10 iterations, 2 per stage, guard 5
    constant C_NS : integer := 5;
    constant C_CG : integer := 5;
    type t_cord is array(0 to C_NS) of signed(18 downto 0);
    type t_cang is array(0 to C_NS) of signed(12 downto 0);
    type t_cb   is array(0 to C_NS) of std_logic;
    signal cordx, cordy : t_cord := (others => (others => '0'));
    signal cordz        : t_cang := (others => (others => '0'));
    signal cordsw, corddxs, corddys : t_cb := (others => '0');

    type t_atan is array(0 to 9) of integer;
    constant C_ATAN : t_atan := (512, 302, 160, 81, 41, 20, 10, 5, 3, 1);

    -- RU (#11): unscaled radius (Q2) + folded angle
    signal ru_r   : unsigned(11 downto 0) := (others => '0');
    signal ru_ang : signed(12 downto 0) := (others => '0');

    -- RB (#12): ring split, border test
    signal rb_ring : unsigned(5 downto 0) := (others => '0');
    signal rb_fr   : signed(7 downto 0) := (others => '0');   -- radial frac, Q7
    signal rb_rz   : std_logic := '0';                        -- ring-0 flag
    signal rb_bord : std_logic := '0';
    signal rb_ang  : signed(12 downto 0) := (others => '0');

    -- T1 (#13): theta*ring, ring*twist
    signal t1_m   : signed(19 downto 0) := (others => '0');
    signal t1_tw  : signed(14 downto 0) := (others => '0');
    signal t1_fr  : signed(7 downto 0) := (others => '0');
    signal t1_rz  : std_logic := '0';

    -- T2 (#14): tangential frac
    signal t2_ft  : signed(7 downto 0) := (others => '0');
    signal t2_fr  : signed(7 downto 0) := (others => '0');

    -- D1 (#15): squares
    signal d1_fp2, d1_ft2 : unsigned(14 downto 0) := (others => '0');

    -- D2 (#16): normalized distance^2
    signal d2_d2  : unsigned(15 downto 0) := (others => '0');

    -- C1a (#17): dot key (subtract + clamp only)
    signal c1a_key : unsigned(7 downto 0) := (others => '0');
    -- C1b (#18): lace compose (invert / cut / ghost select)
    signal c1_key : unsigned(7 downto 0) := (others => '0');
    signal c1_gh  : unsigned(7 downto 0) := (others => '0');
    signal c1_useg : std_logic := '0';

    ----------------------------------------------------------------------
    -- luma side pipeline
    ----------------------------------------------------------------------
    type t_ysr is array(0 to 15) of unsigned(9 downto 0);
    signal y_sr : t_ysr := (others => (others => '0'));

    signal lp1_p  : unsigned(16 downto 0) := (others => '0');  -- lv*gain (#14)
    signal lp2_l  : unsigned(7 downto 0) := (others => '0');   -- dot radius Q7ish (#15)
    signal lp3_rsq : unsigned(15 downto 0) := (others => '0'); -- radius^2 (#16)

    signal g1_ink : std_logic := '0';                          -- raw edge flag (#13)
    signal g1_sr  : std_logic_vector(0 to 4) := (others => '0');
    signal g2_ink : std_logic := '0';                          -- dilated (#14)
    signal i1_ink, i2_ink, i3_ink : std_logic := '0';          -- (#15,#16,#17)
    signal gh_v, gh_v2 : unsigned(7 downto 0) := (others => '0'); -- ghost luma (#16,#17)

    ----------------------------------------------------------------------
    -- 1-bit carries
    ----------------------------------------------------------------------
    signal seam_sr : std_logic_vector(0 to 14) := (others => '0'); -- A1 -> C1b
    signal bord_sr : std_logic_vector(0 to 4) := (others => '0');  -- RB -> C1b
    type t_sty is array(0 to 8) of unsigned(1 downto 0);
    signal sty_sr : t_sty := (others => (others => '0'));          -- A1 -> RB

    signal s_out_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_out_u : unsigned(9 downto 0) := C_MID;
    signal s_out_v : unsigned(9 downto 0) := C_MID;

    ----------------------------------------------------------------------
    -- sync delay
    ----------------------------------------------------------------------
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- relay the pixel-rate multiply operands (frame-stable)
    ------------------------------------------------------------------------
    p_relay : process(clk)
    begin
        if rising_edge(clk) then
            s_twist_d <= s_twist;
            s_gain_d  <= s_gain;
        end if;
    end process p_relay;

    ------------------------------------------------------------------------
    -- raster measurement (hypnos pattern)
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_q <= data_in.avid;
            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then                 -- avid just fell
                if s_xcnt > 16 then s_W <= s_xcnt; end if;
                s_xcnt <= (others => '0');
                s_lcnt <= s_lcnt + 1;
            end if;

            if s_vs_pulse = '1' then
                if s_lcnt > 8 then
                    if s_ilace = '1' then s_H <= shift_left(s_lcnt, 1);
                    else                  s_H <= s_lcnt; end if;
                end if;
                s_lcnt <= (others => '0');
            end if;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer.  The vsync trigger is
    -- guarded with "active lines seen" so serrated analog vsync edges can't
    -- re-run the sequencer (and double-step the drift) within one blank.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_d   : signed(10 downto 0);
        variable v_e   : signed(10 downto 0);
        variable v_q6  : signed(10 downto 0);
        variable v_sh  : integer range 0 to 7;
        variable v_s   : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            s_mp <= s_ma * s_mb;

            if data_in.vsync_n = '0' and s_prev_vsync = '1' and s_lcnt > 8 then
                s_vs_pulse <= '1';

                s_k_fill  <= unsigned(registers_in(0));
                s_k_gain  <= unsigned(registers_in(1));
                s_k_drift <= unsigned(registers_in(2));
                s_k_ink   <= unsigned(registers_in(3));
                s_k_bord  <= unsigned(registers_in(4));
                s_k_twist <= unsigned(registers_in(5));
                s_zoom_k  <= unsigned(registers_in(7));
                s_ivory   <= registers_in(6)(0);
                s_inv     <= registers_in(6)(1);
                s_inkon   <= registers_in(6)(2);
                s_weave   <= registers_in(6)(3);
                s_ghost   <= registers_in(6)(4);

                s_fpar <= data_in.field_n;
                if data_in.field_n /= s_fpar then s_ilace <= '1';
                else                              s_ilace <= '0'; end if;

                s_seq <= to_unsigned(22, 5);

            elsif s_seq /= 0 and data_in.avid = '0' then
                s_seq <= s_seq - 1;

                case to_integer(s_seq) is

                    -- SCALE: s = 2^((P12-512)/512) in Q8, one octave each way;
                    -- SD sources get an extra >>1 (bands 128 px, not 256).
                    when 22 =>
                        v_e  := signed('0' & s_zoom_k) - to_signed(512, 11);
                        v_q6 := shift_right(v_e, 3);
                        s_svi <= to_integer(shift_right(v_q6, 6));
                        s_svf <= unsigned(v_q6(5 downto 0));
                    when 21 =>
                        v_sh := 2 - s_svi;
                        if s_W < 1000 then v_sh := v_sh + 1; end if;
                        v_s := shift_right(C_MANT(to_integer(s_svf)), v_sh);
                        s_scl <= v_s(9 downto 0);

                    -- u0 = UOFF - (W/2)*s   (shared multiplier)
                    when 19 => s_ma <= signed(resize(s_W(11 downto 1), 13));
                               s_mb <= signed('0' & s_scl);
                    when 17 => s_u0 <= C_UOFF - unsigned(s_mp(19 downto 0));

                    -- v0 = VOFF + scroll - (H/2)*s
                    when 16 => s_ma <= signed(resize(s_H(11 downto 1), 13));
                               s_mb <= signed('0' & s_scl);
                    when 14 => s_v0 <= C_VOFF + s_scroll - unsigned(s_mp(19 downto 0));

                    -- DRIFT: scroll += (K3-512), deadbanded (Q8 lace/frame)
                    when 12 =>
                        v_d := signed('0' & s_k_drift) - to_signed(512, 11);
                        if abs(v_d) >= C_DB then
                            s_scroll <= s_scroll + unsigned(resize(v_d, 20));
                        end if;

                    -- luma -> dot radius mapping
                    when 11 => s_gain <= s_k_gain(9 downto 1);
                    when 10 => s_bias <= s_k_fill(9 downto 2);

                    -- TWIST: per-ring cell rotation, deadbanded
                    when 9 =>
                        v_d := signed('0' & s_k_twist) - to_signed(512, 11);
                        if abs(v_d) < C_DB then s_twist <= (others => '0');
                        else s_twist <= resize(shift_right(v_d, 2), 8); end if;

                    -- INK threshold (K4 up = more ink)
                    when 8 =>
                        s_inkth <= to_unsigned(8, 8)
                                   + resize(shift_right(to_unsigned(1023, 10) - s_k_ink, 3), 8);

                    -- BORDER weight (Q2) and effective fan edges
                    when 7 => s_bw <= s_k_bord(9 downto 1);
                    when 6 => s_re0 <= to_unsigned(C_REDGE0, 12) - resize(s_bw, 12);
                    when 5 => s_re1 <= to_unsigned(C_REDGE1, 12) - resize(s_bw, 12);
                    when 4 => s_re2 <= to_unsigned(C_REDGE0, 12) - resize(s_bw, 12);
                    when 3 => s_re3 <= to_unsigned(C_REDGE3, 12) - resize(s_bw, 12);

                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- lace-space DDA.  u loads u0 at each active-line start and steps +s per
    -- pixel; v loads v0 at the frame's first active line (+s on the offset
    -- field when interlaced) and steps +s per line (+2s interlaced).
    -- Advances on ACTIVE lines only (interlace screen-phase rule).
    ------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_v0 : unsigned(19 downto 0);
        variable v_st : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            s_avid_p <= data_in.avid;

            if s_vs_pulse = '1' then
                s_newframe <= '1';
            elsif data_in.avid = '1' and s_avid_p = '0' then      -- active-line start
                if s_ilace = '1' then v_st := shift_left(resize(s_scl, 20), 1);
                else                  v_st := resize(s_scl, 20); end if;
                if s_newframe = '1' then
                    if s_ilace = '1' and data_in.field_n = '1' then
                        v_v0 := s_v0 + resize(s_scl, 20);
                    else
                        v_v0 := s_v0;
                    end if;
                    s_vc <= v_v0;
                    s_vb <= v_v0 + v_st;
                    s_newframe <= '0';
                else
                    s_vc <= s_vb;
                    s_vb <= s_vb + v_st;
                end if;
                s_u <= s_u0;
            elsif data_in.avid = '1' then
                s_u <= s_u + resize(s_scl, 20);
            end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- A1: band split + style decode + local centre coordinates (Q2).
    -- Style from band index k = v[17:16]; Weave B drops fans/blooms.
    -- Half-P stagger on even bands = one XOR on the top local-x bit.
    ------------------------------------------------------------------------
    p_a1 : process(clk)
        variable v_k    : unsigned(1 downto 0);
        variable v_sty  : unsigned(1 downto 0);
        variable v_u9   : unsigned(8 downto 0);
        variable v_u8   : unsigned(7 downto 0);
        variable v_ux10 : unsigned(9 downto 0);
        variable v_ux11 : unsigned(10 downto 0);
        variable v_dx   : signed(12 downto 0);
        variable v_dep  : signed(12 downto 0);
        variable v_seam : integer;
        variable v_adx  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            v_k := s_vc(17 downto 16);
            if s_weave = '1' then
                if v_k(0) = '1' then v_sty := "10"; else v_sty := "00"; end if;
            else
                v_sty := v_k;
            end if;

            if v_sty = "11" then                                  -- BLOOM, P=256
                v_u8 := s_u(15 downto 8);
                v_u8(7) := v_u8(7) xor (not v_k(0));
                v_ux10 := v_u8 & s_u(7 downto 6);
                v_dx := signed(resize(v_ux10, 13)) - to_signed(512, 13);
                v_dep := C_DEPTH3;
                v_seam := C_SEAM8;
            else                                                  -- P=512
                v_u9 := s_u(16 downto 8);
                v_u9(8) := v_u9(8) xor (not v_k(0));
                v_ux11 := v_u9 & s_u(7 downto 6);
                v_dx := signed(resize(v_ux11, 13)) - to_signed(1024, 13);
                if v_sty = "01" then v_dep := C_DEPTH1;
                else                 v_dep := C_DEPTH0; end if;
                v_seam := C_SEAM9;
            end if;

            a1_dx  <= v_dx;
            a1_dy  <= signed(resize(s_vc(15 downto 6), 13)) - v_dep;
            a1_sty <= v_sty;

            if v_dx < 0 then v_adx := -v_dx; else v_adx := v_dx; end if;
            if to_integer(v_adx) > v_seam then a1_seam <= '1';
            else                               a1_seam <= '0'; end if;
        end if;
    end process p_a1;

    ------------------------------------------------------------------------
    -- A2: |dx|,|dy| + signs      A3: max/min swap
    ------------------------------------------------------------------------
    -- |dy| can be exactly 2048 (needs all 12 bits): SLICE the negated value,
    -- never resize it (the signed-resize-on-abs trap).
    p_a2 : process(clk)
        variable v_nx, v_ny : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            if a1_dx(12) = '1' then v_nx := -a1_dx; else v_nx := a1_dx; end if;
            if a1_dy(12) = '1' then v_ny := -a1_dy; else v_ny := a1_dy; end if;
            a2_a <= unsigned(v_nx(11 downto 0));
            a2_b <= unsigned(v_ny(11 downto 0));
            a2_dxs <= a1_dx(12);
            a2_dys <= a1_dy(12);
        end if;
    end process p_a2;

    p_a3 : process(clk)
    begin
        if rising_edge(clk) then
            if a2_a >= a2_b then a3_hi <= a2_a; a3_lo <= a2_b; a3_swap <= '0';
            else                 a3_hi <= a2_b; a3_lo <= a2_a; a3_swap <= '1'; end if;
            a3_dxs <= a2_dxs;
            a3_dys <= a2_dys;
        end if;
    end process p_a3;

    ------------------------------------------------------------------------
    -- CORDIC vectoring: 10 iterations, 2 per stage, 5 guard bits.
    ------------------------------------------------------------------------
    p_cordic : process(clk)
        variable x0, y0, x1, y1 : signed(18 downto 0);
        variable z0, z1         : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            cordx(0)   <= shift_left(signed(resize(a3_hi, 19)), C_CG);
            cordy(0)   <= shift_left(signed(resize(a3_lo, 19)), C_CG);
            cordz(0)   <= (others => '0');
            cordsw(0)  <= a3_swap;
            corddxs(0) <= a3_dxs;
            corddys(0) <= a3_dys;
            for s in 0 to C_NS - 1 loop
                if cordy(s) >= 0 then
                    x0 := cordx(s) + shift_right(cordy(s), 2 * s);
                    y0 := cordy(s) - shift_right(cordx(s), 2 * s);
                    z0 := cordz(s) + to_signed(C_ATAN(2 * s), 13);
                else
                    x0 := cordx(s) - shift_right(cordy(s), 2 * s);
                    y0 := cordy(s) + shift_right(cordx(s), 2 * s);
                    z0 := cordz(s) - to_signed(C_ATAN(2 * s), 13);
                end if;
                if y0 >= 0 then
                    x1 := x0 + shift_right(y0, 2 * s + 1);
                    y1 := y0 - shift_right(x0, 2 * s + 1);
                    z1 := z0 + to_signed(C_ATAN(2 * s + 1), 13);
                else
                    x1 := x0 - shift_right(y0, 2 * s + 1);
                    y1 := y0 + shift_right(x0, 2 * s + 1);
                    z1 := z0 - to_signed(C_ATAN(2 * s + 1), 13);
                end if;
                cordx(s + 1)   <= x1;
                cordy(s + 1)   <= y1;
                cordz(s + 1)   <= z1;
                cordsw(s + 1)  <= cordsw(s);
                corddxs(s + 1) <= corddxs(s);
                corddys(s + 1) <= corddys(s);
            end loop;
        end if;
    end process p_cordic;

    ------------------------------------------------------------------------
    -- RU: undo CORDIC gain+guard (r = x*312 >> 14, Q2) and fold the angle
    -- out of the first octant into a full polar angle (4096 = full circle).
    ------------------------------------------------------------------------
    p_ru : process(clk)
        variable vc   : unsigned(18 downto 0);
        variable vp   : unsigned(26 downto 0);
        variable vmag : signed(12 downto 0);
        variable vang : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            vc := unsigned(cordx(C_NS));
            vp := resize(shift_left(resize(vc, 27), 8), 27)
                  + resize(shift_left(resize(vc, 27), 5), 27)
                  + resize(shift_left(resize(vc, 27), 4), 27)
                  + resize(shift_left(resize(vc, 27), 3), 27);
            ru_r <= resize(shift_right(vp, 14), 12);

            if cordsw(C_NS) = '1' then vmag := to_signed(1024, 13) - cordz(C_NS);
            else                       vmag := cordz(C_NS); end if;
            if corddxs(C_NS) = '0' and corddys(C_NS) = '0' then    vang := vmag;
            elsif corddxs(C_NS) = '1' and corddys(C_NS) = '0' then vang := to_signed(2048, 13) - vmag;
            elsif corddxs(C_NS) = '1' and corddys(C_NS) = '1' then vang := to_signed(2048, 13) + vmag;
            else                                                   vang := -vmag; end if;
            ru_ang <= vang;
        end if;
    end process p_ru;

    ------------------------------------------------------------------------
    -- RB: ring split at the style's pitch, radial cell fraction (Q7),
    -- ring-0 centre dot, and the scalloped border test (r > R_edge - K5).
    ------------------------------------------------------------------------
    p_rb : process(clk)
        variable v_sty  : unsigned(1 downto 0);
        variable v_ring : unsigned(5 downto 0);
        variable v_fr8  : unsigned(7 downto 0);
        variable v_re   : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            v_sty := sty_sr(8);

            if v_sty(1) = '1' then                       -- SEED/BLOOM, G=16
                v_ring := ru_r(11 downto 6);
                v_fr8  := ru_r(5 downto 0) & "00";
            else                                         -- PEBBLE/FAN, G=32
                v_ring := '0' & ru_r(11 downto 7);
                v_fr8  := ru_r(6 downto 0) & '0';
            end if;

            rb_ring <= v_ring;
            if v_ring = 0 then
                rb_rz <= '1';
                -- centre dot: distance measured straight from r (always +ve)
                rb_fr <= signed(resize(v_fr8(7 downto 1), 8));
            else
                rb_rz <= '0';
                rb_fr <= signed(v_fr8 - to_unsigned(128, 8));
            end if;

            case v_sty is
                when "00"   => v_re := s_re0;
                when "01"   => v_re := s_re1;
                when "10"   => v_re := s_re2;
                when others => v_re := s_re3;
            end case;
            if ru_r > v_re then rb_bord <= '1'; else rb_bord <= '0'; end if;

            rb_ang <= ru_ang;
        end if;
    end process p_rb;

    ------------------------------------------------------------------------
    -- T1: the two narrow multiplies -- theta*ring and ring*twist
    ------------------------------------------------------------------------
    p_t1 : process(clk)
    begin
        if rising_edge(clk) then
            t1_m  <= rb_ang * signed(resize(rb_ring, 7));
            t1_tw <= signed(resize(rb_ring, 7)) * s_twist_d;
            t1_fr <= rb_fr;
            t1_rz <= rb_rz;
        end if;
    end process p_t1;

    ------------------------------------------------------------------------
    -- T2: cells around the ring = 6*ring, so cell coordinate = theta*ring*6
    -- / 4096.  The mod-4096 slice makes the wrap seamless; twist shifts each
    -- ring's dots by ring*K6 (phyllotaxis shear).
    ------------------------------------------------------------------------
    p_t2 : process(clk)
        variable v_m6 : signed(22 downto 0);
        variable v_mt : signed(22 downto 0);
    begin
        if rising_edge(clk) then
            v_m6 := shift_left(resize(t1_m, 23), 2) + shift_left(resize(t1_m, 23), 1);
            v_mt := v_m6 + shift_left(resize(t1_tw, 23), 5);
            if t1_rz = '1' then
                t2_ft <= (others => '0');
            else
                t2_ft <= signed(unsigned(v_mt(11 downto 4)) - to_unsigned(128, 8));
            end if;
            t2_fr <= t1_fr;
        end if;
    end process p_t2;

    ------------------------------------------------------------------------
    -- D1/D2: normalized distance^2 = fr^2 + ft^2  (Q14, 16384 = half-cell^2)
    ------------------------------------------------------------------------
    p_d1 : process(clk)
        variable vp, vq : signed(15 downto 0);
    begin
        if rising_edge(clk) then
            vp := t2_fr * t2_fr;
            vq := t2_ft * t2_ft;
            d1_fp2 <= unsigned(vp(14 downto 0));
            d1_ft2 <= unsigned(vq(14 downto 0));
        end if;
    end process p_d1;

    p_d2 : process(clk)
    begin
        if rising_edge(clk) then
            d2_d2 <= resize(d1_fp2, 16) + resize(d1_ft2, 16);
        end if;
    end process p_d2;

    ------------------------------------------------------------------------
    -- luma side: delayed input luma -> dot radius -> radius^2, plus the
    -- gradient INK strokes and the ghost tap.  All aligned to land at C1.
    ------------------------------------------------------------------------
    p_luma : process(clk)
        variable v_l  : unsigned(16 downto 0);
        variable v_li : integer;
        variable v_ga, v_gb : unsigned(7 downto 0);
        variable v_gd : integer;
    begin
        if rising_edge(clk) then
            y_sr(0) <= unsigned(data_in.y);
            for i in 1 to 15 loop
                y_sr(i) <= y_sr(i - 1);
            end loop;

            -- dot radius: l = clamp(bias + lv*gain/256, 0, 250)
            lp1_p <= y_sr(12)(9 downto 2) * s_gain_d;
            v_li  := to_integer(s_bias) + to_integer(lp1_p(16 downto 8));
            if v_li > 250 then v_li := 250; end if;
            lp2_l <= to_unsigned(v_li, 8);
            lp3_rsq <= lp2_l * lp2_l;

            -- ink: horizontal luma gradient over a 4-px baseline (taps chosen
            -- so the dilated stroke lands centred on the key pixel at C1)
            v_ga := y_sr(8)(9 downto 2);
            v_gb := y_sr(12)(9 downto 2);
            v_gd := to_integer(v_ga) - to_integer(v_gb);
            if v_gd < 0 then v_gd := -v_gd; end if;
            if v_gd > to_integer(s_inkth) then g1_ink <= '1';
            else                               g1_ink <= '0'; end if;
            g1_sr <= g1_ink & g1_sr(0 to 3);
            g2_ink <= g1_ink or g1_sr(0) or g1_sr(1) or g1_sr(2) or g1_sr(3) or g1_sr(4);
            i1_ink <= g2_ink;
            i2_ink <= i1_ink;
            i3_ink <= i2_ink;

            -- ghost: dim video for the ground
            gh_v  <= y_sr(14)(9 downto 2);
            gh_v2 <= gh_v;
        end if;
    end process p_luma;

    ------------------------------------------------------------------------
    -- 1-bit / 2-bit carries
    ------------------------------------------------------------------------
    p_carry : process(clk)
    begin
        if rising_edge(clk) then
            seam_sr(0) <= a1_seam;
            for i in 1 to 14 loop
                seam_sr(i) <= seam_sr(i - 1);
            end loop;
            bord_sr(0) <= rb_bord;
            for i in 1 to 4 loop
                bord_sr(i) <= bord_sr(i - 1);
            end loop;
            sty_sr(0) <= a1_sty;
            for i in 1 to 8 loop
                sty_sr(i) <= sty_sr(i - 1);
            end loop;
        end if;
    end process p_carry;

    ------------------------------------------------------------------------
    -- C1a: dot key with a soft edge (subtract + clamp ONLY -- keeping the
    -- compose muxes out of this cone was the hd_hdmi critical path).
    ------------------------------------------------------------------------
    p_c1a : process(clk)
        variable v_df : signed(17 downto 0);
        variable v_k  : integer;
    begin
        if rising_edge(clk) then
            v_df := signed(resize(lp3_rsq, 18)) - signed(resize(d2_d2, 18));
            v_k  := 128 + to_integer(shift_right(v_df, 3));
            if v_k < 0 then v_k := 0; end if;
            if v_k > 255 then v_k := 255; end if;
            c1a_key <= to_unsigned(v_k, 8);
        end if;
    end process p_c1a;

    ------------------------------------------------------------------------
    -- C1b: lace composition -- invert the figure/ground if asked, then cut
    -- every black line (border scallops, seam wisps, ink strokes) on top.
    ------------------------------------------------------------------------
    p_c1b : process(clk)
        variable v_k   : unsigned(7 downto 0);
        variable v_cut : std_logic;
    begin
        if rising_edge(clk) then
            v_k := c1a_key;
            if s_inv = '1' then v_k := not v_k; end if;

            v_cut := bord_sr(4) or seam_sr(14);
            if s_inkon = '1' then v_cut := v_cut or i3_ink; end if;
            if v_cut = '1' then v_k := (others => '0'); end if;

            c1_key <= v_k;
            c1_gh  <= gh_v2;
            if s_ghost = '1' and s_inv = '0' and v_cut = '1' then c1_useg <= '1';
            elsif s_ghost = '1' and s_inv = '0' and c1a_key = 0 then c1_useg <= '1';
            else c1_useg <= '0'; end if;
        end if;
    end process p_c1b;

    ------------------------------------------------------------------------
    -- C2: colour map (cream on black) + blanking gate
    ------------------------------------------------------------------------
    p_c2 : process(clk)
        variable v_y : unsigned(10 downto 0);   -- 11-bit: 64+1020 overflows 10
    begin
        if rising_edge(clk) then
            if s_avid_sr(C_LATENCY - 2) = '0' then
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            elsif c1_useg = '1' then
                s_out_y <= to_unsigned(64, 10) + resize(c1_gh(7 downto 1), 10);
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            else
                -- Y = 64 + key*3.5 -> 64..957
                v_y := to_unsigned(64, 11)
                       + shift_left(resize(c1_key, 11), 2)
                       - resize(c1_key(7 downto 1), 11);
                s_out_y <= v_y(9 downto 0);
                if s_ivory = '1' then
                    s_out_u <= C_MID - resize(c1_key(7 downto 3), 10);
                    s_out_v <= C_MID + resize(c1_key(7 downto 4), 10);
                else
                    s_out_u <= C_MID;
                    s_out_v <= C_MID;
                end if;
            end if;
        end if;
    end process p_c2;

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

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture chantilly;
