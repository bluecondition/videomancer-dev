-- ribbon.vhd
--
-- RIBBON -- full-screen candy-ribbon stripes.  A standalone fork of the
-- "rainbow taffy" texture from SUGARCOAT, blown up to fill the frame and
-- grown into a full instrument.  No video in -- a pure generator.
--
--   P12 WARP     bend depth, 0..63 px          CONTINUOUS
--   K1  ANGLE    stripe rotation, 0..180 deg   CONTINUOUS (128 steps)
--   K2  DENSITY  pitch octave, 4 steps
--   K3  SCALE    ribbon size, 16:1 zoom        CONTINUOUS
--   K4  PALETTE  crossfade through 7 ramps     CONTINUOUS (wraps)
--   K5  RIPPLE   warp wavelength               CONTINUOUS
--   K6  GLOSS    rounding + specular + emboss
--   S7  CHEVRON  mirror the field about screen centre -> arrowheads
--   S8  PLAID    cross-weave a second ribbon set at 90 deg
--   S9  BAKERY   force the brown Bakery ramp
--   S10 ROPE     mirror the palette ramp -> symmetric twisted ropes
--   S11 OUTLINE  dark seam between stops -> individually wrapped candies
--
-- EVERYTHING RIDES ACCUMULATORS.  The stripe projection is not computed from
-- (dx,dy) at all: A advances by `ca` per pixel and by `sa` per active line,
-- so a continuous rotation costs ZERO pixel-path multiplies (see
-- [[videosky]]'s accumulator-pair trick).  The perpendicular weave B and both
-- ripple phases are three more accumulators.  A is in 1/256 px, which is what
-- makes the warp and the zoom smooth rather than stepped.  The accumulators
-- are seeded at 0 each frame rather than at screen centre -- the field is
-- periodic, so the origin only sets an invisible phase.
--
-- Multiplies (HX4K has no DSP): ONE in the pixel path (the warp, 8x11) plus a
-- tiny 6x9 for the ribbon rounding.  Everything else -- the palette crossfade,
-- the sin/cos, the zoom -- is done ONCE PER FRAME in the vblank sequencer and
-- handed to the pixel path as constants or as an 8-entry palette register
-- file.  Per-frame logic is still timed at the pixel clock, so every derived
-- term gets its own sequencer cycle.
--
-- Pipeline (streaming, C_LATENCY = 15, one line RAM):
--   A      accumulator snapshot (A, B, ripple phases, x)
--   W1..W3 triangle bend -> one multiply -> added to both projections
--   Q      one barrel shift yields stop index AND sub-stop position
--   ST     rope fold + outline decision
--   R1/R2  palette register file x2 -> plaid select -> outline darken
--   S1/S2  ribbon rounding + specular line from the sub-stop position
--   L0..L4 line RAM -> gx/gy -> lit -> shift -> spec add + clamp
--   O      colour + shade + sheen, clamped, U/V swapped for hardware
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture ribbon of program_top is

    constant C_LATENCY : integer := 15;

    ----------------------------------------------------------------------
    -- helpers
    ----------------------------------------------------------------------
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

    -- Folded TRIANGLE wave (10-bit phase, 1024 = one cycle), returned as a
    -- SIGNED INDEX (-63..63) rather than SUGARCOAT's scaled +/-504.  Exactly
    -- linear inside each quarter, so the bend is a true triangle with no
    -- staircase -- and the 7-bit operand keeps the warp multiplier a third
    -- the size of a 10-bit one.  tri504 = tri7 * 8.
    function f_tri7(a : unsigned(9 downto 0)) return signed is
        variable v_i : unsigned(5 downto 0);
        variable v_t : signed(6 downto 0);
    begin
        if a(8) = '0' then v_i := a(7 downto 2);
        else               v_i := not a(7 downto 2); end if;
        v_t := signed(resize(v_i, 7));                -- 0..63
        if a(9) = '1' then return -v_t; else return v_t; end if;
    end function;

    ----------------------------------------------------------------------
    -- quarter cosine table: C_COS(k) = round(362 * cos(k * 90/64 deg)).
    -- 362 = 256*sqrt(2), so at 45 deg ca = sa = 256 exactly -- i.e. the
    -- accumulator matches a plain (dx+dy) diagonal at unity zoom.
    ----------------------------------------------------------------------
    type t_cos65 is array (0 to 64) of signed(10 downto 0);
    constant C_COS : t_cos65 := (
        to_signed(362, 11), to_signed(362, 11), to_signed(362, 11), to_signed(361, 11), to_signed(360, 11), to_signed(359, 11), to_signed(358, 11), to_signed(357, 11),   -- 0
        to_signed(355, 11), to_signed(353, 11), to_signed(351, 11), to_signed(349, 11), to_signed(346, 11), to_signed(344, 11), to_signed(341, 11), to_signed(338, 11),   -- 8
        to_signed(334, 11), to_signed(331, 11), to_signed(327, 11), to_signed(323, 11), to_signed(319, 11), to_signed(315, 11), to_signed(310, 11), to_signed(306, 11),   -- 16
        to_signed(301, 11), to_signed(296, 11), to_signed(291, 11), to_signed(285, 11), to_signed(280, 11), to_signed(274, 11), to_signed(268, 11), to_signed(262, 11),   -- 24
        to_signed(256, 11), to_signed(250, 11), to_signed(243, 11), to_signed(236, 11), to_signed(230, 11), to_signed(223, 11), to_signed(216, 11), to_signed(208, 11),   -- 32
        to_signed(201, 11), to_signed(194, 11), to_signed(186, 11), to_signed(178, 11), to_signed(171, 11), to_signed(163, 11), to_signed(155, 11), to_signed(147, 11),   -- 40
        to_signed(139, 11), to_signed(130, 11), to_signed(122, 11), to_signed(114, 11), to_signed(105, 11), to_signed(97, 11), to_signed(88, 11), to_signed(79, 11),   -- 48
        to_signed(71, 11), to_signed(62, 11), to_signed(53, 11), to_signed(44, 11), to_signed(35, 11), to_signed(27, 11), to_signed(18, 11), to_signed(9, 11),   -- 56
        to_signed(0, 11)   -- 64
    );

    -- Full-circle cosine from the quarter table.  i is a 256-step angle
    -- (1.40625 deg/step).  Folded to ONE table read (four separate reads
    -- would instance the ROM four times).
    function f_cos(i : unsigned(7 downto 0)) return signed is
        variable k : integer range 0 to 64;
        variable v : signed(10 downto 0);
    begin
        if i(6) = '0' then k := to_integer(i(5 downto 0));
        else               k := 64 - to_integer(i(5 downto 0)); end if;
        v := C_COS(k);
        if (i(7) xor i(6)) = '1' then return -v; else return v; end if;
    end function;

    ----------------------------------------------------------------------
    -- candy palette ROM (from sugarcoat/genpal.py) -- flat 56-entry ROMs,
    -- index = pal*8 + stop, 10-bit standard BT.601 YUV, chroma centred 512.
    -- Read only by the vblank crossfade engine, never in the pixel path.
    ----------------------------------------------------------------------
    type t_pal56 is array (0 to 55) of unsigned(9 downto 0);
    constant C_PAL_Y : t_pal56 := (
        to_unsigned(53, 10), to_unsigned(201, 10), to_unsigned(329, 10), to_unsigned(588, 10), to_unsigned(800, 10), to_unsigned(652, 10), to_unsigned(275, 10), to_unsigned(1023, 10),  -- 0: Penny Candy
        to_unsigned(144, 10), to_unsigned(324, 10), to_unsigned(598, 10), to_unsigned(751, 10), to_unsigned(782, 10), to_unsigned(732, 10), to_unsigned(792, 10), to_unsigned(1009, 10),  -- 8: Bubblegum
        to_unsigned(104, 10), to_unsigned(242, 10), to_unsigned(381, 10), to_unsigned(305, 10), to_unsigned(494, 10), to_unsigned(774, 10), to_unsigned(981, 10), to_unsigned(1023, 10),  -- 16: Peppermint
        to_unsigned(88, 10), to_unsigned(182, 10), to_unsigned(329, 10), to_unsigned(488, 10), to_unsigned(638, 10), to_unsigned(778, 10), to_unsigned(901, 10), to_unsigned(993, 10),  -- 24: Bakery
        to_unsigned(123, 10), to_unsigned(624, 10), to_unsigned(840, 10), to_unsigned(880, 10), to_unsigned(444, 10), to_unsigned(643, 10), to_unsigned(871, 10), to_unsigned(1023, 10),  -- 32: Sour
        to_unsigned(109, 10), to_unsigned(606, 10), to_unsigned(734, 10), to_unsigned(868, 10), to_unsigned(606, 10), to_unsigned(700, 10), to_unsigned(692, 10), to_unsigned(1007, 10),  -- 40: Tropical Taffy
        to_unsigned(245, 10), to_unsigned(554, 10), to_unsigned(678, 10), to_unsigned(829, 10), to_unsigned(875, 10), to_unsigned(786, 10), to_unsigned(854, 10), to_unsigned(1023, 10)  -- 48: Holo Wrapper
    );
    constant C_PAL_U : t_pal56 := (
        to_unsigned(505, 10), to_unsigned(444, 10), to_unsigned(417, 10), to_unsigned(181, 10), to_unsigned(62, 10), to_unsigned(213, 10), to_unsigned(718, 10), to_unsigned(512, 10),  -- 0: Penny Candy
        to_unsigned(555, 10), to_unsigned(578, 10), to_unsigned(571, 10), to_unsigned(541, 10), to_unsigned(546, 10), to_unsigned(676, 10), to_unsigned(642, 10), to_unsigned(509, 10),  -- 8: Bubblegum
        to_unsigned(499, 10), to_unsigned(466, 10), to_unsigned(433, 10), to_unsigned(431, 10), to_unsigned(437, 10), to_unsigned(212, 10), to_unsigned(502, 10), to_unsigned(512, 10),  -- 16: Peppermint
        to_unsigned(481, 10), to_unsigned(450, 10), to_unsigned(404, 10), to_unsigned(362, 10), to_unsigned(345, 10), to_unsigned(345, 10), to_unsigned(389, 10), to_unsigned(461, 10),  -- 24: Bakery
        to_unsigned(465, 10), to_unsigned(206, 10), to_unsigned(39, 10), to_unsigned(62, 10), to_unsigned(713, 10), to_unsigned(647, 10), to_unsigned(293, 10), to_unsigned(512, 10),  -- 32: Sour
        to_unsigned(519, 10), to_unsigned(374, 10), to_unsigned(235, 10), to_unsigned(182, 10), to_unsigned(600, 10), to_unsigned(603, 10), to_unsigned(518, 10), to_unsigned(499, 10),  -- 40: Tropical Taffy
        to_unsigned(532, 10), to_unsigned(652, 10), to_unsigned(672, 10), to_unsigned(519, 10), to_unsigned(359, 10), to_unsigned(521, 10), to_unsigned(562, 10), to_unsigned(512, 10)  -- 48: Holo Wrapper
    );
    constant C_PAL_V : t_pal56 := (
        to_unsigned(589, 10), to_unsigned(769, 10), to_unsigned(905, 10), to_unsigned(821, 10), to_unsigned(670, 10), to_unsigned(391, 10), to_unsigned(573, 10), to_unsigned(512, 10),  -- 0: Penny Candy
        to_unsigned(581, 10), to_unsigned(709, 10), to_unsigned(814, 10), to_unsigned(706, 10), to_unsigned(327, 10), to_unsigned(334, 10), to_unsigned(548, 10), to_unsigned(522, 10),  -- 8: Bubblegum
        to_unsigned(495, 10), to_unsigned(368, 10), to_unsigned(298, 10), to_unsigned(865, 10), to_unsigned(845, 10), to_unsigned(618, 10), to_unsigned(542, 10), to_unsigned(512, 10),  -- 16: Peppermint
        to_unsigned(549, 10), to_unsigned(583, 10), to_unsigned(620, 10), to_unsigned(650, 10), to_unsigned(643, 10), to_unsigned(615, 10), to_unsigned(570, 10), to_unsigned(533, 10),  -- 24: Bakery
        to_unsigned(482, 10), to_unsigned(411, 10), to_unsigned(485, 10), to_unsigned(614, 10), to_unsigned(923, 10), to_unsigned(112, 10), to_unsigned(406, 10), to_unsigned(512, 10),  -- 32: Sour
        to_unsigned(549, 10), to_unsigned(809, 10), to_unsigned(718, 10), to_unsigned(622, 10), to_unsigned(195, 10), to_unsigned(185, 10), to_unsigned(747, 10), to_unsigned(524, 10),  -- 40: Tropical Taffy
        to_unsigned(509, 10), to_unsigned(546, 10), to_unsigned(372, 10), to_unsigned(322, 10), to_unsigned(575, 10), to_unsigned(652, 10), to_unsigned(504, 10), to_unsigned(512, 10)  -- 48: Holo Wrapper
    );

    ----------------------------------------------------------------------
    -- frame-latched controls + per-frame terms
    ----------------------------------------------------------------------
    signal s_k1 : unsigned(9 downto 0) := to_unsigned(256, 10);   -- Angle
    signal s_k2 : unsigned(9 downto 0) := to_unsigned(512, 10);   -- Density
    signal s_k3 : unsigned(9 downto 0) := to_unsigned(512, 10);   -- Scale
    signal s_k4 : unsigned(9 downto 0) := (others => '0');        -- Palette
    signal s_k5 : unsigned(9 downto 0) := to_unsigned(250, 10);   -- Ripple
    signal s_k6 : unsigned(9 downto 0) := to_unsigned(640, 10);   -- Gloss
    signal s_p12 : unsigned(9 downto 0) := to_unsigned(320, 10);  -- Warp

    signal s_chev  : std_logic := '0';   -- S7
    signal s_plaid : std_logic := '0';   -- S8
    signal s_bakery: std_logic := '0';   -- S9
    signal s_rope  : std_logic := '0';   -- S10
    signal s_outl  : std_logic := '0';   -- S11

    -- raw quarter-table reads, then zoom-scaled step sizes
    signal s_c0, s_s0 : signed(10 downto 0) := (others => '0');
    signal s_zoom : unsigned(9 downto 0) := to_unsigned(256, 10);   -- 32..512
    signal s_ca : signed(10 downto 0) := to_signed(256, 11);        -- x step
    signal s_sa : signed(10 downto 0) := to_signed(256, 11);        -- y step
    -- pitch bit of the 1/256-px accumulator (DENSITY octave only; SCALE is
    -- continuous and lives in s_zoom).  12..15 -> the barrel shift is 7..10.
    signal s_psh : integer range 12 to 15 := 13;
    signal s_wamp  : signed(10 downto 0) := to_signed(160, 11);     -- WARP depth
    signal s_wstep : unsigned(14 downto 0) := to_unsigned(4128, 15);-- RIPPLE step
    signal s_gsh   : integer range 0 to 5 := 5;                     -- emboss shift
    signal s_gspec : unsigned(8 downto 0) := to_unsigned(220, 9);   -- emboss spec thr
    signal s_gdif  : unsigned(7 downto 0) := to_unsigned(160, 8);   -- ribbon rounding
    signal s_gspc  : unsigned(9 downto 0) := to_unsigned(320, 10);  -- specular line

    -- blended palette register file (K4 crossfade result), read by the pixel
    -- path as a plain 8:1 mux -- no ROM, no multiply, no ramp maths per pixel.
    type t_pal8 is array (0 to 7) of unsigned(9 downto 0);
    -- init chroma to neutral: an all-zero chroma RAM renders a saturated
    -- green wall, not black, if anything ever samples it pre-fill.
    signal s_ply : t_pal8 := (others => (others => '0'));
    signal s_plu : t_pal8 := (others => to_unsigned(512, 10));
    signal s_plv : t_pal8 := (others => to_unsigned(512, 10));
    signal s_offa, s_offb : unsigned(5 downto 0) := (others => '0');
    signal s_pmix : unsigned(5 downto 0) := (others => '0');   -- crossfade t, 0..63
    -- crossfade engine working registers
    signal b_ay, b_au, b_av : unsigned(9 downto 0) := (others => '0');
    signal b_by, b_bu, b_bv : unsigned(9 downto 0) := (others => '0');
    signal b_dy, b_du, b_dv : signed(10 downto 0) := (others => '0');
    signal b_py, b_pu, b_pv : signed(17 downto 0) := (others => '0');
    signal s_bi  : unsigned(2 downto 0) := (others => '0');
    signal s_bph : unsigned(2 downto 0) := (others => '0');
    signal s_bdone : std_logic := '0';
    -- final blended value + PRE-DECODED one-hot write enable.  The hop into
    -- the 240-flop palette file must be flop -> flop with no logic: that file
    -- places scattered, and driving it through an add+clamp AND a decoded
    -- enable was 10.6 ns of pure ROUTING on the critical path.
    signal b_wy, b_wu, b_wv : unsigned(9 downto 0) := (others => '0');
    signal b_we : std_logic_vector(7 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- raster measurement
    ----------------------------------------------------------------------
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse   : std_logic := '0';
    signal s_avid_q     : std_logic := '0';
    signal s_xcnt       : unsigned(11 downto 0) := (others => '0');
    signal s_W          : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_cxu        : unsigned(11 downto 0) := to_unsigned(360, 12);
    signal s_ilace      : std_logic := '0';
    signal s_fpar       : std_logic := '0';

    signal s_seq : unsigned(3 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- projection + ripple accumulators (no multiplies anywhere here)
    ----------------------------------------------------------------------
    signal s_avid_p   : std_logic := '0';
    signal s_newframe : std_logic := '1';
    signal s_arow, s_apx : signed(22 downto 0) := (others => '0');  -- ribbons
    signal s_brow, s_bpx : signed(22 downto 0) := (others => '0');  -- cross weave
    signal s_wphx, s_wphy : unsigned(19 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- pattern pipeline
    ----------------------------------------------------------------------
    signal a_ap, a_bp : signed(22 downto 0) := (others => '0');
    signal a_wx, a_wy : unsigned(9 downto 0) := (others => '0');
    signal a_x        : unsigned(10 downto 0) := (others => '0');
    signal w1_ts      : signed(7 downto 0) := (others => '0');
    signal w1_ap, w1_bp : signed(22 downto 0) := (others => '0');
    signal w1_x       : unsigned(10 downto 0) := (others => '0');
    signal w2_ow      : signed(16 downto 0) := (others => '0');
    signal w2_ap, w2_bp : signed(22 downto 0) := (others => '0');
    signal w2_x       : unsigned(10 downto 0) := (others => '0');
    signal w3_ap, w3_bp : signed(22 downto 0) := (others => '0');
    signal w3_x       : unsigned(10 downto 0) := (others => '0');
    signal q_a, q_b   : unsigned(8 downto 0) := (others => '0');
    signal q_x        : unsigned(10 downto 0) := (others => '0');
    signal st_sa, st_sb   : unsigned(2 downto 0) := (others => '0');
    signal st_ua, st_ub   : unsigned(4 downto 0) := (others => '0');
    signal st_ol      : std_logic := '0';
    signal st_x       : unsigned(10 downto 0) := (others => '0');
    signal ra_y, ra_u, ra_v : unsigned(9 downto 0) := (others => '0');
    signal rb_y, rb_u, rb_v : unsigned(9 downto 0) := (others => '0');
    signal r1_ol      : std_logic := '0';
    signal r1_ua, r1_ub : unsigned(4 downto 0) := (others => '0');
    signal r1_x       : unsigned(10 downto 0) := (others => '0');
    signal r_y, r_u, r_v : unsigned(9 downto 0) := (others => '0');
    signal r_sub      : unsigned(4 downto 0) := (others => '0');
    signal r_x        : unsigned(10 downto 0) := (others => '0');

    -- ribbon rounding / specular (from the sub-stop position)
    signal sh_d, sh_s : signed(10 downto 0) := (others => '0');
    signal sh        : signed(11 downto 0) := (others => '0');

    -- ribbon colour delayed to meet the sheen; the shade folds in at index 2
    type t_csr is array (0 to 4) of unsigned(9 downto 0);
    signal cy_sr, cu_sr, cv_sr : t_csr := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- GLOSS emboss: the ribbons' own (flat) luma is the heightfield, so
    -- every fold edge picks up a lit / shadowed seam.  The ROUNDING above
    -- is what gives the ribbon its body; this puts the crease on it.
    ----------------------------------------------------------------------
    type t_lram is array (0 to 2047) of std_logic_vector(9 downto 0);
    signal lram   : t_lram;
    signal lb_rd  : std_logic_vector(9 downto 0) := (others => '0');
    signal gpl, gpl_d : unsigned(9 downto 0) := (others => '0');
    signal ggx, ggy : signed(10 downto 0) := (others => '0');
    signal glit     : signed(11 downto 0) := (others => '0');
    signal spec_hi  : std_logic := '0';
    signal spec_hi2 : std_logic := '0';
    signal g_sh     : signed(13 downto 0) := (others => '0');
    signal g_add    : signed(10 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- output
    ----------------------------------------------------------------------
    signal o_y, o_u, o_v : unsigned(9 downto 0) := (others => '0');

    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- raster measurement (interlace-aware).  Only the WIDTH is needed now --
    -- the accumulators are seeded at 0, not at screen centre -- but CHEVRON
    -- mirrors about the horizontal midpoint, so keep s_cxu.
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            if data_in.vsync_n = '0' and s_prev_vsync = '1' then
                s_vs_pulse <= '1';
            end if;

            s_avid_q <= data_in.avid;
            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then
                if s_xcnt > 16 then s_W <= s_xcnt; end if;
                s_xcnt <= (others => '0');
            end if;

            if s_vs_pulse = '1' then
                s_fpar <= data_in.field_n;
                if data_in.field_n /= s_fpar then s_ilace <= '1';
                else                              s_ilace <= '0'; end if;
            end if;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer, then the K4 palette
    -- CROSSFADE engine which fills the 8-entry palette register file the
    -- pixel path reads.  Both run only while avid = '0'.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_idx : unsigned(7 downto 0);
        variable v_z   : integer;
        variable v_c   : integer;
        variable v_p7  : unsigned(12 downto 0);
        variable v_q   : unsigned(8 downto 0);
        variable v_pa  : integer range 0 to 6;
        variable v_pb  : integer range 0 to 6;
        variable v_i   : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            if s_vs_pulse = '1' then
                s_k1  <= unsigned(registers_in(0));
                s_k2  <= unsigned(registers_in(1));
                s_k3  <= unsigned(registers_in(2));
                s_k4  <= unsigned(registers_in(3));
                s_k5  <= unsigned(registers_in(4));
                s_k6  <= unsigned(registers_in(5));
                s_p12 <= unsigned(registers_in(7));
                s_chev   <= registers_in(6)(0);        -- S7
                s_plaid  <= registers_in(6)(1);        -- S8
                s_bakery <= registers_in(6)(2);        -- S9
                s_rope   <= registers_in(6)(3);        -- S10
                s_outl   <= registers_in(6)(4);        -- S11
                s_cxu <= '0' & s_W(11 downto 1);
                s_bi   <= (others => '0');
                s_bph  <= (others => '0');
                s_bdone <= '0';
                b_we    <= (others => '0');
                s_seq  <= to_unsigned(8, 4);
            elsif data_in.avid = '0' then
                if s_seq /= 0 then
                    -- ONE derived term per cycle: per-frame logic is timed at
                    -- the pixel clock, so a fat cone here caps Fmax exactly
                    -- like a pixel-path one.
                    case to_integer(s_seq) is
                        when 8 =>
                            -- ANGLE (K1): 128 steps over 0..180 deg.  Both
                            -- quarter-table reads happen here; the zoom
                            -- multiply is a SEPARATE cycle.
                            v_idx := '0' & s_k1(9 downto 3);
                            s_c0 <= f_cos(v_idx);
                            s_s0 <= f_cos(v_idx + 192);      -- sin = cos(-90)
                        when 7 =>
                            -- SCALE (K3): CONTINUOUS 16:1 zoom.  Bigger zoom =
                            -- bigger accumulator step = finer ribbons, so the
                            -- knob is inverted; K3 = 512 gives exactly 256 =
                            -- unity (the original SUGARCOAT pitch).
                            v_z := 512 - to_integer(s_k3(9 downto 1));
                            if v_z < 32 then v_z := 32; end if;
                            s_zoom <= to_unsigned(v_z, 10);
                        when 6 =>
                            s_ca <= resize(shift_right(s_c0 * signed(resize(s_zoom, 11)), 8), 11);
                        when 5 =>
                            s_sa <= resize(shift_right(s_s0 * signed(resize(s_zoom, 11)), 8), 11);
                        when 4 =>
                            -- DENSITY (K2): pitch OCTAVE only (4 steps).
                            s_psh  <= (6 - to_integer(s_k2(9 downto 8))) + 9;
                            -- WARP (P12): 0..512 -> 0..63 px, continuous.
                            s_wamp <= signed(shift_right(resize(s_p12, 11) + 1, 1));
                        when 3 =>
                            -- RIPPLE (K5): warp wavelength.  step 128..16496
                            -- of a 2^20 phase = ~8000 px down to ~64 px.
                            s_wstep <= resize(shift_left(resize(s_k5, 15), 4) + 128, 15);
                            v_c := to_integer(s_k6(9 downto 7));
                            if v_c > 5 then v_c := 5; end if;
                            s_gsh   <= v_c;
                            s_gspec <= to_unsigned(300, 9) - resize(s_k6(9 downto 3), 9);
                        when 2 =>
                            -- GLOSS body: how hard each ribbon is rounded and
                            -- how hot the specular line running along it is.
                            s_gdif <= s_k6(9 downto 2);
                            s_gspc <= '0' & s_k6(9 downto 1);
                        when others =>
                            -- PALETTE (K4): position through the 7 ramps, and
                            -- the crossfade weight to the NEXT one (wraps 6->0
                            -- so the knob is a continuous loop).
                            -- NB: unsigned*natural returns DOUBLE width, so
                            -- resize explicitly rather than assuming 13 bits.
                            v_p7 := resize(s_k4 * 7, 13);              -- 0..7161
                            v_q  := resize(shift_right(v_p7, 4), 9);   -- 0..447
                            v_pa := to_integer(v_q(8 downto 6));
                            if v_pa >= 6 then v_pb := 0; else v_pb := v_pa + 1; end if;
                            if s_bakery = '1' then
                                s_offa <= to_unsigned(24, 6);          -- Bakery
                                s_offb <= to_unsigned(24, 6);
                                s_pmix <= (others => '0');
                            else
                                s_offa <= to_unsigned(v_pa * 8, 6);
                                s_offb <= to_unsigned(v_pb * 8, 6);
                                s_pmix <= v_q(5 downto 0);
                            end if;
                    end case;
                    s_seq <= s_seq - 1;
                elsif s_bdone = '0' then
                    -- PALETTE CROSSFADE, 5 cycles per stop.  Every step is
                    -- deliberately thin: fetch, SUBTRACT (its own cycle -- with
                    -- it feeding the multiplier's partial products this was the
                    -- design's critical path at 13.6 ns), MULTIPLY, form the
                    -- result + one-hot enable, then a bare flop-to-flop write.
                    v_i := to_integer(s_bi);
                    case to_integer(s_bph) is
                        when 0 =>
                            b_ay <= C_PAL_Y(to_integer(s_offa) + v_i);
                            b_au <= C_PAL_U(to_integer(s_offa) + v_i);
                            b_av <= C_PAL_V(to_integer(s_offa) + v_i);
                            b_by <= C_PAL_Y(to_integer(s_offb) + v_i);
                            b_bu <= C_PAL_U(to_integer(s_offb) + v_i);
                            b_bv <= C_PAL_V(to_integer(s_offb) + v_i);
                        when 1 =>
                            b_dy <= signed(resize(b_by, 11)) - signed(resize(b_ay, 11));
                            b_du <= signed(resize(b_bu, 11)) - signed(resize(b_au, 11));
                            b_dv <= signed(resize(b_bv, 11)) - signed(resize(b_av, 11));
                        when 2 =>
                            b_py <= b_dy * signed(resize(s_pmix, 7));
                            b_pu <= b_du * signed(resize(s_pmix, 7));
                            b_pv <= b_dv * signed(resize(s_pmix, 7));
                        when 3 =>
                            b_wy <= f_cu10(signed(resize(b_ay, 13))
                                           + resize(shift_right(b_py, 6), 13));
                            b_wu <= f_cu10(signed(resize(b_au, 13))
                                           + resize(shift_right(b_pu, 6), 13));
                            b_wv <= f_cu10(signed(resize(b_av, 13))
                                           + resize(shift_right(b_pv, 6), 13));
                            b_we <= (others => '0');
                            b_we(v_i) <= '1';
                        when others =>
                            -- the only cycle that touches the palette file
                            for k in 0 to 7 loop
                                if b_we(k) = '1' then
                                    s_ply(k) <= b_wy;
                                    s_plu(k) <= b_wu;
                                    s_plv(k) <= b_wv;
                                end if;
                            end loop;
                            b_we <= (others => '0');
                    end case;
                    if s_bph = 4 then
                        s_bph <= (others => '0');
                        if s_bi = 7 then s_bdone <= '1';
                        else             s_bi <= s_bi + 1; end if;
                    else
                        s_bph <= s_bph + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- ACCUMULATORS.  A = the ribbon projection (rotates with K1), B = the
    -- perpendicular weave, wphx/wphy = the two ripple phases.  All four are
    -- pure adders: no multiply is needed for rotation, zoom or ripple.
    -- Advanced on ACTIVE lines only (interlace-safe -- see the DDA/field
    -- phase trap), and mirrored past screen centre when CHEVRON is on.
    ------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_ar, v_br : signed(22 downto 0);
        variable v_wr : unsigned(19 downto 0);
    begin
        if rising_edge(clk) then
            s_avid_p <= data_in.avid;
            if s_vs_pulse = '1' then
                s_newframe <= '1';
            elsif data_in.avid = '1' and s_avid_p = '0' then
                if s_newframe = '1' then
                    -- an interlaced odd field starts one line down
                    if s_ilace = '1' and data_in.field_n = '1' then
                        v_ar := resize(s_sa, 23);
                        v_br := resize(s_ca, 23);
                        v_wr := resize(s_wstep, 20);
                    else
                        v_ar := (others => '0');
                        v_br := (others => '0');
                        v_wr := (others => '0');
                    end if;
                    s_newframe <= '0';
                elsif s_ilace = '1' then
                    v_ar := s_arow + s_sa + s_sa;
                    v_br := s_brow + s_ca + s_ca;
                    v_wr := s_wphx + s_wstep + s_wstep;
                else
                    v_ar := s_arow + s_sa;
                    v_br := s_brow + s_ca;
                    v_wr := s_wphx + s_wstep;
                end if;
                s_arow <= v_ar;  s_apx <= v_ar;
                s_brow <= v_br;  s_bpx <= v_br;
                s_wphx <= v_wr;
                s_wphy <= (others => '0');
            elsif data_in.avid = '1' then
                if s_chev = '1' and s_xcnt >= s_cxu then
                    s_apx  <= s_apx - s_ca;      -- mirror about screen centre
                    s_bpx  <= s_bpx + s_sa;
                    s_wphy <= s_wphy - s_wstep;
                else
                    s_apx  <= s_apx + s_ca;
                    s_bpx  <= s_bpx - s_sa;
                    s_wphy <= s_wphy + s_wstep;
                end if;
            end if;
        end if;
    end process p_acc;

    ------------------------------------------------------------------------
    -- PATTERN: ripple bend -> projections -> stop index -> palette.
    ------------------------------------------------------------------------
    p_pat : process(clk)
        variable v_shift : integer range 7 to 10;
        variable v_ix    : unsigned(3 downto 0);
        variable v_ua, v_ub : unsigned(4 downto 0);
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_sub   : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            -- A: accumulator snapshot
            a_ap <= s_apx;
            a_bp <= s_bpx;
            a_wx <= s_wphx(19 downto 10);
            a_wy <= s_wphy(19 downto 10);
            a_x  <= s_xcnt(10 downto 0);

            -- W1: cross-axis triangle bend.  The two axes' displacements
            -- always enter the projection with equal weight, so they can be
            -- SUMMED BEFORE the multiply -- one multiply instead of two.
            w1_ts <= resize(f_tri7(a_wx), 8) + resize(f_tri7(a_wy), 8);
            w1_ap <= a_ap;  w1_bp <= a_bp;  w1_x <= a_x;

            -- W2: WARP depth.  The only pixel-path multiply of any size.
            -- >>1 puts the +/-63 px bend into the 1/256-px accumulator domain.
            w2_ow <= resize(shift_right(w1_ts * s_wamp, 1), 17);
            w2_ap <= w1_ap;  w2_bp <= w1_bp;  w2_x <= w1_x;

            -- W3: bend both weaves identically so PLAID stays coherent
            w3_ap <= w2_ap + resize(w2_ow, 23);
            w3_bp <= w2_bp + resize(w2_ow, 23);
            w3_x  <= w2_x;

            -- Q: ONE barrel shift per weave yields both the stop index
            -- (bits 8..5) and the sub-stop position (bits 4..0) used by the
            -- rounding, the specular line and the outline.
            v_shift := s_psh - 5;
            q_a <= resize(shift_right(unsigned(w3_ap), v_shift), 9);
            q_b <= resize(shift_right(unsigned(w3_bp), v_shift), 9);
            q_x <= w3_x;

            -- ST: ROPE folds the ramp (0..7,7..0) so each ribbon is
            -- symmetric; OUTLINE marks the first 1/16 of every stop.
            if s_rope = '1' then
                v_ix := q_a(8 downto 5);
                if v_ix(3) = '1' then st_sa <= not v_ix(2 downto 0);
                else                  st_sa <= v_ix(2 downto 0); end if;
                v_ix := q_b(8 downto 5);
                if v_ix(3) = '1' then st_sb <= not v_ix(2 downto 0);
                else                  st_sb <= v_ix(2 downto 0); end if;
            else
                st_sa <= q_a(7 downto 5);
                st_sb <= q_b(7 downto 5);
            end if;
            v_ua := q_a(4 downto 0);
            v_ub := q_b(4 downto 0);
            st_ua <= v_ua;
            st_ub <= v_ub;
            if s_outl = '1' and (v_ua < 2 or (s_plaid = '1' and v_ub < 2)) then
                st_ol <= '1';
            else
                st_ol <= '0';
            end if;
            st_x <= q_x;

            -- R1: two reads of the blended palette register file (8:1 muxes)
            ra_y <= s_ply(to_integer(st_sa));
            ra_u <= s_plu(to_integer(st_sa));
            ra_v <= s_plv(to_integer(st_sa));
            rb_y <= s_ply(to_integer(st_sb));
            rb_u <= s_plu(to_integer(st_sb));
            rb_v <= s_plv(to_integer(st_sb));
            r1_ol <= st_ol;  r1_ua <= st_ua;  r1_ub <= st_ub;  r1_x <= st_x;

            -- R2: PLAID picks the darker weave (they read as overlapping
            -- translucent bands); OUTLINE darkens the seam to a quarter.
            if s_plaid = '1' and rb_y < ra_y then
                v_y := rb_y;  v_u := rb_u;  v_v := rb_v;  v_sub := r1_ub;
            else
                v_y := ra_y;  v_u := ra_u;  v_v := ra_v;  v_sub := r1_ua;
            end if;
            if r1_ol = '1' then
                r_y <= shift_right(v_y, 2);
            else
                r_y <= v_y;
            end if;
            r_u <= v_u;  r_v <= v_v;  r_sub <= v_sub;  r_x <= r1_x;
        end if;
    end process p_pat;

    ------------------------------------------------------------------------
    -- GLOSS BODY: the sub-stop position across each ribbon is its surface
    -- normal, so a linear ramp across it reads as a rounded rope and a
    -- narrow band near the light side reads as the specular highlight.
    -- This is what makes GLOSS carry weight on a flat generated field --
    -- the luma-gradient emboss below only ever fires at the seams.
    ------------------------------------------------------------------------
    p_shade : process(clk)
        variable v_s6 : signed(5 downto 0);
        variable v_ss : signed(5 downto 0);
        variable v_sd : signed(14 downto 0);
        variable v_sh : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- S1: diffuse ramp (bright toward the upper-left light) and the
            -- specular line, both scaled by K6.
            v_s6 := signed(resize(r_sub, 6));
            v_ss := v_s6 - 16;                       -- -16..15 across the rope
            -- >>4 (not >>3): the rounding tops out at +/-255 so the candy
            -- colour still reads at full GLOSS -- the specular line below is
            -- what sells the shine, an over-deep diffuse just crushes the ramp.
            v_sd := (-v_ss) * signed(resize(s_gdif, 9));
            sh_d <= resize(shift_right(v_sd, 4), 11);
            if r_sub >= 4 and r_sub <= 6 then
                sh_s <= signed('0' & s_gspc);
            else
                sh_s <= (others => '0');
            end if;

            -- S2: sum + bound
            v_sh := resize(sh_d, 12) + resize(sh_s, 12);
            if    v_sh >  700 then sh <= to_signed(700, 12);
            elsif v_sh < -450 then sh <= to_signed(-450, 12);
            else                   sh <= v_sh; end if;
        end if;
    end process p_shade;

    ------------------------------------------------------------------------
    -- GLOSS EMBOSS: line-RAM luma heightfield -> specular sheen (g_add).
    ------------------------------------------------------------------------
    p_gloss : process(clk)
        variable v_a   : signed(13 downto 0);
        variable v_lit : signed(11 downto 0);
        variable v_sh  : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            -- L0: line RAM (read previous line, then write current).  Gated on
            -- the avid delayed to THIS stage so blanking never poisons col 0.
            if s_avid_sr(8) = '1' then
                lb_rd <= lram(to_integer(r_x));
                lram(to_integer(r_x)) <= std_logic_vector(r_y);
            end if;
            gpl   <= r_y;
            gpl_d <= gpl;

            -- L1: gradients (gpl=P, gpl_d=P-1 horizontal, lb_rd=P one line up)
            ggx <= signed('0' & gpl) - signed('0' & gpl_d);
            ggy <= signed('0' & gpl) - signed('0' & lb_rd);

            -- L2: lit = gx+gy.  Threshold + bound decided HERE (parallel, has
            -- slack) so L3 is a bare barrel shift and L4 a bare add + clamp.
            v_lit := resize(ggx, 12) + resize(ggy, 12);
            if v_lit > signed('0' & s_gspec) then spec_hi <= '1';
            else                                  spec_hi <= '0'; end if;
            if    v_lit >  255 then glit <= to_signed(255, 12);
            elsif v_lit < -255 then glit <= to_signed(-255, 12);
            else                    glit <= v_lit; end if;

            -- L3: sheen SHIFT only.  gsh = 0 -> matte.
            if s_gsh = 0 then
                v_sh := (others => '0');
            else
                v_sh := shift_left(resize(glit, 14), s_gsh - 1);
            end if;
            g_sh     <= v_sh;
            spec_hi2 <= spec_hi;

            -- L4: blown-crescent add + clamp (separate cycle from the shift).
            v_a := g_sh;
            if spec_hi2 = '1' then v_a := v_a + 360; end if;
            if v_a > 511 then v_a := to_signed(511, 14);
            elsif v_a < -300 then v_a := to_signed(-300, 14); end if;
            g_add <= resize(v_a, 11);
        end if;
    end process p_gloss;

    ------------------------------------------------------------------------
    -- OUTPUT: ribbon colour, delayed to meet the sheen, with the rounding
    -- folded in mid-chain (index 2) so no stage carries two adds.
    -- U/V are SWAPPED on the way out: the palettes are authored standard
    -- BT.601 but the hardware's u wire carries Cr and v carries Cb (verified
    -- on hardware for SUGARCOAT 2026-07-27).
    ------------------------------------------------------------------------
    p_out : process(clk)
    begin
        if rising_edge(clk) then
            cy_sr(0) <= r_y;      cu_sr(0) <= r_u;      cv_sr(0) <= r_v;
            cy_sr(1) <= cy_sr(0); cu_sr(1) <= cu_sr(0); cv_sr(1) <= cv_sr(0);
            -- ribbon rounding + specular line lands here
            cy_sr(2) <= f_cu10(signed(resize(cy_sr(1), 13)) + resize(sh, 13));
            cu_sr(2) <= cu_sr(1); cv_sr(2) <= cv_sr(1);
            cy_sr(3) <= cy_sr(2); cu_sr(3) <= cu_sr(2); cv_sr(3) <= cv_sr(2);
            cy_sr(4) <= cy_sr(3); cu_sr(4) <= cu_sr(3); cv_sr(4) <= cv_sr(3);

            o_y <= f_cu10(signed(resize(cy_sr(4), 12)) + resize(g_add, 12));
            o_u <= cv_sr(4);
            o_v <= cu_sr(4);
        end if;
    end process p_out;

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

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(o_u);
    data_out.v       <= std_logic_vector(o_v);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture ribbon;
