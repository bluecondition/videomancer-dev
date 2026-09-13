-- pinboard.vhd  (v0.1)
--
-- A dense field of square pins seen in isometric 3/4 view, with a ball hovering
-- and skating over it.  Pins near the ball are agitated: they pop up out of the
-- board and sink back at random phases, so a shimmering patch of raised tiles
-- follows the ball around.  Optionally the ball also presses a stepped dome up
-- out of the field ("Bulge").
--
-- Port of the Shadertoy raymarcher https://www.shadertoy.com/view/tssSDN
-- (a grid of boxes whose heights are sin(hash + t) attenuated by the distance to
-- a moving sphere).  The 128-step raymarch is replaced by an orthographic
-- isometric heightfield march, which is what an HX4K can actually run at pixel
-- rate; the perspective camera and the horizon are the cost of that.
--
-- ENGINE (forked from ziggurat v0.4, then reformulated on remainders)
--
--   Two accumulators walk the isometric coordinates with no multiply: the cell
--   under a pixel is i = floor(V/W2), j = floor(-U/W2) with W2 = 2ab, kept as
--   index + remainder pairs (ivrem, jurem in [0,W2)) that step by b per pixel
--   and by a per active line.
--
--   Writing i = iv+k, j = ju+k, ziggurat's two cover half-planes collapse to
--   small numbers -- both of them become
--
--        ivrem + e >= 0     and     jurem + e >= 0,      e = aH - k*W2
--
--   so U, V and the W2*i products never have to exist at all.  Marching k from
--   KF (nearest) down to 0 and taking the first cover gives the nearest visible
--   pin; k=0 always covers, so the screen is always filled.
--
--   Cover at k needs aH > (k-1)*W2, and one cell of pin height is W2/2, so a
--   two-cell pin only ever reaches k=1 -- KF=1 is not a compromise here, it is
--   the exact depth the height budget can use.  KF=3 fitted at 104% of the
--   device, KF=2 at 94% and 54 MHz; this is 85%.
--
--   For the winner, p = ivrem + e and q = jurem + e locate the pixel inside the
--   pin's silhouette:  p,q < W2 -> TOP face, else RIGHT if p>q else LEFT
--   (p-q = ivrem-jurem is a pure horizontal coordinate, p+q a pure vertical one).
--
--   Pin height is evaluated in an 8-stage side pipeline (H1..H8) fed only by the
--   cell indices, so no hash and no product ever lands on the march's critical
--   path -- ivrem/jurem are simply delayed to meet it.  Height is
--     aH = (A * tri(hash(i,j) + time) / 32 + B) >> gs
--   where tri is a 5-bit triangle wave (a fold, not a LUT), gs is a 5-step
--   radial gate around the ball (4 compares) and A,B are per-frame constants,
--   so the "multiply" is a five-term add of shifted copies of A -- and since A
--   is a register, every shifted copy is free wiring.  The gate is applied to
--   the finished height rather than to the amplitude: it is the same number
--   either way, but it keeps two wide barrel muxes per marched cell out of the
--   design and off what was the critical path.
--
--   The KF+1 candidates are the diagonal iv+k, ju+k, so their hash bases are
--   constant increments of a single i*73 / j*149.  The gate and the hash must
--   still be evaluated per k: sharing one across the marched cells would make a
--   cell's height depend on which pixel was looking at it, and its top face
--   would break up.
--
--   The ball is drawn from two squared-distance accumulators (sqx += 2dx+1 per
--   pixel, sqy += 2dy+1 per line -- no per-pixel multiply).  Its shade is
--     sh = (R2 - r2)>>ks - (dx+dy)>>kd
--   whose level sets are exact circles offset up-left, so that one expression
--   gives the dome, the light direction and (by threshold) a round specular.
--   Occlusion against the board is one compare: the ball hides behind a pin iff
--   bi+bj < i+j.
--
--   Everything with a real multiply in it (K1 scaling, a*b, amplitude, orbit,
--   ball radius, cx^2, cy^2) runs on ONE 16x16 shift-add sequential multiplier
--   driven by a vblank sequencer: ~200 cycles per field, one 18-bit add per
--   cycle, so the blanking cone is not what caps Fmax either.
--
-- CONTROLS
--   K1  Pin Size      cell size of the board
--   K2  Rate          shimmer speed (also drives the ball's drift)
--   K3  Pop           how far the agitated pins rise (0..2 cells; Bulge halves it)
--   K4  Reach         radius of the ball's influence, in cells
--   K5  Roam          size of the ball's path
--   K6  Hue           board hue (sides); the ball takes the complement
--   T7  Edges         face-seam outlines
--   T8  Field         Near = pins near the ball shimmer / Far = inverted
--   T9  View          Standard 2:1 iso / Shallow
--   T10 Video         paint the incoming picture onto the pin faces
--   T11 Bulge         ball also presses a stepped dome up out of the board
--   P12 Ball Size     ball diameter, tiny dot -> half the screen (it rises with
--                     its own radius so it always clears the board)
--
-- Latency: 16 clocks.  BRAM: 6 (the shared sin/cos LUT).  No DSP on this part.
-- Chroma is authored in standard BT.601 and swapped once at the output pins
-- (Videomancer hardware swaps U/V); outputs are forced neutral outside avid.
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture pinboard of program_top is

    --------------------------------------------------------------------------
    -- Sizes
    --------------------------------------------------------------------------
    constant KF   : integer := 1;               -- march depth (KF+1 cells tested)
    constant C_W  : integer := 19;              -- march arithmetic (p, q, e, aH)
    subtype  t_w  is signed(C_W - 1 downto 0);
    constant C_IW : integer := 11;              -- cell index
    subtype  t_i  is signed(C_IW - 1 downto 0);
    constant C_DW : integer := 16;              -- cell distance, Q4 cells
    subtype  t_d  is signed(C_DW - 1 downto 0);
    subtype  t_du is unsigned(C_DW - 2 downto 0);
    subtype  t_ds is unsigned(10 downto 0);      -- saturated gate distance
    constant C_LAT : integer := 16;             -- data_in -> data_out
    constant SY   : integer := 4 * (C_LAT - 2); -- last sync group (stage C_LAT-1)

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_warr is array(0 to KF) of t_w;
    type t_iarr is array(0 to KF) of t_i;
    type t_darr is array(0 to KF) of t_d;
    type t_uarr is array(0 to KF) of t_du;
    type t_sarr is array(0 to KF) of t_ds;
    type t_harr is array(0 to KF) of unsigned(19 downto 0);
    type t_parr is array(0 to KF) of unsigned(9 downto 0);
    type t_tarr is array(0 to KF) of unsigned(4 downto 0);
    type t_garr is array(0 to KF) of unsigned(2 downto 0);

    --------------------------------------------------------------------------
    -- Controls
    --------------------------------------------------------------------------
    signal s_k1, s_k2, s_k3, s_k4, s_k5, s_k6 : unsigned(9 downto 0) := (others => '0');
    signal s_p12      : unsigned(9 downto 0) := (others => '0');
    signal s_edges    : std_logic := '1';       -- T7
    signal s_farfield : std_logic := '0';       -- T8
    signal s_shallow  : std_logic := '0';       -- T9
    signal s_video    : std_logic := '0';       -- T10
    signal s_bulge    : std_logic := '0';       -- T11

    --------------------------------------------------------------------------
    -- Timing / raster measurement
    --------------------------------------------------------------------------
    signal s_timing   : t_video_timing_port;
    signal s_hcnt     : unsigned(11 downto 0) := (others => '0');
    signal s_vcnt     : unsigned(11 downto 0) := (others => '0');
    signal s_meas_h   : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_meas_v   : unsigned(11 downto 0) := to_unsigned(540, 12);
    signal s_ilace    : std_logic := '0';
    signal s_botfield : std_logic := '0';

    --------------------------------------------------------------------------
    -- Per-frame constants (written by the vblank sequencer)
    --------------------------------------------------------------------------
    signal s_a      : t_w := to_signed(40, C_W);      -- cell half width, px
    signal s_b      : t_w := to_signed(20, C_W);      -- cell half height, px
    signal s_W2     : t_w := to_signed(1600, C_W);    -- 2ab = one cell
    signal s_lstep  : t_w := to_signed(40, C_W);      -- iso V step per active line
    signal s_seedrem: t_w := (others => '0');         -- field-parity seed
    signal s_thr    : t_w := to_signed(50, C_W);      -- outline half-width
    signal s_bmw2   : t_w := to_signed(-1580, C_W);   -- b - W2  (pre-biased step)
    signal s_w2mb   : t_w := to_signed(1580, C_W);    -- W2 - b
    signal s_amp    : t_w := (others => '0');         -- A: wobble amplitude
    signal s_blg    : t_w := (others => '0');         -- B: bulge amplitude
    signal s_kw2    : t_warr := (others => (others => '0'));   -- k*W2
    signal s_r1, s_r2t, s_r3, s_r4 : t_ds := (others => '0');

    signal s_ci, s_cj   : t_i := (others => '0');     -- cell at screen centre
    signal s_bi, s_bj   : t_i := (others => '0');     -- ball's cell
    signal s_bdepth     : signed(C_IW + 1 downto 0) := (others => '0');  -- bi+bj
    signal s_oi, s_oj   : t_d := (others => '0');     -- ball offset, Q4 cells
    signal s_oif, s_ojf : t_d := (others => '0');     -- its sub-cell part

    signal s_cx, s_cy   : signed(13 downto 0) := (others => '0');   -- ball centre, px
    signal s_cx2, s_cy2 : unsigned(23 downto 0) := (others => '0'); -- their squares
    signal s_rad        : unsigned(10 downto 0) := (others => '0'); -- ball radius
    signal s_rad2       : signed(24 downto 0) := (others => '0');   -- radius^2
    signal s_ksh        : integer range 0 to 15 := 0;   -- dome shift right
    signal s_ksl        : integer range 0 to 15 := 0;   -- dome shift left (small balls)
    signal s_kdr        : integer range 0 to 7  := 0;   -- lambert shift right
    signal s_kdl        : integer range 0 to 7  := 0;   -- lambert shift left

    -- colours (standard BT.601, swapped once at the output)
    signal s_top_y, s_top_u, s_top_v : unsigned(9 downto 0) := C_MID;
    signal s_lft_y, s_rgt_y          : unsigned(9 downto 0) := C_MID;
    signal s_sid_u, s_sid_v          : unsigned(9 downto 0) := C_MID;
    signal s_bal_u, s_bal_v          : unsigned(9 downto 0) := C_MID;

    -- animation phases (advance once per field)
    signal s_time : unsigned(9 downto 0) := (others => '0');
    signal s_ph1  : unsigned(9 downto 0) := (others => '0');
    signal s_ph2  : unsigned(9 downto 0) := to_unsigned(256, 10);

    --------------------------------------------------------------------------
    -- Shared sequential multiplier + vblank sequencer
    --------------------------------------------------------------------------
    constant OP_LAST : integer := 10;
    signal s_op    : integer range 0 to 15 := OP_LAST;
    signal s_bit   : integer range 0 to 17 := 17;
    signal s_run   : std_logic := '0';
    signal s_mA    : unsigned(16 downto 0) := (others => '0');
    signal s_mB    : unsigned(15 downto 0) := (others => '0');
    signal s_macc  : unsigned(32 downto 0) := (others => '0');
    signal s_msign : std_logic := '0';
    signal s_armed : std_logic := '0';

    signal s_lut_ang : std_logic_vector(9 downto 0) := (others => '0');
    signal s_lut_sin, s_lut_cos : signed(9 downto 0);
    signal s_sin1, s_sin2 : signed(9 downto 0) := (others => '0');
    signal s_orb   : unsigned(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Stage 1: isometric accumulators
    --------------------------------------------------------------------------
    signal s_iv, s_ju       : t_i := (others => '0');
    signal s_ivrem, s_jurem : t_w := (others => '0');
    signal s_base           : t_i := (others => '0');
    signal s_baserem        : t_w := (others => '0');

    --------------------------------------------------------------------------
    -- Stage 1: ball distance accumulators
    --------------------------------------------------------------------------
    signal s_dx   : signed(13 downto 0) := (others => '0');
    signal s_dy   : signed(13 downto 0) := (others => '0');
    signal s_sqx  : unsigned(23 downto 0) := (others => '0');
    signal s_sqy  : unsigned(23 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Height pipeline H1..H7
    --------------------------------------------------------------------------
    signal h1_hi, h1_hj   : unsigned(19 downto 0) := (others => '0');
    signal h1_di, h1_dj   : t_d := (others => '0');
    signal h2_hx          : t_harr := (others => (others => '0'));
    signal h2_di, h2_dj   : t_darr := (others => (others => '0'));
    signal h3_ph          : t_parr := (others => (others => '0'));
    signal h3_ad, h3_aj   : t_uarr := (others => (others => '0'));
    signal h4_tri         : t_tarr := (others => (others => '0'));
    signal h4_d           : t_sarr := (others => (others => '0'));
    signal h5_tri         : t_tarr := (others => (others => '0'));
    signal h5_gs          : t_garr := (others => (others => '0'));
    signal h6_s1, h6_s2   : t_warr := (others => (others => '0'));
    signal h6_gs          : t_garr := (others => (others => '0'));
    signal h7_raw         : t_warr := (others => (others => '0'));
    signal h7_gs          : t_garr := (others => (others => '0'));
    signal h8_e           : t_warr := (others => (others => '0'));
    signal h8_gs          : t_garr := (others => (others => '0'));

    -- ivrem / jurem / iv / ju delayed to meet the march (index n valid at stage n+2)
    subtype t_rem is unsigned(14 downto 0);
    type t_wsr is array(0 to 7) of t_rem;
    type t_isr is array(0 to 7) of t_i;
    signal dl_ivrem, dl_jurem : t_wsr := (others => (others => '0'));
    signal dl_iv, dl_ju       : t_isr := (others => (others => '0'));

    --------------------------------------------------------------------------
    -- March / classify
    --------------------------------------------------------------------------
    signal m1_cov  : std_logic_vector(0 to KF) := (others => '0');
    signal m1_e    : t_warr := (others => (others => '0'));
    signal m1_gs   : t_garr := (others => (others => '0'));
    signal m1_ivrem, m1_jurem : t_w := (others => '0');
    signal m1_iv, m1_ju       : t_i := (others => '0');

    signal m2_e    : t_w := (others => '0');
    signal m2_gs   : unsigned(2 downto 0) := (others => '0');
    signal m2_k    : integer range 0 to KF := 0;
    signal m2_ivrem, m2_jurem : t_w := (others => '0');
    signal m2_iv, m2_ju       : t_i := (others => '0');

    signal c1_p, c1_q : t_w := (others => '0');
    signal c1_pw, c1_qw, c1_dpq : t_w := (others => '0');
    signal c1_gs      : unsigned(2 downto 0) := (others => '0');
    signal c1_i, c1_j : t_i := (others => '0');
    signal c1_depth   : signed(C_IW + 1 downto 0) := (others => '0');

    signal c2_top, c2_right, c2_line : std_logic := '0';
    signal c2_gs    : unsigned(2 downto 0) := (others => '0');
    signal c2_cell  : std_logic_vector(11 downto 0) := (others => '0');
    signal c2_depth : signed(C_IW + 1 downto 0) := (others => '0');

    signal c3_y, c3_u, c3_v : unsigned(9 downto 0) := (others => '0');
    signal c3_depth : signed(C_IW + 1 downto 0) := (others => '0');
    signal s_prev_cell : std_logic_vector(11 downto 0) := (others => '1');
    signal s_hy : unsigned(9 downto 0) := (others => '0');
    signal s_hu, s_hv : unsigned(9 downto 0) := C_MID;

    signal c4_y, c4_u, c4_v : unsigned(9 downto 0) := (others => '0');
    signal s_io : t_video_stream_yuv444_30b;

    --------------------------------------------------------------------------
    -- Ball shading pipeline (stages 2..4), then delayed to stage 13
    --------------------------------------------------------------------------
    signal b1_r2   : signed(24 downto 0) := (others => '0');
    signal b1_dsum : signed(19 downto 0) := (others => '0');
    signal b2_dif  : signed(24 downto 0) := (others => '0');
    signal b2_lam  : signed(19 downto 0) := (others => '0');
    signal b2_in   : std_logic := '0';
    signal b3_sh   : signed(19 downto 0) := (others => '0');
    signal b3_in2  : std_logic := '0';
    signal b3_y    : unsigned(9 downto 0) := (others => '0');
    signal b3_in, b3_spec : std_logic := '0';
    type t_bsr is array(0 to 8) of unsigned(11 downto 0);
    signal dl_ball : t_bsr := (others => (others => '0'));

    signal s_sync_sr : std_logic_vector(0 to 4 * (C_LAT - 1) - 1) := (others => '0');

    --------------------------------------------------------------------------
    -- 16-entry hue wheel (radius 400 about 512, standard BT.601: U=Cb, V=Cr)
    --------------------------------------------------------------------------
    type t_hue is array(0 to 15) of integer;
    constant C_HU : t_hue := (912, 882, 795, 665, 512, 359, 229, 142,
                              112, 142, 229, 359, 512, 665, 795, 882);
    constant C_HV : t_hue := (512, 665, 795, 882, 912, 882, 795, 665,
                              512, 359, 229, 142, 112, 142, 229, 359);

    -- 0 <= x < thr, with thr always under 11 bits: one short compare plus a
    -- zero test on the rest, instead of a full-width carry chain.
    function near0(x : t_w; thr : t_w) return boolean is
    begin
        return x(C_W - 1 downto 11) = 0 and x(10 downto 0) < thr(10 downto 0);
    end function;

begin

    --------------------------------------------------------------------------
    -- Timing infrastructure
    --------------------------------------------------------------------------
    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    sincos_inst : entity work.sin_cos_full_lut_10x10
        port map (angle_in => s_lut_ang, sin_out => s_lut_sin, cos_out => s_lut_cos);

    -- The orbit angles only change once per field, and each sequencer op takes
    -- 18 clocks, so this read has ample time to settle before it is sampled.
    s_lut_ang <= std_logic_vector(s_ph1) when s_op <= 1 else std_logic_vector(s_ph2);

    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_hcnt > 0 then s_meas_h <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            elsif s_timing.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            end if;

            if s_timing.vsync_start = '1' then
                if s_vcnt > 0 then s_meas_v <= s_vcnt; end if;
                s_vcnt     <= (others => '0');
                s_ilace    <= s_timing.is_interlaced;
                s_botfield <= not s_timing.field_n;      -- bottom field = field_n '0'
            elsif s_timing.avid_start = '1' then
                s_vcnt <= s_vcnt + 1;
            end if;

            -- the cell under the screen centre: the ball's frame of reference
            if s_timing.avid = '1' and s_hcnt = shift_right(s_meas_h, 1)
                                   and s_vcnt = shift_right(s_meas_v, 1) then
                s_ci <= s_iv;
                s_cj <= s_ju;
            end if;
        end if;
    end process p_measure;

    --------------------------------------------------------------------------
    -- Per-frame constants.  One 16x16 shift-add multiply per op, 18 clocks
    -- each, all of it inside the vertical blanking interval.  Nothing in here
    -- is wider than an 18-bit add.
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_acc  : unsigned(33 downto 0);
        variable v_a, v_b, v_ab, v_w2 : t_w;
        variable v_prod : unsigned(32 downto 0);
        variable v_sg   : signed(C_DW downto 0);
        variable v_mag  : unsigned(15 downto 0);
        variable v_hi   : integer range 0 to 15;
        variable v_hu, v_hv : integer range 0 to 1023;
        variable v_h2, v_v2 : signed(13 downto 0);
        variable v_r1   : t_ds;
        variable v_l, v_l2 : integer range 0 to 24;
        variable v_step : unsigned(5 downto 0);
        variable v_oi, v_oj : t_d;
    begin
        if rising_edge(clk) then

            if s_timing.vsync_start = '1' then
                s_k1  <= unsigned(registers_in(0));
                s_k2  <= unsigned(registers_in(1));
                s_k3  <= unsigned(registers_in(2));
                s_k4  <= unsigned(registers_in(3));
                s_k5  <= unsigned(registers_in(4));
                s_k6  <= unsigned(registers_in(5));
                s_edges    <= registers_in(6)(0);
                s_farfield <= registers_in(6)(1);
                s_shallow  <= registers_in(6)(2);
                s_video    <= registers_in(6)(3);
                s_bulge    <= registers_in(6)(4);
                s_p12 <= unsigned(registers_in(7));

                -- animation clocks; the ball keeps a slow drift even at Rate 0
                v_step := resize(shift_right(unsigned(registers_in(1)), 5), 6);
                s_time <= s_time + v_step;
                s_ph1  <= s_ph1 + resize(shift_right(v_step, 2), 10) + 1;
                s_ph2  <= s_ph2 + resize(shift_right(v_step, 2)
                                       + shift_right(v_step, 4), 10) + 1;

                s_op    <= 0;
                s_bit   <= 0;
                s_run   <= '1';
                s_armed <= '1';

            elsif s_run = '1' and s_timing.avid = '0' then

                if s_bit = 0 then
                    ---------------------------------------------------------
                    -- load operands
                    ---------------------------------------------------------
                    s_macc  <= (others => '0');
                    s_msign <= '0';
                    v_mag   := (others => '0');
                    case s_op is
                        when 0 =>       -- K1 * raster width -> cell half width
                            s_mA <= resize(s_k1, 17);
                            s_mB <= resize(s_meas_h, 16);

                        when 1 =>       -- a * b
                            s_mA <= resize(unsigned(s_a(11 downto 0)), 17);
                            s_mB <= resize(unsigned(s_b(11 downto 0)), 16);

                        when 2 =>       -- K3 * W2 -> wobble amplitude (0..2 cells)
                            s_mA <= resize(s_k3, 17);
                            s_mB <= unsigned(s_W2(15 downto 0));

                        when 3 =>       -- P12 * raster width -> ball radius
                            s_mA <= resize(s_p12, 17);
                            s_mB <= resize(s_meas_h, 16);

                        when 4 =>       -- radius^2
                            s_mA <= resize(s_rad, 17);
                            s_mB <= resize(s_rad, 16);

                        when 5 =>       -- sin(ph1) * roam
                            if s_sin1 < 0 then
                                v_mag   := resize(unsigned(-s_sin1), 16);
                                s_msign <= '1';
                            else
                                v_mag   := resize(unsigned(s_sin1), 16);
                            end if;
                            s_mA <= resize(v_mag, 17);
                            s_mB <= resize(s_orb, 16);

                        when 6 =>       -- cos(ph2) * roam
                            if s_sin2 < 0 then
                                v_mag   := resize(unsigned(-s_sin2), 16);
                                s_msign <= '1';
                            else
                                v_mag   := resize(unsigned(s_sin2), 16);
                            end if;
                            s_mA <= resize(v_mag, 17);
                            s_mB <= resize(s_orb, 16);

                        when 7 =>       -- a * (oi - oj) -> ball screen x offset
                            v_sg := resize(s_oi, C_DW + 1) - resize(s_oj, C_DW + 1);
                            if v_sg < 0 then
                                v_mag   := resize(unsigned(-v_sg), 16);
                                s_msign <= '1';
                            else
                                v_mag   := resize(unsigned(v_sg), 16);
                            end if;
                            s_mA <= resize(unsigned(s_a(11 downto 0)), 17);
                            s_mB <= v_mag;

                        when 8 =>       -- b * (oi + oj) -> ball screen y offset
                            v_sg := resize(s_oi, C_DW + 1) + resize(s_oj, C_DW + 1);
                            if v_sg < 0 then
                                v_mag   := resize(unsigned(-v_sg), 16);
                                s_msign <= '1';
                            else
                                v_mag   := resize(unsigned(v_sg), 16);
                            end if;
                            s_mA <= resize(unsigned(s_b(11 downto 0)), 17);
                            s_mB <= v_mag;

                        when 9 =>       -- cx^2
                            if s_cx < 0 then v_mag := resize(unsigned(-s_cx), 16);
                            else             v_mag := resize(unsigned(s_cx), 16); end if;
                            s_mA <= resize(v_mag, 17);
                            s_mB <= v_mag;

                        when others =>  -- cy^2
                            if s_cy < 0 then v_mag := resize(unsigned(-s_cy), 16);
                            else             v_mag := resize(unsigned(s_cy), 16); end if;
                            s_mA <= resize(v_mag, 17);
                            s_mB <= v_mag;
                    end case;
                    s_bit <= 1;

                elsif s_bit <= 16 then
                    ---------------------------------------------------------
                    -- one shift-add step (18-bit adder, right-shifting acc)
                    ---------------------------------------------------------
                    v_acc := '0' & s_macc;
                    if s_mB(0) = '1' then
                        v_acc(33 downto 16) := ('0' & v_acc(32 downto 16)) + ('0' & s_mA);
                    end if;
                    s_macc <= v_acc(33 downto 1);
                    s_mB   <= '0' & s_mB(15 downto 1);
                    s_bit  <= s_bit + 1;

                else
                    ---------------------------------------------------------
                    -- capture
                    ---------------------------------------------------------
                    v_prod := s_macc;
                    case s_op is
                        when 0 =>
                            -- pins are 24..~270 px across at HD, scaled by raster
                            v_a := to_signed(12, C_W)
                                 + signed(resize(v_prod(24 downto 14), C_W));
                            if s_shallow = '1' then
                                v_b := shift_right(v_a, 2) + shift_right(v_a, 4);
                            else
                                v_b := shift_right(v_a, 1);
                            end if;
                            s_a <= v_a;
                            s_b <= v_b;
                            if s_ilace = '1' then s_lstep <= shift_left(v_a, 1);
                            else                  s_lstep <= v_a; end if;
                            if s_ilace = '1' and s_botfield = '1' then
                                s_seedrem <= v_a;
                            else
                                s_seedrem <= (others => '0');
                            end if;

                            -- reach ladder, Q4 cells: R, 1.5R, 2R, 2.5R
                            v_r1  := resize(shift_right(s_k4, 1), 11) + 16;
                            s_r1  <= v_r1;
                            s_r2t <= v_r1 + shift_right(v_r1, 1);
                            s_r3  <= shift_left(v_r1, 1);
                            s_r4  <= shift_left(v_r1, 1) + shift_right(v_r1, 1);
                            s_orb <= shift_right(s_k5, 2);

                            -- hue wheel: sides fully saturated, tops tinted white
                            v_hi := to_integer(s_k6(9 downto 6));
                            v_hu := C_HU(v_hi);
                            v_hv := C_HV(v_hi);
                            s_sid_u <= to_unsigned(v_hu, 10);
                            s_sid_v <= to_unsigned(v_hv, 10);
                            s_top_u <= to_unsigned(512 + (v_hu - 512) / 4, 10);
                            s_top_v <= to_unsigned(512 + (v_hv - 512) / 4, 10);
                            s_bal_u <= to_unsigned(1024 - v_hu, 10);
                            s_bal_v <= to_unsigned(1024 - v_hv, 10);
                            s_top_y <= to_unsigned(790, 10);
                            s_lft_y <= to_unsigned(470, 10);
                            s_rgt_y <= to_unsigned(270, 10);

                        when 1 =>
                            v_ab := signed(resize(v_prod(19 downto 0), C_W));
                            v_w2 := shift_left(v_ab, 1);
                            s_W2  <= v_w2;
                            s_thr <= shift_right(v_w2, 5) + 2;
                            for k in 0 to KF loop
                                s_kw2(k) <= resize(to_signed(k, 3) * v_w2, C_W);
                            end loop;
                            s_bmw2 <= s_b - v_w2;
                            s_w2mb <= v_w2 - s_b;
                            s_sin1 <= s_lut_sin;

                        when 2 =>
                            -- aH must stay <= W2 (2 cells) or a pin could out-reach
                            -- the march, so Bulge halves Pop and takes the rest
                            if s_bulge = '1' then
                                s_amp <= signed(resize(v_prod(25 downto 11), C_W));
                                s_blg <= shift_right(s_W2, 1);
                            else
                                s_amp <= signed(resize(v_prod(25 downto 10), C_W));
                                s_blg <= (others => '0');
                            end if;

                        when 3 =>
                            s_rad  <= resize(v_prod(23 downto 12), 11) + 6;
                            s_sin2 <= s_lut_cos;

                        when 4 =>
                            s_rad2 <= signed(resize(v_prod(23 downto 0), 25));
                            -- keep both shade terms resolution-independent:
                            -- (R2 - r2) >> ksh lands in [512, 1024)
                            v_l2 := 0;
                            for n in 0 to 23 loop
                                if v_prod(n) = '1' then v_l2 := n; end if;
                            end loop;
                            if v_l2 >= 9 then s_ksh <= v_l2 - 9; else s_ksh <= 0; end if;
                            v_l := 0;
                            for n in 0 to 10 loop
                                if s_rad(n) = '1' then v_l := n; end if;
                            end loop;
                            if v_l >= 7 then s_kdr <= v_l - 7;  s_kdl <= 0;
                            else               s_kdr <= 0;        s_kdl <= 7 - v_l; end if;

                        when 5 =>
                            if s_msign = '1' then
                                s_oi <= -signed(resize(v_prod(18 downto 9), C_DW));
                            else
                                s_oi <=  signed(resize(v_prod(18 downto 9), C_DW));
                            end if;

                        when 6 =>
                            if s_msign = '1' then
                                v_oj := -signed(resize(v_prod(18 downto 9), C_DW));
                            else
                                v_oj :=  signed(resize(v_prod(18 downto 9), C_DW));
                            end if;
                            v_oi := s_oi;
                            s_oj <= v_oj;
                            -- ball's cell, its sub-cell remainder and its depth key
                            s_bi <= s_ci + resize(shift_right(v_oi, 4), C_IW);
                            s_bj <= s_cj + resize(shift_right(v_oj, 4), C_IW);
                            s_oif <= v_oi - shift_left(shift_right(v_oi, 4), 4);
                            s_ojf <= v_oj - shift_left(shift_right(v_oj, 4), 4);
                            s_bdepth <= resize(s_ci, C_IW + 2) + resize(s_cj, C_IW + 2)
                                      + resize(shift_right(v_oi, 4), C_IW + 2)
                                      + resize(shift_right(v_oj, 4), C_IW + 2);

                        when 7 =>
                            v_h2 := signed(resize(shift_right(s_meas_h, 1), 14));
                            if s_msign = '1' then
                                s_cx <= v_h2 - signed(resize(v_prod(17 downto 4), 14));
                            else
                                s_cx <= v_h2 + signed(resize(v_prod(17 downto 4), 14));
                            end if;

                        when 8 =>
                            if s_ilace = '1' then
                                v_v2 := signed(resize(s_meas_v, 14));
                            else
                                v_v2 := signed(resize(shift_right(s_meas_v, 1), 14));
                            end if;
                            -- the ball rides one cell clear of its own underside
                            v_v2 := v_v2 - signed(resize(s_rad, 14)) - resize(s_b, 14);
                            if s_msign = '1' then
                                s_cy <= v_v2 - signed(resize(v_prod(17 downto 4), 14));
                            else
                                s_cy <= v_v2 + signed(resize(v_prod(17 downto 4), 14));
                            end if;

                        when 9 =>
                            s_cx2 <= resize(v_prod(23 downto 0), 24);

                        when others =>
                            s_cy2 <= resize(v_prod(23 downto 0), 24);
                    end case;

                    if s_op = OP_LAST then
                        s_run <= '0';
                    else
                        s_op <= s_op + 1;
                    end if;
                    s_bit <= 0;
                end if;
            end if;

            if s_timing.avid_start = '1' then
                s_armed <= '0';
            end if;
        end if;
    end process p_frame;

    --------------------------------------------------------------------------
    -- Stage 1a: isometric accumulators.  iv/ju step with their remainders, so
    -- there is no divide anywhere and no wide V/U to carry down the pipe.
    --------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_brem, v_next, v_wrap : t_w;
        variable v_base : t_i;
    begin
        if rising_edge(clk) then
            if s_timing.avid_start = '1' then
                if s_armed = '1' then                 -- first active line of the field
                    v_brem := s_seedrem;
                    v_base := (others => '0');
                else
                    v_brem := s_baserem;
                    v_base := s_base;
                end if;
                s_ivrem <= v_brem;  s_jurem <= v_brem;
                s_iv    <= v_base;  s_ju    <= v_base;

                v_next := v_brem + s_lstep;
                if v_next >= s_W2 then
                    s_base <= v_base + 1;  s_baserem <= v_next - s_W2;
                else
                    s_base <= v_base;      s_baserem <= v_next;
                end if;

            elsif s_timing.avid = '1' then
                -- V += b  ->  ivrem climbs, iv steps up.  Both candidates are
                -- computed in parallel from pre-biased constants and selected on
                -- a sign bit; add-then-compare-then-subtract was a 3-deep chain.
                v_wrap := s_ivrem + s_bmw2;
                if v_wrap >= 0 then
                    s_iv <= s_iv + 1;  s_ivrem <= v_wrap;
                else
                    s_ivrem <= s_ivrem + s_b;
                end if;
                -- U += b  ->  jurem falls, ju steps down
                v_wrap := s_jurem - s_b;
                if v_wrap >= 0 then
                    s_jurem <= v_wrap;
                else
                    s_ju <= s_ju - 1;  s_jurem <= s_jurem + s_w2mb;
                end if;
            end if;
        end if;
    end process p_acc;

    --------------------------------------------------------------------------
    -- Stage 1b: ball distance accumulators.  sqx/sqy hold dx^2 and dy^2 and are
    -- advanced by 2d+1 (4d+4 for an interlaced two-line step), so the squared
    -- distance to the ball costs no multiply per pixel.
    --------------------------------------------------------------------------
    p_ball_acc : process(clk)
        variable v_dy   : signed(13 downto 0);
        variable v_seed : signed(24 downto 0);
    begin
        if rising_edge(clk) then
            if s_timing.avid_start = '1' then
                s_dx  <= -s_cx;
                s_sqx <= s_cx2;
                if s_armed = '1' then
                    if s_ilace = '1' and s_botfield = '1' then
                        -- field parity: seed with (cy-1)^2 = cy^2 - 2cy + 1
                        s_dy   <= -s_cy + 1;
                        v_seed := signed('0' & s_cy2) - resize(shift_left(s_cy, 1), 25) + 1;
                        if v_seed < 0 then
                            s_sqy <= (others => '0');
                        else
                            s_sqy <= unsigned(v_seed(23 downto 0));
                        end if;
                    else
                        s_dy  <= -s_cy;
                        s_sqy <= s_cy2;
                    end if;
                end if;
            elsif s_timing.avid = '1' then
                s_dx  <= s_dx + 1;
                s_sqx <= s_sqx + unsigned(resize(shift_left(s_dx, 1) + 1, 24));
            end if;

            if s_timing.avid_end = '1' then
                v_dy := s_dy;
                if s_ilace = '1' then
                    s_dy  <= v_dy + 2;
                    s_sqy <= s_sqy + unsigned(resize(shift_left(v_dy, 2) + 4, 24));
                else
                    s_dy  <= v_dy + 1;
                    s_sqy <= s_sqy + unsigned(resize(shift_left(v_dy, 1) + 1, 24));
                end if;
            end if;
        end if;
    end process p_ball_acc;

    --------------------------------------------------------------------------
    -- Ball shading (stages 2..4).  sh = (R2-r2)>>ks - (dx+dy)>>kd; its level
    -- sets are exact circles offset up-left, so this one expression is the
    -- dome, the light direction and (by threshold) a round specular.
    --------------------------------------------------------------------------
    p_ball : process(clk)
        variable v_sh  : signed(19 downto 0);
        variable v_y   : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            -- B1
            b1_r2   <= signed('0' & (resize(s_sqx, 24) + resize(s_sqy, 24)));
            b1_dsum <= resize(s_dx, 20) + resize(s_dy, 20);

            -- B2a: the difference and the light term in parallel
            if b1_r2 <= s_rad2 then b2_in <= '1'; else b2_in <= '0'; end if;
            b2_dif <= s_rad2 - b1_r2;
            b2_lam <= resize(shift_right(shift_left(resize(b1_dsum, 25), s_kdl),
                                         s_kdr), 20);

            -- B2b: gate, scale and combine
            b3_sh <= resize(shift_right(b2_dif, s_ksh), 20) - b2_lam;
            b3_in2 <= b2_in;

            -- B3
            v_sh := b3_sh;
            v_y  := to_signed(150, 20) + v_sh;
            if v_sh > 700 then
                b3_y    <= to_unsigned(1000, 10);
                b3_spec <= '1';
            else
                b3_spec <= '0';
                if    v_y > 1000 then b3_y <= to_unsigned(1000, 10);
                elsif v_y < 90   then b3_y <= to_unsigned(90, 10);
                else                  b3_y <= unsigned(v_y(9 downto 0)); end if;
            end if;
            b3_in <= b3_in2;

            -- align with the field pipeline (consumed at stage 14)
            dl_ball(0)(11)          <= b3_spec;
            dl_ball(0)(10)          <= b3_in;
            dl_ball(0)(9 downto 0)  <= b3_y;
            for n in 1 to 8 loop
                dl_ball(n) <= dl_ball(n - 1);
            end loop;
        end if;
    end process p_ball;

    --------------------------------------------------------------------------
    -- H1: hash bases and the ball-relative cell offset (Q4 cells).
    --------------------------------------------------------------------------
    p_h1 : process(clk)
        variable v_i, v_j : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            v_i := resize(s_iv, 20);
            v_j := resize(s_ju, 20);
            -- i*73 and j*149: different odd multipliers, so the xor below does
            -- not collapse along the diagonal the march walks
            h1_hi <= unsigned(shift_left(v_i, 6) + shift_left(v_i, 3) + v_i);
            h1_hj <= unsigned(shift_left(v_j, 7) + shift_left(v_j, 4)
                            + shift_left(v_j, 2) + v_j);
            h1_di <= shift_left(resize(s_iv - s_bi, C_DW), 4) - s_oif;
            h1_dj <= shift_left(resize(s_ju - s_bj, C_DW), 4) - s_ojf;
        end if;
    end process p_h1;

    --------------------------------------------------------------------------
    -- H2: per-marched-cell hash and offset.  The KF+1 candidates are the
    -- diagonal iv+k, ju+k, so their hash bases are constant increments.
    --------------------------------------------------------------------------
    p_h2 : process(clk)
    begin
        if rising_edge(clk) then
            for k in 0 to KF loop
                h2_hx(k) <= (h1_hi + to_unsigned(73 * k, 20))
                        xor (h1_hj + to_unsigned(149 * k, 20));
                h2_di(k) <= h1_di + to_signed(16 * k, C_DW);
                h2_dj(k) <= h1_dj + to_signed(16 * k, C_DW);
            end loop;
        end if;
    end process p_h2;

    --------------------------------------------------------------------------
    -- H3: fold the hash into a wobble phase; take the absolute offsets.
    --------------------------------------------------------------------------
    p_h3 : process(clk)
        variable v_di, v_dj : t_d;
    begin
        if rising_edge(clk) then
            for k in 0 to KF loop
                h3_ph(k) <= (h2_hx(k)(9 downto 0) xor h2_hx(k)(19 downto 10)) + s_time;
                if h2_di(k) < 0 then v_di := -h2_di(k); else v_di := h2_di(k); end if;
                if h2_dj(k) < 0 then v_dj := -h2_dj(k); else v_dj := h2_dj(k); end if;
                h3_ad(k) <= unsigned(v_di(C_DW - 2 downto 0));
                h3_aj(k) <= unsigned(v_dj(C_DW - 2 downto 0));
            end loop;
        end if;
    end process p_h3;

    --------------------------------------------------------------------------
    -- H4: triangle wave (a fold, not a LUT) and the octagonal distance norm.
    --------------------------------------------------------------------------
    p_h4 : process(clk)
        variable v_n : unsigned(C_DW - 1 downto 0);
    begin
        if rising_edge(clk) then
            for k in 0 to KF loop
                if h3_ph(k)(9) = '0' then
                    h4_tri(k) <= h3_ph(k)(8 downto 4);
                else
                    h4_tri(k) <= not h3_ph(k)(8 downto 4);
                end if;
                if h3_ad(k) >= h3_aj(k) then
                    v_n := resize(h3_ad(k), C_DW) + resize(shift_right(h3_aj(k), 1), C_DW);
                else
                    v_n := resize(h3_aj(k), C_DW) + resize(shift_right(h3_ad(k), 1), C_DW);
                end if;
                if v_n(C_DW - 1 downto 11) /= 0 then
                    h4_d(k) <= (others => '1');
                else
                    h4_d(k) <= v_n(10 downto 0);
                end if;
            end loop;
        end if;
    end process p_h4;

    --------------------------------------------------------------------------
    -- H5: five-step radial gate around the ball.  The gate shift is applied to
    -- the finished height in H8, not to the amplitude here -- that keeps two
    -- wide barrel muxes per marched cell out of the design entirely, and it is
    -- exactly equivalent because the shift is linear.
    --------------------------------------------------------------------------
    p_h5 : process(clk)
        variable v_g : integer range 0 to 4;
    begin
        if rising_edge(clk) then
            for k in 0 to KF loop
                if    h4_d(k) < s_r1  then v_g := 0;
                elsif h4_d(k) < s_r2t then v_g := 1;
                elsif h4_d(k) < s_r3  then v_g := 2;
                elsif h4_d(k) < s_r4  then v_g := 3;
                else                       v_g := 4;
                end if;
                if s_farfield = '1' then v_g := 4 - v_g; end if;
                h5_gs(k)  <= to_unsigned(v_g, 3);
                h5_tri(k) <= h4_tri(k);
            end loop;
        end if;
    end process p_h5;

    --------------------------------------------------------------------------
    -- H6: amplitude * triangle as a five-term add of shifted copies.  s_amp is
    -- a per-frame register, so every shifted copy is free wiring.
    --------------------------------------------------------------------------
    p_h6 : process(clk)
        variable v_t : t_w;
    begin
        if rising_edge(clk) then
            for k in 0 to KF loop
                v_t := (others => '0');
                if h5_tri(k)(4) = '1' then v_t := v_t + shift_right(s_amp, 1); end if;
                if h5_tri(k)(3) = '1' then v_t := v_t + shift_right(s_amp, 2); end if;
                if h5_tri(k)(2) = '1' then v_t := v_t + shift_right(s_amp, 3); end if;
                h6_s1(k) <= v_t;
                v_t := s_blg;
                if h5_tri(k)(1) = '1' then v_t := v_t + shift_right(s_amp, 4); end if;
                if h5_tri(k)(0) = '1' then v_t := v_t + shift_right(s_amp, 5); end if;
                h6_s2(k) <= v_t;
                h6_gs(k) <= h5_gs(k);
            end loop;
        end if;
    end process p_h6;

    --------------------------------------------------------------------------
    -- H7: the ungated height.
    --------------------------------------------------------------------------
    p_h7 : process(clk)
    begin
        if rising_edge(clk) then
            for k in 0 to KF loop
                h7_raw(k) <= h6_s1(k) + h6_s2(k);
                h7_gs(k)  <= h6_gs(k);
            end loop;
        end if;
    end process p_h7;

    --------------------------------------------------------------------------
    -- H8: apply the radial gate, then e = aH - k*W2 -- the only thing the
    -- march needs to know about a pin.
    --------------------------------------------------------------------------
    p_h8 : process(clk)
    begin
        if rising_edge(clk) then
            for k in 0 to KF loop
                if h7_gs(k) = 4 then
                    h8_e(k) <= -s_kw2(k);
                else
                    h8_e(k) <= shift_right(h7_raw(k), to_integer(h7_gs(k))) - s_kw2(k);
                end if;
                h8_gs(k) <= h7_gs(k);
            end loop;
        end if;
    end process p_h8;

    --------------------------------------------------------------------------
    -- Delay the remainders / indices to meet the height pipeline.
    --------------------------------------------------------------------------
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            dl_ivrem(0) <= unsigned(s_ivrem(14 downto 0));
            dl_jurem(0) <= unsigned(s_jurem(14 downto 0));
            dl_iv(0)    <= s_iv;     dl_ju(0)    <= s_ju;
            for n in 1 to 7 loop
                dl_ivrem(n) <= dl_ivrem(n - 1);
                dl_jurem(n) <= dl_jurem(n - 1);
                dl_iv(n)    <= dl_iv(n - 1);
                dl_ju(n)    <= dl_ju(n - 1);
            end loop;
        end if;
    end process p_delay;

    --------------------------------------------------------------------------
    -- M1: cover test.  Both of ziggurat's half-planes reduce to a sign check.
    --------------------------------------------------------------------------
    p_m1 : process(clk)
    begin
        if rising_edge(clk) then
            for k in 0 to KF loop
                if (signed('0' & dl_ivrem(7)) + h8_e(k) >= 0)
               and (signed('0' & dl_jurem(7)) + h8_e(k) >= 0) then
                    m1_cov(k) <= '1';
                else
                    m1_cov(k) <= '0';
                end if;
                m1_e(k)  <= h8_e(k);
                m1_gs(k) <= h8_gs(k);
            end loop;
            m1_ivrem <= resize(signed('0' & dl_ivrem(7)), C_W);
            m1_jurem <= resize(signed('0' & dl_jurem(7)), C_W);
            m1_iv    <= dl_iv(7);     m1_ju    <= dl_ju(7);
        end if;
    end process p_m1;

    --------------------------------------------------------------------------
    -- M2: nearest cover wins (k = KF is nearest; k = 0 always covers, so the
    -- screen is always filled and there is no background case).
    --------------------------------------------------------------------------
    p_m2 : process(clk)
        variable v_found : std_logic;
        variable v_e : t_w;
        variable v_g : unsigned(2 downto 0);
        variable v_k : integer range 0 to KF;
    begin
        if rising_edge(clk) then
            v_found := '0';
            v_e := m1_e(0);  v_g := m1_gs(0);  v_k := 0;
            for k in KF downto 0 loop
                if v_found = '0' and m1_cov(k) = '1' then
                    v_found := '1';
                    v_e := m1_e(k);  v_g := m1_gs(k);  v_k := k;
                end if;
            end loop;
            m2_e  <= v_e;
            m2_gs <= v_g;
            m2_k  <= v_k;
            m2_ivrem <= m1_ivrem;  m2_jurem <= m1_jurem;
            m2_iv    <= m1_iv;     m2_ju    <= m1_ju;
        end if;
    end process p_m2;

    --------------------------------------------------------------------------
    -- C1: position inside the winning pin's silhouette, plus its cell.
    --------------------------------------------------------------------------
    p_c1 : process(clk)
    begin
        if rising_edge(clk) then
            c1_p   <= m2_ivrem + m2_e;
            c1_q   <= m2_jurem + m2_e;
            c1_pw  <= (m2_ivrem + m2_e) - s_W2;
            c1_qw  <= (m2_jurem + m2_e) - s_W2;
            c1_dpq <= m2_ivrem - m2_jurem;      -- e cancels: a pure horizontal axis
            c1_gs  <= m2_gs;
            c1_i  <= m2_iv + to_signed(m2_k, C_IW);
            c1_j  <= m2_ju + to_signed(m2_k, C_IW);
            c1_depth <= resize(m2_iv, C_IW + 2) + resize(m2_ju, C_IW + 2)
                      + to_signed(2 * m2_k, C_IW + 2);
        end if;
    end process p_c1;

    --------------------------------------------------------------------------
    -- C2: face select and outlines (compares only).
    --------------------------------------------------------------------------
    p_c2 : process(clk)
        variable v_line : boolean;
    begin
        if rising_edge(clk) then
            -- p,q < W2 is just the sign of p-W2 / q-W2, and p>q the sign of p-q
            if c1_pw < 0 and c1_qw < 0 then c2_top <= '1'; else c2_top <= '0'; end if;
            if c1_dpq > 0 then c2_right <= '1'; else c2_right <= '0'; end if;

            v_line := false;
            if s_edges = '1' then
                if near0(c1_p, s_thr) or near0(c1_q, s_thr)          -- back edges
                or near0(c1_pw, s_thr) or near0(c1_qw, s_thr) then   -- front edges
                    v_line := true;
                elsif (c1_pw >= 0 or c1_qw >= 0)
                  and (near0(c1_dpq, s_thr) or near0(-c1_dpq, s_thr)) then
                    v_line := true;                                  -- vertical edge
                end if;
            end if;
            if v_line then c2_line <= '1'; else c2_line <= '0'; end if;

            c2_gs    <= c1_gs;
            c2_depth <= c1_depth;
            c2_cell  <= std_logic_vector(c1_i(5 downto 0)) & std_logic_vector(c1_j(5 downto 0));
        end if;
    end process p_c2;

    --------------------------------------------------------------------------
    -- C3: colour the pin.  Video mode samples and holds the live picture once
    -- per pin; the sample is a few pixels ahead of the pin it lands on, which
    -- is invisible and saves delaying 30 bits down the whole pipeline.
    --------------------------------------------------------------------------
    p_c3 : process(clk)
        variable v_y : unsigned(9 downto 0);
        variable v_u, v_v : unsigned(9 downto 0);
        variable v_lift : unsigned(9 downto 0);
        variable v_sum : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if c2_cell /= s_prev_cell then
                s_hy <= unsigned(data_in.y);
                s_hu <= unsigned(data_in.v);       -- pre-swapped; the output swaps back
                s_hv <= unsigned(data_in.u);
            end if;
            s_prev_cell <= c2_cell;

            -- pins inside the ball's influence glow slightly, so the disturbed
            -- patch reads even when nothing is popped up
            case c2_gs is
                when "000"  => v_lift := to_unsigned(72, 10);
                when "001"  => v_lift := to_unsigned(48, 10);
                when "010"  => v_lift := to_unsigned(28, 10);
                when "011"  => v_lift := to_unsigned(12, 10);
                when others => v_lift := (others => '0');
            end case;

            if c2_line = '1' then
                v_y := to_unsigned(50, 10);
                v_u := C_MID;  v_v := C_MID;
            elsif c2_top = '1' then
                v_y := s_top_y;  v_u := s_top_u;  v_v := s_top_v;
            elsif c2_right = '1' then
                v_y := s_rgt_y;  v_u := s_sid_u;  v_v := s_sid_v;
            else
                v_y := s_lft_y;  v_u := s_sid_u;  v_v := s_sid_v;
            end if;

            if s_video = '1' and c2_line = '0' then
                -- the pin keeps its face shading, the picture supplies the colour
                v_u   := s_hu;
                v_v   := s_hv;
                v_sum := ('0' & v_y) + ('0' & s_hy);
                v_y   := v_sum(10 downto 1);
            end if;

            v_sum := ('0' & v_y) + ('0' & v_lift);
            if v_sum > 1023 then c3_y <= to_unsigned(1023, 10);
            else                 c3_y <= v_sum(9 downto 0); end if;
            c3_u <= v_u;
            c3_v <= v_v;
            c3_depth <= c2_depth;
        end if;
    end process p_c3;

    --------------------------------------------------------------------------
    -- C4: composite the ball.  It hides behind any pin nearer than its own
    -- cell (bi+bj < i+j), which is what lets a popped pin eclipse it.
    --------------------------------------------------------------------------
    p_c4 : process(clk)
        variable v_ball : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            v_ball := dl_ball(8);
            if v_ball(10) = '1' and s_bdepth >= c3_depth then
                c4_y <= v_ball(9 downto 0);
                if v_ball(11) = '1' then           -- specular: wash out to white
                    c4_u <= C_MID;
                    c4_v <= C_MID;
                else
                    c4_u <= s_bal_u;
                    c4_v <= s_bal_v;
                end if;
            else
                c4_y <= c3_y;
                c4_u <= c3_u;
                c4_v <= c3_v;
            end if;
        end if;
    end process p_c4;

    --------------------------------------------------------------------------
    -- C5 + sync.  Chroma is swapped once here for the hardware convention and
    -- everything is forced neutral outside active video.
    --------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_sync_sr(0) <= data_in.hsync_n;
            s_sync_sr(1) <= data_in.vsync_n;
            s_sync_sr(2) <= data_in.avid;
            s_sync_sr(3) <= data_in.field_n;
            for n in 4 to 4 * (C_LAT - 1) - 1 loop
                s_sync_sr(n) <= s_sync_sr(n - 4);
            end loop;
        end if;
    end process p_sync;

    p_io : process(clk)
    begin
        if rising_edge(clk) then
            if s_sync_sr(SY + 2) = '1' then
                s_io.y <= std_logic_vector(c4_y);
                s_io.u <= std_logic_vector(c4_v);       -- swap out to hardware
                s_io.v <= std_logic_vector(c4_u);
            else
                s_io.y <= (others => '0');
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            end if;
            s_io.hsync_n <= s_sync_sr(SY + 0);
            s_io.vsync_n <= s_sync_sr(SY + 1);
            s_io.avid    <= s_sync_sr(SY + 2);
            s_io.field_n <= s_sync_sr(SY + 3);
        end if;
    end process p_io;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture pinboard;
