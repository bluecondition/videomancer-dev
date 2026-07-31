-- hypnos.vhd  (v0.1 -- hard-edged concentric-ring / bullseye generator)
--
-- 1960s op-art target: Vasarely / Bridget Riley / the Vertigo spiral.  For every
-- pixel we take the distance from a freely-movable centre, warp that distance,
-- and band it.  Everything interesting is in the warp and the banding.
--
--   dx,dy   = x-cx, y-cy         centre travels +/-2 screens off-frame; the
--                                visible field becomes a shallow arc, then
--                                near-parallel stripes -- that range IS the
--                                instrument, so dx/dy are NEVER clamped to raster.
--   a,b     = |dx|,|dy|
--   r       = hi + (k*lo)>>8     METRIC (K5): k=256 diamond(L1), ~105 circle(L2),
--                                0 square(Linf).  One coefficient, continuous morph.
--   r'      = Rref*(r/Rref)^p    WARP (K4): gamma on the radius.  p<1 packs rings
--             = exp2(p*(log2 r - LREF) + LREF)   toward the centre (infinite
--                                tunnel/vortex), p=1 flat, p>1 toward the edge
--                                (sphere dome).  log2 = priority-encoder + LUT,
--                                exp2 = barrel-shift + mantissa LUT.  No sqrt.
--   phi     = r' * f            f from PERIOD (K3) and ZOOM (P12), both exponential
--                                and folded to one mantissa*shift.  Z is a dolly:
--                                pushing it scales the period so rings stream past
--                                at a roughly constant rate (fly-in).
--   phi    += anim + luma       ANIM (S11) travels the rings; input luma displaces
--                                the phase (feed black -> uniform -> pure generator;
--                                feed video -> the image ripples the rings).
--   band    = frac(phi) < duty  DUTY (K6).  Hard 1-bit key, OR a soft radial
--                                gradient (S10=Gradient).  Local ring frequency
--                                (|d phi/dx|) raises the softness automatically as
--                                the pattern approaches the alias limit, so it
--                                greys out gracefully instead of shimmering.
--   out     = true black / true white   (contrast is the whole point).
--
-- CENTRE glide: an independent one-pole slew on X (S7), Y (S8) and Z (S9), applied
-- to the SMOOTHED register value so CV steps become liquid drifts too.  Glide-time,
-- anim-rate and the softness baseline are compile-time constants -- the box has no
-- register to store them in (6 knobs + 5 switches + 1 slider is the whole surface).
--
-- Renderer: streaming, no BRAM, no line buffer.  Four pixel-rate multiplies
-- (metric, warp exponent, phase, key), each isolated in its own stage; the
-- per-frame centre resolve shares one registered multiplier across vblank cycles
-- (the ziffern trap: a wide combinational product into a vsync-latched reg is
-- still STA-timed).
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture hypnos of program_top is

    constant C_LATENCY : integer := 26;   -- +4 for the catch-up pre-shift stages
    constant C_MID     : unsigned(9 downto 0) := to_unsigned(512, 10);   -- neutral chroma

    -- log/exp fixed point: Q6 in the log2 domain.  Rref = 1024 -> LREF = 10.
    constant C_UFRAC : integer := 6;
    constant C_LREF  : integer := 10;                                    -- log2(1024)
    constant C_LREFQ : integer := C_LREF * 64;                           -- 640, Q6

    constant C_DB    : integer := 14;                                    -- detent deadband

    constant C_GSH   : integer := 6;                                     -- glide one-pole shift
    constant C_IDS   : integer := 4;                                     -- input-luma phase depth
    constant C_ANIM  : integer := 1;                                     -- frac units / frame
    constant C_KW    : integer := 3;                                     -- catch-up weight reach (128<<C_KW px)

    constant C_SH_KEY  : integer := 5;    -- edge hardness as a shift (2^s gain)
    constant C_SH_GRAD : integer := 2;    -- soft/gradient baseline

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
    -- log2 mantissa ROM: round(64 * log2(1 + i/32)), i = 0..31  -> Q6
    ----------------------------------------------------------------------
    type t_logm is array(0 to 31) of unsigned(5 downto 0);
    constant C_LOGM : t_logm := (
        to_unsigned( 0,6), to_unsigned( 3,6), to_unsigned( 6,6), to_unsigned( 8,6),
        to_unsigned(11,6), to_unsigned(13,6), to_unsigned(16,6), to_unsigned(18,6),
        to_unsigned(21,6), to_unsigned(23,6), to_unsigned(25,6), to_unsigned(27,6),
        to_unsigned(29,6), to_unsigned(31,6), to_unsigned(34,6), to_unsigned(35,6),
        to_unsigned(37,6), to_unsigned(39,6), to_unsigned(41,6), to_unsigned(43,6),
        to_unsigned(45,6), to_unsigned(47,6), to_unsigned(48,6), to_unsigned(50,6),
        to_unsigned(52,6), to_unsigned(53,6), to_unsigned(55,6), to_unsigned(56,6),
        to_unsigned(58,6), to_unsigned(60,6), to_unsigned(61,6), to_unsigned(63,6));

    -- log2 (priority-encoder + mantissa LUT) and exp2 (mantissa LUT + barrel
    -- shift) are inlined and pipelined across R5/R5b and R6b/R7 respectively.

    ----------------------------------------------------------------------
    -- frame-latched controls
    ----------------------------------------------------------------------
    signal s_cx_k, s_cy_k : signed(10 downto 0) := (others => '0');      -- (Kn-512), deadbanded
    signal s_k3_per : unsigned(9 downto 0) := to_unsigned(430, 10);
    signal s_warpk  : unsigned(9 downto 0) := to_unsigned(512, 10);      -- raw K4 (warp)
    signal s_metric : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_duty_k : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_zoom_k : unsigned(9 downto 0) := (others => '0');
    signal s_glx, s_gly, s_glz : std_logic := '1';   -- GLIDE (S7, all axes) -- default on
    signal s_dyn    : std_logic := '0';              -- catch-up motion (S8)
    signal s_inv    : std_logic := '0';              -- invert figure/ground (S9)
    signal s_grad   : std_logic := '0';

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
    signal s_cx, s_cy : signed(13 downto 0) := (others => '0');          -- glided centre (px)
    signal s_cxs, s_cys : signed(21 downto 0) := (others => '0');        -- glide accum, Q8
    signal s_ex, s_ey : signed(10 downto 0) := (others => '0');          -- glide lag (target-current), px
    signal s_zs   : unsigned(15 downto 0) := (others => '0');            -- zoom glide accum, Q6
    signal s_spiral : signed(7 downto 0) := (others => '0');             -- K5 spiral (bipolar, 0=circles)
    signal s_p    : unsigned(8 downto 0) := to_unsigned(64, 9);          -- warp exponent, Q6 (p=K4/512)
    signal s_lf     : signed(15 downto 0) := to_signed(-704, 16);        -- freq log2, Q6
    signal s_flat : std_logic := '1';                                    -- warp in detent
    signal s_fnum : unsigned(10 downto 0) := to_unsigned(1024, 11);      -- freq mantissa
    signal s_sft  : integer range 0 to 15 := 8;                          -- freq shift (phi Q8)
    signal s_duty : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_shbase : integer range 0 to 7 := 5;                         -- edge hardness shift
    signal s_animph : unsigned(7 downto 0) := (others => '0');           -- travelling phase

    -- relay copies of the per-frame multiply operands.  These are frame-stable,
    -- so a 1-cycle delay is harmless; the relay lets the placer put the operand
    -- register next to its multiply instead of anchoring it at the sequencer.
    signal s_spiral_d : signed(7 downto 0) := (others => '0');
    signal s_p_d    : unsigned(8 downto 0) := to_unsigned(64, 9);
    signal s_fnum_d : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_ex_d, s_ey_d : signed(10 downto 0) := (others => '0');   -- relayed glide-lag operands

    -- vblank sequencer + one shared multiplier for the centre resolve
    signal s_seq   : unsigned(4 downto 0) := (others => '0');
    signal s_prev_vsync : std_logic := '1';
    signal s_vs_pulse   : std_logic := '0';
    signal s_ma    : signed(11 downto 0) := (others => '0');
    signal s_mb    : signed(12 downto 0) := (others => '0');
    signal s_mp    : signed(24 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- pixel-coordinate accumulators.  dx per pixel, dy per active line.
    ----------------------------------------------------------------------
    signal s_dx    : signed(14 downto 0) := (others => '0');
    signal s_dyb   : signed(14 downto 0) := (others => '0');
    signal s_dyc   : signed(14 downto 0) := (others => '0');
    signal s_avid_p : std_logic := '0';
    signal s_newframe : std_logic := '1';

    ----------------------------------------------------------------------
    -- pixel pipeline
    ----------------------------------------------------------------------
    signal r1_a, r1_b : unsigned(13 downto 0) := (others => '0');
    signal r1_ctr : std_logic := '0';
    signal r1_dxs, r1_dys : std_logic := '0';                   -- signs of dx,dy (for the angle)

    signal r2_hi, r2_lo : unsigned(13 downto 0) := (others => '0');
    signal r2_ctr : std_logic := '0';
    signal r2_swap, r2_dxs, r2_dys : std_logic := '0';          -- |dx|<|dy|, signs

    -- METRIC = TRUE Euclidean circle via a CORDIC magnitude pipeline (shift-adds
    -- only -- no multiply, no sqrt).  Vectoring mode also yields the polar ANGLE
    -- for free: cordz accumulates atan(2^-i) steps -> the SPIRAL (K5, Vertigo).
    constant C_NS : integer := 4;                               -- pipeline stages (8 iters)
    constant C_CG : integer := 5;                               -- guard bits
    type t_cord is array(0 to C_NS) of signed(19 downto 0);
    type t_cang is array(0 to C_NS) of signed(12 downto 0);     -- 4096 = full circle
    type t_cb   is array(0 to C_NS) of std_logic;
    signal cordx, cordy : t_cord := (others => (others => '0'));
    signal cordz        : t_cang := (others => (others => '0'));
    signal cordctr, cordsw, corddxs, corddys : t_cb := (others => '0');

    type t_atan is array(0 to 9) of integer;                    -- atan(2^-i), 4096=circle
    constant C_ATAN : t_atan := (512, 302, 160, 81, 41, 20, 10, 5, 3, 1);

    signal ru_rc  : unsigned(14 downto 0) := (others => '0');   -- true circle r
    signal ru_ang : signed(12 downto 0) := (others => '0');     -- full polar angle 0..4096
    signal ru_ctr : std_logic := '0';

    signal rb_r      : unsigned(14 downto 0) := (others => '0');
    signal rb_spiral : signed(7 downto 0) := (others => '0');   -- angular phase twist (mod cycle)
    signal rb_ctr    : std_logic := '0';

    -- catch-up centre shift (S8): before the CORDIC, nudge dx,dy toward the
    -- moving target by a radius-weighted fraction of the glide lag, so the
    -- CORDIC measures distance from a per-ring shifted centre (inner leads).
    -- Split into four one-op stages so the weight-compute never feeds the
    -- multiply in the same cycle (that was an 18 ns path).
    signal pre1_dx, pre1_dy : signed(14 downto 0) := (others => '0');
    signal pre1_rp          : unsigned(13 downto 0) := (others => '0');  -- max(|dx|,|dy|)
    signal pre2_dx, pre2_dy : signed(14 downto 0) := (others => '0');
    signal pre2_vw          : signed(8 downto 0) := (others => '0');     -- weight Q7 (0..128)
    signal pre3_dx, pre3_dy : signed(14 downto 0) := (others => '0');
    signal pre3_ox, pre3_oy : signed(14 downto 0) := (others => '0');    -- weighted lag
    signal pre4_dx, pre4_dy : signed(14 downto 0) := (others => '0');    -- shifted coords

    -- delay the spiral twist to meet the radial phase at R10 (aligned like ctr)
    type t_sp is array(0 to 8) of signed(7 downto 0);
    signal spiral_sr : t_sp := (others => (others => '0'));

    -- log2 split across two stages: R5 priority-encodes the MSB, R5b does the
    -- normalise-shift + mantissa LUT.
    signal r5_msb : unsigned(3 downto 0) := (others => '0');
    signal r5_r   : unsigned(14 downto 0) := (others => '0');
    signal r5_ctr : std_logic := '0';
    signal r5_y   : unsigned(9 downto 0) := (others => '0');

    signal r5b_u  : unsigned(9 downto 0) := (others => '0');
    signal r5b_r  : unsigned(14 downto 0) := (others => '0');
    signal r5b_ctr: std_logic := '0';
    signal r5b_y  : unsigned(9 downto 0) := (others => '0');

    -- warp multiply p*(log2 r - LREF) split into two partial products (R6)
    -- then combined (R6a), like the metric/phase multiplies.
    signal r6_ph  : signed(17 downto 0) := (others => '0');   -- vlu * p[8:4]
    signal r6_pl  : signed(16 downto 0) := (others => '0');   -- vlu * p[3:0]
    signal r6_r   : unsigned(14 downto 0) := (others => '0');
    signal r6_ctr : std_logic := '0';
    signal r6_y   : unsigned(9 downto 0) := (others => '0');

    signal r6a_pe : signed(15 downto 0) := (others => '0');
    signal r6a_r  : unsigned(14 downto 0) := (others => '0');
    signal r6a_ctr: std_logic := '0';
    signal r6a_y  : unsigned(9 downto 0) := (others => '0');

    -- exp2 split across two stages: R6b decodes (mantissa + shift + saturate
    -- select), R7 does the shrunk barrel shift + flat bypass.
    signal r6b_m  : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal r6b_sh : integer range -10 to 4 := 0;
    signal r6b_sel: unsigned(1 downto 0) := (others => '0');   -- 0 shift, 1 zero, 2 max
    signal r6b_r  : unsigned(14 downto 0) := (others => '0');
    signal r6b_ctr: std_logic := '0';
    signal r6b_y  : unsigned(9 downto 0) := (others => '0');

    signal r7_rp  : unsigned(14 downto 0) := (others => '0');
    signal r7_ctr : std_logic := '0';
    signal r7_y   : unsigned(9 downto 0) := (others => '0');

    -- phase multiply r'*fnum split the same way on r' (wide operand).
    signal r8_php : unsigned(16 downto 0) := (others => '0');   -- r'[14:7] * fnum[10:2]
    signal r8_plp : unsigned(15 downto 0) := (others => '0');   -- r'[6:0]  * fnum[10:2]
    signal r8_ctr : std_logic := '0';
    signal r8_y   : unsigned(9 downto 0) := (others => '0');

    signal r8b_prod : unsigned(25 downto 0) := (others => '0');
    signal r8b_ctr  : std_logic := '0';
    signal r8b_y    : unsigned(9 downto 0) := (others => '0');

    signal r9_phi  : unsigned(15 downto 0) := (others => '0');
    signal r9_phip : unsigned(15 downto 0) := (others => '0');
    signal r9_ctr  : std_logic := '0';
    signal r9_y    : unsigned(9 downto 0) := (others => '0');

    signal r10_frac : unsigned(7 downto 0) := (others => '0');
    signal r10_loc  : unsigned(7 downto 0) := (others => '0');
    signal r10_ctr  : std_logic := '0';

    signal r11_m    : signed(8 downto 0) := (others => '0');
    signal r11_sh   : integer range 0 to 7 := 0;
    signal r11_ctr  : std_logic := '0';

    signal r12_key  : unsigned(7 downto 0) := (others => '0');
    signal s_out_y  : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- sync delay
    ----------------------------------------------------------------------
    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- relay the frame-constant multiply operands (placement decoupling)
    ------------------------------------------------------------------------
    p_relay : process(clk)
    begin
        if rising_edge(clk) then
            s_spiral_d <= s_spiral;
            s_p_d      <= s_p;
            s_fnum_d   <= s_fnum;
            s_ex_d     <= s_ex;
            s_ey_d     <= s_ey;
        end if;
    end process p_relay;

    ------------------------------------------------------------------------
    -- raster measurement: width from the avid run, height from active lines.
    -- Own counters snapshotted the cycle before they reset, so we never read
    -- the sync line-counter after blanking has zeroed it.
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
    -- per-frame control latch + vblank sequencer.
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_kx, v_ky : signed(10 downto 0);
        variable v_ct   : signed(24 downto 0);
        variable v_targ : signed(21 downto 0);
        variable v_wa   : integer;
        variable v_err  : integer;
        variable v_metd : integer;
        variable v_msh  : integer;
        variable v_zt   : unsigned(15 downto 0);
        variable v_zd   : signed(16 downto 0);
        variable v_wp   : signed(15 downto 0);
        variable v_ip   : integer;
        variable v_fp   : integer range 0 to 63;
        variable v_pm   : unsigned(10 downto 0);
        variable v_psh  : integer;
        variable v_pv   : unsigned(15 downto 0);
        variable v_lf   : signed(15 downto 0);
        variable v_li   : integer;
        variable v_lfr  : integer range 0 to 63;
        variable v_fsh  : integer;
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            s_mp <= s_ma * s_mb;

            if data_in.vsync_n = '0' and s_prev_vsync = '1' then
                s_vs_pulse <= '1';

                v_kx := signed('0' & registers_in(0)) - to_signed(512, 11);
                v_ky := signed('0' & registers_in(1)) - to_signed(512, 11);
                if abs(v_kx) < C_DB then v_kx := (others => '0'); end if;
                if abs(v_ky) < C_DB then v_ky := (others => '0'); end if;
                s_cx_k <= v_kx;
                s_cy_k <= v_ky;

                s_k3_per <= unsigned(registers_in(2));
                v_wa := to_integer(signed('0' & registers_in(3))) - 512;
                s_warpk <= unsigned(registers_in(3));
                if v_wa < C_DB and v_wa > -C_DB then s_flat <= '1'; else s_flat <= '0'; end if;
                s_metric <= unsigned(registers_in(4));
                s_duty_k <= unsigned(registers_in(5));
                s_zoom_k <= unsigned(registers_in(7));
                s_glx <= registers_in(6)(0);   -- S7 GLIDE arms all three axes
                s_gly <= registers_in(6)(0);
                s_glz <= registers_in(6)(0);
                s_dyn <= registers_in(6)(1);   -- S8 catch-up motion
                s_inv <= registers_in(6)(2);   -- S9 invert figure/ground
                s_grad<= registers_in(6)(3);   -- S10 output Key/Gradient

                s_fpar <= data_in.field_n;
                if data_in.field_n /= s_fpar then s_ilace <= '1';
                else                              s_ilace <= '0'; end if;

                if registers_in(6)(4) = '1' then s_animph <= s_animph + C_ANIM; end if;

                s_seq <= to_unsigned(20, 5);

            elsif s_seq /= 0 and data_in.avid = '0' then
                s_seq <= s_seq - 1;

                case to_integer(s_seq) is

                    -- centre X: cx = W/2 + (K1-512)*W/256, via the shared mult
                    when 20 => s_ma <= resize(s_cx_k, 12); s_mb <= signed('0' & s_W);
                    when 18 =>
                        v_ct := s_mp;
                        v_targ := shift_left(resize(signed('0' & s_W(11 downto 1)), 22), 8)
                                  + resize(v_ct, 22);
                        v_err := to_integer(shift_right(v_targ - s_cxs, 8));   -- lag (px)
                        if    v_err >  1023 then v_err :=  1023;
                        elsif v_err < -1023 then v_err := -1023; end if;
                        s_ex <= to_signed(v_err, 11);
                        if s_glx = '1' then s_cxs <= s_cxs + shift_right(v_targ - s_cxs, C_GSH);
                        else                s_cxs <= v_targ; end if;

                    -- centre Y
                    when 16 => s_ma <= resize(s_cy_k, 12); s_mb <= signed('0' & s_H);
                    when 14 =>
                        v_ct := s_mp;
                        v_targ := shift_left(resize(signed('0' & s_H(11 downto 1)), 22), 8)
                                  + resize(v_ct, 22);
                        v_err := to_integer(shift_right(v_targ - s_cys, 8));   -- lag (px)
                        if    v_err >  1023 then v_err :=  1023;
                        elsif v_err < -1023 then v_err := -1023; end if;
                        s_ey <= to_signed(v_err, 11);
                        if s_gly = '1' then s_cys <= s_cys + shift_right(v_targ - s_cys, C_GSH);
                        else                s_cys <= v_targ; end if;

                    -- zoom glide (Q6)
                    when 12 =>
                        v_zt := s_zoom_k & "000000";
                        if s_glz = '1' then
                            v_zd := signed('0' & v_zt) - signed('0' & s_zs);
                            s_zs <= s_zs + unsigned(resize(shift_right(v_zd, C_GSH), 16));
                        else
                            s_zs <= v_zt;
                        end if;

                    -- SPIRAL (K5): bipolar twist amount (K5-512)/4, detent = 0
                    -- (pure circles).  Positive = CW spiral, negative = CCW.
                    when 10 =>
                        v_metd := to_integer(s_metric);
                        if (v_metd - 512) < C_DB and (v_metd - 512) > -C_DB then
                            s_spiral <= (others => '0');
                        else
                            s_spiral <= to_signed((v_metd - 512) / 4, 8);
                        end if;

                    -- WARP exponent p = K4/512 (Q6), linear in the knob: 0 extreme
                    -- tunnel .. 1 flat (detent) .. 2 dome.  Just a shift -- the old
                    -- 2^((K4-512)/300) exp2 decode was a 15 ns vblank cone.
                    when 8 =>
                        if s_warpk < 8 then s_p <= to_unsigned(1, 9);
                        else                s_p <= resize(shift_right(s_warpk, 3), 9); end if;

                    -- FREQUENCY: Lf(Q6) = -704 + Period*144/256 + Zoom*64/256.
                    -- Split: (6) Lf, (5) exp2 decode -> mantissa + shift.
                    when 6 =>
                        s_lf <= to_signed(-704, 16)
                                + resize(shift_right(signed('0' & s_k3_per) * to_signed(144, 9), 8), 16)
                                + resize(shift_right(signed('0' & s_zs(15 downto 6)) * to_signed(64, 9), 8), 16);
                    when 5 =>
                        v_li  := to_integer(shift_right(s_lf, 6));
                        v_lfr := to_integer(unsigned(s_lf(5 downto 0)));
                        s_fnum <= C_MANT(v_lfr);
                        v_fsh := 2 - v_li;                  -- R9 shifts by (sft-2)
                        if    v_fsh < 2  then v_fsh := 2;
                        elsif v_fsh > 15 then v_fsh := 15; end if;
                        s_sft <= v_fsh;

                    -- DUTY 13..243 ; key hardness from output mode
                    when 4 =>
                        s_duty <= to_unsigned(13 + to_integer(s_duty_k(9 downto 2)) * 230 / 256, 8);
                        if s_grad = '1' then s_shbase <= C_SH_GRAD;
                        else                 s_shbase <= C_SH_KEY; end if;

                    -- publish the glided centre in pixels
                    when 2 =>
                        s_cx <= resize(shift_right(s_cxs, 8), 14);
                        s_cy <= resize(shift_right(s_cys, 8), 14);

                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- coordinate accumulators.  dx loads -cx at each active-line start and
    -- steps +1 per pixel; dy loads -cy at the FIRST active line of the frame
    -- (after the vblank sequencer has published cx/cy) and steps per line --
    -- by 2 on interlaced sources, with a half-step offset on field 2.
    ------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_dy0 : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            s_avid_p <= data_in.avid;

            if s_vs_pulse = '1' then
                s_newframe <= '1';
            elsif data_in.avid = '1' and s_avid_p = '0' then      -- active-line start
                if s_newframe = '1' then
                    if s_ilace = '1' and data_in.field_n = '1' then v_dy0 := resize(-s_cy, 15) + 1;
                    else                                            v_dy0 := resize(-s_cy, 15); end if;
                    s_dyc <= v_dy0;
                    if s_ilace = '1' then s_dyb <= v_dy0 + 2; else s_dyb <= v_dy0 + 1; end if;
                    s_newframe <= '0';
                else
                    s_dyc <= s_dyb;
                    if s_ilace = '1' then s_dyb <= s_dyb + 2; else s_dyb <= s_dyb + 1; end if;
                end if;
                s_dx <= resize(-s_cx, 15);
            elsif data_in.avid = '1' then
                s_dx <= s_dx + 1;
            end if;
        end if;
    end process p_acc;

    -- PRE1: proxy radius rp ~= max + min/2 (octagon metric, close to circular).
    p_pre1 : process(clk)
        variable va, vb : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            if s_dx(14) = '1'  then va := unsigned(resize(-s_dx, 14));
            else                    va := unsigned(resize( s_dx, 14)); end if;
            if s_dyc(14) = '1' then vb := unsigned(resize(-s_dyc, 14));
            else                    vb := unsigned(resize( s_dyc, 14)); end if;
            if va >= vb then pre1_rp <= va + shift_right(vb, 1);
            else             pre1_rp <= vb + shift_right(va, 1); end if;
            pre1_dx <= s_dx;
            pre1_dy <= s_dyc;
        end if;
    end process p_pre1;

    -- PRE2: weight s = clamp(128 - (rp>>C_KW), 0, 128) in Q7.
    p_pre2 : process(clk)
        variable vsw : integer;
    begin
        if rising_edge(clk) then
            vsw := 128 - to_integer(shift_right(pre1_rp, C_KW));
            if vsw < 0 then vsw := 0; end if;
            pre2_vw <= to_signed(vsw, 9);
            pre2_dx <= pre1_dx;
            pre2_dy <= pre1_dy;
        end if;
    end process p_pre2;

    -- PRE3: weighted lag offsets s*(ex,ey)>>7 (multiply only)
    p_pre3 : process(clk)
    begin
        if rising_edge(clk) then
            pre3_ox <= resize(shift_right(pre2_vw * s_ex_d, 7), 15);
            pre3_oy <= resize(shift_right(pre2_vw * s_ey_d, 7), 15);
            pre3_dx <= pre2_dx;
            pre3_dy <= pre2_dy;
        end if;
    end process p_pre3;

    -- PRE4: shift the centre toward the target (S8 on), else pass through
    p_pre4 : process(clk)
    begin
        if rising_edge(clk) then
            if s_dyn = '1' then
                pre4_dx <= pre3_dx - pre3_ox;
                pre4_dy <= pre3_dy - pre3_oy;
            else
                pre4_dx <= pre3_dx;
                pre4_dy <= pre3_dy;
            end if;
        end if;
    end process p_pre4;

    ------------------------------------------------------------------------
    -- R1: |dx|,|dy| of the (catch-up-shifted) coords, sign capture for angle
    ------------------------------------------------------------------------
    p_r1 : process(clk)
        variable va, vb : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            if pre4_dx(14) = '1' then va := unsigned(resize(-pre4_dx, 14));
            else                      va := unsigned(resize( pre4_dx, 14)); end if;
            if pre4_dy(14) = '1' then vb := unsigned(resize(-pre4_dy, 14));
            else                      vb := unsigned(resize( pre4_dy, 14)); end if;
            r1_a <= va;
            r1_b <= vb;
            if va <= 2 and vb <= 2 then r1_ctr <= '1'; else r1_ctr <= '0'; end if;
            r1_dxs <= pre4_dx(14);
            r1_dys <= pre4_dy(14);
        end if;
    end process p_r1;

    ------------------------------------------------------------------------
    -- R2: max / min
    ------------------------------------------------------------------------
    p_r2 : process(clk)
    begin
        if rising_edge(clk) then
            if r1_a >= r1_b then r2_hi <= r1_a; r2_lo <= r1_b; r2_swap <= '0';
            else                 r2_hi <= r1_b; r2_lo <= r1_a; r2_swap <= '1'; end if;
            r2_ctr <= r1_ctr;
            r2_dxs <= r1_dxs;
            r2_dys <= r1_dys;
        end if;
    end process p_r2;

    ------------------------------------------------------------------------
    -- CORDIC: |(hi,lo)| in C_NC pipelined vectoring iterations.  Each stage is
    -- two adds + a sign mux; the >>i shifts are constant (free wiring).  After
    -- convergence cordx = K * hypot(hi,lo), K ~ 1.6468.
    ------------------------------------------------------------------------
    p_cordic : process(clk)
        variable x0, y0, x1, y1 : signed(19 downto 0);
        variable z0, z1         : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            cordx(0)   <= shift_left(signed(resize(r2_hi, 20)), C_CG);   -- guard bits
            cordy(0)   <= shift_left(signed(resize(r2_lo, 20)), C_CG);
            cordz(0)   <= (others => '0');
            cordsw(0)  <= r2_swap;
            corddxs(0) <= r2_dxs;
            corddys(0) <= r2_dys;
            cordctr(0) <= r2_ctr;
            for s in 0 to C_NS - 1 loop
                -- iteration 2s: rotate toward the x-axis, accumulate the angle
                if cordy(s) >= 0 then
                    x0 := cordx(s) + shift_right(cordy(s), 2 * s);
                    y0 := cordy(s) - shift_right(cordx(s), 2 * s);
                    z0 := cordz(s) + to_signed(C_ATAN(2 * s), 13);
                else
                    x0 := cordx(s) - shift_right(cordy(s), 2 * s);
                    y0 := cordy(s) + shift_right(cordx(s), 2 * s);
                    z0 := cordz(s) - to_signed(C_ATAN(2 * s), 13);
                end if;
                -- iteration 2s+1
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
                cordctr(s + 1) <= cordctr(s);
            end loop;
        end if;
    end process p_cordic;

    ------------------------------------------------------------------------
    -- RU: undo the CORDIC gain (true circle r) and fold the CORDIC angle out of
    -- the first octant into a full polar angle (0..4096).
    ------------------------------------------------------------------------
    p_ru : process(clk)
        variable vc   : unsigned(19 downto 0);
        variable vp   : unsigned(27 downto 0);
        variable vmag : signed(12 downto 0);   -- 0..1024 = 0..90 deg
        variable vang : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- r_circle = cordx * 312 >> 14  (undo gain K=1.6468 and the 5 guard bits;
            -- one final shift keeps low-bit precision -> sub-pixel circles).
            vc := unsigned(cordx(C_NS));
            vp := resize(shift_left(resize(vc, 28), 8), 28)
                  + resize(shift_left(resize(vc, 28), 5), 28)
                  + resize(shift_left(resize(vc, 28), 4), 28)
                  + resize(shift_left(resize(vc, 28), 3), 28);
            ru_rc  <= resize(shift_right(vp, 14), 15);

            -- magnitude angle: swap folds the [0,45) octant into [0,90)
            if cordsw(C_NS) = '1' then vmag := to_signed(1024, 13) - cordz(C_NS);
            else                       vmag := cordz(C_NS); end if;
            -- quadrant from the dx,dy signs (1024=90, 2048=180, 4096=360)
            if corddxs(C_NS) = '0' and corddys(C_NS) = '0' then     vang := vmag;
            elsif corddxs(C_NS) = '1' and corddys(C_NS) = '0' then  vang := to_signed(2048, 13) - vmag;
            elsif corddxs(C_NS) = '1' and corddys(C_NS) = '1' then  vang := to_signed(2048, 13) + vmag;
            else                                                    vang := -vmag; end if;
            ru_ang <= vang;
            ru_ctr <= cordctr(C_NS);
        end if;
    end process p_ru;

    ------------------------------------------------------------------------
    -- RB: pure true circle, plus the SPIRAL twist -- phase += angle * spiral.
    -- Only the twist mod one cycle matters downstream, so keep the low 8 bits.
    ------------------------------------------------------------------------
    p_rb : process(clk)
        variable vt : signed(20 downto 0);
    begin
        if rising_edge(clk) then
            if ru_rc = 0 then rb_r <= to_unsigned(1, 15);
            else              rb_r <= ru_rc; end if;
            vt := resize(shift_right(ru_ang * s_spiral_d, 8), 21);
            rb_spiral <= resize(vt, 8);
            rb_ctr    <= ru_ctr;
        end if;
    end process p_rb;

    ------------------------------------------------------------------------
    -- delay the spiral twist to line up with the radial phase at R10
    ------------------------------------------------------------------------
    p_spiral_dl : process(clk)
    begin
        if rising_edge(clk) then
            spiral_sr(0) <= rb_spiral;
            for i in 1 to 8 loop
                spiral_sr(i) <= spiral_sr(i - 1);
            end loop;
        end if;
    end process p_spiral_dl;

    ------------------------------------------------------------------------
    -- R5: log2(r) stage 1 -- priority-encode the MSB
    ------------------------------------------------------------------------
    p_r5 : process(clk)
        variable vmsb : integer range 0 to 14;
    begin
        if rising_edge(clk) then
            vmsb := 0;
            for i in 0 to 14 loop
                if rb_r(i) = '1' then vmsb := i; end if;
            end loop;
            r5_msb <= to_unsigned(vmsb, 4);
            r5_r   <= rb_r;
            r5_ctr <= rb_ctr;
        end if;
    end process p_r5;

    ------------------------------------------------------------------------
    -- R5b: log2(r) stage 2 -- normalise-shift + mantissa correction LUT
    ------------------------------------------------------------------------
    p_r5b : process(clk)
        variable vmsb : integer range 0 to 14;
        variable vsh  : unsigned(14 downto 0);
    begin
        if rising_edge(clk) then
            vmsb := to_integer(r5_msb);
            vsh  := shift_left(r5_r, 14 - vmsb);
            r5b_u  <= to_unsigned(vmsb * 64, 10) + resize(C_LOGM(to_integer(vsh(13 downto 9))), 10);
            r5b_r  <= r5_r;
            r5b_ctr<= r5_ctr;
            r5b_y  <= r5_y;
        end if;
    end process p_r5b;

    ------------------------------------------------------------------------
    -- R6: warp exponent  pe = p*(log2 r - LREF)   (MULT 2)
    ------------------------------------------------------------------------
    p_r6 : process(clk)
        variable vlu : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            vlu := resize(signed('0' & r5b_u), 12) - to_signed(C_LREFQ, 12);
            r6_ph <= vlu * signed('0' & s_p_d(8 downto 4));
            r6_pl <= vlu * signed('0' & s_p_d(3 downto 0));
            r6_r   <= r5b_r;
            r6_ctr <= r5b_ctr;
            r6_y   <= r5b_y;
        end if;
    end process p_r6;

    ------------------------------------------------------------------------
    -- R6a: combine partial products -> pe = p*(log2 r - LREF) >> 6
    ------------------------------------------------------------------------
    p_r6a : process(clk)
    begin
        if rising_edge(clk) then
            r6a_pe  <= resize(shift_right(shift_left(resize(r6_ph, 22), 4)
                                          + resize(r6_pl, 22), 6), 16);
            r6a_r   <= r6_r;
            r6a_ctr <= r6_ctr;
            r6a_y   <= r6_y;
        end if;
    end process p_r6a;

    ------------------------------------------------------------------------
    -- R6b: exp2 decode -- mantissa lookup + shift amount + saturate select.
    -- r' clamps to [0,32767], so only shifts in [-10,4] give an in-range
    -- value; everything else selects 0 or 32767 and the R7 shifter stays small.
    ------------------------------------------------------------------------
    p_r6b : process(clk)
        variable ve  : signed(15 downto 0);
        variable vie : integer range -512 to 511;
        variable vfe : integer range 0 to 63;
        variable vsh : integer;
    begin
        if rising_edge(clk) then
            ve  := r6a_pe + to_signed(C_LREFQ, 16);
            vie := to_integer(shift_right(ve, C_UFRAC));
            vfe := to_integer(unsigned(ve(5 downto 0)));
            vsh := vie - C_LREF;
            r6b_m <= C_MANT(vfe);
            if    vsh >= 5   then r6b_sel <= "10"; r6b_sh <= 0;
            elsif vsh <= -11 then r6b_sel <= "01"; r6b_sh <= 0;
            else                  r6b_sel <= "00"; r6b_sh <= vsh; end if;
            r6b_r  <= r6a_r;
            r6b_ctr<= r6a_ctr;
            r6b_y  <= r6a_y;
        end if;
    end process p_r6b;

    ------------------------------------------------------------------------
    -- R7: r' = shifted mantissa (or 0 / 32767), or r when the warp is flat
    ------------------------------------------------------------------------
    p_r7 : process(clk)
        variable vv : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            if s_flat = '1' then
                r7_rp <= r6b_r;
            else
                case r6b_sel is
                    when "10"   => r7_rp <= to_unsigned(32767, 15);
                    when "01"   => r7_rp <= (others => '0');
                    when others =>
                        if r6b_sh >= 0 then vv := shift_left(resize(r6b_m, 16), r6b_sh);
                        else                vv := shift_right(resize(r6b_m, 16), -r6b_sh); end if;
                        r7_rp <= resize(vv(14 downto 0), 15);
                end case;
            end if;
            r7_ctr <= r6b_ctr;
            r7_y   <= r6b_y;
        end if;
    end process p_r7;

    ------------------------------------------------------------------------
    -- R8: phase magnitude  r'*fnum  as two narrow partial products (MULT 3a/3b)
    ------------------------------------------------------------------------
    p_r8 : process(clk)
    begin
        if rising_edge(clk) then
            r8_php <= r7_rp(14 downto 7) * s_fnum_d(10 downto 2);
            r8_plp <= r7_rp(6 downto 0)  * s_fnum_d(10 downto 2);
            r8_ctr <= r7_ctr;
            r8_y   <= r7_y;
        end if;
    end process p_r8;

    ------------------------------------------------------------------------
    -- R8b: combine the partial products -> r'*fnum
    ------------------------------------------------------------------------
    p_r8b : process(clk)
    begin
        if rising_edge(clk) then
            -- prod = r'*fnum[10:2] = r'*(fnum/4); R9 shifts >>(sft-2) for r'*fnum>>sft
            r8b_prod <= resize(shift_left(resize(r8_php, 26), 7) + resize(r8_plp, 26), 26);
            r8b_ctr  <= r8_ctr;
            r8b_y    <= r8_y;
        end if;
    end process p_r8b;

    ------------------------------------------------------------------------
    -- R9: phi (Q8) = product >> sft ; keep the previous pixel for local freq
    ------------------------------------------------------------------------
    p_r9 : process(clk)
        variable vph : unsigned(25 downto 0);
    begin
        if rising_edge(clk) then
            vph := shift_right(r8b_prod, s_sft - 2);
            r9_phi  <= vph(15 downto 0);
            r9_phip <= r9_phi;
            r9_ctr  <= r8b_ctr;
            r9_y    <= r8b_y;
        end if;
    end process p_r9;

    ------------------------------------------------------------------------
    -- R10: fractional phase (+ travel + spiral) and local frequency
    ------------------------------------------------------------------------
    p_r10 : process(clk)
        variable vd  : signed(16 downto 0);
        variable vld : signed(16 downto 0);
    begin
        if rising_edge(clk) then
            vd := signed('0' & r9_phi) - signed('0' & r9_phip);
            if vd(16) = '1' then vd := -vd; end if;
            if vd > 255 then r10_loc <= to_unsigned(255, 8);
            else             r10_loc <= resize(unsigned(vd(7 downto 0)), 8); end if;

            -- fractional phase + travel + spiral twist.  + duty/2 biases the centre
            -- (phi=0) to the MIDDLE of band A -> the inner ring is a solid disc.
            vld := resize(signed('0' & r9_phi(7 downto 0)), 17)
                   + resize(signed('0' & s_animph), 17)
                   + resize(spiral_sr(8), 17)
                   + resize(signed('0' & s_duty(7 downto 1)), 17);
            r10_frac <= unsigned(vld(7 downto 0));
            r10_ctr  <= r9_ctr;
        end if;
    end process p_r10;

    ------------------------------------------------------------------------
    -- R11: signed distance to the nearest band edge, adaptive gain
    ------------------------------------------------------------------------
    p_r11 : process(clk)
        variable vf     : unsigned(7 downto 0);
        variable e1, e2 : unsigned(8 downto 0);
        variable vm     : unsigned(7 downto 0);
        variable vg     : integer;
    begin
        if rising_edge(clk) then
            vf := r10_frac;
            if vf < s_duty then
                e1 := resize(vf, 9);
                e2 := resize(s_duty - vf, 9);
                if e1 < e2 then vm := e1(7 downto 0); else vm := e2(7 downto 0); end if;
                r11_m <= signed('0' & vm);
            else
                e1 := resize(vf - s_duty, 9);
                e2 := to_unsigned(256, 9) - resize(vf, 9);
                if e1 < e2 then vm := e1(7 downto 0); else vm := e2(7 downto 0); end if;
                r11_m <= -signed('0' & vm);
            end if;

            -- adaptive softening: drop the hardness shift as local frequency rises
            vg := s_shbase - to_integer(shift_right(r10_loc, 5));
            if vg < 0 then vg := 0; end if;
            r11_sh  <= vg;
            r11_ctr <= r10_ctr;
        end if;
    end process p_r11;

    ------------------------------------------------------------------------
    -- R12: key = clamp(128 + m<<sh) ; hardness is a power-of-2 shift, not a
    -- multiply.  sh large = hard 1-bit edge; sh 0 = full triangle gradient.
    -- INVERT (S9) swaps figure/ground (not = 255-key on 8 bits).
    ------------------------------------------------------------------------
    p_r12 : process(clk)
        variable vk : signed(15 downto 0);
        variable kv : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            vk := to_signed(128, 16) + shift_left(resize(r11_m, 16), r11_sh);
            if    vk < 0   then kv := (others => '0');
            elsif vk > 255 then kv := (others => '1');
            else                kv := unsigned(vk(7 downto 0)); end if;
            if s_inv = '1' then r12_key <= not kv;
            else                r12_key <= kv; end if;
        end if;
    end process p_r12;

    ------------------------------------------------------------------------
    -- R13: 8-bit key -> true-black..true-white 10-bit luma (endpoints exact)
    ------------------------------------------------------------------------
    p_out : process(clk)
    begin
        if rising_edge(clk) then
            s_out_y <= shift_left(resize(r12_key, 10), 2)
                       or resize(r12_key(7 downto 6), 10);
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

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(C_MID);
    data_out.v       <= std_logic_vector(C_MID);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture hypnos;
