-- iris.vhd  (v0.4 -- IRIS: perspective dot-tunnel eye, three-layer colour split)
--
-- THE PICTURE: a large iris circle nearly filling the screen, a dark pupil
-- anywhere inside it, and hundreds of round dots on spiral arms sweeping
-- from the pupil to the rim, growing geometrically as they go.  Drawn three
-- times (R, G, B) with a distance-growing rotational/radial split.
--
-- GEOMETRY (the whole trick): the pupil circle and the iris circle are two
-- nested circles.  Every pair of nested circles is a coaxal pencil with two
-- LIMITING POINTS a (inside the pupil) and b (outside the iris); the Moebius
-- map  w = (z-a)/(z-b)  sends BOTH circles to circles about w=0.  In the
-- w-plane the dot field is an ordinary log-polar lattice (geometric rings,
-- A dots per ring, twist = per-ring rotation); because a Moebius map sends
-- circles to circles, every ring and every dot is a TRUE CIRCLE on screen,
-- squeezed on the near side and fanned on the far side -- the cone seen in
-- perspective.  We never form w: a vectoring CORDIC gives |z-a|, arg(z-a),
-- |z-b|, arg(z-b); a per-frame log table turns the magnitudes straight into
-- ring units and the angles subtract to theta = arg w.
--   u     = (L - L_ref) / log2(g)      ring coordinate (ring k = floor u)
--   v     = theta*A + T*(k+1/2+phase)  arm coordinate (dot a = round v)
--   dot   :  S(dv) <= B(du)
--           B(du) = (1 - (e^x-1)^2/K^2) e^-x, x = du*ln g   (per-frame table)
--           S(dv) = 2(1 - cos(dv*2pi/A)) / K^2                (per-frame table)
--           -> |w_pix/w_dot - 1|^2 <= K^2 exactly: a true circle in w-space
--           K = fill*min(tanh(ln g/2), sin(pi/A)): dots kiss their neighbours
--
-- LINE-RATE GEOMETRY ENGINE: the log-polar coordinates are smooth, so they
-- are evaluated every 8 px by ONE serial CORDIC (4 steps of 3 iterations,
-- lanes a and b time-shared, 8 clocks per sample) plus one log lane, one
-- line ahead of the beam, into a 4-EBR sample buffer.  There is no
-- multiplier in the engine: the log table is pre-scaled to ring units
-- (u), theta*A comes from two small per-frame tables (hi/lo halves of the
-- angle) and the layer split from a table indexed by u.  The pixel path
-- runs a DDA between samples and does the per-pixel dot work with ONE dot
-- machine time-shared by the three layers (each layer is evaluated every
-- third pixel and held), table lookups, and an anti-alias ramp scaled by the
-- local conformal factor |z-a||z-b|/|a-b|, capped for sub-pixel dots.
-- The ring lattice is anchored at a REFERENCE pupil of 6% of the iris.
--
-- SEQUENCER: a microcoded accumulator machine (register file and program in
-- EBR) runs in blanking only (measured hblank/vblank budgets), probes the
-- engine for every log/angle reference it needs (so CORDIC gain and ROM
-- rounding cancel by construction), one serial multiplier / divider, and
-- rebuilds the five per-frame tables across the following hblanks.
--
-- Controls: K1/K2 pupil X/Y, K3 pupil size, K4 arms (6..96), K5 rings
-- (3..48), K6 twist (bipolar, dead zone), P12 split; S9 Light/Ink,
-- S10 Still/Flow, S11 Framed/Full.  S7/S8 (video dots/backdrop) are inert
-- in this build: the video halftone path did not fit the HX4K.
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture iris of program_top is

    constant C_LATENCY  : integer := 20;
    constant C_MID      : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLANK_MARGIN : integer := 40;  -- cycles kept free at the end of a blank

    -- CORDIC: 12 iterations as 4 serial steps of 3, 2 guard bits, 18-bit data
    constant C_CG : integer := 2;
    constant C_CW : integer := 19;   -- 18 overflowed at the screen corners (magnitude x CORDIC gain)

-- @@ROMS@@

    ----------------------------------------------------------------------
    -- EBR tables
    ----------------------------------------------------------------------
    type t_ram16 is array(0 to 255) of std_logic_vector(15 downto 0);
    type t_ram4  is array(0 to 255) of std_logic_vector(3 downto 0);
    type t_ram32 is array(0 to 127) of std_logic_vector(31 downto 0);
    type t_ram512 is array(0 to 511) of std_logic_vector(15 downto 0);
    type t_prog is array(0 to 1279) of std_logic_vector(15 downto 0);

    function f_init_fun return t_ram512 is
        variable r : t_ram512;
    begin
        for i in 0 to 511 loop r(i) := std_logic_vector(C_FUN(i)); end loop;
        return r;
    end function;
    function f_init_prog return t_prog is
        variable r : t_prog;
    begin
        for i in 0 to 1279 loop r(i) := std_logic_vector(C_PROG(i)); end loop;
        return r;
    end function;
    function f_init_rf return t_ram32 is
        variable r : t_ram32;
    begin
        for i in 0 to 127 loop r(i) := std_logic_vector(to_signed(C_RFINIT(i), 32)); end loop;
        return r;
    end function;

    signal t1_ram   : t_ram16 := (others => (others => '0'));             -- B/K2 (Q11 signed), 256 entries
    signal tk_ram   : t_ram16 := (others => (others => '0'));             -- twist term per ring: 2T(k+1/2)+T*ph (Q12 arm)
    signal t2_ram   : t_ram16 := (others => (others => '0'));             -- S/K2 (13b Q11) & slope(3)
    signal tm_ram   : t_ram16 := (others => (others => '0'));             -- scaled log mantissa: value(11) & slope(5)
    signal tv_ram   : t_ram16 := (others => (others => '0'));             -- theta*A: 0..127 hi part, 128..255 lo part
    signal ts_ram   : t_ram16 := (others => (others => '0'));             -- layer split vs ring
    signal sb0, sb1, sb2, sb3 : t_ram16 := (others => (others => '0'));   -- line sample buffer (64-bit words)
    signal sd_ram   : t_ram4 := (others => (others => '0'));                -- sync delay
    signal rom_fun  : t_ram512 := f_init_fun;                               -- sin / exp2 / log2 mantissa
    signal rf_mem   : t_ram32 := f_init_rf;
    signal prog_mem : t_prog := f_init_prog;

    attribute ram_style : string;
    attribute ram_style of t1_ram : signal is "block";
    attribute ram_style of tk_ram : signal is "block";
    attribute ram_style of t2_ram : signal is "block";
    attribute ram_style of tm_ram : signal is "block";
    attribute ram_style of tv_ram : signal is "block";
    attribute ram_style of ts_ram : signal is "block";
    attribute ram_style of sb0 : signal is "block";
    attribute ram_style of sb1 : signal is "block";
    attribute ram_style of sb2 : signal is "block";
    attribute ram_style of sb3 : signal is "block";
    attribute ram_style of sd_ram : signal is "block";
    attribute ram_style of rom_fun : signal is "block";
    attribute ram_style of rf_mem : signal is "block";
    attribute ram_style of prog_mem : signal is "block";

    ----------------------------------------------------------------------
    -- raster measurement / timing
    ----------------------------------------------------------------------
    signal av_r, av_q, vs_r, vs_q, fl_r, hs_r, hs_q : std_logic := '0';
    signal xcnt   : unsigned(11 downto 0) := (others => '0');
    signal act_w  : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal aline  : unsigned(11 downto 0) := (others => '0');
    signal act_h  : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal frame_act : std_logic := '0';
    signal ilace  : std_logic := '0';
    signal anc_fld : std_logic := '0';
    signal vstep_base : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal blank_cnt : unsigned(9 downto 0) := (others => '0');
    signal hbl_len   : unsigned(9 downto 0) := to_unsigned(100, 10);
    signal hcnt      : unsigned(11 downto 0) := (others => '0');
    signal hline     : unsigned(11 downto 0) := to_unsigned(800, 12);
    signal nb        : unsigned(1 downto 0) := (others => '0');
    signal in_vblank : std_logic := '0';
    signal in_vblank_q : std_logic := '0';
    signal blank_ok  : std_logic := '0';
    signal seq_req   : std_logic := '0';
    signal dcnt      : unsigned(7 downto 0) := (others => '0');
    signal sw_ink, sw_flow, sw_full : std_logic := '0';
    signal sw_mesh, sw_glow : std_logic := '0';                    -- S7 Dots/Mesh, S8 Hard/Glow
    -- local copies of the control registers: the SPI-decoded words come from the far
    -- side of the die, and the sequencer's operand mux should not reach that far
    type t_regs is array(0 to 7) of unsigned(9 downto 0);
    signal reg_q  : t_regs := (others => (others => '0'));
    signal ilace_q, fld_q : std_logic := '0';

    ----------------------------------------------------------------------
    -- per-frame constants written by the sequencer
    ----------------------------------------------------------------------
    -- The iris is FIXED and CENTRED, so the screen is normalised to make it the unit
    -- circle and the map is the disc automorphism w = (z-a)/(1 - conj(a) z).  Both lanes
    -- are then affine in the pixel coordinates and bounded, so each is a plain Q16
    -- accumulator with constant per-sample and per-line steps.
    signal c_xa0, c_ya0, c_xb0, c_yb0 : signed(19 downto 0) := (others => '0');   -- line-0 starts
    signal c_sax, c_say : signed(19 downto 0) := (others => '0');                 -- lane A: per sample, per line
    signal c_sbx, c_sby : signed(19 downto 0) := (others => '0');                 -- lane B: per sample
    signal c_sblx, c_sbly : signed(19 downto 0) := (others => '0');               -- lane B: per line
    signal c_lp     : signed(17 downto 0) := (others => '0');   -- lattice anchor (reference pupil), normalised units
    signal c_cdot   : signed(19 downto 0) := (others => '0');   -- AA normaliser (needs 19 bits: ~151k)
    signal c_up     : signed(19 downto 0) := (others => '0');   -- pupil edge in u (Q12)
    signal c_invn   : unsigned(10 downto 0) := to_unsigned(1024, 11);  -- rings per octave, normalised
    signal c_inve   : unsigned(2 downto 0) := to_unsigned(2, 3);        -- ... shift
    signal c_n      : signed(7 downto 0) := to_signed(30, 8);
    signal c_nm1    : signed(7 downto 0) := to_signed(29, 8);
    signal c_t      : signed(10 downto 0) := (others => '0');    -- twist, Q10 arm/ring
    signal c_ph     : unsigned(11 downto 0) := (others => '0');
    signal c_fade   : unsigned(7 downto 0) := (others => '0');   -- outermost ring's level (flow phase)
    signal c_fl     : unsigned(3 downto 0) := (others => '0');   -- per-frame flags: bit 0 = split is zero
    signal c_a      : unsigned(3 downto 0) := (others => '0');   -- arms mod 16 (the arm coordinate's jump at the angle cut)

    ----------------------------------------------------------------------
    -- geometry engine
    ----------------------------------------------------------------------
    signal eng_busy   : std_logic := '0';
    signal eng_oh     : std_logic_vector(7 downto 0) := (0 => '1', others => '0');  -- one-hot phase
    signal eng_k      : unsigned(8 downto 0) := (others => '0');  -- sample being loaded
    signal eng_nsamp  : unsigned(8 downto 0) := to_unsigned(91, 9);
    signal eng_last   : unsigned(8 downto 0) := to_unsigned(93, 9);   -- nsamp + 2: last period of a run
    signal eng_kl     : std_logic := '0';
    signal pend_line, pend_row0 : std_logic := '0';
    signal st_go      : std_logic := '0';                            -- run start, registered
    signal st_kind    : unsigned(1 downto 0) := (others => '0');     -- 0 line, 2 row 0
    signal nsamp_r, last_r : unsigned(8 downto 0) := to_unsigned(91, 9);
    signal row0_req   : std_logic := '0';                          -- from the CPU (LINE0)
    signal sxa, sya   : signed(19 downto 0) := (others => '0');   -- lane A position (z - a)
    signal sxb, syb   : signed(19 downto 0) := (others => '0');   -- lane B position (1 - conj(a) z)
    signal lxb, lyb   : signed(19 downto 0) := (others => '0');   -- ... its line start
    -- serial CORDIC
    -- 3-stage CORDIC pipeline: stage s holds one lane and runs iterations 4s..4s+3
    signal c0x, c0y, c1x, c1y, c2x, c2y : signed(C_CW - 1 downto 0) := (others => '0');
    signal c0z, c1z, c2z : signed(15 downto 0) := (others => '0');
    signal c0f, c1f, c2f : std_logic_vector(2 downto 0) := (others => '0');   -- sw & sx & sy
    -- one-hot iteration index, one KEPT copy per CORDIC stage: the select needs no
    -- decode and the placer can sit each copy beside its own stage
    signal cj0, cj1, cj2 : std_logic_vector(3 downto 0) := "0001";
    signal eng_oh2    : std_logic_vector(7 downto 0) := (0 => '1', others => '0');  -- phase copy for the sample buffer
    attribute keep : boolean;
    attribute keep of cj0 : signal is true;
    attribute keep of cj1 : signal is true;
    attribute keep of cj2 : signal is true;
    attribute keep of eng_oh2 : signal is true;
    signal ld_hi, ld_lo : unsigned(14 downto 0) := (others => '0'); -- prepared CORDIC load operands
    signal la_x, la_y : unsigned(14 downto 0) := (others => '0');   -- ... after abs/saturate, before the swap
    signal la_s       : std_logic_vector(1 downto 0) := (others => '0');
    signal ld_f       : std_logic_vector(2 downto 0) := (others => '0');
    signal ra_x, rb_x : unsigned(C_CW - 1 downto 0) := (others => '0');
    signal ra_z, rb_z : signed(15 downto 0) := (others => '0');
    signal ra_sw, ra_sx, ra_sy, rb_sw, rb_sx, rb_sy : std_logic := '0';
    -- angle fold results
    signal fa, fb     : unsigned(16 downto 0) := (others => '0');
    signal th         : unsigned(16 downto 0) := (others => '0');
    -- log lane (shared, phase-scheduled): normalise, table, interpolate, exponent product
    signal lg_in      : unsigned(C_CW - 1 downto 0) := (others => '0');
    signal lg_e0      : unsigned(4 downto 0) := (others => '0');  -- exponent (msb index)
    signal lg_n1      : unsigned(C_CW - 1 downto 0) := (others => '0');
    signal lg_f1      : unsigned(1 downto 0) := (others => '0');
    signal lg_e1      : unsigned(4 downto 0) := (others => '0');
    signal lg_hi2     : unsigned(7 downto 0) := (others => '0');
    signal lg_lo2     : unsigned(3 downto 0) := (others => '0');
    signal lg_e2      : unsigned(4 downto 0) := (others => '0');
    signal lg_w3      : std_logic_vector(15 downto 0) := (others => '0');
    signal lg_lo3     : unsigned(3 downto 0) := (others => '0');
    signal lg_e3      : unsigned(4 downto 0) := (others => '0');
    signal lg_v4      : unsigned(10 downto 0) := (others => '0');
    signal lg_q4      : unsigned(4 downto 0) := (others => '0');
    signal lg_lo4     : unsigned(3 downto 0) := (others => '0');
    signal lg_e4      : unsigned(4 downto 0) := (others => '0');
    signal lg_p5      : unsigned(9 downto 0) := (others => '0');
    signal lg_v5      : unsigned(10 downto 0) := (others => '0');
    signal lg_ep5     : unsigned(15 downto 0) := (others => '0');
    signal lg_out     : unsigned(16 downto 0) := (others => '0');
    signal lg_a, lg_b : unsigned(16 downto 0) := (others => '0');
    signal ea, eb     : unsigned(4 downto 0) := (others => '0');   -- lane exponents / mantissa tops
    signal ha, hb     : unsigned(7 downto 0) := (others => '0');
    -- tail
    signal e_l        : signed(17 downto 0) := (others => '0');
    signal e_ld       : signed(18 downto 0) := (others => '0');
    signal e_gs       : unsigned(17 downto 0) := (others => '0');
    signal e_rd, e_rd2 : unsigned(3 downto 0) := (others => '0');
    signal e_at, e_at2 : unsigned(5 downto 0) := (others => '0');   -- resolution fade, same pipeline as e_rd
    signal e_u        : signed(19 downto 0) := (others => '0');
    signal e_vh       : unsigned(15 downto 0) := (others => '0');
    signal e_vl       : unsigned(15 downto 0) := (others => '0');
    signal e_v        : unsigned(15 downto 0) := (others => '0');   -- arm coordinate, 4 int + 12 frac
    signal vcut       : unsigned(3 downto 0) := (others => '0');    -- arms to add so v stays continuous across the cut
    signal thq_p      : unsigned(1 downto 0) := (others => '0');    -- previous sample's angle quadrant
    signal e_sep      : signed(13 downto 0) := (others => '0');
    signal tv_ad      : unsigned(7 downto 0) := (others => '0');
    signal tv_w       : std_logic_vector(15 downto 0) := (others => '0');
    signal ts_ad      : unsigned(7 downto 0) := (others => '0');
    signal ts_w       : std_logic_vector(15 downto 0) := (others => '0');
    signal stx_val : signed(20 downto 0) := (others => '0');
    signal stx_sel : unsigned(4 downto 0) := (others => '0');
    signal stx_en  : std_logic := '0';

    ----------------------------------------------------------------------
    -- pixel path (true stages)
    ----------------------------------------------------------------------
    signal px      : unsigned(10 downto 0) := (others => '1');  -- pixel index + 1 (true 1)
    signal slot    : unsigned(1 downto 0) := (others => '0');   -- layer slot 0 R, 1 G, 2 B
    signal sq0, sq1, sq2, sq3 : std_logic_vector(15 downto 0) := (others => '0');
    signal cur_u, nxt_u : signed(19 downto 0) := (others => '0');
    signal cur_v, nxt_v : signed(15 downto 0) := (others => '0');
    signal nxt_s   : signed(13 downto 0) := (others => '0');
    -- (all modular: only u's 20 bits and v's low 12 ever reach the dot test, so
    -- the accumulators carry exactly those bits plus the three of DDA fraction)
    signal du8     : signed(19 downto 0) := (others => '0');
    signal dv8     : signed(14 downto 0) := (others => '0');
    signal acc_u   : signed(22 downto 0) := (others => '0');
    signal acc_v   : signed(14 downto 0) := (others => '0');
    signal sep6    : signed(13 downto 0) := (others => '0');
    signal rd_h1   : unsigned(3 downto 0) := (others => '0');
    -- P7
    signal ul7     : signed(19 downto 0) := (others => '0');
    signal slot7   : unsigned(1 downto 0) := (others => '0');
    signal av7     : std_logic := '0';                          -- the pixel is inside active video
    signal v7      : unsigned(11 downto 0) := (others => '0');
    -- P8
    signal ug8     : signed(19 downto 0) := (others => '0');
    signal vg8     : unsigned(11 downto 0) := (others => '0');
    signal tk_q    : std_logic_vector(15 downto 0) := (others => '0');
    -- P9
    signal kk9     : signed(7 downto 0) := (others => '0');
    signal fg9     : unsigned(11 downto 0) := (others => '0');
    signal tk9     : signed(13 downto 0) := (others => '0');
    signal vg9     : unsigned(11 downto 0) := (others => '0');
    -- P9/P10/P11: dot attenuation.  The pupil is no longer a mask over the finished
    -- picture but a ramp on the dots' own coverage, so it darkens ring by ring from the
    -- centre out -- and, because it lands before the ink/light composite, it goes WHITE
    -- in Ink just as it goes black in Light.  The outermost ring's share of the same
    -- ramp fades the ring out across its own width as Flow pushes it through the rim.
    signal pu9     : unsigned(7 downto 0) := (others => '0');
    signal pu10    : unsigned(7 downto 0) := (others => '0');
    signal att11   : unsigned(7 downto 0) := (others => '0');
    -- how far past the sampler's reach this span's geometry is: the engine samples every
    -- 8 px, so ring structure finer than that cannot be interpolated, only guessed at
    signal res6    : unsigned(7 downto 0) := (others => '0');
    -- P10
    signal ig10    : unsigned(7 downto 0) := (others => '0');
    signal jg10    : unsigned(7 downto 0) := (others => '0');
    signal lvg10   : unsigned(2 downto 0) := (others => '0');
    signal vldg10, fdg10 : std_logic := '0';
    -- P11 EBR outs
    signal t1g11, t2g11 : std_logic_vector(15 downto 0) := (others => '0');
    signal lvg11   : unsigned(2 downto 0) := (others => '0');
    -- P12
    signal bvg12   : signed(15 downto 0) := (others => '0');
    signal svg12   : unsigned(12 downto 0) := (others => '0');
    signal ssg12   : unsigned(2 downto 0) := (others => '0');
    signal lvg12   : unsigned(2 downto 0) := (others => '0');
    -- P13
    signal dfg13   : signed(16 downto 0) := (others => '0');
    -- P14
    signal bar14_g : signed(8 downto 0) := (others => '0');
    -- P15 (held per layer)
    signal al15_r, al15_g, al15_b : unsigned(7 downto 0) := (others => '0');   -- latest evaluation of each layer
    signal pv_r, pv_g, pv_b : unsigned(7 downto 0) := (others => '0');         -- ...and the one before it
    signal fr_r, fr_b : std_logic := '0';                                       -- first evaluation of the line pending
    -- P16
    signal cr16, cg16, cb16 : unsigned(7 downto 0) := (others => '0');
    -- P17 / P18
    signal yy17 : unsigned(11 downto 0) := (others => '0');
    signal r17, b17 : unsigned(7 downto 0) := (others => '0');
    signal y18 : unsigned(7 downto 0) := (others => '0');
    signal u18, v18 : signed(8 downto 0) := (others => '0');
    -- output (P19)
    signal s_out_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_out_u : unsigned(9 downto 0) := C_MID;
    signal s_out_v : unsigned(9 downto 0) := C_MID;
    signal sd_q  : std_logic_vector(3 downto 0) := "0110";
    signal sd_o  : std_logic_vector(3 downto 0) := "0110";

    -- bit carries (d(k) holds true stage k+1)
    type t_pa is array(7 to 15) of unsigned(2 downto 0);                       -- active & slot
    signal lay_d  : t_pa := (others => (others => '0'));
    type t_att is array(12 to 14) of unsigned(7 downto 0);
    signal att_d  : t_att := (others => (others => '0'));
    type t_k is array(10 to 14) of std_logic;
    signal vldg_d : t_k := (others => '0');

    ----------------------------------------------------------------------
    -- micro-sequencer
    ----------------------------------------------------------------------
    -- instruction word (16 bits): op(15:11) arg(10:0): reg = arg(6:0), imm = arg signed, jump target = arg(10:0)
    constant OP_LINE0 : integer := 0;
    constant OP_LD   : integer := 1;
    constant OP_ST   : integer := 2;
    constant OP_ADD  : integer := 3;
    constant OP_SUB  : integer := 4;
    constant OP_LDI  : integer := 5;
    constant OP_ADDI : integer := 6;
    constant OP_SHR  : integer := 7;
    constant OP_SHL  : integer := 8;
    constant OP_MUL  : integer := 9;
    constant OP_LDP  : integer := 10;
    constant OP_DIV  : integer := 11;
    constant OP_SHRV : integer := 12;
    constant OP_NEG  : integer := 13;
    constant OP_ABS  : integer := 14;
    constant OP_JMP  : integer := 15;
    constant OP_JZ   : integer := 16;
    constant OP_JN   : integer := 17;
    constant OP_JNZ  : integer := 18;
    constant OP_JP   : integer := 19;
    constant OP_TW   : integer := 21;
    constant OP_ROM  : integer := 22;
    constant OP_WAITF : integer := 23;
    constant OP_LDQ  : integer := 24;
    constant OP_SHLV : integer := 25;
    constant OP_LDF  : integer := 26;
    constant OP_LDX  : integer := 27;
    constant OP_STX  : integer := 28;
    constant OP_AND  : integer := 29;
    constant OP_CALL : integer := 30;
    constant OP_RET  : integer := 31;

    signal cs_st    : unsigned(1 downto 0) := (others => '0');
    signal pc       : unsigned(10 downto 0) := (others => '0');
    signal lnk, lnk2 : unsigned(10 downto 0) := (others => '0');   -- two-deep call stack
    signal ir       : std_logic_vector(15 downto 0) := (others => '0');
    signal d_op     : unsigned(4 downto 0) := (others => '0');          -- second (fabric) decode register
    signal d_arg    : std_logic_vector(10 downto 0) := (others => '0');
    signal sp_r     : signed(31 downto 0) := (others => '0');            -- special operand, captured early
    signal acc      : signed(31 downto 0) := (others => '0');
    signal rf_q     : std_logic_vector(31 downto 0) := (others => '0');
    signal rf_we    : std_logic := '0';
    signal rf_wa    : unsigned(6 downto 0) := (others => '0');
    signal sh_cnt   : unsigned(5 downto 0) := (others => '0');
    signal sh_busy  : std_logic := '0';
    signal m_am   : unsigned(31 downto 0) := (others => '0');
    signal m_acc  : unsigned(51 downto 0) := (others => '0');
    signal m_cnt  : unsigned(4 downto 0) := (others => '0');
    signal m_busy, m_go, m_neg : std_logic := '0';
    signal d_n    : unsigned(31 downto 0) := (others => '0');
    signal d_d    : unsigned(15 downto 0) := (others => '0');
    signal d_rem  : unsigned(16 downto 0) := (others => '0');
    signal d_cnt  : unsigned(5 downto 0) := (others => '0');
    signal d_busy, d_go : std_logic := '0';
    signal busy   : std_logic := '0';
    signal fun_addr : unsigned(8 downto 0) := (others => '0');
    signal fun_q    : std_logic_vector(15 downto 0) := (others => '0');
    signal tw_addr : unsigned(8 downto 0) := (others => '0');
    signal tw_d16  : std_logic_vector(15 downto 0) := (others => '0');
    signal tw_we   : unsigned(2 downto 0) := (others => '0');   -- 0 none, 1 T1, 2 T2, 3 TM, 4 TV, 5 TS, 6 TK
    signal alu_a, alu_b : signed(31 downto 0) := (others => '0');   -- ALU operands registered at execute, summed next clock
    signal alu_c, alu_pend, alu_and : std_logic := '0';

    function f_lzc(v : unsigned(C_CW - 1 downto 0)) return unsigned is
        variable e : unsigned(4 downto 0) := (others => '0');
    begin
        e := (others => '0');
        for i in 0 to C_CW - 1 loop
            if v(i) = '1' then e := to_unsigned(i, 5); end if;
        end loop;
        return e;
    end function;

    -- (clamped to +-256: the alpha ramp only ever looks at -128..127 of it)
    function f_bar(d : signed(16 downto 0); r : unsigned(3 downto 0)) return signed is
        variable v : signed(24 downto 0);
    begin
        v := shift_right(d & x"00", to_integer(r));
        if v > 255 then return to_signed(255, 9);
        elsif v < -256 then return to_signed(-256, 9);
        else return v(8 downto 0); end if;
    end function;

    -- alpha ramp: one constant add (the +128 bias) and one clamp (cap is always <= 255,
    -- so it subsumes the 255 ceiling).  The attenuation is NOT folded in here: b
    -- saturates at 511 over a dot's whole interior, so an offset on the ramp only
    -- shrinks the edge -- it can never dim a dot's core.  It is subtracted after the
    -- clamp instead, which carries a dot smoothly all the way to nothing.
    -- (The former sub-pixel cap is gone: the resolution fade has removed every
    -- dot below one ring per 4 px long before the cap would have engaged.)
    -- With `edge` the ramp is centred on the geometric edge, as anti-aliasing
    -- wants; without it the ramp starts AT the edge, which is what Glow wants.
    function f_alpha(b : signed(8 downto 0); vld : std_logic; edge : std_logic) return unsigned is
        variable v : signed(9 downto 0);
    begin
        if edge = '1' then v := resize(b, 10) + to_signed(128, 10);
        else               v := resize(b, 10); end if;
        if vld = '0' or v <= 0 then return to_unsigned(0, 8);
        elsif v > 255 then return to_unsigned(255, 8);
        else return unsigned(v(7 downto 0)); end if;
    end function;

    function f_comp(al : unsigned(7 downto 0); ink : std_logic) return unsigned is
    begin
        if ink = '1' then return not al; else return al; end if;
    end function;

    -- one CORDIC vectoring iteration; the shift amount base+j (j = 0..3) is
    -- muxed at the adder inputs so each pipeline stage has one add/sub set.
    procedure cordic_it(variable x, y : inout signed(C_CW - 1 downto 0); variable z : inout signed(15 downto 0);
                        constant base : in integer; constant j : in std_logic_vector(3 downto 0)) is
        variable d : std_logic;
        variable mp, mn, cp, cn : signed(C_CW - 1 downto 0);
        variable zp, zc : signed(15 downto 0);
        variable tx, ty : signed(C_CW - 1 downto 0);
        variable at : signed(15 downto 0);
    begin
        case j is
            when "0001" => tx := shift_right(x, base);     ty := shift_right(y, base);     at := to_signed(C_ATAN(base), 16);
            when "0010" => tx := shift_right(x, base + 1); ty := shift_right(y, base + 1); at := to_signed(C_ATAN(base + 1), 16);
            when "0100" => tx := shift_right(x, base + 2); ty := shift_right(y, base + 2); at := to_signed(C_ATAN(base + 2), 16);
            when others => tx := shift_right(x, base + 3); ty := shift_right(y, base + 3); at := to_signed(C_ATAN(base + 3), 16);
        end case;
        d  := y(C_CW - 1);
        mp := (others => d);  mn := (others => not d);
        cp := (others => '0'); cp(0) := d;
        cn := (others => '0'); cn(0) := not d;
        zp := (others => d);  zc := (others => '0'); zc(0) := d;
        x := x + (ty xor mp) + cp;
        y := y + (tx xor mn) + cn;
        z := z + (at xor zp) + zc;
    end procedure;

begin
