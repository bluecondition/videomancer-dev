-- Nimbus: Flowfazer-inspired Generative Plasma
--
-- Jordan Mechner's 1989 DOS program "Flowfazer" rendered hypnotic flowing
-- plasma patterns into a small color palette, where almost all motion
-- came from cycling the palette each frame instead of recomputing
-- geometry. Nimbus revives that trick on the iCE40 HX4K.
--
-- A scalar field phi(x, y, t) is built from a sum of three sin waves
-- (a 1024-entry combinational LUT), then quantized to an 8-bit palette
-- index. The index reads three programmable BRAMs (Y, U, V) holding
-- 4 curated palette banks of 256 entries each. A per-frame DDS rotates
-- the palette read offset to produce visible motion even when the
-- scalar field is static. Per-pixel "knob x coord" interactions are
-- implemented as barrel-shifts so the plasma core uses no multipliers
-- (HX4K has no DSP blocks).
--
-- Modes (toggle_switch_8 & toggle_switch_7 = bit 1 & 0):
--   0 = Horizontal+Vertical sin sum     (classic flow)
--   1 = Diagonal interference            (rhombic grid)
--   2 = Radial rings (Manhattan)         (pulsing concentric)
--   3 = Twin sources / moire             (interfering rings)
--
-- Register map:
--   registers_in(0) = Freq A   shift amount on term-A coordinate
--   registers_in(1) = Freq B   shift amount on term-B coordinate
--   registers_in(2) = Warp     shift amount on term-C / twin-origin offset
--   registers_in(3) = Drift    palette-rotation DDS step per frame
--   registers_in(4) = Palette  top 2b = bank select; lower 8b = hue offset
--   registers_in(5) = Scale    pre-shift on sx, sy (overall density)
--   registers_in(6) = Switches  bit0=Mode_Lo bit1=Mode_Hi
--                               bit2=Source(Synth/Video) bit3=Modulate(Phase/Palette)
--                               bit4=Invert
--   registers_in(7) = Speed    master t-increment per frame
--
-- License: GPL-3.0
-- Author: ron

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture nimbus of program_top is

    -- Total cycles between data_in and data_out:
    --   1 timing_gen + 1 counters + 6 pipeline + 1 BRAM read = 9
    constant C_DELAY_CLKS : integer := 9;

    -- ====================================================================
    -- Controls (combinational view of registers_in)
    -- ====================================================================
    signal s_freq_a    : unsigned(9 downto 0);
    signal s_freq_b    : unsigned(9 downto 0);
    signal s_warp      : unsigned(9 downto 0);
    signal s_drift     : unsigned(9 downto 0);
    signal s_palette   : unsigned(9 downto 0);
    signal s_scale     : unsigned(9 downto 0);
    signal s_mode_lo   : std_logic;
    signal s_mode_hi   : std_logic;
    signal s_src_video : std_logic;
    signal s_mod_pal   : std_logic;
    signal s_invert    : std_logic;
    signal s_speed     : unsigned(9 downto 0);

    -- Vsync-latched copies (stable for the whole frame)
    signal r_freq_a_sh   : unsigned(2 downto 0) := "100";  -- 0..7 shift count
    signal r_freq_b_sh   : unsigned(2 downto 0) := "011";
    signal r_warp_sh     : unsigned(2 downto 0) := "100";
    signal r_scale_sh    : unsigned(1 downto 0) := "10";   -- 0..3 pre-shift
    signal r_pal_bank    : unsigned(1 downto 0) := "00";
    signal r_pal_hue     : unsigned(7 downto 0) := (others => '0');
    signal r_mode        : unsigned(1 downto 0) := "00";
    signal r_src_video   : std_logic := '0';
    signal r_mod_pal     : std_logic := '1';
    signal r_invert      : std_logic := '0';
    signal r_warp_offset : unsigned(11 downto 0) := (others => '0'); -- twin-origin K3

    -- Frame DDS: 16-bit accumulator, top 10 bits used as the phase t.
    -- This lets the Speed slider produce slow Mechner-style motion -- at
    -- max speed a full angle cycle takes roughly 4-8 s; at min speed it
    -- holds still. (Was 10-bit, which made motion 60x too fast.)
    signal r_frame_t_acc : unsigned(15 downto 0) := (others => '0');
    -- Palette rotation DDS: 16-bit accumulator, top 8 bits = palette
    -- rotation byte. At max Drift a full 256-entry cycle takes ~5 s.
    signal r_pal_off_acc : unsigned(15 downto 0) := (others => '0');

    -- ====================================================================
    -- Video Timing + pixel counters
    -- ====================================================================
    signal s_timing : t_video_timing_port;
    signal s_hcount : unsigned(11 downto 0) := (others => '0');
    signal s_vcount : unsigned(11 downto 0) := (others => '0');
    signal r_max_x  : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal r_max_y  : unsigned(11 downto 0) := to_unsigned(480, 12);
    signal r_cx     : unsigned(11 downto 0) := to_unsigned(360, 12);
    signal r_cy     : unsigned(11 downto 0) := to_unsigned(240, 12);
    -- Second origin for Mode 3 (twin sources). Pre-computed at vsync so
    -- the S1 plasma path doesn't have to evaluate cx + warp_offset every
    -- pixel (that adds carry-chain delay).
    signal r_cx2    : unsigned(11 downto 0) := to_unsigned(360, 12);
    signal r_cy2    : unsigned(11 downto 0) := to_unsigned(240, 12);

    -- ====================================================================
    -- Palette ROMs (3 channels x 1024 entries x 8 bits = 6 EBR)
    -- 4 banks of 256 entries each; address = {bank[1:0], idx[7:0]}.
    -- Each bank uses a cubic-smoothstep symmetric index so the 256-entry
    -- wrap is seamless (no seam when cycling).
    -- ====================================================================
    type t_pal is array (0 to 1023) of std_logic_vector(7 downto 0);

    -- Integer helpers (elaboration-time only).
    function ilerp(a, b, t, tmax : integer) return integer is
    begin
        if tmax = 0 then return a; end if;
        return a + (b - a) * t / tmax;
    end function;

    function iclamp(x : integer) return integer is
    begin
        if x < 0   then return 0;
        elsif x > 255 then return 255;
        else return x; end if;
    end function;

    -- Symmetric cubic-smoothstep mapping 0..255 -> 0..128..0 so that the
    -- palette read wraps seamlessly when the rotation DDS sweeps the
    -- 8-bit index. Identical to Perlin's fn_smooth_idx pattern.
    function fn_smooth_idx(idx : integer) return integer is
        variable t  : integer;
        variable ss : integer;
    begin
        if idx < 128 then
            t := idx;
        else
            t := 255 - idx;
        end if;
        ss := (t * t * (384 - 2 * t)) / 16384;
        ss := (ss * 255) / 128;
        return iclamp(ss);
    end function;

    -- ----- Per-bank palette stops (Y, Cb, Cr in BT.601 convention) -----
    -- Each function takes the smoothed t in 0..128 and returns the YUV
    -- channel value at that "brightness" along the bank's color curve.
    --   Bank 0 -- Aurora     (deep blue -> teal -> green -> white-green)
    --   Bank 1 -- Magma      (black -> red -> orange -> yellow -> white)
    --   Bank 2 -- Cathode    (CRT glow: dark blue -> cyan -> white)
    --   Bank 3 -- Mono       (pure greyscale palette cycling)

    function fn_y(pal, t : integer) return integer is
        variable y : integer := 0;
    begin
        case pal is
            when 0 =>  -- Aurora
                if    t < 32  then y := ilerp( 20,  50, t,      32);
                elsif t < 64  then y := ilerp( 50, 120, t-32,   32);
                elsif t < 96  then y := ilerp(120, 180, t-64,   32);
                else               y := ilerp(180, 240, t-96,   32); end if;
            when 1 =>  -- Magma
                if    t < 32  then y := ilerp(  5,  50, t,      32);
                elsif t < 64  then y := ilerp( 50, 130, t-32,   32);
                elsif t < 96  then y := ilerp(130, 200, t-64,   32);
                else               y := ilerp(200, 245, t-96,   32); end if;
            when 2 =>  -- Cathode
                if    t < 32  then y := ilerp( 10,  40, t,      32);
                elsif t < 64  then y := ilerp( 40, 120, t-32,   32);
                elsif t < 96  then y := ilerp(120, 200, t-64,   32);
                else               y := ilerp(200, 245, t-96,   32); end if;
            when others =>  -- Mono: triangular Y from t (already symmetric)
                y := t * 2;
                if y > 245 then y := 245; end if;
        end case;
        return iclamp(y);
    end function;

    function fn_u(pal, t : integer) return integer is
        variable u : integer := 128;
    begin
        case pal is
            when 0 =>  -- Aurora: starts cool-blue (high Cb), passes through
                       -- green (low Cb), ends warm-violet (mid Cb).
                if    t < 32  then u := ilerp(170, 180, t,      32);
                elsif t < 64  then u := ilerp(180, 140, t-32,   32);
                elsif t < 96  then u := ilerp(140,  70, t-64,   32);
                else               u := ilerp( 70, 120, t-96,   32); end if;
            when 1 =>  -- Magma: Cb decreases (cool->orange/red)
                if    t < 32  then u := ilerp(128, 100, t,      32);
                elsif t < 64  then u := ilerp(100,  65, t-32,   32);
                elsif t < 96  then u := ilerp( 65,  45, t-64,   32);
                else               u := ilerp( 45, 100, t-96,   32); end if;
            when 2 =>  -- Cathode: high Cb (cyan), softens to neutral
                if    t < 32  then u := ilerp(180, 175, t,      32);
                elsif t < 64  then u := ilerp(175, 165, t-32,   32);
                elsif t < 96  then u := ilerp(165, 150, t-64,   32);
                else               u := ilerp(150, 130, t-96,   32); end if;
            when others =>  -- Mono: neutral
                u := 128;
        end case;
        return iclamp(u);
    end function;

    function fn_v(pal, t : integer) return integer is
        variable v : integer := 128;
    begin
        case pal is
            when 0 =>  -- Aurora: low Cr (toward cyan), spikes for violet end
                if    t < 32  then v := ilerp(110, 140, t,      32);
                elsif t < 64  then v := ilerp(140,  85, t-32,   32);
                elsif t < 96  then v := ilerp( 85,  90, t-64,   32);
                else               v := ilerp( 90, 130, t-96,   32); end if;
            when 1 =>  -- Magma: high Cr (red/orange), descends to neutral
                if    t < 32  then v := ilerp(128, 180, t,      32);
                elsif t < 64  then v := ilerp(180, 200, t-32,   32);
                elsif t < 96  then v := ilerp(200, 165, t-64,   32);
                else               v := ilerp(165, 128, t-96,   32); end if;
            when 2 =>  -- Cathode: slightly low Cr (cyan), neutralizes
                if    t < 32  then v := ilerp(120, 105, t,      32);
                elsif t < 64  then v := ilerp(105, 100, t-32,   32);
                elsif t < 96  then v := ilerp(100, 110, t-64,   32);
                else               v := ilerp(110, 128, t-96,   32); end if;
            when others =>  -- Mono: neutral
                v := 128;
        end case;
        return iclamp(v);
    end function;

    function fn_init_y return t_pal is
        variable r : t_pal;
        variable m : integer;
    begin
        for p in 0 to 3 loop
            for i in 0 to 255 loop
                m := fn_smooth_idx(i);
                r(p*256+i) := std_logic_vector(to_unsigned(fn_y(p, m), 8));
            end loop;
        end loop;
        return r;
    end function;

    function fn_init_u return t_pal is
        variable r : t_pal;
        variable m : integer;
    begin
        for p in 0 to 3 loop
            for i in 0 to 255 loop
                m := fn_smooth_idx(i);
                r(p*256+i) := std_logic_vector(to_unsigned(fn_u(p, m), 8));
            end loop;
        end loop;
        return r;
    end function;

    function fn_init_v return t_pal is
        variable r : t_pal;
        variable m : integer;
    begin
        for p in 0 to 3 loop
            for i in 0 to 255 loop
                m := fn_smooth_idx(i);
                r(p*256+i) := std_logic_vector(to_unsigned(fn_v(p, m), 8));
            end loop;
        end loop;
        return r;
    end function;

    signal s_pal_y : t_pal := fn_init_y;
    signal s_pal_u : t_pal := fn_init_u;
    signal s_pal_v : t_pal := fn_init_v;

    signal s_pal_addr : unsigned(9 downto 0) := (others => '0');
    signal r_pray     : unsigned(7 downto 0) := (others => '0');
    signal r_prau     : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal r_prav     : unsigned(7 downto 0) := to_unsigned(128, 8);

    -- ====================================================================
    -- Sin LUT instantiation (3 combinational copies, one per term)
    -- ====================================================================
    signal s_sin_a_in  : std_logic_vector(9 downto 0);
    signal s_sin_b_in  : std_logic_vector(9 downto 0);
    signal s_sin_c_in  : std_logic_vector(9 downto 0);
    signal s_sin_a_out : signed(9 downto 0);
    signal s_sin_b_out : signed(9 downto 0);
    signal s_sin_c_out : signed(9 downto 0);
    signal s_cos_unused: signed(9 downto 0);

    -- ====================================================================
    -- Pipeline registers (S1 .. S7)
    -- ====================================================================
    -- S1: shifted coordinates, derived geometry, captured input luma
    signal s1_sx     : unsigned(11 downto 0) := (others => '0');
    signal s1_sy     : unsigned(11 downto 0) := (others => '0');
    signal s1_xpy    : unsigned(12 downto 0) := (others => '0');
    signal s1_xmy    : unsigned(11 downto 0) := (others => '0');
    signal s1_rmanh  : unsigned(12 downto 0) := (others => '0');
    signal s1_r2manh : unsigned(12 downto 0) := (others => '0');
    signal s1_inluma : unsigned(9 downto 0)  := (others => '0');

    -- S2: angles (10-bit unsigned, fed into sin LUT combinationally)
    signal s2_ang_a   : unsigned(9 downto 0) := (others => '0');
    signal s2_ang_b   : unsigned(9 downto 0) := (others => '0');
    signal s2_ang_c   : unsigned(9 downto 0) := (others => '0');
    signal s2_inluma  : unsigned(9 downto 0) := (others => '0');

    -- S3: registered sin LUT outputs
    signal s3_phi_a   : signed(9 downto 0) := (others => '0');
    signal s3_phi_b   : signed(9 downto 0) := (others => '0');
    signal s3_phi_c   : signed(9 downto 0) := (others => '0');
    signal s3_inluma  : unsigned(9 downto 0) := (others => '0');

    -- S4: phi sum -> palette index byte
    signal s4_phi_u8  : unsigned(7 downto 0) := (others => '0');
    signal s4_inluma  : unsigned(9 downto 0) := (others => '0');

    -- S5: palette address (issued; BRAM read latches in next cycle)
    -- (no extra signals here: s_pal_addr is the registered address)

    -- S6: registered BRAM outputs r_pray/u/v are 8-bit.

    -- S7: 8 -> 10 bit expanded output
    signal s7_y      : unsigned(9 downto 0) := (others => '0');
    signal s7_u      : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s7_v      : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- ====================================================================
    -- Sync / data delay shift register (C_DELAY_CLKS deep)
    -- Aligns hsync/vsync/avid/field with the processed y/u/v.
    -- ====================================================================
    signal s_avid_d    : std_logic;
    signal s_hsync_n_d : std_logic;
    signal s_vsync_n_d : std_logic;
    signal s_field_n_d : std_logic;

    -- ====================================================================
    -- Local helpers
    -- ====================================================================
    function abs_diff(a, b : unsigned) return unsigned is
    begin
        if a >= b then
            return a - b;
        else
            return b - a;
        end if;
    end function;

    -- 8-position right-shift of a 13-bit unsigned value driven by a 3-bit
    -- shift count. Synthesizes to a small case-mux tree (no multiplier).
    function vshr13(v : unsigned(12 downto 0); n : unsigned(2 downto 0))
        return unsigned is
    begin
        case n is
            when "000"  => return v;
            when "001"  => return shift_right(v, 1);
            when "010"  => return shift_right(v, 2);
            when "011"  => return shift_right(v, 3);
            when "100"  => return shift_right(v, 4);
            when "101"  => return shift_right(v, 5);
            when "110"  => return shift_right(v, 6);
            when others => return shift_right(v, 7);
        end case;
    end function;

    function vshr12(v : unsigned(11 downto 0); n : unsigned(2 downto 0))
        return unsigned is
        variable r : unsigned(12 downto 0);
    begin
        r := vshr13(resize(v, 13), n);
        return r(11 downto 0);
    end function;

begin

    -- ====================================================================
    -- Register Mapping (combinational view)
    -- ====================================================================
    s_freq_a    <= unsigned(registers_in(0));
    s_freq_b    <= unsigned(registers_in(1));
    s_warp      <= unsigned(registers_in(2));
    s_drift     <= unsigned(registers_in(3));
    s_palette   <= unsigned(registers_in(4));
    s_scale     <= unsigned(registers_in(5));
    s_mode_lo   <= registers_in(6)(0);
    s_mode_hi   <= registers_in(6)(1);
    s_src_video <= registers_in(6)(2);
    s_mod_pal   <= registers_in(6)(3);
    s_invert    <= registers_in(6)(4);
    s_speed     <= unsigned(registers_in(7));

    -- ====================================================================
    -- Video Timing Generator
    -- ====================================================================
    timing_gen_inst : entity work.video_timing_generator
        port map (
            clk         => clk,
            ref_hsync_n => data_in.hsync_n,
            ref_vsync_n => data_in.vsync_n,
            ref_avid    => data_in.avid,
            timing      => s_timing
        );

    -- ====================================================================
    -- Pixel counters + per-frame parameter capture / DDS update
    -- ====================================================================
    process (clk)
        variable v_t_step      : unsigned(8 downto 0);
        variable v_pal_step    : signed(9 downto 0);
        variable v_drift_s     : signed(10 downto 0);
        variable v_pal_acc_sum : signed(16 downto 0);
        variable v_cx_calc     : unsigned(11 downto 0);
        variable v_cy_calc     : unsigned(11 downto 0);
        variable v_cx2_s       : signed(13 downto 0);
        variable v_cy2_s       : signed(13 downto 0);
        variable v_warp_off    : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then

            if s_timing.avid = '1' then
                s_hcount <= s_hcount + 1;
            end if;
            if s_timing.hsync_start = '1' then
                if s_hcount > to_unsigned(32, 12) then
                    r_max_x <= s_hcount;
                end if;
                s_hcount <= (others => '0');
                s_vcount <= s_vcount + 1;
            end if;
            if s_timing.vsync_start = '1' then
                if s_vcount > to_unsigned(32, 12) then
                    r_max_y <= s_vcount;
                end if;
                s_vcount <= (others => '0');

                -- Capture screen centre for radial modes.
                v_cx_calc := shift_right(r_max_x, 1);
                v_cy_calc := shift_right(r_max_y, 1);
                r_cx <= v_cx_calc;
                r_cy <= v_cy_calc;

                -- Pre-compute the Mode-3 twin-source second origin so the
                -- per-pixel S1 path doesn't have to redo it.
                v_warp_off := shift_left(resize(s_warp(7 downto 2), 12), 2);
                v_cx2_s := signed(resize(v_cx_calc, 14))
                         + signed(resize(v_warp_off, 14));
                v_cy2_s := signed(resize(v_cy_calc, 14))
                         - signed(resize(v_warp_off, 14));
                if v_cx2_s < 0 then
                    r_cx2 <= (others => '0');
                elsif v_cx2_s > signed('0' & '0' & r_max_x) then
                    r_cx2 <= r_max_x;
                else
                    r_cx2 <= unsigned(v_cx2_s(11 downto 0));
                end if;
                if v_cy2_s < 0 then
                    r_cy2 <= (others => '0');
                elsif v_cy2_s > signed('0' & '0' & r_max_y) then
                    r_cy2 <= r_max_y;
                else
                    r_cy2 <= unsigned(v_cy2_s(11 downto 0));
                end if;

                -- Latch knob/switch state (stable for the frame).
                r_freq_a_sh <= s_freq_a(9 downto 7);
                r_freq_b_sh <= s_freq_b(9 downto 7);
                r_warp_sh   <= s_warp  (9 downto 7);
                r_scale_sh  <= s_scale (9 downto 8);
                r_pal_bank  <= s_palette(9 downto 8);
                r_pal_hue   <= s_palette(7 downto 0);
                r_mode      <= s_mode_hi & s_mode_lo;
                r_src_video <= s_src_video;
                r_mod_pal   <= s_mod_pal;
                r_invert    <= s_invert;

                -- r_warp_offset retained for compatibility; the actual
                -- value used by the plasma is r_cx2 / r_cy2 above.
                r_warp_offset <= v_warp_off;

                -- Frame DDS step: top 9 bits of Speed slider. Step lives
                -- in the 16-bit accumulator, so visible angle phase
                -- (top 10 bits) only advances when these 9 bits roll over.
                -- Max step 511 -> one full 1024-phase cycle in ~128 frames
                -- (~2 s @ 60 Hz). Zero step holds the plasma still.
                v_t_step := s_speed(9 downto 1);
                r_frame_t_acc <= r_frame_t_acc + resize(v_t_step, 16);

                -- Palette rotation DDS: Drift knob is bidirectional around
                -- 512 (centre = stopped). 16-bit accumulator with signed
                -- step = (drift - 512) >> 1. Max step 256 -> full 256-entry
                -- palette cycle in ~256 frames (~4 s @ 60 Hz).
                v_drift_s := signed('0' & s_drift) - to_signed(512, 11);
                v_pal_step := resize(shift_right(v_drift_s, 1), 10);
                v_pal_acc_sum := signed('0' & r_pal_off_acc)
                               + resize(v_pal_step, 17);
                r_pal_off_acc <= unsigned(v_pal_acc_sum(15 downto 0));
            end if;
        end if;
    end process;

    -- ====================================================================
    -- Palette BRAM read (registered output one cycle after addr)
    -- ====================================================================
    process (clk)
    begin
        if rising_edge(clk) then
            r_pray <= unsigned(s_pal_y(to_integer(s_pal_addr)));
            r_prau <= unsigned(s_pal_u(to_integer(s_pal_addr)));
            r_prav <= unsigned(s_pal_v(to_integer(s_pal_addr)));
        end if;
    end process;

    -- ====================================================================
    -- Sin LUT instances (combinational; angle in -> signed sin out)
    -- ====================================================================
    sin_a_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => s_sin_a_in,
            sin_out  => s_sin_a_out,
            cos_out  => s_cos_unused
        );
    sin_b_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => s_sin_b_in,
            sin_out  => s_sin_b_out,
            cos_out  => open
        );
    sin_c_inst : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => s_sin_c_in,
            sin_out  => s_sin_c_out,
            cos_out  => open
        );

    s_sin_a_in <= std_logic_vector(s2_ang_a);
    s_sin_b_in <= std_logic_vector(s2_ang_b);
    s_sin_c_in <= std_logic_vector(s2_ang_c);

    -- ====================================================================
    -- Main processing pipeline (S1 .. S7)
    -- ====================================================================
    process (clk)
        variable v_sx, v_sy           : unsigned(11 downto 0);
        variable v_xpy                : unsigned(12 downto 0);
        variable v_xmy                : unsigned(11 downto 0);
        variable v_dx, v_dy           : unsigned(11 downto 0);
        variable v_rmanh              : unsigned(12 downto 0);
        variable v_dx2, v_dy2         : unsigned(11 downto 0);
        variable v_r2manh             : unsigned(12 downto 0);
        variable v_pre_sx, v_pre_sy   : unsigned(11 downto 0);

        variable v_term_a             : unsigned(11 downto 0);
        variable v_term_b             : unsigned(11 downto 0);
        variable v_term_c             : unsigned(11 downto 0);
        variable v_xor_sxy            : unsigned(11 downto 0);
        variable v_phase_mod          : signed(10 downto 0);
        variable v_ang_a              : unsigned(10 downto 0);
        variable v_ang_b              : unsigned(10 downto 0);
        variable v_ang_c              : unsigned(10 downto 0);

        variable v_phi_sum            : signed(11 downto 0);
        variable v_phi_u8             : unsigned(7 downto 0);

        variable v_pal_idx            : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then

            ----------------------------------------------------------------
            -- STAGE 1: scaled coordinates + Manhattan radii + capture luma
            ----------------------------------------------------------------
            -- Global pre-scale: 0..3 right-shift on raw hcount/vcount.
            case r_scale_sh is
                when "00"  => v_pre_sx := s_hcount;
                                v_pre_sy := s_vcount;
                when "01"  => v_pre_sx := shift_right(s_hcount, 1);
                                v_pre_sy := shift_right(s_vcount, 1);
                when "10"  => v_pre_sx := shift_right(s_hcount, 2);
                                v_pre_sy := shift_right(s_vcount, 2);
                when others => v_pre_sx := shift_right(s_hcount, 3);
                                v_pre_sy := shift_right(s_vcount, 3);
            end case;

            v_xpy := resize(s_hcount, 13) + resize(s_vcount, 13);
            v_xmy := abs_diff(s_hcount, s_vcount);
            v_dx  := abs_diff(s_hcount, r_cx);
            v_dy  := abs_diff(s_vcount, r_cy);
            v_rmanh := resize(v_dx, 13) + resize(v_dy, 13);

            -- Mode-3 twin-source second origin (r_cx2, r_cy2) is
            -- pre-computed at vsync to keep this stage short.
            v_dx2 := abs_diff(s_hcount, r_cx2);
            v_dy2 := abs_diff(s_vcount, r_cy2);
            v_r2manh := resize(v_dx2, 13) + resize(v_dy2, 13);

            s1_sx     <= v_pre_sx;
            s1_sy     <= v_pre_sy;
            s1_xpy    <= v_xpy;
            s1_xmy    <= v_xmy;
            s1_rmanh  <= v_rmanh;
            s1_r2manh <= v_r2manh;
            s1_inluma <= unsigned(data_in.y);

            ----------------------------------------------------------------
            -- STAGE 2: per-mode angle generation (shift + add only)
            ----------------------------------------------------------------
            -- Build the three "term" coordinates per mode, then shift each
            -- by its respective freq knob and add a phase derived from
            -- r_frame_t. XOR of sx, sy provides a cheap third term across
            -- all modes (rotational symmetry breaker).
            v_xor_sxy := s1_sx xor s1_sy;

            case r_mode is
                when "00" =>  -- Mode 0: Horizontal + Vertical sin sum
                    v_term_a := s1_sx;
                    v_term_b := s1_sy;
                    v_term_c := v_xor_sxy;
                when "01" =>  -- Mode 1: Diagonal interference
                    v_term_a := s1_xpy(12 downto 1);  -- xpy/2 in 12b
                    v_term_b := s1_xmy;
                    v_term_c := v_xor_sxy;
                when "10" =>  -- Mode 2: Radial rings
                    v_term_a := s1_rmanh(12 downto 1);
                    v_term_b := s1_sx;
                    v_term_c := s1_sy;
                when others => -- Mode 3: Twin sources / moire
                    v_term_a := s1_rmanh(12 downto 1);
                    v_term_b := s1_r2manh(12 downto 1);
                    v_term_c := v_xor_sxy;
            end case;

            -- Shift each term by its frequency knob (0..7 right-shift).
            -- Then mod-1024 add a phase from the upper 10 bits of the
            -- 16-bit frame accumulator (with different multipliers per
            -- term to keep them desynced).
            v_ang_a := resize(vshr12(v_term_a, r_freq_a_sh), 11)
                     + resize(r_frame_t_acc(15 downto 6), 11);
            v_ang_b := resize(vshr12(v_term_b, r_freq_b_sh), 11)
                     + resize(r_frame_t_acc(15 downto 6) & '0', 11);  -- 2t
            v_ang_c := resize(vshr12(v_term_c, r_warp_sh), 11)
                     + resize(r_frame_t_acc(15 downto 6), 11);

            -- Input-luma modulation: if T9=Video and T10=Phase, perturb
            -- the A-angle by (input_y - 512) >> 4 = +/- 32 range. Small
            -- enough to be subtle, large enough to be visible.
            if (r_src_video = '1') and (r_mod_pal = '0') then
                v_phase_mod := signed('0' & s1_inluma) - to_signed(512, 11);
                v_ang_a := v_ang_a
                         + unsigned(resize(shift_right(v_phase_mod, 4), 11));
            end if;

            s2_ang_a  <= v_ang_a(9 downto 0);
            s2_ang_b  <= v_ang_b(9 downto 0);
            s2_ang_c  <= v_ang_c(9 downto 0);
            s2_inluma <= s1_inluma;

            ----------------------------------------------------------------
            -- STAGE 3: register sin LUT outputs
            ----------------------------------------------------------------
            s3_phi_a  <= s_sin_a_out;
            s3_phi_b  <= s_sin_b_out;
            s3_phi_c  <= s_sin_c_out;
            s3_inluma <= s2_inluma;

            ----------------------------------------------------------------
            -- STAGE 4: phi sum, quantize to 8-bit palette byte, invert
            ----------------------------------------------------------------
            v_phi_sum := resize(s3_phi_a, 12)
                       + resize(s3_phi_b, 12)
                       + resize(s3_phi_c, 12);
            -- Sum range about [-1533, +1533] fits in 12b signed.
            -- Map signed -> unsigned 0..255 by taking bits 11..4 (which
            -- is approx sum/16 in signed) then XOR the top bit so that
            -- negative values wrap into the lower palette half and
            -- positive into the upper half.
            v_phi_u8 := unsigned(v_phi_sum(11 downto 4)) + to_unsigned(128, 8);

            if r_invert = '1' then
                v_phi_u8 := not v_phi_u8;
            end if;

            s4_phi_u8 <= v_phi_u8;
            s4_inluma <= s3_inluma;

            ----------------------------------------------------------------
            -- STAGE 5: build palette address (issued to BRAM)
            ----------------------------------------------------------------
            -- pal_idx = phi + rotation + hue offset (mod 256).
            -- Top 8 bits of the 16-bit palette accumulator are the
            -- per-frame rotation byte (sub-LSB increments give smooth slow
            -- cycling).
            v_pal_idx := resize(s4_phi_u8, 10)
                       + resize(r_pal_off_acc(15 downto 8), 10)
                       + resize(r_pal_hue, 10);
            -- Optional: input luma adds to palette index when T9=Video
            -- and T10=Palette. Uses top 8 bits of input luma so the full
            -- 0..255 input range walks across the palette.
            if (r_src_video = '1') and (r_mod_pal = '1') then
                v_pal_idx := v_pal_idx
                           + resize(s4_inluma(9 downto 2), 10);
            end if;
            -- {bank[1:0], idx[7:0]}
            s_pal_addr <= r_pal_bank & v_pal_idx(7 downto 0);

            ----------------------------------------------------------------
            -- STAGE 6: BRAM read happens here (registered r_pray/u/v).
            ----------------------------------------------------------------

            ----------------------------------------------------------------
            -- STAGE 7: expand 8-bit palette entries to 10-bit YUV
            ----------------------------------------------------------------
            s7_y <= shift_left(resize(r_pray, 10), 2);
            s7_u <= shift_left(resize(r_prau, 10), 2);
            s7_v <= shift_left(resize(r_prav, 10), 2);

        end if;
    end process;

    -- ====================================================================
    -- Sync delay (align output sync with processed pixel data)
    -- ====================================================================
    process (clk)
        type t_sdly is array (0 to C_DELAY_CLKS-1) of std_logic;
        variable v_avid : t_sdly := (others => '0');
        variable v_hs   : t_sdly := (others => '1');
        variable v_vs   : t_sdly := (others => '1');
        variable v_fd   : t_sdly := (others => '1');
    begin
        if rising_edge(clk) then
            v_avid := data_in.avid    & v_avid(0 to C_DELAY_CLKS-2);
            v_hs   := data_in.hsync_n & v_hs  (0 to C_DELAY_CLKS-2);
            v_vs   := data_in.vsync_n & v_vs  (0 to C_DELAY_CLKS-2);
            v_fd   := data_in.field_n & v_fd  (0 to C_DELAY_CLKS-2);
            s_avid_d    <= v_avid(C_DELAY_CLKS-1);
            s_hsync_n_d <= v_hs  (C_DELAY_CLKS-1);
            s_vsync_n_d <= v_vs  (C_DELAY_CLKS-1);
            s_field_n_d <= v_fd  (C_DELAY_CLKS-1);
        end if;
    end process;

    -- ====================================================================
    -- Output assignment
    -- ====================================================================
    data_out.y       <= std_logic_vector(s7_y);
    data_out.u       <= std_logic_vector(s7_u);
    data_out.v       <= std_logic_vector(s7_v);
    data_out.avid    <= s_avid_d;
    data_out.hsync_n <= s_hsync_n_d;
    data_out.vsync_n <= s_vsync_n_d;
    data_out.field_n <= s_field_n_d;

end architecture nimbus;
