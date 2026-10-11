-- Slipframe v2.0: codec meltdown.
--
-- The picture is redrawn as a broken compressed stream. All damage follows a
-- macroblock grid and grows over a GOP (group of pictures). A keyframe then
-- snaps it clean (or the damage heals: Breathe) and the pattern reseeds.
--
-- Damage layers (all stateless hashes of {block, epoch seed} except drips):
--   * Slice slips  - per 4-block-row slice: from a random error block on a
--                    random row to the end of the slice, the row slides by d
--                    (wrapping into the previous row like a codec slice).
--                    d = hash x K2 x width, plus a per-slice drift that
--                    keeps sliding with GOP age.
--   * Corrupt blocks - Walsh "DCT" AC pattern (luma +-A, A = K4), smear (the
--                    block reads its own left column), or melt (reads back by
--                    luma >> m). Corrupt blocks also get a magenta/green chroma
--                    DC error (U and V offset equally = immune to the
--                    hardware U/V-swap parity, K5).
--   * Drips        - per block column: the line buffer stops being written,
--                    so the block's row streaks downward. Flow (S7) scrolls
--                    the frozen row +-1..2 px per line, wrapping inside its
--                    own block column: a diagonal melt. Bleed
--                    (S10) freezes only chroma: colour smears over live luma.
--   * Chroma slip  - damaged pixels read U/V K5 px further back than Y.
--   Damage grows across the GOP: block/slice/drip hashes compare their top
--   6 bits against a ramp7 that rises over the GOP (monotone: blocks join,
--   never flicker), and a second field against the P12 Mosh density.
--
-- Memory: one SINGLE-bank line buffer per channel (Y/U/V, 2048x10, 15 EBR),
-- addressed by active-pixel column. Every pixel - clean or damaged - is read
-- back from it at column (x - off) mod W, so the output video comes out of
-- EBR (raw input is only 6 FFs deep). Writes lead reads by K_COL = 5
-- columns: addresses <= x+4 hold this line, >= x+6 the previous line (or a
-- frozen row), and the one colliding offset (W - K_COL) is redirected to x.
-- The U RAM stores u(9:1) & hsync->avid parity of the row that wrote it; U/V
-- are swapped at output wherever that differs from the display row's parity
-- (drape's rule for replaying stored rows, per pixel here). Drip state =
-- 256x32 column RAM (2 EBR), pipelined read-modify-write on the last pixel
-- of each block column on block-row-start lines.
--
-- Pixel pipeline (LATENCY = 17, identical in every mode):
--   s1  input register, column x
--   s2  block decode (bx, xl, Walsh column cell), slice-slip test
--   s3  hash round 1 (block key, or the sequencer's slice key in hblank);
--       drip-state RAM read
--   s4  hash round 2; state fabric register
--   s5  hash round 3; drip frozen test, n = rows since capture
--   s6  VIDEO RAM WRITE (frozen columns skip it); hash decode; per-pixel
--       offset terms; drip-state update
--   s7  offset partial sums; flow-scrolled column offset, column base
--   s8  offset sums; drip address inside the block column
--   s9  normalise mod W
--   s10 W - off; collision flag
--   s11 read address (x - off, wrapped; frozen columns: drip address)
--   s12 RAM read; s13 fabric register
--   s14 U/V parity swap, Walsh luma, chroma DC error
--   s15 clamp; s16 Quant; s17 blanking gate
--
-- Per-line sequencer (hblank, <= 49 clocks; SD hblank is 138, sim 64):
--   slice hash (borrowed from the pixel hash pipe while avid is low), slip
--   start column, displacement and drift on one serial shift-add multiplier,
--   mod W by repeated subtraction; committed as row_* registers.
--
-- Register map:
--   registers_in(0)    K1 Block   8 / 16 / 32 / 64 px macroblocks (labelled)
--   registers_in(1)    K2 Slip    slice displacement and drift speed
--   registers_in(2)    K3 Drip    drip length
--   registers_in(3)    K4 Garble  Walsh amplitude, melt depth
--   registers_in(4)    K5 Chroma  chroma plane slip + magenta/green DC error
--   registers_in(5)    K6 GOP     keyframe interval 0.13 .. 8.5 s
--   registers_in(6)(0) S7  Drip     Freeze / Flow
--   registers_in(6)(1) S8  Keyframe Snap / Breathe
--   registers_in(6)(2) S9  Quant    coarse luma/chroma steps
--   registers_in(6)(3) S10 Bleed    Full / Color (drips freeze chroma only)
--   registers_in(6)(4) S11 Comb     odd lines of slipped rows slip one block
--                                   further
--   registers_in(7)    P12 Mosh   damage density; above 50% it also lifts
--                                 the GOP ramp floor (100% = no clean frame)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture slipframe of program_top is

    constant LATENCY   : integer := 17;
    constant K_COL     : integer := 5;   -- video RAM write leads its read
    constant C_Y_BLACK : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_UV_ZERO : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- GOP phase increment per field: GOP = 8 * 2^(6i/31) fields, 8 .. 512
    type t_u14_32 is array(0 to 31) of unsigned(13 downto 0);
    constant GOP_INC_LUT : t_u14_32 := (
        to_unsigned( 8192, 14), to_unsigned( 7164, 14), to_unsigned( 6264, 14), to_unsigned( 5478, 14),
        to_unsigned( 4790, 14), to_unsigned( 4189, 14), to_unsigned( 3663, 14), to_unsigned( 3203, 14),
        to_unsigned( 2801, 14), to_unsigned( 2449, 14), to_unsigned( 2142, 14), to_unsigned( 1873, 14),
        to_unsigned( 1638, 14), to_unsigned( 1432, 14), to_unsigned( 1252, 14), to_unsigned( 1095, 14),
        to_unsigned(  958, 14), to_unsigned(  837, 14), to_unsigned(  732, 14), to_unsigned(  640, 14),
        to_unsigned(  560, 14), to_unsigned(  490, 14), to_unsigned(  428, 14), to_unsigned(  374, 14),
        to_unsigned(  327, 14), to_unsigned(  286, 14), to_unsigned(  250, 14), to_unsigned(  219, 14),
        to_unsigned(  191, 14), to_unsigned(  167, 14), to_unsigned(  146, 14), to_unsigned(  128, 14)
    );

    ----------------------------------------------------------------------------
    -- Counters
    ----------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal prev_avid    : std_logic := '0';
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal max_x_r      : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal aline        : unsigned(10 downto 0) := (others => '0');
    signal h_cnt        : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal max_y_r      : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal saw_active   : std_logic := '0';
    signal hpar         : std_logic := '0';  -- clock parity since hsync
    signal line_par     : std_logic := '0';  -- hsync->avid parity, this line

    ----------------------------------------------------------------------------
    -- Controls + per-frame values (free-running, one op per stage)
    ----------------------------------------------------------------------------
    signal k1_r, k2_r, k3_r, k4_r, k5_r, k6_r, p12_r : unsigned(9 downto 0) := (others => '0');
    signal flow_r, breathe_r, quant_r, bleed_r, comb_r : std_logic := '0';
    signal blk_r     : unsigned(1 downto 0) := "01";            -- 8 << blk px
    signal short_r   : std_logic := '0';                         -- field <= 400 lines
    signal sv_r      : unsigned(2 downto 0) := to_unsigned(4, 3); -- vertical shift 2..6
    signal w_r       : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal wk_r      : unsigned(11 downto 0) := to_unsigned(1920 - K_COL, 12);
    signal wm1_r     : unsigned(11 downto 0) := to_unsigned(1919, 12);

    signal ph        : unsigned(15 downto 0) := (others => '0');
    signal inc_r     : unsigned(13 downto 0) := to_unsigned(1095, 14);
    signal seed      : unsigned(31 downto 0) := x"2545F491";
    signal age       : unsigned(9 downto 0) := (others => '0');
    signal ramp_r    : unsigned(6 downto 0) := (others => '0');
    signal p12hi_r   : unsigned(6 downto 0) := (others => '0');
    signal ramp7     : unsigned(6 downto 0) := (others => '0');
    signal densb7    : unsigned(6 downto 0) := (others => '0');
    signal denss7    : unsigned(6 downto 0) := (others => '0');
    signal densd10   : unsigned(9 downto 0) := (others => '0');  -- x/512
    signal msq_r     : unsigned(11 downto 0) := (others => '0'); -- (P12/16)^2

    signal sm_hi, sm_lo : unsigned(14 downto 0) := (others => '0');
    signal slipmax   : unsigned(10 downto 0) := (others => '0');  -- K2 x W / 2
    signal un_hi, un_lo : unsigned(9 downto 0) := (others => '0');
    signal unit_r    : unsigned(8 downto 0) := (others => '0');   -- drip length unit
    signal len1, len2, len3, len4 : unsigned(10 downto 0) := (others => '0');
    signal amp_r     : unsigned(7 downto 0) := (others => '0');   -- Walsh amplitude
    signal cm_r      : unsigned(8 downto 0) := (others => '0');   -- chroma DC error
    signal cs_r      : unsigned(6 downto 0) := (others => '0');   -- chroma slip px
    signal k2hi_r    : unsigned(1 downto 0) := (others => '0');
    signal bsz_r     : unsigned(6 downto 0) := to_unsigned(16, 7); -- block size px

    ----------------------------------------------------------------------------
    -- Row parameters (committed by the hblank sequencer)
    ----------------------------------------------------------------------------
    signal row_aline : unsigned(10 downto 0) := (others => '0');
    signal row_by    : unsigned(7 downto 0) := (others => '0');
    signal row_yl0   : std_logic := '0';
    signal row_yc3   : unsigned(2 downto 0) := (others => '0');
    signal row_x0    : unsigned(11 downto 0) := (others => '1');  -- slip start
    signal row_d     : unsigned(11 downto 0) := (others => '0');  -- slip, [0,W)

    signal seq_run   : std_logic := '0';
    signal seq_cnt   : unsigned(5 downto 0) := (others => '0');
    signal seq_ln    : unsigned(10 downto 0) := (others => '0');
    signal seq_by    : unsigned(7 downto 0) := (others => '0');
    signal seq_yl0   : std_logic := '0';
    signal seq_yc3   : unsigned(2 downto 0) := (others => '0');
    signal seq_key   : unsigned(15 downto 0) := (others => '0');
    signal seq_rs    : unsigned(1 downto 0) := (others => '0');
    signal seq_mode  : unsigned(1 downto 0) := (others => '0');  -- 0 none, 1 all, 2 from x0
    signal seq_v     : unsigned(3 downto 0) := (others => '0');
    signal seq_x0    : unsigned(11 downto 0) := (others => '0');
    signal seq_d     : unsigned(14 downto 0) := (others => '0');
    signal hs        : unsigned(31 downto 0) := (others => '0');
    signal mul_a     : unsigned(5 downto 0) := (others => '0');
    signal mul_b     : unsigned(17 downto 0) := (others => '0');
    signal mul_acc   : unsigned(17 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Memories (collision-free by construction: no_rw_check)
    ----------------------------------------------------------------------------
    type t_vram is array(0 to 2047) of std_logic_vector(9 downto 0);
    signal ram_y  : t_vram := (others => "0001000000");
    signal ram_u  : t_vram := (others => "1000000000");
    signal ram_v  : t_vram := (others => "1000000000");
    type t_sram is array(0 to 255) of std_logic_vector(31 downto 0);
    signal ram_st : t_sram := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of ram_y, ram_u, ram_v, ram_st : signal is true;

    ----------------------------------------------------------------------------
    -- Pixel pipeline
    ----------------------------------------------------------------------------
    signal s1_y, s1_u, s1_v : unsigned(9 downto 0) := (others => '0');
    signal s1_avid : std_logic := '0';
    signal s1_x    : unsigned(10 downto 0) := (others => '0');

    signal s2_y, s2_u, s2_v : unsigned(9 downto 0) := (others => '0');
    signal s2_avid, s2_slip, s2_last : std_logic := '0';
    signal s2_x    : unsigned(10 downto 0) := (others => '0');
    signal s2_bx   : unsigned(7 downto 0) := (others => '0');
    signal s2_xl   : unsigned(5 downto 0) := (others => '0');
    signal s2_xc3  : unsigned(2 downto 0) := (others => '0');

    signal s3_y, s3_u, s3_v : unsigned(9 downto 0) := (others => '0');
    signal s3_avid, s3_slip, s3_last : std_logic := '0';
    signal s3_x    : unsigned(10 downto 0) := (others => '0');
    signal s3_bx   : unsigned(7 downto 0) := (others => '0');
    signal s3_xl   : unsigned(5 downto 0) := (others => '0');
    signal s3_xc3  : unsigned(2 downto 0) := (others => '0');
    signal s3_h    : unsigned(31 downto 0) := (others => '0');
    signal s3_st   : std_logic_vector(31 downto 0) := (others => '0');

    signal s4_y, s4_u, s4_v : unsigned(9 downto 0) := (others => '0');
    signal s4_avid, s4_slip, s4_last : std_logic := '0';
    signal s4_x    : unsigned(10 downto 0) := (others => '0');
    signal s4_bx   : unsigned(7 downto 0) := (others => '0');
    signal s4_xl   : unsigned(5 downto 0) := (others => '0');
    signal s4_xc3  : unsigned(2 downto 0) := (others => '0');
    signal s4_h    : unsigned(31 downto 0) := (others => '0');
    signal s4_st   : std_logic_vector(31 downto 0) := (others => '0');

    signal s5_y, s5_u, s5_v : unsigned(9 downto 0) := (others => '0');
    signal s5_avid, s5_slip, s5_last : std_logic := '0';
    signal s5_x    : unsigned(10 downto 0) := (others => '0');
    signal s5_bx   : unsigned(7 downto 0) := (others => '0');
    signal s5_xl   : unsigned(5 downto 0) := (others => '0');
    signal s5_xc3  : unsigned(2 downto 0) := (others => '0');
    signal s5_h    : unsigned(31 downto 0) := (others => '0');
    signal s5_valid, s5_frz, s5_le, s5_dneg, s5_dmag : std_logic := '0';
    signal s5_len  : unsigned(1 downto 0) := (others => '0');
    signal s5_start : unsigned(10 downto 0) := (others => '0');
    signal s5_n    : unsigned(10 downto 0) := (others => '0');

    -- s6: video RAM write port + decoded damage
    signal s6_we_y, s6_we_c : std_logic := '0';
    signal s6_wa   : unsigned(10 downto 0) := (others => '0');
    signal s6_wy, s6_wu, s6_wv : std_logic_vector(9 downto 0) := (others => '0');
    signal s6_st_we : std_logic := '0';
    signal s6_st_wa : unsigned(7 downto 0) := (others => '0');
    signal s6_st_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal s6_x    : unsigned(10 downto 0) := (others => '0');
    signal s6_frz, s6_slip, s6_corrupt, s6_walsh, s6_wsign, s6_dneg : std_logic := '0';
    signal s6_e    : signed(10 downto 0) := (others => '0');
    signal s6_ym   : unsigned(7 downto 0) := (others => '0');
    signal s6_xl   : unsigned(5 downto 0) := (others => '0');
    signal s6_dslip : unsigned(11 downto 0) := (others => '0');
    signal s6_m6   : unsigned(5 downto 0) := (others => '0');   -- flow shift mod 64
    signal s6_xlr  : unsigned(5 downto 0) := (others => '0');   -- raw column offset

    -- s7..s11: read-address pipeline
    signal s7_x    : unsigned(10 downto 0) := (others => '0');
    signal s7_frz  : std_logic := '0';
    signal s7_a1, s7_a2 : unsigned(11 downto 0) := (others => '0');
    signal s7_ym   : unsigned(7 downto 0) := (others => '0');
    signal s7_t6   : unsigned(5 downto 0) := (others => '0');
    signal s7_cb   : unsigned(10 downto 0) := (others => '0');

    signal s8_x    : unsigned(10 downto 0) := (others => '0');
    signal s8_frz  : std_logic := '0';
    signal s8_oy, s8_oc : unsigned(11 downto 0) := (others => '0');
    signal s8_ad   : unsigned(10 downto 0) := (others => '0');

    signal s9_x    : unsigned(10 downto 0) := (others => '0');
    signal s9_offy, s9_offc : unsigned(11 downto 0) := (others => '0');
    signal s9_ad   : unsigned(10 downto 0) := (others => '0');
    signal s9_frz  : std_logic := '0';

    signal s10_x   : unsigned(10 downto 0) := (others => '0');
    signal s10_offy, s10_offc, s10_wmoy, s10_wmoc : unsigned(11 downto 0) := (others => '0');
    signal s10_gy, s10_gc : std_logic := '0';
    signal s10_ad  : unsigned(10 downto 0) := (others => '0');
    signal s10_fy, s10_fc : std_logic := '0';

    signal s11_ray, s11_rac : unsigned(10 downto 0) := (others => '0');

    -- Per-pixel compose flags, s7 .. s13
    type t_fx is record
        walsh : std_logic;
        wsign : std_logic;
        dmg   : std_logic;
        e     : signed(10 downto 0);
    end record;
    type t_fx_arr is array(7 to 13) of t_fx;
    signal fx : t_fx_arr := (others => ('0', '0', '0', (others => '0')));

    signal s12_ry, s12_ru, s12_rv : std_logic_vector(9 downto 0) := (others => '0');
    signal s13_ry, s13_ru, s13_rv : std_logic_vector(9 downto 0) := (others => '0');

    signal s14_y, s14_u, s14_v : signed(11 downto 0) := (others => '0');
    signal s14_dmg : std_logic := '0';
    signal s15_y, s15_u, s15_v : unsigned(9 downto 0) := (others => '0');
    signal s15_dmg : std_logic := '0';
    signal s16_y, s16_u, s16_v : unsigned(9 downto 0) := (others => '0');
    signal s17_y   : unsigned(9 downto 0) := C_Y_BLACK;
    signal s17_u, s17_v : unsigned(9 downto 0) := C_UV_ZERO;

    ----------------------------------------------------------------------------
    -- Sync delay lines (entry i = i+1 clocks old)
    ----------------------------------------------------------------------------
    type t_bit_sr is array(0 to LATENCY-1) of std_logic;
    signal s_hsync_sr : t_bit_sr := (others => '1');
    signal s_vsync_sr : t_bit_sr := (others => '1');
    signal s_field_sr : t_bit_sr := (others => '1');
    signal s_avid_sr  : t_bit_sr := (others => '0');

    function f_parity(v : unsigned) return std_logic is
        variable r : std_logic := '0';
    begin
        for i in v'range loop
            r := r xor v(i);
        end loop;
        return r;
    end function;

    function f_xorshift(s : unsigned(31 downto 0)) return unsigned is
        variable x : unsigned(31 downto 0);
    begin
        x := s xor shift_left(s, 13);
        x := x xor shift_right(x, 17);
        x := x xor shift_left(x, 5);
        return x;
    end function;

begin

    main_p : process(clk)
        variable v_ph     : unsigned(16 downto 0);
        variable v_dn     : unsigned(7 downto 0);
        variable v_ds     : unsigned(7 downto 0);
        variable v_wc     : unsigned(11 downto 0);
        variable v_x0     : unsigned(11 downto 0);
        variable v_key    : unsigned(15 downto 0);
        variable v_hx     : unsigned(31 downto 0);
        variable v_t      : unsigned(31 downto 0);
        variable v_h      : unsigned(31 downto 0);
        variable v_start  : unsigned(10 downto 0);
        variable v_e      : unsigned(10 downto 0);
        variable v_corrupt, v_hit, v_act : std_logic;
        variable v_typ    : unsigned(1 downto 0);
        variable v_dr     : unsigned(8 downto 0);
        variable v_mag    : unsigned(8 downto 0);
        variable v_lsel   : unsigned(1 downto 0);
        variable v_l      : unsigned(10 downto 0);
        variable v_base   : unsigned(10 downto 0);
        variable v_sum12  : unsigned(11 downto 0);
        variable v_eend   : unsigned(10 downto 0);
        variable v_dir    : std_logic_vector(1 downto 0);
        variable v_sub    : signed(12 downto 0);
        variable v_ra     : signed(12 downto 0);
        variable v_rb     : unsigned(11 downto 0);
        variable v_us, v_vs : unsigned(9 downto 0);
        variable v_q      : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -------------------------------------------------------------------
            -- Controls (free-running)
            -------------------------------------------------------------------
            k1_r      <= unsigned(registers_in(0)(9 downto 0));
            k2_r      <= unsigned(registers_in(1)(9 downto 0));
            k3_r      <= unsigned(registers_in(2)(9 downto 0));
            k4_r      <= unsigned(registers_in(3)(9 downto 0));
            k5_r      <= unsigned(registers_in(4)(9 downto 0));
            k6_r      <= unsigned(registers_in(5)(9 downto 0));
            p12_r     <= unsigned(registers_in(7)(9 downto 0));
            flow_r    <= registers_in(6)(0);
            breathe_r <= registers_in(6)(1);
            quant_r   <= registers_in(6)(2);
            bleed_r   <= registers_in(6)(3);
            comb_r    <= registers_in(6)(4);

            -- Labelled K1: the firmware loads the label INDEX at program
            -- load, a real 0..1023 position once the knob moves.
            if k1_r < 4 then
                blk_r <= k1_r(1 downto 0);
            else
                blk_r <= k1_r(9 downto 8);
            end if;
            if max_y_r < 400 then
                short_r <= '1';   -- interlaced SD field: halve block height
            else
                short_r <= '0';
            end if;
            if short_r = '1' then
                sv_r <= resize(blk_r, 3) + 2;
            else
                sv_r <= resize(blk_r, 3) + 3;
            end if;
            case blk_r is
                when "00"   => bsz_r <= to_unsigned(8, 7);
                when "01"   => bsz_r <= to_unsigned(16, 7);
                when "10"   => bsz_r <= to_unsigned(32, 7);
                when others => bsz_r <= to_unsigned(64, 7);
            end case;

            if max_x_r > 2048 then
                w_r <= to_unsigned(2048, 12);
            else
                w_r <= max_x_r;
            end if;
            wk_r  <= w_r - K_COL;
            wm1_r <= w_r - 1;

            inc_r <= GOP_INC_LUT(to_integer(k6_r(9 downto 5)));

            -- GOP ramp: Snap = sawtooth (keyframe at the wrap), Breathe =
            -- triangle (heals back to clean at the wrap)
            if breathe_r = '0' then
                ramp_r <= '0' & ph(15 downto 10);
            elsif ph(15) = '0' then
                ramp_r <= '0' & ph(14 downto 9);
            else
                ramp_r <= '0' & not ph(14 downto 9);
            end if;
            -- Mosh above 50% lifts the ramp floor; 100% = never clean
            if p12_r(9) = '0' then
                p12hi_r <= (others => '0');
            elsif p12_r >= 1016 then
                p12hi_r <= to_unsigned(64, 7);
            else
                p12hi_r <= '0' & p12_r(8 downto 3);
            end if;
            if p12hi_r > ramp_r then
                ramp7 <= p12hi_r;
            else
                ramp7 <= ramp_r;
            end if;

            -- Densities on a quadratic Mosh curve m2 = (P12/16)^2 / 64
            -- (0..62), so the lower half stays readable: blocks 4..48 /64,
            -- slices 12..64 /64 (linear), drips 2..126 /512 per block-row
            -- roll (a column rolls once per block row, so even 1/64 would
            -- drip almost every column each frame); 100% = every column.
            msq_r <= p12_r(9 downto 4) * p12_r(9 downto 4);
            v_dn := resize(msq_r(11 downto 6), 8) + 4;
            if v_dn > 48 then
                densb7 <= to_unsigned(48, 7);
            else
                densb7 <= v_dn(6 downto 0);
            end if;
            v_ds := resize(p12_r(9 downto 4), 8) + 12;
            if v_ds > 64 then
                denss7 <= to_unsigned(64, 7);
            else
                denss7 <= v_ds(6 downto 0);
            end if;
            if p12_r >= 1016 then
                densd10 <= to_unsigned(512, 10);
            else
                densd10 <= shift_left(resize(msq_r(11 downto 6), 10), 1) + 2;
            end if;

            -- slipmax = K2(6b) x W >> 7 (0 .. W/2), split 3b x 12b products
            sm_hi   <= resize(k2_r(9 downto 7) * w_r, 15);
            sm_lo   <= resize(k2_r(6 downto 4) * w_r, 15);
            v_sum12 := resize(shift_right(shift_left(resize(sm_hi, 19), 3)
                                          + resize(sm_lo, 19), 7), 12);
            slipmax <= v_sum12(10 downto 0);

            -- Drip length unit = K3(6b) x ramp7 >> 3 (0..504 rows)
            un_hi  <= k3_r(9 downto 7) * ramp7;
            un_lo  <= k3_r(6 downto 4) * ramp7;
            unit_r <= resize(shift_right(shift_left(resize(un_hi, 13), 3)
                                         + resize(un_lo, 13), 3), 9);
            len1 <= resize(unit_r, 11);
            len2 <= shift_left(resize(unit_r, 11), 1);
            len3 <= shift_left(resize(unit_r, 11), 1) + resize(unit_r, 11);
            len4 <= shift_left(resize(unit_r, 11), 2);

            amp_r  <= k4_r(9 downto 2);
            cm_r   <= k5_r(9 downto 1);
            cs_r   <= k5_r(9 downto 3);
            k2hi_r <= k2_r(9 downto 8);

            -------------------------------------------------------------------
            -- Counters. Rows count ACTIVE lines (advance at avid fall, reset
            -- at vsync). Per-field work fires once on the first vsync edge
            -- after active video (serration-safe).
            -------------------------------------------------------------------
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            prev_avid    <= data_in.avid;

            if data_in.avid = '1' then
                pixel_x    <= pixel_x + 1;
                saw_active <= '1';
            end if;

            hpar <= not hpar;
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then
                if pixel_x > to_unsigned(32, 12) then
                    max_x_r <= pixel_x;
                end if;
                pixel_x <= (others => '0');
                hpar    <= '0';
            end if;

            if data_in.avid = '1' and prev_avid = '0' then
                line_par <= hpar;
            end if;

            if data_in.avid = '0' and prev_avid = '1' then
                aline <= aline + 1;
                h_cnt <= aline + 1;
            end if;

            if data_in.vsync_n = '0' and prev_vsync_n = '1' then
                aline <= (others => '0');
                if h_cnt > to_unsigned(32, 11) then
                    max_y_r <= h_cnt;
                end if;
                if saw_active = '1' then
                    saw_active <= '0';
                    v_ph := resize(ph, 17) + resize(inc_r, 17);
                    ph   <= v_ph(15 downto 0);
                    if v_ph(16) = '1' then
                        -- keyframe: new epoch
                        seed <= f_xorshift(seed);
                        age  <= (others => '0');
                    elsif age /= 1023 then
                        age <= age + 1;
                    end if;
                end if;
            end if;

            -------------------------------------------------------------------
            -- Per-line sequencer: params for the NEXT line, run in hblank.
            -- Started at the avid fall (aline advances on that edge) and
            -- restarted at vsync (aline reset), committed at count 48.
            -------------------------------------------------------------------
            if (data_in.avid = '0' and prev_avid = '1')
               or (data_in.vsync_n = '0' and prev_vsync_n = '1') then
                seq_cnt <= (others => '0');
                seq_run <= '1';
            elsif seq_run = '1' then
                seq_cnt <= seq_cnt + 1;
                case to_integer(seq_cnt) is
                    when 1 =>
                        seq_ln <= aline;
                    when 2 =>
                        case to_integer(sv_r) is
                            when 2 =>
                                seq_by  <= seq_ln(9 downto 2);
                                seq_yc3 <= seq_ln(1 downto 0) & '0';
                                if seq_ln(1 downto 0) = 0 then seq_yl0 <= '1'; else seq_yl0 <= '0'; end if;
                            when 3 =>
                                seq_by  <= seq_ln(10 downto 3);
                                seq_yc3 <= seq_ln(2 downto 0);
                                if seq_ln(2 downto 0) = 0 then seq_yl0 <= '1'; else seq_yl0 <= '0'; end if;
                            when 4 =>
                                seq_by  <= '0' & seq_ln(10 downto 4);
                                seq_yc3 <= seq_ln(3 downto 1);
                                if seq_ln(3 downto 0) = 0 then seq_yl0 <= '1'; else seq_yl0 <= '0'; end if;
                            when 5 =>
                                seq_by  <= "00" & seq_ln(10 downto 5);
                                seq_yc3 <= seq_ln(4 downto 2);
                                if seq_ln(4 downto 0) = 0 then seq_yl0 <= '1'; else seq_yl0 <= '0'; end if;
                            when others =>
                                seq_by  <= "000" & seq_ln(10 downto 6);
                                seq_yc3 <= seq_ln(5 downto 3);
                                if seq_ln(5 downto 0) = 0 then seq_yl0 <= '1'; else seq_yl0 <= '0'; end if;
                        end case;
                    when 3 =>
                        -- slice key: by field 0xF0 never occurs for blocks
                        seq_key <= x"F0" & "00" & seq_by(7 downto 2);
                        seq_rs  <= seq_by(1 downto 0);
                    when 8 =>
                        hs <= s5_h xor shift_right(s5_h, 15);
                    when 9 =>
                        -- slice: corrupt from row r0 / column x0 to slice end
                        if resize(hs(31 downto 26), 7) < ramp7
                           and resize(hs(25 downto 20), 7) < denss7 then
                            if seq_rs > hs(19 downto 18) then
                                seq_mode <= "01";
                            elsif seq_rs = hs(19 downto 18) then
                                seq_mode <= "10";
                            else
                                seq_mode <= "00";
                            end if;
                        else
                            seq_mode <= "00";
                        end if;
                        -- drift speed px/field = vsel x K2 top bits (0..9)
                        case hs(4 downto 3) is
                            when "00"   => seq_v <= (others => '0');
                            when "01"   => seq_v <= resize(k2hi_r, 4);
                            when "10"   => seq_v <= shift_left(resize(k2hi_r, 4), 1);
                            when others => seq_v <= shift_left(resize(k2hi_r, 4), 1) + resize(k2hi_r, 4);
                        end case;
                        mul_a   <= hs(17 downto 12);
                        mul_b   <= resize(w_r, 18);
                        mul_acc <= (others => '0');
                    when 16 =>
                        v_x0 := mul_acc(17 downto 6);
                        case blk_r is
                            when "00"   => v_x0(2 downto 0) := (others => '0');
                            when "01"   => v_x0(3 downto 0) := (others => '0');
                            when "10"   => v_x0(4 downto 0) := (others => '0');
                            when others => v_x0(5 downto 0) := (others => '0');
                        end case;
                        seq_x0  <= v_x0;
                        mul_a   <= hs(11 downto 6);
                        mul_b   <= resize(slipmax, 18);
                        mul_acc <= (others => '0');
                    when 23 =>
                        seq_d   <= resize(mul_acc(17 downto 6), 15);
                        mul_a   <= resize(seq_v, 6);
                        mul_b   <= resize(age, 18);
                        mul_acc <= (others => '0');
                    when 30 =>
                        if comb_r = '1' and seq_ln(0) = '1' then
                            seq_d <= seq_d + resize(mul_acc, 15) + resize(bsz_r, 15);
                        else
                            seq_d <= seq_d + resize(mul_acc, 15);
                        end if;
                    when 47 =>
                        if hs(5) = '1' and seq_d /= 0 then
                            seq_d <= resize(w_r, 15) - seq_d;
                        end if;
                    when 48 =>
                        row_aline <= seq_ln;
                        row_by    <= seq_by;
                        row_yl0   <= seq_yl0;
                        row_yc3   <= seq_yc3;
                        case seq_mode is
                            when "01"   => row_x0 <= (others => '0');
                            when "10"   => row_x0 <= seq_x0;
                            when others => row_x0 <= (others => '1');
                        end case;
                        row_d   <= seq_d(11 downto 0);
                        seq_run <= '0';
                    when others =>
                        null;
                end case;
                -- shared serial multiplier (6 steps after each load)
                if (seq_cnt >= 10 and seq_cnt <= 15)
                   or (seq_cnt >= 17 and seq_cnt <= 22)
                   or (seq_cnt >= 24 and seq_cnt <= 29) then
                    if mul_a(0) = '1' then
                        mul_acc <= mul_acc + mul_b;
                    end if;
                    mul_a <= shift_right(mul_a, 1);
                    mul_b <= shift_left(mul_b, 1);
                end if;
                -- mod W by repeated subtraction
                if seq_cnt >= 31 and seq_cnt <= 46 then
                    if seq_d >= resize(w_r, 15) then
                        seq_d <= seq_d - resize(w_r, 15);
                    end if;
                end if;
            end if;

            -------------------------------------------------------------------
            -- s1: input
            -------------------------------------------------------------------
            s1_y    <= unsigned(data_in.y);
            s1_u    <= unsigned(data_in.u);
            s1_v    <= unsigned(data_in.v);
            s1_avid <= data_in.avid;
            s1_x    <= pixel_x(10 downto 0);

            -------------------------------------------------------------------
            -- s2: block decode, slip test
            -------------------------------------------------------------------
            case blk_r is
                when "00" =>
                    s2_bx  <= s1_x(10 downto 3);
                    s2_xl  <= "000" & s1_x(2 downto 0);
                    s2_xc3 <= s1_x(2 downto 0);
                    if s1_x(2 downto 0) = "111" or ('0' & s1_x) = wm1_r then s2_last <= '1'; else s2_last <= '0'; end if;
                when "01" =>
                    s2_bx  <= '0' & s1_x(10 downto 4);
                    s2_xl  <= "00" & s1_x(3 downto 0);
                    s2_xc3 <= s1_x(3 downto 1);
                    if s1_x(3 downto 0) = "1111" or ('0' & s1_x) = wm1_r then s2_last <= '1'; else s2_last <= '0'; end if;
                when "10" =>
                    s2_bx  <= "00" & s1_x(10 downto 5);
                    s2_xl  <= '0' & s1_x(4 downto 0);
                    s2_xc3 <= s1_x(4 downto 2);
                    if s1_x(4 downto 0) = "11111" or ('0' & s1_x) = wm1_r then s2_last <= '1'; else s2_last <= '0'; end if;
                when others =>
                    s2_bx  <= "000" & s1_x(10 downto 6);
                    s2_xl  <= s1_x(5 downto 0);
                    s2_xc3 <= s1_x(5 downto 3);
                    if s1_x(5 downto 0) = "111111" or ('0' & s1_x) = wm1_r then s2_last <= '1'; else s2_last <= '0'; end if;
            end case;
            if ('0' & s1_x) >= row_x0 then
                s2_slip <= '1';
            else
                s2_slip <= '0';
            end if;
            s2_y <= s1_y; s2_u <= s1_u; s2_v <= s1_v;
            s2_avid <= s1_avid;
            s2_x    <= s1_x;

            -------------------------------------------------------------------
            -- s3: hash round 1 (block key; slice key while avid is low),
            -- drip-state read
            -------------------------------------------------------------------
            if s2_avid = '1' then
                v_key := row_by & s2_bx;
            else
                v_key := seq_key;
            end if;
            v_hx := seed xor (v_key & x"0000") xor resize(v_key, 32);
            s3_h <= v_hx + shift_left(v_hx, 11);
            s3_st <= ram_st(to_integer(s2_bx));
            s3_y <= s2_y; s3_u <= s2_u; s3_v <= s2_v;
            s3_avid <= s2_avid; s3_slip <= s2_slip; s3_last <= s2_last;
            s3_x <= s2_x; s3_bx <= s2_bx; s3_xl <= s2_xl; s3_xc3 <= s2_xc3;

            -------------------------------------------------------------------
            -- s4: hash round 2, state fabric register
            -------------------------------------------------------------------
            v_t  := s3_h xor shift_right(s3_h, 12);
            s4_h <= v_t + shift_left(v_t, 9);
            s4_st <= s3_st;
            s4_y <= s3_y; s4_u <= s3_u; s4_v <= s3_v;
            s4_avid <= s3_avid; s4_slip <= s3_slip; s4_last <= s3_last;
            s4_x <= s3_x; s4_bx <= s3_bx; s4_xl <= s3_xl; s4_xc3 <= s3_xc3;

            -------------------------------------------------------------------
            -- s5: hash round 3, drip frozen test
            -- state word: 31 valid, 30 neg, 29 mag(2px), 28:27 len,
            --             26:16 capture row, 10:0 last frozen row
            -------------------------------------------------------------------
            v_t  := s4_h xor shift_right(s4_h, 11);
            s5_h <= v_t + shift_left(v_t, 9);
            v_start := unsigned(s4_st(26 downto 16));
            v_e     := unsigned(s4_st(10 downto 0));
            if row_aline <= v_e then
                s5_le <= '1';
            else
                s5_le <= '0';
            end if;
            if s4_st(31) = '1' and row_aline > v_start and row_aline <= v_e then
                s5_frz <= '1';
            else
                s5_frz <= '0';
            end if;
            s5_valid <= s4_st(31);
            s5_dneg  <= s4_st(30);
            s5_dmag  <= s4_st(29);
            s5_len   <= unsigned(s4_st(28 downto 27));
            s5_start <= v_start;
            s5_n     <= row_aline - v_start;
            s5_y <= s4_y; s5_u <= s4_u; s5_v <= s4_v;
            s5_avid <= s4_avid; s5_slip <= s4_slip; s5_last <= s4_last;
            s5_x <= s4_x; s5_bx <= s4_bx; s5_xl <= s4_xl; s5_xc3 <= s4_xc3;

            -------------------------------------------------------------------
            -- s6: video RAM write (frozen columns keep their captured row),
            -- hash decode, offset terms, drip-state update
            -------------------------------------------------------------------
            s6_we_y <= s5_avid and ((not s5_frz) or bleed_r);
            s6_we_c <= s5_avid and (not s5_frz);
            s6_wa   <= s5_x;
            s6_wy   <= std_logic_vector(s5_y);
            s6_wu   <= std_logic_vector(s5_u(9 downto 1)) & line_par;
            s6_wv   <= std_logic_vector(s5_v);

            -- h: 31:26 a (vs GOP ramp), 25:20 block density, 19:14 & 11:9
            -- drip roll (9 bits), 13:12 type, 11:9 u, 8:6 v, 5 chroma size,
            -- 4 chroma sign, 3:2 drip direction, 1:0 drip length
            v_h := s5_h xor shift_right(s5_h, 15);
            if resize(v_h(31 downto 26), 7) < ramp7
               and resize(v_h(25 downto 20), 7) < densb7 then
                v_corrupt := '1';
            else
                v_corrupt := '0';
            end if;
            v_dr := v_h(19 downto 14) & v_h(11 downto 9);
            if resize(v_h(31 downto 26), 7) < ramp7
               and resize(v_dr, 10) < densd10 and unit_r /= 0 then
                v_hit := '1';
            else
                v_hit := '0';
            end if;
            v_typ := v_h(13 downto 12);
            s6_corrupt <= v_corrupt;
            if v_corrupt = '1' and (v_typ = "00" or v_typ = "11") then
                s6_walsh <= '1';
            else
                s6_walsh <= '0';
            end if;
            s6_wsign <= f_parity(s5_xc3 and v_h(11 downto 9))
                        xor f_parity(row_yc3 and v_h(8 downto 6));
            if v_corrupt = '1' and (v_typ = "01" or v_typ = "11") then
                s6_xl <= s5_xl;   -- smear: read the block's left column
            else
                s6_xl <= (others => '0');
            end if;
            if v_corrupt = '1' and v_typ = "10" then
                case k4_r(9 downto 8) is   -- melt: read back by luma >> m
                    when "11"   => s6_ym <= s5_y(9 downto 2);
                    when "10"   => s6_ym <= '0' & s5_y(9 downto 3);
                    when "01"   => s6_ym <= "00" & s5_y(9 downto 4);
                    when others => s6_ym <= "000" & s5_y(9 downto 5);
                end case;
            else
                s6_ym <= (others => '0');
            end if;
            if v_h(5) = '1' then
                v_mag := cm_r;
            else
                v_mag := '0' & cm_r(8 downto 1);
            end if;
            if v_corrupt = '0' then
                s6_e <= (others => '0');
            elsif v_h(4) = '1' then
                s6_e <= -signed(resize(v_mag, 11));
            else
                s6_e <= signed(resize(v_mag, 11));
            end if;
            if s5_slip = '1' then
                s6_dslip <= row_d;
            else
                s6_dslip <= (others => '0');
            end if;
            -- Flow: the frozen row scrolls 1 or 2 px per row WITHIN its own
            -- block column (wrap mod B), so a drip only ever shows its own
            -- captured row; Freeze = no scroll
            if flow_r = '0' then
                s6_m6 <= (others => '0');
            elsif s5_dmag = '1' then
                s6_m6 <= s5_n(4 downto 0) & '0';
            else
                s6_m6 <= s5_n(5 downto 0);
            end if;
            s6_xlr  <= s5_xl;
            s6_dneg <= s5_dneg;
            s6_frz  <= s5_frz;
            s6_slip <= s5_slip;
            s6_x    <= s5_x;

            -- Drip state: on block-row-start lines, at the column's last
            -- pixel (the next column's reads are already past it). A live
            -- drip keeps its capture row and grows/shrinks with the current
            -- length; an expired or empty column may start a new drip here.
            if s5_avid = '1' and s5_last = '1' and row_yl0 = '1' then
                s6_st_we <= '1';
                s6_st_wa <= s5_bx;
                v_act := s5_valid and s5_le;
                if v_act = '1' then
                    v_base := s5_start;
                    v_lsel := s5_len;
                    v_dir  := s5_dneg & s5_dmag;
                else
                    v_base := row_aline;
                    v_lsel := v_h(1 downto 0);
                    v_dir  := std_logic_vector(v_h(3 downto 2));
                end if;
                case v_lsel is
                    when "00"   => v_l := len1;
                    when "01"   => v_l := len2;
                    when "10"   => v_l := len3;
                    when others => v_l := len4;
                end case;
                v_sum12 := resize(v_base, 12) + resize(v_l, 12);
                if v_sum12(11) = '1' then
                    v_eend := (others => '1');
                else
                    v_eend := v_sum12(10 downto 0);
                end if;
                s6_st_wd <= (v_act or v_hit) & v_dir & std_logic_vector(v_lsel)
                            & std_logic_vector(v_base) & "00000"
                            & std_logic_vector(v_eend);
            else
                s6_st_we <= '0';
            end if;

            -------------------------------------------------------------------
            -- Memories: one write + one unconditional read per RAM
            -------------------------------------------------------------------
            if s6_we_y = '1' then
                ram_y(to_integer(s6_wa)) <= s6_wy;
            end if;
            if s6_we_c = '1' then
                ram_u(to_integer(s6_wa)) <= s6_wu;
                ram_v(to_integer(s6_wa)) <= s6_wv;
            end if;
            if s6_st_we = '1' then
                ram_st(to_integer(s6_st_wa)) <= s6_st_wd;
            end if;
            s12_ry <= ram_y(to_integer(s11_ray));
            s12_ru <= ram_u(to_integer(s11_rac));
            s12_rv <= ram_v(to_integer(s11_rac));

            -------------------------------------------------------------------
            -- s7: offset partial sums (all terms >= 0), drip offset mod W
            -------------------------------------------------------------------
            s7_a1 <= s6_dslip + resize(s6_xl, 12);
            s7_ym <= s6_ym;
            if s6_corrupt = '1' or s6_slip = '1' or s6_frz = '1' then
                s7_a2 <= resize(s6_ym, 12) + resize(cs_r, 12);
            else
                s7_a2 <= resize(s6_ym, 12);
            end if;
            if s6_dneg = '1' then
                s7_t6 <= s6_xlr + s6_m6;
            else
                s7_t6 <= s6_xlr - s6_m6;
            end if;
            case blk_r is
                when "00"   => s7_cb <= s6_x(10 downto 3) & "000";
                when "01"   => s7_cb <= s6_x(10 downto 4) & "0000";
                when "10"   => s7_cb <= s6_x(10 downto 5) & "00000";
                when others => s7_cb <= s6_x(10 downto 6) & "000000";
            end case;
            s7_frz  <= s6_frz;
            s7_x    <= s6_x;
            fx(7).walsh <= s6_walsh;
            fx(7).wsign <= s6_wsign;
            fx(7).dmg   <= s6_corrupt or s6_slip or s6_frz;
            fx(7).e     <= s6_e;

            -------------------------------------------------------------------
            -- s8: Y/C offset sums (< 2W); drip address inside the column
            -------------------------------------------------------------------
            s8_oy <= s7_a1 + resize(s7_ym, 12);
            s8_oc <= s7_a1 + s7_a2;
            case blk_r is
                when "00"   => s8_ad <= s7_cb(10 downto 3) & s7_t6(2 downto 0);
                when "01"   => s8_ad <= s7_cb(10 downto 4) & s7_t6(3 downto 0);
                when "10"   => s8_ad <= s7_cb(10 downto 5) & s7_t6(4 downto 0);
                when others => s8_ad <= s7_cb(10 downto 6) & s7_t6(5 downto 0);
            end case;
            s8_frz <= s7_frz;
            s8_x   <= s7_x;

            -------------------------------------------------------------------
            -- s9: normalise mod W
            -------------------------------------------------------------------
            if s8_oy >= w_r then
                s9_offy <= s8_oy - w_r;
            else
                s9_offy <= s8_oy;
            end if;
            if s8_oc >= w_r then
                s9_offc <= s8_oc - w_r;
            else
                s9_offc <= s8_oc;
            end if;
            s9_ad  <= s8_ad;
            s9_frz <= s8_frz;
            s9_x   <= s8_x;

            -------------------------------------------------------------------
            -- s10: wrap term, collision flag (offset W-K_COL would read the
            -- column being written this clock)
            -------------------------------------------------------------------
            s10_wmoy <= w_r - s9_offy;
            s10_wmoc <= w_r - s9_offc;
            if s9_offy = wk_r then s10_gy <= '1'; else s10_gy <= '0'; end if;
            if s9_offc = wk_r then s10_gc <= '1'; else s10_gc <= '0'; end if;
            s10_offy <= s9_offy;
            s10_offc <= s9_offc;
            s10_x    <= s9_x;
            s10_ad   <= s9_ad;
            s10_fy   <= s9_frz and not bleed_r;
            s10_fc   <= s9_frz;

            -------------------------------------------------------------------
            -- s11: read address = (x - off) mod W; frozen columns read their
            -- own captured row (never written while frozen: no collision)
            -------------------------------------------------------------------
            v_ra := signed(resize(s10_x, 13)) - signed(resize(s10_offy, 13));
            v_rb := resize(s10_x, 12) + s10_wmoy;
            if s10_fy = '1' then
                s11_ray <= s10_ad;
            elsif s10_gy = '1' then
                s11_ray <= s10_x;
            elsif v_ra < 0 then
                s11_ray <= v_rb(10 downto 0);
            else
                s11_ray <= unsigned(v_ra(10 downto 0));
            end if;
            v_ra := signed(resize(s10_x, 13)) - signed(resize(s10_offc, 13));
            v_rb := resize(s10_x, 12) + s10_wmoc;
            if s10_fc = '1' then
                s11_rac <= s10_ad;
            elsif s10_gc = '1' then
                s11_rac <= s10_x;
            elsif v_ra < 0 then
                s11_rac <= v_rb(10 downto 0);
            else
                s11_rac <= unsigned(v_ra(10 downto 0));
            end if;

            -- compose flags ride along
            for i in 8 to 13 loop
                fx(i) <= fx(i-1);
            end loop;

            -------------------------------------------------------------------
            -- s13: fabric register on the RAM data
            -------------------------------------------------------------------
            s13_ry <= s12_ry;
            s13_ru <= s12_ru;
            s13_rv <= s12_rv;

            -------------------------------------------------------------------
            -- s14: U/V swap where the stored row's parity differs from this
            -- row's, Walsh luma pattern, chroma DC error (equal on U and V)
            -------------------------------------------------------------------
            if (s13_ru(0) xor line_par) = '1' then
                v_us := unsigned(s13_rv);
                v_vs := unsigned(s13_ru(9 downto 1)) & '0';
            else
                v_us := unsigned(s13_ru(9 downto 1)) & '0';
                v_vs := unsigned(s13_rv);
            end if;
            s14_u <= signed(resize(v_us, 12)) + resize(fx(13).e, 12);
            s14_v <= signed(resize(v_vs, 12)) + resize(fx(13).e, 12);
            if fx(13).walsh = '0' then
                s14_y <= signed(resize(unsigned(s13_ry), 12));
            elsif fx(13).wsign = '1' then
                s14_y <= signed(resize(unsigned(s13_ry), 12)) + signed(resize(amp_r, 12));
            else
                s14_y <= signed(resize(unsigned(s13_ry), 12)) - signed(resize(amp_r, 12));
            end if;
            s14_dmg <= fx(13).dmg;

            -------------------------------------------------------------------
            -- s15: clamp
            -------------------------------------------------------------------
            if s14_y < 64 then
                s15_y <= C_Y_BLACK;
            elsif s14_y > 940 then
                s15_y <= to_unsigned(940, 10);
            else
                s15_y <= unsigned(s14_y(9 downto 0));
            end if;
            if s14_u < 16 then
                s15_u <= to_unsigned(16, 10);
            elsif s14_u > 1008 then
                s15_u <= to_unsigned(1008, 10);
            else
                s15_u <= unsigned(s14_u(9 downto 0));
            end if;
            if s14_v < 16 then
                s15_v <= to_unsigned(16, 10);
            elsif s14_v > 1008 then
                s15_v <= to_unsigned(1008, 10);
            else
                s15_v <= unsigned(s14_v(9 downto 0));
            end if;
            s15_dmg <= s14_dmg;

            -------------------------------------------------------------------
            -- s16: Quant. Damaged pixels: 8 luma levels (64, 192 .. 896),
            -- chroma in 128 steps about 512; clean pixels: 16 floored luma
            -- levels, chroma in 64 steps.
            -------------------------------------------------------------------
            if quant_r = '0' then
                s16_y <= s15_y;
                s16_u <= s15_u;
                s16_v <= s15_v;
            elsif s15_dmg = '1' then
                if s15_y(9 downto 7) = "111" then
                    s16_y <= to_unsigned(940, 10);
                else
                    s16_y <= s15_y(9 downto 7) & "1000000";
                end if;
                v_q := resize(s15_u, 11) + 64;
                if v_q(10) = '1' then s16_u <= to_unsigned(896, 10); else s16_u <= v_q(9 downto 7) & "0000000"; end if;
                v_q := resize(s15_v, 11) + 64;
                if v_q(10) = '1' then s16_v <= to_unsigned(896, 10); else s16_v <= v_q(9 downto 7) & "0000000"; end if;
            else
                if s15_y(9 downto 6) = 0 then
                    s16_y <= C_Y_BLACK;
                else
                    s16_y <= s15_y(9 downto 6) & "000000";
                end if;
                v_q := resize(s15_u, 11) + 32;
                if v_q(10) = '1' then s16_u <= to_unsigned(960, 10); else s16_u <= v_q(9 downto 6) & "000000"; end if;
                v_q := resize(s15_v, 11) + 32;
                if v_q(10) = '1' then s16_v <= to_unsigned(960, 10); else s16_v <= v_q(9 downto 6) & "000000"; end if;
            end if;

            -------------------------------------------------------------------
            -- s17: blanking gate
            -------------------------------------------------------------------
            if s_avid_sr(LATENCY-2) = '0' then
                s17_y <= C_Y_BLACK;
                s17_u <= C_UV_ZERO;
                s17_v <= C_UV_ZERO;
            else
                s17_y <= s16_y;
                s17_u <= s16_u;
                s17_v <= s16_v;
            end if;

            -------------------------------------------------------------------
            -- Sync delay lines
            -------------------------------------------------------------------
            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to LATENCY-1 loop
                s_hsync_sr(i) <= s_hsync_sr(i-1);
                s_vsync_sr(i) <= s_vsync_sr(i-1);
                s_field_sr(i) <= s_field_sr(i-1);
                s_avid_sr(i)  <= s_avid_sr(i-1);
            end loop;
        end if;
    end process main_p;

    data_out.hsync_n <= s_hsync_sr(LATENCY-1);
    data_out.vsync_n <= s_vsync_sr(LATENCY-1);
    data_out.field_n <= s_field_sr(LATENCY-1);
    data_out.avid    <= s_avid_sr(LATENCY-1);
    data_out.y       <= std_logic_vector(s17_y);
    data_out.u       <= std_logic_vector(s17_u);
    data_out.v       <= std_logic_vector(s17_v);

end architecture slipframe;
