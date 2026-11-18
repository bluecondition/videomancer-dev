-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Windrose v0.1.0 -- Straightedge.  The compass rose of portolan charts and engraved maps: tiers of pointed
-- stars, every point split down its axis into a light and a dark half, the tiers turning against each other.
-- Luma only (U = V = 512).  Every edge is the zero of an affine function.
--
--   star j (four: the great 4-point star, a 4-point star, two 8-point stars):
--     u, v = the pixel in the star's turned frame; four accumulators A = c u, B = s u, C = c v, D = s v
--     (c, s = cos, sin of the edge normal's angle from the point's axis, set by Point)
--     4 points:  E = (|u| >= |v| ? |A| + |D| : |C| + |B|) - K
--     8 points:  E = min(max(|A|, |C|) + min(|B|, |D|),  (|A| + |C| + ||B| - |D||) / sqrt 2) - K
--     coverage from E and the distance to the point's own axis (so needles and tips taper), light / dark from
--     the fold's parity, Fine = the outline and the axis line
--   Grey: the stars painted one over another (smaller = greyer).  Ink: even-odd.
--   Field: the accumulators' steps reverse at every cell border (a mirror fold), so the screen is a wall of
--     roses, each the mirror image of its neighbours.
--   Camera: the luma (or Osc) is averaged over 32 x 32 virtual px blocks into a block-RAM grid (two banks); the
--     next field reads the pixel's own block, holds it across the block, and it sets the WEIGHT of echo lines
--     (period 16 virtual px; 32 at 480i / 576i) ruled around the great star at constant distance from its edges.
--
--   K1  X        the rose's centre, +-640 virtual px (Field: pan, +- one cell)         (glided)
--   K2  Y        +-360 virtual px                                                       (glided)
--   K3  Scale    Rose: R = 175 .. 1605 virtual px; Field: cell 89 .. 818, exponential   (glided)
--   K4  Points   4 / 8 / 16 / 24 (1 .. 4 stars; stepped, hysteresis)
--   K5  Drift    bipolar speed: still at the centre, 2^-13 .. 2^-6 quarter turn per field; star rates 1, -3/2,
--                2, -3
--   K6  Camera   echo line weight, 0 .. 75 % duty at white                               (glided)
--   S7  Glide    Off / On       S8  Form  Rose / Field       S9  Tone  Grey / Ink
--   S10 Weight   Bold / Fine    S11 Source  Video / Osc
--   P12 POINT    fat (edge normal 52 degrees from the axis) .. compass (79.2) .. needles (88)   (glided)
--
-- Sections (the integer model .straightedge_work/rb_windrose/imodel.py mirrors each one):
--   [A] TIMING            timing-ID steps, vblank zone, width, line count, generated avid, sync delay
--   [B] CONTROLS          switches, Points hysteresis
--   [C] FIELD ENGINE      Hardline flute's microcoded engine (windrose.py): holds, glides, drift, sin / cos,
--                         every DDA's step / wraps / seed -> one bank of the table; the picture uses the bank
--                         written during the field before
--   [D] SOURCE FIELD      luma or the Osc lattice -> block sums -> vertical one-pole -> gain -> grid (two banks)
--   [E] DDAS              16 accumulators with their own increment registers, the table loader, the mirror fold
--   [P] STARS             per star: fold, candidates, E, coverage
--   [G] BLEND             echo band; the two topmost stars blended over the third (or the echo); Ink
--   [F] OUTPUT            luma, [1 2 1] / 4, 13/16 toward black, blanking gate
--
-- Timing: T = the g_start clock.  The 8-point stars' accumulators hold DDA index i at T + 6 + i, the 4-point
-- stars' at T + 9 + i; every star's coverage is valid at T + 15 + i; the tone at T + 23, the luma at T + 24;
-- output column c is the [1 2 1] filter centred on DDA index c + 5.  One latency (32) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.windrose_pkg.all;
use work.all;

architecture windrose of program_top is
    constant C_LAT  : integer := 32;   -- = processing_delay_clks
    constant C_NEXT : integer := 6;    -- DDA steps per line = width + C_NEXT (windrose.py NEXTRA)
    constant C_RLEN : integer := 5;    -- run / hold / flip delay line

    ---------------------------------------------------------------------------
    -- [A] timing-ID steps
    ---------------------------------------------------------------------------
    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 12);
            when 4          => return to_unsigned(576, 12);
            when 8          => return to_unsigned(960, 12);
            when 12         => return to_unsigned(480, 12);
            when 5 | 6 | 14 => return to_unsigned(384, 12);
            when 1 | 2 | 10 => return to_unsigned(512, 12);
            when others     => return to_unsigned(256, 12);
        end case;
    end function;
    -- x step in h (half virtual px) per pixel: 1080 = 2, 720p = 3, SD = 4
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    -- interlaced modes (field lines are two pixels tall): 480i, 576i, 1080i
    function f_il(t : std_logic_vector(3 downto 0)) return std_logic is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 8 | 1 | 2 | 10 => return '1';
            when others             => return '0';
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- the DDA delay of star j (the 4-point stars' front is three stages shorter)
    function f_skew(j : integer) return integer is
    begin
        if j < 2 then return 3; else return 0; end if;
    end function;
    -- clamp to 0 .. 8
    function f_cl8(v : signed(8 downto 0)) return unsigned is
    begin
        if v(8) = '1' then return to_unsigned(0, 4);
        elsif v(7 downto 3) /= "00000" then return to_unsigned(8, 4);
        else return '0' & unsigned(v(2 downto 0)); end if;
    end function;
    -- |x| by one's complement
    function f_abs14(v : signed(14 downto 0)) return unsigned is
        variable r : unsigned(13 downto 0);
    begin
        for i in 0 to 13 loop r(i) := v(i) xor v(14); end loop;
        return r;
    end function;
    -- saturate to 5 bits (the axis distance) and to 4 (the swap line's)
    function f_sat5(v : unsigned(13 downto 0)) return unsigned is
    begin
        if v(13 downto 5) /= "000000000" then return to_unsigned(31, 5); else return v(4 downto 0); end if;
    end function;
    function f_sat4(v : unsigned(15 downto 0)) return unsigned is
    begin
        if v(15 downto 4) /= "000000000000" then return to_unsigned(15, 4); else return v(3 downto 0); end if;
    end function;
    -- a star's tone of a facet / line coverage 0 .. 8
    function f_tone(j : integer; fine : std_logic; x : unsigned(3 downto 0)) return unsigned is
        variable hi, lo, xi, r : integer;
    begin
        case j is
            when 0      => hi := 255; lo := 77;
            when 1      => hi := 204; lo := 56;
            when 2      => hi := 158; lo := 41;
            when others => hi := 117; lo := 28;
        end case;
        xi := to_integer(x);
        if xi > 8 then xi := 8; end if;
        if fine = '1' then r := (hi * xi + 4) / 8;
        else r := lo + ((hi - lo) * xi + 4) / 8; end if;
        return to_unsigned(r, 8);
    end function;
    -- a sample times a weight 0 .. 4
    function f_mulw(l : unsigned(7 downto 0); w : unsigned(2 downto 0)) return unsigned is
        variable a, b : unsigned(10 downto 0);
    begin
        if w(2) = '1' then a := resize(l & "00", 11);
        elsif w(1) = '1' then a := resize(l & '0', 11);
        else a := (others => '0'); end if;
        if w(0) = '1' then b := resize(l, 11); else b := (others => '0'); end if;
        return a + b;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width / lines
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_ilr, s_il : std_logic := '0';
    signal s_ush  : unsigned(1 downto 0) := "00";                  -- the anti-aliasing unit = 2^ush virtual px
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_lcnt, s_hn : unsigned(11 downto 0) := (others => '0');
    signal s_hv, z_hbig : std_logic := '0';
    signal s_bank : std_logic := '0';                              -- the bank being written this field

    -- [A] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 3) := (others => '0');   -- s_st(k) = g_start delayed k + 1

    -- [B] switches, Points
    signal s_glide : std_logic := '1';
    signal s_field, s_ink, s_fine, s_src : std_logic := '0';
    signal s_zn : unsigned(1 downto 0) := to_unsigned(2, 2);       -- tiers - 1
    signal z_up, z_dn : std_logic := '0';
    signal s_k4 : unsigned(9 downto 0) := (others => '0');

    -- ROM: the engine's table (2^(i/256) + quarter sine)
    signal r_ex_a : unsigned(8 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');

    -- [C] the per-field engine (Hardline flute's): 37-bit accumulator, 256 x 32 register file, one adder,
    -- serial 18 x 18 multiplier, serial right shift; microprogram in block RAM
    constant OP_LD   : std_logic_vector(3 downto 0) := x"0";
    constant OP_ADD  : std_logic_vector(3 downto 0) := x"1";
    constant OP_SUB  : std_logic_vector(3 downto 0) := x"2";
    constant OP_MUL  : std_logic_vector(3 downto 0) := x"3";
    constant OP_LDI  : std_logic_vector(3 downto 0) := x"4";
    constant OP_ADDI : std_logic_vector(3 downto 0) := x"5";
    constant OP_SUBI : std_logic_vector(3 downto 0) := x"6";
    constant OP_MULI : std_logic_vector(3 downto 0) := x"7";
    constant OP_ST   : std_logic_vector(3 downto 0) := x"8";
    constant OP_IN   : std_logic_vector(3 downto 0) := x"9";
    constant OP_OUT  : std_logic_vector(3 downto 0) := x"A";
    constant OP_SHR  : std_logic_vector(3 downto 0) := x"B";
    constant OP_SKIP : std_logic_vector(3 downto 0) := x"C";
    constant OP_JMP  : std_logic_vector(3 downto 0) := x"D";
    constant OP_CALL : std_logic_vector(3 downto 0) := x"E";
    constant OP_SPC  : std_logic_vector(3 downto 0) := x"F";      -- 0 return, 1 halt
    signal rf_ram : t_c_wr_rfi := C_WR_RFI;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q  : std_logic_vector(31 downto 0) := (others => '0');
    signal e_run : std_logic := '0';
    signal e_ph  : unsigned(1 downto 0) := "11";
    signal e_pc, e_lr : unsigned(9 downto 0) := (others => '0');
    signal e_ir  : std_logic_vector(15 downto 0) := x"F001";      -- the microprogram ROM's output register
    signal e_irf : std_logic_vector(15 downto 0) := x"F001";      -- its fabric copy
    signal x_mul, x_shr, x_halt, x_out, x_st : std_logic := '0';
    signal x_gate, x_subr, x_shift, x_cz : std_logic := '0';
    signal e_acc, e_b : signed(36 downto 0) := (others => '0');
    signal e_cnt : unsigned(4 downto 0) := (others => '0');
    signal e_out : std_logic;
    signal w_in  : unsigned(15 downto 0);

    -- [C][E] the DDA table: two banks of 256 words; the engine writes bank s_bank, the loader reads the other
    type t_dt is array (0 to 511) of std_logic_vector(31 downto 0);
    signal dt_ram : t_dt := (others => (others => '0'));
    attribute no_rw_check of dt_ram : signal is true;
    signal dt_q, dt_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal dt_ra, dt_wa : unsigned(7 downto 0) := (others => '0');
    signal dt_we : std_logic := '0';

    -- [E] the field's constants (from the table, at the zone start)
    type t_s16x4 is array (0 to 3) of signed(15 downto 0);
    signal c_K   : t_s16x4 := (others => (others => '0'));         -- K per star, 1/8 unit
    signal c_t8  : unsigned(5 downto 0) := (others => '0');        -- Fine line weight, 1/8 unit (at most 48)
    signal c_th8 : unsigned(4 downto 0) := (others => '0');
    signal c_n0x, c_wc, c_n0y, c_hc : unsigned(11 downto 0) := (others => '0');
    signal c_ox, c_oy : std_logic := '0';
    signal c_g6  : unsigned(7 downto 0) := (others => '0');
    signal f_run, f_v1, f_v2 : std_logic := '0';
    signal f_t, f_i1, f_i2 : unsigned(2 downto 0) := (others => '0');

    -- [E] DDAs and their loader
    type t_s24x16 is array (0 to 15) of signed(23 downto 0);
    signal ca_inc, ca_acc : t_s24x16 := (others => (others => '0'));
    signal ca_ld, ca_en : std_logic_vector(15 downto 0) := (others => '0');
    signal ca_rst : std_logic := '0';
    signal x_en, x_fl, x_rn, x_neg : std_logic_vector(3 downto 0) := (others => '0');
    signal l_kick, l_run : std_logic := '0';
    signal l_t : unsigned(4 downto 0) := (others => '0');
    signal l_word : std_logic_vector(2 downto 0) := "010";
    signal l_v1, l_v2, l_o1, l_o2 : std_logic := '0';
    signal l_c1, l_c2 : unsigned(3 downto 0) := (others => '0');
    signal b_run, b_end : std_logic := '0';
    signal b_cnt, b_mm1 : unsigned(11 downto 0) := (others => '0');
    signal fx_cnt, fy_cnt : unsigned(11 downto 0) := (others => '1');
    signal fy_neg : std_logic := '0';
    signal r_run, r_hold, r_flip : std_logic_vector(0 to C_RLEN - 1) := (others => '0');
    signal b_hold, b_flip : std_logic;

    -- [P] stars
    type t_u4x4 is array (0 to 3) of unsigned(3 downto 0);
    signal st_cs, st_x : t_u4x4 := (others => (others => '0'));
    type t_u7x4 is array (0 to 3) of unsigned(6 downto 0);
    signal st_em : t_u7x4 := (others => (others => '0'));

    -- [G] blend
    type t_u8x4 is array (0 to 3) of unsigned(7 downto 0);
    signal g1_tone : t_u8x4 := (others => (others => '0'));
    signal g1_cs, g1_x : t_u4x4 := (others => (others => '0'));
    signal e1_ax : unsigned(6 downto 0) := (others => '0');
    signal e2_a, e2_b, e3_cb : unsigned(3 downto 0) := (others => '0');
    signal g2_t1, g2_t2, g2_base, g3_t1, g3_base, g4_t1, g4_base, g5_v, g6_v, g7_tone : unsigned(7 downto 0) := (others => '0');
    signal g2_c1, g2_c2, g3_c1, g3_c2, g4_c1, g5_c1 : unsigned(3 downto 0) := (others => '0');
    signal g3_d, g5_d : signed(8 downto 0) := (others => '0');
    signal g4_p, g6_p : signed(12 downto 0) := (others => '0');

    -- [D] camera grid: the read side
    signal r_vy : unsigned(12 downto 0) := (others => '0');
    signal r_rb : unsigned(10 downto 0) := (others => '0');        -- the line's block row x 60
    signal r_first : std_logic := '1';
    signal cm_ph : unsigned(5 downto 0) := (others => '0');
    signal cm_col : unsigned(5 downto 0) := (others => '0');
    signal cm_i : unsigned(2 downto 0) := (others => '0');
    type t_gr is array (0 to 4095) of unsigned(7 downto 0);
    signal gr_ram : t_gr := (others => (others => '0'));
    attribute no_rw_check of gr_ram : signal is true;
    signal gr_ra, gr_wa : unsigned(11 downto 0) := (others => '0');
    signal gr_q, gr_f, gr_wd : unsigned(7 downto 0) := (others => '0');
    signal gr_we : std_logic := '0';
    signal cm_thr : unsigned(5 downto 0) := (others => '0');

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl  : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_fl, s_seen : std_logic := '0';
    signal w_ph  : unsigned(5 downto 0) := (others => '0');
    signal w_l   : unsigned(7 downto 0) := (others => '0');
    signal w_wa, w_wb : unsigned(2 downto 0) := (others => '0');
    signal w_cy, w_v1, w_cy2, w_v2, w_dv, w_v4, w_v5, w_v6 : std_logic := '0';
    signal w_pa, w_pb : unsigned(10 downto 0) := (others => '0');
    signal w_acc, w_dump, w_dd, w_qd, w_qn : unsigned(13 downto 0) := (others => '0');
    signal w_col, w_dcol, w_c4, w_c5, w_c6 : unsigned(5 downto 0) := (others => '0');
    signal w_diff : signed(14 downto 0) := (others => '0');
    type t_iq is array (0 to 63) of unsigned(15 downto 0);
    signal iq_ram : t_iq := (others => (others => '0'));
    attribute no_rw_check of iq_ram : signal is true;
    signal iq_q, iq_f : unsigned(15 downto 0) := (others => '0');
    signal iq_we : std_logic := '0';
    signal w_glo, w_ghi : unsigned(11 downto 0) := (others => '0');

    -- [F] output conditioning
    signal s_y23, s_yd1, s_yd2, s_yb : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_ya : unsigned(11 downto 0) := (others => '0');
    signal s_y26, s_y27, s_y28 : unsigned(9 downto 0) := to_unsigned(64, 10);

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_WR_TAB(to_integer(r_ex_a));
        end if;
    end process p_rom;

    -- DDA table: written by the engine (bank s_bank), read by the loader (the other bank)
    p_dtram : process(clk)
    begin
        if rising_edge(clk) then
            dt_q <= dt_ram(to_integer((not s_bank) & dt_ra));
            if dt_we = '1' then dt_ram(to_integer(s_bank & dt_wa)) <= dt_wd; end if;
        end if;
    end process p_dtram;

    -- camera grid: 2 banks x 34 rows x 60 blocks
    p_grram : process(clk)
    begin
        if rising_edge(clk) then
            gr_q <= gr_ram(to_integer(gr_ra));
            if gr_we = '1' then gr_ram(to_integer(gr_wa)) <= gr_wd; end if;
        end if;
    end process p_grram;

    -- the vertical one-pole's state, one word per block column
    p_iqram : process(clk)
    begin
        if rising_edge(clk) then
            iq_q <= iq_ram(to_integer(w_col));
            if iq_we = '1' then iq_ram(to_integer(w_c5)) <= resize(w_qn, 16); end if;
        end if;
    end process p_iqram;

    ---------------------------------------------------------------------------
    -- [A] TIMING + [B] CONTROLS
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_lo, v_hi : unsigned(10 downto 0);
        variable v_vy : unsigned(13 downto 0);
        variable v_hd : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- [A] timing ID -> steps (decoded free-running)
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            s_ilr <= f_il(s_tid);

            -- [A] vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;

            -- [A] width: source avid pixels per line, max over the field; lines per field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;
            if g_start = '1' then s_lcnt <= s_lcnt + 1; end if;
            -- (the line count is re-latched only when it moves by two or more, and always on the first field:
            -- fields that differ by one line must not shift the centre)
            v_hd := signed(resize(s_lcnt, 13)) - signed(resize(s_hn, 13));
            if v_hd > 1 or v_hd < -1 then z_hbig <= '1'; else z_hbig <= '0'; end if;

            -- [B] Points: 4 steps of 256; a step is left only 8 counts past its edge (the outer steps are
            -- entered from inside, so a knob resting at either end reaches them).  The comparison runs free;
            -- the latch is per field.
            s_k4 <= unsigned(registers_in(3));
            v_lo := '0' & s_zn & "00000000";
            if s_zn = 3 then v_hi := to_unsigned(2047, 11); else v_hi := v_lo + 264; end if;
            if resize(s_k4, 11) >= v_hi then z_up <= '1'; else z_up <= '0'; end if;
            if resize(s_k4, 11) + 8 < v_lo then z_dn <= '1'; else z_dn <= '0'; end if;

            -- [A][B] per-field latches
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                if s_hv = '0' or z_hbig = '1' then s_hn <= s_lcnt; end if;
                s_hv <= '1'; s_lcnt <= (others => '0');
                s_bank  <= not s_bank;
                s_glide <= registers_in(6)(0);
                s_field <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                s_il    <= s_ilr;
                if z_up = '1' or z_dn = '1' then s_zn <= s_k4(9 downto 8); end if;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                if s_xsr = 2 then s_ush <= '0' & s_ilr;
                elsif s_ilr = '1' then s_ush <= "10";
                else s_ush <= "01"; end if;
            end if;
            g_wm1 <= s_w - 1;
            b_mm1 <= s_w + (C_NEXT - 1);
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- [D] the line's block row (x 60): 32 virtual lines per row
            if s_zone = '1' then
                r_vy <= (others => '0'); r_rb <= (others => '0'); r_first <= '1';
            elsif g_start = '1' then
                if r_first = '1' then
                    r_first <= '0';
                else
                    v_vy := ('0' & r_vy) + resize(s_ys, 14);
                    r_vy <= v_vy(12 downto 0);
                    if v_vy(13) = '1' then r_rb <= r_rb + 60; end if;
                end if;
            end if;

            -- [A] sync delay, generated-avid and start-pulse shift registers (field_n is tied high by the
            -- core on every input, so it only gets a register)
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 2);
        end if;
    end process p_ctrl;

    ------------------------------------------------------------------------
    -- [C] THE PER-FIELD ENGINE (Hardline flute's).  Started at every zone start
    -- from address 0 (so a stalled or garbled run cannot outlive a field);
    -- three clocks per micro-op (register-file read, operand, execute), serial
    -- multiply (18 steps) and shift.  ACC = 37 bits; MUL = ACC(17:0) x
    -- operand(17:0), both signed, by add-and-shift-right with the last step
    -- subtracting.  The microprogram (windrose.py) does the knob holds and
    -- glides, the drift, Point's sin / cos, the scale and centre (or the Field
    -- lattice's fold) and every accumulator's step, wraps and seed; it writes
    -- them to table bank s_bank while the other bank is being drawn.
    ------------------------------------------------------------------------
    with e_irf(3 downto 0) select w_in <=
        resize(s_ys, 16)            when x"8",
        resize(s_xs, 16)            when x"9",
        "000000000000000" & s_il    when x"A",
        r_ex_q                      when x"B",
        resize(s_w, 16)             when x"C",
        resize(s_zn, 16)            when x"D",
        resize(s_hn, 16)            when x"E",
        (others => '0')             when x"F",
        resize(unsigned(registers_in(to_integer(unsigned(e_irf(2 downto 0))))), 16) when others;

    e_out <= '1' when e_run = '1' and e_ph = "10" and x_out = '1' else '0';

    p_eng : process(clk)
        variable v_op  : std_logic_vector(3 downto 0);
        variable v_arg : std_logic_vector(11 downto 0);
        variable v_shift, v_last, v_cond, v_fetch : std_logic;
        variable v_bx  : signed(36 downto 0);
        variable v_t   : signed(37 downto 0);
        variable v_sum : signed(36 downto 0);
    begin
        if rising_edge(clk) then
            v_op := e_irf(15 downto 12);
            v_arg := e_irf(11 downto 0);
            v_fetch := '0';
            if e_ph = "00" then
                rf_q <= rf_ram(to_integer(unsigned(e_ir(7 downto 0))));
            end if;
            if e_run = '1' and e_ph = "10" and x_st = '1' then
                rf_ram(to_integer(unsigned(e_irf(7 downto 0)))) <= std_logic_vector(e_acc(31 downto 0));
            end if;

            if s_zgo = '1' then
                e_run <= '1'; e_pc <= (others => '0'); e_ph <= "11";
            elsif e_run = '1' then
                case e_ph is
                    when "11" =>                          -- fetch (first micro-op)
                        v_fetch := '1';
                        e_ph <= "00";
                    when "00" =>                          -- register-file read
                        e_irf <= e_ir;
                        e_ph <= "01";
                    when "01" =>                          -- operand, accumulator clear, next address
                        case v_op is
                            when OP_LD | OP_ADD | OP_SUB => e_b <= resize(signed(rf_q), 37);
                            when OP_LDI | OP_ADDI | OP_SUBI => e_b <= resize(signed(v_arg), 37);
                            when OP_IN   => e_b <= signed(resize(w_in, 37));
                            when OP_MUL  => e_b <= shift_left(resize(signed(rf_q(17 downto 0)), 37), 18);
                            when OP_MULI => e_b <= shift_left(resize(signed(v_arg), 37), 18);
                            when others  => null;
                        end case;
                        if v_op = OP_LD or v_op = OP_LDI or v_op = OP_IN then
                            e_acc <= (others => '0');
                        elsif v_op = OP_MUL or v_op = OP_MULI then
                            e_acc(36 downto 18) <= (others => '0');
                        end if;
                        if v_op = OP_MUL or v_op = OP_MULI then e_cnt <= to_unsigned(17, 5);
                        elsif v_op = OP_SHR then e_cnt <= unsigned(v_arg(4 downto 0)) - 1; end if;
                        case v_arg(2 downto 0) is
                            when "000"  => v_cond := e_acc(36);
                            when "010"  => v_cond := s_field;
                            when others => v_cond := s_glide;
                        end case;
                        v_cond := v_cond xor v_arg(3);
                        if v_op = OP_SKIP and v_cond = '1' then e_pc <= e_pc + 2;
                        elsif v_op = OP_JMP then e_pc <= unsigned(v_arg(9 downto 0));
                        elsif v_op = OP_CALL then e_pc <= unsigned(v_arg(9 downto 0)); e_lr <= e_pc + 1;
                        elsif v_op = OP_SPC and v_arg(1 downto 0) = "00" then e_pc <= e_lr;
                        else e_pc <= e_pc + 1; end if;
                        x_mul <= '0'; x_shr <= '0'; x_halt <= '0'; x_out <= '0'; x_st <= '0';
                        x_gate <= '0'; x_subr <= '0'; x_shift <= '0'; x_cz <= '0';
                        case v_op is
                            when OP_LD | OP_ADD | OP_LDI | OP_ADDI | OP_IN => x_gate <= '1';
                            when OP_SUB | OP_SUBI => x_gate <= '1'; x_subr <= '1';
                            when OP_MUL | OP_MULI => x_mul <= '1'; x_shift <= '1'; x_gate <= e_acc(0);
                            when OP_SHR  =>
                                x_shr <= '1'; x_shift <= '1';
                                if v_arg(4 downto 0) = "00001" then x_cz <= '1'; end if;
                            when OP_ST   => x_st <= '1';
                            when OP_OUT  => x_out <= '1';
                            when OP_SPC  => x_halt <= v_arg(0);
                            when others  => null;
                        end case;
                        e_ph <= "10";
                    when others =>                        -- execute (the adder sees registers only)
                        v_shift := x_shift; v_last := '1';
                        if x_gate = '0' then v_bx := (others => '0');
                        elsif x_subr = '1' then v_bx := not e_b;
                        else v_bx := e_b; end if;
                        v_t := (e_acc & '1') + (v_bx & (x_subr and x_gate));
                        v_sum := v_t(37 downto 1);
                        if x_mul = '1' or x_shr = '1' then
                            if x_cz = '0' then v_last := '0'; end if;
                            e_cnt <= e_cnt - 1;
                            if e_cnt = 1 then x_cz <= '1'; x_subr <= x_mul; end if;   -- the next step is the last
                            if x_mul = '1' then x_gate <= v_sum(1); end if;           -- the next multiplier bit
                        end if;
                        if x_halt = '1' then e_run <= '0'; end if;
                        if v_shift = '1' then e_acc <= v_sum(36) & v_sum(36 downto 1);
                        else e_acc <= v_sum; end if;
                        if v_last = '1' then
                            v_fetch := '1';
                            e_ph <= "00";
                        end if;
                end case;
            end if;
            if v_fetch = '1' then
                e_ir <= C_WR_UC(to_integer(e_pc));
            end if;

            -- the engine's outputs: the table ROM's address, the DDA table's write address and data
            dt_we <= '0';
            if dt_we = '1' then dt_wa <= dt_wa + 1; end if;
            if e_out = '1' then
                case e_irf(1 downto 0) is
                    when "00"   => r_ex_a <= unsigned(e_acc(8 downto 0));
                    when "01"   => dt_wa <= unsigned(e_acc(7 downto 0));
                    when others => dt_wd <= std_logic_vector(e_acc(31 downto 0)); dt_we <= '1';
                end case;
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- [A] Generated avid (parity-locked start 3/4 clocks after the source's
    -- avid rise, field-latched width)
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
    -- [E] DDAS.  Twenty accumulators (Q12.14 of the anti-aliasing unit), each
    -- with its own increment register: the pixel step on every clock of the
    -- line window (width + 6 steps), then one add of the line wrap (line step
    -- - the window's net pixel steps) parked in the same register, so the
    -- adder has no input mux.  Field: at a cell border the increment registers
    -- are complemented (carry-in 1 = negated), with one held clock when the
    -- cell width is odd: a mirror fold.  Vertically the loader picks the wrap
    -- word for a forward, a backward or a held line.  The zone start loads the
    -- field's constants and seeds every accumulator from the bank the engine
    -- finished during the field before.
    ---------------------------------------------------------------------------
    b_flip <= '1' when b_run = '1' and fx_cnt = 1 else '0';
    b_hold <= b_flip and c_ox;

    p_load : process(clk)
        variable v_star : unsigned(1 downto 0);
    begin
        if rising_edge(clk) then
            -- line window and the x fold
            b_end <= '0';
            if s_st(3) = '1' then
                b_run <= '1'; b_cnt <= (others => '0');
            elsif b_run = '1' then
                b_cnt <= b_cnt + 1;
                if b_cnt = b_mm1 then b_run <= '0'; b_end <= '1'; end if;
            end if;
            if b_run = '0' then fx_cnt <= c_n0x;
            elsif fx_cnt = 1 then fx_cnt <= c_wc;
            else fx_cnt <= fx_cnt - 1; end if;
            r_run  <= b_run & r_run(0 to C_RLEN - 2);
            r_hold <= b_hold & r_hold(0 to C_RLEN - 2);
            r_flip <= b_flip & r_flip(0 to C_RLEN - 2);
            for g in 0 to 3 loop
                if f_skew(g) = 0 then
                    x_en(g) <= b_run and not b_hold; x_fl(g) <= b_flip; x_rn(g) <= b_run;
                else
                    x_en(g) <= r_run(f_skew(g) - 1) and not r_hold(f_skew(g) - 1);
                    x_fl(g) <= r_flip(f_skew(g) - 1); x_rn(g) <= r_run(f_skew(g) - 1);
                end if;
                if x_rn(g) = '0' then x_neg(g) <= '0';
                elsif x_fl(g) = '1' then x_neg(g) <= not x_neg(g); end if;
            end loop;

            -- the zone start: eight constants words, then the seeds
            if s_zgo = '1' then
                f_run <= '1'; f_t <= (others => '0');
            elsif f_run = '1' then
                f_t <= f_t + 1;
                if f_t = 7 then f_run <= '0'; end if;
            end if;
            f_v1 <= f_run; f_i1 <= f_t; f_v2 <= f_v1; f_i2 <= f_i1;
            l_kick <= '0';
            if f_v2 = '1' then
                case to_integer(f_i2) is
                    when 0 => c_K(0) <= signed(dt_q(15 downto 0));
                    when 1 => c_K(1) <= signed(dt_q(15 downto 0));
                    when 2 => c_K(2) <= signed(dt_q(15 downto 0));
                    when 3 => c_K(3) <= signed(dt_q(15 downto 0));
                    when 4 => c_t8 <= unsigned(dt_q(5 downto 0)); c_th8 <= unsigned(dt_q(12 downto 8));
                    when 5 => c_n0x <= unsigned(dt_q(11 downto 0)); c_wc <= unsigned(dt_q(23 downto 12)); c_ox <= dt_q(24);
                    when 6 => c_n0y <= unsigned(dt_q(11 downto 0)); c_hc <= unsigned(dt_q(23 downto 12)); c_oy <= dt_q(24);
                    when others => c_g6 <= unsigned(dt_q(7 downto 0)); l_kick <= '1';
                end case;
            end if;

            -- the y fold: which wrap word the coming line step takes
            if l_kick = '1' then
                fy_cnt <= c_n0y; fy_neg <= '0'; l_word <= "010";
            elsif b_end = '1' then
                if fy_cnt = 1 and c_oy = '1' then l_word <= "100";
                elsif fy_neg = '1' then l_word <= "011";
                else l_word <= "001"; end if;
                if fy_cnt = 1 then fy_neg <= not fy_neg; fy_cnt <= c_hc;
                else fy_cnt <= fy_cnt - 1; end if;
            end if;

            -- loader: 32 table reads, star 3 first: (seed or wrap), (step)
            ca_rst <= l_kick;
            if l_kick = '1' or b_end = '1' then
                l_run <= '1'; l_t <= (others => '0');
            elsif l_run = '1' then
                l_t <= l_t + 1;
                if l_t = 31 then l_run <= '0'; end if;
            end if;
            v_star := not l_t(4 downto 3);
            if f_run = '1' then dt_ra <= "10100" & f_t;
            elsif l_t(0) = '1' then dt_ra <= '0' & v_star & l_t(2 downto 1) & "000";
            else dt_ra <= '0' & v_star & l_t(2 downto 1) & unsigned(l_word); end if;
            -- the word addressed now is in dt_q two clocks on: it goes into the accumulator's increment
            -- register then (ca_ld), and a wrap / seed word is added one clock after that (ca_en), on the
            -- same edge that puts the pixel step back
            l_v1 <= l_run; l_c1 <= v_star & l_t(2 downto 1); l_o1 <= l_t(0);
            l_v2 <= l_v1;  l_c2 <= l_c1;                     l_o2 <= l_o1;
            for k in 0 to 15 loop
                if l_v1 = '1' and l_c1 = k then ca_ld(k) <= '1'; else ca_ld(k) <= '0'; end if;
                if l_v2 = '1' and l_o2 = '0' and l_c2 = k then ca_en(k) <= '1'; else ca_en(k) <= '0'; end if;
            end loop;
        end if;
    end process p_load;

    g_dda : for d in 0 to 15 generate
        p_dda : process(clk)
            variable v_s : unsigned(24 downto 0);
        begin
            if rising_edge(clk) then
                if ca_ld(d) = '1' then ca_inc(d) <= signed(dt_q(23 downto 0));
                elsif x_fl(d / 4) = '1' then ca_inc(d) <= not ca_inc(d); end if;
                if ca_rst = '1' then
                    ca_acc(d) <= (others => '0');
                elsif x_en(d / 4) = '1' or ca_en(d) = '1' then
                    v_s := (unsigned(ca_acc(d)) & '1') + (unsigned(ca_inc(d)) & x_neg(d / 4));
                    ca_acc(d) <= signed(v_s(24 downto 1));
                end if;
            end if;
        end process p_dda;
    end generate g_dda;

    ---------------------------------------------------------------------------
    -- [P] STARS.  Front (4 points: 3 stages; 8 points: 6) -> the candidate's
    -- f, the distance sb from the point's axis and ad from the line where the
    -- candidates swap, the fold's parity q.  Back (3 stages): E = f - K,
    -- coverage = clamp(1/2 - E) - clamp(1/2 + E - 2 sb) (the far edge of the
    -- same point), the shrunk star for the Fine outline, the axis line, the
    -- facet coverage.  Values are 1/8 unit: accumulator bits 23 .. 9.
    ---------------------------------------------------------------------------
    g_star : for j in 0 to 3 generate
        signal p1_a, p1_b, p1_c, p1_d : unsigned(13 downto 0) := (others => '0');
        signal p1_su, p1_sv : std_logic := '0';
        signal fr_e0 : unsigned(15 downto 0) := (others => '0');
        signal fr_sb : unsigned(4 downto 0) := (others => '0');
        signal fr_ad : unsigned(3 downto 0) := (others => '0');
        signal fr_q  : std_logic := '0';
        signal c1_es : signed(6 downto 0) := (others => '0');
        signal c1_sb : unsigned(4 downto 0) := (others => '0');
        signal c1_dw : unsigned(2 downto 0) := (others => '0');
        signal c1_q, c2_q : std_logic := '0';
        signal c2_a, c2_b, c2_ia, c2_ib, c2_ax, c2_xa, c2_xd : unsigned(3 downto 0) := (others => '0');
    begin
        p_abs : process(clk)
        begin
            if rising_edge(clk) then
                p1_a <= f_abs14(ca_acc(4 * j)(23 downto 9));
                p1_b <= f_abs14(ca_acc(4 * j + 1)(23 downto 9));
                p1_c <= f_abs14(ca_acc(4 * j + 2)(23 downto 9));
                p1_d <= f_abs14(ca_acc(4 * j + 3)(23 downto 9));
                p1_su <= ca_acc(4 * j + 1)(23);
                p1_sv <= ca_acc(4 * j + 3)(23);
            end if;
        end process p_abs;

        g_f4 : if j < 2 generate
            signal p2_f1, p2_f2 : unsigned(14 downto 0) := (others => '0');
            signal p2_dg : signed(14 downto 0) := (others => '0');
            signal p2_sb : unsigned(4 downto 0) := (others => '0');
            signal p2_pq : std_logic := '0';
        begin
            p_f4 : process(clk)
                variable v_dg : signed(14 downto 0);
            begin
                if rising_edge(clk) then
                    -- 2
                    p2_f1 <= resize(p1_a, 15) + resize(p1_d, 15);
                    p2_f2 <= resize(p1_c, 15) + resize(p1_b, 15);
                    v_dg := signed(resize(p1_b, 15)) - signed(resize(p1_d, 15));
                    p2_dg <= v_dg;
                    if v_dg(14) = '0' then p2_sb <= f_sat5(p1_d); else p2_sb <= f_sat5(p1_b); end if;
                    p2_pq <= p1_su xor p1_sv;
                    -- 3
                    if p2_dg(14) = '0' then fr_e0 <= resize(p2_f1, 16); else fr_e0 <= resize(p2_f2, 16); end if;
                    fr_sb <= p2_sb;
                    fr_ad <= f_sat4(resize(f_abs14(p2_dg), 16));
                    fr_q <= p2_pq xor not p2_dg(14);
                end if;
            end process p_f4;
        end generate g_f4;

        g_f8 : if j >= 2 generate
            signal p2_dg : signed(14 downto 0) := (others => '0');
            signal p2_sac, p3_f1, p4_f1, p5_f1 : unsigned(14 downto 0) := (others => '0');
            signal p2_mx, p2_mn : unsigned(13 downto 0) := (others => '0');
            signal p2_pq, p3_pq, p4_pq, p5_pq : std_logic := '0';
            signal p3_f2p, p4_m1, p5_f2 : unsigned(15 downto 0) := (others => '0');
            signal p4_m2 : unsigned(12 downto 0) := (others => '0');
            signal p3_s1, p3_s2, p4_s1, p4_s2, p5_s1, p5_s2 : unsigned(4 downto 0) := (others => '0');
        begin
            p_f8 : process(clk)
                variable v_dg : signed(14 downto 0);
                variable v_ad, v_r : unsigned(13 downto 0);
                variable v_d : signed(16 downto 0);
                variable v_a : unsigned(15 downto 0);
            begin
                if rising_edge(clk) then
                    -- 2: |u| >= |v|: the larger of |A|, |C|, the smaller of |B|, |D|
                    v_dg := signed(resize(p1_b, 15)) - signed(resize(p1_d, 15));
                    p2_dg <= v_dg;
                    p2_sac <= resize(p1_a, 15) + resize(p1_c, 15);
                    if v_dg(14) = '0' then p2_mx <= p1_a; p2_mn <= p1_d;
                    else p2_mx <= p1_c; p2_mn <= p1_b; end if;
                    p2_pq <= p1_su xor p1_sv xor not v_dg(14);
                    -- 3: the first candidate; the second before its 1 / sqrt 2
                    v_ad := f_abs14(p2_dg);
                    p3_f1 <= resize(p2_mx, 15) + resize(p2_mn, 15);
                    p3_f2p <= resize(p2_sac, 16) + resize(v_ad, 16);
                    p3_s1 <= f_sat5(p2_mn);
                    v_r := v_ad - shift_right(v_ad, 2);
                    p3_s2 <= f_sat5(v_r);
                    p3_pq <= p2_pq;
                    -- 4, 5: x 181 / 256 = 1/2 + 1/8 + 1/16 + 1/64 + 1/256
                    p4_m1 <= resize(shift_right(p3_f2p, 1), 16) + resize(shift_right(p3_f2p, 3), 16);
                    p4_m2 <= resize(shift_right(p3_f2p, 4), 13) + resize(shift_right(p3_f2p, 6), 13) + resize(shift_right(p3_f2p, 8), 13);
                    p4_f1 <= p3_f1; p4_s1 <= p3_s1; p4_s2 <= p3_s2; p4_pq <= p3_pq;
                    p5_f2 <= p4_m1 + resize(p4_m2, 16);
                    p5_f1 <= p4_f1; p5_s1 <= p4_s1; p5_s2 <= p4_s2; p5_pq <= p4_pq;
                    -- 6: the smaller candidate
                    v_d := signed(resize(p5_f2, 17)) - signed(resize(p5_f1, 17));
                    if v_d(16) = '0' then fr_e0 <= resize(p5_f1, 16); fr_sb <= p5_s1;
                    else fr_e0 <= p5_f2; fr_sb <= p5_s2; end if;
                    for i in 0 to 15 loop v_a(i) := v_d(i) xor v_d(16); end loop;
                    fr_ad <= f_sat4(v_a);
                    fr_q <= p5_pq xor not v_d(16);
                end if;
            end process p_f8;
        end generate g_f8;

        p_back : process(clk)
            variable v_e   : signed(17 downto 0);
            variable v_w   : unsigned(3 downto 0);
            variable v_es, v_ei, v_s2, v_sb : signed(8 downto 0);
            variable v_cs, v_ci, v_x, v_o, v_l, v_m : unsigned(3 downto 0);
        begin
            if rising_edge(clk) then
                -- c1: E = f - K, saturated to +-8 units; the swap line's distance x 3/4
                v_e := signed(resize(fr_e0, 18)) - resize(c_K(j), 18);
                if v_e(17) = '0' and v_e(16 downto 6) /= "00000000000" then c1_es <= to_signed(63, 7);
                elsif v_e(17) = '1' and v_e(16 downto 6) /= "11111111111" then c1_es <= to_signed(-64, 7);
                else c1_es <= v_e(6 downto 0); end if;
                st_em(j) <= unsigned(v_e(6 downto 0));
                c1_sb <= fr_sb;
                v_w := fr_ad - shift_right(fr_ad, 2);
                if v_w(3) = '1' then c1_dw <= (others => '1'); else c1_dw <= v_w(2 downto 0); end if;
                c1_q <= fr_q;
                -- c2: the clamps
                v_es := resize(c1_es, 9);
                v_s2 := signed(resize(c1_sb & '0', 9));
                v_sb := signed(resize(c1_sb, 9));
                v_ei := v_es + signed(resize(c_t8, 9));
                c2_a  <= f_cl8(4 - v_es);
                c2_b  <= f_cl8(4 + v_es - v_s2);
                c2_ia <= f_cl8(4 - v_ei);
                c2_ib <= f_cl8(4 + v_ei - v_s2);
                c2_ax <= f_cl8(4 + signed(resize(c_th8, 9)) - v_sb);
                c2_xa <= f_cl8(4 - v_sb);
                c2_xd <= f_cl8(4 - signed(resize(c1_dw, 9)));
                c2_q <= c1_q;
                -- c3: coverage; Bold: the facet's light coverage, Fine: the outline and the axis line
                if c2_a > c2_b then v_cs := c2_a - c2_b; else v_cs := (others => '0'); end if;
                if c2_ia > c2_ib then v_ci := c2_ia - c2_ib; else v_ci := (others => '0'); end if;
                if c2_xa > c2_xd then v_x := c2_xa; else v_x := c2_xd; end if;
                if c2_q = '1' then v_o := 8 - v_x; else v_o := v_x; end if;
                if v_cs < c2_ax then v_m := v_cs; else v_m := c2_ax; end if;
                v_l := v_cs - v_ci;
                if v_m > v_l then v_l := v_m; end if;
                st_cs(j) <= v_cs;
                if s_fine = '1' then st_x(j) <= v_l; else st_x(j) <= v_o; end if;
            end if;
        end process p_back;
    end generate g_star;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD -> the camera grid.  luma (or the Osc lattice) -> exact
    -- block sums over 64 h (32 virtual px; a pixel that straddles a border is
    -- split by its h weights) -> a vertical one-pole per block column (1/16
    -- per line at 1080p, 1/8 at 2-px units, 1/4 at 480i / 576i; loaded on the
    -- field's first line) -> x gain -> grid bank s_bank, rewritten on every
    -- line so the row keeps its last line's value.  The read side follows the
    -- great star's pixel: its own block, registered twice, then the echo
    -- line's half width.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_np : unsigned(6 downto 0);
        variable v_r : unsigned(2 downto 0);
        variable v_dq : signed(14 downto 0);
        variable v_n : signed(15 downto 0);
        variable v_g : unsigned(15 downto 0);
        variable v_cp : unsigned(6 downto 0);
        variable v_h3 : unsigned(9 downto 0);
        variable v_sc : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then   -- both phases restart from this field's drift
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);    -- ot * 96
            end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if s_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid;

            -- the first line of a field loads the one-pole instead of stepping it
            if s_zone = '1' then
                s_fl <= '1'; s_seen <= '0';
            elsif w_v1 = '1' and w_v2 = '0' then           -- (a line's start, after the zone flag has dropped)
                if s_seen = '1' then s_fl <= '0'; end if;
                s_seen <= '1';
            end if;

            -- W1: the sample's weights in this block and the next
            if s_sv = '1' then
                v_np := ('0' & w_ph) + resize(s_xs, 7);
                w_ph <= v_np(5 downto 0);
                if v_np(6) = '1' then
                    v_r := v_np(2 downto 0);
                    w_cy <= '1'; w_wa <= s_xs - v_r; w_wb <= v_r;
                else
                    w_cy <= '0'; w_wa <= s_xs; w_wb <= (others => '0');
                end if;
            else
                w_ph <= (others => '0'); w_cy <= '0';
            end if;
            w_l <= s_sl; w_v1 <= s_sv;
            -- W2
            w_pa <= f_mulw(w_l, w_wa); w_pb <= f_mulw(w_l, w_wb);
            w_cy2 <= w_cy and w_v1; w_v2 <= w_v1;
            -- W3: the block sum
            w_dv <= '0';
            if w_v2 = '1' then
                if w_cy2 = '1' then
                    w_dump <= w_acc + resize(w_pa, 14); w_acc <= resize(w_pb, 14);
                    w_dv <= '1'; w_dcol <= w_col; w_col <= w_col + 1;
                else
                    w_acc <= w_acc + resize(w_pa, 14);
                end if;
            else
                w_acc <= (others => '0'); w_col <= (others => '0');
            end if;
            -- W4: the one-pole's state of that column (read while the block was summed)
            iq_f <= iq_q;
            w_diff <= signed(resize(w_dump, 15)) - signed(resize(iq_f(13 downto 0), 15));
            w_dd <= w_dump; w_qd <= iq_f(13 downto 0); w_v4 <= w_dv; w_c4 <= w_dcol;
            -- W5
            case s_ush is
                when "00"   => v_dq := shift_right(w_diff, 4);
                when "01"   => v_dq := shift_right(w_diff, 3);
                when others => v_dq := shift_right(w_diff, 2);
            end case;
            v_n := signed(resize(w_qd, 16)) + resize(v_dq, 16);
            if s_fl = '1' then w_qn <= w_dd; else w_qn <= unsigned(v_n(13 downto 0)); end if;
            iq_we <= w_v4; w_v5 <= w_v4; w_c5 <= w_c4;
            -- W6: x the camera gain (two 8 x 4 products)
            w_glo <= w_qn(13 downto 6) * c_g6(3 downto 0);
            w_ghi <= w_qn(13 downto 6) * c_g6(7 downto 4);
            w_v6 <= w_v5; w_c6 <= w_c5;
            -- W7
            v_g := resize(w_glo, 16) + shift_left(resize(w_ghi, 16), 4);
            if v_g(15) = '1' then gr_wd <= (others => '1'); else gr_wd <= v_g(14 downto 7); end if;
            gr_we <= w_v6;
            gr_wa <= s_bank & (r_rb + resize(w_c6, 11));

            -- the read side: the block of the great star's pixel (DDA index i at T + 10 + i here; the grid's
            -- column of screen column i - 5)
            if r_run(4) = '1' then
                if cm_i /= 5 then
                    cm_i <= cm_i + 1;
                else
                    v_cp := ('0' & cm_ph) + resize(s_xs, 7);
                    cm_ph <= v_cp(5 downto 0);
                    if v_cp(6) = '1' then cm_col <= cm_col + 1; end if;
                end if;
            else
                cm_i <= (others => '0'); cm_ph <= (others => '0'); cm_col <= (others => '0');
            end if;
            gr_ra <= (not s_bank) & (r_rb + resize(cm_col, 11));
            gr_f <= gr_q;
            -- echo half width, 1/8 unit: 3/16 of the held value >> ush (Fine: half); 480i / 576i as the 2-px unit
            v_h3 := (gr_f & "00") - resize(gr_f, 10);
            if s_ush = 0 then v_sc := "000"; else v_sc := "001"; end if;
            if s_fine = '1' then v_sc := v_sc + 1; end if;
            case to_integer(v_sc) is
                when 0      => cm_thr <= v_h3(9 downto 4);
                when 1      => cm_thr <= resize(v_h3(9 downto 5), 6);
                when others => cm_thr <= resize(v_h3(9 downto 6), 6);
            end case;
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [G] BLEND.  Echo band: the great star's E modulo 16 virtual px (32 at 480i / 576i), a band
    -- of half width cm_thr about the period's middle (both edges, so a zero
    -- weight draws nothing).  Grey: the two topmost stars that touch the pixel
    -- are blended by their coverage over the third (taken whole) or over the
    -- echo.  Ink: the coverages summed modulo 2 (Fine: the lines' union) as one
    -- white layer over the echo.
    ---------------------------------------------------------------------------
    p_blend : process(clk)
        variable v_em : unsigned(6 downto 0);
        variable v_xe, v_nx : signed(7 downto 0);
        variable v_t, v_ax : signed(8 downto 0);
        variable v_sum : unsigned(4 downto 0);
        variable v_nz : std_logic_vector(3 downto 0);
        variable v_t1, v_t2, v_t3, v_eb : unsigned(7 downto 0);
        variable v_c1, v_c2, v_i, v_m : unsigned(3 downto 0);
        variable v_h3 : std_logic;
        variable v_k : integer range 0 to 3;
        variable v_p1, v_p2, v_p3 : signed(12 downto 0);
        variable v_o : signed(12 downto 0);
        variable v_v : unsigned(7 downto 0);
        variable v_y : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            -- e1: the distance from the band's middle
            case s_ush is
                when "00"   => v_em := st_em(0);                    v_xe := signed(resize(v_em, 8)) - 64;
                when "01"   => v_em := '0' & st_em(0)(5 downto 0);  v_xe := signed(resize(v_em, 8)) - 32;
                when others => v_em := '0' & st_em(0)(5 downto 0);  v_xe := signed(resize(v_em, 8)) - 32;
            end case;
            v_nx := -v_xe;
            if v_xe(7) = '1' then e1_ax <= unsigned(v_nx(6 downto 0)); else e1_ax <= unsigned(v_xe(6 downto 0)); end if;
            -- e2, e3
            v_t := signed(resize(cm_thr, 9)) + 4;
            v_ax := signed(resize(e1_ax, 9));
            e2_a <= f_cl8(v_t - v_ax);
            e2_b <= f_cl8(v_t + v_ax);
            v_sum := resize(e2_a, 5) + resize(e2_b, 5);
            if v_sum(4) = '1' then e3_cb <= to_unsigned(8, 4);
            elsif v_sum(3) = '1' then e3_cb <= '0' & v_sum(2 downto 0);
            else e3_cb <= (others => '0'); end if;

            -- g1: every star's tone
            for j in 0 to 3 loop
                g1_tone(j) <= f_tone(j, s_fine, st_x(j));
                g1_cs(j) <= st_cs(j); g1_x(j) <= st_x(j);
            end loop;

            -- g2: the first, second and third star (from the top) with any coverage; the echo's tone
            for j in 0 to 3 loop
                if g1_cs(j) /= 0 then v_nz(j) := '1'; else v_nz(j) := '0'; end if;
            end loop;
            v_t1 := (others => '0'); v_c1 := (others => '0'); v_t2 := (others => '0'); v_c2 := (others => '0');
            v_t3 := (others => '0'); v_h3 := '0';
            v_k := 0;
            for j in 0 to 3 loop
                if v_nz(j) = '1' then
                    if v_k = 0 then v_t1 := g1_tone(j); v_c1 := g1_cs(j);
                    elsif v_k = 1 then v_t2 := g1_tone(j); v_c2 := g1_cs(j);
                    elsif v_k = 2 then v_t3 := g1_tone(j); v_h3 := '1'; end if;
                    if v_k /= 3 then v_k := v_k + 1; end if;
                end if;
            end loop;
            if s_ink = '1' then
                -- the even-odd value of the summed coverage (Fine: the largest line coverage)
                if s_fine = '0' then
                    v_m := g1_cs(0) + g1_cs(1) + g1_cs(2) + g1_cs(3);
                    if v_m > 8 then v_i := 0 - v_m; else v_i := v_m; end if;
                else
                    if g1_x(0) > g1_x(1) then v_i := g1_x(0); else v_i := g1_x(1); end if;
                    if g1_x(2) > g1_x(3) then v_m := g1_x(2); else v_m := g1_x(3); end if;
                    if v_m > v_i then v_i := v_m; end if;
                end if;
                if e3_cb(3) = '1' then v_eb := (others => '1'); else v_eb := e3_cb(2 downto 0) & "00000"; end if;
                g2_t1 <= (others => '1'); g2_c1 <= v_i; g2_t2 <= (others => '0'); g2_c2 <= (others => '0');
                g2_base <= v_eb;
            else
                v_eb := resize(e3_cb & "00000", 8) - resize(e3_cb & "000", 8);      -- 24 x: 3/4 white
                g2_t1 <= v_t1; g2_c1 <= v_c1; g2_t2 <= v_t2; g2_c2 <= v_c2;
                if v_h3 = '1' then g2_base <= v_t3; else g2_base <= v_eb; end if;
            end if;

            -- g3 .. g5: the second star over the base
            g3_d <= signed(resize(g2_t2, 9)) - signed(resize(g2_base, 9));
            g3_base <= g2_base; g3_c2 <= g2_c2; g3_t1 <= g2_t1; g3_c1 <= g2_c1;
            if g3_c2(3) = '1' then v_p1 := shift_left(resize(g3_d, 13), 3);
            elsif g3_c2(2) = '1' then v_p1 := shift_left(resize(g3_d, 13), 2);
            else v_p1 := (others => '0'); end if;
            if g3_c2(1) = '1' then v_p2 := shift_left(resize(g3_d, 13), 1); else v_p2 := (others => '0'); end if;
            if g3_c2(0) = '1' then v_p3 := resize(g3_d, 13); else v_p3 := (others => '0'); end if;
            g4_p <= v_p1 + v_p2 + v_p3;
            g4_base <= g3_base; g4_t1 <= g3_t1; g4_c1 <= g3_c1;
            v_o := signed(resize(g4_base, 13)) + shift_right(g4_p, 3);
            v_v := unsigned(v_o(7 downto 0));
            g5_v <= v_v;
            g5_d <= signed(resize(g4_t1, 9)) - signed(resize(v_v, 9));
            g5_c1 <= g4_c1;
            -- g6, g7: the first star over that
            if g5_c1(3) = '1' then v_p1 := shift_left(resize(g5_d, 13), 3);
            elsif g5_c1(2) = '1' then v_p1 := shift_left(resize(g5_d, 13), 2);
            else v_p1 := (others => '0'); end if;
            if g5_c1(1) = '1' then v_p2 := shift_left(resize(g5_d, 13), 1); else v_p2 := (others => '0'); end if;
            if g5_c1(0) = '1' then v_p3 := resize(g5_d, 13); else v_p3 := (others => '0'); end if;
            g6_p <= v_p1 + v_p2 + v_p3;
            g6_v <= g5_v;
            v_o := signed(resize(g6_v, 13)) + shift_right(g6_p, 3);
            g7_tone <= unsigned(v_o(7 downto 0));
            -- g8: luma = 64 + tone x 55 / 16
            v_y := resize(g7_tone & "000000", 14) - resize(g7_tone & "000", 14) - resize(g7_tone, 14);
            s_y23 <= resize(v_y(13 downto 4), 10) + 64;
        end if;
    end process p_blend;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING: luma [1 2 1]/4, luma 13/16 toward black, blanking
    -- gate.  Chroma is 512 on every pixel.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_dl : unsigned(9 downto 0);
        variable v_y  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_yd1 <= s_y23; s_yd2 <= s_yd1;
            s_ya <= resize(s_y23, 12) + resize(s_yd2, 12) + 2;
            s_yb <= s_yd1;
            v_y := s_ya + shift_left(resize(s_yb, 12), 1);
            s_y26 <= v_y(11 downto 2);
            if s_y26 > 64 then v_dl := s_y26 - 64; else v_dl := (others => '0'); end if;
            s_y27 <= s_y26 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y28 <= s_y27;
            else
                s_y28 <= to_unsigned(64, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y28);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture windrose;
