-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Stitch v0.1.0 -- Straightedge.  Curve stitching / string art (Mary Everest Boole's sewing cards, Naum Gabo's
-- strung constructions): families of perfectly straight lines whose envelope the eye reads as a curve.  Every
-- edge is straight; the curves exist only where the lines crowd.  Luma only (U = V = 512).
--
--   fold:     x' = |x - X|, y' = |y - Y| (one family serves all four quadrants)
--   line l:   normal angle phi_l = phi0 + (l + f) pi / N (N = 8 / 12 / 16 / 24, f = the drift fraction)
--     Corners  the corner of an implied square at (H, H): d = c (H - x') + s (H - y') - L s c / (s + c), the
--              tangents of the parabola sqrt(u) + sqrt(v) = sqrt(L), phi in (-pi/4, 3 pi/4) (both ends at infinity)
--     Cross    d = c (x' + Hc) + s (y' + Hc) - sqrt(2k |sin 2 phi|): the tangents of the hyperbolas u v = +-k,
--              phi in (-pi/2, pi/2) (both ends on the fold axes)
--     every d is one affine DDA (Q12 of the anti-aliasing unit), folded by reversing its step at X and at Y
--   coverage: Bold clamp(d + 1/2), Fine clamp(T - |d| + 1/2) (1/8 steps); S = the sum over the lines
--   tone:     a per-field table of (camera level, S): Grey = the count mod 4 as black / dark / light / white,
--             Ink = the parity, Fine Grey = films; the camera lifts the tones (Ink: flips the parity) block by block
--
--   K1  X        the fold centre, +-640 virtual px                                       (glided)
--   K2  Y        +-360 virtual px, up = up                                                (glided)
--   K3  Scale    the implied square H = 117 .. 330 .. 536 virtual px, exponential        (glided)
--   K4  Pins     8 / 12 / 16 / 24 lines per quadrant (stepped, hysteresis, both ends reachable)
--   K5  Drift    bipolar speed: still at the centre, 2^-12 .. 2^-4 line step per field; wraps at one step
--   K6  Camera   how far the held block level lifts the tones (Ink: how many levels flip) (glided)
--   S7  Glide    Off / On       S8  Form  Corners / Cross     S9  Tone  Grey / Ink
--   S10 Weight   Bold / Fine    S11 Source  Video / Osc
--   P12 SPAN     Corners: the pin span L = 5.5 H .. 2 H .. 0.73 H (a small star in big checks .. the classic ..
--                a wide clear centre);  Cross: Hc = 0.5 .. 0 .. -1.25 sqrt(k)                 (glided)
--
-- Sections (the integer model .straightedge_work/rb_stitch/imodel.py mirrors each one):
--   [A] TIMING            timing-ID steps, vblank zone, width, line count, generated avid, sync delay
--   [B] CONTROLS          switches, Pins hysteresis
--   [C] FIELD ENGINE      Hardline flute's microcoded engine (stitch.py): holds, glides, drift, sin / cos, every
--                         line's step / wraps / seed, the tone table -> one bank; the picture uses the bank written
--                         during the field before
--   [D] SOURCE FIELD      luma or the Osc lattice -> block sums -> vertical one-pole -> 4 levels -> grid (two banks)
--   [E] DDAS              24 accumulators with their own increment registers, the table loader, the folds
--   [P] LINES             per line coverage
--   [G] SUM AND TONE      adder tree, tone table
--   [F] OUTPUT            [1 2 1] / 4, 13/16 toward black, blanking gate
--
-- Timing: T = the g_start clock.  The accumulators hold DDA index i during T + 6 + i, the coverage T + 8 + i, the
-- sum T + 12 + i, the luma T + 15 + i; output column c is the [1 2 1] filter centred on DDA index c + 1.  One
-- latency (20) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.stitch_pkg.all;
use work.all;

architecture stitch of program_top is
    constant C_LAT  : integer := 20;   -- = processing_delay_clks
    constant C_NEXT : integer := 6;    -- DDA steps per line = width + C_NEXT (stitch.py NEXTRA)
    constant C_NL   : integer := 24;   -- lines (stitch.py NL)

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

    -- [B] switches (latched per field; the pixel path's copies one field later, matching the table), Pins
    signal s_glide : std_logic := '1';
    signal s_cross, s_ink, s_fine, s_src : std_logic := '0';
    signal p_ink, p_fine : std_logic := '0';
    signal s_zn : unsigned(1 downto 0) := to_unsigned(3, 2);       -- Pins: 8 / 12 / 16 / 24
    signal z_up, z_dn : std_logic := '0';
    signal s_k4 : unsigned(9 downto 0) := (others => '0');

    -- ROM: the engine's table (2^(i/256), quarter sine, reciprocals, sqrt of sine)
    signal r_ex_a : unsigned(9 downto 0) := (others => '0');
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
    signal rf_ram : t_c_st_rfi := C_ST_RFI;
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

    -- [C][E] the line table: two banks of 256 words; the engine writes bank s_bank, the loader reads the other
    type t_dt is array (0 to 511) of std_logic_vector(31 downto 0);
    signal dt_ram : t_dt := (others => (others => '0'));
    attribute no_rw_check of dt_ram : signal is true;
    signal dt_q, dt_wd : std_logic_vector(31 downto 0) := (others => '0');
    signal dt_ra : unsigned(7 downto 0) := (others => '0');
    signal dt_wa : unsigned(8 downto 0) := (others => '0');
    signal dt_we : std_logic := '0';

    -- [G] the tone table: two banks of 4 levels x 32 sums, 10-bit luma
    type t_tn is array (0 to 255) of unsigned(9 downto 0);
    signal tn_ram : t_tn := (others => to_unsigned(64, 10));
    attribute no_rw_check of tn_ram : signal is true;
    signal tn_q : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal tn_ra : unsigned(7 downto 0) := (others => '0');

    -- [E] the field's constants (from the table, at the zone start)
    signal c_n0x, c_n0y : unsigned(11 downto 0) := (others => '0');
    signal c_ox, c_oy : std_logic := '0';
    signal c_t84 : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal f_run, f_v1, f_v2 : std_logic := '0';
    signal f_t, f_i1, f_i2 : unsigned(1 downto 0) := (others => '0');

    -- [E] DDAs and their loader
    type t_s25 is array (0 to C_NL - 1) of signed(24 downto 0);
    signal ca_inc, ca_acc : t_s25 := (others => (others => '0'));
    signal ca_ld, ca_en : std_logic_vector(C_NL - 1 downto 0) := (others => '0');
    signal ca_rst : std_logic := '0';
    signal x_en, x_fl, x_rn, x_neg : std_logic := '0';
    signal l_kick, l_run : std_logic := '0';
    signal l_t : unsigned(5 downto 0) := (others => '0');
    signal l_word : std_logic_vector(2 downto 0) := "010";
    signal l_v1, l_v2, l_o1, l_o2 : std_logic := '0';
    signal l_c1, l_c2 : unsigned(4 downto 0) := (others => '0');
    signal b_run, b_end : std_logic := '0';
    signal b_cnt, b_mm1 : unsigned(11 downto 0) := (others => '0');
    signal fx_cnt, fy_cnt : unsigned(11 downto 0) := (others => '1');
    signal fy_neg : std_logic := '0';
    signal b_hold, b_flip : std_logic;
    signal r_run : std_logic_vector(0 to 3) := (others => '0');

    -- [P] lines
    type t_s7 is array (0 to C_NL - 1) of signed(6 downto 0);
    signal c1_b : t_s7 := (others => (others => '0'));
    type t_u4 is array (0 to C_NL - 1) of unsigned(3 downto 0);
    signal c2_c : t_u4 := (others => (others => '0'));

    -- [G] sum
    type t_u5x12 is array (0 to 11) of unsigned(4 downto 0);
    signal s3 : t_u5x12 := (others => (others => '0'));
    type t_u6x6 is array (0 to 5) of unsigned(5 downto 0);
    signal s4 : t_u6x6 := (others => (others => '0'));
    type t_u7x3 is array (0 to 2) of unsigned(6 downto 0);
    signal s5 : t_u7x3 := (others => (others => '0'));
    signal s6 : unsigned(7 downto 0) := (others => '0');

    -- [D] camera grid: the read side
    signal r_vy : unsigned(12 downto 0) := (others => '0');
    signal r_rb : unsigned(10 downto 0) := (others => '0');        -- the line's block row x 60
    signal r_first : std_logic := '1';
    signal cm_ph : unsigned(5 downto 0) := (others => '0');
    signal cm_col : unsigned(5 downto 0) := (others => '0');
    signal cm_i : std_logic := '0';
    type t_gr is array (0 to 4095) of unsigned(1 downto 0);
    signal gr_ram : t_gr := (others => (others => '0'));
    attribute no_rw_check of gr_ram : signal is true;
    signal gr_ra, gr_wa : unsigned(11 downto 0) := (others => '0');
    signal gr_q, gr_f, gr_wd : unsigned(1 downto 0) := (others => '0');
    signal gr_we : std_logic := '0';

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
    signal w_cy, w_v1, w_cy2, w_v2, w_dv, w_v4, w_v5 : std_logic := '0';
    signal w_pa, w_pb : unsigned(10 downto 0) := (others => '0');
    signal w_acc, w_dump, w_dd, w_qd, w_qn : unsigned(13 downto 0) := (others => '0');
    signal w_col, w_dcol, w_c4, w_c5 : unsigned(5 downto 0) := (others => '0');
    signal w_diff : signed(14 downto 0) := (others => '0');
    type t_iq is array (0 to 63) of unsigned(15 downto 0);
    signal iq_ram : t_iq := (others => (others => '0'));
    attribute no_rw_check of iq_ram : signal is true;
    signal iq_q, iq_f : unsigned(15 downto 0) := (others => '0');
    signal iq_we : std_logic := '0';

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
            r_ex_q <= C_ST_TAB(to_integer(r_ex_a));
        end if;
    end process p_rom;

    -- line table: written by the engine (bank s_bank, addresses 0 .. 255), read by the loader (the other bank)
    p_dtram : process(clk)
    begin
        if rising_edge(clk) then
            dt_q <= dt_ram(to_integer((not s_bank) & dt_ra));
            if dt_we = '1' and dt_wa(8) = '0' then dt_ram(to_integer(s_bank & dt_wa(7 downto 0))) <= dt_wd; end if;
        end if;
    end process p_dtram;

    -- tone table: written by the engine (bank s_bank, addresses 256 .. 383), read by the pixel path
    p_tnram : process(clk)
    begin
        if rising_edge(clk) then
            tn_q <= tn_ram(to_integer(tn_ra));
            if dt_we = '1' and dt_wa(8) = '1' then
                tn_ram(to_integer(s_bank & dt_wa(6 downto 0))) <= unsigned(dt_wd(9 downto 0));
            end if;
        end if;
    end process p_tnram;

    -- camera grid: 2 banks x 34 rows x 60 blocks, the 2-bit level
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

            -- [B] Pins: 4 steps of 256; a step is left only 8 counts past its edge (the outer steps are entered
            -- from inside, so a knob resting at either end reaches them).  The comparison runs free; the latch
            -- is per field.
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
                s_cross <= registers_in(6)(1);
                s_ink   <= registers_in(6)(2);
                s_fine  <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                p_ink   <= s_ink;                 -- the pixel path draws with the table of the field before
                p_fine  <= s_fine;
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
    -- subtracting.  The microprogram (stitch.py) does the knob holds and
    -- glides, the drift, every line's angle, sin / cos and conic offset, its
    -- step, wraps and seed, the folds and the tone table; it writes them to
    -- bank s_bank while the other bank is being drawn.
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
                            when "010"  => v_cond := s_cross;
                            when "011"  => v_cond := s_ink;
                            when "100"  => v_cond := s_fine;
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
                e_ir <= C_ST_UC(to_integer(e_pc));
            end if;

            -- the engine's outputs: the ROM's address, the table's write address and data
            dt_we <= '0';
            if dt_we = '1' then dt_wa <= dt_wa + 1; end if;
            if e_out = '1' then
                case e_irf(1 downto 0) is
                    when "00"   => r_ex_a <= unsigned(e_acc(9 downto 0));
                    when "01"   => dt_wa <= unsigned(e_acc(8 downto 0));
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
    -- [E] DDAS.  Twenty-four accumulators (Q12 of the anti-aliasing unit, 25
    -- bits), each with its own increment register: the pixel step on every
    -- clock of the line window (width + 6 steps), then one add of the line
    -- wrap (line step - the window's net pixel steps) parked in the same
    -- register, so the adder has no input mux.  At the x fold the increment
    -- registers are complemented (carry-in 1 = negated), with one held clock
    -- when the fold falls between two pixels; vertically the loader picks the
    -- wrap word for a forward, a backward or a held line.  The zone start
    -- loads the field's constants and seeds every accumulator from the bank
    -- the engine finished during the field before.
    ---------------------------------------------------------------------------
    b_flip <= '1' when b_run = '1' and fx_cnt = 1 else '0';
    b_hold <= b_flip and c_ox;

    p_load : process(clk)
    begin
        if rising_edge(clk) then
            -- line window and the x fold (one flip per line)
            b_end <= '0';
            if s_st(3) = '1' then
                b_run <= '1'; b_cnt <= (others => '0');
            elsif b_run = '1' then
                b_cnt <= b_cnt + 1;
                if b_cnt = b_mm1 then b_run <= '0'; b_end <= '1'; end if;
            end if;
            if b_run = '0' then fx_cnt <= c_n0x;
            elsif fx_cnt = 1 then fx_cnt <= (others => '1');
            else fx_cnt <= fx_cnt - 1; end if;
            x_en <= b_run and not b_hold; x_fl <= b_flip; x_rn <= b_run;
            if x_rn = '0' then x_neg <= '0';
            elsif x_fl = '1' then x_neg <= not x_neg; end if;
            r_run <= b_run & r_run(0 to 2);

            -- the zone start: three constants words, then the seeds
            if s_zgo = '1' then
                f_run <= '1'; f_t <= (others => '0');
            elsif f_run = '1' then
                f_t <= f_t + 1;
                if f_t = 3 then f_run <= '0'; end if;
            end if;
            f_v1 <= f_run; f_i1 <= f_t; f_v2 <= f_v1; f_i2 <= f_i1;
            l_kick <= '0';
            if f_v2 = '1' then
                case to_integer(f_i2) is
                    when 0 => c_n0x <= unsigned(dt_q(11 downto 0)); c_ox <= dt_q(12);
                    when 1 => c_n0y <= unsigned(dt_q(11 downto 0)); c_oy <= dt_q(12);
                    when 2 => c_t84 <= unsigned(dt_q(5 downto 0));
                    when others => l_kick <= '1';
                end case;
            end if;

            -- the y fold: which wrap word the coming line step takes
            if l_kick = '1' then
                fy_cnt <= c_n0y; fy_neg <= '0'; l_word <= "010";
            elsif b_end = '1' then
                if fy_cnt = 1 and c_oy = '1' then l_word <= "100";
                elsif fy_neg = '1' then l_word <= "011";
                else l_word <= "001"; end if;
                if fy_cnt = 1 then fy_neg <= not fy_neg; fy_cnt <= (others => '1');
                else fy_cnt <= fy_cnt - 1; end if;
            end if;

            -- loader: 48 table reads, line by line: (seed or wrap), (step)
            ca_rst <= l_kick;
            if l_kick = '1' or b_end = '1' then
                l_run <= '1'; l_t <= (others => '0');
            elsif l_run = '1' then
                l_t <= l_t + 1;
                if l_t = 2 * C_NL - 1 then l_run <= '0'; end if;
            end if;
            if f_run = '1' then dt_ra <= "110000" & f_t;
            elsif l_t(0) = '1' then dt_ra <= l_t(5 downto 1) & "000";
            else dt_ra <= l_t(5 downto 1) & unsigned(l_word); end if;
            -- the word addressed now is in dt_q two clocks on: it goes into the accumulator's increment
            -- register then (ca_ld), and a wrap / seed word is added one clock after that (ca_en), on the
            -- same edge that puts the pixel step back
            l_v1 <= l_run; l_c1 <= l_t(5 downto 1); l_o1 <= l_t(0);
            l_v2 <= l_v1;  l_c2 <= l_c1;            l_o2 <= l_o1;
            for k in 0 to C_NL - 1 loop
                if l_v1 = '1' and l_c1 = k then ca_ld(k) <= '1'; else ca_ld(k) <= '0'; end if;
                if l_v2 = '1' and l_o2 = '0' and l_c2 = k then ca_en(k) <= '1'; else ca_en(k) <= '0'; end if;
            end loop;
        end if;
    end process p_load;

    g_dda : for d in 0 to C_NL - 1 generate
        p_dda : process(clk)
            variable v_s : unsigned(25 downto 0);
        begin
            if rising_edge(clk) then
                if ca_ld(d) = '1' then ca_inc(d) <= signed(dt_q(24 downto 0));
                elsif x_fl = '1' then ca_inc(d) <= not ca_inc(d); end if;
                if ca_rst = '1' then
                    ca_acc(d) <= (others => '0');
                elsif x_en = '1' or ca_en(d) = '1' then
                    v_s := (unsigned(ca_acc(d)) & '1') + (unsigned(ca_inc(d)) & x_neg);
                    ca_acc(d) <= signed(v_s(25 downto 1));
                end if;
            end if;
        end process p_dda;
    end generate g_dda;

    ---------------------------------------------------------------------------
    -- [P] LINES.  c1: the distance in 1/8 unit (accumulator bits 24 .. 9)
    -- saturated to +-64, Fine: its magnitude (one's complement); c2: the
    -- coverage 0 .. 8: Bold clamp(d + 4), Fine clamp(T + 4 - |d|).
    ---------------------------------------------------------------------------
    g_line : for l in 0 to C_NL - 1 generate
        p_line : process(clk)
            variable v_v : signed(6 downto 0);
            variable v_x : signed(8 downto 0);
        begin
            if rising_edge(clk) then
                -- c1
                if ca_acc(l)(24 downto 15) = "0000000000" or ca_acc(l)(24 downto 15) = "1111111111" then
                    v_v := ca_acc(l)(15 downto 9);
                elsif ca_acc(l)(24) = '0' then
                    v_v := to_signed(63, 7);
                else
                    v_v := to_signed(-64, 7);
                end if;
                if p_fine = '1' and v_v(6) = '1' then c1_b(l) <= not v_v;
                else c1_b(l) <= v_v; end if;
                -- c2
                if p_fine = '1' then
                    v_x := signed(resize(c_t84, 9)) - resize(c1_b(l), 9);
                else
                    v_x := resize(c1_b(l), 9) + 4;
                end if;
                if v_x(8) = '1' then c2_c(l) <= (others => '0');
                elsif v_x(7 downto 3) /= "00000" then c2_c(l) <= to_unsigned(8, 4);
                else c2_c(l) <= '0' & unsigned(v_x(2 downto 0)); end if;
            end if;
        end process p_line;
    end generate g_line;

    ---------------------------------------------------------------------------
    -- [G] SUM AND TONE.  S = the sum of the 24 coverages (four stages); the
    -- tone table's address = the camera level & (Fine Grey: min(S, 31), else
    -- S mod 32); its output register, then a fabric copy.
    ---------------------------------------------------------------------------
    p_sum : process(clk)
    begin
        if rising_edge(clk) then
            for p in 0 to 11 loop
                s3(p) <= resize(c2_c(2 * p), 5) + resize(c2_c(2 * p + 1), 5);
            end loop;
            for p in 0 to 5 loop
                s4(p) <= resize(s3(2 * p), 6) + resize(s3(2 * p + 1), 6);
            end loop;
            for p in 0 to 2 loop
                s5(p) <= resize(s4(2 * p), 7) + resize(s4(2 * p + 1), 7);
            end loop;
            s6 <= resize(s5(0), 8) + resize(s5(1), 8) + resize(s5(2), 8);
            -- the tone table's address (camera level from the grid, registered twice)
            if p_fine = '1' and p_ink = '0' and s6(7 downto 5) /= "000" then
                tn_ra <= (not s_bank) & gr_f & "11111";
            else
                tn_ra <= (not s_bank) & gr_f & s6(4 downto 0);
            end if;
            s_y23 <= tn_q;
        end if;
    end process p_sum;

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD -> the camera grid.  luma (or the Osc lattice) -> exact
    -- block sums over 64 h (32 virtual px; a pixel that straddles a border is
    -- split by its h weights) -> a vertical one-pole per block column (1/16
    -- per line at 1080p, 1/8 at 2-px units, 1/4 at 480i / 576i; loaded on the
    -- field's first line) -> 4 levels (block mean > 72, > 103, > 136 of 219)
    -- -> grid bank s_bank, rewritten on every line so the row keeps its last
    -- line's value.  The read side follows the pixel: its own block (screen
    -- column = DDA index - 1), registered twice.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l : unsigned(7 downto 0);
        variable v_s : unsigned(7 downto 0);
        variable v_np : unsigned(6 downto 0);
        variable v_r : unsigned(2 downto 0);
        variable v_dq : signed(14 downto 0);
        variable v_n : signed(15 downto 0);
        variable v_m : unsigned(7 downto 0);
        variable v_cp : unsigned(6 downto 0);
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
            -- W6: the level
            v_m := w_qn(13 downto 6);
            if v_m > 136 then gr_wd <= "11";
            elsif v_m > 103 then gr_wd <= "10";
            elsif v_m > 72 then gr_wd <= "01";
            else gr_wd <= "00"; end if;
            gr_we <= w_v5;
            gr_wa <= s_bank & (r_rb + resize(w_c5, 11));

            -- the read side: DDA index i's block column during T + 9 + i (index 0 and 1: column 0)
            if r_run(3) = '1' then
                if cm_i = '0' then
                    cm_i <= '1';
                else
                    v_cp := ('0' & cm_ph) + resize(s_xs, 7);
                    cm_ph <= v_cp(5 downto 0);
                    if v_cp(6) = '1' then cm_col <= cm_col + 1; end if;
                end if;
            else
                cm_i <= '0'; cm_ph <= (others => '0'); cm_col <= (others => '0');
            end if;
            gr_ra <= (not s_bank) & (r_rb + resize(cm_col, 11));
            gr_f <= gr_q;
        end if;
    end process p_src;

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

end architecture stitch;
