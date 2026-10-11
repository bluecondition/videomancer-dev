-- Solitude: Glitch Cascade on Teal  (v0.3)
--
-- A ragged mass of crawling pixel-sort streaks in jewel tones floats on a
-- flat muted teal background.  Every stripe column is cut into runs of its
-- own length (32..512 rows); each run is one colour shading dark -> bright
-- down its length, the way a brightness sort leaves a column, and the runs
-- crawl at per-column speeds.  Slanted colour passes (bands of y - x/2)
-- separated by jagged black seams shift the palette from pass to pass.
-- The top edge is a sloped, stepped skyline with towers; the side edges
-- break into columns of uneven height; drip columns stretch out of the
-- bottom, detach and fall to the floor.  White lightning dashes and speckle
-- cap the top edge and re-strike as the picture flows; whole rows flip to
-- inverted colour.
--
-- Controls (latched once per field):
--   K1 Edge       tower height on the skyline top edge
--   K2 Drip       how many columns drip
--   K3 Lit Dns    lightning density near the top edge
--   K4 Flow       crawl speed of runs and drips, lightning re-strike rate
--   K5 Stripe     stripe width 1/2/4/8 px
--   K6 Lit Depth  how far down the lightning reaches
--   S7 Palette    Jewel / Cool
--   S8 BG         Teal / Black
--   S9 Seams      Off / On: slanted colour passes with black seams
--   S10 Lightning Off / On
--   S11 Sort      Down / Up: run shading and crawl direction
--   P12 Bright    brightness, 50% = unity
--
-- Colours are authored in BT.601 and swapped U<->V at the output compose
-- (the hardware swap).  The program makes its own avid (cubist/aurora
-- recipe) and blanks to 64/512/512 outside it.  Geometry comes from the
-- measured active raster: rows count active lines from vsync (frame lines
-- under interlace), and feature scales double at HD.
--
-- Latency: 18 clocks, constant across all modes.  EBR: 0.
--
-- License: GPL-3.0
-- Author: ron

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture solitude of program_top is

    constant C_LAT : natural := 18;

    subtype t_bsr is std_logic_vector(1 to C_LAT - 1);

    -- teal background, BT.601 (swapped at the output like everything else)
    constant C_TEAL_Y64 : unsigned(9 downto 0) := to_unsigned(504, 10);  -- Y 568
    constant C_TEAL_U   : unsigned(9 downto 0) := to_unsigned(608, 10);
    constant C_TEAL_V   : unsigned(9 downto 0) := to_unsigned(300, 10);
    constant C_WHITE64  : unsigned(9 downto 0) := to_unsigned(876, 10);  -- Y 940
    constant C_MID      : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- palette slot per (palette, zone, colour bin): index = cool & zone & bin
    type t_nat64 is array (0 to 63) of natural range 0 to 7;
    constant C_PALIDX : t_nat64 := (
        0, 0, 1, 1, 5, 3, 6, 7,   0, 3, 1, 2, 3, 3, 6, 7,     -- jewel
        2, 3, 2, 0, 3, 6, 4, 7,   2, 2, 4, 3, 5, 6, 4, 7,
        2, 3, 2, 4, 3, 7, 6, 7,   2, 2, 4, 3, 3, 7, 6, 4,     -- cool
        4, 2, 7, 4, 3, 2, 6, 7,   2, 4, 4, 2, 3, 7, 6, 4);

    -- per-pass stripe seeds (pass 0 doubles as the seamless seed)
    constant C_SEED0 : unsigned(15 downto 0) := x"A3B1";
    constant C_SEED1 : unsigned(15 downto 0) := x"7C2D";
    constant C_SEED2 : unsigned(15 downto 0) := x"D4A9";
    constant C_SEED3 : unsigned(15 downto 0) := x"6E3F";

    -- three add-based mixing rounds (checked numerically: neighbouring
    -- inputs give independent bits, unlike the old xor-only shuffle)
    function f_r1(h : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := h + shift_left(h, 5);
        return t xor shift_right(t, 7);
    end function;
    function f_r2(h : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := h + shift_left(h, 3);
        return t xor shift_right(t, 9);
    end function;
    function f_r3(h : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := h + shift_left(h, 6);
        return t xor shift_right(t, 5);
    end function;

    -- timing / raster measurement
    signal s_timing : t_video_timing_port;
    signal s_hcnt   : unsigned(11 downto 0) := (others => '0');
    signal s_meas_h : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_vcnt   : unsigned(11 downto 0) := (others => '1');
    signal s_meas_v : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal s_ilace  : std_logic := '0';
    signal s_botf   : std_logic := '0';
    signal s_row    : unsigned(11 downto 0) := (others => '0');
    signal s_saw    : std_logic := '0';
    signal s_vs_go  : std_logic := '0';

    -- generated avid (parity-locked start, fixed-width end)
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal s_bcol : unsigned(11 downto 0) := (others => '0');

    -- sync delay
    signal d_hs, d_vs, d_fl, d_gv : t_bsr := (others => '0');
    signal s_io : t_video_stream_yuv444_30b;

    -- field-latched controls
    signal k_edge   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal k_drip8  : unsigned(7 downto 0) := to_unsigned(125, 8);
    signal k_lit    : unsigned(9 downto 0) := to_unsigned(500, 10);
    signal k_flow   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal k_stripe : unsigned(1 downto 0) := "01";
    signal k_litd   : unsigned(1 downto 0) := "10";
    signal k_bright : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal sw_cool, sw_bgblk, sw_up : std_logic := '0';
    signal sw_seams, sw_lit         : std_logic := '1';

    -- per-field geometry (free-running from the latched raster)
    signal f_w     : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal f_h     : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal f_hd    : std_logic := '1';
    signal f_bsh   : unsigned(1 downto 0) := "10";
    signal f_ml, f_mle, f_mr, f_mre : unsigned(11 downto 0) := (others => '0');
    signal f_mw, f_z1, f_z2, f_z3   : unsigned(11 downto 0) := (others => '0');
    signal f_top   : unsigned(11 downto 0) := to_unsigned(67, 12);
    signal f_bot   : unsigned(11 downto 0) := to_unsigned(945, 12);
    signal f_trav  : unsigned(9 downto 0)  := to_unsigned(270, 10);
    signal f_q     : unsigned(5 downto 0)  := to_unsigned(16, 6);
    signal f_jag   : unsigned(9 downto 0)  := (others => '0');
    signal f_jagh  : unsigned(9 downto 0)  := (others => '0');

    -- motion: time accumulator, per-speed crawl offsets, re-strike epoch
    signal s_t     : unsigned(16 downto 0) := (others => '0');
    signal f_inc   : unsigned(8 downto 0)  := to_unsigned(32, 9);
    signal f_o1, f_o2, f_o3, f_o4     : unsigned(11 downto 0) := (others => '0');
    signal f_os1, f_os2, f_os3, f_os4 : unsigned(11 downto 0) := (others => '0');
    signal f_ep16  : unsigned(15 downto 0) := x"7C2D";
    signal f_eprow : unsigned(4 downto 0)  := (others => '0');

    -- pixel pipeline
    signal p1_hc, p1_st, p1_rh, p1_la : unsigned(15 downto 0) := (others => '0');
    signal p1_y      : unsigned(11 downto 0) := (others => '0');
    signal p1_yrot   : signed(13 downto 0) := (others => '0');
    signal p1_inh, p1_nl, p1_nr : std_logic := '0';
    signal p1_zone   : unsigned(1 downto 0) := (others => '0');
    signal p1_xtri   : signed(7 downto 0) := (others => '0');
    signal p1_slope  : unsigned(11 downto 0) := (others => '0');

    signal p2_hc, p2_st, p2_rh, p2_lh : unsigned(15 downto 0) := (others => '0');
    signal p2_y      : unsigned(11 downto 0) := (others => '0');
    signal p2_band   : unsigned(1 downto 0) := (others => '0');
    signal p2_bfrac  : unsigned(5 downto 0) := (others => '0');
    signal p2_inh, p2_nl, p2_nr : std_logic := '0';
    signal p2_zone   : unsigned(1 downto 0) := (others => '0');
    signal p2_xtri   : signed(8 downto 0) := (others => '0');
    signal p2_tb     : signed(13 downto 0) := (others => '0');

    signal p3_hc, p3_st, p3_rh, p3_lh, p3_bseed : unsigned(15 downto 0) := (others => '0');
    signal p3_y      : unsigned(11 downto 0) := (others => '0');
    signal p3_band   : unsigned(1 downto 0) := (others => '0');
    signal p3_bfrac  : unsigned(5 downto 0) := (others => '0');
    signal p3_inh, p3_nl, p3_nr : std_logic := '0';
    signal p3_zone   : unsigned(1 downto 0) := (others => '0');
    signal p3_tb     : signed(13 downto 0) := (others => '0');

    signal p4_hc, p4_lh, p4_bseed : unsigned(15 downto 0) := (others => '0');
    signal p4_st     : unsigned(7 downto 0) := (others => '0');
    signal p4_inv    : std_logic := '0';
    signal p4_y      : unsigned(11 downto 0) := (others => '0');
    signal p4_band   : unsigned(1 downto 0) := (others => '0');
    signal p4_bfrac  : unsigned(5 downto 0) := (others => '0');
    signal p4_inh, p4_nl, p4_nr : std_logic := '0';
    signal p4_zone   : unsigned(1 downto 0) := (others => '0');
    signal p4_tb     : signed(13 downto 0) := (others => '0');

    signal p5_spk    : unsigned(17 downto 0) := (others => '0');
    signal p5_tall, p5_dsel, p5_cut, p5_border, p5_inh, p5_inv : std_logic := '0';
    signal p5_po     : unsigned(11 downto 0) := (others => '0');
    signal p5_dp     : unsigned(7 downto 0) := (others => '0');
    signal p5_dl     : unsigned(2 downto 0) := (others => '0');
    signal p5_k      : unsigned(1 downto 0) := (others => '0');
    signal p5_ft     : unsigned(6 downto 0) := (others => '0');
    signal p5_fb     : unsigned(5 downto 0) := (others => '0');
    signal p5_bs     : unsigned(5 downto 0) := (others => '0');
    signal p5_tb     : signed(13 downto 0) := (others => '0');
    signal p5_zone   : unsigned(1 downto 0) := (others => '0');
    signal p5_hcx, p5_lh : unsigned(15 downto 0) := (others => '0');
    signal p5_y      : unsigned(11 downto 0) := (others => '0');

    signal p6_ye     : unsigned(11 downto 0) := (others => '0');
    signal p6_spike  : unsigned(9 downto 0) := (others => '0');
    signal p6_dh     : unsigned(17 downto 0) := (others => '0');
    signal p6_dlen   : unsigned(9 downto 0) := (others => '0');
    signal p6_tb, p6_bot : signed(13 downto 0) := (others => '0');
    signal p6_fb     : unsigned(5 downto 0) := (others => '0');
    signal p6_k      : unsigned(1 downto 0) := (others => '0');
    signal p6_dsel, p6_cut, p6_border, p6_inh, p6_inv : std_logic := '0';
    signal p6_zone   : unsigned(1 downto 0) := (others => '0');
    signal p6_hcx, p6_lh : unsigned(15 downto 0) := (others => '0');
    signal p6_y      : unsigned(11 downto 0) := (others => '0');

    signal p7_span   : unsigned(6 downto 0) := (others => '0');
    signal p7_frac   : unsigned(5 downto 0) := (others => '0');
    signal p7_top, p7_head, p7_bot : signed(13 downto 0) := (others => '0');
    signal p7_spike  : unsigned(9 downto 0) := (others => '0');
    signal p7_dlen   : unsigned(9 downto 0) := (others => '0');
    signal p7_dsel, p7_cut, p7_border, p7_inh, p7_inv : std_logic := '0';
    signal p7_zone   : unsigned(1 downto 0) := (others => '0');
    signal p7_hcx, p7_lh : unsigned(15 downto 0) := (others => '0');
    signal p7_y      : unsigned(11 downto 0) := (others => '0');

    signal p8_m      : unsigned(15 downto 0) := (others => '0');
    signal p8_frac   : unsigned(5 downto 0) := (others => '0');
    signal p8_top, p8_head, p8_tail, p8_bot : signed(13 downto 0) := (others => '0');
    signal p8_dsel, p8_cut, p8_border, p8_inh, p8_inv : std_logic := '0';
    signal p8_zone   : unsigned(1 downto 0) := (others => '0');
    signal p8_hcx, p8_lh : unsigned(15 downto 0) := (others => '0');
    signal p8_y      : unsigned(11 downto 0) := (others => '0');

    signal p9_hs, p9_lh : unsigned(15 downto 0) := (others => '0');
    signal p9_yft    : unsigned(11 downto 0) := (others => '0');
    signal p9_frac   : unsigned(5 downto 0) := (others => '0');
    signal p9_in, p9_border, p9_inv : std_logic := '0';
    signal p9_zone   : unsigned(1 downto 0) := (others => '0');

    signal p10_hs, p10_lh : unsigned(15 downto 0) := (others => '0');
    signal p10_yfs   : unsigned(11 downto 0) := (others => '0');
    signal p10_frac  : unsigned(5 downto 0) := (others => '0');
    signal p10_in, p10_border, p10_inv : std_logic := '0';
    signal p10_zone  : unsigned(1 downto 0) := (others => '0');

    signal p11_hs    : unsigned(15 downto 0) := (others => '0');
    signal p11_frac  : unsigned(5 downto 0) := (others => '0');
    signal p11_in, p11_border, p11_inv, p11_lit, p11_spk : std_logic := '0';
    signal p11_zone  : unsigned(1 downto 0) := (others => '0');

    signal p12_bin   : unsigned(2 downto 0) := (others => '0');
    signal p12_frac  : unsigned(5 downto 0) := (others => '0');
    signal p12_in, p12_border, p12_inv, p12_lit, p12_spk : std_logic := '0';
    signal p12_zone  : unsigned(1 downto 0) := (others => '0');

    signal p13_py    : unsigned(9 downto 0) := (others => '0');   -- Y - 64
    signal p13_pu, p13_pv : signed(9 downto 0) := (others => '0'); -- U/V - 512
    signal p13_g, p13_gc  : unsigned(5 downto 0) := (others => '0');
    signal p13_in, p13_border, p13_inv, p13_lit, p13_spk : std_logic := '0';

    signal p14_dy    : unsigned(15 downto 0) := (others => '0');
    signal p14_du, p14_dv : signed(16 downto 0) := (others => '0');
    signal p14_in, p14_border, p14_inv, p14_lit, p14_spk : std_logic := '0';

    signal p15_y64, p15_u, p15_v : unsigned(9 downto 0) := (others => '0');
    signal p15_in, p15_border, p15_inv, p15_lit, p15_spk : std_logic := '0';

    signal p16_y64, p16_u, p16_v : unsigned(9 downto 0) := (others => '0');

    signal p17_yb    : unsigned(20 downto 0) := (others => '0');
    signal p17_u, p17_v : unsigned(9 downto 0) := (others => '0');

begin

    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    -- ====================================================================
    -- Raster measurement.  Width = active pixels per line; rows count
    -- ACTIVE lines from the vsync edge (all-ones reset, +1 at each avid
    -- start) so row 0 is the first visible line, and the height is taken
    -- from that count -- never from blanking lines.
    -- ====================================================================
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_hcnt > 32 then s_meas_h <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            elsif s_timing.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            end if;

            if s_timing.vsync_start = '1' then
                if s_vcnt(11) = '0' and s_vcnt > 32 then
                    s_meas_v <= s_vcnt + 1;
                end if;
                s_vcnt  <= (others => '1');
                s_ilace <= s_timing.is_interlaced;
                s_botf  <= not s_timing.field_n;      -- bottom field = field_n '0'
            elsif s_timing.avid_start = '1' then
                s_vcnt <= s_vcnt + 1;
            end if;

            if s_ilace = '1' then
                s_row <= s_vcnt(10 downto 0) & s_botf;
            else
                s_row <= s_vcnt;
            end if;
        end if;
    end process p_measure;

    -- ====================================================================
    -- Clean avid (cubist/aurora recipe).  g_ph is the hsync->avid-rise
    -- distance parity of this line; g_par is the voted usual parity; a line
    -- whose avid lands off-parity starts one clock later, so the generated
    -- start always sits at the same hsync parity.  It ends after the width
    -- latched per field, or at hsync at the latest.
    -- ====================================================================
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;    -- hsync_n falling
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;       -- avid rising

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

            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;

            -- beam column: counts the generated avid from hsync
            if g_hsr = '1' then
                s_bcol <= (others => '0');
            elsif g_avid = '1' then
                s_bcol <= s_bcol + 1;
            end if;
        end if;
    end process p_gavid;

    -- ====================================================================
    -- Per-field control latch and motion.  s_vs_go fires once per field:
    -- the first vsync edge after active video (analog vsync serrates).
    -- Everything below the latch is free-running from latched values, so
    -- the pixel path only ever sees registers.
    -- ====================================================================
    p_frame : process(clk)
        variable v_off : unsigned(11 downto 0);
        variable v_ep  : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            s_vs_go <= '0';
            if s_timing.avid = '1' then
                s_saw <= '1';
            elsif s_timing.vsync_start = '1' and s_saw = '1' then
                s_saw   <= '0';
                s_vs_go <= '1';
            end if;

            if s_vs_go = '1' then
                g_wm1 <= s_meas_h - 1;
                f_w   <= s_meas_h;
                if s_ilace = '1' then
                    f_h <= s_meas_v(10 downto 0) & '0';
                else
                    f_h <= s_meas_v;
                end if;
                k_edge   <= unsigned(registers_in(0));
                k_drip8  <= unsigned(registers_in(1)(9 downto 2));
                k_lit    <= unsigned(registers_in(2));
                k_flow   <= unsigned(registers_in(3));
                k_stripe <= unsigned(registers_in(4)(9 downto 8));
                k_litd   <= unsigned(registers_in(5)(9 downto 8));
                sw_cool  <= registers_in(6)(0);
                sw_bgblk <= registers_in(6)(1);
                sw_seams <= registers_in(6)(2);
                sw_lit   <= registers_in(6)(3);
                sw_up    <= registers_in(6)(4);
                k_bright <= unsigned(registers_in(7));
                s_t      <= s_t + resize(f_inc, 17);
            end if;

            -- raster class: HD doubles feature scales; pass height 64 /
            -- 128 / 256 rows for SD / 720 / 1080 keeps ~4 passes on screen
            if f_w >= 1000 then f_hd <= '1'; else f_hd <= '0'; end if;
            if f_h >= 900 then
                f_bsh <= "10";
            elsif f_w >= 1000 then
                f_bsh <= "01";
            else
                f_bsh <= "00";
            end if;

            -- mass: centred 75 % of the width, edge fringe W/16 deep
            f_ml  <= shift_right(f_w, 3);
            f_mle <= shift_right(f_w, 3) + shift_right(f_w, 4);
            f_mr  <= f_w - shift_right(f_w, 3);
            f_mre <= f_mr - shift_right(f_w, 4);
            f_mw  <= f_w - shift_right(f_w, 2);
            f_z1  <= f_ml + shift_right(f_mw, 2);
            f_z2  <= f_ml + shift_right(f_mw, 1);
            f_z3  <= f_mr - shift_right(f_mw, 2);

            -- vertical: top H/16, body bottom 7H/8, drips fall the last H/8
            -- (head travels H/4 so a drip clears the floor before it wraps)
            f_top  <= shift_right(f_h, 4);
            f_bot  <= f_h - shift_right(f_h, 3);
            f_trav <= f_h(11 downto 2);
            f_q    <= f_h(11 downto 6);

            -- K1 Edge: tower amplitude 0 .. 3H/8
            case k_edge(9 downto 7) is
                when "000"  => f_jag <= (others => '0');
                when "001"  => f_jag <= resize(shift_right(f_h, 5), 10);
                when "010"  => f_jag <= resize(shift_right(f_h, 4), 10);
                when "011"  => f_jag <= resize(shift_right(f_h, 4) + shift_right(f_h, 5), 10);
                when "100"  => f_jag <= resize(shift_right(f_h, 3), 10);
                when "101"  => f_jag <= resize(shift_right(f_h, 3) + shift_right(f_h, 4), 10);
                when "110"  => f_jag <= resize(shift_right(f_h, 2), 10);
                when others => f_jag <= resize(shift_right(f_h, 2) + shift_right(f_h, 3), 10);
            end case;
            f_jagh <= shift_right(f_jag, 1);

            -- K4 Flow, two-slope: 1/32 .. 1 px/field over the first half,
            -- up to ~9 px/field at the top (x1..x4 per column)
            if k_flow(9) = '0' then
                f_inc <= to_unsigned(1, 9) + resize(k_flow(8 downto 4), 9);
            else
                f_inc <= to_unsigned(32, 9) + resize(k_flow(8 downto 1), 9);
            end if;

            -- crawl offsets in px for the four column speeds; 12-bit wrap is
            -- seamless because every run length divides 4096
            v_off := s_t(16 downto 5);
            f_o1 <= v_off;
            f_o2 <= shift_left(v_off, 1);
            f_o3 <= v_off + shift_left(v_off, 1);
            f_o4 <= shift_left(v_off, 2);
            if sw_up = '1' then
                f_os1 <= f_o1; f_os2 <= f_o2; f_os3 <= f_o3; f_os4 <= f_o4;
            else
                f_os1 <= 0 - f_o1; f_os2 <= 0 - f_o2;
                f_os3 <= 0 - f_o3; f_os4 <= 0 - f_o4;
            end if;

            -- lightning / invert-row epoch: re-strikes as the picture flows
            v_ep    := s_t(16 downto 9);
            f_ep16  <= (v_ep(3 downto 0) & v_ep & v_ep(7 downto 4)) xor x"7C2D";
            f_eprow <= v_ep(4 downto 0);
        end if;
    end process p_frame;

    -- ====================================================================
    -- Pixel pipeline.  Stage 0 is (s_bcol, s_row) on the generated avid;
    -- p1..p17 compute, s_io is clock C_LAT.
    -- ====================================================================
    p_pixel : process(clk)
        variable x, y     : unsigned(11 downto 0);
        variable v_sidx   : unsigned(11 downto 0);
        variable v_xt     : unsigned(11 downto 0);
        variable v_tu     : unsigned(6 downto 0);
        variable v_ry     : unsigned(11 downto 0);
        variable v_yrs    : signed(13 downto 0);
        variable v_lh     : unsigned(15 downto 0);
        variable v_hc     : unsigned(15 downto 0);
        variable v_os     : unsigned(11 downto 0);
        variable v_g      : unsigned(7 downto 0);
        variable v_edge   : boolean;
        variable v_jit    : unsigned(5 downto 0);
        variable v_zd     : unsigned(1 downto 0);
        variable v_bs     : unsigned(6 downto 0);
        variable v_ys     : signed(13 downto 0);
        variable v_in     : boolean;
        variable v_thr    : unsigned(7 downto 0);
        variable v_pi     : natural range 0 to 7;
        variable v_fr     : unsigned(5 downto 0);
        variable v_u, v_v : signed(11 downto 0);
        variable v_yo     : unsigned(10 downto 0);
        variable v_ks     : unsigned(2 downto 0);
        variable v_idx    : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then

            ----------------------------------------------------------------
            -- p1: stripe index, rotated y, horizontal mass test, hash seeds
            ----------------------------------------------------------------
            x := s_bcol;
            y := s_row;
            case k_stripe is
                when "00"   => v_sidx := x;
                when "01"   => v_sidx := '0' & x(11 downto 1);
                when "10"   => v_sidx := "00" & x(11 downto 2);
                when others => v_sidx := "000" & x(11 downto 3);
            end case;
            p1_hc <= resize(v_sidx, 16) + to_unsigned(16#3C6B#, 16);
            p1_y  <= y;
            p1_yrot <= signed("00" & y) - signed("000" & x(11 downto 1));

            if x >= f_ml and x <= f_mr then p1_inh <= '1'; else p1_inh <= '0'; end if;
            if x < f_mle then p1_nl <= '1'; else p1_nl <= '0'; end if;
            if x > f_mre then p1_nr <= '1'; else p1_nr <= '0'; end if;
            if x < f_z1 then
                p1_zone <= "00";
            elsif x < f_z2 then
                p1_zone <= "01";
            elsif x < f_z3 then
                p1_zone <= "10";
            else
                p1_zone <= "11";
            end if;

            -- top-edge features run on x/2 at HD (same shapes as SD)
            if f_hd = '1' then v_xt := '0' & x(11 downto 1); else v_xt := x; end if;
            if v_xt(7) = '0' then v_tu := v_xt(6 downto 0); else v_tu := not v_xt(6 downto 0); end if;
            p1_xtri  <= shift_right(signed('0' & v_tu) - to_signed(64, 8), 1);
            p1_st    <= resize(v_xt(11 downto 5), 16) + to_unsigned(16#0A5E#, 16);
            p1_slope <= "00" & x(11 downto 2);

            -- invert rows: 1/16 of rows (row pairs at HD), re-rolled per epoch
            if f_hd = '1' then v_ry := '0' & y(11 downto 1); else v_ry := y; end if;
            p1_rh <= (resize(v_ry, 16) xor (f_eprow & "00000000000"))
                     + to_unsigned(16#07C9#, 16);

            -- lightning hash operands: x/2 + 3y (kept weakly mixed, so the
            -- dashes keep their diagonal grain)
            p1_la <= resize(x(11 downto 1), 16) + shift_left(resize(y, 16), 1);

            ----------------------------------------------------------------
            -- p2: hash round 1, pass band + seam fraction, top baseline
            ----------------------------------------------------------------
            p2_hc <= f_r1(p1_hc);
            p2_st <= f_r1(p1_st);
            p2_rh <= f_r1(p1_rh);
            p2_lh <= (p1_la + resize(p1_y, 16))
                     xor shift_left(resize(p1_y, 16), 4) xor f_ep16;

            case f_bsh is
                when "00"   => v_yrs := p1_yrot;
                when "01"   => v_yrs := shift_right(p1_yrot, 1);
                when others => v_yrs := shift_right(p1_yrot, 2);
            end case;
            p2_band  <= unsigned(v_yrs(7 downto 6));
            p2_bfrac <= unsigned(v_yrs(5 downto 0));

            if f_hd = '1' then
                p2_xtri <= shift_left(resize(p1_xtri, 9), 1);
            else
                p2_xtri <= resize(p1_xtri, 9);
            end if;
            p2_tb   <= signed("00" & f_top) + signed("00" & p1_slope);
            p2_y    <= p1_y;
            p2_inh  <= p1_inh;
            p2_nl   <= p1_nl;
            p2_nr   <= p1_nr;
            p2_zone <= p1_zone;

            ----------------------------------------------------------------
            -- p3: hash round 2, lightning shuffle, pass seed
            ----------------------------------------------------------------
            p3_hc <= f_r2(p2_hc);
            p3_st <= f_r2(p2_st);
            p3_rh <= f_r2(p2_rh);
            v_lh := p2_lh;
            v_lh := v_lh xor shift_left(v_lh, 5);
            v_lh := v_lh xor shift_right(v_lh, 11);
            v_lh := v_lh xor shift_left(v_lh, 7);
            p3_lh <= v_lh;

            p3_tb <= p2_tb + resize(p2_xtri, 14);
            if sw_seams = '1' then
                p3_band <= p2_band;
                case p2_band is
                    when "00"   => p3_bseed <= C_SEED0;
                    when "01"   => p3_bseed <= C_SEED1;
                    when "10"   => p3_bseed <= C_SEED2;
                    when others => p3_bseed <= C_SEED3;
                end case;
            else
                p3_band  <= "00";
                p3_bseed <= C_SEED0;
            end if;
            p3_bfrac <= p2_bfrac;
            p3_y     <= p2_y;
            p3_inh   <= p2_inh;
            p3_nl    <= p2_nl;
            p3_nr    <= p2_nr;
            p3_zone  <= p2_zone;

            ----------------------------------------------------------------
            -- p4: hash round 3 -> column hash, step height, invert flag
            ----------------------------------------------------------------
            p4_hc <= f_r3(p3_hc);
            v_hc  := f_r3(p3_st);
            p4_st <= v_hc(7 downto 0);
            v_hc  := f_r3(p3_rh);
            if v_hc(15 downto 12) = "0000" then p4_inv <= '1'; else p4_inv <= '0'; end if;
            p4_lh    <= p3_lh;
            p4_tb    <= p3_tb;
            p4_band  <= p3_band;
            p4_bseed <= p3_bseed;
            p4_bfrac <= p3_bfrac;
            p4_y     <= p3_y;
            p4_inh   <= p3_inh;
            p4_nl    <= p3_nl;
            p4_nr    <= p3_nr;
            p4_zone  <= p3_zone;

            ----------------------------------------------------------------
            -- p5: per-column decisions from the column hash
            --   8:0 run phase   10:9 run length   12:11 crawl speed
            --   15:8 drip pick  7:0 tower + drip phase   13:11 drip length
            --   1:0 tall tower  4:2 edge drop  13:7 / 15:10 edge fringe
            --   5:0 bottom rag  14:12 seam jitter  6:5 zone dither
            ----------------------------------------------------------------
            v_hc := p4_hc;
            p5_spk <= v_hc(7 downto 0) * f_jag;
            if v_hc(1 downto 0) = "00" then p5_tall <= '1'; else p5_tall <= '0'; end if;

            case v_hc(12 downto 11) is
                when "00"   => v_os := f_os1; v_g := f_o1(8 downto 1);
                when "01"   => v_os := f_os2; v_g := f_o2(8 downto 1);
                when "10"   => v_os := f_os3; v_g := f_o3(8 downto 1);
                when others => v_os := f_os4; v_g := f_o4(8 downto 1);
            end case;
            p5_po <= resize(v_hc(8 downto 0), 12) + v_os;
            p5_dp <= v_hc(7 downto 0) + v_g;
            if v_hc(15 downto 8) < k_drip8 then p5_dsel <= '1'; else p5_dsel <= '0'; end if;
            p5_dl <= v_hc(13 downto 11);
            p5_k  <= v_hc(10 downto 9);

            v_edge := (p4_nl = '1') or (p4_nr = '1');
            if v_edge and v_hc(4 downto 2) = "000" then p5_cut <= '1'; else p5_cut <= '0'; end if;
            if v_edge then
                p5_ft <= v_hc(13 downto 7);
                p5_fb <= v_hc(15 downto 10);
            else
                p5_ft <= (others => '0');
                p5_fb <= (others => '0');
            end if;
            p5_bs <= v_hc(5 downto 0);

            if f_hd = '1' then
                p5_tb <= p4_tb + signed("00000" & p4_st & '0');
            else
                p5_tb <= p4_tb + signed("000000" & p4_st);
            end if;

            v_jit := "000" & v_hc(14 downto 12);
            if sw_seams = '1' and (p4_bfrac < to_unsigned(4, 6) + v_jit
                                   or p4_bfrac > to_unsigned(58, 6) - v_jit) then
                p5_border <= '1';
            else
                p5_border <= '0';
            end if;

            case v_hc(6 downto 5) is
                when "00" =>
                    if p4_zone = "00" then v_zd := p4_zone; else v_zd := p4_zone - 1; end if;
                when "11" =>
                    if p4_zone = "11" then v_zd := p4_zone; else v_zd := p4_zone + 1; end if;
                when others =>
                    v_zd := p4_zone;
            end case;
            p5_zone <= v_zd + p4_band;
            p5_hcx  <= v_hc xor p4_bseed;
            p5_lh   <= p4_lh;
            p5_y    <= p4_y;
            p5_inh  <= p4_inh;
            p5_inv  <= p4_inv;

            ----------------------------------------------------------------
            -- p6: crawled y, tower height, drip products, column bottom
            ----------------------------------------------------------------
            p6_ye <= p5_y + p5_po;
            if p5_tall = '1' then
                p6_spike <= p5_spk(17 downto 8);
            else
                p6_spike <= "00" & p5_spk(17 downto 10);
            end if;
            p6_dh   <= p5_dp * f_trav;
            p6_dlen <= ('0' & p5_dl + 1) * f_q;
            if f_hd = '1' then
                p6_tb <= p5_tb + signed("000000" & p5_ft & '0');
                v_bs  := p5_bs & '0';
            else
                p6_tb <= p5_tb + signed("0000000" & p5_ft);
                v_bs  := '0' & p5_bs;
            end if;
            if p5_dsel = '1' then
                p6_bot <= signed("00" & f_bot);
            else
                p6_bot <= signed("00" & f_bot) - signed("0000000" & v_bs);
            end if;
            p6_fb     <= p5_fb;
            p6_k      <= p5_k;
            p6_dsel   <= p5_dsel;
            p6_cut    <= p5_cut;
            p6_border <= p5_border;
            p6_inh    <= p5_inh;
            p6_inv    <= p5_inv;
            p6_zone   <= p5_zone;
            p6_hcx    <= p5_hcx;
            p6_lh     <= p5_lh;
            p6_y      <= p5_y;

            ----------------------------------------------------------------
            -- p7: run index + position in run; top / drip head / bottom
            ----------------------------------------------------------------
            v_ks := f_hd & p6_k;
            case v_ks is
                when "000" =>           -- 32 rows
                    p7_span <= p6_ye(11 downto 5);
                    p7_frac <= p6_ye(4 downto 0) & '0';
                when "001" | "100" =>   -- 64
                    p7_span <= '0' & p6_ye(11 downto 6);
                    p7_frac <= p6_ye(5 downto 0);
                when "010" | "101" =>   -- 128
                    p7_span <= "00" & p6_ye(11 downto 7);
                    p7_frac <= p6_ye(6 downto 1);
                when "011" | "110" =>   -- 256
                    p7_span <= "000" & p6_ye(11 downto 8);
                    p7_frac <= p6_ye(7 downto 2);
                when others =>          -- 512
                    p7_span <= "0000" & p6_ye(11 downto 9);
                    p7_frac <= p6_ye(8 downto 3);
            end case;
            p7_top  <= p6_tb + signed("0000" & f_jagh);
            p7_head <= signed("00" & f_bot) + signed("0000" & p6_dh(17 downto 8));
            if f_hd = '1' then
                p7_bot <= p6_bot - signed("000000" & p6_fb & "00");
            else
                p7_bot <= p6_bot - signed("0000000" & p6_fb & '0');
            end if;
            p7_spike  <= p6_spike;
            p7_dlen   <= p6_dlen;
            p7_dsel   <= p6_dsel;
            p7_cut    <= p6_cut;
            p7_border <= p6_border;
            p7_inh    <= p6_inh;
            p7_inv    <= p6_inv;
            p7_zone   <= p6_zone;
            p7_hcx    <= p6_hcx;
            p7_lh     <= p6_lh;
            p7_y      <= p6_y;

            ----------------------------------------------------------------
            -- p8: run hash multiplier (x133), final top, drip tail
            ----------------------------------------------------------------
            p8_m <= shift_left(resize(p7_span, 16), 7) + shift_left(resize(p7_span, 16), 2)
                    + resize(p7_span, 16);
            p8_top  <= p7_top - signed("0000" & p7_spike);
            p8_tail <= p7_head - signed("0000" & p7_dlen);
            p8_head   <= p7_head;
            p8_bot    <= p7_bot;
            p8_frac   <= p7_frac;
            p8_dsel   <= p7_dsel;
            p8_cut    <= p7_cut;
            p8_border <= p7_border;
            p8_inh    <= p7_inh;
            p8_inv    <= p7_inv;
            p8_zone   <= p7_zone;
            p8_hcx    <= p7_hcx;
            p8_lh     <= p7_lh;
            p8_y      <= p7_y;

            ----------------------------------------------------------------
            -- p9: run hash seed, in-mass test, depth below the top edge
            ----------------------------------------------------------------
            p9_hs <= p8_hcx + p8_m;
            v_ys := signed("00" & p8_y);
            v_in := p8_inh = '1' and p8_cut = '0' and v_ys >= p8_top
                    and (v_ys <= p8_bot
                         or (p8_dsel = '1' and v_ys >= p8_tail and v_ys <= p8_head));
            if v_in then p9_in <= '1'; else p9_in <= '0'; end if;
            if v_ys >= p8_top then
                p9_yft <= unsigned(resize(v_ys - p8_top, 12));
            else
                p9_yft <= (others => '0');
            end if;
            p9_frac   <= p8_frac;
            p9_border <= p8_border;
            p9_inv    <= p8_inv;
            p9_zone   <= p8_zone;
            p9_lh     <= p8_lh;

            ----------------------------------------------------------------
            -- p10: run hash round 2, lightning depth scale
            ----------------------------------------------------------------
            p10_hs <= f_r2(p9_hs);
            v_ks := '0' & k_litd;
            if f_hd = '1' then v_ks := v_ks + 1; end if;
            case v_ks is
                when "000"  => p10_yfs <= p9_yft;
                when "001"  => p10_yfs <= shift_right(p9_yft, 1);
                when "010"  => p10_yfs <= shift_right(p9_yft, 2);
                when "011"  => p10_yfs <= shift_right(p9_yft, 3);
                when others => p10_yfs <= shift_right(p9_yft, 4);
            end case;
            p10_frac   <= p9_frac;
            p10_in     <= p9_in;
            p10_border <= p9_border;
            p10_inv    <= p9_inv;
            p10_zone   <= p9_zone;
            p10_lh     <= p9_lh;

            ----------------------------------------------------------------
            -- p11: run hash round 3, lightning + speckle tests
            ----------------------------------------------------------------
            p11_hs <= f_r3(p10_hs);
            if p10_yfs(11 downto 7) /= "00000" then
                v_thr := (others => '0');
            else
                case p10_yfs(6 downto 5) is
                    when "00"   => v_thr := resize(k_lit(9 downto 4), 8);
                    when "01"   => v_thr := resize(k_lit(9 downto 5), 8);
                    when "10"   => v_thr := resize(k_lit(9 downto 6), 8);
                    when others => v_thr := resize(k_lit(9 downto 7), 8);
                end case;
            end if;
            if sw_lit = '1' and (p10_lh(7 downto 0) < v_thr or p10_lh(15 downto 8) < v_thr) then
                p11_lit <= '1';
            else
                p11_lit <= '0';
            end if;
            if p10_lh(15 downto 9) = "0000000" then p11_spk <= '1'; else p11_spk <= '0'; end if;
            p11_frac   <= p10_frac;
            p11_in     <= p10_in;
            p11_border <= p10_border;
            p11_inv    <= p10_inv;
            p11_zone   <= p10_zone;

            ----------------------------------------------------------------
            -- p12: colour bin (1/8 near-black shadow runs)
            ----------------------------------------------------------------
            if p11_hs(10 downto 8) = "000" then
                p12_bin <= to_unsigned(6, 3);
            else
                p12_bin <= p11_hs(14 downto 12);
            end if;
            p12_frac   <= p11_frac;
            p12_in     <= p11_in;
            p12_border <= p11_border;
            p12_inv    <= p11_inv;
            p12_lit    <= p11_lit;
            p12_spk    <= p11_spk;
            p12_zone   <= p11_zone;

            ----------------------------------------------------------------
            -- p13: palette (stored as Y-64, U-512, V-512) + run shading gains
            ----------------------------------------------------------------
            v_idx := sw_cool & p12_zone & p12_bin;
            v_pi  := C_PALIDX(to_integer(v_idx));
            case v_pi is
                when 0 =>      -- magenta
                    p13_py <= to_unsigned(516, 10); p13_pu <= to_signed(128, 10);  p13_pv <= to_signed(368, 10);
                when 1 =>      -- crimson
                    p13_py <= to_unsigned(286, 10); p13_pu <= to_signed(-132, 10); p13_pv <= to_signed(368, 10);
                when 2 =>      -- cobalt
                    p13_py <= to_unsigned(236, 10); p13_pu <= to_signed(308, 10);  p13_pv <= to_signed(-132, 10);
                when 3 =>      -- violet
                    p13_py <= to_unsigned(216, 10); p13_pu <= to_signed(268, 10);  p13_pv <= to_signed(148, 10);
                when 4 =>      -- emerald
                    p13_py <= to_unsigned(376, 10); p13_pu <= to_signed(-92, 10);  p13_pv <= to_signed(-222, 10);
                when 5 =>      -- amber
                    p13_py <= to_unsigned(496, 10); p13_pu <= to_signed(-252, 10); p13_pv <= to_signed(228, 10);
                when 6 =>      -- deep maroon shadow
                    p13_py <= to_unsigned(46, 10);  p13_pu <= to_signed(8, 10);    p13_pv <= to_signed(48, 10);
                when others => -- pale highlight (luma capped below 768)
                    p13_py <= to_unsigned(696, 10); p13_pu <= to_signed(68, 10);   p13_pv <= to_signed(-72, 10);
            end case;
            -- luma 15/64 .. 63/64 and chroma 32/64 .. 63/64 along the run;
            -- Sort Up puts the bright end at the top
            if sw_up = '1' then v_fr := not p12_frac; else v_fr := p12_frac; end if;
            p13_g  <= to_unsigned(15, 6) + (v_fr - shift_right(v_fr, 2));
            p13_gc <= to_unsigned(32, 6) + shift_right(v_fr, 1);
            p13_in     <= p12_in;
            p13_border <= p12_border;
            p13_inv    <= p12_inv;
            p13_lit    <= p12_lit;
            p13_spk    <= p12_spk;

            ----------------------------------------------------------------
            -- p14: run shading products
            ----------------------------------------------------------------
            p14_dy <= p13_py * p13_g;
            p14_du <= p13_pu * signed('0' & p13_gc);
            p14_dv <= p13_pv * signed('0' & p13_gc);
            p14_in     <= p13_in;
            p14_border <= p13_border;
            p14_inv    <= p13_inv;
            p14_lit    <= p13_lit;
            p14_spk    <= p13_spk;

            ----------------------------------------------------------------
            -- p15: shaded colour
            ----------------------------------------------------------------
            p15_y64 <= p14_dy(15 downto 6);
            v_u := to_signed(512, 12) + resize(shift_right(p14_du, 6), 12);
            v_v := to_signed(512, 12) + resize(shift_right(p14_dv, 6), 12);
            p15_u <= unsigned(v_u(9 downto 0));
            p15_v <= unsigned(v_v(9 downto 0));
            p15_in     <= p14_in;
            p15_border <= p14_border;
            p15_inv    <= p14_inv;
            p15_lit    <= p14_lit;
            p15_spk    <= p14_spk;

            ----------------------------------------------------------------
            -- p16: overrides (seam > speckle > lightning > invert) and the
            -- background outside the mass
            ----------------------------------------------------------------
            if p15_in = '0' then
                if sw_bgblk = '1' then
                    p16_y64 <= (others => '0'); p16_u <= C_MID; p16_v <= C_MID;
                else
                    p16_y64 <= C_TEAL_Y64; p16_u <= C_TEAL_U; p16_v <= C_TEAL_V;
                end if;
            elsif p15_border = '1' then
                p16_y64 <= (others => '0'); p16_u <= C_MID; p16_v <= C_MID;
            elsif p15_spk = '1' or p15_lit = '1' then
                p16_y64 <= C_WHITE64; p16_u <= C_MID; p16_v <= C_MID;
            elsif p15_inv = '1' then
                p16_y64 <= to_unsigned(895, 10) - p15_y64;      -- 1023 - Y, less 64
                p16_u   <= not p15_u;
                p16_v   <= not p15_v;
            else
                p16_y64 <= p15_y64; p16_u <= p15_u; p16_v <= p15_v;
            end if;

            ----------------------------------------------------------------
            -- p17: brightness about black, gain 0.5 .. 1.5 (512 = unity)
            ----------------------------------------------------------------
            p17_yb <= p16_y64 * (to_unsigned(512, 11) + resize(k_bright, 11));
            p17_u  <= p16_u;
            p17_v  <= p16_v;

            ----------------------------------------------------------------
            -- output: clamp, blank outside the generated avid, swap U/V
            ----------------------------------------------------------------
            v_yo := p17_yb(20 downto 10) + 64;
            if v_yo > 940 then
                v_yo := to_unsigned(940, 11);
            end if;
            if d_gv(C_LAT - 1) = '1' then
                s_io.y <= std_logic_vector(v_yo(9 downto 0));
                s_io.u <= std_logic_vector(p17_v);
                s_io.v <= std_logic_vector(p17_u);
            else
                s_io.y <= std_logic_vector(to_unsigned(64, 10));
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            end if;
            s_io.hsync_n <= d_hs(C_LAT - 1);
            s_io.vsync_n <= d_vs(C_LAT - 1);
            s_io.field_n <= d_fl(C_LAT - 1);
            s_io.avid    <= d_gv(C_LAT - 1);
        end if;
    end process p_pixel;

    -- ====================================================================
    -- Sync delay (syncs and the generated avid only; no video is carried)
    -- ====================================================================
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            d_hs(1) <= data_in.hsync_n;
            d_vs(1) <= data_in.vsync_n;
            d_fl(1) <= data_in.field_n;
            d_gv(1) <= g_avid;
            for i in 2 to C_LAT - 1 loop
                d_hs(i) <= d_hs(i - 1);
                d_vs(i) <= d_vs(i - 1);
                d_fl(i) <= d_fl(i - 1);
                d_gv(i) <= d_gv(i - 1);
            end loop;
        end if;
    end process p_delay;

    data_out <= s_io;

end architecture solitude;
