-- ziffern.vhd  (multi-plane, 3-speed depth)
--
-- A Kraftwerk-inspired green "computer screen" wall of pixelated digits 1-9.
-- The digits change at random, staggered times; the whole field zooms toward the
-- viewer (numbers enlarge) and recedes, centred on the screen.  Pure synthesis:
-- the incoming video is ignored; output is drawn over a black/dark background.
--
-- Depth (the "different speeds" effect):
--  * NP=3 independent grid PLANES are composited front-to-back.  Each plane is a
--    full inverse-mapped digit grid (ziggurat-style accumulators, no per-pixel
--    multiply) at its OWN cell size, drawing full glyphs (so digits render
--    correctly at any size).  Each plane only DRAWS the cells it "owns" (a per-
--    cell hash partitions the grid into the 3 planes).
--  * Zoom (P12) animates every plane's cell size from a dense background to a
--    foreground, at DIFFERENT speeds (plane 0 fastest .. plane 2 slowest); all
--    start co-planar (background) and end co-planar (foreground), so the field is
--    flat at both ends and scattered in depth between -> numbers reach the front
--    at different times, then recede the same way.
--
-- Controls: K1 Color, K2 Change Rate, K3 Number Set, K4 Brightness, K5 Spacing,
--           K6 Glow, S7 Animate, S8 Background, S9 Scanlines, S10 Font,
--           S11 Invert, P12 Zoom (slider).
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

architecture ziffern of program_top is

    constant NP : integer := 3;          -- depth planes (front=0 .. back=NP-1)

    -- per-plane zoom-speed gain: cz = clamp(zoom + zoom>>gshift, 1024).
    -- shift 1 -> x1.5 (fast), 2 -> x1.25, 15 -> x1.0 (slowest); ordered so the
    -- front plane is always the largest (stable occlusion).
    type t_gsh is array (0 to NP - 1) of integer range 0 to 15;
    constant C_GSHIFT : t_gsh := (1, 2, 15);

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    --------------------------------------------------------------------------
    -- Hue ring (16 entries; index = hue_knob(9 downto 6), no multiply).
    -- ROYGBIV interpolated, stored U/V-swapped for HW (stored U=Cr, V=Cb).  Green=7.
    --------------------------------------------------------------------------
    constant C_NHUE : integer := 16;
    type t_hue is array (0 to C_NHUE - 1) of unsigned(9 downto 0);
    -- luma already brightened ~1.5x (clamped) so digits pop on the dark bg.
    constant C_HUE_Y : t_hue := (
        to_unsigned(492,10), to_unsigned(660,10), to_unsigned(828,10), to_unsigned(996,10),
        to_unsigned(1023,10),to_unsigned(1023,10),to_unsigned(1017,10),to_unsigned(831,10),
        to_unsigned(558,10), to_unsigned(285,10), to_unsigned(261,10), to_unsigned(280,10),
        to_unsigned(346,10), to_unsigned(450,10), to_unsigned(519,10), to_unsigned(505,10));
    constant C_HUE_U : t_hue := (
        to_unsigned(960,10), to_unsigned(878,10), to_unsigned(796,10), to_unsigned(713,10),
        to_unsigned(631,10), to_unsigned(500,10), to_unsigned(304,10), to_unsigned(155,10),
        to_unsigned(288,10), to_unsigned(421,10), to_unsigned(503,10), to_unsigned(576,10),
        to_unsigned(645,10), to_unsigned(710,10), to_unsigned(782,10), to_unsigned(871,10));
    constant C_HUE_V : t_hue := (
        to_unsigned(360,10), to_unsigned(295,10), to_unsigned(230,10), to_unsigned(166,10),
        to_unsigned(101,10), to_unsigned( 92,10), to_unsigned(159,10), to_unsigned(262,10),
        to_unsigned(588,10), to_unsigned(914,10), to_unsigned(861,10), to_unsigned(746,10),
        to_unsigned(735,10), to_unsigned(803,10), to_unsigned(790,10), to_unsigned(575,10));

    --------------------------------------------------------------------------
    -- Digit glyphs 1-9 (8x8, MSB = leftmost).  idx 0..8 = plain '1'..'9',
    -- idx 9..17 = sci-fi '1'..'9'.  From titler_font_pkg entries 28..36 / 92..100.
    --------------------------------------------------------------------------
    type t_digrom is array (0 to 17, 0 to 7) of std_logic_vector(7 downto 0);
    constant C_DIGITS : t_digrom := (
         0 => (x"10",x"30",x"10",x"10",x"10",x"10",x"38",x"00"),  -- 1
         1 => (x"38",x"44",x"04",x"08",x"10",x"20",x"7C",x"00"),  -- 2
         2 => (x"78",x"04",x"04",x"38",x"04",x"04",x"78",x"00"),  -- 3
         3 => (x"08",x"18",x"28",x"48",x"7C",x"08",x"08",x"00"),  -- 4
         4 => (x"7C",x"40",x"78",x"04",x"04",x"44",x"38",x"00"),  -- 5
         5 => (x"38",x"44",x"40",x"78",x"44",x"44",x"38",x"00"),  -- 6
         6 => (x"7C",x"04",x"08",x"10",x"20",x"20",x"20",x"00"),  -- 7
         7 => (x"38",x"44",x"44",x"38",x"44",x"44",x"38",x"00"),  -- 8
         8 => (x"38",x"44",x"44",x"3C",x"04",x"44",x"38",x"00"),  -- 9
         9 => (x"10",x"30",x"50",x"10",x"10",x"10",x"7C",x"00"),  -- 1 sci-fi
        10 => (x"38",x"44",x"04",x"08",x"10",x"20",x"7C",x"00"),  -- 2
        11 => (x"78",x"04",x"04",x"38",x"04",x"04",x"78",x"00"),  -- 3
        12 => (x"3C",x"24",x"24",x"3C",x"04",x"04",x"04",x"00"),  -- 4
        13 => (x"7C",x"40",x"78",x"04",x"04",x"44",x"38",x"00"),  -- 5
        14 => (x"38",x"40",x"40",x"78",x"44",x"44",x"38",x"00"),  -- 6
        15 => (x"7C",x"04",x"08",x"10",x"20",x"20",x"20",x"00"),  -- 7
        16 => (x"38",x"44",x"44",x"38",x"44",x"44",x"38",x"00"),  -- 8
        17 => (x"38",x"44",x"44",x"3C",x"04",x"04",x"38",x"00")); -- 9

    type t_dlut is array (0 to 15) of integer range 0 to 8;
    constant C_DIGLUT : t_dlut := (0,1,2,3,4,5,6,7,8,0,1,2,3,4,5,6);

    --------------------------------------------------------------------------
    -- Per-plane array types
    --------------------------------------------------------------------------
    type t_uarr  is array (0 to NP - 1) of unsigned(11 downto 0);
    type t_sarr  is array (0 to NP - 1) of signed(8 downto 0);
    type t_u3arr is array (0 to NP - 1) of unsigned(2 downto 0);
    type t_u16arr is array (0 to NP - 1) of unsigned(15 downto 0);
    type t_slv8arr is array (0 to NP - 1) of std_logic_vector(7 downto 0);
    type t_idxarr is array (0 to NP - 1) of integer range 0 to 17;
    type t_slarr is array (0 to NP - 1) of std_logic;
    type t_zparr is array (0 to NP - 1) of unsigned(21 downto 0);
    type t_czarr is array (0 to NP - 1) of unsigned(10 downto 0);

    --------------------------------------------------------------------------
    -- Helpers
    --------------------------------------------------------------------------
    function mix16(x : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := x;
        v := v xor shift_left(v, 7);
        v := v xor shift_right(v, 9);
        v := v xor shift_left(v, 8);
        v := v xor shift_right(v, 5);
        return v;
    end function;

    function spreadcr(col, row : signed(8 downto 0)) return unsigned is
        variable cu, ru : unsigned(15 downto 0);
    begin
        cu := resize(unsigned(col(7 downto 0)), 16);
        ru := resize(unsigned(row(7 downto 0)), 16);
        return (shift_left(cu, 8) + cu) + (shift_left(ru, 12) + ru);
    end function;

    function getbit(rb : std_logic_vector(7 downto 0); p : integer) return std_logic is
    begin
        if p >= 0 and p <= 7 then return rb(7 - p); else return '0'; end if;
    end function;

    function scale8(val : unsigned(9 downto 0); sel : unsigned(2 downto 0)) return unsigned is
        variable r : unsigned(9 downto 0);
    begin
        case sel is
            when "000" => r := shift_right(val, 3);
            when "001" => r := shift_right(val, 2);
            when "010" => r := shift_right(val, 2) + shift_right(val, 3);
            when "011" => r := shift_right(val, 1);
            when "100" => r := shift_right(val, 1) + shift_right(val, 3);
            when "101" => r := shift_right(val, 1) + shift_right(val, 2);
            when "110" => r := shift_right(val, 1) + shift_right(val, 2) + shift_right(val, 3);
            when others => r := val;
        end case;
        return r;
    end function;

    --------------------------------------------------------------------------
    -- Knob / switch reads
    --------------------------------------------------------------------------
    signal s_hue_knob    : unsigned(9 downto 0);
    signal s_change_rate : unsigned(9 downto 0);
    signal s_seed_knob   : unsigned(9 downto 0);
    signal s_zoom        : unsigned(9 downto 0);
    signal s_space       : unsigned(9 downto 0);
    signal s_glow        : unsigned(9 downto 0);
    signal s_master      : unsigned(9 downto 0);
    signal s_animate     : std_logic;
    signal s_bg_tint     : std_logic;
    signal s_scan_en     : std_logic;
    signal s_font_sel    : std_logic;
    signal s_invert      : std_logic;

    --------------------------------------------------------------------------
    -- Timing / resolution
    --------------------------------------------------------------------------
    signal s_timing  : t_video_timing_port;
    signal s_h_count : unsigned(11 downto 0);
    signal s_v_count : unsigned(11 downto 0);
    signal s_h_pixel_counter : unsigned(11 downto 0) := (others => '0');
    signal s_v_line_counter  : unsigned(11 downto 0) := (others => '0');
    signal s_measured_h      : unsigned(11 downto 0) := to_unsigned(960, 12);
    signal s_measured_v      : unsigned(11 downto 0) := to_unsigned(540, 12);
    signal s_vsync_prev  : std_logic := '1';
    signal s_vsync_pulse : std_logic := '0';

    --------------------------------------------------------------------------
    -- Per-frame constants
    --------------------------------------------------------------------------
    signal s_W   : t_uarr := (others => to_unsigned(40, 12));   -- cell PITCH per plane
    signal s_G   : t_uarr := (others => to_unsigned(40, 12));   -- glyph FOOTPRINT
    signal s_G8  : t_uarr := (others => to_unsigned(5, 12));    -- footprint/8
    signal s_delta : unsigned(11 downto 0) := (others => '0');
    signal s_Gbg   : unsigned(11 downto 0) := (others => '0');

    signal s_seed  : unsigned(15 downto 0) := (others => '0');
    signal s_T     : unsigned(23 downto 0) := (others => '0');

    signal s_num_y, s_num_u, s_num_v : unsigned(9 downto 0) := (others => '0');
    signal s_bg_y,  s_bg_u,  s_bg_v  : unsigned(9 downto 0) := (others => '0');
    signal s_glow_y : unsigned(9 downto 0) := (others => '0');
    signal s_glow_on : std_logic := '0';

    -- sequential zoom multiply (NP products)
    signal s_zprod  : t_zparr := (others => (others => '0'));
    signal s_cz     : t_czarr := (others => (others => '0'));
    signal s_zm_acc : t_zparr := (others => (others => '0'));
    signal s_zm_cnt : integer range 0 to 12 := 12;
    signal s_zm_busy : std_logic := '0';

    --------------------------------------------------------------------------
    -- Per-frame reducer (centre offsets + glyph-pixel offsets; no divider)
    --------------------------------------------------------------------------
    type t_init_state is (S_IDLE, S_RED, S_DONE);
    signal s_istate : t_init_state := S_IDLE;
    signal s_iacc   : signed(12 downto 0) := (others => '0');
    signal s_step   : signed(12 downto 0) := (others => '0');
    signal s_iidx   : signed(8 downto 0)  := (others => '0');
    signal s_ipl    : integer range 0 to NP - 1 := 0;
    signal s_iph    : integer range 0 to 3 := 0;     -- 0=Xcell 1=Xglyph 2=Ycell 3=Yglyph
    signal s_itmp   : unsigned(11 downto 0) := (others => '0');
    signal s_col0, s_row0 : t_sarr := (others => (others => '0'));
    signal s_remx, s_remy : t_uarr := (others => (others => '0'));
    signal s_gx0, s_gy0   : t_u3arr := (others => (others => '0'));
    signal s_r8x0, s_r8y0 : t_uarr := (others => (others => '0'));

    --------------------------------------------------------------------------
    -- Per-pixel / per-line accumulators
    --------------------------------------------------------------------------
    signal s_col, s_row : t_sarr := (others => (others => '0'));
    signal s_remh, s_remh8 : t_uarr := (others => (others => '0'));
    signal s_remv, s_remv8 : t_uarr := (others => (others => '0'));
    signal s_gx, s_gy : t_u3arr := (others => (others => '0'));
    signal s_firstline : std_logic := '1';

    --------------------------------------------------------------------------
    -- Pipeline registers (per plane)
    --------------------------------------------------------------------------
    signal s0_col, s0_row : t_sarr := (others => (others => '0'));
    signal s0_gx, s0_gy   : t_u3arr := (others => (others => '0'));
    signal s0_render      : t_slarr := (others => '0');

    signal s1_base : t_u16arr := (others => (others => '0'));
    signal s1_gx, s1_gy : t_u3arr := (others => (others => '0'));
    signal s1_render : t_slarr := (others => '0');

    signal sA_gen8 : t_slv8arr := (others => (others => '0'));
    signal sA_base : t_u16arr := (others => (others => '0'));
    signal sA_gx, sA_gy : t_u3arr := (others => (others => '0'));
    signal sA_owned, sA_render : t_slarr := (others => '0');

    signal s2_idx : t_idxarr := (others => 0);
    signal s2_gx, s2_gy : t_u3arr := (others => (others => '0'));
    signal s2_vis : t_slarr := (others => '0');   -- owned and render

    signal s3_rb_u, s3_rb_c, s3_rb_d : t_slv8arr := (others => (others => '0'));
    signal s3_gx : t_u3arr := (others => (others => '0'));
    signal s3_vis : t_slarr := (others => '0');

    signal s4_center, s4_glow : t_slarr := (others => '0');

    -- sync delay pipeline (aligned to s4): [vpar field avid vsync hsync]
    type t_syncp is array (0 to 5) of std_logic_vector(4 downto 0);
    signal s_syncp : t_syncp := (others => (others => '0'));

    signal s5_y, s5_u, s5_v : unsigned(9 downto 0) := (others => '0');
    signal s5_sync : std_logic_vector(4 downto 0) := (others => '0');
    signal s_io : t_video_stream_yuv444_30b;

begin

    s_hue_knob    <= unsigned(registers_in(0));
    s_change_rate <= unsigned(registers_in(1));
    s_seed_knob   <= unsigned(registers_in(2));
    s_master      <= unsigned(registers_in(3));   -- K4 Brightness
    s_space       <= unsigned(registers_in(4));    -- K5 Spacing
    s_glow        <= unsigned(registers_in(5));
    s_zoom        <= unsigned(registers_in(7));    -- P12 Zoom (slider)
    s_animate     <= registers_in(6)(0);
    s_bg_tint     <= registers_in(6)(1);
    s_scan_en     <= registers_in(6)(2);
    s_font_sel    <= registers_in(6)(3);
    s_invert      <= registers_in(6)(4);

    --------------------------------------------------------------------------
    -- Timing infrastructure (from ziggurat)
    --------------------------------------------------------------------------
    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    pixel_counter_inst : entity work.pixel_counter
        port map (clk => clk, timing => s_timing,
                  h_count => s_h_count, v_count => s_v_count);

    p_measure_resolution : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_h_pixel_counter > 0 then s_measured_h <= s_h_pixel_counter; end if;
                s_h_pixel_counter <= (others => '0');
            elsif s_timing.avid = '1' then
                s_h_pixel_counter <= s_h_pixel_counter + 1;
            end if;
            if s_timing.vsync_start = '1' then
                if s_v_line_counter > 0 then s_measured_v <= s_v_line_counter; end if;
                s_v_line_counter <= (others => '0');
            elsif s_timing.avid_start = '1' then
                s_v_line_counter <= s_v_line_counter + 1;
            end if;
        end if;
    end process;

    p_vsync_edge : process(clk)
    begin
        if rising_edge(clk) then
            s_vsync_prev <= s_timing.vsync_n;
            if s_vsync_prev = '0' and s_timing.vsync_n = '1' then
                s_vsync_pulse <= '1';
            else
                s_vsync_pulse <= '0';
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Sequential zoom multiply: zprod(i) = cz(i) * delta, shift-add per cycle.
    --------------------------------------------------------------------------
    p_zmul : process(clk)
        variable v_cz   : unsigned(11 downto 0);
        variable v_term : unsigned(21 downto 0);
    begin
        if rising_edge(clk) then
            if s_vsync_pulse = '1' then
                for i in 0 to NP - 1 loop
                    v_cz := resize(s_zoom, 12) + shift_right(resize(s_zoom, 12), C_GSHIFT(i));
                    if v_cz > to_unsigned(1024, 12) then v_cz := to_unsigned(1024, 12); end if;
                    s_cz(i)     <= v_cz(10 downto 0);
                    s_zm_acc(i) <= (others => '0');
                end loop;
                s_zm_cnt  <= 0;
                s_zm_busy <= '1';
            elsif s_zm_busy = '1' then
                if s_zm_cnt <= 10 then
                    v_term := shift_left(resize(s_delta, 22), s_zm_cnt);
                    for i in 0 to NP - 1 loop
                        if s_cz(i)(s_zm_cnt) = '1' then
                            s_zm_acc(i) <= s_zm_acc(i) + v_term;
                        end if;
                    end loop;
                    s_zm_cnt <= s_zm_cnt + 1;
                else
                    for i in 0 to NP - 1 loop
                        s_zprod(i) <= s_zm_acc(i);
                    end loop;
                    s_zm_busy <= '0';
                end if;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Per-frame constants (latched at vsync).
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_Gbg, v_Gfg, v_G : unsigned(11 downto 0);
        variable v_hidx : integer range 0 to 15;
        variable v_y0, v_u0, v_v0 : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_Gbg := shift_right(s_measured_h, 5);   -- ~32 across (packed, dense)
            v_Gfg := shift_right(s_measured_h, 3);   -- ~8 across  (zoomed in)
            s_Gbg <= v_Gbg;
            if v_Gfg > v_Gbg then s_delta <= v_Gfg - v_Gbg; else s_delta <= (others => '0'); end if;

            if s_vsync_pulse = '1' then
                for i in 0 to NP - 1 loop
                    v_G := s_Gbg + resize(shift_right(s_zprod(i), 10), 12);
                    if v_G > v_Gfg then v_G := v_Gfg; end if;
                    if v_G < 8 then v_G := to_unsigned(8, 12); end if;
                    s_G(i)  <= v_G;
                    s_G8(i) <= shift_right(v_G, 3);
                end loop;

                s_seed <= mix16(resize(s_seed_knob, 16) xor x"1B7F");
                if s_animate = '1' then
                    s_T <= s_T + (resize(shift_right(s_change_rate, 4), 24) + 1);
                end if;

                v_hidx := to_integer(s_hue_knob(9 downto 6));
                v_y0 := C_HUE_Y(v_hidx);
                v_u0 := C_HUE_U(v_hidx);
                v_v0 := C_HUE_V(v_hidx);
                s_num_y <= scale8(v_y0, s_master(9 downto 7));
                s_num_u <= v_u0;
                s_num_v <= v_v0;
            end if;
        end if;
    end process;

    -- Cell PITCH = footprint + spacing gap, in its own registered process so the
    -- gap shift-add stays off the G-compute cone.
    p_pitch : process(clk)
        variable v_gap : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            for i in 0 to NP - 1 loop
                case s_space(9 downto 7) is
                    when "000"  => v_gap := (others => '0');
                    when "001"  => v_gap := shift_right(s_G(i), 2);
                    when "010"  => v_gap := shift_right(s_G(i), 1);
                    when "011"  => v_gap := s_G(i);
                    when "100"  => v_gap := s_G(i) + shift_right(s_G(i), 1);
                    when "101"  => v_gap := shift_left(s_G(i), 1);
                    when "110"  => v_gap := shift_left(s_G(i), 1) + s_G(i);
                    when others => v_gap := shift_left(s_G(i), 2);
                end case;
                s_W(i) <= s_G(i) + v_gap;
            end loop;
        end if;
    end process;

    -- Glow + background colours from the REGISTERED number colour (keeps the two
    -- serial scale8 muxes off one timing cone).
    p_colaux : process(clk)
    begin
        if rising_edge(clk) then
            s_glow_y <= shift_right(scale8(s_num_y, s_glow(9 downto 7)), 1);
            if s_glow > to_unsigned(32, 10) then s_glow_on <= '1'; else s_glow_on <= '0'; end if;
            if s_bg_tint = '1' then
                s_bg_y <= shift_right(s_num_y, 4); s_bg_u <= s_num_u; s_bg_v <= s_num_v;
            else
                s_bg_y <= (others => '0'); s_bg_u <= C_MID; s_bg_v <= C_MID;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Reducer: per plane find col0/remx/gx0/r8x0 (from -cx) and the Y analogues.
    -- 4 phases per plane: reduce -cx by W; reduce that remainder by G8 (glyph
    -- column of the left partial cell -- fixes the left/top edge alignment); same
    -- for -cy.  All shift/add, no divider.
    --------------------------------------------------------------------------
    p_init : process(clk)
        variable v_cx, v_cy : signed(12 downto 0);
        variable v_cap : boolean;
    begin
        if rising_edge(clk) then
            v_cx := -signed(resize(shift_right(s_measured_h, 1), 13));
            v_cy := -signed(resize(shift_right(s_measured_v, 1), 13));
            v_cap := (s_iph = 1) or (s_iph = 3);   -- glyph (G8) phases cap idx at 7

            case s_istate is
                when S_IDLE =>
                    if s_vsync_pulse = '1' then
                        s_ipl <= 0; s_iph <= 0; s_iidx <= (others => '0');
                        s_iacc <= v_cx;
                        s_step <= signed(resize(s_W(0), 13));
                        s_istate <= S_RED;
                    end if;
                when S_RED =>
                    if (not v_cap) and (s_iacc < 0) then
                        s_iacc <= s_iacc + s_step;
                        s_iidx <= s_iidx - 1;
                    elsif (s_iacc >= s_step) and ((not v_cap) or (s_iidx < 7)) then
                        s_iacc <= s_iacc - s_step;
                        s_iidx <= s_iidx + 1;
                    else
                        -- phase complete; preload the next phase's step register
                        case s_iph is
                            when 0 =>   -- Xcell done: col0, remx; keep acc for glyph reduce
                                s_col0(s_ipl) <= s_iidx;
                                s_itmp <= unsigned(s_iacc(11 downto 0));
                                s_iidx <= (others => '0');
                                s_iph  <= 1;
                                s_step <= signed(resize(s_G8(s_ipl), 13));
                            when 1 =>   -- Xglyph done: gx0, r8x0, store remx
                                s_gx0(s_ipl)  <= unsigned(s_iidx(2 downto 0));
                                s_r8x0(s_ipl) <= unsigned(s_iacc(11 downto 0));
                                s_remx(s_ipl) <= s_itmp;
                                s_iacc <= v_cy;
                                s_iidx <= (others => '0');
                                s_iph  <= 2;
                                s_step <= signed(resize(s_W(s_ipl), 13));
                            when 2 =>   -- Ycell done: row0, remy
                                s_row0(s_ipl) <= s_iidx;
                                s_itmp <= unsigned(s_iacc(11 downto 0));
                                s_iidx <= (others => '0');
                                s_iph  <= 3;
                                s_step <= signed(resize(s_G8(s_ipl), 13));
                            when others =>  -- Yglyph done: gy0, r8y0, store remy; next plane
                                s_gy0(s_ipl)  <= unsigned(s_iidx(2 downto 0));
                                s_r8y0(s_ipl) <= unsigned(s_iacc(11 downto 0));
                                s_remy(s_ipl) <= s_itmp;
                                if s_ipl = NP - 1 then
                                    s_istate <= S_DONE;
                                else
                                    s_ipl <= s_ipl + 1; s_iph <= 0;
                                    s_iacc <= v_cx; s_iidx <= (others => '0');
                                    s_step <= signed(resize(s_W(s_ipl + 1), 13));
                                end if;
                        end case;
                    end if;
                when S_DONE =>
                    s_istate <= S_IDLE;
            end case;
        end if;
    end process;

    p_firstline : process(clk)
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_firstline <= '1';
            elsif s_timing.avid_start = '1' then
                s_firstline <= '0';
            end if;
        end if;
    end process;

    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_syncp(0) <= s_v_count(0) & data_in.field_n & data_in.avid &
                          data_in.vsync_n & data_in.hsync_n;
            for k in 1 to 5 loop
                s_syncp(k) <= s_syncp(k - 1);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S0: per-pixel cell accumulators + footprint gate, all planes.
    --------------------------------------------------------------------------
    p_acc : process(clk)
        variable cur_col, cur_row : signed(8 downto 0);
        variable cur_gx, cur_gy   : unsigned(2 downto 0);
        variable nx_col           : signed(8 downto 0);
        variable nx_remh, nx_remh8 : unsigned(11 downto 0);
        variable nx_gx            : unsigned(2 downto 0);
        variable ln_row           : signed(8 downto 0);
        variable ln_remv, ln_remv8 : unsigned(11 downto 0);
        variable ln_gy            : unsigned(2 downto 0);
        variable cur_posx, cur_posy : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            for i in 0 to NP - 1 loop
                cur_col := s_col(i); cur_row := s_row(i);
                cur_gx := s_gx(i);   cur_gy := s_gy(i);
                cur_posx := s_remh(i); cur_posy := s_remv(i);

                if s_timing.avid_start = '1' then
                    -- vertical advance (per line)
                    if s_firstline = '1' then
                        ln_row := s_row0(i); ln_remv := s_remy(i);
                        ln_gy := s_gy0(i); ln_remv8 := s_r8y0(i);
                    else
                        ln_remv := s_remv(i) + 1; ln_remv8 := s_remv8(i) + 1;
                        if ln_remv >= s_W(i) then
                            ln_row := s_row(i) + 1; ln_remv := (others => '0');
                            ln_gy := (others => '0'); ln_remv8 := (others => '0');
                        elsif ln_remv8 >= s_G8(i) then
                            ln_row := s_row(i);
                            if s_gy(i) < 7 then ln_gy := s_gy(i) + 1; else ln_gy := s_gy(i); end if;
                            ln_remv8 := (others => '0');
                        else
                            ln_row := s_row(i); ln_gy := s_gy(i);
                        end if;
                    end if;
                    s_row(i) <= ln_row; s_remv(i) <= ln_remv; s_gy(i) <= ln_gy; s_remv8(i) <= ln_remv8;

                    -- horizontal load (first pixel of the line = col0 with its glyph offset)
                    cur_col := s_col0(i); cur_row := ln_row;
                    cur_gx := s_gx0(i); cur_gy := ln_gy;
                    cur_posx := s_remx(i); cur_posy := ln_remv;
                    nx_remh := s_remx(i) + 1; nx_remh8 := s_r8x0(i) + 1;
                    if nx_remh >= s_W(i) then
                        nx_col := s_col0(i) + 1; nx_remh := (others => '0');
                        nx_gx := (others => '0'); nx_remh8 := (others => '0');
                    elsif nx_remh8 >= s_G8(i) then
                        nx_col := s_col0(i);
                        if s_gx0(i) < 7 then nx_gx := s_gx0(i) + 1; else nx_gx := s_gx0(i); end if;
                        nx_remh8 := (others => '0');
                    else
                        nx_col := s_col0(i); nx_gx := s_gx0(i);
                    end if;
                    s_col(i) <= nx_col; s_remh(i) <= nx_remh; s_gx(i) <= nx_gx; s_remh8(i) <= nx_remh8;

                elsif s_timing.avid = '1' then
                    nx_remh := s_remh(i) + 1; nx_remh8 := s_remh8(i) + 1;
                    if nx_remh >= s_W(i) then
                        nx_col := s_col(i) + 1; nx_remh := (others => '0');
                        nx_gx := (others => '0'); nx_remh8 := (others => '0');
                    elsif nx_remh8 >= s_G8(i) then
                        nx_col := s_col(i);
                        if s_gx(i) < 7 then nx_gx := s_gx(i) + 1; else nx_gx := s_gx(i); end if;
                        nx_remh8 := (others => '0');
                    else
                        nx_col := s_col(i); nx_gx := s_gx(i);
                    end if;
                    s_col(i) <= nx_col; s_remh(i) <= nx_remh; s_gx(i) <= nx_gx; s_remh8(i) <= nx_remh8;
                end if;

                s0_col(i) <= cur_col; s0_row(i) <= cur_row;
                s0_gx(i) <= cur_gx;   s0_gy(i) <= cur_gy;
                if (cur_posx < s_G(i)) and (cur_posy < s_G(i)) then
                    s0_render(i) <= '1';
                else
                    s0_render(i) <= '0';
                end if;
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S1: spatial de-correlation.
    --------------------------------------------------------------------------
    p_hashA : process(clk)
    begin
        if rising_edge(clk) then
            for i in 0 to NP - 1 loop
                s1_base(i) <= spreadcr(s0_col(i), s0_row(i));
                s1_gx(i) <= s0_gx(i); s1_gy(i) <= s0_gy(i);
                s1_render(i) <= s0_render(i);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- SA: per-cell hash -> staggered generation index + plane ownership.
    --------------------------------------------------------------------------
    p_hashMix : process(clk)
        variable v_h : unsigned(15 downto 0);
        variable v_rate : integer range 0 to 9;
        variable v_gen  : unsigned(23 downto 0);
        variable v_pof  : integer range 0 to 3;
    begin
        if rising_edge(clk) then
            for i in 0 to NP - 1 loop
                v_h := mix16(s1_base(i) xor s_seed);
                v_rate := 2 + to_integer(v_h(10 downto 8));
                v_gen  := shift_right(s_T + resize(v_h(7 downto 0), 24), v_rate);
                sA_gen8(i) <= std_logic_vector(v_gen(7 downto 0));
                v_pof := to_integer(v_h(13 downto 12));
                if v_pof >= NP then v_pof := v_pof - NP; end if;
                if v_pof = i then sA_owned(i) <= '1'; else sA_owned(i) <= '0'; end if;
                sA_base(i) <= s1_base(i);
                sA_gx(i) <= s1_gx(i); sA_gy(i) <= s1_gy(i);
                sA_render(i) <= s1_render(i);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S2: digit glyph index.
    --------------------------------------------------------------------------
    p_hashB : process(clk)
        variable v_h : unsigned(15 downto 0);
        variable v_off : integer range 0 to 8;
    begin
        if rising_edge(clk) then
            for i in 0 to NP - 1 loop
                v_h := mix16(sA_base(i) xor s_seed xor (unsigned(sA_gen8(i)) & unsigned(sA_gen8(i))));
                v_off := C_DIGLUT(to_integer(v_h(7 downto 4)));
                if s_font_sel = '1' then s2_idx(i) <= v_off + 9;
                else                     s2_idx(i) <= v_off; end if;
                s2_gx(i) <= sA_gx(i); s2_gy(i) <= sA_gy(i);
                s2_vis(i) <= sA_owned(i) and sA_render(i);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S3: glyph rows (centre + above/below for the dilation glow).
    --------------------------------------------------------------------------
    p_font : process(clk)
        variable v_gy : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            for i in 0 to NP - 1 loop
                v_gy := to_integer(s2_gy(i));
                s3_rb_c(i) <= C_DIGITS(s2_idx(i), v_gy);
                if v_gy > 0 then s3_rb_u(i) <= C_DIGITS(s2_idx(i), v_gy - 1);
                else             s3_rb_u(i) <= (others => '0'); end if;
                if v_gy < 7 then s3_rb_d(i) <= C_DIGITS(s2_idx(i), v_gy + 1);
                else             s3_rb_d(i) <= (others => '0'); end if;
                s3_gx(i) <= s2_gx(i);
                s3_vis(i) <= s2_vis(i);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S4: per-plane centre pixel + 1-glyph-pixel dilation glow.
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable p : integer range -1 to 8;
        variable v_center, v_glow : std_logic;
    begin
        if rising_edge(clk) then
            for i in 0 to NP - 1 loop
                p := to_integer(s3_gx(i));
                v_center := getbit(s3_rb_c(i), p) and s3_vis(i);
                v_glow := ( getbit(s3_rb_u(i), p-1) or getbit(s3_rb_u(i), p) or getbit(s3_rb_u(i), p+1) or
                            getbit(s3_rb_c(i), p-1) or                          getbit(s3_rb_c(i), p+1) or
                            getbit(s3_rb_d(i), p-1) or getbit(s3_rb_d(i), p) or getbit(s3_rb_d(i), p+1) )
                          and s3_vis(i) and (not v_center);
                s4_center(i) <= v_center;
                s4_glow(i)   <= v_glow;
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S5: composite front-to-back, colour, depth fade, invert, scanlines.
    --------------------------------------------------------------------------
    p_color : process(clk)
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_win : integer range -1 to NP - 1;
        variable v_glowsel : std_logic;
        variable v_numy_f : unsigned(9 downto 0);
        variable v_on_y, v_on_u, v_on_v : unsigned(9 downto 0);
        variable v_off_y, v_off_u, v_off_v : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_win := -1;
            for i in NP - 1 downto 0 loop
                if s4_center(i) = '1' then v_win := i; end if;
            end loop;
            v_glowsel := '0';
            for i in 0 to NP - 1 loop
                if s4_glow(i) = '1' then v_glowsel := '1'; end if;
            end loop;

            -- depth fade: nearer (front) plane brighter
            case v_win is
                when 0      => v_numy_f := s_num_y;
                when 1      => v_numy_f := s_num_y - shift_right(s_num_y, 3);
                when others => v_numy_f := s_num_y - shift_right(s_num_y, 2);
            end case;

            if s_invert = '0' then
                v_on_y := v_numy_f; v_on_u := s_num_u; v_on_v := s_num_v;
                v_off_y := s_bg_y;  v_off_u := s_bg_u; v_off_v := s_bg_v;
            else
                v_on_y := s_bg_y;   v_on_u := s_bg_u; v_on_v := s_bg_v;
                v_off_y := v_numy_f; v_off_u := s_num_u; v_off_v := s_num_v;
            end if;

            if v_win >= 0 then
                v_y := v_on_y; v_u := v_on_u; v_v := v_on_v;
            elsif (v_glowsel = '1') and (s_glow_on = '1') then
                v_y := s_glow_y; v_u := s_num_u; v_v := s_num_v;
            else
                v_y := v_off_y; v_u := v_off_u; v_v := v_off_v;
            end if;

            if (s_scan_en = '1') and (s_syncp(5)(4) = '1') then
                v_y := v_y - shift_right(v_y, 2);
            end if;

            s5_y <= v_y; s5_u <= v_u; s5_v <= v_v;
            s5_sync <= s_syncp(5);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Output.
    --------------------------------------------------------------------------
    p_io : process(clk)
    begin
        if rising_edge(clk) then
            s_io.y       <= std_logic_vector(s5_y);
            s_io.u       <= std_logic_vector(s5_u);
            s_io.v       <= std_logic_vector(s5_v);
            s_io.hsync_n <= s5_sync(0);
            s_io.vsync_n <= s5_sync(1);
            s_io.avid    <= s5_sync(2);
            s_io.field_n <= s5_sync(3);
        end if;
    end process;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture ziffern;
