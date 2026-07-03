-- ziffern.vhd  (sub-pixel multi-plane perspective, P12 zoom position)
--
-- A Kraftwerk-inspired green "computer screen" wall of pixelated digits 1-9.
-- The digits change at random, staggered times; the field zooms toward the viewer
-- in true perspective (centred on screen), driven by P12 as a ZOOM POSITION.
--
-- Smooth zoom: each plane is a fractional PHASE ACCUMULATOR (DDA), NOT an integer
-- cell grid.  Per pixel a fixed-point phase advances by step = (1<<FRAC)/cellsize;
-- the cell index is the integer part, the glyph column the top fraction bits.
-- Centring is phase = (x - centre)*step (continuous), so the grid GLIDES as the
-- cell size changes instead of snapping on a per-pixel modulo.  No per-pixel
-- multiply/divide (step + centre offset are per-frame; per pixel is just an add).
--
-- Depth: NP=3 planes composited front-to-back, each at its own cell size that
-- scales from dense/far to large/near at a different speed (front fastest); the
-- numbers are SPLIT across the planes (a balanced per-cell hash assigns each cell
-- to one plane) so each number lives at one depth.  P12=100% = all far/dense/small
-- (one plane), pull toward 0% = enlarge in perspective at different speeds,
-- 0% = all co-planar at the front.  Movement only when P12 moves.
--
-- Background is black or a dark tint; S9 keys incoming video into the black areas.
--
-- Controls: K1 Color, K2 Change Rate, K3 Number Set, K4 Brightness, K5 Spacing,
--           K6 Glow, S7 Animate, S8 Background, S9 Video, S10 Font, S11 Invert,
--           P12 Zoom.
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

    constant NP   : integer := 3;        -- depth planes (front=0 .. back=NP-1)
    constant FRAC : integer := 18;       -- phase fraction bits (sub-pixel, smooth zoom)

    -- per-plane speed: cz = prog + triangle(prog)>>gshift.  triangle is 0 at both
    -- throw ends (so all planes start together / converge at the ends) and peaks
    -- mid-throw, bumped differently per plane so the front plane leads.  shift 1 =
    -- biggest lead (fastest), 15 = ~none (slowest, straight).
    type t_gsh is array (0 to NP - 1) of integer range 0 to 15;
    constant C_GSHIFT : t_gsh := (1, 2, 15);

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    --------------------------------------------------------------------------
    -- Reciprocal LUT: C_RECIP(w) = (1<<FRAC)/w  (step = cells-per-pixel, Q.FRAC).
    --------------------------------------------------------------------------
    type t_recip is array (0 to 255) of unsigned(17 downto 0);
    function gen_recip return t_recip is
        variable r : t_recip;
        variable v : integer;
    begin
        r(0) := (others => '0');
        for w in 1 to 255 loop
            v := (2 ** FRAC) / w;
            if v > 262143 then v := 262143; end if;   -- clamp to 18 bits (only w<2)
            r(w) := to_unsigned(v, 18);
        end loop;
        return r;
    end function;
    constant C_RECIP : t_recip := gen_recip;

    --------------------------------------------------------------------------
    -- Hue ring (16 entries; index = hue_knob(9 downto 6)).  U/V-swapped, green=7.
    --------------------------------------------------------------------------
    constant C_NHUE : integer := 16;
    type t_hue is array (0 to C_NHUE - 1) of unsigned(9 downto 0);
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
    -- Digit glyphs (8x8, MSB = leftmost). idx 0..8 = plain '1'..'9';
    -- idx 9..17 = alien alphabet.
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
         9 => (x"00",x"20",x"20",x"20",x"28",x"3C",x"10",x"10"),  -- alien
        10 => (x"40",x"40",x"40",x"40",x"60",x"60",x"70",x"00"),
        11 => (x"40",x"48",x"40",x"40",x"78",x"40",x"00",x"00"),
        12 => (x"40",x"40",x"70",x"40",x"40",x"60",x"70",x"20"),
        13 => (x"00",x"20",x"20",x"38",x"24",x"20",x"20",x"20"),
        14 => (x"00",x"78",x"40",x"40",x"40",x"40",x"48",x"00"),
        15 => (x"00",x"50",x"78",x"40",x"40",x"40",x"40",x"00"),
        16 => (x"00",x"20",x"20",x"3C",x"3C",x"20",x"20",x"00"),
        17 => (x"00",x"20",x"20",x"20",x"34",x"30",x"38",x"20"));

    type t_dlut is array (0 to 15) of integer range 0 to 8;
    constant C_DIGLUT : t_dlut := (0,1,2,3,4,5,6,7,8,0,1,2,3,4,5,6);

    --------------------------------------------------------------------------
    -- Per-plane array types
    --------------------------------------------------------------------------
    type t_uarr   is array (0 to NP - 1) of unsigned(11 downto 0);
    type t_sarr   is array (0 to NP - 1) of signed(8 downto 0);
    type t_u3arr  is array (0 to NP - 1) of unsigned(2 downto 0);
    type t_u16arr is array (0 to NP - 1) of unsigned(15 downto 0);
    type t_slv8arr is array (0 to NP - 1) of std_logic_vector(7 downto 0);
    type t_idxarr is array (0 to NP - 1) of integer range 0 to 17;
    type t_slarr  is array (0 to NP - 1) of std_logic;
    type t_zparr  is array (0 to NP - 1) of unsigned(27 downto 0);
    type t_czarr  is array (0 to NP - 1) of unsigned(10 downto 0);
    type t_steparr is array (0 to NP - 1) of unsigned(17 downto 0);  -- step (cells/px)
    type t_marr   is array (0 to NP - 1) of unsigned(27 downto 0);   -- centre*step
    type t_pharr  is array (0 to NP - 1) of signed(27 downto 0);     -- phase Q10.18
    type t_frarr  is array (0 to NP - 1) of unsigned(17 downto 0);   -- cell fraction

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
    signal s_video       : std_logic;
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
    signal s_delta   : unsigned(17 downto 0) := (others => '0');    -- stepfar - stepnear
    signal s_stepfar : unsigned(17 downto 0) := to_unsigned(6554, 18);  -- (1<<FRAC)/Gbg
    signal s_stepnear: unsigned(17 downto 0) := to_unsigned(1092, 18);  -- (1<<FRAC)/Gfg
    signal s_pstep : t_steparr := (others => to_unsigned(6554, 18));  -- per-plane step (cells/px)
    signal s_mh, s_mv : t_marr := (others => (others => '0'));        -- centre*step
    signal s_spcsh : integer range 12 to 15 := 15;                    -- spacing: glyph = cell>>(...)

    signal s_seed  : unsigned(15 downto 0) := (others => '0');
    signal s_T     : unsigned(23 downto 0) := (others => '0');

    signal s_num_y, s_num_u, s_num_v : unsigned(9 downto 0) := (others => '0');
    signal s_bg_y,  s_bg_u,  s_bg_v  : unsigned(9 downto 0) := (others => '0');
    signal s_glow_y : unsigned(9 downto 0) := (others => '0');
    signal s_glow_on : std_logic := '0';

    -- sequential zoom multiply (cz*delta -> per-plane cell size)
    signal s_zprod  : t_zparr := (others => (others => '0'));
    signal s_cz     : t_czarr := (others => (others => '0'));
    signal s_zm_acc : t_zparr := (others => (others => '0'));
    signal s_zm_cnt : integer range 0 to 12 := 12;
    signal s_zm_busy : std_logic := '0';

    -- sequential centre multiply (centre*step -> phase start offset)
    signal s_pm_ah, s_pm_av : t_marr := (others => (others => '0'));
    signal s_pm_cnt  : integer range 0 to 19 := 19;
    signal s_pm_busy : std_logic := '0';
    signal s_cxr, s_cyr : unsigned(10 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Per-pixel / per-line phase accumulators
    --------------------------------------------------------------------------
    signal s_phh, s_phv : t_pharr := (others => (others => '0'));
    signal s_firstline : std_logic := '1';

    --------------------------------------------------------------------------
    -- Pipeline registers (per plane)
    --------------------------------------------------------------------------
    signal s0_col, s0_row : t_sarr := (others => (others => '0'));
    signal s0_frh, s0_frv : t_frarr := (others => (others => '0'));   -- cell fraction

    signal s1_base : t_u16arr := (others => (others => '0'));
    signal s1_gx, s1_gy : t_u3arr := (others => (others => '0'));
    signal s1_render : t_slarr := (others => '1');

    signal sA_gen8 : t_slv8arr := (others => (others => '0'));
    signal sA_base : t_u16arr := (others => (others => '0'));
    signal sA_gx, sA_gy : t_u3arr := (others => (others => '0'));
    signal sA_owned, sA_render : t_slarr := (others => '0');

    signal s2_idx : t_idxarr := (others => 0);
    signal s2_gx, s2_gy : t_u3arr := (others => (others => '0'));
    signal s2_vis : t_slarr := (others => '0');

    signal s3_rb_u, s3_rb_c, s3_rb_d : t_slv8arr := (others => (others => '0'));
    signal s3_gx : t_u3arr := (others => (others => '0'));
    signal s3_vis : t_slarr := (others => '0');

    signal s4_center, s4_glow : t_slarr := (others => '0');

    -- sync delay pipeline (aligned to s4): [vpar field avid vsync hsync]
    type t_syncp is array (0 to 5) of std_logic_vector(4 downto 0);
    signal s_syncp : t_syncp := (others => (others => '0'));

    -- incoming video delayed to align with the colour stage (S9 video key)
    type t_vidp is array (0 to 5) of unsigned(9 downto 0);
    signal s_vy, s_vu, s_vv : t_vidp := (others => (others => '0'));

    signal s5_y, s5_u, s5_v : unsigned(9 downto 0) := (others => '0');
    signal s5_sync : std_logic_vector(4 downto 0) := (others => '0');
    signal s_io : t_video_stream_yuv444_30b;

begin

    s_hue_knob    <= unsigned(registers_in(0));
    s_change_rate <= unsigned(registers_in(1));
    s_seed_knob   <= unsigned(registers_in(2));
    s_master      <= unsigned(registers_in(3));   -- K4 Brightness
    s_space       <= unsigned(registers_in(4));    -- K5 Spacing (density)
    s_glow        <= unsigned(registers_in(5));
    s_zoom        <= unsigned(registers_in(7));    -- P12 Zoom (100%=far, 0%=front)
    s_animate     <= registers_in(6)(0);
    s_bg_tint     <= registers_in(6)(1);
    s_video       <= registers_in(6)(2);
    s_font_sel    <= registers_in(6)(3);
    s_invert      <= registers_in(6)(4);

    --------------------------------------------------------------------------
    -- Timing infrastructure
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
    -- prog = 1023 - P12 (forward progress: 0 at P12=100% far, 1023 at 0% front).
    --------------------------------------------------------------------------
    p_zmul : process(clk)
        variable v_ez   : unsigned(10 downto 0);   -- prog = 1023 - P12
        variable v_tri  : unsigned(10 downto 0);   -- triangle: 0 at ends, peak mid
        variable v_cz   : unsigned(11 downto 0);
        variable v_term : unsigned(27 downto 0);
    begin
        if rising_edge(clk) then
            if s_vsync_pulse = '1' then
                v_ez := to_unsigned(1023, 11) - resize(s_zoom, 11);
                if v_ez < to_unsigned(512, 11) then v_tri := v_ez;
                else v_tri := to_unsigned(1023, 11) - v_ez; end if;
                for i in 0 to NP - 1 loop
                    -- prog + leading bump; 0 at both ends -> all start together, converge
                    v_cz := resize(v_ez, 12) + resize(shift_right(v_tri, C_GSHIFT(i)), 12);
                    s_cz(i)     <= v_cz(10 downto 0);
                    s_zm_acc(i) <= (others => '0');
                end loop;
                s_zm_cnt  <= 0;
                s_zm_busy <= '1';
            elsif s_zm_busy = '1' then
                if s_zm_cnt <= 10 then
                    v_term := shift_left(resize(s_delta, 28), s_zm_cnt);
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
    -- Per-frame cell sizes, colour, change-time (latched at vsync).
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_Gbg, v_Gfg : unsigned(11 downto 0);
        variable v_sf, v_sn : unsigned(17 downto 0);
        variable v_hidx : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            v_Gbg := shift_right(s_measured_h, 5);   -- far cell size (~32 across)
            v_Gfg := shift_right(s_measured_h, 3);   -- near cell size (~8 across)
            -- step endpoints (cells/pixel) = (1<<FRAC)/cellsize, via the recip LUT.
            v_sf := C_RECIP(to_integer(v_Gbg(7 downto 0)));   -- far  (bigger step)
            v_sn := C_RECIP(to_integer(v_Gfg(7 downto 0)));   -- near (smaller step)
            s_stepfar  <= v_sf;
            s_stepnear <= v_sn;
            -- delta from the REGISTERED endpoints (keeps the LUT-read off the subtract cone)
            if s_stepfar > s_stepnear then s_delta <= s_stepfar - s_stepnear;
            else s_delta <= (others => '0'); end if;

            -- K5 Spacing: glyph occupies the cell shifted by s_spcsh (11=packed .. 8=wide gap)
            case s_space(9 downto 8) is
                when "00"   => s_spcsh <= 15;   -- packed (glyph fills cell)
                when "01"   => s_spcsh <= 14;
                when "10"   => s_spcsh <= 13;
                when others => s_spcsh <= 12;   -- wide gap
            end case;

            if s_vsync_pulse = '1' then
                s_seed <= mix16(resize(s_seed_knob, 16) xor x"1B7F");
                if s_animate = '1' then
                    s_T <= s_T + (resize(shift_right(s_change_rate, 6), 24) + 1);
                end if;

                v_hidx := to_integer(s_hue_knob(9 downto 6));
                s_num_y <= scale8(C_HUE_Y(v_hidx), s_master(9 downto 7));
                s_num_u <= C_HUE_U(v_hidx);
                s_num_v <= C_HUE_V(v_hidx);
            end if;
        end if;
    end process;

    -- per-plane step = lerp(stepfar, stepnear, prog_i):  step = stepfar - zprod>>10,
    -- where zprod = clamp(prog + prog>>gshift) * (stepfar-stepnear) from p_zmul.
    -- step is continuous in P12 (no per-G quantisation) -> the zoom GLIDES.
    p_recip : process(clk)
        variable v_st : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            for i in 0 to NP - 1 loop
                v_st := s_stepfar - resize(shift_right(s_zprod(i), 10), 18);
                if v_st < to_unsigned(64, 18) then v_st := to_unsigned(64, 18); end if;
                s_pstep(i) <= v_st;
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Sequential centre multiply: m(i) = centre * step(i) (shift-add per cycle),
    -- so the line/column start phase = -m(i) puts the screen centre at phase 0.
    --------------------------------------------------------------------------
    p_phasemul : process(clk)
        variable th, tv : unsigned(27 downto 0);
    begin
        if rising_edge(clk) then
            if s_vsync_pulse = '1' then
                s_cxr <= resize(shift_right(s_measured_h, 1), 11);
                s_cyr <= resize(shift_right(s_measured_v, 1), 11);
                for i in 0 to NP - 1 loop
                    s_pm_ah(i) <= (others => '0');
                    s_pm_av(i) <= (others => '0');
                end loop;
                s_pm_cnt  <= 0;
                s_pm_busy <= '1';
            elsif s_pm_busy = '1' then
                if s_pm_cnt <= 17 then
                    th := shift_left(resize(s_cxr, 28), s_pm_cnt);
                    tv := shift_left(resize(s_cyr, 28), s_pm_cnt);
                    for i in 0 to NP - 1 loop
                        if s_pstep(i)(s_pm_cnt) = '1' then
                            s_pm_ah(i) <= s_pm_ah(i) + th;
                            s_pm_av(i) <= s_pm_av(i) + tv;
                        end if;
                    end loop;
                    s_pm_cnt <= s_pm_cnt + 1;
                else
                    for i in 0 to NP - 1 loop
                        s_mh(i) <= s_pm_ah(i);
                        s_mv(i) <= s_pm_av(i);
                    end loop;
                    s_pm_busy <= '0';
                end if;
            end if;
        end if;
    end process;

    -- Glow + background colours from the REGISTERED number colour.
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
            s_vy(0) <= unsigned(data_in.y);
            s_vu(0) <= unsigned(data_in.u);
            s_vv(0) <= unsigned(data_in.v);
            for k in 1 to 5 loop
                s_syncp(k) <= s_syncp(k - 1);
                s_vy(k) <= s_vy(k - 1);
                s_vu(k) <= s_vu(k - 1);
                s_vv(k) <= s_vv(k - 1);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S0: per-pixel phase accumulators -> cell index + glyph column, all planes.
    --   phase advances by step each pixel (horiz) / each line (vert); cell =
    --   integer part, glyph col/row = top 3 fraction bits.  Smooth = no modulo.
    --------------------------------------------------------------------------
    p_acc : process(clk)
        variable stp : signed(27 downto 0);
        variable cur_h, cur_v, nx_v : signed(27 downto 0);
    begin
        if rising_edge(clk) then
            for i in 0 to NP - 1 loop
                stp := signed(resize(s_pstep(i), 28));

                if s_timing.avid_start = '1' then
                    -- vertical advance (per line)
                    if s_firstline = '1' then
                        nx_v := -signed(resize(s_mv(i), 28));
                    else
                        nx_v := s_phv(i) + stp;
                    end if;
                    s_phv(i) <= nx_v;
                    cur_v := nx_v;
                    -- horizontal restart at line's first pixel
                    cur_h := -signed(resize(s_mh(i), 28));
                    s_phh(i) <= cur_h + stp;
                elsif s_timing.avid = '1' then
                    cur_h := s_phh(i);
                    cur_v := s_phv(i);
                    s_phh(i) <= cur_h + stp;
                else
                    cur_h := s_phh(i);
                    cur_v := s_phv(i);
                end if;

                -- register cell index + fraction; the spacing slice happens in S1.
                s0_col(i) <= cur_h(26 downto 18);
                s0_row(i) <= cur_v(26 downto 18);
                s0_frh(i) <= unsigned(cur_h(17 downto 0));
                s0_frv(i) <= unsigned(cur_v(17 downto 0));
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- S1: spatial de-correlation.
    --------------------------------------------------------------------------
    p_hashA : process(clk)
        variable v_slh, v_slv : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            for i in 0 to NP - 1 loop
                s1_base(i) <= spreadcr(s0_col(i), s0_row(i));
                -- spacing: cell split into 2^(18-s_spcsh) slots; glyph = first 8, gap after.
                v_slh := shift_right(s0_frh(i), s_spcsh);
                v_slv := shift_right(s0_frv(i), s_spcsh);
                s1_gx(i) <= v_slh(2 downto 0);
                s1_gy(i) <= v_slv(2 downto 0);
                if (v_slh < 8) and (v_slv < 8) then s1_render(i) <= '1';
                else s1_render(i) <= '0'; end if;
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- SA: per-cell hash -> staggered generation index + balanced plane split.
    --------------------------------------------------------------------------
    p_hashMix : process(clk)
        variable v_h : unsigned(15 downto 0);
        variable v_pc : integer range 0 to 4;
        variable v_rate : integer range 0 to 10;
        variable v_gen  : unsigned(23 downto 0);
        variable v_hi : unsigned(7 downto 0);
        variable v_pof : integer range 0 to NP - 1;
    begin
        if rising_edge(clk) then
            for i in 0 to NP - 1 loop
                v_h := mix16(s1_base(i) xor s_seed);
                v_pc := 0;
                for b in 8 to 11 loop
                    if v_h(b) = '1' then v_pc := v_pc + 1; end if;
                end loop;
                v_rate := 6 + v_pc;                                  -- 6..10
                v_gen  := shift_right(s_T + resize(v_h(7 downto 0), 24), v_rate);
                sA_gen8(i) <= std_logic_vector(v_gen(7 downto 0));
                -- balanced 3-way split (well-mixed high byte -> equal thirds)
                v_hi := v_h(15 downto 8);
                if    v_hi < to_unsigned(86, 8)  then v_pof := 0;
                elsif v_hi < to_unsigned(171, 8) then v_pof := 1;
                else                                  v_pof := 2; end if;
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
    -- S5: composite front-to-back, colour, depth fade, video key, invert.
    --------------------------------------------------------------------------
    p_color : process(clk)
        variable v_y, v_u, v_v : unsigned(9 downto 0);
        variable v_bgy, v_bgu, v_bgv : unsigned(9 downto 0);
        variable v_win : integer range -1 to NP - 1;
        variable v_glowsel : std_logic;
        variable v_numy_f : unsigned(9 downto 0);
        variable v_on_y, v_on_u, v_on_v : unsigned(9 downto 0);
        variable v_off_y, v_off_u, v_off_v : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if s_video = '1' then
                if s_bg_tint = '1' then
                    v_bgy := resize(shift_right(resize(s_bg_y,11) + resize(s_vy(5),11), 1), 10);
                    v_bgu := resize(shift_right(resize(s_bg_u,11) + resize(s_vu(5),11), 1), 10);
                    v_bgv := resize(shift_right(resize(s_bg_v,11) + resize(s_vv(5),11), 1), 10);
                else
                    v_bgy := s_vy(5); v_bgu := s_vu(5); v_bgv := s_vv(5);
                end if;
            else
                v_bgy := s_bg_y; v_bgu := s_bg_u; v_bgv := s_bg_v;
            end if;

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
                v_off_y := v_bgy;   v_off_u := v_bgu; v_off_v := v_bgv;
            else
                v_on_y := v_bgy;    v_on_u := v_bgu; v_on_v := v_bgv;
                v_off_y := v_numy_f; v_off_u := s_num_u; v_off_v := s_num_v;
            end if;

            if v_win >= 0 then
                v_y := v_on_y; v_u := v_on_u; v_v := v_on_v;
            elsif (v_glowsel = '1') and (s_glow_on = '1') then
                v_y := s_glow_y; v_u := s_num_u; v_v := s_num_v;
            else
                v_y := v_off_y; v_u := v_off_u; v_v := v_off_v;
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
