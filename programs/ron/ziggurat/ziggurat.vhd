-- ziggurat.vhd  (v0.4 — per-pixel isometric heightfield ray-march)
--
-- Fills the WHOLE screen with a dense isometric field of stacked boxes (an
-- Escher-style city).  No sprites and no line buffer: every output pixel marches
-- the fixed isometric view-ray through a procedural heightfield in a fixed-depth
-- pipeline and keeps the nearest box hit, so there is no box-count limit.
--
-- Math (no per-pixel multiply):
--  * Two cheap accumulators give the iso coords:  U = b*x - a*y,  V = b*x + a*y
--    (step +b per pixel, ∓a per line).
--  * With W2 = 2*a*b, the cell under a pixel is  i = floor(V/W2),  j = floor(-U/W2)
--    (kept as accumulators with remainders — no division), and a box of height
--    level L has aH = L*ab (a shift of W2/2, no multiply).
--  * A box at cell (i,j) covers the pixel iff   V + aH >= W2*i   AND   U + W2*j <= aH.
--    Marching i from front (iv+KF) down to iv and taking the first cover gives the
--    nearest visible box (occlusion).  Front faces win over the ones behind.
--  * Face of the winning box (vs its front-top corner C):
--      u' = U - U_C,  v' = V - V_C  -> TOP (u'>0,v'<0), RIGHT (v'>0,u'+v'>0), LEFT.
--
-- Controls: K1 Base Hue, K2 Box Scale (cell size), K4 Shade Contrast, K6 Height
-- Pattern seed, T7 Outline, T8 Color Mode, T9 View Angle, Slider Brightness.
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

architecture ziggurat of program_top is

    constant C_CHROMA_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_MAX_VAL    : unsigned(9 downto 0) := to_unsigned(1023, 10);
    constant C_OUTLINE_LUMA : unsigned(9 downto 0) := to_unsigned(40, 10);
    constant C_BG_LUMA      : unsigned(9 downto 0) := to_unsigned(16, 10);
    constant C_LUMA_THR     : unsigned(9 downto 0) := to_unsigned(80, 10);  -- "luma positive"

    constant KF      : integer := 6;            -- march depth (boxes up to ~KF cells tall)
    constant NSTAGE  : integer := KF + 1;       -- cells tested per pixel: iv+KF .. iv
    constant C_W     : integer := 19;           -- iso-coord / accumulator width
    subtype  t_w  is signed(C_W - 1 downto 0);
    constant C_IW : integer := 10;              -- cell index width
    subtype  t_i  is signed(C_IW - 1 downto 0);

    constant F_TOP  : std_logic_vector(1 downto 0) := "00";
    constant F_LEFT : std_logic_vector(1 downto 0) := "01";
    constant F_RIGHT: std_logic_vector(1 downto 0) := "10";
    constant F_LINE : std_logic_vector(1 downto 0) := "11";

    --------------------------------------------------------------------------
    -- Parameters
    --------------------------------------------------------------------------
    signal s_base_hue  : unsigned(9 downto 0);
    signal s_box_scale : unsigned(9 downto 0);
    signal s_hspread   : unsigned(9 downto 0);          -- K3: height spread
    signal s_shade     : unsigned(9 downto 0);
    signal s_hpat_pot  : unsigned(9 downto 0);
    signal s_outline_en: std_logic;
    signal s_color_var : std_logic;
    signal s_view_shlw : std_logic;
    signal s_vid_map   : std_logic;                     -- T10: paint video on faces
    signal s_brightness: unsigned(9 downto 0);

    -- incoming video delayed to align with the colour stage; per-face shade deltas
    type t_vdel is array(0 to 6) of std_logic_vector(9 downto 0);
    signal s_vy, s_vu, s_vv : t_vdel := (others => (others => '0'));
    signal s_left_d, s_right_d : unsigned(9 downto 0) := (others => '0');

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
    signal s_a, s_b   : t_w := to_signed(40, C_W);
    signal s_W2       : t_w := to_signed(1600, C_W);     -- 2*a*b
    signal s_W2KF     : t_w := to_signed(9600, C_W);     -- W2*KF (march span)
    signal s_abL1, s_abL2, s_abL3 : t_w := (others => '0');  -- building heights (1/3/6 cells)
    signal s_thr      : t_w := to_signed(40, C_W);
    signal s_top_y, s_left_y, s_right_y : unsigned(9 downto 0) := (others => '0');
    signal s_hseed    : unsigned(3 downto 0) := (others => '0');

    -- one frame multiply: ab = a*b  (registered both ends; updated at vsync)
    signal s_ab_a, s_ab_b : signed(11 downto 0) := (others => '0');
    signal s_ab : signed(23 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Per-line / per-pixel accumulators
    --------------------------------------------------------------------------
    signal s_Vline, s_Uline : t_w := (others => '0');
    signal s_base   : t_i := (others => '0');
    signal s_baserem: t_w := (others => '0');
    signal s_V, s_U : t_w := (others => '0');
    signal s_iv, s_ju : t_i := (others => '0');
    signal s_ivrem, s_jurem : t_w := (others => '0');

    --------------------------------------------------------------------------
    -- Parallel march: stage A computes every candidate cell's cover thresholds
    -- (hashes run in parallel, off the critical path); stage B does all the
    -- cover compares and picks the nearest hit.  Both stages are shallow.
    --------------------------------------------------------------------------
    type t_warrK is array(0 to KF) of t_w;
    type t_iarrK is array(0 to KF) of t_i;
    -- stage A outputs (one entry per marched cell, s=0 = frontmost = iv+KF)
    signal a_Vlo, a_Uhi : t_warrK := (others => (others => '0'));  -- V>=Vlo, U<=Uhi
    signal a_i, a_j     : t_iarrK := (others => (others => '0'));
    signal a_U, a_V     : t_w := (others => '0');
    signal a_sync       : std_logic_vector(3 downto 0) := (others => '0');
    -- stage B1 outputs (cover bits computed; arrays carried for the winner mux)
    signal c_cov        : std_logic_vector(0 to KF) := (others => '0');
    signal c_Vlo, c_Uhi : t_warrK := (others => (others => '0'));
    signal c_i, c_j     : t_iarrK := (others => (others => '0'));
    signal c_U, c_V     : t_w := (others => '0');
    signal c_sync       : std_logic_vector(3 downto 0) := (others => '0');
    -- stage B2 outputs (winner; Vlo/Uhi give corner C: U_C=Uhi-W2, V_C=Vlo+W2)
    signal b_vlo, b_uhi, b_U, b_V : t_w := (others => '0');
    signal b_i, b_j     : t_i := (others => '0');
    signal b_sync       : std_logic_vector(3 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Output
    --------------------------------------------------------------------------
    -- p_color stage 1 outputs
    signal s_up, s_vp, s_upvp : t_w := (others => '0');
    signal s_hue1   : unsigned(9 downto 0) := (others => '0');
    signal s_bi1, s_bj1 : t_i := (others => '0');   -- winning cell carried for video-hold
    signal s_sync1  : std_logic_vector(3 downto 0) := (others => '0');
    -- p_color stage 2a outputs (face/outline condition bits)
    signal s_line2, s_top2, s_right2 : std_logic := '0';
    signal s_hue2  : unsigned(9 downto 0) := (others => '0');
    signal s_bi2, s_bj2 : t_i := (others => '0');
    signal s_sync2 : std_logic_vector(3 downto 0) := (others => '0');
    -- per-building held video colour (so each building is ONE solid colour)
    signal s_prev_bldg : std_logic_vector(15 downto 0) := (others => '1');
    signal s_hy, s_hu, s_hv : unsigned(9 downto 0) := (others => '0');
    -- p_color stage 2b outputs
    signal s_o_y : unsigned(9 downto 0) := (others => '0');
    signal s_o_u : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_o_v : unsigned(9 downto 0) := C_CHROMA_MID;
    signal s_sync_c : std_logic_vector(3 downto 0) := (others => '0');
    signal s_io : t_video_stream_yuv444_30b;

    -- per-cell height level -> aH.  Heights are COARSE (one height per building of
    -- 2^BSHIFT cells) so each building reads as a flat-topped box with tall sides at
    -- its edges, rather than per-cell noise.  Shift/xor only (no multiply).
    constant BSHIFT : integer := 1;               -- 2-cell buildings
    function cell_level(ci, cj : t_i; seed : unsigned(3 downto 0)) return unsigned is
        variable vi, vj : t_i;
        variable h : signed(C_IW + 3 downto 0);
    begin
        vi := shift_right(ci, BSHIFT);            -- coarse building coords
        vj := shift_right(cj, BSHIFT);
        h := resize(shift_left(vi, 2) + vi, C_IW + 4)     -- vi*5
           xor resize(shift_left(vj, 1) + vj, C_IW + 4)   -- vj*3
           xor to_signed(to_integer(seed), C_IW + 4);
        return unsigned(std_logic_vector(h(3 downto 2)));
    end function;

begin

    s_base_hue   <= unsigned(registers_in(0));
    s_box_scale  <= unsigned(registers_in(1));
    s_hspread    <= unsigned(registers_in(2));          -- K3
    s_shade      <= unsigned(registers_in(3));
    s_hpat_pot   <= unsigned(registers_in(5));
    s_outline_en <= registers_in(6)(0);
    s_color_var  <= registers_in(6)(1);
    s_view_shlw  <= registers_in(6)(2);
    s_vid_map    <= registers_in(6)(3);                 -- T10
    s_brightness <= unsigned(registers_in(7));

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
    -- One frame multiply: ab = a*b (registered in/out).
    --------------------------------------------------------------------------
    p_abmul : process(clk)
    begin
        if rising_edge(clk) then
            s_ab_a <= resize(s_a, 12);
            s_ab_b <= resize(s_b, 12);
            s_ab   <= resize(s_ab_a * s_ab_b, 24);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Per-frame constants (latched at vsync).
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_a, v_b : t_w;
        variable v_mid, v_d : signed(11 downto 0);
        variable v_ab, v_abh : t_w;
        variable v_ext : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            if s_vsync_pulse = '1' then
                v_a := signed(resize(shift_right(s_measured_h, 6)
                                   + shift_right(s_box_scale, 5), C_W));   -- cell half-width
                -- Upper half of the knob adds extra reach so the top of the range
                -- is ~2x the cell size of the old max (lower half unchanged).
                if s_box_scale > 512 then
                    v_ext := s_box_scale - to_unsigned(512, 10);          -- 0..511
                    v_a := v_a + signed(resize(shift_right(v_ext, 4)
                                             + shift_right(v_ext, 5), C_W));
                end if;
                if s_view_shlw = '1' then v_b := shift_right(v_a, 2) + shift_right(v_a, 4);
                else                      v_b := shift_right(v_a, 1); end if;
                s_a <= v_a;
                s_b <= v_b;
                s_thr <= shift_right(v_a, 1) + to_signed(1, C_W);

                v_ab := resize(s_ab, C_W);             -- a*b from p_abmul (prev frame's a,b ~ stable)
                s_W2   <= shift_left(v_ab, 1);                          -- cell = 2ab
                s_W2KF <= shift_left(v_ab, 2) + shift_left(v_ab, 3);    -- 2ab*KF = 12ab (KF=6)
                -- K3 Height Spread scales the height unit (flat -> tall city); the
                -- three building levels are 1 / 3 / 6 cells of this unit.
                if    s_hspread < 341 then v_abh := shift_right(v_ab, 2);  -- flat
                elsif s_hspread < 682 then v_abh := shift_right(v_ab, 1);  -- medium
                else                       v_abh := v_ab;                  -- tall
                end if;
                s_abL1 <= shift_left(v_abh, 1);                         -- 1 cell  (2*unit)
                s_abL2 <= shift_left(v_abh, 1) + shift_left(v_abh, 2);  -- 3 cells (6*unit)
                s_abL3 <= shift_left(v_abh, 2) + shift_left(v_abh, 3);  -- 6 cells (12*unit)

                v_mid := to_signed(512, 12);
                v_d   := signed(resize(shift_right(s_shade, 2), 12));
                -- top brightest, left mid, right darkest (deep side shadows)
                s_top_y   <= unsigned(resize(v_mid + v_d, 10));
                s_left_y  <= unsigned(resize(v_mid - shift_right(v_d, 1), 10));
                s_right_y <= unsigned(resize(v_mid - v_d - shift_right(v_d, 1), 10));
                -- how much darker the side faces are vs the top (applied to video too)
                s_left_d  <= unsigned(resize(v_d + shift_right(v_d, 1), 10));        -- 1.5d
                s_right_d <= unsigned(resize(shift_left(v_d, 1) + shift_right(v_d, 1), 10)); -- 2.5d

                s_hseed <= s_hpat_pot(9 downto 6);
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Iso-coord accumulators.  Origin at top-left (sxr=px, syr=py).
    --------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_ivrem, v_jurem : t_w;
    begin
        if rising_edge(clk) then
            if s_timing.vsync_start = '1' then
                s_Vline <= (others => '0'); s_Uline <= (others => '0');
                s_base  <= (others => '0'); s_baserem <= (others => '0');
                s_V <= (others => '0'); s_U <= (others => '0');
                s_iv <= (others => '0'); s_ju <= (others => '0');
                s_ivrem <= (others => '0'); s_jurem <= (others => '0');
            elsif s_timing.avid_start = '1' then
                -- start of a new active line: load line-start values, advance line
                s_V <= s_Vline; s_U <= s_Uline;
                s_iv <= s_base; s_ju <= s_base;
                s_ivrem <= s_baserem; s_jurem <= s_baserem;
                s_Vline <= s_Vline + s_a;
                s_Uline <= s_Uline - s_a;
                v_jurem := s_baserem + s_a;       -- next line base (V_line += a)
                if v_jurem >= s_W2 then s_base <= s_base + 1; s_baserem <= v_jurem - s_W2;
                else                    s_baserem <= v_jurem; end if;
            elsif s_timing.avid = '1' then
                s_V <= s_V + s_b; s_U <= s_U + s_b;
                -- iv = floor(V/W2): V += b
                v_ivrem := s_ivrem + s_b;
                if v_ivrem >= s_W2 then s_iv <= s_iv + 1; s_ivrem <= v_ivrem - s_W2;
                else                    s_ivrem <= v_ivrem; end if;
                -- ju = floor(-U/W2): -U -= b  (U += b)
                if s_jurem >= s_b then s_jurem <= s_jurem - s_b;
                else                   s_ju <= s_ju - 1; s_jurem <= s_jurem + s_W2 - s_b; end if;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage A: cover thresholds for every candidate cell iv+KF .. iv, in
    -- parallel.  Each cell's aH (hash + level mux) feeds only its own threshold
    -- subtract, so the KF+1 hashes run side-by-side (not in a chain).
    --------------------------------------------------------------------------
    p_marchA : process(clk)
        variable v_w2i0, v_w2j0, v_aH, v_sw2 : t_w;
        impure function aH_of(ci, cj : t_i) return t_w is
            variable v : t_w;
        begin
            case cell_level(ci, cj, s_hseed) is           -- flat / 1 / 3 / 6 cells tall
                when "00" => v := (others => '0');
                when "01" => v := s_abL1;
                when "10" => v := s_abL2;
                when others => v := s_abL3;
            end case;
            return v;
        end function;
    begin
        if rising_edge(clk) then
            v_w2i0 := (s_V - s_ivrem) + s_W2KF;           -- W2*(iv+KF)
            v_w2j0 := ((-s_U) - s_jurem) + s_W2KF;        -- W2*(ju+KF)
            for s in 0 to KF loop                          -- s=0 frontmost (iv+KF)
                -- s*W2 as an independent constant multiple (no running chain)
                case s is
                    when 0 => v_sw2 := (others => '0');
                    when 1 => v_sw2 := s_W2;
                    when 2 => v_sw2 := shift_left(s_W2, 1);
                    when 3 => v_sw2 := s_W2 + shift_left(s_W2, 1);
                    when 4 => v_sw2 := shift_left(s_W2, 2);
                    when 5 => v_sw2 := shift_left(s_W2, 2) + s_W2;
                    when others => v_sw2 := shift_left(s_W2, 1) + shift_left(s_W2, 2);
                end case;
                v_aH := aH_of(s_iv + (KF - s), s_ju + (KF - s));
                a_Vlo(s) <= (v_w2i0 - v_sw2) - v_aH;        -- cover: V >= W2i - aH
                a_Uhi(s) <= v_aH - (v_w2j0 - v_sw2);        -- cover: U <= aH - W2j
                a_i(s)   <= s_iv + (KF - s);
                a_j(s)   <= s_ju + (KF - s);
            end loop;
            a_U <= s_U;
            a_V <= s_V;
            a_sync <= data_in.field_n & data_in.avid &
                      data_in.vsync_n & data_in.hsync_n;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage B1: all cover compares in parallel -> a cover bit per cell.
    --------------------------------------------------------------------------
    p_marchB1 : process(clk)
    begin
        if rising_edge(clk) then
            for s in 0 to KF loop
                if (a_V >= a_Vlo(s)) and (a_U <= a_Uhi(s)) then
                    c_cov(s) <= '1'; else c_cov(s) <= '0'; end if;
            end loop;
            c_Vlo <= a_Vlo; c_Uhi <= a_Uhi;
            c_i <= a_i;     c_j <= a_j;
            c_U <= a_U;     c_V <= a_V;
            c_sync <= a_sync;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage B2: pick the nearest (smallest s) covered cell from the cover bits
    -- (priority over precomputed bits -> no compares here).  Back cell always
    -- covers, so there is always a winner (screen filled).
    --------------------------------------------------------------------------
    p_marchB2 : process(clk)
        variable v_found : std_logic;
        variable v_vlo, v_uhi : t_w;
        variable v_i, v_j : t_i;
    begin
        if rising_edge(clk) then
            v_found := '0';
            v_vlo := c_Vlo(KF); v_uhi := c_Uhi(KF);        -- default = back cell
            v_i := c_i(KF);     v_j := c_j(KF);
            for s in 0 to KF loop                          -- front -> back, first cover wins
                if (v_found = '0') and (c_cov(s) = '1') then
                    v_found := '1';
                    v_vlo := c_Vlo(s); v_uhi := c_Uhi(s);
                    v_i := c_i(s);     v_j := c_j(s);
                end if;
            end loop;
            b_vlo <= v_vlo; b_uhi <= v_uhi;
            b_i <= v_i;     b_j <= v_j;
            b_U <= c_U;     b_V <= c_V;
            b_sync <= c_sync;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Classify the winning box + colour.
    --------------------------------------------------------------------------
    -- Stage 1: u' = U - U_C, v' = V - V_C  (U_C = Uhi - W2, V_C = Vlo + W2),
    -- and u'+v' (the W2 cancels) + per-cell hue.  Cheap 2-term adds.
    p_color1 : process(clk)
        variable v_du, v_dv : t_w;
        variable v_hash : signed(C_IW + 3 downto 0);
    begin
        if rising_edge(clk) then
            v_du := b_U - b_uhi;
            v_dv := b_V - b_vlo;
            s_up   <= v_du + s_W2;
            s_vp   <= v_dv - s_W2;
            s_upvp <= v_du + v_dv;

            v_hash := resize(shift_left(b_i, 2) + b_i, C_IW + 4)
                    xor resize(shift_left(b_j, 3) + b_j, C_IW + 4);
            if s_color_var = '1' then
                s_hue1 <= s_base_hue + shift_left(resize(unsigned(std_logic_vector(v_hash(5 downto 3))), 10), 7);
            else
                s_hue1 <= s_base_hue;
            end if;
            s_bi1 <= b_i; s_bj1 <= b_j;
            s_sync1 <= b_sync;
        end if;
    end process;

    -- Stage 2a: resolve face/outline conditions (parallel compares only).
    p_color2a : process(clk)
        variable v_line : boolean;
    begin
        if rising_edge(clk) then
            v_line := false;
            if s_outline_en = '1' then
                if ((s_vp < s_thr) and (s_vp > -s_thr) and (s_up > 0)) or
                   ((s_up < s_thr) and (s_up > -s_thr) and (s_vp < 0)) or
                   ((s_upvp < s_thr) and (s_upvp > -s_thr) and (s_vp > 0)) then
                    v_line := true;
                end if;
            end if;
            if v_line then s_line2 <= '1'; else s_line2 <= '0'; end if;
            if (s_up > 0) and (s_vp < 0) then s_top2 <= '1'; else s_top2 <= '0'; end if;
            if (s_vp > 0) and (s_upvp > 0) then s_right2 <= '1'; else s_right2 <= '0'; end if;
            s_hue2  <= s_hue1;
            s_bi2 <= s_bi1; s_bj2 <= s_bj1;
            s_sync2 <= s_sync1;
        end if;
    end process;

    -- Incoming video delayed to align with the colour stage (T10 = paint on faces).
    p_vdelay : process(clk)
    begin
        if rising_edge(clk) then
            s_vy(0) <= data_in.y; s_vu(0) <= data_in.u; s_vv(0) <= data_in.v;
            for i in 1 to 6 loop
                s_vy(i) <= s_vy(i - 1);
                s_vu(i) <= s_vu(i - 1);
                s_vv(i) <= s_vv(i - 1);
            end loop;
        end if;
    end process;

    -- Stage 2b: pick face (priority mux) and colour.  When T10 is on, each
    -- LUMA MOD (T10): a building is only shown if the incoming video has positive
    -- luma in it (sampled/held per building); dark video -> the building is hidden.
    p_color2b : process(clk)
        variable v_bldg : std_logic_vector(15 downto 0);
        variable v_y : unsigned(9 downto 0);
        variable vbi, vbj : t_i;
    begin
        if rising_edge(clk) then
            s_sync_c <= s_sync2;

            -- per-building sample/hold of the video luma (same BSHIFT building as
            -- the heights, so the whole building shares one luma value)
            vbi := shift_right(s_bi2, BSHIFT);
            vbj := shift_right(s_bj2, BSHIFT);
            v_bldg := std_logic_vector(vbi(7 downto 0)) & std_logic_vector(vbj(7 downto 0));
            if v_bldg /= s_prev_bldg then          -- new building -> fresh luma sample
                v_y := unsigned(s_vy(6));
            else                                   -- same building -> hold it
                v_y := s_hy;
            end if;
            s_prev_bldg <= v_bldg;
            s_hy <= v_y;

            if (s_vid_map = '1') and (v_y <= C_LUMA_THR) then
                -- luma mod on, no positive luma here -> hide the building
                s_o_y <= C_BG_LUMA; s_o_u <= C_CHROMA_MID; s_o_v <= C_CHROMA_MID;
            elsif s_line2 = '1' then
                s_o_y <= C_OUTLINE_LUMA; s_o_u <= C_CHROMA_MID; s_o_v <= C_CHROMA_MID;
            elsif s_top2 = '1' then
                s_o_y <= s_top_y;   s_o_u <= s_hue2; s_o_v <= C_MAX_VAL - s_hue2;
            elsif s_right2 = '1' then
                s_o_y <= s_right_y; s_o_u <= s_hue2; s_o_v <= C_MAX_VAL - s_hue2;
            else
                s_o_y <= s_left_y;  s_o_u <= s_hue2; s_o_v <= C_MAX_VAL - s_hue2;
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Output.
    --------------------------------------------------------------------------
    p_io : process(clk)
    begin
        if rising_edge(clk) then
            s_io.y       <= std_logic_vector(s_o_y);
            s_io.u       <= std_logic_vector(s_o_u);
            s_io.v       <= std_logic_vector(s_o_v);
            s_io.hsync_n <= s_sync_c(0);
            s_io.vsync_n <= s_sync_c(1);
            s_io.avid    <= s_sync_c(2);
            s_io.field_n <= s_sync_c(3);
        end if;
    end process;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture ziggurat;
