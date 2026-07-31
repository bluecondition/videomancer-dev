-- gumball.vhd  (v0.1 — video as a wall of glossy spheres)
--
-- Replaces the incoming video with a hex-packed field of shaded spheres, one
-- per cell, coloured by the live video (a pixelation, but every "pixel" is a
-- lit 3D ball).  Rows are offset by half a cell (hex packing) and the sphere
-- radius can exceed the cell, in which case neighbouring spheres SQUASH flat
-- against each other along the hex Voronoi boundary — round balls with gently
-- flattened contact facets, like pressed bubbles.
--
-- Geometry: two half-offset X lattices (A = even rows, B = odd rows) run in
-- parallel; per pixel the squared distance to the nearest own-row centre and
-- the nearest adjacent-row centre is compared (nearest-of-2 IS the exact hex
-- Voronoi for offset rows), the winner supplies the cell colour and the local
-- (dx, dy) used for shading.  Row height H = W*7/8 (~hex).  No divide and no
-- wide multiply on the pixel path.
--
-- Shading: one offset-light distance field e2 = (dx-ox)^2 + (dy-oy)^2 drives
-- both the diffuse dome (bright at the light spot, falling to ambient at the
-- far rim) and the specular hotspot (tight ramp of the SAME field).  Both are
-- normalised without a pixel divide: per frame a shift sl and an 8-bit gain
-- k are found such that value*2^sl/256*k/128 == value/range, so the pixel path
-- is a 2-stage pipelined barrel shift + one small multiply per field.  The
-- per-frame maths (r, light vector, ranges, two serial divides) runs in a
-- ~85-cycle FSM on ONE shared 12x12 multiplier inside the vsync cone.  The
-- per-LINE squares are updated incrementally at each h edge (adds only), so
-- no line-rate multiplier exists for STA to time at the pixel clock.
--
-- Cell colours are point-sampled once per cell row: row R's colours are
-- written (sat-boosted, plus the optional luma->radius term) into its parity's
-- buffer while the raster scans the CENTRE line of row R-1 — before any pixel
-- of row R's spheres can win the Voronoi test (they first win ~0.16*H later),
-- so the buffer swap is never visible.  Row 0 is sampled on the first active
-- line.  Per-cell radius^2 is stored alongside the colour so Luma Size costs
-- nothing per pixel.
--
-- Controls:
--   K1  Ball Size    (smooth 12..128 px)
--   K2  Light Angle  (0..360 deg — spins the highlight/shadow around each ball)
--   K3  Specular     (highlight intensity, up to a deliberate blowout)
--   K4  Shine        (highlight tightness: broad satin sheen -> pinpoint gloss)
--   K5  Squash       (radius vs cell: separated beads -> touching -> pressed
--                     flat into a full-coverage domed mosaic)
--   K6  Candy        (chroma gain on the sampled colours; 0.75x .. 2.7x)
--   S7  Backdrop     (Black / dimmed live video behind and between the balls)
--   S8  Luma Size    (bright cells grow, dark cells shrink, +/- r/2)
--   S9  Packing      (Hex offset rows / straight square grid)
--   S10 Chrome       (drop chroma: monochrome ball-bearings)
--   S11 Bypass
--   P12 Relief       (THE fader: 0 = flat unlit mosaic tiles, max = deep
--                     shadowed, wet-gloss 3D — shading span and specular
--                     master gain ride it together)
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

architecture gumball of program_top is

    constant LATENCY  : natural := 15;                 -- video pipe 0..14, out at c16
    constant C_MAXCOL : natural := 128;                -- max cell columns (W >= 12 -> <= 107 + guard)

    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- normalising shift: smallest sl with (v << sl) in [2^15, 2^16)
    function norm_shift(v : unsigned(15 downto 0)) return natural is
    begin
        for i in 15 downto 0 loop
            if v(i) = '1' then
                return 15 - i;
            end if;
        end loop;
        return 15;
    end function;

    --------------------------------------------------------------------------
    -- Raster position: two half-offset X lattices + row counter.
    --------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal r_hedge      : std_logic := '0';            -- registered h edge (line maths kick)
    signal r_anchor     : std_logic := '0';            -- first-active-line event

    signal xa_loc, xb_loc : unsigned(7 downto 0) := (others => '0');
    signal xa_idx, xb_idx : unsigned(6 downto 0) := (others => '0');
    signal celly_loc      : unsigned(7 downto 0) := (others => '0');
    signal celly_par      : std_logic := '0';
    signal frame_act      : std_logic := '0';
    signal band0          : std_logic := '0';          -- top band: no row above
    signal line0          : std_logic := '0';          -- first active line (row-0 sample)

    --------------------------------------------------------------------------
    -- Per-frame parameters (latched at vsync, then refined by the frame FSM).
    --------------------------------------------------------------------------
    signal lk1, lk2, lk3, lk4, lk5, lk6, lp12 : unsigned(9 downto 0) := (others => '0');

    signal s_w      : unsigned(7 downto 0) := to_unsigned(48, 8);   -- cell width W
    signal s_wm1    : unsigned(7 downto 0) := to_unsigned(47, 8);
    signal s_wh     : unsigned(7 downto 0) := to_unsigned(24, 8);   -- W/2
    signal s_h      : unsigned(7 downto 0) := to_unsigned(42, 8);   -- row height H
    signal s_hm1    : unsigned(7 downto 0) := to_unsigned(41, 8);
    signal s_hh     : unsigned(7 downto 0) := to_unsigned(21, 8);   -- H/2
    signal s_rfrac  : unsigned(10 downto 0) := to_unsigned(1000, 11);
    signal s_kshine : unsigned(10 downto 0) := to_unsigned(576, 11);
    signal s_r      : unsigned(7 downto 0) := to_unsigned(25, 8);   -- sphere radius
    signal s_rim2   : unsigned(7 downto 0) := to_unsigned(75, 8);   -- ~3r: 1.5 px AA band
    signal s_ls     : unsigned(7 downto 0) := to_unsigned(15, 8);   -- light offset 5r/8
    signal s_es     : unsigned(7 downto 0) := to_unsigned(12, 8);   -- specular radius
    signal s_ox, s_oy : signed(7 downto 0) := (others => '0');      -- light offset vector
    signal s_e2     : unsigned(15 downto 0) := to_unsigned(1600, 16);
    signal s_es2    : unsigned(15 downto 0) := to_unsigned(144, 16);
    signal s_sle    : unsigned(3 downto 0) := (others => '0');      -- diffuse norm shift
    signal s_sls    : unsigned(3 downto 0) := (others => '0');      -- spec norm shift
    signal s_kd8    : unsigned(7 downto 0) := (others => '0');      -- diffuse norm gain
    signal s_ks9    : unsigned(9 downto 0) := (others => '0');      -- spec norm gain
    signal s_sp     : unsigned(7 downto 0) := (others => '0');      -- shading span (P12)
    signal s_amb    : unsigned(7 downto 0) := to_unsigned(255, 8);  -- ambient = 255 - span
    signal s_si     : unsigned(8 downto 0) := (others => '0');      -- spec intensity (K3)
    signal s_speff  : unsigned(9 downto 0) := (others => '0');      -- spec * relief master
    signal s_sg     : unsigned(7 downto 0) := to_unsigned(64, 8);   -- sat gain (64 = unity)

    signal s_bgvid  : std_logic := '0';                -- S7
    signal s_lsize  : std_logic := '0';                -- S8
    signal s_square : std_logic := '0';                -- S9
    signal s_mono   : std_logic := '0';                -- S10
    signal s_bypass : std_logic := '0';                -- S11

    -- sine LUT (per-frame light angle)
    signal lut_sin, lut_cos : signed(9 downto 0);
    signal sinv, cosv       : signed(9 downto 0) := (others => '0');

    -- frame FSM shared multiplier + serial divider
    signal fsm_t   : unsigned(7 downto 0) := (others => '1');       -- FF = idle
    signal ma, mb  : signed(11 downto 0) := (others => '0');
    signal mp      : signed(23 downto 0) := (others => '0');
    signal div_run : std_logic := '0';
    signal div_cnt : unsigned(4 downto 0) := (others => '0');
    signal div_d   : unsigned(17 downto 0) := (others => '0');      -- dividend (shifting)
    signal div_q   : unsigned(17 downto 0) := (others => '0');      -- quotient
    signal div_rem : unsigned(8 downto 0)  := (others => '0');
    signal div_v   : unsigned(7 downto 0)  := (others => '1');      -- divisor (128..255)

    --------------------------------------------------------------------------
    -- Per-line constants (dy / light-relative dy, squared).  dy advances by
    -- exactly +1 per line, so the squares are updated INCREMENTALLY at each
    -- h edge — (d+1)^2 = d^2 + 2(d+1) - 1, adds only, no multiplier for STA
    -- to time — with the band-wrap / mid-band-jump reset values precomputed
    -- per frame by the FSM (hh2q / hsq / eyw*sq / eyj2sq).
    --------------------------------------------------------------------------
    signal lm_dy1   : signed(8 downto 0) := (others => '0');
    signal lm_dy2   : signed(8 downto 0) := (others => '0');
    signal lm_ey1   : signed(8 downto 0) := (others => '0');
    signal lm_ey2   : signed(8 downto 0) := (others => '0');
    signal dy1sq_r  : unsigned(15 downto 0) := (others => '0');
    signal dy2sq_r  : unsigned(15 downto 0) := (others => '0');
    signal ey1sq_r  : unsigned(15 downto 0) := (others => '0');
    signal ey2sq_r  : unsigned(15 downto 0) := (others => '0');
    signal hh2q     : unsigned(15 downto 0) := (others => '0');     -- (H/2)^2
    signal hsq      : unsigned(15 downto 0) := (others => '0');     -- H^2
    signal eyw1sq   : unsigned(15 downto 0) := (others => '0');     -- (H/2 + oy)^2
    signal eyw2sq   : unsigned(15 downto 0) := (others => '0');     -- (H/2 - oy)^2
    signal eyj2sq   : unsigned(15 downto 0) := (others => '0');     -- (H + oy)^2

    --------------------------------------------------------------------------
    -- Cell buffers: one per row parity, word = Y10 & U10 & V10 & rsq16.
    -- 1W1R, registered read (BRAM).
    --------------------------------------------------------------------------
    subtype t_cword is std_logic_vector(45 downto 0);
    type t_cbuf is array(0 to C_MAXCOL - 1) of t_cword;
    signal buf0, buf1 : t_cbuf := (others => (others => '0'));
    signal rd0, rd1   : t_cword := (others => '0');

    -- sample/write pipe (W0..W3 -> write)
    signal w0_stb : std_logic := '0';
    signal w0_par : std_logic := '0';
    signal w0_idx : unsigned(6 downto 0) := (others => '0');
    signal w0_y, w0_u, w0_v : unsigned(9 downto 0) := C_MID;
    signal w1_stb : std_logic := '0';
    signal w1_par : std_logic := '0';
    signal w1_idx : unsigned(6 downto 0) := (others => '0');
    signal w1_y   : unsigned(9 downto 0) := C_MID;
    signal w1_pu, w1_pv : signed(19 downto 0) := (others => '0');
    signal w1_pl  : signed(19 downto 0) := (others => '0');
    signal w2_stb : std_logic := '0';
    signal w2_par : std_logic := '0';
    signal w2_idx : unsigned(6 downto 0) := (others => '0');
    signal w2_y, w2_u, w2_v : unsigned(9 downto 0) := C_MID;
    signal w2_rr  : unsigned(7 downto 0) := (others => '0');
    signal w3_stb : std_logic := '0';
    signal w3_par : std_logic := '0';
    signal w3_idx : unsigned(6 downto 0) := (others => '0');
    signal w3_y, w3_u, w3_v : unsigned(9 downto 0) := C_MID;
    signal w3_rsq : unsigned(15 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Pixel pipeline (c1..c16).
    --------------------------------------------------------------------------
    signal g0_dx1, g0_dx2 : signed(7 downto 0) := (others => '0');
    signal g0_a0, g0_a1   : unsigned(6 downto 0) := (others => '0');
    signal g0_noadj       : std_logic := '0';

    signal p2_dx1, p2_dx2 : signed(7 downto 0) := (others => '0');
    signal p2_noadj       : std_logic := '0';
    signal q1, q2         : unsigned(13 downto 0) := (others => '0');

    signal p3_dx1, p3_dx2 : signed(7 downto 0) := (others => '0');
    signal p3_rd0, p3_rd1 : t_cword := (others => '0');
    signal d1, d2         : unsigned(15 downto 0) := (others => '0');

    signal p4_dsq  : unsigned(15 downto 0) := (others => '0');
    signal p4_dx   : signed(7 downto 0) := (others => '0');
    signal p4_eys  : unsigned(15 downto 0) := (others => '0');
    signal p4_word : t_cword := (others => '0');

    signal p5_ex   : signed(7 downto 0) := (others => '0');
    signal p5_in   : std_logic := '0';
    signal p5_core : std_logic := '0';
    signal p5_eys  : unsigned(15 downto 0) := (others => '0');
    signal p5_y, p5_u, p5_v : unsigned(9 downto 0) := C_MID;

    signal p6_exq  : unsigned(13 downto 0) := (others => '0');
    signal p6_eys  : unsigned(15 downto 0) := (others => '0');

    signal p7_e2   : unsigned(16 downto 0) := (others => '0');
    signal p8_dl   : unsigned(14 downto 0) := (others => '0');
    signal p8_ds   : unsigned(13 downto 0) := (others => '0');
    signal p9_dlc  : unsigned(15 downto 0) := (others => '0');
    signal p9_dsc  : unsigned(15 downto 0) := (others => '0');
    signal p10_dlb : unsigned(7 downto 0) := (others => '0');
    signal p10_dsb : unsigned(7 downto 0) := (others => '0');
    signal p11_pl  : unsigned(15 downto 0) := (others => '0');
    signal p11_ps  : unsigned(17 downto 0) := (others => '0');
    signal p12_l   : unsigned(7 downto 0) := (others => '0');
    signal p12_s   : unsigned(8 downto 0) := (others => '0');
    signal p13_ym  : unsigned(17 downto 0) := (others => '0');
    signal p13_cf  : unsigned(7 downto 0) := (others => '0');
    signal p13_s   : unsigned(8 downto 0) := (others => '0');
    signal p14_um, p14_vm : signed(19 downto 0) := (others => '0');
    signal p14_y   : unsigned(9 downto 0) := (others => '0');
    signal p15_y, p15_u, p15_v : unsigned(9 downto 0) := C_MID;

    -- flag / colour pipes
    signal in_p   : std_logic_vector(0 to 8) := (others => '0');
    signal core_p : std_logic_vector(0 to 8) := (others => '0');
    type t_c10p is array (natural range <>) of unsigned(9 downto 0);
    signal yp : t_c10p(0 to 7) := (others => C_MID);
    signal up : t_c10p(0 to 7) := (others => C_MID);
    signal vp : t_c10p(0 to 7) := (others => C_MID);

    --------------------------------------------------------------------------
    -- Sync / video alignment pipe
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal s_io : t_video_stream_yuv444_30b;

begin

    u_sincos : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(lk2),
            sin_out  => lut_sin,
            cos_out  => lut_cos
        );

    --------------------------------------------------------------------------
    -- Raster position counters + sample trigger.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_tp   : std_logic;
        variable v_trig : std_logic;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            r_hedge  <= v_h_edge;
            r_anchor <= '0';

            -- X lattices: A aligned, B half-offset (or aligned too in square mode).
            if v_h_edge = '1' then
                xa_loc <= (others => '0');
                xa_idx <= (others => '0');
                if s_square = '1' then
                    xb_loc <= (others => '0');
                else
                    xb_loc <= s_wh;
                end if;
                xb_idx <= (others => '0');
            elsif data_in.avid = '1' then
                if xa_loc = s_wm1 then
                    xa_loc <= (others => '0');
                    xa_idx <= xa_idx + 1;
                else
                    xa_loc <= xa_loc + 1;
                end if;
                if xb_loc = s_wm1 then
                    xb_loc <= (others => '0');
                    xb_idx <= xb_idx + 1;
                else
                    xb_loc <= xb_loc + 1;
                end if;
            end if;

            -- Y row counter, anchored to the first active line of the frame.
            if v_v_edge = '1' then
                celly_loc <= (others => '0');
                celly_par <= '0';
                frame_act <= '0';
                band0     <= '1';
            elsif data_in.avid = '1' and frame_act = '0' then
                frame_act <= '1';
                celly_loc <= (others => '0');
                celly_par <= '0';
                band0     <= '1';
                line0     <= '1';
                r_anchor  <= '1';
            elsif v_h_edge = '1' then
                line0 <= '0';
                if celly_loc = s_hm1 then
                    celly_loc <= (others => '0');
                    celly_par <= not celly_par;
                    band0     <= '0';
                else
                    celly_loc <= celly_loc + 1;
                end if;
            end if;

            -- Sample trigger: row R's colours are grabbed on the centre line of
            -- row R-1 (target parity = not current), at the target lattice's
            -- cell centres; row 0 is grabbed on the first active line.
            v_trig := '0';
            v_tp   := not celly_par;
            if data_in.avid = '1' and frame_act = '1' then
                if line0 = '1' then
                    v_tp := '0';
                    if xa_loc = s_wh then v_trig := '1'; end if;
                elsif celly_loc = s_hh then
                    if v_tp = '0' then
                        if xa_loc = s_wh then v_trig := '1'; end if;
                    else
                        if xb_loc = s_wh then v_trig := '1'; end if;
                    end if;
                end if;
            end if;

            w0_stb <= v_trig;
            w0_par <= v_tp;
            if v_tp = '0' then
                w0_idx <= xa_idx;
            else
                w0_idx <= xb_idx;
            end if;
            w0_y <= unsigned(data_in.y);
            w0_u <= unsigned(data_in.u);
            w0_v <= unsigned(data_in.v);
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Sample write pipe: sat boost + optional luma->radius, then radius^2.
    --------------------------------------------------------------------------
    p_sample : process(clk)
        variable v_u, v_vv : signed(11 downto 0);
        variable v_rr      : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- W1: chroma gain + luma*r products (dedicated small mults)
            w1_pu  <= (signed(resize(w0_u, 11)) - to_signed(512, 11))
                    * signed(resize(s_sg, 9));
            w1_pv  <= (signed(resize(w0_v, 11)) - to_signed(512, 11))
                    * signed(resize(s_sg, 9));
            w1_pl  <= (signed(resize(w0_y, 11)) - to_signed(512, 11))
                    * signed(resize(s_r, 9));
            w1_stb <= w0_stb;
            w1_par <= w0_par;
            w1_idx <= w0_idx;
            w1_y   <= w0_y;

            -- W2: recombine + clamp
            v_u := to_signed(512, 12) + resize(shift_right(w1_pu, 6), 12);
            if v_u < 0 then v_u := (others => '0'); end if;
            if v_u > 1023 then v_u := to_signed(1023, 12); end if;
            v_vv := to_signed(512, 12) + resize(shift_right(w1_pv, 6), 12);
            if v_vv < 0 then v_vv := (others => '0'); end if;
            if v_vv > 1023 then v_vv := to_signed(1023, 12); end if;

            if s_lsize = '1' then
                v_rr := signed(resize(s_r, 12)) + resize(shift_right(w1_pl, 10), 12);
                if v_rr < 3 then v_rr := to_signed(3, 12); end if;
                if v_rr > 180 then v_rr := to_signed(180, 12); end if;
            else
                v_rr := signed(resize(s_r, 12));
            end if;

            w2_u   <= unsigned(v_u(9 downto 0));
            w2_v   <= unsigned(v_vv(9 downto 0));
            w2_rr  <= unsigned(v_rr(7 downto 0));
            w2_stb <= w1_stb;
            w2_par <= w1_par;
            w2_idx <= w1_idx;
            w2_y   <= w1_y;

            -- W3: radius^2 (dedicated squarer)
            w3_rsq <= resize(w2_rr * w2_rr, 16);
            w3_stb <= w2_stb;
            w3_par <= w2_par;
            w3_idx <= w2_idx;
            w3_y   <= w2_y;
            w3_u   <= w2_u;
            w3_v   <= w2_v;
        end if;
    end process p_sample;

    --------------------------------------------------------------------------
    -- Cell buffers (canonical 1W1R, registered read, pre-muxed write word).
    --------------------------------------------------------------------------
    p_buf0 : process(clk)
    begin
        if rising_edge(clk) then
            if w3_stb = '1' and w3_par = '0' then
                buf0(to_integer(w3_idx)) <= std_logic_vector(w3_y) & std_logic_vector(w3_u)
                                          & std_logic_vector(w3_v) & std_logic_vector(w3_rsq);
            end if;
            rd0 <= buf0(to_integer(g0_a0));
        end if;
    end process p_buf0;

    p_buf1 : process(clk)
    begin
        if rising_edge(clk) then
            if w3_stb = '1' and w3_par = '1' then
                buf1(to_integer(w3_idx)) <= std_logic_vector(w3_y) & std_logic_vector(w3_u)
                                          & std_logic_vector(w3_v) & std_logic_vector(w3_rsq);
            end if;
            rd1 <= buf1(to_integer(g0_a1));
        end if;
    end process p_buf1;

    --------------------------------------------------------------------------
    -- Per-line maths: dy / (dy - oy) squared for both row candidates,
    -- updated incrementally (adds only) one cycle after each h edge.
    --------------------------------------------------------------------------
    p_line : process(clk)
        -- sq((d+1)) from sq(d) and the NEW d: sq + 2d_new - 1
        function upd(sq : unsigned(15 downto 0); dn : signed(8 downto 0))
            return unsigned is
            variable v : signed(17 downto 0);
        begin
            v := signed(resize(sq, 18)) + resize(dn & '0', 18) - 1;
            return unsigned(v(15 downto 0));
        end function;
        variable v_hh, v_h : signed(8 downto 0);
    begin
        if rising_edge(clk) then
            if r_hedge = '1' or r_anchor = '1' then
                v_hh := signed(resize(s_hh, 9));
                v_h  := signed(resize(s_h, 9));
                if celly_loc = 0 then
                    -- band wrap (or frame anchor)
                    lm_dy1  <= -v_hh;
                    lm_dy2  <= v_hh;
                    lm_ey1  <= -v_hh - resize(s_oy, 9);
                    lm_ey2  <= v_hh - resize(s_oy, 9);
                    dy1sq_r <= hh2q;
                    dy2sq_r <= hh2q;
                    ey1sq_r <= eyw1sq;
                    ey2sq_r <= eyw2sq;
                elsif celly_loc = s_hh then
                    -- adjacent row flips from above to below: dy2 jumps to -H
                    lm_dy1  <= lm_dy1 + 1;
                    dy1sq_r <= upd(dy1sq_r, lm_dy1 + 1);
                    lm_ey1  <= lm_ey1 + 1;
                    ey1sq_r <= upd(ey1sq_r, lm_ey1 + 1);
                    lm_dy2  <= -v_h;
                    dy2sq_r <= hsq;
                    lm_ey2  <= -v_h - resize(s_oy, 9);
                    ey2sq_r <= eyj2sq;
                else
                    lm_dy1  <= lm_dy1 + 1;
                    dy1sq_r <= upd(dy1sq_r, lm_dy1 + 1);
                    lm_ey1  <= lm_ey1 + 1;
                    ey1sq_r <= upd(ey1sq_r, lm_ey1 + 1);
                    lm_dy2  <= lm_dy2 + 1;
                    dy2sq_r <= upd(dy2sq_r, lm_dy2 + 1);
                    lm_ey2  <= lm_ey2 + 1;
                    ey2sq_r <= upd(ey2sq_r, lm_ey2 + 1);
                end if;
            end if;
        end if;
    end process p_line;

    --------------------------------------------------------------------------
    -- Frame FSM: knob shaping, light vector, ranges, 2 serial divides.
    -- One shared 14x14 multiplier; everything settles inside the vsync cone.
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_w   : integer range 0 to 255;
        variable v_e   : unsigned(8 downto 0);
        variable v_acc : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            mp <= ma * mb;

            -- serial restoring divider (18-bit dividend / 8-bit divisor)
            if div_run = '1' then
                v_acc := (div_rem & div_d(17)) - resize(div_v, 10);
                if v_acc(9) = '0' then
                    div_rem <= v_acc(8 downto 0);
                    div_q   <= div_q(16 downto 0) & '1';
                else
                    div_rem <= div_rem(7 downto 0) & div_d(17);
                    div_q   <= div_q(16 downto 0) & '0';
                end if;
                div_d <= div_d(16 downto 0) & '0';
                if div_cnt = 17 then
                    div_run <= '0';
                else
                    div_cnt <= div_cnt + 1;
                end if;
            end if;

            if prev_vsync_n = '1' and data_in.vsync_n = '0' then
                lk1  <= unsigned(registers_in(0)(9 downto 0));
                lk2  <= unsigned(registers_in(1)(9 downto 0));
                lk3  <= unsigned(registers_in(2)(9 downto 0));
                lk4  <= unsigned(registers_in(3)(9 downto 0));
                lk5  <= unsigned(registers_in(4)(9 downto 0));
                lk6  <= unsigned(registers_in(5)(9 downto 0));
                lp12 <= unsigned(registers_in(7)(9 downto 0));
                s_bgvid  <= registers_in(6)(0);
                s_lsize  <= registers_in(6)(1);
                s_square <= registers_in(6)(2);
                s_mono   <= registers_in(6)(3);
                s_bypass <= registers_in(6)(4);
                fsm_t <= (others => '0');
            elsif fsm_t /= x"FF" then
                fsm_t <= fsm_t + 1;

                case to_integer(fsm_t) is
                    when 0 =>
                        v_w := 12 + to_integer(lk1(9 downto 3));
                        if v_w > 128 then v_w := 128; end if;
                        s_w      <= to_unsigned(v_w, 8);
                        s_sp     <= lp12(9 downto 2);
                        s_amb    <= 255 - lp12(9 downto 2);
                        s_si     <= lk3(9 downto 1);
                        s_sg     <= resize(to_unsigned(48, 8) + lk6(9 downto 3), 8);
                        s_rfrac  <= resize(to_unsigned(676, 11) + lk5(9 downto 1)
                                         + lk5(9 downto 3), 11);
                        s_kshine <= to_unsigned(1088, 11) - resize(lk4, 11);
                    when 1 =>
                        s_wm1 <= s_w - 1;
                        s_wh  <= '0' & s_w(7 downto 1);
                        if s_square = '1' then
                            s_h   <= s_w;
                        else
                            s_h   <= s_w - ('0' & s_w(7 downto 3));       -- H = W*7/8
                        end if;
                        ma <= signed(resize(s_w, 12));
                        mb <= signed(resize(s_rfrac, 12));
                    when 2 =>
                        s_hm1 <= s_h - 1;
                        s_hh  <= '0' & s_h(7 downto 1);
                    when 4 =>
                        -- r = W*rfrac/2048, 4..82
                        if mp(18 downto 11) < 4 then
                            s_r <= to_unsigned(4, 8);
                        else
                            s_r <= unsigned(mp(18 downto 11));
                        end if;
                    when 5 =>
                        s_ls   <= ('0' & s_r(7 downto 1)) + ("000" & s_r(7 downto 3));
                        s_rim2 <= resize(s_r + ('0' & s_r(7 downto 1)), 8)
                                + ("00" & s_r(7 downto 2));               -- 1.75r: ~0.9px AA ring
                        ma <= signed(resize(s_r, 12));
                        mb <= signed(resize(s_kshine, 12));
                        sinv <= lut_sin;
                        cosv <= lut_cos;
                    when 8 =>
                        -- Es = 3 + r*(1088-K4)/1024
                        s_es <= resize(to_unsigned(3, 8) + unsigned(mp(17 downto 10)), 8);
                        ma <= signed(resize(s_ls, 12));
                        mb <= resize(cosv, 12);
                    when 11 =>
                        s_ox <= resize(shift_right(mp, 9), 8);
                        ma <= signed(resize(s_ls, 12));
                        mb <= resize(sinv, 12);
                    when 14 =>
                        s_oy <= resize(shift_right(mp, 9), 8);
                        v_e := resize(s_r, 9) + resize(s_ls, 9);
                        ma <= signed(resize(v_e, 12));
                        mb <= signed(resize(v_e, 12));
                    when 17 =>
                        s_e2 <= unsigned(mp(15 downto 0));
                        ma <= signed(resize(s_es, 12));
                        mb <= signed(resize(s_es, 12));
                    when 20 =>
                        s_es2 <= unsigned(mp(15 downto 0));
                        ma <= signed(resize(s_si, 12));
                        mb <= signed(resize(s_sp, 12));
                    when 23 =>
                        s_speff <= unsigned(mp(16 downto 7));
                    when 24 =>
                        s_sle <= to_unsigned(norm_shift(s_e2), 4);
                    when 25 =>
                        div_v   <= resize(shift_right(shift_left(resize(s_e2, 16),
                                          to_integer(s_sle)), 8), 8);
                        div_d   <= resize(s_sp, 18) sll 7;
                        div_q   <= (others => '0');
                        div_rem <= (others => '0');
                        div_cnt <= (others => '0');
                        div_run <= '1';
                    when 45 =>
                        if div_q > 255 then
                            s_kd8 <= (others => '1');
                        else
                            s_kd8 <= div_q(7 downto 0);
                        end if;
                        s_sls <= to_unsigned(norm_shift(s_es2), 4);
                    when 46 =>
                        div_v   <= resize(shift_right(shift_left(resize(s_es2, 16),
                                          to_integer(s_sls)), 8), 8);
                        div_d   <= resize(s_speff, 18) sll 7;
                        div_q   <= (others => '0');
                        div_rem <= (others => '0');
                        div_cnt <= (others => '0');
                        div_run <= '1';
                    when 66 =>
                        if div_q > 1023 then
                            s_ks9 <= (others => '1');
                        else
                            s_ks9 <= div_q(9 downto 0);
                        end if;
                        ma <= signed(resize(s_hh, 12));
                        mb <= signed(resize(s_hh, 12));
                    when 69 =>
                        hh2q <= unsigned(mp(15 downto 0));
                        ma <= signed(resize(s_h, 12));
                        mb <= signed(resize(s_h, 12));
                    when 72 =>
                        hsq <= unsigned(mp(15 downto 0));
                        ma <= signed(resize(s_hh, 12)) + resize(s_oy, 12);
                        mb <= signed(resize(s_hh, 12)) + resize(s_oy, 12);
                    when 75 =>
                        eyw1sq <= unsigned(mp(15 downto 0));
                        ma <= signed(resize(s_hh, 12)) - resize(s_oy, 12);
                        mb <= signed(resize(s_hh, 12)) - resize(s_oy, 12);
                    when 78 =>
                        eyw2sq <= unsigned(mp(15 downto 0));
                        ma <= signed(resize(s_h, 12)) + resize(s_oy, 12);
                        mb <= signed(resize(s_h, 12)) + resize(s_oy, 12);
                    when 81 =>
                        eyj2sq <= unsigned(mp(15 downto 0));
                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    --------------------------------------------------------------------------
    -- Pixel pipeline.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_par    : std_logic;
        variable v_word   : t_cword;
        variable v_rsq    : unsigned(15 downto 0);
        variable v_rin    : signed(17 downto 0);
        variable v_dl     : signed(17 downto 0);
        variable v_ds     : signed(17 downto 0);
        variable v_l      : unsigned(9 downto 0);
        variable v_s      : unsigned(10 downto 0);
        variable v_cf     : signed(10 downto 0);
        variable v_yd     : unsigned(10 downto 0);
        variable v_su     : signed(11 downto 0);
        variable v_sy, v_uu, v_vv : unsigned(9 downto 0);
        variable v_bgy, v_bgu, v_bgv : unsigned(9 downto 0);
        variable v_oy2, v_ou, v_ov : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- sync/video delay line
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            ------------------------------------------------------------------
            -- c1 (G0): local dx for both lattices, buffer addresses, top guard.
            -- Own row = parity's lattice; adjacent row = the other one.
            ------------------------------------------------------------------
            if celly_par = '0' then
                g0_dx1 <= signed(resize(xa_loc, 8)) - signed(resize(s_wh, 8));
                g0_dx2 <= signed(resize(xb_loc, 8)) - signed(resize(s_wh, 8));
            else
                g0_dx1 <= signed(resize(xb_loc, 8)) - signed(resize(s_wh, 8));
                g0_dx2 <= signed(resize(xa_loc, 8)) - signed(resize(s_wh, 8));
            end if;
            g0_a0 <= xa_idx;
            g0_a1 <= xb_idx;
            if band0 = '1' and celly_loc < s_hh then
                g0_noadj <= '1';
            else
                g0_noadj <= '0';
            end if;

            -- c2: dx squares (buffer reads land in rd0/rd1 this cycle too)
            q1 <= resize(unsigned(g0_dx1 * g0_dx1), 14);
            q2 <= resize(unsigned(g0_dx2 * g0_dx2), 14);
            p2_dx1   <= g0_dx1;
            p2_dx2   <= g0_dx2;
            p2_noadj <= g0_noadj;

            -- c3: full squared distances to both candidate centres
            d1 <= resize(q1, 16) + resize(dy1sq_r, 16);
            if p2_noadj = '1' then
                d2 <= (others => '1');
            else
                d2 <= resize(q2, 16) + resize(dy2sq_r, 16);
            end if;
            p3_dx1 <= p2_dx1;
            p3_dx2 <= p2_dx2;
            p3_rd0 <= rd0;
            p3_rd1 <= rd1;

            ------------------------------------------------------------------
            -- c4: Voronoi select.  Winning candidate's parity = row parity xor
            -- sel, which picks the buffer word directly.
            ------------------------------------------------------------------
            if d2 < d1 then
                p4_dsq <= d2;
                p4_dx  <= p3_dx2;
                p4_eys <= ey2sq_r;
                v_par  := not celly_par;
            else
                p4_dsq <= d1;
                p4_dx  <= p3_dx1;
                p4_eys <= ey1sq_r;
                v_par  := celly_par;
            end if;
            if v_par = '1' then
                p4_word <= p3_rd1;
            else
                p4_word <= p3_rd0;
            end if;

            -- c5: inside / core tests + light-relative dx + colour unpack
            v_word := p4_word;
            v_rsq  := unsigned(v_word(15 downto 0));
            v_rin  := signed(resize(v_rsq, 18)) - signed(resize(s_rim2, 18));
            if resize(p4_dsq, 16) < v_rsq then
                p5_in <= '1';
            else
                p5_in <= '0';
            end if;
            if signed(resize(p4_dsq, 18)) < v_rin then
                p5_core <= '1';
            else
                p5_core <= '0';
            end if;
            p5_ex  <= p4_dx - s_ox;
            p5_eys <= p4_eys;
            p5_y   <= unsigned(v_word(45 downto 36));
            p5_u   <= unsigned(v_word(35 downto 26));
            p5_v   <= unsigned(v_word(25 downto 16));

            -- c6: light-relative dx squared
            p6_exq <= resize(unsigned(p5_ex * p5_ex), 14);
            p6_eys <= p5_eys;

            -- c7: light-relative squared distance
            p7_e2 <= resize(p6_exq, 17) + resize(p6_eys, 17);

            -- c8: diffuse / specular head-room (clamped at 0)
            v_dl := signed(resize(s_e2, 18)) - signed(resize(p7_e2, 18));
            if v_dl < 0 then v_dl := (others => '0'); end if;
            p8_dl <= unsigned(v_dl(14 downto 0));
            v_ds := signed(resize(s_es2, 18)) - signed(resize(p7_e2, 18));
            if v_ds < 0 then v_ds := (others => '0'); end if;
            p8_ds <= unsigned(v_ds(13 downto 0));

            -- c9/c10: 2-stage barrel normalise (coarse 0/4/8/12, then fine 0..3)
            p9_dlc <= shift_left(resize(p8_dl, 16),
                                 4 * to_integer(s_sle(3 downto 2)));
            p9_dsc <= shift_left(resize(p8_ds, 16),
                                 4 * to_integer(s_sls(3 downto 2)));
            p10_dlb <= shift_left(p9_dlc, to_integer(s_sle(1 downto 0)))(15 downto 8);
            p10_dsb <= shift_left(p9_dsc, to_integer(s_sls(1 downto 0)))(15 downto 8);

            -- c11: normalise gains
            p11_pl <= p10_dlb * s_kd8;
            p11_ps <= p10_dsb * s_ks9;

            -- c12: diffuse level (ambient floor) + specular level
            v_l := resize(s_amb, 10) + resize(p11_pl(15 downto 7), 10);
            if v_l > 255 then v_l := to_unsigned(255, 10); end if;
            p12_l <= v_l(7 downto 0);
            v_s := resize(p11_ps(17 downto 7), 11);
            if v_s > 511 then v_s := to_unsigned(511, 11); end if;
            p12_s <= v_s(8 downto 0);

            -- c13: Y * diffuse; chroma factor = diffuse minus spec desat
            p13_ym <= yp(6) * p12_l;
            v_cf := signed(resize(p12_l, 11)) - signed(resize(p12_s(8 downto 1), 11));
            if v_cf < 0 then v_cf := (others => '0'); end if;
            p13_cf <= unsigned(v_cf(7 downto 0));
            p13_s  <= p12_s;

            -- c14: chroma * factor; Y + specular
            p14_um <= (signed(resize(up(7), 11)) - to_signed(512, 11))
                    * signed(resize(p13_cf, 9));
            p14_vm <= (signed(resize(vp(7), 11)) - to_signed(512, 11))
                    * signed(resize(p13_cf, 9));
            v_yd := resize(p13_ym(17 downto 8), 11) + resize(p13_s, 11);
            if v_yd > 1023 then v_yd := to_unsigned(1023, 11); end if;
            p14_y <= v_yd(9 downto 0);

            ------------------------------------------------------------------
            -- c15: compose sphere over backdrop (rim row gets a 50% AA blend).
            ------------------------------------------------------------------
            v_sy := p14_y;
            v_su := to_signed(512, 12) + resize(shift_right(p14_um, 8), 12);
            v_uu := unsigned(v_su(9 downto 0));
            v_su := to_signed(512, 12) + resize(shift_right(p14_vm, 8), 12);
            v_vv := unsigned(v_su(9 downto 0));

            if s_bgvid = '1' then
                v_bgy := resize(unsigned(pipe(13).y(9 downto 1)), 10) + 32;
                v_bgu := resize(unsigned(pipe(13).u(9 downto 1)), 10) + 256;
                v_bgv := resize(unsigned(pipe(13).v(9 downto 1)), 10) + 256;
            else
                v_bgy := to_unsigned(64, 10);
                v_bgu := C_MID;
                v_bgv := C_MID;
            end if;

            if in_p(8) = '1' then
                if core_p(8) = '1' then
                    v_oy2 := v_sy;  v_ou := v_uu;  v_ov := v_vv;
                else
                    v_oy2 := ('0' & v_sy(9 downto 1)) + ('0' & v_bgy(9 downto 1));
                    v_ou  := ('0' & v_uu(9 downto 1)) + ('0' & v_bgu(9 downto 1));
                    v_ov  := ('0' & v_vv(9 downto 1)) + ('0' & v_bgv(9 downto 1));
                end if;
            else
                v_oy2 := v_bgy;  v_ou := v_bgu;  v_ov := v_bgv;
            end if;

            if s_mono = '1' then
                v_ou := C_MID;
                v_ov := C_MID;
            end if;

            p15_y <= v_oy2;
            p15_u <= v_ou;
            p15_v <= v_ov;

            -- c16: bypass mux + sync attach
            if s_bypass = '1' then
                s_io.y <= pipe(LATENCY - 1).y;
                s_io.u <= pipe(LATENCY - 1).u;
                s_io.v <= pipe(LATENCY - 1).v;
            else
                s_io.y <= std_logic_vector(p15_y);
                s_io.u <= std_logic_vector(p15_u);
                s_io.v <= std_logic_vector(p15_v);
            end if;
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= pipe(LATENCY - 1).avid;
            s_io.field_n <= pipe(LATENCY - 1).field_n;

            -- flag / colour pipes
            in_p(0)   <= p5_in;
            core_p(0) <= p5_core;
            for i in 1 to 8 loop
                in_p(i)   <= in_p(i - 1);
                core_p(i) <= core_p(i - 1);
            end loop;
            yp(0) <= p5_y;  up(0) <= p5_u;  vp(0) <= p5_v;
            for i in 1 to 7 loop
                yp(i) <= yp(i - 1);
                up(i) <= up(i - 1);
                vp(i) <= vp(i - 1);
            end loop;
        end if;
    end process p_pipe;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture gumball;
