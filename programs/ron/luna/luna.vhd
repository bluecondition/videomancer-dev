-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- LUNA v0.1.0 - the Moon as seen from Earth, lit as a sphere, in real time.
--
-- WHAT IT DRAWS
--   The near side of the Moon (NASA LRO albedo map, 128 x 128 texels at 4
--   bits, bilinear filtered), on a disc of any size and position, lit by a
--   sun that swings round it: new -> crescent -> quarter -> gibbous -> full.
--   A second small map of the surface's east-west slope tilts the lighting,
--   so craters throw shadows along the terminator.  Earthshine on the dark
--   side, an optional halo, a warm / cold tone, and the live picture as the
--   sky behind the Moon or as the Moon's own surface.
--
-- HOW
--   Everything per pixel is an accumulator; nothing is multiplied by a
--   coordinate.  With xn, yn the position in disc radii from the centre:
--     u, v   texture coordinates (texels)            u += du per pixel
--     m      xn * sin(A), the sideways sun term      m += dm per pixel
--     t      xn^2 + yn^2 (second-order: t += d, d += dd per pixel; the row
--            value T and its own difference Dy step once per active line)
--   t < 1 is the disc; z = sqrt(1 - t) comes from a 1024-entry ROM, and
--   mu = z cos(A) +/- m is the cosine of the sun's height over the surface.
--   The slope map adds -slope * Relief to mu.  Brightness is a soft ramp of
--   mu (clamp 4 mu, eased), times the albedo.  cos(A) is linear in the
--   Phase slider, so the terminator crosses the disc at a steady rate.
--
--   A vblank sequencer (one serial divider, one serial multiplier, started
--   when no active video has been seen for 2048 clocks) turns the knobs
--   into the start values and steps: 1/R, the first pixel's xn / yn, and
--   the twelve products the accumulators need.  Sizes and positions are in
--   virtual 1080-line units stepped from the timing ID, so SD, 720p and
--   1080p show the same picture (round on a 4:3 SD screen).
--
--   Bilinear taps without a mux on the RAM data: each map is split into
--   four sub-banks by (row parity, column parity); one read per bank gives
--   the 2 x 2 neighbourhood, and the weight is flipped (16 - f) when the
--   left / top texel is the odd one.
--
--   Pipeline: accumulators (S0) -> addresses (S1) -> ROM (S2) -> fabric
--   re-register (S3) -> row lerps, z cos(A) (S4) -> column lerp, mu (S5) ->
--   relief product (S6) -> ramp, surface level (S7) -> ease (S8) ->
--   earthshine, edge cover, sky + halo (S9) -> level x light, sky weight
--   (S10) -> sum, tone (S11) -> output (S12).  The accumulators run 3
--   pixels ahead of the raster, so the live picture is only ever 7
--   registers deep (latency 8 in every mode); the first 3 pixels of a line
--   repeat pixel 3.
--
--   CLEAN AVID (aurora / cubist recipe): the program makes its own avid,
--   starting 3 or 4 clocks after the source's, whichever keeps the
--   hsync -> start parity at its usual value, and ending after the
--   field-latched width.  Output is 64 / 512 / 512 outside it.
--
--   Colour is authored in the hardware convention (the u register carries
--   Cr): Tone > 0 raises u and lowers v = amber.
--
-- CONTROLS
--   K1  Size        disc radius, 12% .. 107% of the screen height
--   K2  X Position  centre, left edge .. right edge
--   K3  Y Position  centre, above the top .. below the bottom
--   K4  Relief      crater shadow depth along the terminator, and emboss
--   K5  Tone        cold blue <- grey -> amber
--   K6  Orbit       0 = still; else the phase cycles by itself (4.5 min .. 1 s)
--   S7  Sky         Black / Video (the live picture behind the Moon)
--   S8  Surface     Moon / Video (the live picture lit as the Moon's face)
--   S9  Lit Side    Right (waxing) / Left (waning)
--   S10 Earthshine  Faint / Bright (the dark side)
--   S11 Halo        Off / On (glow round the disc, brighter toward full)
--   P12 PHASE       new moon -> full moon
--
-- Latency: 8 clocks, constant across all modes.  EBR: 26.  No DSP.
-- Build with router1 (NEXTPNR_ROUTER="--router router1"); see SUMMARY.md.
--
-- Author: czartroniks

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.luna_pkg.all;

architecture luna of program_top is

    constant C_LAT  : natural := 8;
    constant C_BLK  : unsigned(9 downto 0) := to_unsigned(64, 10);
    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);
    -- texel centre offset: u = (xn + 1) * 64 - 0.5 = xn * 64 + 63.5 (Q7.18)
    constant C_UOFF : unsigned(24 downto 0) := to_unsigned(127 * 131072, 25);

    --------------------------------------------------------------------------
    -- Resolution scaling (mondrian's functions): steps per pixel / per field
    -- line in virtual 1080-line units, from the video timing ID.
    --------------------------------------------------------------------------
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;

    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 12);  -- 480i
            when 4          => return to_unsigned(576, 12);   -- 480p
            when 8          => return to_unsigned(960, 12);   -- 576i
            when 12         => return to_unsigned(480, 12);   -- 576p
            when 5 | 6 | 14 => return to_unsigned(384, 12);   -- 720p
            when 1 | 2 | 10 => return to_unsigned(512, 12);   -- 1080i
            when others     => return to_unsigned(256, 12);   -- 1080p
        end case;
    end function;

    -- knob glide, Q10.4: a += (k - a) / 16 per field
    function f_glide(a : unsigned(13 downto 0); k : std_logic_vector(9 downto 0))
        return unsigned is
        variable v_t : unsigned(13 downto 0);
        variable v_d : signed(14 downto 0);
        variable v_n : signed(14 downto 0);
    begin
        v_t := unsigned(k) & "0000";
        v_d := signed(resize(v_t, 15)) - signed(resize(a, 15));
        v_n := signed(resize(a, 15)) + shift_right(v_d, 4);
        return unsigned(v_n(13 downto 0));
    end function;

    -- bilinear lerps, weight f = 0 .. 16:  16 a + f (b - a)
    function f_lerp4(a, b : unsigned(3 downto 0); f : unsigned(4 downto 0))
        return unsigned is
        variable v_d : signed(5 downto 0);
        variable v_p : signed(11 downto 0);
        variable v_r : signed(11 downto 0);
    begin
        v_d := signed(resize(b, 6)) - signed(resize(a, 6));
        v_p := v_d * signed(resize(f, 6));
        v_r := shift_left(signed(resize(a, 12)), 4) + v_p;
        return unsigned(v_r(7 downto 0));
    end function;

    function f_lerp8(a, b : unsigned(7 downto 0); f : unsigned(4 downto 0))
        return unsigned is
        variable v_d : signed(9 downto 0);
        variable v_p : signed(15 downto 0);
        variable v_r : signed(15 downto 0);
    begin
        v_d := signed(resize(b, 10)) - signed(resize(a, 10));
        v_p := v_d * signed(resize(f, 6));
        v_r := shift_left(signed(resize(a, 16)), 4) + v_p;
        return unsigned(v_r(11 downto 0));
    end function;

    function f_slerp4(a, b : signed(3 downto 0); f : unsigned(4 downto 0))
        return signed is
        variable v_d : signed(4 downto 0);
        variable v_p : signed(10 downto 0);
        variable v_r : signed(10 downto 0);
    begin
        v_d := resize(b, 5) - resize(a, 5);
        v_p := v_d * signed(resize(f, 6));
        v_r := shift_left(resize(a, 11), 4) + v_p;
        return v_r(8 downto 0);
    end function;

    function f_slerp9(a, b : signed(8 downto 0); f : unsigned(4 downto 0))
        return signed is
        variable v_d : signed(9 downto 0);
        variable v_p : signed(15 downto 0);
        variable v_r : signed(15 downto 0);
    begin
        v_d := resize(b, 10) - resize(a, 10);
        v_p := v_d * signed(resize(f, 6));
        v_r := shift_left(resize(a, 16), 4) + v_p;
        return v_r(12 downto 0);
    end function;

    --------------------------------------------------------------------------
    -- Sync / video delay, generated avid
    --------------------------------------------------------------------------
    type t_dsr is array (1 to C_LAT - 1) of unsigned(9 downto 0);
    subtype t_bsr is std_logic_vector(1 to C_LAT - 1);
    signal d_y, d_u, d_v : t_dsr := (others => (others => '0'));
    signal d_hs, d_vs, d_fl, d_av, d_gv : t_bsr := (others => '0');
    signal s_io : t_video_stream_yuv444_30b;

    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid, g_avd : std_logic := '0';
    signal g_start : std_logic;
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal s_hcnt, s_meas_h : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_lcnt  : unsigned(4 downto 0) := (others => '0');
    signal s_ltick : std_logic := '0';
    signal s_quiet : unsigned(11 downto 0) := (others => '0');
    signal s_qfired, s_go : std_logic := '0';

    -- timing class (free-running decode of the timing ID)
    signal s_xs : unsigned(2 downto 0)  := to_unsigned(2, 3);
    signal s_ys : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_sd : std_logic := '0';

    --------------------------------------------------------------------------
    -- Vblank sequencer
    --------------------------------------------------------------------------
    type t_st is (ST_IDLE, ST_GLIDE, ST_PREP, ST_DIV, ST_SX, ST_SX2, ST_LOAD, ST_MUL,
                  ST_STORE, ST_ZW0, ST_ZW1, ST_ZW2, ST_ZW3);
    signal q_st : t_st := ST_IDLE;
    attribute fsm_encoding : string;
    attribute fsm_encoding of q_st : signal is "none";

    signal q_k    : unsigned(3 downto 0) := (others => '0');
    signal q_cnt  : unsigned(4 downto 0) := (others => '0');
    signal a_k1   : unsigned(13 downto 0) := to_unsigned(277 * 16, 14);
    signal a_k2   : unsigned(13 downto 0) := to_unsigned(512 * 16, 14);
    signal a_k3   : unsigned(13 downto 0) := to_unsigned(512 * 16, 14);
    signal a_p12  : unsigned(13 downto 0) := to_unsigned(720 * 16, 14);
    signal q_th   : unsigned(15 downto 0) := (others => '0');
    signal q_rq2  : unsigned(12 downto 0) := to_unsigned(1620, 13);
    signal q_rem  : unsigned(12 downto 0) := (others => '0');
    signal q_dbit : std_logic := '0';
    signal q_quo  : unsigned(17 downto 0) := (others => '0');
    signal q_sx   : unsigned(19 downto 0) := (others => '0');
    signal q_sy   : unsigned(20 downto 0) := (others => '0');
    signal q_x0e  : signed(15 downto 0) := (others => '0');
    signal q_y0e  : signed(14 downto 0) := (others => '0');
    signal q_xn0, q_yn0 : signed(29 downto 0) := (others => '0');
    signal q_aci  : unsigned(8 downto 0) := (others => '0');
    signal q_d0a  : signed(33 downto 0) := (others => '0');
    signal q_sin  : unsigned(7 downto 0) := (others => '0');
    signal mul_a   : signed(29 downto 0) := (others => '0');
    signal mul_b   : unsigned(22 downto 0) := (others => '0');
    signal mul_acc : signed(30 downto 0) := (others => '0');
    signal mul_lo  : unsigned(22 downto 0) := (others => '0');
    signal s_zsel  : std_logic := '0';
    signal s_seq_za : unsigned(9 downto 0) := (others => '0');
    signal s_ready : std_logic := '0';

    -- per-field constants used by the pixel path
    signal f_u0  : unsigned(24 downto 0) := (others => '0');
    signal f_m0  : unsigned(23 downto 0) := (others => '0');
    signal f_dm  : unsigned(23 downto 0) := (others => '0');
    signal f_d0  : signed(33 downto 0) := (others => '0');
    signal f_dd  : unsigned(22 downto 0) := (others => '0');
    signal f_ddy : unsigned(24 downto 0) := (others => '0');
    signal f_ci  : signed(9 downto 0) := (others => '0');
    signal f_p6  : unsigned(5 downto 0) := (others => '0');
    signal f_csh : unsigned(2 downto 0) := (others => '0');   -- cover shift - 14
    signal f_gk  : unsigned(5 downto 0) := (others => '0');
    signal f_gks : signed(6 downto 0) := (others => '0');     -- Relief, signed by the lit side
    signal f_tone : signed(5 downto 0) := (others => '0');
    signal f_sun, f_sky, f_surf, f_earth, f_halo : std_logic := '0';

    -- per-line running values
    signal l_v  : unsigned(24 downto 0) := (others => '0');
    signal l_T  : unsigned(33 downto 0) := (others => '1');
    signal l_Dy : signed(33 downto 0) := (others => '0');
    signal r_rE, r_rO : unsigned(5 downto 0) := (others => '0');
    signal r_2E, r_2O : unsigned(4 downto 0) := (others => '0');
    signal r_fv, r_fv2 : unsigned(4 downto 0) := (others => '0');
    -- the per-line values only ever clear to a constant or add a registered
    -- increment (one carry chain into each register, no data mux)
    signal s_tinc : unsigned(33 downto 0) := (others => '0');
    signal s_dinc : signed(33 downto 0) := (others => '0');
    signal s_vinc : unsigned(24 downto 0) := (others => '0');
    signal s_clr_t, s_clr_d, s_clr_v : std_logic := '0';
    signal s_cq_t, s_cq_d, s_cq_v    : std_logic := '0';
    signal s_add_t, s_add_d, s_lacc  : std_logic := '0';

    --------------------------------------------------------------------------
    -- Pixel path
    --------------------------------------------------------------------------
    signal p_u : unsigned(24 downto 0) := (others => '0');
    signal p_m : unsigned(23 downto 0) := (others => '0');
    signal p_t : unsigned(33 downto 0) := (others => '1');
    signal p_d : signed(33 downto 0) := (others => '0');

    signal a1_cE, a1_cO : unsigned(5 downto 0) := (others => '0');
    signal a1_fu : unsigned(4 downto 0) := (others => '0');
    signal a1_za : unsigned(9 downto 0) := (others => '0');
    signal a1_cv : unsigned(2 downto 0) := (others => '0');
    signal a1_h  : unsigned(5 downto 0) := (others => '0');
    signal a1_m  : signed(11 downto 0) := (others => '0');

    signal s_ad_a00, s_ad_a01, s_ad_a10, s_ad_a11 : unsigned(11 downto 0);
    signal s_ad_s00, s_ad_s01, s_ad_s10, s_ad_s11 : unsigned(10 downto 0);
    signal q_a00, q_a01, q_a10, q_a11 : std_logic_vector(3 downto 0) := (others => '0');
    signal q_s00, q_s01, q_s10, q_s11 : std_logic_vector(3 downto 0) := (others => '0');
    signal q_z : std_logic_vector(7 downto 0) := (others => '0');

    signal b2_fu : unsigned(4 downto 0) := (others => '0');
    signal b2_cv : unsigned(2 downto 0) := (others => '0');
    signal b2_h, b2_hp : unsigned(5 downto 0) := (others => '0');
    signal b2_m  : signed(11 downto 0) := (others => '0');

    signal c3_a00, c3_a01, c3_a10, c3_a11 : unsigned(3 downto 0) := (others => '0');
    signal c3_s00, c3_s01, c3_s10, c3_s11 : signed(3 downto 0) := (others => '0');
    signal c3_z  : unsigned(7 downto 0) := (others => '0');
    signal c3_fu : unsigned(4 downto 0) := (others => '0');
    signal c3_cv : unsigned(2 downto 0) := (others => '0');
    signal c3_g  : unsigned(6 downto 0) := (others => '0');
    signal c3_m  : signed(11 downto 0) := (others => '0');

    signal d4_hE, d4_hO : unsigned(7 downto 0) := (others => '0');
    signal d4_sE, d4_sO : signed(8 downto 0) := (others => '0');
    signal d4_pz : signed(11 downto 0) := (others => '0');
    signal d4_m  : signed(11 downto 0) := (others => '0');
    signal d4_cv : unsigned(2 downto 0) := (others => '0');
    signal d4_g  : unsigned(6 downto 0) := (others => '0');

    signal e5_ab  : unsigned(11 downto 0) := (others => '0');
    signal e5_sb  : signed(11 downto 0) := (others => '0');
    signal e5_mu0 : signed(12 downto 0) := (others => '0');
    signal e5_cv  : unsigned(2 downto 0) := (others => '0');
    signal e5_g   : unsigned(6 downto 0) := (others => '0');

    signal f6_rel  : signed(11 downto 0) := (others => '0');
    signal f6_albb : unsigned(9 downto 0) := (others => '0');
    signal f6_mu0  : signed(12 downto 0) := (others => '0');
    signal f6_cv   : unsigned(2 downto 0) := (others => '0');
    signal f6_g    : unsigned(6 downto 0) := (others => '0');

    signal g7_s8   : unsigned(7 downto 0) := (others => '0');
    signal g7_albv : unsigned(9 downto 0) := (others => '0');
    signal g7_cv   : unsigned(2 downto 0) := (others => '0');
    signal g7_g    : unsigned(6 downto 0) := (others => '0');

    signal h8_shade : unsigned(7 downto 0) := (others => '0');
    signal h8_albv  : unsigned(9 downto 0) := (others => '0');
    signal h8_cv    : unsigned(2 downto 0) := (others => '0');
    signal h8_g     : unsigned(6 downto 0) := (others => '0');

    signal i9_lit  : unsigned(7 downto 0) := (others => '0');
    signal i9_bgs  : unsigned(9 downto 0) := (others => '0');
    signal i9_albv : unsigned(9 downto 0) := (others => '0');
    signal i9_cv   : unsigned(2 downto 0) := (others => '0');

    signal j10_prod : unsigned(17 downto 0) := (others => '0');
    signal j10_bgw  : unsigned(9 downto 0) := (others => '0');
    signal j10_moon : std_logic := '0';
    signal j10_sh   : unsigned(1 downto 0) := (others => '0');

    signal k11_y    : unsigned(9 downto 0) := (others => '0');
    signal k11_ct   : signed(8 downto 0) := (others => '0');
    signal k11_vu, k11_vv : signed(10 downto 0) := (others => '0');
    signal k11_moon : std_logic := '0';

begin

    -- ====================================================================
    -- Timing class, free-running
    -- ====================================================================
    p_class : process(clk)
        variable v_t : std_logic_vector(3 downto 0);
    begin
        if rising_edge(clk) then
            v_t := registers_in(8)(3 downto 0);
            s_xs <= f_vstep_x(v_t);
            s_ys <= f_vstep_y(v_t);
            if v_t(1 downto 0) = "00" then s_sd <= '1'; else s_sd <= '0'; end if;
        end if;
    end process p_class;

    -- ====================================================================
    -- Clean avid (see header), line width, line tick, field tick
    -- ====================================================================
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1')
               else '0';

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

            if g_start = '1' then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;

            -- source line width (clocks of avid between hsyncs)
            if g_hsr = '1' then
                if s_hcnt /= 0 then s_meas_h <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            elsif data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            end if;

            -- line tick: 16 clocks after the generated avid ends (active
            -- lines only), when the tail of the line has left the stages
            -- that use the per-line values
            g_avd <= g_avid;
            s_ltick <= '0';
            if g_avd = '1' and g_avid = '0' then
                s_lcnt <= to_unsigned(16, 5);
            elsif s_lcnt /= 0 then
                s_lcnt <= s_lcnt - 1;
                if s_lcnt = 1 then s_ltick <= '1'; end if;
            end if;

            -- field tick: no active video for 2048 clocks (hblank is at most
            -- 830), once per vertical blanking
            s_go <= '0';
            if data_in.avid = '1' then
                s_quiet  <= (others => '0');
                s_qfired <= '0';
            elsif s_quiet(11) = '0' then
                s_quiet <= s_quiet + 1;
            elsif s_qfired = '0' then
                s_qfired <= '1';
                s_go     <= '1';
            end if;
        end if;
    end process p_gavid;

    -- ====================================================================
    -- Vblank sequencer + per-line accumulators
    --   s   = 2^26 / (4 R)                      Q.24 per virtual unit
    --   sx  = s * (x step)                      Q.24 per pixel
    --   product k:  0 xn0 = x0 s      1 yn0 = y0 s     2 sy = ystep s
    --               3 cos^2 -> sqrt ROM -> sin          4 m0 = xn0 sin
    --               5 dm = sx sin     6 d0 = 2 xn0 sx   7 sx^2 (dd, d0)
    --               8 xn0^2 (T)       9 Dy = 2 yn0 sy   10 sy^2 (ddy, Dy)
    --               11 yn0^2 (T)
    -- ====================================================================
    p_seq : process(clk)
        variable v_p    : signed(53 downto 0);
        variable v_sx   : std_logic_vector(23 downto 0);
        variable v_sum  : signed(30 downto 0);
        variable v_r    : unsigned(13 downto 0);
        variable v_k6   : unsigned(9 downto 0);
        variable v_mant : unsigned(11 downto 0);
        variable v_ph   : unsigned(10 downto 0);
        variable v_p10  : unsigned(9 downto 0);
        variable v_w    : std_logic;
        variable v_ci   : signed(9 downto 0);
        variable v_d2, v_d3 : signed(14 downto 0);
        variable v_cx   : signed(15 downto 0);
        variable v_cy   : signed(15 downto 0);
        variable v_xl   : signed(15 downto 0);
        variable v_abs  : signed(23 downto 0);
        variable v_kn   : std_logic_vector(9 downto 0);
        variable v_msb  : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            -- per-line step (active lines; never during the sequencer):
            -- load the increments now, p_line adds them on the next clock
            s_clr_t <= '0';  s_clr_d <= '0';  s_clr_v <= '0';
            s_add_t <= '0';  s_add_d <= '0';
            s_lacc  <= s_ltick;
            if s_ltick = '1' then
                s_vinc <= resize(q_sy, 25);
                s_tinc <= unsigned(shift_right(l_Dy, 9));
                s_dinc <= signed(resize(f_ddy, 34));
            end if;

            v_p(53 downto 23) := mul_acc;
            v_p(22 downto 0)  := signed(mul_lo);

            case q_st is
                when ST_IDLE =>
                    if s_go = '1' then q_st <= ST_GLIDE; end if;

                when ST_GLIDE =>
                    a_k1  <= f_glide(a_k1,  registers_in(0));
                    a_k2  <= f_glide(a_k2,  registers_in(1));
                    a_k3  <= f_glide(a_k3,  registers_in(2));
                    a_p12 <= f_glide(a_p12, registers_in(7));
                    f_gk  <= unsigned(registers_in(3)(9 downto 4));
                    v_kn  := registers_in(4);
                    f_tone(5) <= not v_kn(9);                 -- (knob - 512) / 16
                    f_tone(4 downto 0) <= signed(v_kn(8 downto 4));
                    f_sky   <= registers_in(6)(0);
                    f_surf  <= registers_in(6)(1);
                    f_earth <= registers_in(6)(3);
                    f_halo  <= registers_in(6)(4);
                    g_wm1   <= s_meas_h - 1;
                    -- Orbit: 0 = still (phase = the slider); else 4.5 min .. 1 s
                    v_k6 := unsigned(registers_in(5));
                    if v_k6(9 downto 4) = 0 then
                        q_th <= (others => '0');
                    else
                        v_mant := (others => '0');
                        v_mant(4) := '1';
                        v_mant(3 downto 0) := v_k6(6 downto 3);
                        v_mant := shift_left(v_mant, to_integer(v_k6(9 downto 7)));
                        q_th <= q_th + resize(v_mant(11 downto 2), 16);
                    end if;
                    q_st <= ST_PREP;

                when ST_PREP =>
                    q_rq2 <= to_unsigned(512, 13) + resize(a_k1(13 downto 2), 13);
                    -- phase: 0 .. 1023 waxing, then back down waning
                    v_ph := resize(a_p12(13 downto 4), 11) + q_th(15 downto 5);
                    if v_ph(10) = '0' then
                        v_p10 := v_ph(9 downto 0);       v_w := '0';
                    else
                        v_p10 := not v_ph(9 downto 0);   v_w := '1';
                    end if;
                    f_sun <= v_w xor registers_in(6)(2);
                    v_ci(9) := not v_p10(9);                 -- p10 - 512
                    v_ci(8 downto 0) := signed(v_p10(8 downto 0));
                    f_ci  <= v_ci;
                    -- |cos| as a one's complement (0 .. 511), so cos^2 < 1
                    for i in 0 to 8 loop
                        q_aci(i) <= v_ci(i) xor v_ci(9);
                    end loop;
                    if (v_w xor registers_in(6)(2)) = '1' then
                        f_gks <= -signed(resize(f_gk, 7));
                    else
                        f_gks <= signed(resize(f_gk, 7));
                    end if;
                    f_p6  <= v_p10(9 downto 4);
                    -- first pixel / line relative to the centre, 1/8 virtual units
                    v_d2 := signed(resize(a_k2, 15)) - to_signed(8192, 15);
                    if s_sd = '1' then v_cx := to_signed(5760, 16);
                    else               v_cx := to_signed(7680, 16); end if;
                    v_cx := v_cx + resize(v_d2, 16) - resize(shift_right(v_d2, 4), 16);
                    case to_integer(s_xs) is             -- 3 pixels of lookahead
                        when 4      => v_xl := to_signed(48, 16);
                        when 3      => v_xl := to_signed(36, 16);
                        when others => v_xl := to_signed(24, 16);
                    end case;
                    q_x0e <= v_xl - v_cx;
                    v_d3 := signed(resize(a_k3, 15)) - to_signed(8192, 15);
                    v_cy := to_signed(4320, 16) + resize(shift_right(v_d3, 1), 16)
                            + resize(shift_right(v_d3, 5), 16);
                    v_cy := -v_cy;
                    q_y0e <= v_cy(14 downto 0);
                    q_rem  <= (others => '0');
                    q_quo  <= (others => '0');
                    q_dbit <= '1';
                    q_cnt  <= to_unsigned(26, 5);
                    q_st   <= ST_DIV;

                when ST_DIV =>                           -- 2^26 / rq2, restoring
                    v_r := q_rem & q_dbit;
                    if v_r >= resize(q_rq2, 14) then
                        v_r := v_r - resize(q_rq2, 14);
                        q_quo <= q_quo(16 downto 0) & '1';
                    else
                        q_quo <= q_quo(16 downto 0) & '0';
                    end if;
                    q_rem  <= v_r(12 downto 0);
                    q_dbit <= '0';
                    if q_cnt = 0 then q_st <= ST_SX;
                    else q_cnt <= q_cnt - 1; end if;

                when ST_SX =>
                    case to_integer(s_xs) is
                        when 4      => q_sx <= resize(q_quo, 19) & '0';
                        when 3      => q_sx <= resize(q_quo, 20) + resize(q_quo(17 downto 1), 20);
                        when others => q_sx <= resize(q_quo, 20);
                    end case;
                    q_st <= ST_SX2;

                when ST_SX2 =>
                    -- edge cover shift = msb(sx) (+1 on 1080-line rasters), 14 .. 18
                    if    q_sx(19) = '1' or q_sx(18) = '1' then v_msb := "100";
                    elsif q_sx(17) = '1' then v_msb := "011";
                    elsif q_sx(16) = '1' then v_msb := "010";
                    elsif q_sx(15) = '1' then v_msb := "001";
                    elsif q_sx(14) = '1' then v_msb := "000";
                    else                      v_msb := "111";       -- 13
                    end if;
                    if s_xs = 2 then
                        if v_msb = "100" then f_csh <= "100";
                        else f_csh <= v_msb + 1; end if;
                    else
                        if v_msb = "111" then f_csh <= "000";
                        else f_csh <= v_msb; end if;
                    end if;
                    q_k  <= (others => '0');
                    q_st <= ST_LOAD;

                when ST_LOAD =>
                    case to_integer(q_k) is
                        when 0 =>  mul_a <= resize(q_x0e, 30);  mul_b <= resize(q_quo, 23);
                        when 1 =>  mul_a <= resize(q_y0e, 30);  mul_b <= resize(q_quo, 23);
                        when 2 =>  mul_a <= signed(resize(s_ys, 30));  mul_b <= resize(q_quo, 23);
                        when 3 =>  mul_a <= signed(resize(q_aci, 30)); mul_b <= resize(q_aci, 23);
                        when 4 =>  mul_a <= q_xn0;              mul_b <= resize(q_sin, 23);
                        when 5 =>  mul_a <= signed(resize(q_sx, 30));  mul_b <= resize(q_sin, 23);
                        when 6 =>  mul_a <= q_xn0;              mul_b <= resize(q_sx, 23);
                        when 7 =>  mul_a <= signed(resize(q_sx, 30));  mul_b <= resize(q_sx, 23);
                        when 8 =>                -- |x| as a one's complement
                            for i in 0 to 23 loop
                                v_abs(i) := q_xn0(i + 6) xor q_xn0(29);
                            end loop;
                            mul_a <= resize(v_abs, 30);  mul_b <= unsigned(v_abs(22 downto 0));
                        when 9 =>  mul_a <= q_yn0;              mul_b <= resize(q_sy, 23);
                        when 10 => mul_a <= signed(resize(q_sy, 30));  mul_b <= resize(q_sy, 23);
                        when others =>
                            for i in 0 to 23 loop
                                v_abs(i) := q_yn0(i + 6) xor q_yn0(29);
                            end loop;
                            mul_a <= resize(v_abs, 30);  mul_b <= unsigned(v_abs(22 downto 0));
                    end case;
                    mul_acc <= (others => '0');
                    mul_lo  <= (others => '0');
                    q_cnt   <= to_unsigned(22, 5);
                    q_st    <= ST_MUL;

                when ST_MUL =>                           -- 23 shift-add steps
                    if mul_b(0) = '1' then v_sum := mul_acc + resize(mul_a, 31);
                    else                   v_sum := mul_acc; end if;
                    mul_acc <= shift_right(v_sum, 1);
                    mul_lo  <= v_sum(0) & mul_lo(22 downto 1);
                    mul_b   <= '0' & mul_b(22 downto 1);
                    if q_cnt = 0 then q_st <= ST_STORE;
                    else q_cnt <= q_cnt - 1; end if;

                when ST_STORE =>
                    q_k  <= q_k + 1;
                    q_st <= ST_LOAD;
                    case to_integer(q_k) is
                        when 0 =>
                            q_xn0 <= v_p(32 downto 3);
                            f_u0  <= unsigned(v_p(27 downto 3)) + C_UOFF;
                        when 1 =>
                            q_yn0   <= v_p(32 downto 3);
                            s_vinc  <= unsigned(v_p(27 downto 3));
                            s_clr_v <= '1';
                        when 2 =>
                            q_sy <= unsigned(v_p(28 downto 8));
                        when 3 =>
                            s_seq_za <= unsigned(v_p(17 downto 8));
                            s_zsel <= '1';
                            q_st   <= ST_ZW0;
                        when 4 =>                    -- lit on the left: m negated
                            for i in 0 to 23 loop    -- (one's complement)
                                f_m0(i) <= v_p(i + 10) xor f_sun;
                            end loop;
                        when 5 =>
                            v_sx := (others => '0');
                            v_sx(18 downto 0) := std_logic_vector(v_p(28 downto 10));
                            for i in 0 to 23 loop
                                f_dm(i) <= v_sx(i) xor f_sun;
                            end loop;
                        when 6 =>
                            q_d0a <= v_p(47 downto 14);
                        when 7 =>
                            f_dd <= unsigned(v_p(36 downto 14));
                            f_d0 <= q_d0a + signed(resize(unsigned(v_p(40 downto 15)), 34));
                        when 8 =>
                            s_tinc  <= unsigned(v_p(45 downto 12));
                            s_clr_t <= '1';
                        when 9 =>
                            s_dinc  <= v_p(47 downto 14);
                            s_clr_d <= '1';
                        when 10 =>
                            f_ddy   <= unsigned(v_p(38 downto 14));
                            s_dinc  <= signed(resize(unsigned(v_p(42 downto 15)), 34));
                            s_add_d <= '1';
                        when others =>
                            s_tinc  <= unsigned(v_p(45 downto 12));
                            s_add_t <= '1';
                            s_ready <= '1';
                            q_st    <= ST_IDLE;
                    end case;

                -- sin(A) = sqrt(1 - cos^2) from the pixel path's sqrt ROM
                when ST_ZW0 => q_st <= ST_ZW1;
                when ST_ZW1 => q_st <= ST_ZW2;
                when ST_ZW2 => q_st <= ST_ZW3;
                when ST_ZW3 =>
                    q_sin  <= c3_z;
                    s_zsel <= '0';
                    q_st   <= ST_LOAD;
            end case;
        end if;
    end process p_seq;

    -- per-line running values: clear to a constant, or add an increment
    p_line : process(clk)
    begin
        if rising_edge(clk) then
            s_cq_t <= s_clr_t;  s_cq_d <= s_clr_d;  s_cq_v <= s_clr_v;
            if s_clr_t = '1' then
                l_T <= to_unsigned(8192, 34);        -- keeps t >= 0 at the centre
            elsif s_cq_t = '1' or s_add_t = '1' or s_lacc = '1' then
                l_T <= l_T + s_tinc;
            end if;
            if s_clr_d = '1' then
                l_Dy <= (others => '0');
            elsif s_cq_d = '1' or s_add_d = '1' or s_lacc = '1' then
                l_Dy <= l_Dy + s_dinc;
            end if;
            if s_clr_v = '1' then
                l_v <= C_UOFF;
            elsif s_cq_v = '1' or s_lacc = '1' then
                l_v <= l_v + s_vinc;
            end if;
        end if;
    end process p_line;

    -- per-line texture rows and weights (static across the line)
    p_rows : process(clk)
        variable v_iv  : unsigned(6 downto 0);
        variable v_fv  : unsigned(3 downto 0);
        variable v_iv2 : unsigned(5 downto 0);
        variable v_fv2 : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            v_iv := l_v(24 downto 18);
            v_fv := l_v(17 downto 14);
            r_rO <= v_iv(6 downto 1);
            if v_iv(0) = '1' then
                r_rE <= v_iv(6 downto 1) + 1;
                r_fv <= to_unsigned(16, 5) - resize(v_fv, 5);
            else
                r_rE <= v_iv(6 downto 1);
                r_fv <= resize(v_fv, 5);
            end if;
            -- slope rows: exactly half the albedo row coordinate
            v_iv2 := v_iv(6 downto 1);
            v_fv2 := v_iv(0) & v_fv(3 downto 1);
            r_2O <= v_iv2(5 downto 1);
            if v_iv2(0) = '1' then
                r_2E  <= v_iv2(5 downto 1) + 1;
                r_fv2 <= to_unsigned(16, 5) - resize(v_fv2, 5);
            else
                r_2E  <= v_iv2(5 downto 1);
                r_fv2 <= resize(v_fv2, 5);
            end if;
        end if;
    end process p_rows;

    -- ====================================================================
    -- S0: accumulators.  They hold pixel 3's values through blanking and
    -- step from one clock before the generated avid.
    -- ====================================================================
    p_dda : process(clk)
    begin
        if rising_edge(clk) then
            if g_start = '1' or g_avid = '1' then
                p_u <= p_u + resize(q_sx, 25);
                p_m <= p_m + f_dm;
                p_t <= unsigned(signed(p_t) + shift_right(p_d, 9));
                p_d <= p_d + signed(resize(f_dd, 34));
            else
                p_u <= f_u0;
                p_m <= f_m0;
                p_t <= l_T;
                p_d <= f_d0;
            end if;
        end if;
    end process p_dda;

    -- ====================================================================
    -- S2: the nine ROM reads (one inferred block RAM group each)
    -- ====================================================================
    s_ad_a00 <= r_rE & a1_cE;
    s_ad_a01 <= r_rE & a1_cO;
    s_ad_a10 <= r_rO & a1_cE;
    s_ad_a11 <= r_rO & a1_cO;
    s_ad_s00 <= r_2E & a1_cE;
    s_ad_s01 <= r_2E & a1_cO;
    s_ad_s10 <= r_2O & a1_cE;
    s_ad_s11 <= r_2O & a1_cO;

    p_ra00 : process(clk) begin if rising_edge(clk) then q_a00 <= C_LU_A00(to_integer(s_ad_a00)); end if; end process;
    p_ra01 : process(clk) begin if rising_edge(clk) then q_a01 <= C_LU_A01(to_integer(s_ad_a01)); end if; end process;
    p_ra10 : process(clk) begin if rising_edge(clk) then q_a10 <= C_LU_A10(to_integer(s_ad_a10)); end if; end process;
    p_ra11 : process(clk) begin if rising_edge(clk) then q_a11 <= C_LU_A11(to_integer(s_ad_a11)); end if; end process;
    p_rs00 : process(clk) begin if rising_edge(clk) then q_s00 <= C_LU_S00(to_integer(s_ad_s00)); end if; end process;
    p_rs01 : process(clk) begin if rising_edge(clk) then q_s01 <= C_LU_S01(to_integer(s_ad_s01)); end if; end process;
    p_rs10 : process(clk) begin if rising_edge(clk) then q_s10 <= C_LU_S10(to_integer(s_ad_s10)); end if; end process;
    p_rs11 : process(clk) begin if rising_edge(clk) then q_s11 <= C_LU_S11(to_integer(s_ad_s11)); end if; end process;
    p_rz   : process(clk) begin if rising_edge(clk) then q_z   <= C_LU_Z(to_integer(a1_za));     end if; end process;

    -- ====================================================================
    -- Pixel pipeline S1 .. S12
    -- ====================================================================
    p_pix : process(clk)
        variable v_iu   : unsigned(6 downto 0);
        variable v_fu   : unsigned(3 downto 0);
        variable v_w    : unsigned(23 downto 0);
        variable v_x    : unsigned(9 downto 0);
        variable v_tq   : unsigned(6 downto 0);
        variable v_hh   : unsigned(11 downto 0);
        variable v_pz   : signed(18 downto 0);
        variable v_rl   : signed(18 downto 0);
        variable v_s13  : signed(12 downto 0);
        variable v_mu   : signed(13 downto 0);
        variable v_vy   : unsigned(9 downto 0);
        variable v_base : unsigned(9 downto 0);
        variable v_emb  : signed(11 downto 0);
        variable v_al   : signed(12 downto 0);
        variable v_f9   : unsigned(8 downto 0);
        variable v_kn   : unsigned(8 downto 0);
        variable v_l    : unsigned(9 downto 0);
        variable v_bg   : unsigned(9 downto 0);
        variable v_bs   : unsigned(10 downto 0);
        variable v_b3   : unsigned(11 downto 0);
        variable v_ys   : unsigned(11 downto 0);
        variable v_ctp  : signed(12 downto 0);
        variable v_du, v_dv : signed(10 downto 0);
        variable v_uo, v_vo : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            ---------------------------------------------------------------- S1
            v_iu := p_u(24 downto 18);
            v_fu := p_u(17 downto 14);
            a1_cO <= v_iu(6 downto 1);
            if v_iu(0) = '1' then
                a1_cE <= v_iu(6 downto 1) + 1;
                a1_fu <= to_unsigned(16, 5) - resize(v_fu, 5);
            else
                a1_cE <= v_iu(6 downto 1);
                a1_fu <= resize(v_fu, 5);
            end if;
            if s_zsel = '1' then a1_za <= s_seq_za;
            else                 a1_za <= p_t(23 downto 14); end if;
            a1_m <= signed(p_m(23 downto 12));

            -- edge cover 0 .. 4 (quarters) from 1 - t, over 1 .. 4 pixels
            v_w := not p_t(23 downto 0);
            case to_integer(f_csh) is
                when 0      => v_x := v_w(23 downto 14);
                when 1      => v_x := resize(v_w(23 downto 15), 10);
                when 2      => v_x := resize(v_w(23 downto 16), 10);
                when 3      => v_x := resize(v_w(23 downto 17), 10);
                when others => v_x := resize(v_w(23 downto 18), 10);
            end case;
            if p_t(33 downto 24) /= 0 or s_ready = '0' then
                a1_cv <= "000";
            elsif v_x(9 downto 2) /= 0 then
                a1_cv <= "100";
            else
                a1_cv <= '0' & v_x(1 downto 0);
            end if;

            -- halo depth: 63 on the disc, falling to 0 at t = 3
            v_tq := p_t(25 downto 19);
            if p_t(33 downto 26) /= 0 or v_tq(6 downto 5) = "11" then
                a1_h <= (others => '0');
            elsif v_tq(6 downto 5) = "00" then
                a1_h <= (others => '1');
            else
                a1_h <= (not v_tq(6)) & (not v_tq(4 downto 0));    -- 95 - tq
            end if;

            ---------------------------------------------------------------- S2
            b2_fu <= a1_fu;
            b2_cv <= a1_cv;
            b2_m  <= a1_m;
            b2_h  <= a1_h;
            v_hh  := a1_h * f_p6;
            b2_hp <= v_hh(11 downto 6);

            ---------------------------------------------------------------- S3
            c3_a00 <= unsigned(q_a00);  c3_a01 <= unsigned(q_a01);
            c3_a10 <= unsigned(q_a10);  c3_a11 <= unsigned(q_a11);
            c3_s00 <= signed(q_s00);    c3_s01 <= signed(q_s01);
            c3_s10 <= signed(q_s10);    c3_s11 <= signed(q_s11);
            c3_z   <= unsigned(q_z);
            c3_fu  <= b2_fu;
            c3_cv  <= b2_cv;
            c3_m   <= b2_m;
            v_hh   := b2_h * b2_hp;
            if f_halo = '1' then c3_g <= v_hh(11 downto 5);
            else                 c3_g <= (others => '0'); end if;

            ---------------------------------------------------------------- S4
            d4_hE <= f_lerp4(c3_a00, c3_a01, c3_fu);
            d4_hO <= f_lerp4(c3_a10, c3_a11, c3_fu);
            d4_sE <= f_slerp4(c3_s00, c3_s01, c3_fu);
            d4_sO <= f_slerp4(c3_s10, c3_s11, c3_fu);
            v_pz  := signed(resize(c3_z, 9)) * f_ci;
            d4_pz <= v_pz(18 downto 7);
            d4_m  <= c3_m;
            d4_cv <= c3_cv;
            d4_g  <= c3_g;

            ---------------------------------------------------------------- S5
            e5_ab <= f_lerp8(d4_hE, d4_hO, r_fv);
            v_s13 := f_slerp9(d4_sE, d4_sO, r_fv2);
            e5_sb <= v_s13(11 downto 0);
            e5_mu0 <= resize(d4_pz, 13) + resize(d4_m, 13);
            e5_cv <= d4_cv;
            e5_g  <= d4_g;

            ---------------------------------------------------------------- S6
            v_rl    := e5_sb * f_gks;
            f6_rel  <= v_rl(18 downto 7);
            f6_albb <= to_unsigned(208, 10) + resize(e5_ab(11 downto 3), 10);
            f6_mu0  <= e5_mu0;
            f6_cv   <= e5_cv;
            f6_g    <= e5_g;

            ---------------------------------------------------------------- S7
            v_mu := resize(f6_mu0, 14) - resize(f6_rel, 14);
            if v_mu(13) = '1' then
                g7_s8 <= (others => '0');
            elsif v_mu(12 downto 8) /= 0 then
                g7_s8 <= (others => '1');
            else
                g7_s8 <= unsigned(v_mu(7 downto 0));
            end if;
            -- surface level: the map, or the live picture (3/4, black-referred)
            if d_av(2) = '0' or d_y(2) < C_BLK then v_vy := (others => '0');
            else v_vy := d_y(2) - C_BLK; end if;
            if f_surf = '1' then v_base := v_vy - shift_right(v_vy, 2);
            else                 v_base := f6_albb; end if;
            v_emb := shift_right(f6_rel, 1);
            v_al := signed(resize(v_base, 13)) - resize(v_emb, 13);
            if v_al(12) = '1' then
                g7_albv <= (others => '0');
            elsif v_al > to_signed(704, 13) then
                g7_albv <= to_unsigned(704, 10);
            else
                g7_albv <= unsigned(v_al(9 downto 0));
            end if;
            g7_cv <= f6_cv;
            g7_g  <= f6_g;

            ---------------------------------------------------------------- S8
            -- ease s (2 - s) in four straight pieces, slopes 7/4 5/4 3/4 1/4
            v_f9 := resize(g7_s8(5 downto 0), 9);
            case g7_s8(7 downto 6) is
                when "00"   => v_kn := shift_right(shift_left(v_f9, 3) - v_f9, 2);
                when "01"   => v_kn := to_unsigned(112, 9) + shift_right(shift_left(v_f9, 2) + v_f9, 2);
                when "10"   => v_kn := to_unsigned(192, 9) + shift_right(shift_left(v_f9, 1) + v_f9, 2);
                when others => v_kn := to_unsigned(240, 9) + shift_right(v_f9, 2);
            end case;
            h8_shade <= v_kn(7 downto 0);
            h8_albv  <= g7_albv;
            h8_cv    <= g7_cv;
            h8_g     <= g7_g;

            ---------------------------------------------------------------- S9
            -- earthshine floor, then the edge cover
            if f_earth = '1' then
                v_l := resize(h8_shade, 10) - resize(shift_right(h8_shade, 3), 10)
                       - resize(shift_right(h8_shade, 5), 10) + 40;
            else
                v_l := resize(h8_shade, 10) - resize(shift_right(h8_shade, 5), 10) + 8;
            end if;
            if v_l > 255 then v_l := to_unsigned(255, 10); end if;
            case h8_cv is
                when "000"  => i9_lit <= (others => '0');
                when "001"  => i9_lit <= resize(shift_right(v_l, 2), 8);
                when "010"  => i9_lit <= resize(shift_right(v_l, 1), 8);
                when "011"  => i9_lit <= resize(shift_right(shift_left(resize(v_l, 12), 1) + resize(v_l, 12), 2), 8);
                when others => i9_lit <= v_l(7 downto 0);
            end case;
            -- sky: black or the live picture, plus the halo
            if f_sky = '1' and d_av(4) = '1' then v_bg := d_y(4);
            else                                  v_bg := C_BLK; end if;
            v_bs := resize(v_bg, 11) + resize(h8_g, 11);
            if v_bs(10) = '1' then i9_bgs <= (others => '1');
            else                   i9_bgs <= v_bs(9 downto 0); end if;
            i9_albv <= h8_albv;
            i9_cv   <= h8_cv;

            ---------------------------------------------------------------- S10
            j10_prod <= i9_albv * i9_lit;
            v_b3 := shift_left(resize(i9_bgs, 12), 1) + resize(i9_bgs, 12);
            case i9_cv is
                when "000"  => j10_bgw <= i9_bgs;
                when "001"  => j10_bgw <= v_b3(11 downto 2) + 16;
                when "010"  => j10_bgw <= resize(shift_right(i9_bgs, 1), 10) + 32;
                when "011"  => j10_bgw <= resize(shift_right(i9_bgs, 2), 10) + 48;
                when others => j10_bgw <= C_BLK;
            end case;
            if i9_cv(2) = '1' or i9_cv(1) = '1' then j10_moon <= '1';
            else                                     j10_moon <= '0'; end if;
            if    i9_lit(7) = '1' then j10_sh <= "00";
            elsif i9_lit(6) = '1' then j10_sh <= "01";
            elsif i9_lit(5) = '1' then j10_sh <= "10";
            else                       j10_sh <= "11";
            end if;

            ---------------------------------------------------------------- S11
            v_ys := resize(j10_prod(17 downto 8), 12) + resize(j10_bgw, 12);
            if v_ys > 940 then k11_y <= to_unsigned(940, 10);
            else               k11_y <= v_ys(9 downto 0); end if;
            v_ctp  := f_tone * signed(resize(j10_prod(17 downto 12), 7));
            k11_ct <= v_ctp(12 downto 4);
            -- Surface = Video: the picture's own colour, fading with the light
            if f_surf = '1' and d_av(6) = '1' then
                v_du := signed(resize(d_u(6), 11)) - to_signed(512, 11);
                v_dv := signed(resize(d_v(6), 11)) - to_signed(512, 11);
                case j10_sh is
                    when "00"   => k11_vu <= v_du;                  k11_vv <= v_dv;
                    when "01"   => k11_vu <= shift_right(v_du, 1);  k11_vv <= shift_right(v_dv, 1);
                    when "10"   => k11_vu <= shift_right(v_du, 2);  k11_vv <= shift_right(v_dv, 2);
                    when others => k11_vu <= shift_right(v_du, 3);  k11_vv <= shift_right(v_dv, 3);
                end case;
            else
                k11_vu <= (others => '0');
                k11_vv <= (others => '0');
            end if;
            k11_moon <= j10_moon;

            ---------------------------------------------------------------- S12
            v_uo := to_signed(512, 13) + resize(k11_ct, 13) + resize(k11_vu, 13);
            v_vo := to_signed(512, 13) - resize(k11_ct, 13) + resize(k11_vv, 13);
            if d_gv(C_LAT - 1) = '0' then                -- blanking gate
                s_io.y <= std_logic_vector(C_BLK);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            else
                s_io.y <= std_logic_vector(k11_y);
                if k11_moon = '1' then
                    if    v_uo(12) = '1'           then s_io.u <= (others => '0');
                    elsif v_uo(11 downto 10) /= 0  then s_io.u <= (others => '1');
                    else                                s_io.u <= std_logic_vector(v_uo(9 downto 0)); end if;
                    if    v_vo(12) = '1'           then s_io.v <= (others => '0');
                    elsif v_vo(11 downto 10) /= 0  then s_io.v <= (others => '1');
                    else                                s_io.v <= std_logic_vector(v_vo(9 downto 0)); end if;
                elsif f_sky = '1' and d_av(C_LAT - 1) = '1' then
                    s_io.u <= std_logic_vector(d_u(C_LAT - 1));
                    s_io.v <= std_logic_vector(d_v(C_LAT - 1));
                else
                    s_io.u <= std_logic_vector(C_MID);
                    s_io.v <= std_logic_vector(C_MID);
                end if;
            end if;
            s_io.hsync_n <= d_hs(C_LAT - 1);
            s_io.vsync_n <= d_vs(C_LAT - 1);
            s_io.field_n <= d_fl(C_LAT - 1);
            s_io.avid    <= d_gv(C_LAT - 1);
        end if;
    end process p_pix;

    -- ====================================================================
    -- Sync / video delay
    -- ====================================================================
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            d_y(1)  <= unsigned(data_in.y);
            d_u(1)  <= unsigned(data_in.u);
            d_v(1)  <= unsigned(data_in.v);
            d_hs(1) <= data_in.hsync_n;
            d_vs(1) <= data_in.vsync_n;
            d_fl(1) <= data_in.field_n;
            d_av(1) <= data_in.avid;
            d_gv(1) <= g_avid;
            for i in 2 to C_LAT - 1 loop
                d_y(i)  <= d_y(i - 1);
                d_u(i)  <= d_u(i - 1);
                d_v(i)  <= d_v(i - 1);
                d_hs(i) <= d_hs(i - 1);
                d_vs(i) <= d_vs(i - 1);
                d_fl(i) <= d_fl(i - 1);
                d_av(i) <= d_av(i - 1);
                d_gv(i) <= d_gv(i - 1);
            end loop;
        end if;
    end process p_delay;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture luna;
