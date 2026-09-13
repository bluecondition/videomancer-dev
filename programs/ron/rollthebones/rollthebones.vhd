-- rollthebones.vhd  (v0.1 — video as a lit tabletop of six-sided dice)
--
-- The incoming picture is re-rendered as a full-screen grid of six-sided dice.
-- Every cell holds one die: a rounded-corner square face carrying 1..6 drilled
-- pips.  The pip count tracks the local video (luma, or chroma saturation), so
-- the field of dice reads as a halftone portrait of the source.
--
-- Look: dice are lit by a movable light.  The flat top face is matte; the
-- ROUNDED EDGES catch a soft pillow bevel (bright on the light side, dark on
-- the far) and the corners throw a small specular glint — the "reflection on
-- the curve".  Each recessed pip is a shallow dimple with a tiny rim glint.
-- Together the dice look cast onto a table, not drawn by a computer.
--
-- Colour: eight inks — Casino White, Blood Red, Onyx, Ivory, Jade, Gold, plus
-- per-die Rainbow (each die a different wheel hue) and Spectrum (die hue driven
-- by the sampled video chroma).  Pip colour auto-contrasts.  Ink=Video tints
-- the die bodies with the live picture's colour.
--
-- Geometry / timing (modelled on gumball): a single square lattice; each cell's
-- colour + die parameters are point-sampled once, during the PREVIOUS cell row,
-- into a ping-pong BRAM (buf0/buf1 by row parity) so a row's dice are ready
-- before their first pixel.  Per-cell squared constants (corner radius^2, pip
-- radius^2) are precomputed in the sample pipe, so the pixel path carries only
-- eight small multiplies: 2 (small-angle rotation) + 2 (edge bevel u*lx,v*ly)
-- + 2 (rounded-corner distance) + 2 (pip distance).  Per-frame geometry settles
-- in a short vsync FSM on one shared multiplier.  No pixel-path divide.
--
-- Tumble (P12) spills the neat aligned grid into a scattered, per-die-rotated,
-- shadow-casting pile: dice shrink to open gaps of table, rotate by a stable
-- per-cell random angle, jitter off centre, and cast a bottom-right shadow.
--
-- Colour convention: all constant inks are stored with U/V (Cb/Cr) SWAPPED to
-- match the Videomancer hardware output convention (as in mondrian/CGA/C64).
-- Live video (backdrop / bypass / Ink=Video) passes through in its native
-- convention.  NOTE: headless BT.601 sims therefore show the inks U/V-swapped;
-- verify geometry + luma in sim, trust colour from the verified-palette rule.
--
-- Controls:
--   K1 Zoom  K2 Palette  K3 Hue  K4 Light  K5 Gloss  K6 Contrast
--   S7 Source(Luma/Chroma)  S8 Luma Size  S9 Table  S10 Ink  S11 Bypass
--   P12 Tumble
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

architecture rollthebones of program_top is

    constant LATENCY  : natural := 12;                 -- video pipe depth: sync/bypass
                                                       -- tap (pipe 11) aligns with the
                                                       -- dice content (c9), so all modes
                                                       -- present ONE latency (avoids the
                                                       -- mode-dependent-tap green hue).
    constant C_MAXCOL : natural := 128;
    constant C_MID    : unsigned(9 downto 0) := to_unsigned(512, 10);

    --------------------------------------------------------------------------
    -- Scheme (ink) colour tables.  Chroma stored U/V-SWAPPED (HW convention).
    -- idx: 0 White 1 Red 2 Onyx 3 Ivory 4 Jade 5 Gold 6 Rainbow 7 Spectrum
    --------------------------------------------------------------------------
    type t_pal is array(0 to 7) of integer;
    constant SCH_Y : t_pal := (830, 300, 110, 850, 430, 560, 700, 700);
    constant SCH_U : t_pal := (512, 850, 512, 540, 356, 650, 512, 512);  -- = Cr
    constant SCH_V : t_pal := (512, 400, 512, 500, 432, 300, 512, 512);  -- = Cb
    constant SCH_POL : std_logic_vector(0 to 7) := "01101000";  -- '1' dark body, light pips

    -- 16-point chroma wheel, stored U/V-swapped (HW convention).
    type t_h16 is array(0 to 15) of integer;
    constant HUE_U : t_h16 := (512,619,710,771,792,771,710,619,512,405,314,253,232,253,314,405);
    constant HUE_V : t_h16 := (792,771,710,619,512,405,314,253,232,253,314,405,512,619,710,771);

    -- Felt table colour (HW-swapped).
    constant FELT_Y : integer := 150;
    constant FELT_U : integer := 356;
    constant FELT_V : integer := 430;

    -- Face -> 3x3 pip mask.  node n = (gv+1)*3+(gu+1); mask(n)='1' -> pip.
    function facemask(f : unsigned) return std_logic_vector is
    begin
        case to_integer(f) is
            when 1      => return "000010000";
            when 2      => return "100000001";
            when 3      => return "100010001";
            when 4      => return "101000101";
            when 5      => return "101010101";
            when others => return "101101101";
        end case;
    end function;

    --------------------------------------------------------------------------
    -- Raster position.
    --------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal r_anchor     : std_logic := '0';

    signal xloc      : unsigned(7 downto 0) := (others => '0');
    signal xidx      : unsigned(6 downto 0) := (others => '0');
    signal celly_loc : unsigned(7 downto 0) := (others => '0');
    signal celly_par : std_logic := '0';
    signal frame_act : std_logic := '0';
    signal line0     : std_logic := '0';
    signal dyc_line  : signed(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Per-frame parameters.
    --------------------------------------------------------------------------
    signal lk1,lk2,lk3,lk4,lk5,lk6,lp12 : unsigned(9 downto 0) := (others => '0');

    signal s_d    : unsigned(7 downto 0) := to_unsigned(71, 8);
    signal s_dm1  : unsigned(7 downto 0) := to_unsigned(70, 8);
    signal s_dh   : unsigned(7 downto 0) := to_unsigned(35, 8);
    signal s_dhlo : unsigned(7 downto 0) := to_unsigned(28, 8);   -- Dh-7 (denoise win)
    signal s_dhi  : unsigned(7 downto 0) := to_unsigned(36, 8);   -- Dh+1 (denoise strobe)
    signal s_r0   : unsigned(7 downto 0) := to_unsigned(31, 8);
    signal s_bw   : unsigned(7 downto 0) := to_unsigned(7, 8);
    signal s_rmax : unsigned(7 downto 0) := to_unsigned(34, 8);
    signal s_rotsc: unsigned(7 downto 0) := (others => '0');
    signal s_jitsc: unsigned(7 downto 0) := (others => '0');
    signal s_shad : unsigned(7 downto 0) := (others => '0');
    signal s_t1,s_t2,s_t3,s_t4,s_t5 : unsigned(9 downto 0) := C_MID;
    signal s_span   : unsigned(8 downto 0) := to_unsigned(245, 9);  -- gloss -> shade clamp
    signal s_glossy : std_logic := '0';                             -- high gloss (corner spec)
    signal lx, ly   : signed(7 downto 0) := (others => '0');        -- light vector (cos/4, sin/4)
    signal s_hueoff : unsigned(3 downto 0) := (others => '0');
    signal s_idx    : unsigned(2 downto 0) := (others => '0');
    signal s_pol    : std_logic := '0';
    signal s_lsg    : unsigned(5 downto 0) := to_unsigned(24, 6);

    signal s_src    : std_logic := '0';
    signal s_lsize  : std_logic := '0';
    signal s_tabvid : std_logic := '0';
    signal s_inkvid : std_logic := '0';
    signal s_bypass : std_logic := '0';

    signal fsm_t  : unsigned(7 downto 0) := (others => '1');
    signal ma, mb : signed(17 downto 0) := (others => '0');
    signal mp     : signed(35 downto 0) := (others => '0');

    signal lut_sin, lut_cos : signed(9 downto 0);

    --------------------------------------------------------------------------
    -- Cell buffers.  word (100b, index 99..0):
    --  face 99..97 | sin 96..87 | jx 86..79 | jy 78..71 | R 70..63 |
    --  Rin 62..55 | P 54..47 | pr 46..39 | rc 38..31 | bY 30..21 |
    --  bU 20..11 | bV 10..1 | pol 0    (pr/rc are radii, not squares -> the
    --  pixel path uses an octagonal distance, no square-of-distance needed)
    --------------------------------------------------------------------------
    constant CW : natural := 99;
    subtype t_word is std_logic_vector(CW downto 0);
    type t_cbuf is array(0 to C_MAXCOL - 1) of t_word;
    signal buf0, buf1 : t_cbuf := (others => (others => '0'));
    signal rd0, rd1   : t_word := (others => '0');

    --------------------------------------------------------------------------
    -- Sample pipe.
    --------------------------------------------------------------------------
    signal lfsr : unsigned(15 downto 0) := x"ACE1";
    signal acc_y, acc_u, acc_v : unsigned(12 downto 0) := (others => '0');  -- 8px denoise

    signal w0_stb : std_logic := '0';
    signal w0_par : std_logic := '0';
    signal w0_idx : unsigned(6 downto 0) := (others => '0');
    signal w0_u, w0_v : unsigned(9 downto 0) := C_MID;
    signal w0_drv : unsigned(10 downto 0) := (others => '0');
    signal w0_rr, w0_rjx, w0_rjy : signed(8 downto 0) := (others => '0');

    -- W1: multiply products + face + octant (one shallow stage each)
    signal w1_stb : std_logic := '0';
    signal w1_par : std_logic := '0';
    signal w1_idx : unsigned(6 downto 0) := (others => '0');
    signal w1_u, w1_v : unsigned(9 downto 0) := C_MID;
    signal w1_drv : unsigned(10 downto 0) := (others => '0');
    signal w1_face : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal w1_rotp, w1_jxp, w1_jyp : signed(17 downto 0) := (others => '0');
    signal w1_lsp : signed(19 downto 0) := (others => '0');
    signal w1_oct : unsigned(3 downto 0) := (others => '0');
    signal w1_rrlo : unsigned(3 downto 0) := (others => '0');

    -- W2: finalize sin / jitter / radius / hue index
    signal w2_stb : std_logic := '0';
    signal w2_par : std_logic := '0';
    signal w2_idx : unsigned(6 downto 0) := (others => '0');
    signal w2_u, w2_v : unsigned(9 downto 0) := C_MID;
    signal w2_face : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal w2_sin  : signed(9 downto 0) := (others => '0');
    signal w2_jx, w2_jy : signed(7 downto 0) := (others => '0');
    signal w2_r    : unsigned(7 downto 0) := (others => '0');
    signal w2_hidx : unsigned(3 downto 0) := (others => '0');

    -- W3: per-cell squared geometry + body colour
    signal w3_stb : std_logic := '0';
    signal w3_par : std_logic := '0';
    signal w3_idx : unsigned(6 downto 0) := (others => '0');
    signal w3_face : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal w3_sin  : signed(9 downto 0) := (others => '0');
    signal w3_jx, w3_jy : signed(7 downto 0) := (others => '0');
    signal w3_r, w3_rin, w3_p : unsigned(7 downto 0) := (others => '0');
    signal w3_pr, w3_rc : unsigned(7 downto 0) := (others => '0');
    signal w3_by, w3_bu, w3_bv : unsigned(9 downto 0) := C_MID;

    -- W4: packed word ready to write
    signal w4_stb : std_logic := '0';
    signal w4_par : std_logic := '0';
    signal w4_idx : unsigned(6 downto 0) := (others => '0');
    signal w4_word : t_word := (others => '0');

    --------------------------------------------------------------------------
    -- Pixel pipeline (c1..c9 + c10 output register in s_io).
    --------------------------------------------------------------------------
    signal c1_dx0 : signed(9 downto 0) := (others => '0');
    signal c1_par : std_logic := '0';

    signal c2_word : t_word := (others => '0');
    signal c2_dxj, c2_dyj : signed(9 downto 0) := (others => '0');

    signal c3_dxj, c3_dyj : signed(9 downto 0) := (others => '0');
    signal c3_rsx, c3_rsy : signed(16 downto 0) := (others => '0');
    signal c3_word : t_word := (others => '0');

    signal c4_u, c4_v   : signed(9 downto 0) := (others => '0');
    signal c4_ub, c4_vb : signed(7 downto 0) := (others => '0');   -- clamped for bevel mult
    signal c4_au, c4_av : unsigned(8 downto 0) := (others => '0');
    signal c4_word : t_word := (others => '0');

    signal c5_cu, c5_cv   : unsigned(8 downto 0) := (others => '0');
    signal c5_incorner    : std_logic := '0';
    signal c5_edgein      : std_logic := '0';
    signal c5_bx, c5_by   : signed(15 downto 0) := (others => '0');
    signal c5_ru, c5_rv   : signed(8 downto 0) := (others => '0');
    signal c5_node        : unsigned(3 downto 0) := (others => '0');
    signal c5_inband      : std_logic := '0';
    signal c5_su, c5_sv   : std_logic := '0';
    signal c5_maxa        : unsigned(8 downto 0) := (others => '0');
    signal c5_face        : unsigned(2 downto 0) := to_unsigned(1, 3);
    signal c5_word        : t_word := (others => '0');

    signal c6_cdoct : unsigned(9 downto 0) := (others => '0');
    signal c6_pdoct : unsigned(9 downto 0) := (others => '0');
    signal c6_bev  : signed(16 downto 0) := (others => '0');
    signal c6_incorner : std_logic := '0';
    signal c6_edgein   : std_logic := '0';
    signal c6_active   : std_logic := '0';
    signal c6_inband   : std_logic := '0';
    signal c6_su, c6_sv : std_logic := '0';
    signal c6_maxa     : unsigned(8 downto 0) := (others => '0');
    signal c6_ru, c6_rv : signed(8 downto 0) := (others => '0');
    signal c6_word     : t_word := (others => '0');

    signal c7_indie, c7_inpip : std_logic := '0';
    signal c7_bev  : signed(16 downto 0) := (others => '0');
    signal c7_inband : std_logic := '0';
    signal c7_incorner : std_logic := '0';
    signal c7_shadow   : std_logic := '0';
    signal c7_pdoct : unsigned(9 downto 0) := (others => '0');
    signal c7_pr    : unsigned(7 downto 0) := (others => '0');
    signal c7_ru, c7_rv : signed(8 downto 0) := (others => '0');
    signal c7_word : t_word := (others => '0');

    -- c7b: dimple direction, raw shade, glint, colour unpack (offloads c8a).
    signal c7b_shraw  : signed(11 downto 0) := (others => '0');
    signal c7b_pipdot : signed(9 downto 0) := (others => '0');
    signal c7b_py     : unsigned(9 downto 0) := (others => '0');
    signal c7b_by, c7b_bu, c7b_bv : unsigned(9 downto 0) := C_MID;
    signal c7b_glint  : std_logic := '0';
    signal c7b_inpip, c7b_indie, c7b_shadow : std_logic := '0';
    signal c7b_incorner, c7b_inband : std_logic := '0';

    -- c8a: pre-computed shade / pip value / flags; c8b: final assembly.
    signal c8a_shade : signed(11 downto 0) := (others => '0');
    signal c8a_pipb  : signed(11 downto 0) := (others => '0');
    signal c8a_by, c8a_bu, c8a_bv : unsigned(9 downto 0) := C_MID;
    signal c8a_spec  : std_logic := '0';
    signal c8a_inpip, c8a_indie, c8a_shadow : std_logic := '0';

    signal c8_class : unsigned(1 downto 0) := (others => '0');  -- 0 tbl 1 shadow 2 body 3 pip
    signal c8_y, c8_u, c8_v : unsigned(9 downto 0) := C_MID;

    signal c9_y, c9_u, c9_v : unsigned(9 downto 0) := C_MID;

    signal s_io   : t_video_stream_yuv444_30b;
    signal s_bpass : t_video_stream_yuv444_30b;   -- 1-cycle raw passthrough (== SDK passthru)

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

begin

    u_sincos : entity work.sin_cos_full_lut_10x10
        port map (
            angle_in => std_logic_vector(lk4),
            sin_out  => lut_sin,
            cos_out  => lut_cos
        );

    --------------------------------------------------------------------------
    -- Raster counters + per-cell sample trigger + stable per-cell LFSR.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h, v_v : std_logic;
        variable v_trig   : std_logic;
        variable v_tp     : std_logic;
        variable v_su, v_sv : signed(11 downto 0);
        variable v_online : boolean;
        variable v_avy, v_avu, v_avv : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;

            v_h := '0'; v_v := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v := '1'; end if;
            r_anchor <= '0';

            if v_h = '1' then
                xloc <= (others => '0');
                xidx <= (others => '0');
            elsif data_in.avid = '1' then
                if xloc = s_dm1 then
                    xloc <= (others => '0');
                    xidx <= xidx + 1;
                else
                    xloc <= xloc + 1;
                end if;
            end if;

            if v_v = '1' then
                celly_loc <= (others => '0');
                celly_par <= '0';
                frame_act <= '0';
            elsif data_in.avid = '1' and frame_act = '0' then
                frame_act <= '1';
                celly_loc <= (others => '0');
                celly_par <= '0';
                line0     <= '1';
                r_anchor  <= '1';
            elsif v_h = '1' then
                line0 <= '0';
                if celly_loc = s_dm1 then
                    celly_loc <= (others => '0');
                    celly_par <= not celly_par;
                else
                    celly_loc <= celly_loc + 1;
                end if;
            end if;

            if v_h = '1' or r_anchor = '1' then
                dyc_line <= signed(resize(celly_loc, 10)) - signed(resize(s_dh, 10));
            end if;

            -- 8px horizontal area-average around each cell centre on the sample
            -- line (memoryless denoise -> far fewer frame-to-frame face flips
            -- from video noise; a point sample flickers, an average is stable).
            v_online := frame_act = '1' and data_in.avid = '1'
                        and (line0 = '1' or celly_loc = s_dh);
            if v_online and xloc = s_dhlo then
                acc_y <= resize(unsigned(data_in.y), 13);
                acc_u <= resize(unsigned(data_in.u), 13);
                acc_v <= resize(unsigned(data_in.v), 13);
            elsif v_online and xloc > s_dhlo and xloc <= s_dh then
                acc_y <= acc_y + unsigned(data_in.y);
                acc_u <= acc_u + unsigned(data_in.u);
                acc_v <= acc_v + unsigned(data_in.v);
            end if;

            -- strobe one pixel after the 8px window closes
            v_trig := '0';
            v_tp   := not celly_par;
            if v_online and xloc = s_dhi then
                if line0 = '1' then v_tp := '0'; end if;
                v_trig := '1';
            end if;

            w0_stb <= v_trig;
            w0_par <= v_tp;
            w0_idx <= xidx;
            if v_trig = '1' then
                v_avy := resize(shift_right(acc_y, 3), 10);
                v_avu := resize(shift_right(acc_u, 3), 10);
                v_avv := resize(shift_right(acc_v, 3), 10);
                w0_u <= v_avu;
                w0_v <= v_avv;
                v_su := signed(resize(v_avu, 12)) - 512;
                v_sv := signed(resize(v_avv, 12)) - 512;
                if s_src = '1' then
                    w0_drv <= resize(unsigned(abs(v_su)), 11) + resize(unsigned(abs(v_sv)), 11);
                else
                    w0_drv <= '0' & v_avy;
                end if;
            end if;

            if v_v = '1' then
                lfsr <= x"ACE1";
            elsif v_trig = '1' then
                lfsr <= lfsr(14 downto 0) & (lfsr(15) xor lfsr(13) xor lfsr(12) xor lfsr(10));
            end if;
            w0_rr  <= signed(resize(lfsr(7 downto 0), 9))  - 128;
            w0_rjx <= signed(resize(lfsr(11 downto 4), 9)) - 128;
            w0_rjy <= signed(resize(lfsr(15 downto 8), 9)) - 128;
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Sample pipe.
    --------------------------------------------------------------------------
    p_sample : process(clk)
        variable v_lvl  : integer range 0 to 5;
        variable v_face : integer range 1 to 6;
        variable v_sin  : signed(17 downto 0);
        variable v_jx, v_jy : signed(17 downto 0);
        variable v_dl   : signed(19 downto 0);
        variable v_r    : signed(9 downto 0);
        variable v_rc, v_rin, v_p, v_pr : unsigned(7 downto 0);
        variable v_rc2, v_pr2 : unsigned(13 downto 0);
        variable v_by, v_bu, v_bv : unsigned(9 downto 0);
        variable v_hi   : integer range 0 to 15;
        variable v_du, v_dv : signed(10 downto 0);
        variable v_au, v_av : unsigned(10 downto 0);
        variable v_oct  : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            ----------------------------------------------------------------
            -- W1: face (thresholds), the three scaling multiplies, octant.
            ----------------------------------------------------------------
            v_lvl := 0;
            if w0_drv >= s_t1 then v_lvl := v_lvl + 1; end if;
            if w0_drv >= s_t2 then v_lvl := v_lvl + 1; end if;
            if w0_drv >= s_t3 then v_lvl := v_lvl + 1; end if;
            if w0_drv >= s_t4 then v_lvl := v_lvl + 1; end if;
            if w0_drv >= s_t5 then v_lvl := v_lvl + 1; end if;
            if s_pol = '1' then v_face := 1 + v_lvl; else v_face := 6 - v_lvl; end if;
            w1_face <= to_unsigned(v_face, 3);

            w1_rotp <= resize(w0_rr  * signed('0' & s_rotsc), 18);
            w1_jxp  <= resize(w0_rjx * signed('0' & s_jitsc), 18);
            w1_jyp  <= resize(w0_rjy * signed('0' & s_jitsc), 18);
            w1_lsp  <= resize((signed(resize(w0_drv, 12)) - 512) * signed(resize(s_lsg, 7)), 20);

            v_du := signed(resize(w0_u, 11)) - 512;
            v_dv := signed(resize(w0_v, 11)) - 512;
            v_au := unsigned(abs(v_du));
            v_av := unsigned(abs(v_dv));
            if v_du >= 0 and v_dv >= 0 then
                if v_au > v_av then v_oct := 0; else v_oct := 4; end if;
            elsif v_du < 0 and v_dv >= 0 then
                if v_au > v_av then v_oct := 8; else v_oct := 4; end if;
            elsif v_du < 0 and v_dv < 0 then
                if v_au > v_av then v_oct := 8; else v_oct := 12; end if;
            else
                if v_au > v_av then v_oct := 0; else v_oct := 12; end if;
            end if;
            w1_oct  <= to_unsigned(v_oct, 4);
            w1_rrlo <= unsigned(w0_rr(3 downto 0));

            w1_stb <= w0_stb;  w1_par <= w0_par;  w1_idx <= w0_idx;
            w1_u <= w0_u;  w1_v <= w0_v;  w1_drv <= w0_drv;

            ----------------------------------------------------------------
            -- W2: finalize sin / jitter / radius (luma-size) / hue index.
            ----------------------------------------------------------------
            v_sin := shift_right(resize(w1_rotp, 18), 8);
            if    v_sin >  255 then v_sin := to_signed( 255, 18);
            elsif v_sin < -255 then v_sin := to_signed(-255, 18); end if;
            w2_sin <= resize(v_sin, 10);
            w2_jx  <= resize(shift_right(w1_jxp, 8), 8);
            w2_jy  <= resize(shift_right(w1_jyp, 8), 8);

            if s_lsize = '1' then
                v_r := signed(resize(s_r0, 10)) + resize(shift_right(w1_lsp, 7), 10);
                if v_r < 6 then v_r := to_signed(6, 10); end if;
                if v_r > signed('0' & s_rmax) then v_r := resize(signed('0' & s_rmax), 10); end if;
            else
                v_r := signed(resize(s_r0, 10));
            end if;
            w2_r <= unsigned(v_r(7 downto 0));

            if s_idx = 6 then
                v_hi := to_integer((w1_rrlo + s_hueoff) and "1111");
            else
                v_hi := to_integer((w1_oct + s_hueoff) and "1111");
            end if;
            w2_hidx <= to_unsigned(v_hi, 4);

            w2_face <= w1_face;
            w2_stb <= w1_stb;  w2_par <= w1_par;  w2_idx <= w1_idx;
            w2_u <= w1_u;  w2_v <= w1_v;

            ----------------------------------------------------------------
            -- W3: per-cell squared geometry + body colour.
            ----------------------------------------------------------------
            v_rc  := "00" & w2_r(7 downto 2);
            w3_rin <= w2_r - ("00" & w2_r(7 downto 2));
            w3_p   <= '0'  & w2_r(7 downto 1);
            w3_pr  <= ("000" & w2_r(7 downto 3)) + ("0000" & w2_r(7 downto 4));  -- ~0.19R
            w3_rc  <= v_rc;                                                       -- R/4
            w3_r   <= w2_r;

            v_by := to_unsigned(SCH_Y(to_integer(s_idx)), 10);
            v_bu := to_unsigned(SCH_U(to_integer(s_idx)), 10);
            v_bv := to_unsigned(SCH_V(to_integer(s_idx)), 10);
            if s_idx = 6 or s_idx = 7 then
                v_bu := to_unsigned(HUE_U(to_integer(w2_hidx)), 10);
                v_bv := to_unsigned(HUE_V(to_integer(w2_hidx)), 10);
            end if;
            if s_inkvid = '1' then
                v_bu := w2_u;  v_bv := w2_v;
            end if;
            w3_by <= v_by;  w3_bu <= v_bu;  w3_bv <= v_bv;

            w3_face <= w2_face;  w3_sin <= w2_sin;  w3_jx <= w2_jx;  w3_jy <= w2_jy;
            w3_stb <= w2_stb;  w3_par <= w2_par;  w3_idx <= w2_idx;

            ----------------------------------------------------------------
            -- W4: pack.
            ----------------------------------------------------------------
            w4_word <= std_logic_vector(w3_face)
                     & std_logic_vector(w3_sin)
                     & std_logic_vector(w3_jx)
                     & std_logic_vector(w3_jy)
                     & std_logic_vector(w3_r)
                     & std_logic_vector(w3_rin)
                     & std_logic_vector(w3_p)
                     & std_logic_vector(w3_pr)
                     & std_logic_vector(w3_rc)
                     & std_logic_vector(w3_by)
                     & std_logic_vector(w3_bu)
                     & std_logic_vector(w3_bv)
                     & s_pol;
            w4_stb <= w3_stb;  w4_par <= w3_par;  w4_idx <= w3_idx;
        end if;
    end process p_sample;

    --------------------------------------------------------------------------
    -- Cell BRAMs.
    --------------------------------------------------------------------------
    p_buf0 : process(clk)
    begin
        if rising_edge(clk) then
            if w4_stb = '1' and w4_par = '0' then
                buf0(to_integer(w4_idx)) <= w4_word;
            end if;
            rd0 <= buf0(to_integer(xidx));
        end if;
    end process p_buf0;

    p_buf1 : process(clk)
    begin
        if rising_edge(clk) then
            if w4_stb = '1' and w4_par = '1' then
                buf1(to_integer(w4_idx)) <= w4_word;
            end if;
            rd1 <= buf1(to_integer(xidx));
        end if;
    end process p_buf1;

    --------------------------------------------------------------------------
    -- Frame FSM.
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_d   : integer range 0 to 255;
        variable v_sp  : integer range 0 to 300;
        variable v_gl  : integer range 0 to 1023;
        variable v_r0  : integer range -256 to 255;
    begin
        if rising_edge(clk) then
            mp <= ma * mb;

            if prev_vsync_n = '1' and data_in.vsync_n = '0' then
                lk1 <=unsigned(registers_in(0)(9 downto 0));
                lk2 <=unsigned(registers_in(1)(9 downto 0));
                lk3 <=unsigned(registers_in(2)(9 downto 0));
                lk4 <=unsigned(registers_in(3)(9 downto 0));
                lk5 <=unsigned(registers_in(4)(9 downto 0));
                lk6 <=unsigned(registers_in(5)(9 downto 0));
                lp12<=unsigned(registers_in(7)(9 downto 0));
                s_src    <= registers_in(6)(0);
                s_lsize  <= registers_in(6)(1);
                s_tabvid <= registers_in(6)(2);
                s_inkvid <= registers_in(6)(3);
                s_bypass <= registers_in(6)(4);
                fsm_t <= (others => '0');
            elsif fsm_t /= x"FF" then
                fsm_t <= fsm_t + 1;
                case to_integer(fsm_t) is
                    when 0 =>
                        v_d := 18 + to_integer(lk1(9 downto 3));
                        if v_d > 150 then v_d := 150; end if;
                        s_d   <= to_unsigned(v_d, 8);
                        s_idx <= lk2(9 downto 7);
                        s_pol <= SCH_POL(to_integer(lk2(9 downto 7)));
                        s_hueoff <= lk3(9 downto 6);
                        s_rotsc  <= '0' & lp12(9 downto 3);
                        s_shad   <= '0' & lp12(9 downto 3);
                        v_sp := 34 + to_integer(lk6(9 downto 4));
                        s_t3 <= C_MID;
                        s_t2 <= to_unsigned(512 - v_sp, 10);
                        s_t4 <= to_unsigned(512 + v_sp, 10);
                        s_t1 <= to_unsigned(512 - 2*v_sp, 10);
                        s_t5 <= to_unsigned(512 + 2*v_sp, 10);
                        -- Gloss is the shade clamp: matte (tight) -> deep (wide).
                        -- The pixel path uses a fixed bevel shift (no barrel).
                        v_gl := to_integer(lk5(9 downto 0));
                        s_span <= to_unsigned(70 + (v_gl / 4), 9);        -- 70..325
                        if lk5 >= 620 then s_glossy <= '1'; else s_glossy <= '0'; end if;
                    when 1 =>
                        s_dm1 <= s_d - 1;
                        s_dh  <= '0' & s_d(7 downto 1);
                        ma <= signed(resize(lp12, 18));
                        mb <= signed(resize('0' & s_d(7 downto 2), 18));   -- Dh/2 = D/4
                    when 2 =>
                        s_dhlo <= s_dh - 7;   -- precomputed denoise window bounds
                        s_dhi  <= s_dh + 1;   -- (keeps the arithmetic out of p_position)
                    when 4 =>
                        v_r0 := to_integer(s_dh) - 2 - to_integer(unsigned(mp(17 downto 10)));
                        if v_r0 < 6 then v_r0 := 6; end if;
                        s_r0 <= to_unsigned(v_r0, 8);
                    when 5 =>
                        if s_r0 < 8 then s_bw <= to_unsigned(2, 8);
                        else s_bw <= "00" & s_r0(7 downto 2); end if;
                        s_rmax <= s_dh - 1;
                        ma <= signed(resize(s_dh - s_r0, 18));
                        mb <= signed(resize(lp12, 18));
                    when 8 =>
                        s_jitsc <= resize(unsigned(mp(17 downto 11)), 8);
                    when others => null;
                end case;
            end if;

            lx <= resize(shift_right(lut_cos, 2), 8);   -- cos/4  (+/-127)
            ly <= resize(shift_right(lut_sin, 2), 8);   -- sin/4
        end if;
    end process p_frame;

    --------------------------------------------------------------------------
    -- Pixel pipeline.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_word : t_word;
        variable v_u, v_v : signed(9 downto 0);
        variable v_absu, v_absv : unsigned(9 downto 0);
        variable v_r, v_rin, v_p, v_ph : unsigned(7 downto 0);
        variable v_au, v_av : unsigned(8 downto 0);
        variable v_gu, v_gv : integer range -1 to 1;
        variable v_gup, v_gvp : signed(9 downto 0);
        variable v_maxa : unsigned(8 downto 0);
        variable v_shade, v_shraw : signed(11 downto 0);
        variable v_by, v_bu, v_bv : unsigned(9 downto 0);
        variable v_py : unsigned(9 downto 0);
        variable v_yo, v_uo, v_vo : signed(11 downto 0);
        variable v_pol  : std_logic;
        variable v_mask : std_logic_vector(8 downto 0);
        variable v_bgy, v_bgu, v_bgv : unsigned(9 downto 0);
        variable v_glint : std_logic;
        variable v_t1, v_t2, v_pipdot : signed(9 downto 0);
        variable v_pipb : signed(11 downto 0);
        variable v_ru9, v_rv9 : unsigned(8 downto 0);
        variable v_aru, v_arv, v_acu, v_acv : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            ----------------------------------------------------------------
            -- c1
            ----------------------------------------------------------------
            c1_dx0 <= signed(resize(xloc, 10)) - signed(resize(s_dh, 10));
            c1_par <= celly_par;

            ----------------------------------------------------------------
            -- c2: select word, apply jitter.
            ----------------------------------------------------------------
            if c1_par = '1' then v_word := rd1; else v_word := rd0; end if;
            c2_word <= v_word;
            c2_dxj  <= c1_dx0   - resize(signed(v_word(86 downto 79)), 10);   -- - jx
            c2_dyj  <= dyc_line - resize(signed(v_word(78 downto 71)), 10);   -- - jy

            ----------------------------------------------------------------
            -- c3: rotation products (sin at 96..87).
            ----------------------------------------------------------------
            -- 8x9 rotation mult: dxj/dyj fit +/-127 (die fits its cell), sin +/-255.
            c3_rsx  <= resize(c2_dxj, 8) * resize(signed(c2_word(96 downto 87)), 9);
            c3_rsy  <= resize(c2_dyj, 8) * resize(signed(c2_word(96 downto 87)), 9);
            c3_dxj  <= c2_dxj;
            c3_dyj  <= c2_dyj;
            c3_word <= c2_word;

            ----------------------------------------------------------------
            -- c4: u,v (small-angle) + magnitudes.
            ----------------------------------------------------------------
            v_u := c3_dxj + resize(shift_right(c3_rsy, 9), 10);
            v_v := c3_dyj - resize(shift_right(c3_rsx, 9), 10);
            c4_u <= v_u;  c4_v <= v_v;
            v_absu := unsigned(abs(v_u));
            v_absv := unsigned(abs(v_v));
            c4_au <= v_absu(8 downto 0);
            c4_av <= v_absv(8 downto 0);
            -- clamp to +/-127 for the bevel mult (die interior |u|,|v| <= R < 127,
            -- so lossless where it shows; keeps the multiply 8x8).
            if    v_u >  127 then c4_ub <= to_signed( 127, 8);
            elsif v_u < -128 then c4_ub <= to_signed(-128, 8);
            else                  c4_ub <= resize(v_u, 8); end if;
            if    v_v >  127 then c4_vb <= to_signed( 127, 8);
            elsif v_v < -128 then c4_vb <= to_signed(-128, 8);
            else                  c4_vb <= resize(v_v, 8); end if;
            c4_word <= c3_word;

            ----------------------------------------------------------------
            -- c5
            ----------------------------------------------------------------
            v_word := c4_word;
            v_r    := unsigned(v_word(70 downto 63));
            v_rin  := unsigned(v_word(62 downto 55));
            v_p    := unsigned(v_word(54 downto 47));
            v_ph   := '0' & v_p(7 downto 1);
            v_au   := c4_au;  v_av := c4_av;

            if v_au <= ('0' & v_r) and v_av <= ('0' & v_r) then
                c5_edgein <= '1'; else c5_edgein <= '0'; end if;
            if v_au > ('0' & v_rin) and v_av > ('0' & v_rin) then
                c5_incorner <= '1'; else c5_incorner <= '0'; end if;
            if v_au > ('0' & v_rin) then c5_cu <= v_au - ('0' & v_rin); else c5_cu <= (others=>'0'); end if;
            if v_av > ('0' & v_rin) then c5_cv <= v_av - ('0' & v_rin); else c5_cv <= (others=>'0'); end if;

            c5_bx <= c4_ub * lx;
            c5_by <= c4_vb * ly;

            if    c4_u < -signed('0' & v_ph) then v_gu := -1;
            elsif c4_u >  signed('0' & v_ph) then v_gu :=  1;
            else  v_gu := 0; end if;
            if    c4_v < -signed('0' & v_ph) then v_gv := -1;
            elsif c4_v >  signed('0' & v_ph) then v_gv :=  1;
            else  v_gv := 0; end if;
            case v_gu is
                when -1     => v_gup := resize(-signed('0' & v_p), 10);
                when  1     => v_gup := resize( signed('0' & v_p), 10);
                when others => v_gup := (others => '0');
            end case;
            case v_gv is
                when -1     => v_gvp := resize(-signed('0' & v_p), 10);
                when  1     => v_gvp := resize( signed('0' & v_p), 10);
                when others => v_gvp := (others => '0');
            end case;
            c5_ru <= resize(c4_u - v_gup, 9);
            c5_rv <= resize(c4_v - v_gvp, 9);
            c5_node <= to_unsigned((v_gv + 1) * 3 + (v_gu + 1), 4);
            c5_face <= unsigned(v_word(CW downto CW-2));

            if v_au > v_av then v_maxa := v_au; else v_maxa := v_av; end if;
            c5_maxa <= v_maxa;
            -- edge band flag only (bevel is whole-face; the flag just gates the
            -- corner specular) -> a compare, no subtract.
            if v_maxa > (v_r - s_bw) then c5_inband <= '1'; else c5_inband <= '0'; end if;
            c5_su <= c4_u(9);
            c5_sv <= c4_v(9);
            c5_word <= v_word;

            ----------------------------------------------------------------
            -- c6
            ----------------------------------------------------------------
            -- octagonal distance = max(a,b) + min(a,b)/2 (adds/shifts only), a
            -- near-circular metric -> round pips and rounded corners with no
            -- distance-squared multiply.
            v_ru9 := unsigned(abs(c5_ru));  v_rv9 := unsigned(abs(c5_rv));
            if v_ru9 >= v_rv9 then c6_pdoct <= resize(v_ru9 + ('0' & v_rv9(8 downto 1)), 10);
            else                   c6_pdoct <= resize(v_rv9 + ('0' & v_ru9(8 downto 1)), 10); end if;
            if c5_cu >= c5_cv then c6_cdoct <= resize(c5_cu + ('0' & c5_cv(8 downto 1)), 10);
            else                   c6_cdoct <= resize(c5_cv + ('0' & c5_cu(8 downto 1)), 10); end if;
            c6_bev <= resize(c5_bx, 17) + resize(c5_by, 17);
            v_mask := facemask(c5_face);
            c6_active   <= v_mask(to_integer(c5_node));
            c6_incorner <= c5_incorner;
            c6_edgein   <= c5_edgein;
            c6_inband   <= c5_inband;
            c6_su <= c5_su;  c6_sv <= c5_sv;  c6_maxa <= c5_maxa;
            c6_ru <= c5_ru;  c6_rv <= c5_rv;
            c6_word <= c5_word;

            ----------------------------------------------------------------
            -- c7
            ----------------------------------------------------------------
            v_word := c6_word;
            v_r    := unsigned(v_word(70 downto 63));
            if c6_edgein = '1' and (c6_incorner = '0' or c6_cdoct <= unsigned(v_word(38 downto 31))) then
                c7_indie <= '1'; else c7_indie <= '0'; end if;
            if c6_active = '1' and c6_pdoct <= unsigned(v_word(46 downto 39)) then
                c7_inpip <= '1'; else c7_inpip <= '0'; end if;
            if c6_su = '0' and c6_sv = '0'
               and signed('0' & c6_maxa) > signed('0' & v_r)
               and signed('0' & c6_maxa) <= (signed('0' & v_r) + signed('0' & s_shad(7 downto 1))) then
                c7_shadow <= '1'; else c7_shadow <= '0'; end if;
            c7_bev <= c6_bev;
            c7_inband <= c6_inband;
            c7_incorner <= c6_incorner;
            c7_pdoct <= c6_pdoct;
            c7_pr    <= unsigned(v_word(46 downto 39));
            c7_ru <= c6_ru;  c7_rv <= c6_rv;
            c7_word <= v_word;

            ----------------------------------------------------------------
            -- c7b: dimple direction, raw shade, glint, colour unpack.
            ----------------------------------------------------------------
            v_word := c7_word;
            c7b_by <= unsigned(v_word(30 downto 21));
            c7b_bu <= unsigned(v_word(20 downto 11));
            c7b_bv <= unsigned(v_word(10 downto 1));
            if v_word(0) = '1' then c7b_py <= to_unsigned(880, 10); else c7b_py <= to_unsigned(55, 10); end if;

            c7b_shraw <= resize(shift_right(c7_bev, 6), 12);

            if lx(7) = '0' then v_t1 := resize(c7_ru, 10); else v_t1 := resize(-c7_ru, 10); end if;
            if ly(7) = '0' then v_t2 := resize(c7_rv, 10); else v_t2 := resize(-c7_rv, 10); end if;
            v_pipdot := resize(v_t1 + v_t2, 10);
            c7b_pipdot <= v_pipdot;
            if c7_pdoct > (c7_pr - ('0' & c7_pr(7 downto 2))) and v_pipdot < -2 then
                c7b_glint <= '1'; else c7b_glint <= '0'; end if;

            c7b_inpip <= c7_inpip;  c7b_indie <= c7_indie;  c7b_shadow <= c7_shadow;
            c7b_incorner <= c7_incorner;  c7b_inband <= c7_inband;

            ----------------------------------------------------------------
            -- c8a: shade clamp / pip value / specular flag (shallow).
            ----------------------------------------------------------------
            v_shraw := c7b_shraw;
            v_shade := v_shraw;
            if v_shraw >  signed('0' & s_span) then v_shade := resize( signed('0' & s_span), 12); end if;
            if v_shraw < -signed('0' & s_span) then v_shade := resize(-signed('0' & s_span), 12); end if;
            c8a_shade <= v_shade;

            v_pipb := signed(resize(c7b_py, 12)) - resize(shift_left(c7b_pipdot, 3), 12);
            if c7b_glint = '1' then v_pipb := v_pipb + 260; end if;
            c8a_pipb <= v_pipb;

            if s_glossy = '1' and c7b_incorner = '1' and c7b_inband = '1'
               and v_shraw >= signed('0' & s_span) - 6 then
                c8a_spec <= '1'; else c8a_spec <= '0'; end if;

            c8a_by <= c7b_by;  c8a_bu <= c7b_bu;  c8a_bv <= c7b_bv;
            c8a_inpip <= c7b_inpip;  c8a_indie <= c7b_indie;  c8a_shadow <= c7b_shadow;

            ----------------------------------------------------------------
            -- c8b: assemble + single final clamp.
            ----------------------------------------------------------------
            if c8a_inpip = '1' then
                v_yo := c8a_pipb;
                v_uo := to_signed(512, 12);
                v_vo := to_signed(512, 12);
                c8_class <= "11";
            elsif c8a_indie = '1' then
                v_yo := signed(resize(c8a_by, 12)) + resize(c8a_shade, 12);
                if c8a_spec = '1' then v_yo := v_yo + 300; end if;
                v_uo := signed(resize(c8a_bu, 12));
                v_vo := signed(resize(c8a_bv, 12));
                c8_class <= "10";
            else
                v_yo := (others => '0');
                v_uo := to_signed(512, 12);
                v_vo := to_signed(512, 12);
                if c8a_shadow = '1' then c8_class <= "01"; else c8_class <= "00"; end if;
            end if;
            if v_yo < 0 then v_yo := (others => '0'); end if;
            if v_yo > 1023 then v_yo := to_signed(1023, 12); end if;
            c8_y <= unsigned(v_yo(9 downto 0));
            c8_u <= unsigned(v_uo(9 downto 0));
            c8_v <= unsigned(v_vo(9 downto 0));

            ----------------------------------------------------------------
            -- c9: compose over table.
            ----------------------------------------------------------------
            if s_tabvid = '1' then
                v_bgy := resize(unsigned(pipe(10).y(9 downto 1)), 10) + 40;
                v_bgu := unsigned(pipe(10).u);
                v_bgv := unsigned(pipe(10).v);
            else
                v_bgy := to_unsigned(FELT_Y, 10);
                v_bgu := to_unsigned(FELT_U, 10);
                v_bgv := to_unsigned(FELT_V, 10);
            end if;

            case to_integer(c8_class) is
                when 2 | 3 =>
                    c9_y <= c8_y;  c9_u <= c8_u;  c9_v <= c8_v;
                when 1 =>
                    c9_y <= '0' & v_bgy(9 downto 1);
                    c9_u <= v_bgu;  c9_v <= v_bgv;
                when others =>
                    c9_y <= v_bgy;  c9_u <= v_bgu;  c9_v <= v_bgv;
            end case;

            ----------------------------------------------------------------
            -- c10: processed output (sync latency-aligned to c9 content).
            -- Bypass is a SEPARATE 1-cycle raw forward (s_bpass), muxed at the
            -- output below, so S11 is byte- and timing-identical to SDK passthru.
            ----------------------------------------------------------------
            s_io.y <= std_logic_vector(c9_y);
            s_io.u <= std_logic_vector(c9_u);
            s_io.v <= std_logic_vector(c9_v);
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= pipe(LATENCY - 1).avid;
            s_io.field_n <= pipe(LATENCY - 1).field_n;

            -- 1-cycle raw passthrough (exactly SDK passthru: data_out <= data_in)
            s_bpass <= data_in;
        end if;
    end process p_pipe;

    -- S11 bypass forwards the raw input untouched (like passthru); otherwise the
    -- rendered dice.  Mux at the output so bypass never inherits the pipeline.
    data_out.y       <= s_bpass.y       when s_bypass = '1' else s_io.y;
    data_out.u       <= s_bpass.u       when s_bypass = '1' else s_io.u;
    data_out.v       <= s_bpass.v       when s_bypass = '1' else s_io.v;
    data_out.hsync_n <= s_bpass.hsync_n when s_bypass = '1' else s_io.hsync_n;
    data_out.vsync_n <= s_bpass.vsync_n when s_bypass = '1' else s_io.vsync_n;
    data_out.avid    <= s_bpass.avid    when s_bypass = '1' else s_io.avid;
    data_out.field_n <= s_bpass.field_n when s_bypass = '1' else s_io.field_n;

end architecture rollthebones;
