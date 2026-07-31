-- pipedream.vhd  (v2.0 — a generative network of glossy 3D pipes)
--
-- No video in: this is a SYNTHESIS program.  The screen is tiled by an
-- invisible cell grid; each cell decides which of its four edges carry a pipe
-- by HASHING that edge.  Because an edge is shared by two cells and both hash
-- the same edge coordinate, neighbours always agree -- so the pipes connect by
-- construction, with no stored grid and no neighbour lookups.  The four
-- edge-bits give every join type for free: straight-throughs, elbows, tees and
-- crosses, plus rounded caps at dead-ends.  Connection DENSITY is a knob; the
-- network can also slowly reshuffle (Morph).
--
-- Each pipe is a rounded, specular-lit 3D tube (gumball's fake-Phong on the 1-D
-- cross-section: an offset-light field drives a bright diffuse band + a tight
-- white highlight), with a glossy ball at every bend / junction / cap.  Colour
-- is a bright, saturated hue generated straight into chroma -- a flowing
-- rainbow across the screen, or a distinct colour per cell -- drifting over
-- time.  Bright pipes on a near-black field.
--
-- No divide on the pixel path; the per-line dy and the ball's (dy-oy)^2 advance
-- incrementally (adds only).  Per-frame radius / light / range maths + two
-- serial divides run in a ~60-cycle FSM on one shared multiplier in vblank.
--
-- Controls:
--   K1  Scale      (pipe/cell size, 48..176 px)
--   K2  Connections(sparse strands -> dense mesh of tees & crosses)
--   K3  Fatness    (tube radius)
--   K4  Shine      (broad sheen -> pinpoint gloss)
--   K5  Hue        (base colour)
--   K6  Spread     (one solid colour -> full rainbow across the screen)
--   S7  Joints     (Sharp elbows / Beaded ball joints)
--   S8  Colour     (Rainbow flow / Per-cell)
--   S9  Morph      (Static network / slowly reshuffling)
--   S10 Light      (Fixed / Spinning highlight)
--   S11 Bypass     (show the input instead)
--   P12 Flow       (THE fader: bright liquid-light pulses coursing ALONG the
--                   pipes; max = the whole network alight)
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

architecture pipedream of program_top is

    constant LATENCY : natural := 17;  -- +3 for the rounded-elbow stages (c3b/c3c/c3d)
    constant C_MID : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_MAXCOL : natural := 48;                 -- <= this many cells wide

    function norm_shift(v : unsigned(15 downto 0)) return natural is
    begin
        for i in 15 downto 0 loop
            if v(i) = '1' then return 15 - i; end if;
        end loop;
        return 15;
    end function;

    function f_absu(v : signed) return unsigned is
    begin
        if v(v'high) = '1' then return unsigned(-v); else return unsigned(v); end if;
    end function;

    function f_b(b : boolean) return std_logic is
    begin
        if b then return '1'; else return '0'; end if;
    end function;

    -- Saturating highlight-offset map: clamp(4*d, +/-m).  Applied to the SAME
    -- (light . cross-section) projection d on straights (per-frame) and elbows
    -- (per-pixel), so the highlight sits at a CONSTANT +/-m offset -- concentric
    -- with the tube, following an elbow's curve -- except in a short transition
    -- where d crosses zero; junctions stay flush because both sides map the
    -- same continuous d.
    -- Takes d pre-scaled by 4 (keeps 1/4-px resolution through the gain so the
    -- transition sweep doesn't staircase in 4-px steps).
    function f_oclamp(d4 : signed(13 downto 0); m : unsigned(8 downto 0)) return signed is
        variable v : signed(13 downto 0);
        variable mm : signed(13 downto 0);
    begin
        v  := d4;
        mm := signed(resize(m, 14));
        if v > mm then v := mm; elsif v < -mm then v := -mm; end if;
        return resize(v, 12);
    end function;

    -- quarter-wave sine ROM (fireworks): sin/cos over 0..2pi, 0 EBR.
    type t_qsin is array(0 to 63) of signed(9 downto 0);
    constant C_QSIN : t_qsin := (
        to_signed(  0, 10), to_signed( 13, 10), to_signed( 25, 10), to_signed( 38, 10), to_signed( 50, 10), to_signed( 63, 10), to_signed( 75, 10), to_signed( 87, 10),
        to_signed(100, 10), to_signed(112, 10), to_signed(124, 10), to_signed(136, 10), to_signed(148, 10), to_signed(160, 10), to_signed(172, 10), to_signed(184, 10),
        to_signed(196, 10), to_signed(207, 10), to_signed(218, 10), to_signed(230, 10), to_signed(241, 10), to_signed(252, 10), to_signed(263, 10), to_signed(273, 10),
        to_signed(284, 10), to_signed(294, 10), to_signed(304, 10), to_signed(314, 10), to_signed(324, 10), to_signed(334, 10), to_signed(343, 10), to_signed(352, 10),
        to_signed(361, 10), to_signed(370, 10), to_signed(379, 10), to_signed(387, 10), to_signed(395, 10), to_signed(403, 10), to_signed(410, 10), to_signed(418, 10),
        to_signed(425, 10), to_signed(432, 10), to_signed(438, 10), to_signed(445, 10), to_signed(451, 10), to_signed(456, 10), to_signed(462, 10), to_signed(467, 10),
        to_signed(472, 10), to_signed(477, 10), to_signed(481, 10), to_signed(485, 10), to_signed(489, 10), to_signed(492, 10), to_signed(496, 10), to_signed(499, 10),
        to_signed(501, 10), to_signed(503, 10), to_signed(505, 10), to_signed(507, 10), to_signed(509, 10), to_signed(510, 10), to_signed(510, 10), to_signed(511, 10));

    function f_qsin(a : unsigned(9 downto 0)) return signed is
        variable v_i : unsigned(5 downto 0);
        variable v_t : signed(9 downto 0);
    begin
        if a(8) = '0' then v_i := a(7 downto 2); else v_i := not a(7 downto 2); end if;
        v_t := C_QSIN(to_integer(v_i));
        if a(9) = '1' then return -v_t; else return v_t; end if;
    end function;

    -- Cheap edge hash: mixes the edge's grid coords + a domain bit + seed into
    -- 8 bits with a couple of shift-adds and xor-folds (visual variety, not
    -- crypto).  Deterministic in (a,b,dom) so a shared edge hashes identically
    -- from either cell.
    -- Edge hash, SPLIT so the long mix is not one combinational chain: f_hraw
    -- is the linear add-tree (registered at c1), f_hfold the avalanche (c2).
    function f_hraw(a : unsigned(6 downto 0); b : unsigned(6 downto 0);
                    dom : std_logic; seed : unsigned(7 downto 0)) return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := (resize(a, 16) sll 5) + (resize(a, 16) sll 2) + resize(a, 16)
           + (resize(b, 16) sll 7) + (resize(b, 16) sll 4) + (resize(b, 16) sll 1)
           + resize(seed, 16);
        if dom = '1' then h := h + to_unsigned(16#B5A3#, 16); end if;
        return h;
    end function;

    function f_hfold(h : unsigned(15 downto 0)) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := h xor (h srl 7);
        v := v xor (v srl 4);
        return v(7 downto 0);
    end function;

    ----------------------------------------------------------------------------
    -- Raster position + animation phases.
    ----------------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal r_hedge      : std_logic := '0';
    signal r_anchor     : std_logic := '0';

    signal x_loc     : unsigned(7 downto 0) := (others => '0');
    signal x_idx     : unsigned(6 downto 0) := (others => '0');
    signal celly_loc : unsigned(7 downto 0) := (others => '0');
    signal celly_idx : unsigned(6 downto 0) := (others => '0');
    signal frame_act : std_logic := '0';
    signal x_f  : unsigned(11 downto 0) := (others => '0');
    signal y_ln : unsigned(10 downto 0) := (others => '0');
    signal y_f  : unsigned(11 downto 0) := (others => '0');

    signal s_fprev : std_logic := '0';
    signal s_ilace : std_logic := '0';
    signal s_fodd  : std_logic := '0';

    signal s_phase   : unsigned(15 downto 0) := (others => '0');  -- flow
    signal s_huep    : unsigned(15 downto 0) := (others => '0');  -- hue drift
    signal s_morphp  : unsigned(7 downto 0)  := (others => '0');  -- network reshuffle
    signal s_lightp  : unsigned(9 downto 0)  := (others => '0');  -- spinning light

    ----------------------------------------------------------------------------
    -- Per-frame parameters (latched at vsync, refined by the frame FSM).
    ----------------------------------------------------------------------------
    signal lk1, lk2, lk3, lk4, lk5, lk6, lp12 : unsigned(9 downto 0) := (others => '0');

    signal s_w     : unsigned(7 downto 0) := to_unsigned(96, 8);
    signal s_wm1   : unsigned(7 downto 0) := to_unsigned(95, 8);
    signal s_wh    : unsigned(7 downto 0) := to_unsigned(48, 8);
    signal s_rp    : unsigned(7 downto 0) := to_unsigned(96, 8);
    signal s_rpm1  : unsigned(7 downto 0) := to_unsigned(95, 8);
    signal s_rfrac : unsigned(10 downto 0) := to_unsigned(560, 11);
    signal s_r     : unsigned(7 downto 0) := to_unsigned(24, 8);
    signal s_ls    : unsigned(7 downto 0) := to_unsigned(15, 8);
    signal s_omag  : signed(8 downto 0) := to_signed(15, 9);
    signal s_es    : unsigned(7 downto 0) := to_unsigned(12, 8);
    signal s_oh, s_ov : signed(8 downto 0) := (others => '0');
    signal s_omax : unsigned(8 downto 0) := (others => '0');   -- max(|s_ov|,|s_oh|): shared highlight-offset magnitude
    signal s_ediff2 : unsigned(15 downto 0) := to_unsigned(1521, 16);
    signal s_espec2 : unsigned(15 downto 0) := to_unsigned(144, 16);
    signal s_sle   : unsigned(3 downto 0) := (others => '0');
    signal s_sls   : unsigned(3 downto 0) := (others => '0');
    signal s_kd8   : unsigned(7 downto 0) := (others => '0');
    signal s_ks9   : unsigned(9 downto 0) := (others => '0');
    signal s_amb   : unsigned(7 downto 0) := to_unsigned(120, 8);
    signal s_span  : unsigned(7 downto 0) := to_unsigned(135, 8);
    signal s_speff : unsigned(9 downto 0) := to_unsigned(420, 10);
    signal s_kshine : unsigned(10 downto 0) := to_unsigned(560, 11);
    signal s_lx, s_ly : signed(9 downto 0) := (others => '0');
    signal s_lx6, s_ly6 : signed(6 downto 0) := (others => '0');       -- narrowed light dir for the elbow dot-product

    signal s_thresh : unsigned(7 downto 0) := to_unsigned(150, 8);  -- edge density
    signal s_seed   : unsigned(7 downto 0) := to_unsigned(90, 8);
    signal s_hue    : unsigned(9 downto 0) := (others => '0');
    signal s_hshift : unsigned(2 downto 0) := to_unsigned(3, 3);    -- rainbow scale
    signal s_sat    : unsigned(9 downto 0) := to_unsigned(430, 10);  -- vivid chroma

    -- rounded-elbow per-frame constants (quarter-torus of radius wh, centred on
    -- the cell corner the bend wraps).  Membership = squared-distance band;
    -- shading = linearised radial distance (rho^2 - wh^2) * 1/(2wh).
    signal s_wh2   : unsigned(15 downto 0) := to_unsigned(2304, 16);   -- wh^2
    signal s_qlo   : unsigned(16 downto 0) := to_unsigned(576, 17);    -- (wh-r)^2
    signal s_qhi   : unsigned(16 downto 0) := to_unsigned(5184, 17);   -- (wh+r)^2
    signal s_krec  : unsigned(6 downto 0)  := to_unsigned(43, 7);      -- round(4096/(2wh)): etb = ((u>>6)*krec)>>6, exact at any scale
    signal s_twh   : unsigned(8 downto 0)  := to_unsigned(96, 9);      -- 2*wh
    signal s_w2    : unsigned(16 downto 0) := to_unsigned(9216, 17);   -- (2wh)^2  (qxE/qyS reset)
    signal s_w2o   : unsigned(16 downto 0) := to_unsigned(9025, 17);   -- (2wh-1)^2 (interlace-odd reset)

    signal s_flowspd : unsigned(9 downto 0) := (others => '0');
    signal s_floww   : unsigned(6 downto 0) := to_unsigned(24, 7);
    signal s_flowg   : unsigned(2 downto 0) := (others => '0');
    signal s_flowon  : std_logic := '0';
    signal s_huespd  : unsigned(9 downto 0) := to_unsigned(6, 10);

    signal s_round  : std_logic := '1';                -- S7 (1 = rounded elbows, 0 = square)
    signal s_percell : std_logic := '0';               -- S8 (1 = per-cell colour)
    signal s_morph  : std_logic := '0';                -- S9
    signal s_spin   : std_logic := '0';                -- S10
    signal s_bypass : std_logic := '0';                -- S11

    signal fsm_t   : unsigned(7 downto 0) := (others => '1');
    signal ma, mb  : signed(12 downto 0) := (others => '0');
    signal mp      : signed(25 downto 0) := (others => '0');
    signal div_run : std_logic := '0';
    signal div_cnt : unsigned(4 downto 0) := (others => '0');
    signal div_d   : unsigned(17 downto 0) := (others => '0');
    signal div_q   : unsigned(17 downto 0) := (others => '0');
    signal div_rem : unsigned(8 downto 0)  := (others => '0');
    signal div_v   : unsigned(7 downto 0)  := (others => '1');

    ----------------------------------------------------------------------------
    -- Per-line dy and ball (dy-oy)^2, incremental at each h edge.
    ----------------------------------------------------------------------------
    signal lm_dy   : signed(9 downto 0) := (others => '0');

    -- Corner-distance squares for the rounded elbow, kept incrementally (adds
    -- only, NO multiply): qxW/qxE = (dx+wh)^2 / (dx-wh)^2 per pixel; qyN/qyS =
    -- (dy+wh)^2 / (dy-wh)^2 per line.  rho^2 to the bend corner = pick one of
    -- each by the cell's ports and add.
    signal qxW, qxE : unsigned(15 downto 0) := (others => '0');
    signal qyN, qyS : unsigned(15 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- Pixel pipeline.
    ----------------------------------------------------------------------------
    signal g0_dx   : signed(9 downto 0) := (others => '0');
    signal g0_dy   : signed(9 downto 0) := (others => '0');
    signal g0_xf, g0_yf : unsigned(11 downto 0) := (others => '0');
    signal g0_hN, g0_hS, g0_hE, g0_hW : unsigned(15 downto 0) := (others => '0');
    signal g0_ang  : unsigned(9 downto 0) := (others => '0');
    signal g0_qxw, g0_qxe, g0_qyn, g0_qys : unsigned(15 downto 0) := (others => '0');

    signal p2_absdx, p2_absdy : unsigned(9 downto 0) := (others => '0');
    signal p2_dx, p2_dy : signed(9 downto 0) := (others => '0');
    signal p2_xf, p2_yf : unsigned(11 downto 0) := (others => '0');
    signal p2_mask : std_logic_vector(3 downto 0) := (others => '0');  -- N S E W

    signal p2_qxw, p2_qxe, p2_qyn, p2_qys : unsigned(15 downto 0) := (others => '0');

    signal p3_hit  : std_logic := '0';
    signal p3_et   : signed(11 downto 0) := (others => '0');
    signal p3_along : signed(12 downto 0) := (others => '0');


    -- rounded-elbow: rho^2 to the bend corner (c3, picked from the accumulators),
    -- then the two inserted stages (u = rho^2-wh^2, then the isolated linearise).
    signal p3_rho2 : unsigned(16 downto 0) := (others => '0');
    signal p3_elb  : std_logic := '0';
    signal p3_excx, p3_eycy : signed(9 downto 0) := (others => '0');   -- corner-relative coords (radial dir)
    signal p3b_mx, p3b_my   : signed(16 downto 0) := (others => '0');  -- light . radial  (dot-product terms)

    signal p3b_ehit  : std_logic := '0';                         -- rho^2 band test (done at c3b)
    signal p3b_u     : signed(17 downto 0) := (others => '0');   -- rho^2 - wh^2
    signal p3b_et    : signed(11 downto 0) := (others => '0');
    signal p3b_hit, p3b_elb : std_logic := '0';
    signal p3b_along : signed(12 downto 0) := (others => '0');


    signal p3c_et    : signed(11 downto 0) := (others => '0');   -- straight-tube cross-section (carried)
    signal p3c_ets   : signed(11 downto 0) := (others => '0');   -- elbow cross-section, scaled to px
    signal p3c_do    : signed(11 downto 0) := (others => '0');   -- raw x4 light projection (pre-clamp)
    signal p3c_elb, p3c_ehit, p3c_hit : std_logic := '0';
    signal p3c_along : signed(12 downto 0) := (others => '0');

    signal p3d_et    : signed(11 downto 0) := (others => '0');
    signal p3d_hit   : std_logic := '0';
    signal p3d_along : signed(12 downto 0) := (others => '0');


    signal p4_e2   : unsigned(16 downto 0) := (others => '0');
    signal p5_dl   : unsigned(15 downto 0) := (others => '0');
    signal p5_ds   : unsigned(15 downto 0) := (others => '0');
    signal p6_dlc  : unsigned(15 downto 0) := (others => '0');
    signal p6_dsc  : unsigned(15 downto 0) := (others => '0');
    signal p7_dlb  : unsigned(7 downto 0) := (others => '0');
    signal p7_dsb  : unsigned(7 downto 0) := (others => '0');
    signal p8_pl   : unsigned(15 downto 0) := (others => '0');
    signal p8_ps   : unsigned(17 downto 0) := (others => '0');
    signal p8_flow : unsigned(9 downto 0) := (others => '0');
    signal p9_l    : unsigned(7 downto 0) := (others => '0');
    signal p9_s    : unsigned(9 downto 0) := (others => '0');
    signal p9_flow : unsigned(9 downto 0) := (others => '0');
    signal p10_cf  : unsigned(7 downto 0) := (others => '0');
    signal p10_y   : unsigned(9 downto 0) := (others => '0');
    signal p11_um, p11_vm : signed(19 downto 0) := (others => '0');
    signal p11_y   : unsigned(9 downto 0) := (others => '0');
    signal p12_y, p12_u, p12_v : unsigned(9 downto 0) := C_MID;

    -- carry pipes (from c3)
    type t_c10 is array (natural range <>) of unsigned(9 downto 0);
    -- hue rides the pipe as its 10-bit ANGLE and becomes chroma only at the
    -- end (qsin at c9, saturation multiply + clamp at c10): one carried value
    -- instead of two, ~110 fewer flops than carrying U/V chroma from c3.
    signal angA : t_c10(0 to 9) := (others => (others => '0'));
    signal cosR, sinR : signed(9 downto 0) := (others => '0');
    signal scu, scv   : signed(9 downto 0) := (others => '0');
    signal hitA : std_logic_vector(0 to 8) := (others => '0');

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);
    signal s_io : t_video_stream_yuv444_30b;

begin

    ----------------------------------------------------------------------------
    -- Raster counters, interlace, frame coords, animation phases.
    ----------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;
            r_hedge  <= v_h_edge;
            r_anchor <= '0';

            if v_v_edge = '1' then
                s_fprev <= data_in.field_n;
                s_ilace <= data_in.field_n xor s_fprev;
                s_fodd  <= data_in.field_n;
                s_phase  <= s_phase + s_flowspd;
                s_huep   <= s_huep + s_huespd;
                if s_morph = '1' then s_morphp <= s_morphp + 1; end if;
                if s_spin  = '1' then s_lightp <= s_lightp + 3; end if;
            end if;

            if v_h_edge = '1' then
                x_loc <= (others => '0'); x_idx <= (others => '0'); x_f <= (others => '0');
                qxW <= (others => '0'); qxE <= s_w2(15 downto 0);   -- x_loc^2 / (2wh-x_loc)^2 at x_loc=0
            elsif data_in.avid = '1' then
                x_f <= x_f + 1;
                if x_loc = s_wm1 then
                    x_loc <= (others => '0');
                    if x_idx /= C_MAXCOL - 1 then x_idx <= x_idx + 1; end if;
                    qxW <= (others => '0'); qxE <= s_w2(15 downto 0);
                else
                    x_loc <= x_loc + 1;
                    qxW <= qxW + (resize(x_loc, 16) sll 1) + 1;                       -- (x_loc+1)^2
                    qxE <= qxE - (resize(s_twh - x_loc, 16) sll 1) + 1;               -- (2wh-x_loc-1)^2
                end if;
            end if;

            if v_v_edge = '1' then
                celly_loc <= (others => '0'); celly_idx <= (others => '0');
                frame_act <= '0'; y_ln <= (others => '0');
            elsif data_in.avid = '1' and frame_act = '0' then
                frame_act <= '1'; r_anchor <= '1';
                celly_loc <= (others => '0'); celly_idx <= (others => '0'); y_ln <= (others => '0');
            elsif v_h_edge = '1' and frame_act = '1' then
                y_ln <= y_ln + 1;
                if celly_loc = s_rpm1 then
                    celly_loc <= (others => '0');
                    if celly_idx /= 127 then celly_idx <= celly_idx + 1; end if;
                else
                    celly_loc <= celly_loc + 1;
                end if;
            end if;

            if s_ilace = '1' then
                y_f <= (y_ln & '0') + ("0000000000" & s_fodd);
            else
                y_f <= '0' & y_ln;
            end if;
        end if;
    end process p_position;

    ----------------------------------------------------------------------------
    -- Per-line dy and the elbow corner y-squares (adds only).
    ----------------------------------------------------------------------------
    p_line : process(clk)
        function upd1(sq : unsigned(15 downto 0); dn : signed(10 downto 0)) return unsigned is
            variable v : signed(18 downto 0);
        begin v := signed(resize(sq, 19)) + resize(dn & '0', 19) - 1; return unsigned(v(15 downto 0)); end function;
        function upd2(sq : unsigned(15 downto 0); dn : signed(10 downto 0)) return unsigned is
            variable v : signed(18 downto 0);
        begin v := signed(resize(sq, 19)) + resize(dn & "00", 19) - 4; return unsigned(v(15 downto 0)); end function;
        variable v_hh  : signed(9 downto 0);
        variable v_ynn, v_yns : signed(10 downto 0);   -- new (dy+wh), (dy-wh)
    begin
        if rising_edge(clk) then
            if r_hedge = '1' or r_anchor = '1' then
                v_hh := signed(resize(s_wh, 10));
                if celly_loc = 0 then
                    if s_ilace = '1' and s_fodd = '1' then
                        lm_dy <= -v_hh + 1;
                        qyN <= to_unsigned(1, 16); qyS <= s_w2o(15 downto 0);   -- (1)^2 / (2wh-1)^2
                    else
                        lm_dy <= -v_hh;
                        qyN <= (others => '0'); qyS <= s_w2(15 downto 0);        -- (0)^2 / (2wh)^2
                    end if;
                elsif s_ilace = '1' then
                    lm_dy <= lm_dy + 2;
                    v_ynn := resize(lm_dy + 2, 11) + signed(resize(s_wh, 11));
                    v_yns := resize(lm_dy + 2, 11) - signed(resize(s_wh, 11));
                    qyN <= upd2(qyN, v_ynn); qyS <= upd2(qyS, v_yns);
                else
                    lm_dy <= lm_dy + 1;
                    v_ynn := resize(lm_dy + 1, 11) + signed(resize(s_wh, 11));
                    v_yns := resize(lm_dy + 1, 11) - signed(resize(s_wh, 11));
                    qyN <= upd1(qyN, v_ynn); qyS <= upd1(qyS, v_yns);
                end if;
            end if;
        end if;
    end process p_line;

    ----------------------------------------------------------------------------
    -- Frame FSM: knob shaping, light vector, ranges, 2 serial divides.
    ----------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_w   : integer range 0 to 255;
        variable v_hpo : signed(10 downto 0);
        variable v_acc : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            mp <= ma * mb;
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
                if div_cnt = 17 then div_run <= '0'; else div_cnt <= div_cnt + 1; end if;
            end if;

            if prev_vsync_n = '1' and data_in.vsync_n = '0' then
                lk1  <= unsigned(registers_in(0)(9 downto 0));
                lk2  <= unsigned(registers_in(1)(9 downto 0));
                lk3  <= unsigned(registers_in(2)(9 downto 0));
                lk4  <= unsigned(registers_in(3)(9 downto 0));
                lk5  <= unsigned(registers_in(4)(9 downto 0));
                lk6  <= unsigned(registers_in(5)(9 downto 0));
                lp12 <= unsigned(registers_in(7)(9 downto 0));
                s_round   <= registers_in(6)(0);
                s_percell <= registers_in(6)(1);
                s_morph   <= registers_in(6)(2);
                s_spin    <= registers_in(6)(3);
                s_bypass  <= registers_in(6)(4);
                fsm_t <= (others => '0');
            elsif fsm_t /= x"FF" then
                fsm_t <= fsm_t + 1;
                case to_integer(fsm_t) is
                    when 0 =>
                        v_w := 48 + to_integer(lk1(9 downto 3));   -- 48..175
                        if v_w > 176 then v_w := 176; end if;
                        s_w      <= to_unsigned(v_w, 8);
                        s_rfrac  <= resize(to_unsigned(430, 11) + lk3(9 downto 1), 11);  -- radius frac
                        s_kshine <= to_unsigned(1088, 11) - resize(lk4, 11);
                        s_thresh <= resize(to_unsigned(70, 8) + lk2(9 downto 3), 8);     -- density
                        s_hue    <= lk5;
                        s_hshift <= resize(7 - lk6(9 downto 7), 3);                      -- high Spread = small shift = more rainbow
                        s_flowspd <= resize(lp12(9 downto 3), 10);
                        s_floww   <= resize(to_unsigned(8, 7) + lp12(9 downto 5), 7);
                        s_flowg   <= resize(lp12(9 downto 8), 3);
                        if lp12 > 32 then s_flowon <= '1'; else s_flowon <= '0'; end if;
                        if s_spin = '1' then s_lx <= f_qsin(s_lightp + 256); s_ly <= f_qsin(s_lightp);
                        else                 s_lx <= f_qsin(to_unsigned(160, 10)); s_ly <= f_qsin(to_unsigned(160, 10)); end if;
                    when 1 =>
                        s_wm1 <= s_w - 1;
                        s_wh  <= '0' & s_w(7 downto 1);
                        if s_ilace = '1' then s_rp <= '0' & s_w(7 downto 1); else s_rp <= s_w; end if;
                        ma <= signed(resize(s_w, 13)); mb <= signed(resize(s_rfrac, 13));
                    when 2 =>
                        s_rpm1 <= s_rp - 1;
                        s_twh  <= s_wh & '0';                 -- 2*wh
                        s_lx6  <= resize(shift_right(s_lx, 3), 7);   -- narrowed light dir
                        s_ly6  <= resize(shift_right(s_ly, 3), 7);
                    when 4 =>
                        if mp(18 downto 11) < 4 then s_r <= to_unsigned(4, 8); else s_r <= unsigned(mp(18 downto 11)); end if;
                    when 5 =>
                        s_ls <= ('0' & s_r(7 downto 1)) + ("000" & s_r(7 downto 3));     -- 5r/8
                        ma <= signed(resize(s_r, 13)); mb <= signed(resize(s_kshine, 13));
                    when 6 =>
                        s_omag <= signed(resize(s_ls, 9));
                    when 8 =>
                        s_es <= resize(to_unsigned(2, 8) + unsigned(mp(18 downto 11)), 8);
                        ma <= signed(resize(s_wh, 13)); mb <= resize(s_lx6, 13);          -- wh*lx6
                    when 10 =>
                        -- straight-tube highlight offset, PROPORTIONAL to the light
                        -- (wh*light >> 8) -- same model the elbow uses, so straight
                        -- and elbow highlights line up exactly at every junction.
                        s_ov <= resize(shift_right(mp, 8), 9);                            -- vertical tube: wh*lx6 >> 8
                        ma <= signed(resize(s_wh, 13)); mb <= resize(s_ly6, 13);          -- wh*ly6
                    when 12 =>
                        s_oh <= resize(shift_right(mp, 8), 9);                            -- horizontal tube: wh*ly6 >> 8
                        ma <= signed(resize(s_r, 13)) + resize(s_omag, 13);
                        mb <= signed(resize(s_r, 13)) + resize(s_omag, 13);
                    when 13 =>
                        if f_absu(s_ov) > f_absu(s_oh) then s_omax <= f_absu(s_ov);
                        else                                s_omax <= f_absu(s_oh); end if;
                    when 14 =>
                        -- run the straight offsets through the SAME saturating map
                        -- the elbow applies per pixel: both tube families get one
                        -- constant highlight-offset magnitude, elbow arcs stay
                        -- concentric, junctions meet flush.
                        s_ov <= resize(f_oclamp(resize(s_ov, 14) sll 2, s_omax), 9);
                        s_oh <= resize(f_oclamp(resize(s_oh, 14) sll 2, s_omax), 9);
                    when 15 =>
                        s_ediff2 <= unsigned(mp(15 downto 0));
                        ma <= signed(resize(s_es, 13)); mb <= signed(resize(s_es, 13));
                    when 18 =>
                        s_espec2 <= unsigned(mp(15 downto 0));
                    when 21 =>
                        s_sle <= to_unsigned(norm_shift(s_ediff2), 4);
                    when 22 =>
                        div_d   <= resize(s_span, 18) sll 7;
                        div_v   <= resize(shift_right(shift_left(resize(s_ediff2, 24), to_integer(s_sle)), 8), 8);
                        div_q <= (others => '0'); div_rem <= (others => '0'); div_cnt <= (others => '0'); div_run <= '1';
                    when 43 =>
                        if div_q > 255 then s_kd8 <= (others => '1'); else s_kd8 <= div_q(7 downto 0); end if;
                        s_sls <= to_unsigned(norm_shift(s_espec2), 4);
                    when 44 =>
                        div_d   <= resize(s_speff, 18) sll 7;
                        div_v   <= resize(shift_right(shift_left(resize(s_espec2, 24), to_integer(s_sls)), 8), 8);
                        div_q <= (others => '0'); div_rem <= (others => '0'); div_cnt <= (others => '0'); div_run <= '1';
                    when 65 =>
                        if div_q > 1023 then s_ks9 <= (others => '1'); else s_ks9 <= div_q(9 downto 0); end if;
                    -- rounded-elbow constants (all in vblank, shared multiplier + divider)
                    when 66 =>
                        ma <= signed(resize(s_wh, 13)); mb <= signed(resize(s_wh, 13));               -- wh^2
                    when 69 =>
                        s_wh2 <= unsigned(mp(15 downto 0));
                        ma <= signed(resize(s_wh, 13)) - signed(resize(s_r, 13));
                        mb <= signed(resize(s_wh, 13)) - signed(resize(s_r, 13));                     -- (wh-r)^2
                    when 72 =>
                        s_qlo <= unsigned(mp(16 downto 0));
                        s_w2  <= resize(s_wh2 & "00", 17);                                            -- (2wh)^2
                        s_w2o <= resize(s_wh2 & "00", 17) - resize(s_wh & "00", 17) + 1;              -- (2wh-1)^2
                        ma <= signed(resize(s_wh, 13)) + signed(resize(s_r, 13));
                        mb <= signed(resize(s_wh, 13)) + signed(resize(s_r, 13));                     -- (wh+r)^2
                    when 75 =>
                        s_qhi <= unsigned(mp(16 downto 0));
                    when 78 =>
                        -- s_krec = round(4096 / 2wh): the exact reciprocal the pixel
                        -- path multiplies in after a FIXED >>6, so the elbow
                        -- cross-section is in TRUE pixel units at any scale (the old
                        -- power-of-2 shift alone was off by 0.71..1.41x with scale,
                        -- which is why highlights only matched near 2wh = 64/128).
                        div_d <= resize(to_unsigned(4096, 13) + resize(s_wh, 13), 18);
                        div_v <= s_twh(7 downto 0);
                        div_q <= (others => '0'); div_rem <= (others => '0'); div_cnt <= (others => '0'); div_run <= '1';
                    when 99 =>
                        if div_q > 127 then s_krec <= (others => '1');
                        else s_krec <= div_q(6 downto 0); end if;
                    when others => null;
                end case;
            end if;
        end if;
    end process p_frame;

    ----------------------------------------------------------------------------
    -- Pixel pipeline.
    ----------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_ang  : unsigned(9 downto 0);
        variable v_cxp1, v_cyp1 : unsigned(6 downto 0);
        variable v_mask : std_logic_vector(3 downto 0);
        variable v_hitH, v_hitV : boolean;
        variable v_along : signed(12 downto 0);
        variable v_et   : signed(11 downto 0);
        variable v_hit : std_logic;
        variable v_e2   : unsigned(16 downto 0);
        variable v_dl, v_ds : signed(17 downto 0);
        variable v_lv   : unsigned(9 downto 0);
        variable v_sv   : unsigned(10 downto 0);
        variable v_cf   : signed(10 downto 0);
        variable v_yd   : unsigned(11 downto 0);
        variable v_fph  : unsigned(6 downto 0);
        variable v_tri  : unsigned(6 downto 0);
        variable v_su   : signed(11 downto 0);
        variable v_chu, v_chv : signed(12 downto 0);
        variable v_elb  : std_logic;
        variable v_qx, v_qy : unsigned(15 downto 0);    -- selected corner-distance squares
        variable v_cx, v_cy : signed(9 downto 0);       -- selected corner (radial origin)
        variable v_off  : signed(11 downto 0);          -- light . radial highlight offset
        variable v_etb0 : signed(9 downto 0);           -- (rho^2 - wh^2) >> 6
        variable v_etb  : signed(11 downto 0);          -- linearised radial dist
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            ------------------------------------------------------------------
            -- c1: local coords + the four edge-pipe bits + hue angle.
            ------------------------------------------------------------------
            g0_dx  <= signed(resize(x_loc, 10)) - signed(resize(s_wh, 10));
            g0_dy  <= lm_dy;
            g0_xf  <= x_f; g0_yf <= y_f;
            g0_qxw <= qxW; g0_qxe <= qxE; g0_qyn <= qyN; g0_qys <= qyS;

            -- linear part of each of the four edge hashes (avalanche at c2)
            v_cxp1 := x_idx + 1;
            v_cyp1 := celly_idx + 1;
            g0_hN <= f_hraw(x_idx,  celly_idx, '0', s_seed);
            g0_hS <= f_hraw(x_idx,  v_cyp1,    '0', s_seed);
            g0_hW <= f_hraw(x_idx,  celly_idx, '1', s_seed);
            g0_hE <= f_hraw(v_cxp1, celly_idx, '1', s_seed);

            -- hue angle: flowing rainbow across the screen, or per-cell
            if s_percell = '1' then
                v_ang := f_hraw(x_idx, celly_idx, '0', s_seed + 51)(9 downto 2) & "00";
            else
                v_ang := resize(shift_right(resize(x_f, 13) + resize(y_f, 13), to_integer(s_hshift)), 10);
            end if;
            g0_ang <= v_ang + s_hue + s_huep(15 downto 6);

            ------------------------------------------------------------------
            -- c2: abs terms, ball dx^2, finished port mask, sin/cos.
            ------------------------------------------------------------------
            p2_absdx <= f_absu(g0_dx);
            p2_absdy <= f_absu(g0_dy);
            p2_dx    <= g0_dx; p2_dy <= g0_dy;
            p2_xf    <= g0_xf; p2_yf <= g0_yf;
            p2_mask  <= f_b((f_hfold(g0_hN) + s_morphp) < s_thresh)
                      & f_b((f_hfold(g0_hS) + s_morphp) < s_thresh)
                      & f_b((f_hfold(g0_hE) + s_morphp) < s_thresh)
                      & f_b((f_hfold(g0_hW) + s_morphp) < s_thresh);
            angA(0) <= g0_ang;
            for i in 1 to 9 loop angA(i) <= angA(i - 1); end loop;
            p2_qxw   <= g0_qxw; p2_qxe <= g0_qxe; p2_qyn <= g0_qyn; p2_qys <= g0_qys;

            ------------------------------------------------------------------
            -- c3: hit tests (H/V tubes gated by ports).  Pure elbows are rounded
            -- downstream (S7); tees/crosses/caps are the straight-tube overlap.
            ------------------------------------------------------------------
            v_mask := p2_mask;                            -- N S E W

            v_hitV := (p2_absdx < resize(s_r, 10)) and
                      ((v_mask(3) = '1' and p2_dy < 0) or (v_mask(2) = '1' and p2_dy >= 0));
            v_hitH := (p2_absdy < resize(s_r, 10)) and
                      ((v_mask(1) = '1' and p2_dx >= 0) or (v_mask(0) = '1' and p2_dx < 0));

            v_hit := '0';
            v_et := (others => '0'); v_along := (others => '0');
            if v_hitH and (not v_hitV or p2_absdy <= p2_absdx) then
                v_hit := '1';
                v_et := resize(p2_dy, 12) - resize(s_oh, 12);
                v_along := resize(signed(p2_xf), 13);
            elsif v_hitV then
                v_hit := '1';
                v_et := resize(p2_dx, 12) - resize(s_ov, 12);
                v_along := resize(signed(p2_yf), 13);
            end if;

            p3_hit  <= v_hit;
            p3_et   <= v_et;
            p3_along <= v_along;

            -- rounded elbow (Sharp mode only): a PURE elbow = exactly one of N/S
            -- and one of E/W.  rho^2 to the bend's corner = the near-edge
            -- x-square + the near-edge y-square, read from the incremental
            -- accumulators (a select + an add here -- no pixel multiply); the
            -- corner-relative coords feed the (light . radial) highlight offset.
            v_elb := (p2_mask(3) xor p2_mask(2)) and (p2_mask(1) xor p2_mask(0))
                     and s_round;
            if p2_mask(1) = '1' then v_qx := p2_qxe; v_cx := signed(resize(s_wh, 10));      -- E corner
            else                     v_qx := p2_qxw; v_cx := -signed(resize(s_wh, 10)); end if; -- W corner
            if p2_mask(3) = '1' then v_qy := p2_qyn; v_cy := -signed(resize(s_wh, 10));     -- N corner
            else                     v_qy := p2_qys; v_cy := signed(resize(s_wh, 10)); end if;  -- S corner
            p3_rho2 <= resize(v_qx, 17) + resize(v_qy, 17);
            p3_excx <= resize(p2_dx, 10) - v_cx;      -- radial direction from the bend corner
            p3_eycy <= resize(p2_dy, 10) - v_cy;
            p3_elb  <= v_elb;

            ------------------------------------------------------------------
            -- c3b: u = rho^2 - wh^2 (isolated subtract) + light.radial (two small
            -- mults) + the rho^2 band test (so c3c only merges).
            ------------------------------------------------------------------
            p3b_ehit <= f_b(p3_rho2 >= s_qlo and p3_rho2 <= s_qhi);
            p3b_u    <= signed(resize(p3_rho2, 18)) - signed(resize(s_wh2, 18));
            p3b_mx   <= p3_excx * s_lx6;              -- light . radial (x, y terms)
            p3b_my   <= p3_eycy * s_ly6;
            p3b_et    <= p3_et;
            p3b_hit   <= p3_hit;
            p3b_elb   <= p3_elb;
            p3b_along <= p3_along;

            ------------------------------------------------------------------
            -- c3c: linearise the radial distance u/(2wh) into the tube
            -- cross-section (the ONE isolated multiply), band-test membership,
            -- and MERGE the elbow over the straight-tube result for elbow cells.
            ------------------------------------------------------------------
            -- linearise u/(2wh) EXACTLY: fixed >>6 (free wiring) then multiply by
            -- the per-frame reciprocal krec = 4096/(2wh), so the elbow
            -- cross-section is in TRUE pixel units at every scale.  The old
            -- power-of-2 shift alone was off by 0.71..1.41x depending on scale,
            -- which made elbow highlight width/position match the straights only
            -- when 2wh sat near a power of 2.  The (light . radial) projection
            -- sum registers RAW here; its saturating clamp + the subtract live
            -- in their own stage (c3d) -- one stage was too long post-route.
            v_etb0 := resize(shift_right(p3b_u, 6), 10);
            p3c_ets  <= resize(shift_right(v_etb0 * signed('0' & s_krec), 6), 12);
            p3c_do   <= resize(shift_right(resize(p3b_mx, 18) + resize(p3b_my, 18), 6), 12);
            p3c_et   <= p3b_et;
            p3c_elb  <= p3b_elb;
            p3c_ehit <= p3b_ehit;
            p3c_hit  <= p3b_hit;
            p3c_along <= p3b_along;

            ------------------------------------------------------------------
            -- c3d: clamp the highlight offset, subtract it, and MERGE the elbow
            -- over the straight-tube result.
            ------------------------------------------------------------------
            if p3c_elb = '1' then
                p3d_et  <= p3c_ets - f_oclamp(resize(p3c_do, 14), s_omax);
                p3d_hit <= p3c_ehit;
            else
                p3d_et  <= p3c_et;
                p3d_hit <= p3c_hit;
            end if;
            p3d_along <= p3c_along;

            ------------------------------------------------------------------
            -- c4: shading field e2 (rounded 1-D tube cross-section).
            ------------------------------------------------------------------
            v_e2 := resize(unsigned(p3d_et * p3d_et), 17);
            p4_e2 <= v_e2;

            ------------------------------------------------------------------
            -- c5: diffuse / specular head-room.
            ------------------------------------------------------------------
            v_dl := signed(resize(s_ediff2, 18)) - signed(resize(p4_e2, 18));
            if v_dl < 0 then v_dl := (others => '0'); end if;
            p5_dl <= unsigned(v_dl(15 downto 0));
            v_ds := signed(resize(s_espec2, 18)) - signed(resize(p4_e2, 18));
            if v_ds < 0 then v_ds := (others => '0'); end if;
            p5_ds <= unsigned(v_ds(15 downto 0));

            -- c6/c7: barrel normalise
            p6_dlc <= shift_left(p5_dl, 4 * to_integer(s_sle(3 downto 2)));
            p6_dsc <= shift_left(p5_ds, 4 * to_integer(s_sls(3 downto 2)));
            p7_dlb <= shift_left(p6_dlc, to_integer(s_sle(1 downto 0)))(15 downto 8);
            p7_dsb <= shift_left(p6_dsc, to_integer(s_sls(1 downto 0)))(15 downto 8);

            ------------------------------------------------------------------
            -- c8: gains + flow pulse.
            ------------------------------------------------------------------
            p8_pl <= p7_dlb * s_kd8;
            p8_ps <= p7_dsb * s_ks9;
            v_fph := unsigned(p3d_along(7 downto 1)) + s_phase(14 downto 8);
            if s_flowon = '1' and v_fph < s_floww then
                if v_fph < (s_floww - v_fph) then v_tri := v_fph; else v_tri := s_floww - v_fph; end if;
                p8_flow <= resize(shift_left(resize(v_tri, 10), to_integer(s_flowg)), 10);
            else
                p8_flow <= (others => '0');
            end if;

            ------------------------------------------------------------------
            -- c9: diffuse level (bright ambient floor) + specular level.
            ------------------------------------------------------------------
            cosR <= f_qsin(angA(9) + 256);
            sinR <= f_qsin(angA(9));
            v_lv := resize(s_amb, 10) + resize(p8_pl(15 downto 7), 10);
            if v_lv > 255 then v_lv := to_unsigned(255, 10); end if;
            p9_l <= v_lv(7 downto 0);
            v_sv := resize(p8_ps(17 downto 7), 11) + resize(p8_flow, 11);
            if v_sv > 511 then v_sv := to_unsigned(511, 11); end if;
            p9_s <= '0' & v_sv(8 downto 0);
            p9_flow <= p8_flow;

            ------------------------------------------------------------------
            -- c10: chroma desaturation factor (highlight -> white) + luma.
            ------------------------------------------------------------------
            v_chu := resize(shift_right(signed('0' & s_sat) * cosR, 9), 13);
            if v_chu < -512 then v_chu := to_signed(-512, 13); elsif v_chu > 511 then v_chu := to_signed(511, 13); end if;
            scu <= resize(v_chu, 10);
            v_chv := resize(shift_right(signed('0' & s_sat) * sinR, 9), 13);
            if v_chv < -512 then v_chv := to_signed(-512, 13); elsif v_chv > 511 then v_chv := to_signed(511, 13); end if;
            scv <= resize(v_chv, 10);
            v_cf := to_signed(255, 11) - signed(resize(p9_s(9 downto 1), 11));
            if v_cf < 0 then v_cf := (others => '0'); end if;
            p10_cf <= unsigned(v_cf(7 downto 0));
            v_yd := resize(p9_l, 12) + resize(p9_s, 12) + resize(p9_flow(9 downto 1), 12);
            if v_yd > 1023 then v_yd := to_unsigned(1023, 12); end if;
            p10_y <= v_yd(9 downto 0);

            ------------------------------------------------------------------
            -- c11: bright hue chroma * factor.
            ------------------------------------------------------------------
            p11_um <= resize(scu * signed(resize(p10_cf, 9)), 20);
            p11_vm <= resize(scv * signed(resize(p10_cf, 9)), 20);
            p11_y  <= p10_y;

            ------------------------------------------------------------------
            -- c12: compose pipe over the near-black field.
            ------------------------------------------------------------------
            if hitA(7) = '1' then
                v_su := to_signed(512, 12) + resize(shift_right(p11_um, 8), 12);
                p12_u <= unsigned(v_su(9 downto 0));
                v_su := to_signed(512, 12) + resize(shift_right(p11_vm, 8), 12);
                p12_v <= unsigned(v_su(9 downto 0));
                p12_y <= p11_y;
            else
                p12_y <= to_unsigned(28, 10);  -- near-black field
                p12_u <= C_MID; p12_v <= C_MID;
            end if;

            ------------------------------------------------------------------
            -- carry pipes + output.
            ------------------------------------------------------------------
            hitA(0) <= p3d_hit;
            for i in 1 to 8 loop hitA(i) <= hitA(i - 1); end loop;

            if s_bypass = '1' then
                s_io.y <= pipe(LATENCY - 1).y; s_io.u <= pipe(LATENCY - 1).u; s_io.v <= pipe(LATENCY - 1).v;
            else
                s_io.y <= std_logic_vector(p12_y); s_io.u <= std_logic_vector(p12_u); s_io.v <= std_logic_vector(p12_v);
            end if;
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= pipe(LATENCY - 1).avid;
            s_io.field_n <= pipe(LATENCY - 1).field_n;
        end if;
    end process p_pipe;

    data_out.y <= s_io.y;  data_out.u <= s_io.u;  data_out.v <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture pipedream;
