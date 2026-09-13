-- Porthole: catalogue combo #6 (seed 20260912, phosphor slot redrawn) --
-- polar warp (spherize) + Perlin displacement + picture-in-picture with
-- warping.  Two knobs each; the switches and the slider decide how they
-- combine.
--
--   K1 Curve    spherize: row scale by row radius, bipolar (pinch / bulge)
--   K2 Zoom     overall scale 0.25x .. 4x (log)
--   K3 Noise    value-noise displacement amount (64x32 lattice, scrolling)
--   K4 Speed    lattice scroll speed
--   K5 Size     window width (10 .. 100% of the picture; height follows)
--   K6 Position window centre column
--
--   S7  Outside original picture / the warped picture outside the window
--   S8  Noise   inside the window only / everywhere
--   S9  Border  black frame around the window
--   S10 Mirror  window content flipped
--   S11 Fit     window shows the whole picture compressed / a 1:1 crop
--   P12 Drift   the window swings sideways on a sine, speed = slider
--
-- Two DDAs (global map, window map) selected per pixel by the window
-- rectangle; both take the spherize row scale and the noise lattice
-- displacement.  Latency 12 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture porthole of program_top is

    constant C_LAT : integer := 12;
    constant C_K_MIN : integer := 96;
    constant C_SEED_NOISE : unsigned(15 downto 0) := x"3C6E";

    function f_round1(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 5);
        return t xor shift_right(t, 6);
    end function;
    function f_round2(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 3);
        return t xor shift_right(t, 8);
    end function;
    function f_round3(k : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := k + shift_left(k, 7);
        t := t xor shift_right(t, 5);
        return t xor shift_left(t, 8);
    end function;

    type t_zoom_rom is array (0 to 63) of unsigned(12 downto 0);
    constant C_ZOOM_ROM : t_zoom_rom := (
        to_unsigned(256, 13), to_unsigned(267, 13), to_unsigned(279, 13), to_unsigned(292, 13), to_unsigned(304, 13), to_unsigned(318, 13), to_unsigned(332, 13), to_unsigned(347, 13),
        to_unsigned(362, 13), to_unsigned(378, 13), to_unsigned(395, 13), to_unsigned(412, 13), to_unsigned(431, 13), to_unsigned(450, 13), to_unsigned(470, 13), to_unsigned(490, 13),
        to_unsigned(512, 13), to_unsigned(535, 13), to_unsigned(558, 13), to_unsigned(583, 13), to_unsigned(609, 13), to_unsigned(636, 13), to_unsigned(664, 13), to_unsigned(693, 13),
        to_unsigned(724, 13), to_unsigned(756, 13), to_unsigned(790, 13), to_unsigned(825, 13), to_unsigned(861, 13), to_unsigned(899, 13), to_unsigned(939, 13), to_unsigned(981, 13),
        to_unsigned(1024, 13), to_unsigned(1069, 13), to_unsigned(1117, 13), to_unsigned(1166, 13), to_unsigned(1218, 13), to_unsigned(1272, 13), to_unsigned(1328, 13), to_unsigned(1387, 13),
        to_unsigned(1448, 13), to_unsigned(1512, 13), to_unsigned(1579, 13), to_unsigned(1649, 13), to_unsigned(1722, 13), to_unsigned(1798, 13), to_unsigned(1878, 13), to_unsigned(1961, 13),
        to_unsigned(2048, 13), to_unsigned(2139, 13), to_unsigned(2233, 13), to_unsigned(2332, 13), to_unsigned(2435, 13), to_unsigned(2543, 13), to_unsigned(2656, 13), to_unsigned(2774, 13),
        to_unsigned(2896, 13), to_unsigned(3025, 13), to_unsigned(3158, 13), to_unsigned(3298, 13), to_unsigned(3444, 13), to_unsigned(3597, 13), to_unsigned(3756, 13), to_unsigned(3922, 13)
    );

    type t_sine_rom is array (0 to 255) of integer range -256 to 255;
    constant C_SINE : t_sine_rom := (
        -255, -255, -255, -254, -254, -253, -252, -251,
        -250, -249, -247, -246, -244, -242, -240, -238,
        -236, -233, -231, -228, -225, -222, -219, -215,
        -212, -208, -205, -201, -197, -193, -189, -185,
        -180, -176, -171, -167, -162, -157, -152, -147,
        -142, -136, -131, -126, -120, -115, -109, -103,
         -98,  -92,  -86,  -80,  -74,  -68,  -62,  -56,
         -50,  -44,  -37,  -31,  -25,  -19,  -13,   -6,
           0,    6,   13,   19,   25,   31,   37,   44,
          50,   56,   62,   68,   74,   80,   86,   92,
          98,  103,  109,  115,  120,  126,  131,  136,
         142,  147,  152,  157,  162,  167,  171,  176,
         180,  185,  189,  193,  197,  201,  205,  208,
         212,  215,  219,  222,  225,  228,  231,  233,
         236,  238,  240,  242,  244,  246,  247,  249,
         250,  251,  252,  253,  254,  254,  255,  255,
         255,  255,  255,  254,  254,  253,  252,  251,
         250,  249,  247,  246,  244,  242,  240,  238,
         236,  233,  231,  228,  225,  222,  219,  215,
         212,  208,  205,  201,  197,  193,  189,  185,
         180,  176,  171,  167,  162,  157,  152,  147,
         142,  136,  131,  126,  120,  115,  109,  103,
          98,   92,   86,   80,   74,   68,   62,   56,
          50,   44,   37,   31,   25,   19,   13,    6,
           0,   -6,  -13,  -19,  -25,  -31,  -37,  -44,
         -50,  -56,  -62,  -68,  -74,  -80,  -86,  -92,
         -98, -103, -109, -115, -120, -126, -131, -136,
        -142, -147, -152, -157, -162, -167, -171, -176,
        -180, -185, -189, -193, -197, -201, -205, -208,
        -212, -215, -219, -222, -225, -228, -231, -233,
        -236, -238, -240, -242, -244, -246, -247, -249,
        -250, -251, -252, -253, -254, -254, -255, -255
    );

    --------------------------------------------------------------------------
    -- Line buffers
    --------------------------------------------------------------------------
    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    --------------------------------------------------------------------------
    -- Controls
    --------------------------------------------------------------------------
    signal s_k_curve, s_k_zoom, s_k_noise, s_k_speed, s_k_size, s_k_pos : unsigned(9 downto 0);
    signal s_sw_outside, s_sw_noise_all, s_sw_border, s_sw_mirror, s_sw_fit : std_logic;
    signal s_p_drift : unsigned(9 downto 0);

    --------------------------------------------------------------------------
    -- Tracking
    --------------------------------------------------------------------------
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_saw_active : std_logic := '0';
    signal s_line       : unsigned(10 downto 0) := (others => '0');
    signal s_height     : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_half_h     : unsigned(10 downto 0) := to_unsigned(135, 11);
    signal s_half_w     : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_frame      : unsigned(15 downto 0) := (others => '0');
    signal s_oy         : unsigned(15 downto 0) := (others => '0');
    signal s_wpar, s_rbank, s_we : std_logic := '0';

    --------------------------------------------------------------------------
    -- Vblank sequencer
    --------------------------------------------------------------------------
    signal s_vstep   : unsigned(5 downto 0) := "111111";
    signal s_div_rem : unsigned(11 downto 0) := (others => '0');
    signal s_div_q   : unsigned(20 downto 0) := (others => '0');
    signal s_div_i   : unsigned(4 downto 0) := (others => '0');
    signal s_div_d   : unsigned(10 downto 0) := (others => '0');
    signal s_div_n   : unsigned(20 downto 0) := (others => '0');
    signal s_inv_hh  : unsigned(13 downto 0) := to_unsigned(7767, 14);
    signal s_kw      : unsigned(13 downto 0) := to_unsigned(1024, 14);   -- Q10 W/ww
    signal s_c       : signed(10 downto 0) := (others => '0');
    signal s_zoom    : unsigned(12 downto 0) := to_unsigned(1024, 13);
    signal s_qa, s_qb : unsigned(10 downto 0) := (others => '0');
    signal s_vph : unsigned(15 downto 0) := (others => '0');
    signal s_vpl : unsigned(16 downto 0) := (others => '0');
    signal s_vp      : unsigned(21 downto 0) := (others => '0');
    signal s_amp_eff : unsigned(9 downto 0) := (others => '0');
    signal s_ww      : unsigned(10 downto 0) := to_unsigned(240, 11);   -- window width
    signal s_whh     : unsigned(10 downto 0) := to_unsigned(68, 11);    -- window half height
    signal s_wcx     : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_wx0, s_wx1 : unsigned(10 downto 0) := (others => '0');
    signal s_wy0, s_wy1 : unsigned(10 downto 0) := (others => '0');
    signal s_drift_ph : unsigned(15 downto 0) := (others => '0');
    signal s_sin_addr : unsigned(7 downto 0) := (others => '0');
    signal s_sin_q   : signed(9 downto 0) := (others => '0');
    signal s_sinneg  : std_logic := '0';
    signal s_drp     : signed(21 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Per-line sequencer (shared 14x14 multiplier, operand-split, 3-clock)
    --------------------------------------------------------------------------
    signal s_lstep    : unsigned(4 downto 0) := "11111";
    signal s_ma, s_mb : unsigned(13 downto 0) := (others => '0');
    signal s_mph      : unsigned(20 downto 0) := (others => '0');
    signal s_mpl      : unsigned(20 downto 0) := (others => '0');
    signal s_mp       : unsigned(27 downto 0) := (others => '0');
    signal s_dy       : unsigned(10 downto 0) := (others => '0');
    signal s_rhoq     : unsigned(10 downto 0) := (others => '0');
    signal s_w        : unsigned(10 downto 0) := (others => '0');
    signal s_f        : unsigned(12 downto 0) := to_unsigned(1024, 13);
    signal s_kz       : unsigned(12 downto 0) := to_unsigned(1024, 13);
    signal s_kk       : unsigned(13 downto 0) := to_unsigned(1024, 14);
    signal s_pg       : unsigned(23 downto 0) := (others => '0');
    signal s_pw       : unsigned(24 downto 0) := (others => '0');
    signal s_initg, s_initw : signed(25 downto 0) := (others => '0');
    signal s_stepw    : signed(14 downto 0) := (others => '0');
    signal s_inrows   : std_logic := '0';
    signal s_ncy, s_ncy1 : unsigned(8 downto 0) := (others => '0');
    signal s_nfy      : unsigned(4 downto 0) := (others => '0');
    signal s_atw0     : std_logic := '0';

    --------------------------------------------------------------------------
    -- Noise lattice engine
    --------------------------------------------------------------------------
    signal s_nstep  : unsigned(3 downto 0) := "1111";
    signal s_ct     : unsigned(6 downto 0) := (others => '0');
    signal s_nrem   : unsigned(1 downto 0) := (others => '0');
    signal s_nshift : std_logic := '0';
    signal s_ha, s_hb : unsigned(15 downto 0) := (others => '0');
    signal s_h0     : unsigned(7 downto 0) := (others => '0');
    signal s_hd     : signed(8 downto 0) := (others => '0');
    signal s_hp     : signed(14 downto 0) := (others => '0');
    signal s_hl     : signed(9 downto 0) := (others => '0');
    signal s_hlc    : signed(8 downto 0) := (others => '0');
    signal s_hq     : signed(19 downto 0) := (others => '0');
    signal s_vA, s_vB, s_vC : signed(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Pixel pipeline
    --------------------------------------------------------------------------
    signal s_accg, s_accw : signed(25 downto 0) := (others => '0');
    signal s_nacc : signed(15 downto 0) := (others => '0');
    signal s_nstp : signed(9 downto 0) := (others => '0');
    signal s_x1   : unsigned(10 downto 0) := (others => '0');
    signal s_ug2, s_uw2 : signed(14 downto 0) := (others => '0');
    signal s_n2   : signed(9 downto 0) := (others => '0');
    signal s_inwin2, s_bord2 : std_logic := '0';
    signal s_x2   : unsigned(10 downto 0) := (others => '0');
    signal s_u3   : signed(14 downto 0) := (others => '0');
    signal s_bord3 : std_logic := '0';
    signal s_addr : unsigned(10 downto 0) := (others => '0');
    signal s_bg   : std_logic := '0';
    signal s_bord4 : std_logic := '0';
    signal s_a_bg, s_a_rbank, s_a_bord : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_y7, s_u7, s_v7 : unsigned(9 downto 0) := (others => '0');
    type t_y is array (7 to 11) of unsigned(9 downto 0);
    signal s_yd, s_ud, s_vd : t_y := (others => (others => '0'));
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Sync delay
    --------------------------------------------------------------------------
    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_curve <= unsigned(registers_in(0));
    s_k_zoom  <= unsigned(registers_in(1));
    s_k_noise <= unsigned(registers_in(2));
    s_k_speed <= unsigned(registers_in(3));
    s_k_size  <= unsigned(registers_in(4));
    s_k_pos   <= unsigned(registers_in(5));
    s_sw_outside   <= registers_in(6)(0);
    s_sw_noise_all <= registers_in(6)(1);
    s_sw_border    <= registers_in(6)(2);
    s_sw_mirror    <= registers_in(6)(3);
    s_sw_fit       <= registers_in(6)(4);
    s_p_drift <= unsigned(registers_in(7));

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin_q <= to_signed(C_SINE(to_integer(s_sin_addr)), 10);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage 1 + sequencers
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_key  : unsigned(15 downto 0);
        variable v_ysum : unsigned(15 downto 0);
        variable v_v    : signed(9 downto 0);
        variable v_rem  : unsigned(12 downto 0);
        variable v_kk   : signed(22 downto 0);
        variable v_fq   : signed(13 downto 0);
        variable v_x    : signed(12 downto 0);
        variable v_kz   : unsigned(14 downto 0);
        variable v_kkq  : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_prev_avid    <= data_in.avid;
            if data_in.avid = '1' then
                s_rx <= s_rx + 1;
                s_saw_active <= '1';
            else
                s_rx <= (others => '0');
            end if;
            if s_in_avid = '1' then
                s_wr_x <= s_wr_x + 1;
            else
                s_wr_x <= (others => '0');
            end if;
            s_we <= data_in.avid;
            s_x1 <= s_rx;
            if s_rx + 1 = s_wx0 then s_atw0 <= '1'; else s_atw0 <= '0'; end if;
            -- shared multipliers (operand-split, product valid 3 clocks after set)
            s_mph <= s_ma * s_mb(13 downto 7);
            s_mpl <= s_ma * s_mb(6 downto 0);
            s_mp  <= shift_left(resize(s_mph, 28), 7) + resize(s_mpl, 28);
            s_vph <= s_qa * s_qb(10 downto 6);
            s_vpl <= s_qa * s_qb(5 downto 0);
            s_vp  <= shift_left(resize(s_vph, 22), 6) + resize(s_vpl, 22);

            -- DDAs: global map (step kz), window map (step kk from window start)
            if data_in.avid = '1' then
                s_accg <= s_accg + signed(resize(s_kz, 26));
                if s_atw0 = '1' then
                    s_accw <= s_initw;
                else
                    s_accw <= s_accw + resize(s_stepw, 26);
                end if;
                if s_rx(5 downto 0) = 0 and s_rx /= 0 then
                    s_nacc  <= shift_left(resize(s_vB, 16), 6);
                    s_nstp  <= s_vC - s_vB;
                    s_vA    <= s_vB;
                    s_vB    <= s_vC;
                    s_nrem  <= "01";
                    s_nshift <= '0';
                    s_nstep <= (others => '0');
                else
                    s_nacc <= s_nacc + resize(s_nstp, 16);
                end if;
            else
                s_accg <= s_initg;
                s_accw <= s_initw;
                s_nstp <= s_vB - s_vA;
                s_nacc <= shift_left(resize(s_vA, 16), 6) - resize(s_vB - s_vA, 16);
            end if;

            ------------------------------------------------------------------
            -- Noise lattice engine (roulette)
            ------------------------------------------------------------------
            case to_integer(s_nstep) is
                when 0 =>
                    v_key := s_ncy & s_ct;
                    s_ha  <= C_SEED_NOISE + v_key;
                    v_key := s_ncy1 & s_ct;
                    s_hb  <= C_SEED_NOISE + v_key;
                    s_nstep <= s_nstep + 1;
                when 1 => s_ha <= f_round1(s_ha); s_hb <= f_round1(s_hb); s_nstep <= s_nstep + 1;
                when 2 => s_ha <= f_round2(s_ha); s_hb <= f_round2(s_hb); s_nstep <= s_nstep + 1;
                when 3 => s_ha <= f_round3(s_ha); s_hb <= f_round3(s_hb); s_nstep <= s_nstep + 1;
                when 4 =>
                    s_h0 <= s_ha(15 downto 8);
                    s_hd <= signed('0' & s_hb(15 downto 8)) - signed('0' & s_ha(15 downto 8));
                    s_nstep <= s_nstep + 1;
                when 5 => s_hp <= s_hd * signed('0' & s_nfy); s_nstep <= s_nstep + 1;
                when 6 => s_hl <= signed(resize(s_h0, 10)) + resize(shift_right(s_hp, 5), 10); s_nstep <= s_nstep + 1;
                when 7 => s_hlc <= resize(s_hl - to_signed(128, 10), 9); s_nstep <= s_nstep + 1;
                when 8 => s_hq <= s_hlc * signed('0' & s_amp_eff); s_nstep <= s_nstep + 1;
                when 9 =>
                    v_v := resize(shift_right(s_hq, 9), 10);
                    if s_nshift = '1' then
                        s_vA <= s_vB;
                        s_vB <= s_vC;
                    end if;
                    s_vC <= v_v;
                    s_ct <= s_ct + 1;
                    if s_nrem > 1 then
                        s_nrem  <= s_nrem - 1;
                        s_nstep <= (others => '0');
                    else
                        s_nrem  <= (others => '0');
                        s_nstep <= "1111";
                    end if;
                when others => null;
            end case;

            ------------------------------------------------------------------
            -- Per-line sequencer: one shared multiply at a time
            ------------------------------------------------------------------
            case to_integer(s_lstep) is
                when 0 =>
                    if s_line >= s_half_h then s_dy <= s_line - s_half_h; else s_dy <= s_half_h - s_line; end if;
                    v_ysum := resize(s_line, 16) + s_oy;
                    s_ncy  <= v_ysum(13 downto 5);
                    s_ncy1 <= v_ysum(13 downto 5) + 1;
                    s_nfy  <= v_ysum(4 downto 0);
                    if s_line >= s_wy0 and s_line < s_wy1 then s_inrows <= '1'; else s_inrows <= '0'; end if;
                    s_ct <= (others => '0'); s_nrem <= "11"; s_nshift <= '1'; s_nstep <= (others => '0');
                    s_lstep <= s_lstep + 1;
                when 1 =>
                    s_ma <= resize(s_dy, 14); s_mb <= s_inv_hh;            -- rho = dy * inv_hh
                    s_lstep <= s_lstep + 1;
                when 2 | 3 => s_lstep <= s_lstep + 1;
                when 4 =>
                    if s_mp(24 downto 20) /= 0 then s_rhoq <= to_unsigned(1024, 11); else s_rhoq <= s_mp(20 downto 10); end if;
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    s_ma <= resize(s_rhoq, 14); s_mb <= resize(s_rhoq, 14);  -- r2
                    s_lstep <= s_lstep + 1;
                when 6 | 7 => s_lstep <= s_lstep + 1;
                when 8 =>
                    if s_mp(21 downto 10) > 1024 then s_w <= (others => '0'); else s_w <= to_unsigned(1024, 11) - s_mp(20 downto 10); end if;
                    s_lstep <= s_lstep + 1;
                when 9 =>
                    if s_c < 0 then s_ma <= resize(unsigned(-s_c), 14); else s_ma <= resize(unsigned(s_c), 14); end if;
                    s_mb <= resize(s_w, 14);                                -- |c| * w
                    s_lstep <= s_lstep + 1;
                when 10 | 11 => s_lstep <= s_lstep + 1;
                when 12 =>
                    if s_c < 0 then
                        v_kk := to_signed(1024, 23) - signed(resize(s_mp(22 downto 9), 23));
                    else
                        v_kk := to_signed(1024, 23) + signed(resize(s_mp(22 downto 8), 23));
                    end if;
                    if v_kk < C_K_MIN then v_kk := to_signed(C_K_MIN, 23); end if;
                    if v_kk > 4095 then v_kk := to_signed(4095, 23); end if;
                    s_f <= unsigned(v_kk(12 downto 0));
                    s_lstep <= s_lstep + 1;
                when 13 =>
                    s_ma <= resize(s_zoom, 14); s_mb <= resize(s_f, 14);     -- kz = zoom * f
                    s_lstep <= s_lstep + 1;
                when 14 | 15 => s_lstep <= s_lstep + 1;
                when 16 =>
                    v_kz := s_mp(24 downto 10);
                    if v_kz > 8191 then s_kz <= to_unsigned(8191, 13); elsif v_kz < 64 then s_kz <= to_unsigned(64, 13); else s_kz <= v_kz(12 downto 0); end if;
                    s_lstep <= s_lstep + 1;
                when 17 =>
                    if s_sw_fit = '1' then s_ma <= s_kw; else s_ma <= to_unsigned(1024, 14); end if;
                    s_mb <= resize(s_kz, 14);                               -- kk = kw * kz
                    s_lstep <= s_lstep + 1;
                when 18 | 19 => s_lstep <= s_lstep + 1;
                when 20 =>
                    v_kkq := s_mp(25 downto 10);
                    if v_kkq > 16383 then s_kk <= (others => '1'); else s_kk <= v_kkq(13 downto 0); end if;
                    s_ma <= resize(s_half_w, 14); s_mb <= resize(s_kz, 14);  -- pg = half_w * kz
                    s_lstep <= s_lstep + 1;
                when 21 | 22 => s_lstep <= s_lstep + 1;
                when 23 =>
                    s_pg <= s_mp(23 downto 0);
                    s_ma <= resize(s_ww(10 downto 1), 14); s_mb <= s_kk;   -- pw = (ww/2) * kk
                    s_lstep <= s_lstep + 1;
                when 24 =>
                    s_initg <= signed(shift_left(resize(s_half_w, 26), 10)) - signed(resize(s_pg, 26)) - signed(resize(s_kz, 26));
                    s_lstep <= s_lstep + 1;
                when 25 => s_lstep <= s_lstep + 1;
                when 26 =>
                    s_pw <= s_mp(24 downto 0);
                    s_lstep <= s_lstep + 1;
                when 27 =>
                    if s_sw_mirror = '1' then
                        s_initw <= signed(shift_left(resize(s_half_w, 26), 10)) + signed(resize(s_pw, 26));
                        s_stepw <= -signed(resize(s_kk, 15));
                    else
                        s_initw <= signed(shift_left(resize(s_half_w, 26), 10)) - signed(resize(s_pw, 26));
                        s_stepw <= signed(resize(s_kk, 15));
                    end if;
                    s_lstep <= s_lstep + 1;
                when others => null;
            end case;

            ------------------------------------------------------------------
            -- Vblank sequencer: knobs, window geometry, dividers, drift
            ------------------------------------------------------------------
            case to_integer(s_vstep) is
                when 0 =>
                    s_half_h  <= resize(s_height(10 downto 1), 11);
                    s_half_w  <= resize(s_line_width(10 downto 1), 11);
                    s_c       <= signed(resize(s_k_curve, 11)) - to_signed(512, 11);
                    s_zoom    <= C_ZOOM_ROM(to_integer(s_k_zoom(9 downto 4)));
                    s_sin_addr <= s_drift_ph(15 downto 8);
                    s_qa <= resize(s_k_noise, 11); s_qb <= resize(s_k_noise, 11);        -- amp = k^2
                    s_vstep <= s_vstep + 1;
                when 1 | 2 => s_vstep <= s_vstep + 1;
                when 3 =>
                    s_amp_eff <= s_vp(19 downto 10);
                    s_qa <= resize(to_unsigned(103, 10) + s_k_size(9 downto 1) + s_k_size(9 downto 2), 11);
                    s_qb <= resize(s_line_width(10 downto 1), 11);                       -- ww
                    s_vstep <= s_vstep + 1;
                when 4 | 5 => s_vstep <= s_vstep + 1;
                when 6 =>
                    if s_vp(19 downto 9) < 16 then s_ww <= to_unsigned(16, 11); else s_ww <= s_vp(19 downto 9); end if;
                    if s_sin_q < 0 then s_qa <= resize(unsigned(-s_sin_q), 11); s_sinneg <= '1';
                    else s_qa <= resize(unsigned(s_sin_q), 11); s_sinneg <= '0'; end if;
                    s_qb <= resize(s_p_drift, 11);                                      -- drift
                    s_vstep <= s_vstep + 1;
                when 7 | 8 => s_vstep <= s_vstep + 1;
                when 9 =>
                    if s_sinneg = '1' then s_drp <= -signed(resize(s_vp(20 downto 0), 22)); else s_drp <= signed(resize(s_vp(20 downto 0), 22)); end if;
                    s_qa <= resize(s_k_pos, 11); s_qb <= s_line_width;                  -- pos
                    s_vstep <= s_vstep + 1;
                when 10 | 11 => s_vstep <= s_vstep + 1;
                when 12 =>
                    v_x := signed(resize(s_vp(20 downto 10), 13)) + resize(shift_right(s_drp, 11), 13);
                    if v_x < 0 then v_x := (others => '0'); elsif v_x > signed(resize(s_line_width, 13)) then v_x := signed(resize(s_line_width, 13)); end if;
                    s_wcx <= unsigned(v_x(10 downto 0));
                    s_qa <= s_ww; s_qb <= s_height;                                      -- whh
                    s_vstep <= s_vstep + 1;
                when 13 | 14 => s_vstep <= s_vstep + 1;
                when 15 =>
                    s_whh <= resize(s_vp(21 downto 12), 11);
                    s_vstep <= s_vstep + 1;
                when 16 =>
                    if s_wcx >= s_ww(10 downto 1) then s_wx0 <= s_wcx - s_ww(10 downto 1); else s_wx0 <= (others => '0'); end if;
                    s_wx1 <= s_wcx + s_ww(10 downto 1);
                    if s_half_h >= s_whh then s_wy0 <= s_half_h - s_whh; else s_wy0 <= (others => '0'); end if;
                    s_wy1 <= s_half_h + s_whh;
                    s_div_n <= "100000000000000000000";
                    s_div_d <= s_half_h;
                    s_div_rem <= (others => '0'); s_div_q <= (others => '0'); s_div_i <= (others => '0');
                    s_vstep <= s_vstep + 1;
                when 17 | 20 =>
                    v_rem := s_div_rem & s_div_n(20 - to_integer(s_div_i));
                    if v_rem >= resize(s_div_d, 13) then
                        s_div_rem <= resize(v_rem - resize(s_div_d, 13), 12);
                        s_div_q   <= s_div_q(19 downto 0) & '1';
                    else
                        s_div_rem <= v_rem(11 downto 0);
                        s_div_q   <= s_div_q(19 downto 0) & '0';
                    end if;
                    if s_div_i = 20 then
                        s_vstep <= s_vstep + 1;
                    end if;
                    s_div_i <= s_div_i + 1;
                when 18 =>
                    if s_half_h < 16 then s_inv_hh <= (others => '0'); else s_inv_hh <= s_div_q(13 downto 0); end if;
                    s_vstep <= s_vstep + 1;
                when 19 =>
                    s_div_n <= resize(s_line_width, 21) sll 10;
                    s_div_d <= s_ww;
                    s_div_rem <= (others => '0'); s_div_q <= (others => '0'); s_div_i <= (others => '0');
                    s_vstep <= s_vstep + 1;
                when 21 =>
                    if s_div_q > 16383 then s_kw <= (others => '1'); else s_kw <= s_div_q(13 downto 0); end if;
                    s_vstep <= "111111";
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
                s_line   <= s_line + 1;
                s_height <= s_line + 1;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar  <= not s_wpar;
                s_rbank <= s_wpar;
                s_lstep <= (others => '0');
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0');
                s_wpar <= '0';
                if s_saw_active = '1' then
                    s_frame <= s_frame + 1;
                    s_oy    <= s_oy + resize(s_k_speed(9 downto 6), 16);
                    s_drift_ph <= s_drift_ph + resize(s_p_drift(9 downto 4), 16) + 16;
                    s_vstep <= (others => '0');
                end if;
                s_saw_active <= '0';
            end if;

            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Address chain 2..4
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_w : signed(14 downto 0);
        variable v_u : signed(14 downto 0);
        variable v_in : std_logic;
    begin
        if rising_edge(clk) then
            -- stage 2
            s_ug2 <= s_accg(24 downto 10);
            s_uw2 <= s_accw(24 downto 10);
            s_n2  <= s_nacc(15 downto 6);
            if s_inrows = '1' and s_x1 >= s_wx0 and s_x1 < s_wx1 then v_in := '1'; else v_in := '0'; end if;
            s_inwin2 <= v_in;
            if v_in = '1' and (s_x1 < s_wx0 + 4 or s_x1 >= s_wx1 - 4 or s_line < s_wy0 + 4 or s_line >= s_wy1 - 4) then
                s_bord2 <= '1';
            else
                s_bord2 <= '0';
            end if;
            s_x2 <= s_x1;
            -- stage 3: select map, add noise
            if s_inwin2 = '1' then
                v_u := s_uw2 + resize(s_n2, 15);
            elsif s_sw_outside = '1' then
                v_u := s_ug2;
                if s_sw_noise_all = '1' then v_u := v_u + resize(s_n2, 15); end if;
            else
                v_u := signed(resize(s_x2, 15));
                if s_sw_noise_all = '1' then v_u := v_u + resize(s_n2, 15); end if;
            end if;
            s_u3 <= v_u;
            s_bord3 <= s_bord2 and s_sw_border;
            -- stage 4: range
            v_w := signed(resize(s_line_width, 15));
            if s_u3 < 0 or s_u3 >= v_w then s_bg <= '1'; else s_bg <= '0'; end if;
            s_addr <= unsigned(s_u3(10 downto 0));
            s_bord4 <= s_bord3;
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (read at stage 5)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr));
            if s_we = '1' and s_wpar = '0' then lbY0(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr));
            if s_we = '1' and s_wpar = '1' then lbY1(to_integer(s_wr_x)) <= std_logic_vector(s_in_y); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbU0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbU1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_u(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '0' then lbV0(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr(10 downto 1)));
            if s_we = '1' and s_wpar = '1' then lbV1(to_integer(s_wr_x(10 downto 1))) <= std_logic_vector(s_in_v(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stages 5..12
    --------------------------------------------------------------------------
    p_pix : process(clk)
    begin
        if rising_edge(clk) then
            s_a_bg <= s_bg; s_a_rbank <= s_rbank; s_a_bord <= s_bord4;
            if s_a_bg = '1' or s_a_bord = '1' then
                s_wet_y <= to_unsigned(64, 10); s_wet_u <= to_unsigned(512, 10); s_wet_v <= to_unsigned(512, 10);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            s_yd(7) <= s_wet_y; s_ud(7) <= s_wet_u; s_vd(7) <= s_wet_v;
            for i in 8 to 11 loop
                s_yd(i) <= s_yd(i - 1); s_ud(i) <= s_ud(i - 1); s_vd(i) <= s_vd(i - 1);
            end loop;
            if s_avid_sr(10) = '1' then
                s_y12 <= s_yd(11); s_u12 <= s_ud(11); s_v12 <= s_vd(11);
            else
                s_y12 <= to_unsigned(64, 10); s_u12 <= to_unsigned(512, 10); s_v12 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y12);
    data_out.u       <= std_logic_vector(s_u12);
    data_out.v       <= std_logic_vector(s_v12);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture porthole;
