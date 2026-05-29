-- Sobeledge: Sobel edge detector with false-color magnitude.
--
-- A 3-line BRAM buffer holds the previous two scanlines of Y.  Three
-- Y samples per row are needed:  rowN-2, rowN-1, rowN.  Combined with
-- 3 horizontal samples (current, x-1, x-2) this gives the 3×3
-- neighborhood needed for the Sobel kernels:
--     Gx = -1 0 +1     Gy = -1 -2 -1
--          -2 0 +2           0  0  0
--          -1 0 +1          +1 +2 +1
-- |G| ≈ |Gx| + |Gy| is then thresholded against the Threshold slider.
-- Above-threshold pixels light up in either monochrome or "false color"
-- mode (palette indexed by edge magnitude).  Background can be dimmed
-- or hidden entirely.
--
-- BRAM: 2 line buffers (rowN-1 and rowN-2) × 1 BRAM each = 2 BRAMs.
-- Designed for 74.25 MHz HD timing on iCE40-HX4K.
--
-- Register map:
--   registers_in(0) = Gx Gain
--   registers_in(1) = Gy Gain
--   registers_in(2) = Background (0=hidden, large=full source)
--   registers_in(3) = Edge Color (palette shift)
--   registers_in(4) = Brightness
--   registers_in(5) = Saturation
--   registers_in(6) = Switches:
--     b0 Mode      (0=Color edge / 1=Mono)
--     b1 Polarity  (0=Bright / 1=Dark edges)
--     b2 Background (0=Show / 1=Hide)
--     b3 False Color (0=Off / 1=On)
--     b4 Invert
--   registers_in(7) = Threshold (slider, KEY)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;

architecture sobeledge of program_top is

    constant LATENCY    : natural := 16;
    constant C_BUF_DEPTH : integer := 11;
    constant C_BUF_SIZE  : integer := 2**C_BUF_DEPTH;

    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal pixel_y      : unsigned(11 downto 0) := (others => '0');
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal prev_avid    : std_logic := '0';

    -- Param latches
    signal gx_gain_r  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal gy_gain_r  : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal bg_r       : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal edge_col_r : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal bright_r   : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal sat_r      : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal thresh_r   : unsigned(9 downto 0) := to_unsigned(256, 10);
    signal mono_r     : std_logic := '0';
    signal dark_r     : std_logic := '0';
    signal hide_bg_r  : std_logic := '0';
    signal false_r    : std_logic := '1';
    signal invert_r   : std_logic := '0';

    -- Active pixel counter
    signal active_x       : unsigned(C_BUF_DEPTH - 1 downto 0) := (others => '0');

    -- BRAM line buffers
    type t_bram is array (0 to C_BUF_SIZE - 1)
        of std_logic_vector(9 downto 0);
    signal bram_l1 : t_bram := (others => (others => '0'));
    signal bram_l2 : t_bram := (others => (others => '0'));

    -- Bank toggle: alternate which BRAM is "rowN-1" each line.
    signal bank_r : std_logic := '0';

    -- Read addr and registers
    signal rd_l1 : std_logic_vector(9 downto 0) := (others => '0');
    signal rd_l2 : std_logic_vector(9 downto 0) := (others => '0');

    -- Pipeline signals
    -- s0: register pixel Y + neighbors (3 horizontal: c, x-1, x-2).
    signal s0_y_c : unsigned(9 downto 0);   -- current
    signal s0_y_l : unsigned(9 downto 0);   -- prev pixel (x-1)
    signal s0_y_ll: unsigned(9 downto 0);   -- (x-2)
    signal s0_y_in : unsigned(9 downto 0);

    -- Vertical neighbors: from BRAM reads, registered.
    -- rd_l1 = row N-1 at addr (current x — 1 cycle ahead of bram read latch).
    -- For 3×3 we need 9 samples; with horizontal shift registers we hold
    -- (c, l, ll) for row N, N-1, N-2.

    -- Maintain horizontal shift registers for the BRAM read outputs.
    signal s1_n_c : unsigned(9 downto 0);  -- row N (current line, current pixel)
    signal s1_n_l : unsigned(9 downto 0);  -- N x-1
    signal s1_n_ll: unsigned(9 downto 0);  -- N x-2
    signal s1_m_c : unsigned(9 downto 0);  -- row N-1 c
    signal s1_m_l : unsigned(9 downto 0);  -- N-1 l
    signal s1_m_ll: unsigned(9 downto 0);  -- N-1 ll
    signal s1_p_c : unsigned(9 downto 0);  -- row N-2 c
    signal s1_p_l : unsigned(9 downto 0);  -- N-2 l
    signal s1_p_ll: unsigned(9 downto 0);  -- N-2 ll

    -- S2: Sobel sums.  Signed because of subtractions.
    -- gx = -ll_m2 -2*l_m1 -ll_m0  +  c_m2 +2*c_m1 +c_m0
    -- Actually using the proper kernel:
    -- Gx = (P(N-2,x-2)*-1 + P(N-2,x)*+1 + P(N-1,x-2)*-2 + P(N-1,x)*+2 + P(N,x-2)*-1 + P(N,x)*+1)
    -- Equivalent: Gx = (n_c + 2*m_c + p_c) - (n_ll + 2*m_ll + p_ll)
    -- Gy = (p_ll + 2*p_l + p_c) - (n_ll + 2*n_l + n_c)
    signal s2_gx_pos : unsigned(11 downto 0);  -- sum positives
    signal s2_gx_neg : unsigned(11 downto 0);
    signal s2_gy_pos : unsigned(11 downto 0);
    signal s2_gy_neg : unsigned(11 downto 0);
    signal s2_y_center : unsigned(9 downto 0);

    -- S3: signed gx, gy.
    signal s3_gx : signed(12 downto 0);
    signal s3_gy : signed(12 downto 0);
    signal s3_y_center : unsigned(9 downto 0);

    -- S4: abs gx, abs gy.
    signal s4_agx : unsigned(11 downto 0);
    signal s4_agy : unsigned(11 downto 0);
    signal s4_y_center : unsigned(9 downto 0);

    -- S5: scaled gx*gx_gain, gy*gy_gain (registered raw).
    signal s5_agx_mul : unsigned(21 downto 0);
    signal s5_agy_mul : unsigned(21 downto 0);
    signal s5_y_center : unsigned(9 downto 0);

    -- S6: mag = (agx + agy) >> 10.
    signal s6_mag : unsigned(10 downto 0);
    signal s6_y_center : unsigned(9 downto 0);

    -- S7: clamp mag to 10-bit; threshold mask.
    signal s7_mag : unsigned(9 downto 0);
    signal s7_hit : std_logic;
    signal s7_y_center : unsigned(9 downto 0);

    -- S8: palette lookup + background mix.
    signal s8_y : unsigned(9 downto 0);
    signal s8_u : unsigned(9 downto 0);
    signal s8_v : unsigned(9 downto 0);

    -- S9: brightness multiply.
    signal s9_y_mul : unsigned(19 downto 0);
    signal s9_u     : unsigned(9 downto 0);
    signal s9_v     : unsigned(9 downto 0);

    -- S10: extract Y; sat mul.
    signal s10_y     : unsigned(9 downto 0);
    signal s10_u_mul : signed(22 downto 0);
    signal s10_v_mul : signed(22 downto 0);

    -- S11: re-center.
    signal s11_y : unsigned(9 downto 0);
    signal s11_u : unsigned(9 downto 0);
    signal s11_v : unsigned(9 downto 0);

    -- S12: slack + invert.
    signal s12_y : unsigned(9 downto 0);
    signal s12_u : unsigned(9 downto 0);
    signal s12_v : unsigned(9 downto 0);
    signal s13_y : unsigned(9 downto 0);
    signal s13_u : unsigned(9 downto 0);
    signal s13_v : unsigned(9 downto 0);
    signal s14_y : unsigned(9 downto 0);
    signal s14_u : unsigned(9 downto 0);
    signal s14_v : unsigned(9 downto 0);

    -- 8-entry false-color palette (cyan→magenta hot scale).
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant C_PY_FALSE : t_pal := (
        to_unsigned( 80, 10), to_unsigned(192, 10),
        to_unsigned(349, 10), to_unsigned(580, 10),
        to_unsigned(680, 10), to_unsigned(840, 10),
        to_unsigned(900, 10), to_unsigned(1000, 10));
    constant C_PU_FALSE : t_pal := (
        to_unsigned(640, 10), to_unsigned(608, 10),
        to_unsigned(756, 10), to_unsigned(136, 10),
        to_unsigned(440, 10), to_unsigned(584, 10),
        to_unsigned(540, 10), to_unsigned(512, 10));
    constant C_PV_FALSE : t_pal := (
        to_unsigned(216, 10), to_unsigned(696, 10),
        to_unsigned(852, 10), to_unsigned(216, 10),
        to_unsigned(960, 10), to_unsigned( 64, 10),
        to_unsigned(540, 10), to_unsigned(512, 10));

    function sat10_s(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10); end if;
        if v > 1023 then return to_unsigned(1023, 10); end if;
        return unsigned(std_logic_vector(resize(v, 10)));
    end function;

    function abs_s13(v : signed(12 downto 0)) return unsigned is
        variable t : signed(12 downto 0);
    begin
        if v(12) = '1' then t := -v; else t := v; end if;
        return unsigned(std_logic_vector(t(11 downto 0)));
    end function;

begin

    -- ========================================================================
    -- Position + param latch + active-x tracking + bank toggle on hsync
    -- ========================================================================
    p_position : process(clk)
        variable v_h_edge : std_logic;
        variable v_v_edge : std_logic;
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            prev_avid    <= data_in.avid;
            v_h_edge := '0';
            v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            if v_h_edge = '1' then
                pixel_x <= (others => '0');
                active_x <= (others => '0');
                bank_r <= not bank_r;
            else
                if data_in.avid = '1' and prev_avid = '0' then
                    active_x <= (others => '0');
                elsif data_in.avid = '1' then
                    active_x <= active_x + 1;
                end if;
                if data_in.avid = '1' then
                    pixel_x <= pixel_x + 1;
                end if;
            end if;

            if v_v_edge = '1' then
                pixel_y <= (others => '0');

                gx_gain_r  <= unsigned(registers_in(0));
                gy_gain_r  <= unsigned(registers_in(1));
                bg_r       <= unsigned(registers_in(2));
                edge_col_r <= unsigned(registers_in(3));
                bright_r   <= unsigned(registers_in(4));
                sat_r      <= unsigned(registers_in(5));
                mono_r     <= registers_in(6)(0);
                dark_r     <= registers_in(6)(1);
                hide_bg_r  <= registers_in(6)(2);
                false_r    <= registers_in(6)(3);
                invert_r   <= registers_in(6)(4);
                thresh_r   <= unsigned(registers_in(7));
            elsif v_h_edge = '1' then
                pixel_y <= pixel_y + 1;
            end if;
        end if;
    end process p_position;

    -- ========================================================================
    -- BRAM: two line buffers (alternating bank).  Each cycle: write current
    -- Y into bram[bank], read previous lines from the other bank.
    -- bank toggles at hsync.
    -- ========================================================================
    p_bram_w1 : process(clk)
    begin
        if rising_edge(clk) then
            if bank_r = '0' then
                bram_l1(to_integer(active_x)) <= data_in.y;
            end if;
        end if;
    end process p_bram_w1;

    p_bram_w2 : process(clk)
    begin
        if rising_edge(clk) then
            if bank_r = '1' then
                bram_l2(to_integer(active_x)) <= data_in.y;
            end if;
        end if;
    end process p_bram_w2;

    p_bram_r1 : process(clk)
    begin
        if rising_edge(clk) then
            rd_l1 <= bram_l1(to_integer(active_x));
        end if;
    end process p_bram_r1;

    p_bram_r2 : process(clk)
    begin
        if rising_edge(clk) then
            rd_l2 <= bram_l2(to_integer(active_x));
        end if;
    end process p_bram_r2;

    -- ========================================================================
    -- Pipeline
    -- ========================================================================
    p_pipe : process(clk)
        variable v_pidx : integer range 0 to 7;
        variable v_y_b  : unsigned(10 downto 0);
        variable v_u_c  : signed(11 downto 0);
        variable v_v_c  : signed(11 downto 0);
        variable v_u_re : signed(13 downto 0);
        variable v_v_re : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop
                pipe(i) <= pipe(i - 1);
            end loop;

            -- S0: register raw Y + horizontal shift registers
            s0_y_in <= unsigned(data_in.y);
            s0_y_c  <= unsigned(data_in.y);
            s0_y_l  <= s0_y_c;
            s0_y_ll <= s0_y_l;

            -- S1: 3x3 neighborhood.  Use bank_r to pick which BRAM read is
            -- N-1 vs N-2.  When bank=0, BRAM l1 is being written this line
            -- so it holds line N-2; l2 holds N-1.  When bank=1, the reverse.
            s1_n_c  <= s0_y_c;
            s1_n_l  <= s0_y_l;
            s1_n_ll <= s0_y_ll;
            if bank_r = '0' then
                -- l2 holds N-1, l1 holds N-2.
                s1_m_c  <= unsigned(rd_l2);
                s1_p_c  <= unsigned(rd_l1);
            else
                s1_m_c  <= unsigned(rd_l1);
                s1_p_c  <= unsigned(rd_l2);
            end if;
            -- For (l) and (ll) need horizontal shifts of the above
            -- registered every cycle.  Shift them through here too:
            s1_m_l  <= s1_m_c;
            s1_m_ll <= s1_m_l;
            s1_p_l  <= s1_p_c;
            s1_p_ll <= s1_p_l;

            -- S2: Sobel positives / negatives (registered, additions
            -- broken across two stages to limit carry chain length).
            -- gx_pos = n_c + 2*m_c + p_c
            -- gx_neg = n_ll + 2*m_ll + p_ll
            s2_gx_pos <= resize(s1_n_c, 12) + (s1_m_c & '0') +
                         resize(s1_p_c, 12);
            s2_gx_neg <= resize(s1_n_ll, 12) + (s1_m_ll & '0') +
                         resize(s1_p_ll, 12);
            -- gy_pos = p_ll + 2*p_l + p_c
            -- gy_neg = n_ll + 2*n_l + n_c
            s2_gy_pos <= resize(s1_p_ll, 12) + (s1_p_l & '0') +
                         resize(s1_p_c, 12);
            s2_gy_neg <= resize(s1_n_ll, 12) + (s1_n_l & '0') +
                         resize(s1_n_c, 12);
            s2_y_center <= s1_m_l;  -- center pixel (rowN-1, x-1)

            -- S3: signed gx, gy.
            s3_gx <= signed('0' & std_logic_vector(s2_gx_pos)) -
                     signed('0' & std_logic_vector(s2_gx_neg));
            s3_gy <= signed('0' & std_logic_vector(s2_gy_pos)) -
                     signed('0' & std_logic_vector(s2_gy_neg));
            s3_y_center <= s2_y_center;

            -- S4: abs gx, abs gy.
            s4_agx <= abs_s13(s3_gx);
            s4_agy <= abs_s13(s3_gy);
            s4_y_center <= s3_y_center;

            -- S5: scaled magnitudes (registered raw).
            s5_agx_mul <= s4_agx * gx_gain_r;
            s5_agy_mul <= s4_agy * gy_gain_r;
            s5_y_center <= s4_y_center;

            -- S6: combine.
            s6_mag <= resize(s5_agx_mul(21 downto 11) +
                              s5_agy_mul(21 downto 11), 11);
            s6_y_center <= s5_y_center;

            -- S7: clamp + threshold.
            if s6_mag > 1023 then
                s7_mag <= to_unsigned(1023, 10);
            else
                s7_mag <= s6_mag(9 downto 0);
            end if;
            if s6_mag(9 downto 0) > thresh_r then
                s7_hit <= '1';
            else
                s7_hit <= '0';
            end if;
            s7_y_center <= s6_y_center;

            -- S8: composite.  Edge ? palette[mag]/bright : (background or dim).
            if s7_hit = '1' then
                if false_r = '1' then
                    v_pidx := to_integer(s7_mag(9 downto 7) + edge_col_r(9 downto 7));
                    s8_y <= C_PY_FALSE(v_pidx);
                    s8_u <= C_PU_FALSE(v_pidx);
                    s8_v <= C_PV_FALSE(v_pidx);
                else
                    if dark_r = '0' then
                        s8_y <= s7_mag;
                    else
                        s8_y <= to_unsigned(1023, 10) - s7_mag;
                    end if;
                    s8_u <= to_unsigned(512, 10);
                    s8_v <= to_unsigned(512, 10);
                end if;
            else
                if hide_bg_r = '1' then
                    s8_y <= to_unsigned(0, 10);
                else
                    -- Show the source dimmed by bg_r.
                    v_y_b := ('0' & s7_y_center) +
                             ('0' & bg_r);
                    if v_y_b > 1023 then
                        s8_y <= to_unsigned(1023, 10);
                    else
                        s8_y <= v_y_b(9 downto 0);
                    end if;
                end if;
                s8_u <= to_unsigned(512, 10);
                s8_v <= to_unsigned(512, 10);
            end if;

            -- S9: brightness multiply.
            s9_y_mul <= s8_y * bright_r;
            s9_u     <= s8_u;
            s9_v     <= s8_v;

            -- S10: extract Y; sat mul.
            s10_y <= s9_y_mul(19 downto 10);
            v_u_c := signed('0' & std_logic_vector(s9_u)) - to_signed(512, 12);
            v_v_c := signed('0' & std_logic_vector(s9_v)) - to_signed(512, 12);
            s10_u_mul <= v_u_c * signed('0' & std_logic_vector(sat_r));
            s10_v_mul <= v_v_c * signed('0' & std_logic_vector(sat_r));

            -- S11: re-center + clamp.
            s11_y <= s10_y;
            v_u_re := resize(shift_right(s10_u_mul, 10), 14) +
                      to_signed(512, 14);
            v_v_re := resize(shift_right(s10_v_mul, 10), 14) +
                      to_signed(512, 14);
            s11_u <= sat10_s(v_u_re);
            s11_v <= sat10_s(v_v_re);

            -- S12-S13 slack + S14 invert.
            s12_y <= s11_y; s12_u <= s11_u; s12_v <= s11_v;
            s13_y <= s12_y; s13_u <= s12_u; s13_v <= s12_v;
            if invert_r = '1' then
                s14_y <= to_unsigned(1023, 10) - s13_y;
            else
                s14_y <= s13_y;
            end if;
            s14_u <= s13_u;
            s14_v <= s13_v;
        end if;
    end process p_pipe;

    data_out.y       <= std_logic_vector(s14_y);
    data_out.u       <= std_logic_vector(s14_u);
    data_out.v       <= std_logic_vector(s14_v);
    data_out.avid    <= pipe(LATENCY - 1).avid;
    data_out.hsync_n <= pipe(LATENCY - 1).hsync_n;
    data_out.vsync_n <= pipe(LATENCY - 1).vsync_n;
    data_out.field_n <= pipe(LATENCY - 1).field_n;

end architecture sobeledge;
