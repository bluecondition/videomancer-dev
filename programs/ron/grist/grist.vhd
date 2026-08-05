-- grist.vhd
--
-- GRIST -- the grain from MORSEL's cookie dough, promoted to the whole
-- instrument.  Nothing else survived the fork: no cells, no chips, no dough.
--
-- The engine is morsel's surface noise: a 16-bit xor-rotate-add hash of the
-- live pixel counters giving a per-pixel fine tooth, the same hash on
-- shifted coordinates giving block-scale mottle, and a sparse threshold test
-- giving pores.  It is a STATIC function of (x, y) -- a material texture,
-- not TV snow -- unless S7 folds a scrambled frame counter into the seeds,
-- which turns the same texture into living film grain.
--
-- The whole program is one comparator:
--
--     out = clamp( 512 + (Y - thr + N) * gain )
--
-- where N is the octave-mixed grain field and gain rides P12 (DEVELOP).
-- At gain = 1 the multiply is transparent and the program is additive film
-- grain; as gain rises the video's tones slam toward black and white with
-- the grain as dither, and the picture re-renders itself as a MEZZOTINT --
-- pure stipple density, the standout of the ext_*.png exploration.  One
-- multiply covers the entire clean -> filmic -> engraving journey.
--
-- Panel:
--   K1 FINE     per-pixel tooth amount
--   K2 COARSE   block-mottle amount (4 px mid octave + K3-sized octave)
--   K3 BLOCK    coarse octave cell size, 8/16/32/64 px
--   K4 RESPONSE 0 = uniform grain, up = film-stock mid-tone weighting
--               (shadows and highlights print clean)
--   K5 PITS     pore density; S8 flips dark pits <-> bright sparkle
--   K6 EXPOSURE comparator midpoint: darker or lighter print
--   P12 DEVELOP additive grain -> hard mezzotint (the climax)
--   S7 ANIMATE  frozen etching <-> living grain (frame-seeded)
--   S8 SPARKLE  pore polarity
--   S9 MONO     B/W print vs the video's own chroma
--   S10 ETCH    3-bit posterize after the comparator (boiling dithered
--               contours between flat plates)
--   S11 INVERT  negative print
--
-- Pipeline (streaming, C_LATENCY = 10):
--   1     coordinate registers (block shifts registered BEFORE the hash --
--         morsel timing lesson) and the frame-row map for interlace
--   2-3   three hashes: fine / 4 px / K3 block (mix1 then mix2)
--   4     small grain multiplies (5x5, 6x5) and the pore test
--   5     octave sum + pore add
--   6     tonal-response multiply (11x5), weight from the luma delay line
--   7     D = (Y - 512) + grain + exposure bias
--   8     the develop multiply D * m (12x7); gain is m << e, normalised at
--         vblank so the pixel path multiplies by a 6-bit mantissa only
--   9     barrel shift << e, >> 4, clamp
--   10    512 + result, clamp, etch / invert / mono muxes
-- Multiplies: three small grain gains, the response weight, and the develop
-- mantissa.  Everything else is compares, shifts and adds.  No BRAM, no ROM.
--
-- Interlace: the hash row is the FRAME row -- (line << 1) | field, with the
-- offset on field_n = '0' (bottom field carries the odd rows; seeding the
-- opposite way was ribbon's crawling-comb hardware bug).  Progressive modes
-- use the line counter directly so block sizes stay true.
--
-- Chroma is passthrough (or 512 in MONO), so the hardware U/V swap is
-- irrelevant here -- both channels are treated identically.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture grist of program_top is

    constant C_LATENCY : integer := 10;

    ----------------------------------------------------------------------
    -- helpers
    ----------------------------------------------------------------------
    function f_cu10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return resize(unsigned(v), 10);
        end if;
    end function;

    -- 16-bit xor-rotate-add hash, bit-identical to morsel's.  The rotate is
    -- load-bearing: without it (a xor b) depends only on x xor y and every
    -- cell on a diagonal shares a hash (see the vhdl-hash-pack trap).
    function f_pack(x, y : unsigned(11 downto 0);
                    seed : unsigned(15 downto 0)) return unsigned is
        variable a, b : unsigned(15 downto 0);
    begin
        a := x(9 downto 0) & y(5 downto 0);
        b := y(9 downto 0) & x(5 downto 0);
        return (a xor rotate_left(b, 5)) xor seed;
    end function;

    function f_mix1(a : unsigned(15 downto 0)) return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := a;
        h := h xor rotate_left(h, 7);
        h := h + rotate_left(h, 3);
        return h;
    end function;

    function f_mix2(a : unsigned(15 downto 0)) return unsigned is
        variable h : unsigned(15 downto 0);
    begin
        h := a;
        h := h xor rotate_left(h, 11);
        h := h + rotate_left(h, 5);
        return h;
    end function;

    ----------------------------------------------------------------------
    -- raster measurement
    ----------------------------------------------------------------------
    signal s_xcnt : unsigned(11 downto 0) := (others => '0');
    signal s_lcnt : unsigned(10 downto 0) := (others => '0');
    signal s_avid_q      : std_logic := '0';
    signal s_prev_vsync  : std_logic := '1';
    signal s_vs_pulse    : std_logic := '0';
    signal s_fpar        : std_logic := '0';
    signal s_ilace       : std_logic := '0';
    signal s_frame : unsigned(15 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- frame-latched controls + per-frame terms
    ----------------------------------------------------------------------
    signal s_k1, s_k2, s_k3 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_k4, s_k5, s_k6 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal s_p12 : unsigned(9 downto 0) := to_unsigned(300, 10);
    signal s_anim, s_spark, s_mono, s_etch, s_inv : std_logic := '0';
    signal s_seq : unsigned(3 downto 0) := (others => '0');

    signal s_gf, s_gc : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal s_msc  : integer range 3 to 6 := 4;         -- coarse block shift
    signal s_tone : unsigned(3 downto 0) := (others => '0');
    signal s_pthr : unsigned(3 downto 0) := (others => '0');
    signal s_bias : signed(9 downto 0) := (others => '0');
    signal s_p8   : unsigned(7 downto 0) := to_unsigned(75, 8);
    signal s_g10  : unsigned(10 downto 0) := to_unsigned(16, 11);
    signal s_m    : unsigned(5 downto 0) := to_unsigned(16, 6);
    signal s_e    : integer range 0 to 5 := 0;
    signal s_fs1, s_fs2 : unsigned(15 downto 0) := (others => '0');
    signal s_sa : unsigned(15 downto 0) := x"1234";
    signal s_sb : unsigned(15 downto 0) := x"77A1";
    signal s_sc : unsigned(15 downto 0) := x"C65D";

    ----------------------------------------------------------------------
    -- pixel pipeline
    ----------------------------------------------------------------------
    signal c1_xa, c1_ya, c1_xb, c1_yb, c1_xc, c1_yc :
        unsigned(11 downto 0) := (others => '0');
    signal c2_a, c2_b, c2_c : unsigned(15 downto 0) := (others => '0');
    signal c3_a, c3_b, c3_c : unsigned(15 downto 0) := (others => '0');
    signal c4_g0, c4_g1, c4_g2 : signed(8 downto 0) := (others => '0');
    signal c4_pore : std_logic := '0';
    signal c5_n2 : signed(10 downto 0) := (others => '0');
    signal t1_w5 : unsigned(4 downto 0) := (others => '0');
    signal t2_w  : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal c6_m  : signed(11 downto 0) := (others => '0');
    signal c7_d  : signed(12 downto 0) := (others => '0');
    signal c8_p  : signed(19 downto 0) := (others => '0');
    signal c9_r  : signed(11 downto 0) := (others => '0');
    signal c10_y, c10_u, c10_v : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- video + sync delay
    ----------------------------------------------------------------------
    type t_sr is array (0 to 8) of std_logic_vector(9 downto 0);
    signal s_y_sr, s_u_sr, s_v_sr : t_sr := (others => (others => '0'));

    signal s_avid_sr    : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '1');
    signal s_field_n_sr : std_logic_vector(0 to C_LATENCY - 1) := (others => '0');

begin

    ------------------------------------------------------------------------
    -- raster measurement
    ------------------------------------------------------------------------
    p_measure : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_vsync <= data_in.vsync_n;
            s_vs_pulse   <= '0';
            if data_in.vsync_n = '0' and s_prev_vsync = '1' then
                s_vs_pulse <= '1';
            end if;

            s_avid_q <= data_in.avid;
            if data_in.avid = '1' then
                s_xcnt <= s_xcnt + 1;
            elsif s_avid_q = '1' then
                s_xcnt <= (others => '0');
                s_lcnt <= s_lcnt + 1;
            end if;
            if s_vs_pulse = '1' then
                s_lcnt  <= (others => '0');
                s_frame <= s_frame + 1;
                -- interlace = field_n alternates between vertical syncs
                s_fpar <= data_in.field_n;
                if data_in.field_n /= s_fpar then s_ilace <= '1';
                else                              s_ilace <= '0'; end if;
            end if;
        end if;
    end process p_measure;

    ------------------------------------------------------------------------
    -- per-frame control latch + vblank sequencer
    -- (everything with a ladder in it lives here -- a vblank cone is timed
    -- at the pixel clock like everything else; morsel lesson)
    ------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_p10 : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            if s_vs_pulse = '1' then
                s_k1  <= unsigned(registers_in(0));
                s_k2  <= unsigned(registers_in(1));
                s_k3  <= unsigned(registers_in(2));
                s_k4  <= unsigned(registers_in(3));
                s_k5  <= unsigned(registers_in(4));
                s_k6  <= unsigned(registers_in(5));
                s_p12 <= unsigned(registers_in(7));
                s_anim  <= registers_in(6)(0);            -- S7
                s_spark <= registers_in(6)(1);            -- S8
                s_mono  <= registers_in(6)(2);            -- S9
                s_etch  <= registers_in(6)(3);            -- S10
                s_inv   <= registers_in(6)(4);            -- S11
                s_seq <= to_unsigned(15, 4);
            elsif s_seq /= 0 and data_in.avid = '0' then
                case to_integer(s_seq) is
                    when 15 =>
                        s_gf <= s_k1(9 downto 6);
                        s_gc <= s_k2(9 downto 6);
                    when 14 =>
                        -- K3 BLOCK: coarse octave 8/16/32/64 px
                        if    s_k3 < 256 then s_msc <= 3;
                        elsif s_k3 < 512 then s_msc <= 4;
                        elsif s_k3 < 768 then s_msc <= 5;
                        else                  s_msc <= 6; end if;
                    when 13 =>
                        s_tone <= s_k4(9 downto 6);
                        s_pthr <= s_k5(9 downto 6);
                    when 12 =>
                        -- K6 EXPOSURE: +-256 around the 512 midpoint
                        s_bias <= resize(
                            shift_right(signed(resize(s_k6, 11)) - 512, 1), 10);
                    when 11 =>
                        -- DEADBAND: the fader ADC dithers an LSB or two per
                        -- frame, and at high gain the comparator would make
                        -- that visible as frame-rate shimmer (morsel lesson)
                        if s_p12(9 downto 2) > s_p8 + 1
                           or s_p12(9 downto 2) + 1 < s_p8 then
                            s_p8 <= s_p12(9 downto 2);
                        end if;
                    when 10 =>
                        -- P12 DEVELOP -> linear gain, slow first half then
                        -- steep: 16..144 over the bottom, 144..1164 over the
                        -- top (gain/16 = 1x .. ~73x)
                        v_p10 := shift_left(resize(s_p8, 11), 2);
                        if v_p10 < 512 then
                            s_g10 <= to_unsigned(16, 11)
                                     + resize(v_p10(10 downto 2), 11);
                        else
                            s_g10 <= to_unsigned(144, 11)
                                     + shift_left(v_p10 - 512, 1);
                        end if;
                    when 9 =>
                        -- normalise gain to m << e so the pixel path only
                        -- ever multiplies by the 6-bit mantissa
                        if    s_g10 < 64 then
                            s_e <= 0; s_m <= s_g10(5 downto 0);
                        elsif s_g10 < 128 then
                            s_e <= 1; s_m <= s_g10(6 downto 1);
                        elsif s_g10 < 256 then
                            s_e <= 2; s_m <= s_g10(7 downto 2);
                        elsif s_g10 < 512 then
                            s_e <= 3; s_m <= s_g10(8 downto 3);
                        elsif s_g10 < 1024 then
                            s_e <= 4; s_m <= s_g10(9 downto 4);
                        else
                            s_e <= 5; s_m <= to_unsigned(63, 6);
                        end if;
                    when 8 =>
                        s_fs1 <= f_mix1(s_frame);
                    when 7 =>
                        s_fs2 <= f_mix2(s_fs1);
                    when 6 =>
                        if s_anim = '1' then
                            s_sa <= x"1234" xor s_fs2;
                            s_sb <= x"77A1" xor s_fs2;
                            s_sc <= x"C65D" xor s_fs2;
                        else
                            s_sa <= x"1234";
                            s_sb <= x"77A1";
                            s_sc <= x"C65D";
                        end if;
                    when others =>
                        null;
                end case;
                s_seq <= s_seq - 1;
            end if;
        end if;
    end process p_frame;

    ------------------------------------------------------------------------
    -- GRAIN FIELD: three hash octaves, pores, tonal response
    ------------------------------------------------------------------------
    p_noise : process(clk)
        variable v_y12 : unsigned(11 downto 0);
        variable v_n5  : signed(4 downto 0);
        variable v_n6  : signed(5 downto 0);
        variable v_sum : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 1: coordinates.  Interlace: hash the FRAME row -- offset on
            -- field_n = '0' (bottom field carries the odd rows).  The block
            -- shifts are registered here, before the hash (morsel lesson).
            if s_ilace = '1' then
                v_y12 := shift_left(resize(s_lcnt, 12), 1);
                if data_in.field_n = '0' then
                    v_y12(0) := '1';
                end if;
            else
                v_y12 := resize(s_lcnt, 12);
            end if;
            c1_xa <= s_xcnt;
            c1_ya <= v_y12;
            c1_xb <= shift_right(s_xcnt, 2);
            c1_yb <= shift_right(v_y12, 2);
            c1_xc <= shift_right(s_xcnt, s_msc);
            c1_yc <= shift_right(v_y12, s_msc);

            -- 2-3: the three hashes
            c2_a <= f_mix1(f_pack(c1_xa, c1_ya, s_sa));
            c2_b <= f_mix1(f_pack(c1_xb, c1_yb, s_sb));
            c2_c <= f_mix1(f_pack(c1_xc, c1_yc, s_sc));
            c3_a <= f_mix2(c2_a);
            c3_b <= f_mix2(c2_b);
            c3_c <= f_mix2(c2_c);

            -- 4: octave gains (small multiplies) and the pore test
            v_n5 := signed(resize(c3_a(3 downto 0), 5)) - 8;
            c4_g0 <= resize(shift_right(v_n5 * signed('0' & s_gf), 1), 9);
            v_n5 := signed(resize(c3_b(7 downto 4), 5)) - 8;
            c4_g1 <= resize(shift_right(v_n5 * signed('0' & s_gc), 1), 9);
            v_n6 := signed(resize(c3_c(4 downto 0), 6)) - 16;
            c4_g2 <= resize(shift_right(v_n6 * signed('0' & s_gc), 1), 9);
            if c3_a(15 downto 12) < s_pthr then c4_pore <= '1';
            else                                c4_pore <= '0'; end if;

            -- 5: octave sum + pore
            v_sum := resize(c4_g0, 11) + resize(c4_g1, 11) + resize(c4_g2, 11);
            if c4_pore = '1' then
                if s_spark = '1' then
                    c5_n2 <= v_sum + 144;
                else
                    c5_n2 <= v_sum - 144;
                end if;
            else
                c5_n2 <= v_sum;
            end if;
        end if;
    end process p_noise;

    ------------------------------------------------------------------------
    -- tonal-response weight, from the luma delay line (film stock puts its
    -- grain in the mid-tones; K4 fades between uniform and that)
    ------------------------------------------------------------------------
    p_tone : process(clk)
        variable v_y  : unsigned(9 downto 0);
        variable v_mn : unsigned(9 downto 0);
        variable v_d  : signed(5 downto 0);
    begin
        if rising_edge(clk) then
            v_y := unsigned(s_y_sr(2));
            v_mn := v_y;
            if v_mn > (1023 - v_y) then v_mn := 1023 - v_y; end if;
            t1_w5 <= v_mn(9 downto 5);                     -- 0..15

            v_d := signed(resize(t1_w5, 6)) - 16;          -- -16..-1
            t2_w <= unsigned(resize(
                to_signed(16, 6)
                + shift_right(v_d * signed(resize(s_tone, 5)), 4),
                5));                                       -- 1..16
        end if;
    end process p_tone;

    ------------------------------------------------------------------------
    -- THE COMPARATOR: weight the grain, bias, develop-multiply, clamp
    ------------------------------------------------------------------------
    p_core : process(clk)
        variable v24 : signed(24 downto 0);
        variable v20 : signed(20 downto 0);
        variable v12 : signed(12 downto 0);
        variable vy  : unsigned(9 downto 0);
        variable vu  : unsigned(9 downto 0);
        variable vv  : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 6: grain through the response weight
            c6_m <= resize(shift_right(c5_n2 * signed('0' & t2_w), 4), 12);

            -- 7: distance from the print midpoint
            c7_d <= (signed(resize(unsigned(s_y_sr(5)), 13)) - 512)
                    + resize(c6_m, 13) + resize(s_bias, 13);

            -- 8: the develop multiply (mantissa only; << e comes next)
            c8_p <= resize(c7_d * signed('0' & s_m), 20);

            -- 9: barrel shift, unscale, clamp
            v24 := shift_left(resize(c8_p, 25), s_e);
            v20 := resize(shift_right(v24, 4), 21);
            if    v20 >  640 then c9_r <= to_signed(640, 12);
            elsif v20 < -640 then c9_r <= to_signed(-640, 12);
            else                  c9_r <= resize(v20, 12); end if;

            -- 10: back to luma; etch / invert / mono
            v12 := to_signed(512, 13) + resize(c9_r, 13);
            vy  := f_cu10(v12);
            if s_etch = '1' then
                vy := vy(9 downto 7) & vy(9 downto 7) & vy(9 downto 7) & vy(9);
            end if;
            if s_inv = '1' then
                vy := to_unsigned(1023, 10) - vy;
            end if;
            c10_y <= vy;

            vu := unsigned(s_u_sr(8));
            vv := unsigned(s_v_sr(8));
            if s_mono = '1' then
                c10_u <= to_unsigned(512, 10);
                c10_v <= to_unsigned(512, 10);
            elsif s_inv = '1' then
                c10_u <= to_unsigned(1023, 10) - vu;
                c10_v <= to_unsigned(1023, 10) - vv;
            else
                c10_u <= vu;
                c10_v <= vv;
            end if;
        end if;
    end process p_core;

    ------------------------------------------------------------------------
    -- video delay (chroma is symmetric here, so no U/V swap needed)
    ------------------------------------------------------------------------
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_y_sr(0) <= data_in.y;
            s_u_sr(0) <= data_in.u;
            s_v_sr(0) <= data_in.v;
            for i in 1 to 8 loop
                s_y_sr(i) <= s_y_sr(i - 1);
                s_u_sr(i) <= s_u_sr(i - 1);
                s_v_sr(i) <= s_v_sr(i - 1);
            end loop;
        end if;
    end process p_delay;

    ------------------------------------------------------------------------
    -- sync delay + output
    ------------------------------------------------------------------------
    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= data_in.avid    & s_avid_sr   (0 to C_LATENCY - 2);
            s_hsync_n_sr <= data_in.hsync_n & s_hsync_n_sr(0 to C_LATENCY - 2);
            s_vsync_n_sr <= data_in.vsync_n & s_vsync_n_sr(0 to C_LATENCY - 2);
            s_field_n_sr <= data_in.field_n & s_field_n_sr(0 to C_LATENCY - 2);
        end if;
    end process p_sync;

    data_out.y       <= std_logic_vector(c10_y);
    data_out.u       <= std_logic_vector(c10_u);
    data_out.v       <= std_logic_vector(c10_v);
    data_out.avid    <= s_avid_sr(C_LATENCY - 1);
    data_out.hsync_n <= s_hsync_n_sr(C_LATENCY - 1);
    data_out.vsync_n <= s_vsync_n_sr(C_LATENCY - 1);
    data_out.field_n <= s_field_n_sr(C_LATENCY - 1);

end architecture grist;
