-- ruttetra.vhd - Classic Rutt-Etra scan processor for Videomancer
--
-- Incoming video is re-rendered as a field of horizontal scanlines whose
-- vertical position is displaced by the picture's luma (the classic
-- Rutt/Etra "Z-displacement" look): bright material rises out of a sagging
-- dark floor, drawn as glowing lines on black, with optional hidden-line
-- terrain occlusion (each nearer line drops an opaque black skirt that
-- hides the lines behind it).
--
-- Engine
--   Every Nth active row is a "sampled" row.  As it streams in, its luma
--   (after contrast + horizontal smoothing) is converted to a displacement
--   (sag, in rows, applied DOWNWARD from the source row) and a 4-bit
--   brightness, and written column-by-column (columns decimated x2) into
--   one of 8 rotating BRAM banks (1024 x 12 each; 12-bit word packs
--   disp[7:0] & bright[3:0], mapping to 3x 1024x4 EBR per bank, 24 EBR
--   total).  A displaced line lands on rows BELOW its source row, so at
--   display time the 8 banks hold every sampled row whose line could pass
--   through the current pixel: 8 parallel candidate tests (age_k - disp_k
--   against the stroke window) find the hits, and a recency-priority
--   resolve picks the nearest (lowest source row) - which is also the
--   hidden-line rule.  Max displacement = 8 x spacing - 1 (capped 255), so
--   denser line fields have proportionally shallower relief.
--
--   Because displacement is one-sided (down), "Peaks Bright" inverts luma
--   into the sag term: bright stays near its source row, dark sags away.
--
-- Controls
--   Pot 1  (registers_in(0))    : Scan Lines - line density (CW = denser)
--   Pot 2  (registers_in(1))    : Thickness  - stroke height 1..8 rows
--   Pot 3  (registers_in(2))    : Line Color - White/Green/Amber/Cyan/
--                                 Magenta/Ice/Fire/Terrain (altitude tint)
--   Pot 4  (registers_in(3))    : Contrast   - input gain into disp+shade
--   Pot 5  (registers_in(4))    : Backdrop   - dim grayscale source ghost
--   Pot 6  (registers_in(5))    : Smoothing  - horizontal IIR on the
--                                 profile (raw nervous edges -> flowing)
--   Tog 7  (registers_in(6)(0)) : Peaks      - Bright rises / Dark rises
--   Tog 8  (registers_in(6)(1)) : Hidden Line- terrain skirts on/off
--   Tog 9  (registers_in(6)(2)) : Shading    - Banded (2-level) / Luma
--   Tog 10 (registers_in(6)(3)) : Line Style - Hard / Glow (soft halo)
--   Tog 11 (registers_in(6)(4)) : Input      - Positive / Negative
--   Fader  (registers_in(7))    : RELIEF     - displacement depth (glided)
--
-- Latency: 12 clocks, identical in every mode.  Interlace: the terrain is
-- rebuilt per field (banks invalidated at vsync, idempotent under
-- serration); ages advance on active lines only.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_timing_pkg.all;

architecture ruttetra of program_top is

    constant C_LAT : integer := 12;      -- display pipeline registers
    constant NB    : integer := 8;       -- sampled-row banks

    attribute keep : boolean;

    ----------------------------------------------------------------------
    -- Latched controls (at vsync, quilt-style)
    ----------------------------------------------------------------------
    signal s_dspace : unsigned(6 downto 0) := to_unsigned(39, 7);  -- 8..71
    signal s_dmax   : unsigned(7 downto 0) := to_unsigned(255, 8);
    signal s_thick  : unsigned(3 downto 0) := to_unsigned(2, 4);   -- 1..8
    -- thin strokes integrate less light: small shading lift at T=1..3
    signal s_bboost : unsigned(1 downto 0) := "10";
    signal s_zone   : unsigned(2 downto 0) := (others => '0');
    signal s_gain8  : unsigned(7 downto 0) := to_unsigned(128, 8);
    signal s_bg8    : unsigned(7 downto 0) := (others => '0');
    signal s_smooth : unsigned(2 downto 0) := "010";
    signal s_scale9 : unsigned(8 downto 0) := to_unsigned(8, 9);   -- glided
    signal s_peakdark : std_logic := '0';
    signal s_hlr      : std_logic := '1';
    signal s_shade    : std_logic := '1';
    signal s_glow     : std_logic := '0';
    signal s_neg      : std_logic := '0';

    ----------------------------------------------------------------------
    -- Position + row bookkeeping
    ----------------------------------------------------------------------
    signal prev_hsync_n : std_logic := '1';
    signal prev_vsync_n : std_logic := '1';
    signal pixel_x      : unsigned(11 downto 0) := (others => '0');
    signal s_row_armed  : std_logic := '1';
    signal s_saw_frame  : std_logic := '0';
    signal s_spacing    : unsigned(6 downto 0) := (others => '0');
    signal s_sampled    : std_logic := '0';
    signal s_head       : unsigned(2 downto 0) := (others => '0');

    type t_b10 is array (0 to NB - 1) of unsigned(9 downto 0);
    type t_b8  is array (0 to NB - 1) of unsigned(7 downto 0);
    type t_b4  is array (0 to NB - 1) of unsigned(3 downto 0);
    type t_b3  is array (0 to NB - 1) of unsigned(2 downto 0);
    type t_b11s is array (0 to NB - 1) of signed(10 downto 0);

    signal s_age   : t_b10 := (others => (others => '0'));
    signal s_valid : std_logic_vector(NB - 1 downto 0) := (others => '0');
    -- valid AND age /= 0 (the bank being written this row is masked: iCE40
    -- EBR read-during-write returns undefined data)
    signal s_live  : std_logic_vector(NB - 1 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Write path (profile capture on sampled rows)
    ----------------------------------------------------------------------
    signal w0_y    : unsigned(9 downto 0) := (others => '0');
    signal w1_prod : unsigned(17 downto 0) := (others => '0');
    signal w2_acc  : unsigned(13 downto 0) := (others => '0');
    signal w3_prod : unsigned(14 downto 0) := (others => '0');
    signal w3_brt  : unsigned(3 downto 0) := (others => '0');
    signal w4_data : std_logic_vector(11 downto 0) := (others => '0');
    signal w4_addr : unsigned(9 downto 0) := (others => '0');
    signal w4_we   : std_logic := '0';

    type t_wapipe is array (0 to 3) of unsigned(9 downto 0);
    signal w_addr : t_wapipe := (others => (others => '0'));
    signal w_av   : std_logic_vector(3 downto 0) := (others => '0');
    signal w_par  : std_logic_vector(3 downto 0) := (others => '0');

    ----------------------------------------------------------------------
    -- Bank read data (A2 fabric registers, after the EBR output register)
    ----------------------------------------------------------------------
    signal a2_disp : t_b8 := (others => (others => '0'));
    signal a2_brt  : t_b4 := (others => (others => '0'));

    ----------------------------------------------------------------------
    -- Display pipeline
    ----------------------------------------------------------------------
    signal a3_sub  : t_b11s := (others => (others => '0'));
    signal a3_brt  : t_b4 := (others => (others => '0'));
    signal a3_alt  : t_b3 := (others => (others => '0'));

    signal a4_hit  : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a4_halo : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a4_cov  : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a4_brt  : t_b4 := (others => (others => '0'));
    signal a4_alt  : t_b3 := (others => (others => '0'));

    signal a5_rhit : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a5_rcov : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a5_halo : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a5_brt  : t_b4 := (others => (others => '0'));
    signal a5_alt  : t_b3 := (others => (others => '0'));

    signal a6_hidx   : unsigned(2 downto 0) := (others => '0');
    signal a6_cidx   : unsigned(2 downto 0) := (others => '0');
    signal a6_anyhit : std_logic := '0';
    signal a6_anycov : std_logic := '0';
    signal a6_halo   : std_logic_vector(NB - 1 downto 0) := (others => '0');
    signal a6_brt    : t_b4 := (others => (others => '0'));
    signal a6_alt    : t_b3 := (others => (others => '0'));

    signal a7_line  : std_logic := '0';
    signal a7_skirt : std_logic := '0';
    signal a7_halo  : std_logic := '0';
    signal a7_brt   : unsigned(3 downto 0) := (others => '0');
    signal a7_alt   : unsigned(2 downto 0) := (others => '0');

    signal a8_yo    : unsigned(9 downto 0) := (others => '0');
    signal a8_uo    : signed(9 downto 0) := (others => '0');
    signal a8_vo    : signed(9 downto 0) := (others => '0');
    signal a8_bf1   : unsigned(4 downto 0) := (others => '0');
    signal a8_line  : std_logic := '0';
    signal a8_skirt : std_logic := '0';
    signal a8_bga   : unsigned(9 downto 0) := (others => '0');

    signal a9_ym    : unsigned(14 downto 0) := (others => '0');
    signal a9_um    : signed(15 downto 0) := (others => '0');
    signal a9_vm    : signed(15 downto 0) := (others => '0');
    signal a9_line  : std_logic := '0';
    signal a9_skirt : std_logic := '0';
    signal a9_bgp   : unsigned(17 downto 0) := (others => '0');

    signal a10_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal a10_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal a10_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    signal a11_y : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal a11_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal a11_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    ----------------------------------------------------------------------
    -- Sync / luma delay line (tap 10 gates blanking, tap 11 drives sync out,
    -- tap 7 feeds the backdrop lane)
    ----------------------------------------------------------------------
    type t_dly is record
        y       : std_logic_vector(9 downto 0);
        avid    : std_logic;
        hsync_n : std_logic;
        vsync_n : std_logic;
        field_n : std_logic;
    end record;
    type t_dly_arr is array (0 to C_LAT - 1) of t_dly;
    signal s_dly : t_dly_arr := (others =>
        (y => (others => '0'), avid => '0',
         hsync_n => '1', vsync_n => '1', field_n => '1'));

    ----------------------------------------------------------------------
    -- Line palettes.  10-bit YUV with U/V pre-swapped for the hardware
    -- encoder (U slot carries Cr): Y stored as offset-from-black (yo, so
    -- 64 + yo*16/16 <= 768, respecting the drawn-luma cap), chroma as
    -- signed offsets from 512.
    ----------------------------------------------------------------------
    type t_pal10 is array (0 to 7) of unsigned(9 downto 0);
    type t_pal10s is array (0 to 7) of signed(9 downto 0);
    -- zones 0..6: White, Green, Amber, Cyan, Magenta, Ice, Fire (7=Terrain)
    constant C_ZY : t_pal10 := (to_unsigned(704, 10), to_unsigned(587, 10),
        to_unsigned(647, 10), to_unsigned(614, 10), to_unsigned(516, 10),
        to_unsigned(565, 10), to_unsigned(400, 10), to_unsigned(704, 10));
    constant C_ZU : t_pal10s := (to_signed(0, 10), to_signed(-300, 10),
        to_signed(166, 10), to_signed(-448, 10), to_signed(335, 10),
        to_signed(-156, 10), to_signed(347, 10), to_signed(0, 10));
    constant C_ZV : t_pal10s := (to_signed(0, 10), to_signed(-237, 10),
        to_signed(-373, 10), to_signed(151, 10), to_signed(208, 10),
        to_signed(179, 10), to_signed(-181, 10), to_signed(0, 10));
    -- Terrain altitude tint, indexed by disp(7:5): 0 = least sag (peak,
    -- white-hot) .. 7 = deepest sag (valley, dim blue)
    constant C_TY : t_pal10 := (to_unsigned(704, 10), to_unsigned(585, 10),
        to_unsigned(682, 10), to_unsigned(604, 10), to_unsigned(468, 10),
        to_unsigned(414, 10), to_unsigned(271, 10), to_unsigned(111, 10));
    constant C_TU : t_pal10s := (to_signed(30, 10), to_signed(212, 10),
        to_signed(77, 10), to_signed(-121, 10), to_signed(-277, 10),
        to_signed(-238, 10), to_signed(-133, 10), to_signed(-81, 10));
    constant C_TV : t_pal10s := (to_signed(-52, 10), to_signed(-262, 10),
        to_signed(-343, 10), to_signed(-297, 10), to_signed(-118, 10),
        to_signed(165, 10), to_signed(299, 10), to_signed(239, 10));

begin

    ----------------------------------------------------------------------
    -- Position counters, per-row bank bookkeeping, vsync parameter latch
    ----------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_samp    : std_logic;
        variable v_newhead : unsigned(2 downto 0);
        variable v_inv     : unsigned(9 downto 0);
        variable v_d7      : unsigned(6 downto 0);
        variable v_dm      : unsigned(7 downto 0);
        variable v_targ    : unsigned(9 downto 0);
        variable v_diff    : signed(10 downto 0);
        variable v_next    : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            if v_h_edge = '1' then
                pixel_x     <= (others => '0');
                s_row_armed <= '1';
            elsif data_in.avid = '1' then
                pixel_x <= pixel_x + 1;
            end if;

            if data_in.avid = '1' then s_saw_frame <= '1'; end if;

            -- Row-start bookkeeping: fires on the FIRST active pixel of each
            -- row.  Everything decided here is first consumed >= 3 pipeline
            -- stages later, so the one-cycle registration lag is safe.
            if data_in.avid = '1' and s_row_armed = '1' then
                s_row_armed <= '0';
                if s_spacing = 0 then
                    v_samp := '1';
                    s_spacing <= s_dspace - 1;
                else
                    v_samp := '0';
                    s_spacing <= s_spacing - 1;
                end if;
                s_sampled <= v_samp;
                if v_samp = '1' then
                    v_newhead := s_head + 1;
                else
                    v_newhead := s_head;
                end if;
                s_head <= v_newhead;
                for k in 0 to NB - 1 loop
                    if v_samp = '1' and k = to_integer(v_newhead) then
                        s_age(k)   <= (others => '0');
                        s_valid(k) <= '1';
                        s_live(k)  <= '0';
                    elsif s_valid(k) = '1' then
                        if s_age(k) /= 1023 then
                            s_age(k) <= s_age(k) + 1;
                        end if;
                        s_live(k) <= '1';
                    end if;
                end loop;
            end if;

            -- Field/frame start.  Analog vsync serrates (several edges per
            -- field): every action here is idempotent, and the relief glide
            -- is guarded to one step per field by s_saw_frame.
            if v_v_edge = '1' then
                s_valid   <= (others => '0');
                s_live    <= (others => '0');
                s_spacing <= (others => '0');

                -- K1 Scan Lines: CW = denser.  Spacing 8..71 rows.
                v_inv := 1023 - unsigned(registers_in(0));
                v_d7  := to_unsigned(8, 7) + v_inv(9 downto 4);
                s_dspace <= v_d7;
                -- Max displacement: a bank lives 8 sampled rows, so a line
                -- stays retrievable while disp < 8*spacing (storage caps 255).
                if v_d7 >= 32 then
                    s_dmax <= to_unsigned(255, 8);
                else
                    v_dm := shift_left(resize(v_d7, 8), 3) - 1;
                    s_dmax <= v_dm;
                end if;

                s_thick  <= to_unsigned(1, 4) + resize(unsigned(registers_in(1)(9 downto 7)), 4);
                case registers_in(1)(9 downto 7) is
                    when "000"  => s_bboost <= "11";   -- T=1: +3
                    when "001"  => s_bboost <= "10";   -- T=2: +2
                    when "010"  => s_bboost <= "01";   -- T=3: +1
                    when others => s_bboost <= "00";
                end case;
                s_zone   <= unsigned(registers_in(2)(9 downto 7));
                s_gain8  <= unsigned(registers_in(3)(9 downto 2));
                s_bg8    <= unsigned(registers_in(4)(9 downto 2));
                s_smooth <= unsigned(registers_in(5)(9 downto 7));

                s_peakdark <= registers_in(6)(0);
                s_hlr      <= registers_in(6)(1);
                s_shade    <= registers_in(6)(2);
                s_glow     <= registers_in(6)(3);
                s_neg      <= registers_in(6)(4);

                -- P12 Relief -> displacement scale, glided (ease by diff/4
                -- with a +/-1 deadband) so pot ADC noise cannot make the
                -- whole line field breathe.
                if s_saw_frame = '1' then
                    s_saw_frame <= '0';
                    v_targ := to_unsigned(8, 10) + resize(unsigned(registers_in(7)(9 downto 2)), 10);
                    v_diff := signed(resize(v_targ, 11)) - signed(resize(s_scale9, 11));
                    v_next := signed(resize(s_scale9, 11));
                    if v_diff > 3 then
                        v_next := v_next + shift_right(v_diff, 2);
                    elsif v_diff > 1 then
                        v_next := v_next + 1;
                    elsif v_diff < -3 then
                        v_next := v_next + shift_right(v_diff, 2);
                    elsif v_diff < -1 then
                        v_next := v_next - 1;
                    end if;
                    s_scale9 <= unsigned(v_next(8 downto 0));
                end if;
            end if;
        end if;
    end process p_position;

    ----------------------------------------------------------------------
    -- Write path: luma -> (negative) -> contrast gain -> smoothing IIR ->
    -- displacement + brightness, written into bank[head] on sampled rows
    -- (even columns only; the store is column-decimated x2).
    ----------------------------------------------------------------------
    p_write : process(clk)
        variable v_arg  : unsigned(9 downto 0);
        variable v_l11  : unsigned(10 downto 0);
        variable v_lum  : unsigned(9 downto 0);
        variable v_diff : signed(14 downto 0);
        variable v_step : signed(14 downto 0);
        variable v_nacc : signed(14 downto 0);
        variable v_d6   : unsigned(5 downto 0);
        variable v_dr   : unsigned(9 downto 0);
        variable v_disp : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- W0: input register + S11 negative
            if s_neg = '1' then
                w0_y <= to_unsigned(1023, 10) - unsigned(data_in.y);
            else
                w0_y <= unsigned(data_in.y);
            end if;
            w_addr(0) <= pixel_x(10 downto 1);
            w_av(0)   <= data_in.avid;
            w_par(0)  <= pixel_x(0);

            -- W1: contrast gain (unity at knob centre: x gain8/128, 0..2x)
            if w0_y > 64 then
                v_arg := w0_y - 64;
            else
                v_arg := (others => '0');
            end if;
            w1_prod   <= v_arg * s_gain8;
            w_addr(1) <= w_addr(0);
            w_av(1)   <= w_av(0);
            w_par(1)  <= w_par(0);

            -- W2: clamp + horizontal smoothing IIR (14-bit accumulator with
            -- 4 fractional bits; reset to the first sample of each row)
            v_l11 := w1_prod(17 downto 7);
            if v_l11 > 1023 then
                v_lum := to_unsigned(1023, 10);
            else
                v_lum := v_l11(9 downto 0);
            end if;
            if w_av(1) = '1' then
                if w_av(2) = '0' then
                    w2_acc <= v_lum & "0000";
                else
                    v_diff := signed(resize(v_lum & "0000", 15)) - signed(resize(w2_acc, 15));
                    case to_integer(s_smooth) is
                        when 0      => v_step := v_diff;
                        when 1      => v_step := shift_right(v_diff, 1);
                        when 2      => v_step := shift_right(v_diff, 2);
                        when 3      => v_step := shift_right(v_diff, 3);
                        when 4      => v_step := shift_right(v_diff, 4);
                        when 5      => v_step := shift_right(v_diff, 5);
                        when 6      => v_step := shift_right(v_diff, 6);
                        when others => v_step := shift_right(v_diff, 7);
                    end case;
                    v_nacc := signed(resize(w2_acc, 15)) + v_step;
                    w2_acc <= unsigned(v_nacc(13 downto 0));
                end if;
            end if;
            w_addr(2) <= w_addr(1);
            w_av(2)   <= w_av(1);
            w_par(2)  <= w_par(1);

            -- W3: sag term x relief scale (6x9 multiply, own stage)
            if s_peakdark = '1' then
                v_d6 := w2_acc(13 downto 8);
            else
                v_d6 := 63 - w2_acc(13 downto 8);
            end if;
            w3_prod   <= v_d6 * s_scale9;
            w3_brt    <= w2_acc(13 downto 10);
            w_addr(3) <= w_addr(2);
            w_av(3)   <= w_av(2);
            w_par(3)  <= w_par(2);

            -- W4: displacement clamp (min 1 so a line never lands on its own
            -- write row) + pack.  Write on even columns of sampled rows.
            v_dr := w3_prod(14 downto 5);
            if v_dr > resize(s_dmax, 10) then
                v_disp := s_dmax;
            elsif v_dr = 0 then
                v_disp := to_unsigned(1, 8);
            else
                v_disp := v_dr(7 downto 0);
            end if;
            w4_data <= std_logic_vector(v_disp) & std_logic_vector(w3_brt);
            w4_addr <= w_addr(3);
            if w_av(3) = '1' and s_sampled = '1' and w_par(3) = '0' then
                w4_we <= '1';
            else
                w4_we <= '0';
            end if;
        end if;
    end process p_write;

    ----------------------------------------------------------------------
    -- 8 rotating profile banks: canonical 1W1R, 1024 x 12 each.
    -- Per-bank registered read address (kept) so the placer can sit each
    -- copy beside its own EBR group instead of routing one high-fanout net.
    ----------------------------------------------------------------------
    gen_banks : for k in 0 to NB - 1 generate
        type t_ram is array (0 to 1023) of std_logic_vector(11 downto 0);
        signal ram    : t_ram := (others => (others => '0'));
        signal rdaddr : unsigned(9 downto 0) := (others => '0');
        signal rdata  : std_logic_vector(11 downto 0) := (others => '0');
        attribute keep of rdaddr : signal is true;
    begin
        p_bank : process(clk)
        begin
            if rising_edge(clk) then
                rdaddr <= pixel_x(10 downto 1);
                if w4_we = '1' and to_integer(s_head) = k then
                    ram(to_integer(w4_addr)) <= w4_data;
                end if;
                rdata <= ram(to_integer(rdaddr));
                -- second (fabric) register after the EBR-absorbed one
                a2_disp(k) <= unsigned(rdata(11 downto 4));
                a2_brt(k)  <= unsigned(rdata(3 downto 0));
            end if;
        end process p_bank;
    end generate gen_banks;

    ----------------------------------------------------------------------
    -- Display pipeline: candidate tests -> recency-priority resolve ->
    -- palette + shading -> compose -> blanking gate
    ----------------------------------------------------------------------
    p_disp : process(clk)
        variable v_t    : signed(10 downto 0);
        variable v_2t   : signed(10 downto 0);
        variable v_nt   : signed(10 downto 0);
        variable v_core : boolean;
        variable v_halo : boolean;
        variable v_cov  : boolean;
        variable v_idx  : integer range 0 to 7;
        variable v_hidx : integer range 0 to 7;
        variable v_cidx : integer range 0 to 7;
        variable v_anyh : std_logic;
        variable v_anyc : std_logic;
        variable v_line : std_logic;
        variable v_bank : integer range 0 to 7;
        variable v_bf   : unsigned(3 downto 0);
        variable v_bf5  : unsigned(4 downto 0);
        variable v_ba   : unsigned(9 downto 0);
        variable v_yi   : unsigned(10 downto 0);
        variable v_ui   : integer range -2048 to 2047;
        variable v_vi   : integer range -2048 to 2047;
    begin
        if rising_edge(clk) then
            -- A3: candidate offset = age - disp (one op after the BRAM regs)
            for k in 0 to NB - 1 loop
                a3_sub(k) <= signed(resize(s_age(k), 11)) - signed(resize(a2_disp(k), 11));
                a3_brt(k) <= a2_brt(k);
                a3_alt(k) <= a2_disp(k)(7 downto 5);
            end loop;

            -- A4: classify each candidate against the stroke window
            v_t  := signed(resize(s_thick, 11));
            v_2t := signed(resize(shift_left(resize(s_thick, 5), 1), 11));
            v_nt := -v_t;
            for k in 0 to NB - 1 loop
                v_cov  := s_live(k) = '1' and a3_sub(k) >= 0;
                v_core := v_cov and a3_sub(k) < v_t;
                v_halo := s_glow = '1' and s_live(k) = '1' and
                          ((a3_sub(k) >= v_nt and a3_sub(k) < 0) or
                           (a3_sub(k) >= v_t and a3_sub(k) < v_2t));
                if v_core or v_halo then a4_hit(k) <= '1'; else a4_hit(k) <= '0'; end if;
                if v_halo and not v_core then a4_halo(k) <= '1'; else a4_halo(k) <= '0'; end if;
                if v_cov then a4_cov(k) <= '1'; else a4_cov(k) <= '0'; end if;
                a4_brt(k) <= a3_brt(k);
                a4_alt(k) <= a3_alt(k);
            end loop;

            -- A5: rotate hit/cover vectors so index 0 = most recent bank
            -- (lowest source row = nearest = hidden-line priority)
            for i in 0 to NB - 1 loop
                v_idx := to_integer(s_head - to_unsigned(i, 3));
                a5_rhit(i) <= a4_hit(v_idx);
                a5_rcov(i) <= a4_cov(v_idx);
            end loop;
            a5_halo <= a4_halo;
            a5_brt  <= a4_brt;
            a5_alt  <= a4_alt;

            -- A6: priority encode (lowest rotated index wins)
            v_hidx := 7; v_anyh := '0';
            v_cidx := 7; v_anyc := '0';
            for i in NB - 1 downto 0 loop
                if a5_rhit(i) = '1' then v_hidx := i; v_anyh := '1'; end if;
                if a5_rcov(i) = '1' then v_cidx := i; v_anyc := '1'; end if;
            end loop;
            a6_hidx   <= to_unsigned(v_hidx, 3);
            a6_cidx   <= to_unsigned(v_cidx, 3);
            a6_anyhit <= v_anyh;
            a6_anycov <= v_anyc;
            a6_halo   <= a5_halo;
            a6_brt    <= a5_brt;
            a6_alt    <= a5_alt;

            -- A7: draw decision + winner payload mux.  A hit draws unless a
            -- strictly nearer line's skirt covers this pixel (hidden-line
            -- mode only); otherwise the skirt paints black.
            if a6_anyhit = '1' and (s_hlr = '0' or a6_anycov = '0' or a6_hidx <= a6_cidx) then
                v_line := '1';
            else
                v_line := '0';
            end if;
            a7_line <= v_line;
            if s_hlr = '1' and a6_anycov = '1' and v_line = '0' then
                a7_skirt <= '1';
            else
                a7_skirt <= '0';
            end if;
            v_bank := to_integer(s_head - a6_hidx);
            a7_brt  <= a6_brt(v_bank);
            a7_alt  <= a6_alt(v_bank);
            a7_halo <= a6_halo(v_bank);

            -- A8: palette + shading factor
            if s_zone = 7 then
                a8_yo <= C_TY(to_integer(a7_alt));
                a8_uo <= C_TU(to_integer(a7_alt));
                a8_vo <= C_TV(to_integer(a7_alt));
            else
                a8_yo <= C_ZY(to_integer(s_zone));
                a8_uo <= C_ZU(to_integer(s_zone));
                a8_vo <= C_ZV(to_integer(s_zone));
            end if;
            if s_shade = '1' then
                v_bf := a7_brt;
            elsif a7_brt >= 8 then
                v_bf := to_unsigned(15, 4);
            else
                v_bf := to_unsigned(4, 4);
            end if;
            if a7_halo = '1' then
                v_bf := "00" & v_bf(3 downto 2);
            end if;
            -- +1 (full scale at 16) plus the thin-stroke lift, held at the
            -- 768 drawn-luma cap (bf1 = 16)
            v_bf5 := resize(v_bf, 5) + 1 + resize(s_bboost, 5);
            if v_bf5 > 16 then
                v_bf5 := to_unsigned(16, 5);
            end if;
            a8_bf1   <= v_bf5;
            a8_line  <= a7_line;
            a8_skirt <= a7_skirt;

            -- A8 (backdrop lane): dim grayscale ghost of the source
            v_ba := unsigned(s_dly(7).y);
            if v_ba > 64 then
                a8_bga <= v_ba - 64;
            else
                a8_bga <= (others => '0');
            end if;

            -- A9: shading multiplies (each gets the stage to itself)
            a9_ym    <= a8_yo * a8_bf1;
            a9_um    <= a8_uo * signed(resize(a8_bf1, 6));
            a9_vm    <= a8_vo * signed(resize(a8_bf1, 6));
            a9_bgp   <= a8_bga * s_bg8;
            a9_line  <= a8_line;
            a9_skirt <= a8_skirt;

            -- A10: compose (line > skirt > backdrop).  Line luma tops out at
            -- 64 + 704 = 768 by construction (drawn-luma cap); chroma at
            -- 512 +/- 448.  Backdrop tops out at 64 + 477.
            if a9_line = '1' then
                v_yi := to_unsigned(64, 11) + resize(a9_ym(14 downto 4), 11);
                a10_y <= v_yi(9 downto 0);
                v_ui := 512 + to_integer(shift_right(a9_um, 4));
                v_vi := 512 + to_integer(shift_right(a9_vm, 4));
                a10_u <= to_unsigned(v_ui, 10);
                a10_v <= to_unsigned(v_vi, 10);
            elsif a9_skirt = '1' then
                a10_y <= to_unsigned(64, 10);
                a10_u <= to_unsigned(512, 10);
                a10_v <= to_unsigned(512, 10);
            else
                a10_y <= to_unsigned(64, 10) + resize(a9_bgp(17 downto 9), 10);
                a10_u <= to_unsigned(512, 10);
                a10_v <= to_unsigned(512, 10);
            end if;

            -- A11: blanking gate (one stage before the output register is
            -- this register: outputs are neutral whenever aligned avid = 0)
            if s_dly(10).avid = '1' then
                a11_y <= a10_y;
                a11_u <= a10_u;
                a11_v <= a10_v;
            else
                a11_y <= to_unsigned(64, 10);
                a11_u <= to_unsigned(512, 10);
                a11_v <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_disp;

    ----------------------------------------------------------------------
    -- Sync / luma delay line
    ----------------------------------------------------------------------
    p_dly : process(clk)
    begin
        if rising_edge(clk) then
            s_dly(0).y       <= data_in.y;
            s_dly(0).avid    <= data_in.avid;
            s_dly(0).hsync_n <= data_in.hsync_n;
            s_dly(0).vsync_n <= data_in.vsync_n;
            s_dly(0).field_n <= data_in.field_n;
            for i in 1 to C_LAT - 1 loop
                s_dly(i) <= s_dly(i - 1);
            end loop;
        end if;
    end process p_dly;

    ----------------------------------------------------------------------
    -- Outputs
    ----------------------------------------------------------------------
    data_out.y <= std_logic_vector(a11_y);
    data_out.u <= std_logic_vector(a11_u);
    data_out.v <= std_logic_vector(a11_v);

    data_out.avid    <= s_dly(C_LAT - 1).avid;
    data_out.hsync_n <= s_dly(C_LAT - 1).hsync_n;
    data_out.vsync_n <= s_dly(C_LAT - 1).vsync_n;
    data_out.field_n <= s_dly(C_LAT - 1).field_n;

end ruttetra;
