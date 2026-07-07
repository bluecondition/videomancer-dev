-- Sidewinder: per-scanline horizontal wave displacement.
--
-- Incoming video is written into a dual-bank line buffer and read back one
-- line later at a horizontally shifted address. The shift is a triangle (or
-- sine) wave of the line number, so the picture snakes left and right as it
-- goes down the screen; the wave phase advances every frame so the bends
-- roll vertically.
--
--   offset(line) = wave(frame_phase + y*freq) * Amount
--                + wave(3*(frame_phase + y*freq)) * Ripple      (3x harmonic)
--   out(x, y)    = in(x - offset, y - 1)
--
-- Controls:
--   K1 Amount  displacement amplitude, 0..+-255 px
--   K2 Period  vertical wavelength; quadratic knob so the low end sweeps
--              slowly through long whole-screen bends (freq = K2^2 >> 8)
--   K3 Rate    bidirectional with a centre detent: centre = frozen, right =
--              wave rolls down, left = rolls up. Quadratic away from centre
--              (rate = sign * (K3-512)^2 >> 6) so near-centre gives ultra
--              slow drift and the extremes get properly violent.
--   K4 Ripple  3x-frequency harmonic amount, 0..+-63 px
--   S7 Shape   Triangle / Sine (sine table is -cos so shapes stay aligned)
--   S8 Swell   slow automatic triangle LFO on the amplitude (breathing)
--   S9 Steps   quantize the offset to 16 px -> staircase band shred
--   S10 Edge   Black (off-screen reads go black) / Wrap (screen is a tube)
--   S11 Bypass, P12 Mix (dry/wet)
--
-- All wave math runs once per line in a small step-sequenced pipeline during
-- horizontal blanking (and once per frame during vertical blanking), so every
-- multiply has registered inputs and outputs -- nothing per-pixel except the
-- buffer read/write and two run-along read counters (see ziffern memory:
-- combinational mults feeding latched signals are still STA-timed).
--
-- Wrap mode never computes (p + offset) mod width per pixel: the wrapped read
-- pointer is initialised once per line during blanking and then just runs
-- along with pixel_x, wrapping at width-1 with a single compare.
--
-- Buffer = inlined dual-bank ping-pong BRAM (from shearline, minus the
-- feedback): each line WRITES its own parity bank, READS the other (the line
-- above). Y full 2048 deep; U/V 2:1 decimated (1024). ~22 EBRs.
--
-- Pipeline (rising-edge clocks, data_in -> data_out):
--   E1  p_ctrl: register inputs, pixel_x / q / ptr counters, parity,
--       line + frame step sequencers (combinational read addr mux)
--   E2  bank reads registered; write of live pixel lands; capture bg/rbank
--   E3  p_wet: bank select / background fill -> s_wet_*
--   E4-E7 interpolator_u (4-clock) dry/wet mix
--   E7  data_out (combinational bypass mux)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture sidewinder of program_top is

    --------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------
    constant C_DW            : integer := C_VIDEO_DATA_WIDTH;  -- 10
    constant C_Y_AW          : integer := 11;                  -- 2048 (full)
    constant C_UV_AW         : integer := 10;                  -- 1024 (2:1 dec)
    constant C_TOTAL_LATENCY : integer := 7;

    -- Swell LFO speed: added to a 16-bit phase every frame (~6 s per cycle).
    constant C_SWELL_RATE    : integer := 192;

    --------------------------------------------------------------------------
    -- Sine ROM: 256 x signed, value = round(-255*cos(2*pi*i/256)) so index 0
    -- is the wave minimum, matching the triangle fold below.
    --------------------------------------------------------------------------
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

    -- Triangle fold of a 16-bit phase to -256..255 (min at phase 0).
    function f_tri(phase : unsigned(15 downto 0)) return signed is
        variable v_ramp : signed(9 downto 0);
    begin
        v_ramp := signed(resize(phase(14 downto 6), 10));  -- 0..511
        if phase(15) = '0' then
            return v_ramp - to_signed(256, 10);            -- -256 .. 255
        else
            return to_signed(255, 10) - v_ramp;            --  255 .. -256
        end if;
    end function f_tri;

    --------------------------------------------------------------------------
    -- Inlined dual-bank line buffers
    --------------------------------------------------------------------------
    type t_lb_y  is array (0 to 2047) of std_logic_vector(C_DW - 1 downto 0);
    type t_lb_uv is array (0 to 1023) of std_logic_vector(C_DW - 1 downto 0);

    signal lbY0, lbY1 : t_lb_y  := (others => (others => '0'));
    signal lbU0, lbU1 : t_lb_uv := (others => (others => '0'));
    signal lbV0, lbV1 : t_lb_uv := (others => (others => '0'));

    signal s_rdY0, s_rdY1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');
    signal s_rdV0, s_rdV1 : std_logic_vector(C_DW - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Input registers / position tracking (E1)
    --------------------------------------------------------------------------
    signal s_in_y, s_in_u, s_in_v : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_in_avid              : std_logic := '0';

    signal s_pixel_x      : unsigned(C_Y_AW - 1 downto 0) := (others => '0');
    signal s_par          : std_logic := '0';
    signal s_prev_hsync_n : std_logic := '1';
    signal s_prev_vsync_n : std_logic := '1';

    signal s_line_width   : unsigned(C_Y_AW - 1 downto 0) := (others => '0');

    -- Run-along read counters (init during hblank, +1 per active pixel)
    signal s_q   : signed(12 downto 0)          := (others => '0');  -- unwrapped
    signal s_ptr : unsigned(C_Y_AW - 1 downto 0) := (others => '0'); -- wrapped

    --------------------------------------------------------------------------
    -- Per-frame step sequencer (vertical blanking)
    --------------------------------------------------------------------------
    signal s_vstep       : unsigned(2 downto 0) := "111";
    signal s_d           : signed(10 downto 0) := (others => '0');
    signal s_absd        : unsigned(9 downto 0) := (others => '0');
    signal s_dsq         : unsigned(19 downto 0) := (others => '0');
    signal s_k2sq        : unsigned(19 downto 0) := (others => '0');
    signal s_swell_phase : unsigned(15 downto 0) := (others => '0');
    signal s_swell_tri   : unsigned(9 downto 0) := (others => '0');
    signal s_swell_prod  : unsigned(19 downto 0) := (others => '0');
    signal s_rate        : signed(13 downto 0) := (others => '0');
    signal s_freq        : unsigned(11 downto 0) := (others => '0');
    signal s_eff_amp     : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_frame_phase : unsigned(15 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Per-line step sequencer (horizontal blanking)
    --------------------------------------------------------------------------
    signal s_lstep      : unsigned(2 downto 0) := "111";
    signal s_line_phase : unsigned(15 downto 0) := (others => '0');
    signal s_phase3     : unsigned(15 downto 0) := (others => '0');
    signal s_sin_main   : signed(9 downto 0) := (others => '0');
    signal s_tri_main   : signed(9 downto 0) := (others => '0');
    signal s_sin_rip    : signed(9 downto 0) := (others => '0');
    signal s_tri_rip    : signed(9 downto 0) := (others => '0');
    signal s_wave_main  : signed(9 downto 0) := (others => '0');
    signal s_wave_rip   : signed(9 downto 0) := (others => '0');
    signal s_prod_main  : signed(20 downto 0) := (others => '0');
    signal s_prod_rip   : signed(20 downto 0) := (others => '0');
    signal s_offset     : signed(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Control (decoded from registers)
    --------------------------------------------------------------------------
    signal s_amount   : unsigned(C_DW - 1 downto 0);
    signal s_period_k : unsigned(C_DW - 1 downto 0);
    signal s_rate_k   : unsigned(C_DW - 1 downto 0);
    signal s_ripple_k : unsigned(C_DW - 1 downto 0);
    signal s_shape    : std_logic;
    signal s_swell    : std_logic;
    signal s_steps    : std_logic;
    signal s_wrap     : std_logic;
    signal s_bypass   : std_logic;
    signal s_mix_t    : unsigned(C_DW - 1 downto 0);

    --------------------------------------------------------------------------
    -- Combinational read address
    --------------------------------------------------------------------------
    signal s_rd_addr_y  : unsigned(C_Y_AW - 1 downto 0) := (others => '0');
    signal s_rd_addr_uv : unsigned(C_UV_AW - 1 downto 0) := (others => '0');
    signal s_rd_bg      : std_logic := '0';

    --------------------------------------------------------------------------
    -- Stage E2 capture
    --------------------------------------------------------------------------
    signal s_a_bg    : std_logic := '0';
    signal s_a_rbank : std_logic := '0';

    --------------------------------------------------------------------------
    -- Wet result (E3)
    --------------------------------------------------------------------------
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(C_DW - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Dry / sync delay shift registers
    --------------------------------------------------------------------------
    type t_data_shift is array (0 to C_TOTAL_LATENCY - 1)
        of std_logic_vector(C_DW - 1 downto 0);
    type t_bit_shift  is array (0 to C_TOTAL_LATENCY - 1) of std_logic;

    signal s_y_sr     : t_data_shift := (others => (others => '0'));
    signal s_u_sr     : t_data_shift := (others => (others => '0'));
    signal s_v_sr     : t_data_shift := (others => (others => '0'));
    signal s_hsync_sr : t_bit_shift  := (others => '1');
    signal s_vsync_sr : t_bit_shift  := (others => '1');
    signal s_field_sr : t_bit_shift  := (others => '1');
    signal s_avid_sr  : t_bit_shift  := (others => '0');

    --------------------------------------------------------------------------
    -- Interpolator I/O
    --------------------------------------------------------------------------
    signal s_interp_y_a, s_interp_u_a, s_interp_v_a : unsigned(C_DW - 1 downto 0);
    signal s_interp_y_result, s_interp_u_result, s_interp_v_result : unsigned(C_DW - 1 downto 0);
    signal s_interp_y_valid, s_interp_u_valid, s_interp_v_valid : std_logic;

begin

    --------------------------------------------------------------------------
    -- Control decode
    --------------------------------------------------------------------------
    s_amount   <= unsigned(registers_in(0));
    s_period_k <= unsigned(registers_in(1));
    s_rate_k   <= unsigned(registers_in(2));
    s_ripple_k <= unsigned(registers_in(3));
    s_shape    <= registers_in(6)(0);
    s_swell    <= registers_in(6)(1);
    s_steps    <= registers_in(6)(2);
    s_wrap     <= registers_in(6)(3);
    s_bypass   <= registers_in(6)(4);
    s_mix_t    <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- Combinational read address: wrap mode uses the pre-wrapped run-along
    -- pointer; black mode uses the unwrapped counter with a range check.
    --------------------------------------------------------------------------
    p_rdaddr : process(s_wrap, s_ptr, s_q, s_line_width)
        variable v_rd : unsigned(C_Y_AW - 1 downto 0);
    begin
        if s_wrap = '1' then
            s_rd_bg <= '0';
            v_rd    := s_ptr;
        elsif s_q < 0 or s_q >= signed(resize(s_line_width, 13)) then
            s_rd_bg <= '1';
            v_rd    := (others => '0');
        else
            s_rd_bg <= '0';
            v_rd    := unsigned(s_q(C_Y_AW - 1 downto 0));
        end if;
        s_rd_addr_y  <= v_rd;
        s_rd_addr_uv <= v_rd(C_Y_AW - 1 downto 1);  -- >> 1 decimate
    end process p_rdaddr;

    --------------------------------------------------------------------------
    -- E1: input registers, counters, parity, and both step sequencers.
    -- Everything phase/offset related lives in this one process (single
    -- driver for s_line_phase, s_q, s_ptr).
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_amp   : signed(10 downto 0);
        variable v_rip   : signed(10 downto 0);
        variable v_off   : signed(9 downto 0);
        variable v_mag   : unsigned(13 downto 0);
        variable v_fp    : unsigned(15 downto 0);
        variable v_abs   : signed(10 downto 0);
        variable v_off11 : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y    <= unsigned(data_in.y);
            s_in_u    <= unsigned(data_in.u);
            s_in_v    <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;

            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;

            -- Per-pixel: write column counter and both read counters run
            -- along together, so read stays (write - offset) all line.
            if data_in.avid = '1' then
                s_pixel_x <= s_pixel_x + 1;
                s_q       <= s_q + 1;
                if s_ptr = s_line_width - 1 then
                    s_ptr <= (others => '0');
                else
                    s_ptr <= s_ptr + 1;
                end if;
            end if;

            ------------------------------------------------------------------
            -- Per-line sequencer: one registered step per clock during
            -- horizontal blanking; done long before active video starts.
            ------------------------------------------------------------------
            case to_integer(s_lstep) is
                when 0 =>       -- 3x harmonic phase
                    s_phase3 <= shift_left(s_line_phase, 1) + s_line_phase;
                    s_lstep  <= s_lstep + 1;
                when 1 =>       -- main wave lookup (both shapes)
                    s_sin_main <= to_signed(C_SINE(to_integer(s_line_phase(15 downto 8))), 10);
                    s_tri_main <= f_tri(s_line_phase);
                    s_lstep    <= s_lstep + 1;
                when 2 =>       -- ripple wave lookup; select main shape
                    s_sin_rip <= to_signed(C_SINE(to_integer(s_phase3(15 downto 8))), 10);
                    s_tri_rip <= f_tri(s_phase3);
                    if s_shape = '1' then
                        s_wave_main <= s_sin_main;
                    else
                        s_wave_main <= s_tri_main;
                    end if;
                    s_lstep <= s_lstep + 1;
                when 3 =>       -- select ripple shape; main multiply
                    if s_shape = '1' then
                        s_wave_rip <= s_sin_rip;
                    else
                        s_wave_rip <= s_tri_rip;
                    end if;
                    v_amp       := signed('0' & s_eff_amp);
                    s_prod_main <= s_wave_main * v_amp;
                    s_lstep     <= s_lstep + 1;
                when 4 =>       -- ripple multiply
                    v_rip      := signed('0' & s_ripple_k);
                    s_prod_rip <= s_wave_rip * v_rip;
                    s_lstep    <= s_lstep + 1;
                when 5 =>       -- combine, optional 16 px quantize
                    v_off := resize(shift_right(s_prod_main, 10), 10)
                           + resize(shift_right(s_prod_rip, 12), 10);
                    if s_steps = '1' then
                        v_off(3 downto 0) := (others => '0');
                    end if;
                    s_offset <= v_off;
                    s_lstep  <= s_lstep + 1;
                when 6 =>       -- initialise the run-along read counters
                    s_q     <= resize(s_offset, 13);
                    v_off11 := resize(s_offset, 11);
                    if s_line_width > to_unsigned(320, C_Y_AW) then
                        -- |offset| <= 318 < width: one mod-2048 correction
                        -- (unsigned(v_off11) is already offset mod 2048)
                        if s_offset < 0 then
                            s_ptr <= unsigned(v_off11) + s_line_width;
                        else
                            s_ptr <= unsigned(v_off11);
                        end if;
                    else
                        s_ptr <= (others => '0');
                    end if;
                    s_lstep <= s_lstep + 1;
                when others =>  -- idle
                    null;
            end case;

            ------------------------------------------------------------------
            -- Per-frame sequencer (vertical blanking): rate curve, period
            -- curve, swell LFO -> s_rate / s_freq / s_eff_amp, then advance
            -- the frame phase.
            ------------------------------------------------------------------
            case to_integer(s_vstep) is
                when 0 =>
                    s_d           <= signed('0' & s_rate_k) - to_signed(512, 11);
                    s_swell_phase <= s_swell_phase + to_unsigned(C_SWELL_RATE, 16);
                    s_vstep       <= s_vstep + 1;
                when 1 =>
                    v_abs  := abs(s_d);
                    s_absd <= unsigned(v_abs(9 downto 0));
                    if s_swell_phase(15) = '0' then
                        s_swell_tri <= s_swell_phase(14 downto 5);
                    else
                        s_swell_tri <= to_unsigned(1023, 10) - s_swell_phase(14 downto 5);
                    end if;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_dsq        <= s_absd * s_absd;
                    s_k2sq       <= s_period_k * s_period_k;
                    s_swell_prod <= s_amount * s_swell_tri;
                    s_vstep      <= s_vstep + 1;
                when 3 =>
                    v_mag := s_dsq(19 downto 6);
                    if s_absd < to_unsigned(8, 10) then     -- centre detent
                        s_rate <= (others => '0');
                    elsif s_d(10) = '1' then
                        s_rate <= -signed(v_mag);
                    else
                        s_rate <= signed(v_mag);
                    end if;
                    s_freq <= s_k2sq(19 downto 8);
                    if s_swell = '1' then
                        s_eff_amp <= s_swell_prod(19 downto 10);
                    else
                        s_eff_amp <= s_amount;
                    end if;
                    s_vstep <= s_vstep + 1;
                when 4 =>
                    v_fp := unsigned(signed(s_frame_phase) + resize(s_rate, 16));
                    s_frame_phase <= v_fp;
                    s_line_phase  <= v_fp;
                    s_vstep       <= s_vstep + 1;
                when others =>
                    null;
            end case;

            -- hsync falling edge = new line: latch width, reset counters,
            -- flip parity, advance the wave phase, kick the line sequencer.
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_pixel_x /= 0 then
                    s_line_width <= s_pixel_x;
                end if;
                s_pixel_x    <= (others => '0');
                s_par        <= not s_par;
                s_line_phase <= s_line_phase + resize(s_freq, 16);
                s_lstep      <= (others => '0');
            end if;

            -- vsync falling edge = new frame: re-sync parity, kick the frame
            -- sequencer (placed last so its line_phase load wins over the
            -- hsync add on a coincident edge).
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_par   <= '0';
                s_vstep <= (others => '0');
            end if;

            -- Dry / sync shift registers
            s_y_sr(0)     <= data_in.y;
            s_u_sr(0)     <= data_in.u;
            s_v_sr(0)     <= data_in.v;
            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_TOTAL_LATENCY - 1 loop
                s_y_sr(i)     <= s_y_sr(i - 1);
                s_u_sr(i)     <= s_u_sr(i - 1);
                s_v_sr(i)     <= s_v_sr(i - 1);
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- E2: capture read-side context aligned with the bank read data.
    -- Read uses the live (already flipped) parity -> the line above.
    --------------------------------------------------------------------------
    p_stage_a : process(clk)
    begin
        if rising_edge(clk) then
            s_a_bg    <= s_rd_bg;
            s_a_rbank <= not s_par;
        end if;
    end process p_stage_a;

    --------------------------------------------------------------------------
    -- E3: bank select + background fill -> wet result.
    --------------------------------------------------------------------------
    p_wet : process(clk)
    begin
        if rising_edge(clk) then
            if s_a_bg = '1' then
                s_wet_y <= (others => '0');
                s_wet_u <= to_unsigned(512, C_DW);
                s_wet_v <= to_unsigned(512, C_DW);
            elsif s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1);
                s_wet_u <= unsigned(s_rdU1);
                s_wet_v <= unsigned(s_rdV1);
            else
                s_wet_y <= unsigned(s_rdY0);
                s_wet_u <= unsigned(s_rdU0);
                s_wet_v <= unsigned(s_rdV0);
            end if;
        end if;
    end process p_wet;

    --------------------------------------------------------------------------
    -- Bank arrays: one process per bank (single driver, single write port).
    -- Write = live E1-registered pixel into this line's parity bank; read =
    -- combinational shifted address, registered out (valid E2).
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_rd_addr_y));
            if s_in_avid = '1' and s_par = '0' then
                lbY0(to_integer(s_pixel_x)) <= std_logic_vector(s_in_y);
            end if;
        end if;
    end process p_lbY0;

    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_rd_addr_y));
            if s_in_avid = '1' and s_par = '1' then
                lbY1(to_integer(s_pixel_x)) <= std_logic_vector(s_in_y);
            end if;
        end if;
    end process p_lbY1;

    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_rd_addr_uv));
            if s_in_avid = '1' and s_par = '0' then
                lbU0(to_integer(s_pixel_x(C_Y_AW - 1 downto 1))) <= std_logic_vector(s_in_u);
            end if;
        end if;
    end process p_lbU0;

    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_rd_addr_uv));
            if s_in_avid = '1' and s_par = '1' then
                lbU1(to_integer(s_pixel_x(C_Y_AW - 1 downto 1))) <= std_logic_vector(s_in_u);
            end if;
        end if;
    end process p_lbU1;

    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_rd_addr_uv));
            if s_in_avid = '1' and s_par = '0' then
                lbV0(to_integer(s_pixel_x(C_Y_AW - 1 downto 1))) <= std_logic_vector(s_in_v);
            end if;
        end if;
    end process p_lbV0;

    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_rd_addr_uv));
            if s_in_avid = '1' and s_par = '1' then
                lbV1(to_integer(s_pixel_x(C_Y_AW - 1 downto 1))) <= std_logic_vector(s_in_v);
            end if;
        end if;
    end process p_lbV1;

    --------------------------------------------------------------------------
    -- Dry/wet mix: interpolator_u, 4-clock latency.
    -- s_wet_* is valid 3 clocks after data_in -> dry tap is index 2.
    --------------------------------------------------------------------------
    s_interp_y_a <= unsigned(s_y_sr(2));
    s_interp_u_a <= unsigned(s_u_sr(2));
    s_interp_v_a <= unsigned(s_v_sr(2));

    interp_y_inst : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DW,
            G_FRAC_BITS  => C_DW,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => s_interp_y_a,
            b      => s_wet_y,
            t      => s_mix_t,
            result => s_interp_y_result,
            valid  => s_interp_y_valid
        );

    interp_u_inst : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DW,
            G_FRAC_BITS  => C_DW,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => s_interp_u_a,
            b      => s_wet_u,
            t      => s_mix_t,
            result => s_interp_u_result,
            valid  => s_interp_u_valid
        );

    interp_v_inst : entity work.interpolator_u
        generic map(
            G_WIDTH      => C_DW,
            G_FRAC_BITS  => C_DW,
            G_OUTPUT_MIN => 0,
            G_OUTPUT_MAX => 1023
        )
        port map(
            clk    => clk,
            enable => '1',
            a      => s_interp_v_a,
            b      => s_wet_v,
            t      => s_mix_t,
            result => s_interp_v_result,
            valid  => s_interp_v_valid
        );

    --------------------------------------------------------------------------
    -- Output mux at E7. Sync from the shift register; video from the
    -- interpolator unless Bypass is on.
    --------------------------------------------------------------------------
    data_out.hsync_n <= s_hsync_sr(C_TOTAL_LATENCY - 1);
    data_out.vsync_n <= s_vsync_sr(C_TOTAL_LATENCY - 1);
    data_out.field_n <= s_field_sr(C_TOTAL_LATENCY - 1);
    data_out.avid    <= s_avid_sr(C_TOTAL_LATENCY - 1);

    data_out.y <= s_y_sr(C_TOTAL_LATENCY - 1) when s_bypass = '1'
                  else std_logic_vector(s_interp_y_result);
    data_out.u <= s_u_sr(C_TOTAL_LATENCY - 1) when s_bypass = '1'
                  else std_logic_vector(s_interp_u_result);
    data_out.v <= s_v_sr(C_TOTAL_LATENCY - 1) when s_bypass = '1'
                  else std_logic_vector(s_interp_v_result);

end architecture sidewinder;
