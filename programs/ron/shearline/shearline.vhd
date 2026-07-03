-- Shearline: recursive line-feedback smear.
--
-- The idea: buffer a whole scanline, then on the NEXT line re-draw the
-- *previous output* line shifted horizontally by a few pixels, blending a
-- little live video back in each line. Because every line re-draws the
-- already-shifted previous line (recursive feedback, not a one-shot skew), a
-- bright source pixel streaks into a diagonal comet trail sliding down-screen.
--
-- Per active pixel (column p on the current line):
--   prev   = feedback_buffer[ p - shift ]      (previous line, slid sideways)
--   result = ( decay*prev + (1023-decay)*live ) >> 10
--   feedback_buffer[ p ] = result             (stored for the NEXT line)
--   out    = mix( live, result, Mix )
--
-- Decay = trail length. Using (1023-decay) as the live-inject weight keeps the
-- feedback loop's energy bounded (no runaway to white). This is exactly the
-- persist blend in slipframe, but fed its own output instead of raw input, and
-- read at a horizontally shifted address.
--
-- Buffer = inlined dual-bank (ping-pong) BRAM arrays, one pair per channel.
--   * Each scanline READS the bank the previous line wrote and WRITES the other.
--   * The bank a pixel writes into is its line parity, captured at read-issue
--     time and carried down the pipeline to the write stage. Because the result
--     lands a couple of cycles after the read, the trailing writes at a line's
--     end carry the OLD parity and so finish into the correct (old) bank with no
--     special boundary handling -- this sidesteps the classic write-addr /
--     bank-flip race (see project memory: lb_write_addr_timing).
--   * Reads use the live (already-flipped at hsync) parity, so a new line
--     immediately reads what the line above just wrote.
--
-- Chroma economy: Y is full 2048-deep; U and V are stored 2:1 horizontally
-- decimated (1024 deep). Chroma is low-frequency so this is near-invisible and
-- keeps total BRAM ~22 of 32 EBRs on the iCE40 HX4K.
--
-- Pipeline (rising-edge clocks, data_in -> data_out):
--   E0  data_in combinational
--   E1  p_input:  register inputs, pixel_x / parity / clear FSM, dry+sync SR
--       (combinational s_rd_addr_* drives the bank read ports)
--   E2  p_stage_a / bank reads: s_rd*0/1 valid; capture live/par/addr/we/bg
--   E3  p_stage_b: select prev bank, multiply (decay*prev, inj*live), carry
--   E4  s_res_* combinational sum>>10 -> bank WRITE + register s_wet_*
--   E5-E8 interpolator_u (4-clock) dry/wet mix
--   E8  data_out (combinational bypass mux)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture shearline of program_top is

    --------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------
    constant C_DW            : integer := C_VIDEO_DATA_WIDTH;  -- 10
    constant C_Y_AW          : integer := 11;                  -- 2048 (full)
    constant C_UV_AW         : integer := 10;                  -- 1024 (2:1 dec)
    constant C_TOTAL_LATENCY : integer := 9;
    constant C_MAX_SHIFT     : integer := 31;

    --------------------------------------------------------------------------
    -- Inlined dual-bank line buffers (feedback store)
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
    -- Input registers / position tracking
    --------------------------------------------------------------------------
    signal s_in_y, s_in_u, s_in_v : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_in_avid              : std_logic := '0';

    signal s_pixel_x      : unsigned(C_Y_AW - 1 downto 0) := (others => '0');
    signal s_par          : std_logic := '0';
    signal s_prev_hsync_n : std_logic := '1';
    signal s_prev_vsync_n : std_logic := '1';

    -- LFSR + per-line latched random for Slope Mod jitter
    signal s_lfsr_q       : std_logic_vector(15 downto 0);
    signal s_line_rand    : unsigned(4 downto 0) := (others => '0');
    signal s_eff_shift    : unsigned(4 downto 0) := (others => '0');

    -- Clear sweep (Frame=Reset): wipe both banks during vblank
    signal s_clearing     : std_logic := '0';
    signal s_clr_addr     : unsigned(C_Y_AW - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Control (decoded from registers)
    --------------------------------------------------------------------------
    signal s_decay        : unsigned(C_DW - 1 downto 0);
    signal s_base_shift   : unsigned(4 downto 0);
    signal s_slope        : unsigned(C_DW - 1 downto 0);
    signal s_dir          : std_logic;
    signal s_reset_mode   : std_logic;
    signal s_bypass       : std_logic;
    signal s_mix_t        : unsigned(C_DW - 1 downto 0);

    --------------------------------------------------------------------------
    -- Combinational read-address generation
    --------------------------------------------------------------------------
    signal s_rd_addr_y  : unsigned(C_Y_AW - 1 downto 0) := (others => '0');
    signal s_rd_addr_uv : unsigned(C_UV_AW - 1 downto 0) := (others => '0');
    signal s_rd_bg      : std_logic := '0';

    --------------------------------------------------------------------------
    -- Stage A: captured per-pixel context
    --------------------------------------------------------------------------
    signal s_a_live_y, s_a_live_u, s_a_live_v : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_a_bg      : std_logic := '0';
    signal s_a_par     : std_logic := '0';
    signal s_a_rbank   : std_logic := '0';
    signal s_a_addr_y  : unsigned(C_Y_AW - 1 downto 0)  := (others => '0');
    signal s_a_addr_uv : unsigned(C_UV_AW - 1 downto 0) := (others => '0');
    signal s_a_we      : std_logic := '0';

    --------------------------------------------------------------------------
    -- Stage A2 "prep": prev bank selected, diff = (prev - live), blend amount
    -- t (= decay, or 0 at a background read). Registered so the multiply in the
    -- next stage sees clean register inputs (no mux in the multiply path).
    --------------------------------------------------------------------------
    signal s_pp_diffY, s_pp_diffU, s_pp_diffV : signed(C_DW downto 0) := (others => '0');
    signal s_pp_liveY, s_pp_liveU, s_pp_liveV : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_pp_t      : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_pp_par    : std_logic := '0';
    signal s_pp_addr_y : unsigned(C_Y_AW - 1 downto 0)  := (others => '0');
    signal s_pp_addr_uv: unsigned(C_UV_AW - 1 downto 0) := (others => '0');
    signal s_pp_we     : std_logic := '0';

    --------------------------------------------------------------------------
    -- Stage B "mult": one signed multiply per channel, t * (prev - live).
    -- result = live + (t*(prev-live)) >> 10  (a lerp: t=0 -> live, t=1023 -> prev)
    --------------------------------------------------------------------------
    signal s_b_prodY, s_b_prodU, s_b_prodV : signed(C_DW + 11 downto 0) := (others => '0');
    signal s_b_liveY, s_b_liveU, s_b_liveV : unsigned(C_DW - 1 downto 0) := (others => '0');
    signal s_b_par     : std_logic := '0';
    signal s_b_addr_y  : unsigned(C_Y_AW - 1 downto 0)  := (others => '0');
    signal s_b_addr_uv : unsigned(C_UV_AW - 1 downto 0) := (others => '0');
    signal s_b_we      : std_logic := '0';

    --------------------------------------------------------------------------
    -- Stage C: combinational blend result (write-back + wet path)
    --------------------------------------------------------------------------
    signal s_resY, s_resU, s_resV : std_logic_vector(C_DW - 1 downto 0);

    signal s_wet_y, s_wet_u, s_wet_v : unsigned(C_DW - 1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Dry / sync delay shift registers (length = C_TOTAL_LATENCY)
    -- index 0 = 1 clock delayed, index N = (N+1) clocks delayed
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
    s_decay      <= unsigned(registers_in(1));
    s_base_shift <= unsigned('0' & registers_in(0)(9 downto 6));  -- 0..15 px
    s_slope      <= unsigned(registers_in(2));
    s_dir        <= registers_in(6)(0);
    s_reset_mode <= registers_in(6)(1);
    s_bypass     <= registers_in(6)(4);
    s_mix_t      <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- Combinational read-address gen: column (p -/+ eff_shift), with edge
    -- clamp -> background flag when the shifted address leaves the line.
    --------------------------------------------------------------------------
    p_rdaddr : process(s_pixel_x, s_eff_shift, s_dir)
        variable v_p  : signed(C_Y_AW + 1 downto 0);
        variable v_sh : signed(C_Y_AW + 1 downto 0);
        variable v_rd : signed(C_Y_AW + 1 downto 0);
    begin
        v_p  := signed(resize(s_pixel_x, C_Y_AW + 2));
        v_sh := signed(resize(s_eff_shift, C_Y_AW + 2));
        if s_dir = '0' then
            v_rd := v_p - v_sh;   -- Right slide: read from the left
        else
            v_rd := v_p + v_sh;   -- Left slide: read from the right
        end if;

        if v_rd < 0 or v_rd > 2047 then
            s_rd_bg      <= '1';
            s_rd_addr_y  <= (others => '0');
            s_rd_addr_uv <= (others => '0');
        else
            s_rd_bg      <= '0';
            s_rd_addr_y  <= unsigned(v_rd(C_Y_AW - 1 downto 0));
            s_rd_addr_uv <= unsigned(v_rd(C_UV_AW downto 1));  -- >> 1 decimate
        end if;
    end process p_rdaddr;

    --------------------------------------------------------------------------
    -- Stage E1: input register, position/parity tracking, Slope Mod jitter,
    -- clear FSM, dry+sync shift registers.
    --------------------------------------------------------------------------
    p_input : process(clk)
        variable v_jit_prod : unsigned(14 downto 0);
        variable v_jitter   : unsigned(4 downto 0);
        variable v_eff      : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            s_in_y    <= unsigned(data_in.y);
            s_in_u    <= unsigned(data_in.u);
            s_in_v    <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;

            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;

            if data_in.avid = '1' then
                s_pixel_x <= s_pixel_x + 1;
            end if;

            -- hsync falling edge = new line: reset x, flip parity, latch the
            -- per-line random and recompute the effective shift for the line.
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_pixel_x  <= (others => '0');
                s_par      <= not s_par;
                s_line_rand <= unsigned(s_lfsr_q(4 downto 0));

                -- Slope Mod: per-line jitter added to the base shift. slope(10b)
                -- x rand(5b) >> 10 gives 0..~31; gate by the slope knob.
                v_jit_prod := s_slope * s_line_rand;
                v_jitter   := v_jit_prod(14 downto 10);
                v_eff      := resize(s_base_shift, 6) + resize(v_jitter, 6);
                if v_eff > to_unsigned(C_MAX_SHIFT, 6) then
                    s_eff_shift <= to_unsigned(C_MAX_SHIFT, 5);
                else
                    s_eff_shift <= v_eff(4 downto 0);
                end if;
            end if;

            -- vsync falling edge = new frame: re-sync parity; kick clear sweep
            -- when Frame=Reset so trails don't bleed across frames.
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_par <= '0';
                if s_reset_mode = '1' then
                    s_clearing <= '1';
                    s_clr_addr <= (others => '0');
                end if;
            end if;

            -- Clear sweep advance
            if s_clearing = '1' then
                if s_clr_addr = to_unsigned(2047, C_Y_AW) then
                    s_clearing <= '0';
                else
                    s_clr_addr <= s_clr_addr + 1;
                end if;
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
    end process p_input;

    --------------------------------------------------------------------------
    -- Stage E2: capture per-pixel context aligned with the bank read data.
    -- Read uses the LIVE parity (already flipped at hsync) -> reads the line
    -- above; the write parity is this pixel's own parity, carried to the write.
    --------------------------------------------------------------------------
    p_stage_a : process(clk)
    begin
        if rising_edge(clk) then
            s_a_live_y  <= s_in_y;
            s_a_live_u  <= s_in_u;
            s_a_live_v  <= s_in_v;
            s_a_bg      <= s_rd_bg;
            s_a_par     <= s_par;
            s_a_rbank   <= not s_par;
            s_a_addr_y  <= s_pixel_x;
            s_a_addr_uv <= s_pixel_x(C_Y_AW - 1 downto 1);
            s_a_we      <= s_in_avid;
        end if;
    end process p_stage_a;

    --------------------------------------------------------------------------
    -- Stage E3: select previous bank, then the recursive blend multiplies.
    -- At a background (off-line) read, force prev=0 and full live weight so
    -- edge columns pass live video instead of darkening.
    --------------------------------------------------------------------------
    p_prep : process(clk)
        variable v_prevY, v_prevU, v_prevV : unsigned(C_DW - 1 downto 0);
    begin
        if rising_edge(clk) then
            if s_a_rbank = '1' then
                v_prevY := unsigned(s_rdY1);
                v_prevU := unsigned(s_rdU1);
                v_prevV := unsigned(s_rdV1);
            else
                v_prevY := unsigned(s_rdY0);
                v_prevU := unsigned(s_rdU0);
                v_prevV := unsigned(s_rdV0);
            end if;

            -- diff = prev - live  (signed, -1023..1023)
            s_pp_diffY <= signed(resize(v_prevY, C_DW + 1)) - signed(resize(s_a_live_y, C_DW + 1));
            s_pp_diffU <= signed(resize(v_prevU, C_DW + 1)) - signed(resize(s_a_live_u, C_DW + 1));
            s_pp_diffV <= signed(resize(v_prevV, C_DW + 1)) - signed(resize(s_a_live_v, C_DW + 1));

            s_pp_liveY <= s_a_live_y;
            s_pp_liveU <= s_a_live_u;
            s_pp_liveV <= s_a_live_v;

            -- blend amount: decay, forced to 0 on an off-line read so edge
            -- columns pass live video instead of feeding garbage back.
            if s_a_bg = '1' then
                s_pp_t <= (others => '0');
            else
                s_pp_t <= s_decay;
            end if;

            s_pp_par     <= s_a_par;
            s_pp_addr_y  <= s_a_addr_y;
            s_pp_addr_uv <= s_a_addr_uv;
            s_pp_we      <= s_a_we;
        end if;
    end process p_prep;

    --------------------------------------------------------------------------
    -- Stage B "mult": isolated signed multiply t*(prev-live), one per channel.
    -- Register inputs (from p_prep) and outputs keep it off any other path.
    --------------------------------------------------------------------------
    p_mult : process(clk)
        variable v_t : signed(C_DW downto 0);
    begin
        if rising_edge(clk) then
            v_t := signed(resize(s_pp_t, C_DW + 1));   -- 0..1023, positive
            s_b_prodY <= v_t * s_pp_diffY;             -- 11 x 11 -> 22-bit signed
            s_b_prodU <= v_t * s_pp_diffU;
            s_b_prodV <= v_t * s_pp_diffV;

            s_b_liveY <= s_pp_liveY;
            s_b_liveU <= s_pp_liveU;
            s_b_liveV <= s_pp_liveV;

            s_b_par     <= s_pp_par;
            s_b_addr_y  <= s_pp_addr_y;
            s_b_addr_uv <= s_pp_addr_uv;
            s_b_we      <= s_pp_we;
        end if;
    end process p_mult;

    --------------------------------------------------------------------------
    -- Stage C (combinational): result = clamp( live + (t*(prev-live)) >> 10 ).
    -- Slicing bits (21..10) of the signed product is an arithmetic >> 10.
    --------------------------------------------------------------------------
    p_result : process(s_b_prodY, s_b_prodU, s_b_prodV,
                       s_b_liveY, s_b_liveU, s_b_liveV)
        variable v_sumY, v_sumU, v_sumV : signed(C_DW + 2 downto 0);
    begin
        v_sumY := signed(resize(s_b_liveY, C_DW + 3)) + resize(s_b_prodY(C_DW + 11 downto 10), C_DW + 3);
        v_sumU := signed(resize(s_b_liveU, C_DW + 3)) + resize(s_b_prodU(C_DW + 11 downto 10), C_DW + 3);
        v_sumV := signed(resize(s_b_liveV, C_DW + 3)) + resize(s_b_prodV(C_DW + 11 downto 10), C_DW + 3);

        if v_sumY < 0 then s_resY <= (others => '0');
        elsif v_sumY > to_signed(1023, C_DW + 3) then s_resY <= std_logic_vector(to_unsigned(1023, C_DW));
        else s_resY <= std_logic_vector(v_sumY(C_DW - 1 downto 0)); end if;

        if v_sumU < 0 then s_resU <= (others => '0');
        elsif v_sumU > to_signed(1023, C_DW + 3) then s_resU <= std_logic_vector(to_unsigned(1023, C_DW));
        else s_resU <= std_logic_vector(v_sumU(C_DW - 1 downto 0)); end if;

        if v_sumV < 0 then s_resV <= (others => '0');
        elsif v_sumV > to_signed(1023, C_DW + 3) then s_resV <= std_logic_vector(to_unsigned(1023, C_DW));
        else s_resV <= std_logic_vector(v_sumV(C_DW - 1 downto 0)); end if;
    end process p_result;

    -- Register the wet result for the dry/wet mixer.
    p_wet : process(clk)
    begin
        if rising_edge(clk) then
            s_wet_y <= unsigned(s_resY);
            s_wet_u <= unsigned(s_resU);
            s_wet_v <= unsigned(s_resV);
        end if;
    end process p_wet;

    --------------------------------------------------------------------------
    -- Bank arrays: one process per bank (single driver). Each registers its
    -- read at the combinational read address and writes either the clear sweep
    -- or the carried blend result for the pixels whose parity matches the bank.
    --------------------------------------------------------------------------
    -- NOTE: each bank has exactly ONE write port. The clear sweep and the
    -- blend write-back are MUXED into a single (addr,data,we) so the array maps
    -- to a single-write-port EBR. Splitting them into two addressed writes makes
    -- yosys see two write ports and fall back to flip-flops (huge LC blow-up).
    p_lbY0 : process(clk)
        variable v_we   : std_logic;
        variable v_addr : unsigned(C_Y_AW - 1 downto 0);
        variable v_data : std_logic_vector(C_DW - 1 downto 0);
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_rd_addr_y));
            if s_clearing = '1' then
                v_we := '1'; v_addr := s_clr_addr; v_data := (others => '0');
            elsif s_b_we = '1' and s_b_par = '0' then
                v_we := '1'; v_addr := s_b_addr_y; v_data := s_resY;
            else
                v_we := '0'; v_addr := s_b_addr_y; v_data := s_resY;
            end if;
            if v_we = '1' then
                lbY0(to_integer(v_addr)) <= v_data;
            end if;
        end if;
    end process p_lbY0;

    p_lbY1 : process(clk)
        variable v_we   : std_logic;
        variable v_addr : unsigned(C_Y_AW - 1 downto 0);
        variable v_data : std_logic_vector(C_DW - 1 downto 0);
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_rd_addr_y));
            if s_clearing = '1' then
                v_we := '1'; v_addr := s_clr_addr; v_data := (others => '0');
            elsif s_b_we = '1' and s_b_par = '1' then
                v_we := '1'; v_addr := s_b_addr_y; v_data := s_resY;
            else
                v_we := '0'; v_addr := s_b_addr_y; v_data := s_resY;
            end if;
            if v_we = '1' then
                lbY1(to_integer(v_addr)) <= v_data;
            end if;
        end if;
    end process p_lbY1;

    p_lbU0 : process(clk)
        variable v_we   : std_logic;
        variable v_addr : unsigned(C_UV_AW - 1 downto 0);
        variable v_data : std_logic_vector(C_DW - 1 downto 0);
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_rd_addr_uv));
            if s_clearing = '1' and s_clr_addr < to_unsigned(1024, C_Y_AW) then
                v_we := '1'; v_addr := s_clr_addr(C_UV_AW - 1 downto 0); v_data := (others => '0');
            elsif s_b_we = '1' and s_b_par = '0' then
                v_we := '1'; v_addr := s_b_addr_uv; v_data := s_resU;
            else
                v_we := '0'; v_addr := s_b_addr_uv; v_data := s_resU;
            end if;
            if v_we = '1' then
                lbU0(to_integer(v_addr)) <= v_data;
            end if;
        end if;
    end process p_lbU0;

    p_lbU1 : process(clk)
        variable v_we   : std_logic;
        variable v_addr : unsigned(C_UV_AW - 1 downto 0);
        variable v_data : std_logic_vector(C_DW - 1 downto 0);
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_rd_addr_uv));
            if s_clearing = '1' and s_clr_addr < to_unsigned(1024, C_Y_AW) then
                v_we := '1'; v_addr := s_clr_addr(C_UV_AW - 1 downto 0); v_data := (others => '0');
            elsif s_b_we = '1' and s_b_par = '1' then
                v_we := '1'; v_addr := s_b_addr_uv; v_data := s_resU;
            else
                v_we := '0'; v_addr := s_b_addr_uv; v_data := s_resU;
            end if;
            if v_we = '1' then
                lbU1(to_integer(v_addr)) <= v_data;
            end if;
        end if;
    end process p_lbU1;

    p_lbV0 : process(clk)
        variable v_we   : std_logic;
        variable v_addr : unsigned(C_UV_AW - 1 downto 0);
        variable v_data : std_logic_vector(C_DW - 1 downto 0);
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_rd_addr_uv));
            if s_clearing = '1' and s_clr_addr < to_unsigned(1024, C_Y_AW) then
                v_we := '1'; v_addr := s_clr_addr(C_UV_AW - 1 downto 0); v_data := (others => '0');
            elsif s_b_we = '1' and s_b_par = '0' then
                v_we := '1'; v_addr := s_b_addr_uv; v_data := s_resV;
            else
                v_we := '0'; v_addr := s_b_addr_uv; v_data := s_resV;
            end if;
            if v_we = '1' then
                lbV0(to_integer(v_addr)) <= v_data;
            end if;
        end if;
    end process p_lbV0;

    p_lbV1 : process(clk)
        variable v_we   : std_logic;
        variable v_addr : unsigned(C_UV_AW - 1 downto 0);
        variable v_data : std_logic_vector(C_DW - 1 downto 0);
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_rd_addr_uv));
            if s_clearing = '1' and s_clr_addr < to_unsigned(1024, C_Y_AW) then
                v_we := '1'; v_addr := s_clr_addr(C_UV_AW - 1 downto 0); v_data := (others => '0');
            elsif s_b_we = '1' and s_b_par = '1' then
                v_we := '1'; v_addr := s_b_addr_uv; v_data := s_resV;
            else
                v_we := '0'; v_addr := s_b_addr_uv; v_data := s_resV;
            end if;
            if v_we = '1' then
                lbV1(to_integer(v_addr)) <= v_data;
            end if;
        end if;
    end process p_lbV1;

    --------------------------------------------------------------------------
    -- Dry/wet mix: interpolator_u, 4-clock latency per channel.
    -- a = dry (input delayed to match wet, 4 clocks), b = wet, t = Mix.
    -- s_wet_* is valid 5 clocks after data_in -> dry tap is index 4.
    --------------------------------------------------------------------------
    s_interp_y_a <= unsigned(s_y_sr(4));
    s_interp_u_a <= unsigned(s_u_sr(4));
    s_interp_v_a <= unsigned(s_v_sr(4));

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
    -- LFSR noise source (free-running) for Slope Mod
    --------------------------------------------------------------------------
    lfsr_inst : entity work.lfsr16
        port map(
            clk    => clk,
            enable => '1',
            seed   => x"0000",
            load   => '0',
            q      => s_lfsr_q
        );

    --------------------------------------------------------------------------
    -- Output mux at E8. Sync from the bypass shift register; video from the
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

end architecture shearline;
