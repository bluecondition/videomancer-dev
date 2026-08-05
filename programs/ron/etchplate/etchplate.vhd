-- Videomancer SDK - Open source FPGA-based video effects development kit
-- Copyright (C) 2026 ron
-- File: etchplate.vhd - 1-bit shredder: white strands / streaks / tear-cuts
-- License: GNU General Public License v3.0
--
-- Etchplate v3.0 "SHRED" - full rework of the look engine.
--
-- Goal (from the original inspiration shot): stark black & white, the
-- picture ripped into shards of fine white strands on black.  Ragged
-- horizontal streaks trail off every vertical edge, vertical filaments
-- drip down from horizontal edges, and black tear-cuts shred the white
-- body itself into fibres.  Predominantly horizontal/vertical, with
-- crackling per-frame randomness.
--
-- Engine (3-cycle latency, one dual-bank line buffer, two LFSRs):
--
--   Stage d1/d2: delay video 2 cycles to meet the line buffer's 2-cycle
--     read latency.  bw = (y > threshold) xor negate, computed at d2.
--
--   Line buffer (7 bits/column): {bw_of_prev_line, vlife(5:0)}.
--     Read addr = hcnt (col of the incoming pixel); the data for column
--     x emerges exactly when the d2-delayed pixel x arrives.  Write addr
--     = hcnt delayed 3 = read addr delayed by the full pipeline depth
--     (the classic write-addr trap).  Writes park at addr 2047 (beyond
--     active width) during blanking because the buffer has no write
--     enable.  Bank toggle at hsync start; the <=3 pending tail writes
--     land before the toggle thanks to the front porch.
--
--   Horizontal streaks: on a white->black transition a down-counter is
--     loaded with (rand & length_mask) -- random length per edge, so a
--     vertical edge sheds a ragged fringe of different-length filaments
--     line by line.  A black->white transition loads a second counter
--     (half mask) that bites black into the white leading edge.
--
--   Vertical filaments: when the line above was white and this pixel is
--     black (bottom edge of white), vlife is seeded from rand & fil_mask;
--     otherwise it decays by 1 per line while over black and dies inside
--     white.  vlife > 0 draws a 1px white strand.
--
--   Tear: a run-length dash machine advances only inside white body:
--     keep-run (rand >> tear_level, so higher tear = shorter keeps) then
--     cut-run (rand & cut_mask).  Cuts render black inside white (Body
--     mode) or are the only place white shows (Bones mode = skeleton).
--
--   Grain: strand/streak pixels drop out when lfsr_b(2:0) < gap_level --
--     per-pixel breaks that make everything fibrous and crackling.
--
--   P12 Shred adds to the streak/filament/tear/grain levels on top of
--   the knobs (bold always-on: knobs work everywhere, slider max is the
--   designed climax).
--
-- Synthesis rule kept from v2.0: state-machine outer conditions never
-- read a register they write inside (yosys collapse trap).

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;

architecture etchplate of program_top is

    -- Parameters
    signal s_knob_thresh : unsigned(9 downto 0);
    signal s_knob_streak : unsigned(9 downto 0);
    signal s_knob_fil    : unsigned(9 downto 0);
    signal s_knob_tear   : unsigned(9 downto 0);
    signal s_knob_grain  : unsigned(9 downto 0);
    signal s_slider      : unsigned(9 downto 0);
    signal s_sw_negate   : std_logic;
    signal s_sw_bones    : std_logic;
    signal s_sw_bypass   : std_logic;

    -- Effect levels 0..7 (knob + slider, saturated), registered per clock
    signal s_lvl_streak : unsigned(2 downto 0) := (others => '0');
    signal s_lvl_fil    : unsigned(2 downto 0) := (others => '0');
    signal s_lvl_tear   : unsigned(2 downto 0) := (others => '0');
    signal s_gap_lvl    : unsigned(2 downto 0) := (others => '0');

    -- Level -> random-length masks, registered
    signal s_streak_mask : unsigned(9 downto 0) := (others => '0');
    signal s_fil_mask    : unsigned(5 downto 0) := (others => '0');
    signal s_cut_mask    : unsigned(3 downto 0) := (others => '0');

    -- Pixel pipeline delays
    signal s_y_d1, s_y_d2 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_u_d1, s_u_d2 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_v_d1, s_v_d2 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_avid_d1, s_avid_d2 : std_logic := '0';
    signal s_hs_d1, s_hs_d2     : std_logic := '1';
    signal s_vs_d1, s_vs_d2     : std_logic := '1';
    signal s_fn_d1, s_fn_d2     : std_logic := '0';
    signal s_bw_d2 : std_logic := '0';

    -- Column counter and its delays (write addr = read addr delayed 3)
    signal s_hcnt    : unsigned(10 downto 0) := (others => '0');
    signal s_hcnt_d1 : unsigned(10 downto 0) := (others => '0');
    signal s_hcnt_d2 : unsigned(10 downto 0) := (others => '0');

    -- Line buffer
    signal s_lb_ab   : std_logic := '0';
    signal s_hs_prev : std_logic := '1';
    signal s_lb_wr_data : std_logic_vector(6 downto 0) := (others => '0');
    signal s_lb_wr_addr : unsigned(10 downto 0) := (others => '1');
    signal s_lb_rd_data : std_logic_vector(6 downto 0);

    -- Stage-3 state
    signal s_prev_bw  : std_logic := '0';
    signal s_wht_cnt  : unsigned(9 downto 0) := (others => '0');
    signal s_blk_cnt  : unsigned(9 downto 0) := (others => '0');
    signal s_keep_cnt : unsigned(8 downto 0) := (others => '0');
    signal s_cut_cnt  : unsigned(4 downto 0) := (others => '0');

    -- LFSRs (b gets a different seed via a one-shot load)
    signal s_lfsr_a, s_lfsr_b : std_logic_vector(15 downto 0);
    signal s_boot : std_logic := '1';

    -- Output registers
    signal s_y_out, s_u_out, s_v_out : std_logic_vector(9 downto 0) := (others => '0');
    signal s_avid_out, s_hs_out, s_vs_out, s_fn_out : std_logic := '0';

    function f_sat7(a : unsigned(2 downto 0); b : unsigned(1 downto 0))
        return unsigned is
        variable v : unsigned(3 downto 0);
    begin
        v := ('0' & a) + ("00" & b);
        if v > 7 then
            v := "0111";
        end if;
        return v(2 downto 0);
    end function;

begin

    s_knob_thresh <= unsigned(registers_in(0));
    s_knob_streak <= unsigned(registers_in(1));
    s_knob_fil    <= unsigned(registers_in(2));
    s_knob_tear   <= unsigned(registers_in(3));
    s_knob_grain  <= unsigned(registers_in(5));
    s_slider      <= unsigned(registers_in(7));
    s_sw_negate   <= registers_in(6)(0);
    s_sw_bones    <= registers_in(6)(1);
    s_sw_bypass   <= registers_in(6)(2);

    --------------------------------------------------------------------------
    -- Effect levels and masks (slow -- knobs only), registered
    --------------------------------------------------------------------------
    p_levels : process(clk)
        variable v_shred : unsigned(1 downto 0);
    begin
        if rising_edge(clk) then
            v_shred := s_slider(9 downto 8);
            s_lvl_streak <= f_sat7(s_knob_streak(9 downto 7), v_shred);
            s_lvl_fil    <= f_sat7(s_knob_fil(9 downto 7), v_shred);
            s_lvl_tear   <= f_sat7(s_knob_tear(9 downto 7), v_shred);
            -- gap level 0..7 out of 8; grain knob 0..5 plus up to 2 from slider
            s_gap_lvl    <= f_sat7('0' & s_knob_grain(9 downto 8), v_shred);

            -- streak length mask, calibrated to the reference image: fine
            -- dashes a few px to a few dozen px; ~127 px only at full climax
            case to_integer(s_lvl_streak) is
                when 0      => s_streak_mask <= to_unsigned(0, 10);
                when 1      => s_streak_mask <= to_unsigned(3, 10);
                when 2      => s_streak_mask <= to_unsigned(7, 10);
                when 3      => s_streak_mask <= to_unsigned(15, 10);
                when 4      => s_streak_mask <= to_unsigned(31, 10);
                when 5      => s_streak_mask <= to_unsigned(31, 10);
                when 6      => s_streak_mask <= to_unsigned(63, 10);
                when others => s_streak_mask <= to_unsigned(127, 10);
            end case;

            -- filament length mask: short vertical drips, matching the
            -- streak scale (max 31 lines only at full climax)
            case to_integer(s_lvl_fil) is
                when 0      => s_fil_mask <= to_unsigned(0, 6);
                when 1      => s_fil_mask <= to_unsigned(1, 6);
                when 2      => s_fil_mask <= to_unsigned(3, 6);
                when 3      => s_fil_mask <= to_unsigned(3, 6);
                when 4      => s_fil_mask <= to_unsigned(7, 6);
                when 5      => s_fil_mask <= to_unsigned(15, 6);
                when others => s_fil_mask <= to_unsigned(31, 6);
            end case;

            -- cut length mask: level 0 = off, else 1..15 px cuts
            case to_integer(s_lvl_tear) is
                when 0      => s_cut_mask <= to_unsigned(0, 4);
                when 1 | 2  => s_cut_mask <= to_unsigned(1, 4);
                when 3 | 4  => s_cut_mask <= to_unsigned(3, 4);
                when 5 | 6  => s_cut_mask <= to_unsigned(7, 4);
                when others => s_cut_mask <= to_unsigned(15, 4);
            end case;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Delay pipeline, column counter, line-buffer bank toggle
    --------------------------------------------------------------------------
    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            s_y_d1 <= data_in.y;  s_y_d2 <= s_y_d1;
            s_u_d1 <= data_in.u;  s_u_d2 <= s_u_d1;
            s_v_d1 <= data_in.v;  s_v_d2 <= s_v_d1;
            s_avid_d1 <= data_in.avid;     s_avid_d2 <= s_avid_d1;
            s_hs_d1   <= data_in.hsync_n;  s_hs_d2   <= s_hs_d1;
            s_vs_d1   <= data_in.vsync_n;  s_vs_d2   <= s_vs_d1;
            s_fn_d1   <= data_in.field_n;  s_fn_d2   <= s_fn_d1;

            -- bw threshold at d2 (from d1 data)
            if unsigned(s_y_d1) > s_knob_thresh then
                s_bw_d2 <= '1' xor s_sw_negate;
            else
                s_bw_d2 <= '0' xor s_sw_negate;
            end if;

            -- column counter tracks the incoming pixel
            if data_in.hsync_n = '0' then
                s_hcnt <= (others => '0');
            elsif data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            end if;
            s_hcnt_d1 <= s_hcnt;
            s_hcnt_d2 <= s_hcnt_d1;

            -- bank toggle at hsync start (front porch covers pending writes)
            s_hs_prev <= data_in.hsync_n;
            if s_hs_prev = '1' and data_in.hsync_n = '0' then
                s_lb_ab <= not s_lb_ab;
            end if;

            s_boot <= '0';
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage 3: streak counters, tear dash machine, filament life, output
    --------------------------------------------------------------------------
    p_shred : process(clk)
        variable v_bw        : std_logic;
        variable v_bw_above  : std_logic;
        variable v_vlife     : unsigned(5 downto 0);
        variable v_vlife_new : unsigned(5 downto 0);
        variable v_rand_a    : unsigned(9 downto 0);
        variable v_rand_b    : unsigned(8 downto 0);
        variable v_gap       : std_logic;
        variable v_cutting   : std_logic;
        variable v_body      : std_logic;
        variable v_out_bit   : std_logic;
    begin
        if rising_edge(clk) then
            v_bw       := s_bw_d2;
            v_bw_above := s_lb_rd_data(6);
            v_vlife    := unsigned(s_lb_rd_data(5 downto 0));
            v_rand_a   := unsigned(s_lfsr_a(9 downto 0));
            v_rand_b   := unsigned(s_lfsr_b(8 downto 0));

            -- per-pixel break gate for strands/streaks
            if unsigned(s_lfsr_b(11 downto 9)) < s_gap_lvl then
                v_gap := '1';
            else
                v_gap := '0';
            end if;

            ------------------------------------------------------------------
            -- Horizontal streak counters (edge-fired, random length)
            ------------------------------------------------------------------
            if s_hs_d2 = '0' then
                s_wht_cnt <= (others => '0');
                s_blk_cnt <= (others => '0');
                s_prev_bw <= '0';
            elsif s_avid_d2 = '1' then
                s_prev_bw <= v_bw;
                -- white streak: fires where white ends (trails into black)
                if s_prev_bw = '1' and v_bw = '0' and s_streak_mask /= 0 then
                    s_wht_cnt <= (v_rand_a and s_streak_mask) or to_unsigned(1, 10);
                elsif s_wht_cnt /= 0 then
                    s_wht_cnt <= s_wht_cnt - 1;
                end if;
                -- black streak: bites into the leading edge of white
                if s_prev_bw = '0' and v_bw = '1' and s_streak_mask /= 0 then
                    s_blk_cnt <= (v_rand_a and ('0' & s_streak_mask(9 downto 1)))
                                 or to_unsigned(1, 10);
                elsif s_blk_cnt /= 0 then
                    s_blk_cnt <= s_blk_cnt - 1;
                end if;
            end if;

            ------------------------------------------------------------------
            -- Tear dash machine: advances only inside white body
            ------------------------------------------------------------------
            v_cutting := '0';
            if s_cut_cnt /= 0 then
                v_cutting := '1';
            end if;
            if s_avid_d2 = '1' and v_bw = '1' then
                if s_cut_cnt /= 0 then
                    s_cut_cnt <= s_cut_cnt - 1;
                elsif s_keep_cnt /= 0 then
                    s_keep_cnt <= s_keep_cnt - 1;
                elsif s_cut_mask /= 0 then
                    -- start a new cut, then a new keep run (shorter at high tear)
                    s_cut_cnt  <= ('0' & (v_rand_b(3 downto 0) and s_cut_mask))
                                  + to_unsigned(1, 5);
                    s_keep_cnt <= shift_right(v_rand_b, to_integer(s_lvl_tear))
                                  + to_unsigned(4, 9);
                else
                    s_keep_cnt <= (others => '1');
                end if;
            end if;

            ------------------------------------------------------------------
            -- Vertical filament life for this column (written to line buffer)
            ------------------------------------------------------------------
            v_vlife_new := (others => '0');
            if s_avid_d2 = '1' then
                if v_bw = '1' then
                    v_vlife_new := (others => '0');
                elsif v_bw_above = '1' and s_fil_mask /= 0 then
                    -- bottom edge of white above: seed a strand
                    v_vlife_new := (v_rand_a(5 downto 0) and s_fil_mask)
                                   or to_unsigned(1, 6);
                elsif v_vlife /= 0 then
                    v_vlife_new := v_vlife - 1;
                end if;
            end if;
            s_lb_wr_data <= v_bw & std_logic_vector(v_vlife_new);
            if s_avid_d2 = '1' then
                s_lb_wr_addr <= s_hcnt_d2;
            else
                s_lb_wr_addr <= (others => '1');  -- park beyond active width
            end if;

            ------------------------------------------------------------------
            -- Compose the output bit
            ------------------------------------------------------------------
            -- body: white region, shredded by cuts (Bones = only cuts show)
            if v_bw = '1' then
                if s_sw_bones = '1' then
                    v_body := v_cutting;
                else
                    v_body := not v_cutting;
                end if;
                -- black streak erosion of the leading edge
                if s_blk_cnt /= 0 and v_gap = '0' then
                    v_body := '0';
                end if;
            else
                v_body := '0';
            end if;

            v_out_bit := v_body;
            -- strands/streaks draw white over black, with grain breaks
            if v_bw = '0' and v_gap = '0' then
                if s_wht_cnt /= 0 then
                    v_out_bit := '1';
                end if;
                if v_vlife /= 0 then
                    v_out_bit := '1';
                end if;
            end if;

            ------------------------------------------------------------------
            -- Output mux (both paths 3-cycle latency), blanking-gated
            ------------------------------------------------------------------
            if s_sw_bypass = '1' then
                s_y_out <= s_y_d2;
                s_u_out <= s_u_d2;
                s_v_out <= s_v_d2;
            elsif s_avid_d2 = '1' then
                if v_out_bit = '1' then
                    s_y_out <= (others => '1');
                else
                    s_y_out <= (others => '0');
                end if;
                s_u_out <= std_logic_vector(to_unsigned(512, 10));
                s_v_out <= std_logic_vector(to_unsigned(512, 10));
            else
                s_y_out <= (others => '0');
                s_u_out <= std_logic_vector(to_unsigned(512, 10));
                s_v_out <= std_logic_vector(to_unsigned(512, 10));
            end if;

            s_avid_out <= s_avid_d2;
            s_hs_out   <= s_hs_d2;
            s_vs_out   <= s_vs_d2;
            s_fn_out   <= s_fn_d2;
        end if;
    end process;

    data_out.y       <= s_y_out;
    data_out.u       <= s_u_out;
    data_out.v       <= s_v_out;
    data_out.avid    <= s_avid_out;
    data_out.hsync_n <= s_hs_out;
    data_out.vsync_n <= s_vs_out;
    data_out.field_n <= s_fn_out;

    lb_inst : entity work.video_line_buffer
        generic map (
            G_WIDTH => 7,
            G_DEPTH => 11
        )
        port map (
            clk       => clk,
            i_ab      => s_lb_ab,
            i_wr_addr => s_lb_wr_addr,
            i_rd_addr => s_hcnt,
            i_data    => s_lb_wr_data,
            o_data    => s_lb_rd_data
        );

    lfsr_a_inst : entity work.lfsr16
        port map (
            clk    => clk,
            enable => '1',
            seed   => x"0000",
            load   => '0',
            q      => s_lfsr_a
        );

    lfsr_b_inst : entity work.lfsr16
        port map (
            clk    => clk,
            enable => '1',
            seed   => x"5A5A",
            load   => s_boot,
            q      => s_lfsr_b
        );

end architecture etchplate;
