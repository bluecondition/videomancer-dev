-- Videomancer SDK - Open source FPGA-based video effects development kit
-- Copyright (C) 2026 ron
-- File: etchplate.vhd - 1-bit shredder: hatch patches of single thin lines
-- License: GNU General Public License v3.0
--
-- Etchplate v3.1 "HATCH" - render rule rebuilt around the reference image.
--
-- The reference is made of exactly three things and nothing else:
--   1. Solid white regions (thresholded subject)
--   2. Solid black background
--   3. GROUPS OF PARALLEL SINGLE-PIXEL LINES -- coherent hatched patches
--      where a region decomposes into thin white lines with clean black
--      gaps (a line every 2nd..5th scanline), patches tens of px wide and
--      a handful..30 lines tall, anchored around edges; plus occasional
--      solo 1px vertical strands.
-- There is NO per-pixel noise / static anywhere.  All randomness is at
-- the patch level: where a patch starts and how big it is.  Inside a
-- patch the pattern is perfectly regular.
--
-- Engine (3-cycle latency, one dual-bank line buffer, two LFSRs):
--
--   Fringe zones ("the patches"): at a white->black edge an OUT run of
--   random length starts (fringe over black); at a black->white edge an
--   IN run (fringe eating into the white body).  When a run fires it
--   also latches a random patch HEIGHT; the line buffer carries a
--   per-column life so the patch extends downward that many lines.
--   Inside any fringe zone the output is simply line_hit -- white on
--   every Nth scanline, black between.  That single rule produces the
--   horizontal thin-line groups on both sides of every edge.
--
--   Vertical hatch: where the line above was white and this pixel is
--   black (bottom edge of white), a per-column life is seeded; while it
--   lives the output over plain black is col_hit -- white on every Nth
--   column = groups of thin vertical lines under horizontal edges.
--
--   Filaments: solo solid 1px vertical strands seeded at bottom edges,
--   living longer than the hatch patches.
--
--   Pitch (K6): the line/column spacing inside all groups, 2..5 px.
--   P12 Shred adds to streak/tear/height levels (always-on knobs,
--   slider max = designed climax).  S8 Bones renders the entire white
--   body as thin lines.  No noise gates anywhere.
--
-- Line buffer, 25 bits per column:
--   [24]    bw of previous line
--   [23:18] filament life      (solid vertical strand, lines remaining)
--   [17:12] vertical hatch life
--   [11:6]  out-fringe life    (patch over black)
--   [5:0]   in-fringe life     (patch over white)
--
-- v3.2: threshold hysteresis (+/-24) + edge qualification (a transition
-- only fires when the run it ends was >= 6 px) -- real video is full of
-- threshold-noise micro-edges; without these the fringe zones chain into
-- full-width hatch bars.  Knob tables made strictly monotonic and wider
-- (patch width to 255 px, height to 63 lines) so K2-K4 read clearly.
-- Read addr = hcnt (column of incoming pixel); data for column x arrives
-- exactly with the 2-cycle-delayed pixel x; write addr = hcnt delayed by
-- the full 3-cycle pipeline depth (the classic write-addr trap).  Writes
-- park at addr 2047 during blanking (buffer has no write enable); bank
-- toggles at hsync start, front porch covers the pending tail writes.
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
    signal s_knob_height : unsigned(9 downto 0);
    signal s_knob_tear   : unsigned(9 downto 0);
    signal s_knob_pitch  : unsigned(9 downto 0);
    signal s_slider      : unsigned(9 downto 0);
    signal s_sw_negate   : std_logic;
    signal s_sw_bones    : std_logic;
    signal s_sw_bypass   : std_logic;
    signal s_sw_rawthr   : std_logic;  -- S10: bypass hysteresis (diagnostic)
    signal s_sw_loose    : std_logic;  -- S11: bypass edge qualification

    -- Hysteresis bands, registered (11-bit to avoid wrap at the rails)
    signal s_thr_hi : unsigned(10 downto 0) := (others => '0');
    signal s_thr_lo : unsigned(10 downto 0) := (others => '0');

    -- Effect levels 0..7 (knob + slider, saturated), registered per clock
    signal s_lvl_streak : unsigned(2 downto 0) := (others => '0');
    signal s_lvl_height : unsigned(2 downto 0) := (others => '0');
    signal s_lvl_tear   : unsigned(2 downto 0) := (others => '0');
    signal s_pitch_max  : unsigned(2 downto 0) := "001";  -- pitch-1: 1..4

    -- Level -> random-length masks, registered
    signal s_out_mask   : unsigned(7 downto 0) := (others => '0');  -- out-run px
    signal s_in_mask    : unsigned(7 downto 0) := (others => '0');  -- in-run px
    signal s_patch_mask : unsigned(5 downto 0) := (others => '0');  -- patch lines
    signal s_fil_mask   : unsigned(5 downto 0) := (others => '0');  -- strand lines

    -- Pixel pipeline delays
    signal s_y_d1, s_y_d2 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_u_d1, s_u_d2 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_v_d1, s_v_d2 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_avid_d1, s_avid_d2 : std_logic := '0';
    signal s_hs_d1, s_hs_d2     : std_logic := '1';
    signal s_vs_d1, s_vs_d2     : std_logic := '1';
    signal s_fn_d1, s_fn_d2     : std_logic := '0';
    signal s_bwr   : std_logic := '0';  -- hysteresis threshold state (pixel d2)

    -- Column counter and delays (write addr = read addr delayed 3)
    signal s_hcnt    : unsigned(10 downto 0) := (others => '0');
    signal s_hcnt_d1 : unsigned(10 downto 0) := (others => '0');
    signal s_hcnt_d2 : unsigned(10 downto 0) := (others => '0');

    -- Line/column pitch phase
    signal s_linep    : unsigned(2 downto 0) := (others => '0');
    signal s_line_hit : std_logic := '1';
    signal s_colp     : unsigned(2 downto 0) := (others => '0');

    -- Line buffer
    signal s_lb_ab      : std_logic := '0';
    signal s_hs_prev    : std_logic := '1';
    signal s_vs_prev    : std_logic := '1';
    signal s_lb_wr_data : std_logic_vector(24 downto 0) := (others => '0');
    signal s_lb_wr_addr : unsigned(10 downto 0) := (others => '1');
    signal s_lb_rd_data : std_logic_vector(24 downto 0);

    -- Stage-3 state
    signal s_prev_bw  : std_logic := '0';
    signal s_run_len  : unsigned(3 downto 0) := (others => '0');
    signal s_out_cnt  : unsigned(7 downto 0) := (others => '0');
    signal s_out_life : unsigned(5 downto 0) := (others => '0');
    signal s_in_cnt   : unsigned(7 downto 0) := (others => '0');
    signal s_in_life  : unsigned(5 downto 0) := (others => '0');

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
    s_knob_height <= unsigned(registers_in(2));
    s_knob_tear   <= unsigned(registers_in(3));
    s_knob_pitch  <= unsigned(registers_in(5));
    s_slider      <= unsigned(registers_in(7));
    s_sw_negate   <= registers_in(6)(0);
    s_sw_bones    <= registers_in(6)(1);
    s_sw_bypass   <= registers_in(6)(2);
    s_sw_rawthr   <= registers_in(6)(3);
    s_sw_loose    <= registers_in(6)(4);

    --------------------------------------------------------------------------
    -- Effect levels and masks (slow -- knobs only), registered
    --------------------------------------------------------------------------
    p_levels : process(clk)
        variable v_shred : unsigned(1 downto 0);
    begin
        if rising_edge(clk) then
            v_shred := s_slider(9 downto 8);
            s_lvl_streak <= f_sat7(s_knob_streak(9 downto 7), v_shred);
            s_lvl_height <= f_sat7(s_knob_height(9 downto 7), v_shred);
            s_lvl_tear   <= f_sat7(s_knob_tear(9 downto 7), v_shred);
            -- pitch: line/column spacing inside groups, 2..5 px
            s_pitch_max  <= ('0' & s_knob_pitch(9 downto 8)) + 1;

            -- hysteresis bands; S10 Raw collapses the dead band to zero
            if s_sw_rawthr = '1' then
                s_thr_hi <= '0' & s_knob_thresh;
                s_thr_lo <= '0' & s_knob_thresh;
            else
                s_thr_hi <= ('0' & s_knob_thresh) + 24;
                if s_knob_thresh > 24 then
                    s_thr_lo <= '0' & (s_knob_thresh - 24);
                else
                    s_thr_lo <= (others => '0');
                end if;
            end if;

            -- out-fringe run length (px over black): every level distinct,
            -- 255 px at the top so the knob is unmistakable
            case to_integer(s_lvl_streak) is
                when 0      => s_out_mask <= to_unsigned(0, 8);
                when 1      => s_out_mask <= to_unsigned(7, 8);
                when 2      => s_out_mask <= to_unsigned(15, 8);
                when 3      => s_out_mask <= to_unsigned(31, 8);
                when 4      => s_out_mask <= to_unsigned(63, 8);
                when 5      => s_out_mask <= to_unsigned(127, 8);
                when 6      => s_out_mask <= to_unsigned(255, 8);
                when others => s_out_mask <= to_unsigned(255, 8);
            end case;

            -- in-fringe run length (px eaten into white)
            case to_integer(s_lvl_tear) is
                when 0      => s_in_mask <= to_unsigned(0, 8);
                when 1      => s_in_mask <= to_unsigned(7, 8);
                when 2      => s_in_mask <= to_unsigned(15, 8);
                when 3      => s_in_mask <= to_unsigned(31, 8);
                when 4      => s_in_mask <= to_unsigned(63, 8);
                when 5      => s_in_mask <= to_unsigned(127, 8);
                when others => s_in_mask <= to_unsigned(255, 8);
            end case;

            -- patch height (lines) and filament length (lines)
            case to_integer(s_lvl_height) is
                when 0 =>
                    s_patch_mask <= to_unsigned(0, 6);
                    s_fil_mask   <= to_unsigned(0, 6);
                when 1 =>
                    s_patch_mask <= to_unsigned(3, 6);
                    s_fil_mask   <= to_unsigned(7, 6);
                when 2 =>
                    s_patch_mask <= to_unsigned(7, 6);
                    s_fil_mask   <= to_unsigned(15, 6);
                when 3 =>
                    s_patch_mask <= to_unsigned(15, 6);
                    s_fil_mask   <= to_unsigned(31, 6);
                when 4 =>
                    s_patch_mask <= to_unsigned(31, 6);
                    s_fil_mask   <= to_unsigned(63, 6);
                when others =>
                    s_patch_mask <= to_unsigned(63, 6);
                    s_fil_mask   <= to_unsigned(63, 6);
            end case;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Delay pipeline, counters, pitch phases, line-buffer bank toggle
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

            -- bw threshold at d2 (from d1 data) with hysteresis: white above
            -- hi band, black below lo band, HOLD previous pixel in between
            -- (plain clocked-if hold -- no variable feedback for yosys to
            -- mangle; negate is applied downstream in p_shred)
            if s_hs_d1 = '0' then
                s_bwr <= '0';
            elsif ('0' & unsigned(s_y_d1)) > s_thr_hi then
                s_bwr <= '1';
            elsif ('0' & unsigned(s_y_d1)) < s_thr_lo then
                s_bwr <= '0';
            end if;

            -- column counter tracks the incoming pixel
            if data_in.hsync_n = '0' then
                s_hcnt <= (others => '0');
            elsif data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            end if;
            s_hcnt_d1 <= s_hcnt;
            s_hcnt_d2 <= s_hcnt_d1;

            s_hs_prev <= data_in.hsync_n;
            s_vs_prev <= data_in.vsync_n;

            -- bank toggle + line pitch phase at hsync start
            if s_hs_prev = '1' and data_in.hsync_n = '0' then
                s_lb_ab <= not s_lb_ab;
                if s_vs_prev = '1' and data_in.vsync_n = '0' then
                    s_linep <= (others => '0');
                elsif s_linep >= s_pitch_max then
                    s_linep <= (others => '0');
                else
                    s_linep <= s_linep + 1;
                end if;
            end if;
            if s_linep = 0 then
                s_line_hit <= '1';
            else
                s_line_hit <= '0';
            end if;

            -- column pitch phase, aligned with stage 3 (d2 pixel stream)
            if s_hs_d2 = '0' then
                s_colp <= (others => '0');
            elsif s_avid_d2 = '1' then
                if s_colp >= s_pitch_max then
                    s_colp <= (others => '0');
                else
                    s_colp <= s_colp + 1;
                end if;
            end if;

            s_boot <= '0';
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage 3: fringe runs + patch lives + hatch/filament, compose output
    --------------------------------------------------------------------------
    p_shred : process(clk)
        variable v_bw        : std_logic;
        variable v_bw_above  : std_logic;
        variable v_fil       : unsigned(5 downto 0);
        variable v_vhatch    : unsigned(5 downto 0);
        variable v_fout      : unsigned(5 downto 0);
        variable v_fin       : unsigned(5 downto 0);
        variable v_fil_new    : unsigned(5 downto 0);
        variable v_vhatch_new : unsigned(5 downto 0);
        variable v_fout_new   : unsigned(5 downto 0);
        variable v_fin_new    : unsigned(5 downto 0);
        variable v_rand_a    : unsigned(7 downto 0);
        variable v_rand_b    : unsigned(7 downto 0);
        variable v_randh_a   : unsigned(5 downto 0);
        variable v_randh_b   : unsigned(5 downto 0);
        variable v_qual      : std_logic;
        variable v_col_hit   : std_logic;
        variable v_h_edge    : std_logic;
        variable v_out_zone  : std_logic;
        variable v_in_zone   : std_logic;
        variable v_out_bit   : std_logic;
    begin
        if rising_edge(clk) then
            v_bw       := s_bwr xor s_sw_negate;
            v_bw_above := s_lb_rd_data(24);
            v_fil      := unsigned(s_lb_rd_data(23 downto 18));
            v_vhatch   := unsigned(s_lb_rd_data(17 downto 12));
            v_fout     := unsigned(s_lb_rd_data(11 downto 6));
            v_fin      := unsigned(s_lb_rd_data(5 downto 0));
            v_rand_a   := unsigned(s_lfsr_a(7 downto 0));
            v_rand_b   := unsigned(s_lfsr_b(7 downto 0));
            v_randh_a  := unsigned(s_lfsr_a(13 downto 8));
            v_randh_b  := unsigned(s_lfsr_b(13 downto 8));

            if s_colp = 0 then
                v_col_hit := '1';
            else
                v_col_hit := '0';
            end if;

            ------------------------------------------------------------------
            -- Fringe runs: fire at edges with random length + random patch
            -- height (latched per run -- coherent patches, never per-pixel)
            ------------------------------------------------------------------
            -- Edge qualification: a transition only fires a patch when the
            -- run it terminates was >= 6 px.  Threshold-noise flicker makes
            -- micro-runs; without this every gray region becomes one giant
            -- fringe zone = full-width hatch bars on real video.
            -- S11 Loose bypasses the qualification (diagnostic / denser look).
            v_qual := '0';
            if s_run_len >= 6 or s_sw_loose = '1' then
                v_qual := '1';
            end if;

            if s_hs_d2 = '0' then
                s_out_cnt <= (others => '0');
                s_in_cnt  <= (others => '0');
                s_prev_bw <= '0';
                s_run_len <= (others => '0');
            elsif s_avid_d2 = '1' then
                s_prev_bw <= v_bw;
                if v_bw /= s_prev_bw then
                    s_run_len <= to_unsigned(1, 4);
                elsif s_run_len /= 15 then
                    s_run_len <= s_run_len + 1;
                end if;
                -- out fringe: fires where white ends, extends over black
                if s_prev_bw = '1' and v_bw = '0' and v_qual = '1'
                   and s_out_mask /= 0 then
                    s_out_cnt  <= (v_rand_a and s_out_mask) or to_unsigned(1, 8);
                    s_out_life <= (v_randh_a and s_patch_mask) or to_unsigned(1, 6);
                elsif s_out_cnt /= 0 then
                    s_out_cnt <= s_out_cnt - 1;
                end if;
                -- in fringe: fires where white begins, eats into the body
                if s_prev_bw = '0' and v_bw = '1' and v_qual = '1'
                   and s_in_mask /= 0 then
                    s_in_cnt  <= (v_rand_b and s_in_mask) or to_unsigned(1, 8);
                    s_in_life <= (v_randh_b and s_patch_mask) or to_unsigned(1, 6);
                elsif s_in_cnt /= 0 then
                    s_in_cnt <= s_in_cnt - 1;
                end if;
            end if;

            ------------------------------------------------------------------
            -- Zone membership for THIS pixel
            ------------------------------------------------------------------
            v_out_zone := '0';
            if v_bw = '0' and (s_out_cnt /= 0 or v_fout /= 0) then
                v_out_zone := '1';
            end if;
            v_in_zone := '0';
            if v_bw = '1' and (s_in_cnt /= 0 or v_fin /= 0) then
                v_in_zone := '1';
            end if;

            v_h_edge := '0';
            if v_bw_above = '1' and v_bw = '0' then
                v_h_edge := '1';
            end if;

            ------------------------------------------------------------------
            -- Per-column lives written back to the line buffer
            ------------------------------------------------------------------
            v_fout_new := (others => '0');
            if v_bw = '0' then
                if s_out_cnt /= 0 then
                    v_fout_new := s_out_life;
                elsif v_fout /= 0 then
                    v_fout_new := v_fout - 1;
                end if;
            end if;

            v_fin_new := (others => '0');
            if v_bw = '1' then
                if s_in_cnt /= 0 then
                    v_fin_new := s_in_life;
                elsif v_fin /= 0 then
                    v_fin_new := v_fin - 1;
                end if;
            end if;

            v_vhatch_new := (others => '0');
            if v_bw = '0' then
                if v_h_edge = '1' and s_patch_mask /= 0 then
                    v_vhatch_new := (v_randh_b and s_patch_mask) or to_unsigned(1, 6);
                elsif v_vhatch /= 0 then
                    v_vhatch_new := v_vhatch - 1;
                end if;
            end if;

            v_fil_new := (others => '0');
            if v_bw = '0' then
                if v_h_edge = '1' and s_fil_mask /= 0 and v_rand_a(0) = '1' then
                    v_fil_new := (v_randh_a and s_fil_mask) or to_unsigned(1, 6);
                elsif v_fil /= 0 then
                    v_fil_new := v_fil - 1;
                end if;
            end if;

            s_lb_wr_data <= v_bw & std_logic_vector(v_fil_new)
                            & std_logic_vector(v_vhatch_new)
                            & std_logic_vector(v_fout_new)
                            & std_logic_vector(v_fin_new);
            if s_avid_d2 = '1' then
                s_lb_wr_addr <= s_hcnt_d2;
            else
                s_lb_wr_addr <= (others => '1');  -- park beyond active width
            end if;

            ------------------------------------------------------------------
            -- Compose: fringe zones render as thin-line groups, everything
            -- else is solid.  No noise anywhere.
            ------------------------------------------------------------------
            if s_sw_rawthr = '1' then
                -- S10 Raw: pure 1-bit threshold, engine fully out of the
                -- path (hard bisect: if this is black, the fault is in the
                -- threshold/output plumbing, not the hatch engine)
                v_out_bit := v_bw;
            elsif v_out_zone = '1' or v_in_zone = '1'
               or (s_sw_bones = '1' and v_bw = '1') then
                v_out_bit := s_line_hit;
            elsif v_bw = '1' then
                v_out_bit := '1';
            elsif v_fil /= 0 then
                v_out_bit := '1';                 -- solo vertical strand
            elsif v_vhatch /= 0 then
                v_out_bit := v_col_hit;           -- thin vertical line group
            else
                v_out_bit := '0';
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
            G_WIDTH => 25,
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
