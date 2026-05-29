-- Videomancer SDK - Open source FPGA-based video effects development kit
-- Copyright (C) 2026 ron
-- File: etchplate.vhd - 1-bit Threshold + Edge-Triggered Striped Bands
-- License: GNU General Public License v3.0
--
-- Etchplate v2.0 - FRESH START
--
-- Modeled directly on phosphor.vhd's edge-band state machine, which is
-- known to work correctly on rev_b hardware. Phosphor walks through 8
-- rainbow color bands to the right of each detected edge. Etchplate
-- does the same thing, but with just two "colors": INVERTED-bw (the
-- thin line) and ORIGINAL-bw (the gap between lines). Result is a
-- sequence of thin contrast lines with gaps between them, extending
-- to the right of each B/W edge.
--
-- Key architectural rule learned the hard way: state-machine outer
-- conditions must NEVER read the same register they write inside. The
-- phosphor pattern follows this rule. v0.9-v1.3 violated it (read
-- s_h_active in the outer condition and wrote it inside) and yosys
-- collapsed everything to inactive.
--
-- State machine:
--   At hsync         -> s_band_idx = 15 (inactive)
--   At fire          -> s_band_idx = 0, s_band_pixel = 0
--   At band end      -> s_band_pixel = 0, s_band_idx += 1 (or stops at max)
--   Otherwise        -> s_band_pixel += 1
--
-- Output per band index:
--   15 (inactive)    -> normal threshold (bw_eff)
--   even (0,2,4,...) -> "line" = inverted bw_eff
--   odd  (1,3,5,...) -> "gap"  = normal bw_eff
--
-- Controls:
--   K1 Threshold - luma threshold
--   K2 Density   - probability per edge that a band cluster fires
--   K3 Width     - width of each line/gap in pixels (1..64)
--   K4 Count     - number of bands per cluster (1..15)
--   K6 Force     - >= 50%: bypass LFSR coin gate (every edge fires)
--   S7 Negate    - swap B/W
--   S9 Bypass    - true 1-cycle latency-matched passthru
--------------------------------------------------------------------------------

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
    signal s_knob_dens   : unsigned(9 downto 0);
    signal s_knob_width  : unsigned(9 downto 0);
    signal s_knob_count  : unsigned(9 downto 0);
    signal s_knob_force  : unsigned(9 downto 0);
    signal s_sw_negate   : std_logic;
    signal s_sw_bypass   : std_logic;

    -- Derived
    signal s_dens_byte  : unsigned(7 downto 0);
    signal s_band_width : unsigned(5 downto 0);  -- 1..64 px per band
    signal s_band_max   : unsigned(3 downto 0);  -- max band index 1..15

    -- Edge detection state
    signal s_prev_bw : std_logic := '0';

    -- Band state machine (PHOSPHOR PATTERN)
    -- s_band_pixel  : counter 0..band_width-1
    -- s_band_idx    : 0..15, where 15 means "no active cluster"
    -- CRITICAL: outer conditions never read s_band_idx, only s_band_pixel.
    -- Writes to s_band_idx are nested inside conditions that read
    -- s_band_pixel. This is the phosphor pattern that synthesizes
    -- correctly on iCE40.
    signal s_band_pixel : unsigned(5 downto 0) := (others => '0');
    signal s_band_idx   : unsigned(3 downto 0) := "1111";  -- 15 = inactive

    -- LFSR
    signal s_lfsr_out : std_logic_vector(15 downto 0);

    -- Output registered
    signal s_y_out, s_u_out, s_v_out : std_logic_vector(9 downto 0) := (others => '0');
    signal s_avid_out, s_hs_out, s_vs_out, s_fn_out : std_logic := '0';

begin

    s_knob_thresh <= unsigned(registers_in(0));
    s_knob_dens   <= unsigned(registers_in(1));
    s_knob_width  <= unsigned(registers_in(2));
    s_knob_count  <= unsigned(registers_in(3));
    s_knob_force  <= unsigned(registers_in(5));
    s_sw_negate   <= registers_in(6)(0);
    s_sw_bypass   <= registers_in(6)(2);

    -- Density: top 8 bits -> 0..255
    s_dens_byte  <= s_knob_dens(9 downto 2);
    -- Band width: top 6 bits -> 0..63 (treat 0 as "huge band" effectively)
    s_band_width <= s_knob_width(9 downto 4);
    -- Band count max: top 4 bits -> 0..15
    s_band_max   <= s_knob_count(9 downto 6);

    --------------------------------------------------------------------------
    -- Single process: threshold + edge + phosphor-style band state + output
    --------------------------------------------------------------------------
    p_main : process(clk)
        variable v_bw_raw  : std_logic;
        variable v_bw_eff  : std_logic;
        variable v_edge    : std_logic;
        variable v_coin    : unsigned(7 downto 0);
        variable v_fire    : std_logic;
        variable v_out_bit : std_logic;
    begin
        if rising_edge(clk) then
            -- Threshold + negate
            if unsigned(data_in.y) > s_knob_thresh then
                v_bw_raw := '1';
            else
                v_bw_raw := '0';
            end if;
            v_bw_eff := v_bw_raw xor s_sw_negate;

            -- H edge detect
            v_edge := v_bw_eff xor s_prev_bw;

            -- Update prev_bw
            if data_in.hsync_n = '0' then
                s_prev_bw <= '0';
            elsif data_in.avid = '1' then
                s_prev_bw <= v_bw_eff;
            end if;

            -- Fire decision: LFSR coin OR force
            v_coin := unsigned(s_lfsr_out(7 downto 0));
            if v_edge = '1' and data_in.avid = '1'
               and (v_coin <= s_dens_byte or s_knob_force(9) = '1') then
                v_fire := '1';
            else
                v_fire := '0';
            end if;

            ------------------------------------------------------------------
            -- BAND STATE MACHINE - phosphor pattern verbatim.
            -- Outer condition reads s_band_pixel (a counter), NEVER reads
            -- s_band_idx in any outer elsif. s_band_idx is only written
            -- inside nested ifs that read s_band_pixel. This is the
            -- synthesizable pattern.
            ------------------------------------------------------------------
            if data_in.hsync_n = '0' then
                s_band_idx   <= "1111";
                s_band_pixel <= (others => '0');
            elsif v_fire = '1' then
                s_band_idx   <= (others => '0');
                s_band_pixel <= (others => '0');
            elsif s_band_pixel = s_band_width then
                s_band_pixel <= (others => '0');
                if s_band_idx /= "1111" and s_band_idx < s_band_max then
                    s_band_idx <= s_band_idx + 1;
                else
                    s_band_idx <= "1111";
                end if;
            else
                s_band_pixel <= s_band_pixel + 1;
            end if;

            ------------------------------------------------------------------
            -- Output bit:
            --   15 (inactive)    -> normal threshold
            --   even idx (0,2,4) -> "line" (inverted)
            --   odd  idx (1,3,5) -> "gap"  (normal)
            ------------------------------------------------------------------
            if s_band_idx = "1111" then
                v_out_bit := v_bw_eff;
            elsif s_band_idx(0) = '0' then
                v_out_bit := not v_bw_eff;
            else
                v_out_bit := v_bw_eff;
            end if;

            -- Output mux
            if s_sw_bypass = '1' then
                s_y_out <= data_in.y;
                s_u_out <= data_in.u;
                s_v_out <= data_in.v;
            else
                if v_out_bit = '1' then
                    s_y_out <= (others => '1');
                else
                    s_y_out <= (others => '0');
                end if;
                s_u_out <= std_logic_vector(to_unsigned(512, 10));
                s_v_out <= std_logic_vector(to_unsigned(512, 10));
            end if;

            s_avid_out <= data_in.avid;
            s_hs_out   <= data_in.hsync_n;
            s_vs_out   <= data_in.vsync_n;
            s_fn_out   <= data_in.field_n;
        end if;
    end process;

    data_out.y       <= s_y_out;
    data_out.u       <= s_u_out;
    data_out.v       <= s_v_out;
    data_out.avid    <= s_avid_out;
    data_out.hsync_n <= s_hs_out;
    data_out.vsync_n <= s_vs_out;
    data_out.field_n <= s_fn_out;

    lfsr_inst : entity work.lfsr16
        port map (
            clk    => clk,
            enable => '1',
            seed   => x"0000",
            load   => '0',
            q      => s_lfsr_out
        );

end architecture etchplate;
