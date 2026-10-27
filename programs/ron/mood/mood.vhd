-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- mood.vhd -- MOOD (VINTAGE render).  The design lives in mood_core.vhd,
-- shared with MOOD HD (programs/ron/mood_hd, CLEAN render).

library ieee;
use ieee.std_logic_1164.all;

library work;
use work.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;

architecture mood of program_top is
begin
    core : entity work.mood_core
        generic map (G_CLEAN => false)
        port map (
            clk          => clk,
            registers_in => registers_in,
            data_in      => data_in,
            data_out     => data_out
        );
end architecture mood;
