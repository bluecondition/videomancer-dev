-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of Videomancer Community Programs.
-- See LICENSE file in the repository root for full license text.
--
-- mood_hd.vhd -- MOOD HD (CLEAN render).  The design lives in mood_core.vhd,
-- copied from programs/ron/mood by the build hook (mood_hd.py).

library ieee;
use ieee.std_logic_1164.all;

library work;
use work.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;

architecture mood_hd of program_top is
begin
    core : entity work.mood_core
        generic map (G_CLEAN => true)
        port map (
            clk          => clk,
            registers_in => registers_in,
            data_in      => data_in,
            data_out     => data_out
        );
end architecture mood_hd;
