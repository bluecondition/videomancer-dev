-- tb_motion_key.vhd - golden testbench: motion_key RTL vs model/wake_model.py
-- Streams NTSC-timed fields (858 clk/line, 262 lines, 243 x 720 active, top
-- field first) from stim.txt into motion_key and logs every finalizer write
-- (M grid, reference) plus the per-update energy to tb_out.txt. golden.py
-- compares the log bit-exactly with the model. Run via tb/run_golden.sh.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use ieee.std_logic_textio.all;

library work;
use work.video_stream_pkg.all;
use work.motion_key_pkg.all;

entity tb_motion_key is
    generic (
        G_STIM    : string  := "stim.txt";
        G_OUT     : string  := "tb_out.txt";
        G_NFIELDS : integer := 20;
        G_WAKE    : integer := 528;
        G_FLOOR   : integer := 154;
        G_CHROMA  : integer := 700;
        G_TID     : string  := "0000";     -- "0010" = 1080i59.94 (HD field logic)
        G_STEADY  : integer := 0
    );
end entity;

architecture sim of tb_motion_key is
    function to_sl(i : integer) return std_logic is
    begin
        if i /= 0 then return '1'; else return '0'; end if;
    end function;
    function to_slv(s : string) return std_logic_vector is
        variable v : std_logic_vector(3 downto 0);
    begin
        for i in 0 to 3 loop
            if s(s'left + i) = '1' then v(3 - i) := '1'; else v(3 - i) := '0'; end if;
        end loop;
        return v;
    end function;
    constant C_LINE  : integer := 858;
    constant C_LINES : integer := 262;
    constant C_V0    : integer := 19;     -- first active line
    constant C_H0    : integer := 122;    -- first active clock
    constant C_W     : integer := 720;

    signal clk  : std_logic := '0';
    signal done : boolean := false;
    signal vin  : t_video_stream_yuv444_30b :=
        (y => (others => '0'), u => (others => '0'), v => (others => '0'),
         avid => '0', hsync_n => '1', vsync_n => '1', field_n => '1');
    signal ap   : t_mk_analysis_params;
    signal rp   : t_mk_render_params;
    signal key, matte : unsigned(9 downto 0);
    signal st   : t_mk_stats;
    signal m_we, r_we, upd : std_logic;
    signal m_addr : unsigned(9 downto 0);
    signal m_data : unsigned(11 downto 0);
    signal r_addr : unsigned(11 downto 0);
    signal r_data : unsigned(11 downto 0);
begin
    clk <= not clk after 5 ns when not done;

    ap <= (wake => to_unsigned(G_WAKE, 10), floor => to_unsigned(G_FLOOR, 10),
           chroma => to_unsigned(G_CHROMA, 10), mode => '0', plate => '0',
           steady => to_sl(G_STEADY), both_fields => '0');
    rp <= (sens => to_unsigned(614, 10), softness => to_unsigned(512, 10),
           detail => (others => '0'), invert => '0', follow => '0');

    dut : entity work.motion_key
        port map (
            clk => clk, ana_in => vin, timing_id => to_slv(G_TID), ana_params => ap,
            rnd_avid => vin.avid, rnd_vsync_n => vin.vsync_n, rnd_y => vin.y,
            rnd_params => rp, key => key, matte => matte, stats => st,
            dbg_m_we => m_we, dbg_m_addr => m_addr, dbg_m_data => m_data,
            dbg_r_we => r_we, dbg_r_addr => r_addr, dbg_r_data => r_data,
            dbg_upd => upd);

    p_stim : process
        file fin     : text open read_mode is G_STIM;
        variable l   : line;
        variable px  : std_logic_vector(31 downto 0);
        type t_buf is array (0 to C_W - 1) of std_logic_vector(29 downto 0);
        variable buf : t_buf;
    begin
        for fld in 0 to G_NFIELDS - 1 loop
            for ln in 0 to C_LINES - 1 loop
                if ln >= C_V0 then
                    readline(fin, l);
                    for i in 0 to C_W - 1 loop
                        hread(l, px);
                        buf(i) := px(29 downto 0);
                    end loop;
                end if;
                for c in 0 to C_LINE - 1 loop
                    wait until rising_edge(clk);
                    if c < 64 then vin.hsync_n <= '0'; else vin.hsync_n <= '1'; end if;
                    if ln >= 3 and ln < 6 then vin.vsync_n <= '0'; else vin.vsync_n <= '1'; end if;
                    if fld mod 2 = 0 then vin.field_n <= '1'; else vin.field_n <= '0'; end if;
                    if ln >= C_V0 and c >= C_H0 and c < C_H0 + C_W then
                        vin.avid <= '1';
                        vin.y <= buf(c - C_H0)(29 downto 20);
                        vin.u <= buf(c - C_H0)(19 downto 10);
                        vin.v <= buf(c - C_H0)(9 downto 0);
                    else
                        vin.avid <= '0';
                        vin.y <= std_logic_vector(to_unsigned(64, 10));
                        vin.u <= std_logic_vector(to_unsigned(512, 10));
                        vin.v <= std_logic_vector(to_unsigned(512, 10));
                    end if;
                end loop;
            end loop;
        end loop;
        for i in 0 to 8 * C_LINE loop
            wait until rising_edge(clk);
        end loop;
        done <= true;
        wait;
    end process;

    p_log : process(clk)
        file fout      : text open write_mode is G_OUT;
        variable l     : line;
        variable cnt   : integer := 0;
        variable upd_q : std_logic := '0';
    begin
        if rising_edge(clk) then
            if m_we = '1' then
                write(l, string'("M ")); write(l, cnt); write(l, ' ');
                write(l, to_integer(m_addr)); write(l, ' '); write(l, to_integer(m_data));
                writeline(fout, l);
            end if;
            if r_we = '1' then
                write(l, string'("R ")); write(l, cnt); write(l, ' ');
                write(l, to_integer(r_addr)); write(l, ' '); write(l, to_integer(r_data));
                writeline(fout, l);
            end if;
            if upd_q = '1' then
                write(l, string'("E ")); write(l, cnt); write(l, ' ');
                write(l, to_integer(st.energy));
                writeline(fout, l);
                cnt := cnt + 1;
            end if;
            upd_q := upd;
        end if;
    end process;
end architecture;
