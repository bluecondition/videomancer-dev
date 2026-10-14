-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- gamut_engine.vhd -- a tiny accumulator engine for GAMUT's per-frame and
-- per-line geometry.  One 20-bit signed accumulator, one add/sub, a barrel
-- source shift, a serial 16x16 multiplier; the program lives in an EBR ROM
-- (gamut_prog_pkg, assembled by gamut_asm.py).  Operands come from a
-- register file / external-source mux owned by the top (rf_addr in the A
-- stage, ext_sel in the D stage, src_val captured at the end of D); stores
-- go out on dst_sel / dst_we / dst_val.
--
-- Pipeline: F (ROM -> ir), A, D1 (file address + external select; the top
-- registers both reads), D2 (operand shift / immediate), X (execute).
-- A taken jump flushes the five younger stages; MUL stalls until done.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.gamut_prog_pkg.all;

entity gamut_engine is
    port (
        clk     : in  std_logic;
        start   : in  std_logic;                -- run the frame program
        start_l : in  std_logic;                -- run the line program
        busy    : out std_logic;
        rf_addr : out unsigned(6 downto 0);     -- D1 stage: register-file read address
        ext_sel : out unsigned(7 downto 0);     -- D1 stage: source index (bit 7 = external)
        src_val : in  signed(19 downto 0);
        dst_sel : out unsigned(6 downto 0);
        dst_we  : out std_logic;
        dst_val : out signed(19 downto 0);
        m_res   : out signed(32 downto 0)
    );
end entity;

architecture rtl of gamut_engine is
    signal prog   : t_prog1024 := C_PROG;
    signal pc     : unsigned(9 downto 0) := (others => '0');
    signal ir     : std_logic_vector(23 downto 0) := (others => '0');
    signal ir_v, a_v, d_v, d2_v, x_v : std_logic := '0';
    signal run    : std_logic := '0';
    signal a_op, d_op, d2_op, x_op : unsigned(4 downto 0) := (others => '0');
    signal a_sh, d_sh, d2_sh, x_sh : unsigned(3 downto 0) := (others => '0');
    signal a_src, d_src     : unsigned(7 downto 0) := (others => '0');
    signal a_dst, d_dst, d2_dst, x_dst : unsigned(6 downto 0) := (others => '0');
    signal a_imm, d_imm, d2_imm, x_imm : signed(14 downto 0) := (others => '0');
    signal x_b    : signed(19 downto 0) := (others => '0');    -- operand, already shifted / immediate
    signal m_prep : std_logic := '0';
    signal acc    : signed(19 downto 0) := (others => '0');
    signal m_a    : unsigned(31 downto 0) := (others => '0');
    signal m_b    : unsigned(15 downto 0) := (others => '0');
    signal m_acc  : unsigned(31 downto 0) := (others => '0');
    signal m_cnt  : unsigned(4 downto 0) := (others => '0');
    signal m_busy, m_neg : std_logic := '0';

    function ilog2(v : signed(19 downto 0)) return integer is
        variable r : integer := 0;
    begin
        for i in 0 to 18 loop
            if v(i) = '1' then r := i; end if;
        end loop;
        return r;
    end function;
begin
    busy    <= run;
    rf_addr <= d_src(6 downto 0);
    ext_sel <= d_src;
    dst_sel <= x_dst;
    dst_val <= acc;
    dst_we  <= '1' when (x_v = '1' and x_op = OP_ST and m_busy = '0' and run = '1') else '0';
    m_res   <= -signed('0' & m_acc) when m_neg = '1' else signed('0' & m_acc);

    p_eng : process(clk)
        variable v_b, v_sum, v_dif : signed(19 downto 0);
        variable v_take : std_logic;
        variable v_word : std_logic_vector(23 downto 0);
        variable v_sr   : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            if m_busy = '1' then
                if m_prep = '1' then
                    -- sign-magnitude prep cycle (keeps the negates off the execute path)
                    m_prep <= '0';
                    if m_a(19) = '1' then m_a <= resize(unsigned(-signed(m_a(19 downto 0))), 32); end if;
                    if m_b(15) = '1' then m_b <= unsigned(-signed(m_b)); end if;
                else
                    if m_b(0) = '1' then m_acc <= m_acc + m_a; end if;
                    m_a <= m_a(30 downto 0) & '0';
                    m_b <= '0' & m_b(15 downto 1);
                    m_cnt <= m_cnt + 1;
                    if m_cnt = 15 then m_busy <= '0'; end if;
                end if;
            end if;

            v_word := prog(to_integer(pc));

            if (start = '1' or start_l = '1') and run = '0' then
                run <= '1';
                if start = '1' then pc <= (others => '0'); else pc <= to_unsigned(C_LINE_BASE, 10); end if;
                ir_v <= '0'; a_v <= '0'; d_v <= '0'; d2_v <= '0'; x_v <= '0';
            elsif run = '1' and m_busy = '0' then
                -- advance the pipeline
                ir <= v_word; ir_v <= '1'; pc <= pc + 1;
                a_op <= unsigned(ir(23 downto 19)); a_sh <= unsigned(ir(18 downto 15));
                a_src <= unsigned(ir(14 downto 7)); a_dst <= unsigned(ir(6 downto 0));
                a_imm <= signed(ir(14 downto 0)); a_v <= ir_v;
                d_op <= a_op; d_sh <= a_sh; d_src <= a_src; d_dst <= a_dst; d_imm <= a_imm; d_v <= a_v;
                d2_op <= d_op; d2_sh <= d_sh; d2_dst <= d_dst; d2_imm <= d_imm; d2_v <= d_v;
                x_op <= d2_op; x_sh <= d2_sh; x_dst <= d2_dst; x_imm <= d2_imm; x_v <= d2_v;
                -- operand select at the end of D2 (src_val is the top's REGISTERED read):
                -- barrel right shift 0..13, <<4, <<8, or immediate
                case to_integer(d2_sh) is
                    when 14 => v_sr := shift_left(src_val, 4);
                    when 15 => v_sr := shift_left(src_val, 8);
                    when others => v_sr := shift_right(src_val, to_integer(d2_sh));
                end case;
                if d2_op = OP_LDI or d2_op = OP_ADDI or d2_op = OP_SUBI or d2_op = OP_MINI or d2_op = OP_MAXI then
                    x_b <= resize(d2_imm, 20);
                else
                    x_b <= v_sr;
                end if;

                if x_v = '1' then
                    v_b := x_b;
                    v_sum := acc + v_b;
                    v_dif := acc - v_b;
                    v_take := '0';
                    case to_integer(x_op) is
                        when OP_LD | OP_LDI   => acc <= v_b;
                        when OP_ADD | OP_ADDI => acc <= v_sum;
                        when OP_SUB | OP_SUBI => acc <= v_dif;
                        when OP_MIN | OP_MINI => if v_dif > 0 then acc <= v_b; end if;
                        when OP_MAX | OP_MAXI => if v_dif < 0 then acc <= v_b; end if;
                        when OP_ABS  => if acc < 0 then acc <= -acc; end if;
                        when OP_LOG2 => acc <= to_signed(ilog2(acc), 20);
                        when OP_MUL  =>
                            m_a <= unsigned(resize(acc, 32)); m_b <= unsigned(v_b(15 downto 0));
                            m_neg <= acc(19) xor v_b(19);
                            m_acc <= (others => '0'); m_cnt <= (others => '0'); m_busy <= '1'; m_prep <= '1';
                        when OP_JMP  => v_take := '1';
                        when OP_JNEG => if acc < 0 then v_take := '1'; end if;
                        when OP_JPOS => if acc > 0 then v_take := '1'; end if;
                        when OP_JZ   => if acc = 0 then v_take := '1'; end if;
                        when OP_JNZ  => if acc /= 0 then v_take := '1'; end if;
                        when OP_END  => run <= '0';
                        when others  => null;
                    end case;
                    if v_take = '1' then
                        pc <= unsigned(x_imm(9 downto 0));
                        ir_v <= '0'; a_v <= '0'; d_v <= '0'; d2_v <= '0'; x_v <= '0';
                    end if;
                end if;
            end if;
        end if;
    end process;
end architecture;
