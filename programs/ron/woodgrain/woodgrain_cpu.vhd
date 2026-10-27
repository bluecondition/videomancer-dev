-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- WOODGRAIN - 6507 CPU core (bus-cycle exact 6502).
--
-- One 6502 bus cycle = one 6-clock SLOT (ph 0..5) of the emulator:
--   ph0..ph2  the system performs this cycle's access at r_addr (read data
--             is registered into din by the end of ph2)
--   ph3       STEP: the state machine consumes din, updates registers and
--             sets the address / write for the NEXT cycle
--   ph4, ph5  ALU stage 1 (low nibble, logic, shifts) and stage 2 (high
--             nibble, decimal adjust, flags, write-back)
-- Every instruction takes its documented cycle count, including page-cross
-- and taken-branch extras; dummy reads are not performed (they would only
-- touch RAM/ROM/TIA reads with no side effects we model) but they cost
-- their cycle.  RMW instructions perform the real double write.
-- The 6507 has no IRQ / NMI pins; BRK is supported.
-- Decimal mode follows the py65 reference model (binary flags for SBC,
-- nibble-sum flags for ADC).  The stable illegal opcodes (LAX SAX DCP ISB
-- SLO RLA SRE RRA ANC ALR ARR SBX LXA, multi-byte NOPs) are decoded; the
-- rest run as 2-cycle NOPs.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.woodgrain_consts_pkg.all;

entity woodgrain_cpu is
    port (
        clk  : in  std_logic;
        ph   : in  unsigned(2 downto 0);
        act  : in  std_logic;                     -- slot active
        run  : in  std_logic;                     -- CPU executes this slot (not halted)
        rst  : in  std_logic;                     -- restart the reset sequence
        din  : in  std_logic_vector(7 downto 0);  -- this cycle's read data (from ph3)
        addr : out unsigned(15 downto 0);
        we   : out std_logic;
        dout : out std_logic_vector(7 downto 0);
        sync : out std_logic;                     -- this cycle is an opcode fetch
        dbg  : out std_logic_vector(55 downto 0)  -- A X Y S P PC (test benches only)
    );
end entity woodgrain_cpu;

architecture rtl of woodgrain_cpu is
    -- states (plain unsigned: no fsm_extract)
    subtype t_st is unsigned(5 downto 0);
    constant S_FETCH : t_st := to_unsigned(0, 6);
    constant S_C2    : t_st := to_unsigned(1, 6);
    constant S_ZPI   : t_st := to_unsigned(2, 6);
    constant S_ABH   : t_st := to_unsigned(3, 6);
    constant S_ABFIX : t_st := to_unsigned(4, 6);
    constant S_IZX1  : t_st := to_unsigned(5, 6);
    constant S_IZX2  : t_st := to_unsigned(6, 6);
    constant S_IZX3  : t_st := to_unsigned(7, 6);
    constant S_IZY1  : t_st := to_unsigned(8, 6);
    constant S_IZY2  : t_st := to_unsigned(9, 6);
    constant S_DATA  : t_st := to_unsigned(10, 6);
    constant S_RMW1  : t_st := to_unsigned(11, 6);
    constant S_RMW2  : t_st := to_unsigned(12, 6);
    constant S_BR1   : t_st := to_unsigned(13, 6);
    constant S_BR2   : t_st := to_unsigned(14, 6);
    constant S_IND1  : t_st := to_unsigned(15, 6);
    constant S_IND2  : t_st := to_unsigned(16, 6);
    constant S_JSR1  : t_st := to_unsigned(17, 6);
    constant S_JSR2  : t_st := to_unsigned(18, 6);
    constant S_JSR3  : t_st := to_unsigned(19, 6);
    constant S_JSR4  : t_st := to_unsigned(20, 6);
    constant S_RTS1  : t_st := to_unsigned(21, 6);
    constant S_RTS2  : t_st := to_unsigned(22, 6);
    constant S_RTS3  : t_st := to_unsigned(23, 6);
    constant S_RTS4  : t_st := to_unsigned(24, 6);
    constant S_RTI1  : t_st := to_unsigned(25, 6);
    constant S_RTI2  : t_st := to_unsigned(26, 6);
    constant S_RTI3  : t_st := to_unsigned(27, 6);
    constant S_RTI4  : t_st := to_unsigned(28, 6);
    constant S_BRK1  : t_st := to_unsigned(29, 6);
    constant S_BRK2  : t_st := to_unsigned(30, 6);
    constant S_BRK3  : t_st := to_unsigned(31, 6);
    constant S_BRK4  : t_st := to_unsigned(32, 6);
    constant S_BRK5  : t_st := to_unsigned(33, 6);
    constant S_PSH1  : t_st := to_unsigned(34, 6);
    constant S_PUL1  : t_st := to_unsigned(35, 6);
    constant S_PUL2  : t_st := to_unsigned(36, 6);
    constant S_RST0  : t_st := to_unsigned(37, 6);
    constant S_RST1  : t_st := to_unsigned(38, 6);
    constant S_RST2  : t_st := to_unsigned(39, 6);

    signal r_st : t_st := S_RST0;
    attribute fsm_encoding : string;
    attribute fsm_encoding of r_st : signal is "none";

    -- programmer-visible registers
    signal r_a, r_x, r_y : unsigned(7 downto 0) := (others => '0');
    signal r_s  : unsigned(7 downto 0) := x"FD";
    signal p_n, p_v, p_d, p_i, p_z, p_c : std_logic := '0';
    signal r_pc : unsigned(15 downto 0) := (others => '0');

    -- internal latches
    signal r_ir    : std_logic_vector(7 downto 0) := x"EA";
    signal r_adl, r_adh : unsigned(7 downto 0) := (others => '0');
    signal r_ba    : unsigned(7 downto 0) := (others => '0');
    signal r_dl    : unsigned(7 downto 0) := (others => '0');
    signal r_cross : std_logic := '0';

    -- bus for the next cycle
    signal r_addr : unsigned(15 downto 0) := x"FFFC";
    signal r_we   : std_logic := '0';
    signal r_dout : std_logic_vector(7 downto 0) := (others => '0');

    -- decode: EBR ROM read every clock with the opcode on din, captured at
    -- ph4 of an opcode-fetch slot (din = opcode during ph3)
    signal dec_raw : std_logic_vector(31 downto 0) := (others => '0');
    signal r_dec   : std_logic_vector(31 downto 0) := C_WG_DECODE(16#EA#);
    signal r_dload : std_logic := '0';

    alias d_md   : std_logic_vector(3 downto 0) is r_dec(3 downto 0);
    alias d_acc  : std_logic_vector(1 downto 0) is r_dec(5 downto 4);
    alias d_aop  : std_logic_vector(4 downto 0) is r_dec(10 downto 6);
    alias d_asel : std_logic_vector(1 downto 0) is r_dec(12 downto 11);
    alias d_bsel : std_logic_vector(2 downto 0) is r_dec(15 downto 13);
    alias d_dst  : std_logic_vector(2 downto 0) is r_dec(18 downto 16);
    alias d_rop  : std_logic_vector(2 downto 0) is r_dec(21 downto 19);
    alias d_wsel : std_logic_vector(1 downto 0) is r_dec(23 downto 22);
    alias d_spec : std_logic_vector(3 downto 0) is r_dec(27 downto 24);
    alias d_fl   : std_logic_vector(2 downto 0) is r_dec(30 downto 28);

    -- ALU request (set at the step) and its two stages
    signal q_go  : std_logic := '0';
    signal q_op  : unsigned(4 downto 0) := (others => '0');
    signal q_a, q_b : unsigned(7 downto 0) := (others => '0');
    signal q_dst : unsigned(2 downto 0) := (others => '0');

    signal s1_go  : std_logic := '0';
    signal s1_op  : unsigned(4 downto 0) := (others => '0');
    signal s1_dst : unsigned(2 downto 0) := (others => '0');
    signal s1_a, s1_b : unsigned(7 downto 0) := (others => '0');   -- s1_b already inverted for SBC/CMP
    signal s1_n0  : unsigned(4 downto 0) := (others => '0');
    signal s1_r   : unsigned(7 downto 0) := (others => '0');       -- logic / shift result
    signal s1_c   : std_logic := '0';                              -- shift / ANC / ALR carry
    signal s1_dec : std_logic := '0';

    constant MD_IMP : std_logic_vector(3 downto 0) := x"0";
    constant MD_ACC : std_logic_vector(3 downto 0) := x"1";
    constant MD_IMM : std_logic_vector(3 downto 0) := x"2";
    constant MD_ZP  : std_logic_vector(3 downto 0) := x"3";
    constant MD_ZPX : std_logic_vector(3 downto 0) := x"4";
    constant MD_ZPY : std_logic_vector(3 downto 0) := x"5";
    constant MD_ABS : std_logic_vector(3 downto 0) := x"6";
    constant MD_ABX : std_logic_vector(3 downto 0) := x"7";
    constant MD_ABY : std_logic_vector(3 downto 0) := x"8";
    constant MD_IZX : std_logic_vector(3 downto 0) := x"9";
    constant MD_IZY : std_logic_vector(3 downto 0) := x"A";
    constant MD_REL : std_logic_vector(3 downto 0) := x"B";
    constant MD_SPC : std_logic_vector(3 downto 0) := x"C";

    constant AC_R : std_logic_vector(1 downto 0) := "01";
    constant AC_W : std_logic_vector(1 downto 0) := "10";
    constant AC_M : std_logic_vector(1 downto 0) := "11";

    constant OP_NONE : unsigned(4 downto 0) := to_unsigned(0, 5);
    constant OP_LD   : unsigned(4 downto 0) := to_unsigned(1, 5);
    constant OP_ORA  : unsigned(4 downto 0) := to_unsigned(2, 5);
    constant OP_AND  : unsigned(4 downto 0) := to_unsigned(3, 5);
    constant OP_EOR  : unsigned(4 downto 0) := to_unsigned(4, 5);
    constant OP_ADC  : unsigned(4 downto 0) := to_unsigned(5, 5);
    constant OP_SBC  : unsigned(4 downto 0) := to_unsigned(6, 5);
    constant OP_CMP  : unsigned(4 downto 0) := to_unsigned(7, 5);
    constant OP_BIT  : unsigned(4 downto 0) := to_unsigned(8, 5);
    constant OP_INC  : unsigned(4 downto 0) := to_unsigned(9, 5);
    constant OP_DEC  : unsigned(4 downto 0) := to_unsigned(10, 5);
    constant OP_ASL  : unsigned(4 downto 0) := to_unsigned(11, 5);
    constant OP_LSR  : unsigned(4 downto 0) := to_unsigned(12, 5);
    constant OP_ROL  : unsigned(4 downto 0) := to_unsigned(13, 5);
    constant OP_ROR  : unsigned(4 downto 0) := to_unsigned(14, 5);
    constant OP_ANC  : unsigned(4 downto 0) := to_unsigned(15, 5);
    constant OP_ALR  : unsigned(4 downto 0) := to_unsigned(16, 5);
    constant OP_ARR  : unsigned(4 downto 0) := to_unsigned(17, 5);
    constant OP_SBX  : unsigned(4 downto 0) := to_unsigned(18, 5);

    constant SP_BRK : std_logic_vector(3 downto 0) := x"1";
    constant SP_JSR : std_logic_vector(3 downto 0) := x"2";
    constant SP_RTI : std_logic_vector(3 downto 0) := x"3";
    constant SP_RTS : std_logic_vector(3 downto 0) := x"4";
    constant SP_PHA : std_logic_vector(3 downto 0) := x"5";
    constant SP_PHP : std_logic_vector(3 downto 0) := x"6";
    constant SP_PLA : std_logic_vector(3 downto 0) := x"7";
    constant SP_PLP : std_logic_vector(3 downto 0) := x"8";
    constant SP_JMPA : std_logic_vector(3 downto 0) := x"9";
    constant SP_JMPI : std_logic_vector(3 downto 0) := x"A";
    constant SP_TXS : std_logic_vector(3 downto 0) := x"B";

    function f_pbyte(n, v, d, i, z, c, b : std_logic) return std_logic_vector is
    begin
        return n & v & '1' & b & d & i & z & c;
    end function;
begin
    addr <= r_addr;
    we   <= r_we;
    dout <= r_dout;
    sync <= '1' when r_st = S_FETCH else '0';
    dbg  <= std_logic_vector(r_a) & std_logic_vector(r_x) & std_logic_vector(r_y)
            & std_logic_vector(r_s) & f_pbyte(p_n, p_v, p_d, p_i, p_z, p_c, '1')
            & std_logic_vector(r_pc);

    -- decode ROM (EBR): address = din every clock
    p_decrom : process(clk)
    begin
        if rising_edge(clk) then
            dec_raw <= C_WG_DECODE(to_integer(unsigned(din)));
        end if;
    end process p_decrom;

    p_cpu : process(clk)
        variable v_din   : unsigned(7 downto 0);
        variable v_sum   : unsigned(8 downto 0);
        variable v_r     : unsigned(7 downto 0);
        variable v_cond  : std_logic;
        variable v_wval  : unsigned(7 downto 0);
        variable v_b     : unsigned(7 downto 0);
        variable v_a     : unsigned(7 downto 0);
        variable v_n1    : unsigned(4 downto 0);
        variable v_hc    : std_logic;
        variable v_c     : std_logic;
        variable v_alu   : unsigned(7 downto 0);
        variable v_res   : unsigned(7 downto 0);
        variable v_lo, v_hi : unsigned(3 downto 0);
        variable v_isb   : boolean;

        -- set up a data cycle at the effective address
        procedure go_data(ad : unsigned(15 downto 0)) is
        begin
            r_addr <= ad;
            if d_acc = AC_W then
                r_we <= '1';
                case d_wsel is
                    when "00"   => r_dout <= std_logic_vector(r_a);
                    when "01"   => r_dout <= std_logic_vector(r_x);
                    when "10"   => r_dout <= std_logic_vector(r_y);
                    when others => r_dout <= std_logic_vector(r_a and r_x);
                end case;
            else
                r_we <= '0';
            end if;
            r_st <= S_DATA;
        end procedure;

        procedure go_fetch(pc : unsigned(15 downto 0)) is
        begin
            r_addr <= pc;
            r_we   <= '0';
            r_st   <= S_FETCH;
        end procedure;

        procedure alu(op : unsigned(4 downto 0); b : unsigned(7 downto 0)) is
        begin
            q_go <= '1';
            q_op <= op;
            q_b  <= b;
            case d_asel is
                when "00"   => q_a <= r_a;
                when "01"   => q_a <= r_x;
                when "10"   => q_a <= r_y;
                when others => q_a <= r_a and r_x;
            end case;
            q_dst <= unsigned(d_dst);
        end procedure;

        function bsel_val(sel : std_logic_vector(2 downto 0); dv, a, x, y, s : unsigned(7 downto 0))
            return unsigned is
        begin
            case sel is
                when "001"  => return a;
                when "010"  => return x;
                when "011"  => return y;
                when "100"  => return s;
                when others => return dv;
            end case;
        end function;
    begin
        if rising_edge(clk) then
            -- ============ decode capture (ph4 of an opcode-fetch slot) ============
            if act = '1' and ph = 4 and r_dload = '1' then
                r_dec   <= dec_raw;
                r_dload <= '0';
            end if;

            -- ============ ALU stage 1 (ph4) ============
            s1_go <= '0';
            if act = '1' and ph = 4 and q_go = '1' then
                q_go   <= '0';
                s1_go  <= '1';
                s1_op  <= q_op;
                s1_dst <= q_dst;
                s1_a   <= q_a;
                s1_dec <= p_d;
                v_b := q_b;
                if q_op = OP_SBC or q_op = OP_CMP or q_op = OP_SBX then
                    v_b := not q_b;
                end if;
                s1_b <= v_b;
                if q_op = OP_CMP or q_op = OP_SBX then
                    v_c := '1';
                else
                    v_c := p_c;
                end if;
                s1_n0 <= ('0' & q_a(3 downto 0)) + ('0' & v_b(3 downto 0)) + ("0000" & v_c);
                s1_c  <= p_c;
                case q_op is
                    when OP_LD  => s1_r <= q_b;
                    when OP_ORA => s1_r <= q_a or q_b;
                    when OP_AND => s1_r <= q_a and q_b;
                    when OP_EOR => s1_r <= q_a xor q_b;
                    when OP_BIT => s1_r <= q_a and q_b;
                    when OP_INC => s1_r <= q_b + 1;
                    when OP_DEC => s1_r <= q_b - 1;
                    when OP_ASL => s1_r <= q_b(6 downto 0) & '0';          s1_c <= q_b(7);
                    when OP_LSR => s1_r <= '0' & q_b(7 downto 1);          s1_c <= q_b(0);
                    when OP_ROL => s1_r <= q_b(6 downto 0) & p_c;          s1_c <= q_b(7);
                    when OP_ROR => s1_r <= p_c & q_b(7 downto 1);          s1_c <= q_b(0);
                    when OP_ANC => s1_r <= q_a and q_b;                    s1_c <= q_a(7) and q_b(7);
                    when OP_ALR =>
                        v_r := q_a and q_b;
                        s1_r <= '0' & v_r(7 downto 1);
                        s1_c <= v_r(0);
                    when OP_ARR =>
                        v_r := q_a and q_b;
                        s1_r <= p_c & v_r(7 downto 1);
                    when others => s1_r <= q_b;
                end case;
            end if;

            -- ============ ALU stage 2 (ph5): finish, flags, write-back ============
            if act = '1' and ph = 5 and s1_go = '1' then
                -- arithmetic high nibble
                if s1_op = OP_ADC and s1_dec = '1' then
                    if s1_n0 > 9 then v_hc := '1'; else v_hc := '0'; end if;
                else
                    v_hc := s1_n0(4);
                end if;
                v_n1  := ('0' & s1_a(7 downto 4)) + ('0' & s1_b(7 downto 4)) + ("0000" & v_hc);
                v_alu := v_n1(3 downto 0) & s1_n0(3 downto 0);
                v_res := s1_r;
                case s1_op is
                    when OP_ADC =>
                        if s1_dec = '1' then
                            if v_n1 > 9 then p_c <= '1'; v_hi := v_n1(3 downto 0) + 6;
                            else              p_c <= '0'; v_hi := v_n1(3 downto 0); end if;
                            if v_hc = '1' then v_lo := s1_n0(3 downto 0) + 6;
                            else               v_lo := s1_n0(3 downto 0); end if;
                            v_res := v_hi & v_lo;
                        else
                            p_c <= v_n1(4);
                            v_res := v_alu;
                        end if;
                        p_v <= (not (s1_a(7) xor s1_b(7))) and (s1_a(7) xor v_alu(7));
                        p_n <= v_alu(7);
                        if v_alu = 0 then p_z <= '1'; else p_z <= '0'; end if;
                    when OP_SBC =>
                        p_c <= v_n1(4);
                        if s1_dec = '1' then
                            v_hi := v_n1(3 downto 0);
                            v_lo := s1_n0(3 downto 0);
                            if v_n1(4) = '0' then v_hi := v_hi + 10; end if;
                            if v_hc = '0' then v_lo := v_lo + 10; end if;
                            v_res := v_hi & v_lo;
                        else
                            v_res := v_alu;
                        end if;
                        p_v <= (not (s1_a(7) xor s1_b(7))) and (s1_a(7) xor v_alu(7));
                        p_n <= v_alu(7);
                        if v_alu = 0 then p_z <= '1'; else p_z <= '0'; end if;
                    when OP_CMP | OP_SBX =>
                        p_c <= v_n1(4);
                        p_n <= v_alu(7);
                        if v_alu = 0 then p_z <= '1'; else p_z <= '0'; end if;
                        v_res := v_alu;
                    when OP_BIT =>
                        p_n <= s1_b(7);
                        p_v <= s1_b(6);
                        if s1_r = 0 then p_z <= '1'; else p_z <= '0'; end if;
                    when OP_ASL | OP_LSR | OP_ROL | OP_ROR | OP_ALR | OP_ANC =>
                        p_c <= s1_c;
                        p_n <= s1_r(7);
                        if s1_r = 0 then p_z <= '1'; else p_z <= '0'; end if;
                    when OP_ARR =>
                        p_c <= s1_r(6);
                        p_v <= s1_r(6) xor s1_r(5);
                        p_n <= s1_r(7);
                        if s1_r = 0 then p_z <= '1'; else p_z <= '0'; end if;
                    when OP_NONE =>
                        null;
                    when others =>        -- LD ORA AND EOR INC DEC
                        p_n <= s1_r(7);
                        if s1_r = 0 then p_z <= '1'; else p_z <= '0'; end if;
                end case;
                if s1_op /= OP_CMP and s1_op /= OP_BIT and s1_op /= OP_NONE then
                    case s1_dst is
                        when "001" => r_a <= v_res;
                        when "010" => r_x <= v_res;
                        when "011" => r_y <= v_res;
                        when "100" => r_a <= v_res; r_x <= v_res;
                        when others => null;
                    end case;
                end if;
            end if;

            -- ============ STEP (ph3) ============
            if act = '1' and run = '1' and ph = 3 then
                v_din := unsigned(din);
                case r_st is
                    -- ---------------- opcode fetch ----------------
                    when S_FETCH =>
                        r_ir    <= din;
                        r_dload <= '1';
                        r_pc    <= r_pc + 1;
                        r_addr  <= r_pc + 1;
                        r_we    <= '0';
                        r_st    <= S_C2;

                    -- ---------------- second cycle ----------------
                    when S_C2 =>
                        case d_md is
                            when MD_IMP | MD_ACC =>
                                case d_fl is
                                    when "001" => p_c <= '0';
                                    when "010" => p_c <= '1';
                                    when "011" => p_i <= '0';
                                    when "100" => p_i <= '1';
                                    when "101" => p_v <= '0';
                                    when "110" => p_d <= '0';
                                    when "111" => p_d <= '1';
                                    when others => null;
                                end case;
                                if d_spec = SP_TXS then
                                    r_s <= r_x;
                                end if;
                                if unsigned(d_aop) /= OP_NONE then
                                    alu(unsigned(d_aop), bsel_val(d_bsel, v_din, r_a, r_x, r_y, r_s));
                                end if;
                                go_fetch(r_pc);
                            when MD_IMM =>
                                if unsigned(d_aop) /= OP_NONE then
                                    alu(unsigned(d_aop), v_din);
                                end if;
                                r_pc <= r_pc + 1;
                                go_fetch(r_pc + 1);
                            when MD_ZP =>
                                r_pc <= r_pc + 1;
                                r_adl <= v_din;
                                r_adh <= x"00";
                                go_data(x"00" & v_din);
                            when MD_ZPX | MD_ZPY | MD_IZX =>
                                r_pc <= r_pc + 1;
                                r_ba <= v_din;
                                r_addr <= x"00" & v_din;            -- dummy read
                                r_we <= '0';
                                if d_md = MD_IZX then r_st <= S_IZX1; else r_st <= S_ZPI; end if;
                            when MD_IZY =>
                                r_pc <= r_pc + 1;
                                r_ba <= v_din;
                                r_addr <= x"00" & v_din;            -- pointer low
                                r_we <= '0';
                                r_st <= S_IZY1;
                            when MD_ABS | MD_ABX | MD_ABY =>
                                r_pc <= r_pc + 1;
                                r_adl <= v_din;
                                r_addr <= r_pc + 1;                 -- high byte
                                r_we <= '0';
                                r_st <= S_ABH;
                            when MD_REL =>
                                r_pc <= r_pc + 1;
                                r_dl <= v_din;
                                case r_ir(7 downto 6) is
                                    when "00"   => v_cond := p_n;
                                    when "01"   => v_cond := p_v;
                                    when "10"   => v_cond := p_c;
                                    when others => v_cond := p_z;
                                end case;
                                if v_cond = r_ir(5) then
                                    r_addr <= r_pc + 1;             -- dummy read
                                    r_we <= '0';
                                    r_st <= S_BR1;
                                else
                                    go_fetch(r_pc + 1);
                                end if;
                            when others =>                          -- MD_SPC
                                case d_spec is
                                    when SP_JSR =>
                                        r_pc <= r_pc + 1;
                                        r_adl <= v_din;
                                        r_addr <= x"01" & r_s;      -- dummy stack read
                                        r_we <= '0';
                                        r_st <= S_JSR1;
                                    when SP_BRK =>
                                        r_pc <= r_pc + 1;
                                        r_addr <= x"01" & r_s;
                                        r_we <= '1';
                                        r_dout <= std_logic_vector(r_pc(15 downto 8) + ("0000000" & (r_pc(7) and r_pc(6) and r_pc(5) and r_pc(4) and r_pc(3) and r_pc(2) and r_pc(1) and r_pc(0))));
                                        r_st <= S_BRK1;
                                    when SP_PHA | SP_PHP =>
                                        r_addr <= x"01" & r_s;
                                        r_we <= '1';
                                        if d_spec = SP_PHA then
                                            r_dout <= std_logic_vector(r_a);
                                        else
                                            r_dout <= f_pbyte(p_n, p_v, p_d, p_i, p_z, p_c, '1');
                                        end if;
                                        r_st <= S_PSH1;
                                    when others =>                  -- RTS RTI PLA PLP
                                        r_addr <= x"01" & r_s;      -- dummy stack read
                                        r_we <= '0';
                                        if d_spec = SP_RTS then
                                            r_st <= S_RTS1;
                                        elsif d_spec = SP_RTI then
                                            r_st <= S_RTI1;
                                        else
                                            r_st <= S_PUL1;
                                        end if;
                                end case;
                        end case;

                    -- ---------------- zero page indexed: add the index ----------------
                    when S_ZPI =>
                        if d_md = MD_ZPY then
                            v_r := r_ba + r_y;
                        else
                            v_r := r_ba + r_x;
                        end if;
                        r_adl <= v_r;
                        r_adh <= x"00";
                        go_data(x"00" & v_r);

                    -- ---------------- absolute: high byte (+ index) ----------------
                    when S_ABH =>
                        r_pc <= r_pc + 1;
                        if d_spec = SP_JMPA then
                            r_pc <= v_din & r_adl;
                            go_fetch(v_din & r_adl);
                        elsif d_spec = SP_JMPI then
                            r_adh <= v_din;
                            r_addr <= v_din & r_adl;
                            r_we <= '0';
                            r_st <= S_IND1;
                        elsif d_md = MD_ABS then
                            r_adh <= v_din;
                            go_data(v_din & r_adl);
                        else
                            if d_md = MD_ABY then
                                v_sum := ('0' & r_adl) + ('0' & r_y);
                            else
                                v_sum := ('0' & r_adl) + ('0' & r_x);
                            end if;
                            r_adl <= v_sum(7 downto 0);
                            r_adh <= v_din;
                            r_cross <= v_sum(8);
                            if d_acc = AC_R and v_sum(8) = '0' then
                                go_data(v_din & v_sum(7 downto 0));
                            else
                                r_addr <= v_din & v_sum(7 downto 0);   -- dummy read
                                r_we <= '0';
                                r_st <= S_ABFIX;
                            end if;
                        end if;

                    -- ---------------- page fix-up (indexed abs / (zp),y) ----------------
                    when S_ABFIX =>
                        v_r := r_adh + ("0000000" & r_cross);
                        r_adh <= v_r;
                        go_data(v_r & r_adl);

                    -- ---------------- (zp,x) ----------------
                    when S_IZX1 =>
                        v_r := r_ba + r_x;
                        r_ba <= v_r;
                        r_addr <= x"00" & v_r;
                        r_we <= '0';
                        r_st <= S_IZX2;
                    when S_IZX2 =>
                        r_adl <= v_din;
                        r_addr <= x"00" & (r_ba + 1);
                        r_we <= '0';
                        r_st <= S_IZX3;
                    when S_IZX3 =>
                        r_adh <= v_din;
                        go_data(v_din & r_adl);

                    -- ---------------- (zp),y ----------------
                    when S_IZY1 =>
                        r_adl <= v_din;
                        r_addr <= x"00" & (r_ba + 1);
                        r_we <= '0';
                        r_st <= S_IZY2;
                    when S_IZY2 =>
                        v_sum := ('0' & r_adl) + ('0' & r_y);
                        r_adl <= v_sum(7 downto 0);
                        r_adh <= v_din;
                        r_cross <= v_sum(8);
                        if d_acc = AC_R and v_sum(8) = '0' then
                            go_data(v_din & v_sum(7 downto 0));
                        else
                            r_addr <= v_din & v_sum(7 downto 0);
                            r_we <= '0';
                            r_st <= S_ABFIX;
                        end if;

                    -- ---------------- the data cycle ----------------
                    when S_DATA =>
                        if d_acc = AC_M then
                            r_dl <= v_din;
                            r_addr <= r_adh & r_adl;
                            r_we <= '1';
                            r_dout <= din;                          -- dummy write of the old value
                            r_st <= S_RMW1;
                        else
                            if d_acc = AC_R and unsigned(d_aop) /= OP_NONE then
                                alu(unsigned(d_aop), v_din);
                            end if;
                            go_fetch(r_pc);
                        end if;
                    when S_RMW1 =>
                        v_c := p_c;
                        case d_rop is
                            when "000" => v_r := r_dl(6 downto 0) & '0';  v_c := r_dl(7);   -- ASL
                            when "001" => v_r := r_dl(6 downto 0) & p_c;  v_c := r_dl(7);   -- ROL
                            when "010" => v_r := '0' & r_dl(7 downto 1);  v_c := r_dl(0);   -- LSR
                            when "011" => v_r := p_c & r_dl(7 downto 1);  v_c := r_dl(0);   -- ROR
                            when "100" => v_r := r_dl + 1;                                  -- INC
                            when others => v_r := r_dl - 1;                                 -- DEC
                        end case;
                        p_c <= v_c;
                        p_n <= v_r(7);
                        if v_r = 0 then p_z <= '1'; else p_z <= '0'; end if;
                        r_dl <= v_r;
                        r_addr <= r_adh & r_adl;
                        r_we <= '1';
                        r_dout <= std_logic_vector(v_r);
                        r_st <= S_RMW2;
                    when S_RMW2 =>
                        if unsigned(d_aop) /= OP_NONE then
                            alu(unsigned(d_aop), r_dl);             -- SLO RLA SRE RRA DCP ISB
                        end if;
                        go_fetch(r_pc);

                    -- ---------------- branches ----------------
                    when S_BR1 =>
                        v_sum := ('0' & r_pc(7 downto 0)) + ('0' & r_dl);
                        r_pc(7 downto 0) <= v_sum(7 downto 0);
                        -- page crossed when the carry disagrees with the offset sign
                        if (v_sum(8) xor r_dl(7)) = '1' then
                            r_addr <= r_pc(15 downto 8) & v_sum(7 downto 0);   -- dummy read
                            r_we <= '0';
                            r_st <= S_BR2;
                        else
                            go_fetch(r_pc(15 downto 8) & v_sum(7 downto 0));
                        end if;
                    when S_BR2 =>
                        if r_dl(7) = '1' then
                            v_r := r_pc(15 downto 8) - 1;
                        else
                            v_r := r_pc(15 downto 8) + 1;
                        end if;
                        r_pc(15 downto 8) <= v_r;
                        go_fetch(v_r & r_pc(7 downto 0));

                    -- ---------------- JMP (ind) ----------------
                    when S_IND1 =>
                        r_dl <= v_din;
                        r_addr <= r_adh & (r_adl + 1);              -- page-wrap bug
                        r_we <= '0';
                        r_st <= S_IND2;
                    when S_IND2 =>
                        r_pc <= v_din & r_dl;
                        go_fetch(v_din & r_dl);

                    -- ---------------- JSR ----------------
                    when S_JSR1 =>
                        r_addr <= x"01" & r_s;
                        r_we <= '1';
                        r_dout <= std_logic_vector(r_pc(15 downto 8));
                        r_st <= S_JSR2;
                    when S_JSR2 =>
                        r_s <= r_s - 1;
                        r_addr <= x"01" & (r_s - 1);
                        r_we <= '1';
                        r_dout <= std_logic_vector(r_pc(7 downto 0));
                        r_st <= S_JSR3;
                    when S_JSR3 =>
                        r_s <= r_s - 1;
                        r_addr <= r_pc;
                        r_we <= '0';
                        r_st <= S_JSR4;
                    when S_JSR4 =>
                        r_pc <= v_din & r_adl;
                        go_fetch(v_din & r_adl);

                    -- ---------------- RTS ----------------
                    when S_RTS1 =>
                        r_s <= r_s + 1;
                        r_addr <= x"01" & (r_s + 1);
                        r_we <= '0';
                        r_st <= S_RTS2;
                    when S_RTS2 =>
                        r_pc(7 downto 0) <= v_din;
                        r_s <= r_s + 1;
                        r_addr <= x"01" & (r_s + 1);
                        r_we <= '0';
                        r_st <= S_RTS3;
                    when S_RTS3 =>
                        r_pc(15 downto 8) <= v_din;
                        r_addr <= v_din & r_pc(7 downto 0);         -- dummy read
                        r_we <= '0';
                        r_st <= S_RTS4;
                    when S_RTS4 =>
                        r_pc <= r_pc + 1;
                        go_fetch(r_pc + 1);

                    -- ---------------- RTI ----------------
                    when S_RTI1 =>
                        r_s <= r_s + 1;
                        r_addr <= x"01" & (r_s + 1);
                        r_we <= '0';
                        r_st <= S_RTI2;
                    when S_RTI2 =>
                        p_n <= v_din(7); p_v <= v_din(6); p_d <= v_din(3);
                        p_i <= v_din(2); p_z <= v_din(1); p_c <= v_din(0);
                        r_s <= r_s + 1;
                        r_addr <= x"01" & (r_s + 1);
                        r_we <= '0';
                        r_st <= S_RTI3;
                    when S_RTI3 =>
                        r_pc(7 downto 0) <= v_din;
                        r_s <= r_s + 1;
                        r_addr <= x"01" & (r_s + 1);
                        r_we <= '0';
                        r_st <= S_RTI4;
                    when S_RTI4 =>
                        r_pc(15 downto 8) <= v_din;
                        go_fetch(v_din & r_pc(7 downto 0));

                    -- ---------------- BRK ----------------
                    when S_BRK1 =>
                        r_s <= r_s - 1;
                        r_addr <= x"01" & (r_s - 1);
                        r_we <= '1';
                        r_dout <= std_logic_vector(r_pc(7 downto 0));
                        r_st <= S_BRK2;
                    when S_BRK2 =>
                        r_s <= r_s - 1;
                        r_addr <= x"01" & (r_s - 1);
                        r_we <= '1';
                        r_dout <= f_pbyte(p_n, p_v, p_d, p_i, p_z, p_c, '1');
                        r_st <= S_BRK3;
                    when S_BRK3 =>
                        r_s <= r_s - 1;
                        p_i <= '1';
                        r_addr <= x"FFFE";
                        r_we <= '0';
                        r_st <= S_BRK4;
                    when S_BRK4 =>
                        r_pc(7 downto 0) <= v_din;
                        r_addr <= x"FFFF";
                        r_we <= '0';
                        r_st <= S_BRK5;
                    when S_BRK5 =>
                        r_pc(15 downto 8) <= v_din;
                        go_fetch(v_din & r_pc(7 downto 0));

                    -- ---------------- PHA / PHP ----------------
                    when S_PSH1 =>
                        r_s <= r_s - 1;
                        go_fetch(r_pc);

                    -- ---------------- PLA / PLP ----------------
                    when S_PUL1 =>
                        r_s <= r_s + 1;
                        r_addr <= x"01" & (r_s + 1);
                        r_we <= '0';
                        r_st <= S_PUL2;
                    when S_PUL2 =>
                        if d_spec = SP_PLA then
                            alu(OP_LD, v_din);
                        else
                            p_n <= v_din(7); p_v <= v_din(6); p_d <= v_din(3);
                            p_i <= v_din(2); p_z <= v_din(1); p_c <= v_din(0);
                        end if;
                        go_fetch(r_pc);

                    -- ---------------- reset ----------------
                    when S_RST0 =>
                        r_addr <= x"FFFC";
                        r_we <= '0';
                        r_st <= S_RST1;
                    when S_RST1 =>
                        r_pc(7 downto 0) <= v_din;
                        r_addr <= x"FFFD";
                        r_we <= '0';
                        r_st <= S_RST2;
                    when others =>                                   -- S_RST2
                        r_pc(15 downto 8) <= v_din;
                        r_s <= x"FD";
                        p_i <= '1';
                        go_fetch(v_din & r_pc(7 downto 0));
                end case;
            end if;

            if rst = '1' then
                r_st <= S_RST0;
                r_addr <= x"FFFC";
                r_we <= '0';
                q_go <= '0';
                r_dload <= '0';
            end if;
        end if;
    end process p_cpu;
end architecture rtl;
