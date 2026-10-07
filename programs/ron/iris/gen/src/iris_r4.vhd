    ------------------------------------------------------------------------
    -- table write ports (one write statement per RAM)
    ------------------------------------------------------------------------
    p_twrite : process(clk)
    begin
        if rising_edge(clk) then
            if tw_we = 1 then t1_ram(to_integer(tw_addr(7 downto 0))) <= tw_d16; end if;
            if tw_we = 2 then t2_ram(to_integer(tw_addr(7 downto 0))) <= tw_d16; end if;
            if tw_we = 3 then tm_ram(to_integer(tw_addr(7 downto 0))) <= tw_d16; end if;
            if tw_we = 4 then tv_ram(to_integer(tw_addr(7 downto 0))) <= tw_d16; end if;
            if tw_we = 5 then ts_ram(to_integer(tw_addr(7 downto 0))) <= tw_d16; end if;
            if tw_we = 6 then tk_ram(to_integer(tw_addr(7 downto 0))) <= tw_d16; end if;
        end if;
    end process p_twrite;

    busy <= m_busy or d_busy or m_go or d_go;

    ------------------------------------------------------------------------
    -- micro-sequencer: accumulator machine, 3 cycles per instruction
    -- (fetch / register read / execute), gated by blank_ok.  Multi-cycle
    -- instructions (shifts, serial units, probe, frame wait) hold in the
    -- execute phase.  Every load goes through the one adder.
    ------------------------------------------------------------------------
    p_seq : process(clk)
        variable v_op   : integer range 0 to 31;
        variable v_reg  : unsigned(6 downto 0);
        variable v_imm  : signed(10 downto 0);
        variable v_sel  : integer range 0 to 31;
        variable v_rq   : signed(31 downto 0);
        variable v_ms   : signed(32 downto 0);
        variable v_am   : signed(32 downto 0);
        variable v_opa, v_opb : signed(31 downto 0);
        variable v_sub  : std_logic;
        variable v_lx   : unsigned(19 downto 0);
        variable v_rem  : unsigned(16 downto 0);
        variable v_done : std_logic;
        variable v_jump : std_logic;
    begin
        if rising_edge(clk) then
            -- The serial units and the register-file write port run FREE, outside the
            -- blanking gate: they are self-contained, their results are only consumed by
            -- the gated CPU, and keeping them off that gate's fanout both shortens its
            -- routing and lets a multiply or divide finish during active video.
            m_go <= '0'; d_go <= '0'; stx_en <= '0';
            tw_we <= (others => '0');
            rf_we <= '0'; row0_req <= '0';
            if in_vblank = '1' and in_vblank_q = '0' then seq_req <= '1'; end if;
            if rf_we = '1' then
                rf_mem(to_integer(rf_wa)) <= std_logic_vector(acc);
            end if;

            -- serial multiplier (signed 32 x signed 20, 20 steps; the last step subtracts)
            if m_go = '1' then
                m_cnt  <= (others => '0');
                m_busy <= '1';
            elsif m_busy = '1' then
                v_ms := resize(signed(m_acc(51 downto 20)), 33);
                v_am := resize(signed(m_am), 33);
                if m_acc(0) = '1' then
                    if m_cnt = 19 then v_ms := v_ms - v_am; else v_ms := v_ms + v_am; end if;
                end if;
                m_acc <= unsigned(v_ms(32 downto 0)) & m_acc(19 downto 1);
                m_cnt <= m_cnt + 1;
                if m_cnt = 19 then
                    m_busy <= '0';
                end if;
            end if;

            -- serial divider (32 / 16 -> 32, restoring)
            if d_go = '1' then
                d_rem  <= (others => '0');
                d_cnt  <= (others => '0');
                d_busy <= '1';
            elsif d_busy = '1' then
                -- the remainder is always below the 16-bit divisor, so 17 bits suffice
                v_rem := d_rem(15 downto 0) & d_n(31);
                if v_rem >= resize(d_d, 17) then
                    v_rem := v_rem - resize(d_d, 17);
                    d_n   <= d_n(30 downto 0) & '1';
                else
                    d_n   <= d_n(30 downto 0) & '0';
                end if;
                d_rem <= v_rem;
                d_cnt <= d_cnt + 1;
                if d_cnt = 31 then d_busy <= '0'; end if;
            end if;


            if blank_ok = '1' then
                -- CPU.  The instruction register is absorbed into the program EBR's
                -- output register, so everything downstream decodes from a SECOND,
                -- fabric-registered copy (d_op / d_arg) captured during the register-read
                -- state; only the register-file address is taken straight off the EBR.
                v_op  := to_integer(d_op);
                v_reg := unsigned(d_arg(6 downto 0));
                v_imm := signed(d_arg);
                v_sel := to_integer(unsigned(d_arg(4 downto 0)));
                v_rq  := signed(rf_q);
                v_done := '1';
                v_jump := '0';


                v_sub := '0';
                v_opa := (others => '0');
                case v_op is
                    when OP_ADD  => v_opa := acc; v_opb := v_rq;
                    when OP_ADDI => v_opa := acc; v_opb := resize(v_imm, 32);
                    when OP_SUB  => v_opa := acc; v_opb := v_rq; v_sub := '1';
                    when OP_NEG | OP_ABS => v_opb := acc; v_sub := '1';
                    when OP_LDI  => v_opb := resize(v_imm, 32);
                    when OP_LDP | OP_LDQ | OP_LDF | OP_LDX => v_opb := sp_r;
                    when others  => v_opb := v_rq;
                end case;
                if v_sub = '1' then v_opb := not v_opb; end if;

                -- ALU writeback (operands registered at execute; the add is its own clock)
                if alu_pend = '1' then
                    alu_pend <= '0';
                    if alu_and = '1' then acc <= alu_a and alu_b;
                    else acc <= alu_a + alu_b + ("0000000000000000000000000000000" & alu_c); end if;
                end if;

                case to_integer(cs_st) is
                when 0 =>
                    ir <= prog_mem(to_integer(pc));
                    fun_q <= rom_fun(to_integer(fun_addr));
                    cs_st <= to_unsigned(1, 2);
                when 1 =>
                    -- direct slices of the EBR output only: no logic on this hop
                    rf_q  <= rf_mem(to_integer(unsigned(ir(6 downto 0))));
                    d_op  <= unsigned(ir(15 downto 11));
                    d_arg <= ir(10 downto 0);
                    cs_st <= to_unsigned(2, 2);
                    sh_busy <= '0';
                when 2 =>
                    -- the special operand sources get a clock of their own, decoded from
                    -- the fabric copy of the instruction rather than from the ROM output
                    case v_op is
                        when OP_LDQ => sp_r <= signed(d_n);
                        when OP_LDF => sp_r <= signed(resize(unsigned(fun_q), 32));
                        when OP_LDP =>
                            if v_sel = 0 then sp_r <= signed(m_acc(31 downto 0));
                            else              sp_r <= signed(m_acc(47 downto 16)); end if;
                        when others =>
                            case v_sel is
                                when 0 to 5 => v_lx := resize(reg_q(to_integer(unsigned(d_arg(2 downto 0)))), 20);
                                when 6  => v_lx := resize(reg_q(7), 20);
                                when 7  => v_lx := resize(act_w, 20);
                                when 8  => v_lx := resize(act_h, 20);
                                when 9  => v_lx := resize(vstep_base, 20);
                                when 10 => v_lx := (0 => sw_flow, 1 => sw_full, 2 => ilace_q, 3 => fld_q, 4 => sw_mesh, others => '0');
                                when others => v_lx := (others => '0');
                            end case;
                            sp_r <= signed(resize(v_lx, 32));
                    end case;
                    cs_st <= to_unsigned(3, 2);
                when others =>
                    case v_op is
                    when OP_LD | OP_LDI | OP_ADD | OP_ADDI | OP_SUB | OP_NEG | OP_LDP | OP_LDQ | OP_LDF | OP_LDX =>
                        alu_a <= v_opa; alu_b <= v_opb; alu_c <= v_sub; alu_and <= '0'; alu_pend <= '1';
                    when OP_ABS  =>
                        alu_a <= v_opa; alu_b <= v_opb; alu_c <= v_sub; alu_and <= '0';
                        if acc < 0 then alu_pend <= '1'; end if;
                    when OP_ST   => rf_we <= '1'; rf_wa <= v_reg;
                    when OP_AND  => alu_a <= acc; alu_b <= v_rq; alu_and <= '1'; alu_pend <= '1';
                    when OP_SHR | OP_SHL | OP_SHRV | OP_SHLV =>
                        if sh_busy = '0' then
                            if v_op = OP_SHR or v_op = OP_SHL then sh_cnt <= unsigned(v_imm(5 downto 0));
                            else sh_cnt <= unsigned(v_rq(5 downto 0)); end if;
                            sh_busy <= '1';
                            v_done  := '0';
                        elsif sh_cnt /= 0 then
                            if v_op = OP_SHR or v_op = OP_SHRV then acc <= shift_right(acc, 1);
                            else                                     acc <= shift_left(acc, 1); end if;
                            sh_cnt <= sh_cnt - 1;
                            v_done := '0';
                        end if;
                    when OP_MUL =>
                        if sh_busy = '0' then
                            m_am  <= unsigned(acc);
                            m_acc <= x"00000000" & unsigned(v_rq(19 downto 0));
                            m_go <= '1';
                            sh_busy <= '1'; v_done := '0';
                        elsif busy = '1' then
                            v_done := '0';
                        end if;
                    when OP_DIV =>
                        if sh_busy = '0' then
                            d_n <= unsigned(acc); d_d <= unsigned(v_rq(15 downto 0)); d_go <= '1';
                            sh_busy <= '1'; v_done := '0';
                        elsif busy = '1' then
                            v_done := '0';
                        end if;
                    when OP_LINE0 => row0_req <= '1';
                    when OP_WAITF =>
                        if seq_req = '1' then seq_req <= '0'; else v_done := '0'; end if;
                    when OP_JMP  => v_jump := '1';
                    when OP_JZ   => if acc = 0 then v_jump := '1'; end if;
                    when OP_JNZ  => if acc /= 0 then v_jump := '1'; end if;
                    when OP_JN   => if acc < 0 then v_jump := '1'; end if;
                    when OP_JP   => if acc > 0 then v_jump := '1'; end if;
                    when OP_CALL => v_jump := '1'; lnk <= pc + 1; lnk2 <= lnk;
                    when OP_RET  => lnk <= lnk2;
                    when OP_ROM  => fun_addr <= unsigned(acc(8 downto 0));
                    when OP_TW   => tw_we <= to_unsigned(v_sel, 3);
                    when OP_STX =>
                        stx_val <= acc(20 downto 0); stx_sel <= to_unsigned(v_sel, 5); stx_en <= '1';
                        case v_sel is
                            when 10 => c_lp <= acc(17 downto 0);
                            when 11 => c_cdot <= acc(19 downto 0);
                            when 12 => c_up <= acc(19 downto 0);
                            when 13 => c_invn <= unsigned(acc(10 downto 0));
                            when 14 => c_inve <= unsigned(acc(2 downto 0));
                            when 15 => c_n <= acc(7 downto 0); c_nm1 <= acc(7 downto 0) - 1;
                            when 16 => c_t <= acc(10 downto 0);
                            when 17 => c_ph <= unsigned(acc(11 downto 0));
                            when 18 => c_fade <= unsigned(acc(7 downto 0));
                            when 24 => tw_addr <= unsigned(acc(8 downto 0));
                            when 25 => tw_d16 <= std_logic_vector(acc(15 downto 0));
                            when 26 => c_fl <= unsigned(acc(3 downto 0));
                            when 19 => c_a <= unsigned(acc(3 downto 0));
                            when others => null;
                        end case;
                    when others => null;
                    end case;

                    if v_done = '1' then
                        if v_op = OP_RET then     pc <= lnk;
                        elsif v_jump = '1' then  pc <= unsigned(v_imm(10 downto 0));
                        else                     pc <= pc + 1; end if;
                        cs_st   <= (others => '0');
                        sh_busy <= '0';
                    end if;
                end case;
            end if;
        end if;
    end process p_seq;

end architecture iris;
