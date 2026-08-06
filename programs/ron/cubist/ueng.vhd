    ------------------------------------------------------------------------
    -- MICROCODED GEOMETRY ENGINE
    --
    -- Replaces ~110 hand-written frame-setup states.  Each of those states
    -- carried its own adders, comparators and address arithmetic, and each
    -- one that touched the multiplier was another input to an 18-bit operand
    -- mux; together that measured 2,666 cells for math that runs once per
    -- frame in blanking with ~99,000 spare clocks.
    --
    -- Here there is ONE ALU, the scratch RAM is the register file (two
    -- copies so an instruction can read both operands in a cycle), and the
    -- program lives in a block RAM.  Three cycles per instruction, which at
    -- ~1,500 dynamic instructions is ~4,500 clocks of the vblank budget.
    ------------------------------------------------------------------------
    p_ueng : process(clk)
        variable v_a, v_b : signed(31 downto 0);
        variable v_r      : signed(31 downto 0);
        variable v_op     : integer range 0 to 31;
        variable v_imm    : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            u_we   <= '0';
            g_we   <= '0';
            mu_go_f <= '0';
            frd_start <= '0';

            if s_fstart = '1' then
                u_pc   <= (others => '0');
                u_st   <= "00";
                u_done <= '0';
            elsif u_done = '0' then
                case to_integer(u_st) is

                    -- FETCH: the ROM answers next cycle
                    when 0 =>
                        u_st <= "01";

                    -- DECODE: latch the instruction, issue both register reads
                    when 1 =>
                        u_ir <= u_rom;
                        u_ra <= unsigned(u_rom(27 downto 21));
                        u_rb <= unsigned(u_rom(20 downto 14));
                        u_st <= "10";

                    -- EXECUTE
                    when others =>
                        v_op  := to_integer(unsigned(u_ir(39 downto 35)));
                        v_imm := signed(u_ir(13 downto 0));
                        v_a   := resize(signed(u_rda), 32);
                        v_b   := resize(signed(u_rdb), 32);
                        v_r   := (others => '0');
                        u_st  <= "00";
                        u_pc  <= u_pc + 1;

                        case v_op is
                            when 1  => v_r := v_a;
                            when 2  => v_r := v_a + v_b;
                            when 3  => v_r := v_a - v_b;
                            when 4  => v_r := shift_right(v_a, to_integer(v_imm(3 downto 0)));
                            when 5  => v_r := shift_left(v_a, to_integer(v_imm(3 downto 0)));
                            when 6  => v_r := -v_a;
                            when 7  => if v_a < 0 then v_r := -v_a; else v_r := v_a; end if;
                            when 8  => if v_a < v_b then v_r := v_a; else v_r := v_b; end if;
                            when 9  => if v_a > v_b then v_r := v_a; else v_r := v_b; end if;
                            when 10 =>                     -- MUL: start
                                fm_a <= v_a;
                                fm_b <= resize(v_b, 14);
                                mu_go_f <= '1';
                            when 11 =>                     -- MRD: stall, then take
                                if mu_idle = '0' then
                                    u_st <= "10";
                                    u_pc <= u_pc;
                                else
                                    v_r := resize(shift_right(mu_p,
                                              to_integer(v_imm(4 downto 0))), 32);
                                end if;
                            when 12 =>                     -- DIV: start
                                frd_ns <= shift_left(resize(unsigned(abs(v_a)), 32),
                                                     to_integer(v_imm(4 downto 0)));
                                if v_b = 0 then frd_ds <= to_unsigned(1, 21);
                                else frd_ds <= resize(unsigned(abs(v_b)), 21); end if;
                                fd_sgn <= (v_a(17) xor v_b(17));
                                frd_start <= '1';
                            when 13 =>                     -- DRD: stall, then take
                                if dv_bsy = '1' then
                                    u_st <= "10";
                                    u_pc <= u_pc;
                                elsif fd_sgn = '1' then
                                    v_r := -resize(signed(dv_q(15 downto 0)), 32);
                                else
                                    v_r := resize(signed(dv_q(15 downto 0)), 32);
                                end if;
                            when 14 => v_r := resize(C_SIN(to_integer(
                                              unsigned(v_a(7 downto 0)))), 32);
                            when 15 => v_r := resize(v_imm, 32);
                            when 16 =>                     -- GWR
                                g_wa <= resize(unsigned(v_imm(7 downto 0))
                                               + unsigned(v_a(7 downto 0)), 8);
                                g_wd <= std_logic_vector(resize(v_b, 16));
                                g_we <= '1';
                            when 17 =>                     -- SLW
                                u_slp <= unsigned(v_imm(3 downto 0));
                                u_slv <= resize(v_a, 32);
                                u_slw <= '1';
                            when 18 => u_pc <= unsigned(v_imm(8 downto 0));
                            when 19 => if v_a /= 0 then
                                           u_pc <= unsigned(v_imm(8 downto 0));
                                       end if;
                            when 20 => if v_a < v_b then
                                           u_pc <= unsigned(v_imm(8 downto 0));
                                       end if;
                            when 21 => if v_a >= v_b then
                                           u_pc <= unsigned(v_imm(8 downto 0));
                                       end if;
                            when 22 =>
                                if v_a < 0 then v_r := (others => '0');
                                elsif v_a > v_b then v_r := v_b;
                                else v_r := v_a; end if;
                            when 24 => u_done <= '1';
                            when others => null;
                        end case;

                        -- one write port, one write per instruction
                        if v_op /= 0 and v_op < 24 and v_op /= 10 and v_op /= 12
                           and v_op /= 16 and v_op /= 17 and v_op /= 18
                           and v_op /= 19 and v_op /= 20 and v_op /= 21 then
                            u_wa <= unsigned(u_ir(34 downto 28));
                            u_wd <= std_logic_vector(resize(v_r, 32));
                            u_we <= '1';
                        end if;
                end case;
            end if;
        end if;
    end process p_ueng;

    ------------------------------------------------------------------------
    -- microcode ROM and the two register-file copies (dual read)
    ------------------------------------------------------------------------
    p_urom : process(clk)
    begin
        if rising_edge(clk) then
            u_rom <= C_UCODE(to_integer(u_pc));
        end if;
    end process p_urom;

    p_rfa : process(clk)
    begin
        if rising_edge(clk) then
            if u_we = '1' then rf_a(to_integer(u_wa)) <= u_wd; end if;
            u_rda <= rf_a(to_integer(u_ra));
        end if;
    end process p_rfa;

    p_rfb : process(clk)
    begin
        if rising_edge(clk) then
            if u_we = '1' then rf_b(to_integer(u_wa)) <= u_wd; end if;
            u_rdb <= rf_b(to_integer(u_rb));
        end if;
    end process p_rfb;
