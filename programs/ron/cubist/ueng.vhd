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
        variable v_q      : unsigned(16 downto 0);
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

                    -- INDEXED second access (LDX/STX only)
                    when 3 =>
                        u_st <= "00";
                        u_pc <= u_pc + 1;
                        if to_integer(unsigned(u_ir(39 downto 35))) = 30 then
                            u_wa <= unsigned(u_ir(34 downto 28));
                            u_wd <= u_rda;
                            u_we <= '1';
                        else
                            u_wa <= u_xa;
                            u_wd <= std_logic_vector(resize(u_xd, 32));
                            u_we <= '1';
                        end if;

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
                                fm_a <= resize(v_a, 18);
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
                                fd_sgn <= (v_a(31) xor v_b(31));
                                frd_start <= '1';
                            when 13 =>                     -- DRD: stall, then take
                                -- the quotient is a MAGNITUDE up to 65535, so it
                                -- has to be zero-extended before the sign goes on
                                if dv_bsy = '1' then
                                    u_st <= "10";
                                    u_pc <= u_pc;
                                else
                                    if dv_q > 65535 then
                                        v_q := to_unsigned(65535, 17);
                                    else
                                        v_q := resize(dv_q(15 downto 0), 17);
                                    end if;
                                    if fd_sgn = '1' then
                                        v_r := -resize(signed('0' & v_q), 32);
                                    else
                                        v_r := resize(signed('0' & v_q), 32);
                                    end if;
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
                                u_slp <= unsigned(v_imm(4 downto 0));
                                u_slv <= resize(v_a, 32);
                                u_sls <= unsigned(v_b(1 downto 0));
                                u_slw <= '1';
                            when 18 => u_pc <= unsigned(v_imm(9 downto 0));
                            when 19 => if v_a /= 0 then
                                           u_pc <= unsigned(v_imm(9 downto 0));
                                       end if;
                            when 20 => if v_a < v_b then
                                           u_pc <= unsigned(v_imm(9 downto 0));
                                       end if;
                            when 21 => if v_a >= v_b then
                                           u_pc <= unsigned(v_imm(9 downto 0));
                                       end if;
                            when 22 =>
                                if v_a < 0 then v_r := (others => '0');
                                elsif v_a > v_b then v_r := v_b;
                                else v_r := v_a; end if;
                            when 23 => v_r := resize(signed('0' & C_GAM(
                                              to_integer(unsigned(v_a(7 downto 0))))), 32);
                            when 25 => v_r := u_ctlv;      -- knobs and raster
                            when 27 => v_r := v_a and resize(signed('0' & v_imm), 32);
                            when 28 => v_r := resize(shift_right(v_a,
                                              to_integer(v_imm(3 downto 0)))
                                              and to_signed(1, 32), 32);
                            when 29 => v_r := v_a or shift_left(to_signed(1, 32),
                                              to_integer(v_imm(3 downto 0)));
                            when 30 | 31 =>                -- LDX / STX
                                u_xa <= resize(unsigned(v_a(6 downto 0))
                                               + unsigned(v_imm(6 downto 0)), 7);
                                u_xd <= resize(v_b, 32);
                                u_ra <= resize(unsigned(v_a(6 downto 0))
                                               + unsigned(v_imm(6 downto 0)), 7);
                                u_st <= "11";
                                u_pc <= u_pc;
                            when 24 => u_done <= '1';
                            when others => null;
                        end case;

                        -- one write port, one write per instruction: every op
                        -- that produced a value in v_r above, and only those
                        if (v_op >= 1 and v_op <= 9) or v_op = 11 or v_op = 13
                           or v_op = 14 or v_op = 15 or v_op = 22 or v_op = 23
                           or v_op = 25 or (v_op >= 27 and v_op <= 29) then
                          -- ...but not on the cycle a read op is still stalling
                          if not ((v_op = 11 and mu_idle = '0')
                                  or (v_op = 13 and dv_bsy = '1')) then
                            u_wa <= unsigned(u_ir(34 downto 28));
                            u_wd <= std_logic_vector(resize(v_r, 32));
                            u_we <= '1';
                          end if;
                        end if;
                end case;
            end if;
        end if;
    end process p_ueng;

    ------------------------------------------------------------------------
    -- CTL reads the outside world.  u_ir is latched at DECODE, so selecting
    -- on its immediate field is stable through EXECUTE.
    ------------------------------------------------------------------------
    with to_integer(unsigned(u_ir(3 downto 0))) select u_ctlv <=
        resize(signed('0' & s_k1),   32) when 0,
        resize(signed('0' & s_k2),   32) when 1,
        resize(signed('0' & s_k3),   32) when 2,
        resize(signed('0' & s_W),    32) when 3,
        resize(signed('0' & s_H),    32) when 4,
        resize(signed('0' & s_hf),   32) when 5,
        resize(signed('0' & s_cx),   32) when 6,
        resize(signed('0' & s_cy),   32) when 7,
        (0 => s_ilace, others => '0')    when 8,
        (0 => s_zfirst, others => '0')   when 10,
        (others => '0')                  when others;

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
