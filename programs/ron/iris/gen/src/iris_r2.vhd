    p_relay : process(clk)
    begin
        if rising_edge(clk) then
            for i in 0 to 7 loop reg_q(i) <= unsigned(registers_in(i)); end loop;
            ilace_q <= ilace; fld_q <= fl_r;
            sw_ink  <= registers_in(6)(2);
            sw_flow <= registers_in(6)(3);
            sw_full <= registers_in(6)(4);
            sw_mesh <= registers_in(6)(0);
            sw_glow <= registers_in(6)(1);
        end if;
    end process p_relay;

    ------------------------------------------------------------------------
    -- raster measurement, blank budgets, frame anchor
    ------------------------------------------------------------------------
    p_time : process(clk)
    begin
        if rising_edge(clk) then
            av_r  <= data_in.avid;
            av_q  <= av_r;
            vs_r  <= data_in.vsync_n;
            vs_q  <= vs_r;
            fl_r  <= data_in.field_n;
            hs_r  <= data_in.hsync_n;
            hs_q  <= hs_r;
            dcnt  <= dcnt + 1;

            if av_r = '1' then
                xcnt <= xcnt + 1;
            elsif av_q = '1' then
                if xcnt > 16 then act_w <= xcnt; end if;
                xcnt  <= (others => '0');
                aline <= aline + 1;
            end if;

            -- blank budgets: hblank length (avid fall -> rise) and the hsync period;
            -- vblank = two hsyncs without active video
            if hs_r = '0' and hs_q = '1' then
                hcnt <= (others => '0');
                if hcnt > 100 then hline <= hcnt; end if;
                if nb /= 3 then nb <= nb + 1; end if;
            elsif hcnt /= 4095 then
                hcnt <= hcnt + 1;
            end if;
            if av_r = '1' then
                blank_cnt <= (others => '0');
                nb <= (others => '0');
                if av_q = '0' and blank_cnt /= 1023 then hbl_len <= blank_cnt; end if;
            elsif blank_cnt /= 1023 then
                blank_cnt <= blank_cnt + 1;
            end if;
            if nb >= 2 then in_vblank <= '1'; else in_vblank <= '0'; end if;
            in_vblank_q <= in_vblank;

            blank_ok <= '0';
            if av_r = '0' and blank_cnt >= 2 then
                if in_vblank = '1' then
                    if hcnt + 64 < hline then blank_ok <= '1'; end if;
                else
                    if blank_cnt + C_BLANK_MARGIN < hbl_len then blank_ok <= '1'; end if;
                end if;
            end if;

            if vs_r = '0' and vs_q = '1' then
                frame_act <= '0';
            elsif av_r = '1' and av_q = '0' and frame_act = '0' then
                frame_act <= '1';
                if aline > 50 then act_h <= aline; end if;
                aline <= (others => '0');
                if fl_r /= anc_fld then ilace <= '1'; else ilace <= '0'; end if;
                anc_fld <= fl_r;
            end if;

            if ilace = '1' then
                if act_h >= 400 then    vstep_base <= to_unsigned(16, 6);
                elsif act_h >= 270 then vstep_base <= to_unsigned(15, 6);
                else                    vstep_base <= to_unsigned(18, 6); end if;
            else
                if act_h >= 700 then    vstep_base <= to_unsigned(16, 6);
                elsif act_h >= 540 then vstep_base <= to_unsigned(15, 6);
                else                    vstep_base <= to_unsigned(18, 6); end if;
            end if;
        end if;
    end process p_time;

    ------------------------------------------------------------------------
    -- Geometry engine: one sample every 8 clocks, 8 px apart, one line ahead.
    -- Lane a is loaded at ph0, lane b at ph4, into a 3-stage CORDIC pipeline
    -- (one iteration per clock per stage, 12 clocks per lane): lane a of
    -- sample k is delivered at ph4 of k+1, lane b at ph0 of k+2.  The log
    -- lane (6 stages) is loaded with lane b at ph1 and lane a at ph5; the
    -- tail is phase-scheduled and a sample's word is written at ph7 three
    -- periods after it was loaded.
    --
    -- Units: the log table is pre-scaled by the sequencer to ring units
    -- (normalised: rings per octave = invn << inve), so a lane value is
    -- L_n = E*invn + M[hi] + lo*slope and u = (La_n - Lb_n - lp_n) << inve.
    -- theta*A is the sum of two table halves of the 14-bit angle; the
    -- split is a table of u.  The AA shift uses exponents + top mantissa
    -- bits as log2 (error < 0.09 octave, below the shift quantum).
    ------------------------------------------------------------------------
    p_eng : process(clk)
        variable vx, vy : signed(C_CW - 1 downto 0);
        variable vz     : signed(15 downto 0);
        variable v0x, v0y, v1x, v1y, v2x, v2y : signed(C_CW - 1 downto 0);
        variable v0z, v1z, v2z : signed(15 downto 0);
        variable ax, ay : signed(15 downto 0);
        variable hi, lo : unsigned(14 downto 0);
        variable sw     : std_logic;
        variable lx, ly : signed(15 downto 0);
        variable v_m    : signed(17 downto 0);
        variable v_fold : unsigned(16 downto 0);
        variable v_th   : unsigned(16 downto 0);
        variable v_say, v_sblx, v_sbly : signed(19 downto 0);
        variable v_n    : unsigned(C_CW - 1 downto 0);
        variable v_u    : signed(21 downto 0);
        variable v_sh   : signed(9 downto 0);
        variable v_dl   : signed(26 downto 0);
        variable v_xc   : unsigned(C_CW - 1 downto 0);
        variable v_rz   : signed(15 downto 0);
        variable v_rsw, v_rsx, v_rsy : std_logic;
        variable v_rx   : unsigned(C_CW - 1 downto 0);
        variable v_sh0  : unsigned(4 downto 0);
        variable v_es   : unsigned(5 downto 0);
        variable v_hs   : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then

            -- requests
            if av_r = '1' and av_q = '0' then pend_line <= '1'; end if;
            if vs_r = '0' and vs_q = '1' then pend_row0 <= '1'; end if;
            -- Row 0 is re-seeded ONLY at vsync.  Honouring the sequencer's request the
            -- moment it arrives would reset the vertical accumulators mid-frame whenever
            -- the per-frame pass ran past the vertical blanking interval, restarting the
            -- iris partway down the screen (seen on hardware as displaced arcs).
            if stx_en = '1' then
                case stx_sel is
                    when "00000" => c_xa0 <= stx_val(19 downto 0);
                    when "00001" => c_ya0 <= stx_val(19 downto 0);
                    when "00010" => c_xb0 <= stx_val(19 downto 0);
                    when "00011" => c_yb0 <= stx_val(19 downto 0);
                    when "00100" => c_sax <= stx_val(19 downto 0);
                    when "00101" => c_say <= stx_val(19 downto 0);
                    when "00110" => c_sbx <= stx_val(19 downto 0);
                    when "00111" => c_sby <= stx_val(19 downto 0);
                    when "01000" => c_sblx <= stx_val(19 downto 0);
                    when "01001" => c_sbly <= stx_val(19 downto 0);
                    when others  => null;                       -- selectors above 9 belong to the CPU
                end case;
            end if;

            v_say  := c_say; v_sblx := c_sblx; v_sbly := c_sbly;
            if ilace = '1' then
                v_say  := shift_left(v_say, 1);
                v_sblx := shift_left(v_sblx, 1);
                v_sbly := shift_left(v_sbly, 1);
            end if;

            -- free-running log lane pipeline (shared by both lanes)
            v_sh0 := to_unsigned(C_CW - 1, 5) - lg_e0;
            case v_sh0(4 downto 2) is
                when "000"  => lg_n1 <= lg_in;
                when "001"  => lg_n1 <= shift_left(lg_in, 4);
                when "010"  => lg_n1 <= shift_left(lg_in, 8);
                when "011"  => lg_n1 <= shift_left(lg_in, 12);
                when others => lg_n1 <= shift_left(lg_in, 16);
            end case;
            lg_f1 <= v_sh0(1 downto 0); lg_e1 <= lg_e0;
            v_n := shift_left(lg_n1, to_integer(lg_f1));
            -- the normaliser puts the leading one at bit C_CW-1, so the mantissa slices
            -- must track C_CW (they were hard-wired for an 18-bit datapath)
            lg_hi2 <= v_n(C_CW - 2 downto C_CW - 9); lg_lo2 <= v_n(C_CW - 10 downto C_CW - 13); lg_e2 <= lg_e1;
            lg_w3 <= tm_ram(to_integer(lg_hi2)); lg_lo3 <= lg_lo2; lg_e3 <= lg_e2;
            lg_v4 <= unsigned(lg_w3(15 downto 5)); lg_q4 <= unsigned(lg_w3(4 downto 0)); lg_lo4 <= lg_lo3; lg_e4 <= lg_e3;
            lg_p5 <= lg_lo4 * (lg_q4 & '0'); lg_v5 <= lg_v4;
            lg_ep5 <= lg_e4 * c_invn;
            lg_out <= resize(lg_ep5, 17) + resize(lg_v5, 17) + resize(lg_p5(9 downto 5), 17);

            -- last-period flag, registered (eng_k only changes at ph7)
            if eng_k = eng_last then eng_kl <= '1'; else eng_kl <= '0'; end if;

            -- table reads for theta*A and the split (addresses registered by the tail)
            tv_w <= tv_ram(to_integer(tv_ad));
            ts_w <= ts_ram(to_integer(ts_ad));

            -- CORDIC load preparation, registered in two steps so neither is a long
            -- combinational chain: select + abs + saturate at ph6 (lane a) / ph2 (lane b),
            -- compare + swap at ph7 / ph3, load at ph0 / ph4.
            if eng_oh(6) = '1' then
                lx := sxa(18 downto 3); ly := sya(18 downto 3);     -- lane A: z - a, in Q13
            else
                lx := sxb(18 downto 3); ly := syb(18 downto 3);     -- lane B: 1 - conj(a) z, in Q13
            end if;
            if lx(15) = '1' then ax := -lx; else ax := lx; end if;
            if ly(15) = '1' then ay := -ly; else ay := ly; end if;
            if ax(15) = '1' then ax := to_signed(32767, 16); end if;
            if ay(15) = '1' then ay := to_signed(32767, 16); end if;
            if eng_oh(6) = '1' or eng_oh(2) = '1' then
                la_x <= unsigned(ax(14 downto 0)); la_y <= unsigned(ay(14 downto 0));
                la_s <= lx(15) & ly(15);
            end if;
            if eng_oh(7) = '1' or eng_oh(3) = '1' then
                if la_x >= la_y then
                    ld_hi <= la_x; ld_lo <= la_y; ld_f <= '0' & la_s;
                else
                    ld_hi <= la_y; ld_lo <= la_x; ld_f <= '1' & la_s;
                end if;
            end if;

            -- free-running run lengths (act_w is static), so the start path has no adders
            nsamp_r <= resize(act_w(11 downto 3), 9) + 1;
            last_r  <= resize(act_w(11 downto 3), 9) + 3;

            -- run start in two steps: pick the pending request (a priority chain over three
            -- flags), then load the run registers from a registered 2-bit selector
            if st_go = '1' then
                st_go <= '0'; eng_busy <= '1';
                eng_oh <= "00100000"; eng_oh2 <= "00100000"; eng_k <= (others => '1');
                cj0 <= "0001"; cj1 <= "0001"; cj2 <= "0001";   -- runs start at ph5, so the index reads 3 at ph0
                eng_nsamp <= nsamp_r; eng_last <= last_r;
                sxa <= c_xa0;
                if st_kind(1) = '0' then                       -- ordinary line: advance down the frame
                    sya <= sya + v_say;
                    lxb <= lxb + v_sblx;  sxb <= lxb + v_sblx;
                    lyb <= lyb + v_sbly;  syb <= lyb + v_sbly;
                elsif ilace = '1' and fl_r = '0' then          -- row 0, bottom field: start half a line down
                    sya <= c_ya0 + shift_right(v_say, 1);
                    lxb <= c_xb0 + shift_right(v_sblx, 1);  sxb <= c_xb0 + shift_right(v_sblx, 1);
                    lyb <= c_yb0 + shift_right(v_sbly, 1);  syb <= c_yb0 + shift_right(v_sbly, 1);
                else                                           -- row 0
                    sya <= c_ya0;  lxb <= c_xb0;  sxb <= c_xb0;  lyb <= c_yb0;  syb <= c_yb0;
                end if;
            elsif eng_busy = '0' then
                if pend_line = '1' then
                    pend_line <= '0'; st_go <= '1'; st_kind <= "00";
                elsif pend_row0 = '1' then
                    pend_row0 <= '0'; st_go <= '1'; st_kind <= "10";
                end if;
            else
                eng_oh <= eng_oh(6 downto 0) & eng_oh(7); eng_oh2 <= eng_oh2(6 downto 0) & eng_oh2(7);
                -- CORDIC pipeline: each stage runs one iteration per clock (j = ph-1 mod 4);
                -- at ph0/ph4 (j = 3) the stages hand over: stage 0 loads a lane (a at
                -- ph0, b at ph4), stage 2 delivers a lane (a at ph4, b at ph0).
                cj0 <= cj0(2 downto 0) & cj0(3); cj1 <= cj1(2 downto 0) & cj1(3); cj2 <= cj2(2 downto 0) & cj2(3);
                vx := c0x; vy := c0y; vz := c0z; cordic_it(vx, vy, vz, 0, cj0); v0x := vx; v0y := vy; v0z := vz;
                vx := c1x; vy := c1y; vz := c1z; cordic_it(vx, vy, vz, 4, cj1); v1x := vx; v1y := vy; v1z := vz;
                vx := c2x; vy := c2y; vz := c2z; cordic_it(vx, vy, vz, 8, cj2); v2x := vx; v2y := vy; v2z := vz;
                c0x <= v0x; c0y <= v0y; c0z <= v0z;
                c1x <= v1x; c1y <= v1y; c1z <= v1z;
                c2x <= v2x; c2y <= v2y; c2z <= v2z;
                -- stage-0 load (lane a at ph0, lane b at ph4) from values prepared one clock earlier
                if eng_oh(0) = '1' or eng_oh(4) = '1' then
                    c0x <= shift_left(signed(resize(ld_hi, C_CW)), C_CG);
                    c0y <= shift_left(signed(resize(ld_lo, C_CW)), C_CG);
                    c0z <= (others => '0');
                    c0f <= ld_f;
                    c1x <= v0x; c1y <= v0y; c1z <= v0z; c1f <= c0f;
                    c2x <= v1x; c2y <= v1y; c2z <= v1z; c2f <= c1f;
                    if v2x < 0 then v_xc := (others => '0'); else v_xc := unsigned(v2x); end if;
                    if eng_oh(0) = '1' then
                        rb_x <= v_xc; rb_z <= v2z; rb_sw <= c2f(2); rb_sx <= c2f(1); rb_sy <= c2f(0);
                        sxa <= sxa + c_sax;                         -- lane A's y is constant along a line
                    else
                        ra_x <= v_xc; ra_z <= v2z; ra_sw <= c2f(2); ra_sx <= c2f(1); ra_sy <= c2f(0);
                        sxb <= sxb + c_sbx; syb <= syb + c_sby;
                    end if;
                end if;
                -- shared angle fold (lane b at ph1, lane a at ph5)
                if eng_oh(1) = '1' then
                    v_rz := rb_z; v_rsw := rb_sw; v_rsx := rb_sx; v_rsy := rb_sy; v_rx := rb_x;
                else
                    v_rz := ra_z; v_rsw := ra_sw; v_rsx := ra_sx; v_rsy := ra_sy; v_rx := ra_x;
                end if;
                if v_rsw = '1' then v_m := to_signed(32768, 18) - resize(v_rz, 18);
                else                v_m := resize(v_rz, 18); end if;
                if v_rsx = '0' and v_rsy = '0' then    v_fold := unsigned(v_m(16 downto 0));
                elsif v_rsx = '1' and v_rsy = '0' then v_fold := unsigned(resize(to_signed(65536, 18) - v_m, 17));
                elsif v_rsx = '1' and v_rsy = '1' then v_fold := unsigned(resize(to_signed(65536, 18) + v_m, 17));
                else                                   v_fold := unsigned(resize(-v_m, 17)); end if;
                if eng_oh(1) = '1' or eng_oh(5) = '1' then
                    lg_in <= v_rx; lg_e0 <= f_lzc(v_rx);
                    if eng_oh(1) = '1' then fb <= v_fold; else fa <= v_fold; end if;
                end if;

                -- tail.  Sample k is loaded in period k; lane a's log is in lg_out
                -- during ph4..7 of k+2, lane b's during ph0..3 of k+3 (each log-lane
                -- stage holds for 4 clocks).  The word of sample k is written at
                -- ph7 of period k+3.
                if eng_oh(0) = '1' then
                    ea <= lg_e2; ha <= lg_hi2;                                          -- lane A, sample k-2
                    lg_b <= lg_out;                                                     -- lane b, sample k-3
                    e_v <= e_vh + e_vl + (vcut & x"000");                              -- sample k-2, cut-corrected
                end if;
                if eng_oh(1) = '1' then
                    e_l <= signed(resize(lg_a, 18)) - signed(resize(lg_b, 18));        -- sample k-3
                end if;
                if eng_oh(2) = '1' then
                    v_th := fa - fb;
                    th <= v_th;                                                         -- sample k-2
                    e_ld <= resize(e_l, 19) - resize(c_lp, 19);                        -- sample k-3
                    -- The arm coordinate holds 16 arms, so where the angle wraps (its cut,
                    -- a curve running out of the pupil) v jumps by A mod 16 arms.  That is
                    -- a whole number of arms, so the dot test is right AT the samples --
                    -- but the pixel-path DDA interpolates between the two samples that
                    -- straddle the cut and sweeps through those arms in 8 px, planting one
                    -- phantom dot per arm along the cut.  Count the crossings along the
                    -- line (by angle quadrant, 3->0 forward, 0->3 back) and add that many
                    -- times A mod 16 to v, which keeps consecutive samples continuous and
                    -- changes nothing mod one arm.
                    if eng_k(8) = '0' and eng_k >= 2 then                              -- real samples only
                        if eng_k = 2 then vcut <= (others => '0');
                        elsif thq_p = "11" and v_th(16 downto 15) = "00" then vcut <= vcut + c_a;
                        elsif thq_p = "00" and v_th(16 downto 15) = "11" then vcut <= vcut - c_a;
                        end if;
                        thq_p <= v_th(16 downto 15);
                    end if;
                end if;
                if eng_oh(3) = '1' then
                    tv_ad <= '0' & th(16 downto 10);
                    v_dl := shift_left(resize(e_ld, 27), to_integer(c_inve));
                    if v_dl > 524287 then e_u <= to_signed(524287, 20);
                    elsif v_dl < -524288 then e_u <= to_signed(-524288, 20);
                    else e_u <= v_dl(19 downto 0); end if;
                end if;
                if eng_oh(4) = '1' then
                    lg_a <= lg_out;                                                     -- lane a, sample k-2
                    eb <= lg_e2; hb <= lg_hi2;                                          -- lane b, sample k-2
                    tv_ad <= '1' & th(9 downto 3);
                    if e_u(19) = '1' then ts_ad <= (others => '0');
                    elsif e_u(18) = '1' then ts_ad <= (others => '1');
                    else ts_ad <= unsigned(e_u(17 downto 10)); end if;
                end if;
                if eng_oh(5) = '1' then
                    e_vh <= unsigned(tv_w);
                    -- the anti-alias ramp normalises d(log|w|)/d(pixel) = (1-|a|^2)/(r1 r2 R),
                    -- so it needs log2(r1 r2) from both lanes' exponents and mantissa tops
                    v_es := resize(ea, 6) + resize(eb, 6);
                    v_hs := resize(ha, 9) + resize(hb, 9);
                    e_gs <= (v_es & x"000") + resize(v_hs & "0000", 18);
                end if;
                if eng_oh(6) = '1' then
                    e_vl <= unsigned(tv_w);
                    e_sep <= signed(ts_w(13 downto 0));                                 -- sample k-3
                    v_u  := resize(c_cdot, 22) - signed(resize(e_gs, 22));
                    -- round to the nearest octave for the AA shift with a 10-bit
                    -- increment off the same subtraction, not a second 22-bit add
                    if v_u(11) = '1' then v_sh := v_u(21 downto 12) + 1;
                    else                  v_sh := v_u(21 downto 12); end if;
                    e_rd2 <= e_rd;                                                      -- sample k-3
                    -- four bits: the resolution fade has emptied everything past rd 10,
                    -- so the barrel never needs a fifth shift stage
                    if v_sh < 0 then e_rd <= (others => '0');
                    elsif v_sh > 15 then e_rd <= to_unsigned(15, 4);
                    else e_rd <= unsigned(v_sh(3 downto 0)); end if;
                    -- Resolution fade.  v_u is log2 of the lattice density in Q12 ring
                    -- units per pixel: f_bar's shift turns a Q12 ring distance into a
                    -- 256-per-pixel alpha ramp, which pins rd = log2(units per pixel)
                    -- exactly.  Because the map is conformal that ONE number covers both
                    -- axes -- rings and arms crowd together by the same factor -- so the
                    -- arms need no test of their own (and a difference of the arm
                    -- coordinate could not supply one anyway: it wraps every 16 arms, and
                    -- the wrap would black out a radial spoke, a seam of the kind being
                    -- cured here).  Past one ring per 8 px (rd 9) the lattice is finer
                    -- than the sampler's own pitch, the DDA interpolates across structure
                    -- it never saw, and the dots degenerate into coloured hash; by one
                    -- ring per 4 px (rd 10) there is nothing left to draw.  That span is
                    -- exactly one octave and sits on a power of two, so the ramp between
                    -- them is a plain slice of the subtraction already made above.
                    e_at2 <= e_at;                                                      -- sample k-3
                    if v_u(21 downto 12) < to_signed(9, 10) then e_at <= (others => '0');
                    elsif v_u(21 downto 12) > to_signed(9, 10) then e_at <= (others => '1');
                    else e_at <= unsigned(v_u(11 downto 6)); end if;
                end if;
                if eng_oh(7) = '1' then
                    if eng_kl = '1' then
                        eng_busy <= '0';
                    else
                        eng_k <= eng_k + 1;
                    end if;
                end if;
            end if;
        end if;
    end process p_eng;

    ------------------------------------------------------------------------
    -- sample buffer (4 EBR, single bank: the engine's writes for the next
    -- line trail the beam's reads of this line) + sync delay line
    ------------------------------------------------------------------------
    p_sbuf : process(clk)
        variable v_ra : unsigned(8 downto 0);
        variable v_wk : unsigned(8 downto 0);
        variable v_d0, v_d1, v_d2, v_d3 : std_logic_vector(15 downto 0);
    begin
        if rising_edge(clk) then
            -- engine write of the word of sample k-3 at ph7
            v_wk := eng_k - 3;
            v_d0 := std_logic_vector(e_u(15 downto 0));
            v_d1 := std_logic_vector(e_u(19 downto 16)) & std_logic_vector(e_v(11 downto 0));
            v_d2 := "0" & std_logic_vector(e_v(15 downto 12)) & std_logic_vector(e_sep(10 downto 0));
            v_d3 := std_logic_vector(e_sep(13 downto 11)) & "000" & std_logic_vector(e_rd2) & std_logic_vector(e_at2);
            if eng_busy = '1' and eng_oh2(7) = '1' and eng_k >= 3 and v_wk < eng_nsamp then
                sb0(to_integer(v_wk(7 downto 0))) <= v_d0;
                sb1(to_integer(v_wk(7 downto 0))) <= v_d1;
                sb2(to_integer(v_wk(7 downto 0))) <= v_d2;
                sb3(to_integer(v_wk(7 downto 0))) <= v_d3;
            end if;
            -- beam reads (px = pixel-1, span k = pixels 8k..8k+7 uses samples k, k+1):
            -- at px=1 mod 8 fetch sample k+1 (the pair's next member, loaded at px=2);
            -- at px=2 mod 8 fetch sample k-1, whose AA shift is captured at px=3 for
            -- stage 14 (13 clocks behind the input, i.e. span k-1)
            if av_q = '0' then v_ra := (others => '0');
            elsif px(2 downto 0) = "010" then v_ra := resize(px(10 downto 3), 9) - 1;
            else v_ra := resize(px(10 downto 3), 9) + 1; end if;
            sq0 <= sb0(to_integer(v_ra(7 downto 0)));
            sq1 <= sb1(to_integer(v_ra(7 downto 0)));
            sq2 <= sb2(to_integer(v_ra(7 downto 0)));
            sq3 <= sb3(to_integer(v_ra(7 downto 0)));
            if px(2 downto 0) = "011" then
                rd_h1 <= unsigned(sq3(9 downto 6));     -- (word: sep[15:13] 000[12:10] rd[9:6] fade[5:0])
                res6  <= unsigned(sq3(5 downto 0)) & "00";
            end if;
            sd_ram(to_integer(dcnt)) <= av_r & hs_r & vs_r & fl_r;
            sd_q <= sd_ram(to_integer(dcnt - 19));   -- 17, plus the two pixels the layer holds now trail by
        end if;
    end process p_sbuf;

    ------------------------------------------------------------------------
    -- pixel front: sample pair registers + DDA between samples (stage 6:
    -- the accumulators ARE the stage-6 values)
    ------------------------------------------------------------------------
    p_dda : process(clk)
        variable v_du : signed(19 downto 0);
        variable v_dv : signed(15 downto 0);
        variable v_wu : std_logic_vector(19 downto 0);
        variable v_wv : std_logic_vector(15 downto 0);
        variable v_ws : std_logic_vector(13 downto 0);
    begin
        if rising_edge(clk) then
            if av_q = '0' then px <= (others => '1'); else px <= px + 1; end if;
            if av_q = '0' then slot <= (others => '0'); else slot <= slot + 1; end if;   -- G,R,G,B
            if av_q = '0' or px(2 downto 0) = "010" then
                cur_u <= nxt_u; cur_v <= nxt_v; sep6 <= nxt_s;
                v_wu := sq1(15 downto 12) & sq0;
                v_wv := sq2(14 downto 11) & sq1(11 downto 0);
                v_ws := sq3(15 downto 13) & sq2(10 downto 0);
                nxt_u <= signed(v_wu);
                nxt_v <= signed(v_wv);
                nxt_s <= signed(v_ws);
            end if;
            -- reload during blanking as well, otherwise the accumulators free-run
            -- through the whole blanking interval and the first pixels of every line
            -- are drawn from stale values (random dots down the left edge)
            if av_q = '0' or px(2 downto 0) = "011" then
                du8   <= nxt_u - cur_u;
                -- (The 16-bit arm coordinate wraps between samples, but only its low 12
                -- bits reach the dot test, and the wrap error is exactly 16 whole arms --
                -- a multiple of 4096 -- so the exact difference is already correct here.)
                v_dv  := nxt_v - cur_v;
                dv8   <= v_dv(14 downto 0);
                v_du  := cur_u - signed(resize(c_ph, 20));                          -- flow phase folded in
                acc_u <= v_du & "000";
                acc_v <= cur_v(11 downto 0) & "000";
            else
                acc_u <= acc_u + resize(du8, 23);
                acc_v <= acc_v + dv8;
            end if;
        end if;
    end process p_dda;
