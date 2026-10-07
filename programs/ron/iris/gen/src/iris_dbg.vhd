    ------------------------------------------------------------------------
    -- simulation-only debug taps (spliced in by IRIS_DEBUG=1 assemble2.sh)
    ------------------------------------------------------------------------
    p_dbg : process(clk)
        variable v_fr : integer := 0;
        variable v_ln : integer := 0;
    begin
        if rising_edge(clk) then
            if vs_r = '0' and vs_q = '1' then v_fr := v_fr + 1; v_ln := 0; end if;
            if av_r = '0' and av_q = '1' then v_ln := v_ln + 1; end if;
            if av_r = '1' and av_q = '0' and v_ln = 200 then
                report "F" & integer'image(v_fr) & " RASTER w=" & integer'image(to_integer(act_w)) & " h=" & integer'image(to_integer(act_h))
                     & " vstep=" & integer'image(to_integer(vstep_base)) & " ilace=" & std_logic'image(ilace)
                     & " | lp=" & integer'image(to_integer(c_lp)) & " up=" & integer'image(to_integer(c_up))
                     & " invn=" & integer'image(to_integer(c_invn)) & " inve=" & integer'image(to_integer(c_inve))
                     & " n=" & integer'image(to_integer(c_n)) & " cdot=" & integer'image(to_integer(c_cdot))
                     & " xa0=" & integer'image(to_integer(c_xa0)) & " ya0=" & integer'image(to_integer(c_ya0))
                     & " xb0=" & integer'image(to_integer(c_xb0)) & " yb0=" & integer'image(to_integer(c_yb0))
                     & " sax=" & integer'image(to_integer(c_sax)) & " say=" & integer'image(to_integer(c_say))
                     & " sbx=" & integer'image(to_integer(c_sbx)) & " sblx=" & integer'image(to_integer(c_sblx));
            end if;
            if eng_busy = '1' and eng_oh(7) = '1' and eng_k >= 3 and v_fr = 4
               and (v_ln = 60 or v_ln = 160 or v_ln = 260) and (eng_k = 3 or eng_k = 45 or eng_k = 88) then
                report "F" & integer'image(v_fr) & " S ln=" & integer'image(v_ln) & " k=" & integer'image(to_integer(eng_k) - 3)
                     & " sxa=" & integer'image(to_integer(sxa)) & " sya=" & integer'image(to_integer(sya))
                     & " sxb=" & integer'image(to_integer(sxb)) & " syb=" & integer'image(to_integer(syb))
                     & " u=" & integer'image(to_integer(e_u)) & " v=" & integer'image(to_integer(e_v))
                     & " rd=" & integer'image(to_integer(e_rd2));
            end if;
            if st_go = '1' and v_fr = 4 and v_ln < 3 then
                report "F" & integer'image(v_fr) & " RUN ln=" & integer'image(v_ln) & " kind=" & integer'image(to_integer(st_kind))
                     & " sya=" & integer'image(to_integer(sya)) & " lxb=" & integer'image(to_integer(lxb));
            end if;
            if row0_req = '1' and v_fr >= 3 and v_fr <= 5 then
                report "F" & integer'image(v_fr) & " LINE0 at line " & integer'image(v_ln);
            end if;
        end if;
    end process p_dbg;

