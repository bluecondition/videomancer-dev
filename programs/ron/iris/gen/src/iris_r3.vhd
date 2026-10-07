    ------------------------------------------------------------------------
    -- One dot machine shared by the three layers: at pixel p it evaluates
    -- layer slot(p) (0 R, 1 G, 2 B) with that layer's split offset, and the
    -- resulting alpha is held for the next three pixels.  Stage k of the
    -- machine is true stage k.
    --
    -- P7: layer offset, pupil alpha
    -- P8: twist term (per-ring EBR table)
    ------------------------------------------------------------------------
    p_lat : process(clk)
        variable v_u6  : signed(19 downto 0);
        variable v_v6  : unsigned(11 downto 0);
        variable v_ul  : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            -- P7
            v_u6 := acc_u(22 downto 3);                -- already offset by the flow phase
            v_v6 := unsigned(acc_v(14 downto 3));
            v_ul := v_u6;
            -- Four-phase layer schedule G,R,G,B: green (which carries most of the
            -- luminance, and so most of the apparent sharpness) is refreshed every
            -- second pixel instead of every third; red and blue, which only carry the
            -- colour fringing, are refreshed every fourth -- and smoothed to a 2 px
            -- cadence in p_out.
            case slot is
                when "01" =>                       -- red leads
                    ul7 <= v_ul + resize(shift_right(sep6, 1), 20);
                    v7  <= v_v6 + unsigned(sep6(11 downto 0));
                when "11" =>                       -- blue trails
                    ul7 <= v_ul - resize(shift_right(sep6, 2), 20);
                    v7  <= v_v6 - unsigned(sep6(11 downto 0));
                when others =>                     -- green anchors (slots 0 and 2)
                    ul7 <= v_ul;
                    v7  <= v_v6;
            end case;
            slot7 <= slot;
            av7   <= av_q;
            -- P8 (twist term from the per-ring table)
            ug8 <= ul7;
            vg8 <= v7;
            tk_q <= tk_ram(to_integer(unsigned(ul7(19 downto 12))));
        end if;
    end process p_lat;

    ------------------------------------------------------------------------
    -- P9: ring/frac split, twist offset
    -- P10: arm coordinate (mod 1) -> |dv| -> table indices, validity flags
    ------------------------------------------------------------------------
    p_cell : process(clk)
        variable v_tk : unsigned(11 downto 0);
        variable v_vg : unsigned(11 downto 0);
        variable v_d  : signed(11 downto 0);
        variable v_ad : unsigned(11 downto 0);
        variable v_pd : signed(20 downto 0);
        variable v_rc : std_logic_vector(19 downto 0);
    begin
        if rising_edge(clk) then
            -- P9: ring/frac split, and the pupil fade.  The distance is measured from the
            -- RING'S OWN CENTRE (its dots' u), not from the pixel's u, so every dot in a
            -- ring gets one and the same attenuation: the rings go out one at a time as
            -- the knob turns instead of each ring shading across itself.  Two rings from
            -- full to black, so only one ring is ever in transition.
            kk9 <= ug8(19 downto 12); fg9 <= unsigned(ug8(11 downto 0));
            v_rc := std_logic_vector(ug8(19 downto 12)) & x"800";
            v_pd := resize(c_up, 21) - resize(signed(v_rc), 21);
            if v_pd <= 0 then pu9 <= (others => '0');
            elsif v_pd(20 downto 13) /= x"00" then pu9 <= x"FF";
            else pu9 <= unsigned(v_pd(12 downto 5)); end if;
            tk9 <= signed(tk_q(13 downto 0));
            vg9 <= vg8;
            -- P10: v = theta*A (+sep) + T(k+1/2+ph), then |dv|, indices
            v_tk := unsigned(tk9(11 downto 0));
            v_vg := vg9 + v_tk;
            v_d := signed(v_vg xor x"800");
            if v_d(11) = '1' then v_ad := unsigned(-v_d); else v_ad := unsigned(v_d); end if;
            if v_ad(11) = '1' then v_ad := x"7FF"; end if;
            jg10 <= v_ad(10 downto 3); lvg10 <= v_ad(2 downto 0);
            ig10 <= fg9(11 downto 4);
            pu10 <= pu9;                       -- (ig10 doubles as the rim fade's position across the ring)
            if kk9 < c_n then vldg10 <= '1'; else vldg10 <= '0'; end if;
            if kk9 = c_nm1 then fdg10 <= '1'; else fdg10 <= '0'; end if;
        end if;
    end process p_cell;

    ------------------------------------------------------------------------
    -- P11: table EBR reads   P12: re-register   P13: diff = B - (S + slope*lo)
    ------------------------------------------------------------------------
    p_dot : process(clk)
        variable v_pg : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            t1g11 <= t1_ram(to_integer(ig10));
            t2g11 <= t2_ram(to_integer(jg10));
            lvg11 <= lvg10;

            bvg12 <= signed(t1g11); svg12 <= unsigned(t2g11(15 downto 3)); ssg12 <= unsigned(t2g11(2 downto 0));
            lvg12 <= lvg11;

            v_pg := ssg12 * lvg12;
            dfg13 <= resize(bvg12, 17) - signed(resize(svg12, 17)) - signed(resize(v_pg, 17));
        end if;
    end process p_dot;

    ------------------------------------------------------------------------
    -- bit carries: pupil alpha / layer id (P7 -> P15), flags (P10 -> P14)
    ------------------------------------------------------------------------
    p_carry : process(clk)
        variable v_rm : unsigned(7 downto 0);
        variable v_at : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            lay_d(7) <= av7 & slot7;
            for i in 8 to 15 loop lay_d(i) <= lay_d(i - 1); end loop;
            vldg_d(10) <= vldg10;
            for i in 11 to 14 loop
                vldg_d(i) <= vldg_d(i - 1);
            end loop;
            -- P11: total attenuation.  The outermost ring's level is a PER-FRAME
            -- constant (c_fade = flow phase), so the whole ring dims together as it
            -- drifts out through the rim and is gone by the time it would cross -- rather
            -- than shading from one side of itself to the other.  It also keeps the field
            -- inside the rim: Flow carries the band up to a ring past |w| = 1, and this
            -- is what stops it being drawn there.
            if fdg10 = '1' then v_rm := c_fade; else v_rm := (others => '0'); end if;
            v_at := resize(pu10, 10) + resize(v_rm, 10) + resize(res6, 10);
            if v_at(9 downto 8) /= "00" then att11 <= x"FF"; else att11 <= v_at(7 downto 0); end if;
            att_d(12) <= att11;
            att_d(13) <= att_d(12);
            att_d(14) <= att_d(13);
        end if;
    end process p_carry;

    ------------------------------------------------------------------------
    -- P14: AA barrel   P15: alpha, demuxed into per-layer holds
    -- P16: composite + pupil  P17: Y product   P18: Y8/U/V   P19: output
    --
    -- Red and blue are evaluated once every four pixels and used to be simply
    -- HELD, so the edge of every red or blue dot was a staircase of 4 px
    -- treads -- the "pixelated" coloured edges.  Now each layer keeps its
    -- previous evaluation too, and for the two pixels after a fresh one it
    -- shows the midpoint of the two before showing the new value: a 2 px
    -- cadence with a half step between, the same quality as green's own
    -- 2 px hold.  That output trails the beam by two or three pixels, so
    -- green is taken from ITS previous evaluation (two or three behind at
    -- its cadence) to match, and the sync tap moves by two so nothing on
    -- screen shifts.  At the first evaluation of a line the "previous" value
    -- is the line above's tail, so it is seeded with the fresh one instead.
    -- With the split at zero all three layers sit on the same lattice and
    -- would draw the SAME dots at different sampling phases -- the coloured
    -- fringe seen at 0 %.  The sequencer raises c_fl(0) there and red and
    -- blue simply become copies of green.
    ------------------------------------------------------------------------
    p_out : process(clk)
        variable v_rd : unsigned(3 downto 0);
        variable v_sl : unsigned(1 downto 0);
        variable v_ar, v_ab : unsigned(8 downto 0);
        variable v_r, v_b : unsigned(7 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_bmy, v_rmy : signed(9 downto 0);
        variable v_u, v_v : signed(9 downto 0);
        variable v_yo : unsigned(10 downto 0);
        variable v_uo, v_vo : signed(11 downto 0);
        variable v_ax : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            -- P14.  Glow: shift by a constant 11 -- the scale at which a dot is one
            -- pixel wide -- so the ramp spans the whole dot instead of one pixel of
            -- its edge, and every dot becomes a soft ball with its peak at the centre.
            if sw_glow = '1' then v_rd := "1011"; else v_rd := rd_h1; end if;
            bar14_g <= f_bar(dfg13, v_rd);
            -- P15
            v_ax := f_alpha(bar14_g, vldg_d(13), not sw_glow);
            if v_ax > att_d(14) then v_ax := v_ax - att_d(14); else v_ax := (others => '0'); end if;
            if lay_d(13)(2) = '1' and lay_d(14)(2) = '0' then    -- first active pixel of the line
                fr_r <= '1'; fr_b <= '1';
            end if;
            if lay_d(13)(2) = '1' then                            -- (nothing moves during blanking)
                case lay_d(13)(1 downto 0) is
                    when "01"   => if fr_r = '1' then pv_r <= v_ax; else pv_r <= al15_r; end if;
                                   al15_r <= v_ax; fr_r <= '0';
                    when "11"   => if fr_b = '1' then pv_b <= v_ax; else pv_b <= al15_b; end if;
                                   al15_b <= v_ax; fr_b <= '0';
                    when others => pv_g <= al15_g; al15_g <= v_ax;
                end case;
            end if;
            -- P16 composite (light: additive on black; ink: subtractive on white).
            -- Red was evaluated at slot 1, so slots 1 and 2 show its midpoint; blue
            -- at slot 3, so slots 3 and 0 show its midpoint.
            v_sl := lay_d(14)(1 downto 0);
            v_ar := resize(al15_r, 9) + resize(pv_r, 9);
            v_ab := resize(al15_b, 9) + resize(pv_b, 9);
            if v_sl(0) /= v_sl(1) then v_r := v_ar(8 downto 1); else v_r := al15_r; end if;
            if v_sl(0) = v_sl(1)  then v_b := v_ab(8 downto 1); else v_b := al15_b; end if;
            if c_fl(0) = '1' then v_r := pv_g; v_b := pv_g; end if;
            cr16 <= f_comp(v_r, sw_ink);
            cg16 <= f_comp(pv_g, sw_ink);
            cb16 <= f_comp(v_b, sw_ink);
            -- P17: Y*16 = 5R + 9G + 2B
            v_y := shift_left(resize(cr16, 12), 2) + resize(cr16, 12)
                 + shift_left(resize(cg16, 12), 3) + resize(cg16, 12)
                 + shift_left(resize(cb16, 12), 1);
            yy17 <= v_y;
            r17 <= cr16; b17 <= cb16;
            -- P18: Y8, U = 0.5625 (B-Y), V = 0.719 (R-Y)
            y18 <= yy17(11 downto 4);
            v_bmy := signed(resize(b17, 10)) - signed(resize(yy17(11 downto 4), 10));
            v_rmy := signed(resize(r17, 10)) - signed(resize(yy17(11 downto 4), 10));
            v_u := shift_right(v_bmy, 1) + shift_right(v_bmy, 4);
            v_v := shift_right(v_rmy, 1) + shift_right(v_rmy, 2) - shift_right(v_rmy, 5);
            u18 <= resize(v_u, 9);
            v18 <= resize(v_v, 9);
            -- P19: to 10-bit (Y 64..~768, chroma 512 +/- 3*U8), blanking gate, U/V swap
            v_yo := to_unsigned(64, 11) + shift_left(resize(y18, 11), 1)
                    + resize(shift_right(y18, 1), 11) + resize(shift_right(y18, 2), 11);
            v_uo := to_signed(512, 12) + resize(u18, 12) + resize(u18, 12) + resize(u18, 12);
            v_vo := to_signed(512, 12) + resize(v18, 12) + resize(v18, 12) + resize(v18, 12);
            sd_o <= sd_q;
            if sd_q(3) = '0' then
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            else
                s_out_y <= v_yo(9 downto 0);
                s_out_u <= unsigned(v_vo(9 downto 0));
                s_out_v <= unsigned(v_uo(9 downto 0));
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_out_y);
    data_out.u       <= std_logic_vector(s_out_u);
    data_out.v       <= std_logic_vector(s_out_v);
    data_out.avid    <= sd_o(3);
    data_out.hsync_n <= sd_o(2);
    data_out.vsync_n <= sd_o(1);
    data_out.field_n <= sd_o(0);
