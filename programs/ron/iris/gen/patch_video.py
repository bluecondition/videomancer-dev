#!/usr/bin/env python3
"""Video path: S7 Dots Synth/Video (halftone), S8 Backdrop Solid/Video (lighten/darken)."""
S = __import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), 'src') + '/'

# ---------------------------------------------------------------- r1
p = S + 'iris_r1.vhd'; s = open(p).read()
old = """    signal sd_ram   : t_ram4 := (others => (others => '0'));                -- sync delay
"""
new = """    signal sd_ram   : t_ram4 := (others => (others => '0'));                -- sync delay
    signal vd0, vd1 : t_ram16 := (others => (others => '0'));               -- video delay, 30 bits in 2 EBR
"""
assert old in s; s = s.replace(old, new)
old = """    attribute ram_style of sd_ram : signal is "block";
"""
new = """    attribute ram_style of sd_ram : signal is "block";
    attribute ram_style of vd0 : signal is "block";
    attribute ram_style of vd1 : signal is "block";
"""
assert old in s; s = s.replace(old, new)
old = """    signal sw_ink, sw_flow, sw_full : std_logic := '0';"""
new = """    signal sw_dots, sw_back, sw_ink, sw_flow, sw_full : std_logic := '0';
    -- video delay line: the picture the program was handed re-emerges beside the sync
    -- tap, so the halftone and the backdrop line up with it
    signal vr_y, vr_u, vr_v : unsigned(9 downto 0) := (others => '0');
    signal vq0, vq1 : std_logic_vector(15 downto 0) := (others => '0');
    signal vy2, vu2, vv2 : unsigned(9 downto 0) := (others => '0');
    signal vdk      : unsigned(7 downto 0) := (others => '0');   -- video darkness -> dot attenuation
    signal vwin18   : std_logic := '0';                          -- backdrop wins this pixel"""
assert old in s; s = s.replace(old, new)
open(p, 'w').write(s)

# ---------------------------------------------------------------- r2
p = S + 'iris_r2.vhd'; s = open(p).read()
old = """            sw_ink  <= registers_in(6)(2);"""
new = """            sw_dots <= registers_in(6)(0);
            sw_back <= registers_in(6)(1);
            sw_ink  <= registers_in(6)(2);"""
assert old in s; s = s.replace(old, new)
old = """            sd_ram(to_integer(dcnt)) <= av_r & hs_r & vs_r & fl_r;
            sd_q <= sd_ram(to_integer(dcnt - 17));"""
new = """            sd_ram(to_integer(dcnt)) <= av_r & hs_r & vs_r & fl_r;
            sd_q <= sd_ram(to_integer(dcnt - 17));
            -- the video takes the same road: one input register, the same free-running
            -- address, and a read two fabric registers short of the sync tap
            vr_y <= unsigned(data_in.y); vr_u <= unsigned(data_in.u); vr_v <= unsigned(data_in.v);
            vd0(to_integer(dcnt)) <= std_logic_vector(vr_y) & std_logic_vector(vr_u(9 downto 4));
            vd1(to_integer(dcnt)) <= std_logic_vector(vr_u(3 downto 0)) & std_logic_vector(vr_v) & "00";
            vq0 <= vd0(to_integer(dcnt - 16));
            vq1 <= vd1(to_integer(dcnt - 16));
            vy2 <= unsigned(vq0(15 downto 6));
            vu2 <= unsigned(vq0(5 downto 0)) & unsigned(vq1(15 downto 12));
            vv2 <= unsigned(vq1(11 downto 2));"""
assert old in s; s = s.replace(old, new)
open(p, 'w').write(s)

# ---------------------------------------------------------------- r3
p = S + 'iris_r3.vhd'; s = open(p).read()

# Dots: Video -- the incoming luma becomes attenuation.  ig10 already carries the
# fraction the rim fade wants, so the halftone is the only new term in the sum.
old = """            if fdg10 = '1' and sw_flow = '1' then v_rm := rm10; else v_rm := (others => '0'); end if;
            v_at := resize(pu10, 9) + resize(v_rm, 9);
            if v_at(8) = '1' then att11 <= x"FF"; else att11 <= v_at(7 downto 0); end if;"""
new = """            if fdg10 = '1' and sw_flow = '1' then v_rm := rm10; else v_rm := (others => '0'); end if;
            v_at := resize(pu10, 10) + resize(v_rm, 10) + resize(vdk, 10);
            if v_at(9 downto 8) /= "00" then att11 <= x"FF"; else att11 <= v_at(7 downto 0); end if;"""
assert old in s; s = s.replace(old, new)
old = """        variable v_at : unsigned(8 downto 0);"""
new = """        variable v_at : unsigned(9 downto 0);"""
assert old in s; s = s.replace(old, new)

old = """            -- P18: Y8, U = 0.5625 (B-Y), V = 0.719 (R-Y)
            y18 <= yy17(11 downto 4);"""
new = """            -- Dots: Video -- the incoming luma turned into attenuation, so a dark picture
            -- eats the dots and a bright one lets them fill: the iris becomes a halftone
            -- screen whose cells are the spiral lattice.  Inverting the top eight bits is
            -- the whole scale (white leaves 20/255 of attenuation, black leaves 239).
            if sw_dots = '1' then vdk <= not vy2(9 downto 2); else vdk <= (others => '0'); end if;
            -- Backdrop: Video -- lighten in Light, darken in Ink.  Both come to the same
            -- thing: wherever the dot field is at its ground (black, or paper) the
            -- picture wins, and the dots stand on top of it.  Deciding it here, from the
            -- luma before it is registered, leaves P19 a plain mux.
            v_dy := to_unsigned(64, 11) + shift_left(resize(yy17(11 downto 4), 11), 1)
                    + resize(yy17(11 downto 5), 11) + resize(yy17(11 downto 6), 11);
            if sw_ink = '0' then
                if resize(vy2, 11) > v_dy then vwin18 <= '1'; else vwin18 <= '0'; end if;
            else
                if resize(vy2, 11) < v_dy then vwin18 <= '1'; else vwin18 <= '0'; end if;
            end if;
            -- P18: Y8, U = 0.5625 (B-Y), V = 0.719 (R-Y)
            y18 <= yy17(11 downto 4);"""
assert old in s; s = s.replace(old, new)

old = """    p_out : process(clk)
        variable v_n  : signed(6 downto 0);"""
new = """    p_out : process(clk)
        variable v_dy : unsigned(10 downto 0);
        variable v_n  : signed(6 downto 0);"""
assert old in s; s = s.replace(old, new)

old = """            sd_o <= sd_q;
            if sd_q(3) = '0' then
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            else
                s_out_y <= v_yo(9 downto 0);
                s_out_u <= unsigned(v_vo(9 downto 0));
                s_out_v <= unsigned(v_uo(9 downto 0));
            end if;"""
new = """            sd_o <= sd_q;
            if sd_q(3) = '0' then
                s_out_y <= to_unsigned(64, 10);
                s_out_u <= C_MID;
                s_out_v <= C_MID;
            elsif sw_back = '1' and vwin18 = '1' then
                s_out_y <= vy2;
                s_out_u <= vu2;
                s_out_v <= vv2;
            else
                s_out_y <= v_yo(9 downto 0);
                s_out_u <= unsigned(v_vo(9 downto 0));
                s_out_v <= unsigned(v_uo(9 downto 0));
            end if;"""
assert old in s; s = s.replace(old, new)
open(p, 'w').write(s)
print('video patch applied')
