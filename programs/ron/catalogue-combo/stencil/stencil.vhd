-- Stencil: catalogue combo #2 (seed 20260912) -- mosaic with per-cell colour
-- averaging + masking by luma / chroma.  Three knobs each; the switches and
-- the slider decide how they combine.
--
--   K1 Cells    cell width 2 .. 64 px (rows follow via K2)
--   K2 Aspect   cell height = width / 1, 2, 4, 8
--   K3 Grout    dark grid lines between cells
--   K4 Level    key threshold
--   K5 Soft     key edge softness
--   K6 Source   key from luma (0) .. saturation (100), blended
--
--   S7  Invert  mask inverted
--   S8  Key     mask computed from the mosaic colour (blocky) / the original
--   S9  Reveal  the masked region shows the original / its negative
--   S10 Sharp   luma stays original; only chroma is mosaic'd
--   S11 Grout   grid lines everywhere / only inside the mask
--   P12 Tint    masked cells pushed toward a hue, amount = slider
--
-- Horizontal cell average = running box sum (EBR ring delay) written into
-- the line buffer through a delayed write port; cells hold the read
-- address; rows are captured every M lines.  Dry video rides an EBR ring
-- for the key and the reveal.  Latency 16 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture stencil of program_top is

    constant C_LAT    : integer := 16;
    constant C_RING_D : integer := 12;     -- dry ring: data_in -> stage 14

    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then
            return to_unsigned(0, 10);
        elsif v > 1023 then
            return to_unsigned(1023, 10);
        else
            return unsigned(v(9 downto 0));
        end if;
    end function;

    --------------------------------------------------------------------------
    -- Memories
    --------------------------------------------------------------------------
    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    type t_bring is array (0 to 63) of std_logic_vector(31 downto 0);
    type t_ring is array (0 to 31) of std_logic_vector(9 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal bring : t_bring := (others => x"00040200");
    signal ringY : t_ring := (others => "0001000000");
    signal ringU : t_ring := (others => "1000000000");
    signal ringV : t_ring := (others => "1000000000");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";
    signal s_bring_q : std_logic_vector(31 downto 0) := x"00040200";
    signal s_bring_w : unsigned(5 downto 0) := (others => '0');
    signal s_ring_qy, s_ring_qu, s_ring_qv : std_logic_vector(9 downto 0) := (others => '0');
    signal s_ring_w : unsigned(4 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Controls / per-frame
    --------------------------------------------------------------------------
    signal s_k_cells, s_k_aspect, s_k_grout, s_k_level, s_k_soft, s_k_src : unsigned(9 downto 0);
    signal s_sw_inv, s_sw_keymos, s_sw_neg, s_sw_coarse, s_sw_groutin : std_logic;
    signal s_p_tint : unsigned(9 downto 0);
    signal s_nsh    : unsigned(2 downto 0) := to_unsigned(3, 3);   -- cell = 2^nsh, 1..6
    signal s_rows   : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_grout  : unsigned(7 downto 0) := (others => '0');
    signal s_level  : unsigned(9 downto 0) := (others => '0');
    signal s_soft   : unsigned(5 downto 0) := (others => '0');
    signal s_srcm   : unsigned(7 downto 0) := (others => '0');
    signal s_tint   : unsigned(7 downto 0) := (others => '0');
    signal s_vstep  : unsigned(1 downto 0) := "11";
    signal s_saw_active : std_logic := '0';

    --------------------------------------------------------------------------
    -- Tracking / box sum / delayed write
    --------------------------------------------------------------------------
    signal s_rx      : unsigned(10 downto 0) := (others => '0');
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line    : unsigned(10 downto 0) := (others => '0');
    signal s_line_width : unsigned(10 downto 0) := to_unsigned(480, 11);
    signal s_vcnt    : unsigned(6 downto 0) := (others => '0');
    signal s_capture : std_logic := '1';
    signal s_wpar, s_rbank : std_logic := '0';
    signal s_rowtop  : std_logic := '0';
    signal s_accy, s_accu, s_accv : unsigned(15 downto 0) := (others => '0');
    signal s_ay3, s_au3, s_av3 : unsigned(9 downto 0) := (others => '0');
    type t_x is array (1 to 3) of unsigned(10 downto 0);
    signal s_wx : t_x := (others => (others => '0'));
    type t_b3 is array (1 to 3) of std_logic;
    signal s_wp, s_wc : t_b3 := (others => '0');
    signal s_we4, s_wp4 : std_logic := '0';
    signal s_wx4 : unsigned(10 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Address / pixel chain
    --------------------------------------------------------------------------
    signal s_x4      : unsigned(10 downto 0) := (others => '0');
    signal s_hold5   : unsigned(10 downto 0) := (others => '0');
    signal s_addr    : unsigned(10 downto 0) := (others => '0');
    signal s_bg      : std_logic := '0';
    signal s_gr5     : std_logic := '0';
    signal s_a_bg, s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_dry_y, s_dry_u, s_dry_v : unsigned(9 downto 0) := (others => '0');
    type t_b is array (6 to 14) of std_logic;
    signal s_grd : t_b := (others => '0');
    signal s_ky8, s_ku8, s_kv8 : unsigned(9 downto 0) := (others => '0');
    signal s_du9, s_dv9 : signed(10 downto 0) := (others => '0');
    signal s_ky9     : unsigned(9 downto 0) := (others => '0');
    signal s_sat10   : unsigned(10 downto 0) := (others => '0');
    signal s_ky10    : unsigned(9 downto 0) := (others => '0');
    signal s_vl11, s_vs11 : unsigned(17 downto 0) := (others => '0');
    signal s_val12   : unsigned(9 downto 0) := (others => '0');
    signal s_dif13   : signed(11 downto 0) := (others => '0');
    signal s_ap14    : signed(18 downto 0) := (others => '0');
    signal s_alpha15 : unsigned(7 downto 0) := (others => '0');
    type t_y is array (7 to 15) of unsigned(9 downto 0);
    signal s_my, s_mu, s_mv : t_y := (others => (others => '0'));
    signal s_oy, s_ou, s_ov : t_y := (others => (others => '0'));
    signal s_ey16, s_eu16, s_ev16 : signed(10 downto 0) := (others => '0');
    signal s_ap16 : unsigned(7 downto 0) := (others => '0');
    signal s_my16, s_mu16, s_mv16 : unsigned(9 downto 0) := (others => '0');
    signal s_py17, s_pu17, s_pv17 : signed(19 downto 0) := (others => '0');
    signal s_my17, s_mu17, s_mv17 : unsigned(9 downto 0) := (others => '0');
    signal s_ap17 : unsigned(7 downto 0) := (others => '0');
    signal s_y18, s_u18, s_v18 : unsigned(9 downto 0) := (others => '0');
    signal s_gr18 : std_logic := '0';
    signal s_y19, s_u19, s_v19 : unsigned(9 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Sync delay
    --------------------------------------------------------------------------
    type t_bit_shift is array (0 to 18) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_cells  <= unsigned(registers_in(0));
    s_k_aspect <= unsigned(registers_in(1));
    s_k_grout  <= unsigned(registers_in(2));
    s_k_level  <= unsigned(registers_in(3));
    s_k_soft   <= unsigned(registers_in(4));
    s_k_src    <= unsigned(registers_in(5));
    s_sw_inv     <= registers_in(6)(0);
    s_sw_keymos  <= registers_in(6)(1);
    s_sw_neg     <= registers_in(6)(2);
    s_sw_coarse  <= registers_in(6)(3);
    s_sw_groutin <= registers_in(6)(4);
    s_p_tint   <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- Rings: box-sum delay (64 deep, read N back) and dry (32 deep)
    --------------------------------------------------------------------------
    p_rings : process(clk)
        variable v_n : unsigned(5 downto 0);
    begin
        if rising_edge(clk) then
            case to_integer(s_nsh) is
                -- read N-1 back: the sum then spans exactly N samples ending at the
                -- current pixel (N back gave N+1 samples and, at 64, a same-address
                -- read-during-write)
                when 1 => v_n := to_unsigned(1, 6);
                when 2 => v_n := to_unsigned(3, 6);
                when 3 => v_n := to_unsigned(7, 6);
                when 4 => v_n := to_unsigned(15, 6);
                when 5 => v_n := to_unsigned(31, 6);
                when others => v_n := to_unsigned(63, 6);
            end case;
            s_bring_w <= s_bring_w + 1;
            s_bring_q <= bring(to_integer(s_bring_w - v_n));
            bring(to_integer(s_bring_w)) <= "00" & data_in.y & data_in.u & data_in.v;
            s_ring_w  <= s_ring_w + 1;
            s_ring_qy <= ringY(to_integer(s_ring_w - C_RING_D));
            s_ring_qu <= ringU(to_integer(s_ring_w - C_RING_D));
            s_ring_qv <= ringV(to_integer(s_ring_w - C_RING_D));
            ringY(to_integer(s_ring_w)) <= data_in.y;
            ringU(to_integer(s_ring_w)) <= data_in.u;
            ringV(to_integer(s_ring_w)) <= data_in.v;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stage 1..4: tracking, box sums, cell average, delayed write port
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_oy, v_ou, v_ov : unsigned(9 downto 0);
        variable v_n : integer range 1 to 6;
        variable v_nr : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_prev_avid    <= data_in.avid;
            if data_in.avid = '1' then
                s_rx <= s_rx + 1;
                s_saw_active <= '1';
            else
                s_rx <= (others => '0');
            end if;

            -- stage 2: running sums (subtract the sample N back from the ring)
            v_oy := unsigned(s_bring_q(29 downto 20));
            v_ou := unsigned(s_bring_q(19 downto 10));
            v_ov := unsigned(s_bring_q(9 downto 0));
            if s_avid_sr(0) = '0' then
                s_accy <= (others => '0'); s_accu <= (others => '0'); s_accv <= (others => '0');
            else
                s_accy <= s_accy + resize(unsigned(data_in.y), 16) - resize(v_oy, 16);
                s_accu <= s_accu + resize(unsigned(data_in.u), 16) - resize(v_ou, 16);
                s_accv <= s_accv + resize(unsigned(data_in.v), 16) - resize(v_ov, 16);
            end if;
            -- stage 3: average = sum >> nsh (start-of-line partial windows read low; acceptable)
            v_n := to_integer(s_nsh);
            s_ay3 <= resize(shift_right(s_accy, v_n), 10);
            s_au3 <= resize(shift_right(s_accu, v_n), 10);
            s_av3 <= resize(shift_right(s_accv, v_n), 10);
            -- write port delayed 3 (+1)
            s_wx(1) <= s_rx; s_wp(1) <= s_wpar; s_wc(1) <= data_in.avid and s_capture;
            for i in 2 to 3 loop
                s_wx(i) <= s_wx(i - 1); s_wp(i) <= s_wp(i - 1); s_wc(i) <= s_wc(i - 1);
            end loop;
            s_wx4 <= s_wx(3); s_wp4 <= s_wp(3); s_we4 <= s_wc(3);

            -- per-frame decode
            case to_integer(s_vstep) is
                when 0 =>
                    v_nr := to_unsigned(1, 4) + resize(s_k_cells(9 downto 7), 4);
                    if v_nr > 6 then v_nr := to_unsigned(6, 4); end if;
                    s_nsh   <= v_nr(2 downto 0);
                    s_grout <= s_k_grout(9 downto 2);
                    s_level <= s_k_level;
                    s_soft  <= to_unsigned(63, 6) - s_k_soft(9 downto 4);
                    s_srcm  <= s_k_src(9 downto 2);
                    s_tint  <= s_p_tint(9 downto 2);
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    case to_integer(s_k_aspect(9 downto 8)) is
                        when 0 => s_rows <= resize(shift_left(to_unsigned(1, 7), to_integer(s_nsh)), 7);
                        when 1 => s_rows <= resize(shift_left(to_unsigned(1, 7), to_integer(s_nsh)) srl 1, 7);
                        when 2 => s_rows <= resize(shift_left(to_unsigned(1, 7), to_integer(s_nsh)) srl 2, 7);
                        when others => s_rows <= resize(shift_left(to_unsigned(1, 7), to_integer(s_nsh)) srl 3, 7);
                    end case;
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    if s_rows = 0 then s_rows <= to_unsigned(1, 7); end if;
                    s_vstep <= "11";
                when others => null;
            end case;

            -- line / frame events: row capture (roulette pixelation scheme)
            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx;
                s_line <= s_line + 1;
                if s_vcnt >= s_rows - 1 then
                    s_vcnt <= (others => '0');
                else
                    s_vcnt <= s_vcnt + 1;
                end if;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                if s_vcnt = 0 then
                    s_capture <= '1';
                    s_wpar    <= not s_wpar;
                    s_rbank   <= s_wpar;
                    s_rowtop  <= '1';
                else
                    s_capture <= '0';
                    s_rbank   <= s_wpar;
                    if s_vcnt = 1 then s_rowtop <= '1'; else s_rowtop <= '0'; end if;
                end if;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0');
                s_vcnt <= (others => '0');
                s_wpar <= '0';
                if s_saw_active = '1' then
                    s_vstep <= (others => '0');
                end if;
                s_saw_active <= '0';
            end if;

            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to 18 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Address chain 4..5: cell hold (address = cell start + half cell)
    --------------------------------------------------------------------------
    p_addr : process(clk)
        variable v_mask : unsigned(10 downto 0);
        variable v_half : unsigned(10 downto 0);
        variable v_a : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_avid_sr(2) = '0' then
                s_x4 <= (others => '0');
            else
                s_x4 <= s_x4 + 1;
            end if;
            v_mask := (others => '0');
            v_half := (others => '0');
            for i in 0 to 10 loop
                if i < to_integer(s_nsh) then v_mask(i) := '1'; end if;
            end loop;
            v_half(to_integer(s_nsh) - 1) := '1';
            v_a := resize(s_x4 and not v_mask, 12) + resize(v_half, 12);
            if v_a >= resize(s_line_width, 12) then
                s_addr <= s_line_width - 1;
            else
                s_addr <= v_a(10 downto 0);
            end if;
            s_bg <= '0';
            -- grout: first 2 px of a cell, or first 2 rows of a cell row
            if (s_x4 and v_mask) < 2 or s_rowtop = '1' then
                s_gr5 <= '1';
            else
                s_gr5 <= '0';
            end if;
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (averaged data written at stage 4, read at stage 6)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addr));
            if s_we4 = '1' and s_wp4 = '0' then lbY0(to_integer(s_wx4)) <= std_logic_vector(s_ay3); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addr));
            if s_we4 = '1' and s_wp4 = '1' then lbY1(to_integer(s_wx4)) <= std_logic_vector(s_ay3); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addr(10 downto 1)));
            if s_we4 = '1' and s_wp4 = '0' then lbU0(to_integer(s_wx4(10 downto 1))) <= std_logic_vector(s_au3(9 downto 2)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addr(10 downto 1)));
            if s_we4 = '1' and s_wp4 = '1' then lbU1(to_integer(s_wx4(10 downto 1))) <= std_logic_vector(s_au3(9 downto 2)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addr(10 downto 1)));
            if s_we4 = '1' and s_wp4 = '0' then lbV0(to_integer(s_wx4(10 downto 1))) <= std_logic_vector(s_av3(9 downto 2)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addr(10 downto 1)));
            if s_we4 = '1' and s_wp4 = '1' then lbV1(to_integer(s_wx4(10 downto 1))) <= std_logic_vector(s_av3(9 downto 2)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Stages 6..19: fetch, key, blend, grout, gate
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_a : signed(12 downto 0);
        variable v_y, v_u, v_v : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 6 ctx
            s_a_bg <= s_bg; s_a_rbank <= s_rbank;
            s_grd(6) <= s_gr5;
            for i in 7 to 14 loop s_grd(i) <= s_grd(i - 1); end loop;
            -- stage 7: mosaic fetch
            if s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            s_dry_y <= unsigned(s_ring_qy); s_dry_u <= unsigned(s_ring_qu); s_dry_v <= unsigned(s_ring_qv);
            if s_sw_coarse = '1' then
                s_my(7) <= s_dry_y;
            else
                s_my(7) <= s_wet_y;
            end if;
            s_mu(7) <= s_wet_u; s_mv(7) <= s_wet_v;
            s_oy(7) <= s_dry_y; s_ou(7) <= s_dry_u; s_ov(7) <= s_dry_v;
            for i in 8 to 15 loop
                s_my(i) <= s_my(i - 1); s_mu(i) <= s_mu(i - 1); s_mv(i) <= s_mv(i - 1);
                s_oy(i) <= s_oy(i - 1); s_ou(i) <= s_ou(i - 1); s_ov(i) <= s_ov(i - 1);
            end loop;
            -- stage 8: key source colour
            if s_sw_keymos = '1' then
                s_ky8 <= s_wet_y; s_ku8 <= s_wet_u; s_kv8 <= s_wet_v;
            else
                s_ky8 <= s_dry_y; s_ku8 <= s_dry_u; s_kv8 <= s_dry_v;
            end if;
            -- stage 9: chroma centred
            s_du9 <= signed('0' & s_ku8) - to_signed(512, 11);
            s_dv9 <= signed('0' & s_kv8) - to_signed(512, 11);
            s_ky9 <= s_ky8;
            -- stage 10: saturation
            if s_du9 < 0 then s_sat10 <= resize(unsigned(resize(-s_du9, 10)), 11); else s_sat10 <= resize(unsigned(resize(s_du9, 10)), 11); end if;
            s_ky10 <= s_ky9;
            -- stage 11: (|du| + |dv|) done as two products with the blend weights
            s_vl11 <= s_ky10 * (to_unsigned(255, 8) - s_srcm);
            s_vs11 <= s_sat10(9 downto 0) * s_srcm;
            -- stage 12: value
            s_val12 <= resize(shift_right(s_vl11 + s_vs11, 8), 10);
            -- stage 13: difference from level
            s_dif13 <= signed(resize(s_val12, 12)) - signed(resize(s_level, 12));
            -- stage 14: soft product
            s_ap14 <= s_dif13 * signed('0' & s_soft);
            -- stage 15: alpha 0..255 (128 at level)
            v_a := resize(shift_right(s_ap14, 4), 13) + 128;
            if v_a < 0 then v_a := (others => '0'); elsif v_a > 255 then v_a := to_signed(255, 13); end if;
            if s_sw_inv = '1' then
                s_alpha15 <= not unsigned(v_a(7 downto 0));
            else
                s_alpha15 <= unsigned(v_a(7 downto 0));
            end if;
            -- stage 16: differences (reveal - mosaic)
            if s_sw_neg = '1' then
                s_ey16 <= signed('0' & (not s_oy(15))) - signed('0' & s_my(15));
                s_eu16 <= signed('0' & (not s_ou(15))) - signed('0' & s_mu(15));
                s_ev16 <= signed('0' & (not s_ov(15))) - signed('0' & s_mv(15));
            else
                s_ey16 <= signed('0' & s_oy(15)) - signed('0' & s_my(15));
                s_eu16 <= signed('0' & s_ou(15)) - signed('0' & s_mu(15));
                s_ev16 <= signed('0' & s_ov(15)) - signed('0' & s_mv(15));
            end if;
            s_ap16 <= s_alpha15;
            s_my16 <= s_my(15); s_mu16 <= s_mu(15); s_mv16 <= s_mv(15);
            -- stage 17: products
            s_py17 <= s_ey16 * signed('0' & s_ap16);
            s_pu17 <= s_eu16 * signed('0' & s_ap16);
            s_pv17 <= s_ev16 * signed('0' & s_ap16);
            s_my17 <= s_my16; s_mu17 <= s_mu16; s_mv17 <= s_mv16;
            s_ap17 <= s_ap16;
            -- stage 18: blend, tint, grout
            v_y := signed(resize(s_my17, 13)) + resize(shift_right(s_py17, 8), 13);
            v_u := signed(resize(s_mu17, 13)) + resize(shift_right(s_pu17, 8), 13);
            v_v := signed(resize(s_mv17, 13)) + resize(shift_right(s_pv17, 8), 13);
            -- tint: masked (alpha low = mosaic side) cells pushed by tint
            if s_ap17 < 128 then
                v_u := v_u + signed(resize(s_tint, 13));
                v_v := v_v - signed(resize(s_tint(7 downto 1), 13));
            end if;
            if s_grd(14) = '1' and (s_sw_groutin = '0' or s_ap17 < 128) then
                v_y := v_y - signed(resize(s_grout & "0", 13));
            end if;
            s_y18 <= f_clamp10(v_y); s_u18 <= f_clamp10(v_u); s_v18 <= f_clamp10(v_v);
            -- stage 19: gate
            if s_avid_sr(17) = '1' then
                s_y19 <= s_y18; s_u19 <= s_u18; s_v19 <= s_v18;
            else
                s_y19 <= to_unsigned(64, 10); s_u19 <= to_unsigned(512, 10); s_v19 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y19);
    data_out.u       <= std_logic_vector(s_u19);
    data_out.v       <= std_logic_vector(s_v19);
    data_out.hsync_n <= s_hsync_sr(18);
    data_out.vsync_n <= s_vsync_sr(18);
    data_out.field_n <= s_field_sr(18);
    data_out.avid    <= s_avid_sr(18);

end architecture stencil;
