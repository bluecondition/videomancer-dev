-- Cartridge: catalogue combo #15 (seed 20260914) -- cartoon / cel (edge +
-- flat colour) + palette quantization (Gameboy / CGA / C64 / EGA).  Three
-- knobs each; the switches and the slider decide how they combine.
--
--   K1 Edge      cel edge threshold (lower = more ink)
--   K2 Levels    posterize depth 8 / 6 / 4 / 3 / 2 bits
--   K3 Width     edge tap spacing 1 / 2 / 4 / 8 px (line weight)
--   K4 Palette   Gameboy / CGA / C64 / EGA
--   K5 Dither    4x4 Bayer dither amplitude before the quantizer
--   K6 Range     luma contrast into the quantizer
--
--   S7  Ink      edges black / the palette's darkest entry
--   S8  Order    posterize then quantize / quantize then posterize
--   S9  Edges    edges keep the original colour
--   S10 Luma     luma from the palette / from the cel (palette chroma only)
--   S11 Invert   cel luma inverted before quantizing
--   P12 Inking   thickens the ink and lowers the edge threshold together
--
-- Pure per-pixel plus the EBR quantizer LUT (free-running builder).
-- Latency 16 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture cartridge of program_top is

    constant C_LAT : integer := 16;

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
    function f_absd8(a, b : unsigned(7 downto 0)) return unsigned is
    begin
        if a >= b then return a - b; else return b - a; end if;
    end function;

    -- Palette ROM: 4 zones x 16 entries, 8-bit y,u,v packed as y&u&v (24 bits)
    type t_pal_rom is array (0 to 63) of std_logic_vector(31 downto 0);
    constant C_PAL_ROM : t_pal_rom := (
        x"00317174",
        x"00526D71",
        x"008B7C40",
        x"00987D39",
        x"00317174",
        x"00526D71",
        x"008B7C40",
        x"00987D39",
        x"00317174",
        x"00526D71",
        x"008B7C40",
        x"00987D39",
        x"00317174",
        x"00526D71",
        x"008B7C40",
        x"00987D39",
        x"00008080",
        x"00CD2A9D",
        x"009CC7B8",
        x"00FF8080",
        x"00008080",
        x"00CD2A9D",
        x"009CC7B8",
        x"00FF8080",
        x"00008080",
        x"00CD2A9D",
        x"009CC7B8",
        x"00FF8080",
        x"00008080",
        x"00CD2A9D",
        x"009CC7B8",
        x"00FF8080",
        x"00108080",
        x"00EB8080",
        x"0050A276",
        x"00A9598A",
        x"00619D9B",
        x"00846163",
        x"004179B0",
        x"00D18748",
        x"00609E65",
        x"00429063",
        x"0083A675",
        x"004F8080",
        x"00798080",
        x"00CB6162",
        x"007A78B6",
        x"00A88080",
        x"00108080",
        x"002073CA",
        x"0065414E",
        x"00763599",
        x"003BCA66",
        x"004CBEB1",
        x"0066AB4E",
        x"00A28080",
        x"00598080",
        x"006973CA",
        x"00AE414E",
        x"00BF3599",
        x"0084CA66",
        x"0095BEB1",
        x"00DA8C35",
        x"00EB8080"
    );

    -- Pixel-path copies of the palette (16-bit words -> EBR)
    type t_pal16 is array (0 to 63) of std_logic_vector(15 downto 0);
    constant C_PAL_YU : t_pal16 := (
        x"3171",
        x"526D",
        x"8B7C",
        x"987D",
        x"3171",
        x"526D",
        x"8B7C",
        x"987D",
        x"3171",
        x"526D",
        x"8B7C",
        x"987D",
        x"3171",
        x"526D",
        x"8B7C",
        x"987D",
        x"0080",
        x"CD2A",
        x"9CC7",
        x"FF80",
        x"0080",
        x"CD2A",
        x"9CC7",
        x"FF80",
        x"0080",
        x"CD2A",
        x"9CC7",
        x"FF80",
        x"0080",
        x"CD2A",
        x"9CC7",
        x"FF80",
        x"1080",
        x"EB80",
        x"50A2",
        x"A959",
        x"619D",
        x"8461",
        x"4179",
        x"D187",
        x"609E",
        x"4290",
        x"83A6",
        x"4F80",
        x"7980",
        x"CB61",
        x"7A78",
        x"A880",
        x"1080",
        x"2073",
        x"6541",
        x"7635",
        x"3BCA",
        x"4CBE",
        x"66AB",
        x"A280",
        x"5980",
        x"6973",
        x"AE41",
        x"BF35",
        x"84CA",
        x"95BE",
        x"DA8C",
        x"EB80"
    );
    constant C_PAL_V : t_pal16 := (
        x"0074",
        x"0071",
        x"0040",
        x"0039",
        x"0074",
        x"0071",
        x"0040",
        x"0039",
        x"0074",
        x"0071",
        x"0040",
        x"0039",
        x"0074",
        x"0071",
        x"0040",
        x"0039",
        x"0080",
        x"009D",
        x"00B8",
        x"0080",
        x"0080",
        x"009D",
        x"00B8",
        x"0080",
        x"0080",
        x"009D",
        x"00B8",
        x"0080",
        x"0080",
        x"009D",
        x"00B8",
        x"0080",
        x"0080",
        x"0080",
        x"0076",
        x"008A",
        x"009B",
        x"0063",
        x"00B0",
        x"0048",
        x"0065",
        x"0063",
        x"0075",
        x"0080",
        x"0080",
        x"0062",
        x"00B6",
        x"0080",
        x"0080",
        x"00CA",
        x"004E",
        x"0099",
        x"0066",
        x"00B1",
        x"004E",
        x"0080",
        x"0080",
        x"00CA",
        x"004E",
        x"0099",
        x"0066",
        x"00B1",
        x"0035",
        x"0080"
    );

    type t_bayer is array (0 to 15) of integer range 0 to 15;
    constant C_BAYER : t_bayer := (0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5);

    type t_qlut  is array (0 to 1023) of std_logic_vector(3 downto 0);
    signal qlut     : t_qlut := (others => (others => '0'));
    signal s_lut_q  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_pyu_q, s_pv_q : std_logic_vector(15 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Controls / per-frame
    --------------------------------------------------------------------------
    signal s_k_edge, s_k_levels, s_k_width, s_k_pal, s_k_dither, s_k_range : unsigned(9 downto 0);
    signal s_sw_inkpal, s_sw_order, s_sw_keep, s_sw_lumacel, s_sw_inv : std_logic;
    signal s_p_ink : unsigned(9 downto 0);
    signal s_zone   : unsigned(1 downto 0) := (others => '0');
    signal s_thr    : unsigned(9 downto 0) := (others => '0');
    signal s_wsel   : unsigned(1 downto 0) := (others => '0');
    signal s_ymask, s_yhalf, s_cmask, s_chalf : unsigned(9 downto 0) := (others => '0');
    signal s_dfrac  : unsigned(7 downto 0) := (others => '0');
    signal s_range  : unsigned(7 downto 0) := to_unsigned(64, 8);
    signal s_prev_hsync_n, s_prev_vsync_n : std_logic := '1';
    signal s_hcx    : unsigned(1 downto 0) := (others => '0');
    signal s_hcy    : unsigned(1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Palette LUT builder
    --------------------------------------------------------------------------
    signal s_pb_st  : unsigned(2 downto 0) := (others => '0');
    signal s_pb_e   : unsigned(9 downto 0) := (others => '0');
    signal s_pb_i   : unsigned(3 downto 0) := (others => '0');
    signal s_pb_rom : std_logic_vector(31 downto 0) := (others => '0');
    signal s_pb_ady, s_pb_adu, s_pb_adv : unsigned(7 downto 0) := (others => '0');
    signal s_pb_d   : unsigned(9 downto 0) := (others => '0');
    signal s_pb_min : unsigned(9 downto 0) := (others => '1');
    signal s_pb_idx : unsigned(3 downto 0) := (others => '0');
    signal s_pb_we  : std_logic := '0';
    signal s_pb_wa  : unsigned(9 downto 0) := (others => '0');
    signal s_pb_wd  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_dark_idx : unsigned(3 downto 0) := (others => '0');
    signal s_dark_y   : unsigned(7 downto 0) := (others => '1');

    --------------------------------------------------------------------------
    -- Pixel chain
    --------------------------------------------------------------------------
    type t_taps is array (0 to 16) of unsigned(9 downto 0);
    signal s_t, s_cu, s_cv : t_taps := (others => (others => '0'));
    signal s_d2    : signed(10 downto 0) := (others => '0');
    signal s_y2, s_u2, s_v2 : unsigned(9 downto 0) := (others => '0');
    signal s_edge3 : std_logic := '0';
    signal s_y3, s_u3, s_v3 : unsigned(9 downto 0) := (others => '0');
    signal s_y4, s_u4, s_v4 : unsigned(9 downto 0) := (others => '0');
    signal s_ym4a : signed(10 downto 0) := (others => '0');
    signal s_rp4b : signed(19 downto 0) := (others => '0');
    signal s_u4a, s_v4a, s_u4b, s_v4b : unsigned(9 downto 0) := (others => '0');
    signal s_doff5 : signed(12 downto 0) := (others => '0');
    signal s_y5, s_u5, s_v5 : unsigned(9 downto 0) := (others => '0');
    signal s_y6, s_u6, s_v6 : unsigned(9 downto 0) := (others => '0');
    signal s_idx8  : unsigned(3 downto 0) := (others => '0');
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := (others => '0');
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := (others => '0');
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := (others => '0');
    signal s_y13, s_u13, s_v13 : unsigned(9 downto 0) := (others => '0');
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');
    type t_y is array (4 to 14) of unsigned(9 downto 0);
    signal s_oy, s_ou, s_ov, s_cy : t_y := (others => (others => '0'));
    type t_b is array (3 to 14) of std_logic;
    signal s_ed : t_b := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_edge   <= unsigned(registers_in(0));
    s_k_levels <= unsigned(registers_in(1));
    s_k_width  <= unsigned(registers_in(2));
    s_k_pal    <= unsigned(registers_in(3));
    s_k_dither <= unsigned(registers_in(4));
    s_k_range  <= unsigned(registers_in(5));
    s_sw_inkpal  <= registers_in(6)(0);
    s_sw_order   <= registers_in(6)(1);
    s_sw_keep    <= registers_in(6)(2);
    s_sw_lumacel <= registers_in(6)(3);
    s_sw_inv     <= registers_in(6)(4);
    s_p_ink    <= unsigned(registers_in(7));

    --------------------------------------------------------------------------
    -- ROM / LUT reads
    --------------------------------------------------------------------------
    p_mem : process(clk)
        variable v_la : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            v_la := s_y6(9 downto 6) & s_u6(9 downto 7) & s_v6(9 downto 7);
            s_lut_q <= qlut(to_integer(v_la));
            if s_pb_we = '1' then
                qlut(to_integer(s_pb_wa)) <= s_pb_wd;
            end if;
            s_pyu_q <= C_PAL_YU(to_integer(s_zone & s_idx8));
            s_pv_q  <= C_PAL_V(to_integer(s_zone & s_idx8));
        end if;
    end process;

    p_pal_build : process(clk)
        variable v_yq, v_uq, v_vq : unsigned(7 downto 0);
    begin
        if rising_edge(clk) then
            s_pb_we <= '0';
            v_yq := s_pb_e(9 downto 6) & "1000";
            v_uq := s_pb_e(5 downto 3) & "10000";
            v_vq := s_pb_e(2 downto 0) & "10000";
            case to_integer(s_pb_st) is
                when 0 =>
                    s_pb_rom <= C_PAL_ROM(to_integer(s_zone & s_pb_i));
                    s_pb_st  <= s_pb_st + 1;
                when 1 =>
                    s_pb_ady <= f_absd8(unsigned(s_pb_rom(23 downto 16)), v_yq);
                    s_pb_adu <= f_absd8(unsigned(s_pb_rom(15 downto 8)), v_uq);
                    s_pb_adv <= f_absd8(unsigned(s_pb_rom(7 downto 0)), v_vq);
                    s_pb_st  <= s_pb_st + 1;
                when 2 =>
                    s_pb_d  <= shift_left(resize(s_pb_ady, 10), 1) + resize(s_pb_adu, 10) + resize(s_pb_adv, 10);
                    s_pb_st <= s_pb_st + 1;
                when 3 =>
                    if s_pb_d < s_pb_min then
                        s_pb_min <= s_pb_d;
                        s_pb_idx <= s_pb_i;
                    end if;
                    if s_pb_i = 15 then
                        s_pb_st <= s_pb_st + 1;
                    else
                        s_pb_i  <= s_pb_i + 1;
                        s_pb_st <= (others => '0');
                    end if;
                when others =>
                    s_pb_we  <= '1';
                    s_pb_wa  <= s_pb_e;
                    s_pb_wd  <= std_logic_vector(s_pb_idx);
                    s_pb_e   <= s_pb_e + 1;
                    s_pb_i   <= (others => '0');
                    s_pb_min <= (others => '1');
                    s_pb_st  <= (others => '0');
            end case;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Per-frame decode
    --------------------------------------------------------------------------
    p_frame : process(clk)
        variable v_pb : integer range 1 to 10;
        variable v_m, v_h : unsigned(9 downto 0);
        variable v_t : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_zone  <= s_k_pal(9 downto 8);
            v_t := signed(resize(s_k_edge, 12)) - signed(resize(s_p_ink(9 downto 1), 12));
            if v_t < 8 then v_t := to_signed(8, 12); end if;
            s_thr   <= unsigned(v_t(9 downto 0));
            if s_p_ink(9 downto 8) = "11" then
                s_wsel <= "11";
            elsif s_k_width(9 downto 8) + s_p_ink(9 downto 8) > 3 then
                s_wsel <= "11";
            else
                s_wsel <= resize(s_k_width(9 downto 8) + s_p_ink(9 downto 8), 2);
            end if;
            case to_integer(s_k_levels(9 downto 7)) is
                when 0 => v_pb := 8;
                when 1 | 2 => v_pb := 6;
                when 3 | 4 => v_pb := 4;
                when 5 | 6 => v_pb := 3;
                when others => v_pb := 2;
            end case;
            v_m := (others => '0'); v_h := (others => '0');
            for i in 0 to 9 loop
                if i >= 10 - v_pb then v_m(i) := '1'; end if;
            end loop;
            v_h(9 - v_pb) := '1';
            s_ymask <= v_m; s_yhalf <= v_h; s_cmask <= v_m; s_chalf <= v_h;
            s_dfrac <= s_k_dither(9 downto 2);
            s_range <= s_k_range(9 downto 2);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel pipeline
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_l, v_r : unsigned(9 downto 0);
        variable v_bay : integer range 0 to 15;
        variable v_b : signed(5 downto 0);
        variable v_y, v_u, v_v : signed(12 downto 0);
        variable v_yq : unsigned(9 downto 0);
        variable v_rp : signed(19 downto 0);
    begin
        if rising_edge(clk) then
            -- stage 1: taps (centre = t(8))
            s_t(0) <= unsigned(data_in.y); s_cu(0) <= unsigned(data_in.u); s_cv(0) <= unsigned(data_in.v);
            for i in 1 to 16 loop
                s_t(i) <= s_t(i - 1); s_cu(i) <= s_cu(i - 1); s_cv(i) <= s_cv(i - 1);
            end loop;
            -- stage 2: gradient across the centre with selectable spacing
            case to_integer(s_wsel) is
                when 0 => v_l := s_t(9); v_r := s_t(7);
                when 1 => v_l := s_t(10); v_r := s_t(6);
                when 2 => v_l := s_t(12); v_r := s_t(4);
                when others => v_l := s_t(16); v_r := s_t(0);
            end case;
            s_d2 <= signed('0' & v_r) - signed('0' & v_l);
            s_y2 <= s_t(8); s_u2 <= s_cu(8); s_v2 <= s_cv(8);
            -- stage 3: edge flag; posterize (cel) if order = posterize first
            if s_d2 > signed('0' & s_thr) or s_d2 < -signed('0' & s_thr) then s_ed(3) <= '1'; else s_ed(3) <= '0'; end if;
            if s_sw_inv = '1' then v_yq := not s_y2; else v_yq := s_y2; end if;
            if s_sw_order = '0' then
                s_y3 <= (v_yq and s_ymask) or s_yhalf;
                s_u3 <= (s_u2 and s_cmask) or s_chalf;
                s_v3 <= (s_v2 and s_cmask) or s_chalf;
            else
                s_y3 <= v_yq; s_u3 <= s_u2; s_v3 <= s_v2;
            end if;
            s_oy(4) <= s_y2; s_ou(4) <= s_u2; s_ov(4) <= s_v2;
            -- stage 4a/4b/4c: range (contrast into the quantizer): y = 64 + (y-64)*range/64
            s_ym4a <= signed('0' & s_y3) - 64;
            s_u4a <= s_u3; s_v4a <= s_v3; s_cy(4) <= s_y3;
            s_rp4b <= s_ym4a * signed('0' & s_range);
            s_u4b <= s_u4a; s_v4b <= s_v4a;
            s_y4 <= f_clamp10(resize(shift_right(s_rp4b, 6), 13) + 64);
            s_u4 <= s_u4b; s_v4 <= s_v4b;
            -- stage 5: dither offset (Bayer counters on stage-4 avid / hsync)
            if s_avid_sr(4) = '0' then s_hcx <= (others => '0'); else s_hcx <= s_hcx + 1; end if;
            if s_vsync_sr(4) = '0' then
                s_hcy <= (others => '0');
            elsif s_hsync_sr(4) = '0' and s_hsync_sr(5) = '1' then
                s_hcy <= s_hcy + 1;
            end if;
            v_bay := C_BAYER(to_integer(s_hcy & s_hcx));
            v_b := to_signed(v_bay - 8, 6);
            s_doff5 <= resize(v_b * signed('0' & s_dfrac), 13);
            s_y5 <= s_y4; s_u5 <= s_u4; s_v5 <= s_v4;
            -- stage 6: dither apply -> LUT address source
            s_y6 <= f_clamp10(signed(resize(s_y5, 13)) + shift_right(s_doff5, 3));
            s_u6 <= f_clamp10(signed(resize(s_u5, 13)) + shift_right(s_doff5, 4));
            s_v6 <= f_clamp10(signed(resize(s_v5, 13)) + shift_right(s_doff5, 4));
            -- stage 7: LUT read (p_mem); stage 8: index copy
            s_idx8 <= unsigned(s_lut_q);
            -- stage 9: palette ROM read (p_mem); stage 10: colour
            s_y10 <= unsigned(s_pyu_q(15 downto 8)) & "00";
            s_u10 <= unsigned(s_pyu_q(7 downto 0)) & "00";
            s_v10 <= unsigned(s_pv_q(7 downto 0)) & "00";
            -- stage 11: order = quantize first -> posterize now; luma choice
            if s_sw_order = '1' then
                s_y11 <= (s_y10 and s_ymask) or s_yhalf;
                s_u11 <= (s_u10 and s_cmask) or s_chalf;
                s_v11 <= (s_v10 and s_cmask) or s_chalf;
            else
                s_y11 <= s_y10; s_u11 <= s_u10; s_v11 <= s_v10;
            end if;
            -- stage 12: luma from cel option
            if s_sw_lumacel = '1' then
                s_y12 <= s_cy(13);
            else
                s_y12 <= s_y11;
            end if;
            s_u12 <= s_u11; s_v12 <= s_v11;
            -- stage 13: edges: ink (black / darkest palette entry) or original colour
            if s_ed(14) = '1' then
                if s_sw_keep = '1' then
                    s_y13 <= s_oy(14); s_u13 <= s_ou(14); s_v13 <= s_ov(14);
                elsif s_sw_inkpal = '1' then
                    s_y13 <= s_dark_y & "00"; s_u13 <= s_u12; s_v13 <= s_v12;
                else
                    s_y13 <= to_unsigned(64, 10); s_u13 <= to_unsigned(512, 10); s_v13 <= to_unsigned(512, 10);
                end if;
            else
                s_y13 <= s_y12; s_u13 <= s_u12; s_v13 <= s_v12;
            end if;
            -- stage 14: gate
            if s_avid_sr(14) = '1' then
                s_y14 <= s_y13; s_u14 <= s_u13; s_v14 <= s_v13;
            else
                s_y14 <= to_unsigned(64, 10); s_u14 <= to_unsigned(512, 10); s_v14 <= to_unsigned(512, 10);
            end if;
            -- chains
            for i in 5 to 14 loop
                s_oy(i) <= s_oy(i - 1); s_ou(i) <= s_ou(i - 1); s_ov(i) <= s_ov(i - 1); s_cy(i) <= s_cy(i - 1);
            end loop;
            for i in 4 to 14 loop s_ed(i) <= s_ed(i - 1); end loop;
            -- sync chain
            s_hsync_sr(0) <= data_in.hsync_n;
            s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n;
            s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1);
                s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1);
                s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_pix;

    -- darkest palette entry tracker (rides the builder's ROM reads)
    p_dark : process(clk)
    begin
        if rising_edge(clk) then
            if s_pb_st = 1 then
                if s_pb_i = 0 then
                    s_dark_y <= unsigned(s_pb_rom(23 downto 16));
                elsif unsigned(s_pb_rom(23 downto 16)) < s_dark_y then
                    s_dark_y <= unsigned(s_pb_rom(23 downto 16));
                end if;
            end if;
        end if;
    end process;

    data_out.y       <= std_logic_vector(s_y14);
    data_out.u       <= std_logic_vector(s_u14);
    data_out.v       <= std_logic_vector(s_v14);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture cartridge;
