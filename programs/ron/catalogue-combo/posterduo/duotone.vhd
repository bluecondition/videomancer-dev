-- duotone: two-ink tone mapping, 7-clock latency (matches solar_fold).
--   t = clamp((y - 64 - bal) * con / 64 + 512, 0..1023)  (0 = shadow ink, 1023 = light ink)
--   chroma = shadow + (light - shadow) * t;  tritone pulls the midtones to neutral.
--   flat: luma replaced by the ramp t (scaled to 64..940).  enable = '0' passes the input through.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity duotone is
    port (
        clk    : in  std_logic;
        y_in   : in  unsigned(9 downto 0);
        u_in   : in  unsigned(9 downto 0);
        v_in   : in  unsigned(9 downto 0);
        su, sv, lu, lv : in signed(8 downto 0);
        con    : in  unsigned(7 downto 0);
        bal    : in  unsigned(9 downto 0);
        tri    : in  std_logic;
        flat   : in  std_logic;
        enable : in  std_logic;
        y_out  : out unsigned(9 downto 0);
        u_out  : out unsigned(9 downto 0);
        v_out  : out unsigned(9 downto 0)
    );
end entity duotone;

architecture rtl of duotone is
    type t_y is array (1 to 7) of unsigned(9 downto 0);
    signal s_y, s_u, s_v : t_y := (others => (others => '0'));
    signal s_d1 : signed(11 downto 0) := (others => '0');
    signal s_p2 : signed(20 downto 0) := (others => '0');
    signal s_t3 : unsigned(9 downto 0) := (others => '0');
    signal s_du3, s_dv3 : signed(9 downto 0) := (others => '0');
    signal s_pu4, s_pv4 : signed(20 downto 0) := (others => '0');
    signal s_t4 : unsigned(9 downto 0) := (others => '0');
    signal s_uc5, s_vc5 : signed(10 downto 0) := (others => '0');
    signal s_w5 : unsigned(9 downto 0) := (others => '0');
    signal s_t5 : unsigned(9 downto 0) := (others => '0');
    signal s_pw6u, s_pw6v : signed(21 downto 0) := (others => '0');
    signal s_uc6, s_vc6 : signed(10 downto 0) := (others => '0');
    signal s_t6 : unsigned(9 downto 0) := (others => '0');
    function f_clamp10(v : signed) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 10);
        elsif v > 1023 then return to_unsigned(1023, 10);
        else return unsigned(v(9 downto 0)); end if;
    end function;
begin
    process(clk)
        variable v_t : signed(13 downto 0);
        variable v_w : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            s_y(1) <= y_in; s_u(1) <= u_in; s_v(1) <= v_in;
            for i in 2 to 7 loop s_y(i) <= s_y(i - 1); s_u(i) <= s_u(i - 1); s_v(i) <= s_v(i - 1); end loop;
            -- 1
            s_d1 <= signed(resize(y_in, 12)) - 64 - signed(resize(bal, 12)) + 512;
            -- 2
            s_p2 <= s_d1 * signed(resize(con, 9));
            -- 3: ramp t
            v_t := resize(shift_right(s_p2, 6), 14) + 512;
            s_t3 <= f_clamp10(v_t);
            s_du3 <= resize(lu, 10) - resize(su, 10);
            s_dv3 <= resize(lv, 10) - resize(sv, 10);
            -- 4
            s_pu4 <= s_du3 * signed(resize(s_t3, 11));
            s_pv4 <= s_dv3 * signed(resize(s_t3, 11));
            s_t4 <= s_t3;
            -- 5: ink chroma; tritone weight = distance from the midpoint
            s_uc5 <= resize(su, 11) + resize(shift_right(s_pu4, 10), 11);
            s_vc5 <= resize(sv, 11) + resize(shift_right(s_pv4, 10), 11);
            if s_t4 >= 512 then v_w := signed(resize(s_t4, 11)) - 512; else v_w := 512 - signed(resize(s_t4, 11)); end if;
            if tri = '0' then s_w5 <= to_unsigned(1023, 10);
            elsif v_w >= 511 then s_w5 <= to_unsigned(1023, 10);
            else s_w5 <= shift_left(unsigned(v_w(9 downto 0)), 1); end if;
            s_t5 <= s_t4;
            -- 6
            s_pw6u <= s_uc5 * signed(resize(s_w5, 11));
            s_pw6v <= s_vc5 * signed(resize(s_w5, 11));
            s_uc6 <= s_uc5; s_vc6 <= s_vc5; s_t6 <= s_t5;
            -- 7: outputs
            if enable = '1' then
                u_out <= f_clamp10(resize(shift_right(s_pw6u, 10), 12) + 512);
                v_out <= f_clamp10(resize(shift_right(s_pw6v, 10), 12) + 512);
                if flat = '1' then y_out <= resize(shift_right(resize(s_t6, 11) * 7, 3), 10) + 64; else y_out <= s_y(6); end if;
            else
                y_out <= s_y(6); u_out <= s_u(6); v_out <= s_v(6);
            end if;
        end if;
    end process;
end architecture rtl;
