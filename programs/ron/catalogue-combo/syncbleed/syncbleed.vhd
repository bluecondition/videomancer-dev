-- Syncbleed: catalogue combo #131 (seed 20260927) -- colour bleeding / NTSC
-- dot-crawl emulation + sync loss / horizontal collapse.  Three knobs
-- each; the switches and the slider decide how they combine.
--
--   K1 Bleed    chroma smear length (IIR, 0 .. 128 px)
--   K2 Lag      chroma lags luma by 0 .. 63 px
--   K3 Crawl    dot-crawl amount (luma dots, strongest on colour)
--   K4 Collapse base squeeze of every row toward the seam
--   K5 Seam     collapse centre X, bipolar (centre = screen centre)
--   K6 Wave     the squeeze breathes down the picture with a slow rolling wave
--
--   S7  Bleed   before the collapse (smear squeezed too) / after (screen-width smear)
--   S8  Length  fixed / scales with the local collapse (squeezed rows bleed more)
--   S9  Chroma  collapses with luma / collapses more (colour fringes)
--   S10 Crawl   locked checkerboard / drifting (inverts every frame)
--   S11 Smear   chroma only / luma too (ghosting)
--   P12 Loss    signal loss: collapse, bleed and crawl climb to a total-collapse smear
--
-- Per-line squeeze q = base + sine wave; luma and chroma run their own
-- DDA read addresses (chroma with its own gain and a lag), both edge-filled;
-- chroma IIR either on the write port or after the fetch; crawl adds
-- saturation-scaled luma dots.  Latency 14 clocks, all modes, blanking-gated.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.all;

architecture syncbleed of program_top is

    constant C_LAT  : integer := 14;
    constant C_EDGE : integer := 12;

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

    -- Edge clamp: reads off the line or inside its C_EDGE blanking columns take
    -- the line's own edge pixel.  Both compares run on the source in parallel.
    function f_edge_addr(v : signed(18 downto 0); we : signed(18 downto 0); we11 : unsigned(10 downto 0)) return unsigned is
    begin
        if v < C_EDGE then
            return to_unsigned(C_EDGE, 11);
        elsif v > we then
            return we11;
        else
            return unsigned(v(10 downto 0));
        end if;
    end function;

    type t_sine_rom is array (0 to 255) of integer range -256 to 255;
    constant C_SINE : t_sine_rom := (
        -255, -255, -255, -254, -254, -253, -252, -251,
        -250, -249, -247, -246, -244, -242, -240, -238,
        -236, -233, -231, -228, -225, -222, -219, -215,
        -212, -208, -205, -201, -197, -193, -189, -185,
        -180, -176, -171, -167, -162, -157, -152, -147,
        -142, -136, -131, -126, -120, -115, -109, -103,
         -98,  -92,  -86,  -80,  -74,  -68,  -62,  -56,
         -50,  -44,  -37,  -31,  -25,  -19,  -13,   -6,
           0,    6,   13,   19,   25,   31,   37,   44,
          50,   56,   62,   68,   74,   80,   86,   92,
          98,  103,  109,  115,  120,  126,  131,  136,
         142,  147,  152,  157,  162,  167,  171,  176,
         180,  185,  189,  193,  197,  201,  205,  208,
         212,  215,  219,  222,  225,  228,  231,  233,
         236,  238,  240,  242,  244,  246,  247,  249,
         250,  251,  252,  253,  254,  254,  255,  255,
         255,  255,  255,  254,  254,  253,  252,  251,
         250,  249,  247,  246,  244,  242,  240,  238,
         236,  233,  231,  228,  225,  222,  219,  215,
         212,  208,  205,  201,  197,  193,  189,  185,
         180,  176,  171,  167,  162,  157,  152,  147,
         142,  136,  131,  126,  120,  115,  109,  103,
          98,   92,   86,   80,   74,   68,   62,   56,
          50,   44,   37,   31,   25,   19,   13,    6,
           0,   -6,  -13,  -19,  -25,  -31,  -37,  -44,
         -50,  -56,  -62,  -68,  -74,  -80,  -86,  -92,
         -98, -103, -109, -115, -120, -126, -131, -136,
        -142, -147, -152, -157, -162, -167, -171, -176,
        -180, -185, -189, -193, -197, -201, -205, -208,
        -212, -215, -219, -222, -225, -228, -231, -233,
        -236, -238, -240, -242, -244, -246, -247, -249,
        -250, -251, -252, -253, -254, -254, -255, -255
    );

    type t_lb_y is array (0 to 2047) of std_logic_vector(9 downto 0);
    type t_lb_c is array (0 to 1023) of std_logic_vector(7 downto 0);
    signal lbY0, lbY1 : t_lb_y := (others => "0001000000");
    signal lbU0, lbU1 : t_lb_c := (others => x"80");
    signal lbV0, lbV1 : t_lb_c := (others => x"80");
    signal s_rdY0, s_rdY1 : std_logic_vector(9 downto 0) := (others => '0');
    signal s_rdU0, s_rdU1 : std_logic_vector(7 downto 0) := x"80";
    signal s_rdV0, s_rdV1 : std_logic_vector(7 downto 0) := x"80";

    -- controls
    signal s_k_bleed, s_k_lag, s_k_crawl, s_k_col, s_k_seam, s_k_wave : unsigned(9 downto 0);
    signal s_sw_after, s_sw_local, s_sw_fringe, s_sw_drift, s_sw_lumasmear : std_logic;
    signal s_p_loss : unsigned(9 downto 0);

    -- per frame
    signal s_col8, s_wamp, s_loss8, s_cg : unsigned(7 downto 0) := (others => '0');
    signal s_cgp    : unsigned(8 downto 0) := (others => '0');
    signal s_lag    : unsigned(5 downto 0) := (others => '0');
    signal s_bn     : unsigned(2 downto 0) := (others => '0');
    signal s_bnl    : unsigned(1 downto 0) := (others => '0');
    signal s_frame  : std_logic := '0';
    signal s_seam_p : unsigned(20 downto 0) := (others => '0');
    signal s_qb     : unsigned(13 downto 0) := to_unsigned(256, 14);
    signal s_seam   : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_seam256 : unsigned(18 downto 0) := (others => '0');
    signal s_lag256 : unsigned(13 downto 0) := (others => '0');
    signal s_fphase, s_line_phase : unsigned(15 downto 0) := (others => '0');
    signal s_vstep  : unsigned(2 downto 0) := "111";
    signal s_saw_active : std_logic := '0';

    -- per line
    signal s_lstep  : unsigned(3 downto 0) := (others => '1');
    signal s_sin_addr : unsigned(7 downto 0) := (others => '0');
    signal s_sin_q, s_sin_q2 : signed(9 downto 0) := (others => '0');
    signal s_jp     : signed(18 downto 0) := (others => '0');
    signal s_q      : unsigned(13 downto 0) := to_unsigned(256, 14);
    signal s_qc     : unsigned(14 downto 0) := to_unsigned(256, 15);
    signal s_bn_row : unsigned(2 downto 0) := (others => '0');
    signal s_pq     : unsigned(24 downto 0) := (others => '0');
    signal s_pqc    : unsigned(25 downto 0) := (others => '0');
    -- free-running split products (seam x q hi/lo, seam x qc hi/lo) with kept seam copies
    signal s_seam_a, s_seam_b, s_seam_c, s_seam_d : unsigned(10 downto 0) := to_unsigned(240, 11);
    signal s_pqh    : unsigned(17 downto 0) := (others => '0');
    signal s_pql    : unsigned(17 downto 0) := (others => '0');
    signal s_pqch   : unsigned(18 downto 0) := (others => '0');
    signal s_pqcl   : unsigned(17 downto 0) := (others => '0');
    attribute keep : boolean;
    attribute keep of s_seam_a, s_seam_b, s_seam_c, s_seam_d : signal is true;
    signal s_src0, s_srcc0 : signed(26 downto 0) := (others => '0');

    -- tracking
    signal s_rx, s_wr_x : unsigned(10 downto 0) := (others => '0');
    signal s_in_y, s_in_u, s_in_v : unsigned(9 downto 0) := (others => '0');
    signal s_in_avid : std_logic := '0';
    signal s_prev_hsync_n, s_prev_vsync_n, s_prev_avid : std_logic := '1';
    signal s_line, s_line_width : unsigned(10 downto 0) := to_unsigned(270, 11);
    signal s_wpar, s_rbank : std_logic := '0';
    signal s_w_edge : unsigned(10 downto 0) := to_unsigned(200, 11);
    signal s_we19   : signed(18 downto 0) := to_signed(200, 19);

    -- write path (pre-bleed IIR, Q6)
    signal s_py, s_pu, s_pv : unsigned(15 downto 0) := to_unsigned(512 * 64, 16);
    signal s_wx2    : unsigned(10 downto 0) := (others => '0');
    signal s_we2, s_wp2 : std_logic := '0';

    -- address chain
    type t_b is array (1 to 4) of std_logic;
    signal s_avd    : t_b := (others => '0');
    signal s_accy, s_accc : signed(26 downto 0) := (others => '0');
    signal s_srcy4, s_srcc4 : signed(18 downto 0) := (others => '0');
    signal s_addrY, s_addrC : unsigned(10 downto 0) := (others => '0');

    -- pixel path
    signal s_a_rbank : std_logic := '0';
    signal s_wet_y, s_wet_u, s_wet_v : unsigned(9 downto 0) := (others => '0');
    signal s_qy, s_qu, s_qv : unsigned(15 downto 0) := to_unsigned(512 * 64, 16);
    signal s_y9, s_u9, s_v9 : unsigned(9 downto 0) := (others => '0');
    signal s_du9, s_dv9 : signed(10 downto 0) := (others => '0');
    signal s_y10, s_u10, s_v10 : unsigned(9 downto 0) := (others => '0');
    signal s_sat10  : unsigned(10 downto 0) := (others => '0');
    signal s_y11, s_u11, s_v11 : unsigned(9 downto 0) := (others => '0');
    signal s_sf11   : unsigned(9 downto 0) := (others => '0');
    signal s_y12, s_u12, s_v12 : unsigned(9 downto 0) := (others => '0');
    signal s_pc12   : unsigned(17 downto 0) := (others => '0');
    signal s_y13, s_u13, s_v13 : unsigned(9 downto 0) := (others => '0');
    signal s_cx, s_clp, s_prev_hs : std_logic := '0';
    signal s_y14, s_u14, s_v14 : unsigned(9 downto 0) := (others => '0');

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_field_sr : t_bit_shift := (others => '1');
    signal s_avid_sr  : t_bit_shift := (others => '0');

begin

    s_k_bleed <= unsigned(registers_in(0));
    s_k_lag   <= unsigned(registers_in(1));
    s_k_crawl <= unsigned(registers_in(2));
    s_k_col   <= unsigned(registers_in(3));
    s_k_seam  <= unsigned(registers_in(4));
    s_k_wave  <= unsigned(registers_in(5));
    s_sw_after     <= registers_in(6)(0);
    s_sw_local     <= registers_in(6)(1);
    s_sw_fringe    <= registers_in(6)(2);
    s_sw_drift     <= registers_in(6)(3);
    s_sw_lumasmear <= registers_in(6)(4);
    s_p_loss <= unsigned(registers_in(7));

    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            s_sin_q <= to_signed(C_SINE(to_integer(s_sin_addr)), 10);
            s_sin_q2 <= s_sin_q;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Control, per-frame / per-line sequencers, write path
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_q : signed(15 downto 0);
        variable v_n : unsigned(3 downto 0);
        variable v_s : integer range 0 to 7;
    begin
        if rising_edge(clk) then
            s_in_y <= unsigned(data_in.y); s_in_u <= unsigned(data_in.u); s_in_v <= unsigned(data_in.v);
            s_in_avid <= data_in.avid;
            s_prev_hsync_n <= data_in.hsync_n; s_prev_vsync_n <= data_in.vsync_n; s_prev_avid <= data_in.avid;
            if data_in.avid = '1' then s_rx <= s_rx + 1; s_saw_active <= '1'; else s_rx <= (others => '0'); end if;
            if s_in_avid = '1' then s_wr_x <= s_wr_x + 1; else s_wr_x <= (others => '0'); end if;

            -- stage 2: pre-bleed IIR on the write port (stage-1 inputs)
            v_s := to_integer(s_bn_row);
            if s_in_avid = '0' then
                s_py <= s_in_y & "000000"; s_pu <= s_in_u & "000000"; s_pv <= s_in_v & "000000";
            elsif s_sw_after = '0' then
                s_pu <= s_pu - shift_right(s_pu, v_s) + shift_right(s_in_u & "000000", v_s);
                s_pv <= s_pv - shift_right(s_pv, v_s) + shift_right(s_in_v & "000000", v_s);
                if s_sw_lumasmear = '1' then
                    s_py <= s_py - shift_right(s_py, v_s) + shift_right(s_in_y & "000000", v_s);
                else
                    s_py <= s_in_y & "000000";
                end if;
            else
                s_py <= s_in_y & "000000"; s_pu <= s_in_u & "000000"; s_pv <= s_in_v & "000000";
            end if;
            s_wx2 <= s_wr_x; s_we2 <= s_in_avid; s_wp2 <= s_wpar;

            s_w_edge <= s_line_width - (C_EDGE + 1);
            s_we19 <= signed(resize(s_w_edge, 19));

            -- per-frame decode
            case to_integer(s_vstep) is
                when 0 =>
                    s_col8 <= s_k_col(9 downto 2);
                    s_lag  <= s_k_lag(9 downto 4);
                    s_wamp <= s_k_wave(9 downto 2);
                    s_bn   <= s_k_bleed(9 downto 7);
                    s_bnl  <= s_p_loss(9 downto 8);
                    s_loss8 <= s_p_loss(9 downto 2);
                    s_cgp  <= resize(s_k_crawl(9 downto 2), 9) + resize(s_p_loss(9 downto 3), 9);
                    s_frame <= not s_frame;
                    s_vstep <= s_vstep + 1;
                when 1 =>
                    s_seam_p <= s_k_seam * s_line_width;
                    s_qb <= to_unsigned(256, 14) + resize(s_col8 & "000", 14) + resize(s_loss8 & "0000", 14);
                    s_vstep <= s_vstep + 1;
                when 2 =>
                    s_seam <= s_seam_p(20 downto 10);
                    if s_cgp > 255 then s_cg <= (others => '1'); else s_cg <= s_cgp(7 downto 0); end if;
                    s_fphase <= s_fphase + 300;
                    s_vstep <= s_vstep + 1;
                when 3 =>
                    s_seam256 <= s_seam & x"00";
                    s_lag256 <= s_lag & x"00";
                    s_vstep <= "111";
                when others => null;
            end case;

            -- free-running split products (operands registered per line / per frame)
            s_seam_a <= s_seam; s_seam_b <= s_seam; s_seam_c <= s_seam; s_seam_d <= s_seam;
            s_pqh  <= s_seam_a * s_q(13 downto 7);
            s_pql  <= s_seam_b * s_q(6 downto 0);
            s_pqch <= s_seam_c * s_qc(14 downto 7);
            s_pqcl <= s_seam_d * s_qc(6 downto 0);
            -- per-line: wave, squeeze factors, DDA starts
            case to_integer(s_lstep) is
                when 0 =>
                    s_sin_addr <= s_line_phase(15 downto 8);
                    s_lstep <= s_lstep + 1;
                when 1 | 2 =>
                    s_lstep <= s_lstep + 1;                                   -- ROM latency
                when 3 =>
                    s_jp <= s_sin_q2 * signed('0' & s_wamp);
                    s_lstep <= s_lstep + 1;
                when 4 =>
                    v_q := signed(resize(s_qb, 16)) + resize(shift_right(s_jp, 5), 16);
                    if v_q < 64 then v_q := to_signed(64, 16); end if;
                    s_q <= unsigned(v_q(13 downto 0));
                    s_lstep <= s_lstep + 1;
                when 5 =>
                    if s_sw_fringe = '1' then s_qc <= resize(s_q, 15) + resize(s_q(13 downto 1), 15); else s_qc <= resize(s_q, 15); end if;
                    if s_sw_local = '1' then
                        v_n := resize(s_bn, 4) + resize(s_bnl, 4) + resize(s_q(13 downto 11), 4);
                    else
                        v_n := resize(s_bn, 4) + resize(s_bnl, 4);
                    end if;
                    if v_n > 7 then v_n := to_unsigned(7, 4); end if;
                    s_bn_row <= v_n(2 downto 0);
                    s_lstep <= s_lstep + 1;
                when 6 =>
                    s_lstep <= s_lstep + 1;                                   -- split products settle
                when 7 =>
                    s_pq  <= shift_left(resize(s_pqh, 25), 7) + resize(s_pql, 25);
                    s_pqc <= shift_left(resize(s_pqch, 26), 7) + resize(s_pqcl, 26);
                    s_lstep <= s_lstep + 1;
                when 8 =>
                    s_src0  <= signed(resize(s_seam256, 27)) - signed(resize(s_pq, 27)) - signed(resize(s_q, 27));
                    s_srcc0 <= signed(resize(s_seam256, 27)) - signed(resize(s_pqc, 27)) - signed(resize(s_qc, 27)) - signed(resize(s_lag256, 27));
                    s_lstep <= s_lstep + 1;
                when others => null;
            end case;

            if s_prev_avid = '1' and data_in.avid = '0' then
                s_line_width <= s_rx; s_line <= s_line + 1;
            end if;
            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_wpar <= not s_wpar; s_rbank <= s_wpar; s_lstep <= (others => '0');
                s_line_phase <= s_line_phase + 96;
            end if;
            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                s_line <= (others => '0'); s_wpar <= '0';
                s_line_phase <= s_fphase;
                if s_saw_active = '1' then s_vstep <= (others => '0'); end if;
                s_saw_active <= '0';
            end if;

            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_field_sr(0) <= data_in.field_n; s_avid_sr(0)  <= data_in.avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_field_sr(i) <= s_field_sr(i - 1); s_avid_sr(i)  <= s_avid_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Address chain: two DDAs (luma / chroma), edge fill (stages 2..5)
    --------------------------------------------------------------------------
    p_addr : process(clk)
    begin
        if rising_edge(clk) then
            s_avd(1) <= s_in_avid;
            for i in 2 to 4 loop s_avd(i) <= s_avd(i - 1); end loop;
            -- 3: DDA
            if s_avd(1) = '0' then
                s_accy <= s_src0; s_accc <= s_srcc0;
            else
                s_accy <= s_accy + signed(resize(s_q, 27));
                s_accc <= s_accc + signed(resize(s_qc, 27));
            end if;
            -- 4: integer source x
            s_srcy4 <= s_accy(26 downto 8);
            s_srcc4 <= s_accc(26 downto 8);
            -- 5: edge fill
            s_addrY <= f_edge_addr(s_srcy4, s_we19, s_w_edge);
            s_addrC <= f_edge_addr(s_srcc4, s_we19, s_w_edge);
        end if;
    end process p_addr;

    --------------------------------------------------------------------------
    -- Line buffers (written stage 2, read stage 5 -> data 6)
    --------------------------------------------------------------------------
    p_lbY0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY0 <= lbY0(to_integer(s_addrY));
            if s_we2 = '1' and s_wp2 = '0' then lbY0(to_integer(s_wx2)) <= std_logic_vector(s_py(15 downto 6)); end if;
        end if;
    end process;
    p_lbY1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdY1 <= lbY1(to_integer(s_addrY));
            if s_we2 = '1' and s_wp2 = '1' then lbY1(to_integer(s_wx2)) <= std_logic_vector(s_py(15 downto 6)); end if;
        end if;
    end process;
    p_lbU0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU0 <= lbU0(to_integer(s_addrC(10 downto 1)));
            if s_we2 = '1' and s_wp2 = '0' then lbU0(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_pu(15 downto 8)); end if;
        end if;
    end process;
    p_lbU1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdU1 <= lbU1(to_integer(s_addrC(10 downto 1)));
            if s_we2 = '1' and s_wp2 = '1' then lbU1(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_pu(15 downto 8)); end if;
        end if;
    end process;
    p_lbV0 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV0 <= lbV0(to_integer(s_addrC(10 downto 1)));
            if s_we2 = '1' and s_wp2 = '0' then lbV0(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_pv(15 downto 8)); end if;
        end if;
    end process;
    p_lbV1 : process(clk)
    begin
        if rising_edge(clk) then
            s_rdV1 <= lbV1(to_integer(s_addrC(10 downto 1)));
            if s_we2 = '1' and s_wp2 = '1' then lbV1(to_integer(s_wx2(10 downto 1))) <= std_logic_vector(s_pv(15 downto 8)); end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Pixel path (stages 7..14): fetch, post bleed, crawl, gate
    --------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_s : integer range 0 to 7;
        variable v_au, v_av : unsigned(9 downto 0);
        variable v_p : std_logic;
        variable v_y : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- 6 (rd) / 7: fetch
            s_a_rbank <= s_rbank;
            if s_a_rbank = '1' then
                s_wet_y <= unsigned(s_rdY1); s_wet_u <= unsigned(s_rdU1) & "00"; s_wet_v <= unsigned(s_rdV1) & "00";
            else
                s_wet_y <= unsigned(s_rdY0); s_wet_u <= unsigned(s_rdU0) & "00"; s_wet_v <= unsigned(s_rdV0) & "00";
            end if;
            -- 8: post bleed IIR (Q6)
            v_s := to_integer(s_bn_row);
            if s_avid_sr(6) = '0' then
                s_qy <= s_wet_y & "000000"; s_qu <= s_wet_u & "000000"; s_qv <= s_wet_v & "000000";
            elsif s_sw_after = '1' then
                s_qu <= s_qu - shift_right(s_qu, v_s) + shift_right(s_wet_u & "000000", v_s);
                s_qv <= s_qv - shift_right(s_qv, v_s) + shift_right(s_wet_v & "000000", v_s);
                if s_sw_lumasmear = '1' then
                    s_qy <= s_qy - shift_right(s_qy, v_s) + shift_right(s_wet_y & "000000", v_s);
                else
                    s_qy <= s_wet_y & "000000";
                end if;
            else
                s_qy <= s_wet_y & "000000"; s_qu <= s_wet_u & "000000"; s_qv <= s_wet_v & "000000";
            end if;
            -- 9: chroma differences
            s_y9 <= s_qy(15 downto 6); s_u9 <= s_qu(15 downto 6); s_v9 <= s_qv(15 downto 6);
            s_du9 <= signed('0' & s_qu(15 downto 6)) - to_signed(512, 11);
            s_dv9 <= signed('0' & s_qv(15 downto 6)) - to_signed(512, 11);
            -- 10: saturation
            if s_du9 < 0 then v_au := unsigned(resize(-s_du9, 10)); else v_au := unsigned(resize(s_du9, 10)); end if;
            if s_dv9 < 0 then v_av := unsigned(resize(-s_dv9, 10)); else v_av := unsigned(resize(s_dv9, 10)); end if;
            s_sat10 <= resize(v_au, 11) + resize(v_av, 11);
            s_y10 <= s_y9; s_u10 <= s_u9; s_v10 <= s_v9;
            -- 11: crawl floor (dots everywhere, strongest on colour)
            s_sf11 <= resize(s_sat10(10 downto 2), 10) + to_unsigned(32, 10);
            s_y11 <= s_y10; s_u11 <= s_u10; s_v11 <= s_v10;
            -- 12: crawl amplitude
            s_pc12 <= s_sf11 * s_cg;
            s_y12 <= s_y11; s_u12 <= s_u11; s_v12 <= s_v11;
            -- 13: crawl pattern
            if s_avid_sr(11) = '0' then s_cx <= '0'; else s_cx <= not s_cx; end if;
            s_prev_hs <= s_hsync_sr(11);
            if s_vsync_sr(11) = '0' then s_clp <= '0';
            elsif s_hsync_sr(11) = '0' and s_prev_hs = '1' then s_clp <= not s_clp; end if;
            v_p := s_cx xor s_clp xor (s_sw_drift and s_frame);
            if v_p = '1' then
                v_y := signed(resize(s_y12, 13)) + signed(resize(s_pc12(17 downto 9), 13));
            else
                v_y := signed(resize(s_y12, 13)) - signed(resize(s_pc12(17 downto 9), 13));
            end if;
            s_y13 <= f_clamp10(v_y); s_u13 <= s_u12; s_v13 <= s_v12;
            -- 14: gate
            if s_avid_sr(C_LAT - 2) = '1' then
                s_y14 <= s_y13; s_u14 <= s_u13; s_v14 <= s_v13;
            else
                s_y14 <= to_unsigned(64, 10); s_u14 <= to_unsigned(512, 10); s_v14 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_pix;

    data_out.y       <= std_logic_vector(s_y14);
    data_out.u       <= std_logic_vector(s_u14);
    data_out.v       <= std_logic_vector(s_v14);
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_sr(C_LAT - 1);
    data_out.avid    <= s_avid_sr(C_LAT - 1);

end architecture syncbleed;
