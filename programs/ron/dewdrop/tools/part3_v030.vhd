
    -- argmin payload
    type t_mi is record
        m   : signed(24 downto 0);
        i   : unsigned(2 downto 0);     -- circle index; i(2) = small class
    end record;
    type t_mi_arr is array(natural range <>) of t_mi;

    function f_min(a, b : t_mi) return t_mi is
    begin
        if a.m(24 downto 8) <= b.m(24 downto 8) then return a; else return b; end if;
    end function;

    function f_ilog2(v : unsigned(24 downto 0)) return integer is
        variable r : integer range 0 to 24 := 0;
    begin
        for i in 0 to 24 loop
            if v(i) = '1' then r := i; end if;
        end loop;
        return r;
    end function;

    -- v * k/8, k in 0..15
    function f_fr8(v : unsigned(15 downto 0); k : unsigned(3 downto 0)) return unsigned is
        variable e : unsigned(15 downto 0);
        variable r : unsigned(15 downto 0);
    begin
        e := shift_right(v, 3);
        r := (others => '0');
        if k(0) = '1' then r := r + e; end if;
        if k(1) = '1' then r := r + shift_left(e, 1); end if;
        if k(2) = '1' then r := r + shift_left(e, 2); end if;
        if k(3) = '1' then r := r + shift_left(e, 3); end if;
        return r;
    end function;

    -- barrel shifts as slice selects (one 16:1 mux per output bit)
    function f_bsel8(v : signed(24 downto 0); sh : unsigned(3 downto 0)) return unsigned is
        variable r : unsigned(7 downto 0);
    begin
        case sh is
            when x"0" => r := unsigned(v(7 downto 0));
            when x"1" => r := unsigned(v(8 downto 1));
            when x"2" => r := unsigned(v(9 downto 2));
            when x"3" => r := unsigned(v(10 downto 3));
            when x"4" => r := unsigned(v(11 downto 4));
            when x"5" => r := unsigned(v(12 downto 5));
            when x"6" => r := unsigned(v(13 downto 6));
            when x"7" => r := unsigned(v(14 downto 7));
            when x"8" => r := unsigned(v(15 downto 8));
            when x"9" => r := unsigned(v(16 downto 9));
            when x"A" => r := unsigned(v(17 downto 10));
            when x"B" => r := unsigned(v(18 downto 11));
            when x"C" => r := unsigned(v(19 downto 12));
            when x"D" => r := unsigned(v(20 downto 13));
            when x"E" => r := unsigned(v(21 downto 14));
            when others => r := unsigned(v(22 downto 15));
        end case;
        return r;
    end function;

    function f_bsel6(v : signed(24 downto 0); sh : unsigned(3 downto 0)) return unsigned is
        variable r : unsigned(5 downto 0);
    begin
        case sh is
            when x"0" => r := unsigned(v(7 downto 2));
            when x"1" => r := unsigned(v(8 downto 3));
            when x"2" => r := unsigned(v(9 downto 4));
            when x"3" => r := unsigned(v(10 downto 5));
            when x"4" => r := unsigned(v(11 downto 6));
            when x"5" => r := unsigned(v(12 downto 7));
            when x"6" => r := unsigned(v(13 downto 8));
            when x"7" => r := unsigned(v(14 downto 9));
            when x"8" => r := unsigned(v(15 downto 10));
            when x"9" => r := unsigned(v(16 downto 11));
            when x"A" => r := unsigned(v(17 downto 12));
            when x"B" => r := unsigned(v(18 downto 13));
            when x"C" => r := unsigned(v(19 downto 14));
            when x"D" => r := unsigned(v(20 downto 15));
            when x"E" => r := unsigned(v(21 downto 16));
            when others => r := unsigned(v(22 downto 17));
        end case;
        return r;
    end function;

    function f_sh14(v : signed(13 downto 0); sh : unsigned(2 downto 0)) return signed is
        variable r : signed(13 downto 0);
    begin
        case sh is
            when "000" => r := v;
            when "001" => r := resize(v(13 downto 1), 14);
            when "010" => r := resize(v(13 downto 2), 14);
            when "011" => r := resize(v(13 downto 3), 14);
            when "100" => r := resize(v(13 downto 4), 14);
            when "101" => r := resize(v(13 downto 5), 14);
            when "110" => r := resize(v(13 downto 6), 14);
            when others => r := resize(v(13 downto 7), 14);
        end case;
        return r;
    end function;

    function f_clamp8(v : signed) return signed is
    begin
        if v > 127 then return to_signed(127, 8);
        elsif v < -127 then return to_signed(-127, 8);
        else return resize(v, 8); end if;
    end function;

    function f_clamp_y(v : signed) return unsigned is
    begin
        if v < 64 then return to_unsigned(64, 10);
        elsif v > 775 then return to_unsigned(775, 10);
        else return unsigned(resize(v, 10)); end if;
    end function;

    function f_cl4(v : integer) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 4);
        elsif v > 15 then return to_unsigned(15, 4);
        else return to_unsigned(v, 4); end if;
    end function;

    function f_cl3(v : integer) return unsigned is
    begin
        if v < 0 then return to_unsigned(0, 3);
        elsif v > 7 then return to_unsigned(7, 3);
        else return to_unsigned(v, 3); end if;
    end function;

    -- ==== timing / raster ====
    signal s_timing  : t_video_timing_port;
    signal s_hcnt    : unsigned(11 downto 0) := (others => '0');
    signal s_vcnt    : unsigned(11 downto 0) := (others => '0');
    signal s_meas_h  : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_meas_v  : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal s_ilace   : std_logic := '0';
    signal s_botf    : std_logic := '0';
    signal s_fh      : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal s_fw      : unsigned(3 downto 0) := to_unsigned(6, 4);
    signal s_h16     : unsigned(15 downto 0) := to_unsigned(1080*16, 16);
    signal s_stepf   : unsigned(5 downto 0) := to_unsigned(16, 6);   -- frame-line step (Q4 px)
    signal s_step    : unsigned(6 downto 0) := to_unsigned(16, 7);   -- displayed-line step
    signal s_foff    : unsigned(5 downto 0) := (others => '0');       -- field offset (Q4)

    -- ==== controls ====
    signal s_dark, s_pairs, s_stripes, s_solid, s_chrome : std_logic := '0';
    signal s_k5, s_k6 : unsigned(9 downto 0) := C_MID;
    signal s_saw    : std_logic := '0';
    signal s_vs_go  : std_logic := '0';
    signal s_a2, s_a12 : unsigned(13 downto 0) := (others => '0');
    signal s_k1s, s_k2s, s_k3s, s_k4s, s_p12s : unsigned(9 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal s_bcol : unsigned(10 downto 0) := (others => '0');

    -- ==== per-frame constants ====
    signal s_bm    : unsigned(4 downto 0) := to_unsigned(20, 5);     -- band mantissa 16..31
    signal s_be    : unsigned(1 downto 0) := "01";                    -- band exponent
    signal s_k     : unsigned(5 downto 0) := to_unsigned(32, 6);      -- refract
    signal s_A     : unsigned(7 downto 0) := to_unsigned(75, 8);      -- wave amplitude / 4 px
    signal s_R     : unsigned(10 downto 0) := to_unsigned(300, 11);
    signal s_Rs    : unsigned(10 downto 0) := to_unsigned(180, 11);
    signal s_R2, s_Rs2 : unsigned(24 downto 0) := (others => '0');
    signal s_w     : unsigned(8 downto 0) := to_unsigned(16, 9);
    signal s_Dbig, s_Dsml, s_Dfbig, s_Dfsml : unsigned(24 downto 0) := (others => '0');
    signal s_fs, s_fss, s_dsh, s_dssh : unsigned(3 downto 0) := (others => '0');
    signal s_ss, s_sss : unsigned(2 downto 0) := (others => '0');
    signal s_rate  : unsigned(15 downto 0) := to_unsigned(32, 16);
    signal s_fad   : unsigned(9 downto 0) := (others => '0');
    signal s_fneg  : std_logic := '0';
    signal s_fq    : unsigned(9 downto 0) := (others => '0');
    signal s_ph16  : unsigned(15 downto 0) := (others => '0');
    signal s_sc16  : unsigned(15 downto 0) := (others => '0');
    signal s_Rmin  : unsigned(10 downto 0) := to_unsigned(60, 11);
    signal s_Wh    : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_Hh    : unsigned(10 downto 0) := to_unsigned(540, 11);
    signal s_WhR   : unsigned(10 downto 0) := to_unsigned(900, 11);
    signal s_seeded : std_logic := '0';
    signal s_lin   : unsigned(24 downto 0) := (others => '0');
    signal s_lR, s_lw : integer range 0 to 24 := 8;

    -- per-circle orbit constants
    type t_k4 is array(0 to C_N-1) of unsigned(3 downto 0);
    type t_k8 is array(0 to C_N-1) of unsigned(7 downto 0);
    constant C_KAX : t_k4 := (x"8", x"6", x"7", x"5", x"8", x"7", x"6", x"8");
    constant C_KAY : t_k4 := (x"7", x"8", x"5", x"8", x"6", x"8", x"7", x"5");
    constant C_KRX : t_k4 := (x"5", x"7", x"9", x"6", x"B", x"8", x"7", x"A");
    constant C_KRY : t_k4 := (x"7", x"5", x"6", x"9", x"8", x"B", x"A", x"6");
    constant C_TH0 : t_k8 := (x"00", x"3A", x"7C", x"B1", x"21", x"66", x"9E", x"D8");
    constant C_PH0 : t_k8 := (x"40", x"8C", x"11", x"CA", x"6B", x"F2", x"2F", x"95");

    -- circle table EBR: 8 words per circle
    --   0 cx, 1 cy, 2 M0 lo, 3 M0 hi, 4 mline lo, 5 mline hi, 6 theta, 7 phi
    type t_ct is array(0 to 255) of unsigned(15 downto 0);
    signal ct_ram : t_ct := (others => (others => '0'));
    attribute no_rw_check : boolean;
    attribute no_rw_check of ct_ram : signal is true;
    signal ct_ra, ct_wa : unsigned(7 downto 0) := (others => '0');
    signal ct_wd : unsigned(15 downto 0) := (others => '0');
    signal ct_we : std_logic := '0';
    signal ct_rd, ct_rd2 : unsigned(15 downto 0) := (others => '0');
    signal q_ra, q_wa : unsigned(7 downto 0) := (others => '0');
    signal q_wd : unsigned(15 downto 0) := (others => '0');
    signal q_we : std_logic := '0';
    signal l_ra, l_wa : unsigned(7 downto 0) := (others => '0');
    signal l_wd : unsigned(15 downto 0) := (others => '0');
    signal l_we : std_logic := '0';

    -- sine ROM port (shared)
    signal sin_addr : unsigned(7 downto 0) := (others => '0');
    signal q_sa     : unsigned(7 downto 0) := (others => '0');
    signal sin_q    : unsigned(9 downto 0) := (others => '0');
    signal sin_q2   : signed(9 downto 0) := (others => '0');

    -- sequencer
    signal s_op   : integer range 0 to 63 := 63;
    signal s_bit  : integer range 0 to 15 := 0;
    signal s_mA   : unsigned(23 downto 0) := (others => '0');
    signal s_mB   : unsigned(11 downto 0) := (others => '0');
    signal s_acc  : unsigned(23 downto 0) := (others => '0');
    signal s_kr   : unsigned(3 downto 0) := (others => '0');
    signal s_rk   : unsigned(15 downto 0) := (others => '0');
    signal s_ci   : unsigned(2 downto 0) := (others => '0');   -- pass index
    signal s_cj   : unsigned(2 downto 0) := (others => '0');   -- circle = ci(0) & ci(2..1)
    signal s_th, s_phs : unsigned(15 downto 0) := (others => '0');
    signal s_sx, s_sy : signed(9 downto 0) := (others => '0');
    signal s_cxq, s_cyq : unsigned(11 downto 0) := (others => '0');
    signal s_cxb, s_cyb : unsigned(11 downto 0) := (others => '0');  -- last big circle
    signal s_qm0 : signed(24 downto 0) := (others => '0');
    signal s_seq_busy : std_logic := '0';
    signal s_seq_done : std_logic := '0';
    attribute fsm_encoding : string;
    attribute fsm_encoding of s_op : signal is "none";

    -- ==== line FSM (compute next-line values, load accumulators) ====
    signal s_ls   : integer range 0 to 63 := 0;
    signal s_lj   : unsigned(2 downto 0) := (others => '0');
    signal s_lbit : integer range 0 to 15 := 0;
    signal s_qa   : unsigned(31 downto 0) := (others => '0');   -- square multiplicand (shifting)
    signal s_qb   : unsigned(15 downto 0) := (others => '0');
    signal s_qacc : unsigned(31 downto 0) := (others => '0');
    signal s_mllo : unsigned(15 downto 0) := (others => '0');
    signal s_mline : signed(24 downto 0) := (others => '0');
    signal s_yq4   : unsigned(15 downto 0) := (others => '0');
    signal s_yq4n  : unsigned(15 downto 0) := (others => '0');   -- next line
    signal s_ypx   : unsigned(11 downto 0) := (others => '0');
    signal s_yadv  : std_logic := '0';
    signal s_init_req, s_comp_req, s_load_req : std_logic := '0';
    signal s_init_mode : std_logic := '0';
    signal s_line_busy : std_logic := '0';
    signal l_ld_we  : std_logic := '0';
    signal l_ld_idx : unsigned(2 downto 0) := (others => '0');
    signal l_ld_i   : signed(12 downto 0) := (others => '0');
    attribute fsm_encoding of s_ls : signal is "none";

    -- ==== stage 0: DDAs ====
    type t_s25a is array(0 to C_N-1) of signed(24 downto 0);
    type t_s13a is array(0 to C_N-1) of signed(12 downto 0);
    signal s_m    : t_s25a := (others => (others => '0'));
    signal s_incx : t_s13a := (others => (others => '0'));   -- 2x + (1 - 2cx), registered
    signal s_ix0  : t_s13a := (others => (others => '0'));
    signal g_avid_d : std_logic := '0';
    signal s_bcol_d : unsigned(10 downto 0) := (others => '0');

    -- ==== pixel pipeline ====
    signal ins1, rim1 : std_logic_vector(0 to C_N-1) := (others => '0');
    signal m1c : t_s25a := (others => (others => '0'));
    signal m2  : signed(24 downto 0) := (others => '0');
    signal idx2 : unsigned(2 downto 0) := (others => '0');
    signal ins2, rim2 : std_logic := '0';
    signal a3 : t_mi;
    signal ins3, rim3 : std_logic := '0';

    -- cx / cy lookup EBRs (written by the sequencer alongside the table)
    type t_lut16 is array(0 to 255) of unsigned(15 downto 0);
    signal cxt_ram, cyt_ram : t_lut16 := (others => (others => '0'));
    attribute no_rw_check of cxt_ram : signal is true;
    attribute no_rw_check of cyt_ram : signal is true;
    signal cxt_rd, cyt_rd : unsigned(15 downto 0) := (others => '0');

    signal ins4, rim4, cls4 : std_logic := '0';
    signal mf4, r2_4 : signed(24 downto 0) := (others => '0');
    signal keff4 : unsigned(5 downto 0) := (others => '0');
    signal fs4, ds4 : unsigned(3 downto 0) := (others => '0');

    signal cx5, cy5 : unsigned(11 downto 0) := (others => '0');
    signal x5 : unsigned(10 downto 0) := (others => '0');
    signal fr5 : unsigned(7 downto 0) := (others => '0');
    signal disp5 : unsigned(5 downto 0) := (others => '0');
    signal zone5, rim5, ins5, cls5 : std_logic := '0';
    signal keff5 : unsigned(5 downto 0) := (others => '0');

    signal dx6, dy6 : signed(12 downto 0) := (others => '0');
    signal keff6 : unsigned(5 downto 0) := (others => '0');
    signal dlA_wd : unsigned(15 downto 0) := (others => '0');  -- stage 6: fr(8) disp(6) zone rim
    signal rim6, zone6, ins6, cls6 : std_logic := '0';
    signal x6 : unsigned(10 downto 0) := (others => '0');

    signal dxk7a, dxk7b, dyk7a, dyk7b : signed(16 downto 0) := (others => '0');
    signal sh7 : signed(13 downto 0) := (others => '0');
    signal rim7, zone7, ins7, cls7 : std_logic := '0';
    signal x7 : unsigned(10 downto 0) := (others => '0');

    signal dxk8, dyk8 : signed(19 downto 0) := (others => '0');
    signal shade8 : signed(7 downto 0) := (others => '0');
    signal rim8, zone8, ins8 : std_logic := '0';
    signal x8 : unsigned(10 downto 0) := (others => '0');

    signal x9, y9 : signed(12 downto 0) := (others => '0');
    signal dlB_wd : unsigned(15 downto 0) := (others => '0');  -- stage 9: shade(8) rim zone ins

    signal a10, b10 : signed(12 downto 0) := (others => '0');
    signal bf11 : signed(17 downto 0) := (others => '0');
    signal wa12 : unsigned(7 downto 0) := (others => '0');
    signal sa15 : signed(18 downto 0) := (others => '0');
    signal sa16 : signed(18 downto 0) := (others => '0');
    signal p17 : signed(13 downto 0) := (others => '0');
    signal pb18 : signed(19 downto 0) := (others => '0');
    signal idx19 : unsigned(7 downto 0) := (others => '0');
    signal pal20_y, pal20_u, pal20_v : unsigned(9 downto 0) := (others => '0');
    signal pal21_y, pal21_u, pal21_v : unsigned(9 downto 0) := (others => '0');
    signal rimy21 : unsigned(9 downto 0) := C_BLK;
    signal shade21 : signed(7 downto 0) := (others => '0');
    signal rim21, zone21, ins21 : std_logic := '0';
    signal c22_y : unsigned(9 downto 0) := C_BLK;
    signal c22_u, c22_v : unsigned(9 downto 0) := C_MID;
    signal c22d_y : unsigned(9 downto 0) := C_BLK;
    signal c22d_u, c22d_v : unsigned(9 downto 0) := C_MID;
    signal s23_y, s23_u, s23_v : unsigned(10 downto 0) := (others => '0');
    signal s23d_y, s23d_u, s23d_v : unsigned(10 downto 0) := (others => '0');
    signal s24_y : unsigned(9 downto 0) := C_BLK;
    signal s24_u, s24_v : unsigned(9 downto 0) := C_MID;
    signal g25_y, g26_y : unsigned(9 downto 0) := C_BLK;
    signal g25_u, g26_u : unsigned(9 downto 0) := C_MID;
    signal g25_v, g26_v : unsigned(9 downto 0) := C_MID;

    -- delay-line EBRs (written every clock at dl_wa, read N back; data
    -- written at stage S lands in *_rd2 at stage S + N + 2)
    type t_dl is array(0 to 255) of unsigned(15 downto 0);
    signal dlA_ram, dlB_ram, dlC_ram, dlX_ram : t_dl := (others => (others => '0'));
    attribute no_rw_check of dlA_ram : signal is true;
    attribute no_rw_check of dlB_ram : signal is true;
    attribute no_rw_check of dlC_ram : signal is true;
    attribute no_rw_check of dlX_ram : signal is true;
    signal dl_wa : unsigned(7 downto 0) := (others => '0');
    signal dlA_rd, dlA_rd2, dlB_rd, dlB_rd2 : unsigned(15 downto 0) := (others => '0');
    signal dlC_rd, dlC_rd2, dlX_rd, dlX_rd2 : unsigned(15 downto 0) := (others => '0');
    constant C_DLX : integer := 3;    -- x (stage 0) -> stage 5
    constant C_DLA : integer := 10;   -- bundle A (stage 6) -> stage 18
    constant C_DLB : integer := 9;    -- bundle B (stage 9) -> stage 20
    constant C_DLC : integer := 4;    -- a (stage 10) -> stage 16

    -- ==== sync delay ====
    type t_bsr is array(1 to C_LAT - 1) of std_logic;
    signal d_hs, d_vs, d_fl, d_gv : t_bsr := (others => '0');

    signal s_io : t_video_stream_yuv444_30b
        := ((others => '0'), (others => '0'), (others => '0'), '0', '1', '1', '1');

begin

    timing_gen_inst : entity work.video_timing_generator
        port map (clk => clk, ref_hsync_n => data_in.hsync_n,
                  ref_vsync_n => data_in.vsync_n, ref_avid => data_in.avid,
                  timing => s_timing);

    -- ====================================================================
    -- Raster measurement + step class
    -- ====================================================================
    p_measure : process(clk)
        variable v_fh : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            if s_timing.hsync_start = '1' then
                if s_hcnt > 0 then s_meas_h <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            elsif s_timing.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            end if;

            if s_timing.vsync_start = '1' then
                if s_vcnt > 0 then s_meas_v <= s_vcnt; end if;
                s_vcnt  <= (others => '0');
                s_ilace <= s_timing.is_interlaced;
                s_botf  <= not s_timing.field_n;      -- bottom field = field_n '0'
                if s_timing.is_interlaced = '1' then
                    v_fh := s_meas_v(10 downto 0) & '0';
                else
                    v_fh := s_meas_v;
                end if;
                s_fh <= v_fh;
                if s_meas_h >= to_unsigned(1000, 12) then       -- HD / 720p: square
                    s_stepf <= to_unsigned(16, 6);
                    s_h16 <= v_fh & "0000";
                    if s_meas_h >= to_unsigned(1600, 12) then
                        s_fw <= to_unsigned(6, 4);
                    else
                        s_fw <= to_unsigned(9, 4);
                    end if;
                elsif v_fh >= to_unsigned(540, 12) then          -- PAL: 15/16
                    s_stepf <= to_unsigned(15, 6);
                    s_h16 <= (v_fh & "0000") - v_fh;
                    s_fw <= to_unsigned(11, 4);
                else                                            -- NTSC: 18/16
                    s_stepf <= to_unsigned(18, 6);
                    s_h16 <= (v_fh & "0000") + (v_fh & '0');
                    s_fw <= to_unsigned(11, 4);
                end if;
            elsif s_timing.avid_start = '1' then
                s_vcnt <= s_vcnt + 1;
            end if;

            if s_ilace = '1' then
                s_step <= s_stepf & '0';
                if s_botf = '1' then s_foff <= s_stepf; else s_foff <= (others => '0'); end if;
            else
                s_step <= '0' & s_stepf;
                s_foff <= (others => '0');
            end if;
        end if;
    end process p_measure;

    -- ====================================================================
    -- Per-field control latch + motion phases
    -- ====================================================================
    p_frame : process(clk)
        variable v_d : signed(11 downto 0);
        variable v_a : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            s_vs_go <= '0';
            if s_timing.avid = '1' then
                s_saw <= '1';
            elsif s_timing.vsync_start = '1' and s_saw = '1' then
                s_saw   <= '0';
                s_vs_go <= '1';
            end if;

            if s_vs_go = '1' then
                g_wm1     <= s_meas_h - 1;
                s_dark    <= registers_in(6)(0);
                s_pairs   <= not registers_in(6)(1);   -- label 0 = Pairs
                s_stripes <= registers_in(6)(2);
                s_solid   <= registers_in(6)(3);
                s_chrome  <= registers_in(6)(4);
                s_k5 <= unsigned(registers_in(4));
                s_k6 <= unsigned(registers_in(5));
                if s_fneg = '1' then
                    s_ph16 <= s_ph16 - (s_fq & '0');
                    s_sc16 <= s_sc16 - s_fq;
                else
                    s_ph16 <= s_ph16 + (s_fq & '0');
                    s_sc16 <= s_sc16 + s_fq;
                end if;
            end if;

            s_k2s  <= s_a2(13 downto 4);
            s_p12s <= s_a12(13 downto 4);

            -- Flow: bipolar deadband +-32
            v_d := resize(signed('0' & s_k6), 12) - to_signed(512, 12);
            if v_d < 0 then v_a := resize(unsigned(-v_d), 11); s_fneg <= '1';
            else            v_a := resize(unsigned(v_d), 11);  s_fneg <= '0'; end if;
            if v_a > to_unsigned(32, 11) then s_fad <= resize(v_a - 32, 10);
            else                              s_fad <= (others => '0'); end if;

            -- orbit base rate: 4 .. 260 (1/65536 turn per frame per 8/8)
            s_rate <= resize(s_k5(9 downto 2), 16) + 4;
            -- band scale = (16 + k1[7:4]) << k1[9:8]: 16..248, monotonic
            s_bm   <= to_unsigned(16, 5) + resize(s_k1s(7 downto 4), 5);
            s_be   <= s_k1s(9 downto 8);
            s_k    <= s_k3s(9 downto 4);
            s_Wh   <= s_meas_h(11 downto 1);
            s_Hh   <= s_h16(15 downto 5);
            s_Rmin <= resize(s_meas_h(11 downto 5), 11);
        end if;
    end process p_frame;

    -- ====================================================================
    -- Vblank sequencer: slews, shared serial multiplier (24x12, 14 cycles),
    -- frame constants, per-circle orbits (theta/phi in the circle table,
    -- sine ROM time-shared), cx/cy/M0 into the circle table.
    -- ====================================================================
    p_seq : process(clk)
        variable v_t   : unsigned(13 downto 0);
        variable v_d   : signed(14 downto 0);
        variable v_n   : signed(15 downto 0);
        variable v_R   : unsigned(10 downto 0);
        variable v_cx, v_cy : unsigned(11 downto 0);
        variable v_p   : unsigned(11 downto 0);
        variable v_ps  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_seq_done <= '0';
            q_we <= '0';
            s_rk <= f_fr8(s_rate, s_kr);
            if s_vs_go = '1' then
                s_op  <= 0;
                s_bit <= 0;
            elsif s_timing.avid = '0' then
                case s_op is
                    when 0 =>
                        if s_line_busy = '0' then
                            s_seq_busy <= '1';
                            s_op <= 1;
                        end if;
                    -- 1-pole slews (Q10.4): acc += (target - acc)/16
                    when 1 =>
                        s_k1s <= unsigned(registers_in(0));
                        s_k3s <= unsigned(registers_in(2));
                        s_k4s <= unsigned(registers_in(3));
                        s_op <= 2;
                    when 2 =>
                        v_t := unsigned(registers_in(1)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a2);
                        v_n := resize(signed('0' & s_a2), 16) + resize(shift_right(v_d, 4), 16);
                        s_a2 <= unsigned(v_n(13 downto 0));
                        s_op <= 5;
                    when 5 =>
                        v_t := unsigned(registers_in(7)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a12);
                        v_n := resize(signed('0' & s_a12), 16) + resize(shift_right(v_d, 4), 16);
                        s_a12 <= unsigned(v_n(13 downto 0));
                        s_WhR <= s_Wh - s_Rmin;
                        s_op <= 6;

                    -- shared multiplier: 12-bit mB consumed at bits 1..12,
                    -- store at bit 13
                    when 6 to 13 =>
                        if s_bit = 0 then
                            s_acc <= (others => '0');
                            case s_op is
                                when 6      => s_mA <= resize(s_WhR, 24);        s_mB <= resize(s_p12s, 12);
                                when 7      => s_mA <= resize(s_R, 24);          s_mB <= resize(s_R, 12);
                                when 8      => s_mA <= resize(s_Rs, 24);         s_mB <= resize(s_Rs, 12);
                                when 9      => s_mA <= resize(s_R, 24);          s_mB <= resize(s_k4s, 12);
                                when 10     => s_mA <= resize(s_R + s_w, 24);    s_mB <= resize(s_R + s_w, 12);
                                when 11     => s_mA <= resize(s_Rs + s_w, 24);   s_mB <= resize(s_Rs + s_w, 12);
                                when 12     => s_mA <= resize(s_Wh, 24);         s_mB <= resize(s_k2s, 12);
                                when others => s_mA <= resize(s_fad, 24);        s_mB <= resize(s_fad, 12);
                            end case;
                            s_bit <= 1;
                        else
                            if s_mB(0) = '1' then
                                s_acc <= s_acc + s_mA;
                            end if;
                            s_mA <= s_mA(22 downto 0) & '0';
                            s_mB <= '0' & s_mB(11 downto 1);
                            if s_bit = 13 then
                                s_bit <= 0;
                                s_op  <= s_op + 1;
                                case s_op is
                                    when 6      => v_R := s_Rmin + resize(s_acc(20 downto 10), 11);
                                                   s_R  <= v_R;
                                                   -- 0.61 R
                                                   s_Rs <= v_R - shift_right(v_R, 2) - shift_right(v_R, 3) - shift_right(v_R, 6);
                                    when 7      => s_R2  <= resize(s_acc, 25);
                                    when 8      => s_Rs2 <= resize(s_acc, 25);
                                    when 9      => s_w   <= to_unsigned(2, 9) + resize(s_acc(20 downto 12), 9);
                                    when 10     => s_Dbig <= resize(s_acc, 25) - s_R2;
                                    when 11     => s_Dsml <= resize(s_acc, 25) - s_Rs2;
                                    when 12     => s_A   <= s_acc(19 downto 12);
                                    when others => s_fq  <= resize(s_acc(17 downto 8), 10);
                                end case;
                            else
                                s_bit <= s_bit + 1;
                            end if;
                        end if;

                    when 14 =>
                        s_Dfbig <= s_Dbig - shift_right(s_Dbig, 2);
                        s_Dfsml <= s_Dsml - shift_right(s_Dsml, 2);
                        s_lin <= resize(s_R, 25);
                        s_op <= 15;
                    when 15 =>
                        s_lR <= f_ilog2(s_lin);
                        s_lin <= resize(s_w, 25);
                        s_op <= 16;
                    when 16 =>
                        s_lw <= f_ilog2(s_lin);
                        s_op <= 17;
                    when 17 =>
                        -- shade: |dx+dy| <= 1.41 R -> +-127 ; dispersion: r^2 < R^2 -> 0..255
                        -- fringe: m - Df < D/4 ~ R w / 2 -> 0..255
                        s_ss   <= f_cl3(s_lR - 6);
                        s_sss  <= f_cl3(s_lR - 7);
                        s_dsh  <= f_cl4(2 * s_lR - 7);
                        s_dssh <= f_cl4(2 * s_lR - 8);
                        s_fs   <= f_cl4(s_lR + s_lw - 8);
                        s_fss  <= f_cl4(s_lR + s_lw - 9);
                        s_ci <= (others => '0');
                        s_op <= 20;

                    -- ---- per circle: j = ci(0) & ci(2..1): 0,4,1,5,2,6,3,7 ----
                    when 20 =>
                        s_cj <= s_ci(0) & s_ci(2 downto 1);
                        s_op <= 21;
                    when 21 =>
                        q_ra <= "00" & s_cj & "110";                     -- theta
                        s_kr <= C_KRX(to_integer(s_cj));
                        s_op <= 22;
                    when 22 => s_op <= 23;
                    when 23 => s_op <= 24;
                    when 24 =>
                        if s_seeded = '0' then
                            s_th <= shift_left(resize(C_TH0(to_integer(s_cj)), 16), 8);
                        else
                            s_th <= ct_rd2 + s_rk;
                        end if;
                        q_ra <= "00" & s_cj & "111";                     -- phi
                        s_kr <= C_KRY(to_integer(s_cj));
                        s_op <= 25;
                    when 25 =>
                        q_wa <= "00" & s_cj & "110"; q_wd <= s_th; q_we <= '1';
                        q_sa <= s_th(15 downto 8);
                        s_op <= 26;
                    when 26 => s_op <= 27;
                    when 27 =>
                        if s_seeded = '0' then
                            s_phs <= shift_left(resize(C_PH0(to_integer(s_cj)), 16), 8);
                        else
                            s_phs <= ct_rd2 + s_rk;
                        end if;
                        s_op <= 28;
                    when 28 =>
                        s_sx <= sin_q2;                           -- sin(theta)
                        q_wa <= "00" & s_cj & "111"; q_wd <= s_phs; q_we <= '1';
                        q_sa <= s_phs(15 downto 8);
                        s_op <= 29;
                    when 29 => s_op <= 30;
                    when 30 => s_op <= 31;
                    when 31 =>
                        s_sy <= sin_q2;                           -- sin(phi)
                        s_op <= 32;
                    -- three products: ax*|sx|, ay*|sy|, cx*cx
                    when 32 to 34 =>
                        if s_bit = 0 then
                            s_acc <= (others => '0');
                            case s_op is
                                when 32 =>
                                    s_mA <= resize(s_Wh, 24);
                                    s_mB <= resize(unsigned(abs(s_sx)), 12);
                                    s_kr <= C_KAX(to_integer(s_cj));
                                when 33 =>
                                    s_mA <= resize(s_Hh, 24);
                                    s_mB <= resize(unsigned(abs(s_sy)), 12);
                                    s_kr <= C_KAY(to_integer(s_cj));
                                when others =>
                                    s_mA <= resize(s_cxq, 24);
                                    s_mB <= s_cxq;
                            end case;
                            s_bit <= 1;
                        else
                            if s_mB(0) = '1' then
                                s_acc <= s_acc + s_mA;
                            end if;
                            s_mA <= s_mA(22 downto 0) & '0';
                            s_mB <= '0' & s_mB(11 downto 1);
                            if s_bit = 13 then
                                s_bit <= 0;
                                s_op  <= s_op + 1;
                                case s_op is
                                    when 32 =>
                                        v_p := resize(shift_right(f_fr8(s_acc(18 downto 3), s_kr), 5), 12);
                                        if s_sx < 0 then v_ps := signed(resize(s_Wh, 13)) - signed(resize(v_p, 13));
                                        else             v_ps := signed(resize(s_Wh, 13)) + signed(resize(v_p, 13)); end if;
                                        if v_ps < 0 then s_cxq <= (others => '0');
                                        else s_cxq <= unsigned(v_ps(11 downto 0)); end if;
                                    when 33 =>
                                        v_p := resize(shift_right(f_fr8(s_acc(18 downto 3), s_kr), 5), 12);
                                        if s_sy < 0 then v_ps := signed(resize(s_Hh, 13)) - signed(resize(v_p, 13));
                                        else             v_ps := signed(resize(s_Hh, 13)) + signed(resize(v_p, 13)); end if;
                                        if v_ps < 0 then v_cy := (others => '0');
                                        else v_cy := unsigned(v_ps(11 downto 0)); end if;
                                        -- pairs: the small partner hangs 0.875 R under the big one
                                        if s_pairs = '1' and s_cj(2) = '1' then
                                            s_cxq <= s_cxb;
                                            s_cyq <= s_cyb + resize(s_R, 12) - resize(shift_right(s_R, 3), 12);
                                        else
                                            s_cyq <= v_cy;
                                        end if;
                                    when others =>
                                        if s_cj(2) = '1' then
                                            s_qm0 <= signed(resize(s_acc, 25)) - signed(s_Rs2);
                                        else
                                            s_qm0 <= signed(resize(s_acc, 25)) - signed(s_R2);
                                        end if;
                                end case;
                            else
                                s_bit <= s_bit + 1;
                            end if;
                        end if;
                    -- write cx, cy, M0 lo, M0 hi
                    when 35 =>
                        q_wa <= "00" & s_cj & "000"; q_wd <= resize(s_cxq, 16); q_we <= '1';
                        if s_cj(2) = '0' then s_cxb <= s_cxq; s_cyb <= s_cyq; end if;
                        s_op <= 36;
                    when 36 =>
                        q_wa <= "00" & s_cj & "001"; q_wd <= resize(s_cyq, 16); q_we <= '1';
                        s_op <= 37;
                    when 37 =>
                        q_wa <= "00" & s_cj & "010"; q_wd <= unsigned(s_qm0(15 downto 0)); q_we <= '1';
                        s_op <= 38;
                    when 38 =>
                        q_wa <= "00" & s_cj & "011"; q_wd <= unsigned(resize(s_qm0(24 downto 16), 16)); q_we <= '1';
                        if s_ci = 7 then
                            s_op <= 39;
                        else
                            s_ci <= s_ci + 1;
                            s_op <= 20;
                        end if;
                    when 39 =>
                        s_seeded <= '1';
                        s_seq_busy <= '0';
                        s_seq_done <= '1';
                        s_op <= 63;
                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process p_seq;

    -- circle-table port: sequencer in vblank, line FSM otherwise
    ct_ra <= q_ra when s_seq_busy = '1' else l_ra;
    ct_wa <= q_wa when s_seq_busy = '1' else l_wa;
    ct_wd <= q_wd when s_seq_busy = '1' else l_wd;
    ct_we <= q_we when s_seq_busy = '1' else l_we;

    p_ct : process(clk)
    begin
        if rising_edge(clk) then
            if ct_we = '1' then
                ct_ram(to_integer(ct_wa)) <= ct_wd;
            end if;
            ct_rd  <= ct_ram(to_integer(ct_ra));
            ct_rd2 <= ct_rd;
        end if;
    end process p_ct;

    -- shared sine ROM
    sin_addr <= q_sa when s_seq_busy = '1' else wa12;
    p_rom_sin : process(clk)
    begin
        if rising_edge(clk) then
            sin_q  <= C_SIN(to_integer(sin_addr));
            sin_q2 <= signed(sin_q);
        end if;
    end process p_rom_sin;

    -- ====================================================================
    -- Clean avid (cubist recipe, verbatim from aurora)
    -- ====================================================================
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;

            if g_hsr = '1' then
                g_ph  <= '0';
                g_arm <= '1';
            else
                g_ph <= not g_ph;
            end if;
            g_rsr <= g_rsr(1 downto 0) & '0';
            if g_avr = '1' and g_arm = '1' then
                g_arm    <= '0';
                g_rsr(0) <= '1';
                g_sel    <= g_ph xor g_par;
                if g_ph = '1' then
                    if g_pc /= 15 then g_pc <= g_pc + 1; end if;
                else
                    if g_pc /= 0 then g_pc <= g_pc - 1; end if;
                end if;
            end if;
            if g_pc = 15 then g_par <= '1';
            elsif g_pc = 0 then g_par <= '0'; end if;

            if (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;

            if g_hsr = '1' then
                s_bcol <= (others => '0');
            elsif g_avid = '1' then
                s_bcol <= s_bcol + 1;
            end if;
        end if;
    end process p_gavid;

    -- ====================================================================
    -- Line FSM.  COMPUTE (at each active-line start, for the NEXT line; at
    -- init for line 0): per circle read cy, d = yq4_next - 16 cy, square it
    -- serially, mline = M0 + d^2/256 -> circle table.  LOAD (at the hsync
    -- after an active line; after init): per circle read mline, cx, cy ->
    -- the x accumulators and dy registers.  y advances on ACTIVE lines
    -- only (flag at avid end, act at hsync, when the pipe has drained).
    -- ====================================================================
    p_lines : process(clk)
        variable v_d  : signed(17 downto 0);
        variable v_abs : signed(17 downto 0);
        variable v_ad : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            l_we <= '0';
            l_ld_we <= '0';

            -- y bookkeeping
            if s_seq_done = '1' then
                s_yq4  <= resize(s_foff, 16);
                s_yq4n <= resize(s_foff, 16);
                s_yadv <= '0';
                s_init_req <= '1';
            elsif s_timing.avid_end = '1' then
                s_yadv <= '1';
            elsif s_timing.hsync_start = '1' and s_yadv = '1' then
                s_yadv <= '0';
                s_yq4  <= s_yq4n;
                s_load_req <= '1';
            end if;
            if s_timing.avid_start = '1' then
                s_yq4n <= s_yq4 + resize(s_step, 16);
                s_comp_req <= '1';
            end if;
            s_ypx <= s_yq4(15 downto 4);

            case s_ls is
                when 0 =>
                    if s_seq_busy = '1' then
                        s_line_busy <= '0';
                    elsif s_init_req = '1' then
                        s_init_req <= '0';
                        s_init_mode <= '1';
                        s_line_busy <= '1';
                        s_lj <= (others => '0');
                        s_ls <= 1;
                    elsif s_comp_req = '1' then
                        s_comp_req <= '0';
                        s_line_busy <= '1';
                        s_lj <= (others => '0');
                        s_ls <= 10;
                    elsif s_load_req = '1' then
                        s_load_req <= '0';
                        s_line_busy <= '1';
                        s_lj <= (others => '0');
                        s_ls <= 40;
                    else
                        s_line_busy <= '0';
                    end if;

                -- init: compute for line 0, then load
                when 1 => s_ls <= 10;

                -- ---- COMPUTE ----
                when 10 =>
                    l_ra <= "00" & s_lj & "001";                         -- cy
                    s_ls <= 11;
                when 11 => s_ls <= 12;
                when 12 => s_ls <= 13;
                when 13 =>
                    -- d = yq4_next - 16*cy  (Q4 horizontal-pixel units)
                    v_d := signed(resize(s_yq4n, 18)) - signed(shift_left(resize(ct_rd2(11 downto 0), 18), 4));
                    v_abs := abs(v_d);
                    v_ad := unsigned(v_abs(15 downto 0));
                    s_qa <= resize(v_ad, 32);
                    s_qb <= v_ad;
                    s_qacc <= (others => '0');
                    s_lbit <= 0;
                    l_ra <= "00" & s_lj & "010";                         -- M0 lo
                    s_ls <= 14;
                when 14 =>
                    l_ra <= "00" & s_lj & "011";                         -- M0 hi
                    s_ls <= 15;
                when 15 => s_ls <= 16;
                when 16 =>
                    s_mllo <= ct_rd2;
                    s_ls <= 17;
                when 17 =>
                    -- M0 hi arrives; serial square runs 16 steps meanwhile
                    s_mline <= resize(signed(ct_rd2(8 downto 0)), 25);  -- sign-extended hi
                    s_ls <= 18;
                when 18 =>
                    if s_lbit = 15 then
                        s_ls <= 19;
                    end if;
                when 19 =>
                    -- mline = M0 + d^2/256
                    s_mline <= (s_mline(8 downto 0) & signed(s_mllo)) + signed(resize(s_qacc(31 downto 8), 25));
                    s_ls <= 20;
                when 20 =>
                    l_wa <= "00" & s_lj & "100"; l_wd <= unsigned(s_mline(15 downto 0)); l_we <= '1';
                    s_ls <= 21;
                when 21 =>
                    l_wa <= "00" & s_lj & "101"; l_wd <= unsigned(resize(s_mline(24 downto 16), 16)); l_we <= '1';
                    if s_lj = 7 then
                        if s_init_mode = '1' then
                            s_init_mode <= '0';
                            s_load_req  <= '1';   -- init: load line 0 next
                        end if;
                        s_ls <= 0;
                    else
                        s_lj <= s_lj + 1;
                        s_ls <= 10;
                    end if;

                -- ---- LOAD ----
                when 40 =>
                    l_ra <= "00" & s_lj & "100";                         -- mline lo
                    s_ls <= 41;
                when 41 =>
                    l_ra <= "00" & s_lj & "101";                         -- mline hi
                    s_ls <= 42;
                when 42 =>
                    l_ra <= "00" & s_lj & "000";                         -- cx
                    s_ls <= 43;
                when 43 =>
                    s_mllo <= ct_rd2;
                    s_ls <= 44;
                when 44 =>
                    s_mline <= (signed(resize(ct_rd2(8 downto 0), 9)) & signed(s_mllo));
                    s_ls <= 45;
                when 45 =>
                    l_ld_we  <= '1';
                    l_ld_idx <= s_lj;
                    l_ld_i   <= to_signed(1, 13) - shift_left(signed(resize(ct_rd2(11 downto 0), 13)), 1);
                    if s_lj = 7 then
                        s_ls <= 0;
                    else
                        s_lj <= s_lj + 1;
                        s_ls <= 40;
                    end if;

                when others =>
                    s_ls <= 0;
            end case;

            -- serial 16x16 square (runs from state 13 onward)
            if s_ls >= 14 and s_ls <= 18 then
                if s_qb(0) = '1' then
                    s_qacc <= s_qacc + s_qa;
                end if;
                s_qa <= s_qa(30 downto 0) & '0';
                s_qb <= '0' & s_qb(15 downto 1);
                if s_lbit /= 15 then s_lbit <= s_lbit + 1; end if;
            end if;
        end if;
    end process p_lines;

    -- ====================================================================
    -- Stage 0: per-circle m DDAs along the line (loaded by the line FSM)
    -- ====================================================================
    -- The increment 2(x-cx)+1 is a REGISTER computed from the column counter
    -- one cycle ahead (2*s_bcol + ix0), so m is aligned with s_bcol_d and
    -- the accumulator has no load/increment mux.
    p_dda : process(clk)
    begin
        if rising_edge(clk) then
            g_avid_d <= g_avid;
            s_bcol_d <= s_bcol;
            for j in 0 to C_N-1 loop
                s_incx(j) <= signed(resize(s_bcol & '0', 13)) + s_ix0(j);
                if g_avid_d = '1' then
                    s_m(j) <= s_m(j) + resize(s_incx(j), 25);
                elsif l_ld_we = '1' and to_integer(l_ld_idx) = j then
                    s_m(j)   <= s_mline;
                    s_ix0(j) <= l_ld_i;
                end if;
            end loop;
        end if;
    end process p_dda;

    -- ====================================================================
    -- Pixel pipeline, stages 1..22 (stage k data pairs with d_gv(k+1))
    -- ====================================================================
    p_pix : process(clk)
        variable v_sh  : signed(13 downto 0);
        variable v_D   : unsigned(24 downto 0);
        variable v_rim : signed(11 downto 0);
        variable v_y   : signed(11 downto 0);
        variable v_ss  : unsigned(2 downto 0);
        variable v_pi  : unsigned(7 downto 0);
        variable v_in, v_rm : std_logic;
        variable v_idx : unsigned(2 downto 0);
    begin
        if rising_edge(clk) then
            -- ---- stage 1: per-circle inside (sign bit) and rim flags; m copies ----
            for j in 0 to C_N-1 loop
                if j >= 4 then v_D := s_Dsml; else v_D := s_Dbig; end if;
                ins1(j) <= s_m(j)(24);
                if s_m(j)(24) = '0' and unsigned(s_m(j)) < v_D then rim1(j) <= '1'; else rim1(j) <= '0'; end if;
                m1c(j) <= s_m(j);
            end loop;
            -- ---- stage 2: owner = lowest-index circle we are inside (else the
            -- lowest-index rim); union rim = some rim and inside none ----
            v_in := '0'; v_rm := '0'; v_idx := (others => '0');
            for j in C_N-1 downto 0 loop
                if rim1(j) = '1' then v_rm := '1'; v_idx := to_unsigned(j, 3); end if;
            end loop;
            for j in C_N-1 downto 0 loop
                if ins1(j) = '1' then v_in := '1'; v_idx := to_unsigned(j, 3); end if;
            end loop;
            m2   <= m1c(to_integer(v_idx));
            idx2 <= v_idx;
            ins2 <= v_in;
            rim2 <= v_rm and not v_in;
            -- ---- stage 3 ----
            a3.m <= m2; a3.i <= idx2; ins3 <= ins2; rim3 <= rim2;

            -- ---- stage 4: fringe / dispersion operands; cx/cy lookup issues ----
            ins4 <= ins3;
            rim4 <= rim3;
            if a3.i(2) = '1' then
                mf4  <= a3.m - signed(s_Dfsml);
                r2_4 <= a3.m + signed(s_Rs2);
                fs4 <= s_fss; ds4 <= s_dssh;
            else
                mf4  <= a3.m - signed(s_Dfbig);
                r2_4 <= a3.m + signed(s_R2);
                fs4 <= s_fs; ds4 <= s_dsh;
            end if;
            if ins3 = '1' then keff4 <= s_k; else keff4 <= (others => '0'); end if;
            cls4 <= a3.i(2);

            -- ---- stage 5: barrels; lookups re-registered ----
            cx5 <= cxt_rd(11 downto 0);
            cy5 <= cyt_rd(11 downto 0);
            x5  <= dlX_rd2(10 downto 0);
            fr5 <= f_bsel8(mf4, fs4);
            if ins4 = '1' then
                disp5 <= f_bsel6(r2_4, ds4);
            else
                disp5 <= (others => '0');
            end if;
            zone5 <= rim4 and not mf4(24);
            rim5  <= rim4;
            ins5  <= ins4;
            cls5  <= cls4;
            keff5 <= keff4;

            -- ---- stage 6: lens operands; bundle A into its delay line ----
            dx6 <= signed(resize(x5, 13)) - signed(resize(cx5, 13));
            dy6 <= signed(resize(s_ypx, 13)) - signed(resize(cy5, 13));
            keff6 <= keff5;
            dlA_wd <= fr5 & disp5 & zone5 & rim5;
            rim6 <= rim5; zone6 <= zone5; ins6 <= ins5; cls6 <= cls5;
            x6 <= x5;

            -- ---- stage 7: lens multiplies (6-bit k split 3/3); shade operand ----
            dxk7a <= dx6 * signed('0' & keff6(5 downto 3));
            dxk7b <= dx6 * signed('0' & keff6(2 downto 0));
            dyk7a <= dy6 * signed('0' & keff6(5 downto 3));
            dyk7b <= dy6 * signed('0' & keff6(2 downto 0));
            sh7 <= -(resize(dx6, 14) + resize(dy6, 14));
            rim7 <= rim6; zone7 <= zone6; ins7 <= ins6; cls7 <= cls6;
            x7 <= x6;

            -- ---- stage 8 ----
            dxk8 <= shift_left(resize(dxk7a, 20), 3) + resize(dxk7b, 20);
            dyk8 <= shift_left(resize(dyk7a, 20), 3) + resize(dyk7b, 20);
            if cls7 = '1' then v_ss := s_sss; else v_ss := s_ss; end if;
            v_sh := f_sh14(sh7, v_ss);
            shade8 <= f_clamp8(v_sh);
            rim8 <= rim7; zone8 <= zone7; ins8 <= ins7;
            x8 <= x7;

            -- ---- stage 9: displaced coordinates; bundle B ----
            x9 <= signed(resize(x8, 13)) - resize(dxk8(18 downto 6), 13);
            y9 <= signed(resize(s_ypx, 13)) - resize(dyk8(18 downto 6), 13);
            dlB_wd <= unsigned(shade8) & rim8 & zone8 & ins8 & "00000";

            -- ---- stage 10: stripes swap ----
            if s_stripes = '1' then
                a10 <= y9; b10 <= x9;
            else
                a10 <= x9; b10 <= y9;
            end if;

            -- ---- stage 11: wave frequency ----
            bf11 <= b10 * signed('0' & s_fw);
            -- ---- stage 12: sine address (1.5 cycles per screen) ----
            wa12 <= unsigned(bf11(11 downto 4)) + s_ph16(15 downto 8);
            -- ---- stage 13: ROM, stage 14: sin_q2 (p_rom_sin) ----
            -- ---- stage 15: amplitude ----
            sa15 <= sin_q2 * signed('0' & s_A);
            -- ---- stage 16 ----
            sa16 <= sa15;
            -- ---- stage 17: displaced coordinate + wave (A in 4-px units) ----
            p17 <= resize(signed(dlC_rd2(12 downto 0)), 14) + resize(sa16(18 downto 6), 14);
            -- ---- stage 18: band mantissa ----
            pb18 <= p17 * signed('0' & s_bm);

            -- ---- stage 19: palette index (rim fringe overrides); dlA_rd2 is stage 18 ----
            if dlA_rd2(0) = '1' and dlA_rd2(1) = '1' then
                idx19 <= dlA_rd2(15 downto 8);
            else
                case s_be is
                    when "00"   => v_pi := unsigned(pb18(15 downto 8));
                    when "01"   => v_pi := unsigned(pb18(14 downto 7));
                    when "10"   => v_pi := unsigned(pb18(13 downto 6));
                    when others => v_pi := unsigned(pb18(12 downto 5));
                end case;
                idx19 <= v_pi + resize(dlA_rd2(7 downto 2), 8) + s_sc16(15 downto 8);
            end if;

            -- ---- stage 20: palette ROMs (p_rom_pal); stage 21 re-register ----
            pal21_y <= pal20_y;
            pal21_u <= pal20_u;
            pal21_v <= pal20_v;
            -- rim luma from the bundle-B shade (stage 20): black, lit to white top-left
            v_rim := resize(signed(dlB_rd2(15 downto 8)), 12) - to_signed(30, 12);
            v_rim := shift_left(v_rim, 3);
            if v_rim < 0 then
                rimy21 <= C_BLK;
            elsif v_rim > 680 then
                rimy21 <= to_unsigned(744, 10);
            else
                rimy21 <= unsigned(resize(v_rim, 10)) + C_BLK;
            end if;
            shade21 <= signed(dlB_rd2(15 downto 8));
            rim21   <= dlB_rd2(7);
            zone21  <= dlB_rd2(6);
            ins21   <= dlB_rd2(5);

            -- ---- stage 22: compose ----
            if rim21 = '1' then
                if zone21 = '1' then
                    c22_y <= pal21_y; c22_u <= pal21_u; c22_v <= pal21_v;
                else
                    c22_y <= rimy21; c22_u <= C_MID; c22_v <= C_MID;
                end if;
            elsif ins21 = '1' then
                v_y := signed(resize(pal21_y, 12)) + resize(shade21, 12);
                c22_y <= f_clamp_y(v_y);
                if s_chrome = '1' then
                    c22_u <= C_MID; c22_v <= C_MID;
                else
                    c22_u <= pal21_u; c22_v <= pal21_v;
                end if;
            else
                if s_solid = '1' then
                    if s_dark = '1' then
                        c22_y <= to_unsigned(186, 10); c22_u <= to_unsigned(423, 10); c22_v <= to_unsigned(532, 10);
                    else
                        c22_y <= to_unsigned(651, 10); c22_u <= to_unsigned(318, 10); c22_v <= to_unsigned(543, 10);
                    end if;
                else
                    if s_dark = '1' then
                        c22_y <= resize(pal21_y(9 downto 2), 10) + to_unsigned(48, 10);
                    else
                        c22_y <= resize(pal21_y(9 downto 1), 10) + to_unsigned(368, 10);
                    end if;
                    if s_chrome = '1' then
                        c22_u <= C_MID; c22_v <= C_MID;
                    else
                        c22_u <= resize(pal21_u(9 downto 1), 10) + to_unsigned(256, 10);
                        c22_v <= resize(pal21_v(9 downto 1), 10) + to_unsigned(256, 10);
                    end if;
                end if;
            end if;
        end if;
    end process p_pix;

    -- cx / cy lookups: one write (sequencer words 0 / 1), one read per pixel
    p_cxt : process(clk)
    begin
        if rising_edge(clk) then
            if q_we = '1' and s_seq_busy = '1' and q_wa(2 downto 0) = "000" then
                cxt_ram(to_integer(resize(q_wa(5 downto 3), 8))) <= q_wd;
            end if;
            cxt_rd <= cxt_ram(to_integer(resize(a3.i, 8)));
        end if;
    end process p_cxt;
    p_cyt : process(clk)
    begin
        if rising_edge(clk) then
            if q_we = '1' and s_seq_busy = '1' and q_wa(2 downto 0) = "001" then
                cyt_ram(to_integer(resize(q_wa(5 downto 3), 8))) <= q_wd;
            end if;
            cyt_rd <= cyt_ram(to_integer(resize(a3.i, 8)));
        end if;
    end process p_cyt;

    p_rom_py : process(clk) begin if rising_edge(clk) then pal20_y <= C_PAL_Y(to_integer(idx19)); end if; end process;
    p_rom_pu : process(clk) begin if rising_edge(clk) then pal20_u <= C_PAL_U(to_integer(idx19)); end if; end process;
    p_rom_pv : process(clk) begin if rising_edge(clk) then pal20_v <= C_PAL_V(to_integer(idx19)); end if; end process;

    -- delay-line EBRs
    p_dl : process(clk)
    begin
        if rising_edge(clk) then
            dl_wa <= dl_wa + 1;
            dlX_ram(to_integer(dl_wa)) <= resize(s_bcol_d, 16);
            dlA_ram(to_integer(dl_wa)) <= dlA_wd;
            dlB_ram(to_integer(dl_wa)) <= dlB_wd;
            dlC_ram(to_integer(dl_wa)) <= unsigned(resize(a10, 16));
            dlX_rd  <= dlX_ram(to_integer(dl_wa - C_DLX));
            dlA_rd  <= dlA_ram(to_integer(dl_wa - C_DLA));
            dlB_rd  <= dlB_ram(to_integer(dl_wa - C_DLB));
            dlC_rd  <= dlC_ram(to_integer(dl_wa - C_DLC));
            dlX_rd2 <= dlX_rd;
            dlA_rd2 <= dlA_rd;
            dlB_rd2 <= dlB_rd;
            dlC_rd2 <= dlC_rd;
        end if;
    end process p_dl;

    -- ====================================================================
    -- Output conditioning ([1 2 1]/4 on Y/U/V), blanking gate, output
    -- ====================================================================
    p_out : process(clk)
    begin
        if rising_edge(clk) then
            -- stage 23
            c22d_y <= c22_y; c22d_u <= c22_u; c22d_v <= c22_v;
            s23_y <= resize(c22_y, 11) + resize(c22d_y, 11);
            s23_u <= resize(c22_u, 11) + resize(c22d_u, 11);
            s23_v <= resize(c22_v, 11) + resize(c22d_v, 11);
            -- stage 24
            s23d_y <= s23_y; s23d_u <= s23_u; s23d_v <= s23_v;
            s24_y <= resize(shift_right(resize(s23_y, 12) + resize(s23d_y, 12), 2), 10);
            s24_u <= resize(shift_right(resize(s23_u, 12) + resize(s23d_u, 12), 2), 10);
            s24_v <= resize(shift_right(resize(s23_v, 12) + resize(s23d_v, 12), 2), 10);
            -- stage 25: blanking gate (stage-24 data pairs with d_gv(25))
            if d_gv(25) = '1' then
                g25_y <= s24_y; g25_u <= s24_u; g25_v <= s24_v;
            else
                g25_y <= C_BLK; g25_u <= C_MID; g25_v <= C_MID;
            end if;
            -- stage 26: padding to the 28-clock latency
            g26_y <= g25_y; g26_u <= g25_u; g26_v <= g25_v;
            -- stage 27 = 28th register after g_avid: output
            s_io.y <= std_logic_vector(g26_y);
            s_io.u <= std_logic_vector(g26_u);
            s_io.v <= std_logic_vector(g26_v);
            s_io.hsync_n <= d_hs(C_LAT - 1);
            s_io.vsync_n <= d_vs(C_LAT - 1);
            s_io.field_n <= d_fl(C_LAT - 1);
            s_io.avid    <= d_gv(C_LAT - 1);
        end if;
    end process p_out;

    p_delay : process(clk)
    begin
        if rising_edge(clk) then
            d_hs(1) <= data_in.hsync_n;
            d_vs(1) <= data_in.vsync_n;
            d_fl(1) <= data_in.field_n;
            d_gv(1) <= g_avid;
            for i in 2 to C_LAT - 1 loop
                d_hs(i) <= d_hs(i - 1);
                d_vs(i) <= d_vs(i - 1);
                d_fl(i) <= d_fl(i - 1);
                d_gv(i) <= d_gv(i - 1);
            end loop;
        end if;
    end process p_delay;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture dewdrop;
