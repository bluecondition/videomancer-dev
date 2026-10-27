
    -- argmin payload
    type t_mi is record
        m : signed(24 downto 0);
        i : unsigned(3 downto 0);
    end record;
    type t_mi_arr is array(natural range <>) of t_mi;

    function f_min(a, b : t_mi) return t_mi is
    begin
        if a.m <= b.m then return a; else return b; end if;
    end function;

    function f_ilog2(v : unsigned(24 downto 0)) return integer is
        variable r : integer range 0 to 24 := 0;
    begin
        for i in 0 to 24 loop
            if v(i) = '1' then r := i; end if;
        end loop;
        return r;
    end function;

    -- step multiply: v * step, step = 16 / 18 / 15 (cls 0/1/2), doubled if dbl
    function f_stepmul(v : signed(19 downto 0); cls : unsigned(1 downto 0); dbl : std_logic)
        return signed is
        variable t : signed(25 downto 0);
        variable w : signed(25 downto 0);
    begin
        w := resize(v, 26);
        case cls is
            when "01"   => t := shift_left(w, 4) + shift_left(w, 1);   -- 18
            when "10"   => t := shift_left(w, 4) - w;                  -- 15
            when others => t := shift_left(w, 4);                      -- 16
        end case;
        if dbl = '1' then t := shift_left(t, 1); end if;
        return t;
    end function;

    -- v * k/8 for k in 1..7 (seeding fractions)
    function f_frac8(v : unsigned(15 downto 0); k : unsigned(2 downto 0)) return unsigned is
        variable e : unsigned(15 downto 0);
        variable r : unsigned(15 downto 0);
    begin
        e := shift_right(v, 3);
        r := (others => '0');
        if k(0) = '1' then r := r + e; end if;
        if k(1) = '1' then r := r + shift_left(e, 1); end if;
        if k(2) = '1' then r := r + shift_left(e, 2); end if;
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

    -- ==== timing / raster ====
    signal s_timing  : t_video_timing_port;
    signal s_hcnt    : unsigned(11 downto 0) := (others => '0');
    signal s_vcnt    : unsigned(11 downto 0) := (others => '0');
    signal s_meas_h  : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_meas_v  : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal s_ilace   : std_logic := '0';
    signal s_botf    : std_logic := '0';
    signal s_fh      : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal s_scls    : unsigned(1 downto 0) := "00";           -- step class
    signal s_fw      : unsigned(7 downto 0) := to_unsigned(91, 8);
    signal s_w16     : unsigned(15 downto 0) := to_unsigned(1920*16, 16);
    signal s_h16     : unsigned(15 downto 0) := to_unsigned(1080*16, 16);
    signal s_stepf   : unsigned(5 downto 0) := to_unsigned(16, 6);   -- frame-line step
    signal s_step    : unsigned(6 downto 0) := to_unsigned(16, 7);   -- displayed-line step
    signal s_step2x2 : unsigned(12 downto 0) := to_unsigned(512, 13); -- 2*step^2
    signal s_foff    : unsigned(5 downto 0) := (others => '0');       -- field offset (Q4)

    -- ==== controls ====
    signal s_dark, s_pairs, s_stripes, s_solid, s_chrome : std_logic := '0';
    signal s_k5, s_k6 : unsigned(9 downto 0) := C_MID;
    signal s_saw    : std_logic := '0';
    signal s_vs_go  : std_logic := '0';
    signal s_a1, s_a2, s_a3, s_a4, s_a12 : unsigned(13 downto 0) := (others => '0');
    signal s_k1s, s_k2s, s_k3s, s_k4s, s_p12s : unsigned(9 downto 0) := (others => '0');

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');
    signal g_wm1  : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal s_bcol : unsigned(10 downto 0) := (others => '0');

    -- ==== per-frame constants (sequencer outputs) ====
    signal s_bs    : unsigned(8 downto 0) := to_unsigned(64, 9);     -- band scale
    signal s_k     : unsigned(7 downto 0) := to_unsigned(128, 8);    -- refract
    signal s_kd    : unsigned(5 downto 0) := to_unsigned(32, 6);     -- dispersion
    signal s_A     : unsigned(9 downto 0) := to_unsigned(300, 10);   -- wave amplitude
    signal s_R     : unsigned(10 downto 0) := to_unsigned(300, 11);
    signal s_Rs    : unsigned(10 downto 0) := to_unsigned(180, 11);
    signal s_R2, s_Rs2 : unsigned(24 downto 0) := (others => '0');
    signal s_w     : unsigned(8 downto 0) := to_unsigned(16, 9);
    signal s_Dbig, s_Dsml, s_Dfbig, s_Dfsml : unsigned(24 downto 0) := (others => '0');
    signal s_fs, s_fss, s_dsh, s_dssh : unsigned(3 downto 0) := (others => '0');
    signal s_ss, s_sss : unsigned(2 downto 0) := (others => '0');
    signal s_nsub  : unsigned(6 downto 0) := to_unsigned(8, 7);
    signal s_fad   : unsigned(9 downto 0) := (others => '0');
    signal s_fneg  : std_logic := '0';
    signal s_fq    : unsigned(9 downto 0) := (others => '0');
    signal s_ph16  : unsigned(15 downto 0) := (others => '0');       -- wave phase
    signal s_sc16  : unsigned(15 downto 0) := (others => '0');       -- band scroll
    signal s_Rmin  : unsigned(10 downto 0) := to_unsigned(60, 11);
    signal s_Wh    : unsigned(10 downto 0) := to_unsigned(960, 11);
    signal s_WhR   : unsigned(10 downto 0) := to_unsigned(900, 11);
    signal s_seeded : std_logic := '0';

    -- circle state
    type t_u16a is array(0 to C_N-1) of unsigned(15 downto 0);
    type t_u12a is array(0 to C_N-1) of unsigned(11 downto 0);
    type t_u13a is array(0 to C_N-1) of unsigned(12 downto 0);
    type t_s13a is array(0 to C_N-1) of signed(12 downto 0);
    type t_s25a is array(0 to C_N-1) of signed(24 downto 0);
    type t_s24a is array(0 to C_N-1) of signed(23 downto 0);
    type t_u31a is array(0 to C_N-1) of unsigned(30 downto 0);
    type t_sl_a is array(0 to C_N-1) of std_logic;
    type t_dir  is array(0 to C_N-1) of integer;
    type t_k8   is array(0 to C_N-1) of unsigned(2 downto 0);
    constant C_DIRX : t_dir := (3, -2, 1, 2, -3, 1, -2, 3, -1, 2);
    constant C_DIRY : t_dir := (1, 2, -3, 2, -1, 2, -1, -2, 3, -3);
    constant C_FX   : t_k8 := ("001", "100", "111", "010", "110", "011", "101", "001", "111", "100");
    constant C_FY   : t_k8 := ("010", "011", "010", "110", "110", "001", "110", "111", "111", "001");

    signal s_px, s_py : t_u16a := (others => (others => '0'));     -- Q4 positions
    signal s_sgx, s_sgy : t_sl_a := (others => '0');              -- direction sign flips
    signal s_cx, s_cy : t_u12a := (others => (others => '0'));
    signal s_cxy  : t_u13a := (others => (others => '0'));
    signal s_M0   : t_s25a := (others => (others => '0'));
    signal s_dy2i : t_u31a := (others => (others => '0'));        -- dy^2 at line 0 (Q8)
    signal s_iy0  : t_s24a := (others => (others => '0'));        -- incy at line 0
    signal s_ix0  : t_s13a := (others => (others => '0'));        -- incx at column 0

    -- sequencer
    signal s_op   : integer range 0 to 63 := 63;
    signal s_bit  : integer range 0 to 15 := 0;
    signal s_mA   : unsigned(23 downto 0) := (others => '0');
    signal s_mB   : unsigned(11 downto 0) := (others => '0');
    signal s_acc  : unsigned(35 downto 0) := (others => '0');
    signal s_cj   : integer range 0 to C_N-1 := 0;
    signal s_sub  : unsigned(6 downto 0) := (others => '0');
    signal s_tx, s_ty : unsigned(15 downto 0) := (others => '0');
    signal s_cx2, s_cy2 : unsigned(23 downto 0) := (others => '0');
    signal s_seed_now : std_logic := '0';
    signal s_seq_done : std_logic := '0';
    attribute fsm_encoding : string;
    attribute fsm_encoding of s_op : signal is "none";

    -- ==== line state ====
    signal s_yadv   : std_logic := '0';
    signal s_hs_d   : std_logic := '0';
    signal s_yq4    : unsigned(15 downto 0) := (others => '0');
    signal s_ypx    : unsigned(11 downto 0) := (others => '0');
    signal s_dy2    : t_u31a := (others => (others => '0'));
    signal s_incy   : t_s24a := (others => (others => '0'));
    signal s_mline  : t_s25a := (others => (others => '0'));

    -- ==== stage 0: DDAs ====
    signal s_m    : t_s25a := (others => (others => '0'));
    signal s_incx : t_s13a := (others => (others => '0'));

    -- ==== pixel pipeline ====
    signal a1 : t_mi_arr(0 to 4);
    signal a2 : t_mi_arr(0 to 2);
    signal a3 : t_mi_arr(0 to 1);
    signal a4 : t_mi;
    signal m1c : t_s25a := (others => (others => '0'));
    signal r1_idx, r2_idx : unsigned(3 downto 0) := (others => '0');
    signal r1_any, r2_any, r3_any, r4_any, r5_any, r6_any : std_logic := '0';
    signal r2_m, r3_m : signed(24 downto 0) := (others => '0');
    signal r3_cxy : unsigned(12 downto 0) := (others => '0');
    signal r3_cls, r4_cls : std_logic := '0';
    signal r4_mf  : signed(24 downto 0) := (others => '0');
    signal r4_sh  : signed(13 downto 0) := (others => '0');
    signal r5_fr, r6_fr : unsigned(7 downto 0) := (others => '0');
    signal r5_zone, r6_zone : std_logic := '0';
    signal r5_shade, r6_shade : signed(7 downto 0) := (others => '0');

    type t_x_arr is array(1 to 8) of unsigned(10 downto 0);
    signal xd : t_x_arr := (others => (others => '0'));
    signal xy3 : unsigned(12 downto 0) := (others => '0');
    signal xy4, xy5 : unsigned(12 downto 0) := (others => '0');

    signal cx5, cy5 : unsigned(11 downto 0) := (others => '0');
    signal cxy5 : unsigned(12 downto 0) := (others => '0');
    signal m5 : signed(24 downto 0) := (others => '0');
    signal ins5, ins6, cls5, cls6 : std_logic := '0';
    signal dx6, dy6 : signed(12 downto 0) := (others => '0');
    signal r2_6 : signed(24 downto 0) := (others => '0');
    signal keff6 : unsigned(7 downto 0) := (others => '0');
    signal sh6 : signed(13 downto 0) := (others => '0');
    signal dxk7a, dxk7b, dyk7a, dyk7b : signed(17 downto 0) := (others => '0');
    signal disp7 : unsigned(7 downto 0) := (others => '0');
    signal shade7, shade8 : signed(7 downto 0) := (others => '0');
    signal dxk8, dyk8 : signed(20 downto 0) := (others => '0');
    signal disp8 : unsigned(13 downto 0) := (others => '0');
    signal y8s : signed(12 downto 0) := (others => '0');
    signal x9, y9 : signed(12 downto 0) := (others => '0');
    signal disp9 : unsigned(5 downto 0) := (others => '0');
    signal a10, b10 : signed(12 downto 0) := (others => '0');
    signal bf11a, bf11b : signed(17 downto 0) := (others => '0');
    signal bf12 : signed(20 downto 0) := (others => '0');
    signal wa13 : unsigned(7 downto 0) := (others => '0');
    signal sin14 : unsigned(9 downto 0) := (others => '0');
    signal sin15 : signed(9 downto 0) := (others => '0');
    signal sa16a, sa16b : signed(15 downto 0) := (others => '0');
    signal sa17 : signed(19 downto 0) := (others => '0');
    signal p18 : signed(13 downto 0) := (others => '0');
    signal pb19a : signed(18 downto 0);
    signal pb19b : signed(19 downto 0) := (others => '0');
    signal pb20 : signed(23 downto 0) := (others => '0');
    signal idx21 : unsigned(7 downto 0) := (others => '0');
    signal pal22_y, pal22_u, pal22_v : unsigned(9 downto 0) := (others => '0');
    signal pal23_y, pal23_u, pal23_v : unsigned(9 downto 0) := (others => '0');
    signal rimy23 : unsigned(9 downto 0) := C_BLK;
    signal c24_y : unsigned(9 downto 0) := C_BLK;
    signal c24_u, c24_v : unsigned(9 downto 0) := C_MID;
    signal c24d_y : unsigned(9 downto 0) := C_BLK;
    signal c24d_u, c24d_v : unsigned(9 downto 0) := C_MID;
    signal s25_y, s25_u, s25_v : unsigned(10 downto 0) := (others => '0');
    signal s25d_y, s25d_u, s25d_v : unsigned(10 downto 0) := (others => '0');
    signal s26_y : unsigned(9 downto 0) := C_BLK;
    signal s26_u, s26_v : unsigned(9 downto 0) := C_MID;
    signal g27_y : unsigned(9 downto 0) := C_BLK;
    signal g27_u, g27_v : unsigned(9 downto 0) := C_MID;

    -- delay chains for late consumers
    type t_a_arr is array(11 to 17) of signed(12 downto 0);
    signal ad : t_a_arr := (others => (others => '0'));
    type t_d6_arr is array(10 to 20) of unsigned(5 downto 0);
    signal dispd : t_d6_arr := (others => (others => '0'));
    type t_fr_arr is array(7 to 20) of unsigned(7 downto 0);
    signal frd : t_fr_arr := (others => (others => '0'));
    type t_sh_arr is array(7 to 22) of signed(7 downto 0);
    signal rshd : t_sh_arr := (others => (others => '0'));
    type t_sh2_arr is array(9 to 23) of signed(7 downto 0);
    signal shd : t_sh2_arr := (others => (others => '0'));
    type t_fl_arr is array(7 to 23) of std_logic;
    signal anyd, zoned, insd : t_fl_arr := (others => '0');
    type t_fl2_arr is array(7 to 20) of std_logic;
    signal zonef, anyf : t_fl2_arr := (others => '0');

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
                s_fh  <= v_fh;
                s_w16 <= s_meas_h & "0000";
                if s_meas_h >= to_unsigned(1000, 12) then       -- HD / 720p: square
                    s_scls <= "00";
                    s_stepf <= to_unsigned(16, 6);
                    s_h16 <= v_fh & "0000";
                    if s_meas_h >= to_unsigned(1600, 12) then
                        s_fw <= to_unsigned(91, 8);
                    else
                        s_fw <= to_unsigned(137, 8);
                    end if;
                elsif v_fh >= to_unsigned(540, 12) then          -- PAL: 15/16
                    s_scls <= "10";
                    s_stepf <= to_unsigned(15, 6);
                    s_h16 <= (v_fh & "0000") - v_fh;
                    s_fw <= to_unsigned(182, 8);
                else                                            -- NTSC: 18/16
                    s_scls <= "01";
                    s_stepf <= to_unsigned(18, 6);
                    s_h16 <= (v_fh & "0000") + (v_fh & '0');
                    s_fw <= to_unsigned(182, 8);
                end if;
            elsif s_timing.avid_start = '1' then
                s_vcnt <= s_vcnt + 1;
            end if;

            -- displayed-line step and 2*step^2
            if s_ilace = '1' then
                s_step <= s_stepf & '0';
                case s_scls is
                    when "01"   => s_step2x2 <= to_unsigned(2592, 13);
                    when "10"   => s_step2x2 <= to_unsigned(1800, 13);
                    when others => s_step2x2 <= to_unsigned(2048, 13);
                end case;
                if s_botf = '1' then s_foff <= s_stepf; else s_foff <= (others => '0'); end if;
            else
                s_step <= '0' & s_stepf;
                case s_scls is
                    when "01"   => s_step2x2 <= to_unsigned(648, 13);
                    when "10"   => s_step2x2 <= to_unsigned(450, 13);
                    when others => s_step2x2 <= to_unsigned(512, 13);
                end case;
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

            s_k1s  <= s_a1(13 downto 4);
            s_k2s  <= s_a2(13 downto 4);
            s_k3s  <= s_a3(13 downto 4);
            s_k4s  <= s_a4(13 downto 4);
            s_p12s <= s_a12(13 downto 4);

            -- Flow: bipolar deadband +-32
            v_d := resize(signed('0' & s_k6), 12) - to_signed(512, 12);
            if v_d < 0 then v_a := resize(unsigned(-v_d), 11); s_fneg <= '1';
            else            v_a := resize(unsigned(v_d), 11);  s_fneg <= '0'; end if;
            if v_a > to_unsigned(32, 11) then s_fad <= resize(v_a - 32, 10);
            else                              s_fad <= (others => '0'); end if;

            s_nsub <= resize(s_k5(9 downto 4), 7) + 1;
            s_bs   <= to_unsigned(17, 9) + resize(s_k1s(9 downto 2), 9);
            s_k    <= s_k3s(9 downto 2);
            s_kd   <= s_k3s(9 downto 4);
            s_Wh   <= s_meas_h(11 downto 1);
            s_Rmin <= resize(s_meas_h(11 downto 5), 11);
        end if;
    end process p_frame;

    -- ====================================================================
    -- Vblank sequencer: slews, shared serial multiplier, circle constants,
    -- drift sub-steps.  ~1900 cycles worst case, all inside vblank.
    -- ====================================================================
    p_seq : process(clk)
        variable v_t   : unsigned(13 downto 0);
        variable v_d   : signed(14 downto 0);
        variable v_n   : signed(15 downto 0);
        variable v_dq  : signed(19 downto 0);
        variable v_nx, v_ny : signed(17 downto 0);
        variable v_dx, v_dy : signed(17 downto 0);
        variable v_l   : integer range 0 to 24;
        variable v_cy  : unsigned(11 downto 0);
        variable v_cx  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            s_seq_done <= '0';
            if s_vs_go = '1' then
                s_op  <= 0;
                s_bit <= 0;
            elsif s_timing.avid = '0' then
                case s_op is
                    -- 1-pole slews (Q10.4): acc += (target - acc)/16
                    when 0 =>
                        v_t := unsigned(registers_in(0)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a1);
                        v_n := resize(signed('0' & s_a1), 16) + resize(shift_right(v_d, 4), 16);
                        s_a1 <= unsigned(v_n(13 downto 0));
                        s_op <= 1;
                    when 1 =>
                        v_t := unsigned(registers_in(1)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a2);
                        v_n := resize(signed('0' & s_a2), 16) + resize(shift_right(v_d, 4), 16);
                        s_a2 <= unsigned(v_n(13 downto 0));
                        s_op <= 2;
                    when 2 =>
                        v_t := unsigned(registers_in(2)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a3);
                        v_n := resize(signed('0' & s_a3), 16) + resize(shift_right(v_d, 4), 16);
                        s_a3 <= unsigned(v_n(13 downto 0));
                        s_op <= 3;
                    when 3 =>
                        v_t := unsigned(registers_in(3)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a4);
                        v_n := resize(signed('0' & s_a4), 16) + resize(shift_right(v_d, 4), 16);
                        s_a4 <= unsigned(v_n(13 downto 0));
                        s_op <= 4;
                    when 4 =>
                        v_t := unsigned(registers_in(7)) & "0000";
                        v_d := signed('0' & v_t) - signed('0' & s_a12);
                        v_n := resize(signed('0' & s_a12), 16) + resize(shift_right(v_d, 4), 16);
                        s_a12 <= unsigned(v_n(13 downto 0));
                        s_op <= 5;
                    when 5 =>
                        s_WhR <= s_Wh - s_Rmin;
                        if s_seeded = '0' and s_meas_h >= 100 and s_fh >= 100 then
                            s_seed_now <= '1';
                            s_seeded   <= '1';
                        else
                            s_seed_now <= '0';
                        end if;
                        s_op <= 6;

                    -- shared multiplier: 12-bit mB consumed at bits 1..12,
                    -- store at bit 13
                    when 6 to 14 =>
                        if s_bit = 0 then
                            s_acc <= (others => '0');
                            case s_op is
                                when 6      => s_mA <= resize(s_WhR, 24);        s_mB <= resize(s_p12s, 12);
                                when 7      => s_mA <= resize(s_R, 24);          s_mB <= resize(s_R, 12);
                                when 8      => s_mA <= resize(s_R, 24);          s_mB <= to_unsigned(154, 12);
                                when 9      => s_mA <= resize(s_Rs, 24);         s_mB <= resize(s_Rs, 12);
                                when 10     => s_mA <= resize(s_R, 24);          s_mB <= resize(s_k4s, 12);
                                when 11     => s_mA <= resize(s_R + s_w, 24);    s_mB <= resize(s_R + s_w, 12);
                                when 12     => s_mA <= resize(s_Rs + s_w, 24);   s_mB <= resize(s_Rs + s_w, 12);
                                when 13     => s_mA <= resize(s_Wh, 24);         s_mB <= resize(s_k2s, 12);
                                when others => s_mA <= resize(s_fad, 24);        s_mB <= resize(s_fad, 12);
                            end case;
                            s_bit <= 1;
                        else
                            if s_mB(0) = '1' then
                                s_acc <= s_acc + resize(s_mA, 36);
                            end if;
                            s_mA <= s_mA(22 downto 0) & '0';
                            s_mB <= '0' & s_mB(11 downto 1);
                            if s_bit = 13 then
                                s_bit <= 0;
                                s_op  <= s_op + 1;
                                case s_op is
                                    when 6      => s_R   <= s_Rmin + resize(s_acc(20 downto 10), 11);
                                    when 7      => s_R2  <= resize(s_acc(24 downto 0), 25);
                                    when 8      => s_Rs  <= resize(s_acc(18 downto 8), 11);
                                    when 9      => s_Rs2 <= resize(s_acc(24 downto 0), 25);
                                    when 10     => s_w   <= to_unsigned(2, 9) + resize(s_acc(20 downto 12), 9);
                                    when 11     => s_Dbig   <= resize(s_acc(24 downto 0), 25) - s_R2;
                                    when 12     => s_Dsml  <= resize(s_acc(24 downto 0), 25) - s_Rs2;
                                    when 13     => s_A   <= resize(s_acc(19 downto 10), 10);
                                    when others => s_fq  <= resize(s_acc(17 downto 8), 10);
                                end case;
                            else
                                s_bit <= s_bit + 1;
                            end if;
                        end if;

                    when 15 =>
                        s_Dfbig  <= s_Dbig - shift_right(s_Dbig, 2);
                        s_Dfsml <= s_Dsml - shift_right(s_Dsml, 2);
                        s_op <= 16;
                    when 16 =>
                        v_l := f_ilog2(shift_right(s_Dbig, 2));
                        if v_l < 7 then s_fs <= (others => '0');
                        elsif v_l > 22 then s_fs <= to_unsigned(15, 4);
                        else s_fs <= to_unsigned(v_l - 7, 4); end if;
                        s_op <= 17;
                    when 17 =>
                        v_l := f_ilog2(shift_right(s_Dsml, 2));
                        if v_l < 7 then s_fss <= (others => '0');
                        elsif v_l > 22 then s_fss <= to_unsigned(15, 4);
                        else s_fss <= to_unsigned(v_l - 7, 4); end if;
                        s_op <= 18;
                    when 18 =>
                        v_l := f_ilog2(s_R2);
                        if v_l < 7 then s_dsh <= (others => '0');
                        elsif v_l > 22 then s_dsh <= to_unsigned(15, 4);
                        else s_dsh <= to_unsigned(v_l - 7, 4); end if;
                        s_op <= 19;
                    when 19 =>
                        v_l := f_ilog2(s_Rs2);
                        if v_l < 7 then s_dssh <= (others => '0');
                        elsif v_l > 22 then s_dssh <= to_unsigned(15, 4);
                        else s_dssh <= to_unsigned(v_l - 7, 4); end if;
                        s_op <= 20;
                    when 20 =>
                        v_l := f_ilog2(resize(s_R, 25));
                        if v_l < 6 then s_ss <= (others => '0');
                        elsif v_l > 13 then s_ss <= to_unsigned(7, 3);
                        else s_ss <= to_unsigned(v_l - 6, 3); end if;
                        v_l := f_ilog2(resize(s_Rs, 25));
                        if v_l < 6 then s_sss <= (others => '0');
                        elsif v_l > 13 then s_sss <= to_unsigned(7, 3);
                        else s_sss <= to_unsigned(v_l - 6, 3); end if;
                        s_sub <= (others => '0');
                        s_cj  <= 0;
                        s_op  <= 21;

                    -- drift: nsub sub-steps x 10 circles, 2 cycles each
                    when 21 =>
                        if s_seed_now = '1' then
                            s_px(s_cj) <= f_frac8(s_w16, C_FX(s_cj));
                            s_py(s_cj) <= f_frac8(s_h16, C_FY(s_cj));
                            s_sgx(s_cj) <= '0';
                            s_sgy(s_cj) <= '0';
                        end if;
                        s_tx <= s_px(s_cj);
                        s_ty <= s_py(s_cj);
                        s_op <= 22;
                    when 22 =>
                        if s_sgx(s_cj) = '1' then v_dx := to_signed(-C_DIRX(s_cj), 18);
                        else                      v_dx := to_signed(C_DIRX(s_cj), 18); end if;
                        if s_sgy(s_cj) = '1' then v_dy := to_signed(-C_DIRY(s_cj), 18);
                        else                      v_dy := to_signed(C_DIRY(s_cj), 18); end if;
                        v_nx := signed(resize(s_tx, 18)) + v_dx;
                        v_ny := signed(resize(s_ty, 18)) + v_dy;
                        if s_seed_now = '0' then
                            if v_nx < 0 or v_nx >= signed(resize(s_w16, 18)) then
                                s_sgx(s_cj) <= not s_sgx(s_cj);
                            else
                                s_px(s_cj) <= unsigned(v_nx(15 downto 0));
                            end if;
                            if v_ny < 0 or v_ny >= signed(resize(s_h16, 18)) then
                                s_sgy(s_cj) <= not s_sgy(s_cj);
                            else
                                s_py(s_cj) <= unsigned(v_ny(15 downto 0));
                            end if;
                        end if;
                        if s_cj = C_N - 1 then
                            s_cj <= 0;
                            if s_sub + 1 >= s_nsub then
                                s_op <= 23;
                            else
                                s_sub <= s_sub + 1;
                                s_op  <= 21;
                            end if;
                        else
                            s_cj <= s_cj + 1;
                            s_op <= 21;
                        end if;

                    -- circle constants, one circle per pass
                    when 23 =>
                        if s_pairs = '1' and s_cj >= 5 then
                            v_cx := s_cx(s_cj - 5);
                            v_cy := s_cy(s_cj - 5) + resize(s_R, 12) - resize(shift_right(s_R, 3), 12);
                        else
                            v_cx := s_px(s_cj)(15 downto 4);
                            v_cy := s_py(s_cj)(15 downto 4);
                        end if;
                        s_cx(s_cj)  <= v_cx;
                        s_cy(s_cj)  <= v_cy;
                        s_cxy(s_cj) <= resize(v_cx, 13) + resize(v_cy, 13);
                        s_ix0(s_cj) <= to_signed(1, 13) - shift_left(signed(resize(v_cx, 13)), 1);
                        s_mA <= resize(v_cx, 24);
                        s_mB <= v_cx;
                        s_acc <= (others => '0');
                        s_bit <= 1;
                        s_op <= 24;
                    when 24 =>                       -- cx^2
                        if s_mB(0) = '1' then
                            s_acc <= s_acc + resize(s_mA, 36);
                        end if;
                        s_mA <= s_mA(22 downto 0) & '0';
                        s_mB <= '0' & s_mB(11 downto 1);
                        if s_bit = 13 then
                            s_bit <= 0;
                            s_op  <= 25;
                        else
                            s_bit <= s_bit + 1;
                        end if;
                    when 25 =>
                        s_cx2 <= s_acc(23 downto 0);
                        s_mA <= resize(s_cy(s_cj), 24);
                        s_mB <= s_cy(s_cj);
                        s_acc <= (others => '0');
                        s_bit <= 1;
                        s_op <= 26;
                    when 26 =>                       -- cy^2
                        if s_mB(0) = '1' then
                            s_acc <= s_acc + resize(s_mA, 36);
                        end if;
                        s_mA <= s_mA(22 downto 0) & '0';
                        s_mB <= '0' & s_mB(11 downto 1);
                        if s_bit = 13 then
                            s_bit <= 0;
                            s_op  <= 27;
                        else
                            s_bit <= s_bit + 1;
                        end if;
                    when 27 =>
                        s_cy2 <= s_acc(23 downto 0);
                        if s_cj >= 5 then
                            s_M0(s_cj) <= signed(resize(s_cx2, 25)) - signed(s_Rs2);
                        else
                            s_M0(s_cj) <= signed(resize(s_cx2, 25)) - signed(s_R2);
                        end if;
                        s_op <= 28;
                    when 28 =>
                        -- dy_q4 at line 0 = foff - 16*cy
                        v_dq := resize(signed('0' & s_foff), 20) - signed(shift_left(resize(s_cy(s_cj), 20), 4));
                        -- incy0 = step * (2*dy0 + step)
                        s_iy0(s_cj) <= resize(f_stepmul(resize(shift_left(v_dq, 1), 20) + resize(signed('0' & s_step), 20), s_scls, s_ilace), 24);
                        -- dy0^2 = (16cy)^2 + stepf*(stepf - 32cy)   (foff is 0 or stepf)
                        if s_foff /= 0 then
                            s_dy2i(s_cj) <= unsigned(signed(shift_left(resize(s_cy2, 31), 8))
                                            + resize(f_stepmul(resize(signed('0' & s_stepf), 20) - signed(shift_left(resize(s_cy(s_cj), 20), 5)), s_scls, '0'), 31));
                        else
                            s_dy2i(s_cj) <= shift_left(resize(s_cy2, 31), 8);
                        end if;
                        if s_cj = C_N - 1 then
                            s_cj <= 0;
                            s_op <= 29;
                        else
                            s_cj <= s_cj + 1;
                            s_op <= 23;
                        end if;
                    when 29 =>
                        s_seq_done <= '1';
                        s_op <= 63;
                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process p_seq;

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
    -- Line state: y (Q4 px), per-circle dy^2 DDAs, line-start loads.
    -- Advance on ACTIVE lines only: flag at avid end, act at the next
    -- hsync (the pipe has drained by then), load mline one clock later.
    -- ====================================================================
    p_lines : process(clk)
    begin
        if rising_edge(clk) then
            s_hs_d <= s_timing.hsync_start;
            if s_seq_done = '1' then
                s_yq4  <= resize(s_foff, 16);
                s_yadv <= '0';
                for j in 0 to C_N-1 loop
                    s_dy2(j)  <= s_dy2i(j);
                    s_incy(j) <= s_iy0(j);
                end loop;
            elsif s_timing.avid_end = '1' then
                s_yadv <= '1';
            elsif s_timing.hsync_start = '1' and s_yadv = '1' then
                s_yadv <= '0';
                s_yq4  <= s_yq4 + resize(s_step, 16);
                for j in 0 to C_N-1 loop
                    s_dy2(j)  <= unsigned(signed(s_dy2(j)) + resize(s_incy(j), 31));
                    s_incy(j) <= s_incy(j) + signed(resize(s_step2x2, 24));
                end loop;
            end if;
            s_ypx <= s_yq4(15 downto 4);
            if s_hs_d = '1' then
                for j in 0 to C_N-1 loop
                    s_mline(j) <= s_M0(j) + signed(resize(s_dy2(j)(30 downto 8), 25));
                end loop;
            end if;
        end if;
    end process p_lines;

    -- ====================================================================
    -- Stage 0: per-circle m DDAs along the line
    -- ====================================================================
    p_dda : process(clk)
    begin
        if rising_edge(clk) then
            for j in 0 to C_N-1 loop
                if g_avid = '1' then
                    s_m(j)    <= s_m(j) + resize(s_incx(j), 25);
                    s_incx(j) <= s_incx(j) + 2;
                else
                    s_m(j)    <= s_mline(j);
                    s_incx(j) <= s_ix0(j);
                end if;
            end loop;
        end if;
    end process p_dda;

    -- ====================================================================
    -- Pixel pipeline, stages 1..24
    -- ====================================================================
    p_pix : process(clk)
        variable v_r   : std_logic_vector(0 to C_N-1);
        variable v_idx : unsigned(3 downto 0);
        variable v_any : std_logic;
        variable v_D   : unsigned(24 downto 0);
        variable v_sh  : signed(13 downto 0);
        variable v_t17 : signed(16 downto 0);
        variable v_rim : signed(11 downto 0);
        variable v_y   : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- x delay chain
            xd(1) <= s_bcol;
            for i in 2 to 8 loop xd(i) <= xd(i-1); end loop;

            -- ---- stage 1: argmin level 1, rim flags + priority index ----
            for p in 0 to 4 loop
                a1(p) <= f_min((m => s_m(2*p),   i => to_unsigned(2*p, 4)),
                               (m => s_m(2*p+1), i => to_unsigned(2*p+1, 4)));
            end loop;
            v_any := '0';
            v_idx := (others => '0');
            for j in C_N-1 downto 0 loop
                if j >= 5 then v_D := s_Dsml; else v_D := s_Dbig; end if;
                if s_m(j)(24) = '0' and unsigned(s_m(j)) < v_D then
                    v_r(j) := '1';
                else
                    v_r(j) := '0';
                end if;
                if v_r(j) = '1' then
                    v_any := '1';
                    v_idx := to_unsigned(j, 4);
                end if;
                m1c(j) <= s_m(j);
            end loop;
            r1_idx <= v_idx;
            r1_any <= v_any;

            -- ---- stage 2 ----
            a2(0) <= f_min(a1(0), a1(1));
            a2(1) <= f_min(a1(2), a1(3));
            a2(2) <= a1(4);
            r2_m   <= m1c(to_integer(r1_idx));
            r2_idx <= r1_idx;
            r2_any <= r1_any;

            -- ---- stage 3 ----
            a3(0) <= f_min(a2(0), a2(1));
            a3(1) <= a2(2);
            r3_m   <= r2_m;
            r3_cxy <= s_cxy(to_integer(r2_idx));
            if r2_idx >= 5 then r3_cls <= '1'; else r3_cls <= '0'; end if;
            r3_any <= r2_any;
            xy3 <= resize(xd(2), 13) + resize(s_ypx, 13);

            -- ---- stage 4 ----
            a4 <= f_min(a3(0), a3(1));
            if r3_cls = '1' then
                r4_mf <= r3_m - signed(s_Dfsml);
            else
                r4_mf <= r3_m - signed(s_Dfbig);
            end if;
            r4_sh  <= signed(resize(r3_cxy, 14)) - signed(resize(xy3, 14));
            r4_cls <= r3_cls;
            r4_any <= r3_any;
            xy4 <= xy3;

            -- ---- stage 5 ----
            cx5  <= s_cx(to_integer(a4.i));
            cy5  <= s_cy(to_integer(a4.i));
            cxy5 <= s_cxy(to_integer(a4.i));
            m5   <= a4.m;
            ins5 <= a4.m(24);
            if a4.i >= 5 then cls5 <= '1'; else cls5 <= '0'; end if;
            xy5 <= xy4;
            if r4_cls = '1' then
                r5_fr <= unsigned(shift_right(r4_mf, to_integer(s_fss))(7 downto 0));
                v_sh := shift_right(r4_sh, to_integer(s_sss));
            else
                r5_fr <= unsigned(shift_right(r4_mf, to_integer(s_fs))(7 downto 0));
                v_sh := shift_right(r4_sh, to_integer(s_ss));
            end if;
            r5_shade <= f_clamp8(v_sh);
            r5_zone  <= not r4_mf(24);
            r5_any   <= r4_any;

            -- ---- stage 6 ----
            dx6 <= signed(resize(xd(5), 13)) - signed(resize(cx5, 13));
            dy6 <= signed(resize(s_ypx, 13)) - signed(resize(cy5, 13));
            if cls5 = '1' then
                r2_6 <= m5 + signed(s_Rs2);
            else
                r2_6 <= m5 + signed(s_R2);
            end if;
            if ins5 = '1' then keff6 <= s_k; else keff6 <= (others => '0'); end if;
            sh6  <= signed(resize(cxy5, 14)) - signed(resize(xy5, 14));
            ins6 <= ins5;
            cls6 <= cls5;
            r6_fr    <= r5_fr;
            r6_zone  <= r5_zone;
            r6_shade <= r5_shade;
            r6_any   <= r5_any;

            -- ---- stage 7: lens multiplies (split hi/lo), dispersion, shade ----
            dxk7a <= dx6 * signed('0' & keff6(7 downto 4));
            dxk7b <= dx6 * signed('0' & keff6(3 downto 0));
            dyk7a <= dy6 * signed('0' & keff6(7 downto 4));
            dyk7b <= dy6 * signed('0' & keff6(3 downto 0));
            if ins6 = '1' then
                if cls6 = '1' then
                    disp7 <= unsigned(shift_right(r2_6, to_integer(s_dssh))(7 downto 0));
                else
                    disp7 <= unsigned(shift_right(r2_6, to_integer(s_dsh))(7 downto 0));
                end if;
            else
                disp7 <= (others => '0');
            end if;
            if cls6 = '1' then
                v_sh := shift_right(sh6, to_integer(s_sss));
            else
                v_sh := shift_right(sh6, to_integer(s_ss));
            end if;
            shade7 <= f_clamp8(v_sh);
            -- rim results enter their delay chains
            frd(7)   <= r6_fr;
            rshd(7)  <= r6_shade;
            zoned(7) <= r6_zone;
            anyd(7)  <= r6_any;
            insd(7)  <= ins6;

            -- ---- stage 8 ----
            dxk8 <= shift_left(resize(dxk7a, 21), 4) + resize(dxk7b, 21);
            dyk8 <= shift_left(resize(dyk7a, 21), 4) + resize(dyk7b, 21);
            disp8 <= disp7 * s_kd;
            shade8 <= shade7;
            y8s <= signed(resize(s_ypx, 13));

            -- ---- stage 9 ----
            x9 <= signed(resize(xd(8), 13)) - resize(dxk8(20 downto 8), 13);
            y9 <= y8s - resize(dyk8(20 downto 8), 13);
            disp9 <= disp8(13 downto 8);
            shd(9) <= shade8;

            -- ---- stage 10: stripes swap ----
            if s_stripes = '1' then
                a10 <= y9; b10 <= x9;
            else
                a10 <= x9; b10 <= y9;
            end if;
            dispd(10) <= disp9;

            -- ---- stage 11: wave frequency ----
            bf11a <= b10 * signed('0' & s_fw(7 downto 4));
            bf11b <= b10 * signed('0' & s_fw(3 downto 0));
            ad(11) <= a10;

            -- ---- stage 12 ----
            bf12 <= shift_left(resize(bf11a, 21), 4) + resize(bf11b, 21);

            -- ---- stage 13: sine address ----
            wa13 <= unsigned(bf12(15 downto 8)) + s_ph16(15 downto 8);

            -- ---- stage 14: sine ROM in p_rom; stage 15 re-register ----
            sin15 <= signed(sin14);

            -- ---- stage 16: amplitude (split) ----
            sa16a <= sin15 * signed('0' & s_A(9 downto 5));
            sa16b <= sin15 * signed('0' & s_A(4 downto 0));

            -- ---- stage 17 ----
            sa17 <= shift_left(resize(sa16a, 20), 5) + resize(sa16b, 20);

            -- ---- stage 18: displaced coordinate + wave ----
            p18 <= resize(ad(17), 14) + resize(sa17(19 downto 8), 14);

            -- ---- stage 19: band scale (split) ----
            pb19a <= p18 * signed('0' & s_bs(8 downto 5));
            pb19b <= p18 * signed('0' & s_bs(4 downto 0));

            -- ---- stage 20 ----
            pb20 <= shift_left(resize(pb19a, 24), 5) + resize(pb19b, 24);

            -- ---- stage 21: palette index (rim fringe overrides) ----
            if anyf(20) = '1' and zonef(20) = '1' then
                idx21 <= frd(20);
            else
                idx21 <= unsigned(pb20(15 downto 8)) + resize(dispd(20), 8) + s_sc16(15 downto 8);
            end if;

            -- ---- stage 22: palette ROMs in p_rom; stage 23 re-register ----
            pal23_y <= pal22_y;
            pal23_u <= pal22_u;
            pal23_v <= pal22_v;
            -- rim luma: black, lit to white toward the top-left
            v_rim := resize(rshd(22), 12) - to_signed(30, 12);
            v_rim := shift_left(v_rim, 3);
            if v_rim < 0 then
                rimy23 <= C_BLK;
            elsif v_rim > 680 then
                rimy23 <= to_unsigned(744, 10);
            else
                rimy23 <= unsigned(resize(v_rim, 10)) + C_BLK;
            end if;

            -- ---- stage 24: compose ----
            if anyd(23) = '1' then
                if zoned(23) = '1' then
                    c24_y <= pal23_y; c24_u <= pal23_u; c24_v <= pal23_v;
                else
                    c24_y <= rimy23; c24_u <= C_MID; c24_v <= C_MID;
                end if;
            elsif insd(23) = '1' then
                v_y := signed(resize(pal23_y, 12)) + resize(shd(23), 12);
                c24_y <= f_clamp_y(v_y);
                if s_chrome = '1' then
                    c24_u <= C_MID; c24_v <= C_MID;
                else
                    c24_u <= pal23_u; c24_v <= pal23_v;
                end if;
            else
                if s_solid = '1' then
                    if s_dark = '1' then
                        c24_y <= to_unsigned(186, 10); c24_u <= to_unsigned(423, 10); c24_v <= to_unsigned(532, 10);
                    else
                        c24_y <= to_unsigned(651, 10); c24_u <= to_unsigned(318, 10); c24_v <= to_unsigned(543, 10);
                    end if;
                else
                    if s_dark = '1' then
                        c24_y <= resize(pal23_y(9 downto 2), 10) + to_unsigned(48, 10);
                    else
                        c24_y <= resize(pal23_y(9 downto 1), 10) + to_unsigned(368, 10);
                    end if;
                    if s_chrome = '1' then
                        c24_u <= C_MID; c24_v <= C_MID;
                    else
                        c24_u <= resize(pal23_u(9 downto 1), 10) + to_unsigned(256, 10);
                        c24_v <= resize(pal23_v(9 downto 1), 10) + to_unsigned(256, 10);
                    end if;
                end if;
            end if;

            -- ---- delay chains ----
            for i in 12 to 17 loop ad(i) <= ad(i-1); end loop;
            for i in 11 to 20 loop dispd(i) <= dispd(i-1); end loop;
            for i in 8 to 20 loop frd(i) <= frd(i-1); end loop;
            for i in 8 to 22 loop rshd(i) <= rshd(i-1); end loop;
            for i in 10 to 23 loop shd(i) <= shd(i-1); end loop;
            for i in 8 to 23 loop
                anyd(i) <= anyd(i-1); zoned(i) <= zoned(i-1); insd(i) <= insd(i-1);
            end loop;
            zonef(7) <= r6_zone; anyf(7) <= r6_any;
            for i in 8 to 20 loop zonef(i) <= zonef(i-1); anyf(i) <= anyf(i-1); end loop;
        end if;
    end process p_pix;

    -- ROMs (registered reads -> EBR)
    p_rom_sin : process(clk) begin if rising_edge(clk) then sin14 <= C_SIN(to_integer(wa13)); end if; end process;
    p_rom_py  : process(clk) begin if rising_edge(clk) then pal22_y <= C_PAL_Y(to_integer(idx21)); end if; end process;
    p_rom_pu  : process(clk) begin if rising_edge(clk) then pal22_u <= C_PAL_U(to_integer(idx21)); end if; end process;
    p_rom_pv  : process(clk) begin if rising_edge(clk) then pal22_v <= C_PAL_V(to_integer(idx21)); end if; end process;

    -- ====================================================================
    -- Output conditioning ([1 2 1]/4 on Y/U/V), blanking gate, output
    -- ====================================================================
    p_out : process(clk)
    begin
        if rising_edge(clk) then
            -- stage 25
            c24d_y <= c24_y; c24d_u <= c24_u; c24d_v <= c24_v;
            s25_y <= resize(c24_y, 11) + resize(c24d_y, 11);
            s25_u <= resize(c24_u, 11) + resize(c24d_u, 11);
            s25_v <= resize(c24_v, 11) + resize(c24d_v, 11);
            -- stage 26
            s25d_y <= s25_y; s25d_u <= s25_u; s25d_v <= s25_v;
            s26_y <= resize(shift_right(resize(s25_y, 12) + resize(s25d_y, 12), 2), 10);
            s26_u <= resize(shift_right(resize(s25_u, 12) + resize(s25d_u, 12), 2), 10);
            s26_v <= resize(shift_right(resize(s25_v, 12) + resize(s25d_v, 12), 2), 10);
            -- stage 27: blanking gate
            if d_gv(26) = '1' then
                g27_y <= s26_y; g27_u <= s26_u; g27_v <= s26_v;
            else
                g27_y <= C_BLK; g27_u <= C_MID; g27_v <= C_MID;
            end if;
            -- stage 28: output register
            s_io.y <= std_logic_vector(g27_y);
            s_io.u <= std_logic_vector(g27_u);
            s_io.v <= std_logic_vector(g27_v);
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
