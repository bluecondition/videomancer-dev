-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Precess v0.1.0 -- Hardline.  Sixteen nested ellipses about one centre, all the same shape, each turned a
-- step from the one outside it.  The eye reads a spiral that is not drawn; as the twist grows the ellipses
-- cross and the bands break into a lace of crescents.  Black and white only (U = V = 512 on every pixel).
-- After the op-art "twisted ellipses", Duchamp's Rotoreliefs and the rosette of a precessing orbit.
--
--   ellipse k (k = 0 inner .. 15 outer):  e^2 u^2 + v^2 < e^2 A_k^2,  (u, v) = the pixel turned by (k - 15) tau
--   F_k = a x^2 + b x y + c y^2 - T_k + camera,  a, c = C0 +- C1 cos 2 theta,  b = 2 C1 sin 2 theta
--   Bands: the parity of the number of ellipses containing the pixel (triangle of the summed coverage)
--   Stack: painter's order, smallest on top, alternately black and white
--
--   K1  X        the centre, +-1152 virtual px            K2  Y   the centre, +-648 virtual px
--   K3  Period   ellipse spacing P = 30 .. 240 virtual px, exponential
--   K4  Oval     the aspect e: 0.9 (0) .. 0.6 (512) .. 0.3 (1023)
--   K5  Drift    bipolar speed of the twist (Hypnos decode), +-24 counts still, 2^-16 .. 2^-8 cycle per field,
--                one cycle = 180 degrees per ellipse (the picture repeats exactly)
--   K6  Camera   luma added to every F (bands bend round the subject), (K6/1023)^2
--   S7  Glide    S8 Spacing (Even / Log)   S9 Fill (Bands / Stack)   S10 Weight (Bold / Fine)   S11 Source
--   P12 TWIST    tau = +-40 degrees per ellipse at the ends, +-8-count detent at centre (a plain oval target)
--
-- Each ellipse is an exact quadratic run by second differences in pixel units (13 fractional bits):
--   per pixel  F += D;  D += DD + V        (V: the camera's second difference, shared by all sixteen)
--   per line   F0 += E;  E += EE;  D0 += G (the line unit, in the register file)
-- A microcoded engine (precess.py: 48-bit accumulator, 512 x 48 register file, serial multiplier) computes every
-- ellipse's seeds ONCE PER FIELD IN THE BACKGROUND for the next field (two banks), stalled while the line unit
-- owns the register file.  Coverage: a per-ellipse power-of-two window (a four-stage log shifter) of F.
--
-- Sections (the integer model .hardline_work/rb_precess/imodel.py mirrors each one):
--   [A] TIMING AND VIRTUAL COORDINATES   timing ID, vblank zone, width, generated avid, sync delay
--   [B] SWITCHES AND BANKS               per-field latches, bank swap
--   [C] FIELD ENGINE                     the microcoded machine (knob holds / glides, drift, seeds)
--   [L] LINE UNIT                        beam load (3 clocks per ellipse) and line DDA step (5 per ellipse)
--   [E] ELLIPSES                         sixteen beam DDAs, log shifters, coverage; Stack masks; sum; tone
--   [D] SOURCE FIELD                     luma or Osc -> gain table -> 64-px box -> vertical one-pole -> C, V
--   [F] OUTPUT CONDITIONING              luma [1 2 1]/4, 13/16 toward black, blanking gate
--
-- Stage numbering: T = the g_start clock.  The beam DDAs hold pixel n during B0 + n, B0 = T + 25 (pixel -2 at
-- T + 23: each line's DDA starts two pixels early so the camera can enter through V).  Output pixel n leaves at
-- T + 41 + n = T + C_LAT + 1 + n.  One latency (40) in every mode.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.precess_pkg.all;
use work.all;

architecture precess of program_top is
    constant C_LAT : integer := 40;   -- = processing_delay_clks
    constant NE    : integer := 16;

    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;

    attribute no_rw_check : boolean;

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo, s_zgo_d : std_logic := '0';
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal c_nq1  : unsigned(8 downto 0) := to_unsigned(479, 9);    -- the camera store's last column, W/4 - 1
    -- per-field mode constants (the engine's IN ports)
    signal m_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal m_ys   : unsigned(10 downto 0) := to_unsigned(256, 11);
    signal m_ihy  : unsigned(16 downto 0) := to_unsigned(65535, 17);
    signal m_ihx  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal m_x0   : unsigned(13 downto 0) := to_unsigned(15352, 14);
    signal m_lhm  : unsigned(13 downto 0) := (others => '0');
    signal m_hw4  : unsigned(15 downto 0) := to_unsigned(36864, 16);
    signal m_sh   : unsigned(2 downto 0) := to_unsigned(6, 3);

    -- [A] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_hsync_sr : t_bit_shift := (others => '1');
    signal s_vsync_sr : t_bit_shift := (others => '1');
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k): '1' during T + 2 + k .. T + W + 1 + k
    signal s_field_q  : std_logic := '1';
    signal s_st       : std_logic_vector(0 to 23) := (others => '0');   -- s_st(k) = '1' at T + k + 1

    -- [B] switches: e_* = the engine's (latched at the zone start), d_* = the beam's (taken with the bank swap)
    signal e_glide : std_logic := '1';
    signal e_log, e_stack, e_fine, e_src : std_logic := '0';
    signal d_stack, d_fine, d_src : std_logic := '0';
    signal b_cur  : std_logic := '0';                               -- the bank the beam and the line unit use

    -- [C] the engine
    constant OP_LD   : std_logic_vector(3 downto 0) := x"0";
    constant OP_ADD  : std_logic_vector(3 downto 0) := x"1";
    constant OP_SUB  : std_logic_vector(3 downto 0) := x"2";
    constant OP_MUL  : std_logic_vector(3 downto 0) := x"3";
    constant OP_LDI  : std_logic_vector(3 downto 0) := x"4";
    constant OP_ADDI : std_logic_vector(3 downto 0) := x"5";
    constant OP_ANDI : std_logic_vector(3 downto 0) := x"6";
    constant OP_MULI : std_logic_vector(3 downto 0) := x"7";
    constant OP_ST   : std_logic_vector(3 downto 0) := x"8";
    constant OP_IN   : std_logic_vector(3 downto 0) := x"9";
    constant OP_OUT  : std_logic_vector(3 downto 0) := x"A";
    constant OP_SHR  : std_logic_vector(3 downto 0) := x"B";
    constant OP_SKIP : std_logic_vector(3 downto 0) := x"C";
    constant OP_JMP  : std_logic_vector(3 downto 0) := x"D";
    constant OP_CALL : std_logic_vector(3 downto 0) := x"E";
    constant OP_SPC  : std_logic_vector(3 downto 0) := x"F";
    signal rf_ram : t_c_pc_rfi := C_PC_RFI;
    attribute no_rw_check of rf_ram : signal is true;
    signal rf_q  : std_logic_vector(47 downto 0) := (others => '0');
    signal rf_ra, rf_wa : unsigned(8 downto 0) := (others => '0');
    signal rf_wd : std_logic_vector(47 downto 0) := (others => '0');
    signal rf_we : std_logic := '0';
    signal u_run, u_done : std_logic := '0';
    signal u_ph  : unsigned(1 downto 0) := "11";
    signal u_pc, u_lr : unsigned(9 downto 0) := (others => '0');
    signal u_ir  : std_logic_vector(15 downto 0) := x"F001";
    signal u_irf : std_logic_vector(15 downto 0) := x"F001";
    signal u_mul, u_shr, u_halt, u_xo, u_xst : std_logic := '0';
    signal u_gate, u_subr, u_shift, u_cz, u_and : std_logic := '0';
    signal u_acc, u_b : signed(47 downto 0) := (others => '0');
    signal u_hx, u_cl : std_logic := '0';                           -- execute half; the low half's carry
    signal u_sl : signed(23 downto 0) := (others => '0');           -- the low half's sum
    signal u_cnt : unsigned(4 downto 0) := (others => '0');
    signal u_x   : unsigned(3 downto 0) := (others => '0');
    signal u_stall : std_logic;
    signal w_in  : signed(17 downto 0);
    signal ft_adr : unsigned(9 downto 0) := (others => '0');
    signal ft_sin, ft_exp : std_logic_vector(15 downto 0) := (others => '0');
    signal u_gstp : unsigned(15 downto 0) := (others => '0');      -- the gain step the run produced
    signal c_gst  : unsigned(15 downto 0) := (others => '0');      -- the gain step in use
    signal e_eng_ra : unsigned(8 downto 0);

    -- [L] line unit
    signal l_go, l_on, l_ld : std_logic := '0';
    signal l_c   : unsigned(7 downto 0) := (others => '0');
    signal l_k   : unsigned(3 downto 0) := (others => '0');        -- read side: ellipse, slot step
    signal l_s   : unsigned(2 downto 0) := (others => '0');
    signal l_rv  : std_logic := '0';                                -- read side valid
    type t_rd is array (1 to 4) of unsigned(3 downto 0);
    type t_rs is array (1 to 4) of unsigned(2 downto 0);
    signal l_kd  : t_rd := (others => (others => '0'));             -- read side delayed (1 .. 4)
    signal l_sd  : t_rs := (others => (others => '0'));
    signal l_vd  : std_logic_vector(1 to 4) := (others => '0');
    signal l_ldd : std_logic_vector(1 to 4) := (others => '0');     -- load (vs step) delayed
    signal rq, rqd : signed(47 downto 0) := (others => '0');
    signal l_dsub : signed(36 downto 0) := (others => '0');
    signal l_wsum : signed(47 downto 0) := (others => '0');
    signal l_we  : std_logic := '0';
    signal l_wa  : unsigned(8 downto 0) := (others => '0');
    signal l_ra  : unsigned(8 downto 0) := (others => '0');
    signal b_bus : unsigned(24 downto 0) := (others => '0');        -- DD (16..0), t (20..17), H (24..21)
    signal b_v   : signed(19 downto 0) := (others => '0');          -- D's upper operand (bits 36 .. 17)
    signal b_run : std_logic := '0';
    signal e_rst, e_cef, e_ced, e_ceth : std_logic_vector(0 to NE - 1) := (others => '0');

    -- [E] ellipses
    type t_s37 is array (0 to NE - 1) of signed(36 downto 0);
    type t_u17 is array (0 to NE - 1) of unsigned(16 downto 0);
    type t_u4  is array (0 to NE - 1) of unsigned(3 downto 0);
    type t_s12 is array (0 to NE - 1) of signed(11 downto 0);
    type t_s8  is array (0 to NE - 1) of signed(7 downto 0);
    type t_s6  is array (0 to NE - 1) of signed(5 downto 0);
    type t_s5  is array (0 to NE - 1) of signed(4 downto 0);
    type t_u5  is array (0 to NE - 1) of unsigned(4 downto 0);
    signal e_F, e_D : t_s37 := (others => (others => '0'));
    signal e_dd  : t_u17 := (others => (others => '0'));
    signal e_t, e_h : t_u4 := (others => (others => '0'));
    signal a1 : t_s12 := (others => (others => '0'));
    signal a2 : t_s8 := (others => (others => '0'));
    signal a3 : t_s6 := (others => (others => '0'));
    signal a4 : t_s5 := (others => (others => '0'));
    signal ok1, ok2, ok3, ok4 : std_logic_vector(0 to NE - 1) := (others => '0');
    signal sg1, sg2, sg3, sg4 : std_logic_vector(0 to NE - 1) := (others => '0');
    signal cv5 : t_u5 := (others => (others => '0'));
    -- Stack: inside flags and their prefix ORs
    signal in1, in2, in3 : std_logic_vector(0 to NE - 1) := (others => '0');
    signal pg2 : std_logic_vector(0 to NE - 1) := (others => '0');  -- in-group prefix (exclusive)
    signal gr2 : std_logic_vector(0 to 3) := (others => '0');       -- group ORs
    signal pi3 : std_logic_vector(0 to NE - 1) := (others => '0');  -- prefix over all (exclusive)
    signal any3 : std_logic := '0';
    signal pi4, in4 : std_logic_vector(0 to NE - 1) := (others => '0');
    signal col4, any4 : std_logic := '0';
    signal off_sr : std_logic_vector(5 to 9) := (others => '0');    -- 16 to add (Bold Stack): stages 5 .. 9
    type t_s6x8 is array (0 to 7) of unsigned(5 downto 0);
    type t_s7x4 is array (0 to 3) of unsigned(6 downto 0);
    type t_s8x2 is array (0 to 1) of unsigned(7 downto 0);
    signal sm6 : t_s6x8 := (others => (others => '0'));
    signal sm7 : t_s7x4 := (others => (others => '0'));
    signal sm8 : t_s8x2 := (others => (others => '0'));
    signal sm9 : unsigned(8 downto 0) := (others => '0');
    signal o10 : unsigned(4 downto 0) := (others => '0');
    signal y11a : unsigned(14 downto 0) := (others => '0');
    signal y11b : unsigned(8 downto 0) := (others => '0');

    -- [F] output conditioning
    signal y12, y13, y14, y14f, y15, y16 : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- [D] source field
    signal s_ox  : unsigned(15 downto 0) := (others => '0');
    signal s_oy  : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_osum : unsigned(7 downto 0) := (others => '0');
    signal s_sl  : unsigned(7 downto 0) := (others => '0');
    signal s_sv, s_sv2 : std_logic := '0';
    type t_gt is array (0 to 255) of unsigned(15 downto 0);
    signal gt_ram : t_gt := (others => (others => '0'));
    attribute no_rw_check of gt_ram : signal is true;
    signal gt_q, s_gq, gt_wd : unsigned(15 downto 0) := (others => '0');
    signal gt_wa : unsigned(7 downto 0) := (others => '0');
    signal gt_we, gt_run : std_logic := '0';
    signal gt_acc : unsigned(23 downto 0) := (others => '0');
    signal gt_i  : unsigned(7 downto 0) := (others => '0');
    signal s_gv, s_gvd : std_logic := '0';
    type t_bx is array (0 to 255) of unsigned(15 downto 0);
    signal bx_ram : t_bx := (others => (others => '0'));
    attribute no_rw_check of bx_ram : signal is true;
    signal bx_wp, bx_rp : unsigned(7 downto 0) := (others => '0');
    signal bx_q, bx_qr : unsigned(15 downto 0) := (others => '0');
    signal s_bcnt : unsigned(6 downto 0) := (others => '0');
    signal s_bsum : unsigned(21 downto 0) := (others => '0');
    signal s_first : unsigned(15 downto 0) := (others => '0');
    signal s_col : unsigned(10 downto 0) := (others => '0');
    signal s_bv  : std_logic := '0';
    signal s_bcol : unsigned(10 downto 0) := (others => '0');
    signal s_fl1, s_flr, s_flr2 : std_logic := '1';                  -- the field's first source line
    signal s_iv, s_iv2, s_iv3 : std_logic := '0';
    signal s_ic, s_ic2, s_ic3 : unsigned(8 downto 0) := (others => '0');
    signal s_eb, s_eb2, s_eb3, s_q, s_q2, s_q3 : unsigned(15 downto 0) := (others => '0');
    signal s_id, s_ii : signed(17 downto 0) := (others => '0');
    signal s_flr3 : std_logic := '1';
    type t_lq is array (0 to 511) of unsigned(15 downto 0);
    signal lq_ram : t_lq := (others => (others => '0'));
    attribute no_rw_check of lq_ram : signal is true;
    signal lq_q1, lq_q2, lq_wd : unsigned(15 downto 0) := (others => '0');
    signal lq_wa : unsigned(8 downto 0) := (others => '0');
    signal lq_we : std_logic := '0';
    -- pixel path
    signal p_cnt : signed(12 downto 0) := (others => '0');          -- the pixel this read is for
    signal p_on  : std_logic := '0';
    signal p_ra  : unsigned(8 downto 0) := (others => '0');
    signal p_f1, p_f2, p_f3, p_f4 : unsigned(1 downto 0) := "00";
    signal p_v1, p_v2, p_v3, p_v4 : std_logic := '0';
    signal p_q0, p_q1 : unsigned(15 downto 0) := (others => '0');
    signal p_d3  : signed(16 downto 0) := (others => '0');
    signal p_b3  : unsigned(15 downto 0) := (others => '0');
    signal c_st, c_st1 : signed(19 downto 0) := (others => '0');    -- the camera stream C(p), C(p - 1)
    signal c_u   : signed(19 downto 0) := (others => '0');          -- C(p - 1) - 2 C(p)  (one clock ahead)

begin

    ---------------------------------------------------------------------------
    -- ROMs and RAMs (one write statement each; registered reads)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            ft_sin <= C_PC_SIN(to_integer(ft_adr));
            ft_exp <= C_PC_EXP(to_integer(ft_adr(8 downto 0)));
        end if;
    end process p_rom;

    -- the register file: the engine (stalled while the line unit runs) or the line unit
    rf_ra <= l_ra when u_stall = '1' else e_eng_ra;
    p_rf : process(clk)
    begin
        if rising_edge(clk) then
            rf_q <= rf_ram(to_integer(rf_ra));
            if rf_we = '1' then rf_ram(to_integer(rf_wa)) <= rf_wd; end if;
        end if;
    end process p_rf;

    p_gtram : process(clk)
    begin
        if rising_edge(clk) then
            gt_q <= gt_ram(to_integer(s_sl));
            if gt_we = '1' then gt_ram(to_integer(gt_wa)) <= gt_wd; end if;
        end if;
    end process p_gtram;

    p_bxram : process(clk)
    begin
        if rising_edge(clk) then
            bx_q <= bx_ram(to_integer(bx_rp));
            if s_gv = '1' then bx_ram(to_integer(bx_wp)) <= s_gq; end if;
        end if;
    end process p_bxram;

    -- the camera store (1/4 horizontal resolution): port 1 = the one-pole's own read, port 2 = the pixel path
    p_lqram : process(clk)
    begin
        if rising_edge(clk) then
            lq_q1 <= lq_ram(to_integer(s_ic));
            lq_q2 <= lq_ram(to_integer(p_ra));
            if lq_we = '1' then lq_ram(to_integer(lq_wa)) <= lq_wd; end if;
        end if;
    end process p_lqram;

    ---------------------------------------------------------------------------
    -- [A] TIMING + [B] SWITCHES AND BANKS
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_t : integer range 0 to 15;
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            v_t := to_integer(unsigned(s_tid));

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;
            s_zgo_d <= s_zgo;

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- per-field latches.  The bank swap: if the engine finished its run, the beam takes that run's
            -- bank, gain step and switches; then the engine starts again with this field's switches.
            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                if u_done = '1' then
                    b_cur <= not b_cur;
                    c_gst <= u_gstp;
                    d_stack <= e_stack; d_fine <= e_fine; d_src <= e_src;
                end if;
                e_glide <= registers_in(6)(0);
                e_log   <= registers_in(6)(1);
                e_stack <= registers_in(6)(2);
                e_fine  <= registers_in(6)(3);
                e_src   <= registers_in(6)(4);
                m_xs  <= C_PC_MXS(v_t);  m_ys  <= C_PC_MYS(v_t);  m_ihy <= C_PC_MIHY(v_t);
                m_ihx <= C_PC_MIHX(v_t); m_x0  <= C_PC_MX0(v_t);  m_lhm <= C_PC_MLHM(v_t);
                m_hw4 <= C_PC_MHW4(v_t); m_sh  <= C_PC_MSH(v_t);
                case to_integer(C_PC_MXS(v_t)) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                s_oys <= C_PC_MOYS(v_t);
            end if;
            g_wm1 <= s_w - 1;
            c_nq1 <= resize(shift_right(s_w, 2), 9) - 1;

            -- sync delay, generated-avid and start-pulse shift registers (field_n is tied high by the core on
            -- every input, so it only gets a register)
            s_hsync_sr(0) <= data_in.hsync_n; s_vsync_sr(0) <= data_in.vsync_n;
            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_hsync_sr(i) <= s_hsync_sr(i - 1); s_vsync_sr(i) <= s_vsync_sr(i - 1);
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            s_field_q <= data_in.field_n;
            s_st <= g_start & s_st(0 to 22);
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- [A] Generated avid (parity-locked start 3/4 clocks after the source's
    -- avid rise, field-latched width)
    ---------------------------------------------------------------------------
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

            if g_start = '1' then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- [C] FIELD ENGINE (after cassini / flute).  Started at every zone start from
    -- address 0; four clocks per micro-op (register-file read, operand /
    -- decode / next address, execute in two halves: the 48-bit adder is split
    -- at bit 24 with a registered carry), serial multiply (20 steps of two
    -- clocks) and shift.  It has a whole field to run in.
    -- MUL = ACC(19:0) x operand(27:0), both signed, by add-and-shift-right with
    -- the last step subtracting.  Stalled (frozen) while the line unit owns the
    -- register file; a stall caught between the register-file read and its use
    -- replays the read.
    ---------------------------------------------------------------------------
    u_stall <= l_on;
    e_eng_ra <= ('1' & (not b_cur) & u_x & unsigned(u_ir(2 downto 0))) when u_ir(8) = '1' else
                ('0' & unsigned(u_ir(7 downto 0)));

    with u_irf(3 downto 0) select w_in <=
        signed(resize(unsigned(registers_in(0)), 18)) when x"0",
        signed(resize(unsigned(registers_in(1)), 18)) when x"1",
        signed(resize(unsigned(registers_in(2)), 18)) when x"2",
        signed(resize(unsigned(registers_in(3)), 18)) when x"3",
        signed(resize(unsigned(registers_in(4)), 18)) when x"4",
        signed(resize(unsigned(registers_in(5)), 18)) when x"5",
        resize(signed(ft_sin), 18)                     when x"6",
        signed(resize(unsigned(registers_in(7)), 18)) when x"7",
        signed(resize(m_xs, 18))                       when x"8",
        signed(resize(m_ys, 18))                       when x"9",
        signed(resize(m_ihy, 18))                      when x"A",
        signed(resize(m_x0, 18))                       when x"B",
        signed(resize(m_lhm, 18))                      when x"C",
        signed(resize(m_hw4, 18))                      when x"D",
        signed(resize(unsigned(ft_exp), 18))           when x"E",
        signed(resize(m_ihx, 18))                      when others;

    p_eng : process(clk)
        variable v_op  : std_logic_vector(3 downto 0);
        variable v_arg : std_logic_vector(11 downto 0);
        variable v_gate, v_sub, v_shift, v_last, v_cond, v_fetch : std_logic;
        variable v_bx  : signed(47 downto 0);
        variable v_tl  : signed(25 downto 0);
        variable v_th  : signed(25 downto 0);
        variable v_sum : signed(48 downto 0);
    begin
        if rising_edge(clk) then
            v_op := u_irf(15 downto 12);
            v_arg := u_irf(11 downto 0);
            v_fetch := '0';

            if s_zgo = '1' then
                u_run <= '1'; u_done <= '0'; u_pc <= (others => '0'); u_ph <= "11"; u_hx <= '0';
            elsif u_stall = '1' then
                if u_ph = "01" then u_ph <= "00"; end if;
            elsif u_run = '1' then
                case u_ph is
                    when "11" =>                          -- fetch (first micro-op)
                        v_fetch := '1';
                        u_ph <= "00";
                    when "00" =>                          -- register-file read
                        u_irf <= u_ir;
                        u_ph <= "01";
                    when "01" =>                          -- operand, accumulator clear, next address
                        case v_op is
                            when OP_LD | OP_ADD | OP_SUB => u_b <= signed(rf_q);
                            when OP_LDI | OP_ADDI => u_b <= resize(signed(v_arg), 48);
                            when OP_ANDI => u_b <= signed(resize(unsigned(v_arg), 48));
                            when OP_IN   => u_b <= resize(w_in, 48);
                            when OP_MUL  => u_b <= signed(rf_q(27 downto 0)) & to_signed(0, 20);
                            when OP_MULI => u_b <= resize(signed(v_arg), 28) & to_signed(0, 20);
                            when others  => null;
                        end case;
                        if v_op = OP_LD or v_op = OP_LDI or v_op = OP_IN then
                            u_acc <= (others => '0');
                        elsif v_op = OP_MUL or v_op = OP_MULI then
                            u_acc(47 downto 20) <= (others => '0');
                        end if;
                        if v_op = OP_MUL or v_op = OP_MULI then u_cnt <= to_unsigned(19, 5);
                        elsif v_op = OP_SHR then u_cnt <= unsigned(v_arg(4 downto 0)) - 1; end if;
                        case v_arg(2 downto 0) is
                            when "000"  => v_cond := u_acc(47);
                            when "001"  => v_cond := e_glide;
                            when "010"  => v_cond := e_log;
                            when "011"  => v_cond := e_fine;
                            when others => v_cond := e_src;
                        end case;
                        v_cond := v_cond xor v_arg(3);
                        if v_op = OP_SKIP and v_cond = '1' then u_pc <= u_pc + 2;
                        elsif v_op = OP_JMP then u_pc <= unsigned(v_arg(9 downto 0));
                        elsif v_op = OP_CALL then u_pc <= unsigned(v_arg(9 downto 0)); u_lr <= u_pc + 1;
                        elsif v_op = OP_SPC and v_arg(0) = '0' then u_pc <= u_lr;
                        else u_pc <= u_pc + 1; end if;
                        u_mul <= '0'; u_shr <= '0'; u_halt <= '0'; u_xo <= '0'; u_xst <= '0';
                        u_gate <= '0'; u_subr <= '0'; u_shift <= '0'; u_cz <= '0'; u_and <= '0';
                        case v_op is
                            when OP_LD | OP_ADD | OP_LDI | OP_ADDI | OP_IN => u_gate <= '1';
                            when OP_SUB  => u_gate <= '1'; u_subr <= '1';
                            when OP_ANDI => u_and <= '1';
                            when OP_MUL | OP_MULI => u_mul <= '1'; u_shift <= '1'; u_gate <= u_acc(0);
                            when OP_SHR  =>
                                u_shr <= '1'; u_shift <= '1';
                                if v_arg(4 downto 0) = "00001" then u_cz <= '1'; end if;
                            when OP_ST   => u_xst <= '1';
                            when OP_OUT  => u_xo <= '1';
                            when OP_SPC  => u_halt <= v_arg(0);
                            when others  => null;
                        end case;
                        u_ph <= "10";
                    when others =>                        -- execute: the adder in two 24-bit halves, two clocks
                        v_gate := u_gate; v_sub := u_subr; v_shift := u_shift; v_last := '1';
                        if v_gate = '0' then v_bx := (others => '0');
                        elsif v_sub = '1' then v_bx := not u_b;
                        else v_bx := u_b; end if;
                        if u_hx = '0' then
                            -- low half: bits 23 .. 0 and the carry out of them
                            v_tl := ('0' & u_acc(23 downto 0) & '1') + ('0' & v_bx(23 downto 0) & (v_sub and v_gate));
                            u_sl <= v_tl(24 downto 1); u_cl <= v_tl(25);
                            u_hx <= '1';
                        else
                            -- high half (sign-extended: 25 bits = bits 48 .. 24 of the sum)
                            v_th := (u_acc(47) & u_acc(47 downto 24) & '1') + (v_bx(47) & v_bx(47 downto 24) & u_cl);
                            v_sum := v_th(25 downto 1) & u_sl;
                            u_hx <= '0';
                            if u_mul = '1' or u_shr = '1' then
                                if u_cz = '0' then v_last := '0'; end if;
                                u_cnt <= u_cnt - 1;
                                if u_cnt = 1 then u_cz <= '1'; u_subr <= u_mul; end if;   -- the next step is the last
                                if u_mul = '1' then u_gate <= v_sum(1); end if;           -- the next multiplier bit
                            end if;
                            if u_halt = '1' then u_run <= '0'; u_done <= '1'; end if;
                            if u_and = '1' then u_acc <= u_acc and u_b;
                            elsif v_shift = '1' then u_acc <= v_sum(48 downto 1);
                            else u_acc <= v_sum(47 downto 0); end if;
                            if u_xo = '1' then
                                case u_irf(3 downto 0) is
                                    when x"0"   => ft_adr <= unsigned(u_acc(9 downto 0));
                                    when x"1"   => u_x <= unsigned(u_acc(3 downto 0));
                                    when x"4"   => u_gstp <= unsigned(u_acc(15 downto 0));
                                    when others => null;
                                end case;
                            end if;
                            if v_last = '1' then
                                v_fetch := '1';
                                u_ph <= "00";
                            end if;
                        end if;
                end case;
            end if;
            if v_fetch = '1' then
                u_ir <= C_PC_UC(to_integer(u_pc));
            end if;
        end if;
    end process p_eng;

    ---------------------------------------------------------------------------
    -- [L] LINE UNIT.  Started after the bank swap (line 0) and after every
    -- line's last pixel.  LOAD (read clocks 0 .. 47): per ellipse F0, D0, DDW;
    -- the beam takes them in three clocks P0 .. P2 (3k + 3 .. 3k + 5):
    --   P0: F, D <- 0;  DD <- F0(16:0)
    --   P1: D <- 0 + {V = F0(36:17), DD};  DD <- (D0 - F0)(16:0)
    --   P2: F <- F + D = F0;  D <- F0 + {V = (D0 - F0)(36:17), DD} = D0;  DD, t, H <- DDW
    -- STEP (read clocks 48 .. 127, five per ellipse: F0 E EE D0 G): F0 += E,
    -- E += EE, D0 += G written back four clocks after the first read.
    ---------------------------------------------------------------------------
    p_lu : process(clk)
        variable v_slot : unsigned(2 downto 0);
        variable v_d    : signed(37 downto 0);
    begin
        if rising_edge(clk) then
            -- start: after the swap (zone) or after the line's last pixel
            l_go <= s_zgo_d or (s_gav_sr(26) and not s_gav_sr(25));
            if l_go = '1' then
                l_on <= '1'; l_c <= (others => '0'); l_k <= (others => '0'); l_s <= (others => '0');
                l_ld <= '1'; l_rv <= '1';
            elsif l_on = '1' then
                l_c <= l_c + 1;
                if l_ld = '1' then
                    if l_s = 2 then
                        l_s <= (others => '0'); l_k <= l_k + 1;
                        if l_k = 15 then l_ld <= '0'; end if;
                    else
                        l_s <= l_s + 1;
                    end if;
                elsif l_rv = '1' then
                    if l_s = 4 then
                        l_s <= (others => '0'); l_k <= l_k + 1;
                        if l_k = 15 then l_rv <= '0'; end if;
                    else
                        l_s <= l_s + 1;
                    end if;
                end if;
                if l_c = 140 then l_on <= '0'; end if;
            end if;
            -- the read address for this clock's read side (registered: it is used next clock)
            if l_ld = '1' then
                case to_integer(l_s) is
                    when 0      => v_slot := "000";      -- F0
                    when 1      => v_slot := "010";      -- D0
                    when others => v_slot := "101";      -- DDW
                end case;
            else
                case to_integer(l_s) is
                    when 0      => v_slot := "000";      -- F0
                    when 1      => v_slot := "001";      -- E
                    when 2      => v_slot := "011";      -- EE
                    when 3      => v_slot := "010";      -- D0
                    when others => v_slot := "100";      -- G
                end case;
            end if;
            l_ra <= '1' & b_cur & l_k & v_slot;
            -- read side delayed: 1 = the clock the RAM register holds the data, 2 = rq, 3 = rqd ...
            l_kd(1) <= l_k; l_sd(1) <= l_s; l_vd(1) <= l_on and (l_ld or l_rv); l_ldd(1) <= l_ld;
            for i in 2 to 4 loop
                l_kd(i) <= l_kd(i - 1); l_sd(i) <= l_sd(i - 1); l_vd(i) <= l_vd(i - 1); l_ldd(i) <= l_ldd(i - 1);
            end loop;
            rq <= signed(rf_q); rqd <= rq;

            -- LOAD: bus and V (registered: valid in the next clock, P0 .. P2)
            v_d := resize(rq(36 downto 0), 38) - resize(rqd(36 downto 0), 38);
            b_run <= b_run;
            if s_st(21) = '1' then b_run <= '1'; elsif l_go = '1' then b_run <= '0'; end if;
            if l_vd(3) = '1' and l_ldd(3) = '1' and l_sd(3) = 0 then
                b_bus <= resize(unsigned(rq(16 downto 0)), 25);                   -- F0 low   (P0 next)
            elsif l_vd(3) = '1' and l_ldd(3) = '1' and l_sd(3) = 1 then
                b_bus <= resize(unsigned(v_d(16 downto 0)), 25);                  -- (D0 - F0) low (P1 next)
                l_dsub <= v_d(36 downto 0);
            elsif l_vd(3) = '1' and l_ldd(3) = '1' and l_sd(3) = 2 then
                b_bus <= unsigned(rq(24 downto 0));                               -- DDW      (P2 next)
            end if;
            -- e_* controls for the next clock (the load side = the read side three clocks later)
            for k in 0 to NE - 1 loop
                e_rst(k) <= '0'; e_cef(k) <= '0'; e_ced(k) <= '0'; e_ceth(k) <= '0';
                if l_vd(3) = '1' and l_ldd(3) = '1' and l_kd(3) = k then
                    case to_integer(l_sd(3)) is
                        when 0      => e_rst(k) <= '1'; e_cef(k) <= '1'; e_ced(k) <= '1';
                        when 1      => e_ced(k) <= '1';
                        when others => e_cef(k) <= '1'; e_ced(k) <= '1'; e_ceth(k) <= '1';
                    end case;
                end if;
            end loop;

            -- STEP: write back four clocks after the read side issued F0 / E / D0
            l_wsum <= rq + rqd;
            l_we <= '0';
            if l_vd(4) = '1' and l_ldd(4) = '0' then
                case to_integer(l_sd(4)) is
                    when 0      => l_we <= '1'; l_wa <= '1' & b_cur & l_kd(4) & unsigned'("000");   -- F0 += E
                    when 1      => l_we <= '1'; l_wa <= '1' & b_cur & l_kd(4) & unsigned'("001");   -- E  += EE
                    when 3      => l_we <= '1'; l_wa <= '1' & b_cur & l_kd(4) & unsigned'("010");   -- D0 += G
                    when others => null;
                end case;
            end if;
        end if;
    end process p_lu;

    -- register-file write port: the line unit, or the engine's ST (never both: the engine is stalled)
    rf_we <= l_we when u_stall = '1' else
             '1' when u_run = '1' and u_ph = "10" and u_hx = '1' and u_xst = '1' else '0';
    rf_wa <= l_wa when u_stall = '1' else
             ('1' & (not b_cur) & u_x & unsigned(u_irf(2 downto 0))) when u_irf(8) = '1' else
             ('0' & unsigned(u_irf(7 downto 0)));
    rf_wd <= std_logic_vector(l_wsum) when u_stall = '1' else std_logic_vector(u_acc);

    ---------------------------------------------------------------------------
    -- [E] ELLIPSES (stages 0 .. 5).  Stage 0: the beam DDAs.  1 .. 4: the log
    -- shifter v = (F >> 10) >> t, keeping five bits and whether v fits them.
    -- 5: coverage (0 .. 16), masked for Stack.
    --   Bold: 8 - v for v in -8 .. 7 (v in 1/16 px), else 16 inside / 0 outside
    --   Fine: clamp(4 (H - |v|), 0, 16) (v in 1/4 px, a line of half-width H - 2)
    ---------------------------------------------------------------------------
    g_ell : for k in 0 to NE - 1 generate
        p_e : process(clk)
            variable g  : signed(26 downto 0);
            variable ok : std_logic;
            variable av : unsigned(3 downto 0);
            variable lc : signed(7 downto 0);
            variable cv : unsigned(4 downto 0);
            variable mk : std_logic;
            variable v_op : signed(36 downto 0);
        begin
            if rising_edge(clk) then
                v_op(36 downto 17) := b_v;
                v_op(16 downto 0) := signed(e_dd(k));
                if e_ced(k) = '1' then e_dd(k) <= b_bus(16 downto 0); end if;      -- DD loads in P0 .. P2
                if e_ceth(k) = '1' then e_t(k) <= b_bus(20 downto 17); e_h(k) <= b_bus(24 downto 21); end if;
                if b_run = '1' or e_cef(k) = '1' then
                    if e_rst(k) = '1' then e_F(k) <= (others => '0'); else e_F(k) <= e_F(k) + e_D(k); end if;
                end if;
                if b_run = '1' or e_ced(k) = '1' then
                    if e_rst(k) = '1' then e_D(k) <= (others => '0');
                    else e_D(k) <= e_D(k) + v_op; end if;
                end if;

                -- 1: shift by 8 t(3)
                g := e_F(k)(36 downto 10);
                sg1(k) <= g(26);
                in1(k) <= g(26);
                if e_t(k)(3) = '1' then
                    a1(k) <= g(19 downto 8);
                    if g(26 downto 19) = 0 or g(26 downto 19) = -1 then ok1(k) <= '1'; else ok1(k) <= '0'; end if;
                else
                    a1(k) <= g(11 downto 0);
                    if g(26 downto 11) = 0 or g(26 downto 11) = -1 then ok1(k) <= '1'; else ok1(k) <= '0'; end if;
                end if;
                -- 2: by 4 t(2)
                sg2(k) <= sg1(k);
                if e_t(k)(2) = '1' then
                    a2(k) <= a1(k)(11 downto 4); ok2(k) <= ok1(k);
                else
                    a2(k) <= a1(k)(7 downto 0);
                    if a1(k)(11 downto 7) = 0 or a1(k)(11 downto 7) = -1 then ok2(k) <= ok1(k); else ok2(k) <= '0'; end if;
                end if;
                -- 3: by 2 t(1)
                sg3(k) <= sg2(k);
                if e_t(k)(1) = '1' then
                    a3(k) <= a2(k)(7 downto 2); ok3(k) <= ok2(k);
                else
                    a3(k) <= a2(k)(5 downto 0);
                    if a2(k)(7 downto 5) = 0 or a2(k)(7 downto 5) = -1 then ok3(k) <= ok2(k); else ok3(k) <= '0'; end if;
                end if;
                -- 4: by t(0)
                sg4(k) <= sg3(k);
                if e_t(k)(0) = '1' then
                    a4(k) <= a3(k)(5 downto 1); ok4(k) <= ok3(k);
                else
                    a4(k) <= a3(k)(4 downto 0);
                    if a3(k)(5) = a3(k)(4) then ok4(k) <= ok3(k); else ok4(k) <= '0'; end if;
                end if;
                -- 5: coverage
                if d_fine = '0' then
                    if ok4(k) = '1' and a4(k)(4) = a4(k)(3) then
                        cv := unsigned(to_signed(8, 5) - a4(k));
                    elsif sg4(k) = '1' then cv := to_unsigned(16, 5);
                    else cv := (others => '0'); end if;
                else
                    if a4(k)(4) = '1' then av := not unsigned(a4(k)(3 downto 0));
                    else av := unsigned(a4(k)(3 downto 0)); end if;
                    lc := signed(resize(e_h(k), 8)) - signed(resize(av, 8));
                    if ok4(k) = '0' or lc(7) = '1' then cv := (others => '0');
                    elsif lc > 3 then cv := to_unsigned(16, 5);
                    else cv := unsigned(lc(2 downto 0)) & "00"; end if;
                end if;
                -- Stack mask: no plate above k contains the pixel, and (Bold) k is the top plate or its colour
                -- differs from the top plate's
                mk := '1';
                if d_stack = '1' then
                    if d_fine = '1' then
                        mk := not pi4(k);
                    elsif (k mod 2) = 1 then
                        mk := (not pi4(k)) and (in4(k) or not col4);
                    else
                        mk := (not pi4(k)) and (in4(k) or col4);
                    end if;
                end if;
                if mk = '1' then cv5(k) <= cv; else cv5(k) <= (others => '0'); end if;
            end if;
        end process p_e;
    end generate g_ell;

    ---------------------------------------------------------------------------
    -- [E] Stack prefix (stages 2 .. 4), sum (6 .. 9), tone (10), luma (11)
    ---------------------------------------------------------------------------
    p_sum : process(clk)
        variable v_g  : std_logic_vector(0 to 3);
        variable v_p  : std_logic;
        variable v_c  : std_logic;
        variable v_s  : unsigned(8 downto 0);
        variable v_m  : unsigned(4 downto 0);
    begin
        if rising_edge(clk) then
            -- 2: in-group exclusive prefixes and group ORs (groups of four)
            for gi in 0 to 3 loop
                v_p := '0';
                for j in 0 to 3 loop
                    pg2(4 * gi + j) <= v_p;
                    v_p := v_p or in1(4 * gi + j);
                end loop;
                gr2(gi) <= v_p;
            end loop;
            in2 <= in1;
            -- 3: over all
            for gi in 0 to 3 loop
                v_p := '0';
                for h in 0 to 3 loop
                    if h < gi then v_p := v_p or gr2(h); end if;
                end loop;
                for j in 0 to 3 loop
                    pi3(4 * gi + j) <= pg2(4 * gi + j) or v_p;
                end loop;
            end loop;
            any3 <= gr2(0) or gr2(1) or gr2(2) or gr2(3);
            in3 <= in2;
            -- 4: the top plate's colour (odd = white)
            v_c := '0';
            for k in 0 to NE - 1 loop
                if (k mod 2) = 1 then v_c := v_c or (in3(k) and not pi3(k)); end if;
            end loop;
            col4 <= v_c; any4 <= any3; pi4 <= pi3; in4 <= in3;
            -- 5 .. 9: Bold Stack adds 16 when a black plate is on top
            off_sr(5) <= d_stack and (not d_fine) and any4 and not col4;
            for i in 6 to 9 loop off_sr(i) <= off_sr(i - 1); end loop;
            -- 6 .. 9: the sum of the sixteen coverages
            for i in 0 to 7 loop sm6(i) <= resize(cv5(2 * i), 6) + resize(cv5(2 * i + 1), 6); end loop;
            for i in 0 to 3 loop sm7(i) <= resize(sm6(2 * i), 7) + resize(sm6(2 * i + 1), 7); end loop;
            for i in 0 to 1 loop sm8(i) <= resize(sm7(2 * i), 8) + resize(sm7(2 * i + 1), 8); end loop;
            sm9 <= resize(sm8(0), 9) + resize(sm8(1), 9);
            -- 10: Fine min(S, 16); Bold the triangle of S mod 32 (+16 for a black top plate in Stack)
            v_s := sm9;
            if off_sr(9) = '1' then v_s(4) := not v_s(4); end if;
            if d_fine = '1' then
                if sm9 > 16 then o10 <= to_unsigned(16, 5); else o10 <= sm9(4 downto 0); end if;
            else
                v_m := v_s(4 downto 0);
                if v_s(4) = '1' and v_s(3 downto 0) /= 0 then o10 <= to_unsigned(0, 5) - v_m;
                else o10 <= v_m; end if;
            end if;
            -- 11: 876 o = 1024 o - 128 o - 16 o - 4 o
            y11a <= shift_left(resize(o10, 15), 10) - shift_left(resize(o10, 15), 7);
            y11b <= shift_left(resize(o10, 9), 4) + shift_left(resize(o10, 9), 2);
        end if;
    end process p_sum;

    ---------------------------------------------------------------------------
    -- [F] OUTPUT CONDITIONING (stages 12 .. 16): y = 64 + 876 o / 16, black
    -- outside pixels 0 .. W - 1; [1 2 1] / 4; 13/16 toward black (white 777);
    -- blanking gate.  U = V = 512 always.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_y  : unsigned(14 downto 0);
        variable v_s  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 12 (pixel n during B0 + n + 11 = T + 36 + n is in the window when s_gav_sr(34) = '1')
            v_y := y11a - resize(y11b, 15);
            if s_gav_sr(34) = '1' then y12 <= to_unsigned(64, 10) + v_y(13 downto 4);
            else y12 <= to_unsigned(64, 10); end if;
            -- 13, 14: [1 2 1] / 4
            y13 <= y12; y14 <= y13;
            v_s := resize(y12, 12) + resize(y13 & '0', 12) + resize(y14, 12) + 2;
            y14f <= v_s(11 downto 2);
            -- 15: 13/16 toward black
            if y14f > 64 then v_dl := y14f - 64; else v_dl := (others => '0'); end if;
            y15 <= y14f - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            -- 16: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then y16 <= y15; else y16 <= to_unsigned(64, 10); end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(y16);
    data_out.u       <= std_logic_vector(to_unsigned(512, 10));
    data_out.v       <= std_logic_vector(to_unsigned(512, 10));
    data_out.hsync_n <= s_hsync_sr(C_LAT - 1);
    data_out.vsync_n <= s_vsync_sr(C_LAT - 1);
    data_out.field_n <= s_field_q;
    data_out.avid    <= s_gav_sr(C_LAT - 1);

    ---------------------------------------------------------------------------
    -- [D] SOURCE FIELD: luma (or the Osc lattice) -> gain table (refilled at the
    -- bank swap: T(l) = (l gst) >> 7) -> 64-px causal box (the line's first
    -- sample replicated, reloaded at every line start) -> every fourth column a
    -- vertical one-pole q += (box/16 - q + r) >> sh in the store (the field's
    -- first line loads it) -> the pixel path: C(p) = (q0 + ((q1 - q0) f >> 2))
    -- >> 2 between columns p/4 and p/4 + 1 (units of 16 Q), zero before pixel 0;
    -- V(m) = C(m) - 2 C(m - 1) + C(m - 2) drives the beam's D adders.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_l  : unsigned(7 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_tap : unsigned(15 downto 0);
        variable v_m  : signed(18 downto 0);
        variable v_c  : signed(17 downto 0);
        variable v_pc : signed(12 downto 0);
        variable v_ca : unsigned(9 downto 0);
        variable v_q  : signed(17 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator: 2-D triangle lattice, 640 x 540 virtual px, drifting (both phases restart in the zone)
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' and data_in.avid = '0' then
                s_ox <= shift_left(resize(s_ot, 16), 7);
                s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5);
            end if;
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_s := resize(f_tri7(s_ox(15 downto 8)), 8) + resize(f_tri7(s_oy(15 downto 8)), 8);
            s_osum <= v_s - shift_right(v_s, 3);

            -- the source sample: (luma >> 2) - 16, or the lattice
            v_l := unsigned(data_in.y(9 downto 2));
            if d_src = '1' then s_sl <= s_osum;
            elsif v_l > 16 then s_sl <= v_l - 16;
            else s_sl <= (others => '0'); end if;
            s_sv <= data_in.avid;

            -- gain table: gt_q holds T(s_sl) one clock after s_sl; s_gq a fabric copy
            s_sv2 <= s_sv;
            s_gq <= gt_q;
            s_gv <= s_sv2; s_gvd <= s_gv;                                  -- s_gv marks s_gq
            gt_we <= '0';
            if s_zgo_d = '1' then
                gt_run <= '1'; gt_i <= (others => '0'); gt_acc <= (others => '0');
            elsif gt_run = '1' then
                gt_we <= '1'; gt_wa <= gt_i; gt_wd <= resize(shift_right(gt_acc, 7), 16);
                gt_acc <= gt_acc + resize(c_gst, 24); gt_i <= gt_i + 1;
                if gt_i = 255 then gt_run <= '0'; end if;
            end if;

            -- box: sum of the last 64 samples; before the line's 64th sample the first one stands in
            -- (the ring's read runs two clocks ahead: bx_q, then a fabric copy bx_qr = the sample 64 back)
            if s_gv = '1' then bx_wp <= bx_wp + 1; bx_rp <= bx_rp + 1;
            else bx_wp <= (others => '0'); bx_rp <= to_unsigned(194, 8); end if;
            bx_qr <= bx_q;
            if s_bcnt < 64 then v_tap := s_first; else v_tap := bx_qr; end if;
            if s_gv = '1' and s_gvd = '0' then
                s_first <= s_gq;
                s_bsum <= shift_left(resize(s_gq, 22), 6);
                s_bcnt <= to_unsigned(1, 7);
            elsif s_gv = '1' then
                s_bsum <= s_bsum + s_gq - v_tap;
                if s_bcnt /= 64 then s_bcnt <= s_bcnt + 1; end if;
            end if;
            if s_gv = '1' then s_col <= s_col + 1; else s_col <= (others => '0'); end if;
            s_bv <= s_gv; s_bcol <= s_col;
            -- the field's first source line: set in the zone, cleared at the end of the first line
            if s_zone = '1' then s_fl1 <= '1';
            elsif s_bv = '1' and s_gv = '0' then s_fl1 <= '0'; end if;

            -- one-pole on columns 3, 7, 11, ... (store column = col / 4).  The store is read for the column from
            -- its first sample on (lq_q1, then the fabric copy s_q), so its old value is ready when the box is:
            -- i5 eb, q -> i6 d = eb - q -> i7 (d + r) >> sh -> i8 q + that -> written in i9.
            s_ic <= s_bcol(10 downto 2);
            s_q <= lq_q1;
            if s_bv = '1' and s_bcol(1 downto 0) = "11" then s_iv <= '1'; else s_iv <= '0'; end if;
            s_eb <= s_bsum(19 downto 4);
            s_flr <= s_fl1;
            -- i6
            s_iv2 <= s_iv; s_ic2 <= s_ic; s_flr2 <= s_flr; s_eb2 <= s_eb; s_q2 <= s_q;
            s_id <= signed(resize(s_eb, 18)) - signed(resize(s_q, 18));
            -- i7
            s_iv3 <= s_iv2; s_ic3 <= s_ic2; s_flr3 <= s_flr2; s_eb3 <= s_eb2; s_q3 <= s_q2;
            case to_integer(m_sh) is
                when 4      => s_ii <= shift_right(s_id + 8, 4);
                when 5      => s_ii <= shift_right(s_id + 16, 5);
                when others => s_ii <= shift_right(s_id + 32, 6);
            end case;
            -- i8
            v_q := signed(resize(s_q3, 18)) + s_ii;
            lq_we <= s_iv3;
            lq_wa <= s_ic3;
            if s_flr3 = '1' then lq_wd <= s_eb3;
            else lq_wd <= unsigned(v_q(15 downto 0)); end if;

            -- pixel path.  p_cnt = the pixel this column read is for (x0): -4 at T + 13; the store's second
            -- port reads column p/4 + 1 (clamped to the last column).  C(p) lands in c_st (x5) at T + 22 + p.
            if s_st(11) = '1' then p_cnt <= to_signed(-4, 13); p_on <= '1';
            elsif p_on = '1' then
                p_cnt <= p_cnt + 1;
                if p_cnt = 2047 then p_on <= '0'; end if;
            end if;
            v_pc := shift_right(p_cnt, 2) + 1;
            if v_pc(12) = '1' then v_ca := (others => '0');
            elsif v_pc(11 downto 9) /= 0 or unsigned(v_pc(8 downto 0)) > c_nq1 then v_ca := resize(c_nq1, 10);
            else v_ca := unsigned(v_pc(9 downto 0)); end if;
            -- x1: the read address
            p_ra <= v_ca(8 downto 0);
            p_f1 <= unsigned(p_cnt(1 downto 0));
            if p_on = '1' and p_cnt(12) = '0' then p_v1 <= '1'; else p_v1 <= '0'; end if;   -- p >= 0
            -- x2: lq_q2 holds column p/4 + 1
            p_f2 <= p_f1; p_v2 <= p_v1;
            -- x3: q0, q1 = columns p/4, p/4 + 1
            if p_f2 = "00" then p_q0 <= p_q1; p_q1 <= lq_q2; end if;
            p_f3 <= p_f2; p_v3 <= p_v2;
            -- x4: difference and base
            p_d3 <= signed(resize(p_q1, 17)) - signed(resize(p_q0, 17));
            p_b3 <= p_q0; p_f4 <= p_f3; p_v4 <= p_v3;
            -- x5: C(p) = (q0 + ((q1 - q0) f >> 2)) >> 2, zero before pixel 0
            v_m := (others => '0');
            if p_f4(1) = '1' then v_m := shift_left(resize(p_d3, 19), 1); end if;
            if p_f4(0) = '1' then v_m := v_m + resize(p_d3, 19); end if;
            v_c := shift_right(signed(resize(p_b3, 18)) + resize(shift_right(v_m, 2), 18), 2);
            if p_v4 = '1' then c_st <= resize(v_c, 20); else c_st <= (others => '0'); end if;
            c_st1 <= c_st;
            c_u <= c_st1 - shift_left(c_st, 1);
            -- V (the beam's D operand): the line unit's values in the load, else the camera's second difference
            if l_vd(3) = '1' and l_ldd(3) = '1' and l_sd(3) = 1 then
                b_v <= rqd(36 downto 17);                                          -- F0 high (P1 next)
            elsif l_vd(3) = '1' and l_ldd(3) = '1' and l_sd(3) = 2 then
                b_v <= l_dsub(36 downto 17);                                       -- (D0 - F0) high (P2 next)
            else
                b_v <= c_st + c_u;
            end if;
        end if;
    end process p_src;

end architecture precess;
