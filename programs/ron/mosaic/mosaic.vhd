-- mosaic.vhd  (v0.1 -- "Mosaic": multi-bit colour frame-store feedback test-bed)
--
-- Sibling of afterimage: instead of 256x128 one-bit cells this keeps a
-- 64x32 grid of 16-bit COLOUR cells {Yout(4), Ysample(4), U(4), V(4)} in
-- two ping-pong banks (8 EBR each), so the feedback law can see last
-- frame's OUTPUT and last frame's SAMPLE separately.  That second field
-- is what a temporal derivative needs: |sample - old sample| is real
-- motion, not a self-feeding recurrence.
--
-- Feedback laws (K4):
--   Trails : out = max(sample, old_out - decay)        (decay stochastic, P12)
--   Blur   : out = old_out + (sample - old_out) >> k   (stochastic rounding)
--   Motion : out = max(2|sample - old_sample|, old_out - decay)
--   Hold   : cell updates only when |sample - old_sample| > threshold
-- Where the old cell is read from (K1) is the same family as afterimage:
-- hold / zoom in / zoom out / left / right / up / down / spin, plus K6
-- random-walk diffusion and S9 mirror.
--
-- Display: nearest-neighbour blocks.  K5 = Video (stored luma + stored
-- chroma) or a tint palette (paper->ink lerp by cell luma); S8 swaps the
-- chroma for live video chroma scaled by the cell; S10 keys live video
-- luma through the cell (video seen through the trails); S7 inverts.
--
-- 16 EBR, 6 small (11x5) multipliers.  Chroma stored as a signed 4-bit
-- offset so all-zero RAM is black + neutral.  Palette U/V-swapped.
--
-- Author: bluecondition

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture mosaic of program_top is

    constant LATENCY : natural := 7;

    constant C_MID  : unsigned(9 downto 0) := to_unsigned(512, 10);
    constant C_BLKY : unsigned(9 downto 0) := to_unsigned(64, 10);

    -- Tint palettes, index 1..7 (0 = Video).  Stored U = Cr, V = Cb.
    -- 1 Mono  2 Paper  3 Game Boy  4 Phosphor  5 Amber  6 Blueprint  7 Neon
    type t_pal is array (0 to 7) of unsigned(9 downto 0);
    constant INK_Y : t_pal := (to_unsigned(768,10), to_unsigned(768,10), to_unsigned( 64,10),
        to_unsigned(198,10), to_unsigned(651,10), to_unsigned(681,10),
        to_unsigned(768,10), to_unsigned(533,10));
    constant INK_U : t_pal := (to_unsigned(512,10), to_unsigned(512,10), to_unsigned(512,10),
        to_unsigned(452,10), to_unsigned(212,10), to_unsigned(701,10),
        to_unsigned(512,10), to_unsigned(809,10));
    constant INK_V : t_pal := (to_unsigned(512,10), to_unsigned(512,10), to_unsigned(512,10),
        to_unsigned(464,10), to_unsigned(275,10), to_unsigned(156,10),
        to_unsigned(512,10), to_unsigned(677,10));
    constant PAP_Y : t_pal := (to_unsigned( 64,10), to_unsigned( 64,10), to_unsigned(768,10),
        to_unsigned(608,10), to_unsigned( 64,10), to_unsigned( 64,10),
        to_unsigned(268,10), to_unsigned(118,10));
    constant PAP_U : t_pal := (to_unsigned(512,10), to_unsigned(512,10), to_unsigned(512,10),
        to_unsigned(503,10), to_unsigned(512,10), to_unsigned(512,10),
        to_unsigned(413,10), to_unsigned(498,10));
    constant PAP_V : t_pal := (to_unsigned(512,10), to_unsigned(512,10), to_unsigned(512,10),
        to_unsigned(228,10), to_unsigned(512,10), to_unsigned(512,10),
        to_unsigned(711,10), to_unsigned(600,10));

    function rotl16(x : unsigned(15 downto 0); k : natural) return unsigned is
        variable v : unsigned(15 downto 0);
    begin
        v := x(15 - k downto 0) & x(15 downto 16 - k);
        return v;
    end function;

    function hround(x : unsigned(15 downto 0); ra : natural; sh : natural) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := x xor rotl16(x, ra);
        return t + shift_left(t, sh);
    end function;

    function map3(h : unsigned(2 downto 0)) return signed is
    begin
        case to_integer(h) is
            when 0 => return to_signed(-3, 4);
            when 1 => return to_signed(-2, 4);
            when 2 => return to_signed(-1, 4);
            when 5 => return to_signed( 1, 4);
            when 6 => return to_signed( 2, 4);
            when 7 => return to_signed( 3, 4);
            when others => return to_signed(0, 4);
        end case;
    end function;

    -- clamp a signed value to 0..15
    function clamp4u(x : signed) return unsigned is
    begin
        if x < 0 then return to_unsigned(0, 4);
        elsif x > 15 then return to_unsigned(15, 4);
        else return unsigned(x(3 downto 0)); end if;
    end function;

    -- clamp a signed value to -8..7
    function clamp4s(x : signed) return signed is
    begin
        if x < -8 then return to_signed(-8, 4);
        elsif x > 7 then return to_signed(7, 4);
        else return x(3 downto 0); end if;
    end function;

    --------------------------------------------------------------------------
    -- Control decode.
    --------------------------------------------------------------------------
    signal rk1, rk2, rk3, rk4, rk5, rk6, rp12 : unsigned(9 downto 0) := (others => '0');
    signal rsw : std_logic_vector(4 downto 0) := (others => '0');
    signal s_mode : unsigned(2 downto 0) := (others => '0');
    signal s_gsel : unsigned(1 downto 0) := (others => '0');
    signal s_exp  : signed(11 downto 0) := (others => '0');
    signal s_law  : unsigned(1 downto 0) := (others => '0');
    signal s_psel : unsigned(2 downto 0) := (others => '0');
    signal s_jp8  : unsigned(7 downto 0) := (others => '0');
    signal s_p9   : unsigned(8 downto 0) := to_unsigned(64, 9);
    signal s_thr  : unsigned(3 downto 0) := (others => '0');
    signal s_bk   : unsigned(1 downto 0) := "01";     -- blur shift 1..3
    signal s_dec2 : std_logic := '0';                  -- decay 2/frame at low persist
    signal sw_inv, sw_lch, sw_mir, sw_vid, sw_frz : std_logic := '0';
    signal m_zoom, m_spin : std_logic := '0';

    --------------------------------------------------------------------------
    -- Raster / geometry (k=0).
    --------------------------------------------------------------------------
    type t_pipe is array (natural range <>) of t_video_stream_yuv444_30b;
    signal pipe : t_pipe(0 to LATENCY - 1);

    signal prev_hsync_n, prev_vsync_n, prev_avid : std_logic := '1';
    signal lfsr : unsigned(15 downto 0) := x"BEEF";
    signal px   : unsigned(11 downto 0) := (others => '0');
    signal py   : unsigned(10 downto 0) := (others => '0');
    signal line_act, frame_act, frame_armed : std_logic := '0';
    signal w_meas : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal h_meas : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal g_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal g_h    : unsigned(11 downto 0) := to_unsigned(1080, 12);
    signal g_cols : unsigned(6 downto 0) := to_unsigned(64, 7);
    signal g_rows : unsigned(5 downto 0) := to_unsigned(32, 6);
    signal g_cx   : unsigned(5 downto 0) := to_unsigned(32, 6);
    signal g_ry   : unsigned(4 downto 0) := to_unsigned(16, 5);
    signal g_cm1  : signed(9 downto 0) := to_signed(63, 10);
    signal par    : std_logic := '0';
    signal seed1 : unsigned(15 downto 0) := x"1234";
    signal seed2 : unsigned(15 downto 0) := x"9ABC";
    signal seed3 : unsigned(15 downto 0) := x"5678";

    signal xacc : unsigned(11 downto 0) := (others => '0');
    signal c    : unsigned(6 downto 0) := (others => '0');
    signal cs   : std_logic := '0';
    signal yacc : unsigned(11 downto 0) := (others => '0');
    signal r    : unsigned(5 downto 0) := (others => '0');
    signal mid_done, sample_line : std_logic := '0';
    signal af   : std_logic := '0';
    signal lpy0, lpy1, lpy2 : unsigned(9 downto 0) := C_BLKY;
    signal lpu0, lpu1, lpu2 : unsigned(9 downto 0) := C_MID;
    signal lpv0, lpv1, lpv2 : unsigned(9 downto 0) := C_MID;
    signal wp0  : std_logic := '0';

    --------------------------------------------------------------------------
    -- F-chain (k=1..4).
    --------------------------------------------------------------------------
    signal f1_dc, f1_dr : signed(9 downto 0) := (others => '0');
    signal f1_t8 : unsigned(5 downto 0) := (others => '0');
    signal f1_h1, f1_h2, f1_h3 : unsigned(15 downto 0) := (others => '0');
    signal f1_c  : unsigned(5 downto 0) := (others => '0');
    signal f1_r  : unsigned(4 downto 0) := (others => '0');

    signal f2_sx, f2_sy : signed(9 downto 0) := (others => '0');
    signal f2_t8 : unsigned(5 downto 0) := (others => '0');
    signal f2_h1, f2_h2, f2_h3 : unsigned(15 downto 0) := (others => '0');
    signal f2_c  : unsigned(5 downto 0) := (others => '0');
    signal f2_r  : unsigned(4 downto 0) := (others => '0');

    signal f3_qx, f3_qy : signed(3 downto 0) := (others => '0');
    signal f3_fx, f3_fy : unsigned(5 downto 0) := (others => '0');
    signal f3_t8 : unsigned(5 downto 0) := (others => '0');
    signal f3_h1, f3_h2, f3_h3 : unsigned(15 downto 0) := (others => '0');
    signal f3_c  : unsigned(5 downto 0) := (others => '0');
    signal f3_r  : unsigned(4 downto 0) := (others => '0');

    signal f4_qx, f4_qy : signed(3 downto 0) := (others => '0');
    signal f4_fx, f4_fy : unsigned(5 downto 0) := (others => '0');
    signal f4_t8 : unsigned(5 downto 0) := (others => '0');
    signal f4_h1, f4_h2, f4_h3 : unsigned(15 downto 0) := (others => '0');
    signal f4_c  : unsigned(5 downto 0) := (others => '0');
    signal f4_r  : unsigned(4 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- P-chain.
    --------------------------------------------------------------------------
    signal p0_v : std_logic := '0';
    signal p0_qx, p0_qy : signed(3 downto 0) := (others => '0');
    signal p0_fx, p0_fy : unsigned(5 downto 0) := (others => '0');
    signal p0_t8 : unsigned(5 downto 0) := (others => '0');
    signal p0_h1, p0_h2, p0_h3 : unsigned(15 downto 0) := (others => '0');
    signal p0_c  : unsigned(5 downto 0) := (others => '0');
    signal p0_r  : unsigned(4 downto 0) := (others => '0');
    signal p0_ly, p0_lu, p0_lv : unsigned(9 downto 0) := (others => '0');

    signal p1_v : std_logic := '0';
    signal p1_vq : signed(12 downto 0) := (others => '0');
    signal p1_du, p1_dv : signed(10 downto 0) := (others => '0');
    signal p1_surv, p1_jon, p1_half : std_logic := '0';
    signal p1_jx, p1_jy : signed(3 downto 0) := (others => '0');
    signal p1_sx, p1_sy : signed(4 downto 0) := (others => '0');
    signal p1_hb : unsigned(5 downto 0) := (others => '0');
    signal p1_c  : unsigned(5 downto 0) := (others => '0');
    signal p1_r  : unsigned(4 downto 0) := (others => '0');

    signal p2_v, p2_surv : std_logic := '0';
    signal p2_ys : unsigned(3 downto 0) := (others => '0');
    signal p2_us, p2_vs : signed(3 downto 0) := (others => '0');
    signal p2_ox, p2_oy : signed(5 downto 0) := (others => '0');
    signal p2_hb : unsigned(5 downto 0) := (others => '0');
    signal p2_c  : unsigned(5 downto 0) := (others => '0');
    signal p2_r  : unsigned(4 downto 0) := (others => '0');

    signal p3_v, p3_surv : std_logic := '0';
    signal p3_ys : unsigned(3 downto 0) := (others => '0');
    signal p3_us, p3_vs : signed(3 downto 0) := (others => '0');
    signal p3_cx : signed(9 downto 0) := (others => '0');
    signal p3_ry : signed(8 downto 0) := (others => '0');
    signal p3_hb : unsigned(5 downto 0) := (others => '0');
    signal p3_c  : unsigned(5 downto 0) := (others => '0');
    signal p3_r  : unsigned(4 downto 0) := (others => '0');

    signal p4_v, p4_surv, p4_rv : std_logic := '0';
    signal p4_ys : unsigned(3 downto 0) := (others => '0');
    signal p4_us, p4_vs : signed(3 downto 0) := (others => '0');
    signal p4_cx : signed(9 downto 0) := (others => '0');
    signal p4_ry : signed(8 downto 0) := (others => '0');
    signal p4_hb : unsigned(5 downto 0) := (others => '0');
    signal p4_c  : unsigned(5 downto 0) := (others => '0');
    signal p4_r  : unsigned(4 downto 0) := (others => '0');

    signal p5_v, p5_surv, p5_fbv : std_logic := '0';
    signal p5_ys : unsigned(3 downto 0) := (others => '0');
    signal p5_us, p5_vs : signed(3 downto 0) := (others => '0');
    signal p5_hb : unsigned(5 downto 0) := (others => '0');
    signal p5_addr, p5_wa : unsigned(10 downto 0) := (others => '0');

    -- request register + issued context
    signal req_v, req_surv, req_fbv : std_logic := '0';
    signal req_ys : unsigned(3 downto 0) := (others => '0');
    signal req_us, req_vs : signed(3 downto 0) := (others => '0');
    signal req_hb : unsigned(5 downto 0) := (others => '0');
    signal req_addr, req_wa : unsigned(10 downto 0) := (others => '0');

    type t_ctx is record
        surv, fbv : std_logic;
        ys : unsigned(3 downto 0);
        us, vs : signed(3 downto 0);
        hb : unsigned(5 downto 0);
        wa : unsigned(10 downto 0);
    end record;
    constant C_CTX0 : t_ctx := ('0', '0', (others => '0'), (others => '0'), (others => '0'), (others => '0'), (others => '0'));
    signal c1, c2, c3 : t_ctx := C_CTX0;

    --------------------------------------------------------------------------
    -- Frame store: two 2048 x 16 banks, ping-pong by frame parity.
    --------------------------------------------------------------------------
    type t_bank is array (0 to 2047) of std_logic_vector(15 downto 0);
    signal bank_a, bank_b : t_bank := (others => (others => '0'));
    signal raddr : unsigned(10 downto 0) := (others => '0');
    signal rtag1, rtag2, rtag3 : std_logic := '0';
    signal rd_a, rd_b, rda_f, rdb_f : std_logic_vector(15 downto 0) := (others => '0');

    -- k=4 law operands
    signal o4_v : std_logic := '0';
    signal o4_ydec : unsigned(3 downto 0) := (others => '0');
    signal o4_ys, o4_ysold, o4_yo : unsigned(3 downto 0) := (others => '0');
    signal o4_us, o4_vs, o4_uo, o4_vo : signed(3 downto 0) := (others => '0');
    signal o4_dy, o4_ds, o4_du, o4_dv : signed(4 downto 0) := (others => '0');
    signal o4_hb : unsigned(5 downto 0) := (others => '0');
    signal o4_wa : unsigned(10 downto 0) := (others => '0');
    -- k=5 write word
    signal w5 : std_logic_vector(15 downto 0) := (others => '0');
    signal wv5 : std_logic := '0';
    signal waddr5 : unsigned(10 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Display.
    --------------------------------------------------------------------------
    signal disp4 : std_logic_vector(15 downto 0) := (others => '0');
    signal pal3_iy, pal3_iu, pal3_iv, pal3_py, pal3_pu, pal3_pv : unsigned(9 downto 0) := C_MID;
    signal pal4_py, pal4_pu, pal4_pv : unsigned(9 downto 0) := C_MID;
    signal pal4_dy, pal4_du, pal4_dv : signed(10 downto 0) := (others => '0');
    signal pal5_py, pal5_pu, pal5_pv : unsigned(9 downto 0) := C_MID;
    signal m5_ty, m5_tu, m5_tv : signed(15 downto 0) := (others => '0');
    signal m5_vy, m5_vu, m5_vv : signed(15 downto 0) := (others => '0');
    signal y5_46 : unsigned(9 downto 0) := (others => '0');
    signal u5_st, v5_st : unsigned(9 downto 0) := C_MID;
    signal y6, u6, v6 : unsigned(9 downto 0) := C_BLKY;
    signal blank6 : std_logic := '1';
    signal s_io : t_video_stream_yuv444_30b;

begin

    --------------------------------------------------------------------------
    -- Free-running control decode.
    --------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_p : unsigned(8 downto 0);
    begin
        if rising_edge(clk) then
            rk1  <= unsigned(registers_in(0)(9 downto 0));
            rk2  <= unsigned(registers_in(1)(9 downto 0));
            rk3  <= unsigned(registers_in(2)(9 downto 0));
            rk4  <= unsigned(registers_in(3)(9 downto 0));
            rk5  <= unsigned(registers_in(4)(9 downto 0));
            rk6  <= unsigned(registers_in(5)(9 downto 0));
            rsw  <= registers_in(6)(4 downto 0);
            rp12 <= unsigned(registers_in(7)(9 downto 0));

            s_mode <= rk1(9 downto 7);
            s_gsel <= rk2(9 downto 8);
            s_exp  <= signed(resize(rk3, 12)) - to_signed(512, 12);
            s_law  <= rk4(9 downto 8);
            s_psel <= rk5(9 downto 7);
            s_jp8  <= rk6(9 downto 2);
            v_p := to_unsigned(64, 9) + resize(rp12(9 downto 2), 9) - resize(rp12(9 downto 4), 9);
            s_p9  <= v_p;                               -- 64..256
            s_thr <= resize(shift_right(v_p - to_unsigned(64, 9), 4), 4);   -- 0..12
            if rp12 < 341 then s_bk <= "01"; elsif rp12 < 682 then s_bk <= "10"; else s_bk <= "11"; end if;
            if rp12 < 256 then s_dec2 <= '1'; else s_dec2 <= '0'; end if;

            sw_inv <= rsw(0);   -- S7  Invert
            sw_lch <= rsw(1);   -- S8  Chroma stored/live
            sw_mir <= rsw(2);   -- S9  Mirror
            sw_vid <= rsw(3);   -- S10 Luma cells/video-through
            sw_frz <= rsw(4);   -- S11 Input live/freeze

            if rk1(9 downto 7) = 1 or rk1(9 downto 7) = 2 then m_zoom <= '1'; else m_zoom <= '0'; end if;
            if rk1(9 downto 7) = 7 then m_spin <= '1'; else m_spin <= '0'; end if;
        end if;
    end process p_ctrl;

    --------------------------------------------------------------------------
    -- Raster counters, cell DDAs, per-frame geometry latch.
    --------------------------------------------------------------------------
    p_position : process(clk)
        variable v_h_edge, v_v_edge : std_logic;
        variable v_x, v_y : unsigned(12 downto 0);
        variable v_first : std_logic;
        variable v_dy, v_du, v_dv : signed(10 downto 0);
        variable v_ny, v_nu, v_nv : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            prev_hsync_n <= data_in.hsync_n;
            prev_vsync_n <= data_in.vsync_n;
            prev_avid    <= data_in.avid;
            v_h_edge := '0'; v_v_edge := '0';
            if data_in.hsync_n = '0' and prev_hsync_n = '1' then v_h_edge := '1'; end if;
            if data_in.vsync_n = '0' and prev_vsync_n = '1' then v_v_edge := '1'; end if;

            if lfsr(0) = '1' then
                lfsr <= ('0' & lfsr(15 downto 1)) xor x"B400";
            else
                lfsr <= '0' & lfsr(15 downto 1);
            end if;

            if prev_avid = '1' and data_in.avid = '0' then af <= '1'; else af <= '0'; end if;

            lpy1 <= lpy0; lpy2 <= lpy1;
            lpu1 <= lpu0; lpu2 <= lpu1;
            lpv1 <= lpv0; lpv2 <= lpv1;

            if v_h_edge = '1' then
                if px > 0 then w_meas <= px; end if;
                px <= (others => '0');
                if line_act = '1' then
                    line_act <= '0';
                    py       <= py + 1;
                    h_meas   <= resize(py, 12) + 1;
                    v_y := ('0' & yacc) + resize(g_rows, 13);
                    if v_y >= ('0' & g_h) then
                        yacc        <= resize(v_y - ('0' & g_h), 12);
                        r           <= r + 1;
                        mid_done    <= '0';
                        sample_line <= '0';
                    else
                        yacc <= v_y(11 downto 0);
                        if mid_done = '0' and v_y >= ("00" & g_h(11 downto 1)) then
                            sample_line <= '1';
                            mid_done    <= '1';
                        else
                            sample_line <= '0';
                        end if;
                    end if;
                end if;
            elsif data_in.avid = '1' then
                frame_armed <= '1';
                v_first := not line_act;
                line_act <= '1';
                px <= px + 1;
                if frame_act = '0' then
                    frame_act <= '1';
                    py        <= (others => '0');
                end if;
                if v_first = '1' then
                    xacc <= (others => '0');
                    c    <= (others => '0');
                    cs   <= '0';
                    lpy0 <= unsigned(data_in.y);
                    lpu0 <= unsigned(data_in.u);
                    lpv0 <= unsigned(data_in.v);
                else
                    v_x := ('0' & xacc) + resize(g_cols, 13);
                    if v_x >= ('0' & g_w) then
                        xacc <= resize(v_x - ('0' & g_w), 12);
                        c    <= c + 1;
                        cs   <= '1';
                    else
                        xacc <= v_x(11 downto 0);
                        cs   <= '0';
                    end if;
                    -- one-pole IIR (shift 3 = ~8 px), all three channels
                    v_dy := signed('0' & unsigned(data_in.y)) - signed('0' & lpy0);
                    v_du := signed('0' & unsigned(data_in.u)) - signed('0' & lpu0);
                    v_dv := signed('0' & unsigned(data_in.v)) - signed('0' & lpv0);
                    v_ny := signed('0' & lpy0) + shift_right(v_dy, 3);
                    v_nu := signed('0' & lpu0) + shift_right(v_du, 3);
                    v_nv := signed('0' & lpv0) + shift_right(v_dv, 3);
                    lpy0 <= unsigned(v_ny(9 downto 0));
                    lpu0 <= unsigned(v_nu(9 downto 0));
                    lpv0 <= unsigned(v_nv(9 downto 0));
                end if;
            else
                cs <= '0';
            end if;

            if v_v_edge = '1' and frame_armed = '1' then
                frame_armed <= '0';
                frame_act   <= '0';
                py          <= (others => '0');
                yacc        <= (others => '0');
                r           <= (others => '0');
                mid_done    <= '0';
                sample_line <= '0';
                par         <= not par;
                seed1 <= lfsr;
                seed2 <= rotl16(lfsr, 5) xor x"5A5A";
                seed3 <= rotl16(lfsr, 11) xor x"C3A5";
                g_w <= w_meas;
                g_h <= h_meas;
                case s_gsel is
                    when "00" => g_cols <= to_unsigned(64, 7); g_rows <= to_unsigned(32, 6);
                                 g_cx <= to_unsigned(32, 6); g_ry <= to_unsigned(16, 5);
                                 g_cm1 <= to_signed(63, 10);
                    when "01" => g_cols <= to_unsigned(32, 7); g_rows <= to_unsigned(16, 6);
                                 g_cx <= to_unsigned(16, 6); g_ry <= to_unsigned(8, 5);
                                 g_cm1 <= to_signed(31, 10);
                    when "10" => g_cols <= to_unsigned(16, 7); g_rows <= to_unsigned(8, 6);
                                 g_cx <= to_unsigned(8, 6); g_ry <= to_unsigned(4, 5);
                                 g_cm1 <= to_signed(15, 10);
                    when others => g_cols <= to_unsigned(8, 7); g_rows <= to_unsigned(4, 6);
                                 g_cx <= to_unsigned(4, 6); g_ry <= to_unsigned(2, 5);
                                 g_cm1 <= to_signed(7, 10);
                end case;
            end if;

            if sample_line = '1' and (cs = '1' or af = '1') then
                wp0 <= '1';
            else
                wp0 <= '0';
            end if;
        end if;
    end process p_position;

    --------------------------------------------------------------------------
    -- Main pipeline.
    --------------------------------------------------------------------------
    p_pipe : process(clk)
        variable v_a15 : unsigned(15 downto 0);
        variable v_x1, v_x2, v_x3, v_y1, v_y2, v_y3 : unsigned(15 downto 0);
        variable v_sx, v_sy : signed(4 downto 0);
        variable v_ox, v_oy, v_jx, v_jy : signed(5 downto 0);
        variable v_issue : std_logic;
        variable v_rd : std_logic_vector(15 downto 0);
        variable v_yo, v_ysold : unsigned(3 downto 0);
        variable v_uo, v_vo : signed(3 downto 0);
        variable v_dec : unsigned(1 downto 0);
        variable v_ydec : signed(5 downto 0);
        variable v_m : unsigned(4 downto 0);
        variable v_m2 : unsigned(3 downto 0);
        variable v_sty, v_stu, v_stv : signed(4 downto 0);
        variable v_wy, v_wys : unsigned(3 downto 0);
        variable v_wu, v_wv : signed(3 downto 0);
        variable v_iy, v_iu, v_iv, v_py, v_pu, v_pv : unsigned(9 downto 0);
        variable v_yod : unsigned(3 downto 0);
        variable v_yo5 : signed(5 downto 0);
        variable v_ty, v_tu, v_tv : signed(11 downto 0);
        variable v_ly, v_lu, v_lv : signed(11 downto 0);
        variable v_yy : signed(11 downto 0);
        variable v_t5 : signed(4 downto 0);
        variable v_u11, v_v11 : signed(10 downto 0);
    begin
        if rising_edge(clk) then
            pipe(0) <= data_in;
            for i in 1 to LATENCY - 1 loop pipe(i) <= pipe(i - 1); end loop;

            -----------------------------------------------------------------
            -- F1
            -----------------------------------------------------------------
            f1_dc <= signed(resize(c(5 downto 0), 10)) - signed(resize(g_cx, 10));
            f1_dr <= signed(resize(r(4 downto 0), 10)) - signed(resize(g_ry, 10));
            f1_t8 <= (c(0) xor r(0)) & r(0) & (c(1) xor r(1)) & r(1) & (c(2) xor r(2)) & r(2);
            v_a15 := "00000" & c(5 downto 0) & r(4 downto 0);
            v_x1 := v_a15 xor rotl16(seed1, 3);
            v_y1 := rotl16(v_a15, 5) xor seed1;
            f1_h1 <= v_x1 + rotl16(v_y1, 7);
            v_x2 := v_a15 xor rotl16(seed2, 3);
            v_y2 := rotl16(v_a15, 5) xor seed2;
            f1_h2 <= v_x2 + rotl16(v_y2, 7);
            v_x3 := v_a15 xor rotl16(seed3, 3);
            v_y3 := rotl16(v_a15, 5) xor seed3;
            f1_h3 <= v_x3 + rotl16(v_y3, 7);
            f1_c <= c(5 downto 0);
            f1_r <= r(4 downto 0);

            -----------------------------------------------------------------
            -- F2
            -----------------------------------------------------------------
            if m_zoom = '1' then
                f2_sx <= f1_dc; f2_sy <= f1_dr;
            elsif m_spin = '1' then
                f2_sx <= f1_dr; f2_sy <= -f1_dc;
            else
                f2_sx <= (others => '0'); f2_sy <= (others => '0');
            end if;
            f2_t8 <= f1_t8;
            f2_h1 <= hround(f1_h1, 5, 3);
            f2_h2 <= hround(f1_h2, 5, 3);
            f2_h3 <= hround(f1_h3, 5, 3);
            f2_c <= f1_c; f2_r <= f1_r;

            -----------------------------------------------------------------
            -- F3
            -----------------------------------------------------------------
            f3_qx <= f2_sx(9 downto 6); f3_fx <= unsigned(f2_sx(5 downto 0));
            f3_qy <= f2_sy(9 downto 6); f3_fy <= unsigned(f2_sy(5 downto 0));
            f3_t8 <= f2_t8;
            f3_h1 <= hround(f2_h1, 11, 5);
            f3_h2 <= hround(f2_h2, 11, 5);
            f3_h3 <= hround(f2_h3, 11, 5);
            f3_c <= f2_c; f3_r <= f2_r;

            -----------------------------------------------------------------
            -- F4
            -----------------------------------------------------------------
            f4_qx <= f3_qx; f4_fx <= f3_fx; f4_qy <= f3_qy; f4_fy <= f3_fy;
            f4_t8 <= f3_t8;
            f4_h1 <= hround(f3_h1, 7, 3);
            f4_h2 <= hround(f3_h2, 7, 3);
            f4_h3 <= hround(f3_h3, 7, 3);
            f4_c <= f3_c; f4_r <= f3_r;

            -----------------------------------------------------------------
            -- P0: pulse capture of the ended cell (wp0 is k=1: F4 = pixel
            -- p-3, lp*2 = IIR after the cell's last pixel)
            -----------------------------------------------------------------
            p0_v  <= wp0;
            p0_qx <= f4_qx; p0_fx <= f4_fx; p0_qy <= f4_qy; p0_fy <= f4_fy;
            p0_t8 <= f4_t8;
            p0_h1 <= f4_h1; p0_h2 <= f4_h2; p0_h3 <= f4_h3;
            p0_c  <= f4_c;  p0_r  <= f4_r;
            p0_ly <= lpy2; p0_lu <= lpu2; p0_lv <= lpv2;

            -----------------------------------------------------------------
            -- P1: exposure + dither, chroma offsets, survival, jitter,
            -- stochastic rounding
            -----------------------------------------------------------------
            p1_v  <= p0_v;
            -- vq = (l - 64 + exp) * 9/8 + bayer  -> >>6 gives 0..15
            v_yy := signed(resize(p0_ly, 12)) - to_signed(64, 12) + s_exp;
            p1_vq <= resize(v_yy, 13) + resize(shift_right(v_yy, 3), 13) + signed(resize(p0_t8, 13));
            p1_du <= signed(resize(p0_lu, 11)) - to_signed(480, 11);
            p1_dv <= signed(resize(p0_lv, 11)) - to_signed(480, 11);
            if p0_h1(8 downto 0) < s_p9 then p1_surv <= '1'; else p1_surv <= '0'; end if;
            if p0_h2(7 downto 0) < s_jp8 then p1_jon <= '1'; else p1_jon <= '0'; end if;
            p1_jx <= map3(p0_h1(11 downto 9));
            p1_jy <= map3(p0_h1(14 downto 12));
            p1_half <= p0_h1(15);
            v_sx := resize(p0_qx, 5);
            if p0_h3(5 downto 0) < p0_fx then v_sx := v_sx + 1; end if;
            v_sy := resize(p0_qy, 5);
            if p0_h3(11 downto 6) < p0_fy then v_sy := v_sy + 1; end if;
            p1_sx <= v_sx; p1_sy <= v_sy;
            p1_hb <= p0_h2(13 downto 8);
            p1_c <= p0_c; p1_r <= p0_r;

            -----------------------------------------------------------------
            -- P2: quantise, mode offset + jitter (drift = half a cell/frame)
            -----------------------------------------------------------------
            p2_v <= p1_v;
            p2_ys <= clamp4u(shift_right(p1_vq, 6));
            p2_us <= clamp4s(shift_right(p1_du, 6));
            p2_vs <= clamp4s(shift_right(p1_dv, 6));
            p2_surv <= p1_surv;
            case s_mode is
                when "001"  => v_ox := -resize(p1_sx, 6); v_oy := -resize(p1_sy, 6);
                when "010"  => v_ox :=  resize(p1_sx, 6); v_oy :=  resize(p1_sy, 6);
                when "011"  => v_ox := (others => '0'); v_oy := (others => '0');
                               if p1_half = '1' then v_ox := to_signed( 1, 6); end if;
                when "100"  => v_ox := (others => '0'); v_oy := (others => '0');
                               if p1_half = '1' then v_ox := to_signed(-1, 6); end if;
                when "101"  => v_ox := (others => '0'); v_oy := (others => '0');
                               if p1_half = '1' then v_oy := to_signed( 1, 6); end if;
                when "110"  => v_ox := (others => '0'); v_oy := (others => '0');
                               if p1_half = '1' then v_oy := to_signed(-1, 6); end if;
                when "111"  => v_ox :=  resize(p1_sx, 6); v_oy :=  resize(p1_sy, 6);
                when others => v_ox := (others => '0');   v_oy := (others => '0');
            end case;
            if p1_jon = '1' then
                v_jx := resize(p1_jx, 6); v_jy := resize(p1_jy, 6);
            else
                v_jx := (others => '0'); v_jy := (others => '0');
            end if;
            p2_ox <= v_ox + v_jx;
            p2_oy <= v_oy + v_jy;
            p2_hb <= p1_hb;
            p2_c <= p1_c; p2_r <= p1_r;

            -----------------------------------------------------------------
            -- P3: source coordinates
            -----------------------------------------------------------------
            p3_v <= p2_v; p3_surv <= p2_surv;
            p3_ys <= p2_ys; p3_us <= p2_us; p3_vs <= p2_vs;
            p3_cx <= signed(resize(p2_c, 10)) + resize(p2_ox, 10);
            p3_ry <= signed(resize(p2_r, 9)) + resize(p2_oy, 9);
            p3_hb <= p2_hb;
            p3_c <= p2_c; p3_r <= p2_r;

            -----------------------------------------------------------------
            -- P4: mirror, row validity
            -----------------------------------------------------------------
            p4_v <= p3_v; p4_surv <= p3_surv;
            p4_ys <= p3_ys; p4_us <= p3_us; p4_vs <= p3_vs;
            if sw_mir = '1' then p4_cx <= g_cm1 - p3_cx; else p4_cx <= p3_cx; end if;
            if p3_ry >= 0 and p3_ry < signed(resize(g_rows, 9)) then p4_rv <= '1'; else p4_rv <= '0'; end if;
            p4_ry <= p3_ry;
            p4_hb <= p3_hb;
            p4_c <= p3_c; p4_r <= p3_r;

            -----------------------------------------------------------------
            -- P5: column validity, addresses
            -----------------------------------------------------------------
            p5_v <= p4_v; p5_surv <= p4_surv;
            p5_ys <= p4_ys; p5_us <= p4_us; p5_vs <= p4_vs;
            if p4_rv = '1' and p4_cx >= 0 and p4_cx < signed(resize(g_cols, 10)) then
                p5_fbv <= '1';
            else
                p5_fbv <= '0';
            end if;
            p5_addr <= unsigned(p4_ry(4 downto 0)) & unsigned(p4_cx(5 downto 0));
            p5_wa   <= p4_r & p4_c;
            p5_hb   <= p4_hb;

            -----------------------------------------------------------------
            -- Request register + read-port arbitration
            -----------------------------------------------------------------
            v_issue := '0';
            if req_v = '1' and cs = '0' then v_issue := '1'; end if;
            if v_issue = '1' then
                raddr <= req_addr;
                rtag1 <= '1';
                req_v <= '0';
            else
                raddr <= r(4 downto 0) & c(5 downto 0);
                rtag1 <= '0';
            end if;
            c1 <= (surv => req_surv, fbv => req_fbv, ys => req_ys, us => req_us,
                   vs => req_vs, hb => req_hb, wa => req_wa);
            if p5_v = '1' then
                req_v <= '1';
                req_addr <= p5_addr; req_wa <= p5_wa;
                req_surv <= p5_surv; req_fbv <= p5_fbv;
                req_ys <= p5_ys; req_us <= p5_us; req_vs <= p5_vs;
                req_hb <= p5_hb;
            end if;

            -- k=2: EBR reads
            rd_a <= bank_a(to_integer(raddr));
            rd_b <= bank_b(to_integer(raddr));
            rtag2 <= rtag1; c2 <= c1;

            -- k=3: fabric re-register
            rda_f <= rd_a; rdb_f <= rd_b;
            rtag3 <= rtag2; c3 <= c2;

            -----------------------------------------------------------------
            -- k=4: display hold / law operands
            -----------------------------------------------------------------
            if par = '0' then v_rd := rdb_f; else v_rd := rda_f; end if;
            if c3.fbv = '1' then
                v_yo := unsigned(v_rd(15 downto 12)); v_ysold := unsigned(v_rd(11 downto 8));
                v_uo := signed(v_rd(7 downto 4));     v_vo := signed(v_rd(3 downto 0));
            else
                v_yo := (others => '0'); v_ysold := (others => '0');
                v_uo := (others => '0'); v_vo := (others => '0');
            end if;
            if c3.surv = '1' then v_dec := "00"; elsif s_dec2 = '1' then v_dec := "10"; else v_dec := "01"; end if;
            v_ydec := signed(resize(v_yo, 6)) - signed(resize(v_dec, 6));
            if v_ydec < 0 then v_ydec := (others => '0'); end if;

            if rtag3 = '0' then
                disp4 <= v_rd;
                o4_v  <= '0';
            else
                o4_v <= '1';
            end if;
            o4_ydec  <= unsigned(v_ydec(3 downto 0));
            o4_yo    <= v_yo;
            o4_ysold <= v_ysold;
            o4_uo    <= v_uo;  o4_vo <= v_vo;
            if sw_frz = '1' then
                -- frozen input: the sample IS the old cell (pure feedback)
                o4_ys <= v_yo; o4_us <= v_uo; o4_vs <= v_vo;
                o4_dy <= (others => '0'); o4_ds <= (others => '0');
                o4_du <= (others => '0'); o4_dv <= (others => '0');
            else
                o4_ys <= c3.ys; o4_us <= c3.us; o4_vs <= c3.vs;
                o4_dy <= signed(resize(c3.ys, 5)) - signed(resize(v_yo, 5));
                o4_ds <= signed(resize(c3.ys, 5)) - signed(resize(v_ysold, 5));
                o4_du <= resize(c3.us, 5) - resize(v_uo, 5);
                o4_dv <= resize(c3.vs, 5) - resize(v_vo, 5);
            end if;
            o4_hb <= c3.hb;
            o4_wa <= c3.wa;

            -----------------------------------------------------------------
            -- k=5: feedback law -> new cell word
            -----------------------------------------------------------------
            -- stochastic IIR steps (floor + probabilistic carry)
            case s_bk is
                when "01" =>
                    v_sty := shift_right(o4_dy, 1); if o4_hb(0 downto 0) < unsigned(o4_dy(0 downto 0)) then v_sty := v_sty + 1; end if;
                    v_stu := shift_right(o4_du, 1); if o4_hb(1 downto 1) < unsigned(o4_du(0 downto 0)) then v_stu := v_stu + 1; end if;
                    v_stv := shift_right(o4_dv, 1); if o4_hb(2 downto 2) < unsigned(o4_dv(0 downto 0)) then v_stv := v_stv + 1; end if;
                when "10" =>
                    v_sty := shift_right(o4_dy, 2); if o4_hb(1 downto 0) < unsigned(o4_dy(1 downto 0)) then v_sty := v_sty + 1; end if;
                    v_stu := shift_right(o4_du, 2); if o4_hb(3 downto 2) < unsigned(o4_du(1 downto 0)) then v_stu := v_stu + 1; end if;
                    v_stv := shift_right(o4_dv, 2); if o4_hb(5 downto 4) < unsigned(o4_dv(1 downto 0)) then v_stv := v_stv + 1; end if;
                when others =>
                    v_sty := shift_right(o4_dy, 3); if o4_hb(2 downto 0) < unsigned(o4_dy(2 downto 0)) then v_sty := v_sty + 1; end if;
                    v_stu := shift_right(o4_du, 3); if o4_hb(5 downto 3) < unsigned(o4_du(2 downto 0)) then v_stu := v_stu + 1; end if;
                    v_stv := shift_right(o4_dv, 3); if (o4_hb(1 downto 0) & o4_hb(5)) < unsigned(o4_dv(2 downto 0)) then v_stv := v_stv + 1; end if;
            end case;
            v_m := unsigned(abs(o4_ds));
            if v_m >= 8 then v_m2 := "1111"; else v_m2 := v_m(2 downto 0) & '0'; end if;

            case s_law is
                when "00" =>   -- Trails
                    if o4_ys >= o4_ydec then
                        v_wy := o4_ys; v_wu := o4_us; v_wv := o4_vs;
                    else
                        v_wy := o4_ydec; v_wu := o4_uo; v_wv := o4_vo;
                    end if;
                    v_wys := o4_ys;
                when "01" =>   -- Blur
                    v_t5 := signed(resize(o4_yo, 5)) + v_sty;
                    v_wy := unsigned(v_t5(3 downto 0));
                    v_wu := resize(resize(o4_uo, 5) + v_stu, 4);
                    v_wv := resize(resize(o4_vo, 5) + v_stv, 4);
                    v_wys := o4_ys;
                when "10" =>   -- Motion
                    if v_m2 >= o4_ydec then
                        v_wy := v_m2; v_wu := o4_us; v_wv := o4_vs;
                    else
                        v_wy := o4_ydec; v_wu := o4_uo; v_wv := o4_vo;
                    end if;
                    v_wys := o4_ys;
                when others => -- Hold
                    if v_m > s_thr then
                        v_wy := o4_ys; v_wu := o4_us; v_wv := o4_vs; v_wys := o4_ys;
                    else
                        v_wy := o4_yo; v_wu := o4_uo; v_wv := o4_vo; v_wys := o4_ysold;
                    end if;
            end case;
            w5 <= std_logic_vector(v_wy) & std_logic_vector(v_wys) & std_logic_vector(v_wu) & std_logic_vector(v_wv);
            wv5 <= o4_v;
            waddr5 <= o4_wa;

            -- k=6: single write per bank
            if wv5 = '1' then
                if par = '0' then
                    bank_a(to_integer(waddr5)) <= w5;
                else
                    bank_b(to_integer(waddr5)) <= w5;
                end if;
            end if;

            -----------------------------------------------------------------
            -- Display: palette regs (k=3/4), multiplies (k=5), compose (k=6)
            -----------------------------------------------------------------
            v_iy := INK_Y(to_integer(s_psel)); v_iu := INK_U(to_integer(s_psel)); v_iv := INK_V(to_integer(s_psel));
            v_py := PAP_Y(to_integer(s_psel)); v_pu := PAP_U(to_integer(s_psel)); v_pv := PAP_V(to_integer(s_psel));
            pal3_iy <= v_iy; pal3_iu <= v_iu; pal3_iv <= v_iv;
            pal3_py <= v_py; pal3_pu <= v_pu; pal3_pv <= v_pv;
            pal4_py <= pal3_py; pal4_pu <= pal3_pu; pal4_pv <= pal3_pv;
            pal4_dy <= signed(resize(pal3_iy, 11)) - signed(resize(pal3_py, 11));
            pal4_du <= signed(resize(pal3_iu, 11)) - signed(resize(pal3_pu, 11));
            pal4_dv <= signed(resize(pal3_iv, 11)) - signed(resize(pal3_pv, 11));

            -- k=5
            if sw_inv = '1' then v_yod := not unsigned(disp4(15 downto 12)); else v_yod := unsigned(disp4(15 downto 12)); end if;
            v_yo5 := signed(resize(v_yod, 6));
            m5_ty <= resize(pal4_dy * v_yo5, 16);
            m5_tu <= resize(pal4_du * v_yo5, 16);
            m5_tv <= resize(pal4_dv * v_yo5, 16);
            v_ly := signed(resize(unsigned(pipe(4).y), 12)) - to_signed(64, 12);
            v_lu := signed(resize(unsigned(pipe(4).u), 12)) - to_signed(512, 12);
            v_lv := signed(resize(unsigned(pipe(4).v), 12)) - to_signed(512, 12);
            m5_vy <= resize(v_ly * v_yo5, 16);
            m5_vu <= resize(v_lu * v_yo5, 16);
            m5_vv <= resize(v_lv * v_yo5, 16);
            y5_46 <= (resize(v_yod, 10) sll 5) + (resize(v_yod, 10) sll 3) + (resize(v_yod, 10) sll 2) + (resize(v_yod, 10) sll 1);
            v_u11 := to_signed(512, 11) + shift_left(resize(signed(disp4(7 downto 4)), 11), 6);
            v_v11 := to_signed(512, 11) + shift_left(resize(signed(disp4(3 downto 0)), 11), 6);
            u5_st <= unsigned(v_u11(9 downto 0));
            v5_st <= unsigned(v_v11(9 downto 0));
            pal5_py <= pal4_py; pal5_pu <= pal4_pu; pal5_pv <= pal4_pv;

            -- k=6: compose
            if sw_vid = '1' then
                v_ty := to_signed(64, 12) + resize(shift_right(m5_vy, 4), 12);
            elsif s_psel = 0 then
                v_ty := signed(resize(y5_46, 12)) + to_signed(64, 12);
            else
                v_ty := signed(resize(pal5_py, 12)) + resize(shift_right(m5_ty, 4), 12);
            end if;
            if sw_lch = '1' then
                v_tu := to_signed(512, 12) + resize(shift_right(m5_vu, 4), 12);
                v_tv := to_signed(512, 12) + resize(shift_right(m5_vv, 4), 12);
            elsif s_psel = 0 then
                v_tu := signed(resize(u5_st, 12));
                v_tv := signed(resize(v5_st, 12));
            else
                v_tu := signed(resize(pal5_pu, 12)) + resize(shift_right(m5_tu, 4), 12);
                v_tv := signed(resize(pal5_pv, 12)) + resize(shift_right(m5_tv, 4), 12);
            end if;
            if v_ty < 64 then v_ty := to_signed(64, 12); elsif v_ty > 1000 then v_ty := to_signed(1000, 12); end if;
            if v_tu < 64 then v_tu := to_signed(64, 12); elsif v_tu > 960 then v_tu := to_signed(960, 12); end if;
            if v_tv < 64 then v_tv := to_signed(64, 12); elsif v_tv > 960 then v_tv := to_signed(960, 12); end if;
            y6 <= unsigned(v_ty(9 downto 0));
            u6 <= unsigned(v_tu(9 downto 0));
            v6 <= unsigned(v_tv(9 downto 0));
            blank6 <= not pipe(5).avid;

            -- k=7: output register with blanking gate
            if blank6 = '1' then
                s_io.y <= std_logic_vector(C_BLKY);
                s_io.u <= std_logic_vector(C_MID);
                s_io.v <= std_logic_vector(C_MID);
            else
                s_io.y <= std_logic_vector(y6);
                s_io.u <= std_logic_vector(u6);
                s_io.v <= std_logic_vector(v6);
            end if;
            s_io.hsync_n <= pipe(LATENCY - 1).hsync_n;
            s_io.vsync_n <= pipe(LATENCY - 1).vsync_n;
            s_io.avid    <= pipe(LATENCY - 1).avid;
            s_io.field_n <= pipe(LATENCY - 1).field_n;
        end if;
    end process p_pipe;

    data_out.y       <= s_io.y;
    data_out.u       <= s_io.u;
    data_out.v       <= s_io.v;
    data_out.hsync_n <= s_io.hsync_n;
    data_out.vsync_n <= s_io.vsync_n;
    data_out.avid    <= s_io.avid;
    data_out.field_n <= s_io.field_n;

end architecture mosaic;
