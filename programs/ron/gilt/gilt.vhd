-- gilt.vhd -- GILT v2: live video re-rendered as a Gustav Klimt Golden Phase painting.
--
-- Built from GILT_V2_PROMPT.md + STYLE_SPEC.md (measured from the paintings). Faces and
-- hands stay photographic but porcelain-pale; everything else becomes gold leaf: a calm
-- glittering FIELD and a dense ROBE of cloisonne cells -- bars, tablets, ring-discs (Kiss
-- bank), eyes, spirals, concentric squares, checkers, ovals (Adele/Stoclet bank).
--
-- Structure (streaming, no frame buffer, latency C_LAT for every mode):
--   * coarse map: per 2^bs-px block (64 HD / 32 SD): mean luma, detail, skin and green,
--     sampled on one rotating line per block row, IIR-smoothed frame to frame
--     in two ping-pong banks (frame N writes, frame N+1 reads).
--   * cell layout: horizontal BANDS of hashed height, each cut into TILES of hashed width
--     with a per-band phase (tile edges never align between bands); one hashed KD split per
--     tile.  A small engine precomputes each tile (geometry + its zone from the coarse map)
--     two tiles ahead of the beam; the pixel path only compares and subtracts.
--   * zone per tile: ROBE where detail + closeness to the skin centroid beats ORNAMENT,
--     else FIELD (MEADOW optional); flesh is keyed per pixel (CAMEO: gated by a box round
--     the skin centroid).
--   * motifs: compares on the cell-local offset (octagonal radius, Chebyshev, 16-sector
--     angle) -> palette index -> palette ROM (bank x LEAF x 64), then grain, flecks,
--     relief, aura, flesh and GILD staging.
--
-- Hardware contracts honoured: U/V swapped at input latch and output compose; generated
-- avid (parity-locked to hsync); outputs blanked to 64/512/512 outside it; original video
-- carried in an EBR delay line (not a deep FF pipe); single latency; field-latched params;
-- vsync serration guarded (saw-active); frame-line hashing under interlace.
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;
use work.gilt_pkg.all;

architecture gilt of program_top is

    constant C_LAT  : integer := 12;    -- data_in -> data_out, all modes
    constant C_WMIN : integer := 20;    -- min tile width (engine needs 17 clocks per tile)
    constant C_LMIN : integer := 8;     -- min leaf size after a split
    constant C_HMIN : integer := 16;    -- min band height
    constant C_SKCB : integer := 440;   -- live-video skin centre (BT.601 10-bit)
    constant C_SKCR : integer := 590;

    constant C_SEED   : unsigned(15 downto 0) := x"3A91";
    constant C_SLT_TA : unsigned(15 downto 0) := x"51F3";
    constant C_SLT_TB : unsigned(15 downto 0) := x"A7C5";
    constant C_SLT_TM : unsigned(15 downto 0) := x"2D6B";
    constant C_SLT_BD : unsigned(15 downto 0) := x"C0DE";
    constant C_SLT_GR : unsigned(15 downto 0) := x"7E11";
    constant C_SLT_MD : unsigned(15 downto 0) := x"1B9D";

    -- motif ids
    constant M_PLAIN  : integer := 0;
    constant M_BAR    : integer := 1;
    constant M_TABLET : integer := 2;
    constant M_BUTTON : integer := 3;
    constant M_RING   : integer := 4;
    constant M_ENGR   : integer := 5;
    constant M_SPIRAL : integer := 6;
    constant M_CONCSQ : integer := 7;
    constant M_CHECK  : integer := 8;
    constant M_EYE    : integer := 9;
    constant M_OVAL   : integer := 10;
    constant M_TESS   : integer := 11;
    constant M_BARTAB : integer := 12;
    constant M_INK    : integer := 13;
    constant M_FRECT  : integer := 14;
    constant M_MEADOW : integer := 15;

    ----------------------------------------------------------------------------
    -- helpers
    ----------------------------------------------------------------------------
    function rotl16(v : unsigned(15 downto 0); k : natural) return unsigned is
    begin
        return v(15 - k downto 0) & v(15 downto 16 - k);
    end function;

    -- hash (tools/hash_check.py, tools/hash_search.py): 4 registered stages
    function f_hs0(a, b : unsigned(15 downto 0)) return unsigned is
    begin
        return (a xor C_SEED) + rotl16(b, 8);
    end function;
    function f_hs1(v : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := v + shift_left(v, 11) + shift_left(v, 12);
        return t xor shift_right(t, 7);
    end function;
    function f_hs2(v : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := v + shift_left(v, 3) + shift_left(v, 11);
        return t xor shift_right(t, 6);
    end function;
    function f_hs3(v : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := v + shift_left(v, 4) + shift_left(v, 8);
        return t xor shift_right(t, 9);
    end function;

    -- xorshift16 (7,9,8): per-pixel grain / meadow streams, reseeded per line
    function f_xs(v : unsigned(15 downto 0)) return unsigned is
        variable t : unsigned(15 downto 0);
    begin
        t := v xor shift_left(v, 7);
        t := t xor shift_right(t, 9);
        t := t xor shift_left(t, 8);
        return t;
    end function;

    function f_abs(v : signed) return unsigned is
    begin
        if v < 0 then
            return unsigned(-v);
        else
            return unsigned(v);
        end if;
    end function;

    function f_absn(v : signed; n : natural) return unsigned is
    begin
        return resize(f_abs(v), n);
    end function;

    function f_s13(u : unsigned) return signed is
    begin
        return signed(resize(u, 13));
    end function;

    function f_min_s(a, b : signed) return signed is
    begin
        if a < b then return a; else return b; end if;
    end function;

    function f_umin(a, b : unsigned) return unsigned is
        variable ra : unsigned(a'length - 1 downto 0) := a;
        variable rb : unsigned(b'length - 1 downto 0) := b;
    begin
        if ra < rb then return ra; else return rb; end if;
    end function;
    function f_umax(a, b : unsigned) return unsigned is
        variable ra : unsigned(a'length - 1 downto 0) := a;
        variable rb : unsigned(b'length - 1 downto 0) := b;
    begin
        if ra > rb then return ra; else return rb; end if;
    end function;

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

    -- IIR toward a target with a guaranteed 1-step minimum (no stalling short of target)
    function f_iiru(o, n : unsigned; k : natural) return unsigned is
        variable d, s : signed(o'length + 1 downto 0);
    begin
        d := signed(resize(n, o'length + 2)) - signed(resize(o, o'length + 2));
        s := shift_right(d, k);
        if d > 0 and s = 0 then
            s := to_signed(1, s'length);
        end if;
        return resize(unsigned(signed(resize(o, o'length + 2)) + s), o'length);
    end function;

    function f_iirs(o, n : signed; k : natural) return signed is
        variable d, s : signed(o'length + 1 downto 0);
    begin
        d := resize(n, o'length + 2) - resize(o, o'length + 2);
        s := shift_right(d, k);
        if d > 0 and s = 0 then
            s := to_signed(1, s'length);
        end if;
        return resize(resize(o, o'length + 2) + s, o'length);
    end function;

    -- jewel palette index for a colour class (48-55 jewels, 55-58 greens, 60 cobalt)
    function f_jewel(cls : unsigned(1 downto 0); h : unsigned(2 downto 0)) return unsigned is
    begin
        case to_integer(cls) is
            when 1 =>
                case to_integer(h(1 downto 0)) is
                    when 0 => return to_unsigned(49, 6);    -- vermilion
                    when 1 => return to_unsigned(50, 6);    -- coral
                    when 2 => return to_unsigned(52, 6);    -- rose
                    when others => return to_unsigned(54, 6);  -- brick
                end case;
            when 2 =>
                case to_integer(h(1 downto 0)) is
                    when 0 => return to_unsigned(48, 6);    -- slate violet
                    when 1 => return to_unsigned(51, 6);    -- mauve
                    when 2 => return to_unsigned(53, 6);    -- lapis
                    when others => return to_unsigned(60, 6);  -- cobalt
                end case;
            when 3 =>
                case to_integer(h(1 downto 0)) is
                    when 0 | 1 => return to_unsigned(55, 6);  -- teal
                    when 2 => return to_unsigned(57, 6);
                    when others => return to_unsigned(58, 6);
                end case;
            when others =>
                return "110" & h;
        end case;
    end function;

    ----------------------------------------------------------------------------
    -- tile record (one level-0 cell + its KD split, zone and hashes)
    ----------------------------------------------------------------------------
    type t_tile is record
        wm1  : unsigned(9 downto 0);    -- width - 1
        w    : unsigned(9 downto 0);
        s1   : std_logic;               -- split?
        ax   : std_logic;               -- '1' = cut along x (vertical cut line)
        p    : unsigned(9 downto 0);    -- split position (tile-local)
        rm   : unsigned(9 downto 0);    -- side - p
        zone : unsigned(1 downto 0);    -- 0 field, 1 robe, 2 meadow
        tone : unsigned(4 downto 0);    -- block mean luma (gold-ramp level, 0..31)
        skin : std_logic;               -- block holds skin (hair/ink rule)
        gild : std_logic;               -- laid in gold at the current GILD level
        ha   : unsigned(7 downto 0);    -- leaf hash (leaf 1 uses a derived value)
        cls  : unsigned(1 downto 0);    -- colour class: 0 neutral 1 warm 2 cool 3 green
    end record;
    constant C_TILE0 : t_tile := (wm1 => to_unsigned(63, 10), w => to_unsigned(64, 10), s1 => '0',
        ax => '0', p => (others => '0'), rm => (others => '0'), zone => "00", tone => "10000",
        skin => '0', gild => '0', ha => (others => '0'),
        cls => "00");

    ----------------------------------------------------------------------------
    -- input / timing
    ----------------------------------------------------------------------------
    signal i_y, i_u, i_v   : unsigned(9 downto 0) := (others => '0');
    signal i_hs, i_vs, i_av, i_fn : std_logic := '1';
    signal i_hs_d, i_vs_d, i_av_d : std_logic := '1';
    signal hc      : unsigned(11 downto 0) := (others => '0');
    signal x0      : unsigned(11 downto 0) := to_unsigned(192, 12);
    signal av_first : std_logic := '0';
    signal avs     : unsigned(11 downto 0) := (others => '0');
    signal v_up, v_dn, v_far : unsigned(7 downto 0) := (others => '0');
    signal saw     : std_logic := '0';
    signal fev     : std_logic := '0';           -- field event pulse
    signal lact    : std_logic := '0';
    signal avcnt   : unsigned(11 downto 0) := (others => '0');
    signal w_meas  : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal aline   : unsigned(10 downto 0) := (others => '0');
    signal yf      : unsigned(10 downto 0) := (others => '0');   -- frame line of current line
    signal fbit    : std_logic := '0';
    signal il      : std_logic := '0';           -- interlaced
    signal fc      : unsigned(15 downto 0) := (others => '0');
    signal fp      : std_logic := '0';           -- field parity (map ping-pong)
    signal hs_ev, hs_ev_d : std_logic := '0';
    signal endact  : std_logic := '0';           -- pulse: x reached W on an active line
    -- registered beam position for the pixel in the input latch (P0): equality
    -- compares only, so nothing downstream sits behind a subtract-and-compare
    signal xc      : unsigned(11 downto 0) := (others => '0');
    signal x0m1, wm1 : unsigned(11 downto 0) := (others => '0');
    signal p0_xz, p0_run, p0_end : std_logic := '0';

    -- latched controls
    signal k1, k2, k3, k4, k5, k6, p12 : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal sw      : std_logic_vector(4 downto 0) := "10010";
    signal tid     : std_logic_vector(3 downto 0) := "0111";

    -- frame-latched geometry / derived parameters
    signal wl      : unsigned(10 downto 0) := to_unsigned(1920, 11);
    signal hl      : unsigned(10 downto 0) := to_unsigned(1080, 11);
    signal s_s     : unsigned(8 downto 0) := to_unsigned(64, 9);
    signal s8      : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal bs6     : std_logic := '1';           -- block size 64 ('1') or 32
    signal ncols   : unsigned(5 downto 0) := to_unsigned(30, 6);
    signal psplit  : unsigned(8 downto 0) := (others => '0');
    signal plthr   : unsigned(7 downto 0) := to_unsigned(136, 8);
    signal jthr    : unsigned(7 downto 0) := to_unsigned(77, 8);
    signal rthr    : unsigned(8 downto 0) := to_unsigned(127, 9);
    signal skt     : unsigned(9 downto 0) := to_unsigned(90, 10);
    signal leaf3   : unsigned(2 downto 0) := "100";
    signal bank    : std_logic := '0';
    signal gthr    : unsigned(8 downto 0) := (others => '1');
    signal mr      : unsigned(2 downto 0) := "100";
    signal gcomp   : unsigned(2 downto 0) := "000";
    signal pregild : std_logic := '0';
    signal so      : unsigned(5 downto 0) := (others => '0');   -- sample line offset in block
    signal ymd     : unsigned(10 downto 0) := to_unsigned(900, 11);
    signal prep0   : std_logic := '0';           -- first-line prep request (from vseq)

    -- vblank sequencer
    signal vs_st   : integer range 0 to 15 := 0;

    -- generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc   : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr  : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt  : unsigned(11 downto 0) := (others => '0');

    -- sync delay
    signal sr_hs, sr_vs, sr_fn : std_logic_vector(0 to C_LAT - 1) := (others => '1');
    signal sr_ga  : std_logic_vector(0 to C_LAT - 4) := (others => '0');

    ----------------------------------------------------------------------------
    -- coarse map + sample-line statistics
    ----------------------------------------------------------------------------
    -- map word: Y5(15:11) det3(10:8) Cb4(7:4) Cr4(3:0)
    type t_map is array (0 to 511) of std_logic_vector(15 downto 0);
    signal map0, map1 : t_map := (others => x"8000");   -- mid grey, neutral
    signal mq0, mq1   : std_logic_vector(15 downto 0) := (others => '0');
    signal m_qr       : std_logic_vector(15 downto 0) := (others => '0');
    signal m_raddr    : unsigned(8 downto 0) := (others => '0');
    signal m_waddr    : unsigned(8 downto 0) := (others => '0');
    signal m_wdata    : std_logic_vector(15 downto 0) := (others => '0');
    signal m_we       : std_logic := '0';
    attribute no_rw_check : boolean;
    attribute no_rw_check of map0 : signal is true;
    attribute no_rw_check of map1 : signal is true;

    type t_seg is array (0 to 255) of std_logic_vector(15 downto 0);
    signal segm       : t_seg := (others => (others => '0'));
    signal sg_q, sg_qr : std_logic_vector(15 downto 0) := (others => '0');
    signal sg_raddr   : unsigned(7 downto 0) := (others => '0');
    signal sg_waddr   : unsigned(7 downto 0) := (others => '0');
    signal sg_wdata   : std_logic_vector(15 downto 0) := (others => '0');
    signal sg_we      : std_logic := '0';
    attribute no_rw_check of segm : signal is true;

    signal samp, samp_pend : std_logic := '0';
    signal rows       : unsigned(3 downto 0) := (others => '0');
    signal a_y, a_d   : unsigned(13 downto 0) := (others => '0');
    signal a_cb, a_cr : signed(12 downto 0) := (others => '0');
    signal a_col      : unsigned(5 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- hash unit + engine (band walker, tile precompute, IIR pass) + beam walker
    ----------------------------------------------------------------------------
    signal h_a, h_b   : unsigned(15 downto 0) := (others => '0');
    signal hq1, hq2, hq3, hq4 : unsigned(15 downto 0) := (others => '0');

    signal es        : integer range 0 to 63 := 0;
    signal pr_first  : std_logic := '0';
    signal pr_mode   : std_logic := '0';      -- 1 while preparing a line (3 tiles)
    signal pr_req, pr_req_first : std_logic := '0';
    signal adv_req   : std_logic := '0';
    signal yt        : unsigned(10 downto 0) := (others => '0');
    signal ic        : unsigned(5 downto 0) := (others => '0');
    signal ipv       : std_logic_vector(0 to 2) := (others => '0');
    type t_c6 is array (0 to 2) of unsigned(5 downto 0);
    signal ipc       : t_c6 := (others => (others => '0'));

    -- band state (committed for the CURRENT line) and next-line working copy
    signal bj, nj    : unsigned(9 downto 0) := (others => '0');
    signal tv, ntv   : unsigned(9 downto 0) := (others => '0');
    signal hb, nhb   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal by0, nby0 : signed(12 downto 0) := (others => '0');
    signal ph, nph   : unsigned(3 downto 0) := (others => '0');
    signal cyb       : signed(12 downto 0) := (others => '0');
    signal rowb      : unsigned(3 downto 0) := (others => '0');
    signal lowb      : std_logic := '0';
    signal gseed, mseed, gseed_n, mseed_n : unsigned(15 downto 0) := x"ACE1";
    signal hbi       : unsigned(4 downto 0) := (others => '0');
    signal mul_in    : unsigned(4 downto 0) := (others => '0');
    signal mul_out   : unsigned(10 downto 0);

    -- tile work registers
    signal t_k       : unsigned(7 downto 0) := (others => '0');
    signal t_xs      : signed(12 downto 0) := (others => '0');
    signal t_slot    : integer range 0 to 1 := 0;
    signal t_m       : unsigned(4 downto 0) := (others => '0');
    signal t_rnd     : unsigned(7 downto 0) := (others => '0');
    signal t_fs      : unsigned(2 downto 0) := (others => '0');
    signal t_flip    : std_logic := '0';
    signal t_wr      : unsigned(10 downto 0) := (others => '0');
    signal t_w       : unsigned(9 downto 0) := (others => '0');
    signal t_cx      : signed(12 downto 0) := (others => '0');
    signal t_side    : unsigned(9 downto 0) := (others => '0');
    signal t_fk      : unsigned(3 downto 0) := (others => '0');
    signal t_p       : unsigned(9 downto 0) := (others => '0');
    signal t_rm      : unsigned(9 downto 0) := (others => '0');
    signal t_sa      : unsigned(8 downto 0) := (others => '0');
    signal t_sat     : unsigned(4 downto 0) := (others => '0');
    signal t_score   : unsigned(9 downto 0) := (others => '0');
    signal t_rank    : unsigned(8 downto 0) := (others => '0');
    signal t_jit     : unsigned(4 downto 0) := (others => '0');
    signal t_rec     : t_tile := C_TILE0;
    signal t_det     : unsigned(2 downto 0) := (others => '0');

    -- beam walker: current tile + 1-deep queue (the engine finishes a tile in 17
    -- clocks, inside the 20-px minimum tile, so one look-ahead record is enough)
    signal cur, q0   : t_tile := C_TILE0;
    signal tu        : unsigned(9 downto 0) := (others => '0');

    ----------------------------------------------------------------------------
    -- pixel path (P0 = input latch ... P11 = output register; C_LAT = 12)
    ----------------------------------------------------------------------------
    signal p1_av, p1_ls : std_logic := '0';
    signal p1_x      : signed(12 downto 0) := (others => '0');
    -- early keys
    signal k1_dcb, k1_dcr : unsigned(9 downto 0) := (others => '0');
    signal k1_yok, k1_dark : std_logic := '0';
    signal k1_cb, k1_cr, k2_cb, k2_cr : signed(6 downto 0) := (others => '0');
    signal k1_ya, k1_yd1, k1_yd2 : unsigned(7 downto 0) := (others => '0');
    signal k2_skin, k2_av : std_logic := '0';
    signal k2_ya, k2_det : unsigned(7 downto 0) := (others => '0');
    signal k2_x      : signed(12 downto 0) := (others => '0');

    -- P2: leaf of the tile
    signal p2_u, p2_v, p2_cw, p2_ch : unsigned(9 downto 0) := (others => '0');
    signal p2_zone   : unsigned(1 downto 0) := (others => '0');
    signal p2_tone   : unsigned(4 downto 0) := (others => '0');
    signal p2_ya     : unsigned(7 downto 0) := (others => '0');
    signal p2_skin, p2_gild, p2_av, p2_ls : std_logic := '0';
    signal p2_h      : unsigned(7 downto 0) := (others => '0');
    signal p2_cls    : unsigned(1 downto 0) := (others => '0');
    -- P3: |offsets|, size classes
    signal p3_ax, p3_ay : unsigned(10 downto 0) := (others => '0');
    signal p3_sx, p3_sy, p3_tall, p3_wide, p3_small, p3_large, p3_av, p3_ls, p3_rb : std_logic := '0';
    signal p3_mh, p3_cw, p3_ch, p3_mn, p3_v : unsigned(9 downto 0) := (others => '0');
    signal p3_u      : unsigned(5 downto 0) := (others => '0');
    signal p3_zone   : unsigned(1 downto 0) := (others => '0');
    signal p3_tone   : unsigned(4 downto 0) := (others => '0');
    signal p3_h      : unsigned(7 downto 0) := (others => '0');
    signal p3_cls    : unsigned(1 downto 0) := (others => '0');
    -- P4: motif, inner extents, max/min
    signal p4_mot    : unsigned(3 downto 0) := (others => '0');
    signal p4_hx, p4_hy, p4_r0 : signed(11 downto 0) := (others => '0');
    signal p4_rmax, p4_rmin, p4_ax, p4_ay : unsigned(10 downto 0) := (others => '0');
    signal p4_sx, p4_sy, p4_large, p4_av, p4_ls, p4_rb : std_logic := '0';
    signal p4_u      : unsigned(5 downto 0) := (others => '0');
    signal p4_v      : unsigned(9 downto 0) := (others => '0');
    -- P5: octagonal radius + extents
    signal p5_r      : unsigned(11 downto 0) := (others => '0');
    signal p5_rmax   : unsigned(10 downto 0) := (others => '0');
    signal p5_rq, p5_hx, p5_hy, p5_ew : signed(11 downto 0) := (others => '0');
    signal p5_ax, p5_ay, p5_ayb, p5_vt : unsigned(10 downto 0) := (others => '0');
    signal p5_sx, p5_sy, p5_large, p5_av, p5_ls, p5_rb : std_logic := '0';
    signal p5_mot    : unsigned(3 downto 0) := (others => '0');
    signal p5_u, p5_v : unsigned(5 downto 0) := (others => '0');
    -- P6: compares, aura
    signal p6_inR, p6_in34, p6_in2, p6_in4, p6_olR, p6_ol2 : std_logic := '0';
    signal p6_cR, p6_c34, p6_c2, p6_c4, p6_olc : std_logic := '0';
    signal p6_inx, p6_iny, p6_bdx, p6_bdy : std_logic := '0';
    signal p6_tin, p6_tout, p6_tri, p6_dot, p6_iris, p6_pupil, p6_eng, p6_slit : std_logic := '0';
    signal p6_dl, p6_omx, p6_omn : unsigned(12 downto 0) := (others => '0');
    signal p6_ew     : signed(11 downto 0) := (others => '0');
    signal p6_sub    : unsigned(2 downto 0) := (others => '0');
    signal p6_quad   : unsigned(1 downto 0) := (others => '0');
    signal p6_r      : unsigned(11 downto 0) := (others => '0');
    signal p6_mot    : unsigned(3 downto 0) := (others => '0');
    signal p6_u, p6_v : unsigned(5 downto 0) := (others => '0');
    signal p6_large, p6_av, p6_ls, p6_aura, p6_upm : std_logic := '0';
    signal hd_cnt    : unsigned(1 downto 0) := (others => '0');
    signal hd_prev   : std_logic := '0';
    -- P7: lens, spiral, flag vector
    signal p7_lens, p7_lensi, p7_pupil, p7_dot, p7_spl, p7_slit : std_logic := '0';
    signal p7_f      : std_logic_vector(19 downto 0) := (others => '0');
    signal p7_mot    : unsigned(3 downto 0) := (others => '0');
    signal p7_u, p7_v : unsigned(5 downto 0) := (others => '0');
    signal p7_av, p7_ls, p7_aura, p7_upm : std_logic := '0';
    signal ms        : unsigned(15 downto 0) := x"BEEF";
    signal mph       : unsigned(1 downto 0) := (others => '0');
    -- P8: palette address + relief; P9 carries
    signal pal_a     : unsigned(9 downto 0) := (others => '0');
    signal p8_leaf, p8_field, p8_robe, p8_av, p8_ls, p8_gild, p8_hl, p8_sh, p8_glint, p8_mask : std_logic := '0';
    signal p8_tone   : unsigned(4 downto 0) := (others => '0');
    signal p9_leaf, p9_field, p9_robe, p9_gild, p9_hl, p9_sh, p9_glint : std_logic := '0';
    signal p9_tone   : unsigned(4 downto 0) := (others => '0');
    signal gs        : unsigned(15 downto 0) := x"ACE1";
    signal gpar      : std_logic := '0';
    signal fk_hold   : std_logic := '0';
    signal pal_q, pal_qr : std_logic_vector(31 downto 0) := (others => '0');
    -- P10: luma offset, alternative (flesh / video), select
    signal p10_s     : signed(10 downto 0) := (others => '0');
    signal p10_ay, p10_acb, p10_acr : unsigned(9 downto 0) := (others => '0');
    signal p10_alt   : std_logic := '0';
    signal o_y, o_cb, o_cr : unsigned(9 downto 0) := (others => '0');

    -- per-pixel flags: flesh P3 -> P9, dark P2 -> P7
    signal fl_sr     : std_logic_vector(3 to 9) := (others => '0');
    signal dk_sr     : std_logic_vector(2 to 7) := (others => '0');

    ----------------------------------------------------------------------------
    -- EBR: video delay, per-cell carry, palette ROM, aura/relief line buffer
    ----------------------------------------------------------------------------
    type t_vd is array (0 to 255) of std_logic_vector(31 downto 0);
    signal vdr       : t_vd := (others => (others => '0'));
    signal vd_wp     : unsigned(7 downto 0) := (others => '0');
    signal vd_q, vd_qr : std_logic_vector(31 downto 0) := (others => '0');
    attribute no_rw_check of vdr : signal is true;

    -- per-pixel values written at P2, read back at P7 (EBR delay instead of FFs):
    -- cls(25:24) ya(23:16) zone(15:14) tone(13:9) skin(8) h(7:0) ; gild(26)
    type t_cry is array (0 to 255) of std_logic_vector(31 downto 0);
    signal cry       : t_cry := (others => (others => '0'));
    signal cr_wp     : unsigned(7 downto 0) := (others => '0');
    signal cr_q, cr_qr : std_logic_vector(31 downto 0) := (others => '0');
    attribute no_rw_check of cry : signal is true;

    type t_lb is array (0 to 2047) of std_logic_vector(3 downto 0);
    signal lbr       : t_lb := (others => (others => '0'));
    signal lb_q, lb_qr : std_logic_vector(3 downto 0) := (others => '0');
    signal lb_wd     : std_logic_vector(3 downto 0) := (others => '0');
    signal lb_we     : std_logic := '0';
    signal lbx_r, lbx_w : unsigned(10 downto 0) := (others => '0');
    signal lb_wa     : unsigned(10 downto 0) := (others => '0');
    attribute no_rw_check of lbr : signal is true;
    signal lb_vd, lb_vd_d1 : unsigned(1 downto 0) := (others => '0');
    signal lmask     : std_logic := '0';

begin

    ----------------------------------------------------------------------------
    -- input latch, line / field timing, x origin, controls
    ----------------------------------------------------------------------------
    p_in : process(clk)
        variable v_d : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- HW carries chroma U<->V swapped vs BT.601: swap once here (internal u = Cb)
            i_y <= unsigned(data_in.y);
            i_u <= unsigned(data_in.v);
            i_v <= unsigned(data_in.u);
            i_hs <= data_in.hsync_n;  i_hs_d <= i_hs;
            i_vs <= data_in.vsync_n;  i_vs_d <= i_vs;
            i_av <= data_in.avid;     i_av_d <= i_av;
            i_fn <= data_in.field_n;

            -- hc counts clocks from hsync fall, aligned with i_*
            if data_in.hsync_n = '0' and i_hs = '1' then
                hc <= (others => '0');
            elsif hc /= 4095 then
                hc <= hc + 1;
            end if;

            hs_ev  <= '0';
            fev    <= '0';
            endact <= '0';

            -- avid start of the line -> x-origin vote (hsync-locked x, robust to
            -- analog avid wander)
            if i_av = '1' and i_av_d = '0' and av_first = '1' then
                av_first <= '0';
                avs <= hc;
                v_d := signed('0' & hc) - signed('0' & x0);
                if v_d > 0 and v_up /= 255 then v_up <= v_up + 1; end if;
                if v_d < 0 and v_dn /= 255 then v_dn <= v_dn + 1; end if;
                if (v_d > 3 or v_d < -3) and v_far /= 255 then v_far <= v_far + 1; end if;
            end if;
            if i_av = '1' then
                lact <= '1';
                saw  <= '1';
                avcnt <= avcnt + 1;
            end if;

            -- beam position of the pixel entering the input latch
            if hc = x0m1 then
                xc <= (others => '0');
                p0_xz <= '1';
                p0_run <= '0';
            else
                xc <= xc + 1;
                p0_xz <= '0';
                if p0_xz = '1' or (p0_run = '1' and xc /= wm1) then p0_run <= '1'; else p0_run <= '0'; end if;
            end if;
            if p0_run = '1' and xc = wm1 then p0_end <= '1'; else p0_end <= '0'; end if;
            x0m1 <= x0 - 1;
            wm1 <= resize(wl, 12) - 1;
            -- end of the active window (our own x), once per active line
            if p0_end = '1' and lact = '1' then
                endact <= '1';
            end if;

            -- hsync fall: close the line
            if i_hs_d = '1' and i_hs = '0' then
                hs_ev <= '1';
                av_first <= '1';
                if lact = '1' then
                    w_meas <= avcnt;
                    if aline /= 2047 then aline <= aline + 1; end if;
                end if;
                lact  <= '0';
                avcnt <= (others => '0');
            end if;

            -- vsync fall after active video: field event (serration-safe)
            if i_vs_d = '1' and i_vs = '0' and saw = '1' then
                saw <= '0';
                fev <= '1';
                fc  <= fc + 1;
                fp  <= not fp;
                fbit <= not i_fn;           -- bottom field = field_n '0'
                k1 <= unsigned(registers_in(0));
                k2 <= unsigned(registers_in(1));
                k3 <= unsigned(registers_in(2));
                k4 <= unsigned(registers_in(3));
                k5 <= unsigned(registers_in(4));
                k6 <= unsigned(registers_in(5));
                sw <= registers_in(6)(4 downto 0);
                p12 <= unsigned(registers_in(7));
                tid <= registers_in(8)(3 downto 0);
                wl  <= w_meas(10 downto 0);
                if il = '1' then
                    hl <= aline(9 downto 0) & '0';
                else
                    hl <= aline;
                end if;
                aline <= (others => '0');
                -- x-origin: snap when most lines disagree, else creep by 1
                if v_far > 64 then
                    x0 <= avs;
                elsif v_up > v_dn + 8 then
                    x0 <= x0 + 1;
                elsif v_dn > v_up + 8 then
                    x0 <= x0 - 1;
                end if;
                v_up <= (others => '0'); v_dn <= (others => '0'); v_far <= (others => '0');
            end if;

            -- interlaced timings
            case tid is
                when "0000" | "1000" | "1010" | "0010" | "0001" => il <= '1';
                when others => il <= '0';
            end case;

            -- frame line of the current line
            if il = '1' then
                yf <= aline(9 downto 0) & fbit;
            else
                yf <= aline;
            end if;
        end if;
    end process p_in;

    ----------------------------------------------------------------------------
    -- generated avid: starts 3 or 4 clocks after the source's avid rise, whichever
    -- keeps the hsync->start parity at its voted usual value; ends after W pixels
    ----------------------------------------------------------------------------
    p_gav : process(clk)
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
                if g_cnt = resize(wl, 12) - 1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;
        end if;
    end process p_gav;

    p_sync : process(clk)
    begin
        if rising_edge(clk) then
            sr_hs <= data_in.hsync_n & sr_hs(0 to C_LAT - 2);
            sr_vs <= data_in.vsync_n & sr_vs(0 to C_LAT - 2);
            sr_fn <= data_in.field_n & sr_fn(0 to C_LAT - 2);
            sr_ga <= g_avid & sr_ga(0 to C_LAT - 5);
        end if;
    end process p_sync;

    ----------------------------------------------------------------------------
    -- vblank sequencer: per-field parameters, skin-centroid division
    ----------------------------------------------------------------------------
    p_vseq : process(clk)
        variable v_t   : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            prep0 <= '0';
            if fev = '1' then
                vs_st <= 1;
            else
                case vs_st is
                    when 1 =>
                        v_t := resize(wl(10 downto 5) * k3(9 downto 5), 12);
                        s_s <= resize(to_unsigned(24, 9) + resize(v_t(11 downto 4), 9), 9);
                        if wl >= 1024 then bs6 <= '1'; else bs6 <= '0'; end if;
                        bank  <= sw(0);
                        leaf3 <= k6(9 downto 7);
                        vs_st <= 2;
                    when 2 =>
                        s8 <= s_s(8 downto 3);
                        if bs6 = '1' then
                            ncols <= resize(shift_right(wl + 63, 6), 6);
                            if (fbit and il) = '1' then so <= resize(fc(2 downto 0) & "101", 6);
                            else so <= resize(fc(2 downto 0) & "100", 6); end if;
                        else
                            ncols <= resize(shift_right(wl + 31, 5), 6);
                            if (fbit and il) = '1' then so <= resize(fc(2 downto 0) & "11", 6);
                            else so <= resize(fc(2 downto 0) & "10", 6); end if;
                        end if;
                        -- GILD staging (P12)
                        if p12 < 154 then
                            pregild <= '1';
                            gcomp <= resize(p12(9 downto 5), 3);
                            gthr <= (others => '0');
                        else
                            pregild <= '0';
                            gcomp <= "100";
                            if p12 < 614 then
                                v_t := resize(p12 - 154, 12);
                                gthr <= resize(shift_right(resize(v_t, 13) + shift_left(resize(v_t, 13), 3), 4), 9);
                            else
                                gthr <= (others => '1');
                            end if;
                        end if;
                        if p12 < 614 then
                            mr <= "000";
                        elsif p12 < 870 then
                            mr <= resize(shift_right(p12 - 614, 6), 3);
                        else
                            mr <= "100";
                        end if;
                        vs_st <= 3;
                    when 3 =>
                        -- INTRICACY -> split probability + plain-leaf fraction,
                        -- ramped in by the GILD motif stage (mr 0..4)
                        case to_integer(mr) is
                            when 0 => psplit <= (others => '0');
                                      plthr <= (others => '1');
                                      jthr <= (others => '0');
                            when 1 => psplit <= resize(k4(9 downto 4), 9);
                                      plthr <= to_unsigned(230, 8) - resize(k4(9 downto 5), 8);
                                      jthr <= resize(k5(9 downto 4), 8);
                            when 2 => psplit <= resize(k4(9 downto 3), 9);
                                      plthr <= to_unsigned(215, 8) - resize(k4(9 downto 4), 8);
                                      jthr <= resize(k5(9 downto 3), 8);
                            when 3 => psplit <= resize(k4(9 downto 3), 9) + resize(k4(9 downto 4), 9);
                                      plthr <= to_unsigned(200, 8) - resize(k4(9 downto 4), 8);
                                      jthr <= resize(k5(9 downto 3), 8) + resize(k5(9 downto 4), 8);
                            when others =>
                                      psplit <= resize(k4(9 downto 2), 9);
                                      plthr <= to_unsigned(200, 8) - resize(k4(9 downto 3), 8);
                                      jthr <= resize(k5(9 downto 2), 8);
                        end case;
                        -- ORNAMENT: robe threshold (high ORNAMENT = low threshold)
                        rthr <= to_unsigned(300, 9) - resize(k2(9 downto 2), 9);
                        -- FIGURE: skin tolerance 32..159, tightened by GILD 90-100 %
                        if p12 > 921 then
                            skt <= to_unsigned(32, 10) + resize(k1(9 downto 4), 10);
                        else
                            skt <= to_unsigned(32, 10) + resize(k1(9 downto 3), 10);
                        end if;
                        ymd <= hl - resize(shift_right(hl, 3) + shift_right(hl, 4), 11);
                        vs_st <= 10;
                    when 10 =>
                        prep0 <= '1';
                        vs_st <= 0;
                    when others =>
                        null;
                end case;
            end if;
        end if;
    end process p_vseq;

    ----------------------------------------------------------------------------
    -- early per-pixel keys + coarse-map sample accumulators + centroid sums
    ----------------------------------------------------------------------------
    p_keys : process(clk)
        variable v_x  : signed(12 downto 0);
        variable v_bm : unsigned(5 downto 0);
        variable v_w  : std_logic_vector(15 downto 0);
        variable v_t  : unsigned(13 downto 0);
        variable v_emit : std_logic;
        variable v_c, v_e : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            -- P1
            p1_x <= signed('0' & xc);
            p1_av <= p0_xz or p0_run;
            p1_ls <= p0_xz;
            k1_dcb <= f_absn(signed('0' & i_u) - C_SKCB, 10);
            k1_dcr <= f_absn(signed('0' & i_v) - C_SKCR, 10);
            if i_y > 150 and i_y < 940 then k1_yok <= '1'; else k1_yok <= '0'; end if;
            k1_cb  <= resize(shift_right(signed('0' & i_u) - 512, 3), 7);
            k1_cr  <= resize(shift_right(signed('0' & i_v) - 512, 3), 7);
            if i_y < 230 then k1_dark <= '1'; else k1_dark <= '0'; end if;
            k1_ya  <= i_y(9 downto 2);
            k1_yd1 <= k1_ya;
            k1_yd2 <= k1_yd1;

            -- P2: skin key, detail, cameo distance
            if resize(k1_dcb, 11) + resize(k1_dcr, 11) < resize(skt, 11) and k1_yok = '1' then
                k2_skin <= '1'; else k2_skin <= '0'; end if;
            k2_det <= f_absn(signed('0' & k1_ya) - signed('0' & k1_yd2), 8);
            k2_ya  <= k1_ya;
            k2_x   <= p1_x;  k2_av <= p1_av;
            k2_cb  <= k1_cb; k2_cr <= k1_cr;


            -- sample line? (one rotating line per block row per field)
            hs_ev_d <= hs_ev;
            if hs_ev_d = '1' then
                if bs6 = '1' then
                    if yf(5 downto 0) = so then samp <= '1'; else samp <= '0'; end if;
                    if yf(10 downto 6) > 15 then rows <= "1111"; else rows <= yf(9 downto 6); end if;
                else
                    if yf(4 downto 0) = so(4 downto 0) then samp <= '1'; else samp <= '0'; end if;
                    if yf(10 downto 5) > 15 then rows <= "1111"; else rows <= yf(8 downto 5); end if;
                end if;
            end if;

            -- segment accumulators on the sample line (P2 domain).  A finished
            -- segment is packed from the sums as they stand (before this pixel
            -- restarts them) and written to the segment buffer at its column.
            sg_we <= '0';
            if bs6 = '1' then v_bm := unsigned(k2_x(5 downto 0)); else v_bm := '0' & unsigned(k2_x(4 downto 0)); end if;
            v_emit := '0';
            if samp = '1' and k2_av = '1' and v_bm = 0 and k2_x /= 0 then v_emit := '1'; end if;
            if samp = '1' and k2_av = '0' and k2_x = f_s13(wl) then v_emit := '1'; end if;
            if v_emit = '1' then
                -- map word: Y5 (mean luma / 32) det3 Cb4 Cr4 (mean chroma / 64)
                if bs6 = '1' then
                    v_w(15 downto 11) := std_logic_vector(a_y(13 downto 9));
                    v_t := shift_right(a_d, 9);
                    v_c := shift_right(a_cb, 9);
                    v_e := shift_right(a_cr, 9);
                else
                    v_w(15 downto 11) := std_logic_vector(a_y(12 downto 8));
                    v_t := shift_right(a_d, 8);
                    v_c := shift_right(a_cb, 8);
                    v_e := shift_right(a_cr, 8);
                end if;
                v_w(10 downto 8) := std_logic_vector(f_umin(v_t, to_unsigned(7, 14))(2 downto 0));
                if v_c > 7 then v_c := to_signed(7, 13); elsif v_c < -8 then v_c := to_signed(-8, 13); end if;
                if v_e > 7 then v_e := to_signed(7, 13); elsif v_e < -8 then v_e := to_signed(-8, 13); end if;
                v_w(7 downto 4) := std_logic_vector(v_c(3 downto 0));
                v_w(3 downto 0) := std_logic_vector(v_e(3 downto 0));
                sg_wdata <= v_w;
                sg_waddr <= resize(a_col, 8);
                sg_we    <= '1';
            end if;
            if samp = '1' and k2_av = '1' then
                if v_bm = 0 then
                    a_y  <= resize(k2_ya, 14);
                    a_d  <= resize(k2_det, 14);
                    a_cb <= resize(k2_cb, 13);
                    a_cr <= resize(k2_cr, 13);
                    if bs6 = '1' then
                        a_col <= unsigned(k2_x(11 downto 6));
                    else
                        a_col <= unsigned(k2_x(10 downto 5));
                    end if;
                else
                    a_y <= a_y + k2_ya;
                    a_d <= a_d + k2_det;
                    a_cb <= a_cb + k2_cb;
                    a_cr <= a_cr + k2_cr;
                end if;
            end if;

        end if;
    end process p_keys;

    ----------------------------------------------------------------------------
    -- hash unit: 4 stages, one input per clock (engine-owned)
    ----------------------------------------------------------------------------
    p_hash : process(clk)
    begin
        if rising_edge(clk) then
            hq1 <= f_hs0(h_a, h_b);
            hq2 <= f_hs1(hq1);
            hq3 <= f_hs2(hq2);
            hq4 <= f_hs3(hq3);
        end if;
    end process p_hash;

    ----------------------------------------------------------------------------
    -- engine (line prep: IIR pass, band step, 3 tiles; then one tile per advance)
    -- + beam walker.  One process: both own the tile queue.
    ----------------------------------------------------------------------------
    p_eng : process(clk)
        variable v_tvn  : unsigned(10 downto 0);
        variable v_col  : signed(12 downto 0);
        variable v_c5   : unsigned(4 downto 0);
        variable v_ow, v_nw, v_ww : std_logic_vector(15 downto 0);
        variable v_rec  : t_tile;
        variable v_xn   : signed(12 downto 0);
        variable v_adv  : std_logic;
        variable v_lut  : unsigned(4 downto 0);
        variable v_cb4, v_cr4 : signed(3 downto 0);
    begin
        if rising_edge(clk) then
            m_we <= '0';

            -- requests
            if endact = '1' then
                pr_req <= '1';
                if il = '1' then yt <= yf + 2; else yt <= yf + 1; end if;
            end if;
            if prep0 = '1' then
                pr_req_first <= '1';
                if (fbit and il) = '1' then yt <= to_unsigned(1, 11); else yt <= (others => '0'); end if;
            end if;

            ------------------------------------------------------------------
            -- beam walker (pixel at P0 -> P1)
            ------------------------------------------------------------------
            v_adv := '0';
            if p0_xz = '1' then
                tu <= resize(ph, 10);
            elsif p0_run = '1' then
                if tu = cur.wm1 then
                    v_adv := '1';
                    tu  <= (others => '0');
                    cur <= q0;
                    adv_req <= '1';
                else
                    tu <= tu + 1;
                end if;
            end if;

            ------------------------------------------------------------------
            -- engine
            ------------------------------------------------------------------
            case es is
                when 0 =>
                    if pr_req_first = '1' then
                        pr_req_first <= '0';
                        pr_req <= '0';
                        pr_first <= '1';
                        pr_mode <= '1';
                        es <= 10;
                    elsif pr_req = '1' then
                        pr_req <= '0';
                        adv_req <= '0';
                        pr_first <= '0';
                        pr_mode <= '1';
                        if samp = '1' then
                            samp_pend <= '1';
                            es <= 2;
                        else
                            es <= 10;
                        end if;
                    elsif adv_req = '1' then
                        adv_req <= '0';
                        pr_mode <= '0';
                        t_slot <= 1;
                        es <= 30;
                    end if;

                -- ---------------- IIR pass over the sample row -------------
                when 2 =>
                    ic <= (others => '0');
                    ipv <= (others => '0');
                    es <= 3;
                when 3 =>
                    if ic < ncols then
                        m_raddr  <= rows & ic(4 downto 0);
                        sg_raddr <= resize(ic, 8);
                        ipv(0) <= '1';
                        ipc(0) <= ic;
                        ic <= ic + 1;
                    else
                        ipv(0) <= '0';
                        if ipv = "000" then
                            samp_pend <= '0';
                            es <= 10;
                        end if;
                    end if;
                    ipv(1) <= ipv(0); ipv(2) <= ipv(1);
                    ipc(1) <= ipc(0); ipc(2) <= ipc(1);
                    if ipv(2) = '1' then
                        v_ow := m_qr;
                        v_nw := sg_qr;
                        v_ww(15 downto 11) := std_logic_vector(f_iiru(unsigned(v_ow(15 downto 11)), unsigned(v_nw(15 downto 11)), 2));
                        v_ww(10 downto 8)  := std_logic_vector(f_iiru(unsigned(v_ow(10 downto 8)), unsigned(v_nw(10 downto 8)), 1));
                        v_ww(7 downto 4)   := std_logic_vector(f_iirs(signed(v_ow(7 downto 4)), signed(v_nw(7 downto 4)), 1));
                        v_ww(3 downto 0)   := std_logic_vector(f_iirs(signed(v_ow(3 downto 0)), signed(v_nw(3 downto 0)), 1));
                        m_wdata <= v_ww;
                        m_waddr <= rows & ipc(2)(4 downto 0);
                        m_we    <= '1';
                    end if;

                -- ---------------- band step for the target line ------------
                when 10 =>
                    if pr_first = '1' then
                        nj <= (others => '0');
                        nby0 <= (others => '0');
                        ntv <= resize(yt, 10);
                    else
                        if il = '1' then v_tvn := resize(tv, 11) + 2; else v_tvn := resize(tv, 11) + 1; end if;
                        if v_tvn >= resize(hb, 11) then
                            nj <= bj + 1;
                            nby0 <= by0 + signed(resize(hb, 13));
                            ntv <= resize(v_tvn - resize(hb, 11), 10);
                        else
                            nj <= bj;
                            nby0 <= by0;
                            ntv <= resize(v_tvn, 10);
                        end if;
                    end if;
                    es <= 11;
                when 11 =>
                    h_a <= resize(nj, 16);
                    h_b <= C_SLT_BD;
                    es <= 12;
                when 12 =>
                    h_a <= resize(yt, 16);
                    h_b <= C_SLT_GR;
                    es <= 13;
                when 13 =>
                    h_a <= resize(yt(10 downto 2), 16);
                    h_b <= C_SLT_MD;
                    es <= 14;
                when 14 =>
                    es <= 15;
                when 15 =>
                    es <= 16;
                when 16 =>
                    if bank = '0' then v_lut := C_HLUT_K(to_integer(hq4(3 downto 0)));
                    else v_lut := C_HLUT_A(to_integer(hq4(3 downto 0))); end if;
                    hbi <= v_lut;
                    mul_in <= v_lut;
                    nph <= hq4(7 downto 4);
                    es <= 17;
                when 17 =>
                    gseed_n <= hq4 or x"0001";
                    nhb <= resize(mul_out, 10);
                    es <= 18;
                when 18 =>
                    mseed_n <= hq4 or x"0001";
                    if nhb < C_HMIN then nhb <= to_unsigned(C_HMIN, 10); end if;
                    es <= 19;
                when 19 =>
                    -- commit the band for the next line (current line's pixels have
                    -- all passed P2 by now)
                    bj <= nj; by0 <= nby0; tv <= ntv; hb <= nhb; ph <= nph;
                    gseed <= gseed_n; mseed <= mseed_n;
                    cyb <= nby0 + signed(resize(shift_right(nhb, 1), 13));
                    es <= 20;
                when 20 =>
                    if cyb < 0 then
                        rowb <= (others => '0');
                    elsif bs6 = '1' then
                        if cyb(12 downto 6) > 15 then rowb <= "1111"; else rowb <= unsigned(cyb(9 downto 6)); end if;
                    else
                        if cyb(12 downto 5) > 15 then rowb <= "1111"; else rowb <= unsigned(cyb(8 downto 5)); end if;
                    end if;
                    if cyb > f_s13(ymd) then lowb <= '1'; else lowb <= '0'; end if;
                    t_k <= (others => '0');
                    t_xs <= -signed(resize(ph, 13));
                    t_slot <= 0;
                    es <= 30;

                -- ---------------- one tile -------------------------------------
                when 30 =>
                    h_a <= bj(7 downto 0) & t_k;
                    h_b <= C_SLT_TA;
                    es <= 31;
                when 31 =>
                    h_b <= C_SLT_TB;
                    es <= 32;
                when 32 =>
                    h_a <= bj(8 downto 1) & "00" & t_k(7 downto 2);
                    h_b <= C_SLT_TM;
                    es <= 33;
                when 33 => es <= 34;
                when 34 => es <= 35;
                when 35 =>
                    if bank = '0' then v_lut := C_WLUT_K(to_integer(hq4(3 downto 0)));
                    else v_lut := C_WLUT_A(to_integer(hq4(3 downto 0))); end if;
                    t_m <= v_lut;
                    mul_in <= v_lut;
                    t_rnd <= hq4(11 downto 4);
                    t_fs <= hq4(14 downto 12);
                    t_flip <= hq4(15);
                    es <= 36;
                when 36 =>
                    t_wr <= mul_out;
                    t_rec.ha <= hq4(7 downto 0);
                    t_flip <= t_flip and hq4(3);
                    es <= 37;
                when 37 =>
                    if t_wr < C_WMIN then
                        t_w <= to_unsigned(C_WMIN, 10);
                    elsif t_wr > 1000 then
                        t_w <= to_unsigned(1000, 10);
                    else
                        t_w <= t_wr(9 downto 0);
                    end if;
                    t_jit <= hq4(15 downto 11);
                    es <= 38;
                when 38 =>
                    t_cx <= t_xs + signed(resize(shift_right(t_w, 1), 13));
                    t_rec.w <= t_w;
                    t_rec.wm1 <= t_w - 1;
                    if t_w > hb then t_rec.ax <= not t_flip; else t_rec.ax <= t_flip; end if;
                    es <= 39;
                when 39 =>
                    if t_cx < 0 then
                        v_c5 := (others => '0');
                    elsif bs6 = '1' then
                        if t_cx(12 downto 6) > 31 then v_c5 := "11111"; else v_c5 := unsigned(t_cx(10 downto 6)); end if;
                    else
                        if t_cx(12 downto 5) > 31 then v_c5 := "11111"; else v_c5 := unsigned(t_cx(9 downto 5)); end if;
                    end if;
                    m_raddr <= rowb & v_c5;
                    if t_rec.ax = '1' then t_side <= t_w; else t_side <= hb; end if;
                    case to_integer(t_fs) is        -- split at 1/4 1/3 1/2 2/3 3/4 (x/16)
                        when 0 => t_fk <= to_unsigned(4, 4);
                        when 1 => t_fk <= to_unsigned(5, 4);
                        when 2 | 5 => t_fk <= to_unsigned(8, 4);
                        when 3 | 7 => t_fk <= to_unsigned(11, 4);
                        when 4 => t_fk <= to_unsigned(12, 4);
                        when others => t_fk <= to_unsigned(5, 4);
                    end case;
                    es <= 40;
                when 40 =>
                    t_p <= resize(shift_right(t_side * t_fk, 4), 10);
                    es <= 41;
                when 41 =>
                    t_rm <= t_side - t_p;
                    es <= 42;
                when 42 =>
                    -- the block under the tile centre: luma, texture, colour
                    if t_rnd < psplit and t_p >= C_LMIN and t_rm >= C_LMIN then
                        t_rec.s1 <= '1';
                    else
                        t_rec.s1 <= '0';
                    end if;
                    t_rec.p  <= t_p;
                    t_rec.rm <= t_rm;
                    t_det <= unsigned(m_qr(10 downto 8));
                    t_rec.tone <= unsigned(m_qr(15 downto 11));
                    v_cb4 := signed(m_qr(7 downto 4));
                    v_cr4 := signed(m_qr(3 downto 0));
                    t_sat <= resize(f_absn(resize(v_cb4, 5), 4), 5) + resize(f_absn(resize(v_cr4, 5), 4), 5);
                    -- colour class: 1 warm (reds, oranges, skin, wood), 2 cool (blues,
                    -- violets), 3 green, 0 neutral
                    if v_cb4 < 0 and v_cr4 < 0 and (v_cb4 < -1 or v_cr4 < -1) then
                        t_rec.cls <= "11";
                    elsif resize(v_cr4, 6) - resize(v_cb4, 6) >= 2 then
                        t_rec.cls <= "01";
                    elsif resize(v_cb4, 6) - resize(v_cr4, 6) >= 2 then
                        t_rec.cls <= "10";
                    else
                        t_rec.cls <= "00";
                    end if;
                    -- skin-like block (Cameo gate and hair rule)
                    if v_cb4 < 0 and v_cb4 >= -3 and v_cr4 > 0 and v_cr4 <= 3 then
                        t_rec.skin <= '1'; else t_rec.skin <= '0'; end if;
                    es <= 43;
                when 43 =>
                    -- ornament score: texture and colourfulness of the picture
                    t_sa <= resize(t_det & "0000", 9) + resize(t_det & "000", 9) + resize(t_sat & "0000", 9);
                    es <= 44;
                when 44 =>
                    t_score <= resize(t_sa, 10) + resize(t_jit, 10);
                    -- gilding order: plain areas first, busy ones last
                    t_rank <= resize(t_rec.ha(7 downto 1), 9) + resize(t_sa(8 downto 2), 9);
                    es <= 45;
                when 45 =>
                    if sw(3) = '1' and (t_rec.cls = "11" or (lowb = '1' and t_rec.tone >= 6 and t_rec.tone <= 20)) then
                        t_rec.zone <= "10";
                    elsif t_score >= resize(rthr, 10) then
                        t_rec.zone <= "01";
                    else
                        t_rec.zone <= "00";
                    end if;
                    if t_rank < gthr then t_rec.gild <= '1'; else t_rec.gild <= '0'; end if;
                    es <= 46;
                when 46 =>
                    v_rec := t_rec;
                    if pr_mode = '1' and t_slot = 0 then
                        cur <= v_rec;
                    else
                        q0 <= v_rec;
                    end if;
                    t_xs <= t_xs + signed(resize(t_rec.w, 13));
                    t_k <= t_k + 1;
                    if pr_mode = '1' and t_slot = 0 then
                        t_slot <= t_slot + 1;
                        es <= 30;
                    else
                        pr_mode <= '0';
                        es <= 0;
                    end if;
                when others =>
                    es <= 0;
            end case;
        end if;
    end process p_eng;

    ----------------------------------------------------------------------------
    -- coarse map banks (ping-pong by field parity), segment buffer
    ----------------------------------------------------------------------------
    p_map : process(clk)
    begin
        if rising_edge(clk) then
            if m_we = '1' and fp = '0' then
                map0(to_integer(m_waddr)) <= m_wdata;
            end if;
            mq0 <= map0(to_integer(m_raddr));
        end if;
    end process p_map;
    p_map1 : process(clk)
    begin
        if rising_edge(clk) then
            if m_we = '1' and fp = '1' then
                map1(to_integer(m_waddr)) <= m_wdata;
            end if;
            mq1 <= map1(to_integer(m_raddr));
        end if;
    end process p_map1;
    p_mapq : process(clk)
    begin
        if rising_edge(clk) then
            if fp = '0' then m_qr <= mq1; else m_qr <= mq0; end if;   -- read last field's bank
            sg_qr <= sg_q;
        end if;
    end process p_mapq;
    p_seg : process(clk)
    begin
        if rising_edge(clk) then
            if sg_we = '1' then
                segm(to_integer(sg_waddr)) <= sg_wdata;
            end if;
            sg_q <= segm(to_integer(sg_raddr));
        end if;
    end process p_seg;

    ----------------------------------------------------------------------------
    -- original-video delay line (EBR): written at P0, valid (fabric reg) at P9
    ----------------------------------------------------------------------------
    p_vd : process(clk)
    begin
        if rising_edge(clk) then
            vdr(to_integer(vd_wp)) <= "00" & std_logic_vector(i_y) & std_logic_vector(i_u) & std_logic_vector(i_v);
            vd_q <= vdr(to_integer(vd_wp - 7));
            vd_wp <= vd_wp + 1;
            vd_qr <= vd_q;
        end if;
    end process p_vd;

    ----------------------------------------------------------------------------
    -- per-cell values carried P2 -> P7 through an EBR (read 3 behind write)
    ----------------------------------------------------------------------------
    p_cry : process(clk)
    begin
        if rising_edge(clk) then
            cry(to_integer(cr_wp)) <= "00000" & p2_gild & std_logic_vector(p2_cls) & std_logic_vector(p2_ya) &
                                      std_logic_vector(p2_zone) & std_logic_vector(p2_tone) & p2_skin & std_logic_vector(p2_h);
            cr_q <= cry(to_integer(cr_wp - 3));
            cr_wp <= cr_wp + 1;
            cr_qr <= cr_q;
        end if;
    end process p_cry;

    ----------------------------------------------------------------------------
    -- aura / relief line buffer: {robe, vdist(2), motif mask} per column
    ----------------------------------------------------------------------------
    p_lb : process(clk)
    begin
        if rising_edge(clk) then
            if lb_we = '1' then
                lbr(to_integer(lb_wa)) <= lb_wd;
            end if;
            lb_q <= lbr(to_integer(lbx_r));
            lb_qr <= lb_q;
        end if;
    end process p_lb;

    ----------------------------------------------------------------------------
    -- palette ROM (bank x LEAF x 64 entries)
    ----------------------------------------------------------------------------
    p_pal : process(clk)
    begin
        if rising_edge(clk) then
            pal_q <= C_GPAL(to_integer(pal_a));
            pal_qr <= pal_q;
        end if;
    end process p_pal;

    ----------------------------------------------------------------------------
    -- pixel path P2 .. P11
    ----------------------------------------------------------------------------
    p_pix : process(clk)
        variable v_c1   : std_logic;
        variable v_x0, v_y0, v_cw, v_ch, v_mn, v_mg : unsigned(9 downto 0);
        variable v_dx, v_dy : signed(11 downto 0);
        variable v_mot  : integer range 0 to 15;
        variable v_sel, v_cl : unsigned(1 downto 0);
        variable v_bright : std_logic;
        variable v_t12  : signed(11 downto 0);
        variable v_omx, v_omn, v_oct, v_a4 : unsigned(12 downto 0);
        variable v_r12, v_m12, v_q2, v_q4, v_ax12, v_ay12, v_ab12 : signed(11 downto 0);
        variable v_sec  : unsigned(3 downto 0);
        variable v_sp   : unsigned(11 downto 0);
        variable v_idx, v_base, v_gold, v_jw, v_jw2 : unsigned(5 downto 0);
        variable v_jon, v_mask, v_leaf, v_left, v_upm : std_logic;
        variable v_zone : unsigned(1 downto 0);
        variable v_tone : unsigned(4 downto 0);
        variable v_ya   : unsigned(7 downto 0);
        variable v_cls  : unsigned(1 downto 0);
        variable v_t5   : unsigned(5 downto 0);
        variable v_skin, v_gild : std_logic;
        variable v_h    : unsigned(7 downto 0);
        variable v_vy, v_vcb, v_vcr : unsigned(9 downto 0);
        variable v_s    : signed(11 downto 0);
        variable v_d10, v_gr, v_fx : signed(10 downto 0);
        variable v_tmp  : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            --------------------------------------------------------------------
            -- P2: leaf of the tile (one KD split), cell-local u,v and size
            --------------------------------------------------------------------
            v_c1 := '0';
            if cur.s1 = '1' then
                if cur.ax = '1' then
                    if tu >= cur.p then v_c1 := '1'; end if;
                else
                    if tv >= cur.p then v_c1 := '1'; end if;
                end if;
            end if;
            v_x0 := (others => '0'); v_y0 := (others => '0');
            v_cw := cur.w; v_ch := hb;
            if cur.s1 = '1' then
                if cur.ax = '1' then
                    if v_c1 = '1' then v_x0 := cur.p; v_cw := cur.rm; else v_cw := cur.p; end if;
                else
                    if v_c1 = '1' then v_y0 := cur.p; v_ch := cur.rm; else v_ch := cur.p; end if;
                end if;
            end if;
            p2_u  <= tu - v_x0;
            p2_v  <= tv - v_y0;
            p2_cw <= v_cw;
            p2_ch <= v_ch;
            p2_zone <= cur.zone; p2_tone <= cur.tone;
            p2_skin <= cur.skin; p2_gild <= cur.gild; p2_cls <= cur.cls;
            p2_ya <= k1_ya;
            if v_c1 = '1' then p2_h <= (cur.ha(4 downto 0) & cur.ha(7 downto 5)) xor x"A5"; else p2_h <= cur.ha; end if;
            p2_av <= p1_av; p2_ls <= p1_ls;

            --------------------------------------------------------------------
            -- P3: |centred offset| (half-px units), size classes, margin
            --------------------------------------------------------------------
            v_dx := signed(resize(p2_u & '1', 12)) - signed(resize(p2_cw, 12));
            v_dy := signed(resize(p2_v & '1', 12)) - signed(resize(p2_ch, 12));
            p3_ax <= f_absn(v_dx, 11);
            p3_ay <= f_absn(v_dy, 11);
            p3_sx <= v_dx(11); p3_sy <= v_dy(11);
            if resize(p2_ch & '0', 12) > resize(p2_cw, 12) + resize(p2_cw & '0', 12) then p3_tall <= '1'; else p3_tall <= '0'; end if;
            if resize(p2_cw & '0', 12) > resize(p2_ch, 12) + resize(p2_ch & '0', 12) then p3_wide <= '1'; else p3_wide <= '0'; end if;
            v_mn := f_umin(p2_cw, p2_ch);
            if v_mn < 24 then p3_small <= '1'; else p3_small <= '0'; end if;
            if v_mn > 96 then p3_large <= '1'; else p3_large <= '0'; end if;
            v_mg := shift_right(v_mn, 3);
            if v_mg < 2 then v_mg := to_unsigned(2, 10); end if;
            p3_mh <= v_mg(8 downto 0) & '0';
            p3_mn <= v_mn;
            p3_cw <= p2_cw; p3_ch <= p2_ch; p3_u <= p2_u(5 downto 0); p3_v <= p2_v;
            p3_zone <= p2_zone; p3_tone <= p2_tone; p3_h <= p2_h; p3_cls <= p2_cls;
            p3_av <= p2_av; p3_ls <= p2_ls;
            if p2_zone = "01" then p3_rb <= '1'; else p3_rb <= '0'; end if;

            --------------------------------------------------------------------
            -- P4: motif choice, inner half-extents, max/min
            --------------------------------------------------------------------
            -- What goes where comes from the picture: the block's colour class and
            -- luma band choose the motif family; the cell hash only varies it (and
            -- 1 cell in 4 borrows another family so areas never read as stamped).
            v_sel := p3_h(7 downto 6);
            if p3_h(5 downto 4) = "00" then v_cl := p3_h(3 downto 2); else v_cl := p3_cls; end if;
            v_bright := p3_tone(4);
            if p3_zone = "10" then
                v_mot := M_MEADOW;
            elsif p3_zone = "00" then
                if p3_h = 0 and p3_tone(0) = '0' then v_mot := M_FRECT; else v_mot := M_PLAIN; end if;
            elsif p3_tone <= 3 then
                v_mot := M_INK;
            elsif p3_h < plthr then
                v_mot := M_PLAIN;
            elsif bank = '0' then
                case to_integer(v_cl) is
                    when 1 =>                                   -- warm: ring-discs
                        if p3_tall = '1' and v_sel = "11" then v_mot := M_BUTTON; else v_mot := M_RING; end if;
                    when 2 =>                                   -- cool: silver tablets, bars
                        if p3_tall = '1' then
                            if v_sel(0) = '1' then v_mot := M_BARTAB; else v_mot := M_BAR; end if;
                        elsif v_sel(0) = '1' then v_mot := M_BUTTON; else v_mot := M_TABLET; end if;
                    when 3 =>                                   -- green: engraved gold
                        v_mot := M_ENGR;
                    when others =>                              -- neutral: dark bars, bright tablets
                        if v_bright = '0' then
                            if p3_wide = '1' then v_mot := M_TABLET; else v_mot := M_BAR; end if;
                        elsif v_sel(0) = '1' then v_mot := M_ENGR; else v_mot := M_TABLET; end if;
                end case;
            else
                case to_integer(v_cl) is
                    when 1 =>                                   -- warm: red squares, ovals
                        if p3_large = '1' and v_sel(0) = '1' then v_mot := M_OVAL; else v_mot := M_CONCSQ; end if;
                    when 2 =>                                   -- cool: eyes, spirals
                        if p3_small = '1' or v_sel(0) = '1' then v_mot := M_SPIRAL; else v_mot := M_EYE; end if;
                    when 3 =>                                   -- green: spirals
                        v_mot := M_SPIRAL;
                    when others =>                              -- neutral: dark bars/checkers, bright eyes/spirals
                        if v_bright = '0' then
                            if p3_tall = '1' then v_mot := M_BAR; else v_mot := M_CHECK; end if;
                        elsif v_sel(0) = '1' then v_mot := M_SPIRAL; else v_mot := M_EYE; end if;
                end case;
            end if;
            p4_mot <= to_unsigned(v_mot, 4);
            v_t12 := signed(resize(p3_cw, 12)) - signed(resize(p3_mh, 12)) - 1;
            if v_t12 < 0 then p4_hx <= (others => '0'); else p4_hx <= v_t12; end if;
            v_t12 := signed(resize(p3_ch, 12)) - signed(resize(p3_mh, 12)) - 1;
            if v_t12 < 0 then p4_hy <= (others => '0'); else p4_hy <= v_t12; end if;
            v_t12 := signed(resize(p3_mn, 12)) - signed(resize(p3_mh, 12)) - 1;
            if v_t12 < 0 then p4_r0 <= (others => '0'); else p4_r0 <= v_t12; end if;
            p4_rmax <= f_umax(p3_ax, p3_ay);
            p4_rmin <= f_umin(p3_ax, p3_ay);
            p4_ax <= p3_ax; p4_ay <= p3_ay; p4_sx <= p3_sx; p4_sy <= p3_sy;
            p4_u <= p3_u; p4_v <= p3_v; p4_large <= p3_large; p4_av <= p3_av; p4_ls <= p3_ls; p4_rb <= p3_rb;

            --------------------------------------------------------------------
            -- P5: octagonal radius, the few per-cell extents (derived thresholds
            -- are shifts, formed where they are compared)
            --------------------------------------------------------------------
            p5_r <= resize(p4_rmax, 12) + resize(shift_right(p4_rmin, 2), 12) + resize(shift_right(p4_rmin, 3), 12);
            p5_rmax <= p4_rmax;
            p5_rq <= p4_r0;
            p5_hx <= p4_hx;
            p5_hy <= p4_hy;
            if p4_mot = M_OVAL then
                p5_ew <= shift_right(p4_r0, 1) + shift_right(p4_r0, 3) + shift_right(p4_r0, 4);
            else
                p5_ew <= shift_right(p4_hx, 1) + shift_right(p4_hx, 3);
            end if;
            p5_ax <= p4_ax; p5_ay <= p4_ay;
            p5_ayb <= f_absn(signed(resize(p4_ay, 13)) - resize(shift_right(p4_hy, 1), 13), 11);
            p5_vt <= resize(p4_v, 11) + resize(shift_right(p4_v, 3), 11) + resize(shift_right(p4_v, 5), 11);
            p5_sx <= p4_sx; p5_sy <= p4_sy;
            p5_mot <= p4_mot; p5_u <= p4_u; p5_v <= p4_v(5 downto 0);
            p5_large <= p4_large; p5_av <= p4_av; p5_ls <= p4_ls; p5_rb <= p4_rb;

            --------------------------------------------------------------------
            -- P6: compares
            --------------------------------------------------------------------
            v_r12 := signed(p5_r);
            v_m12 := signed(resize(p5_rmax, 12));
            v_q2 := shift_right(p5_rq, 1);
            v_q4 := shift_right(p5_rq, 2);
            if v_r12 < p5_rq then p6_inR <= '1'; else p6_inR <= '0'; end if;
            if v_r12 < p5_rq - v_q4 then p6_in34 <= '1'; else p6_in34 <= '0'; end if;
            if v_r12 < v_q2 then p6_in2 <= '1'; else p6_in2 <= '0'; end if;
            if v_r12 < v_q4 then p6_in4 <= '1'; else p6_in4 <= '0'; end if;
            if v_r12 >= p5_rq - 3 then p6_olR <= '1'; else p6_olR <= '0'; end if;
            if v_r12 >= v_q2 - 2 and v_r12 < v_q2 then p6_ol2 <= '1'; else p6_ol2 <= '0'; end if;
            if v_m12 < p5_rq then p6_cR <= '1'; else p6_cR <= '0'; end if;
            if v_m12 < p5_rq - v_q4 then p6_c34 <= '1'; else p6_c34 <= '0'; end if;
            if v_m12 < v_q2 then p6_c2 <= '1'; else p6_c2 <= '0'; end if;
            if v_m12 < v_q4 then p6_c4 <= '1'; else p6_c4 <= '0'; end if;
            if v_m12 >= p5_rq - 3 then p6_olc <= '1'; else p6_olc <= '0'; end if;
            v_ax12 := signed(resize(p5_ax, 12));
            v_ay12 := signed(resize(p5_ay, 12));
            v_ab12 := signed(resize(p5_ayb, 12));
            if v_ax12 < p5_hx then p6_inx <= '1'; else p6_inx <= '0'; end if;
            if v_ay12 < p5_hy then p6_iny <= '1'; else p6_iny <= '0'; end if;
            if v_ax12 >= p5_hx - 3 - shift_right(p5_hx, 3) then p6_bdx <= '1'; else p6_bdx <= '0'; end if;
            if v_ay12 >= p5_hy - 3 - shift_right(p5_hx, 3) then p6_bdy <= '1'; else p6_bdy <= '0'; end if;
            -- tablets inside a bar (at +-hy/2): outer and inner rectangles
            if v_ax12 < shift_right(p5_hx, 1) + 2 and v_ab12 < shift_right(p5_hy, 2) + 2 then
                p6_tout <= '1'; else p6_tout <= '0'; end if;
            if v_ax12 < shift_right(p5_hx, 1) - 2 and v_ab12 < shift_right(p5_hy, 2) - 1 then
                p6_tin <= '1'; else p6_tin <= '0'; end if;
            -- button dots: Chebyshev squares at +-hy/2
            if f_umax(p5_ax, p5_ayb) < unsigned(resize(shift_right(p5_hx, 1), 11)) then
                p6_dot <= '1'; else p6_dot <= '0'; end if;
            if resize(p5_ax, 11) <= p5_vt then p6_tri <= '1'; else p6_tri <= '0'; end if;
            -- eye lens terms: diamond (pointy) + rounded octagon, aspect 4:1
            v_a4 := resize(p5_ay & "00", 13);
            p6_dl <= resize(p5_ax, 13) + v_a4;
            if resize(p5_ax, 13) > v_a4 then v_omx := resize(p5_ax, 13); v_omn := v_a4;
            else v_omx := v_a4; v_omn := resize(p5_ax, 13); end if;
            p6_omx <= v_omx; p6_omn <= v_omn;
            p6_ew <= p5_ew;
            if v_r12 < shift_right(p5_ew, 2) + shift_right(p5_ew, 3) then p6_iris <= '1'; else p6_iris <= '0'; end if;
            if v_r12 < shift_right(p5_ew, 3) then p6_pupil <= '1'; else p6_pupil <= '0'; end if;
            -- 16-sector angle: quadrant from signs, sub-sector from ratio compares
            if p5_sx = '0' and p5_sy = '0' then p6_quad <= "00";
            elsif p5_sx = '1' and p5_sy = '0' then p6_quad <= "01";
            elsif p5_sx = '1' and p5_sy = '1' then p6_quad <= "10";
            else p6_quad <= "11"; end if;
            if p5_ay < shift_right(p5_ax, 3) + shift_right(p5_ax, 4) then p6_sub <= "000";
            elsif p5_ay < shift_right(p5_ax, 1) + shift_right(p5_ax, 3) + shift_right(p5_ax, 5) then p6_sub <= "001";
            elsif p5_ax > shift_right(p5_ay, 1) + shift_right(p5_ay, 3) + shift_right(p5_ay, 5) then p6_sub <= "010";
            elsif p5_ax > shift_right(p5_ay, 3) + shift_right(p5_ay, 4) then p6_sub <= "011";
            else p6_sub <= "100"; end if;
            p6_r <= p5_r;
            if p5_r(3 downto 1) = "000" then p6_eng <= '1'; else p6_eng <= '0'; end if;
            if p5_ax < 3 then p6_slit <= '1'; else p6_slit <= '0'; end if;
            p6_mot <= p5_mot; p6_u <= p5_u; p6_v <= p5_v;
            p6_large <= p5_large; p6_av <= p5_av; p6_ls <= p5_ls;
            -- aura: within 3 px / 3 lines after a robe<->field change (each scan
            -- direction marks one side, giving a thin line round the robe mass)
            if p5_ls = '1' then
                hd_cnt <= "11";
            elsif p5_rb /= hd_prev then
                hd_cnt <= "00";
            elsif hd_cnt /= "11" then
                hd_cnt <= hd_cnt + 1;
            end if;
            hd_prev <= p5_rb;
            if lb_qr(3) /= p5_rb then
                lb_vd <= "00";
            elsif unsigned(lb_qr(2 downto 1)) /= 3 then
                lb_vd <= unsigned(lb_qr(2 downto 1)) + 1;
            else
                lb_vd <= "11";
            end if;
            p6_upm <= lb_qr(0);
            if p5_av = '1' and (hd_cnt /= "11" or lb_qr(3) /= p5_rb or unsigned(lb_qr(2 downto 1)) < 2) then
                p6_aura <= '1';
            else
                p6_aura <= '0';
            end if;

            --------------------------------------------------------------------
            -- P7: lens / spiral decisions, flag vector
            --------------------------------------------------------------------
            v_oct := p6_omx + shift_right(p6_omn, 2) + shift_right(p6_omn, 3);
            if signed('0' & p6_dl) <= resize(p6_ew, 14) or
               signed('0' & v_oct) <= resize(p6_ew - shift_right(p6_ew, 3), 14) then
                p7_lens <= '1'; else p7_lens <= '0'; end if;
            if signed('0' & p6_dl) <= resize(p6_ew - 4, 14) or
               signed('0' & v_oct) <= resize(p6_ew - shift_right(p6_ew, 3) - 4, 14) then
                p7_lensi <= '1'; else p7_lensi <= '0'; end if;
            p7_pupil <= p6_pupil;
            p7_dot <= p6_dot;
            -- spiral: (r - sector * pitch/16) mod pitch < line; pitch 16 or 32 half-px
            case p6_quad is
                when "00" => v_sec := resize(p6_sub, 4);
                when "01" => v_sec := to_unsigned(8, 4) - resize(p6_sub, 4);
                when "10" => v_sec := to_unsigned(8, 4) + resize(p6_sub, 4);
                when others => v_sec := to_unsigned(0, 4) - resize(p6_sub, 4);
            end case;
            if p6_large = '1' then
                v_sp := p6_r - shift_left(resize(v_sec, 12), 1);
                if v_sp(4 downto 2) = "000" then p7_spl <= '1'; else p7_spl <= '0'; end if;
            else
                v_sp := p6_r - resize(v_sec, 12);
                if v_sp(3 downto 2) = "00" then p7_spl <= '1'; else p7_spl <= '0'; end if;
            end if;
            p7_f <= p6_inR & p6_in34 & p6_in2 & p6_in4 & p6_olR & p6_ol2 & p6_cR & p6_c34 & p6_c2 & p6_c4 &
                    p6_olc & p6_inx & p6_iny & p6_bdx & p6_bdy & p6_tin & p6_tout & p6_tri & p6_eng & p6_iris;
            p7_slit <= p6_slit;
            p7_mot <= p6_mot; p7_u <= p6_u; p7_v <= p6_v;
            p7_av <= p6_av; p7_ls <= p6_ls; p7_aura <= p6_aura; p7_upm <= p6_upm;
            lb_vd_d1 <= lb_vd;
            -- meadow stream: describes the pixel entering P7
            if p6_ls = '1' then
                ms <= mseed; mph <= "00";
            else
                mph <= mph + 1;
                if mph = "11" then ms <= f_xs(ms); end if;
            end if;

            --------------------------------------------------------------------
            -- P8: motif -> palette index -> palette address; relief; line buffer
            --------------------------------------------------------------------
            v_zone := unsigned(cr_qr(15 downto 14));
            v_tone := unsigned(cr_qr(13 downto 9));
            v_skin := cr_qr(8);
            v_gild := cr_qr(26);
            v_h    := unsigned(cr_qr(7 downto 0));
            v_ya   := unsigned(cr_qr(23 downto 16));
            v_cls  := unsigned(cr_qr(25 downto 24));
            -- gold paint: the picture's own luma through v0.10's gold/brown ramp,
            -- per pixel (FIELD Leaf: 3 steps brighter).  Robe leaf: the block's
            -- luma 4 steps up (a brighter figure gold), motif fills 8 up.
            if v_zone = "00" then
                if sw(2) = '1' then
                    v_t5 := resize(v_ya(7 downto 3), 6) + 3;
                else
                    v_t5 := resize(v_ya(7 downto 3), 6);
                end if;
            else
                v_t5 := resize(v_tone, 6) + 4;
            end if;
            if v_t5 > 31 then v_base := "011111"; else v_base := v_t5; end if;
            v_t5 := resize(v_tone, 6) + 8;
            if v_t5 > 31 then v_gold := "011111"; else v_gold := v_t5; end if;
            -- jewel colours follow the block's colour class (warm reds, cool
            -- blues/violets, greens); neutral blocks take any of the eight
            v_jon := '0';
            if (v_h(4 downto 0) & v_h(7 downto 5)) < jthr then v_jon := '1'; end if;
            v_jw := f_jewel(v_cls, v_h(2 downto 0));
            v_jw2 := f_jewel(v_cls, v_h(2 downto 0) + 3);
            v_idx := v_base; v_mask := '0'; v_leaf := '1';
            case to_integer(p7_mot) is
                when M_BAR =>
                    if p7_f(8) = '1' and p7_f(7) = '1' then
                        v_mask := '1'; v_leaf := '0';
                        if bank = '0' then v_idx := to_unsigned(33, 6);
                        elsif v_h(1) = '1' then v_idx := to_unsigned(60, 6);
                        else v_idx := to_unsigned(32, 6); end if;
                    end if;
                when M_BARTAB =>
                    if p7_f(4) = '1' then
                        v_mask := '1'; v_leaf := '0';
                        v_idx := to_unsigned(34 + 1, 6);
                    elsif p7_f(3) = '1' then
                        v_mask := '1'; v_leaf := '0'; v_idx := to_unsigned(32, 6);
                    elsif p7_f(8) = '1' and p7_f(7) = '1' then
                        v_mask := '1'; v_leaf := '0'; v_idx := to_unsigned(33, 6);
                    end if;
                when M_TABLET | M_BUTTON =>
                    if p7_f(8) = '1' and p7_f(7) = '1' then
                        v_mask := '1'; v_leaf := '0';
                        if p7_f(6) = '1' or p7_f(5) = '1' then
                            v_idx := to_unsigned(32, 6);
                        elsif p7_mot = M_BUTTON and p7_dot = '1' then
                            v_idx := to_unsigned(32, 6);
                        elsif v_h(2) = '1' then
                            v_idx := to_unsigned(35, 6);
                        else
                            v_idx := to_unsigned(36, 6);
                        end if;
                    end if;
                when M_RING =>
                    if p7_f(19) = '1' then
                        v_mask := '1'; v_leaf := '0';
                        if p7_f(15) = '1' or p7_f(14) = '1' then
                            v_idx := to_unsigned(32, 6);
                        elsif p7_f(16) = '1' then
                            if v_jon = '1' then v_idx := v_jw; else v_idx := to_unsigned(39, 6); end if;
                        elsif p7_f(17) = '1' then
                            v_idx := v_gold;
                        elsif p7_f(18) = '1' then
                            if v_jon = '1' then v_idx := v_jw2; else v_idx := to_unsigned(34, 6); end if;
                        else
                            v_idx := v_gold;
                        end if;
                    end if;
                when M_ENGR =>
                    if p7_f(19) = '1' then
                        v_mask := '1';
                        if p7_f(1) = '1' or p7_f(15) = '1' then v_idx := to_unsigned(38, 6); v_leaf := '0';
                        else v_idx := v_gold; end if;
                    end if;
                when M_SPIRAL =>
                    if p7_f(19) = '1' then
                        v_mask := '1';
                        if p7_spl = '1' or p7_f(15) = '1' then
                            v_leaf := '0';
                            if v_h(3) = '1' then v_idx := to_unsigned(32, 6); else v_idx := to_unsigned(38, 6); end if;
                        else
                            v_idx := v_gold;
                        end if;
                    end if;
                when M_CONCSQ =>
                    if p7_f(13) = '1' then
                        v_mask := '1'; v_leaf := '0';
                        if p7_f(9) = '1' then
                            v_idx := to_unsigned(32, 6);
                        elsif p7_f(10) = '1' then
                            if v_jon = '1' then v_idx := v_jw; else v_idx := to_unsigned(42, 6); end if;
                        elsif p7_f(11) = '1' then
                            v_idx := to_unsigned(32, 6);
                        elsif p7_f(12) = '1' then
                            v_idx := v_gold;
                        else
                            v_idx := to_unsigned(41, 6);
                        end if;
                    end if;
                when M_CHECK =>
                    if p7_f(8) = '1' and p7_f(7) = '1' then
                        v_mask := '1'; v_leaf := '0';
                        if p7_f(6) = '1' or p7_f(5) = '1' then
                            v_idx := to_unsigned(32, 6);
                        elsif v_h(0) = '1' then
                            if (p7_u(4) xor p7_v(4)) = '1' then v_idx := to_unsigned(36, 6); else v_idx := to_unsigned(32, 6); end if;
                        else
                            if (p7_u(3) xor p7_v(3)) = '1' then v_idx := to_unsigned(36, 6); else v_idx := to_unsigned(32, 6); end if;
                        end if;
                        if (p7_u(5 downto 4) + p7_v(5 downto 4) + v_h(1 downto 0)) = "00" and p7_f(6) = '0' and p7_f(5) = '0' then
                            v_idx := v_gold;
                        end if;
                    end if;
                when M_EYE =>
                    if p7_lens = '1' then
                        v_mask := '1'; v_leaf := '0';
                        if p7_lensi = '0' or p7_pupil = '1' then
                            v_idx := to_unsigned(32, 6);
                        elsif p7_f(0) = '1' then
                            if v_jon = '1' then v_idx := v_jw; else v_idx := to_unsigned(60, 6); end if;
                        else
                            v_idx := to_unsigned(36, 6);
                        end if;
                    elsif bank = '1' and p7_f(2) = '1' and p7_f(7) = '1' then
                        v_mask := '1';
                        v_idx := to_unsigned(39, 6);       -- pale gold triangle behind the eye
                    end if;
                when M_OVAL =>
                    v_leaf := '0';
                    if p7_f(19) = '1' then
                        v_mask := '1';
                        if p7_lens = '1' then
                            if p7_slit = '1' or p7_lensi = '0' then v_idx := to_unsigned(32, 6);
                            else v_idx := to_unsigned(36, 6); end if;
                        elsif p7_f(1) = '1' or p7_f(15) = '1' then
                            v_idx := to_unsigned(38, 6);
                        else
                            v_idx := v_gold;
                        end if;
                    else
                        v_idx := to_unsigned(41, 6);       -- red-orange enamel ground
                    end if;
                when M_INK =>
                    if p7_f(8) = '1' and p7_f(7) = '1' then
                        v_mask := '1'; v_leaf := '0'; v_idx := to_unsigned(32, 6);
                    end if;
                when M_FRECT =>
                    if p7_f(10) = '1' then
                        v_leaf := '0'; v_idx := to_unsigned(34, 6);
                    end if;
                when M_MEADOW =>
                    v_leaf := '0';
                    if ms(6 downto 5) /= "00" and (mph - ("0" & ms(2))) < 2 and
                       (yf(1 downto 0) - ("0" & ms(3))) < 2 then
                        case to_integer(ms(9 downto 7)) is
                            when 0 | 1 => v_idx := to_unsigned(46, 6);
                            when 2 => v_idx := to_unsigned(47, 6);
                            when 3 => v_idx := to_unsigned(61, 6);
                            when 4 => v_idx := to_unsigned(62, 6);
                            when 5 => v_idx := to_unsigned(63, 6);
                            when 6 => v_idx := to_unsigned(48, 6);
                            when others => v_idx := to_unsigned(51, 6);
                        end case;
                    else
                        v_idx := "1110" & v_tone(4 downto 3);
                    end if;
                when others =>
                    null;
            end case;
            -- hair: dark pixels in cells that hold skin read as warm near-black
            if dk_sr(7) = '1' and v_skin = '1' and v_zone /= "10" then
                v_idx := to_unsigned(32, 6); v_leaf := '0'; v_mask := '0';
            end if;
            if p7_aura = '1' and v_mask = '0' then
                v_idx := to_unsigned(37, 6); v_leaf := '0';
            end if;
            pal_a <= bank & leaf3 & v_idx;
            -- relief: inner highlight top/left, drop shadow bottom/right outside
            v_left := lmask;
            v_upm := p7_upm;
            lmask <= v_mask;
            p8_hl <= '0'; p8_sh <= '0'; p8_glint <= '0';
            if sw(4) = '1' and v_zone = "01" then
                if v_mask = '1' and (v_left = '0' or v_upm = '0') then
                    p8_hl <= '1';
                    if (v_h(5 downto 0) + fc(9 downto 4)) = 0 then p8_glint <= '1'; end if;
                end if;
                if v_mask = '0' and (v_left = '1' or v_upm = '1') then p8_sh <= '1'; end if;
            end if;
            p8_leaf <= v_leaf;
            if v_zone = "00" then p8_field <= '1'; else p8_field <= '0'; end if;
            if v_zone = "01" then p8_robe <= '1'; else p8_robe <= '0'; end if;
            p8_tone <= v_tone; p8_gild <= v_gild; p8_av <= p7_av; p8_ls <= p7_ls;
            -- line buffer write (column counter restarted at x = 0)
            if p7_ls = '1' then lbx_w <= (others => '0'); else lbx_w <= lbx_w + 1; end if;
            if p7_ls = '1' then lb_wa <= (others => '0'); else lb_wa <= lbx_w + 1; end if;
            lb_we <= p7_av;
            if v_zone = "01" then lb_wd(3) <= '1'; else lb_wd(3) <= '0'; end if;
            lb_wd(2 downto 1) <= std_logic_vector(lb_vd_d1);
            lb_wd(0) <= v_mask;
            -- line-buffer read address, aligned with P3
            if p2_ls = '1' then lbx_r <= (others => '0'); else lbx_r <= lbx_r + 1; end if;

            --------------------------------------------------------------------
            -- P9: carries; grain stream (reseeded per line)
            --------------------------------------------------------------------
            p9_leaf <= p8_leaf; p9_field <= p8_field; p9_robe <= p8_robe; p9_gild <= p8_gild;
            p9_hl <= p8_hl; p9_sh <= p8_sh; p9_glint <= p8_glint; p9_tone <= p8_tone;
            if p8_ls = '1' then
                gs <= gseed; gpar <= '0';
            else
                gs <= f_xs(gs); gpar <= not gpar;
            end if;

            --------------------------------------------------------------------
            -- P10: luma offset (life + grain + flecks + relief); the alternative
            -- picture (porcelain flesh or pre-gild video) and the select
            --------------------------------------------------------------------
            v_vy := unsigned(vd_qr(29 downto 20));
            v_vcb := unsigned(vd_qr(19 downto 10));
            v_vcr := unsigned(vd_qr(9 downto 0));
            -- life: gold paint keeps the picture's full light and shade (the ramp
            -- step plus the luma within it); robe leaf a quarter of the block contrast
            if p9_field = '1' then
                v_d10 := signed(resize(v_vy(4 downto 0), 11)) - 16;
            else
                v_d10 := shift_right(signed(resize(v_vy(9 downto 2), 11)) - signed(resize(p9_tone & "100", 11)), 2);
            end if;
            if p9_leaf = '0' then v_d10 := (others => '0'); end if;
            -- grain: gold paint +-32 (Nebula) / +-24 (Leaf), robe leaf +-64; a
            -- whisper on enamel
            v_gr := signed(resize(gs(6 downto 0), 11)) - 64;
            if p9_field = '1' then v_gr := shift_right(v_gr, 1); end if;
            if sw(2) = '1' then v_gr := v_gr - shift_right(v_gr, 2); end if;
            if p9_leaf = '0' then v_gr := shift_right(v_gr, 3); end if;
            -- flecks (2x1 px): ~6 % of field leaf, a quarter of that on robe leaf
            if gpar = '0' then
                if p9_field = '1' and gs(15 downto 10) < 4 then fk_hold <= '1';
                elsif p9_robe = '1' and gs(15 downto 10) = 0 then fk_hold <= '1';
                else fk_hold <= '0'; end if;
            end if;
            v_fx := (others => '0');
            if fk_hold = '1' and p9_leaf = '1' then
                if sw(2) = '1' then v_fx := to_signed(100, 11); else v_fx := to_signed(120, 11); end if;
            end if;
            if p9_hl = '1' then v_fx := v_fx + 56; end if;
            if p9_glint = '1' then v_fx := v_fx + 140; end if;
            if p9_sh = '1' then v_fx := v_fx - 72; end if;
            p10_s <= v_d10 + v_gr + v_fx;
            if pregild = '1' or p9_gild = '0' then
                -- pre-gild video: luma compressed toward the Klimt mid-tones
                v_s := signed(resize(v_vy, 12)) - 480;
                case to_integer(gcomp) is
                    when 0 => v_tmp := resize(v_vy, 12);
                    when 1 => v_tmp := unsigned(signed(resize(v_vy, 12)) - shift_right(v_s, 3));
                    when 2 => v_tmp := unsigned(signed(resize(v_vy, 12)) - shift_right(v_s, 2));
                    when 3 => v_tmp := unsigned(signed(resize(v_vy, 12)) - shift_right(v_s, 2) - shift_right(v_s, 3));
                    when others => v_tmp := unsigned(signed(resize(v_vy, 12)) - shift_right(v_s, 1));
                end case;
                p10_ay <= v_tmp(9 downto 0);
                p10_acb <= v_vcb; p10_acr <= v_vcr;
                p10_alt <= '1';
            elsif fl_sr(9) = '1' then
                -- porcelain flesh: contrast -25 % round the Klimt flesh point, chroma
                -- halfway to it, strong reds (lips, cheeks) only a quarter of the way
                v_s := signed(resize(v_vy, 12)) - C_FL_Y;
                p10_ay <= f_clamp10(signed(resize(v_vy, 12)) - shift_right(v_s, 2) + 12);
                p10_acb <= resize(shift_right(resize(v_vcb, 11) + C_FL_CB, 1), 10);
                if v_vcr > 600 then
                    p10_acr <= f_clamp10(signed(resize(v_vcr, 12)) - shift_right(signed(resize(v_vcr, 12)) - C_FL_CR, 2));
                else
                    p10_acr <= resize(shift_right(resize(v_vcr, 11) + C_FL_CR, 1), 10);
                end if;
                p10_alt <= '1';
            else
                p10_alt <= '0';
            end if;

            --------------------------------------------------------------------
            -- P11: compose + blanking gate on the generated avid
            --------------------------------------------------------------------
            if sr_ga(C_LAT - 6) = '0' then
                o_y <= to_unsigned(64, 10); o_cb <= to_unsigned(512, 10); o_cr <= to_unsigned(512, 10);
            elsif p10_alt = '1' then
                o_y <= p10_ay; o_cb <= p10_acb; o_cr <= p10_acr;
            else
                v_s := signed(resize(unsigned(pal_qr(29 downto 20)), 12)) + resize(p10_s, 12);
                if v_s < 64 then o_y <= to_unsigned(64, 10);
                elsif v_s > 800 then o_y <= to_unsigned(800, 10);
                else o_y <= unsigned(v_s(9 downto 0)); end if;
                o_cb <= unsigned(pal_qr(19 downto 10));
                o_cr <= unsigned(pal_qr(9 downto 0));
            end if;

            --------------------------------------------------------------------
            -- carried flags: flesh (skin & cameo gate) P4 -> P9, dark P2 -> P7
            --------------------------------------------------------------------
            if k2_skin = '1' and (sw(1) = '0' or p2_skin = '1') then
                fl_sr(3) <= '1';
            else
                fl_sr(3) <= '0';
            end if;
            fl_sr(4 to 9) <= fl_sr(3 to 8);
            dk_sr(2) <= k1_dark;
            dk_sr(3 to 7) <= dk_sr(2 to 6);
        end if;
    end process p_pix;

    mul_out <= resize(s8 * mul_in, 11);

    data_out.y       <= std_logic_vector(o_y);
    data_out.u       <= std_logic_vector(o_cr);   -- swap back to the hardware's order
    data_out.v       <= std_logic_vector(o_cb);
    data_out.avid    <= sr_ga(C_LAT - 5);
    data_out.hsync_n <= sr_hs(C_LAT - 1);
    data_out.vsync_n <= sr_vs(C_LAT - 1);
    data_out.field_n <= sr_fn(C_LAT - 1);

end architecture gilt;
