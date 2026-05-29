-- hypercube.vhd  (v2)
--
-- 4D tesseract wireframe carving the screen into 8 Voronoi cells, each
-- applying its own chroma rotation to the live video pixel.
--
-- v2 changes vs v1:
--   * Vertex compute is a state machine sharing ONE pipelined multiplier
--     instance (multiplier_s, 10-cycle latency, 1/cycle throughput) for
--     all rotation, projection, and edge-slope math. v1's unrolled
--     combinational rotation generated a multi-thousand-LC mass that
--     could not synthesise in reasonable time.
--   * Per-pixel pipeline is split into many short stages so no single
--     cycle has more than ~3 LUT levels of combinational depth. Cell
--     distance, min tree, edge raster, chroma rotation, cell-tint
--     interpolation, and brightness are all pipelined.
--   * Edge slope dx_dy uses a real reciprocal LUT + multiply, replacing
--     v1's coarse octave-bucket approximation.
--
-- 4D math:
--   16 vertices at every (+/-1, +/-1, +/-1, +/-1).
--   32 edges (vertex pairs differing in exactly one coordinate).
--   8 cell directions ({+/-X, +/-Y, +/-Z, +/-W} unit normals) -> 2D
--   Voronoi seed for cell-id classification.
--   6 rotation planes: XY XZ YZ XW YW ZW. Angles accumulate each frame
--   from base speed (knob 1) with the W-planes scaled by knob 2 (4D twist).
--   W -> Schlegel projection: scale = 1 / (W_PERSP - w). Inner cube
--   (w=+1) shrinks; outer cube (w=-1) expands.
--
-- Per frame (during vblank), driven by a state machine over MUL_LATENCY=10:
--   PHASE_ROT   : 6 planes x 24 elements (16 verts + 8 cells) x 4 muls
--                 = 576 mul issues, each one tagged with a destination
--                 descriptor that is drained 10 cycles later and combined
--                 (sub for axis_a, add for axis_b) before write-back to
--                 the working register file.
--   PHASE_PROJ  : 24 elements x 2 muls (scale*x, scale*y) = 48 issues.
--   PHASE_DXDY  : 32 edges x 1 mul (dx * recip(dy)) = 32 issues.
--   Total ~672 mul issues + ~30 cycles of drain. HD vblank is ~99k cycles.
--
-- Per scanline (during hblank): incremental adder per edge advances
--   current_x by dx_dy. Active flag set on entry to y_min, cleared on
--   exit at y_max. No multiply per scanline.
--
-- Per pixel pipeline (LATENCY=17 cycles):
--   P0 : capture px, py and forward video.
--   P1 : 8x cell sub (px-cx, py-cy), 32x edge sub (px-edge_x).
--   P2 : 8x abs of dx, dy; 32x abs of edge diff.
--   P3 : 8x add to make Manhattan distance; 32x compare to thickness.
--   P4 : pairwise min on 8 cell distances (4 winners); OR-reduce edge
--        hits in tiers (16-way at this stage).
--   P5 : pairwise min (2 winners); OR-reduce edge to 8-way.
--   P6 : final min -> cell_id (3b); final OR-reduce -> edge_hit (1b).
--   P7 : look up cell variant (swap UV / negate U / negate V) from
--        cell_id; produce u_xform, v_xform from delayed pixel chroma.
--   P8-P11 : interpolator_u (4 stages) crossfading original vs
--        cell-rotated chroma using cell_tint knob.
--   P12: composite: select between bg (black/video), cell-tinted, and
--        edge color (white or cell-bumped luma) based on toggles and
--        edge_hit.
--   P13-P16 : interpolator_u (4 stages) for master brightness on Y.
--             U,V are simply delayed 4 cycles to align with Y.
--   P17: output.
--
-- Sync delay: 17 clocks.
--
-- Resource note: targets iCE40 HX4K, 7680 LC / 32 BRAM. No DSP. The
-- shared vertex multiplier is one multiplier_s instance (~110 LC at
-- W=14). Per-pixel banks of comparators dominate LC: 8-cell distance
-- bank (~400 LC), 32-edge sub/abs/compare (~1200 LC), 32-way OR tree
-- (~30 LC). 3 interpolators (~600 LC). State machine + LUT/RAM ~400 LC.
-- Estimated: ~3500 LC and 1-2 BRAMs (sin LUT inferred).
--
-- Register map:
--   registers_in(0) = Speed       (10b, base rotation rate)
--   registers_in(1) = 4D Twist    (10b, scales XW/YW/ZW vs XY/XZ/YZ)
--   registers_in(2) = Inner Depth (10b, signed offset on initial w)
--   registers_in(3) = Cell Tint   (10b, dry/wet between input and rotated)
--   registers_in(4) = Edge Width  (10b, top 3 bits select 1..16 px)
--   registers_in(5) = Edge Bright (10b, edge luma intensity)
--   registers_in(6) = Switches    (b0=Wire b1=BG b2=CellFill b3=EdgeColor
--                                  b4=Direction)
--   registers_in(7) = Brightness  (10b, master output gain)

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.all;
use work.core_pkg.all;
use work.video_stream_pkg.all;
use work.video_timing_pkg.all;

architecture hypercube of program_top is

    --------------------------------------------------------------------------
    -- Constants
    --------------------------------------------------------------------------

    constant LATENCY     : natural := 10;

    constant FX_W        : natural := 14;     -- 14-bit signed Q3.10 fixed-point
    constant FX_FRAC     : natural := 10;     -- matches sin/cos LUT scale (1023 ~ 1.0)
    constant MUL_LATENCY : natural := (FX_W + 1) / 2 + 3;  -- = 10 for FX_W=14
    constant FX_ONE_INT  : integer := 2**FX_FRAC;

    -- Element layout in working set: 0..15 = 16 vertices, 16..23 = 8 cells.
    constant N_VERTS  : natural := 16;
    constant N_CELLS  : natural := 8;
    constant N_TOTAL  : natural := N_VERTS + N_CELLS;
    constant N_EDGES  : natural := 32;

    -- Phase counters within ROT.
    constant ROT_PER_PLANE : natural := N_TOTAL * 4;        -- 96
    constant ROT_TOTAL     : natural := 6 * ROT_PER_PLANE;  -- 576
    constant PROJ_TOTAL    : natural := N_TOTAL * 2;        -- 48
    constant DXDY_TOTAL    : natural := N_EDGES;            -- 32

    --------------------------------------------------------------------------
    -- Types
    --------------------------------------------------------------------------

    subtype t_fx is signed(FX_W-1 downto 0);
    type    t_v4 is array(0 to 3) of t_fx;
    type    t_v4_arr is array(natural range <>) of t_v4;

    -- Screen-space coordinates as signed 13b (covers 1920x1080 with margin).
    subtype t_px is signed(12 downto 0);
    type    t_px_arr is array(natural range <>) of t_px;

    -- Edge slope as signed Q4.10 in 16 bits.
    subtype t_slope is signed(15 downto 0);
    type    t_slope_arr is array(natural range <>) of t_slope;

    --------------------------------------------------------------------------
    -- Initial constants for vertices, cell directions, and edges.
    --------------------------------------------------------------------------

    function fx_one return t_fx is
    begin
        return to_signed(FX_ONE_INT, FX_W);
    end function;

    function fx_neg_one return t_fx is
    begin
        return to_signed(-FX_ONE_INT, FX_W);
    end function;

    function init_vertices return t_v4_arr is
        variable r : t_v4_arr(0 to 15);
        variable v : t_v4;
    begin
        for i in 0 to 15 loop
            for k in 0 to 3 loop
                if ((i / (2**k)) mod 2) = 0 then
                    v(k) := fx_neg_one;
                else
                    v(k) := fx_one;
                end if;
            end loop;
            r(i) := v;
        end loop;
        return r;
    end function;

    function init_cells return t_v4_arr is
        variable r : t_v4_arr(0 to 7);
        variable axis : integer;
    begin
        for i in 0 to 7 loop
            axis := i / 2;
            for k in 0 to 3 loop
                r(i)(k) := (others => '0');
            end loop;
            if (i mod 2) = 0 then
                r(i)(axis) := fx_one;
            else
                r(i)(axis) := fx_neg_one;
            end if;
        end loop;
        return r;
    end function;

    constant C_INIT_VERTS : t_v4_arr(0 to 15) := init_vertices;
    constant C_INIT_CELLS : t_v4_arr(0 to 7)  := init_cells;

    -- 32 edges: pairs (a, b) of vertex indices differing in exactly one bit.
    type t_edge is record
        a : integer range 0 to 15;
        b : integer range 0 to 15;
    end record;
    type t_edge_arr is array(natural range <>) of t_edge;

    function init_edges return t_edge_arr is
        variable r   : t_edge_arr(0 to 31);
        variable n   : integer := 0;
        variable d   : unsigned(3 downto 0);
        variable cnt : integer;
    begin
        for i in 0 to 15 loop
            for j in i+1 to 15 loop
                d := to_unsigned(i, 4) xor to_unsigned(j, 4);
                cnt := 0;
                for k in 0 to 3 loop
                    if d(k) = '1' then
                        cnt := cnt + 1;
                    end if;
                end loop;
                if cnt = 1 then
                    r(n).a := i;
                    r(n).b := j;
                    n := n + 1;
                end if;
            end loop;
        end loop;
        return r;
    end function;

    constant C_EDGES : t_edge_arr(0 to 31) := init_edges;
    -- Phase table: 64 rotation phases, 16 verts and 8 cells.
    -- Index = phase * N + element. Generated offline; sized for HD.
    constant T_PHASES : natural := 64;
    type t_vert_sx is array(0 to 1024-1) of signed(12 downto 0);
    function gen_vert_sx return t_vert_sx is
        variable r : t_vert_sx;
    begin
        r(   0) := to_signed( -120, 13);
        r(   1) := to_signed(  120, 13);
        r(   2) := to_signed( -120, 13);
        r(   3) := to_signed(  120, 13);
        r(   4) := to_signed( -120, 13);
        r(   5) := to_signed(  120, 13);
        r(   6) := to_signed( -120, 13);
        r(   7) := to_signed(  120, 13);
        r(   8) := to_signed( -200, 13);
        r(   9) := to_signed(  200, 13);
        r(  10) := to_signed( -200, 13);
        r(  11) := to_signed(  200, 13);
        r(  12) := to_signed( -200, 13);
        r(  13) := to_signed(  200, 13);
        r(  14) := to_signed( -200, 13);
        r(  15) := to_signed(  200, 13);
        r(  16) := to_signed(  -74, 13);
        r(  17) := to_signed(  158, 13);
        r(  18) := to_signed(  -99, 13);
        r(  19) := to_signed(  139, 13);
        r(  20) := to_signed( -101, 13);
        r(  21) := to_signed(  136, 13);
        r(  22) := to_signed( -127, 13);
        r(  23) := to_signed(  116, 13);
        r(  24) := to_signed( -164, 13);
        r(  25) := to_signed(  210, 13);
        r(  26) := to_signed( -208, 13);
        r(  27) := to_signed(  180, 13);
        r(  28) := to_signed( -213, 13);
        r(  29) := to_signed(  176, 13);
        r(  30) := to_signed( -262, 13);
        r(  31) := to_signed(  143, 13);
        r(  32) := to_signed(  -31, 13);
        r(  33) := to_signed(  194, 13);
        r(  34) := to_signed(  -75, 13);
        r(  35) := to_signed(  156, 13);
        r(  36) := to_signed(  -80, 13);
        r(  37) := to_signed(  149, 13);
        r(  38) := to_signed( -128, 13);
        r(  39) := to_signed(  108, 13);
        r(  40) := to_signed( -131, 13);
        r(  41) := to_signed(  214, 13);
        r(  42) := to_signed( -207, 13);
        r(  43) := to_signed(  153, 13);
        r(  44) := to_signed( -213, 13);
        r(  45) := to_signed(  141, 13);
        r(  46) := to_signed( -301, 13);
        r(  47) := to_signed(   67, 13);
        r(  48) := to_signed(    9, 13);
        r(  49) := to_signed(  227, 13);
        r(  50) := to_signed(  -49, 13);
        r(  51) := to_signed(  174, 13);
        r(  52) := to_signed(  -56, 13);
        r(  53) := to_signed(  158, 13);
        r(  54) := to_signed( -121, 13);
        r(  55) := to_signed(   96, 13);
        r(  56) := to_signed( -103, 13);
        r(  57) := to_signed(  208, 13);
        r(  58) := to_signed( -199, 13);
        r(  59) := to_signed(  117, 13);
        r(  60) := to_signed( -205, 13);
        r(  61) := to_signed(   94, 13);
        r(  62) := to_signed( -317, 13);
        r(  63) := to_signed(  -21, 13);
        r(  64) := to_signed(   45, 13);
        r(  65) := to_signed(  258, 13);
        r(  66) := to_signed(  -21, 13);
        r(  67) := to_signed(  191, 13);
        r(  68) := to_signed(  -33, 13);
        r(  69) := to_signed(  163, 13);
        r(  70) := to_signed( -106, 13);
        r(  71) := to_signed(   85, 13);
        r(  72) := to_signed(  -81, 13);
        r(  73) := to_signed(  192, 13);
        r(  74) := to_signed( -187, 13);
        r(  75) := to_signed(   73, 13);
        r(  76) := to_signed( -194, 13);
        r(  77) := to_signed(   40, 13);
        r(  78) := to_signed( -314, 13);
        r(  79) := to_signed( -107, 13);
        r(  80) := to_signed(   78, 13);
        r(  81) := to_signed(  284, 13);
        r(  82) := to_signed(    9, 13);
        r(  83) := to_signed(  208, 13);
        r(  84) := to_signed(  -11, 13);
        r(  85) := to_signed(  163, 13);
        r(  86) := to_signed(  -86, 13);
        r(  87) := to_signed(   75, 13);
        r(  88) := to_signed(  -66, 13);
        r(  89) := to_signed(  161, 13);
        r(  90) := to_signed( -172, 13);
        r(  91) := to_signed(   25, 13);
        r(  92) := to_signed( -181, 13);
        r(  93) := to_signed(  -16, 13);
        r(  94) := to_signed( -298, 13);
        r(  95) := to_signed( -179, 13);
        r(  96) := to_signed(  107, 13);
        r(  97) := to_signed(  303, 13);
        r(  98) := to_signed(   39, 13);
        r(  99) := to_signed(  224, 13);
        r( 100) := to_signed(   11, 13);
        r( 101) := to_signed(  159, 13);
        r( 102) := to_signed(  -62, 13);
        r( 103) := to_signed(   69, 13);
        r( 104) := to_signed(  -57, 13);
        r( 105) := to_signed(  118, 13);
        r( 106) := to_signed( -157, 13);
        r( 107) := to_signed(  -25, 13);
        r( 108) := to_signed( -170, 13);
        r( 109) := to_signed(  -68, 13);
        r( 110) := to_signed( -276, 13);
        r( 111) := to_signed( -229, 13);
        r( 112) := to_signed(  129, 13);
        r( 113) := to_signed(  312, 13);
        r( 114) := to_signed(   69, 13);
        r( 115) := to_signed(  238, 13);
        r( 116) := to_signed(   31, 13);
        r( 117) := to_signed(  154, 13);
        r( 118) := to_signed(  -33, 13);
        r( 119) := to_signed(   71, 13);
        r( 120) := to_signed(  -55, 13);
        r( 121) := to_signed(   64, 13);
        r( 122) := to_signed( -144, 13);
        r( 123) := to_signed(  -72, 13);
        r( 124) := to_signed( -159, 13);
        r( 125) := to_signed( -112, 13);
        r( 126) := to_signed( -250, 13);
        r( 127) := to_signed( -257, 13);
        r( 128) := to_signed(  144, 13);
        r( 129) := to_signed(  308, 13);
        r( 130) := to_signed(   96, 13);
        r( 131) := to_signed(  250, 13);
        r( 132) := to_signed(   49, 13);
        r( 133) := to_signed(  149, 13);
        r( 134) := to_signed(   -3, 13);
        r( 135) := to_signed(   80, 13);
        r( 136) := to_signed(  -59, 13);
        r( 137) := to_signed(    5, 13);
        r( 138) := to_signed( -132, 13);
        r( 139) := to_signed( -113, 13);
        r( 140) := to_signed( -150, 13);
        r( 141) := to_signed( -146, 13);
        r( 142) := to_signed( -224, 13);
        r( 143) := to_signed( -268, 13);
        r( 144) := to_signed(  151, 13);
        r( 145) := to_signed(  294, 13);
        r( 146) := to_signed(  118, 13);
        r( 147) := to_signed(  256, 13);
        r( 148) := to_signed(   66, 13);
        r( 149) := to_signed(  147, 13);
        r( 150) := to_signed(   29, 13);
        r( 151) := to_signed(   98, 13);
        r( 152) := to_signed(  -69, 13);
        r( 153) := to_signed(  -53, 13);
        r( 154) := to_signed( -125, 13);
        r( 155) := to_signed( -148, 13);
        r( 156) := to_signed( -142, 13);
        r( 157) := to_signed( -170, 13);
        r( 158) := to_signed( -198, 13);
        r( 159) := to_signed( -265, 13);
        r( 160) := to_signed(  149, 13);
        r( 161) := to_signed(  270, 13);
        r( 162) := to_signed(  132, 13);
        r( 163) := to_signed(  256, 13);
        r( 164) := to_signed(   81, 13);
        r( 165) := to_signed(  150, 13);
        r( 166) := to_signed(   60, 13);
        r( 167) := to_signed(  124, 13);
        r( 168) := to_signed(  -83, 13);
        r( 169) := to_signed( -106, 13);
        r( 170) := to_signed( -121, 13);
        r( 171) := to_signed( -175, 13);
        r( 172) := to_signed( -134, 13);
        r( 173) := to_signed( -183, 13);
        r( 174) := to_signed( -173, 13);
        r( 175) := to_signed( -252, 13);
        r( 176) := to_signed(  140, 13);
        r( 177) := to_signed(  240, 13);
        r( 178) := to_signed(  137, 13);
        r( 179) := to_signed(  247, 13);
        r( 180) := to_signed(   95, 13);
        r( 181) := to_signed(  158, 13);
        r( 182) := to_signed(   89, 13);
        r( 183) := to_signed(  155, 13);
        r( 184) := to_signed(  -99, 13);
        r( 185) := to_signed( -152, 13);
        r( 186) := to_signed( -121, 13);
        r( 187) := to_signed( -197, 13);
        r( 188) := to_signed( -126, 13);
        r( 189) := to_signed( -188, 13);
        r( 190) := to_signed( -149, 13);
        r( 191) := to_signed( -232, 13);
        r( 192) := to_signed(  126, 13);
        r( 193) := to_signed(  209, 13);
        r( 194) := to_signed(  132, 13);
        r( 195) := to_signed(  228, 13);
        r( 196) := to_signed(  110, 13);
        r( 197) := to_signed(  175, 13);
        r( 198) := to_signed(  114, 13);
        r( 199) := to_signed(  188, 13);
        r( 200) := to_signed( -116, 13);
        r( 201) := to_signed( -190, 13);
        r( 202) := to_signed( -126, 13);
        r( 203) := to_signed( -215, 13);
        r( 204) := to_signed( -117, 13);
        r( 205) := to_signed( -184, 13);
        r( 206) := to_signed( -127, 13);
        r( 207) := to_signed( -206, 13);
        r( 208) := to_signed(  109, 13);
        r( 209) := to_signed(  179, 13);
        r( 210) := to_signed(  117, 13);
        r( 211) := to_signed(  199, 13);
        r( 212) := to_signed(  124, 13);
        r( 213) := to_signed(  198, 13);
        r( 214) := to_signed(  132, 13);
        r( 215) := to_signed(  217, 13);
        r( 216) := to_signed( -132, 13);
        r( 217) := to_signed( -220, 13);
        r( 218) := to_signed( -134, 13);
        r( 219) := to_signed( -230, 13);
        r( 220) := to_signed( -106, 13);
        r( 221) := to_signed( -171, 13);
        r( 222) := to_signed( -106, 13);
        r( 223) := to_signed( -177, 13);
        r( 224) := to_signed(   92, 13);
        r( 225) := to_signed(  153, 13);
        r( 226) := to_signed(   96, 13);
        r( 227) := to_signed(  161, 13);
        r( 228) := to_signed(  139, 13);
        r( 229) := to_signed(  228, 13);
        r( 230) := to_signed(  144, 13);
        r( 231) := to_signed(  238, 13);
        r( 232) := to_signed( -145, 13);
        r( 233) := to_signed( -243, 13);
        r( 234) := to_signed( -144, 13);
        r( 235) := to_signed( -243, 13);
        r( 236) := to_signed(  -90, 13);
        r( 237) := to_signed( -149, 13);
        r( 238) := to_signed(  -89, 13);
        r( 239) := to_signed( -147, 13);
        r( 240) := to_signed(   76, 13);
        r( 241) := to_signed(  129, 13);
        r( 242) := to_signed(   70, 13);
        r( 243) := to_signed(  118, 13);
        r( 244) := to_signed(  156, 13);
        r( 245) := to_signed(  261, 13);
        r( 246) := to_signed(  149, 13);
        r( 247) := to_signed(  246, 13);
        r( 248) := to_signed( -154, 13);
        r( 249) := to_signed( -259, 13);
        r( 250) := to_signed( -155, 13);
        r( 251) := to_signed( -256, 13);
        r( 252) := to_signed(  -71, 13);
        r( 253) := to_signed( -117, 13);
        r( 254) := to_signed(  -73, 13);
        r( 255) := to_signed( -119, 13);
        r( 256) := to_signed(   64, 13);
        r( 257) := to_signed(  109, 13);
        r( 258) := to_signed(   44, 13);
        r( 259) := to_signed(   71, 13);
        r( 260) := to_signed(  174, 13);
        r( 261) := to_signed(  294, 13);
        r( 262) := to_signed(  147, 13);
        r( 263) := to_signed(  238, 13);
        r( 264) := to_signed( -158, 13);
        r( 265) := to_signed( -269, 13);
        r( 266) := to_signed( -165, 13);
        r( 267) := to_signed( -269, 13);
        r( 268) := to_signed(  -46, 13);
        r( 269) := to_signed(  -77, 13);
        r( 270) := to_signed(  -60, 13);
        r( 271) := to_signed(  -97, 13);
        r( 272) := to_signed(   57, 13);
        r( 273) := to_signed(   90, 13);
        r( 274) := to_signed(   17, 13);
        r( 275) := to_signed(   22, 13);
        r( 276) := to_signed(  194, 13);
        r( 277) := to_signed(  320, 13);
        r( 278) := to_signed(  141, 13);
        r( 279) := to_signed(  218, 13);
        r( 280) := to_signed( -155, 13);
        r( 281) := to_signed( -271, 13);
        r( 282) := to_signed( -174, 13);
        r( 283) := to_signed( -282, 13);
        r( 284) := to_signed(  -15, 13);
        r( 285) := to_signed(  -32, 13);
        r( 286) := to_signed(  -47, 13);
        r( 287) := to_signed(  -81, 13);
        r( 288) := to_signed(   55, 13);
        r( 289) := to_signed(   71, 13);
        r( 290) := to_signed(   -7, 13);
        r( 291) := to_signed(  -27, 13);
        r( 292) := to_signed(  214, 13);
        r( 293) := to_signed(  337, 13);
        r( 294) := to_signed(  132, 13);
        r( 295) := to_signed(  186, 13);
        r( 296) := to_signed( -144, 13);
        r( 297) := to_signed( -267, 13);
        r( 298) := to_signed( -179, 13);
        r( 299) := to_signed( -294, 13);
        r( 300) := to_signed(   20, 13);
        r( 301) := to_signed(   13, 13);
        r( 302) := to_signed(  -36, 13);
        r( 303) := to_signed(  -72, 13);
        r( 304) := to_signed(   58, 13);
        r( 305) := to_signed(   51, 13);
        r( 306) := to_signed(  -28, 13);
        r( 307) := to_signed(  -77, 13);
        r( 308) := to_signed(  233, 13);
        r( 309) := to_signed(  340, 13);
        r( 310) := to_signed(  121, 13);
        r( 311) := to_signed(  146, 13);
        r( 312) := to_signed( -123, 13);
        r( 313) := to_signed( -255, 13);
        r( 314) := to_signed( -178, 13);
        r( 315) := to_signed( -305, 13);
        r( 316) := to_signed(   60, 13);
        r( 317) := to_signed(   54, 13);
        r( 318) := to_signed(  -25, 13);
        r( 319) := to_signed(  -70, 13);
        r( 320) := to_signed(   67, 13);
        r( 321) := to_signed(   27, 13);
        r( 322) := to_signed(  -44, 13);
        r( 323) := to_signed( -124, 13);
        r( 324) := to_signed(  251, 13);
        r( 325) := to_signed(  328, 13);
        r( 326) := to_signed(  109, 13);
        r( 327) := to_signed(  102, 13);
        r( 328) := to_signed(  -92, 13);
        r( 329) := to_signed( -237, 13);
        r( 330) := to_signed( -171, 13);
        r( 331) := to_signed( -311, 13);
        r( 332) := to_signed(  100, 13);
        r( 333) := to_signed(   85, 13);
        r( 334) := to_signed(  -13, 13);
        r( 335) := to_signed(  -74, 13);
        r( 336) := to_signed(   80, 13);
        r( 337) := to_signed(   -1, 13);
        r( 338) := to_signed(  -55, 13);
        r( 339) := to_signed( -169, 13);
        r( 340) := to_signed(  266, 13);
        r( 341) := to_signed(  301, 13);
        r( 342) := to_signed(   98, 13);
        r( 343) := to_signed(   54, 13);
        r( 344) := to_signed(  -52, 13);
        r( 345) := to_signed( -214, 13);
        r( 346) := to_signed( -156, 13);
        r( 347) := to_signed( -311, 13);
        r( 348) := to_signed(  140, 13);
        r( 349) := to_signed(  103, 13);
        r( 350) := to_signed(    1, 13);
        r( 351) := to_signed(  -81, 13);
        r( 352) := to_signed(   96, 13);
        r( 353) := to_signed(  -35, 13);
        r( 354) := to_signed(  -60, 13);
        r( 355) := to_signed( -209, 13);
        r( 356) := to_signed(  276, 13);
        r( 357) := to_signed(  260, 13);
        r( 358) := to_signed(   87, 13);
        r( 359) := to_signed(    6, 13);
        r( 360) := to_signed(   -6, 13);
        r( 361) := to_signed( -189, 13);
        r( 362) := to_signed( -133, 13);
        r( 363) := to_signed( -303, 13);
        r( 364) := to_signed(  175, 13);
        r( 365) := to_signed(  108, 13);
        r( 366) := to_signed(   15, 13);
        r( 367) := to_signed(  -89, 13);
        r( 368) := to_signed(  114, 13);
        r( 369) := to_signed(  -75, 13);
        r( 370) := to_signed(  -60, 13);
        r( 371) := to_signed( -242, 13);
        r( 372) := to_signed(  282, 13);
        r( 373) := to_signed(  205, 13);
        r( 374) := to_signed(   77, 13);
        r( 375) := to_signed(  -43, 13);
        r( 376) := to_signed(   44, 13);
        r( 377) := to_signed( -161, 13);
        r( 378) := to_signed( -102, 13);
        r( 379) := to_signed( -286, 13);
        r( 380) := to_signed(  206, 13);
        r( 381) := to_signed(  102, 13);
        r( 382) := to_signed(   30, 13);
        r( 383) := to_signed(  -96, 13);
        r( 384) := to_signed(  131, 13);
        r( 385) := to_signed( -118, 13);
        r( 386) := to_signed(  -54, 13);
        r( 387) := to_signed( -265, 13);
        r( 388) := to_signed(  282, 13);
        r( 389) := to_signed(  137, 13);
        r( 390) := to_signed(   68, 13);
        r( 391) := to_signed(  -88, 13);
        r( 392) := to_signed(   93, 13);
        r( 393) := to_signed( -134, 13);
        r( 394) := to_signed(  -67, 13);
        r( 395) := to_signed( -260, 13);
        r( 396) := to_signed(  231, 13);
        r( 397) := to_signed(   87, 13);
        r( 398) := to_signed(   46, 13);
        r( 399) := to_signed( -101, 13);
        r( 400) := to_signed(  146, 13);
        r( 401) := to_signed( -162, 13);
        r( 402) := to_signed(  -44, 13);
        r( 403) := to_signed( -276, 13);
        r( 404) := to_signed(  274, 13);
        r( 405) := to_signed(   59, 13);
        r( 406) := to_signed(   58, 13);
        r( 407) := to_signed( -129, 13);
        r( 408) := to_signed(  139, 13);
        r( 409) := to_signed( -107, 13);
        r( 410) := to_signed(  -28, 13);
        r( 411) := to_signed( -225, 13);
        r( 412) := to_signed(  250, 13);
        r( 413) := to_signed(   67, 13);
        r( 414) := to_signed(   63, 13);
        r( 415) := to_signed( -101, 13);
        r( 416) := to_signed(  156, 13);
        r( 417) := to_signed( -201, 13);
        r( 418) := to_signed(  -31, 13);
        r( 419) := to_signed( -275, 13);
        r( 420) := to_signed(  257, 13);
        r( 421) := to_signed(  -26, 13);
        r( 422) := to_signed(   48, 13);
        r( 423) := to_signed( -163, 13);
        r( 424) := to_signed(  181, 13);
        r( 425) := to_signed(  -81, 13);
        r( 426) := to_signed(   12, 13);
        r( 427) := to_signed( -186, 13);
        r( 428) := to_signed(  262, 13);
        r( 429) := to_signed(   44, 13);
        r( 430) := to_signed(   79, 13);
        r( 431) := to_signed(  -96, 13);
        r( 432) := to_signed(  158, 13);
        r( 433) := to_signed( -232, 13);
        r( 434) := to_signed(  -17, 13);
        r( 435) := to_signed( -265, 13);
        r( 436) := to_signed(  226, 13);
        r( 437) := to_signed( -108, 13);
        r( 438) := to_signed(   37, 13);
        r( 439) := to_signed( -188, 13);
        r( 440) := to_signed(  217, 13);
        r( 441) := to_signed(  -56, 13);
        r( 442) := to_signed(   52, 13);
        r( 443) := to_signed( -144, 13);
        r( 444) := to_signed(  267, 13);
        r( 445) := to_signed(   23, 13);
        r( 446) := to_signed(   94, 13);
        r( 447) := to_signed(  -86, 13);
        r( 448) := to_signed(  150, 13);
        r( 449) := to_signed( -250, 13);
        r( 450) := to_signed(   -5, 13);
        r( 451) := to_signed( -247, 13);
        r( 452) := to_signed(  181, 13);
        r( 453) := to_signed( -178, 13);
        r( 454) := to_signed(   23, 13);
        r( 455) := to_signed( -203, 13);
        r( 456) := to_signed(  249, 13);
        r( 457) := to_signed(  -31, 13);
        r( 458) := to_signed(   91, 13);
        r( 459) := to_signed( -101, 13);
        r( 460) := to_signed(  265, 13);
        r( 461) := to_signed(    6, 13);
        r( 462) := to_signed(  109, 13);
        r( 463) := to_signed(  -73, 13);
        r( 464) := to_signed(  130, 13);
        r( 465) := to_signed( -256, 13);
        r( 466) := to_signed(    4, 13);
        r( 467) := to_signed( -226, 13);
        r( 468) := to_signed(  119, 13);
        r( 469) := to_signed( -226, 13);
        r( 470) := to_signed(    6, 13);
        r( 471) := to_signed( -208, 13);
        r( 472) := to_signed(  275, 13);
        r( 473) := to_signed(   -7, 13);
        r( 474) := to_signed(  128, 13);
        r( 475) := to_signed(  -61, 13);
        r( 476) := to_signed(  255, 13);
        r( 477) := to_signed(   -4, 13);
        r( 478) := to_signed(  122, 13);
        r( 479) := to_signed(  -55, 13);
        r( 480) := to_signed(   94, 13);
        r( 481) := to_signed( -252, 13);
        r( 482) := to_signed(    7, 13);
        r( 483) := to_signed( -203, 13);
        r( 484) := to_signed(   45, 13);
        r( 485) := to_signed( -249, 13);
        r( 486) := to_signed(  -15, 13);
        r( 487) := to_signed( -206, 13);
        r( 488) := to_signed(  295, 13);
        r( 489) := to_signed(   17, 13);
        r( 490) := to_signed(  163, 13);
        r( 491) := to_signed(  -22, 13);
        r( 492) := to_signed(  238, 13);
        r( 493) := to_signed(   -6, 13);
        r( 494) := to_signed(  133, 13);
        r( 495) := to_signed(  -36, 13);
        r( 496) := to_signed(   42, 13);
        r( 497) := to_signed( -240, 13);
        r( 498) := to_signed(    3, 13);
        r( 499) := to_signed( -183, 13);
        r( 500) := to_signed(  -31, 13);
        r( 501) := to_signed( -251, 13);
        r( 502) := to_signed(  -40, 13);
        r( 503) := to_signed( -197, 13);
        r( 504) := to_signed(  307, 13);
        r( 505) := to_signed(   40, 13);
        r( 506) := to_signed(  197, 13);
        r( 507) := to_signed(   15, 13);
        r( 508) := to_signed(  216, 13);
        r( 509) := to_signed(   -2, 13);
        r( 510) := to_signed(  142, 13);
        r( 511) := to_signed(  -15, 13);
        r( 512) := to_signed(  -20, 13);
        r( 513) := to_signed( -225, 13);
        r( 514) := to_signed(  -13, 13);
        r( 515) := to_signed( -167, 13);
        r( 516) := to_signed(  -93, 13);
        r( 517) := to_signed( -238, 13);
        r( 518) := to_signed(  -67, 13);
        r( 519) := to_signed( -185, 13);
        r( 520) := to_signed(  313, 13);
        r( 521) := to_signed(   63, 13);
        r( 522) := to_signed(  227, 13);
        r( 523) := to_signed(   50, 13);
        r( 524) := to_signed(  191, 13);
        r( 525) := to_signed(    8, 13);
        r( 526) := to_signed(  147, 13);
        r( 527) := to_signed(    7, 13);
        r( 528) := to_signed(  -85, 13);
        r( 529) := to_signed( -210, 13);
        r( 530) := to_signed(  -42, 13);
        r( 531) := to_signed( -155, 13);
        r( 532) := to_signed( -135, 13);
        r( 533) := to_signed( -216, 13);
        r( 534) := to_signed(  -92, 13);
        r( 535) := to_signed( -171, 13);
        r( 536) := to_signed(  309, 13);
        r( 537) := to_signed(   87, 13);
        r( 538) := to_signed(  252, 13);
        r( 539) := to_signed(   82, 13);
        r( 540) := to_signed(  168, 13);
        r( 541) := to_signed(   24, 13);
        r( 542) := to_signed(  148, 13);
        r( 543) := to_signed(   29, 13);
        r( 544) := to_signed( -142, 13);
        r( 545) := to_signed( -197, 13);
        r( 546) := to_signed(  -84, 13);
        r( 547) := to_signed( -150, 13);
        r( 548) := to_signed( -154, 13);
        r( 549) := to_signed( -191, 13);
        r( 550) := to_signed( -113, 13);
        r( 551) := to_signed( -156, 13);
        r( 552) := to_signed(  297, 13);
        r( 553) := to_signed(  111, 13);
        r( 554) := to_signed(  267, 13);
        r( 555) := to_signed(  113, 13);
        r( 556) := to_signed(  149, 13);
        r( 557) := to_signed(   43, 13);
        r( 558) := to_signed(  146, 13);
        r( 559) := to_signed(   51, 13);
        r( 560) := to_signed( -185, 13);
        r( 561) := to_signed( -187, 13);
        r( 562) := to_signed( -133, 13);
        r( 563) := to_signed( -151, 13);
        r( 564) := to_signed( -157, 13);
        r( 565) := to_signed( -165, 13);
        r( 566) := to_signed( -126, 13);
        r( 567) := to_signed( -140, 13);
        r( 568) := to_signed(  279, 13);
        r( 569) := to_signed(  136, 13);
        r( 570) := to_signed(  266, 13);
        r( 571) := to_signed(  140, 13);
        r( 572) := to_signed(  137, 13);
        r( 573) := to_signed(   65, 13);
        r( 574) := to_signed(  140, 13);
        r( 575) := to_signed(   73, 13);
        r( 576) := to_signed( -214, 13);
        r( 577) := to_signed( -182, 13);
        r( 578) := to_signed( -179, 13);
        r( 579) := to_signed( -158, 13);
        r( 580) := to_signed( -148, 13);
        r( 581) := to_signed( -139, 13);
        r( 582) := to_signed( -130, 13);
        r( 583) := to_signed( -125, 13);
        r( 584) := to_signed(  255, 13);
        r( 585) := to_signed(  161, 13);
        r( 586) := to_signed(  246, 13);
        r( 587) := to_signed(  160, 13);
        r( 588) := to_signed(  132, 13);
        r( 589) := to_signed(   90, 13);
        r( 590) := to_signed(  133, 13);
        r( 591) := to_signed(   93, 13);
        r( 592) := to_signed( -231, 13);
        r( 593) := to_signed( -183, 13);
        r( 594) := to_signed( -213, 13);
        r( 595) := to_signed( -171, 13);
        r( 596) := to_signed( -133, 13);
        r( 597) := to_signed( -115, 13);
        r( 598) := to_signed( -127, 13);
        r( 599) := to_signed( -110, 13);
        r( 600) := to_signed(  228, 13);
        r( 601) := to_signed(  184, 13);
        r( 602) := to_signed(  207, 13);
        r( 603) := to_signed(  170, 13);
        r( 604) := to_signed(  135, 13);
        r( 605) := to_signed(  119, 13);
        r( 606) := to_signed(  127, 13);
        r( 607) := to_signed(  113, 13);
        r( 608) := to_signed( -241, 13);
        r( 609) := to_signed( -190, 13);
        r( 610) := to_signed( -230, 13);
        r( 611) := to_signed( -185, 13);
        r( 612) := to_signed( -114, 13);
        r( 613) := to_signed(  -91, 13);
        r( 614) := to_signed( -116, 13);
        r( 615) := to_signed(  -95, 13);
        r( 616) := to_signed(  200, 13);
        r( 617) := to_signed(  203, 13);
        r( 618) := to_signed(  157, 13);
        r( 619) := to_signed(  165, 13);
        r( 620) := to_signed(  144, 13);
        r( 621) := to_signed(  150, 13);
        r( 622) := to_signed(  123, 13);
        r( 623) := to_signed(  130, 13);
        r( 624) := to_signed( -247, 13);
        r( 625) := to_signed( -203, 13);
        r( 626) := to_signed( -234, 13);
        r( 627) := to_signed( -197, 13);
        r( 628) := to_signed(  -94, 13);
        r( 629) := to_signed(  -69, 13);
        r( 630) := to_signed( -101, 13);
        r( 631) := to_signed(  -79, 13);
        r( 632) := to_signed(  170, 13);
        r( 633) := to_signed(  210, 13);
        r( 634) := to_signed(  107, 13);
        r( 635) := to_signed(  142, 13);
        r( 636) := to_signed(  160, 13);
        r( 637) := to_signed(  184, 13);
        r( 638) := to_signed(  122, 13);
        r( 639) := to_signed(  144, 13);
        r( 640) := to_signed( -253, 13);
        r( 641) := to_signed( -220, 13);
        r( 642) := to_signed( -227, 13);
        r( 643) := to_signed( -202, 13);
        r( 644) := to_signed(  -72, 13);
        r( 645) := to_signed(  -47, 13);
        r( 646) := to_signed(  -83, 13);
        r( 647) := to_signed(  -62, 13);
        r( 648) := to_signed(  137, 13);
        r( 649) := to_signed(  196, 13);
        r( 650) := to_signed(   65, 13);
        r( 651) := to_signed(  105, 13);
        r( 652) := to_signed(  182, 13);
        r( 653) := to_signed(  218, 13);
        r( 654) := to_signed(  126, 13);
        r( 655) := to_signed(  154, 13);
        r( 656) := to_signed( -259, 13);
        r( 657) := to_signed( -238, 13);
        r( 658) := to_signed( -216, 13);
        r( 659) := to_signed( -199, 13);
        r( 660) := to_signed(  -49, 13);
        r( 661) := to_signed(  -27, 13);
        r( 662) := to_signed(  -61, 13);
        r( 663) := to_signed(  -44, 13);
        r( 664) := to_signed(  101, 13);
        r( 665) := to_signed(  156, 13);
        r( 666) := to_signed(   32, 13);
        r( 667) := to_signed(   64, 13);
        r( 668) := to_signed(  210, 13);
        r( 669) := to_signed(  249, 13);
        r( 670) := to_signed(  135, 13);
        r( 671) := to_signed(  161, 13);
        r( 672) := to_signed( -266, 13);
        r( 673) := to_signed( -252, 13);
        r( 674) := to_signed( -202, 13);
        r( 675) := to_signed( -191, 13);
        r( 676) := to_signed(  -25, 13);
        r( 677) := to_signed(   -8, 13);
        r( 678) := to_signed(  -37, 13);
        r( 679) := to_signed(  -25, 13);
        r( 680) := to_signed(   58, 13);
        r( 681) := to_signed(   92, 13);
        r( 682) := to_signed(    9, 13);
        r( 683) := to_signed(   27, 13);
        r( 684) := to_signed(  242, 13);
        r( 685) := to_signed(  270, 13);
        r( 686) := to_signed(  148, 13);
        r( 687) := to_signed(  166, 13);
        r( 688) := to_signed( -276, 13);
        r( 689) := to_signed( -259, 13);
        r( 690) := to_signed( -186, 13);
        r( 691) := to_signed( -177, 13);
        r( 692) := to_signed(    1, 13);
        r( 693) := to_signed(    8, 13);
        r( 694) := to_signed(   -9, 13);
        r( 695) := to_signed(   -4, 13);
        r( 696) := to_signed(    9, 13);
        r( 697) := to_signed(   19, 13);
        r( 698) := to_signed(   -7, 13);
        r( 699) := to_signed(   -1, 13);
        r( 700) := to_signed(  273, 13);
        r( 701) := to_signed(  274, 13);
        r( 702) := to_signed(  166, 13);
        r( 703) := to_signed(  168, 13);
        r( 704) := to_signed( -287, 13);
        r( 705) := to_signed( -256, 13);
        r( 706) := to_signed( -170, 13);
        r( 707) := to_signed( -160, 13);
        r( 708) := to_signed(   27, 13);
        r( 709) := to_signed(   20, 13);
        r( 710) := to_signed(   24, 13);
        r( 711) := to_signed(   20, 13);
        r( 712) := to_signed(  -46, 13);
        r( 713) := to_signed(  -44, 13);
        r( 714) := to_signed(  -18, 13);
        r( 715) := to_signed(  -19, 13);
        r( 716) := to_signed(  298, 13);
        r( 717) := to_signed(  256, 13);
        r( 718) := to_signed(  188, 13);
        r( 719) := to_signed(  169, 13);
        r( 720) := to_signed( -298, 13);
        r( 721) := to_signed( -246, 13);
        r( 722) := to_signed( -154, 13);
        r( 723) := to_signed( -140, 13);
        r( 724) := to_signed(   50, 13);
        r( 725) := to_signed(   28, 13);
        r( 726) := to_signed(   62, 13);
        r( 727) := to_signed(   45, 13);
        r( 728) := to_signed( -102, 13);
        r( 729) := to_signed(  -91, 13);
        r( 730) := to_signed(  -25, 13);
        r( 731) := to_signed(  -30, 13);
        r( 732) := to_signed(  305, 13);
        r( 733) := to_signed(  219, 13);
        r( 734) := to_signed(  212, 13);
        r( 735) := to_signed(  171, 13);
        r( 736) := to_signed( -306, 13);
        r( 737) := to_signed( -231, 13);
        r( 738) := to_signed( -137, 13);
        r( 739) := to_signed( -118, 13);
        r( 740) := to_signed(   68, 13);
        r( 741) := to_signed(   33, 13);
        r( 742) := to_signed(  107, 13);
        r( 743) := to_signed(   74, 13);
        r( 744) := to_signed( -156, 13);
        r( 745) := to_signed( -122, 13);
        r( 746) := to_signed(  -31, 13);
        r( 747) := to_signed(  -34, 13);
        r( 748) := to_signed(  280, 13);
        r( 749) := to_signed(  172, 13);
        r( 750) := to_signed(  237, 13);
        r( 751) := to_signed(  172, 13);
        r( 752) := to_signed( -309, 13);
        r( 753) := to_signed( -211, 13);
        r( 754) := to_signed( -117, 13);
        r( 755) := to_signed(  -93, 13);
        r( 756) := to_signed(   74, 13);
        r( 757) := to_signed(   36, 13);
        r( 758) := to_signed(  158, 13);
        r( 759) := to_signed(  105, 13);
        r( 760) := to_signed( -203, 13);
        r( 761) := to_signed( -140, 13);
        r( 762) := to_signed(  -37, 13);
        r( 763) := to_signed(  -34, 13);
        r( 764) := to_signed(  222, 13);
        r( 765) := to_signed(  125, 13);
        r( 766) := to_signed(  258, 13);
        r( 767) := to_signed(  174, 13);
        r( 768) := to_signed( -306, 13);
        r( 769) := to_signed( -190, 13);
        r( 770) := to_signed(  -95, 13);
        r( 771) := to_signed(  -64, 13);
        r( 772) := to_signed(   66, 13);
        r( 773) := to_signed(   39, 13);
        r( 774) := to_signed(  213, 13);
        r( 775) := to_signed(  138, 13);
        r( 776) := to_signed( -242, 13);
        r( 777) := to_signed( -150, 13);
        r( 778) := to_signed(  -44, 13);
        r( 779) := to_signed(  -30, 13);
        r( 780) := to_signed(  141, 13);
        r( 781) := to_signed(   82, 13);
        r( 782) := to_signed(  269, 13);
        r( 783) := to_signed(  175, 13);
        r( 784) := to_signed( -296, 13);
        r( 785) := to_signed( -166, 13);
        r( 786) := to_signed(  -68, 13);
        r( 787) := to_signed(  -31, 13);
        r( 788) := to_signed(   48, 13);
        r( 789) := to_signed(   42, 13);
        r( 790) := to_signed(  264, 13);
        r( 791) := to_signed(  173, 13);
        r( 792) := to_signed( -271, 13);
        r( 793) := to_signed( -154, 13);
        r( 794) := to_signed(  -55, 13);
        r( 795) := to_signed(  -23, 13);
        r( 796) := to_signed(   58, 13);
        r( 797) := to_signed(   48, 13);
        r( 798) := to_signed(  263, 13);
        r( 799) := to_signed(  174, 13);
        r( 800) := to_signed( -280, 13);
        r( 801) := to_signed( -140, 13);
        r( 802) := to_signed(  -36, 13);
        r( 803) := to_signed(    9, 13);
        r( 804) := to_signed(   26, 13);
        r( 805) := to_signed(   46, 13);
        r( 806) := to_signed(  300, 13);
        r( 807) := to_signed(  206, 13);
        r( 808) := to_signed( -292, 13);
        r( 809) := to_signed( -155, 13);
        r( 810) := to_signed(  -70, 13);
        r( 811) := to_signed(  -15, 13);
        r( 812) := to_signed(  -14, 13);
        r( 813) := to_signed(   21, 13);
        r( 814) := to_signed(  233, 13);
        r( 815) := to_signed(  171, 13);
        r( 816) := to_signed( -257, 13);
        r( 817) := to_signed( -111, 13);
        r( 818) := to_signed(    1, 13);
        r( 819) := to_signed(   55, 13);
        r( 820) := to_signed(    7, 13);
        r( 821) := to_signed(   52, 13);
        r( 822) := to_signed(  311, 13);
        r( 823) := to_signed(  235, 13);
        r( 824) := to_signed( -307, 13);
        r( 825) := to_signed( -154, 13);
        r( 826) := to_signed(  -91, 13);
        r( 827) := to_signed(   -5, 13);
        r( 828) := to_signed(  -69, 13);
        r( 829) := to_signed(   -1, 13);
        r( 830) := to_signed(  184, 13);
        r( 831) := to_signed(  162, 13);
        r( 832) := to_signed( -228, 13);
        r( 833) := to_signed(  -78, 13);
        r( 834) := to_signed(   43, 13);
        r( 835) := to_signed(  109, 13);
        r( 836) := to_signed(   -6, 13);
        r( 837) := to_signed(   60, 13);
        r( 838) := to_signed(  293, 13);
        r( 839) := to_signed(  256, 13);
        r( 840) := to_signed( -317, 13);
        r( 841) := to_signed( -150, 13);
        r( 842) := to_signed( -116, 13);
        r( 843) := to_signed(    5, 13);
        r( 844) := to_signed( -110, 13);
        r( 845) := to_signed(  -17, 13);
        r( 846) := to_signed(  121, 13);
        r( 847) := to_signed(  148, 13);
        r( 848) := to_signed( -194, 13);
        r( 849) := to_signed(  -41, 13);
        r( 850) := to_signed(   84, 13);
        r( 851) := to_signed(  167, 13);
        r( 852) := to_signed(  -13, 13);
        r( 853) := to_signed(   69, 13);
        r( 854) := to_signed(  256, 13);
        r( 855) := to_signed(  267, 13);
        r( 856) := to_signed( -324, 13);
        r( 857) := to_signed( -146, 13);
        r( 858) := to_signed( -145, 13);
        r( 859) := to_signed(   12, 13);
        r( 860) := to_signed( -139, 13);
        r( 861) := to_signed(  -31, 13);
        r( 862) := to_signed(   57, 13);
        r( 863) := to_signed(  127, 13);
        r( 864) := to_signed( -153, 13);
        r( 865) := to_signed(    1, 13);
        r( 866) := to_signed(  119, 13);
        r( 867) := to_signed(  224, 13);
        r( 868) := to_signed(  -13, 13);
        r( 869) := to_signed(   79, 13);
        r( 870) := to_signed(  212, 13);
        r( 871) := to_signed(  265, 13);
        r( 872) := to_signed( -327, 13);
        r( 873) := to_signed( -142, 13);
        r( 874) := to_signed( -173, 13);
        r( 875) := to_signed(   15, 13);
        r( 876) := to_signed( -158, 13);
        r( 877) := to_signed(  -42, 13);
        r( 878) := to_signed(   -2, 13);
        r( 879) := to_signed(  101, 13);
        r( 880) := to_signed( -107, 13);
        r( 881) := to_signed(   48, 13);
        r( 882) := to_signed(  145, 13);
        r( 883) := to_signed(  272, 13);
        r( 884) := to_signed(   -8, 13);
        r( 885) := to_signed(   90, 13);
        r( 886) := to_signed(  171, 13);
        r( 887) := to_signed(  253, 13);
        r( 888) := to_signed( -327, 13);
        r( 889) := to_signed( -138, 13);
        r( 890) := to_signed( -197, 13);
        r( 891) := to_signed(   11, 13);
        r( 892) := to_signed( -172, 13);
        r( 893) := to_signed(  -53, 13);
        r( 894) := to_signed(  -50, 13);
        r( 895) := to_signed(   69, 13);
        r( 896) := to_signed(  -56, 13);
        r( 897) := to_signed(   97, 13);
        r( 898) := to_signed(  161, 13);
        r( 899) := to_signed(  302, 13);
        r( 900) := to_signed(    2, 13);
        r( 901) := to_signed(  101, 13);
        r( 902) := to_signed(  139, 13);
        r( 903) := to_signed(  233, 13);
        r( 904) := to_signed( -323, 13);
        r( 905) := to_signed( -135, 13);
        r( 906) := to_signed( -214, 13);
        r( 907) := to_signed(   -3, 13);
        r( 908) := to_signed( -180, 13);
        r( 909) := to_signed(  -64, 13);
        r( 910) := to_signed(  -86, 13);
        r( 911) := to_signed(   35, 13);
        r( 912) := to_signed(    1, 13);
        r( 913) := to_signed(  145, 13);
        r( 914) := to_signed(  169, 13);
        r( 915) := to_signed(  309, 13);
        r( 916) := to_signed(   18, 13);
        r( 917) := to_signed(  110, 13);
        r( 918) := to_signed(  116, 13);
        r( 919) := to_signed(  208, 13);
        r( 920) := to_signed( -316, 13);
        r( 921) := to_signed( -135, 13);
        r( 922) := to_signed( -222, 13);
        r( 923) := to_signed(  -27, 13);
        r( 924) := to_signed( -183, 13);
        r( 925) := to_signed(  -75, 13);
        r( 926) := to_signed( -112, 13);
        r( 927) := to_signed(    0, 13);
        r( 928) := to_signed(   60, 13);
        r( 929) := to_signed(  187, 13);
        r( 930) := to_signed(  172, 13);
        r( 931) := to_signed(  294, 13);
        r( 932) := to_signed(   38, 13);
        r( 933) := to_signed(  119, 13);
        r( 934) := to_signed(  102, 13);
        r( 935) := to_signed(  181, 13);
        r( 936) := to_signed( -303, 13);
        r( 937) := to_signed( -141, 13);
        r( 938) := to_signed( -220, 13);
        r( 939) := to_signed(  -57, 13);
        r( 940) := to_signed( -181, 13);
        r( 941) := to_signed(  -87, 13);
        r( 942) := to_signed( -128, 13);
        r( 943) := to_signed(  -34, 13);
        r( 944) := to_signed(  118, 13);
        r( 945) := to_signed(  218, 13);
        r( 946) := to_signed(  173, 13);
        r( 947) := to_signed(  263, 13);
        r( 948) := to_signed(   62, 13);
        r( 949) := to_signed(  126, 13);
        r( 950) := to_signed(   96, 13);
        r( 951) := to_signed(  156, 13);
        r( 952) := to_signed( -287, 13);
        r( 953) := to_signed( -154, 13);
        r( 954) := to_signed( -211, 13);
        r( 955) := to_signed(  -91, 13);
        r( 956) := to_signed( -176, 13);
        r( 957) := to_signed( -100, 13);
        r( 958) := to_signed( -134, 13);
        r( 959) := to_signed(  -63, 13);
        r( 960) := to_signed(  172, 13);
        r( 961) := to_signed(  233, 13);
        r( 962) := to_signed(  172, 13);
        r( 963) := to_signed(  224, 13);
        r( 964) := to_signed(   90, 13);
        r( 965) := to_signed(  130, 13);
        r( 966) := to_signed(   98, 13);
        r( 967) := to_signed(  135, 13);
        r( 968) := to_signed( -267, 13);
        r( 969) := to_signed( -174, 13);
        r( 970) := to_signed( -197, 13);
        r( 971) := to_signed( -122, 13);
        r( 972) := to_signed( -165, 13);
        r( 973) := to_signed( -114, 13);
        r( 974) := to_signed( -132, 13);
        r( 975) := to_signed(  -87, 13);
        r( 976) := to_signed(  216, 13);
        r( 977) := to_signed(  230, 13);
        r( 978) := to_signed(  171, 13);
        r( 979) := to_signed(  183, 13);
        r( 980) := to_signed(  122, 13);
        r( 981) := to_signed(  134, 13);
        r( 982) := to_signed(  107, 13);
        r( 983) := to_signed(  117, 13);
        r( 984) := to_signed( -246, 13);
        r( 985) := to_signed( -203, 13);
        r( 986) := to_signed( -179, 13);
        r( 987) := to_signed( -149, 13);
        r( 988) := to_signed( -150, 13);
        r( 989) := to_signed( -128, 13);
        r( 990) := to_signed( -122, 13);
        r( 991) := to_signed( -104, 13);
        r( 992) := to_signed(  247, 13);
        r( 993) := to_signed(  208, 13);
        r( 994) := to_signed(  168, 13);
        r( 995) := to_signed(  143, 13);
        r( 996) := to_signed(  156, 13);
        r( 997) := to_signed(  135, 13);
        r( 998) := to_signed(  121, 13);
        r( 999) := to_signed(  105, 13);
        r(1000) := to_signed( -226, 13);
        r(1001) := to_signed( -235, 13);
        r(1002) := to_signed( -161, 13);
        r(1003) := to_signed( -170, 13);
        r(1004) := to_signed( -131, 13);
        r(1005) := to_signed( -140, 13);
        r(1006) := to_signed( -105, 13);
        r(1007) := to_signed( -114, 13);
        r(1008) := to_signed(  260, 13);
        r(1009) := to_signed(  170, 13);
        r(1010) := to_signed(  163, 13);
        r(1011) := to_signed(  107, 13);
        r(1012) := to_signed(  191, 13);
        r(1013) := to_signed(  136, 13);
        r(1014) := to_signed(  139, 13);
        r(1015) := to_signed(   99, 13);
        r(1016) := to_signed( -212, 13);
        r(1017) := to_signed( -268, 13);
        r(1018) := to_signed( -143, 13);
        r(1019) := to_signed( -184, 13);
        r(1020) := to_signed( -107, 13);
        r(1021) := to_signed( -148, 13);
        r(1022) := to_signed(  -84, 13);
        r(1023) := to_signed( -116, 13);
        return r;
    end function;
    constant C_VERT_SX : t_vert_sx := gen_vert_sx;

    type t_vert_sy is array(0 to 1024-1) of signed(12 downto 0);
    function gen_vert_sy return t_vert_sy is
        variable r : t_vert_sy;
    begin
        r(   0) := to_signed( -120, 13);
        r(   1) := to_signed( -120, 13);
        r(   2) := to_signed(  120, 13);
        r(   3) := to_signed(  120, 13);
        r(   4) := to_signed( -120, 13);
        r(   5) := to_signed( -120, 13);
        r(   6) := to_signed(  120, 13);
        r(   7) := to_signed(  120, 13);
        r(   8) := to_signed( -200, 13);
        r(   9) := to_signed( -200, 13);
        r(  10) := to_signed(  200, 13);
        r(  11) := to_signed(  200, 13);
        r(  12) := to_signed( -200, 13);
        r(  13) := to_signed( -200, 13);
        r(  14) := to_signed(  200, 13);
        r(  15) := to_signed(  200, 13);
        r(  16) := to_signed( -103, 13);
        r(  17) := to_signed(  -90, 13);
        r(  18) := to_signed(  126, 13);
        r(  19) := to_signed(  153, 13);
        r(  20) := to_signed( -126, 13);
        r(  21) := to_signed( -114, 13);
        r(  22) := to_signed(  109, 13);
        r(  23) := to_signed(  136, 13);
        r(  24) := to_signed( -192, 13);
        r(  25) := to_signed( -179, 13);
        r(  26) := to_signed(  175, 13);
        r(  27) := to_signed(  225, 13);
        r(  28) := to_signed( -234, 13);
        r(  29) := to_signed( -225, 13);
        r(  30) := to_signed(  149, 13);
        r(  31) := to_signed(  199, 13);
        r(  32) := to_signed(  -84, 13);
        r(  33) := to_signed(  -67, 13);
        r(  34) := to_signed(  134, 13);
        r(  35) := to_signed(  182, 13);
        r(  36) := to_signed( -125, 13);
        r(  37) := to_signed( -111, 13);
        r(  38) := to_signed(  101, 13);
        r(  39) := to_signed(  147, 13);
        r(  40) := to_signed( -179, 13);
        r(  41) := to_signed( -169, 13);
        r(  42) := to_signed(  155, 13);
        r(  43) := to_signed(  240, 13);
        r(  44) := to_signed( -247, 13);
        r(  45) := to_signed( -252, 13);
        r(  46) := to_signed(  104, 13);
        r(  47) := to_signed(  184, 13);
        r(  48) := to_signed(  -68, 13);
        r(  49) := to_signed(  -51, 13);
        r(  50) := to_signed(  144, 13);
        r(  51) := to_signed(  207, 13);
        r(  52) := to_signed( -118, 13);
        r(  53) := to_signed( -111, 13);
        r(  54) := to_signed(   99, 13);
        r(  55) := to_signed(  154, 13);
        r(  56) := to_signed( -165, 13);
        r(  57) := to_signed( -170, 13);
        r(  58) := to_signed(  140, 13);
        r(  59) := to_signed(  244, 13);
        r(  60) := to_signed( -244, 13);
        r(  61) := to_signed( -277, 13);
        r(  62) := to_signed(   72, 13);
        r(  63) := to_signed(  157, 13);
        r(  64) := to_signed(  -55, 13);
        r(  65) := to_signed(  -46, 13);
        r(  66) := to_signed(  154, 13);
        r(  67) := to_signed(  227, 13);
        r(  68) := to_signed( -107, 13);
        r(  69) := to_signed( -113, 13);
        r(  70) := to_signed(  102, 13);
        r(  71) := to_signed(  161, 13);
        r(  72) := to_signed( -154, 13);
        r(  73) := to_signed( -184, 13);
        r(  74) := to_signed(  130, 13);
        r(  75) := to_signed(  236, 13);
        r(  76) := to_signed( -230, 13);
        r(  77) := to_signed( -295, 13);
        r(  78) := to_signed(   56, 13);
        r(  79) := to_signed(  129, 13);
        r(  80) := to_signed(  -47, 13);
        r(  81) := to_signed(  -52, 13);
        r(  82) := to_signed(  163, 13);
        r(  83) := to_signed(  241, 13);
        r(  84) := to_signed(  -95, 13);
        r(  85) := to_signed( -116, 13);
        r(  86) := to_signed(  111, 13);
        r(  87) := to_signed(  169, 13);
        r(  88) := to_signed( -148, 13);
        r(  89) := to_signed( -208, 13);
        r(  90) := to_signed(  123, 13);
        r(  91) := to_signed(  218, 13);
        r(  92) := to_signed( -210, 13);
        r(  93) := to_signed( -302, 13);
        r(  94) := to_signed(   54, 13);
        r(  95) := to_signed(  108, 13);
        r(  96) := to_signed(  -47, 13);
        r(  97) := to_signed(  -69, 13);
        r(  98) := to_signed(  169, 13);
        r(  99) := to_signed(  247, 13);
        r( 100) := to_signed(  -82, 13);
        r( 101) := to_signed( -118, 13);
        r( 102) := to_signed(  125, 13);
        r( 103) := to_signed(  179, 13);
        r( 104) := to_signed( -148, 13);
        r( 105) := to_signed( -239, 13);
        r( 106) := to_signed(  116, 13);
        r( 107) := to_signed(  192, 13);
        r( 108) := to_signed( -187, 13);
        r( 109) := to_signed( -296, 13);
        r( 110) := to_signed(   63, 13);
        r( 111) := to_signed(  100, 13);
        r( 112) := to_signed(  -54, 13);
        r( 113) := to_signed(  -98, 13);
        r( 114) := to_signed(  169, 13);
        r( 115) := to_signed(  243, 13);
        r( 116) := to_signed(  -69, 13);
        r( 117) := to_signed( -116, 13);
        r( 118) := to_signed(  141, 13);
        r( 119) := to_signed(  195, 13);
        r( 120) := to_signed( -152, 13);
        r( 121) := to_signed( -271, 13);
        r( 122) := to_signed(  108, 13);
        r( 123) := to_signed(  162, 13);
        r( 124) := to_signed( -162, 13);
        r( 125) := to_signed( -277, 13);
        r( 126) := to_signed(   79, 13);
        r( 127) := to_signed(  108, 13);
        r( 128) := to_signed(  -70, 13);
        r( 129) := to_signed( -135, 13);
        r( 130) := to_signed(  160, 13);
        r( 131) := to_signed(  227, 13);
        r( 132) := to_signed(  -56, 13);
        r( 133) := to_signed( -108, 13);
        r( 134) := to_signed(  157, 13);
        r( 135) := to_signed(  215, 13);
        r( 136) := to_signed( -159, 13);
        r( 137) := to_signed( -297, 13);
        r( 138) := to_signed(   96, 13);
        r( 139) := to_signed(  130, 13);
        r( 140) := to_signed( -137, 13);
        r( 141) := to_signed( -245, 13);
        r( 142) := to_signed(   98, 13);
        r( 143) := to_signed(  129, 13);
        r( 144) := to_signed(  -91, 13);
        r( 145) := to_signed( -176, 13);
        r( 146) := to_signed(  141, 13);
        r( 147) := to_signed(  198, 13);
        r( 148) := to_signed(  -44, 13);
        r( 149) := to_signed(  -95, 13);
        r( 150) := to_signed(  171, 13);
        r( 151) := to_signed(  238, 13);
        r( 152) := to_signed( -167, 13);
        r( 153) := to_signed( -314, 13);
        r( 154) := to_signed(   80, 13);
        r( 155) := to_signed(  100, 13);
        r( 156) := to_signed( -110, 13);
        r( 157) := to_signed( -203, 13);
        r( 158) := to_signed(  119, 13);
        r( 159) := to_signed(  160, 13);
        r( 160) := to_signed( -117, 13);
        r( 161) := to_signed( -215, 13);
        r( 162) := to_signed(  110, 13);
        r( 163) := to_signed(  155, 13);
        r( 164) := to_signed(  -33, 13);
        r( 165) := to_signed(  -74, 13);
        r( 166) := to_signed(  180, 13);
        r( 167) := to_signed(  260, 13);
        r( 168) := to_signed( -174, 13);
        r( 169) := to_signed( -318, 13);
        r( 170) := to_signed(   60, 13);
        r( 171) := to_signed(   71, 13);
        r( 172) := to_signed(  -81, 13);
        r( 173) := to_signed( -153, 13);
        r( 174) := to_signed(  138, 13);
        r( 175) := to_signed(  197, 13);
        r( 176) := to_signed( -143, 13);
        r( 177) := to_signed( -249, 13);
        r( 178) := to_signed(   69, 13);
        r( 179) := to_signed(   99, 13);
        r( 180) := to_signed(  -22, 13);
        r( 181) := to_signed(  -49, 13);
        r( 182) := to_signed(  181, 13);
        r( 183) := to_signed(  276, 13);
        r( 184) := to_signed( -177, 13);
        r( 185) := to_signed( -309, 13);
        r( 186) := to_signed(   37, 13);
        r( 187) := to_signed(   44, 13);
        r( 188) := to_signed(  -51, 13);
        r( 189) := to_signed(  -95, 13);
        r( 190) := to_signed(  154, 13);
        r( 191) := to_signed(  235, 13);
        r( 192) := to_signed( -165, 13);
        r( 193) := to_signed( -275, 13);
        r( 194) := to_signed(   22, 13);
        r( 195) := to_signed(   34, 13);
        r( 196) := to_signed(  -10, 13);
        r( 197) := to_signed(  -19, 13);
        r( 198) := to_signed(  174, 13);
        r( 199) := to_signed(  281, 13);
        r( 200) := to_signed( -174, 13);
        r( 201) := to_signed( -290, 13);
        r( 202) := to_signed(   14, 13);
        r( 203) := to_signed(   20, 13);
        r( 204) := to_signed(  -18, 13);
        r( 205) := to_signed(  -30, 13);
        r( 206) := to_signed(  167, 13);
        r( 207) := to_signed(  270, 13);
        r( 208) := to_signed( -183, 13);
        r( 209) := to_signed( -293, 13);
        r( 210) := to_signed(  -27, 13);
        r( 211) := to_signed(  -34, 13);
        r( 212) := to_signed(    1, 13);
        r( 213) := to_signed(   12, 13);
        r( 214) := to_signed(  157, 13);
        r( 215) := to_signed(  270, 13);
        r( 216) := to_signed( -164, 13);
        r( 217) := to_signed( -260, 13);
        r( 218) := to_signed(   -8, 13);
        r( 219) := to_signed(   -2, 13);
        r( 220) := to_signed(   18, 13);
        r( 221) := to_signed(   40, 13);
        r( 222) := to_signed(  174, 13);
        r( 223) := to_signed(  297, 13);
        r( 224) := to_signed( -193, 13);
        r( 225) := to_signed( -302, 13);
        r( 226) := to_signed(  -72, 13);
        r( 227) := to_signed( -101, 13);
        r( 228) := to_signed(   12, 13);
        r( 229) := to_signed(   40, 13);
        r( 230) := to_signed(  131, 13);
        r( 231) := to_signed(  239, 13);
        r( 232) := to_signed( -145, 13);
        r( 233) := to_signed( -221, 13);
        r( 234) := to_signed(  -25, 13);
        r( 235) := to_signed(  -21, 13);
        r( 236) := to_signed(   57, 13);
        r( 237) := to_signed(  113, 13);
        r( 238) := to_signed(  176, 13);
        r( 239) := to_signed(  310, 13);
        r( 240) := to_signed( -196, 13);
        r( 241) := to_signed( -306, 13);
        r( 242) := to_signed( -111, 13);
        r( 243) := to_signed( -160, 13);
        r( 244) := to_signed(   22, 13);
        r( 245) := to_signed(   61, 13);
        r( 246) := to_signed(  100, 13);
        r( 247) := to_signed(  189, 13);
        r( 248) := to_signed( -119, 13);
        r( 249) := to_signed( -175, 13);
        r( 250) := to_signed(  -36, 13);
        r( 251) := to_signed(  -36, 13);
        r( 252) := to_signed(   96, 13);
        r( 253) := to_signed(  184, 13);
        r( 254) := to_signed(  173, 13);
        r( 255) := to_signed(  307, 13);
        r( 256) := to_signed( -191, 13);
        r( 257) := to_signed( -304, 13);
        r( 258) := to_signed( -140, 13);
        r( 259) := to_signed( -210, 13);
        r( 260) := to_signed(   29, 13);
        r( 261) := to_signed(   69, 13);
        r( 262) := to_signed(   67, 13);
        r( 263) := to_signed(  126, 13);
        r( 264) := to_signed(  -84, 13);
        r( 265) := to_signed( -121, 13);
        r( 266) := to_signed(  -39, 13);
        r( 267) := to_signed(  -45, 13);
        r( 268) := to_signed(  136, 13);
        r( 269) := to_signed(  249, 13);
        r( 270) := to_signed(  167, 13);
        r( 271) := to_signed(  288, 13);
        r( 272) := to_signed( -181, 13);
        r( 273) := to_signed( -299, 13);
        r( 274) := to_signed( -160, 13);
        r( 275) := to_signed( -248, 13);
        r( 276) := to_signed(   33, 13);
        r( 277) := to_signed(   62, 13);
        r( 278) := to_signed(   32, 13);
        r( 279) := to_signed(   57, 13);
        r( 280) := to_signed(  -41, 13);
        r( 281) := to_signed(  -62, 13);
        r( 282) := to_signed(  -34, 13);
        r( 283) := to_signed(  -48, 13);
        r( 284) := to_signed(  172, 13);
        r( 285) := to_signed(  299, 13);
        r( 286) := to_signed(  158, 13);
        r( 287) := to_signed(  257, 13);
        r( 288) := to_signed( -165, 13);
        r( 289) := to_signed( -292, 13);
        r( 290) := to_signed( -170, 13);
        r( 291) := to_signed( -273, 13);
        r( 292) := to_signed(   31, 13);
        r( 293) := to_signed(   39, 13);
        r( 294) := to_signed(   -1, 13);
        r( 295) := to_signed(  -10, 13);
        r( 296) := to_signed(    8, 13);
        r( 297) := to_signed(    1, 13);
        r( 298) := to_signed(  -21, 13);
        r( 299) := to_signed(  -42, 13);
        r( 300) := to_signed(  202, 13);
        r( 301) := to_signed(  328, 13);
        r( 302) := to_signed(  148, 13);
        r( 303) := to_signed(  218, 13);
        r( 304) := to_signed( -146, 13);
        r( 305) := to_signed( -282, 13);
        r( 306) := to_signed( -172, 13);
        r( 307) := to_signed( -285, 13);
        r( 308) := to_signed(   21, 13);
        r( 309) := to_signed(    2, 13);
        r( 310) := to_signed(  -31, 13);
        r( 311) := to_signed(  -71, 13);
        r( 312) := to_signed(   60, 13);
        r( 313) := to_signed(   65, 13);
        r( 314) := to_signed(   -1, 13);
        r( 315) := to_signed(  -28, 13);
        r( 316) := to_signed(  222, 13);
        r( 317) := to_signed(  334, 13);
        r( 318) := to_signed(  136, 13);
        r( 319) := to_signed(  178, 13);
        r( 320) := to_signed( -127, 13);
        r( 321) := to_signed( -268, 13);
        r( 322) := to_signed( -167, 13);
        r( 323) := to_signed( -285, 13);
        r( 324) := to_signed(    5, 13);
        r( 325) := to_signed(  -44, 13);
        r( 326) := to_signed(  -59, 13);
        r( 327) := to_signed( -122, 13);
        r( 328) := to_signed(  110, 13);
        r( 329) := to_signed(  128, 13);
        r( 330) := to_signed(   23, 13);
        r( 331) := to_signed(   -6, 13);
        r( 332) := to_signed(  230, 13);
        r( 333) := to_signed(  320, 13);
        r( 334) := to_signed(  123, 13);
        r( 335) := to_signed(  141, 13);
        r( 336) := to_signed( -109, 13);
        r( 337) := to_signed( -249, 13);
        r( 338) := to_signed( -156, 13);
        r( 339) := to_signed( -272, 13);
        r( 340) := to_signed(  -20, 13);
        r( 341) := to_signed(  -95, 13);
        r( 342) := to_signed(  -84, 13);
        r( 343) := to_signed( -162, 13);
        r( 344) := to_signed(  155, 13);
        r( 345) := to_signed(  185, 13);
        r( 346) := to_signed(   49, 13);
        r( 347) := to_signed(   24, 13);
        r( 348) := to_signed(  225, 13);
        r( 349) := to_signed(  291, 13);
        r( 350) := to_signed(  109, 13);
        r( 351) := to_signed(  110, 13);
        r( 352) := to_signed(  -94, 13);
        r( 353) := to_signed( -224, 13);
        r( 354) := to_signed( -143, 13);
        r( 355) := to_signed( -247, 13);
        r( 356) := to_signed(  -52, 13);
        r( 357) := to_signed( -143, 13);
        r( 358) := to_signed( -108, 13);
        r( 359) := to_signed( -190, 13);
        r( 360) := to_signed(  189, 13);
        r( 361) := to_signed(  234, 13);
        r( 362) := to_signed(   73, 13);
        r( 363) := to_signed(   58, 13);
        r( 364) := to_signed(  207, 13);
        r( 365) := to_signed(  256, 13);
        r( 366) := to_signed(   94, 13);
        r( 367) := to_signed(   87, 13);
        r( 368) := to_signed(  -85, 13);
        r( 369) := to_signed( -189, 13);
        r( 370) := to_signed( -130, 13);
        r( 371) := to_signed( -211, 13);
        r( 372) := to_signed(  -91, 13);
        r( 373) := to_signed( -182, 13);
        r( 374) := to_signed( -131, 13);
        r( 375) := to_signed( -205, 13);
        r( 376) := to_signed(  210, 13);
        r( 377) := to_signed(  274, 13);
        r( 378) := to_signed(   91, 13);
        r( 379) := to_signed(   92, 13);
        r( 380) := to_signed(  180, 13);
        r( 381) := to_signed(  222, 13);
        r( 382) := to_signed(   76, 13);
        r( 383) := to_signed(   72, 13);
        r( 384) := to_signed(  -83, 13);
        r( 385) := to_signed( -142, 13);
        r( 386) := to_signed( -120, 13);
        r( 387) := to_signed( -168, 13);
        r( 388) := to_signed( -133, 13);
        r( 389) := to_signed( -208, 13);
        r( 390) := to_signed( -154, 13);
        r( 391) := to_signed( -208, 13);
        r( 392) := to_signed(  218, 13);
        r( 393) := to_signed(  304, 13);
        r( 394) := to_signed(  101, 13);
        r( 395) := to_signed(  122, 13);
        r( 396) := to_signed(  147, 13);
        r( 397) := to_signed(  195, 13);
        r( 398) := to_signed(   56, 13);
        r( 399) := to_signed(   63, 13);
        r( 400) := to_signed(  -86, 13);
        r( 401) := to_signed(  -85, 13);
        r( 402) := to_signed( -116, 13);
        r( 403) := to_signed( -124, 13);
        r( 404) := to_signed( -176, 13);
        r( 405) := to_signed( -213, 13);
        r( 406) := to_signed( -177, 13);
        r( 407) := to_signed( -200, 13);
        r( 408) := to_signed(  214, 13);
        r( 409) := to_signed(  325, 13);
        r( 410) := to_signed(  101, 13);
        r( 411) := to_signed(  145, 13);
        r( 412) := to_signed(  112, 13);
        r( 413) := to_signed(  177, 13);
        r( 414) := to_signed(   33, 13);
        r( 415) := to_signed(   59, 13);
        r( 416) := to_signed(  -93, 13);
        r( 417) := to_signed(  -20, 13);
        r( 418) := to_signed( -119, 13);
        r( 419) := to_signed(  -83, 13);
        r( 420) := to_signed( -217, 13);
        r( 421) := to_signed( -193, 13);
        r( 422) := to_signed( -201, 13);
        r( 423) := to_signed( -183, 13);
        r( 424) := to_signed(  202, 13);
        r( 425) := to_signed(  336, 13);
        r( 426) := to_signed(   90, 13);
        r( 427) := to_signed(  157, 13);
        r( 428) := to_signed(   79, 13);
        r( 429) := to_signed(  170, 13);
        r( 430) := to_signed(    8, 13);
        r( 431) := to_signed(   57, 13);
        r( 432) := to_signed( -102, 13);
        r( 433) := to_signed(   46, 13);
        r( 434) := to_signed( -131, 13);
        r( 435) := to_signed(  -50, 13);
        r( 436) := to_signed( -250, 13);
        r( 437) := to_signed( -147, 13);
        r( 438) := to_signed( -225, 13);
        r( 439) := to_signed( -161, 13);
        r( 440) := to_signed(  186, 13);
        r( 441) := to_signed(  339, 13);
        r( 442) := to_signed(   71, 13);
        r( 443) := to_signed(  158, 13);
        r( 444) := to_signed(   50, 13);
        r( 445) := to_signed(  171, 13);
        r( 446) := to_signed(  -19, 13);
        r( 447) := to_signed(   56, 13);
        r( 448) := to_signed( -108, 13);
        r( 449) := to_signed(  106, 13);
        r( 450) := to_signed( -152, 13);
        r( 451) := to_signed(  -29, 13);
        r( 452) := to_signed( -265, 13);
        r( 453) := to_signed(  -83, 13);
        r( 454) := to_signed( -248, 13);
        r( 455) := to_signed( -136, 13);
        r( 456) := to_signed(  167, 13);
        r( 457) := to_signed(  333, 13);
        r( 458) := to_signed(   42, 13);
        r( 459) := to_signed(  149, 13);
        r( 460) := to_signed(   31, 13);
        r( 461) := to_signed(  178, 13);
        r( 462) := to_signed(  -46, 13);
        r( 463) := to_signed(   52, 13);
        r( 464) := to_signed( -106, 13);
        r( 465) := to_signed(  153, 13);
        r( 466) := to_signed( -182, 13);
        r( 467) := to_signed(  -21, 13);
        r( 468) := to_signed( -253, 13);
        r( 469) := to_signed(  -12, 13);
        r( 470) := to_signed( -268, 13);
        r( 471) := to_signed( -114, 13);
        r( 472) := to_signed(  151, 13);
        r( 473) := to_signed(  321, 13);
        r( 474) := to_signed(    7, 13);
        r( 475) := to_signed(  129, 13);
        r( 476) := to_signed(   24, 13);
        r( 477) := to_signed(  187, 13);
        r( 478) := to_signed(  -73, 13);
        r( 479) := to_signed(   45, 13);
        r( 480) := to_signed(  -90, 13);
        r( 481) := to_signed(  184, 13);
        r( 482) := to_signed( -218, 13);
        r( 483) := to_signed(  -26, 13);
        r( 484) := to_signed( -209, 13);
        r( 485) := to_signed(   52, 13);
        r( 486) := to_signed( -281, 13);
        r( 487) := to_signed(  -97, 13);
        r( 488) := to_signed(  139, 13);
        r( 489) := to_signed(  303, 13);
        r( 490) := to_signed(  -34, 13);
        r( 491) := to_signed(  101, 13);
        r( 492) := to_signed(   30, 13);
        r( 493) := to_signed(  194, 13);
        r( 494) := to_signed(  -98, 13);
        r( 495) := to_signed(   34, 13);
        r( 496) := to_signed(  -56, 13);
        r( 497) := to_signed(  199, 13);
        r( 498) := to_signed( -259, 13);
        r( 499) := to_signed(  -42, 13);
        r( 500) := to_signed( -133, 13);
        r( 501) := to_signed(  101, 13);
        r( 502) := to_signed( -282, 13);
        r( 503) := to_signed(  -86, 13);
        r( 504) := to_signed(  134, 13);
        r( 505) := to_signed(  281, 13);
        r( 506) := to_signed(  -79, 13);
        r( 507) := to_signed(   65, 13);
        r( 508) := to_signed(   50, 13);
        r( 509) := to_signed(  196, 13);
        r( 510) := to_signed( -117, 13);
        r( 511) := to_signed(   20, 13);
        r( 512) := to_signed(   -4, 13);
        r( 513) := to_signed(  198, 13);
        r( 514) := to_signed( -297, 13);
        r( 515) := to_signed(  -69, 13);
        r( 516) := to_signed(  -42, 13);
        r( 517) := to_signed(  132, 13);
        r( 518) := to_signed( -267, 13);
        r( 519) := to_signed(  -81, 13);
        r( 520) := to_signed(  136, 13);
        r( 521) := to_signed(  253, 13);
        r( 522) := to_signed( -126, 13);
        r( 523) := to_signed(   23, 13);
        r( 524) := to_signed(   80, 13);
        r( 525) := to_signed(  192, 13);
        r( 526) := to_signed( -129, 13);
        r( 527) := to_signed(    1, 13);
        r( 528) := to_signed(   59, 13);
        r( 529) := to_signed(  186, 13);
        r( 530) := to_signed( -323, 13);
        r( 531) := to_signed( -105, 13);
        r( 532) := to_signed(   45, 13);
        r( 533) := to_signed(  147, 13);
        r( 534) := to_signed( -236, 13);
        r( 535) := to_signed(  -82, 13);
        r( 536) := to_signed(  148, 13);
        r( 537) := to_signed(  222, 13);
        r( 538) := to_signed( -171, 13);
        r( 539) := to_signed(  -27, 13);
        r( 540) := to_signed(  114, 13);
        r( 541) := to_signed(  182, 13);
        r( 542) := to_signed( -131, 13);
        r( 543) := to_signed(  -20, 13);
        r( 544) := to_signed(  122, 13);
        r( 545) := to_signed(  163, 13);
        r( 546) := to_signed( -326, 13);
        r( 547) := to_signed( -147, 13);
        r( 548) := to_signed(  115, 13);
        r( 549) := to_signed(  146, 13);
        r( 550) := to_signed( -190, 13);
        r( 551) := to_signed(  -87, 13);
        r( 552) := to_signed(  166, 13);
        r( 553) := to_signed(  186, 13);
        r( 554) := to_signed( -204, 13);
        r( 555) := to_signed(  -84, 13);
        r( 556) := to_signed(  146, 13);
        r( 557) := to_signed(  166, 13);
        r( 558) := to_signed( -120, 13);
        r( 559) := to_signed(  -44, 13);
        r( 560) := to_signed(  175, 13);
        r( 561) := to_signed(  130, 13);
        r( 562) := to_signed( -296, 13);
        r( 563) := to_signed( -191, 13);
        r( 564) := to_signed(  163, 13);
        r( 565) := to_signed(  135, 13);
        r( 566) := to_signed( -134, 13);
        r( 567) := to_signed(  -95, 13);
        r( 568) := to_signed(  189, 13);
        r( 569) := to_signed(  145, 13);
        r( 570) := to_signed( -218, 13);
        r( 571) := to_signed( -145, 13);
        r( 572) := to_signed(  173, 13);
        r( 573) := to_signed(  145, 13);
        r( 574) := to_signed(  -97, 13);
        r( 575) := to_signed(  -69, 13);
        r( 576) := to_signed(  213, 13);
        r( 577) := to_signed(   89, 13);
        r( 578) := to_signed( -234, 13);
        r( 579) := to_signed( -232, 13);
        r( 580) := to_signed(  192, 13);
        r( 581) := to_signed(  115, 13);
        r( 582) := to_signed(  -78, 13);
        r( 583) := to_signed( -104, 13);
        r( 584) := to_signed(  213, 13);
        r( 585) := to_signed(   96, 13);
        r( 586) := to_signed( -202, 13);
        r( 587) := to_signed( -208, 13);
        r( 588) := to_signed(  193, 13);
        r( 589) := to_signed(  118, 13);
        r( 590) := to_signed(  -65, 13);
        r( 591) := to_signed(  -93, 13);
        r( 592) := to_signed(  235, 13);
        r( 593) := to_signed(   41, 13);
        r( 594) := to_signed( -153, 13);
        r( 595) := to_signed( -261, 13);
        r( 596) := to_signed(  206, 13);
        r( 597) := to_signed(   86, 13);
        r( 598) := to_signed(  -28, 13);
        r( 599) := to_signed( -114, 13);
        r( 600) := to_signed(  235, 13);
        r( 601) := to_signed(   40, 13);
        r( 602) := to_signed( -155, 13);
        r( 603) := to_signed( -263, 13);
        r( 604) := to_signed(  206, 13);
        r( 605) := to_signed(   86, 13);
        r( 606) := to_signed(  -28, 13);
        r( 607) := to_signed( -114, 13);
        r( 608) := to_signed(  243, 13);
        r( 609) := to_signed(  -15, 13);
        r( 610) := to_signed(  -71, 13);
        r( 611) := to_signed( -272, 13);
        r( 612) := to_signed(  207, 13);
        r( 613) := to_signed(   49, 13);
        r( 614) := to_signed(   14, 13);
        r( 615) := to_signed( -121, 13);
        r( 616) := to_signed(  255, 13);
        r( 617) := to_signed(  -25, 13);
        r( 618) := to_signed(  -85, 13);
        r( 619) := to_signed( -298, 13);
        r( 620) := to_signed(  212, 13);
        r( 621) := to_signed(   46, 13);
        r( 622) := to_signed(   10, 13);
        r( 623) := to_signed( -131, 13);
        r( 624) := to_signed(  241, 13);
        r( 625) := to_signed(  -75, 13);
        r( 626) := to_signed(   -1, 13);
        r( 627) := to_signed( -260, 13);
        r( 628) := to_signed(  200, 13);
        r( 629) := to_signed(    5, 13);
        r( 630) := to_signed(   46, 13);
        r( 631) := to_signed( -125, 13);
        r( 632) := to_signed(  270, 13);
        r( 633) := to_signed(  -96, 13);
        r( 634) := to_signed(   -9, 13);
        r( 635) := to_signed( -301, 13);
        r( 636) := to_signed(  212, 13);
        r( 637) := to_signed(    0, 13);
        r( 638) := to_signed(   45, 13);
        r( 639) := to_signed( -140, 13);
        r( 640) := to_signed(  229, 13);
        r( 641) := to_signed( -133, 13);
        r( 642) := to_signed(   54, 13);
        r( 643) := to_signed( -229, 13);
        r( 644) := to_signed(  184, 13);
        r( 645) := to_signed(  -46, 13);
        r( 646) := to_signed(   69, 13);
        r( 647) := to_signed( -126, 13);
        r( 648) := to_signed(  279, 13);
        r( 649) := to_signed( -163, 13);
        r( 650) := to_signed(   63, 13);
        r( 651) := to_signed( -268, 13);
        r( 652) := to_signed(  208, 13);
        r( 653) := to_signed(  -52, 13);
        r( 654) := to_signed(   76, 13);
        r( 655) := to_signed( -140, 13);
        r( 656) := to_signed(  210, 13);
        r( 657) := to_signed( -182, 13);
        r( 658) := to_signed(   94, 13);
        r( 659) := to_signed( -188, 13);
        r( 660) := to_signed(  160, 13);
        r( 661) := to_signed( -103, 13);
        r( 662) := to_signed(   83, 13);
        r( 663) := to_signed( -125, 13);
        r( 664) := to_signed(  283, 13);
        r( 665) := to_signed( -211, 13);
        r( 666) := to_signed(  123, 13);
        r( 667) := to_signed( -208, 13);
        r( 668) := to_signed(  198, 13);
        r( 669) := to_signed( -108, 13);
        r( 670) := to_signed(  103, 13);
        r( 671) := to_signed( -131, 13);
        r( 672) := to_signed(  183, 13);
        r( 673) := to_signed( -213, 13);
        r( 674) := to_signed(  121, 13);
        r( 675) := to_signed( -143, 13);
        r( 676) := to_signed(  126, 13);
        r( 677) := to_signed( -163, 13);
        r( 678) := to_signed(   91, 13);
        r( 679) := to_signed( -121, 13);
        r( 680) := to_signed(  281, 13);
        r( 681) := to_signed( -226, 13);
        r( 682) := to_signed(  170, 13);
        r( 683) := to_signed( -139, 13);
        r( 684) := to_signed(  182, 13);
        r( 685) := to_signed( -162, 13);
        r( 686) := to_signed(  125, 13);
        r( 687) := to_signed( -114, 13);
        r( 688) := to_signed(  150, 13);
        r( 689) := to_signed( -225, 13);
        r( 690) := to_signed(  138, 13);
        r( 691) := to_signed( -103, 13);
        r( 692) := to_signed(   80, 13);
        r( 693) := to_signed( -220, 13);
        r( 694) := to_signed(   92, 13);
        r( 695) := to_signed( -116, 13);
        r( 696) := to_signed(  272, 13);
        r( 697) := to_signed( -204, 13);
        r( 698) := to_signed(  207, 13);
        r( 699) := to_signed(  -72, 13);
        r( 700) := to_signed(  159, 13);
        r( 701) := to_signed( -203, 13);
        r( 702) := to_signed(  144, 13);
        r( 703) := to_signed(  -91, 13);
        r( 704) := to_signed(  109, 13);
        r( 705) := to_signed( -220, 13);
        r( 706) := to_signed(  148, 13);
        r( 707) := to_signed(  -68, 13);
        r( 708) := to_signed(   19, 13);
        r( 709) := to_signed( -266, 13);
        r( 710) := to_signed(   87, 13);
        r( 711) := to_signed( -110, 13);
        r( 712) := to_signed(  257, 13);
        r( 713) := to_signed( -161, 13);
        r( 714) := to_signed(  237, 13);
        r( 715) := to_signed(  -14, 13);
        r( 716) := to_signed(  128, 13);
        r( 717) := to_signed( -222, 13);
        r( 718) := to_signed(  161, 13);
        r( 719) := to_signed(  -65, 13);
        r( 720) := to_signed(   63, 13);
        r( 721) := to_signed( -203, 13);
        r( 722) := to_signed(  152, 13);
        r( 723) := to_signed(  -41, 13);
        r( 724) := to_signed(  -57, 13);
        r( 725) := to_signed( -294, 13);
        r( 726) := to_signed(   76, 13);
        r( 727) := to_signed( -104, 13);
        r( 728) := to_signed(  236, 13);
        r( 729) := to_signed( -111, 13);
        r( 730) := to_signed(  261, 13);
        r( 731) := to_signed(   34, 13);
        r( 732) := to_signed(   89, 13);
        r( 733) := to_signed( -216, 13);
        r( 734) := to_signed(  175, 13);
        r( 735) := to_signed(  -36, 13);
        r( 736) := to_signed(   14, 13);
        r( 737) := to_signed( -183, 13);
        r( 738) := to_signed(  150, 13);
        r( 739) := to_signed(  -20, 13);
        r( 740) := to_signed( -144, 13);
        r( 741) := to_signed( -303, 13);
        r( 742) := to_signed(   57, 13);
        r( 743) := to_signed(  -99, 13);
        r( 744) := to_signed(  210, 13);
        r( 745) := to_signed(  -65, 13);
        r( 746) := to_signed(  282, 13);
        r( 747) := to_signed(   73, 13);
        r( 748) := to_signed(   48, 13);
        r( 749) := to_signed( -189, 13);
        r( 750) := to_signed(  188, 13);
        r( 751) := to_signed(   -8, 13);
        r( 752) := to_signed(  -36, 13);
        r( 753) := to_signed( -163, 13);
        r( 754) := to_signed(  143, 13);
        r( 755) := to_signed(   -5, 13);
        r( 756) := to_signed( -229, 13);
        r( 757) := to_signed( -295, 13);
        r( 758) := to_signed(   31, 13);
        r( 759) := to_signed(  -94, 13);
        r( 760) := to_signed(  183, 13);
        r( 761) := to_signed(  -27, 13);
        r( 762) := to_signed(  300, 13);
        r( 763) := to_signed(  105, 13);
        r( 764) := to_signed(   11, 13);
        r( 765) := to_signed( -152, 13);
        r( 766) := to_signed(  199, 13);
        r( 767) := to_signed(   20, 13);
        r( 768) := to_signed(  -83, 13);
        r( 769) := to_signed( -146, 13);
        r( 770) := to_signed(  128, 13);
        r( 771) := to_signed(    7, 13);
        r( 772) := to_signed( -294, 13);
        r( 773) := to_signed( -274, 13);
        r( 774) := to_signed(   -4, 13);
        r( 775) := to_signed(  -89, 13);
        r( 776) := to_signed(  156, 13);
        r( 777) := to_signed(    3, 13);
        r( 778) := to_signed(  316, 13);
        r( 779) := to_signed(  133, 13);
        r( 780) := to_signed(  -15, 13);
        r( 781) := to_signed( -112, 13);
        r( 782) := to_signed(  207, 13);
        r( 783) := to_signed(   48, 13);
        r( 784) := to_signed( -127, 13);
        r( 785) := to_signed( -132, 13);
        r( 786) := to_signed(  106, 13);
        r( 787) := to_signed(   15, 13);
        r( 788) := to_signed( -328, 13);
        r( 789) := to_signed( -248, 13);
        r( 790) := to_signed(  -45, 13);
        r( 791) := to_signed(  -83, 13);
        r( 792) := to_signed(  130, 13);
        r( 793) := to_signed(   26, 13);
        r( 794) := to_signed(  328, 13);
        r( 795) := to_signed(  159, 13);
        r( 796) := to_signed(  -29, 13);
        r( 797) := to_signed(  -74, 13);
        r( 798) := to_signed(  209, 13);
        r( 799) := to_signed(   75, 13);
        r( 800) := to_signed( -165, 13);
        r( 801) := to_signed( -122, 13);
        r( 802) := to_signed(   74, 13);
        r( 803) := to_signed(   22, 13);
        r( 804) := to_signed( -333, 13);
        r( 805) := to_signed( -220, 13);
        r( 806) := to_signed(  -86, 13);
        r( 807) := to_signed(  -73, 13);
        r( 808) := to_signed(  104, 13);
        r( 809) := to_signed(   45, 13);
        r( 810) := to_signed(  334, 13);
        r( 811) := to_signed(  185, 13);
        r( 812) := to_signed(  -34, 13);
        r( 813) := to_signed(  -42, 13);
        r( 814) := to_signed(  203, 13);
        r( 815) := to_signed(  101, 13);
        r( 816) := to_signed( -198, 13);
        r( 817) := to_signed( -114, 13);
        r( 818) := to_signed(   32, 13);
        r( 819) := to_signed(   29, 13);
        r( 820) := to_signed( -319, 13);
        r( 821) := to_signed( -191, 13);
        r( 822) := to_signed( -121, 13);
        r( 823) := to_signed(  -61, 13);
        r( 824) := to_signed(   79, 13);
        r( 825) := to_signed(   60, 13);
        r( 826) := to_signed(  332, 13);
        r( 827) := to_signed(  211, 13);
        r( 828) := to_signed(  -36, 13);
        r( 829) := to_signed(  -15, 13);
        r( 830) := to_signed(  189, 13);
        r( 831) := to_signed(  125, 13);
        r( 832) := to_signed( -228, 13);
        r( 833) := to_signed( -108, 13);
        r( 834) := to_signed(  -18, 13);
        r( 835) := to_signed(   37, 13);
        r( 836) := to_signed( -298, 13);
        r( 837) := to_signed( -165, 13);
        r( 838) := to_signed( -141, 13);
        r( 839) := to_signed(  -44, 13);
        r( 840) := to_signed(   54, 13);
        r( 841) := to_signed(   73, 13);
        r( 842) := to_signed(  319, 13);
        r( 843) := to_signed(  239, 13);
        r( 844) := to_signed(  -37, 13);
        r( 845) := to_signed(    7, 13);
        r( 846) := to_signed(  168, 13);
        r( 847) := to_signed(  148, 13);
        r( 848) := to_signed( -254, 13);
        r( 849) := to_signed( -102, 13);
        r( 850) := to_signed(  -70, 13);
        r( 851) := to_signed(   49, 13);
        r( 852) := to_signed( -276, 13);
        r( 853) := to_signed( -140, 13);
        r( 854) := to_signed( -148, 13);
        r( 855) := to_signed(  -23, 13);
        r( 856) := to_signed(   28, 13);
        r( 857) := to_signed(   85, 13);
        r( 858) := to_signed(  292, 13);
        r( 859) := to_signed(  267, 13);
        r( 860) := to_signed(  -41, 13);
        r( 861) := to_signed(   26, 13);
        r( 862) := to_signed(  142, 13);
        r( 863) := to_signed(  167, 13);
        r( 864) := to_signed( -277, 13);
        r( 865) := to_signed(  -94, 13);
        r( 866) := to_signed( -115, 13);
        r( 867) := to_signed(   67, 13);
        r( 868) := to_signed( -258, 13);
        r( 869) := to_signed( -116, 13);
        r( 870) := to_signed( -145, 13);
        r( 871) := to_signed(    0, 13);
        r( 872) := to_signed(    0, 13);
        r( 873) := to_signed(   97, 13);
        r( 874) := to_signed(  253, 13);
        r( 875) := to_signed(  295, 13);
        r( 876) := to_signed(  -48, 13);
        r( 877) := to_signed(   41, 13);
        r( 878) := to_signed(  114, 13);
        r( 879) := to_signed(  182, 13);
        r( 880) := to_signed( -296, 13);
        r( 881) := to_signed(  -82, 13);
        r( 882) := to_signed( -145, 13);
        r( 883) := to_signed(   92, 13);
        r( 884) := to_signed( -243, 13);
        r( 885) := to_signed(  -93, 13);
        r( 886) := to_signed( -137, 13);
        r( 887) := to_signed(   24, 13);
        r( 888) := to_signed(  -31, 13);
        r( 889) := to_signed(  110, 13);
        r( 890) := to_signed(  204, 13);
        r( 891) := to_signed(  320, 13);
        r( 892) := to_signed(  -58, 13);
        r( 893) := to_signed(   53, 13);
        r( 894) := to_signed(   85, 13);
        r( 895) := to_signed(  191, 13);
        r( 896) := to_signed( -311, 13);
        r( 897) := to_signed(  -62, 13);
        r( 898) := to_signed( -157, 13);
        r( 899) := to_signed(  121, 13);
        r( 900) := to_signed( -232, 13);
        r( 901) := to_signed(  -70, 13);
        r( 902) := to_signed( -128, 13);
        r( 903) := to_signed(   45, 13);
        r( 904) := to_signed(  -63, 13);
        r( 905) := to_signed(  125, 13);
        r( 906) := to_signed(  150, 13);
        r( 907) := to_signed(  337, 13);
        r( 908) := to_signed(  -72, 13);
        r( 909) := to_signed(   62, 13);
        r( 910) := to_signed(   56, 13);
        r( 911) := to_signed(  193, 13);
        r( 912) := to_signed( -319, 13);
        r( 913) := to_signed(  -34, 13);
        r( 914) := to_signed( -155, 13);
        r( 915) := to_signed(  151, 13);
        r( 916) := to_signed( -225, 13);
        r( 917) := to_signed(  -48, 13);
        r( 918) := to_signed( -121, 13);
        r( 919) := to_signed(   64, 13);
        r( 920) := to_signed(  -98, 13);
        r( 921) := to_signed(  141, 13);
        r( 922) := to_signed(   97, 13);
        r( 923) := to_signed(  343, 13);
        r( 924) := to_signed(  -89, 13);
        r( 925) := to_signed(   69, 13);
        r( 926) := to_signed(   26, 13);
        r( 927) := to_signed(  189, 13);
        r( 928) := to_signed( -318, 13);
        r( 929) := to_signed(    5, 13);
        r( 930) := to_signed( -142, 13);
        r( 931) := to_signed(  180, 13);
        r( 932) := to_signed( -220, 13);
        r( 933) := to_signed(  -26, 13);
        r( 934) := to_signed( -115, 13);
        r( 935) := to_signed(   79, 13);
        r( 936) := to_signed( -133, 13);
        r( 937) := to_signed(  159, 13);
        r( 938) := to_signed(   48, 13);
        r( 939) := to_signed(  337, 13);
        r( 940) := to_signed( -111, 13);
        r( 941) := to_signed(   72, 13);
        r( 942) := to_signed(   -3, 13);
        r( 943) := to_signed(  179, 13);
        r( 944) := to_signed( -306, 13);
        r( 945) := to_signed(   53, 13);
        r( 946) := to_signed( -123, 13);
        r( 947) := to_signed(  203, 13);
        r( 948) := to_signed( -217, 13);
        r( 949) := to_signed(   -4, 13);
        r( 950) := to_signed( -110, 13);
        r( 951) := to_signed(   91, 13);
        r( 952) := to_signed( -166, 13);
        r( 953) := to_signed(  176, 13);
        r( 954) := to_signed(    7, 13);
        r( 955) := to_signed(  318, 13);
        r( 956) := to_signed( -136, 13);
        r( 957) := to_signed(   71, 13);
        r( 958) := to_signed(  -33, 13);
        r( 959) := to_signed(  163, 13);
        r( 960) := to_signed( -282, 13);
        r( 961) := to_signed(  107, 13);
        r( 962) := to_signed(  -99, 13);
        r( 963) := to_signed(  222, 13);
        r( 964) := to_signed( -214, 13);
        r( 965) := to_signed(   17, 13);
        r( 966) := to_signed( -106, 13);
        r( 967) := to_signed(   98, 13);
        r( 968) := to_signed( -195, 13);
        r( 969) := to_signed(  189, 13);
        r( 970) := to_signed(  -25, 13);
        r( 971) := to_signed(  291, 13);
        r( 972) := to_signed( -163, 13);
        r( 973) := to_signed(   65, 13);
        r( 974) := to_signed(  -61, 13);
        r( 975) := to_signed(  142, 13);
        r( 976) := to_signed( -247, 13);
        r( 977) := to_signed(  163, 13);
        r( 978) := to_signed(  -72, 13);
        r( 979) := to_signed(  235, 13);
        r( 980) := to_signed( -211, 13);
        r( 981) := to_signed(   36, 13);
        r( 982) := to_signed( -102, 13);
        r( 983) := to_signed(  103, 13);
        r( 984) := to_signed( -216, 13);
        r( 985) := to_signed(  194, 13);
        r( 986) := to_signed(  -48, 13);
        r( 987) := to_signed(  258, 13);
        r( 988) := to_signed( -193, 13);
        r( 989) := to_signed(   54, 13);
        r( 990) := to_signed(  -87, 13);
        r( 991) := to_signed(  119, 13);
        r( 992) := to_signed( -202, 13);
        r( 993) := to_signed(  216, 13);
        r( 994) := to_signed(  -44, 13);
        r( 995) := to_signed(  244, 13);
        r( 996) := to_signed( -206, 13);
        r( 997) := to_signed(   53, 13);
        r( 998) := to_signed(  -97, 13);
        r( 999) := to_signed(  106, 13);
        r(1000) := to_signed( -228, 13);
        r(1001) := to_signed(  188, 13);
        r(1002) := to_signed(  -63, 13);
        r(1003) := to_signed(  225, 13);
        r(1004) := to_signed( -222, 13);
        r(1005) := to_signed(   37, 13);
        r(1006) := to_signed( -109, 13);
        r(1007) := to_signed(   93, 13);
        r(1008) := to_signed( -152, 13);
        r(1009) := to_signed(  260, 13);
        r(1010) := to_signed(  -15, 13);
        r(1011) := to_signed(  251, 13);
        r(1012) := to_signed( -200, 13);
        r(1013) := to_signed(   67, 13);
        r(1014) := to_signed(  -89, 13);
        r(1015) := to_signed(  109, 13);
        r(1016) := to_signed( -232, 13);
        r(1017) := to_signed(  172, 13);
        r(1018) := to_signed(  -70, 13);
        r(1019) := to_signed(  193, 13);
        r(1020) := to_signed( -250, 13);
        r(1021) := to_signed(   14, 13);
        r(1022) := to_signed( -128, 13);
        r(1023) := to_signed(   68, 13);
        return r;
    end function;
    constant C_VERT_SY : t_vert_sy := gen_vert_sy;

    type t_cell_sx is array(0 to 512-1) of signed(12 downto 0);
    function gen_cell_sx return t_cell_sx is
        variable r : t_cell_sx;
    begin
        r(   0) := to_signed(  150, 13);
        r(   1) := to_signed( -150, 13);
        r(   2) := to_signed(    0, 13);
        r(   3) := to_signed(    0, 13);
        r(   4) := to_signed(    0, 13);
        r(   5) := to_signed(    0, 13);
        r(   6) := to_signed(    0, 13);
        r(   7) := to_signed(    0, 13);
        r(   8) := to_signed(  153, 13);
        r(   9) := to_signed( -142, 13);
        r(  10) := to_signed(  -15, 13);
        r(  11) := to_signed(   14, 13);
        r(  12) := to_signed(  -16, 13);
        r(  13) := to_signed(   16, 13);
        r(  14) := to_signed(  -25, 13);
        r(  15) := to_signed(   15, 13);
        r(  16) := to_signed(  151, 13);
        r(  17) := to_signed( -129, 13);
        r(  18) := to_signed(  -29, 13);
        r(  19) := to_signed(   27, 13);
        r(  20) := to_signed(  -32, 13);
        r(  21) := to_signed(   30, 13);
        r(  22) := to_signed(  -49, 13);
        r(  23) := to_signed(   31, 13);
        r(  24) := to_signed(  143, 13);
        r(  25) := to_signed( -113, 13);
        r(  26) := to_signed(  -40, 13);
        r(  27) := to_signed(   37, 13);
        r(  28) := to_signed(  -45, 13);
        r(  29) := to_signed(   43, 13);
        r(  30) := to_signed(  -71, 13);
        r(  31) := to_signed(   46, 13);
        r(  32) := to_signed(  130, 13);
        r(  33) := to_signed(  -95, 13);
        r(  34) := to_signed(  -48, 13);
        r(  35) := to_signed(   44, 13);
        r(  36) := to_signed(  -56, 13);
        r(  37) := to_signed(   54, 13);
        r(  38) := to_signed(  -91, 13);
        r(  39) := to_signed(   62, 13);
        r(  40) := to_signed(  112, 13);
        r(  41) := to_signed(  -77, 13);
        r(  42) := to_signed(  -51, 13);
        r(  43) := to_signed(   47, 13);
        r(  44) := to_signed(  -62, 13);
        r(  45) := to_signed(   62, 13);
        r(  46) := to_signed( -107, 13);
        r(  47) := to_signed(   77, 13);
        r(  48) := to_signed(   90, 13);
        r(  49) := to_signed(  -59, 13);
        r(  50) := to_signed(  -50, 13);
        r(  51) := to_signed(   46, 13);
        r(  52) := to_signed(  -64, 13);
        r(  53) := to_signed(   67, 13);
        r(  54) := to_signed( -120, 13);
        r(  55) := to_signed(   92, 13);
        r(  56) := to_signed(   68, 13);
        r(  57) := to_signed(  -43, 13);
        r(  58) := to_signed(  -45, 13);
        r(  59) := to_signed(   42, 13);
        r(  60) := to_signed(  -62, 13);
        r(  61) := to_signed(   67, 13);
        r(  62) := to_signed( -130, 13);
        r(  63) := to_signed(  106, 13);
        r(  64) := to_signed(   47, 13);
        r(  65) := to_signed(  -29, 13);
        r(  66) := to_signed(  -38, 13);
        r(  67) := to_signed(   34, 13);
        r(  68) := to_signed(  -57, 13);
        r(  69) := to_signed(   63, 13);
        r(  70) := to_signed( -138, 13);
        r(  71) := to_signed(  119, 13);
        r(  72) := to_signed(   29, 13);
        r(  73) := to_signed(  -18, 13);
        r(  74) := to_signed(  -28, 13);
        r(  75) := to_signed(   26, 13);
        r(  76) := to_signed(  -48, 13);
        r(  77) := to_signed(   54, 13);
        r(  78) := to_signed( -144, 13);
        r(  79) := to_signed(  130, 13);
        r(  80) := to_signed(   15, 13);
        r(  81) := to_signed(   -9, 13);
        r(  82) := to_signed(  -18, 13);
        r(  83) := to_signed(   16, 13);
        r(  84) := to_signed(  -36, 13);
        r(  85) := to_signed(   41, 13);
        r(  86) := to_signed( -148, 13);
        r(  87) := to_signed(  140, 13);
        r(  88) := to_signed(    6, 13);
        r(  89) := to_signed(   -4, 13);
        r(  90) := to_signed(   -9, 13);
        r(  91) := to_signed(    8, 13);
        r(  92) := to_signed(  -22, 13);
        r(  93) := to_signed(   24, 13);
        r(  94) := to_signed( -150, 13);
        r(  95) := to_signed(  146, 13);
        r(  96) := to_signed(    1, 13);
        r(  97) := to_signed(    0, 13);
        r(  98) := to_signed(   -2, 13);
        r(  99) := to_signed(    1, 13);
        r( 100) := to_signed(   -5, 13);
        r( 101) := to_signed(    6, 13);
        r( 102) := to_signed( -150, 13);
        r( 103) := to_signed(  150, 13);
        r( 104) := to_signed(   -1, 13);
        r( 105) := to_signed(    1, 13);
        r( 106) := to_signed(    2, 13);
        r( 107) := to_signed(   -2, 13);
        r( 108) := to_signed(   13, 13);
        r( 109) := to_signed(  -13, 13);
        r( 110) := to_signed( -149, 13);
        r( 111) := to_signed(  150, 13);
        r( 112) := to_signed(    0, 13);
        r( 113) := to_signed(    0, 13);
        r( 114) := to_signed(    2, 13);
        r( 115) := to_signed(   -2, 13);
        r( 116) := to_signed(   31, 13);
        r( 117) := to_signed(  -33, 13);
        r( 118) := to_signed( -146, 13);
        r( 119) := to_signed(  147, 13);
        r( 120) := to_signed(    0, 13);
        r( 121) := to_signed(    0, 13);
        r( 122) := to_signed(   -2, 13);
        r( 123) := to_signed(    3, 13);
        r( 124) := to_signed(   50, 13);
        r( 125) := to_signed(  -51, 13);
        r( 126) := to_signed( -141, 13);
        r( 127) := to_signed(  142, 13);
        r( 128) := to_signed(    0, 13);
        r( 129) := to_signed(    0, 13);
        r( 130) := to_signed(  -10, 13);
        r( 131) := to_signed(   11, 13);
        r( 132) := to_signed(   67, 13);
        r( 133) := to_signed(  -68, 13);
        r( 134) := to_signed( -134, 13);
        r( 135) := to_signed(  134, 13);
        r( 136) := to_signed(   -3, 13);
        r( 137) := to_signed(    2, 13);
        r( 138) := to_signed(  -21, 13);
        r( 139) := to_signed(   24, 13);
        r( 140) := to_signed(   81, 13);
        r( 141) := to_signed(  -82, 13);
        r( 142) := to_signed( -124, 13);
        r( 143) := to_signed(  123, 13);
        r( 144) := to_signed(   -9, 13);
        r( 145) := to_signed(    6, 13);
        r( 146) := to_signed(  -33, 13);
        r( 147) := to_signed(   39, 13);
        r( 148) := to_signed(   92, 13);
        r( 149) := to_signed(  -94, 13);
        r( 150) := to_signed( -113, 13);
        r( 151) := to_signed(  112, 13);
        r( 152) := to_signed(  -20, 13);
        r( 153) := to_signed(   12, 13);
        r( 154) := to_signed(  -45, 13);
        r( 155) := to_signed(   56, 13);
        r( 156) := to_signed(   98, 13);
        r( 157) := to_signed( -102, 13);
        r( 158) := to_signed(  -99, 13);
        r( 159) := to_signed(   98, 13);
        r( 160) := to_signed(  -34, 13);
        r( 161) := to_signed(   22, 13);
        r( 162) := to_signed(  -57, 13);
        r( 163) := to_signed(   74, 13);
        r( 164) := to_signed(  101, 13);
        r( 165) := to_signed( -107, 13);
        r( 166) := to_signed(  -83, 13);
        r( 167) := to_signed(   83, 13);
        r( 168) := to_signed(  -51, 13);
        r( 169) := to_signed(   34, 13);
        r( 170) := to_signed(  -66, 13);
        r( 171) := to_signed(   89, 13);
        r( 172) := to_signed(   99, 13);
        r( 173) := to_signed( -107, 13);
        r( 174) := to_signed(  -66, 13);
        r( 175) := to_signed(   67, 13);
        r( 176) := to_signed(  -70, 13);
        r( 177) := to_signed(   48, 13);
        r( 178) := to_signed(  -73, 13);
        r( 179) := to_signed(  101, 13);
        r( 180) := to_signed(   94, 13);
        r( 181) := to_signed( -103, 13);
        r( 182) := to_signed(  -48, 13);
        r( 183) := to_signed(   50, 13);
        r( 184) := to_signed(  -89, 13);
        r( 185) := to_signed(   63, 13);
        r( 186) := to_signed(  -77, 13);
        r( 187) := to_signed(  109, 13);
        r( 188) := to_signed(   86, 13);
        r( 189) := to_signed(  -94, 13);
        r( 190) := to_signed(  -30, 13);
        r( 191) := to_signed(   32, 13);
        r( 192) := to_signed( -106, 13);
        r( 193) := to_signed(   78, 13);
        r( 194) := to_signed(  -76, 13);
        r( 195) := to_signed(  111, 13);
        r( 196) := to_signed(   75, 13);
        r( 197) := to_signed(  -82, 13);
        r( 198) := to_signed(  -11, 13);
        r( 199) := to_signed(   12, 13);
        r( 200) := to_signed( -120, 13);
        r( 201) := to_signed(   93, 13);
        r( 202) := to_signed(  -72, 13);
        r( 203) := to_signed(  107, 13);
        r( 204) := to_signed(   61, 13);
        r( 205) := to_signed(  -67, 13);
        r( 206) := to_signed(    7, 13);
        r( 207) := to_signed(   -8, 13);
        r( 208) := to_signed( -129, 13);
        r( 209) := to_signed(  105, 13);
        r( 210) := to_signed(  -64, 13);
        r( 211) := to_signed(   98, 13);
        r( 212) := to_signed(   47, 13);
        r( 213) := to_signed(  -50, 13);
        r( 214) := to_signed(   24, 13);
        r( 215) := to_signed(  -29, 13);
        r( 216) := to_signed( -131, 13);
        r( 217) := to_signed(  116, 13);
        r( 218) := to_signed(  -54, 13);
        r( 219) := to_signed(   83, 13);
        r( 220) := to_signed(   31, 13);
        r( 221) := to_signed(  -33, 13);
        r( 222) := to_signed(   40, 13);
        r( 223) := to_signed(  -51, 13);
        r( 224) := to_signed( -128, 13);
        r( 225) := to_signed(  122, 13);
        r( 226) := to_signed(  -43, 13);
        r( 227) := to_signed(   66, 13);
        r( 228) := to_signed(   15, 13);
        r( 229) := to_signed(  -17, 13);
        r( 230) := to_signed(   56, 13);
        r( 231) := to_signed(  -72, 13);
        r( 232) := to_signed( -119, 13);
        r( 233) := to_signed(  125, 13);
        r( 234) := to_signed(  -31, 13);
        r( 235) := to_signed(   47, 13);
        r( 236) := to_signed(    1, 13);
        r( 237) := to_signed(   -1, 13);
        r( 238) := to_signed(   70, 13);
        r( 239) := to_signed(  -92, 13);
        r( 240) := to_signed( -106, 13);
        r( 241) := to_signed(  122, 13);
        r( 242) := to_signed(  -19, 13);
        r( 243) := to_signed(   28, 13);
        r( 244) := to_signed(  -11, 13);
        r( 245) := to_signed(   12, 13);
        r( 246) := to_signed(   84, 13);
        r( 247) := to_signed( -110, 13);
        r( 248) := to_signed(  -90, 13);
        r( 249) := to_signed(  113, 13);
        r( 250) := to_signed(   -8, 13);
        r( 251) := to_signed(   12, 13);
        r( 252) := to_signed(  -20, 13);
        r( 253) := to_signed(   23, 13);
        r( 254) := to_signed(   97, 13);
        r( 255) := to_signed( -125, 13);
        r( 256) := to_signed(  -73, 13);
        r( 257) := to_signed(   98, 13);
        r( 258) := to_signed(    0, 13);
        r( 259) := to_signed(    0, 13);
        r( 260) := to_signed(  -25, 13);
        r( 261) := to_signed(   30, 13);
        r( 262) := to_signed(  108, 13);
        r( 263) := to_signed( -138, 13);
        r( 264) := to_signed(  -56, 13);
        r( 265) := to_signed(   78, 13);
        r( 266) := to_signed(    6, 13);
        r( 267) := to_signed(   -7, 13);
        r( 268) := to_signed(  -26, 13);
        r( 269) := to_signed(   34, 13);
        r( 270) := to_signed(  119, 13);
        r( 271) := to_signed( -147, 13);
        r( 272) := to_signed(  -40, 13);
        r( 273) := to_signed(   55, 13);
        r( 274) := to_signed(    9, 13);
        r( 275) := to_signed(  -10, 13);
        r( 276) := to_signed(  -24, 13);
        r( 277) := to_signed(   33, 13);
        r( 278) := to_signed(  129, 13);
        r( 279) := to_signed( -152, 13);
        r( 280) := to_signed(  -24, 13);
        r( 281) := to_signed(   33, 13);
        r( 282) := to_signed(    8, 13);
        r( 283) := to_signed(   -9, 13);
        r( 284) := to_signed(  -18, 13);
        r( 285) := to_signed(   27, 13);
        r( 286) := to_signed(  138, 13);
        r( 287) := to_signed( -154, 13);
        r( 288) := to_signed(  -11, 13);
        r( 289) := to_signed(   14, 13);
        r( 290) := to_signed(    5, 13);
        r( 291) := to_signed(   -5, 13);
        r( 292) := to_signed(  -10, 13);
        r( 293) := to_signed(   15, 13);
        r( 294) := to_signed(  145, 13);
        r( 295) := to_signed( -153, 13);
        r( 296) := to_signed(    1, 13);
        r( 297) := to_signed(   -1, 13);
        r( 298) := to_signed(    0, 13);
        r( 299) := to_signed(    0, 13);
        r( 300) := to_signed(    1, 13);
        r( 301) := to_signed(   -1, 13);
        r( 302) := to_signed(  150, 13);
        r( 303) := to_signed( -150, 13);
        r( 304) := to_signed(    9, 13);
        r( 305) := to_signed(  -10, 13);
        r( 306) := to_signed(   -6, 13);
        r( 307) := to_signed(    7, 13);
        r( 308) := to_signed(   14, 13);
        r( 309) := to_signed(  -22, 13);
        r( 310) := to_signed(  153, 13);
        r( 311) := to_signed( -145, 13);
        r( 312) := to_signed(   14, 13);
        r( 313) := to_signed(  -15, 13);
        r( 314) := to_signed(  -11, 13);
        r( 315) := to_signed(   13, 13);
        r( 316) := to_signed(   28, 13);
        r( 317) := to_signed(  -45, 13);
        r( 318) := to_signed(  152, 13);
        r( 319) := to_signed( -138, 13);
        r( 320) := to_signed(   16, 13);
        r( 321) := to_signed(  -15, 13);
        r( 322) := to_signed(  -14, 13);
        r( 323) := to_signed(   17, 13);
        r( 324) := to_signed(   44, 13);
        r( 325) := to_signed(  -67, 13);
        r( 326) := to_signed(  149, 13);
        r( 327) := to_signed( -130, 13);
        r( 328) := to_signed(   14, 13);
        r( 329) := to_signed(  -13, 13);
        r( 330) := to_signed(  -14, 13);
        r( 331) := to_signed(   19, 13);
        r( 332) := to_signed(   61, 13);
        r( 333) := to_signed(  -87, 13);
        r( 334) := to_signed(  141, 13);
        r( 335) := to_signed( -121, 13);
        r( 336) := to_signed(    9, 13);
        r( 337) := to_signed(   -9, 13);
        r( 338) := to_signed(  -11, 13);
        r( 339) := to_signed(   16, 13);
        r( 340) := to_signed(   78, 13);
        r( 341) := to_signed( -104, 13);
        r( 342) := to_signed(  130, 13);
        r( 343) := to_signed( -110, 13);
        r( 344) := to_signed(    3, 13);
        r( 345) := to_signed(   -3, 13);
        r( 346) := to_signed(   -5, 13);
        r( 347) := to_signed(    8, 13);
        r( 348) := to_signed(   96, 13);
        r( 349) := to_signed( -116, 13);
        r( 350) := to_signed(  116, 13);
        r( 351) := to_signed(  -99, 13);
        r( 352) := to_signed(   -2, 13);
        r( 353) := to_signed(    2, 13);
        r( 354) := to_signed(    3, 13);
        r( 355) := to_signed(   -6, 13);
        r( 356) := to_signed(  112, 13);
        r( 357) := to_signed( -124, 13);
        r( 358) := to_signed(  100, 13);
        r( 359) := to_signed(  -87, 13);
        r( 360) := to_signed(   -5, 13);
        r( 361) := to_signed(    6, 13);
        r( 362) := to_signed(   15, 13);
        r( 363) := to_signed(  -23, 13);
        r( 364) := to_signed(  126, 13);
        r( 365) := to_signed( -128, 13);
        r( 366) := to_signed(   82, 13);
        r( 367) := to_signed(  -73, 13);
        r( 368) := to_signed(   -6, 13);
        r( 369) := to_signed(    8, 13);
        r( 370) := to_signed(   28, 13);
        r( 371) := to_signed(  -42, 13);
        r( 372) := to_signed(  136, 13);
        r( 373) := to_signed( -130, 13);
        r( 374) := to_signed(   63, 13);
        r( 375) := to_signed(  -58, 13);
        r( 376) := to_signed(   -4, 13);
        r( 377) := to_signed(    6, 13);
        r( 378) := to_signed(   44, 13);
        r( 379) := to_signed(  -60, 13);
        r( 380) := to_signed(  141, 13);
        r( 381) := to_signed( -129, 13);
        r( 382) := to_signed(   43, 13);
        r( 383) := to_signed(  -41, 13);
        r( 384) := to_signed(    0, 13);
        r( 385) := to_signed(    0, 13);
        r( 386) := to_signed(   61, 13);
        r( 387) := to_signed(  -75, 13);
        r( 388) := to_signed(  139, 13);
        r( 389) := to_signed( -126, 13);
        r( 390) := to_signed(   23, 13);
        r( 391) := to_signed(  -23, 13);
        r( 392) := to_signed(    6, 13);
        r( 393) := to_signed(  -11, 13);
        r( 394) := to_signed(   78, 13);
        r( 395) := to_signed(  -85, 13);
        r( 396) := to_signed(  131, 13);
        r( 397) := to_signed( -120, 13);
        r( 398) := to_signed(    4, 13);
        r( 399) := to_signed(   -5, 13);
        r( 400) := to_signed(   15, 13);
        r( 401) := to_signed(  -25, 13);
        r( 402) := to_signed(   94, 13);
        r( 403) := to_signed(  -91, 13);
        r( 404) := to_signed(  118, 13);
        r( 405) := to_signed( -113, 13);
        r( 406) := to_signed(  -14, 13);
        r( 407) := to_signed(   15, 13);
        r( 408) := to_signed(   25, 13);
        r( 409) := to_signed(  -40, 13);
        r( 410) := to_signed(  107, 13);
        r( 411) := to_signed(  -94, 13);
        r( 412) := to_signed(  101, 13);
        r( 413) := to_signed( -103, 13);
        r( 414) := to_signed(  -32, 13);
        r( 415) := to_signed(   35, 13);
        r( 416) := to_signed(   35, 13);
        r( 417) := to_signed(  -54, 13);
        r( 418) := to_signed(  114, 13);
        r( 419) := to_signed(  -92, 13);
        r( 420) := to_signed(   83, 13);
        r( 421) := to_signed(  -92, 13);
        r( 422) := to_signed(  -49, 13);
        r( 423) := to_signed(   55, 13);
        r( 424) := to_signed(   45, 13);
        r( 425) := to_signed(  -65, 13);
        r( 426) := to_signed(  114, 13);
        r( 427) := to_signed(  -88, 13);
        r( 428) := to_signed(   64, 13);
        r( 429) := to_signed(  -78, 13);
        r( 430) := to_signed(  -66, 13);
        r( 431) := to_signed(   73, 13);
        r( 432) := to_signed(   53, 13);
        r( 433) := to_signed(  -73, 13);
        r( 434) := to_signed(  106, 13);
        r( 435) := to_signed(  -81, 13);
        r( 436) := to_signed(   47, 13);
        r( 437) := to_signed(  -62, 13);
        r( 438) := to_signed(  -82, 13);
        r( 439) := to_signed(   90, 13);
        r( 440) := to_signed(   59, 13);
        r( 441) := to_signed(  -75, 13);
        r( 442) := to_signed(   92, 13);
        r( 443) := to_signed(  -72, 13);
        r( 444) := to_signed(   32, 13);
        r( 445) := to_signed(  -45, 13);
        r( 446) := to_signed(  -96, 13);
        r( 447) := to_signed(  105, 13);
        r( 448) := to_signed(   62, 13);
        r( 449) := to_signed(  -73, 13);
        r( 450) := to_signed(   74, 13);
        r( 451) := to_signed(  -61, 13);
        r( 452) := to_signed(   19, 13);
        r( 453) := to_signed(  -29, 13);
        r( 454) := to_signed( -110, 13);
        r( 455) := to_signed(  118, 13);
        r( 456) := to_signed(   59, 13);
        r( 457) := to_signed(  -67, 13);
        r( 458) := to_signed(   55, 13);
        r( 459) := to_signed(  -49, 13);
        r( 460) := to_signed(    8, 13);
        r( 461) := to_signed(  -14, 13);
        r( 462) := to_signed( -123, 13);
        r( 463) := to_signed(  129, 13);
        r( 464) := to_signed(   52, 13);
        r( 465) := to_signed(  -57, 13);
        r( 466) := to_signed(   37, 13);
        r( 467) := to_signed(  -36, 13);
        r( 468) := to_signed(    1, 13);
        r( 469) := to_signed(   -2, 13);
        r( 470) := to_signed( -133, 13);
        r( 471) := to_signed(  137, 13);
        r( 472) := to_signed(   41, 13);
        r( 473) := to_signed(  -43, 13);
        r( 474) := to_signed(   22, 13);
        r( 475) := to_signed(  -23, 13);
        r( 476) := to_signed(   -3, 13);
        r( 477) := to_signed(    6, 13);
        r( 478) := to_signed( -141, 13);
        r( 479) := to_signed(  143, 13);
        r( 480) := to_signed(   26, 13);
        r( 481) := to_signed(  -27, 13);
        r( 482) := to_signed(   10, 13);
        r( 483) := to_signed(  -12, 13);
        r( 484) := to_signed(   -5, 13);
        r( 485) := to_signed(    8, 13);
        r( 486) := to_signed( -147, 13);
        r( 487) := to_signed(  147, 13);
        r( 488) := to_signed(    9, 13);
        r( 489) := to_signed(  -10, 13);
        r( 490) := to_signed(    3, 13);
        r( 491) := to_signed(   -3, 13);
        r( 492) := to_signed(   -3, 13);
        r( 493) := to_signed(    4, 13);
        r( 494) := to_signed( -150, 13);
        r( 495) := to_signed(  149, 13);
        r( 496) := to_signed(   -8, 13);
        r( 497) := to_signed(    8, 13);
        r( 498) := to_signed(   -1, 13);
        r( 499) := to_signed(    2, 13);
        r( 500) := to_signed(    3, 13);
        r( 501) := to_signed(   -4, 13);
        r( 502) := to_signed( -149, 13);
        r( 503) := to_signed(  150, 13);
        r( 504) := to_signed(  -23, 13);
        r( 505) := to_signed(   25, 13);
        r( 506) := to_signed(   -2, 13);
        r( 507) := to_signed(    3, 13);
        r( 508) := to_signed(   12, 13);
        r( 509) := to_signed(  -17, 13);
        r( 510) := to_signed( -146, 13);
        r( 511) := to_signed(  149, 13);
        return r;
    end function;
    constant C_CELL_SX : t_cell_sx := gen_cell_sx;

    type t_cell_sy is array(0 to 512-1) of signed(12 downto 0);
    function gen_cell_sy return t_cell_sy is
        variable r : t_cell_sy;
    begin
        r(   0) := to_signed(    0, 13);
        r(   1) := to_signed(    0, 13);
        r(   2) := to_signed(  150, 13);
        r(   3) := to_signed( -150, 13);
        r(   4) := to_signed(    0, 13);
        r(   5) := to_signed(    0, 13);
        r(   6) := to_signed(    0, 13);
        r(   7) := to_signed(    0, 13);
        r(   8) := to_signed(   12, 13);
        r(   9) := to_signed(  -11, 13);
        r(  10) := to_signed(  151, 13);
        r(  11) := to_signed( -146, 13);
        r(  12) := to_signed(  -13, 13);
        r(  13) := to_signed(   13, 13);
        r(  14) := to_signed(  -15, 13);
        r(  15) := to_signed(    9, 13);
        r(  16) := to_signed(   19, 13);
        r(  17) := to_signed(  -16, 13);
        r(  18) := to_signed(  150, 13);
        r(  19) := to_signed( -141, 13);
        r(  20) := to_signed(  -25, 13);
        r(  21) := to_signed(   24, 13);
        r(  22) := to_signed(  -30, 13);
        r(  23) := to_signed(   18, 13);
        r(  24) := to_signed(   20, 13);
        r(  25) := to_signed(  -16, 13);
        r(  26) := to_signed(  148, 13);
        r(  27) := to_signed( -136, 13);
        r(  28) := to_signed(  -33, 13);
        r(  29) := to_signed(   31, 13);
        r(  30) := to_signed(  -41, 13);
        r(  31) := to_signed(   27, 13);
        r(  32) := to_signed(   15, 13);
        r(  33) := to_signed(  -11, 13);
        r(  34) := to_signed(  146, 13);
        r(  35) := to_signed( -133, 13);
        r(  36) := to_signed(  -36, 13);
        r(  37) := to_signed(   35, 13);
        r(  38) := to_signed(  -50, 13);
        r(  39) := to_signed(   34, 13);
        r(  40) := to_signed(    8, 13);
        r(  41) := to_signed(   -5, 13);
        r(  42) := to_signed(  145, 13);
        r(  43) := to_signed( -133, 13);
        r(  44) := to_signed(  -33, 13);
        r(  45) := to_signed(   33, 13);
        r(  46) := to_signed(  -55, 13);
        r(  47) := to_signed(   40, 13);
        r(  48) := to_signed(   -1, 13);
        r(  49) := to_signed(    1, 13);
        r(  50) := to_signed(  146, 13);
        r(  51) := to_signed( -133, 13);
        r(  52) := to_signed(  -25, 13);
        r(  53) := to_signed(   26, 13);
        r(  54) := to_signed(  -57, 13);
        r(  55) := to_signed(   43, 13);
        r(  56) := to_signed(  -10, 13);
        r(  57) := to_signed(    6, 13);
        r(  58) := to_signed(  148, 13);
        r(  59) := to_signed( -135, 13);
        r(  60) := to_signed(  -12, 13);
        r(  61) := to_signed(   13, 13);
        r(  62) := to_signed(  -55, 13);
        r(  63) := to_signed(   45, 13);
        r(  64) := to_signed(  -15, 13);
        r(  65) := to_signed(    9, 13);
        r(  66) := to_signed(  149, 13);
        r(  67) := to_signed( -136, 13);
        r(  68) := to_signed(    6, 13);
        r(  69) := to_signed(   -6, 13);
        r(  70) := to_signed(  -50, 13);
        r(  71) := to_signed(   43, 13);
        r(  72) := to_signed(  -17, 13);
        r(  73) := to_signed(   10, 13);
        r(  74) := to_signed(  148, 13);
        r(  75) := to_signed( -135, 13);
        r(  76) := to_signed(   26, 13);
        r(  77) := to_signed(  -29, 13);
        r(  78) := to_signed(  -42, 13);
        r(  79) := to_signed(   38, 13);
        r(  80) := to_signed(  -14, 13);
        r(  81) := to_signed(    9, 13);
        r(  82) := to_signed(  144, 13);
        r(  83) := to_signed( -131, 13);
        r(  84) := to_signed(   48, 13);
        r(  85) := to_signed(  -54, 13);
        r(  86) := to_signed(  -32, 13);
        r(  87) := to_signed(   30, 13);
        r(  88) := to_signed(   -9, 13);
        r(  89) := to_signed(    5, 13);
        r(  90) := to_signed(  135, 13);
        r(  91) := to_signed( -123, 13);
        r(  92) := to_signed(   71, 13);
        r(  93) := to_signed(  -78, 13);
        r(  94) := to_signed(  -19, 13);
        r(  95) := to_signed(   19, 13);
        r(  96) := to_signed(   -1, 13);
        r(  97) := to_signed(    1, 13);
        r(  98) := to_signed(  120, 13);
        r(  99) := to_signed( -111, 13);
        r( 100) := to_signed(   92, 13);
        r( 101) := to_signed( -100, 13);
        r( 102) := to_signed(   -5, 13);
        r( 103) := to_signed(    5, 13);
        r( 104) := to_signed(    5, 13);
        r( 105) := to_signed(   -3, 13);
        r( 106) := to_signed(  100, 13);
        r( 107) := to_signed(  -95, 13);
        r( 108) := to_signed(  110, 13);
        r( 109) := to_signed( -117, 13);
        r( 110) := to_signed(   11, 13);
        r( 111) := to_signed(  -11, 13);
        r( 112) := to_signed(   10, 13);
        r( 113) := to_signed(   -6, 13);
        r( 114) := to_signed(   76, 13);
        r( 115) := to_signed(  -74, 13);
        r( 116) := to_signed(  124, 13);
        r( 117) := to_signed( -129, 13);
        r( 118) := to_signed(   29, 13);
        r( 119) := to_signed(  -29, 13);
        r( 120) := to_signed(   12, 13);
        r( 121) := to_signed(   -7, 13);
        r( 122) := to_signed(   50, 13);
        r( 123) := to_signed(  -51, 13);
        r( 124) := to_signed(  132, 13);
        r( 125) := to_signed( -134, 13);
        r( 126) := to_signed(   47, 13);
        r( 127) := to_signed(  -47, 13);
        r( 128) := to_signed(   10, 13);
        r( 129) := to_signed(   -6, 13);
        r( 130) := to_signed(   24, 13);
        r( 131) := to_signed(  -26, 13);
        r( 132) := to_signed(  132, 13);
        r( 133) := to_signed( -134, 13);
        r( 134) := to_signed(   65, 13);
        r( 135) := to_signed(  -65, 13);
        r( 136) := to_signed(    3, 13);
        r( 137) := to_signed(   -2, 13);
        r( 138) := to_signed(    2, 13);
        r( 139) := to_signed(   -2, 13);
        r( 140) := to_signed(  125, 13);
        r( 141) := to_signed( -126, 13);
        r( 142) := to_signed(   82, 13);
        r( 143) := to_signed(  -82, 13);
        r( 144) := to_signed(   -5, 13);
        r( 145) := to_signed(    3, 13);
        r( 146) := to_signed(  -17, 13);
        r( 147) := to_signed(   20, 13);
        r( 148) := to_signed(  111, 13);
        r( 149) := to_signed( -113, 13);
        r( 150) := to_signed(   99, 13);
        r( 151) := to_signed(  -98, 13);
        r( 152) := to_signed(  -15, 13);
        r( 153) := to_signed(    9, 13);
        r( 154) := to_signed(  -30, 13);
        r( 155) := to_signed(   37, 13);
        r( 156) := to_signed(   91, 13);
        r( 157) := to_signed(  -94, 13);
        r( 158) := to_signed(  113, 13);
        r( 159) := to_signed( -112, 13);
        r( 160) := to_signed(  -22, 13);
        r( 161) := to_signed(   15, 13);
        r( 162) := to_signed(  -38, 13);
        r( 163) := to_signed(   49, 13);
        r( 164) := to_signed(   67, 13);
        r( 165) := to_signed(  -71, 13);
        r( 166) := to_signed(  125, 13);
        r( 167) := to_signed( -125, 13);
        r( 168) := to_signed(  -27, 13);
        r( 169) := to_signed(   18, 13);
        r( 170) := to_signed(  -41, 13);
        r( 171) := to_signed(   55, 13);
        r( 172) := to_signed(   42, 13);
        r( 173) := to_signed(  -45, 13);
        r( 174) := to_signed(  133, 13);
        r( 175) := to_signed( -135, 13);
        r( 176) := to_signed(  -25, 13);
        r( 177) := to_signed(   17, 13);
        r( 178) := to_signed(  -40, 13);
        r( 179) := to_signed(   55, 13);
        r( 180) := to_signed(   17, 13);
        r( 181) := to_signed(  -19, 13);
        r( 182) := to_signed(  137, 13);
        r( 183) := to_signed( -143, 13);
        r( 184) := to_signed(  -18, 13);
        r( 185) := to_signed(   13, 13);
        r( 186) := to_signed(  -37, 13);
        r( 187) := to_signed(   52, 13);
        r( 188) := to_signed(   -6, 13);
        r( 189) := to_signed(    7, 13);
        r( 190) := to_signed(  138, 13);
        r( 191) := to_signed( -148, 13);
        r( 192) := to_signed(   -5, 13);
        r( 193) := to_signed(    4, 13);
        r( 194) := to_signed(  -32, 13);
        r( 195) := to_signed(   47, 13);
        r( 196) := to_signed(  -27, 13);
        r( 197) := to_signed(   29, 13);
        r( 198) := to_signed(  135, 13);
        r( 199) := to_signed( -150, 13);
        r( 200) := to_signed(   13, 13);
        r( 201) := to_signed(  -10, 13);
        r( 202) := to_signed(  -29, 13);
        r( 203) := to_signed(   44, 13);
        r( 204) := to_signed(  -43, 13);
        r( 205) := to_signed(   47, 13);
        r( 206) := to_signed(  129, 13);
        r( 207) := to_signed( -149, 13);
        r( 208) := to_signed(   33, 13);
        r( 209) := to_signed(  -27, 13);
        r( 210) := to_signed(  -29, 13);
        r( 211) := to_signed(   43, 13);
        r( 212) := to_signed(  -54, 13);
        r( 213) := to_signed(   58, 13);
        r( 214) := to_signed(  120, 13);
        r( 215) := to_signed( -145, 13);
        r( 216) := to_signed(   53, 13);
        r( 217) := to_signed(  -46, 13);
        r( 218) := to_signed(  -32, 13);
        r( 219) := to_signed(   49, 13);
        r( 220) := to_signed(  -59, 13);
        r( 221) := to_signed(   63, 13);
        r( 222) := to_signed(  110, 13);
        r( 223) := to_signed( -137, 13);
        r( 224) := to_signed(   69, 13);
        r( 225) := to_signed(  -66, 13);
        r( 226) := to_signed(  -39, 13);
        r( 227) := to_signed(   60, 13);
        r( 228) := to_signed(  -58, 13);
        r( 229) := to_signed(   62, 13);
        r( 230) := to_signed(   98, 13);
        r( 231) := to_signed( -126, 13);
        r( 232) := to_signed(   79, 13);
        r( 233) := to_signed(  -83, 13);
        r( 234) := to_signed(  -50, 13);
        r( 235) := to_signed(   76, 13);
        r( 236) := to_signed(  -52, 13);
        r( 237) := to_signed(   56, 13);
        r( 238) := to_signed(   85, 13);
        r( 239) := to_signed( -111, 13);
        r( 240) := to_signed(   83, 13);
        r( 241) := to_signed(  -95, 13);
        r( 242) := to_signed(  -65, 13);
        r( 243) := to_signed(   96, 13);
        r( 244) := to_signed(  -41, 13);
        r( 245) := to_signed(   45, 13);
        r( 246) := to_signed(   72, 13);
        r( 247) := to_signed(  -95, 13);
        r( 248) := to_signed(   80, 13);
        r( 249) := to_signed(  -99, 13);
        r( 250) := to_signed(  -83, 13);
        r( 251) := to_signed(  117, 13);
        r( 252) := to_signed(  -27, 13);
        r( 253) := to_signed(   31, 13);
        r( 254) := to_signed(   59, 13);
        r( 255) := to_signed(  -77, 13);
        r( 256) := to_signed(   70, 13);
        r( 257) := to_signed(  -93, 13);
        r( 258) := to_signed( -101, 13);
        r( 259) := to_signed(  134, 13);
        r( 260) := to_signed(  -12, 13);
        r( 261) := to_signed(   15, 13);
        r( 262) := to_signed(   46, 13);
        r( 263) := to_signed(  -59, 13);
        r( 264) := to_signed(   53, 13);
        r( 265) := to_signed(  -74, 13);
        r( 266) := to_signed( -119, 13);
        r( 267) := to_signed(  147, 13);
        r( 268) := to_signed(    2, 13);
        r( 269) := to_signed(   -2, 13);
        r( 270) := to_signed(   34, 13);
        r( 271) := to_signed(  -42, 13);
        r( 272) := to_signed(   32, 13);
        r( 273) := to_signed(  -44, 13);
        r( 274) := to_signed( -133, 13);
        r( 275) := to_signed(  154, 13);
        r( 276) := to_signed(   14, 13);
        r( 277) := to_signed(  -19, 13);
        r( 278) := to_signed(   23, 13);
        r( 279) := to_signed(  -27, 13);
        r( 280) := to_signed(    6, 13);
        r( 281) := to_signed(   -8, 13);
        r( 282) := to_signed( -140, 13);
        r( 283) := to_signed(  154, 13);
        r( 284) := to_signed(   23, 13);
        r( 285) := to_signed(  -34, 13);
        r( 286) := to_signed(   13, 13);
        r( 287) := to_signed(  -15, 13);
        r( 288) := to_signed(  -24, 13);
        r( 289) := to_signed(   30, 13);
        r( 290) := to_signed( -139, 13);
        r( 291) := to_signed(  148, 13);
        r( 292) := to_signed(   29, 13);
        r( 293) := to_signed(  -45, 13);
        r( 294) := to_signed(    5, 13);
        r( 295) := to_signed(   -6, 13);
        r( 296) := to_signed(  -56, 13);
        r( 297) := to_signed(   66, 13);
        r( 298) := to_signed( -128, 13);
        r( 299) := to_signed(  136, 13);
        r( 300) := to_signed(   31, 13);
        r( 301) := to_signed(  -50, 13);
        r( 302) := to_signed(    0, 13);
        r( 303) := to_signed(    0, 13);
        r( 304) := to_signed(  -87, 13);
        r( 305) := to_signed(   95, 13);
        r( 306) := to_signed( -108, 13);
        r( 307) := to_signed(  119, 13);
        r( 308) := to_signed(   30, 13);
        r( 309) := to_signed(  -49, 13);
        r( 310) := to_signed(   -3, 13);
        r( 311) := to_signed(    3, 13);
        r( 312) := to_signed( -116, 13);
        r( 313) := to_signed(  118, 13);
        r( 314) := to_signed(  -83, 13);
        r( 315) := to_signed(   96, 13);
        r( 316) := to_signed(   26, 13);
        r( 317) := to_signed(  -41, 13);
        r( 318) := to_signed(   -3, 13);
        r( 319) := to_signed(    3, 13);
        r( 320) := to_signed( -138, 13);
        r( 321) := to_signed(  134, 13);
        r( 322) := to_signed(  -54, 13);
        r( 323) := to_signed(   68, 13);
        r( 324) := to_signed(   18, 13);
        r( 325) := to_signed(  -28, 13);
        r( 326) := to_signed(    0, 13);
        r( 327) := to_signed(    0, 13);
        r( 328) := to_signed( -150, 13);
        r( 329) := to_signed(  143, 13);
        r( 330) := to_signed(  -27, 13);
        r( 331) := to_signed(   36, 13);
        r( 332) := to_signed(    8, 13);
        r( 333) := to_signed(  -12, 13);
        r( 334) := to_signed(    6, 13);
        r( 335) := to_signed(   -5, 13);
        r( 336) := to_signed( -151, 13);
        r( 337) := to_signed(  147, 13);
        r( 338) := to_signed(   -1, 13);
        r( 339) := to_signed(    1, 13);
        r( 340) := to_signed(   -4, 13);
        r( 341) := to_signed(    5, 13);
        r( 342) := to_signed(   15, 13);
        r( 343) := to_signed(  -13, 13);
        r( 344) := to_signed( -143, 13);
        r( 345) := to_signed(  146, 13);
        r( 346) := to_signed(   21, 13);
        r( 347) := to_signed(  -33, 13);
        r( 348) := to_signed(  -17, 13);
        r( 349) := to_signed(   21, 13);
        r( 350) := to_signed(   27, 13);
        r( 351) := to_signed(  -23, 13);
        r( 352) := to_signed( -127, 13);
        r( 353) := to_signed(  141, 13);
        r( 354) := to_signed(   39, 13);
        r( 355) := to_signed(  -63, 13);
        r( 356) := to_signed(  -31, 13);
        r( 357) := to_signed(   34, 13);
        r( 358) := to_signed(   39, 13);
        r( 359) := to_signed(  -34, 13);
        r( 360) := to_signed( -107, 13);
        r( 361) := to_signed(  130, 13);
        r( 362) := to_signed(   54, 13);
        r( 363) := to_signed(  -85, 13);
        r( 364) := to_signed(  -44, 13);
        r( 365) := to_signed(   45, 13);
        r( 366) := to_signed(   52, 13);
        r( 367) := to_signed(  -47, 13);
        r( 368) := to_signed(  -85, 13);
        r( 369) := to_signed(  115, 13);
        r( 370) := to_signed(   66, 13);
        r( 371) := to_signed(  -98, 13);
        r( 372) := to_signed(  -55, 13);
        r( 373) := to_signed(   52, 13);
        r( 374) := to_signed(   65, 13);
        r( 375) := to_signed(  -60, 13);
        r( 376) := to_signed(  -64, 13);
        r( 377) := to_signed(   95, 13);
        r( 378) := to_signed(   75, 13);
        r( 379) := to_signed( -102, 13);
        r( 380) := to_signed(  -62, 13);
        r( 381) := to_signed(   57, 13);
        r( 382) := to_signed(   77, 13);
        r( 383) := to_signed(  -74, 13);
        r( 384) := to_signed(  -45, 13);
        r( 385) := to_signed(   71, 13);
        r( 386) := to_signed(   82, 13);
        r( 387) := to_signed( -100, 13);
        r( 388) := to_signed(  -65, 13);
        r( 389) := to_signed(   58, 13);
        r( 390) := to_signed(   87, 13);
        r( 391) := to_signed(  -87, 13);
        r( 392) := to_signed(  -27, 13);
        r( 393) := to_signed(   45, 13);
        r( 394) := to_signed(   87, 13);
        r( 395) := to_signed(  -95, 13);
        r( 396) := to_signed(  -63, 13);
        r( 397) := to_signed(   58, 13);
        r( 398) := to_signed(   96, 13);
        r( 399) := to_signed(  -99, 13);
        r( 400) := to_signed(  -11, 13);
        r( 401) := to_signed(   18, 13);
        r( 402) := to_signed(   91, 13);
        r( 403) := to_signed(  -88, 13);
        r( 404) := to_signed(  -58, 13);
        r( 405) := to_signed(   55, 13);
        r( 406) := to_signed(  102, 13);
        r( 407) := to_signed( -109, 13);
        r( 408) := to_signed(    5, 13);
        r( 409) := to_signed(   -7, 13);
        r( 410) := to_signed(   93, 13);
        r( 411) := to_signed(  -82, 13);
        r( 412) := to_signed(  -50, 13);
        r( 413) := to_signed(   51, 13);
        r( 414) := to_signed(  106, 13);
        r( 415) := to_signed( -117, 13);
        r( 416) := to_signed(   20, 13);
        r( 417) := to_signed(  -30, 13);
        r( 418) := to_signed(   94, 13);
        r( 419) := to_signed(  -76, 13);
        r( 420) := to_signed(  -41, 13);
        r( 421) := to_signed(   45, 13);
        r( 422) := to_signed(  108, 13);
        r( 423) := to_signed( -120, 13);
        r( 424) := to_signed(   35, 13);
        r( 425) := to_signed(  -51, 13);
        r( 426) := to_signed(   94, 13);
        r( 427) := to_signed(  -73, 13);
        r( 428) := to_signed(  -32, 13);
        r( 429) := to_signed(   39, 13);
        r( 430) := to_signed(  108, 13);
        r( 431) := to_signed( -120, 13);
        r( 432) := to_signed(   50, 13);
        r( 433) := to_signed(  -68, 13);
        r( 434) := to_signed(   92, 13);
        r( 435) := to_signed(  -70, 13);
        r( 436) := to_signed(  -24, 13);
        r( 437) := to_signed(   32, 13);
        r( 438) := to_signed(  104, 13);
        r( 439) := to_signed( -116, 13);
        r( 440) := to_signed(   66, 13);
        r( 441) := to_signed(  -84, 13);
        r( 442) := to_signed(   88, 13);
        r( 443) := to_signed(  -69, 13);
        r( 444) := to_signed(  -19, 13);
        r( 445) := to_signed(   27, 13);
        r( 446) := to_signed(   98, 13);
        r( 447) := to_signed( -108, 13);
        r( 448) := to_signed(   82, 13);
        r( 449) := to_signed(  -98, 13);
        r( 450) := to_signed(   83, 13);
        r( 451) := to_signed(  -68, 13);
        r( 452) := to_signed(  -15, 13);
        r( 453) := to_signed(   23, 13);
        r( 454) := to_signed(   89, 13);
        r( 455) := to_signed(  -96, 13);
        r( 456) := to_signed(   98, 13);
        r( 457) := to_signed( -111, 13);
        r( 458) := to_signed(   76, 13);
        r( 459) := to_signed(  -67, 13);
        r( 460) := to_signed(  -14, 13);
        r( 461) := to_signed(   23, 13);
        r( 462) := to_signed(   78, 13);
        r( 463) := to_signed(  -82, 13);
        r( 464) := to_signed(  111, 13);
        r( 465) := to_signed( -122, 13);
        r( 466) := to_signed(   68, 13);
        r( 467) := to_signed(  -65, 13);
        r( 468) := to_signed(  -15, 13);
        r( 469) := to_signed(   25, 13);
        r( 470) := to_signed(   64, 13);
        r( 471) := to_signed(  -66, 13);
        r( 472) := to_signed(  122, 13);
        r( 473) := to_signed( -131, 13);
        r( 474) := to_signed(   60, 13);
        r( 475) := to_signed(  -62, 13);
        r( 476) := to_signed(  -18, 13);
        r( 477) := to_signed(   30, 13);
        r( 478) := to_signed(   47, 13);
        r( 479) := to_signed(  -48, 13);
        r( 480) := to_signed(  130, 13);
        r( 481) := to_signed( -137, 13);
        r( 482) := to_signed(   52, 13);
        r( 483) := to_signed(  -59, 13);
        r( 484) := to_signed(  -23, 13);
        r( 485) := to_signed(   38, 13);
        r( 486) := to_signed(   29, 13);
        r( 487) := to_signed(  -29, 13);
        r( 488) := to_signed(  134, 13);
        r( 489) := to_signed( -140, 13);
        r( 490) := to_signed(   45, 13);
        r( 491) := to_signed(  -55, 13);
        r( 492) := to_signed(  -29, 13);
        r( 493) := to_signed(   46, 13);
        r( 494) := to_signed(   10, 13);
        r( 495) := to_signed(  -10, 13);
        r( 496) := to_signed(  133, 13);
        r( 497) := to_signed( -139, 13);
        r( 498) := to_signed(   38, 13);
        r( 499) := to_signed(  -51, 13);
        r( 500) := to_signed(  -36, 13);
        r( 501) := to_signed(   55, 13);
        r( 502) := to_signed(   -9, 13);
        r( 503) := to_signed(    9, 13);
        r( 504) := to_signed(  130, 13);
        r( 505) := to_signed( -136, 13);
        r( 506) := to_signed(   34, 13);
        r( 507) := to_signed(  -48, 13);
        r( 508) := to_signed(  -43, 13);
        r( 509) := to_signed(   63, 13);
        r( 510) := to_signed(  -27, 13);
        r( 511) := to_signed(   27, 13);
        return r;
    end function;
    constant C_CELL_SY : t_cell_sy := gen_cell_sy;


    -- Plane axes table: each rotation plane operates on two of the 4 axes.
    --   plane 0 = XY (axes 0, 1)
    --   plane 1 = XZ (axes 0, 2)
    --   plane 2 = YZ (axes 1, 2)
    --   plane 3 = XW (axes 0, 3)
    --   plane 4 = YW (axes 1, 3)
    --   plane 5 = ZW (axes 2, 3)
    type t_axis_pair is array(0 to 1) of integer range 0 to 3;
    type t_axis_table is array(0 to 5) of t_axis_pair;
    constant C_PLANE_AXES : t_axis_table := (
        0 => (0, 1),
        1 => (0, 2),
        2 => (1, 2),
        3 => (0, 3),
        4 => (1, 3),
        5 => (2, 3)
    );

    --------------------------------------------------------------------------
    -- Sin LUT: full-cycle 1024-entry table of sin(2*pi*i/1024) * 1023.
    -- Used as a single-port BRAM with synchronous read; the read port is
    -- driven by the sincos load state machine that fills s_sin/s_cos at
    -- the start of every frame.
    --------------------------------------------------------------------------

    type t_sin_full is array(0 to 1023) of signed(10 downto 0);
    function gen_sin_full return t_sin_full is
        variable r : t_sin_full;
    begin
        r(   0) := to_signed(    0, 11); r(   1) := to_signed(    6, 11);
        r(   2) := to_signed(   13, 11); r(   3) := to_signed(   19, 11);
        r(   4) := to_signed(   25, 11); r(   5) := to_signed(   31, 11);
        r(   6) := to_signed(   38, 11); r(   7) := to_signed(   44, 11);
        r(   8) := to_signed(   50, 11); r(   9) := to_signed(   56, 11);
        r(  10) := to_signed(   63, 11); r(  11) := to_signed(   69, 11);
        r(  12) := to_signed(   75, 11); r(  13) := to_signed(   82, 11);
        r(  14) := to_signed(   88, 11); r(  15) := to_signed(   94, 11);
        r(  16) := to_signed(  100, 11); r(  17) := to_signed(  107, 11);
        r(  18) := to_signed(  113, 11); r(  19) := to_signed(  119, 11);
        r(  20) := to_signed(  125, 11); r(  21) := to_signed(  131, 11);
        r(  22) := to_signed(  138, 11); r(  23) := to_signed(  144, 11);
        r(  24) := to_signed(  150, 11); r(  25) := to_signed(  156, 11);
        r(  26) := to_signed(  163, 11); r(  27) := to_signed(  169, 11);
        r(  28) := to_signed(  175, 11); r(  29) := to_signed(  181, 11);
        r(  30) := to_signed(  187, 11); r(  31) := to_signed(  193, 11);
        r(  32) := to_signed(  200, 11); r(  33) := to_signed(  206, 11);
        r(  34) := to_signed(  212, 11); r(  35) := to_signed(  218, 11);
        r(  36) := to_signed(  224, 11); r(  37) := to_signed(  230, 11);
        r(  38) := to_signed(  236, 11); r(  39) := to_signed(  242, 11);
        r(  40) := to_signed(  249, 11); r(  41) := to_signed(  255, 11);
        r(  42) := to_signed(  261, 11); r(  43) := to_signed(  267, 11);
        r(  44) := to_signed(  273, 11); r(  45) := to_signed(  279, 11);
        r(  46) := to_signed(  285, 11); r(  47) := to_signed(  291, 11);
        r(  48) := to_signed(  297, 11); r(  49) := to_signed(  303, 11);
        r(  50) := to_signed(  309, 11); r(  51) := to_signed(  315, 11);
        r(  52) := to_signed(  321, 11); r(  53) := to_signed(  327, 11);
        r(  54) := to_signed(  333, 11); r(  55) := to_signed(  339, 11);
        r(  56) := to_signed(  345, 11); r(  57) := to_signed(  351, 11);
        r(  58) := to_signed(  356, 11); r(  59) := to_signed(  362, 11);
        r(  60) := to_signed(  368, 11); r(  61) := to_signed(  374, 11);
        r(  62) := to_signed(  380, 11); r(  63) := to_signed(  386, 11);
        r(  64) := to_signed(  391, 11); r(  65) := to_signed(  397, 11);
        r(  66) := to_signed(  403, 11); r(  67) := to_signed(  409, 11);
        r(  68) := to_signed(  415, 11); r(  69) := to_signed(  420, 11);
        r(  70) := to_signed(  426, 11); r(  71) := to_signed(  432, 11);
        r(  72) := to_signed(  437, 11); r(  73) := to_signed(  443, 11);
        r(  74) := to_signed(  449, 11); r(  75) := to_signed(  454, 11);
        r(  76) := to_signed(  460, 11); r(  77) := to_signed(  466, 11);
        r(  78) := to_signed(  471, 11); r(  79) := to_signed(  477, 11);
        r(  80) := to_signed(  482, 11); r(  81) := to_signed(  488, 11);
        r(  82) := to_signed(  493, 11); r(  83) := to_signed(  499, 11);
        r(  84) := to_signed(  504, 11); r(  85) := to_signed(  510, 11);
        r(  86) := to_signed(  515, 11); r(  87) := to_signed(  521, 11);
        r(  88) := to_signed(  526, 11); r(  89) := to_signed(  531, 11);
        r(  90) := to_signed(  537, 11); r(  91) := to_signed(  542, 11);
        r(  92) := to_signed(  547, 11); r(  93) := to_signed(  553, 11);
        r(  94) := to_signed(  558, 11); r(  95) := to_signed(  563, 11);
        r(  96) := to_signed(  568, 11); r(  97) := to_signed(  574, 11);
        r(  98) := to_signed(  579, 11); r(  99) := to_signed(  584, 11);
        r( 100) := to_signed(  589, 11); r( 101) := to_signed(  594, 11);
        r( 102) := to_signed(  599, 11); r( 103) := to_signed(  604, 11);
        r( 104) := to_signed(  609, 11); r( 105) := to_signed(  614, 11);
        r( 106) := to_signed(  619, 11); r( 107) := to_signed(  624, 11);
        r( 108) := to_signed(  629, 11); r( 109) := to_signed(  634, 11);
        r( 110) := to_signed(  639, 11); r( 111) := to_signed(  644, 11);
        r( 112) := to_signed(  649, 11); r( 113) := to_signed(  654, 11);
        r( 114) := to_signed(  659, 11); r( 115) := to_signed(  663, 11);
        r( 116) := to_signed(  668, 11); r( 117) := to_signed(  673, 11);
        r( 118) := to_signed(  678, 11); r( 119) := to_signed(  682, 11);
        r( 120) := to_signed(  687, 11); r( 121) := to_signed(  692, 11);
        r( 122) := to_signed(  696, 11); r( 123) := to_signed(  701, 11);
        r( 124) := to_signed(  705, 11); r( 125) := to_signed(  710, 11);
        r( 126) := to_signed(  714, 11); r( 127) := to_signed(  719, 11);
        r( 128) := to_signed(  723, 11); r( 129) := to_signed(  728, 11);
        r( 130) := to_signed(  732, 11); r( 131) := to_signed(  737, 11);
        r( 132) := to_signed(  741, 11); r( 133) := to_signed(  745, 11);
        r( 134) := to_signed(  750, 11); r( 135) := to_signed(  754, 11);
        r( 136) := to_signed(  758, 11); r( 137) := to_signed(  762, 11);
        r( 138) := to_signed(  766, 11); r( 139) := to_signed(  771, 11);
        r( 140) := to_signed(  775, 11); r( 141) := to_signed(  779, 11);
        r( 142) := to_signed(  783, 11); r( 143) := to_signed(  787, 11);
        r( 144) := to_signed(  791, 11); r( 145) := to_signed(  795, 11);
        r( 146) := to_signed(  799, 11); r( 147) := to_signed(  803, 11);
        r( 148) := to_signed(  806, 11); r( 149) := to_signed(  810, 11);
        r( 150) := to_signed(  814, 11); r( 151) := to_signed(  818, 11);
        r( 152) := to_signed(  822, 11); r( 153) := to_signed(  825, 11);
        r( 154) := to_signed(  829, 11); r( 155) := to_signed(  833, 11);
        r( 156) := to_signed(  836, 11); r( 157) := to_signed(  840, 11);
        r( 158) := to_signed(  844, 11); r( 159) := to_signed(  847, 11);
        r( 160) := to_signed(  851, 11); r( 161) := to_signed(  854, 11);
        r( 162) := to_signed(  858, 11); r( 163) := to_signed(  861, 11);
        r( 164) := to_signed(  864, 11); r( 165) := to_signed(  868, 11);
        r( 166) := to_signed(  871, 11); r( 167) := to_signed(  874, 11);
        r( 168) := to_signed(  877, 11); r( 169) := to_signed(  881, 11);
        r( 170) := to_signed(  884, 11); r( 171) := to_signed(  887, 11);
        r( 172) := to_signed(  890, 11); r( 173) := to_signed(  893, 11);
        r( 174) := to_signed(  896, 11); r( 175) := to_signed(  899, 11);
        r( 176) := to_signed(  902, 11); r( 177) := to_signed(  905, 11);
        r( 178) := to_signed(  908, 11); r( 179) := to_signed(  911, 11);
        r( 180) := to_signed(  914, 11); r( 181) := to_signed(  917, 11);
        r( 182) := to_signed(  919, 11); r( 183) := to_signed(  922, 11);
        r( 184) := to_signed(  925, 11); r( 185) := to_signed(  927, 11);
        r( 186) := to_signed(  930, 11); r( 187) := to_signed(  933, 11);
        r( 188) := to_signed(  935, 11); r( 189) := to_signed(  938, 11);
        r( 190) := to_signed(  940, 11); r( 191) := to_signed(  943, 11);
        r( 192) := to_signed(  945, 11); r( 193) := to_signed(  948, 11);
        r( 194) := to_signed(  950, 11); r( 195) := to_signed(  952, 11);
        r( 196) := to_signed(  954, 11); r( 197) := to_signed(  957, 11);
        r( 198) := to_signed(  959, 11); r( 199) := to_signed(  961, 11);
        r( 200) := to_signed(  963, 11); r( 201) := to_signed(  965, 11);
        r( 202) := to_signed(  967, 11); r( 203) := to_signed(  969, 11);
        r( 204) := to_signed(  971, 11); r( 205) := to_signed(  973, 11);
        r( 206) := to_signed(  975, 11); r( 207) := to_signed(  977, 11);
        r( 208) := to_signed(  979, 11); r( 209) := to_signed(  981, 11);
        r( 210) := to_signed(  983, 11); r( 211) := to_signed(  984, 11);
        r( 212) := to_signed(  986, 11); r( 213) := to_signed(  988, 11);
        r( 214) := to_signed(  989, 11); r( 215) := to_signed(  991, 11);
        r( 216) := to_signed(  992, 11); r( 217) := to_signed(  994, 11);
        r( 218) := to_signed(  995, 11); r( 219) := to_signed(  997, 11);
        r( 220) := to_signed(  998, 11); r( 221) := to_signed(  999, 11);
        r( 222) := to_signed( 1001, 11); r( 223) := to_signed( 1002, 11);
        r( 224) := to_signed( 1003, 11); r( 225) := to_signed( 1005, 11);
        r( 226) := to_signed( 1006, 11); r( 227) := to_signed( 1007, 11);
        r( 228) := to_signed( 1008, 11); r( 229) := to_signed( 1009, 11);
        r( 230) := to_signed( 1010, 11); r( 231) := to_signed( 1011, 11);
        r( 232) := to_signed( 1012, 11); r( 233) := to_signed( 1013, 11);
        r( 234) := to_signed( 1014, 11); r( 235) := to_signed( 1015, 11);
        r( 236) := to_signed( 1015, 11); r( 237) := to_signed( 1016, 11);
        r( 238) := to_signed( 1017, 11); r( 239) := to_signed( 1017, 11);
        r( 240) := to_signed( 1018, 11); r( 241) := to_signed( 1019, 11);
        r( 242) := to_signed( 1019, 11); r( 243) := to_signed( 1020, 11);
        r( 244) := to_signed( 1020, 11); r( 245) := to_signed( 1021, 11);
        r( 246) := to_signed( 1021, 11); r( 247) := to_signed( 1021, 11);
        r( 248) := to_signed( 1022, 11); r( 249) := to_signed( 1022, 11);
        r( 250) := to_signed( 1022, 11); r( 251) := to_signed( 1023, 11);
        r( 252) := to_signed( 1023, 11); r( 253) := to_signed( 1023, 11);
        r( 254) := to_signed( 1023, 11); r( 255) := to_signed( 1023, 11);
        r( 256) := to_signed( 1023, 11); r( 257) := to_signed( 1023, 11);
        r( 258) := to_signed( 1023, 11); r( 259) := to_signed( 1023, 11);
        r( 260) := to_signed( 1023, 11); r( 261) := to_signed( 1023, 11);
        r( 262) := to_signed( 1022, 11); r( 263) := to_signed( 1022, 11);
        r( 264) := to_signed( 1022, 11); r( 265) := to_signed( 1021, 11);
        r( 266) := to_signed( 1021, 11); r( 267) := to_signed( 1021, 11);
        r( 268) := to_signed( 1020, 11); r( 269) := to_signed( 1020, 11);
        r( 270) := to_signed( 1019, 11); r( 271) := to_signed( 1019, 11);
        r( 272) := to_signed( 1018, 11); r( 273) := to_signed( 1017, 11);
        r( 274) := to_signed( 1017, 11); r( 275) := to_signed( 1016, 11);
        r( 276) := to_signed( 1015, 11); r( 277) := to_signed( 1015, 11);
        r( 278) := to_signed( 1014, 11); r( 279) := to_signed( 1013, 11);
        r( 280) := to_signed( 1012, 11); r( 281) := to_signed( 1011, 11);
        r( 282) := to_signed( 1010, 11); r( 283) := to_signed( 1009, 11);
        r( 284) := to_signed( 1008, 11); r( 285) := to_signed( 1007, 11);
        r( 286) := to_signed( 1006, 11); r( 287) := to_signed( 1005, 11);
        r( 288) := to_signed( 1003, 11); r( 289) := to_signed( 1002, 11);
        r( 290) := to_signed( 1001, 11); r( 291) := to_signed(  999, 11);
        r( 292) := to_signed(  998, 11); r( 293) := to_signed(  997, 11);
        r( 294) := to_signed(  995, 11); r( 295) := to_signed(  994, 11);
        r( 296) := to_signed(  992, 11); r( 297) := to_signed(  991, 11);
        r( 298) := to_signed(  989, 11); r( 299) := to_signed(  988, 11);
        r( 300) := to_signed(  986, 11); r( 301) := to_signed(  984, 11);
        r( 302) := to_signed(  983, 11); r( 303) := to_signed(  981, 11);
        r( 304) := to_signed(  979, 11); r( 305) := to_signed(  977, 11);
        r( 306) := to_signed(  975, 11); r( 307) := to_signed(  973, 11);
        r( 308) := to_signed(  971, 11); r( 309) := to_signed(  969, 11);
        r( 310) := to_signed(  967, 11); r( 311) := to_signed(  965, 11);
        r( 312) := to_signed(  963, 11); r( 313) := to_signed(  961, 11);
        r( 314) := to_signed(  959, 11); r( 315) := to_signed(  957, 11);
        r( 316) := to_signed(  954, 11); r( 317) := to_signed(  952, 11);
        r( 318) := to_signed(  950, 11); r( 319) := to_signed(  948, 11);
        r( 320) := to_signed(  945, 11); r( 321) := to_signed(  943, 11);
        r( 322) := to_signed(  940, 11); r( 323) := to_signed(  938, 11);
        r( 324) := to_signed(  935, 11); r( 325) := to_signed(  933, 11);
        r( 326) := to_signed(  930, 11); r( 327) := to_signed(  927, 11);
        r( 328) := to_signed(  925, 11); r( 329) := to_signed(  922, 11);
        r( 330) := to_signed(  919, 11); r( 331) := to_signed(  917, 11);
        r( 332) := to_signed(  914, 11); r( 333) := to_signed(  911, 11);
        r( 334) := to_signed(  908, 11); r( 335) := to_signed(  905, 11);
        r( 336) := to_signed(  902, 11); r( 337) := to_signed(  899, 11);
        r( 338) := to_signed(  896, 11); r( 339) := to_signed(  893, 11);
        r( 340) := to_signed(  890, 11); r( 341) := to_signed(  887, 11);
        r( 342) := to_signed(  884, 11); r( 343) := to_signed(  881, 11);
        r( 344) := to_signed(  877, 11); r( 345) := to_signed(  874, 11);
        r( 346) := to_signed(  871, 11); r( 347) := to_signed(  868, 11);
        r( 348) := to_signed(  864, 11); r( 349) := to_signed(  861, 11);
        r( 350) := to_signed(  858, 11); r( 351) := to_signed(  854, 11);
        r( 352) := to_signed(  851, 11); r( 353) := to_signed(  847, 11);
        r( 354) := to_signed(  844, 11); r( 355) := to_signed(  840, 11);
        r( 356) := to_signed(  836, 11); r( 357) := to_signed(  833, 11);
        r( 358) := to_signed(  829, 11); r( 359) := to_signed(  825, 11);
        r( 360) := to_signed(  822, 11); r( 361) := to_signed(  818, 11);
        r( 362) := to_signed(  814, 11); r( 363) := to_signed(  810, 11);
        r( 364) := to_signed(  806, 11); r( 365) := to_signed(  803, 11);
        r( 366) := to_signed(  799, 11); r( 367) := to_signed(  795, 11);
        r( 368) := to_signed(  791, 11); r( 369) := to_signed(  787, 11);
        r( 370) := to_signed(  783, 11); r( 371) := to_signed(  779, 11);
        r( 372) := to_signed(  775, 11); r( 373) := to_signed(  771, 11);
        r( 374) := to_signed(  766, 11); r( 375) := to_signed(  762, 11);
        r( 376) := to_signed(  758, 11); r( 377) := to_signed(  754, 11);
        r( 378) := to_signed(  750, 11); r( 379) := to_signed(  745, 11);
        r( 380) := to_signed(  741, 11); r( 381) := to_signed(  737, 11);
        r( 382) := to_signed(  732, 11); r( 383) := to_signed(  728, 11);
        r( 384) := to_signed(  723, 11); r( 385) := to_signed(  719, 11);
        r( 386) := to_signed(  714, 11); r( 387) := to_signed(  710, 11);
        r( 388) := to_signed(  705, 11); r( 389) := to_signed(  701, 11);
        r( 390) := to_signed(  696, 11); r( 391) := to_signed(  692, 11);
        r( 392) := to_signed(  687, 11); r( 393) := to_signed(  682, 11);
        r( 394) := to_signed(  678, 11); r( 395) := to_signed(  673, 11);
        r( 396) := to_signed(  668, 11); r( 397) := to_signed(  663, 11);
        r( 398) := to_signed(  659, 11); r( 399) := to_signed(  654, 11);
        r( 400) := to_signed(  649, 11); r( 401) := to_signed(  644, 11);
        r( 402) := to_signed(  639, 11); r( 403) := to_signed(  634, 11);
        r( 404) := to_signed(  629, 11); r( 405) := to_signed(  624, 11);
        r( 406) := to_signed(  619, 11); r( 407) := to_signed(  614, 11);
        r( 408) := to_signed(  609, 11); r( 409) := to_signed(  604, 11);
        r( 410) := to_signed(  599, 11); r( 411) := to_signed(  594, 11);
        r( 412) := to_signed(  589, 11); r( 413) := to_signed(  584, 11);
        r( 414) := to_signed(  579, 11); r( 415) := to_signed(  574, 11);
        r( 416) := to_signed(  568, 11); r( 417) := to_signed(  563, 11);
        r( 418) := to_signed(  558, 11); r( 419) := to_signed(  553, 11);
        r( 420) := to_signed(  547, 11); r( 421) := to_signed(  542, 11);
        r( 422) := to_signed(  537, 11); r( 423) := to_signed(  531, 11);
        r( 424) := to_signed(  526, 11); r( 425) := to_signed(  521, 11);
        r( 426) := to_signed(  515, 11); r( 427) := to_signed(  510, 11);
        r( 428) := to_signed(  504, 11); r( 429) := to_signed(  499, 11);
        r( 430) := to_signed(  493, 11); r( 431) := to_signed(  488, 11);
        r( 432) := to_signed(  482, 11); r( 433) := to_signed(  477, 11);
        r( 434) := to_signed(  471, 11); r( 435) := to_signed(  466, 11);
        r( 436) := to_signed(  460, 11); r( 437) := to_signed(  454, 11);
        r( 438) := to_signed(  449, 11); r( 439) := to_signed(  443, 11);
        r( 440) := to_signed(  437, 11); r( 441) := to_signed(  432, 11);
        r( 442) := to_signed(  426, 11); r( 443) := to_signed(  420, 11);
        r( 444) := to_signed(  415, 11); r( 445) := to_signed(  409, 11);
        r( 446) := to_signed(  403, 11); r( 447) := to_signed(  397, 11);
        r( 448) := to_signed(  391, 11); r( 449) := to_signed(  386, 11);
        r( 450) := to_signed(  380, 11); r( 451) := to_signed(  374, 11);
        r( 452) := to_signed(  368, 11); r( 453) := to_signed(  362, 11);
        r( 454) := to_signed(  356, 11); r( 455) := to_signed(  351, 11);
        r( 456) := to_signed(  345, 11); r( 457) := to_signed(  339, 11);
        r( 458) := to_signed(  333, 11); r( 459) := to_signed(  327, 11);
        r( 460) := to_signed(  321, 11); r( 461) := to_signed(  315, 11);
        r( 462) := to_signed(  309, 11); r( 463) := to_signed(  303, 11);
        r( 464) := to_signed(  297, 11); r( 465) := to_signed(  291, 11);
        r( 466) := to_signed(  285, 11); r( 467) := to_signed(  279, 11);
        r( 468) := to_signed(  273, 11); r( 469) := to_signed(  267, 11);
        r( 470) := to_signed(  261, 11); r( 471) := to_signed(  255, 11);
        r( 472) := to_signed(  249, 11); r( 473) := to_signed(  242, 11);
        r( 474) := to_signed(  236, 11); r( 475) := to_signed(  230, 11);
        r( 476) := to_signed(  224, 11); r( 477) := to_signed(  218, 11);
        r( 478) := to_signed(  212, 11); r( 479) := to_signed(  206, 11);
        r( 480) := to_signed(  200, 11); r( 481) := to_signed(  193, 11);
        r( 482) := to_signed(  187, 11); r( 483) := to_signed(  181, 11);
        r( 484) := to_signed(  175, 11); r( 485) := to_signed(  169, 11);
        r( 486) := to_signed(  163, 11); r( 487) := to_signed(  156, 11);
        r( 488) := to_signed(  150, 11); r( 489) := to_signed(  144, 11);
        r( 490) := to_signed(  138, 11); r( 491) := to_signed(  131, 11);
        r( 492) := to_signed(  125, 11); r( 493) := to_signed(  119, 11);
        r( 494) := to_signed(  113, 11); r( 495) := to_signed(  107, 11);
        r( 496) := to_signed(  100, 11); r( 497) := to_signed(   94, 11);
        r( 498) := to_signed(   88, 11); r( 499) := to_signed(   82, 11);
        r( 500) := to_signed(   75, 11); r( 501) := to_signed(   69, 11);
        r( 502) := to_signed(   63, 11); r( 503) := to_signed(   56, 11);
        r( 504) := to_signed(   50, 11); r( 505) := to_signed(   44, 11);
        r( 506) := to_signed(   38, 11); r( 507) := to_signed(   31, 11);
        r( 508) := to_signed(   25, 11); r( 509) := to_signed(   19, 11);
        r( 510) := to_signed(   13, 11); r( 511) := to_signed(    6, 11);
        r( 512) := to_signed(    0, 11); r( 513) := to_signed(   -6, 11);
        r( 514) := to_signed(  -13, 11); r( 515) := to_signed(  -19, 11);
        r( 516) := to_signed(  -25, 11); r( 517) := to_signed(  -31, 11);
        r( 518) := to_signed(  -38, 11); r( 519) := to_signed(  -44, 11);
        r( 520) := to_signed(  -50, 11); r( 521) := to_signed(  -56, 11);
        r( 522) := to_signed(  -63, 11); r( 523) := to_signed(  -69, 11);
        r( 524) := to_signed(  -75, 11); r( 525) := to_signed(  -82, 11);
        r( 526) := to_signed(  -88, 11); r( 527) := to_signed(  -94, 11);
        r( 528) := to_signed( -100, 11); r( 529) := to_signed( -107, 11);
        r( 530) := to_signed( -113, 11); r( 531) := to_signed( -119, 11);
        r( 532) := to_signed( -125, 11); r( 533) := to_signed( -131, 11);
        r( 534) := to_signed( -138, 11); r( 535) := to_signed( -144, 11);
        r( 536) := to_signed( -150, 11); r( 537) := to_signed( -156, 11);
        r( 538) := to_signed( -163, 11); r( 539) := to_signed( -169, 11);
        r( 540) := to_signed( -175, 11); r( 541) := to_signed( -181, 11);
        r( 542) := to_signed( -187, 11); r( 543) := to_signed( -193, 11);
        r( 544) := to_signed( -200, 11); r( 545) := to_signed( -206, 11);
        r( 546) := to_signed( -212, 11); r( 547) := to_signed( -218, 11);
        r( 548) := to_signed( -224, 11); r( 549) := to_signed( -230, 11);
        r( 550) := to_signed( -236, 11); r( 551) := to_signed( -242, 11);
        r( 552) := to_signed( -249, 11); r( 553) := to_signed( -255, 11);
        r( 554) := to_signed( -261, 11); r( 555) := to_signed( -267, 11);
        r( 556) := to_signed( -273, 11); r( 557) := to_signed( -279, 11);
        r( 558) := to_signed( -285, 11); r( 559) := to_signed( -291, 11);
        r( 560) := to_signed( -297, 11); r( 561) := to_signed( -303, 11);
        r( 562) := to_signed( -309, 11); r( 563) := to_signed( -315, 11);
        r( 564) := to_signed( -321, 11); r( 565) := to_signed( -327, 11);
        r( 566) := to_signed( -333, 11); r( 567) := to_signed( -339, 11);
        r( 568) := to_signed( -345, 11); r( 569) := to_signed( -351, 11);
        r( 570) := to_signed( -356, 11); r( 571) := to_signed( -362, 11);
        r( 572) := to_signed( -368, 11); r( 573) := to_signed( -374, 11);
        r( 574) := to_signed( -380, 11); r( 575) := to_signed( -386, 11);
        r( 576) := to_signed( -391, 11); r( 577) := to_signed( -397, 11);
        r( 578) := to_signed( -403, 11); r( 579) := to_signed( -409, 11);
        r( 580) := to_signed( -415, 11); r( 581) := to_signed( -420, 11);
        r( 582) := to_signed( -426, 11); r( 583) := to_signed( -432, 11);
        r( 584) := to_signed( -437, 11); r( 585) := to_signed( -443, 11);
        r( 586) := to_signed( -449, 11); r( 587) := to_signed( -454, 11);
        r( 588) := to_signed( -460, 11); r( 589) := to_signed( -466, 11);
        r( 590) := to_signed( -471, 11); r( 591) := to_signed( -477, 11);
        r( 592) := to_signed( -482, 11); r( 593) := to_signed( -488, 11);
        r( 594) := to_signed( -493, 11); r( 595) := to_signed( -499, 11);
        r( 596) := to_signed( -504, 11); r( 597) := to_signed( -510, 11);
        r( 598) := to_signed( -515, 11); r( 599) := to_signed( -521, 11);
        r( 600) := to_signed( -526, 11); r( 601) := to_signed( -531, 11);
        r( 602) := to_signed( -537, 11); r( 603) := to_signed( -542, 11);
        r( 604) := to_signed( -547, 11); r( 605) := to_signed( -553, 11);
        r( 606) := to_signed( -558, 11); r( 607) := to_signed( -563, 11);
        r( 608) := to_signed( -568, 11); r( 609) := to_signed( -574, 11);
        r( 610) := to_signed( -579, 11); r( 611) := to_signed( -584, 11);
        r( 612) := to_signed( -589, 11); r( 613) := to_signed( -594, 11);
        r( 614) := to_signed( -599, 11); r( 615) := to_signed( -604, 11);
        r( 616) := to_signed( -609, 11); r( 617) := to_signed( -614, 11);
        r( 618) := to_signed( -619, 11); r( 619) := to_signed( -624, 11);
        r( 620) := to_signed( -629, 11); r( 621) := to_signed( -634, 11);
        r( 622) := to_signed( -639, 11); r( 623) := to_signed( -644, 11);
        r( 624) := to_signed( -649, 11); r( 625) := to_signed( -654, 11);
        r( 626) := to_signed( -659, 11); r( 627) := to_signed( -663, 11);
        r( 628) := to_signed( -668, 11); r( 629) := to_signed( -673, 11);
        r( 630) := to_signed( -678, 11); r( 631) := to_signed( -682, 11);
        r( 632) := to_signed( -687, 11); r( 633) := to_signed( -692, 11);
        r( 634) := to_signed( -696, 11); r( 635) := to_signed( -701, 11);
        r( 636) := to_signed( -705, 11); r( 637) := to_signed( -710, 11);
        r( 638) := to_signed( -714, 11); r( 639) := to_signed( -719, 11);
        r( 640) := to_signed( -723, 11); r( 641) := to_signed( -728, 11);
        r( 642) := to_signed( -732, 11); r( 643) := to_signed( -737, 11);
        r( 644) := to_signed( -741, 11); r( 645) := to_signed( -745, 11);
        r( 646) := to_signed( -750, 11); r( 647) := to_signed( -754, 11);
        r( 648) := to_signed( -758, 11); r( 649) := to_signed( -762, 11);
        r( 650) := to_signed( -766, 11); r( 651) := to_signed( -771, 11);
        r( 652) := to_signed( -775, 11); r( 653) := to_signed( -779, 11);
        r( 654) := to_signed( -783, 11); r( 655) := to_signed( -787, 11);
        r( 656) := to_signed( -791, 11); r( 657) := to_signed( -795, 11);
        r( 658) := to_signed( -799, 11); r( 659) := to_signed( -803, 11);
        r( 660) := to_signed( -806, 11); r( 661) := to_signed( -810, 11);
        r( 662) := to_signed( -814, 11); r( 663) := to_signed( -818, 11);
        r( 664) := to_signed( -822, 11); r( 665) := to_signed( -825, 11);
        r( 666) := to_signed( -829, 11); r( 667) := to_signed( -833, 11);
        r( 668) := to_signed( -836, 11); r( 669) := to_signed( -840, 11);
        r( 670) := to_signed( -844, 11); r( 671) := to_signed( -847, 11);
        r( 672) := to_signed( -851, 11); r( 673) := to_signed( -854, 11);
        r( 674) := to_signed( -858, 11); r( 675) := to_signed( -861, 11);
        r( 676) := to_signed( -864, 11); r( 677) := to_signed( -868, 11);
        r( 678) := to_signed( -871, 11); r( 679) := to_signed( -874, 11);
        r( 680) := to_signed( -877, 11); r( 681) := to_signed( -881, 11);
        r( 682) := to_signed( -884, 11); r( 683) := to_signed( -887, 11);
        r( 684) := to_signed( -890, 11); r( 685) := to_signed( -893, 11);
        r( 686) := to_signed( -896, 11); r( 687) := to_signed( -899, 11);
        r( 688) := to_signed( -902, 11); r( 689) := to_signed( -905, 11);
        r( 690) := to_signed( -908, 11); r( 691) := to_signed( -911, 11);
        r( 692) := to_signed( -914, 11); r( 693) := to_signed( -917, 11);
        r( 694) := to_signed( -919, 11); r( 695) := to_signed( -922, 11);
        r( 696) := to_signed( -925, 11); r( 697) := to_signed( -927, 11);
        r( 698) := to_signed( -930, 11); r( 699) := to_signed( -933, 11);
        r( 700) := to_signed( -935, 11); r( 701) := to_signed( -938, 11);
        r( 702) := to_signed( -940, 11); r( 703) := to_signed( -943, 11);
        r( 704) := to_signed( -945, 11); r( 705) := to_signed( -948, 11);
        r( 706) := to_signed( -950, 11); r( 707) := to_signed( -952, 11);
        r( 708) := to_signed( -954, 11); r( 709) := to_signed( -957, 11);
        r( 710) := to_signed( -959, 11); r( 711) := to_signed( -961, 11);
        r( 712) := to_signed( -963, 11); r( 713) := to_signed( -965, 11);
        r( 714) := to_signed( -967, 11); r( 715) := to_signed( -969, 11);
        r( 716) := to_signed( -971, 11); r( 717) := to_signed( -973, 11);
        r( 718) := to_signed( -975, 11); r( 719) := to_signed( -977, 11);
        r( 720) := to_signed( -979, 11); r( 721) := to_signed( -981, 11);
        r( 722) := to_signed( -983, 11); r( 723) := to_signed( -984, 11);
        r( 724) := to_signed( -986, 11); r( 725) := to_signed( -988, 11);
        r( 726) := to_signed( -989, 11); r( 727) := to_signed( -991, 11);
        r( 728) := to_signed( -992, 11); r( 729) := to_signed( -994, 11);
        r( 730) := to_signed( -995, 11); r( 731) := to_signed( -997, 11);
        r( 732) := to_signed( -998, 11); r( 733) := to_signed( -999, 11);
        r( 734) := to_signed(-1001, 11); r( 735) := to_signed(-1002, 11);
        r( 736) := to_signed(-1003, 11); r( 737) := to_signed(-1005, 11);
        r( 738) := to_signed(-1006, 11); r( 739) := to_signed(-1007, 11);
        r( 740) := to_signed(-1008, 11); r( 741) := to_signed(-1009, 11);
        r( 742) := to_signed(-1010, 11); r( 743) := to_signed(-1011, 11);
        r( 744) := to_signed(-1012, 11); r( 745) := to_signed(-1013, 11);
        r( 746) := to_signed(-1014, 11); r( 747) := to_signed(-1015, 11);
        r( 748) := to_signed(-1015, 11); r( 749) := to_signed(-1016, 11);
        r( 750) := to_signed(-1017, 11); r( 751) := to_signed(-1017, 11);
        r( 752) := to_signed(-1018, 11); r( 753) := to_signed(-1019, 11);
        r( 754) := to_signed(-1019, 11); r( 755) := to_signed(-1020, 11);
        r( 756) := to_signed(-1020, 11); r( 757) := to_signed(-1021, 11);
        r( 758) := to_signed(-1021, 11); r( 759) := to_signed(-1021, 11);
        r( 760) := to_signed(-1022, 11); r( 761) := to_signed(-1022, 11);
        r( 762) := to_signed(-1022, 11); r( 763) := to_signed(-1023, 11);
        r( 764) := to_signed(-1023, 11); r( 765) := to_signed(-1023, 11);
        r( 766) := to_signed(-1023, 11); r( 767) := to_signed(-1023, 11);
        r( 768) := to_signed(-1023, 11); r( 769) := to_signed(-1023, 11);
        r( 770) := to_signed(-1023, 11); r( 771) := to_signed(-1023, 11);
        r( 772) := to_signed(-1023, 11); r( 773) := to_signed(-1023, 11);
        r( 774) := to_signed(-1022, 11); r( 775) := to_signed(-1022, 11);
        r( 776) := to_signed(-1022, 11); r( 777) := to_signed(-1021, 11);
        r( 778) := to_signed(-1021, 11); r( 779) := to_signed(-1021, 11);
        r( 780) := to_signed(-1020, 11); r( 781) := to_signed(-1020, 11);
        r( 782) := to_signed(-1019, 11); r( 783) := to_signed(-1019, 11);
        r( 784) := to_signed(-1018, 11); r( 785) := to_signed(-1017, 11);
        r( 786) := to_signed(-1017, 11); r( 787) := to_signed(-1016, 11);
        r( 788) := to_signed(-1015, 11); r( 789) := to_signed(-1015, 11);
        r( 790) := to_signed(-1014, 11); r( 791) := to_signed(-1013, 11);
        r( 792) := to_signed(-1012, 11); r( 793) := to_signed(-1011, 11);
        r( 794) := to_signed(-1010, 11); r( 795) := to_signed(-1009, 11);
        r( 796) := to_signed(-1008, 11); r( 797) := to_signed(-1007, 11);
        r( 798) := to_signed(-1006, 11); r( 799) := to_signed(-1005, 11);
        r( 800) := to_signed(-1003, 11); r( 801) := to_signed(-1002, 11);
        r( 802) := to_signed(-1001, 11); r( 803) := to_signed( -999, 11);
        r( 804) := to_signed( -998, 11); r( 805) := to_signed( -997, 11);
        r( 806) := to_signed( -995, 11); r( 807) := to_signed( -994, 11);
        r( 808) := to_signed( -992, 11); r( 809) := to_signed( -991, 11);
        r( 810) := to_signed( -989, 11); r( 811) := to_signed( -988, 11);
        r( 812) := to_signed( -986, 11); r( 813) := to_signed( -984, 11);
        r( 814) := to_signed( -983, 11); r( 815) := to_signed( -981, 11);
        r( 816) := to_signed( -979, 11); r( 817) := to_signed( -977, 11);
        r( 818) := to_signed( -975, 11); r( 819) := to_signed( -973, 11);
        r( 820) := to_signed( -971, 11); r( 821) := to_signed( -969, 11);
        r( 822) := to_signed( -967, 11); r( 823) := to_signed( -965, 11);
        r( 824) := to_signed( -963, 11); r( 825) := to_signed( -961, 11);
        r( 826) := to_signed( -959, 11); r( 827) := to_signed( -957, 11);
        r( 828) := to_signed( -954, 11); r( 829) := to_signed( -952, 11);
        r( 830) := to_signed( -950, 11); r( 831) := to_signed( -948, 11);
        r( 832) := to_signed( -945, 11); r( 833) := to_signed( -943, 11);
        r( 834) := to_signed( -940, 11); r( 835) := to_signed( -938, 11);
        r( 836) := to_signed( -935, 11); r( 837) := to_signed( -933, 11);
        r( 838) := to_signed( -930, 11); r( 839) := to_signed( -927, 11);
        r( 840) := to_signed( -925, 11); r( 841) := to_signed( -922, 11);
        r( 842) := to_signed( -919, 11); r( 843) := to_signed( -917, 11);
        r( 844) := to_signed( -914, 11); r( 845) := to_signed( -911, 11);
        r( 846) := to_signed( -908, 11); r( 847) := to_signed( -905, 11);
        r( 848) := to_signed( -902, 11); r( 849) := to_signed( -899, 11);
        r( 850) := to_signed( -896, 11); r( 851) := to_signed( -893, 11);
        r( 852) := to_signed( -890, 11); r( 853) := to_signed( -887, 11);
        r( 854) := to_signed( -884, 11); r( 855) := to_signed( -881, 11);
        r( 856) := to_signed( -877, 11); r( 857) := to_signed( -874, 11);
        r( 858) := to_signed( -871, 11); r( 859) := to_signed( -868, 11);
        r( 860) := to_signed( -864, 11); r( 861) := to_signed( -861, 11);
        r( 862) := to_signed( -858, 11); r( 863) := to_signed( -854, 11);
        r( 864) := to_signed( -851, 11); r( 865) := to_signed( -847, 11);
        r( 866) := to_signed( -844, 11); r( 867) := to_signed( -840, 11);
        r( 868) := to_signed( -836, 11); r( 869) := to_signed( -833, 11);
        r( 870) := to_signed( -829, 11); r( 871) := to_signed( -825, 11);
        r( 872) := to_signed( -822, 11); r( 873) := to_signed( -818, 11);
        r( 874) := to_signed( -814, 11); r( 875) := to_signed( -810, 11);
        r( 876) := to_signed( -806, 11); r( 877) := to_signed( -803, 11);
        r( 878) := to_signed( -799, 11); r( 879) := to_signed( -795, 11);
        r( 880) := to_signed( -791, 11); r( 881) := to_signed( -787, 11);
        r( 882) := to_signed( -783, 11); r( 883) := to_signed( -779, 11);
        r( 884) := to_signed( -775, 11); r( 885) := to_signed( -771, 11);
        r( 886) := to_signed( -766, 11); r( 887) := to_signed( -762, 11);
        r( 888) := to_signed( -758, 11); r( 889) := to_signed( -754, 11);
        r( 890) := to_signed( -750, 11); r( 891) := to_signed( -745, 11);
        r( 892) := to_signed( -741, 11); r( 893) := to_signed( -737, 11);
        r( 894) := to_signed( -732, 11); r( 895) := to_signed( -728, 11);
        r( 896) := to_signed( -723, 11); r( 897) := to_signed( -719, 11);
        r( 898) := to_signed( -714, 11); r( 899) := to_signed( -710, 11);
        r( 900) := to_signed( -705, 11); r( 901) := to_signed( -701, 11);
        r( 902) := to_signed( -696, 11); r( 903) := to_signed( -692, 11);
        r( 904) := to_signed( -687, 11); r( 905) := to_signed( -682, 11);
        r( 906) := to_signed( -678, 11); r( 907) := to_signed( -673, 11);
        r( 908) := to_signed( -668, 11); r( 909) := to_signed( -663, 11);
        r( 910) := to_signed( -659, 11); r( 911) := to_signed( -654, 11);
        r( 912) := to_signed( -649, 11); r( 913) := to_signed( -644, 11);
        r( 914) := to_signed( -639, 11); r( 915) := to_signed( -634, 11);
        r( 916) := to_signed( -629, 11); r( 917) := to_signed( -624, 11);
        r( 918) := to_signed( -619, 11); r( 919) := to_signed( -614, 11);
        r( 920) := to_signed( -609, 11); r( 921) := to_signed( -604, 11);
        r( 922) := to_signed( -599, 11); r( 923) := to_signed( -594, 11);
        r( 924) := to_signed( -589, 11); r( 925) := to_signed( -584, 11);
        r( 926) := to_signed( -579, 11); r( 927) := to_signed( -574, 11);
        r( 928) := to_signed( -568, 11); r( 929) := to_signed( -563, 11);
        r( 930) := to_signed( -558, 11); r( 931) := to_signed( -553, 11);
        r( 932) := to_signed( -547, 11); r( 933) := to_signed( -542, 11);
        r( 934) := to_signed( -537, 11); r( 935) := to_signed( -531, 11);
        r( 936) := to_signed( -526, 11); r( 937) := to_signed( -521, 11);
        r( 938) := to_signed( -515, 11); r( 939) := to_signed( -510, 11);
        r( 940) := to_signed( -504, 11); r( 941) := to_signed( -499, 11);
        r( 942) := to_signed( -493, 11); r( 943) := to_signed( -488, 11);
        r( 944) := to_signed( -482, 11); r( 945) := to_signed( -477, 11);
        r( 946) := to_signed( -471, 11); r( 947) := to_signed( -466, 11);
        r( 948) := to_signed( -460, 11); r( 949) := to_signed( -454, 11);
        r( 950) := to_signed( -449, 11); r( 951) := to_signed( -443, 11);
        r( 952) := to_signed( -437, 11); r( 953) := to_signed( -432, 11);
        r( 954) := to_signed( -426, 11); r( 955) := to_signed( -420, 11);
        r( 956) := to_signed( -415, 11); r( 957) := to_signed( -409, 11);
        r( 958) := to_signed( -403, 11); r( 959) := to_signed( -397, 11);
        r( 960) := to_signed( -391, 11); r( 961) := to_signed( -386, 11);
        r( 962) := to_signed( -380, 11); r( 963) := to_signed( -374, 11);
        r( 964) := to_signed( -368, 11); r( 965) := to_signed( -362, 11);
        r( 966) := to_signed( -356, 11); r( 967) := to_signed( -351, 11);
        r( 968) := to_signed( -345, 11); r( 969) := to_signed( -339, 11);
        r( 970) := to_signed( -333, 11); r( 971) := to_signed( -327, 11);
        r( 972) := to_signed( -321, 11); r( 973) := to_signed( -315, 11);
        r( 974) := to_signed( -309, 11); r( 975) := to_signed( -303, 11);
        r( 976) := to_signed( -297, 11); r( 977) := to_signed( -291, 11);
        r( 978) := to_signed( -285, 11); r( 979) := to_signed( -279, 11);
        r( 980) := to_signed( -273, 11); r( 981) := to_signed( -267, 11);
        r( 982) := to_signed( -261, 11); r( 983) := to_signed( -255, 11);
        r( 984) := to_signed( -249, 11); r( 985) := to_signed( -242, 11);
        r( 986) := to_signed( -236, 11); r( 987) := to_signed( -230, 11);
        r( 988) := to_signed( -224, 11); r( 989) := to_signed( -218, 11);
        r( 990) := to_signed( -212, 11); r( 991) := to_signed( -206, 11);
        r( 992) := to_signed( -200, 11); r( 993) := to_signed( -193, 11);
        r( 994) := to_signed( -187, 11); r( 995) := to_signed( -181, 11);
        r( 996) := to_signed( -175, 11); r( 997) := to_signed( -169, 11);
        r( 998) := to_signed( -163, 11); r( 999) := to_signed( -156, 11);
        r(1000) := to_signed( -150, 11); r(1001) := to_signed( -144, 11);
        r(1002) := to_signed( -138, 11); r(1003) := to_signed( -131, 11);
        r(1004) := to_signed( -125, 11); r(1005) := to_signed( -119, 11);
        r(1006) := to_signed( -113, 11); r(1007) := to_signed( -107, 11);
        r(1008) := to_signed( -100, 11); r(1009) := to_signed(  -94, 11);
        r(1010) := to_signed(  -88, 11); r(1011) := to_signed(  -82, 11);
        r(1012) := to_signed(  -75, 11); r(1013) := to_signed(  -69, 11);
        r(1014) := to_signed(  -63, 11); r(1015) := to_signed(  -56, 11);
        r(1016) := to_signed(  -50, 11); r(1017) := to_signed(  -44, 11);
        r(1018) := to_signed(  -38, 11); r(1019) := to_signed(  -31, 11);
        r(1020) := to_signed(  -25, 11); r(1021) := to_signed(  -19, 11);
        r(1022) := to_signed(  -13, 11); r(1023) := to_signed(   -6, 11);
        return r;
    end function;

    constant C_SIN_FULL : t_sin_full := gen_sin_full;

    --------------------------------------------------------------------------
    -- Reciprocal LUT for dx_dy compute. 256 entries of recip[i] = 4096/i.
    -- For dy >= 256, the issuing logic right-shifts dy first to fit the
    -- table, then right-shifts the looked-up value by the same amount,
    -- since recip(2k * dy_hi) = recip(dy_hi) >> k. Table is read with a
    -- synchronous BRAM read pattern.
    --------------------------------------------------------------------------

    type t_recip is array(0 to 255) of unsigned(13 downto 0);
    function gen_recip return t_recip is
        variable r : t_recip;
    begin
        r(  0) := to_unsigned( 4095, 14); r(  1) := to_unsigned( 4095, 14);
        r(  2) := to_unsigned( 2048, 14); r(  3) := to_unsigned( 1365, 14);
        r(  4) := to_unsigned( 1024, 14); r(  5) := to_unsigned(  819, 14);
        r(  6) := to_unsigned(  682, 14); r(  7) := to_unsigned(  585, 14);
        r(  8) := to_unsigned(  512, 14); r(  9) := to_unsigned(  455, 14);
        r( 10) := to_unsigned(  409, 14); r( 11) := to_unsigned(  372, 14);
        r( 12) := to_unsigned(  341, 14); r( 13) := to_unsigned(  315, 14);
        r( 14) := to_unsigned(  292, 14); r( 15) := to_unsigned(  273, 14);
        r( 16) := to_unsigned(  256, 14); r( 17) := to_unsigned(  240, 14);
        r( 18) := to_unsigned(  227, 14); r( 19) := to_unsigned(  215, 14);
        r( 20) := to_unsigned(  204, 14); r( 21) := to_unsigned(  195, 14);
        r( 22) := to_unsigned(  186, 14); r( 23) := to_unsigned(  178, 14);
        r( 24) := to_unsigned(  170, 14); r( 25) := to_unsigned(  163, 14);
        r( 26) := to_unsigned(  157, 14); r( 27) := to_unsigned(  151, 14);
        r( 28) := to_unsigned(  146, 14); r( 29) := to_unsigned(  141, 14);
        r( 30) := to_unsigned(  136, 14); r( 31) := to_unsigned(  132, 14);
        r( 32) := to_unsigned(  128, 14); r( 33) := to_unsigned(  124, 14);
        r( 34) := to_unsigned(  120, 14); r( 35) := to_unsigned(  117, 14);
        r( 36) := to_unsigned(  113, 14); r( 37) := to_unsigned(  110, 14);
        r( 38) := to_unsigned(  107, 14); r( 39) := to_unsigned(  105, 14);
        r( 40) := to_unsigned(  102, 14); r( 41) := to_unsigned(   99, 14);
        r( 42) := to_unsigned(   97, 14); r( 43) := to_unsigned(   95, 14);
        r( 44) := to_unsigned(   93, 14); r( 45) := to_unsigned(   91, 14);
        r( 46) := to_unsigned(   89, 14); r( 47) := to_unsigned(   87, 14);
        r( 48) := to_unsigned(   85, 14); r( 49) := to_unsigned(   83, 14);
        r( 50) := to_unsigned(   81, 14); r( 51) := to_unsigned(   80, 14);
        r( 52) := to_unsigned(   78, 14); r( 53) := to_unsigned(   77, 14);
        r( 54) := to_unsigned(   75, 14); r( 55) := to_unsigned(   74, 14);
        r( 56) := to_unsigned(   73, 14); r( 57) := to_unsigned(   71, 14);
        r( 58) := to_unsigned(   70, 14); r( 59) := to_unsigned(   69, 14);
        r( 60) := to_unsigned(   68, 14); r( 61) := to_unsigned(   67, 14);
        r( 62) := to_unsigned(   66, 14); r( 63) := to_unsigned(   65, 14);
        r( 64) := to_unsigned(   64, 14); r( 65) := to_unsigned(   63, 14);
        r( 66) := to_unsigned(   62, 14); r( 67) := to_unsigned(   61, 14);
        r( 68) := to_unsigned(   60, 14); r( 69) := to_unsigned(   59, 14);
        r( 70) := to_unsigned(   58, 14); r( 71) := to_unsigned(   57, 14);
        r( 72) := to_unsigned(   56, 14); r( 73) := to_unsigned(   56, 14);
        r( 74) := to_unsigned(   55, 14); r( 75) := to_unsigned(   54, 14);
        r( 76) := to_unsigned(   53, 14); r( 77) := to_unsigned(   53, 14);
        r( 78) := to_unsigned(   52, 14); r( 79) := to_unsigned(   51, 14);
        r( 80) := to_unsigned(   51, 14); r( 81) := to_unsigned(   50, 14);
        r( 82) := to_unsigned(   49, 14); r( 83) := to_unsigned(   49, 14);
        r( 84) := to_unsigned(   48, 14); r( 85) := to_unsigned(   48, 14);
        r( 86) := to_unsigned(   47, 14); r( 87) := to_unsigned(   47, 14);
        r( 88) := to_unsigned(   46, 14); r( 89) := to_unsigned(   46, 14);
        r( 90) := to_unsigned(   45, 14); r( 91) := to_unsigned(   45, 14);
        r( 92) := to_unsigned(   44, 14); r( 93) := to_unsigned(   44, 14);
        r( 94) := to_unsigned(   43, 14); r( 95) := to_unsigned(   43, 14);
        r( 96) := to_unsigned(   42, 14); r( 97) := to_unsigned(   42, 14);
        r( 98) := to_unsigned(   41, 14); r( 99) := to_unsigned(   41, 14);
        r(100) := to_unsigned(   40, 14); r(101) := to_unsigned(   40, 14);
        r(102) := to_unsigned(   40, 14); r(103) := to_unsigned(   39, 14);
        r(104) := to_unsigned(   39, 14); r(105) := to_unsigned(   39, 14);
        r(106) := to_unsigned(   38, 14); r(107) := to_unsigned(   38, 14);
        r(108) := to_unsigned(   37, 14); r(109) := to_unsigned(   37, 14);
        r(110) := to_unsigned(   37, 14); r(111) := to_unsigned(   36, 14);
        r(112) := to_unsigned(   36, 14); r(113) := to_unsigned(   36, 14);
        r(114) := to_unsigned(   35, 14); r(115) := to_unsigned(   35, 14);
        r(116) := to_unsigned(   35, 14); r(117) := to_unsigned(   35, 14);
        r(118) := to_unsigned(   34, 14); r(119) := to_unsigned(   34, 14);
        r(120) := to_unsigned(   34, 14); r(121) := to_unsigned(   33, 14);
        r(122) := to_unsigned(   33, 14); r(123) := to_unsigned(   33, 14);
        r(124) := to_unsigned(   33, 14); r(125) := to_unsigned(   32, 14);
        r(126) := to_unsigned(   32, 14); r(127) := to_unsigned(   32, 14);
        r(128) := to_unsigned(   32, 14); r(129) := to_unsigned(   31, 14);
        r(130) := to_unsigned(   31, 14); r(131) := to_unsigned(   31, 14);
        r(132) := to_unsigned(   31, 14); r(133) := to_unsigned(   30, 14);
        r(134) := to_unsigned(   30, 14); r(135) := to_unsigned(   30, 14);
        r(136) := to_unsigned(   30, 14); r(137) := to_unsigned(   29, 14);
        r(138) := to_unsigned(   29, 14); r(139) := to_unsigned(   29, 14);
        r(140) := to_unsigned(   29, 14); r(141) := to_unsigned(   29, 14);
        r(142) := to_unsigned(   28, 14); r(143) := to_unsigned(   28, 14);
        r(144) := to_unsigned(   28, 14); r(145) := to_unsigned(   28, 14);
        r(146) := to_unsigned(   28, 14); r(147) := to_unsigned(   27, 14);
        r(148) := to_unsigned(   27, 14); r(149) := to_unsigned(   27, 14);
        r(150) := to_unsigned(   27, 14); r(151) := to_unsigned(   27, 14);
        r(152) := to_unsigned(   26, 14); r(153) := to_unsigned(   26, 14);
        r(154) := to_unsigned(   26, 14); r(155) := to_unsigned(   26, 14);
        r(156) := to_unsigned(   26, 14); r(157) := to_unsigned(   26, 14);
        r(158) := to_unsigned(   25, 14); r(159) := to_unsigned(   25, 14);
        r(160) := to_unsigned(   25, 14); r(161) := to_unsigned(   25, 14);
        r(162) := to_unsigned(   25, 14); r(163) := to_unsigned(   25, 14);
        r(164) := to_unsigned(   24, 14); r(165) := to_unsigned(   24, 14);
        r(166) := to_unsigned(   24, 14); r(167) := to_unsigned(   24, 14);
        r(168) := to_unsigned(   24, 14); r(169) := to_unsigned(   24, 14);
        r(170) := to_unsigned(   24, 14); r(171) := to_unsigned(   23, 14);
        r(172) := to_unsigned(   23, 14); r(173) := to_unsigned(   23, 14);
        r(174) := to_unsigned(   23, 14); r(175) := to_unsigned(   23, 14);
        r(176) := to_unsigned(   23, 14); r(177) := to_unsigned(   23, 14);
        r(178) := to_unsigned(   23, 14); r(179) := to_unsigned(   22, 14);
        r(180) := to_unsigned(   22, 14); r(181) := to_unsigned(   22, 14);
        r(182) := to_unsigned(   22, 14); r(183) := to_unsigned(   22, 14);
        r(184) := to_unsigned(   22, 14); r(185) := to_unsigned(   22, 14);
        r(186) := to_unsigned(   22, 14); r(187) := to_unsigned(   21, 14);
        r(188) := to_unsigned(   21, 14); r(189) := to_unsigned(   21, 14);
        r(190) := to_unsigned(   21, 14); r(191) := to_unsigned(   21, 14);
        r(192) := to_unsigned(   21, 14); r(193) := to_unsigned(   21, 14);
        r(194) := to_unsigned(   21, 14); r(195) := to_unsigned(   21, 14);
        r(196) := to_unsigned(   20, 14); r(197) := to_unsigned(   20, 14);
        r(198) := to_unsigned(   20, 14); r(199) := to_unsigned(   20, 14);
        r(200) := to_unsigned(   20, 14); r(201) := to_unsigned(   20, 14);
        r(202) := to_unsigned(   20, 14); r(203) := to_unsigned(   20, 14);
        r(204) := to_unsigned(   20, 14); r(205) := to_unsigned(   19, 14);
        r(206) := to_unsigned(   19, 14); r(207) := to_unsigned(   19, 14);
        r(208) := to_unsigned(   19, 14); r(209) := to_unsigned(   19, 14);
        r(210) := to_unsigned(   19, 14); r(211) := to_unsigned(   19, 14);
        r(212) := to_unsigned(   19, 14); r(213) := to_unsigned(   19, 14);
        r(214) := to_unsigned(   19, 14); r(215) := to_unsigned(   19, 14);
        r(216) := to_unsigned(   18, 14); r(217) := to_unsigned(   18, 14);
        r(218) := to_unsigned(   18, 14); r(219) := to_unsigned(   18, 14);
        r(220) := to_unsigned(   18, 14); r(221) := to_unsigned(   18, 14);
        r(222) := to_unsigned(   18, 14); r(223) := to_unsigned(   18, 14);
        r(224) := to_unsigned(   18, 14); r(225) := to_unsigned(   18, 14);
        r(226) := to_unsigned(   18, 14); r(227) := to_unsigned(   18, 14);
        r(228) := to_unsigned(   17, 14); r(229) := to_unsigned(   17, 14);
        r(230) := to_unsigned(   17, 14); r(231) := to_unsigned(   17, 14);
        r(232) := to_unsigned(   17, 14); r(233) := to_unsigned(   17, 14);
        r(234) := to_unsigned(   17, 14); r(235) := to_unsigned(   17, 14);
        r(236) := to_unsigned(   17, 14); r(237) := to_unsigned(   17, 14);
        r(238) := to_unsigned(   17, 14); r(239) := to_unsigned(   17, 14);
        r(240) := to_unsigned(   17, 14); r(241) := to_unsigned(   16, 14);
        r(242) := to_unsigned(   16, 14); r(243) := to_unsigned(   16, 14);
        r(244) := to_unsigned(   16, 14); r(245) := to_unsigned(   16, 14);
        r(246) := to_unsigned(   16, 14); r(247) := to_unsigned(   16, 14);
        r(248) := to_unsigned(   16, 14); r(249) := to_unsigned(   16, 14);
        r(250) := to_unsigned(   16, 14); r(251) := to_unsigned(   16, 14);
        r(252) := to_unsigned(   16, 14); r(253) := to_unsigned(   16, 14);
        r(254) := to_unsigned(   16, 14); r(255) := to_unsigned(   16, 14);
        return r;
    end function;

    constant C_RECIP : t_recip := gen_recip;



    --------------------------------------------------------------------------
    -- Position counters and timing.
    --------------------------------------------------------------------------
    signal s_active_pixel  : unsigned(11 downto 0) := (others => '0');
    signal s_active_line   : unsigned(11 downto 0) := (others => '0');
    signal s_prev_hsync_n  : std_logic := '1';
    signal s_prev_vsync_n  : std_logic := '1';
    signal s_prev_avid     : std_logic := '0';
    signal s_active_width  : unsigned(11 downto 0) := to_unsigned(720, 12);
    signal s_active_height : unsigned(11 downto 0) := to_unsigned(486, 12);
    signal s_frame_count   : unsigned(15 downto 0) := (others => '0');
    signal s_vsync_edge    : std_logic := '0';
    signal s_hsync_edge    : std_logic := '0';

    --------------------------------------------------------------------------
    -- Parameters.
    --------------------------------------------------------------------------
    signal s_speed        : unsigned(9 downto 0);
    signal s_twist4d      : unsigned(9 downto 0);
    signal s_size  : unsigned(9 downto 0);
    signal s_cell_tint    : unsigned(9 downto 0);
    signal s_edge_w_idx   : unsigned(2 downto 0);
    signal s_edge_w       : unsigned(4 downto 0);
    signal s_edge_bright  : unsigned(9 downto 0);
    signal s_sw_wire      : std_logic;
    signal s_sw_bg_video  : std_logic;
    signal s_sw_cellfill  : std_logic;
    signal s_sw_edgewhite : std_logic;
    signal s_sw_reverse   : std_logic;
    signal s_brightness   : unsigned(9 downto 0);

    --------------------------------------------------------------------------
    -- Rotation angle accumulators and per-frame sin/cos.
    --------------------------------------------------------------------------
    type t_angles is array(0 to 5) of unsigned(9 downto 0);
    signal s_ang : t_angles := (others => (others => '0'));

    type t_sc is array(0 to 5) of signed(10 downto 0);
    signal s_sin : t_sc := (others => (others => '0'));
    signal s_cos : t_sc := (others => (others => '0'));

    -- Sin LUT BRAM read pattern.
    signal s_sin_addr : unsigned(9 downto 0) := (others => '0');
    signal s_sin_q    : signed(10 downto 0)  := (others => '0');

    -- Recip LUT BRAM read pattern.
    signal s_recip_addr : unsigned(7 downto 0)  := (others => '0');
    signal s_recip_q    : unsigned(13 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Vertex compute state machine state.
    --------------------------------------------------------------------------
    type t_vphase is (
        VS_IDLE, VS_LUTLOAD, VS_RAST_CLR,
        VS_RAST_LOAD, VS_RAST_LOAD2,
        VS_RAST_PLOT, VS_RAST_STEP, VS_DONE);
    signal s_vphase     : t_vphase := VS_IDLE;
    signal s_v_counter  : integer range 0 to 1023 := 0;

    -- v6: phase counter advances each frame; addresses precomputed
    -- vertex/cell position tables. Speed knob (top 4b) drives the
    -- per-frame step size.
    signal s_phase     : unsigned(5 downto 0)  := (others => '0');
    -- 16-bit fractional phase accumulator. Top 6 bits = s_phase index;
    -- low 10 bits give sub-phase resolution. Per frame the accumulator
    -- advances by the 10-bit Speed knob value, so the full knob range
    -- covers ~1 second per revolution at 60 fps (max), down to glacial.
    signal s_phase_acc : unsigned(15 downto 0) := (others => '0');

    -- Phase table BRAM read signals.
    signal s_vsx_addr : unsigned(9 downto 0) := (others => '0');
    signal s_vsx_data : signed(12 downto 0)  := (others => '0');
    signal s_vsy_addr : unsigned(9 downto 0) := (others => '0');
    signal s_vsy_data : signed(12 downto 0)  := (others => '0');
    signal s_csx_addr : unsigned(8 downto 0) := (others => '0');
    signal s_csx_data : signed(12 downto 0)  := (others => '0');
    signal s_csy_addr : unsigned(8 downto 0) := (others => '0');
    signal s_csy_data : signed(12 downto 0)  := (others => '0');

    -- Working register file: 24 elements x 4 axes of t_fx.
    signal s_work : t_v4_arr(0 to N_TOTAL-1) := (others => (others => (others => '0')));

    -- Output (post-projection) screen coords.
    signal s_vert_sx : t_px_arr(0 to 15) := (others => (others => '0'));
    signal s_vert_sy : t_px_arr(0 to 15) := (others => (others => '0'));
    signal s_cell_sx : t_px_arr(0 to 7)  := (others => (others => '0'));
    signal s_cell_sy : t_px_arr(0 to 7)  := (others => (others => '0'));

    -- Edge slope storage (Q4.10 fixed slope).
    signal s_edge_slope : t_slope_arr(0 to 31) := (others => (others => '0'));

    -- Edge geometry derived combinationally from vertex coords (registered
    -- after vertex compute completes).
    signal s_edge_x_at_ymin : t_px_arr(0 to 31) := (others => (others => '0'));
    signal s_edge_y_min     : t_px_arr(0 to 31) := (others => (others => '0'));
    signal s_edge_y_max     : t_px_arr(0 to 31) := (others => (others => '0'));
    signal s_edge_dx        : signed(13 downto 0) := (others => '0');  -- scratch
    signal s_edge_dy_abs    : unsigned(10 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Shared multiplier_s instance and result-tag pipeline.
    --------------------------------------------------------------------------
    signal s_mul_x      : signed(FX_W-1 downto 0) := (others => '0');
    signal s_mul_y      : signed(FX_W-1 downto 0) := (others => '0');
    signal s_mul_z      : signed(FX_W-1 downto 0) := (others => '0');
    signal s_mul_enable : std_logic := '0';
    signal s_mul_result : signed(FX_W-1 downto 0);
    signal s_mul_valid  : std_logic;

    -- Descriptor that travels with each in-flight mul.
    type t_op_kind is (OP_NONE, OP_ROT, OP_PROJ_X, OP_PROJ_Y, OP_DXDY);
    type t_descriptor is record
        op     : t_op_kind;
        elem   : integer range 0 to 31;     -- working set index (or edge idx)
        plane  : integer range 0 to 7;
        op_idx : integer range 0 to 3;      -- 0..3 within a rotation
        axis_a : integer range 0 to 3;
        axis_b : integer range 0 to 3;
    end record;
    constant C_DESC_NONE : t_descriptor :=
        (op=>OP_NONE, elem=>0, plane=>0, op_idx=>0, axis_a=>0, axis_b=>0);

    type t_desc_pipe is array(0 to MUL_LATENCY-1) of t_descriptor;
    signal s_desc_pipe : t_desc_pipe := (others => C_DESC_NONE);

    -- Rotation result aggregator.
    signal s_rot_tmp0 : t_fx := (others => '0');
    signal s_rot_tmp2 : t_fx := (others => '0');
    signal s_rot_combine_a : t_fx := (others => '0');

    -- Projection result aggregator.
    signal s_proj_x : signed(FX_W-1 downto 0) := (others => '0');
    signal s_proj_have_x : std_logic := '0';
    signal s_proj_elem_x : integer range 0 to 31 := 0;
    signal s_proj_w_scale : signed(FX_W-1 downto 0) := (others => '0');

    -- DXDY 2-cycle pipeline registers (carry from VS_DXDY_RD to VS_DXDY).
    signal s_dxdy_dx_prev     : signed(13 downto 0) := (others => '0');
    signal s_dxdy_octave_prev : integer range 0 to 3 := 0;
    signal s_dxdy_edge_prev   : integer range 0 to 31 := 0;

    --------------------------------------------------------------------------
    -- Per-scanline edge state.
    --------------------------------------------------------------------------
    --------------------------------------------------------------------------
    -- Low-resolution edge framebuffer (256 x 144, 1 bit per pixel).
    -- Filled by a Bresenham state machine during vblank; read by the
    -- per-pixel pipeline using (line>>3, px>>3) as the address. yosys
    -- infers this as block RAM (~9 BRAMs).
    --------------------------------------------------------------------------
    constant EB_W      : natural := 256;
    constant EB_H      : natural := 144;
    constant EB_SIZE   : natural := EB_W * EB_H;
    constant EB_ADDR_W : natural := 16;  -- enough for EB_SIZE-1 = 36863
    type t_eb_ram is array(0 to EB_SIZE - 1) of std_logic;
    signal s_edge_buf : t_eb_ram := (others => '0');

    signal s_eb_wr_en   : std_logic := '0';
    signal s_eb_wr_addr : unsigned(EB_ADDR_W-1 downto 0) := (others => '0');
    signal s_eb_wr_data : std_logic := '0';
    signal s_eb_rd_addr : unsigned(EB_ADDR_W-1 downto 0) := (others => '0');
    signal s_eb_rd_data : std_logic := '0';

    -- Bresenham DDA state (one edge in flight at a time).
    signal s_dda_edge   : integer range 0 to 31 := 0;
    -- Pipeline registers for VS_RAST_LOAD -> VS_RAST_LOAD2 stage split.
    signal s_load_xa    : signed(8 downto 0) := (others => '0');
    signal s_load_ya    : signed(8 downto 0) := (others => '0');
    signal s_load_xb    : signed(8 downto 0) := (others => '0');
    signal s_load_yb    : signed(8 downto 0) := (others => '0');

    -- Bresenham step-cycle pipeline registers (TEST -> STEP split).
    signal s_step_x_flag : std_logic := '0';
    signal s_step_y_flag : std_logic := '0';

    -- LUTLOAD scaled-value pipeline registers (Size-scale -> add-center
    -- split). Splits the case mux + shift+add + center-add chain so HD
    -- timing closes.
    signal s_lut_scaled_x  : signed(14 downto 0) := (others => '0');
    signal s_lut_scaled_y  : signed(14 downto 0) := (others => '0');
    signal s_lut_scaled_cx : signed(14 downto 0) := (others => '0');
    signal s_lut_scaled_cy : signed(14 downto 0) := (others => '0');
    signal s_lut_capture_kind : std_logic_vector(1 downto 0) := "00";  -- 00 idle, 01 vert, 10 cell
    signal s_lut_capture_idx  : integer range 0 to 23 := 0;
    signal s_dda_x      : signed(8 downto 0) := (others => '0');
    signal s_dda_y      : signed(8 downto 0) := (others => '0');
    signal s_dda_x2     : signed(8 downto 0) := (others => '0');
    signal s_dda_y2     : signed(8 downto 0) := (others => '0');
    signal s_dda_dx     : signed(8 downto 0) := (others => '0');
    signal s_dda_dy     : signed(8 downto 0) := (others => '0');
    signal s_dda_sx     : signed(1 downto 0) := (others => '0');
    signal s_dda_sy     : signed(1 downto 0) := (others => '0');
    signal s_dda_err    : signed(9 downto 0) := (others => '0');

    -- Clear-pass counter and edge-of-pass flags handled inside VS_RAST*.
    signal s_clr_addr   : unsigned(EB_ADDR_W-1 downto 0) := (others => '0');

    --------------------------------------------------------------------------
    -- Per-pixel pipeline signals.
    -- LATENCY = 17. Stages numbered 1..17. Stage 0 is the input boundary.
    --------------------------------------------------------------------------

    -- Stage 1: per-cell, per-edge sub.
    type t_dxy is array(0 to 7) of signed(13 downto 0);
    signal p1_cell_dx : t_dxy := (others => (others => '0'));
    signal p1_cell_dy : t_dxy := (others => (others => '0'));
    type t_edge_diff is array(0 to 31) of signed(13 downto 0);
    signal p1_edge_diff : t_edge_diff := (others => (others => '0'));
    signal p1_edge_active : std_logic_vector(0 to 31) := (others => '0');

    -- Stage 2: per-cell, per-edge abs.
    type t_dxy_u is array(0 to 7) of unsigned(12 downto 0);
    signal p2_cell_adx : t_dxy_u := (others => (others => '0'));
    signal p2_cell_ady : t_dxy_u := (others => (others => '0'));
    type t_edge_abs is array(0 to 31) of unsigned(12 downto 0);
    signal p2_edge_adiff : t_edge_abs := (others => (others => '0'));
    signal p2_edge_active : std_logic_vector(0 to 31) := (others => '0');
    signal p2_thresh      : unsigned(12 downto 0) := (others => '0');

    -- Stage 3: per-cell sum (Manhattan), per-edge thresh compare.
    type t_dist8 is array(0 to 7) of unsigned(13 downto 0);
    signal p3_cell_dist : t_dist8 := (others => (others => '0'));
    signal p3_edge_hit  : std_logic_vector(0 to 31) := (others => '0');

    -- Stage 4: pair-min (4 winners), edge OR-tier-1 (4-way blocks).
    type t_min4 is array(0 to 3) of unsigned(13 downto 0);
    type t_idx4 is array(0 to 3) of unsigned(2 downto 0);
    signal p4_min : t_min4 := (others => (others => '0'));
    signal p4_idx : t_idx4 := (others => (others => '0'));
    signal p4_edge_or : std_logic_vector(0 to 7) := (others => '0');

    -- Stage 5: pair-min (2 winners), edge OR-tier-2 (8-way -> 2-way).
    type t_min2 is array(0 to 1) of unsigned(13 downto 0);
    type t_idx2 is array(0 to 1) of unsigned(2 downto 0);
    signal p5_min : t_min2 := (others => (others => '0'));
    signal p5_idx : t_idx2 := (others => (others => '0'));
    signal p5_edge_or : std_logic_vector(0 to 1) := (others => '0');

    -- Stage 6: final cell_id and edge_hit.
    signal p6_cell_id  : unsigned(2 downto 0) := (others => '0');
    signal p6_edge_hit : std_logic := '0';

    -- Stage 7: variant lookup + chroma rotate.
    -- Variant bits: bit0 = swap UV, bit1 = neg U, bit2 = neg V.
    function cell_variant(idx : integer) return std_logic_vector is
        variable r : std_logic_vector(2 downto 0);
    begin
        case idx is
            when 0      => r := "000";
            when 1      => r := "001";
            when 2      => r := "010";
            when 3      => r := "011";
            when 4      => r := "100";
            when 5      => r := "101";
            when 6      => r := "110";
            when others => r := "111";
        end case;
        return r;
    end function;

    signal p7_cell_id  : unsigned(2 downto 0) := (others => '0');
    signal p7_edge_hit : std_logic := '0';
    signal p7_u_xform  : unsigned(9 downto 0) := (others => '0');
    signal p7_v_xform  : unsigned(9 downto 0) := (others => '0');

    -- Stages 8..11: cell-tint interpolators (4 cycles each, instantiated).
    signal p11_u_mixed : unsigned(9 downto 0);
    signal p11_v_mixed : unsigned(9 downto 0);
    signal p11_edge_hit : std_logic := '0';
    signal p11_cell_id  : unsigned(2 downto 0) := (others => '0');

    -- Stage 12: composite output (Y, U, V) before brightness.
    signal p12_y : unsigned(9 downto 0) := (others => '0');
    signal p12_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p12_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- Stages 13..16: brightness interpolator on Y, plus 4-cycle delay on U/V.
    signal p16_y : unsigned(9 downto 0) := (others => '0');
    signal p16_u : unsigned(9 downto 0) := to_unsigned(512, 10);
    signal p16_v : unsigned(9 downto 0) := to_unsigned(512, 10);

    -- 4-cycle delay shift register for U, V to match brightness interpolator.
    type t_uvdel is array(0 to 3) of unsigned(9 downto 0);
    signal p_u_delay : t_uvdel := (others => to_unsigned(512, 10));
    signal p_v_delay : t_uvdel := (others => to_unsigned(512, 10));

    --------------------------------------------------------------------------
    -- Sync and YUV passthrough shift registers (depth = LATENCY).
    --------------------------------------------------------------------------
    signal s_avid_sr    : std_logic_vector(LATENCY-1 downto 0) := (others => '0');
    signal s_hsync_n_sr : std_logic_vector(LATENCY-1 downto 0) := (others => '1');
    signal s_vsync_n_sr : std_logic_vector(LATENCY-1 downto 0) := (others => '1');
    signal s_field_n_sr : std_logic_vector(LATENCY-1 downto 0) := (others => '1');

    type t_y_pipe is array(0 to LATENCY-1) of std_logic_vector(9 downto 0);
    signal s_y_sr : t_y_pipe := (others => (others => '0'));
    signal s_u_sr : t_y_pipe := (others => (9 => '1', others => '0'));
    signal s_v_sr : t_y_pipe := (others => (9 => '1', others => '0'));

begin

    --------------------------------------------------------------------------
    -- Parameters.
    --------------------------------------------------------------------------
    s_speed        <= unsigned(registers_in(0));
    s_twist4d      <= unsigned(registers_in(1));
    s_size  <= unsigned(registers_in(2));
    s_cell_tint    <= unsigned(registers_in(3));
    s_edge_w_idx   <= unsigned(registers_in(4)(9 downto 7));
    s_edge_bright  <= unsigned(registers_in(5));
    s_sw_wire      <= registers_in(6)(0);
    s_sw_bg_video  <= registers_in(6)(1);
    s_sw_cellfill  <= registers_in(6)(2);
    s_sw_edgewhite <= registers_in(6)(3);
    s_sw_reverse   <= registers_in(6)(4);
    s_brightness   <= unsigned(registers_in(7));

    p_edge_w : process(s_edge_w_idx)
    begin
        case to_integer(s_edge_w_idx) is
            when 0      => s_edge_w <= to_unsigned(1, 5);
            when 1      => s_edge_w <= to_unsigned(2, 5);
            when 2      => s_edge_w <= to_unsigned(3, 5);
            when 3      => s_edge_w <= to_unsigned(4, 5);
            when 4      => s_edge_w <= to_unsigned(6, 5);
            when 5      => s_edge_w <= to_unsigned(8, 5);
            when 6      => s_edge_w <= to_unsigned(12, 5);
            when others => s_edge_w <= to_unsigned(16, 5);
        end case;
    end process;

    --------------------------------------------------------------------------
    -- Position counters and frame edge detect.
    --------------------------------------------------------------------------
    p_pos : process(clk)
    begin
        if rising_edge(clk) then
            s_prev_hsync_n <= data_in.hsync_n;
            s_prev_vsync_n <= data_in.vsync_n;
            s_prev_avid    <= data_in.avid;

            s_vsync_edge <= '0';
            s_hsync_edge <= '0';

            if data_in.hsync_n = '0' and s_prev_hsync_n = '1' then
                s_active_pixel <= (others => '0');
                s_hsync_edge   <= '1';
            elsif data_in.avid = '1' then
                s_active_pixel <= s_active_pixel + 1;
            end if;

            if data_in.avid = '0' and s_prev_avid = '1' then
                s_active_width <= s_active_pixel;
                s_active_line  <= s_active_line + 1;
            end if;

            if data_in.vsync_n = '0' and s_prev_vsync_n = '1' then
                if s_active_line /= 0 then
                    s_active_height <= s_active_line;
                end if;
                s_active_line <= (others => '0');
                s_frame_count <= s_frame_count + 1;
                s_vsync_edge  <= '1';
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Rotation angle accumulators (advance once per frame).
    --------------------------------------------------------------------------
    --------------------------------------------------------------------------
    -- v6: vertex/cell positions are sourced from precomputed phase tables
    -- (C_VERT_SX/SY, C_CELL_SX/SY) instead of being computed on-the-fly
    -- by a multiplier-based rotation state machine. This trades dynamic
    -- per-axis rotation control for a massive LC reduction.
    --
    -- s_phase advances each frame by an amount derived from the Speed
    -- knob; 4D Twist is repurposed as a phase-jump knob (advances faster
    -- when high). All other knobs work unchanged.
    --------------------------------------------------------------------------

    p_phase_advance : process(clk)
        variable v_acc_next : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            if s_vsync_edge = '1' then
                -- Add or subtract the full 10-bit Speed knob to the 16b
                -- fractional accumulator (natural mod 2^16). Top 6 bits
                -- become the phase index used to read the position
                -- tables, giving 1024-step sub-phase resolution.
                if s_sw_reverse = '1' then
                    v_acc_next := s_phase_acc - resize(s_speed, 16);
                else
                    v_acc_next := s_phase_acc + resize(s_speed, 16);
                end if;
                s_phase_acc <= v_acc_next;
                s_phase     <= v_acc_next(15 downto 10);
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- BRAM read processes for the four phase tables. yosys infers each
    -- as block RAM. Read latency is 1 cycle.
    --------------------------------------------------------------------------
    p_vsx_rd : process(clk)
    begin
        if rising_edge(clk) then
            s_vsx_data <= C_VERT_SX(to_integer(s_vsx_addr));
        end if;
    end process;

    p_vsy_rd : process(clk)
    begin
        if rising_edge(clk) then
            s_vsy_data <= C_VERT_SY(to_integer(s_vsy_addr));
        end if;
    end process;

    p_csx_rd : process(clk)
    begin
        if rising_edge(clk) then
            s_csx_data <= C_CELL_SX(to_integer(s_csx_addr));
        end if;
    end process;

    p_csy_rd : process(clk)
    begin
        if rising_edge(clk) then
            s_csy_data <= C_CELL_SY(to_integer(s_csy_addr));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Vertex/cell lookup state machine. On each vsync edge, walk through
    -- 24 elements (16 verts + 8 cells), driving table read addresses and
    -- capturing results into the screen-coord output registers. After
    -- positions are loaded, transition to edge-buffer raster phases.
    --------------------------------------------------------------------------
    p_vstate : process(clk)
        variable v_idx        : integer range 0 to 1023;
        -- DDA scratch
        variable v_da_xa, v_da_ya : signed(8 downto 0);
        variable v_da_xb, v_da_yb : signed(8 downto 0);
        variable v_dxn, v_dyn     : signed(8 downto 0);
        variable v_da_dxabs       : signed(8 downto 0);
        variable v_da_dyabs       : signed(8 downto 0);
        variable v_xclip, v_yclip : signed(8 downto 0);
        variable v_e2             : signed(10 downto 0);
        variable v_err_new        : signed(9 downto 0);
        variable v_x_new, v_y_new : signed(8 downto 0);
        -- LUTLOAD scratch: scaled vertex/cell coords before center add.
        variable v_scaled_x, v_scaled_y, v_scaled_cx, v_scaled_cy : signed(14 downto 0);
    begin
        if rising_edge(clk) then
            -- Default: idle the edge-buffer write port.
            s_eb_wr_en <= '0';

            case s_vphase is
                when VS_IDLE =>
                    if s_vsync_edge = '1' then
                        s_v_counter <= 0;
                        s_vphase    <= VS_LUTLOAD;
                    end if;

                when VS_LUTLOAD =>
                    -- 3-stage pipeline:
                    --   Stage A (counter K): drive table read addr for elem K.
                    --   Stage B (counter K+1): BRAM data emerges; apply Size
                    --     scaling (case + shift + add) and register into
                    --     s_lut_scaled_*.
                    --   Stage C (counter K+2): add screen center to scaled
                    --     value, write into s_vert_sx/sy or s_cell_sx/sy.
                    -- Counter runs 0..25 (24 elements + 2 cycles drain).

                    -- Stage A: drive read address for the element being
                    -- requested this cycle (elem = s_v_counter).
                    if s_v_counter < 16 then
                        v_idx := to_integer(s_phase) * 16 + s_v_counter;
                        s_vsx_addr <= to_unsigned(v_idx, 10);
                        s_vsy_addr <= to_unsigned(v_idx, 10);
                    elsif s_v_counter < 24 then
                        v_idx := to_integer(s_phase) * 8 + (s_v_counter - 16);
                        s_csx_addr <= to_unsigned(v_idx, 9);
                        s_csy_addr <= to_unsigned(v_idx, 9);
                    end if;

                    -- Stage B: at counter K, BRAM data for elem K-1 is
                    -- available on s_*sx_data/s_*sy_data. Apply size
                    -- scaling and register the result.
                    if s_v_counter > 0 and s_v_counter <= 16 then
                        -- elem K-1 was a vertex
                        case s_size(9 downto 8) is
                            when "00"   =>
                                s_lut_scaled_x <= resize(s_vsx_data, 15);
                                s_lut_scaled_y <= resize(s_vsy_data, 15);
                            when "01"   =>
                                s_lut_scaled_x <= resize(s_vsx_data, 15) +
                                    resize(shift_right(s_vsx_data, 1), 15);
                                s_lut_scaled_y <= resize(s_vsy_data, 15) +
                                    resize(shift_right(s_vsy_data, 1), 15);
                            when "10"   =>
                                s_lut_scaled_x <= shift_left(resize(s_vsx_data, 15), 1);
                                s_lut_scaled_y <= shift_left(resize(s_vsy_data, 15), 1);
                            when others =>
                                s_lut_scaled_x <= resize(s_vsx_data, 15) +
                                    shift_left(resize(s_vsx_data, 15), 1);
                                s_lut_scaled_y <= resize(s_vsy_data, 15) +
                                    shift_left(resize(s_vsy_data, 15), 1);
                        end case;
                    elsif s_v_counter > 16 and s_v_counter <= 24 then
                        -- elem K-1 was a cell
                        case s_size(9 downto 8) is
                            when "00"   =>
                                s_lut_scaled_x <= resize(s_csx_data, 15);
                                s_lut_scaled_y <= resize(s_csy_data, 15);
                            when "01"   =>
                                s_lut_scaled_x <= resize(s_csx_data, 15) +
                                    resize(shift_right(s_csx_data, 1), 15);
                                s_lut_scaled_y <= resize(s_csy_data, 15) +
                                    resize(shift_right(s_csy_data, 1), 15);
                            when "10"   =>
                                s_lut_scaled_x <= shift_left(resize(s_csx_data, 15), 1);
                                s_lut_scaled_y <= shift_left(resize(s_csy_data, 15), 1);
                            when others =>
                                s_lut_scaled_x <= resize(s_csx_data, 15) +
                                    shift_left(resize(s_csx_data, 15), 1);
                                s_lut_scaled_y <= resize(s_csy_data, 15) +
                                    shift_left(resize(s_csy_data, 15), 1);
                        end case;
                    end if;

                    -- Stage C: at counter K, write output for elem K-2.
                    -- elem K-2 in 0..15 = vertex; in 16..23 = cell.
                    if s_v_counter >= 2 and s_v_counter <= 17 then
                        s_vert_sx(s_v_counter - 2) <=
                            resize(s_lut_scaled_x, 13) +
                            signed(resize(shift_right(s_active_width,  1), 13));
                        s_vert_sy(s_v_counter - 2) <=
                            resize(s_lut_scaled_y, 13) +
                            signed(resize(shift_right(s_active_height, 1), 13));
                    elsif s_v_counter >= 18 and s_v_counter <= 25 then
                        s_cell_sx(s_v_counter - 18) <=
                            resize(s_lut_scaled_x, 13) +
                            signed(resize(shift_right(s_active_width,  1), 13));
                        s_cell_sy(s_v_counter - 18) <=
                            resize(s_lut_scaled_y, 13) +
                            signed(resize(shift_right(s_active_height, 1), 13));
                    end if;

                    if s_v_counter < 25 then
                        s_v_counter <= s_v_counter + 1;
                    else
                        s_clr_addr   <= (others => '0');
                        s_eb_wr_en   <= '1';
                        s_eb_wr_addr <= (others => '0');
                        s_eb_wr_data <= '0';
                        s_vphase     <= VS_RAST_CLR;
                    end if;

                when VS_RAST_CLR =>
                    s_eb_wr_en   <= '1';
                    s_eb_wr_addr <= s_clr_addr;
                    s_eb_wr_data <= '0';
                    if s_clr_addr < to_unsigned(EB_SIZE - 1, EB_ADDR_W) then
                        s_clr_addr <= s_clr_addr + 1;
                    else
                        s_clr_addr <= (others => '0');
                        s_dda_edge <= 0;
                        s_eb_wr_en <= '0';
                        s_vphase   <= VS_RAST_LOAD;
                    end if;

                when VS_RAST_LOAD =>
                    -- Stage 1 (load): walk the C_EDGES table and the
                    -- vertex screen-coord arrays, register the four
                    -- endpoint coords. Splits the long combinational
                    -- chain (s_dda_edge -> C_EDGES mux -> s_vert_sx mux
                    -- -> shift -> dda regs) into two cycles for timing.
                    s_load_xa <= resize(shift_right(
                        s_vert_sx(C_EDGES(s_dda_edge).a), 3), 9);
                    s_load_ya <= resize(shift_right(
                        s_vert_sy(C_EDGES(s_dda_edge).a), 3), 9);
                    s_load_xb <= resize(shift_right(
                        s_vert_sx(C_EDGES(s_dda_edge).b), 3), 9);
                    s_load_yb <= resize(shift_right(
                        s_vert_sy(C_EDGES(s_dda_edge).b), 3), 9);
                    s_vphase <= VS_RAST_LOAD2;

                when VS_RAST_LOAD2 =>
                    -- Stage 2 (compute): use registered endpoints to
                    -- compute deltas, signs, init err.
                    v_da_xa := s_load_xa;
                    v_da_ya := s_load_ya;
                    v_da_xb := s_load_xb;
                    v_da_yb := s_load_yb;
                    v_dxn := v_da_xb - v_da_xa;
                    v_dyn := v_da_yb - v_da_ya;
                    if v_dxn >= 0 then
                        s_dda_dx <= v_dxn; s_dda_sx <= to_signed(1, 2);
                        v_da_dxabs := v_dxn;
                    else
                        s_dda_dx <= -v_dxn; s_dda_sx <= to_signed(-1, 2);
                        v_da_dxabs := -v_dxn;
                    end if;
                    if v_dyn >= 0 then
                        s_dda_dy <= v_dyn; s_dda_sy <= to_signed(1, 2);
                        v_da_dyabs := v_dyn;
                    else
                        s_dda_dy <= -v_dyn; s_dda_sy <= to_signed(-1, 2);
                        v_da_dyabs := -v_dyn;
                    end if;
                    s_dda_x  <= v_da_xa;
                    s_dda_y  <= v_da_ya;
                    s_dda_x2 <= v_da_xb;
                    s_dda_y2 <= v_da_yb;
                    s_dda_err <= resize(v_da_dxabs, 10) - resize(v_da_dyabs, 10);
                    s_vphase <= VS_RAST_PLOT;

                when VS_RAST_PLOT =>
                    -- TEST cycle: plot the current pixel, evaluate the two
                    -- Bresenham comparisons, register the resulting step
                    -- flags. The compare->add chain that previously fed
                    -- s_dda_err in one cycle is split here.
                    s_eb_wr_en <= '1';
                    if s_dda_x < 0 then
                        v_xclip := (others => '0');
                    elsif s_dda_x >= to_signed(EB_W, 9) then
                        v_xclip := to_signed(EB_W - 1, 9);
                    else
                        v_xclip := s_dda_x;
                    end if;
                    if s_dda_y < 0 then
                        v_yclip := (others => '0');
                    elsif s_dda_y >= to_signed(EB_H, 9) then
                        v_yclip := to_signed(EB_H - 1, 9);
                    else
                        v_yclip := s_dda_y;
                    end if;
                    s_eb_wr_addr <=
                        shift_left(resize(unsigned(v_yclip(7 downto 0)), EB_ADDR_W), 8) or
                        resize(unsigned(v_xclip(7 downto 0)), EB_ADDR_W);
                    s_eb_wr_data <= '1';

                    v_e2 := shift_left(resize(s_dda_err, 11), 1);
                    if (s_dda_x = s_dda_x2) and (s_dda_y = s_dda_y2) then
                        if s_dda_edge < 31 then
                            s_dda_edge <= s_dda_edge + 1;
                            s_vphase   <= VS_RAST_LOAD;
                        else
                            s_eb_wr_en <= '0';
                            s_vphase   <= VS_DONE;
                        end if;
                    else
                        if v_e2 > -resize(s_dda_dy, 11) then
                            s_step_x_flag <= '1';
                        else
                            s_step_x_flag <= '0';
                        end if;
                        if v_e2 < resize(s_dda_dx, 11) then
                            s_step_y_flag <= '1';
                        else
                            s_step_y_flag <= '0';
                        end if;
                        s_vphase <= VS_RAST_STEP;
                    end if;

                when VS_RAST_STEP =>
                    -- STEP cycle: apply registered step flags. err and
                    -- x/y are each updated by a single conditional
                    -- add/sub (or both for err), with no intermediate
                    -- compare in the path.
                    s_eb_wr_en <= '0';
                    v_err_new := s_dda_err;
                    v_x_new   := s_dda_x;
                    v_y_new   := s_dda_y;
                    if s_step_x_flag = '1' then
                        v_err_new := v_err_new - resize(s_dda_dy, 10);
                        v_x_new   := v_x_new + resize(s_dda_sx, 9);
                    end if;
                    if s_step_y_flag = '1' then
                        v_err_new := v_err_new + resize(s_dda_dx, 10);
                        v_y_new   := v_y_new + resize(s_dda_sy, 9);
                    end if;
                    s_dda_err <= v_err_new;
                    s_dda_x   <= v_x_new;
                    s_dda_y   <= v_y_new;
                    s_vphase  <= VS_RAST_PLOT;

                when VS_DONE =>
                    if s_vsync_edge = '1' then
                        s_vphase    <= VS_LUTLOAD;
                        s_v_counter <= 0;
                    end if;

                when others =>
                    -- Legacy phase enum values are unused in v6.
                    s_vphase <= VS_IDLE;
            end case;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Per-scanline edge x update — INCREMENTAL.
    --
    -- Stored slope = (dx * 4 / dy) in integer scaling (recip*FRAC math),
    -- so the per-scanline x-step in fractional pixels is slope/4. We
    -- maintain a Q11.2 accumulator (16-bit signed) per edge that holds
    -- the current x-position scaled by 4. Each scanline within an edge's
    -- y-range, the accumulator advances by slope; the active current_x
    -- output is the accumulator >> 2.
    --
    -- This avoids a per-scanline multiply (was 32 parallel 16x14 muls).
    --------------------------------------------------------------------------
    --------------------------------------------------------------------------
    -- Edge buffer BRAM read/write process. Single-port style with read
    -- and write enables driven by the rasterizer (writes during vblank)
    -- and the per-pixel pipeline (reads during active video).
    --------------------------------------------------------------------------
    p_edge_buf : process(clk)
    begin
        if rising_edge(clk) then
            if s_eb_wr_en = '1' then
                s_edge_buf(to_integer(s_eb_wr_addr)) <= s_eb_wr_data;
            end if;
            s_eb_rd_data <= s_edge_buf(to_integer(s_eb_rd_addr));
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Sync and YUV passthrough shift register (LATENCY-deep).
    --------------------------------------------------------------------------
    p_sync_pipe : process(clk)
    begin
        if rising_edge(clk) then
            s_avid_sr    <= s_avid_sr   (LATENCY-2 downto 0) & data_in.avid;
            s_hsync_n_sr <= s_hsync_n_sr(LATENCY-2 downto 0) & data_in.hsync_n;
            s_vsync_n_sr <= s_vsync_n_sr(LATENCY-2 downto 0) & data_in.vsync_n;
            s_field_n_sr <= s_field_n_sr(LATENCY-2 downto 0) & data_in.field_n;

            for k in LATENCY-1 downto 1 loop
                s_y_sr(k) <= s_y_sr(k-1);
                s_u_sr(k) <= s_u_sr(k-1);
                s_v_sr(k) <= s_v_sr(k-1);
            end loop;
            s_y_sr(0) <= data_in.y;
            s_u_sr(0) <= data_in.u;
            s_v_sr(0) <= data_in.v;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Per-pixel pipeline.
    -- P1: 8x cell sub. Drive edge-buffer read address from
    -- (line>>3, px>>3); the read result lands at P2 from the BRAM.
    --------------------------------------------------------------------------
    p_pipe1 : process(clk)
        variable v_px, v_py : t_px;
    begin
        if rising_edge(clk) then
            v_px := signed(resize(s_active_pixel, 13));
            v_py := signed(resize(s_active_line,  13));
            for ci in 0 to 7 loop
                p1_cell_dx(ci) <= resize(v_px - s_cell_sx(ci), 14);
                p1_cell_dy(ci) <= resize(v_py - s_cell_sy(ci), 14);
            end loop;
            -- Edge buffer addr = (line>>3) * EB_W + (px>>3) =
            -- (line(10:3) << 8) | px(10:3) since EB_W = 256.
            s_eb_rd_addr <= s_active_line(10 downto 3) &
                            s_active_pixel(10 downto 3);
        end if;
    end process;

    --------------------------------------------------------------------------
    -- P2: abs of cell distance components. Edge buffer read result
    -- (s_eb_rd_data) is captured by p_edge_align below into a 5-deep
    -- shift register that aligns edge_hit with p6_cell_id.
    --------------------------------------------------------------------------
    p_pipe2 : process(clk)
        variable d : signed(13 downto 0);
    begin
        if rising_edge(clk) then
            for ci in 0 to 7 loop
                d := p1_cell_dx(ci);
                if d < 0 then
                    p2_cell_adx(ci) <= unsigned(resize(-d, 13));
                else
                    p2_cell_adx(ci) <= unsigned(resize( d, 13));
                end if;
                d := p1_cell_dy(ci);
                if d < 0 then
                    p2_cell_ady(ci) <= unsigned(resize(-d, 13));
                else
                    p2_cell_ady(ci) <= unsigned(resize( d, 13));
                end if;
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Edge-hit alignment shift register. The BRAM lookup happens at P2
    -- (data emerges 1 cycle after the address was driven at P1). We then
    -- shift it through P3..P6 to align with p6_cell_id.
    --------------------------------------------------------------------------
    eh_align : block
        signal eh2, eh3, eh4, eh5 : std_logic := '0';
    begin
        process(clk)
        begin
            if rising_edge(clk) then
                eh2 <= s_eb_rd_data;
                eh3 <= eh2;
                eh4 <= eh3;
                eh5 <= eh4;
                p6_edge_hit <= eh5;
            end if;
        end process;
    end block;

    --------------------------------------------------------------------------
    -- P3: cell Manhattan add, edge thresh compare.
    --------------------------------------------------------------------------
    p_pipe3 : process(clk)
    begin
        if rising_edge(clk) then
            for ci in 0 to 7 loop
                p3_cell_dist(ci) <=
                    resize(p2_cell_adx(ci), 14) + resize(p2_cell_ady(ci), 14);
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- P4: pair-min on 8 cell distances, OR-tier-1 on edges (8x4-way).
    --------------------------------------------------------------------------
    p_pipe4 : process(clk)
    begin
        if rising_edge(clk) then
            for k in 0 to 3 loop
                if p3_cell_dist(2*k) <= p3_cell_dist(2*k+1) then
                    p4_min(k) <= p3_cell_dist(2*k);
                    p4_idx(k) <= to_unsigned(2*k, 3);
                else
                    p4_min(k) <= p3_cell_dist(2*k+1);
                    p4_idx(k) <= to_unsigned(2*k+1, 3);
                end if;
            end loop;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- P5: pair-min on 4 -> 2, OR-tier-2 on edges (8 -> 2).
    --------------------------------------------------------------------------
    p_pipe5 : process(clk)
    begin
        if rising_edge(clk) then
            if p4_min(0) <= p4_min(1) then
                p5_min(0) <= p4_min(0);
                p5_idx(0) <= p4_idx(0);
            else
                p5_min(0) <= p4_min(1);
                p5_idx(0) <= p4_idx(1);
            end if;
            if p4_min(2) <= p4_min(3) then
                p5_min(1) <= p4_min(2);
                p5_idx(1) <= p4_idx(2);
            else
                p5_min(1) <= p4_min(3);
                p5_idx(1) <= p4_idx(3);
            end if;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- P6: final cell_id, final edge_hit.
    --------------------------------------------------------------------------
    p_pipe6 : process(clk)
    begin
        if rising_edge(clk) then
            if p5_min(0) <= p5_min(1) then
                p6_cell_id <= p5_idx(0);
            else
                p6_cell_id <= p5_idx(1);
            end if;
            -- p6_edge_hit driven by p_eh_align block below.
        end if;
    end process;

    --------------------------------------------------------------------------
    -- P7: variant lookup + chroma rotate.
    -- Reads U, V from the LATENCY shift register at depth = 6 (P0..P6 done).
    -- Variant bits: bit0 = swap UV, bit1 = neg U, bit2 = neg V.
    -- Negation around 512: u_neg = 1023 - u (bitwise NOT for 10b).
    --------------------------------------------------------------------------
    p_pipe7 : process(clk)
        variable variant : std_logic_vector(2 downto 0);
        variable v_u_in, v_v_in : unsigned(9 downto 0);
        variable v_u_step, v_v_step : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            variant := cell_variant(to_integer(p6_cell_id));
            v_u_in := unsigned(s_u_sr(6));
            v_v_in := unsigned(s_v_sr(6));
            if variant(0) = '1' then
                v_u_step := v_v_in;
                v_v_step := v_u_in;
            else
                v_u_step := v_u_in;
                v_v_step := v_v_in;
            end if;
            if variant(1) = '1' then
                v_u_step := not v_u_step;
            end if;
            if variant(2) = '1' then
                v_v_step := not v_v_step;
            end if;
            p7_u_xform  <= v_u_step;
            p7_v_xform  <= v_v_step;
            p7_cell_id  <= p6_cell_id;
            p7_edge_hit <= p6_edge_hit;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- P8..P11: cell-tint interpolators (4 stages).
    -- a = original chroma, b = rotated chroma, t = cell_tint.
    -- result = a + (b-a)*t/1024. So tint=0 -> original, tint=max -> rotated.
    --
    -- We need to feed the original chroma at the *same* pipeline position
    -- as p7_u_xform, i.e. delayed by 7 cycles from input. So 'a' = s_u_sr(7),
    -- 'b' = p7_u_xform.
    --------------------------------------------------------------------------
    --------------------------------------------------------------------------
    -- Cell-tint blend: 4-level shift-based crossfade indexed by the top 2
    -- bits of the tint knob. Replaces the 4-stage interpolator_u with a
    -- 1-cycle combinational mux. Logically:
    --   tint(9..8) = 00 -> orig       (no tint)
    --   tint(9..8) = 01 -> 3/4 orig + 1/4 rot
    --   tint(9..8) = 10 -> 1/2 orig + 1/2 rot
    --   tint(9..8) = 11 -> rotated
    -- Aliased to p11_* for compatibility with the composite stage; the
    -- pipeline depth from input to here is now 7.
    --------------------------------------------------------------------------
    p_tint_blend : process(clk)
        variable v_a, v_b : unsigned(11 downto 0);
    begin
        if rising_edge(clk) then
            -- Original chroma at depth 7 (one stage after p_pipe7's tap 6).
            v_a := resize(unsigned(s_u_sr(7)), 12);
            v_b := resize(p7_u_xform, 12);
            case s_cell_tint(9 downto 8) is
                when "00"   => p11_u_mixed <= v_a(9 downto 0);
                when "01"   => p11_u_mixed <= resize(shift_right(v_a + v_a + v_a + v_b, 2), 10);
                when "10"   => p11_u_mixed <= resize(shift_right(v_a + v_b, 1), 10);
                when others => p11_u_mixed <= v_b(9 downto 0);
            end case;

            v_a := resize(unsigned(s_v_sr(7)), 12);
            v_b := resize(p7_v_xform, 12);
            case s_cell_tint(9 downto 8) is
                when "00"   => p11_v_mixed <= v_a(9 downto 0);
                when "01"   => p11_v_mixed <= resize(shift_right(v_a + v_a + v_a + v_b, 2), 10);
                when "10"   => p11_v_mixed <= resize(shift_right(v_a + v_b, 1), 10);
                when others => p11_v_mixed <= v_b(9 downto 0);
            end case;

            p11_edge_hit <= p7_edge_hit;
            p11_cell_id  <= p7_cell_id;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- P12: composite.
    --   - Default: cell-tinted chroma (p11_u_mixed, p11_v_mixed) with luma
    --     from delayed video at depth 11.
    --   - bg_video toggle off: blacked-out background (Y=64, U=V=512). The
    --     wireframe still draws over.
    --   - cellfill toggle off: bypass the cell tint (use raw delayed chroma).
    --   - wireframe enabled and edge_hit: paint edge color (white or
    --     cell-luma-bumped) over the composited result.
    --------------------------------------------------------------------------
    p_pipe12 : process(clk)
        variable v_y : unsigned(9 downto 0);
        variable v_u : unsigned(9 downto 0);
        variable v_v : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- Original Y/UV at depth 8 to align with p11_*.
            v_y := unsigned(s_y_sr(8));
            if s_sw_cellfill = '1' then
                v_u := p11_u_mixed;
                v_v := p11_v_mixed;
            else
                v_u := unsigned(s_u_sr(8));
                v_v := unsigned(s_v_sr(8));
            end if;
            if s_sw_bg_video = '0' then
                v_y := to_unsigned(64,  10);
                v_u := to_unsigned(512, 10);
                v_v := to_unsigned(512, 10);
            end if;
            if s_sw_wire = '1' and p11_edge_hit = '1' then
                v_y := s_edge_bright;
                if s_sw_edgewhite = '1' then
                    v_u := to_unsigned(512, 10);
                    v_v := to_unsigned(512, 10);
                end if;
            end if;
            p12_y <= v_y;
            p12_u <= v_u;
            p12_v <= v_v;
        end if;
    end process;

    --------------------------------------------------------------------------
    -- Output drive (P8). Brightness control was dropped to save LCs;
    -- the user's brightness knob now adjusts only the edge brightness
    -- (s_edge_bright). The fader/slider still maps to register 7 but
    -- has no effect on the composited Y unless the wireframe is hit.
    --------------------------------------------------------------------------
    p_output : process(clk)
    begin
        if rising_edge(clk) then
            data_out.y       <= std_logic_vector(p12_y);
            data_out.u       <= std_logic_vector(p12_u);
            data_out.v       <= std_logic_vector(p12_v);
            data_out.avid    <= s_avid_sr   (LATENCY-1);
            data_out.hsync_n <= s_hsync_n_sr(LATENCY-1);
            data_out.vsync_n <= s_vsync_n_sr(LATENCY-1);
            data_out.field_n <= s_field_n_sr(LATENCY-1);
        end if;
    end process;

end architecture hypercube;
