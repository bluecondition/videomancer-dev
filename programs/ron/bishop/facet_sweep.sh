#!/usr/bin/env bash
# Facet-stress sweep (EXPERIMENT-ONLY, deletable: `rm facet_sweep.sh`).
#
# Measures the cost of adding K wireframe edges to Bishop:
#   * LCs + Fmax x6  -- from the real FPGA build (deterministic, reliable)
#   * SD per-line margin -- from the pure-SW model (budget=1716), for both
#     the realistic "spread" distribution and the worst-case "stacked" one.
# The SD PIXEL render is intentionally NOT used: it is an unreliable overrun
# oracle near threshold (reproducible non-monotonic results).
#
# LC/Fmax depend only on edge COUNT, not row distribution, so we build once
# per K (on the spread mesh) and compute both model margins for free.
# Restores the clean committed design + vmprog at the end.
set -u
ROOT=/home/ron/videomancer-dev
MESH=$ROOT/programs/ron/bishop
CSV=/tmp/facet_sweep_results.csv
LOG=/tmp/facet_sweep.log
SLOPE=4
: > "$LOG"

# Regenerate the mesh with K stress edges of the given mode; echo the model's
# SD margin (signed integer).  Side effect: rewrites bishop_mesh_pkg.vhd.
run_model() { # <K> <mode>
  BISHOP_STRESS_EDGES="$1" BISHOP_STRESS_MODE="$2" BISHOP_STRESS_SLOPE="$SLOPE" \
  BISHOP_STRESS_ROW=0 python3 "$MESH/build_face_mesh.py" 2>&1 \
    | sed -n 's/.*margin \(-\{0,1\}[0-9]\{1,\}\).*/\1/p'
}

echo "K,C_NUM_EDGES,LCs_hdhdmi,Fmax_HDanalog,Fmax_SDanalog,Fmax_HDhdmi,Fmax_SDhdmi,Fmax_HDdual,Fmax_SDdual,SDmargin_spread,SDmargin_stacked" > "$CSV"

for K in 0 20 40 80 120 160 200; do
  echo "=== K=$K  $(date +%T) ===" | tee -a "$LOG"

  # spread mesh -> used for the build, and capture its model margin
  SPREAD=$(run_model "$K" spread)
  NEDGES=$(grep -oE 'C_NUM_EDGES : natural := [0-9]+' "$MESH/bishop_mesh_pkg.vhd" | grep -oE '[0-9]+')

  # build all 6 configs on the spread mesh
  BUILD=$(cd "$ROOT" && ./build_programs.sh ron bishop 2>&1)
  echo "$BUILD" >> "$LOG"

  # parse the 6 "Fmax: NN.NN MHz, LCs: NNNN/7680" lines, in build order:
  # HD Analog, SD Analog, HD HDMI, SD HDMI, HD Dual, SD Dual
  mapfile -t FX < <(echo "$BUILD" | grep -oE 'Fmax: [0-9.]+ MHz, LCs: [0-9]+/7680')
  if [ "${#FX[@]}" -ne 6 ]; then
    echo "  WARN: parsed ${#FX[@]}/6 Fmax lines at K=$K" | tee -a "$LOG"
  fi
  fmax() { echo "${FX[$1]:-}" | grep -oE 'Fmax: [0-9.]+' | grep -oE '[0-9.]+'; }
  lcs()  { echo "${FX[$1]:-}" | grep -oE 'LCs: [0-9]+'   | grep -oE '[0-9]+'; }

  # stacked mesh -> only to read the worst-case model margin (no build)
  STACKED=$(run_model "$K" stacked)

  echo "$K,$NEDGES,$(lcs 2),$(fmax 0),$(fmax 1),$(fmax 2),$(fmax 3),$(fmax 4),$(fmax 5),$SPREAD,$STACKED" | tee -a "$CSV"
done

# restore the clean committed design + vmprog (sweep left stressed artifacts)
python3 "$MESH/build_face_mesh.py" >/dev/null 2>&1
(cd "$ROOT" && ./build_programs.sh ron bishop >/dev/null 2>&1)
echo "SWEEP DONE $(date +%T) -- results: $CSV ; clean design + vmprog restored" | tee -a "$LOG"
