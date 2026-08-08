#!/usr/bin/env bash
#
# Mix-and-match BVH builder / pre-processing / post-processing sweep for the
# benchmark binary.
#
# For every builder configuration it runs the full cross-product of
#   presplit factors  x  reinsertion (batch_size_ratio, max_itr)
# including the baseline "no pre-processing / no post-processing" run, each with
# RERUNS render passes, and tabulates every metric the benchmark emits.
#
# Every run is stored in its own subdirectory:
#   OUT_DIR/row_0001/{row.txt, row.err, image.ppm}
# plus an aggregated OUT_DIR/results.tsv.
#
# Usage:
#   ./bench_sweep.sh [model] [reruns] [out_dir]
# (OUT_DIR must be a relative path for safety; use RUN_LIMIT to bound the sweep.)
#
# Environment knobs (all optional):
#   BENCH            path to the benchmark binary (default: repo build dir)
#   TIMEOUT          per-run timeout in seconds   (default: 300)
#   RUN_LIMIT        stop after this many runs    (default: unlimited)
#   PRESPLIT_FACTORS split factors   (default: 0.2 0.4 0.6)
#   REINSERT_RATIOS  ratios          (default: 0.02 0.05 0.10)
#   REINSERT_ITRS    max iters       (default: 1 3 5)
#   BINS             binned bins     (default: 4 16 64)
#   PRIM_PAIRS      (min,max) pairs  (default: 1 1,1 2,2 4)
#   GRIDS            ploc grid       (default: 256 1024 4096)
#   BITS             ploc log_bits   (default: 8 10)
#   RADII            ploc radius     (default: 5 15 30)
#   NO_EST=1         skip calibration/ETA run
#
# Defaults produce ~1200 runs (30 builders x 4 pre x 10 post). A baseline run
# is timed up-front to print a lower-bound ETA, so you can judge scale before
# committing. Shrink via the env lists or bound with RUN_LIMIT, e.g.:
#   PRESPLIT_FACTORS="0.3 0.4" REINSERT_ITRS="1 2" ./bench_sweep.sh model.obj

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MODEL="${1:-$SCRIPT_DIR/../../../Sponza/sponza.obj}"
RERUNS="${2:-3}"
OUT_DIR="${3:-bench_results}"

BENCH_DEFAULT="$SCRIPT_DIR/../../build/examples/benchmark/benchmark"
BENCH="${BENCH:-$BENCH_DEFAULT}"

TIMEOUT="${TIMEOUT:-300}"
RUN_LIMIT="${RUN_LIMIT:-}"

read -r -a PRESPLIT_FACTORS <<<"${PRESPLIT_FACTORS:=0.2 0.4 0.6}"
read -r -a REINSERT_RATIOS <<<"${REINSERT_RATIOS:=0.02 0.05 0.10}"
read -r -a REINSERT_ITRS <<<"${REINSERT_ITRS:=1 3 5}"

# builder sweep dimensions (space-separated for single values, comma-separated
# for the (min,max) prim pairs). Override via env to shrink/enlarge the sweep.
read -r -a BINS <<<"${BINS:=4 16 64}"
read -r -a GRIDS <<<"${GRIDS:=256 1024 4096}"
read -r -a BITS <<<"${BITS:=8 10}"
read -r -a RADII <<<"${RADII:=5 15 30}"
IFS=',' read -r -a PRIM_PAIRS_TMP <<<"${PRIM_PAIRS:=1 1,1 2,2 4}"
PRIM_PAIRS=("${PRIM_PAIRS_TMP[@]}")

if [ ! -x "$BENCH" ]; then
  echo "error: benchmark binary not found at $BENCH (set BENCH=...)" >&2
  exit 1
fi
if [ ! -f "$MODEL" ]; then
  echo "error: model file not found at $MODEL (pass a valid model path)" >&2
  exit 1
fi

case "$OUT_DIR" in
  ""|"/"|"."|"..")
    echo "error: refusing to use OUT_DIR='$OUT_DIR'. Pick a dedicated results dir." >&2
    exit 1
    ;;
  /*|"$HOME"|"$HOME"/*)
    echo "error: refusing to use OUT_DIR='$OUT_DIR' (absolute or home path is risky). Use a relative path like 'bench_results'." >&2
    exit 1
    ;;
esac

rm -rf -- "$OUT_DIR"
mkdir -p -- "$OUT_DIR"
OUT_DIR="$(cd -- "$OUT_DIR" && pwd)"

RESULT="$OUT_DIR/results.tsv"
ALIGNED="$OUT_DIR/results.txt"
printf 'row\tlabel\tprocessing\tbuilder\tbins\tmin\tmax\tgrid\tlog_bits\tradius\tpresplit\treinsert_batch\treinsert_itr\tbuilder_ms\tdepth\tsah\tepo\tsah_epo\tnodes\trays_ns\trender_total_ms\tstatus\n' >"$RESULT"

# one shared fixed-width format for header and rows (same spec -> guaranteed
# alignment). text columns are left-aligned, numeric columns right-aligned.
ALIGN_FMT='%-9s %-84s %-19s %-12s %6s %6s %6s %8s %8s %8s %9s %16s %14s %11s %6s %12s %12s %12s %11s %15s %15s %-16s'
ALIGN_HEADER=(row label processing builder bins min max grid log_bits radius presplit reinsert_batch reinsert_itr builder_ms depth sah epo sah_epo nodes rays_ns render_total_ms status)
printf '%s\n' "$(printf "$ALIGN_FMT\n" "${ALIGN_HEADER[@]}")" >"$ALIGNED"

PRE_COMBOS=$(( ${#PRESPLIT_FACTORS[@]} + 1 ))
POST_COMBOS=$(( ${#REINSERT_RATIOS[@]} * ${#REINSERT_ITRS[@]} + 1 ))

# builder sweep dimensions (single source of truth for both the loops and the
# total-run count below)
N_BINNED=$(( ${#BINS[@]} * ${#PRIM_PAIRS[@]} ))
N_SWEEP=${#PRIM_PAIRS[@]}
N_PLOC=$(( ${#GRIDS[@]} * ${#BITS[@]} * ${#RADII[@]} ))
N_BUILDERS=$(( N_BINNED + N_SWEEP + N_PLOC ))

TOTAL=$(( N_BUILDERS * PRE_COMBOS * POST_COMBOS ))

echo "runner: model=$MODEL reruns=$RERUNS binary=$BENCH" >&2
echo "runner: $PRE_COMBOS pre-processing combos x $POST_COMBOS post-processing combos per builder" >&2
echo "runner: builders: $N_BINNED binned_sah, $N_SWEEP sweep_sah, $N_PLOC ploc ($N_BUILDERS total)" >&2
echo "runner: total runs = $TOTAL" >&2
echo "runner: results -> $RESULT (per-run artifacts in $OUT_DIR/row_NNNN/)" >&2

# rough ETA by timing a single baseline run and extrapolating to TOTAL.
# (a lower bound: builds like presplit and reinsertion can be slower per run.)
if [ -z "${NO_EST:-}" ]; then
  CAL_DIR="$OUT_DIR/_calibrate"
  mkdir -p -- "$CAL_DIR"
  CAL_START=$(date +%s%N)
  (cd "$CAL_DIR" && timeout "$TIMEOUT" "$BENCH" -m "$MODEL" -n "$RERUNS" -b binned_sah >/dev/null 2>&1) || true
  CAL_END=$(date +%s%N)
  CAL_MS=$(( (CAL_END - CAL_START) / 1000000 ))
  if [ "$CAL_MS" -gt 0 ]; then
    ETA_S=$(( CAL_MS * TOTAL / 1000 ))
    printf 'runner: calibration ~%s ms/run; ETA ~%dh %dm %ds for %d runs (lower bound)\n' \
      "$CAL_MS" "$(( ETA_S / 3600 ))" "$(( (ETA_S % 3600) / 60 ))" "$(( ETA_S % 60 ))" "$TOTAL" >&2
  else
    echo "runner: calibration failed under resolution; ETA unknown" >&2
  fi
  rm -rf -- "$CAL_DIR"
fi

count=0
done_number() {
  local denom=$TOTAL
  if [ -n "${RUN_LIMIT:-}" ] && [ "$RUN_LIMIT" -lt "$TOTAL" ]; then denom=$RUN_LIMIT; fi
  printf 'done %s / %s (%s left)' "$count" "$denom" "$((denom - count))"
}
race() {
  [ -z "${RUN_LIMIT:-}" ] && return 0
  [ "$count" -lt "$RUN_LIMIT" ]
}

# append a fixed-width, aligned line to results.txt (nice for `tail -f`).
aligned_row() {
  printf '%s\n' "$(printf "$ALIGN_FMT\n" "$@")" >>"$ALIGNED"
}

run_cfg() {
  local lbl="$1"
  shift
  if ! race; then return; fi

  local PROC=none
  if [ "$c_ppf" != "-" ] && [ "$c_ratio" != "-" ]; then PROC="presplit+reinsert"
  elif [ "$c_ratio" != "-" ]; then PROC=reinsert
  elif [ "$c_ppf" != "-" ]; then PROC=presplit
  fi

  count=$((count + 1))
  local dir="$OUT_DIR/row_$(printf '%04d' "$count")"
  mkdir -p -- "$dir"
  local out="$dir/row.txt" err="$dir/row.err"
  printf '  %s | %s\n' "$(done_number)" "$lbl" >&2

  local rc=0
  if (cd "$dir" && timeout "$TIMEOUT" "$BENCH" "$@" >"$out" 2>"$err"); then
    :
  else
    rc=$?
  fi

  # preserve the rendered image regardless of success/failure
  if [ -f "$dir/test.ppm" ]; then mv -- "$dir/test.ppm" "$dir/image.ppm"; fi

  if [ "$rc" -ne 0 ]; then
    printf '%s\t%s\t%s\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\t-\tfailed(rc=%s)\n' \
      "$(basename "$dir")" "$lbl" "$PROC" "$rc" >>"$RESULT"
    aligned_row "$(basename "$dir")" "$lbl" "$PROC" \
      - - - - - - - - - - - - - - - - - - "failed(rc=$rc)"
    echo "    -> failed rc=$rc (see $err)" >&2
    return
  fi

  local build_ms depth sah epo sah_epo nodes rays_ns render_total
  build_ms=$(grep -oP 'builder took: \K[0-9]+' "$out" | head -1)
  depth=$(grep -oP 'depth of bvh: \K[0-9]+' "$out" | head -1)
  sah=$(grep -oP 'sah of bvh: \K[0-9.]+' "$out" | head -1)
  epo=$(grep -oP 'epo of bvh: \K[0-9.]+' "$out" | head -1)
  sah_epo=$(grep -oP 'sah-epo: \K[0-9.]+' "$out" | head -1)
  nodes=$(grep -oP 'num nodes: \K[0-9]+' "$out" | head -1)
  rays_ns=$(grep -oP 'average time taken per ray: \K[0-9.]+' "$out" |
    awk '{ s += $1; n++ } END { printf n ? "%.2f" : "-", s / n }')
  render_total=$(grep -oP 'render took: \K[0-9]+' "$out" |
    awk '{ s += $1; n++ } END { if (n) print s; else print "-" }')

  local d="-" ; [ -n "$build_ms" ] && d=$build_ms
  local e="-" ; [ -n "$depth" ] && e=$depth
  local f="-" ; [ -n "$sah" ] && f=$sah
  local g="-" ; [ -n "$epo" ] && g=$epo
  local h="-" ; [ -n "$sah_epo" ] && h=$sah_epo
  local j="-" ; [ -n "$nodes" ] && j=$nodes
  local k="-" ; [ -n "$rays_ns" ] && k=$rays_ns
  local m="-" ; [ -n "$render_total" ] && m=$render_total

  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$(basename "$dir")" "$lbl" "$PROC" "$c_builder" "$c_bins" "$c_min" "$c_max" "$c_grid" "$c_bits" "$c_radius" \
    "$c_ppf" "$c_ratio" "$c_itr" "$d" "$e" "$f" "$g" "$h" "$j" "$k" "$m" "ok" >>"$RESULT"
  aligned_row "$(basename "$dir")" "$lbl" "$PROC" \
    "$c_builder" "$c_bins" "$c_min" "$c_max" "$c_grid" "$c_bits" "$c_radius" \
    "$c_ppf" "$c_ratio" "$c_itr" "$d" "$e" "$f" "$g" "$h" "$j" "$k" "$m" ok
}

# context columns describing the builder + processing of the current run
# (c_ prefix keeps them from colliding with the sweep arrays BINS/BITS/GRIDS/RADII)
c_builder="-"; c_bins="-"; c_min="-"; c_max="-"; c_grid="-"; c_bits="-"; c_radius="-"
c_ppf="-"; c_ratio="-"; c_itr="-"

sweep_pre_post() {  # $1 = -b spec string, $2.. = fixed builder columns
  local spec="$1"; shift
  c_builder="$1"; c_bins="$2"; c_min="$3"; c_max="$4"; c_grid="$5"; c_bits="$6"; c_radius="$7"

  local args=(-m "$MODEL" -n "$RERUNS" -b "$spec")
  local p
  for p in none "${PRESPLIT_FACTORS[@]}"; do
    local pre_args=()
    if [ "$p" = "none" ]; then
      c_ppf="-"
    else
      c_ppf="$p"
      pre_args=(--presplit --presplit-factor "$p")
    fi

    local r
    for r in none "${REINSERT_RATIOS[@]}"; do
      if [ "$r" = "none" ]; then
        c_ratio="-"; c_itr="-"
        local lbl="builder=${c_builder} presplit=${c_ppf} reinsert=none"
        run_cfg "$lbl" "${args[@]}" "${pre_args[@]}"
        continue
      fi
      local i
      for i in "${REINSERT_ITRS[@]}"; do
        c_ratio="$r"; c_itr="$i"
        local lbl="builder=${c_builder} presplit=${c_ppf} reinsert=${r}:${i}"
        run_cfg "$lbl" "${args[@]}" "${pre_args[@]}" \
          --reinsert --reinsert-ratio "$r" --reinsert-itr "$i"
      done
    done
  done
}

# ---------------- binned_sah : bins from 4 to 64, min<=max prims --------------
for bins in "${BINS[@]}"; do
  for pair in "${PRIM_PAIRS[@]}"; do
    read -r min max <<<"$pair"
    sweep_pre_post "binned_sah:num_samples=$bins,max_primitives=$max,min_primitives=$min" \
      "binned_sah" "$bins" "$min" "$max" "-" "-" "-"
  done
done

# ---------------- sweep_sah : min<=max prims ----------------
for pair in "${PRIM_PAIRS[@]}"; do
  read -r min max <<<"$pair"
  sweep_pre_post "sweep_sah:min_primitives=$min,max_primitives=$max" \
    "sweep_sah" "-" "$min" "$max" "-" "-" "-"
done

# ---------------- ploc : grid 256..2048, log_bits range, radius 5..30 ----------
for grid in "${GRIDS[@]}"; do
  for bits in "${BITS[@]}"; do
    for radius in "${RADII[@]}"; do
      sweep_pre_post "ploc:grid_dim=$grid,log_bits=$bits,search_radius=$radius" \
        "ploc" "-" "-" "-" "$grid" "$bits" "$radius"
    done
  done
done

echo "runner: finished - $count / $TOTAL runs" >&2
echo "== table =="
column -t -s $'\t' "$RESULT"
