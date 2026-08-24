#!/usr/bin/env bash
# Fetch a stock darknet cfg + its pretrained backbone and patch it for our
# dataset. Run after prepare_dataset.py.
#
# Usage:  ./prepare_cfg.sh [num_classes] [tiny|full] [network_size] [max_batches]
# Default: 1 class, yolov4-tiny, 640x640, 2000 iterations
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

CLASSES="${1:-1}"
VARIANT="${2:-tiny}"
SIDE="${3:-640}"                   # must be a multiple of 32
MAX_BATCHES="${4:-2000}"

FILTERS=$(( (CLASSES + 5) * 3 ))   # 3 anchors per yolo layer

# burn_in and steps are not independent of max_batches: burn_in ramps the
# learning rate up from ~0 over the first N iterations, and steps drops it x10
# at 80%/90% to converge. Shortening max_batches without scaling these ends
# training mid-ramp, before the LR ever reaches target or either step fires.
BURN_IN=$(( MAX_BATCHES / 10 ))
[ "$BURN_IN" -lt 100 ] && BURN_IN=100
STEPS="$(( MAX_BATCHES * 80 / 100 )),$(( MAX_BATCHES * 90 / 100 ))"

if [ "$VARIANT" = "tiny" ]; then
  CFG=yolov4-tiny.cfg
  PRE=yolov4-tiny.conv.29
  SUBDIV=16
else
  CFG=yolov4.cfg
  PRE=yolov4.conv.137
  SUBDIV=32                        # full yolov4 at 640 needs smaller batches
fi
OUT="unitree-go1-${VARIANT}.cfg"

BASE_URL=https://raw.githubusercontent.com/AlexeyAB/darknet/master/cfg
PRE_URL=https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v4_pre

[ -f "$CFG" ] || curl -fsSL -o "$CFG" "$BASE_URL/$CFG"
[ -f "$PRE" ] || curl -fL   -o "$PRE" "$PRE_URL/$PRE"

cp "$CFG" "$OUT"
sed -i \
  -e "s/^batch=.*/batch=64/" \
  -e "s/^subdivisions=.*/subdivisions=${SUBDIV}/" \
  -e "s/^width=.*/width=${SIDE}/" \
  -e "s/^height=.*/height=${SIDE}/" \
  -e "s/^max_batches *=.*/max_batches = ${MAX_BATCHES}/" \
  -e "s/^burn_in *=.*/burn_in=${BURN_IN}/" \
  -e "s/^steps *=.*/steps=${STEPS}/" \
  -e "s/^classes *=.*/classes=${CLASSES}/" \
  -e "s/^filters=255/filters=${FILTERS}/" \
  "$OUT"

# The stock cfg ships COCO's anchors (prior box shapes). If they don't match the
# size distribution of your objects, most anchors go unused and box regression
# is poor -- low avg IoU, low recall, however long you train. Regenerate them
# with:  ./darknet.sh detector calcanchors obj.data -numofclusters 6 \
#            -width <SIDE> -height <SIDE>
# and this picks up the resulting anchors.txt automatically.
if [ -f anchors.txt ]; then
  ANCHORS=$(tr -d ' \n' < anchors.txt)
  sed -i -e "s/^anchors *=.*/anchors = ${ANCHORS}/" "$OUT"
  echo "applied custom anchors from anchors.txt: ${ANCHORS}"
else
  echo "NOTE: using stock COCO anchors. Run calcanchors if your objects are an" \
       "unusual size -- see the comment in this script."
fi

echo "wrote $OUT  (classes=$CLASSES filters=$FILTERS max_batches=$MAX_BATCHES ${SIDE}x${SIDE})"
echo
grep -nE "^(batch|subdivisions|width|height|burn_in|steps|classes)=|^max_batches |^filters=${FILTERS}$" "$OUT"

# Sanity check: one filters= line per [yolo] block must have been rewritten.
n_yolo=$(grep -c "^\[yolo\]" "$OUT")
n_filt=$(grep -c "^filters=${FILTERS}$" "$OUT")
echo
if [ "$n_yolo" -ne "$n_filt" ]; then
  echo "WARNING: $n_yolo [yolo] blocks but $n_filt patched filters= lines." >&2
  echo "Check the conv layer before each [yolo] reads filters=${FILTERS}." >&2
  exit 1
fi
echo "OK: $n_yolo [yolo] blocks, each preceded by filters=${FILTERS}"
echo "next: ~/darknet/build/src-cli/darknet detector train obj.data $OUT $PRE -map -dontshow"
