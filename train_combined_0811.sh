#!/usr/bin/env bash
set -euo pipefail

DATA="/home/parkjinwoo/urrc_hanla/combined_0811.yaml"
PREVIOUS_BEST="/home/parkjinwoo/Downloads/yolo11_seg_runs/combined_3_seg/weights/best.pt"
BASE_MODEL="/home/parkjinwoo/Downloads/yolo11n-seg.pt"
PROJECT="/home/parkjinwoo/Downloads/yolo11_seg_runs"
export MPLCONFIGDIR="/home/parkjinwoo/urrc_hanla/.cache/matplotlib"
mkdir -p "$MPLCONFIGDIR"

if [[ -f "$PREVIOUS_BEST" ]]; then
  MODEL="${MODEL:-$PREVIOUS_BEST}"
else
  MODEL="${MODEL:-$BASE_MODEL}"
fi

for required in "$DATA" "$MODEL"; do
  if [[ ! -f "$required" ]]; then
    echo "Required file not found: $required" >&2
    exit 1
  fi
done

yolo segment train \
  model="$MODEL" \
  data="$DATA" \
  epochs="${EPOCHS:-100}" \
  imgsz="${IMGSZ:-640}" \
  batch="${BATCH:-1}" \
  device="${DEVICE:-0}" \
  workers="${WORKERS:-2}" \
  project="$PROJECT" \
  name="${RUN_NAME:-combined_0811_seg}" \
  patience="${PATIENCE:-100}" \
  pretrained=true \
  plots=true
