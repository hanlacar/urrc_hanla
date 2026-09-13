#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_dir="$workspace_dir/.yolo_runtime"
models_dir="$workspace_dir/src/camera_yolo_inference/models"
source_model="$models_dir/hanla_competition_11class_best.pt"
engine_model="$models_dir/hanla_competition_11class_best_rtx5060_fp16.engine"

if [[ ! -x "$runtime_dir/bin/python" ]]; then
    python3 -m venv --system-site-packages "$runtime_dir"
fi

if ! "$runtime_dir/bin/python" -c 'import torch, ultralytics, tensorrt' 2>/dev/null; then
    "$runtime_dir/bin/python" -m pip install --upgrade pip
    "$runtime_dir/bin/python" -m pip install ultralytics 'tensorrt-cu13==10.16.1.11'
fi

if [[ ! -f "$engine_model" ]]; then
    [[ -f "$source_model" ]] || { echo "Missing source model: $source_model" >&2; exit 1; }
    "$runtime_dir/bin/python" - "$source_model" "$engine_model" <<'PY'
from pathlib import Path
import shutil
import sys
from ultralytics import YOLO

source = Path(sys.argv[1]).resolve()
target = Path(sys.argv[2]).resolve()
generated = Path(YOLO(str(source), task="segment").export(
    format="engine", imgsz=(480, 640), half=True, dynamic=False,
    batch=1, simplify=False, device=0,
)).resolve()
if generated != target:
    shutil.move(generated, target)
PY
fi

"$runtime_dir/bin/python" - "$engine_model" <<'PY'
import sys
import tensorrt
import torch
from ultralytics import YOLO

assert torch.cuda.is_available(), "CUDA is not available"
model = YOLO(sys.argv[1], task="segment")
model.predict(source=torch.zeros((480, 640, 3), dtype=torch.uint8).numpy(),
              imgsz=(480, 640), device=0, verbose=False)
print(f"TensorRT {tensorrt.__version__}: engine load and inference OK")
PY
