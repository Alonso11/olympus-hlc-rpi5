# Optimized vision model artifacts

Produced by the local-only optimization pass documented in
[../../docs/model-optimization.md](../../../docs/model-optimization.md).

## Committed (drop-in / future-ready)

| File | Size | cv2.dnn | onnxruntime | Notes |
|---|---|---|---|---|
| `lunar_seg_fp16.onnx` | 26.72 MB | ✅ | ✅ | **Drop-in for `lunar_seg`.** Half-size, IO kept FP32. Point `LUNAR_MODEL_PATH` at this file to use. |
| `yolov8n_int8.onnx` | 3.50 MB | ❌ | ✅ | INT8 dynamic (weight-only). Size win only; latency regresses on CPU until the Pi runtime moves to onnxruntime and a static-quant calibration set exists. |

> ⚠️ `yolov8n_int8.onnx` uses `DynamicQuantizeLinear`, which OpenCV's ONNX
> importer cannot dispatch. Do **not** point `vision.py` / `config.py` at it
> under the current OpenCV backend — loading will throw
> `error: (-2:Unspecified error): Node [...] failed to parse`.

## Not committed (reproducible on demand)

The following artifacts were also produced locally and are retained in
`/tmp/opencode/model-opt/work/` during the optimization pass, but are **not**
tracked in git to keep the repo lean. They are trivially reproducible from
the recipe in [docs/model-optimization.md](../../../docs/model-optimization.md):

- `yolov8n-seg_int8.onnx` (3.84 MB) — INT8 dynamic, ORT-only.
- `yolov8n_opt.onnx` / `yolov8n-seg_opt.onnx` / `lunar_seg_opt.onnx` —
  ORT `ORT_ENABLE_ALL`-fused; backend-locked to onnxruntime via
  `com.microsoft.nchwc` ops; ≈ baseline latency.
- `*_slim.onnx` — onnxslim output; node counts identical to baseline (the
  Ultralytics/UNet exports are already minimal graphs).