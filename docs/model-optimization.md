# Vision Model Optimization Report

Local-only optimization pass over the three FP32 ONNX models shipped with
the Olympus HLC. No retraining; no cloud; all work performed offline on the
dev machine. Optimized artifacts are written to
`recipes-apps/python3-rover-bridge/files/models-optimized/`.

This report records **honest** results — including regressions — so the
verification status in the README stays credible.

## Source models (all FP32, `dtype=FLOAT`)

| Model | Opset | IR | Input | Output | Size |
|---|---|---|---|---|---|
| `yolov8n.onnx` | 12 | 7 | `[1,3,640,640]` | `[1,84,8400]` | 12.85 MB |
| `yolov8n-seg.onnx` | 12 | 7 | `[1,3,640,640]` | `[1,116,8400]`, `[1,32,160,160]` | 13.87 MB |
| `lunar_seg.onnx` | 11 | 6 | `[1,3,384,384]` | `[1,5,384,384]` | 53.41 MB |

## Techniques applied

1. **Graph simplification / operator fusion (onnxslim)** — constant folding,
   identity removal, dead-code elimination, shape inference.
2. **ONNX-runtime graph partitioning / operator fusion** — `ORT_ENABLE_ALL`
   via `SessionOptions.optimized_model_filepath` (NchwcTransformer + conv fusions).
3. **INT8 dynamic quantization (weight-only, no calibration)** — applied to the
   two YOLO models via `onnxruntime.quantization.quantize_dynamic` (QInt8,
   op types `Conv`/`MatMul`/`Gemm`). No calibration dataset required.
4. **FP16 conversion** — applied to `lunar_seg` (segmentation-sensitive) via
   `onnxconverter_common.float16.convert_float_to_float16(keep_io_types=True)`
   so the I/O stays FP32-compatible with the current runtime.

## Results

Latency = mean of 10 ORT `InferenceSession` runs (CPUExecutionProvider,
1 intra-op thread) after 3 warmups, on the dev PC. **Not the RPi5.**

### yolov8n (bbox)

| Variant | Size | cv2.dnn loads? | ORT latency |
|---|---|---|---|
| `yolov8n.onnx` (baseline) | 12.85 MB | ✅ 239 layers | 107.9 ms |
| `yolov8n_slim.onnx` | 12.85 MB | ✅ 239 layers | 106.9 ms |
| `yolov8n_opt.onnx` (ORT-fused) | 12.84 MB | ❌ `com.microsoft.nchwc` ops | 107.4 ms |
| `yolov8n_int8.onnx` (dynamic) | 3.50 MB | ❌ `DynamicQuantizeLinear` unsupported | **450.6 ms** ⚠️ |

### yolov8n-seg (segmentation)

| Variant | Size | cv2.dnn loads? | ORT latency |
|---|---|---|---|
| `yolov8n-seg.onnx` (baseline) | 13.87 MB | ✅ 275 layers | 141.9 ms |
| `yolov8n-seg_slim.onnx` | 13.87 MB | ✅ 275 layers | 141.2 ms |
| `yolov8n-seg_opt.onnx` (ORT-fused) | 13.86 MB | ❌ `com.microsoft.nchwc` ops | 141.5 ms |
| `yolov8n-seg_int8.onnx` (dynamic) | 3.84 MB | ❌ `DynamicQuantizeLinear` unsupported | **622.5 ms** ⚠️ |

### lunar_seg (lunar navigation)

| Variant | Size | cv2.dnn loads? | ORT latency |
|---|---|---|---|
| `lunar_seg.onnx` (baseline) | 53.41 MB | ✅ 124 layers | 373.2 ms |
| `lunar_seg_slim.onnx` | 53.42 MB | ✅ 124 layers | 377.5 ms |
| `lunar_seg_opt.onnx` (ORT-fused) | 53.40 MB | ❌ `com.microsoft.nchwc` ops | 365.7 ms |
| `lunar_seg_fp16.onnx` (IO kept FP32) | 26.72 MB | ✅ 135 layers | 401.3 ms |

## Honest conclusions

### What worked

- **`lunar_seg_fp16.onnx`** — **halves the model size** (53.41 → 26.72 MB) and
  **loads in `cv2.dnn`** (the current Pi runtime). Latency cost on CPU is small
  (~7 % slower) and zero on hardware with native FP16. This is the one ready
  drop-in: point `LUNAR_MODEL_PATH` in `config.py` at the new file.

- **onnxslim** confirmed the Ultralytics/UNet exports are already minimal graphs
  (node counts unchanged, no constant-fold opportunities). So no further
  graph-simplification gains are available upstream of the export step.

### What did NOT work — and why (important)

- **Dynamic INT8 is a *size* win but a *latency regression* on plain CPU.**
  `yolov8n_int8` is 4× smaller but **4× slower** (450 ms vs 108 ms).
  Reason: dynamic quantization inserts `QuantizeLinear`/`DequantizeLinear`
  around activations, and the ORT CPU EP without an INT8 kernel for these ops
  falls back to FP32 + extra cast overhead. Dynamic INT8 only pays off when the
  backend has native INT8 kernels — which the standard ORT CPU build does **not**
  invoke at runtime for this QOperator format.

- **Dynamic INT8 does not load in `cv2.dnn` at all** — OpenCV's ONNX importer
  has no dispatch map for `DynamicQuantizeLinear`. So under the *current* Pi
  runtime, the INT8 YOLO models are unusable until the backend changes.

- **ORT-fused `*_opt` models do not load in `cv2.dnn`** — they contain
  `com.microsoft.nchwc` hardware-specific ops (NchwcTransformer). Runtime *is*
  the same CPU EP that produced them, so latency ≈ baseline anyway. These are
  backend-locked to onnxruntime and offer no speedup on the dev CPU measured
  here.

### Path to a real latency win

The optimization-only pass above cannot manufacture a speedup that the backend
doesn't support. A genuine latency improvement on the RPi5 requires one of:

1. **Switch the Pi runtime from `cv2.dnn` → onnxruntime.** Then the
   `*_int8.onnx` files become loadable, and ORT's MLAS INT8 conv kernels
   (present on aarch64) provide the actual speedup — but only via
   **static** quantization with a calibration set (which this repo does not
   yet have). Dynamic quantization (as done here) is the wrong tool for speed.
2. **Static INT8 quantization + a small calibration set (50–200 OV5647
   frames).** Needs representative images; out of scope for this pass
   (no training/calibration data staged yet).
3. **Move capture off the per-frame `rpicam-still` subprocess** (see the
   performance audit) — that change yields ~2× FPS without touching the model.

## Artifacts written to the repo

```
files/models-optimized/
├── yolov8n_int8.onnx          # 3.50 MB  — ORT-only; size win; latency reg.
├── yolov8n-seg_int8.onnx      # 3.84 MB  — ORT-only; size win; latency reg.
├── yolov8n_opt.onnx           # ORT-fused; backend-locked; ≈baseline
├── yolov8n-seg_opt.onnx       # ORT-fused; backend-locked; ≈baseline
├── lunar_seg_opt.onnx         # ORT-fused; backend-locked; ≈baseline
└── lunar_seg_fp16.onnx        # 26.72 MB — ✅ loads in cv2.dnn; drop-in ready
```

## Recommended action

Swap `lunar_seg` → `lunar_seg_fp16` (single `config.py` / YAML change, no
rebuild of weights). Keep the original FP32 YOLO models in place until the
runtime moves to onnxruntime + a real calibration set exists for static
quantization. The INT8/ORT-opt models are retained in
`files/models-optimized/` as future-ready artifacts, clearly marked with their
backend requirement in this report and in the directory README.

All work was done locally. No models were uploaded, downloaded, or trained.