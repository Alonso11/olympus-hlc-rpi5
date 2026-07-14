# Olympus HLC — RPi5 Yocto Image
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Systems Engineering](https://img.shields.io/badge/Focus-Systems%20Engineering-blue.svg)](#)

## Overview

Custom Linux image for the **Raspberry Pi 5** built with Yocto (Scarthgap).
Acts as the **High-Level Controller (HLC)** of the Olympus rover, communicating
with an Arduino Mega 2560 (Low-Level Controller) over UART/USB using the MSM
protocol. Long-range telecommand and telemetry use **CSP (CubeSat Space
Protocol)** with **SFP** (Serial Fragmentation Protocol) for image and video
transfer, sourced from the parent systems-engineering project
[Olympus-Project-TFG-TEC](https://github.com/Alonso11/Olympus-Project-TFG-TEC),
which holds the IEEE 29148 SRS the design is traced to.

---

## Technical Highlights

- **Dual HLC/LLC architecture** — the Raspberry Pi 5 (HLC, this repository) runs
  GNC + YOLOv8n-seg vision (≤2 s cycle), while an ATmega2560 Rust firmware
  ([rover-low-level-controller](https://github.com/Alonso11/rover-low-level-controller))
  owns hard real-time motor/encoder control (20 ms loop), so a software fault on
  the HLC cannot compromise physical safety.
- **CSI camera setup (config-ready, single-module validated)** — device-tree
  overlays are deployed for both IMX219 on CAM0 and OV5647 on CAM1
  (EEPROM-less modules; `camera_auto_detect=0`), but only the **OV5647 on CAM1
  has been validated on hardware**; IMX219 on CAM0 is configured but untested.
- **Packet-radio stack (untested)** — CSP/RDP/SFP over a NinoTNC 9600A KISS TNC
  + Baofeng UHF radio, with a `ninotnc-probe` presence detector that gates the
  `csp-rover` systemd service so it only runs when the TNC is connected.
  End-to-end UHF communication has **not** been validated yet.
- **Verification status:** MSM HLC↔LLC bridge — verified; YOLOv8n-seg vision
  (OV5647) — verified in lab; CSP/SFP over UHF — untested (pending hardware
  validation); IMX219 CAM0 — configured, untested.

---

## Repository Structure

```
olympus-hlc-rpi5/
├── 📄 README.md                          # This file
├── 📄 LICENSE                            # MIT
├── 📄 deploy-hlc-ssh.sh                  # Remote deploy helper
├── 📂 scripts/                           # setup-env.sh, flash/deploy helpers
├── 📂 docs/                              # architecture, testing, build, decision-log…
│
├── 📂 layers/
│   ├── 📂 meta-olympus/                  # Project Yocto layer
│   │   ├── 📂 conf/                      # Layer config
│   │   ├── 📂 recipes-core/              # olympus-image, custom-udev-rules
│   │   ├── 📂 recipes-apps/              # python3-rover-bridge (Rust/PyO3)
│   │   ├── 📂 recipes-bsp/bootfiles/     # rpi-config bbappend, ov5647.dtbo
│   │   ├── 📂 recipes-connectivity/      # libcsp, csp-sfp-rover, wifi-config, olympus-ap
│   │   ├── 📂 recipes-kernel/linux/      # Kernel config fragments
│   │   ├── 📂 recipes-multimedia/        # libcamera, libcamera-apps
│   │   └── 📂 recipes-support/           # opencv, resize-rootfs
│   ├── meta-raspberrypi/                 # Raspberry Pi hardware support
│   ├── meta-openembedded/
│   ├── meta-onnxruntime/
│   └── meta-tensorflow-lite/
│
└── 📂 build/conf/local.conf              # Build config (MACHINE, RPI_EXTRA_CONFIG…)
```

## Architecture

```
┌─────────────────────────────────────────┐       ┌──────────────────────────────────┐
│  Raspberry Pi 5 (HLC)                   │       │  Arduino Mega 2560 (LLC)         │
│                                         │       │                                  │
│  olympus_hlc/ (v3.0)                    │       │  MSM: STB/EXP/AVD/RET/FLT       │
│  ├── HlcEngine (bucle de control)       │       │                                  │
│  ├── VisionSource (YOLOv8n ONNX)        │       │  6 Motors (PWM L298N)            │
│  ├── ManualSource (stdin)               │       │  HC-SR04 D38/D39                 │
│  ├── GCSSource (UDP CSP/CRC-32)         │       │  VL53L0X ToF I2C                 │
│  ├── WaypointTracker                    │       │  6 Hall encoders (INT0–INT5)     │
│  ├── EnergyMonitor (4S Li-ion)          │       │                                  │
│  └── OlympusLogger → /var/log/          │       │                                  │
│                                         │       │                                  │
│  rover_bridge.so (Rust/PyO3) ───────────┼─ USB ─┼─── USART0 (CDC-ACM 115200 8N1)  │
│  /dev/arduino_mega                      │       └──────────────────────────────────┘
│                                         │
│  ┌─ CSP / SFP ──────────────────────┐   │
│  │  csp_sfp_rover (systemd)         │   │
│  │  ├── KISS TNC @ 57600 baud       │───┼── UART GPIO ─── NinoTNC 9600A ─── Baofeng
│  │  ├── SFP: image + video + tlm    │   │
│  │  └── RDP: window=4, timeout=15s  │   │
│  └──────────────────────────────────┘   │
│                                         │
│  Cámara CSI IMX219 (CAM0)  [config-only]│
│  Cámara CSI OV5647 (CAM1)  [validada]   │
└─────────────────────────────────────────┘
```

See [docs/architecture.md](docs/architecture.md) for the full system overview.

---

## Quick Start

### 1. Clone and set up the environment

```bash
git clone https://github.com/Alonso11/olympus-hlc-rpi5.git
cd olympus-hlc-rpi5
./scripts/setup-env.sh
```

The script automatically clones: poky, meta-raspberrypi, meta-openembedded.

### 2. Build the image

```bash
source layers/poky/oe-init-build-env build
bitbake olympus-image
```

### 3. Flash the microSD

```bash
# Download image from GCP VM
~/deploy-olympus-image.sh

# Flash (requires sudo and bmaptool)
sudo ~/flash-olympus-image.sh
```

See [docs/build-and-deploy.md](docs/build-and-deploy.md) for detailed instructions.

---

## Running the controller

```bash
ssh root@<IP_RPi5>

# Manual mode — send MSM commands from stdin
python3 -m olympus_hlc --mode manual

# Vision mode — obstacle detection via YOLOv8n + CSI camera
python3 -m olympus_hlc --mode vision

# GCS mode — UDP commands from Ground Control Station (CSP/CRC-32, SRS-013)
python3 -m olympus_hlc --mode gcs

# Dry-run (no Arduino required — for testing)
python3 -m olympus_hlc --mode manual --dry-run

# Custom log path
python3 -m olympus_hlc --mode vision --log-path /var/log/olympus/mission.log
```

> The legacy `olympus_controller.py` script (v2.4) is still installed at
> `/usr/bin/olympus_controller.py` for backwards compatibility.

The controller connects to `/dev/arduino_mega` at 115200 baud and manages
the MSM state machine (STB → EXP → AVD/RET → STB). It sends `PING` every 1 s
when idle to keep the Arduino watchdog alive (~2 s timeout → FAULT).

### MSM commands

| Command | Action |
|---------|--------|
| `STB` | Standby — motors stopped |
| `EXP:<l>:<r>` | Explore at speeds 0–100 (e.g. `EXP:80:80`) |
| `AVD:L` / `AVD:R` | Avoidance turn left / right |
| `RET` | Retreat |
| `RST` | Reset → Standby |
| `PING` | Keepalive — resets Arduino watchdog |

---

## CSP / SFP Rover

> ⚠️ **Status: untested.** The CSP/SFP rover stack compiles and the
> `ninotnc-probe` gates the service, but end-to-end UHF communication over the
> NinoTNC + Baofeng has **not** been validated on hardware yet. RDP parameters
> below are analytically tuned estimates (BDP-based), not measured values.

The `csp-sfp-rover` service provides long-range telecommand and telemetry
over UHF packet radio using the NinoTNC 9600A KISS TNC and a Baofeng radio:

```
systemctl status csp-rover.service
```

### Protocol stack

| Layer | Component |
|-------|-----------|
| Link | AX.25 (KISS) over UART @ 57600 baud |
| Network | CSP (CubeSat Space Protocol) |
| Transport | RDP (Reliable Datagram Protocol) |
| Fragmentation | SFP (Serial Fragmentation Protocol) |
| Application | CSP port 10 — commands: `i` (image), `v` (video), `d` (telemetry dump) |

### RDP tuning (half-duplex UHF)

Tuned analytically (BDP-based) for NinoTNC + Baofeng — **pending hardware
validation**:

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| window_size | 4 | BDP-limited: 960 B/s × 0.838 s ≈ 804 B → 5 frames max, safe at 4 |
| packet_timeout | 15000 ms | Must exceed RTT under congestion (2000–4000 ms) |
| conn_timeout | 60000 ms | Covers long image transfers |
| delayed_acks | 0 | Immediate ACK — avoids timeout expiry in half-duplex |
| ack_timeout | 2000 ms | One-way half-duplex turnaround |

### Conditional start

The service only starts when the NinoTNC is physically connected:

```bash
/usr/bin/ninotnc-probe   # → exit 0 if TNC responds to KISS GETALL
```

`ExecStartPre` in `csp-rover.service` runs the probe before launching
`csp_sfp_rover`. If the TNC is absent (bench testing), the service stays
inactive — no wasted CPU or memory.

### SFP commands (on CSP port 10)

| Command | Response |
|---------|----------|
| `i` | Sends `rover_test.jpg` via SFP fragmentation |
| `v` | Sends `rover_test.mp4` via SFP fragmentation |
| `d` | 31-field telemetry dump (distances, voltages, IMU, currents, temps, encoders) |
| `t` | Temperature string |
| `h` | Humidity string |
| `s` | Temperature + humidity |

---

## What the image includes

| Component | Description |
|-----------|-------------|
| `rover_bridge.so` | Rust/PyO3 module — MSM UART protocol with the Arduino |
| `olympus_hlc/` | HLC package v3.0 (SOLID refactor) — `python3 -m olympus_hlc` |
| `olympus_controller.py` | Legacy HLC controller v2.4 (kept for backwards compat) |
| `olympus_controller.yaml` | Operational config at `/etc/olympus/` (editable without rebuilding) |
| `yolov8n.onnx` | Obstacle detection model (YOLOv8n opset 12, 13 MB) |
| `yolov8n-seg.onnx` | Segmentation model (YOLOv8n-seg opset 12, 14 MB — GNC-REQ-002) |
| `custom-udev-rules` | Stable symlink `/dev/arduino_mega` |
| `wifi-config` | Automatic WiFi connection (wpa_supplicant) |
| `wifi-power-save` | WiFi power saving (systemd oneshot) |
| `resize-rootfs` | rootfs expansion on first boot |
| OpenCV (cv2.dnn) | Computer vision and ONNX inference |
| libcamera | CSI camera support (rpi/pisp pipeline, RPi5) |
| `libcsp` | CSP library v4.2 (ELANav fork — SFP, RDP tuning, UDP, KISS interfaces) |
| `libcsp_py3.so` | Python 3 CSP bindings (UDP init, KISS init, buffer management) |
| `csp_sfp_rover` | Rover SFP node — telemetry, image, and video over UHF packet radio |
| `ninotnc-probe` | NinoTNC presence detector — enables rover service conditionally |
| `csp-rover.service` | systemd service (probe → rover, restart on failure) |
| `ov5647.dtbo` | Device tree overlay for OV5647 camera (CAM1) |
| Test scripts | `/usr/bin/test_bridge_interactive.py`, etc. |

---

## Configuration

Operational parameters can be changed on the RPi5 without rebuilding the image:

```bash
nano /etc/olympus/olympus_controller.yaml
```

Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ping_interval_s` | 1.0 | PING keepalive interval (s) |
| `tlm_timeout_s` | 5.0 | Link-loss timeout (s) → forces STB |
| `retreat_dist_mm` | 300 | Tactical HLC retreat threshold (mm) |
| `batt_warn_mv` | 14000 | Battery WARN level (3.5 V/cell × 4S) |
| `batt_critical_mv` | 12800 | Battery CRITICAL → force STB (3.2 V/cell × 4S) |
| `vision_conf_min` | 0.5 | Minimum YOLOv8n detection confidence |

---

## Development environment (PC)

All PC-side tools and tests use [uv](https://github.com/astral-sh/uv):

```bash
cd layers/meta-olympus/recipes-apps/python3-rover-bridge/files/

# Create venv and install dev deps (pytest, opencv-python, numpy)
uv sync --dev

# Run unit tests
uv run pytest tests/ -v

# Run debug_view.py (part of SSH pipe workflow — see below)
uv run python3 debug_view.py
```

---

## Testing on the RPi5

```bash
ssh root@<IP_RPi5>
test_bridge_interactive.py   # Manual MSM command prompt
test_bridge.py               # Automated send/receive test
test_opencv_camera.py        # CSI camera capture + edge detection
test_ultrasonic_rpi.py       # HC-SR04 via RPi5 GPIO (future sensor)
test_rover.py                # Basic rover_bridge smoke test
```

See [docs/testing.md](docs/testing.md) for the full testing guide.

### Live vision debug (SSH pipe)

Stream annotated inference frames from the RPi5 to your PC in real time:

```bash
# On your PC — usar el entorno uv (ver sección Development environment):
uv run python3 debug_view.py   # recibe el pipe

# Bbox mode (faster):
ssh root@<IP_RPi5> "python3 /opt/olympus/debug_vision.py --mode bbox" | python3 debug_view.py

# Segmentation mode (shows masks + zone coverage):
ssh root@<IP_RPi5> "python3 /opt/olympus/debug_vision.py --mode seg" | python3 debug_view.py

# Capture N frames only:
ssh root@<IP_RPi5> "python3 /opt/olympus/debug_vision.py --mode seg --frames 20" | python3 debug_view.py
```

`debug_vision.py` runs on the RPi5 and writes length-prefixed JPEGs to stdout.
`debug_view.py` runs locally and displays them with `cv2.imshow`. Press `q` to quit.

The annotated frame shows:
- Vertical zone lines at 33 % / 67 % of frame width (LEFT / CENTER / RIGHT)
- Bounding box of best detection (bbox mode) or green mask overlay (seg mode)
- Zone coverage percentages in the ROI (seg mode)
- Decided MSM command (green = EXP, red = AVD/RET)

---

## Documentation

| Doc | Description |
|-----|-------------|
| [docs/architecture.md](docs/architecture.md) | System overview |
| [docs/rover-bridge.md](docs/rover-bridge.md) | Rust/PyO3 module, API, MSM protocol |
| [docs/yocto-recipes.md](docs/yocto-recipes.md) | meta-olympus recipes and image packages |
| [docs/testing.md](docs/testing.md) | How to test each component |
| [docs/build-and-deploy.md](docs/build-and-deploy.md) | Build and flash the image |
| [docs/decision-log.md](docs/decision-log.md) | Chronological design decision log |

---

## License

This project is distributed under the MIT License. See the LICENSE file for details.

---

## Author

Fabián Alonso Gómez Quesada     
Instituto Tecnológico de Costa Rica (TEC)        
School of Electronics Engineering           
SETEC Lab – Space Systems Laboratory     
