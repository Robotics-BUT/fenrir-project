# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

This is the public `fenrir-project` repo — the Fenrir robot (hardware + on-robot software). It is one of five repos in the `~/Developer/bpc-prp/` workspace; the workspace-root `CLAUDE.md` covers the broader picture, this file focuses on this repo only.

## What this repo is

Fenrir is an open-source educational robotic platform. It is the **physical robot that BPC-PRP bachelor students program**. The robot is a differential-drive chassis built around a **Raspberry Pi 4** (compute, ROS 2, WiFi, USB) and an **Arduino Nano Every** (peripheral I/O extender). This repo holds the robot's hardware design, on-robot software, and an mdBook documentation site.

## Branch state

> **⚠ PARKED until end of semester (decision 2026-05-20).** The BPC-PRP
> course is currently in session and runs against this robot. Hold the
> `modernization/phase-3 → main` merge until the active course run ends —
> mixing the Phase 1 doc updates into `main` while the rest of the
> modernization (T2.1 Pi-stack Jazzy migration, T3.3 robot-runtime image)
> is still pending could confuse anyone reading the latest fenrir docs.
> Pre-drafted PR description in `bpc-prp-devel/MODERNIZATION_ROADMAP.md`
> Appendix C (Merge 4/5).

- **`main`** is the pushed baseline.
- **`modernization/phase-1`** (pushed to origin) carries the `rgb_leds_handler` node-name fix (T1.9) and CLAUDE.md additions. 3 commits ahead of `main`.
- **`modernization/phase-3`** (pushed to origin, branched 2026-05-20 off phase-1) is where Phase 3 work *will* happen here — the `robot-runtime` Docker image (T3.3 in `bpc-prp-devel/MODERNIZATION_ROADMAP.md` §7). Currently just carries CLAUDE.md amendments describing the plan. 4 commits ahead of `main` (includes phase-1 ancestry).

**Phase 2 status note:** T2.1 (Pi workspace to ROS 2 Jazzy) and T2.3 (provisioning to Ubuntu 24.04) are **deferred** to a bundled hands-on robot session along with T2.5. Consequence: **T3.3 robot-runtime is BLOCKED** until T2.1 lands — the image needs the Pi nodes to build on Jazzy before it can be containerized. T2.2 (`main_controller` on Jazzy) is done in the `bpc-prp-devel` repo and does not affect this one.

Expect three known Jazzy-migration fixes when T2.1 is finally done (see roadmap §2.4):
- Every `uint*_t` / `int*_t` / `size_t` user needs `#include <cstdint>` (GCC 13.3 / libstdc++ 13 dropped transitive `<cstdint>` from `<cmath>` etc.).
- `cv_bridge/cv_bridge.h` is gone in Iron+; `camera_handler_cpp` must switch to `cv_bridge/cv_bridge.hpp`.
- The `rplidar` git submodule must be bumped to a Jazzy-supported tag/branch.

## Architecture — two-tier compute

```
ROS 2 graph (other machines / student code)
        ▲   /bpc_prp_robot/* topics
        │
  Raspberry Pi 4  — ROS 2 Humble nodes
        │   I2C bus
        ▼
  Arduino Nano Every — I2C slave @ 0x50, firmware: motor PWM + sensor sampling
        │
  motors · encoders · ultrasounds · line sensors · current probe
```

**Arduino Nano (`software/arduino_nano/main/main.ino`)** is an I2C slave at address `0x50`. It exposes a 21-byte register map (the `eeprom[]` array, layout documented in the file header): ultrasound distances, two 32-bit encoder counts, current probe, two line sensors, and two requested motor speeds. It samples sensors in a cooperative task loop and runs a per-wheel PID (encoder feedback, 576 pulses/rotation) that converts requested speed to motor PWM. Motor speed encoding: `127` = stop, `255` ≈ +1.28 m/s, `0` ≈ −1.27 m/s.

**Raspberry Pi** runs ROS 2 nodes that bridge hardware to the ROS graph. All robot peripherals on the I2C bus (Arduino at 0x50, BME280, MPU6050, ADS1115, DS3231, 20x4 LCD at 0x27) are read/written by the `i2c_handler` node. The Pi publishes/subscribes the `/bpc_prp_robot/*` topic namespace — **this is the contract BPC-PRP student code programs against**; do not rename topics without coordinating with the BPC-PRP course repos.

### Pi ROS 2 nodes (`software/raspberry_pi/ros2_ws/src/`)

| Package | Type | Role |
|---|---|---|
| `i2c_handler` | ament_python | Polls Arduino (0x50) + I2C sensors. Publishes `/bpc_prp_robot/ultrasounds`, `/line_sensors`, `/current_probes`, `/encoders`, `/imu`, `/mag`, `/temp`, `/air_press`, `/adc`, `/time`. Subscribes `/set_motor_speeds`, `/set_time`, `/set_lcd_text`, `/set_lcd_cursor`, `/set_lcd_clear`. |
| `buttons_handler` | ament_python | Publishes `/bpc_prp_robot/buttons` (GPIO buttons). |
| `rgb_leds_handler` | ament_python | Subscribes `/bpc_prp_robot/rgb_leds` (NeoPixel/WS281x LEDs). |
| `camera_handler_cpp` | ament_cmake (C++17) | Publishes `/bpc_prp_robot/camera` as `sensor_msgs/Image` (`bgr8`) via `image_transport` with the `compressed` transport hint; OpenCV-built frames. |
| `rplidar` | git submodule | Slamtec `rplidar_ros`; `rplidar_node` publishes `/scan` remapped to `/bpc_prp_robot/lidar`. |

### Launch split — root vs user nodes

- `launch/root_nodes.launch.py` — runs as **root** (`prp_root.service`, `User=root`): `rgb_leds_handler` (LED hardware needs root GPIO access).
- `launch/user_nodes.launch.py` — runs as the **`robot`** user (`prp_user.service`): `i2c_handler`, `buttons_handler`, `camera_handler_cpp`, `rplidar_node`.

Both launch files start their nodes with `respawn=True`; both services restart on failure.

## Common commands

### Build the Pi ROS 2 workspace
```sh
cd software/raspberry_pi/ros2_ws
git submodule update --init --recursive   # fetches the rplidar submodule
colcon build
source install/setup.bash
```
Toolchain: ROS 2 **Humble**, Ubuntu Server 22.04. The `rplidar` submodule must be present before `colcon build` succeeds.

### Run the robot nodes
```sh
# from software/raspberry_pi/ros2_ws/, after sourcing install/setup.bash
ros2 launch launch/root_nodes.launch.py
ros2 launch launch/user_nodes.launch.py
```
On the robot these run as the systemd services `prp_root.service` / `prp_user.service` (both `WantedBy=multi-user.target`, `ROS_DOMAIN_ID=0`).

### Arduino firmware
Open `software/arduino_nano/main/main.ino` in the Arduino IDE, select board **Arduino Nano Every** (the "Every" is required — it has enough external interrupts), Verify, then Upload over USB. There is no command-line build configured.

### Provisioning a fresh Raspberry Pi (`software/raspberry_pi/`)
Run in order — the system must be configured for the user **`robot`** (service files hard-code `/home/robot/...` paths):
1. `first_start.sh` — system update, edits `/boot/firmware/config.txt`, creates a 4 GB swapfile, adds the user to `video`/`tty`/`dialout`, upgrades systemd/udev. **Reboot after this.**
2. `install.sh` — installs apt + pip packages, installs ROS 2 Humble + colcon, clones this repo, builds the ROS 2 workspace, and installs + enables `prp_root.service` / `prp_user.service`.
- `slow_startup.sh` — optional boot-time reduction: masks `systemd-networkd-wait-online`, disables snap and cloud-init services.

### Documentation site (mdBook)
```sh
mdbook build      # outputs to ./book
mdbook serve      # local preview
```
`book.toml` sources from `src/`. The `.github/workflows/github-pages.yaml` workflow builds with mdBook 0.4.10 and deploys `./book` to GitHub Pages on push to `main`/`master`.

## Documentation structure (`src/`, ordered by `SUMMARY.md`)

1. `1_overview/` — project overview
2. `2_hardware/` — 3D-printed parts, custom PCBs, modules, wiring, bill of materials
3. `3_software/` — software architecture: Raspberry Pi software, Arduino firmware, I2C communication
4. `4_assembly/` — step-by-step robot assembly
5. `5_setup/` — Raspberry Pi installation and Arduino programming

Hardware design files live outside `src/`: `pcbs/` (KiCad), `3d-print/` (STL incl. the modular maze), `drawings/`, `media/`. The full wiring table and component/equipment list are in `README.md`.
