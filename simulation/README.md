# fenrir_sim — Phase 4 simulation (work in progress)

Gazebo Harmonic + ROS 2 Jazzy simulation of the Fenrir robot.

The vertical slice described in `bpc-prp-devel/MODERNIZATION_ROADMAP.md`
§8 de-risking note has shipped end-to-end: a tunable PID line follower
drives the simulated robot around the line.sdf track using the same
`/bpc_prp_robot/*` contract a real robot would expose.

## Status

| Piece | State |
|---|---|
| Robot description (URDF/xacro) | ✅ dimensions match the real Fenrir spec (15 cm cube chassis, 12 cm wheelbase, 33 mm radius wheels, 30 mm wheel width, front + rear casters) |
| Diff-drive + lidar + camera + IMU plugins | ✅ wired into the URDF |
| Ultrasounds (forward + 45° L + 45° R, mid-height) | ✅ URDF sensors in place, gz topics `/us_{front,left,right}`. T4.3 follow-up bridges them to `/bpc_prp_robot/ultrasounds` |
| Line sensors | ✅ downward floor-camera + line_sensor_bridge samples two pixel positions slightly left and right of the front leg. Polarity matches real ADC: **white floor → low, black line → high**, range 0..1023 |
| LiDAR (Fenrir mounting: 180° backward, CW scan) | ✅ lidar_bridge re-orders the gz scan so `ranges[0]` = backward, `ranges[N/4]` = LEFT, `ranges[N/2]` = forward, `ranges[3N/4]` = RIGHT, with `angle_increment` = −2π/N |
| Encoders / buttons / RGB LEDs / ultrasounds | ❌ T4.3 follow-up — bridge stubs needed |
| `empty.sdf` (spawn-test world) | ✅ |
| `line.sdf` (line-following) | ✅ first cut, simple rectangle with 90° corners; the actual `tracks.drawio` geometry is a separate follow-up |
| `corridor_{straight,loop,double_loop}.sdf` | ✅ Lab 12 exam tracks (40 cm cells, 30 cm tall red walls, hollow inner cells) |
| `maze.sdf` | ❌ T4.4 follow-up |
| `motor_bridge` node (set_motor_speeds ↔ cmd_vel + 1 s watchdog) | ✅ |
| `ros_gz_bridge` config | ✅ for camera / IMU / floor_camera / lidar (→ /internal/lidar) / cmd_vel / odom / TF / clock |
| `line.launch.py` | ✅ |
| `corridor.launch.py` (with `world:=…` arg) | ✅ |
| `maze.launch.py` | ❌ T4.5 follow-up |
| Sim Docker image (`docker/sim/Dockerfile`) | ✅ |
| **End-to-end closed-loop demos** | ✅ `examples/line_follower.py` (P-only line PID) + `examples/corridor_follower.py` (lidar-based corridor centering with turn-at-corner) |

## How to run

The simulation runs **on demand** inside the `bpc-prp-sim:jazzy` Docker image
— no native Gazebo install needed on the host.

### One-time: build the images

```bash
# from BPC-PRP/ root, build the Phase 3 base if you do not already have it
docker build \
  --build-arg UID=$(id -u) --build-arg GID=$(id -g) \
  -t bpc-prp-base:jazzy \
  docker/base

# from fenrir-project/ root, build the sim image
docker build -t bpc-prp-sim:jazzy docker/sim
```

### Launch the line-following world

```bash
# Linux + X11 host:
xhost +local:docker

docker run --rm -it \
  --net=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "$PWD:/workspace" \
  -w /workspace \
  bpc-prp-sim:jazzy bash -lc '
    colcon build --packages-select fenrir_sim &&
    source install/setup.bash &&
    ros2 launch fenrir_sim line.launch.py
  '
```

Headless smoke-test (no GUI window — useful in CI / on a robot):

```bash
docker run --rm -i \
  --net=host \
  -v "$PWD:/workspace" -w /workspace \
  bpc-prp-sim:jazzy bash -lc '
    colcon build --packages-select fenrir_sim &&
    source install/setup.bash &&
    ros2 launch fenrir_sim line.launch.py headless:=true
  '
```

### Drive the robot

Standard `/bpc_prp_robot/*` contract. From another terminal in the same
container or from a host with ROS 2 Jazzy:

All multi-element `/bpc_prp_robot/*` topics are ordered **left-to-right**:

| Topic | Data layout |
|---|---|
| `set_motor_speeds` | `[left, right]` uint8 ×2 |
| `line_sensors`     | `[left, right]` uint16 ×2 |
| `ultrasounds`      | `[left, center, right]` uint8 ×3  *(T4.3 follow-up)* |

```bash
# forward, both wheels at ~0.5 m/s (191 = ~0.5 m/s above 127=stop):
ros2 topic pub --once /bpc_prp_robot/set_motor_speeds \
    std_msgs/msg/UInt8MultiArray '{data: [191, 191]}'   # [left, right]

# stop:
ros2 topic pub --once /bpc_prp_robot/set_motor_speeds \
    std_msgs/msg/UInt8MultiArray '{data: [127, 127]}'

# spin left in place (left wheel reverse, right wheel forward):
ros2 topic pub --once /bpc_prp_robot/set_motor_speeds \
    std_msgs/msg/UInt8MultiArray '{data: [63, 191]}'    # [left, right]
```

The 1-second watchdog stops the robot if no message arrives — matches the
real Fenrir firmware.

### Inspect topics

```bash
ros2 topic list                       # /bpc_prp_robot/{lidar,camera,imu,...}
ros2 topic hz   /bpc_prp_robot/lidar  # ~12 Hz expected
ros2 topic echo /bpc_prp_robot/imu --once
```

## Architecture (sim vs real robot)

```
real robot                            sim
  ros 2 nodes                           ros 2 nodes
  on Pi:                                in container:
    i2c_handler                            (none — the sim IS the hardware)
    rgb_leds_handler
    camera_handler_cpp
    rplidar_node
  ↓                                     ↓
  /bpc_prp_robot/*  (the contract)      /bpc_prp_robot/*  (same contract)
  ↑                                     ↑
  student code                          student code (BYTE-IDENTICAL)
```

**Student code does not change** between sim and real robot. That is the
whole point. If your code follows a line in `line.launch.py`, it should
follow a line on the physical Fenrir too — within sensor noise.

## Robot dimensions and sensor layout

Confirmed against the real Fenrir spec (Adam, 2026-05-20) and the STL
files in `fenrir-project/3d-print/robot/`:

| Quantity | Value | Source |
|---|---|---|
| Chassis bounding box | 150 × 150 × 150 mm cube | spec; `plate_*` STLs are 150×150 mm in footprint |
| Chassis mass (sim estimate) | 1.0 kg | rough, no real weighing yet |
| Wheel radius | 33 mm | spec (STL says 34 mm — 1 mm tolerance) |
| Wheel width | 30 mm | from `wheel.STL` |
| Interwheel base | 120 mm | spec |
| LiDAR | XY-center, 20 mm above chassis top, 12 Hz, 360 samples | RPLiDAR A1 spec |
| LiDAR mounting | rotated 180° about z (0° points BACKWARD); scan is **clockwise** | real Fenrir hardware (Adam 2026-05-20) |
| Camera | front-top-edge of chassis, pitched 45° down, 640×480 @ 30 Hz, ~60° FOV | RPi camera |
| Ultrasounds (3×) | front face, mid-height, +5 mm stick-out; left/right at front-corners with ±45° yaw | spec |
| Line sensors | mounted at front-leg X (caster_front_x = 65 mm forward of center), centerline; two virtual samples ~2 cm apart on the floor | spec ("slightly left and right from the front leg") |
| Casters | front + rear sphere casters at x = ±65 mm, both at ground level | approximates the real `leg_front_2` skid + `leg_rear_2` support + `ball_2` |

Wheel mass, caster radius, IMU noise (σ = 0.005 rad/s gyro, 0.02 m/s²
accel) and friction (μ = 1.0 wheels, 0.05 casters) are sim defaults
that still want tuning against the real MPU6050 datasheet and a real
weigh-in.

The chassis collision is a single solid 15 cm cube — the real Fenrir is
three plates with pillars and air gaps in between. The simplification
matters for the LIDAR (which sits above the chassis and scans
horizontally so the chassis never occludes the beam) and for sensors
mounted inside the chassis volume (the gpu_lidar would falsely clamp to
min_range), which is why the ultrasounds stick out 5 mm past the front
face.

## Topic-ordering convention

All multi-element `/bpc_prp_robot/*` topics are **left → right**:

| Topic | Type | Layout |
|---|---|---|
| `set_motor_speeds` | `UInt8MultiArray` | `[left, right]`, 0..255 with 127 = stop |
| `line_sensors` | `UInt16MultiArray` | `[left, right]`, 0..1023 ADC (white→low, black→high) |
| `ultrasounds` *(T4.3)* | `UInt8MultiArray` | `[left, center, right]`, cm |
| `encoders` *(T4.3)* | `UInt32MultiArray` | `[left, right]`, ticks (576 ppr) |
| `rgb_leds` *(T4.3)* | `UInt8MultiArray` | `[led0_r, led0_g, led0_b, led1_r, ...]` |

## Follow-up work (roadmap §8)

- **T4.3** — finish the bridge: encoders (joint_states → uint32 ticks),
  ultrasounds, buttons, RGB LEDs, current_probes. Bit-identical match against
  Appendix B of the roadmap.
- **T4.4** — corridor world (Labs 8–9) and maze world (Labs 10–13, final
  exam). Maze STLs are in `3d-print/maze/`; use as gz models.
- **T4.5** — `corridor.launch.py`, `maze.launch.py`.
- **T4.6** — add a "Simulation" section to every hardware lab in `BPC-PRP/`.

## Corridor following (Lab 12)

Three exam tracks per BPC-PRP Lab 12. 40 cm × 40 cm grid cells, red
walls 30 cm tall, hollow inner cells (4 narrow walls each).

| World | Layout |
|---|---|
| `corridor_straight.sdf` | 5 cells long × 1 cell wide, dead-ends at both ends |
| `corridor_loop.sdf` | 5×5 cells outer wall + 3×3 cells hollow inner ring → 0.4 m corridor ring |
| `corridor_double_loop.sdf` | Figure-8: two 3×3 loops sharing one corner cell, each with a hollow inner cell |

```bash
ros2 launch fenrir_sim corridor.launch.py world:=corridor_straight.sdf
ros2 launch fenrir_sim corridor.launch.py world:=corridor_loop.sdf
ros2 launch fenrir_sim corridor.launch.py world:=corridor_double_loop.sdf
```

### Lidar contract (Fenrir-specific)

The real Fenrir RPLiDAR is mounted **180° rotated about z** (0° points
backward) and its scan rotates **clockwise**. The `lidar_bridge` node
matches that contract on the simulated `/bpc_prp_robot/lidar`:

```
ranges[0]              distance directly BEHIND the robot
ranges[N/4]            distance to robot's LEFT
ranges[N/2]            distance directly AHEAD
ranges[3*N/4]          distance to robot's RIGHT
angle_min       = +π
angle_increment = -2π / N         (negative → CW)
```

Sample 0 is preserved as backward; the rest of the array is reversed
to flip CCW (gz native) → CW. Index-based access (`ranges[N/2]` etc.)
is the recommended way to read the scan.

### Corridor follower demo (`examples/corridor_follower.py`)

State machine on three lidar windows (left, forward, right, each
averaged across ±3 samples):

| State | When | Behaviour |
|---|---|---|
| `TURN-LEFT` / `TURN-RIGHT` | forward < `turn_trigger_dist` (0.35 m default) | rotate in place toward the **wider** side until forward clears |
| `BALANCE` | both walls within `wall_limit` (0.40 m default) | PID on `right − left` to center the robot |
| `HUG-LEFT` / `HUG-RIGHT` | only one wall within `wall_limit` | keep `side_target_dist` (0.18 m default) from that wall |
| `OPEN` | no walls within `wall_limit` | drive straight |

Key trick: **walls farther than `wall_limit` (1 cell) are ignored** for
steering. Without this clamp the controller balances a 2 m open
direction against a 20 cm wall and steers itself into a corner.

Verified on all three tracks:

- `corridor_straight`: drives forward centered, slows and stops at the
  north dead-end.
- `corridor_loop`: completes the ring, picking the wider side at each
  90° corner.
- `corridor_double_loop`: navigates the figure-8 across the shared
  cell, transitioning HUG ↔ BALANCE as the lidar topology changes.

## End-to-end demo (line following)

`examples/line_follower.py` is a small two-state controller that proves
the contract closes a real control loop. It uses only `/bpc_prp_robot/*`
topics — no sim-specific shortcuts. Same code would run on the real
robot.

```bash
# terminal 1 — inside bpc-prp-sim:jazzy:
ros2 launch fenrir_sim line.launch.py            # with GUI
# or:
ros2 launch fenrir_sim line.launch.py headless:=true

# terminal 2 — same container (docker exec) or any ROS 2 Jazzy host:
python3 /workspace/simulation/examples/line_follower.py
```

### Controller logic

```
TRACKING  if max(line_sensors) > threshold   # black line under at least one sensor
          err = right - left                 # > 0 when line is to the right
          u   = kp*err + kd*(err - prev_err) # current default: kp=0.075, kd=0
          motors = [127 + base + u,          # left
                    127 + base - u]          # right

LOST      else                                # both sensors over white
          # rotate toward last-known line direction at a slow crawl
          u    = sign(last_err) * search_turn
          base = search_base
```

Current defaults (gentle, smooth, slow):

| Parameter | Value | Note |
|---|---|---|
| `base_byte_above_stop` | 9 | ~0.09 m/s forward — half the original demo speed |
| `kp` | 0.075 | gentle P-only, tuned 2026-05-20 |
| `kd` | 0.0 | D off |
| `line_threshold` | 400 | mid-scale of 0..1023 |
| `search_turn` | 55 | strong fixed turn while LOST |
| `search_base` | 8 | slow forward crawl while LOST |

### Limits of the current demo

The placeholder `line.sdf` is a rectangle with sharp 90° corners. A
P-only controller is gentle enough to track straights smoothly and to
recover from minor drift via the LOST-state rotate-and-search behaviour,
but the corners are still hard. Two cheap follow-ups close the gap:

1. Replace the placeholder track with curved segments or the actual
   `tracks.drawio` geometry the course uses.
2. Try the `bpc-prp-devel` `solution` package against the sim — that is
   the strongest validation, since it is the same code real students
   run on the real robot.

### Line-sensor calibration notes

The line-sensor bridge has tunable parameters in `launch/line.launch.py`:

| Param | Default | What it does |
|---|---|---|
| `left_col_frac`, `right_col_frac` | 0.25, 0.75 | Pixel column (as a fraction of image width) where each virtual sensor samples. Default puts them ~2.3 cm apart on the floor; tighten if the real Fenrir IR sensors turn out to be closer together. |
| `sample_radius` | 2 | Half-side of a (2r+1)² px window averaged at each sample point. Smooths noise. |
| `row_frac` | 0.5 | Image row to sample; 0.5 = center, directly under the front of the robot. |
| `max_reading` | 1023 | ADC scale; matches MODERNIZATION_ROADMAP §B. |

Observed readings in the `line.sdf` world after the 2026-05-20 polarity
flip:

| Position | Reading | Comment |
|---|---|---|
| ON line | `[800, 810]` | both high, ~80% of full scale |
| OFF line, well-lit | `[240, 270]` | both low, white floor |
| OFF line, shaded | `[130, 160]` | lighting non-uniform; threshold-based code still discriminates cleanly |

**Polarity** (matches real Fenrir ADC): white floor → low reading,
black line → high reading. If real hardware turns out to be inverted,
flip the one-line formula in
`fenrir_sim/line_sensor_bridge.py::_brightness_to_reading` — no other
contract change.
