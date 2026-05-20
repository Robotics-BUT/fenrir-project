# fenrir_sim — Phase 4 simulation (work in progress)

Gazebo Harmonic + ROS 2 Jazzy simulation of the Fenrir robot.

This is the **vertical slice** described in `bpc-prp-devel/MODERNIZATION_ROADMAP.md`
§8 de-risking note: enough robot + sensors + bridge to do **line following in
sim**, then expand to corridor and maze worlds.

## Status

| Piece | State |
|---|---|
| Robot description (URDF/xacro) | ✅ first cut, with placeholder dimensions to be refined from the SolidWorks CAD |
| Diff-drive + lidar + camera + IMU plugins | ✅ wired into the URDF |
| Line sensors | ❌ not implemented — needs a custom plugin or downward camera (T4.2 follow-up) |
| Encoders / ultrasounds / buttons / LEDs | ❌ not implemented — bridge stubs needed |
| `empty.sdf` (spawn-test world) | ✅ |
| `line.sdf` (line-following) | ✅ first cut, simple oval; refine with `tracks.drawio` geometry |
| `corridor.sdf` | ❌ T4.4 follow-up |
| `maze.sdf` | ❌ T4.4 follow-up |
| `motor_bridge` node (set_motor_speeds ↔ cmd_vel + watchdog) | ✅ |
| `ros_gz_bridge` config | ✅ for lidar / camera / IMU / cmd_vel / odom / TF / clock |
| `line.launch.py` | ✅ |
| `corridor.launch.py` / `maze.launch.py` | ❌ T4.5 follow-up |
| Sim Docker image (`docker/sim/Dockerfile`) | ✅ |

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

```bash
# forward, both wheels at half speed (~0.32 m/s):
ros2 topic pub --once /bpc_prp_robot/set_motor_speeds \
    std_msgs/msg/UInt8MultiArray '{data: [191, 191]}'

# stop:
ros2 topic pub --once /bpc_prp_robot/set_motor_speeds \
    std_msgs/msg/UInt8MultiArray '{data: [127, 127]}'

# spin left (right wheel forward, left wheel reverse):
ros2 topic pub --once /bpc_prp_robot/set_motor_speeds \
    std_msgs/msg/UInt8MultiArray '{data: [191, 63]}'
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

## Known assumptions / placeholders (refine before quantitative claims)

The numbers in `description/fenrir.urdf.xacro` are best-effort estimates:

- Chassis 0.16 m × 0.14 m × 0.05 m, mass 0.8 kg.
- Wheel radius 0.033 m, separation 0.20 m (matches MPC-MAP reference).
- Lidar at 0.10 m above the chassis, 12 Hz, 360 samples (RPLiDAR A1 spec).
- Camera 60° FOV, 640×480 at 30 Hz.
- IMU noise σ = 0.005 rad/s (gyro), 0.02 m/s² (accel). Loosely modelled on
  MPU6050 datasheet.

Cross-check against the editable SolidWorks CAD in
`bpc-prp-devel/hw_design/3d_model/` before relying on simulation outputs
for grading or quantitative arguments.

## Follow-up work (roadmap §8)

- **T4.2** — line sensors. Either a custom Gazebo plugin reading a texture
  mask under the robot, or a downward-pointing tiny camera + image-processing
  in the bridge. The roadmap flags this as the trickiest sensor.
- **T4.3** — finish the bridge: encoders (joint_states → uint32 ticks),
  ultrasounds, buttons, RGB LEDs, current_probes. Bit-identical match against
  Appendix B of the roadmap.
- **T4.4** — corridor world (Labs 8–9) and maze world (Labs 10–13, final
  exam). Maze STLs are in `3d-print/maze/`; use as gz models.
- **T4.5** — `corridor.launch.py`, `maze.launch.py`.
- **T4.6** — add a "Simulation" section to every hardware lab in `BPC-PRP/`.
