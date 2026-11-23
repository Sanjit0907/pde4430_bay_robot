# PDE4430 Bay Robot

Coursework for **PDE4430 Mobile Robotics** (Middlesex University Dubai, 2026–27).

A differential-drive mobile robot with a passive **V-cradle** pusher. It moves a large sphere, a small sphere and an upright cylinder through the opening of the `assessment_bay` loading bay, using **physical contact only**. There are no attach joints and no pose plugins.

> Status: robot model, Gazebo integration and test tools done. Teleoperation tools and autonomy are in progress.

---

## 1. Approach and robot design

| Part | Choice | Why |
|---|---|---|
| Base | 2 driven wheels + rear ball caster, 0.30 × 0.30 m chassis | Simple, turns on the spot, easy to teleoperate precisely |
| Pusher | V-shaped cradle: 0.16 m flat back plate + two 45° wings | Wings funnel round objects to the centre so they don't roll off sideways. The flat back gives the upright cylinder a stable face to push against |
| Cradle height | 0.01 – 0.13 m above ground | Contacts spheres below their centre line and the cylinder low down, so it won't tip |
| Cradle friction | low (μ = 0.2) | Objects slide into the V centre instead of being dragged sideways |
| Lidar | 360°, 0.12–8 m, scan plane 0.15 m | Detects bay walls and the opening |
| Camera | 640×480, tilted 20° down | Detects the coloured floor strip and objects |
| Footprint | **0.447 m × 0.414 m** | Inside the 0.5 m × 0.5 m spawn limit |

Total mass is 5.55 kg. All inertias are computed from the solid-body formulas (comments in the URDF).

## 2. Package layout

```
pde4430_bay_robot/
├── urdf/bay_robot.urdf          robot model + Gazebo plugins and sensors
├── launch/display.launch.py     view the model in RViz
├── launch/sim.launch.py         Gazebo + spawn + ROS<->Gazebo bridge
├── config/test_world.sdf        small test arena (wall, floor strip, ball)
├── scripts/check_urdf.py        offline URDF checks (tree, inertia, footprint)
├── scripts/urdf_preview.html    browser 3D viewer for the URDF
├── scripts/docker_sim_test.sh   headless end-to-end simulation test
└── pde4430_bay_robot/           Python nodes
```

## 3. Requirements

- Ubuntu 24.04, **ROS 2 Jazzy**, **Gazebo Harmonic**
- `ros-jazzy-ros-gz`, `ros-jazzy-joint-state-publisher-gui`, `ros-jazzy-teleop-twist-keyboard`

## 4. Build and run

```bash
mkdir -p ~/pde4430_ws/src && cd ~/pde4430_ws/src
git clone <this-repo-url> pde4430_bay_robot
cd ~/pde4430_ws && colcon build --symlink-install
source install/setup.bash
```

**View the model in RViz**
```bash
ros2 launch pde4430_bay_robot display.launch.py
```

**Simulate in the assessment world**
```bash
ros2 launch pde4430_bay_robot sim.launch.py world:=/path/to/assessment_bay.sdf x:=0 y:=0 yaw:=0
```

**Drive it** (in a second terminal)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 5. Architecture (nodes, topics, messages)

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo | velocity commands to the diff-drive |
| `/odom` | `nav_msgs/Odometry` | Gazebo → ROS | wheel odometry |
| `/tf` | `tf2_msgs/TFMessage` | Gazebo → ROS | `odom → base_footprint` |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo → ROS | wheel angles, for robot_state_publisher |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo → ROS | lidar |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo → ROS | camera |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS | simulation time |

Third-party nodes used:
- `robot_state_publisher`: publishes TF for the fixed links from the URDF
- `ros_gz_sim create`: spawns the URDF into Gazebo
- `ros_gz_bridge parameter_bridge`: converts the Gazebo topics above to and from ROS 2
- `teleop_twist_keyboard`: keyboard driving

*(rqt_graph and the custom nodes will be added here as they are written.)*

## 6. Testing

**Offline URDF check** (any OS with Python and numpy):
```bash
python scripts/check_urdf.py
```

**Full headless simulation test in Docker** (build, parse, spawn, drive, push a ball, sensors):
```bash
docker run --rm -v <path-to>/pde4430_ws:/ws osrf/ros:jazzy-desktop-full \
  bash /ws/src/pde4430_bay_robot/scripts/docker_sim_test.sh
```
Latest result: all checks pass. The robot drives 0.58 m in 3 s at 0.2 m/s, turns 82° in 3 s at 0.5 rad/s, and pushes a 0.12 m ball from x = 0.60 to x = 0.93 with no sideways drift. The lidar and camera both publish.

> The test reads the ball's position from Gazebo **only to verify** the push. The robot itself never uses Gazebo's object positions, as the coursework rules require.

## 7. Running without Ubuntu (browser desktop in Docker)

`docker/` builds a ROS 2 Jazzy + Gazebo Harmonic desktop that runs in Docker and opens in a web browser through noVNC. It works on Windows or macOS with Docker Desktop.

```bash
docker build -t bay_desktop docker/
docker run -d --name bay_desktop -p 127.0.0.1:6080:6080 --shm-size=2g \
  -v <path-to>/pde4430_ws:/ws bay_desktop
```
Then open `http://localhost:6080/vnc.html?autoconnect=true&resize=remote`. The container builds the workspace on start. Its terminals have these shortcuts:

| Command | Does |
|---|---|
| `sim` | Gazebo + robot in `config/test_world.sdf` (add `world:=...` for another world) |
| `teleop` | keyboard driving (`teleop_twist_keyboard`) |
| `model` | RViz model viewer |
| `rebuild` | rebuild after editing files on the host |

The container has no GPU, so rendering is software-only (Mesa) and slower than native Ubuntu. Native Ubuntu is still the target for the final demo. The VNC port is bound to `127.0.0.1` only.
