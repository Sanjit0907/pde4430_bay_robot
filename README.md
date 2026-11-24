# PDE4430 Bay Robot

Coursework for **PDE4430 Mobile Robotics** (Middlesex University Dubai, 2026–27).

A differential-drive robot with a low front bumper and a **fold-out gripper on a lift**: two motorised jaws with rubber grip pads, a **scoop tray** (ramp) and two **hook fingers**. It **picks up all three objects**: the small sphere and the cylinder in the jaw pads, the large sphere on the tray, locked there by the fingers. It delivers the large sphere, the small sphere and the upright cylinder of the [`assessment_bay`](https://github.com/sameerkishore/ros2_assessment_world) world through the bay opening, using **physical contact only**. No joints, plugins or services attach objects or set their pose.

> Status: robot design, gripper (jaws, scoop tray, hook fingers) and lift, Gazebo integration, gripper control, scripted delivery trials (all three objects picked up and delivered) and the teleoperation GUI done. Autonomy is in progress on the `autonomy` branch: bay perception (LiDAR + camera) done, mission control not started (section 8).

---

## 1. The task, in numbers

From the `assessment_bay` package:

| Thing | Size | Consequence for the design |
|---|---|---|
| Spawn footprint | ≤ 0.5 × 0.5 m | Anything wider must fold out after spawning |
| Large sphere | Ø 0.60 m, 1.0 kg, μ 0.8 | Wider than the robot. A plain V cannot hold it, and it rolls off sideways when turning |
| Small sphere | Ø 0.20 m, 0.3 kg | Rolls freely and keeps rolling after a push |
| Cylinder | Ø 0.20 × 0.40 m, CoM 0.20 m, μ 0.5 | Slides if pushed low, **tips if pushed above ~0.20 m** |
| Bay opening | 1.0 m wide, faces south at y = 1.6 m | The large sphere must be lined up to within ±0.2 m |
| Start pad | (0, −2.6), facing +y | Spawn pose |

## 2. Approach and robot design

![Robot top view](docs/robot_modes.svg)

| Part | Choice | Why |
|---|---|---|
| Base | Differential drive, 0.24 × 0.26 m chassis, wheels r 0.05 m on a 0.30 m track. Front caster right under the bumper (0.085 m ahead of the wheels), rear caster, **2 kg battery ballast** at the back | Turns on the spot for small, precise corrections. With the tray and fingers out and the 1 kg sphere held 0.44 m ahead, the centre of mass moves forward: the ballast and the forward caster keep the robot on its wheels (without them it tipped 8.7° onto the tray tip as soon as the lift rose) |
| Bumper | Plate 0.34 m wide, **0.02–0.24 m** above the floor | Pushes the cylinder with its contact centroid at ~0.13 m, below its 0.20 m CoM, so it slides instead of tipping. Meets the large sphere just under its equator |
| **Gripper jaws** | Two 0.44 m plates, 0.02–0.30 m high, pivoting about vertical axes at the bumper corners, with a 0.40 m post at each tip. Gazebo joint position controllers in force mode (P 60, capped at 5 N·m, about 25 N squeeze at the pads) | Folded back along the sides at spawn. Swing out after spawn to guide, cage or grip each object (modes below) |
| **Scoop tray (ramp)** | 0.36 × 0.30 m tray with a 0.10 m bevelled tip, hinged at the foot of the bumper, on the lift. Folded up at spawn, flat when deployed: top 16 mm above the floor, tip 4 mm, tip underside 1.5 mm, a 7° slope. Reaches 0.59 m ahead | The large sphere sits on it, so the lift can raise it. Low and gentle, so a sphere pushed against the tip rolls up onto it easily. Folded, a rubber pad on its underside takes over from the bumper pad it covers |
| **Hook fingers** | A 0.35 m finger hinged at each jaw tip, 0.25–0.40 m high (the large sphere's equator). Three positions: `fold` (along the jaw), `open` (straight on, reach 0.79 m), `hook` (90° to the jaw, inwards). Force mode, 3 N·m, slippery (μ 0.2) | The large sphere is wider than the jaw pivots, so the jaws alone can only touch it behind its middle. Swinging from `open` to `hook`, the fingers sweep round its front and push it up onto the tray and back against the bumper. Hooked, they lock it there (section below) |
| **Grip pads** | Black rubber pads (μ 1.5), 0.12 × 0.15 m, 0.05–0.20 m above the floor, mounted **turned 27°** on the jaws, plus a rubber pad on the bumper face | When the jaws close on a 0.2 m object (they stop at about 27° inward), the two pad faces are **parallel** and press straight across its middle, like a parallel-jaw gripper, so a sphere cannot roll out. Separate links on preserved fixed joints, so they grip while the rest of each jaw stays slippery for guiding |
| **Lift** | Carriage on two steel rails, prismatic joint 0–0.15 m, PID force control (capped at 70 N, ~2.5× the heaviest load) | Raises the jaws, fingers and tray together, so whatever they hold is **picked up** and carried clear of the floor, then set down in the bay |
| Friction | Wheels μ 1.0, casters 0, bumper 0.3, jaws 0.1, fingers 0.2, tray 0.4, pads 1.5 | Objects slide along the jaws and fingers into the centre and onto the tray, and the pads hold the small objects when squeezed |
| LiDAR | 360°, 0.12–8 m, on a mast, scan plane **0.35 m** | Above the arms and bumper so they don't block it. Below the 0.5 m bay walls, and sees the 0.4 m cylinder and the large sphere |
| Camera | 640×480, on a pole at **0.72 m**, tilted 31° down | Sees the jaws and, over the top of the 0.6 m sphere, the yellow strip and objects ahead |
| Motion limits | 0.5 m/s, 0.5 m/s², 1.5 rad/s | Gentle acceleration so pushed spheres aren't flung ahead |

Total mass 9.75 kg. Footprint at spawn 0.481 × 0.475 m (`scripts/check_urdf.py`). Inertias use the solid-body formulas (comments in the URDF).

### Arm modes

Angles are measured outward from straight ahead. They come from the object sizes (see `scripts/check_urdf.py` and the design notes in the URDF):

| Mode | Joint (rad) | Arm angle | Use |
|---|---|---|---|
| `fold` | 0.000 | folded back | Spawn pose, fits 0.5 × 0.5 m |
| `wide` | 2.094 | 60° out | Funnel for catching any object. Deployed automatically after spawn |
| `large` | 2.531 | 35° out | **V cradle** for the 0.6 m sphere (~1 cm play). Two long arm faces push it from behind its widest point, which keeps it centred while it is pushed straight |
| `straight` | 3.142 | 0° | Arms parallel. Releasing a small object inside the bay without hitting the bay walls |
| `close` | 3.526 | 22° in | Arms meet in front (2 cm gap): a cage for the 0.2 m sphere or cylinder (room for r = 0.117 m) |
| `grip` | 3.840 | 40° in (target) | The pads meet a 0.2 m object at about 27°. The target is deliberately past that, so the motors stay at their 5 N·m limit and the squeeze is the same for every object |

Lift heights (`/lift_mode`): `down` 0 m, `carry` 0.08 m, `up` 0.15 m, or any height in metres.

Scoop tray (`/scoop_mode`): `up` (folded, spawn) or `down` (flat). It is deployed automatically after spawn, together with the jaws.

Fingers (`/finger_mode`), angle to their jaw: `fold` 0° (along the jaw, spawn), `hook` 90° (target 70°, past where they meet the sphere, so they keep pressing with the full 3 N·m), `open` 180° (straight on). Gazebo spawns joints at 0, so 0 has to be the folded position.

**Safety rules in `arm_controller`:**
- The jaws can only open (any mode except `grip`/`close`) while the lift is down, so a lifted object is never dropped by accident.
- Hooked fingers can only open or fold while the lift is down (they hold the lifted sphere).
- The jaws cannot move while the fingers are hooked, and with the fingers open they can only move between `wide`, `large` and `straight` (closer, the long fingers would cross). The fingers can only open or hook with the jaws in one of those modes.
- The scoop tray only moves while the lift is down.
- `fold` folds everything (fingers, tray, jaws) and lowers the lift first. All targets are clamped to the joint limits.

### Delivery strategy

- **Large sphere (scoop and hook):** tray `down`, jaws `large` (the 35° V), fingers `open`. Drive up slowly and **stop with the sphere just short of the tray tip** (centre ~0.66 m ahead of the robot centre). Then:
  1. Fingers `hook`: they sweep round the front of the sphere and push it backwards. The only way it can go is up the tray tip, so it rolls onto the tray and against the bumper. Hooked, the fingers press its front sides, and it is locked between the bumper, the jaws and the fingers, on the tray.
  2. Lift `carry`: it is picked up (+9 cm).
  3. Drive to the bay (turns are fine, it is held). Line up first: with the fingers hooked the robot is ~0.9 m wide, and the opening is 1.0 m.
  4. Lift `down`, fingers `open`: it rolls forward off the tray. Reverse out.
- **Small sphere / cylinder (pick and place):** tray `up` (folded: these are held by the jaw pads, and the tray tip would push them away), fingers `fold`, open `wide` and drive up slowly. **Stop before the bumper touches the object**: a sphere that is bumped rolls away at the robot's speed. Then:
  1. `close`: the jaws swing in around it and centre it.
  2. `grip`: the pads squeeze it.
  3. Lift `carry`: it is picked up off the floor.
  4. Drive into the bay (turns are fine, it is held).
  5. Lift `down`, then `straight` to release, and reverse out.
- **Pushing the large sphere** in the `large` V still works as a fallback: line up with the opening, push straight, stop about 0.7 m short of the strip and let it roll in (the original strategy, design history 1–4).

*Design history (from the trials in section 7):*
1. **Catching from `wide`, then closing to 35°** squeezed the sphere forward out of the V, because the arms touch it behind its widest point. Fix: put the arms at 35° *before* reaching the sphere and drive into it.
2. **Turning while pushing** loses the large sphere whatever the arm shape. It rolls with almost no resistance, so any sideways push gives it speed it never loses, and a pusher cannot pull it back. Holding it through turns would need arms that wrap past its middle, which cannot fold inside the 0.5 m spawn square because the sphere is wider than the robot. **Line up and push straight** (the world's README also says the approach "has to be lined up").
3. **A narrow fork** (arms at 10°, sphere in the notch between the tips) was tried and dropped. In camera snapshots the sphere bounced between the two thin tips and slid off. The V pushes along two long faces and stays centred.
4. **Tip posts:** each arm got a 0.40 m post at its tip, so a sphere that rides along an arm meets a flat face rather than the plate's 0.30 m top edge (exactly its equator).
5. **Picking up the large sphere: a passive ramp does not work.** The first scoop tray (12° tip, 2 mm above the floor) only nudged the sphere, which rolled away: at 0.12 m/s and even at 0.45 m/s it rose 6 mm on the tip and was kicked ahead. A free ball needs only a tiny push to roll, and that is all a moving ramp gives it. Something must hold the ball while it goes onto the ramp, hence the **fingers**: they push it backwards into the tip, so it can only go up.
6. **The fingers stalled the sphere half-way** (0.55 m instead of 0.44 m): climbing a 12° tip onto a 22 mm tray took more push than the fingers could give before their reaction forced the soft jaws open (35° → 47°). Fixes: a lower tray (16 mm) with a 7° tip, slippery fingers (μ 0.2, so the sphere rolls along them; they hold it by position, not friction) and stiffer jaws (P 20 → 60).
7. **The robot tipped onto its nose** (8.7°) whenever the lift rose with the fingers out: the centre of mass had moved ahead of the front caster, and the tray tip had been propping it up. atan(8 cm / 0.52 m) = 8.7°, exactly where the lifted tip meets the floor again. Fixes: 2 kg ballast at the back and the front caster moved under the bumper. Pitch while carrying the large sphere: +0.3°.

## 3. Package layout

```
pde4430_bay_robot/
├── urdf/bay_robot.urdf              robot model, Gazebo plugins and sensors
├── launch/sim.launch.py             assessment_bay world + robot + bridge + arm controller
├── launch/display.launch.py         view the model in RViz (sliders for the arms)
├── pde4430_bay_robot/arm_controller.py   gripper: jaw modes + lift heights -> joint targets
├── pde4430_bay_robot/teleop_gui.py       teleoperation window: drive, speed limit, jaws, lift
├── pde4430_bay_robot/bay_detector.py     LiDAR: finds the bay by fitting its three walls
├── pde4430_bay_robot/strip_detector.py   camera: finds the yellow strip at the opening
├── pde4430_bay_robot/odom_history.py     helpers: sensor mounting + robot pose at a sensor time
├── launch/perception.launch.py           starts both detectors
├── scripts/check_urdf.py            offline URDF checks (tree, inertia, footprint)
├── scripts/bay_trials.py            scripted delivery trials (testing only)
├── scripts/docker_sim_test.sh       full end-to-end test in Docker
├── scripts/urdf_preview.html        browser 3D viewer for the URDF
└── docker/                          browser-based ROS 2 + Gazebo desktop
```

## 4. Requirements

- Ubuntu 24.04, **ROS 2 Jazzy**, **Gazebo Harmonic**
- `ros-jazzy-ros-gz`, `ros-jazzy-joint-state-publisher-gui`, `ros-jazzy-teleop-twist-keyboard`
- The [`assessment_bay`](https://github.com/sameerkishore/ros2_assessment_world) package in the same workspace

## 5. Build and run

```bash
mkdir -p ~/pde4430_ws/src && cd ~/pde4430_ws/src
git clone https://github.com/sameerkishore/ros2_assessment_world.git
git clone <this-repo-url> pde4430_bay_robot
cd ~/pde4430_ws && colcon build --symlink-install
source install/setup.bash
```

**World + robot on the start pad** (the arms open to `wide` about 3 s after spawning):
```bash
ros2 launch pde4430_bay_robot sim.launch.py
ros2 launch pde4430_bay_robot sim.launch.py layout:=random seed:=42   # other object layouts
ros2 launch pde4430_bay_robot sim.launch.py render_engine:=ogre2 rviz:=true
```

**Teleoperation GUI** (second terminal):
```bash
ros2 run pde4430_bay_robot teleop_gui
```

![Teleop GUI](docs/teleop_gui.png)

- **Drive:** hold a direction button, or hold W/A/S/D or the arrow keys (two keys together for a curve). Releasing stops the robot; Space or **Stop** stops it at once. The robot also stops if the window loses focus.
- **Speed limit:** sliders for drive speed (up to 0.5 m/s) and turn rate (up to 1.5 rad/s), the robot's own limits.
- **Jaws / Lift:** one button per mode, plus a height slider for the lift. The highlighted button is the state `arm_controller` reports, so a refused request (e.g. opening the jaws while lifted) shows up straight away.

The keyboard teleop also still works: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`.

**Arms from the command line** (third terminal):
```bash
ros2 topic pub --once /arm_mode std_msgs/msg/String "{data: grip}"    # fold|wide|large|straight|close|grip
ros2 topic pub --once /lift_mode std_msgs/msg/String "{data: carry}"  # down|carry|up or metres
ros2 topic pub --once /scoop_mode std_msgs/msg/String "{data: down}"  # up|down
ros2 topic pub --once /finger_mode std_msgs/msg/String "{data: hook}" # fold|open|hook
```

**Reset objects / check results** (from `assessment_bay`; the checker is for testing only):
```bash
ros2 launch assessment_bay spawn_objects.launch.py
ros2 launch assessment_bay bay_checker.launch.py
```

## 6. Architecture (nodes, topics, messages)

```
                 /arm_mode, /lift_mode
teleop_gui  ------------------------------->  arm_controller
    |       <-------------------------------       |
    |           /arm_state, /lift_state            |  /left_arm/cmd_pos, /right_arm/cmd_pos,
    | /cmd_vel                                     |  /lift/cmd_pos
    v                                              v
ros_gz_bridge  <=========================================>  Gazebo (assessment_bay + bay_robot)
    |
    |  /odom /tf /joint_states /scan /camera/image_raw /clock
    v
robot_state_publisher -> /tf (robot links), RViz
```

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo | drive commands to the diff-drive plugin |
| `/arm_mode` | `std_msgs/String` | `teleop_gui` / user → `arm_controller` | jaw mode: `fold`, `wide`, `large`, `straight`, `close`, `grip` |
| `/lift_mode` | `std_msgs/String` | `teleop_gui` / user → `arm_controller` | lift: `down`, `carry`, `up` or a height in metres |
| `/scoop_mode` | `std_msgs/String` | `teleop_gui` / user → `arm_controller` | scoop tray: `up`, `down` |
| `/finger_mode` | `std_msgs/String` | `teleop_gui` / user → `arm_controller` | fingers: `fold`, `open`, `hook` |
| `/arm_state`, `/lift_state`, `/scoop_state`, `/finger_state` | `std_msgs/String` (latched) | `arm_controller` → `teleop_gui` | current jaw / lift / tray / finger setting |
| `/lift/cmd_pos` | `std_msgs/Float64` | ROS → Gazebo | lift height target (m) |
| `/scoop/cmd_pos` | `std_msgs/Float64` | ROS → Gazebo | scoop tray angle target (rad) |
| `/fingers/cmd_pos` | `std_msgs/Float64` | ROS → Gazebo | finger angle target (rad), both fingers |
| `/left_arm/cmd_pos`, `/right_arm/cmd_pos` | `std_msgs/Float64` | ROS → Gazebo | arm joint targets (rad) |
| `/odom` | `nav_msgs/Odometry` | Gazebo → ROS | wheel odometry |
| `/tf` | `tf2_msgs/TFMessage` | Gazebo → ROS | `odom → base_footprint` |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo → ROS | wheel and arm angles |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo → ROS | LiDAR |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo → ROS | camera |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo → ROS | simulation time |
| `/bay/opening` | `geometry_msgs/PoseStamped` | `bay_detector` → | centre of the bay opening, x axis into the bay (odom) |
| `/bay/markers` | `visualization_msgs/MarkerArray` | `bay_detector` → RViz | fitted bay walls |
| `/bay_strip/pose` | `geometry_msgs/PoseStamped` | `strip_detector` → | centre of the visible strip, x axis into the bay (base_footprint) |
| `/bay_strip/length` | `std_msgs/Float32` | `strip_detector` → | visible strip length (~1.0 m = whole strip in view) |
| `/bay_strip/debug` | `sensor_msgs/Image` | `strip_detector` → | camera image with the detection drawn on it |

**Own nodes:**

`teleop_gui` is the operator window (tkinter). It publishes `/cmd_vel` at 10 Hz while a drive button or key is held, scaled by the speed sliders, then sends a few zero commands and goes quiet, so it does not fight other nodes driving the robot. Jaw and lift buttons send requests on `/arm_mode` and `/lift_mode`; the window shows the state `arm_controller` reports back.

`bay_detector` and `strip_detector` find the bay with the robot's own sensors (section 8).

`arm_controller` is the gripper controller. It maps jaw modes, lift heights, tray and finger settings to joint targets. It checks names against fixed lists, clamps targets to the joint limits, and applies the safety rules above (e.g. it refuses to open the jaws or unhook the fingers while an object is lifted). It re-sends targets at 5 Hz so none is lost while the bridge starts, and deploys the jaws and the tray automatically after spawning.

**Third-party nodes, and why:**
- `robot_state_publisher` publishes the robot's link transforms from the URDF and `/joint_states`, which RViz and later navigation need.
- `ros_gz_sim create` is the standard way to spawn a URDF into Gazebo Harmonic.
- `ros_gz_bridge parameter_bridge` converts the Gazebo transport topics above to and from ROS 2 messages.
- `teleop_twist_keyboard` gives simple keyboard driving on `/cmd_vel` (an alternative to `teleop_gui`).
- The Gazebo plugins `DiffDrive`, `JointPositionController` (jaws and lift in PID force mode) and `JointStatePublisher` turn velocity commands into wheel motion and drive the gripper.

*(The rqt_graph will be added here.)*

## 7. Testing

**Offline URDF check** (any OS with Python and numpy):
```bash
python scripts/check_urdf.py
```

**End-to-end test in Docker**: builds the workspace, checks the URDF, launches the assessment world and robot, checks spawn pose, arm deployment and sensors, then runs **delivery trials** for all three objects:
```bash
docker run --rm --network none --shm-size=2g -v <path-to>/pde4430_ws:/ws --entrypoint bash bay_desktop \
  /ws/src/pde4430_bay_robot/scripts/docker_sim_test.sh
```

Each trial (`scripts/bay_trials.py`) places the object 1.3 m ahead of the robot, drives up and stops just short of it, and **picks it up** (large sphere: tray and fingers; small objects: jaw pads). It drives an S-bend (about 26° right, then 26° left) and checks the object is still held and raised. It then lines up with the bay, drives in, sets the object down, releases it, reverses out, and judges the result with `assessment_bay`'s own `object_in_bay()` rule. It also runs natively: `python3 scripts/bay_trials.py` with the simulation up.

Latest full run, with the scoop tray and hook fingers (native Ubuntu, headless; **3/3 objects picked up and delivered**):

| Object | Held by | Lift / object raised | Held while carried | Final position | Result |
|---|---|---|---|---|---|
| Large sphere | **scoop tray + hooked fingers** | 7.7 cm / **9.0 cm** | held up through the S-bend, robot pitch +0.3° | (−0.20, 2.49) | in the bay |
| Small sphere | jaw pads (`grip`) | 8.0 cm / **8.0 cm** | held up through the S-bend | (+0.01, 2.46) | in the bay |
| Cylinder | jaw pads (`grip`) | 8.0 cm / **8.1 cm** | held up and upright | (+0.01, 2.21) | in the bay, upright |

The large sphere is held about 5 cm off-centre (the two fingers do not sweep exactly together) and rolls a little sideways when released, so it ended at x = −0.20, exactly the limit for a 0.6 m sphere in the 1.0 m bay. It counts as delivered, but with no margin: releasing it further in, or centring it before the lift (e.g. jaws briefly to `large` again), would give some.

*Gripper tuning found by the trials:*
1. **Squeeze force:** a jaw target only 1° past the contact angle gave almost no squeeze, and the small sphere slipped. The `grip` target is now well past contact (40°), so the jaw motors always push at their effort limit.
2. **Effort limit:** at 12 N·m the cylinder was wedged so hard between the pads and the bumper that the physics solver locked the robot in place. 5 N·m (about 5× what the 0.5 kg cylinder needs) grips without jamming.
3. **Pad angle:** flat pads on the angled jaws pressed a sphere slightly ahead of its centre, so it rolled backwards out of the grip (it rose only 2–5 cm of the 8 cm lift). Turning the pads 27° makes them parallel when gripping: 0.2 cm slip.
4. **Lift control:** a velocity-mode controller could not move the lift under the jaws' weight. A PID force controller, capped at about 3× the lifted weight, rises smoothly and holds height.
5. **Deploy timing:** the jaws now deploy only after the robot's joints appear on `/joint_states`, and targets are re-sent on a steady clock. A command sent before the robot had spawned was lost.
6. **Jaw control:** after a Gazebo Harmonic update (gz-sim vendor package, Nov 2026) the velocity-mode jaw controllers still received their targets but the jaws no longer moved, so they stayed folded after spawning. The jaws now use the same PID force mode as the lift: p 20, d 2, torque capped at 5 N·m (the joint effort limit). In `grip` the target is about 13° past contact, so the controller asks for about 4.6 N·m and the squeeze is nearly the same as before. All three trials pass again with the same lift heights.

Robot camera views during the trials (from `bay_trials.py`, `SNAPSHOTS=<dir>`):

| Large sphere in the V | Cylinder caged |
|---|---|
| ![](docs/camera_large_sphere.png) | ![](docs/camera_cylinder.png) |

> The trials use Gazebo ground truth **only to set up and judge** each test, as the brief allows. The robot's own nodes never use object positions.

## 8. Autonomy (in progress, `autonomy` branch)

The plan: find the bay with the sensors, then find each object, pick it up or push it, and deliver it, using the manual strategy from section 2. The first part, **finding the bay**, is done. The robot uses two independent sensors, as the brief requires (no ground-truth positions).

```bash
ros2 launch pde4430_bay_robot sim.launch.py render_engine:=ogre2
ros2 launch pde4430_bay_robot perception.launch.py
ros2 topic echo /bay/opening          # bay opening in odom
ros2 run rqt_image_view rqt_image_view /bay_strip/debug
```

> **Use `render_engine:=ogre2` for anything that needs the LiDAR.** With the default `ogre` engine, Gazebo's `gpu_lidar` returns its minimum range (0.12 m) on every ray, so the scan is empty. (It is not the LiDAR housing: removing it changes nothing.) The camera works with both.

### LiDAR: bay detector (`bay_detector`)

The bay walls are 0.5 m high, so the 0.35 m scan plane sees them. A simple "find a gap" approach fails: from the start pad the LiDAR mostly sees the back wall through the opening, and from the side it sees a single flat wall. So the detector fits a **model of the bay** (three wall rectangles with the sizes from the brief, not its position) to the scan:

1. Split the scan into straight segments.
2. Pair each segment with each of the bay's six long wall faces. Each pairing gives a candidate bay pose.
3. Score each candidate: scan points on the model's walls count for it. Evidence against it is **rays that pass through a model wall**, and points that **carry on in line past the end of a wall**. That rejects fits to the arena walls, for example the model fitted flush behind an arena corner.
4. Refine the best few candidates by least squares, and accept one only if it is **pinned down**: points on at least two different wall faces, and no slightly shifted or turned fit explaining the scan as well. One flat wall seen side-on fits several bay poses, so the detector reports nothing rather than guess.

The result is converted to the odom frame with the robot's pose **at the scan time**, so the bay's position can be remembered while driving.

### Camera: strip detector (`strip_detector`)

The yellow strip is visual only. The camera finds it by colour, then projects each strip pixel onto the floor using the camera model and its mounting (from TF).

- **The jaws are yellow too** and fill the bottom of every frame. The strip is pure yellow (no blue, saturation ~255), and the jaw paint has some blue (saturation ≤ ~210), so a saturation threshold of 235 separates them.
- **Resting tilt:** the robot sits about 0.5° nose-down on its casters. That moves a floor point 4 m away by ~25 cm, so the projection corrects for it (`pitch_correction_deg`, calibrated against the known strip position).

### Results

Offline, on 40 random robot poses around the arena (objects in place), scored against the known layout:

| | Correct | Wrong | Not found | Accuracy when found |
|---|---|---|---|---|
| LiDAR bay detector | 10 | **0** | 30 | 0.8 cm, 0.2° mean (max 1.4 cm, 0.7°), up to 4.3 m away |
| Camera strip detector | 8 (every frame with the strip in view) | **0** | 32 (strip not in view) | ≤ 4 cm, ≤ 0.6°, 1–4 m away |

Most LiDAR misses are views of one bay wall side-on (refused on purpose) or far views with few points. The LiDAR fit takes ~0.5 s, so the bay updates at about 2 Hz.

Live, driving from the start pad (forward 1.3 m, turning ±25°, then reversing), both detectors place the opening at (4.2, 0) in odom within **1 cm** while the odometry is still good. After turning on the spot both drift by the same 8–10 cm, which is wheel odometry error (about 1° of heading), not detector error. The autonomy will re-detect the bay on the way in.

*Problems found and fixed on the way:*
1. **TF too slow in Python nodes.** Gazebo's `JointStatePublisher` published joint states every physics step (870 Hz). `robot_state_publisher` turned that into hundreds of `/tf` messages per second, and the Python TF listener lagged ~2 s behind the sensors. Detections were converted with an old robot pose: they looked shifted but not rotated. Fixes: joint states at 50 Hz (`<update_rate>` in the URDF), and the detectors read their fixed sensor mounting from TF once and the robot pose from `/odom` (`odom_history.py`).
2. **LiDAR blind with `ogre`** (above).

### Next

- Object detection: the cylinder and large sphere in the LiDAR (round clusters of the right radius), the small sphere and colours in the camera.
- Mission node: a state machine for find object → approach → grip/push → go to the bay (lined up with the opening) → release, re-detecting the bay on the way in.

## 9. Running without Ubuntu (browser desktop in Docker)

`docker/` builds a ROS 2 Jazzy + Gazebo Harmonic desktop that runs in Docker and opens in a web browser through noVNC. It works on Windows or macOS with Docker Desktop.

```bash
docker build -t bay_desktop docker/
docker run -d --name bay_desktop -p 127.0.0.1:6080:6080 --shm-size=2g \
  -v <path-to>/pde4430_ws:/ws bay_desktop
```
Then open `http://localhost:6080/vnc.html?autoconnect=true&resize=remote`. The container builds the workspace (both packages) on start. Its terminals have these shortcuts:

| Command | Does |
|---|---|
| `sim` | assessment_bay world + robot on the start pad |
| `teleop` | keyboard driving |
| `arms <mode>` | `fold`, `wide`, `large`, `straight`, `close`, `grip` |
| `lift <height>` | `down`, `carry`, `up` or metres |
| `reset_objects` | put the three objects back |
| `baycheck` | ground-truth bay checker (testing only) |
| `model` | RViz model viewer |
| `rebuild` | rebuild after editing files on the host |

The container has no GPU, so rendering is software-only (Mesa) and much slower than native Ubuntu. Native Ubuntu is the target for the final demo. The VNC port is bound to `127.0.0.1` only.
