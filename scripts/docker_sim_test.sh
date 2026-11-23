#!/usr/bin/env bash
# docker_sim_test.sh - headless end-to-end test of the bay robot in ROS 2 Jazzy + Gazebo Harmonic.
#
# Run from Windows/macOS/Linux with Docker (workspace mounted at /ws):
#   docker run --rm -v <path>/pde4430_ws:/ws osrf/ros:jazzy-desktop-full bash /ws/src/pde4430_bay_robot/scripts/docker_sim_test.sh
#
# Steps: build package -> check_urdf -> URDF->SDF conversion -> spawn in headless Gazebo
#        -> drive forward and turn via /cmd_vel -> confirm /odom changes -> confirm /scan and camera publish.
# note: no "set -u" - the ROS setup scripts reference unset variables
source /opt/ros/jazzy/setup.bash
export QT_QPA_PLATFORM=offscreen
PASS=0; FAIL=0
ok()   { echo "[PASS] $*"; PASS=$((PASS+1)); }
bad()  { echo "[FAIL] $*"; FAIL=$((FAIL+1)); }

# ---------- 1. build ----------
cd /ws
rm -rf build install log
if colcon build --packages-select pde4430_bay_robot > /tmp/build.log 2>&1; then ok "colcon build"; else bad "colcon build"; cat /tmp/build.log; fi
source /ws/install/setup.bash
URDF=$(ros2 pkg prefix pde4430_bay_robot)/share/pde4430_bay_robot/urdf/bay_robot.urdf

# ---------- 2. URDF parser (urdfdom) ----------
if command -v check_urdf >/dev/null; then
  if check_urdf "$URDF" > /tmp/check.log 2>&1; then ok "check_urdf parses the model"; sed -n '1,30p' /tmp/check.log
  else bad "check_urdf"; cat /tmp/check.log; fi
fi

# ---------- 3. URDF -> SDF conversion (what Gazebo actually loads) ----------
if gz sdf -p "$URDF" > /tmp/robot.sdf 2>/tmp/sdf.err; then
  ok "gz sdf converts URDF to SDF"
  grep -q 'gz-sim-diff-drive-system' /tmp/robot.sdf && ok "diff-drive plugin present in SDF" || bad "diff-drive plugin missing"
  grep -q 'gpu_lidar' /tmp/robot.sdf && ok "lidar sensor present in SDF" || bad "lidar missing"
else bad "gz sdf conversion"; cat /tmp/sdf.err; fi

# ---------- 4. headless Gazebo + spawn + bridge ----------
export LIBGL_ALWAYS_SOFTWARE=1   # no GPU in Docker: use Mesa software rendering for lidar/camera
WORLD=${WORLD:-$(ros2 pkg prefix pde4430_bay_robot)/share/pde4430_bay_robot/config/test_world.sdf}
gz sim -s -r --headless-rendering "$WORLD" > /tmp/gz.log 2>&1 &
sleep 10
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat "$URDF")" -p use_sim_time:=true > /tmp/rsp.log 2>&1 &
ros2 run ros_gz_sim create -file "$URDF" -name bay_robot -z 0.01 > /tmp/spawn.log 2>&1 \
  && ok "robot spawned in Gazebo" || { bad "spawn"; cat /tmp/spawn.log; }
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image \
  /joint_states@sensor_msgs/msg/JointState[gz.msgs.Model > /tmp/bridge.log 2>&1 &
sleep 6

# Python helper: read odom, drive, read again, report scan/camera
python3 - <<'EOF'
import math, time, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Image

rclpy.init()
n = Node('sim_test')
state = {'odom': None, 'scan': None, 'img': None}
n.create_subscription(Odometry, '/odom', lambda m: state.__setitem__('odom', m), 10)
n.create_subscription(LaserScan, '/scan', lambda m: state.__setitem__('scan', m), 10)
n.create_subscription(Image, '/camera/image_raw', lambda m: state.__setitem__('img', m), 10)
pub = n.create_publisher(Twist, '/cmd_vel', 10)

def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)

def pose():
    p = state['odom'].pose.pose
    yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
    return p.position.x, p.position.y, yaw

def drive(lin, ang, sec):
    t = Twist(); t.linear.x = lin; t.angular.z = ang
    end = time.time() + sec
    while time.time() < end:
        pub.publish(t); rclpy.spin_once(n, timeout_sec=0.05)
    pub.publish(Twist()); spin(1.0)

import subprocess
def ball_pose():
    """Test-only: ask Gazebo where the test ball is (not used by the robot itself)."""
    out = subprocess.run(['gz', 'model', '-m', 'ball', '-p'], capture_output=True, text=True).stdout
    lines = [l.strip() for l in out.splitlines() if l.strip().startswith('[')]
    return lines[0] if lines else out.strip()[-200:]

spin(8)
print(f'[INFO] ball xyz before push: {ball_pose()}')
if state['odom'] is None:
    print('[FAIL] no /odom received'); raise SystemExit
x0, y0, th0 = pose()
print(f'[INFO] start pose x={x0:.3f} y={y0:.3f} yaw={math.degrees(th0):.1f}')
drive(0.2, 0.0, 3.0)
x1, y1, th1 = pose()
print(f'[INFO] after 3 s at 0.2 m/s: x={x1:.3f} y={y1:.3f} yaw={math.degrees(th1):.1f}  (ideal ~0.6 m)')
print(('[PASS]' if x1 - x0 > 0.3 and abs(y1 - y0) < 0.05 else '[FAIL]') + ' drives straight forward on /cmd_vel')
print(f'[INFO] ball xyz after push:  {ball_pose()}  (pushed if x > 0.6 and y ~ 0)')
drive(0.0, 0.5, 3.0)
x2, y2, th2 = pose()
d = math.degrees(th2 - th1)
print(f'[INFO] after 3 s at 0.5 rad/s: yaw change {d:.1f} deg (ideal ~86)')
print(('[PASS]' if d > 40 else '[FAIL]') + ' turns on the spot on /cmd_vel')
z = state['odom'].pose.pose.position.z
print(f'[INFO] odom z = {z:.3f} (should stay ~0, robot not tipping)')

s = state['scan']
if s:
    finite = [r for r in s.ranges if math.isfinite(r)]
    print(f'[PASS] /scan publishing: {len(s.ranges)} beams, {len(finite)} hits (empty world -> few hits expected)')
else:
    print('[WARN] no /scan (gpu_lidar needs GPU/EGL rendering; may not work headless in Docker)')
i = state['img']
if i:
    print(f'[PASS] camera publishing: {i.width}x{i.height} {i.encoding}')
else:
    print('[WARN] no camera image (needs rendering; may not work headless in Docker)')
n.destroy_node(); rclpy.shutdown()
EOF

echo "---- gazebo log (errors only) ----"
grep -iE 'error|exception' /tmp/gz.log | head -20
echo "---- summary: $PASS shell checks passed, $FAIL failed (see python lines above) ----"
