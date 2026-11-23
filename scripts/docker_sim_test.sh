#!/usr/bin/env bash
# docker_sim_test.sh - end-to-end test of the bay robot in the assessment_bay world.
#
# Runs in its OWN throwaway container from the bay_desktop image, so it never clashes
# with a simulation you have open in the browser desktop:
#   docker run --rm --network none --shm-size=2g -v <path>/pde4430_ws:/ws --entrypoint bash bay_desktop \
#       /ws/src/pde4430_bay_robot/scripts/docker_sim_test.sh
# (a virtual X display is started below for the LiDAR/camera rendering)
#
# Steps: rebuild -> URDF checks -> launch world + robot (Gazebo server only)
#        -> spawn/footprint/arm/sensor checks -> delivery trials for all three objects.
# note: no "set -u" - the ROS setup scripts reference unset variables
source /opt/ros/jazzy/setup.bash
export DISPLAY=:9 LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=offscreen
# Isolate this test's ROS 2 and Gazebo traffic: containers on the same Docker network
# otherwise discover each other and two simulations fight over /clock.
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=bay_trials GZ_IP=127.0.0.1
PKG=/ws/src/pde4430_bay_robot
# private virtual display for sensor rendering
rm -f /tmp/.X9-lock /tmp/.X11-unix/X9
Xvnc :9 -geometry 1024x768 -depth 24 -SecurityTypes None -localhost > /tmp/xvnc.log 2>&1 &
sleep 2

# stop anything left from an earlier run
pkill -f "gz sim" ; pkill -f parameter_bridge ; pkill -f arm_controller ; pkill -f robot_state_publisher
sleep 2

# ---------- 1. build + URDF checks ----------
cd /root && colcon --log-base /root/ws/log build --base-paths /ws/src \
    --build-base /root/ws/build --install-base /root/ws/install > /tmp/build.log 2>&1 \
    && echo "[PASS] colcon build" || { echo "[FAIL] colcon build"; tail -20 /tmp/build.log; exit 1; }
source /root/ws/install/setup.bash
python3 $PKG/scripts/check_urdf.py | grep -E "footprint|mass|PASSED|FAIL|  -"
check_urdf $PKG/urdf/bay_robot.urdf > /dev/null 2>&1 && echo "[PASS] check_urdf (urdfdom)" || echo "[FAIL] check_urdf"

# ---------- 2. launch world + robot ----------
ros2 launch pde4430_bay_robot sim.launch.py headless:=true > /tmp/sim.log 2>&1 &
echo "[INFO] waiting for world, objects and robot..."
for i in $(seq 1 150); do
  L=$(gz model --list 2>/dev/null)
  echo "$L" | grep -q large_sphere && echo "$L" | grep -q small_sphere && echo "$L" | grep -q cylinder \
    && echo "$L" | grep -q bay_robot && break
  sleep 2
done
L=$(gz model --list 2>/dev/null)
for m in bay_robot large_sphere small_sphere cylinder; do
  echo "$L" | grep -q "$m" && echo "[PASS] $m present" || { echo "[FAIL] $m missing"; grep -iE "error|died" /tmp/sim.log | head; exit 1; }
done

# ---------- 3. spawn pose, arms, sensors ----------
python3 - <<'EOF'
import math, re, subprocess, time, rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan, Image
rclpy.init(); n = Node('checks'); s = {}
n.create_subscription(JointState, '/joint_states', lambda m: s.__setitem__('js', m), 10)
n.create_subscription(LaserScan, '/scan', lambda m: s.__setitem__('scan', m), 10)
n.create_subscription(Image, '/camera/image_raw', lambda m: s.__setitem__('img', m), 10)
out = subprocess.run(['gz','model','-m','bay_robot','-p'], capture_output=True, text=True).stdout
x, y, z = [[float(v) for v in g.split()] for g in re.findall(r'\[([-\d.e ]+)\]', out)
           if len(g.split()) == 3][0]
ok = abs(x) < 0.05 and abs(y + 2.6) < 0.05
print(f"[{'PASS' if ok else 'FAIL'}] robot on start pad: ({x:+.3f}, {y:+.3f})")
end = time.time() + 15
while time.time() < end and not all(k in s for k in ('js', 'scan', 'img')):
    rclpy.spin_once(n, timeout_sec=0.1)
end = time.time() + 6          # give the arms time to deploy
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
if 'js' in s:
    q = dict(zip(s['js'].name, s['js'].position))
    l, r = q.get('left_arm_joint', -1), q.get('right_arm_joint', -1)
    print(f"[{'PASS' if abs(l-2.094)<0.1 and abs(r-2.094)<0.1 else 'FAIL'}] arms deployed to 'wide': "
          f"left {math.degrees(l):.0f} deg, right {math.degrees(r):.0f} deg (target 120)")
else: print('[FAIL] no /joint_states')
if 'scan' in s:
    hits = [v for v in s['scan'].ranges if math.isfinite(v)]
    print(f"[PASS] /scan: {len(hits)}/{len(s['scan'].ranges)} beams hit, nearest {min(hits):.2f} m")
else: print('[FAIL] no /scan')
print(f"[{'PASS' if 'img' in s else 'FAIL'}] camera " + (f"{s['img'].width}x{s['img'].height}" if 'img' in s else 'no image'))
EOF

# ---------- 4. delivery trials ----------
echo "[INFO] delivery trials (ground truth used by the TEST only)"
python3 $PKG/scripts/bay_trials.py ${TRIALS:-}

echo "---- gazebo errors ----"
grep -iE "\[err|error" /tmp/sim.log | grep -viE "nvidia|libEGL|dri|glx" | head -10
