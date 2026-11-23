#!/usr/bin/env bash
# start_desktop.sh - container entrypoint: starts a VNC desktop, builds the workspace,
# adds shortcut commands, and serves the desktop to the browser via noVNC on port 6080.

# ---- virtual display (VNC only reachable inside the container) ----
export DISPLAY=:1
Xvnc :1 -geometry 1600x900 -depth 24 -SecurityTypes None -localhost -rfbport 5901 -AlwaysShared &
sleep 2
dbus-launch --exit-with-session startxfce4 > /tmp/xfce.log 2>&1 &

# ---- build the mounted workspace into container-local folders ----
# (keeps build output off the Windows-mounted drive, which is slow and lacks symlinks)
source /opt/ros/jazzy/setup.bash
mkdir -p /root/ws
colcon --log-base /root/ws/log build --base-paths /ws/src \
    --build-base /root/ws/build --install-base /root/ws/install > /tmp/build.log 2>&1 \
    && echo "workspace built" || echo "BUILD FAILED - see /tmp/build.log"

# ---- shell setup and shortcut commands for every terminal ----
cat >> /root/.bashrc <<'EOF'
source /opt/ros/jazzy/setup.bash
source /root/ws/install/setup.bash 2>/dev/null
export LIBGL_ALWAYS_SOFTWARE=1      # no GPU in Docker: Mesa software rendering
TEST_WORLD=/root/ws/install/pde4430_bay_robot/share/pde4430_bay_robot/config/test_world.sdf

# rebuild after editing files on Windows
alias rebuild='(cd /root && colcon --log-base /root/ws/log build --base-paths /ws/src --build-base /root/ws/build --install-base /root/ws/install) && source /root/ws/install/setup.bash'
# Gazebo + robot in the test arena (pass world:=... to use another world)
alias sim='ros2 launch pde4430_bay_robot sim.launch.py world:=$TEST_WORLD'
# RViz model viewer only
alias model='ros2 launch pde4430_bay_robot display.launch.py'
# keyboard driving: i=forward  j/l=turn  k=stop  z=slower
alias teleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard'

echo "Commands:  sim | teleop | model | rviz2 | rqt_graph | rebuild"
EOF

# ---- open a terminal on the desktop to start in ----
sleep 3
xfce4-terminal --geometry=110x30 &

# ---- serve the desktop to the browser (foreground keeps container alive) ----
exec websockify --web /usr/share/novnc 6080 localhost:5901
