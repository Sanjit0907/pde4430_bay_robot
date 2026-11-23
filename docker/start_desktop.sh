#!/usr/bin/env bash
# start_desktop.sh - container entrypoint: starts a VNC desktop, builds the workspace,
# adds shortcut commands, and serves the desktop to the browser via noVNC on port 6080.

# ---- virtual display (VNC only reachable inside the container) ----
export DISPLAY=:1
# Remove lock files left by a previous run; otherwise after "docker stop/start"
# Xvnc exits with "Server is already active for display 1" and noVNC cannot connect.
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
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
# (only added once, so restarting the container does not duplicate them)
grep -q '# bay_robot shortcuts' /root/.bashrc || cat >> /root/.bashrc <<'EOF'
# bay_robot shortcuts
source /opt/ros/jazzy/setup.bash
source /root/ws/install/setup.bash 2>/dev/null
export LIBGL_ALWAYS_SOFTWARE=1      # no GPU in Docker: Mesa software rendering

# rebuild after editing files on Windows
alias rebuild='(cd /root && colcon --log-base /root/ws/log build --base-paths /ws/src --build-base /root/ws/build --install-base /root/ws/install) && source /root/ws/install/setup.bash'
# assessment_bay world + robot on the start pad (add layout:=random etc.)
alias sim='ros2 launch pde4430_bay_robot sim.launch.py'
# RViz model viewer only
alias model='ros2 launch pde4430_bay_robot display.launch.py'
# keyboard driving: i=forward  j/l=turn  k=stop  z=slower
alias teleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard'
# jaw modes: arms fold | wide | large | straight | close | grip
arms() { ros2 topic pub --once /arm_mode std_msgs/msg/String "{data: $1}" > /dev/null && echo "arms -> $1"; }
# lift: lift down | carry | up | <metres>
lift() { ros2 topic pub --once /lift_mode std_msgs/msg/String "{data: '$1'}" > /dev/null && echo "lift -> $1"; }
# put the three objects back (fixed layout) between attempts
alias reset_objects='ros2 launch assessment_bay spawn_objects.launch.py'
# ground-truth checker (testing only): which objects are in the bay
alias baycheck='ros2 launch assessment_bay bay_checker.launch.py'

echo "Commands:  sim | teleop | arms <fold|wide|large|straight|close|grip> | lift <down|carry|up> | reset_objects | baycheck | model | rebuild"
EOF

# ---- open a terminal on the desktop to start in ----
sleep 3
xfce4-terminal --geometry=110x30 &

# ---- serve the desktop to the browser (foreground keeps container alive) ----
exec websockify --web /usr/share/novnc 6080 localhost:5901
