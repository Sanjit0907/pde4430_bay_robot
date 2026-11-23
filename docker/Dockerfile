# Dockerfile - browser-accessible ROS 2 Jazzy + Gazebo Harmonic desktop for developing
# the bay robot on a machine without Ubuntu (e.g. Windows with Docker Desktop).
#
# Build:  docker build -t bay_desktop docker/
# Run:    docker run -d --name bay_desktop -p 127.0.0.1:6080:6080 --shm-size=2g \
#             -v <path>/pde4430_ws:/ws bay_desktop
# Open:   http://localhost:6080/vnc.html?autoconnect=true&resize=remote
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Light XFCE desktop, TigerVNC server, noVNC web client, teleop and GL test tools
RUN apt-get update && apt-get install -y --no-install-recommends \
        xfce4 xfce4-terminal dbus-x11 \
        tigervnc-standalone-server novnc python3-websockify \
        ros-jazzy-teleop-twist-keyboard \
        mesa-utils \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

COPY start_desktop.sh /start_desktop.sh
RUN chmod +x /start_desktop.sh

EXPOSE 6080
CMD ["/start_desktop.sh"]
