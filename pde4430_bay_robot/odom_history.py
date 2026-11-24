"""odom_history.py - helpers the detectors share to place detections in the odom frame.

Why not plain TF? The detectors spend most of their time in Python fitting code. A
Python TF listener then cannot keep up with /tf (~80 messages/s: odometry plus the
robot's moving joints), its queue stays full and every lookup comes out ~2 s stale.
So instead:

  * MountLookup reads the sensor's fixed mounting on the robot from TF once (fixed
    joints come on the latched /tf_static) and then stops listening.
  * OdomHistory keeps the last few seconds of /odom (30 Hz, cheap to handle) and
    gives the robot's pose at the exact time a scan or image was taken.
"""
import bisect
import math
from collections import deque


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b):
    """Planar pose composition a (+) b, poses as (x, y, yaw)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class OdomHistory:
    """Recent /odom poses, interpolated to any time stamp."""

    def __init__(self, node, callback_group=None, seconds=3.0, max_extrapolate=0.25):
        from nav_msgs.msg import Odometry
        self.times, self.poses = deque(), deque()
        self.seconds = seconds
        self.max_extrapolate = max_extrapolate
        node.create_subscription(Odometry, '/odom', self.on_odom, 50,
                                 callback_group=callback_group)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t = stamp_to_sec(msg.header.stamp)
        if self.times and t <= self.times[-1]:
            if t < self.times[-1] - 1.0:          # simulation restarted: start over
                self.times.clear()
                self.poses.clear()
            else:
                return
        self.times.append(t)
        self.poses.append((p.x, p.y, 2 * math.atan2(q.z, q.w)))
        while self.times and self.times[0] < t - self.seconds:
            self.times.popleft()
            self.poses.popleft()

    def pose_at(self, t):
        """Robot (base_footprint) pose in odom at time t, or None if not known."""
        times, poses = list(self.times), list(self.poses)
        if not times or t < times[0]:
            return None
        if t >= times[-1]:
            return poses[-1] if t - times[-1] <= self.max_extrapolate else None
        i = bisect.bisect_right(times, t)
        t0, t1 = times[i - 1], times[i]
        a, b = poses[i - 1], poses[i]
        f = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]),
                wrap(a[2] + f * wrap(b[2] - a[2])))


class MountLookup:
    """A sensor's fixed pose on the robot, read from TF once."""

    def __init__(self, node, target='base_footprint'):
        from tf2_ros import Buffer, TransformListener
        self.node, self.target = node, target
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        self.transform = None

    def get(self, frame):
        """geometry_msgs/Transform of `frame` in the target frame, or None (not yet)."""
        if self.transform is None:
            from rclpy.time import Time
            from tf2_ros import TransformException
            try:
                self.transform = self.buffer.lookup_transform(self.target, frame,
                                                              Time()).transform
            except TransformException as e:
                self.node.get_logger().warn(f'Waiting for the {frame} mounting: {e}',
                                            throttle_duration_sec=5.0)
                return None
            self.listener.unregister()             # fixed: no need to keep listening
            self.listener = None
        return self.transform
