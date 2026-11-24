"""strip_detector.py - finds the yellow strip across the bay opening with the camera.

The strip (1.0 m x 0.1 m, floor level, visual only) marks the bay opening. LiDAR
cannot see it, the camera can.

  1. Threshold pure yellow in HSV. The robot's own jaws are yellow too and fill the
     bottom of the image, but their paint has some blue in it (rgba 0.98 0.78 0.15),
     so their saturation stays below ~210. The strip is pure yellow (blue = 0),
     saturation ~255. A saturation threshold of 235 keeps the strip and drops the
     jaws.
  2. Project every strip pixel onto the floor (z = 0), using the camera model
     (640x480, horizontal FOV from the URDF) and the camera's mounting from TF.
  3. Fit a line to the floor points: centre, direction and visible length of the
     strip, in the robot's frame.

Subscribes
    /camera/image_raw     sensor_msgs/Image (rgb8)
    /odom                 nav_msgs/Odometry  (only used with output_frame odom)
Publishes
    /bay_strip/pose       geometry_msgs/PoseStamped  centre of the visible strip;
                          x axis across the strip, away from the robot (= into the
                          bay). Frame: output_frame.
    /bay_strip/length     std_msgs/Float32   visible length (m). About 1.0 when the
                          whole strip is in view; less if it runs off the image, and
                          then the centre is only the centre of the visible part.
    /bay_strip/debug      sensor_msgs/Image  camera image with the detection drawn
                          on it (only computed while someone subscribes)

Parameters
    output_frame (str, default "base_footprint")   frame for /bay_strip/pose:
                          base_footprint or odom (robot pose from /odom at the image
                          time; see odom_history.py)
    horizontal_fov (float, default 1.3963)         rad, must match the URDF camera
    hue_min, hue_max (int, default 20, 38)         OpenCV hue range (0-180)
    sat_min (int, default 235), val_min (int, default 150)
    min_pixels (int, default 40)                   smaller yellow blobs are ignored
    pitch_correction_deg (float, default 0.5)      the robot rests ~0.5 deg nose-down
                          on its casters. Far floor points are very sensitive to this
                          (0.5 deg moves a point 4 m away by ~20 cm), so the camera
                          mounting is corrected by it. Calibrated against the known
                          strip position at 1-4 m: errors from 25 cm to under 2 cm.
"""
import math

import numpy as np

MAX_WIDTH = 0.3          # m, a floor blob wider than this is not the 0.1 m strip
MIN_LENGTH = 0.15        # m, shorter blobs are too small to trust
MAX_RANGE = 7.0          # m, floor points further away than this are ignored


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yellow_mask(rgb, hue=(20, 38), sat_min=235, val_min=150):
    import cv2
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = ((hsv[..., 0] >= hue[0]) & (hsv[..., 0] <= hue[1]) &
            (hsv[..., 1] >= sat_min) & (hsv[..., 2] >= val_min)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))


def pixels_to_floor(u, v, width, height, hfov, R, t):
    """Project pixels (optical frame) onto the floor z = 0 of the target frame.
    R, t: rotation and position of the camera optical frame in that frame."""
    f = (width / 2) / math.tan(hfov / 2)
    rays = np.c_[(u - width / 2) / f, (v - height / 2) / f, np.ones(len(u))] @ R.T
    down = rays[:, 2] < -1e-6                      # only rays that reach the floor
    s = -t[2] / rays[down, 2]
    pts = t[:2] + rays[down, :2] * s[:, None]
    return pts[np.hypot(pts[:, 0], pts[:, 1]) < MAX_RANGE]


def detect(rgb, hfov, R, t, hue=(20, 38), sat_min=235, val_min=150, min_pixels=40):
    """Find the strip in an RGB image.

    Returns dict(centre, yaw, length, width, pixels, mask) - centre and yaw in the
    target frame of (R, t), yaw pointing across the strip away from the camera - or
    None.
    """
    import cv2
    mask = yellow_mask(rgb, hue, sat_min, val_min)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    best = None
    h, w = mask.shape
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < min_pixels:
            continue
        v, u = np.nonzero(labels == i)
        pts = pixels_to_floor(u.astype(float), v.astype(float), w, h, hfov, R, t)
        if len(pts) < min_pixels // 2:
            continue
        centre = pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts - centre)
        along, across = vt[0], vt[1]
        a = (pts - centre) @ along
        c = (pts - centre) @ across
        length = a.max() - a.min()
        width = np.percentile(c, 98) - np.percentile(c, 2)
        if length < MIN_LENGTH or width > MAX_WIDTH:
            continue
        if best is None or length > best['length']:
            mid = centre + along * (a.max() + a.min()) / 2
            normal = across if across @ (mid - t[:2]) > 0 else -across
            best = dict(centre=mid, yaw=math.atan2(normal[1], normal[0]),
                        length=float(length), width=float(width),
                        pixels=int(stats[i, cv2.CC_STAT_AREA]), label=i)
    if best is not None:
        best['mask'] = (labels == best.pop('label'))
    return best


# ====================================================================== ROS node
def main(args=None):
    import cv2
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32

    from pde4430_bay_robot.odom_history import (MountLookup, OdomHistory, compose,
                                                stamp_to_sec)

    class StripDetector(Node):
        def __init__(self):
            super().__init__('strip_detector')
            self.frame = self.declare_parameter('output_frame', 'base_footprint').value
            if self.frame not in ('odom', 'base_footprint'):
                self.get_logger().warn(f'output_frame "{self.frame}" not supported, '
                                       'using base_footprint')
                self.frame = 'base_footprint'
            self.hfov = self.declare_parameter('horizontal_fov', 1.3963).value
            self.hue = (self.declare_parameter('hue_min', 20).value,
                        self.declare_parameter('hue_max', 38).value)
            self.sat_min = self.declare_parameter('sat_min', 235).value
            self.val_min = self.declare_parameter('val_min', 150).value
            self.min_pixels = self.declare_parameter('min_pixels', 40).value
            self.pitch = math.radians(self.declare_parameter('pitch_correction_deg', 0.5).value)
            self.mount_lookup = MountLookup(self)
            # /odom on its own thread, so it is up to date while an image is processed
            self.odom = OdomHistory(self, MutuallyExclusiveCallbackGroup())
            self.pose_pub = self.create_publisher(PoseStamped, '/bay_strip/pose', 10)
            self.len_pub = self.create_publisher(Float32, '/bay_strip/length', 10)
            self.debug_pub = self.create_publisher(Image, '/bay_strip/debug', 1)
            self.create_subscription(Image, '/camera/image_raw', self.on_image, 1)
            self.mount = None
            self.found = None
            self.get_logger().info('Strip detector ready (camera, yellow floor strip)')

        def camera_mount(self, frame):
            """Camera pose in base_footprint (fixed, so looked up once)."""
            if self.mount is None:
                tr = self.mount_lookup.get(frame)
                if tr is None:
                    return None
                q, p = tr.rotation, tr.translation
                # resting tilt: rotate the mounting about the robot's y axis
                c, s = math.cos(self.pitch), math.sin(self.pitch)
                tilt = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
                self.mount = (tilt @ quat_to_matrix(q.x, q.y, q.z, q.w),
                              tilt @ np.array([p.x, p.y, p.z]))
            return self.mount

        def on_image(self, msg):
            mount = self.camera_mount(msg.header.frame_id)
            if mount is None:
                return
            if msg.encoding != 'rgb8':
                self.get_logger().warn(f'Expected rgb8, got {msg.encoding}',
                                       throttle_duration_sec=10.0)
                return
            rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
            rgb = rgb[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            res = detect(rgb, self.hfov, *mount, self.hue, self.sat_min, self.val_min,
                         self.min_pixels)

            if res is None:
                if self.found is not False:
                    self.get_logger().info('Strip not in view')
                self.found = False
            else:
                if self.found is not True:
                    c = res['centre']
                    self.get_logger().info(
                        f'Strip found: {math.hypot(c[0], c[1]):.2f} m away, '
                        f'{res["length"]:.2f} m of it visible')
                self.found = True
                p = (float(res['centre'][0]), float(res['centre'][1]), res['yaw'])
                publish = True
                if self.frame == 'odom':
                    robot = self.odom.pose_at(stamp_to_sec(msg.header.stamp))
                    if robot is None:
                        self.get_logger().warn('No odometry for the image time yet',
                                               throttle_duration_sec=5.0)
                        publish = False
                    else:
                        p = compose(robot, p)
                if publish:
                    pose = PoseStamped()
                    pose.header.stamp = msg.header.stamp
                    pose.header.frame_id = self.frame
                    pose.pose.position.x, pose.pose.position.y = p[0], p[1]
                    pose.pose.orientation.z = math.sin(p[2] / 2)
                    pose.pose.orientation.w = math.cos(p[2] / 2)
                    self.pose_pub.publish(pose)
                self.len_pub.publish(Float32(data=res['length']))

            if self.debug_pub.get_subscription_count():
                img = rgb.copy()
                if res is not None:
                    img[res['mask']] = (255, 0, 255)
                    text = (f'strip {math.hypot(*res["centre"]):.2f} m, '
                            f'{math.degrees(res["yaw"]):+.0f} deg, len {res["length"]:.2f} m')
                else:
                    text = 'no strip'
                cv2.putText(img, text, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (255, 0, 255), 2)
                out = Image(header=msg.header, height=msg.height, width=msg.width,
                            encoding='rgb8', step=msg.width * 3, data=img.tobytes())
                self.debug_pub.publish(out)

    rclpy.init(args=args)
    node = StripDetector()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
