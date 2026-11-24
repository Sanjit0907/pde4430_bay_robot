"""bay_detector.py - finds the loading bay in the LiDAR scan.

The bay is three walls 0.5 m high, so the LiDAR (scan plane 0.35 m) sees them. The
detector fits a model of the bay - three rectangles with the sizes from the
assessment_bay brief - to the scan:

  1. Split the scan into straight segments (split-and-merge).
  2. Each segment could be any of the six long wall faces (inner/outer side walls,
     inner/outer back wall). Each pairing fixes the bay's rotation, and its position
     up to a slide along the face, which is sampled. That gives a few hundred
     candidate bay poses.
  3. Score each candidate: scan points that lie on the model's walls count for it;
     rays that pass straight through a model wall count heavily against it (a real
     wall there would have stopped them). This rejects fits to the arena walls,
     obstacles and objects.
  4. Refine the best candidate by least squares on its wall points.

Only the bay's known shape and size are used - never its position in the world, so
the robot finds the bay with its own sensor, as the brief requires.

Subscribes
    /scan                 sensor_msgs/LaserScan
    /odom                 nav_msgs/Odometry   robot pose at the scan time
Publishes
    /bay/opening          geometry_msgs/PoseStamped   centre of the opening, x axis
                                                      pointing into the bay
                                                      (frame: output_frame), stamped
                                                      with the scan time
    /bay/markers          visualization_msgs/MarkerArray  fitted walls, for RViz

Parameters
    output_frame (str, default "odom")   frame for /bay/opening: odom or base_footprint.
                                         odom uses the robot pose from /odom at the
                                         scan time (see odom_history.py for why not TF)
    min_inliers (int, default 12)        scan points that must lie on the bay walls
    max_violations (int, default 2)      rays allowed to pass through a fitted wall
"""
import math

import numpy as np

# ---- bay model (assessment_bay brief), in the bay frame: origin at the centre of the
# opening, +x into the bay, +y to the left when facing into the bay ----
INNER_W = 1.0           # opening / clear width
INNER_D = 1.2           # clear depth from the opening to the back wall
WALL_T = 0.1
HW = INNER_W / 2
# walls as (xmin, xmax, ymin, ymax)
WALLS = np.array([
    [0.0, INNER_D + WALL_T, HW, HW + WALL_T],                     # left side wall
    [0.0, INNER_D + WALL_T, -HW - WALL_T, -HW],                   # right side wall
    [INNER_D, INNER_D + WALL_T, -HW - WALL_T, HW + WALL_T],       # back wall
])
# long faces a scan segment can lie on: (point on face, direction along face,
# outward normal, start, end along the direction)
FACES = [
    ((0.0, HW + WALL_T), (1, 0), (0, 1), 0.0, INNER_D + WALL_T),    # left outer
    ((0.0, HW), (1, 0), (0, -1), 0.0, INNER_D),                     # left inner
    ((0.0, -HW), (1, 0), (0, 1), 0.0, INNER_D),                     # right inner
    ((0.0, -HW - WALL_T), (1, 0), (0, -1), 0.0, INNER_D + WALL_T),  # right outer
    ((INNER_D, -HW), (0, 1), (-1, 0), 0.0, INNER_W),                # back inner
    ((INNER_D + WALL_T, -HW - WALL_T), (0, 1), (1, 0), 0.0, INNER_W + 2 * WALL_T),  # back outer
]

INLIER_DIST = 0.05      # m, a point this close to a wall surface supports the fit
PASS_MARGIN = 0.12      # m, a ray reaching this far past a wall "passes through" it
SLIDE_STEP = 0.05       # m, sampling of the position along a face
MIN_SEG_LEN = 0.25      # m, shorter segments are not used as seeds
SPLIT_DIST = 0.03       # m, split-and-merge tolerance
EXTEND_DIST = 0.03      # m, a point this close to a face's line is "in line" with it
EXTEND_GAP = 0.15       # m, ignore points just past a face's end (corners, noise)
EXTEND_LEN = 1.0        # m, how far past a face's end to look for a wall carrying on
LENGTH_TOL = 0.05       # m, a segment can't be longer than the face it lies on
SHIFT_M, SHIFT_RAD = 0.15, 0.15   # ambiguity test: nudge the fit by this much...
SHIFT_KEEP = 0.8        # ...and reject it if a nudged fit keeps this share of points
MAX_TRIES = 30          # candidate fits refined and checked per scan


# ------------------------------------------------------------------ geometry
def rot(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def scan_points(ranges, angle_min, angle_inc, range_min, range_max):
    """Return (angles of all rays, valid mask, xy of valid points) in the scan frame."""
    ranges = np.asarray(ranges, dtype=float)
    angles = angle_min + angle_inc * np.arange(len(ranges))
    valid = np.isfinite(ranges) & (ranges > range_min) & (ranges < range_max * 0.99)
    pts = np.c_[ranges * np.cos(angles), ranges * np.sin(angles)][valid]
    return angles, valid, pts


def split_segments(pts):
    """Split consecutive scan points into straight segments (split-and-merge)."""
    if len(pts) < 2:
        return []
    # clusters: break where neighbouring points are far apart (scaled with range)
    gaps = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    limit = 0.08 + 0.04 * np.linalg.norm(pts[1:], axis=1)
    breaks = np.where(gaps > limit)[0] + 1
    segs = []

    def split(p):
        if len(p) < 4:
            return
        a, b = p[0], p[-1]
        d = b - a
        length = np.linalg.norm(d)
        if length < 1e-6:
            return
        n = np.array([-d[1], d[0]]) / length
        dev = np.abs((p - a) @ n)
        i = int(np.argmax(dev))
        if dev[i] > SPLIT_DIST and 2 <= i <= len(p) - 3:
            split(p[:i + 1])
            split(p[i:])
        else:
            segs.append(p)

    for c in np.split(pts, breaks):
        split(c)
    out = []
    for p in segs:
        centre = p.mean(axis=0)
        _, _, vt = np.linalg.svd(p - centre)
        u = vt[0]
        t = (p - centre) @ u
        length = t.max() - t.min()
        if length >= MIN_SEG_LEN:
            mid = centre + u * (t.max() + t.min()) / 2
            n = np.array([-u[1], u[0]])
            if n @ mid > 0:          # normal must point back towards the sensor
                n = -n
            out.append((mid, u, n, length))
    return out


def wall_distance(q):
    """Distance from points q (N x 2, bay frame) to the nearest wall; 0 inside a wall."""
    best = np.full(len(q), np.inf)
    for xmin, xmax, ymin, ymax in WALLS:
        dx = np.maximum(np.maximum(xmin - q[:, 0], 0), q[:, 0] - xmax)
        dy = np.maximum(np.maximum(ymin - q[:, 1], 0), q[:, 1] - ymax)
        best = np.minimum(best, np.hypot(dx, dy))
    return best


def ray_hit_range(origin, dirs):
    """Range at which rays (bay frame) first hit a wall; inf if they miss."""
    hit = np.full(len(dirs), np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        for xmin, xmax, ymin, ymax in WALLS:
            tx1 = (xmin - origin[0]) / dirs[:, 0]
            tx2 = (xmax - origin[0]) / dirs[:, 0]
            ty1 = (ymin - origin[1]) / dirs[:, 1]
            ty2 = (ymax - origin[1]) / dirs[:, 1]
            tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
            tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
            ok = (tmax >= np.maximum(tmin, 0)) & np.isfinite(tmin)
            hit = np.where(ok, np.minimum(hit, np.maximum(tmin, 0)), hit)
    return hit


class Scan:
    """One scan, prepared for scoring candidate bay poses."""

    def __init__(self, ranges, angle_min, angle_inc, range_min, range_max):
        self.angles, self.valid, self.pts = scan_points(
            ranges, angle_min, angle_inc, range_min, range_max)
        r = np.asarray(ranges, dtype=float)
        # rays with no return reached at least range_max
        self.ranges = np.where(self.valid, r, range_max)
        self.dirs = np.c_[np.cos(self.angles), np.sin(self.angles)]
        self.range_max = range_max

    def to_bay(self, pose, pts):
        """Scan-frame points -> bay frame, for a bay pose (x, y, theta) in the scan frame."""
        x, y, th = pose
        return (pts - [x, y]) @ rot(th)

    def inliers(self, pose):
        return wall_distance(self.to_bay(pose, self.pts)) < INLIER_DIST

    def violations(self, pose):
        """Evidence against the bay being at this pose:
        - rays that pass right through a wall (a real wall would have stopped them)
        - points that carry on in line with a wall face past its end. The bay's walls
          end; a longer wall (e.g. the arena wall, when the model is fitted flush
          against an arena corner from behind) does not."""
        x, y, th = pose
        R = rot(th)
        origin = R.T @ -np.array([x, y])
        hit = ray_hit_range(origin, self.dirs @ R)
        through = int(np.sum(self.ranges > hit + PASS_MARGIN))

        q = self.to_bay(pose, self.pts)
        off_wall = wall_distance(q) > INLIER_DIST
        beyond = np.zeros(len(q), bool)
        for p0, fd, fn, a, b in FACES:
            p0, fd, fn = np.array(p0, float), np.array(fd, float), np.array(fn, float)
            s = (q - p0) @ fd
            on_line = np.abs((q - p0) @ fn) < EXTEND_DIST
            beyond |= on_line & (((s > b + EXTEND_GAP) & (s < b + EXTEND_LEN)) |
                                 ((s < a - EXTEND_GAP) & (s > a - EXTEND_LEN)))
        extended = int(np.sum(beyond & off_wall))
        return through + extended // 2


def candidates(segments):
    """Bay poses (x, y, theta) in the scan frame from segment <-> face pairings."""
    out = []
    for mid, u, n, length in segments:
        for p0, fd, fn, a, b in FACES:
            fd, fn, p0 = np.array(fd, float), np.array(fn, float), np.array(p0, float)
            if length > (b - a) + LENGTH_TOL:
                continue
            # rotation that turns the face normal into the segment normal
            th = math.atan2(n[1], n[0]) - math.atan2(fn[1], fn[0])
            R = rot(th)
            lo, hi = a + length / 2, b - length / 2
            if hi < lo:
                lo = hi = (a + b) / 2
            for c in np.arange(lo, hi + 1e-9, SLIDE_STEP):
                q = p0 + fd * c                       # segment midpoint on the face
                t = mid - R @ q
                out.append((t[0], t[1], th))
    return out


def nearest_face(q):
    """For bay-frame points q (N x 2): index of the nearest wall face within its
    extent (-1 if none) and the signed offset from it, along its outward normal."""
    faces = FACES + END_FACES
    p0 = np.array([f[0] for f in faces], float)        # F x 2
    fd = np.array([f[1] for f in faces], float)
    fn = np.array([f[2] for f in faces], float)
    a = np.array([f[3] for f in faces]) - 0.05
    b = np.array([f[4] for f in faces]) + 0.05
    rel = q[:, None, :] - p0[None]                      # N x F x 2
    s = np.einsum('nfk,fk->nf', rel, fd)
    e = np.einsum('nfk,fk->nf', rel, fn)
    dist = np.where((s >= a) & (s <= b), np.abs(e), np.inf)
    idx = np.argmin(dist, axis=1)
    rows = np.arange(len(q))
    idx = np.where(np.isfinite(dist[rows, idx]), idx, -1)
    return idx, e[rows, np.maximum(idx, 0)], fn


def refine(scan, pose, iterations=6):
    """Least-squares fit of the bay pose to its wall points (point-to-face)."""
    x, y, th = pose
    for _ in range(iterations):
        q = scan.to_bay((x, y, th), scan.pts)
        use = wall_distance(q) < INLIER_DIST * 1.5
        idx, e, fn = nearest_face(q[use])
        ok = idx >= 0
        if ok.sum() < 6:
            break
        e, n_b, dp = e[ok], fn[idx[ok]], scan.pts[use][ok] - [x, y]
        # residual e = fn . (R^T (p - t) - p0); linearise in (dx, dy, dth):
        # de/dt = -(R fn), de/dth = fn . dR^T/dth (p - t)
        R = rot(th)
        nw = n_b @ R.T
        dRT = np.array([[-math.sin(th), math.cos(th)], [-math.cos(th), -math.sin(th)]])
        J = np.c_[-nw, np.einsum('nk,nk->n', n_b, dp @ dRT.T)]
        sol, *_ = np.linalg.lstsq(J, -e, rcond=None)
        x, y, th = x + sol[0], y + sol[1], th + sol[2]
        if np.linalg.norm(sol[:2]) < 1e-4 and abs(sol[2]) < 1e-4:
            break
    return x, y, th


def faces_seen(scan, pose, min_points=3):
    """Number of different wall faces with at least min_points scan points on them.
    One flat wall alone cannot place the bay (it fits several bay poses equally)."""
    q = scan.to_bay(pose, scan.pts[scan.inliers(pose)])
    if len(q) == 0:
        return 0
    idx, e, _ = nearest_face(q)
    idx = idx[(idx >= 0) & (np.abs(e) < INLIER_DIST)]
    return int(np.sum(np.bincount(idx, minlength=len(FACES) + len(END_FACES)) >= min_points))


def ambiguous(scan, pose, n_in, max_violations):
    """True if the scan does not pin the bay down: e.g. only one flat wall is in view,
    which fits the bay slid along it just as well. Nudges the fit along and across
    the bay, and turns it about the centre of its wall points. If a nudged fit keeps
    most of the wall points and no rays pass through its walls, it explains the scan
    as well as the original, so the pose is not determined."""
    x, y, th = pose
    R = rot(th)
    inl = scan.inliers(pose)
    if not inl.any():
        return True
    centre = scan.pts[inl].mean(axis=0)
    nudged = []
    for dx, dy in [(SHIFT_M, 0), (-SHIFT_M, 0), (0, SHIFT_M), (0, -SHIFT_M)]:
        d = R @ [dx, dy]                      # nudge in the bay frame
        nudged.append((x + d[0], y + d[1], th))
    for dth in (SHIFT_RAD, -SHIFT_RAD):       # turn about the wall points' centre
        t = centre + rot(dth) @ (np.array([x, y]) - centre)
        nudged.append((t[0], t[1], th + dth))
    for p in nudged:
        if (scan.inliers(p).sum() > SHIFT_KEEP * n_in
                and scan.violations(p) <= max_violations):
            return True
    return False


# short end faces of the side walls (at the opening), used only when refining
END_FACES = [
    ((0.0, HW), (0, 1), (-1, 0), 0.0, WALL_T),
    ((0.0, -HW - WALL_T), (0, 1), (-1, 0), 0.0, WALL_T),
]


def detect(ranges, angle_min, angle_inc, range_min, range_max,
           min_inliers=12, max_violations=2):
    """Find the bay in one scan.

    Returns (x, y, theta, inliers, violations) of the bay frame in the scan frame -
    (x, y) is the centre of the opening and theta the direction into the bay - or
    None if no fit is good enough.
    """
    scan = Scan(ranges, angle_min, angle_inc, range_min, range_max)
    segs = split_segments(scan.pts)
    cands = candidates(segs)
    if not cands:
        return None
    # score every candidate: wall points minus a penalty for evidence against it.
    # (Ranking by wall points alone favours fits along the long arena walls, which
    # the ray check then rejects.) Refine and fully check the best few, keeping the
    # best one that passes every check.
    counts = np.array([scan.inliers(c).sum() for c in cands])
    scores = np.full(len(cands), -np.inf)
    for i in np.nonzero(counts >= min_inliers)[0]:
        scores[i] = counts[i] - 3 * scan.violations(cands[i])
    best, best_score = None, -np.inf
    tried = []
    for i in np.argsort(-scores)[:MAX_TRIES]:
        if scores[i] < min_inliers:
            break
        pose = refine(scan, cands[i])
        if any(abs(pose[0] - p[0]) < 0.03 and abs(pose[1] - p[1]) < 0.03
               and abs(math.remainder(pose[2] - p[2], math.tau)) < 0.03 for p in tried):
            continue                           # converged to a fit already checked
        tried.append(pose)
        n_in, n_v = int(scan.inliers(pose).sum()), scan.violations(pose)
        score = n_in - 3 * n_v
        if (score <= best_score or n_in < min_inliers or n_v > max_violations
                or faces_seen(scan, pose) < 2 or ambiguous(scan, pose, n_in, max_violations)):
            continue
        best, best_score = (pose, n_in, n_v), score
    if best is None:
        return None
    (x, y, th), n_in, n_v = best
    return x, y, math.remainder(th, math.tau), n_in, n_v


# ====================================================================== ROS node
def main(args=None):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.duration import Duration
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from visualization_msgs.msg import Marker, MarkerArray

    from pde4430_bay_robot.odom_history import (MountLookup, OdomHistory, compose,
                                                stamp_to_sec)

    class BayDetector(Node):
        def __init__(self):
            super().__init__('bay_detector')
            self.frame = self.declare_parameter('output_frame', 'odom').value
            if self.frame not in ('odom', 'base_footprint'):
                self.get_logger().warn(f'output_frame "{self.frame}" not supported, '
                                       'using odom')
                self.frame = 'odom'
            self.min_inliers = self.declare_parameter('min_inliers', 12).value
            self.max_violations = self.declare_parameter('max_violations', 2).value
            self.mount = MountLookup(self)
            # /odom on its own thread, so it is up to date while a scan is being fitted
            self.odom = OdomHistory(self, MutuallyExclusiveCallbackGroup())
            self.pub = self.create_publisher(PoseStamped, '/bay/opening', 10)
            self.markers = self.create_publisher(MarkerArray, '/bay/markers', 10)
            self.create_subscription(LaserScan, '/scan', self.on_scan, 1)
            self.found = None
            self.get_logger().info(f'Bay detector ready (LiDAR model fit, output in '
                                   f'{self.frame})')

        def on_scan(self, msg):
            tr = self.mount.get(msg.header.frame_id)
            if tr is None:
                return
            mount = (tr.translation.x, tr.translation.y,
                     2 * math.atan2(tr.rotation.z, tr.rotation.w))
            res = detect(msg.ranges, msg.angle_min, msg.angle_increment,
                         msg.range_min, msg.range_max, self.min_inliers, self.max_violations)
            if res is None:
                if self.found is not False:
                    self.get_logger().info('Bay not in view')
                self.found = False
                return
            x, y, th, n_in, n_v = res
            # scanner frame -> robot -> odom, using the robot pose when the scan was taken
            pose = compose(mount, (x, y, th))
            if self.frame == 'odom':
                robot = self.odom.pose_at(stamp_to_sec(msg.header.stamp))
                if robot is None:
                    self.get_logger().warn('No odometry for the scan time yet',
                                           throttle_duration_sec=5.0)
                    return
                pose = compose(robot, pose)
            if self.found is not True:
                self.get_logger().info(
                    f'Bay found: opening {math.hypot(x, y):.2f} m away, '
                    f'{n_in} wall points, {n_v} rays through walls')
            self.found = True
            out = PoseStamped()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.frame
            out.pose.position.x, out.pose.position.y = float(pose[0]), float(pose[1])
            out.pose.orientation.z = math.sin(pose[2] / 2)
            out.pose.orientation.w = math.cos(pose[2] / 2)
            self.pub.publish(out)
            self.publish_markers(msg.header, x, y, th)

        def publish_markers(self, header, x, y, th):
            """The fitted walls, in the scanner frame (RViz places them with TF)."""
            arr = MarkerArray()
            R = rot(th)
            for i, (xmin, xmax, ymin, ymax) in enumerate(WALLS):
                c = R @ [(xmin + xmax) / 2, (ymin + ymax) / 2] + [x, y]
                m = Marker()
                m.header = header
                m.ns, m.id, m.type, m.action = 'bay_walls', i, Marker.CUBE, Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = c[0], c[1], 0.0
                m.pose.orientation.z, m.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
                m.scale.x, m.scale.y, m.scale.z = xmax - xmin, ymax - ymin, 0.5
                m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.8, 1.0, 0.5
                m.lifetime = Duration(seconds=0.5).to_msg()
                arr.markers.append(m)
            self.markers.publish(arr)

    rclpy.init(args=args)
    node = BayDetector()
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
