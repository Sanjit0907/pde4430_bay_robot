#!/usr/bin/env python3
"""bay_trials.py - TEST HARNESS ONLY: scripted delivery trials in the assessment_bay world.

Checks that the robot's mechanical design can catch, carry (through an S-bend) and
deliver each of the three objects into the bay. It uses Gazebo ground truth (gz CLI)
to place things and to judge the result, which the brief allows for testing and
verification. None of this is used by the robot's own nodes.

Run with the simulation already up (sim.launch.py, objects spawned):
    python3 bay_trials.py [large_sphere small_sphere cylinder]

Each trial:
  1. teleport the robot to (0, -0.9) facing the bay, gripper set up for the object
  2. teleport the object to (0, 0.4), straight ahead
  3. drive up and stop just short of it
  4. pick it up:
       large sphere: scoop tray down, jaws in the 35 deg V, fingers open; the fingers
         HOOK round its front and push it up onto the tray, against the bumper
       small objects: scoop tray folded, jaws close (cage) then grip (squeeze)
     then raise the lift = PICK UP
  5. drive an S-bend (turn right then left) - it must stay held and raised
  6. line up with the bay, drive in, set it down, release, reverse out
  7. judge with assessment_bay's own object_in_bay() rule; check the cylinder is upright
"""
import math
import os
import re
import struct
import subprocess
import sys
import threading
import time
import zlib

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

sys.path.insert(0, '/root/ws/install/assessment_bay/lib/assessment_bay')   # Docker
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),    # native
                                '../../../install/assessment_bay/lib/assessment_bay'))
import bay_layout as L  # noqa: E402  (provided by the assessment_bay package)

WORLD = L.WORLD_NAME
ROBOT = 'bay_robot'
FRONT = 0.10                     # bumper face, metres ahead of the robot centre
# per object:
#   how      'hook' (large sphere: scoop tray + fingers) or 'grip' (jaw pads)
#   pre      jaw mode while approaching
#   stop_at  stop the approach when the object centre is this far ahead of the
#            robot centre (just short of touching it: a bumped sphere rolls away)
#   held_at  where the object centre sits once held (for lining up with the bay)
#   z        spawn height of the object centre
OBJECTS = {
    'large_sphere': dict(how='hook', pre='large', stop_at=0.66, held_at=0.41, z=0.30),
    'small_sphere': dict(how='grip', pre='wide', stop_at=0.24, held_at=0.23, z=0.10),
    'cylinder':     dict(how='grip', pre='wide', stop_at=0.24, held_at=0.23, z=0.20),
}
DROP_Y = 2.2          # object centre y when set down in the bay (strip at 1.55 - 1.65,
                      # back wall at 2.8: room for the 0.3 m radius large sphere)


class PoseStream:
    """Live ground-truth poses of every moving model, streamed from Gazebo's
    /world/<world>/dynamic_pose/info topic in a background thread (TEST ONLY).
    Much faster than calling `gz model -p` (1-2 s per call in a slow simulation)."""

    def __init__(self):
        self.poses = {}
        self.proc = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', f'/world/{WORLD}/dynamic_pose/info'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        """Token parser for the protobuf text output (works for one- or multi-line
        layouts; fields equal to zero are omitted by protobuf, so default to 0)."""
        rec, block, key = None, None, None
        for line in self.proc.stdout:
            for tok in re.findall(r'"[^"]*"|[{}]|[^\s{}]+', line):
                if key == 'name':
                    rec = {'p': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                           'o': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}}
                    self.poses[tok.strip('"')] = rec
                    key = None
                elif key in ('x', 'y', 'z', 'w') and rec is not None and block:
                    rec[block][key] = float(tok)
                    key = None
                elif tok == 'name:':
                    key = 'name'
                elif tok in ('position', 'orientation'):
                    block = tok[0] if tok == 'position' else 'o'
                elif tok == '}':
                    block = None
                elif tok.rstrip(':') in ('x', 'y', 'z', 'w') and tok.endswith(':'):
                    key = tok[0]
                else:
                    key = None

    def get(self, name, timeout=30.0):
        """(x, y, z, roll, pitch, yaw) of a model."""
        end = time.time() + timeout
        while name not in self.poses:
            if time.time() > end:
                raise RuntimeError(f'no pose received for "{name}"')
            time.sleep(0.05)
        p, o = self.poses[name]['p'], self.poses[name]['o']
        qx, qy, qz, qw = o['x'], o['y'], o['z'], o['w']
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        return (p['x'], p['y'], p['z'], roll, pitch, yaw)


STREAM = None


def gz_pose(name):
    """Return (x, y, z, roll, pitch, yaw) of a model from Gazebo ground truth."""
    return STREAM.get(name)


def gz_set_pose(name, x, y, z, yaw=0.0):
    """Teleport a model (test setup only)."""
    req = (f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
    subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '3000', '--req', req],
                   capture_output=True, text=True, timeout=20)


class Trials(Node):
    def __init__(self):
        super().__init__('bay_trials')
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm = self.create_publisher(String, '/arm_mode', 10)
        self.lift = self.create_publisher(String, '/lift_mode', 10)
        self.scoop = self.create_publisher(String, '/scoop_mode', 10)
        self.fingers = self.create_publisher(String, '/finger_mode', 10)
        self.finger_state = None
        self.create_subscription(String, '/finger_state',
                                 lambda m: setattr(self, 'finger_state', m.data), 10)
        # Simulation time from /clock: Gazebo may run slower than real time (e.g. no GPU),
        # so all durations below are measured in simulated seconds.
        self.sim_t = None
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.arm_q = {}
        self.wheel_v = {}
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.image = None
        self.create_subscription(Image, '/camera/image_raw',
                                 lambda m: setattr(self, 'image', m), 1)

    def snapshot(self, tag):
        """Save the robot camera's current view as a PNG (pure Python, no OpenCV)."""
        if os.environ.get('SNAPSHOTS') is None:
            return
        self.image = None
        end = time.time() + 30
        while self.image is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        m = self.image
        if m is None or m.encoding != 'rgb8':
            return
        rows = b''.join(b'\x00' + bytes(m.data[r * m.step:r * m.step + m.width * 3])
                        for r in range(m.height))

        def chunk(kind, data):
            return (struct.pack('>I', len(data)) + kind + data
                    + struct.pack('>I', zlib.crc32(kind + data) & 0xffffffff))
        png = (b'\x89PNG\r\n\x1a\n'
               + chunk(b'IHDR', struct.pack('>IIBBBBB', m.width, m.height, 8, 2, 0, 0, 0))
               + chunk(b'IDAT', zlib.compress(rows, 6)) + chunk(b'IEND', b''))
        path = os.path.join(os.environ['SNAPSHOTS'], f'{tag}.png')
        with open(path, 'wb') as f:
            f.write(png)

    def on_joints(self, msg):
        for n, q in zip(msg.name, msg.position):
            if 'arm' in n or 'lift' in n or 'finger' in n or 'scoop' in n:
                self.arm_q[n] = q
        for n, v in zip(msg.name, msg.velocity):
            if 'wheel' in n:
                self.wheel_v[n] = v

    def arms_deg(self):
        """Actual arm angles as 'L/R' in degrees outward from straight ahead."""
        rclpy.spin_once(self, timeout_sec=0.05)
        return '/'.join(f'{180 - math.degrees(self.arm_q.get(j, float("nan"))):.0f}'
                        for j in ('left_arm_joint', 'right_arm_joint'))

    def on_clock(self, msg):
        self.sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9

    # ---- helpers ----
    def sleep(self, sec, wall_timeout=120.0):
        """Wait `sec` seconds of SIMULATION time (bounded by a wall-clock timeout)."""
        while self.sim_t is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        end, wall_end = self.sim_t + sec, time.time() + wall_timeout
        while self.sim_t < end and time.time() < wall_end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self, settle=0.5):
        self.cmd.publish(Twist())
        self.sleep(settle)

    def drive(self, v, w, until, timeout=60.0, stop=True):
        """Publish (v, w) until until(robot_pose) is true or `timeout` seconds of
        SIMULATION time pass (a slow, contact-heavy simulation must not cut it short).
        stop=False keeps moving afterwards (needed while pushing a rolling sphere).
        Returns success; the last result is kept in self.last_drive for the log."""
        t = Twist()
        t.linear.x, t.angular.z = v, w
        while self.sim_t is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        t0, wall0 = self.sim_t, time.time()
        end, wall_end = t0 + timeout, wall0 + 3600.0
        ok = False
        while self.sim_t < end and time.time() < wall_end:
            self.cmd.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)
            if until(gz_pose(ROBOT)):
                ok = True
                break
        sim_dt, wall_dt = self.sim_t - t0, time.time() - wall0
        self.last_drive = (f'{"ok" if ok else "TIMEOUT"} after {sim_dt:.0f} s sim '
                           f'(RTF {sim_dt / max(wall_dt, 1e-3):.2f})')
        if stop or not ok:
            self.stop(0.2)
        return ok

    def drive_for(self, v, w, sec, stop=True):
        """Publish (v, w) for `sec` seconds of simulation time (open loop)."""
        t = Twist()
        t.linear.x, t.angular.z = v, w
        while self.sim_t is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        end = self.sim_t + sec
        while self.sim_t < end:
            self.cmd.publish(t)
            rclpy.spin_once(self, timeout_sec=0.05)
        if stop:
            self.stop(0.3)

    def set_arms(self, mode, wait=2.5):
        self.arm.publish(String(data=mode))
        self.sleep(wait)

    def set_lift(self, height, wait=2.5):
        self.lift.publish(String(data=height))
        self.sleep(wait)

    def set_scoop(self, mode, wait=2.0):
        self.scoop.publish(String(data=mode))
        self.sleep(wait)

    def set_fingers(self, mode, wait=5.5):
        """Fingers move ramped (40 deg/s); 'hook' runs arm_controller's sweep-centre-hold
        routine, so wait until /finger_state reports it done."""
        self.fingers.publish(String(data=mode))
        if mode == 'hook':
            self.sleep(1.0)
            end = time.time() + 90
            while self.finger_state != 'hook' and time.time() < end:
                rclpy.spin_once(self, timeout_sec=0.1)
            self.sleep(1.0)
        else:
            self.sleep(wait)

    def face(self, x, y, tol=math.radians(1.5)):
        """Turn on the spot to face the point (x, y) (ground-truth pose, test only)."""
        def err(p):
            return math.remainder(math.atan2(y - p[1], x - p[0]) - p[5], math.tau)
        for _ in range(3):          # turn, settle, correct the overshoot
            e = err(gz_pose(ROBOT))
            if abs(e) < tol:
                return
            self.drive(0.0, 0.3 if e > 0 else -0.3, lambda p: abs(err(p)) < tol, timeout=30)
            self.stop(0.5)

    def goto(self, x, y, speed=0.12):
        """Face (x, y) and drive to it, steering on the way (test only)."""
        self.face(x, y)
        t = Twist()
        while self.sim_t is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        end = self.sim_t + 60
        while self.sim_t < end:
            p = gz_pose(ROBOT)
            d = math.hypot(x - p[0], y - p[1])
            if d < 0.03:
                break
            e = math.remainder(math.atan2(y - p[1], x - p[0]) - p[5], math.tau)
            t.linear.x = min(speed, 0.5 * d + 0.03)
            t.angular.z = max(-0.4, min(0.4, 1.5 * e))
            self.cmd.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)
        self.stop(0.5)

    def in_jaws(self, obj):
        """Object position in the robot frame: (forward, lateral)."""
        rx, ry, _, _, _, ryaw = gz_pose(ROBOT)
        ox, oy, *_ = gz_pose(obj)
        dx, dy = ox - rx, oy - ry
        return (dx * math.cos(ryaw) + dy * math.sin(ryaw),
                -dx * math.sin(ryaw) + dy * math.cos(ryaw))

    # ---- one trial ----
    def trial(self, obj):
        spec = OBJECTS[obj]
        hook = spec['how'] == 'hook'
        log = []
        default = L.FIXED_LAYOUT[obj]

        # 1-2. setup: robot 1.3 m south of where the object will be
        self.stop()
        self.set_lift('down', 1.0)        # lift must be down before anything opens
        self.set_fingers('fold', 1.5)
        self.set_arms(spec['pre'], 1.0)
        self.set_scoop('down' if hook else 'up', 1.5)
        if hook:
            self.set_fingers('open')
        gz_set_pose(ROBOT, 0.0, -0.9, 0.05, math.pi / 2)
        self.sleep(1.0)
        gz_set_pose(obj, 0.0, 0.4, spec['z'])
        self.sleep(3.0)

        # 3. approach and stop just short of the object
        self.drive(0.15, 0.0, lambda p: self.in_jaws(obj)[0] <= spec['stop_at'] + 0.25)
        ok = self.drive(0.05, 0.0, lambda p: self.in_jaws(obj)[0] <= spec['stop_at'])
        log.append(f'approach {"ok" if ok else "TIMEOUT"}')

        # 4. take hold and pick it up
        if hook:
            self.set_fingers('hook', 4.0)    # sweep it up onto the tray and lock it
        else:
            self.set_arms('close')
            self.set_arms('grip', 2.0)
        self.snapshot(f'{obj}_1_held')
        f, l = self.in_jaws(obj)
        log.append(f'held at fwd {f:.2f} lat {l:+.2f} arms {self.arms_deg()}')
        self.set_lift('carry')
        self.sleep(2.0)
        raised = gz_pose(obj)[2] - spec['z']
        log.append(f'robot pitch {math.degrees(gz_pose(ROBOT)[4]):+.1f} deg, '
                   f'lift {self.arm_q.get("lift_joint", float("nan")) * 100:.1f} cm, '
                   f'object raised {raised * 100:+.1f} cm -> '
                   f'{"PICKED UP" if raised > 0.05 else "NOT LIFTED"}')

        # 5. S-bend (~0.67 m radius), right then left, 26 deg each
        self.drive_for(0.10, -0.15, 3.0)
        self.drive_for(0.10, 0.15, 3.0)
        self.snapshot(f'{obj}_2_carry')
        f, l = self.in_jaws(obj)
        still_up = gz_pose(obj)[2] - spec['z'] > 0.05
        kept = abs(f - spec['held_at']) < 0.15 and abs(l) < 0.15 and still_up
        log.append(f'after S-bend fwd {f:.2f} lat {l:+.2f} '
                   f'{"still held up" if still_up else "DROPPED"} -> '
                   f'{"kept" if kept else "LOST"}')

        # 6. line up with the bay (the opening is 1.0 m; with the fingers hooked the
        #    robot is ~0.9 m wide), drive in until the object is over DROP_Y, set it
        #    down, release, reverse out
        r_obj = L.OBJECTS[obj]['radius']
        line_up_y = min(gz_pose(ROBOT)[1] + 0.3,                  # ahead, never back
                        L.STRIP_INNER_Y - 0.1 - spec['held_at'] - r_obj - 0.05)
        self.goto(0.0, line_up_y)
        self.face(0.0, 3.0)
        stop_y = DROP_Y - spec['held_at']
        self.drive(0.10, 0.0, lambda p: p[1] >= stop_y)
        log.append(f'carry to bay {self.last_drive}')
        self.set_lift('down')
        if hook:
            self.set_fingers('open')
        else:
            self.set_arms('straight', 2.0)
        self.snapshot(f'{obj}_3_released')
        log.append(f'released at y {gz_pose(obj)[1]:.2f} (robot {gz_pose(ROBOT)[1]:.2f})')
        self.drive(-0.15, 0.0, lambda p: p[1] <= 0.6)
        self.sleep(4.0)

        # 7. judge (assessment_bay rule)
        x, y, z, roll, pitch, _ = gz_pose(obj)
        r = L.OBJECTS[obj]['radius']
        upright = True
        if L.OBJECTS[obj]['shape'] == 'cylinder':    # only the cylinder can tip over
            upright = math.cos(roll) * math.cos(pitch) > math.cos(math.radians(45))
            if not upright:
                r = L.OBJECTS[obj]['length'] / 2
        delivered = L.object_in_bay(x, y, r)
        log.append(f'final ({x:+.2f}, {y:.2f}) {"IN BAY" if delivered else "not in bay"}'
                   + ('' if upright else ' TIPPED'))
        print(f'[{"PASS" if delivered and kept else "FAIL"}] {obj:<13} ' + ' | '.join(log),
              flush=True)

        # put it back where it started so it is not in the way of the next trial
        gz_set_pose(obj, default[0], default[1], spec['z'])
        self.sleep(1.0)
        return delivered and kept


def main():
    global STREAM
    names = sys.argv[1:] or list(OBJECTS)
    STREAM = PoseStream()
    STREAM.get(ROBOT)          # wait until poses are flowing
    rclpy.init()
    node = Trials()
    node.sleep(1.0)
    results = [node.trial(n) for n in names]
    print(f'---- {sum(results)}/{len(results)} objects delivered ----', flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
