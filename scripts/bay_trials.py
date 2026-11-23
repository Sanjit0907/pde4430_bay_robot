#!/usr/bin/env python3
"""bay_trials.py - TEST HARNESS ONLY: scripted delivery trials in the assessment_bay world.

Checks that the robot's mechanical design can catch, carry (through an S-bend) and
deliver each of the three objects into the bay. It uses Gazebo ground truth (gz CLI)
to place things and to judge the result, which the brief allows for testing and
verification. None of this is used by the robot's own nodes.

Run with the simulation already up (sim.launch.py, objects spawned):
    python3 bay_trials.py [large_sphere small_sphere cylinder]

Each trial:
  1. teleport the robot to (0, -0.5) facing the bay, arms 'wide'
  2. teleport the object to (0, 0.4), straight ahead
  3. drive forward until the object is between the arms
  4. switch arms to the carry mode ('large' or 'close')
  5. caged objects: drive an S-bend (turn right then left) - must stay in the jaws;
     large sphere: push straight (it rolls freely and cannot be caged) - must stay centred
  6. push towards the bay and release
       large sphere: stop short of the opening, it rolls on into the bay
       small objects: drive in, open arms to 'straight', reverse out
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

sys.path.insert(0, '/root/ws/install/assessment_bay/lib/assessment_bay')
import bay_layout as L  # noqa: E402  (provided by the assessment_bay package)

WORLD = L.WORLD_NAME
ROBOT = 'bay_robot'
FRONT = 0.10                     # bumper face, metres ahead of the robot centre
# per object:
#   pre     arm mode while approaching
#   mode    arm mode while carrying
#   reach   object centre distance ahead of the bumper face when engaged
#   gap     stop the approach this far short of `reach` (0 = drive into contact)
#   push_to robot y at which the object is far enough into the bay
#   turns   True = test carrying it through an S-bend (caged objects only)
#   z       spawn height of the object centre
OBJECTS = {
    # large sphere: arms already in the 35 deg V, drive into it, push straight, stop
    # short of the opening (the V is 0.9 m wide) and let it roll on into the bay
    'large_sphere': dict(pre='large', mode='large', reach=0.294, gap=0.0, push_to=0.9,
                         turns=False, z=0.30),
    # small objects: open funnel, stop before touching (a bumped sphere rolls away), cage
    'small_sphere': dict(pre='wide', mode='close', reach=0.10, gap=0.04, push_to=1.72,
                         turns=True, z=0.10),
    'cylinder':     dict(pre='wide', mode='close', reach=0.10, gap=0.04, push_to=1.72,
                         turns=True, z=0.20),
}


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
        # Simulation time from /clock: Gazebo may run slower than real time (e.g. no GPU),
        # so all durations below are measured in simulated seconds.
        self.sim_t = None
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.arm_q = {}
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
            if 'arm' in n:
                self.arm_q[n] = q

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

    def drive(self, v, w, until, timeout=300.0, stop=True):
        """Publish (v, w) until until(robot_pose) is true or a wall timeout. Returns success.
        stop=False keeps moving afterwards (needed while pushing a rolling sphere)."""
        t = Twist()
        t.linear.x, t.angular.z = v, w
        end = time.time() + timeout
        while time.time() < end:
            self.cmd.publish(t)
            rclpy.spin_once(self, timeout_sec=0.02)
            if until(gz_pose(ROBOT)):
                if stop:
                    self.stop(0.2)
                return True
        self.stop()
        return False

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
        log = []
        default = L.FIXED_LAYOUT[obj]

        # 1-2. setup: robot 1.5 m south of where the object will be
        self.stop()
        self.set_arms(spec['pre'], 0.5)
        gz_set_pose(ROBOT, 0.0, -0.9, 0.05, math.pi / 2)
        self.sleep(1.0)
        gz_set_pose(obj, 0.0, 0.4, spec['z'])
        self.sleep(3.0)

        # 3. approach. Small objects: stop while BETWEEN the open arms but not yet
        #    touched (a bumped sphere rolls away at the robot's speed), then cage.
        #    Large sphere: drive slowly into the V. Fast to 0.25 m short, then slow.
        def gap(p):
            return gz_pose(obj)[1] - (p[1] + FRONT + spec['reach'])
        pushing = spec['gap'] == 0.0       # large sphere: never stop once in contact
        self.drive(0.15, 0.0, lambda p: gap(p) <= 0.25)
        ok = self.drive(0.08 if pushing else 0.05, 0.0,
                        lambda p: gap(p) <= spec['gap'], stop=not pushing)
        log.append(f'approach {"ok" if ok else "TIMEOUT"}')

        # 4. carry mode (caged objects) / keep pushing to seat the sphere in the V
        if pushing:
            self.drive_for(0.10, 0.0, 1.5, stop=False)
        else:
            self.set_arms(spec['mode'])
        self.snapshot(f'{obj}_1_approach')
        f, l = self.in_jaws(obj)
        log.append(f'gripped at fwd {f:.2f} lat {l:+.2f} arms {self.arms_deg()}')

        if spec['turns']:
            # 5a. caged objects: S-bend (~0.67 m radius), right then left, 26 deg each
            self.drive_for(0.10, -0.15, 3.0)
            self.drive_for(0.10, 0.15, 3.0)
            what = 'S-bend'
        else:
            # 5b. large sphere: it rolls freely and cannot be caged, so it is lined up
            #     and pushed STRAIGHT; the V keeps it centred. Push 0.4 m and check.
            self.drive_for(0.10, 0.0, 4.0, stop=False)
            what = 'straight push'
        self.snapshot(f'{obj}_2_carry')
        f, l = self.in_jaws(obj)
        if spec['turns']:
            kept = 0.05 < f < 0.85 and abs(l) < 0.15
            log.append(f'after {what} fwd {f:.2f} lat {l:+.2f} -> '
                       f'{"kept" if kept else "LOST"}')
        else:
            kept = True                    # judged only by where it ends up
            log.append(f'after {what} fwd {f:.2f} lat {l:+.2f}')

        # 6. deliver
        if spec['mode'] == 'close':
            # caged: drive into the bay, open the arms (parallel arms fit), reverse out
            self.drive(0.12, 0.0, lambda p: p[1] >= spec['push_to'])
            self.set_arms('straight')
            self.drive(-0.15, 0.0, lambda p: p[1] <= 0.8)
            self.sleep(3.0)
        else:
            # large sphere: keep pushing to just short of the opening, then stop -
            # it rolls on straight into the bay
            self.drive(0.12, 0.0, lambda p: p[1] >= spec['push_to'])
            self.sleep(12.0)
            self.snapshot(f'{obj}_3_released')

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
