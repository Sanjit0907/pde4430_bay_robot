"""arm_controller.py - gripper controller: named modes -> joint targets.

The gripper has four parts, all on the lift carriage:
  * two JAWS that pivot about vertical axes at the bumper corners
  * a SCOOP TRAY (ramp) hinged at the foot of the bumper: folded up at spawn, flat on
    the floor when deployed, so a sphere the robot drives into rolls onto it
  * two HOOK FINGERS at the jaw tips. Folded they lie along the jaws; open they
    continue the jaws forwards past the large sphere; swinging from open to hook
    they sweep round its front, push it back up onto the tray and press it against
    the bumper
  * the LIFT that raises all of them, and whatever they hold
Objects are only ever held by contact - nothing is attached.

Subscribes
    /arm_mode           std_msgs/String   jaw mode: fold, wide, large, straight, close, grip
    /lift_mode          std_msgs/String   lift: down, carry, up - or a height in metres
    /scoop_mode         std_msgs/String   scoop tray: up (folded) or down (flat)
    /finger_mode        std_msgs/String   fingers: fold, open or hook
Publishes
    /left_arm/cmd_pos   std_msgs/Float64  left jaw joint target (rad), bridged to Gazebo
    /right_arm/cmd_pos  std_msgs/Float64  right jaw joint target (rad), bridged to Gazebo
    /lift/cmd_pos       std_msgs/Float64  lift height target (m), bridged to Gazebo
    /scoop/cmd_pos      std_msgs/Float64  scoop tray angle target (rad)
    /fingers/cmd_pos    std_msgs/Float64  finger angle target (rad), both fingers
    /arm_state          std_msgs/String   current jaw mode (latched)
    /lift_state         std_msgs/String   current lift setting (latched)
    /scoop_state        std_msgs/String   current scoop setting (latched)
    /finger_state       std_msgs/String   current finger setting (latched): fold, open,
                                          hooking (routine running), hook

Jaw angle convention (see urdf/bay_robot.urdf): 0 rad = folded back along the body,
pi rad = pointing straight ahead; the jaw direction is (180 deg - angle) outward.

Safety rules
    * Jaws can only open (any mode except grip/close) when the lift is down, so a
      lifted object is never dropped from height by accident.
    * Fingers can only open or fold while the lift is down (hooked, they hold the
      lifted large sphere).
    * Jaws cannot move while the fingers are hooked (they would drag or crush the
      sphere the fingers are holding), and with the fingers open the jaws can only
      move between wide, large and straight (closer, the long fingers would cross).
    * Fingers can only open or hook with the jaws in wide, large or straight.
    * The fingers move to a new setting at 40 deg/s (ramped), not at full speed.
    * Hooking runs a routine: sweep the large sphere onto the tray (jaws relaxed to
      40 deg), centre it (jaws squeeze / fingers press, three times), hold. The
      finger state reads 'hooking' until it is done, then 'hook'. Letting go relaxes
      the jaws first, then opens or folds the fingers.
    * The scoop tray only moves while the lift is down.
    * 'fold' folds everything (fingers, tray, jaws) and lowers the lift first.
    * All targets are clamped to the joint limits.

Parameters
    deploy_mode (str, default "wide")  jaw mode to move to automatically after start-up
    deploy_scoop (str, default "down") scoop setting after start-up ("" = stay folded)
    deploy_delay (float, default 3.0)  seconds after the robot appears before deploying
    mode_angles.<name> (float, rad)    override the angle of any jaw mode
    lift_heights.<name> (float, m)     override any named lift height
"""
import math
from collections import deque

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

# Jaw mode angles (rad), derived from the assessment_bay object sizes (README, "Arm modes").
DEFAULT_MODES = {
    'fold': 0.0,                       # spawn pose, inside the 0.5 m x 0.5 m footprint
    'wide': math.radians(120.0),       # 60 deg outward: wide funnel to catch objects
    'large': math.radians(145.0),      # 35 deg outward: V that cradles the 0.6 m sphere
    'straight': math.pi,               # jaws parallel, straight ahead (release in the bay)
    'close': math.radians(202.0),      # 22 deg inward: loose cage for the 0.2 m objects
    'grip': math.radians(220.0),       # 40 deg inward target, past where the pads meet a
                                       # 0.2 m object (~25-27 deg): the jaw motors stay at
                                       # their effort limit, so the squeeze force is set by
                                       # the URDF effort (5 N m), the same for every object
}
HOLDING_MODES = ('grip', 'close')      # modes that keep an object between the jaws
HOOK_OK_MODES = ('wide', 'large', 'straight')   # jaws open enough for the fingers

# Lift heights (m) of the jaw carriage.
DEFAULT_LIFTS = {'down': 0.0, 'carry': 0.08, 'up': 0.15}

# Scoop tray: folded up / flat. 'down' asks for a little past flat, so the tray rests on
# its joint stop and the weight on it is carried by the stop.
SCOOP = {'up': -0.05, 'down': math.pi / 2 + 0.05}
# Fingers (angle to their jaw): folded along it / hooked at ~90 deg / open, straight on.
# 'fold' asks a little past 0 so they rest on the joint stop.
HOOK_SWEEP = math.radians(80.0)        # sweep target: a little past where they meet the
                                       # large sphere, so they push it up onto the tray
HOOK_LIGHT = math.radians(92.0)        # just touching: lets the jaws move the sphere
HOOK_HOLD = math.radians(84.0)         # final hold: pressing it into the jaws' V
FINGERS = {'fold': -0.05, 'hook': HOOK_HOLD, 'open': math.pi}
# After the sweep the sphere sits wherever the sweep left it (4-7 cm off centre in the
# trials). Centring routine: a few cycles of "jaws squeeze to 35 deg with the fingers
# just touching" (the jaws' V pushes it towards the middle) and "jaws relax, fingers
# press" (pushes it back onto the tray). Each cycle moves it closer to the middle:
# measured -7 cm -> -1 cm. Then hold with the jaws at 35 deg and the fingers pressing.
CENTRE_CYCLES = 3
CENTRE_TIME = 2.0                      # s per half-cycle (computer time)
RELAX_TIME = 1.0                       # s, jaws relax before the fingers let go
# The finger target moves towards a new setting at this rate instead of jumping, so
# both fingers sweep round the sphere slowly and together. Swinging at full speed one
# reached it first and shoved it 5-13 cm sideways.
FINGER_RATE = math.radians(40.0)       # rad/s: open -> hook takes ~2.5 s
# With the jaws in 'large', they relax from 35 to 40 deg outward (~1.8 cm clear of the
# large sphere) while the fingers are open or sweeping, and BEFORE the fingers let go
# of it. Squeezing during the sweep wedged the sphere off centre, and squeezing at
# release made the jaws spring back and shoot it 1 m out of the jaws.
LARGE_HOLD = math.radians(140.0)       # 40 deg outward
FINGER_TICK = 0.05                     # s, ramp update period

JAW_LIMIT = (0.0, 3.9)                 # must match <limit> in the URDF
LIFT_LIMIT = (0.0, 0.15)
SCOOP_LIMIT = (-0.1, math.pi / 2 + 0.1)
FINGER_LIMIT = (-0.1, 3.25)
LIFTED = 0.01                          # above this the lift counts as raised


class ArmController(Node):
    """Maps gripper modes to joint targets and keeps re-sending them."""

    def __init__(self):
        super().__init__('arm_controller')

        # ---- parameters ----
        self.declare_parameter('deploy_mode', 'wide')
        self.declare_parameter('deploy_scoop', 'down')
        self.declare_parameter('deploy_delay', 3.0)
        self.modes = {n: self.declare_parameter(f'mode_angles.{n}', a).value
                      for n, a in DEFAULT_MODES.items()}
        self.lifts = {n: self.declare_parameter(f'lift_heights.{n}', h).value
                      for n, h in DEFAULT_LIFTS.items()}

        # ---- publishers / subscribers ----
        self.left_pub = self.create_publisher(Float64, '/left_arm/cmd_pos', 10)
        self.right_pub = self.create_publisher(Float64, '/right_arm/cmd_pos', 10)
        self.lift_pub = self.create_publisher(Float64, '/lift/cmd_pos', 10)
        self.scoop_pub = self.create_publisher(Float64, '/scoop/cmd_pos', 10)
        self.finger_pub = self.create_publisher(Float64, '/fingers/cmd_pos', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/arm_state', latched)
        self.lift_state_pub = self.create_publisher(String, '/lift_state', latched)
        self.scoop_state_pub = self.create_publisher(String, '/scoop_state', latched)
        self.finger_state_pub = self.create_publisher(String, '/finger_state', latched)
        self.create_subscription(String, '/arm_mode', self.on_mode, 10)
        self.create_subscription(String, '/lift_mode', self.on_lift, 10)
        self.create_subscription(String, '/scoop_mode', self.on_scoop, 10)
        self.create_subscription(String, '/finger_mode', self.on_fingers, 10)

        # Start folded and down. Deploy only once the robot exists in the simulation
        # (its joints appear on /joint_states), then after a short delay - a command
        # sent before the robot has spawned is lost.
        self.mode, self.target = 'fold', self.modes['fold']
        self.lift_name, self.lift_target = 'down', 0.0
        self.scoop = 'up'
        self.fingers = 'fold'                  # requested finger setting
        self.finger_cmd = FINGERS['fold']      # ramped finger target actually sent
        self.finger_goal = FINGERS['fold']     # where the ramp is heading now
        self.jaw_override = None               # relaxed jaw angle in 'large', or None
        self.steps = deque()                   # finger routine still to run
        self.step_left = 0.0                   # s left in the current routine step
        self.deploy_timer = None
        self.deployed = False
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)

        # Re-send targets at 5 Hz: Gazebo only listens once the bridge is up, so a single
        # message at start-up could be lost. This timer runs on the computer's steady
        # clock: a simulation-time timer created before /clock exists never fires.
        self.create_timer(0.2, self.send_targets, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(FINGER_TICK, self.ramp_fingers,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_state()
        self.get_logger().info(
            f'Gripper ready. Jaw modes: {", ".join(self.modes)}; '
            f'lift: {", ".join(self.lifts)} or metres (0 - {LIFT_LIMIT[1]}); '
            f'scoop: {", ".join(SCOOP)}; fingers: {", ".join(FINGERS)}')

    @property
    def lifted(self):
        return self.lift_target > LIFTED

    # ------------------------------------------------------------------ start-up
    def on_joints(self, msg):
        """First sign of the spawned robot: start the deploy countdown (once)."""
        if self.deploy_timer is None and 'left_arm_joint' in msg.name:
            self.deploy_timer = self.create_timer(
                self.get_parameter('deploy_delay').value, self.deploy_once)

    def deploy_once(self):
        """Move from the folded spawn pose to the deploy modes, once."""
        self.deploy_timer.cancel()
        if self.deployed:
            return
        self.deployed = True
        deploy = self.get_parameter('deploy_mode').value
        if deploy and deploy != 'fold' and self.mode == 'fold':   # user may already have
            self.set_mode(deploy)                                 # chosen a mode
        scoop = self.get_parameter('deploy_scoop').value
        if scoop and self.scoop == 'up':
            self.set_scoop(scoop)

    # ------------------------------------------------------------------ jaws
    def on_mode(self, msg):
        self.set_mode(msg.data.strip().lower())

    def set_mode(self, name):
        """Validate a jaw mode, apply the safety rules and make it the target."""
        if name not in self.modes:
            self.get_logger().warn(f'Unknown jaw mode "{name}". Valid: {", ".join(self.modes)}')
            return
        if name == 'fold':
            # fold everything: lift down, fingers in, tray up, then the jaws
            if self.lifted:
                self.get_logger().info('Lowering the lift before folding')
                self.set_lift('down')
            if self.fingers != 'fold':
                self.set_fingers('fold')
            if self.scoop != 'up':
                self.set_scoop('up')
        elif self.fingers == 'hook':
            self.get_logger().warn(
                f'Refusing to move the jaws ("{name}") while the fingers are hooked - '
                'they would drag the held sphere. Fold the fingers first: /finger_mode fold')
            return
        elif self.fingers == 'open' and name not in HOOK_OK_MODES:
            self.get_logger().warn(
                f'Refusing to move the jaws to "{name}" with the fingers open - the long '
                'fingers would cross. Fold the fingers first: /finger_mode fold')
            return
        elif name not in HOLDING_MODES and self.lifted:
            self.get_logger().warn(
                f'Refusing to open the jaws ("{name}") while lifted - it would drop the '
                'object. Lower first: /lift_mode down')
            return
        angle = self.modes[name]
        clamped = min(max(angle, JAW_LIMIT[0]), JAW_LIMIT[1])
        if clamped != angle:
            self.get_logger().warn(f'Jaw mode "{name}" angle {angle:.3f} clamped to {clamped:.3f}')
        self.mode, self.target = name, clamped
        self.get_logger().info(f'Jaws -> {name} ({180 - math.degrees(clamped):+.0f} deg outward)')
        self.publish_state()
        self.send_targets()

    # ------------------------------------------------------------------ lift
    def on_lift(self, msg):
        self.set_lift(msg.data.strip().lower())

    def set_lift(self, request):
        """Accept a named height or a number in metres; clamp to the lift's range."""
        if request in self.lifts:
            name, height = request, self.lifts[request]
        else:
            try:
                height = float(request)
            except ValueError:
                self.get_logger().warn(
                    f'Unknown lift request "{request}". Use {", ".join(self.lifts)} or metres')
                return
            name = f'{height:.3f} m'
        clamped = min(max(height, LIFT_LIMIT[0]), LIFT_LIMIT[1])
        if clamped != height:
            self.get_logger().warn(f'Lift {height:.3f} m clamped to {clamped:.3f} m')
        if clamped > LIFTED and self.mode not in HOLDING_MODES and self.fingers != 'hook':
            self.get_logger().info(
                f'Lifting with the jaws in "{self.mode}" and the fingers folded - nothing '
                'is held (grip the jaws or hook the fingers first to pick something up)')
        self.lift_name, self.lift_target = name, clamped
        self.get_logger().info(f'Lift -> {name} ({clamped:.3f} m)')
        self.publish_state()
        self.send_targets()

    # ------------------------------------------------------------------ scoop tray
    def on_scoop(self, msg):
        self.set_scoop(msg.data.strip().lower())

    def set_scoop(self, name):
        if name not in SCOOP:
            self.get_logger().warn(f'Unknown scoop mode "{name}". Valid: {", ".join(SCOOP)}')
            return
        if self.lifted:
            self.get_logger().warn(
                f'Refusing to move the scoop tray ("{name}") while lifted. '
                'Lower first: /lift_mode down')
            return
        self.scoop = name
        self.get_logger().info(f'Scoop tray -> {name}')
        self.publish_state()
        self.send_targets()

    # ------------------------------------------------------------------ fingers
    def on_fingers(self, msg):
        self.set_fingers(msg.data.strip().lower())

    def set_fingers(self, name):
        if name not in FINGERS:
            self.get_logger().warn(f'Unknown finger mode "{name}". Valid: {", ".join(FINGERS)}')
            return
        if name != 'hook' and self.fingers == 'hook' and self.lifted:
            self.get_logger().warn(
                f'Refusing to {name} the fingers while lifted - they hold the sphere. '
                'Lower first: /lift_mode down')
            return
        if name != 'fold' and self.mode not in HOOK_OK_MODES:
            self.get_logger().warn(
                f'Refusing to {name} the fingers with the jaws in "{self.mode}" - they '
                f'would cross. Open the jaws first ({", ".join(HOOK_OK_MODES)})')
            return
        if name == self.fingers:
            return
        # The routine: (finger target, jaw override, seconds to hold once reached)
        steps = []
        if self.fingers == 'hook':                    # letting go: relax the jaws first
            steps.append((self.finger_goal, LARGE_HOLD, RELAX_TIME))
        if name == 'hook':
            steps.append((HOOK_SWEEP, LARGE_HOLD, 1.0))         # sweep it onto the tray
            for _ in range(CENTRE_CYCLES):                      # centre it
                steps.append((HOOK_LIGHT, None, CENTRE_TIME))
                steps.append((HOOK_SWEEP, LARGE_HOLD, CENTRE_TIME))
            steps.append((HOOK_HOLD, None, 0.0))                # hold
        else:
            steps.append((FINGERS[name], LARGE_HOLD if name == 'open' else None, 0.0))
        self.fingers = name
        self.steps = deque(steps)
        self.next_step()
        self.get_logger().info(f'Fingers -> {name}'
                               + (' (sweep, centre, hold)' if name == 'hook' else ''))
        self.publish_state()
        self.send_targets()

    def next_step(self):
        self.finger_goal, self.jaw_override, self.step_left = self.steps.popleft()

    def ramp_fingers(self):
        """20 Hz: move the sent finger target towards the current step's target; once
        there, hold for the step's time, then start the next step."""
        if self.finger_cmd != self.finger_goal:
            step = FINGER_RATE * FINGER_TICK
            self.finger_cmd += max(-step, min(step, self.finger_goal - self.finger_cmd))
        elif self.step_left > 0:
            self.step_left -= FINGER_TICK
        elif self.steps:
            self.next_step()
        else:
            if self.finger_state() != self.published_fingers:
                self.publish_state()            # routine finished ('hooking' -> 'hook')
            return
        self.send_targets()

    def finger_state(self):
        busy = self.steps or self.finger_cmd != self.finger_goal or self.step_left > 0
        return 'hooking' if self.fingers == 'hook' and busy else self.fingers

    def jaw_target(self):
        """The jaw angle to send: the mode's angle, or relaxed (LARGE_HOLD) while the
        finger routine asks for it in 'large'."""
        if self.mode == 'large' and self.jaw_override is not None:
            return self.jaw_override
        return self.target

    @staticmethod
    def clamp_finger(angle):
        return min(max(angle, FINGER_LIMIT[0]), FINGER_LIMIT[1])

    # ------------------------------------------------------------------ output
    def send_targets(self):
        """Publish every target (the URDF mirrors the jaws and the fingers)."""
        jaw = Float64(data=float(self.jaw_target()))
        self.left_pub.publish(jaw)
        self.right_pub.publish(jaw)
        self.lift_pub.publish(Float64(data=float(self.lift_target)))
        scoop = min(max(SCOOP[self.scoop], SCOOP_LIMIT[0]), SCOOP_LIMIT[1])
        self.scoop_pub.publish(Float64(data=float(scoop)))
        self.finger_pub.publish(Float64(data=float(self.clamp_finger(self.finger_cmd))))

    def publish_state(self):
        self.state_pub.publish(String(data=self.mode))
        self.lift_state_pub.publish(String(data=self.lift_name))
        self.scoop_state_pub.publish(String(data=self.scoop))
        self.published_fingers = self.finger_state()
        self.finger_state_pub.publish(String(data=self.published_fingers))


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
