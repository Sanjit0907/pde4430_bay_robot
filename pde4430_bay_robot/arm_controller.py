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
    /finger_state       std_msgs/String   current finger setting (latched)

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
# Fingers (angle to their jaw): folded along it / hooked at 90 deg / open, straight on.
# 'hook' asks for less than 90 deg, past where the fingers meet the large sphere, so
# they keep pressing it with the full 3 N m. 'fold' asks a little past 0 so they rest
# on the joint stop.
FINGERS = {'fold': -0.05, 'hook': math.radians(70.0), 'open': math.pi}

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
        self.fingers = 'fold'
        self.deploy_timer = None
        self.deployed = False
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)

        # Re-send targets at 5 Hz: Gazebo only listens once the bridge is up, so a single
        # message at start-up could be lost. This timer runs on the computer's steady
        # clock: a simulation-time timer created before /clock exists never fires.
        self.create_timer(0.2, self.send_targets, clock=Clock(clock_type=ClockType.STEADY_TIME))
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
        self.fingers = name
        self.get_logger().info(f'Fingers -> {name}')
        self.publish_state()
        self.send_targets()

    # ------------------------------------------------------------------ output
    def send_targets(self):
        """Publish every target (the URDF mirrors the jaws and the fingers)."""
        jaw = Float64(data=float(self.target))
        self.left_pub.publish(jaw)
        self.right_pub.publish(jaw)
        self.lift_pub.publish(Float64(data=float(self.lift_target)))
        scoop = min(max(SCOOP[self.scoop], SCOOP_LIMIT[0]), SCOOP_LIMIT[1])
        self.scoop_pub.publish(Float64(data=float(scoop)))
        finger = min(max(FINGERS[self.fingers], FINGER_LIMIT[0]), FINGER_LIMIT[1])
        self.finger_pub.publish(Float64(data=float(finger)))

    def publish_state(self):
        self.state_pub.publish(String(data=self.mode))
        self.lift_state_pub.publish(String(data=self.lift_name))
        self.scoop_state_pub.publish(String(data=self.scoop))
        self.finger_state_pub.publish(String(data=self.fingers))


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
