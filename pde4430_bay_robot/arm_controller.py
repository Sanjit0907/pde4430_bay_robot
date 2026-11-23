"""arm_controller.py - turns named arm modes into joint angles for the two fold-out arms.

Subscribes
    /arm_mode           std_msgs/String   one of: fold, wide, large, close, straight
Publishes
    /left_arm/cmd_pos   std_msgs/Float64  left arm joint target (rad), bridged to Gazebo
    /right_arm/cmd_pos  std_msgs/Float64  right arm joint target (rad), bridged to Gazebo
    /arm_state          std_msgs/String   the mode currently commanded (latched)

Joint angle convention (see urdf/bay_robot.urdf): 0 rad = folded back along the body,
pi rad = pointing straight ahead; the arm direction is (180 deg - angle) outward.

Parameters
    deploy_mode (str, default "wide")  mode to move to automatically after start-up
    deploy_delay (float, default 3.0)  seconds to wait before deploying (lets the
                                       robot settle on the ground after spawning)
    mode_angles.<name> (float, rad)    override the angle of any mode
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64, String

# Default mode angles in radians. Derived from the object sizes in the
# assessment_bay world (see README, "Arm modes").
DEFAULT_MODES = {
    'fold': 0.0,                       # spawn pose, inside the 0.5 m x 0.5 m footprint
    'wide': math.radians(120.0),       # 60 deg outward: wide funnel to catch objects
    'large': math.radians(145.0),      # 35 deg outward: V that cradles the 0.6 m sphere
                                       # (1 cm play), keeping it centred while pushed
    'straight': math.pi,               # both arms straight ahead (parallel)
    'close': math.radians(202.0),      # 22 deg inward: cage for the 0.2 m objects
}
JOINT_LIMIT = (0.0, 3.6)               # must match <limit> in the URDF


class ArmController(Node):
    """Maps arm mode names to joint targets and keeps re-sending them."""

    def __init__(self):
        super().__init__('arm_controller')

        # ---- parameters ----
        self.declare_parameter('deploy_mode', 'wide')
        self.declare_parameter('deploy_delay', 3.0)
        self.modes = {}
        for name, angle in DEFAULT_MODES.items():
            self.declare_parameter(f'mode_angles.{name}', angle)
            self.modes[name] = self.get_parameter(f'mode_angles.{name}').value

        # ---- publishers / subscribers ----
        self.left_pub = self.create_publisher(Float64, '/left_arm/cmd_pos', 10)
        self.right_pub = self.create_publisher(Float64, '/right_arm/cmd_pos', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/arm_state', latched)
        self.create_subscription(String, '/arm_mode', self.on_mode, 10)

        # Start folded; deploy after a short delay.
        self.mode = 'fold'
        self.target = self.modes['fold']
        self.deploy_timer = self.create_timer(
            self.get_parameter('deploy_delay').value, self.deploy_once)

        # Re-send the target at 5 Hz: Gazebo only listens once the bridge is up,
        # so a single message at start-up could be lost.
        self.create_timer(0.2, self.send_target)
        self.publish_state()
        self.get_logger().info(
            f'Arm controller ready. Modes: {", ".join(self.modes)}  '
            '(e.g. ros2 topic pub --once /arm_mode std_msgs/String "{data: large}")')

    def deploy_once(self):
        """Move from the folded spawn pose to the deploy mode, once."""
        self.deploy_timer.cancel()
        deploy = self.get_parameter('deploy_mode').value
        if deploy and deploy != 'fold':
            self.set_mode(deploy)

    def on_mode(self, msg):
        """Handle a mode request on /arm_mode."""
        self.set_mode(msg.data.strip().lower())

    def set_mode(self, name):
        """Validate a mode name and make it the new target."""
        if name not in self.modes:
            self.get_logger().warn(
                f'Unknown arm mode "{name}". Valid: {", ".join(self.modes)}')
            return
        angle = self.modes[name]
        # Safety: never command outside the joint limits, even if a parameter is wrong.
        clamped = min(max(angle, JOINT_LIMIT[0]), JOINT_LIMIT[1])
        if clamped != angle:
            self.get_logger().warn(f'Mode "{name}" angle {angle:.3f} clamped to {clamped:.3f}')
        self.mode, self.target = name, clamped
        self.get_logger().info(f'Arms -> {name} ({math.degrees(clamped):.0f} deg)')
        self.publish_state()
        self.send_target()

    def send_target(self):
        """Publish the same target to both arms (the URDF mirrors the joints)."""
        msg = Float64(data=float(self.target))
        self.left_pub.publish(msg)
        self.right_pub.publish(msg)

    def publish_state(self):
        self.state_pub.publish(String(data=self.mode))


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
