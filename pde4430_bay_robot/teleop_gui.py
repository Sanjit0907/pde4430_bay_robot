"""teleop_gui.py - window for driving the robot and working the gripper by hand.

    ros2 run pde4430_bay_robot teleop_gui

Drive: hold a direction button, or hold W/A/S/D or the arrow keys (two keys together
for a curve). Releasing stops the robot. Space or the Stop button stops it at once.
The window also stops the robot when it loses focus, so a robot never keeps driving
because a key release went to another window.

Subscribes
    /arm_state    std_msgs/String   current jaw mode (latched, from arm_controller)
    /lift_state   std_msgs/String   current lift setting (latched, from arm_controller)
Publishes
    /cmd_vel      geometry_msgs/Twist  drive command, 10 Hz while driving
    /arm_mode     std_msgs/String      jaw mode: fold, wide, large, straight, close, grip
    /lift_mode    std_msgs/String      lift: down, carry, up - or a height in metres

The jaw and lift buttons only send requests. arm_controller applies the safety rules
(e.g. it refuses to open the jaws while lifted), so the mode shown in the window is
the one it reports back on /arm_state and /lift_state, not the button last pressed.

Parameters
    max_linear (float, default 0.5)   top of the speed slider, m/s (URDF drive limit)
    max_angular (float, default 1.5)  top of the turn slider, rad/s (URDF drive limit)
"""
import time
import tkinter as tk

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

JAW_MODES = ('fold', 'wide', 'large', 'straight', 'close', 'grip')
LIFT_MODES = ('down', 'carry', 'up')
LIFT_MAX = 0.15                      # m, lift joint limit

# Direction buttons: (row, column, label, forward, turn left). 0, 0 = stop.
DRIVE_BUTTONS = [
    (0, 0, '↖', 1, 1), (0, 1, '↑', 1, 0), (0, 2, '↗', 1, -1),
    (1, 0, '↶', 0, 1), (1, 1, 'Stop', 0, 0), (1, 2, '↷', 0, -1),
    (2, 0, '↙', -1, -1), (2, 1, '↓', -1, 0), (2, 2, '↘', -1, 1),
]
# Keys: (forward, turn left) each key adds while held.
KEYS = {
    'w': (1, 0), 'Up': (1, 0), 's': (-1, 0), 'Down': (-1, 0),
    'a': (0, 1), 'Left': (0, 1), 'd': (0, -1), 'Right': (0, -1),
}

TICK_MS = 100                        # 10 Hz drive commands
STOP_REPEATS = 3                     # zero commands sent after stopping, then quiet
KEY_RELEASE_MS = 60                  # ignore X11 key auto-repeat release/press pairs

HIGHLIGHT = '#2e7d32'
NORMAL = '#e0e0e0'


class TeleopGui(Node):
    """ROS side: publishers, subscribers and the current drive request."""

    def __init__(self):
        super().__init__('teleop_gui')
        self.max_linear = self.declare_parameter('max_linear', 0.5).value
        self.max_angular = self.declare_parameter('max_angular', 1.5).value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(String, '/arm_mode', 10)
        self.lift_pub = self.create_publisher(String, '/lift_mode', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.arm_state, self.lift_state = '?', '?'
        self.create_subscription(String, '/arm_state', self.on_arm_state, latched)
        self.create_subscription(String, '/lift_state', self.on_lift_state, latched)

    def on_arm_state(self, msg):
        self.arm_state = msg.data

    def on_lift_state(self, msg):
        self.lift_state = msg.data

    def drive(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def send_jaws(self, mode):
        self.arm_pub.publish(String(data=mode))

    def send_lift(self, request):
        self.lift_pub.publish(String(data=request))


class Window:
    """Tk side: buttons, sliders and keys. ROS is serviced from the Tk timer."""

    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Bay robot teleop')
        self.root.resizable(False, False)

        self.button_dir = (0, 0)            # direction from a held drive button
        self.keys_down = set()              # drive keys currently held
        self.key_release_time = {}
        self.stop_repeats = 0

        self.build_drive()
        self.build_speed()
        self.build_jaws()
        self.build_lift()
        self.status = tk.Label(self.root, anchor='w', font=('TkFixedFont', 10))
        self.status.grid(row=2, column=0, columnspan=2, sticky='we', padx=8, pady=(0, 8))

        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<KeyRelease>', self.on_key_release)
        self.root.bind('<space>', lambda _e: self.stop())
        self.root.bind('<FocusOut>', self.on_focus_out)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(TICK_MS, self.tick)

    # ------------------------------------------------------------------ layout
    def build_drive(self):
        frame = tk.LabelFrame(self.root, text='Drive (hold, or W A S D / arrows; Space = stop)')
        frame.grid(row=0, column=0, sticky='nsew', padx=8, pady=8)
        for row, col, label, fwd, left in DRIVE_BUTTONS:
            b = tk.Button(frame, text=label, width=5, height=2, font=('TkDefaultFont', 14))
            b.grid(row=row, column=col, padx=2, pady=2)
            if (fwd, left) == (0, 0):
                b.configure(bg='#c62828', fg='white', activebackground='#e53935',
                            command=self.stop)
            else:
                b.bind('<ButtonPress-1>', lambda _e, d=(fwd, left): self.press_button(d))
                b.bind('<ButtonRelease-1>', lambda _e: self.press_button((0, 0)))

    def build_speed(self):
        frame = tk.LabelFrame(self.root, text='Speed limit')
        frame.grid(row=1, column=0, sticky='nsew', padx=8, pady=(0, 8))
        self.linear = tk.Scale(frame, from_=0.05, to=self.node.max_linear, resolution=0.05,
                               orient='horizontal', length=220, label='Drive (m/s)')
        self.linear.set(min(0.25, self.node.max_linear))
        self.linear.pack(fill='x', padx=4)
        self.angular = tk.Scale(frame, from_=0.1, to=self.node.max_angular, resolution=0.1,
                                orient='horizontal', length=220, label='Turn (rad/s)')
        self.angular.set(min(0.8, self.node.max_angular))
        self.angular.pack(fill='x', padx=4)

    def build_jaws(self):
        frame = tk.LabelFrame(self.root, text='Jaws')
        frame.grid(row=0, column=1, sticky='nsew', padx=(0, 8), pady=8)
        self.jaw_buttons = {}
        for i, mode in enumerate(JAW_MODES):
            b = tk.Button(frame, text=mode, width=9, bg=NORMAL,
                          command=lambda m=mode: self.node.send_jaws(m))
            b.grid(row=i // 2, column=i % 2, padx=2, pady=2)
            self.jaw_buttons[mode] = b

    def build_lift(self):
        frame = tk.LabelFrame(self.root, text='Lift')
        frame.grid(row=1, column=1, sticky='nsew', padx=(0, 8), pady=(0, 8))
        self.lift_buttons = {}
        for i, mode in enumerate(LIFT_MODES):
            b = tk.Button(frame, text=mode, width=6, bg=NORMAL,
                          command=lambda m=mode: self.node.send_lift(m))
            b.grid(row=0, column=i, padx=2, pady=2)
            self.lift_buttons[mode] = b
        self.height = tk.Scale(frame, from_=0.0, to=LIFT_MAX, resolution=0.01,
                               orient='horizontal', label='Height (m)')
        self.height.grid(row=1, column=0, columnspan=2, sticky='we', padx=2)
        tk.Button(frame, text='Set', width=6,
                  command=lambda: self.node.send_lift(f'{self.height.get():.2f}')
                  ).grid(row=1, column=2, padx=2, sticky='s')

    # ------------------------------------------------------------------ input
    def press_button(self, direction):
        self.button_dir = direction

    def on_key_press(self, event):
        key = event.keysym if event.keysym in KEYS else event.keysym.lower()
        if key in KEYS:
            self.keys_down.add(key)
            self.key_release_time.pop(key, None)

    def on_key_release(self, event):
        key = event.keysym if event.keysym in KEYS else event.keysym.lower()
        if key in KEYS:
            # Holding a key makes X11 send release+press pairs. Only treat it as
            # released if no new press arrives shortly after.
            stamp = time.monotonic()
            self.key_release_time[key] = stamp
            self.root.after(KEY_RELEASE_MS, lambda: self.release_key(key, stamp))

    def release_key(self, key, stamp):
        if self.key_release_time.get(key) == stamp:
            self.keys_down.discard(key)

    def on_focus_out(self, _event):
        # Focus also moves between widgets inside the window (e.g. clicking a
        # slider). Stop only when it has left the window altogether.
        self.root.after(20, self.stop_if_unfocused)

    def stop_if_unfocused(self):
        try:
            focused = self.root.focus_get()
        except (KeyError, tk.TclError):
            focused = None
        if focused is None:
            self.stop()

    def stop(self):
        self.button_dir = (0, 0)
        self.keys_down.clear()
        self.node.drive(0.0, 0.0)
        self.stop_repeats = STOP_REPEATS

    # ------------------------------------------------------------------ loop
    def direction(self):
        key_fwd = sum(KEYS[k][0] for k in self.keys_down)
        key_left = sum(KEYS[k][1] for k in self.keys_down)
        if key_fwd < 0:
            # S+A means "back and to the left", like the button: when reversing,
            # the rear swings left if the robot turns clockwise.
            key_left = -key_left
        fwd = self.button_dir[0] + key_fwd
        left = self.button_dir[1] + key_left
        return max(-1, min(1, fwd)), max(-1, min(1, left))

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

        fwd, left = self.direction()
        linear = fwd * self.linear.get()
        angular = left * self.angular.get()
        if fwd or left:
            self.node.drive(linear, angular)
            self.stop_repeats = STOP_REPEATS
        elif self.stop_repeats > 0:          # a few zeros so the stop is not missed,
            self.node.drive(0.0, 0.0)        # then go quiet so other nodes can drive
            self.stop_repeats -= 1

        arm, lift = self.node.arm_state, self.node.lift_state
        for mode, b in self.jaw_buttons.items():
            b.configure(bg=HIGHLIGHT if mode == arm else NORMAL,
                        fg='white' if mode == arm else 'black')
        for mode, b in self.lift_buttons.items():
            b.configure(bg=HIGHLIGHT if mode == lift else NORMAL,
                        fg='white' if mode == lift else 'black')
        self.status.configure(
            text=f'cmd  v {linear:+.2f} m/s  w {angular:+.2f} rad/s   |   '
                 f'jaws: {arm}   lift: {lift}')
        self.root.after(TICK_MS, self.tick)

    def close(self):
        self.stop()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopGui()
    window = Window(node)
    try:
        window.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.drive(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
