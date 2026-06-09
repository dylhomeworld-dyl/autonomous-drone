import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
import sys
import termios
import tty
import select
import threading


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teolop_node')

        self.current_state = State()

        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        self.set_mode_clitent = self.create_client(
            SetMode, '/mavros/set_mode')
        
        # The target the drone always flies toward
        self.target = PoseStamped()
        self.target.pose.position.x = 0.0
        self.target.pose.position.y = 0.0
        self.target.pose.position.z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.speed = 0.5 # speed of held key

        # state Machine IDLE, TAKEOFF, FLYING, LANDING
        self.mode = 'IDLE'
        self.frame_count = 0
        self.takeoff_frames = 0

        # Keyboard handling
        self.last_key = None
        self.key_lock = threading.Lock()
        self.running = True
        self.kb_thread = threading.Thread(target=self.read_keys, daemon=True)
        self.kb_thread.start()

        self.timer = self.create_timer(0.05,self.control_loop)

        self.print_help()
    
    def print_help(self):
        self.get_logger().info(
            '\n=== DRONE TELEOP ===\n'
            ' t = takeoff   l = land\n'
            ' w/s = fwd/back a/d = left/right r/f = up/down\n'
            ' q = quit\n'
            'Press t to begin.')
    def state_callback(self, msg):
        self.current_state = msg

    def read_keys(self):
        # Runs on a background thread, continuosly reading single keypresses
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self.running:
                # Wait up to 0.1s for a key; is none, loop again
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    with self.key_lock:
                        self.last_key = key
        finally: termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def get_key(self):
        # Grab and clear the most recent key (None is nothing is pressed
        with self.key_lock:
            key = self.last_key
            self.last_key = None
        return key
    
    def set_offboard(self):
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        self.set_mode_slient.call_async(req)

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req)

    def control_loop(self):
        self.frame_count += 1

        # Always publish the current target to keep OFFBOARD alive
        self.target.header.stamp = self.get_clock().now().to_msg()
        self.target.header.frame_id = 'map'
        self.setpoint_pub.publish(self.target)

        key = self.get_key()

       # Handle mode-changing keys
        if key == 'q':
            self.get_logger().info('Quitting.')
            self.running = False
            rclpy.shutdown()
            return
        elif key == 't' and self.mode == 'IDLE':
            self.get_logger().info('Takeoff sequence starting...')
            self.mode = 'TAKEOFF'
            self.takeoff_frames = 0
        elif key == 'l' and self.mode == 'FLYING':
            self.get_logger().info('Landing...')
            self.mode = 'LANDING'

        # Run the active mode
        if self.mode == 'TAKEOFF':
            self.do_takeoff()
        elif self.mode == 'FLYING':
            self.do_flying(key)
        elif self.mode == 'LANDING':
            self.do_landing()

    def do_takeoff(self):
        self.takeoff_frames += 1
        # Keep the climb target at 1.5m
        self.target.pose.position.z = 1.5

        if self.current_state.mode != 'OFFBOARD':
            if self.takeoff_frames % 40 == 0:
                self.set_offboard()
            return
        if not self.current_state.armed:
            if self.takeoff_frames % 40 == 0:
                self.arm()
            return
        # OFFBOARD + armed -> we are flying
        self.get_logger().info('Airborne. You have control.')
        self.mode = 'FLYING'

    def do_flying(self, key):
        # Set velocity based on key, decay if no key
        if key == 'w':
            self.vx = self.speed
        elif key == 's':
            self.vx = -self.speed
        elif key == 'a':
            self.vy = self.speed
        elif key == 'd':
            self.vy = -self.speed
        elif key == 'r':
            self.vz = self.speed
        elif key == 'f':
            self.vz = -self.speed
        else:
            # No movement key this tick -> decay toward zero (coast to stop)
            self.vx *= 0.85
            self.vy *= 0.85
            self.vz *= 0.85

        # Integrate velocity into the target position
        self.target.pose.position.x += self.vx * 0.05
        self.target.pose.position.y += self.vy * 0.05
        self.target.pose.position.z += self.vz * 0.05

        # Don't let it command below ground
        if self.target.pose.position.z < 0.5:
            self.target.pose.position.z = 0.5

    def do_landing(self):
        self.target.pose.position.z = 0.0
        # Once low enough, disarm and go idle
        if self.current_state.armed and self.frame_count % 40 == 0:
            req = CommandBool.Request()
            req.value = False
            self.arming_client.call_async(req)
        if not self.current_state.armed:
            self.get_logger().info('Landed and disarmed.')
            self.mode = 'IDLE'
            self.target.pose.position.x = 0.0
            self.target.pose.position.y = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.running = False
    rclpy.shutdown()


if __name__ == '__main__':
    main() 

       