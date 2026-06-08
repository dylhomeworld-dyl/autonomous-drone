import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped


class TakeoffNode(Node):

    def __init__(self):
        super().__init__('takeoff_node')

        # Current drone state, updated by the subscriber
        self.current_state = State()

        # Subscriber - listens to drone state from PX4
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10)

        # Publisher - sends position setpoints to PX4
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10)

        # Service clients for arming and mode changes
        self.arming_client = self.create_client(
            CommandBool,
            '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(
            SetMode,
            '/mavros/set_mode')

        # Target position - hover at 2 meters above start
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 2.0

        # Timer fires the control loop at 20Hz (every 0.05s)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Counters/flags for sequencing
        self.frame_count = 0
        self.armed_time = None

        self.get_logger().info('Takeoff node initialized')

    def state_callback(self, msg):
        # Store the latest state PX4 reports
        self.current_state = msg

    def timer_callback(self):
        # Always stamp and publish the setpoint - this stream must never stop
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose.header.frame_id = 'map'
        self.setpoint_pub.publish(self.target_pose)

        self.frame_count += 1
        elapsed = self.frame_count * 0.05  # 20Hz -> seconds

        # Step 1: after ~2s of setpoints, request OFFBOARD. Retry every 2s.
        if elapsed > 2.0 and self.current_state.mode != 'OFFBOARD':
            if self.frame_count % 40 == 0:
                self.set_offboard_mode()
            return

        # Step 2: once OFFBOARD is confirmed, arm. Retry every 2s.
        if self.current_state.mode == 'OFFBOARD' and not self.current_state.armed:
            if self.frame_count % 40 == 0:
                self.arm_drone()
            return

        # Step 3: armed and in OFFBOARD - climbing/holding. Land after 20s.
        if self.current_state.armed:
            if self.armed_time is None:
                self.armed_time = elapsed
                self.get_logger().info('Airborne - holding at 2m')
            if elapsed - self.armed_time > 20.0:
                if self.target_pose.pose.position.z != 0.0:
                    self.get_logger().info('Landing...')
                self.target_pose.pose.position.z = 0.0

    def set_offboard_mode(self):
        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'
        self.set_mode_client.call_async(req)
        self.get_logger().info('Requesting OFFBOARD mode')

    def arm_drone(self):
        req = CommandBool.Request()
        req.value = True
        self.arming_client.call_async(req)
        self.get_logger().info('Requesting arm')


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()