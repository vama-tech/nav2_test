import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # The message type for /odom topic

class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg):
        # Extract position and orientation from the Odometry message
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        orientation_x = msg.pose.pose.orientation.x
        orientation_y = msg.pose.pose.orientation.y
        orientation_z = msg.pose.pose.orientation.z
        orientation_w = msg.pose.pose.orientation.w

        # Log or print the extracted values
        self.get_logger().info(f'Position: x={position_x}, y={position_y}')
        self.get_logger().info(f'Orientation: x={orientation_x}, y={orientation_y}, z={orientation_z}, w={orientation_w}')

def main(args=None):
    rclpy.init(args=args)
    odom_logger = OdomLogger()
    rclpy.spin(odom_logger)
    odom_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
