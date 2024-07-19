import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class TTCNode(Node):
    def __init__(self):i
        super().__init__('ttc_node')
        self.subscription = self.create_substriction(LaserScan, '/scan', self.lidar_callback, 10)
        self.subscription
        self.velocity_subscription = self.create_sunscription(Odometry, '/odom', self.odom_callback, 10)
        self.odom_subscription
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/ackermann_cmd', 10)

        self.current_velocity = 0
        self.ttc_threshold = 1.0

    def lidar(self, msg):
        min_distance = min(msg.ranges)
        if self.current_velocity > 0:
            ttc = min_distance / self.current_velocity
            self.get_logger().info(f'TTC: {tcc:.2f} seconds')

            if ttc < self.ttc_threshold:
                self.stop_car()
        else:
            self.get_logger().info("Velocity is zero")


    def odom(self, msg):
        x_velocity = msg.twist.twist.linear.x
        y_velocity = msg.twist.twist.linear.y
        self.current_velocity = math.sqrt(x_velocity**2 + y_velocity**2)

    def stop_car(self):
        stop_msg = AckermannDriveStamped()
        strop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0
        self.ackermann_publisher.publish(stop_msg)
        self.get_logger().info('TTC below treshold')

def main(args=None):
    rclpy.init(args=args)
    node = TTCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()
