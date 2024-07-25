
import rclpy
import math
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import std_msgs 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class TTCNode(Node):
    def start(self):
        drive = AckermannDrive(steering_angle=0.0, speed=3.0)
        data = AckermannDriveStamped(header=std_msgs.msg.Header(), drive=drive)
        
       # start_vel = AckermannDriveStamped()
       # start_vel.drive.speed = 0.5
       # start_vel.drive.steering_angle = 0.0
        self.ackermann_publisher.publish(data)
        self.get_logger().info('Publishing commands to /drive topic')    

    def __init__(self):
        super().__init__('ttc_node')
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar, 10)
        self.subscription
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.odom_subscription
        self.ackermann_publisher = self.create_publisher(AckermannDriveStamped, '/drive', qos_profile_sensor_data)
        
        self.current_velocity = 0
        self.ttc_threshold = 1.0
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.move_callback)
        self.min_distance = 10.0
        self.ttc = 10.0
        self.isStopped = False 
        
        
    def move_callback(self):
    	if (self.ttc < self.ttc_threshold) or not self.isStopped:
    		self.isStopped = True
    		self.stop_car()
    	else:
    		self.start()
    		
    def lidar(self, msg):
        self.min_distance = min(msg.ranges)
        self.get_logger().info(f'Min dist {self.min_distance}')
        if self.current_velocity > 0:
            self.ttc = self.min_distance / self.current_velocity
    
    def odom(self, msg):
        x_velocity = msg.twist.twist.linear.x
        y_velocity = msg.twist.twist.linear.y
        self.current_velocity = math.sqrt(x_velocity**2 + y_velocity**2)

    def stop_car(self):
        drive = AckermannDrive(steering_angle=0.0, speed=0.0)
        data = AckermannDriveStamped(header=std_msgs.msg.Header(), drive=drive)
        
       # start_vel = AckermannDriveStamped()
       # start_vel.drive.speed = 0.5
       # start_vel.drive.steering_angle = 0.0
        self.get_logger().info('STOP!!!!!!!!!!!')  
        # stop_msg = AckermannDriveStamped()
        # stop_msg.drive.speed = 0.0
        # stop_msg.drive.steering_angle = 0.0
        self.ackermann_publisher.publish(data)
        self.get_logger().info('TTC below treshold')

def main(args=None):
    rclpy.init(args=args)
    node = TTCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()
