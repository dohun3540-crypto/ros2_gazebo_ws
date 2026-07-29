import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import math

class ParkingController(Node):
    def __init__(self):
        super().__init__('parking_controller')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.state = 'SEARCH'
        self.escape_timer = 0.0
        self.is_danger = False

        self.PARKING_CENTER_X = 6.15
        self.TARGET_Y = -3.2
        self.SAFE_MARGIN = 0.45

    def euler_from_quaternion(self, quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        is_danger = False
        for i, r in enumerate(ranges):
            if math.isinf(r) or math.isnan(r) or r > 3.0:
                continue
            
            angle = angle_min + i * angle_increment
            if abs(angle) > math.pi * 0.75:
                continue

            if r < self.SAFE_MARGIN:
                is_danger = True
                break

        self.is_danger = is_danger

    def timer_callback(self):
        msg = Twist()
        
        if self.is_danger and self.state != 'FINISHED':
            self.escape_timer = 0.5

        if self.escape_timer > 0 and self.state != 'FINISHED':
            self.escape_timer -= 0.05
            msg.linear.x = 0.5
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.get_logger().warn('Collision Danger! Stopping & Forwarding.')
            
        else:
            if self.state == 'SEARCH':
                error_x = self.PARKING_CENTER_X - self.x
                msg.linear.x = float(min(1.0, max(-0.5, 1.5 * error_x)))
                msg.linear.y = 0.0
                msg.angular.z = float(-self.yaw * 1.5)
                
                if abs(error_x) < 0.05:
                    self.state = 'ZERO_TURN'
                    self.get_logger().info('State: ZERO_TURN')

            elif self.state == 'ZERO_TURN':
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                
                target_theta = math.pi / 2
                error_theta = target_theta - self.yaw
                msg.angular.z = float(min(1.0, max(-1.0, 2.0 * error_theta)))

                if abs(error_theta) < 0.05:
                    self.state = 'STRAIGHT_REVERSE'
                    self.get_logger().info('State: STRAIGHT_REVERSE')

            elif self.state == 'STRAIGHT_REVERSE':
                error_y = self.y - self.TARGET_Y
                msg.linear.x = -float(min(0.8, max(0.1, 1.0 * error_y)))
                msg.linear.y = 0.0
                
                error_x = self.PARKING_CENTER_X - self.x
                target_theta = (math.pi / 2) - (error_x * 0.8)
                error_theta = target_theta - self.yaw
                msg.angular.z = float(1.5 * error_theta)

                if error_y < 0.05:
                    self.state = 'FINISHED'
                    self.get_logger().info('PARKING FINISHED!')

            else:
                msg.linear.x = 0.0
                msg.linear.y = 0.0
                msg.angular.z = 0.0

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ParkingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
