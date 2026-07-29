import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanCheck(Node):
    def __init__(self):
        super().__init__('scan_check')
        self.create_subscription(LaserScan, '/scan', self.cb, 10)

    def cb(self, msg):
        finite = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        print('total:', len(msg.ranges), 'finite(장애물 감지):', len(finite))
        if finite:
            print('min:', min(finite), 'max:', max(finite))
        rclpy.shutdown()


def main():
    rclpy.init()
    rclpy.spin(ScanCheck())


if __name__ == '__main__':
    main()
