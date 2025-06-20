import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class WallAvoider(Node):
    def __init__(self):
        super().__init__('wall_avoider')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def scan_callback(self, scan):
        ranges = np.array(scan.ranges)
        center = len(ranges) // 2
        window = 20  # Ángulo de apertura frontal: puedes ajustar este valor
        front = ranges[center - window : center + window]
        valid_front = front[~np.isnan(front)]

        cmd = Twist()
        if valid_front.size == 0:
            # Si no hay lecturas válidas (todo es NaN), gira
            cmd.angular.z = 0.1
            self.get_logger().info("Frente vacío (NaN), girando para buscar salida")
        else:
            min_dist = valid_front.min()
            if min_dist < 0.55:
                cmd.angular.z = 0.1
                self.get_logger().info(f"Obstáculo cerca ({min_dist:.2f} m), girando")
            else:
                cmd.linear.x = 0.05
                self.get_logger().info("Camino libre, avanzando")
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = WallAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
