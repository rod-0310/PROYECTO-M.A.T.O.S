import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class PersonRescue(Node):
    def __init__(self):
        super().__init__('person_rescue')

        self.declare_parameter("image_width", 300)
        self.image_width = self.get_parameter("image_width").get_parameter_value().integer_value

        self.Kp = 0.005
        self.Ki = 0.0
        self.Kd = 0.009
        self.error_integral = 0.0
        self.error_prev = 0.2

        self.person_x = None
        self.person_detected = False
        self.min_dist_front = float('inf')
        self.nan_front = False

        self.create_subscription(Float32MultiArray, 'person_coordinates', self.person_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.drop_pub = self.create_publisher(Bool, 'drop_cmd', 10)

        self.get_logger().info('✅ Nodo PersonRescue inicializado')

    def person_callback(self, msg):
        if len(msg.data) < 1:
            self.person_detected = False
            return
        self.person_x = msg.data[0]
        self.person_detected = True

    def scan_callback(self, scan):
        # --- Calcular distancia frontal y checar NaN ---
        ranges = np.array(scan.ranges)
        center = len(ranges)//2
        window = 20
        front = ranges[center-window:center+window]
        valid_front = front[~np.isnan(front)]
        self.nan_front = valid_front.size == 0
        self.min_dist_front = valid_front.min() if valid_front.size else float('inf')

        cmd = Twist()
        # --- MODO 1: Persona detectada ---
        if self.person_detected and self.person_x is not None:
            center_x = (self.image_width / 2) + 5
            error = center_x - self.person_x

            # PID
            self.error_integral += error
            derivative = error - self.error_prev
            z_angular = (
                self.Kp * error +
                self.Ki * self.error_integral +
                self.Kd * derivative
            )
            self.error_prev = error

            if self.nan_front:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.7
                self.get_logger().info("👤 Persona detectada, pero sensor frontal está ciego (NaN), girando para buscar camino.")
                self.drop_pub.publish(Bool(data=False))
            elif self.min_dist_front < 0.6:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info("👤 Persona cerca, ¡deteniendo para entregar caja!")
                self.drop_pub.publish(Bool(data=True))
            else:
                cmd.linear.x = 0.18  # Avanza mientras se centra
                cmd.angular.z = float(z_angular)
                self.get_logger().info(
                    f"Center X: {center_x:.1f}, Person X: {self.person_x:.1f}, Error: {error:.1f}, Z cmd: {z_angular:.4f}, Distancia frontal: {self.min_dist_front:.2f} m"
                )
                self.drop_pub.publish(Bool(data=False))
        # --- MODO 2: Patrulla (no hay persona detectada) ---
        else:
            if self.nan_front:
                cmd.angular.z = 0.7
                self.get_logger().info("Patrulla: Frente ciego (NaN), girando")
            elif self.min_dist_front < 0.5:
                cmd.angular.z = 0.7
                self.get_logger().info("Patrulla: Obstáculo cerca, girando")
            else:
                cmd.linear.x = 0.18
                self.get_logger().info("Patrulla: Camino libre, avanzando")
            self.drop_pub.publish(Bool(data=False))

        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PersonRescue()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
