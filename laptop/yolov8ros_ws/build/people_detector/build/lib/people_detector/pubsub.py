import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class PersonFollower(Node):
    def __init__(self):
        super().__init__('person_follower')

        # === Parámetros ===
        self.declare_parameter("image_width", 300)  # Ajusta a tu cámara
        self.image_width = self.get_parameter("image_width").get_parameter_value().integer_value

        self.Kp = 0.005   # Aumentado Kp para correcciones más rápidas

        # Para PID extendido:
        self.Ki = 0.000
        self.Kd = 0.009
        self.error_integral = 0.0
        self.error_prev = 0.2

        # Suscripción a coordenadas de la persona
        self.create_subscription(
            Float32MultiArray,
            'person_coordinates',
            self.person_callback,
            10
        )

        # Publicador de cmd_vel
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.get_logger().info('✅ Nodo PersonFollower inicializado')

    def person_callback(self, msg):
        if len(msg.data) < 1:
            return
    
        x = msg.data[0]
        center_x = (self.image_width / 2) + 5  # Ajuste del centro (puedes cambiar el valor del offset)

        # Error en píxeles (desviación horizontal)
        error = center_x - x

        # === PID ===
        self.error_integral += error
        derivative = error - self.error_prev

        z_angular = (
            self.Kp * error +
            self.Ki * self.error_integral +
            self.Kd * derivative
        )

        self.error_prev = error

        # Construir mensaje Twist
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(z_angular)  # El signo puede invertirse según tu robot

        self.cmd_vel_pub.publish(twist)

        # Mostrar log de los valores calculados
        self.get_logger().info(
            f"Center X: {center_x:.1f}, Person X: {x:.1f}, Error: {error:.1f}, Z cmd: {z_angular:.4f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
