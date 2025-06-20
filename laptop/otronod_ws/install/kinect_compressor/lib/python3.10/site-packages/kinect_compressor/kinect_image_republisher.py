import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time

class KinectImageRepublisher(Node):
    def __init__(self):
        super().__init__('kinect_image_republisher')

        # Tamaño fijo de la imagen (ajusta a lo que necesites)
        self.target_width = 360    # <---- Cambia aquí el ancho deseado
        self.target_height = 280   # <---- Cambia aquí el alto deseado

        # FPS máximo (ajusta aquí)
        self.max_fps = 100           # <---- Cambia aquí los FPS máximos que deseas
        self.min_interval = 1.0 / self.max_fps
        self.last_pub_time = 0

        # QoS para imágenes rápidas tipo streaming
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image,
            '/kinect/image_raw',
            self.listener_callback,
            qos
        )

        self.publisher = self.create_publisher(
            CompressedImage,
            '/video_frames/compressed',
            qos
        )

        self.bridge = CvBridge()
        self.get_logger().info('✅ Nodo KinectImageRepublisher iniciado.')

    def listener_callback(self, msg):
        try:
            # Limita la frecuencia de publicación (FPS)
            current_time = time.time()
            if (current_time - self.last_pub_time) < self.min_interval:
                return  # Salir sin publicar si no ha pasado el tiempo suficiente
            self.last_pub_time = current_time

            # Convierte a imagen de color (BGR)
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Redimensiona la imagen a tamaño fijo
            resized_image = cv2.resize(
                color_image,
                (self.target_width, self.target_height),
                interpolation=cv2.INTER_AREA
            )

            # Comprime a JPEG
            success, encoded_image = cv2.imencode('.jpg', resized_image, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            if not success:
                self.get_logger().warning("⚠️ Fallo al comprimir imagen.")
                return

            # Publica como CompressedImage
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = 'jpeg'
            compressed_msg.data = encoded_image.tobytes()
            self.publisher.publish(compressed_msg)

            self.get_logger().info(
                f'📤 Imagen RGB publicada ({self.target_width}x{self.target_height}) a {self.max_fps} FPS máx.'
            )

        except Exception as e:
            self.get_logger().error(f'❌ Error al procesar imagen: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = KinectImageRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
