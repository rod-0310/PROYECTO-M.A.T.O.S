# Import the necessary libraries
import rclpy  # Python Client Library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library


class VideoPublisher(Node):
    """
    Create a VideoPublisher class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('video_publisher')

        # Create the publisher: publishes an Image to 'video_frames'
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)

        # Publish a message every 0.1 seconds (~10 FPS)
        timer_period = 0.1  # seconds

        # Use the laptop's webcam instead of a file
        self.cap = cv2.VideoCapture(0)  # 0 is the default webcam

        if not self.cap.isOpened():
            self.get_logger().error('❌ No se pudo abrir la cámara de la laptop.')
            exit(1)
        else:
            self.get_logger().info('✅ Cámara abierta correctamente.')

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        ret, frame = self.cap.read()

        if ret:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
            self.get_logger().info('📸 Publicando frame de la cámara.')
        else:
            self.get_logger().warning('⚠️ No se pudo leer un frame de la cámara.')


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    video_publisher = VideoPublisher()

    # Spin the node so the callback function is called.
    rclpy.spin(video_publisher)

    # Destroy the node explicitly
    video_publisher.destroy_node()  # <-- CORREGIDO

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
