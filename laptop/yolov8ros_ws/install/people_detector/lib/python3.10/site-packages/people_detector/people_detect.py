from __future__ import print_function
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
import torch
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')

        package_share_directory = get_package_share_directory('people_detector')
        self.br = CvBridge()

        path = "/yolov8n.pt"
        full_path = package_share_directory + path

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"🔥 YOLO using device: {self.device}")

        self.model = YOLO(full_path)

        self.declare_parameter("conf_threshold", 0.35)
        self.conf_threshold = self.get_parameter("conf_threshold").get_parameter_value().double_value
        print(f"🔍 Confidence Threshold: {self.conf_threshold}")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            'video_frames/compressed',
            self.listener_callback,
            qos
        )

        self.publisher_ = self.create_publisher(
            Float32MultiArray,
            'person_coordinates',
            10
        )

        self.frame_count = 0
        self.last_annotated = None

    def listener_callback(self, data):
        frame = self.br.compressed_imgmsg_to_cv2(data)
        self.frame_count += 1

        if self.frame_count % 2 == 0:
            start = time.time()
            results = self.model.predict(
                source=frame,
                device=self.device,
                stream=True,
                conf=self.conf_threshold,
                classes=[0]  # Solo personas
            )
            for r in results:
                boxes = r.boxes.xyxy.cpu().numpy()
                if len(boxes) > 0:
                    # Tomamos el primer bounding box
                    x1, y1, x2, y2 = boxes[0]
                    x_center = (x1 + x2) / 2
                    y_center = (y1 + y2) / 2

                    # Publicar coordenadas
                    msg = Float32MultiArray()
                    msg.data = [float(x_center), float(y_center)]
                    self.publisher_.publish(msg)
                    print(f"👤 Persona detectada en: x={x_center:.1f}, y={y_center:.1f}")

                self.last_annotated = r.plot()
                break
            end = time.time()
            print(f"Inference time: {end-start:.3f} s")

        if self.last_annotated is not None:
            cv2.imshow("YOLOv8 Detection (GPU)", self.last_annotated)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    video_subscriber = VideoSubscriber()
    rclpy.spin(video_subscriber)
    video_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
