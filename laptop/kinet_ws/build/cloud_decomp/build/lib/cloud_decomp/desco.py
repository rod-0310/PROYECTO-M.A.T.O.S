import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ByteMultiArray
import zlib
import array
import pickle

class LaserScanCompressor(Node):
    def __init__(self):
        super().__init__('laser_scan_compressor')
        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.pub = self.create_publisher(ByteMultiArray, '/scan/compressed', 10)
        self.get_logger().info('LaserScanCompressor node started, listening to /scan')

    def callback(self, msg):
        try:
            # Serializa el mensaje LaserScan
            raw_bytes = pickle.dumps(msg)
            compressed = zlib.compress(raw_bytes, level=3)
            out = ByteMultiArray()
            out.data = array.array('B', compressed)
            self.pub.publish(out)
        except Exception as e:
            self.get_logger().error(f"Exception in callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanCompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
