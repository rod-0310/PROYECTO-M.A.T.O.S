import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from tf2_ros import TransformBroadcaster
import matplotlib.pyplot as plt
import numpy as np
import math
import threading
import time

class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.07   # Velocidad lineal (ajusta si quieres más rápido)
        self.w = 0.25   # Velocidad angular (ajusta si quieres más giro)
        self.timer = self.create_timer(0.05, self.publish_odom)

    def publish_odom(self):
        self.theta += self.w
        self.x += self.v * math.cos(self.theta)
        self.y += self.v * math.sin(self.theta)

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "camera_depth_frame"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = Quaternion()
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation = q
        odom.twist.twist = Twist()
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "camera_depth_frame"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

class MapViewer(Node):
    def __init__(self):
        super().__init__('map_viewer')
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.callback, 1)
        self.fig, self.ax = plt.subplots()
        self.img = None
        self.first = True

    def callback(self, msg):
        if msg.info.width == 0 or msg.info.height == 0:
            print("Mapa vacío.")
            return
        data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
        data = np.flipud(data)
        if self.img is None:
            self.img = self.ax.imshow(data, cmap='gray', vmin=-1, vmax=100)
            plt.title("/map (OccupancyGrid)")
            plt.tight_layout()
        else:
            self.img.set_data(data)
        if self.first:
            plt.show(block=False)
            self.first = False
        plt.pause(0.001)

def start_fake_odom():
    rclpy.init(args=None)
    fake_odom = FakeOdomPublisher()
    rclpy.spin(fake_odom)
    fake_odom.destroy_node()
    rclpy.shutdown()

def start_map_viewer():
    time.sleep(1)  # Da tiempo a que fake_odom arranque
    rclpy.init(args=None)
    map_viewer = MapViewer()
    try:
        while rclpy.ok():
            rclpy.spin_once(map_viewer, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    map_viewer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    # Inicia fake odom en un hilo aparte
    threading.Thread(target=start_fake_odom, daemon=True).start()
    start_map_viewer()
